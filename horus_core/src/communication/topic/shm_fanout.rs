//! SHM FanoutRing — cross-process contention-free MPMC via mmap'd SPSC matrix.
//!
//! Extends the intra-process [`FanoutRing`](super::fanout::FanoutRing) design to
//! shared memory. Each (publisher, subscriber) pair gets its own SHM-backed SPSC
//! channel with cache-line-separated head/tail atomics.
//!
//! # SHM File Layout
//!
//! A single SHM file per topic (reusing the existing topic SHM region), with the
//! fanout channel matrix appended after the standard TopicHeader:
//!
//! ```text
//! ┌──────────────────────────────────────────┐ offset 0
//! │ TopicHeader (640 bytes)                  │  — standard topic metadata
//! ├──────────────────────────────────────────┤ offset 640
//! │ padding to page boundary                 │
//! ├──────────────────────────────────────────┤ offset 4096 (FANOUT_META_OFFSET)
//! │ FanoutShmMeta (128 bytes, cache-aligned) │  — fanout dimensions & strides
//! ├──────────────────────────────────────────┤ offset 4224
//! │ padding to cache-line boundary           │
//! ├──────────────────────────────────────────┤ offset CHANNELS_BASE
//! │ Channel[0][0]: head_cl(64) + tail_cl(64) │
//! │   + data slots[capacity]                 │
//! ├──────────────────────────────────────────┤
//! │ Channel[0][1]: ...                       │
//! ├──────────────────────────────────────────┤
//! │ ...                                      │
//! │ Channel[max_pubs-1][max_subs-1]: ...     │
//! └──────────────────────────────────────────┘
//! ```
//!
//! # Design Decisions
//!
//! - **Single file**: One SHM file per topic (not per-channel) to match existing
//!   infrastructure. Simpler lifecycle, single mmap, existing cleanup works.
//!
//! - **Pre-allocated matrix**: Allocates for `MAX_FANOUT_ENDPOINTS × MAX_FANOUT_ENDPOINTS`
//!   channels upfront. Memory cost is modest (~164KB for u64 with cap=64, 16×16).
//!
//! - **Cache-line isolation**: Each channel's head and tail live on separate 64-byte
//!   cache lines. No false sharing between producers and consumers.
//!
//! - **POD-only slots (initially)**: Slots store raw T data (no sequence numbers —
//!   SPSC doesn't need them). Serde support can be added later by storing
//!   `[len: u32, serialized_bytes]` in larger slots.
//!
//! - **Process-safe registration**: Publisher/subscriber IDs allocated via atomic
//!   `fetch_add` on counters in the FanoutShmMeta header. Combined with flock-based
//!   lease detection for process death handling.

use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

// ============================================================================
// Constants
// ============================================================================

/// Maximum publishers and subscribers per SHM fanout topic.
pub(crate) const MAX_FANOUT_ENDPOINTS: usize = 16;

/// Magic number for FanoutShmMeta validation: "FANOUT\0\0"
pub(crate) const FANOUT_MAGIC: u64 = 0x0000_5455_4F4E_4146;

/// Offset of FanoutShmMeta in the SHM file (page-aligned, after TopicHeader).
pub(crate) const FANOUT_META_OFFSET: usize = 4096;

/// Size of the FanoutShmMeta header (2 cache lines = 128 bytes).
pub(crate) const FANOUT_META_SIZE: usize = 128;

/// Base offset where channels start (meta offset + meta size, cache-aligned).
pub(crate) const FANOUT_CHANNELS_BASE: usize = FANOUT_META_OFFSET + FANOUT_META_SIZE;

/// Minimum slot size (8 bytes) — ensures atomic-width alignment.
const MIN_SLOT_SIZE: usize = 8;

/// Per-channel overhead: head cache line (64B) + tail cache line (64B) = 128B.
const CHANNEL_HEADER_SIZE: usize = 128;

// ============================================================================
// FanoutShmMeta — shared metadata header in SHM
// ============================================================================

/// Fanout metadata stored at `FANOUT_META_OFFSET` in the SHM region.
///
/// Two cache lines (128 bytes). First cache line is read-mostly (dimensions),
/// second cache line has the dynamic counters (publisher/subscriber registration).
#[repr(C, align(64))]
pub(crate) struct FanoutShmMeta {
    // --- Cache line 1: read-mostly dimensions (set once at creation) ---
    /// Magic number for validation (`FANOUT_MAGIC`).
    pub magic: u64,
    /// Maximum number of publishers this layout supports.
    pub max_publishers: u32,
    /// Maximum number of subscribers this layout supports.
    pub max_subscribers: u32,
    /// SPSC channel capacity (power of 2).
    pub channel_capacity: u32,
    /// Slot size in bytes (sizeof(T) rounded up to MIN_SLOT_SIZE alignment).
    pub slot_size: u32,
    /// Total bytes per SPSC channel (CHANNEL_HEADER_SIZE + capacity × slot_size).
    pub channel_stride: u64,
    /// Capacity mask (channel_capacity - 1) for fast modulo.
    pub capacity_mask: u32,
    /// Reserved for future use.
    pub _reserved1: u32,
    /// Total file size in bytes.
    pub total_file_size: u64,
    _pad1: [u8; 8],

    // --- Cache line 2: dynamic registration counters ---
    /// Number of registered publishers (atomic for cross-process CAS).
    pub num_publishers: AtomicU32,
    /// Number of registered subscribers (atomic for cross-process CAS).
    pub num_subscribers: AtomicU32,
    /// Reserved for lease/death tracking.
    pub _reserved2: [u8; 56],
}

// Static assertions
const _: () = {
    assert!(std::mem::size_of::<FanoutShmMeta>() == FANOUT_META_SIZE);
    assert!(std::mem::align_of::<FanoutShmMeta>() == 64);
};

// ============================================================================
// ShmSpscChannel — a single SPSC channel operating on mmap'd memory
// ============================================================================

/// View into a single SPSC channel within the SHM fanout region.
///
/// This is not a standalone struct — it's a computed view into the mmap'd
/// memory. Head and tail are cache-line-separated `AtomicU64` values.
///
/// ```text
/// channel_base + 0:    [head: AtomicU64] + [padding to 64B]
/// channel_base + 64:   [tail: AtomicU64] + [padding to 64B]
/// channel_base + 128:  [slot[0]: T]
/// channel_base + 128 + slot_size: [slot[1]: T]
/// ...
/// ```
pub(crate) struct ShmSpscChannel {
    /// Pointer to head AtomicU64 (producer-owned, on its own cache line).
    head_ptr: *const AtomicU64,
    /// Pointer to tail AtomicU64 (consumer-owned, on its own cache line).
    tail_ptr: *const AtomicU64,
    /// Pointer to the first data slot.
    data_ptr: *mut u8,
    /// Slot size in bytes.
    slot_size: usize,
    /// Capacity mask (capacity - 1).
    mask: u64,
    /// Capacity.
    capacity: u64,
}

impl ShmSpscChannel {
    /// Construct a channel view from a base pointer within the SHM region.
    ///
    /// # Safety
    ///
    /// `base` must point to a properly initialized channel region within a valid
    /// mmap'd SHM file, with at least `CHANNEL_HEADER_SIZE + capacity * slot_size`
    /// bytes available.
    #[inline]
    pub(crate) unsafe fn from_raw(
        base: *mut u8,
        slot_size: usize,
        capacity: u32,
        capacity_mask: u32,
    ) -> Self {
        Self {
            head_ptr: base as *const AtomicU64,
            tail_ptr: base.add(64) as *const AtomicU64,
            data_ptr: base.add(CHANNEL_HEADER_SIZE),
            slot_size,
            mask: capacity_mask as u64,
            capacity: capacity as u64,
        }
    }

    /// Try to send a POD value into this SPSC channel.
    ///
    /// # Safety
    ///
    /// - Only one producer may call this concurrently (SPSC contract).
    /// - `T` must be POD (no Drop, no heap pointers).
    /// - `cached_tail` must be the caller's local cache of the tail value.
    #[inline(always)]
    pub(crate) unsafe fn try_send_pod<T>(&self, msg: &T, cached_tail: &mut u64) -> bool {
        let head = (*self.head_ptr).load(Ordering::Relaxed);

        // Lazy tail cache: only load atomic tail when cached says full
        if head.wrapping_sub(*cached_tail) >= self.capacity {
            *cached_tail = (*self.tail_ptr).load(Ordering::Acquire);
            if head.wrapping_sub(*cached_tail) >= self.capacity {
                return false; // Full
            }
        }

        let index = (head & self.mask) as usize;
        let slot = self.data_ptr.add(index * self.slot_size);

        // Write data — raw memcpy for POD types
        std::ptr::copy_nonoverlapping(msg as *const T as *const u8, slot, std::mem::size_of::<T>());

        // Release store makes data visible to consumer
        (*self.head_ptr).store(head.wrapping_add(1), Ordering::Release);
        true
    }

    /// Try to receive a POD value from this SPSC channel.
    ///
    /// # Safety
    ///
    /// - Only one consumer may call this concurrently (SPSC contract).
    /// - `T` must be POD (bitwise-copyable).
    #[inline(always)]
    pub(crate) unsafe fn try_recv_pod<T>(&self) -> Option<T> {
        let tail = (*self.tail_ptr).load(Ordering::Relaxed);
        let head = (*self.head_ptr).load(Ordering::Acquire);

        if tail >= head {
            return None; // Empty
        }

        let index = (tail & self.mask) as usize;
        let slot = self.data_ptr.add(index * self.slot_size);

        // Read data — raw memcpy for POD types
        let mut val = std::mem::MaybeUninit::<T>::uninit();
        std::ptr::copy_nonoverlapping(slot, val.as_mut_ptr() as *mut u8, std::mem::size_of::<T>());

        // Release store advances tail, making slot reusable by producer
        (*self.tail_ptr).store(tail.wrapping_add(1), Ordering::Release);
        Some(val.assume_init())
    }

    /// Try to send serialized bytes into this SPSC channel (for Serde types).
    ///
    /// Stores `[len: u32, data: [u8; len]]` in the slot. Slot must be large enough.
    ///
    /// # Safety
    ///
    /// Single-producer contract. `bytes.len() + 4` must fit in `slot_size`.
    #[inline(always)]
    pub(crate) unsafe fn try_send_serde(&self, bytes: &[u8], cached_tail: &mut u64) -> bool {
        let needed = 4 + bytes.len();
        if needed > self.slot_size {
            return false; // Message too large for slot
        }

        let head = (*self.head_ptr).load(Ordering::Relaxed);

        if head.wrapping_sub(*cached_tail) >= self.capacity {
            *cached_tail = (*self.tail_ptr).load(Ordering::Acquire);
            if head.wrapping_sub(*cached_tail) >= self.capacity {
                return false;
            }
        }

        let index = (head & self.mask) as usize;
        let slot = self.data_ptr.add(index * self.slot_size);

        // Write length prefix
        let len = bytes.len() as u32;
        std::ptr::copy_nonoverlapping(&len as *const u32 as *const u8, slot, 4);
        // Write serialized data
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), slot.add(4), bytes.len());

        (*self.head_ptr).store(head.wrapping_add(1), Ordering::Release);
        true
    }

    /// Try to receive serialized bytes from this SPSC channel (for Serde types).
    ///
    /// Returns a Vec<u8> containing the serialized data.
    ///
    /// # Safety
    ///
    /// Single-consumer contract.
    #[inline(always)]
    pub(crate) unsafe fn try_recv_serde(&self) -> Option<Vec<u8>> {
        let tail = (*self.tail_ptr).load(Ordering::Relaxed);
        let head = (*self.head_ptr).load(Ordering::Acquire);

        if tail >= head {
            return None;
        }

        let index = (tail & self.mask) as usize;
        let slot = self.data_ptr.add(index * self.slot_size);

        // Read length prefix
        let mut len: u32 = 0;
        std::ptr::copy_nonoverlapping(slot, &mut len as *mut u32 as *mut u8, 4);

        // Read serialized data
        let mut data = vec![0u8; len as usize];
        std::ptr::copy_nonoverlapping(slot.add(4), data.as_mut_ptr(), len as usize);

        (*self.tail_ptr).store(tail.wrapping_add(1), Ordering::Release);
        Some(data)
    }
}

// SAFETY: ShmSpscChannel is a view into mmap'd memory. The SPSC contract
// (single producer, single consumer) ensures memory safety. The mmap region
// outlives the channel view (guaranteed by ShmFanoutRing holding Arc<ShmRegion>).
unsafe impl Send for ShmSpscChannel {}
unsafe impl Sync for ShmSpscChannel {}

// ============================================================================
// ShmFanoutRing — cross-process fan-out MPMC via SHM SPSC matrix
// ============================================================================

/// Cross-process contention-free MPMC ring using a matrix of SHM-backed SPSC channels.
///
/// This is the SHM equivalent of [`FanoutRing`](super::fanout::FanoutRing).
/// Each publisher-subscriber pair gets a dedicated SPSC channel in shared memory.
///
/// # Usage
///
/// Created by the topic system when `BackendMode::FanoutShm` is selected
/// (cross-process, pubs > 1, subs > 1).
pub(crate) struct ShmFanoutRing {
    /// Pointer to the FanoutShmMeta in the SHM region.
    meta_ptr: *const FanoutShmMeta,
    /// Pre-computed channel views: channels[pub_id * max_subs + sub_id]
    channels: Vec<ShmSpscChannel>,
    /// Max publishers (from meta).
    max_publishers: usize,
    /// Max subscribers (from meta).
    max_subscribers: usize,
    /// Per-subscriber round-robin cursor (local to this process).
    recv_cursors: [std::cell::Cell<usize>; MAX_FANOUT_ENDPOINTS],
    /// Per-channel cached tail values for producers (local to this process).
    /// Layout: cached_tails[pub_id * max_subs + sub_id]
    cached_tails: Vec<std::cell::Cell<u64>>,
    /// Whether this is a POD type (determines send/recv path).
    is_pod: bool,
}

// SAFETY (Send): The ShmFanoutRing's mmap pointer and Cell fields transfer
// cleanly to a new thread — no thread-pinned resources.
unsafe impl Send for ShmFanoutRing {}

// SAFETY (Sync): The SPSC channels use atomics for inter-thread head/tail
// coordination. However, the process-local `recv_cursors` (Cell<usize>) and
// `cached_tails` (Cell<u64>) are NOT synchronized — concurrent recv_pod calls
// from different threads on the same subscriber index would race on these Cells.
//
// INVARIANT: Each subscriber index must be accessed from exactly one thread.
// This is enforced by the register_subscriber() protocol: each caller gets a
// unique sub_id and uses only that index.
unsafe impl Sync for ShmFanoutRing {}

impl ShmFanoutRing {
    /// Initialize a new ShmFanoutRing from a freshly created SHM region.
    ///
    /// The caller (SHM region owner) must have already written the TopicHeader.
    /// This writes the FanoutShmMeta and returns the ring view.
    ///
    /// # Safety
    ///
    /// `shm_base` must point to a valid mmap'd region of at least
    /// `Self::required_file_size(type_size, channel_capacity)` bytes.
    pub(crate) unsafe fn init_owner(
        shm_base: *mut u8,
        type_size: usize,
        is_pod: bool,
        channel_capacity: u32,
    ) -> Self {
        let slot_size = Self::compute_slot_size(type_size, is_pod);
        let cap = (channel_capacity as usize).next_power_of_two().max(16) as u32;
        let max_pubs = MAX_FANOUT_ENDPOINTS as u32;
        let max_subs = MAX_FANOUT_ENDPOINTS as u32;
        let channel_stride = (CHANNEL_HEADER_SIZE + cap as usize * slot_size) as u64;
        let total_size = Self::required_file_size(type_size, is_pod, cap as usize);

        // Write FanoutShmMeta
        let meta = &mut *(shm_base.add(FANOUT_META_OFFSET) as *mut FanoutShmMeta);
        meta.max_publishers = max_pubs;
        meta.max_subscribers = max_subs;
        meta.channel_capacity = cap;
        meta.slot_size = slot_size as u32;
        meta.channel_stride = channel_stride;
        meta.capacity_mask = cap - 1;
        meta.total_file_size = total_size as u64;
        meta.num_publishers = AtomicU32::new(0);
        meta.num_subscribers = AtomicU32::new(0);

        // Zero-init all channel head/tail atomics
        let channels_base = shm_base.add(FANOUT_CHANNELS_BASE);
        let num_channels = max_pubs as usize * max_subs as usize;
        for i in 0..num_channels {
            let ch_base = channels_base.add(i * channel_stride as usize);
            // head = 0
            (ch_base as *mut AtomicU64).write(AtomicU64::new(0));
            // tail = 0
            (ch_base.add(64) as *mut AtomicU64).write(AtomicU64::new(0));
        }

        // Write magic LAST (signals initialization complete)
        std::sync::atomic::fence(Ordering::Release);
        meta.magic = FANOUT_MAGIC;

        Self::build_views(shm_base, is_pod, type_size)
    }

    /// Attach to an existing SHM fanout region (non-owner process).
    ///
    /// Spins waiting for `FANOUT_MAGIC` to appear (owner still initializing).
    ///
    /// # Safety
    ///
    /// `shm_base` must point to a valid mmap'd SHM region.
    pub(crate) unsafe fn attach(shm_base: *mut u8, is_pod: bool, type_size: usize) -> Self {
        let meta = &*(shm_base.add(FANOUT_META_OFFSET) as *const FanoutShmMeta);

        // Spin-wait for owner to finish initialization.
        // meta is a raw pointer read through a shared-memory mapping; the mutation
        // happens in another process, so clippy cannot see it — suppress the lint.
        let mut spins = 0u32;
        #[allow(clippy::while_immutable_condition)]
        while meta.magic != FANOUT_MAGIC {
            std::hint::spin_loop();
            spins += 1;
            if spins > 1_000_000 {
                // Yield after extensive spinning
                std::thread::yield_now();
                spins = 0;
            }
        }
        std::sync::atomic::fence(Ordering::Acquire);

        Self::build_views(shm_base, is_pod, type_size)
    }

    /// Build channel views from an initialized SHM region.
    unsafe fn build_views(shm_base: *mut u8, is_pod: bool, _type_size: usize) -> Self {
        let meta = &*(shm_base.add(FANOUT_META_OFFSET) as *const FanoutShmMeta);
        let max_pubs = meta.max_publishers as usize;
        let max_subs = meta.max_subscribers as usize;
        let slot_size = meta.slot_size as usize;
        let capacity = meta.channel_capacity;
        let capacity_mask = meta.capacity_mask;
        let channel_stride = meta.channel_stride as usize;

        let channels_base = shm_base.add(FANOUT_CHANNELS_BASE);
        let num_channels = max_pubs * max_subs;

        let mut channels = Vec::with_capacity(num_channels);
        for i in 0..num_channels {
            let ch_base = channels_base.add(i * channel_stride);
            channels.push(ShmSpscChannel::from_raw(
                ch_base,
                slot_size,
                capacity,
                capacity_mask,
            ));
        }

        let cached_tails: Vec<std::cell::Cell<u64>> =
            (0..num_channels).map(|_| std::cell::Cell::new(0)).collect();

        // SAFETY: CELL_INIT is a const used only as an array initializer pattern.
        // Cell<usize> is intentional — recv_cursors tracks per-subscriber read
        // positions with thread-local (not atomic) semantics.
        #[allow(clippy::declare_interior_mutable_const)]
        const CELL_INIT: std::cell::Cell<usize> = std::cell::Cell::new(0);
        Self {
            meta_ptr: shm_base.add(FANOUT_META_OFFSET) as *const FanoutShmMeta,
            channels,
            max_publishers: max_pubs,
            max_subscribers: max_subs,
            recv_cursors: [CELL_INIT; MAX_FANOUT_ENDPOINTS],
            cached_tails,
            is_pod,
        }
    }

    /// Register a new publisher (cross-process safe via atomic fetch_add).
    /// Returns the publisher ID.
    pub(crate) fn register_publisher(&self) -> usize {
        let meta = unsafe { &*self.meta_ptr };
        let id = meta.num_publishers.fetch_add(1, Ordering::Relaxed) as usize;
        assert!(
            id < self.max_publishers,
            "ShmFanoutRing: too many publishers ({} >= {})",
            id,
            self.max_publishers
        );
        id
    }

    /// Register a new subscriber (cross-process safe via atomic fetch_add).
    /// Returns the subscriber ID.
    pub(crate) fn register_subscriber(&self) -> usize {
        let meta = unsafe { &*self.meta_ptr };
        let id = meta.num_subscribers.fetch_add(1, Ordering::Relaxed) as usize;
        assert!(
            id < self.max_subscribers,
            "ShmFanoutRing: too many subscribers ({} >= {})",
            id,
            self.max_subscribers
        );
        id
    }

    /// Send a POD message from publisher `pub_id` to ALL subscribers.
    ///
    /// # Safety
    ///
    /// `T` must be POD. Only one thread per process should use a given `pub_id`.
    #[inline(always)]
    pub(crate) unsafe fn send_pod<T>(&self, msg: &T, pub_id: usize) -> bool {
        let meta = &*self.meta_ptr;
        let n_subs = meta.num_subscribers.load(Ordering::Relaxed) as usize;
        if n_subs == 0 {
            return true; // No subscribers — silent drop
        }

        let mut all_ok = true;
        for sub_id in 0..n_subs {
            let ch_idx = pub_id * self.max_subscribers + sub_id;
            let cached_tail = &self.cached_tails[ch_idx];
            let mut ct = cached_tail.get();
            let ok = self.channels[ch_idx].try_send_pod(msg, &mut ct);
            cached_tail.set(ct);
            if !ok {
                all_ok = false;
            }
        }
        all_ok
    }

    /// Receive a POD message for subscriber `sub_id` from ANY publisher.
    /// Round-robin polls across publishers for fairness.
    ///
    /// # Safety
    ///
    /// `T` must be POD. Only one thread per process should use a given `sub_id`.
    #[inline(always)]
    pub(crate) unsafe fn recv_pod<T>(&self, sub_id: usize) -> Option<T> {
        let meta = &*self.meta_ptr;
        let n_pubs = meta.num_publishers.load(Ordering::Relaxed) as usize;
        if n_pubs == 0 {
            return None;
        }

        let start = self.recv_cursors[sub_id].get() % n_pubs;

        for offset in 0..n_pubs {
            let pub_id = (start + offset) % n_pubs;
            let ch_idx = pub_id * self.max_subscribers + sub_id;
            if let Some(msg) = self.channels[ch_idx].try_recv_pod::<T>() {
                self.recv_cursors[sub_id].set(pub_id + 1);
                return Some(msg);
            }
        }

        None
    }

    /// Send serialized bytes from publisher `pub_id` to ALL subscribers.
    ///
    /// # Safety
    ///
    /// Single-producer-per-pub_id contract.
    #[inline(always)]
    pub(crate) unsafe fn send_serde(&self, bytes: &[u8], pub_id: usize) -> bool {
        let meta = &*self.meta_ptr;
        let n_subs = meta.num_subscribers.load(Ordering::Relaxed) as usize;
        if n_subs == 0 {
            return true;
        }

        let mut all_ok = true;
        for sub_id in 0..n_subs {
            let ch_idx = pub_id * self.max_subscribers + sub_id;
            let cached_tail = &self.cached_tails[ch_idx];
            let mut ct = cached_tail.get();
            let ok = self.channels[ch_idx].try_send_serde(bytes, &mut ct);
            cached_tail.set(ct);
            if !ok {
                all_ok = false;
            }
        }
        all_ok
    }

    /// Receive serialized bytes for subscriber `sub_id` from ANY publisher.
    ///
    /// # Safety
    ///
    /// Single-consumer-per-sub_id contract.
    #[inline(always)]
    pub(crate) unsafe fn recv_serde(&self, sub_id: usize) -> Option<Vec<u8>> {
        let meta = &*self.meta_ptr;
        let n_pubs = meta.num_publishers.load(Ordering::Relaxed) as usize;
        if n_pubs == 0 {
            return None;
        }

        let start = self.recv_cursors[sub_id].get() % n_pubs;

        for offset in 0..n_pubs {
            let pub_id = (start + offset) % n_pubs;
            let ch_idx = pub_id * self.max_subscribers + sub_id;
            if let Some(data) = self.channels[ch_idx].try_recv_serde() {
                self.recv_cursors[sub_id].set(pub_id + 1);
                return Some(data);
            }
        }

        None
    }

    /// Check if this ring is operating on POD types.
    #[inline]
    pub(crate) fn is_pod(&self) -> bool {
        self.is_pod
    }

    // ========================================================================
    // Test-only introspection helpers
    // ========================================================================

    /// Get the number of registered publishers.
    #[cfg(test)]
    #[inline]
    pub(crate) fn num_publishers(&self) -> usize {
        let meta = unsafe { &*self.meta_ptr };
        meta.num_publishers.load(Ordering::Relaxed) as usize
    }

    /// Get the number of registered subscribers.
    #[cfg(test)]
    #[inline]
    pub(crate) fn num_subscribers(&self) -> usize {
        let meta = unsafe { &*self.meta_ptr };
        meta.num_subscribers.load(Ordering::Relaxed) as usize
    }

    // ========================================================================
    // Sizing helpers
    // ========================================================================

    /// Compute slot size from type size.
    fn compute_slot_size(type_size: usize, is_pod: bool) -> usize {
        if is_pod {
            // POD: raw T with minimum alignment
            type_size.max(MIN_SLOT_SIZE).next_power_of_two().min(4096)
        } else {
            // Serde: [len: u32, data: [u8; ...]] — use 8KB default
            8192
        }
    }

    /// Calculate the total SHM file size needed for a fanout layout.
    pub(crate) fn required_file_size(
        type_size: usize,
        is_pod: bool,
        channel_capacity: usize,
    ) -> usize {
        let slot_size = Self::compute_slot_size(type_size, is_pod);
        let cap = channel_capacity.next_power_of_two().max(16);
        let channel_stride = CHANNEL_HEADER_SIZE + cap * slot_size;
        let num_channels = MAX_FANOUT_ENDPOINTS * MAX_FANOUT_ENDPOINTS;

        FANOUT_CHANNELS_BASE + num_channels * channel_stride
    }

    /// Total pending messages across all channels for a subscriber.
    #[cfg(test)]
    pub(crate) fn pending_count_for_sub(&self, sub_id: usize) -> u64 {
        let n_pubs = self.num_publishers();
        let mut total = 0u64;
        for pub_id in 0..n_pubs {
            let ch_idx = pub_id * self.max_subscribers + sub_id;
            let ch = &self.channels[ch_idx];
            unsafe {
                let head = (*ch.head_ptr).load(Ordering::Relaxed);
                let tail = (*ch.tail_ptr).load(Ordering::Relaxed);
                total += head.wrapping_sub(tail);
            }
        }
        total
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::alloc::{alloc_zeroed, Layout};

    /// Allocate a properly aligned buffer simulating an SHM region.
    fn alloc_shm_sim(type_size: usize, is_pod: bool, cap: usize) -> (*mut u8, Layout) {
        let size = ShmFanoutRing::required_file_size(type_size, is_pod, cap);
        let layout = Layout::from_size_align(size, 4096).unwrap();
        let ptr = unsafe { alloc_zeroed(layout) };
        assert!(!ptr.is_null());
        (ptr, layout)
    }

    #[test]
    fn meta_size_and_alignment() {
        assert_eq!(std::mem::size_of::<FanoutShmMeta>(), 128);
        assert_eq!(std::mem::align_of::<FanoutShmMeta>(), 64);
    }

    #[test]
    fn required_file_size_u64() {
        let size = ShmFanoutRing::required_file_size(8, true, 64);
        // 4224 base + 256 channels × (128 header + 64 slots × 8 bytes)
        let expected = FANOUT_CHANNELS_BASE + 256 * (128 + 64 * 8);
        assert_eq!(size, expected);
    }

    #[test]
    fn init_owner_sets_magic() {
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = ShmFanoutRing::init_owner(ptr, 8, true, 64);
            let meta = &*(ptr.add(FANOUT_META_OFFSET) as *const FanoutShmMeta);
            assert_eq!(meta.magic, FANOUT_MAGIC);
            assert_eq!(meta.max_publishers, MAX_FANOUT_ENDPOINTS as u32);
            assert_eq!(meta.max_subscribers, MAX_FANOUT_ENDPOINTS as u32);
            assert_eq!(meta.channel_capacity, 64);
            assert_eq!(ring.num_publishers(), 0);
            assert_eq!(ring.num_subscribers(), 0);
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn register_publisher_subscriber() {
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = ShmFanoutRing::init_owner(ptr, 8, true, 64);
            assert_eq!(ring.register_publisher(), 0);
            assert_eq!(ring.register_publisher(), 1);
            assert_eq!(ring.register_subscriber(), 0);
            assert_eq!(ring.num_publishers(), 2);
            assert_eq!(ring.num_subscribers(), 1);
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn send_recv_pod_1p1s() {
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = ShmFanoutRing::init_owner(ptr, 8, true, 64);
            let pub_id = ring.register_publisher();
            let sub_id = ring.register_subscriber();

            let val: u64 = 42;
            assert!(ring.send_pod(&val, pub_id));

            let recv: Option<u64> = ring.recv_pod(sub_id);
            assert_eq!(recv, Some(42));

            // Empty after consuming
            let recv2: Option<u64> = ring.recv_pod(sub_id);
            assert_eq!(recv2, None);

            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn send_recv_pod_2p2s() {
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = ShmFanoutRing::init_owner(ptr, 8, true, 64);
            let p0 = ring.register_publisher();
            let p1 = ring.register_publisher();
            let s0 = ring.register_subscriber();
            let s1 = ring.register_subscriber();

            // Publisher 0 sends 100
            let v100: u64 = 100;
            ring.send_pod(&v100, p0);
            // Publisher 1 sends 200
            let v200: u64 = 200;
            ring.send_pod(&v200, p1);

            // Both subscribers should see both messages
            let r0a: Option<u64> = ring.recv_pod(s0);
            let r0b: Option<u64> = ring.recv_pod(s0);
            let mut got0 = vec![r0a.unwrap(), r0b.unwrap()];
            got0.sort();
            assert_eq!(got0, vec![100, 200]);

            let r1a: Option<u64> = ring.recv_pod(s1);
            let r1b: Option<u64> = ring.recv_pod(s1);
            let mut got1 = vec![r1a.unwrap(), r1b.unwrap()];
            got1.sort();
            assert_eq!(got1, vec![100, 200]);

            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn send_recv_serde() {
        let (ptr, layout) = alloc_shm_sim(0, false, 32);
        unsafe {
            let ring = ShmFanoutRing::init_owner(ptr, 0, false, 32);
            let pub_id = ring.register_publisher();
            let sub_id = ring.register_subscriber();

            let data = b"hello world";
            assert!(ring.send_serde(data, pub_id));

            let recv = ring.recv_serde(sub_id);
            assert_eq!(recv.as_deref(), Some(&b"hello world"[..]));

            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn capacity_full_returns_false() {
        let (ptr, layout) = alloc_shm_sim(8, true, 16);
        unsafe {
            let ring = ShmFanoutRing::init_owner(ptr, 8, true, 16);
            let pub_id = ring.register_publisher();
            let _sub_id = ring.register_subscriber();

            // Fill all 16 slots
            for i in 0u64..16 {
                assert!(ring.send_pod(&i, pub_id), "send {} should succeed", i);
            }
            // 17th should fail
            let v17: u64 = 17;
            assert!(!ring.send_pod(&v17, pub_id));

            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn round_robin_fairness() {
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = ShmFanoutRing::init_owner(ptr, 8, true, 64);
            let p0 = ring.register_publisher();
            let p1 = ring.register_publisher();
            let sub = ring.register_subscriber();

            // P0 sends 10, 20; P1 sends 30, 40
            for v in [10u64, 20] {
                ring.send_pod(&v, p0);
            }
            for v in [30u64, 40] {
                ring.send_pod(&v, p1);
            }

            // Round-robin should alternate: first from P0, then P1, etc.
            let r1: u64 = ring.recv_pod(sub).unwrap();
            let r2: u64 = ring.recv_pod(sub).unwrap();
            // r1 should be from P0 (10), r2 from P1 (30) — round-robin
            assert_eq!(r1, 10);
            assert_eq!(r2, 30);

            // Continue: P0 (20), then P1 (40)
            let r3: u64 = ring.recv_pod(sub).unwrap();
            let r4: u64 = ring.recv_pod(sub).unwrap();
            assert_eq!(r3, 20);
            assert_eq!(r4, 40);

            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn attach_after_init() {
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            // Owner initializes
            let _owner = ShmFanoutRing::init_owner(ptr, 8, true, 64);

            // Simulated second process attaches
            let joiner = ShmFanoutRing::attach(ptr, true, 8);
            assert_eq!(joiner.max_publishers, MAX_FANOUT_ENDPOINTS);
            assert_eq!(joiner.max_subscribers, MAX_FANOUT_ENDPOINTS);

            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn pending_count() {
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = ShmFanoutRing::init_owner(ptr, 8, true, 64);
            let p0 = ring.register_publisher();
            let sub = ring.register_subscriber();

            assert_eq!(ring.pending_count_for_sub(sub), 0);

            for i in 0u64..5 {
                ring.send_pod(&i, p0);
            }
            assert_eq!(ring.pending_count_for_sub(sub), 5);

            let _: Option<u64> = ring.recv_pod(sub);
            let _: Option<u64> = ring.recv_pod(sub);
            assert_eq!(ring.pending_count_for_sub(sub), 3);

            std::alloc::dealloc(ptr, layout);
        }
    }

    // ====================================================================
    // Production-grade tests
    // ====================================================================

    #[test]
    fn wrap_around_pod_correctness() {
        // Force multiple wraps through a small-capacity ring
        let (ptr, layout) = alloc_shm_sim(8, true, 16);
        unsafe {
            let ring = ShmFanoutRing::init_owner(ptr, 8, true, 16);
            let p = ring.register_publisher();
            let s = ring.register_subscriber();

            for round in 0u64..20 {
                for i in 0u64..16 {
                    let val = round * 1000 + i;
                    assert!(
                        ring.send_pod(&val, p),
                        "send failed round={} i={}",
                        round,
                        i
                    );
                }
                for i in 0u64..16 {
                    let expected = round * 1000 + i;
                    let got: Option<u64> = ring.recv_pod(s);
                    assert_eq!(got, Some(expected), "mismatch round={} i={}", round, i);
                }
            }
            // 320 messages through 16-slot ring = 20 wraps, all correct
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn wrap_around_serde_correctness() {
        let (ptr, layout) = alloc_shm_sim(0, false, 16);
        unsafe {
            let ring = ShmFanoutRing::init_owner(ptr, 0, false, 16);
            let p = ring.register_publisher();
            let s = ring.register_subscriber();

            for round in 0..10 {
                for i in 0..16 {
                    let msg = format!("r{}i{}", round, i);
                    assert!(ring.send_serde(msg.as_bytes(), p));
                }
                for i in 0..16 {
                    let expected = format!("r{}i{}", round, i);
                    let got = ring.recv_serde(s).unwrap();
                    assert_eq!(got, expected.as_bytes());
                }
            }
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn large_pod_type() {
        // Test with a larger POD type (128 bytes)
        #[repr(C)]
        #[derive(Clone, Copy, Debug, PartialEq)]
        struct LargePod {
            data: [u64; 16], // 128 bytes
        }

        let (ptr, layout) = alloc_shm_sim(128, true, 32);
        unsafe {
            let ring = ShmFanoutRing::init_owner(ptr, 128, true, 32);
            let p = ring.register_publisher();
            let s = ring.register_subscriber();

            for i in 0u64..32 {
                let val = LargePod { data: [i; 16] };
                assert!(ring.send_pod(&val, p));
            }
            for i in 0u64..32 {
                let got: Option<LargePod> = ring.recv_pod(s);
                let expected = LargePod { data: [i; 16] };
                assert_eq!(got, Some(expected), "mismatch at i={}", i);
            }
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    #[ignore] // Stress test — too slow in debug mode (spin-loop on 8-slot ring). Run with --release --ignored.
    fn cross_thread_pod_stress() {
        // 2P/2S cross-thread with data integrity verification
        let size = ShmFanoutRing::required_file_size(8, true, 256);
        let layout = std::alloc::Layout::from_size_align(size, 4096).unwrap();
        let ptr = unsafe { alloc_zeroed(layout) };
        assert!(!ptr.is_null());

        let ring = unsafe { ShmFanoutRing::init_owner(ptr, 8, true, 256) };
        // Box it so we can share across threads via raw pointer
        let ring = Box::into_raw(Box::new(ring));

        let msgs = 10_000usize; // Lower for debug mode; 50K+ in release
        let barrier = std::sync::Arc::new(std::sync::Barrier::new(4));

        // Register endpoints before spawning threads
        let p0 = unsafe { (*ring).register_publisher() };
        let p1 = unsafe { (*ring).register_publisher() };
        let s0 = unsafe { (*ring).register_subscriber() };
        let s1 = unsafe { (*ring).register_subscriber() };

        let ring_addr = ring as usize;

        let b = barrier.clone();
        let h_p0 = std::thread::spawn(move || {
            let ring = unsafe { &*(ring_addr as *const ShmFanoutRing) };
            b.wait();
            for i in 0..msgs as u64 {
                let val = i * 2; // even
                while !unsafe { ring.send_pod(&val, p0) } {
                    std::hint::spin_loop();
                }
            }
        });

        let b = barrier.clone();
        let h_p1 = std::thread::spawn(move || {
            let ring = unsafe { &*(ring_addr as *const ShmFanoutRing) };
            b.wait();
            for i in 0..msgs as u64 {
                let val = i * 2 + 1; // odd
                while !unsafe { ring.send_pod(&val, p1) } {
                    std::hint::spin_loop();
                }
            }
        });

        let b = barrier.clone();
        let h_s0 = std::thread::spawn(move || {
            let ring = unsafe { &*(ring_addr as *const ShmFanoutRing) };
            b.wait();
            let mut received = Vec::with_capacity(msgs * 2);
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
            while received.len() < msgs * 2 && std::time::Instant::now() < deadline {
                if let Some(v) = unsafe { ring.recv_pod::<u64>(s0) } {
                    received.push(v);
                }
            }
            received
        });

        let b = barrier.clone();
        let h_s1 = std::thread::spawn(move || {
            let ring = unsafe { &*(ring_addr as *const ShmFanoutRing) };
            b.wait();
            let mut received = Vec::with_capacity(msgs * 2);
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
            while received.len() < msgs * 2 && std::time::Instant::now() < deadline {
                if let Some(v) = unsafe { ring.recv_pod::<u64>(s1) } {
                    received.push(v);
                }
            }
            received
        });

        h_p0.join().unwrap();
        h_p1.join().unwrap();
        let s0_data = h_s0.join().unwrap();
        let s1_data = h_s1.join().unwrap();

        // Verify both subscribers got all messages
        assert_eq!(
            s0_data.len(),
            msgs * 2,
            "s0 got {}/{}",
            s0_data.len(),
            msgs * 2
        );
        assert_eq!(
            s1_data.len(),
            msgs * 2,
            "s1 got {}/{}",
            s1_data.len(),
            msgs * 2
        );

        // Verify FIFO per publisher
        for data in [&s0_data, &s1_data] {
            let even: Vec<u64> = data.iter().filter(|v| *v % 2 == 0).copied().collect();
            let odd: Vec<u64> = data.iter().filter(|v| *v % 2 == 1).copied().collect();
            assert_eq!(even.len(), msgs);
            assert_eq!(odd.len(), msgs);
            for w in even.windows(2) {
                assert!(w[0] < w[1], "even ordering: {} >= {}", w[0], w[1]);
            }
            for w in odd.windows(2) {
                assert!(w[0] < w[1], "odd ordering: {} >= {}", w[0], w[1]);
            }
        }

        // Verify no duplicates
        let mut s0_sorted = s0_data.clone();
        s0_sorted.sort();
        s0_sorted.dedup();
        assert_eq!(s0_sorted.len(), msgs * 2, "s0 has duplicates");

        unsafe {
            drop(Box::from_raw(ring));
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn capacity_recovery_after_full() {
        let (ptr, layout) = alloc_shm_sim(8, true, 16);
        unsafe {
            let ring = ShmFanoutRing::init_owner(ptr, 8, true, 16);
            let p = ring.register_publisher();
            let s = ring.register_subscriber();

            for cycle in 0u64..5 {
                // Fill
                for i in 0u64..16 {
                    assert!(ring.send_pod(&(cycle * 100 + i), p));
                }
                let overflow: u64 = 999;
                assert!(!ring.send_pod(&overflow, p), "should be full");

                // Drain and verify
                for i in 0u64..16 {
                    let got: Option<u64> = ring.recv_pod(s);
                    assert_eq!(got, Some(cycle * 100 + i));
                }
                let empty: Option<u64> = ring.recv_pod(s);
                assert_eq!(empty, None);
            }
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn max_endpoints_boundary() {
        let (ptr, layout) = alloc_shm_sim(8, true, 16);
        unsafe {
            let ring = ShmFanoutRing::init_owner(ptr, 8, true, 16);

            // Register max publishers and subscribers
            let mut pubs = Vec::new();
            let mut subs = Vec::new();
            for _ in 0..MAX_FANOUT_ENDPOINTS {
                pubs.push(ring.register_publisher());
                subs.push(ring.register_subscriber());
            }

            // Each publisher sends to all subscribers
            for &pid in &pubs {
                let val: u64 = pid as u64;
                ring.send_pod(&val, pid);
            }

            // Each subscriber receives all messages
            for &sid in &subs {
                let mut received = Vec::new();
                while let Some(v) = ring.recv_pod::<u64>(sid) {
                    received.push(v);
                }
                assert_eq!(
                    received.len(),
                    MAX_FANOUT_ENDPOINTS,
                    "sub {} got {} (expected {})",
                    sid,
                    received.len(),
                    MAX_FANOUT_ENDPOINTS
                );
            }
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn no_publishers_recv_none() {
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = ShmFanoutRing::init_owner(ptr, 8, true, 64);
            let s = ring.register_subscriber();
            let got: Option<u64> = ring.recv_pod(s);
            assert_eq!(got, None);
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn concurrent_registration() {
        // Multiple threads registering simultaneously
        let size = ShmFanoutRing::required_file_size(8, true, 64);
        let layout = std::alloc::Layout::from_size_align(size, 4096).unwrap();
        let ptr = unsafe { alloc_zeroed(layout) };
        let ring = unsafe { ShmFanoutRing::init_owner(ptr, 8, true, 64) };
        let ring = Box::into_raw(Box::new(ring));
        let ring_addr = ring as usize;

        let barrier = std::sync::Arc::new(std::sync::Barrier::new(8));
        let mut handles = Vec::new();

        for _ in 0..4 {
            let b = barrier.clone();
            handles.push(std::thread::spawn(move || {
                let ring = unsafe { &*(ring_addr as *const ShmFanoutRing) };
                b.wait();
                ring.register_publisher()
            }));
        }
        for _ in 0..4 {
            let b = barrier.clone();
            handles.push(std::thread::spawn(move || {
                let ring = unsafe { &*(ring_addr as *const ShmFanoutRing) };
                b.wait();
                ring.register_subscriber()
            }));
        }

        let ids: Vec<usize> = handles.into_iter().map(|h| h.join().unwrap()).collect();
        let pub_ids: std::collections::HashSet<usize> = ids[..4].iter().copied().collect();
        let sub_ids: std::collections::HashSet<usize> = ids[4..].iter().copied().collect();

        // All IDs should be unique within their category
        assert_eq!(pub_ids.len(), 4, "duplicate pub IDs");
        assert_eq!(sub_ids.len(), 4, "duplicate sub IDs");

        unsafe {
            assert_eq!((*ring).num_publishers(), 4);
            assert_eq!((*ring).num_subscribers(), 4);
            drop(Box::from_raw(ring));
            std::alloc::dealloc(ptr, layout);
        }
    }
}
