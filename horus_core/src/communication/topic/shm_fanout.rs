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
//! │ FanoutShmMeta (256 bytes, cache-aligned) │  — dims + endpoint bitmasks/PIDs
//! ├──────────────────────────────────────────┤ offset 4352
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
//! - **Process-safe registration (COMM-H1)**: Each endpoint slot is claimed under
//!   an exclusive `flock` on a per-endpoint lock file (the claim authority), then
//!   recorded in a reusable bitmask + owner-PID array in the FanoutShmMeta header.
//!   A crashed process's flock is released by the OS, which is the sole proof of
//!   abandonment that permits reclaiming its slot — see `register_publisher`.

use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

use horus_sys::fs::FileLock;

use super::seqlock::{seqlock_consume, seqlock_publish};

/// Which endpoint kind a lock file / claim belongs to. Drives the lock-file path
/// component and (for subscribers) the fresh-slot channel reset on claim.
#[derive(Clone, Copy)]
enum EndpointKind {
    Publisher,
    Subscriber,
}

impl EndpointKind {
    /// Path component used in the lock-file name (`pub`/`sub`).
    fn tag(self) -> &'static str {
        match self {
            EndpointKind::Publisher => "pub",
            EndpointKind::Subscriber => "sub",
        }
    }
}

// ============================================================================
// Constants
// ============================================================================

/// Maximum publishers and subscribers per SHM fanout topic.
pub(crate) const MAX_FANOUT_ENDPOINTS: usize = 16;

/// Magic number for FanoutShmMeta validation: "FANOUT\0" + layout version byte.
///
/// The final byte is a LAYOUT VERSION and MUST be bumped whenever the on-SHM
/// channel layout changes, so a region written by an older binary is rejected
/// (clean magic mismatch → SpscShm fallback in `init_shm_backend`) rather than
/// silently reinterpreted with the new strides. v2 added the per-slot version
/// array (each channel grew by `capacity * 8` bytes for the seqlock stamps).
/// v3 (COMM-H1) replaced the monotonic `num_publishers`/`num_subscribers`
/// counters with reusable endpoint bitmasks + per-slot owner PIDs, growing the
/// meta header from 128 to 256 bytes — a v2 region MUST NOT be read with v3
/// strides, hence the version bump and the `attach` rejection of a stale magic.
pub(crate) const FANOUT_MAGIC: u64 = 0x0300_5455_4F4E_4146;

/// Offset of FanoutShmMeta in the SHM file (page-aligned, after TopicHeader).
pub(crate) const FANOUT_META_OFFSET: usize = 4096;

/// Size of the FanoutShmMeta header (4 cache lines = 256 bytes as of v3).
pub(crate) const FANOUT_META_SIZE: usize = 256;

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
/// Four cache lines (256 bytes). Cache line 1 is read-mostly (dimensions set once
/// at creation). Cache lines 2+ hold the dynamic cross-process registration state:
/// per-kind endpoint bitmasks (`pub_active`/`sub_active`, bit i = slot i live) and
/// per-slot owner PIDs (`pub_owner_pids`/`sub_owner_pids`, for the same-process
/// reclaim guard).
///
/// The bitmask replaces the old monotonic `num_publishers`/`num_subscribers`
/// counters. Those counters lived in SHM and so persisted across process death;
/// a crashed process never ran Drop to decrement, so after 16 cumulative
/// (re)registrations the `assert!(id < max)` panicked forever (COMM-H1). A
/// reusable bitmask lets a freed OR crash-abandoned slot be reclaimed. Crash
/// liveness is proven by `flock` on a per-endpoint lock file (see
/// `register_publisher`), NOT by these fields — the OS releases the flock on
/// process death, which is the only sound proof of abandonment.
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
    _pad1: [u8; 16],

    // --- Cache line 2+: dynamic cross-process registration state ---
    /// Live-publisher slot bitmask (bit i set = publisher slot i in use). A
    /// reusable bitmask, not a monotonic counter: a freed or crash-abandoned slot
    /// is reclaimed (COMM-H1), so cumulative (re)registrations never overflow the
    /// fixed 16-slot matrix. This is the hot-path "active set"; `flock` is the
    /// ownership authority and this bit is derived state set only after the flock
    /// is held.
    pub pub_active: AtomicU64,
    /// Live-subscriber slot bitmask (bit i set = subscriber slot i in use).
    pub sub_active: AtomicU64,
    /// Owner PID recorded per publisher slot — read ONLY by the same-process
    /// reclaim guard (a live in-process sibling that the OS let us re-`flock`).
    /// Never an authority: `flock` is the claim authority.
    pub pub_owner_pids: [AtomicU32; MAX_FANOUT_ENDPOINTS],
    /// Owner PID recorded per subscriber slot (same-process reclaim guard).
    pub sub_owner_pids: [AtomicU32; MAX_FANOUT_ENDPOINTS],
    /// Reserved for future use (pads the header to `FANOUT_META_SIZE`).
    pub _reserved2: [u8; 48],
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
/// channel_base + 128 + capacity*slot_size: [ver[0]: AtomicU64]
/// ...                                       [ver[capacity-1]: AtomicU64]
/// ```
pub(crate) struct ShmSpscChannel {
    /// Pointer to head AtomicU64 (producer-owned, on its own cache line).
    head_ptr: *const AtomicU64,
    /// Pointer to tail AtomicU64 (consumer-owned, on its own cache line).
    tail_ptr: *const AtomicU64,
    /// Pointer to the first data slot.
    data_ptr: *mut u8,
    /// Pointer to the per-slot version-stamp array (`capacity` × `AtomicU64`),
    /// laid out immediately after the data slots. See [`super::seqlock`].
    ver_ptr: *const AtomicU64,
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
    /// mmap'd SHM file, with at least
    /// `CHANNEL_HEADER_SIZE + capacity * slot_size + capacity * 8` bytes available.
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
            // Version array sits immediately after the `capacity` data slots.
            ver_ptr: base.add(CHANNEL_HEADER_SIZE + capacity as usize * slot_size)
                as *const AtomicU64,
            slot_size,
            mask: capacity_mask as u64,
            capacity: capacity as u64,
        }
    }

    /// Send a POD value (drop-oldest, never fails, overwrites oldest when full).
    ///
    /// # Safety
    ///
    /// - Only one producer may call this concurrently (SPSC contract).
    /// - `T` must be POD (no Drop, no heap pointers).
    #[inline(always)]
    pub(crate) unsafe fn try_send_pod<T>(&self, msg: &T) {
        let head = &*self.head_ptr;
        let pos = head.load(Ordering::Relaxed);
        seqlock_publish(self.ver_ptr, self.mask, head, pos, |index| {
            let slot = self.data_ptr.add(index * self.slot_size);
            // Raw memcpy for POD types; overwrites the prior occupant in place.
            std::ptr::copy_nonoverlapping(
                msg as *const T as *const u8,
                slot,
                std::mem::size_of::<T>(),
            );
        });
    }

    /// Receive a POD value (latest-wins), or `None` if empty.
    ///
    /// # Safety
    ///
    /// - Only one consumer may call this concurrently (SPSC contract).
    /// - `T` must be POD (bitwise-copyable).
    #[inline(always)]
    pub(crate) unsafe fn try_recv_pod<T>(&self) -> Option<T> {
        let head = &*self.head_ptr;
        let tail = &*self.tail_ptr;
        seqlock_consume(
            self.ver_ptr,
            self.mask,
            self.capacity,
            head,
            tail,
            |index| {
                let slot = self.data_ptr.add(index * self.slot_size);
                let mut val = std::mem::MaybeUninit::<T>::uninit();
                std::ptr::copy_nonoverlapping(
                    slot,
                    val.as_mut_ptr() as *mut u8,
                    std::mem::size_of::<T>(),
                );
                val.assume_init()
            },
            // POD has no Drop; forget the copy without running any destructor in
            // case a torn read produced bit garbage.
            |val| std::mem::forget(val),
        )
    }

    /// Send serialized bytes (drop-oldest). Returns `false` if the message is
    /// too large for a slot (unrelated to backpressure — sends never block).
    ///
    /// Stores `[len: u32, data: [u8; len]]` in the slot.
    ///
    /// # Safety
    ///
    /// Single-producer contract.
    #[inline(always)]
    pub(crate) unsafe fn try_send_serde(&self, bytes: &[u8]) -> bool {
        let needed = 4 + bytes.len();
        if needed > self.slot_size {
            return false; // Message too large for slot
        }

        let head = &*self.head_ptr;
        let pos = head.load(Ordering::Relaxed);
        seqlock_publish(self.ver_ptr, self.mask, head, pos, |index| {
            let slot = self.data_ptr.add(index * self.slot_size);
            let len = bytes.len() as u32;
            std::ptr::copy_nonoverlapping(&len as *const u32 as *const u8, slot, 4);
            std::ptr::copy_nonoverlapping(bytes.as_ptr(), slot.add(4), bytes.len());
        });
        true
    }

    /// Receive serialized bytes (latest-wins), or `None` if empty.
    ///
    /// # Safety
    ///
    /// Single-consumer contract.
    #[inline(always)]
    pub(crate) unsafe fn try_recv_serde(&self) -> Option<Vec<u8>> {
        let head = &*self.head_ptr;
        let tail = &*self.tail_ptr;
        let max_payload = self.slot_size - 4;
        seqlock_consume(
            self.ver_ptr,
            self.mask,
            self.capacity,
            head,
            tail,
            |index| {
                let slot = self.data_ptr.add(index * self.slot_size);
                let mut len_bytes = [0u8; 4];
                std::ptr::copy_nonoverlapping(slot, len_bytes.as_mut_ptr(), 4);
                let len = u32::from_ne_bytes(len_bytes) as usize;
                // A torn read may yield a garbage length; bound it before
                // allocating so we never attempt a huge/UB copy. The seqlock
                // re-check then discards the (empty) result as torn.
                if len > max_payload {
                    return Vec::new();
                }
                let mut data = vec![0u8; len];
                std::ptr::copy_nonoverlapping(slot.add(4), data.as_mut_ptr(), len);
                data
            },
            // Vec<u8> is always a valid owned allocation here (empty or real
            // bytes) — dropping a discarded copy is safe and avoids a leak.
            drop,
        )
    }

    /// Reset the consumer position to the current producer head, discarding any
    /// buffered messages. Called when a subscriber slot is reclaimed (COMM-H1) so
    /// the new owner starts fresh instead of reading the prior owner's backlog.
    ///
    /// # Safety
    ///
    /// The caller must hold the subscriber slot exclusively (no live consumer on
    /// this channel) so the `tail` store is uncontended. The producer-owned `head`
    /// is only read — a live producer may keep advancing it concurrently, which is
    /// safe (distinct atomics; benign message-window race).
    #[inline]
    pub(crate) unsafe fn reset_tail_to_head(&self) {
        let head = (*self.head_ptr).load(Ordering::Acquire);
        (*self.tail_ptr).store(head, Ordering::Release);
    }
}

/// Claim the lowest free slot in `[0, limit)` from an endpoint bitmask via CAS.
///
/// Returns `None` when all `limit` slots are set (every endpoint simultaneously
/// live). The CAS loop makes concurrent claims pick distinct slots. Mirrors the
/// intra-process `super::fanout::claim_slot`.
///
/// Test-only: the production cross-process path (`claim_endpoint_locked`) sets a
/// SPECIFIC flocked slot's bit rather than the lowest-free one, so it does not use
/// this helper. It backs the in-process bitmask/routing tests.
#[cfg(test)]
#[inline]
fn claim_bit(mask: &AtomicU64, limit: usize) -> Option<usize> {
    let mut cur = mask.load(Ordering::Relaxed);
    loop {
        let free = (!cur).trailing_zeros() as usize;
        if free >= limit {
            return None; // all slots live
        }
        let bit = 1u64 << free;
        match mask.compare_exchange_weak(cur, cur | bit, Ordering::AcqRel, Ordering::Relaxed) {
            Ok(_) => return Some(free),
            Err(actual) => cur = actual, // lost the race — retry with the new mask
        }
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
// (recv_cursors round-robin state.)
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
        let channel_stride =
            (CHANNEL_HEADER_SIZE + cap as usize * slot_size + cap as usize * 8) as u64;
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
        meta.pub_active = AtomicU64::new(0);
        meta.sub_active = AtomicU64::new(0);
        for i in 0..MAX_FANOUT_ENDPOINTS {
            meta.pub_owner_pids[i] = AtomicU32::new(0);
            meta.sub_owner_pids[i] = AtomicU32::new(0);
        }

        // Zero-init all channel head/tail atomics and per-slot version stamps.
        // Version 0 == done(pos 0); head starts at 0 so no position is read
        // before the producer writes (and re-stamps) it — this also makes a
        // reused (non-fresh) region safe.
        let channels_base = shm_base.add(FANOUT_CHANNELS_BASE);
        let num_channels = max_pubs as usize * max_subs as usize;
        let ver_offset = CHANNEL_HEADER_SIZE + cap as usize * slot_size;
        for i in 0..num_channels {
            let ch_base = channels_base.add(i * channel_stride as usize);
            // head = 0
            (ch_base as *mut AtomicU64).write(AtomicU64::new(0));
            // tail = 0
            (ch_base.add(64) as *mut AtomicU64).write(AtomicU64::new(0));
            // versions[0..cap] = 0
            let ver_base = ch_base.add(ver_offset) as *mut AtomicU64;
            for s in 0..cap as usize {
                ver_base.add(s).write(AtomicU64::new(0));
            }
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
    ///
    /// Returns `None` if the region carries an incompatible (older/foreign) layout
    /// version, so the caller can reject-and-rebuild (fall back to SpscShm) instead
    /// of reinterpreting a stale region with the new strides.
    pub(crate) unsafe fn attach(shm_base: *mut u8, is_pod: bool, type_size: usize) -> Option<Self> {
        let meta = &*(shm_base.add(FANOUT_META_OFFSET) as *const FanoutShmMeta);

        // Spin-wait for the owner to finish initialization (it writes `magic` LAST).
        // The magic lives in a cross-process mapping, so read it volatile.
        //
        // Version discipline (COMM-H1): a region already carrying a DIFFERENT
        // non-zero magic is an older/incompatible layout (e.g. the v2 128-byte
        // header) or foreign data. Reject it immediately — reading it with the v3
        // strides would be UB, and spinning would hang forever (no v3 owner will
        // ever write the new magic into an already-initialized old region). A
        // `magic == 0` region is a fresh, zero-filled region whose owner is still
        // initializing → keep spinning (unchanged from prior behavior).
        let mut spins = 0u32;
        loop {
            let m = std::ptr::read_volatile(std::ptr::addr_of!(meta.magic));
            if m == FANOUT_MAGIC {
                break;
            }
            if m != 0 {
                return None; // stale/incompatible layout — reject, do not misread
            }
            std::hint::spin_loop();
            spins += 1;
            if spins > 1_000_000 {
                // Yield after extensive spinning
                std::thread::yield_now();
                spins = 0;
            }
        }
        std::sync::atomic::fence(Ordering::Acquire);

        Some(Self::build_views(shm_base, is_pod, type_size))
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
            is_pod,
        }
    }

    /// Register a new publisher, claiming a free endpoint slot under an exclusive
    /// `flock` (COMM-H1 cross-process crash-liveness). Returns `(id, lock)`; the
    /// caller MUST keep `lock` alive for the endpoint's lifetime — it is the
    /// OS-level proof the slot is live. The OS releases it on process death, which
    /// is what lets a peer later reclaim a crashed owner's slot. Returns `None` when
    /// every slot is held by a live process (a genuine capacity limit, NEVER the
    /// pre-fix panic).
    ///
    /// `topic_name` derives the per-endpoint lock-file path; every process on the
    /// topic derives the SAME path, so the flock is a machine-wide claim.
    pub(crate) fn register_publisher_locked(&self, topic_name: &str) -> Option<(usize, FileLock)> {
        let meta = unsafe { &*self.meta_ptr };
        self.claim_endpoint_locked(
            topic_name,
            &meta.pub_active,
            &meta.pub_owner_pids,
            self.max_publishers,
            EndpointKind::Publisher,
        )
    }

    /// Register a new subscriber (symmetric to `register_publisher_locked`). On
    /// claim, the slot's channels are reset (tail→head) so a reclaimed slot starts
    /// fresh instead of inheriting a prior/crashed owner's unread backlog.
    pub(crate) fn register_subscriber_locked(&self, topic_name: &str) -> Option<(usize, FileLock)> {
        let meta = unsafe { &*self.meta_ptr };
        let (id, lock) = self.claim_endpoint_locked(
            topic_name,
            &meta.sub_active,
            &meta.sub_owner_pids,
            self.max_subscribers,
            EndpointKind::Subscriber,
        )?;
        // A publisher never resets its channels (their tails are consumer-owned and
        // may belong to LIVE subscribers — never write them). A subscriber skips any
        // stale backlog on its freshly-claimed slot. Mirrors the intra FanoutRing.
        self.reset_subscriber_channels(id);
        Some((id, lock))
    }

    /// The core flock claim/reclaim protocol (COMM-H1).
    ///
    /// # #1 INVARIANT (a wrong reclaim = silent cross-process UB, strictly worse
    /// than the pre-fix panic)
    ///
    /// A slot is reclaimed from a prior owner ONLY on PROOF of abandonment: we hold
    /// its exclusive `flock`, which the OS releases solely on `LOCK_UN` (clean drop)
    /// or process death. Acquiring it therefore proves no live process holds the
    /// slot. The four cases after `try_exclusive`:
    ///
    /// - `Err(_)` / `Ok(None)` → cannot prove abandonment (IO error, or a LIVE
    ///   process holds it) → skip. Never reclaim.
    /// - `Ok(Some(lock))` + `owner_pid == my_pid && bit set` → a LIVE in-process
    ///   sibling that the OS let us re-`flock` (same-process permissiveness) →
    ///   drop the lock and skip. Never reclaim a live same-process endpoint.
    /// - `Ok(Some(lock))` otherwise (bit clear = free, OR owner is a dead/other
    ///   process) → the slot is ours: set the bit, record our PID, return the lock.
    ///
    /// All slots skipped → `None` (a genuine "all endpoints live" capacity limit —
    /// the graceful-degradation worst case, never a panic and never a mis-reclaim).
    ///
    /// RESTS-ON-DESIGN: the real crash path — a peer SIGKILLed, the OS releasing its
    /// `flock`, another process then reclaiming — relies on the documented `flock`
    /// OS contract (release on process death). It is NOT integration-testable in
    /// this headless sandbox (cross-process SHM is env-broken here). It is exercised
    /// in-process via a *simulated-dead* lock file (unlocked + foreign PID). The
    /// same-process guard and the free/reclaim bit logic ARE verified in-process.
    fn claim_endpoint_locked(
        &self,
        topic_name: &str,
        active: &AtomicU64,
        owner_pids: &[AtomicU32; MAX_FANOUT_ENDPOINTS],
        max: usize,
        kind: EndpointKind,
    ) -> Option<(usize, FileLock)> {
        let my_pid = std::process::id();
        for i in 0..max {
            // Derive + create the per-endpoint lock file. Any IO error → we cannot
            // prove anything about this slot → skip it (fail-safe, never reclaim).
            let path = match self.endpoint_lock_path(topic_name, kind, i) {
                Some(p) => p,
                None => continue,
            };
            let file = match horus_sys::fs::open_private(&path) {
                Ok(f) => f,
                Err(_) => continue,
            };
            // The flock is the claim AUTHORITY:
            //   Err(_)      → real error, cannot prove → skip.
            //   Ok(None)    → a LIVE process holds it → skip (never reclaim live).
            //   Ok(Some(l)) → we hold it: the prior owner (if any) is provably gone.
            let lock = match FileLock::try_exclusive(&file) {
                Ok(Some(lock)) => lock,
                Ok(None) => continue,
                Err(_) => continue,
            };
            // Same-process guard: on some OSes `flock` on a second fd within the SAME
            // process succeeds, so holding the lock does NOT prove a same-process
            // sibling is dead. If the recorded owner is THIS process AND the slot is
            // still active, it is a LIVE in-process endpoint → do NOT reclaim.
            let owner_pid = owner_pids[i].load(Ordering::Relaxed);
            let bit_set = active.load(Ordering::Acquire) & (1u64 << i) != 0;
            if owner_pid == my_pid && bit_set {
                drop(lock); // release our re-lock; leave the live sibling untouched
                continue;
            }
            // Claim: the slot is free (bit clear) or abandoned by a dead/other-process
            // owner (we hold its flock). Set the active bit, record our PID, and hand
            // the HELD lock to the caller. Order: the flock (already held) is the
            // ownership truth; the bit is derived state, set only AFTER the flock.
            active.fetch_or(1u64 << i, Ordering::AcqRel);
            owner_pids[i].store(my_pid, Ordering::Release);
            return Some((i, lock));
        }
        None
    }

    /// Deterministic per-endpoint lock-file path, mirroring the SHM region naming
    /// (`{topic}_fanout`) determinism so every process on the topic agrees on it:
    /// `{shm_base_dir}/fanout_locks/horus.{sanitized_topic}.fanout.{pub|sub}.{slot}.lock`.
    /// `shm_base_dir()` is namespace-aware (`HORUS_NAMESPACE`), matching the SHM
    /// region's isolation. A leftover lock file is harmless — content is irrelevant,
    /// only the `flock` state matters. Returns `None` if the shared lock directory
    /// can't be created (→ caller skips the slot, fail-safe).
    fn endpoint_lock_path(
        &self,
        topic_name: &str,
        kind: EndpointKind,
        slot: usize,
    ) -> Option<PathBuf> {
        let dir = horus_sys::shm::shm_base_dir().join("fanout_locks");
        if horus_sys::fs::create_dir_secure(&dir).is_err() {
            return None;
        }
        // Flatten the topic name into a safe filename component (keep alnum/_/./-;
        // map path separators & anything else to '_'). Deterministic across procs.
        let mut sanitized = String::with_capacity(topic_name.len());
        for c in topic_name.chars() {
            if c.is_ascii_alphanumeric() || c == '_' || c == '.' || c == '-' {
                sanitized.push(c);
            } else {
                sanitized.push('_');
            }
        }
        Some(dir.join(format!(
            "horus.{}.fanout.{}.{}.lock",
            sanitized,
            kind.tag(),
            slot
        )))
    }

    /// Reset every publisher-channel feeding subscriber `id` to skip whatever a
    /// prior (possibly crash-abandoned) owner left buffered. Producer-owned `head`
    /// is only READ; the consumer-owned `tail` store is uncontended because this
    /// slot was just claimed (no live consumer on `id`).
    #[inline]
    fn reset_subscriber_channels(&self, id: usize) {
        for pub_id in 0..self.max_publishers {
            let ch_idx = pub_id * self.max_subscribers + id;
            // SAFETY: `ch_idx` is in range (`pub_id < max_publishers`,
            // `id < max_subscribers`); the channel view points into the live mmap.
            unsafe { self.channels[ch_idx].reset_tail_to_head() };
        }
    }

    /// Test-only bitmask claim (no flock) — exercises the reusable-slot bitmask +
    /// hot-path routing logic in-process without touching the filesystem.
    /// Production uses `register_publisher_locked`.
    #[cfg(test)]
    pub(crate) fn register_publisher(&self) -> Option<usize> {
        let meta = unsafe { &*self.meta_ptr };
        claim_bit(&meta.pub_active, self.max_publishers)
    }

    /// Test-only bitmask claim (no flock). Production uses
    /// `register_subscriber_locked`.
    #[cfg(test)]
    pub(crate) fn register_subscriber(&self) -> Option<usize> {
        let meta = unsafe { &*self.meta_ptr };
        let id = claim_bit(&meta.sub_active, self.max_subscribers)?;
        self.reset_subscriber_channels(id);
        Some(id)
    }

    /// Send a POD message from publisher `pub_id` to ALL subscribers.
    ///
    /// # Safety
    ///
    /// `T` must be POD. Only one thread per process should use a given `pub_id`.
    #[inline(always)]
    pub(crate) unsafe fn send_pod<T>(&self, msg: &T, pub_id: usize) -> bool {
        let meta = &*self.meta_ptr;
        let subs = meta.sub_active.load(Ordering::Relaxed);
        if subs == 0 {
            return true; // No subscribers — silent drop
        }

        // Drop-oldest fan-out to every LIVE subscriber slot (holes from reused or
        // crash-abandoned endpoints are skipped via the active bitmask). Never fails.
        for sub_id in 0..self.max_subscribers {
            if subs & (1u64 << sub_id) != 0 {
                let ch_idx = pub_id * self.max_subscribers + sub_id;
                self.channels[ch_idx].try_send_pod(msg);
            }
        }
        true
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
        let pubs = meta.pub_active.load(Ordering::Relaxed);
        if pubs == 0 {
            return None;
        }

        // Round-robin over LIVE publisher slots (holes from reused/crashed endpoints
        // are skipped), starting from the last cursor position, bounded by the fixed
        // matrix width so a stale cursor can never index out of range.
        let max_pubs = self.max_publishers;
        let start = self.recv_cursors[sub_id].get() % max_pubs;

        for offset in 0..max_pubs {
            let pub_id = (start + offset) % max_pubs;
            if pubs & (1u64 << pub_id) == 0 {
                continue; // inactive publisher slot
            }
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
        let subs = meta.sub_active.load(Ordering::Relaxed);
        if subs == 0 {
            return true;
        }

        // Drop-oldest fan-out to every LIVE subscriber slot (holes skipped): never
        // blocks. `try_send_serde` returns false only if a message is too large for
        // a slot — surface that as the aggregate.
        let mut all_ok = true;
        for sub_id in 0..self.max_subscribers {
            if subs & (1u64 << sub_id) != 0 {
                let ch_idx = pub_id * self.max_subscribers + sub_id;
                if !self.channels[ch_idx].try_send_serde(bytes) {
                    all_ok = false;
                }
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
        let pubs = meta.pub_active.load(Ordering::Relaxed);
        if pubs == 0 {
            return None;
        }

        let max_pubs = self.max_publishers;
        let start = self.recv_cursors[sub_id].get() % max_pubs;

        for offset in 0..max_pubs {
            let pub_id = (start + offset) % max_pubs;
            if pubs & (1u64 << pub_id) == 0 {
                continue; // inactive publisher slot
            }
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

    /// Get the number of currently-live publishers (popcount of the active mask).
    #[cfg(test)]
    #[inline]
    pub(crate) fn num_publishers(&self) -> usize {
        let meta = unsafe { &*self.meta_ptr };
        meta.pub_active.load(Ordering::Relaxed).count_ones() as usize
    }

    /// Get the number of currently-live subscribers (popcount of the active mask).
    #[cfg(test)]
    #[inline]
    pub(crate) fn num_subscribers(&self) -> usize {
        let meta = unsafe { &*self.meta_ptr };
        meta.sub_active.load(Ordering::Relaxed).count_ones() as usize
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
        // stride = head/tail cache lines + data slots + per-slot version array.
        let channel_stride = CHANNEL_HEADER_SIZE + cap * slot_size + cap * 8;
        let num_channels = MAX_FANOUT_ENDPOINTS * MAX_FANOUT_ENDPOINTS;

        FANOUT_CHANNELS_BASE + num_channels * channel_stride
    }

    /// Total pending messages across all channels for a subscriber.
    #[cfg(test)]
    pub(crate) fn pending_count_for_sub(&self, sub_id: usize) -> u64 {
        let meta = unsafe { &*self.meta_ptr };
        let pubs = meta.pub_active.load(Ordering::Relaxed);
        let mut total = 0u64;
        for pub_id in 0..self.max_publishers {
            if pubs & (1u64 << pub_id) == 0 {
                continue;
            }
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
        assert_eq!(std::mem::size_of::<FanoutShmMeta>(), 256);
        assert_eq!(std::mem::align_of::<FanoutShmMeta>(), 64);
    }

    #[test]
    fn required_file_size_u64() {
        let size = ShmFanoutRing::required_file_size(8, true, 64);
        // 4224 base + 256 channels × (128 header + 64 slots × 8B data + 64 × 8B versions)
        let expected = FANOUT_CHANNELS_BASE + 256 * (128 + 64 * 8 + 64 * 8);
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
            assert_eq!(ring.register_publisher(), Some(0));
            assert_eq!(ring.register_publisher(), Some(1));
            assert_eq!(ring.register_subscriber(), Some(0));
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
            let pub_id = ring.register_publisher().unwrap();
            let sub_id = ring.register_subscriber().unwrap();

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
            let p0 = ring.register_publisher().unwrap();
            let p1 = ring.register_publisher().unwrap();
            let s0 = ring.register_subscriber().unwrap();
            let s1 = ring.register_subscriber().unwrap();

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
            let pub_id = ring.register_publisher().unwrap();
            let sub_id = ring.register_subscriber().unwrap();

            let data = b"hello world";
            assert!(ring.send_serde(data, pub_id));

            let recv = ring.recv_serde(sub_id);
            assert_eq!(recv.as_deref(), Some(&b"hello world"[..]));

            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn capacity_full_drops_oldest() {
        let (ptr, layout) = alloc_shm_sim(8, true, 16);
        unsafe {
            let ring = ShmFanoutRing::init_owner(ptr, 8, true, 16);
            let pub_id = ring.register_publisher().unwrap();
            let sub_id = ring.register_subscriber().unwrap();

            // Drop-oldest: sends never fail. Push 20 into a 16-slot ring; the
            // oldest 4 (0..4) are overwritten by the newest 4 (16..20).
            for i in 0u64..20 {
                assert!(ring.send_pod(&i, pub_id), "send {} never fails", i);
            }
            let mut got = Vec::new();
            while let Some(v) = ring.recv_pod::<u64>(sub_id) {
                got.push(v);
            }
            assert_eq!(got.len(), 16, "ring retains exactly capacity newest msgs");
            let expected: Vec<u64> = (4u64..20).collect();
            assert_eq!(got, expected, "oldest dropped, order preserved");

            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn round_robin_fairness() {
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = ShmFanoutRing::init_owner(ptr, 8, true, 64);
            let p0 = ring.register_publisher().unwrap();
            let p1 = ring.register_publisher().unwrap();
            let sub = ring.register_subscriber().unwrap();

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
            let joiner = ShmFanoutRing::attach(ptr, true, 8).unwrap();
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
            let p0 = ring.register_publisher().unwrap();
            let sub = ring.register_subscriber().unwrap();

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
            let p = ring.register_publisher().unwrap();
            let s = ring.register_subscriber().unwrap();

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
            let p = ring.register_publisher().unwrap();
            let s = ring.register_subscriber().unwrap();

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
            let p = ring.register_publisher().unwrap();
            let s = ring.register_subscriber().unwrap();

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
        let p0 = unsafe { (*ring).register_publisher().unwrap() };
        let p1 = unsafe { (*ring).register_publisher().unwrap() };
        let s0 = unsafe { (*ring).register_subscriber().unwrap() };
        let s1 = unsafe { (*ring).register_subscriber().unwrap() };

        let ring_addr = ring as usize;

        let b = barrier.clone();
        let h_p0 = std::thread::spawn(move || {
            let ring = unsafe { &*(ring_addr as *const ShmFanoutRing) };
            b.wait();
            for i in 0..msgs as u64 {
                let val = i * 2; // even
                // Drop-oldest send never fails — never blocks on a slow consumer.
                let _ = unsafe { ring.send_pod(&val, p0) };
            }
        });

        let b = barrier.clone();
        let h_p1 = std::thread::spawn(move || {
            let ring = unsafe { &*(ring_addr as *const ShmFanoutRing) };
            b.wait();
            for i in 0..msgs as u64 {
                let val = i * 2 + 1; // odd
                let _ = unsafe { ring.send_pod(&val, p1) };
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

        // Drop-oldest (latest-wins): a slow subscriber MAY miss messages when a
        // fast producer laps it, so delivery is not exhaustive. The integrity
        // invariants are that every subscriber (a) never over-delivers, (b) makes
        // progress, and — crucially — (c) never sees a duplicate or an
        // out-of-order value within a publisher's stream.
        for (name, data) in [("s0", &s0_data), ("s1", &s1_data)] {
            assert!(data.len() <= msgs * 2, "{name} over-delivered {}", data.len());
            assert!(!data.is_empty(), "{name} received nothing");

            let even: Vec<u64> = data.iter().filter(|v| *v % 2 == 0).copied().collect();
            let odd: Vec<u64> = data.iter().filter(|v| *v % 2 == 1).copied().collect();
            // Strictly increasing ⇒ FIFO order preserved AND no duplicates.
            for w in even.windows(2) {
                assert!(w[0] < w[1], "{name} even reorder/dup: {} >= {}", w[0], w[1]);
            }
            for w in odd.windows(2) {
                assert!(w[0] < w[1], "{name} odd reorder/dup: {} >= {}", w[0], w[1]);
            }
        }

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
            let p = ring.register_publisher().unwrap();
            let s = ring.register_subscriber().unwrap();

            for cycle in 0u64..5 {
                // Fill exactly to capacity (no overwrite at avail == capacity).
                for i in 0u64..16 {
                    assert!(ring.send_pod(&(cycle * 100 + i), p));
                }

                // Drain and verify — all 16 come back in FIFO order.
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
                pubs.push(ring.register_publisher().unwrap());
                subs.push(ring.register_subscriber().unwrap());
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
            let s = ring.register_subscriber().unwrap();
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
                ring.register_publisher().unwrap()
            }));
        }
        for _ in 0..4 {
            let b = barrier.clone();
            handles.push(std::thread::spawn(move || {
                let ring = unsafe { &*(ring_addr as *const ShmFanoutRing) };
                b.wait();
                ring.register_subscriber().unwrap()
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
