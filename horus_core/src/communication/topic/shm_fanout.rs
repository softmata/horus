//! SHM FanoutRing — cross-process contention-free MPMC via mmap'd SPSC matrix.
//!
//! A contention-free fan-out design backed by shared memory. Each
//! (publisher, subscriber) pair gets its own SHM-backed SPSC channel with
//! cache-line-separated head/tail atomics.
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

/// Largest POD slot the fanout matrix will carry.
///
/// The matrix is `MAX_FANOUT_ENDPOINTS²` = 256 channels, so every byte of slot
/// costs `256 × capacity` bytes of mapping: at the 16-slot minimum capacity a
/// 4096-byte slot is already a 16.8 MB region per topic, and doubling the slot
/// doubles that. The cap is what stops a large message type from asking the
/// kernel for gigabytes of `/dev/shm` per topic.
///
/// A POD type above it is REFUSED, not clamped — see
/// [`ShmFanoutRing::compute_slot_size`].
const MAX_POD_SLOT_SIZE: usize = 4096;

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
    /// - `size_of::<T>()` must fit `slot_size`. This memcpy is the one with no
    ///   length check of its own, so the bound lives at both construction sites
    ///   instead: `compute_slot_size` refuses a POD type above the slot cap, and
    ///   `validate_meta` refuses an attached region whose slots are too small.
    #[inline(always)]
    pub(crate) unsafe fn try_send_pod<T>(&self, msg: &T) {
        debug_assert!(
            std::mem::size_of::<T>() <= self.slot_size,
            "try_send_pod would write {} bytes into a {}-byte slot",
            std::mem::size_of::<T>(),
            self.slot_size
        );
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
    pub(crate) unsafe fn try_recv_pod<T>(&self, skipped: &mut u64) -> Option<T> {
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
            skipped,
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
    pub(crate) unsafe fn try_recv_serde(&self, skipped: &mut u64) -> Option<Vec<u8>> {
        let head = &*self.head_ptr;
        let tail = &*self.tail_ptr;
        // `saturating_sub`, not `-`: `slot_size` reaches this view from the SHM
        // meta block. `attach` now refuses a `slot_size` below MIN_SLOT_SIZE, but
        // the subtraction is the one place where being wrong is unrecoverable —
        // with `overflow-checks` off a slot_size of 0..3 wraps `max_payload` to
        // ~usize::MAX, which turns the length bound below into a no-op and lets a
        // slot's own 32-bit length word drive a 4 GiB out-of-bounds read.
        let max_payload = self.slot_size.saturating_sub(4);
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
            skipped,
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
/// live). The CAS loop makes concurrent claims pick distinct slots.
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
    /// Returns `None` — before touching the region — when this layout cannot
    /// carry the type (see [`Self::compute_slot_size`]), which is the same answer
    /// `attach` gives for a region it refuses and which `init_shm_backend`
    /// already handles by falling back to SpscShm. The owner path must not build
    /// a geometry [`Self::validate_meta`] would reject on the attach side.
    ///
    /// # Safety
    ///
    /// `shm_base` must point to a valid mmap'd region of at least
    /// `Self::required_file_size(type_size, is_pod, channel_capacity)` bytes.
    pub(crate) unsafe fn init_owner(
        shm_base: *mut u8,
        type_size: usize,
        is_pod: bool,
        channel_capacity: u32,
    ) -> Option<Self> {
        let slot_size = Self::compute_slot_size(type_size, is_pod)?;
        let cap = Self::compute_capacity(channel_capacity as usize)?;
        let max_pubs = MAX_FANOUT_ENDPOINTS as u32;
        let max_subs = MAX_FANOUT_ENDPOINTS as u32;
        // Same helper `required_file_size` uses, so the stride written into the
        // meta block and the stride the region was sized for cannot disagree.
        let channel_stride = Self::channel_stride(cap, slot_size)? as u64;
        let total_size = Self::required_file_size(type_size, is_pod, cap as usize)?;

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

        Some(Self::build_views(shm_base, is_pod, type_size))
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
    /// version, or dimensions that do not describe a matrix inside `region_len`,
    /// so the caller can reject-and-rebuild (fall back to SpscShm) instead of
    /// reinterpreting a stale or hostile region with the new strides.
    ///
    /// `region_len` is the length of the mapping `shm_base` points at. It is the
    /// only thing that bounds the dimensions in the region's own header, all of
    /// which another process wrote — see [`Self::validate_meta`].
    pub(crate) unsafe fn attach(
        shm_base: *mut u8,
        region_len: usize,
        is_pod: bool,
        type_size: usize,
    ) -> Option<Self> {
        // Before the meta reference is even formed: a region too small to hold
        // the meta block would make `&*` itself out of bounds.
        if region_len < FANOUT_META_OFFSET + FANOUT_META_SIZE {
            return None;
        }
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
        //
        // The wait is bounded. A `magic == 0` region is normally an owner that is
        // milliseconds from finishing, but it is also what a process that died
        // between creating the region and stamping the magic leaves behind —
        // a SIGKILL, an OOM kill, a panic during init. That region never becomes
        // valid, and the loop had no exit for it: every later node opening the
        // same topic span forever, burning a core, with no timeout, no error and
        // no log line. Restarting the robot did not help, because the region
        // outlives the process that made it.
        //
        // Giving up returns `None`, which is the same answer the incompatible
        // -layout branch gives and which the caller already handles by rebuilding
        // on SpscShm. A slow but live owner is nowhere near this bound: it writes
        // the magic immediately after mapping.
        const ATTACH_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(2);
        /// Spins between clock reads. `Instant::now` is ~20ns against a ~1ns
        /// spin hint, so checking every iteration would dominate the wait it is
        /// meant to measure.
        const SPINS_PER_CLOCK_CHECK: u32 = 4096;

        let started = std::time::Instant::now();
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
            if spins.is_multiple_of(SPINS_PER_CLOCK_CHECK) && started.elapsed() >= ATTACH_TIMEOUT {
                // The owner is never going to finish. Let the caller fall back.
                return None;
            }
            if spins > 1_000_000 {
                // Yield after extensive spinning — but only with time left. The
                // enclosing loop already re-checks `ATTACH_TIMEOUT` every
                // `SPINS_PER_CLOCK_CHECK` spins, so this only tightens the
                // overshoot of the last yield; it is here so that no unguarded
                // `yield_now` remains on a deadline-bounded path.
                if started.elapsed() >= ATTACH_TIMEOUT {
                    return None;
                }
                std::thread::yield_now();
                spins = 0;
            }
        }
        std::sync::atomic::fence(Ordering::Acquire);

        // The magic proves the layout VERSION, not the numbers. Everything
        // `build_views` is about to turn into a pointer is still whatever the
        // writing process stored.
        if !Self::validate_meta(meta, region_len, is_pod, type_size) {
            return None;
        }

        Some(Self::build_views(shm_base, is_pod, type_size))
    }

    /// Check the dimensions an existing `FanoutShmMeta` declares before any
    /// pointer is derived from them.
    ///
    /// Every field here was written by another process into a region on the
    /// shared `/dev/shm`, so on a robot where one node is compromised — or
    /// merely buggy — all of them are attacker-controlled. `build_views` used
    /// them raw: `max_publishers * max_subscribers` channel views at
    /// `channels_base + i * channel_stride`, and `ShmSpscChannel::from_raw`
    /// derives `data_ptr`, `ver_ptr` and the index mask from `slot_size`,
    /// `channel_capacity` and `capacity_mask`. A stride, a capacity or a mask
    /// bigger than the region therefore placed live channels OUTSIDE the
    /// mapping, and the first `try_send_pod` wrote through one — an
    /// out-of-bounds write at an offset the writer picks. `max_publishers`
    /// above `MAX_FANOUT_ENDPOINTS` was equally unchecked, and endpoint ids
    /// derived from it index the fixed-size `recv_cursors` and `*_owner_pids`
    /// arrays.
    ///
    /// `init_owner` writes exactly ONE geometry for a given capacity, so this
    /// recomputes that geometry and demands equality rather than mere
    /// plausibility: `channel_stride` must be the stride the layout implies
    /// (a smaller one overlaps the next channel's slots, a larger one walks the
    /// matrix off the end), and the whole matrix must fit inside `region_len`.
    /// Anything else is refused, and `init_shm_backend` falls back to SpscShm
    /// exactly as it already does for an incompatible layout version.
    fn validate_meta(
        meta: &FanoutShmMeta,
        region_len: usize,
        is_pod: bool,
        type_size: usize,
    ) -> bool {
        let max_pubs = meta.max_publishers as usize;
        let max_subs = meta.max_subscribers as usize;
        if max_pubs == 0 || max_pubs > MAX_FANOUT_ENDPOINTS {
            return false;
        }
        if max_subs == 0 || max_subs > MAX_FANOUT_ENDPOINTS {
            return false;
        }

        let capacity = meta.channel_capacity as usize;
        if capacity == 0 || !capacity.is_power_of_two() {
            return false;
        }
        // `seqlock_consume`'s contract is `capacity == mask + 1`; its slot index
        // is `pos & mask`, which is only inside the ring when that holds.
        if meta.capacity_mask as usize != capacity - 1 {
            return false;
        }

        let slot_size = meta.slot_size as usize;
        if slot_size < MIN_SLOT_SIZE {
            return false;
        }
        // `try_send_pod` memcpys `size_of::<T>()` bytes into a slot with no
        // length check of its own — the slot has to be at least that big.
        if is_pod && slot_size < type_size {
            return false;
        }

        // The stride `init_owner` would have written for this capacity and slot
        // size: head/tail cache lines + `capacity` data slots + `capacity`
        // version stamps. Checked throughout — with `overflow-checks` off, a
        // wrapped product comes back small and every containment test below it
        // passes for a matrix that does not fit.
        let Some(seq_bytes) = capacity.checked_mul(8) else {
            return false;
        };
        let Some(data_bytes) = capacity.checked_mul(slot_size) else {
            return false;
        };
        let Some(stride) = data_bytes
            .checked_add(seq_bytes)
            .and_then(|body| body.checked_add(CHANNEL_HEADER_SIZE))
        else {
            return false;
        };
        if meta.channel_stride != stride as u64 {
            return false;
        }

        let Some(total) = max_pubs
            .checked_mul(max_subs)
            .and_then(|channels| channels.checked_mul(stride))
            .and_then(|matrix| matrix.checked_add(FANOUT_CHANNELS_BASE))
        else {
            return false;
        };
        total <= region_len
    }

    /// Build channel views from an initialized SHM region.
    ///
    /// # Safety
    ///
    /// The meta block's dimensions must already describe a matrix that fits the
    /// mapping: either this process wrote them (`init_owner`) or they passed
    /// [`Self::validate_meta`] (`attach`). Every pointer below is derived from
    /// them without a further bound.
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

    /// Release this instance's publisher slot on CLEAN drop (COMM-H1): clear the
    /// active bit, then the owner PID. The caller drops the held `FileLock` AFTER
    /// this (releasing the flock), so the ordering is: bit clear → PID clear →
    /// flock release. The flock stays held throughout, so no peer can reclaim
    /// mid-teardown; once released, the cleared bit makes the slot immediately
    /// reusable — crucially by THIS process too (the same-process guard would
    /// otherwise refuse to reclaim our own still-marked slot). A CRASH runs none of
    /// this, but the OS releases the flock and a peer reclaims via the dead-owner
    /// path (the stale bit/PID self-heal on that reclaim).
    pub(crate) fn deregister_publisher(&self, id: usize) {
        let meta = unsafe { &*self.meta_ptr };
        meta.pub_active.fetch_and(!(1u64 << id), Ordering::AcqRel);
        meta.pub_owner_pids[id].store(0, Ordering::Relaxed);
    }

    /// Release this instance's subscriber slot on clean drop (symmetric).
    pub(crate) fn deregister_subscriber(&self, id: usize) {
        let meta = unsafe { &*self.meta_ptr };
        meta.sub_active.fetch_and(!(1u64 << id), Ordering::AcqRel);
        meta.sub_owner_pids[id].store(0, Ordering::Relaxed);
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
    pub(crate) unsafe fn recv_pod<T>(&self, sub_id: usize, skipped: &mut u64) -> Option<T> {
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
            if let Some(msg) = self.channels[ch_idx].try_recv_pod::<T>(skipped) {
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
    pub(crate) unsafe fn recv_serde(&self, sub_id: usize, skipped: &mut u64) -> Option<Vec<u8>> {
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
            if let Some(data) = self.channels[ch_idx].try_recv_serde(skipped) {
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

    /// Compute slot size from type size, or `None` when this layout cannot carry
    /// the type at all (a POD `T` above [`MAX_POD_SLOT_SIZE`]).
    ///
    /// Refusing is what keeps the owner and attach paths in agreement. This used
    /// to `.min(MAX_POD_SLOT_SIZE)`, so `init_owner` happily wrote a POD geometry
    /// whose slots were SMALLER than the message — the exact geometry
    /// `validate_meta` refuses on the attach side (`is_pod && slot_size <
    /// type_size`), because `try_send_pod` memcpys `size_of::<T>()` bytes into a
    /// slot with no length check of its own. Every send through such a ring wrote
    /// past its slot, over the neighbouring slots, the version stamps and — from
    /// the last channel — the end of the mapping.
    ///
    /// `None` surfaces as a failed fanout init in `init_shm_backend`, which
    /// already falls back to SpscShm the same way it does for a rejected `attach`.
    fn compute_slot_size(type_size: usize, is_pod: bool) -> Option<usize> {
        if is_pod {
            // POD: raw T with minimum alignment
            let slot = type_size.max(MIN_SLOT_SIZE).checked_next_power_of_two()?;
            (slot <= MAX_POD_SLOT_SIZE).then_some(slot)
        } else {
            // Serde: [len: u32, data: [u8; ...]] — use 8KB default
            // (`try_send_serde` bounds each payload against the slot itself.)
            Some(8192)
        }
    }

    /// Round a requested capacity up to the power-of-two slot count the layout
    /// uses, or `None` when the result cannot be named exactly.
    ///
    /// `meta.channel_capacity` is a `u32`, so a capacity that does not fit one
    /// has to be refused rather than cast. `init_owner` used to write
    /// `(channel_capacity as usize).next_power_of_two().max(16) as u32`: on a
    /// 64-bit host any capacity above 2^31 rounds to 2^32 and that cast lands on
    /// `cap == 0`, from which `meta.capacity_mask = cap - 1` hands every channel
    /// a mask of `u32::MAX`. [`Self::build_views`] derives every slot pointer
    /// from those two numbers with no further bound — its safety contract is
    /// that the owner wrote a matrix that fits — so the only thing between that
    /// geometry and a live ring was `ShmRegion::new` happening to refuse the
    /// petabyte-scale mapping first.
    fn compute_capacity(channel_capacity: usize) -> Option<u32> {
        let cap = channel_capacity.checked_next_power_of_two()?.max(16);
        u32::try_from(cap).ok()
    }

    /// Per-channel stride: head/tail cache lines + `cap` data slots + `cap`
    /// version stamps. `None` when that does not fit a `usize`.
    ///
    /// This is the owner-side twin of the reconstruction in
    /// [`Self::validate_meta`], and it is checked for the same reason that one
    /// is. `capacity` is caller-supplied and unbounded (`Topic::with_capacity`
    /// rejects only 0), the release profile this ships with has no
    /// `overflow-checks`, and `usize` is 32 bits on the
    /// `armv7-unknown-linux-gnueabihf` controllers `horus deploy --arch armv7`
    /// targets. There, capacity 524288 over an 8192-byte serde slot wraps
    /// `cap * slot_size` to 0: the owner asks for a 1 GiB region instead of the
    /// honest 1025 GiB — which would simply have failed to allocate and fallen
    /// back — and then writes `channel_capacity = 524288, slot_size = 8192,
    /// channel_stride = 4194432` into the meta block. Slot 512 onward is already
    /// outside its channel and the ring's tail is ~4 GiB past the end of the
    /// mapping. `validate_meta` refuses exactly that meta block on the attach
    /// side, so the wrap put the owner and attach paths back out of agreement —
    /// the same disagreement the POD slot cap closes, reached through the other
    /// input.
    fn channel_stride(cap: u32, slot_size: usize) -> Option<usize> {
        let cap = cap as usize;
        let data_bytes = cap.checked_mul(slot_size)?;
        let seq_bytes = cap.checked_mul(8)?;
        data_bytes
            .checked_add(seq_bytes)?
            .checked_add(CHANNEL_HEADER_SIZE)
    }

    /// Calculate the total SHM file size needed for a fanout layout, or `None`
    /// when [`Self::compute_slot_size`] refuses the type,
    /// [`Self::compute_capacity`] refuses the capacity, or the matrix does not
    /// fit a `usize`.
    pub(crate) fn required_file_size(
        type_size: usize,
        is_pod: bool,
        channel_capacity: usize,
    ) -> Option<usize> {
        let slot_size = Self::compute_slot_size(type_size, is_pod)?;
        let cap = Self::compute_capacity(channel_capacity)?;
        let channel_stride = Self::channel_stride(cap, slot_size)?;
        let num_channels = MAX_FANOUT_ENDPOINTS * MAX_FANOUT_ENDPOINTS;

        num_channels
            .checked_mul(channel_stride)?
            .checked_add(FANOUT_CHANNELS_BASE)
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
        let size = ShmFanoutRing::required_file_size(type_size, is_pod, cap)
            .expect("test geometry must be buildable");
        let layout = Layout::from_size_align(size, 4096).unwrap();
        let ptr = unsafe { alloc_zeroed(layout) };
        assert!(!ptr.is_null());
        (ptr, layout)
    }

    /// `init_owner` for the buildable geometries these tests use — it only
    /// returns `None` for a POD type above the slot cap, which
    /// [`owner_refuses_the_pod_geometry_its_own_attach_would_reject`] covers.
    ///
    /// # Safety
    ///
    /// Same contract as [`ShmFanoutRing::init_owner`].
    unsafe fn init_owner_ok(
        shm_base: *mut u8,
        type_size: usize,
        is_pod: bool,
        channel_capacity: u32,
    ) -> ShmFanoutRing {
        ShmFanoutRing::init_owner(shm_base, type_size, is_pod, channel_capacity)
            .expect("test geometry must be buildable")
    }

    /// A region whose owner died before stamping the magic must not hang the
    /// next process that opens the topic.
    ///
    /// `magic == 0` is normally an owner milliseconds from finishing, so `attach`
    /// spins for it — but it is also exactly what a SIGKILL or OOM kill during
    /// init leaves behind, and that region never becomes valid. The loop had no
    /// exit for that case: every later node opening the same topic spun forever
    /// on a core, with no timeout, no error and no log line, and restarting did
    /// not help because the region outlives the process that made it.
    #[test]
    fn attach_gives_up_on_a_region_whose_owner_never_finished() {
        let (ptr, layout) = alloc_shm_sim(8, true, 16);
        // Zero-filled and never initialised: magic stays 0 forever.
        let addr = ptr as usize;

        let (tx, rx) = std::sync::mpsc::channel();
        let region_len = layout.size();
        std::thread::spawn(move || {
            let ptr = addr as *mut u8;
            let result = unsafe { ShmFanoutRing::attach(ptr, region_len, true, 8) };
            let _ = tx.send(result.is_none());
        });

        // The bound inside `attach` is 2s; allow generous slack for a loaded
        // machine. Waiting rather than joining means a regression fails the test
        // instead of hanging the suite.
        match rx.recv_timeout(std::time::Duration::from_secs(20)) {
            Ok(gave_up) => assert!(
                gave_up,
                "attach returned a ring for a region that was never initialised"
            ),
            Err(_) => panic!(
                "attach never returned for a region whose owner died before \
                 stamping the magic — every node opening this topic would spin \
                 forever"
            ),
        }

        // The attach thread has returned, so nothing is reading the region.
        unsafe { std::alloc::dealloc(ptr, layout) };
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
        assert_eq!(size, Some(expected));
    }

    #[test]
    fn init_owner_sets_magic() {
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = init_owner_ok(ptr, 8, true, 64);
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
            let ring = init_owner_ok(ptr, 8, true, 64);
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
            let ring = init_owner_ok(ptr, 8, true, 64);
            let pub_id = ring.register_publisher().unwrap();
            let sub_id = ring.register_subscriber().unwrap();

            let val: u64 = 42;
            assert!(ring.send_pod(&val, pub_id));

            let recv: Option<u64> = ring.recv_pod(sub_id, &mut 0);
            assert_eq!(recv, Some(42));

            // Empty after consuming
            let recv2: Option<u64> = ring.recv_pod(sub_id, &mut 0);
            assert_eq!(recv2, None);

            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn send_recv_pod_2p2s() {
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = init_owner_ok(ptr, 8, true, 64);
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
            let r0a: Option<u64> = ring.recv_pod(s0, &mut 0);
            let r0b: Option<u64> = ring.recv_pod(s0, &mut 0);
            let mut got0 = vec![r0a.unwrap(), r0b.unwrap()];
            got0.sort();
            assert_eq!(got0, vec![100, 200]);

            let r1a: Option<u64> = ring.recv_pod(s1, &mut 0);
            let r1b: Option<u64> = ring.recv_pod(s1, &mut 0);
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
            let ring = init_owner_ok(ptr, 0, false, 32);
            let pub_id = ring.register_publisher().unwrap();
            let sub_id = ring.register_subscriber().unwrap();

            let data = b"hello world";
            assert!(ring.send_serde(data, pub_id));

            let recv = ring.recv_serde(sub_id, &mut 0);
            assert_eq!(recv.as_deref(), Some(&b"hello world"[..]));

            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn capacity_full_drops_oldest() {
        let (ptr, layout) = alloc_shm_sim(8, true, 16);
        unsafe {
            let ring = init_owner_ok(ptr, 8, true, 16);
            let pub_id = ring.register_publisher().unwrap();
            let sub_id = ring.register_subscriber().unwrap();

            // Drop-oldest: sends never fail. Push 20 into a 16-slot ring; the
            // oldest 4 (0..4) are overwritten by the newest 4 (16..20).
            for i in 0u64..20 {
                assert!(ring.send_pod(&i, pub_id), "send {} never fails", i);
            }
            let mut got = Vec::new();
            while let Some(v) = ring.recv_pod::<u64>(sub_id, &mut 0) {
                got.push(v);
            }
            assert_eq!(got.len(), 16, "ring retains exactly capacity newest msgs");
            let expected: Vec<u64> = (4u64..20).collect();
            assert_eq!(got, expected, "oldest dropped, order preserved");

            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn drop_oldest_reports_how_many_it_dropped() {
        // Drop-oldest is the designed overload behaviour, but it used to leave
        // no trace: `dropped_count()` reports send failures and these sends
        // never fail, so a subscriber that lost a full lap looked identical to
        // one that lost nothing. Observed in the multi-thread broadcast test —
        // 400 messages published, one subscriber received m145..m400, and every
        // counter in reach read zero.
        let (ptr, layout) = alloc_shm_sim(8, true, 16);
        unsafe {
            let ring = init_owner_ok(ptr, 8, true, 16);
            let pub_id = ring.register_publisher().unwrap();
            let sub_id = ring.register_subscriber().unwrap();

            for i in 0u64..20 {
                assert!(ring.send_pod(&i, pub_id), "send {} never fails", i);
            }

            let mut skipped = 0u64;
            let mut got = Vec::new();
            while let Some(v) = ring.recv_pod::<u64>(sub_id, &mut skipped) {
                got.push(v);
            }

            assert_eq!(got.len(), 16, "ring retains exactly capacity newest msgs");
            assert_eq!(
                skipped, 4,
                "20 sent into a 16-slot ring drops the oldest 4, and the \
                 consumer must be able to find that out"
            );
            assert_eq!(
                skipped as usize + got.len(),
                20,
                "skipped + received must account for everything sent"
            );

            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn round_robin_fairness() {
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = init_owner_ok(ptr, 8, true, 64);
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
            let r1: u64 = ring.recv_pod(sub, &mut 0).unwrap();
            let r2: u64 = ring.recv_pod(sub, &mut 0).unwrap();
            // r1 should be from P0 (10), r2 from P1 (30) — round-robin
            assert_eq!(r1, 10);
            assert_eq!(r2, 30);

            // Continue: P0 (20), then P1 (40)
            let r3: u64 = ring.recv_pod(sub, &mut 0).unwrap();
            let r4: u64 = ring.recv_pod(sub, &mut 0).unwrap();
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
            let _owner = init_owner_ok(ptr, 8, true, 64);

            // Simulated second process attaches
            let joiner = ShmFanoutRing::attach(ptr, layout.size(), true, 8).unwrap();
            assert_eq!(joiner.max_publishers, MAX_FANOUT_ENDPOINTS);
            assert_eq!(joiner.max_subscribers, MAX_FANOUT_ENDPOINTS);

            std::alloc::dealloc(ptr, layout);
        }
    }

    // ── The meta block as untrusted input ───────────────────────────────
    //
    // Every dimension in `FanoutShmMeta` is written by another process into a
    // region on the shared `/dev/shm`. `build_views` turns all of them into
    // channel pointers, and `try_send_pod` writes through those pointers, so a
    // dimension that does not fit the mapping is an out-of-bounds write at an
    // offset the writing process chooses. These plant the values a corrupt or
    // hostile writer leaves behind and require `attach` to refuse — the caller
    // then falls back to SpscShm, exactly as it does for a stale layout version.

    /// Initialise a region, mutate one field of its meta block, and report
    /// whether `attach` still accepts it.
    fn attach_accepts_after(mutate: impl FnOnce(&mut FanoutShmMeta)) -> bool {
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        // SAFETY: `ptr` is a live allocation of `layout.size()` bytes, aligned to
        // a page and large enough for the geometry `init_owner` writes; nothing
        // else refers to it for the duration of this function.
        unsafe {
            let owner = init_owner_ok(ptr, 8, true, 64);
            let meta = &mut *(ptr.add(FANOUT_META_OFFSET) as *mut FanoutShmMeta);
            mutate(meta);
            let accepted = ShmFanoutRing::attach(ptr, layout.size(), true, 8).is_some();
            drop(owner);
            std::alloc::dealloc(ptr, layout);
            accepted
        }
    }

    #[test]
    fn attach_refuses_a_stride_that_walks_the_matrix_off_the_region() {
        // 256 channels at this stride put the last one gigabytes past a ~300 KB
        // mapping; the first send through any of them writes there.
        let huge = attach_accepts_after(|m| m.channel_stride = 1 << 26);
        assert!(
            !huge,
            "a stride that does not fit the mapping must be refused"
        );
        // A stride SMALLER than the layout implies is equally wrong: channel i's
        // slots then overlap channel i-1's version stamps.
        let small = attach_accepts_after(|m| m.channel_stride = 128);
        assert!(
            !small,
            "a stride that overlaps neighbouring channels must be refused"
        );
    }

    #[test]
    fn attach_refuses_more_endpoints_than_the_matrix_has_room_for() {
        // Endpoint ids derived from these index `recv_cursors` and the
        // owner-PID arrays, both fixed at MAX_FANOUT_ENDPOINTS.
        assert!(!attach_accepts_after(|m| m.max_publishers = 4096));
        assert!(!attach_accepts_after(|m| m.max_subscribers = 4096));
        // Zero is not a matrix either — `recv_pod` computes `cursor % max_pubs`.
        assert!(!attach_accepts_after(|m| m.max_publishers = 0));
    }

    #[test]
    fn attach_refuses_a_mask_wider_than_the_channel() {
        // `pos & mask` in `seqlock_publish` is the ONLY bound on a slot index.
        let accepted = attach_accepts_after(|m| m.capacity_mask = u32::MAX);
        assert!(
            !accepted,
            "a mask inconsistent with the channel capacity must be refused, not \
             used to index the ring"
        );
    }

    #[test]
    fn attach_refuses_a_capacity_that_is_not_a_power_of_two() {
        // `seqlock_consume` requires `capacity == mask + 1`; without a power of
        // two there is no mask that makes `pos & mask` stay inside the ring.
        assert!(!attach_accepts_after(|m| m.channel_capacity = 63));
        assert!(!attach_accepts_after(|m| m.channel_capacity = 0));
    }

    #[test]
    fn attach_refuses_a_region_whose_slots_are_smaller_than_this_message() {
        // A region another node (or a previous run) built for an 8-byte message,
        // opened by a participant whose `T` is 64 bytes. The geometry is
        // internally consistent — it is simply too small — and `try_send_pod`
        // memcpys `size_of::<T>()` bytes into each slot with no length check.
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let owner = init_owner_ok(ptr, 8, true, 64);
            assert!(ShmFanoutRing::attach(ptr, layout.size(), true, 64).is_none());
            drop(owner);
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn owner_refuses_the_pod_geometry_its_own_attach_would_reject() {
        // The mirror image of the test above, on the side that WRITES the meta
        // block. `compute_slot_size` used to clamp a POD slot to the cap instead
        // of refusing, so a POD `T` larger than the cap produced a ring whose
        // slots could not hold the message — and `try_send_pod` memcpys
        // `size_of::<T>()` bytes into a slot with no bound of its own, so the
        // first send walked over the neighbouring slots, the version stamps, and
        // from the last channel the end of the mapping. `attach` refuses exactly
        // that geometry (`is_pod && slot_size < type_size`), so the owner path
        // and the attach path disagreed by construction.
        const OVERSIZED: usize = MAX_POD_SLOT_SIZE * 2 + 8;

        // Sized for a type AT the cap, which is precisely the region the clamped
        // geometry used to write.
        let (ptr, layout) = alloc_shm_sim(MAX_POD_SLOT_SIZE, true, 16);
        // SAFETY: `ptr` is a live, page-aligned allocation of `layout.size()`
        // bytes, large enough for every geometry `init_owner` can pick for a POD
        // type; nothing else refers to it for the duration of this test.
        let refused_slot = unsafe {
            match ShmFanoutRing::init_owner(ptr, OVERSIZED, true, 16) {
                // Refused before touching the region — the caller falls back.
                None => None,
                Some(owner) => {
                    let slot_size =
                        (*(ptr.add(FANOUT_META_OFFSET) as *const FanoutShmMeta)).slot_size;
                    // Anything the owner does build must be something a peer
                    // opening the same region with the same `T` accepts.
                    let accepted =
                        ShmFanoutRing::attach(ptr, layout.size(), true, OVERSIZED).is_some();
                    drop(owner);
                    (!accepted).then_some(slot_size)
                }
            }
        };
        unsafe { std::alloc::dealloc(ptr, layout) };
        assert!(
            refused_slot.is_none(),
            "init_owner built a {}-byte POD slot for a {OVERSIZED}-byte message — \
             a geometry its own attach refuses, and one that every try_send_pod \
             writes past the end of",
            refused_slot.unwrap_or_default()
        );
    }

    /// The same owner/attach agreement, reached through the capacity input
    /// instead of the type input.
    ///
    /// Gated to 64-bit because `1usize << 31` overflows the stride on a 32-bit
    /// `usize` — which is the *other* half of this bug and is unreachable from a
    /// test runner this repo has (no 32-bit CI target).
    #[test]
    #[cfg(target_pointer_width = "64")]
    fn a_capacity_the_meta_block_cannot_name_is_refused_not_truncated() {
        // `meta.channel_capacity` is a `u32`. Any capacity above 2^31 rounds up
        // to 2^32, which used to reach the meta block through `as u32` — landing
        // on `cap == 0`, from which `meta.capacity_mask = cap - 1` hands every
        // channel a mask of `u32::MAX` while the region is sized for 16 slots.
        assert!(
            ShmFanoutRing::required_file_size(8, true, (1usize << 31) + 1).is_none(),
            "a capacity that rounds to 2^32 cannot be named by a u32 meta field \
             and must be refused, not truncated"
        );
        // The largest capacity the field *can* name is still accepted, so this
        // refuses only what it has to.
        assert!(ShmFanoutRing::required_file_size(8, true, 1usize << 31).is_some());

        // The owner half is the one that matters: it writes the meta block that
        // `build_views` then trusts without a further bound.
        let (ptr, layout) = alloc_shm_sim(8, true, 16);
        let built_cap = {
            // SAFETY: `ptr` is a live, page-aligned allocation of
            // `layout.size()` bytes. `init_owner` settles the capacity before it
            // dereferences the region, so a refusal never touches `ptr`.
            match unsafe { ShmFanoutRing::init_owner(ptr, 8, true, u32::MAX) } {
                None => None,
                Some(owner) => {
                    // SAFETY: `init_owner` returned `Some`, so the meta block at
                    // `FANOUT_META_OFFSET` is initialised.
                    let cap = unsafe {
                        (*(ptr.add(FANOUT_META_OFFSET) as *const FanoutShmMeta)).channel_capacity
                    };
                    drop(owner);
                    Some(cap)
                }
            }
        };
        unsafe { std::alloc::dealloc(ptr, layout) };
        assert_eq!(
            built_cap,
            None,
            "init_owner wrote channel_capacity = {} into a region sized for 16 slots",
            built_cap.unwrap_or_default()
        );
    }

    #[test]
    fn attach_refuses_a_region_too_small_for_its_own_meta_block() {
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let owner = init_owner_ok(ptr, 8, true, 64);
            // Forming the `&FanoutShmMeta` is itself out of bounds here, so the
            // length check has to come before the reference, not after it.
            assert!(ShmFanoutRing::attach(ptr, FANOUT_META_OFFSET, true, 8).is_none());
            drop(owner);
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn pending_count() {
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = init_owner_ok(ptr, 8, true, 64);
            let p0 = ring.register_publisher().unwrap();
            let sub = ring.register_subscriber().unwrap();

            assert_eq!(ring.pending_count_for_sub(sub), 0);

            for i in 0u64..5 {
                ring.send_pod(&i, p0);
            }
            assert_eq!(ring.pending_count_for_sub(sub), 5);

            let _: Option<u64> = ring.recv_pod(sub, &mut 0);
            let _: Option<u64> = ring.recv_pod(sub, &mut 0);
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
            let ring = init_owner_ok(ptr, 8, true, 16);
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
                    let got: Option<u64> = ring.recv_pod(s, &mut 0);
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
            let ring = init_owner_ok(ptr, 0, false, 16);
            let p = ring.register_publisher().unwrap();
            let s = ring.register_subscriber().unwrap();

            for round in 0..10 {
                for i in 0..16 {
                    let msg = format!("r{}i{}", round, i);
                    assert!(ring.send_serde(msg.as_bytes(), p));
                }
                for i in 0..16 {
                    let expected = format!("r{}i{}", round, i);
                    let got = ring.recv_serde(s, &mut 0).unwrap();
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
            let ring = init_owner_ok(ptr, 128, true, 32);
            let p = ring.register_publisher().unwrap();
            let s = ring.register_subscriber().unwrap();

            for i in 0u64..32 {
                let val = LargePod { data: [i; 16] };
                assert!(ring.send_pod(&val, p));
            }
            for i in 0u64..32 {
                let got: Option<LargePod> = ring.recv_pod(s, &mut 0);
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
        let size = ShmFanoutRing::required_file_size(8, true, 256).unwrap();
        let layout = std::alloc::Layout::from_size_align(size, 4096).unwrap();
        let ptr = unsafe { alloc_zeroed(layout) };
        assert!(!ptr.is_null());

        let ring = unsafe { init_owner_ok(ptr, 8, true, 256) };
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
                if let Some(v) = unsafe { ring.recv_pod::<u64>(s0, &mut 0) } {
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
                if let Some(v) = unsafe { ring.recv_pod::<u64>(s1, &mut 0) } {
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
            assert!(
                data.len() <= msgs * 2,
                "{name} over-delivered {}",
                data.len()
            );
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
            let ring = init_owner_ok(ptr, 8, true, 16);
            let p = ring.register_publisher().unwrap();
            let s = ring.register_subscriber().unwrap();

            for cycle in 0u64..5 {
                // Fill exactly to capacity (no overwrite at avail == capacity).
                for i in 0u64..16 {
                    assert!(ring.send_pod(&(cycle * 100 + i), p));
                }

                // Drain and verify — all 16 come back in FIFO order.
                for i in 0u64..16 {
                    let got: Option<u64> = ring.recv_pod(s, &mut 0);
                    assert_eq!(got, Some(cycle * 100 + i));
                }
                let empty: Option<u64> = ring.recv_pod(s, &mut 0);
                assert_eq!(empty, None);
            }
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn max_endpoints_boundary() {
        let (ptr, layout) = alloc_shm_sim(8, true, 16);
        unsafe {
            let ring = init_owner_ok(ptr, 8, true, 16);

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
                while let Some(v) = ring.recv_pod::<u64>(sid, &mut 0) {
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
            let ring = init_owner_ok(ptr, 8, true, 64);
            let s = ring.register_subscriber().unwrap();
            let got: Option<u64> = ring.recv_pod(s, &mut 0);
            assert_eq!(got, None);
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn concurrent_registration() {
        // Multiple threads registering simultaneously
        let size = ShmFanoutRing::required_file_size(8, true, 64).unwrap();
        let layout = std::alloc::Layout::from_size_align(size, 4096).unwrap();
        let ptr = unsafe { alloc_zeroed(layout) };
        let ring = unsafe { init_owner_ok(ptr, 8, true, 64) };
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

    // ====================================================================
    // COMM-H1: cross-process liveness (bitmask reuse, holes, flock reclaim).
    //
    // All tests here are IN-PROCESS. Real cross-process SHM does not work in the
    // headless sandbox, so the *live-other-process → skip* case and the real
    // SIGKILL → OS-flock-release → peer-reclaim case REST ON DESIGN (the flock
    // OS contract) and are not integration-testable here. What IS verified
    // in-process: the reusable bitmask, hot-path routing over non-contiguous
    // active masks, the clean-drop free/reuse sequence, reclaim of a
    // *simulated-dead* slot (unlocked lock file + foreign owner PID), and — the
    // anti-UB guard — refusal to reclaim a live SAME-process slot.
    // ====================================================================

    /// Unique-per-test topic name so serial tests never share a lock file.
    fn test_topic(tag: &str) -> String {
        format!("commh1.{}.{}", tag, std::process::id())
    }

    #[test]
    fn commh1_bitmask_reuse_past_16_no_panic() {
        // Pre-fix: a monotonic counter panicked on the 17th cumulative register.
        // The reusable bitmask refuses the 17th SIMULTANEOUS endpoint gracefully
        // (None), and a freed slot is reclaimed and still delivers.
        let (ptr, layout) = alloc_shm_sim(8, true, 16);
        unsafe {
            let ring = init_owner_ok(ptr, 8, true, 16);
            let subs: Vec<usize> = (0..MAX_FANOUT_ENDPOINTS)
                .map(|_| ring.register_subscriber().expect("first 16 register"))
                .collect();
            assert_eq!(
                ring.register_subscriber(),
                None,
                "17th subscriber refused gracefully, not a panic"
            );
            let freed = subs[5];
            ring.deregister_subscriber(freed);
            let reclaimed = ring
                .register_subscriber()
                .expect("a freed slot must be reclaimable");
            assert_eq!(reclaimed, freed, "freed slot reused, not leaked");
            let p = ring.register_publisher().unwrap();
            ring.send_pod(&99u64, p);
            let got: Option<u64> = ring.recv_pod(reclaimed, &mut 0);
            assert_eq!(got, Some(99), "reclaimed subscriber receives after reuse");
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn commh1_hot_path_send_skips_subscriber_holes() {
        // Non-contiguous active subscriber mask {0,2,5}: a send fans out to EXACTLY
        // those channels and skips the holes.
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = init_owner_ok(ptr, 8, true, 64);
            let p = ring.register_publisher().unwrap(); // slot 0
            let subs: Vec<usize> = (0..6)
                .map(|_| ring.register_subscriber().unwrap())
                .collect();
            for &h in &[1usize, 3, 4] {
                ring.deregister_subscriber(subs[h]);
            }
            let meta = &*ring.meta_ptr;
            assert_eq!(
                meta.sub_active.load(Ordering::Relaxed),
                (1 << 0) | (1 << 2) | (1 << 5),
                "active sub mask is the non-contiguous set {{0,2,5}}"
            );
            ring.send_pod(&123u64, p);
            for s in [0usize, 2, 5] {
                assert_eq!(
                    ring.pending_count_for_sub(s),
                    1,
                    "active sub {s} got the msg"
                );
                let got: Option<u64> = ring.recv_pod(s, &mut 0);
                assert_eq!(got, Some(123), "active sub {s} receives");
            }
            for s in [1usize, 3, 4] {
                assert_eq!(ring.pending_count_for_sub(s), 0, "hole sub {s} untouched");
            }
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn commh1_hot_path_recv_round_robin_over_publisher_holes() {
        // Non-contiguous active PUBLISHER mask {0,2,5}: a subscriber's round-robin
        // recv polls EXACTLY those publisher channels (skips holes) and rotates
        // over the active set without getting stuck on a hole.
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = init_owner_ok(ptr, 8, true, 64);
            let pubs: Vec<usize> = (0..6).map(|_| ring.register_publisher().unwrap()).collect();
            for &h in &[1usize, 3, 4] {
                ring.deregister_publisher(pubs[h]);
            }
            let s = ring.register_subscriber().unwrap(); // slot 0
            let meta = &*ring.meta_ptr;
            assert_eq!(
                meta.pub_active.load(Ordering::Relaxed),
                (1 << 0) | (1 << 2) | (1 << 5),
                "active pub mask is {{0,2,5}}"
            );
            ring.send_pod(&100u64, 0);
            ring.send_pod(&102u64, 2);
            ring.send_pod(&105u64, 5);
            let mut got = Vec::new();
            while let Some(v) = ring.recv_pod::<u64>(s, &mut 0) {
                got.push(v);
            }
            got.sort();
            assert_eq!(
                got,
                vec![100, 102, 105],
                "recv polls only active pubs {{0,2,5}}, skipping holes"
            );
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn commh1_clean_drop_sequence_frees_flock_slot_for_reuse() {
        // The clean-drop path (RingTopic::drop) does: deregister (clear bit+PID)
        // then drop the held FileLock. Exercise that sequence directly and prove
        // the slot is reusable afterward BY THE SAME PROCESS — the same-process
        // guard would refuse a slot still marked ours, so clearing bit+PID matters.
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = init_owner_ok(ptr, 8, true, 64);
            let topic = test_topic("cleandrop");
            let (id, lock) = ring.register_publisher_locked(&topic).expect("claim");
            assert_eq!(id, 0);
            let meta = &*ring.meta_ptr;
            assert!(meta.pub_active.load(Ordering::Relaxed) & 1 != 0);
            assert_eq!(
                meta.pub_owner_pids[0].load(Ordering::Relaxed),
                std::process::id()
            );
            // Clean-drop sequence: clear bit+PID, then release the flock.
            ring.deregister_publisher(id);
            drop(lock);
            assert!(
                meta.pub_active.load(Ordering::Relaxed) & 1 == 0,
                "deregister clears the active bit"
            );
            assert_eq!(
                meta.pub_owner_pids[0].load(Ordering::Relaxed),
                0,
                "deregister clears the owner PID"
            );
            // Same process re-registers → reuses slot 0 as a FREE claim (bit clear).
            let (id2, _lock2) = ring.register_publisher_locked(&topic).expect("reuse");
            assert_eq!(id2, 0, "cleanly-freed slot reused by the same process");
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn commh1_simulated_dead_owner_slot_is_reclaimed() {
        // Reclaim path (in-process simulation of a dead OTHER-process owner): slot 0
        // marked active with a FOREIGN owner PID and NO live flock (as after the OS
        // released a SIGKILLed owner's lock). A new register must RECLAIM slot 0
        // (acquire the lock; foreign PID != ours so the guard does not fire), NOT
        // skip past it to a fresh slot.
        //
        // RESTS-ON-DESIGN: the real SIGKILL → OS-flock-release step is not
        // reproducible in this sandbox; here the released state is simulated by
        // simply not holding a flock on the slot's lock file.
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = init_owner_ok(ptr, 8, true, 64);
            let topic = test_topic("dead");
            let meta = &*ring.meta_ptr;
            let foreign = std::process::id().wrapping_add(1); // definitely not us
            meta.pub_active.fetch_or(1 << 0, Ordering::Relaxed);
            meta.pub_owner_pids[0].store(foreign, Ordering::Relaxed);
            let (id, _lock) = ring
                .register_publisher_locked(&topic)
                .expect("dead slot must be reclaimable");
            assert_eq!(
                id, 0,
                "must reclaim abandoned slot 0, not skip to a fresh slot"
            );
            assert_eq!(
                meta.pub_owner_pids[0].load(Ordering::Relaxed),
                std::process::id(),
                "reclaim rewrites the owner PID to us"
            );
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn commh1_simulated_dead_subscriber_reclaim_skips_stale_backlog() {
        // A reclaimed (dead-owner) subscriber slot must start fresh — the tail is
        // reset to head on claim, so the prior owner's unread backlog is skipped.
        let (ptr, layout) = alloc_shm_sim(8, true, 16);
        unsafe {
            let ring = init_owner_ok(ptr, 8, true, 16);
            let topic = test_topic("deadsub");
            let p = ring.register_publisher().unwrap(); // slot 0
            let meta = &*ring.meta_ptr;
            // Simulate a DEAD subscriber that owned slot 0 (foreign PID, no flock).
            meta.sub_active.fetch_or(1 << 0, Ordering::Relaxed);
            meta.sub_owner_pids[0].store(std::process::id().wrapping_add(1), Ordering::Relaxed);
            for i in 0..5u64 {
                ring.send_pod(&i, p); // backlog into channel[p][0]
            }
            let (sid, _lock) = ring
                .register_subscriber_locked(&topic)
                .expect("reclaim dead subscriber slot");
            assert_eq!(sid, 0, "reclaim the abandoned subscriber slot 0");
            let stale: Option<u64> = ring.recv_pod(sid, &mut 0);
            assert_eq!(
                stale, None,
                "reclaimed subscriber must skip the dead owner's backlog (tail reset)"
            );
            ring.send_pod(&99u64, p);
            let fresh: Option<u64> = ring.recv_pod(sid, &mut 0);
            assert_eq!(fresh, Some(99), "delivery works after reclaim");
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn commh1_same_process_live_slot_is_not_reclaimed() {
        // THE ANTI-UB GUARD. A slot whose bit is SET and owner PID == THIS process
        // must NOT be reclaimed even when try_exclusive succeeds (the permissive-OS
        // re-lock case). Simulated by marking slot 0 active+owned-by-us but NOT
        // holding a real flock, so try_exclusive returns Ok(Some); the guard must
        // still refuse slot 0 and claim the next free slot (1) instead. Handing
        // slot 0 to a second owner would give two producers one SPSC channel = UB.
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = init_owner_ok(ptr, 8, true, 64);
            let topic = test_topic("guard");
            let meta = &*ring.meta_ptr;
            meta.pub_active.fetch_or(1 << 0, Ordering::Relaxed);
            meta.pub_owner_pids[0].store(std::process::id(), Ordering::Relaxed); // OUR pid
            let (id, _lock) = ring
                .register_publisher_locked(&topic)
                .expect("claim a slot");
            assert_ne!(
                id, 0,
                "must NOT reclaim our own LIVE slot 0 (anti-UB guard)"
            );
            assert_eq!(id, 1, "guard skips slot 0, claims the next free slot");
            // Slot 0 left untouched (still ours, still active).
            assert_eq!(
                meta.pub_owner_pids[0].load(Ordering::Relaxed),
                std::process::id(),
                "guarded slot 0 owner PID untouched"
            );
            assert!(
                meta.pub_active.load(Ordering::Relaxed) & 1 != 0,
                "guarded slot 0 still active"
            );
            std::alloc::dealloc(ptr, layout);
        }
    }

    #[test]
    fn commh1_two_locked_publishers_same_process_get_distinct_slots() {
        // Same-process double registration via the flock path must yield DISTINCT
        // slots — on Linux via flock's same-process denial (Ok(None) → skip), on
        // permissive OSes via the PID guard. Either way, never two owners on one
        // SPSC channel.
        let (ptr, layout) = alloc_shm_sim(8, true, 64);
        unsafe {
            let ring = init_owner_ok(ptr, 8, true, 64);
            let topic = test_topic("two");
            let (a, _la) = ring.register_publisher_locked(&topic).expect("first");
            let (b, _lb) = ring.register_publisher_locked(&topic).expect("second");
            assert_ne!(a, b, "two live same-process publishers get distinct slots");
            std::alloc::dealloc(ptr, layout);
        }
    }
}
