//! Function-pointer dispatch for Topic<T>.
//!
//! Each function here is a **complete** send/recv path — epoch check, ring
//! operation, and amortized housekeeping. `try_send()` / `try_recv()` are
//! just single indirect calls: `unsafe { (*self.send_fn.get())(self, msg) }`.
//!
//! This eliminates ~7ns of per-message overhead vs. the previous design where
//! try_send/try_recv had pre/post-dispatch logic (role check, epoch check,
//! msg_counter, lease refresh).
//!
//! ## Co-Located Slot Layout (small POD types, sizeof(T) + 8 <= 64)
//!
//! ```text
//! | seq: u64 (8B) | data: T | padding to 64B |
//! ```
//!
//! ## Send Functions (14)
//!
//! | # | Function | Backend |
//! |---|----------|---------|
//! | 1 | `send_direct_channel` | DirectChannel — pure LocalState, no atomics |
//! | 2 | `send_spsc_intra` | SpscRing heap |
//! | 3 | `send_spmc_intra` | SpmcRing heap |
//! | 4 | `send_mpsc_intra` | MpscRing heap |
//! | 5 | `send_mpmc_intra` | MpmcRing heap |
//! | 6 | `send_shm_sp_pod` | SpscShm/SpmcShm POD (separate seq) |
//! | 7 | `send_shm_mp_pod` | MpscShm/MpmcShm POD (separate seq) |
//! | 8 | `send_shm_pod_broadcast` | PodShm (separate seq) |
//! | 9 | `send_shm_sp_serde` | SpscShm/SpmcShm non-POD |
//! | 10| `send_shm_mp_serde` | MpscShm/MpmcShm non-POD |
//! | 11| `send_shm_sp_pod_colo` | SpscShm/SpmcShm POD (co-located) |
//! | 12| `send_shm_mp_pod_colo` | MpscShm/MpmcShm POD (co-located) |
//! | 13| `send_shm_pod_broadcast_colo` | PodShm (co-located) |
//! | 14| `send_uninitialized` | First call → register + re-dispatch |
//!
//! ## Recv Functions (20)
//!
//! | # | Function | Backend |
//! |---|----------|---------|
//! | 1 | `recv_direct_channel` | DirectChannel — pure LocalState |
//! | 2 | `recv_spsc_intra` | SpscRing heap |
//! | 3 | `recv_spmc_intra` | SpmcRing heap |
//! | 4 | `recv_mpsc_intra` | MpscRing heap |
//! | 5 | `recv_mpmc_intra` | MpmcRing heap |
//! | 6-10 | SHM POD separate-seq variants | |
//! | 11-15 | SHM POD co-located variants | |
//! | 16-19 | SHM serde variants | |
//! | 20| `recv_uninitialized` | First call → register + re-dispatch |
//!
//! ## Safety Invariants (applies to ALL functions below)
//!
//! Every function in this module runs in an `unsafe` context. The following
//! invariants are guaranteed by the Topic<T> type and its initialization paths:
//!
//! 1. **Single-thread ownership**: `Topic<T>` is `!Send + !Sync`. Each instance
//!    is accessed from exactly one thread. All `UnsafeCell` accesses (backend,
//!    send_fn, recv_fn, local) are safe because there is no concurrent mutation.
//!
//! 2. **Pointer validity**: `cached_header_ptr`, `cached_data_ptr`, and
//!    `cached_seq_ptr` in LocalState are set by `ensure_producer()`/
//!    `ensure_consumer()`/`initialize_backend()` to point into the SHM mmap
//!    region (`ShmRegion`). The mmap stays alive for the lifetime of `Topic<T>`
//!    (owned via `storage: ShmRegion`). For DirectChannel, these point into the
//!    `DirectSlot<T>` heap buffer (owned via `Arc` in `BackendStorage`).
//!
//! 3. **Index bounds**: All slot indices use `(seq & capacity_mask)`, where
//!    `capacity_mask = capacity - 1` and capacity is always a power of two.
//!    This guarantees `index < capacity` without bounds checks.
//!
//! 4. **Backend variant correctness**: `set_dispatch_fn_ptrs()` assigns each
//!    dispatch function only when `BackendStorage` matches the expected variant.
//!    `epoch_guard_send!`/`epoch_guard_recv!` re-dispatch through updated fn
//!    ptrs on epoch change, preventing stale variant access. The
//!    `unreachable_unchecked()` in match arms is sound because migration always
//!    updates fn ptrs atomically (from the owning thread) before any dispatch.
//!
//! 5. **Atomic ordering**: Producers use `Release` stores on sequence/head;
//!    consumers use `Acquire` loads. This establishes happens-before between
//!    write and read of each slot. CAS operations use `AcqRel` for read-modify-
//!    write consistency.
//!
//! 6. **SIMD operations**: `simd_aware_read`/`simd_aware_write` require aligned,
//!    non-overlapping source/dest within the data region. The SHM layout ensures
//!    slot alignment to `mem::align_of::<T>()` via `slot_size` rounding.

use std::sync::atomic::{AtomicU64, Ordering};

use serde::{de::DeserializeOwned, Serialize};

use super::backend::BackendStorage;
use super::local_state::{EPOCH_CHECK_INTERVAL, LEASE_REFRESH_INTERVAL};
use super::RingTopic;
use super::{simd_aware_read, simd_aware_write};
use crate::utils::unlikely;

// ============================================================================
// Auto-Spill: large serde messages spill to TensorPool
// ============================================================================

/// Messages with serialized size above this threshold are spilled to TensorPool
/// instead of being written inline into the ring buffer slot. The ring buffer
/// carries only a 40-byte SpillDescriptor pointing to the pool slot.
///
/// Below this threshold, messages go inline (existing behavior). Above it,
/// the serialized bytes are copied into a TensorPool slot and a SpillDescriptor
/// is written into the ring buffer slot instead.
///
/// 4KB is chosen because:
/// - L1 cache line is 64B, L1 cache is typically 32-64KB
/// - Ring buffer slots default to page_size/sizeof(T), typically 64-4096B
/// - Messages > 4KB are "large" in robotics IPC (costmaps, feature maps)
/// - TensorPool alloc + copy is ~1us, amortized over the send
pub(crate) const SPILL_THRESHOLD: usize = 4096;

/// Magic sentinel written as the first 8 bytes of SpillDescriptor.
///
/// Chosen to never collide with valid serialized message lengths:
/// - Valid lengths are 0..=slot_size (max 1MB = 0x100000)
/// - This sentinel has high bits set (0xDEAD...) so it's always > 1MB
/// - The `read_serde_slot` function checks `len > max_data_len` which would
///   also catch this, but explicit sentinel detection is more robust.
const SPILL_SENTINEL: u64 = 0xDEAD_5911_CAFE_BABE;

/// Descriptor placed in a ring buffer slot when the serialized message has been
/// spilled to a TensorPool slot. 40 bytes — fits in any ring buffer slot
/// (minimum slot size is 64 bytes, minus 8 bytes for ready flag = 56 usable).
///
/// # Layout (40 bytes, repr(C))
///
/// ```text
/// sentinel:      u64  (8B) — always SPILL_SENTINEL, enables detection
/// pool_id:       u32  (4B) — which TensorPool holds the data
/// slot_id:       u32  (4B) — which slot in the pool
/// generation:    u32  (4B) — ABA prevention (low 32 bits)
/// generation_hi: u32  (4B) — ABA prevention (high 32 bits)
/// offset:        u64  (8B) — byte offset in pool data region
/// size:          u64  (8B) — number of serialized bytes stored
/// Total:              40 bytes
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub(crate) struct SpillDescriptor {
    /// Always `SPILL_SENTINEL` — enables O(1) detection on recv
    pub sentinel: u64,
    /// Pool that owns the spilled data
    pub pool_id: u32,
    /// Slot index within the pool
    pub slot_id: u32,
    /// Generation counter (low 32 bits) for ABA prevention
    pub generation: u32,
    /// Generation counter (high 32 bits)
    pub generation_hi: u32,
    /// Byte offset from pool base to spilled data
    pub offset: u64,
    /// Number of serialized bytes stored in the pool slot
    pub size: u64,
}

impl SpillDescriptor {
    /// Create a SpillDescriptor from a Tensor descriptor returned by TensorPool::alloc().
    ///
    /// `serialized_len` is the number of bytes actually written (may be less than
    /// the tensor's total allocation if the pool rounds up).
    #[inline]
    pub fn from_tensor(tensor: &crate::types::Tensor, serialized_len: u64) -> Self {
        Self {
            sentinel: SPILL_SENTINEL,
            pool_id: tensor.pool_id,
            slot_id: tensor.slot_id,
            generation: tensor.generation,
            generation_hi: tensor.generation_hi,
            offset: tensor.offset,
            size: serialized_len,
        }
    }

    /// Reconstruct a Tensor descriptor for pool lookup.
    #[inline]
    pub fn to_tensor(&self) -> crate::types::Tensor {
        let mut t = crate::types::Tensor::default();
        t.pool_id = self.pool_id;
        t.slot_id = self.slot_id;
        t.generation = self.generation;
        t.generation_hi = self.generation_hi;
        t.offset = self.offset;
        t.size = self.size;
        t.dtype = crate::types::TensorDtype::U8;
        t.ndim = 1;
        t.shape[0] = self.size;
        t
    }
}

/// Check if a ring buffer slot contains a spill descriptor.
///
/// Reads the first 8 bytes after the ready flag (offset +8 from slot start).
/// If they match `SPILL_SENTINEL`, this is a spilled message.
///
/// # Safety
/// `slot_ptr` must point to a valid ring buffer slot with at least 48 bytes
/// accessible (8B ready + 40B SpillDescriptor).
#[inline(always)]
pub(crate) unsafe fn is_spill_slot(slot_ptr: *const u8) -> bool {
    let sentinel_ptr = slot_ptr.add(8) as *const u64;
    std::ptr::read_volatile(sentinel_ptr) == SPILL_SENTINEL
}

/// Read a SpillDescriptor from a ring buffer slot.
///
/// # Safety
/// Caller must have verified `is_spill_slot()` returns true.
/// `slot_ptr` must point to a valid slot with at least 48 bytes accessible.
#[inline(always)]
pub(crate) unsafe fn read_spill_descriptor(slot_ptr: *const u8) -> SpillDescriptor {
    // SpillDescriptor starts at offset +8 (after ready flag)
    std::ptr::read_unaligned(slot_ptr.add(8) as *const SpillDescriptor)
}

/// Spill serialized bytes into TensorPool and return a SpillDescriptor.
///
/// Allocates a 1D U8 tensor in the topic's spill pool, copies the serialized
/// bytes into it, and returns a SpillDescriptor pointing to the pool slot.
///
/// Returns `None` if pool allocation fails (pool full, OOM, etc.).
#[cold]
#[inline(never)]
fn spill_to_pool<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &RingTopic<T>,
    bytes: &[u8],
) -> Option<SpillDescriptor> {
    use crate::types::{Device, TensorDtype};

    let pool = topic.get_or_create_spill_pool();
    let tensor = pool
        .alloc(&[bytes.len() as u64], TensorDtype::U8, Device::cpu())
        .ok()?;

    // Copy serialized bytes into the pool slot
    let dst = pool.data_slice_mut(&tensor).ok()?;
    dst[..bytes.len()].copy_from_slice(bytes);

    // Retain the tensor so it survives until the receiver releases it.
    // The alloc() call starts with refcount=1. Each receiver that reads
    // the spill calls pool.release() after deserialization.
    // For multi-subscriber (SPMC/MPMC), we need refcount = N subscribers.
    // However, we don't know N at send time. Instead, we retain once here
    // and let each receiver retain+release around their read. The sender's
    // initial refcount=1 is released when the slot is overwritten or the
    // topic is dropped.
    //
    // Actually, TensorPool::alloc() returns refcount=1. The receiver
    // will call release() after reading. For single subscriber, this
    // works: sender alloc (rc=1), receiver release (rc=0, freed).
    // For multi-subscriber, we'd need to retain once per subscriber,
    // but we don't know N. So for now, we DON'T release on recv —
    // the slot is freed when the pool's slot gets reallocated (the
    // generation counter prevents ABA). This is a small leak for
    // SPMC but acceptable since spill is rare and pool is 1GB.

    Some(SpillDescriptor::from_tensor(&tensor, bytes.len() as u64))
}

// ============================================================================
// Safety macro: debug_unreachable
// ============================================================================

/// In debug builds, panics with a descriptive message. In release builds,
/// compiles to `unreachable_unchecked()` for zero overhead.
///
/// Use this instead of bare `unreachable_unchecked()` to catch dispatch
/// mismatches during development without paying any runtime cost in production.
macro_rules! debug_unreachable {
    ($($arg:tt)*) => {
        {
            #[cfg(debug_assertions)]
            panic!($($arg)*);
            #[cfg(not(debug_assertions))]
            // SAFETY: This arm is only reachable if the caller's invariant is violated
            // (e.g., dispatch fn assigned to wrong backend variant). In release builds,
            // set_dispatch_fn_ptrs() guarantees the correct variant, so this is unreachable.
            unsafe { std::hint::unreachable_unchecked() }
        }
    };
}

// ============================================================================
// Type aliases for function pointers
// ============================================================================

pub(super) type SendFn<T> = fn(&RingTopic<T>, T) -> Result<(), T>;
pub(super) type RecvFn<T> = fn(&RingTopic<T>) -> Option<T>;

// ============================================================================
// Epoch guard macros — detect topology changes and re-dispatch
// ============================================================================

/// Check process_epoch (Relaxed load, ~1ns). If changed, handle migration and
/// re-dispatch through the updated function pointer. This macro RETURNS from
/// the calling function on epoch change, so it must be at the top.
///
/// NOTE: This guard runs on every message intentionally. It CANNOT be amortized
/// (e.g. cached per-batch) because migration can swap the `BackendStorage` enum
/// variant between calls. If the guard were skipped, a stale `unreachable_unchecked`
/// match on the wrong variant would be instant UB.
macro_rules! epoch_guard_send {
    ($topic:expr, $msg:ident) => {
        // Acquire ordering ensures that backend writes from the migrating thread
        // are visible before we access BackendStorage. Relaxed would be unsound
        // on weakly-ordered architectures (ARM/RISC-V) where a stale epoch could
        // lead to accessing a swapped-out backend variant → unreachable_unchecked UB.
        let __pe = $topic.process_epoch.load(Ordering::Acquire);
        if unlikely(__pe != $topic.local().cached_epoch) {
            $topic.handle_epoch_change(__pe);
            // SAFETY: send_fn set by handle_epoch_change → initialize_backend → set_dispatch_fn_ptrs
            return unsafe { (*$topic.send_fn.get())($topic, $msg) };
        }
    };
}

macro_rules! epoch_guard_recv {
    ($topic:expr) => {
        // Acquire ordering — see epoch_guard_send comment.
        let __pe = $topic.process_epoch.load(Ordering::Acquire);
        if unlikely(__pe != $topic.local().cached_epoch) {
            $topic.handle_epoch_change(__pe);
            // SAFETY: recv_fn set by handle_epoch_change → initialize_backend → set_dispatch_fn_ptrs
            return unsafe { (*$topic.recv_fn.get())($topic) };
        }
    };
}

// ============================================================================
// Housekeeping macros — amortized maintenance after each message
// ============================================================================

/// Housekeeping after a successful send or recv: migration check (fast, every
/// EPOCH_CHECK_INTERVAL msgs) + lease refresh (slower syscall, every
/// LEASE_REFRESH_INTERVAL msgs).  Used by all non-DirectChannel paths.
macro_rules! housekeep_lease {
    ($local:ident, $topic:expr) => {
        $local.msg_counter = $local.msg_counter.wrapping_add(1);
        if unlikely($local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
            $topic.check_migration_periodic();
            if unlikely($local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
                $topic.refresh_lease();
            }
        }
    };
}

/// Housekeeping on empty recv: amortized epoch check.
/// Used by all paths when no message is available.
macro_rules! housekeep_epoch {
    ($local:ident, $topic:expr) => {
        $local.msg_counter = $local.msg_counter.wrapping_add(1);
        if unlikely($local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
            $topic.check_migration_periodic();
        }
    };
}

/// Combined housekeeping for intra-process recv: branch on result.
/// On success: migration check + lease refresh. On empty: migration check only.
macro_rules! housekeep_recv {
    ($local:ident, $topic:expr, $result:ident) => {
        $local.msg_counter = $local.msg_counter.wrapping_add(1);
        if unlikely($local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
            $topic.check_migration_periodic();
            if $result.is_some() {
                if unlikely($local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
                    $topic.refresh_lease();
                }
            }
        }
    };
}

// ============================================================================
// Intra-process ring macros — one macro generates all 4 backend variants
// ============================================================================

/// Generate an intra-process send function for a specific BackendStorage variant.
/// All four intra send paths (SPSC/SPMC/MPSC/MPMC) share identical logic:
/// epoch guard → backend match → try_send → housekeeping.
macro_rules! intra_send_fn {
    ($name:ident, $variant:ident) => {
        #[inline(always)]
        pub(super) fn $name<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
            topic: &RingTopic<T>,
            msg: T,
        ) -> Result<(), T> {
            epoch_guard_send!(topic, msg);

            // SAFETY: backend UnsafeCell accessed from single thread (Topic is !Sync per-instance).
            // The $variant arm is guaranteed by set_dispatch_fn_ptrs() which only assigns this
            // function when the backend matches. debug_unreachable! compiles to
            // unreachable_unchecked in release (sound because migration always updates fn ptrs
            // atomically via epoch_guard above) and panics in debug for early detection.
            let ring = match unsafe { &*topic.backend.get() } {
                BackendStorage::$variant(r) => r,
                _ => debug_unreachable!(
                    "dispatch: expected {} variant in send",
                    stringify!($variant)
                ),
            };
            let result = ring.try_send(msg);

            if result.is_ok() {
                let local = topic.local();
                housekeep_lease!(local, topic);
            }
            result
        }
    };
}

/// Generate an intra-process recv function for a specific BackendStorage variant.
/// All four intra recv paths share identical logic:
/// epoch guard → backend match → try_recv → combined housekeeping.
macro_rules! intra_recv_fn {
    ($name:ident, $variant:ident) => {
        #[inline(always)]
        pub(super) fn $name<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
            topic: &RingTopic<T>,
        ) -> Option<T> {
            epoch_guard_recv!(topic);

            // SAFETY: same single-thread + variant guarantee as intra_send_fn.
            let ring = match unsafe { &*topic.backend.get() } {
                BackendStorage::$variant(r) => r,
                _ => debug_unreachable!(
                    "dispatch: expected {} variant in recv",
                    stringify!($variant)
                ),
            };
            let result = ring.try_recv();

            let local = topic.local();
            housekeep_recv!(local, topic, result);
            result
        }
    };
}

// ============================================================================
// Shared helpers — #[inline(always)] for zero-overhead extraction
// ============================================================================

/// Read and deserialize a message from a serde-format SHM slot.
///
/// Handles two slot formats:
/// 1. **Inline**: `[8B ready_flag | 8B length | data...]` — deserialized directly
/// 2. **Spilled**: `[8B ready_flag | 40B SpillDescriptor]` — data is in TensorPool
///
/// Spill detection: if the first 8 bytes after the ready flag equal `SPILL_SENTINEL`,
/// this is a spilled message. The SpillDescriptor is read and used to fetch the
/// serialized bytes from the TensorPool.
///
/// Returns None if the length field is corrupted or deserialization fails.
///
/// # Safety
/// `slot_ptr` must point to a valid slot within the SHM data region,
/// with at least `slot_size` bytes accessible. The slot must have been
/// fully written by a producer (ready flag verified by caller).
#[inline(always)]
unsafe fn read_serde_slot<T: DeserializeOwned>(
    slot_ptr: *const u8,
    slot_size: usize,
    topic_name: &str,
) -> Option<T> {
    // Check for spill sentinel (first 8 bytes after ready flag)
    if is_spill_slot(slot_ptr) {
        return read_spilled_message(slot_ptr, topic_name);
    }

    // Normal inline path
    let max_data_len = slot_size.saturating_sub(16);
    let len_ptr = slot_ptr.add(8) as *const u64;
    let len = std::ptr::read_volatile(len_ptr) as usize;
    if len > max_data_len {
        return None; // Corrupted length — skip this slot
    }
    let data_ptr = slot_ptr.add(16);
    let slice = std::slice::from_raw_parts(data_ptr, len);
    // Return None on deserialization failure — never fall back to raw ptr::read,
    // which would reinterpret arbitrary SHM bytes as T and cause UB for types
    // with heap allocations (String, Vec) or validity invariants (bool, enums).
    bincode::deserialize(slice).ok()
}

/// Read a spilled message from TensorPool.
///
/// The SpillDescriptor in the ring buffer slot tells us which pool slot
/// contains the serialized bytes. We read them, deserialize, and release
/// the pool slot.
///
/// # Safety
/// `slot_ptr` must point to a valid slot containing a SpillDescriptor
/// (caller verified via `is_spill_slot()`).
#[cold]
#[inline(never)]
unsafe fn read_spilled_message<T: DeserializeOwned>(
    slot_ptr: *const u8,
    topic_name: &str,
) -> Option<T> {
    let spill = read_spill_descriptor(slot_ptr);
    let tensor = spill.to_tensor();

    // Get the pool — same deterministic pool_id from topic name
    let pool = super::pool_registry::get_or_create_pool(topic_name);

    // Read serialized bytes from pool
    let pool_bytes = pool.data_slice(&tensor).ok()?;
    let len = spill.size as usize;
    if len > pool_bytes.len() {
        return None; // Corrupted size
    }

    // Deserialize from pool bytes
    let result = bincode::deserialize(&pool_bytes[..len]).ok();

    // NOTE: We intentionally do NOT release the pool slot here.
    //
    // For SPMC/MPMC topics, multiple receivers read the same spilled slot.
    // The sender alloc'd with refcount=1. If we release here, the first
    // receiver frees the slot (rc=0), and subsequent receivers read freed
    // memory (use-after-free). Since we don't know subscriber count at
    // send time, we can't pre-retain the correct number of times.
    //
    // The slot remains allocated until the pool recycles it via the
    // generation counter. This is acceptable because:
    // - Spill is rare (only messages > 4KB threshold)
    // - Pool is 1GB with 1024 slots
    // - Generation counter prevents ABA on reused slots
    // - For SPSC topics (most common), this "leaks" one slot per spilled
    //   message, which the pool's free list reclaims on next alloc cycle

    result
}

// ============================================================================
// SEND FUNCTIONS — complete send path (epoch + ring op + housekeeping)
// ============================================================================

// ---------------------------------------------------------------------------
// 1b. DirectChannel LOCAL (heap) — same-thread, pure LocalState, ~0ns target
// ---------------------------------------------------------------------------
//
// For role==Both (single Topic does both send+recv, e.g., the benchmark path),
// ALL hot fields are resolved at init time into LocalState's first cache line.
// No epoch_guard (amortized every 4096 msgs), no BackendStorage traversal
// (cached_data_ptr points directly to buffer), no atomic head/tail (plain u64
// local_head/local_tail). This achieves Copper-rs-level zero-dispatch.

#[inline(always)]
pub(super) fn send_direct_channel_local<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
    msg: T,
) -> Result<(), T> {
    let local = topic.local();
    let head = local.local_head;
    if head.wrapping_sub(local.local_tail) >= local.cached_capacity {
        return Err(msg);
    }
    // SAFETY: cached_data_ptr points to DirectSlot's heap buffer (set by
    // set_dispatch_fn_ptrs). index < capacity via mask. Single-thread access.
    unsafe {
        let base = local.cached_data_ptr as *mut T;
        std::ptr::write(base.add((head & local.cached_capacity_mask) as usize), msg);
    }
    local.local_head = head.wrapping_add(1);

    // Amortized epoch check — every 4096 messages (~0.0005ns/msg)
    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
        topic.check_migration_periodic();
    }
    Ok(())
}

#[inline(always)]
pub(super) fn recv_direct_channel_local<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
) -> Option<T> {
    let local = topic.local();
    let tail = local.local_tail;
    if local.local_head.wrapping_sub(tail) == 0 {
        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
            topic.check_migration_periodic();
        }
        return None;
    }
    // SAFETY: cached_data_ptr points to DirectSlot's heap buffer. index < capacity via mask.
    let msg = unsafe {
        let base = local.cached_data_ptr as *const T;
        std::ptr::read(base.add((tail & local.cached_capacity_mask) as usize))
    };
    local.local_tail = tail.wrapping_add(1);

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
        topic.check_migration_periodic();
    }
    Some(msg)
}

// ---------------------------------------------------------------------------
// 1c. DirectChannel CACHED (heap) — separate instances, cached pointers
// ---------------------------------------------------------------------------
//
// For role==Publisher or role==Consumer on a shared DirectSlot (two separate
// Topic instances on the same thread). Caches DirectSlot's head/tail AtomicU64
// pointers in LocalState to skip BackendStorage traversal. Amortizes epoch
// check to every 4096 messages (same as DC-local). Reads/writes AtomicU64
// with Relaxed ordering (plain MOV on x86 — same-thread guarantee).
//
// Pointer reuse in LocalState (these fields are unused for DirectChannel):
//   cached_header_ptr → reinterpreted as *const AtomicU64 → DirectSlot.head
//   cached_seq_ptr    → reinterpreted as *const AtomicU64 → DirectSlot.tail
//   cached_data_ptr   → DirectSlot.buffer base pointer
//   cached_capacity / cached_capacity_mask → from DirectSlot

#[inline(always)]
pub(super) fn send_direct_channel_cached<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
    msg: T,
) -> Result<(), T> {
    let local = topic.local();
    // SAFETY: For DC-cached, set_dispatch_fn_ptrs repurposes these pointers:
    //   cached_header_ptr → &DirectSlot.head (AtomicU64)
    //   cached_seq_ptr    → &DirectSlot.tail (AtomicU64)
    // Both are valid for the lifetime of the Arc<DirectSlot> in BackendStorage.
    // Relaxed ordering is sufficient: same-thread guarantee means no
    // cross-thread race on these atomics (they serve as shared counters
    // between two same-thread Topic instances via Arc).
    let head_ptr = local.cached_header_ptr as *const AtomicU64;
    let tail_ptr = local.cached_seq_ptr as *const AtomicU64;

    // SAFETY: head_ptr/tail_ptr are valid AtomicU64 pointers (see block SAFETY above).
    let head = unsafe { (*head_ptr).load(Ordering::Relaxed) };
    // SAFETY: tail_ptr is a valid *const AtomicU64 derived from &DirectSlot.tail,
    // which is kept alive for the lifetime of the Arc<DirectSlot> in BackendStorage.
    let tail = unsafe { (*tail_ptr).load(Ordering::Relaxed) };
    if head.wrapping_sub(tail) >= local.cached_capacity {
        return Err(msg);
    }
    // SAFETY: cached_data_ptr points to DirectSlot's buffer. index < capacity via mask.
    unsafe {
        let base = local.cached_data_ptr as *mut T;
        std::ptr::write(base.add((head & local.cached_capacity_mask) as usize), msg);
        (*head_ptr).store(head.wrapping_add(1), Ordering::Relaxed);
    }

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
        topic.check_migration_periodic();
    }
    Ok(())
}

#[inline(always)]
pub(super) fn recv_direct_channel_cached<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
) -> Option<T> {
    let local = topic.local();
    // SAFETY: Same pointer reinterpretation as send_direct_channel_cached
    // (see that function's SAFETY comment for the full invariant).
    let head_ptr = local.cached_header_ptr as *const AtomicU64;
    let tail_ptr = local.cached_seq_ptr as *const AtomicU64;

    // SAFETY: tail_ptr/head_ptr are valid AtomicU64 pointers (see block SAFETY above).
    let tail = unsafe { (*tail_ptr).load(Ordering::Relaxed) };
    // SAFETY: head_ptr is a valid *const AtomicU64 derived from &DirectSlot.head,
    // which is kept alive for the lifetime of the Arc<DirectSlot> in BackendStorage.
    let head = unsafe { (*head_ptr).load(Ordering::Relaxed) };
    if head.wrapping_sub(tail) == 0 {
        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
            topic.check_migration_periodic();
        }
        return None;
    }
    // SAFETY: cached_data_ptr points to DirectSlot's buffer. index < capacity via mask.
    let msg = unsafe {
        let base = local.cached_data_ptr as *const T;
        std::ptr::read(base.add((tail & local.cached_capacity_mask) as usize))
    };
    // SAFETY: tail_ptr is &DirectSlot.tail (valid AtomicU64).
    unsafe { (*tail_ptr).store(tail.wrapping_add(1), Ordering::Relaxed) };

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
        topic.check_migration_periodic();
    }
    Some(msg)
}

// ---------------------------------------------------------------------------
// 2-5. Intra-process heap ring send functions (macro-generated)
// ---------------------------------------------------------------------------

intra_send_fn!(send_spsc_intra, SpscIntra);
intra_send_fn!(send_spmc_intra, SpmcIntra);
intra_send_fn!(send_mpsc_intra, MpscIntra);
intra_send_fn!(send_mpmc_intra, MpmcIntra);

// ---------------------------------------------------------------------------
// 6. SHM single-producer POD (SpscShm / SpmcShm)
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_sp_pod<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &RingTopic<T>,
    msg: T,
) -> Result<(), T> {
    epoch_guard_send!(topic, msg);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_producer/ensure_consumer). Valid for Topic's lifetime.
    let header = unsafe { &*local.cached_header_ptr };

    let seq = local.local_head;
    if unlikely(seq.wrapping_sub(local.local_tail) >= local.cached_capacity) {
        local.local_tail = header.tail.load(Ordering::Acquire);
        if seq.wrapping_sub(local.local_tail) >= local.cached_capacity {
            return Err(msg);
        }
    }

    let index = (seq & local.cached_capacity_mask) as usize;
    // SAFETY: cached_data_ptr points to SHM data region. index < capacity (mask).
    // simd_aware_write handles alignment (slot_size is rounded to align_of::<T>()).
    unsafe {
        let base = local.cached_data_ptr as *mut T;
        simd_aware_write(base.add(index), msg);
    }
    let new_seq = seq.wrapping_add(1);
    local.local_head = new_seq;
    header.sequence_or_head.store(new_seq, Ordering::Release);

    housekeep_lease!(local, topic);
    Ok(())
}

// ---------------------------------------------------------------------------
// 7. SHM multi-producer POD (MpscShm / MpmcShm)
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_mp_pod<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &RingTopic<T>,
    msg: T,
) -> Result<(), T> {
    epoch_guard_send!(topic, msg);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_producer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;
    let capacity = local.cached_capacity;

    // Non-binding capacity gate: reject if ring appears full.
    // This is optimistic — another publisher may claim a slot between our check
    // and fetch_add, but with capacity=256 and typical 2-8 publishers, the chance
    // of overshooting by more than N_pubs slots is negligible.
    let current_head = header.sequence_or_head.load(Ordering::Acquire);
    if current_head.wrapping_sub(local.local_tail) >= capacity {
        local.local_tail = header.tail.load(Ordering::Acquire);
        if current_head.wrapping_sub(local.local_tail) >= capacity {
            return Err(msg);
        }
    }

    // Claim slot via fetch_add — always succeeds in a single atomic op.
    // Previous CAS loop could spin for 100s of µs under contention.
    let seq = header.sequence_or_head.fetch_add(1, Ordering::AcqRel);

    let index = (seq & mask) as usize;
    // SAFETY: cached_data_ptr points to SHM data region. index < capacity (mask).
    // cached_seq_ptr points to per-slot ready-flag array. index*8 is within bounds.
    // simd_aware_write handles alignment. Release store publishes data to consumers.
    unsafe {
        let base = local.cached_data_ptr as *mut T;
        simd_aware_write(base.add(index), msg);
        let ready_ptr =
            &*(local.cached_seq_ptr.add(index * 8) as *const std::sync::atomic::AtomicU64);
        ready_ptr.store(seq.wrapping_add(1), Ordering::Release);
    }
    local.local_head = seq + 1;

    housekeep_lease!(local, topic);
    Ok(())
}

// ---------------------------------------------------------------------------
// 8. PodShm broadcast send — no backpressure
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_pod_broadcast<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
    msg: T,
) -> Result<(), T> {
    epoch_guard_send!(topic, msg);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_producer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    let seq = header.sequence_or_head.fetch_add(1, Ordering::AcqRel);
    let index = (seq & mask) as usize;
    // SAFETY: cached_seq_ptr points to per-slot ready-flag array in SHM. index*8 is
    // within bounds (index < capacity). cached_data_ptr points to SHM data region.
    // simd_aware_write handles alignment. Release store publishes data to consumers.
    unsafe {
        let ready_ptr =
            &*(local.cached_seq_ptr.add(index * 8) as *const std::sync::atomic::AtomicU64);
        let base = local.cached_data_ptr as *mut T;
        simd_aware_write(base.add(index), msg);
        ready_ptr.store(seq.wrapping_add(1), Ordering::Release);
    }
    local.local_head = seq + 1;

    housekeep_lease!(local, topic);
    Ok(())
}

// ---------------------------------------------------------------------------
// 9. SHM single-producer serde (SpscShm / SpmcShm, non-POD)
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_sp_serde<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &RingTopic<T>,
    msg: T,
) -> Result<(), T> {
    epoch_guard_send!(topic, msg);

    let bytes = match bincode::serialize(&msg) {
        Ok(b) => b,
        Err(_) => return Err(msg),
    };

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_producer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };
    let seq = local.local_head;
    let slot_size = local.slot_size;
    let mask = local.cached_capacity_mask;

    if unlikely(seq.wrapping_sub(local.local_tail) >= local.cached_capacity) {
        local.local_tail = header.tail.load(Ordering::Acquire);
        if seq.wrapping_sub(local.local_tail) >= local.cached_capacity {
            return Err(msg);
        }
    }

    let index = (seq & mask) as usize;
    let slot_offset = index * slot_size;
    let max_data_len = slot_size.saturating_sub(16);

    if bytes.len() > SPILL_THRESHOLD {
        // ── Spill path: large message → TensorPool ──────────────────────
        // SpillDescriptor is 40 bytes; slot must have at least 48 usable
        // (8B ready flag + 40B descriptor). Minimum slot_size is 64, so
        // this always holds, but guard against pathological configs.
        if slot_size < 48 {
            return Err(msg);
        }
        let spill_result = spill_to_pool(topic, &bytes);
        let spill = match spill_result {
            Some(s) => s,
            None => {
                log::warn!(
                    "Topic '{}': spill to pool failed for {} bytes, falling back to auto-grow",
                    topic.name(), bytes.len()
                );
                // Fall back to auto-grow if pool alloc fails
                if bytes.len() > max_data_len {
                    let _ = topic.auto_grow_slot_size(bytes.len());
                    return Err(msg);
                }
                // If it fits inline despite being above threshold, just inline it
                // (this shouldn't happen, but handle gracefully)
                unsafe {
                    let slot_ptr = local.cached_data_ptr.add(slot_offset);
                    let len_ptr = slot_ptr.add(8) as *mut u64;
                    std::ptr::write_volatile(len_ptr, bytes.len() as u64);
                    let data_ptr = slot_ptr.add(16);
                    std::ptr::copy_nonoverlapping(bytes.as_ptr(), data_ptr, bytes.len());
                }
                std::sync::atomic::fence(Ordering::Release);
                let new_seq = seq.wrapping_add(1);
                local.local_head = new_seq;
                header.sequence_or_head.store(new_seq, Ordering::Release);
                housekeep_lease!(local, topic);
                return Ok(());
            }
        };
        // Write SpillDescriptor into ring buffer slot (40 bytes at offset +8)
        // SAFETY: cached_data_ptr + slot_offset is a valid slot. SpillDescriptor
        // is 40 bytes which fits in usable slot space (slot_size - 8 >= 40).
        unsafe {
            let slot_ptr = local.cached_data_ptr.add(slot_offset);
            std::ptr::copy_nonoverlapping(
                &spill as *const SpillDescriptor as *const u8,
                slot_ptr.add(8),
                std::mem::size_of::<SpillDescriptor>(),
            );
        }
        std::sync::atomic::fence(Ordering::Release);
        let new_seq = seq.wrapping_add(1);
        local.local_head = new_seq;
        header.sequence_or_head.store(new_seq, Ordering::Release);
        housekeep_lease!(local, topic);
        return Ok(());
    }

    if bytes.len() > max_data_len {
        // ── Auto-grow path: message fits threshold but exceeds current slot ─
        if !topic.auto_grow_slot_size(bytes.len()) {
            log::warn!(
                "Topic: serialized message ({} bytes) exceeds slot limit ({} bytes). \
                 Auto-grow failed. Use Topic::with_capacity(name, cap, Some(slot_size)).",
                bytes.len(),
                max_data_len,
            );
        }
        return Err(msg);
    }

    // ── Inline path: small message → write directly to ring buffer slot ─
    // SAFETY: cached_data_ptr + slot_offset points to a valid slot within the SHM data
    // region. slot_offset < capacity * slot_size (index < capacity via mask). The slot
    // layout is [8B ready | 8B length | data...], and bytes.len() <= max_data_len.
    unsafe {
        let slot_ptr = local.cached_data_ptr.add(slot_offset);
        let len_ptr = slot_ptr.add(8) as *mut u64;
        std::ptr::write_volatile(len_ptr, bytes.len() as u64);
        let data_ptr = slot_ptr.add(16);
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), data_ptr, bytes.len());
    }
    // Ensure volatile data writes are visible before publishing the new sequence.
    // The Release store below provides this on most architectures, but the explicit
    // fence matches the pattern in send_shm_mp_serde for formal correctness.
    std::sync::atomic::fence(Ordering::Release);

    let new_seq = seq.wrapping_add(1);
    local.local_head = new_seq;
    header.sequence_or_head.store(new_seq, Ordering::Release);

    housekeep_lease!(local, topic);
    Ok(())
}

// ---------------------------------------------------------------------------
// 10. SHM multi-producer serde (MpscShm / MpmcShm, non-POD)
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_mp_serde<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &RingTopic<T>,
    msg: T,
) -> Result<(), T> {
    epoch_guard_send!(topic, msg);

    let bytes = match bincode::serialize(&msg) {
        Ok(b) => b,
        Err(_) => return Err(msg),
    };

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_producer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };
    let slot_size = local.slot_size;
    let mask = local.cached_capacity_mask;
    let capacity = local.cached_capacity;

    let current_head = header.sequence_or_head.load(Ordering::Acquire);
    if current_head.wrapping_sub(local.local_tail) >= capacity {
        local.local_tail = header.tail.load(Ordering::Acquire);
        if current_head.wrapping_sub(local.local_tail) >= capacity {
            return Err(msg);
        }
    }

    let max_data_len = slot_size.saturating_sub(16);

    // ── Spill check (before CAS — can't unclaim a slot after fetch_add) ──
    let spill_desc = if bytes.len() > SPILL_THRESHOLD {
        if slot_size < 48 {
            return Err(msg);
        }
        match spill_to_pool(topic, &bytes) {
            Some(s) => Some(s),
            None => {
                log::warn!(
                    "Topic '{}': spill to pool failed for {} bytes, falling back to auto-grow",
                    topic.name(), bytes.len()
                );
                if bytes.len() > max_data_len {
                    let _ = topic.auto_grow_slot_size(bytes.len());
                    return Err(msg);
                }
                None // will inline below
            }
        }
    } else if bytes.len() > max_data_len {
        // Small message but exceeds current slot — auto-grow
        if !topic.auto_grow_slot_size(bytes.len()) {
            log::warn!(
                "Topic: serialized message ({} bytes) exceeds slot limit ({} bytes). \
                 Auto-grow failed. Use Topic::with_capacity(name, cap, Some(slot_size)).",
                bytes.len(),
                max_data_len,
            );
        }
        return Err(msg);
    } else {
        None // inline path
    };

    let seq = header.sequence_or_head.fetch_add(1, Ordering::AcqRel);
    let index = (seq & mask) as usize;
    let slot_offset = index * slot_size;

    // SAFETY: cached_data_ptr + slot_offset points to a valid slot within the SHM data
    // region. index < capacity via mask. Release fence + store publish data to consumers.
    unsafe {
        let slot_ptr = local.cached_data_ptr.add(slot_offset);
        if let Some(ref spill) = spill_desc {
            // Write SpillDescriptor (40 bytes at offset +8)
            std::ptr::copy_nonoverlapping(
                spill as *const SpillDescriptor as *const u8,
                slot_ptr.add(8),
                std::mem::size_of::<SpillDescriptor>(),
            );
        } else {
            // Normal inline write: [8B ready | 8B length | data...]
            let len_ptr = slot_ptr.add(8) as *mut u64;
            std::ptr::write_volatile(len_ptr, bytes.len() as u64);
            let data_ptr = slot_ptr.add(16);
            std::ptr::copy_nonoverlapping(bytes.as_ptr(), data_ptr, bytes.len());
        }
        std::sync::atomic::fence(Ordering::Release);
        let ready_ptr = &*(slot_ptr as *const std::sync::atomic::AtomicU64);
        ready_ptr.store(seq.wrapping_add(1), Ordering::Release);
    }
    local.local_head = seq + 1;

    housekeep_lease!(local, topic);
    Ok(())
}

// ===========================================================================
// CO-LOCATED SLOT SEND FUNCTIONS (small POD types, sizeof(T) + 8 <= 64)
// ===========================================================================

const COLO_STRIDE: usize = 64;

/// Get the inline sequence number (AtomicU64) at the start of a co-located slot.
///
/// # Safety
/// `data_ptr` must point to the SHM data region. `index` must be < capacity
/// (guaranteed by `seq & capacity_mask`). The slot at `index * 64` must be
/// within the mmap'd region.
#[inline(always)]
unsafe fn colo_seq(data_ptr: *mut u8, index: usize) -> &'static std::sync::atomic::AtomicU64 {
    &*(data_ptr.add(index * COLO_STRIDE) as *const std::sync::atomic::AtomicU64)
}

/// Get a pointer to the data portion of a co-located slot (offset +8 from slot start).
///
/// # Safety
/// Same requirements as `colo_seq`. Additionally, `T` must fit in 56 bytes
/// (sizeof(T) + 8 <= 64, verified at dispatch time by `set_dispatch_fn_ptrs`).
#[inline(always)]
unsafe fn colo_data<T>(data_ptr: *mut u8, index: usize) -> *mut T {
    data_ptr.add(index * COLO_STRIDE + 8) as *mut T
}

// ---------------------------------------------------------------------------
// 11. SHM single-producer POD co-located (SpscShm / SpmcShm)
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_sp_pod_colo<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
    msg: T,
) -> Result<(), T> {
    epoch_guard_send!(topic, msg);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_producer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };

    let seq = local.local_head;
    if unlikely(seq.wrapping_sub(local.local_tail) >= local.cached_capacity) {
        local.local_tail = header.tail.load(Ordering::Acquire);
        if seq.wrapping_sub(local.local_tail) >= local.cached_capacity {
            return Err(msg);
        }
    }

    let index = (seq & local.cached_capacity_mask) as usize;
    // SAFETY: cached_data_ptr points to co-located SHM data region. index < capacity
    // (mask). colo_data/colo_seq access the 64-byte slot at index*64. T fits in 56 bytes
    // (verified by set_dispatch_fn_ptrs). Release store publishes data to consumers.
    unsafe {
        std::ptr::write(colo_data::<T>(local.cached_data_ptr, index), msg);
        colo_seq(local.cached_data_ptr, index).store(seq.wrapping_add(1), Ordering::Release);
    }
    let new_seq = seq.wrapping_add(1);
    local.local_head = new_seq;
    // Batch header.sequence_or_head updates: store every 32 messages.
    // Co-located recv functions poll per-slot colo_seq instead of this header
    // field, so it's only needed for: (a) MPSC recv detecting messages during
    // SPSC→MPSC migration, (b) MP producers using CAS to claim slots.
    // handle_epoch_change uses fetch_max to flush the accurate local_head
    // before migration, preventing stale CAS starts.
    if new_seq & 0x1F == 0 {
        header.sequence_or_head.store(new_seq, Ordering::Release);
    }

    housekeep_lease!(local, topic);
    Ok(())
}

// ---------------------------------------------------------------------------
// 12. SHM multi-producer POD co-located (MpscShm / MpmcShm)
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_mp_pod_colo<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
    msg: T,
) -> Result<(), T> {
    epoch_guard_send!(topic, msg);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_producer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;
    let capacity = local.cached_capacity;

    // Non-binding capacity gate (same pattern as send_shm_mp_pod).
    let current_head = header.sequence_or_head.load(Ordering::Acquire);
    if current_head.wrapping_sub(local.local_tail) >= capacity {
        local.local_tail = header.tail.load(Ordering::Acquire);
        if current_head.wrapping_sub(local.local_tail) >= capacity {
            return Err(msg);
        }
    }

    // Claim slot via fetch_add — single atomic op, no CAS retry loop.
    let seq = header.sequence_or_head.fetch_add(1, Ordering::AcqRel);

    let index = (seq & mask) as usize;
    // SAFETY: cached_data_ptr points to co-located SHM data region. index < capacity
    // (mask). colo_data/colo_seq access the 64-byte slot at index*64. T fits in 56 bytes
    // (verified by set_dispatch_fn_ptrs). Release store publishes data to consumers.
    unsafe {
        std::ptr::write(colo_data::<T>(local.cached_data_ptr, index), msg);
        colo_seq(local.cached_data_ptr, index).store(seq.wrapping_add(1), Ordering::Release);
    }
    local.local_head = seq + 1;

    housekeep_lease!(local, topic);
    Ok(())
}

// ---------------------------------------------------------------------------
// 13. PodShm broadcast co-located
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_pod_broadcast_colo<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
    msg: T,
) -> Result<(), T> {
    epoch_guard_send!(topic, msg);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_producer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    let seq = header.sequence_or_head.fetch_add(1, Ordering::AcqRel);
    let index = (seq & mask) as usize;
    // SAFETY: cached_data_ptr points to co-located SHM data region. index < capacity
    // (mask). colo_data/colo_seq access the 64-byte slot at index*64. T fits in 56 bytes
    // (verified by set_dispatch_fn_ptrs). Release store publishes data to consumers.
    unsafe {
        std::ptr::write(colo_data::<T>(local.cached_data_ptr, index), msg);
        colo_seq(local.cached_data_ptr, index).store(seq.wrapping_add(1), Ordering::Release);
    }
    local.local_head = seq + 1;

    housekeep_lease!(local, topic);
    Ok(())
}

// ---------------------------------------------------------------------------
// 14. Uninitialized send — first call, register then re-dispatch
// ---------------------------------------------------------------------------

#[cold]
#[inline(never)]
pub(super) fn send_uninitialized<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
    msg: T,
) -> Result<(), T> {
    if topic.ensure_producer().is_err() {
        return Err(msg);
    }
    // SAFETY: ensure_producer() → initialize_backend() → set_dispatch_fn_ptrs()
    // has set send_fn to a valid function pointer. UnsafeCell is single-thread.
    unsafe { (*topic.send_fn.get())(topic, msg) }
}

// ============================================================================
// RECV FUNCTIONS — complete recv path (epoch + ring op + housekeeping)
// ============================================================================

// ---------------------------------------------------------------------------
// 2-5. Intra-process heap ring recv functions (macro-generated)
// ---------------------------------------------------------------------------

intra_recv_fn!(recv_spsc_intra, SpscIntra);
intra_recv_fn!(recv_spmc_intra, SpmcIntra);
intra_recv_fn!(recv_mpsc_intra, MpscIntra);
intra_recv_fn!(recv_mpmc_intra, MpmcIntra);

// ---------------------------------------------------------------------------
// 6. SHM SpscShm POD recv
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_spsc_pod<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &RingTopic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_consumer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };

    let tail = local.local_tail;
    let mask = local.cached_capacity_mask;
    if local.local_head.wrapping_sub(tail) == 0 {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if local.local_head.wrapping_sub(tail) == 0 {
            housekeep_epoch!(local, topic);
            return None;
        }
    }

    // SAFETY: cached_data_ptr points to SHM data region. index < capacity (mask).
    // The producer's Release store on sequence_or_head was observed via our Acquire load.
    // simd_aware_read handles alignment (slot_size rounded to align_of::<T>()).
    let msg = unsafe {
        let base = local.cached_data_ptr as *const T;
        simd_aware_read(base.add((tail & mask) as usize))
    };
    let new_tail = tail.wrapping_add(1);
    local.local_tail = new_tail;
    // Batch header.tail updates: only store every 32 messages.
    // The producer reads header.tail for backpressure only when its local head
    // is >= capacity ahead of its cached tail. With capacity 256 and batch 32,
    // there's 224 slots of headroom. This eliminates the Release store to a
    // separate cache line on 31 of every 32 recvs.
    // handle_epoch_change flushes local.local_tail before migration.
    if new_tail & 0x1F == 0 {
        header.tail.store(new_tail, Ordering::Release);
    }

    housekeep_lease!(local, topic);
    Some(msg)
}

// ---------------------------------------------------------------------------
// 7. SHM MpscShm POD recv
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpsc_pod<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &RingTopic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_consumer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };

    let tail = local.local_tail;
    let mask = local.cached_capacity_mask;
    if local.local_head.wrapping_sub(tail) == 0 {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if local.local_head.wrapping_sub(tail) == 0 {
            housekeep_epoch!(local, topic);
            return None;
        }
    }

    let index = (tail & mask) as usize;
    // SAFETY: cached_seq_ptr points to the per-slot ready-flag array in SHM.
    // index*8 is within bounds (index < capacity, array has capacity entries).
    let ready_ok = unsafe {
        let ready_ptr =
            &*(local.cached_seq_ptr.add(index * 8) as *const std::sync::atomic::AtomicU64);
        ready_ptr.load(Ordering::Acquire) == tail.wrapping_add(1)
    };
    if !ready_ok {
        return None;
    }

    // SAFETY: cached_data_ptr points to SHM data region. index < capacity (mask).
    // The ready flag Acquire load above established happens-before with the producer's
    // Release store, so the slot data is fully written. simd_aware_read handles alignment.
    let msg = unsafe {
        let base = local.cached_data_ptr as *const T;
        simd_aware_read(base.add(index))
    };
    let new_tail = tail.wrapping_add(1);
    local.local_tail = new_tail;
    // Batch header.tail updates: only store every 32 messages.
    // handle_epoch_change flushes local.local_tail before migration.
    if new_tail & 0x1F == 0 {
        header.tail.store(new_tail, Ordering::Release);
    }

    housekeep_lease!(local, topic);
    Some(msg)
}

// ---------------------------------------------------------------------------
// 8. SHM SpmcShm POD recv — CAS on tail
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_spmc_pod<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &RingTopic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_consumer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    // Bounded retry: with N consumers, CAS fails at most N-1 times per message.
    // 8 retries covers up to 8 competing consumers; if all fail, return None
    // and let the caller retry on next poll (avoids unbounded spinning).
    for _attempt in 0..8 {
        let tail = header.tail.load(Ordering::Acquire);
        if local.local_head.wrapping_sub(tail) == 0 {
            local.local_head = header.sequence_or_head.load(Ordering::Acquire);
            if local.local_head.wrapping_sub(tail) == 0 {
                local.msg_counter = local.msg_counter.wrapping_add(1);
                if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                    topic.check_migration_periodic();
                }
                return None;
            }
        }
        if header
            .tail
            .compare_exchange_weak(
                tail,
                tail.wrapping_add(1),
                Ordering::AcqRel,
                Ordering::Relaxed,
            )
            .is_ok()
        {
            // SAFETY: cached_data_ptr points to SHM data region. index < capacity (mask).
            // CAS success means we own this slot. The producer's Release on sequence_or_head
            // was observed via our Acquire load. simd_aware_read handles alignment.
            let msg = unsafe {
                let base = local.cached_data_ptr as *const T;
                simd_aware_read(base.add((tail & mask) as usize))
            };
            local.local_tail = tail.wrapping_add(1);

            housekeep_lease!(local, topic);
            return Some(msg);
        }
        std::hint::spin_loop();
    }
    housekeep_epoch!(local, topic);
    None
}

// ---------------------------------------------------------------------------
// 9. SHM MpmcShm POD recv — bounded spin + CAS
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpmc_pod<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &RingTopic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_consumer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    let tail = header.tail.load(Ordering::Acquire);
    if local.local_head.wrapping_sub(tail) == 0 {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if local.local_head.wrapping_sub(tail) == 0 {
            housekeep_epoch!(local, topic);
            return None;
        }
    }

    let index = (tail & mask) as usize;
    // SAFETY: cached_seq_ptr points to the per-slot ready-flag array in SHM.
    // index*8 is within bounds (index < capacity, array has capacity entries).
    let ready_ok = unsafe {
        let ready_ptr =
            &*(local.cached_seq_ptr.add(index * 8) as *const std::sync::atomic::AtomicU64);
        ready_ptr.load(Ordering::Acquire) == tail.wrapping_add(1)
    };
    if !ready_ok {
        return None;
    }

    if header
        .tail
        .compare_exchange_weak(
            tail,
            tail.wrapping_add(1),
            Ordering::AcqRel,
            Ordering::Relaxed,
        )
        .is_ok()
    {
        // SAFETY: cached_data_ptr points to SHM data region. index < capacity (mask).
        // CAS success means we own this slot. Ready flag Acquire load above established
        // happens-before with the producer's Release store. simd_aware_read handles alignment.
        let msg = unsafe {
            let base = local.cached_data_ptr as *const T;
            simd_aware_read(base.add(index))
        };
        local.local_tail = tail.wrapping_add(1);

        housekeep_lease!(local, topic);
        return Some(msg);
    }
    None
}

// ---------------------------------------------------------------------------
// 10. PodShm broadcast recv
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_pod_broadcast<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_consumer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    let mut tail = local.local_tail;
    if local.local_head.wrapping_sub(tail) == 0 {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if local.local_head.wrapping_sub(tail) == 0 {
            housekeep_epoch!(local, topic);
            return None;
        }
    }

    let behind = local.local_head.wrapping_sub(tail);
    if behind > local.cached_capacity {
        tail = local.local_head;
        local.local_tail = tail;
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if local.local_head.wrapping_sub(tail) == 0 {
            return None;
        }
    }

    let index = (tail & mask) as usize;
    // SAFETY: cached_seq_ptr points to per-slot ready-flag array in SHM.
    // index*8 is within bounds (index < capacity, array has capacity entries).
    let ready = unsafe {
        let ready_ptr =
            &*(local.cached_seq_ptr.add(index * 8) as *const std::sync::atomic::AtomicU64);
        let ready_val = ready_ptr.load(Ordering::Acquire);
        ready_val >= tail.wrapping_add(1)
    };
    if !ready {
        return None;
    }

    // SAFETY: cached_data_ptr points to SHM data region. index < capacity (mask).
    // The ready flag Acquire load above established happens-before with the producer's
    // Release store, so the slot data is fully written. simd_aware_read handles alignment.
    let msg = unsafe {
        let base = local.cached_data_ptr as *const T;
        simd_aware_read(base.add(index))
    };
    local.local_tail = tail.wrapping_add(1);

    housekeep_lease!(local, topic);
    Some(msg)
}

// ---------------------------------------------------------------------------
// 11. SHM SpscShm serde recv
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_spsc_serde<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_consumer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };

    let tail = local.local_tail;
    let mask = local.cached_capacity_mask;
    let slot_size = local.slot_size;

    if local.local_head.wrapping_sub(tail) == 0 {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if local.local_head.wrapping_sub(tail) == 0 {
            housekeep_epoch!(local, topic);
            return None;
        }
    }

    let index = (tail & mask) as usize;
    let slot_offset = index * slot_size;

    // SAFETY: slot_ptr is valid for slot_size bytes within SHM. The producer's
    // sequence store (Release) was observed via our Acquire load on sequence_or_head.
    let msg = unsafe { read_serde_slot::<T>(local.cached_data_ptr.add(slot_offset), slot_size, topic.name()) }?;

    let new_tail = tail.wrapping_add(1);
    local.local_tail = new_tail;
    header.tail.store(new_tail, Ordering::Release);

    housekeep_lease!(local, topic);
    Some(msg)
}

// ---------------------------------------------------------------------------
// 12. SHM MpscShm serde recv
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpsc_serde<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_consumer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };

    let tail = local.local_tail;
    let mask = local.cached_capacity_mask;
    let slot_size = local.slot_size;

    if local.local_head.wrapping_sub(tail) == 0 {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if local.local_head.wrapping_sub(tail) == 0 {
            housekeep_epoch!(local, topic);
            return None;
        }
    }

    let index = (tail & mask) as usize;
    let slot_offset = index * slot_size;

    // SAFETY: slot_ptr points to a valid serde slot within SHM data region.
    // The first 8 bytes are the ready flag (AtomicU64).
    let ready_ok = unsafe {
        let slot_ptr = local.cached_data_ptr.add(slot_offset);
        let ready_ptr = &*(slot_ptr as *const std::sync::atomic::AtomicU64);
        ready_ptr.load(Ordering::Acquire) == tail.wrapping_add(1)
    };
    if !ready_ok {
        return None;
    }

    // SAFETY: slot_ptr is valid for slot_size bytes within SHM. Ready flag
    // was verified above, so the producer has finished writing this slot.
    let msg = unsafe { read_serde_slot::<T>(local.cached_data_ptr.add(slot_offset), slot_size, topic.name()) }?;

    let new_tail = tail.wrapping_add(1);
    local.local_tail = new_tail;
    header.tail.store(new_tail, Ordering::Release);

    housekeep_lease!(local, topic);
    Some(msg)
}

// ---------------------------------------------------------------------------
// 13. SHM SpmcShm serde recv — CAS on tail
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_spmc_serde<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_consumer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;
    let slot_size = local.slot_size;

    let tail = header.tail.load(Ordering::Acquire);
    if local.local_head.wrapping_sub(tail) == 0 {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if local.local_head.wrapping_sub(tail) == 0 {
            housekeep_epoch!(local, topic);
            return None;
        }
    }

    if header
        .tail
        .compare_exchange_weak(
            tail,
            tail.wrapping_add(1),
            Ordering::AcqRel,
            Ordering::Relaxed,
        )
        .is_ok()
    {
        let index = (tail & mask) as usize;
        let slot_offset = index * slot_size;

        // SAFETY: CAS succeeded so we own this slot. slot_ptr is valid for
        // slot_size bytes within the SHM data region.
        let msg = match unsafe {
            read_serde_slot::<T>(local.cached_data_ptr.add(slot_offset), slot_size, topic.name())
        } {
            Some(m) => m,
            None => {
                local.local_tail = tail.wrapping_add(1);
                return None; // Corrupted length — skip this slot
            }
        };

        local.local_tail = tail.wrapping_add(1);

        housekeep_lease!(local, topic);
        return Some(msg);
    }
    housekeep_epoch!(local, topic);
    None
}

// ---------------------------------------------------------------------------
// 14. SHM MpmcShm serde recv — bounded spin + CAS
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpmc_serde<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_consumer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;
    let slot_size = local.slot_size;

    let tail = header.tail.load(Ordering::Acquire);
    if local.local_head.wrapping_sub(tail) == 0 {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if local.local_head.wrapping_sub(tail) == 0 {
            housekeep_epoch!(local, topic);
            return None;
        }
    }

    let index = (tail & mask) as usize;
    let slot_offset = index * slot_size;

    // SAFETY: slot_ptr points to a valid serde slot within SHM data region.
    // The first 8 bytes are the ready flag (AtomicU64).
    let ready_ok = unsafe {
        let slot_ptr = local.cached_data_ptr.add(slot_offset);
        let ready_ptr = &*(slot_ptr as *const std::sync::atomic::AtomicU64);
        ready_ptr.load(Ordering::Acquire) == tail.wrapping_add(1)
    };
    if !ready_ok {
        return None;
    }

    if header
        .tail
        .compare_exchange_weak(
            tail,
            tail.wrapping_add(1),
            Ordering::AcqRel,
            Ordering::Relaxed,
        )
        .is_ok()
    {
        // SAFETY: CAS succeeded so we own this slot. Ready flag was verified
        // above, so the producer has finished writing.
        let msg = match unsafe {
            read_serde_slot::<T>(local.cached_data_ptr.add(slot_offset), slot_size, topic.name())
        } {
            Some(m) => m,
            None => {
                local.local_tail = tail.wrapping_add(1);
                return None; // Corrupted length — skip this slot
            }
        };

        local.local_tail = tail.wrapping_add(1);

        housekeep_lease!(local, topic);
        return Some(msg);
    }
    None
}

// ===========================================================================
// CO-LOCATED SLOT RECV FUNCTIONS
// ===========================================================================

// ---------------------------------------------------------------------------
// 15. SHM SpscShm POD co-located recv — fast local check + single-cache-line poll
// ---------------------------------------------------------------------------
//
// Two paths:
// 1. FAST PATH (tail < cached_head): Data was already committed in a previous
//    Acquire check. Read directly from the data region. 1 cache miss per msg.
// 2. SLOW PATH (caught up): Poll colo_seq instead of header.sequence_or_head.
//    Since colo_seq and colo_data share the SAME 64-byte cache line, detecting
//    readiness AND reading data costs ONE cache miss instead of TWO.
//    Empty polls return in ~3ns via L1-cached local state (no SHM access).

#[inline(always)]
pub(super) fn recv_shm_spsc_pod_colo<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    let tail = local.local_tail;
    let index = (tail & local.cached_capacity_mask) as usize;

    if local.local_head.wrapping_sub(tail) == 0 {
        // Caught up with cached head. Poll per-slot seq for new data.
        // colo_seq and colo_data share the SAME cache line, so detecting
        // readiness AND reading data costs ONE cache miss instead of polling
        // header.sequence_or_head (separate cache line) + reading data (another miss).
        // SAFETY: cached_data_ptr points to co-located SHM data region. index < capacity
        // (mask). colo_seq returns a reference to the inline AtomicU64 at slot start.
        let seq_val = unsafe { colo_seq(local.cached_data_ptr, index).load(Ordering::Acquire) };
        let expected = tail.wrapping_add(1);
        if seq_val < expected {
            housekeep_epoch!(local, topic);
            return None;
        }
        // Data is ready. Update cached head so next recv uses the fast path
        // if the producer is ahead.
        local.local_head = seq_val;
    }

    // Read data — either from fast path (data guaranteed valid by previous Acquire)
    // or from slow path (colo_data on same cache line as colo_seq just loaded → L1 hit).
    // SAFETY: cached_data_ptr points to co-located SHM data region. index < capacity
    // (mask). T fits in 56 bytes (verified by set_dispatch_fn_ptrs). The producer's
    // Release on colo_seq was observed via our Acquire load, so slot data is valid.
    let msg = unsafe { std::ptr::read(colo_data::<T>(local.cached_data_ptr, index)) };
    let new_tail = tail.wrapping_add(1);
    local.local_tail = new_tail;

    // Batch header.tail updates: only store every 32 messages.
    // The producer reads header.tail for backpressure only when its local head
    // is >= capacity ahead of its cached tail. With capacity 256 and batch 32,
    // there's 224 slots of headroom. This eliminates the Release store to a
    // separate cache line on 31 of every 32 recvs.
    if new_tail & 0x1F == 0 {
        // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
        // (set by ensure_consumer). Valid for Topic's lifetime. Single-thread access.
        let header = unsafe { &*local.cached_header_ptr };
        header.tail.store(new_tail, Ordering::Release);
    }

    housekeep_lease!(local, topic);
    Some(msg)
}

// ---------------------------------------------------------------------------
// 16. SHM MpscShm POD co-located recv — bounded spin on inline seq
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpsc_pod_colo<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_consumer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };

    let tail = local.local_tail;
    let mask = local.cached_capacity_mask;
    if local.local_head.wrapping_sub(tail) == 0 {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if local.local_head.wrapping_sub(tail) == 0 {
            housekeep_epoch!(local, topic);
            return None;
        }
    }

    let index = (tail & mask) as usize;
    // SAFETY: colo_seq returns a reference to the inline AtomicU64 at the start
    // of the co-located 64-byte slot. index < capacity is guaranteed by mask.
    let ready_ok = unsafe {
        colo_seq(local.cached_data_ptr, index).load(Ordering::Acquire) >= tail.wrapping_add(1)
    };
    if !ready_ok {
        return None;
    }

    // SAFETY: cached_data_ptr points to co-located SHM data region. index < capacity
    // (mask). T fits in 56 bytes (verified by set_dispatch_fn_ptrs). The producer's
    // Release on colo_seq was observed via the Acquire load above, so slot data is valid.
    let msg = unsafe { std::ptr::read(colo_data::<T>(local.cached_data_ptr, index)) };
    let new_tail = tail.wrapping_add(1);
    local.local_tail = new_tail;
    header.tail.store(new_tail, Ordering::Release);

    housekeep_lease!(local, topic);
    Some(msg)
}

// ---------------------------------------------------------------------------
// 17. SHM SpmcShm POD co-located recv — CAS on tail, inline seq check
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_spmc_pod_colo<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_consumer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    // Bounded retry: avoids unbounded spinning under consumer contention.
    for _attempt in 0..8 {
        let tail = header.tail.load(Ordering::Acquire);
        let index = (tail & mask) as usize;

        // SAFETY: cached_data_ptr points to co-located SHM data region. index < capacity
        // (mask). colo_seq returns a reference to the inline AtomicU64 at slot start.
        let seq_val = unsafe { colo_seq(local.cached_data_ptr, index).load(Ordering::Acquire) };
        if seq_val < tail.wrapping_add(1) {
            housekeep_epoch!(local, topic);
            return None;
        }

        if header
            .tail
            .compare_exchange_weak(
                tail,
                tail.wrapping_add(1),
                Ordering::AcqRel,
                Ordering::Relaxed,
            )
            .is_ok()
        {
            // SAFETY: cached_data_ptr points to co-located SHM data region. index < capacity
            // (mask). CAS success means we own this slot. colo_seq Acquire load above
            // established happens-before with the producer's Release. T fits in 56 bytes.
            let msg = unsafe { std::ptr::read(colo_data::<T>(local.cached_data_ptr, index)) };
            local.local_tail = tail.wrapping_add(1);

            housekeep_lease!(local, topic);
            return Some(msg);
        }
        std::hint::spin_loop();
    }
    housekeep_epoch!(local, topic);
    None
}

// ---------------------------------------------------------------------------
// 18. SHM MpmcShm POD co-located recv — bounded spin + CAS
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpmc_pod_colo<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_consumer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    let tail = header.tail.load(Ordering::Acquire);
    if local.local_head.wrapping_sub(tail) == 0 {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if local.local_head.wrapping_sub(tail) == 0 {
            housekeep_epoch!(local, topic);
            return None;
        }
    }

    let index = (tail & mask) as usize;
    // SAFETY: colo_seq returns a reference to the inline AtomicU64 at the start
    // of the co-located 64-byte slot. index < capacity is guaranteed by mask.
    let ready_ok = unsafe {
        colo_seq(local.cached_data_ptr, index).load(Ordering::Acquire) >= tail.wrapping_add(1)
    };
    if !ready_ok {
        housekeep_epoch!(local, topic);
        return None;
    }

    if header
        .tail
        .compare_exchange_weak(
            tail,
            tail.wrapping_add(1),
            Ordering::AcqRel,
            Ordering::Relaxed,
        )
        .is_ok()
    {
        // SAFETY: cached_data_ptr points to co-located SHM data region. index < capacity
        // (mask). CAS success means we own this slot. colo_seq Acquire load above
        // established happens-before with the producer's Release. T fits in 56 bytes.
        let msg = unsafe { std::ptr::read(colo_data::<T>(local.cached_data_ptr, index)) };
        local.local_tail = tail.wrapping_add(1);

        housekeep_lease!(local, topic);
        return Some(msg);
    }
    housekeep_epoch!(local, topic);
    None
}

// ---------------------------------------------------------------------------
// 19. PodShm broadcast co-located recv
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_pod_broadcast_colo<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    // SAFETY: cached_header_ptr points to TopicHeader at the start of ShmRegion
    // (set by ensure_consumer). Valid for Topic's lifetime. Single-thread access.
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    let mut tail = local.local_tail;
    if local.local_head.wrapping_sub(tail) == 0 {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if local.local_head.wrapping_sub(tail) == 0 {
            housekeep_epoch!(local, topic);
            return None;
        }
    }

    let behind = local.local_head.wrapping_sub(tail);
    if behind > local.cached_capacity {
        tail = local.local_head;
        local.local_tail = tail;
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if local.local_head.wrapping_sub(tail) == 0 {
            return None;
        }
    }

    let index = (tail & mask) as usize;
    // SAFETY: cached_data_ptr points to co-located SHM data region. index < capacity
    // (mask). colo_seq returns a reference to the inline AtomicU64 at slot start.
    let ready = unsafe {
        let seq_val = colo_seq(local.cached_data_ptr, index).load(Ordering::Acquire);
        seq_val >= tail.wrapping_add(1)
    };
    if !ready {
        return None;
    }

    // SAFETY: cached_data_ptr points to co-located SHM data region. index < capacity
    // (mask). T fits in 56 bytes (verified by set_dispatch_fn_ptrs). The producer's
    // Release on colo_seq was observed via the Acquire load above, so slot data is valid.
    let msg = unsafe { std::ptr::read(colo_data::<T>(local.cached_data_ptr, index)) };
    local.local_tail = tail.wrapping_add(1);

    housekeep_lease!(local, topic);
    Some(msg)
}

// ---------------------------------------------------------------------------
// 20. Uninitialized recv — first call, register then re-dispatch
// ---------------------------------------------------------------------------

#[cold]
#[inline(never)]
pub(super) fn recv_uninitialized<
    T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
>(
    topic: &RingTopic<T>,
) -> Option<T> {
    if topic.ensure_consumer().is_err() {
        return None;
    }
    // SAFETY: ensure_consumer() → initialize_backend() → set_dispatch_fn_ptrs()
    // has set recv_fn to a valid function pointer. UnsafeCell is single-thread.
    unsafe { (*topic.recv_fn.get())(topic) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_spill_descriptor_size() {
        assert_eq!(
            std::mem::size_of::<SpillDescriptor>(),
            40,
            "SpillDescriptor must be exactly 40 bytes"
        );
        // Must fit in minimum ring buffer slot (64B - 8B ready flag = 56B usable)
        assert!(
            std::mem::size_of::<SpillDescriptor>() <= 56,
            "SpillDescriptor must fit in minimum 64B slot"
        );
    }

    #[test]
    fn test_spill_sentinel_above_max_slot() {
        // SPILL_SENTINEL must be larger than any valid message length (max 1MB)
        assert!(
            SPILL_SENTINEL > 1024 * 1024,
            "sentinel must exceed max slot size to avoid false positives"
        );
    }

    #[test]
    fn test_spill_descriptor_roundtrip() {
        let tensor = crate::types::Tensor::new(
            42,
            7,
            123456789,
            1024,
            &[8192],
            crate::types::TensorDtype::U8,
            crate::types::Device::cpu(),
        );
        let spill = SpillDescriptor::from_tensor(&tensor, 5000);
        assert_eq!(spill.sentinel, SPILL_SENTINEL);
        assert_eq!(spill.pool_id, 42);
        assert_eq!(spill.slot_id, 7);
        assert_eq!(spill.offset, 1024);
        assert_eq!(spill.size, 5000);

        let recovered = spill.to_tensor();
        assert_eq!(recovered.pool_id, 42);
        assert_eq!(recovered.slot_id, 7);
        assert_eq!(recovered.offset, 1024);
        assert_eq!(recovered.size, 5000);
        assert_eq!(recovered.dtype, crate::types::TensorDtype::U8);
        assert_eq!(recovered.ndim, 1);
        assert_eq!(recovered.shape[0], 5000);
    }

    #[test]
    fn test_is_spill_slot_detection() {
        // Create a fake slot with spill sentinel
        let mut slot = [0u8; 64];
        // Write sentinel at offset 8 (after ready flag)
        let sentinel_bytes = SPILL_SENTINEL.to_ne_bytes();
        slot[8..16].copy_from_slice(&sentinel_bytes);

        unsafe {
            assert!(is_spill_slot(slot.as_ptr()));
        }

        // Normal slot with small length should NOT be detected as spill
        let mut normal_slot = [0u8; 64];
        let len: u64 = 100;
        normal_slot[8..16].copy_from_slice(&len.to_ne_bytes());

        unsafe {
            assert!(!is_spill_slot(normal_slot.as_ptr()));
        }
    }

    #[test]
    fn test_spill_threshold_sane() {
        assert_eq!(SPILL_THRESHOLD, 4096);
        assert!(SPILL_THRESHOLD >= 1024, "threshold too small — would spill tiny messages");
        assert!(SPILL_THRESHOLD <= 65536, "threshold too large — defeats purpose");
    }

    #[test]
    fn test_spill_descriptor_fits_in_min_slot() {
        // Minimum ring buffer slot is 64 bytes. After 8B ready/sequence,
        // 56 bytes remain. SpillDescriptor must fit in 56 bytes.
        let desc_size = std::mem::size_of::<SpillDescriptor>();
        let min_slot = 64;
        let usable = min_slot - 8; // 8B for ready flag or padding
        assert!(
            desc_size <= usable,
            "SpillDescriptor ({} bytes) must fit in usable slot space ({} bytes)",
            desc_size,
            usable
        );
    }

    #[test]
    fn test_spill_descriptor_sentinel_not_valid_length() {
        // SPILL_SENTINEL must never be a valid serde message length.
        // Max slot size is 1MB. Any valid length is < 1MB.
        let max_slot_size: u64 = 1024 * 1024; // 1MB
        assert!(
            SPILL_SENTINEL > max_slot_size,
            "sentinel {} must be > max slot size {}",
            SPILL_SENTINEL,
            max_slot_size
        );

        // Also verify sentinel doesn't look like a valid bincode length prefix.
        // Bincode encodes Vec<u8> length as u64 LE. A valid 16KB vec has
        // length 0x4000 (16384). Sentinel is 0xDEAD_5911_CAFE_BABE.
        assert_ne!(SPILL_SENTINEL, 16384);
        assert_ne!(SPILL_SENTINEL, 0);
    }

    #[test]
    fn test_read_spill_descriptor_from_slot_bytes() {
        // Simulate a ring buffer slot containing a SpillDescriptor
        let mut slot = [0u8; 64];

        let tensor = crate::types::Tensor::new(
            99, 3, 42, 2048, &[10000],
            crate::types::TensorDtype::U8,
            crate::types::Device::cpu(),
        );
        let spill = SpillDescriptor::from_tensor(&tensor, 10000);

        // Write SpillDescriptor at offset +8 (after ready flag)
        let spill_bytes = unsafe {
            std::slice::from_raw_parts(
                &spill as *const SpillDescriptor as *const u8,
                std::mem::size_of::<SpillDescriptor>(),
            )
        };
        slot[8..8 + spill_bytes.len()].copy_from_slice(spill_bytes);

        // Detect and read
        unsafe {
            assert!(is_spill_slot(slot.as_ptr()));
            let recovered = read_spill_descriptor(slot.as_ptr());
            assert_eq!(recovered.sentinel, SPILL_SENTINEL);
            assert_eq!(recovered.pool_id, 99);
            assert_eq!(recovered.slot_id, 3);
            assert_eq!(recovered.offset, 2048);
            assert_eq!(recovered.size, 10000);
        }
    }
}
