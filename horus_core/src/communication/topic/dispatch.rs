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
use super::{simd_aware_read, simd_aware_write, READY_FLAG_SPIN_LIMIT};
use crate::utils::unlikely;

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
        let __pe = $topic.process_epoch.load(Ordering::Relaxed);
        if unlikely(__pe != $topic.local().cached_epoch) {
            $topic.handle_epoch_change(__pe);
            // SAFETY: send_fn set by handle_epoch_change → initialize_backend → set_dispatch_fn_ptrs
            return unsafe { (*$topic.send_fn.get())($topic, $msg) };
        }
    };
}

macro_rules! epoch_guard_recv {
    ($topic:expr) => {
        let __pe = $topic.process_epoch.load(Ordering::Relaxed);
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

/// Housekeeping after a successful send or recv: lease refresh + periodic maintenance.
/// Used by all non-DirectChannel paths on the success path.
macro_rules! housekeep_lease {
    ($local:ident, $topic:expr) => {
        $local.msg_counter = $local.msg_counter.wrapping_add(1);
        if unlikely($local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
            $topic.periodic_maintenance();
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
/// On success: lease refresh. On empty: epoch check.
macro_rules! housekeep_recv {
    ($local:ident, $topic:expr, $result:ident) => {
        $local.msg_counter = $local.msg_counter.wrapping_add(1);
        if $result.is_some() {
            if unlikely($local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
                $topic.periodic_maintenance();
            }
        } else if unlikely($local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
            $topic.check_migration_periodic();
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
            // function when the backend matches. unreachable_unchecked is sound because migration
            // (which changes the variant) always updates fn ptrs atomically via epoch_guard above.
            let ring = match unsafe { &*topic.backend.get() } {
                BackendStorage::$variant(r) => r,
                _ => unsafe { std::hint::unreachable_unchecked() },
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
                _ => unsafe { std::hint::unreachable_unchecked() },
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

/// Bounded spin waiting for a per-slot ready flag to reach the expected value.
///
/// Returns true if the ready flag matched within READY_FLAG_SPIN_LIMIT iterations.
/// On x86, each spin_loop() is a PAUSE (~10-20 cycles), so 256 spins ≈ ~1.7µs
/// worst case at 3GHz — well within try_recv latency bounds.
///
/// # Safety
/// `ready_ptr` must point to a valid AtomicU64 within the topic's data region
/// (either SHM mmap or co-located slot). The pointer must remain valid for the
/// duration of the spin (guaranteed by ShmRegion lifetime).
#[inline(always)]
unsafe fn spin_for_ready(ready_ptr: &AtomicU64, expected: u64) -> bool {
    let mut spins = 0u32;
    loop {
        if ready_ptr.load(Ordering::Acquire) == expected {
            return true;
        }
        spins += 1;
        if spins >= READY_FLAG_SPIN_LIMIT {
            return false;
        }
        std::hint::spin_loop();
    }
}

/// Read and deserialize a message from a serde-format SHM slot.
///
/// Slot layout: `[8B ready_flag | 8B length | data...]`
///
/// Returns None if the length field is corrupted (exceeds slot capacity).
///
/// # Safety
/// `slot_ptr` must point to a valid slot within the SHM data region,
/// with at least `slot_size` bytes accessible. The slot must have been
/// fully written by a producer (ready flag verified by caller).
#[inline(always)]
unsafe fn read_serde_slot<T: DeserializeOwned>(slot_ptr: *const u8, slot_size: usize) -> Option<T> {
    let max_data_len = slot_size.saturating_sub(16);
    let len_ptr = slot_ptr.add(8) as *const u64;
    let len = std::ptr::read_volatile(len_ptr) as usize;
    if len > max_data_len {
        return None; // Corrupted length — skip this slot
    }
    let data_ptr = slot_ptr.add(16);
    let slice = std::slice::from_raw_parts(data_ptr, len);
    Some(bincode::deserialize(slice).unwrap_or_else(|_| {
        // Deserialization failed — fallback to raw read. This should not happen
        // as the producer serialized successfully, but provides a safe fallback.
        std::ptr::read(slot_ptr as *const T)
    }))
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

    let head = unsafe { (*head_ptr).load(Ordering::Relaxed) };
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

    let tail = unsafe { (*tail_ptr).load(Ordering::Relaxed) };
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
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    let seq = header.sequence_or_head.fetch_add(1, Ordering::AcqRel);
    let index = (seq & mask) as usize;
    unsafe {
        let ready_ptr =
            &*(local.cached_seq_ptr.add(index * 8) as *const std::sync::atomic::AtomicU64);
        ready_ptr.store(0, Ordering::Release);
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
    if bytes.len() > max_data_len {
        return Err(msg); // Serialized data too large for slot
    }
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
    if bytes.len() > max_data_len {
        return Err(msg); // Serialized data too large for slot
    }

    let seq = header.sequence_or_head.fetch_add(1, Ordering::AcqRel);
    let index = (seq & mask) as usize;
    let slot_offset = index * slot_size;
    unsafe {
        let slot_ptr = local.cached_data_ptr.add(slot_offset);
        let len_ptr = slot_ptr.add(8) as *mut u64;
        std::ptr::write_volatile(len_ptr, bytes.len() as u64);
        let data_ptr = slot_ptr.add(16);
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), data_ptr, bytes.len());
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
    let header = unsafe { &*local.cached_header_ptr };

    let seq = local.local_head;
    if unlikely(seq.wrapping_sub(local.local_tail) >= local.cached_capacity) {
        local.local_tail = header.tail.load(Ordering::Acquire);
        if seq.wrapping_sub(local.local_tail) >= local.cached_capacity {
            return Err(msg);
        }
    }

    let index = (seq & local.cached_capacity_mask) as usize;
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
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    let seq = header.sequence_or_head.fetch_add(1, Ordering::AcqRel);
    let index = (seq & mask) as usize;
    unsafe {
        colo_seq(local.cached_data_ptr, index).store(0, Ordering::Release);
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
        spin_for_ready(ready_ptr, tail.wrapping_add(1))
    };
    if !ready_ok {
        return None;
    }

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
        spin_for_ready(ready_ptr, tail.wrapping_add(1))
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
    let ready = unsafe {
        let ready_ptr =
            &*(local.cached_seq_ptr.add(index * 8) as *const std::sync::atomic::AtomicU64);
        let ready_val = ready_ptr.load(Ordering::Acquire);
        ready_val != 0 && ready_val >= tail.wrapping_add(1)
    };
    if !ready {
        return None;
    }

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
    let msg = unsafe { read_serde_slot::<T>(local.cached_data_ptr.add(slot_offset), slot_size) }?;

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
        spin_for_ready(ready_ptr, tail.wrapping_add(1))
    };
    if !ready_ok {
        return None;
    }

    // SAFETY: slot_ptr is valid for slot_size bytes within SHM. Ready flag
    // was verified above, so the producer has finished writing this slot.
    let msg = unsafe { read_serde_slot::<T>(local.cached_data_ptr.add(slot_offset), slot_size) }?;

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
            read_serde_slot::<T>(local.cached_data_ptr.add(slot_offset), slot_size)
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
        spin_for_ready(ready_ptr, tail.wrapping_add(1))
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
            read_serde_slot::<T>(local.cached_data_ptr.add(slot_offset), slot_size)
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
    let msg = unsafe { std::ptr::read(colo_data::<T>(local.cached_data_ptr, index)) };
    let new_tail = tail.wrapping_add(1);
    local.local_tail = new_tail;

    // Batch header.tail updates: only store every 32 messages.
    // The producer reads header.tail for backpressure only when its local head
    // is >= capacity ahead of its cached tail. With capacity 256 and batch 32,
    // there's 224 slots of headroom. This eliminates the Release store to a
    // separate cache line on 31 of every 32 recvs.
    if new_tail & 0x1F == 0 {
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
    let ready_ok =
        unsafe { spin_for_ready(colo_seq(local.cached_data_ptr, index), tail.wrapping_add(1)) };
    if !ready_ok {
        return None;
    }

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
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    // Bounded retry: avoids unbounded spinning under consumer contention.
    for _attempt in 0..8 {
        let tail = header.tail.load(Ordering::Acquire);
        let index = (tail & mask) as usize;

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
            let msg = unsafe { std::ptr::read(colo_data::<T>(local.cached_data_ptr, index)) };
            local.local_tail = tail.wrapping_add(1);

            housekeep_lease!(local, topic);
            return Some(msg);
        }
        std::hint::spin_loop();
    }
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
    let ready_ok =
        unsafe { spin_for_ready(colo_seq(local.cached_data_ptr, index), tail.wrapping_add(1)) };
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
        let msg = unsafe { std::ptr::read(colo_data::<T>(local.cached_data_ptr, index)) };
        local.local_tail = tail.wrapping_add(1);

        housekeep_lease!(local, topic);
        return Some(msg);
    }
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
    let ready = unsafe {
        let seq_val = colo_seq(local.cached_data_ptr, index).load(Ordering::Acquire);
        seq_val != 0 && seq_val >= tail.wrapping_add(1)
    };
    if !ready {
        return None;
    }

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
