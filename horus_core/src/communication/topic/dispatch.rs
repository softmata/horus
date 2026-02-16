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

use std::sync::atomic::{AtomicU64, Ordering};

use serde::{de::DeserializeOwned, Serialize};

use super::backend::BackendStorage;
use super::{simd_aware_read, simd_aware_write, READY_FLAG_SPIN_LIMIT};
use super::Topic;
use super::local_state::{LEASE_REFRESH_INTERVAL, EPOCH_CHECK_INTERVAL};
use crate::utils::unlikely;

// ============================================================================
// Type aliases for function pointers
// ============================================================================

pub(super) type SendFn<T> = fn(&Topic<T>, T) -> Result<(), T>;
pub(super) type RecvFn<T> = fn(&Topic<T>) -> Option<T>;

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
            return unsafe { (*$topic.send_fn.get())($topic, $msg) };
        }
    };
}

macro_rules! epoch_guard_recv {
    ($topic:expr) => {
        let __pe = $topic.process_epoch.load(Ordering::Relaxed);
        if unlikely(__pe != $topic.local().cached_epoch) {
            $topic.handle_epoch_change(__pe);
            return unsafe { (*$topic.recv_fn.get())($topic) };
        }
    };
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
pub(super) fn send_direct_channel_local<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    let local = topic.local();
    let head = local.local_head;
    if head.wrapping_sub(local.local_tail) >= local.cached_capacity {
        return Err(msg);
    }
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
pub(super) fn recv_direct_channel_local<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let local = topic.local();
    let tail = local.local_tail;
    if tail >= local.local_head {
        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
            topic.check_migration_periodic();
        }
        return None;
    }
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
pub(super) fn send_direct_channel_cached<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    let local = topic.local();
    // cached_header_ptr repurposed as *const AtomicU64 → DirectSlot.head
    let head_ptr = local.cached_header_ptr as *const AtomicU64;
    // cached_seq_ptr repurposed as *const AtomicU64 → DirectSlot.tail
    let tail_ptr = local.cached_seq_ptr as *const AtomicU64;

    let head = unsafe { (*head_ptr).load(Ordering::Relaxed) };
    let tail = unsafe { (*tail_ptr).load(Ordering::Relaxed) };
    if head.wrapping_sub(tail) >= local.cached_capacity {
        return Err(msg);
    }
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
pub(super) fn recv_direct_channel_cached<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let local = topic.local();
    // cached_header_ptr repurposed as *const AtomicU64 → DirectSlot.head
    let head_ptr = local.cached_header_ptr as *const AtomicU64;
    // cached_seq_ptr repurposed as *const AtomicU64 → DirectSlot.tail
    let tail_ptr = local.cached_seq_ptr as *const AtomicU64;

    let tail = unsafe { (*tail_ptr).load(Ordering::Relaxed) };
    let head = unsafe { (*head_ptr).load(Ordering::Relaxed) };
    if tail >= head {
        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
            topic.check_migration_periodic();
        }
        return None;
    }
    let msg = unsafe {
        let base = local.cached_data_ptr as *const T;
        std::ptr::read(base.add((tail & local.cached_capacity_mask) as usize))
    };
    unsafe { (*(tail_ptr as *const AtomicU64)).store(tail.wrapping_add(1), Ordering::Relaxed) };

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
        topic.check_migration_periodic();
    }
    Some(msg)
}

// ---------------------------------------------------------------------------
// 2-5. Intra-process heap ring send functions
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_spsc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    epoch_guard_send!(topic, msg);

    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::SpscIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let result = ring.try_send(msg);

    if result.is_ok() {
        let local = topic.local();
        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
            topic.periodic_maintenance();
        }
    }
    result
}

#[inline(always)]
pub(super) fn send_spmc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    epoch_guard_send!(topic, msg);

    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::SpmcIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let result = ring.try_send(msg);

    if result.is_ok() {
        let local = topic.local();
        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
            topic.periodic_maintenance();
        }
    }
    result
}

#[inline(always)]
pub(super) fn send_mpsc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    epoch_guard_send!(topic, msg);

    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::MpscIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let result = ring.try_send(msg);

    if result.is_ok() {
        let local = topic.local();
        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
            topic.periodic_maintenance();
        }
    }
    result
}

#[inline(always)]
pub(super) fn send_mpmc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    epoch_guard_send!(topic, msg);

    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::MpmcIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let result = ring.try_send(msg);

    if result.is_ok() {
        let local = topic.local();
        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
            topic.periodic_maintenance();
        }
    }
    result
}

// ---------------------------------------------------------------------------
// 6. SHM single-producer POD (SpscShm / SpmcShm)
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_sp_pod<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
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
        let base = local.cached_data_ptr as *mut T;
        simd_aware_write(base.add(index), msg);
    }
    let new_seq = seq.wrapping_add(1);
    local.local_head = new_seq;
    header.sequence_or_head.store(new_seq, Ordering::Release);

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// 7. SHM multi-producer POD (MpscShm / MpmcShm)
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_mp_pod<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    epoch_guard_send!(topic, msg);

    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;
    let capacity = local.cached_capacity;

    let seq = loop {
        let head = header.sequence_or_head.load(Ordering::Acquire);
        if head.wrapping_sub(local.local_tail) >= capacity {
            local.local_tail = header.tail.load(Ordering::Acquire);
            if head.wrapping_sub(local.local_tail) >= capacity {
                return Err(msg);
            }
        }
        if header.sequence_or_head.compare_exchange_weak(
            head, head.wrapping_add(1), Ordering::AcqRel, Ordering::Relaxed,
        ).is_ok() {
            break head;
        }
        std::hint::spin_loop();
    };

    let index = (seq & mask) as usize;
    unsafe {
        let base = local.cached_data_ptr as *mut T;
        simd_aware_write(base.add(index), msg);
        let ready_ptr = &*(local.cached_seq_ptr.add(index * 8)
            as *const std::sync::atomic::AtomicU64);
        ready_ptr.store(seq.wrapping_add(1), Ordering::Release);
    }
    local.local_head = seq + 1;

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// 8. PodShm broadcast send — no backpressure
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_pod_broadcast<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    epoch_guard_send!(topic, msg);

    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    let seq = header.sequence_or_head.fetch_add(1, Ordering::AcqRel);
    let index = (seq & mask) as usize;
    unsafe {
        let ready_ptr = &*(local.cached_seq_ptr.add(index * 8)
            as *const std::sync::atomic::AtomicU64);
        ready_ptr.store(0, Ordering::Release);
        let base = local.cached_data_ptr as *mut T;
        simd_aware_write(base.add(index), msg);
        ready_ptr.store(seq.wrapping_add(1), Ordering::Release);
    }
    local.local_head = seq + 1;

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// 9. SHM single-producer serde (SpscShm / SpmcShm, non-POD)
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_sp_serde<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
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

    let new_seq = seq.wrapping_add(1);
    local.local_head = new_seq;
    header.sequence_or_head.store(new_seq, Ordering::Release);

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// 10. SHM multi-producer serde (MpscShm / MpmcShm, non-POD)
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_mp_serde<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
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

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Ok(())
}

// ===========================================================================
// CO-LOCATED SLOT SEND FUNCTIONS (small POD types, sizeof(T) + 8 <= 64)
// ===========================================================================

const COLO_STRIDE: usize = 64;

#[inline(always)]
unsafe fn colo_seq(data_ptr: *mut u8, index: usize) -> &'static std::sync::atomic::AtomicU64 {
    &*(data_ptr.add(index * COLO_STRIDE) as *const std::sync::atomic::AtomicU64)
}

#[inline(always)]
unsafe fn colo_data<T>(data_ptr: *mut u8, index: usize) -> *mut T {
    data_ptr.add(index * COLO_STRIDE + 8) as *mut T
}

// ---------------------------------------------------------------------------
// 11. SHM single-producer POD co-located (SpscShm / SpmcShm)
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_sp_pod_colo<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
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

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// 12. SHM multi-producer POD co-located (MpscShm / MpmcShm)
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_mp_pod_colo<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    epoch_guard_send!(topic, msg);

    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;
    let capacity = local.cached_capacity;

    let seq = loop {
        let head = header.sequence_or_head.load(Ordering::Acquire);
        if head.wrapping_sub(local.local_tail) >= capacity {
            local.local_tail = header.tail.load(Ordering::Acquire);
            if head.wrapping_sub(local.local_tail) >= capacity {
                return Err(msg);
            }
        }
        if header.sequence_or_head.compare_exchange_weak(
            head, head.wrapping_add(1), Ordering::AcqRel, Ordering::Relaxed,
        ).is_ok() {
            break head;
        }
        std::hint::spin_loop();
    };

    let index = (seq & mask) as usize;
    unsafe {
        std::ptr::write(colo_data::<T>(local.cached_data_ptr, index), msg);
        colo_seq(local.cached_data_ptr, index).store(seq.wrapping_add(1), Ordering::Release);
    }
    local.local_head = seq + 1;

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// 13. PodShm broadcast co-located
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_pod_broadcast_colo<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
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

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// 14. Uninitialized send — first call, register then re-dispatch
// ---------------------------------------------------------------------------

#[cold]
#[inline(never)]
pub(super) fn send_uninitialized<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    if topic.ensure_producer().is_err() {
        return Err(msg);
    }
    // fn ptrs are now set; re-dispatch through the real send function
    unsafe { (*topic.send_fn.get())(topic, msg) }
}

// ============================================================================
// RECV FUNCTIONS — complete recv path (epoch + ring op + housekeeping)
// ============================================================================

// ---------------------------------------------------------------------------
// 2-5. Intra-process heap ring recv functions
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_spsc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::SpscIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let result = ring.try_recv();

    let local = topic.local();
    local.msg_counter = local.msg_counter.wrapping_add(1);
    if result.is_some() {
        if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
            topic.periodic_maintenance();
        }
    } else if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
        topic.check_migration_periodic();
    }
    result
}

#[inline(always)]
pub(super) fn recv_spmc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::SpmcIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let result = ring.try_recv();

    let local = topic.local();
    local.msg_counter = local.msg_counter.wrapping_add(1);
    if result.is_some() {
        if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
            topic.periodic_maintenance();
        }
    } else if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
        topic.check_migration_periodic();
    }
    result
}

#[inline(always)]
pub(super) fn recv_mpsc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::MpscIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let result = ring.try_recv();

    let local = topic.local();
    local.msg_counter = local.msg_counter.wrapping_add(1);
    if result.is_some() {
        if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
            topic.periodic_maintenance();
        }
    } else if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
        topic.check_migration_periodic();
    }
    result
}

#[inline(always)]
pub(super) fn recv_mpmc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::MpmcIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let result = ring.try_recv();

    let local = topic.local();
    local.msg_counter = local.msg_counter.wrapping_add(1);
    if result.is_some() {
        if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
            topic.periodic_maintenance();
        }
    } else if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
        topic.check_migration_periodic();
    }
    result
}

// ---------------------------------------------------------------------------
// 6. SHM SpscShm POD recv
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_spsc_pod<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };

    let tail = local.local_tail;
    let mask = local.cached_capacity_mask;
    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
            local.msg_counter = local.msg_counter.wrapping_add(1);
            if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                topic.check_migration_periodic();
            }
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

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Some(msg)
}

// ---------------------------------------------------------------------------
// 7. SHM MpscShm POD recv
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpsc_pod<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };

    let tail = local.local_tail;
    let mask = local.cached_capacity_mask;
    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
            local.msg_counter = local.msg_counter.wrapping_add(1);
            if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                topic.check_migration_periodic();
            }
            return None;
        }
    }

    let index = (tail & mask) as usize;
    let ready_ok = unsafe {
        let ready_ptr = &*(local.cached_seq_ptr.add(index * 8)
            as *const std::sync::atomic::AtomicU64);
        let expected = tail.wrapping_add(1);
        let mut spins = 0u32;
        loop {
            if ready_ptr.load(Ordering::Acquire) == expected {
                break true;
            }
            spins += 1;
            if spins >= READY_FLAG_SPIN_LIMIT {
                break false;
            }
            std::hint::spin_loop();
        }
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

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Some(msg)
}

// ---------------------------------------------------------------------------
// 8. SHM SpmcShm POD recv — CAS on tail
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_spmc_pod<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    loop {
        let tail = header.tail.load(Ordering::Acquire);
        if tail >= local.local_head {
            local.local_head = header.sequence_or_head.load(Ordering::Acquire);
            if tail >= local.local_head {
                local.msg_counter = local.msg_counter.wrapping_add(1);
                if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                    topic.check_migration_periodic();
                }
                return None;
            }
        }
        if header.tail.compare_exchange_weak(
            tail, tail.wrapping_add(1), Ordering::AcqRel, Ordering::Relaxed,
        ).is_ok() {
            let msg = unsafe {
                let base = local.cached_data_ptr as *const T;
                simd_aware_read(base.add((tail & mask) as usize))
            };
            local.local_tail = tail.wrapping_add(1);

            local.msg_counter = local.msg_counter.wrapping_add(1);
            if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
                topic.periodic_maintenance();
            }
            return Some(msg);
        }
        std::hint::spin_loop();
    }
}

// ---------------------------------------------------------------------------
// 9. SHM MpmcShm POD recv — bounded spin + CAS
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpmc_pod<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    let tail = header.tail.load(Ordering::Acquire);
    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
            local.msg_counter = local.msg_counter.wrapping_add(1);
            if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                topic.check_migration_periodic();
            }
            return None;
        }
    }

    let index = (tail & mask) as usize;
    let ready_ok = unsafe {
        let ready_ptr = &*(local.cached_seq_ptr.add(index * 8)
            as *const std::sync::atomic::AtomicU64);
        let expected = tail.wrapping_add(1);
        let mut spins = 0u32;
        loop {
            if ready_ptr.load(Ordering::Acquire) == expected {
                break true;
            }
            spins += 1;
            if spins >= READY_FLAG_SPIN_LIMIT {
                break false;
            }
            std::hint::spin_loop();
        }
    };
    if !ready_ok {
        return None;
    }

    if header.tail.compare_exchange_weak(
        tail, tail.wrapping_add(1), Ordering::AcqRel, Ordering::Relaxed,
    ).is_ok() {
        let msg = unsafe {
            let base = local.cached_data_ptr as *const T;
            simd_aware_read(base.add(index))
        };
        local.local_tail = tail.wrapping_add(1);

        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
            topic.periodic_maintenance();
        }
        return Some(msg);
    }
    None
}

// ---------------------------------------------------------------------------
// 10. PodShm broadcast recv
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_pod_broadcast<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    let mut tail = local.local_tail;
    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
            local.msg_counter = local.msg_counter.wrapping_add(1);
            if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                topic.check_migration_periodic();
            }
            return None;
        }
    }

    let behind = local.local_head.wrapping_sub(tail);
    if behind > local.cached_capacity {
        tail = local.local_head;
        local.local_tail = tail;
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
            return None;
        }
    }

    let index = (tail & mask) as usize;
    let ready = unsafe {
        let ready_ptr = &*(local.cached_seq_ptr.add(index * 8)
            as *const std::sync::atomic::AtomicU64);
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

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Some(msg)
}

// ---------------------------------------------------------------------------
// 11. SHM SpscShm serde recv
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_spsc_serde<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };

    let tail = local.local_tail;
    let mask = local.cached_capacity_mask;
    let slot_size = local.slot_size;

    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
            local.msg_counter = local.msg_counter.wrapping_add(1);
            if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                topic.check_migration_periodic();
            }
            return None;
        }
    }

    let index = (tail & mask) as usize;
    let slot_offset = index * slot_size;

    let max_data_len = slot_size.saturating_sub(16);
    let msg = unsafe {
        let slot_ptr = local.cached_data_ptr.add(slot_offset);
        let len_ptr = slot_ptr.add(8) as *const u64;
        let len = std::ptr::read_volatile(len_ptr) as usize;
        if len > max_data_len {
            return None; // Corrupted length — skip this slot
        }
        let data_ptr = slot_ptr.add(16);
        let slice = std::slice::from_raw_parts(data_ptr, len);
        bincode::deserialize(slice).unwrap_or_else(|_| {
            std::ptr::read(slot_ptr as *const T)
        })
    };

    let new_tail = tail.wrapping_add(1);
    local.local_tail = new_tail;
    header.tail.store(new_tail, Ordering::Release);

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Some(msg)
}

// ---------------------------------------------------------------------------
// 12. SHM MpscShm serde recv
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpsc_serde<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };

    let tail = local.local_tail;
    let mask = local.cached_capacity_mask;
    let slot_size = local.slot_size;

    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
            local.msg_counter = local.msg_counter.wrapping_add(1);
            if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                topic.check_migration_periodic();
            }
            return None;
        }
    }

    let index = (tail & mask) as usize;
    let slot_offset = index * slot_size;

    let ready_ok = unsafe {
        let slot_ptr = local.cached_data_ptr.add(slot_offset);
        let ready_ptr = &*(slot_ptr as *const std::sync::atomic::AtomicU64);
        let expected = tail.wrapping_add(1);
        let mut spins = 0u32;
        loop {
            if ready_ptr.load(Ordering::Acquire) == expected {
                break true;
            }
            spins += 1;
            if spins >= READY_FLAG_SPIN_LIMIT {
                break false;
            }
            std::hint::spin_loop();
        }
    };
    if !ready_ok {
        return None;
    }

    let max_data_len = slot_size.saturating_sub(16);
    let msg = unsafe {
        let slot_ptr = local.cached_data_ptr.add(slot_offset);
        let len_ptr = slot_ptr.add(8) as *const u64;
        let len = std::ptr::read_volatile(len_ptr) as usize;
        if len > max_data_len {
            return None; // Corrupted length — skip this slot
        }
        let data_ptr = slot_ptr.add(16);
        let slice = std::slice::from_raw_parts(data_ptr, len);
        bincode::deserialize(slice).unwrap_or_else(|_| {
            std::ptr::read(slot_ptr as *const T)
        })
    };

    let new_tail = tail.wrapping_add(1);
    local.local_tail = new_tail;
    header.tail.store(new_tail, Ordering::Release);

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Some(msg)
}

// ---------------------------------------------------------------------------
// 13. SHM SpmcShm serde recv — CAS on tail
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_spmc_serde<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;
    let slot_size = local.slot_size;

    let tail = header.tail.load(Ordering::Acquire);
    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
            local.msg_counter = local.msg_counter.wrapping_add(1);
            if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                topic.check_migration_periodic();
            }
            return None;
        }
    }

    if header.tail.compare_exchange_weak(
        tail, tail.wrapping_add(1), Ordering::AcqRel, Ordering::Relaxed,
    ).is_ok() {
        let index = (tail & mask) as usize;
        let slot_offset = index * slot_size;
        let max_data_len = slot_size.saturating_sub(16);

        let msg = unsafe {
            let slot_ptr = local.cached_data_ptr.add(slot_offset);
            let len_ptr = slot_ptr.add(8) as *const u64;
            let len = std::ptr::read_volatile(len_ptr) as usize;
            if len > max_data_len {
                local.local_tail = tail.wrapping_add(1);
                return None; // Corrupted length — skip this slot
            }
            let data_ptr = slot_ptr.add(16);
            let slice = std::slice::from_raw_parts(data_ptr, len);
            bincode::deserialize(slice).unwrap_or_else(|_| {
                std::ptr::read(slot_ptr as *const T)
            })
        };

        local.local_tail = tail.wrapping_add(1);

        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
            topic.periodic_maintenance();
        }
        return Some(msg);
    }
    None
}

// ---------------------------------------------------------------------------
// 14. SHM MpmcShm serde recv — bounded spin + CAS
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpmc_serde<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;
    let slot_size = local.slot_size;

    let tail = header.tail.load(Ordering::Acquire);
    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
            local.msg_counter = local.msg_counter.wrapping_add(1);
            if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                topic.check_migration_periodic();
            }
            return None;
        }
    }

    let index = (tail & mask) as usize;
    let slot_offset = index * slot_size;

    let ready_ok = unsafe {
        let slot_ptr = local.cached_data_ptr.add(slot_offset);
        let ready_ptr = &*(slot_ptr as *const std::sync::atomic::AtomicU64);
        let expected = tail.wrapping_add(1);
        let mut spins = 0u32;
        loop {
            if ready_ptr.load(Ordering::Acquire) == expected {
                break true;
            }
            spins += 1;
            if spins >= READY_FLAG_SPIN_LIMIT {
                break false;
            }
            std::hint::spin_loop();
        }
    };
    if !ready_ok {
        return None;
    }

    if header.tail.compare_exchange_weak(
        tail, tail.wrapping_add(1), Ordering::AcqRel, Ordering::Relaxed,
    ).is_ok() {
        let max_data_len = slot_size.saturating_sub(16);
        let msg = unsafe {
            let slot_ptr = local.cached_data_ptr.add(slot_offset);
            let len_ptr = slot_ptr.add(8) as *const u64;
            let len = std::ptr::read_volatile(len_ptr) as usize;
            if len > max_data_len {
                local.local_tail = tail.wrapping_add(1);
                return None; // Corrupted length — skip this slot
            }
            let data_ptr = slot_ptr.add(16);
            let slice = std::slice::from_raw_parts(data_ptr, len);
            bincode::deserialize(slice).unwrap_or_else(|_| {
                std::ptr::read(slot_ptr as *const T)
            })
        };

        local.local_tail = tail.wrapping_add(1);

        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
            topic.periodic_maintenance();
        }
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
pub(super) fn recv_shm_spsc_pod_colo<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    let tail = local.local_tail;
    let index = (tail & local.cached_capacity_mask) as usize;

    if tail >= local.local_head {
        // Caught up with cached head. Poll per-slot seq for new data.
        // colo_seq and colo_data share the SAME cache line, so detecting
        // readiness AND reading data costs ONE cache miss instead of polling
        // header.sequence_or_head (separate cache line) + reading data (another miss).
        let seq_val = unsafe { colo_seq(local.cached_data_ptr, index).load(Ordering::Acquire) };
        let expected = tail.wrapping_add(1);
        if seq_val < expected {
            local.msg_counter = local.msg_counter.wrapping_add(1);
            if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                topic.check_migration_periodic();
            }
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

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Some(msg)
}

// ---------------------------------------------------------------------------
// 16. SHM MpscShm POD co-located recv — bounded spin on inline seq
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpsc_pod_colo<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };

    let tail = local.local_tail;
    let mask = local.cached_capacity_mask;
    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
            local.msg_counter = local.msg_counter.wrapping_add(1);
            if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                topic.check_migration_periodic();
            }
            return None;
        }
    }

    let index = (tail & mask) as usize;
    let ready_ok = unsafe {
        let seq_atom = colo_seq(local.cached_data_ptr, index);
        let expected = tail.wrapping_add(1);
        let mut spins = 0u32;
        loop {
            if seq_atom.load(Ordering::Acquire) == expected {
                break true;
            }
            spins += 1;
            if spins >= READY_FLAG_SPIN_LIMIT {
                break false;
            }
            std::hint::spin_loop();
        }
    };
    if !ready_ok {
        return None;
    }

    let msg = unsafe { std::ptr::read(colo_data::<T>(local.cached_data_ptr, index)) };
    let new_tail = tail.wrapping_add(1);
    local.local_tail = new_tail;
    header.tail.store(new_tail, Ordering::Release);

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Some(msg)
}

// ---------------------------------------------------------------------------
// 17. SHM SpmcShm POD co-located recv — CAS on tail, inline seq check
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_spmc_pod_colo<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    loop {
        let tail = header.tail.load(Ordering::Acquire);
        let index = (tail & mask) as usize;

        let seq_val = unsafe { colo_seq(local.cached_data_ptr, index).load(Ordering::Acquire) };
        if seq_val < tail.wrapping_add(1) {
            local.msg_counter = local.msg_counter.wrapping_add(1);
            if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                topic.check_migration_periodic();
            }
            return None;
        }

        if header.tail.compare_exchange_weak(
            tail, tail.wrapping_add(1), Ordering::AcqRel, Ordering::Relaxed,
        ).is_ok() {
            let msg = unsafe { std::ptr::read(colo_data::<T>(local.cached_data_ptr, index)) };
            local.local_tail = tail.wrapping_add(1);

            local.msg_counter = local.msg_counter.wrapping_add(1);
            if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
                topic.periodic_maintenance();
            }
            return Some(msg);
        }
        std::hint::spin_loop();
    }
}

// ---------------------------------------------------------------------------
// 18. SHM MpmcShm POD co-located recv — bounded spin + CAS
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpmc_pod_colo<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    let tail = header.tail.load(Ordering::Acquire);
    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
            local.msg_counter = local.msg_counter.wrapping_add(1);
            if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                topic.check_migration_periodic();
            }
            return None;
        }
    }

    let index = (tail & mask) as usize;
    let ready_ok = unsafe {
        let seq_atom = colo_seq(local.cached_data_ptr, index);
        let expected = tail.wrapping_add(1);
        let mut spins = 0u32;
        loop {
            if seq_atom.load(Ordering::Acquire) == expected {
                break true;
            }
            spins += 1;
            if spins >= READY_FLAG_SPIN_LIMIT {
                break false;
            }
            std::hint::spin_loop();
        }
    };
    if !ready_ok {
        return None;
    }

    if header.tail.compare_exchange_weak(
        tail, tail.wrapping_add(1), Ordering::AcqRel, Ordering::Relaxed,
    ).is_ok() {
        let msg = unsafe { std::ptr::read(colo_data::<T>(local.cached_data_ptr, index)) };
        local.local_tail = tail.wrapping_add(1);

        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
            topic.periodic_maintenance();
        }
        return Some(msg);
    }
    None
}

// ---------------------------------------------------------------------------
// 19. PodShm broadcast co-located recv
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_pod_broadcast_colo<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    let mut tail = local.local_tail;
    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
            local.msg_counter = local.msg_counter.wrapping_add(1);
            if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                topic.check_migration_periodic();
            }
            return None;
        }
    }

    let behind = local.local_head.wrapping_sub(tail);
    if behind > local.cached_capacity {
        tail = local.local_head;
        local.local_tail = tail;
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
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

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Some(msg)
}

// ---------------------------------------------------------------------------
// 20. Uninitialized recv — first call, register then re-dispatch
// ---------------------------------------------------------------------------

#[cold]
#[inline(never)]
pub(super) fn recv_uninitialized<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    if topic.ensure_consumer().is_err() {
        return None;
    }
    // fn ptrs are now set; re-dispatch through the real recv function
    unsafe { (*topic.recv_fn.get())(topic) }
}
