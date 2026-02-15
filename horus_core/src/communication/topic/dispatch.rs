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

use std::mem::MaybeUninit;
use std::sync::atomic::Ordering;

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
// 1. DirectChannel (heap) — same-thread, Relaxed atomics (plain MOV on x86)
// ---------------------------------------------------------------------------
//
// Uses DirectSlot's AtomicU64 head/tail so separate Topic instances sharing
// the same DirectSlot see each other's updates. Relaxed ordering = plain MOV
// on x86, so there's zero overhead beyond L1 cache hits.

#[inline(always)]
pub(super) fn send_direct_channel<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    epoch_guard_send!(topic, msg);

    let slot = match unsafe { &*topic.backend.get() } {
        BackendStorage::DirectChannel(s) => s,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let head = slot.head.load(Ordering::Relaxed);
    let tail = slot.tail.load(Ordering::Relaxed);
    if head.wrapping_sub(tail) >= slot.capacity {
        return Err(msg);
    }
    let index = (head & slot.mask) as usize;
    unsafe {
        let s = &*slot.buffer.get_unchecked(index);
        s.get().write(MaybeUninit::new(msg));
    }
    slot.head.store(head.wrapping_add(1), Ordering::Relaxed);

    let local = topic.local();
    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Ok(())
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
    header.sequence_or_head.store(new_seq, Ordering::Release);

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
// 1. DirectChannel recv (heap) — same-thread, Relaxed atomics
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_direct_channel<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let slot = match unsafe { &*topic.backend.get() } {
        BackendStorage::DirectChannel(s) => s,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let tail = slot.tail.load(Ordering::Relaxed);
    let head = slot.head.load(Ordering::Relaxed);
    if tail >= head {
        let local = topic.local();
        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
            topic.check_migration_periodic();
        }
        return None;
    }
    let msg = unsafe {
        let idx = (tail & slot.mask) as usize;
        let s = &*slot.buffer.get_unchecked(idx);
        (*s.get()).assume_init_read()
    };
    slot.tail.store(tail.wrapping_add(1), Ordering::Relaxed);

    let local = topic.local();
    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
        topic.periodic_maintenance();
    }
    Some(msg)
}

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
    header.tail.store(new_tail, Ordering::Release);

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
    header.tail.store(new_tail, Ordering::Release);

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

    let msg = unsafe {
        let slot_ptr = local.cached_data_ptr.add(slot_offset);
        let len_ptr = slot_ptr.add(8) as *const u64;
        let len = std::ptr::read_volatile(len_ptr) as usize;
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

    let msg = unsafe {
        let slot_ptr = local.cached_data_ptr.add(slot_offset);
        let len_ptr = slot_ptr.add(8) as *const u64;
        let len = std::ptr::read_volatile(len_ptr) as usize;
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

        let msg = unsafe {
            let slot_ptr = local.cached_data_ptr.add(slot_offset);
            let len_ptr = slot_ptr.add(8) as *const u64;
            let len = std::ptr::read_volatile(len_ptr) as usize;
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
        let msg = unsafe {
            let slot_ptr = local.cached_data_ptr.add(slot_offset);
            let len_ptr = slot_ptr.add(8) as *const u64;
            let len = std::ptr::read_volatile(len_ptr) as usize;
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
// 15. SHM SpscShm POD co-located recv — header notification, colo addressing
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_spsc_pod_colo<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };

    let tail = local.local_tail;
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

    let index = (tail & local.cached_capacity_mask) as usize;
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
