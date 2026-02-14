//! Function-pointer dispatch for Topic<T>.
//!
//! Each function here is a fully specialized send/recv path for one specific
//! backend configuration. By resolving the backend at initialization time
//! (in `initialize_backend()`) and storing a function pointer, the hot-path
//! `try_send()` / `try_recv()` becomes a single indirect call — zero enum
//! matches, zero mode branches.
//!
//! Epoch checking is handled by `try_send()`/`try_recv()` in mod.rs before
//! calling these functions. Lease refresh is done here periodically (every
//! LEASE_REFRESH_INTERVAL messages) for participant table maintenance.
//!
//! ## Send Functions (10)
//!
//! | # | Function | Backend |
//! |---|----------|---------|
//! | 1 | `send_direct_channel` | DirectChannel heap |
//! | 2 | `send_spsc_intra` | SpscRing heap |
//! | 3 | `send_spmc_intra` | SpmcRing heap |
//! | 4 | `send_mpsc_intra` | MpscRing heap |
//! | 5 | `send_mpmc_intra` | MpmcRing heap |
//! | 6 | `send_shm_sp_pod` | SpscShm/SpmcShm POD |
//! | 7 | `send_shm_mp_pod` | MpscShm/MpmcShm/PodShm POD |
//! | 8 | `send_shm_sp_serde` | SpscShm/SpmcShm non-POD |
//! | 9 | `send_shm_mp_serde` | MpscShm/MpmcShm non-POD |
//! | 10| `send_uninitialized` | Before registration |
//!
//! ## Recv Functions (14)
//!
//! | # | Function | Backend |
//! |---|----------|---------|
//! | 1 | `recv_direct_channel` | DirectChannel heap |
//! | 2 | `recv_spsc_intra` | SpscRing heap |
//! | 3 | `recv_spmc_intra` | SpmcRing heap |
//! | 4 | `recv_mpsc_intra` | MpscRing heap |
//! | 5 | `recv_mpmc_intra` | MpmcRing heap |
//! | 6 | `recv_shm_spsc_pod` | SpscShm POD |
//! | 7 | `recv_shm_mpsc_pod` | MpscShm POD |
//! | 8 | `recv_shm_spmc_pod` | SpmcShm POD |
//! | 9 | `recv_shm_mpmc_pod` | MpmcShm/PodShm POD |
//! | 10| `recv_shm_spsc_serde` | SpscShm non-POD |
//! | 11| `recv_shm_mpsc_serde` | MpscShm non-POD |
//! | 12| `recv_shm_spmc_serde` | SpmcShm non-POD |
//! | 13| `recv_shm_mpmc_serde` | MpmcShm non-POD |
//! | 14| `recv_uninitialized` | Before registration |

use std::mem::MaybeUninit;
use std::sync::atomic::Ordering;

use serde::{de::DeserializeOwned, Serialize};

use super::backend::BackendStorage;
use super::local_state::LEASE_REFRESH_INTERVAL;
use super::{simd_aware_read, simd_aware_write, READY_FLAG_SPIN_LIMIT};
use super::Topic;
use crate::utils::unlikely;

// ============================================================================
// Type aliases for function pointers
// ============================================================================

/// Function pointer type for try_send dispatch.
pub(super) type SendFn<T> = fn(&Topic<T>, T) -> Result<(), T>;

/// Function pointer type for try_recv dispatch.
pub(super) type RecvFn<T> = fn(&Topic<T>) -> Option<T>;

// ============================================================================
// SEND FUNCTIONS
// ============================================================================

// ---------------------------------------------------------------------------
// 1. DirectChannel (heap) — same-thread, Relaxed atomics only
// ---------------------------------------------------------------------------

/// DirectChannel send: pure heap write with Relaxed ordering.
/// No lease refresh needed (same-thread, no cross-participant coordination).
#[inline(always)]
pub(super) fn send_direct_channel<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    let slot = match unsafe { &*topic.backend.get() } {
        BackendStorage::DirectChannel(s) => s,
        // SAFETY: initialize_backend guarantees the backend matches the fn ptr
        _ => unsafe { std::hint::unreachable_unchecked() },
    };

    let h = slot.head.load(Ordering::Relaxed);
    let t = slot.tail.load(Ordering::Relaxed);
    if h.wrapping_sub(t) >= slot.capacity {
        return Err(msg);
    }
    unsafe {
        let idx = (h & slot.mask) as usize;
        let s = &*slot.buffer.get_unchecked(idx);
        s.get().write(MaybeUninit::new(msg));
    }
    slot.head.store(h.wrapping_add(1), Ordering::Relaxed);
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
    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::SpscIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let result = ring.try_send(msg);
    if result.is_ok() {
        let local = topic.local();
        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
            topic.refresh_lease();
        }
    }
    result
}

#[inline(always)]
pub(super) fn send_spmc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::SpmcIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let result = ring.try_send(msg);
    if result.is_ok() {
        let local = topic.local();
        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
            topic.refresh_lease();
        }
    }
    result
}

#[inline(always)]
pub(super) fn send_mpsc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::MpscIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let result = ring.try_send(msg);
    if result.is_ok() {
        let local = topic.local();
        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
            topic.refresh_lease();
        }
    }
    result
}

#[inline(always)]
pub(super) fn send_mpmc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::MpmcIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let result = ring.try_send(msg);
    if result.is_ok() {
        let local = topic.local();
        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
            topic.refresh_lease();
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
        let ready_ptr = &*(local.cached_seq_ptr.add(index * 8)
            as *const std::sync::atomic::AtomicU64);
        ready_ptr.store(seq.wrapping_add(1), Ordering::Release);
    }
    let new_seq = seq.wrapping_add(1);
    local.local_head = new_seq;
    header.sequence_or_head.store(new_seq, Ordering::Release);

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
        topic.refresh_lease();
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// 7. SHM multi-producer POD (MpscShm / MpmcShm / PodShm)
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_mp_pod<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
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
    unsafe {
        let base = local.cached_data_ptr as *mut T;
        simd_aware_write(base.add(index), msg);
        let ready_ptr = &*(local.cached_seq_ptr.add(index * 8)
            as *const std::sync::atomic::AtomicU64);
        ready_ptr.store(seq.wrapping_add(1), Ordering::Release);
    }
    local.local_head = seq + 1;

    local.msg_counter = local.msg_counter.wrapping_add(1);
    if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
        topic.refresh_lease();
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// 8. SHM single-producer serde (SpscShm / SpmcShm, non-POD)
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_sp_serde<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
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
    if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
        topic.refresh_lease();
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// 9. SHM multi-producer serde (MpscShm / MpmcShm, non-POD)
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_mp_serde<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
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
    if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
        topic.refresh_lease();
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// 10. Uninitialized send — safety net (try_send handles registration)
// ---------------------------------------------------------------------------

#[cold]
#[inline(never)]
pub(super) fn send_uninitialized<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    _topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    // try_send() handles role check and registration before calling fn ptrs.
    // This function should only be reachable if there's a logic error.
    Err(msg)
}

// ============================================================================
// RECV FUNCTIONS
// ============================================================================

// ---------------------------------------------------------------------------
// 1. DirectChannel recv (heap) — same-thread, Relaxed atomics only
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_direct_channel<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let slot = match unsafe { &*topic.backend.get() } {
        BackendStorage::DirectChannel(s) => s,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    slot.try_recv()
}

// ---------------------------------------------------------------------------
// 2-5. Intra-process heap ring recv functions
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_spsc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::SpscIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let result = ring.try_recv();
    if result.is_some() {
        let local = topic.local();
        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
            topic.refresh_lease();
        }
    }
    result
}

#[inline(always)]
pub(super) fn recv_spmc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::SpmcIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let result = ring.try_recv();
    if result.is_some() {
        let local = topic.local();
        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
            topic.refresh_lease();
        }
    }
    result
}

#[inline(always)]
pub(super) fn recv_mpsc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::MpscIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let result = ring.try_recv();
    if result.is_some() {
        let local = topic.local();
        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
            topic.refresh_lease();
        }
    }
    result
}

#[inline(always)]
pub(super) fn recv_mpmc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::MpmcIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    let result = ring.try_recv();
    if result.is_some() {
        let local = topic.local();
        local.msg_counter = local.msg_counter.wrapping_add(1);
        if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
            topic.refresh_lease();
        }
    }
    result
}

// ---------------------------------------------------------------------------
// 6. SHM SpscShm POD recv — single-producer single-consumer, no ready flag
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_spsc_pod<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };

    let tail = local.local_tail;
    let mask = local.cached_capacity_mask;
    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
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
    if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
        topic.refresh_lease();
    }
    Some(msg)
}

// ---------------------------------------------------------------------------
// 7. SHM MpscShm POD recv — multi-producer single-consumer, bounded spin
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpsc_pod<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };

    let tail = local.local_tail;
    let mask = local.cached_capacity_mask;
    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
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
    if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
        topic.refresh_lease();
    }
    Some(msg)
}

// ---------------------------------------------------------------------------
// 8. SHM SpmcShm POD recv — single-producer multi-consumer, CAS on tail
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_spmc_pod<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    loop {
        let tail = header.tail.load(Ordering::Acquire);
        if tail >= local.local_head {
            local.local_head = header.sequence_or_head.load(Ordering::Acquire);
            if tail >= local.local_head {
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
            if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
                topic.refresh_lease();
            }
            return Some(msg);
        }
        std::hint::spin_loop();
    }
}

// ---------------------------------------------------------------------------
// 9. SHM MpmcShm/PodShm POD recv — bounded spin + CAS on tail
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpmc_pod<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    let tail = header.tail.load(Ordering::Acquire);
    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
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
        if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
            topic.refresh_lease();
        }
        return Some(msg);
    }
    None
}

// ---------------------------------------------------------------------------
// 10. SHM SpscShm serde recv — single-producer single-consumer, no ready flag
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_spsc_serde<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };

    let tail = local.local_tail;
    let mask = local.cached_capacity_mask;
    let slot_size = local.slot_size;

    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
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
    if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
        topic.refresh_lease();
    }
    Some(msg)
}

// ---------------------------------------------------------------------------
// 11. SHM MpscShm serde recv — multi-producer single-consumer, ready flag spin
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpsc_serde<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };

    let tail = local.local_tail;
    let mask = local.cached_capacity_mask;
    let slot_size = local.slot_size;

    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
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
    if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
        topic.refresh_lease();
    }
    Some(msg)
}

// ---------------------------------------------------------------------------
// 12. SHM SpmcShm serde recv — single-producer multi-consumer, CAS on tail
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_spmc_serde<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;
    let slot_size = local.slot_size;

    let tail = header.tail.load(Ordering::Acquire);
    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
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
        if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
            topic.refresh_lease();
        }
        return Some(msg);
    }
    None
}

// ---------------------------------------------------------------------------
// 13. SHM MpmcShm serde recv — bounded spin + CAS on tail
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpmc_serde<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;
    let slot_size = local.slot_size;

    let tail = header.tail.load(Ordering::Acquire);
    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
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
        if unlikely(local.msg_counter.is_multiple_of(LEASE_REFRESH_INTERVAL)) {
            topic.refresh_lease();
        }
        return Some(msg);
    }
    None
}

// ---------------------------------------------------------------------------
// 14. Uninitialized recv — safety net (try_recv handles registration)
// ---------------------------------------------------------------------------

#[cold]
#[inline(never)]
pub(super) fn recv_uninitialized<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    _topic: &Topic<T>,
) -> Option<T> {
    // try_recv() handles role check and registration before calling fn ptrs.
    // This function should only be reachable if there's a logic error.
    None
}
