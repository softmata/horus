//! Function-pointer dispatch for Topic<T>.
//!
//! Each function here is a fully specialized send/recv path for one specific
//! backend configuration. By resolving the backend at initialization time
//! (in `initialize_backend()`) and storing a function pointer, the hot-path
//! `try_send()` / `try_recv()` becomes a single indirect call — zero enum
//! matches, zero mode branches.
//!
//! **All housekeeping (msg_counter, lease refresh, epoch checks) is handled
//! by try_send/try_recv OUTSIDE these functions.** Dispatch functions are
//! pure ring operations — nothing more.
//!
//! ## Co-Located Slot Layout (small POD types, sizeof(T) + 8 <= 64)
//!
//! For types where the data fits in a single 64-byte cache line alongside an
//! 8-byte sequence counter, we use a co-located layout:
//!
//! ```text
//! | seq: u64 (8B) | data: T | padding to 64B |
//! ```
//!
//! The inline `seq` serves as the notification mechanism. The consumer reads
//! `seq` and `data` from the SAME cache line — ONE inter-core transfer instead
//! of two. This cuts SpscShm latency roughly in half (~200ns → ~100ns).
//!
//! ## Send Functions (14)
//!
//! | # | Function | Backend |
//! |---|----------|---------|
//! | 1 | `send_direct_channel` | DirectChannel heap |
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
//! | 14| `send_uninitialized` | Before registration |
//!
//! ## Recv Functions (20)
//!
//! | # | Function | Backend |
//! |---|----------|---------|
//! | 1 | `recv_direct_channel` | DirectChannel heap |
//! | 2 | `recv_spsc_intra` | SpscRing heap |
//! | 3 | `recv_spmc_intra` | SpmcRing heap |
//! | 4 | `recv_mpsc_intra` | MpscRing heap |
//! | 5 | `recv_mpmc_intra` | MpmcRing heap |
//! | 6 | `recv_shm_spsc_pod` | SpscShm POD (separate seq) |
//! | 7 | `recv_shm_mpsc_pod` | MpscShm POD (separate seq) |
//! | 8 | `recv_shm_spmc_pod` | SpmcShm POD (separate seq) |
//! | 9 | `recv_shm_mpmc_pod` | MpmcShm POD (separate seq) |
//! | 10| `recv_shm_pod_broadcast` | PodShm (separate seq) |
//! | 11| `recv_shm_spsc_pod_colo` | SpscShm POD (co-located) |
//! | 12| `recv_shm_mpsc_pod_colo` | MpscShm POD (co-located) |
//! | 13| `recv_shm_spmc_pod_colo` | SpmcShm POD (co-located) |
//! | 14| `recv_shm_mpmc_pod_colo` | MpmcShm POD (co-located) |
//! | 15| `recv_shm_pod_broadcast_colo` | PodShm (co-located) |
//! | 16| `recv_shm_spsc_serde` | SpscShm non-POD |
//! | 17| `recv_shm_mpsc_serde` | MpscShm non-POD |
//! | 18| `recv_shm_spmc_serde` | SpmcShm non-POD |
//! | 19| `recv_shm_mpmc_serde` | MpmcShm non-POD |
//! | 20| `recv_uninitialized` | Before registration |

use std::mem::MaybeUninit;
use std::sync::atomic::Ordering;

use serde::{de::DeserializeOwned, Serialize};

use super::backend::BackendStorage;
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
// SEND FUNCTIONS — pure ring operations, zero housekeeping
// ============================================================================

// ---------------------------------------------------------------------------
// 1. DirectChannel (heap) — same-thread, Relaxed atomics only
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_direct_channel<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    let slot = match unsafe { &*topic.backend.get() } {
        BackendStorage::DirectChannel(s) => s,
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
    ring.try_send(msg)
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
    ring.try_send(msg)
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
    ring.try_send(msg)
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
    ring.try_send(msg)
}

// ---------------------------------------------------------------------------
// 6. SHM single-producer POD (SpscShm / SpmcShm)
// ---------------------------------------------------------------------------
//
// No ready flag write needed: SpscShm and SpmcShm consumers don't check it.
// Data visibility is guaranteed by Release store on sequence_or_head: the
// data write (simd_aware_write) happens-before the head store, and the
// consumer's Acquire load of head ensures it sees the data.

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
    }
    let new_seq = seq.wrapping_add(1);
    local.local_head = new_seq;
    header.sequence_or_head.store(new_seq, Ordering::Release);
    Ok(())
}

// ---------------------------------------------------------------------------
// 7. SHM multi-producer POD (MpscShm / MpmcShm) — CAS loop for correctness
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

    // CAS loop to atomically claim a slot — prevents ring overflow race condition.
    // (Old code did load + check + fetch_add which could overflow by N-1 producers)
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
    Ok(())
}

// ---------------------------------------------------------------------------
// 8. PodShm broadcast send — no backpressure, always succeeds
// ---------------------------------------------------------------------------

/// PodShm broadcast: producer never blocks. Overwrites oldest data if consumers
/// are slow. This is the correct semantics for robotics sensor data where
/// freshness matters more than reliability.
///
/// Two-phase ready protocol: the producer invalidates the ready flag (store 0)
/// before writing data, then sets it to seq+1 after data is complete. This
/// prevents consumers from reading partially-overwritten data when the ring wraps.
#[inline(always)]
pub(super) fn send_shm_pod_broadcast<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    // Unconditionally claim slot — no backpressure check.
    let seq = header.sequence_or_head.fetch_add(1, Ordering::AcqRel);
    let index = (seq & mask) as usize;
    unsafe {
        let ready_ptr = &*(local.cached_seq_ptr.add(index * 8)
            as *const std::sync::atomic::AtomicU64);
        // Phase 1: Invalidate ready flag BEFORE writing data.
        // Prevents consumers from reading partially-overwritten slot data
        // when the ring wraps and a new producer overwrites the slot.
        ready_ptr.store(0, Ordering::Release);

        let base = local.cached_data_ptr as *mut T;
        simd_aware_write(base.add(index), msg);

        // Phase 2: Mark data as ready with the new sequence.
        ready_ptr.store(seq.wrapping_add(1), Ordering::Release);
    }
    local.local_head = seq + 1;
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
    Ok(())
}

// ===========================================================================
// CO-LOCATED SLOT SEND FUNCTIONS (small POD types, sizeof(T) + 8 <= 64)
//
// Slot layout: [seq: u64 (8 bytes) | data: T | padding to 64 bytes]
// All fields on the SAME cache line — consumer reads seq + data in ONE transfer.
// ===========================================================================

/// Co-located slot stride: 64 bytes (cache line aligned).
const COLO_STRIDE: usize = 64;

/// Get the inline sequence atomic at the start of a co-located slot.
#[inline(always)]
unsafe fn colo_seq(data_ptr: *mut u8, index: usize) -> &'static std::sync::atomic::AtomicU64 {
    &*(data_ptr.add(index * COLO_STRIDE) as *const std::sync::atomic::AtomicU64)
}

/// Get the data pointer within a co-located slot (offset 8 from slot start).
#[inline(always)]
unsafe fn colo_data<T>(data_ptr: *mut u8, index: usize) -> *mut T {
    data_ptr.add(index * COLO_STRIDE + 8) as *mut T
}

// ---------------------------------------------------------------------------
// 11. SHM single-producer POD co-located (SpscShm / SpmcShm)
// ---------------------------------------------------------------------------
//
// Writes data + inline seq on the SAME cache line. Consumer checks inline
// seq instead of header.sequence_or_head, reducing cross-core transfers
// from 2 to 1.

#[inline(always)]
pub(super) fn send_shm_sp_pod_colo<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
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
        // Write data at slot+8 (same cache line as seq at slot+0)
        std::ptr::write(colo_data::<T>(local.cached_data_ptr, index), msg);
        // Store inline seq with Release — data is guaranteed visible to consumer
        // before this seq value becomes visible (Acquire on consumer side).
        colo_seq(local.cached_data_ptr, index).store(seq.wrapping_add(1), Ordering::Release);
    }
    let new_seq = seq.wrapping_add(1);
    local.local_head = new_seq;
    // Also update header for migration correctness (migration reads this to drain).
    // This goes to a different cache line (header line 2) but doesn't stall the
    // producer — on x86 it's just a MOV to the store buffer.
    header.sequence_or_head.store(new_seq, Ordering::Release);
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
    Ok(())
}

// ---------------------------------------------------------------------------
// 13. PodShm broadcast co-located — no backpressure
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn send_shm_pod_broadcast_colo<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    let seq = header.sequence_or_head.fetch_add(1, Ordering::AcqRel);
    let index = (seq & mask) as usize;
    unsafe {
        // Two-phase ready protocol: invalidate before data write, validate after.
        colo_seq(local.cached_data_ptr, index).store(0, Ordering::Release);
        std::ptr::write(colo_data::<T>(local.cached_data_ptr, index), msg);
        colo_seq(local.cached_data_ptr, index).store(seq.wrapping_add(1), Ordering::Release);
    }
    local.local_head = seq + 1;
    Ok(())
}

// ---------------------------------------------------------------------------
// 14. Uninitialized send — safety net
// ---------------------------------------------------------------------------

#[cold]
#[inline(never)]
pub(super) fn send_uninitialized<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    _topic: &Topic<T>,
    msg: T,
) -> Result<(), T> {
    Err(msg)
}

// ============================================================================
// RECV FUNCTIONS — pure ring operations, zero housekeeping
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
    ring.try_recv()
}

#[inline(always)]
pub(super) fn recv_spmc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::SpmcIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    ring.try_recv()
}

#[inline(always)]
pub(super) fn recv_mpsc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::MpscIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    ring.try_recv()
}

#[inline(always)]
pub(super) fn recv_mpmc_intra<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::MpmcIntra(r) => r,
        _ => unsafe { std::hint::unreachable_unchecked() },
    };
    ring.try_recv()
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
            return Some(msg);
        }
        std::hint::spin_loop();
    }
}

// ---------------------------------------------------------------------------
// 9. SHM MpmcShm POD recv — bounded spin + CAS on tail
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
        return Some(msg);
    }
    None
}

// ---------------------------------------------------------------------------
// 10. PodShm broadcast recv — local tail, no CAS, no spin-wait
// ---------------------------------------------------------------------------

/// PodShm broadcast recv: each consumer tracks its own tail position independently.
/// No CAS contention between consumers. Single ready-flag check (no spin loop)
/// means we return instantly if data isn't ready yet.
///
/// Fast-forward: if the consumer falls more than `capacity` messages behind the
/// producer (e.g. after migration or long pause), old data has been overwritten
/// and ready flags are stale. We skip to current head and retry on next poll.
///
/// This eliminates the CAS contention that caused 155ms queuing delay in 2P2C
/// cross-process scenarios.
#[inline(always)]
pub(super) fn recv_shm_pod_broadcast<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    let mut tail = local.local_tail;
    // Check if data available using local tail (no SHM tail load)
    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
            return None;
        }
    }

    // Fast-forward if fallen more than capacity behind the producer.
    // In broadcast mode, producers overwrite old slots without waiting.
    // Ready flags from overwritten slots have newer sequence numbers that
    // don't match our stale tail. Skip ahead to current head so the next
    // poll reads freshly-written data with matching ready flags.
    let behind = local.local_head.wrapping_sub(tail);
    if behind > local.cached_capacity {
        tail = local.local_head;
        local.local_tail = tail;
        // Reload head in case producer advanced during skip
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
            return None;
        }
    }

    let index = (tail & mask) as usize;
    // Single ready-flag check — no bounded spin loop.
    // In broadcast mode, if data isn't ready yet, just return None and try next poll.
    //
    // Use >= comparison (not ==) because the ring wraps in broadcast mode:
    // the slot may have been written by a newer sequence than `tail`, which is
    // fine — we read the latest data. ready_val == 0 means a producer is
    // mid-write (invalidation phase), so we must not read.
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
    // Don't update header.tail — broadcast mode, producer doesn't wait for consumers.
    // Each consumer advances independently. Slow consumers miss messages (acceptable
    // for POD sensor data where freshness > reliability).
    Some(msg)
}

// ---------------------------------------------------------------------------
// 11. SHM SpscShm serde recv — single-producer single-consumer, no ready flag
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
    Some(msg)
}

// ---------------------------------------------------------------------------
// 12. SHM MpscShm serde recv — multi-producer single-consumer, ready flag spin
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
    Some(msg)
}

// ---------------------------------------------------------------------------
// 13. SHM SpmcShm serde recv — single-producer multi-consumer, CAS on tail
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
        return Some(msg);
    }
    None
}

// ---------------------------------------------------------------------------
// 14. SHM MpmcShm serde recv — bounded spin + CAS on tail
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
        return Some(msg);
    }
    None
}

// ===========================================================================
// CO-LOCATED SLOT RECV FUNCTIONS (small POD types, sizeof(T) + 8 <= 64)
//
// Consumer reads inline seq + data from the SAME cache line. For SpscShm,
// this eliminates the separate header.sequence_or_head read — the inline
// seq IS the notification. ONE cache line transfer instead of TWO.
// ===========================================================================

// ---------------------------------------------------------------------------
// 16. SHM SpscShm POD co-located recv — header notification, colo addressing
// ---------------------------------------------------------------------------
//
// CRITICAL: For SPSC, the consumer MUST poll header.sequence_or_head (a DIFFERENT
// cache line than the data). If the consumer polled the inline seq (same cache line
// as data), the producer's data write would stall on the snoop/invalidation of the
// consumer's cached copy — adding ~50-60ns of contention overhead.
//
// Instead, we poll header.sequence_or_head (cache line 2) while reading data from
// the co-located slot offset (slot+8). This gives 2 cache line transfers
// (notification + data), same as the non-co-located layout, but with correct
// addressing for the 64-byte slot stride.

#[inline(always)]
pub(super) fn recv_shm_spsc_pod_colo<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };

    let tail = local.local_tail;
    if tail >= local.local_head {
        // Poll header.sequence_or_head — SEPARATE cache line from data slot.
        // This avoids contention: producer writes data without stalling on
        // consumer's cached copy.
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
            return None;
        }
    }

    let index = (tail & local.cached_capacity_mask) as usize;
    // Read data from co-located offset (slot+8, stride=64)
    let msg = unsafe { std::ptr::read(colo_data::<T>(local.cached_data_ptr, index)) };
    let new_tail = tail.wrapping_add(1);
    local.local_tail = new_tail;
    header.tail.store(new_tail, Ordering::Release);
    Some(msg)
}

// ---------------------------------------------------------------------------
// 17. SHM MpscShm POD co-located recv — bounded spin on inline seq
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpsc_pod_colo<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
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
    // Bounded spin on inline seq (same cache line as data)
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
    Some(msg)
}

// ---------------------------------------------------------------------------
// 18. SHM SpmcShm POD co-located recv — CAS on tail, inline seq check
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_spmc_pod_colo<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    loop {
        let tail = header.tail.load(Ordering::Acquire);
        let index = (tail & mask) as usize;

        // Check inline seq to see if data is available (same cache line as data)
        let seq_val = unsafe { colo_seq(local.cached_data_ptr, index).load(Ordering::Acquire) };
        if seq_val < tail.wrapping_add(1) {
            return None;
        }

        if header.tail.compare_exchange_weak(
            tail, tail.wrapping_add(1), Ordering::AcqRel, Ordering::Relaxed,
        ).is_ok() {
            let msg = unsafe { std::ptr::read(colo_data::<T>(local.cached_data_ptr, index)) };
            local.local_tail = tail.wrapping_add(1);
            return Some(msg);
        }
        std::hint::spin_loop();
    }
}

// ---------------------------------------------------------------------------
// 19. SHM MpmcShm POD co-located recv — bounded spin + CAS
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_mpmc_pod_colo<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
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
        return Some(msg);
    }
    None
}

// ---------------------------------------------------------------------------
// 20. PodShm broadcast co-located recv — local tail, no CAS
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn recv_shm_pod_broadcast_colo<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &Topic<T>,
) -> Option<T> {
    let local = topic.local();
    let header = unsafe { &*local.cached_header_ptr };
    let mask = local.cached_capacity_mask;

    let mut tail = local.local_tail;
    if tail >= local.local_head {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if tail >= local.local_head {
            return None;
        }
    }

    // Fast-forward if fallen behind
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
    // Single ready check on inline seq (same cache line as data)
    let ready = unsafe {
        let seq_val = colo_seq(local.cached_data_ptr, index).load(Ordering::Acquire);
        seq_val != 0 && seq_val >= tail.wrapping_add(1)
    };
    if !ready {
        return None;
    }

    let msg = unsafe { std::ptr::read(colo_data::<T>(local.cached_data_ptr, index)) };
    local.local_tail = tail.wrapping_add(1);
    Some(msg)
}

// ---------------------------------------------------------------------------
// 21. Uninitialized recv — safety net
// ---------------------------------------------------------------------------

#[cold]
#[inline(never)]
pub(super) fn recv_uninitialized<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    _topic: &Topic<T>,
) -> Option<T> {
    None
}
