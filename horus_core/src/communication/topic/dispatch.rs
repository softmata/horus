#![allow(dead_code)]
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
//! Every topic is SHM-backed, so all dispatch functions operate on the shared
//! memory data region (or, for `FanoutShm`, a separate SHM-backed SPSC matrix).
//!
//! ## Send Functions
//!
//! | Function | Backend |
//! |----------|---------|
//! | `send_fanout_shm` | FanoutShm — cross-process broadcast SPSC matrix |
//! | `send_shm_sp_pod` | SpmcShm POD (single-producer, separate seq) |
//! | `send_shm_mp_pod` | SpscShm/MpscShm POD (multi-producer claim, separate seq) |
//! | `send_shm_pod_broadcast` | PodShm (broadcast on shared ring) |
//! | `send_shm_sp_serde` | SpmcShm non-POD |
//! | `send_shm_mp_serde` | SpscShm/MpscShm non-POD |
//! | `send_uninitialized` | First call → register + re-dispatch |
//!
//! ## Recv Functions
//!
//! | Function | Backend |
//! |----------|---------|
//! | `recv_fanout_shm` | FanoutShm — cross-process broadcast SPSC matrix |
//! | `recv_shm_spsc_pod` / `recv_shm_mpsc_pod` / `recv_shm_spmc_pod` | SHM POD variants |
//! | `recv_shm_pod_broadcast` | PodShm broadcast |
//! | `recv_shm_spsc_serde` / `recv_shm_mpsc_serde` / `recv_shm_spmc_serde` | SHM non-POD variants |
//! | `recv_uninitialized` | First call → register + re-dispatch |
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
//!    (owned via `storage: ShmRegion`).
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

use std::sync::atomic::{AtomicBool, Ordering};

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
    // `to_tensor` takes &self because SpillDescriptor is not Copy (contains u64 fields
    // that are best borrowed), and the tensor is reconstructed from multiple fields.
    #[allow(clippy::wrong_self_convention)]
    #[inline]
    pub fn to_tensor(&self) -> crate::types::Tensor {
        let mut shape = [0u64; crate::types::tensor::MAX_TENSOR_DIMS];
        shape[0] = self.size;
        crate::types::Tensor {
            pool_id: self.pool_id,
            slot_id: self.slot_id,
            generation: self.generation,
            generation_hi: self.generation_hi,
            offset: self.offset,
            size: self.size,
            dtype: crate::types::TensorDtype::U8,
            ndim: 1,
            shape,
            ..Default::default()
        }
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

    // The alloc starts at refcount=1. How that refcount is reclaimed depends on
    // the backend (both COMM-H3 — the old "freed when the slot gets reallocated"
    // was a myth: nothing ever released it, so it never recycled and the pool
    // leaked until full, silently dropping large messages):
    //   - SpscShm / MpscShm (single consumer): the receiver releases after its
    //     read (`read_spilled_once`) — sender alloc (rc=1), receiver release (rc=0).
    //   - FanoutShm (multi-subscriber, N unknown at send time): the sender keeps
    //     this refcount as a keep-alive in `LocalState::spill_keepalive` and
    //     releases it once the ring position is overwritten; each receiver
    //     `try_retain`s around its read (`read_spilled_retained`). The generation
    //     counter + rc>0 pin turn a concurrent release/reuse into a clean miss,
    //     never a torn read.
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
/// LEASE_REFRESH_INTERVAL msgs).  Used by all dispatched (fn-ptr) send/recv paths.
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

/// Housekeeping on empty recv: epoch check on EVERY empty recv.
///
/// When recv returns None, the topic may be empty because a cross-process
/// producer joined and this participant hasn't observed the migration epoch
/// yet (still pointed at the old SHM ring). Checking the SHM
/// epoch on every empty recv ensures migration is detected immediately.
/// Cost: one Relaxed atomic load (~1ns) — negligible for polling loops.
macro_rules! housekeep_epoch {
    ($local:ident, $topic:expr) => {
        $local.msg_counter = $local.msg_counter.wrapping_add(1);
        $topic.check_migration_periodic();
    };
}

/// Combined housekeeping for the dispatched recv path: branch on result.
/// Epoch check on EVERY recv (not amortized) to detect cross-process joins immediately.
/// Lease refresh remains amortized (syscall cost).
macro_rules! housekeep_recv {
    ($local:ident, $topic:expr, $result:ident) => {
        $local.msg_counter = $local.msg_counter.wrapping_add(1);
        // Check SHM epoch on every recv — detects cross-process producers immediately.
        // Cost: one Relaxed atomic load (~1ns), negligible vs the ring buffer read.
        $topic.check_migration_periodic();
        if $result.is_some() {
            if unlikely($local.msg_counter & (LEASE_REFRESH_INTERVAL - 1) == 0) {
                $topic.refresh_lease();
            }
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
    // SP/MP slot layout: the SpillDescriptor sits at offset +8 (after the ready flag).
    let spill = read_spill_descriptor(slot_ptr);
    read_spilled_once(spill, topic_name)
}

/// Deserialize a message from a spill slot, generation-validated.
///
/// Returns `None` on a corrupted or stale (recycled) descriptor — `data_slice`
/// checks the generation, so a recycled slot yields `None`, never garbage. The
/// borrow of the pool slot ends before this returns (bincode copies into an owned
/// `T`), so the caller may safely `release()` the slot afterwards.
fn deserialize_spill_slot<T: DeserializeOwned>(
    pool: &crate::memory::TensorPool,
    tensor: &crate::types::Tensor,
    len: usize,
) -> Option<T> {
    let pool_bytes = pool.data_slice(tensor).ok()?;
    if len > pool_bytes.len() {
        return None; // Corrupted size or stale (recycled) pool slot.
    }
    bincode::deserialize(&pool_bytes[..len]).ok()
}

/// Read a spilled message on a **single-consumer** (SpscShm / MpscShm) topic and
/// release the producer's allocation refcount.
///
/// There is exactly one reader, so releasing after the read is safe — no other
/// receiver can be looking at the slot. `release` is generation-checked, so a
/// stale descriptor (e.g. after a topic recreate) no-ops rather than freeing an
/// unrelated slot. This is the reclaim that stops the spill-pool leak (COMM-H3)
/// for the SP/MP backends; FanoutShm uses `read_spilled_retained` instead.
fn read_spilled_once<T: DeserializeOwned>(spill: SpillDescriptor, topic_name: &str) -> Option<T> {
    let tensor = spill.to_tensor();
    let pool = super::pool_registry::get_or_create_pool(topic_name);
    let msg = deserialize_spill_slot::<T>(&pool, &tensor, spill.size as usize);
    // Single consumer owns the alloc refcount; free it now (gen-checked no-op if stale).
    pool.release(&tensor);
    msg
}

/// Read a spilled message on a **multi-subscriber** (FanoutShm) topic.
///
/// The subscriber count is unknown at send time, so the producer holds the slot
/// alive as a keep-alive (`LocalState::spill_keepalive`) and releases it when the
/// ring position is overwritten (COMM-H3). Each reader `try_retain`s the slot for
/// the duration of its read: `Ok` pins it (rc > 0, generation matches) so a
/// concurrent producer release/reuse cannot tear the `data_slice`; `Err` means the
/// message was already superseded and freed (a correct drop-oldest miss) → `None`.
fn read_spilled_retained<T: DeserializeOwned>(
    spill: SpillDescriptor,
    topic_name: &str,
) -> Option<T> {
    let tensor = spill.to_tensor();
    let pool = super::pool_registry::get_or_create_pool(topic_name);
    // Pin the slot for the read. Err => superseded + freed => clean miss.
    if pool.try_retain(&tensor).is_err() {
        return None;
    }
    let msg = deserialize_spill_slot::<T>(&pool, &tensor, spill.size as usize);
    pool.release(&tensor); // drop our read pin
    msg
}

// ============================================================================
// SEND FUNCTIONS — complete send path (epoch + ring op + housekeeping)
// ============================================================================

// ---------------------------------------------------------------------------
// 5b. FanoutShm — cross-process contention-free MPMC via SHM SPSC matrix
// ---------------------------------------------------------------------------

/// Send via ShmFanoutRing — registers publisher on first call, fans out to all subscribers.
#[inline(always)]
pub(super) fn send_fanout_shm<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &RingTopic<T>,
    msg: T,
) -> Result<(), T> {
    epoch_guard_send!(topic, msg);

    // SAFETY: backend UnsafeCell accessed from single owning thread (invariant 1).
    // FanoutShm variant guaranteed by set_dispatch_fn_ptrs (invariant 4).
    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::FanoutShm(r) => r,
        _ => debug_unreachable!("dispatch: expected FanoutShm variant in send"),
    };

    let local = topic.local();

    // Lazy publisher registration — first send registers this Topic as a publisher.
    // register_publisher returns None only when all 16 endpoint slots are
    // simultaneously live (COMM-H1) — return the message rather than the pre-fix
    // panic; a freed/crash-abandoned slot lets a later send succeed.
    let pub_id = match local.fanout_shm_pub_id {
        Some(id) => id,
        None => match ring.register_publisher_locked(topic.name()) {
            Some((id, lock)) => {
                local.fanout_shm_pub_id = Some(id);
                // Hold the endpoint flock for this Topic's lifetime (COMM-H1) — its
                // Drop (or process death) releases it, letting a peer reclaim.
                local.fanout_pub_lock = Some(lock);
                id
            }
            None => {
                // COMM-H1: never let "all 16 slots live" become a SILENT no-comms
                // endpoint (worse than the pre-fix loud panic). Warn LOUDLY, but
                // once per process so a hot send-loop can't flood the log.
                static WARNED: AtomicBool = AtomicBool::new(false);
                if !WARNED.swap(true, Ordering::Relaxed) {
                    tracing::warn!(
                        "FanoutShm topic '{}': all {} publisher endpoint slots are \
                         live — this publisher has NO comms until a slot frees \
                         (COMM-H1 capacity limit, not a panic).",
                        topic.name(),
                        super::shm_fanout::MAX_FANOUT_ENDPOINTS,
                    );
                }
                return Err(msg);
            }
        },
    };

    let ok = if ring.is_pod() {
        // SAFETY: ring points into valid ShmFanoutRing. send_pod writes T bytes
        // into SHM slots via SIMD-aware copy (invariant 6). pub_id identifies this
        // producer's SPSC channel within the fanout matrix.
        unsafe { ring.send_pod(&msg, pub_id) }
    } else {
        // Serde path: serialize once, send bytes to all subscribers.
        match bincode::serialize(&msg) {
            Ok(bytes) if bytes.len() > SPILL_THRESHOLD => {
                // Large serde message: spill the bytes to the TensorPool ONCE and
                // broadcast a 40-byte SpillDescriptor through every subscriber
                // channel (fits the fixed 8 KiB fanout slot). Without this, a
                // message larger than the slot is dropped on multi-subscriber
                // FanoutShm topics — whereas SpscShm auto-grows to deliver it.
                //
                // COMM-H3: hold each spilled slot alive (`spill_keepalive`) until its
                // ring position is overwritten — after `window` more sends no
                // subscriber can reach the descriptor (drop-oldest) — then release it,
                // so the pool can't fill up and silently drop large messages. Readers
                // `try_retain` around their read (`read_spilled_retained`), so a
                // release here can never tear an in-progress read. Evict BEFORE the
                // alloc so the pool always has a free slot for the new spill.
                let pool = topic.get_or_create_spill_pool();
                let window = (local.cached_capacity as usize).max(1);
                while local.spill_keepalive.len() >= window {
                    if let Some(old) = local.spill_keepalive.pop_front() {
                        pool.release(&old);
                    }
                }
                match spill_to_pool(topic, &bytes) {
                    Some(desc) => {
                        // Hold the alloc refcount until the ring overwrites this slot.
                        local.spill_keepalive.push_back(desc.to_tensor());
                        // SAFETY: SpillDescriptor is repr(C), Copy, 40 bytes with
                        // no padding and no pointers — a raw byte view is sound.
                        let desc_bytes = unsafe {
                            std::slice::from_raw_parts(
                                &desc as *const SpillDescriptor as *const u8,
                                std::mem::size_of::<SpillDescriptor>(),
                            )
                        };
                        // SAFETY: writes the descriptor bytes into SHM slots.
                        unsafe { ring.send_serde(desc_bytes, pub_id) }
                    }
                    // Spill failed (pool full/OOM): fall back to inline. Still
                    // bounded by the fixed slot, but preserves prior behavior.
                    None => unsafe { ring.send_serde(&bytes, pub_id) },
                }
            }
            // SAFETY: same as send_pod — writes serialized bytes into SHM slots.
            Ok(bytes) => unsafe { ring.send_serde(&bytes, pub_id) },
            Err(_) => false,
        }
    };

    if ok {
        housekeep_lease!(local, topic);
        Ok(())
    } else {
        Err(msg)
    }
}

/// Recv via ShmFanoutRing — registers subscriber on first call, round-robin polls publishers.
#[inline(always)]
pub(super) fn recv_fanout_shm<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static>(
    topic: &RingTopic<T>,
) -> Option<T> {
    epoch_guard_recv!(topic);

    // SAFETY: backend UnsafeCell accessed from single owning thread (invariant 1).
    // FanoutShm variant guaranteed by set_dispatch_fn_ptrs (invariant 4).
    let ring = match unsafe { &*topic.backend.get() } {
        BackendStorage::FanoutShm(r) => r,
        _ => debug_unreachable!("dispatch: expected FanoutShm variant in recv"),
    };

    let local = topic.local();

    // Lazy subscriber registration — first recv registers this Topic as a subscriber.
    // register_subscriber returns None only when all 16 endpoint slots are
    // simultaneously live (COMM-H1) — nothing to receive until a slot frees;
    // recoverable, not the pre-fix panic.
    let sub_id = match local.fanout_shm_sub_id {
        Some(id) => id,
        None => match ring.register_subscriber_locked(topic.name()) {
            Some((id, lock)) => {
                local.fanout_shm_sub_id = Some(id);
                // Hold the endpoint flock for this Topic's lifetime (COMM-H1).
                local.fanout_sub_lock = Some(lock);
                id
            }
            None => {
                // COMM-H1: same as the send path — loud once, never silent.
                static WARNED: AtomicBool = AtomicBool::new(false);
                if !WARNED.swap(true, Ordering::Relaxed) {
                    tracing::warn!(
                        "FanoutShm topic '{}': all {} subscriber endpoint slots are \
                         live — this subscriber has NO comms until a slot frees \
                         (COMM-H1 capacity limit, not a panic).",
                        topic.name(),
                        super::shm_fanout::MAX_FANOUT_ENDPOINTS,
                    );
                }
                return None;
            }
        },
    };

    let result: Option<T> = if ring.is_pod() {
        // SAFETY: ring points into valid ShmFanoutRing. recv_pod reads T bytes
        // from SHM slots via SIMD-aware copy (invariant 6). sub_id identifies this
        // consumer's SPSC channel within the fanout matrix.
        unsafe { ring.recv_pod(sub_id) }
    } else {
        // SAFETY: same as recv_pod — reads serialized bytes from SHM. A payload
        // that is exactly a sentinel-prefixed SpillDescriptor (40 bytes) is a
        // spilled large message → reconstruct from the pool; otherwise the bytes
        // are an inline bincode message.
        unsafe {
            ring.recv_serde(sub_id).and_then(|bytes| {
                if bytes.len() == std::mem::size_of::<SpillDescriptor>()
                    && u64::from_ne_bytes(bytes[..8].try_into().expect("len == 40 >= 8"))
                        == SPILL_SENTINEL
                {
                    // SAFETY: `bytes` holds exactly a SpillDescriptor (repr(C), 40B);
                    // read_unaligned copies it out with no alignment assumption.
                    let desc =
                        std::ptr::read_unaligned(bytes.as_ptr() as *const SpillDescriptor);
                    read_spilled_retained::<T>(desc, topic.name())
                } else {
                    bincode::deserialize(&bytes).ok()
                }
            })
        }
    };

    housekeep_recv!(local, topic, result);
    result
}

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
    let new_seq = seq.wrapping_add(1);
    // SAFETY: cached_data_ptr points to SHM data region. index < capacity (mask).
    // cached_seq_ptr points to the per-slot ready-flag array (set unconditionally
    // in ensure_role). index*8 is within bounds. simd_aware_write handles alignment
    // (slot_size is rounded to align_of::<T>()).
    unsafe {
        let base = local.cached_data_ptr as *mut T;
        simd_aware_write(base.add(index), msg);
        // Publish the per-slot ready flag in addition to sequence_or_head. This
        // is what an MpscShm consumer gates on (recv_shm_mpsc_pod checks
        // seq_array[idx] == tail+1), whereas the SpscShm consumer gates on
        // sequence_or_head. Writing both means every message sent under SpscShm
        // stays readable if the topic later migrates SpscShm -> MpscShm (a second
        // producer joining) — without it, the buffered [tail,head) range becomes
        // permanently unreadable after the switch (softmata-brain 1327, the
        // sp->mp ready-flag conversion gap). The extra Release store is inert for
        // the SpscShm consumer (which never reads the seq array) and is issued by
        // the same single producer in the same happens-before as the
        // sequence_or_head publish below, so it adds no new concurrency.
        let ready_ptr =
            &*(local.cached_seq_ptr.add(index * 8) as *const std::sync::atomic::AtomicU64);
        ready_ptr.store(new_seq, Ordering::Release);
    }
    local.local_head = new_seq;
    header.sequence_or_head.store(new_seq, Ordering::Release);

    housekeep_lease!(local, topic);
    Ok(())
}

// ---------------------------------------------------------------------------
// 7. SHM multi-producer POD (MpscShm)
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

    // Claim a slot with CAS so we NEVER overshoot the ring's free window.
    //
    // An optimistic `fetch_add` (the previous approach) claims unconditionally: two
    // producers that both pass a non-atomic capacity check then both fetch_add can
    // claim `seq >= tail + capacity`, and the write below would overwrite an
    // UNCONSUMED slot (it aliases `seq - capacity`) with a wrong ready-flag —
    // corrupting data and stalling the in-order consumer forever (mp_send_no_overshoot
    // _corruption). CAS makes the room check and the claim atomic: we only advance
    // head when `head - tail < capacity` still holds at claim time, so no claim ever
    // targets a live slot. On a genuinely full ring we return Err (the non-blocking
    // try_send contract is preserved — we never wait on the consumer). `header.tail`
    // lags (batched), which is conservative: it can only make us reject early.
    let seq = loop {
        let head = header.sequence_or_head.load(Ordering::Acquire);
        if head.wrapping_sub(local.local_tail) >= capacity {
            local.local_tail = header.tail.load(Ordering::Acquire);
            if head.wrapping_sub(local.local_tail) >= capacity {
                return Err(msg);
            }
        }
        match header.sequence_or_head.compare_exchange_weak(
            head,
            head.wrapping_add(1),
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => break head,
            // Another producer claimed between our load and CAS (or a spurious weak
            // failure). Retry with a fresh head — bounded by the number of concurrent
            // producers, which all make progress.
            Err(_) => std::hint::spin_loop(),
        }
    };

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

    let seq = header.sequence_or_head.fetch_add(1, Ordering::Relaxed);
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
                    topic.name(),
                    bytes.len()
                );
                // Fall back to auto-grow if pool alloc fails
                if bytes.len() > max_data_len {
                    let _ = topic.auto_grow_slot_size(bytes.len());
                    return Err(msg);
                }
                // If it fits inline despite being above threshold, just inline it
                // (this shouldn't happen, but handle gracefully)
                // SAFETY: cached_data_ptr points into SHM data region (invariant 2).
                // slot_offset < capacity * slot_size (invariant 3). bytes.len() <= max_data_len
                // (checked above). Writes length header + data into serde slot layout.
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
// 10. SHM multi-producer serde (MpscShm, non-POD)
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
                    topic.name(),
                    bytes.len()
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

    // Claim a slot with CAS so we never overshoot the ring's free window and
    // overwrite an unconsumed slot (see send_shm_mp_pod for the full rationale and
    // mp_send_no_overshoot_corruption for the gate). On a genuinely full ring we
    // return Err. Rare corner: if we already spilled a large message to the pool and
    // the ring turns out full here, that pool slot is orphaned — the pool reclaims it
    // by generation-tagged reallocation (spill is rare; matches the existing
    // best-effort spill lifetime), so it is a self-healing soft leak, not corruption.
    let seq = loop {
        let head = header.sequence_or_head.load(Ordering::Acquire);
        if head.wrapping_sub(local.local_tail) >= capacity {
            local.local_tail = header.tail.load(Ordering::Acquire);
            if head.wrapping_sub(local.local_tail) >= capacity {
                return Err(msg);
            }
        }
        match header.sequence_or_head.compare_exchange_weak(
            head,
            head.wrapping_add(1),
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => break head,
            Err(_) => std::hint::spin_loop(),
        }
    };
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
    // Batch header.tail updates for performance. The batch interval must be
    // SMALLER than the ring capacity, otherwise the producer sees a stale tail
    // and drops messages (backpressure) even though the consumer has read them.
    //
    // Use min(capacity/2, 32) as the flush interval:
    // - capacity/2 ensures the producer always has headroom
    // - 32 caps the interval for large-capacity rings
    let flush_interval = (local.cached_capacity / 2).max(1);
    let flush_mask = if flush_interval.is_power_of_two() {
        flush_interval - 1
    } else {
        flush_interval.next_power_of_two() - 1
    };
    if new_tail & flush_mask == 0 {
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
    // Batch header.tail updates. Interval must be < capacity to avoid
    // backpressure when the producer has written but consumer hasn't flushed.
    let flush_interval = (local.cached_capacity / 2).max(1);
    let flush_mask = if flush_interval.is_power_of_two() {
        flush_interval - 1
    } else {
        flush_interval.next_power_of_two() - 1
    };
    if new_tail & flush_mask == 0 {
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
    // Ring occupancy (head - tail) is always in [0, capacity]. A computed value
    // ABOVE capacity means `tail` has overshot `head` — the wrapping_sub wrapped
    // around — so the slots ahead were never written by the producer. Reading
    // them would return Some forever, spinning a multi-consumer drain loop into a
    // hang (observed with N racing consumers when tail races past head). Treat
    // both empty (== 0) and overshoot (> capacity) as drained.
    let cap = mask.wrapping_add(1);
    let drained = |head: u64, tail: u64| -> bool {
        let avail = head.wrapping_sub(tail);
        avail == 0 || avail > cap
    };
    for _attempt in 0..8 {
        let tail = header.tail.load(Ordering::Acquire);
        if drained(local.local_head, tail) {
            local.local_head = header.sequence_or_head.load(Ordering::Acquire);
            if drained(local.local_head, tail) {
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
                Ordering::Relaxed, // Relaxed: ready flag Acquire provides ordering
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
    let msg = unsafe {
        read_serde_slot::<T>(
            local.cached_data_ptr.add(slot_offset),
            slot_size,
            topic.name(),
        )
    }?;

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
    let msg = unsafe {
        read_serde_slot::<T>(
            local.cached_data_ptr.add(slot_offset),
            slot_size,
            topic.name(),
        )
    }?;

    let new_tail = tail.wrapping_add(1);
    local.local_tail = new_tail;
    // Batch header.tail updates — MUST match recv_shm_mpsc_pod. header.tail is the
    // SHARED consumed frontier; flushing it on EVERY recv makes it track a single
    // consumer's progress, so a second same-process subscriber that syncs its
    // local_tail up to header.tail is starved (reads 0) — the POD path's lagging
    // (batched) frontier is what let POD multi-sub broadcast work while this serde
    // path, flushing eagerly, silently delivered to only one subscriber. Interval
    // < capacity avoids producer backpressure. (Broadcast via independent per-handle
    // local_tail is only sound while the frontier lags and messages fit the ring;
    // a designed broadcast backend is the general answer — see roadmap-mrgqzlmb-ixl127.)
    let flush_interval = (local.cached_capacity / 2).max(1);
    let flush_mask = if flush_interval.is_power_of_two() {
        flush_interval - 1
    } else {
        flush_interval.next_power_of_two() - 1
    };
    if new_tail & flush_mask == 0 {
        header.tail.store(new_tail, Ordering::Release);
    }

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

    // Occupancy (head - tail) is in [0, capacity]; a value above capacity means
    // `tail` overshot `head` (wrapping_sub wrapped) — treat as drained so racing
    // consumers never CAS-claim a never-written slot and spin forever. See
    // recv_shm_spmc_pod for the full rationale.
    let cap = mask.wrapping_add(1);
    let drained = |head: u64, tail: u64| -> bool {
        let avail = head.wrapping_sub(tail);
        avail == 0 || avail > cap
    };
    let tail = header.tail.load(Ordering::Acquire);
    if drained(local.local_head, tail) {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if drained(local.local_head, tail) {
            housekeep_epoch!(local, topic);
            return None;
        }
    }

    if header
        .tail
        .compare_exchange_weak(
            tail,
            tail.wrapping_add(1),
            Ordering::Relaxed, // Relaxed: ready flag Acquire provides ordering
            Ordering::Relaxed,
        )
        .is_ok()
    {
        let index = (tail & mask) as usize;
        let slot_offset = index * slot_size;

        // SAFETY: CAS succeeded so we own this slot. slot_ptr is valid for
        // slot_size bytes within the SHM data region.
        let msg = match unsafe {
            read_serde_slot::<T>(
                local.cached_data_ptr.add(slot_offset),
                slot_size,
                topic.name(),
            )
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
        let sentinel = SPILL_SENTINEL;
        assert!(
            sentinel > 1024 * 1024,
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
        let threshold = SPILL_THRESHOLD;
        assert_eq!(threshold, 4096);
        assert!(
            threshold >= 1024,
            "threshold too small — would spill tiny messages"
        );
        assert!(threshold <= 65536, "threshold too large — defeats purpose");
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
            99,
            3,
            42,
            2048,
            &[10000],
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

    // ====================================================================
    // Edge case tests — SpillDescriptor fields, sentinel, alignment, Pod
    // ====================================================================

    #[test]
    fn test_spill_sentinel_exact_value() {
        // Pin the sentinel to its documented hex value so accidental edits break loudly
        let sentinel = SPILL_SENTINEL;
        assert_eq!(sentinel, 0xDEAD_5911_CAFE_BABE);

        // High bits are set (0xDEAD prefix) — ensures it can never be a valid
        // inline message length since max slot size is 1MB (0x100000)
        assert!(
            sentinel >> 32 != 0,
            "sentinel must have high bits set to avoid collisions with valid lengths"
        );
    }

    #[test]
    fn test_spill_descriptor_zero_pool_id() {
        // pool_id == 0 is a valid edge case (first pool created)
        let tensor = crate::types::Tensor::new(
            0,
            0,
            0,
            0,
            &[1],
            crate::types::TensorDtype::U8,
            crate::types::Device::cpu(),
        );
        let spill = SpillDescriptor::from_tensor(&tensor, 1);
        assert_eq!(spill.sentinel, SPILL_SENTINEL);
        assert_eq!(spill.pool_id, 0);
        assert_eq!(spill.slot_id, 0);
        assert_eq!(spill.generation, 0);
        assert_eq!(spill.generation_hi, 0);
        assert_eq!(spill.offset, 0);
        assert_eq!(spill.size, 1);

        let recovered = spill.to_tensor();
        assert_eq!(recovered.pool_id, 0);
        assert_eq!(recovered.slot_id, 0);
        assert_eq!(recovered.generation, 0);
        assert_eq!(recovered.generation_hi, 0);
        assert_eq!(recovered.offset, 0);
        assert_eq!(recovered.size, 1);
    }

    #[test]
    fn test_spill_descriptor_max_offset() {
        // Stress the offset field with u64::MAX — pool implementations must
        // reject this, but SpillDescriptor is just a descriptor and stores it.
        let mut tensor = crate::types::Tensor::default();
        tensor.pool_id = 1;
        tensor.slot_id = 1;
        tensor.offset = u64::MAX;
        tensor.size = 0;

        let spill = SpillDescriptor::from_tensor(&tensor, 0);
        assert_eq!(spill.offset, u64::MAX);
        assert_eq!(spill.size, 0);

        let recovered = spill.to_tensor();
        assert_eq!(recovered.offset, u64::MAX);
        assert_eq!(recovered.size, 0);
    }

    #[test]
    fn test_spill_descriptor_zero_data_len() {
        // A zero-length serialized message is degenerate but valid at the
        // descriptor level (bincode can serialize () to 0 bytes).
        let tensor = crate::types::Tensor::new(
            5,
            10,
            999,
            4096,
            &[0],
            crate::types::TensorDtype::U8,
            crate::types::Device::cpu(),
        );
        let spill = SpillDescriptor::from_tensor(&tensor, 0);
        assert_eq!(spill.size, 0);
        assert_eq!(spill.sentinel, SPILL_SENTINEL);

        let recovered = spill.to_tensor();
        assert_eq!(recovered.size, 0);
        assert_eq!(recovered.shape[0], 0);
        assert_eq!(recovered.ndim, 1);
    }

    #[test]
    fn test_spill_descriptor_max_u32_fields() {
        // Push pool_id, slot_id, generation, generation_hi to u32::MAX
        let mut tensor = crate::types::Tensor::default();
        tensor.pool_id = u32::MAX;
        tensor.slot_id = u32::MAX;
        tensor.generation = u32::MAX;
        tensor.generation_hi = u32::MAX;
        tensor.offset = 0;
        tensor.size = 0;

        let spill = SpillDescriptor::from_tensor(&tensor, u64::MAX);
        assert_eq!(spill.pool_id, u32::MAX);
        assert_eq!(spill.slot_id, u32::MAX);
        assert_eq!(spill.generation, u32::MAX);
        assert_eq!(spill.generation_hi, u32::MAX);
        assert_eq!(spill.size, u64::MAX);

        let recovered = spill.to_tensor();
        assert_eq!(recovered.pool_id, u32::MAX);
        assert_eq!(recovered.slot_id, u32::MAX);
        assert_eq!(recovered.generation, u32::MAX);
        assert_eq!(recovered.generation_hi, u32::MAX);
        assert_eq!(recovered.size, u64::MAX);
    }

    #[test]
    fn test_spill_descriptor_alignment_and_size() {
        // repr(C) layout guarantees:
        // - sentinel: u64 at offset 0 (8 bytes)
        // - pool_id: u32 at offset 8 (4 bytes)
        // - slot_id: u32 at offset 12 (4 bytes)
        // - generation: u32 at offset 16 (4 bytes)
        // - generation_hi: u32 at offset 20 (4 bytes)
        // - offset: u64 at offset 24 (8 bytes)
        // - size: u64 at offset 32 (8 bytes)
        // Total: 40 bytes, alignment: 8 (from u64)
        assert_eq!(std::mem::size_of::<SpillDescriptor>(), 40);
        assert_eq!(std::mem::align_of::<SpillDescriptor>(), 8);
    }

    #[test]
    fn test_spill_descriptor_bytemuck_roundtrip() {
        // SpillDescriptor is repr(C) and all-plain-data. Verify that raw byte
        // reinterpretation produces identical field values (the property Pod
        // would guarantee, tested here without requiring the Pod impl).
        let tensor = crate::types::Tensor::new(
            77,
            33,
            0xAAAA_BBBB_CCCC_DDDD_u64,
            512,
            &[8192],
            crate::types::TensorDtype::U8,
            crate::types::Device::cpu(),
        );
        let original = SpillDescriptor::from_tensor(&tensor, 7777);

        // Reinterpret as bytes and back
        let bytes: &[u8] = unsafe {
            std::slice::from_raw_parts(
                &original as *const SpillDescriptor as *const u8,
                std::mem::size_of::<SpillDescriptor>(),
            )
        };
        assert_eq!(bytes.len(), 40);

        let roundtripped: SpillDescriptor =
            unsafe { std::ptr::read_unaligned(bytes.as_ptr() as *const SpillDescriptor) };
        assert_eq!(roundtripped.sentinel, original.sentinel);
        assert_eq!(roundtripped.pool_id, original.pool_id);
        assert_eq!(roundtripped.slot_id, original.slot_id);
        assert_eq!(roundtripped.generation, original.generation);
        assert_eq!(roundtripped.generation_hi, original.generation_hi);
        assert_eq!(roundtripped.offset, original.offset);
        assert_eq!(roundtripped.size, original.size);
    }

    #[test]
    fn test_spill_threshold_boundary_under() {
        // A message of exactly SPILL_THRESHOLD bytes does NOT spill
        // (the comparison is `bytes.len() > SPILL_THRESHOLD`, strictly greater)
        let at_threshold = SPILL_THRESHOLD;
        assert!(
            (at_threshold <= SPILL_THRESHOLD),
            "exactly at threshold must NOT spill"
        );

        let one_under = SPILL_THRESHOLD - 1;
        assert!(
            (one_under <= SPILL_THRESHOLD),
            "one byte under threshold must NOT spill"
        );
    }

    #[test]
    fn test_spill_threshold_boundary_over() {
        // A message one byte over SPILL_THRESHOLD DOES spill
        let one_over = SPILL_THRESHOLD + 1;
        assert!(
            one_over > SPILL_THRESHOLD,
            "one byte over threshold MUST spill"
        );
    }

    #[test]
    fn test_spill_threshold_is_power_of_two() {
        // SPILL_THRESHOLD = 4096 = 2^12. Being a power of two is not a hard
        // requirement, but it aligns with page sizes and cache lines, and
        // changes should be deliberate.
        assert!(
            SPILL_THRESHOLD.is_power_of_two(),
            "SPILL_THRESHOLD ({}) should be a power of two for alignment",
            SPILL_THRESHOLD
        );
    }

    #[test]
    fn test_is_spill_slot_all_zeros() {
        // An all-zero slot must NOT be detected as a spill
        let slot = [0u8; 64];
        unsafe {
            assert!(
                !is_spill_slot(slot.as_ptr()),
                "all-zero slot must not be a spill"
            );
        }
    }

    #[test]
    fn test_is_spill_slot_all_ones() {
        // An all-0xFF slot has 0xFFFF_FFFF_FFFF_FFFF at offset 8,
        // which does not equal SPILL_SENTINEL
        let slot = [0xFFu8; 64];
        unsafe {
            assert!(
                !is_spill_slot(slot.as_ptr()),
                "all-0xFF slot must not be a spill (sentinel is not 0xFFFF...)"
            );
        }
    }

    #[test]
    fn test_is_spill_slot_sentinel_wrong_offset() {
        // Sentinel at offset 0 (wrong position — should be at offset 8)
        let mut slot = [0u8; 64];
        slot[0..8].copy_from_slice(&SPILL_SENTINEL.to_ne_bytes());
        // offset 8..16 is all zeros, not the sentinel
        unsafe {
            assert!(
                !is_spill_slot(slot.as_ptr()),
                "sentinel at offset 0 (wrong position) must not be detected"
            );
        }
    }

    #[test]
    fn test_is_spill_slot_sentinel_off_by_one() {
        // Sentinel value +-1 must not match
        let mut slot_plus = [0u8; 64];
        slot_plus[8..16].copy_from_slice(&(SPILL_SENTINEL.wrapping_add(1)).to_ne_bytes());
        unsafe {
            assert!(
                !is_spill_slot(slot_plus.as_ptr()),
                "sentinel+1 must not match"
            );
        }

        let mut slot_minus = [0u8; 64];
        slot_minus[8..16].copy_from_slice(&(SPILL_SENTINEL.wrapping_sub(1)).to_ne_bytes());
        unsafe {
            assert!(
                !is_spill_slot(slot_minus.as_ptr()),
                "sentinel-1 must not match"
            );
        }
    }

    #[test]
    fn test_spill_descriptor_generation_split() {
        // Verify generation is correctly split into low/high u32 halves
        let gen_full: u64 = 0x1234_5678_9ABC_DEF0;
        let tensor = crate::types::Tensor::new(
            1,
            1,
            gen_full,
            0,
            &[64],
            crate::types::TensorDtype::U8,
            crate::types::Device::cpu(),
        );
        let spill = SpillDescriptor::from_tensor(&tensor, 64);

        // Tensor::new splits generation_full into low/high halves
        assert_eq!(spill.generation, gen_full as u32); // 0x9ABC_DEF0
        assert_eq!(spill.generation_hi, (gen_full >> 32) as u32); // 0x1234_5678

        // to_tensor must reconstruct the same halves
        let recovered = spill.to_tensor();
        assert_eq!(recovered.generation, gen_full as u32);
        assert_eq!(recovered.generation_hi, (gen_full >> 32) as u32);
        assert_eq!(recovered.generation_full(), gen_full);
    }

    #[test]
    fn test_spill_descriptor_to_tensor_dtype_and_ndim() {
        // to_tensor always sets dtype=U8 and ndim=1 regardless of input
        let tensor = crate::types::Tensor::new(
            1,
            1,
            0,
            0,
            &[100],
            crate::types::TensorDtype::U8,
            crate::types::Device::cpu(),
        );
        let spill = SpillDescriptor::from_tensor(&tensor, 100);
        let recovered = spill.to_tensor();
        assert_eq!(recovered.dtype, crate::types::TensorDtype::U8);
        assert_eq!(recovered.ndim, 1);
        // shape[0] == size (the serialized byte count)
        assert_eq!(recovered.shape[0], 100);
    }

    #[test]
    fn test_read_spill_descriptor_preserves_all_fields() {
        // Write a SpillDescriptor with every field set to distinct non-zero values,
        // then read it back through read_spill_descriptor and verify all fields.
        let desc = SpillDescriptor {
            sentinel: SPILL_SENTINEL,
            pool_id: 0xAABB_CCDD,
            slot_id: 0x1122_3344,
            generation: 0xDEAD_BEEF,
            generation_hi: 0xCAFE_F00D,
            offset: 0x0102_0304_0506_0708,
            size: 0x0A0B_0C0D_0E0F_1011,
        };

        let mut slot = [0u8; 64];
        let desc_bytes = unsafe {
            std::slice::from_raw_parts(
                &desc as *const SpillDescriptor as *const u8,
                std::mem::size_of::<SpillDescriptor>(),
            )
        };
        slot[8..8 + desc_bytes.len()].copy_from_slice(desc_bytes);

        unsafe {
            assert!(is_spill_slot(slot.as_ptr()));
            let recovered = read_spill_descriptor(slot.as_ptr());
            assert_eq!(recovered.sentinel, SPILL_SENTINEL);
            assert_eq!(recovered.pool_id, 0xAABB_CCDD);
            assert_eq!(recovered.slot_id, 0x1122_3344);
            assert_eq!(recovered.generation, 0xDEAD_BEEF);
            assert_eq!(recovered.generation_hi, 0xCAFE_F00D);
            assert_eq!(recovered.offset, 0x0102_0304_0506_0708);
            assert_eq!(recovered.size, 0x0A0B_0C0D_0E0F_1011);
        }
    }
}
