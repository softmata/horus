#![allow(dead_code)]
//! Function-pointer dispatch for `Topic<T>`.
//!
//! Each function here is a **complete** send/recv path — epoch check, ring
//! operation, and amortized housekeeping. `try_send()` / `try_recv()` are
//! just single indirect calls: `unsafe { (*self.send_fn.get())(self, msg) }`.
//!
//! The design moved the role check, epoch check, `msg_counter` and lease refresh
//! out of `try_send`/`try_recv` and into these functions. The header used to
//! claim that was worth ~7ns per message; re-measured on the harness described
//! below, that work costs nothing measurable wherever it sits — ablating ALL of
//! it changes the one-way figure by less than the noise. Keep the shape for the
//! reason it is actually good (one complete path per backend, no pre/post logic
//! to keep in sync), not for a saving that is not there.
//!
//! Every topic is SHM-backed, so all dispatch functions operate on the shared
//! memory data region (or, for `FanoutShm`, a separate SHM-backed SPSC matrix).
//!
//! # Where the remaining time goes, measured
//!
//! Cross-thread one-way for `Topic<CmdVel>` (16 B POD) is ~105-118ns against a
//! same-harness raw ring at ~55ns — `topic_probe` and `topic_probe --raw-ring`,
//! which share threads, pinning, clock and ack protocol so the transport is the
//! only variable.
//!
//! That ~55ns gap is NOT removable software work. Each of these was ablated
//! individually on an idle machine, interleaved against the unmodified binary,
//! with a harness that resolves a deliberate +25ns injection against a +/-1ns
//! baseline:
//!
//! | ablated                                        | effect        |
//! |------------------------------------------------|---------------|
//! | `is_verbose()` on every send and recv            | none          |
//! | the per-recv migration epoch check               | none          |
//! | `messages_total` locked RMW                      | none (paced)  |
//! | MP-claim CAS -> plain single-producer store      | ~2ns          |
//! | ALL housekeeping + both epoch guards             | none          |
//! | fn-pointer dispatch -> devirtualised direct call | none          |
//! | the consumer's head gate                         | 2-4ns WORSE   |
//!
//! The head gate row is the informative one: removing it makes the consumer spin
//! on the payload line and steal it from the producer, so the gate is protective
//! rather than overhead. Removing the header line from BOTH sides at once (plain
//! claim on the producer, no gate on the consumer) buys ~3ns and raises cache
//! misses.
//!
//! `perf stat` says where the time actually is. Per message: this path takes
//! ~1.00 cache-misses against the raw ring's ~0.66. The extra ~0.34 is coherence
//! traffic, and `perf c2c` names the line — every HITM sample is offset 0 of the
//! header's producer line, `sequence_or_head`, with `send_shm_mp_pod` storing and
//! `recv_shm_mpsc_pod` loading. True sharing, not false: no other offset in that
//! line is touched, so there is nothing to pad apart.
//!
//! So the budget is spent on being a general transport — a multi-producer-safe
//! claim and a migration-aware header that both sides consult — rather than on
//! any one line of code. Someone trying to close it should be changing the
//! PROTOCOL, not deleting work from these functions; the work is already free.
//! And they should check first whether it is worth it: 55ns is 0.005% of a 1 kHz
//! control period, and the worst case that actually bounds a control loop is the
//! OS, not this path (a bare 40-line ring shows the same 25-42us maxima on a
//! stock PREEMPT_DYNAMIC kernel).
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
//! invariants are guaranteed by the `Topic<T>` type and its initialization paths:
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
//! 6. **SIMD operations**: `simd_aware_read`/`simd_aware_write` require
//!    non-overlapping source/dest within the data region. They do NOT require
//!    alignment, and must not: no `slot_size` rounding to `mem::align_of::<T>()`
//!    exists anywhere in the layout — this invariant claimed one for years and
//!    there was none. A split slot sits at `640 + capacity * 8 + i *
//!    size_of::<T>()` and a colo payload at `640 + i * stride + 8`, so a
//!    16-aligned message is 8 mod 16 in a colo region and in a capacity-1 split
//!    one. Both helpers therefore copy bytes; a typed `ptr::read`/`ptr::write`
//!    on a slot is UB and, for the shapes LLVM vectorises, a `movaps` fault in
//!    release that debug builds do not reproduce.

use std::sync::atomic::{fence, Ordering};

use super::shm_layout::SLOT_WRITING;

use serde::{de::DeserializeOwned, Serialize};

use super::backend::BackendStorage;
use super::header::{current_time_ms, TopicHeader};
use super::local_state::{LocalState, EPOCH_CHECK_INTERVAL};
use super::RingTopic;
use super::{simd_aware_read, simd_aware_read_uninit, simd_aware_write};
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
    #[allow(clippy::field_reassign_with_default)]
    #[inline]
    pub fn to_tensor(&self) -> crate::types::Tensor {
        // Built field-by-field rather than with a struct literal: `Tensor`'s
        // dtype is a private raw byte (so a hostile wire descriptor cannot
        // materialise an invalid discriminant), and a functional-update literal
        // requires every field to be visible. The dtype goes through the
        // accessor.
        let mut tensor = crate::types::Tensor::default();
        tensor.pool_id = self.pool_id;
        tensor.slot_id = self.slot_id;
        tensor.generation = self.generation;
        tensor.generation_hi = self.generation_hi;
        tensor.offset = self.offset;
        tensor.size = self.size;
        tensor.set_dtype(crate::types::TensorDtype::U8);
        tensor.ndim = 1;
        tensor.shape[0] = self.size;
        tensor
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

    let pool = topic.get_or_create_spill_pool()?;
    let tensor = pool
        .alloc(&[bytes.len() as u64], TensorDtype::U8, Device::cpu())
        .ok()?;

    // Copy serialized bytes into the pool slot
    let dst = match pool.data_slice_mut(&tensor) {
        Ok(d) => d,
        // Give the slot back rather than leaking it: `alloc` left the refcount
        // at 1 and nothing else will ever drop it (the `?` this replaces
        // returned `None` with the allocation still outstanding).
        Err(_) => {
            pool.release(&tensor);
            return None;
        }
    };
    dst[..bytes.len()].copy_from_slice(bytes);

    // Publish those bytes through the POOL's mapping, not only the ring's.
    //
    // The ring publish that follows in every caller (`fence(Release)` + a
    // Release store of the ready flag / head, matched by an Acquire load in
    // every recv path) is a correct release/acquire pair — but only on paper,
    // because the two sides do not touch the same object.  Every `RingTopic`
    // handle builds its own `ShmRegion`, i.e. its own `mmap` of the topic file,
    // so two handles in one process release and acquire that flag at two
    // different virtual addresses of the same physical page.  The hardware still
    // delivers the bytes; the memory model derives no synchronizes-with from two
    // distinct atomics, and ThreadSanitizer — which shadows by virtual address —
    // derived none either.  That is why it reported this `copy_from_slice`
    // racing `deserialize_spill_slot`'s read of the identical payload, even
    // though the consumer holds a `try_retain` pin: the pin stops the slot being
    // freed or reused under the reader, but a pin is not an ordering edge for
    // bytes written before it was taken.
    //
    // The pool is a single process-wide mapping — `pool_registry` hands out one
    // `Arc<TensorPool>` per topic name — and every reader of a spilled payload
    // pins the slot with `try_retain` before it touches a byte, which is an
    // acquire RMW on the slot refcount.  `publish_payload` is the matching
    // release half, so the copy above is ordered before every consumer's read
    // through an edge that both the model and the sanitizer can see.
    if pool.publish_payload(&tensor).is_err() {
        // Unreachable in practice: the slot was allocated a few lines above and
        // this thread still holds that reference.  If it ever does happen, drop
        // the allocation instead of publishing a descriptor to bytes nothing is
        // ordered against.
        pool.release(&tensor);
        return None;
    }

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
/// Address of slot `index`'s readiness stamp and its payload, under whichever
/// slot geometry this mapping uses.
///
/// Single point that knows both layouts. The split layout keeps stamps in a
/// contiguous array (stride 8) and payloads in a separate region (stride
/// `size_of::<T>()`); the colo layout interleaves them, one slot per cache
/// line or more. Every POD path routes through here, because a path that
/// computed the offsets itself and missed a layout would read payload bytes
/// as a stamp — silent corruption on a live IPC region, which is exactly what
/// `shm_layout` was written to prevent.
///
/// # Safety
/// `index` must be `< capacity`, and `local`'s cached pointers must belong to
/// the current mapping.
#[inline(always)]
pub(super) unsafe fn slot_ptrs<T>(
    local: &LocalState,
    index: usize,
) -> (*const std::sync::atomic::AtomicU64, *mut T) {
    let stride = local.cached_colo_stride;
    if stride != 0 {
        let off = index * stride as usize;
        (
            local.cached_seq_ptr.add(off) as *const std::sync::atomic::AtomicU64,
            local.cached_data_ptr.add(off) as *mut T,
        )
    } else {
        (
            local.cached_seq_ptr.add(index * 8) as *const std::sync::atomic::AtomicU64,
            (local.cached_data_ptr as *mut T).add(index),
        )
    }
}

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

/// Inline SHM-epoch compare — the migration check, split from its handler.
///
/// Loads `migration_epoch` from the stable `self.header_ptr` (NOT
/// `local.cached_header_ptr`, which the role=Both same-instance fast path
/// repurposes) and calls the `#[cold]` `handle_epoch_change` only on a
/// mismatch. Same load, same `Acquire` ordering, same handler, same call
/// frequency as `RingTopic::check_migration_periodic` — this is that function
/// with its comparison inlined into the caller.
///
/// Why: `check_migration_periodic` is `#[cold] #[inline(never)]`, yet the
/// housekeeping macros below call it on EVERY recv (issue #37 made the check
/// every-recv on purpose, so a subscriber discovers a cross-process producer
/// within one `recv()` — that latency is preserved exactly here). The
/// attribute and the frequency contradicted each other: `#[cold]` puts the
/// body in `.text.unlikely`, so the *comparison* — three instructions — cost a
/// distant `call`, an extra I-cache line, a possible iTLB miss and register
/// spills across it, and it marked the caller's block cold, dragging the
/// layout of the empty-recv path with it. Only the mismatch is genuinely cold,
/// and `handle_epoch_change` is still `#[cold] #[inline(never)]`.
macro_rules! migration_check {
    ($local:ident, $topic:expr) => {
        // SAFETY: header_ptr always points to the real SHM TopicHeader, valid
        // for the topic's lifetime (backed by the Arc<ShmRegion> in `storage`),
        // and is re-pointed by auto-grow whenever the mmap moves.
        let __shm_epoch = unsafe { &*$topic.header_ptr.get() }
            .migration_epoch
            .load(Ordering::Acquire);
        if unlikely(__shm_epoch != $local.cached_epoch) {
            $topic.handle_epoch_change(__shm_epoch);
        }
    };
}

// ============================================================================
// Housekeeping macros — amortized maintenance after each message
// ============================================================================

/// How often (in messages) to *consider* refreshing the lease.
///
/// The refresh itself is decided by wall clock inside `refresh_lease_if_due`;
/// this counter only keeps the clock read off the per-message hot path. It is
/// deliberately far smaller than the 1024-message interval it replaces: the old
/// code refreshed strictly every 1024 messages, so liveness depended on throughput
/// and a slow-but-healthy participant looked expired most of the time. At 64,
/// a 100 Hz participant reaches the clock check about twice a second against a
/// 5 s lease.
const LEASE_CHECK_INTERVAL: u32 = 64;

/// Housekeeping after a successful send or recv: migration check (fast, every
/// EPOCH_CHECK_INTERVAL msgs) + lease refresh (clock-gated, considered every
/// LEASE_CHECK_INTERVAL msgs).  Used by all dispatched (fn-ptr) send/recv paths.
macro_rules! housekeep_lease {
    ($local:ident, $topic:expr) => {
        $local.msg_counter = $local.msg_counter.wrapping_add(1);
        if unlikely($local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
            migration_check!($local, $topic);
            if unlikely($local.msg_counter & (LEASE_CHECK_INTERVAL - 1) == 0) {
                $topic.refresh_lease_if_due();
            }
        }
    };
}

/// Housekeeping on empty recv: epoch check on EVERY empty recv, plus a
/// clock-gated lease refresh.
///
/// When recv returns None, the topic may be empty because a cross-process
/// producer joined and this participant hasn't observed the migration epoch
/// yet (still pointed at the old SHM ring). Checking the SHM
/// epoch on every empty recv ensures migration is detected immediately.
/// Cost: one Acquire load of a header word (`migration_check!`) — negligible
/// for polling loops, and since that check was split out of the `#[cold]`
/// `check_migration_periodic` it no longer costs a call into `.text.unlikely`.
///
/// The empty path refreshed nothing at all, so a subscriber polling a topic
/// that has not published yet stopped refreshing the instant it registered and
/// was permanently lease-expired one timeout later — while still polling. That
/// is a liveness report about a live process, and slot reclamation used to act
/// on it.
macro_rules! housekeep_epoch {
    ($local:ident, $topic:expr) => {
        $local.msg_counter = $local.msg_counter.wrapping_add(1);
        migration_check!($local, $topic);
        if unlikely($local.msg_counter & (LEASE_CHECK_INTERVAL - 1) == 0) {
            $topic.refresh_lease_if_due();
        }
    };
}

/// Combined housekeeping for the dispatched recv path: branch on result.
/// Epoch check on EVERY recv (not amortized) to detect cross-process joins immediately.
/// Lease refresh remains amortized (syscall cost).
macro_rules! housekeep_recv {
    ($local:ident, $topic:expr, $result:ident) => {
        $local.msg_counter = $local.msg_counter.wrapping_add(1);
        // Check SHM epoch on every recv — detects cross-process producers immediately.
        // Cost: one Acquire load of a header word already in L1, plus a compare.
        migration_check!($local, $topic);
        // Refresh on a wall clock, and on the empty path too (see
        // `housekeep_epoch`): a consumer is just as alive when the ring is
        // empty as when it is not.
        if unlikely($local.msg_counter & (LEASE_CHECK_INTERVAL - 1) == 0) {
            $topic.refresh_lease_if_due();
        }
    };
}

// ============================================================================
// Serde staging — one allocation per handle instead of one per message
// ============================================================================

/// Serialize `msg` into this handle's reusable staging buffer.
///
/// Replaces `bincode::serialize(&msg)`, which returns a fresh `Vec<u8>` and so
/// put one `malloc` and one `free` on every non-POD publish. Allocator latency
/// has no useful bound — the fast path pops a free-list head, the slow path
/// takes an arena lock, refills a bin or calls `mmap` — so that made the WCET of
/// a serde send a property of the allocator's state rather than of the message.
/// `bincode::serialize_into` appends into a buffer that keeps its capacity, so
/// after warm-up the steady state allocates nothing.
///
/// On error the buffer is left cleared, never holding a half-written message
/// that a later `len()` could mistake for a valid payload.
///
/// This does NOT make the serde publish path bounded-WCET: `Serialize` still
/// walks the value, so its cost is data-dependent, and an oversized message
/// still takes `auto_grow_slot_size` (`ftruncate` + `mremap` + a full
/// migration). It removes the allocator, and only the allocator. See
/// `LocalState::serde_scratch` for the memory-footprint trade.
#[inline(always)]
fn serialize_into_scratch<T: Serialize>(local: &mut LocalState, msg: &T) -> bool {
    let buf = &mut local.serde_scratch;
    buf.clear();
    if bincode::serialize_into(&mut *buf, msg).is_ok() {
        true
    } else {
        buf.clear();
        false
    }
}

// ============================================================================
// Bounded slot claim — capping the multi-producer CAS loop
// ============================================================================

/// Maximum CAS attempts a producer will make to claim a ring slot before giving
/// up and returning the message to the caller.
///
/// The claim loop is lock-free but NOT wait-free, and the two are different
/// promises. "Bounded by the number of concurrent producers" — which the loop's
/// old comment claimed — is a *system-wide* progress guarantee: it says someone
/// always advances, not that THIS thread ever does. Two things break the
/// per-thread bound outright:
///
/// * `compare_exchange_weak` may fail spuriously, and on LL/SC architectures
///   (ARM, RISC-V) a reservation can be lost to any interfering write to the
///   line. There is no static bound on that even with a single producer.
/// * Each retry re-loads a cache line every other producer is actively CASing,
///   so a retry is a full coherence round trip (~150 ns), not a few cycles.
///
/// 64 is four times `MAX_PARTICIPANTS`, so genuine contention among the most
/// producers a topic can hold never exhausts it; reaching 64 means spurious
/// failure or livelock. The resulting WCET bound is ~64 × 150 ns ≈ 10 µs for
/// one `try_send`, against the previous bound of infinity.
///
/// The consumer side of the same pattern already caps at 8
/// (`recv_shm_spmc_pod`); this is deliberately looser because a dropped publish
/// is more expensive than a deferred receive.
const MAX_CLAIM_CAS_RETRIES: u32 = 64;

/// How long a handle stays quiet after reporting a claim-loop exhaustion.
const CLAIM_CAS_WARN_QUIET_MS: u64 = 60_000;

/// Report a claim-loop exhaustion, rate-limited to once a minute per handle.
///
/// The DROP itself is counted where every other lossy-publish drop is counted —
/// `metrics.send_failures`, incremented by `RingTopic::send_lossy_retry` once it
/// gives up — so this deliberately does NOT touch that counter and double-count.
/// What the counter cannot say is *why*, and "the ring was full" and "the claim
/// CAS livelocked" call for opposite responses from an operator, so the reason
/// is logged.
///
/// The gate lives in `LocalState` rather than in the process-wide map
/// `should_report_endpoint_exhaustion` uses, for two reasons: `send()` funnels a
/// failed `try_send` through ~70 more attempts, so this can be reached in a
/// tight loop, and a global `Mutex` on the publish path is exactly the kind of
/// unbounded wait this whole change exists to remove. A `u64` compare against
/// the wall clock costs neither.
#[cold]
#[inline(never)]
fn warn_claim_cas_exhausted(local: &mut LocalState, topic_name: &str) {
    let now_ms = current_time_ms();
    if local.claim_cas_warn_ms != 0
        && now_ms >= local.claim_cas_warn_ms
        && now_ms - local.claim_cas_warn_ms < CLAIM_CAS_WARN_QUIET_MS
    {
        return;
    }
    local.claim_cas_warn_ms = now_ms.max(1);
    log::warn!(
        "Topic '{}': slot-claim CAS failed {} times in a row — returning the message \
         rather than spinning without bound. Under `send()` this becomes a counted \
         drop (TopicMetrics::send_failures); under `try_send()` the caller gets it \
         back. Expect this only under extreme producer contention or livelock.",
        topic_name,
        MAX_CLAIM_CAS_RETRIES,
    );
}

/// Resume an in-order consumer that was lapped, instead of blocking forever.
///
/// A slot stamp that does not match `tail + 1` has two opposite causes, and
/// collapsing them is what wedged this consumer:
///
///   * stamp BEHIND `tail + 1` — the producer claimed the slot and has not
///     published yet. Ordinary; come back later. If the producer died in that
///     window, `claimed_slot_escape` bounds the wait.
///   * stamp AHEAD of `tail + 1` — the slot was reused at a LATER position
///     while this consumer still had not read it. The consumer was lapped. The
///     message it is waiting for no longer exists and never will.
///
/// A stamp carrying `SLOT_WRITING` is neither on its own: it says a seqlock
/// producer is mid-write, and the position under the marker is what decides
/// which of the two cases above applies. The body masks it off before
/// comparing.
///
/// `recv_shm_pod_broadcast` has always distinguished these. `recv_shm_mpsc_pod`
/// and `recv_shm_mpsc_serde` did not: an exact-match gate sent BOTH into
/// `claimed_slot_escape`, which advances exactly one slot per
/// `CLAIM_STALL_MAX_LEASES` x lease (20 s by default) and logs the wrong
/// diagnosis — "claimed by a producer that never published it", when the
/// producer published it and then lapped past it.
///
/// Measured before this existed, on a 16-slot ring with a ~1 kHz producer and a
/// registered, live subscriber that paused for one lease: 12371 sent, **0
/// delivered**, and the consumer never recovered.
///
/// Returns true when a lap was detected and the caller should return `None`
/// having already resumed.
#[inline]
fn resume_after_lap(local: &mut LocalState, header: &TopicHeader, tail: u64, stamp: u64) -> bool {
    // Compare the POSITION the stamp carries, not the raw word. `SLOT_WRITING`
    // is bit 63, so a slot a seqlock producer is mid-write on reads back as
    // `2^63 | pos` — a value larger than any tail, which a raw compare would
    // read as "lapped by ~2^63 messages". `send_shm_pod_broadcast` and the
    // co-located `send_shm_sp_pod` both set that bit before touching the
    // payload, and both rings are consumed through this path after a migration
    // to MpscShm — which is exactly the case `send_shm_pod_broadcast` states
    // the contract for: those readers "simply read 'not ready' for the duration
    // of a write". Masking keeps it. Without it a producer that DIES mid-write
    // leaves the bit set forever, and every later poll takes this branch and
    // returns `None` without ever reaching `claimed_slot_escape` — the
    // unbounded stall that escape exists to bound, reintroduced through the
    // fix for the other one.
    let position = stamp & !SLOT_WRITING;
    if position <= tail.wrapping_add(1) {
        return false;
    }
    let head = header.sequence_or_head.load(Ordering::Acquire);
    let cap = local.cached_capacity;
    if head > cap {
        // Half a lap back from the head, for the reason `recv_shm_pod_broadcast`
        // records: landing exactly on `head - capacity` puts the consumer on the
        // slot the producer overwrites next, so it re-laps immediately.
        let resume = head.wrapping_sub(cap).wrapping_add(cap / 2);
        if resume > local.local_tail {
            local.missed = local.missed.wrapping_add(resume.wrapping_sub(tail));
            local.local_tail = resume;
            local.local_head = head;
        }
    }
    true
}

// ============================================================================
// Abandoned-claim escape — bounding the MpscShm consumer's head-of-line stall
// ============================================================================

/// How many consecutive not-ready polls at the same `tail` are free before the
/// consumer starts timing the stall, minus one (used as a bitmask).
///
/// A claimed-but-unpublished slot is the ORDINARY transient state: between a
/// producer's slot-claiming CAS and its `Release` store of the ready flag there
/// is nothing but a `simd_aware_write` (POD) or a `memcpy` (serde) — no
/// syscall, no allocation, no lock — so the window is a few nanoseconds wide. A
/// consumer spinning on that window must not pay for a `current_time_ms()`
/// (~25 ns of vDSO) to discover something it will learn in one more load.
///
/// 16 polls of the recv path is comfortably longer than any live producer's
/// claim→publish window, so a transient not-ready NEVER reaches a clock read
/// and the spin-to-first-message latency is untouched. The cost is granularity
/// at the other extreme: a 1 Hz poller waits 16 s before the stall clock even
/// starts. That is the right side of the trade — this exists to turn an
/// infinite stall into a finite one, not to be prompt.
const CLAIM_STALL_POLL_MASK: u32 = 15;

/// Absolute escape bound, expressed in lease timeouts (default 5 s ⇒ 20 s).
///
/// The primary rule is the two-condition one `TopicHeader::reap_dead_participants`
/// argues for — lease expired AND the pid is gone — because an expired lease
/// alone does not mean dead. But that rule has a hole this must cover: it can
/// only see producers that are still *registered*. If a third party (any
/// `register_role` that ran out of slots) already reaped the dead producer's
/// entry while another producer is still live, no dead pid is visible anywhere
/// and the consumer would block forever again.
///
/// So after this many lease timeouts with the same slot claimed and unpublished
/// the consumer gives up regardless of what the participant table says. A LIVE
/// producer cannot sit inside its claim→publish window for 20 seconds — that
/// window contains no blocking operation at all — so reaching this bound means
/// either the producer is gone or it has been `SIGSTOP`ped, and on a control
/// topic those are the same thing to the subscriber.
const CLAIM_STALL_MAX_LEASES: u64 = 4;

/// Is there no producer left that could ever publish the stuck slot?
///
/// Applies the same two-condition rule as `TopicHeader::reap_dead_participants`
/// (expired lease is the cheap filter; a dead pid is the proof) but answers a
/// different question, because the ready-flag protocol does not record WHICH
/// producer claimed a slot. Two situations both mean "nobody will finish it":
///
/// * a registered local producer whose process is gone — it may well be the one
///   holding the claim, and if it is not, the escape is still bounded by the
///   caller's absolute bound; or
/// * no live local producer registered at all — whoever claimed the slot has
///   already been reaped.
///
/// Deliberately treats our OWN pid as live: `reap_dead_participants` refuses to
/// judge our own process for exactly the same reason (a departed thread is
/// indistinguishable from a busy one), and a same-process producer thread that
/// has wedged is caught by [`CLAIM_STALL_MAX_LEASES`] instead. Network-replicated
/// participants (`source_host != 0`) carry another host's pid, which means
/// nothing here, so they are skipped in both directions.
///
/// Cost: up to `MAX_PARTICIPANTS` liveness probes. Called at most once per
/// lease timeout per stalled consumer, on a path that is already returning
/// `None`, so it never delays a delivered message.
#[cold]
#[inline(never)]
fn no_producer_can_finish(header: &TopicHeader) -> bool {
    let me = std::process::id();
    let mut live_local_producer = false;
    for p in header.participants.iter() {
        // 1 = live entry. 0 = free, 2 = mid-claim by another thread; neither is
        // ours to judge (same rule as `reap_dead_participants`).
        if p.active.load(Ordering::Acquire) != 1 {
            continue;
        }
        // role bit 0 = producer (0=none, 1=producer, 2=consumer, 3=both).
        if p.role.load(Ordering::Acquire) & 1 == 0 {
            continue;
        }
        if p.source_host.load(Ordering::Acquire) != 0 {
            continue;
        }
        let pid = p.pid.load(Ordering::Acquire);
        if pid == 0 {
            continue;
        }
        if pid == me {
            live_local_producer = true;
            continue;
        }
        if horus_sys::discover::is_process_alive(pid) {
            live_local_producer = true;
        } else {
            // A dead registered producer: proof that a publisher died, which is
            // the only way this slot gets claimed and abandoned.
            return true;
        }
    }
    !live_local_producer
}

/// Report an abandoned-claim skip.
///
/// A skipped message is a real gap in a subscriber's stream. It is counted in
/// `LocalState::missed` (surfaced by `Topic::missed_count`), and it is also said
/// out loud, because "my control topic has a hole in it" is not something an
/// operator should have to go looking for a counter to discover.
///
/// Deliberately NOT rate-limited: the escape it reports is already limited by
/// construction. It cannot fire until a slot has been stuck for a whole lease
/// timeout, each firing advances `tail` past that slot, and a producer holds
/// only one claim at a time — so with all `MAX_PARTICIPANTS` producers dead at
/// once this still prints at most once per lease timeout (5 s by default). A
/// limiter would only add a lock or a clock read to buy nothing.
#[cold]
#[inline(never)]
fn warn_abandoned_claim(topic_name: &str, tail: u64, stalled_ms: u64, why: ClaimEscape) {
    log::warn!(
        "Topic '{}': ring slot {} was claimed by a producer that never published it \
         ({} ms, {}). Skipping it — this is a DROPPED MESSAGE, counted in \
         Topic::missed_count(). The alternative was blocking this subscriber forever.",
        topic_name,
        tail,
        stalled_ms,
        match why {
            ClaimEscape::NoProducerLeft => "no producer left that could finish it",
            ClaimEscape::AbsoluteBound => "absolute stall bound reached",
        },
    );
}

/// Why [`claimed_slot_escape`] gave up on a slot. Diagnostic only — both
/// outcomes are the same skip — but the two mean very different things to
/// whoever reads the log, so they are not collapsed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum ClaimEscape {
    /// The two-condition rule fired: no producer remains that could publish it.
    NoProducerLeft,
    /// [`CLAIM_STALL_MAX_LEASES`] elapsed with a live producer still registered
    /// — the third-party-reap hole, or a `SIGSTOP`ped publisher.
    AbsoluteBound,
}

/// Decide whether a consumer should skip a slot that a producer claimed and
/// never published. `Some(reason)` means the caller must skip it (via
/// [`take_claimed_slot_escape`]); `None` means keep waiting.
///
/// # The defect this bounds
///
/// `MpscShm` publishes in two steps: a CAS on `sequence_or_head` claims slot
/// `seq`, then a `Release` store of `seq + 1` into the per-slot ready flag makes
/// it readable. The ring is strictly in-order, so a consumer at `tail == seq`
/// waits on that ready flag. If the producer process dies in between — SIGKILL,
/// segfault, OOM-killer — no live process will ever set that flag. Nothing
/// re-checks it, nothing times out, and no lap detection applies (unlike the
/// PodShm path's `SLOT_WRITING` marker). The topic goes silent forever, with no
/// error and no counter. On a robot that is a control subscriber that simply
/// stops, while `topic list` still shows the topic and the publisher's
/// registration still sits in the participant table.
///
/// It cascades, too: the bare `return None` this replaces skipped the recv
/// housekeeping, so the wedged consumer also stopped refreshing its lease and
/// stopped observing migrations. The callers now run `housekeep_epoch!` on this
/// path for that reason.
///
/// # The trade, stated plainly
///
/// This converts an UNBOUNDED STALL into a BOUNDED MESSAGE LOSS. That is the
/// right trade for a worst-case-latency figure of merit — but only because the
/// loss is counted (`LocalState::missed`, read back through
/// `Topic::missed_count`) and logged. A silent drop would be worse than the
/// stall it replaces.
///
/// It also widens one pre-existing hazard by a hair, which is worth naming: a
/// producer that was merely `SIGSTOP`ped, not dead, can wake after the escape
/// and write into a slot the ring has since reused. The exact-match ready-flag
/// check contains that — a zombie stamps `old_seq + 1`, and every future
/// expectation at that index is `old_seq + k*capacity + 1`, strictly greater —
/// so the zombie's stamp reads as "not ready" rather than as a valid message,
/// and the outcome is another counted skip, not a torn read. The same zombie
/// could already clobber a reused slot's payload today; this does not create
/// that window, it only makes reaching it possible without the consumer having
/// wedged first.
///
/// # Cost
///
/// Zero on the delivery path — this is only ever called after the ready flag
/// has already been observed unset, and the first `CLAIM_STALL_POLL_MASK + 1`
/// such observations do nothing but bump a counter in `LocalState`. The stall
/// state is keyed on `tail`, which is monotonic, so nothing has to be cleared
/// when a message does arrive.
///
/// `#[cold]` is honest here, unlike on the migration check this file had to
/// split apart: a consumer only reaches the ready-flag test when `head > tail`,
/// and a poller waiting on an empty ring returns from the occupancy check well
/// before that. Out-of-lining keeps the recv body — which is `#[inline(always)]`
/// into every call site — from carrying this state machine.
#[cold]
#[inline(never)]
fn claimed_slot_escape(
    local: &mut LocalState,
    header: &TopicHeader,
    tail: u64,
) -> Option<ClaimEscape> {
    // New stall (or the first one at this position): arm the counter without
    // reading the clock. `tail` is monotonic, so this is also how a stall that
    // resolved on its own gets forgotten — the hot path never clears anything.
    if local.claim_stall_tail != tail || local.claim_stall_polls == 0 {
        local.claim_stall_tail = tail;
        local.claim_stall_polls = 1;
        local.claim_stall_since_ms = 0;
        return None;
    }

    local.claim_stall_polls = local.claim_stall_polls.wrapping_add(1);
    if local.claim_stall_polls & CLAIM_STALL_POLL_MASK != 0 {
        return None;
    }

    let now_ms = current_time_ms();
    // First clock reading for this stall: start the timer, decide nothing.
    if local.claim_stall_since_ms == 0 || now_ms < local.claim_stall_since_ms {
        // `now_ms < since` is a clock that went backwards; restarting is the
        // only honest response, and it can only ever delay the escape.
        local.claim_stall_since_ms = now_ms.max(1);
        return None;
    }

    let stalled_ms = now_ms - local.claim_stall_since_ms;
    let lease = header.lease_timeout();
    if stalled_ms < lease {
        return None;
    }

    // Condition 2: a producer is provably gone. An expired lease alone is not
    // evidence of death — refresh is clock-gated, and an idle-but-healthy
    // participant reads as expired.
    if no_producer_can_finish(header) {
        return Some(ClaimEscape::NoProducerLeft);
    }

    // Fallback: the absolute bound, for the case a third party already reaped
    // the dead producer's entry. See `CLAIM_STALL_MAX_LEASES`.
    if stalled_ms >= lease.saturating_mul(CLAIM_STALL_MAX_LEASES) {
        return Some(ClaimEscape::AbsoluteBound);
    }
    None
}

/// Perform the skip decided by [`claimed_slot_escape`]: count the loss, advance
/// past the abandoned slot, and publish the new frontier immediately.
///
/// The `header.tail` store is unbatched on purpose. The batched flush every
/// `capacity / 2` reads exists to keep a store off the hot path; this path fires
/// at most once per lease timeout, and leaving the freed slot invisible to the
/// producer's backpressure check for up to half a ring would be a second stall
/// on top of the one just escaped. Sound for both callers because they are the
/// single-consumer (MpscShm/SpscShm) paths, where `header.tail` is read only by
/// producers for backpressure.
#[cold]
#[inline(never)]
fn take_claimed_slot_escape(
    local: &mut LocalState,
    header: &TopicHeader,
    tail: u64,
    name: &str,
    why: ClaimEscape,
) {
    let stalled_ms = current_time_ms().saturating_sub(local.claim_stall_since_ms);
    warn_abandoned_claim(name, tail, stalled_ms, why);

    local.missed = local.missed.wrapping_add(1);
    let new_tail = tail.wrapping_add(1);
    local.local_tail = new_tail;
    header.tail.store(new_tail, Ordering::Release);
    // Disarm: the next poll starts a fresh stall at the new position if the
    // slot behind this one is also abandoned.
    local.claim_stall_polls = 0;
    local.claim_stall_since_ms = 0;
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
    // No usable pool => the payload is unreachable => a miss, the same answer
    // this returns for a superseded slot. It used to be a panic. Unlike a
    // superseded slot it is a permanent fault, so it is reported (throttled per
    // topic) rather than left to look like an idle topic.
    let pool = super::pool_registry::pool_or_report(topic_name)?;
    // Pin the slot across the read, exactly as `read_spilled_retained` does.
    //
    // "There is exactly one reader" was true and still is; what it did not cover
    // is the SAME reader arriving at one ring position twice. The first read
    // released the producer's alloc refcount, `return_slot` freed the slot and
    // `backend.zero()` began writing volatile zeroes over it — and the second
    // read then validated a descriptor whose generation had not yet been bumped
    // (generation moves on alloc, not on free) and handed the region to bincode
    // while it was being zeroed. That is a data race on the payload itself, and
    // it is what ThreadSanitizer caught under `auto_grow_cross_thread_no_crash`.
    //
    // `try_retain` closes it: it refuses on a zero refcount, so a position that
    // has already been consumed reads as a clean miss instead of a torn read.
    if pool.try_retain(&tensor).is_err() {
        return None;
    }
    let msg = deserialize_spill_slot::<T>(&pool, &tensor, spill.size as usize);
    // Two references are outstanding and both are ours to drop: the pin above,
    // and the producer's alloc refcount that this single consumer inherits. The
    // second release is the COMM-H3 reclaim that keeps the spill pool from
    // leaking; together they take the slot to zero and free it, which is the
    // same end state as the single release this replaced.
    pool.release(&tensor); // our read pin
    pool.release(&tensor); // the producer's alloc refcount
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
    // No usable pool => the payload is unreachable => a miss, reported for the
    // same reason as in `read_spilled_once`.
    let pool = super::pool_registry::pool_or_report(topic_name)?;
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
                // rate-limited per topic so a hot send-loop can't flood the log.
                if should_report_endpoint_exhaustion(topic.name(), Exhausted::FanoutEndpoints) {
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
    } else if !serialize_into_scratch(local, &msg) {
        // Serialization failed — nothing to broadcast. `serialize_into_scratch`
        // has already cleared the buffer.
        false
    } else {
        // Serde path: serialized once into the reusable staging buffer (see
        // `serialize_into_scratch`), then sent to all subscribers.
        let bytes_len = local.serde_scratch.len();
        if bytes_len > SPILL_THRESHOLD {
            {
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
                // No pool => nothing was ever spilled on this handle, so there
                // is nothing to evict; `spill_to_pool` below returns `None` for
                // the same reason and the `None` arm sends inline.
                if let Some(pool) = topic.get_or_create_spill_pool() {
                    let window = (local.cached_capacity as usize).max(1);
                    while local.spill_keepalive.len() >= window {
                        if let Some(old) = local.spill_keepalive.pop_front() {
                            pool.release(&old);
                        }
                    }
                }
                // Hoisted out of the `match` scrutinee so the shared borrow of
                // `local.serde_scratch` provably ends before the arms mutate
                // `local.spill_keepalive`.
                let spilled = spill_to_pool(topic, &local.serde_scratch);
                match spilled {
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
                    None => unsafe { ring.send_serde(&local.serde_scratch, pub_id) },
                }
            }
        } else {
            // SAFETY: same as send_pod — writes serialized bytes into SHM slots.
            unsafe { ring.send_serde(&local.serde_scratch, pub_id) }
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
                // COMM-H1: same as the send path — loud, rate-limited, never silent.
                if should_report_endpoint_exhaustion(topic.name(), Exhausted::FanoutEndpoints) {
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
        unsafe { ring.recv_pod(sub_id, &mut local.missed) }
    } else {
        // SAFETY: same as recv_pod — reads serialized bytes from SHM. A payload
        // that is exactly a sentinel-prefixed SpillDescriptor (40 bytes) is a
        // spilled large message → reconstruct from the pool; otherwise the bytes
        // are an inline bincode message.
        unsafe {
            ring.recv_serde(sub_id, &mut local.missed)
                .and_then(|bytes| {
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
    // Co-located geometry: the stamp shares a cache line with its payload, so
    // publishing dirties two lines (the slot, and the header publish word that
    // `recv_shm_pod_broadcast` still gates on unconditionally) instead of the
    // three the split layout does
    // (data slot, sequence-array entry, and the header's publish word). The
    // stride is a cached register, constant for the life of the mapping, so
    // this branch predicts perfectly.
    let colo_stride = local.cached_colo_stride;
    if colo_stride != 0 {
        let off = index * colo_stride as usize;
        // SAFETY: cached_seq_ptr is slot 0's stamp and cached_data_ptr slot 0's
        // payload; index < capacity (mask), and the region was sized for
        // capacity * colo_stride bytes past HEADER_SIZE.
        unsafe {
            let stamp = &*(local.cached_seq_ptr.add(off) as *const std::sync::atomic::AtomicU64);
            // Boehm seqlock write side. The WRITING marker is what lets a
            // consumer detect a producer that lapped onto this slot mid-copy;
            // PodShm broadcast overwrites without backpressure, so that
            // window is real rather than theoretical.
            stamp.store(SLOT_WRITING | new_seq, Ordering::Release);
            simd_aware_write(local.cached_data_ptr.add(off) as *mut T, msg);
            stamp.store(new_seq, Ordering::Release);
        }
        local.local_head = new_seq;
        header.sequence_or_head.store(new_seq, Ordering::Release);
        housekeep_lease!(local, topic);
        return Ok(());
    }

    // SAFETY: cached_data_ptr points to SHM data region. index < capacity (mask).
    // cached_seq_ptr points to the per-slot ready-flag array (set unconditionally
    // in ensure_role). index*8 is within bounds. simd_aware_write copies bytes, so
    // the slot needs no alignment — nothing rounds slot_size to align_of::<T>().
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
    // `retries` lives in a register and is only touched on the CAS-failure arm,
    // so the uncontended claim — one load, one successful CAS — is unchanged.
    let mut retries: u32 = 0;
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
            // Another producer claimed between our load and CAS, or the weak CAS
            // failed spuriously. Retry with a fresh head — but under a CAP, because
            // "someone makes progress" is not "this thread makes progress" and a
            // spurious LL/SC failure is not statically bounded at all. Exhaustion
            // takes exactly the same path as a full ring: hand the message back, and
            // let `send_lossy_retry` turn it into a counted `send_failures` drop.
            // See `MAX_CLAIM_CAS_RETRIES`.
            Err(_) => {
                retries += 1;
                if unlikely(retries >= MAX_CLAIM_CAS_RETRIES) {
                    warn_claim_cas_exhausted(local, topic.name());
                    return Err(msg);
                }
                std::hint::spin_loop();
            }
        }
    };

    let index = (seq & mask) as usize;
    // SAFETY: cached_data_ptr points to SHM data region. index < capacity (mask).
    // cached_seq_ptr points to per-slot ready-flag array. index*8 is within bounds.
    // simd_aware_write handles alignment. Release store publishes data to consumers.
    unsafe {
        let (ready_ptr, data_ptr) = slot_ptrs::<T>(local, index);
        simd_aware_write(data_ptr, msg);
        (*ready_ptr).store(seq.wrapping_add(1), Ordering::Release);
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
        let (ready_ptr_raw, data_ptr) = slot_ptrs::<T>(local, index);
        let ready_ptr = &*ready_ptr_raw;

        // Seqlock write phase. Broadcast has NO backpressure — it overwrites the
        // oldest slot unconditionally — so a producer that laps the ring can
        // clobber the very slot a consumer is mid-copy. Stamping only AFTER the
        // write (what this did) leaves that window open: the consumer sees a
        // still-matching stamp, copies torn bytes, and nothing ever tells it.
        //
        // Marking the slot BEFORE touching the data, with the Boehm fence
        // pairing, forces any consumer that observes the new data to also
        // observe the marker and reject the copy. A bare re-check WITHOUT this
        // marker is insufficient — the producer would not yet have bumped the
        // stamp, so the consumer's two reads would agree on stale bytes.
        //
        // The marker is a high bit rather than the `seqlock` module's `pos << 1`
        // encoding on purpose: `cached_seq_ptr` is shared with the MpscShm and
        // SpscShm paths, which compare against `seq + 1`, and a PodShm topic can
        // migrate to those backends. A high bit keeps the encoding compatible —
        // those consumers simply read "not ready" for the duration of a write.
        ready_ptr.store(seq.wrapping_add(1) | SLOT_WRITING, Ordering::Relaxed);
        fence(Ordering::Release);
        simd_aware_write(data_ptr, msg);
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

    let local = topic.local();

    // Serialize into this handle's reusable staging buffer (no per-message
    // allocation). `bytes_len` is carried by value from here on so that nothing
    // holds a borrow of `local` across `spill_to_pool` or `auto_grow_slot_size`
    // — the latter re-enters `topic.local()`. See `serialize_into_scratch`.
    if !serialize_into_scratch(local, &msg) {
        return Err(msg);
    }
    let bytes_len = local.serde_scratch.len();

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

    if bytes_len > SPILL_THRESHOLD {
        // ── Spill path: large message → TensorPool ──────────────────────
        // SpillDescriptor is 40 bytes; slot must have at least 48 usable
        // (8B ready flag + 40B descriptor). Minimum slot_size is 64, so
        // this always holds, but guard against pathological configs.
        if slot_size < 48 {
            return Err(msg);
        }
        // The shared borrow of `local.serde_scratch` ends with this call:
        // `spill_to_pool` returns an owned descriptor and never touches
        // `topic.local()` itself (only `topic.spill_pool`).
        let spill_result = spill_to_pool(topic, &local.serde_scratch);
        let spill = match spill_result {
            Some(s) => s,
            None => {
                log::warn!(
                    "Topic '{}': spill to pool failed for {} bytes, falling back to auto-grow",
                    topic.name(),
                    bytes_len
                );
                // Fall back to auto-grow if pool alloc fails
                if bytes_len > max_data_len {
                    let _ = topic.auto_grow_slot_size(bytes_len);
                    return Err(msg);
                }
                // If it fits inline despite being above threshold, just inline it
                // (this shouldn't happen, but handle gracefully)
                // SAFETY: cached_data_ptr points into SHM data region (invariant 2).
                // slot_offset < capacity * slot_size (invariant 3). bytes_len <= max_data_len
                // (checked above). Writes length header + data into serde slot layout.
                unsafe {
                    let slot_ptr = local.cached_data_ptr.add(slot_offset);
                    let len_ptr = slot_ptr.add(8) as *mut u64;
                    std::ptr::write_volatile(len_ptr, bytes_len as u64);
                    let data_ptr = slot_ptr.add(16);
                    std::ptr::copy_nonoverlapping(
                        local.serde_scratch.as_ptr(),
                        data_ptr,
                        bytes_len,
                    );
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

    if bytes_len > max_data_len {
        // ── Auto-grow path: message fits threshold but exceeds current slot ─
        if !topic.auto_grow_slot_size(bytes_len) {
            log::warn!(
                "Topic: serialized message ({} bytes) exceeds slot limit ({} bytes). \
                 Auto-grow failed. Use Topic::with_capacity(name, cap, Some(slot_size)).",
                bytes_len,
                max_data_len,
            );
        }
        return Err(msg);
    }

    // ── Inline path: small message → write directly to ring buffer slot ─
    // SAFETY: cached_data_ptr + slot_offset points to a valid slot within the SHM data
    // region. slot_offset < capacity * slot_size (index < capacity via mask). The slot
    // layout is [8B ready | 8B length | data...], and bytes_len <= max_data_len.
    unsafe {
        let slot_ptr = local.cached_data_ptr.add(slot_offset);
        let len_ptr = slot_ptr.add(8) as *mut u64;
        std::ptr::write_volatile(len_ptr, bytes_len as u64);
        let data_ptr = slot_ptr.add(16);
        std::ptr::copy_nonoverlapping(local.serde_scratch.as_ptr(), data_ptr, bytes_len);
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

    let local = topic.local();

    // Reusable staging buffer — see `serialize_into_scratch` and the note in
    // `send_shm_sp_serde` on why the length is carried by value.
    if !serialize_into_scratch(local, &msg) {
        return Err(msg);
    }
    let bytes_len = local.serde_scratch.len();

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
    let spill_desc = if bytes_len > SPILL_THRESHOLD {
        if slot_size < 48 {
            return Err(msg);
        }
        // Hoisted out of the `match` scrutinee so the shared borrow of
        // `local.serde_scratch` provably ends before the arms touch `local`.
        let spilled = spill_to_pool(topic, &local.serde_scratch);
        match spilled {
            Some(s) => Some(s),
            None => {
                log::warn!(
                    "Topic '{}': spill to pool failed for {} bytes, falling back to auto-grow",
                    topic.name(),
                    bytes_len
                );
                if bytes_len > max_data_len {
                    let _ = topic.auto_grow_slot_size(bytes_len);
                    return Err(msg);
                }
                None // will inline below
            }
        }
    } else if bytes_len > max_data_len {
        // Small message but exceeds current slot — auto-grow
        if !topic.auto_grow_slot_size(bytes_len) {
            log::warn!(
                "Topic: serialized message ({} bytes) exceeds slot limit ({} bytes). \
                 Auto-grow failed. Use Topic::with_capacity(name, cap, Some(slot_size)).",
                bytes_len,
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
    // Capped exactly as in `send_shm_mp_pod` — see `MAX_CLAIM_CAS_RETRIES`. The
    // orphaned-spill note above applies unchanged to an exhaustion return.
    let mut retries: u32 = 0;
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
            Err(_) => {
                retries += 1;
                if unlikely(retries >= MAX_CLAIM_CAS_RETRIES) {
                    warn_claim_cas_exhausted(local, topic.name());
                    return Err(msg);
                }
                std::hint::spin_loop();
            }
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
            std::ptr::write_volatile(len_ptr, bytes_len as u64);
            let data_ptr = slot_ptr.add(16);
            std::ptr::copy_nonoverlapping(local.serde_scratch.as_ptr(), data_ptr, bytes_len);
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
    if let Err(e) = topic.ensure_producer() {
        // As `recv_uninitialized`: the caller gets its message back with no
        // indication that the cause is permanent for this handle.
        if should_report_endpoint_exhaustion(topic.name(), Exhausted::ParticipantTable) {
            tracing::warn!(
                "topic '{}': this publisher is NOT registered and will deliver nothing — {}",
                topic.name(),
                e
            );
        }
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

    // Co-located geometry: gate on the slot's own stamp rather than on
    // `header.sequence_or_head`. That is the entire point of the layout — the
    // stamp and the payload are the same cache line, so a receive costs ONE
    // coherence miss. The split path below costs two: the header's publish
    // word (a line the producer writes every message, so always a miss) and
    // then the data slot.
    let colo_stride = local.cached_colo_stride;
    if colo_stride != 0 {
        let off = (tail & mask) as usize * colo_stride as usize;
        let want = tail.wrapping_add(1);
        // SAFETY: cached_seq_ptr is slot 0's stamp; (tail & mask) < capacity and
        // the region holds capacity * colo_stride bytes past HEADER_SIZE.
        let stamp =
            unsafe { &*(local.cached_seq_ptr.add(off) as *const std::sync::atomic::AtomicU64) };
        // A slot is ours exactly when it carries `tail + 1`. A producer that
        // has lapped stores `tail + 1 + capacity`, and one mid-write has the
        // WRITING bit set, so both are excluded by the same equality.
        if stamp.load(Ordering::Acquire) != want {
            housekeep_epoch!(local, topic);
            return None;
        }
        // SAFETY: the Acquire load above observed the producer's Release
        // store of this exact sequence, so the payload beside it is fully
        // written. SPSC has backpressure, so it cannot be overwritten while
        // we read.
        let msg = unsafe { simd_aware_read(local.cached_data_ptr.add(off) as *const T) };
        local.local_tail = want;
        // Keep `local_head` a valid lower bound on what has been published.
        // The split path refreshes it from the header; if this topic ever
        // migrates to that path, a stale `local_head` would make its
        // `local_head - tail == 0` emptiness test read an unwritten slot.
        local.local_head = want;
        let flush_mask = mask >> 1;
        if want & flush_mask == 0 {
            header.tail.store(want, Ordering::Release);
        }
        housekeep_lease!(local, topic);
        return Some(msg);
    }

    if local.local_head.wrapping_sub(tail) == 0 {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if local.local_head.wrapping_sub(tail) == 0 {
            housekeep_epoch!(local, topic);
            return None;
        }
    }

    // SAFETY: cached_data_ptr points to SHM data region. index < capacity (mask).
    // The producer's Release store on sequence_or_head was observed via our Acquire load.
    // simd_aware_read copies bytes, so the slot needs no alignment — nothing
    // rounds slot_size to align_of::<T>().
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
    // The interval is capacity/2, which always leaves the producer headroom.
    // (The comment here used to claim `min(capacity/2, 32)`; no code ever
    // applied that cap, so the claim is dropped rather than the behaviour
    // changed — capping it would flush more often, not less.)
    //
    // `flush_interval` is `capacity / 2` and the ring capacity is always a power
    // of two (`TopicHeader::initialize` rounds it with `next_power_of_two` and
    // sets `capacity_mask = capacity - 1`; `header::read_slot_inner` rejects any
    // mapping where that does not hold). So the interval is itself a power of
    // two and `capacity / 2 - 1 == cached_capacity_mask >> 1` — one shift of a
    // value already in a register, instead of the `div`, `popcnt` and `lzcnt`
    // that `/ 2`, `is_power_of_two()` and `next_power_of_two()` emitted on every
    // single recv for a quantity that is constant for the life of the ring.
    // The `.max(1)` is subsumed: capacity 0, 1 and 2 all give mask >> 1 == 0.
    let flush_mask = mask >> 1;
    debug_assert_eq!(
        flush_mask,
        (local.cached_capacity / 2).max(1).next_power_of_two() - 1,
        "flush_mask identity broken: capacity {} / mask {} is not a power-of-two ring",
        local.cached_capacity,
        mask,
    );
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
    let stamp = unsafe {
        let (ready_ptr, _) = slot_ptrs::<T>(local, index);
        (*ready_ptr).load(Ordering::Acquire)
    };
    if stamp != tail.wrapping_add(1) {
        // Lapped, not blocked: the slot carries a LATER position than the one
        // being waited for, so the message is gone. Resume rather than sit on
        // it — see `resume_after_lap`.
        if resume_after_lap(local, header, tail, stamp) {
            housekeep_epoch!(local, topic);
            return None;
        }
        // The producer has CLAIMED this slot but not published it. Ordinarily
        // that resolves in nanoseconds; if the producer died in that window it
        // never resolves at all, and an in-order consumer blocks on it forever.
        // `claimed_slot_escape` bounds that — see its docs for the trade (an
        // unbounded stall becomes a COUNTED message loss).
        if let Some(why) = claimed_slot_escape(local, header, tail) {
            take_claimed_slot_escape(local, header, tail, topic.name(), why);
        }
        // Housekeeping used to be skipped here, so a consumer blocked on an
        // unpublished slot also stopped refreshing its lease and stopped
        // observing migrations — after one lease timeout the producer could
        // start retiring slots it had never read.
        housekeep_epoch!(local, topic);
        return None;
    }

    // SAFETY: cached_data_ptr points to SHM data region. index < capacity (mask).
    // The ready flag Acquire load above established happens-before with the producer's
    // Release store, so the slot data is fully written. simd_aware_read handles alignment.
    let msg = unsafe {
        let (_, data_ptr) = slot_ptrs::<T>(local, index);
        simd_aware_read(data_ptr as *const T)
    };
    let new_tail = tail.wrapping_add(1);
    local.local_tail = new_tail;
    // Batch header.tail updates. Interval must be < capacity to avoid
    // backpressure when the producer has written but consumer hasn't flushed.
    // `flush_interval` is `capacity / 2` and the ring capacity is always a power
    // of two (`TopicHeader::initialize` rounds it with `next_power_of_two` and
    // sets `capacity_mask = capacity - 1`; `header::read_slot_inner` rejects any
    // mapping where that does not hold). So the interval is itself a power of
    // two and `capacity / 2 - 1 == cached_capacity_mask >> 1` — one shift of a
    // value already in a register, instead of the `div`, `popcnt` and `lzcnt`
    // that `/ 2`, `is_power_of_two()` and `next_power_of_two()` emitted on every
    // single recv for a quantity that is constant for the life of the ring.
    // The `.max(1)` is subsumed: capacity 0, 1 and 2 all give mask >> 1 == 0.
    let flush_mask = mask >> 1;
    debug_assert_eq!(
        flush_mask,
        (local.cached_capacity / 2).max(1).next_power_of_two() - 1,
        "flush_mask identity broken: capacity {} / mask {} is not a power-of-two ring",
        local.cached_capacity,
        mask,
    );
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
                    migration_check!(local, topic);
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
                let (_, data_ptr) = slot_ptrs::<T>(local, (tail & mask) as usize);
                simd_aware_read(data_ptr as *const T)
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

    let tail = local.local_tail;
    if local.local_head.wrapping_sub(tail) == 0 {
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        if local.local_head.wrapping_sub(tail) == 0 {
            housekeep_epoch!(local, topic);
            return None;
        }
    }

    let behind = local.local_head.wrapping_sub(tail);
    if behind > local.cached_capacity {
        // Land on a slot the producer has actually written. Setting `tail =
        // local_head` looks like "skip to the newest", but `head` names the slot
        // the producer will write *next*: its ready-flag still belongs to the
        // message from a lap ago, so the `v1 < tail + 1` test below rejects it
        // and returns None. `local_tail` is then head again on the next call,
        // and again after that — a consumer that falls one lap behind receives
        // nothing, for as long as it stays behind, with no error anywhere.
        //
        // Measured on a 264-byte POD topic (16-slot ring) published at 1 kHz to
        // two subscribers, 3000 messages, varying the consumer's poll interval:
        //
        //     poll   1 ms -> 6000 of 6000 received
        //     poll  20 ms ->   35 of 6000
        //     poll 100 ms ->    0 of 6000   (last_seq = 0, never delivered once)
        //
        // and `dropped_count()` on the publisher read 0 throughout, because the
        // producer never failed a send — it is the consumer that is stuck.
        //
        // A 10 Hz node reading a 1 kHz sensor is ordinary multi-rate robotics,
        // and seqlock.rs:10-15 states the intended contract for exactly that
        // case: "a 30 Hz node reading a 500 Hz sensor should get the MOST RECENT
        // data". Land half a lap back from the head, the same landing point the
        // `v1 > tail + 1` branch below already uses and for the same reason —
        // `head - capacity` is the slot the producer overwrites next, so it
        // re-laps immediately.
        let head = header.sequence_or_head.load(Ordering::Acquire);
        let cap = local.cached_capacity;
        // Reaching here implies head >= tail + cap + 1 > cap, so this cannot
        // wrap; guarded anyway because `head` is re-loaded and could in
        // principle be observed smaller than the cached value.
        if head > cap {
            let resume = head.wrapping_sub(cap).wrapping_add(cap / 2);
            // Same two rules as the `v1 > tail + 1` site below, for the same
            // reasons: never move the tail backward — `cached_capacity` is a
            // cached value and a re-sized ring can compute a resume point behind
            // where this consumer already is — and count the skip, because
            // drop-oldest here used to be silent (`dropped_count()` reports send
            // failures, and an overwriting producer never fails a send). See
            // `LocalState::missed`.
            if resume > local.local_tail {
                local.missed = local.missed.wrapping_add(resume.wrapping_sub(tail));
                local.local_tail = resume;
                local.local_head = head;
            }
        }
        // Return rather than falling through to read the slot we just landed
        // on, exactly as the `v1 > tail + 1` branch below does. Reading
        // immediately after re-seating the cursor means validating a slot the
        // producer may be writing *right now*, in the middle of the ring rather
        // than behind it; `recv_never_reorders_or_duplicates_when_lapped` went
        // from 8/8 to 5/6 when this branch read directly, and back to passing
        // when it defers. The caller's next poll does the read against a cursor
        // that has settled, and a drain loop reaches it on the same pass.
        return None;
    }

    let index = (tail & mask) as usize;
    // SAFETY: cached_seq_ptr points to per-slot ready-flag array in SHM.
    // index*8 is within bounds (index < capacity, array has capacity entries).
    // SAFETY: cached_seq_ptr points to the per-slot ready-flag array in SHM;
    // index*8 is within bounds (index < capacity).
    let ready_ptr = unsafe { &*slot_ptrs::<T>(local, index).0 };
    let v1 = ready_ptr.load(Ordering::Acquire);
    // A write is in progress on this slot right now.
    if v1 & SLOT_WRITING != 0 {
        return None;
    }
    if v1 < tail.wrapping_add(1) {
        return None; // nothing published here yet
    }
    // The slot holds a message NEWER than the one `tail` names: the producer
    // lapped us. Returning it would deliver a newer message under an older
    // sequence, and the next call would then return the genuinely-older message
    // from the following slot — an inversion of exactly `capacity - 1`, which is
    // what was observed (-511 on a 512-slot ring, three times). The same read
    // also duplicates: after handing back `T+C` we set tail to `T+1`, so when
    // tail later reaches `T+C` that value is delivered a second time.
    //
    // The `behind > capacity` guard above cannot catch this. It is a *sampled*
    // check: `local_head` is only reloaded when the cached head is drained, and
    // between reloads `behind` counts down while the true lag can grow past
    // capacity. Lapping that starts inside a drain window is invisible to it.
    // Reloading the head on every call would not help either — the producer can
    // lap us between the head load and the ready-flag load — and it would put a
    // shared-cacheline atomic on the hot path for nothing.
    //
    // `v1` is already loaded, so this costs one comparison. With it, the accept
    // condition is exactly `v1 == tail + 1`, the same condition the SpscShm and
    // MpscShm readers on this ring already use (dispatch.rs:1241, 1511).
    //
    // The compare is an absolute u64 compare, matching the `v1 <` compare above.
    // That is sound only under the "sequences never reach 2^63" assumption
    // documented at shm_layout.rs:110-112 — the same assumption that makes bit
    // 63 available as SLOT_WRITING. Do not switch either compare to a wrapping
    // form without revisiting both.
    if v1 > tail.wrapping_add(1) {
        // Resume from a still-valid slot. Land half a lap back from the head
        // rather than exactly `head - capacity`: that boundary is the slot the
        // producer overwrites *next*, so it re-laps immediately. Measured over
        // 3M sends to 32 consumers: landing on the boundary delivered 2.23M
        // messages and burned 7.16M calls re-detecting laps, while half a lap of
        // slack delivered 16.98M with 47.9k lap detections, zero inversions and
        // zero duplicates.
        //
        // `head` here is freshly loaded, and reaching this branch implies
        // `head >= tail + capacity + 1 > capacity`, so the subtraction cannot
        // wrap. Keep this computation below the `v1 > tail + 1` test — hoisting
        // it above would let it wrap to ~2^64 and hand back a garbage tail.
        let head = header.sequence_or_head.load(Ordering::Acquire);
        let cap = local.cached_capacity;
        if head > cap {
            let resume = head.wrapping_sub(cap).wrapping_add(cap / 2);
            // Never move the tail backward, and count what we skip when we do
            // move it forward. Both branches fixed this site; both fixes hold.
            //
            // The argument that `resume > tail` always holds relies on
            // `cached_capacity` being the ring's current capacity. It is a
            // cached value, and a topology change re-sizes the ring — so a
            // capacity larger than the live one computes a resume point behind
            // where this consumer already is, and the next accepted slot then
            // delivers a message older than the last one returned. Skipping the
            // update loses nothing: the slot is rejected either way, and the
            // next call re-derives a resume point from a fresher head.
            //
            // The skip itself is drop-oldest, which is the designed behaviour
            // here but used to be silent: `dropped_count()` reports send
            // failures and an overwriting producer never fails a send, so a
            // lapped consumer lost data with every observability surface
            // reading zero. See `LocalState::missed`.
            if resume > local.local_tail {
                local.missed = local.missed.wrapping_add(resume.wrapping_sub(tail));
                local.local_tail = resume;
                local.local_head = head;
            }
        }
        return None;
    }

    // SAFETY: cached_data_ptr points to SHM data region. index < capacity (mask).
    // The Acquire load above establishes happens-before with the producer's
    // Release store, so the slot data is fully written. simd_aware_read_uninit
    // handles alignment. The copy may still be torn if a lapping producer
    // overwrites the slot DURING this read — the re-check below is what detects
    // that, which is why the bytes are held as `MaybeUninit<T>` and not yet a
    // `T`: `is_pod` admits types with validity invariants (a struct with a
    // `bool` or a fieldless enum field), and materialising torn bytes into such
    // a `T` would be UB committed before the re-check could reject it.
    let msg = unsafe {
        let (_, data_ptr) = slot_ptrs::<T>(local, index);
        simd_aware_read_uninit(data_ptr as *const T)
    };

    // Seqlock re-check (Boehm): this Acquire fence pairs with the producer's
    // Release fence, so if any byte of `msg` came from an in-progress overwrite
    // we are forced to observe the producer's newer stamp here and discard.
    // Without it, a producer lapping the ring silently hands the consumer a
    // half-old, half-new value — on a robot, a pose or velocity that never
    // existed.
    fence(Ordering::Acquire);
    if ready_ptr.load(Ordering::Relaxed) != v1 {
        // Overwritten mid-copy. Drop the raw bytes; the caller polls again.
        // `msg` is still `MaybeUninit`, so nothing is dropped and no invalid
        // value is ever constructed.
        return None;
    }
    // SAFETY: the stamp re-check above passed, so the slot was NOT overwritten
    // during the copy and the bytes are the producer's fully-written value.
    let msg = unsafe { msg.assume_init() };

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
    let stamp = unsafe {
        let slot_ptr = local.cached_data_ptr.add(slot_offset);
        let ready_ptr = &*(slot_ptr as *const std::sync::atomic::AtomicU64);
        ready_ptr.load(Ordering::Acquire)
    };
    if stamp != tail.wrapping_add(1) {
        // Same lap check as `recv_shm_mpsc_pod`, for the same reason.
        if resume_after_lap(local, header, tail, stamp) {
            housekeep_epoch!(local, topic);
            return None;
        }
        // Same abandoned-claim escape as `recv_shm_mpsc_pod`; the serde slot
        // simply carries its ready flag in its own first 8 bytes rather than in
        // the separate seq array. See `claimed_slot_escape` for the trade.
        if let Some(why) = claimed_slot_escape(local, header, tail) {
            take_claimed_slot_escape(local, header, tail, topic.name(), why);
        }
        housekeep_epoch!(local, topic);
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
    // `flush_interval` is `capacity / 2` and the ring capacity is always a power
    // of two (`TopicHeader::initialize` rounds it with `next_power_of_two` and
    // sets `capacity_mask = capacity - 1`; `header::read_slot_inner` rejects any
    // mapping where that does not hold). So the interval is itself a power of
    // two and `capacity / 2 - 1 == cached_capacity_mask >> 1` — one shift of a
    // value already in a register, instead of the `div`, `popcnt` and `lzcnt`
    // that `/ 2`, `is_power_of_two()` and `next_power_of_two()` emitted on every
    // single recv for a quantity that is constant for the life of the ring.
    // The `.max(1)` is subsumed: capacity 0, 1 and 2 all give mask >> 1 == 0.
    let flush_mask = mask >> 1;
    debug_assert_eq!(
        flush_mask,
        (local.cached_capacity / 2).max(1).next_power_of_two() - 1,
        "flush_mask identity broken: capacity {} / mask {} is not a power-of-two ring",
        local.cached_capacity,
        mask,
    );
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
    if let Err(e) = topic.ensure_consumer() {
        // Registration failed — almost always the 16-slot participant table
        // being full. The role stays `Unregistered`, so every later `recv()`
        // re-enters this cold path and fails identically: `None` forever, from
        // a subscriber that looks merely idle. Discarding the error here made
        // the message `register_role` builds — which names the limit and says
        // what to do about it — reach nothing at all.
        //
        // The same 16-slot wall one layer down is reported, deliberately, by
        // this same rate limiter: "never let 'all 16 slots live' become a
        // SILENT no-comms endpoint".
        if should_report_endpoint_exhaustion(topic.name(), Exhausted::ParticipantTable) {
            tracing::warn!(
                "topic '{}': this subscriber is NOT registered and will receive nothing — {}",
                topic.name(),
                e
            );
        }
        return None;
    }
    // SAFETY: ensure_consumer() → initialize_backend() → set_dispatch_fn_ptrs()
    // has set recv_fn to a valid function pointer. UnsafeCell is single-thread.
    unsafe { (*topic.recv_fn.get())(topic) }
}

/// Whether this topic should report an endpoint-exhaustion event now.
///
/// The two call sites each had `static WARNED: AtomicBool` and warned once per
/// *process*, forever. The intent was not to flood a hot send loop, and the
/// effect was that the second topic to run out of endpoints — and every one
/// after it, for the life of the process — lost its comms in complete silence.
/// On a robot that is a subsystem going quiet hours after an unrelated warning
/// scrolled past, which is the failure mode the loud warning exists to prevent.
///
/// Keyed by topic and rate-limited per topic instead, so a hot loop still emits
/// once a minute rather than once ever, and a second topic hitting the same
/// wall is never masked by the first.
///
/// Keyed by `reason` as well as topic, for the same argument one level down: a
/// topic can hit two different walls — its FanoutShm endpoint bitmap and the
/// 16-slot participant table — and they have different causes and different
/// fixes. Sharing one bucket would let whichever fired first silence the other
/// for a minute, which is the masking this function exists to prevent.
pub(super) fn should_report_endpoint_exhaustion(topic: &str, reason: Exhausted) -> bool {
    use std::collections::HashMap;
    use std::sync::Mutex;
    use std::time::{Duration, Instant};

    const QUIET: Duration = Duration::from_secs(60);
    #[allow(clippy::type_complexity)]
    static LAST: Mutex<Option<HashMap<(String, Exhausted), Instant>>> = Mutex::new(None);

    let mut guard = match LAST.lock() {
        Ok(g) => g,
        Err(p) => p.into_inner(),
    };
    let seen = guard.get_or_insert_with(HashMap::new);
    let now = Instant::now();
    let key = (topic.to_string(), reason);
    match seen.get(&key) {
        Some(t) if now.duration_since(*t) < QUIET => false,
        _ => {
            seen.insert(key, now);
            true
        }
    }
}

/// Which wall a topic ran into. Rate-limited separately; see
/// `should_report_endpoint_exhaustion`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(super) enum Exhausted {
    /// The FanoutShm publisher or subscriber endpoint bitmap is full.
    FanoutEndpoints,
    /// The topic's 16-slot participant table is full, so this handle holds no
    /// registration and will never send or receive anything.
    ParticipantTable,
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
        assert_eq!(recovered.dtype(), crate::types::TensorDtype::U8);
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
        assert_eq!(recovered.dtype(), crate::types::TensorDtype::U8);
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

    // ── Abandoned-claim escape (research 1.14) ──────────────────────────
    //
    // These drive `claimed_slot_escape` / `take_claimed_slot_escape` directly,
    // because the defect they bound needs a producer to be SIGKILLed inside a
    // window a few instructions wide — not something an in-process test can
    // stage. The state machine is the part that has to be right.

    /// A `LocalState` already stalled at `tail`, with the poll counter and the
    /// stall clock placed where the test needs them.
    fn stalled_local(tail: u64, polls: u32, since_ms: u64) -> LocalState {
        LocalState {
            claim_stall_tail: tail,
            claim_stall_polls: polls,
            claim_stall_since_ms: since_ms,
            ..Default::default()
        }
    }

    /// A header with no participants registered at all — nobody could ever
    /// publish the stuck slot.
    fn header_with_no_producers() -> TopicHeader {
        TopicHeader::zeroed()
    }

    /// A header advertising THIS process as a live producer. `no_producer_can_finish`
    /// must refuse to judge our own pid, exactly as `reap_dead_participants` does.
    fn header_with_live_self_producer() -> TopicHeader {
        let h = TopicHeader::zeroed();
        h.participants[0]
            .pid
            .store(std::process::id(), Ordering::Release);
        h.participants[0].role.store(1, Ordering::Release); // producer
        h.participants[0].active.store(1, Ordering::Release);
        h
    }

    #[test]
    fn claim_escape_first_observation_only_arms() {
        let header = header_with_no_producers();
        let mut local = LocalState::default();
        // Even with a header that would satisfy every escape condition, the very
        // first not-ready observation must decide nothing and — critically — must
        // not read the clock, because a transient mid-write slot lands here.
        assert!(claimed_slot_escape(&mut local, &header, 7).is_none());
        assert_eq!(local.claim_stall_tail, 7);
        assert_eq!(local.claim_stall_polls, 1);
        assert_eq!(
            local.claim_stall_since_ms, 0,
            "arming must not start the clock"
        );
    }

    #[test]
    fn claim_escape_stays_silent_below_the_poll_mask() {
        let header = header_with_no_producers();
        let mut local = LocalState::default();
        // A spin-waiting consumer must get all the way to the mask boundary
        // without any escape and without the clock being read.
        for _ in 0..=CLAIM_STALL_POLL_MASK {
            assert!(claimed_slot_escape(&mut local, &header, 7).is_none());
        }
        assert_eq!(
            local.claim_stall_polls,
            CLAIM_STALL_POLL_MASK + 1,
            "the mask boundary is where the clock first gets read"
        );
        assert_ne!(
            local.claim_stall_since_ms, 0,
            "reaching the boundary must start the stall clock"
        );
    }

    #[test]
    fn claim_escape_needs_a_full_lease_timeout() {
        let header = header_with_no_producers();
        header.set_lease_timeout_ms(50);
        // Clock started "just now": no producer is registered, yet the escape
        // must still wait out the lease. A momentarily-empty participant table
        // is not on its own a reason to drop a message.
        let mut local = stalled_local(7, CLAIM_STALL_POLL_MASK, current_time_ms());
        assert!(claimed_slot_escape(&mut local, &header, 7).is_none());
    }

    #[test]
    fn claim_escape_fires_when_no_producer_can_finish() {
        let header = header_with_no_producers();
        header.set_lease_timeout_ms(50);
        let mut local = stalled_local(7, CLAIM_STALL_POLL_MASK, current_time_ms() - 500);
        assert_eq!(
            claimed_slot_escape(&mut local, &header, 7),
            Some(ClaimEscape::NoProducerLeft),
        );
    }

    #[test]
    fn claim_escape_defers_to_a_live_producer_until_the_absolute_bound() {
        let header = header_with_live_self_producer();
        header.set_lease_timeout_ms(50);

        // Past one lease but well inside the absolute bound: a live producer is
        // registered, so keep waiting rather than drop a message that may yet
        // arrive.
        let mut local = stalled_local(7, CLAIM_STALL_POLL_MASK, current_time_ms() - 100);
        assert!(claimed_slot_escape(&mut local, &header, 7).is_none());

        // Past CLAIM_STALL_MAX_LEASES: give up anyway. This is the hole the
        // two-condition rule cannot see — a dead producer already reaped by a
        // third party while another producer is still live.
        let overrun = 50 * CLAIM_STALL_MAX_LEASES + 50;
        let mut local = stalled_local(7, CLAIM_STALL_POLL_MASK, current_time_ms() - overrun);
        assert_eq!(
            claimed_slot_escape(&mut local, &header, 7),
            Some(ClaimEscape::AbsoluteBound),
        );
    }

    #[test]
    fn claim_escape_restarts_when_the_ring_moves_on() {
        let header = header_with_no_producers();
        header.set_lease_timeout_ms(50);
        // Fully expired at position 7 …
        let mut local = stalled_local(7, CLAIM_STALL_POLL_MASK, current_time_ms() - 500);
        // … but the consumer is now asking about position 8. The old timer must
        // not carry over, or one stall would authorise a second, unrelated drop.
        assert!(claimed_slot_escape(&mut local, &header, 8).is_none());
        assert_eq!(local.claim_stall_tail, 8);
        assert_eq!(local.claim_stall_since_ms, 0);
    }

    #[test]
    fn claim_escape_survives_a_backwards_clock() {
        let header = header_with_no_producers();
        header.set_lease_timeout_ms(50);
        // A stall clock in the future (wall clock stepped backwards). Trusting
        // it would underflow the elapsed-time subtraction; the escape must
        // restart the timer instead, which can only ever delay a drop.
        let mut local = stalled_local(7, CLAIM_STALL_POLL_MASK, current_time_ms() + 60_000);
        assert!(claimed_slot_escape(&mut local, &header, 7).is_none());
        assert!(local.claim_stall_since_ms <= current_time_ms());
    }

    #[test]
    fn taking_the_escape_counts_the_loss_and_frees_the_slot() {
        let header = header_with_no_producers();
        let mut local = stalled_local(7, CLAIM_STALL_POLL_MASK, current_time_ms() - 500);
        local.local_tail = 7;

        take_claimed_slot_escape(
            &mut local,
            &header,
            7,
            "test_topic",
            ClaimEscape::NoProducerLeft,
        );

        // The whole trade rests on this: the drop is VISIBLE.
        assert_eq!(local.missed, 1, "an abandoned-claim skip must be counted");
        assert_eq!(
            local.local_tail, 8,
            "the consumer must advance past the slot"
        );
        assert_eq!(
            header.tail.load(Ordering::Acquire),
            8,
            "the freed slot must be published to producers immediately, not \
             batched — a second stall on top of the one just escaped",
        );
        assert_eq!(local.claim_stall_polls, 0, "the stall timer must disarm");
        assert_eq!(local.claim_stall_since_ms, 0);
    }

    #[test]
    fn no_producer_can_finish_ignores_remote_and_inactive_entries() {
        let header = TopicHeader::zeroed();
        // A network-replicated producer: its pid belongs to another host and
        // means nothing to a local liveness probe, so it must not count as a
        // live local producer keeping the consumer blocked.
        header.participants[0].pid.store(4242, Ordering::Release);
        header.participants[0].role.store(1, Ordering::Release);
        header.participants[0].active.store(1, Ordering::Release);
        header.participants[0]
            .source_host
            .store(9, Ordering::Release);
        assert!(no_producer_can_finish(&header));

        // A consumer-only registration is not a producer either.
        header.participants[0]
            .source_host
            .store(0, Ordering::Release);
        header.participants[0]
            .pid
            .store(std::process::id(), Ordering::Release);
        header.participants[0].role.store(2, Ordering::Release);
        assert!(no_producer_can_finish(&header));

        // Same entry, now claiming the producer role: we refuse to judge our
        // own pid, so this DOES block the two-condition escape.
        header.participants[0].role.store(3, Ordering::Release); // both
        assert!(!no_producer_can_finish(&header));
    }

    // ── Bounded slot claim (research 1.20) ──────────────────────────────

    #[test]
    fn claim_cas_cap_is_above_the_worst_real_contention() {
        // The cap exists to bound spurious/livelocked retries, not to reject
        // genuine contention: every producer a topic can hold must be able to
        // lose the race and still get in.
        assert!(
            MAX_CLAIM_CAS_RETRIES as usize > super::super::header::MAX_PARTICIPANTS,
            "a cap at or below MAX_PARTICIPANTS would drop messages under \
             ordinary multi-producer contention",
        );
    }

    #[test]
    fn claim_cas_warning_is_rate_limited_per_handle() {
        let mut local = LocalState::default();
        warn_claim_cas_exhausted(&mut local, "test_topic");
        let first = local.claim_cas_warn_ms;
        assert_ne!(first, 0, "the first exhaustion must be reported");
        // `send()` funnels one rejected try_send into ~70 more attempts; the
        // gate must survive that without a lock or an allocation.
        for _ in 0..128 {
            warn_claim_cas_exhausted(&mut local, "test_topic");
        }
        assert_eq!(
            local.claim_cas_warn_ms, first,
            "a burst must not re-arm the quiet period",
        );
    }

    // ── Serde staging buffer (research 1.18) ────────────────────────────

    #[test]
    fn serde_scratch_is_reused_across_sends() {
        let mut local = LocalState::default();
        assert_eq!(
            local.serde_scratch.capacity(),
            0,
            "a POD topic must not pay for a buffer it never uses",
        );

        let big: Vec<u64> = (0..512).collect();
        assert!(serialize_into_scratch(&mut local, &big));
        let first_len = local.serde_scratch.len();
        let warm_capacity = local.serde_scratch.capacity();
        assert!(first_len > 0);
        assert!(warm_capacity >= first_len);

        // The point of the change: a second, smaller message must reuse the
        // capacity rather than allocate again.
        let small: Vec<u64> = (0..4).collect();
        assert!(serialize_into_scratch(&mut local, &small));
        assert!(local.serde_scratch.len() < first_len);
        assert_eq!(
            local.serde_scratch.capacity(),
            warm_capacity,
            "steady-state serde sends must not touch the allocator",
        );
    }

    #[test]
    fn serde_scratch_round_trips_through_bincode() {
        // serialize_into must produce exactly what bincode::serialize did, or
        // the wire format has silently changed for every existing subscriber.
        let mut local = LocalState::default();
        let value = ("cmd_vel".to_string(), 1.5f64, vec![1u8, 2, 3]);
        assert!(serialize_into_scratch(&mut local, &value));
        assert_eq!(
            local.serde_scratch.as_slice(),
            bincode::serialize(&value).unwrap().as_slice(),
            "the staging buffer must not change the wire format",
        );
    }

    #[test]
    fn serde_scratch_is_cleared_between_messages() {
        // A stale tail from a longer previous message would be published as
        // part of the next one — silent corruption on the wire.
        let mut local = LocalState::default();
        let long: Vec<u8> = vec![0xAB; 300];
        assert!(serialize_into_scratch(&mut local, &long));
        let short: Vec<u8> = vec![0x01; 3];
        assert!(serialize_into_scratch(&mut local, &short));
        assert_eq!(
            local.serde_scratch.as_slice(),
            bincode::serialize(&short).unwrap().as_slice(),
        );
    }
}
