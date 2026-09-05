//! Local state for a Topic participant.
//!
//! Caches frequently accessed values locally to avoid reading from shared
//! memory on the hot path.

use super::header::TopicHeader;
use super::types::{BackendMode, TopicRole};

/// Default serialized message slot size (8KB)
pub(crate) const DEFAULT_SLOT_SIZE: usize = 8 * 1024;

// `LEASE_REFRESH_INTERVAL` (1024 messages) lived here. Lease refresh is no
// longer driven by a message count at all: refreshing strictly every N messages
// made liveness a function of throughput, so a healthy but slow participant
// spent most of its life looking expired to the reaper. It is now gated on the
// wall clock in `RingTopic::refresh_lease_if_due`, which fires at half the
// lease timeout regardless of rate; `dispatch::LEASE_CHECK_INTERVAL` only keeps
// the clock read off the per-message hot path.

/// Epoch check interval — reads migration_epoch from SHM header every N messages.
/// Must be a power of two (used with bitmask).
///
/// Set to 32 (~1ns amortized) to detect cross-process migration promptly.
/// At 1 Hz this checks every ~4 s; at 30 Hz every ~133 ms; at 1 kHz every ~4 ms.
/// Lowered from 32 to 4 to fix cross-process discovery at low frequencies
/// (GitHub issue #37: 1Hz publisher took 32s to detect a new subscriber).
/// Cost: one Relaxed mmap load (~20ns) every 4 messages ≈ 5ns amortized.
pub(crate) const EPOCH_CHECK_INTERVAL: u32 = 4;

/// Local state for an Topic participant
///
/// ## Cache-Optimized Design
///
/// Fields are ordered to match the ACCESS PATTERN in send():
/// 1. First: cached_mode (branch decision)
/// 2. Then: local_head, cached_data_ptr, cached_capacity_mask (role=Both send hot path)
/// 3. Then: local_tail, cached_capacity, cached_header_ptr (ring backpressure + migration)
///
/// First cache line (0-63): ALL hot path fields in access order
/// Second cache line (64+): Cold path fields (registration, migration, etc.)
#[repr(C)] // Prevent compiler reordering - layout is critical for performance
pub(crate) struct LocalState {
    // ========== FIRST CACHE LINE (0-63 bytes) - HOT PATH ==========
    /// Cached backend mode - FIRST field because it's checked FIRST in send()
    pub cached_mode: BackendMode, // offset 0 (1 byte)

    /// Our role (accessed early in some paths)
    pub role: TopicRole, // offset 1 (1 byte)

    /// Is POD type (cached for performance)
    pub is_pod: bool, // offset 2 (1 byte)

    /// Cached is_same_process result
    pub is_same_process: bool, // offset 3 (1 byte)

    /// Message counter for sampling lease refresh (4 bytes, align to 4)
    pub msg_counter: u32, // offset 4

    /// Locally cached head index - CRITICAL: accessed immediately after mode check
    pub local_head: u64, // offset 8

    /// Cached capacity mask for bitwise AND (seq & mask)
    ///
    /// Invariant: `cached_capacity_mask == cached_capacity - 1` and
    /// `cached_capacity` is a power of two (or both are 0 before `sync_local`
    /// has run). `TopicHeader::initialize` rounds capacity with
    /// `next_power_of_two()` and derives `capacity_mask` from it, and
    /// `header::read_slot_inner` refuses to read any mapping where the pair
    /// disagrees. `sync_local` copies both out of the header together.
    ///
    /// The recv paths lean on this beyond index masking: the batched
    /// `header.tail` flush interval is `capacity / 2`, so its mask is exactly
    /// `cached_capacity_mask >> 1` — see the three `flush_mask` sites in
    /// `dispatch.rs`, which derive it with a shift instead of recomputing a
    /// divide plus `is_power_of_two`/`next_power_of_two` on every message.
    pub cached_capacity_mask: u64, // offset 16

    /// Cached pointer to data region - for ring buffer write
    /// SAFETY: Valid for Topic lifetime (points into `Arc<ShmRegion>`)
    pub cached_data_ptr: *mut u8, // offset 24

    /// Locally cached tail index (for backpressure check on the ring)
    pub local_tail: u64, // offset 32

    /// Cached capacity (for backpressure check)
    pub cached_capacity: u64, // offset 40

    /// Cached pointer to header - for atomic updates on the SHM header
    /// SAFETY: Valid for Topic lifetime (points into `Arc<ShmRegion>`)
    pub cached_header_ptr: *const TopicHeader, // offset 48

    /// Cached pointer to per-slot sequence array (for multi-producer ready flags)
    /// SAFETY: Valid for Topic lifetime (points into `Arc<ShmRegion>`)
    pub cached_seq_ptr: *mut u8, // offset 56

    /// Bytes between consecutive slots under the co-located layout, or 0 when
    /// this topic uses the historical split layout.
    ///
    /// Nonzero doubles as the "is colo" predicate. Cached rather than read from
    /// the header per message so the hot path branches on a register that is
    /// constant for the life of the mapping — a perfectly predicted branch,
    /// not a shared-line load.
    pub cached_colo_stride: u64,

    // ========== SECOND CACHE LINE (64+ bytes) - COLD PATH ==========
    /// Our slot index in the participant array (-1 if not registered)
    pub slot_index: i32,

    /// Generation of the participant entry at `slot_index` when this handle
    /// claimed it. Together they name a registration unambiguously, so `Drop`
    /// can give back its own and never a later one that reused the slot.
    pub participant_generation: u16,

    /// Slot size for serialized messages (non-POD)
    pub slot_size: usize,

    /// Cached epoch (to detect migrations)
    pub cached_epoch: u64,

    /// SHM FanoutRing publisher ID (registered on first send when mode is FanoutShm)
    pub fanout_shm_pub_id: Option<usize>,

    /// SHM FanoutRing subscriber ID (registered on first recv when mode is FanoutShm)
    pub fanout_shm_sub_id: Option<usize>,

    /// Messages this consumer skipped past without delivering them.
    ///
    /// Two causes, both counted here because both are the same thing to a
    /// subscriber — a gap in the stream:
    ///
    /// * **Lapped.** Drop-oldest is the designed behaviour under overload — a
    ///   10 Hz node reading a 1 kHz sensor is ordinary multi-rate robotics —
    ///   but the loss used to be completely invisible: `dropped_count()`
    ///   reports *send* failures, and a producer that overwrites never fails a
    ///   send.
    /// * **Abandoned claim.** A slot an MpscShm producer claimed and then died
    ///   before publishing. The consumer skips it rather than head-of-line
    ///   blocking on it forever (`dispatch::claimed_slot_escape`). That escape
    ///   is only defensible because the loss lands here: a silent drop would be
    ///   worse than the stall it replaces.
    ///
    /// This is the consumer's own count, so it needs no atomics.
    pub missed: u64,

    /// Ring position (`tail`) the consumer is currently stuck on because the
    /// slot is claimed-but-unpublished, and the wall clock when the stall was
    /// first timed. `claim_stall_since_ms == 0` means "no stall timed yet".
    ///
    /// Purely consumer-local bookkeeping for `dispatch::claimed_slot_escape`:
    /// no atomics, no shared memory, and — critically — never touched on the
    /// successful-recv path. Staleness is keyed on `tail`, which is monotonic,
    /// so a later stall at a different position resets the timer on its own and
    /// the hot path never has to clear these.
    pub claim_stall_tail: u64,
    /// Wall clock (ms since epoch) at which the current claim stall started
    /// being timed; 0 = not timing. See [`Self::claim_stall_tail`].
    pub claim_stall_since_ms: u64,
    /// Consecutive not-ready polls at `claim_stall_tail`. Only every 16th one
    /// pays for a clock read — see `dispatch::CLAIM_STALL_POLL_MASK`.
    pub claim_stall_polls: u32,

    /// Wall clock (ms) of this handle's last "slot-claim CAS exhausted" warning;
    /// 0 = never warned. Producer-side counterpart of the `claim_stall_*` fields.
    ///
    /// Per handle and clock-based on purpose. The obvious alternative — the
    /// process-wide `Mutex<HashMap<..>>` limiter in `dispatch` — would put a
    /// lock acquisition on the publish path, and `send()` funnels a rejected
    /// `try_send` through ~70 further attempts, so it is reachable in a tight
    /// loop. See `dispatch::warn_claim_cas_exhausted`.
    pub claim_cas_warn_ms: u64,

    /// COMM-H1: exclusive `flock` proving this process holds the FanoutShm publisher
    /// endpoint slot. Held for the endpoint's lifetime — its Drop (or process death)
    /// releases the lock, which is what lets a peer reclaim a crashed owner's slot.
    /// `None` for every non-FanoutShm backend.
    pub fanout_pub_lock: Option<horus_sys::fs::FileLock>,

    /// COMM-H1: exclusive `flock` proving this process holds the FanoutShm
    /// subscriber endpoint slot (symmetric to `fanout_pub_lock`).
    pub fanout_sub_lock: Option<horus_sys::fs::FileLock>,

    /// SHM region backing the fanout channel matrix (must stay alive for lifetime of ring)
    pub fanout_shm_storage: Option<std::sync::Arc<crate::memory::shm_region::ShmRegion>>,

    /// Wall-clock ms after which a degraded FanoutShm handle may retry `attach`.
    ///
    /// Zero means "not degraded". A non-zero value means this handle asked for a
    /// FanoutShm backend, `ShmFanoutRing::attach` returned `None`, and it fell
    /// back to the shared `ShmData` ring. That fallback used to be PERMANENT:
    /// `initialize_backend` compares the backend against `cached_mode` (which the
    /// fallback pins to `SpscShm`), so it short-circuits forever, and
    /// `check_migration`'s `is_optimal()` gate is satisfied by the shared header
    /// alone, so the branch that would restore `cached_mode` never runs. A
    /// subscriber that lost one attach race then polled a ring nobody writes for
    /// the rest of the process's life, while `backend_name()` — which reports the
    /// SHARED header mode — still called it FanoutShm.
    ///
    /// The common cause is not a corrupt region but a slow one: the creator
    /// publishes the ring's magic last, so every attacher waits on it, and
    /// attach gives up after a fixed deadline. Retrying is therefore usually
    /// enough. The timestamp rate-limits it, because a genuinely incompatible
    /// layout would otherwise mmap and munmap a region that can be hundreds of
    /// megabytes on every poll.
    pub fanout_retry_at_ms: u64,

    /// COMM-H3: producer keep-alive for spilled FanoutShm messages. Each large
    /// (> SPILL_THRESHOLD) serde message is copied into a TensorPool slot and only
    /// a 40-byte descriptor is broadcast; the slot must stay alive until every
    /// subscriber that can still see the descriptor has read it. This holds the
    /// last `capacity` (ring window R) spilled tensors and releases each one once
    /// its ring position is overwritten (drop-oldest ⇒ no subscriber can reach it),
    /// so the spill pool can't fill up and start silently dropping large messages.
    /// Empty for every non-FanoutShm backend. See `dispatch::spill_to_pool` /
    /// `read_spilled_retained`.
    pub spill_keepalive: std::collections::VecDeque<crate::types::Tensor>,

    /// Reusable serialization staging buffer for non-POD (serde) sends.
    ///
    /// Every serde send used to call `bincode::serialize(&msg)`, which returns
    /// a fresh `Vec<u8>`: one `malloc` and one `free` per message, on the
    /// publish path. Allocator latency is not bounded — a `malloc` that has to
    /// take the arena lock, refill a bin or go to `mmap` is orders of magnitude
    /// slower than one that pops a free-list head — so the WCET of a serde
    /// publish was a property of the allocator's state, not of the message.
    /// `bincode::serialize_into` writing into this buffer keeps the capacity
    /// between messages, so after warm-up a steady-state serde send performs no
    /// allocation at all.
    ///
    /// TRADE, stated plainly: the buffer is never shrunk, so this handle's
    /// footprint grows to the largest message it has ever serialized and stays
    /// there. That is deliberate — shrinking would put the `realloc` back on the
    /// publish path, which is the exact cost being removed — but it means a
    /// topic that sends one 10 MB message holds 10 MB per publishing handle for
    /// the handle's lifetime. Bounded by the message type's largest realistic
    /// value, which is working set the topic needs anyway.
    ///
    /// This does NOT make serde publish a bounded-WCET operation: serialization
    /// time is still a function of the value, and a message that outgrows its
    /// slot still takes the `auto_grow_slot_size` path (`ftruncate` + `mremap` +
    /// migration). It removes the allocator from the steady state, nothing more.
    /// POD topics never touch this buffer.
    pub serde_scratch: Vec<u8>,
}

impl Default for LocalState {
    fn default() -> Self {
        Self {
            cached_mode: BackendMode::Unknown,
            role: TopicRole::Unregistered,
            is_pod: false,
            is_same_process: true, // Assume same process until checked
            msg_counter: 0,
            local_head: 0,
            cached_capacity_mask: 0,
            cached_data_ptr: std::ptr::null_mut(),
            local_tail: 0,
            cached_capacity: 0,
            cached_header_ptr: std::ptr::null(),
            cached_seq_ptr: std::ptr::null_mut(),
            cached_colo_stride: 0,
            slot_index: -1,
            participant_generation: 0,
            slot_size: DEFAULT_SLOT_SIZE,
            cached_epoch: 0,
            fanout_shm_pub_id: None,
            fanout_shm_sub_id: None,
            missed: 0,
            claim_stall_tail: 0,
            claim_stall_since_ms: 0,
            claim_stall_polls: 0,
            claim_cas_warn_ms: 0,
            fanout_pub_lock: None,
            fanout_sub_lock: None,
            fanout_shm_storage: None,
            fanout_retry_at_ms: 0,
            spill_keepalive: std::collections::VecDeque::new(),
            // Deliberately empty: a POD topic must never pay for a serde
            // staging buffer it will never use. The first serde send grows it.
            serde_scratch: Vec::new(),
        }
    }
}
