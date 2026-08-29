#![allow(private_interfaces)]
//! Shared memory header for topic detection.
//!
//! The `TopicHeader` is laid out in shared memory for cross-process
//! rendezvous. It contains participant tracking, topology detection, and
//! migration coordination.

use std::mem;
use std::mem::offset_of;
use std::sync::atomic::{AtomicU32, AtomicU64, AtomicU8, Ordering};

use crate::error::{HorusError, HorusResult};

use super::shm_layout as layout;
use super::types::BackendMode;

// ============================================================================
// Constants
// ============================================================================

/// Magic number for topic header validation
pub(crate) const TOPIC_MAGIC: u64 = 0x4144415054495645; // "ADAPTIVE" (kept for backwards compat)

/// Header version for compatibility checking
/// v2: Added slot_size field for large message support
/// v3: Added per-slot sequence array for multi-producer write-completion tracking
/// v4: Two independent breaking changes, shipped together.
///
///     (a) Every millisecond timestamp in this header — participant leases,
///         `last_topology_change_ms`, `stall_since_ms` — moved from
///         `CLOCK_REALTIME` to `CLOCK_MONOTONIC`. See [`current_time_ms`].
///     (b) Added `layout_kind`, selecting the co-located slot geometry for
///         small POD types. It is carved out of `_pad1a`, so no existing field
///         moves, but a v3 reader does not know to consult it: it would address
///         a colo region with the split layout's offsets and read payload bytes
///         as readiness stamps. That is the same class of silent corruption the
///         horus_net offset drift produced once already.
///
/// # Why v4 is a hard break and not a soft migration
///
/// Neither change is detectable in-band by a v3 process.
///
/// The timestamps did not change layout, size or alignment; they changed
/// MEANING. A v3 process stamps a lease at ~1.75e12 (ms since 1970) and a v4
/// process stamps the same field at ~1e7 (ms since boot), and the two compare
/// their values against each other through shared memory with no way to tell
/// which epoch a number came from. Mixed on one segment they mis-judge liveness
/// in both directions at once — the v3 writer's leases look permanently expired
/// to the v4 reader (its slot is reclaimed underneath a live process), and the
/// v4 writer's leases look ~55 years in the future to the v3 reader (a crashed
/// process is never reaped).
///
/// The geometry is worse still: a v3 reader has no `layout_kind` concept at
/// all, so it cannot even ask. There is no in-band discriminator that could
/// make either safe, so the only correct handling is to refuse the segment:
/// `Topic`'s open path rejects a version mismatch outright, which turns a
/// silent corruption into a startup error naming both versions.
///
/// The practical consequence is that a v4 binary will not attach to a topic
/// segment created by a v3 binary still running, and vice versa. Segments live
/// in `/dev/shm` and do not survive a reboot, so on a machine where every HORUS
/// process is restarted together this is invisible; a rolling restart that
/// leaves old and new processes sharing a topic gets the error instead of the
/// corruption.
pub(crate) const TOPIC_VERSION: u32 = 4;

/// Default lease timeout in milliseconds (5 seconds)
pub(crate) const DEFAULT_LEASE_TIMEOUT_MS: u64 = 5000;

/// POD flag values
pub(crate) const POD_NO: u8 = 1;
pub(crate) const POD_YES: u8 = 2;

/// Migration lock states
pub(crate) const MIGRATION_UNLOCKED: u8 = 0;
pub(crate) const MIGRATION_LOCKED: u8 = 1;

/// Maximum number of participants to track for lease management
/// With 24-byte entries: 16 * 24 = 384 bytes (6 cache lines)
pub(crate) const MAX_PARTICIPANTS: usize = 16;

// ============================================================================
// Participant Entry
// ============================================================================

/// Participant entry in the header (24 bytes, cache-friendly)
#[repr(C)]
#[derive(Debug)]
pub(crate) struct ParticipantEntry {
    /// Process ID (0 = empty slot)
    /// MUST be atomic for cross-process visibility in shared memory
    pub(crate) pid: AtomicU32,
    /// Thread ID hash (lower 32 bits for same-thread detection)
    /// MUST be atomic for cross-process visibility in shared memory
    pub(crate) thread_id_hash: AtomicU32,
    /// Role: 0=none, 1=producer, 2=consumer, 3=both
    pub(crate) role: AtomicU8,
    /// Active flag for atomic operations
    pub(crate) active: AtomicU8,
    /// Source host identifier for network-replicated participants.
    /// 0 = local process (default). Non-zero = low byte of remote peer_id_hash.
    /// Set by horus_net ShmRingWriter when writing network-received data.
    pub(crate) source_host: AtomicU8,
    /// Padding for alignment
    pub(crate) _pad: [u8; 5],
    /// When this lease runs out, in [`current_time_ms`] milliseconds —
    /// MONOTONIC since boot, NOT since the Unix epoch. Written by the owning
    /// process and compared by any other process on the host; see
    /// [`current_time_ms`] for why that comparison is sound and for the
    /// clock-step failures the wall clock caused here.
    pub(crate) lease_expires_ms: AtomicU64,
}

impl ParticipantEntry {
    /// Check if the lease has expired
    #[inline]
    pub(crate) fn is_lease_expired(&self, now_ms: u64) -> bool {
        let expires = self.lease_expires_ms.load(Ordering::Acquire);
        if expires == 0 {
            return true;
        }
        now_ms > expires
    }

    /// Update lease expiration timestamp
    #[inline]
    pub(crate) fn refresh_lease(&self, now_ms: u64, timeout_ms: u64) {
        self.lease_expires_ms
            .store(now_ms + timeout_ms, Ordering::Release);
    }

    /// Clear this entry (use atomic for thread safety)
    pub(crate) fn clear(&self) {
        self.active.store(0, Ordering::Release);
        self.lease_expires_ms.store(0, Ordering::Release);
        self.role.store(0, Ordering::Release);
        self.pid.store(0, Ordering::Release);
        self.thread_id_hash.store(0, Ordering::Release);
        self.source_host.store(0, Ordering::Release);
    }
}

/// Decrement a shared-memory participant counter, never below zero.
///
/// These counters are incremented by one process and decremented by another,
/// on memory that survives either of them being killed mid-update. A plain
/// `fetch_sub` on a counter that has already reached zero wraps to
/// `u32::MAX`, and since nothing ever resets these counters short of
/// re-creating the segment, one such wrap makes the topic report four billion
/// subscribers for good.
#[inline]
fn decrement_to_floor(counter: &AtomicU32) {
    let _ = counter.fetch_update(Ordering::AcqRel, Ordering::Acquire, |v| {
        if v == 0 {
            None
        } else {
            Some(v - 1)
        }
    });
}

// ============================================================================
// Topic Header (Cache-Optimized)
// ============================================================================

/// Topic classification for discovery and introspection.
///
/// Stored in the `topic_kind` field of `TopicHeader`. External tools use this
/// to distinguish data topics from service/action transport topics without
/// relying on naming conventions (`.request`/`.response` suffixes).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopicKind {
    /// Normal data topic (default)
    Data = 0,
    /// Service request channel (`{name}.request`)
    ServiceRequest = 1,
    /// Service response channel (`{name}.response`)
    ServiceResponse = 2,
    /// Action goal channel (`{name}/goal`)
    ActionGoal = 3,
    /// Action feedback channel (`{name}/feedback`)
    ActionFeedback = 4,
    /// Action result channel (`{name}/result`)
    ActionResult = 5,
    /// Action status channel (`{name}/status`)
    ActionStatus = 6,
    /// Action cancel channel (`{name}/cancel`)
    ActionCancel = 7,
    /// Internal system topic (e.g. `horus.ctl.*`)
    System = 8,
}

impl TopicKind {
    /// Convert from raw u8 value, defaulting to Data for unknown values.
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::ServiceRequest,
            2 => Self::ServiceResponse,
            3 => Self::ActionGoal,
            4 => Self::ActionFeedback,
            5 => Self::ActionResult,
            6 => Self::ActionStatus,
            7 => Self::ActionCancel,
            8 => Self::System,
            _ => Self::Data,
        }
    }
}

/// Shared memory header for topic detection.
///
/// This header enables fully automatic backend selection based on:
/// - Thread ID comparison (same-thread detection)
/// - PID comparison (same-process detection)
/// - Publisher/subscriber counts (access pattern detection)
/// - POD type registration (zero-copy optimization)
///
/// ## Cache Line Optimization
///
/// **CRITICAL**: Producer (head) and consumer (tail) are on SEPARATE cache lines
/// to prevent false sharing. This is the single most important optimization for
/// achieving sub-20ns latency.
///
/// Layout: 640 bytes (10 cache lines)
/// - Cache line 1: Core metadata (read-mostly)
/// - Cache line 2: PRODUCER ONLY - sequence_or_head (written by sender)
/// - Cache line 3: CONSUMER ONLY - tail (written by receiver)
/// - Cache line 4: Counters and timestamps
/// - Cache lines 5-10: Participant tracking
#[repr(C, align(64))]
pub(crate) struct TopicHeader {
    // === Cache line 1 (bytes 0-63): Core metadata (read-mostly) ===
    /// Magic number for validation
    pub(crate) magic: u64,
    /// Header version for compatibility
    pub(crate) version: u32,
    /// Type size in bytes
    pub(crate) type_size: u32,
    /// Type alignment requirement
    pub(crate) type_align: u32,
    /// Is POD type: 0=unknown, 1=no, 2=yes
    pub(crate) is_pod: AtomicU8,
    /// Current backend mode (BackendMode as u8)
    pub(crate) backend_mode: AtomicU8,
    /// Migration lock: 0=unlocked, 1=locked
    pub(crate) migration_lock: AtomicU8,
    /// Verbose content logging flag — toggled at runtime by TUI monitor.
    /// 0 = disabled (default), non-zero = log send/recv content to GLOBAL_LOG_BUFFER.
    /// Note: metrics (messages_total) are always collected regardless of this flag.
    pub(crate) verbose: AtomicU8,
    /// Creator process ID
    pub(crate) creator_pid: u32,
    /// Creator thread ID hash (for same-thread detection)
    pub(crate) creator_thread_id_hash: u64,
    /// Migration epoch (incremented on each backend switch)
    pub(crate) migration_epoch: AtomicU64,
    /// Topic kind: Data=0, ServiceRequest=1, ServiceResponse=2, etc.
    /// Set once at topic creation. Used by discovery to classify topics.
    pub(crate) topic_kind: u8,
    /// Which slot geometry the data region uses: `LAYOUT_SPLIT` or
    /// `LAYOUT_COLO` (see `shm_layout`).
    ///
    /// Carved out of `_pad1a`, so `messages_total` keeps offset 56 and every
    /// constant in `shm_layout` is unchanged. Atomic because a reader in
    /// another process loads it while attaching.
    pub(crate) layout_kind: AtomicU8,
    /// Alignment padding for messages_total
    pub(crate) _pad1a: [u8; 6],
    /// Padding where `messages_total` used to live.
    ///
    /// It moved to the producer line below. This cache line is read by
    /// consumers on the hot path — `migration_check!` Acquire-loads
    /// `migration_epoch` at offset 40 on EVERY recv — and a producer doing a
    /// locked read-modify-write here on every send made the two false-share,
    /// ping-ponging the line between cores once per message. Measured at ~36ns
    /// of a ~150ns one-way latency on an i7-10750H, against a comment that
    /// claimed the increment cost ~1ns.
    pub(crate) _pad1b: [u8; 8],

    // === Cache line 2 (bytes 64-127): PRODUCER WRITE LINE ===
    // This cache line is ONLY written by producers (senders)
    // NEVER put consumer-written fields here!
    /// Write sequence / head (for POD/ring backends) - PRODUCER ONLY
    pub(crate) sequence_or_head: AtomicU64,
    /// Ring buffer capacity (power of 2)
    pub(crate) capacity: u32,
    /// Capacity mask for fast modulo (capacity - 1, only valid if capacity is power of 2)
    pub(crate) capacity_mask: u32,
    /// Slot size in bytes (for non-POD types, includes header + data)
    pub(crate) slot_size: u32,
    /// Alignment padding so the two 8-byte stall words below stay 8-aligned.
    pub(crate) _pad_producer_align: [u8; 4],
    /// Stall detector: the `tail` a producer last observed on a full ring.
    ///
    /// Carved out of what was reserved producer padding, so the header stays
    /// 640 bytes and every existing offset is unchanged. Written only by
    /// producers, from the cold "ring is full" path — see
    /// [`TopicHeader::drain_has_stalled`].
    pub(crate) stall_tail: AtomicU64,
    /// Stall detector: when [`TopicHeader::stall_tail`] was observed
    /// ([`current_time_ms`] monotonic ms). 0 means "no observation on record".
    pub(crate) stall_since_ms: AtomicU64,
    /// Padding to fill cache line (64 - 8 - 4 - 4 - 4 - 4 - 8 - 8 = 24 bytes)
    pub(crate) _pad_producer: [u8; 24],

    // === Cache line 3 (bytes 128-191): LOW-TRAFFIC LINE ===
    //
    // This was "CONSUMER WRITE LINE / NEVER put producer-written fields here",
    // and `messages_total` below breaks that rule deliberately. The rule was
    // aimed at the right problem — keep the per-message writers of the two
    // roles off one line — but `tail` is not a per-message write: a consumer
    // stores it once per `capacity / 2` messages (see the `flush_mask` sites in
    // dispatch.rs), and a producer loads it only when its local view says the
    // ring is full. So this line is idle almost all the time, which is exactly
    // what `messages_total` needs and what neither of the two lines above can
    // offer: line 1 carries `migration_epoch`, polled by every consumer on
    // every recv, and line 2 carries `sequence_or_head`, read by the broadcast
    // recv path on every recv.
    //
    // Measured, best-of-6-reps one-way on an i7-10750H, Topic<CmdVel>:
    // counter on line 1 gave a scattered 160-201ns, on line 2 a flat
    // 166-175ns, here 139-187ns with a 135ns floor when the counter is removed
    // outright. The scatter is the tell — a locked RMW against a line the peer
    // is reading stalls for a variable time, which is why the median moved
    // further than the floor did.
    //
    // The invariant to preserve is therefore "no two PER-MESSAGE writers from
    // different roles on one line", not "one role per line".
    /// Read tail (for ring backends) - CONSUMER ONLY
    pub(crate) tail: AtomicU64,
    /// Total messages ever sent on this topic (always-on atomic counter).
    /// Incremented on every send() regardless of verbose flag.
    ///
    /// Placed here, on the consumer line, because the two lines that look like
    /// its natural home are both read by consumers on EVERY recv: line 1 holds
    /// `migration_epoch`, which `migration_check!` polls per recv, and line 2
    /// holds `sequence_or_head`, which the broadcast recv path reads per recv.
    /// A producer doing a locked read-modify-write on either made the line
    /// ping-pong once per message. Both placements were measured; line 1 gave a
    /// scattered 132-191ns against a 127-133ns floor without the counter, and
    /// line 2 was worse still at a flat 166-175ns.
    ///
    /// `tail` is the only other word here and a consumer stores it once per
    /// `capacity / 2` messages, so this line is very nearly producer-exclusive.
    pub(crate) messages_total: AtomicU64,
    /// Padding to fill cache line (64 - 8 - 8 = 48 bytes)
    pub(crate) _pad_consumer: [u8; 48],

    // === Cache line 4 (bytes 192-255): Counters and metadata ===
    /// Number of active publishers
    pub(crate) publisher_count: AtomicU32,
    /// Number of active subscribers
    pub(crate) subscriber_count: AtomicU32,
    /// Total participants ever connected
    pub(crate) total_participants: AtomicU32,
    /// Lease timeout in milliseconds.
    ///
    /// Atomic because it lives in shared memory and is read by every
    /// participant while the creator writes it, and because the freeze
    /// detector below uses it as its grace period — a test that wants a short
    /// grace has to be able to set it on a live header.
    pub(crate) lease_timeout_ms: AtomicU32,
    /// Last topology change, in [`current_time_ms`] monotonic milliseconds.
    pub(crate) last_topology_change_ms: AtomicU64,
    /// Message type name (null-terminated, e.g. "CmdVel", "Imu").
    /// Set once at topic creation via `std::any::type_name::<T>()`.
    /// External tools read this directly from the mmap'd header.
    pub(crate) type_name: [u8; 32],
    /// Layout hash of the message type, or 0 when unknown.
    ///
    /// The open path validates the type *name* (short name only) and, for POD
    /// types, `type_size`. Neither says anything about field layout, so two
    /// revisions of the same message that keep the name and size but reorder
    /// fields both pass:
    ///
    /// ```text
    /// v1::Pose { x: f32, y: f32 }   sent (x=1, y=2)
    /// v2::Pose { y: f32, x: f32 }   received Pose { y: 1.0, x: 2.0 }
    /// ```
    ///
    /// Same short name "Pose", same 8 bytes — opened without complaint, and the
    /// coordinates arrived swapped with no error anywhere. That is the ordinary
    /// consequence of two nodes built against different revisions of a message
    /// crate, which is exactly what a fleet looks like mid-rollout.
    ///
    /// Carved out of what was reserved padding, so the header stays 640 bytes
    /// and every existing offset is unchanged. Zero means "not supplied":
    /// headers written before this field existed read as 0, and so do topics
    /// opened through `Topic::new`, which cannot know `T`'s layout. Validation
    /// only fires when both sides supply a hash, so an old peer is never
    /// rejected — it is simply not protected.
    pub(crate) layout_hash: AtomicU32,
    /// When the participant table was last swept for dead registrants, as the
    /// low 32 bits of [`current_time_ms`]. 0 means "never swept".
    ///
    /// This is the amortisation window for [`Self::reap_dead_participants`],
    /// whose per-participant liveness probe is a `kill(2)` plus a `/proc` read
    /// and which is reachable from inside `send()`. Shared rather than
    /// process-local so that N processes on one topic cost one sweep per window
    /// between them, not N.
    ///
    /// Carved out of what was reserved padding, so the header stays 640 bytes
    /// and every existing offset is unchanged. It sits on the counters line
    /// deliberately: every caller of the sweep already reads `subscriber_count`
    /// from this same line, so the throttle check pulls in no cache line that
    /// was not being pulled in anyway — and in particular it stays OFF the
    /// producer's write line, which `send()` dirties on every message.
    ///
    /// 32 bits, so it wraps every 49.7 days of uptime. All arithmetic on it is
    /// `wrapping_sub`, which yields the correct elapsed time across a wrap for
    /// any interval shorter than the wrap period — and the window here is a
    /// fraction of a lease timeout, i.e. seconds.
    pub(crate) last_reap_ms: AtomicU32,

    // === Cache lines 5-10 (bytes 256-639): Participant tracking (384 bytes = 16 * 24) ===
    /// Participant entries for lease management
    pub(crate) participants: [ParticipantEntry; MAX_PARTICIPANTS],
}

// Size assertion: Header must be exactly 640 bytes (10 cache lines)
const _: () = assert!(mem::size_of::<TopicHeader>() == 640);

impl TopicHeader {
    #[cfg(test)]
    pub fn zeroed() -> Self {
        Self {
            magic: 0,
            version: 0,
            type_size: 0,
            type_align: 0,
            is_pod: AtomicU8::new(0),
            backend_mode: AtomicU8::new(0),
            migration_lock: AtomicU8::new(0),
            verbose: AtomicU8::new(0),
            creator_pid: 0,
            creator_thread_id_hash: 0,
            migration_epoch: AtomicU64::new(0),
            topic_kind: 0,
            layout_kind: AtomicU8::new(0),
            _pad1a: [0; 6],
            _pad1b: [0; 8],
            // Cache line 2: Producer write line
            sequence_or_head: AtomicU64::new(0),
            capacity: 0,
            capacity_mask: 0,
            slot_size: 0,
            _pad_producer_align: [0; 4],
            stall_tail: AtomicU64::new(0),
            stall_since_ms: AtomicU64::new(0),
            _pad_producer: [0; 24],
            // Cache line 3: Consumer write line
            tail: AtomicU64::new(0),
            messages_total: AtomicU64::new(0),
            _pad_consumer: [0; 48],
            // Cache line 4: Counters
            publisher_count: AtomicU32::new(0),
            subscriber_count: AtomicU32::new(0),
            total_participants: AtomicU32::new(0),
            lease_timeout_ms: AtomicU32::new(0),
            last_topology_change_ms: AtomicU64::new(0),
            type_name: [0; 32],
            layout_hash: AtomicU32::new(0),
            last_reap_ms: AtomicU32::new(0),
            participants: std::array::from_fn(|_| ParticipantEntry {
                pid: AtomicU32::new(0),
                thread_id_hash: AtomicU32::new(0),
                role: AtomicU8::new(0),
                active: AtomicU8::new(0),
                source_host: AtomicU8::new(0),
                _pad: [0; 5],
                lease_expires_ms: AtomicU64::new(0),
            }),
        }
    }

    /// Initialize a new header
    // All 7 arguments are required by the SHM header wire format — grouping them
    // into a struct would add overhead at the only call site (topic open/create).
    #[allow(clippy::too_many_arguments)]
    /// Whether this region uses the co-located slot geometry.
    ///
    /// Read from SHM rather than re-derived from `type_size`: the creator
    /// decided the geometry once, and a later attacher that re-derived it
    /// could disagree (different `horus_core` build, different eligibility
    /// bound) and then read payload bytes as a stamp.
    #[inline]
    pub(crate) fn is_colo(&self) -> bool {
        self.layout_kind.load(Ordering::Acquire) == layout::LAYOUT_COLO
    }

    pub fn init(
        &mut self,
        type_size: u32,
        type_align: u32,
        is_pod: bool,
        capacity: u32,
        slot_size: u32,
        type_name_str: &str,
        topic_kind: u8,
    ) {
        // Ensure capacity is power of 2 for fast modulo
        let capacity = capacity.next_power_of_two();

        // Initialize all fields BEFORE setting magic (race condition fix)
        // Another process might check magic to determine if header is initialized
        self.version = TOPIC_VERSION;
        self.type_size = type_size;
        self.type_align = type_align;
        self.is_pod
            .store(if is_pod { POD_YES } else { POD_NO }, Ordering::Release);
        self.backend_mode
            .store(BackendMode::Unknown as u8, Ordering::Release);
        self.migration_lock
            .store(MIGRATION_UNLOCKED, Ordering::Release);
        self.verbose.store(0, Ordering::Relaxed);
        self.creator_pid = std::process::id();
        self.creator_thread_id_hash = hash_thread_id(std::thread::current().id());
        self.migration_epoch.store(0, Ordering::Release);
        self.topic_kind = topic_kind;
        // Pick the slot geometry once, at creation, and record it. Every later
        // attach reads this rather than re-deriving it from type_size, so a
        // producer and a consumer can never disagree about where a stamp
        // lives — the disagreement mode that made the horus_net offset drift
        // a silent corruption instead of a loud failure.
        self.layout_kind.store(
            if layout::colo_eligible(is_pod, type_size as usize) {
                layout::LAYOUT_COLO
            } else {
                layout::LAYOUT_SPLIT
            },
            Ordering::Release,
        );
        self.messages_total.store(0, Ordering::Release);

        // Cache line 2: Producer write line
        self.sequence_or_head.store(0, Ordering::Release);
        self.capacity = capacity;
        self.capacity_mask = capacity.wrapping_sub(1); // For bitwise AND instead of modulo
        self.slot_size = slot_size;
        self.stall_tail.store(0, Ordering::Release);
        self.stall_since_ms.store(0, Ordering::Release);

        // Cache line 3: Consumer write line
        self.tail.store(0, Ordering::Release);

        // Cache line 4: Counters
        self.publisher_count.store(0, Ordering::Release);
        self.subscriber_count.store(0, Ordering::Release);
        self.total_participants.store(0, Ordering::Release);
        self.lease_timeout_ms
            .store(DEFAULT_LEASE_TIMEOUT_MS as u32, Ordering::Release);
        self.last_topology_change_ms
            .store(current_time_ms(), Ordering::Release);
        // "Never swept". A fresh table has nothing to reap, and this makes the
        // first sweep on the segment unconditional rather than waiting out a
        // window inherited from whatever the memory held before.
        self.last_reap_ms.store(0, Ordering::Release);

        // Cache line 4 (cont): type_name — null-terminated, truncated if needed
        self.type_name = [0u8; 32];
        let stored = type_name_as_stored(type_name_str);
        self.type_name[..stored.len()].copy_from_slice(stored.as_bytes());

        // Clear all participant entries
        for p in &self.participants {
            p.clear();
        }

        // CRITICAL: Set magic LAST with a memory fence to ensure all prior writes
        // are visible to other processes before they see the magic value.
        std::sync::atomic::fence(Ordering::Release);
        self.magic = TOPIC_MAGIC;
    }

    /// Get the current backend mode
    #[inline]
    pub fn mode(&self) -> BackendMode {
        BackendMode::from(self.backend_mode.load(Ordering::Acquire))
    }

    /// Check if verbose content logging is enabled (toggled by TUI monitor).
    /// This only controls content logging (log_summary text), not metrics.
    #[inline(always)]
    pub fn is_verbose(&self) -> bool {
        self.verbose.load(Ordering::Relaxed) != 0
    }

    /// Read the type name stored in the header as a string.
    ///
    /// Returns the null-terminated string from the `type_name` field, or an
    /// empty string if the field is all zeros (header from before this field
    /// was added).
    #[allow(dead_code)] // public API: called by external tools via read_topic_header_info
    pub fn type_name_str(&self) -> &str {
        let end = self
            .type_name
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(self.type_name.len());
        std::str::from_utf8(&self.type_name[..end]).unwrap_or("")
    }

    /// Read the total messages sent counter.
    #[inline]
    #[allow(dead_code)] // public API: called by external tools and tests
    pub fn messages_total(&self) -> u64 {
        self.messages_total.load(Ordering::Relaxed)
    }

    /// Read the topic kind classification.
    #[inline]
    #[allow(dead_code)] // public API: called by external tools and tests
    pub fn topic_kind(&self) -> TopicKind {
        TopicKind::from_u8(self.topic_kind)
    }

    /// Check if all active participants (and caller) are in the same process.
    ///
    /// Also returns `false` when the SHM region was created by a different process
    /// (creator_pid ≠ current PID), even if that process hasn't registered as a
    /// participant yet. This ensures joiners default to cross-process backends
    /// immediately, avoiding the race where a producer sends to a heap ring and
    /// exits before the consumer can trigger migration to SHM.
    #[inline]
    pub fn is_same_process(&self) -> bool {
        let current_pid = std::process::id();

        // If the SHM region was created by a different process, assume cross-process
        // even if the creator hasn't registered in the participant table yet.
        if self.creator_pid != 0 && self.creator_pid != current_pid {
            return false;
        }

        for p in &self.participants {
            if p.active.load(Ordering::Acquire) != 0 && p.pid.load(Ordering::Acquire) != current_pid
            {
                return false;
            }
        }

        true
    }

    /// Check if caller is on the same thread as creator
    #[inline]
    #[cfg(test)]
    pub fn is_same_thread(&self) -> bool {
        self.is_same_process()
            && self.creator_thread_id_hash == hash_thread_id(std::thread::current().id())
    }

    /// Check if type is registered as POD
    #[inline]
    pub fn is_pod_type(&self) -> bool {
        self.is_pod.load(Ordering::Acquire) == POD_YES
    }

    /// Get publisher count
    #[inline]
    pub fn pub_count(&self) -> u32 {
        self.publisher_count.load(Ordering::Acquire)
    }

    /// Get subscriber count
    ///
    /// This counts *registrations*, and a registration only disappears when
    /// something notices the owner is gone — see [`Self::reap_dead_participants`].
    /// Treat it as "how many participants claim to be subscribers", never as
    /// "how many are reading right now": the ring's own `tail` is the only
    /// witness of the latter.
    #[inline]
    pub fn sub_count(&self) -> u32 {
        self.subscriber_count.load(Ordering::Acquire)
    }

    /// How long a participant may go without refreshing its lease before it is
    /// considered gone, in milliseconds.
    ///
    /// Falls back to the default for headers written before the field existed
    /// (they read back as 0, and a zero timeout would expire every participant
    /// the instant it registered).
    #[inline]
    pub(crate) fn lease_timeout(&self) -> u64 {
        match self.lease_timeout_ms.load(Ordering::Acquire) {
            0 => DEFAULT_LEASE_TIMEOUT_MS,
            ms => ms as u64,
        }
    }

    /// Override the lease timeout on a live header.
    ///
    /// Only tests use this: the grace the freeze detector waits out is a whole
    /// lease timeout, and a test that had to sit through the 5-second default
    /// would be five seconds of dead wall clock per case.
    #[cfg(test)]
    pub(crate) fn set_lease_timeout_ms(&self, ms: u32) {
        self.lease_timeout_ms.store(ms, Ordering::Release);
    }

    /// Register as a publisher (returns slot index or error)
    pub fn register_producer(&self) -> HorusResult<usize> {
        self.register_role(1, &self.publisher_count)
    }

    /// Register as a subscriber (returns slot index or error)
    pub fn register_consumer(&self) -> HorusResult<usize> {
        self.register_role(2, &self.subscriber_count)
    }

    /// Claim the first slot that is genuinely free (`active == 0`), if any.
    ///
    /// Split out of [`Self::register_role`] so the "take an empty seat" pass can
    /// run twice — once before and once after reaping dead participants —
    /// without duplicating the CAS protocol.
    #[allow(clippy::too_many_arguments)]
    fn claim_free_slot(
        &self,
        role_bit: u8,
        counter: &AtomicU32,
        pid: u32,
        thread_hash: u32,
        now_ms: u64,
        timeout_ms: u64,
    ) -> Option<usize> {
        for (i, p) in self.participants.iter().enumerate() {
            if p.active.load(Ordering::Acquire) != 0 {
                continue;
            }
            // CAS 0 -> 2 (initializing): exactly one thread wins per slot.
            if p.active
                .compare_exchange(0, 2, Ordering::AcqRel, Ordering::Acquire)
                .is_err()
            {
                continue;
            }
            self.finish_claim(p, role_bit, counter, pid, thread_hash, now_ms, timeout_ms);
            return Some(i);
        }
        None
    }

    /// Populate a slot this thread has already CAS'd to the initializing
    /// sentinel (2), and publish it as active.
    #[allow(clippy::too_many_arguments)]
    fn finish_claim(
        &self,
        p: &ParticipantEntry,
        role_bit: u8,
        counter: &AtomicU32,
        pid: u32,
        thread_hash: u32,
        now_ms: u64,
        timeout_ms: u64,
    ) {
        p.pid.store(pid, Ordering::Release);
        p.thread_id_hash.store(thread_hash, Ordering::Release);
        p.role.store(role_bit, Ordering::Release);
        p.refresh_lease(now_ms, timeout_ms);
        counter.fetch_add(1, Ordering::AcqRel);
        self.total_participants.fetch_add(1, Ordering::AcqRel);
        self.last_topology_change_ms
            .store(now_ms, Ordering::Release);
        // Finalize: transition from initializing (2) to active (1).
        // This makes the slot visible with all fields properly set.
        p.active.store(1, Ordering::Release);
    }

    /// Shared registration logic for producer (role_bit=1) and consumer (role_bit=2).
    fn register_role(&self, role_bit: u8, counter: &AtomicU32) -> HorusResult<usize> {
        let now_ms = current_time_ms();
        let pid = std::process::id();
        let thread_hash = hash_thread_id(std::thread::current().id()) as u32;
        let timeout_ms = self.lease_timeout();

        // First, try to find existing entry for this thread
        for (i, p) in self.participants.iter().enumerate() {
            if p.active.load(Ordering::Acquire) != 0
                && p.pid.load(Ordering::Acquire) == pid
                && p.thread_id_hash.load(Ordering::Acquire) == thread_hash
            {
                let old_role = p.role.fetch_or(role_bit, Ordering::AcqRel);
                if old_role & role_bit == 0 {
                    counter.fetch_add(1, Ordering::AcqRel);
                    self.last_topology_change_ms
                        .store(now_ms, Ordering::Release);
                }
                p.refresh_lease(now_ms, timeout_ms);
                return Ok(i);
            }
        }

        // Find a slot, in three passes of decreasing safety.
        //
        // This used to be a single scan that claimed the first slot that was
        // either free OR merely lease-expired, in index order. An expired lease
        // does NOT mean the owner is gone: leases are refreshed on a message
        // count, so an idle or slow subscriber spends much of its life looking
        // expired (`reap_dead_participants` documents exactly this, and refuses
        // to reap on the lease alone for that reason). The old scan therefore
        // preferred stealing slot 0 from a live, polling subscriber over taking
        // one of the free slots behind it — it decremented `subscriber_count`
        // for a participant that was still reading, after which
        // `nothing_is_draining` saw `sub_count() == 0` and let the producer
        // retire unread slots. Silent message loss, with no counter to show it.
        //
        // We use active=2 as an "initializing" sentinel so that a freshly
        // claimed slot (whose lease_expires_ms is still 0) is not mistaken
        // for an expired slot by another thread.

        // Pass 1: genuinely free slots only. No counter decrement: nothing was
        // registered here.
        if let Some(i) =
            self.claim_free_slot(role_bit, counter, pid, thread_hash, now_ms, timeout_ms)
        {
            return Ok(i);
        }

        // Pass 2: nothing free — retire participants whose process is actually
        // gone (expired lease AND dead pid AND local AND not us), then rescan.
        //
        // Deliberately the UNTHROTTLED sweep. This is the one caller that
        // cannot be told "somebody checked recently": the alternative to
        // finding a slot here is returning "no available participant slots" and
        // failing the registration outright, and a node refusing to start
        // because another process swept the table a millisecond earlier would
        // be a new failure introduced by an optimisation. Registration is a
        // startup-shaped event, so the probes are affordable exactly here.
        let _ = self.reap_dead_participants_now(now_ms);
        if let Some(i) =
            self.claim_free_slot(role_bit, counter, pid, thread_hash, now_ms, timeout_ms)
        {
            return Ok(i);
        }

        // Pass 3, last resort: a stale entry belonging to a thread of THIS
        // process. `Drop for Topic` does not deregister, so a finished thread's
        // slot is never reclaimed by anything else — and the reaper skips our
        // own pid deliberately, because it cannot tell a departed thread from a
        // busy one. A foreign live pid is never stolen.
        for (i, p) in self.participants.iter().enumerate() {
            if p.active.load(Ordering::Acquire) != 1 {
                continue;
            }
            if !p.is_lease_expired(now_ms) {
                continue;
            }
            if p.source_host.load(Ordering::Acquire) != 0 || p.pid.load(Ordering::Acquire) != pid {
                continue;
            }
            if p.active
                .compare_exchange(1, 2, Ordering::AcqRel, Ordering::Acquire)
                .is_err()
            {
                continue; // Someone else got there first.
            }

            // Now that we own the slot, safely decrement the old role counters.
            //
            // Saturating, because these counters live in shared memory
            // that any participant may have been killed in the middle
            // of writing: an unmatched `fetch_sub` on a zero counter
            // wraps to 4 billion, and every consumer of `sub_count()`
            // then believes the topic has four billion subscribers
            // forever. There is no honest recovery from that, so the
            // floor is enforced at the one place the decrement happens.
            let old_role = p.role.load(Ordering::Acquire);
            if old_role & 1 != 0 {
                decrement_to_floor(&self.publisher_count);
            }
            if old_role & 2 != 0 {
                decrement_to_floor(&self.subscriber_count);
            }

            self.finish_claim(p, role_bit, counter, pid, thread_hash, now_ms, timeout_ms);
            return Ok(i);
        }

        // All 16 participant slots occupied by live processes.
        // Count active for diagnostics.
        let active = self
            .participants
            .iter()
            .filter(|p| p.active.load(Ordering::Relaxed) != 0)
            .count();
        Err(HorusError::Communication(
            format!(
                "No available participant slots ({}/{} active). \
                 Consider reducing the number of concurrent publishers/subscribers \
                 on this topic, or increasing lease timeout to allow faster reclamation.",
                active, MAX_PARTICIPANTS
            )
            .into(),
        ))
    }

    /// Drop the registrations of participants whose process no longer exists.
    ///
    /// `subscriber_count` and `publisher_count` had exactly one decrement in
    /// the whole tree — inside `register_role`, and only when a brand-new
    /// participant happened to claim that same expired slot. Nothing else ever
    /// took a registration back: no `Drop`, no deregistration call, no reaper.
    /// So a subscriber that was `kill -9`'d left `sub_count()` at 1 for the life
    /// of the shared-memory segment, and every decision keyed on that count was
    /// made against a participant that had not existed for hours. The one that
    /// mattered: a full ring stays backpressured against a `tail` its only
    /// consumer will never move again, so the producer's `send` drops every
    /// message from then on while `topic list` still reports the topic live at
    /// its real rate. That is the whole of LIVE-5 — one message printed by
    /// `topic echo`, then silence, on a topic publishing at 100 Hz.
    ///
    /// Two conditions, both required, because either alone is wrong:
    ///
    /// * The lease must have expired. This is the cheap filter, and it keeps
    ///   the liveness probe off every participant that is still working.
    /// * The owning process must be gone. An expired lease on its own does NOT
    ///   mean dead. Refresh is clock-gated at half the lease timeout, so a
    ///   healthy participant stays ahead of expiry whatever its rate — but a
    ///   participant that is simply idle (no send, no recv) reaches no refresh
    ///   point at all and will read as expired while perfectly alive. Reaping
    ///   on the lease alone would deregister it.
    ///
    /// Deliberately NOT reaped: our own process (we cannot tell a departed
    /// thread from a busy one, and `Drop` is the right place for that), and
    /// network-replicated participants, whose `pid` belongs to another host and
    /// means nothing here.
    ///
    /// This is not a substitute for the ring's own stall detector
    /// ([`Self::drain_has_stalled`]) — a subscriber can stop reading without
    /// its process dying — but it is what makes the participant table, and
    /// everything that reads it (`topic info`, `detect_optimal_backend`),
    /// describe the system that is actually running.
    ///
    /// # This entry point is AMORTISED. `send()` can reach it.
    ///
    /// `Topic::send` → `send_lossy_retry` → `nothing_is_draining` calls this on
    /// every send once the ring is full, and the liveness probe below is a
    /// `kill(2)` plus a `format!` allocation and an open/read/close on
    /// `/proc/<pid>/stat`, per expired participant, up to 16 of them. That is a
    /// syscall pair on the publish path with no bound on how often it repeats —
    /// and the doc above describes the state that makes it repeat FOREVER: an
    /// idle-but-alive subscriber's lease expires, the sweep probes it, finds it
    /// alive, changes nothing, and the next send does the whole thing again.
    /// Measured shape: tens of microseconds inside one `send()`, on every send,
    /// which is percent of a 1 kHz control period spent asking the kernel a
    /// question whose answer has not changed.
    ///
    /// So a sweep runs at most once per [`Self::reap_interval_ms`] per topic,
    /// coordinated in shared memory so N processes share one window rather than
    /// each getting their own. Callers that need an answer immediately —
    /// `register_role`, which is looking for a slot to claim right now and must
    /// not fail a registration because somebody else swept a millisecond ago —
    /// use [`Self::reap_dead_participants_now`].
    ///
    /// A window is opened only by a sweep that reclaimed NOTHING — see
    /// [`Self::reap_dead_participants_now`]. That is what makes the throttle
    /// target the pathology and not the useful work: the repeat it suppresses is
    /// always a repeat of a sweep that already found nothing to do.
    ///
    /// **What the throttle costs, stated:** if a participant's process dies
    /// during a window that a fruitless sweep opened, reclaiming its slot is
    /// delayed by the remainder of that window. Nothing depends on it being
    /// instant. A slot only becomes reclaimable once its lease expires, which
    /// takes a full lease timeout, so this adds at most a quarter to a delay
    /// that was already a multiple of it. The one path that could care — "the
    /// ring is full and nothing is draining" — has its own independent timer in
    /// [`Self::drain_has_stalled`] running on the FULL lease timeout, four times
    /// longer, so it reaches the same conclusion regardless of whether any given
    /// sweep ran. No safety property and no delivery guarantee is traded here;
    /// only a syscall rate.
    pub(crate) fn reap_dead_participants(&self, now_ms: u64) {
        if !self.claim_reap_window(now_ms) {
            return;
        }
        let _ = self.reap_dead_participants_now(now_ms);
    }

    /// How long a participant-table sweep is good for, in milliseconds.
    ///
    /// A quarter of the lease timeout. Tied to the lease rather than fixed
    /// because the lease is what makes a sweep able to find anything: entries
    /// only become reapable by expiring, which takes a full lease timeout, so
    /// four sweeps per lease period is already three more than the table can
    /// change under. Tests that shorten the lease get a proportionally shorter
    /// window for free.
    ///
    /// Floored at 1 ms so a pathological lease timeout cannot produce a zero
    /// window and silently restore the unthrottled behaviour.
    #[inline]
    fn reap_interval_ms(&self) -> u64 {
        (self.lease_timeout() / 4).max(1)
    }

    /// Claim the right to sweep the participant table now, or decline.
    ///
    /// Exactly one caller wins a given window: the CAS is what makes this hold
    /// across processes, so a topic with one publisher and eight subscribers all
    /// hitting a full ring does one sweep between them, not nine.
    ///
    /// Returns `true` only to the winner.
    #[inline]
    fn claim_reap_window(&self, now_ms: u64) -> bool {
        let now = now_ms as u32;
        let last = self.last_reap_ms.load(Ordering::Relaxed);
        // 0 is "never swept" — always allow, so the first caller on a fresh
        // segment gets a real answer with no warm-up.
        if last != 0 && (now.wrapping_sub(last) as u64) < self.reap_interval_ms() {
            return false;
        }
        // `max(1)` keeps a legitimate reading of zero — the first millisecond
        // after boot on a monotonic clock, and every sweep on a header whose
        // clock is stubbed to 0 in a test — from writing back the "never swept"
        // sentinel and defeating the throttle.
        self.last_reap_ms
            .compare_exchange(last, now.max(1), Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
    }

    /// Sweep the participant table unconditionally, ignoring the throttle.
    ///
    /// For callers that need the table correct *now* and can afford the probes:
    /// `register_role` on its way to failing a registration, and tests. Prefer
    /// [`Self::reap_dead_participants`] anywhere a caller might repeat.
    ///
    /// Returns how many registrations were reclaimed, and records the sweep in
    /// the shared window — but ONLY if it reclaimed nothing. See
    /// [`Self::reap_dead_participants`] for why a fruitful sweep deliberately
    /// leaves the window open.
    pub(crate) fn reap_dead_participants_now(&self, now_ms: u64) -> usize {
        let mut reclaimed = 0usize;
        let me = std::process::id();
        for p in self.participants.iter() {
            // 1 = live entry. 0 = free, 2 = another thread is mid-claim; in
            // both of those cases the slot is not ours to judge.
            if p.active.load(Ordering::Acquire) != 1 {
                continue;
            }
            if !p.is_lease_expired(now_ms) {
                continue;
            }
            let pid = p.pid.load(Ordering::Acquire);
            if pid == 0 || pid == me || p.source_host.load(Ordering::Acquire) != 0 {
                continue;
            }
            if horus_sys::discover::is_process_alive(pid) {
                continue;
            }

            // Claim the slot the same way `register_role` does, so exactly one
            // thread — reaper or new registrant — ever retires a given entry.
            if p.active
                .compare_exchange(1, 2, Ordering::AcqRel, Ordering::Acquire)
                .is_err()
            {
                continue;
            }
            let role = p.role.load(Ordering::Acquire);
            if role & 1 != 0 {
                decrement_to_floor(&self.publisher_count);
            }
            if role & 2 != 0 {
                decrement_to_floor(&self.subscriber_count);
            }
            // Clear the identity BEFORE publishing `active = 0`: that store is
            // what makes the slot claimable, and anything written after it
            // could be overwriting a new owner's fields.
            p.role.store(0, Ordering::Release);
            p.pid.store(0, Ordering::Release);
            p.thread_id_hash.store(0, Ordering::Release);
            p.source_host.store(0, Ordering::Release);
            p.lease_expires_ms.store(0, Ordering::Release);
            p.active.store(0, Ordering::Release);
            self.last_topology_change_ms
                .store(now_ms, Ordering::Release);
            reclaimed += 1;
        }

        // Only a sweep that changed NOTHING opens a quiet window.
        //
        // The cost this throttle exists to remove is the *fruitless repeat* —
        // probe, "still alive", change nothing, probe again on the next send,
        // forever. A sweep that actually reclaimed a slot is not that: the table
        // just changed, `sub_count()` just changed, and whatever asked the
        // question is entitled to ask it again immediately. Writing the sentinel
        // back means the next caller sweeps unthrottled, so the throttle can
        // never sit between a dying participant and the reclaim that unfreezes
        // a ring for it. It costs at most one extra scan per generation of dead
        // participants, because the sweep that follows a fruitful one finds
        // nothing and opens the window itself.
        self.last_reap_ms.store(
            if reclaimed == 0 {
                (now_ms as u32).max(1)
            } else {
                0
            },
            Ordering::Relaxed,
        );
        reclaimed
    }

    /// Has this ring stopped being drained?
    ///
    /// Backpressure is a promise to a consumer: the producer will not overwrite
    /// a message you have not taken. The promise has no expiry, and `tail` is
    /// only moved by an actual `recv`, so a consumer that stops consuming — for
    /// any reason, alive or dead, registered or long gone — freezes the ring
    /// permanently. Every further `send` on the lossy path is then dropped, and
    /// the shared region keeps the same `capacity` messages for the life of the
    /// segment.
    ///
    /// Registration counts cannot answer "is anybody reading". `sub_count()`
    /// says who signed up; a crashed subscriber is still signed up
    /// ([`Self::reap_dead_participants`] fixes that case and only that case),
    /// and a live subscriber that never calls `recv` was never distinguishable
    /// from a busy one. `tail` is the only witness that is a *consequence* of
    /// consumption: it moves if and only if something was consumed.
    ///
    /// So: remember the `tail` seen on a full ring, and the moment it was seen.
    /// If a later full-ring observation finds the same `tail` a whole lease
    /// timeout later, nothing has been taken out of this ring in all that time
    /// and there is nothing left to protect.
    ///
    /// The one thing this is NOT is exact, and it is worth being precise about
    /// where. The SPSC/MPSC recv paths flush `header.tail` only every
    /// `capacity / 2` reads (`dispatch.rs`, so the producer never sees a tail
    /// stale enough to cost it headroom), so a consumer that is reading can
    /// still leave `tail` unmoved for that many messages. For the grace to
    /// run out anyway, a consumer would have to read fewer than `capacity / 2`
    /// messages in a whole lease timeout while the ring stayed full the entire
    /// time — a 512-slot ring means under 51 messages a second against a
    /// producer fast enough to keep it brim-full. That consumer is not being
    /// protected by backpressure in any useful sense: it is being handed a
    /// backlog a full ring deep and reading it minutes late, and today the
    /// producer's every message after the first ring-full is discarded to
    /// preserve that backlog. Lapping it delivers recent data with a counted
    /// gap ([`missed_count`](crate::communication::Topic::missed_count)) instead
    /// of a permanently stale stream, which is the trade this drops on the
    /// right side of.
    ///
    /// Called only from the cold "ring is full" path, so the cost (two relaxed
    /// loads and a clock read) is paid only by a producer that was about to
    /// spin and yield anyway.
    pub(crate) fn drain_has_stalled(&self, now_ms: u64) -> bool {
        let tail = self.tail.load(Ordering::Acquire);
        let observed = self.stall_tail.load(Ordering::Relaxed);
        let since = self.stall_since_ms.load(Ordering::Relaxed);

        // `since == 0` is "no observation yet"; `now_ms < since` is a clock that
        // went backwards, and holding a start time in the future would suppress
        // the detector until wall time caught up.
        if since == 0 || observed != tail || now_ms < since {
            self.stall_tail.store(tail, Ordering::Relaxed);
            self.stall_since_ms.store(now_ms.max(1), Ordering::Relaxed);
            return false;
        }
        now_ms - since >= self.lease_timeout()
    }

    /// Record a `tail` the producer moved itself, without restarting the clock.
    ///
    /// Once the stall is confirmed the producer retires the oldest slot, which
    /// moves `tail`. That is the producer's own write, not evidence a consumer
    /// woke up, so it must not read as drain activity — otherwise the detector
    /// resets on every reclaim and the ring gets exactly one message through per
    /// grace period instead of running freely.
    #[inline]
    pub(crate) fn note_producer_moved_tail(&self, tail: u64) {
        self.stall_tail.store(tail, Ordering::Relaxed);
    }

    /// Try to acquire migration lock
    #[inline]
    pub fn try_lock_migration(&self) -> bool {
        self.migration_lock
            .compare_exchange(
                MIGRATION_UNLOCKED,
                MIGRATION_LOCKED,
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_ok()
    }

    /// Release migration lock
    #[inline]
    pub fn unlock_migration(&self) {
        self.migration_lock
            .store(MIGRATION_UNLOCKED, Ordering::Release);
    }

    /// Detect the optimal backend based on current topology.
    ///
    /// Every topic is SHM-backed (a backing file is created and `creator_pid` is
    /// stamped at `header.init()`), so the data plane always lives in the shared
    /// memory region — a cross-process consumer that joins later can read
    /// published data immediately without a migration + drain cycle. The choice
    /// is purely a function of topology (producer/consumer counts) and whether
    /// the message type is POD.
    pub fn detect_optimal_backend(&self) -> BackendMode {
        let pubs = self.pub_count();
        let subs = self.sub_count();
        let is_pod = self.is_pod_type();

        match (pubs, subs) {
            (0, 0) => BackendMode::Unknown,

            // 1P↔1C (or a single participant anticipating a single counterpart).
            (p, s) if p <= 1 && s <= 1 => BackendMode::SpscShm,

            // Multiple producers, single (or no) consumer → MP claim ring.
            // (Must come before the POD broadcast arm: a multi-producer POD topic
            // is MpscShm, not PodShm.)
            (_, s) if pubs > 1 && s <= 1 => BackendMode::MpscShm,

            // 1→many / 0→many (and MPMC) broadcast of POD types → PodShm broadcast
            // on the shared ring (each subscriber gets every message). This
            // deliberately does NOT use SpmcShm, whose consumers share one tail and
            // COMPETE — a fast consumer would starve the rest, which is wrong for
            // pub/sub broadcast semantics.
            (_, _) if is_pod => BackendMode::PodShm,

            // Non-POD broadcast / MPMC → FanoutShm (contention-free SHM SPSC matrix).
            _ => BackendMode::FanoutShm,
        }
    }
}

// ============================================================================
// Public debug flag API (for external tools like the TUI monitor)
// ============================================================================

/// Byte offset of the `verbose` flag within the topic shared memory header.
/// External tools can write 1/0 to this offset in the mmap'd file to
/// enable/disable verbose content logging for a topic.
pub const TOPIC_VERBOSE_OFFSET: usize = 23;

/// Set the verbose content logging flag on a topic's shared memory region.
///
/// # Safety
/// `shm_ptr` must point to the start of a valid, initialized topic shared
/// memory region (at least 640 bytes). The caller must have write access.
pub unsafe fn set_topic_verbose(shm_ptr: *mut u8, enabled: bool) {
    let flag = shm_ptr.add(TOPIC_VERBOSE_OFFSET);
    flag.write_volatile(if enabled { 1 } else { 0 });
}

// ============================================================================
// Public ring-buffer inspector (for CLI tools like `horus topic echo`)
// ============================================================================

/// Total byte size of `TopicHeader` (= 640, 10 × 64-byte cache lines).
pub const TOPIC_HEADER_SIZE: usize = mem::size_of::<TopicHeader>();

/// Result returned by `read_latest_slot_bytes`.
#[derive(Clone)]
pub struct TopicSlotRead {
    /// Raw payload bytes of the most-recently-written message slot.
    /// For POD types this is the raw struct bytes.  For non-POD types this
    /// is the `bincode`-serialized wire form.
    pub payload: Vec<u8>,
    /// Position of this message in the ring's write sequence.  Pass this back
    /// to `read_latest_slot_bytes` as `last_write_idx` on the next poll; if the
    /// value is unchanged, no new message has been written.
    ///
    /// This counts slots *written*, so it can lag `messages_total`, which
    /// counts `send()` calls including the ones a full ring dropped. It is the
    /// former that indexes the ring, so it is the former this reports.
    pub write_idx: u64,
    /// `true` when the message type is a POD (plain-old-data) type, meaning
    /// `payload` contains raw struct bytes rather than a `bincode` stream.
    pub is_pod: bool,
    /// Message type name read from the header (e.g. "CmdVel", "Imu").
    /// Empty string if the header predates the type_name field.
    pub type_name: String,
    /// Total messages ever sent on this topic.
    pub messages_total: u64,
    /// Topic kind classification (Data, ServiceRequest, etc.).
    pub topic_kind: u8,
}

/// A read-only view of a topic's shared region, opened the way the running
/// platform actually backs it.
///
/// The path-based readers below address a topic by its `topic_shm_path`, which
/// was correct back when every backend was a file: `/dev/shm/horus_<ns>/topics/
/// <name>` on Linux, `/tmp/…` on macOS and on the generic fallback. The Windows
/// backend is not a file at all — `ShmRegion::new` calls
/// `CreateFileMappingW(INVALID_HANDLE_VALUE, …)`, a pagefile-backed section
/// named `Local\horus_<name>`, and `ShmRegion::backing_path()` correspondingly
/// reports `None` there. Nothing is ever created at the topic path, so
/// `File::open` failed with `NotFound` and every reader here returned `None`
/// for every live topic on Windows: `horus topic echo` and `horus topic hz`
/// printed nothing and horus_net's SHM reader exported nothing, on a platform
/// where the ring itself works — the cross-process half is fine there, it is
/// only the region's *address* that differs.
///
/// The path stays the public address because it is the address wherever there
/// is one, and it still *names* the topic where there is not: `topic_shm_path`
/// is `<topics dir>/<name>`, so the name is recoverable from the path and the
/// section can be opened by it.
enum TopicRegion {
    /// A file-backed region (Linux, macOS, the generic fallback), mapped
    /// read-only.
    Mapped(memmap2::Mmap),
    /// A Windows named section. The handle is held for the life of the view.
    #[cfg(target_os = "windows")]
    Section(horus_sys::shm::ShmRegion),
}

impl std::ops::Deref for TopicRegion {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        match self {
            Self::Mapped(mmap) => &mmap[..],
            #[cfg(target_os = "windows")]
            Self::Section(region) => region.as_slice(),
        }
    }
}

/// Map a topic's shared region read-only, or `None` when it is absent, too
/// small to hold a header, or unmappable.
///
/// Every reader in this file goes through here, so the guarantee they all rely
/// on is made once: the returned view is at least `TOPIC_HEADER_SIZE` bytes.
///
/// The file is tried first on every platform, because where one exists it *is*
/// the region. Only when there is none does the Windows section lookup run, so
/// nothing about the file-backed platforms changes.
fn map_topic_region(path: &std::path::Path) -> Option<TopicRegion> {
    use memmap2::MmapOptions;
    use std::fs::File;

    if let Ok(file) = File::open(path) {
        if file.metadata().ok()?.len() < TOPIC_HEADER_SIZE as u64 {
            return None;
        }
        // SAFETY: the file is opened read-only; the mapping is read-only.
        let mmap = unsafe { MmapOptions::new().map(&file).ok()? };
        return Some(TopicRegion::Mapped(mmap));
    }

    open_named_section(path)
}

/// Open the Windows named section a topic path refers to.
///
/// `topic_shm_path` is `shm_topics_dir().join(name)` and a topic name may
/// itself contain separators, so the name is the whole remainder of the path
/// rather than just its last component — `file_name()` on `robot/cmd_vel`
/// would ask the kernel for a section called `cmd_vel`, which is either absent
/// or, worse, a different topic.
#[cfg(target_os = "windows")]
fn open_named_section(path: &std::path::Path) -> Option<TopicRegion> {
    let name = path
        .strip_prefix(horus_sys::shm::shm_topics_dir())
        .ok()?
        .to_str()?;
    let region = horus_sys::shm::ShmRegion::open_existing(name, TOPIC_HEADER_SIZE).ok()?;
    // `open_existing` already refuses a region smaller than the minimum, but it
    // falls back to *assuming* the minimum when `VirtualQuery` fails rather
    // than measuring it. Re-check what we were actually handed, because the
    // readers below index into it on the strength of that guarantee.
    if region.len() < TOPIC_HEADER_SIZE {
        return None;
    }
    Some(TopicRegion::Section(region))
}

/// Non-Windows counterpart of the section lookup: every other backend is
/// file-backed, so a missing file is a missing topic and there is nowhere else
/// to look.
#[cfg(not(target_os = "windows"))]
fn open_named_section(_path: &std::path::Path) -> Option<TopicRegion> {
    None
}

/// Read the latest message payload from a topic's shared-memory region.
///
/// `path` is the topic's `topic_shm_path`. It is the backing file itself on
/// Linux, macOS and the fallback backend; on Windows, where the region is a
/// pagefile-backed named section with nothing on disk, it names the section
/// instead.
///
/// Returns `None` when:
/// - no region exists at `path`, or it cannot be mapped,
/// - the region is too small to contain a valid header,
/// - the magic-number check fails (not a HORUS topic),
/// - `write_idx == last_write_idx` (no new message since the previous call), or
/// - `write_idx == 0` (no message has ever been written on this topic).
///
/// # Layout assumed
///
/// ```text
/// offset 0        : TopicHeader (640 bytes)
/// offset 640      : data region
///   POD  : capacity × type_size bytes; slot[i] = raw POD bytes
///   Serde: capacity × slot_size bytes; slot[i] = [8B pad][8B len][bincode data…]
/// ```
pub fn read_latest_slot_bytes(
    path: &std::path::Path,
    last_write_idx: u64,
) -> Option<TopicSlotRead> {
    read_slot_inner(path, last_write_idx, None)
}

/// Shared body of [`read_latest_slot_bytes`] and [`read_slots_since`].
///
/// `ordinal` selects a specific message by its `messages_total` count; `None`
/// means the newest. Every validation below applies either way — these files
/// are read across namespaces and are not trusted input, so there is one
/// validated path rather than two.
fn read_slot_inner(
    path: &std::path::Path,
    last_write_idx: u64,
    ordinal: Option<u64>,
) -> Option<TopicSlotRead> {
    // ── 1. Map the topic's shared region ─────────────────────────────────────
    // File-backed on Linux/macOS, a named section on Windows; either way at
    // least TOPIC_HEADER_SIZE bytes, which is what everything below assumes.
    let mmap = map_topic_region(path)?;
    let base: *const u8 = mmap.as_ptr();

    // ── 2. Validate magic ─────────────────────────────────────────────────────
    // SAFETY: mmap is at least TOPIC_HEADER_SIZE bytes and page-aligned.
    let magic = unsafe { std::ptr::read_unaligned(base as *const u64) };
    if magic != TOPIC_MAGIC {
        return None;
    }

    // ── 3. Read header fields (unaligned reads — we don't own the struct) ─────
    //   Offsets as per #[repr(C, align(64))] layout (confirmed by size assert):
    //     type_size  : offset 12  (u32)
    //     is_pod     : offset 20  (u8, POD_YES=2)
    //     seq/head   : offset 64  (u64)
    //     capacity   : offset 72  (u32)
    //     cap_mask   : offset 76  (u32)
    //     slot_size  : offset 80  (u32)
    // SAFETY: mmap is validated to be at least TOPIC_HEADER_SIZE (640) bytes above.
    // All offsets (12, 20, 64, 72, 76, 80) are within the header and read_unaligned
    // handles any alignment. base is a valid pointer from the mmap.
    let type_size = unsafe {
        std::ptr::read_unaligned(base.add(offset_of!(TopicHeader, type_size)) as *const u32)
    } as usize;
    // SAFETY: base is a valid mmap pointer; offset 20 is within the validated header region;
    // read_unaligned handles any alignment for the u8 is_pod field.
    let is_pod_raw = unsafe { std::ptr::read_unaligned(base.add(offset_of!(TopicHeader, is_pod))) };
    // SAFETY: base is a valid mmap pointer; offsets 56 and 64 are within the
    // validated header region; read_unaligned handles any alignment for the two
    // u64 counters.
    //
    // The two counters are read together because neither is the answer on its
    // own — which one indexes the ring is decided below, once the geometry is
    // known.
    let messages_total = unsafe {
        std::ptr::read_unaligned(base.add(offset_of!(TopicHeader, messages_total)) as *const u64)
    };
    let head = unsafe {
        std::ptr::read_unaligned(base.add(offset_of!(TopicHeader, sequence_or_head)) as *const u64)
    };
    // SAFETY: base is a valid mmap pointer; offset 72 is within the validated header region;
    // read_unaligned handles any alignment for the u32 capacity field.
    let capacity = unsafe {
        std::ptr::read_unaligned(base.add(offset_of!(TopicHeader, capacity)) as *const u32)
    } as usize;
    // SAFETY: base is a valid mmap pointer; offset 76 is within the validated header region;
    // read_unaligned handles any alignment for the u32 cap_mask field.
    let cap_mask = unsafe {
        std::ptr::read_unaligned(base.add(offset_of!(TopicHeader, capacity_mask)) as *const u32)
    } as usize;
    // SAFETY: base is a valid mmap pointer; offset 80 is within the validated header region;
    // read_unaligned handles any alignment for the u32 slot_size field.
    let slot_size = unsafe {
        std::ptr::read_unaligned(base.add(offset_of!(TopicHeader, slot_size)) as *const u32)
    } as usize;

    // `cap_mask` comes straight out of the file, and `last_written` below is
    // computed as `(write_idx - 1) & cap_mask`. The size check further down
    // validates the mapping against `capacity`, NOT against the mask — so a file
    // declaring `capacity = 8` (passing that check) together with
    // `cap_mask = 0xFFFF_FFFF` yields a slot index up to 4 billion and a
    // `slot_start` far outside the mapping.
    //
    // In the POD branch that is a slice index past the end, i.e. a panic — on
    // whichever thread is reading, and with no `panic = "abort"` that silently
    // kills just that thread. In the serde branch it is worse: `data_len` is
    // pulled with `read_unaligned` at the attacker-chosen offset, which is an
    // out-of-bounds READ rather than a checked index.
    //
    // These files are not trusted input in theory only — this function is how
    // `horus topic echo` and horus_net's export reader consume topics, and
    // `/dev/shm` is walked across namespaces. Require the mask to be exactly the
    // one a power-of-two ring implies, the same check `ShmRingWriter::open_path`
    // already makes on the write side.
    if capacity == 0 || !capacity.is_power_of_two() || cap_mask != capacity - 1 {
        return None;
    }

    let is_pod = is_pod_raw == POD_YES;
    // Which slot geometry the creator chose. Read from the region rather than
    // re-derived from type_size: a reader that re-derived it could disagree
    // with the writer and then read payload bytes as a stamp.
    // SAFETY: OFF_LAYOUT_KIND (49) is inside the validated 640-byte header.
    let colo = unsafe { std::ptr::read_unaligned(base.add(super::shm_layout::OFF_LAYOUT_KIND)) }
        == super::shm_layout::LAYOUT_COLO;

    // ── 3b. Which counter actually indexes the ring ───────────────────────────
    //
    // `messages_total` counts calls to `send()`. `sequence_or_head` counts
    // slots written. They agree only while nothing is dropped — and `send` is
    // the lossy publish, so a full ring drops. Everything below addresses
    // messages in *slot* coordinates (message N lives at `(N - 1) & mask`, and
    // "already lapped" is `head - N >= capacity`), which makes
    // `sequence_or_head` the only counter those two rules are true of.
    //
    // Reading a stale-by-N-drops counter does not fail loudly, which is why
    // this was not obvious: a 512-slot ring with 1000 sends and 488 drops
    // returned message 488's payload labelled as message 1000, with no gap
    // reported. `horus topic echo` printed a plausible, monotonically numbered
    // stream of the wrong messages.
    //
    // An earlier fix keyed on `messages_total` precisely because
    // `sequence_or_head` was seen frozen at the ring capacity on a live 20 Hz
    // topic (116 -> 194 messages while the head sat at 128). That freeze was
    // the symptom of the producer stall, not a property of the counter: a ring
    // that stops being written stops advancing its write cursor. With the stall
    // fixed (`RingTopic::send_lossy_retry`) the head advances again, and it is
    // the only counter that names a slot.
    //
    // The exception is the role=Both same-thread fast path, which writes the
    // data region directly and publishes neither the head nor the per-slot
    // stamps; its `sequence_or_head` sits frozen wherever it was last synced.
    // That is what the stamp probe distinguishes. A ring whose head slot
    // carries a stamp was written by a stamping producer, so its head is live;
    // an unstamped ring has no head worth reading and `messages_total` is the
    // best cursor available for it.
    let write_idx = if head > 0
        && slot_stamp(
            &mmap,
            capacity,
            ((head - 1) as usize) & cap_mask,
            is_pod,
            slot_size,
            colo,
        )
        .is_some_and(|stamp| stamp != 0)
    {
        head
    } else {
        messages_total
    };

    // ── 4. Guard: nothing written yet, or no new data since last poll ─────────
    if write_idx == 0 || write_idx == last_write_idx {
        return None;
    }

    // Which slot to read. `write_idx` counts messages and is incremented *after*
    // the write, so message N lives at `(N - 1) & mask`.
    let target = ordinal.unwrap_or(write_idx);
    if target == 0 || target > write_idx {
        return None;
    }
    // Already lapped: the producer has written a full ring since this message.
    if write_idx.saturating_sub(target) >= capacity as u64 {
        return None;
    }
    let last_written = ((target.wrapping_sub(1)) as usize) & cap_mask;

    // ── 5. Extract payload bytes ──────────────────────────────────────────────
    let payload = if is_pod {
        if type_size == 0 || capacity == 0 {
            return None;
        }
        if colo {
            // Colo layout: [HEADER (640)] [SLOTS (cap * slot_size)], each slot
            // being [stamp (8) | payload | pad to a cache line]. No sequence
            // array, so the data starts immediately after the header.
            let required = super::shm_layout::colo_required_region_len(capacity, slot_size);
            if slot_size < super::shm_layout::COLO_PAYLOAD_OFF + type_size || mmap.len() < required
            {
                return None;
            }
            let slot_start = super::shm_layout::colo_payload_offset(last_written, slot_size);
            let end = slot_start.checked_add(type_size)?;
            mmap.get(slot_start..end)?.to_vec()
        } else {
            // Split layout: [HEADER (640)] [SEQ_ARRAY (cap * 8)] [DATA (cap * type_size)]
            let seq_array_size = capacity * std::mem::size_of::<u64>();
            let data_region_start = TOPIC_HEADER_SIZE + seq_array_size;
            let required = data_region_start + capacity * type_size;
            if mmap.len() < required {
                return None;
            }
            let slot_start = data_region_start + last_written * type_size;
            mmap[slot_start..slot_start + type_size].to_vec()
        }
    } else {
        if slot_size < 16 || capacity == 0 {
            return None;
        }
        // Serde layout: [header (640B)] [seq_array (cap * 8B)] [data slots (cap * slot_size)]
        let seq_array_size = capacity * std::mem::size_of::<u64>();
        let data_region_start = TOPIC_HEADER_SIZE + seq_array_size;
        let required = data_region_start + capacity * slot_size;
        if mmap.len() < required {
            return None;
        }
        // Slot layout: [8 bytes padding][8 bytes data-len (u64 LE)][data…]
        let slot_start = data_region_start + last_written * slot_size;
        let len_offset = slot_start + 8;
        let data_offset = slot_start + 16;
        // SAFETY: len_offset is within mmap bounds (validated by required size check above);
        // read_unaligned handles any alignment.
        let data_len =
            unsafe { std::ptr::read_unaligned(mmap.as_ptr().add(len_offset) as *const u64) }
                as usize;
        // `data_offset + data_len` was computed twice, unchecked, on a length
        // word taken straight out of the file — and release builds do not have
        // `overflow-checks`. A length near `usize::MAX` wrapped the sum down to
        // a small number, so `> mmap.len()` passed, and the slice expression
        // below recomputed the same wrapped value into a range whose start was
        // past its end: a panic on the reading thread of `horus topic echo` or
        // horus_net's export reader, plantable by anyone who can write the file.
        //
        // `checked_add` removes the wrap. The payload is additionally bounded by
        // the SLOT, not by the whole mapping, mirroring the writer's own limit
        // in `write_topic_slot_bytes` — a length larger than a slot can hold was
        // never written by us. `slot_size >= 16 > SERDE_SLOT_OVERHEAD` is
        // guaranteed by the `slot_size < 16` check above, so the subtraction
        // cannot underflow. `get(..)` makes any residual mismatch a `None`
        // rather than a panic.
        let max_payload = slot_size - super::shm_layout::SERDE_SLOT_OVERHEAD;
        let end = data_offset.checked_add(data_len)?;
        if data_len == 0 || data_len > max_payload || end > mmap.len() {
            return None;
        }
        mmap.get(data_offset..end)?.to_vec()
    };

    // Read type_name from header (bytes 216-247, 32 bytes in cache line 4).
    // Offset: cache_line_4(192) + publisher_count(4) + subscriber_count(4) +
    //         total_participants(4) + lease_timeout_ms(4) + last_topology_change_ms(8) = 216
    const TYPE_NAME_OFFSET: usize = offset_of!(TopicHeader, type_name);
    const TYPE_NAME_LEN: usize = 32;
    let type_name = if mmap.len() >= TYPE_NAME_OFFSET + TYPE_NAME_LEN {
        let name_bytes = &mmap[TYPE_NAME_OFFSET..TYPE_NAME_OFFSET + TYPE_NAME_LEN];
        let end = name_bytes
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(TYPE_NAME_LEN);
        std::str::from_utf8(&name_bytes[..end])
            .unwrap_or("")
            .to_string()
    } else {
        String::new()
    };

    // Read topic_kind (byte 48 in cache line 1). `messages_total` was already
    // read above, where the cursor is chosen.
    // SAFETY: mmap is validated to be at least TOPIC_HEADER_SIZE (640) bytes.
    let topic_kind =
        unsafe { std::ptr::read_unaligned(base.add(offset_of!(TopicHeader, topic_kind))) };

    Some(TopicSlotRead {
        payload,
        // The ordinal this slot actually is — NOT the live head. They differ
        // only when an explicit `ordinal` was requested, and that difference is
        // the whole point: `read_slots_since` hands these to a caller whose
        // resume cursor is `last_write_idx.max(slot.write_idx)`. Reporting the
        // head here made the first slot of a batch advance that cursor to the
        // newest message, so every remaining message in the batch was skipped
        // on the next poll — the silent sampling `read_slots_since` exists to
        // end. `target == write_idx` whenever `ordinal` is None, so the
        // freshness contract of `read_latest_slot_bytes` is unchanged.
        write_idx: target,
        is_pod,
        type_name,
        messages_total,
        topic_kind,
    })
}

/// The per-slot ready stamp for `index`, or `None` when the mapping is too
/// small to hold it.
///
/// Readiness deliberately lives in two different places (see `shm_layout`): the
/// sequence array for POD topics, the first word of the slot itself for serde
/// ones. Either way the stamp is `sequence + 1` of the message occupying the
/// slot, with `SLOT_WRITING` set while a producer is mid-write. A slot that no
/// stamping producer has ever touched reads back as 0, which is what
/// `read_slot_inner` uses to tell a live head cursor from a frozen one.
fn slot_stamp(
    mmap: &[u8],
    capacity: usize,
    index: usize,
    is_pod: bool,
    slot_size: usize,
    colo: bool,
) -> Option<u64> {
    use super::shm_layout as layout;

    let offset = if is_pod {
        if colo {
            // Colo has no sequence array: the stamp is the slot's first word.
            layout::colo_stamp_offset(index, slot_size)
        } else {
            layout::seq_slot_offset(index)
        }
    } else {
        if slot_size < layout::SERDE_SLOT_OVERHEAD {
            return None;
        }
        layout::data_slot_offset(capacity, index, slot_size) + layout::SERDE_SLOT_READY_OFF
    };
    if offset.checked_add(mem::size_of::<u64>())? > mmap.len() {
        return None;
    }
    // SAFETY: the read is bounds-checked against the mapping length directly
    // above; read_unaligned handles any alignment.
    let raw = unsafe { std::ptr::read_unaligned(mmap.as_ptr().add(offset) as *const u64) };
    Some(raw & !layout::SLOT_WRITING)
}

/// Read the write-sequence counter from a raw topic SHM file.
///
/// This is a lightweight read used by the monitor to compute accurate message
/// rates without needing a typed `Topic<T>`.  Returns `None` when the file
/// cannot be opened, is too small, or has an invalid magic number.
pub fn read_topic_sequence(path: &std::path::Path) -> Option<u64> {
    let mmap = map_topic_region(path)?;
    let base: *const u8 = mmap.as_ptr();

    // SAFETY: mmap is at least TOPIC_HEADER_SIZE (640) bytes.
    let magic = unsafe { std::ptr::read_unaligned(base as *const u64) };
    if magic != TOPIC_MAGIC {
        return None;
    }
    // SAFETY: offset 64 is within the validated header (sequence_or_head field).
    let seq = unsafe {
        std::ptr::read_unaligned(base.add(offset_of!(TopicHeader, sequence_or_head)) as *const u64)
    };
    Some(seq)
}

/// Read the messages_total counter from a raw topic SHM file.
///
/// Unlike `read_topic_sequence` (which reads `sequence_or_head` at offset 64,
/// flushed lazily for some backends), this reads the `messages_total` counter
/// (offset 56) which is atomically incremented on **every** send() regardless
/// of backend type. Use this for accurate rate measurement.
pub fn read_topic_messages_total(path: &std::path::Path) -> Option<u64> {
    let mmap = map_topic_region(path)?;
    let base: *const u8 = mmap.as_ptr();

    let magic = unsafe { std::ptr::read_unaligned(base as *const u64) };
    if magic != TOPIC_MAGIC {
        return None;
    }
    // SAFETY: offset 56 is within the validated header (messages_total field).
    let total = unsafe {
        std::ptr::read_unaligned(base.add(offset_of!(TopicHeader, messages_total)) as *const u64)
    };
    Some(total)
}

/// Lightweight header-only metadata (no payload read).
pub struct TopicHeaderInfo {
    /// Message type name from the header (e.g. "CmdVel").
    pub type_name: String,
    /// Total messages ever sent.
    pub messages_total: u64,
    /// Topic kind classification.
    pub topic_kind: u8,
    /// Whether the message type is POD.
    pub is_pod: bool,
    /// Message type size in bytes.
    pub type_size: u32,
    /// Number of active publishers.
    pub publisher_count: u32,
    /// Number of active subscribers.
    pub subscriber_count: u32,
}

/// Read header metadata from a topic SHM file without reading any message payload.
///
/// This is cheaper than `read_latest_slot_bytes()` — it only reads the 640-byte
/// header, not the data region. Used by discovery to extract type_name,
/// messages_total, and topic_kind.
/// The prefix of `name` that this header can actually hold.
///
/// The field is 32 bytes with a NUL terminator, so 31 usable. Truncating on a
/// character boundary matters: a cut through a multi-byte character leaves
/// invalid UTF-8, and `type_name_str` renders that as the empty string — which
/// then reads as "this topic has no type" everywhere it is displayed and
/// compared.
///
/// Callers comparing a type name against a header must compare *this*, not the
/// full name. Doing otherwise reported a type as mismatching itself:
///
/// ```text
/// Existing type 'ActionFeedback<CancelTopicFeedb',
/// attempted 'ActionFeedback<CancelTopicFeedback>'
/// ```
pub(crate) fn type_name_as_stored(name: &str) -> &str {
    const USABLE: usize = 31;
    if name.len() <= USABLE {
        return name;
    }
    let mut end = USABLE;
    while end > 0 && !name.is_char_boundary(end) {
        end -= 1;
    }
    &name[..end]
}

pub fn read_topic_header_info(path: &std::path::Path) -> Option<TopicHeaderInfo> {
    let mmap = map_topic_region(path)?;
    let base = mmap.as_ptr();

    // SAFETY: mmap >= TOPIC_HEADER_SIZE (640) bytes.
    let magic = unsafe { std::ptr::read_unaligned(base as *const u64) };
    if magic != TOPIC_MAGIC {
        return None;
    }

    // SAFETY: all offsets are within the validated 640-byte header.
    unsafe {
        let type_size =
            std::ptr::read_unaligned(base.add(offset_of!(TopicHeader, type_size)) as *const u32);
        let is_pod_raw = std::ptr::read_unaligned(base.add(offset_of!(TopicHeader, is_pod)));
        let topic_kind = std::ptr::read_unaligned(base.add(offset_of!(TopicHeader, topic_kind)));
        let messages_total = std::ptr::read_unaligned(
            base.add(offset_of!(TopicHeader, messages_total)) as *const u64,
        );
        let publisher_count = (*(base.add(offset_of!(TopicHeader, publisher_count))
            as *const AtomicU32))
            .load(Ordering::Relaxed);
        let subscriber_count = (*(base.add(offset_of!(TopicHeader, subscriber_count))
            as *const AtomicU32))
            .load(Ordering::Relaxed);

        // type_name at offset 216, 32 bytes
        let name_bytes =
            std::slice::from_raw_parts(base.add(offset_of!(TopicHeader, type_name)), 32);
        let end = name_bytes.iter().position(|&b| b == 0).unwrap_or(32);
        let type_name = std::str::from_utf8(&name_bytes[..end])
            .unwrap_or("")
            .to_string();

        Some(TopicHeaderInfo {
            type_name,
            messages_total,
            topic_kind,
            is_pod: is_pod_raw == POD_YES,
            type_size,
            publisher_count,
            subscriber_count,
        })
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Hash a ThreadId to u64 for storage in shared memory
#[inline]
pub(crate) fn hash_thread_id(id: std::thread::ThreadId) -> u64 {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    let mut hasher = DefaultHasher::new();
    id.hash(&mut hasher);
    hasher.finish()
}

/// Monotonic milliseconds — the timebase for every timestamp this header keeps.
///
/// Participant leases, `last_topology_change_ms` and the drain-stall detector
/// are all stamped with this and compared against it, ACROSS PROCESSES, through
/// shared memory.
///
/// # Why not the wall clock
///
/// This was `SystemTime::now() - UNIX_EPOCH`. Leases are pure interval
/// arithmetic (`now > expires`), so the epoch was never used for anything — but
/// the wall clock can STEP, and a step is read as elapsed time:
///
/// * **Forward** (NTP first sync, VM or container resume, `date -s`): every
///   lease in the table expires in one instant. On the send path that is a
///   burst of liveness probes inside `send()`, then deregistration of live
///   participants, then a topology change, then a migration-epoch bump that
///   forces every other process onto the `handle_epoch_change` path on its next
///   message — a full cascade, on a robot where nothing was wrong.
/// * **Backward**: leases sit in the future and never expire, so a genuinely
///   crashed participant's slot is held forever, `sub_count()` keeps counting
///   it, and a full ring stays backpressured against a `tail` nobody will move
///   again.
///
/// `CLOCK_MONOTONIC` cannot step: it is not settable, and NTP only slews its
/// rate.
///
/// # Why a monotonic clock is sound across processes
///
/// This is the part that has to be checked rather than assumed, because these
/// values are compared between processes. On Linux `CLOCK_MONOTONIC` is
/// documented as "a nonsettable SYSTEM-WIDE clock that represents monotonic
/// time since some unspecified starting point" — the starting point is boot,
/// and it is the same starting point for every process on the host. (The
/// per-process clock is `CLOCK_PROCESS_CPUTIME_ID`, a different clock id;
/// nothing here uses it, and Rust's `Instant` must NOT be substituted either,
/// because its zero is per-process and there is no way to read it out.) So two
/// processes calling this function get directly comparable numbers, which is
/// exactly the property leases need. The same holds on macOS, where
/// `CLOCK_MONOTONIC` is machine uptime.
///
/// Scope of that guarantee, and it is the right scope: values are comparable
/// within ONE BOOT of ONE HOST.
///
/// * **Across hosts** they are meaningless — two machines have unrelated boot
///   times. Nothing carries a lease across a host boundary: `horus_net`
///   addresses this header through `shm_layout`, whose offsets stop at
///   `slot_size` and cover no timestamp or participant field, and
///   `reap_dead_participants` already refuses to judge any participant with a
///   non-zero `source_host` because a remote peer's *pid* is equally
///   meaningless locally. If a lease value is ever added to the network wire
///   format it must be converted to a duration at the sending end; a raw
///   monotonic value must never be replicated.
/// * **Across a reboot** they are meaningless too — but the segments do not
///   survive one. `horus_sys::shm` puts them in `/dev/shm`, a tmpfs the kernel
///   recreates empty at boot. The non-unix fallback backend uses `/tmp`, which
///   is not guaranteed to be cleared, and that platform stays on the wall clock
///   below.
///
/// # Cost
///
/// Unchanged: `clock_gettime(CLOCK_MONOTONIC)` goes through the vDSO exactly as
/// the `SystemTime` read did.
#[cfg(unix)]
#[inline]
pub(crate) fn current_time_ms() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `ts` is a live, writable `timespec` and CLOCK_MONOTONIC is always
    // a valid clock id, so both documented failure modes (EFAULT for a bad
    // pointer, EINVAL for a bad clock id) are unreachable.
    unsafe {
        libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts);
    }
    (ts.tv_sec as u64)
        .wrapping_mul(1_000)
        .wrapping_add(ts.tv_nsec as u64 / 1_000_000)
}

/// Wall-clock milliseconds — the non-unix fallback.
///
/// **STATED COST, NOT HIDDEN:** on this path the clock-step failures described
/// above are still reachable. The two candidates were `Instant`, which is
/// monotonic but whose epoch is per-process and therefore *wrong* for a value
/// two processes compare (it would break liveness on every call, not just after
/// a step), and `GetTickCount64`, which is right but is not in the
/// `windows-sys` feature set this crate enables. Wall clock is the lesser
/// defect of the two available ones. Windows is not a supported deployment
/// target for the real-time paths; if it becomes one, this is the fix to make.
#[cfg(not(unix))]
#[inline]
pub(crate) fn current_time_ms() -> u64 {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::Ordering;

    // ── Helper ──────────────────────────────────────────────────────────

    fn make_header(type_size: u32, type_align: u32, is_pod: bool, capacity: u32) -> TopicHeader {
        let mut h = TopicHeader::zeroed();
        h.init(
            type_size,
            type_align,
            is_pod,
            capacity,
            type_size.max(16),
            "TestType",
            TopicKind::Data as u8,
        );
        h
    }

    /// Like `make_header` but models an in-memory (non-SHM-backed) topic so
    /// `detect_optimal_backend` exercises the heap/intra topology tree rather
    /// than the `shm_backed` (creator_pid != 0) short-circuit that always routes
    /// to SHM backends. `init()` unconditionally stamps the creator PID.
    fn make_inmem_header(
        type_size: u32,
        type_align: u32,
        is_pod: bool,
        capacity: u32,
    ) -> TopicHeader {
        let mut h = make_header(type_size, type_align, is_pod, capacity);
        h.creator_pid = 0;
        h
    }

    // ── Constants ───────────────────────────────────────────────────────

    #[test]
    fn magic_is_ascii_adaptive() {
        let bytes = TOPIC_MAGIC.to_be_bytes();
        assert_eq!(&bytes, b"ADAPTIVE");
    }

    #[test]
    fn header_size_is_640_bytes() {
        assert_eq!(std::mem::size_of::<TopicHeader>(), 640);
        assert_eq!(TOPIC_HEADER_SIZE, 640);
    }

    #[test]
    fn participant_entry_size_is_24_bytes() {
        assert_eq!(std::mem::size_of::<ParticipantEntry>(), 24);
    }

    /// `last_reap_ms` was carved out of reserved padding and must not have
    /// moved anything.
    ///
    /// The header is a cross-process wire format. Two assertions, because they
    /// fail for different reasons: the participant table's offset says no field
    /// above it grew, and `last_reap_ms`'s own offset says it landed in the
    /// four reserved bytes at the end of the counters line rather than
    /// somewhere it would share a line with `sequence_or_head` — which `send()`
    /// dirties on every message, and which is exactly what this field must not
    /// touch.
    #[test]
    fn the_reap_window_fits_in_reserved_padding_without_moving_a_field() {
        assert_eq!(
            std::mem::offset_of!(TopicHeader, participants),
            256,
            "the participant table moved; every process mapping this segment \
             now reads leases at a different offset"
        );
        assert_eq!(
            std::mem::offset_of!(TopicHeader, last_reap_ms),
            252,
            "last_reap_ms is no longer in the counters line's reserved tail"
        );
        // Same 64-byte line as the counters every caller of the sweep already
        // reads, and not the producer's write line.
        assert_eq!(std::mem::offset_of!(TopicHeader, last_reap_ms) / 64, 3);
        assert_eq!(std::mem::offset_of!(TopicHeader, subscriber_count) / 64, 3);
        assert_ne!(
            std::mem::offset_of!(TopicHeader, last_reap_ms) / 64,
            std::mem::offset_of!(TopicHeader, sequence_or_head) / 64
        );
    }

    #[test]
    fn verbose_offset_matches_struct_layout() {
        let h = TopicHeader::zeroed();
        let base = &h as *const TopicHeader as *const u8;
        let debug_ptr = &h.verbose as *const AtomicU8 as *const u8;
        // SAFETY: both pointers derive from the same TopicHeader allocation,
        // so offset_from is well-defined and within the object bounds.
        let offset = unsafe { debug_ptr.offset_from(base) } as usize;
        assert_eq!(offset, TOPIC_VERBOSE_OFFSET);
    }

    // ── TopicHeader::init ───────────────────────────────────────────────

    #[test]
    fn init_sets_magic_and_version() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.magic, TOPIC_MAGIC);
        assert_eq!(h.version, TOPIC_VERSION);
    }

    #[test]
    fn init_sets_type_metadata() {
        let h = make_header(4, 4, false, 32);
        assert_eq!(h.type_size, 4);
        assert_eq!(h.type_align, 4);
        assert_eq!(h.is_pod.load(Ordering::Relaxed), POD_NO);
    }

    #[test]
    fn init_pod_yes_flag() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.is_pod.load(Ordering::Relaxed), POD_YES);
        assert!(h.is_pod_type());
    }

    #[test]
    fn init_pod_no_flag() {
        let h = make_header(8, 8, false, 16);
        assert_eq!(h.is_pod.load(Ordering::Relaxed), POD_NO);
        assert!(!h.is_pod_type());
    }

    #[test]
    fn init_rounds_capacity_to_power_of_two() {
        let h = make_header(8, 8, true, 10);
        assert_eq!(h.capacity, 16); // 10 → next_power_of_two → 16
        assert_eq!(h.capacity_mask, 15);
    }

    #[test]
    fn init_capacity_already_power_of_two() {
        let h = make_header(8, 8, true, 64);
        assert_eq!(h.capacity, 64);
        assert_eq!(h.capacity_mask, 63);
    }

    #[test]
    fn init_capacity_one() {
        let h = make_header(8, 8, true, 1);
        assert_eq!(h.capacity, 1);
        assert_eq!(h.capacity_mask, 0);
    }

    #[test]
    fn init_sets_creator_pid_and_thread() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.creator_pid, std::process::id());
        let expected_hash = hash_thread_id(std::thread::current().id());
        assert_eq!(h.creator_thread_id_hash, expected_hash);
    }

    #[test]
    fn init_backend_starts_unknown() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.mode(), BackendMode::Unknown);
    }

    #[test]
    fn init_counters_are_zero() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.pub_count(), 0);
        assert_eq!(h.sub_count(), 0);
        assert_eq!(h.total_participants.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn init_migration_starts_unlocked() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.migration_lock.load(Ordering::Relaxed), MIGRATION_UNLOCKED);
        assert_eq!(h.migration_epoch.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn init_sequence_and_tail_are_zero() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.sequence_or_head.load(Ordering::Relaxed), 0);
        assert_eq!(h.tail.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn init_verbose_disabled() {
        let h = make_header(8, 8, true, 16);
        assert!(!h.is_verbose());
    }

    #[test]
    fn init_participants_all_cleared() {
        let h = make_header(8, 8, true, 16);
        for p in &h.participants {
            assert_eq!(p.pid.load(Ordering::Relaxed), 0);
            assert_eq!(p.thread_id_hash.load(Ordering::Relaxed), 0);
            assert_eq!(p.role.load(Ordering::Relaxed), 0);
            assert_eq!(p.active.load(Ordering::Relaxed), 0);
            assert_eq!(p.lease_expires_ms.load(Ordering::Relaxed), 0);
        }
    }

    // ── ParticipantEntry ────────────────────────────────────────────────

    #[test]
    fn participant_lease_expired_when_zero() {
        let p = ParticipantEntry {
            pid: AtomicU32::new(0),
            thread_id_hash: AtomicU32::new(0),
            role: AtomicU8::new(0),
            active: AtomicU8::new(0),
            source_host: AtomicU8::new(0),
            _pad: [0; 5],
            lease_expires_ms: AtomicU64::new(0),
        };
        assert!(p.is_lease_expired(1000));
    }

    #[test]
    fn participant_lease_not_expired_when_future() {
        let p = ParticipantEntry {
            pid: AtomicU32::new(0),
            thread_id_hash: AtomicU32::new(0),
            role: AtomicU8::new(0),
            active: AtomicU8::new(0),
            source_host: AtomicU8::new(0),
            _pad: [0; 5],
            lease_expires_ms: AtomicU64::new(5000),
        };
        assert!(!p.is_lease_expired(4999));
    }

    #[test]
    fn participant_lease_expired_when_past() {
        let p = ParticipantEntry {
            pid: AtomicU32::new(0),
            thread_id_hash: AtomicU32::new(0),
            role: AtomicU8::new(0),
            active: AtomicU8::new(0),
            source_host: AtomicU8::new(0),
            _pad: [0; 5],
            lease_expires_ms: AtomicU64::new(5000),
        };
        assert!(p.is_lease_expired(5001));
    }

    #[test]
    fn participant_lease_expired_at_exact_boundary() {
        let p = ParticipantEntry {
            pid: AtomicU32::new(0),
            thread_id_hash: AtomicU32::new(0),
            role: AtomicU8::new(0),
            active: AtomicU8::new(0),
            source_host: AtomicU8::new(0),
            _pad: [0; 5],
            lease_expires_ms: AtomicU64::new(5000),
        };
        // now_ms == expires → not expired (> check, not >=)
        assert!(!p.is_lease_expired(5000));
    }

    #[test]
    fn participant_refresh_lease_updates_expiry() {
        let p = ParticipantEntry {
            pid: AtomicU32::new(0),
            thread_id_hash: AtomicU32::new(0),
            role: AtomicU8::new(0),
            active: AtomicU8::new(0),
            source_host: AtomicU8::new(0),
            _pad: [0; 5],
            lease_expires_ms: AtomicU64::new(0),
        };
        p.refresh_lease(1000, 5000);
        assert_eq!(p.lease_expires_ms.load(Ordering::Relaxed), 6000);
        assert!(!p.is_lease_expired(5999));
        assert!(p.is_lease_expired(6001));
    }

    #[test]
    fn participant_clear_resets_all_fields() {
        let p = ParticipantEntry {
            pid: AtomicU32::new(42),
            thread_id_hash: AtomicU32::new(0xABCD),
            role: AtomicU8::new(3),
            active: AtomicU8::new(1),
            source_host: AtomicU8::new(0),
            _pad: [0; 5],
            lease_expires_ms: AtomicU64::new(99999),
        };
        p.clear();
        assert_eq!(p.pid.load(Ordering::Relaxed), 0);
        assert_eq!(p.thread_id_hash.load(Ordering::Relaxed), 0);
        assert_eq!(p.role.load(Ordering::Relaxed), 0);
        assert_eq!(p.active.load(Ordering::Relaxed), 0);
        assert_eq!(p.lease_expires_ms.load(Ordering::Relaxed), 0);
    }

    // ── Registration ────────────────────────────────────────────────────

    #[test]
    fn register_producer_increments_pub_count() {
        let h = make_header(8, 8, true, 16);
        let slot = h.register_producer().unwrap();
        assert!(slot < MAX_PARTICIPANTS);
        assert_eq!(h.pub_count(), 1);
        assert_eq!(h.sub_count(), 0);
        assert_eq!(h.total_participants.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn register_consumer_increments_sub_count() {
        let h = make_header(8, 8, true, 16);
        let slot = h.register_consumer().unwrap();
        assert!(slot < MAX_PARTICIPANTS);
        assert_eq!(h.sub_count(), 1);
        assert_eq!(h.pub_count(), 0);
    }

    // ── Participant liveness ────────────────────────────────────────────
    //
    // `subscriber_count` had exactly one decrement in the whole tree, inside
    // `register_role`, firing only when a brand-new participant happened to
    // claim that same expired slot. A subscriber that was killed therefore
    // stayed on the books for the life of the segment — and everything keyed on
    // that count, including the backpressure release that keeps `topic echo`
    // showing live data, was decided against a participant that no longer
    // existed.

    /// A pid that is guaranteed not to be running: a child we spawned and then
    /// waited for. Linux hands out pids in increasing order, so it will not be
    /// reused within the lifetime of a test.
    fn a_dead_pid() -> Option<u32> {
        let mut child = std::process::Command::new("/bin/sh")
            .arg("-c")
            .arg("exit 0")
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .spawn()
            .ok()?;
        let pid = child.id();
        child.wait().ok()?;
        Some(pid)
    }

    /// Write the entry a crashed participant leaves behind: role and identity
    /// intact, a lease nobody will ever refresh again.
    fn plant_participant(h: &TopicHeader, index: usize, pid: u32, role: u8, expires_ms: u64) {
        let p = &h.participants[index];
        p.pid.store(pid, Ordering::Release);
        p.thread_id_hash.store(0xDEAD_BEEF, Ordering::Release);
        p.role.store(role, Ordering::Release);
        p.lease_expires_ms.store(expires_ms, Ordering::Release);
        p.active.store(1, Ordering::Release);
        if role & 1 != 0 {
            h.publisher_count.fetch_add(1, Ordering::AcqRel);
        }
        if role & 2 != 0 {
            h.subscriber_count.fetch_add(1, Ordering::AcqRel);
        }
    }

    #[test]
    fn a_subscriber_whose_process_is_gone_stops_being_counted() {
        let Some(dead) = a_dead_pid() else {
            eprintln!("skipping: cannot spawn a process to kill");
            return;
        };
        let h = make_header(8, 8, true, 16);
        plant_participant(&h, 0, dead, 2, 1);
        assert_eq!(h.sub_count(), 1);

        h.reap_dead_participants(current_time_ms());

        assert_eq!(
            h.sub_count(),
            0,
            "the subscriber's process is gone; nothing it registered can still \
             be true"
        );
        assert_eq!(
            h.participants[0].active.load(Ordering::Acquire),
            0,
            "the slot must be free for the next participant, not merely \
             uncounted"
        );
        assert_eq!(h.participants[0].role.load(Ordering::Acquire), 0);
        assert_eq!(h.participants[0].pid.load(Ordering::Acquire), 0);
    }

    #[test]
    fn a_participant_that_is_still_running_is_never_reaped() {
        let h = make_header(8, 8, true, 16);
        // Our own process, with a lease that expired long ago. This is the
        // ordinary state of a healthy low-rate subscriber: leases are refreshed
        // once every 1024 messages, so a 100 Hz consumer refreshes about every
        // 10 s against a 5 s timeout and spends half its life looking expired.
        // Reaping on the lease alone would deregister working subscribers.
        plant_participant(&h, 3, std::process::id(), 2, 1);

        h.reap_dead_participants(current_time_ms());

        assert_eq!(
            h.sub_count(),
            1,
            "an expired lease is not a death certificate — the process is right \
             here"
        );
    }

    #[test]
    fn a_participant_with_a_live_lease_is_never_reaped() {
        let Some(dead) = a_dead_pid() else {
            eprintln!("skipping: cannot spawn a process to kill");
            return;
        };
        let now = current_time_ms();
        let h = make_header(8, 8, true, 16);
        plant_participant(&h, 1, dead, 2, now + 60_000);

        h.reap_dead_participants(now);

        assert_eq!(
            h.sub_count(),
            1,
            "the lease is the cheap filter that keeps the liveness probe off \
             every working participant; a valid lease must short-circuit before \
             the pid is ever looked at"
        );
    }

    #[test]
    fn a_participant_replicated_from_another_host_is_never_reaped() {
        let Some(dead) = a_dead_pid() else {
            eprintln!("skipping: cannot spawn a process to kill");
            return;
        };
        let h = make_header(8, 8, true, 16);
        plant_participant(&h, 2, dead, 2, 1);
        // horus_net stamps `source_host` for network-replicated participants.
        // Their pid identifies a process on the *other* machine, so judging it
        // against this machine's process table would reap every remote peer
        // whose pid happens not to exist locally.
        h.participants[2].source_host.store(7, Ordering::Release);

        h.reap_dead_participants(current_time_ms());

        assert_eq!(
            h.sub_count(),
            1,
            "a remote peer's pid means nothing on this host"
        );
    }

    #[test]
    fn reaping_a_publisher_and_a_consumer_entry_decrements_both() {
        let Some(dead) = a_dead_pid() else {
            eprintln!("skipping: cannot spawn a process to kill");
            return;
        };
        let h = make_header(8, 8, true, 16);
        plant_participant(&h, 4, dead, 3, 1); // role=Both
        assert_eq!((h.pub_count(), h.sub_count()), (1, 1));

        h.reap_dead_participants(current_time_ms());

        assert_eq!((h.pub_count(), h.sub_count()), (0, 0));
    }

    // ── Sweep amortisation ──────────────────────────────────────────────

    /// The sweep must not run on every call, because `send()` is a caller.
    ///
    /// `Topic::send` → `send_lossy_retry` → `nothing_is_draining` calls the
    /// sweep on every send once the ring is full, and each expired participant
    /// costs a `kill(2)` plus an open/read/close on `/proc/<pid>/stat`. The
    /// state that makes it unbounded is ordinary: an idle-but-alive subscriber
    /// looks expired forever, so the probe runs, finds it alive, changes
    /// nothing, and the next send repeats the whole thing. Tens of microseconds
    /// of syscalls inside a publish, on every publish, indefinitely.
    #[test]
    fn the_sweep_runs_at_most_once_per_window() {
        let Some(dead) = a_dead_pid() else {
            eprintln!("skipping: cannot spawn a process to kill");
            return;
        };
        let h = make_header(8, 8, true, 16);
        h.set_lease_timeout_ms(100); // window = lease / 4 = 25 ms

        // A live participant of ours with an expired lease: exactly the state
        // finding 1.6 describes, and the reason the repeat is unbounded. The
        // sweep can never reclaim this entry, so every repeat is pure cost.
        plant_participant(&h, 0, std::process::id(), 2, 1);

        let t0 = current_time_ms();
        h.reap_dead_participants(t0);
        assert_eq!(
            h.sub_count(),
            1,
            "precondition: the sweep found nothing to reclaim, which is what \
             opens the window"
        );

        // Now a registrant's process dies. Inside the window, a caller that may
        // repeat — and `send()` is one — gets nothing: no probe, no syscall.
        plant_participant(&h, 1, dead, 2, 1);
        h.reap_dead_participants(t0 + 24);
        assert_eq!(
            h.sub_count(),
            2,
            "a sweep found nothing 24 ms ago and the window is 25 ms; this call \
             must not have probed /proc again"
        );

        h.reap_dead_participants(t0 + 25);
        assert_eq!(
            h.sub_count(),
            1,
            "the window has elapsed, so the dead registrant is reclaimed — the \
             throttle bounds the syscall rate, it does not disable the sweep"
        );
    }

    /// A sweep that reclaimed something must not suppress the next one.
    ///
    /// The cost being amortised is the *fruitless repeat*: probe, "still
    /// alive", change nothing, probe again on the next send, forever. A sweep
    /// that actually retired a registration is not that — the table just
    /// changed under everyone — so it deliberately leaves the window open. This
    /// is what keeps the throttle from ever sitting between a participant dying
    /// and the reclaim that unfreezes a full ring for it.
    #[test]
    fn a_sweep_that_reclaims_something_does_not_open_a_window() {
        let Some(dead) = a_dead_pid() else {
            eprintln!("skipping: cannot spawn a process to kill");
            return;
        };
        let h = make_header(8, 8, true, 16);
        h.set_lease_timeout_ms(100_000); // a 25 s window, if one ever opened
        let t0 = current_time_ms();

        plant_participant(&h, 0, dead, 2, 1);
        h.reap_dead_participants(t0);
        assert_eq!(
            h.sub_count(),
            0,
            "precondition: this sweep reclaimed a slot"
        );

        // A second one dies a millisecond later — far inside the window a
        // fruitless sweep would have opened.
        plant_participant(&h, 1, dead, 2, 1);
        h.reap_dead_participants(t0 + 1);
        assert_eq!(
            h.sub_count(),
            0,
            "the previous sweep reclaimed a slot, so it must not have opened a \
             window; a producer whose ring is full is entitled to ask again \
             immediately"
        );
    }

    /// The window is a quarter of the lease timeout, and follows it.
    ///
    /// Tied to the lease because the lease is what lets a sweep find anything:
    /// an entry only becomes reapable by expiring, which takes a full lease
    /// timeout. A fixed window would be either uselessly short against a long
    /// lease or, worse, longer than a short one.
    #[test]
    fn the_sweep_window_tracks_the_lease_timeout() {
        let h = make_header(8, 8, true, 16);

        h.set_lease_timeout_ms(4_000);
        assert_eq!(h.reap_interval_ms(), 1_000);

        h.set_lease_timeout_ms(400);
        assert_eq!(h.reap_interval_ms(), 100);

        // Floored, so no lease setting can produce a zero window and silently
        // restore a probe on every send.
        h.set_lease_timeout_ms(1);
        assert_eq!(h.reap_interval_ms(), 1);
    }

    /// A node must never fail to start because somebody else swept recently.
    ///
    /// `register_role`'s second pass reclaims dead participants' slots, and the
    /// alternative to finding one is refusing the registration outright. That
    /// path therefore uses the unthrottled sweep: turning a throttle into a
    /// startup failure would be a new defect introduced by an optimisation.
    #[test]
    fn a_registration_is_never_refused_because_the_sweep_was_throttled() {
        let Some(dead) = a_dead_pid() else {
            eprintln!("skipping: cannot spawn a process to kill");
            return;
        };
        let h = make_header(8, 8, true, 16);
        // A 100 s lease gives a 25 s window — far longer than this test runs,
        // so the throttle is certainly still closed at the registration below.
        h.set_lease_timeout_ms(100_000);

        // Somebody swept a moment ago, on a table that had nothing to reap.
        // The window is now claimed.
        h.reap_dead_participants(current_time_ms());

        // Then every slot fills with participants whose process is gone.
        for i in 0..MAX_PARTICIPANTS {
            plant_participant(&h, i, dead, 2, 1);
        }
        assert_eq!(h.sub_count(), MAX_PARTICIPANTS as u32);

        let slot = h.register_producer().expect(
            "registration must sweep unconditionally: every slot is held by a \
             process that no longer exists, and refusing the node's start \
             because another process swept the table moments ago would be a \
             failure the throttle invented",
        );
        assert!(slot < MAX_PARTICIPANTS);
        assert_eq!(
            h.pub_count(),
            1,
            "the registration is real, not merely a returned index"
        );
    }

    #[test]
    fn a_counter_at_zero_never_wraps_to_four_billion() {
        // These counters are decremented by whichever process notices the owner
        // is gone, on memory that outlives every one of them. Before the floor,
        // one unmatched decrement turned "no subscribers" into "4,294,967,295
        // subscribers" permanently — and every gate that asks `sub_count() == 0`
        // would then be false forever.
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.sub_count(), 0);
        decrement_to_floor(&h.subscriber_count);
        assert_eq!(h.sub_count(), 0);
    }

    // ── Drain stall detection ───────────────────────────────────────────

    #[test]
    fn a_tail_that_keeps_moving_is_never_called_stalled() {
        let h = make_header(8, 8, true, 16);
        h.set_lease_timeout_ms(100);
        let t0 = current_time_ms();
        assert!(
            !h.drain_has_stalled(t0),
            "the first look only starts the clock"
        );
        for step in 1..=10u64 {
            // A consumer took one message: `tail` moved.
            h.tail.store(step, Ordering::Release);
            assert!(
                !h.drain_has_stalled(t0 + step * 1_000),
                "ten seconds have passed but something was consumed within every \
                 100 ms window; backpressure must hold"
            );
        }
    }

    #[test]
    fn a_tail_that_stops_moving_is_called_stalled_after_one_lease() {
        let h = make_header(8, 8, true, 16);
        h.set_lease_timeout_ms(100);
        let t0 = current_time_ms();
        h.tail.store(42, Ordering::Release);
        assert!(!h.drain_has_stalled(t0));
        assert!(
            !h.drain_has_stalled(t0 + 99),
            "inside the grace a consumer is still owed its messages"
        );
        assert!(
            h.drain_has_stalled(t0 + 100),
            "nothing has been taken out of this ring for a whole lease timeout"
        );
    }

    #[test]
    fn the_producers_own_tail_advance_does_not_restart_the_grace() {
        // Once the stall is confirmed the producer retires the oldest slot,
        // which moves `tail`. If that read as consumer activity the clock would
        // restart on every reclaim and exactly one message would get through per
        // grace period — a 100 Hz topic delivering at 0.2 Hz, which looks like
        // the freeze it was supposed to fix.
        let h = make_header(8, 8, true, 16);
        h.set_lease_timeout_ms(100);
        let t0 = current_time_ms();
        h.tail.store(42, Ordering::Release);
        assert!(!h.drain_has_stalled(t0));
        assert!(h.drain_has_stalled(t0 + 100));

        for step in 1..=5u64 {
            h.tail.store(42 + step, Ordering::Release);
            h.note_producer_moved_tail(42 + step);
            assert!(
                h.drain_has_stalled(t0 + 100 + step),
                "the producer moved the tail itself; that is not a consumer \
                 waking up"
            );
        }
    }

    #[test]
    fn a_consumer_waking_up_restores_backpressure() {
        let h = make_header(8, 8, true, 16);
        h.set_lease_timeout_ms(100);
        let t0 = current_time_ms();
        h.tail.store(42, Ordering::Release);
        assert!(!h.drain_has_stalled(t0));
        assert!(h.drain_has_stalled(t0 + 100));

        // Something finally read a message — a position the producer did not
        // write.
        h.tail.store(1_000, Ordering::Release);
        assert!(
            !h.drain_has_stalled(t0 + 101),
            "a ring that is being drained again is owed its backpressure back"
        );
    }

    #[test]
    fn a_clock_that_jumps_backwards_does_not_disable_the_detector() {
        let h = make_header(8, 8, true, 16);
        h.set_lease_timeout_ms(100);
        let t0 = current_time_ms();
        h.tail.store(42, Ordering::Release);
        assert!(!h.drain_has_stalled(t0));
        // `current_time_ms` is monotonic now, so the wall-clock adjustment this
        // was originally written against can no longer reach it. The branch
        // stays and so does this test, because the detector reads its start
        // time out of SHARED MEMORY: `stall_since_ms` is whatever the last
        // producer wrote, and a start time in the future — a stale segment, a
        // corrupted word, a caller passing its own clock — would suppress the
        // detector until real time caught up with it. That is a freeze that
        // outlasts the cause, so the guard is worth keeping even now that the
        // likeliest cause is gone.
        assert!(!h.drain_has_stalled(t0 - 60_000));
        assert!(h.drain_has_stalled(t0 - 60_000 + 100));
    }

    #[test]
    fn a_header_written_before_the_lease_field_existed_uses_the_default_grace() {
        let h = TopicHeader::zeroed();
        assert_eq!(
            h.lease_timeout(),
            DEFAULT_LEASE_TIMEOUT_MS,
            "a zero timeout read out of an old header would expire every \
             participant the instant it registered"
        );
    }

    #[test]
    fn register_same_thread_reuses_slot() {
        let h = make_header(8, 8, true, 16);
        let slot1 = h.register_producer().unwrap();
        let slot2 = h.register_consumer().unwrap();
        assert_eq!(
            slot1, slot2,
            "Same thread should reuse same participant slot"
        );
        assert_eq!(h.pub_count(), 1);
        assert_eq!(h.sub_count(), 1);
        // Role should be both producer and consumer (1 | 2 = 3)
        let role = h.participants[slot1].role.load(Ordering::Relaxed);
        assert_eq!(role, 3);
    }

    #[test]
    fn register_producer_twice_same_thread_no_double_count() {
        let h = make_header(8, 8, true, 16);
        let s1 = h.register_producer().unwrap();
        let s2 = h.register_producer().unwrap();
        assert_eq!(s1, s2);
        assert_eq!(
            h.pub_count(),
            1,
            "Should not double-count same-thread producer"
        );
    }

    #[test]
    fn register_from_different_threads_uses_different_slots() {
        let h = make_header(8, 8, true, 16);
        let slot1 = h.register_producer().unwrap();

        let header_ptr = &h as *const TopicHeader as usize;
        let slot2 = std::thread::spawn(move || {
            // SAFETY: header_ptr was derived from a valid stack-allocated TopicHeader that
            // outlives this thread (main thread joins before h is dropped).
            let h = unsafe { &*(header_ptr as *const TopicHeader) };
            h.register_producer().unwrap()
        })
        .join()
        .unwrap();

        assert_ne!(slot1, slot2);
        assert_eq!(h.pub_count(), 2);
    }

    #[test]
    fn register_fills_all_slots() {
        let h = make_header(8, 8, true, 16);
        let mut slots = Vec::new();

        // Fill MAX_PARTICIPANTS slots from different threads
        let header_ptr = &h as *const TopicHeader as usize;
        let handles: Vec<_> = (0..MAX_PARTICIPANTS)
            .map(|_| {
                std::thread::spawn(move || {
                    // SAFETY: header_ptr was derived from a valid stack-allocated TopicHeader that
                    // outlives all spawned threads (main thread joins before h is dropped).
                    let h = unsafe { &*(header_ptr as *const TopicHeader) };
                    h.register_producer().unwrap()
                })
            })
            .collect();

        for handle in handles {
            slots.push(handle.join().unwrap());
        }

        // All slots should be unique and in range
        slots.sort();
        slots.dedup();
        assert_eq!(slots.len(), MAX_PARTICIPANTS);
        assert_eq!(h.pub_count(), MAX_PARTICIPANTS as u32);
    }

    #[test]
    fn register_fails_when_all_slots_full() {
        let h = make_header(8, 8, true, 16);

        // Fill all slots from different threads
        let header_ptr = &h as *const TopicHeader as usize;
        let handles: Vec<_> = (0..MAX_PARTICIPANTS)
            .map(|_| {
                std::thread::spawn(move || {
                    // SAFETY: header_ptr was derived from a valid stack-allocated TopicHeader that
                    // outlives all spawned threads (main thread joins before h is dropped).
                    let h = unsafe { &*(header_ptr as *const TopicHeader) };
                    h.register_producer().unwrap()
                })
            })
            .collect();
        for handle in handles {
            handle.join().unwrap();
        }

        // The next registration from yet another thread should fail
        let result = std::thread::spawn(move || {
            // SAFETY: header_ptr was derived from a valid stack-allocated TopicHeader that
            // outlives this thread (main thread joins before h is dropped).
            let h = unsafe { &*(header_ptr as *const TopicHeader) };
            h.register_producer()
        })
        .join()
        .unwrap();
        result.unwrap_err();
    }

    // ── Topology detection ──────────────────────────────────────────────

    #[test]
    fn is_same_process_true_when_no_participants() {
        let h = make_header(8, 8, true, 16);
        assert!(h.is_same_process());
    }

    #[test]
    fn is_same_process_true_with_local_participants() {
        let h = make_header(8, 8, true, 16);
        h.register_producer().unwrap();
        assert!(h.is_same_process());
    }

    #[test]
    fn is_same_process_false_with_foreign_pid() {
        let h = make_header(8, 8, true, 16);
        // Manually inject a participant with a different PID
        h.participants[0].pid.store(99999, Ordering::Relaxed);
        h.participants[0].active.store(1, Ordering::Relaxed);
        assert!(!h.is_same_process());
    }

    #[test]
    fn is_same_thread_true_on_same_thread() {
        let h = make_header(8, 8, true, 16);
        assert!(h.is_same_thread());
    }

    // ── Migration lock ──────────────────────────────────────────────────

    #[test]
    fn migration_lock_acquire_and_release() {
        let h = make_header(8, 8, true, 16);
        assert!(h.try_lock_migration());
        assert_eq!(h.migration_lock.load(Ordering::Relaxed), MIGRATION_LOCKED);
        // Second acquire should fail
        assert!(!h.try_lock_migration());
        h.unlock_migration();
        assert_eq!(h.migration_lock.load(Ordering::Relaxed), MIGRATION_UNLOCKED);
        // Now it should succeed again
        assert!(h.try_lock_migration());
    }

    // ── Debug flag ──────────────────────────────────────────────────────

    #[test]
    fn verbose_toggle() {
        let h = make_header(8, 8, true, 16);
        assert!(!h.is_verbose());
        h.verbose.store(1, Ordering::Relaxed);
        assert!(h.is_verbose());
        h.verbose.store(0, Ordering::Relaxed);
        assert!(!h.is_verbose());
    }

    // ── Backend detection ───────────────────────────────────────────────

    #[test]
    fn detect_optimal_backend_no_participants_is_unknown() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.detect_optimal_backend(), BackendMode::Unknown);
    }

    #[test]
    fn detect_optimal_backend_1p1c_pod_is_spsc_shm() {
        let h = make_inmem_header(8, 8, true, 16);
        h.register_producer().unwrap();
        h.register_consumer().unwrap();
        // Every topic is SHM-backed: 1P:1C → SpscShm.
        assert_eq!(h.detect_optimal_backend(), BackendMode::SpscShm);
    }

    #[test]
    fn detect_optimal_backend_1p1c_non_pod_is_spsc_shm() {
        let h = make_inmem_header(8, 8, false, 16);
        h.register_producer().unwrap();
        h.register_consumer().unwrap();
        assert_eq!(h.detect_optimal_backend(), BackendMode::SpscShm);
    }

    #[test]
    fn detect_optimal_backend_single_producer_only() {
        let h = make_inmem_header(8, 8, true, 16);
        h.register_producer().unwrap();
        // 1P, 0C → SpscShm (anticipating single consumer)
        assert_eq!(h.detect_optimal_backend(), BackendMode::SpscShm);
    }

    #[test]
    fn detect_optimal_backend_single_consumer_only() {
        let h = make_inmem_header(8, 8, true, 16);
        h.register_consumer().unwrap();
        assert_eq!(h.detect_optimal_backend(), BackendMode::SpscShm);
    }

    #[test]
    fn detect_optimal_backend_cross_thread_spsc_shm() {
        let h = make_inmem_header(8, 8, true, 16);
        h.register_producer().unwrap();
        let header_ptr = &h as *const TopicHeader as usize;
        std::thread::spawn(move || {
            // SAFETY: header_ptr was derived from a valid stack-allocated TopicHeader that
            // outlives this thread (main thread joins before h is dropped).
            let h = unsafe { &*(header_ptr as *const TopicHeader) };
            h.register_consumer().unwrap();
        })
        .join()
        .unwrap();
        // 1P:1C → SpscShm
        assert_eq!(h.detect_optimal_backend(), BackendMode::SpscShm);
    }

    #[test]
    fn detect_optimal_backend_1p_multi_c_pod_is_pod_shm() {
        let h = make_inmem_header(8, 8, true, 16);
        h.register_producer().unwrap();

        let header_ptr = &h as *const TopicHeader as usize;
        // Register 2 consumers from different threads
        for _ in 0..2 {
            std::thread::spawn(move || {
                // SAFETY: header_ptr was derived from a valid stack-allocated TopicHeader that
                // outlives this thread (main thread joins before h is dropped).
                let h = unsafe { &*(header_ptr as *const TopicHeader) };
                h.register_consumer().unwrap();
            })
            .join()
            .unwrap();
        }
        assert_eq!(h.pub_count(), 1);
        assert!(h.sub_count() >= 2);
        // 1P → multiple consumers, POD → PodShm broadcast (each subscriber gets
        // every message). Deliberately NOT SpmcShm (whose consumers compete).
        assert_eq!(h.detect_optimal_backend(), BackendMode::PodShm);
    }

    #[test]
    fn detect_optimal_backend_multi_p_1c_is_mpsc_shm() {
        let h = make_inmem_header(8, 8, true, 16);
        h.register_consumer().unwrap();

        let header_ptr = &h as *const TopicHeader as usize;
        // Register 2 producers from different threads
        for _ in 0..2 {
            std::thread::spawn(move || {
                // SAFETY: header_ptr was derived from a valid stack-allocated TopicHeader that
                // outlives this thread (main thread joins before h is dropped).
                let h = unsafe { &*(header_ptr as *const TopicHeader) };
                h.register_producer().unwrap();
            })
            .join()
            .unwrap();
        }
        assert!(h.pub_count() >= 2);
        assert_eq!(h.sub_count(), 1);
        assert_eq!(h.detect_optimal_backend(), BackendMode::MpscShm);
    }

    #[test]
    fn detect_optimal_backend_cross_process_spsc_shm() {
        let h = make_header(8, 8, true, 16);
        // Simulate cross-process: inject a participant with different PID
        h.participants[0].pid.store(99999, Ordering::Relaxed);
        h.participants[0]
            .thread_id_hash
            .store(123, Ordering::Relaxed);
        h.participants[0].role.store(1, Ordering::Relaxed); // producer
        h.participants[0].active.store(1, Ordering::Relaxed);
        h.participants[0]
            .lease_expires_ms
            .store(u64::MAX, Ordering::Relaxed);
        h.publisher_count.store(1, Ordering::Relaxed);

        // Register local consumer
        h.participants[1]
            .pid
            .store(std::process::id(), Ordering::Relaxed);
        h.participants[1].thread_id_hash.store(
            hash_thread_id(std::thread::current().id()) as u32,
            Ordering::Relaxed,
        );
        h.participants[1].role.store(2, Ordering::Relaxed); // consumer
        h.participants[1].active.store(1, Ordering::Relaxed);
        h.participants[1]
            .lease_expires_ms
            .store(u64::MAX, Ordering::Relaxed);
        h.subscriber_count.store(1, Ordering::Relaxed);

        assert!(!h.is_same_process());
        assert_eq!(h.detect_optimal_backend(), BackendMode::SpscShm);
    }

    #[test]
    fn detect_optimal_backend_cross_process_pod_shm() {
        let h = make_header(8, 8, true, 16);
        // Simulate multiple cross-process producers and consumers
        h.participants[0].pid.store(99999, Ordering::Relaxed);
        h.participants[0].role.store(1, Ordering::Relaxed);
        h.participants[0].active.store(1, Ordering::Relaxed);
        h.participants[0]
            .lease_expires_ms
            .store(u64::MAX, Ordering::Relaxed);

        h.participants[1].pid.store(99998, Ordering::Relaxed);
        h.participants[1].role.store(3, Ordering::Relaxed); // both
        h.participants[1].active.store(1, Ordering::Relaxed);
        h.participants[1]
            .lease_expires_ms
            .store(u64::MAX, Ordering::Relaxed);

        h.publisher_count.store(2, Ordering::Relaxed);
        h.subscriber_count.store(2, Ordering::Relaxed);

        assert!(!h.is_same_process());
        // Multi-pub, multi-sub, cross-process, POD → PodShm
        assert_eq!(h.detect_optimal_backend(), BackendMode::PodShm);
    }

    // ── Backend mode read/write ─────────────────────────────────────────

    #[test]
    fn mode_roundtrip_all_variants() {
        let h = make_header(8, 8, true, 16);
        let modes = [
            BackendMode::Unknown,
            BackendMode::PodShm,
            BackendMode::MpscShm,
            BackendMode::SpmcShm,
            BackendMode::SpscShm,
            BackendMode::FanoutShm,
        ];
        for mode in modes {
            h.backend_mode.store(mode as u8, Ordering::Relaxed);
            assert_eq!(h.mode(), mode);
        }
    }

    // ── Zeroed header ───────────────────────────────────────────────────

    #[test]
    fn zeroed_has_no_magic() {
        let h = TopicHeader::zeroed();
        assert_eq!(h.magic, 0);
        assert_eq!(h.version, 0);
    }

    // ── hash_thread_id ──────────────────────────────────────────────────

    #[test]
    fn hash_thread_id_deterministic() {
        let id = std::thread::current().id();
        let h1 = hash_thread_id(id);
        let h2 = hash_thread_id(id);
        assert_eq!(h1, h2);
    }

    #[test]
    fn hash_thread_id_different_threads_different_hashes() {
        let main_hash = hash_thread_id(std::thread::current().id());
        let other_hash = std::thread::spawn(|| hash_thread_id(std::thread::current().id()))
            .join()
            .unwrap();
        // Different threads should (almost certainly) produce different hashes
        assert_ne!(main_hash, other_hash);
    }

    // ── current_time_ms ─────────────────────────────────────────────────

    #[test]
    fn current_time_ms_is_reasonable() {
        let now = current_time_ms();
        // Monotonically non-decreasing — the property every lease comparison,
        // the topology timestamp and the drain-stall detector rest on.
        let later = current_time_ms();
        assert!(later >= now);
    }

    /// The lease clock must not be a settable one.
    ///
    /// This test used to assert the OPPOSITE — `now > 1_704_067_200_000`, i.e.
    /// "is this wall-clock time after 2024" — which is what pinned the defect
    /// in place. Leases are stamped into shared memory and compared across
    /// processes, and on a settable clock a single NTP step, VM resume or
    /// `date -s` moves every one of them at once: forward, every participant
    /// expires simultaneously and live subscribers get deregistered out from
    /// under a running robot; backward, no lease ever expires and a crashed
    /// participant holds its slot and its `sub_count()` entry forever.
    ///
    /// A test cannot step the system clock, so it pins the property that makes
    /// a step harmless: the reading is not wall-clock time. Unix-epoch
    /// milliseconds passed 1.7e12 in 2024 and only climb; `CLOCK_MONOTONIC`
    /// counts from boot, so reaching that value takes 54 years of uptime.
    #[test]
    fn the_lease_clock_is_monotonic_not_wall_clock() {
        // Unix-epoch milliseconds at 2024-01-01.
        const EPOCH_MS_2024: u64 = 1_704_067_200_000;

        let now = current_time_ms();

        // The fallback backend is still on the wall clock and says so; see
        // `current_time_ms`. Every deployment target for the real-time paths is
        // unix.
        if !cfg!(unix) {
            return;
        }

        assert!(
            now < EPOCH_MS_2024,
            "current_time_ms() returned {now}, which is wall-clock time since \
             the Unix epoch, not a monotonic reading. Participant leases are \
             compared across processes on this clock: a clock step will now \
             either expire every live participant at once or stop expiring dead \
             ones at all."
        );
    }

    // ── set_topic_verbose (unsafe) ──────────────────────────────────────

    #[test]
    fn set_topic_verbose_via_raw_pointer() {
        let mut h = make_header(8, 8, true, 16);
        assert!(!h.is_verbose());

        let ptr = &mut h as *mut TopicHeader as *mut u8;
        // SAFETY: ptr points to a valid, initialized TopicHeader on the stack (640 bytes).
        unsafe { set_topic_verbose(ptr, true) };
        assert!(h.is_verbose());

        // SAFETY: ptr points to a valid, initialized TopicHeader on the stack (640 bytes).
        unsafe { set_topic_verbose(ptr, false) };
        assert!(!h.is_verbose());
    }

    // ── Lease expiration eviction ───────────────────────────────────────

    #[test]
    fn expired_slot_reclaimed_by_new_registration() {
        let h = make_header(8, 8, true, 16);

        // Manually create an "expired" participant in slot 0
        h.participants[0].pid.store(99999, Ordering::Relaxed);
        h.participants[0]
            .thread_id_hash
            .store(111, Ordering::Relaxed);
        h.participants[0].role.store(1, Ordering::Relaxed); // producer
        h.participants[0].active.store(1, Ordering::Relaxed);
        h.participants[0]
            .lease_expires_ms
            .store(1, Ordering::Relaxed); // expired long ago
        h.publisher_count.store(1, Ordering::Relaxed);

        // INVERTED. This used to assert the new registration took slot 0 — that
        // an expired lease alone was licence to evict. It is not: a lease is
        // refreshed by traffic, so a live-but-idle participant, or one whose
        // refresh point has simply not come round, reads as expired while
        // perfectly alive. Evicting it deregistered working publishers and
        // subscribers, which is how `sub_count()` fell to zero under a live
        // subscriber. Registration now takes a genuinely FREE slot first and
        // leaves the expired entry alone; an expired slot is considered only
        // when no free one remains, and then only if its owner is really gone.
        let slot = h.register_producer().unwrap();
        assert_ne!(
            slot, 0,
            "must not evict an expired slot while others are free"
        );
        assert_eq!(
            h.participants[0].pid.load(Ordering::Relaxed),
            99999,
            "the expired participant must be left untouched"
        );
        assert_eq!(
            h.participants[slot].pid.load(Ordering::Relaxed),
            std::process::id(),
            "the new producer owns the slot it actually claimed"
        );
    }

    // ── Boundary values ─────────────────────────────────────────────────

    #[test]
    fn init_with_zero_type_size() {
        // Zero-sized types (like () or PhantomData) should still work
        let h = make_header(0, 1, true, 16);
        assert_eq!(h.type_size, 0);
        assert_eq!(h.type_align, 1);
    }

    #[test]
    fn init_with_large_type_size() {
        let h = make_header(u32::MAX, 64, false, 16);
        assert_eq!(h.type_size, u32::MAX);
    }

    #[test]
    fn lease_refresh_large_values_no_panic() {
        let p = ParticipantEntry {
            pid: AtomicU32::new(0),
            thread_id_hash: AtomicU32::new(0),
            role: AtomicU8::new(0),
            active: AtomicU8::new(0),
            source_host: AtomicU8::new(0),
            _pad: [0; 5],
            lease_expires_ms: AtomicU64::new(0),
        };
        // Realistic large timestamp (year ~2100 in ms) + large timeout
        let far_future_ms: u64 = 4_102_444_800_000; // 2100-01-01
        let timeout_ms: u64 = 86_400_000; // 24 hours
        p.refresh_lease(far_future_ms, timeout_ms);
        assert_eq!(
            p.lease_expires_ms.load(Ordering::Relaxed),
            far_future_ms + timeout_ms
        );
    }

    // ── type_name field ────────────────────────────────────────────────

    #[test]
    fn init_sets_type_name() {
        let mut h = TopicHeader::zeroed();
        h.init(8, 8, true, 16, 16, "CmdVel", TopicKind::Data as u8);
        assert_eq!(h.type_name_str(), "CmdVel");
    }

    #[test]
    fn init_type_name_truncation_at_31_chars() {
        let long_name = "A".repeat(40);
        let mut h = TopicHeader::zeroed();
        h.init(8, 8, true, 16, 16, &long_name, TopicKind::Data as u8);
        assert_eq!(
            h.type_name_str().len(),
            31,
            "type_name should be truncated to 31 chars, got {}",
            h.type_name_str().len()
        );
        assert_eq!(h.type_name_str(), "A".repeat(31));
    }

    #[test]
    fn init_type_name_empty() {
        let mut h = TopicHeader::zeroed();
        h.init(8, 8, true, 16, 16, "", TopicKind::Data as u8);
        assert_eq!(h.type_name_str(), "");
    }

    #[test]
    fn init_type_name_exact_31_chars() {
        let name = "B".repeat(31);
        let mut h = TopicHeader::zeroed();
        h.init(8, 8, true, 16, 16, &name, TopicKind::Data as u8);
        assert_eq!(h.type_name_str(), name);
    }

    #[test]
    fn type_name_zeroed_returns_empty() {
        let h = TopicHeader::zeroed();
        assert_eq!(h.type_name_str(), "");
    }

    #[test]
    fn init_type_name_with_colons() {
        // Simulates a full Rust path like "horus_library::messages::Imu"
        let mut h = TopicHeader::zeroed();
        h.init(
            8,
            8,
            true,
            16,
            16,
            "horus_library::messages::Imu",
            TopicKind::Data as u8,
        );
        assert_eq!(h.type_name_str(), "horus_library::messages::Imu");
    }

    // ── type_name offset ───────────────────────────────────────────────

    #[test]
    fn type_name_offset_is_216() {
        let h = TopicHeader::zeroed();
        let base = &h as *const TopicHeader as *const u8;
        let field_ptr = h.type_name.as_ptr();
        // SAFETY: both pointers derive from the same TopicHeader allocation.
        let offset = unsafe { field_ptr.offset_from(base) } as usize;
        assert_eq!(offset, 216, "type_name should be at byte offset 216");
    }

    // ── messages_total field ───────────────────────────────────────────

    #[test]
    fn init_messages_total_is_zero() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.messages_total(), 0);
    }

    #[test]
    fn messages_total_increment_readable() {
        let h = make_header(8, 8, true, 16);
        h.messages_total.fetch_add(42, Ordering::Relaxed);
        assert_eq!(h.messages_total(), 42);
    }

    #[test]
    fn messages_total_multiple_increments() {
        let h = make_header(8, 8, true, 16);
        for _ in 0..100 {
            h.messages_total.fetch_add(1, Ordering::Relaxed);
        }
        assert_eq!(h.messages_total(), 100);
    }

    // ── messages_total offset ──────────────────────────────────────────

    /// `messages_total` sits where `shm_layout` says, and off every line a
    /// consumer reads per message.
    ///
    /// The offset alone is already asserted at compile time against
    /// `OFF_MESSAGES_TOTAL`; what this adds is the *reason* the field is where
    /// it is. It used to be pinned to 56 by literal, which said nothing about
    /// why 56, and so could not tell the difference between a deliberate move
    /// and an accidental one. At 56 it shared cache line 1 with
    /// `migration_epoch`, which every consumer Acquire-loads on every recv via
    /// `migration_check!`, so the producer's per-send locked increment
    /// ping-ponged that line between cores.
    #[test]
    fn messages_total_is_off_every_per_recv_line() {
        let h = TopicHeader::zeroed();
        let base = &h as *const TopicHeader as *const u8;
        let field_ptr = &h.messages_total as *const AtomicU64 as *const u8;
        // SAFETY: both pointers derive from the same TopicHeader allocation.
        let offset = unsafe { field_ptr.offset_from(base) } as usize;
        assert_eq!(
            offset,
            super::super::shm_layout::OFF_MESSAGES_TOTAL,
            "messages_total must be where shm_layout tells out-of-crate readers \
             it is"
        );

        const LINE: usize = 64;
        let epoch =
            unsafe { (&h.migration_epoch as *const AtomicU64 as *const u8).offset_from(base) }
                as usize;
        let head =
            unsafe { (&h.sequence_or_head as *const AtomicU64 as *const u8).offset_from(base) }
                as usize;
        assert_ne!(
            offset / LINE,
            epoch / LINE,
            "messages_total shares a cache line with migration_epoch, which \
             every consumer polls on every recv — the producer's per-send \
             locked increment will ping-pong it"
        );
        assert_ne!(
            offset / LINE,
            head / LINE,
            "messages_total shares a cache line with sequence_or_head, which \
             the broadcast recv path reads on every recv"
        );
    }

    // ── topic_kind field ───────────────────────────────────────────────

    #[test]
    fn init_topic_kind_is_data() {
        let h = make_header(8, 8, true, 16);
        assert_eq!(h.topic_kind(), TopicKind::Data);
    }

    #[test]
    fn topic_kind_from_u8_all_variants() {
        assert_eq!(TopicKind::from_u8(0), TopicKind::Data);
        assert_eq!(TopicKind::from_u8(1), TopicKind::ServiceRequest);
        assert_eq!(TopicKind::from_u8(2), TopicKind::ServiceResponse);
        assert_eq!(TopicKind::from_u8(3), TopicKind::ActionGoal);
        assert_eq!(TopicKind::from_u8(4), TopicKind::ActionFeedback);
        assert_eq!(TopicKind::from_u8(5), TopicKind::ActionResult);
        assert_eq!(TopicKind::from_u8(6), TopicKind::ActionStatus);
        assert_eq!(TopicKind::from_u8(7), TopicKind::ActionCancel);
        assert_eq!(TopicKind::from_u8(8), TopicKind::System);
    }

    #[test]
    fn topic_kind_from_u8_unknown_defaults_to_data() {
        assert_eq!(TopicKind::from_u8(9), TopicKind::Data);
        assert_eq!(TopicKind::from_u8(99), TopicKind::Data);
        assert_eq!(TopicKind::from_u8(255), TopicKind::Data);
    }

    // ── topic_kind offset ──────────────────────────────────────────────

    #[test]
    fn topic_kind_offset_is_48() {
        let h = TopicHeader::zeroed();
        let base = &h as *const TopicHeader as *const u8;
        let field_ptr = &h.topic_kind as *const u8;
        // SAFETY: both pointers derive from the same TopicHeader allocation.
        let offset = unsafe { field_ptr.offset_from(base) } as usize;
        assert_eq!(offset, 48, "topic_kind should be at byte offset 48");
    }

    // ── read_topic_header_info ─────────────────────────────────────────

    #[test]
    fn read_topic_header_info_roundtrip() {
        use memmap2::MmapMut;

        let path =
            std::env::temp_dir().join(format!("horus_hdr_info_test_{}.bin", std::process::id()));

        // Write a valid header to temp file
        {
            let file = std::fs::OpenOptions::new()
                .read(true)
                .write(true)
                .create(true)
                .truncate(true)
                .open(&path)
                .expect("create temp file");
            file.set_len(TOPIC_HEADER_SIZE as u64)
                .expect("set file size");
            let mut mmap = unsafe { MmapMut::map_mut(&file).expect("mmap") };
            let header = unsafe { &mut *(mmap.as_mut_ptr() as *mut TopicHeader) };
            *header = TopicHeader::zeroed();
            header.init(4, 4, true, 16, 16, "LaserScan", TopicKind::Data as u8);
            header.messages_total.store(12345, Ordering::Relaxed);
            mmap.flush().expect("flush");
        }

        // Read back via read_topic_header_info
        let info = read_topic_header_info(&path).expect("should read valid header");
        assert_eq!(info.type_name, "LaserScan");
        assert_eq!(info.messages_total, 12345);
        assert_eq!(info.topic_kind, TopicKind::Data as u8);
        assert!(info.is_pod);
        assert_eq!(info.type_size, 4);

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn read_topic_header_info_invalid_magic() {
        let path =
            std::env::temp_dir().join(format!("horus_hdr_bad_magic_{}.bin", std::process::id()));

        // Write garbage
        std::fs::write(&path, [0u8; 640]).expect("write garbage file");
        assert!(
            read_topic_header_info(&path).is_none(),
            "should return None for invalid magic"
        );

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn read_topic_header_info_file_too_small() {
        let path = std::env::temp_dir().join(format!("horus_hdr_small_{}.bin", std::process::id()));

        std::fs::write(&path, [0u8; 100]).expect("write small file");
        assert!(
            read_topic_header_info(&path).is_none(),
            "should return None for file smaller than header"
        );

        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn read_topic_header_info_nonexistent() {
        assert!(
            read_topic_header_info(std::path::Path::new("/tmp/horus_nonexistent_42")).is_none()
        );
    }
}

#[cfg(test)]
mod untrusted_header_tests {
    use super::*;
    use std::io::Write;

    fn tmp(tag: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!(
            "horus_hdr_untrusted_{}_{}",
            tag,
            std::process::id()
        ))
    }

    /// Build a topic file whose header declares `capacity` but a DIFFERENT
    /// `cap_mask` — the shape a hostile file takes.
    use super::super::shm_layout as layout;

    fn put_u64(buf: &mut [u8], off: usize, v: u64) {
        buf[off..off + 8].copy_from_slice(&v.to_ne_bytes());
    }

    fn put_u32(buf: &mut [u8], off: usize, v: u32) {
        buf[off..off + 4].copy_from_slice(&v.to_ne_bytes());
    }

    fn write_header(path: &std::path::Path, capacity: u32, cap_mask: u32, type_size: u32) {
        let total = TOPIC_HEADER_SIZE
            + (capacity as usize) * 8
            + (capacity as usize) * (type_size as usize);
        let mut buf = vec![0u8; total];
        buf[0..8].copy_from_slice(&TOPIC_MAGIC.to_ne_bytes());
        buf[12..16].copy_from_slice(&type_size.to_ne_bytes());
        buf[20] = POD_YES;
        // Both counters, as a real file has them before the ring wraps.
        // `read_latest_slot_bytes` keys on messages_total; sequence_or_head is
        // kept in sync here so the fixture stays faithful.
        //
        // Offsets come from `shm_layout`, never from literals. These fixtures
        // ARE a second implementation of the wire format, and a second copy of
        // the offsets is exactly what drifted in horus_net for four months —
        // and what silently broke these tests when `messages_total` moved off
        // cache line 1.
        put_u64(&mut buf, layout::OFF_MESSAGES_TOTAL, 1);
        put_u64(&mut buf, layout::OFF_SEQUENCE_OR_HEAD, 1);
        put_u32(&mut buf, layout::OFF_CAPACITY, capacity);
        put_u32(&mut buf, layout::OFF_CAPACITY_MASK, cap_mask);
        put_u32(&mut buf, layout::OFF_SLOT_SIZE, type_size);
        let mut f = std::fs::File::create(path).unwrap();
        f.write_all(&buf).unwrap();
    }

    /// `last_written` is `(write_idx - 1) & cap_mask`, and the size check
    /// validates the mapping against `capacity` — NOT the mask. A file declaring
    /// a small capacity with a huge mask therefore produced a slot offset far
    /// outside the mapping: a slice panic in the POD branch, and in the serde
    /// branch a raw `read_unaligned` OUT OF BOUNDS.
    ///
    /// These files are real untrusted input — this is how `horus topic echo` and
    /// horus_net's export reader consume topics, across `/dev/shm` namespaces.
    #[test]
    fn a_cap_mask_wider_than_capacity_is_rejected_not_indexed() {
        let path = tmp("wide_mask");
        write_header(&path, 8, 0xFFFF_FFFF, 8);
        assert!(
            read_latest_slot_bytes(&path, 0).is_none(),
            "a mask inconsistent with capacity must be refused, not used as an index"
        );
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn a_non_power_of_two_capacity_is_rejected() {
        let path = tmp("bad_cap");
        write_header(&path, 7, 6, 8);
        assert!(read_latest_slot_bytes(&path, 0).is_none());
        let _ = std::fs::remove_file(&path);
    }

    /// Build a SERDE-layout topic file with one slot whose length word is
    /// `data_len`.
    ///
    /// Serde slot layout: `[8B ready][8B len][data…]`.
    fn write_serde_header(path: &std::path::Path, slot_size: u32, data_len: u64) {
        let capacity: u32 = 1;
        let total = TOPIC_HEADER_SIZE
            + (capacity as usize) * 8
            + (capacity as usize) * (slot_size as usize);
        let mut buf = vec![0u8; total];
        buf[0..8].copy_from_slice(&TOPIC_MAGIC.to_ne_bytes());
        buf[12..16].copy_from_slice(&8u32.to_ne_bytes()); // type_size (unused on the serde path)
        buf[20] = POD_NO;
        put_u64(&mut buf, layout::OFF_MESSAGES_TOTAL, 1);
        put_u64(&mut buf, layout::OFF_SEQUENCE_OR_HEAD, 1);
        put_u32(&mut buf, layout::OFF_CAPACITY, capacity);
        put_u32(&mut buf, layout::OFF_CAPACITY_MASK, capacity - 1);
        put_u32(&mut buf, layout::OFF_SLOT_SIZE, slot_size);
        let slot_start = TOPIC_HEADER_SIZE + (capacity as usize) * 8;
        buf[slot_start + 8..slot_start + 16].copy_from_slice(&data_len.to_ne_bytes());
        let mut f = std::fs::File::create(path).unwrap();
        f.write_all(&buf).unwrap();
    }

    /// The serde branch's bounds check was `data_offset + data_len > mmap.len()`,
    /// computed on a length word taken straight out of the file with no
    /// `overflow-checks` in release builds. A length near `usize::MAX` wrapped
    /// the sum to a small number, the guard passed, and the slice expression
    /// recomputed the same wrapped value — `mmap[656..400]`, a panic on the
    /// reading thread of `horus topic echo`.
    #[test]
    fn a_wrapping_serde_length_is_rejected_not_sliced() {
        let path = tmp("serde_len_overflow");
        write_serde_header(&path, 64, 0xFFFF_FFFF_FFFF_FF00);
        assert!(
            read_latest_slot_bytes(&path, 0).is_none(),
            "a length word that overflows the offset arithmetic must be refused"
        );
        let _ = std::fs::remove_file(&path);
    }

    /// A length that fits the mapping but not the SLOT is equally impossible to
    /// have been written (`write_topic_slot_bytes` caps payloads at
    /// `slot_size - SERDE_SLOT_OVERHEAD`), and would read a neighbouring slot's
    /// bytes as if they were this message's.
    #[test]
    fn a_serde_length_larger_than_the_slot_is_rejected() {
        let path = tmp("serde_len_over_slot");
        // slot_size 64 => max payload 48.
        write_serde_header(&path, 64, 60);
        assert!(read_latest_slot_bytes(&path, 0).is_none());
        let _ = std::fs::remove_file(&path);
    }

    /// The bound must not reject a payload that really fits.
    #[test]
    fn a_serde_length_within_the_slot_still_reads() {
        let path = tmp("serde_len_ok");
        write_serde_header(&path, 64, 4);
        let slot = read_latest_slot_bytes(&path, 0).expect("a well-formed serde slot must read");
        assert_eq!(slot.payload.len(), 4);
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn a_consistent_header_still_reads() {
        // The guard must not reject legitimate files.
        let path = tmp("good");
        write_header(&path, 8, 7, 8);
        assert!(
            read_latest_slot_bytes(&path, 0).is_some(),
            "a well-formed ring must still be readable"
        );
        let _ = std::fs::remove_file(&path);
    }
}

#[cfg(test)]
mod echo_freshness_tests {
    use crate::communication::Topic;
    use crate::core::DurationExt;

    /// Reading fresh data must keep working after the ring wraps.
    ///
    /// `read_latest_slot_bytes` used to take its freshness signal from
    /// `sequence_or_head`, which is a slot cursor for some backends: it stops
    /// advancing once the ring is full. Measured on a live 20 Hz topic with a
    /// 128-slot ring, `messages_total` climbed 116 -> 194 while
    /// `sequence_or_head` froze at 128.
    ///
    /// Because the caller's test is `write_idx == last_write_idx`, the reader
    /// returned None forever after that point. `horus topic echo` printed
    /// messages until the ring wrapped and then nothing — on a topic
    /// `horus topic list` simultaneously reported as active with a rising
    /// count, because that command reads `messages_total`.
    /// A batch of pending messages must all be delivered, and the caller's
    /// resume cursor must land on the last one it actually got.
    ///
    /// `horus topic echo` advances its cursor with
    /// `last_write_idx = last_write_idx.max(slot.write_idx)`. Every
    /// `TopicSlotRead` used to carry the *live head* rather than its own
    /// ordinal, so the first slot of a batch pushed the cursor straight to the
    /// newest message and the rest of the batch was skipped on the next poll —
    /// the exact silent sampling `read_slots_since` was added to end.
    ///
    /// The original test for that fix polled a 10 Hz publisher, where a batch
    /// is almost always one message and head == ordinal, so it passed against
    /// the broken code. This one leaves several messages pending before it
    /// reads, which is when the two diverge.
    #[test]
    fn a_pending_batch_is_delivered_without_gaps() {
        let name = format!("echo_batch_test_{}", std::process::id());
        let topic: Topic<u64> = match Topic::new(&name) {
            Ok(t) => t,
            Err(e) => {
                eprintln!("skipping: shared memory unavailable ({e})");
                return;
            }
        };
        let Some(path) = horus_sys::shm::topic_shm_path_checked(&name) else {
            eprintln!("skipping: cannot resolve topic path");
            return;
        };

        // Establish the cursor the way `echo` does on its first poll.
        topic.send(0);
        let first = super::read_latest_slot_bytes(&path, 0)
            .expect("a freshly written topic must yield a slot");
        let mut cursor = first.write_idx;

        // Let a batch accumulate before reading — this is the case that broke.
        const BATCH: u64 = 12;
        for i in 1..=BATCH {
            topic.send(i);
        }

        let (slots, lapped) = super::read_slots_since(&path, cursor, 4096);
        assert_eq!(lapped, 0, "a 12-message batch cannot lap a 512-slot ring");
        assert_eq!(
            slots.len() as u64,
            BATCH,
            "expected every pending message, got {}",
            slots.len()
        );

        // Each slot must report its own ordinal, consecutively.
        for (n, slot) in slots.iter().enumerate() {
            let expected = cursor + 1 + n as u64;
            assert_eq!(
                slot.write_idx,
                expected,
                "slot {n} reported ordinal {} but is message {expected}; a \
                 caller resuming from this skips {} message(s)",
                slot.write_idx,
                slot.write_idx.saturating_sub(expected)
            );
        }

        // Advance exactly as echo_topic does, then confirm nothing was skipped.
        for slot in &slots {
            cursor = cursor.max(slot.write_idx);
        }
        assert_eq!(
            cursor,
            first.write_idx + BATCH,
            "the resume cursor overshot the last delivered message"
        );

        topic.send(9_999);
        let (tail, _) = super::read_slots_since(&path, cursor, 4096);
        assert_eq!(
            tail.len(),
            1,
            "after a correct resume the next poll sees exactly the one new \
             message, not a re-read or a hole"
        );
    }

    /// A topic nobody subscribes to must still show its newest message.
    ///
    /// This is the shape of every "let me look at what my node publishes"
    /// session: one publisher, no subscriber, `horus topic echo`. The ring is
    /// backpressured against `tail`, and with no consumer `tail` never moves —
    /// so one ring-full after start-up every further `send` was dropped and the
    /// shared region kept the FIRST `capacity` messages forever. Measured here
    /// before the fix: 1000 sends into a 512-slot ring left the newest readable
    /// payload at message 488, and `send_failures` at 488.
    ///
    /// `messages_total` kept counting all 1000, which is why every command that
    /// reads the header (`topic list`, `topic hz`) reported the topic live at
    /// its real rate while the one that reads the ring showed minute-old data.
    #[test]
    fn a_topic_nobody_subscribes_to_still_shows_its_newest_message() {
        let name = format!("echo_unread_ring_{}", std::process::id());
        let topic: Topic<u64> = match Topic::with_capacity(&name, 64_u32, None) {
            Ok(t) => t,
            Err(e) => {
                eprintln!("skipping: shared memory unavailable ({e})");
                return;
            }
        };
        let Some(path) = horus_sys::shm::topic_shm_path_checked(&name) else {
            eprintln!("skipping: cannot resolve topic path");
            return;
        };

        // Several ring-fulls, so the first wrap is well behind us.
        const SENT: u64 = 500;
        for i in 1..=SENT {
            topic.send(i);
        }

        let slot = super::read_latest_slot_bytes(&path, 0)
            .expect("a topic with 500 messages sent must yield a slot");
        let newest = u64::from_le_bytes(slot.payload[..8].try_into().expect("u64 payload"));
        assert_eq!(
            newest, SENT,
            "the ring is serving message {newest} as the newest of {SENT} — the \
             producer stopped writing once the ring filled and no consumer \
             drained it"
        );
    }

    /// A dropped `send` must not shift every payload by the number dropped.
    ///
    /// `messages_total` counts `send()` calls; `sequence_or_head` counts slots
    /// written. `send` is the lossy publish, so a genuinely full ring — one
    /// whose subscriber is not reading — drops, and from the first drop the two
    /// counters disagree. Addressing a slot as `(messages_total - 1) & mask`
    /// then lands on a different message than the one it names, with no gap
    /// reported: this exact case returned message 8's payload labelled as
    /// message 200.
    ///
    /// A registered-but-idle consumer is what keeps the ring genuinely full
    /// here; without one the producer retires the oldest slot instead (see
    /// `a_topic_nobody_subscribes_to_still_shows_its_newest_message`).
    #[test]
    fn a_dropped_send_does_not_shift_the_payload_by_the_drop_count() {
        const CAPACITY: u32 = 64;
        let name = format!("echo_drop_shift_{}", std::process::id());
        let publisher: Topic<u64> = match Topic::with_capacity(&name, CAPACITY, None) {
            Ok(t) => t,
            Err(e) => {
                eprintln!("skipping: shared memory unavailable ({e})");
                return;
            }
        };
        let subscriber: Topic<u64> = match Topic::with_capacity(&name, CAPACITY, None) {
            Ok(t) => t,
            Err(e) => {
                eprintln!("skipping: shared memory unavailable ({e})");
                return;
            }
        };
        let Some(path) = horus_sys::shm::topic_shm_path_checked(&name) else {
            eprintln!("skipping: cannot resolve topic path");
            return;
        };

        // Registers a consumer without taking anything: the ring now has
        // someone to protect, so backpressure is correct and the overflow is
        // dropped rather than overwritten.
        assert!(subscriber.recv().is_none(), "nothing has been sent yet");

        for i in 1..=(u64::from(CAPACITY) * 3) {
            publisher.send(i);
        }

        let slot = super::read_latest_slot_bytes(&path, 0).expect("the ring holds messages");
        let newest = u64::from_le_bytes(slot.payload[..8].try_into().expect("u64 payload"));
        assert_eq!(
            newest,
            u64::from(CAPACITY),
            "the ring holds messages 1..={CAPACITY} and nothing newer, so the \
             newest readable message is {CAPACITY}, not {newest}"
        );
        assert_eq!(
            slot.write_idx,
            u64::from(CAPACITY),
            "the reported position must name the message actually returned"
        );
        assert!(
            slot.messages_total > slot.write_idx,
            "messages_total ({}) counts the dropped sends too; that is the \
             difference this test exists to keep visible",
            slot.messages_total
        );
    }

    /// Streaming past several ring-fulls must yield every message, in order,
    /// exactly once.
    ///
    /// The contract test that guarded `topic echo` asked for 20 messages, which
    /// fits inside one ring — so it passed while echo replayed the same first
    /// ring-full forever on any topic that had published more than that. Over
    /// eight ring-fulls the difference is unmissable: a frozen ring hands back
    /// `1..=capacity` on repeat, with a backward jump of `capacity - 1` at every
    /// wrap.
    #[test]
    fn streaming_across_many_ring_fulls_yields_each_message_once() {
        const CAPACITY: u32 = 64;
        const BATCH: u64 = 16;
        const ROUNDS: u64 = 32; // 512 messages == 8 ring-fulls
        let name = format!("echo_many_wraps_{}", std::process::id());
        let topic: Topic<u64> = match Topic::with_capacity(&name, CAPACITY, None) {
            Ok(t) => t,
            Err(e) => {
                eprintln!("skipping: shared memory unavailable ({e})");
                return;
            }
        };
        let Some(path) = horus_sys::shm::topic_shm_path_checked(&name) else {
            eprintln!("skipping: cannot resolve topic path");
            return;
        };

        // The cursor bookkeeping `horus topic echo` does, verbatim.
        let mut cursor = 0u64;
        let mut seen: Vec<u64> = Vec::new();
        let mut next = 1u64;
        for _ in 0..ROUNDS {
            for _ in 0..BATCH {
                topic.send(next);
                next += 1;
            }
            let (slots, missed) = super::read_slots_since(&path, cursor, 4096);
            assert_eq!(
                missed, 0,
                "a {BATCH}-message batch cannot lap a {CAPACITY}-slot ring"
            );
            for slot in &slots {
                cursor = cursor.max(slot.write_idx);
                seen.push(u64::from_le_bytes(
                    slot.payload[..8].try_into().expect("u64 payload"),
                ));
            }
        }

        // The first poll deliberately hands back only the newest message rather
        // than replaying history, so the run starts at whatever was current then
        // and must be strictly consecutive from there.
        let start = seen.first().copied().expect("at least one message");
        let expected: Vec<u64> = (start..=(next - 1)).collect();
        assert_eq!(
            seen,
            expected,
            "expected {} consecutive messages from {start}; got {} with {} \
             distinct values (a ring that stopped being written repeats itself)",
            expected.len(),
            seen.len(),
            {
                let mut u = seen.clone();
                u.sort_unstable();
                u.dedup();
                u.len()
            }
        );
    }

    #[test]
    fn slot_reader_still_sees_new_messages_after_the_ring_wraps() {
        let name = format!("echo_wrap_test_{}", std::process::id());
        let topic: Topic<f64> = match Topic::new(&name) {
            Ok(t) => t,
            Err(e) => {
                eprintln!("skipping: shared memory unavailable ({e})");
                return;
            }
        };

        let Some(path) = horus_sys::shm::topic_shm_path_checked(&name) else {
            eprintln!("skipping: cannot resolve topic path");
            return;
        };

        // Fill well past any plausible ring capacity so the cursor has wrapped
        // several times over.
        for i in 0..1024 {
            topic.send(i as f64);
        }

        let mut last = super::read_latest_slot_bytes(&path, 0)
            .expect("a freshly written topic must yield a slot")
            .write_idx;

        // Every further send must be observable.
        let mut observed = 0;
        for i in 0..64 {
            topic.send(10_000.0 + i as f64);
            if let Some(slot) = super::read_latest_slot_bytes(&path, last) {
                assert!(
                    slot.write_idx > last,
                    "the freshness counter must advance monotonically past the \
                     ring capacity; got {} after {last}",
                    slot.write_idx
                );
                last = slot.write_idx;
                observed += 1;
            }
        }

        assert!(
            observed > 32,
            "after wrapping, only {observed}/64 sends were visible — the reader \
             is keying on a counter that stops at the ring capacity"
        );

        let _ = 1_u64.ms();
    }

    // ── LIVE-5 wave 2: a ring that nobody is draining ────────────────────────
    //
    // The first fix for "topic echo shows one message and then nothing" keyed
    // keep-last-N on `sub_count() == 0`. That gate is permanently false once
    // any subscriber has ever registered, because nothing ever took a
    // registration back — so the two tests below are the two ways a subscriber
    // stops draining while its registration stays on the books, and neither
    // was covered.

    /// Child half of `a_subscriber_killed_with_sigkill_does_not_freeze_the_topic`.
    ///
    /// `#[ignore]`d: it never runs in the ordinary suite. The parent runs it by
    /// name in a child process, and the point of a child process is that it can
    /// be `kill -9`'d — which is the only way to produce the state under test,
    /// a participant entry whose owner is gone and which therefore no `Drop`,
    /// no deregistration and no lease refresh will ever tidy up.
    #[test]
    #[ignore]
    fn ghost_subscriber_child() {
        let Ok(name) = std::env::var("HORUS_LIVE5_GHOST_TOPIC") else {
            return;
        };
        // The capacity has to match whatever created the region. The parent
        // test uses a small ring so "past the wrap" is cheap; a live
        // reproduction against a real node leaves it unset and gets the
        // `auto_capacity` default the node itself used.
        let opened = match std::env::var("HORUS_LIVE5_GHOST_CAPACITY")
            .ok()
            .and_then(|v| v.parse::<u32>().ok())
        {
            Some(capacity) => Topic::<u64>::with_capacity(&name, capacity, None),
            None => Topic::<u64>::new(&name),
        };
        let topic = opened.expect("open the parent's ring");
        // One recv is all it takes to register as a consumer. It takes nothing
        // else, ever — like a node that crashed on its first tick.
        let _ = topic.recv();
        std::thread::sleep(std::time::Duration::from_secs(60));
    }

    /// Ring capacity for the SIGKILL case. Small, so "past the ring" is cheap.
    const GHOST_CAPACITY: u32 = 64;

    /// The name the child process is registered under in this test binary.
    fn ghost_child_test_name() -> String {
        // `module_path!()` is crate-qualified; libtest's filter names are not.
        let path = module_path!()
            .split_once("::")
            .map(|(_, rest)| rest)
            .unwrap_or(module_path!());
        format!("{path}::ghost_subscriber_child")
    }

    /// A subscriber killed with SIGKILL must not freeze the topic for good.
    ///
    /// This is the original LIVE-5 report, reproduced against the first fix.
    /// A publisher at 100 Hz and a separate subscriber process; `kill -9` the
    /// subscriber and the header goes:
    ///
    /// ```text
    ///   head FROZEN at 4736, tail 4608, subs=1, messages_total 5155 … 28955
    ///   $ horus topic list   →  vfy_crash  9063  98.0 Hz  active
    ///   $ horus topic echo vfy_crash -n 100
    ///     [15:01:06.020] #1: …            ← payload timestamped 3m22s earlier
    ///     <nothing for the remaining 45 s>
    /// ```
    ///
    /// `subscriber_count` had one decrement in the whole tree, inside
    /// `register_role`, and it only fired if a brand-new participant happened
    /// to claim that exact expired slot. Nothing else ever gave a registration
    /// back, so `sub_count()` stayed at 1 for the life of the segment and the
    /// keep-last-N gate that was supposed to unfreeze the ring was switched off
    /// permanently.
    ///
    /// The `sub_count()` assertion is what pins this to the reaper rather than
    /// to the stall detector: the sends below all happen inside a millisecond,
    /// far inside any stall grace, so the only thing that can free the ring
    /// here is the dead participant being deregistered.
    #[test]
    fn a_subscriber_killed_with_sigkill_does_not_freeze_the_topic() {
        let name = format!("echo_sigkill_sub_{}", std::process::id());
        let publisher: Topic<u64> = match Topic::with_capacity(&name, GHOST_CAPACITY, None) {
            Ok(t) => t,
            Err(e) => {
                eprintln!("skipping: shared memory unavailable ({e})");
                return;
            }
        };
        let Some(path) = horus_sys::shm::topic_shm_path_checked(&name) else {
            eprintln!("skipping: cannot resolve topic path");
            return;
        };
        let header = publisher.ring.header();
        // The reaper will not touch a participant whose lease is still good, and
        // the default lease is 5 s of wall clock. Shorten it before the child
        // registers, so the child's lease is short too.
        header.set_lease_timeout_ms(150);

        // Register the publisher BEFORE the subscriber exists, which is the
        // ordering of every real system — the node is already running when
        // something subscribes to it, and it does not re-register afterwards.
        //
        // This is load-bearing for what the test proves. `register_role` does
        // decrement the counters when a *brand-new* participant happens to
        // claim the dead one's expired slot, so a publisher that first
        // registers after the subscriber has died cleans up the corpse on its
        // way in and the ring is freed by accident. That accident is the only
        // thing that ever retired a registration, and it does not happen to the
        // publisher that was there first.
        publisher.send(0);

        let Ok(exe) = std::env::current_exe() else {
            eprintln!("skipping: cannot locate the test binary");
            return;
        };
        let spawned = std::process::Command::new(exe)
            .args([
                "--exact",
                &ghost_child_test_name(),
                "--ignored",
                "--test-threads=1",
            ])
            .env("HORUS_LIVE5_GHOST_TOPIC", &name)
            .env("HORUS_LIVE5_GHOST_CAPACITY", GHOST_CAPACITY.to_string())
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .spawn();
        let mut child = match spawned {
            Ok(c) => c,
            Err(e) => {
                eprintln!("skipping: cannot spawn the subscriber process ({e})");
                return;
            }
        };

        let registered = {
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(20);
            loop {
                if header.sub_count() > 0 {
                    break true;
                }
                if std::time::Instant::now() >= deadline {
                    break false;
                }
                std::thread::sleep(std::time::Duration::from_millis(20));
            }
        };
        if !registered {
            let _ = child.kill();
            let _ = child.wait();
            panic!("the subscriber process never registered on '{name}'");
        }

        // SIGKILL, then reap the zombie: a zombie still answers `kill(pid, 0)`,
        // so a process that has not been waited for still reads as alive.
        let _ = child.kill();
        let _ = child.wait();
        // Outlive the lease the child registered with.
        std::thread::sleep(std::time::Duration::from_millis(250));

        const SENT: u64 = GHOST_CAPACITY as u64 * 3;
        for i in 1..=SENT {
            publisher.send(i);
        }

        // Read the ring before asserting anything, so whichever assertion fires
        // first still reports both halves of the evidence: what the reader can
        // see, and why.
        let slot = super::read_latest_slot_bytes(&path, 0)
            .expect("a topic with messages sent must yield a slot");
        let newest = u64::from_le_bytes(slot.payload[..8].try_into().expect("u64 payload"));

        assert_eq!(
            header.sub_count(),
            0,
            "the only subscriber was killed and reaped 250 ms ago, and its \
             registration is still on the books — meanwhile the ring is serving \
             message {newest} of {SENT}, backpressured against a `tail` that \
             belongs to a process which no longer exists"
        );
        assert_eq!(
            newest, SENT,
            "the ring is serving message {newest} of {SENT} — it is still \
             backpressured against a `tail` belonging to a process that no \
             longer exists, which is the whole of the original report"
        );
    }

    /// A subscriber that is alive and simply stops reading must not freeze the
    /// topic for good either.
    ///
    /// This is the case no liveness check can reach: the process is running,
    /// the handle exists, the registration is legitimate — it just never calls
    /// `recv` again. Deregistering the dead is not enough on its own, so the
    /// decision is settled by the ring instead of by the participant table:
    /// `tail` moves if and only if something was consumed, so a `tail` that has
    /// not moved for a whole lease timeout means nothing is being consumed.
    ///
    /// Both halves matter and both are asserted. Inside the grace the ring must
    /// still be backpressured — a consumer that is briefly behind is owed its
    /// messages, and a detector that fires immediately would just be silent
    /// data loss with extra steps. After the grace the newest message must be
    /// getting through.
    #[test]
    fn a_live_subscriber_that_stops_reading_does_not_freeze_the_topic_forever() {
        const CAPACITY: u32 = 64;
        const GRACE_MS: u64 = 300;
        let name = format!("echo_idle_sub_{}", std::process::id());
        let publisher: Topic<u64> = match Topic::with_capacity(&name, CAPACITY, None) {
            Ok(t) => t,
            Err(e) => {
                eprintln!("skipping: shared memory unavailable ({e})");
                return;
            }
        };
        let subscriber: Topic<u64> = match Topic::with_capacity(&name, CAPACITY, None) {
            Ok(t) => t,
            Err(e) => {
                eprintln!("skipping: shared memory unavailable ({e})");
                return;
            }
        };
        let Some(path) = horus_sys::shm::topic_shm_path_checked(&name) else {
            eprintln!("skipping: cannot resolve topic path");
            return;
        };
        let header = publisher.ring.header();
        // Phase 1 runs with the grace effectively disabled, rather than with a
        // 300 ms one it is assumed to outrun.
        //
        // The fill below used to run against the real grace, on the reasoning
        // that 128 in-process sends take under a millisecond. They do on an idle
        // machine. Under the full suite on a loaded box they do not, the stall
        // detector fires part-way through the fill, and the ring starts
        // overwriting — so the assertion that it is *still backpressured* fails,
        // reporting a scheduling delay as a backpressure defect. Sleep and
        // scheduling have floors, not ceilings; a test that assumes a ceiling is
        // asserting that the machine was idle.
        //
        // Setting the grace out of reach for phase 1 removes the race instead of
        // widening it, and phase 2 sets the short one it actually wants to
        // measure.
        header.set_lease_timeout_ms(u32::MAX);

        // Registers a consumer, in this very process, and never takes anything.
        assert!(subscriber.recv().is_none(), "nothing has been sent yet");

        // Fill the ring and overrun it. However long this takes, the grace
        // cannot elapse during it.
        for i in 1..=(u64::from(CAPACITY) * 2) {
            publisher.send(i);
        }
        let held = super::read_latest_slot_bytes(&path, 0).expect("the ring holds messages");
        let held_newest = u64::from_le_bytes(held.payload[..8].try_into().expect("u64 payload"));
        assert_eq!(
            held_newest,
            u64::from(CAPACITY),
            "a consumer that has been quiet for under a millisecond is owed its \
             messages; the ring must still be backpressured, not overwriting"
        );

        // Now let the grace pass with the ring still full and the subscriber
        // still registered, still alive, still not reading. This is the
        // measurement, so this is where the short grace goes on.
        header.set_lease_timeout_ms(GRACE_MS as u32);
        let mut next = u64::from(CAPACITY) * 2 + 1;
        let deadline = std::time::Instant::now() + std::time::Duration::from_millis(GRACE_MS + 400);
        while std::time::Instant::now() < deadline {
            publisher.send(next);
            next += 1;
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        let last_sent = next - 1;

        assert!(
            header.sub_count() >= 1,
            "the subscriber is still registered and still alive — this case is \
             not the reaper's, and if the registration went away the test is \
             measuring the wrong mechanism"
        );
        let slot = super::read_latest_slot_bytes(&path, 0).expect("the ring holds messages");
        let newest = u64::from_le_bytes(slot.payload[..8].try_into().expect("u64 payload"));
        assert_eq!(
            newest, last_sent,
            "{GRACE_MS} ms after the last message anyone took out of this ring, \
             it is still serving message {newest} of {last_sent} — a subscriber \
             that stopped reading has stopped the topic"
        );
    }
}

/// Every message still in the ring since `last_write_idx`, oldest first.
///
/// `read_latest_slot_bytes` returns only the newest slot, so a caller polling a
/// topic sees whatever happened to be there at each poll and nothing in
/// between. That is a sampler, and `horus topic echo` was built on it: on a
/// 40 Hz topic it printed one message in twelve seconds, because "is the newest
/// slot different from the one I last saw" is a question that skips everything
/// published while the caller was asleep.
///
/// Messages are addressed by `messages_total`, which is incremented on every
/// `send()` regardless of backend, so message N lives at slot `(N - 1) & mask`
/// until the ring laps it. This returns what is still recoverable and tells the
/// caller what is not: `missed` is the number of messages that were overwritten
/// before it got to them, which is real information on a slow reader rather
/// than a silent gap.
///
/// `max` bounds the work per call. Anything beyond it is not lost — it is
/// simply returned by the next call, because this resumes from
/// `last_write_idx` rather than from the head.
///
/// The returned count is messages the ring overwrote before this reader got to
/// them. That is real information on a slow reader; a silent hole in the
/// numbering is not.
pub fn read_slots_since(
    path: &std::path::Path,
    last_write_idx: u64,
    max: usize,
) -> (Vec<TopicSlotRead>, u64) {
    // The head first, from a single cheap read, so the loop below knows the
    // range before it starts opening slots.
    let Some(head) = read_latest_slot_bytes(path, last_write_idx) else {
        return (Vec::new(), 0);
    };
    let write_idx = head.write_idx;

    // First call: hand back just the newest, rather than replaying a full ring
    // of history the caller never asked for.
    if last_write_idx == 0 {
        return (vec![head], 0);
    }

    let available = write_idx.saturating_sub(last_write_idx);
    if available == 0 {
        return (Vec::new(), 0);
    }

    // Resume from where the caller left off, oldest first — not the newest
    // `max`. Those differ: with `--count 20` and 28 messages waiting, taking
    // the newest twenty delivers a run with an eight-message hole in the
    // middle, when what was asked for was twenty consecutive messages. Taking
    // the oldest twenty gives exactly that, and the rest arrive on the next
    // call.
    let want = available.min(max as u64);

    let mut out = Vec::with_capacity(want as usize);
    let mut lapped = 0u64;
    for n in (last_write_idx + 1)..=(last_write_idx + want) {
        if n == write_idx {
            out.push(head.clone());
        } else if let Some(slot) = read_one_slot(path, n) {
            out.push(slot);
        } else {
            // The producer overwrote this slot before we reached it. Count it
            // rather than leaving a hole in the caller's numbering.
            lapped += 1;
        }
    }
    (out, lapped)
}

/// One message by its `messages_total` ordinal, if it is still in the ring.
///
/// Shares every validation `read_latest_slot_bytes` performs — the magic check,
/// the power-of-two capacity and mask agreement, and the mapping-length checks —
/// because these files are read across namespaces and are not trusted input.
fn read_one_slot(path: &std::path::Path, ordinal: u64) -> Option<TopicSlotRead> {
    read_slot_inner(path, 0, Some(ordinal))
}

#[cfg(test)]
mod stored_type_name_tests {
    use super::type_name_as_stored;

    #[test]
    fn a_short_name_is_kept_whole() {
        assert_eq!(type_name_as_stored("CmdVel"), "CmdVel");
        assert_eq!(type_name_as_stored(""), "");
    }

    #[test]
    fn a_name_at_the_limit_is_kept_whole() {
        let exactly = "a".repeat(31);
        assert_eq!(type_name_as_stored(&exactly), exactly);
    }

    #[test]
    fn a_longer_name_is_cut_to_what_fits() {
        // The regression: a type whose name reaches the field width compared its
        // full name against the truncated one stored in the header, and was
        // reported as mismatching itself. Generic types reach it easily.
        let full = "ActionFeedback<CancelTopicFeedback>";
        assert_eq!(type_name_as_stored(full), "ActionFeedback<CancelTopicFeedb");
        assert_eq!(type_name_as_stored(full).len(), 31);
    }

    #[test]
    fn a_cut_never_lands_inside_a_character() {
        // A cut through a multi-byte character leaves invalid UTF-8, and
        // `type_name_str` renders that as the empty string — which reads as
        // "this topic has no type" everywhere it is displayed and compared.
        let name = format!("{}é", "a".repeat(30)); // 'é' straddles byte 30..32
        let stored = type_name_as_stored(&name);
        assert_eq!(stored, "a".repeat(30));
        std::str::from_utf8(stored.as_bytes()).unwrap();
    }
}
