//! # Topic - Universal Smart Detection IPC
//!
//! This module provides fully automatic backend detection for `Topic::new()`.
//! Users just call `send()`/`recv()` and the system auto-detects the optimal
//! shared-memory backend based on topology and access patterns.
//!
//! Every topic is SHM-backed (a backing file is created and `creator_pid` is
//! stamped at header init), so a cross-process consumer that joins later can
//! read published data immediately. All backends are cross-process:
//!
//! ## Detection Matrix
//!
//! | Backend | Latency | Detection Criteria |
//! |---------|---------|-------------------|
//! | FanoutShm | ~40ns | pubs>1, subs>1, !is_pod (broadcast) |
//! | PodShm | ~50ns | 1→many / 0→many, is_pod (broadcast) |
//! | MpscShm | ~65ns | pubs>1, subs<=1 |
//! | SpmcShm | ~70ns | multi-consumer, non-POD |
//! | SpscShm | ~85ns | pubs<=1, subs<=1 |
//!
//! ## Usage
//!
//! ```rust,ignore
//! use horus_core::communication::Topic;
//!
//! // Just this - backend auto-selected
//! let topic: Topic<Data> = Topic::new("sensor")?;
//! topic.send(data);
//! let msg = topic.recv();
//! ```
//!
//! ## Safety Model
//!
//! ### Thread-Ownership Contract
//!
//! Each `Topic<T>` instance must be used from exactly **one thread at a time**.
//! The type is `Send` — an owned handle may be moved to another thread, and
//! `Clone` hands the new thread its own independent state — but it is
//! deliberately **not** `Sync`: it holds `UnsafeCell`, raw pointers, and
//! `Cell`-based local state that no lock or atomic protects.
//!
//! This paragraph used to claim the type was `!Send` and `!Sync` "by design"
//! while hand-written `unsafe impl Sync` blocks on `Topic` and `RingTopic`
//! said the opposite, so safe code could share one `&Topic` across threads and
//! race the unsynchronised state with no `unsafe` of its own. Those impls are
//! gone; sharing one handle now requires an explicit lock
//! (`Mutex<Topic<T>>` — an `RwLock` read guard is NOT enough, it hands out
//! concurrent `&Topic`) or a per-thread `Clone`.
//!
//! This single-thread-per-instance contract enables:
//!
//! - **Unsynchronized local state**: `LocalState` uses `Cell` for cached head/tail,
//!   role tracking, and epoch caching — no atomic overhead on the hot path.
//! - **Backend dispatch via `UnsafeCell<fn(...)>`**: Function pointers for send/recv
//!   are swapped during migration. The single-thread contract guarantees no concurrent
//!   read/write of the function pointer.
//! - **`UnsafeCell<BackendStorage>`**: Backend enum is accessed without synchronization;
//!   migration (which changes it) can only happen from the owning thread.
//!
//! Multiple Topic instances on *different* threads can share the same underlying ring
//! buffer — the ring's internal atomics handle cross-thread synchronization.
//!
//! ### Lock-Free Ring Buffer Safety
//!
//! All ring buffers (SPSC, SPMC, MPSC, MPMC) are lock-free and use atomic
//! operations for coordination:
//!
//! - **SPSC**: Producer stores head with `Release`, consumer loads with `Acquire`.
//!   No CAS needed — single writer, single reader.
//! - **SPMC**: Producer stores head with `Release`. Multiple consumers CAS on tail
//!   with `AcqRel` to claim exclusive read slots.
//! - **MPSC**: Multiple producers CAS on head with per-slot sequence numbers
//!   (Lamport-style). Single consumer reads tail sequentially.
//! - **MPMC**: Both head (producers) and tail (consumers) use CAS with per-slot
//!   sequence numbers for full multi-writer, multi-reader coordination.
//!
//! ### `read_latest()` and the `T: Copy` Invariant
//!
//! `read_latest()` returns the most recent message without advancing the consumer
//! position. For multi-consumer backends (SPMC, MPMC), this creates a TOCTOU race:
//!
//! 1. `read_latest` loads `head` and computes the slot index for `head - 1`
//! 2. Between step 1 and reading the slot data, a consumer can:
//!    - CAS-advance tail past this slot
//!    - Call `assume_init_read()` (moving the value out)
//!    - For types with heap allocations, the `Drop` impl frees memory
//! 3. `read_latest` then reads freed memory → **use-after-free**
//!
//! The fix: multi-consumer `read_latest()` requires `T: Copy`. Copy types have no
//! `Drop` impl and no heap pointers — the bytes in the slot are always safe to
//! bitwise-copy regardless of whether a consumer has logically consumed the slot.
//!
//! Single-consumer backends (SPSC, MPSC) keep `T: Clone` because only one thread
//! ever reads from tail, so no concurrent consumption race exists.
//!
//! The Topic-level `read_latest()` requires `T: Copy` since the backend can be
//! any variant at runtime (migration can switch SPSC → SPMC).
//!
//! ### Drop Safety
//!
//! All ring buffer `Drop` impls iterate `[tail, head)` and drop initialized-but-
//! unconsumed messages. MPSC and MPMC additionally check per-slot sequence numbers
//! to handle the edge case where a producer panicked between CAS-claiming a slot
//! and completing the write (sequence not yet advanced → slot not fully written).
//!
//! ### Verification
//!
//! Safety is verified through:
//! - **94 unit tests** including cross-thread stress tests for all backend variants
//! - **16 loom exhaustive concurrency tests** that explore every possible thread
//!   interleaving for SPSC, SPMC, MPSC, and MPMC algorithms, including
//!   `read_latest` + `try_recv` races

// Image/PointCloud/DepthImage are large by design; returning them as error
// from try_send() avoids a Box allocation on the hot path.
#![allow(clippy::result_large_err)]

pub(crate) mod header;
pub(crate) mod local_state;
pub mod metrics;
pub(crate) mod migration;
/// Authoritative byte layout of a topic's SHM region, for out-of-crate readers
/// and writers (`horus_net`). Offsets are `offset_of!`-asserted against
/// `TopicHeader`, so drift is a build failure.
pub mod shm_layout;
use shm_layout as layout;
pub mod types;

// Per-path optimized backend modules
pub(crate) mod backend;
pub(crate) mod dispatch;
pub(crate) mod registry;
/// Shared seqlock ring protocol for drop-oldest (latest-wins) fanout.
pub(crate) mod seqlock;
/// Cross-process contention-free MPMC via SHM-backed SPSC matrix.
pub(crate) mod shm_fanout;

// Shared pool registry for all tensor topic extensions
pub(crate) mod pool_registry;

// Auto-managed tensor pool extensions
pub(crate) mod tensor_ext;

// TopicMessage trait — unified wire protocol for Topic<T>
pub mod topic_message;
pub use topic_message::TopicMessage;

// Legacy domain-specific topic wrappers (retained for tests, replaced by Topic<T: TopicMessage>)
mod depth_ext;
mod image_ext;
mod pointcloud_ext;

#[cfg(test)]
mod tests;

use std::borrow::Borrow;
use std::marker::PhantomData;
use std::mem;
use std::sync::atomic::{AtomicU64, AtomicU8, Ordering};
use std::sync::Arc;

use serde::{de::DeserializeOwned, Serialize};

use crate::communication::pod::is_pod;
use crate::error::{HorusError, HorusResult};
use crate::memory::shm_region::ShmRegion;
use crate::memory::simd::{simd_copy_from_shm, simd_copy_to_shm, SIMD_COPY_THRESHOLD};
use crate::utils::unlikely;

// ============================================================================
// Topic Lifecycle Hook — used by horus_net for network replication
// ============================================================================

/// Topic lifecycle event for network registration.
#[derive(Debug, Clone)]
pub enum TopicLifecycleEvent {
    /// A topic was created. Fields: name, type_name_hash, type_size, is_pod.
    Created {
        name: String,
        type_name_hash: u32,
        type_size: u32,
        is_pod: bool,
    },
    /// A handle was first used in one direction: `publisher` is true on its
    /// first send, false on its first recv.
    ///
    /// `Created` cannot carry this. A `Topic<T>` handle can both send and
    /// receive, so at construction the direction is genuinely unknown — which
    /// is why the network layer used to record every topic as "both" and lose
    /// the distinction its import guard depends on. First use is the earliest
    /// moment the answer exists.
    RoleObserved {
        name: String,
        publisher: bool,
        type_name_hash: u32,
        type_size: u32,
        is_pod: bool,
    },
    /// A topic was dropped.
    Dropped { name: String },
}

type TopicLifecycleHook = Box<dyn Fn(TopicLifecycleEvent) + Send + Sync>;

static TOPIC_LIFECYCLE_HOOK: std::sync::OnceLock<TopicLifecycleHook> = std::sync::OnceLock::new();

/// Topics that exist right now, so a hook installed later can be told about
/// them.
///
/// The hook is a `OnceLock` set when the replicator starts, and the replicator
/// starts from `scheduler.run()` — by which time every node has built its topics
/// in `init()` or in its constructor. Those `Created` events fired into a hook
/// that did not exist yet, so the network registry began life empty and stayed
/// that way for exactly the topics a robot actually uses. Replaying on install
/// closes that window without changing when anything else happens.
///
/// Handles are refcounted per name: one process routinely opens the same topic
/// from several nodes, and the network layer only cares that the topic exists,
/// not how many handles hold it. Dropping one handle must not erase a topic the
/// others are still using.
struct LiveTopic {
    created: TopicLifecycleEvent,
    /// Handles currently open on this name.
    handles: usize,
    /// Role events already seen, replayed to a late hook after `created`.
    roles: Vec<TopicLifecycleEvent>,
}

type LiveTopics = std::collections::HashMap<String, LiveTopic>;

static LIVE_TOPICS: std::sync::Mutex<Option<LiveTopics>> = std::sync::Mutex::new(None);

/// Set a global hook called when topics are created or dropped.
///
/// Called by horus_net at replicator startup to populate its TopicRegistry.
/// Can only be set once (first caller wins).
///
/// Topics that already exist are replayed to the hook as `Created` events
/// before this returns, so a late-installed hook sees the same set it would
/// have seen had it been installed first.
pub fn set_topic_lifecycle_hook(hook: impl Fn(TopicLifecycleEvent) + Send + Sync + 'static) {
    let boxed: TopicLifecycleHook = Box::new(hook);
    if TOPIC_LIFECYCLE_HOOK.set(boxed).is_err() {
        // Not the first caller; the existing hook keeps ownership of the stream.
        return;
    }
    // Snapshot and release before calling out: the hook is arbitrary user code
    // and may itself create a topic, which would re-enter this lock.
    let existing: Vec<TopicLifecycleEvent> = match LIVE_TOPICS.lock() {
        Ok(guard) => guard
            .as_ref()
            .map(|live| {
                live.values()
                    .flat_map(|t| {
                        // `Created` first: a hook may key its own state on it.
                        std::iter::once(t.created.clone()).chain(t.roles.iter().cloned())
                    })
                    .collect()
            })
            .unwrap_or_default(),
        Err(_) => Vec::new(),
    };
    if let Some(hook) = TOPIC_LIFECYCLE_HOOK.get() {
        for event in existing {
            hook(event);
        }
    }
}

fn notify_topic_lifecycle(event: TopicLifecycleEvent) {
    // Record first, so the set stays accurate whether or not a hook is
    // installed yet.
    if let Ok(mut guard) = LIVE_TOPICS.lock() {
        let live = guard.get_or_insert_with(LiveTopics::new);
        match &event {
            TopicLifecycleEvent::Created { name, .. } => {
                live.entry(name.clone())
                    .and_modify(|t| t.handles += 1)
                    .or_insert_with(|| LiveTopic {
                        created: event.clone(),
                        handles: 1,
                        roles: Vec::new(),
                    });
            }
            TopicLifecycleEvent::RoleObserved {
                name, publisher, ..
            } => {
                if let Some(t) = live.get_mut(name) {
                    // At most two entries per name — publisher and subscriber —
                    // however many handles observe them.
                    let already = t.roles.iter().any(|e| {
                        matches!(e, TopicLifecycleEvent::RoleObserved { publisher: p, .. } if p == publisher)
                    });
                    if !already {
                        t.roles.push(event.clone());
                    }
                }
            }
            TopicLifecycleEvent::Dropped { name } => {
                if let Some(t) = live.get_mut(name) {
                    t.handles -= 1;
                    if t.handles == 0 {
                        live.remove(name);
                    }
                }
            }
        }
    }
    if let Some(hook) = TOPIC_LIFECYCLE_HOOK.get() {
        hook(event);
    }
}

// ─── Topic-Node Registry (automatic pub/sub discovery) ───────────────────────

use std::collections::HashMap;
use std::sync::{OnceLock, RwLock};

/// Role of a node with respect to a topic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeTopicRole {
    Publisher,
    Subscriber,
    Both,
}

/// A single topic↔node association.
#[derive(Debug, Clone)]
pub struct TopicAssociation {
    pub node_name: String,
    pub role: NodeTopicRole,
    pub type_name: String,
}

/// Global registry mapping topics to the nodes that use them.
///
/// Automatically populated by `Topic::new()` using the thread-local
/// `CURRENT_NODE` context set by the scheduler before each tick.
/// This is how `horus node info` knows which topics a node publishes,
/// and how `horus topic info` knows which nodes publish a topic —
/// without users ever declaring anything.
pub struct TopicNodeRegistry {
    /// topic_name → Vec<(node_name, role)>
    topics: RwLock<HashMap<String, Vec<TopicAssociation>>>,
    /// Monotonic version counter, bumped on every register/unregister.
    /// The scheduler uses this to detect topology changes and rebuild
    /// the dependency graph when needed.
    version: AtomicU64,
}

static GLOBAL_TOPIC_NODE_REGISTRY: OnceLock<TopicNodeRegistry> = OnceLock::new();

/// Get the global TopicNodeRegistry singleton.
pub fn topic_node_registry() -> &'static TopicNodeRegistry {
    GLOBAL_TOPIC_NODE_REGISTRY.get_or_init(TopicNodeRegistry::new)
}

impl TopicNodeRegistry {
    fn new() -> Self {
        Self {
            topics: RwLock::new(HashMap::new()),
            version: AtomicU64::new(0),
        }
    }

    /// Current topology version. Bumped on every register/unregister.
    /// The scheduler compares this to its last-built version to know
    /// whether the dependency graph needs rebuilding.
    pub fn version(&self) -> u64 {
        self.version.load(Ordering::Acquire)
    }

    /// Register that a node uses a topic with the given role.
    /// Called automatically by Topic::new() during a node's tick.
    pub fn register(&self, topic_name: &str, node_name: &str, role: NodeTopicRole) {
        self.register_with_type(topic_name, node_name, role, "");
    }

    pub fn register_with_type(
        &self,
        topic_name: &str,
        node_name: &str,
        role: NodeTopicRole,
        type_name: &str,
    ) {
        let mut topics = self.topics.write().unwrap_or_else(|e| e.into_inner());
        let entries = topics.entry(topic_name.to_string()).or_default();

        // Check if this node already registered for this topic
        if let Some(existing) = entries.iter_mut().find(|a| a.node_name == node_name) {
            // Upgrade role: Publisher + Subscriber = Both
            match (existing.role, role) {
                (NodeTopicRole::Publisher, NodeTopicRole::Subscriber)
                | (NodeTopicRole::Subscriber, NodeTopicRole::Publisher) => {
                    existing.role = NodeTopicRole::Both;
                    self.version.fetch_add(1, Ordering::Release);
                }
                _ => {} // Same role or already Both
            }
        } else {
            entries.push(TopicAssociation {
                node_name: node_name.to_string(),
                role,
                type_name: type_name.to_string(),
            });
            self.version.fetch_add(1, Ordering::Release);
        }
    }

    /// Unregister a node from a topic.
    /// Called on Topic::drop().
    pub fn unregister(&self, topic_name: &str, node_name: &str) {
        let mut topics = self.topics.write().unwrap_or_else(|e| e.into_inner());
        if let Some(entries) = topics.get_mut(topic_name) {
            let before = entries.len();
            entries.retain(|a| a.node_name != node_name);
            if entries.len() != before {
                self.version.fetch_add(1, Ordering::Release);
            }
            if entries.is_empty() {
                topics.remove(topic_name);
            }
        }
    }

    /// Get all topics published by a given node.
    pub fn publishers_for_node(&self, node_name: &str) -> Vec<crate::core::TopicMetadata> {
        let topics = self.topics.read().unwrap_or_else(|e| e.into_inner());
        let mut result = Vec::new();
        for (topic_name, entries) in topics.iter() {
            for assoc in entries {
                if assoc.node_name == node_name
                    && matches!(assoc.role, NodeTopicRole::Publisher | NodeTopicRole::Both)
                {
                    result.push(crate::core::TopicMetadata {
                        topic_name: topic_name.clone(),
                        type_name: assoc.type_name.clone(),
                    });
                }
            }
        }
        result
    }

    /// Get all topics subscribed to by a given node.
    pub fn subscribers_for_node(&self, node_name: &str) -> Vec<crate::core::TopicMetadata> {
        let topics = self.topics.read().unwrap_or_else(|e| e.into_inner());
        let mut result = Vec::new();
        for (topic_name, entries) in topics.iter() {
            for assoc in entries {
                if assoc.node_name == node_name
                    && matches!(assoc.role, NodeTopicRole::Subscriber | NodeTopicRole::Both)
                {
                    result.push(crate::core::TopicMetadata {
                        topic_name: topic_name.clone(),
                        type_name: assoc.type_name.clone(),
                    });
                }
            }
        }
        result
    }

    /// Get all nodes that publish to a given topic.
    pub fn publishers_of_topic(&self, topic_name: &str) -> Vec<String> {
        let topics = self.topics.read().unwrap_or_else(|e| e.into_inner());
        topics
            .get(topic_name)
            .map(|entries| {
                entries
                    .iter()
                    .filter(|a| matches!(a.role, NodeTopicRole::Publisher | NodeTopicRole::Both))
                    .map(|a| a.node_name.clone())
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Get all nodes that subscribe to a given topic.
    pub fn subscribers_of_topic(&self, topic_name: &str) -> Vec<String> {
        let topics = self.topics.read().unwrap_or_else(|e| e.into_inner());
        topics
            .get(topic_name)
            .map(|entries| {
                entries
                    .iter()
                    .filter(|a| matches!(a.role, NodeTopicRole::Subscriber | NodeTopicRole::Both))
                    .map(|a| a.node_name.clone())
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Get all registered topics with their associations.
    pub fn all_topics(&self) -> Vec<(String, Vec<TopicAssociation>)> {
        let topics = self.topics.read().unwrap_or_else(|e| e.into_inner());
        topics.iter().map(|(k, v)| (k.clone(), v.clone())).collect()
    }
}

// ─── Utility ─────────────────────────────────────────────────────────────────

/// FNV-1a over a byte string, usable in a `const` context.
///
/// `message!` needs the hash at compile time so it can be an associated const,
/// and the runtime helper below cannot be called from one.
pub const fn const_fnv1a(bytes: &[u8]) -> u32 {
    let mut hash: u32 = 2166136261;
    let mut i = 0;
    while i < bytes.len() {
        hash ^= bytes[i] as u32;
        hash = hash.wrapping_mul(16777619);
        i += 1;
    }
    hash
}

/// FNV-1a hash (32-bit) for type names. Same algorithm as horus_net::wire::fnv1a_hash.
pub(crate) fn fnv1a_type_hash(s: &str) -> u32 {
    let mut hash: u32 = 2166136261;
    for &byte in s.as_bytes() {
        hash ^= byte as u32;
        hash = hash.wrapping_mul(16777619);
    }
    hash
}

/// Check if a process is still alive.
///
/// Delegates to `horus_sys`, the platform abstraction layer: `kill(pid, 0)` on
/// Unix, `OpenProcess` on Windows. `libc::kill` does not exist on Windows, so
/// calling it directly here broke the Windows build.
fn is_process_alive(pid: u32) -> bool {
    horus_sys::discover::is_process_alive(pid)
}

pub(crate) use header::TopicHeader;
pub(crate) use header::{TOPIC_MAGIC, TOPIC_VERSION};

// Public debug flag API for external tools (TUI monitor)
#[doc(hidden)]
pub use header::{
    peek_topic_type_name, read_latest_slot_bytes, read_slots_since, read_topic_header_info,
    read_topic_messages_total, read_topic_sequence, set_topic_verbose, TopicHeaderInfo, TopicKind,
    TopicSlotRead, TOPIC_VERBOSE_OFFSET,
};
use local_state::LocalState;
pub(crate) use metrics::MigrationMetrics;
pub use metrics::TopicMetrics;

/// Bounded spin iterations for waiting on per-slot ready flags.
///
/// When a multi-producer path does `fetch_add` on head to claim a slot, there's a
/// brief window before the ready flag is written. Consumers spin for up to this many
pub(crate) use migration::{BackendMigrator, MigrationResult};
pub use types::TopicDescriptor;
pub(crate) use types::{BackendMode, ConnectionState, TopicRole};

use header::current_time_ms;
use local_state::{DEFAULT_SLOT_SIZE, EPOCH_CHECK_INTERVAL};

use backend::BackendStorage;

// ============================================================================
// SendBlockingError — returned when send_blocking() cannot deliver
// ============================================================================

/// Error returned by [`Topic::send_blocking`] when the message cannot be delivered.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum SendBlockingError {
    /// The ring buffer remained full for the entire timeout duration.
    #[error("send_blocking timed out: ring buffer full")]
    Timeout,
    /// This topic's backend cannot apply backpressure, so delivery cannot be
    /// guaranteed and waiting would be meaningless.
    ///
    /// `PodShm` broadcast overwrites the oldest unconsumed slot unconditionally
    /// and its send has no failure path, so there is nothing to wait for: the
    /// message is written and may be overwritten again before any subscriber
    /// reads it. Returning this is deliberate. `send_blocking` is documented for
    /// emergency stop and motor setpoints, and reporting success for a delivery
    /// nobody guaranteed is worse on that path than refusing loudly.
    #[error(
        "send_blocking cannot guarantee delivery on a broadcast backend: it \
         overwrites unconsumed slots and never reports a full ring. Migrate the \
         topic to a backpressured backend (MpscShm, SpscShm or SpmcShm), or use \
         send()/try_send() and accept the documented loss. Not FanoutShm: it is \
         the other broadcast backend and is drop-oldest too."
    )]
    NoBackpressure,
}

/// Whether the backend named by [`Topic::backend_name`] refuses a send when its
/// ring is full.
///
/// Keyed by NAME rather than by the private `BackendMode`, because the bindings
/// are where this question actually gets asked and a name is all they can see:
/// `backend_type` is a string over both the Python and C++ FFI. Answering it
/// there by re-deriving the mapping is how a binding drifts from the runtime, so
/// `provides_backpressure()` is defined in terms of this too and the two cannot
/// disagree.
///
/// - `SpscShm`, `MpscShm`, `SpmcShm` — ring-full is a real `Err`: the send paths
///   compare the claimed sequence against `header.tail` and refuse.
/// - `PodShm`, `FanoutShm` — broadcast, overwrite-oldest by construction, so
///   there is no full condition to report.
/// - Anything else, including `Unknown`, fails closed. `Unknown` is the state
///   EVERY topic is in until its first send or recv — including at the "assert
///   this at startup" moment the doc comment above recommends. Answering `true`
///   there made the assertion pass on a topic that was about to resolve to
///   `PodShm` and silently drop, so the check a caller writes to catch a lossy
///   e-stop channel could not catch one. An unresolved backend is not a promise
///   of backpressure.
pub fn backend_provides_backpressure(backend_name: &str) -> bool {
    matches!(backend_name, "SpscShm" | "MpscShm" | "SpmcShm")
}

impl From<SendBlockingError> for crate::error::HorusError {
    fn from(err: SendBlockingError) -> Self {
        crate::error::HorusError::Communication(crate::error::CommunicationError::TopicFull {
            topic: err.to_string(),
        })
    }
}

// ============================================================================
// SIMD-Aware Copy Helpers
// ============================================================================

/// Write a message to a ring buffer slot, using SIMD streaming stores for large POD types.
///
/// Byte copy, never `ptr::write::<T>`, for the same reason
/// `simd_aware_read_uninit` reads bytes rather than a `T`: a slot is untyped
/// shared memory at an address the *layout* picked, not memory the compiler
/// aligned for `T`. `colo_eligible` now keeps types stricter than
/// `COLO_PAYLOAD_OFF` off the colo geometry, but the split geometry is not
/// aligned either — its data region starts at `640 + capacity * 8`, which is
/// 8 mod 16 for `capacity == 1` — and rounding that up would move an offset
/// `horus_net`'s `ShmRingWriter` and `header::read_slot_inner` each compute
/// independently. A typed store is what turned that into a fault: LLVM
/// vectorised `ptr::write` of a 16-byte `#[repr(align(16))]` message into
/// `movaps %xmm0,(%rax)` and the first `send` took SIGSEGV in release while
/// passing in debug.
///
/// `mem::forget` drops nothing that needed dropping: every caller is a
/// `dispatch::send_shm_*_pod` path, and `is_pod` implies `!needs_drop::<T>()`.
///
/// # Safety
/// `dst` must be valid for writes of `size_of::<T>()` bytes. Alignment is NOT
/// required.
#[inline(always)]
unsafe fn simd_aware_write<T>(dst: *mut T, msg: T) {
    if mem::size_of::<T>() >= SIMD_COPY_THRESHOLD {
        simd_copy_to_shm(
            &msg as *const T as *const u8,
            dst as *mut u8,
            mem::size_of::<T>(),
        );
    } else {
        std::ptr::copy_nonoverlapping(
            &msg as *const T as *const u8,
            dst as *mut u8,
            mem::size_of::<T>(),
        );
    }
    mem::forget(msg);
}

/// Read a message from a ring buffer slot, using SIMD prefetched reads for large POD types.
///
/// # Safety
/// `src` must be valid for reads of `size_of::<T>()` bytes, and must hold a
/// valid `T`. Alignment is not required — the copy underneath is byte-wise.
#[inline(always)]
unsafe fn simd_aware_read<T>(src: *const T) -> T {
    simd_aware_read_uninit(src).assume_init()
}

/// Copy a slot's bytes out WITHOUT asserting they are a valid `T`.
///
/// The seqlock consumers copy a slot that a lapping producer may be
/// overwriting, then re-check the slot stamp and discard the copy if it was.
/// Discarding is not enough on its own: producing a `T` from torn bytes is
/// already undefined behaviour at the moment of materialisation — a `bool` that
/// reads back as `0x02`, or an enum discriminant out of range, is UB before any
/// re-check can reject it (see the `is_pod` heuristic in `communication::pod`,
/// which admits structs containing such fields). Keeping the copy as
/// `MaybeUninit<T>` until *after* the stamp re-check closes that window: torn
/// bytes are dropped as raw memory and never become a `T`.
///
/// # Safety
/// `src` must be valid for reads of `size_of::<T>()` bytes. Alignment is not
/// required — the copy is byte-wise. The caller must only `assume_init()` the
/// result once it has established that the bytes are a valid, fully-written
/// `T`.
#[inline(always)]
unsafe fn simd_aware_read_uninit<T>(src: *const T) -> mem::MaybeUninit<T> {
    let mut msg = mem::MaybeUninit::<T>::uninit();
    if mem::size_of::<T>() >= SIMD_COPY_THRESHOLD {
        simd_copy_from_shm(
            src as *const u8,
            msg.as_mut_ptr() as *mut u8,
            mem::size_of::<T>(),
        );
    } else {
        std::ptr::copy_nonoverlapping(
            src as *const u8,
            msg.as_mut_ptr() as *mut u8,
            mem::size_of::<T>(),
        );
    }
    msg
}

// ============================================================================
// Capacity Calculation
// ============================================================================

/// System page size for memory-aligned buffer calculations
const PAGE_SIZE: usize = 4096;

/// Minimum ring buffer capacity (ensures reasonable buffering for small messages)
const MIN_CAPACITY: u32 = 16;

/// Maximum ring buffer capacity (prevents excessive memory usage)
const MAX_CAPACITY: u32 = 1024;

/// Calculate optimal ring buffer capacity based on message type size.
#[inline]
fn auto_capacity<T>() -> u32 {
    let type_size = mem::size_of::<T>();
    if type_size == 0 {
        return MIN_CAPACITY;
    }
    let calculated = (PAGE_SIZE / type_size) as u32;
    calculated
        .clamp(MIN_CAPACITY, MAX_CAPACITY)
        .next_power_of_two()
}

// ============================================================================
// Topics Macro
// ============================================================================

/// Define type-safe topic descriptors for compile-time checked topic names.
#[macro_export]
macro_rules! topics {
    ($($vis:vis $name:ident : $type:ty = $topic_name:expr),* $(,)?) => {
        $(
            $vis const $name: $crate::communication::TopicDescriptor<$type> =
                $crate::communication::TopicDescriptor::new($topic_name);
        )*
    };
}

// ============================================================================
// Topic - Main Public API
// ============================================================================

/// RingTopic - Internal ring buffer with smart detection IPC
///
/// `RingTopic<T>` provides fully automatic backend detection. Users just call
/// `send()`/`recv()` and the system auto-detects the optimal backend from 10 paths
/// based on topology and access patterns.
///
/// This is the internal ring buffer type. Users should use `Topic<T>` which
/// wraps this with the `TopicMessage` conversion layer.
pub(crate) struct RingTopic<T> {
    /// Topic name
    name: String,

    /// Shared memory region containing the header and data
    storage: Arc<ShmRegion>,

    /// SHM-backed ring for this topic's data plane (every topic is SHM-backed).
    backend: std::cell::UnsafeCell<BackendStorage<T>>,

    /// Function pointer for try_send dispatch — set by initialize_backend(),
    /// eliminates all runtime match chains on the hot path.
    send_fn: std::cell::UnsafeCell<dispatch::SendFn<T>>,

    /// Function pointer for try_recv dispatch — set by initialize_backend(),
    /// eliminates all runtime match chains on the hot path.
    recv_fn: std::cell::UnsafeCell<dispatch::RecvFn<T>>,

    /// Local state (role, cached epoch, etc.)
    local: std::cell::UnsafeCell<LocalState>,

    /// Process-local epoch notification — shared with all same-name Topics
    /// in this process. Checked on every send/recv (~1ns L1 heap read)
    /// instead of reading migration_epoch from SHM mmap (~20ns).
    process_epoch: Arc<std::sync::atomic::AtomicU64>,

    /// Metrics for monitoring
    metrics: Arc<MigrationMetrics>,

    /// Raw pointer to the SHM TopicHeader — always valid for the topic's
    /// lifetime (backed by `storage` Arc). Used for the runtime debug flag
    /// check which must work regardless of backend/migration state.
    ///
    /// Wrapped in `Cell` because `auto_grow_slot_size` and `handle_epoch_change`
    /// update the pointer through `&self` when the mmap is regrown. Without
    /// `Cell`, writes via `addr_of!()` cast to `*mut` would be UB under Rust's
    /// aliasing rules (the compiler may legally cache the old value).
    header_ptr: std::cell::Cell<*const TopicHeader>,

    /// Connection state (for network backend compatibility)
    state: AtomicU8,

    /// Lazy-initialized TensorPool for spilling large serde messages.
    ///
    /// `None` until the first message exceeds `SPILL_THRESHOLD`, at which point
    /// a pool is created via `pool_registry::get_or_create_pool(&self.name)`.
    /// Pool-backed types (Image, PointCloud) have their own pool via `Topic<T>::pool`;
    /// this is only for serde types that occasionally send large messages.
    spill_pool: std::cell::UnsafeCell<Option<Arc<TensorPool>>>,

    /// Type marker
    _marker: PhantomData<T>,
}

// SAFETY (Send): Each RingTopic instance has its own UnsafeCell state (backend,
// send_fn, recv_fn, local) that is independent after Clone. Moving a RingTopic to
// another thread transfers exclusive ownership of its per-instance state.
unsafe impl<T: Send> Send for RingTopic<T> {}

// There is deliberately NO `Sync` impl here.
//
// There used to be one, justified as "required so Arc<Topic<T>> can be Send",
// with an INVARIANT comment stating that callers must not share a single
// `&RingTopic` across threads — an invariant the impl itself made
// unenforceable. `send`/`recv`/`try_send`/`try_recv` all take `&self` and then
// read-modify-write `local.local_head`, `local.local_tail`, `local.msg_counter`
// and the cached pointers behind `UnsafeCell`, none of it synchronised, so two
// threads holding `&Topic` from one `Arc` raced on plain fields — a data race,
// UB, with no `unsafe` anywhere in the caller. The `Cell`/`UnsafeCell` fields
// make both types `!Sync` on their own; letting them do so is the fix.
//
// To share one handle, wrap it in an EXCLUSIVE lock (`Mutex<Topic<T>>`; an
// `RwLock` read guard hands out concurrent `&Topic` and does not satisfy this),
// or give each thread its own `Clone`, which gets independent local state.

#[allow(private_interfaces)]
/// Where a consumer's read position lands after a migration resync.
///
/// `header.tail` is a single shared value. On a broadcast backend each consumer
/// keeps its own independent position and the shared tail trails the slowest of
/// them, so adopting it wholesale moved every consumer that was ahead
/// *backward* — and its next `recv()` returned an older sequence than one it had
/// already delivered. Measured by
/// `recv_never_reorders_or_duplicates_when_lapped`: 5 inversions across
/// 1,296,457 messages, on 3 of 16 consumers, at migration boundaries —
/// "consumer 6: 23929 then 23497 (backward by 432)".
///
/// Take the later of the two, then clamp to the head: if the ring itself
/// restarted, the head comes back smaller than our stale tail and the clamp puts
/// us at "nothing to read yet" rather than stranding us beyond the producer
/// forever. On a single-consumer backend the shared tail is this consumer's own
/// position, so the max is a no-op.
///
/// The guard applies only once something has been delivered. Before that there
/// is no ordering to preserve, and refusing to move backward would strand a
/// just-registered handle ahead of messages published while it was joining —
/// the late-join adjustment can leave `local_tail` well past them.
fn resynced_tail(local_tail: u64, shared_tail: u64, new_head: u64, delivered: bool) -> u64 {
    if !delivered {
        // Nothing has been handed to this consumer yet, so there is no ordering
        // to preserve and no message to re-deliver. Adopting the shared position
        // is what lets a handle that registered moments ago still see what was
        // published while it was joining.
        return shared_tail.min(new_head);
    }
    local_tail.max(shared_tail).min(new_head)
}

/// Drop module qualifiers from a type name, keeping its structure.
///
/// This was `rsplit("::").next()`, which is right for a plain type and wrong
/// for a generic one: `horus_core::services::ServiceResponse<pkg::AddTwoIntsResponse>`
/// has `AddTwoIntsResponse>` as its last `::`-separated segment, so that is
/// what went into the header and what `horus topic info` printed — a name with
/// a stray closing bracket and no mention of the wrapper it is actually inside.
/// Every service topic showed one.
///
/// Qualifiers are stripped per segment instead, so the same input yields
/// `ServiceResponse<AddTwoIntsResponse>`.
fn strip_module_paths(full: &str) -> String {
    let mut out = String::with_capacity(full.len());
    let mut segment = String::new();
    for c in full.chars() {
        // Anything that ends an identifier: generics, tuples, arrays, refs.
        if matches!(c, '<' | '>' | ',' | ' ' | '(' | ')' | '[' | ']' | '&' | ';') {
            out.push_str(segment.rsplit("::").next().unwrap_or(&segment));
            segment.clear();
            out.push(c);
        } else {
            segment.push(c);
        }
    }
    out.push_str(segment.rsplit("::").next().unwrap_or(&segment));
    out
}

impl<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static> RingTopic<T> {
    /// Header size in shared memory
    const HEADER_SIZE: usize = mem::size_of::<TopicHeader>();

    /// Create a new topic with auto-sized ring buffer capacity.
    ///
    /// # Errors
    ///
    /// - [`ValidationError`] — topic name is empty, too long, or contains invalid characters
    /// - [`MemoryError::ShmCreateFailed`] — shared memory region could not be created
    /// - [`CommunicationError::TopicCreationFailed`] — ring buffer setup failed
    pub fn new(name: impl Into<String>) -> HorusResult<Self> {
        let name = name.into();
        Self::with_capacity_and_kind(&name, auto_capacity::<T>(), None, TopicKind::Data as u8)
    }

    /// Publish this handle's layout hash, or fail if it contradicts one already
    /// recorded for the topic.
    ///
    /// First writer wins: the hash is installed with a compare-exchange from 0,
    /// so whichever handle arrives first records it and every later handle is
    /// checked against it.
    pub(crate) fn bind_layout_hash(&self, layout_hash: u32) -> HorusResult<()> {
        if layout_hash == 0 {
            return Ok(());
        }
        let header = self.header();
        match header.layout_hash.compare_exchange(
            0,
            layout_hash,
            Ordering::AcqRel,
            Ordering::Acquire,
        ) {
            // We installed it, or it was already ours.
            Ok(_) => Ok(()),
            Err(existing) if existing == layout_hash => Ok(()),
            Err(existing) => Err(HorusError::Communication(
                crate::error::CommunicationError::TopicCreationFailed {
                    topic: self.name.clone(),
                    reason: format!(
                        "message layout mismatch. This build's \
                     '{}' hashes to {:#010x}, but the topic was opened with \
                     {:#010x}.\n\
                     The type name and size match, so only the field layout \
                     differs — two builds of the same message that reordered, \
                     renamed or retyped a field. Reading it would silently \
                     reinterpret the bytes rather than fail.\n\
                     Fix: rebuild both sides against the same message \
                     definition.",
                        std::any::type_name::<T>(),
                        layout_hash,
                        existing
                    ),
                },
            )),
        }
    }

    /// Create a new topic with a specific kind (Data, ServiceRequest, etc.).
    pub fn new_with_kind(name: impl Into<String>, topic_kind: u8) -> HorusResult<Self> {
        let name = name.into();
        Self::with_capacity_and_kind(&name, auto_capacity::<T>(), None, topic_kind)
    }

    /// Create a new topic with custom capacity and optional slot size.
    ///
    /// # Errors
    ///
    /// - [`ValidationError`] — name empty/too long, capacity not power-of-two, slot > 1MB
    /// - [`MemoryError::ShmCreateFailed`] — shared memory region could not be created
    pub fn with_capacity(name: &str, capacity: u32, slot_size: Option<usize>) -> HorusResult<Self> {
        Self::with_capacity_and_kind(name, capacity, slot_size, TopicKind::Data as u8)
    }

    /// Create a new topic with custom capacity, slot size, and topic kind.
    pub fn with_capacity_and_kind(
        name: &str,
        capacity: u32,
        slot_size: Option<usize>,
        topic_kind: u8,
    ) -> HorusResult<Self> {
        // Validate topic name
        if name.is_empty() {
            return Err(crate::HorusError::InvalidInput(
                crate::error::ValidationError::Other("Topic name cannot be empty".to_string()),
            ));
        }
        if name.len() > 255 {
            return Err(crate::HorusError::InvalidInput(
                crate::error::ValidationError::Other(format!(
                    "Topic name too long ({} chars, max 255)",
                    name.len()
                )),
            ));
        }
        if !name
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'/' | b'-' | b'.'))
        {
            return Err(crate::HorusError::InvalidInput(
                crate::error::ValidationError::Other(format!(
                    "Topic name '{}' contains invalid characters \
                     (allowed: alphanumeric, _, /, -, .)",
                    name
                )),
            ));
        }

        // Validate capacity
        if capacity == 0 {
            return Err(crate::HorusError::InvalidInput(
                crate::error::ValidationError::Other("Topic capacity must be >= 1".to_string()),
            ));
        }

        // Validate slot size
        const MAX_SLOT_SIZE: usize = 1024 * 1024; // 1 MB
        if let Some(size) = slot_size {
            if size == 0 {
                return Err(crate::HorusError::InvalidInput(
                    crate::error::ValidationError::Other("Slot size must be > 0".to_string()),
                ));
            }
            if size > MAX_SLOT_SIZE {
                return Err(crate::HorusError::InvalidInput(
                    crate::error::ValidationError::Other(format!(
                        "Slot size {} exceeds maximum (1 MB)",
                        size
                    )),
                ));
            }
        }
        let is_pod = Self::check_is_pod();
        let type_size = mem::size_of::<T>() as u32;
        let type_align = mem::align_of::<T>() as u32;

        // Does this topic get the co-located geometry? See `shm_layout`: the
        // stamp shares a cache line with its payload, so a receive costs one
        // coherence miss instead of two.
        //
        // This branch used to allocate the same 64 bytes per slot with a
        // comment describing that layout — but the stamps were allocated
        // separately below, and the POD paths indexed the data by
        // `size_of::<T>()` rather than by the 64-byte slot. So the padding was
        // paid for and the co-location never happened; `colo_layout_selected_
        // for_small_pod_types` "proved" it by asserting `size_of::<T>() + 8 <=
        // 64`, which is just this branch condition restated.
        //
        // The comment also claimed the 64-byte geometry was a Python<->Rust
        // wire contract that could not be changed on one side. It is not:
        // horus_py contains no slot-size computation at all (nor does
        // horus_cpp) — both are FFI wrappers over this same Rust code, so
        // there is only one side.
        let colo = layout::colo_eligible(is_pod, type_size as usize, type_align as usize);
        let actual_slot_size = if is_pod {
            let ts = type_size as usize;
            if colo {
                layout::colo_slot_size(ts)
            } else {
                ts
            }
        } else {
            slot_size.unwrap_or(DEFAULT_SLOT_SIZE)
        };

        let actual_capacity = capacity.next_power_of_two() as usize;
        // Colo carries readiness inside each slot, so it has no separate
        // sequence array; that array is exactly the second cache line the
        // layout exists to remove.
        let seq_array_size = if colo {
            0
        } else {
            actual_capacity * mem::size_of::<u64>()
        };
        let data_size = actual_capacity * actual_slot_size;
        let total_size = Self::HEADER_SIZE + seq_array_size + data_size;

        let storage = Arc::new(ShmRegion::new(name, total_size)?);
        // Extract a short type name for the header (e.g. "CmdVel" from
        // "horus_robotics::messages::CmdVel").
        let full_type_name = std::any::type_name::<T>();
        let short_type_name = &strip_module_paths(full_type_name);
        let final_slot_size = Self::negotiate_shm_header(
            name,
            &storage,
            type_size,
            type_align,
            is_pod,
            capacity,
            actual_slot_size,
            short_type_name,
            topic_kind,
        )?;

        let header_ptr = storage.as_ptr() as *const TopicHeader;

        // Notify network layer (horus_net) that a topic was created
        let type_name_full = std::any::type_name::<T>();
        notify_topic_lifecycle(TopicLifecycleEvent::Created {
            name: name.to_string(),
            type_name_hash: crate::communication::topic::fnv1a_type_hash(type_name_full),
            type_size,
            is_pod,
        });

        Ok(Self {
            name: name.to_string(),
            process_epoch: registry::get_or_create_process_epoch(name),
            storage,
            backend: std::cell::UnsafeCell::new(BackendStorage::Uninitialized),
            send_fn: std::cell::UnsafeCell::new(dispatch::send_uninitialized::<T>),
            recv_fn: std::cell::UnsafeCell::new(dispatch::recv_uninitialized::<T>),
            local: std::cell::UnsafeCell::new(LocalState {
                is_pod,
                slot_size: final_slot_size,
                ..Default::default()
            }),
            header_ptr: std::cell::Cell::new(header_ptr),
            metrics: Arc::new(MigrationMetrics::default()),
            state: AtomicU8::new(ConnectionState::Connected.into_u8()),
            spill_pool: std::cell::UnsafeCell::new(None),
            _marker: PhantomData,
        })
    }

    /// Negotiate SHM header: initialize if owner, wait if joiner, validate if existing.
    ///
    /// Returns the final slot size from the header.
    // All 9 arguments mirror the SHM header wire fields directly — a wrapper struct
    // would add an allocation at the only call site without semantic benefit.
    #[allow(clippy::too_many_arguments)]
    fn negotiate_shm_header(
        name: &str,
        storage: &ShmRegion,
        type_size: u32,
        type_align: u32,
        is_pod: bool,
        capacity: u32,
        actual_slot_size: usize,
        type_name_str: &str,
        topic_kind: u8,
    ) -> HorusResult<usize> {
        // SAFETY: storage is properly sized (>= HEADER_SIZE) and aligned for TopicHeader
        let header = unsafe { &mut *(storage.as_ptr() as *mut TopicHeader) };

        std::sync::atomic::fence(Ordering::Acquire);

        // Detect stale SHM from a dead process: if the header has valid magic
        // but the creator process no longer exists, reinitialize the header.
        // This prevents panics from stale backend modes (e.g., FanoutShm from
        // a previous run that left behind SHM files).
        let is_stale = header.magic == TOPIC_MAGIC
            && header.creator_pid != 0
            && header.creator_pid != std::process::id()
            && !is_process_alive(header.creator_pid);

        if header.magic != TOPIC_MAGIC || (is_stale && storage.is_owner()) {
            if storage.is_owner() {
                // Fresh or stale SHM — reinitialize the header.
                header.init(
                    type_size,
                    type_align,
                    is_pod,
                    capacity,
                    actual_slot_size as u32,
                    type_name_str,
                    topic_kind,
                );
                return Ok(actual_slot_size);
            }
            // Joiner: wait for owner to initialize the header.
            // Use exponential backoff (1ms→50ms) with a 2s deadline.
            // The generous deadline prevents spurious timeouts under heavy
            // thread contention (e.g., 100+ topics starting simultaneously).
            let started = std::time::Instant::now();
            let deadline = started + std::time::Duration::from_secs(2);
            let mut backoff_ms = 1u64;
            loop {
                std::thread::sleep(std::time::Duration::from_millis(backoff_ms));
                std::sync::atomic::fence(Ordering::Acquire);
                if header.magic == TOPIC_MAGIC {
                    break;
                }
                if std::time::Instant::now() >= deadline {
                    // Structured, for the same reason as the type-mismatch arm
                    // below: wrapped in a bare String this rendered through the
                    // `From<String>` impl as "Communication serialization
                    // failed:", reporting a timeout as a serialization fault.
                    // Callers that want to tell "the owner is still starting"
                    // from "the region is stale or corrupt" now have a variant
                    // to match instead of a message to substring-search.
                    return Err(HorusError::Communication(
                        crate::error::CommunicationError::HeaderInitTimeout {
                            topic: name.to_string(),
                            waited: started.elapsed(),
                        },
                    ));
                }
                backoff_ms = (backoff_ms * 2).min(50);
            }
            // The owner has stamped the magic; its geometry is now this
            // process's ring geometry, and it is untrusted for exactly the same
            // reason as the already-initialized case below.
            return Self::validate_ring_geometry(name, storage, header, type_size);
        }

        // Header already initialized — validate compatibility.
        if header.version != TOPIC_VERSION {
            return Err(HorusError::Communication(
                format!(
                    "Incompatible topic version: {} (expected {})",
                    header.version, TOPIC_VERSION
                )
                .into(),
            ));
        }

        // Type name validation — prevents silent data corruption when two
        // processes open the same topic name with different message types.
        // e.g., Process A: Topic<CmdVel>("motor.cmd"), Process B: Topic<LaserScan>("motor.cmd")
        // Case-insensitive: Python lowercases type names ("imu"), Rust uses PascalCase ("Imu").
        //
        // Exception: GenericMessage is the universal type — it can open any topic
        // regardless of the existing type. This enables Python interop where
        // horus_py always uses GenericMessage (JSON serialized dicts) while
        // Rust nodes use typed topics. GenericMessage handles serialization
        // internally and won't cause data corruption.
        let existing_type = header.type_name_str();
        let is_generic = type_name_str.eq_ignore_ascii_case("GenericMessage")
            || existing_type.eq_ignore_ascii_case("GenericMessage");
        // Compare what was actually stored, not what we would have liked to
        // store. `type_name` is a fixed 32-byte field, so a longer name is
        // written truncated — and then a second opener of the *same* type
        // compared its full name against the truncated one and declared a type
        // mismatch against itself:
        //
        //   Existing type 'ActionFeedback<CancelTopicFeedb',
        //   attempted 'ActionFeedback<CancelTopicFeedback>'
        //
        // Any type whose name reaches the field width hit this; generic types
        // (`ActionFeedback<…>`, `ServiceResponse<…>`) reach it easily.
        let attempted_as_stored = super::topic::header::type_name_as_stored(type_name_str);
        if !is_generic
            && !existing_type.is_empty()
            && !type_name_str.is_empty()
            && !existing_type.eq_ignore_ascii_case(attempted_as_stored)
        {
            // Was wrapped in a bare String, which renders through the
            // `From<String>` impl as "Communication serialization failed:" —
            // so a type mismatch was reported as a serialization failure,
            // behind two stacked prefixes.
            return Err(HorusError::Communication(
                crate::error::CommunicationError::TopicCreationFailed {
                    topic: name.to_string(),
                    reason: format!(
                        "type mismatch. Existing type '{existing_type}', attempted \
                         '{type_name_str}'.\n\
                         Two processes opened the same topic with different message \
                         types.\n\
                         Fix: use distinct topic names for different message types."
                    ),
                },
            ));
        }

        // Size validation.
        //
        // Deliberately NOT exempt for GenericMessage, unlike the type-NAME check
        // above. The POD data path addresses slots as
        // `(cached_data_ptr as *mut T).add(index)` (dispatch.rs), so its stride
        // is `size_of::<T>()` — it must equal the creator's `type_size` no matter
        // what the type names say, and a name-based exemption cannot make two
        // different strides agree. `GenericMessage` is itself POD-classified
        // (fixed array, no Drop), so the old exemption let a
        // `Topic<GenericMessage>` (4364 bytes) join a `Topic<CmdVel>` ring
        // (8 bytes) and write megabytes past the end of the mapping. The header
        // side of the test matters too: `sync_local` adopts `header.is_pod_type()`
        // for dispatch selection, so either side being POD puts a raw stride on
        // this ring.
        if (is_pod || header.is_pod_type()) && header.type_size != type_size {
            return Err(HorusError::Communication(
                crate::error::CommunicationError::TopicCreationFailed {
                    topic: name.to_string(),
                    reason: format!(
                        "type size mismatch — the topic holds {} byte messages, this \
                         build's '{type_name_str}' is {} bytes",
                        header.type_size, type_size
                    ),
                },
            ));
        }

        Self::validate_ring_geometry(name, storage, header, type_size)
    }

    /// Check the ring geometry an existing header declares against the mapping
    /// this process actually holds, and return the validated `slot_size`.
    ///
    /// Every field of a topic header is written by another process, and on a
    /// robot every node shares `/dev/shm`, so one compromised — or merely buggy
    /// — process can put any 32-bit value in `capacity`, `capacity_mask` and
    /// `slot_size` of a topic it can open. The data path then consumes all
    /// three completely unchecked, and says so:
    ///
    /// * `dispatch.rs` lists "index bounds" among its safety invariants —
    ///   "`capacity_mask = capacity - 1` and capacity is always a power of two.
    ///   This guarantees `index < capacity` without bounds checks";
    /// * `init_shm_backend` derives `cached_data_ptr` with the comment
    ///   "capacity was validated when the SHM region was mapped".
    ///
    /// Neither was true. `ensure_role` turns `header.capacity` straight into
    /// `cached_data_ptr` (`HEADER_SIZE + capacity * 8` from the mapping base),
    /// `sync_local` copies `capacity_mask` and `slot_size` into the local state,
    /// and every send/recv computes its slot as `(seq & capacity_mask) *
    /// stride` from that pointer with no further check. A header declaring
    /// `capacity_mask = 0xFFFF_FFFF` therefore puts a `simd_aware_write` up to
    /// `4G * size_of::<T>()` bytes past the end of the mapping — a wild write at
    /// an offset the writer chooses, on a control topic, in another process.
    ///
    /// This function is the single point every attaching process passes
    /// through, so this is where that invariant gets established rather than
    /// assumed. A header that fails is refused: the topic does not open, which
    /// is a loud startup failure instead of a silent one that moves an
    /// actuator.
    ///
    /// The bound is the mapping this process really has (`storage.len()`), not
    /// the size it asked for: `ShmRegion::new` maps `max(requested, file len)`,
    /// so a legitimate joiner of a larger ring — or of one another process
    /// auto-grew — is already mapped over the whole file and passes.
    fn validate_ring_geometry(
        name: &str,
        storage: &ShmRegion,
        header: &TopicHeader,
        type_size: u32,
    ) -> HorusResult<usize> {
        let capacity = header.capacity as usize;
        let capacity_mask = header.capacity_mask;
        let slot_size = header.slot_size.load(Ordering::Acquire) as usize;
        let region_len = storage.len();

        let refuse = |reason: String| {
            HorusError::Communication(crate::error::CommunicationError::TopicCreationFailed {
                topic: name.to_string(),
                reason,
            })
        };

        if capacity == 0 || !capacity.is_power_of_two() {
            return Err(refuse(format!(
                "shared header declares ring capacity {capacity}, which is not a power of \
                 two — slot indices are computed as `seq & capacity_mask` with no bounds \
                 check, so this ring cannot be addressed safely"
            )));
        }
        if capacity_mask as usize != capacity - 1 {
            return Err(refuse(format!(
                "shared header declares capacity {capacity} with capacity_mask {capacity_mask:#x} \
                 (must be capacity - 1) — indexing with that mask reads and writes outside \
                 the ring"
            )));
        }
        if slot_size == 0 {
            return Err(refuse(
                "shared header declares slot_size 0 — every slot would alias slot 0".to_string(),
            ));
        }
        // A colo region carries the readiness stamp inside each slot, so a slot
        // must hold the stamp AND the payload, and the region has no sequence
        // array at all. Both differences change what a valid geometry is, and
        // getting either backwards is not a rejected topic but a misaddressed
        // one: validating a colo header with the split formula demands the
        // `capacity * 8` bytes a colo region correctly does not have, and
        // rejects every valid one.
        let colo = header.is_colo();

        // The split POD path addresses slots as
        // `(cached_data_ptr as *mut T).add(index)`, so its stride is
        // `size_of::<T>()` while the region was sized in units of `slot_size`.
        // The colo path strides by the whole slot and starts the payload eight
        // bytes in, so it needs room for both. The owner always writes a
        // slot_size that satisfies this; a header that does not is one where
        // each write runs into the following slot.
        let min_slot = if colo {
            (type_size as usize).saturating_add(shm_layout::COLO_PAYLOAD_OFF)
        } else {
            type_size as usize
        };
        if header.is_pod_type() && slot_size < min_slot {
            return Err(refuse(format!(
                "shared header declares slot_size {slot_size} for a {type_size}-byte POD \
                 message under the {} layout, which needs at least {min_slot} — each write \
                 would run past its slot",
                if colo { "co-located" } else { "split" }
            )));
        }

        let required = if colo {
            shm_layout::colo_required_region_len_checked(capacity, slot_size)
        } else {
            shm_layout::required_region_len_checked(capacity, slot_size)
        }
        .ok_or_else(|| {
            refuse(format!(
                "shared header declares a ring geometry (capacity {capacity} x slot_size \
                 {slot_size}) whose size overflows the address space"
            ))
        })?;
        if required > region_len {
            return Err(refuse(format!(
                "shared header declares a {required}-byte ring (capacity {capacity}, slot_size \
                 {slot_size}) but only {region_len} bytes are mapped — the data path would \
                 address {} bytes past the end of the mapping",
                required - region_len
            )));
        }

        debug_assert!(
            Self::geometry_is_addressable(header, region_len),
            "validate_ring_geometry accepted a header geometry_is_addressable rejects — \
             the two have drifted apart"
        );
        Ok(slot_size)
    }

    /// The geometry invariant every slot index depends on, as a predicate.
    ///
    /// [`Self::validate_ring_geometry`] is this same test with the diagnostics
    /// for refusing to OPEN a topic; this is the form used as a filter on what a
    /// later header rewrite is allowed to install (see [`Self::sync_local`]).
    /// Validating only at attach would guard the harder attack and leave the
    /// easy one open: a process that can write the header can rewrite the
    /// geometry of a topic that is already running and bump `migration_epoch`,
    /// and every peer re-reads all three fields on the next epoch check.
    fn geometry_is_addressable(header: &TopicHeader, region_len: usize) -> bool {
        let capacity = header.capacity as usize;
        if capacity == 0 || !capacity.is_power_of_two() {
            return false;
        }
        if header.capacity_mask as usize != capacity - 1 {
            return false;
        }
        let slot_size = header.slot_size.load(Ordering::Acquire) as usize;
        if slot_size == 0 {
            return false;
        }
        // Must agree with `validate_ring_geometry` on both counts, or the
        // debug_assert there fires: a colo slot holds the stamp as well as the
        // payload, and a colo region has no sequence array to account for.
        let colo = header.is_colo();
        let min_slot = if colo {
            mem::size_of::<T>().saturating_add(shm_layout::COLO_PAYLOAD_OFF)
        } else {
            mem::size_of::<T>()
        };
        if header.is_pod_type() && slot_size < min_slot {
            return false;
        }
        if colo {
            shm_layout::colo_required_region_len_checked(capacity, slot_size)
        } else {
            shm_layout::required_region_len_checked(capacity, slot_size)
        }
        .is_some_and(|required| required <= region_len)
    }

    /// Check if T is a POD type (auto-detected via needs_drop)
    fn check_is_pod() -> bool {
        is_pod::<T>()
    }

    /// Get a reference to the header
    #[inline(always)]
    fn header(&self) -> &TopicHeader {
        // SAFETY: storage is properly sized and aligned for TopicHeader; initialized in constructor
        unsafe { &*(self.storage.as_ptr() as *const TopicHeader) }
    }

    /// Bump the "new data exists" counter the staleness watchdog reads.
    ///
    /// `RingTopic::send` does this inline. It lives here as well so the public
    /// `Topic::try_send` / `Topic::send_blocking` can do it exactly once per
    /// call: it must NOT move into `RingTopic::try_send`, which
    /// `send_lossy_retry` calls up to ~70 times for a single message.
    #[inline(always)]
    fn bump_messages_total(&self) {
        self.header().messages_total.fetch_add(1, Ordering::Relaxed);
    }

    /// Check if verbose content logging is enabled via the SHM header flag.
    /// Uses the stable `header_ptr` (not `LocalState::cached_header_ptr`, which
    /// is repurposed by the role=Both same-instance fast path).
    #[inline(always)]
    fn is_verbose(&self) -> bool {
        // SAFETY: header_ptr points into the Arc<ShmRegion> storage which outlives self;
        // the header is initialized before construction completes.
        unsafe { (*self.header_ptr.get()).is_verbose() }
    }

    /// Point `cached_seq_ptr`, `cached_data_ptr` and `cached_colo_stride` at
    /// the slot geometry this region actually uses.
    ///
    /// Single writer for all three, because they must agree: a colo stride
    /// paired with a split data pointer reads payload bytes as stamps. Every
    /// site that maps or re-maps the region calls this instead of recomputing
    /// the offsets locally — four such sites had already been copy-pasted, and
    /// independently-maintained copies of layout arithmetic are precisely what
    /// `shm_layout` exists to stop drifting.
    #[inline]
    fn point_at_slots(&self, local: &mut LocalState) {
        // Geometry comes from `LocalState`, never fresh from the header.
        //
        // `sync_local` adopts `slot_size`/`capacity`/`capacity_mask` together and
        // ONLY if `geometry_is_addressable` passes, leaving the handle on its last
        // validated set otherwise. Every caller used to hand this function a
        // capacity taken from that validated state but a `slot_size` re-read from
        // shared memory — so on a refusal, or if a peer rewrote the field between
        // the two reads, the pair described no ring that exists: slot `i` at
        // `base + i * new_stride` inside a mapping sized for the old one. Reading
        // both from `local` makes the pair validated-by-construction, which is the
        // point of this function being the single writer of the three pointers.
        //
        // `is_colo` is still read from the header because the layout kind is fixed
        // when the region is created and auto-grow preserves it; it is not a value
        // a peer can change underneath us.
        let capacity = local.cached_capacity as usize;
        let stride = local.slot_size;
        let colo = self.header().is_colo();
        let base = self.storage.as_ptr();
        // SAFETY: both offsets lie within the region sized by `new`/auto-grow
        // for this capacity and layout; `base` is a valid non-null mapping.
        unsafe {
            local.cached_seq_ptr = base.add(Self::HEADER_SIZE) as *mut u8;
            if colo {
                local.cached_colo_stride = stride as u64;
                local.cached_data_ptr =
                    base.add(Self::HEADER_SIZE + layout::COLO_PAYLOAD_OFF) as *mut u8;
            } else {
                local.cached_colo_stride = 0;
                local.cached_data_ptr = base.add(Self::data_region_offset(capacity)) as *mut u8;
            }
        }
    }

    /// Adopt the header's geometry AND re-derive the pointers that address it.
    ///
    /// These two must happen together. `sync_local` installs a new
    /// `cached_capacity` / `slot_size` / colo-ness into `LocalState`;
    /// `point_at_slots` is what makes `cached_seq_ptr` / `cached_data_ptr` /
    /// `cached_colo_stride` describe that same geometry in this mapping. Run one
    /// without the other and the handle addresses slot `i` at
    /// `old_base + i * new_stride` — off the end of the mapping it actually
    /// holds, because a geometry change is exactly when the region was grown.
    ///
    /// `check_migration` had two branches that called `sync_local` alone and
    /// relied on the trailing `initialize_backend()` to re-point. That works
    /// only when the backend is *replaced*: `initialize_backend` short-circuits
    /// on `backend_matches_mode`, so a resync that keeps the same mode (the
    /// migration-lock contention path) adopted the new geometry and kept the old
    /// pointers. Pairing them here removes the chance to forget, which is the
    /// same reason `point_at_slots` was made the single writer of those three
    /// fields in the first place.
    fn sync_local_and_point(&self, local: &mut LocalState, header: &TopicHeader, strict: bool) {
        Self::sync_local(local, header, strict, self.storage.len());
        self.point_at_slots(local);
    }

    /// Compute the byte offset from storage start to the data region.
    ///
    /// Layout: [HEADER (640)] [SEQ_ARRAY (capacity * 8)] [DATA (capacity * slot_size)]
    #[inline]
    fn data_region_offset(capacity: usize) -> usize {
        Self::HEADER_SIZE + capacity * mem::size_of::<u64>()
    }

    /// Get the local state (interior mutability via UnsafeCell)
    #[inline(always)]
    #[allow(clippy::mut_from_ref)] // UnsafeCell interior mutability: thread-local access pattern
    fn local(&self) -> &mut LocalState {
        // SAFETY: LocalState access is thread-local; no concurrent mutation
        unsafe { &mut *self.local.get() }
    }

    /// Sync local cached fields from the SHM header. Called after epoch changes
    /// and during initial registration to cache header values for zero-overhead
    /// dispatch. When `skip_stale_broadcast` is true, PodShm consumers reset
    /// tail = head to skip stale data from the previous era.
    ///
    /// `region_len` is the length of the mapping this handle holds. It is what
    /// bounds the ring geometry the header declares: this function is the ONE
    /// place where `capacity`, `capacity_mask` and `slot_size` cross from shared
    /// memory into the local state that every send and recv indexes with, so it
    /// is where a geometry the mapping cannot hold gets refused.
    #[inline(always)]
    fn sync_local(
        local: &mut LocalState,
        header: &TopicHeader,
        skip_stale_broadcast: bool,
        region_len: usize,
    ) {
        // Capture the pre-sync capacity to decide whether this resync crosses
        // a data-plane boundary (a capacity grow; see the local_tail logic below).
        let old_capacity = local.cached_capacity;

        local.is_same_process = header.is_same_process();
        local.is_pod = header.is_pod_type();
        local.cached_mode = header.mode();
        // The geometry is adopted only if it is addressable inside this mapping.
        // Refusing leaves the handle on the geometry it last validated, and every
        // caller derives `cached_data_ptr` from whichever geometry ends up
        // installed here — so the pointer, the mask and the slot size always
        // describe one ring inside one mapping. Adopting unconditionally instead
        // would point `(seq & capacity_mask) * stride` outside the region: a wild
        // write on a control topic, at an offset chosen by whichever process
        // wrote the header. A refusal is not silent — `handle_epoch_change` logs
        // it.
        if Self::geometry_is_addressable(header, region_len) {
            local.slot_size = header.slot_size.load(Ordering::Acquire) as usize;
            local.cached_capacity = header.capacity as u64;
            local.cached_capacity_mask = header.capacity_mask as u64;
        }
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        let shared_tail = header.tail.load(Ordering::Acquire);

        // Data-plane-aware consumed frontier (softmata-brain 1327, scenario X).
        //
        // `header.tail` is the SHARED consumed position, but SPSC/MPSC recv flush
        // it only every capacity/2 reads (the single consumer's `local_tail` is
        // authoritative between flushes). So on a resync `header.tail` typically
        // LAGS this handle's true consumed position. Blindly adopting it would
        // REGRESS the consumer and re-deliver already-consumed messages the next
        // recv — a safety-critical hazard (stale command re-sent to a motor across
        // a topology change, e.g. SpscShm -> MpscShm when a 2nd producer joins).
        //
        // When the migration stays within the SHM data plane (both modes are
        // cross-process AND the ring capacity is unchanged), message positions are
        // continuous, so `local_tail` is meaningful in the new coordinates: take
        // the max so the frontier never rewinds. (This is also correct for SPMC,
        // where other consumers CAS `header.tail` ahead of us — max() then adopts
        // the shared frontier, skipping what peers already took.)
        //
        // When the migration CHANGES the data plane (a capacity grow, which
        // rewrites messages at fresh positions), the old `local_tail` is
        // meaningless in the new coordinates, so we must adopt `header.tail`.
        // Fresh handles (`local_tail == 0`) are unaffected either way:
        // `max(0, header.tail) == header.tail`.
        //
        // We key "same data plane" on the topic being SHM-BACKED (creator_pid != 0),
        // not on the OLD mode being cross-process. Every SHM mode
        // (Spsc/Mpsc/Spmc/Pod/Fanout*Shm) shares the ONE ShmData ring, so a transition
        // between any of them preserves positions — INCLUDING from the not-yet-
        // classified `Unknown` a handle carries before its first send/recv. Requiring
        // `old_mode.is_cross_process()` wrongly excluded that Unknown case: an EARLY
        // cross-process subscriber (created before any message, so `cached_mode ==
        // Unknown`) migrating straight to PodShm would be treated as a data-plane
        // CHANGE, trip the broadcast "skip to head" below, and lose all buffered
        // messages that other processes published (softmata-brain bug #2). The only
        // Now that every topic is SHM-backed and all backends share the ONE
        // ShmData ring, the only real data-plane change is a capacity grow.
        // `old_capacity == 0` means this handle has never synced (uninitialized), not a
        // ring resize — treat it as same-plane (there is no prior capacity to differ
        // from). Only a genuine grow (old != new, both nonzero) is a data-plane change.
        let same_data_plane = header.creator_pid != 0
            && local.cached_mode.is_cross_process()
            && (old_capacity == 0 || old_capacity == local.cached_capacity);
        local.local_tail = if same_data_plane {
            local.local_tail.max(shared_tail)
        } else {
            shared_tail
        };

        // Broadcast "skip to head" — start a consumer at the newest message instead
        // of replaying the ring. This is correct ONLY across a DATA-PLANE CHANGE
        // (e.g. intra -> PodShm): the old messages live in a different ring and are
        // stale in the new coordinates. Across a SAME-DATA-PLANE migration (an
        // SpscShm topic gaining a subscriber -> PodShm on the SAME shm ring), an
        // EXISTING consumer's unconsumed backlog is NOT stale — skipping it drops
        // in-flight messages (a subscriber joining a live topic would starve the
        // existing subscribers; softmata-brain 1327 broadcast mid-stream join). So
        // only skip when the data plane actually changed.
        if skip_stale_broadcast && local.cached_mode == BackendMode::PodShm && !same_data_plane {
            local.local_tail = local.local_head;
        }

        // Consumer-join flush (softmata-brain 1327, consumer side). SpmcShm's recv
        // CAS-coordinates competing consumers via the shared `header.tail`. A single
        // consumer arriving from SpscShm/MpscShm advanced only its BATCHED local
        // tail, so `header.tail` lags its true consumed position; if a 2nd consumer
        // now joins and CAS-reads from that stale value, it RE-DELIVERS messages the
        // first consumer already took. Publishing this handle's consumed frontier to
        // `header.tail` at the migration boundary makes the CAS-tail handoff lossless.
        //
        // This is done ONLY here, at resync into a CAS-tail mode — NOT on every recv.
        // Eager per-recv flushing would keep the producer-visible tail accurate on the
        // hot path and destabilize `send_shm_mp_pod`'s optimistic backpressure (whose
        // margin currently relies on the batched tail lagging — see the separately
        // tracked overshoot bug). `fetch_max` never moves the shared tail backward, so
        // it is safe even if a concurrent SpmcShm consumer has already advanced it.
        if local.cached_mode == BackendMode::SpmcShm {
            header.tail.fetch_max(local.local_tail, Ordering::Release);
        }
    }

    /// Register as producer if not already registered
    #[inline]
    fn ensure_producer(&self) -> HorusResult<()> {
        self.ensure_role(true)
    }

    /// Register as consumer if not already registered
    #[inline]
    fn ensure_consumer(&self) -> HorusResult<()> {
        self.ensure_role(false)
    }

    /// Shared registration logic for producer (is_producer=true) or consumer.
    fn ensure_role(&self, is_producer: bool) -> HorusResult<()> {
        let local = self.local();
        if is_producer {
            if local.role.can_send() {
                return Ok(());
            }
        } else if local.role.can_recv() {
            return Ok(());
        }

        let header = self.header();
        let slot = if is_producer {
            header.register_producer()?
        } else {
            header.register_consumer()?
        };

        local.slot_index = slot as i32;
        local.cached_header_ptr = self.storage.as_ptr() as *const TopicHeader;
        local.cached_epoch = header.migration_epoch.load(Ordering::Acquire);
        // Sync BEFORE deriving the data pointer, and derive it from the synced
        // capacity rather than from `header.capacity` directly. The header is
        // shared memory: it was validated when this topic was opened, but another
        // process can have rewritten it since, and `sync_local` is what refuses a
        // geometry this mapping cannot hold. Reading `header.capacity` here
        // instead would put `cached_data_ptr` outside the region before the first
        // message is ever sent.
        Self::sync_local(local, header, false, self.storage.len());
        // `cap` is the capacity `sync_local` accepted, so the whole ring fits
        // inside `storage.len()` under whichever geometry the header declares.
        self.point_at_slots(local);
        // Set role LAST — this gates the fast path via can_send()/can_recv()
        local.role = if is_producer {
            if local.role == TopicRole::Consumer {
                TopicRole::Both
            } else {
                TopicRole::Producer
            }
        } else if local.role == TopicRole::Producer {
            TopicRole::Both
        } else {
            TopicRole::Consumer
        };

        // Tell the network layer which direction this handle actually uses.
        //
        // This runs once per handle per direction — `ensure_role` returns early
        // once the role already covers the call — so it is off the hot path, and
        // it is the earliest point at which the answer is knowable at all.
        // Without it the import guard cannot tell a topic this robot publishes
        // from one it merely subscribes to, and "deny imports for topics we
        // publish" degrades either to denying everything or to allowing a remote
        // peer to overwrite our own commands.
        notify_topic_lifecycle(TopicLifecycleEvent::RoleObserved {
            name: self.name.clone(),
            publisher: is_producer,
            type_name_hash: fnv1a_type_hash(std::any::type_name::<T>()),
            type_size: std::mem::size_of::<T>() as u32,
            is_pod: local.is_pod,
        });

        // Late-join fix: if the ring has wrapped since no consumer was reading,
        // advance tail to skip overwritten slots. Without this, a new consumer
        // sees ready_flag != expected_seq and permanently returns None because
        // the publisher overwrote old slots with newer sequence numbers.
        if !is_producer {
            let head = local.local_head;
            let tail = local.local_tail;
            let cap = local.cached_capacity;
            if cap > 0 && head.wrapping_sub(tail) > cap {
                let new_tail = head.wrapping_sub(cap);
                local.local_tail = new_tail;
                // Advance the shared tail so the producer doesn't see the ring as full.
                // fetch_max is safe with concurrent consumers (never moves tail backward).
                header.tail.fetch_max(new_tail, Ordering::Release);
            }
        }

        self.check_migration();
        self.initialize_backend();

        // Prime msg_counter so the NEXT send/recv triggers an immediate migration
        // check. This catches the common case where two processes create Topics
        // concurrently and one hasn't registered yet when the other calls
        // check_migration() above. (GitHub issue #37)
        local.msg_counter = EPOCH_CHECK_INTERVAL.wrapping_sub(1);

        Ok(())
    }

    /// Check if we need to migrate backends and do so if needed
    fn check_migration(&self) {
        let header = self.header();
        let local = self.local();

        let current_epoch = header.migration_epoch.load(Ordering::Acquire);
        if current_epoch != local.cached_epoch {
            // Epoch changed — another participant completed a migration while we
            // weren't looking.  Sync local state to the new epoch, then
            // re-validate: a *second* concurrent migration may have advanced the
            // epoch again between our Acquire load above and the reads inside
            // sync_local().  Loop (up to 4 times) until the epoch is stable so
            // that initialize_backend() sees a consistent (mode, epoch) pair.
            // If migrations keep arriving we proceed with the best-effort latest
            // epoch; the next send/recv will re-check via the epoch_guard macros.
            const MAX_EPOCH_RETRIES: u32 = 4;
            let mut stable_epoch = current_epoch;
            local.cached_epoch = stable_epoch;
            self.sync_local_and_point(local, header, true);
            for _ in 0..MAX_EPOCH_RETRIES {
                let reloaded = header.migration_epoch.load(Ordering::Acquire);
                if reloaded == stable_epoch {
                    break; // Epoch unchanged — local state is consistent.
                }
                // Concurrent migration advanced the epoch while we were
                // syncing; re-sync to the newer epoch and check again.
                stable_epoch = reloaded;
                local.cached_epoch = stable_epoch;
                self.sync_local_and_point(local, header, true);
            }
            // Re-initialize backend for the new (stable) epoch.
            //
            // `initialize_backend()` short-circuits when `backend_matches_mode`
            // is true — it checks only the *type* of the backend, not the epoch.
            // If the backend MODE is the same across epochs (e.g. SpscShm
            // epoch 0 → SpscShm epoch 2 after a double migration) the
            // short-circuit would silently leave us holding the ring from the old
            // epoch, causing send/recv to diverge from other participants who
            // have moved to the ring for the new epoch.
            //
            // Resetting to Uninitialized before the call bypasses the
            // short-circuit and forces a registry lookup (or creation) for
            // `stable_epoch`, ensuring we always join the correct ring.
            //
            {
                // SAFETY: backend UnsafeCell is accessed from this thread only.
                // The old backend Arc is dropped here; any in-flight messages in it
                // were already handled during the migration that incremented the
                // epoch (the migrator holds the lock and drains before updating the
                // header).  Resetting is therefore safe and loss-free.
                let backend = unsafe { &mut *self.backend.get() };
                *backend = BackendStorage::Uninitialized;
            }
            self.initialize_backend();
            registry::notify_epoch_change(&self.name, stable_epoch);
            self.process_epoch
                .fetch_max(stable_epoch, Ordering::Release);
        }

        let migrator = BackendMigrator::new(header);
        if !migrator.is_optimal() {
            for attempt in 0u32..5 {
                match migrator.migrate_to_optimal() {
                    MigrationResult::Success { new_epoch } => {
                        local.cached_epoch = new_epoch;
                        self.sync_local_and_point(local, header, true);
                        self.metrics.migrations.fetch_add(1, Ordering::Relaxed);
                        // Notify all same-process Topics of the epoch change
                        registry::notify_epoch_change(&self.name, new_epoch);
                        self.process_epoch.fetch_max(new_epoch, Ordering::Release);
                        break;
                    }
                    MigrationResult::AlreadyInProgress | MigrationResult::LockContention => {
                        // Another thread is migrating. Spin-wait for it to complete
                        // (drain takes ~1ms) then re-check and retry.
                        let wait_start = std::time::Instant::now();
                        while migrator.is_migration_in_progress() {
                            std::hint::spin_loop();
                            if wait_start.elapsed() > 5_u64.ms() {
                                // Stale lock from a crashed process — force-unlock
                                header.migration_lock.store(0, Ordering::Release);
                                break;
                            }
                        }
                        // The other migration completed — refresh local state
                        let new_epoch = header.migration_epoch.load(Ordering::Acquire);
                        if new_epoch != local.cached_epoch {
                            local.cached_epoch = new_epoch;
                            self.sync_local_and_point(local, header, true);
                            registry::notify_epoch_change(&self.name, new_epoch);
                            self.process_epoch.fetch_max(new_epoch, Ordering::Release);
                        }
                        if migrator.is_optimal() {
                            break;
                        }
                        // Still not optimal after other migration — retry with
                        // exponential backoff + per-thread jitter to avoid livelock
                        // when many threads race to migrate the same topic.
                        //
                        // attempt 0: spin_loop hint only (no extra sleep — the
                        //   spin-wait above already yielded CPU time)
                        // attempts 1-4: sleep 2^attempt × 100 ns + jitter, capped
                        //   at 1 ms per retry.
                        if attempt > 0 {
                            // Base delay: 200 ns, 400 ns, 800 ns, 1 600 ns → cap 1 ms.
                            let base_ns: u64 = (100u64 << attempt).min(1_000_000);
                            // Per-thread jitter derived from the stack address of a
                            // local variable (differs between threads due to
                            // ASLR + per-thread stacks).  Mixed with a Fibonacci
                            // constant to spread the lower bits.
                            let local_addr = &base_ns as *const u64 as u64;
                            let jitter_ns = (local_addr.wrapping_mul(0x9e3779b97f4a7c15) >> 32)
                                % base_ns.max(1);
                            std::thread::sleep(std::time::Duration::from_nanos(
                                base_ns + jitter_ns,
                            ));
                        }
                    }
                    MigrationResult::NotNeeded => {
                        local.cached_mode = header.mode();
                        break;
                    }
                    MigrationResult::Failed => {
                        local.cached_mode = header.mode();
                        break;
                    }
                }
            }
            local.cached_mode = header.mode();
        }

        // Re-initialize backend to match the (potentially new) mode.
        // initialize_backend short-circuits if the backend already matches.
        self.initialize_backend();
    }

    /// Refresh our lease in the participant table
    fn refresh_lease(&self) {
        let local = self.local();
        if local.slot_index >= 0 {
            let header = self.header();
            let timeout = header.lease_timeout();
            let now = current_time_ms();
            header.participants[local.slot_index as usize].refresh_lease(now, timeout);
        }
    }

    /// Refresh this participant's lease only once it is past the halfway point
    /// to expiry.
    ///
    /// The lease used to be refreshed purely on a message count (once every
    /// 1024 messages), which made liveness a function of throughput rather than
    /// of time: a 100 Hz subscriber refreshed about every 10 s against a 5 s
    /// timeout and spent half its life *looking* expired, and a subscriber that
    /// had not received anything yet never refreshed at all. Everything that
    /// reads the participant table — `sub_count()`, `detect_optimal_backend`,
    /// and (before it was fixed) slot reclamation in `register_role` — was then
    /// making decisions about a live participant it believed was gone.
    ///
    /// Callers still gate this behind a small message counter so the clock read
    /// stays off the per-message hot path; the wall-clock test here is what
    /// decides whether the store actually happens.
    #[inline]
    fn refresh_lease_if_due(&self) {
        let local = self.local();
        if local.slot_index < 0 {
            return;
        }
        let header = self.header();
        let entry = &header.participants[local.slot_index as usize];
        let timeout = header.lease_timeout();
        let expires = entry.lease_expires_ms.load(Ordering::Acquire);
        let now = current_time_ms();
        if expires == 0 || now.saturating_add(timeout / 2) >= expires {
            self.refresh_lease();
        }
    }

    /// Initialize or re-initialize the per-path optimized backend based on current mode.
    ///
    /// Every topic is SHM-backed, so this creates (or restores) an ShmData backend
    /// pointing to the topic's SHM region for the current mode (FanoutShm gets its
    /// own ShmFanoutRing storage). On a SHM→SHM mode change, pending messages are
    /// drained into the new backend.
    fn initialize_backend(&self) {
        let local = self.local();
        let mode = local.cached_mode;
        let is_pod = local.is_pod;

        // SAFETY: backend UnsafeCell accessed through &self; only this thread mutates it
        let backend = unsafe { &mut *self.backend.get() };

        if Self::backend_matches_mode(backend, mode) && !self.fanout_attach_is_due(local, backend) {
            // Backend matches but fn ptrs may be stale (e.g., first call after construction).
            self.set_dispatch_fn_ptrs(mode, is_pod);
            return;
        }

        // Every topic is SHM-backed — the data plane always lives in the SHM region.
        self.init_shm_backend(backend);

        // Re-read the mode: `init_shm_backend` takes it from the SHM header
        // again, so the backend it built can be for a DIFFERENT mode than the
        // `mode` above (another process migrating between the two reads, or the
        // FanoutShm fallback). Dispatch must match the backend that actually
        // exists — `send_fanout_shm`/`recv_fanout_shm` are the only dispatch
        // functions that destructure `BackendStorage`, and their mismatch arm is
        // `unreachable_unchecked()` in release builds, so a FanoutShm function
        // pointer over a `ShmData` backend is undefined behaviour rather than a
        // panic. `init_shm_backend` keeps `cached_mode` in step for exactly this.
        let built_mode = self.local().cached_mode;
        self.set_dispatch_fn_ptrs(built_mode, is_pod);
    }

    /// Backoff between attempts to rejoin FanoutShm after a failed `attach`.
    ///
    /// A fanout region can be hundreds of megabytes, and a retry maps and unmaps
    /// it, so this cannot run on every poll. 50 ms is far below the timescale a
    /// dropped subscriber matters on and far above the cost of the retry.
    const FANOUT_ATTACH_RETRY_MS: u64 = 50;

    /// Whether this handle fell back off FanoutShm and is due to retry `attach`.
    ///
    /// `backend_matches_mode` compares the backend against `cached_mode`, and the
    /// FanoutShm fallback sets `cached_mode` to `SpscShm` — so from that moment
    /// the two agree and `initialize_backend` returns early every time. Nothing
    /// else re-examines it: `check_migration`'s migrator block, which ends by
    /// restoring `cached_mode` from the header, is gated on `!is_optimal()`, and
    /// `is_optimal()` reads only SHARED header state, which still says FanoutShm
    /// and is still optimal. The degradation is invisible there.
    ///
    /// So the condition is checked directly: a `ShmData` backend on a topic whose
    /// header says FanoutShm is a handle that fell back, and once the backoff has
    /// elapsed it should try to rejoin.
    fn fanout_attach_is_due(&self, local: &LocalState, backend: &BackendStorage<T>) -> bool {
        // The anomaly itself is the trigger, not a flag set on one code path: a
        // `ShmData` backend on a topic whose header says FanoutShm is a handle
        // that is not where the topic agreed to be, however it got there. Keying
        // off a marker set only by the attach fallback missed the other route
        // into the same state — a handle whose `cached_mode` is behind the header
        // and whose backend was therefore never rebuilt.
        if !matches!(backend, BackendStorage::ShmData) {
            return false;
        }
        if self.header().mode() != BackendMode::FanoutShm {
            return false;
        }
        // `fanout_retry_at_ms` is purely a rate limiter here: 0 means "no attempt
        // has failed yet, so try now".
        header::current_time_ms() >= local.fanout_retry_at_ms
    }

    /// Check if the current backend storage already matches the requested mode.
    fn backend_matches_mode(backend: &BackendStorage<T>, mode: BackendMode) -> bool {
        match (backend, mode) {
            (BackendStorage::FanoutShm(_), BackendMode::FanoutShm) => true,
            // FanoutShm has its OWN storage variant (a separate ShmFanoutRing),
            // so it is NOT represented by the shared `ShmData` variant. Excluding
            // it here forces `initialize_backend` to call `init_shm_backend`,
            // which actually creates the fanout ring. Without this exclusion a
            // topic already on a ShmData backend (e.g. SpscShm) that migrates to
            // FanoutShm would keep its ShmData storage while the dispatch fn ptr
            // expects FanoutShm — hitting the `expected FanoutShm variant`
            // debug_unreachable in send/recv.
            (BackendStorage::ShmData, _)
                if (mode.is_cross_process() && mode != BackendMode::FanoutShm)
                    || mode == BackendMode::Unknown =>
            {
                true
            }
            _ => false,
        }
    }

    /// Initialize a cross-process SHM backend, restoring cached pointers.
    ///
    /// Restores the cached data/seq pointers into the mmap'd storage region.
    fn init_shm_backend(&self, backend: &mut BackendStorage<T>) {
        let mode = BackendMode::from(self.header().backend_mode.load(Ordering::Acquire));

        // FanoutShm: create ShmFanoutRing backed by a separate SHM region
        if mode == BackendMode::FanoutShm {
            let is_pod = crate::communication::pod::is_pod::<T>();
            let type_size = mem::size_of::<T>();
            // Prefer the capacity `sync_local` validated over a fresh read of the
            // shared header: this number sizes an mmap, and a header rewritten to
            // `u32::MAX` would have every peer ask the kernel for a petabyte-scale
            // region on each fanout init. Before this handle has synced (capacity
            // 0) there is nothing validated to use, and both sides of the size
            // computation clamp to the 16-slot minimum there anyway.
            let capacity = match self.local().cached_capacity as u32 {
                0 => self.header().capacity,
                cached => cached,
            };
            let fanout_name = format!("{}_fanout", self.name);
            // `required_file_size` is `None` for a geometry this layout cannot
            // carry — a POD `T` larger than the fanout slot cap. Taking the
            // SpscShm fallback below is what keeps `try_send_pod`'s unbounded
            // memcpy inside its slot; the ring used to be built anyway with the
            // slot silently clamped.
            let fanout_storage =
                shm_fanout::ShmFanoutRing::required_file_size(type_size, is_pod, capacity as usize)
                    .and_then(|total_size| {
                        crate::memory::shm_region::ShmRegion::new(&fanout_name, total_size).ok()
                    });
            if let Some(fanout_storage) = fanout_storage {
                let is_owner = fanout_storage.is_owner();
                let shm_base = fanout_storage.as_ptr() as *mut u8;
                let ring = unsafe {
                    if is_owner {
                        // `init_owner` refuses the same geometry `attach` refuses
                        // (POD slots too small for the message) rather than
                        // writing a region no peer would accept.
                        shm_fanout::ShmFanoutRing::init_owner(shm_base, type_size, is_pod, capacity)
                    } else {
                        // COMM-H1: `attach` returns None when the region carries an
                        // older/incompatible layout version (the v3 meta bump) —
                        // reject-and-fall-back instead of reinterpreting a stale
                        // region with the new strides. It also refuses dimensions
                        // that do not fit the mapping we actually have, which is
                        // why it needs that length: every channel pointer is
                        // derived from numbers another process wrote.
                        shm_fanout::ShmFanoutRing::attach(
                            shm_base,
                            fanout_storage.len(),
                            is_pod,
                            type_size,
                        )
                    }
                };
                if let Some(ring) = ring {
                    // Store the SHM region in local state so it stays alive
                    let local = self.local();
                    local.fanout_shm_storage = Some(std::sync::Arc::new(fanout_storage));
                    // The backend now IS a fanout ring; record that as the mode
                    // dispatch is resolved against. `cached_mode` came from an
                    // earlier read of the same header word and can already be a
                    // migration behind.
                    local.cached_mode = BackendMode::FanoutShm;
                    local.fanout_retry_at_ms = 0;
                    *backend = BackendStorage::FanoutShm(Box::new(ring));
                    return;
                }
                // else: stale/incompatible fanout layout — drop this region and
                // fall through to the SpscShm fallback below.
            }
            // Fallback: FanoutShm creation failed (stale SHM from previous run,
            // permission error, etc.). Fall back to SpscShm which uses the main
            // SHM region (ShmData) instead of a separate fanout ring.
            // Update cached_mode so set_dispatch_fn_ptrs selects ShmData-compatible
            // dispatch functions instead of FanoutShm dispatch (which would panic).
            tracing::warn!(
                "FanoutShm init failed for '{}', falling back to SpscShm",
                self.name
            );
            let local = self.local();
            local.cached_mode = BackendMode::SpscShm;
            // Mark the handle degraded so `initialize_backend` retries instead of
            // short-circuiting on the fallback mode forever. The usual cause is a
            // creator that has not published the ring's magic yet, which resolves
            // on its own; retrying is what turns a lost race into a hiccup rather
            // than a subscriber that never receives anything again.
            local.fanout_retry_at_ms =
                header::current_time_ms().saturating_add(Self::FANOUT_ATTACH_RETRY_MS);
        }

        // Standard SHM backends: use ShmData.
        *backend = BackendStorage::ShmData;

        // Restore SHM cached pointers so the dispatch functions read/write the
        // mmap'd storage region (they may have been left pointing elsewhere by a
        // prior init, e.g. a FanoutShm fallback).
        let local = self.local();
        // Reaching here with `cached_mode == FanoutShm` means the header said
        // FanoutShm when `sync_local` read it and something else when this
        // function re-read it a moment ago — a concurrent migration. The backend
        // built is `ShmData`, so the cached mode must say so too: otherwise
        // `set_dispatch_fn_ptrs` installs `send_fanout_shm`, whose "not a
        // FanoutShm backend" arm is `unreachable_unchecked()` in release.
        if local.cached_mode == BackendMode::FanoutShm {
            local.cached_mode = mode;
        }
        // Both offsets are within storage bounds because `validate_ring_geometry`
        // refused this header at attach time unless the ring it declares fits the
        // mapping — under whichever geometry it declares. `cap` comes from that
        // same validated header field; storage.as_ptr() is a valid non-null
        // pointer.
        self.point_at_slots(local);
    }

    /// Set dispatch function pointers based on the current backend mode and POD status.
    ///
    /// Called at the end of `initialize_backend()` to resolve the data path at
    /// initialization time. After this, `try_send()`/`try_recv()` are single
    /// indirect calls with zero branches.
    fn set_dispatch_fn_ptrs(&self, mode: BackendMode, is_pod: bool) {
        let local = self.local();
        let role = local.role;

        // SAFETY: UnsafeCell accessed from single thread (same guarantee as backend/local)
        unsafe {
            *self.send_fn.get() = Self::resolve_send_fn(mode, is_pod, role);
            *self.recv_fn.get() = Self::resolve_recv_fn(mode, is_pod, role);
        }
    }

    /// Resolve the send dispatch function pointer for the given backend mode.
    ///
    /// Returns `send_uninitialized` if the role cannot send, ensuring
    /// `ensure_producer()` is called on first send to register the participant.
    fn resolve_send_fn(mode: BackendMode, is_pod: bool, role: TopicRole) -> dispatch::SendFn<T> {
        if !role.can_send() {
            return dispatch::send_uninitialized::<T>;
        }
        match mode {
            BackendMode::FanoutShm => dispatch::send_fanout_shm::<T>,
            // SpscShm uses the SAME multi-producer-compatible protocol as MpscShm
            // (atomic fetch_add slot claim + per-slot ready flag). This makes the
            // SpscShm -> MpscShm transition (a 2nd producer joining) a no-op at the
            // protocol level: every producer claims distinct slots atomically from
            // message one, so there is no incompatible-protocol window on the shared
            // ring and no multi-producer convergence loss (softmata-brain 1327). The
            // 1P case is safe (loom_sp_mp_flag).
            //
            // What it costs, measured on the metric that matters, so nobody
            // spends a quarter on the wrong project.
            //
            // The figure that used to sit here — "an unloaded `send()` goes from
            // 55ns to 25ns, the locked RMW is ~30ns, the largest single cost left
            // on the publish path" — was measured on back-to-back sends, which is
            // exactly the shape a store-buffer barrier penalises and is not how a
            // control loop sends. Re-measured on the round-trip-paced one-way
            // test, on an idle machine where the harness resolves 25ns cleanly
            // against a +/-1ns baseline: replacing this CAS with a plain
            // single-producer store moves the one-way from ~105ns to ~103ns.
            //
            // ~2ns. Not 30. The barrier is free here because the producer is
            // waiting on its peer anyway, and by the time the answer comes back
            // the store buffer has long since drained.
            //
            // So the conclusion stands but the reasoning inverts. The sole-
            // producer handshake this comment used to describe as "buying the
            // 30ns back" buys ~2ns, and it would have to be correct through the
            // one-send window that is precisely 1327 — two producers claiming the
            // same slot, silent corruption on a control topic. That is not a
            // trade worth making, and the point of writing the number down is so
            // the next person does not build the handshake to find out.
            BackendMode::SpscShm if is_pod => dispatch::send_shm_mp_pod::<T>,
            BackendMode::SpscShm => dispatch::send_shm_mp_serde::<T>,
            // SpmcShm KEEPS the single-producer send path: its multi-consumer CAS
            // recv (recv_shm_spmc_*) relies on `sequence_or_head` being published
            // AFTER the data write (send_shm_sp_* stores head last), which the
            // fetch_add-claim MP path does not provide (it advances head on claim,
            // before the data write). SpmcShm is single-producer, so this is safe.
            BackendMode::SpmcShm if is_pod => dispatch::send_shm_sp_pod::<T>,
            BackendMode::SpmcShm => dispatch::send_shm_sp_serde::<T>,
            BackendMode::PodShm => dispatch::send_shm_pod_broadcast::<T>,
            BackendMode::MpscShm if is_pod => dispatch::send_shm_mp_pod::<T>,
            BackendMode::MpscShm => dispatch::send_shm_mp_serde::<T>,
            // Fallback for Unknown mode: use MPMC SHM serde (handles any topology/type).
            // Must NOT return send_uninitialized here — that causes infinite recursion
            // when the role is already registered but stale SHM left mode as Unknown.
            BackendMode::Unknown if is_pod => dispatch::send_shm_mp_pod::<T>,
            BackendMode::Unknown => dispatch::send_shm_mp_serde::<T>,
        }
    }

    /// Resolve the recv dispatch function pointer for the given backend mode.
    ///
    /// Returns `recv_uninitialized` if the role cannot recv, ensuring
    /// `ensure_consumer()` is called on first recv to register the participant.
    fn resolve_recv_fn(mode: BackendMode, is_pod: bool, role: TopicRole) -> dispatch::RecvFn<T> {
        if !role.can_recv() {
            return dispatch::recv_uninitialized::<T>;
        }
        match mode {
            BackendMode::FanoutShm => dispatch::recv_fanout_shm::<T>,
            BackendMode::SpscShm if is_pod => dispatch::recv_shm_mpsc_pod::<T>,
            BackendMode::MpscShm if is_pod => dispatch::recv_shm_mpsc_pod::<T>,
            BackendMode::SpmcShm if is_pod => dispatch::recv_shm_spmc_pod::<T>,
            BackendMode::PodShm => dispatch::recv_shm_pod_broadcast::<T>,
            BackendMode::SpscShm => dispatch::recv_shm_mpsc_serde::<T>,
            BackendMode::MpscShm => dispatch::recv_shm_mpsc_serde::<T>,
            BackendMode::SpmcShm => dispatch::recv_shm_spmc_serde::<T>,
            // Fallback for Unknown mode: use MPMC SHM (handles any topology).
            // Must NOT return recv_uninitialized here — that causes infinite recursion
            // when the role is already registered but stale SHM left mode as Unknown.
            BackendMode::Unknown if is_pod => dispatch::recv_shm_mpsc_pod::<T>,
            BackendMode::Unknown => dispatch::recv_shm_mpsc_serde::<T>,
        }
    }

    /// Try to send a message, returning it on failure (for explicit retry).
    ///
    /// Single indirect call — ALL logic (epoch check, ring op, housekeeping)
    /// lives inside the dispatch function. First call goes through
    /// `send_uninitialized` which handles registration + re-dispatch.
    #[inline(always)]
    pub fn try_send(&self, msg: T) -> Result<(), T> {
        // SAFETY: send_fn UnsafeCell is only mutated by this thread (single-owner contract);
        // the fn pointer is always valid — set to send_uninitialized at construction, then
        // updated to a backend-specific function after registration.
        unsafe { (*self.send_fn.get())(self, msg) }
    }

    /// Try to receive a message without logging.
    ///
    /// Single indirect call — ALL logic (epoch check, ring op, housekeeping)
    /// lives inside the dispatch function. First call goes through
    /// `recv_uninitialized` which handles registration + re-dispatch.
    #[inline(always)]
    pub fn try_recv(&self) -> Option<T> {
        // SAFETY: recv_fn UnsafeCell is only mutated by this thread (single-owner contract);
        // the fn pointer is always valid — set to recv_uninitialized at construction, then
        // updated to a backend-specific function after registration.
        unsafe { (*self.recv_fn.get())(self) }
    }

    /// Auto-grow the SHM slot size when a serialized message exceeds the current limit.
    ///
    /// Grows the backing file, remaps the mmap, updates the header's slot_size,
    /// and triggers a migration so all processes pick up the new layout.
    /// Called from the serde send paths when `bytes.len() > max_data_len`.
    ///
    /// Returns `true` if the grow succeeded and the caller should retry the send.
    #[cold]
    #[inline(never)]
    fn auto_grow_slot_size(&self, needed_bytes: usize) -> bool {
        let local = self.local();
        let old_slot_size = local.slot_size;
        // New slot_size: needed_bytes + 16 (overhead) + 25% headroom, rounded up to next power of 2
        let min_slot = needed_bytes + 16 + (needed_bytes / 4);
        let new_slot_size = min_slot.next_power_of_two().max(old_slot_size * 2);

        let capacity = local.cached_capacity as usize;
        if capacity == 0 {
            return false;
        }

        // A colo region has no sequence array, and growing must not
        // reintroduce one: the offsets of every slot would shift by
        // `capacity * 8` and every already-published message would be read
        // from the wrong address.
        let seq_array_size = if self.header().is_colo() {
            0
        } else {
            capacity * std::mem::size_of::<u64>()
        };
        let new_data_size = capacity * new_slot_size;
        let new_total = Self::HEADER_SIZE + seq_array_size + new_data_size;

        // Grow the backing SHM file and remap.
        //
        // The claim that used to sit here — "single-thread ownership, no
        // concurrent readers/writers" — was false, and it was the bug: this runs
        // on the send path of a `Topic` that is routinely cloned across threads,
        // so sibling clones are reading the mapping while it is replaced.
        // `ShmRegion::grow` now serializes grows and keeps the replaced mapping
        // alive; what remains is ours to do, below — re-derive every cached
        // pointer, because the base address has moved.
        if self.storage.grow(new_total).is_err() {
            log::warn!(
                "Topic '{}': failed to grow SHM from {} to {} bytes for slot_size {}",
                self.name,
                self.storage.len(),
                new_total,
                new_slot_size,
            );
            return false;
        }

        // After grow, the mmap may be at a new address. Update header_ptr
        // and all cached pointers BEFORE accessing the header.
        let new_header_ptr = self.storage.as_ptr() as *const TopicHeader;
        self.header_ptr.set(new_header_ptr);

        // Publish the new slot_size.
        //
        // Through `&TopicHeader`, not `&mut`: every other handle and every other
        // process holds a `&TopicHeader` over these same bytes, so forming a
        // `&mut` here was aliasing UB independently of the store itself. The
        // field is atomic for the same reason — `sync_local`,
        // `geometry_is_addressable` and `point_at_slots` all read it from other
        // threads, and as a plain `u32` this was a data race on shared memory.
        // Release so the grown mapping is visible before the geometry that
        // describes it.
        // SAFETY: header_ptr points into the grown ShmRegion.
        let header = unsafe { &*new_header_ptr };
        header
            .slot_size
            .store(new_slot_size as u32, Ordering::Release);

        // Trigger migration so all processes re-sync (picks up new slot_size + pointers).
        let migrator = crate::communication::topic::migration::BackendMigrator::new(header);
        let _ = migrator.migrate_to_optimal();

        // Re-sync local state from the updated header. The mapping was already
        // grown to `new_total` above, so the geometry just published fits it and
        // `sync_local` adopts it.
        Self::sync_local(local, header, false, self.storage.len());

        // Re-derive ALL cached pointers from the (possibly moved) mmap.
        local.cached_header_ptr = new_header_ptr;
        {
            self.point_at_slots(local);
        }

        self.initialize_backend();

        log::info!(
            "Topic '{}': auto-grew slot_size {} → {} bytes ({} total SHM)",
            self.name,
            old_slot_size,
            new_slot_size,
            new_total,
        );
        true
    }

    /// Get or lazily create a TensorPool for spilling large serde messages.
    ///
    /// The pool is created on first call using the topic name as the pool_id
    /// seed (FNV-1a hash), so publisher and subscriber processes converge on
    /// the same shared memory file.
    ///
    /// `None` when the pool cannot be opened or created — a pool file left by a
    /// different `POOL_VERSION` or geometry. Spilling is an optimisation for
    /// messages larger than a ring slot, so the caller falls back to the inline
    /// send it already has for a full pool, rather than panicking mid-tick.
    ///
    /// # Safety
    /// Must be called from the owning thread (Topic is !Sync for mutation).
    /// Uses UnsafeCell — same single-thread guarantee as all other dispatch code.
    pub(crate) fn get_or_create_spill_pool(&self) -> Option<Arc<TensorPool>> {
        // SAFETY: single-thread ownership — Topic<T> is !Send+!Sync for mutation.
        // UnsafeCell access is safe because dispatch functions run on the owning thread.
        let pool_ref = unsafe { &mut *self.spill_pool.get() };
        if let Some(pool) = pool_ref {
            return Some(Arc::clone(pool));
        }
        let pool = pool_registry::pool_or_report(&self.name)?;
        *pool_ref = Some(Arc::clone(&pool));
        Some(pool)
    }

    /// Migration check — reads `migration_epoch` from the SHM header and calls
    /// the cold handler only when it has moved.
    ///
    /// Uses `self.header_ptr` (the stable pointer to the SHM `TopicHeader`)
    /// rather than `local.cached_header_ptr`, which the role=Both same-instance
    /// fast path repurposes. That is what lets cross-process migration be
    /// detected even while the topic is on that local fast path (issue #37).
    ///
    /// This used to be `check_migration_periodic`, a `#[cold] #[inline(never)]`
    /// function — so its three instructions cost a
    /// `call` into `.text.unlikely`, an extra I-cache line, a possible iTLB miss
    /// and register spills across the call, and it marks the calling block cold
    /// so the surrounding code is laid out for a path that is not taken.
    ///
    /// That is affordable at the amortised call sites. It is not affordable on
    /// the EMPTY-recv path, which `recv()` reached on every poll under a comment
    /// reading "Cost: ~50ns ... negligible on the empty-recv path which is
    /// already a 'nothing to do' path". The empty path is not a nothing-to-do
    /// path: it is what a subscriber spins on while waiting, so its cost IS the
    /// detection latency. A 50ns poll means a message published 1ns after a poll
    /// returns is not seen for another 50ns, however fast the transport was.
    ///
    /// Measured: an empty `recv()` cost ~55ns against a raw ring's ~2ns
    /// single-load poll.
    ///
    /// `dispatch::migration_check!` already made exactly this split for the
    /// dispatched paths; this is the same fix for the outer wrapper, which the
    /// macro could not reach.
    #[inline(always)]
    fn check_migration_inline(&self) {
        // SAFETY: header_ptr always points to the real SHM TopicHeader, valid
        // for the topic's lifetime (backed by the Arc<ShmRegion> in `storage`),
        // and is re-pointed by auto-grow whenever the mmap moves.
        let shm_epoch = unsafe { &*self.header_ptr.get() }
            .migration_epoch
            .load(Ordering::Acquire);
        if unlikely(shm_epoch != self.local().cached_epoch) {
            self.handle_epoch_change(shm_epoch);
        }
    }

    /// Handle an epoch change detected by check_migration_periodic.
    ///
    /// Updates cached local state and re-initializes the backend + fn ptrs.
    #[cold]
    #[inline(never)]
    fn handle_epoch_change(&self, _hint_epoch: u64) {
        let local = self.local();
        // SAFETY: header_ptr always points to the real SHM TopicHeader, valid for the
        // Topic lifetime (backed by Arc<ShmRegion> in `storage`).  We use self.header_ptr
        // rather than local.cached_header_ptr because the latter may be stale or null
        // when the topic is still on the role=Both fast path (cached_header_ptr is
        // only updated after sync_local, not before).
        let header = unsafe { &*self.header_ptr.get() };

        // Flush batched updates before migration — but ONLY when the mode is a
        // real SHM backend (not the not-yet-classified `Unknown`, which has no
        // meaningful local head/tail to flush).
        //
        // For SHM→SHM transitions (e.g. SpscShm→MpmcShm), the flush is needed
        // because both backends share the SHM header for head/tail tracking:
        // - SPSC recv batches header.tail every 32 messages; without flushing,
        //   the re-read below gets a stale value.
        // - SP send batches header.sequence_or_head; without flushing, MP
        //   producers doing CAS start from a stale head.
        if local.cached_mode.is_cross_process() {
            if local.role.can_recv() {
                // fetch_max, not store: on SpmcShm this word is not this
                // handle's private position. `recv_shm_spmc_pod` CAS-coordinates
                // competing consumers THROUGH it, and `is_cross_process()`
                // includes SpmcShm, so a handle whose batched local_tail lags
                // the cursor would publish its own value over a claim another
                // consumer had already made -- and the next CAS hands those
                // messages out a second time.
                //
                // The consumer-join flush above already reasons this way and
                // says so: "fetch_max never moves the shared tail backward, so
                // it is safe even if a concurrent SpmcShm consumer has already
                // advanced it". This site published the same value with a plain
                // store. Modelled in tests/loom_spmc_epoch_flush.rs, which goes
                // RED with the store and green with fetch_max.
                //
                // No behaviour change for the single-consumer backends: there
                // the batched local_tail is always at or ahead of the shared
                // word, so the max is the store.
                header.tail.fetch_max(local.local_tail, Ordering::Release);
            }
            if local.role.can_send() {
                header
                    .sequence_or_head
                    .fetch_max(local.local_head, Ordering::Release);
            }
        }

        // Re-read actual epoch from SHM (_hint_epoch may be from process_epoch)
        let actual_epoch = header.migration_epoch.load(Ordering::Acquire);
        local.cached_epoch = actual_epoch;
        // Sync the process-local epoch hint UP to the authoritative SHM epoch.
        // The epoch guards compare `process_epoch` against `cached_epoch`, but
        // `notify_epoch_change` only best-effort updates `process_epoch` (a
        // non-blocking try_lock that SKIPS on contention), so it can lag the SHM
        // `migration_epoch`. If a consumer's `process_epoch` lags, its guard stops
        // firing and it never re-syncs to a producer's migration — reading a stale
        // backend and dropping ALL messages (observed: multi-producer -> 1-consumer
        // delivering 0). fetch_max never regresses a concurrent migrator's higher
        // value, preserving `process_epoch <= migration_epoch`.
        self.process_epoch
            .fetch_max(actual_epoch, Ordering::Release);
        // An epoch change is the second way an untrusted header reaches the index
        // arithmetic, and the easier one: everything below installs `capacity`,
        // `capacity_mask` and `slot_size` from shared memory into the local state
        // that every send and recv indexes with. Attaching is a one-time event a
        // hostile writer has to win a race for; rewriting the geometry of a
        // RUNNING topic and bumping `migration_epoch` is something it can do at
        // any moment.
        //
        // Order matters here. The legitimate reason a peer publishes a geometry
        // larger than this mapping is `auto_grow_slot_size` in another process,
        // and the answer to that is to grow this mapping to match — so the grow
        // comes FIRST, and `sync_local` then sees the mapping this handle will
        // actually use when it decides whether the geometry is addressable. A
        // geometry that is still not addressable afterwards is refused by
        // `sync_local`, which leaves this handle on the last one it validated,
        // and the cached pointers are then re-derived from whichever geometry
        // ended up installed — so the pointer, the mask and the slot size always
        // describe one ring inside one mapping.
        //
        // Every size here is computed with checked arithmetic. The previous form
        // was `HEADER_SIZE + capacity * 8 + capacity * slot_size` unchecked: with
        // `overflow-checks` off — the release profile a robot ships — a capacity
        // and slot size whose product wraps produced a SMALL total, so the grow
        // was skipped and the wrapped geometry was installed anyway.
        let declared_capacity = header.capacity as usize;
        let declared_slot = header.slot_size.load(Ordering::Acquire) as usize;
        // Size the region with the geometry it actually has.
        //
        // This used the SPLIT formula unconditionally, and
        // `colo_required_region_len_checked`'s own doc says why that is wrong:
        // the split form "would demand `capacity * 8` bytes that a colo region
        // correctly does not have". For `Topic<u64>` at capacity 16 the region
        // is 640 + 16*64 = 1664 bytes and the split formula asks for
        // 640 + 128 + 1024 = 1792, so `required > storage.len()` was true for
        // every colo topic and EVERY epoch change fired a grow that nothing
        // needed. Every other reader of this geometry branches on `is_colo()`
        // -- `validate_ring_geometry`, `geometry_is_addressable`, and the
        // `auto_grow_slot_size` path -- and this was the one that did not.
        //
        // On Linux the spurious grow is waste: ftruncate plus a fresh
        // MAP_SHARED view of the same file, so no byte moves. On Windows it is
        // not waste. `grow_unchecked` there re-opens the SAME named section and
        // memcpys the old view onto the new one -- two views of the same
        // physical pages -- so a live ring is copied onto itself while
        // producers are mutating it, and any write landing inside that copy is
        // rolled back. A rolled-back `sequence_or_head` re-issues sequence
        // numbers that were already published, two producers then claim the
        // same slot, and one message is destroyed with no torn read and no
        // duplicate. That is the shape Windows CI reported: 1800 sends all
        // returning Ok, 1798 values delivered.
        let declared_len = if header.is_colo() {
            shm_layout::colo_required_region_len_checked(declared_capacity, declared_slot)
        } else {
            shm_layout::required_region_len_checked(declared_capacity, declared_slot)
        };
        if let Some(required) = declared_len {
            // `geometry_is_addressable(header, required)` is every part of the
            // invariant EXCEPT containment (it holds trivially against
            // `required` itself), so this grows only for a geometry that would
            // be usable once the mapping matches it.
            if required > self.storage.len() && Self::geometry_is_addressable(header, required) {
                // No longer a safety claim: `ShmRegion::grow` is safe under
                // concurrent readers and concurrent growers. The base address
                // still moves, so the pointer re-derivation below is required.
                if self.storage.grow(required).is_ok() {
                    // Only the header pointer moves here. The cached ring pointers
                    // are re-derived below, from the geometry `sync_local` accepts
                    // — deriving them here from `declared_capacity` would let them
                    // describe a different ring than the mask and slot size in use
                    // if the header changed once more in between.
                    self.header_ptr
                        .set(self.storage.as_ptr() as *const TopicHeader);
                }
            }
        }

        // The grow above may have remapped the region, so `header` is re-derived
        // rather than reused.
        let header = unsafe { &*self.header_ptr.get() };
        let region_len = self.storage.len();
        if !Self::geometry_is_addressable(header, region_len) {
            log::error!(
                "Topic '{}': ignoring the ring geometry published with epoch {} \
                 (capacity {}, mask {:#x}, slot_size {}) — it is not addressable \
                 inside this process's {region_len}-byte mapping. Another process \
                 wrote a header this one cannot index; keeping the last valid \
                 geometry.",
                self.name,
                actual_epoch,
                header.capacity,
                header.capacity_mask,
                header.slot_size.load(Ordering::Acquire),
            );
        }
        Self::sync_local(local, header, true, region_len);

        // Re-derive the cached pointers from the mapping as it is NOW and the
        // capacity `sync_local` actually accepted, so the pointer, the mask and
        // the slot size always describe one ring inside one mapping. That
        // agreement is what every `SAFETY: index < capacity` claim in dispatch.rs
        // rests on.
        local.cached_header_ptr = self.storage.as_ptr() as *const TopicHeader;
        self.point_at_slots(local);

        self.initialize_backend();

        // Re-sync head/tail from SHM after initialize_backend, in case an
        // auto-grow remapped the storage. Use header_ptr (always valid).
        let header_post = unsafe { &*self.header_ptr.get() };
        let new_head = header_post.sequence_or_head.load(Ordering::Acquire);
        let shared_tail = header_post.tail.load(Ordering::Acquire);
        local.local_head = new_head;

        // Never pull this consumer's read position backward.
        //
        // `header.tail` is a single shared value. On a broadcast backend each
        // consumer keeps its own independent position, and the shared tail
        // trails the slowest of them — so adopting it wholesale moved every
        // consumer that was ahead *backward*, and its next `recv()` returned an
        // older sequence than one it had already delivered. Measured by
        // `recv_never_reorders_or_duplicates_when_lapped`: 5 inversions across
        // 1,296,457 messages, on 3 of 16 consumers, at migration boundaries —
        // "consumer 6: 23929 then 23497 (backward by 432)".
        //
        // Take the later of the two, then clamp to the head: if the ring itself
        // restarted, head comes back smaller than our stale tail and the clamp
        // puts us at "nothing to read yet" rather than stranding us beyond the
        // producer forever. On a single-consumer backend the shared tail is this
        // consumer's own position, so the max is a no-op.
        local.local_tail = resynced_tail(
            local.local_tail,
            shared_tail,
            new_head,
            local.msg_counter > 0,
        );
        // Propagate to other same-process Topics
        registry::notify_epoch_change(&self.name, actual_epoch);
        self.process_epoch
            .fetch_max(actual_epoch, Ordering::Release);
    }

    /// Get the topic name
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the current backend mode
    #[doc(hidden)]
    pub fn mode(&self) -> BackendMode {
        self.header().mode()
    }

    #[cfg(test)]
    pub fn role(&self) -> TopicRole {
        self.local().role
    }

    #[cfg(test)]
    pub fn migration_metrics(&self) -> &MigrationMetrics {
        &self.metrics
    }

    /// Messages this handle's consumer side skipped past after being lapped.
    ///
    /// Per handle, not per topic: it is this consumer that fell behind. See
    /// `LocalState::missed`.
    pub fn missed_count(&self) -> u64 {
        self.local().missed
    }

    /// Slots this ring can hold. Used to size the producer's keep-alive queue.
    ///
    /// Prefers the capacity `sync_local` validated against this handle's mapping
    /// over a fresh read of the shared header, exactly as `init_shm_backend`
    /// does and for the same reason: this number bounds how many pool slots the
    /// producer pins, and a header rewritten to `u32::MAX` would have it hold a
    /// reference to every frame it ever sent until the pool ran dry. Before this
    /// handle has synced (cached capacity 0) there is nothing validated to use,
    /// so fall back to the header.
    ///
    /// Meant to be read per publish, not cached by the caller: `sync_local`
    /// adopts a new capacity across a grow, and the keep-alive depth has to grow
    /// with it.
    pub(crate) fn ring_capacity(&self) -> u32 {
        match self.local().cached_capacity as u32 {
            0 => self.header().capacity,
            cached => cached,
        }
    }

    /// Record messages this handle lost for a reason the ring itself cannot see.
    ///
    /// The pool-backed types need this: a frame whose backing was released
    /// before the subscriber read it is lost, but the ring dropped nothing and
    /// lapped nobody, so no counter inside the ring has anything to report.
    /// Without this the loss is invisible to `missed_count()` and to
    /// `stats()`, which is the difference between a lossy transport and an
    /// unaccountable one.
    pub(crate) fn note_missed(&self, n: u64) {
        // One binding, read and written through: `local()` hands out a fresh
        // `&mut LocalState` derived from the `UnsafeCell` on every call, and two
        // of them in one statement is a question about evaluation order nobody
        // should have to answer while reading a counter update.
        let local = self.local();
        local.missed = local.missed.wrapping_add(n);
    }

    /// Get a snapshot of the topic's metrics (compatible with Topic API)
    pub fn metrics(&self) -> TopicMetrics {
        TopicMetrics::new(
            self.metrics.messages_sent.load(Ordering::Relaxed),
            self.metrics.messages_received.load(Ordering::Relaxed),
            self.metrics.send_failures.load(Ordering::Relaxed),
            self.metrics.recv_failures.load(Ordering::Relaxed),
        )
    }

    #[cfg(test)]
    pub fn connection_state(&self) -> ConnectionState {
        ConnectionState::from_u8(self.state.load(Ordering::Relaxed))
    }

    /// Send a message (fire-and-forget with bounded retry).
    ///
    /// Fast path for role=Both (same-instance send+recv): the entire ring write is
    /// inlined here — no function pointer indirection, no BackendStorage match,
    /// no epoch_guard. This bypasses 3 levels of call overhead (~8-10ns savings).
    ///
    /// All other backends fall through to the function pointer dispatch, which
    /// includes epoch check, ring operation, and housekeeping.
    #[inline(always)]
    pub fn send(&self, msg: T) {
        // Always-on metric, and the single most expensive instruction left on
        // the publish path. Measured with `topic_probe --send-only`, which has
        // no consumer and therefore no coherence traffic, so it prices local
        // work at +/-4ns instead of the +/-60ns an end-to-end figure gives:
        //
        //     baseline                                 ~95ns
        //     this line deleted                        ~67ns
        //     plain `store` to the SAME address        ~70ns
        //
        // So the cost is ~28ns in that loop, and the third row says where it
        // goes: a plain store to the same address is as cheap as deleting the
        // line, so none of it is the pointer chase or the cache line. It is
        // entirely the `lock` prefix, which drains the store buffer and stops
        // one send from overlapping the next.
        //
        // But that last clause is also the limit of the result, and the number
        // does NOT generalise to latency. `--send-only` issues sends
        // back-to-back, which is exactly the shape the barrier penalises.
        // Measured on the round-trip-paced one-way test instead — one send per
        // cycle, which is what a control loop does — deleting this same line
        // changes nothing: ~159ns median with it, ~162ns without, interleaved,
        // well inside the noise. By the time the peer has answered, the store
        // buffer has drained anyway and the barrier is free.
        //
        // So this is worth ~28ns to burst THROUGHPUT and approximately nothing
        // to control-loop LATENCY. Anyone trading correctness for it should be
        // chasing the former.
        //
        // The comment here used to say the remainder after moving the counter
        // off `migration_epoch`'s cache line "is the lock prefix itself and is
        // the price of an exact count", which read as though it were small. It
        // is not, and this is what a future attempt has to work with:
        //
        //   * `messages_total` is NOT only a metric. `SubscriptionFreshness`
        //     (scheduling/types.rs) watches it as the "new data exists" signal
        //     behind `.subscribe_with_timeout()`, which drives `StalePolicy::
        //     SafeState` and `Stop`. So it cannot be batched every N sends the
        //     way `header.tail` is: a 1 Hz topic with N=32 would look dead for
        //     32 seconds and safe-state a healthy robot.
        //   * It is nonetheless REDUNDANT. Measured across all five backends
        //     (SpscShm, MpscShm, SpmcShm, PodShm, FanoutShm), 100 sends move
        //     `sequence_or_head` by exactly 100 and `messages_total` by exactly
        //     100 — the protocol already maintains this count for free.
        //   * The catch is that they are not the same quantity: this increment
        //     runs BEFORE dispatch, so it counts send *attempts*, while
        //     `sequence_or_head` counts successful claims. They diverge exactly
        //     when a send is dropped on a full ring.
        //
        // Removing it therefore means deciding whether the watchdog should
        // observe attempts or deliveries, and repointing both it and
        // `read_topic_messages_total` at the head — a semantic change on a
        // safety path, not a micro-optimisation. That is the work; it is worth
        // ~28ns of ~95ns, and it should be done on purpose rather than by
        // flipping this line.
        self.header().messages_total.fetch_add(1, Ordering::Relaxed);
        if unlikely(self.is_verbose()) {
            self.send_with_content_logging(msg);
            // Notify event nodes watching this topic. Gated inside
            // `notify_event`; see the note on the fast path below.
            crate::core::NodeInfo::notify_event(&self.name);
            return;
        }
        // Fast path: role=Both (same-instance, same-thread pub+sub) and POD.
        // Bypasses fn ptr indirection + call chain. LocalState fields are in
        // the Topic struct (no pointer chase), and role is on the hot cache line.
        //
        // POD is not an optimisation choice here, it is a correctness one. This
        // path writes the raw Rust value through `cached_data_ptr as *mut T`,
        // which strides by `size_of::<T>()`. That is the ring's real geometry
        // only for a POD type: a non-POD ring's slots are `slot_size` apart —
        // the serde wire buffer — so for `Topic<String>` the path was striding
        // 24 bytes through slots hundreds of bytes wide, writing a heap pointer
        // into a segment other processes map, and never dropping the value it
        // overwrote. `recv`'s twin below read it back with the same wrong
        // stride, so a send/recv pair on one handle agreed with itself and
        // nothing in-process ever noticed; every reader outside the handle saw
        // a head that had stopped moving. Non-POD goes through the dispatched
        // path, which serialises into the real slots and publishes the head.
        let local = self.local();
        if local.role == TopicRole::Both && local.is_pod {
            let head = local.local_head;
            if head.wrapping_sub(local.local_tail) < local.cached_capacity {
                // SAFETY: cached_data_ptr points into the topic's SHM ring data region; the index is
                // masked to ring capacity, so it is always in bounds. The capacity check above
                // ensures the slot is not occupied (no unconsumed data will be overwritten).
                // `simd_aware_write` rather than `ptr::write`, on both branches,
                // for the reason given at its definition: a slot address is
                // whatever the layout put it at, and a typed store to one that
                // is not `align_of::<T>()`-aligned is UB — a `movaps` fault in
                // release for the shapes LLVM vectorises.
                unsafe {
                    let idx = (head & local.cached_capacity_mask) as usize;
                    let (stamp, data) = dispatch::slot_ptrs::<T>(local, idx);
                    if local.cached_colo_stride != 0 {
                        // Under the colo geometry a reader gates on the slot's
                        // own stamp, so this path has to publish one. The split
                        // geometry's readers gate on the head, which is
                        // published just below — that asymmetry is why this
                        // path could previously skip stamping entirely.
                        let seq = head.wrapping_add(1);
                        // Boehm seqlock write phase, and the fence is the whole
                        // protocol — not decoration. A Release *store* on the
                        // marker orders only the accesses BEFORE it; it says
                        // nothing about the payload store that follows, so the
                        // payload may become visible while the stamp still reads
                        // the previous lap's value. `recv_shm_pod_broadcast`
                        // (dispatch.rs) accepts on `v1 == tail + 1` and re-checks
                        // the same stamp after copying, so it would see two
                        // matching stale stamps around new bytes and return a
                        // mixture of two messages. aarch64 -O makes the gap
                        // visible: the marker compiles to `stlr` and the payload
                        // to a plain `str`, with nothing between them; with the
                        // fence it is `str` + `dmb ish` + `str`. Free on x86,
                        // where the fence emits no instruction at all.
                        //
                        // This is the same pairing as dispatch.rs's
                        // `send_shm_pod_broadcast`, `seqlock::seqlock_publish`
                        // and `communication/mod.rs`'s raw publisher; the naive
                        // all-Release form is the one tests/loom_fanout.rs and
                        // tests/loom_pod_broadcast.rs show failing under loom.
                        (*stamp).store(seq | layout::SLOT_WRITING, Ordering::Relaxed);
                        std::sync::atomic::fence(Ordering::Release);
                        simd_aware_write(data, msg);
                        (*stamp).store(seq, Ordering::Release);
                    } else {
                        simd_aware_write(data, msg);
                    }
                }
                local.local_head = head.wrapping_add(1);
                // Publish the ring head. This path used to advance `local_head`
                // and tell nobody, leaving `header.sequence_or_head` frozen
                // wherever it was last synced — and the head is what every
                // reader outside this handle takes as the count of slots
                // written. A topic that sent a few messages through the
                // dispatched path and then gained the consumer role (this path)
                // reported a head of 8 for good while `messages_total` climbed
                // past 300: `horus topic echo` printed exactly one message and
                // then nothing, on a topic publishing the whole time. That is
                // the LIVE-5 symptom reached from the writer's end rather than
                // the reader's, and no probe in the reader can tell a head that
                // stopped being published from one that stopped moving.
                //
                // It also keeps the two write paths agreeing about where the
                // ring is: the dispatched POD send claims its slot with
                // `sequence_or_head.fetch_add`, so with the head left behind it
                // would restart on top of slots this path had already filled the
                // moment one message fell through to it.
                //
                // `fetch_max` rather than `store` because the head must never
                // move backwards for a reader — `read_slot_inner` decides "this
                // message has been lapped" by subtracting from it — and this
                // handle's `local_head` can be behind another producer's.
                //
                self.header()
                    .sequence_or_head
                    .fetch_max(local.local_head, Ordering::Release);
                local.msg_counter = local.msg_counter.wrapping_add(1);
                if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                    self.check_migration_inline();
                }
                // Notify event nodes watching this topic, by topic name.
                //
                // This was NOT a no-op when no event node existed, whatever the
                // comment here used to claim: only the final `fetch_add` was
                // skipped. Every send on every topic locked a process-global
                // `Mutex` and probed a `HashMap<String, _>` to discover there
                // was nothing to notify — and `std::sync::Mutex` is a futex
                // with no priority inheritance, so a SCHED_FIFO publisher could
                // block behind a preempted SCHED_OTHER thread for an unbounded
                // time, right here in the primary publish API.
                //
                // `notify_event` now gates itself on an atomic bit-filter of
                // the registered names: one Relaxed load and a not-taken branch
                // when no event node is registered under this topic's name,
                // which is the common case and the only cost this path pays.
                // The lock is reached only when the name is (or collides with)
                // a registered one — see `core::node::EVENT_NOTIFIER_FILTER`,
                // which also documents the blocking edge that survives for a
                // topic an event node really is watching.
                crate::core::NodeInfo::notify_event(&self.name);
                return;
            }
            // Buffer full — extremely rare for same-thread, fall through to retry
        }
        self.send_lossy(msg);
        // Notify event nodes after the send through the dispatched path. Gated
        // inside `notify_event`; see the note on the fast path above.
        crate::core::NodeInfo::notify_event(&self.name);
    }

    /// Content logging path for send() — outlined to keep send() hot path tight.
    /// Only reached when the verbose flag is set on the topic header.
    #[cold]
    #[inline(never)]
    fn send_with_content_logging(&self, msg: T) {
        let summary = format!("→ {}", self.name);
        let start = std::time::Instant::now();
        self.send_lossy(msg);
        let ipc_ns = start.elapsed().as_nanos() as u64;

        self.metrics.messages_sent.fetch_add(1, Ordering::Relaxed);
        self.state
            .store(ConnectionState::Connected.into_u8(), Ordering::Relaxed);
        use crate::core::hlog::{current_node_name, current_tick_number};
        use crate::core::log_buffer::{publish_log, LogEntry, LogType};
        let now = chrono::Local::now();
        publish_log(LogEntry {
            timestamp: now.format("%H:%M:%S%.3f").to_string(),
            tick_number: current_tick_number(),
            node_name: current_node_name(),
            log_type: LogType::Publish,
            topic: Some(self.name.clone()),
            message: summary,
            tick_us: 0,
            ipc_ns,
        });
    }

    /// Send with bounded retry, dropping the message on failure.
    ///
    /// Hot path: try_send() succeeds → return immediately.
    /// Cold path (queue full): spin retry → yield retry → drop.
    #[inline(always)]
    fn send_lossy(&self, msg: T) {
        match self.try_send(msg) {
            Ok(()) => (),
            Err(returned) => self.send_lossy_retry(returned),
        }
    }

    /// Is this full ring being drained by anybody?
    ///
    /// The first version of the keep-last-N fix asked `sub_count() == 0`, which
    /// answers a different question — "has anyone ever registered as a
    /// subscriber" — and the two diverge the moment a subscriber goes away.
    /// `subscriber_count` had no `Drop`, no deregistration and no reaper, so
    /// after any subscriber had registered once the gate was false forever and
    /// a `kill -9`'d node froze the topic permanently, reproducing the original
    /// report verbatim: `topic list` showing 98 Hz while `topic echo` printed a
    /// single three-minute-old message and then nothing for 45 s.
    ///
    /// Three questions, cheapest first, and each one covers a case the one
    /// before it cannot:
    ///
    /// 1. Nobody registered at all — two atomic loads, and the common case for
    ///    "let me look at what my node publishes".
    /// 2. Somebody registered and their process is gone. `reap_dead_participants`
    ///    takes the registration back, which also repairs `topic info` and
    ///    backend selection, not just this decision.
    /// 3. Somebody registered, is alive, and is not reading. No liveness signal
    ///    can see this one — the process is running and its handle exists — so
    ///    it is settled by the ring itself: `tail` has not moved for a whole
    ///    lease timeout while the ring was full.
    ///
    /// Only reachable from the cold path with a full ring, which is why a
    /// liveness syscall is affordable here at all.
    #[cold]
    #[inline(never)]
    fn nothing_is_draining(&self, header: &TopicHeader) -> bool {
        if header.sub_count() == 0 {
            return true;
        }
        let now_ms = header::current_time_ms();
        header.reap_dead_participants(now_ms);
        if header.sub_count() == 0 {
            return true;
        }
        header.drain_has_stalled(now_ms)
    }

    /// Retry loop for send_lossy — outlined to keep the fast path tight.
    ///
    /// The retry is designed for TRANSIENT failures (ring buffer momentarily
    /// full). For PERMANENT failures (oversized serde messages that will never
    /// fit in the slot), the serde send functions already log a warning before
    /// returning Err, so we limit retries to avoid burning CPU on a message
    /// that can never succeed.
    #[cold]
    #[inline(never)]
    fn send_lossy_retry(&self, mut msg: T) {
        // Check migration before retrying — if the ring is full because we're
        // on the role=Both fast path with no consumer draining it, a cross-process
        // migration will switch to a SHM backend where the remote subscriber is waiting.
        // (GitHub issue #37)
        self.check_migration_inline();

        // First retry immediately — handles the common "buffer was full for
        // a microsecond" case without any spin overhead.
        match self.try_send(msg) {
            Ok(()) => return,
            Err(returned) => msg = returned,
        }

        // A ring nobody reads must not hold the newest data hostage.
        //
        // Backpressure exists to stop a producer overwriting a message some
        // consumer has not taken yet. If nothing is taking messages out, `tail`
        // never moves, so one ring-full after the first send every further
        // `send` fails here — permanently. The shared region then keeps the
        // FIRST `capacity` messages for the life of the segment while
        // `messages_total` (bumped on every call, delivered or not) keeps
        // climbing, so `topic list` and `topic hz` report the topic as live at
        // its real rate while `topic echo` — which reads the ring — replays
        // minutes-old payloads. Measured on a 100 Hz publisher with no
        // subscriber: 14,474 sends, ring content frozen at message 128; and
        // again on a publisher whose subscriber was `kill -9`'d: 28,955 sends,
        // head frozen at 4,736, `topic echo` printing one 3-minute-old message
        // and then nothing.
        //
        // `send` is the lossy publish, so the answer on a ring that is full and
        // unattended is keep-last-N: retire the oldest slot and take it.
        // `try_send` and `send_blocking` are deliberately left alone — their
        // contracts are "tell me the ring is full", not "drop something".
        let header = self.header();
        let capacity = self.local().cached_capacity;
        if capacity > 0 && self.nothing_is_draining(header) {
            let head = header.sequence_or_head.load(Ordering::Acquire);
            // Free exactly one slot; the ring stays as full of recent
            // history as it can be.
            let want_tail = head.saturating_sub(capacity - 1);
            let prev_tail = header.tail.fetch_max(want_tail, Ordering::Release);
            let new_tail = header.tail.load(Ordering::Acquire);
            // Count what the reclaim retired.
            //
            // `fetch_max` returns the tail we replaced, so `want - prev` is
            // exactly the number of accepted-but-unread messages this producer
            // just destroyed. Discarding that return value made keep-last-N the
            // only path in the transport where HORUS itself throws away a
            // message with no counter recording it -- `dropped_count()` moves
            // only on an ABANDONED send, and this send is about to succeed.
            //
            // Both operands come from this one RMW: `want_tail` is the value it
            // published and `prev_tail` the value it replaced, so the difference
            // is what THIS call retired. Re-loading `tail` instead would fold in
            // whatever moved it afterwards -- a consumer's batched
            // `header.tail.store` flush, or a second producer reclaiming on the
            // same ring -- and charge this publisher for messages that were
            // delivered, or count one reclaim on both producers.
            // `saturating_sub` also does the "did we actually advance it" test:
            // a `fetch_max` that lost to a larger tail retired nothing.
            //
            // Folded into `send_failures` so `dropped_count()` stays the single
            // publisher-side loss number a supervisor has to watch; a message
            // retired to make room is lost to its subscriber either way.
            let retired = want_tail.saturating_sub(prev_tail);
            if retired > 0 {
                self.metrics
                    .send_failures
                    .fetch_add(retired, Ordering::Relaxed);
            }
            // We moved `tail` ourselves. Tell the stall detector so it does not
            // mistake the producer's own write for a consumer waking up and
            // restart its grace period on every single reclaim.
            header.note_producer_moved_tail(new_tail);
            // Never backwards. On a role=Both handle `local_tail` is this
            // handle's own READ position, not a cached copy of the shared one,
            // and assigning over it would re-deliver messages it had already
            // taken. Moving it forward is what keep-last-N means for such a
            // handle: the slot just retired is one it will not see.
            let local = self.local();
            local.local_tail = local.local_tail.max(new_tail);
            match self.try_send(msg) {
                Ok(()) => return,
                Err(returned) => msg = returned,
            }
        }

        // If the second attempt also failed, spin briefly. For oversized
        // messages this is wasteful (~50μs), but the warning log in the serde
        // path fires on every attempt so the user sees it.
        const SPIN_ITERS: u32 = 64;
        const YIELD_ITERS: u32 = 4;
        /// How long the yield phase may spend before giving up and dropping.
        ///
        /// `send` is the lossy publish: it never blocks and never fails, it
        /// drops. But `yield_now` hands the CPU to whatever else is runnable and
        /// getting it back is a scheduling decision, not a bounded wait. Four
        /// unconditional yields on a full ring measured 26.4ms mean / 58.6ms
        /// worst with 32 competing threads, and 109.6ms / 199.0ms with 128 — on
        /// a call that is supposed to drop rather than wait, from a control loop
        /// that may be running at 1kHz. A robot running more nodes than it has
        /// cores is oversubscribed by design, so this is the ordinary case.
        ///
        /// 200μs is far above the ~1.4μs the four yields cost on an idle
        /// machine, so nothing changes when the machine is quiet; it only stops
        /// the loop once yielding has demonstrably become expensive.
        const YIELD_BUDGET: std::time::Duration = std::time::Duration::from_micros(200);

        for _ in 0..SPIN_ITERS {
            std::hint::spin_loop();
            match self.try_send(msg) {
                Ok(()) => return,
                Err(returned) => msg = returned,
            }
        }
        let yield_start = std::time::Instant::now();
        for _ in 0..YIELD_ITERS {
            std::thread::yield_now();
            match self.try_send(msg) {
                Ok(()) => return,
                Err(returned) => msg = returned,
            }
            if yield_start.elapsed() >= YIELD_BUDGET {
                break;
            }
        }
        self.metrics.send_failures.fetch_add(1, Ordering::Relaxed);
    }

    /// Send a message, blocking until the ring has space or the timeout expires.
    ///
    /// Unlike [`send()`](Self::send), which drops the message after a brief
    /// spin+yield retry, this reports a full ring instead of swallowing it --
    /// **on the backends that have a full ring at all.**
    ///
    /// # This does NOT guarantee delivery on a broadcast topic
    ///
    /// Every phase below is a retry of `try_send`, so this method can only block
    /// where `try_send` can fail. On `PodShm` and `FanoutShm` it cannot:
    /// `send_shm_pod_broadcast` has one exit and it is `Ok(())`, and
    /// `ShmFanoutRing::send_serde` documents itself as "never blocks", returning
    /// false only for a message too large for a slot. Those backends overwrite
    /// the oldest unread message rather than refuse a new one.
    ///
    /// And you do not choose the backend -- participant count does. One
    /// subscriber gives `SpscShm` and real backpressure; a second subscriber,
    /// including a logger or a `horus topic echo`, silently switches the same
    /// topic to broadcast. At that point this call returns `Ok(())` in
    /// nanoseconds without waiting, and the message it displaced is gone.
    ///
    /// This doc used to say the method "guarantees delivery" and to recommend it
    /// for "critical command topics (emergency stop, motor setpoints) where
    /// message loss is unacceptable". On a topic with two subscribers that was
    /// exactly backwards, and it is the reason
    /// [`provides_backpressure`](Self::provides_backpressure) now exists.
    ///
    /// Assert it on any topic carrying commands — but AFTER the first send or
    /// recv, not before. A handle does not resolve its backend until it moves a
    /// message, and an unresolved backend answers `false` (it is not a promise
    /// of anything), so an assertion placed before the first send fires on every
    /// topic including the ones that are fine. The check is meaningful once the
    /// backend is known, which is also the first moment loss is possible:
    ///
    /// ```ignore
    /// estop.send(Stop)?; // resolves the backend
    /// assert!(
    ///     estop.provides_backpressure(),
    ///     "e-stop settled on a lossy backend ({})",
    ///     estop.backend_name()
    /// );
    /// ```
    ///
    /// Strategy: spin briefly (256 iters), yield briefly (8 iters), then sleep in
    /// 100μs increments until the deadline.
    ///
    /// # Errors
    ///
    /// - [`SendBlockingError::NoBackpressure`] — the backend overwrites unread
    ///   messages and has no full condition to wait on; checked before the first
    ///   `try_send`, so no time is spent waiting
    /// - [`SendBlockingError::Timeout`] — ring buffer stayed full for the entire `timeout`
    pub fn send_blocking(
        &self,
        msg: T,
        timeout: std::time::Duration,
    ) -> Result<(), SendBlockingError> {
        // A backend with no backpressure cannot keep this method's promise. Its
        // send always succeeds and overwrites whatever was there, so phase 1
        // below would return Ok on the first call having guaranteed nothing --
        // on a topic whose documentation names emergency stop as the use case.
        // Refuse instead, and say what to do about it.
        //
        // Resolve the backend first. A freshly constructed handle caches
        // `Unknown` until something forces initialisation, and refusing on that
        // would reject a topic that simply has not decided what it is yet --
        // which is what `send_blocking_serde_type` does, and it is legitimate.
        self.initialize_backend();
        if !self.local().cached_mode.provides_backpressure() {
            return Err(SendBlockingError::NoBackpressure);
        }

        let deadline = std::time::Instant::now() + timeout;

        // Phase 1: try_send (immediate)
        let mut msg = match self.try_send(msg) {
            Ok(()) => return Ok(()),
            Err(returned) => returned,
        };

        // Phase 2: spin (sub-microsecond latency range)
        for _ in 0..256u32 {
            std::hint::spin_loop();
            msg = match self.try_send(msg) {
                Ok(()) => return Ok(()),
                Err(returned) => returned,
            };
        }

        // Phase 3: yield (microsecond range on an idle machine — unbounded on a
        // busy one).
        //
        // Check the deadline BEFORE each yield. `yield_now` hands the CPU to
        // whatever else is runnable, and on a loaded machine getting it back can
        // take tens of milliseconds; eight of those ran unconditionally, so
        // `send_blocking(msg, Duration::ZERO)` — which reads as "try, do not
        // block" — was measured at 56 ms. At 1 kHz that is 56 missed ticks for a
        // caller that asked to wait for nothing. The spin phase above stays
        // unchecked on purpose: 256 `spin_loop` hints cost well under a
        // microsecond, less than the clock read that would guard them.
        for _ in 0..8u32 {
            if std::time::Instant::now() >= deadline {
                return Err(SendBlockingError::Timeout);
            }
            std::thread::yield_now();
            msg = match self.try_send(msg) {
                Ok(()) => return Ok(()),
                Err(returned) => returned,
            };
        }

        // Phase 4: sleep in 100μs increments until deadline
        let sleep_step = 100_u64.us();
        loop {
            if std::time::Instant::now() >= deadline {
                return Err(SendBlockingError::Timeout);
            }
            std::thread::sleep(sleep_step);
            msg = match self.try_send(msg) {
                Ok(()) => return Ok(()),
                Err(returned) => returned,
            };
        }
    }

    /// Receive a message with optional logging.
    ///
    /// Fast path for role=Both (same-instance send+recv): inlined ring read,
    /// no function pointer indirection. Same optimization as send().
    #[inline(always)]
    pub fn recv(&self) -> Option<T> {
        if unlikely(self.is_verbose()) {
            return self.recv_with_content_logging();
        }
        // Fast path: role=Both (same-instance, same-thread pub+sub) and POD.
        //
        // Gated on `is_pod` in lockstep with `send`: this reads back through
        // `cached_data_ptr as *const T`, the same `size_of::<T>()` stride the
        // write used, so the two are only ever correct together. A non-POD
        // topic now sends through the dispatched path, and reading it here
        // would read a slot nothing wrote.
        let local = self.local();
        if local.role == TopicRole::Both && local.is_pod {
            let tail = local.local_tail;
            if local.local_head.wrapping_sub(tail) > 0 {
                // SAFETY: cached_data_ptr points into the topic's SHM ring data region; the index is
                // masked to ring capacity, so it is always in bounds. The head-tail check above
                // ensures the slot contains a valid, initialized message written by send().
                // `simd_aware_read` and not `ptr::read`, mirroring the write
                // side: the slot carries no alignment guarantee for `T`, and a
                // typed load from one is UB.
                let msg = unsafe {
                    let (_, data) = dispatch::slot_ptrs::<T>(
                        local,
                        (tail & local.cached_capacity_mask) as usize,
                    );
                    simd_aware_read(data as *const T)
                };
                local.local_tail = tail.wrapping_add(1);
                local.msg_counter = local.msg_counter.wrapping_add(1);
                if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                    self.check_migration_inline();
                }
                return Some(msg);
            }
            // Empty — amortized epoch check
            local.msg_counter = local.msg_counter.wrapping_add(1);
            if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                self.check_migration_inline();
            }
            return None;
        }
        let result = self.try_recv();
        if result.is_none() {
            // Always check SHM migration_epoch on empty recv.
            //
            // When another process migrates to SHM, this process's dispatch
            // function stays on the role=Both fast path until it detects the epoch change.
            // The dispatch functions only check every EPOCH_CHECK_INTERVAL
            // messages — but a subscriber that never receives anything never
            // reaches that threshold. Checking on every empty recv ensures
            // cross-process migration is detected within one recv() call.
            //
            // Cost: ~50ns (single Acquire load from mmap header). Negligible
            // on the empty-recv path which is already a "nothing to do" path.
            // (GitHub issue #37)
            self.check_migration_inline();
        }
        result
    }

    /// Content logging path for recv() — outlined to keep recv() hot path tight.
    /// Only reached when the verbose flag is set on the topic header.
    #[cold]
    #[inline(never)]
    fn recv_with_content_logging(&self) -> Option<T> {
        let start = std::time::Instant::now();
        let result = self.try_recv();
        let ipc_ns = start.elapsed().as_nanos() as u64;

        if let Some(ref _msg) = result {
            self.metrics
                .messages_received
                .fetch_add(1, Ordering::Relaxed);
            use crate::core::hlog::{current_node_name, current_tick_number};
            use crate::core::log_buffer::{publish_log, LogEntry, LogType};
            let now = chrono::Local::now();
            let summary = format!("← {}", self.name);
            publish_log(LogEntry {
                timestamp: now.format("%H:%M:%S%.3f").to_string(),
                tick_number: current_tick_number(),
                node_name: current_node_name(),
                log_type: LogType::Subscribe,
                topic: Some(self.name.clone()),
                message: summary,
                tick_us: 0,
                ipc_ns,
            });
        }
        result
    }

    /// Force a migration check NOW — reads SHM header epoch, detects optimal
    /// backend, and re-initializes dispatch if the topology changed.
    ///
    /// Useful when you know a cross-process participant has joined/left and
    /// want immediate migration without waiting for the periodic check.
    #[doc(hidden)]
    pub fn check_migration_now(&self) {
        self.check_migration();
    }

    #[cfg(test)]
    pub fn force_migrate(&self, mode: BackendMode) -> MigrationResult {
        let migrator = BackendMigrator::new(self.header());
        let result = migrator.try_migrate(mode);
        if let MigrationResult::Success { new_epoch } = result {
            self.local().cached_epoch = new_epoch;
            self.metrics.migrations.fetch_add(1, Ordering::Relaxed);
            registry::notify_epoch_change(&self.name, new_epoch);
            self.process_epoch.fetch_max(new_epoch, Ordering::Release);
        }
        result
    }

    /// Read the most recent message without advancing the consumer position.
    ///
    /// Unlike `try_recv()`, this always returns the latest published message
    /// regardless of the consumer's current position. Calling it multiple times
    /// returns the same message until a new one is published.
    ///
    /// Useful for reading infrequently-updated or static data (e.g., TF static transforms).
    ///
    /// # `T: Copy` requirement
    ///
    /// Multi-consumer backends (SPMC, MPMC) have a TOCTOU race: between loading
    /// `head` and reading the slot, a consumer can consume the slot via CAS and
    /// drop the value. For types with heap allocations (`String`, `Vec`), this
    /// would be use-after-free. `T: Copy` guarantees no heap pointers — the bytes
    /// in the slot are always safe to bitwise-copy regardless of consumption state.
    pub fn read_latest(&self) -> Option<T>
    where
        T: Copy,
    {
        // Ensure we're registered as a consumer so the header is initialized
        if self.local().role == TopicRole::Unregistered {
            if let Err(e) = self.ensure_consumer() {
                // See `dispatch::recv_uninitialized`: silence here is a
                // subscriber that reads `None` forever and looks idle.
                if dispatch::should_report_endpoint_exhaustion(
                    self.name(),
                    dispatch::Exhausted::ParticipantTable,
                ) {
                    tracing::warn!(
                        "topic '{}': this reader is NOT registered and will see nothing — {}",
                        self.name(),
                        e
                    );
                }
                return None;
            }
        }

        // Role==Both AND POD uses the local fast path (LocalState head/tail
        // over `cached_data_ptr`) regardless of the negotiated backend,
        // including SHM-backed ones, and the SHM header's sequence_or_head is
        // not what that path advances — so read the local counters here, or
        // read_latest returns a phantom value after the data was drained.
        //
        // Non-POD is deliberately excluded, in lockstep with send/recv: it goes
        // through the dispatched path, which maintains the header, and its
        // local counters stand still.
        {
            let local = self.local();
            if local.role == TopicRole::Both && local.is_pod {
                if local.local_tail >= local.local_head {
                    return None;
                }
                let mask = local.cached_capacity_mask;
                let idx = (local.local_head.wrapping_sub(1) & mask) as usize;
                // SAFETY: idx is masked in-bounds; the slot in [tail, head) was
                // written by send() and is initialized. T: Copy — bitwise read.
                let msg = unsafe {
                    let (_, data) = dispatch::slot_ptrs::<T>(local, idx);
                    simd_aware_read(data as *const T)
                };
                return Some(msg);
            }
        }

        // FanoutShm is a broadcast fan-out matrix with no shared head, so it has
        // no meaningful "latest" slot — return None. All other SHM backends fall
        // through to the header-sequence read below.
        // SAFETY: backend UnsafeCell accessed through &self; only this thread mutates it
        match unsafe { &*self.backend.get() } {
            BackendStorage::FanoutShm(_) => {
                return None;
            }
            BackendStorage::ShmData | BackendStorage::Uninitialized => {
                // Fall through to SHM read below
            }
            // SAFETY: `_Phantom` is never constructed (see BackendStorage).
            BackendStorage::_Phantom(_) => unreachable!(),
        }

        // SHM path: read from header sequence counter and data region
        let header = self.header();
        let head = header.sequence_or_head.load(Ordering::Acquire);

        // No messages published yet
        if head == 0 {
            return None;
        }

        let local = self.local();
        // The mask must be the one `sync_local` validated against this mapping,
        // NOT a fresh read of `header.capacity_mask`. Both the mask and `head`
        // above come out of shared memory, so reading the mask here let another
        // process choose `latest_index` outright — `(head - 1) & 0xFFFF_FFFF`
        // scaled by `size_of::<T>()` — and this function RETURNS what it finds
        // there as the message. That is a wrong value delivered to a caller, not
        // just a crash, which on a control topic is the worse of the two.
        let mask = local.cached_capacity_mask;
        let latest_index = ((head.wrapping_sub(1)) & mask) as usize;

        // SAFETY: `cached_data_ptr` and `cached_capacity_mask` are set together
        // from a geometry validated against this mapping, so
        // `latest_index < capacity` and the slot is inside the data region.
        let msg = unsafe {
            let (_, data) = dispatch::slot_ptrs::<T>(local, latest_index);
            simd_aware_read(data as *const T)
        };
        Some(msg)
    }

    /// Check if a message is available without consuming it
    pub fn has_message(&self) -> bool {
        self.pending_count() > 0
    }

    /// Get the number of pending messages
    pub fn pending_count(&self) -> u64 {
        // Role==Both AND POD uses the local fast path in send()/recv()
        // (LocalState head/tail over `cached_data_ptr`), regardless of the
        // negotiated backend. The SHM header's head/tail are not advanced on
        // that path, so read the local counters here or report phantom pending
        // messages after a drain.
        //
        // Non-POD is excluded in lockstep with send/recv — it is dispatched, so
        // the header is the truth and the local counters stand still.
        let local = self.local();
        if local.role == TopicRole::Both && local.is_pod {
            return local.local_head.wrapping_sub(local.local_tail);
        }
        // All non-Both backends (ShmData / FanoutShm / Uninitialized) use the SHM header.
        let header = self.header();
        let head = header.sequence_or_head.load(Ordering::Acquire);
        // The SPSC-SHM consumer batches its read position into
        // `header.tail` (flushed every ~32 messages), so `header.tail`
        // can lag this consumer's true cursor `local_tail` — making
        // pending_count report phantom messages that were already
        // drained. Use whichever position is further ahead. For
        // multi-consumer SHM the shared `header.tail` advances via CAS
        // and is already >= local_tail, so this is a no-op there.
        let tail = header
            .tail
            .load(Ordering::Acquire)
            .max(self.local().local_tail);
        head.saturating_sub(tail)
    }

    /// Get the backend name (for debugging)
    #[doc(hidden)]
    pub fn backend_name(&self) -> &'static str {
        match self.mode() {
            BackendMode::Unknown => "Unknown",
            BackendMode::PodShm => "PodShm",
            BackendMode::SpscShm => "SpscShm",
            BackendMode::SpmcShm => "SpmcShm",
            BackendMode::MpscShm => "MpscShm",
            BackendMode::FanoutShm => "FanoutShm",
        }
    }

    /// Whether a full ring can make this topic's `try_send` refuse a message.
    ///
    /// **This is not a property of the API you call, it is a property of the
    /// backend, and the backend is chosen at runtime from how many participants
    /// are attached.** A topic with one subscriber selects `SpscShm` and refuses
    /// when full. Attach a second subscriber -- a logger, a recorder, a
    /// `horus topic echo` -- and the same topic becomes `PodShm` or `FanoutShm`,
    /// which overwrite instead. `send_shm_pod_broadcast` has a single exit and it
    /// is `Ok(())`; `ShmFanoutRing::send_serde` documents itself as "never
    /// blocks" and returns false only for a message too large for a slot.
    ///
    /// So on a broadcast backend `try_send` cannot report a full ring and
    /// [`send_blocking`](Self::send_blocking) cannot block. Both still compile,
    /// still return, and quietly lose data.
    ///
    /// Assert this on any topic carrying commands rather than samples — after
    /// the first send or recv, not before it. The backend is `Unknown` until a
    /// handle moves a message, and `Unknown` answers `false` because an
    /// unresolved backend promises nothing, so the assertion placed earlier
    /// fires on every topic. Once the backend is known the answer is real, and
    /// that is also the first point at which loss can occur:
    ///
    /// ```ignore
    /// estop.send(Stop)?; // resolves the backend
    /// assert!(
    ///     estop.provides_backpressure(),
    ///     "e-stop topic fell back to a lossy broadcast backend ({})",
    ///     estop.backend_name(),
    /// );
    /// ```
    pub fn provides_backpressure(&self) -> bool {
        backend_provides_backpressure(self.backend_name())
    }

    #[cfg(test)]
    pub fn is_same_process(&self) -> bool {
        self.header().is_same_process()
    }

    #[cfg(test)]
    pub fn is_same_thread(&self) -> bool {
        self.header().is_same_thread()
    }

    /// Get publisher count (for debugging)
    ///
    /// Reaps participants whose process is gone first. Both counts are
    /// registration counts on shared memory that outlives its registrants, so
    /// without that they keep counting nodes that were killed hours ago — and
    /// these two are exactly the numbers someone reads to answer "is anyone
    /// still talking to this topic".
    #[doc(hidden)]
    pub fn pub_count(&self) -> u32 {
        let header = self.header();
        header.reap_dead_participants(header::current_time_ms());
        header.pub_count()
    }

    /// Get subscriber count (for debugging)
    ///
    /// See [`Self::pub_count`]: dead registrants are reaped before counting.
    #[doc(hidden)]
    pub fn sub_count(&self) -> u32 {
        let header = self.header();
        header.reap_dead_participants(header::current_time_ms());
        header.sub_count()
    }

    /// Get raw pointer to the SHM header (for benchmarking raw atomic latency).
    /// Returns null if the topic hasn't been initialized with SHM yet.
    #[doc(hidden)]
    pub fn local_state_header_ptr(&self) -> *const header::TopicHeader {
        self.local().cached_header_ptr
    }
}

impl<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static> Clone for RingTopic<T> {
    fn clone(&self) -> Self {
        Self {
            name: self.name.clone(),
            process_epoch: self.process_epoch.clone(),
            storage: self.storage.clone(),
            backend: std::cell::UnsafeCell::new(BackendStorage::Uninitialized),
            send_fn: std::cell::UnsafeCell::new(dispatch::send_uninitialized::<T>),
            recv_fn: std::cell::UnsafeCell::new(dispatch::recv_uninitialized::<T>),
            local: std::cell::UnsafeCell::new(LocalState::default()),
            header_ptr: self.header_ptr.clone(),
            metrics: Arc::clone(&self.metrics),
            state: AtomicU8::new(self.state.load(Ordering::Relaxed)),
            // Clone shares the spill pool if one was already created
            spill_pool: std::cell::UnsafeCell::new(
                // SAFETY: `RingTopic` has no `Sync` impl, so `&self` here can
                // only be held by one thread — this shared borrow of the
                // `UnsafeCell` cannot alias a concurrent `&mut` from another.
                unsafe { &*self.spill_pool.get() }.clone(),
            ),
            _marker: PhantomData,
        }
    }
}

impl<T> Drop for RingTopic<T> {
    fn drop(&mut self) {
        // COMM-H3: release any spill keep-alives this producer still holds, so a
        // dropped FanoutShm producer doesn't leak its outstanding spill slots.
        // SAFETY: at drop this thread has exclusive ownership of the RingTopic, so
        // no other access to this LocalState / spill_pool can race.
        let local = unsafe { &mut *self.local.get() };
        if !local.spill_keepalive.is_empty() {
            if let Some(pool) = unsafe { &*self.spill_pool.get() } {
                while let Some(t) = local.spill_keepalive.pop_front() {
                    pool.release(&t);
                }
            }
        }

        // COMM-H1 cross-process: on CLEAN drop, clear this instance's FanoutShm
        // endpoint bit + owner PID, THEN release the held flock. A crashed process
        // runs none of this, but the OS releases its flock and a peer reclaims the
        // slot via the dead-owner path. Clearing bit+PID here (before releasing the
        // flock) makes the slot immediately reusable — including by THIS process,
        // whose same-process guard would otherwise refuse to reclaim its own slot.
        if let BackendStorage::FanoutShm(ring) = unsafe { &*self.backend.get() } {
            if let Some(id) = local.fanout_shm_pub_id.take() {
                ring.deregister_publisher(id);
            }
            if let Some(id) = local.fanout_shm_sub_id.take() {
                ring.deregister_subscriber(id);
            }
            // Release the flock(s) AFTER clearing bit+PID (order matters: a held
            // flock blocks any concurrent claim until the bit is already clear).
            local.fanout_pub_lock.take();
            local.fanout_sub_lock.take();
        }

        // Notify network layer (horus_net) that this topic instance is dropped
        notify_topic_lifecycle(TopicLifecycleEvent::Dropped {
            name: self.name.clone(),
        });
    }
}

// ============================================================================
// Logging Support (requires LogSummary bound)
// ============================================================================

// ============================================================================
// Topic<T: TopicMessage> — Public Unified API
// ============================================================================
//
// This wraps RingTopic<T::Wire> with a TopicMessage conversion layer.
// For direct types (CmdVel, i32, etc.), Wire = T → zero overhead.
// For pool-backed types (Image, PointCloud, DepthImage), Wire = XxxDescriptor.

use crate::core::DurationExt;
use crate::memory::depth_image::DepthImage;
use crate::memory::image::Image;
use crate::memory::pointcloud::PointCloud;
use crate::memory::TensorPool;
use crate::types::Tensor;

/// Topic — Universal IPC with automatic backend detection.
///
/// `Topic<T>` provides a single API for all HORUS communication. It automatically
/// selects the optimal backend from 10 paths based on topology and access patterns.
///
/// Works with any type:
/// - **Direct types** (`CmdVel`, `Imu`, `i32`, `String`, ...): zero-overhead pass-through
/// - **Pool-backed types** (`Image`, `PointCloud`, `DepthImage`): automatic zero-copy
///   transport via lightweight descriptors
///
/// # Example
///
/// ```rust,ignore
/// use horus::prelude::*;
///
/// // Direct type — same as before
/// let topic: Topic<CmdVel> = Topic::new("cmd_vel")?;
/// topic.send(CmdVel { linear: 1.0, angular: 0.0 });
///
/// // Pool-backed type — same API!
/// let topic: Topic<Image> = Topic::new("camera.rgb")?;
/// let img = Image::new(640, 480, Rgb8)?;
/// topic.send(&img);
/// let img = topic.recv();
/// ```
pub struct Topic<T: TopicMessage> {
    /// Internal ring buffer typed with the wire format
    ring: RingTopic<T::Wire>,
    /// Pool for pool-backed types (None for direct types)
    pool: Option<Arc<TensorPool>>,
    /// Whether this topic has registered as Publisher in TopicNodeRegistry.
    registered_pub: std::cell::Cell<bool>,
    /// Whether this topic has registered as Subscriber in TopicNodeRegistry.
    registered_sub: std::cell::Cell<bool>,
    /// Node name captured at creation time (for lazy registration).
    owner_node: Option<String>,
    /// How many times `resolve_owner` has looked for a node context and not
    /// found one. Bounds the per-send cost for a topic that has no owner.
    owner_attempts: std::cell::Cell<u16>,
    /// Keep-alive reference(s) to the pool-backed messages this handle has sent
    /// that the ring can still reach. Oldest first; an entry is released once
    /// the ring's capacity has lapped past it (or on Drop), so the producer's
    /// transport reference does not leak. `(pool, primary, secondary)` — the
    /// exact pool the retain was taken on (so release always matches, even for
    /// the auto-pool Tensor path); `secondary` is `Some` only for the
    /// dual-tensor `CostMap`. Stays empty for non-pool-backed message types.
    sent_keepalives:
        std::cell::RefCell<std::collections::VecDeque<(Arc<TensorPool>, Tensor, Option<Tensor>)>>,
}

// SAFETY (Send): an owned `Topic` carries its `RingTopic` (itself `Send`) plus
// `Cell` state that only this handle touches; the `pool` field is an `Arc`
// (Send+Sync). Moving the whole handle transfers that state exclusively.
unsafe impl<T: TopicMessage> Send for Topic<T> where T::Wire: Send {}

// No `Sync` impl, for the same reason as `RingTopic` above: `send`/`recv` take
// `&self` and mutate `registered_pub`/`registered_sub`/`owner_attempts` and the
// `sent_keepalives` `RefCell`. Two threads sharing one `&Topic<Image>` could
// both push and evict from the same keep-alive queue and release the pool slot
// twice — a use-after-free of shared-memory the subscribers are still reading.
// `RefCell` makes `Topic` `!Sync` all by itself, exactly as the `Cell` it
// replaced did.

// Compile-time guard: `Topic` must stay `Send` (owned handles move between
// threads all over the tree) even though it is no longer `Sync`. If a future
// field silently takes `Send` away, this fails to compile here rather than at
// the far end of a `thread::spawn` in a downstream crate.
const _: fn() = || {
    fn assert_send<T: Send>() {}
    assert_send::<Topic<u64>>();
};

/// Release a keep-alive tuple `(pool, primary, secondary)` on its own stored
/// pool. Safe against the drop-oldest ABA: `release` generation-checks and
/// no-ops on a stale/reallocated slot, and generations are monotonic, so a
/// stale descriptor can never match — and therefore never free — a later slot.
#[inline]
fn release_keepalive_tuple(ka: (Arc<TensorPool>, Tensor, Option<Tensor>)) {
    let (pool, primary, secondary) = ka;
    pool.release(&primary);
    if let Some(second) = secondary {
        pool.release(&second);
    }
}

impl<T: TopicMessage> Topic<T> {
    /// Which node owns this handle, resolved as late as possible.
    ///
    /// The name used to be captured in `Topic::new` and never revisited. Node
    /// constructors are where topics are created — and they run before the
    /// scheduler starts, outside any tick — so `current_node_name()` returned
    /// "unknown" and the handle recorded no owner at all. Nothing ever
    /// registered it, and `horus topic info` answered "Publishers: (none)"
    /// about a topic being published at 40 Hz.
    ///
    /// Resolving here instead means the first `send()` — which happens inside
    /// `tick()`, where the context *is* set — finds the name. The constructor's
    /// value still wins when it had one, which covers a topic created inside
    /// `init()` and used from a helper thread.
    ///
    /// This lives on the fully generic impl because every transport shares it.
    /// It used to sit on the `Wire = T` impl, so the zero-copy specialisations
    /// (`Topic<Image>`, `Topic<PointCloud>`, `Topic<DepthImage>`,
    /// `Topic<Tensor>`) still read the constructor-captured `owner_node` — the
    /// value this fix exists because it is empty — and a camera or lidar node,
    /// the central robotics case, went on reporting "Publishers: (none)".
    #[inline]
    fn resolve_owner(&self) -> Option<String> {
        if let Some(ref node) = self.owner_node {
            return Some(node.clone());
        }
        // A topic that belongs to no node — created by a test, a CLI tool, or a
        // helper thread — would otherwise pay this check on every single send
        // forever, because the "registered" flag only latches on success. Give
        // up after a bounded number of attempts so the steady-state cost on the
        // real-time path is exactly zero.
        //
        // The bound is generous on purpose: a node's topic resolves on its
        // first send inside `tick()`, so only genuinely ownerless topics ever
        // count past one.
        if self.owner_attempts.get() >= Self::OWNER_MAX_ATTEMPTS {
            return None;
        }
        // Checked without allocating; `current_node_name` builds a String and
        // is only reached once the context is known to exist.
        if crate::core::hlog::in_node_context() {
            let name = crate::core::hlog::current_node_name();
            if name != "unknown" {
                return Some(name);
            }
        }
        self.owner_attempts.set(self.owner_attempts.get() + 1);
        None
    }

    /// Record this handle as a publisher of its topic, at most once.
    ///
    /// The latch is set **only** when an owner was actually resolved. The
    /// zero-copy specialisations set it outside the `if let`, so the single
    /// attempt they made — in the constructor, before any node context exists —
    /// latched failure permanently and could never be retried from inside a
    /// tick.
    #[inline]
    fn register_pub(&self, type_name: &str) {
        if self.registered_pub.get() {
            return;
        }
        if let Some(node) = self.resolve_owner() {
            topic_node_registry().register_with_type(
                self.ring.name(),
                &node,
                NodeTopicRole::Publisher,
                type_name,
            );
            self.registered_pub.set(true);
        }
    }

    /// Register this handle's role, computing the type name only if it is
    /// actually needed.
    ///
    /// The call sites all read `self.register_sub(Self::registered_type_name())`.
    /// Rust evaluates arguments eagerly, so `registered_type_name()` —
    /// `std::any::type_name::<T>()` followed by an `rsplit("::")` scan — ran on
    /// EVERY send and EVERY recv, including the overwhelming majority where
    /// `register_sub` immediately returned because the handle was already
    /// registered. The work was done and then discarded.
    ///
    /// Measured on an empty `recv()` against a raw ring: 55ns with the eager
    /// argument, 17ns without — i.e. the whole rest of the receive path,
    /// dispatch and all, cost less than this prefix. It is the same shape as the
    /// `ok_or` vs `ok_or_else` slip fixed in `TensorPool::alloc`, on the hottest
    /// path in the crate.
    ///
    /// The `Cell<bool>` check is one load, so the lazy-registration contract is
    /// unchanged: the first call still registers, every later call still does
    /// nothing — it just no longer pays to find out.
    /// Attempts to resolve an owning node before giving up permanently.
    ///
    /// Generous on purpose: a node's topic resolves on its first send inside
    /// `tick()`, so only genuinely ownerless topics — created by a test, a CLI
    /// tool or a helper thread — ever count past one.
    const OWNER_MAX_ATTEMPTS: u16 = 256;

    #[inline(always)]
    fn register_sub_lazy(&self) {
        if unlikely(self.wants_registration(&self.registered_sub)) {
            self.register_sub(Self::registered_type_name());
        }
    }

    /// Whether calling the registration path could still accomplish anything.
    ///
    /// Two `Cell` loads. False once the handle is registered, and false once
    /// `resolve_owner` has given up — at which point it returns `None`
    /// immediately, so the registration call is a no-op whose only effect is the
    /// cost of the argument it was passed.
    #[inline(always)]
    fn wants_registration(&self, latch: &std::cell::Cell<bool>) -> bool {
        !latch.get() && self.owner_attempts.get() < Self::OWNER_MAX_ATTEMPTS
    }

    /// Producer-side twin of [`Self::register_sub_lazy`].
    #[inline(always)]
    fn register_pub_lazy(&self) {
        if unlikely(self.wants_registration(&self.registered_pub)) {
            self.register_pub(Self::registered_type_name());
        }
    }

    /// Record this handle as a subscriber of its topic, at most once.
    #[inline]
    fn register_sub(&self, type_name: &str) {
        if self.registered_sub.get() {
            return;
        }
        if let Some(node) = self.resolve_owner() {
            topic_node_registry().register_with_type(
                self.ring.name(),
                &node,
                NodeTopicRole::Subscriber,
                type_name,
            );
            self.registered_sub.set(true);
        }
    }

    /// The short type name this handle's messages are registered under.
    #[inline]
    fn registered_type_name() -> &'static str {
        let type_name = std::any::type_name::<T>();
        type_name.rsplit("::").next().unwrap_or(type_name)
    }

    /// The pool a pool-backed keep-alive is retained on for `self.pool`-based
    /// topics (Image/PointCloud/DepthImage): `self.pool`, or the global pool.
    #[inline]
    fn keepalive_pool(&self) -> Arc<TensorPool> {
        self.pool
            .as_ref()
            .cloned()
            .unwrap_or_else(pool_registry::fallback_pool)
    }

    /// Record a new pool-backed keep-alive (on `self.pool`) and release the ones
    /// the ring can no longer reach — one reference per ring slot, oldest
    /// evicted first. Subscribers each hold their own `try_from_wire` reference,
    /// so releasing the transport reference here can only ever drop the
    /// producer's ref — never a live reader's.
    #[inline]
    fn publish_keepalive(&self, primary: Tensor, secondary: Option<Tensor>) {
        let pool = self.keepalive_pool();
        self.publish_keepalive_on(pool, primary, secondary);
    }

    /// Like [`publish_keepalive`] but for topics whose pool is not `self.pool`
    /// (the auto-pool `Topic<Tensor>` handle path passes `self.pool()`).
    #[inline]
    fn publish_keepalive_on(
        &self,
        pool: Arc<TensorPool>,
        primary: Tensor,
        secondary: Option<Tensor>,
    ) {
        // Hold one reference per slot the ring can hold, not one in total.
        //
        // This used to be a depth-1 `Cell`: every send released the previous
        // frame's pool reference, so a frame that was still sitting unread in
        // the ring had its backing freed the moment the next one was published.
        // `try_from_wire` then failed its `try_retain` and `recv()` returned
        // `None` while `pending_count()` was non-zero — which also breaks
        // `while let Some(f) = topic.recv()` drain loops.
        //
        // Nothing counted that loss, and correctly so: the ring dropped
        // nothing and lapped nobody, so `dropped_count()` and `missed_count()`
        // had nothing to report. Measured on a realistic pipeline (publisher
        // ~10 kHz, subscriber polling ~2 kHz, separate threads) it was 500
        // frames sent and 1 received, every counter at zero. These are the
        // Image / PointCloud / DepthImage / Tensor types — camera, lidar and
        // depth, the highest-volume payloads a robot carries.
        //
        // The bound is the ring's own capacity because that is exactly how many
        // descriptors can be outstanding. Fewer re-creates the bug for the
        // difference; more pins pool slots the ring can no longer reach.
        //
        // Read on every publish rather than resolved once: `sync_local` adopts a
        // larger capacity across a grow, and a depth frozen at the pre-grow
        // value would free frames the enlarged ring can still hand out — this
        // same bug, in the window after a migration. `ring_capacity` reads local
        // state this handle already caches, not the shared header.
        let depth = self.ring.ring_capacity().max(1) as usize;

        let mut q = self.sent_keepalives.borrow_mut();
        q.push_back((pool, primary, secondary));
        while q.len() > depth {
            if let Some(prev) = q.pop_front() {
                release_keepalive_tuple(prev);
            }
        }
    }
}

impl<T: TopicMessage> Drop for Topic<T> {
    fn drop(&mut self) {
        // Release the final message's keep-alive so it isn't leaked when the
        // producer stops sending. A late subscriber is unaffected: it holds its
        // own reference (`try_from_wire`), which keeps the slot alive.
        for ka in self.sent_keepalives.borrow_mut().drain(..) {
            release_keepalive_tuple(ka);
        }
    }
}

// ============================================================================
// Shared methods — all TopicMessage types
// ============================================================================

#[allow(private_interfaces)]
impl<T: TopicMessage> Topic<T> {
    /// Get the topic name.
    pub fn name(&self) -> &str {
        self.ring.name()
    }

    /// Get a snapshot of the topic's metrics.
    pub fn metrics(&self) -> TopicMetrics {
        self.ring.metrics()
    }

    /// Number of messages dropped by the **producer** because the ring was full.
    ///
    /// This is the count of `send()` calls where the message was discarded
    /// after the bounded spin+yield retry failed.
    ///
    /// This is *not* the way to detect a slow consumer. The broadcast backends
    /// overwrite rather than fail, so their producers never record a drop while
    /// a subscriber that falls a full lap behind loses everything in between —
    /// [`missed_count`](Self::missed_count) is that number, and it is counted on
    /// the consumer side where the loss actually happens.
    ///
    /// # Example
    /// ```rust,ignore
    /// if topic.dropped_count() > 0 {
    ///     horus_core::terminal::eprint_line(&format!(
    ///         "WARNING: publisher dropped {} messages on '{}'",
    ///         topic.dropped_count(),
    ///         topic.name()
    ///     ));
    /// }
    /// ```
    pub fn dropped_count(&self) -> u64 {
        self.ring.metrics().send_failures()
    }

    /// Number of messages **this subscriber** skipped past because the producer
    /// lapped it.
    ///
    /// Drop-oldest under overload is by design: a 10 Hz node reading a 1 kHz
    /// sensor should get the most recent sample, not a backlog. What was missing
    /// is the number — nothing counted the gap, so a subscriber losing 144 of
    /// 400 messages looked identical to one losing none.
    ///
    /// The count is per handle and per direction: it belongs to the consumer
    /// that fell behind, not to the topic, so two subscribers on the same topic
    /// report independently.
    ///
    /// # Example
    /// ```rust,ignore
    /// if topic.missed_count() > 0 {
    ///     horus_core::terminal::eprint_line(&format!(
    ///         "WARNING: fell behind on '{}' — skipped {} messages",
    ///         topic.name(),
    ///         topic.missed_count()
    ///     ));
    /// }
    /// ```
    pub fn missed_count(&self) -> u64 {
        self.ring.missed_count()
    }

    /// Check if a message is available without consuming it.
    pub fn has_message(&self) -> bool {
        self.ring.has_message()
    }

    /// Get the number of pending messages.
    pub fn pending_count(&self) -> u64 {
        self.ring.pending_count()
    }

    /// Get the backend name (for debugging).
    #[doc(hidden)]
    pub fn backend_name(&self) -> &'static str {
        self.ring.backend_name()
    }

    /// Whether a full ring can make this topic's `try_send` refuse a message.
    ///
    /// Returns `false` on the broadcast backends (`PodShm`, `FanoutShm`), which
    /// is what a topic silently becomes once a second subscriber attaches, and
    /// `false` on a backend this handle has not resolved yet — every topic is in
    /// that state until its first send or recv, and an unresolved backend is not
    /// a promise of backpressure. So check this AFTER the first send/recv; see
    /// [`send_blocking()`](Self::send_blocking) for why a command topic should.
    pub fn provides_backpressure(&self) -> bool {
        self.ring.provides_backpressure()
    }

    /// This handle's FanoutShm subscriber endpoint, if it has claimed one.
    ///
    /// A subscriber bumps `sub_count` when it registers, but only becomes an
    /// *addressable* endpoint once it has claimed a slot — and `send_serde`
    /// fans out only to slots that were active when it ran, so an unclaimed
    /// subscriber receives nothing and nothing says so. Tests that need both
    /// subscribers addressable before a measured stream used to prove it by
    /// waiting to receive a warm-up message, which made a local fact depend on
    /// winning a scheduling race against the producer. This is the fact itself.
    #[doc(hidden)]
    pub fn fanout_endpoint_id(&self) -> Option<usize> {
        self.ring.local().fanout_shm_sub_id
    }

    /// Get publisher count.
    #[doc(hidden)]
    pub fn pub_count(&self) -> u32 {
        self.ring.pub_count()
    }

    /// Get subscriber count.
    #[doc(hidden)]
    pub fn sub_count(&self) -> u32 {
        self.ring.sub_count()
    }

    /// Force a migration check NOW.
    #[doc(hidden)]
    pub fn check_migration_now(&self) {
        self.ring.check_migration_now()
    }

    /// Get raw pointer to the SHM header (for benchmarking).
    #[doc(hidden)]
    pub fn local_state_header_ptr(&self) -> *const u8 {
        self.ring.local_state_header_ptr() as *const u8
    }

    /// Get a raw pointer to the SHM sequence/head atomic (for raw latency benchmarking).
    ///
    /// Returns a pointer to the `AtomicU64` used as the producer sequence counter
    /// in shared memory. Returns null if no SHM header is mapped.
    #[doc(hidden)]
    pub fn sequence_head_ptr(&self) -> *const std::sync::atomic::AtomicU64 {
        let header_ptr = self.ring.local_state_header_ptr();
        if header_ptr.is_null() {
            return std::ptr::null();
        }
        // SAFETY: header_ptr is a valid pointer to a TopicHeader in mapped SHM.
        // sequence_or_head is at a fixed offset within the repr(C) struct.
        unsafe { &(*header_ptr).sequence_or_head as *const std::sync::atomic::AtomicU64 }
    }

    /// Get the current backend mode.
    #[doc(hidden)]
    pub fn mode(&self) -> BackendMode {
        self.ring.mode()
    }

    /// Get the topic role.
    #[cfg(test)]
    pub fn role(&self) -> TopicRole {
        self.ring.role()
    }

    /// Get migration metrics.
    #[cfg(test)]
    pub fn migration_metrics(&self) -> &MigrationMetrics {
        self.ring.migration_metrics()
    }

    /// Get connection state.
    #[cfg(test)]
    pub fn connection_state(&self) -> ConnectionState {
        self.ring.connection_state()
    }

    /// Check if topic is on the same thread.
    #[cfg(test)]
    pub fn is_same_thread(&self) -> bool {
        self.ring.is_same_thread()
    }

    /// Check if topic is in the same process.
    #[cfg(test)]
    pub fn is_same_process(&self) -> bool {
        self.ring.is_same_process()
    }

    /// Force migration to a different backend mode.
    #[cfg(test)]
    pub fn force_migrate(&self, mode: BackendMode) -> MigrationResult {
        self.ring.force_migrate(mode)
    }
}

// ============================================================================
// Unified constructor — single `new()` for all TopicMessage types
// ============================================================================

impl<T: TopicMessage> Topic<T>
where
    T::Wire: Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    /// Create a new topic with auto-sized ring buffer capacity.
    ///
    /// Works for all types:
    /// - Direct types (CmdVel, i32, String, ...): zero-overhead pass-through
    /// - Pool-backed types (Image, PointCloud, DepthImage): automatic zero-copy
    pub fn new(name: impl Into<String>) -> HorusResult<Self> {
        let name_str: String = name.into();
        let ring = RingTopic::new(name_str)?;
        let pool = if T::needs_pool() {
            Some(pool_registry::global_pool()?)
        } else {
            None
        };

        // Capture node name for lazy registration.
        // Registration happens on first send() (→ Publisher) or recv() (→ Subscriber).
        let node_name = crate::core::hlog::current_node_name();
        let owner = if node_name != "unknown" {
            Some(node_name)
        } else {
            None
        };

        // Read before `ring` moves into the struct below.
        let keepalive_cap = ring.ring_capacity().max(1) as usize + 1;
        Ok(Self {
            ring,
            pool,
            registered_pub: std::cell::Cell::new(false),
            registered_sub: std::cell::Cell::new(false),
            owner_node: owner,
            owner_attempts: std::cell::Cell::new(0),
            // Sized here, not grown on the publish path. The deque is bounded
            // by the ring depth, so once it holds that many slots it never
            // reallocates -- but starting empty means the FIRST send allocates,
            // and that send is often the one inside an RT context.
            // `rt_alloc_guard_installed::tensor_backed_publish_does_not_allocate`
            // catches exactly that: one violation, on a path whose whole point
            // is that the payload comes from a pool rather than the heap.
            //
            // A later `sync_local` can adopt a larger capacity across a grow and
            // push past this reservation once. That is inherent to growing and
            // is not the steady state this guard is about.
            sent_keepalives: std::cell::RefCell::new(std::collections::VecDeque::with_capacity(
                keepalive_cap,
            )),
        })
    }

    /// Open a topic and refuse to share it with a differently-shaped message.
    ///
    /// [`Topic::new`] validates the message type's short name and, for POD
    /// types, its size. Neither describes field layout, so two revisions of the
    /// same message that keep the name and size but reorder fields both open
    /// the topic and silently reinterpret each other's bytes:
    ///
    /// ```text
    /// v1::Pose { x: f32, y: f32 }   sent (x=1, y=2)
    /// v2::Pose { y: f32, x: f32 }   received Pose { y: 1.0, x: 2.0 }
    /// ```
    ///
    /// No error, no warning — the coordinates simply arrive swapped. That is
    /// what a fleet looks like halfway through a rollout.
    ///
    /// Types declared with [`message!`](crate::message) carry a `LAYOUT_HASH`
    /// and a generated helper that supplies it:
    ///
    /// ```rust,ignore
    /// message! { Pose { x: f32, y: f32 } }
    ///
    /// let tx = Pose::topic("robot.pose")?;          // checked
    /// let tx = Topic::<Pose>::new("robot.pose")?;   // unchecked, as before
    /// ```
    ///
    /// A hash of 0 disables the check. That is what [`Topic::new`] passes and
    /// what headers written before this field existed contain, so an older or
    /// unchecked peer is never rejected — it is simply not protected.
    pub fn new_checked(name: impl Into<String>, layout_hash: u32) -> HorusResult<Self> {
        let topic = Self::new(name)?;
        topic.ring.bind_layout_hash(layout_hash)?;
        Ok(topic)
    }

    /// Create a new topic with a specific kind (ServiceRequest, ActionGoal, etc.).
    pub fn new_with_kind(name: impl Into<String>, topic_kind: u8) -> HorusResult<Self> {
        let name_str: String = name.into();
        let ring = RingTopic::new_with_kind(name_str, topic_kind)?;
        let pool = if T::needs_pool() {
            Some(pool_registry::global_pool()?)
        } else {
            None
        };

        let node_name = crate::core::hlog::current_node_name();
        let owner = if node_name != "unknown" {
            Some(node_name)
        } else {
            None
        };

        // Read before `ring` moves into the struct below.
        let keepalive_cap = ring.ring_capacity().max(1) as usize + 1;
        Ok(Self {
            ring,
            pool,
            registered_pub: std::cell::Cell::new(false),
            registered_sub: std::cell::Cell::new(false),
            owner_node: owner,
            owner_attempts: std::cell::Cell::new(0),
            // Sized here, not grown on the publish path. The deque is bounded
            // by the ring depth, so once it holds that many slots it never
            // reallocates -- but starting empty means the FIRST send allocates,
            // and that send is often the one inside an RT context.
            // `rt_alloc_guard_installed::tensor_backed_publish_does_not_allocate`
            // catches exactly that: one violation, on a path whose whole point
            // is that the payload comes from a pool rather than the heap.
            //
            // A later `sync_local` can adopt a larger capacity across a grow and
            // push past this reservation once. That is inherent to growing and
            // is not the steady state this guard is about.
            sent_keepalives: std::cell::RefCell::new(std::collections::VecDeque::with_capacity(
                keepalive_cap,
            )),
        })
    }

    /// Create a topic, panicking on failure.
    ///
    /// Use this in examples, tests, and simple applications where topic
    /// creation cannot realistically fail (same-process, valid name).
    /// For production code, prefer [`Topic::new()`] which returns `Result`.
    /// Create a new topic with custom capacity.
    pub fn with_capacity(name: &str, capacity: u32, slot_size: Option<usize>) -> HorusResult<Self> {
        let ring = RingTopic::with_capacity(name, capacity, slot_size)?;
        let pool = if T::needs_pool() {
            Some(pool_registry::global_pool()?)
        } else {
            None
        };
        let node_name = crate::core::hlog::current_node_name();
        let owner = if node_name != "unknown" {
            Some(node_name)
        } else {
            None
        };
        // Read before `ring` moves into the struct below.
        let keepalive_cap = ring.ring_capacity().max(1) as usize + 1;
        Ok(Self {
            ring,
            pool,
            registered_pub: std::cell::Cell::new(false),
            registered_sub: std::cell::Cell::new(false),
            owner_node: owner,
            owner_attempts: std::cell::Cell::new(0),
            // Sized here, not grown on the publish path. The deque is bounded
            // by the ring depth, so once it holds that many slots it never
            // reallocates -- but starting empty means the FIRST send allocates,
            // and that send is often the one inside an RT context.
            // `rt_alloc_guard_installed::tensor_backed_publish_does_not_allocate`
            // catches exactly that: one violation, on a path whose whole point
            // is that the payload comes from a pool rather than the heap.
            //
            // A later `sync_local` can adopt a larger capacity across a grow and
            // push past this reservation once. That is inherent to growing and
            // is not the steady state this guard is about.
            sent_keepalives: std::cell::RefCell::new(std::collections::VecDeque::with_capacity(
                keepalive_cap,
            )),
        })
    }
}

// ============================================================================
// Direct types — T: TopicMessage<Wire = T> (CmdVel, i32, String, Tensor, ...)
// ============================================================================
//
// When Wire = T, the wrapper is pure pass-through. No conversion, no pool.

impl<T> Topic<T>
where
    T: TopicMessage<Wire = T> + Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    /// Send a message (fire-and-forget with bounded retry).
    #[inline(always)]
    pub fn send(&self, msg: T) {
        // Lazy registration: first send() registers as Publisher.
        self.register_pub_lazy();
        self.ring.send(msg)
    }

    /// Receive a message.
    #[inline(always)]
    pub fn recv(&self) -> Option<T> {
        // Lazy registration: first recv() registers as Subscriber.
        self.register_sub_lazy();
        self.ring.recv()
    }

    /// Read the most recent message without advancing the consumer position.
    pub fn read_latest(&self) -> Option<T>
    where
        T: Copy,
    {
        // A `read_latest()` consumer is a subscriber. This carried no
        // registration at all, so a node that only ever peeks at the newest
        // sample was invisible to `horus topic info`.
        self.register_sub_lazy();
        self.ring.read_latest()
    }

    /// Try to send a message, returning it on failure (for explicit retry).
    #[inline(always)]
    pub fn try_send(&self, msg: T) -> Result<(), T> {
        // A backpressure-aware publisher is still a publisher; this had no
        // registration block, so `try_send`-only nodes were unattributed.
        self.register_pub_lazy();
        // `messages_total` is not a metric — `SubscriptionFreshness` watches it
        // as the "new data exists" signal behind `.subscribe_with_timeout()`,
        // and that drives `StalePolicy::SafeState` and `Stop`. Only `send()`
        // used to bump it, so a topic driven entirely by `try_send` read as 0
        // messages and 0 Hz while carrying full traffic, and could safe-state
        // or halt a subscriber node whose data was arriving normally. That is
        // the exact failure the counter was introduced to prevent, and these
        // are the APIs the docs send users to on critical topics.
        //
        // Counted before the call, like `send()`: this is an attempt count, and
        // a publisher hammering a full ring is alive — a ring nobody drains is
        // a different fault, and one the subscriber's own staleness sees.
        self.ring.bump_messages_total();
        self.ring.try_send(msg)
    }

    /// Send a message, blocking until the ring has space or the timeout expires.
    ///
    /// Unlike [`send()`](Self::send), which drops the message after a brief
    /// spin+yield retry, this reports a full ring instead of swallowing it --
    /// **on the backends that have a full ring at all.** It backs off in stages
    /// -- spin, then yield, then sleep -- and re-checks the deadline before
    /// every wait after the spin, so a short or zero `timeout` returns near it
    /// rather than overshooting by a scheduling quantum.
    ///
    /// # This does NOT guarantee delivery on a broadcast topic
    ///
    /// Every phase of the wait is a retry of `try_send`, so this can only block
    /// where `try_send` can fail. On the broadcast backends -- `PodShm` and
    /// `FanoutShm` -- it cannot: they overwrite the oldest unread message rather
    /// than refuse a new one, so there is no full ring to wait for.
    ///
    /// And you do not choose the backend -- participant count does. One
    /// subscriber gives `SpscShm` and real backpressure; a second subscriber,
    /// including a logger or a `horus topic echo`, silently switches the same
    /// POD topic to `PodShm` broadcast. Once this handle has resolved that
    /// change (the first send or recv after it does), this call reports
    /// `SendBlockingError::NoBackpressure` instead of the `Ok(())` in
    /// nanoseconds it would otherwise return for a delivery nobody guaranteed.
    ///
    /// This doc used to recommend the method for "critical command topics
    /// (emergency stop, motor setpoints) where message loss is unacceptable".
    /// On a topic with two subscribers that was exactly backwards, and it is the
    /// reason [`provides_backpressure()`](Self::provides_backpressure) exists.
    /// Assert that on any topic carrying commands -- but AFTER the first send or
    /// recv, not before. A handle does not resolve its backend until it moves a
    /// message, and an unresolved backend answers `false` (it is not a promise
    /// of anything), so an assertion placed before the first send fires on every
    /// topic including the ones that are fine:
    ///
    /// ```rust,ignore
    /// estop.send(Stop); // resolves the backend
    /// assert!(
    ///     estop.provides_backpressure(),
    ///     "e-stop settled on a lossy backend"
    /// );
    /// ```
    ///
    /// # Errors
    ///
    /// - `SendBlockingError::NoBackpressure` — this topic's backend overwrites
    ///   unread messages and never reports a full ring, so there is nothing to
    ///   wait on. Refusing is deliberate: reporting success for a delivery
    ///   nobody guaranteed is worse on an e-stop path than failing loudly. It
    ///   does not clear on retry — it lasts as long as the topology that caused
    ///   it, so the fix is to move the topic back to one subscriber or to accept
    ///   the loss with `send()`/`try_send()`.
    /// - `SendBlockingError::Timeout` — the ring stayed full for the entire
    ///   `timeout`.
    pub fn send_blocking(
        &self,
        msg: T,
        timeout: std::time::Duration,
    ) -> Result<(), SendBlockingError> {
        self.register_pub_lazy();
        let result = self.ring.send_blocking(msg, timeout);
        // Same "new data exists" signal as `try_send` above — but NOT for
        // `NoBackpressure`. That refusal is returned before the send is even
        // attempted, so nothing reached the ring; counting it would report a
        // healthy flow on a topic delivering nothing, which is worse than the
        // silence it replaced. A `Timeout` did engage the ring, so it counts as
        // an attempt like `send()`'s dropped message does.
        if !matches!(result, Err(SendBlockingError::NoBackpressure)) {
            self.ring.bump_messages_total();
        }
        result
    }

    /// Low-level receive without logging/recording hooks.
    ///
    /// Prefer `recv()` — it includes the role=Both fast path, logging,
    /// and recording hooks. This exists for internal/test use only.
    #[doc(hidden)]
    #[inline(always)]
    pub fn try_recv(&self) -> Option<T> {
        self.register_sub_lazy();
        self.ring.try_recv()
    }
}

// Clone for direct types
impl<T> Clone for Topic<T>
where
    T: TopicMessage<Wire = T> + Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    fn clone(&self) -> Self {
        // A clone starts empty but will hold the same depth; size it now.
        let keepalive_cap = self.ring.ring_capacity().max(1) as usize + 1;
        Self {
            ring: self.ring.clone(),
            pool: self.pool.clone(),
            registered_pub: self.registered_pub.clone(),
            registered_sub: self.registered_sub.clone(),
            owner_node: self.owner_node.clone(),
            owner_attempts: self.owner_attempts.clone(),
            // A fresh clone has sent nothing yet; each handle tracks and releases
            // only the keep-alives it published itself, so clones never
            // double-release.
            // Sized here, not grown on the publish path. The deque is bounded
            // by the ring depth, so once it holds that many slots it never
            // reallocates -- but starting empty means the FIRST send allocates,
            // and that send is often the one inside an RT context.
            // `rt_alloc_guard_installed::tensor_backed_publish_does_not_allocate`
            // catches exactly that: one violation, on a path whose whole point
            // is that the payload comes from a pool rather than the heap.
            //
            // A later `sync_local` can adopt a larger capacity across a grow and
            // push past this reservation once. That is inherent to growing and
            // is not the steady state this guard is about.
            sent_keepalives: std::cell::RefCell::new(std::collections::VecDeque::with_capacity(
                keepalive_cap,
            )),
        }
    }
}

// ============================================================================
// Image — Topic<Image> send/recv with zero-copy pool transport
// ============================================================================

impl Topic<Image> {
    /// Send an image (zero-copy).
    ///
    /// Accepts both owned and borrowed images: `topic.send(img)` or `topic.send(&img)`.
    /// The pool retain happens in `to_wire`, BEFORE the descriptor reaches the
    /// ring, so a receiver can never observe a descriptor whose tensor is not
    /// already retained. What happens after the send is the keep-alive
    /// bookkeeping: `publish_keepalive` records this frame's slot and releases
    /// the oldest once more than a ring's worth are outstanding.
    pub fn send(&self, img: impl Borrow<Image>) {
        self.register_pub("Image");
        let wire = img.borrow().to_wire(&self.pool);
        // Ring first, then RECORD the keep-alive — not "then retain". The
        // retain itself already happened inside `to_wire` above; what is
        // ordered after the send is which slots this handle holds onto.
        // Recording first meant a frame the ring immediately dropped still
        // displaced the keep-alive of a previously published, still-unread
        // frame.
        self.ring.send(wire);
        self.publish_keepalive(*wire.tensor(), None);
    }

    /// Try to send an image without blocking. Returns `Err(img)` if the ring is full.
    pub fn try_send(&self, img: Image) -> Result<(), Image> {
        self.register_pub("Image");
        let wire = img.to_wire(&self.pool);
        match self.ring.try_send(wire) {
            Ok(()) => {
                self.publish_keepalive(*wire.tensor(), None);
                Ok(())
            }
            Err(w) => Err(Image::from_wire(w, &self.pool)),
        }
    }

    /// Receive the next image.
    pub fn recv(&self) -> Option<Image> {
        self.register_sub("Image");
        // Loop, do not return None on a released frame. A frame whose backing
        // was already released is lost, and the ring dropped nothing and lapped
        // nobody, so nothing inside it can count this — record it here or it is
        // invisible to every counter.
        //
        // Returning None here would ALSO end the caller's drain: the idiomatic
        // `while let Some(f) = topic.recv() {}` would stop at the first
        // released frame and leave every valid frame behind it unread until
        // some later call, which for a consumer draining once per tick means
        // they are still there a tick later, behind a fresh one. Skip the dead
        // entry and keep going; None then means the ring is genuinely empty,
        // which is what the caller reads it as.
        loop {
            let wire = self.ring.recv()?;
            match Image::try_from_wire(wire, &self.pool) {
                Some(v) => return Some(v),
                None => self.ring.note_missed(1),
            }
        }
    }
}

// ============================================================================
// PointCloud — Topic<PointCloud> send/recv with zero-copy pool transport
// ============================================================================

impl Topic<PointCloud> {
    /// Send a point cloud (zero-copy).
    ///
    /// Accepts both owned and borrowed: `topic.send(pc)` or `topic.send(&pc)`.
    pub fn send(&self, pc: impl Borrow<PointCloud>) {
        self.register_pub("PointCloud");
        let wire = pc.borrow().to_wire(&self.pool);
        // Ring first, then RECORD the keep-alive — not "then retain". The
        // retain itself already happened inside `to_wire` above; what is
        // ordered after the send is which slots this handle holds onto.
        // Recording first meant a frame the ring immediately dropped still
        // displaced the keep-alive of a previously published, still-unread
        // frame.
        self.ring.send(wire);
        self.publish_keepalive(*wire.tensor(), None);
    }

    /// Try to send a point cloud without blocking. Returns `Err(pc)` if the ring is full.
    pub fn try_send(&self, pc: PointCloud) -> Result<(), PointCloud> {
        self.register_pub("PointCloud");
        let wire = pc.to_wire(&self.pool);
        match self.ring.try_send(wire) {
            Ok(()) => {
                self.publish_keepalive(*wire.tensor(), None);
                Ok(())
            }
            Err(w) => Err(PointCloud::from_wire(w, &self.pool)),
        }
    }

    /// Receive the next point cloud.
    pub fn recv(&self) -> Option<PointCloud> {
        self.register_sub("PointCloud");
        // Loop, do not return None on a released frame. A frame whose backing
        // was already released is lost, and the ring dropped nothing and lapped
        // nobody, so nothing inside it can count this — record it here or it is
        // invisible to every counter.
        //
        // Returning None here would ALSO end the caller's drain: the idiomatic
        // `while let Some(f) = topic.recv() {}` would stop at the first
        // released frame and leave every valid frame behind it unread until
        // some later call, which for a consumer draining once per tick means
        // they are still there a tick later, behind a fresh one. Skip the dead
        // entry and keep going; None then means the ring is genuinely empty,
        // which is what the caller reads it as.
        loop {
            let wire = self.ring.recv()?;
            match PointCloud::try_from_wire(wire, &self.pool) {
                Some(v) => return Some(v),
                None => self.ring.note_missed(1),
            }
        }
    }
}

// ============================================================================
// DepthImage — Topic<DepthImage> send/recv with zero-copy pool transport
// ============================================================================

impl Topic<DepthImage> {
    /// Send a depth image (zero-copy).
    ///
    /// Accepts both owned and borrowed: `topic.send(depth)` or `topic.send(&depth)`.
    pub fn send(&self, depth: impl Borrow<DepthImage>) {
        self.register_pub("DepthImage");
        let wire = depth.borrow().to_wire(&self.pool);
        // Ring first, then RECORD the keep-alive — not "then retain". The
        // retain itself already happened inside `to_wire` above; what is
        // ordered after the send is which slots this handle holds onto.
        // Recording first meant a frame the ring immediately dropped still
        // displaced the keep-alive of a previously published, still-unread
        // frame.
        self.ring.send(wire);
        self.publish_keepalive(*wire.tensor(), None);
    }

    /// Try to send a depth image without blocking. Returns `Err(depth)` if the ring is full.
    pub fn try_send(&self, depth: DepthImage) -> Result<(), DepthImage> {
        self.register_pub("DepthImage");
        let wire = depth.to_wire(&self.pool);
        match self.ring.try_send(wire) {
            Ok(()) => {
                self.publish_keepalive(*wire.tensor(), None);
                Ok(())
            }
            Err(w) => Err(DepthImage::from_wire(w, &self.pool)),
        }
    }

    /// Receive the next depth image.
    pub fn recv(&self) -> Option<DepthImage> {
        self.register_sub("DepthImage");
        // Loop, do not return None on a released frame. A frame whose backing
        // was already released is lost, and the ring dropped nothing and lapped
        // nobody, so nothing inside it can count this — record it here or it is
        // invisible to every counter.
        //
        // Returning None here would ALSO end the caller's drain: the idiomatic
        // `while let Some(f) = topic.recv() {}` would stop at the first
        // released frame and leave every valid frame behind it unread until
        // some later call, which for a consumer draining once per tick means
        // they are still there a tick later, behind a fresh one. Skip the dead
        // entry and keep going; None then means the ring is genuinely empty,
        // which is what the caller reads it as.
        loop {
            let wire = self.ring.recv()?;
            match DepthImage::try_from_wire(wire, &self.pool) {
                Some(v) => return Some(v),
                None => self.ring.note_missed(1),
            }
        }
    }
}

// ============================================================================
// Tensor — Topic<Tensor> with pool-managed tensor handles
// ============================================================================

impl Topic<Tensor> {
    /// Get or create the auto-managed tensor pool for this topic.
    ///
    /// Fallible for the same reason [`Topic::new`] is: a pool file left behind
    /// by a build with a different `POOL_VERSION` or geometry cannot be opened
    /// *or* recreated, and this used to turn that into a panic on the
    /// per-frame allocation path.
    #[doc(hidden)]
    pub fn pool(&self) -> HorusResult<Arc<TensorPool>> {
        pool_registry::get_or_create_pool(self.ring.name())
    }

    /// Allocate a tensor from this topic's auto-managed pool.
    #[doc(hidden)]
    pub fn alloc_tensor(
        &self,
        shape: &[u64],
        dtype: crate::types::TensorDtype,
        device: crate::types::Device,
    ) -> HorusResult<crate::memory::TensorHandle> {
        let pool = self.pool()?;
        crate::memory::TensorHandle::alloc(pool, shape, dtype, device)
    }

    /// Send a tensor handle through this topic (zero-copy).
    #[doc(hidden)]
    pub fn send_handle(&self, handle: &crate::memory::TensorHandle) {
        self.register_pub("Tensor");
        handle.pool().retain(handle.tensor());
        self.publish_keepalive_on(handle.pool().clone(), *handle.tensor(), None);
        self.ring.send(*handle.tensor());
    }

    /// Receive a tensor and wrap it in a `TensorHandle`.
    #[doc(hidden)]
    pub fn recv_handle(&self) -> Option<crate::memory::TensorHandle> {
        self.register_sub("Tensor");
        let tensor = self.ring.recv()?;
        // An unusable pool reads as "nothing to receive" — `recv_handle` already
        // returns `None` for a superseded slot, and the caller polls again. It
        // is not the same fault, though, so it is not left silent: the caller
        // polling a faulted topic forever gets a throttled line naming it.
        let pool = pool_registry::pool_or_report(self.ring.name())?;
        // Take a generation-guarded reference so each subscriber owns its own: a
        // co-subscriber dropping its handle cannot free the slot out from under
        // us. `Err` => the slot was superseded (drop-oldest) before we read it
        // => missed message. `try_retain` also validates pool_id (as `from_owned`
        // did), discarding a descriptor sent from a different pool.
        if pool.try_retain(&tensor).is_err() {
            return None;
        }
        crate::memory::TensorHandle::from_owned(tensor, pool).ok()
    }
}

#[cfg(test)]
mod type_name_tests {
    use super::strip_module_paths;

    #[test]
    fn a_plain_type_keeps_only_its_name() {
        assert_eq!(
            strip_module_paths("horus_robotics::messages::CmdVel"),
            "CmdVel"
        );
        assert_eq!(strip_module_paths("f64"), "f64");
    }

    #[test]
    fn a_generic_type_keeps_its_structure() {
        // The regression. `rsplit("::").next()` returned `AddTwoIntsResponse>`
        // for this — a stray closing bracket, and no sign of the wrapper the
        // payload is actually inside. Every service topic reported one, and it
        // is what `horus topic info` printed as the message type.
        assert_eq!(
            strip_module_paths(
                "horus_core::services::ServiceResponse<pkg::svc::AddTwoIntsResponse>"
            ),
            "ServiceResponse<AddTwoIntsResponse>"
        );
    }

    #[test]
    fn nested_and_multi_argument_generics_survive() {
        assert_eq!(
            strip_module_paths("a::Outer<b::Inner<c::Leaf>, d::Other>"),
            "Outer<Inner<Leaf>, Other>"
        );
        assert_eq!(
            strip_module_paths("core::option::Option<alloc::vec::Vec<u8>>"),
            "Option<Vec<u8>>"
        );
    }

    #[test]
    fn arrays_tuples_and_references_survive() {
        assert_eq!(strip_module_paths("[pkg::Item; 8]"), "[Item; 8]");
        assert_eq!(strip_module_paths("(pkg::A, pkg::B)"), "(A, B)");
        assert_eq!(strip_module_paths("&pkg::Thing"), "&Thing");
    }

    #[test]
    fn an_empty_or_unqualified_name_is_unchanged() {
        assert_eq!(strip_module_paths(""), "");
        assert_eq!(strip_module_paths("Bare"), "Bare");
    }
}

#[cfg(test)]
mod resync_tail_tests {
    use super::resynced_tail;

    #[test]
    fn a_consumer_ahead_of_the_shared_tail_keeps_its_position() {
        // The regression. On a broadcast backend the shared tail trails the
        // slowest consumer, so a faster one adopting it is pulled backward and
        // re-delivers messages it has already returned.
        assert_eq!(resynced_tail(23_929, 23_497, 30_000, true), 23_929);
    }

    #[test]
    fn a_handle_that_has_delivered_nothing_adopts_the_shared_position() {
        // There is no ordering to preserve yet, and refusing to move backward
        // would strand a just-registered handle ahead of messages published
        // while it was joining — the late-join adjustment can leave `local_tail`
        // well past them.
        assert_eq!(resynced_tail(0, 5_000, 6_000, false), 5_000);
        assert_eq!(resynced_tail(9_000, 5_000, 6_000, false), 5_000);
    }

    #[test]
    fn a_restarted_ring_pulls_a_stale_tail_down_to_the_head() {
        // If the head comes back smaller than our tail the ring restarted;
        // keeping the old position would strand this consumer beyond the
        // producer forever, waiting for sequences that will not arrive.
        assert_eq!(resynced_tail(23_929, 0, 12, true), 12);
    }

    #[test]
    fn a_single_consumer_backend_is_unaffected() {
        // There the shared tail is this consumer's own position.
        assert_eq!(resynced_tail(4_096, 4_096, 8_192, true), 4_096);
    }

    #[test]
    fn the_result_is_never_beyond_the_producer() {
        for (local, shared, head) in [(9u64, 3u64, 5u64), (0, 99, 7), (100, 100, 100)] {
            assert!(resynced_tail(local, shared, head, true) <= head);
            assert!(resynced_tail(local, shared, head, false) <= head);
        }
    }
}

/// The ring geometry in a topic's SHM header, treated as untrusted input.
///
/// `header.rs`'s `untrusted_header_tests` cover the FILE reader
/// (`read_latest_slot_bytes`, what `horus topic echo` uses). These cover the
/// other consumer of the same bytes and the far more dangerous one: the ATTACH
/// path, where a process maps the region and hands `capacity`, `capacity_mask`
/// and `slot_size` to the dispatch functions, which index with them and never
/// bounds-check. Every one of these headers is one a process sharing
/// `/dev/shm` can write.
#[cfg(test)]
mod untrusted_ring_geometry_tests {
    use super::header::{TopicHeader, TOPIC_HEADER_SIZE};
    use super::RingTopic;

    const CAPACITY: u32 = 64;
    const SLOT: u32 = 64;

    /// Write a topic region by hand, with `poison` applied to an otherwise
    /// well-formed header. Returns false when the region cannot be planted, in
    /// which case the test skips.
    ///
    /// Two reasons it can fail. Shared memory may be unavailable outright (the
    /// sandbox). Or the platform may not back regions with files at all: on
    /// Windows a region is pagefile-backed and has no path, so this function's
    /// `fs::write` lands somewhere the SHM layer never reads, the attach below
    /// then creates a clean region and succeeds, and the test asserts nothing
    /// while appearing to pass. Poisoning a Windows region means opening the
    /// named mapping and storing through it, which this helper does not do.
    fn plant_region(name: &str, poison: impl FnOnce(&mut TopicHeader)) -> bool {
        // Let the SHM layer build its directory tree with the ownership and
        // permissions it insists on before writing a region into it by hand.
        // The seed handle is the sole holder, so it unlinks its own file.
        if !horus_sys::shm::regions_are_file_backed() {
            return false;
        }
        match RingTopic::<u64>::new(name) {
            Ok(seed) => drop(seed),
            Err(_) => return false,
        }
        let Some(path) = horus_sys::shm::topic_shm_path_checked(name) else {
            return false;
        };

        let mut header = TopicHeader::zeroed();
        header.init(8, 8, true, CAPACITY, SLOT, "", 0);
        // A zero creator_pid is never treated as stale, so the opener takes this
        // header as live and goes down the validate-an-existing-header branch —
        // the one a joining node uses.
        header.creator_pid = 0;
        poison(&mut header);

        let total = TOPIC_HEADER_SIZE + CAPACITY as usize * 8 + CAPACITY as usize * SLOT as usize;
        let mut buf = vec![0u8; total];
        // SAFETY: `TopicHeader` is repr(C) and this is exactly the byte image the
        // SHM region holds; `buf` is larger than the header.
        unsafe {
            std::ptr::copy_nonoverlapping(
                &header as *const TopicHeader as *const u8,
                buf.as_mut_ptr(),
                TOPIC_HEADER_SIZE,
            );
        }
        std::fs::write(&path, &buf).is_ok()
    }

    fn cleanup(name: &str) {
        if let Some(path) = horus_sys::shm::topic_shm_path_checked(name) {
            let _ = std::fs::remove_file(path);
        }
    }

    /// The mask is the ONLY bound on a slot index: every send computes
    /// `(seq & capacity_mask)` and writes at that index from `cached_data_ptr`
    /// with no further check (`dispatch::send_shm_mp_pod`). A mask wider than
    /// the capacity puts that write outside the mapping.
    #[test]
    fn a_capacity_mask_wider_than_the_capacity_is_refused() {
        let name = format!("untrusted_geom_mask_{}", std::process::id());
        if !plant_region(&name, |h| h.capacity_mask = u32::MAX) {
            eprintln!("skipping: no file-backed shared memory to plant a hostile region in");
            return;
        }
        assert!(
            RingTopic::<u64>::new(&name).is_err(),
            "a mask inconsistent with the capacity must be refused at attach, not \
             handed to the index arithmetic"
        );
        cleanup(&name);
    }

    /// A capacity that is not a power of two has no mask that keeps
    /// `seq & mask` inside the ring, so the whole `& mask` scheme is void.
    #[test]
    fn a_capacity_that_is_not_a_power_of_two_is_refused() {
        let name = format!("untrusted_geom_cap_{}", std::process::id());
        if !plant_region(&name, |h| {
            h.capacity = 63;
            h.capacity_mask = 62;
        }) {
            eprintln!("skipping: no file-backed shared memory to plant a hostile region in");
            return;
        }
        assert!(RingTopic::<u64>::new(&name).is_err());
        cleanup(&name);
    }

    /// `ensure_role` turns `capacity` straight into `cached_data_ptr`
    /// (`HEADER_SIZE + capacity * 8` from the mapping base). A capacity larger
    /// than the mapping puts that pointer past the end of the region before a
    /// single message is sent.
    #[test]
    fn a_capacity_larger_than_the_mapping_is_refused() {
        let name = format!("untrusted_geom_big_{}", std::process::id());
        if !plant_region(&name, |h| {
            h.capacity = 1 << 20;
            h.capacity_mask = (1 << 20) - 1;
        }) {
            eprintln!("skipping: no file-backed shared memory to plant a hostile region in");
            return;
        }
        assert!(
            RingTopic::<u64>::new(&name).is_err(),
            "a ring the mapping cannot hold must be refused, not addressed"
        );
        cleanup(&name);
    }

    /// The serde path multiplies the slot index by `slot_size`, so an oversized
    /// slot walks off the region just as an oversized capacity does.
    #[test]
    fn a_slot_size_larger_than_the_mapping_is_refused() {
        let name = format!("untrusted_geom_slot_{}", std::process::id());
        if !plant_region(&name, |h| {
            h.slot_size
                .store(1 << 24, std::sync::atomic::Ordering::Release)
        }) {
            eprintln!("skipping: no file-backed shared memory to plant a hostile region in");
            return;
        }
        assert!(RingTopic::<u64>::new(&name).is_err());
        cleanup(&name);
    }

    /// Zero slots would make every slot alias slot 0 — and the geometry it
    /// implies is not one any owner writes.
    #[test]
    fn a_zero_slot_size_is_refused() {
        let name = format!("untrusted_geom_zero_{}", std::process::id());
        if !plant_region(&name, |h| {
            h.slot_size.store(0, std::sync::atomic::Ordering::Release)
        }) {
            eprintln!("skipping: no file-backed shared memory to plant a hostile region in");
            return;
        }
        assert!(RingTopic::<u64>::new(&name).is_err());
        cleanup(&name);
    }

    /// The guard must not reject a region a real publisher wrote.
    #[test]
    fn a_consistent_header_still_opens() {
        let name = format!("untrusted_geom_good_{}", std::process::id());
        if !plant_region(&name, |_| {}) {
            eprintln!("skipping: no file-backed shared memory to plant a hostile region in");
            return;
        }
        let topic = RingTopic::<u64>::new(&name).expect(
            "a well-formed region must still open — the guard is not allowed to \
                     break ordinary joining",
        );
        topic.send(7);
        assert_eq!(topic.recv(), Some(7));
        drop(topic);
        cleanup(&name);
    }
}

#[cfg(test)]
mod staleness_signal_tests {
    use super::{SendBlockingError, Topic};
    use std::time::Duration;

    /// `backend_provides_backpressure` must answer for every backend there is,
    /// and answer `false` for anything it does not recognise.
    ///
    /// The Python and C++ bindings only ever see the backend NAME, so this
    /// string table is what they ask. A backend added without a row here would
    /// silently report "no backpressure" — which fails closed, but would make
    /// `send_blocking` refuse on a backend that actually supports it. The match
    /// below is exhaustive over `BackendMode`, so adding a variant stops
    /// compiling here rather than going unnoticed.
    #[test]
    fn every_backend_name_has_a_backpressure_answer() {
        use super::backend_provides_backpressure;
        use super::types::BackendMode;

        let expected = [
            (BackendMode::Unknown, false),
            (BackendMode::PodShm, false),
            (BackendMode::FanoutShm, false),
            (BackendMode::SpscShm, true),
            (BackendMode::SpmcShm, true),
            (BackendMode::MpscShm, true),
        ];

        for (mode, provides) in expected {
            // Mirrors `RingTopic::backend_name`; exhaustive on purpose.
            let name = match mode {
                BackendMode::Unknown => "Unknown",
                BackendMode::PodShm => "PodShm",
                BackendMode::SpscShm => "SpscShm",
                BackendMode::SpmcShm => "SpmcShm",
                BackendMode::MpscShm => "MpscShm",
                BackendMode::FanoutShm => "FanoutShm",
            };
            assert_eq!(
                backend_provides_backpressure(name),
                provides,
                "{name} is on the wrong side of the backpressure table"
            );
        }

        assert!(
            !backend_provides_backpressure("SomethingNewAndUnknown"),
            "an unrecognised backend must fail closed — an unresolved backend \
             is not a promise of backpressure"
        );
    }

    fn cleanup(name: &str) {
        if let Some(path) = horus_sys::shm::topic_shm_path_checked(name) {
            let _ = std::fs::remove_file(path);
        }
    }

    /// `messages_total` is the staleness watchdog's "new data exists" signal.
    ///
    /// `SubscriptionFreshness::refresh` watches it behind
    /// `.subscribe_with_timeout()`, and its answer drives
    /// `StalePolicy::SafeState` and `StalePolicy::Stop`. Only `RingTopic::send`
    /// bumped it, so a topic published entirely through `try_send` read as 0
    /// messages and 0 Hz while carrying every message — and could safe-state or
    /// halt a subscriber whose data was arriving normally. That is precisely
    /// the failure the counter exists to prevent, and `try_send` /
    /// `send_blocking` are the two APIs the docs point users to on critical
    /// topics.
    #[test]
    fn try_send_moves_the_staleness_counter() {
        let name = format!("staleness_try_send_{}", std::process::id());
        cleanup(&name);
        let Ok(topic) = Topic::<u64>::new(&name) else {
            eprintln!("skipping: no shared memory available");
            return;
        };

        assert_eq!(topic.ring.header().messages_total(), 0);
        for i in 0..10u64 {
            let _ = topic.try_send(i);
        }

        assert_eq!(
            topic.ring.header().messages_total(),
            10,
            "ten try_send calls delivered ten messages; a watchdog reading this \
             counter must not conclude the topic is dead"
        );

        drop(topic);
        cleanup(&name);
    }

    /// The same for `send_blocking`, on a topic that has backpressure.
    #[test]
    fn send_blocking_moves_the_staleness_counter() {
        let name = format!("staleness_send_blocking_{}", std::process::id());
        cleanup(&name);
        let Ok(topic) = Topic::<u64>::new(&name) else {
            eprintln!("skipping: no shared memory available");
            return;
        };

        let mut delivered = 0u64;
        for i in 0..10u64 {
            match topic.send_blocking(i, Duration::from_millis(50)) {
                Ok(()) => delivered += 1,
                // A broadcast backend refuses before it ever touches the ring.
                Err(SendBlockingError::NoBackpressure) => {}
                Err(SendBlockingError::Timeout) => delivered += 1,
            }
        }

        assert_eq!(
            topic.ring.header().messages_total(),
            delivered,
            "every send_blocking that engaged the ring must move the counter, \
             and a NoBackpressure refusal — which never reaches the ring — must \
             not, or a topic delivering nothing would read as healthy"
        );
        assert!(
            delivered > 0,
            "precondition: a single-subscriber topic has backpressure, so these \
             sends must have engaged the ring"
        );

        drop(topic);
        cleanup(&name);
    }
}
