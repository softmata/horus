//! # Topic - Universal Smart Detection IPC
//!
//! This module provides fully automatic backend detection for `Topic::new()`.
//! Users just call `send()`/`recv()` and the system auto-detects the optimal
//! backend from 10 paths based on topology and access patterns.
//!
//! ## Detection Matrix
//!
//! | Backend | Latency | Detection Criteria |
//! |---------|---------|-------------------|
//! | DirectChannel | ~3ns | same_thread |
//! | SpscIntra | ~18ns | same_process, pubs=1, subs=1 |
//! | SpmcIntra | ~24ns | same_process, pubs=1, subs>1 |
//! | MpscIntra | ~26ns | same_process, pubs>1, subs=1 |
//! | FanoutIntra | ~36ns | same_process, pubs>1, subs>1 |
//! | FanoutShm | ~40ns | cross_process, pubs>1, subs>1 |
//! | PodShm | ~50ns | cross_process, is_pod |
//! | MpscShm | ~65ns | cross_process, pubs>1, subs=1 |
//! | SpmcShm | ~70ns | cross_process, pubs=1, subs>1 |
//! | SpscShm | ~85ns | cross_process, pubs=1, subs=1, !is_pod |
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
//! Each `Topic<T>` instance must be used from exactly **one thread**. The type
//! is `!Send` and `!Sync` by design (contains `UnsafeCell`, raw pointers, and
//! `Cell`-based local state). This single-thread-per-instance contract enables:
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
pub mod types;

// Shared types and macros — must be declared before ring modules that use the macros
#[macro_use]
pub(crate) mod primitives;

// Per-path optimized backend modules
pub(crate) mod backend;
pub(crate) mod direct_channel;
pub(crate) mod dispatch;
/// Contention-free MPMC via fan-out SPSC matrix.
pub mod fanout;
pub(crate) mod mpsc_intra;
/// Cross-process contention-free MPMC via SHM-backed SPSC matrix.
pub(crate) mod shm_fanout;
pub(crate) mod registry;
pub(crate) mod spmc_intra;
pub(crate) mod spsc_intra;

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

pub(crate) use header::TopicHeader;
pub(crate) use header::{TOPIC_MAGIC, TOPIC_VERSION};

// Public debug flag API for external tools (TUI monitor)
#[doc(hidden)]
pub use header::{
    read_latest_slot_bytes, read_topic_header_info, read_topic_sequence, set_topic_verbose,
    TopicHeaderInfo, TopicKind, TopicSlotRead, TOPIC_VERBOSE_OFFSET,
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

use direct_channel::DirectSlot;

use mpsc_intra::MpscRing;
use spmc_intra::SpmcRing;
use spsc_intra::SpscRing;

// ============================================================================
// SendBlockingError — returned when send_blocking() cannot deliver
// ============================================================================

/// Error returned by [`Topic::send_blocking`] when the message cannot be delivered.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum SendBlockingError {
    /// The ring buffer remained full for the entire timeout duration.
    #[error("send_blocking timed out: ring buffer full")]
    Timeout,
}

impl From<SendBlockingError> for crate::error::HorusError {
    fn from(err: SendBlockingError) -> Self {
        crate::error::HorusError::Communication(crate::error::CommunicationError::TopicFull {
            topic: err.to_string(),
        })
    }
}

// ============================================================================
// RingDrain trait — uniform push interface for drain protocol
// ============================================================================

/// Trait for pushing messages into a ring buffer during migration drain.
///
/// so `drain_old_into_ring` can be generic over the target backend.
trait RingDrain<T> {
    fn push(&self, msg: T) -> Result<(), T>;
}

impl<T> RingDrain<T> for SpscRing<T> {
    #[inline]
    fn push(&self, msg: T) -> Result<(), T> {
        self.try_send(msg)
    }
}

impl<T> RingDrain<T> for SpmcRing<T> {
    #[inline]
    fn push(&self, msg: T) -> Result<(), T> {
        self.try_send(msg)
    }
}

impl<T> RingDrain<T> for MpscRing<T> {
    #[inline]
    fn push(&self, msg: T) -> Result<(), T> {
        self.try_send(msg)
    }
}

impl<T: Clone> RingDrain<T> for fanout::FanoutRing<T> {
    #[inline]
    fn push(&self, msg: T) -> Result<(), T> {
        self.try_send(msg)
    }
}

impl<T, R: RingDrain<T>> RingDrain<T> for Arc<R> {
    #[inline]
    fn push(&self, msg: T) -> Result<(), T> {
        (**self).push(msg)
    }
}

// ============================================================================
// SIMD-Aware Copy Helpers
// ============================================================================

/// Write a message to a ring buffer slot, using SIMD streaming stores for large POD types.
///
/// # Safety
/// `dst` must be valid for writes of `size_of::<T>()` bytes and properly aligned.
#[inline(always)]
unsafe fn simd_aware_write<T>(dst: *mut T, msg: T) {
    if mem::size_of::<T>() >= SIMD_COPY_THRESHOLD {
        simd_copy_to_shm(
            &msg as *const T as *const u8,
            dst as *mut u8,
            mem::size_of::<T>(),
        );
        mem::forget(msg);
    } else {
        std::ptr::write(dst, msg);
    }
}

/// Read a message from a ring buffer slot, using SIMD prefetched reads for large POD types.
///
/// # Safety
/// `src` must be valid for reads of `size_of::<T>()` bytes and properly aligned.
#[inline(always)]
unsafe fn simd_aware_read<T>(src: *const T) -> T {
    if mem::size_of::<T>() >= SIMD_COPY_THRESHOLD {
        let mut msg = mem::MaybeUninit::<T>::uninit();
        simd_copy_from_shm(
            src as *const u8,
            msg.as_mut_ptr() as *mut u8,
            mem::size_of::<T>(),
        );
        msg.assume_init()
    } else {
        std::ptr::read(src)
    }
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

    /// Per-path optimized backend (heap rings for intra-process, SHM for cross-process)
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
    header_ptr: *const TopicHeader,

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

// Safety: RingTopic can be sent between threads
// The UnsafeCell is only accessed through &self with internal synchronization
unsafe impl<T: Send> Send for RingTopic<T> {}
unsafe impl<T: Send + Sync> Sync for RingTopic<T> {}

#[allow(private_interfaces)]
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

        let actual_slot_size = if is_pod {
            let ts = type_size as usize;
            if ts + 8 <= 64 {
                // Co-located layout: [seq(8) | data(T) | pad to 64]
                // Seq + data on SAME cache line for non-SPSC recv paths.
                64
            } else {
                ts
            }
        } else {
            slot_size.unwrap_or(DEFAULT_SLOT_SIZE)
        };

        let actual_capacity = capacity.next_power_of_two() as usize;
        let seq_array_size = actual_capacity * mem::size_of::<u64>();
        let data_size = actual_capacity * actual_slot_size;
        let total_size = Self::HEADER_SIZE + seq_array_size + data_size;

        let storage = Arc::new(ShmRegion::new(name, total_size)?);
        // Extract a short type name for the header (e.g. "CmdVel" from "horus_library::messages::CmdVel")
        let full_type_name = std::any::type_name::<T>();
        let short_type_name = full_type_name.rsplit("::").next().unwrap_or(full_type_name);
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
            header_ptr,
            metrics: Arc::new(MigrationMetrics::default()),
            state: AtomicU8::new(ConnectionState::Connected.into_u8()),
            spill_pool: std::cell::UnsafeCell::new(None),
            _marker: PhantomData,
        })
    }

    /// Negotiate SHM header: initialize if owner, wait if joiner, validate if existing.
    ///
    /// Returns the final slot size from the header.
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

        if header.magic != TOPIC_MAGIC {
            if storage.is_owner() {
                // Fresh SHM — clear stale registry entries from a previous topic lifetime.
                registry::remove_topic(name);
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
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
            let mut backoff_ms = 1u64;
            loop {
                std::thread::sleep(std::time::Duration::from_millis(backoff_ms));
                std::sync::atomic::fence(Ordering::Acquire);
                if header.magic == TOPIC_MAGIC {
                    break;
                }
                if std::time::Instant::now() >= deadline {
                    return Err(HorusError::Communication(
                        "Timeout waiting for topic header initialization"
                            .to_string()
                            .into(),
                    ));
                }
                backoff_ms = (backoff_ms * 2).min(50);
            }
            return Ok(header.slot_size as usize);
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
        if is_pod && header.type_size != type_size {
            return Err(HorusError::Communication(
                format!(
                    "Type size mismatch: {} (expected {})",
                    header.type_size, type_size
                )
                .into(),
            ));
        }
        Ok(header.slot_size as usize)
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

    /// Check if verbose content logging is enabled via the SHM header flag.
    /// Uses the stable `header_ptr` (not `LocalState::cached_header_ptr` which
    /// is repurposed in DirectChannel mode).
    #[inline(always)]
    fn is_verbose(&self) -> bool {
        // SAFETY: header_ptr points into the Arc<ShmRegion> storage which outlives self;
        // the header is initialized before construction completes.
        unsafe { (*self.header_ptr).is_verbose() }
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
    #[inline(always)]
    fn sync_local(local: &mut LocalState, header: &TopicHeader, skip_stale_broadcast: bool) {
        local.is_same_process = header.is_same_process();
        local.is_pod = header.is_pod_type();
        local.slot_size = header.slot_size as usize;
        local.cached_mode = header.mode();
        local.cached_capacity = header.capacity as u64;
        local.cached_capacity_mask = header.capacity_mask as u64;
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        local.local_tail = header.tail.load(Ordering::Acquire);
        if skip_stale_broadcast && local.cached_mode == BackendMode::PodShm {
            local.local_tail = local.local_head;
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
        let cap = header.capacity as usize;
        // SAFETY: HEADER_SIZE offset is within bounds of allocated storage region
        local.cached_seq_ptr = unsafe { self.storage.as_ptr().add(Self::HEADER_SIZE) as *mut u8 };
        // SAFETY: data_region_offset(cap) is within bounds of allocated storage region
        local.cached_data_ptr =
            unsafe { self.storage.as_ptr().add(Self::data_region_offset(cap)) as *mut u8 };
        local.cached_epoch = header.migration_epoch.load(Ordering::Acquire);
        Self::sync_local(local, header, false);
        // Set role LAST — this gates the fast path via can_send()/can_recv()
        local.role = if is_producer {
            if local.role == TopicRole::Consumer {
                TopicRole::Both
            } else {
                TopicRole::Producer
            }
        } else {
            if local.role == TopicRole::Producer {
                TopicRole::Both
            } else {
                TopicRole::Consumer
            }
        };

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
            Self::sync_local(local, header, true);
            for _ in 0..MAX_EPOCH_RETRIES {
                let reloaded = header.migration_epoch.load(Ordering::Acquire);
                if reloaded == stable_epoch {
                    break; // Epoch unchanged — local state is consistent.
                }
                // Concurrent migration advanced the epoch while we were
                // syncing; re-sync to the newer epoch and check again.
                stable_epoch = reloaded;
                local.cached_epoch = stable_epoch;
                Self::sync_local(local, header, true);
            }
            // Re-initialize backend for the new (stable) epoch.
            //
            // `initialize_backend()` short-circuits when `backend_matches_mode`
            // is true — it checks only the *type* of the backend, not the epoch.
            // If the backend MODE is the same across epochs (e.g. DirectChannel
            // epoch 0 → DirectChannel epoch 2 after a double migration) the
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
        }

        let migrator = BackendMigrator::new(header);
        if !migrator.is_optimal() {
            for attempt in 0u32..5 {
                match migrator.migrate_to_optimal() {
                    MigrationResult::Success { new_epoch } => {
                        local.cached_epoch = new_epoch;
                        Self::sync_local(local, header, true);
                        self.metrics.migrations.fetch_add(1, Ordering::Relaxed);
                        // Notify all same-process Topics of the epoch change
                        registry::notify_epoch_change(&self.name, new_epoch);
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
                            Self::sync_local(local, header, true);
                            registry::notify_epoch_change(&self.name, new_epoch);
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
            let timeout = header.lease_timeout_ms as u64;
            let now = current_time_ms();
            header.participants[local.slot_index as usize].refresh_lease(now, timeout);
        }
    }

    /// Initialize or re-initialize the per-path optimized backend based on current mode.
    ///
    /// For intra-process modes, creates heap-backed rings and stores them in the
    /// global registry so all participants share the same Arc. When switching from
    /// SHM or a different heap ring, drains pending messages into the new backend.
    ///
    /// For cross-process modes, creates an ShmData backend pointing to the SHM region.
    fn initialize_backend(&self) {
        let local = self.local();
        let mode = local.cached_mode;
        let epoch = local.cached_epoch;
        let is_pod = local.is_pod;
        let capacity = local.cached_capacity as u32;

        // SAFETY: backend UnsafeCell accessed through &self; only this thread mutates it
        let backend = unsafe { &mut *self.backend.get() };

        if Self::backend_matches_mode(backend, mode) {
            // Backend matches but fn ptrs may be stale (e.g., first call after construction).
            self.set_dispatch_fn_ptrs(mode, is_pod);
            return;
        }

        if mode.is_intra_process() {
            let cap = if capacity == 0 { 64 } else { capacity };
            self.init_intra_backend(backend, mode, epoch, cap);
        } else {
            self.init_shm_backend(backend, epoch);
        }

        self.set_dispatch_fn_ptrs(mode, is_pod);
    }

    /// Check if the current backend storage already matches the requested mode.
    fn backend_matches_mode(backend: &BackendStorage<T>, mode: BackendMode) -> bool {
        match (backend, mode) {
            (BackendStorage::DirectChannel(_), BackendMode::DirectChannel) => true,
            (BackendStorage::SpscIntra(_), BackendMode::SpscIntra) => true,
            (BackendStorage::SpmcIntra(_), BackendMode::SpmcIntra) => true,
            (BackendStorage::MpscIntra(_), BackendMode::MpscIntra) => true,
            (BackendStorage::FanoutIntra(_), BackendMode::FanoutIntra) => true,
            (BackendStorage::FanoutShm(_), BackendMode::FanoutShm) => true,
            (BackendStorage::ShmData, _)
                if mode.is_cross_process() || mode == BackendMode::Unknown =>
            {
                true
            }
            _ => false,
        }
    }

    /// Initialize an intra-process backend (DirectChannel or ring buffer).
    ///
    /// Looks up existing rings from the registry first, then creates new ones
    /// if needed. Drains old data during backend transitions.
    fn init_intra_backend(
        &self,
        backend: &mut BackendStorage<T>,
        mode: BackendMode,
        epoch: u64,
        cap: u32,
    ) {
        // First, try to look up an existing ring from the registry.
        // Another participant may have already created one for this (topic, epoch).
        if let Some(existing) = registry::lookup_backend(&self.name, epoch) {
            if self.try_set_backend_from_registry(backend, &existing, mode) {
                return;
            }
        }

        // Create a new intra-process ring, drain old data, and register it.
        // All 4 ring types (SPSC/SPMC/MPSC/MPMC) follow identical logic:
        // create → drain old → store_or_get from registry → downcast.
        macro_rules! create_intra_ring {
            ($Ring:ty, $Variant:ident, $cap:expr) => {{
                let new_ring = Arc::new(<$Ring>::new($cap));
                self.drain_old_into_ring(&new_ring, epoch);
                let shared = registry::store_or_get_backend(
                    &self.name,
                    epoch,
                    new_ring.clone() as Arc<dyn std::any::Any + Send + Sync>,
                );
                *backend = if let Ok(ring) = shared.downcast::<$Ring>() {
                    BackendStorage::$Variant(ring)
                } else {
                    BackendStorage::$Variant(new_ring)
                };
            }};
        }

        match mode {
            BackendMode::DirectChannel => {
                let new_ring = Arc::new(DirectSlot::new(cap));
                self.drain_old_into_direct(&new_ring, epoch);
                let shared = registry::store_or_get_backend(
                    &self.name,
                    epoch,
                    new_ring.clone() as Arc<dyn std::any::Any + Send + Sync>,
                );
                *backend = if let Ok(slot) = shared.downcast::<DirectSlot<T>>() {
                    BackendStorage::DirectChannel(slot)
                } else {
                    BackendStorage::DirectChannel(new_ring)
                };
            }
            BackendMode::SpscIntra => create_intra_ring!(SpscRing<T>, SpscIntra, cap),
            BackendMode::SpmcIntra => create_intra_ring!(SpmcRing<T>, SpmcIntra, cap),
            BackendMode::MpscIntra => create_intra_ring!(MpscRing<T>, MpscIntra, cap),
            BackendMode::FanoutIntra => {
                // FanoutRing doesn't implement RingDrain, so we skip draining
                // (messages from old backend are lost during migration — acceptable
                // since migration happens when topology changes, not during steady state)
                let new_ring = Arc::new(fanout::FanoutRing::<T>::new(16, 16, cap as usize));
                let shared = registry::store_or_get_backend(
                    &self.name,
                    epoch,
                    new_ring.clone() as Arc<dyn std::any::Any + Send + Sync>,
                );
                *backend = if let Ok(ring) = shared.downcast::<fanout::FanoutRing<T>>() {
                    BackendStorage::FanoutIntra(ring)
                } else {
                    BackendStorage::FanoutIntra(new_ring)
                };
            }
            _ => {
                // Unrecognized intra-process mode — keep ShmData
                *backend = BackendStorage::ShmData;
            }
        }
    }

    /// Initialize a cross-process SHM backend, restoring cached pointers.
    ///
    /// Drains old heap ring messages into SHM and restores cached data/seq
    /// pointers that may have been overwritten by DirectChannel setup.
    fn init_shm_backend(&self, backend: &mut BackendStorage<T>, epoch: u64) {
        let mode = BackendMode::from(self.header().backend_mode.load(Ordering::Acquire));

        // FanoutShm: create ShmFanoutRing backed by a separate SHM region
        if mode == BackendMode::FanoutShm {
            let is_pod = crate::communication::pod::is_pod::<T>();
            let type_size = mem::size_of::<T>();
            let capacity = self.header().capacity;
            let total_size = shm_fanout::ShmFanoutRing::required_file_size(
                type_size,
                is_pod,
                capacity as usize,
            );
            let fanout_name = format!("{}_fanout", self.name);
            if let Ok(fanout_storage) = crate::memory::shm_region::ShmRegion::new(&fanout_name, total_size) {
                let is_owner = fanout_storage.is_owner();
                let shm_base = fanout_storage.as_ptr() as *mut u8;
                let ring = unsafe {
                    if is_owner {
                        shm_fanout::ShmFanoutRing::init_owner(shm_base, type_size, is_pod, capacity)
                    } else {
                        shm_fanout::ShmFanoutRing::attach(shm_base, is_pod, type_size)
                    }
                };
                // Store the SHM region in local state so it stays alive
                self.local().fanout_shm_storage = Some(std::sync::Arc::new(fanout_storage));
                *backend = BackendStorage::FanoutShm(Box::new(ring));
                return;
            }
            // Fallback to ShmData if fanout SHM creation fails
        }

        // Standard SHM backends: use ShmData.
        // Drain old heap ring messages into SHM before switching.
        self.drain_old_into_shm(epoch);

        *backend = BackendStorage::ShmData;

        // Restore SHM cached pointers. DirectChannel setup overwrites
        // cached_data_ptr/cached_seq_ptr to point at the DirectSlot heap
        // buffer. SHM dispatch functions rely on these pointing into the
        // mmap'd storage region. Without this restore, a DC→SHM migration
        // (e.g., when a cross-process participant joins) would cause SHM
        // dispatch to read/write the DirectSlot buffer instead of SHM — UB.
        let local = self.local();
        let cap = local.cached_capacity as usize;
        // SAFETY: HEADER_SIZE and data_region_offset are within storage bounds
        local.cached_seq_ptr = unsafe { self.storage.as_ptr().add(Self::HEADER_SIZE) as *mut u8 };
        // SAFETY: data_region_offset(cap) is within the storage bounds (capacity was validated
        // when the SHM region was mapped); storage.as_ptr() is a valid non-null pointer.
        local.cached_data_ptr =
            unsafe { self.storage.as_ptr().add(Self::data_region_offset(cap)) as *mut u8 };
    }

    /// Set dispatch function pointers based on the current backend mode and POD status.
    ///
    /// Called at the end of `initialize_backend()` to resolve the data path at
    /// initialization time. After this, `try_send()`/`try_recv()` are single
    /// indirect calls with zero branches.
    fn set_dispatch_fn_ptrs(&self, mode: BackendMode, is_pod: bool) {
        // Co-located layout: sizeof(T) + 8 <= 64, slot_size == 64
        // Seq and data share a cache line → single inter-core transfer
        // for non-SPSC recv paths (MpscShm, SpmcShm, MpmcShm, PodShm).
        // Disabled for cross-process SHM: when a topic transitions from
        // DirectChannel to SHM (on second participant), the dispatch must
        // be consistent across migration boundaries.
        let colo = is_pod && mem::size_of::<T>() + 8 <= 64 && !mode.is_cross_process();
        let local = self.local();
        let role = local.role;

        if mode == BackendMode::DirectChannel {
            self.cache_direct_channel_ptrs(local);
        }

        // SAFETY: UnsafeCell accessed from single thread (same guarantee as backend/local)
        unsafe {
            *self.send_fn.get() = Self::resolve_send_fn(mode, is_pod, colo, role);
            *self.recv_fn.get() = Self::resolve_recv_fn(mode, is_pod, colo, role);
        }
    }

    /// Cache DirectSlot pointers into LocalState so dispatch functions skip
    /// BackendStorage traversal entirely.
    fn cache_direct_channel_ptrs(&self, local: &mut LocalState) {
        // SAFETY: backend UnsafeCell accessed from single thread
        if let BackendStorage::DirectChannel(slot) = unsafe { &*self.backend.get() } {
            local.cached_data_ptr = slot.buffer.as_ptr() as *mut u8;
            local.cached_capacity = slot.capacity;
            local.cached_capacity_mask = slot.mask;
            if local.role == TopicRole::Both {
                // DC-local: plain u64 head/tail (single-instance optimization)
                local.local_head = slot.head.load(Ordering::Relaxed);
                local.local_tail = slot.tail.load(Ordering::Relaxed);
            } else {
                // DC-cached: cache pointers to atomic head/tail for separate instances
                local.cached_header_ptr = &slot.head as *const AtomicU64 as *const TopicHeader;
                local.cached_seq_ptr = &slot.tail as *const AtomicU64 as *mut u8;
            }
        }
    }

    /// Resolve the send dispatch function pointer for the given backend mode.
    ///
    /// Returns `send_uninitialized` if the role cannot send, ensuring
    /// `ensure_producer()` is called on first send to register the participant.
    fn resolve_send_fn(
        mode: BackendMode,
        is_pod: bool,
        colo: bool,
        role: TopicRole,
    ) -> dispatch::SendFn<T> {
        if !role.can_send() {
            return dispatch::send_uninitialized::<T>;
        }
        match mode {
            BackendMode::DirectChannel if role == TopicRole::Both => {
                dispatch::send_direct_channel_local::<T>
            }
            BackendMode::DirectChannel => dispatch::send_direct_channel_cached::<T>,
            BackendMode::SpscIntra => dispatch::send_spsc_intra::<T>,
            BackendMode::SpmcIntra => dispatch::send_spmc_intra::<T>,
            BackendMode::MpscIntra => dispatch::send_mpsc_intra::<T>,
            BackendMode::FanoutIntra => dispatch::send_fanout_intra::<T>,
            BackendMode::FanoutShm => dispatch::send_fanout_shm::<T>,
            BackendMode::SpscShm | BackendMode::SpmcShm if colo => {
                dispatch::send_shm_sp_pod_colo::<T>
            }
            BackendMode::SpscShm | BackendMode::SpmcShm if is_pod => dispatch::send_shm_sp_pod::<T>,
            BackendMode::SpscShm | BackendMode::SpmcShm => dispatch::send_shm_sp_serde::<T>,
            BackendMode::PodShm if colo => dispatch::send_shm_pod_broadcast_colo::<T>,
            BackendMode::PodShm => dispatch::send_shm_pod_broadcast::<T>,
            BackendMode::MpscShm if colo => {
                dispatch::send_shm_mp_pod_colo::<T>
            }
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
    fn resolve_recv_fn(
        mode: BackendMode,
        is_pod: bool,
        colo: bool,
        role: TopicRole,
    ) -> dispatch::RecvFn<T> {
        if !role.can_recv() {
            return dispatch::recv_uninitialized::<T>;
        }
        match mode {
            BackendMode::DirectChannel if role == TopicRole::Both => {
                dispatch::recv_direct_channel_local::<T>
            }
            BackendMode::DirectChannel => dispatch::recv_direct_channel_cached::<T>,
            BackendMode::SpscIntra => dispatch::recv_spsc_intra::<T>,
            BackendMode::SpmcIntra => dispatch::recv_spmc_intra::<T>,
            BackendMode::MpscIntra => dispatch::recv_mpsc_intra::<T>,
            BackendMode::FanoutIntra => dispatch::recv_fanout_intra::<T>,
            BackendMode::FanoutShm => dispatch::recv_fanout_shm::<T>,
            BackendMode::SpscShm if colo => dispatch::recv_shm_spsc_pod_colo::<T>,
            BackendMode::SpscShm if is_pod => dispatch::recv_shm_spsc_pod::<T>,
            BackendMode::MpscShm if colo => dispatch::recv_shm_mpsc_pod_colo::<T>,
            BackendMode::MpscShm if is_pod => dispatch::recv_shm_mpsc_pod::<T>,
            BackendMode::SpmcShm if colo => dispatch::recv_shm_spmc_pod_colo::<T>,
            BackendMode::SpmcShm if is_pod => dispatch::recv_shm_spmc_pod::<T>,
            BackendMode::PodShm if colo => dispatch::recv_shm_pod_broadcast_colo::<T>,
            BackendMode::PodShm => dispatch::recv_shm_pod_broadcast::<T>,
            BackendMode::SpscShm => dispatch::recv_shm_spsc_serde::<T>,
            BackendMode::MpscShm => dispatch::recv_shm_mpsc_serde::<T>,
            BackendMode::SpmcShm => dispatch::recv_shm_spmc_serde::<T>,
            // Fallback for Unknown mode: use MPMC SHM (handles any topology).
            // Must NOT return recv_uninitialized here — that causes infinite recursion
            // when the role is already registered but stale SHM left mode as Unknown.
            BackendMode::Unknown if is_pod => dispatch::recv_shm_mpsc_pod::<T>,
            BackendMode::Unknown => dispatch::recv_shm_mpsc_serde::<T>,
        }
    }

    /// Try to set the backend from a registry entry (returns true if successful).
    fn try_set_backend_from_registry(
        &self,
        backend: &mut BackendStorage<T>,
        existing: &Arc<dyn std::any::Any + Send + Sync>,
        mode: BackendMode,
    ) -> bool {
        match mode {
            BackendMode::DirectChannel => {
                if let Ok(slot) = existing.clone().downcast::<DirectSlot<T>>() {
                    *backend = BackendStorage::DirectChannel(slot);
                    return true;
                }
            }
            BackendMode::SpscIntra => {
                if let Ok(ring) = existing.clone().downcast::<SpscRing<T>>() {
                    *backend = BackendStorage::SpscIntra(ring);
                    return true;
                }
            }
            BackendMode::SpmcIntra => {
                if let Ok(ring) = existing.clone().downcast::<SpmcRing<T>>() {
                    *backend = BackendStorage::SpmcIntra(ring);
                    return true;
                }
            }
            BackendMode::MpscIntra => {
                if let Ok(ring) = existing.clone().downcast::<MpscRing<T>>() {
                    *backend = BackendStorage::MpscIntra(ring);
                    return true;
                }
            }
            BackendMode::FanoutIntra => {
                if let Ok(ring) = existing.clone().downcast::<fanout::FanoutRing<T>>() {
                    *backend = BackendStorage::FanoutIntra(ring);
                    return true;
                }
            }
            _ => {}
        }
        false
    }

    /// Drain pending SHM messages and old heap ring messages into a ring-based backend.
    ///
    /// This is called during backend migration. It:
    /// 1. Drains any pending SHM messages (using CAS on header.tail to claim each slot)
    /// 2. Drains the old heap ring from the previous epoch (if any, via registry)
    fn drain_old_into_ring<R>(&self, new_ring: &R, epoch: u64)
    where
        R: RingDrain<T>,
    {
        // 1. Drain pending SHM messages
        self.drain_shm(|msg| {
            let _ = new_ring.push(msg);
        });

        // 2. Drain old heap ring from previous epoch
        if epoch > 0 {
            if let Some(old) = registry::lookup_backend(&self.name, epoch - 1) {
                // Try each ring type
                if let Ok(ring) = old.clone().downcast::<SpscRing<T>>() {
                    while let Some(msg) = ring.try_recv() {
                        let _ = new_ring.push(msg);
                    }
                } else if let Ok(ring) = old.clone().downcast::<SpmcRing<T>>() {
                    while let Some(msg) = ring.try_recv() {
                        let _ = new_ring.push(msg);
                    }
                } else if let Ok(ring) = old.clone().downcast::<MpscRing<T>>() {
                    while let Some(msg) = ring.try_recv() {
                        let _ = new_ring.push(msg);
                    }
                    while let Some(msg) = ring.try_recv() {
                        let _ = new_ring.push(msg);
                    }
                } else if let Ok(slot) = old.clone().downcast::<DirectSlot<T>>() {
                    // DirectSlot uses header-based head/tail (already drained by drain_shm)
                    let _ = slot; // Nothing extra needed
                }
            }
        }
    }

    /// Drain pending SHM messages and old heap ring messages into a DirectSlot backend.
    ///
    /// DirectSlot uses write(seq, value) semantics synchronized through the SHM header.
    /// SHM messages are drained using the same header atomics.
    fn drain_old_into_direct(&self, new_slot: &DirectSlot<T>, epoch: u64) {
        let header = self.header();
        let local = self.local();
        let mask = local.cached_capacity_mask;

        // 1. Drain pending SHM messages into DirectSlot's self-contained ring
        loop {
            let tail = header.tail.load(Ordering::Acquire);
            let head = header.sequence_or_head.load(Ordering::Acquire);
            if tail >= head {
                break;
            }

            if header
                .tail
                .compare_exchange(
                    tail,
                    tail.wrapping_add(1),
                    Ordering::AcqRel,
                    Ordering::Relaxed,
                )
                .is_ok()
            {
                let index = (tail & mask) as usize;
                let msg = self.read_shm_slot(index);
                let _ = new_slot.try_send(msg);
            } else {
                break;
            }
        }

        // 2. Drain old heap ring from previous epoch
        if epoch > 0 {
            if let Some(old) = registry::lookup_backend(&self.name, epoch - 1) {
                if let Ok(ring) = old.clone().downcast::<SpscRing<T>>() {
                    while let Some(msg) = ring.try_recv() {
                        let _ = new_slot.try_send(msg);
                    }
                } else if let Ok(ring) = old.clone().downcast::<SpmcRing<T>>() {
                    while let Some(msg) = ring.try_recv() {
                        let _ = new_slot.try_send(msg);
                    }
                } else if let Ok(ring) = old.clone().downcast::<MpscRing<T>>() {
                    while let Some(msg) = ring.try_recv() {
                        let _ = new_slot.try_send(msg);
                    }
                    while let Some(msg) = ring.try_recv() {
                        let _ = new_slot.try_send(msg);
                    }
                }
            }
        }
    }

    /// Drain pending SHM messages, calling `push_fn` for each message.
    ///
    /// Uses CAS on header.tail to atomically claim each slot, preventing
    /// double-reads if multiple participants drain concurrently.
    fn drain_shm(&self, mut push_fn: impl FnMut(T)) {
        let header = self.header();
        let local = self.local();
        let mask = local.cached_capacity_mask;

        loop {
            let tail = header.tail.load(Ordering::Acquire);
            let head = header.sequence_or_head.load(Ordering::Acquire);
            if tail >= head {
                break;
            }

            if header
                .tail
                .compare_exchange(
                    tail,
                    tail.wrapping_add(1),
                    Ordering::AcqRel,
                    Ordering::Relaxed,
                )
                .is_ok()
            {
                let index = (tail & mask) as usize;
                let msg = self.read_shm_slot(index);
                push_fn(msg);
            } else {
                std::hint::spin_loop();
            }
        }
    }

    /// Read a message from SHM slot at the given index.
    fn read_shm_slot(&self, index: usize) -> T {
        let local = self.local();
        let data_off = Self::data_region_offset(local.cached_capacity as usize);
        if local.is_pod {
            // SAFETY: data_off + index * size_of::<T>() is within storage bounds
            unsafe {
                let base = self.storage.as_ptr().add(data_off) as *const T;
                std::ptr::read(base.add(index))
            }
        } else {
            let slot_size = local.slot_size;
            let slot_offset = index * slot_size;
            let max_data_len = slot_size.saturating_sub(16);
            // SAFETY: slot_ptr is within storage bounds
            unsafe {
                let slot_ptr = self.storage.as_ptr().add(data_off + slot_offset);
                let len_ptr = slot_ptr.add(8) as *const u64;
                let len = std::ptr::read_volatile(len_ptr) as usize;
                if len > max_data_len {
                    panic!(
                        "TopicReader::read_shm_slot: corrupted length {} exceeds slot capacity {} for type {}",
                        len, max_data_len, std::any::type_name::<T>()
                    );
                }
                let data_ptr = slot_ptr.add(16);
                let slice = std::slice::from_raw_parts(data_ptr, len);
                bincode::deserialize(slice).unwrap_or_else(|e| {
                    // Non-Pod types cannot safely fall back to ptr::read — that would
                    // reinterpret raw SHM bytes as T, causing UB for types with heap
                    // allocations (String, Vec) or validity invariants (bool, enums).
                    panic!(
                        "TopicReader::read_shm_slot: bincode deserialization failed for type {}: {}",
                        std::any::type_name::<T>(), e
                    );
                })
            }
        }
    }

    /// Drain old heap ring messages into SHM when switching from intra to cross-process.
    ///
    /// Called during intra→cross-process migration. Reads all pending messages from
    /// the old heap ring (at epoch-1) and writes them into the SHM data region using
    /// the header's sequence_or_head atomic.
    fn drain_old_into_shm(&self, epoch: u64) {
        if epoch == 0 {
            return;
        }
        let old = match registry::lookup_backend(&self.name, epoch - 1) {
            Some(b) => b,
            None => return,
        };

        let header = self.header();
        let local = self.local();
        let mask = local.cached_capacity_mask;

        // Helper: write one message to SHM at the next available slot
        let data_off = Self::data_region_offset(local.cached_capacity as usize);
        let write_to_shm = |msg: T| {
            if local.is_pod {
                let seq = header.sequence_or_head.fetch_add(1, Ordering::AcqRel);
                let index = (seq & mask) as usize;
                // SAFETY: data at data_off + index * sizeof(T) is within storage bounds
                unsafe {
                    let base = self.storage.as_ptr().add(data_off) as *mut T;
                    simd_aware_write(base.add(index), msg);
                }
            } else {
                let slot_size = local.slot_size;
                match bincode::serialize(&msg) {
                    Ok(bytes) => {
                        let seq = header.sequence_or_head.fetch_add(1, Ordering::AcqRel);
                        let index = (seq & mask) as usize;
                        let slot_offset = index * slot_size;
                        // SAFETY: slot_ptr is within storage bounds
                        unsafe {
                            let slot_ptr = self.storage.as_ptr().add(data_off + slot_offset);
                            // Format: [8 bytes padding][8 bytes length][data...]
                            let len_ptr = slot_ptr.add(8) as *mut u64;
                            std::ptr::write_volatile(len_ptr, bytes.len() as u64);
                            let data_ptr = slot_ptr.add(16) as *mut u8;
                            std::ptr::copy_nonoverlapping(bytes.as_ptr(), data_ptr, bytes.len());
                        }
                    }
                    Err(_) => {
                        // Serialization failure — drop the message
                    }
                }
            }
        };

        // Try each ring type from the previous epoch
        if let Ok(ring) = old.clone().downcast::<SpscRing<T>>() {
            while let Some(msg) = ring.try_recv() {
                write_to_shm(msg);
            }
        } else if let Ok(ring) = old.clone().downcast::<SpmcRing<T>>() {
            while let Some(msg) = ring.try_recv() {
                write_to_shm(msg);
            }
        } else if let Ok(ring) = old.clone().downcast::<MpscRing<T>>() {
            while let Some(msg) = ring.try_recv() {
                write_to_shm(msg);
            }
            while let Some(msg) = ring.try_recv() {
                write_to_shm(msg);
            }
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

        let seq_array_size = capacity * std::mem::size_of::<u64>();
        let new_data_size = capacity * new_slot_size;
        let new_total = Self::HEADER_SIZE + seq_array_size + new_data_size;

        // Grow the backing SHM file and remap.
        // SAFETY: single-thread ownership — no concurrent readers/writers on the raw
        // pointer at this point (we're in the send path, post-serialization, pre-write).
        if unsafe { self.storage.grow_unchecked(new_total) }.is_err() {
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
        // SAFETY: single-thread ownership — no concurrent access to these fields.
        // We write through a raw pointer because header_ptr is not behind UnsafeCell.
        let new_header_ptr = self.storage.as_ptr() as *const TopicHeader;
        unsafe {
            let field_ptr = std::ptr::addr_of!(self.header_ptr) as *mut *const TopicHeader;
            std::ptr::write(field_ptr, new_header_ptr);
        }

        // Update the header with the new slot_size.
        // SAFETY: header_ptr now points into the grown ShmRegion.
        unsafe {
            let header_mut = &mut *(new_header_ptr as *mut TopicHeader);
            header_mut.slot_size = new_slot_size as u32;
        }
        let header = unsafe { &*new_header_ptr };

        // Trigger migration so all processes re-sync (picks up new slot_size + pointers).
        let migrator = crate::communication::topic::migration::BackendMigrator::new(header);
        let _ = migrator.migrate_to_optimal();

        // Re-sync local state from the updated header.
        Self::sync_local(local, header, false);

        // Re-derive ALL cached pointers from the (possibly moved) mmap.
        local.cached_header_ptr = new_header_ptr;
        local.cached_seq_ptr =
            unsafe { self.storage.as_ptr().add(Self::HEADER_SIZE) as *mut u8 };
        local.cached_data_ptr =
            unsafe { self.storage.as_ptr().add(Self::data_region_offset(capacity)) as *mut u8 };

        self.initialize_backend();

        log::info!(
            "Topic '{}': auto-grew slot_size {} → {} bytes ({} total SHM)",
            self.name, old_slot_size, new_slot_size, new_total,
        );
        true
    }

    /// Get or lazily create a TensorPool for spilling large serde messages.
    ///
    /// The pool is created on first call using the topic name as the pool_id
    /// seed (FNV-1a hash), so publisher and subscriber processes converge on
    /// the same shared memory file.
    ///
    /// # Safety
    /// Must be called from the owning thread (Topic is !Sync for mutation).
    /// Uses UnsafeCell — same single-thread guarantee as all other dispatch code.
    pub(crate) fn get_or_create_spill_pool(&self) -> Arc<TensorPool> {
        // SAFETY: single-thread ownership — Topic<T> is !Send+!Sync for mutation.
        // UnsafeCell access is safe because dispatch functions run on the owning thread.
        let pool_ref = unsafe { &mut *self.spill_pool.get() };
        if let Some(pool) = pool_ref {
            return Arc::clone(pool);
        }
        let pool = pool_registry::get_or_create_pool(&self.name);
        *pool_ref = Some(Arc::clone(&pool));
        pool
    }

    /// Periodic migration check — reads migration_epoch from SHM header.
    ///
    /// Uses `self.header_ptr` (stable pointer to the SHM TopicHeader) instead
    /// of `local.cached_header_ptr` which is repurposed in DirectChannel mode.
    /// This ensures cross-process migration is detected even when the local
    /// backend is DirectChannel (issue #37).
    #[cold]
    #[inline(never)]
    fn check_migration_periodic(&self) {
        // SAFETY: header_ptr always points to the real SHM TopicHeader, valid
        // for the topic's lifetime (backed by the Arc<ShmRegion> in `storage`).
        let header = unsafe { &*self.header_ptr };
        let shm_epoch = header.migration_epoch.load(Ordering::Relaxed);
        let local = self.local();
        if shm_epoch != local.cached_epoch {
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
        // SAFETY: cached_header_ptr points to the SHM TopicHeader backed by the Arc<ShmRegion>.
        // This method is only called from non-DirectChannel paths (DC mode returns early in
        // check_migration_periodic before reaching here).
        let header = unsafe { &*local.cached_header_ptr };

        // DirectChannel local path: flush LocalState head/tail back to DirectSlot
        // atomics before migration. In the local path, head/tail are plain u64s in
        // LocalState, not atomics in DirectSlot. We need to sync them back so the
        // drain protocol can read the correct positions.
        if local.cached_mode == BackendMode::DirectChannel && local.role == TopicRole::Both {
            // SAFETY: backend UnsafeCell accessed from single thread
            if let BackendStorage::DirectChannel(slot) = unsafe { &*self.backend.get() } {
                slot.head.store(local.local_head, Ordering::Relaxed);
                slot.tail.store(local.local_tail, Ordering::Relaxed);
            }
        }

        // Flush batched updates before migration — but ONLY when the current
        // backend is already SHM-based.  Intra-process backends (DirectChannel,
        // SpscIntra, etc.) track head/tail in a heap ring, not in the SHM header.
        // Flushing a heap-ring position into SHM sequence_or_head would create
        // phantom message slots that were never written to the SHM data region,
        // causing cross-process subscribers to read garbage (issue #37).
        //
        // For SHM→SHM transitions (e.g. SpscShm→MpmcShm), the flush is needed
        // because both backends share the SHM header for head/tail tracking:
        // - SPSC recv batches header.tail every 32 messages; without flushing,
        //   the re-read below gets a stale value.
        // - SP send batches header.sequence_or_head; without flushing, MP
        //   producers doing CAS start from a stale head.
        if local.cached_mode.is_cross_process() {
            if local.role.can_recv() {
                header.tail.store(local.local_tail, Ordering::Release);
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
        Self::sync_local(local, header, true);

        // If slot_size grew (auto-grow from another process), grow our mmap to match.
        let new_slot_size = local.slot_size;
        let capacity = local.cached_capacity as usize;
        if capacity > 0 && new_slot_size > 0 {
            let needed_total =
                Self::HEADER_SIZE + capacity * std::mem::size_of::<u64>() + capacity * new_slot_size;
            if needed_total > self.storage.len() {
                // SAFETY: single-thread ownership, no concurrent reads on the mmap.
                if unsafe { self.storage.grow_unchecked(needed_total) }.is_ok() {
                    // Update header_ptr to point into the (possibly moved) new mmap.
                    let new_header_ptr = self.storage.as_ptr() as *const TopicHeader;
                    unsafe {
                        let field_ptr =
                            std::ptr::addr_of!(self.header_ptr) as *mut *const TopicHeader;
                        std::ptr::write(field_ptr, new_header_ptr);
                    }
                    local.cached_header_ptr = new_header_ptr;
                    local.cached_seq_ptr =
                        unsafe { self.storage.as_ptr().add(Self::HEADER_SIZE) as *mut u8 };
                    local.cached_data_ptr = unsafe {
                        self.storage
                            .as_ptr()
                            .add(Self::data_region_offset(capacity)) as *mut u8
                    };
                }
            }
        }

        self.initialize_backend();

        // Re-sync head/tail from SHM after initialize_backend.
        // drain_old_into_shm() (called by init_shm_backend) may have advanced
        // SHM sequence_or_head by writing drained heap-ring messages.  Without
        // this re-read, local_head is stale (set by sync_local above, before
        // the drain) and the next send would overwrite drained slots.
        // Use header_ptr (always valid) since cached_header_ptr may have been
        // repurposed by DirectChannel init.
        let header_post = unsafe { &*self.header_ptr };
        local.local_head = header_post.sequence_or_head.load(Ordering::Acquire);
        local.local_tail = header_post.tail.load(Ordering::Acquire);
        // Propagate to other same-process Topics
        registry::notify_epoch_change(&self.name, actual_epoch);
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
    /// Fast path for DirectChannel-local (role=Both): the entire ring write is
    /// inlined here — no function pointer indirection, no BackendStorage match,
    /// no epoch_guard. This bypasses 3 levels of call overhead (~8-10ns savings).
    ///
    /// All other backends fall through to the function pointer dispatch, which
    /// includes epoch check, ring operation, and housekeeping.
    #[inline(always)]
    pub fn send(&self, msg: T) {
        // Always-on metric: ~1ns Relaxed atomic increment
        self.header().messages_total.fetch_add(1, Ordering::Relaxed);
        if unlikely(self.is_verbose()) {
            self.send_with_content_logging(msg);
            // Notify event nodes that new data arrived on this topic
            crate::core::NodeInfo::notify_event(&self.name);
            return;
        }
        // Fast path: DirectChannel-local (role=Both, same-thread pub+sub).
        // Bypasses fn ptr indirection + call chain. LocalState fields are in
        // the Topic struct (no pointer chase), and role is on the hot cache line.
        let local = self.local();
        if local.role == TopicRole::Both {
            let head = local.local_head;
            if head.wrapping_sub(local.local_tail) < local.cached_capacity {
                // SAFETY: cached_data_ptr points to the DirectSlot heap buffer; the index is
                // masked to ring capacity, so it is always in bounds. The capacity check above
                // ensures the slot is not occupied (no unconsumed data will be overwritten).
                unsafe {
                    let base = local.cached_data_ptr as *mut T;
                    std::ptr::write(base.add((head & local.cached_capacity_mask) as usize), msg);
                }
                local.local_head = head.wrapping_add(1);
                local.msg_counter = local.msg_counter.wrapping_add(1);
                if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                    self.check_migration_periodic();
                }
                // Notify event nodes that new data arrived on this topic.
                // This uses the topic name to find registered event node notifiers.
                // Cost: ~100ns (mutex + HashMap lookup). Only impacts topics that
                // have a registered event subscriber. No-op if no event node exists.
                crate::core::NodeInfo::notify_event(&self.name);
                return;
            }
            // Buffer full — extremely rare for same-thread, fall through to retry
        }
        self.send_lossy(msg);
        // Notify event nodes after successful send through dispatched path
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
        // First retry immediately — handles the common "buffer was full for
        // a microsecond" case without any spin overhead.
        match self.try_send(msg) {
            Ok(()) => return,
            Err(returned) => msg = returned,
        }

        // If the second attempt also failed, spin briefly. For oversized
        // messages this is wasteful (~50μs), but the warning log in the serde
        // path fires on every attempt so the user sees it.
        const SPIN_ITERS: u32 = 64;
        const YIELD_ITERS: u32 = 4;

        for _ in 0..SPIN_ITERS {
            std::hint::spin_loop();
            match self.try_send(msg) {
                Ok(()) => return,
                Err(returned) => msg = returned,
            }
        }
        for _ in 0..YIELD_ITERS {
            std::thread::yield_now();
            match self.try_send(msg) {
                Ok(()) => return,
                Err(returned) => msg = returned,
            }
        }
        self.metrics.send_failures.fetch_add(1, Ordering::Relaxed);
    }

    /// Send a message, blocking until the ring has space or the timeout expires.
    ///
    /// Unlike [`send()`](Self::send) which drops the message after a brief spin+yield
    /// retry, this method guarantees delivery or returns an explicit timeout error.
    /// Use this for critical command topics (emergency stop, motor setpoints) where
    /// message loss is unacceptable.
    ///
    /// Strategy: spin briefly (256 iters), yield briefly (8 iters), then sleep in
    /// 100μs increments until the deadline.
    ///
    /// # Errors
    ///
    /// - [`SendBlockingError::Timeout`] — ring buffer stayed full for the entire `timeout`
    /// - [`SendBlockingError::Serialization`] — non-POD message failed to serialize
    pub fn send_blocking(
        &self,
        msg: T,
        timeout: std::time::Duration,
    ) -> Result<(), SendBlockingError> {
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

        // Phase 3: yield (microsecond range)
        for _ in 0..8u32 {
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
    /// Fast path for DirectChannel-local (role=Both): inlined ring read,
    /// no function pointer indirection. Same optimization as send().
    #[inline(always)]
    pub fn recv(&self) -> Option<T> {
        if unlikely(self.is_verbose()) {
            return self.recv_with_content_logging();
        }
        // Fast path: DirectChannel-local (role=Both, same-thread pub+sub)
        let local = self.local();
        if local.role == TopicRole::Both {
            let tail = local.local_tail;
            if local.local_head.wrapping_sub(tail) > 0 {
                // SAFETY: cached_data_ptr points to the DirectSlot heap buffer; the index is
                // masked to ring capacity, so it is always in bounds. The head-tail check above
                // ensures the slot contains a valid, initialized message written by send().
                let msg = unsafe {
                    let base = local.cached_data_ptr as *const T;
                    std::ptr::read(base.add((tail & local.cached_capacity_mask) as usize))
                };
                local.local_tail = tail.wrapping_add(1);
                local.msg_counter = local.msg_counter.wrapping_add(1);
                if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                    self.check_migration_periodic();
                }
                return Some(msg);
            }
            // Empty — amortized epoch check
            local.msg_counter = local.msg_counter.wrapping_add(1);
            if unlikely(local.msg_counter & (EPOCH_CHECK_INTERVAL - 1) == 0) {
                self.check_migration_periodic();
            }
            return None;
        }
        self.try_recv()
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
        if self.local().role == TopicRole::Unregistered && self.ensure_consumer().is_err() {
            return None;
        }

        // Check heap backends first — DirectChannel and intra-process rings track
        // head/tail independently from the SHM header, so we must query them directly
        // rather than relying on header.sequence_or_head as a gatekeeper.
        // SAFETY: backend UnsafeCell accessed through &self; only this thread mutates it
        match unsafe { &*self.backend.get() } {
            BackendStorage::DirectChannel(slot) => {
                // In local mode (role==Both), head is tracked in LocalState, not DirectSlot atomics.
                let local = self.local();
                let h = if local.role == TopicRole::Both {
                    local.local_head
                } else {
                    slot.head.load(Ordering::Relaxed)
                };
                let t = if local.role == TopicRole::Both {
                    local.local_tail
                } else {
                    slot.tail.load(Ordering::Relaxed)
                };
                if t >= h {
                    return None;
                }
                let idx = ((h.wrapping_sub(1)) & slot.mask) as usize;
                // SAFETY: idx within bounds; slot is in [tail, head) so data is initialized.
                // T: Copy — bitwise copy, no double-free risk.
                let msg = unsafe {
                    let s = slot.buffer.get_unchecked(idx);
                    (*s.get()).assume_init_read()
                };
                return Some(msg);
            }
            BackendStorage::SpscIntra(ring) => {
                return ring.read_latest();
            }
            BackendStorage::SpmcIntra(ring) => {
                return ring.read_latest();
            }
            BackendStorage::MpscIntra(ring) => {
                return ring.read_latest();
            }
            BackendStorage::FanoutIntra(_) | BackendStorage::FanoutShm(_) => {
                // FanoutRing doesn't support read_latest (no shared head)
                return None;
            }
            BackendStorage::ShmData | BackendStorage::Uninitialized => {
                // Fall through to SHM read below
            }
        }

        // SHM path: read from header sequence counter and data region
        let header = self.header();
        let head = header.sequence_or_head.load(Ordering::Acquire);

        // No messages published yet
        if head == 0 {
            return None;
        }

        let mask = header.capacity_mask as u64;
        let latest_index = ((head.wrapping_sub(1)) & mask) as usize;

        let local = self.local();
        // SAFETY: data_ptr set from valid storage pointer; latest_index within ring bounds
        let msg = unsafe {
            let base = local.cached_data_ptr as *const T;
            simd_aware_read(base.add(latest_index))
        };
        Some(msg)
    }

    /// Check if a message is available without consuming it
    pub fn has_message(&self) -> bool {
        self.pending_count() > 0
    }

    /// Get the number of pending messages
    pub fn pending_count(&self) -> u64 {
        // Check heap-backed ring first; fall back to SHM header
        // SAFETY: backend UnsafeCell accessed through &self; only this thread mutates it
        match unsafe { &*self.backend.get() } {
            BackendStorage::DirectChannel(slot) => {
                // In local mode (role==Both), head/tail are in LocalState
                let local = self.local();
                if local.role == TopicRole::Both {
                    local.local_head.wrapping_sub(local.local_tail)
                } else {
                    slot.pending_count()
                }
            }
            BackendStorage::SpscIntra(ring) => ring.pending_count(),
            BackendStorage::SpmcIntra(ring) => ring.pending_count(),
            BackendStorage::MpscIntra(ring) => ring.pending_count(),
            BackendStorage::FanoutIntra(ring) => ring.pending_count(),
            _ => {
                // ShmData, Uninitialized: use SHM header
                let header = self.header();
                let head = header.sequence_or_head.load(Ordering::Acquire);
                let tail = header.tail.load(Ordering::Acquire);
                head.saturating_sub(tail)
            }
        }
    }

    /// Get the backend name (for debugging)
    #[doc(hidden)]
    pub fn backend_name(&self) -> &'static str {
        match self.mode() {
            BackendMode::Unknown => "Unknown",
            BackendMode::DirectChannel => "DirectChannel",
            BackendMode::SpscIntra => "SpscIntra",
            BackendMode::SpmcIntra => "SpmcIntra",
            BackendMode::MpscIntra => "MpscIntra",
            BackendMode::FanoutIntra => "FanoutIntra",
            BackendMode::PodShm => "PodShm",
            BackendMode::SpscShm => "SpscShm",
            BackendMode::SpmcShm => "SpmcShm",
            BackendMode::MpscShm => "MpscShm",
            BackendMode::FanoutShm => "FanoutShm",
        }
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
    #[doc(hidden)]
    pub fn pub_count(&self) -> u32 {
        self.header().pub_count()
    }

    /// Get subscriber count (for debugging)
    #[doc(hidden)]
    pub fn sub_count(&self) -> u32 {
        self.header().sub_count()
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
            header_ptr: self.header_ptr,
            metrics: Arc::clone(&self.metrics),
            state: AtomicU8::new(self.state.load(Ordering::Relaxed)),
            // Clone shares the spill pool if one was already created
            spill_pool: std::cell::UnsafeCell::new(
                // SAFETY: single-thread access (Topic is !Sync for mutation)
                unsafe { &*self.spill_pool.get() }.clone(),
            ),
            _marker: PhantomData,
        }
    }
}

impl<T> Drop for RingTopic<T> {
    fn drop(&mut self) {
        // Registry entries are NOT removed here — other instances may still
        // reference the same backend Arc. Entries are cleaned up automatically
        // by store_or_get_backend() when new epochs are created (old epochs
        // are retained for at most 1 generation).
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
}

// Safety: Topic delegates to RingTopic which handles synchronization.
// The pool field is Arc (Send+Sync).
unsafe impl<T: TopicMessage> Send for Topic<T> where T::Wire: Send {}
unsafe impl<T: TopicMessage> Sync for Topic<T> where T::Wire: Send + Sync {}

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

    /// Number of messages dropped because the ring buffer was full.
    ///
    /// This is the count of `send()` calls where the message was discarded
    /// after the bounded spin+yield retry failed. Useful for detecting
    /// backpressure or slow consumers.
    ///
    /// # Example
    /// ```rust,ignore
    /// if topic.dropped_count() > 0 {
    ///     eprintln!("WARNING: {} messages dropped on '{}'", topic.dropped_count(), topic.name());
    /// }
    /// ```
    pub fn dropped_count(&self) -> u64 {
        self.ring.metrics().send_failures()
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
        let ring = RingTopic::new(name)?;
        let pool = if T::needs_pool() {
            Some(pool_registry::global_pool())
        } else {
            None
        };
        Ok(Self { ring, pool })
    }

    /// Create a new topic with a specific kind (ServiceRequest, ActionGoal, etc.).
    pub fn new_with_kind(name: impl Into<String>, topic_kind: u8) -> HorusResult<Self> {
        let ring = RingTopic::new_with_kind(name, topic_kind)?;
        let pool = if T::needs_pool() {
            Some(pool_registry::global_pool())
        } else {
            None
        };
        Ok(Self { ring, pool })
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
            Some(pool_registry::global_pool())
        } else {
            None
        };
        Ok(Self { ring, pool })
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
        self.ring.send(msg)
    }

    /// Receive a message.
    #[inline(always)]
    pub fn recv(&self) -> Option<T> {
        self.ring.recv()
    }

    /// Read the most recent message without advancing the consumer position.
    pub fn read_latest(&self) -> Option<T>
    where
        T: Copy,
    {
        self.ring.read_latest()
    }

    /// Try to send a message, returning it on failure (for explicit retry).
    #[inline(always)]
    pub fn try_send(&self, msg: T) -> Result<(), T> {
        self.ring.try_send(msg)
    }

    /// Send a message, blocking until the ring has space or the timeout expires.
    ///
    /// Use this for critical command topics (emergency stop, motor setpoints) where
    /// message loss is unacceptable. For high-frequency sensor data where dropping
    /// stale frames is acceptable, prefer [`send()`](Self::send).
    ///
    /// Returns `Ok(())` if the message was sent, or `Err(SendBlockingError::Timeout)`
    /// if the ring remained full for the entire timeout duration.
    pub fn send_blocking(
        &self,
        msg: T,
        timeout: std::time::Duration,
    ) -> Result<(), SendBlockingError> {
        self.ring.send_blocking(msg, timeout)
    }

    /// Low-level receive without logging/recording hooks.
    ///
    /// Prefer `recv()` — it includes the DirectChannel fast path, logging,
    /// and recording hooks. This exists for internal/test use only.
    #[doc(hidden)]
    #[inline(always)]
    pub fn try_recv(&self) -> Option<T> {
        self.ring.try_recv()
    }
}

// Clone for direct types
impl<T> Clone for Topic<T>
where
    T: TopicMessage<Wire = T> + Clone + Send + Sync + Serialize + DeserializeOwned + 'static,
{
    fn clone(&self) -> Self {
        Self {
            ring: self.ring.clone(),
            pool: self.pool.clone(),
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
    /// Retains the tensor so it stays alive for receivers, then sends the
    /// descriptor through the ring buffer.
    pub fn send(&self, img: impl Borrow<Image>) {
        let wire = img.borrow().to_wire(&self.pool);
        self.ring.send(wire);
    }

    /// Try to send an image without blocking. Returns `Err(img)` if the ring is full.
    pub fn try_send(&self, img: Image) -> Result<(), Image> {
        let wire = img.to_wire(&self.pool);
        self.ring
            .try_send(wire)
            .map_err(|w| Image::from_wire(w, &self.pool))
    }

    /// Receive the next image.
    pub fn recv(&self) -> Option<Image> {
        let wire = self.ring.recv()?;
        Some(Image::from_wire(wire, &self.pool))
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
        let wire = pc.borrow().to_wire(&self.pool);
        self.ring.send(wire);
    }

    /// Try to send a point cloud without blocking. Returns `Err(pc)` if the ring is full.
    pub fn try_send(&self, pc: PointCloud) -> Result<(), PointCloud> {
        let wire = pc.to_wire(&self.pool);
        self.ring
            .try_send(wire)
            .map_err(|w| PointCloud::from_wire(w, &self.pool))
    }

    /// Receive the next point cloud.
    pub fn recv(&self) -> Option<PointCloud> {
        let wire = self.ring.recv()?;
        Some(PointCloud::from_wire(wire, &self.pool))
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
        let wire = depth.borrow().to_wire(&self.pool);
        self.ring.send(wire);
    }

    /// Try to send a depth image without blocking. Returns `Err(depth)` if the ring is full.
    pub fn try_send(&self, depth: DepthImage) -> Result<(), DepthImage> {
        let wire = depth.to_wire(&self.pool);
        self.ring
            .try_send(wire)
            .map_err(|w| DepthImage::from_wire(w, &self.pool))
    }

    /// Receive the next depth image.
    pub fn recv(&self) -> Option<DepthImage> {
        let wire = self.ring.recv()?;
        Some(DepthImage::from_wire(wire, &self.pool))
    }
}

// ============================================================================
// Tensor — Topic<Tensor> with pool-managed tensor handles
// ============================================================================

impl Topic<Tensor> {
    /// Get or create the auto-managed tensor pool for this topic.
    #[doc(hidden)]
    pub fn pool(&self) -> Arc<TensorPool> {
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
        let pool = self.pool();
        crate::memory::TensorHandle::alloc(pool, shape, dtype, device)
    }

    /// Send a tensor handle through this topic (zero-copy).
    #[doc(hidden)]
    pub fn send_handle(&self, handle: &crate::memory::TensorHandle) {
        handle.pool().retain(handle.tensor());
        self.ring.send(*handle.tensor());
    }

    /// Receive a tensor and wrap it in a `TensorHandle`.
    #[doc(hidden)]
    pub fn recv_handle(&self) -> Option<crate::memory::TensorHandle> {
        let tensor = self.ring.recv()?;
        let pool = self.pool();
        // from_owned validates pool_id matches; returns None (discards the
        // tensor) if the descriptor was sent from a different pool — this
        // prevents refcount corruption on a mismatched pool.
        crate::memory::TensorHandle::from_owned(tensor, pool).ok()
    }
}
