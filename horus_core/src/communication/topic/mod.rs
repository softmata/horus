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
//! | MpmcIntra | ~36ns | same_process, pubs>1, subs>1 |
//! | PodShm | ~50ns | cross_process, is_pod |
//! | MpscShm | ~65ns | cross_process, pubs>1, subs=1 |
//! | SpmcShm | ~70ns | cross_process, pubs=1, subs>1 |
//! | SpscShm | ~85ns | cross_process, pubs=1, subs=1, !is_pod |
//! | MpmcShm | ~167ns | cross_process, pubs>1, subs>1 |
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

pub mod header;
pub mod local_state;
pub mod metrics;
pub mod migration;
pub mod types;

// Per-path optimized backend modules
pub(crate) mod backend;
pub(crate) mod direct_channel;
pub(crate) mod dispatch;
pub(crate) mod mpmc_intra;
pub(crate) mod mpsc_intra;
pub(crate) mod registry;
pub(crate) mod shm_data;
pub(crate) mod spmc_intra;
pub(crate) mod spsc_intra;

#[cfg(test)]
mod tests;

use std::marker::PhantomData;
use std::mem;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::Arc;

use serde::{de::DeserializeOwned, Serialize};

use crate::communication::pod::{is_pod, PodMessage};
use crate::error::{HorusError, HorusResult};
use crate::memory::shm_region::ShmRegion;
use crate::memory::simd::{simd_copy_from_shm, simd_copy_to_shm, SIMD_COPY_THRESHOLD};
use crate::utils::unlikely;

pub use header::{TopicHeader, ParticipantEntry};
pub(crate) use header::{TOPIC_MAGIC, TOPIC_VERSION};
use local_state::LocalState;
pub use metrics::{MigrationMetrics, TopicMetrics};

/// Bounded spin iterations for waiting on per-slot ready flags.
///
/// When a multi-producer path does `fetch_add` on head to claim a slot, there's a
/// brief window before the ready flag is written. Consumers spin for up to this many
/// iterations before returning None. On x86, each spin_loop() is a PAUSE (~10-20 cycles),
/// so 256 * 20 = ~5120 cycles ≈ ~1.7µs worst case at 3GHz — well within try_recv bounds.
const READY_FLAG_SPIN_LIMIT: u32 = 256;
pub use migration::{BackendMigrator, MigrationResult, MigrationStats};
pub use types::{
    BackendMode, BackendHint, ConnectionState, TopicConfig, TopicDescriptor, TopicRole,
};

use header::current_time_ms;
use local_state::DEFAULT_SLOT_SIZE;

use backend::BackendStorage;
use shm_data::ShmDataBackend;

use direct_channel::DirectSlot;
use mpmc_intra::MpmcRing;
use mpsc_intra::MpscRing;
use spmc_intra::SpmcRing;
use spsc_intra::SpscRing;

// ============================================================================
// RingDrain trait — uniform push interface for drain protocol
// ============================================================================

/// Trait for pushing messages into a ring buffer during migration drain.
///
/// Implemented by all heap ring backends (SpscRing, SpmcRing, MpscRing, MpmcRing)
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

impl<T> RingDrain<T> for MpmcRing<T> {
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
    calculated.clamp(MIN_CAPACITY, MAX_CAPACITY)
}

// ============================================================================
// Topics Macro
// ============================================================================

/// Define type-safe topic descriptors for compile-time checked topic names.
#[macro_export]
macro_rules! topics {
    ($($vis:vis $name:ident : $type:ty = $topic_name:expr),* $(,)?) => {
        $(
            $vis const $name: $crate::communication::topic::TopicDescriptor<$type> =
                $crate::communication::topic::TopicDescriptor::new($topic_name);
        )*
    };
}

// ============================================================================
// Topic - Main Public API
// ============================================================================

/// Topic - Universal Smart Detection IPC
///
/// `Topic<T>` provides fully automatic backend detection. Users just call
/// `send()`/`recv()` and the system auto-detects the optimal backend from 10 paths
/// based on topology and access patterns.
pub struct Topic<T> {
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

    /// Optional logging function (set via `with_logging()`)
    log_fn: Option<fn(&T) -> String>,

    /// Connection state (for network backend compatibility)
    state: AtomicU8,

    /// Type marker
    _marker: PhantomData<T>,
}

// Safety: Topic can be sent between threads
// The UnsafeCell is only accessed through &self with internal synchronization
unsafe impl<T: Send> Send for Topic<T> {}
unsafe impl<T: Send + Sync> Sync for Topic<T> {}

impl<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static> Topic<T> {
    /// Header size in shared memory
    const HEADER_SIZE: usize = mem::size_of::<TopicHeader>();

    /// Create a new topic with auto-sized ring buffer capacity.
    pub fn new(name: impl Into<String>) -> HorusResult<Self> {
        let name = name.into();
        Self::with_capacity(&name, auto_capacity::<T>(), None)
    }

    /// Create a new topic with custom capacity and optional slot size
    pub fn with_capacity(name: &str, capacity: u32, slot_size: Option<usize>) -> HorusResult<Self> {
        if capacity == 0 {
            return Err(crate::HorusError::InvalidInput(
                "Topic capacity must be >= 1".to_string(),
            ));
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

        // SAFETY: storage is properly sized (>= HEADER_SIZE) and aligned for TopicHeader
        let header = unsafe { &mut *(storage.as_ptr() as *mut TopicHeader) };

        std::sync::atomic::fence(Ordering::Acquire);

        let final_slot_size = if header.magic != TOPIC_MAGIC {
            if storage.is_owner() {
                // Fresh SHM — clear any stale registry entries from a previous
                // topic lifetime (e.g., previous owner dropped and SHM was unlinked,
                // then re-created with the same name).
                registry::remove_topic(name);

                header.init(
                    type_size,
                    type_align,
                    is_pod,
                    capacity,
                    actual_slot_size as u32,
                );
                actual_slot_size
            } else {
                for _ in 0..100 {
                    std::thread::sleep(std::time::Duration::from_millis(1));
                    std::sync::atomic::fence(Ordering::Acquire);
                    if header.magic == TOPIC_MAGIC {
                        break;
                    }
                }
                if header.magic != TOPIC_MAGIC {
                    return Err(HorusError::Communication(
                        "Timeout waiting for topic header initialization".to_string(),
                    ));
                }
                header.slot_size as usize
            }
        } else {
            if header.version != TOPIC_VERSION {
                return Err(HorusError::Communication(format!(
                    "Incompatible topic version: {} (expected {})",
                    header.version, TOPIC_VERSION
                )));
            }
            if is_pod && header.type_size != type_size {
                return Err(HorusError::Communication(format!(
                    "Type size mismatch: {} (expected {})",
                    header.type_size, type_size
                )));
            }
            header.slot_size as usize
        };

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
            metrics: Arc::new(MigrationMetrics::default()),
            log_fn: None,
            state: AtomicU8::new(ConnectionState::Connected.into_u8()),
            _marker: PhantomData,
        })
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

    /// Compute the byte offset from storage start to the data region.
    ///
    /// Layout: [HEADER (640)] [SEQ_ARRAY (capacity * 8)] [DATA (capacity * slot_size)]
    #[inline]
    fn data_region_offset(capacity: usize) -> usize {
        Self::HEADER_SIZE + capacity * mem::size_of::<u64>()
    }

    /// Get the local state (interior mutability via UnsafeCell)
    #[inline(always)]
    #[allow(clippy::mut_from_ref)]
    fn local(&self) -> &mut LocalState {
        // SAFETY: LocalState access is thread-local; no concurrent mutation
        unsafe { &mut *self.local.get() }
    }

    /// Register as producer if not already registered
    fn ensure_producer(&self) -> HorusResult<()> {
        let local = self.local();
        if local.role.can_send() {
            return Ok(());
        }

        let header = self.header();
        let slot = header.register_producer()?;

        local.slot_index = slot as i32;
        local.cached_header_ptr = self.storage.as_ptr() as *const TopicHeader;
        let cap = header.capacity as usize;
        // SAFETY: HEADER_SIZE offset is within bounds of allocated storage region
        local.cached_seq_ptr = unsafe { self.storage.as_ptr().add(Self::HEADER_SIZE) as *mut u8 };
        local.cached_data_ptr = unsafe { self.storage.as_ptr().add(Self::data_region_offset(cap)) as *mut u8 };
        local.cached_epoch = header.migration_epoch.load(Ordering::Acquire);
        local.is_same_process = header.is_same_process();
        local.is_pod = header.is_pod_type();
        local.slot_size = header.slot_size as usize;
        local.cached_mode = header.mode();
        local.cached_capacity = header.capacity as u64;
        local.cached_capacity_mask = header.capacity_mask as u64;
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        local.local_tail = header.tail.load(Ordering::Acquire);
        // Set role LAST - this gates the fast path via can_send()
        local.role = if local.role == TopicRole::Consumer {
            TopicRole::Both
        } else {
            TopicRole::Producer
        };

        self.metrics.estimated_latency_ns.store(
            local.cached_mode.expected_latency_ns() as u32,
            Ordering::Relaxed,
        );

        self.check_migration();
        self.initialize_backend();

        Ok(())
    }

    /// Register as consumer if not already registered
    fn ensure_consumer(&self) -> HorusResult<()> {
        let local = self.local();
        if local.role.can_recv() {
            return Ok(());
        }

        let header = self.header();
        let slot = header.register_consumer()?;

        local.slot_index = slot as i32;
        local.cached_header_ptr = self.storage.as_ptr() as *const TopicHeader;
        let cap = header.capacity as usize;
        // SAFETY: HEADER_SIZE offset is within bounds of allocated storage region
        local.cached_seq_ptr = unsafe { self.storage.as_ptr().add(Self::HEADER_SIZE) as *mut u8 };
        local.cached_data_ptr = unsafe { self.storage.as_ptr().add(Self::data_region_offset(cap)) as *mut u8 };
        local.cached_epoch = header.migration_epoch.load(Ordering::Acquire);
        local.is_same_process = header.is_same_process();
        local.is_pod = header.is_pod_type();
        local.slot_size = header.slot_size as usize;
        local.cached_mode = header.mode();
        local.cached_capacity = header.capacity as u64;
        local.cached_capacity_mask = header.capacity_mask as u64;
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        local.local_tail = header.tail.load(Ordering::Acquire);
        // Set role LAST - this gates the fast path via can_recv()
        local.role = if local.role == TopicRole::Producer {
            TopicRole::Both
        } else {
            TopicRole::Consumer
        };

        self.metrics.estimated_latency_ns.store(
            local.cached_mode.expected_latency_ns() as u32,
            Ordering::Relaxed,
        );

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
            local.cached_epoch = current_epoch;
            local.is_same_process = header.is_same_process();
            local.is_pod = header.is_pod_type();
            local.slot_size = header.slot_size as usize;
            local.cached_mode = header.mode();
            local.cached_capacity = header.capacity as u64;
            local.cached_capacity_mask = header.capacity_mask as u64;
            local.local_head = header.sequence_or_head.load(Ordering::Acquire);
            local.local_tail = header.tail.load(Ordering::Acquire);
            // PodShm broadcast: skip stale data from previous era.
            // Ready-flags from prior backends use different protocols, so
            // consumers must start fresh from current head after migration.
            if local.cached_mode == BackendMode::PodShm {
                local.local_tail = local.local_head;
            }
            self.metrics.estimated_latency_ns.store(
                local.cached_mode.expected_latency_ns() as u32,
                Ordering::Relaxed,
            );
            // Re-initialize backend + dispatch fn ptrs to match the new mode.
            // Another participant already performed the migration; we just need
            // to update our local storage and fn ptrs to match.
            self.initialize_backend();
            registry::notify_epoch_change(&self.name, current_epoch);
        }

        let migrator = BackendMigrator::new(header);
        if !migrator.is_optimal() {
            for _attempt in 0..5 {
                match migrator.migrate_to_optimal() {
                    MigrationResult::Success { new_epoch } => {
                        local.cached_epoch = new_epoch;
                        local.is_same_process = header.is_same_process();
                        local.is_pod = header.is_pod_type();
                        local.slot_size = header.slot_size as usize;
                        local.cached_mode = header.mode();
                        local.cached_capacity = header.capacity as u64;
                        local.cached_capacity_mask = header.capacity_mask as u64;
                        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
                        local.local_tail = header.tail.load(Ordering::Acquire);
                        // PodShm broadcast: skip stale data from previous era
                        if local.cached_mode == BackendMode::PodShm {
                            local.local_tail = local.local_head;
                        }
                        self.metrics.migrations.fetch_add(1, Ordering::Relaxed);
                        self.metrics.estimated_latency_ns.store(
                            local.cached_mode.expected_latency_ns() as u32,
                            Ordering::Relaxed,
                        );
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
                            if wait_start.elapsed() > std::time::Duration::from_millis(5) {
                                break;
                            }
                        }
                        // The other migration completed — refresh local state
                        let new_epoch = header.migration_epoch.load(Ordering::Acquire);
                        if new_epoch != local.cached_epoch {
                            local.cached_epoch = new_epoch;
                            local.is_same_process = header.is_same_process();
                            local.is_pod = header.is_pod_type();
                            local.slot_size = header.slot_size as usize;
                            local.cached_mode = header.mode();
                            local.cached_capacity = header.capacity as u64;
                            local.cached_capacity_mask = header.capacity_mask as u64;
                            local.local_head = header.sequence_or_head.load(Ordering::Acquire);
                            local.local_tail = header.tail.load(Ordering::Acquire);
                            // PodShm broadcast: skip stale data from previous era
                            if local.cached_mode == BackendMode::PodShm {
                                local.local_tail = local.local_head;
                            }
                            self.metrics.estimated_latency_ns.store(
                                local.cached_mode.expected_latency_ns() as u32,
                                Ordering::Relaxed,
                            );
                            registry::notify_epoch_change(&self.name, new_epoch);
                        }
                        if migrator.is_optimal() {
                            break;
                        }
                        // Still not optimal after other migration — retry our migration
                    }
                    MigrationResult::NotNeeded => {
                        local.cached_mode = header.mode();
                        self.metrics.estimated_latency_ns.store(
                            local.cached_mode.expected_latency_ns() as u32,
                            Ordering::Relaxed,
                        );
                        break;
                    }
                    MigrationResult::Failed => {
                        local.cached_mode = header.mode();
                        self.metrics.estimated_latency_ns.store(
                            local.cached_mode.expected_latency_ns() as u32,
                            Ordering::Relaxed,
                        );
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
        let capacity = local.cached_capacity as u32;

        // SAFETY: backend UnsafeCell accessed through &self; only this thread mutates it
        let backend = unsafe { &mut *self.backend.get() };

        // Check if current backend already matches the mode
        let already_matched = match (&*backend, mode) {
            (BackendStorage::DirectChannel(_), BackendMode::DirectChannel) => true,
            (BackendStorage::SpscIntra(_), BackendMode::SpscIntra) => true,
            (BackendStorage::SpmcIntra(_), BackendMode::SpmcIntra) => true,
            (BackendStorage::MpscIntra(_), BackendMode::MpscIntra) => true,
            (BackendStorage::MpmcIntra(_), BackendMode::MpmcIntra) => true,
            (BackendStorage::ShmData(_), _) if mode.is_cross_process() || mode == BackendMode::Unknown => true,
            _ => false,
        };
        if already_matched {
            // Backend matches but fn ptrs may be stale (e.g., first call after construction).
            // SAFETY: UnsafeCell accessed from single thread.
            self.set_dispatch_fn_ptrs(mode, local.is_pod);
            return;
        }

        if mode.is_intra_process() {
            let cap = if capacity == 0 { 64 } else { capacity };

            // First, try to look up an existing ring from the registry.
            // Another participant may have already created one for this (topic, epoch).
            if let Some(existing) = registry::lookup_backend(&self.name, epoch) {
                if self.try_set_backend_from_registry(backend, &existing, mode) {
                    self.set_dispatch_fn_ptrs(mode, local.is_pod);
                    return;
                }
            }

            // Not found in registry — create new ring, drain old data, and store.
            match mode {
                BackendMode::DirectChannel => {
                    let new_ring = Arc::new(DirectSlot::new(cap));
                    self.drain_old_into_direct(&new_ring, epoch);
                    let shared = registry::store_or_get_backend(
                        &self.name, epoch,
                        new_ring.clone() as Arc<dyn std::any::Any + Send + Sync>,
                    );
                    if let Ok(slot) = shared.downcast::<DirectSlot<T>>() {
                        *backend = BackendStorage::DirectChannel(slot);
                    } else {
                        *backend = BackendStorage::DirectChannel(new_ring);
                    }
                }
                BackendMode::SpscIntra => {
                    let new_ring = Arc::new(SpscRing::new(cap));
                    self.drain_old_into_ring(&new_ring, epoch);
                    let shared = registry::store_or_get_backend(
                        &self.name, epoch,
                        new_ring.clone() as Arc<dyn std::any::Any + Send + Sync>,
                    );
                    if let Ok(ring) = shared.downcast::<SpscRing<T>>() {
                        *backend = BackendStorage::SpscIntra(ring);
                    } else {
                        *backend = BackendStorage::SpscIntra(new_ring);
                    }
                }
                BackendMode::SpmcIntra => {
                    let new_ring = Arc::new(SpmcRing::new(cap));
                    self.drain_old_into_ring(&new_ring, epoch);
                    let shared = registry::store_or_get_backend(
                        &self.name, epoch,
                        new_ring.clone() as Arc<dyn std::any::Any + Send + Sync>,
                    );
                    if let Ok(ring) = shared.downcast::<SpmcRing<T>>() {
                        *backend = BackendStorage::SpmcIntra(ring);
                    } else {
                        *backend = BackendStorage::SpmcIntra(new_ring);
                    }
                }
                BackendMode::MpscIntra => {
                    let new_ring = Arc::new(MpscRing::new(cap));
                    self.drain_old_into_ring(&new_ring, epoch);
                    let shared = registry::store_or_get_backend(
                        &self.name, epoch,
                        new_ring.clone() as Arc<dyn std::any::Any + Send + Sync>,
                    );
                    if let Ok(ring) = shared.downcast::<MpscRing<T>>() {
                        *backend = BackendStorage::MpscIntra(ring);
                    } else {
                        *backend = BackendStorage::MpscIntra(new_ring);
                    }
                }
                BackendMode::MpmcIntra => {
                    let new_ring = Arc::new(MpmcRing::new(cap));
                    self.drain_old_into_ring(&new_ring, epoch);
                    let shared = registry::store_or_get_backend(
                        &self.name, epoch,
                        new_ring.clone() as Arc<dyn std::any::Any + Send + Sync>,
                    );
                    if let Ok(ring) = shared.downcast::<MpmcRing<T>>() {
                        *backend = BackendStorage::MpmcIntra(ring);
                    } else {
                        *backend = BackendStorage::MpmcIntra(new_ring);
                    }
                }
                _ => {
                    // Unrecognized intra-process mode — keep ShmData
                    *backend = BackendStorage::ShmData(ShmDataBackend {
                        data_region: self.storage.clone(),
                        data_offset: Self::data_region_offset(local.cached_capacity as usize),
                        mode,
                        is_pod: local.is_pod,
                        slot_size: local.slot_size,
                        capacity: local.cached_capacity,
                        mask: local.cached_capacity_mask,
                    });
                }
            }
        } else {
            // Cross-process or Unknown: use ShmData.
            // Drain old heap ring messages into SHM before switching.
            self.drain_old_into_shm(epoch);

            *backend = BackendStorage::ShmData(ShmDataBackend {
                data_region: self.storage.clone(),
                data_offset: Self::data_region_offset(local.cached_capacity as usize),
                mode,
                is_pod: local.is_pod,
                slot_size: local.slot_size,
                capacity: local.cached_capacity,
                mask: local.cached_capacity_mask,
            });
        }

        // Set function pointers to match the new backend.
        // SAFETY: UnsafeCell accessed from single thread.
        self.set_dispatch_fn_ptrs(mode, local.is_pod);
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
        let colo = is_pod && mem::size_of::<T>() + 8 <= 64;
        let local = self.local();
        let can_send = local.role.can_send();
        let can_recv = local.role.can_recv();

        // SAFETY: UnsafeCell accessed from single thread (same guarantee as backend/local)
        unsafe {
            // Only set send_fn if registered as producer.
            // Keeping send_uninitialized ensures ensure_producer() is called on first send,
            // which registers the participant and triggers correct topology detection.
            *self.send_fn.get() = if !can_send {
                dispatch::send_uninitialized::<T>
            } else {
                match mode {
                    BackendMode::DirectChannel => dispatch::send_direct_channel::<T>,
                    BackendMode::SpscIntra => dispatch::send_spsc_intra::<T>,
                    BackendMode::SpmcIntra => dispatch::send_spmc_intra::<T>,
                    BackendMode::MpscIntra => dispatch::send_mpsc_intra::<T>,
                    BackendMode::MpmcIntra => dispatch::send_mpmc_intra::<T>,
                    BackendMode::SpscShm | BackendMode::SpmcShm if colo => dispatch::send_shm_sp_pod_colo::<T>,
                    BackendMode::SpscShm | BackendMode::SpmcShm if is_pod => dispatch::send_shm_sp_pod::<T>,
                    BackendMode::SpscShm | BackendMode::SpmcShm => dispatch::send_shm_sp_serde::<T>,
                    BackendMode::PodShm if colo => dispatch::send_shm_pod_broadcast_colo::<T>,
                    BackendMode::PodShm => dispatch::send_shm_pod_broadcast::<T>,
                    BackendMode::MpscShm | BackendMode::MpmcShm if colo => dispatch::send_shm_mp_pod_colo::<T>,
                    BackendMode::MpscShm | BackendMode::MpmcShm if is_pod => dispatch::send_shm_mp_pod::<T>,
                    BackendMode::MpscShm | BackendMode::MpmcShm => dispatch::send_shm_mp_serde::<T>,
                    BackendMode::Unknown => dispatch::send_uninitialized::<T>,
                }
            };

            // Only set recv_fn if registered as consumer.
            // Keeping recv_uninitialized ensures ensure_consumer() is called on first recv,
            // which registers the participant and triggers correct topology detection
            // (e.g., enabling DirectChannel when both pub+sub are on the same thread).
            *self.recv_fn.get() = if !can_recv {
                dispatch::recv_uninitialized::<T>
            } else {
                match mode {
                    BackendMode::DirectChannel => dispatch::recv_direct_channel::<T>,
                    BackendMode::SpscIntra => dispatch::recv_spsc_intra::<T>,
                    BackendMode::SpmcIntra => dispatch::recv_spmc_intra::<T>,
                    BackendMode::MpscIntra => dispatch::recv_mpsc_intra::<T>,
                    BackendMode::MpmcIntra => dispatch::recv_mpmc_intra::<T>,
                    // SpscShm: MUST poll header.sequence_or_head (separate cache line).
                    // Polling inline seq would contend with producer's data write.
                    BackendMode::SpscShm if colo => dispatch::recv_shm_spsc_pod_colo::<T>,
                    BackendMode::SpscShm if is_pod => dispatch::recv_shm_spsc_pod::<T>,
                    // Non-SPSC: inline seq + data on same cache line → 1 fewer transfer
                    BackendMode::MpscShm if colo => dispatch::recv_shm_mpsc_pod_colo::<T>,
                    BackendMode::MpscShm if is_pod => dispatch::recv_shm_mpsc_pod::<T>,
                    BackendMode::SpmcShm if colo => dispatch::recv_shm_spmc_pod_colo::<T>,
                    BackendMode::SpmcShm if is_pod => dispatch::recv_shm_spmc_pod::<T>,
                    BackendMode::PodShm if colo => dispatch::recv_shm_pod_broadcast_colo::<T>,
                    BackendMode::PodShm => dispatch::recv_shm_pod_broadcast::<T>,
                    BackendMode::MpmcShm if colo => dispatch::recv_shm_mpmc_pod_colo::<T>,
                    BackendMode::MpmcShm if is_pod => dispatch::recv_shm_mpmc_pod::<T>,
                    BackendMode::SpscShm => dispatch::recv_shm_spsc_serde::<T>,
                    BackendMode::MpscShm => dispatch::recv_shm_mpsc_serde::<T>,
                    BackendMode::SpmcShm => dispatch::recv_shm_spmc_serde::<T>,
                    BackendMode::MpmcShm => dispatch::recv_shm_mpmc_serde::<T>,
                    BackendMode::Unknown => dispatch::recv_uninitialized::<T>,
                }
            };
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
            BackendMode::MpmcIntra => {
                if let Ok(ring) = existing.clone().downcast::<MpmcRing<T>>() {
                    *backend = BackendStorage::MpmcIntra(ring);
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
        self.drain_shm(|msg| { let _ = new_ring.push(msg); });

        // 2. Drain old heap ring from previous epoch
        if epoch > 0 {
            if let Some(old) = registry::lookup_backend(&self.name, epoch - 1) {
                // Try each ring type
                if let Ok(ring) = old.clone().downcast::<SpscRing<T>>() {
                    while let Some(msg) = ring.try_recv() { let _ = new_ring.push(msg); }
                } else if let Ok(ring) = old.clone().downcast::<SpmcRing<T>>() {
                    while let Some(msg) = ring.try_recv() { let _ = new_ring.push(msg); }
                } else if let Ok(ring) = old.clone().downcast::<MpscRing<T>>() {
                    while let Some(msg) = ring.try_recv() { let _ = new_ring.push(msg); }
                } else if let Ok(ring) = old.clone().downcast::<MpmcRing<T>>() {
                    while let Some(msg) = ring.try_recv() { let _ = new_ring.push(msg); }
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
            if tail >= head { break; }

            if header.tail.compare_exchange(
                tail, tail.wrapping_add(1), Ordering::AcqRel, Ordering::Relaxed
            ).is_ok() {
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
                } else if let Ok(ring) = old.clone().downcast::<MpmcRing<T>>() {
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
            if tail >= head { break; }

            if header.tail.compare_exchange(
                tail, tail.wrapping_add(1), Ordering::AcqRel, Ordering::Relaxed,
            ).is_ok() {
                let index = (tail & mask) as usize;
                let msg = self.read_shm_slot(index);
                push_fn(msg);
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
            // SAFETY: slot_ptr is within storage bounds
            unsafe {
                let slot_ptr = self.storage.as_ptr().add(data_off + slot_offset);
                let len_ptr = slot_ptr.add(8) as *const u64;
                let len = std::ptr::read_volatile(len_ptr) as usize;
                let data_ptr = slot_ptr.add(16);
                let slice = std::slice::from_raw_parts(data_ptr, len);
                bincode::deserialize(slice).unwrap_or_else(|_| {
                    // This should not happen — the producer serialized it successfully.
                    // Fallback: create a default-ish value by re-reading as pod.
                    std::ptr::read(slot_ptr as *const T)
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
                            let slot_ptr =
                                self.storage.as_ptr().add(data_off + slot_offset);
                            // Format: [8 bytes padding][8 bytes length][data...]
                            let len_ptr = slot_ptr.add(8) as *mut u64;
                            std::ptr::write_volatile(len_ptr, bytes.len() as u64);
                            let data_ptr = slot_ptr.add(16) as *mut u8;
                            std::ptr::copy_nonoverlapping(
                                bytes.as_ptr(),
                                data_ptr,
                                bytes.len(),
                            );
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
        } else if let Ok(ring) = old.clone().downcast::<MpmcRing<T>>() {
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
        unsafe { (*self.send_fn.get())(self, msg) }
    }

    /// Try to receive a message without logging.
    ///
    /// Single indirect call — ALL logic (epoch check, ring op, housekeeping)
    /// lives inside the dispatch function. First call goes through
    /// `recv_uninitialized` which handles registration + re-dispatch.
    #[inline(always)]
    pub fn try_recv(&self) -> Option<T> {
        unsafe { (*self.recv_fn.get())(self) }
    }

    /// Periodic maintenance — called every LEASE_REFRESH_INTERVAL messages from dispatch functions.
    /// Handles lease refresh and cross-process epoch detection (SHM header check).
    #[cold]
    #[inline(never)]
    fn periodic_maintenance(&self) {
        self.refresh_lease();
        self.check_migration_periodic();
    }

    /// Periodic migration check — reads migration_epoch from SHM header.
    #[cold]
    #[inline(never)]
    fn check_migration_periodic(&self) {
        let local = self.local();
        let header = unsafe { &*local.cached_header_ptr };
        let shm_epoch = header.migration_epoch.load(Ordering::Relaxed);
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
        let header = unsafe { &*local.cached_header_ptr };
        // Re-read actual epoch from SHM (_hint_epoch may be from process_epoch)
        let actual_epoch = header.migration_epoch.load(Ordering::Acquire);
        local.cached_epoch = actual_epoch;
        local.cached_mode = header.mode();
        local.is_same_process = header.is_same_process();
        local.is_pod = header.is_pod_type();
        local.slot_size = header.slot_size as usize;
        local.cached_capacity = header.capacity as u64;
        local.cached_capacity_mask = header.capacity_mask as u64;
        local.local_head = header.sequence_or_head.load(Ordering::Acquire);
        local.local_tail = header.tail.load(Ordering::Acquire);
        // PodShm broadcast: skip stale data from previous era
        if local.cached_mode == BackendMode::PodShm {
            local.local_tail = local.local_head;
        }
        self.initialize_backend();
        // Propagate to other same-process Topics
        registry::notify_epoch_change(&self.name, actual_epoch);
    }

    /// Get the topic name
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the current backend mode
    pub fn mode(&self) -> BackendMode {
        self.header().mode()
    }

    /// Get the current role
    pub fn role(&self) -> TopicRole {
        self.local().role
    }

    /// Get raw migration metrics (for internal use)
    pub fn migration_metrics(&self) -> &MigrationMetrics {
        &self.metrics
    }

    /// Get a snapshot of the topic's metrics (compatible with Topic API)
    pub fn metrics(&self) -> TopicMetrics {
        TopicMetrics {
            messages_sent: self.metrics.messages_sent.load(Ordering::Relaxed),
            messages_received: self.metrics.messages_received.load(Ordering::Relaxed),
            send_failures: self.metrics.send_failures.load(Ordering::Relaxed),
            recv_failures: self.metrics.recv_failures.load(Ordering::Relaxed),
        }
    }

    /// Get the current connection state
    pub fn connection_state(&self) -> ConnectionState {
        ConnectionState::from_u8(self.state.load(Ordering::Relaxed))
    }

    /// Create a topic from a type-safe descriptor (publishing side).
    pub fn publish(descriptor: TopicDescriptor<T>) -> HorusResult<Self> {
        Self::new(descriptor.name())
    }

    /// Create a topic from a type-safe descriptor (subscribing side).
    pub fn subscribe(descriptor: TopicDescriptor<T>) -> HorusResult<Self> {
        Self::new(descriptor.name())
    }

    /// Send a message (fire-and-forget with bounded retry).
    ///
    /// Hot path: log_fn check (predicted-not-taken) → try_send() → return.
    /// The logging path and retry loops are outlined into cold functions.
    #[inline(always)]
    pub fn send(&self, msg: T) {
        if unlikely(self.log_fn.is_some()) {
            self.send_with_logging(msg);
            return;
        }
        self.send_lossy(msg);
    }

    /// Logging path for send() — outlined to keep send() hot path tight.
    #[cold]
    #[inline(never)]
    fn send_with_logging(&self, msg: T) {
        let log_fn = self.log_fn.unwrap();
        let summary = log_fn(&msg);
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
            Ok(()) => return,
            Err(returned) => self.send_lossy_retry(returned),
        }
    }

    /// Retry loop for send_lossy — outlined to keep the fast path tight.
    #[cold]
    #[inline(never)]
    fn send_lossy_retry(&self, mut msg: T) {
        const SPIN_ITERS: u32 = 256;
        const YIELD_ITERS: u32 = 8;

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

    /// Receive a message with optional logging.
    ///
    /// Hot path: log_fn check (predicted-not-taken) → try_recv() → return.
    #[inline(always)]
    pub fn recv(&self) -> Option<T> {
        if unlikely(self.log_fn.is_some()) {
            return self.recv_with_logging();
        }
        self.try_recv()
    }

    /// Logging path for recv() — outlined to keep recv() hot path tight.
    #[cold]
    #[inline(never)]
    fn recv_with_logging(&self) -> Option<T> {
        let log_fn = self.log_fn.unwrap();
        let start = std::time::Instant::now();
        let result = self.try_recv();
        let ipc_ns = start.elapsed().as_nanos() as u64;

        if let Some(ref msg) = result {
            self.metrics
                .messages_received
                .fetch_add(1, Ordering::Relaxed);
            use crate::core::hlog::{current_node_name, current_tick_number};
            use crate::core::log_buffer::{publish_log, LogEntry, LogType};
            let now = chrono::Local::now();
            let summary = log_fn(msg);
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

    /// Get migration statistics
    pub fn migration_stats(&self) -> MigrationStats {
        let migrator = BackendMigrator::new(self.header());
        migrator.stats()
    }

    /// Force a migration check NOW — reads SHM header epoch, detects optimal
    /// backend, and re-initializes dispatch if the topology changed.
    ///
    /// Useful when you know a cross-process participant has joined/left and
    /// want immediate migration without waiting for the periodic check.
    pub fn check_migration_now(&self) {
        self.check_migration();
    }

    /// Force a backend migration (for testing)
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
    pub fn read_latest(&self) -> Option<T> {
        // Ensure we're registered as a consumer so the header is initialized
        if self.local().role == TopicRole::Unregistered {
            if self.ensure_consumer().is_err() {
                return None;
            }
        }

        let header = self.header();
        let head = header.sequence_or_head.load(Ordering::Acquire);

        // No messages published yet
        if head == 0 {
            return None;
        }

        let mask = header.capacity_mask as u64;
        let latest_index = ((head.wrapping_sub(1)) & mask) as usize;

        // Try heap backends first
        // SAFETY: backend UnsafeCell accessed through &self; only this thread mutates it
        match unsafe { &*self.backend.get() } {
            BackendStorage::DirectChannel(slot) => {
                let h = slot.head.load(Ordering::Relaxed);
                if h == 0 {
                    return None;
                }
                let idx = ((h.wrapping_sub(1)) & slot.mask) as usize;
                // SAFETY: idx within bounds; data was written by producer
                let msg = unsafe {
                    let s = &*slot.buffer.get_unchecked(idx);
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
            BackendStorage::MpmcIntra(ring) => {
                return ring.read_latest();
            }
            BackendStorage::ShmData(_) | BackendStorage::Uninitialized => {
                // Fall through to SHM read below
            }
        }

        // SHM path: read directly from the data region
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
            BackendStorage::DirectChannel(slot) => slot.pending_count(),
            BackendStorage::SpscIntra(ring) => ring.pending_count(),
            BackendStorage::SpmcIntra(ring) => ring.pending_count(),
            BackendStorage::MpscIntra(ring) => ring.pending_count(),
            BackendStorage::MpmcIntra(ring) => ring.pending_count(),
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
    pub fn backend_name(&self) -> &'static str {
        match self.mode() {
            BackendMode::Unknown => "Unknown",
            BackendMode::DirectChannel => "DirectChannel",
            BackendMode::SpscIntra => "SpscIntra",
            BackendMode::SpmcIntra => "SpmcIntra",
            BackendMode::MpscIntra => "MpscIntra",
            BackendMode::MpmcIntra => "MpmcIntra",
            BackendMode::PodShm => "PodShm",
            BackendMode::SpscShm => "SpscShm",
            BackendMode::SpmcShm => "SpmcShm",
            BackendMode::MpscShm => "MpscShm",
            BackendMode::MpmcShm => "MpmcShm",
        }
    }

    /// Get the backend type name (alias for backend_name, API compatibility)
    pub fn backend_type(&self) -> &'static str {
        self.backend_name()
    }

    /// Check if all participants are in the same process (for debugging)
    pub fn is_same_process(&self) -> bool {
        self.header().is_same_process()
    }

    /// Check if caller is on same thread as creator (for debugging)
    pub fn is_same_thread(&self) -> bool {
        self.header().is_same_thread()
    }

    /// Get publisher count (for debugging)
    pub fn pub_count(&self) -> u32 {
        self.header().pub_count()
    }

    /// Get subscriber count (for debugging)
    pub fn sub_count(&self) -> u32 {
        self.header().sub_count()
    }

    /// Get raw pointer to the SHM header (for benchmarking raw atomic latency).
    /// Returns null if the topic hasn't been initialized with SHM yet.
    pub fn local_state_header_ptr(&self) -> *const header::TopicHeader {
        self.local().cached_header_ptr
    }
}

impl<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static> Clone for Topic<T> {
    fn clone(&self) -> Self {
        Self {
            name: self.name.clone(),
            process_epoch: self.process_epoch.clone(),
            storage: self.storage.clone(),
            backend: std::cell::UnsafeCell::new(BackendStorage::Uninitialized),
            send_fn: std::cell::UnsafeCell::new(dispatch::send_uninitialized::<T>),
            recv_fn: std::cell::UnsafeCell::new(dispatch::recv_uninitialized::<T>),
            local: std::cell::UnsafeCell::new(LocalState::default()),
            metrics: Arc::clone(&self.metrics),
            log_fn: self.log_fn,
            state: AtomicU8::new(self.state.load(Ordering::Relaxed)),
            _marker: PhantomData,
        }
    }
}

impl<T> Drop for Topic<T> {
    fn drop(&mut self) {
        // Registry entries are NOT removed here — other instances may still
        // reference the same backend Arc. Entries are cleaned up automatically
        // by store_or_get_backend() when new epochs are created (old epochs
        // are retained for at most 1 generation).
    }
}

// Specialized implementation for POD types
impl<T: PodMessage + Clone + Send + Sync + Serialize + DeserializeOwned + 'static>
    Topic<T>
{
    /// Create a topic for POD types (uses zero-copy path)
    pub fn new_pod(name: &str) -> HorusResult<Self> {
        let type_size = mem::size_of::<T>() as u32;
        let type_align = mem::align_of::<T>() as u32;

        // capacity=1 for new_pod, seq_array = 1 * 8 = 8 bytes
        let total_size = Self::HEADER_SIZE + mem::size_of::<u64>() + type_size as usize;

        let storage = Arc::new(ShmRegion::new(name, total_size)?);

        // SAFETY: storage is properly sized (>= HEADER_SIZE + type_size) and aligned for TopicHeader
        let header = unsafe { &mut *(storage.as_ptr() as *mut TopicHeader) };

        if header.magic != TOPIC_MAGIC {
            header.init(type_size, type_align, true, 1, type_size);
        }

        Ok(Self {
            name: name.to_string(),
            process_epoch: registry::get_or_create_process_epoch(name),
            storage,
            backend: std::cell::UnsafeCell::new(BackendStorage::Uninitialized),
            send_fn: std::cell::UnsafeCell::new(dispatch::send_uninitialized::<T>),
            recv_fn: std::cell::UnsafeCell::new(dispatch::recv_uninitialized::<T>),
            local: std::cell::UnsafeCell::new(LocalState {
                is_pod: true,
                slot_size: type_size as usize,
                ..Default::default()
            }),
            metrics: Arc::new(MigrationMetrics::default()),
            log_fn: None,
            state: AtomicU8::new(ConnectionState::Connected.into_u8()),
            _marker: PhantomData,
        })
    }
}

// ============================================================================
// Logging Support (requires LogSummary bound)
// ============================================================================

impl<T> Topic<T>
where
    T: Clone + Send + Sync + Serialize + DeserializeOwned + crate::core::LogSummary + 'static,
{
    /// Enable automatic logging on send/recv
    pub fn with_logging(mut self) -> Self {
        self.log_fn = Some(|msg: &T| msg.log_summary());
        self
    }
}

// ============================================================================
// TopicConfig Support
// ============================================================================

impl<T: Clone + Send + Sync + Serialize + DeserializeOwned + 'static> Topic<T> {
    /// Create a topic from configuration
    pub fn from_config(config: TopicConfig) -> HorusResult<Self> {
        Self::with_capacity(&config.name, config.capacity, None)
    }

    /// Create a topic from an endpoint string
    pub fn from_endpoint(endpoint: impl Into<String>) -> HorusResult<Self> {
        Self::from_endpoint_with_capacity(endpoint, 64)
    }

    /// Create a topic from an endpoint string with custom capacity
    pub fn from_endpoint_with_capacity(
        endpoint: impl Into<String>,
        capacity: usize,
    ) -> HorusResult<Self> {
        let endpoint_str = endpoint.into();

        let topic_name = if endpoint_str.contains('@') {
            endpoint_str
                .split('@')
                .next()
                .unwrap_or(&endpoint_str)
                .to_string()
        } else {
            endpoint_str
        };

        Self::with_capacity(&topic_name, capacity as u32, None)
    }

    /// Create a Topic from configuration file
    pub fn from_config_named(topic_name: &str) -> HorusResult<Self> {
        use crate::communication::config::HorusConfig;
        let config = HorusConfig::find_and_load()?;
        let hub_config = config.get_hub(topic_name)?;
        let endpoint_str = hub_config.get_endpoint();
        Self::from_endpoint(&endpoint_str)
    }

    /// Create a Topic from a specific config file path
    pub fn from_config_file<P: AsRef<std::path::Path>>(
        config_path: P,
        topic_name: &str,
    ) -> HorusResult<Self> {
        use crate::communication::config::HorusConfig;
        let config = HorusConfig::from_file(config_path)?;
        let hub_config = config.get_hub(topic_name)?;
        let endpoint_str = hub_config.get_endpoint();
        Self::from_endpoint(&endpoint_str)
    }

    /// Create a new topic with custom slot size for large messages
    pub fn with_slot_size(
        name: impl Into<String>,
        capacity: usize,
        slot_size: usize,
    ) -> HorusResult<Self> {
        let name = name.into();
        Self::with_capacity(&name, capacity as u32, Some(slot_size))
    }

    /// Send a message to a network topic (returns Result for error handling)
    pub fn send_to_network(&self, msg: T) -> Result<(), T> {
        let result = self.try_send(msg);
        match &result {
            Ok(()) => {
                self.metrics.messages_sent.fetch_add(1, Ordering::Relaxed);
                self.state.store(
                    ConnectionState::Connected.into_u8(),
                    Ordering::Relaxed,
                );
            }
            Err(_) => {
                self.metrics.send_failures.fetch_add(1, Ordering::Relaxed);
            }
        }
        result
    }

    /// Receive a message from a network topic
    pub fn recv_from_network(
        &self,
        _ctx: &mut Option<&mut crate::core::NodeInfo>,
    ) -> Option<T> {
        let result = self.try_recv();
        if result.is_some() {
            self.metrics
                .messages_received
                .fetch_add(1, Ordering::Relaxed);
            self.state.store(
                ConnectionState::Connected.into_u8(),
                Ordering::Relaxed,
            );
        }
        result
    }

    /// Check if network topic has messages
    pub fn network_has_messages(&self) -> bool {
        self.has_message()
    }

    /// Get backend type name for network topics
    pub fn network_backend_type(&self) -> &'static str {
        self.backend_name()
    }
}
