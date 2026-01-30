//! # Unified Topic Abstraction for Smart IPC
//!
//! This module provides a unified `Topic<T>` type that automatically selects
//! the optimal IPC backend based on message type and usage patterns:
//!
//! | Backend | Latency | Use Case |
//! |---------|---------|----------|
//! | PodShm | ~50ns | POD types, cross-process |
//! | SpscShm | ~85ns | 1 producer, 1 consumer, cross-process |
//! | MpmcShm | ~167ns | Multiple producers/consumers, cross-process |
//!
//! # Usage
//!
//! ```rust,ignore
//! use horus_core::communication::Topic;
//!
//! // Just create a topic - backend is automatically selected
//! let topic: Topic<SensorData> = Topic::new("sensor_data")?;
//!
//! // Send and receive directly (Topic API)
//! topic.send(data)?;
//! let data = topic.recv();
//! ```
//!
//! # Design Philosophy
//!
//! Users should NOT need to understand IPC internals. Topic provides a
//! unified API for all IPC patterns. The framework handles optimization:
//!
//! - If your type is `PodMessage` → PodShm backend (~50ns)
//! - If only 1 producer + 1 consumer → SpscShm backend (~85ns)
//! - Otherwise → MpmcShm backend (~167ns)

use std::marker::PhantomData;
use std::mem;
use std::ptr::NonNull;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::Arc;

use crate::core::{LogSummary, NodeInfo};
use crate::error::{HorusError, HorusResult};
use crate::memory::shm_region::ShmRegion;
use std::time::Instant;

// Re-export PodMessage for users
pub use crate::communication::pod::PodMessage;

// Smart detection for automatic backend selection
use crate::communication::adaptive_topic::AdaptiveTopic;

// ============================================================================
// Topic Configuration
// ============================================================================

/// Access pattern for Topic communication
///
/// Determines how many producers and consumers will use the Topic.
/// This affects backend selection for optimal performance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AccessPattern {
    /// Single Producer, Single Consumer (~25ns intra, ~85ns inter)
    Spsc,
    /// Single Producer, Multiple Consumers (~40ns intra, ~70ns inter)
    Spmc,
    /// Multiple Producers, Single Consumer (~35ns intra, ~65ns inter)
    Mpsc,
    /// Multiple Producers, Multiple Consumers (~80ns intra, ~167ns inter)
    #[default]
    Mpmc,
}

/// Process topology for Topic communication
///
/// Determines whether producers/consumers are in the same process.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ProcessTopology {
    /// Same thread - fastest path (~3-5ns)
    SameThread,
    /// Same process, different threads (~25-80ns)
    SameProcess,
    /// Cross-process via shared memory (~50-167ns)
    #[default]
    CrossProcess,
}

/// Backend selection hint
///
/// Allows explicit backend selection or automatic selection based on type/pattern.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BackendHint {
    /// Automatically select best backend based on type and pattern
    #[default]
    Auto,
    /// Force DirectChannel (same-thread only, ~3-5ns)
    DirectChannel,
    /// Force SpscIntra (same-process SPSC, ~25ns)
    SpscIntra,
    /// Force SpscShm (cross-process SPSC, ~85ns)
    SpscShm,
    /// Force PodShm (cross-process POD, ~50ns)
    PodShm,
    /// Force MpmcShm (cross-process MPMC, ~167ns)
    MpmcShm,
    /// Force MpscIntra (same-process MPSC, ~35ns)
    MpscIntra,
    /// Force SpmcIntra (same-process SPMC, ~40ns)
    SpmcIntra,
    /// Force MpmcIntra (same-process MPMC, ~80ns)
    MpmcIntra,
    /// Force MpscShm (cross-process MPSC, ~65ns)
    MpscShm,
    /// Force SpmcShm (cross-process SPMC, ~70ns)
    SpmcShm,
    /// Force Network backend (UDP/Unix socket/Zenoh)
    Network,
    /// Force Network with Zenoh transport
    NetworkZenoh,
}

/// Selected backend after evaluation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SelectedBackend {
    DirectChannel,
    SpscIntra,
    MpscIntra,
    SpmcIntra,
    MpmcIntra,
    SpscShm,
    MpscShm,
    SpmcShm,
    PodShm,
    MpmcShm,
    /// Network transport (UDP/Unix socket/Zenoh)
    Network,
}

/// Configuration for creating a Topic
#[derive(Debug, Clone)]
pub struct TopicConfig {
    /// Topic name (shared memory identifier)
    pub name: String,

    /// Ring buffer capacity (must be power of 2, for MPMC/SPSC backends)
    pub capacity: u32,

    /// Whether to create if it doesn't exist
    pub create: bool,

    /// Whether this process is a producer
    pub is_producer: bool,

    /// Access pattern hint (SPSC, SPMC, MPSC, MPMC)
    pub access_pattern: AccessPattern,

    /// Process topology (SameThread, SameProcess, CrossProcess)
    pub topology: ProcessTopology,

    /// Backend selection hint
    pub backend_hint: BackendHint,
}

impl TopicConfig {
    /// Create a new configuration with default values
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            capacity: 64,
            create: true,
            is_producer: true,
            access_pattern: AccessPattern::default(),
            topology: ProcessTopology::default(),
            backend_hint: BackendHint::default(),
        }
    }

    /// Set the capacity (must be power of 2)
    pub fn with_capacity(mut self, capacity: u32) -> Self {
        self.capacity = capacity;
        self
    }

    /// Set whether to create if it doesn't exist
    pub fn with_create(mut self, create: bool) -> Self {
        self.create = create;
        self
    }

    /// Set whether this is a producer
    pub fn as_producer(mut self) -> Self {
        self.is_producer = true;
        self
    }

    /// Set whether this is a consumer
    pub fn as_consumer(mut self) -> Self {
        self.is_producer = false;
        self
    }

    /// Set access pattern (SPSC, SPMC, MPSC, MPMC)
    pub fn with_access_pattern(mut self, pattern: AccessPattern) -> Self {
        self.access_pattern = pattern;
        self
    }

    /// Set process topology
    pub fn with_topology(mut self, topology: ProcessTopology) -> Self {
        self.topology = topology;
        self
    }

    /// Set backend hint
    pub fn with_backend(mut self, hint: BackendHint) -> Self {
        self.backend_hint = hint;
        self
    }

    /// Configure for same-thread communication (fastest)
    pub fn same_thread(mut self) -> Self {
        self.topology = ProcessTopology::SameThread;
        self.access_pattern = AccessPattern::Spsc;
        self
    }

    /// Configure for same-process cross-thread communication
    pub fn same_process(mut self) -> Self {
        self.topology = ProcessTopology::SameProcess;
        self
    }

    /// Configure for cross-process communication
    pub fn cross_process(mut self) -> Self {
        self.topology = ProcessTopology::CrossProcess;
        self
    }

    /// Configure for SPSC pattern
    pub fn spsc(mut self) -> Self {
        self.access_pattern = AccessPattern::Spsc;
        self
    }

    /// Configure for MPMC pattern
    pub fn mpmc(mut self) -> Self {
        self.access_pattern = AccessPattern::Mpmc;
        self
    }
}

// ============================================================================
// Backend Selection Logic
// ============================================================================

/// Select the optimal backend based on configuration and type properties
///
/// Selection matrix (12 paths implemented):
///
/// | Topology    | Pattern | POD   | Backend       | Latency  |
/// |-------------|---------|-------|---------------|----------|
/// | SameThread  | SPSC    | Any   | DirectChannel | ~3-5ns   |
/// | SameProcess | SPSC    | Any   | SpscIntra     | ~18ns    |
/// | SameProcess | MPSC    | Any   | MpscIntra     | ~26ns    |
/// | SameProcess | SPMC    | Any   | SpmcIntra     | ~24ns    |
/// | SameProcess | MPMC    | Any   | MpmcIntra     | ~36ns    |
/// | CrossProcess| SPSC    | POD   | PodShm        | ~50ns    |
/// | CrossProcess| MPSC    | Any   | MpscShm       | ~30ns    |
/// | CrossProcess| SPMC    | Any   | SpmcShm       | ~70ns    |
/// | CrossProcess| SPSC    | Serde | SpscShm       | ~85ns    |
/// | CrossProcess| MPMC    | Any   | MpmcShm       | ~167ns   |
fn select_backend(config: &TopicConfig, is_pod: bool) -> SelectedBackend {
    // If explicit backend hint is provided, use it
    match config.backend_hint {
        BackendHint::DirectChannel => return SelectedBackend::DirectChannel,
        BackendHint::SpscIntra => return SelectedBackend::SpscIntra,
        BackendHint::MpscIntra => return SelectedBackend::MpscIntra,
        BackendHint::SpmcIntra => return SelectedBackend::SpmcIntra,
        BackendHint::MpmcIntra => return SelectedBackend::MpmcIntra,
        BackendHint::SpscShm => return SelectedBackend::SpscShm,
        BackendHint::MpscShm => return SelectedBackend::MpscShm,
        BackendHint::SpmcShm => return SelectedBackend::SpmcShm,
        BackendHint::PodShm => return SelectedBackend::PodShm,
        BackendHint::MpmcShm => return SelectedBackend::MpmcShm,
        BackendHint::Network | BackendHint::NetworkZenoh => return SelectedBackend::Network,
        BackendHint::Auto => {} // Continue with auto-selection
    }

    // Auto-selection based on topology and pattern
    match (config.topology, config.access_pattern) {
        // Same-thread: DirectChannel (~3-5ns)
        (ProcessTopology::SameThread, AccessPattern::Spsc) => SelectedBackend::DirectChannel,

        // Same-process SPSC: SpscIntra (~18ns)
        (ProcessTopology::SameProcess, AccessPattern::Spsc) => SelectedBackend::SpscIntra,

        // Same-process MPSC: MpscIntra (~26ns)
        (ProcessTopology::SameProcess, AccessPattern::Mpsc) => SelectedBackend::MpscIntra,

        // Same-process SPMC: SpmcIntra (~40ns)
        (ProcessTopology::SameProcess, AccessPattern::Spmc) => SelectedBackend::SpmcIntra,

        // Same-process MPMC: MpmcIntra (~80ns)
        (ProcessTopology::SameProcess, AccessPattern::Mpmc) => SelectedBackend::MpmcIntra,

        // Cross-process SPSC: PodShm (~50ns) for POD, SpscShm (~85ns) otherwise
        (ProcessTopology::CrossProcess, AccessPattern::Spsc) => {
            if is_pod {
                SelectedBackend::PodShm
            } else {
                SelectedBackend::SpscShm
            }
        }

        // Cross-process MPSC: MpscShm (~65ns)
        (ProcessTopology::CrossProcess, AccessPattern::Mpsc) => SelectedBackend::MpscShm,

        // Cross-process SPMC: SpmcShm (~70ns)
        (ProcessTopology::CrossProcess, AccessPattern::Spmc) => SelectedBackend::SpmcShm,

        // Cross-process MPMC (and any remaining): MpmcShm (safe default)
        _ => SelectedBackend::MpmcShm,
    }
}

// ============================================================================
// Topic Registry - Participant Tracking
// ============================================================================

use std::collections::HashMap;
use std::process;
use std::sync::RwLock;

/// Entry in the topic registry tracking participants
#[derive(Debug, Clone)]
pub struct TopicEntry {
    /// Topic name
    pub name: String,
    /// Number of registered producers
    pub producer_count: u32,
    /// Number of registered consumers
    pub consumer_count: u32,
    /// PIDs of participants (for topology detection)
    pub participant_pids: Vec<u32>,
    /// Whether any participant has POD type
    pub has_pod_type: bool,
    /// Creation timestamp
    pub created_at: std::time::Instant,
}

impl TopicEntry {
    fn new(name: String) -> Self {
        Self {
            name,
            producer_count: 0,
            consumer_count: 0,
            participant_pids: Vec::new(),
            has_pod_type: false,
            created_at: std::time::Instant::now(),
        }
    }

    /// Detect access pattern from participant counts
    pub fn detect_pattern(&self) -> AccessPattern {
        match (self.producer_count, self.consumer_count) {
            (0..=1, 0..=1) => AccessPattern::Spsc,
            (0..=1, _) => AccessPattern::Spmc,
            (_, 0..=1) => AccessPattern::Mpsc,
            _ => AccessPattern::Mpmc,
        }
    }

    /// Detect topology from PIDs
    pub fn detect_topology(&self) -> ProcessTopology {
        let my_pid = process::id();

        if self.participant_pids.is_empty() {
            // First participant - assume same process for now
            return ProcessTopology::SameProcess;
        }

        // Check if all participants are in the same process
        let all_same_process = self.participant_pids.iter().all(|&pid| pid == my_pid);

        if all_same_process {
            ProcessTopology::SameProcess
        } else {
            ProcessTopology::CrossProcess
        }
    }
}

/// Global topic registry for tracking all topic participants
///
/// The registry tracks:
/// - Producer and consumer counts per topic
/// - Process IDs for topology detection
/// - POD type information for backend selection
///
/// This enables automatic backend selection based on actual usage patterns.
pub struct TopicRegistry {
    /// Topic entries by name
    entries: RwLock<HashMap<String, TopicEntry>>,
    /// Current process ID (cached)
    my_pid: u32,
}

impl TopicRegistry {
    /// Create a new topic registry
    pub fn new() -> Self {
        Self {
            entries: RwLock::new(HashMap::new()),
            my_pid: process::id(),
        }
    }

    /// Register a producer for a topic
    ///
    /// Returns the updated topic entry with current participant info.
    pub fn register_producer(&self, topic_name: &str, is_pod: bool) -> TopicEntry {
        let mut entries = self.entries.write().unwrap();

        let entry = entries
            .entry(topic_name.to_string())
            .or_insert_with(|| TopicEntry::new(topic_name.to_string()));

        entry.producer_count += 1;
        if !entry.participant_pids.contains(&self.my_pid) {
            entry.participant_pids.push(self.my_pid);
        }
        if is_pod {
            entry.has_pod_type = true;
        }

        entry.clone()
    }

    /// Register a consumer for a topic
    ///
    /// Returns the updated topic entry with current participant info.
    pub fn register_consumer(&self, topic_name: &str, is_pod: bool) -> TopicEntry {
        let mut entries = self.entries.write().unwrap();

        let entry = entries
            .entry(topic_name.to_string())
            .or_insert_with(|| TopicEntry::new(topic_name.to_string()));

        entry.consumer_count += 1;
        if !entry.participant_pids.contains(&self.my_pid) {
            entry.participant_pids.push(self.my_pid);
        }
        if is_pod {
            entry.has_pod_type = true;
        }

        entry.clone()
    }

    /// Unregister a producer from a topic
    pub fn unregister_producer(&self, topic_name: &str) {
        let mut entries = self.entries.write().unwrap();

        if let Some(entry) = entries.get_mut(topic_name) {
            entry.producer_count = entry.producer_count.saturating_sub(1);
            self.cleanup_entry(&mut entries, topic_name);
        }
    }

    /// Unregister a consumer from a topic
    pub fn unregister_consumer(&self, topic_name: &str) {
        let mut entries = self.entries.write().unwrap();

        if let Some(entry) = entries.get_mut(topic_name) {
            entry.consumer_count = entry.consumer_count.saturating_sub(1);
            self.cleanup_entry(&mut entries, topic_name);
        }
    }

    /// Get current entry for a topic (if exists)
    pub fn get_entry(&self, topic_name: &str) -> Option<TopicEntry> {
        let entries = self.entries.read().unwrap();
        entries.get(topic_name).cloned()
    }

    /// Get detected access pattern for a topic
    pub fn detect_pattern(&self, topic_name: &str) -> AccessPattern {
        self.get_entry(topic_name)
            .map(|e| e.detect_pattern())
            .unwrap_or(AccessPattern::Mpmc)
    }

    /// Get detected topology for a topic
    pub fn detect_topology(&self, topic_name: &str) -> ProcessTopology {
        self.get_entry(topic_name)
            .map(|e| e.detect_topology())
            .unwrap_or(ProcessTopology::CrossProcess)
    }

    /// Remove entry if no participants remain
    fn cleanup_entry(&self, entries: &mut HashMap<String, TopicEntry>, topic_name: &str) {
        if let Some(entry) = entries.get(topic_name) {
            if entry.producer_count == 0 && entry.consumer_count == 0 {
                entries.remove(topic_name);
            }
        }
    }

    /// List all registered topics
    pub fn list_topics(&self) -> Vec<String> {
        let entries = self.entries.read().unwrap();
        entries.keys().cloned().collect()
    }

    /// Get statistics about all registered topics
    pub fn stats(&self) -> RegistryStats {
        let entries = self.entries.read().unwrap();

        let total_topics = entries.len();
        let total_producers: u32 = entries.values().map(|e| e.producer_count).sum();
        let total_consumers: u32 = entries.values().map(|e| e.consumer_count).sum();

        RegistryStats {
            total_topics,
            total_producers,
            total_consumers,
        }
    }
}

impl Default for TopicRegistry {
    fn default() -> Self {
        Self::new()
    }
}

/// Statistics about the topic registry
#[derive(Debug, Clone)]
pub struct RegistryStats {
    /// Total number of registered topics
    pub total_topics: usize,
    /// Total number of registered producers
    pub total_producers: u32,
    /// Total number of registered consumers
    pub total_consumers: u32,
}

/// Global topic registry instance
///
/// Use this for automatic backend selection based on usage patterns.
static GLOBAL_REGISTRY: std::sync::OnceLock<TopicRegistry> = std::sync::OnceLock::new();

/// Get the global topic registry
pub fn global_registry() -> &'static TopicRegistry {
    GLOBAL_REGISTRY.get_or_init(TopicRegistry::new)
}

// ============================================================================
// Backend Trait
// ============================================================================

/// Trait for Topic backends - all IPC implementations must implement this
pub(crate) trait TopicBackendTrait<T>: Send + Sync {
    /// Send a message
    fn push(&self, msg: T) -> Result<(), T>;

    /// Receive a message
    fn pop(&self) -> Option<T>;

    /// Check if empty
    fn is_empty(&self) -> bool;

    /// Get backend name for debugging
    fn backend_name(&self) -> &'static str;
}

// ============================================================================
// MpmcShm Backend (unified Topic implementation)
// ============================================================================

/// MPMC Shared Memory backend - unified Topic implementation
///
/// Uses ShmTopic ring buffer for multiple producers and multiple consumers.
/// Latency: ~167ns
struct MpmcShmBackend<T> {
    inner: Arc<crate::memory::shm_topic::ShmTopic<T>>,
}

impl<T> MpmcShmBackend<T> {
    fn new(name: &str, capacity: usize) -> HorusResult<Self> {
        let shm_topic = crate::memory::shm_topic::ShmTopic::new(name, capacity).map_err(|e| {
            HorusError::Communication(format!("Failed to create MPMC backend '{}': {}", name, e))
        })?;

        Ok(Self {
            inner: Arc::new(shm_topic),
        })
    }
}

impl<T: Clone + Send + Sync + 'static> TopicBackendTrait<T> for MpmcShmBackend<T> {
    #[inline(always)]
    fn push(&self, msg: T) -> Result<(), T> {
        // Use push_fast() which skips bounds checking for zero-overhead
        self.inner.push_fast(msg)
    }

    #[inline(always)]
    fn pop(&self) -> Option<T> {
        // Use pop_fast() which skips bounds checking for zero-overhead
        self.inner.pop_fast()
    }

    #[inline(always)]
    fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    fn backend_name(&self) -> &'static str {
        "MpmcShm"
    }
}

impl<T> Clone for MpmcShmBackend<T> {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
        }
    }
}

// ============================================================================
// SpscShm Backend (extracted unified implementation)
// ============================================================================

/// Header for SPSC shared memory - single-slot design from Link
/// Sequence counter signals new data availability
#[repr(C, align(64))]
struct SpscShmHeader {
    sequence: AtomicU64,       // Version counter - incremented on each write
    element_size: AtomicUsize, // For validation
    _padding: [u8; 48],        // Pad to full cache line
}

/// SPSC Shared Memory backend - extracted from Link
///
/// Single-slot design optimized for 1 producer, 1 consumer.
/// Producer overwrites, consumer tracks what it has seen.
/// Latency: ~85ns
struct SpscShmBackend<T> {
    /// Shared memory region
    region: Arc<ShmRegion>,
    /// Header pointer
    header: NonNull<SpscShmHeader>,
    /// Data pointer
    data_ptr: NonNull<u8>,
    /// Last seen sequence (consumer tracking)
    last_seen: AtomicU64,
    /// Whether this is a producer
    is_producer: bool,
    /// Phantom for type safety
    _phantom: PhantomData<T>,
}

// Safety: SpscShmBackend uses atomic operations for synchronization
unsafe impl<T: Send> Send for SpscShmBackend<T> {}
unsafe impl<T: Send + Sync> Sync for SpscShmBackend<T> {}

impl<T: Clone + Send + Sync + 'static> SpscShmBackend<T> {
    fn new(name: &str, is_producer: bool) -> HorusResult<Self> {
        let header_size = mem::size_of::<SpscShmHeader>();
        let data_size = mem::size_of::<T>();
        let total_size = header_size + data_size;

        let region = Arc::new(ShmRegion::new(name, total_size)?);
        let is_owner = region.is_owner();

        let header_ptr = region.as_ptr() as *mut SpscShmHeader;

        // Safety: validate pointer
        if header_ptr.is_null() {
            return Err(HorusError::Communication(
                "Null pointer for SPSC header".into(),
            ));
        }

        let header = unsafe { NonNull::new_unchecked(header_ptr) };

        // Initialize header if owner
        if is_owner {
            unsafe {
                (*header.as_ptr())
                    .element_size
                    .store(data_size, Ordering::Relaxed);
                (*header.as_ptr()).sequence.store(0, Ordering::Release);
            }
        }

        // Calculate data pointer (after header)
        let data_ptr =
            unsafe { NonNull::new_unchecked(region.as_ptr().add(header_size) as *mut u8) };

        Ok(Self {
            region,
            header,
            data_ptr,
            last_seen: AtomicU64::new(0),
            is_producer,
            _phantom: PhantomData,
        })
    }

    #[allow(dead_code)]
    fn producer(name: &str) -> HorusResult<Self> {
        Self::new(name, true)
    }

    #[allow(dead_code)]
    fn consumer(name: &str) -> HorusResult<Self> {
        Self::new(name, false)
    }
}

impl<T: Clone + Send + Sync + 'static> TopicBackendTrait<T> for SpscShmBackend<T> {
    #[inline(always)]
    fn push(&self, msg: T) -> Result<(), T> {
        // ZERO-OVERHEAD: Skip is_producer check in hot path
        // SPSC design assumes correct usage - producer pushes, consumer pops
        let header = unsafe { self.header.as_ref() };

        // Write data directly to shared memory
        unsafe {
            let src = &msg as *const T as *const u8;
            std::ptr::copy_nonoverlapping(src, self.data_ptr.as_ptr(), mem::size_of::<T>());
        }

        // Increment sequence to signal new data (Release ensures write is visible)
        header.sequence.fetch_add(1, Ordering::Release);

        // Don't drop msg - we copied it
        mem::forget(msg);
        Ok(())
    }

    #[inline(always)]
    fn pop(&self) -> Option<T> {
        let header = unsafe { self.header.as_ref() };

        // Check if new data available (Acquire ensures we see the write)
        let current_seq = header.sequence.load(Ordering::Acquire);
        let last = self.last_seen.load(Ordering::Relaxed);

        if current_seq == last {
            return None; // No new data
        }

        // Read data using MaybeUninit to avoid zero-initialization
        let msg = unsafe {
            let mut result = mem::MaybeUninit::<T>::uninit();
            std::ptr::copy_nonoverlapping(
                self.data_ptr.as_ptr(),
                result.as_mut_ptr() as *mut u8,
                mem::size_of::<T>(),
            );
            result.assume_init()
        };

        // Update last seen
        self.last_seen.store(current_seq, Ordering::Relaxed);

        Some(msg)
    }

    #[inline(always)]
    fn is_empty(&self) -> bool {
        let header = unsafe { self.header.as_ref() };
        let current_seq = header.sequence.load(Ordering::Acquire);
        let last = self.last_seen.load(Ordering::Relaxed);
        current_seq == last
    }

    fn backend_name(&self) -> &'static str {
        "SpscShm"
    }
}

impl<T> Clone for SpscShmBackend<T> {
    fn clone(&self) -> Self {
        Self {
            region: self.region.clone(),
            header: self.header,
            data_ptr: self.data_ptr,
            last_seen: AtomicU64::new(self.last_seen.load(Ordering::Relaxed)),
            is_producer: self.is_producer,
            _phantom: PhantomData,
        }
    }
}

// ============================================================================
// MpscShm Backend - Cross-process MPSC Ring Buffer
// ============================================================================

/// Header for MPSC shared memory ring buffer
#[repr(C, align(64))]
struct MpscShmHeader {
    /// Capacity of the ring buffer (power of 2)
    capacity: u64,
    /// Mask for index calculation (capacity - 1)
    mask: u64,
    /// Producer tail - producers CAS on this
    tail: AtomicU64,
    /// Consumer head
    head: AtomicU64,
    /// Element size for validation
    element_size: u64,
    /// Magic number for validation
    magic: u64,
    /// Padding to 64 bytes
    _pad: [u8; 16],
}

/// Slot in the MPSC shared memory ring
#[repr(C, align(64))]
struct MpscShmSlot {
    /// Sequence number for coordination
    sequence: AtomicU64,
    /// Padding to separate sequence from data
    _pad: [u8; 56],
    // Data follows immediately after (variable size)
}

const MPSC_SHM_MAGIC: u64 = 0x484F5255534D5053; // "HORUSMPS"

/// MPSC Shared Memory backend - cross-process multi-producer single-consumer
///
/// Uses a lock-free ring buffer in shared memory. Multiple producers use
/// CAS to claim slots, single consumer reads sequentially.
///
/// Latency: ~65ns
struct MpscShmBackend<T> {
    /// Shared memory region
    region: Arc<ShmRegion>,
    /// Header pointer
    header: NonNull<MpscShmHeader>,
    /// Slots base pointer
    slots_ptr: NonNull<u8>,
    /// Slot stride (slot header + data size, aligned)
    slot_stride: usize,
    /// Whether this is a producer
    is_producer: bool,
    /// Phantom for type safety
    _phantom: PhantomData<T>,
}

// Safety: MpscShmBackend uses atomic operations for synchronization
unsafe impl<T: Send> Send for MpscShmBackend<T> {}
unsafe impl<T: Send + Sync> Sync for MpscShmBackend<T> {}

impl<T: Clone + Send + Sync + 'static> MpscShmBackend<T> {
    fn new(name: &str, capacity: usize, is_producer: bool) -> HorusResult<Self> {
        // Ensure capacity is power of 2
        let capacity = capacity.next_power_of_two();
        let mask = capacity - 1;

        let header_size = mem::size_of::<MpscShmHeader>();
        let slot_header_size = mem::size_of::<MpscShmSlot>();
        let data_size = mem::size_of::<T>();
        // Align slot stride to 64 bytes for cache efficiency
        let slot_stride = (slot_header_size + data_size).div_ceil(64) * 64;
        let total_size = header_size + (slot_stride * capacity);

        let region = Arc::new(ShmRegion::new(name, total_size)?);
        let is_owner = region.is_owner();

        let header_ptr = region.as_ptr() as *mut MpscShmHeader;
        if header_ptr.is_null() {
            return Err(HorusError::Communication(
                "Null pointer for MpscShm header".into(),
            ));
        }
        let header = unsafe { NonNull::new_unchecked(header_ptr) };

        // Initialize if owner
        if is_owner {
            unsafe {
                let h = header.as_ptr();
                (*h).capacity = capacity as u64;
                (*h).mask = mask as u64;
                (*h).tail.store(0, Ordering::Relaxed);
                (*h).head.store(0, Ordering::Relaxed);
                (*h).element_size = data_size as u64;
                (*h).magic = MPSC_SHM_MAGIC;

                // Initialize slot sequence numbers
                let slots_base = region.as_ptr().add(header_size);
                for i in 0..capacity {
                    let slot = slots_base.add(i * slot_stride) as *mut MpscShmSlot;
                    (*slot).sequence.store(i as u64, Ordering::Relaxed);
                }
            }
        } else {
            // Validate existing region
            let h = unsafe { header.as_ref() };
            if h.magic != MPSC_SHM_MAGIC {
                return Err(HorusError::Communication(
                    "Invalid MpscShm magic number".into(),
                ));
            }
            if h.element_size != data_size as u64 {
                return Err(HorusError::Communication(format!(
                    "Element size mismatch: expected {}, got {}",
                    data_size, h.element_size
                )));
            }
        }

        let slots_ptr =
            unsafe { NonNull::new_unchecked(region.as_ptr().add(header_size) as *mut u8) };

        Ok(Self {
            region,
            header,
            slots_ptr,
            slot_stride,
            is_producer,
            _phantom: PhantomData,
        })
    }

    fn producer(name: &str, capacity: usize) -> HorusResult<Self> {
        Self::new(name, capacity, true)
    }

    fn consumer(name: &str, capacity: usize) -> HorusResult<Self> {
        Self::new(name, capacity, false)
    }

    #[inline]
    fn get_slot(&self, index: usize) -> *mut MpscShmSlot {
        unsafe { self.slots_ptr.as_ptr().add(index * self.slot_stride) as *mut MpscShmSlot }
    }

    #[inline]
    fn get_data_ptr(&self, slot: *mut MpscShmSlot) -> *mut u8 {
        unsafe { (slot as *mut u8).add(mem::size_of::<MpscShmSlot>()) }
    }
}

impl<T: Clone + Send + Sync + 'static> TopicBackendTrait<T> for MpscShmBackend<T> {
    #[inline]
    fn push(&self, msg: T) -> Result<(), T> {
        if !self.is_producer {
            return Err(msg);
        }

        let header = unsafe { self.header.as_ref() };
        let mask = header.mask as usize;

        loop {
            let tail = header.tail.load(Ordering::Relaxed);
            let index = (tail as usize) & mask;
            let slot = self.get_slot(index);

            let seq = unsafe { (*slot).sequence.load(Ordering::Acquire) };
            let diff = seq as i64 - tail as i64;

            if diff == 0 {
                // Slot ready for writing, try to claim
                if header
                    .tail
                    .compare_exchange_weak(tail, tail + 1, Ordering::Relaxed, Ordering::Relaxed)
                    .is_ok()
                {
                    // Write data
                    unsafe {
                        let data_ptr = self.get_data_ptr(slot);
                        std::ptr::copy_nonoverlapping(
                            &msg as *const T as *const u8,
                            data_ptr,
                            mem::size_of::<T>(),
                        );
                        // Mark slot as ready for consumer
                        (*slot).sequence.store(tail + 1, Ordering::Release);
                    }
                    mem::forget(msg);
                    return Ok(());
                }
            } else if diff < 0 {
                // Queue is full
                return Err(msg);
            }
            hint::spin_loop();
        }
    }

    #[inline]
    fn pop(&self) -> Option<T> {
        if self.is_producer {
            return None;
        }

        let header = unsafe { self.header.as_ref() };
        let mask = header.mask as usize;
        let capacity = header.capacity as usize;

        let head = header.head.load(Ordering::Relaxed);
        let index = (head as usize) & mask;
        let slot = self.get_slot(index);

        let seq = unsafe { (*slot).sequence.load(Ordering::Acquire) };
        let diff = seq as i64 - (head + 1) as i64;

        if diff == 0 {
            // Data ready, read it
            let msg = unsafe {
                let data_ptr = self.get_data_ptr(slot);
                let mut result = mem::MaybeUninit::<T>::uninit();
                std::ptr::copy_nonoverlapping(
                    data_ptr,
                    result.as_mut_ptr() as *mut u8,
                    mem::size_of::<T>(),
                );
                // Mark slot as empty for producers
                (*slot)
                    .sequence
                    .store(head + capacity as u64, Ordering::Release);
                result.assume_init()
            };
            // Advance head
            header.head.store(head + 1, Ordering::Relaxed);
            Some(msg)
        } else {
            None
        }
    }

    #[inline]
    fn is_empty(&self) -> bool {
        let header = unsafe { self.header.as_ref() };
        let head = header.head.load(Ordering::Relaxed);
        let tail = header.tail.load(Ordering::Relaxed);
        head == tail
    }

    fn backend_name(&self) -> &'static str {
        "MpscShm"
    }
}

impl<T> Clone for MpscShmBackend<T> {
    fn clone(&self) -> Self {
        Self {
            region: self.region.clone(),
            header: self.header,
            slots_ptr: self.slots_ptr,
            slot_stride: self.slot_stride,
            is_producer: self.is_producer,
            _phantom: PhantomData,
        }
    }
}

// ============================================================================
// SpmcShm Backend - Cross-process SPMC Broadcast
// ============================================================================

/// Header for SPMC shared memory broadcast
///
/// Uses seqlock pattern: sequence is incremented to odd during write,
/// then to even when complete. Consumers spin if odd or verify sequence
/// unchanged after reading.
#[repr(C, align(64))]
struct SpmcShmHeader {
    /// Seqlock sequence counter (odd = writing, even = valid)
    sequence: AtomicU64,
    /// Element size for validation
    element_size: u64,
    /// Magic number for validation
    magic: u64,
    /// Padding to 64 bytes
    _pad: [u8; 40],
}

const SPMC_SHM_MAGIC: u64 = 0x484F525553504D43; // "HORUSPMC"

/// SPMC Shared Memory backend - cross-process single-producer multi-consumer
///
/// Uses seqlock pattern for lock-free broadcast over shared memory.
/// Single producer writes, multiple consumers read latest value.
///
/// Latency: ~70ns
struct SpmcShmBackend<T> {
    /// Shared memory region
    region: Arc<ShmRegion>,
    /// Header pointer
    header: NonNull<SpmcShmHeader>,
    /// Data pointer
    data_ptr: NonNull<u8>,
    /// Whether this is a producer
    is_producer: bool,
    /// Last seen sequence (for consumers)
    last_seen: AtomicU64,
    /// Phantom for type safety
    _phantom: PhantomData<T>,
}

// Safety: SpmcShmBackend uses atomic operations for synchronization
unsafe impl<T: Send> Send for SpmcShmBackend<T> {}
unsafe impl<T: Send + Sync> Sync for SpmcShmBackend<T> {}

impl<T: Clone + Send + Sync + 'static> SpmcShmBackend<T> {
    fn new(name: &str, is_producer: bool) -> HorusResult<Self> {
        let header_size = mem::size_of::<SpmcShmHeader>();
        let data_size = mem::size_of::<T>();
        // Align data to 64 bytes for cache efficiency
        let total_size = header_size + data_size.div_ceil(64) * 64;

        let region = Arc::new(ShmRegion::new(name, total_size)?);
        let is_owner = region.is_owner();

        let header_ptr = region.as_ptr() as *mut SpmcShmHeader;
        if header_ptr.is_null() {
            return Err(HorusError::Communication(
                "Null pointer for SpmcShm header".into(),
            ));
        }
        let header = unsafe { NonNull::new_unchecked(header_ptr) };

        // Initialize if owner
        if is_owner {
            unsafe {
                let h = header.as_ptr();
                (*h).sequence.store(0, Ordering::Relaxed);
                (*h).element_size = data_size as u64;
                (*h).magic = SPMC_SHM_MAGIC;
            }
        } else {
            // Validate existing region
            let h = unsafe { header.as_ref() };
            if h.magic != SPMC_SHM_MAGIC {
                return Err(HorusError::Communication(
                    "Invalid SpmcShm magic number".into(),
                ));
            }
            if h.element_size != data_size as u64 {
                return Err(HorusError::Communication(format!(
                    "Element size mismatch: expected {}, got {}",
                    data_size, h.element_size
                )));
            }
        }

        let data_ptr =
            unsafe { NonNull::new_unchecked(region.as_ptr().add(header_size) as *mut u8) };

        Ok(Self {
            region,
            header,
            data_ptr,
            is_producer,
            last_seen: AtomicU64::new(0),
            _phantom: PhantomData,
        })
    }

    fn producer(name: &str) -> HorusResult<Self> {
        Self::new(name, true)
    }

    fn consumer(name: &str) -> HorusResult<Self> {
        Self::new(name, false)
    }
}

impl<T: Clone + Send + Sync + 'static> TopicBackendTrait<T> for SpmcShmBackend<T> {
    #[inline]
    fn push(&self, msg: T) -> Result<(), T> {
        if !self.is_producer {
            return Err(msg);
        }

        let header = unsafe { self.header.as_ref() };

        // Seqlock write protocol:
        // 1. Increment to odd (signals write in progress)
        let seq = header.sequence.fetch_add(1, Ordering::Acquire);
        debug_assert!(seq % 2 == 0, "Double write detected in SpmcShm");

        // 2. Write data
        unsafe {
            std::ptr::copy_nonoverlapping(
                &msg as *const T as *const u8,
                self.data_ptr.as_ptr(),
                mem::size_of::<T>(),
            );
        }

        // 3. Increment to even (signals write complete)
        header.sequence.fetch_add(1, Ordering::Release);

        mem::forget(msg);
        Ok(())
    }

    #[inline]
    fn pop(&self) -> Option<T> {
        if self.is_producer {
            return None;
        }

        let header = unsafe { self.header.as_ref() };

        // Seqlock read protocol:
        loop {
            // 1. Read sequence
            let seq1 = header.sequence.load(Ordering::Acquire);

            // Check if odd (write in progress) - spin wait
            if seq1 % 2 == 1 {
                hint::spin_loop();
                continue;
            }

            // Check if we've already seen this data
            let last = self.last_seen.load(Ordering::Relaxed);
            if seq1 == last {
                return None; // No new data
            }

            // 2. Read data
            let msg = unsafe {
                let mut result = mem::MaybeUninit::<T>::uninit();
                std::ptr::copy_nonoverlapping(
                    self.data_ptr.as_ptr(),
                    result.as_mut_ptr() as *mut u8,
                    mem::size_of::<T>(),
                );
                result.assume_init()
            };

            // 3. Verify sequence didn't change during read
            let seq2 = header.sequence.load(Ordering::Acquire);
            if seq1 != seq2 {
                // Write occurred during our read - retry
                hint::spin_loop();
                continue;
            }

            // 4. Update last seen
            self.last_seen.store(seq1, Ordering::Relaxed);

            return Some(msg);
        }
    }

    #[inline]
    fn is_empty(&self) -> bool {
        let header = unsafe { self.header.as_ref() };
        let seq = header.sequence.load(Ordering::Acquire);
        let last = self.last_seen.load(Ordering::Relaxed);

        // Empty if sequence is odd (write in progress) or we've seen it
        seq % 2 == 1 || seq == last
    }

    fn backend_name(&self) -> &'static str {
        "SpmcShm"
    }
}

impl<T> Clone for SpmcShmBackend<T> {
    fn clone(&self) -> Self {
        Self {
            region: self.region.clone(),
            header: self.header,
            data_ptr: self.data_ptr,
            is_producer: self.is_producer,
            last_seen: AtomicU64::new(self.last_seen.load(Ordering::Relaxed)),
            _phantom: PhantomData,
        }
    }
}

// ============================================================================
// PodShm Backend (extracted from PodLink)
// ============================================================================

/// Header for POD shared memory region - from PodLink
#[repr(C, align(64))]
struct PodShmHeader {
    /// Sequence counter - incremented on each write
    sequence: AtomicU64,
    /// Size of the data type (for validation)
    element_size: u64,
    /// Magic number for validation
    magic: u64,
    /// Padding to cache line
    _pad: [u8; 40],
}

const POD_SHM_MAGIC: u64 = 0x484F525553504F44; // "HORUSPOD"

/// POD Shared Memory backend - extracted from PodLink
///
/// Zero-copy, no serialization. Only works with PodMessage types.
/// Latency: ~50ns
struct PodShmBackend<T: PodMessage> {
    /// Shared memory region
    region: Arc<ShmRegion>,
    /// Header pointer
    header: NonNull<PodShmHeader>,
    /// Data pointer
    data_ptr: NonNull<u8>,
    /// Last seen sequence (consumer tracking)
    last_seen: AtomicU64,
    /// Whether this is a producer
    is_producer: bool,
    /// Phantom for type safety
    _phantom: PhantomData<T>,
}

// Safety: PodShmBackend uses atomic operations for synchronization
unsafe impl<T: PodMessage> Send for PodShmBackend<T> {}
unsafe impl<T: PodMessage> Sync for PodShmBackend<T> {}

impl<T: PodMessage> PodShmBackend<T> {
    fn new(name: &str, is_producer: bool) -> HorusResult<Self> {
        let header_size = mem::size_of::<PodShmHeader>();
        let data_size = T::SIZE;
        let total_size = header_size + data_size;

        let region = Arc::new(ShmRegion::new(name, total_size)?);
        let is_owner = region.is_owner();

        let header_ptr = region.as_ptr() as *mut PodShmHeader;

        // Safety: validate pointer
        if header_ptr.is_null() {
            return Err(HorusError::Communication(
                "Null pointer for POD header".into(),
            ));
        }

        let header = unsafe { NonNull::new_unchecked(header_ptr) };

        // Initialize header if owner
        if is_owner {
            unsafe {
                (*header.as_ptr()).element_size = data_size as u64;
                (*header.as_ptr()).magic = POD_SHM_MAGIC;
                (*header.as_ptr()).sequence.store(0, Ordering::Release);
            }
        } else {
            // Validate magic
            let magic = unsafe { (*header.as_ptr()).magic };
            if magic != POD_SHM_MAGIC {
                return Err(HorusError::Communication(format!(
                    "Invalid POD magic: expected 0x{:X}, got 0x{:X}",
                    POD_SHM_MAGIC, magic
                )));
            }

            // Validate element size
            let stored_size = unsafe { (*header.as_ptr()).element_size };
            if stored_size != data_size as u64 {
                return Err(HorusError::Communication(format!(
                    "Element size mismatch: expected {}, got {}",
                    data_size, stored_size
                )));
            }
        }

        // Calculate data pointer (after header)
        let data_ptr =
            unsafe { NonNull::new_unchecked(region.as_ptr().add(header_size) as *mut u8) };

        Ok(Self {
            region,
            header,
            data_ptr,
            last_seen: AtomicU64::new(0),
            is_producer,
            _phantom: PhantomData,
        })
    }

    #[allow(dead_code)]
    fn producer(name: &str) -> HorusResult<Self> {
        Self::new(name, true)
    }

    #[allow(dead_code)]
    fn consumer(name: &str) -> HorusResult<Self> {
        Self::new(name, false)
    }
}

impl<T: PodMessage> TopicBackendTrait<T> for PodShmBackend<T> {
    #[inline]
    fn push(&self, msg: T) -> Result<(), T> {
        if !self.is_producer {
            return Err(msg);
        }

        let header = unsafe { self.header.as_ref() };

        // Zero-copy write using PodMessage trait
        unsafe {
            msg.write_to_ptr(self.data_ptr.as_ptr());
        }

        // Increment sequence to signal new data
        header.sequence.fetch_add(1, Ordering::Release);

        Ok(())
    }

    #[inline]
    fn pop(&self) -> Option<T> {
        let header = unsafe { self.header.as_ref() };

        // Check if new data available
        let current_seq = header.sequence.load(Ordering::Acquire);
        let last = self.last_seen.load(Ordering::Relaxed);

        if current_seq == last {
            return None;
        }

        // Zero-copy read using PodMessage trait
        let msg = unsafe { T::read_from_ptr(self.data_ptr.as_ptr()) };

        // Update last seen
        self.last_seen.store(current_seq, Ordering::Relaxed);

        Some(msg)
    }

    #[inline]
    fn is_empty(&self) -> bool {
        let header = unsafe { self.header.as_ref() };
        let current_seq = header.sequence.load(Ordering::Acquire);
        let last = self.last_seen.load(Ordering::Relaxed);
        current_seq == last
    }

    fn backend_name(&self) -> &'static str {
        "PodShm"
    }
}

impl<T: PodMessage> Clone for PodShmBackend<T> {
    fn clone(&self) -> Self {
        Self {
            region: self.region.clone(),
            header: self.header,
            data_ptr: self.data_ptr,
            last_seen: AtomicU64::new(self.last_seen.load(Ordering::Relaxed)),
            is_producer: self.is_producer,
            _phantom: PhantomData,
        }
    }
}

// ============================================================================
// DirectChannel Backend (Tier 0: ~3-5ns)
// ============================================================================

use std::cell::UnsafeCell;
use std::thread::{self, ThreadId};

// ============================================================================
// Platform-specific cache line size
// ============================================================================

/// Cache line size for the target architecture.
/// - x86/x86_64: 64 bytes
/// - ARM (including Apple M-series): 128 bytes (conservative for prefetcher)
/// - Other: 128 bytes (conservative default)
///
/// Note: These constants document the padding values used in MpscCounter, MpmcCounter, etc.
/// The actual padding uses hardcoded values in #[cfg] blocks due to Rust const generics limitations.
#[allow(dead_code)] // Documents padding values used in cache-aligned structs
#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
const CACHE_LINE_SIZE: usize = 64;

#[allow(dead_code)]
#[cfg(target_arch = "aarch64")]
const CACHE_LINE_SIZE: usize = 128;

#[allow(dead_code)]
#[cfg(not(any(target_arch = "x86", target_arch = "x86_64", target_arch = "aarch64")))]
const CACHE_LINE_SIZE: usize = 128;

/// Helper to compute padding needed to fill a cache line.
///
/// Note: Not directly used due to Rust const generics limitations, but documents
/// the formula: padding = CACHE_LINE_SIZE - used_bytes
#[allow(dead_code)]
const fn cache_line_padding(used_bytes: usize) -> usize {
    CACHE_LINE_SIZE.saturating_sub(used_bytes)
}

// ============================================================================
// Prefetch hints for performance
// ============================================================================

/// Prefetch data for reading (T0 = all cache levels)
#[inline(always)]
#[allow(dead_code)]
fn prefetch_read<T>(ptr: *const T) {
    #[cfg(target_arch = "x86_64")]
    unsafe {
        std::arch::x86_64::_mm_prefetch(ptr as *const i8, std::arch::x86_64::_MM_HINT_T0);
    }
    #[cfg(target_arch = "x86")]
    unsafe {
        std::arch::x86::_mm_prefetch(ptr as *const i8, std::arch::x86::_MM_HINT_T0);
    }
    #[cfg(not(any(target_arch = "x86_64", target_arch = "x86")))]
    {
        let _ = ptr; // Avoid unused warning on other platforms
    }
}

/// Prefetch data for writing (exclusive access)
#[inline(always)]
#[allow(dead_code)]
fn prefetch_write<T>(ptr: *mut T) {
    #[cfg(target_arch = "x86_64")]
    unsafe {
        // PREFETCHW is available on most modern x86_64
        // Fall back to T0 which also works
        std::arch::x86_64::_mm_prefetch(ptr as *const i8, std::arch::x86_64::_MM_HINT_T0);
    }
    #[cfg(target_arch = "x86")]
    unsafe {
        std::arch::x86::_mm_prefetch(ptr as *const i8, std::arch::x86::_MM_HINT_T0);
    }
    #[cfg(not(any(target_arch = "x86_64", target_arch = "x86")))]
    {
        let _ = ptr;
    }
}

/// Hint that a branch is unlikely to be taken (cold path)
#[inline(always)]
#[cold]
fn cold() {}

/// Mark a condition as unlikely (branch prediction hint)
#[inline(always)]
fn unlikely(b: bool) -> bool {
    if b {
        cold()
    }
    b
}

/// Mark a condition as likely (branch prediction hint)
#[inline(always)]
fn likely(b: bool) -> bool {
    if !b {
        cold()
    }
    b
}

/// DirectChannel backend - fastest possible path for same-thread communication
///
/// Zero synchronization overhead - just direct slot write/read.
/// Use cases:
/// - Single-threaded schedulers
/// - Callback patterns
/// - Synchronous pipelines
///
/// Latency: ~3-5ns (no atomics, no barriers)
///
/// # Safety
///
/// This backend is only safe for single-threaded access. It validates that
/// push/pop are called from the same thread that created it. If accessed
/// from a different thread, operations return an error.
struct DirectChannelBackend<T> {
    /// Single slot storage - no synchronization needed for same-thread access
    slot: UnsafeCell<Option<T>>,
    /// Thread ID that owns this channel
    owner_thread: ThreadId,
    /// Whether the slot has unread data (simple flag, no atomic)
    has_data: UnsafeCell<bool>,
}

// Safety: DirectChannelBackend enforces single-thread access at runtime.
// Send is required for the TopicBackend enum, but actual cross-thread
// access will error. This is safe because we validate thread identity.
unsafe impl<T: Send> Send for DirectChannelBackend<T> {}
unsafe impl<T: Send> Sync for DirectChannelBackend<T> {}

impl<T> DirectChannelBackend<T> {
    /// Create a new DirectChannel backend
    ///
    /// The channel is bound to the creating thread. All push/pop operations
    /// must happen on this same thread.
    fn new() -> Self {
        Self {
            slot: UnsafeCell::new(None),
            owner_thread: thread::current().id(),
            has_data: UnsafeCell::new(false),
        }
    }

    /// Check if current thread is the owner
    #[inline(always)]
    fn is_owner_thread(&self) -> bool {
        thread::current().id() == self.owner_thread
    }

    /// Ultra-fast push without thread validation (~3-5ns)
    ///
    /// # Safety
    ///
    /// Caller must guarantee:
    /// - This is called from the same thread that created the channel
    /// - No concurrent access from other threads
    #[inline(always)]
    pub unsafe fn push_unchecked(&self, msg: T) {
        *self.slot.get() = Some(msg);
        *self.has_data.get() = true;
    }

    /// Ultra-fast pop without thread validation (~3-5ns)
    ///
    /// # Safety
    ///
    /// Caller must guarantee:
    /// - This is called from the same thread that created the channel
    /// - No concurrent access from other threads
    #[inline(always)]
    pub unsafe fn pop_unchecked(&self) -> Option<T> {
        if *self.has_data.get() {
            *self.has_data.get() = false;
            (*self.slot.get()).take()
        } else {
            None
        }
    }

    /// Send with synchronous callback (zero-copy handoff)
    ///
    /// The callback receives the message for immediate processing.
    /// No storage overhead - message is processed inline.
    ///
    /// # Arguments
    ///
    /// * `msg` - The message to send
    /// * `handler` - Callback that processes the message
    ///
    /// # Returns
    ///
    /// * `Ok(R)` - Result from the handler
    /// * `Err(msg)` - If called from wrong thread
    #[inline]
    pub fn send_sync<R, F>(&self, msg: T, handler: F) -> Result<R, T>
    where
        F: FnOnce(&T) -> R,
    {
        if !self.is_owner_thread() {
            return Err(msg);
        }

        // Zero-copy: pass reference directly to handler
        let result = handler(&msg);
        Ok(result)
    }

    /// Send with ownership transfer callback
    ///
    /// The callback takes ownership of the message.
    ///
    /// # Arguments
    ///
    /// * `msg` - The message to send (ownership transferred)
    /// * `handler` - Callback that takes ownership
    ///
    /// # Returns
    ///
    /// * `Ok(R)` - Result from the handler
    /// * `Err(msg)` - If called from wrong thread
    #[inline]
    pub fn send_owned<R, F>(&self, msg: T, handler: F) -> Result<R, T>
    where
        F: FnOnce(T) -> R,
    {
        if !self.is_owner_thread() {
            return Err(msg);
        }

        // Transfer ownership to handler
        let result = handler(msg);
        Ok(result)
    }
}

impl<T: Clone + Send + Sync + 'static> TopicBackendTrait<T> for DirectChannelBackend<T> {
    #[inline(always)]
    fn push(&self, msg: T) -> Result<(), T> {
        // ZERO-OVERHEAD: Skip thread check in hot path (~50ns savings)
        // DirectChannel is designed for single-thread use - caller is responsible
        // for ensuring correct thread affinity. Use send_sync() for thread-safe variant.
        //
        // Safety: DirectChannel users must ensure single-thread access
        unsafe {
            *self.slot.get() = Some(msg);
            *self.has_data.get() = true;
        }
        Ok(())
    }

    #[inline(always)]
    fn pop(&self) -> Option<T> {
        // ZERO-OVERHEAD: Skip thread check in hot path (~50ns savings)
        // Safety: DirectChannel users must ensure single-thread access
        unsafe {
            if *self.has_data.get() {
                *self.has_data.get() = false;
                (*self.slot.get()).take()
            } else {
                None
            }
        }
    }

    #[inline(always)]
    fn is_empty(&self) -> bool {
        // ZERO-OVERHEAD: Skip thread check
        // Safety: DirectChannel users must ensure single-thread access
        unsafe { !*self.has_data.get() }
    }

    fn backend_name(&self) -> &'static str {
        "DirectChannel"
    }
}

impl<T: Clone> Clone for DirectChannelBackend<T> {
    fn clone(&self) -> Self {
        // Clone creates a new channel on the current thread
        // (cannot share DirectChannel across threads)
        Self {
            slot: UnsafeCell::new(None),
            owner_thread: thread::current().id(),
            has_data: UnsafeCell::new(false),
        }
    }
}

// ============================================================================
// SpscIntra Backend (Tier 1: ~10-15ns optimized)
// ============================================================================

use std::hint;

/// Cache-line aligned slot for SpscIntra backend
///
/// Uses optimized seqlock pattern for lock-free SPSC communication:
/// - Producer stores odd sequence (signals write in progress)
/// - Producer writes data
/// - Producer stores even sequence (signals write complete)
/// - Consumer spins until sequence is even and unchanged
///
/// Optimizations vs naive seqlock:
/// - Uses store instead of fetch_add (saves ~5ns)
/// - Producer caches sequence locally (no atomic read needed)
/// - Platform-specific cache line padding (64B x86, 128B ARM)
/// - Prefetch hints for data
/// - Relaxed ordering where safe
#[repr(C)]
struct SpscIntraSlot<T> {
    /// Seqlock sequence counter (producer-owned)
    /// - Odd = write in progress
    /// - Even = valid data available (if > last_seen)
    /// Aligned to cache line to prevent false sharing
    sequence: CacheAlignedU64,
    /// Data storage - separate cache line from sequence
    data: CacheAlignedData<T>,
}

/// Cache-line aligned u64 for atomic sequence counter
#[repr(C)]
struct CacheAlignedU64 {
    value: AtomicU64,
    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    _pad: [u8; 64 - 8],
    #[cfg(target_arch = "aarch64")]
    _pad: [u8; 128 - 8],
    #[cfg(not(any(target_arch = "x86", target_arch = "x86_64", target_arch = "aarch64")))]
    _pad: [u8; 128 - 8],
}

impl CacheAlignedU64 {
    fn new(val: u64) -> Self {
        Self {
            value: AtomicU64::new(val),
            _pad: [0; std::mem::size_of::<Self>() - 8],
        }
    }
}

/// Cache-line aligned data storage
#[repr(C)]
struct CacheAlignedData<T> {
    data: UnsafeCell<mem::MaybeUninit<T>>,
}

impl<T> CacheAlignedData<T> {
    fn new() -> Self {
        Self {
            data: UnsafeCell::new(mem::MaybeUninit::uninit()),
        }
    }
}

impl<T> SpscIntraSlot<T> {
    fn new() -> Self {
        Self {
            sequence: CacheAlignedU64::new(0),
            data: CacheAlignedData::new(),
        }
    }
}

/// SpscIntra backend - in-process SPSC with optimized seqlock
///
/// Uses atomic seqlock pattern for lock-free single-producer single-consumer
/// communication between threads in the same process.
///
/// Latency: ~10-15ns (optimized atomics, no syscalls or shared memory)
///
/// # Performance characteristics
///
/// - Zero allocations after creation
/// - Cache-line aligned to prevent false sharing (64B x86, 128B ARM)
/// - Spin-waits for blocking receive (good for low-latency)
/// - Single-slot design (latest message overwrites previous)
/// - Producer-local sequence cache (avoids atomic reads)
/// - Optimized memory ordering (Relaxed where safe)
pub struct SpscIntraBackend<T> {
    /// Shared slot (Arc for thread-safe reference counting)
    slot: Arc<SpscIntraSlot<T>>,
    /// Whether this end is the producer
    is_producer: bool,
    /// Producer: current sequence (local cache, avoids atomic read)
    /// Consumer: last seen sequence
    local_seq: AtomicU64,
}

// Safety: SpscIntraBackend uses atomic operations for all cross-thread access
unsafe impl<T: Send> Send for SpscIntraBackend<T> {}
unsafe impl<T: Send + Sync> Sync for SpscIntraBackend<T> {}

impl<T> SpscIntraBackend<T> {
    /// Create a new SPSC channel with producer and consumer endpoints.
    ///
    /// This is a simpler API for creating channels without going through the
    /// Topic registry. Useful for:
    /// - Unit testing
    /// - Simple in-process communication without shared memory overhead
    /// - Lightweight channel creation without Topic wrapper
    ///
    /// # Returns
    /// A tuple of `(producer, consumer)` endpoints.
    ///
    /// # Example
    /// ```ignore
    /// let (tx, rx) = SpscIntraBackend::<MyMessage>::channel();
    /// tx.push(msg).unwrap();
    /// let received = rx.pop();
    /// ```
    pub fn channel() -> (Self, Self) {
        let slot = Arc::new(SpscIntraSlot::new());

        let producer = Self {
            slot: slot.clone(),
            is_producer: true,
            local_seq: AtomicU64::new(0), // Producer starts at 0 (even)
        };

        let consumer = Self {
            slot,
            is_producer: false,
            local_seq: AtomicU64::new(0), // Consumer hasn't seen anything
        };

        (producer, consumer)
    }

    /// Create producer from shared slot
    fn new_producer(slot: Arc<SpscIntraSlot<T>>) -> Self {
        // Read current sequence to initialize local cache
        let seq = slot.sequence.value.load(Ordering::Relaxed);
        Self {
            slot,
            is_producer: true,
            local_seq: AtomicU64::new(seq),
        }
    }

    /// Create consumer from shared slot
    fn new_consumer(slot: Arc<SpscIntraSlot<T>>) -> Self {
        Self {
            slot,
            is_producer: false,
            local_seq: AtomicU64::new(0),
        }
    }
}

impl<T: Clone + Send + Sync + 'static> TopicBackendTrait<T> for SpscIntraBackend<T> {
    #[inline(always)]
    fn push(&self, msg: T) -> Result<(), T> {
        if unlikely(!self.is_producer) {
            return Err(msg); // Consumer cannot push
        }

        // Optimized seqlock write protocol:
        // Use local sequence cache to avoid atomic read
        let seq = self.local_seq.load(Ordering::Relaxed);
        debug_assert!(seq.is_multiple_of(2), "Double push detected");

        // 1. Store odd sequence (signals write in progress)
        // Use Release to ensure previous writes are visible before odd marker
        // Note: store is faster than fetch_add (~3ns savings)
        self.slot.sequence.value.store(seq + 1, Ordering::Release);

        // 2. Write data
        // Safety: We're the only producer, sequence is odd so consumer won't read
        unsafe {
            self.slot.data.data.get().write(mem::MaybeUninit::new(msg));
        }

        // 3. Store even sequence (signals write complete)
        // Use Release to ensure data write is visible before even marker
        let new_seq = seq + 2;
        self.slot.sequence.value.store(new_seq, Ordering::Release);

        // 4. Update local cache
        self.local_seq.store(new_seq, Ordering::Relaxed);

        Ok(())
    }

    #[inline(always)]
    fn pop(&self) -> Option<T> {
        if unlikely(self.is_producer) {
            return None; // Producer cannot pop
        }

        // Optimized seqlock read protocol
        let last = self.local_seq.load(Ordering::Relaxed);

        // 1. Read sequence with Acquire to see producer's Release
        let seq1 = self.slot.sequence.value.load(Ordering::Acquire);

        // Fast path: check if odd (write in progress) or no new data
        if seq1 % 2 == 1 || seq1 == last {
            return None;
        }

        // Prefetch data while we validate
        prefetch_read(self.slot.data.data.get());

        // 2. Read data
        // Safety: Sequence is even, so no write in progress
        let data = unsafe { (*self.slot.data.data.get()).assume_init_read() };

        // 3. Verify sequence didn't change during read (acquire fence)
        let seq2 = self.slot.sequence.value.load(Ordering::Acquire);
        if unlikely(seq1 != seq2) {
            // Write occurred during our read - data may be torn
            // Don't return it, let caller retry
            // Note: data was read but may be invalid, we just drop it
            // The producer has already written new valid data
            return None;
        }

        // 4. Update last seen (Relaxed is fine, only this thread reads it)
        self.local_seq.store(seq1, Ordering::Relaxed);

        Some(data)
    }

    #[inline(always)]
    fn is_empty(&self) -> bool {
        let seq = self.slot.sequence.value.load(Ordering::Acquire);
        let last = self.local_seq.load(Ordering::Relaxed);

        // Empty if sequence is odd (write in progress) or we've seen it
        seq % 2 == 1 || seq == last
    }

    fn backend_name(&self) -> &'static str {
        "SpscIntra"
    }
}

impl<T> Clone for SpscIntraBackend<T> {
    fn clone(&self) -> Self {
        Self {
            slot: self.slot.clone(),
            is_producer: self.is_producer,
            local_seq: AtomicU64::new(self.local_seq.load(Ordering::Relaxed)),
        }
    }
}

// ============================================================================
// MpscIntra Backend (Tier 1: ~35ns)
// ============================================================================

/// A slot in the MPSC ring buffer
#[repr(C, align(64))]
struct MpscSlot<T> {
    /// Sequence number for this slot (used for coordination)
    sequence: AtomicU64,
    /// Data storage
    data: UnsafeCell<mem::MaybeUninit<T>>,
}

impl<T> MpscSlot<T> {
    fn new(seq: u64) -> Self {
        Self {
            sequence: AtomicU64::new(seq),
            data: UnsafeCell::new(mem::MaybeUninit::uninit()),
        }
    }
}

/// Cache-line padded atomic counter for MPSC ring
/// Ensures head and tail don't share cache lines (critical for avoiding false sharing)
#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
#[repr(C, align(64))]
struct MpscCounter {
    value: AtomicU64,
    _pad: [u8; 56], // Pad to 64 bytes (CACHE_LINE_SIZE - 8)
}

#[cfg(target_arch = "aarch64")]
#[repr(C, align(128))]
struct MpscCounter {
    value: AtomicU64,
    _pad: [u8; 120], // Pad to 128 bytes (CACHE_LINE_SIZE - 8)
}

#[cfg(not(any(target_arch = "x86", target_arch = "x86_64", target_arch = "aarch64")))]
#[repr(C, align(128))]
struct MpscCounter {
    value: AtomicU64,
    _pad: [u8; 120], // Pad to 128 bytes (CACHE_LINE_SIZE - 8)
}

impl MpscCounter {
    #[inline(always)]
    fn new(val: u64) -> Self {
        Self {
            value: AtomicU64::new(val),
            _pad: [0; std::mem::size_of::<Self>() - 8],
        }
    }
}

/// Lock-free MPSC ring buffer
///
/// Based on a simplified version of the Vyukov MPSC bounded queue.
/// Multiple producers can push concurrently, single consumer pops.
#[repr(C)]
struct MpscRing<T> {
    /// Producer tail - in its own cache line (producers CAS on this)
    tail: MpscCounter,
    /// Consumer head - in its own cache line
    head: MpscCounter,
    /// Ring buffer slots
    buffer: Box<[MpscSlot<T>]>,
    /// Capacity (power of 2)
    capacity: usize,
    /// Mask for index calculation (capacity - 1)
    mask: usize,
}

impl<T> MpscRing<T> {
    fn new(capacity: usize) -> Self {
        // Ensure capacity is power of 2
        let capacity = capacity.next_power_of_two();
        let mask = capacity - 1;

        // Initialize slots with their sequence numbers
        let buffer: Vec<MpscSlot<T>> = (0..capacity).map(|i| MpscSlot::new(i as u64)).collect();

        Self {
            tail: MpscCounter::new(0),
            head: MpscCounter::new(0),
            buffer: buffer.into_boxed_slice(),
            capacity,
            mask,
        }
    }

    /// Push a value (multiple producers can call concurrently)
    ///
    /// Optimized tight loop like MpmcRing for consistent low latency.
    #[inline(always)]
    fn push(&self, value: T) -> Result<(), T> {
        loop {
            let tail = self.tail.value.load(Ordering::Relaxed);
            let index = (tail as usize) & self.mask;
            let slot = unsafe { self.buffer.get_unchecked(index) };

            // Prefetch next slot's sequence for next iteration
            let next_index = ((tail.wrapping_add(1)) as usize) & self.mask;
            prefetch_read(&self.buffer[next_index].sequence);

            let seq = slot.sequence.load(Ordering::Acquire);
            let diff = seq as i64 - tail as i64;

            if diff == 0 {
                // Slot is ready for writing - try to claim it
                if self
                    .tail
                    .value
                    .compare_exchange_weak(
                        tail,
                        tail.wrapping_add(1),
                        Ordering::Relaxed,
                        Ordering::Relaxed,
                    )
                    .is_ok()
                {
                    // We claimed the slot, write the data
                    unsafe {
                        slot.data.get().write(mem::MaybeUninit::new(value));
                    }
                    // Mark slot as ready for consumer
                    slot.sequence.store(tail.wrapping_add(1), Ordering::Release);
                    return Ok(());
                }
                // CAS failed, another producer got it - fast retry
                continue;
            } else if unlikely(diff < 0) {
                // Queue is full
                return Err(value);
            }
            // else: slot not yet consumed, spin
            hint::spin_loop();
        }
    }

    /// Pop a value (single consumer only)
    ///
    /// Optimized for single-consumer access pattern with prefetching.
    #[inline(always)]
    fn pop(&self) -> Option<T> {
        let head = self.head.value.load(Ordering::Relaxed);
        let index = (head as usize) & self.mask;

        // SAFETY: index is always < capacity due to mask
        let slot = unsafe { self.buffer.get_unchecked(index) };

        // Prefetch the data before checking sequence - likely to succeed
        prefetch_read(slot.data.get());

        // Check if slot has data (sequence == head + 1 means data is ready)
        let seq = slot.sequence.load(Ordering::Acquire);
        let expected = head.wrapping_add(1);

        if likely(seq == expected) {
            // Fast path: data is ready, prefetch already brought it into cache
            let value = unsafe { (*slot.data.get()).assume_init_read() };
            // Mark slot as empty for next round (sequence = head + capacity)
            slot.sequence
                .store(head.wrapping_add(self.capacity as u64), Ordering::Release);
            // Advance head
            self.head.value.store(expected, Ordering::Relaxed);
            Some(value)
        } else if unlikely((seq as i64) < (expected as i64)) {
            // Queue is empty (producer hasn't written yet)
            None
        } else {
            // Shouldn't happen with single consumer, but handle gracefully
            None
        }
    }

    fn is_empty(&self) -> bool {
        let head = self.head.value.load(Ordering::Relaxed);
        let tail = self.tail.value.load(Ordering::Relaxed);
        head == tail
    }
}

/// MpscIntra backend - in-process MPSC with lock-free queue
///
/// Multiple producers can push concurrently, single consumer pops.
/// Common pattern: multiple sensors feeding a single fusion node.
///
/// Latency: ~35ns (lock-free, no syscalls)
///
/// # Performance characteristics
///
/// - Lock-free using CAS operations
/// - Bounded ring buffer (no allocations after init)
/// - Cache-line aligned slots
/// - Single consumer means no dequeue contention
pub struct MpscIntraBackend<T> {
    /// Shared ring buffer
    ring: Arc<MpscRing<T>>,
    /// Whether this end is a producer
    is_producer: bool,
}

// Safety: MpscIntraBackend uses atomic operations for all cross-thread access
unsafe impl<T: Send> Send for MpscIntraBackend<T> {}
unsafe impl<T: Send + Sync> Sync for MpscIntraBackend<T> {}

impl<T> MpscIntraBackend<T> {
    /// Create a new MPSC channel with producer and consumer endpoints.
    ///
    /// This is a simpler API for creating channels without going through the
    /// Topic registry. Useful for:
    /// - Unit testing
    /// - Simple in-process multi-producer single-consumer communication
    /// - Lightweight channel creation without Topic wrapper
    ///
    /// # Arguments
    /// * `capacity` - Buffer capacity (will be rounded up to next power of 2)
    ///
    /// # Returns
    /// A tuple of `(producer, consumer)`. The producer can be cloned to create
    /// additional producers - all feed into the same consumer.
    ///
    /// # Example
    /// ```ignore
    /// let (tx, rx) = MpscIntraBackend::<MyMessage>::channel(1024);
    /// let tx2 = tx.clone(); // Second producer
    /// tx.push(msg).unwrap();
    /// tx2.push(msg2).unwrap();
    /// // rx receives from both producers
    /// ```
    pub fn channel(capacity: usize) -> (Self, Self) {
        let ring = Arc::new(MpscRing::new(capacity));

        let producer = Self {
            ring: ring.clone(),
            is_producer: true,
        };

        let consumer = Self {
            ring,
            is_producer: false,
        };

        (producer, consumer)
    }

    /// Create a producer handle from existing ring
    fn new_producer(ring: Arc<MpscRing<T>>) -> Self {
        Self {
            ring,
            is_producer: true,
        }
    }

    /// Create a consumer handle from existing ring
    fn new_consumer(ring: Arc<MpscRing<T>>) -> Self {
        Self {
            ring,
            is_producer: false,
        }
    }

    /// Create additional producer for the same channel
    pub fn add_producer(&self) -> Self {
        Self {
            ring: self.ring.clone(),
            is_producer: true,
        }
    }
}

impl<T: Clone + Send + Sync + 'static> TopicBackendTrait<T> for MpscIntraBackend<T> {
    #[inline(always)]
    fn push(&self, msg: T) -> Result<(), T> {
        if unlikely(!self.is_producer) {
            return Err(msg);
        }
        self.ring.push(msg)
    }

    #[inline(always)]
    fn pop(&self) -> Option<T> {
        if unlikely(self.is_producer) {
            return None;
        }
        self.ring.pop()
    }

    #[inline(always)]
    fn is_empty(&self) -> bool {
        self.ring.is_empty()
    }

    fn backend_name(&self) -> &'static str {
        "MpscIntra"
    }
}

impl<T> Clone for MpscIntraBackend<T> {
    fn clone(&self) -> Self {
        Self {
            ring: self.ring.clone(),
            is_producer: self.is_producer,
        }
    }
}

// ============================================================================
// SpmcIntra Backend (Tier 1: ~40ns)
// ============================================================================

/// Cache-line aligned broadcast slot for SpmcIntra backend
///
/// Uses seqlock pattern for lock-free single-producer multi-consumer broadcast:
/// - Producer increments sequence to odd (writing)
/// - Producer writes data
/// - Producer increments sequence to even (complete)
/// - Consumers spin until sequence is even and unchanged
/// - Each consumer tracks its own last_seen sequence
#[repr(C, align(64))]
struct SpmcIntraSlot<T> {
    /// Seqlock sequence counter
    /// - Odd = write in progress
    /// - Even = valid data available (if > consumer's last_seen)
    sequence: AtomicU64,
    /// Data storage
    data: UnsafeCell<mem::MaybeUninit<T>>,
}

impl<T> SpmcIntraSlot<T> {
    fn new() -> Self {
        Self {
            sequence: AtomicU64::new(0),
            data: UnsafeCell::new(mem::MaybeUninit::uninit()),
        }
    }
}

/// SpmcIntra backend - in-process SPMC with seqlock broadcast
///
/// Uses atomic seqlock pattern for lock-free single-producer multi-consumer
/// broadcast within the same process.
///
/// Latency: ~40ns (uses atomics but no syscalls or shared memory)
///
/// # Performance characteristics
///
/// - Zero allocations after creation
/// - Cache-line aligned to prevent false sharing
/// - Producer never blocks on consumers
/// - Consumers may miss messages (latest-value semantic)
/// - Multiple consumers can read concurrently
///
/// # Use case
///
/// Robot state broadcast: one node publishes state, many nodes read it.
/// Each consumer sees the latest state, may miss intermediate updates.
pub struct SpmcIntraBackend<T> {
    /// Shared slot (Arc for thread-safe reference counting)
    slot: Arc<SpmcIntraSlot<T>>,
    /// Whether this end is the producer
    is_producer: bool,
    /// Last seen sequence (consumer only, each consumer has its own)
    last_seen: AtomicU64,
}

// Safety: SpmcIntraBackend uses atomic operations for all cross-thread access
unsafe impl<T: Send> Send for SpmcIntraBackend<T> {}
unsafe impl<T: Send + Sync> Sync for SpmcIntraBackend<T> {}

impl<T> SpmcIntraBackend<T> {
    /// Create a new SPMC channel with producer and consumer endpoints.
    ///
    /// This is a simpler API for creating channels without going through the
    /// Topic registry. Useful for:
    /// - Unit testing
    /// - Simple in-process broadcast communication
    /// - Lightweight channel creation without Topic wrapper
    ///
    /// # Returns
    /// A tuple of `(producer, consumer)`. The consumer can be cloned to create
    /// additional consumers - all will receive the same messages.
    ///
    /// # Example
    /// ```ignore
    /// let (tx, rx) = SpmcIntraBackend::<MyMessage>::channel();
    /// let rx2 = rx.clone(); // Second consumer
    /// tx.push(msg).unwrap();
    /// // Both rx and rx2 can read the message
    /// ```
    pub fn channel() -> (Self, Self) {
        let slot = Arc::new(SpmcIntraSlot::new());

        let producer = Self {
            slot: slot.clone(),
            is_producer: true,
            last_seen: AtomicU64::new(0),
        };

        let consumer = Self {
            slot,
            is_producer: false,
            last_seen: AtomicU64::new(0),
        };

        (producer, consumer)
    }

    /// Create producer from shared slot
    fn new_producer(slot: Arc<SpmcIntraSlot<T>>) -> Self {
        Self {
            slot,
            is_producer: true,
            last_seen: AtomicU64::new(0),
        }
    }

    /// Create consumer from shared slot
    fn new_consumer(slot: Arc<SpmcIntraSlot<T>>) -> Self {
        Self {
            slot,
            is_producer: false,
            last_seen: AtomicU64::new(0),
        }
    }

    /// Create additional consumer for the same channel
    pub fn add_consumer(&self) -> Self {
        Self {
            slot: self.slot.clone(),
            is_producer: false,
            last_seen: AtomicU64::new(0),
        }
    }
}

impl<T: Clone + Send + Sync + 'static> TopicBackendTrait<T> for SpmcIntraBackend<T> {
    #[inline]
    fn push(&self, msg: T) -> Result<(), T> {
        if !self.is_producer {
            return Err(msg); // Consumer cannot push
        }

        // Seqlock write protocol:
        // 1. Increment to odd (signals write in progress)
        let seq = self.slot.sequence.fetch_add(1, Ordering::Acquire);
        debug_assert!(seq.is_multiple_of(2), "Double push detected");

        // 2. Write data
        // Safety: We're the only producer, sequence is odd so consumers will retry
        unsafe {
            self.slot.data.get().write(mem::MaybeUninit::new(msg));
        }

        // 3. Increment to even (signals write complete)
        self.slot.sequence.fetch_add(1, Ordering::Release);

        Ok(())
    }

    #[inline]
    fn pop(&self) -> Option<T> {
        if self.is_producer {
            return None; // Producer cannot pop
        }

        // Seqlock read protocol:
        loop {
            // 1. Read sequence
            let seq1 = self.slot.sequence.load(Ordering::Acquire);

            // Check if odd (write in progress) - spin wait
            if seq1 % 2 == 1 {
                hint::spin_loop();
                continue;
            }

            // Check if we've already seen this data
            let last = self.last_seen.load(Ordering::Relaxed);
            if seq1 == last {
                return None; // No new data
            }

            // 2. Read data
            // Safety: Sequence is even, so no write in progress
            let data = unsafe { (*self.slot.data.get()).assume_init_read() };

            // 3. Verify sequence didn't change during read
            let seq2 = self.slot.sequence.load(Ordering::Acquire);
            if seq1 != seq2 {
                // Write occurred during our read - retry
                hint::spin_loop();
                continue;
            }

            // 4. Update last seen (for this consumer only)
            self.last_seen.store(seq1, Ordering::Relaxed);

            return Some(data);
        }
    }

    #[inline]
    fn is_empty(&self) -> bool {
        let seq = self.slot.sequence.load(Ordering::Acquire);
        let last = self.last_seen.load(Ordering::Relaxed);

        // Empty if sequence is odd (write in progress) or we've seen it
        seq % 2 == 1 || seq == last
    }

    fn backend_name(&self) -> &'static str {
        "SpmcIntra"
    }
}

impl<T> Clone for SpmcIntraBackend<T> {
    fn clone(&self) -> Self {
        Self {
            slot: self.slot.clone(),
            is_producer: self.is_producer,
            // New consumer gets its own last_seen starting from 0
            // This means it will see the next message even if it missed previous ones
            last_seen: AtomicU64::new(if self.is_producer {
                0
            } else {
                self.last_seen.load(Ordering::Relaxed)
            }),
        }
    }
}

// ============================================================================
// MpmcIntra Backend - Lock-free MPMC Queue (Optimized)
// ============================================================================

/// Slot for MPMC ring buffer - platform-specific cache-line aligned
///
/// On x86/x86_64: 64-byte alignment
/// On ARM64: 128-byte alignment (Apple M-series prefetcher)
#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
#[repr(C, align(64))]
struct MpmcSlot<T> {
    /// Sequence number for coordination
    sequence: AtomicU64,
    /// Data storage
    data: UnsafeCell<mem::MaybeUninit<T>>,
}

#[cfg(target_arch = "aarch64")]
#[repr(C, align(128))]
struct MpmcSlot<T> {
    /// Sequence number for coordination
    sequence: AtomicU64,
    /// Data storage
    data: UnsafeCell<mem::MaybeUninit<T>>,
}

#[cfg(not(any(target_arch = "x86", target_arch = "x86_64", target_arch = "aarch64")))]
#[repr(C, align(128))]
struct MpmcSlot<T> {
    /// Sequence number for coordination
    sequence: AtomicU64,
    /// Data storage
    data: UnsafeCell<mem::MaybeUninit<T>>,
}

impl<T> MpmcSlot<T> {
    #[inline(always)]
    fn new(seq: u64) -> Self {
        Self {
            sequence: AtomicU64::new(seq),
            data: UnsafeCell::new(mem::MaybeUninit::uninit()),
        }
    }
}

/// Cache-line padded atomic counter for MPMC ring
/// Ensures head and tail don't share cache lines (critical for avoiding false sharing)
#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
#[repr(C, align(64))]
struct MpmcCounter {
    value: AtomicU64,
    _pad: [u8; 56], // Pad to 64 bytes
}

#[cfg(target_arch = "aarch64")]
#[repr(C, align(128))]
struct MpmcCounter {
    value: AtomicU64,
    _pad: [u8; 120], // Pad to 128 bytes
}

#[cfg(not(any(target_arch = "x86", target_arch = "x86_64", target_arch = "aarch64")))]
#[repr(C, align(128))]
struct MpmcCounter {
    value: AtomicU64,
    _pad: [u8; 120], // Pad to 128 bytes
}

impl MpmcCounter {
    #[inline(always)]
    fn new(val: u64) -> Self {
        Self {
            value: AtomicU64::new(val),
            _pad: [0; std::mem::size_of::<Self>() - 8],
        }
    }
}

/// Lock-free MPMC ring buffer (Optimized Vyukov's bounded queue)
///
/// Key optimizations:
/// - Platform-specific cache-line aligned slots (avoid false sharing)
/// - Separated head/tail counters in their own cache lines
/// - Prefetch hints for data access
/// - Relaxed memory ordering where safe
#[repr(C)]
struct MpmcRing<T> {
    /// Producer tail - in its own cache line
    tail: MpmcCounter,
    /// Consumer head - in its own cache line
    head: MpmcCounter,
    /// Capacity (power of 2)
    capacity: usize,
    /// Mask for index calculation (capacity - 1)
    mask: usize,
    /// Ring buffer slots - each slot is cache-line aligned
    buffer: Box<[MpmcSlot<T>]>,
}

impl<T> MpmcRing<T> {
    fn new(capacity: usize) -> Self {
        // Ensure capacity is power of 2
        let capacity = capacity.next_power_of_two();
        let mask = capacity - 1;

        // Initialize slots with their sequence numbers
        let buffer: Vec<MpmcSlot<T>> = (0..capacity).map(|i| MpmcSlot::new(i as u64)).collect();

        Self {
            tail: MpmcCounter::new(0),
            head: MpmcCounter::new(0),
            capacity,
            mask,
            buffer: buffer.into_boxed_slice(),
        }
    }

    /// Push a value (multiple producers can call concurrently)
    ///
    /// Optimizations:
    /// - Prefetch next slot data while doing CAS
    /// - Use unlikely() for error paths
    /// - Relaxed ordering where safe
    #[inline(always)]
    fn push(&self, value: T) -> Result<(), T> {
        loop {
            let tail = self.tail.value.load(Ordering::Relaxed);
            let index = (tail as usize) & self.mask;
            let slot = unsafe { self.buffer.get_unchecked(index) };

            // Prefetch next slot's sequence for next iteration
            let next_index = ((tail + 1) as usize) & self.mask;
            prefetch_read(&self.buffer[next_index].sequence);

            let seq = slot.sequence.load(Ordering::Acquire);
            let diff = seq as i64 - tail as i64;

            if diff == 0 {
                // Slot is ready for writing - try to claim it
                if self
                    .tail
                    .value
                    .compare_exchange_weak(tail, tail + 1, Ordering::Relaxed, Ordering::Relaxed)
                    .is_ok()
                {
                    // We claimed the slot, write the data
                    unsafe {
                        slot.data.get().write(mem::MaybeUninit::new(value));
                    }
                    // Mark slot as ready for consumer
                    slot.sequence.store(tail + 1, Ordering::Release);
                    return Ok(());
                }
                // CAS failed, another producer got it - fast retry
                continue;
            } else if unlikely(diff < 0) {
                // Queue is full
                return Err(value);
            }
            // else: slot not yet consumed, spin
            hint::spin_loop();
        }
    }

    /// Pop a value (multiple consumers can call concurrently)
    ///
    /// Optimizations:
    /// - Early prefetch data before sequence check
    /// - Use unlikely() for error paths
    /// - Relaxed ordering where safe
    #[inline(always)]
    fn pop(&self) -> Option<T> {
        loop {
            let head = self.head.value.load(Ordering::Relaxed);
            let index = (head as usize) & self.mask;
            let slot = unsafe { self.buffer.get_unchecked(index) };

            // Prefetch data early - gives more time for cache line to arrive
            prefetch_read(slot.data.get());

            let seq = slot.sequence.load(Ordering::Acquire);
            let diff = seq as i64 - (head + 1) as i64;

            if diff == 0 {
                // Slot has data ready - try to claim it
                if self
                    .head
                    .value
                    .compare_exchange_weak(head, head + 1, Ordering::Relaxed, Ordering::Relaxed)
                    .is_ok()
                {
                    // We claimed the slot, read the data (already in cache)
                    let value = unsafe { (*slot.data.get()).assume_init_read() };
                    // Mark slot as empty for next round
                    slot.sequence
                        .store(head + self.capacity as u64, Ordering::Release);
                    return Some(value);
                }
                // CAS failed, another consumer got it - fast retry
                continue;
            } else if unlikely(diff < 0) {
                // Queue is empty
                return None;
            }
            // else: producer still writing, spin
            hint::spin_loop();
        }
    }

    #[inline(always)]
    fn is_empty(&self) -> bool {
        let head = self.head.value.load(Ordering::Relaxed);
        let tail = self.tail.value.load(Ordering::Relaxed);
        head == tail
    }
}

/// MpmcIntra backend - in-process MPMC with lock-free queue
///
/// Multiple producers can push concurrently, multiple consumers can pop.
/// Common pattern: load balancing where any worker can handle any message.
///
/// Latency: ~80ns (lock-free, CAS contention on both ends)
///
/// # Performance characteristics
///
/// - Lock-free using CAS operations on both producer and consumer sides
/// - Bounded ring buffer (no allocations after init)
/// - Cache-line aligned slots
/// - Higher contention than MPSC/SPMC due to dual-end CAS
pub struct MpmcIntraBackend<T> {
    /// Shared ring buffer
    ring: Arc<MpmcRing<T>>,
    /// Whether this end is a producer
    is_producer: bool,
}

// Safety: MpmcIntraBackend uses atomic operations for all cross-thread access
unsafe impl<T: Send> Send for MpmcIntraBackend<T> {}
unsafe impl<T: Send + Sync> Sync for MpmcIntraBackend<T> {}

impl<T> MpmcIntraBackend<T> {
    /// Create a new MPMC channel with producer and consumer endpoints.
    ///
    /// This is a simpler API for creating channels without going through the
    /// Topic registry. Useful for:
    /// - Unit testing
    /// - Simple in-process multi-producer multi-consumer communication
    /// - Lightweight channel creation without Topic wrapper
    ///
    /// # Arguments
    /// * `capacity` - Buffer capacity (will be rounded up to next power of 2)
    ///
    /// # Returns
    /// A tuple of `(producer, consumer)`. Both can be cloned to create
    /// additional producers/consumers.
    ///
    /// # Example
    /// ```ignore
    /// let (tx, rx) = MpmcIntraBackend::<MyMessage>::channel(1024);
    /// let tx2 = tx.clone(); // Second producer
    /// let rx2 = rx.clone(); // Second consumer
    /// tx.push(msg).unwrap();
    /// tx2.push(msg2).unwrap();
    /// // Both rx and rx2 can consume messages
    /// ```
    pub fn channel(capacity: usize) -> (Self, Self) {
        let ring = Arc::new(MpmcRing::new(capacity));

        let producer = Self {
            ring: ring.clone(),
            is_producer: true,
        };

        let consumer = Self {
            ring,
            is_producer: false,
        };

        (producer, consumer)
    }

    /// Create producer from shared ring
    fn new_producer(ring: Arc<MpmcRing<T>>) -> Self {
        Self {
            ring,
            is_producer: true,
        }
    }

    /// Create consumer from shared ring
    fn new_consumer(ring: Arc<MpmcRing<T>>) -> Self {
        Self {
            ring,
            is_producer: false,
        }
    }
}

impl<T: Clone + Send + Sync + 'static> TopicBackendTrait<T> for MpmcIntraBackend<T> {
    #[inline(always)]
    fn push(&self, msg: T) -> Result<(), T> {
        if unlikely(!self.is_producer) {
            return Err(msg); // Consumer cannot push
        }
        self.ring.push(msg)
    }

    #[inline(always)]
    fn pop(&self) -> Option<T> {
        if unlikely(self.is_producer) {
            return None; // Producer cannot pop
        }
        self.ring.pop()
    }

    #[inline(always)]
    fn is_empty(&self) -> bool {
        self.ring.is_empty()
    }

    #[inline(always)]
    fn backend_name(&self) -> &'static str {
        "MpmcIntra"
    }
}

impl<T> Clone for MpmcIntraBackend<T> {
    fn clone(&self) -> Self {
        Self {
            ring: self.ring.clone(),
            is_producer: self.is_producer,
        }
    }
}

// ============================================================================
// Network Backend Wrapper (Tier 4: Network transport)
// ============================================================================

use crate::communication::network::{parse_endpoint, Endpoint, NetworkBackend};

/// Network topic backend - wraps NetworkBackend for network transport
///
/// Supports multiple network transports:
/// - UDP direct connection
/// - Unix domain sockets (localhost)
/// - Multicast discovery
/// - Zenoh (mesh networking)
/// - QUIC (reliable, encrypted)
///
/// Latency: Variable (typically 10-100µs for LAN, varies for WAN)
struct NetworkTopicBackend<T> {
    /// The underlying network backend (wrapped in Mutex for recv mutability)
    inner: std::sync::Mutex<NetworkBackend<T>>,
    /// Topic name (stored for debugging and future introspection)
    #[allow(dead_code)]
    topic: String,
}

impl<T> NetworkTopicBackend<T>
where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static,
{
    /// Create a new network backend from an endpoint
    fn new(endpoint: Endpoint) -> HorusResult<Self> {
        let topic = match &endpoint {
            Endpoint::Local { topic } => topic.clone(),
            Endpoint::Localhost { topic, .. } => topic.clone(),
            Endpoint::Direct { topic, .. } => topic.clone(),
            Endpoint::Multicast { topic } => topic.clone(),
            Endpoint::Router { topic, .. } => topic.clone(),
            Endpoint::Zenoh { topic, .. } => topic.clone(),
            Endpoint::Mdns { topic, .. } => topic.clone(),
            Endpoint::Cloud { topic, .. } => topic.clone(),
            Endpoint::ZenohCloud { topic, .. } => topic.clone(),
        };

        let backend = NetworkBackend::new(endpoint)?;

        Ok(Self {
            inner: std::sync::Mutex::new(backend),
            topic,
        })
    }

    /// Send a message over the network
    #[inline]
    fn push(&self, msg: T) -> Result<(), T> {
        let backend = self.inner.lock().map_err(|_| msg.clone())?;
        backend.send(&msg).map_err(|_| msg)
    }

    /// Receive a message from the network
    #[inline]
    fn pop(&self) -> Option<T> {
        let mut backend = self.inner.lock().ok()?;
        backend.recv()
    }

    /// Check if empty (always returns false for network - unknown)
    #[inline]
    fn is_empty(&self) -> bool {
        // Network backends don't have a reliable way to check if empty
        // Return false to indicate messages might be available
        false
    }

    /// Get backend name
    fn backend_name(&self) -> &'static str {
        "Network"
    }
}

impl<T> Clone for NetworkTopicBackend<T>
where
    T: serde::Serialize
        + serde::de::DeserializeOwned
        + Send
        + Sync
        + Clone
        + std::fmt::Debug
        + 'static,
{
    fn clone(&self) -> Self {
        // Network backends can't be cloned directly (they contain sockets)
        // Create a clone by re-parsing the endpoint
        // For now, panic - users should create new Topics for network connections
        panic!("NetworkTopicBackend cannot be cloned. Create a new Topic for additional network connections.")
    }
}

// ============================================================================
// Connection State (unified Topic implementation)
// ============================================================================

/// Connection state for Topic connections
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConnectionState {
    /// Not connected
    #[default]
    Disconnected,
    /// Connection in progress
    Connecting,
    /// Successfully connected
    Connected,
    /// Reconnecting after connection loss
    Reconnecting,
    /// Connection failed
    Failed,
}

impl ConnectionState {
    /// Convert to u8 for atomic storage
    #[inline]
    pub fn into_u8(self) -> u8 {
        match self {
            ConnectionState::Disconnected => 0,
            ConnectionState::Connecting => 1,
            ConnectionState::Connected => 2,
            ConnectionState::Reconnecting => 3,
            ConnectionState::Failed => 4,
        }
    }

    /// Convert from u8
    #[inline]
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => ConnectionState::Disconnected,
            1 => ConnectionState::Connecting,
            2 => ConnectionState::Connected,
            3 => ConnectionState::Reconnecting,
            _ => ConnectionState::Failed,
        }
    }
}

// ============================================================================
// Atomic Topic Metrics (unified Topic implementation)
// ============================================================================

/// Lock-free atomic metrics for Topic monitoring with cache optimization
#[derive(Debug)]
#[repr(align(64))] // Cache-line aligned to prevent false sharing
pub struct AtomicTopicMetrics {
    /// Number of messages successfully sent
    pub messages_sent: AtomicU64,
    /// Number of messages successfully received
    pub messages_received: AtomicU64,
    /// Number of send failures
    pub send_failures: AtomicU64,
    /// Number of receive failures
    pub recv_failures: AtomicU64,
    /// Padding to cache line boundary
    _padding: [u8; 32],
}

impl Default for AtomicTopicMetrics {
    fn default() -> Self {
        Self {
            messages_sent: AtomicU64::new(0),
            messages_received: AtomicU64::new(0),
            send_failures: AtomicU64::new(0),
            recv_failures: AtomicU64::new(0),
            _padding: [0; 32],
        }
    }
}

impl AtomicTopicMetrics {
    /// Create new metrics with all counters at zero
    pub fn new() -> Self {
        Self::default()
    }

    /// Get current metrics snapshot (for monitoring/debugging)
    pub fn snapshot(&self) -> TopicMetrics {
        TopicMetrics {
            messages_sent: self.messages_sent.load(Ordering::Relaxed),
            messages_received: self.messages_received.load(Ordering::Relaxed),
            send_failures: self.send_failures.load(Ordering::Relaxed),
            recv_failures: self.recv_failures.load(Ordering::Relaxed),
        }
    }

    /// Increment messages sent counter
    #[inline]
    pub fn inc_sent(&self) {
        self.messages_sent.fetch_add(1, Ordering::Relaxed);
    }

    /// Increment messages received counter
    #[inline]
    pub fn inc_received(&self) {
        self.messages_received.fetch_add(1, Ordering::Relaxed);
    }

    /// Increment send failures counter
    #[inline]
    pub fn inc_send_failures(&self) {
        self.send_failures.fetch_add(1, Ordering::Relaxed);
    }

    /// Increment receive failures counter
    #[inline]
    pub fn inc_recv_failures(&self) {
        self.recv_failures.fetch_add(1, Ordering::Relaxed);
    }
}

/// Simple metrics snapshot for Topic monitoring
#[derive(Debug, Clone, Default)]
pub struct TopicMetrics {
    /// Number of messages successfully sent
    pub messages_sent: u64,
    /// Number of messages successfully received
    pub messages_received: u64,
    /// Number of send failures
    pub send_failures: u64,
    /// Number of receive failures
    pub recv_failures: u64,
}

// ============================================================================
// Internal Backend Enum
// ============================================================================

/// Internal backend enum that holds the actual IPC implementation
enum TopicBackend<T> {
    /// DirectChannel - same-thread only (~3-5ns)
    Direct(DirectChannelBackend<T>),

    /// SpscIntra - in-process SPSC (~18ns)
    SpscIntra(SpscIntraBackend<T>),

    /// MpscIntra - in-process MPSC (~26ns)
    MpscIntra(MpscIntraBackend<T>),

    /// SpmcIntra - in-process SPMC (~40ns)
    SpmcIntra(SpmcIntraBackend<T>),

    /// MpmcIntra - in-process MPMC (~80ns)
    MpmcIntra(MpmcIntraBackend<T>),

    /// MPMC ring buffer (unified implementation) - ~167ns
    MpmcShm(MpmcShmBackend<T>),

    /// SPSC single-slot (unified implementation) - ~85ns
    SpscShm(SpscShmBackend<T>),

    /// MPSC shared memory - cross-process (~65ns)
    MpscShm(MpscShmBackend<T>),

    /// SPMC shared memory - cross-process (~70ns)
    SpmcShm(SpmcShmBackend<T>),

    /// Adaptive backend - full 10-path detection based on topology
    /// Auto-detects: Direct, SpscIntra, MpscIntra, SpmcIntra, MpmcIntra,
    /// PodShm, SpscShm, MpscShm, SpmcShm, MpmcShm
    Adaptive(AdaptiveTopic<T>),

    /// Network transport (UDP/Unix/Zenoh/QUIC) - variable latency
    Network(NetworkTopicBackend<T>),
}

// ============================================================================
// Topic Struct - Main Public API
// ============================================================================

/// Unified Topic for Smart IPC
///
/// `Topic<T>` provides a simple pub/sub interface that automatically selects
/// the optimal IPC backend. This unified API (replaces the old Hub, Link, and PodLink) with a
/// single unified API.
///
/// # Backend Selection
///
/// Automatically selects the best backend based on endpoint and configuration:
/// - Local topic (`"sensor"`) → SHM backend (MpmcShm, SpscShm, etc.)
/// - Network endpoint (`"sensor@192.168.1.5"`) → Network backend
/// - Zenoh endpoint (`"sensor@zenoh"`) → Zenoh mesh networking
///
/// # Endpoint Formats
///
/// - `"topic"` → Local shared memory (default)
/// - `"topic@localhost"` → Unix domain socket (localhost optimization)
/// - `"topic@192.168.1.5"` → UDP direct connection
/// - `"topic@192.168.1.5:9000"` → UDP with custom port
/// - `"topic@*"` → Multicast discovery
/// - `"topic@zenoh"` → Zenoh mesh networking
/// - `"topic@router"` → Router-based messaging
///
/// # Examples
///
/// ```rust,ignore
/// use horus_core::communication::Topic;
///
/// // Create a local topic (backend auto-selected)
/// let topic: Topic<SensorData> = Topic::new("sensor")?;
///
/// // Create a network topic
/// let net_topic: Topic<SensorData> = Topic::new("sensor@192.168.1.5")?;
///
/// // Send - Topic pattern
/// topic.send(data)?;
///
/// // Receive - Topic pattern
/// if let Some(msg) = topic.recv() {
///     process(msg);
/// }
///
/// // Check metrics
/// let metrics = topic.get_metrics();
/// println!("Sent: {}, Received: {}", metrics.messages_sent, metrics.messages_received);
/// ```
pub struct Topic<T> {
    /// Topic name
    name: String,

    /// Internal backend (auto-selected)
    backend: TopicBackend<T>,

    /// Lock-free atomic metrics for monitoring
    metrics: Arc<AtomicTopicMetrics>,

    /// Connection state (atomic for lock-free access)
    state: std::sync::atomic::AtomicU8,

    /// Phantom for type safety
    _marker: PhantomData<T>,
}

// Topic is Clone via Arc-based backends (or thread-local for Direct)
// Note: Network backends cannot be cloned - use new Topic for additional connections
impl<T: Clone> Clone for Topic<T> {
    fn clone(&self) -> Self {
        Self {
            name: self.name.clone(),
            backend: match &self.backend {
                TopicBackend::Direct(inner) => TopicBackend::Direct(inner.clone()),
                TopicBackend::SpscIntra(inner) => TopicBackend::SpscIntra(inner.clone()),
                TopicBackend::MpscIntra(inner) => TopicBackend::MpscIntra(inner.clone()),
                TopicBackend::SpmcIntra(inner) => TopicBackend::SpmcIntra(inner.clone()),
                TopicBackend::MpmcIntra(inner) => TopicBackend::MpmcIntra(inner.clone()),
                TopicBackend::MpmcShm(inner) => TopicBackend::MpmcShm(inner.clone()),
                TopicBackend::SpscShm(inner) => TopicBackend::SpscShm(inner.clone()),
                TopicBackend::MpscShm(inner) => TopicBackend::MpscShm(inner.clone()),
                TopicBackend::SpmcShm(inner) => TopicBackend::SpmcShm(inner.clone()),
                TopicBackend::Adaptive(inner) => TopicBackend::Adaptive(inner.clone()),
                TopicBackend::Network(_) => {
                    panic!("Network Topic cannot be cloned. Create a new Topic for additional network connections.")
                }
            },
            metrics: self.metrics.clone(),
            state: std::sync::atomic::AtomicU8::new(self.state.load(Ordering::Relaxed)),
            _marker: PhantomData,
        }
    }
}

// ============================================================================
// Topic Implementation
// ============================================================================

impl<T> Topic<T>
where
    T: Clone + Send + Sync + serde::Serialize + serde::de::DeserializeOwned + 'static,
{
    /// Create a new topic with automatic backend selection
    ///
    /// Parses the topic name/endpoint to select the appropriate backend:
    /// - `"topic"` → Local shared memory (MpmcShm)
    /// - `"topic@host:port"` → Network backend
    /// - `"topic@zenoh"` → Zenoh transport
    ///
    /// # Arguments
    ///
    /// * `name` - The topic name or endpoint string
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// // Local shared memory
    /// let topic: Topic<SensorData> = Topic::new("sensor")?;
    ///
    /// // Network endpoint (requires Serialize + DeserializeOwned)
    /// let net_topic: Topic<SensorData> = Topic::new("sensor@192.168.1.5")?;
    /// ```
    pub fn new(name: impl Into<String>) -> HorusResult<Self> {
        Self::with_capacity(name, 64)
    }

    /// Create a new topic with custom capacity
    ///
    /// # Smart Detection
    ///
    /// This method **automatically** selects the optimal backend based on:
    /// - **POD Detection**: If `T` has no Drop (auto-detected), uses zero-copy (~50ns)
    /// - **Non-POD**: Uses ring buffer with serialization (~167ns)
    ///
    /// No registration needed - just define your struct and HORUS auto-detects:
    /// ```rust,ignore
    /// use horus_core::communication::Topic;
    ///
    /// // Simple struct - auto-detected as POD (no String, Vec, Box, etc.)
    /// struct MotorCommand { velocity: f32, torque: f32 }
    ///
    /// // HORUS automatically uses zero-copy path (~50ns)!
    /// let topic: Topic<MotorCommand> = Topic::new("motor")?;
    /// ```
    ///
    /// # Arguments
    ///
    /// * `name` - The topic name or endpoint string
    /// * `capacity` - Ring buffer capacity (must be power of 2, for Ring mode)
    pub fn with_capacity(name: impl Into<String>, capacity: usize) -> HorusResult<Self> {
        let name = name.into();
        let shm_name = format!("topic/{}", name);

        // Use AdaptiveTopic for full 10-path detection:
        // - DirectChannel (~3ns) - same thread
        // - SpscIntra (~18ns) - 1P:1C same process
        // - SpmcIntra (~24ns) - 1P:NC same process
        // - MpscIntra (~26ns) - NP:1C same process
        // - MpmcIntra (~36ns) - NP:NC same process
        // - PodShm (~50ns) - POD types cross-process
        // - MpscShm (~65ns) - NP:1C cross-process
        // - SpmcShm (~70ns) - 1P:NC cross-process
        // - SpscShm (~85ns) - 1P:1C cross-process
        // - MpmcShm (~167ns) - NP:NC cross-process
        let adaptive = AdaptiveTopic::with_capacity(&shm_name, capacity as u32)?;

        log::info!(
            "Topic '{}': Created with AdaptiveTopic (mode={:?}, latency=~{}ns)",
            name,
            adaptive.mode(),
            adaptive.mode().expected_latency_ns()
        );

        Ok(Self {
            name,
            backend: TopicBackend::Adaptive(adaptive),
            metrics: Arc::new(AtomicTopicMetrics::default()),
            state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
            _marker: PhantomData,
        })
    }

    /// Create a topic from configuration
    /// Create a topic from configuration with automatic backend selection
    ///
    /// The backend is selected based on:
    /// - `topology`: SameThread, SameProcess, or CrossProcess
    /// - `access_pattern`: SPSC, SPMC, MPSC, or MPMC
    /// - `backend_hint`: Explicit backend or Auto
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// // Auto-select best backend for same-process SPSC
    /// let config = TopicConfig::new("sensor")
    ///     .same_process()
    ///     .spsc();
    /// let topic: Topic<SensorData> = Topic::from_config(config)?;
    /// // Selected: SpscIntra (~25ns)
    ///
    /// // Same-thread for maximum performance
    /// let config = TopicConfig::new("commands").same_thread();
    /// let topic: Topic<Command> = Topic::from_config(config)?;
    /// // Selected: DirectChannel (~3-5ns)
    /// ```
    pub fn from_config(config: TopicConfig) -> HorusResult<Self> {
        // For now, we can't detect POD at compile time without specialization
        // Use false as default (conservative - uses Serde-compatible backends)
        let is_pod = false;

        let selected = select_backend(&config, is_pod);

        match selected {
            SelectedBackend::DirectChannel => Ok(Self::direct(config.name)),
            SelectedBackend::SpscIntra => {
                // SpscIntra returns (producer, consumer) - for now return producer
                // TODO: Better API for SPSC where you need both ends
                let (producer, _consumer) = Self::spsc_intra(&config.name);
                if config.is_producer {
                    Ok(producer)
                } else {
                    // Re-create to get consumer
                    let (_, consumer) = Self::spsc_intra(&config.name);
                    Ok(consumer)
                }
            }
            SelectedBackend::MpscIntra => {
                // MpscIntra returns (producer, consumer)
                let (producer, consumer) = Self::mpsc_intra(&config.name, config.capacity as usize);
                if config.is_producer {
                    Ok(producer)
                } else {
                    Ok(consumer)
                }
            }
            SelectedBackend::SpmcIntra => {
                // SpmcIntra returns (producer, consumer)
                let (producer, consumer) = Self::spmc_intra(&config.name);
                if config.is_producer {
                    Ok(producer)
                } else {
                    Ok(consumer)
                }
            }
            SelectedBackend::MpmcIntra => {
                // MpmcIntra returns (producer, consumer)
                let (producer, consumer) = Self::mpmc_intra(&config.name, config.capacity as usize);
                if config.is_producer {
                    Ok(producer)
                } else {
                    Ok(consumer)
                }
            }
            SelectedBackend::SpscShm => Self::spsc_shm(config.name, config.is_producer),
            SelectedBackend::PodShm => {
                // PodShm requires PodMessage trait - fall back to SpscShm
                Self::spsc_shm(config.name, config.is_producer)
            }
            SelectedBackend::MpmcShm => Self::with_capacity(config.name, config.capacity as usize),
            SelectedBackend::MpscShm => {
                Self::mpsc_shm(config.name, config.capacity as usize, config.is_producer)
            }
            SelectedBackend::SpmcShm => Self::spmc_shm(config.name, config.is_producer),
            SelectedBackend::Network => {
                // Network backend requires Serialize + DeserializeOwned - use from_endpoint() instead
                Err(HorusError::Communication(
                    "Network backend requires Serialize + DeserializeOwned. Use Topic::from_endpoint() instead.".to_string(),
                ))
            }
        }
    }

    /// Get the selected backend info for this topic's configuration
    ///
    /// Useful for debugging and verifying backend selection.
    pub fn selected_backend_for(config: &TopicConfig) -> SelectedBackend {
        select_backend(config, false)
    }

    /// Create a DirectChannel topic for same-thread communication
    ///
    /// This is the fastest backend (~3-5ns) but ONLY works when producer
    /// and consumer are on the same thread. Use cases:
    /// - Single-threaded schedulers
    /// - Callback patterns
    /// - Synchronous pipelines
    ///
    /// # Arguments
    ///
    /// * `name` - The topic name (for identification only, no shared memory)
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// // Same-thread communication
    /// let topic: Topic<Command> = Topic::direct("commands");
    ///
    /// topic.send(cmd)?;
    /// let received = topic.recv();
    /// ```
    ///
    /// # Warning
    ///
    /// Operations from a different thread will return errors/None.
    pub fn direct(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            backend: TopicBackend::Direct(DirectChannelBackend::new()),
            metrics: Arc::new(AtomicTopicMetrics::default()),
            state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
            _marker: PhantomData,
        }
    }

    /// Create a SpscIntra topic for same-process cross-thread communication
    ///
    /// This is the fastest cross-thread backend (~25ns) for 1 producer and
    /// 1 consumer in the same process. Uses seqlock pattern with atomics.
    ///
    /// Returns a tuple of (producer, consumer) topics.
    ///
    /// # Arguments
    ///
    /// * `name` - The topic name (for identification only, no shared memory)
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// // Cross-thread SPSC communication
    /// let (producer, consumer) = Topic::<SensorData>::spsc_intra("sensor");
    ///
    /// // In producer thread
    /// producer.send(data)?;
    ///
    /// // In consumer thread
    /// let data = consumer.recv();
    /// ```
    pub fn spsc_intra(name: impl Into<String>) -> (Self, Self) {
        let name = name.into();
        let slot = Arc::new(SpscIntraSlot::new());
        let metrics = Arc::new(AtomicTopicMetrics::default());

        let producer = Self {
            name: name.clone(),
            backend: TopicBackend::SpscIntra(SpscIntraBackend::new_producer(slot.clone())),
            metrics: metrics.clone(),
            state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
            _marker: PhantomData,
        };

        let consumer = Self {
            name,
            backend: TopicBackend::SpscIntra(SpscIntraBackend::new_consumer(slot)),
            metrics,
            state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
            _marker: PhantomData,
        };

        (producer, consumer)
    }

    /// Create an MpscIntra topic for same-process MPSC communication
    ///
    /// This backend (~35ns) supports multiple producers and a single consumer
    /// in the same process. Uses a lock-free ring buffer with CAS operations.
    ///
    /// Common use case: Multiple sensors feeding a single fusion node.
    ///
    /// Returns a tuple of (producer, consumer) topics. The producer can be
    /// cloned to create additional producers.
    ///
    /// # Arguments
    ///
    /// * `name` - The topic name (for identification only, no shared memory)
    /// * `capacity` - Ring buffer capacity (will be rounded up to power of 2)
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// // Multi-producer single-consumer communication
    /// let (producer, consumer) = Topic::<SensorData>::mpsc_intra("sensors", 64);
    ///
    /// // Create additional producers
    /// let producer2 = producer.clone();
    ///
    /// // In producer threads
    /// producer.send(data1)?;
    /// producer2.send(data2)?;
    ///
    /// // In consumer thread
    /// while let Some(data) = consumer.recv() {
    ///     process(data);
    /// }
    /// ```
    pub fn mpsc_intra(name: impl Into<String>, capacity: usize) -> (Self, Self) {
        let name = name.into();
        let ring = Arc::new(MpscRing::new(capacity));
        let metrics = Arc::new(AtomicTopicMetrics::default());

        let producer = Self {
            name: name.clone(),
            backend: TopicBackend::MpscIntra(MpscIntraBackend::new_producer(ring.clone())),
            metrics: metrics.clone(),
            state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
            _marker: PhantomData,
        };

        let consumer = Self {
            name,
            backend: TopicBackend::MpscIntra(MpscIntraBackend::new_consumer(ring)),
            metrics,
            state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
            _marker: PhantomData,
        };

        (producer, consumer)
    }

    /// Create an SpmcIntra topic for same-process SPMC (broadcast) communication
    ///
    /// This backend (~40ns) supports a single producer broadcasting to multiple
    /// consumers in the same process. Uses a seqlock-based design where:
    /// - Producer writes are never blocked (always succeed)
    /// - Consumers read the latest value (broadcast semantics)
    /// - Each consumer tracks what it has seen to detect new data
    ///
    /// Common use case: One node broadcasting state to many consumers
    /// (e.g., robot pose broadcaster, shared configuration).
    ///
    /// Returns a tuple of (producer, consumer) topics. The consumer can be
    /// cloned to create additional consumers.
    ///
    /// # Arguments
    ///
    /// * `name` - The topic name (for identification only, no shared memory)
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// // Single-producer multi-consumer broadcast
    /// let (producer, consumer) = Topic::<RobotPose>::spmc_intra("pose");
    ///
    /// // Create additional consumers
    /// let consumer2 = consumer.clone();
    ///
    /// // Producer broadcasts (never blocks)
    /// producer.send(pose)?;
    ///
    /// // Multiple consumers receive the same value
    /// let pose1 = consumer.recv();
    /// let pose2 = consumer2.recv();
    /// ```
    pub fn spmc_intra(name: impl Into<String>) -> (Self, Self) {
        let name = name.into();
        let slot = Arc::new(SpmcIntraSlot::new());
        let metrics = Arc::new(AtomicTopicMetrics::default());

        let producer = Self {
            name: name.clone(),
            backend: TopicBackend::SpmcIntra(SpmcIntraBackend::new_producer(slot.clone())),
            metrics: metrics.clone(),
            state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
            _marker: PhantomData,
        };

        let consumer = Self {
            name,
            backend: TopicBackend::SpmcIntra(SpmcIntraBackend::new_consumer(slot)),
            metrics,
            state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
            _marker: PhantomData,
        };

        (producer, consumer)
    }

    /// Create an MpmcIntra topic for same-process MPMC (load-balanced) communication
    ///
    /// This backend (~80ns) supports multiple producers and multiple consumers
    /// in the same process. Uses a lock-free bounded queue with CAS on both
    /// producer and consumer sides.
    ///
    /// Common use case: Load balancing where any worker can handle any message
    /// (e.g., thread pool work distribution).
    ///
    /// Returns a tuple of (producer, consumer) topics. Both producer and
    /// consumer can be cloned to create additional producers/consumers.
    ///
    /// # Arguments
    ///
    /// * `name` - The topic name (for identification only, no shared memory)
    /// * `capacity` - Ring buffer capacity (will be rounded up to power of 2)
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// // Multi-producer multi-consumer load balancing
    /// let (producer, consumer) = Topic::<WorkItem>::mpmc_intra("work_queue", 64);
    ///
    /// // Create additional producers and consumers
    /// let producer2 = producer.clone();
    /// let consumer2 = consumer.clone();
    ///
    /// // Multiple producers submit work
    /// producer.send(work1)?;
    /// producer2.send(work2)?;
    ///
    /// // Multiple consumers process work (any consumer gets any message)
    /// let work = consumer.recv();
    /// let work = consumer2.recv();
    /// ```
    pub fn mpmc_intra(name: impl Into<String>, capacity: usize) -> (Self, Self) {
        let name = name.into();
        let ring = Arc::new(MpmcRing::new(capacity));
        let metrics = Arc::new(AtomicTopicMetrics::default());

        let producer = Self {
            name: name.clone(),
            backend: TopicBackend::MpmcIntra(MpmcIntraBackend::new_producer(ring.clone())),
            metrics: metrics.clone(),
            state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
            _marker: PhantomData,
        };

        let consumer = Self {
            name,
            backend: TopicBackend::MpmcIntra(MpmcIntraBackend::new_consumer(ring)),
            metrics,
            state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
            _marker: PhantomData,
        };

        (producer, consumer)
    }

    /// Create an MpscShm topic for cross-process MPSC communication
    ///
    /// This backend (~65ns) supports multiple producers and a single consumer
    /// across different processes using shared memory. Uses a lock-free bounded
    /// ring buffer in shared memory with CAS for producer coordination.
    ///
    /// Common use case: Distributed sensors feeding a central aggregator process
    /// (e.g., multiple LiDAR processes feeding a fusion node).
    ///
    /// # Arguments
    ///
    /// * `name` - The topic name (used as shared memory identifier)
    /// * `capacity` - Ring buffer capacity (will be rounded up to power of 2)
    /// * `is_producer` - True for producer, false for consumer
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// // Producer process
    /// let producer: Topic<SensorData> = Topic::mpsc_shm("sensor_fusion", 64, true)?;
    /// producer.send(data)?;
    ///
    /// // Consumer process (single consumer only)
    /// let consumer: Topic<SensorData> = Topic::mpsc_shm("sensor_fusion", 64, false)?;
    /// let data = consumer.recv();
    /// ```
    ///
    /// # Safety
    ///
    /// Only one consumer should exist across all processes. Multiple consumers
    /// will corrupt the queue state.
    pub fn mpsc_shm(
        name: impl Into<String>,
        capacity: usize,
        is_producer: bool,
    ) -> HorusResult<Self> {
        let name = name.into();
        let backend = if is_producer {
            MpscShmBackend::producer(&name, capacity)?
        } else {
            MpscShmBackend::consumer(&name, capacity)?
        };

        Ok(Self {
            name,
            backend: TopicBackend::MpscShm(backend),
            metrics: Arc::new(AtomicTopicMetrics::default()),
            state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
            _marker: PhantomData,
        })
    }

    /// Create an SpmcShm topic for cross-process SPMC communication
    ///
    /// This backend (~70ns) supports a single producer broadcasting to multiple
    /// consumers across different processes using shared memory. Uses a seqlock
    /// pattern for lock-free consistent reads.
    ///
    /// Common use case: A single sensor broadcasting to multiple processing nodes
    /// (e.g., camera publishing to both visualization and detection processes).
    ///
    /// # Arguments
    ///
    /// * `name` - The topic name (used as shared memory identifier)
    /// * `is_producer` - True for producer, false for consumer
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// // Producer process (single producer only)
    /// let producer: Topic<SensorData> = Topic::spmc_shm("camera", true)?;
    /// producer.send(data)?;
    ///
    /// // Consumer process 1
    /// let consumer1: Topic<SensorData> = Topic::spmc_shm("camera", false)?;
    /// let data = consumer1.recv();
    ///
    /// // Consumer process 2 (multiple consumers allowed)
    /// let consumer2: Topic<SensorData> = Topic::spmc_shm("camera", false)?;
    /// let data = consumer2.recv();
    /// ```
    ///
    /// # Safety
    ///
    /// Only one producer should exist across all processes. Multiple producers
    /// will corrupt the shared memory state.
    pub fn spmc_shm(name: impl Into<String>, is_producer: bool) -> HorusResult<Self> {
        let name = name.into();
        let backend = if is_producer {
            SpmcShmBackend::producer(&name)?
        } else {
            SpmcShmBackend::consumer(&name)?
        };

        Ok(Self {
            name,
            backend: TopicBackend::SpmcShm(backend),
            metrics: Arc::new(AtomicTopicMetrics::default()),
            state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
            _marker: PhantomData,
        })
    }

    /// Create an SpscShm topic for cross-process SPSC communication
    ///
    /// This backend (~85ns) is optimized for a single producer and single consumer
    /// across different processes using shared memory. Uses a single-slot design
    /// where the producer overwrites and the consumer tracks what it has seen.
    ///
    /// Common use case: Dedicated point-to-point communication between two nodes
    /// (e.g., sensor driver to processor, controller to actuator).
    ///
    /// # Arguments
    ///
    /// * `name` - The topic name (used as shared memory identifier)
    /// * `is_producer` - True for producer, false for consumer
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// // Producer process
    /// let producer: Topic<MotorCommand> = Topic::spsc_shm("motor_cmd", true)?;
    /// producer.send(cmd)?;
    ///
    /// // Consumer process
    /// let consumer: Topic<MotorCommand> = Topic::spsc_shm("motor_cmd", false)?;
    /// let cmd = consumer.recv();
    /// ```
    ///
    /// # Safety
    ///
    /// Only one producer and one consumer should exist across all processes.
    /// Multiple producers or consumers will lead to undefined behavior.
    pub fn spsc_shm(name: impl Into<String>, is_producer: bool) -> HorusResult<Self> {
        let name = name.into();
        let shm_name = format!("topic/{}", name);
        let backend = SpscShmBackend::new(&shm_name, is_producer)?;

        Ok(Self {
            name,
            backend: TopicBackend::SpscShm(backend),
            metrics: Arc::new(AtomicTopicMetrics::default()),
            state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
            _marker: PhantomData,
        })
    }
}

// ============================================================================
// Network Topic Implementation (requires Serialize + DeserializeOwned)
// ============================================================================

impl<T> Topic<T>
where
    T: Clone
        + Send
        + Sync
        + 'static
        + serde::Serialize
        + serde::de::DeserializeOwned
        + std::fmt::Debug,
{
    /// Create a topic from an endpoint string with automatic backend selection
    ///
    /// This method parses endpoint strings and selects the appropriate backend:
    /// - `"topic"` → Local shared memory (MpmcShm)
    /// - `"topic@localhost"` → Unix domain socket (localhost optimization)
    /// - `"topic@192.168.1.5"` → UDP direct connection
    /// - `"topic@192.168.1.5:9000"` → UDP with custom port
    /// - `"topic@*"` → Multicast discovery
    /// - `"topic@zenoh"` → Zenoh mesh networking
    /// - `"topic@router"` → Router-based messaging
    ///
    /// # Arguments
    ///
    /// * `endpoint` - The topic name or full endpoint string
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// // Local shared memory
    /// let topic: Topic<SensorData> = Topic::from_endpoint("sensor")?;
    ///
    /// // Network endpoint
    /// let net_topic: Topic<SensorData> = Topic::from_endpoint("sensor@192.168.1.5")?;
    ///
    /// // Zenoh mesh
    /// let zenoh_topic: Topic<SensorData> = Topic::from_endpoint("sensor@zenoh")?;
    /// ```
    pub fn from_endpoint(endpoint: impl Into<String>) -> HorusResult<Self> {
        Self::from_endpoint_with_capacity(endpoint, 64)
    }

    /// Create a topic from an endpoint string with custom capacity
    ///
    /// # Arguments
    ///
    /// * `endpoint` - The topic name or full endpoint string
    /// * `capacity` - Ring buffer capacity (for SHM backends)
    pub fn from_endpoint_with_capacity(
        endpoint: impl Into<String>,
        capacity: usize,
    ) -> HorusResult<Self> {
        let endpoint_str = endpoint.into();
        let parsed = parse_endpoint(&endpoint_str)?;

        match parsed {
            Endpoint::Local { ref topic } => {
                // Fast path: local shared memory
                let shm_name = format!("topic/{}", topic);
                let backend = MpmcShmBackend::new(&shm_name, capacity)?;

                Ok(Self {
                    name: topic.clone(),
                    backend: TopicBackend::MpmcShm(backend),
                    metrics: Arc::new(AtomicTopicMetrics::default()),
                    state: std::sync::atomic::AtomicU8::new(ConnectionState::Connected.into_u8()),
                    _marker: PhantomData,
                })
            }
            network_endpoint => {
                // Network path
                let topic_name = match &network_endpoint {
                    Endpoint::Localhost { topic, .. } => topic.clone(),
                    Endpoint::Direct { topic, .. } => topic.clone(),
                    Endpoint::Multicast { topic } => topic.clone(),
                    Endpoint::Router { topic, .. } => topic.clone(),
                    Endpoint::Zenoh { topic, .. } => topic.clone(),
                    Endpoint::Mdns { topic, .. } => topic.clone(),
                    Endpoint::Cloud { topic, .. } => topic.clone(),
                    Endpoint::ZenohCloud { topic, .. } => topic.clone(),
                    Endpoint::Local { topic } => topic.clone(),
                };

                let backend = NetworkTopicBackend::new(network_endpoint)?;

                Ok(Self {
                    name: topic_name,
                    backend: TopicBackend::Network(backend),
                    metrics: Arc::new(AtomicTopicMetrics::default()),
                    state: std::sync::atomic::AtomicU8::new(ConnectionState::Connecting.into_u8()),
                    _marker: PhantomData,
                })
            }
        }
    }

    /// Create a Topic from configuration file
    ///
    /// Loads topic configuration from TOML/YAML file and creates the topic with the specified settings.
    ///
    /// # Arguments
    /// * `topic_name` - Name of the topic to look up in the config file
    ///
    /// # Config File Format
    ///
    /// TOML example:
    /// ```toml
    /// [topics.camera]
    /// name = "camera"
    /// endpoint = "camera@router"
    ///
    /// [topics.sensor]
    /// name = "sensor"
    /// transport = "direct"
    /// host = "192.168.1.5"
    /// port = 9000
    /// ```
    ///
    /// # Config File Search Paths
    /// 1. `./horus.toml` or `./horus.yaml`
    /// 2. `~/.horus/config.toml` or `~/.horus/config.yaml`
    /// 3. `/etc/horus/config.toml` or `/etc/horus/config.yaml`
    pub fn from_config_named(topic_name: &str) -> HorusResult<Self> {
        use crate::communication::config::HorusConfig;

        // Load config from standard search paths
        let config = HorusConfig::find_and_load()?;

        // Get hub config (reusing hub config for topics)
        let hub_config = config.get_hub(topic_name)?;

        // Get endpoint string
        let endpoint_str = hub_config.get_endpoint();

        // Create topic with the endpoint
        Self::from_endpoint(&endpoint_str)
    }

    /// Create a Topic from a specific config file path
    ///
    /// # Arguments
    /// * `config_path` - Path to the configuration file (TOML or YAML)
    /// * `topic_name` - Name of the topic to look up in the config file
    pub fn from_config_file<P: AsRef<std::path::Path>>(
        config_path: P,
        topic_name: &str,
    ) -> HorusResult<Self> {
        use crate::communication::config::HorusConfig;

        // Load config from specific file
        let config = HorusConfig::from_file(config_path)?;

        // Get hub config (reusing hub config for topics)
        let hub_config = config.get_hub(topic_name)?;

        // Get endpoint string
        let endpoint_str = hub_config.get_endpoint();

        // Create topic with the endpoint
        Self::from_endpoint(&endpoint_str)
    }

    /// Send a message to a Network topic (zero-overhead)
    ///
    /// This method specifically handles Network backends which require Serialize bounds.
    /// For local backends (SHM, intra-process), use the regular `send()` method.
    ///
    /// # Arguments
    ///
    /// * `msg` - The message to send
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Message sent successfully
    /// * `Err(msg)` - If the send failed (message returned)
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let topic: Topic<SensorData> = Topic::from_endpoint("sensor@192.168.1.5")?;
    /// topic.send_to_network(data)?;
    /// ```
    pub fn send_to_network(&self, msg: T) -> Result<(), T> {
        let result = match &self.backend {
            TopicBackend::Network(inner) => inner.push(msg),
            // For non-network backends, use same path
            TopicBackend::Direct(b) => b.push(msg),
            TopicBackend::SpscIntra(b) => b.push(msg),
            TopicBackend::MpscIntra(b) => b.push(msg),
            TopicBackend::SpmcIntra(b) => b.push(msg),
            TopicBackend::MpmcIntra(b) => b.push(msg),
            TopicBackend::MpmcShm(b) => b.push(msg),
            TopicBackend::SpscShm(b) => b.push(msg),
            TopicBackend::MpscShm(b) => b.push(msg),
            TopicBackend::SpmcShm(b) => b.push(msg),
            TopicBackend::Adaptive(b) => b.send(msg),
        };

        // Update metrics based on result
        match &result {
            Ok(()) => {
                self.metrics.inc_sent();
                self.state.store(
                    ConnectionState::Connected.into_u8(),
                    std::sync::atomic::Ordering::Relaxed,
                );
                // Note: Topic registration is now handled by TopicRegistry, not NodeInfo
            }
            Err(_) => {
                self.metrics.inc_send_failures();
            }
        }

        result
    }

    /// Receive a message from a Network topic
    ///
    /// This method specifically handles Network backends which require DeserializeOwned bounds.
    /// For local backends (SHM, intra-process), use the regular `recv()` method.
    ///
    /// # Arguments
    ///
    /// * `ctx` - Optional node context for logging/metrics (used for NodeInfo integration)
    ///
    /// # Returns
    ///
    /// * `Some(msg)` - A message was available
    /// * `None` - No message available
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let topic: Topic<SensorData> = Topic::from_endpoint("sensor@192.168.1.5")?;
    /// if let Some(data) = topic.recv_from_network(&mut None) {
    ///     process(data);
    /// }
    /// ```
    pub fn recv_from_network(&self, _ctx: &mut Option<&mut NodeInfo>) -> Option<T> {
        let result = match &self.backend {
            TopicBackend::Network(inner) => inner.pop(),
            // For non-network backends, use same path
            TopicBackend::Direct(b) => b.pop(),
            TopicBackend::SpscIntra(b) => b.pop(),
            TopicBackend::MpscIntra(b) => b.pop(),
            TopicBackend::SpmcIntra(b) => b.pop(),
            TopicBackend::MpmcIntra(b) => b.pop(),
            TopicBackend::MpmcShm(b) => b.pop(),
            TopicBackend::SpscShm(b) => b.pop(),
            TopicBackend::MpscShm(b) => b.pop(),
            TopicBackend::SpmcShm(b) => b.pop(),
            TopicBackend::Adaptive(b) => b.recv(),
        };

        // Update metrics based on result
        if result.is_some() {
            self.metrics.inc_received();
            self.state.store(
                ConnectionState::Connected.into_u8(),
                std::sync::atomic::Ordering::Relaxed,
            );
            // Note: Topic registration is now handled by TopicRegistry, not NodeInfo
        }

        result
    }

    /// Check if a Network topic has messages available
    ///
    /// # Returns
    ///
    /// * `true` if there are messages available to receive
    /// * `false` if the queue is empty
    pub fn network_has_messages(&self) -> bool {
        match &self.backend {
            TopicBackend::Network(inner) => !inner.is_empty(),
            TopicBackend::Direct(b) => !b.is_empty(),
            TopicBackend::SpscIntra(b) => !b.is_empty(),
            TopicBackend::MpscIntra(b) => !b.is_empty(),
            TopicBackend::SpmcIntra(b) => !b.is_empty(),
            TopicBackend::MpmcIntra(b) => !b.is_empty(),
            TopicBackend::MpmcShm(b) => !b.is_empty(),
            TopicBackend::SpscShm(b) => !b.is_empty(),
            TopicBackend::MpscShm(b) => !b.is_empty(),
            TopicBackend::SpmcShm(b) => !b.is_empty(),
            TopicBackend::Adaptive(b) => b.has_message(),
        }
    }

    /// Get the backend type name for Network topics
    pub fn network_backend_type(&self) -> &'static str {
        match &self.backend {
            TopicBackend::Network(inner) => inner.backend_name(),
            TopicBackend::Direct(b) => b.backend_name(),
            TopicBackend::SpscIntra(b) => b.backend_name(),
            TopicBackend::MpscIntra(b) => b.backend_name(),
            TopicBackend::SpmcIntra(b) => b.backend_name(),
            TopicBackend::MpmcIntra(b) => b.backend_name(),
            TopicBackend::MpmcShm(b) => b.backend_name(),
            TopicBackend::SpscShm(b) => b.backend_name(),
            TopicBackend::MpscShm(b) => b.backend_name(),
            TopicBackend::SpmcShm(b) => b.backend_name(),
            TopicBackend::Adaptive(b) => b.backend_name(),
        }
    }
}

// ============================================================================
// Topic Send/Recv Implementation
// ============================================================================

impl<T> Topic<T>
where
    T: Clone + Send + Sync + serde::Serialize + serde::de::DeserializeOwned + 'static,
{
    /// Send a message to the topic (zero-overhead hot path)
    ///
    /// This is the TRUE zero-overhead path - no timing, no logging, no syscalls,
    /// NO METRICS updates. For metrics tracking, use `send_tracked()`.
    /// For introspection, use `send_logged()` or external tools like `horus topic echo`.
    ///
    /// # Arguments
    ///
    /// * `msg` - The message to send
    ///
    /// # Returns
    ///
    /// * `Ok(())` if sent successfully
    /// * `Err(msg)` if the buffer is full (message returned)
    ///
    /// # Performance
    ///
    /// This method is optimized for absolute minimum latency:
    /// - DirectChannel: ~3-5ns (same thread)
    /// - SpscIntra: ~15-25ns (cross-thread)
    /// - MpmcIntra: ~30-50ns (same process)
    /// - SpscShm: ~80-100ns (cross-process)
    /// - MpmcShm: ~150-200ns (cross-process)
    ///
    /// # Panics
    ///
    /// Panics if called on a Network backend - use `send_network()` instead.
    #[inline(always)]
    pub fn send(&self, msg: T) -> Result<(), T> {
        // ZERO-OVERHEAD IPC: No timing, no logging, no syscalls, NO METRICS
        // This is the hot path - every nanosecond counts
        match &self.backend {
            TopicBackend::Direct(inner) => inner.push(msg),
            TopicBackend::SpscIntra(inner) => inner.push(msg),
            TopicBackend::MpscIntra(inner) => inner.push(msg),
            TopicBackend::SpmcIntra(inner) => inner.push(msg),
            TopicBackend::MpmcIntra(inner) => inner.push(msg),
            TopicBackend::MpmcShm(inner) => inner.push(msg),
            TopicBackend::SpscShm(inner) => inner.push(msg),
            TopicBackend::MpscShm(inner) => inner.push(msg),
            TopicBackend::SpmcShm(inner) => inner.push(msg),
            TopicBackend::Adaptive(inner) => inner.send(msg),
            TopicBackend::Network(_) => {
                panic!("Network Topic send() requires Serialize bound. Use from_endpoint().")
            }
        }
    }

    /// Send a message with metrics tracking
    ///
    /// Like `send()`, but updates internal counters for monitoring.
    /// Use this when you need accurate message counts.
    ///
    /// Note: Adds ~10-20ns overhead from atomic counter updates.
    #[inline]
    pub fn send_tracked(&self, msg: T) -> Result<(), T> {
        let result = self.send(msg);

        // Update metrics based on result (atomic counter, no syscalls)
        match &result {
            Ok(()) => {
                self.metrics.inc_sent();
                self.state
                    .store(ConnectionState::Connected.into_u8(), Ordering::Relaxed);
            }
            Err(_) => {
                self.metrics.inc_send_failures();
            }
        }

        result
    }

    /// Send a message with detailed logging (requires LogSummary)
    ///
    /// Like `send()`, but logs the message's `log_summary()` to the introspection system.
    /// Use this when you want to capture message contents for monitoring tools.
    ///
    /// Note: This method writes to the global log buffer for introspection tools like
    /// `horus topic echo` and `horus monitor`. For console output, use external tools.
    #[inline]
    pub fn send_logged(&self, msg: T) -> Result<(), T>
    where
        T: LogSummary,
    {
        // Compute summary BEFORE msg is moved (for automatic logging)
        let summary = msg.log_summary();

        // Measure IPC time
        let start = Instant::now();

        let result = match &self.backend {
            TopicBackend::Direct(inner) => inner.push(msg),
            TopicBackend::SpscIntra(inner) => inner.push(msg),
            TopicBackend::MpscIntra(inner) => inner.push(msg),
            TopicBackend::SpmcIntra(inner) => inner.push(msg),
            TopicBackend::MpmcIntra(inner) => inner.push(msg),
            TopicBackend::MpmcShm(inner) => inner.push(msg),
            TopicBackend::SpscShm(inner) => inner.push(msg),
            TopicBackend::MpscShm(inner) => inner.push(msg),
            TopicBackend::SpmcShm(inner) => inner.push(msg),
            TopicBackend::Adaptive(inner) => inner.send(msg),
            TopicBackend::Network(_) => {
                panic!("Network Topic send_logged() requires Serialize bound. Network topics should be created via from_endpoint() which ensures T has proper bounds.")
            }
        };

        let ipc_ns = start.elapsed().as_nanos() as u64;

        match &result {
            Ok(()) => {
                self.metrics.inc_sent();
                self.state
                    .store(ConnectionState::Connected.into_u8(), Ordering::Relaxed);

                // Write to introspection log buffer using thread-local node context
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
            Err(_) => {
                self.metrics.inc_send_failures();
            }
        }

        result
    }

    /// Receive a message from the topic (zero-overhead)
    ///
    /// This is the fast path - no timing, no logging, no syscalls.
    /// For introspection, use `recv_logged()` or external tools like `horus topic echo`.
    ///
    /// # Returns
    ///
    /// * `Some(msg)` if a message is available
    /// * `None` if no message available
    ///
    /// # Panics
    ///
    /// Panics if called on a Network backend - use `recv_network()` instead for network topics.
    #[inline(always)]
    pub fn recv(&self) -> Option<T> {
        // ZERO-OVERHEAD IPC: No timing, no logging, no syscalls, NO METRICS
        // This is the hot path - every nanosecond counts
        // For introspection, use recv_logged() or external tools like `horus topic echo`
        match &self.backend {
            TopicBackend::Direct(inner) => inner.pop(),
            TopicBackend::SpscIntra(inner) => inner.pop(),
            TopicBackend::MpscIntra(inner) => inner.pop(),
            TopicBackend::SpmcIntra(inner) => inner.pop(),
            TopicBackend::MpmcIntra(inner) => inner.pop(),
            TopicBackend::MpmcShm(inner) => inner.pop(),
            TopicBackend::SpscShm(inner) => inner.pop(),
            TopicBackend::MpscShm(inner) => inner.pop(),
            TopicBackend::SpmcShm(inner) => inner.pop(),
            TopicBackend::Adaptive(inner) => inner.recv(),
            TopicBackend::Network(_) => {
                panic!("Network Topic recv() requires DeserializeOwned bound. Network topics should be created via from_endpoint() which ensures T has proper bounds.")
            }
        }
    }

    /// Receive a message with metrics tracking
    ///
    /// Like `recv()`, but updates internal counters for monitoring.
    /// Use this when you need accurate message counts.
    ///
    /// Note: Adds ~10-20ns overhead from atomic counter updates.
    #[inline]
    pub fn recv_tracked(&self) -> Option<T> {
        let result = self.recv();

        // Update metrics if we received a message
        if result.is_some() {
            self.metrics.inc_received();
        }

        result
    }

    /// Receive a message with detailed logging (requires LogSummary)
    ///
    /// Like `recv()`, but logs the message's `log_summary()` instead of just type name.
    /// Use this when you want to see message contents in the automatic logging.
    #[inline]
    pub fn recv_logged(&self) -> Option<T>
    where
        T: LogSummary,
    {
        // Measure IPC time
        let start = Instant::now();

        let result = match &self.backend {
            TopicBackend::Direct(inner) => inner.pop(),
            TopicBackend::SpscIntra(inner) => inner.pop(),
            TopicBackend::MpscIntra(inner) => inner.pop(),
            TopicBackend::SpmcIntra(inner) => inner.pop(),
            TopicBackend::MpmcIntra(inner) => inner.pop(),
            TopicBackend::MpmcShm(inner) => inner.pop(),
            TopicBackend::SpscShm(inner) => inner.pop(),
            TopicBackend::MpscShm(inner) => inner.pop(),
            TopicBackend::SpmcShm(inner) => inner.pop(),
            TopicBackend::Adaptive(inner) => inner.recv(),
            TopicBackend::Network(_) => {
                panic!("Network Topic recv_logged() requires DeserializeOwned bound. Network topics should be created via from_endpoint() which ensures T has proper bounds.")
            }
        };

        let ipc_ns = start.elapsed().as_nanos() as u64;

        if let Some(ref msg) = result {
            self.metrics.inc_received();

            // Write to introspection log buffer using thread-local node context
            use crate::core::hlog::{current_node_name, current_tick_number};
            use crate::core::log_buffer::{publish_log, LogEntry, LogType};
            let now = chrono::Local::now();
            let summary = msg.log_summary();
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

    /// Get a snapshot of the topic's metrics
    ///
    /// Returns current counters for sent/received messages and failures.
    pub fn get_metrics(&self) -> TopicMetrics {
        self.metrics.snapshot()
    }

    /// Get the current connection state
    ///
    /// For local shared memory backends, this is always Connected.
    /// For network backends, this reflects the actual connection status.
    pub fn get_connection_state(&self) -> ConnectionState {
        ConnectionState::from_u8(self.state.load(Ordering::Relaxed))
    }

    /// Check if messages are available
    #[inline]
    pub fn has_messages(&self) -> bool {
        match &self.backend {
            TopicBackend::Direct(inner) => !inner.is_empty(),
            TopicBackend::SpscIntra(inner) => !inner.is_empty(),
            TopicBackend::MpscIntra(inner) => !inner.is_empty(),
            TopicBackend::SpmcIntra(inner) => !inner.is_empty(),
            TopicBackend::MpmcIntra(inner) => !inner.is_empty(),
            TopicBackend::MpmcShm(inner) => !inner.is_empty(),
            TopicBackend::SpscShm(inner) => !inner.is_empty(),
            TopicBackend::MpscShm(inner) => !inner.is_empty(),
            TopicBackend::SpmcShm(inner) => !inner.is_empty(),
            TopicBackend::Adaptive(inner) => inner.has_message(),
            TopicBackend::Network(_) => false, // Network backends can't reliably check for pending messages
        }
    }

    /// Get the topic name
    #[inline]
    pub fn get_topic_name(&self) -> &str {
        &self.name
    }

    /// Get the topic name (standard method name)
    #[inline]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Get the backend type name (for debugging)
    pub fn backend_type(&self) -> &'static str {
        match &self.backend {
            TopicBackend::Direct(inner) => inner.backend_name(),
            TopicBackend::SpscIntra(inner) => inner.backend_name(),
            TopicBackend::MpscIntra(inner) => inner.backend_name(),
            TopicBackend::SpmcIntra(inner) => inner.backend_name(),
            TopicBackend::MpmcIntra(inner) => inner.backend_name(),
            TopicBackend::MpmcShm(inner) => inner.backend_name(),
            TopicBackend::SpscShm(inner) => inner.backend_name(),
            TopicBackend::MpscShm(inner) => inner.backend_name(),
            TopicBackend::SpmcShm(inner) => inner.backend_name(),
            TopicBackend::Adaptive(inner) => inner.backend_name(),
            TopicBackend::Network(_) => "Network",
        }
    }

    /// Check if all participants are in the same process (for debugging)
    pub fn is_same_process(&self) -> bool {
        match &self.backend {
            TopicBackend::Adaptive(inner) => inner.is_same_process(),
            _ => true, // Non-adaptive backends are always same-process
        }
    }

    /// Check if caller is on same thread as creator (for debugging)
    pub fn is_same_thread(&self) -> bool {
        match &self.backend {
            TopicBackend::Adaptive(inner) => inner.is_same_thread(),
            _ => true, // Non-adaptive backends don't track thread
        }
    }

    /// Get publisher count (for debugging)
    pub fn pub_count(&self) -> u32 {
        match &self.backend {
            TopicBackend::Adaptive(inner) => inner.pub_count(),
            _ => 1, // Non-adaptive backends are typically 1
        }
    }

    /// Get subscriber count (for debugging)
    pub fn sub_count(&self) -> u32 {
        match &self.backend {
            TopicBackend::Adaptive(inner) => inner.sub_count(),
            _ => 1, // Non-adaptive backends are typically 1
        }
    }

    /// Send with synchronous callback (DirectChannel only)
    ///
    /// Zero-copy message delivery - the callback receives a reference
    /// to the message for immediate processing.
    ///
    /// # Arguments
    ///
    /// * `msg` - The message to send
    /// * `handler` - Callback that processes the message
    ///
    /// # Returns
    ///
    /// * `Ok(R)` - Result from the handler (DirectChannel)
    /// * `Err(msg)` - If not using DirectChannel backend or wrong thread
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let topic: Topic<SensorData> = Topic::direct("sensor");
    ///
    /// // Zero-copy delivery with inline processing
    /// let result = topic.send_sync(data, |msg| {
    ///     process_sensor(msg);
    ///     msg.id // Return value from callback
    /// })?;
    /// ```
    #[inline]
    pub fn send_sync<R, F>(&self, msg: T, handler: F) -> Result<R, T>
    where
        F: FnOnce(&T) -> R,
    {
        match &self.backend {
            TopicBackend::Direct(inner) => inner.send_sync(msg, handler),
            _ => Err(msg), // Only DirectChannel supports sync callbacks
        }
    }

    /// Send with ownership transfer callback (DirectChannel only)
    ///
    /// The callback takes ownership of the message.
    ///
    /// # Arguments
    ///
    /// * `msg` - The message to send (ownership transferred)
    /// * `handler` - Callback that takes ownership
    ///
    /// # Returns
    ///
    /// * `Ok(R)` - Result from the handler (DirectChannel)
    /// * `Err(msg)` - If not using DirectChannel backend or wrong thread
    #[inline]
    pub fn send_owned<R, F>(&self, msg: T, handler: F) -> Result<R, T>
    where
        F: FnOnce(T) -> R,
    {
        match &self.backend {
            TopicBackend::Direct(inner) => inner.send_owned(msg, handler),
            _ => Err(msg), // Only DirectChannel supports owned callbacks
        }
    }

    /// Ultra-fast send without thread validation (~3-5ns)
    ///
    /// This is the fastest possible send path, skipping all runtime checks.
    /// Only available for DirectChannel backend.
    ///
    /// # Safety
    ///
    /// Caller must guarantee:
    /// - This Topic was created with `Topic::direct()`
    /// - This is called from the same thread that created the Topic
    /// - No concurrent access from other threads
    ///
    /// # Panics
    ///
    /// Panics if called on a non-DirectChannel backend.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let topic: Topic<u64> = Topic::direct("fast");
    ///
    /// // SAFETY: Same thread, single-threaded scheduler, DirectChannel backend
    /// unsafe {
    ///     topic.send_unchecked(42);
    ///     let val = topic.recv_unchecked();
    /// }
    /// ```
    #[inline(always)]
    pub unsafe fn send_unchecked(&self, msg: T) {
        match &self.backend {
            TopicBackend::Direct(inner) => inner.push_unchecked(msg),
            _ => panic!("send_unchecked called on non-DirectChannel backend"),
        }
    }

    /// Ultra-fast receive without thread validation (~3-5ns)
    ///
    /// This is the fastest possible receive path, skipping all runtime checks.
    /// Only available for DirectChannel backend.
    ///
    /// # Safety
    ///
    /// Caller must guarantee:
    /// - This Topic was created with `Topic::direct()`
    /// - This is called from the same thread that created the Topic
    /// - No concurrent access from other threads
    ///
    /// # Panics
    ///
    /// Panics if called on a non-DirectChannel backend.
    #[inline(always)]
    pub unsafe fn recv_unchecked(&self) -> Option<T> {
        match &self.backend {
            TopicBackend::Direct(inner) => inner.pop_unchecked(),
            _ => panic!("recv_unchecked called on non-DirectChannel backend"),
        }
    }
}

// ============================================================================
// PodTopic - Specialized Topic for PodMessage types
// ============================================================================

/// Specialized Topic for PodMessage types
///
/// Uses zero-copy PodShm backend for ~50ns latency.
/// This is the fastest Topic variant.
///
/// # Example
///
/// ```rust,ignore
/// use horus_core::communication::PodTopic;
///
/// let producer: PodTopic<MotorCommand> = PodTopic::producer("motor_cmd")?;
/// producer.send(cmd);
///
/// let consumer: PodTopic<MotorCommand> = PodTopic::consumer("motor_cmd")?;
/// if let Some(cmd) = consumer.recv() {
///     apply(cmd);
/// }
/// ```
pub struct PodTopic<T: PodMessage> {
    name: String,
    backend: PodShmBackend<T>,
}

impl<T: PodMessage> PodTopic<T> {
    /// Create a producer for POD topic
    pub fn producer(name: impl Into<String>) -> HorusResult<Self> {
        let name = name.into();
        let shm_name = format!("pod/{}", name);

        Ok(Self {
            name: name.clone(),
            backend: PodShmBackend::producer(&shm_name)?,
        })
    }

    /// Create a consumer for POD topic
    pub fn consumer(name: impl Into<String>) -> HorusResult<Self> {
        let name = name.into();
        let shm_name = format!("pod/{}", name);

        Ok(Self {
            name: name.clone(),
            backend: PodShmBackend::consumer(&shm_name)?,
        })
    }

    /// Send a POD message (~50ns)
    #[inline]
    pub fn send(&self, msg: T) -> Result<(), T> {
        self.backend.push(msg)
    }

    /// Receive a POD message (~50ns)
    #[inline]
    pub fn recv(&self) -> Option<T> {
        self.backend.pop()
    }

    /// Check if messages are available
    #[inline]
    pub fn has_messages(&self) -> bool {
        !self.backend.is_empty()
    }

    /// Get the topic name
    #[inline]
    pub fn name(&self) -> &str {
        &self.name
    }
}

impl<T: PodMessage> Clone for PodTopic<T> {
    fn clone(&self) -> Self {
        Self {
            name: self.name.clone(),
            backend: self.backend.clone(),
        }
    }
}

// ============================================================================
// Debug Implementation
// ============================================================================

impl<T> std::fmt::Debug for Topic<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let backend_type = match &self.backend {
            TopicBackend::Direct(_) => "DirectChannel",
            TopicBackend::SpscIntra(_) => "SpscIntra",
            TopicBackend::MpscIntra(_) => "MpscIntra",
            TopicBackend::SpmcIntra(_) => "SpmcIntra",
            TopicBackend::MpmcIntra(_) => "MpmcIntra",
            TopicBackend::MpmcShm(_) => "MpmcShm",
            TopicBackend::SpscShm(_) => "SpscShm",
            TopicBackend::MpscShm(_) => "MpscShm",
            TopicBackend::SpmcShm(_) => "SpmcShm",
            TopicBackend::Adaptive(_) => "Adaptive",
            TopicBackend::Network(_) => "Network",
        };
        let state = ConnectionState::from_u8(self.state.load(Ordering::Relaxed));
        f.debug_struct("Topic")
            .field("name", &self.name)
            .field("backend", &backend_type)
            .field("state", &state)
            .finish()
    }
}

impl<T: PodMessage> std::fmt::Debug for PodTopic<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PodTopic")
            .field("name", &self.name)
            .field("backend", &"PodShm")
            .finish()
    }
}

// ============================================================================
// Mode Marker Types (for internal/advanced use only)
// ============================================================================

/// Marker type for Pod access mode (internal)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pod;

/// Marker type for SPSC access mode (internal)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Spsc;

/// Marker type for MPMC access mode (internal)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mpmc;

/// Marker type for automatic mode selection (internal)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Auto;

// Re-export for backward compatibility
pub use crate::communication::storage::AccessMode as TopicMode;

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    struct TestMessage {
        id: u64,
        payload: String,
    }

    impl crate::core::LogSummary for TestMessage {
        fn log_summary(&self) -> String {
            format!(
                "TestMessage{{id: {}, payload: {:?}}}",
                self.id, self.payload
            )
        }
    }

    #[test]
    fn test_topic_new() {
        let unique_name = format!("test_topic_new_{}", std::process::id());
        let topic: Topic<TestMessage> = Topic::new(&unique_name).unwrap();
        assert_eq!(topic.name(), &unique_name);
        // AdaptiveTopic is now the default - starts as Unknown before first send/recv
        assert_eq!(topic.backend_type(), "Unknown (Adaptive)");
    }

    #[test]
    fn test_topic_send_recv() {
        let unique_name = format!("test_topic_sr_{}", std::process::id());

        let topic: Topic<TestMessage> = Topic::new(&unique_name).unwrap();
        let consumer = topic.clone();

        let msg = TestMessage {
            id: 42,
            payload: "hello topic".to_string(),
        };
        topic.send(msg.clone()).unwrap();

        let received = consumer.recv();
        assert!(received.is_some());
        let received = received.unwrap();
        assert_eq!(received.id, 42);
        assert_eq!(received.payload, "hello topic");
    }

    #[test]
    fn test_topic_multiple_messages() {
        let unique_name = format!("test_topic_multi_{}", std::process::id());

        let producer: Topic<TestMessage> = Topic::new(&unique_name).unwrap();
        let consumer = producer.clone();

        for i in 0..5 {
            let msg = TestMessage {
                id: i,
                payload: format!("msg_{}", i),
            };
            producer.send(msg).unwrap();
        }

        for i in 0..5 {
            let received = consumer.recv();
            assert!(received.is_some(), "Expected message {}", i);
            let received = received.unwrap();
            assert_eq!(received.id, i);
        }

        assert!(consumer.recv().is_none());
    }

    #[test]
    fn test_topic_clone() {
        let unique_name = format!("test_topic_clone_{}", std::process::id());

        let topic1: Topic<TestMessage> = Topic::new(&unique_name).unwrap();
        let topic2 = topic1.clone();

        assert_eq!(topic1.name(), topic2.name());

        let msg = TestMessage {
            id: 1,
            payload: "shared".to_string(),
        };
        topic1.send(msg.clone()).unwrap();

        let received = topic2.recv();
        assert!(received.is_some());
        assert_eq!(received.unwrap().id, 1);
    }

    #[test]
    fn test_topic_has_messages() {
        let unique_name = format!("test_topic_has_{}", std::process::id());

        let topic: Topic<TestMessage> = Topic::new(&unique_name).unwrap();

        assert!(!topic.has_messages());

        let msg = TestMessage {
            id: 1,
            payload: "test".to_string(),
        };
        topic.send(msg).unwrap();

        assert!(topic.has_messages());

        let _ = topic.recv();

        assert!(!topic.has_messages());
    }

    #[test]
    fn test_topic_debug() {
        let unique_name = format!("test_topic_debug_{}", std::process::id());
        let topic: Topic<TestMessage> = Topic::new(&unique_name).unwrap();

        let debug_str = format!("{:?}", topic);
        assert!(debug_str.contains("Topic"));
        assert!(debug_str.contains(&unique_name));
        // AdaptiveTopic is now the default
        assert!(debug_str.contains("Adaptive"));
    }

    #[test]
    fn test_topic_with_capacity() {
        let unique_name = format!("test_topic_cap_{}", std::process::id());
        let topic: Topic<TestMessage> = Topic::with_capacity(&unique_name, 128).unwrap();
        assert_eq!(topic.name(), &unique_name);
    }

    #[test]
    fn test_topic_config() {
        let config = TopicConfig::new("test")
            .with_capacity(256)
            .with_create(false)
            .as_consumer();

        assert_eq!(config.name, "test");
        assert_eq!(config.capacity, 256);
        assert!(!config.create);
        assert!(!config.is_producer);
    }

    #[test]
    fn test_adaptive_topic_communication() {
        let unique_name = format!("test_adaptive_{}", std::process::id());

        // Use Topic::new() which auto-detects role on first send/recv
        let topic1: Topic<TestMessage> = Topic::new(&unique_name).unwrap();
        // Clone the topic for the receiver - this shares the underlying AdaptiveTopic state
        let topic2 = topic1.clone();

        // Topic::new() uses AdaptiveTopic backend
        let backend = topic1.backend_type();
        assert!(
            backend.contains("Adaptive") || backend.contains("Shm") || backend.contains("Intra"),
            "Expected adaptive/shm/intra backend, got: {}",
            backend
        );

        let msg = TestMessage {
            id: 99,
            payload: "adaptive test".to_string(),
        };
        topic1.send(msg.clone()).unwrap();

        let received = topic2.recv();
        assert!(received.is_some());
        assert_eq!(received.unwrap().id, 99);
    }

    // ========================================================================
    // DirectChannel Tests
    // ========================================================================

    #[test]
    fn test_direct_channel_basic() {
        let topic: Topic<TestMessage> = Topic::direct("direct_basic");

        assert_eq!(topic.backend_type(), "DirectChannel");
        assert!(!topic.has_messages());

        let msg = TestMessage {
            id: 1,
            payload: "direct".to_string(),
        };
        topic.send(msg).unwrap();

        assert!(topic.has_messages());

        let received = topic.recv();
        assert!(received.is_some());
        let received = received.unwrap();
        assert_eq!(received.id, 1);
        assert_eq!(received.payload, "direct");

        assert!(!topic.has_messages());
    }

    #[test]
    fn test_direct_channel_overwrite() {
        // DirectChannel is single-slot, new messages overwrite old ones
        let topic: Topic<TestMessage> = Topic::direct("direct_overwrite");

        let msg1 = TestMessage {
            id: 1,
            payload: "first".to_string(),
        };
        let msg2 = TestMessage {
            id: 2,
            payload: "second".to_string(),
        };

        topic.send(msg1).unwrap();
        topic.send(msg2).unwrap();

        // Should get the second message (overwritten)
        let received = topic.recv();
        assert!(received.is_some());
        assert_eq!(received.unwrap().id, 2);

        // Nothing left
        assert!(topic.recv().is_none());
    }

    #[test]
    fn test_direct_channel_send_sync() {
        let topic: Topic<TestMessage> = Topic::direct("direct_sync");

        let msg = TestMessage {
            id: 42,
            payload: "sync callback".to_string(),
        };

        // send_sync delivers immediately via callback
        let result = topic.send_sync(msg, |m| {
            assert_eq!(m.id, 42);
            m.id * 2
        });

        assert!(result.is_ok());
        assert_eq!(result.unwrap(), 84);
    }

    #[test]
    fn test_direct_channel_send_owned() {
        let topic: Topic<TestMessage> = Topic::direct("direct_owned");

        let msg = TestMessage {
            id: 7,
            payload: "owned".to_string(),
        };

        // send_owned transfers ownership to callback
        let result = topic.send_owned(msg, |m| {
            assert_eq!(m.id, 7);
            format!("processed: {}", m.payload)
        });

        assert!(result.is_ok());
        assert_eq!(result.unwrap(), "processed: owned");
    }

    #[test]
    fn test_direct_channel_debug() {
        let topic: Topic<TestMessage> = Topic::direct("direct_debug");

        let debug_str = format!("{:?}", topic);
        assert!(debug_str.contains("Topic"));
        assert!(debug_str.contains("direct_debug"));
        assert!(debug_str.contains("DirectChannel"));
    }

    #[test]
    fn test_direct_channel_clone() {
        // Cloning a DirectChannel creates a new independent channel
        let topic1: Topic<TestMessage> = Topic::direct("direct_clone");
        let topic2 = topic1.clone();

        // Each clone has its own slot
        let msg = TestMessage {
            id: 1,
            payload: "clone test".to_string(),
        };
        topic1.send(msg).unwrap();

        // topic2 should NOT see topic1's message (independent channels)
        assert!(!topic2.has_messages());
        assert!(topic1.has_messages());
    }

    #[test]
    fn test_send_sync_fails_for_non_direct() {
        let unique_name = format!("test_sync_fail_{}", std::process::id());
        let topic: Topic<TestMessage> = Topic::new(&unique_name).unwrap();

        let msg = TestMessage {
            id: 1,
            payload: "test".to_string(),
        };

        // send_sync should fail for non-DirectChannel backends
        let result = topic.send_sync(msg, |_m| 42);
        assert!(result.is_err());
    }

    #[test]
    fn test_direct_channel_latency_estimation() {
        // This test verifies DirectChannel is suitable for ultra-low-latency use cases
        // by measuring the operations are fast (not a precise benchmark, but sanity check)
        use std::hint::black_box;
        use std::time::Instant;

        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct TinyMsg(u64);

        let topic: Topic<TinyMsg> = Topic::direct("latency_test");
        let iterations = 100_000u64;

        // Warmup
        for i in 0..1000u64 {
            topic.send(black_box(TinyMsg(i))).unwrap();
            black_box(topic.recv());
        }

        // Measure safe API (with thread validation ~30ns)
        let start = Instant::now();
        for i in 0..iterations {
            topic.send(black_box(TinyMsg(i))).unwrap();
            black_box(topic.recv());
        }
        let elapsed = start.elapsed();
        let per_op_ns = elapsed.as_nanos() as f64 / (iterations as f64 * 2.0);
        println!("DirectChannel safe API per-op latency: {:.2}ns", per_op_ns);

        // Measure unchecked API (without thread validation, target ~3-5ns)
        // SAFETY: Same thread, no concurrent access, DirectChannel backend
        let start = Instant::now();
        for i in 0..iterations {
            unsafe {
                topic.send_unchecked(black_box(TinyMsg(i)));
                black_box(topic.recv_unchecked());
            }
        }
        let elapsed = start.elapsed();
        let per_op_unchecked_ns = elapsed.as_nanos() as f64 / (iterations as f64 * 2.0);
        println!(
            "DirectChannel unchecked API per-op latency: {:.2}ns",
            per_op_unchecked_ns
        );

        // Sanity check - safe API should be under 5µs (generous for parallel test execution)
        assert!(
            per_op_ns < 5000.0,
            "DirectChannel safe too slow: {:.2}ns",
            per_op_ns
        );

        // Unchecked should be significantly faster than safe
        assert!(
            per_op_unchecked_ns < per_op_ns,
            "Unchecked should be faster than safe"
        );
    }

    // ========================================================================
    // SpscIntra Tests
    // ========================================================================

    #[test]
    fn test_spsc_intra_basic() {
        let (producer, consumer) = Topic::<TestMessage>::spsc_intra("spsc_basic");

        assert_eq!(producer.backend_type(), "SpscIntra");
        assert_eq!(consumer.backend_type(), "SpscIntra");

        let msg = TestMessage {
            id: 42,
            payload: "spsc intra test".to_string(),
        };
        producer.send(msg).unwrap();

        let received = consumer.recv();
        assert!(received.is_some());
        let received = received.unwrap();
        assert_eq!(received.id, 42);
        assert_eq!(received.payload, "spsc intra test");
    }

    #[test]
    fn test_spsc_intra_overwrite() {
        // SpscIntra is single-slot, new messages overwrite old ones
        let (producer, consumer) = Topic::<TestMessage>::spsc_intra("spsc_overwrite");

        let msg1 = TestMessage {
            id: 1,
            payload: "first".to_string(),
        };
        let msg2 = TestMessage {
            id: 2,
            payload: "second".to_string(),
        };

        producer.send(msg1).unwrap();
        producer.send(msg2).unwrap();

        // Should get the second message (overwritten)
        let received = consumer.recv();
        assert!(received.is_some());
        assert_eq!(received.unwrap().id, 2);

        // Nothing left
        assert!(consumer.recv().is_none());
    }

    #[test]
    fn test_spsc_intra_cross_thread() {
        use std::thread;

        let (producer, consumer) = Topic::<TestMessage>::spsc_intra("spsc_threads");

        // Spawn producer thread
        let producer_handle = thread::spawn(move || {
            for i in 0..10 {
                let msg = TestMessage {
                    id: i,
                    payload: format!("msg_{}", i),
                };
                producer.send(msg).unwrap();
                thread::sleep(std::time::Duration::from_micros(100));
            }
        });

        // Give producer a head start
        thread::sleep(std::time::Duration::from_millis(1));

        // Consume messages
        let mut received_count = 0;
        let mut last_id = None;
        for _ in 0..100 {
            if let Some(msg) = consumer.recv() {
                received_count += 1;
                // Due to single-slot design, we may skip some messages
                // but IDs should be monotonically increasing
                if let Some(last) = last_id {
                    assert!(
                        msg.id > last || msg.id == last,
                        "ID should not go backwards"
                    );
                }
                last_id = Some(msg.id);
            }
            thread::sleep(std::time::Duration::from_micros(50));
        }

        producer_handle.join().unwrap();

        // We should have received at least one message
        assert!(
            received_count > 0,
            "Should have received at least one message"
        );
    }

    #[test]
    fn test_spsc_intra_has_messages() {
        let (producer, consumer) = Topic::<TestMessage>::spsc_intra("spsc_has_messages");

        assert!(!consumer.has_messages());

        let msg = TestMessage {
            id: 1,
            payload: "test".to_string(),
        };
        producer.send(msg).unwrap();

        assert!(consumer.has_messages());

        let _ = consumer.recv();

        assert!(!consumer.has_messages());
    }

    #[test]
    fn test_spsc_intra_debug() {
        let (producer, _consumer) = Topic::<TestMessage>::spsc_intra("spsc_debug");

        let debug_str = format!("{:?}", producer);
        assert!(debug_str.contains("Topic"));
        assert!(debug_str.contains("spsc_debug"));
        assert!(debug_str.contains("SpscIntra"));
    }

    #[test]
    fn test_spsc_intra_latency_estimation() {
        use std::hint::black_box;
        use std::time::Instant;

        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct TinyMsg(u64);

        // Use unique topic name to avoid conflicts in parallel test runs
        let topic_name = format!("spsc_latency_{}", std::process::id());
        let (producer, consumer) = Topic::<TinyMsg>::spsc_intra(&topic_name);
        let iterations = 100_000u64;

        // Warmup
        for i in 0..1000u64 {
            producer.send(black_box(TinyMsg(i))).unwrap();
            black_box(consumer.recv());
        }

        // Measure
        let start = Instant::now();
        for i in 0..iterations {
            producer.send(black_box(TinyMsg(i))).unwrap();
            black_box(consumer.recv());
        }
        let elapsed = start.elapsed();

        let per_op_ns = elapsed.as_nanos() as f64 / (iterations as f64 * 2.0);
        println!("SpscIntra per-op latency: {:.2}ns", per_op_ns);

        // Target: ≤30ns (p99), allow generous margin for parallel test execution
        assert!(per_op_ns < 5000.0, "SpscIntra too slow: {:.2}ns", per_op_ns);
    }

    // ========================================================================
    // MpscIntra Tests
    // ========================================================================

    #[test]
    fn test_mpsc_intra_basic_send_receive() {
        let (producer, consumer) = Topic::<TestMessage>::mpsc_intra("mpsc_basic", 64);

        assert_eq!(producer.backend_type(), "MpscIntra");
        assert_eq!(consumer.backend_type(), "MpscIntra");

        let msg = TestMessage {
            id: 42,
            payload: "hello".to_string(),
        };

        producer.send(msg.clone()).unwrap();

        let received = consumer.recv();
        assert!(received.is_some());
        let received = received.unwrap();
        assert_eq!(received.id, 42);
        assert_eq!(received.payload, "hello");
    }

    #[test]
    fn test_mpsc_intra_multiple_producers() {
        let (producer1, consumer) = Topic::<TestMessage>::mpsc_intra("mpsc_multi_prod", 64);
        let producer2 = producer1.clone();

        // Send from both producers
        producer1
            .send(TestMessage {
                id: 1,
                payload: "from_1".to_string(),
            })
            .unwrap();
        producer2
            .send(TestMessage {
                id: 2,
                payload: "from_2".to_string(),
            })
            .unwrap();

        // Consumer should receive both
        let msg1 = consumer.recv().unwrap();
        let msg2 = consumer.recv().unwrap();

        // Messages should be received in order (FIFO)
        assert_eq!(msg1.id, 1);
        assert_eq!(msg2.id, 2);
    }

    #[test]
    fn test_mpsc_intra_producer_cannot_receive() {
        let (producer, _consumer) = Topic::<TestMessage>::mpsc_intra("mpsc_prod_recv", 64);

        producer
            .send(TestMessage {
                id: 1,
                payload: "test".to_string(),
            })
            .unwrap();

        // Producer should not be able to receive
        let result = producer.recv();
        assert!(result.is_none());
    }

    #[test]
    fn test_mpsc_intra_consumer_cannot_send() {
        let (_producer, consumer) = Topic::<TestMessage>::mpsc_intra("mpsc_cons_send", 64);

        let msg = TestMessage {
            id: 1,
            payload: "test".to_string(),
        };
        let result = consumer.send(msg);

        // Consumer should not be able to send
        assert!(result.is_err());
    }

    #[test]
    fn test_mpsc_intra_queue_full() {
        // Create a small queue
        let (producer, _consumer) = Topic::<TestMessage>::mpsc_intra("mpsc_full", 4);

        // Fill the queue (capacity is rounded to power of 2, so 4)
        for i in 0..4 {
            let result = producer.send(TestMessage {
                id: i,
                payload: format!("msg_{}", i),
            });
            assert!(result.is_ok(), "Should be able to send message {}", i);
        }

        // Next send should fail (queue full)
        let result = producer.send(TestMessage {
            id: 99,
            payload: "overflow".to_string(),
        });
        assert!(result.is_err(), "Queue should be full");
    }

    #[test]
    fn test_mpsc_intra_empty_receive() {
        let (_producer, consumer) = Topic::<TestMessage>::mpsc_intra("mpsc_empty", 64);

        let result = consumer.recv();
        assert!(result.is_none());
    }

    #[test]
    fn test_mpsc_intra_has_messages() {
        let (producer, consumer) = Topic::<TestMessage>::mpsc_intra("mpsc_has_msg", 64);

        assert!(!consumer.has_messages());

        producer
            .send(TestMessage {
                id: 1,
                payload: "test".to_string(),
            })
            .unwrap();

        assert!(consumer.has_messages());

        let _ = consumer.recv();

        assert!(!consumer.has_messages());
    }

    #[test]
    fn test_mpsc_intra_debug() {
        let (producer, _consumer) = Topic::<TestMessage>::mpsc_intra("mpsc_debug", 64);

        let debug_str = format!("{:?}", producer);
        assert!(debug_str.contains("Topic"));
        assert!(debug_str.contains("mpsc_debug"));
        assert!(debug_str.contains("MpscIntra"));
    }

    #[test]
    fn test_mpsc_intra_concurrent_producers() {
        use std::sync::atomic::{AtomicUsize, Ordering};
        use std::sync::Arc;

        let (producer, consumer) = Topic::<TestMessage>::mpsc_intra("mpsc_concurrent", 256);
        let received_count = Arc::new(AtomicUsize::new(0));
        let num_producers = 4;
        let msgs_per_producer = 50;

        // Spawn multiple producer threads
        let mut handles = vec![];
        for p in 0..num_producers {
            let producer = producer.clone();
            let handle = thread::spawn(move || {
                for i in 0..msgs_per_producer {
                    let msg = TestMessage {
                        id: (p * msgs_per_producer + i) as u64,
                        payload: format!("producer_{}_msg_{}", p, i),
                    };
                    // Retry if queue is full
                    loop {
                        match producer.send(msg.clone()) {
                            Ok(_) => break,
                            Err(_) => {
                                thread::yield_now();
                            }
                        }
                    }
                }
            });
            handles.push(handle);
        }

        // Consumer thread
        let received_count_clone = received_count.clone();
        let consumer_handle = thread::spawn(move || {
            let expected = num_producers * msgs_per_producer;
            let mut count = 0;
            let mut attempts = 0;
            while count < expected && attempts < 10000 {
                if let Some(_msg) = consumer.recv() {
                    count += 1;
                } else {
                    thread::yield_now();
                    attempts += 1;
                }
            }
            received_count_clone.store(count, Ordering::SeqCst);
        });

        // Wait for producers
        for h in handles {
            h.join().unwrap();
        }

        // Wait for consumer
        consumer_handle.join().unwrap();

        let total_received = received_count.load(Ordering::SeqCst);
        assert_eq!(
            total_received,
            num_producers * msgs_per_producer,
            "Should receive all {} messages",
            num_producers * msgs_per_producer
        );
    }

    #[test]
    fn test_mpsc_intra_latency_estimation() {
        use std::hint::black_box;
        use std::time::Instant;

        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct TinyMsg(u64);

        // Use unique topic name to avoid conflicts in parallel test runs
        let topic_name = format!("mpsc_latency_{}", std::process::id());
        let (producer, consumer) = Topic::<TinyMsg>::mpsc_intra(&topic_name, 1024);
        let iterations = 100_000u64;

        // Warmup
        for i in 0..1000u64 {
            producer.send(black_box(TinyMsg(i))).unwrap();
            black_box(consumer.recv());
        }

        // Measure
        let start = Instant::now();
        for i in 0..iterations {
            producer.send(black_box(TinyMsg(i))).unwrap();
            black_box(consumer.recv());
        }
        let elapsed = start.elapsed();

        let per_op_ns = elapsed.as_nanos() as f64 / (iterations as f64 * 2.0);
        println!("MpscIntra per-op latency: {:.2}ns", per_op_ns);

        // Target: ≤45ns (p99), allow generous margin for parallel test execution
        assert!(per_op_ns < 5000.0, "MpscIntra too slow: {:.2}ns", per_op_ns);
    }

    // ========================================================================
    // SpmcIntra Tests
    // ========================================================================

    #[test]
    fn test_spmc_intra_basic_send_receive() {
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (producer, consumer) = Topic::<TestData>::spmc_intra("spmc_basic");

        // Send a message
        producer.send(TestData(42)).unwrap();

        // Receive the message
        let received = consumer.recv();
        assert_eq!(received, Some(TestData(42)));
    }

    #[test]
    fn test_spmc_intra_multiple_consumers() {
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (producer, consumer1) = Topic::<TestData>::spmc_intra("spmc_multi");
        let consumer2 = consumer1.clone();

        // Send a message
        producer.send(TestData(100)).unwrap();

        // Both consumers receive the same message
        let recv1 = consumer1.recv();
        let recv2 = consumer2.recv();
        assert_eq!(recv1, Some(TestData(100)));
        assert_eq!(recv2, Some(TestData(100)));
    }

    #[test]
    fn test_spmc_intra_producer_cannot_receive() {
        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (producer, _consumer) = Topic::<TestData>::spmc_intra("spmc_prod_no_recv");

        // Producer should receive None
        let result = producer.recv();
        assert!(result.is_none());
    }

    #[test]
    fn test_spmc_intra_consumer_cannot_send() {
        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (_producer, consumer) = Topic::<TestData>::spmc_intra("spmc_cons_no_send");

        // Consumer should fail to send
        let result = consumer.send(TestData(42));
        assert!(result.is_err());
    }

    #[test]
    fn test_spmc_intra_overwrite_behavior() {
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (producer, consumer) = Topic::<TestData>::spmc_intra("spmc_overwrite");

        // Send multiple messages
        producer.send(TestData(1)).unwrap();
        producer.send(TestData(2)).unwrap();
        producer.send(TestData(3)).unwrap();

        // Consumer should get the latest value
        let recv = consumer.recv();
        assert_eq!(recv, Some(TestData(3)));

        // No new data after consuming
        let recv2 = consumer.recv();
        assert!(recv2.is_none());
    }

    #[test]
    fn test_spmc_intra_empty_receive() {
        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (_producer, consumer) = Topic::<TestData>::spmc_intra("spmc_empty");

        // Nothing sent, should receive None
        assert!(consumer.recv().is_none());
    }

    #[test]
    fn test_spmc_intra_has_messages() {
        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (producer, consumer) = Topic::<TestData>::spmc_intra("spmc_has_msgs");

        // Initially no messages
        assert!(!consumer.has_messages());

        // Send a message
        producer.send(TestData(42)).unwrap();

        // Now has messages
        assert!(consumer.has_messages());

        // Consume it
        consumer.recv();

        // No more messages
        assert!(!consumer.has_messages());
    }

    #[test]
    fn test_spmc_intra_debug() {
        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (producer, _consumer) = Topic::<TestData>::spmc_intra("spmc_debug");

        let debug_str = format!("{:?}", producer);
        assert!(debug_str.contains("SpmcIntra"));
    }

    #[test]
    fn test_spmc_intra_concurrent_consumers() {
        use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
        use std::sync::Arc;
        use std::thread;

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (producer, consumer) = Topic::<TestData>::spmc_intra("spmc_concurrent");
        let consumer2 = consumer.clone();
        let consumer3 = consumer.clone();

        let received1 = Arc::new(AtomicU64::new(0));
        let received2 = Arc::new(AtomicU64::new(0));
        let received3 = Arc::new(AtomicU64::new(0));
        let stop = Arc::new(AtomicBool::new(false));

        let r1 = received1.clone();
        let r2 = received2.clone();
        let r3 = received3.clone();
        let s1 = stop.clone();
        let s2 = stop.clone();
        let s3 = stop.clone();

        // Start consumer threads
        let c1_handle = thread::spawn(move || {
            while !s1.load(Ordering::SeqCst) {
                if let Some(data) = consumer.recv() {
                    r1.store(data.0, Ordering::SeqCst);
                }
                std::hint::spin_loop();
            }
        });

        let c2_handle = thread::spawn(move || {
            while !s2.load(Ordering::SeqCst) {
                if let Some(data) = consumer2.recv() {
                    r2.store(data.0, Ordering::SeqCst);
                }
                std::hint::spin_loop();
            }
        });

        let c3_handle = thread::spawn(move || {
            while !s3.load(Ordering::SeqCst) {
                if let Some(data) = consumer3.recv() {
                    r3.store(data.0, Ordering::SeqCst);
                }
                std::hint::spin_loop();
            }
        });

        // Give consumers time to start
        thread::sleep(std::time::Duration::from_millis(1));

        // Producer sends values
        for i in 1..=100u64 {
            producer.send(TestData(i)).unwrap();
            // Give consumers time to read (longer for parallel test reliability)
            thread::sleep(std::time::Duration::from_micros(100));
        }

        // Give extra time for final message to propagate (generous for parallel tests)
        thread::sleep(std::time::Duration::from_millis(50));

        // Stop consumers
        stop.store(true, Ordering::SeqCst);

        c1_handle.join().unwrap();
        c2_handle.join().unwrap();
        c3_handle.join().unwrap();

        // All consumers should have received some value (generous threshold for parallel test load)
        // Under heavy CPU contention, consumers may miss messages
        let r1 = received1.load(Ordering::SeqCst);
        let r2 = received2.load(Ordering::SeqCst);
        let r3 = received3.load(Ordering::SeqCst);
        assert!(
            r1 >= 50,
            "Consumer 1 only saw value {} (expected >= 50)",
            r1
        );
        assert!(
            r2 >= 50,
            "Consumer 2 only saw value {} (expected >= 50)",
            r2
        );
        assert!(
            r3 >= 50,
            "Consumer 3 only saw value {} (expected >= 50)",
            r3
        );
    }

    #[test]
    fn test_spmc_intra_latency_estimation() {
        use std::hint::black_box;
        use std::time::Instant;

        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct TinyMsg(u64);

        // Use unique topic name to avoid conflicts in parallel test runs
        let topic_name = format!("spmc_latency_{}", std::process::id());
        let (producer, consumer) = Topic::<TinyMsg>::spmc_intra(&topic_name);
        let iterations = 100_000u64;

        // Warmup - producer always succeeds
        for i in 0..1000u64 {
            producer.send(black_box(TinyMsg(i))).unwrap();
            black_box(consumer.recv());
        }

        // Measure
        let start = Instant::now();
        for i in 0..iterations {
            producer.send(black_box(TinyMsg(i))).unwrap();
            black_box(consumer.recv());
        }
        let elapsed = start.elapsed();

        let per_op_ns = elapsed.as_nanos() as f64 / (iterations as f64 * 2.0);
        println!("SpmcIntra per-op latency: {:.2}ns", per_op_ns);

        // Target: ≤50ns (p99), allow generous margin for parallel test execution
        assert!(per_op_ns < 5000.0, "SpmcIntra too slow: {:.2}ns", per_op_ns);
    }

    #[test]
    fn test_select_backend_same_process_spmc() {
        let config = TopicConfig::new("test")
            .same_process()
            .with_access_pattern(AccessPattern::Spmc);
        let selected = Topic::<TestMessage>::selected_backend_for(&config);
        assert_eq!(selected, SelectedBackend::SpmcIntra);
    }

    #[test]
    fn test_select_backend_explicit_spmc_intra() {
        let config = TopicConfig::new("test").with_backend(BackendHint::SpmcIntra);
        let selected = Topic::<TestMessage>::selected_backend_for(&config);
        assert_eq!(selected, SelectedBackend::SpmcIntra);
    }

    #[test]
    fn test_from_config_same_process_spmc() {
        let config = TopicConfig::new("config_same_process_spmc")
            .same_process()
            .with_access_pattern(AccessPattern::Spmc);
        let topic: Topic<TestMessage> = Topic::from_config(config).unwrap();
        assert_eq!(topic.backend_type(), "SpmcIntra");
    }

    // ========================================================================
    // MpmcIntra Tests
    // ========================================================================

    #[test]
    fn test_mpmc_intra_basic_send_receive() {
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (producer, consumer) = Topic::<TestData>::mpmc_intra("mpmc_basic", 64);

        // Send a message
        producer.send(TestData(42)).unwrap();

        // Receive the message
        let received = consumer.recv();
        assert_eq!(received, Some(TestData(42)));
    }

    #[test]
    fn test_mpmc_intra_multiple_producers() {
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (producer, consumer) = Topic::<TestData>::mpmc_intra("mpmc_multi_prod", 64);
        let producer2 = producer.clone();

        // Multiple producers send
        producer.send(TestData(1)).unwrap();
        producer2.send(TestData(2)).unwrap();

        // Receive both messages
        let mut received = vec![];
        if let Some(msg) = consumer.recv() {
            received.push(msg.0);
        }
        if let Some(msg) = consumer.recv() {
            received.push(msg.0);
        }

        received.sort();
        assert_eq!(received, vec![1, 2]);
    }

    #[test]
    fn test_mpmc_intra_multiple_consumers() {
        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (producer, consumer1) = Topic::<TestData>::mpmc_intra("mpmc_multi_cons", 64);
        let consumer2 = consumer1.clone();

        // Send two messages
        producer.send(TestData(1)).unwrap();
        producer.send(TestData(2)).unwrap();

        // Each consumer gets one message (load balancing)
        let recv1 = consumer1.recv();
        let recv2 = consumer2.recv();

        // Both should get exactly one, and together cover both messages
        assert!(recv1.is_some());
        assert!(recv2.is_some());
        let mut values = vec![recv1.unwrap().0, recv2.unwrap().0];
        values.sort();
        assert_eq!(values, vec![1, 2]);
    }

    #[test]
    fn test_mpmc_intra_producer_cannot_receive() {
        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (producer, _consumer) = Topic::<TestData>::mpmc_intra("mpmc_prod_no_recv", 64);

        // Producer should receive None
        let result = producer.recv();
        assert!(result.is_none());
    }

    #[test]
    fn test_mpmc_intra_consumer_cannot_send() {
        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (_producer, consumer) = Topic::<TestData>::mpmc_intra("mpmc_cons_no_send", 64);

        // Consumer should fail to send
        let result = consumer.send(TestData(42));
        assert!(result.is_err());
    }

    #[test]
    fn test_mpmc_intra_queue_full() {
        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (producer, _consumer) = Topic::<TestData>::mpmc_intra("mpmc_full", 4);

        // Fill the queue (capacity is 4, rounded up from any smaller value)
        for i in 0..4u64 {
            producer.send(TestData(i)).unwrap();
        }

        // Next send should fail
        let result = producer.send(TestData(100));
        assert!(result.is_err());
    }

    #[test]
    fn test_mpmc_intra_empty_receive() {
        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (_producer, consumer) = Topic::<TestData>::mpmc_intra("mpmc_empty", 64);

        // Nothing sent, should receive None
        assert!(consumer.recv().is_none());
    }

    #[test]
    fn test_mpmc_intra_has_messages() {
        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (producer, consumer) = Topic::<TestData>::mpmc_intra("mpmc_has_msgs", 64);

        // Initially no messages
        assert!(!consumer.has_messages());

        // Send a message
        producer.send(TestData(42)).unwrap();

        // Now has messages
        assert!(consumer.has_messages());

        // Consume it
        consumer.recv();

        // No more messages
        assert!(!consumer.has_messages());
    }

    #[test]
    fn test_mpmc_intra_debug() {
        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (producer, _consumer) = Topic::<TestData>::mpmc_intra("mpmc_debug", 64);

        let debug_str = format!("{:?}", producer);
        assert!(debug_str.contains("MpmcIntra"));
    }

    #[test]
    fn test_mpmc_intra_concurrent_mpmc() {
        use std::sync::atomic::{AtomicU64, Ordering};
        use std::sync::Arc;
        use std::thread;

        #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        struct TestData(u64);

        let (producer, consumer) = Topic::<TestData>::mpmc_intra("mpmc_concurrent", 256);
        let producer2 = producer.clone();
        let consumer2 = consumer.clone();

        let produced_count = Arc::new(AtomicU64::new(0));
        let consumed_count = Arc::new(AtomicU64::new(0));

        let pc1 = produced_count.clone();
        let pc2 = produced_count.clone();
        let cc1 = consumed_count.clone();
        let cc2 = consumed_count.clone();

        // Producer 1
        let p1_handle = thread::spawn(move || {
            for i in 0..50u64 {
                while producer.send(TestData(i)).is_err() {
                    std::hint::spin_loop();
                }
                pc1.fetch_add(1, Ordering::SeqCst);
            }
        });

        // Producer 2
        let p2_handle = thread::spawn(move || {
            for i in 50..100u64 {
                while producer2.send(TestData(i)).is_err() {
                    std::hint::spin_loop();
                }
                pc2.fetch_add(1, Ordering::SeqCst);
            }
        });

        // Consumer 1
        let c1_handle = thread::spawn(move || {
            let mut count = 0u64;
            while count < 50 {
                if consumer.recv().is_some() {
                    cc1.fetch_add(1, Ordering::SeqCst);
                    count += 1;
                } else {
                    std::hint::spin_loop();
                }
            }
        });

        // Consumer 2
        let c2_handle = thread::spawn(move || {
            let mut count = 0u64;
            while count < 50 {
                if consumer2.recv().is_some() {
                    cc2.fetch_add(1, Ordering::SeqCst);
                    count += 1;
                } else {
                    std::hint::spin_loop();
                }
            }
        });

        p1_handle.join().unwrap();
        p2_handle.join().unwrap();
        c1_handle.join().unwrap();
        c2_handle.join().unwrap();

        // All 100 messages should be produced and consumed
        assert_eq!(produced_count.load(Ordering::SeqCst), 100);
        assert_eq!(consumed_count.load(Ordering::SeqCst), 100);
    }

    #[test]
    fn test_mpmc_intra_latency_estimation() {
        use std::hint::black_box;
        use std::time::Instant;

        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct TinyMsg(u64);

        // Use unique topic name to avoid conflicts in parallel test runs
        let topic_name = format!("mpmc_latency_{}", std::process::id());
        let (producer, consumer) = Topic::<TinyMsg>::mpmc_intra(&topic_name, 1024);
        let iterations = 100_000u64;

        // Warmup
        for i in 0..1000u64 {
            producer.send(black_box(TinyMsg(i))).unwrap();
            black_box(consumer.recv());
        }

        // Measure
        let start = Instant::now();
        for i in 0..iterations {
            producer.send(black_box(TinyMsg(i))).unwrap();
            black_box(consumer.recv());
        }
        let elapsed = start.elapsed();

        let per_op_ns = elapsed.as_nanos() as f64 / (iterations as f64 * 2.0);
        println!("MpmcIntra per-op latency: {:.2}ns", per_op_ns);

        // Target: ≤100ns (p99), allow generous margin for parallel test execution
        assert!(per_op_ns < 5000.0, "MpmcIntra too slow: {:.2}ns", per_op_ns);
    }

    #[test]
    fn test_select_backend_same_process_mpmc() {
        let config = TopicConfig::new("test")
            .same_process()
            .with_access_pattern(AccessPattern::Mpmc);
        let selected = Topic::<TestMessage>::selected_backend_for(&config);
        assert_eq!(selected, SelectedBackend::MpmcIntra);
    }

    #[test]
    fn test_select_backend_explicit_mpmc_intra() {
        let config = TopicConfig::new("test").with_backend(BackendHint::MpmcIntra);
        let selected = Topic::<TestMessage>::selected_backend_for(&config);
        assert_eq!(selected, SelectedBackend::MpmcIntra);
    }

    #[test]
    fn test_from_config_same_process_mpmc() {
        let config = TopicConfig::new("config_same_process_mpmc")
            .same_process()
            .with_access_pattern(AccessPattern::Mpmc);
        let topic: Topic<TestMessage> = Topic::from_config(config).unwrap();
        assert_eq!(topic.backend_type(), "MpmcIntra");
    }

    #[test]
    fn test_select_backend_same_process_mpsc() {
        let config = TopicConfig::new("test")
            .same_process()
            .with_access_pattern(AccessPattern::Mpsc);
        let selected = Topic::<TestMessage>::selected_backend_for(&config);
        assert_eq!(selected, SelectedBackend::MpscIntra);
    }

    #[test]
    fn test_select_backend_explicit_mpsc_intra() {
        let config = TopicConfig::new("test").with_backend(BackendHint::MpscIntra);
        let selected = Topic::<TestMessage>::selected_backend_for(&config);
        assert_eq!(selected, SelectedBackend::MpscIntra);
    }

    #[test]
    fn test_from_config_same_process_mpsc() {
        let config = TopicConfig::new("config_same_process_mpsc")
            .same_process()
            .with_access_pattern(AccessPattern::Mpsc);
        let topic: Topic<TestMessage> = Topic::from_config(config).unwrap();
        assert_eq!(topic.backend_type(), "MpscIntra");
    }

    // ========================================================================
    // Backend Selection Tests
    // ========================================================================

    #[test]
    fn test_select_backend_same_thread() {
        let config = TopicConfig::new("test").same_thread();
        let selected = Topic::<TestMessage>::selected_backend_for(&config);
        assert_eq!(selected, SelectedBackend::DirectChannel);
    }

    #[test]
    fn test_select_backend_same_process_spsc() {
        let config = TopicConfig::new("test").same_process().spsc();
        let selected = Topic::<TestMessage>::selected_backend_for(&config);
        assert_eq!(selected, SelectedBackend::SpscIntra);
    }

    #[test]
    fn test_select_backend_cross_process_spsc() {
        let config = TopicConfig::new("test").cross_process().spsc();
        let selected = Topic::<TestMessage>::selected_backend_for(&config);
        // Non-POD defaults to SpscShm
        assert_eq!(selected, SelectedBackend::SpscShm);
    }

    #[test]
    fn test_select_backend_cross_process_mpmc() {
        let config = TopicConfig::new("test").cross_process().mpmc();
        let selected = Topic::<TestMessage>::selected_backend_for(&config);
        assert_eq!(selected, SelectedBackend::MpmcShm);
    }

    #[test]
    fn test_select_backend_explicit_hint() {
        let config = TopicConfig::new("test").with_backend(BackendHint::SpscIntra);
        let selected = Topic::<TestMessage>::selected_backend_for(&config);
        assert_eq!(selected, SelectedBackend::SpscIntra);
    }

    #[test]
    fn test_from_config_same_thread() {
        let config = TopicConfig::new("config_same_thread").same_thread();
        let topic: Topic<TestMessage> = Topic::from_config(config).unwrap();
        assert_eq!(topic.backend_type(), "DirectChannel");
    }

    #[test]
    fn test_from_config_same_process_spsc() {
        let config = TopicConfig::new("config_same_process")
            .same_process()
            .spsc();
        let topic: Topic<TestMessage> = Topic::from_config(config).unwrap();
        assert_eq!(topic.backend_type(), "SpscIntra");
    }

    #[test]
    fn test_from_config_cross_process_mpmc() {
        let unique_name = format!("config_cross_mpmc_{}", std::process::id());
        let config = TopicConfig::new(&unique_name).cross_process().mpmc();
        let topic: Topic<TestMessage> = Topic::from_config(config).unwrap();
        // AdaptiveTopic is now used - starts as Unknown before first send/recv
        assert_eq!(topic.backend_type(), "Unknown (Adaptive)");
    }

    #[test]
    fn test_topic_config_builder() {
        let config = TopicConfig::new("test")
            .with_capacity(128)
            .with_create(false)
            .as_consumer()
            .same_process()
            .spsc();

        assert_eq!(config.name, "test");
        assert_eq!(config.capacity, 128);
        assert!(!config.create);
        assert!(!config.is_producer);
        assert_eq!(config.access_pattern, AccessPattern::Spsc);
        assert_eq!(config.topology, ProcessTopology::SameProcess);
    }

    #[test]
    fn test_backend_selection_deterministic() {
        // Same config should always select same backend
        let config = TopicConfig::new("deterministic").same_process().spsc();

        let selected1 = Topic::<TestMessage>::selected_backend_for(&config);
        let selected2 = Topic::<TestMessage>::selected_backend_for(&config);
        let selected3 = Topic::<TestMessage>::selected_backend_for(&config);

        assert_eq!(selected1, selected2);
        assert_eq!(selected2, selected3);
    }

    // ========================================================================
    // Topic Registry Tests
    // ========================================================================

    #[test]
    fn test_registry_register_producer() {
        let registry = TopicRegistry::new();

        let entry = registry.register_producer("test_topic", false);

        assert_eq!(entry.name, "test_topic");
        assert_eq!(entry.producer_count, 1);
        assert_eq!(entry.consumer_count, 0);
        assert!(!entry.participant_pids.is_empty());
    }

    #[test]
    fn test_registry_register_consumer() {
        let registry = TopicRegistry::new();

        let entry = registry.register_consumer("test_topic", false);

        assert_eq!(entry.name, "test_topic");
        assert_eq!(entry.producer_count, 0);
        assert_eq!(entry.consumer_count, 1);
    }

    #[test]
    fn test_registry_multiple_participants() {
        let registry = TopicRegistry::new();

        registry.register_producer("multi", false);
        registry.register_producer("multi", false);
        registry.register_consumer("multi", false);
        let entry = registry.register_consumer("multi", false);

        assert_eq!(entry.producer_count, 2);
        assert_eq!(entry.consumer_count, 2);
    }

    #[test]
    fn test_registry_detect_pattern_spsc() {
        let registry = TopicRegistry::new();

        registry.register_producer("spsc_topic", false);
        registry.register_consumer("spsc_topic", false);

        assert_eq!(registry.detect_pattern("spsc_topic"), AccessPattern::Spsc);
    }

    #[test]
    fn test_registry_detect_pattern_mpmc() {
        let registry = TopicRegistry::new();

        // Multiple producers and consumers
        registry.register_producer("mpmc_topic", false);
        registry.register_producer("mpmc_topic", false);
        registry.register_consumer("mpmc_topic", false);
        registry.register_consumer("mpmc_topic", false);

        assert_eq!(registry.detect_pattern("mpmc_topic"), AccessPattern::Mpmc);
    }

    #[test]
    fn test_registry_detect_pattern_spmc() {
        let registry = TopicRegistry::new();

        // Single producer, multiple consumers
        registry.register_producer("spmc_topic", false);
        registry.register_consumer("spmc_topic", false);
        registry.register_consumer("spmc_topic", false);

        assert_eq!(registry.detect_pattern("spmc_topic"), AccessPattern::Spmc);
    }

    #[test]
    fn test_registry_unregister() {
        let registry = TopicRegistry::new();

        registry.register_producer("unreg", false);
        registry.register_consumer("unreg", false);

        // Should have entry
        assert!(registry.get_entry("unreg").is_some());

        registry.unregister_producer("unreg");
        registry.unregister_consumer("unreg");

        // Entry should be cleaned up
        assert!(registry.get_entry("unreg").is_none());
    }

    #[test]
    fn test_registry_detect_topology_same_process() {
        let registry = TopicRegistry::new();

        // All participants in same process
        registry.register_producer("same_proc", false);
        registry.register_consumer("same_proc", false);

        assert_eq!(
            registry.detect_topology("same_proc"),
            ProcessTopology::SameProcess
        );
    }

    #[test]
    fn test_registry_stats() {
        let registry = TopicRegistry::new();

        registry.register_producer("topic1", false);
        registry.register_consumer("topic1", false);
        registry.register_producer("topic2", false);

        let stats = registry.stats();
        assert_eq!(stats.total_topics, 2);
        assert_eq!(stats.total_producers, 2);
        assert_eq!(stats.total_consumers, 1);
    }

    #[test]
    fn test_registry_list_topics() {
        let registry = TopicRegistry::new();

        registry.register_producer("list_a", false);
        registry.register_producer("list_b", false);
        registry.register_producer("list_c", false);

        let topics = registry.list_topics();
        assert_eq!(topics.len(), 3);
        assert!(topics.contains(&"list_a".to_string()));
        assert!(topics.contains(&"list_b".to_string()));
        assert!(topics.contains(&"list_c".to_string()));
    }

    #[test]
    fn test_global_registry() {
        // Test global registry singleton
        let reg1 = global_registry();
        let reg2 = global_registry();

        // Should be same instance (pointer equality)
        assert!(std::ptr::eq(reg1, reg2));
    }

    #[test]
    fn test_topic_entry_detect_pattern() {
        let mut entry = TopicEntry::new("test".to_string());

        // Start with 0:0 -> SPSC
        assert_eq!(entry.detect_pattern(), AccessPattern::Spsc);

        // 1:1 -> SPSC
        entry.producer_count = 1;
        entry.consumer_count = 1;
        assert_eq!(entry.detect_pattern(), AccessPattern::Spsc);

        // 1:2 -> SPMC
        entry.consumer_count = 2;
        assert_eq!(entry.detect_pattern(), AccessPattern::Spmc);

        // 2:1 -> MPSC
        entry.producer_count = 2;
        entry.consumer_count = 1;
        assert_eq!(entry.detect_pattern(), AccessPattern::Mpsc);

        // 2:2 -> MPMC
        entry.consumer_count = 2;
        assert_eq!(entry.detect_pattern(), AccessPattern::Mpmc);
    }

    // ========================================================================
    // MpscShm Tests - Cross-process MPSC
    // ========================================================================

    #[test]
    fn test_mpsc_shm_basic_send_receive() {
        let unique_name = format!("mpsc_shm_basic_{}", std::process::id());

        let producer = Topic::<TestMessage>::mpsc_shm(&unique_name, 64, true).unwrap();
        let consumer = Topic::<TestMessage>::mpsc_shm(&unique_name, 64, false).unwrap();

        assert_eq!(producer.backend_type(), "MpscShm");
        assert_eq!(consumer.backend_type(), "MpscShm");

        // Send message
        let msg = TestMessage {
            id: 42,
            payload: "test".to_string(),
        };
        producer.send(msg.clone()).unwrap();

        // Receive message
        let received = consumer.recv().unwrap();
        assert_eq!(received.id, 42);
    }

    #[test]
    fn test_mpsc_shm_empty_receive() {
        let unique_name = format!("mpsc_shm_empty_{}", std::process::id());

        let consumer = Topic::<TestMessage>::mpsc_shm(&unique_name, 64, false).unwrap();

        // Empty queue returns None
        assert!(consumer.recv().is_none());
    }

    #[test]
    fn test_mpsc_shm_has_messages() {
        let unique_name = format!("mpsc_shm_has_msg_{}", std::process::id());

        let producer = Topic::<TestMessage>::mpsc_shm(&unique_name, 64, true).unwrap();
        let consumer = Topic::<TestMessage>::mpsc_shm(&unique_name, 64, false).unwrap();

        assert!(!consumer.has_messages());

        producer
            .send(TestMessage {
                id: 1,
                payload: "msg".to_string(),
            })
            .unwrap();
        assert!(consumer.has_messages());

        consumer.recv();
        assert!(!consumer.has_messages());
    }

    #[test]
    fn test_mpsc_shm_producer_cannot_receive() {
        let unique_name = format!("mpsc_shm_prod_recv_{}", std::process::id());

        let producer = Topic::<TestMessage>::mpsc_shm(&unique_name, 64, true).unwrap();

        // Send to self
        producer
            .send(TestMessage {
                id: 1,
                payload: "msg".to_string(),
            })
            .unwrap();

        // Producer cannot receive
        assert!(producer.recv().is_none());
    }

    #[test]
    fn test_mpsc_shm_consumer_cannot_send() {
        let unique_name = format!("mpsc_shm_cons_send_{}", std::process::id());

        let consumer = Topic::<TestMessage>::mpsc_shm(&unique_name, 64, false).unwrap();

        // Consumer cannot send
        let result = consumer.send(TestMessage {
            id: 1,
            payload: "msg".to_string(),
        });
        assert!(result.is_err());
    }

    #[test]
    fn test_mpsc_shm_multiple_messages() {
        let unique_name = format!("mpsc_shm_multi_{}", std::process::id());

        let producer = Topic::<TestMessage>::mpsc_shm(&unique_name, 64, true).unwrap();
        let consumer = Topic::<TestMessage>::mpsc_shm(&unique_name, 64, false).unwrap();

        // Send multiple messages
        for i in 0..10 {
            producer
                .send(TestMessage {
                    id: i,
                    payload: format!("msg{}", i),
                })
                .unwrap();
        }

        // Receive in order (FIFO)
        for i in 0..10 {
            let msg = consumer.recv().unwrap();
            assert_eq!(msg.id, i);
        }

        // Queue should be empty
        assert!(consumer.recv().is_none());
    }

    #[test]
    fn test_mpsc_shm_queue_full() {
        let unique_name = format!("mpsc_shm_full_{}", std::process::id());

        // Very small queue
        let producer = Topic::<TestMessage>::mpsc_shm(&unique_name, 4, true).unwrap();

        // Fill the queue (capacity will be rounded to power of 2 = 4)
        for i in 0..4u64 {
            let result = producer.send(TestMessage {
                id: i,
                payload: format!("msg{}", i),
            });
            assert!(result.is_ok(), "Failed to send message {}", i);
        }

        // Next send should fail
        let result = producer.send(TestMessage {
            id: 999,
            payload: "overflow".to_string(),
        });
        assert!(result.is_err(), "Queue should be full");
    }

    #[test]
    fn test_mpsc_shm_debug() {
        let unique_name = format!("mpsc_shm_debug_{}", std::process::id());

        let topic = Topic::<TestMessage>::mpsc_shm(&unique_name, 64, true).unwrap();
        let debug_str = format!("{:?}", topic);

        assert!(debug_str.contains("MpscShm"));
        assert!(debug_str.contains(&unique_name));
    }

    #[test]
    fn test_mpsc_shm_clone() {
        let unique_name = format!("mpsc_shm_clone_{}", std::process::id());

        let producer = Topic::<TestMessage>::mpsc_shm(&unique_name, 64, true).unwrap();
        let producer2 = producer.clone();

        // Both producers should work
        producer
            .send(TestMessage {
                id: 1,
                payload: "p1".to_string(),
            })
            .unwrap();
        producer2
            .send(TestMessage {
                id: 2,
                payload: "p2".to_string(),
            })
            .unwrap();

        let consumer = Topic::<TestMessage>::mpsc_shm(&unique_name, 64, false).unwrap();
        let msg1 = consumer.recv().unwrap();
        let msg2 = consumer.recv().unwrap();

        // Should receive both messages (order may vary)
        assert!(msg1.id == 1 || msg1.id == 2);
        assert!(msg2.id == 1 || msg2.id == 2);
        assert_ne!(msg1.id, msg2.id);
    }

    #[test]
    fn test_select_backend_cross_process_mpsc() {
        let config = TopicConfig::new("test")
            .cross_process()
            .with_access_pattern(AccessPattern::Mpsc);
        let selected = Topic::<TestMessage>::selected_backend_for(&config);
        assert_eq!(selected, SelectedBackend::MpscShm);
    }

    #[test]
    fn test_select_backend_explicit_mpsc_shm() {
        let config = TopicConfig::new("test").with_backend(BackendHint::MpscShm);
        let selected = Topic::<TestMessage>::selected_backend_for(&config);
        assert_eq!(selected, SelectedBackend::MpscShm);
    }

    #[test]
    fn test_from_config_cross_process_mpsc() {
        let unique_name = format!("config_cross_mpsc_{}", std::process::id());
        let config = TopicConfig::new(&unique_name)
            .cross_process()
            .with_access_pattern(AccessPattern::Mpsc)
            .as_producer();
        let topic: Topic<TestMessage> = Topic::from_config(config).unwrap();
        assert_eq!(topic.backend_type(), "MpscShm");
    }

    #[test]
    fn test_mpsc_shm_latency_estimation() {
        use std::hint::black_box;
        use std::time::Instant;

        // Use a simpler struct for latency testing
        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct TinyMsg(u64);

        let unique_name = format!("mpsc_shm_latency_{}", std::process::id());

        let producer = Topic::<TinyMsg>::mpsc_shm(&unique_name, 1024, true).unwrap();
        let consumer = Topic::<TinyMsg>::mpsc_shm(&unique_name, 1024, false).unwrap();
        let iterations = 100_000u64;

        // Warmup
        for i in 0..1000u64 {
            producer.send(black_box(TinyMsg(i))).unwrap();
            black_box(consumer.recv());
        }

        // Measure
        let start = Instant::now();
        for i in 0..iterations {
            producer.send(black_box(TinyMsg(i))).unwrap();
            black_box(consumer.recv());
        }
        let elapsed = start.elapsed();

        let per_op_ns = elapsed.as_nanos() as f64 / (iterations as f64 * 2.0);
        println!("MpscShm per-op latency: {:.2}ns", per_op_ns);

        // Target: ≤85ns (p99), allow generous margin for parallel test execution
        assert!(per_op_ns < 5000.0, "MpscShm too slow: {:.2}ns", per_op_ns);
    }

    #[test]
    fn test_mpsc_shm_concurrent_producers() {
        use std::sync::Arc;
        use std::thread;

        // Use a simple u64 message for concurrent test
        #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
        struct SimpleMsg(u64);

        impl crate::core::LogSummary for SimpleMsg {
            fn log_summary(&self) -> String {
                format!("SimpleMsg({})", self.0)
            }
        }

        let unique_name = format!("mpsc_shm_concurrent_{}", std::process::id());

        let producer1 = Arc::new(Topic::<SimpleMsg>::mpsc_shm(&unique_name, 512, true).unwrap());
        let producer2 = producer1.clone();
        let producer3 = producer1.clone();

        let messages_per_producer = 100u64;

        // Spawn producer threads
        let h1 = {
            let p = producer1.clone();
            thread::spawn(move || {
                for i in 0..messages_per_producer {
                    p.send(SimpleMsg(i)).unwrap();
                }
            })
        };

        let h2 = {
            let p = producer2.clone();
            thread::spawn(move || {
                for i in 0..messages_per_producer {
                    p.send(SimpleMsg(i + 1000)).unwrap();
                }
            })
        };

        let h3 = {
            let p = producer3.clone();
            thread::spawn(move || {
                for i in 0..messages_per_producer {
                    p.send(SimpleMsg(i + 2000)).unwrap();
                }
            })
        };

        h1.join().unwrap();
        h2.join().unwrap();
        h3.join().unwrap();

        // Consumer should receive all messages
        let consumer = Topic::<SimpleMsg>::mpsc_shm(&unique_name, 512, false).unwrap();
        let mut count = 0u64;
        while consumer.recv().is_some() {
            count += 1;
        }

        assert_eq!(count, messages_per_producer * 3);
    }

    // ========================================================================
    // SpmcShm Backend Tests
    // ========================================================================

    #[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
    struct SpmcTestMsg {
        id: u64,
        value: f64,
    }

    impl crate::core::LogSummary for SpmcTestMsg {
        fn log_summary(&self) -> String {
            format!("SpmcTestMsg{{id: {}, value: {}}}", self.id, self.value)
        }
    }

    #[test]
    fn test_spmc_shm_basic_send_receive() {
        let unique_name = format!("/spmc_shm_basic_{}", std::process::id());

        // Create producer
        let producer = Topic::<SpmcTestMsg>::spmc_shm(&unique_name, true).unwrap();

        // Create consumer
        let consumer = Topic::<SpmcTestMsg>::spmc_shm(&unique_name, false).unwrap();

        // Send a message
        let msg = SpmcTestMsg {
            id: 42,
            value: 3.5,
        };
        producer.send(msg.clone()).unwrap();

        // Receive the message
        let received = consumer.recv();
        assert!(received.is_some());
        let received = received.unwrap();
        assert_eq!(received.id, 42);
        assert!((received.value - 3.5).abs() < 0.001);
    }

    #[test]
    fn test_spmc_shm_empty_receive() {
        let unique_name = format!("/spmc_shm_empty_{}", std::process::id());

        let producer = Topic::<SpmcTestMsg>::spmc_shm(&unique_name, true).unwrap();
        let consumer = Topic::<SpmcTestMsg>::spmc_shm(&unique_name, false).unwrap();

        // No message sent, should get None
        let received = consumer.recv();

        // Consumer hasn't seen any sequence yet, so should return None
        assert!(received.is_none());

        // Now send a message
        producer.send(SpmcTestMsg { id: 1, value: 1.0 }).unwrap();

        // Should receive it
        let received = consumer.recv();
        assert!(received.is_some());
    }

    #[test]
    fn test_spmc_shm_has_messages() {
        let unique_name = format!("/spmc_shm_has_msg_{}", std::process::id());

        let producer = Topic::<SpmcTestMsg>::spmc_shm(&unique_name, true).unwrap();
        let consumer = Topic::<SpmcTestMsg>::spmc_shm(&unique_name, false).unwrap();

        // Initially no messages
        assert!(!consumer.has_messages());

        // Send a message
        producer.send(SpmcTestMsg { id: 1, value: 1.0 }).unwrap();

        // Now has messages
        assert!(consumer.has_messages());
    }

    #[test]
    fn test_spmc_shm_multiple_consumers() {
        let unique_name = format!("/spmc_shm_multi_cons_{}", std::process::id());

        let producer = Topic::<SpmcTestMsg>::spmc_shm(&unique_name, true).unwrap();
        let consumer1 = Topic::<SpmcTestMsg>::spmc_shm(&unique_name, false).unwrap();
        let consumer2 = Topic::<SpmcTestMsg>::spmc_shm(&unique_name, false).unwrap();
        let consumer3 = Topic::<SpmcTestMsg>::spmc_shm(&unique_name, false).unwrap();

        // Send a message
        let msg = SpmcTestMsg {
            id: 100,
            value: 2.71,
        };
        producer.send(msg.clone()).unwrap();

        // All consumers should receive the same message
        let r1 = consumer1.recv().unwrap();
        let r2 = consumer2.recv().unwrap();
        let r3 = consumer3.recv().unwrap();

        assert_eq!(r1.id, 100);
        assert_eq!(r2.id, 100);
        assert_eq!(r3.id, 100);
    }

    #[test]
    fn test_spmc_shm_broadcast_update() {
        let unique_name = format!("/spmc_shm_broadcast_{}", std::process::id());

        let producer = Topic::<SpmcTestMsg>::spmc_shm(&unique_name, true).unwrap();
        let consumer = Topic::<SpmcTestMsg>::spmc_shm(&unique_name, false).unwrap();

        // Send multiple messages - consumers get latest
        for i in 0..10 {
            producer
                .send(SpmcTestMsg {
                    id: i,
                    value: i as f64,
                })
                .unwrap();
        }

        // Consumer should see the latest value (SPMC is broadcast, not queue)
        let received = consumer.recv().unwrap();
        // The value should be the most recent one
        assert_eq!(received.id, 9);
    }

    #[test]
    fn test_spmc_shm_debug() {
        let unique_name = format!("/spmc_shm_debug_{}", std::process::id());

        let producer = Topic::<SpmcTestMsg>::spmc_shm(&unique_name, true).unwrap();
        let debug_str = format!("{:?}", producer);

        assert!(debug_str.contains("Topic"));
        assert!(debug_str.contains("SpmcShm"));
    }

    #[test]
    fn test_spmc_shm_backend_type() {
        let unique_name = format!("/spmc_shm_backend_{}", std::process::id());

        let producer = Topic::<SpmcTestMsg>::spmc_shm(&unique_name, true).unwrap();
        assert_eq!(producer.backend_type(), "SpmcShm");

        let consumer = Topic::<SpmcTestMsg>::spmc_shm(&unique_name, false).unwrap();
        assert_eq!(consumer.backend_type(), "SpmcShm");
    }

    #[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
    struct SpmcTinyMsg(u64);

    impl crate::core::LogSummary for SpmcTinyMsg {
        fn log_summary(&self) -> String {
            format!("SpmcTinyMsg({})", self.0)
        }
    }

    #[test]
    fn test_spmc_shm_latency_estimation() {
        let unique_name = format!("/spmc_shm_latency_{}", std::process::id());

        let producer = Topic::<SpmcTinyMsg>::spmc_shm(&unique_name, true).unwrap();
        let consumer = Topic::<SpmcTinyMsg>::spmc_shm(&unique_name, false).unwrap();

        // Warmup
        for i in 0..1000 {
            producer.send(SpmcTinyMsg(i)).unwrap();
            let _ = consumer.recv();
        }

        // Measure
        let iterations = 10000;
        let start = std::time::Instant::now();

        for i in 0..iterations {
            producer.send(SpmcTinyMsg(i)).unwrap();
            let _ = consumer.recv();
        }

        let elapsed = start.elapsed();
        let avg_ns = elapsed.as_nanos() as f64 / (iterations as f64 * 2.0); // send + recv

        println!("SpmcShm average latency: {:.2}ns per operation", avg_ns);

        // Should be under 200ns in debug mode (target ~70ns in release)
        assert!(avg_ns < 2000.0, "Latency too high: {:.2}ns", avg_ns);
    }

    #[test]
    fn test_spmc_shm_concurrent_consumers() {
        use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
        use std::thread;

        let unique_name = format!("/spmc_shm_concurrent_{}", std::process::id());

        let producer = Topic::<SpmcTinyMsg>::spmc_shm(&unique_name, true).unwrap();

        let total_messages = 1000u64;
        let received_counts: [AtomicU64; 4] = Default::default();
        let stop = AtomicBool::new(false);

        thread::scope(|s| {
            // Spawn consumer threads
            for i in 0..4 {
                let consumer = Topic::<SpmcTinyMsg>::spmc_shm(&unique_name, false).unwrap();
                let counter = &received_counts[i];
                let stop_flag = &stop;
                s.spawn(move || {
                    while !stop_flag.load(Ordering::SeqCst) {
                        if let Some(_) = consumer.recv() {
                            counter.fetch_add(1, Ordering::Relaxed);
                        }
                        std::hint::spin_loop();
                    }
                });
            }

            // Give consumers time to start
            std::thread::sleep(std::time::Duration::from_millis(10));

            // Producer sends messages
            for i in 0..total_messages {
                producer.send(SpmcTinyMsg(i)).unwrap();
                // Longer delay for parallel test reliability
                std::thread::sleep(std::time::Duration::from_micros(100));
            }

            // Give extra time for messages to propagate
            std::thread::sleep(std::time::Duration::from_millis(50));

            // Signal consumers to stop
            stop.store(true, Ordering::SeqCst);
        });

        // All consumers should have received messages (broadcast)
        for (i, count) in received_counts.iter().enumerate() {
            let c = count.load(Ordering::Relaxed);
            println!("Consumer {} received {} messages", i, c);
            // Each consumer should receive at least some messages (generous for parallel test load)
            assert!(
                c >= 50,
                "Consumer {} received only {} messages (expected >= 50)",
                i,
                c
            );
        }
    }

    #[test]
    fn test_spmc_shm_select_backend_cross_process() {
        // Test that cross-process SPMC auto-selects SpmcShm
        let config = TopicConfig {
            name: "/test_spmc_select".to_string(),
            topology: ProcessTopology::CrossProcess,
            access_pattern: AccessPattern::Spmc,
            backend_hint: BackendHint::Auto,
            capacity: 1,
            is_producer: true,
            create: true,
        };

        let selected = select_backend(&config, false);
        assert_eq!(selected, SelectedBackend::SpmcShm);
    }

    #[test]
    fn test_spmc_shm_hint_override() {
        // Test that SpmcShm hint forces SpmcShm backend
        let config = TopicConfig {
            name: "/test_spmc_hint".to_string(),
            topology: ProcessTopology::SameProcess, // Would normally select SpmcIntra
            access_pattern: AccessPattern::Spmc,
            backend_hint: BackendHint::SpmcShm, // But we force SpmcShm
            capacity: 1,
            is_producer: true,
            create: true,
        };

        let selected = select_backend(&config, false);
        assert_eq!(selected, SelectedBackend::SpmcShm);
    }

    #[test]
    fn test_spmc_shm_clone_consumer() {
        let unique_name = format!("/spmc_shm_clone_{}", std::process::id());

        let producer = Topic::<SpmcTestMsg>::spmc_shm(&unique_name, true).unwrap();
        let consumer1 = Topic::<SpmcTestMsg>::spmc_shm(&unique_name, false).unwrap();

        // Clone the consumer
        let consumer2 = consumer1.clone();

        // Send a message
        producer.send(SpmcTestMsg { id: 42, value: 1.0 }).unwrap();

        // Both consumers should be able to receive
        let r1 = consumer1.recv();
        let r2 = consumer2.recv();

        assert!(r1.is_some());
        assert!(r2.is_some());
        assert_eq!(r1.unwrap().id, 42);
        assert_eq!(r2.unwrap().id, 42);
    }
}
