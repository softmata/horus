//! # Discovery System
//!
//! Local single-machine discovery of HORUS nodes and topics.
//!
//! ## Architecture
//!
//! Discovery uses two complementary data sources:
//!
//! 1. **Presence files** (`shm_nodes_dir()/{name}.json`) — written by each
//!    node at startup, removed at shutdown.  Contains PID, scheduler, topics,
//!    priority, rate, health, and PID start time (for reuse detection).
//!
//! 2. **SHM topic files** (`shm_topics_dir()/`) — memory-mapped files
//!    created by the Topic API.  Contains message data, sequence counters,
//!    and participant tables.
//!
//! Results from both sources are merged: SHM provides size/rate metadata,
//! presence provides pub/sub relationships.  A [`DiscoveryCache`] with
//! configurable TTL avoids redundant filesystem scans.
//!
//! ## Known Limitations
//!
//! - **Local only**: no network discovery of remote HORUS instances.
//! - **16 participant limit**: each SHM topic supports at most 16 concurrent
//!   publishers/subscribers (expired leases are reclaimed automatically).
//! - **CPU first-sample**: the first CPU reading uses lifetime average rather
//!   than instantaneous delta.
//! - **No inotify**: discovery polls the filesystem on each cache miss rather
//!   than using file-change notifications.

pub(crate) mod nodes;
pub(crate) mod topics;

use horus_core::core::HealthStatus;
#[allow(unused_imports)] // used by discovery/tests.rs for SHM presence mocking
use horus_core::core::NodePresence;
use horus_core::error::HorusResult;
use horus_core::memory::shm_topics_dir;
use std::path::Path;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};
use horus_core::core::DurationExt;

// Data structures for comprehensive monitoring
#[derive(Debug, Clone)]
pub struct NodeStatus {
    pub name: String,
    pub status: String,
    pub health: HealthStatus,
    pub priority: u32,
    pub process_id: u32,
    pub command_line: String,
    pub working_dir: String,
    pub cpu_usage: f32,
    pub memory_usage: u64,
    pub start_time: String,
    pub scheduler_name: String,
    pub category: ProcessCategory,
    pub tick_count: u64,
    pub error_count: u32,
    pub actual_rate_hz: u32,
    pub publishers: Vec<TopicInfo>,
    pub subscribers: Vec<TopicInfo>,
    /// Live tick count from SHM SchedulerRegistry (None if registry unavailable).
    pub live_tick_count: Option<u64>,
    /// Live health from SHM SchedulerRegistry (None if registry unavailable).
    pub live_health: Option<u8>,
    /// Live average tick duration (ns) from registry.
    pub live_avg_tick_ns: Option<u64>,
    /// Live max tick duration (ns) from registry.
    pub live_max_tick_ns: Option<u64>,
    /// Live budget misses from registry.
    pub live_budget_misses: Option<u32>,
    /// Live deadline misses from registry.
    pub live_deadline_misses: Option<u32>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ProcessCategory {
    Node, // Runtime scheduler nodes
    Tool, // GUI applications
    CLI,  // Command line tools
}

/// Topic lifecycle status for monitoring
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopicStatus {
    /// Active: has live processes AND recent writes (within 30 seconds)
    Active,
    /// Idle: has live processes but no recent writes (30s - 5min)
    Idle,
    /// Stale: no live processes OR very old (5+ minutes without activity)
    Stale,
}

#[derive(Debug, Clone)]
pub struct SharedMemoryInfo {
    pub topic_name: String,
    pub size_bytes: u64,
    pub active: bool,
    pub accessing_processes: Vec<u32>,
    pub last_modified: Option<std::time::SystemTime>,
    pub message_type: Option<String>,
    pub publishers: Vec<String>,
    pub subscribers: Vec<String>,
    pub message_rate_hz: f32,
    /// Topic lifecycle status (Active, Idle, or Stale)
    pub status: TopicStatus,
    /// Human-readable age string (e.g., "2s ago", "5m ago", "1h ago")
    pub age_string: String,
    /// Whether this is an internal system topic (e.g., horus.ctl.*)
    pub is_system: bool,
    /// Total messages ever sent on this topic (from TopicHeader, always-on).
    pub messages_total: u64,
    /// Topic kind classification (from TopicHeader).
    pub topic_kind: u8,
}

// Enhanced node status with pub/sub info
#[derive(Debug, Clone)]
pub struct TopicInfo {
    pub topic: String,
    pub type_name: String,
}

// Fast discovery cache to avoid expensive filesystem operations
#[derive(Clone)]
struct DiscoveryCache {
    nodes: Vec<NodeStatus>,
    shared_memory: Vec<SharedMemoryInfo>,
    nodes_last_updated: Instant,
    shared_memory_last_updated: Instant,
    cache_duration: Duration,
}

impl DiscoveryCache {
    fn new() -> Self {
        let initial_time = Instant::now() - 10_u64.secs(); // Force initial update
        Self {
            nodes: Vec::new(),
            shared_memory: Vec::new(),
            nodes_last_updated: initial_time,
            shared_memory_last_updated: initial_time,
            cache_duration: crate::config::DISCOVERY_CACHE_MS.ms(),
        }
    }

    fn is_nodes_stale(&self) -> bool {
        self.nodes_last_updated.elapsed() > self.cache_duration
    }

    fn is_shared_memory_stale(&self) -> bool {
        self.shared_memory_last_updated.elapsed() > self.cache_duration
    }

    fn update_nodes(&mut self, nodes: Vec<NodeStatus>) {
        self.nodes = nodes;
        self.nodes_last_updated = Instant::now();
    }

    fn update_shared_memory(&mut self, shm: Vec<SharedMemoryInfo>) {
        self.shared_memory = shm;
        self.shared_memory_last_updated = Instant::now();
    }
}

// Global cache instance
lazy_static::lazy_static! {
    static ref DISCOVERY_CACHE: Arc<RwLock<DiscoveryCache>> = Arc::new(RwLock::new(DiscoveryCache::new()));
}

/// Process information — delegated to [`horus_sys::discover::ProcessInfo`].
#[cfg(test)]
pub(crate) type ProcessInfo = horus_sys::discover::ProcessInfo;

pub fn discover_nodes() -> HorusResult<Vec<NodeStatus>> {
    // Check cache first
    if let Ok(cache) = DISCOVERY_CACHE.read() {
        if !cache.is_nodes_stale() {
            return Ok(cache.nodes.clone());
        }
    }

    // Cache is stale - do synchronous update for immediate detection
    let nodes = nodes::discover_nodes_uncached()?;

    // Update cache with fresh data
    if let Ok(mut cache) = DISCOVERY_CACHE.write() {
        cache.update_nodes(nodes.clone());
    }

    Ok(nodes)
}

pub fn discover_shared_memory() -> HorusResult<Vec<SharedMemoryInfo>> {
    // Check cache first
    if let Ok(cache) = DISCOVERY_CACHE.read() {
        if !cache.is_shared_memory_stale() {
            return Ok(cache.shared_memory.clone());
        }
    }

    // Cache is stale - do synchronous update for immediate detection
    let shared_memory = topics::discover_shared_memory_uncached()?;

    // Update cache with fresh data
    if let Ok(mut cache) = DISCOVERY_CACHE.write() {
        cache.update_shared_memory(shared_memory.clone());
    }

    Ok(shared_memory)
}

/// Clean up stale topic files from shared memory
pub fn cleanup_stale_topics() {
    let topics_path = shm_topics_dir();
    if topics_path.exists() {
        topics::cleanup_stale_topics_in_dir(&topics_path);
    }
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod dispatch_tests {
    use super::*;

    // ── Type construction ───────────────────────────────────────────────

    #[test]
    fn topic_status_active_variant() {
        let s = TopicStatus::Active;
        assert_eq!(s, TopicStatus::Active);
    }

    #[test]
    fn topic_status_idle_variant() {
        let s = TopicStatus::Idle;
        assert_eq!(s, TopicStatus::Idle);
    }

    #[test]
    fn topic_status_stale_variant() {
        let s = TopicStatus::Stale;
        assert_eq!(s, TopicStatus::Stale);
    }

    #[test]
    fn topic_status_all_distinct() {
        assert_ne!(TopicStatus::Active, TopicStatus::Idle);
        assert_ne!(TopicStatus::Active, TopicStatus::Stale);
        assert_ne!(TopicStatus::Idle, TopicStatus::Stale);
    }

    #[test]
    fn process_category_variants() {
        assert_eq!(ProcessCategory::Node, ProcessCategory::Node);
        assert_ne!(ProcessCategory::Node, ProcessCategory::Tool);
        assert_ne!(ProcessCategory::Node, ProcessCategory::CLI);
        assert_ne!(ProcessCategory::Tool, ProcessCategory::CLI);
    }

    #[test]
    fn topic_info_construction() {
        let info = TopicInfo {
            topic: "cmd_vel".to_string(),
            type_name: "CmdVel".to_string(),
        };
        assert_eq!(info.topic, "cmd_vel");
        assert_eq!(info.type_name, "CmdVel");
    }

    // ── DiscoveryCache ──────────────────────────────────────────────────

    #[test]
    fn discovery_cache_new_is_stale() {
        let cache = DiscoveryCache::new();
        // New cache should be stale (forces first fetch)
        assert!(cache.is_nodes_stale());
        assert!(cache.is_shared_memory_stale());
    }

    #[test]
    fn discovery_cache_fresh_after_update() {
        let mut cache = DiscoveryCache::new();
        cache.update_nodes(vec![]);
        assert!(!cache.is_nodes_stale());
    }

    #[test]
    fn discovery_cache_shm_fresh_after_update() {
        let mut cache = DiscoveryCache::new();
        cache.update_shared_memory(vec![]);
        assert!(!cache.is_shared_memory_stale());
    }

    #[test]
    fn discovery_cache_stores_data() {
        let mut cache = DiscoveryCache::new();
        let shm = vec![SharedMemoryInfo {
            topic_name: "test_topic".to_string(),
            size_bytes: 4096,
            active: true,
            accessing_processes: vec![1234],
            last_modified: None,
            message_type: Some("CmdVel".to_string()),
            publishers: vec!["node_a".to_string()],
            subscribers: vec!["node_b".to_string()],
            message_rate_hz: 100.0,
            status: TopicStatus::Active,
            age_string: "just now".to_string(),
            is_system: false,
            messages_total: 42,
            topic_kind: 0,
        }];
        cache.update_shared_memory(shm);
        assert_eq!(cache.shared_memory.len(), 1);
        assert_eq!(cache.shared_memory[0].topic_name, "test_topic");
    }

    // ── Dispatch functions ──────────────────────────────────────────────

    #[test]
    fn discover_nodes_returns_ok() {
        // On a clean system (no running HORUS), should return Ok
        let result = discover_nodes();
        assert!(result.is_ok());
    }

    #[test]
    fn discover_shared_memory_returns_ok() {
        let result = discover_shared_memory();
        assert!(result.is_ok());
    }

    #[test]
    fn cleanup_stale_topics_does_not_panic() {
        // Should not panic even with no SHM directory
        cleanup_stale_topics();
    }
}
