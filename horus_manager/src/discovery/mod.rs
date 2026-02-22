pub(crate) mod nodes;
pub(crate) mod topics;

use horus_core::core::{HealthStatus, NetworkStatus};
use horus_core::error::HorusResult;
use horus_core::memory::{shm_network_dir, shm_topics_dir};
use horus_core::NodePresence;
use std::path::Path;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

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

impl TopicStatus {
    /// Get a display symbol for the status
    pub fn symbol(&self) -> &'static str {
        match self {
            TopicStatus::Active => "●",
            TopicStatus::Idle => "◐",
            TopicStatus::Stale => "○",
        }
    }

    /// Get a description of the status
    pub fn description(&self) -> &'static str {
        match self {
            TopicStatus::Active => "active",
            TopicStatus::Idle => "idle",
            TopicStatus::Stale => "stale",
        }
    }
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
    // Separate timestamps for nodes and shared_memory to prevent cross-contamination
    nodes_last_updated: Instant,
    shared_memory_last_updated: Instant,
    cache_duration: Duration,
}

impl DiscoveryCache {
    fn new() -> Self {
        let initial_time = Instant::now() - Duration::from_secs(10); // Force initial update
        Self {
            nodes: Vec::new(),
            shared_memory: Vec::new(),
            nodes_last_updated: initial_time,
            shared_memory_last_updated: initial_time,
            cache_duration: Duration::from_millis(crate::config::DISCOVERY_CACHE_MS),
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

#[derive(Debug, Default)]
pub(crate) struct ProcessInfo {
    pub(crate) _pid: u32,
    pub(crate) _name: String,
    pub(crate) cmdline: String,
    pub(crate) working_dir: String,
    pub(crate) cpu_percent: f32,
    pub(crate) memory_kb: u64,
    pub(crate) start_time: String,
}

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

// ============================================================================
// Network Status Discovery
// ============================================================================

/// Discover network transport status for all nodes
///
/// Reads network status files from /dev/shm/horus/network/ directory.
/// These files are written by nodes when they use network transports.
pub fn discover_network_status() -> HorusResult<Vec<NetworkStatus>> {
    let network_dir = shm_network_dir();
    if !network_dir.exists() {
        return Ok(Vec::new());
    }

    let mut statuses = Vec::new();
    if let Ok(entries) = std::fs::read_dir(&network_dir) {
        for entry in entries.flatten() {
            if let Ok(content) = std::fs::read_to_string(entry.path()) {
                if let Ok(status) = serde_json::from_str::<NetworkStatus>(&content) {
                    // Only include fresh statuses (within last 30 seconds)
                    if status.is_fresh(30) {
                        statuses.push(status);
                    }
                }
            }
        }
    }

    Ok(statuses)
}

/// Get aggregated network statistics across all nodes
pub fn get_network_summary() -> NetworkSummary {
    let statuses = discover_network_status().unwrap_or_default();

    let mut summary = NetworkSummary::default();
    let mut transport_counts: std::collections::HashMap<String, u32> =
        std::collections::HashMap::new();

    for status in &statuses {
        summary.total_nodes += 1;
        summary.total_bytes_sent += status.bytes_sent;
        summary.total_bytes_received += status.bytes_received;
        summary.total_packets_sent += status.packets_sent;
        summary.total_packets_received += status.packets_received;

        *transport_counts
            .entry(status.transport_type.clone())
            .or_insert(0) += 1;

        for endpoint in &status.remote_endpoints {
            if !summary.unique_endpoints.contains(endpoint) {
                summary.unique_endpoints.push(endpoint.clone());
            }
        }
    }

    summary.transport_breakdown = transport_counts;
    summary.node_statuses = statuses;
    summary
}

/// Summary of network activity across all HORUS nodes
#[derive(Debug, Clone, Default)]
pub struct NetworkSummary {
    /// Total nodes with network status
    pub total_nodes: u32,
    /// Total bytes sent across all nodes
    pub total_bytes_sent: u64,
    /// Total bytes received across all nodes
    pub total_bytes_received: u64,
    /// Total packets sent
    pub total_packets_sent: u64,
    /// Total packets received
    pub total_packets_received: u64,
    /// Breakdown by transport type (e.g., "Udp": 3, "SharedMemory": 5)
    pub transport_breakdown: std::collections::HashMap<String, u32>,
    /// Unique remote endpoints discovered
    pub unique_endpoints: Vec<String>,
    /// Individual node statuses
    pub node_statuses: Vec<NetworkStatus>,
}

#[cfg(test)]
mod tests;
