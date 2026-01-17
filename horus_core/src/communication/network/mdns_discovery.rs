//! High-Level Discovery API for HORUS Nodes
//!
//! This module provides a user-friendly API for discovering HORUS nodes on the network
//! using mDNS/DNS-SD. It builds on the low-level `HorusMdns` module to provide:
//!
//! - Easy-to-use `discover()` function for one-shot discovery
//! - `DiscoveredNode` struct with human-readable fields
//! - Continuous monitoring via `watch()` for real-time updates
//! - JSON serialization support for CLI tooling
//! - Filtering by topic or capability
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::communication::network::mdns_discovery::{discover, DiscoveredNode};
//!
//! // One-shot discovery (2 second scan)
//! let nodes = discover()?;
//! for node in nodes {
//!     println!("{}: {}:{} - {:?}", node.name, node.ip, node.port, node.topics);
//! }
//!
//! // With custom options
//! let nodes = discover_with_options(DiscoveryOptions {
//!     timeout: Duration::from_secs(5),
//!     filter_topic: Some("lidar".to_string()),
//!     ..Default::default()
//! })?;
//!
//! // Continuous monitoring
//! let watcher = watch()?;
//! for event in watcher {
//!     match event {
//!         DiscoveryEvent::NodeJoined(node) => println!("+ {}", node.name),
//!         DiscoveryEvent::NodeLeft(name) => println!("- {}", name),
//!     }
//! }
//! ```

use crate::error::HorusResult;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::{mpsc, Arc, RwLock};
use std::thread;
use std::time::{Duration, Instant, SystemTime};

use super::mdns::{HorusMdns, ServiceInfo, BROWSE_TIMEOUT};

/// A discovered HORUS node on the network
///
/// This struct provides a user-friendly view of a discovered service,
/// with fields formatted for display and JSON serialization.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiscoveredNode {
    /// Human-readable node name (e.g., "arm-controller")
    pub name: String,
    /// Full hostname with .local suffix (e.g., "arm-controller.local")
    pub hostname: String,
    /// IP address of the node
    pub ip: String,
    /// Port number the node is listening on
    pub port: u16,
    /// List of topics this node publishes/subscribes to
    pub topics: Vec<String>,
    /// When the node was discovered (ISO 8601 timestamp)
    pub discovered_at: String,
    /// Time since discovery in human-readable format
    #[serde(skip_serializing_if = "Option::is_none")]
    pub uptime: Option<String>,
    /// HORUS version running on this node (if advertised)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,
}

impl DiscoveredNode {
    /// Create from a ServiceInfo
    fn from_service_info(info: &ServiceInfo) -> Self {
        let now = SystemTime::now();
        let timestamp = now
            .duration_since(SystemTime::UNIX_EPOCH)
            .map(|d| {
                chrono::DateTime::from_timestamp(d.as_secs() as i64, 0)
                    .map(|dt| dt.format("%Y-%m-%dT%H:%M:%SZ").to_string())
                    .unwrap_or_else(|| "unknown".to_string())
            })
            .unwrap_or_else(|_| "unknown".to_string());

        Self {
            name: info.instance_name.clone(),
            hostname: info.hostname.clone(),
            ip: info.address.to_string(),
            port: info.port,
            topics: info.topics.clone(),
            discovered_at: timestamp,
            uptime: None,
            version: None,
        }
    }
}

/// Options for discovery operations
#[derive(Debug, Clone)]
pub struct DiscoveryOptions {
    /// How long to scan for nodes (default: 2 seconds)
    pub timeout: Duration,
    /// Filter to only nodes publishing this topic
    pub filter_topic: Option<String>,
    /// Filter to only nodes whose name contains this string
    pub filter_name: Option<String>,
    /// Include offline/stale nodes from cache
    pub include_cached: bool,
}

impl Default for DiscoveryOptions {
    fn default() -> Self {
        Self {
            timeout: BROWSE_TIMEOUT,
            filter_topic: None,
            filter_name: None,
            include_cached: false,
        }
    }
}

/// Events emitted by the discovery watcher
#[derive(Debug, Clone)]
pub enum DiscoveryEvent {
    /// A new node was discovered
    NodeJoined(DiscoveredNode),
    /// A node left the network (identified by name)
    NodeLeft(String),
    /// A node's information was updated
    NodeUpdated(DiscoveredNode),
    /// Discovery error occurred
    Error(String),
}

/// Result of a discovery operation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiscoveryResult {
    /// List of discovered nodes
    pub nodes: Vec<DiscoveredNode>,
    /// Number of nodes found
    pub count: usize,
    /// How long the scan took
    pub scan_duration_ms: u64,
    /// Timestamp of the scan
    pub timestamp: String,
}

/// Discover all HORUS nodes on the local network
///
/// Performs a one-shot scan using mDNS/DNS-SD and returns all discovered nodes.
/// Uses the default timeout of 2 seconds.
///
/// # Returns
/// A list of discovered nodes, or an error if discovery failed.
///
/// # Example
/// ```rust,ignore
/// let nodes = discover()?;
/// println!("Found {} nodes", nodes.len());
/// ```
pub fn discover() -> HorusResult<Vec<DiscoveredNode>> {
    discover_with_options(DiscoveryOptions::default())
}

/// Discover nodes with custom options
///
/// # Arguments
/// * `options` - Discovery options including timeout, filters, etc.
///
/// # Returns
/// A list of discovered nodes matching the filter criteria.
pub fn discover_with_options(options: DiscoveryOptions) -> HorusResult<Vec<DiscoveredNode>> {
    let mdns = HorusMdns::new()?;
    let services = mdns.browse_services_with_timeout(options.timeout)?;

    let mut nodes: Vec<DiscoveredNode> = services
        .iter()
        .map(DiscoveredNode::from_service_info)
        .collect();

    // Apply filters
    if let Some(ref topic) = options.filter_topic {
        nodes.retain(|n| n.topics.iter().any(|t| t.contains(topic)));
    }

    if let Some(ref name) = options.filter_name {
        nodes.retain(|n| n.name.contains(name) || n.hostname.contains(name));
    }

    Ok(nodes)
}

/// Discover nodes and return a full result with metadata
///
/// This returns more detailed information including scan duration
/// and timestamps, suitable for JSON output.
pub fn discover_full() -> HorusResult<DiscoveryResult> {
    discover_full_with_options(DiscoveryOptions::default())
}

/// Discover nodes with full result and custom options
pub fn discover_full_with_options(options: DiscoveryOptions) -> HorusResult<DiscoveryResult> {
    let start = Instant::now();
    let nodes = discover_with_options(options)?;
    let duration = start.elapsed();

    let now = SystemTime::now();
    let timestamp = now
        .duration_since(SystemTime::UNIX_EPOCH)
        .map(|d| {
            chrono::DateTime::from_timestamp(d.as_secs() as i64, 0)
                .map(|dt| dt.format("%Y-%m-%dT%H:%M:%SZ").to_string())
                .unwrap_or_else(|| "unknown".to_string())
        })
        .unwrap_or_else(|_| "unknown".to_string());

    Ok(DiscoveryResult {
        count: nodes.len(),
        nodes,
        scan_duration_ms: duration.as_millis() as u64,
        timestamp,
    })
}

/// Watcher for continuous node discovery
///
/// Monitors the network for nodes joining and leaving, emitting events
/// via a channel. The watcher runs in a background thread.
pub struct DiscoveryWatcher {
    /// Receiver for discovery events
    receiver: mpsc::Receiver<DiscoveryEvent>,
    /// Handle to stop the watcher thread
    stop_signal: Arc<RwLock<bool>>,
    /// Thread handle
    thread: Option<thread::JoinHandle<()>>,
}

impl DiscoveryWatcher {
    /// Create a new watcher with default scan interval (1 second)
    pub fn new() -> HorusResult<Self> {
        Self::with_interval(Duration::from_secs(1))
    }

    /// Create a watcher with a custom scan interval
    pub fn with_interval(scan_interval: Duration) -> HorusResult<Self> {
        let (sender, receiver) = mpsc::channel();
        let stop_signal = Arc::new(RwLock::new(false));
        let stop_clone = stop_signal.clone();

        let thread = thread::spawn(move || {
            let mdns = match HorusMdns::new() {
                Ok(m) => m,
                Err(e) => {
                    let _ = sender.send(DiscoveryEvent::Error(format!(
                        "Failed to create mDNS: {}",
                        e
                    )));
                    return;
                }
            };

            let mut known_nodes: HashMap<String, DiscoveredNode> = HashMap::new();

            loop {
                // Check stop signal
                if *stop_clone.read().unwrap() {
                    break;
                }

                // Scan for services
                match mdns.browse_services_with_timeout(scan_interval) {
                    Ok(services) => {
                        let current_names: std::collections::HashSet<String> =
                            services.iter().map(|s| s.instance_name.clone()).collect();

                        // Check for new or updated nodes
                        for service in &services {
                            let node = DiscoveredNode::from_service_info(service);

                            if let Some(existing) = known_nodes.get(&service.instance_name) {
                                // Check if anything changed
                                if existing.ip != node.ip
                                    || existing.port != node.port
                                    || existing.topics != node.topics
                                {
                                    let _ = sender.send(DiscoveryEvent::NodeUpdated(node.clone()));
                                    known_nodes.insert(service.instance_name.clone(), node);
                                }
                            } else {
                                // New node
                                let _ = sender.send(DiscoveryEvent::NodeJoined(node.clone()));
                                known_nodes.insert(service.instance_name.clone(), node);
                            }
                        }

                        // Check for removed nodes
                        let removed: Vec<String> = known_nodes
                            .keys()
                            .filter(|name| !current_names.contains(*name))
                            .cloned()
                            .collect();

                        for name in removed {
                            known_nodes.remove(&name);
                            let _ = sender.send(DiscoveryEvent::NodeLeft(name));
                        }
                    }
                    Err(e) => {
                        let _ = sender.send(DiscoveryEvent::Error(format!(
                            "Browse failed: {}",
                            e
                        )));
                    }
                }

                // Small sleep between scans
                thread::sleep(Duration::from_millis(100));
            }
        });

        Ok(Self {
            receiver,
            stop_signal,
            thread: Some(thread),
        })
    }

    /// Stop the watcher
    pub fn stop(&mut self) {
        *self.stop_signal.write().unwrap() = true;
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }

    /// Try to receive an event without blocking
    pub fn try_recv(&self) -> Option<DiscoveryEvent> {
        self.receiver.try_recv().ok()
    }

    /// Receive the next event, blocking until available
    pub fn recv(&self) -> Option<DiscoveryEvent> {
        self.receiver.recv().ok()
    }

    /// Receive with a timeout
    pub fn recv_timeout(&self, timeout: Duration) -> Option<DiscoveryEvent> {
        self.receiver.recv_timeout(timeout).ok()
    }
}

impl Drop for DiscoveryWatcher {
    fn drop(&mut self) {
        self.stop();
    }
}

impl Iterator for DiscoveryWatcher {
    type Item = DiscoveryEvent;

    fn next(&mut self) -> Option<Self::Item> {
        self.recv()
    }
}

/// Start watching for nodes on the network
///
/// Returns a watcher that can be iterated to receive discovery events.
///
/// # Example
/// ```rust,ignore
/// let watcher = watch()?;
/// for event in watcher.take(10) {  // Get 10 events then stop
///     match event {
///         DiscoveryEvent::NodeJoined(node) => println!("+ {}", node.name),
///         DiscoveryEvent::NodeLeft(name) => println!("- {}", name),
///         _ => {}
///     }
/// }
/// ```
pub fn watch() -> HorusResult<DiscoveryWatcher> {
    DiscoveryWatcher::new()
}

/// Start watching with a custom scan interval
pub fn watch_with_interval(interval: Duration) -> HorusResult<DiscoveryWatcher> {
    DiscoveryWatcher::with_interval(interval)
}

/// Find nodes that publish a specific topic
///
/// Convenience function that scans and filters by topic in one call.
pub fn find_nodes_with_topic(topic: &str) -> HorusResult<Vec<DiscoveredNode>> {
    discover_with_options(DiscoveryOptions {
        filter_topic: Some(topic.to_string()),
        ..Default::default()
    })
}

/// Find a specific node by name
///
/// Returns the first node whose name matches exactly.
pub fn find_node(name: &str) -> HorusResult<Option<DiscoveredNode>> {
    let nodes = discover_with_options(DiscoveryOptions {
        filter_name: Some(name.to_string()),
        ..Default::default()
    })?;

    Ok(nodes.into_iter().find(|n| n.name == name))
}

/// Convert discovery result to JSON string
pub fn to_json(result: &DiscoveryResult) -> HorusResult<String> {
    serde_json::to_string_pretty(result)
        .map_err(|e| crate::error::HorusError::Communication(format!("JSON error: {}", e)))
}

/// Convert node list to JSON string
pub fn nodes_to_json(nodes: &[DiscoveredNode]) -> HorusResult<String> {
    serde_json::to_string_pretty(nodes)
        .map_err(|e| crate::error::HorusError::Communication(format!("JSON error: {}", e)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_discovery_options_default() {
        let options = DiscoveryOptions::default();
        assert_eq!(options.timeout, BROWSE_TIMEOUT);
        assert!(options.filter_topic.is_none());
        assert!(options.filter_name.is_none());
        assert!(!options.include_cached);
    }

    #[test]
    fn test_discovered_node_serialization() {
        let node = DiscoveredNode {
            name: "test-robot".to_string(),
            hostname: "test-robot.local".to_string(),
            ip: "192.168.1.100".to_string(),
            port: 9870,
            topics: vec!["lidar".to_string(), "camera".to_string()],
            discovered_at: "2024-01-01T00:00:00Z".to_string(),
            uptime: None,
            version: Some("0.1.7".to_string()),
        };

        let json = serde_json::to_string(&node).unwrap();
        assert!(json.contains("test-robot"));
        assert!(json.contains("192.168.1.100"));
        assert!(json.contains("lidar"));

        // Deserialize back
        let parsed: DiscoveredNode = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.name, "test-robot");
        assert_eq!(parsed.port, 9870);
    }

    #[test]
    fn test_discovery_result_serialization() {
        let result = DiscoveryResult {
            nodes: vec![],
            count: 0,
            scan_duration_ms: 2000,
            timestamp: "2024-01-01T00:00:00Z".to_string(),
        };

        let json = serde_json::to_string_pretty(&result).unwrap();
        assert!(json.contains("scan_duration_ms"));
        assert!(json.contains("2000"));
    }

    // Integration tests require network access
    #[test]
    #[ignore]
    fn test_discover_integration() {
        let result = discover();
        assert!(result.is_ok());
    }

    #[test]
    #[ignore]
    fn test_watch_integration() {
        let watcher = watch().unwrap();

        // Wait for a few events or timeout
        let _event = watcher.recv_timeout(Duration::from_secs(3));

        // Should either get an event or timeout (no crash)
        drop(watcher);
    }
}
