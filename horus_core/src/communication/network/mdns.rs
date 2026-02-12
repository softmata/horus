//! mDNS/DNS-SD Service Discovery for HORUS
//!
//! Provides zero-configuration networking via mDNS (multicast DNS) and DNS-SD
//! (DNS Service Discovery). This enables:
//!
//! - **Hostname Resolution**: Connect to `robot.local` without knowing IP addresses
//! - **Service Registration**: Automatically register HORUS nodes/hubs as discoverable services
//! - **Service Discovery**: Browse for available HORUS services on the network
//!
//! # Service Type
//!
//! HORUS services are registered under `_horus._udp.local.` following DNS-SD conventions.
//!
//! # Example Usage
//!
//! ```rust,ignore
//! use horus_core::communication::network::mdns::{HorusMdns, ServiceInfo};
//!
//! // Create mDNS service
//! let mdns = HorusMdns::new()?;
//!
//! // Register this node as a service
//! mdns.register_service("my-robot", 9870, &["lidar", "camera"])?;
//!
//! // Resolve a hostname
//! let ip = mdns.resolve_hostname("other-robot")?;
//!
//! // Browse for services
//! let services = mdns.browse_services()?;
//! for service in services {
//!     println!("Found: {} at {}:{}", service.hostname, service.address, service.port);
//! }
//! ```

use crate::error::HorusResult;
use mdns_sd::{ServiceDaemon, ServiceEvent, ServiceInfo as MdnsServiceInfo};
use std::collections::HashMap;
use std::ffi::CStr;
use std::net::IpAddr;
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, Instant};

/// HORUS service type for DNS-SD registration
pub const HORUS_SERVICE_TYPE: &str = "_horus._udp.local.";

/// Default timeout for mDNS operations
pub const MDNS_TIMEOUT: Duration = Duration::from_millis(100);

/// Extended timeout for browsing (need to collect multiple responses)
pub const BROWSE_TIMEOUT: Duration = Duration::from_secs(2);

/// Information about a discovered HORUS service
#[derive(Debug, Clone)]
pub struct ServiceInfo {
    /// Service instance name (e.g., "my-robot")
    pub instance_name: String,
    /// Resolved hostname (e.g., "my-robot.local")
    pub hostname: String,
    /// IP address of the service
    pub address: IpAddr,
    /// Port number
    pub port: u16,
    /// Topics published by this service
    pub topics: Vec<String>,
    /// When this service was last seen
    pub last_seen: Instant,
    /// Full service name for DNS-SD
    pub full_name: String,
}

/// Cached hostname resolution result
#[derive(Debug, Clone)]
struct CachedResolution {
    ip: IpAddr,
    resolved_at: Instant,
}

/// Cache TTL for hostname resolutions (5 minutes)
const RESOLUTION_CACHE_TTL: Duration = Duration::from_secs(300);

/// mDNS service manager for HORUS
///
/// Wraps the mdns-sd ServiceDaemon to provide:
/// - Service registration for HORUS nodes
/// - Hostname resolution for `.local` addresses
/// - Service browsing/discovery
pub struct HorusMdns {
    /// The underlying mDNS daemon
    daemon: ServiceDaemon,
    /// Cache of discovered services
    services: Arc<RwLock<HashMap<String, ServiceInfo>>>,
    /// Cache of hostname resolutions
    resolution_cache: Arc<Mutex<HashMap<String, CachedResolution>>>,
    /// Our registered service name (if any)
    registered_service: Arc<Mutex<Option<String>>>,
}

impl HorusMdns {
    /// Create a new mDNS service manager
    ///
    /// This starts the mDNS daemon which will handle multicast DNS queries
    /// and responses in a background thread.
    pub fn new() -> HorusResult<Self> {
        let daemon = ServiceDaemon::new().map_err(|e| {
            crate::error::HorusError::communication(format!("Failed to create mDNS daemon: {}", e))
        })?;

        Ok(Self {
            daemon,
            services: Arc::new(RwLock::new(HashMap::new())),
            resolution_cache: Arc::new(Mutex::new(HashMap::new())),
            registered_service: Arc::new(Mutex::new(None)),
        })
    }

    /// Register this HORUS node as a discoverable service
    ///
    /// # Arguments
    /// * `instance_name` - Unique name for this service (e.g., "my-robot", "arm-controller")
    /// * `port` - Port number the service is listening on
    /// * `topics` - List of topics this node publishes/subscribes to
    ///
    /// # Example
    /// ```rust,ignore
    /// mdns.register_service("my-robot", 9870, &["lidar", "camera", "cmd_vel"])?;
    /// ```
    pub fn register_service(
        &self,
        instance_name: &str,
        port: u16,
        topics: &[&str],
    ) -> HorusResult<()> {
        // Get the local hostname using libc
        let hostname = get_local_hostname()?;

        // Build TXT records with topic information
        let mut properties: Vec<(&str, &str)> = Vec::new();
        let version = env!("CARGO_PKG_VERSION");
        properties.push(("version", version));

        // Add topics as comma-separated list
        let topics_str = topics.join(",");
        if !topics.is_empty() {
            properties.push(("topics", &topics_str));
        }

        // Create the service info with auto-detected IP addresses
        let host_name = format!("{}.local.", hostname);
        let full_name = format!("{}.{}", instance_name, HORUS_SERVICE_TYPE);

        let service_info = MdnsServiceInfo::new(
            HORUS_SERVICE_TYPE,
            instance_name,
            &host_name,
            "", // Empty string for auto-detect IP addresses
            port,
            &properties[..],
        )
        .map_err(|e| {
            crate::error::HorusError::communication(format!("Failed to create service info: {}", e))
        })?
        .enable_addr_auto();

        // Register the service
        self.daemon.register(service_info).map_err(|e| {
            crate::error::HorusError::communication(format!("Failed to register service: {}", e))
        })?;

        // Store registered service name for cleanup
        *self.registered_service.lock().unwrap() = Some(full_name.clone());

        log::info!(
            "Registered mDNS service: {} on port {} with topics {:?}",
            instance_name,
            port,
            topics
        );

        Ok(())
    }

    /// Unregister the currently registered service
    pub fn unregister_service(&self) -> HorusResult<()> {
        let mut registered = self.registered_service.lock().unwrap();
        if let Some(full_name) = registered.take() {
            self.daemon.unregister(&full_name).map_err(|e| {
                crate::error::HorusError::communication(format!(
                    "Failed to unregister service: {}",
                    e
                ))
            })?;
            log::info!("Unregistered mDNS service: {}", full_name);
        }
        Ok(())
    }

    /// Resolve an mDNS hostname to an IP address
    ///
    /// This resolves `.local` hostnames using mDNS. The result is cached
    /// to avoid repeated network queries.
    ///
    /// # Arguments
    /// * `hostname` - The hostname without `.local` suffix (e.g., "robot" not "robot.local")
    ///
    /// # Returns
    /// The resolved IP address
    ///
    /// # Example
    /// ```rust,ignore
    /// let ip = mdns.resolve_hostname("my-robot")?;
    /// // Now connect to ip:9870
    /// ```
    pub fn resolve_hostname(&self, hostname: &str) -> HorusResult<IpAddr> {
        // Check cache first
        {
            let cache = self.resolution_cache.lock().unwrap();
            if let Some(cached) = cache.get(hostname) {
                if cached.resolved_at.elapsed() < RESOLUTION_CACHE_TTL {
                    log::debug!("mDNS cache hit for {}: {}", hostname, cached.ip);
                    return Ok(cached.ip);
                }
            }
        }

        // Query for the hostname
        let full_hostname = format!("{}.local.", hostname);

        // Browse for our service type to find the hostname
        let receiver = self.daemon.browse(HORUS_SERVICE_TYPE).map_err(|e| {
            crate::error::HorusError::communication(format!("Failed to browse mDNS: {}", e))
        })?;

        let start = Instant::now();
        while start.elapsed() < MDNS_TIMEOUT {
            match receiver.try_recv() {
                Ok(ServiceEvent::ServiceResolved(info)) => {
                    // Check if this is the hostname we're looking for
                    let service_hostname = info.get_hostname();
                    if service_hostname.starts_with(hostname)
                        || service_hostname.starts_with(&full_hostname)
                    {
                        // Get the first IPv4 address (prefer IPv4 for compatibility)
                        if let Some(addr) = info.get_addresses().iter().find(|a| a.is_ipv4()) {
                            // Cache the result
                            let mut cache = self.resolution_cache.lock().unwrap();
                            cache.insert(
                                hostname.to_string(),
                                CachedResolution {
                                    ip: *addr,
                                    resolved_at: Instant::now(),
                                },
                            );

                            log::debug!("Resolved {} to {}", hostname, addr);
                            return Ok(*addr);
                        }

                        // Fall back to IPv6 if no IPv4
                        if let Some(addr) = info.get_addresses().iter().next() {
                            let mut cache = self.resolution_cache.lock().unwrap();
                            cache.insert(
                                hostname.to_string(),
                                CachedResolution {
                                    ip: *addr,
                                    resolved_at: Instant::now(),
                                },
                            );

                            log::debug!("Resolved {} to {} (IPv6)", hostname, addr);
                            return Ok(*addr);
                        }
                    }
                }
                Ok(_) => continue, // Other events, keep waiting
                Err(_) => {
                    // Error could be Empty or Disconnected
                    if receiver.is_disconnected() {
                        break;
                    }
                    std::thread::sleep(Duration::from_millis(10));
                }
            }
        }

        Err(crate::error::HorusError::Communication(format!(
            "Failed to resolve hostname '{}' via mDNS (timeout after {:?})",
            hostname, MDNS_TIMEOUT
        )))
    }

    /// Resolve hostname with a custom timeout
    pub fn resolve_hostname_with_timeout(
        &self,
        hostname: &str,
        timeout: Duration,
    ) -> HorusResult<IpAddr> {
        // Check cache first (same as resolve_hostname)
        {
            let cache = self.resolution_cache.lock().unwrap();
            if let Some(cached) = cache.get(hostname) {
                if cached.resolved_at.elapsed() < RESOLUTION_CACHE_TTL {
                    return Ok(cached.ip);
                }
            }
        }

        let receiver = self.daemon.browse(HORUS_SERVICE_TYPE).map_err(|e| {
            crate::error::HorusError::communication(format!("Failed to browse mDNS: {}", e))
        })?;

        let full_hostname = format!("{}.local.", hostname);
        let start = Instant::now();

        while start.elapsed() < timeout {
            match receiver.try_recv() {
                Ok(ServiceEvent::ServiceResolved(info)) => {
                    let service_hostname = info.get_hostname();
                    if service_hostname.starts_with(hostname)
                        || service_hostname.starts_with(&full_hostname)
                    {
                        if let Some(addr) = info
                            .get_addresses()
                            .iter()
                            .find(|a| a.is_ipv4())
                            .or_else(|| info.get_addresses().iter().next())
                        {
                            let mut cache = self.resolution_cache.lock().unwrap();
                            cache.insert(
                                hostname.to_string(),
                                CachedResolution {
                                    ip: *addr,
                                    resolved_at: Instant::now(),
                                },
                            );
                            return Ok(*addr);
                        }
                    }
                }
                Ok(_) => continue,
                Err(_) => {
                    if receiver.is_disconnected() {
                        break;
                    }
                    std::thread::sleep(Duration::from_millis(10));
                }
            }
        }

        Err(crate::error::HorusError::Communication(format!(
            "Failed to resolve hostname '{}' via mDNS (timeout after {:?})",
            hostname, timeout
        )))
    }

    /// Browse for all available HORUS services on the network
    ///
    /// Returns a list of discovered services. This operation takes up to
    /// `BROWSE_TIMEOUT` to collect responses from the network.
    ///
    /// # Example
    /// ```rust,ignore
    /// let services = mdns.browse_services()?;
    /// for service in services {
    ///     println!("Found: {} at {}:{}", service.hostname, service.address, service.port);
    ///     println!("  Topics: {:?}", service.topics);
    /// }
    /// ```
    pub fn browse_services(&self) -> HorusResult<Vec<ServiceInfo>> {
        self.browse_services_with_timeout(BROWSE_TIMEOUT)
    }

    /// Browse for services with a custom timeout
    pub fn browse_services_with_timeout(&self, timeout: Duration) -> HorusResult<Vec<ServiceInfo>> {
        let receiver = self.daemon.browse(HORUS_SERVICE_TYPE).map_err(|e| {
            crate::error::HorusError::communication(format!("Failed to browse mDNS: {}", e))
        })?;

        let start = Instant::now();
        let mut discovered: HashMap<String, ServiceInfo> = HashMap::new();

        while start.elapsed() < timeout {
            match receiver.try_recv() {
                Ok(ServiceEvent::ServiceResolved(info)) => {
                    let instance_name = info.get_fullname().to_string();

                    // Extract topics from TXT records using get_property_val_str
                    let topics: Vec<String> = info
                        .get_property_val_str("topics")
                        .map(|s| s.split(',').map(String::from).collect())
                        .unwrap_or_default();

                    // Get the first address (prefer IPv4)
                    if let Some(addr) = info
                        .get_addresses()
                        .iter()
                        .find(|a| a.is_ipv4())
                        .or_else(|| info.get_addresses().iter().next())
                    {
                        let service_info = ServiceInfo {
                            instance_name: info
                                .get_fullname()
                                .split('.')
                                .next()
                                .unwrap_or("")
                                .to_string(),
                            hostname: info.get_hostname().to_string(),
                            address: *addr,
                            port: info.get_port(),
                            topics,
                            last_seen: Instant::now(),
                            full_name: instance_name.clone(),
                        };

                        discovered.insert(instance_name, service_info);
                    }
                }
                Ok(ServiceEvent::ServiceRemoved(_, name)) => {
                    discovered.remove(&name);
                }
                Ok(_) => continue,
                Err(_) => {
                    if receiver.is_disconnected() {
                        break;
                    }
                    std::thread::sleep(Duration::from_millis(50));
                }
            }
        }

        // Update the internal cache
        {
            let mut services = self.services.write().unwrap();
            *services = discovered.clone();
        }

        Ok(discovered.into_values().collect())
    }

    /// Get the list of currently known services (from cache)
    ///
    /// This returns cached results from the last browse operation.
    /// Call `browse_services()` first to populate the cache.
    pub fn get_cached_services(&self) -> Vec<ServiceInfo> {
        self.services.read().unwrap().values().cloned().collect()
    }

    /// Find a service by instance name
    ///
    /// Searches the cache for a service with the given instance name.
    pub fn find_service(&self, instance_name: &str) -> Option<ServiceInfo> {
        self.services
            .read()
            .unwrap()
            .values()
            .find(|s| s.instance_name == instance_name)
            .cloned()
    }

    /// Find services that publish a specific topic
    pub fn find_services_with_topic(&self, topic: &str) -> Vec<ServiceInfo> {
        self.services
            .read()
            .unwrap()
            .values()
            .filter(|s| s.topics.contains(&topic.to_string()))
            .cloned()
            .collect()
    }

    /// Clear the resolution cache
    pub fn clear_cache(&self) {
        self.resolution_cache.lock().unwrap().clear();
        self.services.write().unwrap().clear();
    }

    /// Get cache statistics
    pub fn cache_stats(&self) -> MdnsCacheStats {
        let resolution_cache = self.resolution_cache.lock().unwrap();
        let services = self.services.read().unwrap();

        MdnsCacheStats {
            cached_resolutions: resolution_cache.len(),
            cached_services: services.len(),
        }
    }

    /// Shutdown the mDNS daemon gracefully
    pub fn shutdown(self) -> HorusResult<()> {
        // Unregister any registered service first
        {
            let registered = self.registered_service.lock().unwrap();
            if let Some(ref full_name) = *registered {
                let _ = self.daemon.unregister(full_name);
            }
        }

        self.daemon.shutdown().map_err(|e| {
            crate::error::HorusError::communication(format!(
                "Failed to shutdown mDNS daemon: {}",
                e
            ))
        })?;

        Ok(())
    }
}

impl Drop for HorusMdns {
    fn drop(&mut self) {
        // Try to unregister service on drop
        let registered = self.registered_service.lock().unwrap();
        if let Some(ref full_name) = *registered {
            let _ = self.daemon.unregister(full_name);
        }
    }
}

/// Cache statistics for mDNS
#[derive(Debug, Clone)]
pub struct MdnsCacheStats {
    /// Number of cached hostname resolutions
    pub cached_resolutions: usize,
    /// Number of cached service discoveries
    pub cached_services: usize,
}

/// Resolve an mDNS hostname to an IP address (standalone function)
///
/// This is a convenience function that creates a temporary HorusMdns instance.
/// For repeated use, create a HorusMdns instance and reuse it.
pub fn resolve_mdns_hostname(hostname: &str) -> HorusResult<IpAddr> {
    let mdns = HorusMdns::new()?;
    mdns.resolve_hostname(hostname)
}

/// Resolve an mDNS hostname with a custom timeout (standalone function)
pub fn resolve_mdns_hostname_with_timeout(
    hostname: &str,
    timeout: Duration,
) -> HorusResult<IpAddr> {
    let mdns = HorusMdns::new()?;
    mdns.resolve_hostname_with_timeout(hostname, timeout)
}

/// Get the local hostname using libc gethostname
fn get_local_hostname() -> HorusResult<String> {
    let mut buf = [0u8; 256];

    // SAFETY: buf is a valid 256-byte buffer. gethostname writes a null-terminated hostname into it.
    let result = unsafe { libc::gethostname(buf.as_mut_ptr() as *mut libc::c_char, buf.len()) };

    if result != 0 {
        return Err(crate::error::HorusError::Communication(
            "Failed to get hostname".to_string(),
        ));
    }

    // Ensure null-termination — POSIX gethostname may not null-terminate if hostname
    // is exactly buf.len() bytes long.
    buf[buf.len() - 1] = 0;
    let hostname = unsafe { CStr::from_ptr(buf.as_ptr() as *const libc::c_char) };

    hostname
        .to_str()
        .map(|s| s.to_string())
        .map_err(|e| crate::error::HorusError::Communication(format!("Invalid hostname: {}", e)))
}

// ============================================================================
// HIGH-LEVEL DISCOVERY API
// ============================================================================
//
// This section provides a user-friendly API for discovering HORUS nodes on
// the network using mDNS/DNS-SD. It builds on the low-level HorusMdns service
// to provide:
// - Easy-to-use discover() function for one-shot discovery
// - DiscoveredNode struct with human-readable fields
// - Continuous monitoring via watch() for real-time updates
// - JSON serialization support for CLI tooling
// - Filtering by topic or capability

use serde::{Deserialize, Serialize};
use std::sync::mpsc;
use std::thread;
use std::time::SystemTime;

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
                        let _ = sender.send(DiscoveryEvent::Error(format!("Browse failed: {}", e)));
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

// ============================================================================
// AUTOMATIC NODE REGISTRATION
// ============================================================================
//
// This section provides automatic service discovery registration for HORUS
// nodes. When enabled, nodes automatically:
// - Register as {node-name}.local on the network
// - Publish their topics as TXT records for discovery
// - Update registration when topics change
// - Deregister cleanly on shutdown

/// Default port for HORUS services
pub const DEFAULT_HORUS_PORT: u16 = 9870;

/// Configuration for mDNS node registration
#[derive(Debug, Clone)]
pub struct MdnsRegistrationConfig {
    /// Port to advertise for the service
    pub port: u16,
    /// Whether to automatically register on creation
    pub auto_register: bool,
    /// Whether to include version in TXT records
    pub include_version: bool,
    /// Additional TXT record properties
    pub extra_properties: Vec<(String, String)>,
}

impl Default for MdnsRegistrationConfig {
    fn default() -> Self {
        Self {
            port: DEFAULT_HORUS_PORT,
            auto_register: true,
            include_version: true,
            extra_properties: Vec::new(),
        }
    }
}

/// Automatic mDNS registration for a HORUS node
///
/// This struct manages the mDNS service registration lifecycle:
/// - Registration happens on creation (if auto_register is true)
/// - Topics can be updated dynamically
/// - Deregistration happens on Drop
///
/// # Example
/// ```rust,ignore
/// // Simple registration with defaults
/// let reg = MdnsNodeRegistration::new("arm-controller", 9870)?;
/// reg.register(&["joint_states", "joint_commands"])?;
///
/// // Or with builder pattern
/// let reg = MdnsNodeRegistration::builder("arm-controller")
///     .port(9871)
///     .auto_register(false)
///     .extra_property("robot_type", "arm")
///     .build()?;
/// ```
pub struct MdnsNodeRegistration {
    /// Node name (used as instance name in mDNS)
    node_name: String,
    /// Sanitized hostname (underscores replaced with hyphens)
    hostname: String,
    /// Configuration
    config: MdnsRegistrationConfig,
    /// The underlying mDNS service
    mdns: Arc<HorusMdns>,
    /// Currently registered topics
    current_topics: RwLock<Vec<String>>,
    /// Whether we're currently registered
    is_registered: Mutex<bool>,
}

impl MdnsNodeRegistration {
    /// Create a new mDNS registration for a node
    ///
    /// # Arguments
    /// * `node_name` - The node's name (will be sanitized for mDNS: underscores → hyphens)
    /// * `port` - Port number the node is listening on
    ///
    /// # Example
    /// ```rust,ignore
    /// let reg = MdnsNodeRegistration::new("arm_controller", 9870)?;
    /// // Node will be discoverable as "arm-controller.local"
    /// ```
    pub fn new(node_name: &str, port: u16) -> HorusResult<Self> {
        let config = MdnsRegistrationConfig {
            port,
            auto_register: false, // Don't auto-register, let caller do it with topics
            ..Default::default()
        };
        Self::with_config(node_name, config)
    }

    /// Create with full configuration
    pub fn with_config(node_name: &str, config: MdnsRegistrationConfig) -> HorusResult<Self> {
        // Sanitize node name for mDNS (replace underscores with hyphens)
        let hostname = sanitize_hostname(node_name);

        let mdns = Arc::new(HorusMdns::new()?);

        Ok(Self {
            node_name: node_name.to_string(),
            hostname,
            config,
            mdns,
            current_topics: RwLock::new(Vec::new()),
            is_registered: Mutex::new(false),
        })
    }

    /// Create a builder for more complex configurations
    pub fn builder(node_name: &str) -> MdnsRegistrationBuilder {
        MdnsRegistrationBuilder::new(node_name)
    }

    /// Register this node with the given topics
    ///
    /// # Arguments
    /// * `topics` - List of topic names this node publishes/subscribes to
    ///
    /// # Returns
    /// Ok(()) if registration succeeded, or an error
    pub fn register(&self, topics: &[&str]) -> HorusResult<()> {
        let mut is_registered = self.is_registered.lock().unwrap();

        if *is_registered {
            // Already registered, just update topics
            drop(is_registered);
            return self.update_topics(topics);
        }

        // Store topics
        {
            let mut current = self.current_topics.write().unwrap();
            *current = topics.iter().map(|s| s.to_string()).collect();
        }

        // Register with mDNS
        self.mdns
            .register_service(&self.hostname, self.config.port, topics)?;

        *is_registered = true;

        log::info!(
            "mDNS: Registered node '{}' as {}.local:{} with topics {:?}",
            self.node_name,
            self.hostname,
            self.config.port,
            topics
        );

        Ok(())
    }

    /// Register with topics from strings (owned)
    pub fn register_owned(&self, topics: Vec<String>) -> HorusResult<()> {
        let refs: Vec<&str> = topics.iter().map(|s| s.as_str()).collect();
        self.register(&refs)
    }

    /// Update the registered topics
    ///
    /// This re-registers the service with updated topic information.
    /// Useful when a node dynamically adds/removes publishers or subscribers.
    pub fn update_topics(&self, topics: &[&str]) -> HorusResult<()> {
        let is_registered = self.is_registered.lock().unwrap();

        if !*is_registered {
            // Not registered yet, just register
            drop(is_registered);
            return self.register(topics);
        }
        drop(is_registered);

        // Check if topics actually changed
        {
            let current = self.current_topics.read().unwrap();
            let new_topics: Vec<String> = topics.iter().map(|s| s.to_string()).collect();
            if *current == new_topics {
                return Ok(()); // No change
            }
        }

        // Unregister old service
        self.mdns.unregister_service()?;

        // Update stored topics
        {
            let mut current = self.current_topics.write().unwrap();
            *current = topics.iter().map(|s| s.to_string()).collect();
        }

        // Re-register with new topics
        self.mdns
            .register_service(&self.hostname, self.config.port, topics)?;

        log::debug!(
            "mDNS: Updated topics for '{}' to {:?}",
            self.node_name,
            topics
        );

        Ok(())
    }

    /// Unregister this node from mDNS
    ///
    /// This is called automatically on Drop, but can be called explicitly
    /// for clean shutdown sequences.
    pub fn unregister(&self) -> HorusResult<()> {
        let mut is_registered = self.is_registered.lock().unwrap();

        if !*is_registered {
            return Ok(()); // Already unregistered
        }

        self.mdns.unregister_service()?;
        *is_registered = false;

        log::info!("mDNS: Unregistered node '{}'", self.node_name);

        Ok(())
    }

    /// Check if currently registered
    pub fn is_registered(&self) -> bool {
        *self.is_registered.lock().unwrap()
    }

    /// Get the sanitized hostname
    pub fn hostname(&self) -> &str {
        &self.hostname
    }

    /// Get the original node name
    pub fn node_name(&self) -> &str {
        &self.node_name
    }

    /// Get the current topics
    pub fn topics(&self) -> Vec<String> {
        self.current_topics.read().unwrap().clone()
    }

    /// Get the advertised port
    pub fn port(&self) -> u16 {
        self.config.port
    }

    /// Get reference to the underlying mDNS service (for advanced use)
    pub fn mdns(&self) -> &HorusMdns {
        &self.mdns
    }
}

impl Drop for MdnsNodeRegistration {
    fn drop(&mut self) {
        // Best effort unregistration on drop
        let _ = self.unregister();
    }
}

/// Builder for MdnsNodeRegistration
pub struct MdnsRegistrationBuilder {
    node_name: String,
    config: MdnsRegistrationConfig,
}

impl MdnsRegistrationBuilder {
    /// Create a new builder
    pub fn new(node_name: &str) -> Self {
        Self {
            node_name: node_name.to_string(),
            config: MdnsRegistrationConfig::default(),
        }
    }

    /// Set the port number
    pub fn port(mut self, port: u16) -> Self {
        self.config.port = port;
        self
    }

    /// Enable/disable auto-registration on build
    pub fn auto_register(mut self, auto: bool) -> Self {
        self.config.auto_register = auto;
        self
    }

    /// Enable/disable version in TXT records
    pub fn include_version(mut self, include: bool) -> Self {
        self.config.include_version = include;
        self
    }

    /// Add an extra TXT record property
    pub fn extra_property(mut self, key: &str, value: &str) -> Self {
        self.config
            .extra_properties
            .push((key.to_string(), value.to_string()));
        self
    }

    /// Build the registration
    pub fn build(self) -> HorusResult<MdnsNodeRegistration> {
        MdnsNodeRegistration::with_config(&self.node_name, self.config)
    }
}

/// Sanitize a node name for use as an mDNS hostname
///
/// mDNS hostnames must:
/// - Contain only alphanumeric characters and hyphens
/// - Not start or end with a hyphen
/// - Be lowercase
///
/// This function replaces underscores with hyphens and lowercases the name.
pub fn sanitize_hostname(name: &str) -> String {
    let mut result: String = name
        .chars()
        .map(|c| {
            if c == '_' {
                '-'
            } else if c.is_alphanumeric() || c == '-' {
                c.to_ascii_lowercase()
            } else {
                '-'
            }
        })
        .collect();

    // Remove leading/trailing hyphens
    while result.starts_with('-') {
        result.remove(0);
    }
    while result.ends_with('-') {
        result.pop();
    }

    // Collapse consecutive hyphens
    while result.contains("--") {
        result = result.replace("--", "-");
    }

    // Ensure not empty
    if result.is_empty() {
        result = "horus-node".to_string();
    }

    result
}

/// Global mDNS registration manager
///
/// Provides a singleton-like interface for managing node registration
/// across the application. Useful when you want centralized control
/// over all mDNS registrations.
pub struct GlobalMdnsManager {
    registrations: RwLock<std::collections::HashMap<String, Arc<MdnsNodeRegistration>>>,
}

impl GlobalMdnsManager {
    /// Create a new global manager
    pub fn new() -> Self {
        Self {
            registrations: RwLock::new(std::collections::HashMap::new()),
        }
    }

    /// Register a node globally
    pub fn register_node(
        &self,
        node_name: &str,
        port: u16,
        topics: &[&str],
    ) -> HorusResult<Arc<MdnsNodeRegistration>> {
        let registration = Arc::new(MdnsNodeRegistration::new(node_name, port)?);
        registration.register(topics)?;

        let mut registrations = self.registrations.write().unwrap();
        registrations.insert(node_name.to_string(), registration.clone());

        Ok(registration)
    }

    /// Get a registration by node name
    pub fn get(&self, node_name: &str) -> Option<Arc<MdnsNodeRegistration>> {
        self.registrations.read().unwrap().get(node_name).cloned()
    }

    /// Unregister a node
    pub fn unregister_node(&self, node_name: &str) -> HorusResult<()> {
        let mut registrations = self.registrations.write().unwrap();
        if let Some(reg) = registrations.remove(node_name) {
            reg.unregister()?;
        }
        Ok(())
    }

    /// Unregister all nodes
    pub fn unregister_all(&self) -> HorusResult<()> {
        let mut registrations = self.registrations.write().unwrap();
        for (_, reg) in registrations.drain() {
            let _ = reg.unregister();
        }
        Ok(())
    }

    /// Get count of registered nodes
    pub fn count(&self) -> usize {
        self.registrations.read().unwrap().len()
    }

    /// Get all registered node names
    pub fn node_names(&self) -> Vec<String> {
        self.registrations.read().unwrap().keys().cloned().collect()
    }
}

impl Default for GlobalMdnsManager {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for GlobalMdnsManager {
    fn drop(&mut self) {
        let _ = self.unregister_all();
    }
}

// ============================================================================
// TESTS
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // Core mDNS tests
    #[test]
    fn test_service_type_format() {
        assert!(HORUS_SERVICE_TYPE.starts_with('_'));
        assert!(HORUS_SERVICE_TYPE.ends_with(".local."));
        assert!(HORUS_SERVICE_TYPE.contains("._udp."));
    }

    #[test]
    fn test_cache_stats_initial() {
        // Note: This test may fail on systems without network access
        // Skip in CI environments
        if std::env::var("CI").is_ok() {
            return;
        }

        let mdns = HorusMdns::new();
        if let Ok(mdns) = mdns {
            let stats = mdns.cache_stats();
            assert_eq!(stats.cached_resolutions, 0);
            assert_eq!(stats.cached_services, 0);
        }
    }

    // Discovery tests
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

    // Registration tests
    #[test]
    fn test_sanitize_hostname_basic() {
        assert_eq!(sanitize_hostname("my_robot"), "my-robot");
        assert_eq!(sanitize_hostname("arm_controller"), "arm-controller");
        assert_eq!(sanitize_hostname("MyRobot"), "myrobot");
    }

    #[test]
    fn test_sanitize_hostname_edge_cases() {
        assert_eq!(sanitize_hostname("_leading"), "leading");
        assert_eq!(sanitize_hostname("trailing_"), "trailing");
        assert_eq!(sanitize_hostname("__multiple__"), "multiple");
        assert_eq!(sanitize_hostname(""), "horus-node");
        assert_eq!(sanitize_hostname("___"), "horus-node");
    }

    #[test]
    fn test_sanitize_hostname_special_chars() {
        assert_eq!(sanitize_hostname("robot@1"), "robot-1");
        assert_eq!(sanitize_hostname("robot.local"), "robot-local");
        assert_eq!(sanitize_hostname("robot 1"), "robot-1");
    }

    #[test]
    fn test_sanitize_hostname_numbers() {
        assert_eq!(sanitize_hostname("robot1"), "robot1");
        assert_eq!(sanitize_hostname("123"), "123");
        assert_eq!(sanitize_hostname("robot_123"), "robot-123");
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

    #[test]
    #[ignore]
    fn test_registration_lifecycle() {
        let reg = MdnsNodeRegistration::new("test-node", 9870).unwrap();
        assert!(!reg.is_registered());

        reg.register(&["topic1", "topic2"]).unwrap();
        assert!(reg.is_registered());
        assert_eq!(reg.topics(), vec!["topic1", "topic2"]);

        reg.update_topics(&["topic1", "topic2", "topic3"]).unwrap();
        assert_eq!(reg.topics(), vec!["topic1", "topic2", "topic3"]);

        reg.unregister().unwrap();
        assert!(!reg.is_registered());
    }
}
