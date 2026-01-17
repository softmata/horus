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
        let daemon = ServiceDaemon::new()
            .map_err(|e| format!("Failed to create mDNS daemon: {}", e))?;

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
            "",  // Empty string for auto-detect IP addresses
            port,
            &properties[..],
        )
        .map_err(|e| format!("Failed to create service info: {}", e))?
        .enable_addr_auto();

        // Register the service
        self.daemon
            .register(service_info)
            .map_err(|e| format!("Failed to register service: {}", e))?;

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
            self.daemon
                .unregister(&full_name)
                .map_err(|e| format!("Failed to unregister service: {}", e))?;
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
        let receiver = self
            .daemon
            .browse(HORUS_SERVICE_TYPE)
            .map_err(|e| format!("Failed to browse mDNS: {}", e))?;

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

        let receiver = self
            .daemon
            .browse(HORUS_SERVICE_TYPE)
            .map_err(|e| format!("Failed to browse mDNS: {}", e))?;

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
        let receiver = self
            .daemon
            .browse(HORUS_SERVICE_TYPE)
            .map_err(|e| format!("Failed to browse mDNS: {}", e))?;

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

        self.daemon
            .shutdown()
            .map_err(|e| format!("Failed to shutdown mDNS daemon: {}", e))?;

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

    // SAFETY: gethostname writes to the buffer and returns 0 on success
    let result = unsafe {
        libc::gethostname(buf.as_mut_ptr() as *mut libc::c_char, buf.len())
    };

    if result != 0 {
        return Err(crate::error::HorusError::Communication(
            "Failed to get hostname".to_string(),
        ));
    }

    // Find the null terminator and convert to string
    let hostname = unsafe {
        CStr::from_ptr(buf.as_ptr() as *const libc::c_char)
    };

    hostname
        .to_str()
        .map(|s| s.to_string())
        .map_err(|e| crate::error::HorusError::Communication(format!("Invalid hostname: {}", e)))
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
