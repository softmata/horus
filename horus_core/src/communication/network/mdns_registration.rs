//! Automatic mDNS Registration for HORUS Nodes
//!
//! This module provides automatic service discovery registration for HORUS nodes.
//! When enabled, nodes automatically:
//! - Register as `{node-name}.local` on the network
//! - Publish their topics as TXT records for discovery
//! - Update registration when topics change
//! - Deregister cleanly on shutdown
//!
//! # Usage
//!
//! ```rust,ignore
//! use horus_core::communication::network::mdns_registration::MdnsNodeRegistration;
//!
//! // In your node's init:
//! let registration = MdnsNodeRegistration::new("my-robot", 9870)?;
//! registration.register(&["lidar", "camera"])?;
//!
//! // Later, if topics change:
//! registration.update_topics(&["lidar", "camera", "cmd_vel"])?;
//!
//! // On shutdown (or use RAII via Drop):
//! registration.unregister()?;
//! ```
//!
//! # Thread Safety
//!
//! `MdnsNodeRegistration` is thread-safe and can be shared across threads using `Arc`.
//! The underlying mDNS daemon runs in its own thread.

use crate::error::HorusResult;
use std::sync::{Arc, Mutex, RwLock};

use super::mdns::HorusMdns;

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

#[cfg(test)]
mod tests {
    use super::*;

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
