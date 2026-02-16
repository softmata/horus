//! Driver loader - High-level driver management respecting DriverMode
//!
//! This module provides the [`DriverLoader`] which integrates with HORUS's
//! configuration system and provides a unified interface for loading drivers
//! in static, dynamic, or hybrid mode.
//!
//! # Driver Modes
//!
//! - **Static**: All drivers compiled into the binary via feature flags. Zero runtime overhead.
//! - **Dynamic**: Drivers loaded at runtime from shared libraries. Maximum flexibility.
//! - **Hybrid**: Core drivers static, specialized drivers loaded dynamically.
//!
//! # Configuration (horus.yaml)
//!
//! ```yaml
//! driver_mode: hybrid  # static, dynamic, or hybrid
//!
//! plugins:
//!   search_paths:
//!     - ~/.horus/drivers
//!     - /usr/local/lib/horus/drivers
//!
//!   # For hybrid mode: which drivers are static
//!   static:
//!     - imu
//!     - motor
//!
//!   # For hybrid mode: which drivers are dynamic
//!   dynamic:
//!     - camera-fancy
//!     - lidar-velodyne
//! ```

use super::loader::{DiscoveredPlugin, PluginLoader};
use super::traits::DriverPlugin;
use super::types::{PluginError, PluginResult};
use crate::driver::SingleDriverConfig;
use crate::error::HorusResult;

use serde::{Deserialize, Serialize};
use std::any::Any;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;

/// Driver loading mode
///
/// Determines how drivers are discovered and loaded.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum DriverMode {
    /// Static mode: Drivers are compiled into the binary via feature flags.
    ///
    /// Pros: Zero runtime overhead, no dlopen, maximum performance.
    /// Cons: Requires recompilation to add/remove drivers.
    #[default]
    Static,

    /// Dynamic mode: Drivers are loaded at runtime from shared libraries.
    ///
    /// Pros: Maximum flexibility, hot-reload support, no recompilation.
    /// Cons: Small runtime overhead, requires libloading.
    Dynamic,

    /// Hybrid mode: Core drivers static, specialized drivers dynamic.
    ///
    /// Pros: Best of both worlds - common drivers fast, rare drivers flexible.
    /// The configuration specifies which drivers are static vs dynamic.
    Hybrid,
}

impl std::fmt::Display for DriverMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DriverMode::Static => write!(f, "static"),
            DriverMode::Dynamic => write!(f, "dynamic"),
            DriverMode::Hybrid => write!(f, "hybrid"),
        }
    }
}

/// Configuration for the driver loader
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DriverLoaderConfig {
    /// Driver loading mode
    #[serde(default)]
    pub mode: DriverMode,

    /// Search paths for dynamic plugins
    #[serde(default)]
    pub search_paths: Vec<PathBuf>,

    /// Drivers to load statically (for hybrid mode)
    #[serde(default)]
    pub static_drivers: Vec<String>,

    /// Drivers to load dynamically (for hybrid mode)
    #[serde(default)]
    pub dynamic_drivers: Vec<String>,

    /// Whether to auto-discover drivers in search paths
    #[serde(default = "crate::utils::default_true")]
    pub auto_discover: bool,
}

/// High-level driver loader that respects DriverMode configuration
///
/// This is the primary interface for the driver system. It wraps [`PluginLoader`]
/// and provides additional functionality:
///
/// - Respects `driver_mode` from configuration
/// - Hybrid mode support (static + dynamic)
/// - Driver category filtering
/// - Auto-detection workflows
///
/// # Example
///
/// ```rust,ignore
/// use horus_core::plugin::{DriverLoader, DriverLoaderConfig, DriverMode};
///
/// // Load from configuration
/// let config = DriverLoaderConfig {
///     mode: DriverMode::Hybrid,
///     search_paths: vec!["~/.horus/drivers".into()],
///     static_drivers: vec!["imu".to_string()],
///     dynamic_drivers: vec!["camera-fancy".to_string()],
///     auto_discover: true,
/// };
///
/// let mut loader = DriverLoader::new(config);
/// loader.initialize()?;
///
/// // List available drivers
/// for driver_id in loader.available_drivers() {
///     println!("Available: {}", driver_id);
/// }
///
/// // Create a driver
/// let imu_config = SingleDriverConfig::default();
/// let imu = loader.create_driver("imu", "mpu6050", &imu_config)?;
/// ```
pub struct DriverLoader {
    /// Configuration
    config: DriverLoaderConfig,

    /// Underlying plugin loader
    plugin_loader: PluginLoader,

    /// Static driver registry (for static/hybrid mode)
    /// Maps driver category/id to factory functions
    static_registry: HashMap<String, Arc<dyn DriverPlugin>>,

    /// Whether the loader has been initialized
    initialized: bool,
}

impl DriverLoader {
    /// Create a new driver loader with the given configuration
    pub fn new(config: DriverLoaderConfig) -> Self {
        let mut plugin_loader = PluginLoader::new();

        // Add configured search paths
        for path in &config.search_paths {
            plugin_loader.add_search_path(path);
        }

        // Add default paths if auto-discover is enabled
        if config.auto_discover {
            plugin_loader.add_default_search_paths();
        }

        Self {
            config,
            plugin_loader,
            static_registry: HashMap::new(),
            initialized: false,
        }
    }

    /// Create a driver loader with default configuration
    pub fn with_defaults() -> Self {
        Self::new(DriverLoaderConfig::default())
    }

    /// Get the current driver mode
    pub fn mode(&self) -> DriverMode {
        self.config.mode
    }

    /// Register a static driver
    ///
    /// Call this for each compiled-in driver. In static mode, only registered
    /// drivers are available. In hybrid mode, these are preferred over dynamic.
    pub fn register_static(&mut self, plugin: Box<dyn DriverPlugin>) {
        let manifest = plugin.manifest();
        let id = manifest.id.clone();

        // Store in static registry
        self.static_registry.insert(id.clone(), Arc::from(plugin));

        // Also register with plugin loader for unified discovery
        // Note: We can't move the Box twice, so we create a wrapper
        // This is handled by the plugin loader's register_static
    }

    /// Register a static driver from an Arc
    pub fn register_static_arc(&mut self, plugin: Arc<dyn DriverPlugin>) {
        let manifest = plugin.manifest();
        let id = manifest.id.clone();
        self.static_registry.insert(id, plugin);
    }

    /// Initialize the driver loader
    ///
    /// This discovers available drivers based on the mode:
    /// - Static: Only uses registered static drivers
    /// - Dynamic: Scans search paths for plugins
    /// - Hybrid: Both static registration and dynamic discovery
    pub fn initialize(&mut self) -> PluginResult<()> {
        match self.config.mode {
            DriverMode::Static => {
                // Static mode: Only static drivers, no discovery needed
                // Already registered via register_static()
            }
            DriverMode::Dynamic | DriverMode::Hybrid => {
                // Discover dynamic plugins
                #[cfg(feature = "dynamic-plugins")]
                {
                    self.plugin_loader.discover()?;
                }
                #[cfg(not(feature = "dynamic-plugins"))]
                {
                    if self.config.mode == DriverMode::Dynamic {
                        return Err(PluginError::LoadError(
                            "Dynamic mode requires 'dynamic-plugins' feature".into(),
                        ));
                    }
                    // Hybrid mode without dynamic-plugins feature is just static mode
                }
            }
        }

        self.initialized = true;
        Ok(())
    }

    /// Check if the loader is initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Get list of available driver IDs
    pub fn available_drivers(&self) -> Vec<String> {
        let mut drivers = Vec::new();

        // Add static drivers
        for id in self.static_registry.keys() {
            drivers.push(id.clone());
        }

        // Add discovered dynamic drivers (if in dynamic/hybrid mode)
        if matches!(self.config.mode, DriverMode::Dynamic | DriverMode::Hybrid) {
            for id in self.plugin_loader.discovered_ids() {
                if !drivers.contains(&id.to_string()) {
                    drivers.push(id.to_string());
                }
            }
        }

        drivers.sort();
        drivers
    }

    /// Get plugin by ID
    ///
    /// In hybrid mode, prefers static over dynamic.
    pub fn get_plugin(&self, id: &str) -> PluginResult<Arc<dyn DriverPlugin>> {
        // Check static registry first (for static/hybrid mode)
        if let Some(plugin) = self.static_registry.get(id) {
            return Ok(Arc::clone(plugin));
        }

        // Check dynamic plugins (for dynamic/hybrid mode)
        if matches!(self.config.mode, DriverMode::Dynamic | DriverMode::Hybrid)
            && self.plugin_loader.is_discovered(id)
        {
            return self.plugin_loader.load(id);
        }

        Err(PluginError::NotFound(id.to_string()))
    }

    /// Create a driver instance
    ///
    /// # Arguments
    ///
    /// * `plugin_id` - Plugin ID (e.g., "horus-imu")
    /// * `backend_id` - Backend ID within the plugin (e.g., "mpu6050")
    /// * `config` - Driver configuration
    ///
    /// # Returns
    ///
    /// A boxed driver instance that should be downcast to the specific type.
    pub fn create_driver(
        &self,
        plugin_id: &str,
        backend_id: &str,
        config: &SingleDriverConfig,
    ) -> HorusResult<Box<dyn Any + Send + Sync>> {
        let plugin = self.get_plugin(plugin_id).map_err(|e| {
            crate::error::HorusError::driver(format!("Failed to get plugin '{}': {}", plugin_id, e))
        })?;

        plugin.create(backend_id, config)
    }

    /// Auto-detect and create a driver from a plugin
    ///
    /// Probes hardware and creates a driver using the first detected backend.
    pub fn auto_create_driver(
        &self,
        plugin_id: &str,
        config: &SingleDriverConfig,
    ) -> HorusResult<Box<dyn Any + Send + Sync>> {
        let plugin = self.get_plugin(plugin_id).map_err(|e| {
            crate::error::HorusError::driver(format!("Failed to get plugin '{}': {}", plugin_id, e))
        })?;

        // Use AutoDetectable trait
        use super::traits::AutoDetectable;
        plugin.auto_create(config)
    }

    /// Probe for hardware using a specific plugin
    pub fn probe(&self, plugin_id: &str) -> PluginResult<Vec<super::types::ProbeResult>> {
        let plugin = self.get_plugin(plugin_id)?;
        Ok(plugin.probe(""))
    }

    /// Get health status of all loaded plugins
    pub fn health(&self) -> HashMap<String, super::types::PluginHealth> {
        let mut health = HashMap::new();

        for (id, plugin) in &self.static_registry {
            health.insert(id.clone(), plugin.health());
        }

        for plugin in self.plugin_loader.loaded_plugins() {
            let manifest = plugin.manifest();
            if !health.contains_key(&manifest.id) {
                health.insert(manifest.id.clone(), plugin.health());
            }
        }

        health
    }

    /// Check if a driver should be static (for hybrid mode)
    pub fn is_static_driver(&self, id: &str) -> bool {
        if self.config.mode != DriverMode::Hybrid {
            return self.config.mode == DriverMode::Static;
        }

        // In hybrid mode, check the configuration
        self.config.static_drivers.iter().any(|s| s == id)
    }

    /// Check if a driver should be dynamic (for hybrid mode)
    pub fn is_dynamic_driver(&self, id: &str) -> bool {
        if self.config.mode != DriverMode::Hybrid {
            return self.config.mode == DriverMode::Dynamic;
        }

        // In hybrid mode, check the configuration
        self.config.dynamic_drivers.iter().any(|s| s == id)
    }

    /// Get the underlying plugin loader (for advanced use)
    pub fn plugin_loader(&self) -> &PluginLoader {
        &self.plugin_loader
    }

    /// Get mutable access to the underlying plugin loader
    pub fn plugin_loader_mut(&mut self) -> &mut PluginLoader {
        &mut self.plugin_loader
    }

    /// Get discovered plugin info
    pub fn get_plugin_info(&self, id: &str) -> Option<&DiscoveredPlugin> {
        self.plugin_loader.get_info(id)
    }

    /// Get search paths
    pub fn search_paths(&self) -> &[PathBuf] {
        self.plugin_loader.search_paths()
    }
}

impl Default for DriverLoader {
    fn default() -> Self {
        Self::with_defaults()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::driver::DriverCategory;
    use crate::plugin::types::{BackendInfo, PluginHealth, PluginManifest, ProbeResult};
    use std::collections::HashMap;

    struct TestPlugin {
        id: String,
    }

    impl DriverPlugin for TestPlugin {
        fn manifest(&self) -> PluginManifest {
            PluginManifest {
                id: self.id.clone(),
                name: format!("Test Plugin {}", self.id),
                version: "1.0.0".into(),
                category: DriverCategory::Sensor,
                description: "Test plugin".into(),
                author: None,
                license: None,
                repository: None,
                backends: vec![BackendInfo {
                    id: "test-backend".into(),
                    name: "Test Backend".into(),
                    description: "Test".into(),
                    hardware_ids: vec![],
                }],
                system_deps: vec![],
                horus_version: ">=0.1.0".into(),
                platforms: vec!["linux".into(), "macos".into(), "windows".into()],
                features: vec![],
                extra: HashMap::new(),
            }
        }

        fn probe(&self, _backend_id: &str) -> Vec<ProbeResult> {
            vec![ProbeResult::detected("test-backend", "/dev/test0")]
        }

        fn create(
            &self,
            backend_id: &str,
            _config: &SingleDriverConfig,
        ) -> HorusResult<Box<dyn Any + Send + Sync>> {
            Ok(Box::new(format!("TestDriver<{}/{}>", self.id, backend_id)))
        }

        fn health(&self) -> PluginHealth {
            PluginHealth::healthy(&self.id)
        }
    }

    #[test]
    fn test_driver_mode_default() {
        assert_eq!(DriverMode::default(), DriverMode::Static);
    }

    #[test]
    fn test_driver_mode_display() {
        assert_eq!(format!("{}", DriverMode::Static), "static");
        assert_eq!(format!("{}", DriverMode::Dynamic), "dynamic");
        assert_eq!(format!("{}", DriverMode::Hybrid), "hybrid");
    }

    #[test]
    fn test_driver_loader_new() {
        let config = DriverLoaderConfig::default();
        let loader = DriverLoader::new(config);
        assert!(!loader.is_initialized());
        assert_eq!(loader.mode(), DriverMode::Static);
    }

    #[test]
    fn test_static_registration() {
        let mut loader = DriverLoader::with_defaults();
        loader.register_static(Box::new(TestPlugin {
            id: "test-imu".into(),
        }));

        loader.initialize().unwrap();

        let drivers = loader.available_drivers();
        assert!(drivers.contains(&"test-imu".to_string()));
    }

    #[test]
    fn test_create_driver() {
        let mut loader = DriverLoader::with_defaults();
        loader.register_static(Box::new(TestPlugin {
            id: "test-imu".into(),
        }));
        loader.initialize().unwrap();

        let config = SingleDriverConfig::default();
        let driver = loader
            .create_driver("test-imu", "test-backend", &config)
            .unwrap();

        let driver_str: &String = driver.downcast_ref().unwrap();
        assert_eq!(driver_str, "TestDriver<test-imu/test-backend>");
    }

    #[test]
    fn test_hybrid_mode_config() {
        let config = DriverLoaderConfig {
            mode: DriverMode::Hybrid,
            search_paths: vec![],
            static_drivers: vec!["imu".to_string(), "motor".to_string()],
            dynamic_drivers: vec!["camera".to_string()],
            auto_discover: false,
        };

        let loader = DriverLoader::new(config);
        assert!(loader.is_static_driver("imu"));
        assert!(loader.is_static_driver("motor"));
        assert!(!loader.is_static_driver("camera"));
        assert!(loader.is_dynamic_driver("camera"));
        assert!(!loader.is_dynamic_driver("imu"));
    }

    #[test]
    fn test_probe_hardware() {
        let mut loader = DriverLoader::with_defaults();
        loader.register_static(Box::new(TestPlugin {
            id: "test-sensor".into(),
        }));
        loader.initialize().unwrap();

        let results = loader.probe("test-sensor").unwrap();
        assert_eq!(results.len(), 1);
        assert!(results[0].detected);
        assert_eq!(results[0].backend_id, "test-backend");
    }

    #[test]
    fn test_health_check() {
        let mut loader = DriverLoader::with_defaults();
        loader.register_static(Box::new(TestPlugin {
            id: "test-driver".into(),
        }));
        loader.initialize().unwrap();

        let health = loader.health();
        assert!(health.contains_key("test-driver"));
        assert!(health["test-driver"].healthy);
    }
}
