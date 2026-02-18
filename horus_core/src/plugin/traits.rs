//! Driver plugin traits for HORUS
//!
//! This module defines the core trait that all driver plugins must implement.
//! Plugins can be loaded statically (compile-time) or dynamically (runtime).
//!
//! # Static vs Dynamic Loading
//!
//! - **Static**: Plugin is compiled into the binary. Zero runtime overhead.
//! - **Dynamic**: Plugin is loaded at runtime via `dlopen`. Enables hot-reloading.
//!
//! # Example: Implementing a Plugin
//!
//! ```rust,ignore
//! use horus_core::plugin::{DriverPlugin, PluginManifest, ProbeResult, PluginHealth};
//! use horus_core::driver::{DriverCategory, SingleDriverConfig};
//! use horus_core::error::HorusResult;
//! use std::any::Any;
//!
//! pub struct Mpu6050Plugin;
//!
//! impl DriverPlugin for Mpu6050Plugin {
//!     fn manifest(&self) -> PluginManifest {
//!         PluginManifest {
//!             id: "horus-imu-mpu6050".into(),
//!             name: "MPU6050 IMU Driver".into(),
//!             // ... rest of manifest
//!         }
//!     }
//!
//!     fn probe(&self, backend_id: &str) -> Vec<ProbeResult> {
//!         // Scan I2C bus for MPU6050 at 0x68 or 0x69
//!         scan_i2c_for_mpu6050()
//!     }
//!
//!     fn create(&self, backend_id: &str, config: &SingleDriverConfig)
//!         -> HorusResult<Box<dyn Any + Send + Sync>>
//!     {
//!         let driver = Mpu6050Driver::new(config)?;
//!         Ok(Box::new(driver))
//!     }
//!
//!     fn health(&self) -> PluginHealth {
//!         PluginHealth::healthy("horus-imu-mpu6050")
//!     }
//! }
//!
//! // Export for dynamic loading
//! #[no_mangle]
//! pub extern "C" fn horus_driver_entry() -> Box<dyn DriverPlugin> {
//!     Box::new(Mpu6050Plugin)
//! }
//! ```

use crate::driver::SingleDriverConfig;
use crate::error::HorusResult;

use super::types::{BackendId, PluginHealth, PluginManifest, ProbeResult};

use std::any::Any;

/// Core trait for driver plugins
///
/// All HORUS driver plugins must implement this trait. It provides:
/// - Plugin metadata via `manifest()`
/// - Hardware detection via `probe()`
/// - Driver instantiation via `create()`
/// - Health monitoring via `health()`
///
/// # Thread Safety
///
/// The trait requires `Send + Sync` for use in multi-threaded contexts.
/// Plugin implementations should be thread-safe.
///
/// # Dynamic Loading
///
/// For dynamic plugins, export the entry point:
///
/// ```rust,ignore
/// #[no_mangle]
/// pub extern "C" fn horus_driver_entry() -> Box<dyn DriverPlugin> {
///     Box::new(MyPlugin)
/// }
/// ```
pub trait DriverPlugin: Send + Sync {
    /// Get the plugin manifest with all metadata
    ///
    /// The manifest describes the plugin, its backends, dependencies,
    /// and compatibility requirements.
    fn manifest(&self) -> PluginManifest;

    /// List available backend IDs
    ///
    /// Returns the IDs of all backends provided by this plugin.
    /// This is a convenience method; the same information is in the manifest.
    fn available_backends(&self) -> Vec<BackendId> {
        self.manifest()
            .backend_ids()
            .into_iter()
            .map(Into::into)
            .collect()
    }

    /// Probe for hardware
    ///
    /// Scans for hardware that this plugin can drive. Returns a list of
    /// `ProbeResult` for each detected (or not detected) device.
    ///
    /// # Arguments
    ///
    /// * `backend_id` - Specific backend to probe, or empty string for all
    ///
    /// # Returns
    ///
    /// A list of probe results, one per potential device location.
    fn probe(&self, backend_id: &str) -> Vec<ProbeResult>;

    /// Create a driver instance
    ///
    /// Instantiates a driver for the specified backend with the given config.
    /// Returns a type-erased `Box<dyn Any>` that should be downcast to the
    /// specific driver type.
    ///
    /// # Arguments
    ///
    /// * `backend_id` - Which backend to use (e.g., "mpu6050", "bno055")
    /// * `config` - Driver configuration from YAML/TOML
    ///
    /// # Returns
    ///
    /// A boxed driver instance that can be downcast to the concrete type.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let driver_any = plugin.create("mpu6050", &config)?;
    /// let driver: Box<ImuDriver> = driver_any
    ///     .downcast()
    ///     .map_err(|_| HorusError::driver("Type mismatch"))?;
    /// ```
    fn create(
        &self,
        backend_id: &str,
        config: &SingleDriverConfig,
    ) -> HorusResult<Box<dyn Any + Send + Sync>>;

    /// Get plugin health status
    ///
    /// Returns the current health of the plugin and its backends.
    /// Use this to check if the plugin is operational.
    fn health(&self) -> PluginHealth;

    /// Called when the plugin is loaded
    ///
    /// Override to perform initialization (e.g., check dependencies).
    /// Default implementation does nothing.
    fn on_load(&self) -> HorusResult<()> {
        Ok(())
    }

    /// Called when the plugin is unloaded
    ///
    /// Override to perform cleanup. Default implementation does nothing.
    fn on_unload(&self) {
        // Default: nothing to clean up
    }
}

/// Type alias for the plugin entry point function
///
/// Dynamic plugins must export this function:
///
/// ```rust,ignore
/// #[no_mangle]
/// pub extern "C" fn horus_driver_entry() -> Box<dyn DriverPlugin> {
///     Box::new(MyPlugin)
/// }
/// ```
#[allow(improper_ctypes_definitions)]
pub type PluginEntryFn = unsafe extern "C" fn() -> Box<dyn DriverPlugin>;

/// Name of the entry point symbol for dynamic loading
pub const PLUGIN_ENTRY_SYMBOL: &str = "horus_driver_entry";

/// Extension trait for plugins that support hardware auto-detection
pub trait AutoDetectable: DriverPlugin {
    /// Auto-detect and create the best available driver
    ///
    /// Probes all backends and creates a driver using the first detected hardware.
    fn auto_create(&self, config: &SingleDriverConfig) -> HorusResult<Box<dyn Any + Send + Sync>> {
        let results = self.probe("");

        // Find the first detected backend with highest confidence
        let best = results.iter().filter(|r| r.detected).max_by(|a, b| {
            a.confidence
                .partial_cmp(&b.confidence)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        match best {
            Some(result) => self.create(&result.backend_id, config),
            None => {
                let manifest = self.manifest();
                Err(crate::error::HorusError::driver(format!(
                    "No hardware detected for plugin '{}'. Available backends: {:?}",
                    manifest.name,
                    manifest.backend_ids()
                )))
            }
        }
    }
}

// Auto-implement AutoDetectable for all DriverPlugin implementations
impl<T: DriverPlugin + ?Sized> AutoDetectable for T {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::driver::DriverCategory;
    use crate::plugin::types::{BackendInfo, PluginManifest};
    use std::collections::HashMap;

    struct MockPlugin;

    impl DriverPlugin for MockPlugin {
        fn manifest(&self) -> PluginManifest {
            PluginManifest {
                id: "mock-plugin".into(),
                name: "Mock Plugin".into(),
                version: "1.0.0".into(),
                category: DriverCategory::Sensor,
                description: "A mock plugin for testing".into(),
                author: None,
                license: None,
                repository: None,
                backends: vec![
                    BackendInfo {
                        id: "mock-a".into(),
                        name: "Mock A".into(),
                        description: "First mock backend".into(),
                        hardware_ids: vec![],
                    },
                    BackendInfo {
                        id: "mock-b".into(),
                        name: "Mock B".into(),
                        description: "Second mock backend".into(),
                        hardware_ids: vec![],
                    },
                ],
                system_deps: vec![],
                horus_version: ">=0.5.0".into(),
                platforms: vec!["linux".into()],
                features: vec![],
                extra: HashMap::new(),
            }
        }

        fn probe(&self, backend_id: &str) -> Vec<ProbeResult> {
            if backend_id.is_empty() || backend_id == "mock-a" {
                vec![ProbeResult::detected("mock-a", "/dev/mock0")]
            } else {
                vec![ProbeResult::not_detected(backend_id)]
            }
        }

        fn create(
            &self,
            backend_id: &str,
            _config: &SingleDriverConfig,
        ) -> HorusResult<Box<dyn Any + Send + Sync>> {
            if backend_id == "mock-a" || backend_id == "mock-b" {
                Ok(Box::new(format!("MockDriver<{}>", backend_id)))
            } else {
                Err(crate::error::HorusError::driver(format!(
                    "Unknown backend: {}",
                    backend_id
                )))
            }
        }

        fn health(&self) -> PluginHealth {
            PluginHealth::healthy("mock-plugin")
        }
    }

    #[test]
    fn test_mock_plugin() {
        let plugin = MockPlugin;

        let manifest = plugin.manifest();
        assert_eq!(manifest.id, "mock-plugin");
        assert_eq!(manifest.backends.len(), 2);

        let backends = plugin.available_backends();
        assert_eq!(backends, vec!["mock-a", "mock-b"]);

        let probe_results = plugin.probe("mock-a");
        assert_eq!(probe_results.len(), 1);
        assert!(probe_results[0].detected);

        let config = SingleDriverConfig::default();
        let driver = plugin.create("mock-a", &config).unwrap();
        let driver_str: &String = driver.downcast_ref().unwrap();
        assert_eq!(driver_str, "MockDriver<mock-a>");

        let health = plugin.health();
        assert!(health.healthy);
    }

    #[test]
    fn test_auto_detectable() {
        let plugin = MockPlugin;
        let config = SingleDriverConfig::default();

        let driver = plugin.auto_create(&config).unwrap();
        let driver_str: &String = driver.downcast_ref().unwrap();
        assert_eq!(driver_str, "MockDriver<mock-a>");
    }
}
