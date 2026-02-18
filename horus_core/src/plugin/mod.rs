//! # HORUS Driver Plugin System
//!
//! This module provides the plugin infrastructure for HORUS drivers.
//! It enables both compile-time (static) and runtime (dynamic) driver loading.
//!
//! ## Overview
//!
//! The plugin system allows HORUS to:
//!
//! - **Static Mode**: Drivers compiled into the binary (zero overhead, maximum performance)
//! - **Dynamic Mode**: Drivers loaded at runtime via `dlopen` (flexibility, hot-reload)
//! - **Hybrid Mode**: Core drivers static, specialized drivers dynamic
//!
//! ## Key Types
//!
//! - [`DriverPlugin`]: Core trait all driver plugins must implement
//! - [`PluginManifest`]: Metadata describing a plugin (id, version, backends, deps)
//! - [`BackendInfo`]: Information about a specific driver backend
//! - [`ProbeResult`]: Hardware detection results
//! - [`PluginHealth`]: Plugin health monitoring
//! - [`PluginLoader`]: Runtime plugin loading and management
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────────┐
//! │                        DriverLoader                                  │
//! │  (Respects DriverMode config from horus.yaml)                       │
//! ├─────────────────────────────────────────────────────────────────────┤
//! │                                                                      │
//! │  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐  │
//! │  │  Static Plugins │    │  Dynamic Loader │    │  Hybrid Bridge  │  │
//! │  │                 │    │                 │    │                 │  │
//! │  │ horus_library/  │    │ ~/.horus/       │    │ Core: static    │  │
//! │  │ drivers/        │    │ plugins/        │    │ Extras: dynamic │  │
//! │  │ (compile-time)  │    │ (runtime)       │    │                 │  │
//! │  └─────────────────┘    └─────────────────┘    └─────────────────┘  │
//! │           │                      │                      │           │
//! │           └──────────────────────┴──────────────────────┘           │
//! │                                  │                                   │
//! │                                  ▼                                   │
//! │                    ┌─────────────────────────┐                       │
//! │                    │     DriverPlugin        │                       │
//! │                    │   (unified interface)   │                       │
//! │                    └─────────────────────────┘                       │
//! └─────────────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Creating a Plugin
//!
//! ```rust,ignore
//! use horus_core::plugin::*;
//! use horus_core::driver::{DriverCategory, SingleDriverConfig};
//!
//! pub struct MyPlugin;
//!
//! impl DriverPlugin for MyPlugin {
//!     fn manifest(&self) -> PluginManifest {
//!         PluginManifest {
//!             id: "my-plugin".into(),
//!             name: "My Plugin".into(),
//!             version: semver::Version::new(1, 0, 0),
//!             category: DriverCategory::Sensor,
//!             // ...
//!         }
//!     }
//!
//!     fn probe(&self, backend_id: &str) -> Vec<ProbeResult> {
//!         // Hardware detection logic
//!     }
//!
//!     fn create(&self, backend_id: &str, config: &SingleDriverConfig)
//!         -> horus_core::HorusResult<Box<dyn std::any::Any + Send + Sync>>
//!     {
//!         // Driver instantiation
//!     }
//!
//!     fn health(&self) -> PluginHealth {
//!         PluginHealth::healthy("my-plugin")
//!     }
//! }
//!
//! // For dynamic loading:
//! #[no_mangle]
//! pub extern "C" fn horus_driver_entry() -> Box<dyn DriverPlugin> {
//!     Box::new(MyPlugin)
//! }
//! ```
//!
//! ## Configuration
//!
//! In `horus.yaml`:
//!
//! ```yaml
//! driver_mode: hybrid  # static, dynamic, or hybrid
//!
//! plugins:
//!   search_paths:
//!     - ~/.horus/plugins
//!     - /usr/local/lib/horus/plugins
//!
//!   static:
//!     - imu
//!     - lidar
//!
//!   dynamic:
//!     - camera-fancy
//!     - motor-custom
//! ```

pub mod driver_loader;
pub mod loader;
pub mod traits;
pub mod types;

// Re-export all public types for convenience
pub use driver_loader::{DriverLoader, DriverLoaderConfig, DriverMode};
pub use loader::{DiscoveredPlugin, PluginLoader};
pub use traits::{AutoDetectable, DriverPlugin, PluginEntryFn, PLUGIN_ENTRY_SYMBOL};
pub use types::{
    BackendHealth, BackendId, BackendInfo, PluginError, PluginFeature, PluginHealth, PluginId,
    PluginManifest, PluginResult, ProbeResult, SystemDependency,
};
