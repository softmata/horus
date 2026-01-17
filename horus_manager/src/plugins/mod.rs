//! HORUS Plugin System
//!
//! This module provides CLI plugin management for HORUS.
//! Plugins are packages that extend the `horus` CLI with new subcommands
//! or provide runtime-loadable drivers for hardware support.
//!
//! ## Architecture
//!
//! - **Plugin Registry**: Tracks installed plugins in `plugins.lock` files
//! - **Plugin Resolver**: Resolves plugins from project and global registries
//! - **Plugin Executor**: Discovers, verifies, and executes plugin binaries
//! - **Plugin Discovery**: Finds available plugins from local workspace and registry
//!
//! ## Storage Locations
//!
//! - Global: `~/.horus/plugins.lock` and `~/.horus/bin/`
//! - Project: `.horus/plugins.lock` and `.horus/bin/`
//!
//! ## Plugin Discovery
//!
//! Plugins are discovered from multiple sources:
//! 1. Local workspace (development plugins)
//! 2. SOFTMATA registry (official plugins)
//! 3. crates.io (community plugins)
//!
//! ## Runtime Plugin Loading
//!
//! Plugins can be loaded at runtime without recompilation:
//! - `horus plugins search camera` - Find camera plugins
//! - `horus plugins install horus-realsense` - Install a plugin
//! - Plugin is loaded dynamically via `~/.horus/plugins/`

mod discovery;
mod executor;
mod registry;
mod resolver;

pub use discovery::{AvailablePlugin, PluginCategory, PluginDiscovery, PluginSourceType};
pub use executor::{PluginExecutor, PluginInfo};
pub use registry::{
    CommandInfo, Compatibility, DisabledPlugin, PluginEntry, PluginRegistry, PluginScope,
    PluginSource,
};
pub use resolver::{PluginResolver, VerificationResult, VerificationStatus};

/// Current schema version for plugins.lock
pub const SCHEMA_VERSION: &str = "1.0";

/// HORUS version for compatibility tracking
pub const HORUS_VERSION: &str = env!("CARGO_PKG_VERSION");
