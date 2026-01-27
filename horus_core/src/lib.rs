//! # HORUS Core
//!
//! The core runtime system for the HORUS robotics framework.
//!
//! HORUS is a distributed real-time robotics system designed for high-performance
//! applications. This crate provides the fundamental building blocks:
//!
//! - **Nodes**: Independent computational units that process data
//! - **Communication**: Publisher-subscriber message passing between nodes
//! - **Memory**: High-performance shared memory and zero-copy messaging
//! - **Scheduling**: Real-time task scheduling and execution
//! - **Monitoring**: Cross-process system monitoring and diagnostics
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use horus_core::{Node, Scheduler, Topic, hlog};
//!
//! struct ExampleNode {
//!     output: Topic<String>,
//! }
//!
//! impl Node for ExampleNode {
//!     fn name(&self) -> &'static str { "example" }
//!
//!     fn tick(&mut self) {
//!         let _ = self.output.send("Hello HORUS!".into());
//!     }
//! }
//! ```
//!
pub mod communication;
pub mod config;
pub mod core;
pub mod driver;
pub mod error;
pub mod memory;
pub mod ml;
pub mod params;
pub mod plugin;
pub mod scheduling;
pub mod terminal;
pub mod types;

// Re-export commonly used types for easy access
// Unified Topic API - the single way to create IPC channels
pub use communication::{PodMessage, Topic};
pub use core::{
    announce_crashed, announce_started, announce_stopped, detect_isolated_cpus,
    detect_nohz_full_cpus, get_rt_recommended_cpus, pin_thread_to_core, read_announcements,
    HealthStatus, LogSummary, Node, NodeAnnouncement, NodeConfig, NodeEvent, NodeInfo,
    NodeMetrics, NodePresence, NodeState, RtApplyResult, RtConfig, RtConfigBuilder, RtCpuInfo,
    RtDegradation, RtKernelInfo, RtScheduler, TopicMetadata, DISCOVERY_TOPIC,
};
pub use error::{HorusError, HorusResult};
// Clean aliases for user-facing API
pub use error::{Error, Result};
pub use params::RuntimeParams;
pub use scheduling::Scheduler;

// Re-export communication traits for backend-agnostic usage
pub use communication::traits::{Channel, Publisher, Subscriber};

// Re-export driver utilities and traits
pub use driver::{
    Actuator, Driver, DriverCategory, DriverStatus, DriversConfig, Sensor, SingleDriverConfig,
};

// Re-export plugin types for driver plugin system
pub use plugin::{
    AutoDetectable, BackendHealth, BackendId, BackendInfo, DriverPlugin, HotReloadable,
    PluginEntryFn, PluginError, PluginFeature, PluginHealth, PluginId, PluginManifest,
    PluginResult, ProbeResult, SystemDependency, PLUGIN_ENTRY_SYMBOL,
};

// Re-export the paste crate for macro usage
pub use paste;

// hlog macro is available at crate root via #[macro_export]
// No need to re-export - it's already at horus_core::hlog

// Re-export serde_json for consistent type usage across crates
pub use serde_json;

// Re-export serde_yaml for consistent type usage across crates
pub use serde_yaml;

// Re-export bytemuck for consistent Pod/Zeroable trait usage
pub use bytemuck;

// Internal types used by macros (not part of public API)
#[doc(hidden)]
pub use types::FixedString;

