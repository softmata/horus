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
//! - **Actions**: Long-running tasks with progress feedback and cancellation
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
//!         self.output.send("Hello HORUS!".into());
//!     }
//! }
//! ```
//!
//! ## Actions
//!
//! Actions provide a pattern for long-running tasks with feedback:
//!
//! ```rust,ignore
//! use horus_core::action;
//!
//! action! {
//!     NavigateToGoal {
//!         goal { target_x: f64, target_y: f64 }
//!         feedback { distance_remaining: f64 }
//!         result { success: bool }
//!     }
//! }
//! ```

pub mod actions;
pub mod communication;
pub mod core;
pub mod driver;
pub mod error;
pub mod memory;
pub mod params;
pub mod scheduling;
pub mod terminal;
pub mod types;
pub(crate) mod utils;

// Re-export commonly used types for easy access
// Unified Topic API - the single way to create IPC channels
pub use communication::{PodMessage, Topic};
pub use core::{
    announce_started, announce_stopped, detect_isolated_cpus, detect_nohz_full_cpus,
    get_rt_recommended_cpus, pin_thread_to_core, HealthStatus, LogSummary, Node, NodeAnnouncement,
    NodeConfig, NodeEvent, NodeInfo, NodeMetrics, NodePresence, NodeState, RtApplyResult, RtConfig,
    RtConfigBuilder, RtCpuInfo, RtDegradation, RtKernelInfo, RtScheduler, TopicMetadata,
    DISCOVERY_TOPIC,
};
pub use error::{HorusError, HorusResult};
pub use params::RuntimeParams;
pub use scheduling::Scheduler;

// Re-export driver utilities
pub use driver::{DriverCategory, DriverStatus, DriversConfig, SingleDriverConfig};

// Re-export action types for easy access
pub use actions::{
    Action, ActionClientBuilder, ActionClientNode, ActionError, ActionServerBuilder,
    ActionServerNode, CancelResponse, ClientGoalHandle, GoalId, GoalOutcome, GoalPriority,
    GoalResponse, GoalStatus, PreemptionPolicy, ServerGoalHandle, SyncActionClient,
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
