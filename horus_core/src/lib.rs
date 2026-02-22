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
pub mod dlpack;
pub mod error;
pub mod memory;
pub mod params;
pub mod scheduling;
pub(crate) mod terminal;
pub(crate) mod utils;

// Re-export user-facing types for easy access
pub use communication::{PodMessage, Topic};
pub use core::{
    DeadlineMissPolicy, HealthStatus, LogSummary, Node, NodeConfig,
    NodeMetrics, NodePresence, NodeState, RtClass, RtConfig, RtConfigBuilder, RtDegradation,
    RtNode, RtPriority, RtStats, TopicMetadata,
};
pub use error::{HorusError, HorusResult, Result};
pub use params::RuntimeParams;
pub use scheduling::Scheduler;

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
