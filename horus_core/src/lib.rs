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
//! **Note:** This is an internal crate. Users should depend on `horus` and
//! import from `horus::prelude::*`.

// Public modules — accessible cross-crate but hidden from user docs.
// Users should go through `horus::prelude`, not import from `horus_core` directly.
#[doc(hidden)]
pub mod actions;
#[doc(hidden)]
pub mod communication;
#[doc(hidden)]
pub mod core;
#[doc(hidden)]
pub mod dlpack;
#[doc(hidden)]
pub mod error;
#[doc(hidden)]
pub mod memory;
#[doc(hidden)]
pub mod params;
#[doc(hidden)]
pub mod scheduling;
pub(crate) mod terminal;
pub(crate) mod utils;

// Crate-internal re-exports (used by `crate::HorusError` etc. within this crate,
// and by horus_py / macro-generated code cross-crate).
#[doc(hidden)]
pub use communication::{PodMessage, Topic};
#[doc(hidden)]
pub use core::{
    DeadlineMissPolicy, HealthStatus, LogSummary, Node, NodeMetrics, NodePresence,
    NodeState, RtClass, RtConfig, RtConfigBuilder, RtDegradation, RtNode, RtPriority, RtStats,
    TopicMetadata,
};
#[doc(hidden)]
pub use error::{HorusError, HorusResult, Result};
#[doc(hidden)]
pub use params::RuntimeParams;
#[doc(hidden)]
pub use scheduling::Scheduler;
#[doc(hidden)]
pub use actions::{
    Action, ActionClientBuilder, ActionClientNode, ActionError, ActionServerBuilder,
    ActionServerNode, CancelResponse, ClientGoalHandle, GoalId, GoalOutcome, GoalPriority,
    GoalResponse, GoalStatus, PreemptionPolicy, ServerGoalHandle, SyncActionClient,
};

// Re-export dependencies used by macro-generated code and horus_py
#[doc(hidden)]
pub use paste;
#[doc(hidden)]
pub use serde_json;
#[doc(hidden)]
pub use serde_yaml;
#[doc(hidden)]
pub use bytemuck;
