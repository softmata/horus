//! # HORUS - Hybrid Optimized Robotics Unified System
//!
//! HORUS provides a comprehensive framework for building robotics applications in Rust,
//! with a focus on performance, safety, and developer experience.
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use horus::prelude::*;
//! use horus::library::messages::cmd_vel::CmdVel;
//!
//! pub struct MyNode {
//!     publisher: Topic<CmdVel>,
//! }
//!
//! impl Node for MyNode {
//!     fn name(&self) -> &'static str { "MyNode" }
//!
//!     fn tick(&mut self) {
//!         // Node logic here
//!     }
//! }
//! ```
//!
//! ## Features
//!
//! - **Zero-copy IPC** with multiple backend support
//! - **Type-safe message passing**
//! - **Built-in monitoring and debugging**
//! - **Standard library of components**
//! - **Comprehensive tooling**

// === User-facing re-exports ===
pub use horus_core::communication::{PodMessage, Topic};
pub use horus_core::core::{
    DeadlineMissPolicy, HealthStatus, LogSummary, Node, NodeMetrics, NodePresence,
    NodeState, RtClass, RtConfig, RtConfigBuilder, RtDegradation, RtNode, RtPriority, RtStats,
};
pub use horus_core::error::{HorusError, HorusResult, Result};
pub use horus_core::params::RuntimeParams;
pub use horus_core::scheduling::Scheduler;

// Action types
pub use horus_core::actions::{
    Action, ActionClientBuilder, ActionClientNode, ActionError, ActionServerBuilder,
    ActionServerNode, CancelResponse, ClientGoalHandle, GoalId, GoalOutcome, GoalPriority,
    GoalResponse, GoalStatus, PreemptionPolicy, ServerGoalHandle, SyncActionClient,
};

// hlog macro
pub use horus_core::hlog;

// Re-export macros
#[cfg(feature = "macros")]
pub use horus_macros::*;

// Re-export standard library with alias
pub use horus_library as library;

// Re-export core types crate
pub use horus_types;

// === Internal plumbing (hidden from docs, used by horus_py / macro-generated code) ===
#[doc(hidden)]
pub use horus_core;
#[doc(hidden)]
pub use horus_core::actions;
#[doc(hidden)]
pub use horus_core::bytemuck;
#[doc(hidden)]
pub use horus_core::communication;
#[doc(hidden)]
pub use horus_core::core;
#[doc(hidden)]
pub use horus_core::dlpack;
#[doc(hidden)]
pub use horus_core::error;
#[doc(hidden)]
pub use horus_core::memory;
#[doc(hidden)]
pub use horus_core::params;
#[doc(hidden)]
pub use horus_core::paste;
#[doc(hidden)]
pub use horus_core::scheduling;
#[doc(hidden)]
pub use horus_core::serde_json;
#[doc(hidden)]
pub use horus_core::serde_yaml;
#[doc(hidden)]
pub use serde;

/// The HORUS prelude — the essentials for building robotics applications.
///
/// `use horus::prelude::*;` gives you everything needed for typical use.
///
/// Advanced types (RT nodes, recording, blackbox, telemetry, tensor pools)
/// are available via qualified paths: `horus::RtNode`, `horus::scheduling::BlackBox`, etc.
pub mod prelude {
    // === Node ===
    pub use horus_core::core::{LogSummary, Node, NodeState};

    // === Topic (IPC) ===
    pub use horus_core::communication::Topic;

    // === Scheduler ===
    pub use horus_core::scheduling::{
        ExecutionMode, FailurePolicy, NodeTier, Scheduler, SchedulerConfig,
    };

    // === Memory (domain types) ===
    pub use horus_core::memory::{DepthImage, Image, PointCloud};

    // === HFrame (coordinate transforms) ===
    pub use horus_library::hframe::{timestamp_now, HFrame, HFrameConfig, Transform};

    // === Message types ===
    pub use horus_library::messages::*;
    pub use horus_types::{Device, ImageEncoding, PointXYZ, PointXYZI, PointXYZRGB, TensorDtype};

    // === Actions ===
    pub use horus_core::actions::{
        Action, ActionClientBuilder, ActionClientNode, ActionError, ActionServerBuilder,
        ActionServerNode, ClientGoalHandle, GoalId, GoalResponse, GoalStatus, ServerGoalHandle,
    };

    // === Parameters ===
    pub use horus_core::params::RuntimeParams;

    // === Errors ===
    pub use horus_core::error::{HorusError, HorusResult, Result};

    // === Macros ===
    pub use horus_core::hlog;
    #[cfg(feature = "macros")]
    pub use horus_macros::*;
    pub use serde::{Deserialize, Serialize};

    // === Std ===
    pub use std::sync::{Arc, Mutex};
    pub use std::time::{Duration, Instant};
}

/// Version information
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Get HORUS version
pub fn version() -> &'static str {
    VERSION
}
