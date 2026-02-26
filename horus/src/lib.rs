//! # HORUS - Hybrid Optimized Robotics Unified System
//!
//! HORUS provides a comprehensive framework for building robotics applications in Rust,
//! with a focus on performance, safety, and developer experience.
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use horus::prelude::*;
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
//! ## Usage
//!
//! Import everything you need from the prelude:
//!
//! ```rust
//! use horus::prelude::*;
//! ```
//!
//! The prelude provides all user-facing types: nodes, topics, schedulers,
//! message types, actions, transforms, memory types, and macros.

// === Internal plumbing (hidden from docs, used by horus_py / macro-generated code / horus_manager) ===
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
#[doc(hidden)]
pub use horus_core::hlog;
#[doc(hidden)]
pub use horus_library as library;
#[doc(hidden)]
pub use horus_core::types;

/// The HORUS prelude — everything you need for building robotics applications.
///
/// ```rust
/// use horus::prelude::*;
/// ```
///
/// This is the **only import** you need. All user-facing types, traits,
/// macros, and message definitions are included.
pub mod prelude {
    // === Node ===
    pub use horus_core::core::{
        HealthStatus, LogSummary, NetworkStatus, Node, NodeInfo, NodeMetrics, NodePresence,
        NodeState, TopicMetadata,
    };

    // === Topic (IPC) ===
    pub use horus_core::communication::{PodMessage, Topic};

    // === Scheduler ===
    pub use horus_core::scheduling::Scheduler;

    // === Memory (domain types) ===
    pub use horus_core::memory::{DepthImage, Image, PointCloud, TensorHandle, TensorPool};

    // === HFrame (coordinate transforms) ===
    pub use horus_library::hframe::{timestamp_now, HFrame, HFrameConfig, Transform};

    // === Message types (all standard robotics messages) ===
    pub use horus_library::messages::*;
    pub use horus_core::types::{Device, ImageEncoding, PointXYZ, PointXYZI, PointXYZRGB, TensorDtype};

    // === Actions ===
    pub use horus_core::actions::{
        Action, ActionClientBuilder, ActionClientNode, ActionError, ActionServerBuilder,
        ActionServerNode, CancelResponse, ClientGoalHandle, GoalId, GoalOutcome, GoalPriority,
        GoalResponse, GoalStatus, PreemptionPolicy, ServerGoalHandle, SyncActionClient,
    };

    // === Real-time ===
    pub use horus_core::core::{
        DeadlineMissPolicy, RtClass, RtConfig, RtConfigBuilder, RtDegradation, RtNode, RtPriority,
        RtStats,
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

    // === Std (commonly needed) ===
    pub use std::sync::{Arc, Mutex};
    pub use std::time::{Duration, Instant};
}

