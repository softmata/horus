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
    DeadlineMissPolicy, HealthStatus, LogSummary, Node, NodeConfig,
    NodeMetrics, NodePresence, NodeState, RtClass, RtConfig, RtConfigBuilder,
    RtDegradation, RtNode, RtPriority, RtStats,
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
pub use horus_core::communication;
#[doc(hidden)]
pub use horus_core::core;
#[doc(hidden)]
pub use horus_core::error;
#[doc(hidden)]
pub use horus_core::memory;
#[doc(hidden)]
pub use horus_core::params;
#[doc(hidden)]
pub use horus_core::scheduling;
#[doc(hidden)]
pub use horus_core::dlpack;
#[doc(hidden)]
pub use horus_core::paste;
#[doc(hidden)]
pub use horus_core::serde_json;
#[doc(hidden)]
pub use horus_core::serde_yaml;
#[doc(hidden)]
pub use horus_core::bytemuck;
#[doc(hidden)]
pub use serde;

/// The HORUS prelude - everything you need to get started
///
/// This includes all core types, advanced features, and commonly used components.
/// Just add `use horus::prelude::*;` to get started.
pub mod prelude {
    // ============================================
    // Core Node Types
    // ============================================
    pub use horus_core::core::node::NodeConfig;
    pub use horus_core::core::{LogSummary, Node, NodeState};

    // ============================================
    // Communication (IPC) - Unified Topic API
    // ============================================
    pub use horus_core::communication::PodMessage;
    pub use horus_core::communication::Topic;

    // ============================================
    // Scheduling
    // ============================================
    pub use horus_core::scheduling::{ExecutionMode, Scheduler, SchedulerConfig};

    // Note: Infrastructure config sub-structs (TimingConfig, RealTimeConfig,
    // MonitoringConfig, ResourceConfig, RecordingConfigYaml) are intentionally
    // NOT in the prelude. Access them via horus::scheduling::* when customizing
    // presets. Most users only need SchedulerConfig presets directly.

    // ============================================
    // Safety & Fault Tolerance
    // ============================================
    pub use horus_core::scheduling::{
        BlackBox, BlackBoxEvent, CircuitState, SafetyState, SafetyStats,
        WCETViolation,
    };

    // ============================================
    // Telemetry
    // ============================================
    pub use horus_core::scheduling::TelemetryEndpoint;

    // ============================================
    // Node Tier
    // ============================================
    pub use horus_core::scheduling::NodeTier;

    // ============================================
    // Record/Replay
    // ============================================
    pub use horus_core::scheduling::{
        NodeRecorder, NodeRecording, NodeReplayer, NodeTickSnapshot, RecordingManager,
        SchedulerRecording,
    };

    // ============================================
    // Real-Time Nodes
    // ============================================
    pub use horus_core::core::{
        DeadlineMissPolicy, RtClass, RtDegradation, RtNode, RtPriority, RtStats,
    };

    // Note: RtConfig and RtConfigBuilder are for thread-level RT setup.
    // Access them via horus::RtConfig or horus::core::rt_config::* when needed.

    // ============================================
    // Memory & Tensors
    // ============================================
    pub use horus_core::memory::{TensorHandle, TensorPool, TensorPoolConfig};

    // Domain-specific types (RAII wrappers with rich API for data access)
    pub use horus_core::memory::{DepthImage, Image, PointCloud};

    // CUDA support (requires "cuda" feature)
    #[cfg(feature = "cuda")]
    pub use horus_core::memory::{cuda_available, cuda_device_count};

    // CUDA Tensor Pool & IPC (requires "cuda" feature)
    #[cfg(feature = "cuda")]
    pub use horus_core::memory::{
        cuda_ffi, CudaPoolStats, CudaTensor, CudaTensorPool, CudaTensorPoolConfig, P2PAccessInfo,
        P2PManager,
    };

    // ============================================
    // HFrame Transform System
    // ============================================
    pub use horus_library::hframe::{timestamp_now, HFrame, HFrameConfig, Transform};

    // ============================================
    // Message Types (ALL from horus_library)
    // ============================================
    pub use horus_library::messages::*;
    pub use horus_types::{Device, HorusTensor, ImageEncoding, PointXYZ, PointXYZI, PointXYZRGB, TensorDtype};
    pub use horus_core::communication::TopicMessage;

    // ============================================
    // Error Types
    // ============================================
    pub use horus_core::error::{HorusError, HorusResult, Result};

    // ============================================
    // Common Std Types
    // ============================================
    pub use std::sync::{Arc, Mutex};
    pub use std::time::{Duration, Instant};

    // ============================================
    // Runtime Parameters
    // ============================================
    pub use horus_core::params::{ParamMetadata, RuntimeParams};

    // ============================================
    // Macros
    // ============================================
    #[cfg(feature = "macros")]
    pub use horus_macros::*;

    pub use horus_core::hlog;

    // ============================================
    // Common Traits
    // ============================================
    pub use serde::{Deserialize, Serialize};

    // ============================================
    // Actions (Long-running tasks with feedback)
    // ============================================
    pub use horus_core::actions::{
        Action, ActionClientBuilder, ActionClientNode, ActionError, ActionServerBuilder,
        ActionServerNode, CancelResponse, ClientGoalHandle, GoalId, GoalOutcome, GoalPriority,
        GoalResponse, GoalStatus, PreemptionPolicy, ServerGoalHandle, SyncActionClient,
    };
}

/// Version information
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Get HORUS version
pub fn version() -> &'static str {
    VERSION
}
