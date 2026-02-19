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

// Re-export core components (avoiding conflicts)
pub use horus_core::{self, *};

// Re-export macros
#[cfg(feature = "macros")]
pub use horus_macros::*;

// Re-export standard library with alias
pub use horus_library as library;

// Re-export core types crate
pub use horus_types;

// Re-export serde at crate root for macro-generated code
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

    // ============================================
    // Scheduler Configuration Types
    // ============================================
    pub use horus_core::scheduling::{
        ConfigValue, FaultConfig, MonitoringConfig, RealTimeConfig, RecordingConfigYaml,
        ResourceConfig, TimingConfig,
    };

    // ============================================
    // Safety & Fault Tolerance
    // ============================================
    pub use horus_core::scheduling::{
        // BlackBox flight recorder
        create_shared_blackbox,
        BlackBox,
        BlackBoxEvent,
        // Circuit breaker
        CircuitBreaker,
        CircuitState,
        // Safety monitoring
        SafetyMonitor,
        SafetyState,
        SafetyStats,
        SharedBlackBox,
        WCETEnforcer,
        Watchdog,
    };

    // ============================================
    // Executors
    // ============================================
    pub use horus_core::scheduling::ParallelExecutor;

    // ============================================
    // Telemetry
    // ============================================
    pub use horus_core::scheduling::{TelemetryEndpoint, TelemetryManager};

    // ============================================
    // Node Tier
    // ============================================
    pub use horus_core::scheduling::NodeTier;

    // ============================================
    // Record/Replay
    // ============================================
    pub use horus_core::scheduling::{
        NodeRecorder, NodeRecording, NodeReplayer, NodeTickSnapshot, RecordingConfig,
        RecordingManager, SchedulerRecording,
    };

    // ============================================
    // Runtime (OS-level features)
    // ============================================
    pub use horus_core::scheduling::{
        apply_rt_optimizations, get_core_count, get_max_rt_priority, get_numa_node_count,
        lock_all_memory, set_thread_affinity,
    };

    // ============================================
    // Real-Time Configuration & Nodes
    // ============================================
    pub use horus_core::core::{
        // CPU affinity and isolation helpers
        detect_isolated_cpus,
        detect_nohz_full_cpus,
        get_rt_recommended_cpus,
        pin_thread_to_core,
        // RtConfig - System-level RT configuration
        prefault_stack,
        prefault_stack_linear,
        // RtNode - Node-level RT constraints
        DeadlineMissPolicy,
        RtApplyResult,
        RtClass,
        RtConfig,
        RtConfigBuilder,
        RtCpuInfo,
        RtDegradation,
        RtKernelInfo,
        RtNode,
        RtNodeWrapper,
        RtPriority,
        RtScheduler,
        RtStats,
        WCETViolation,
    };

    // ============================================
    // ML Model Registry
    // ============================================
    pub use horus_ai::ml::{ModelEntry, ModelLoader, ModelRegistry};

    // ============================================
    // Memory & Tensors
    // ============================================
    pub use horus_core::memory::{shm_base_dir, TensorHandle, TensorPool, TensorPoolConfig};

    // Domain-specific handles (RAII wrappers with rich API for data access)
    pub use horus_core::memory::{DepthImageHandle, ImageHandle, PointCloudHandle};

    // CUDA support (requires "cuda" feature)
    #[cfg(feature = "cuda")]
    pub use horus_core::memory::{cuda_available, cuda_device_count};

    // CUDA Tensor Pool & IPC (requires "cuda" feature)
    #[cfg(feature = "cuda")]
    pub use horus_core::memory::{
        // Low-level FFI for advanced usage
        cuda_ffi,
        CudaPoolStats,
        // Core CUDA tensor types
        CudaTensor,
        CudaTensorPool,
        CudaTensorPoolConfig,
        // Multi-GPU P2P support
        P2PAccessInfo,
        P2PManager,
    };

    // ============================================
    // HFrame Transform System
    // ============================================
    pub use horus_library::hframe::{timestamp_now, HFrame, HFrameConfig, Transform};

    // ============================================
    // Message Types (ALL from horus_library)
    // ============================================
    pub use horus_types::{Device, HorusTensor, TensorDtype};
    // Backward-compatible alias
    pub use horus_library::messages::tensor::TensorDevice;
    // Re-export all message types from horus_library
    pub use horus_library::messages::*;

    // ============================================
    // Error Types (clean aliases - no Horus prefix)
    // ============================================
    pub use horus_core::error::{Error, Result};
    // Backward compatibility - users can still use HorusError/HorusResult
    pub use horus_core::error::{HorusError, HorusResult};

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

    // hlog!() macro for node logging
    pub use horus_core::hlog;

    // ============================================
    // Common Traits
    // ============================================
    pub use serde::{Deserialize, Serialize};

    // Re-export anyhow for error handling
    pub use anyhow::{anyhow, bail, ensure, Context, Result as AnyResult};

    // ============================================
    // Actions (Long-running tasks with feedback)
    // ============================================
    pub use horus_core::actions::{
        Action, ActionClientBuilder, ActionClientNode, ActionError, ActionServerBuilder,
        ActionServerNode, CancelResponse, ClientGoalHandle, GoalId, GoalOutcome, GoalPriority,
        GoalResponse, GoalStatus, PreemptionPolicy, ServerGoalHandle, SyncActionClient,
    };

    // ============================================
    // Driver & Plugin System
    // ============================================
    pub use horus_core::driver::{
        Actuator, Driver, DriverCategory, DriverStatus, DriversConfig, Sensor, SingleDriverConfig,
    };
    pub use horus_core::plugin::{
        AutoDetectable, BackendHealth, BackendId, BackendInfo, DriverPlugin,
        PluginEntryFn, PluginError, PluginFeature, PluginHealth, PluginId, PluginManifest,
        PluginResult, ProbeResult, SystemDependency, PLUGIN_ENTRY_SYMBOL,
    };

    // ============================================
    // Node Infrastructure
    // ============================================
    // Core node types (Node, Topic) are re-exported from horus_core above.
    // Users implement their own nodes using these building blocks.
    // See horus_library::nodes module documentation for usage patterns.

}

/// Version information
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Get HORUS version
pub fn version() -> &'static str {
    VERSION
}
