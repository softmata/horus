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
    pub use horus_core::core::{LogSummary, Node, NodeInfo, NodeState};

    // ============================================
    // Communication (IPC) - Unified Topic API
    // ============================================
    pub use horus_core::communication::Topic;
    pub use horus_core::communication::PodMessage;

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
        SharedBlackBox,
        // Checkpointing
        Checkpoint,
        CheckpointManager,
        CheckpointMetadata,
        NodeCheckpoint,
        // Circuit breaker
        CircuitBreaker,
        CircuitState,
        // Redundancy/TMR voting
        FaultStats,
        RedundancyManager,
        RedundantValue,
        VoteResult,
        Voter,
        VotingStrategy,
        // Safety monitoring
        SafetyMonitor,
        SafetyState,
        SafetyStats,
        WCETEnforcer,
        Watchdog,
    };

    // ============================================
    // Advanced Executors
    // ============================================
    pub use horus_core::scheduling::{
        AsyncIOExecutor, AsyncResult, BackgroundExecutor, ParallelExecutor,
    };

    // ============================================
    // Profiling & Intelligence
    // ============================================
    // NOTE: Internal types (TierClassifier, RuntimeProfiler, DependencyGraph, ExecutionTier)
    // are not exported - use horus_core::scheduling directly if needed for advanced use.
    pub use horus_core::scheduling::{NodeProfile, NodeTier, OfflineProfiler, ProfileData, ProfileError};

    // ============================================
    // Record/Replay
    // ============================================
    pub use horus_core::scheduling::{
        NodeRecorder, NodeRecording, NodeReplayer, NodeTickSnapshot, RecordingConfig,
        RecordingManager, SchedulerRecording,
    };

    // ============================================
    // Telemetry
    // ============================================
    pub use horus_core::scheduling::{
        create_shared_telemetry, Metric, MetricValue, SharedTelemetry, TelemetryEndpoint,
        TelemetryManager, TelemetrySnapshot,
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
        // RtConfig - System-level RT configuration
        prefault_stack, prefault_stack_linear, RtApplyResult, RtConfig, RtConfigBuilder,
        RtDegradation, RtKernelInfo, RtScheduler,
        // CPU affinity and isolation helpers
        detect_isolated_cpus, detect_nohz_full_cpus, get_rt_recommended_cpus, pin_thread_to_core,
        RtCpuInfo,
        // RtNode - Node-level RT constraints
        DeadlineMissPolicy, RtClass, RtNode, RtNodeWrapper, RtPriority, RtStats, WCETViolation,
    };

    // ============================================
    // JIT Compilation
    // ============================================
    pub use horus_core::scheduling::JITCompiler;

    // ============================================
    // ML Model Registry
    // ============================================
    pub use horus_core::ml::{ModelEntry, ModelRegistry};

    // ============================================
    // Memory & Tensors
    // ============================================
    pub use horus_core::memory::{shm_base_dir, TensorHandle, TensorPool, TensorPoolConfig};

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
    pub use horus_library::messages::tensor::{HorusTensor, TensorDevice, TensorDtype};
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
    pub use horus_core::params::{ParamMetadata, RuntimeParams, ValidationRule};

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
    // Hardware Discovery
    // ============================================
    pub use horus_core::hardware::{
        CategoryFilter,
        DeviceCategory,
        // Device database
        DeviceDatabase,
        DeviceInfo,
        DiscoveredDevice,
        DiscoveryOptions,
        DiscoveryReport,
        DiscoverySummary,
        DriverMatch,
        GpuType,
        // Main discovery API
        HardwareDiscovery,
        MatchConfidence,
        // Platform detection
        Platform,
        PlatformCapabilities,
        PlatformDetector,
        SerialDiscovery,
        SerialPort,
        SerialPortType,
        // USB & Serial (all platforms)
        UsbDevice,
        UsbDiscovery,
    };

    // Linux-specific hardware discovery
    #[cfg(target_os = "linux")]
    pub use horus_core::hardware::{
        AudioCard,
        AudioDevice,
        AudioDeviceType,
        AudioDirection,
        // Audio
        AudioDiscovery,
        BluetoothAdapter,
        BluetoothAdapterType,
        BluetoothDevice,
        BluetoothDeviceClass,
        // Bluetooth
        BluetoothDiscovery,
        BluetoothState,
        Camera,
        // Camera
        CameraDiscovery,
        CameraType,
        // CAN
        CanDiscovery,
        CanInterface,
        CanInterfaceType,
        CanState,
        GpioChip,
        // GPIO
        GpioDiscovery,
        GpioLine,
        I2cBus,
        I2cDevice,
        // I2C
        I2cDiscovery,
        // Network
        NetworkDiscovery,
        NetworkInterface,
        NetworkInterfaceType,
        NetworkState,
        PwmChannel,
        PwmChip,
        // PWM
        PwmDiscovery,
        PwmPolarity,
        SpiBus,
        SpiChipSelect,
        // SPI
        SpiDiscovery,
        VideoFormat,
        WifiInfo,
    };

    // ============================================
    // Actions (Long-running tasks with feedback)
    // ============================================
    pub use horus_core::actions::{
        Action, ActionClientBuilder, ActionClientNode, ActionError, ActionServerBuilder,
        ActionServerNode, CancelResponse, ClientGoalHandle, GoalId, GoalOutcome, GoalPriority,
        GoalResponse, GoalStatus, PreemptionPolicy, ServerGoalHandle, SyncActionClient,
    };

    // ============================================
    // State Machines (Hierarchical FSM)
    // ============================================
    pub use horus_core::state_machines::{
        Event, EventPriority, SharedStateMachine, State, StateId, StateMachine,
        StateMachineBuilder, StateMachineError, Transition, TransitionResult,
    };

    // ============================================
    // Behavior Trees (Reactive task orchestration)
    // ============================================
    pub use horus_core::behavior_trees::{
        ActionNode, BTNode, BehaviorTree, BehaviorTreeBuilder, BehaviorTreeError, Blackboard,
        BlackboardValue, ConditionNode, DecoratorNode, DecoratorType, NodeId, NodeStatus,
        NodeType, ParallelNode, ParallelPolicy, ReactiveSelectorNode, ReactiveSequenceNode,
        SelectorNode, SequenceNode, SharedBehaviorTree, TickContext, TreeVisualizer,
    };

    // ============================================
    // Mission Planner (DAG-based goal sequencing)
    // ============================================
    pub use horus_core::mission_planner::{
        ExecutionContext, ExecutionStatus, GoalFailurePolicy, GoalSpec, GoalState,
        MissionEvent, MissionMetrics, MissionMode, MissionPlanner, MissionPlannerBuilder,
        MissionPlannerConfig, MissionPlannerError, MissionSpec, MissionState, Priority,
        RetryPolicy, SharedMissionPlanner, TaskCondition, TaskExecutor, TaskSpec, TaskState,
        // ID types (aliased to avoid collision with actions::GoalId)
        GoalId as MissionGoalId, MissionId, TaskId,
    };

    // ============================================
    // Driver & Plugin System
    // ============================================
    pub use horus_core::driver::{
        Actuator, Driver, DriverCategory, DriverStatus, DriversConfig, Sensor,
        SingleDriverConfig,
    };
    pub use horus_core::plugin::{
        AutoDetectable, BackendHealth, BackendId, BackendInfo, DriverPlugin, HotReloadable,
        PluginEntryFn, PluginError, PluginFeature, PluginHealth, PluginId, PluginManifest,
        PluginResult, ProbeResult, SystemDependency, PLUGIN_ENTRY_SYMBOL,
    };

    // ============================================
    // Communication Traits (Backend-agnostic)
    // ============================================
    pub use horus_core::communication::traits::{Channel, Publisher, Subscriber};

    // ============================================
    // Node Infrastructure
    // ============================================
    // Core node types (Node, NodeInfo, Topic) are re-exported from horus_core above.
    // Users implement their own nodes using these building blocks.
    // See horus_library::nodes module documentation for usage patterns.
}

/// Version information
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Get HORUS version
pub fn version() -> &'static str {
    VERSION
}
