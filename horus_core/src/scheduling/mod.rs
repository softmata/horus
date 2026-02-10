//! # HORUS Scheduling System
//!
//! Simple, unified scheduling system that orchestrates node execution:
//!
//! - **Scheduler**: Unified scheduler with built-in monitoring integration
//! - **Simple Priorities**: Numeric priorities (0 = highest)
//! - **Optional Logging**: Per-node logging configuration
//!
//! ## Usage
//!
//! ```rust,ignore
//! use horus_core::Scheduler;
//!
//! let mut scheduler = Scheduler::new();
//! scheduler.add(Box::new(sensor_node), 10);      // High priority
//! scheduler.add(Box::new(control_node), 20);     // Medium priority
//! scheduler.add(Box::new(background_node), 200); // Low priority
//! scheduler.run(); // Handles initialization automatically
//! ```
//!
//! ## Priority Levels
//!
//! - **0-99**: High priority (real-time, sensors, control)
//! - **100-199**: Normal priority (processing, algorithms)
//! - **200+**: Background priority (logging, diagnostics)

pub mod config;
pub mod safety_monitor;
pub mod scheduler;
pub mod types;

// Advanced execution modules
pub mod executors;
pub mod fault_tolerance;
mod intelligence;

// Runtime OS-level features
pub mod runtime;

// Fault tolerance and monitoring
pub mod blackbox;
pub mod checkpoint;
pub mod redundancy;
pub mod telemetry;

// AI-powered debugging
pub mod ai_debug;

// Record/Replay system
pub mod record_replay;

// Zero-copy high-performance recording
pub mod zero_copy_recording;

// Deterministic execution
pub mod deterministic;

// Runtime capability detection for auto-optimization
pub mod capabilities;

// Builder pattern for explicit scheduler configuration
pub mod builder;

// Node builder for fluent node configuration
pub mod node_builder;

// Distributed multi-process/robot recording
pub mod distributed_recording;

// Expose async_io module for AsyncNode
pub mod async_io {
    pub use super::executors::async_io::AsyncNode;
}

pub use config::{
    ConfigValue, ExecutionMode, FaultConfig, MonitoringConfig, RealTimeConfig, RecordingConfigYaml,
    ResourceConfig, SchedulerConfig, TimingConfig,
};
pub use safety_monitor::{SafetyMonitor, SafetyState, SafetyStats, WCETEnforcer, Watchdog};
pub use scheduler::{DegradationSeverity, RtDegradation, RtFeature, Scheduler};
pub use types::{NodeControlCommand, SchedulerNodeMetrics};

// Re-export runtime features
pub use runtime::{
    apply_rt_optimizations, get_core_count, get_max_rt_priority, get_numa_node_count,
    lock_all_memory, set_thread_affinity,
};

// Re-export fault tolerance
pub use blackbox::{create_shared_blackbox, BlackBox, BlackBoxEvent, SharedBlackBox};
pub use checkpoint::{Checkpoint, CheckpointManager, CheckpointMetadata, NodeCheckpoint};
pub use redundancy::{
    FaultStats, RedundancyManager, RedundantValue, VoteResult, Voter, VotingStrategy,
};
pub use telemetry::{
    create_shared_telemetry, Metric, MetricValue, SharedTelemetry, TelemetryEndpoint,
    TelemetryManager, TelemetrySnapshot,
};

// Re-export record/replay
pub use record_replay::{
    compress_data,
    decompress_data,
    diff_recordings,
    load_recording_compressed,
    save_recording_compressed,
    AutoRecordConfig,
    AutoRecordTrigger,
    // Auto-recording
    AutoRecorder,
    Breakpoint,
    BreakpointCondition,
    DebugEvent,
    DebugSessionState,
    DebuggerState,
    NodeRecorder,
    NodeRecording,
    NodeReplayer,
    NodeTickSnapshot,
    RecordingConfig,
    RecordingDiff,
    RecordingManager,
    // Advanced debugging
    ReplayDebugger,
    ReplayMode,
    ReplayNode,
    SchedulerRecording,
    TriggerCondition,
    WatchExpression,
    WatchType,
    WatchValue,
};

// Re-export intelligence (profiling, classification)
// NOTE: Internal types (TierClassifier, RuntimeProfiler, DependencyGraph, ExecutionTier)
// are intentionally NOT exported - they are implementation details used by scheduler.rs.
// Use NodeTier for developer-facing tier annotation.
pub use intelligence::{NodeProfile, NodeTier, OfflineProfiler, ProfileData, ProfileError};

// Re-export executors
pub use executors::{
    AsyncIOExecutor, AsyncResult, BackgroundExecutor, IsolatedExecutor, IsolatedNodeConfig,
    IsolatedNodeStats, IsolatedResult, ParallelExecutor,
};

// Re-export fault tolerance
pub use fault_tolerance::{CircuitBreaker, CircuitState};

// Re-export zero-copy recording
pub use zero_copy_recording::{
    DataFileHeader, EntryData, EntryHeader, IndexEntry, RecordingMetadata, RecordingStats,
    StreamingRecorder, TickData, TopicTable, ZeroCopyError, ZeroCopyRecorder, ZeroCopyRecording,
    ZeroCopyReplayer,
};

// Re-export deterministic execution
pub use deterministic::{
    DeterministicClock,
    DeterministicConfig,
    DeterministicScheduler,
    DivergenceInfo,
    ExecutionTrace,
    NodeTimingStats,
    // WCET timing analysis
    TimingBounds,
    TimingStatistics,
    TimingViolation,
    TraceEntry,
    TraceEntryType,
    ViolationSeverity,
};

// Re-export distributed recording
pub use distributed_recording::{
    DistributedError, DistributedEvent, DistributedEventType, FleetEvent, FleetEventRef,
    FleetRecorder, HybridReplayConfig, ProcessRecorder, RecordingCoordinator, RobotRecording,
    SnapshotResult, VectorClock,
};

// Re-export AI debug assistant
pub use ai_debug::{
    AnalysisContext, AnalysisResult, AnalysisStats, AssistantConfig, DebugAssistant, DetectedIssue,
    IssueCategory, PatternDetector, Severity, TimelineEvent, TimelineEventType,
};

// Re-export runtime capabilities
pub use capabilities::RuntimeCapabilities;

// Re-export builder
pub use builder::{CircuitBreakerConfig, SchedulerBuilder};

// Re-export node builder
pub use node_builder::{NodeBuilder, NodeConfig};
