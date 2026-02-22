//! # HORUS Scheduling System
//!
//! Simple, unified scheduling system that orchestrates node execution:
//!
//! - **Scheduler**: Unified scheduler with built-in monitoring integration
//! - **Simple Priorities**: Numeric priorities (0 = highest)
//! - **Progressive Disclosure**: `new()` is lightweight; opt in to features via builders
//!
//! ## Usage
//!
//! ```rust,ignore
//! use horus_core::Scheduler;
//!
//! // Development — lightweight, no syscalls
//! let mut scheduler = Scheduler::new();
//! scheduler.add(sensor_node).order(0).done();
//! scheduler.add(control_node).order(1).done();
//! scheduler.run()?;
//!
//! // Production — RT features + flight recorder
//! let mut scheduler = Scheduler::deploy();
//! scheduler.add(motor_ctrl).order(0).rt().done();
//! scheduler.run()?;
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
pub mod fault_tolerance;

// Runtime profiler (moved from intelligence/)
pub(crate) mod profiler;

// Telemetry export for live monitoring
pub mod telemetry;

// Runtime OS-level features + capability detection (merged)
pub(crate) mod rt;

// Flight recorder
pub mod blackbox;

// Record/Replay system
pub mod record_replay;

// Deterministic execution (internal)
pub(crate) mod deterministic;

// Node builder for fluent node configuration
pub mod node_builder;

pub use crate::core::rt_node::WCETViolation;
pub use config::{
    ExecutionMode, MonitoringConfig, RealTimeConfig, RecordingConfigYaml,
    ResourceConfig, SchedulerConfig, TimingConfig,
};
pub use safety_monitor::{SafetyState, SafetyStats};
pub use scheduler::{RtDegradation, Scheduler};
pub use types::SchedulerNodeMetrics;

// Re-export blackbox flight recorder
pub use blackbox::{BlackBox, BlackBoxEvent};

// Re-export record/replay (public API)
pub use record_replay::{
    diff_recordings, Breakpoint, BreakpointCondition, DebugEvent, DebugSessionState, DebuggerState,
    NodeRecorder, NodeRecording, NodeReplayer, NodeTickSnapshot, Recording, RecordingDiff,
    RecordingManager, ReplayDebugger, SchedulerRecording, WatchExpression, WatchType, WatchValue,
};

// Re-export node tier from types
pub use types::NodeTier;

// Re-export telemetry (only user-facing endpoint type)
pub use telemetry::TelemetryEndpoint;

// Re-export fault tolerance (public API)
pub use fault_tolerance::{
    CircuitBreaker, CircuitState, FailureHandler, FailureHandlerStats, FailurePolicy,
};

// Re-export node builder
pub use node_builder::{NodeBuilder, NodeRegistration};
