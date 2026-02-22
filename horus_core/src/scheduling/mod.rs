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
//! let mut scheduler = Scheduler::new().with_config(SchedulerConfig::deploy());
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
pub(crate) mod safety_monitor;
pub mod scheduler;
pub(crate) mod types;

// Advanced execution modules — individual types re-exported below
pub(crate) mod fault_tolerance;

// Runtime profiler (moved from intelligence/)
pub(crate) mod profiler;

// Telemetry export for live monitoring (internal — TelemetryEndpoint re-exported below)
#[cfg(feature = "telemetry")]
pub(crate) mod telemetry;
#[cfg(not(feature = "telemetry"))]
pub(crate) mod telemetry {
    //! No-op telemetry stub (feature "telemetry" disabled).
    use std::collections::HashMap;

    #[derive(Debug, Clone)]
    pub enum TelemetryEndpoint {
        Local,
    }
    impl TelemetryEndpoint {
        pub fn from_string(_s: &str) -> Self { Self::Local }
    }

    pub struct TelemetryManager;
    impl TelemetryManager {
        pub fn new(_ep: TelemetryEndpoint, _interval_ms: u64) -> Self { Self }
        pub fn set_scheduler_name(&mut self, _name: &str) {}
        pub fn counter(&mut self, _name: &str, _value: u64) {}
        pub fn gauge(&mut self, _name: &str, _value: f64) {}
        pub fn counter_with_labels(&mut self, _name: &str, _value: u64, _labels: HashMap<String, String>) {}
        pub fn gauge_with_labels(&mut self, _name: &str, _value: f64, _labels: HashMap<String, String>) {}
        pub fn should_export(&self) -> bool { false }
        pub fn export(&mut self) -> Result<(), String> { Ok(()) }
    }
}

// Runtime OS-level features + capability detection (merged)
pub(crate) mod rt;

// Flight recorder (internal — BlackBox/BlackBoxEvent re-exported below)
#[cfg(feature = "blackbox")]
pub(crate) mod blackbox;
#[cfg(not(feature = "blackbox"))]
pub(crate) mod blackbox {
    //! No-op blackbox stub (feature "blackbox" disabled).
    use serde::{Deserialize, Serialize};
    use std::path::PathBuf;

    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub enum BlackBoxEvent {
        SchedulerStart { name: String, node_count: usize, config: String },
        SchedulerStop { reason: String, total_ticks: u64 },
        NodeAdded { name: String, priority: u32 },
        NodeTick { name: String, duration_us: u64, success: bool },
        NodeError { name: String, error: String },
        DeadlineMiss { name: String, deadline_us: u64, actual_us: u64 },
        WCETViolation { name: String, budget_us: u64, actual_us: u64 },
        CircuitBreakerChange { name: String, new_state: String, failure_count: u32 },
        LearningComplete { duration_ms: u64, tier_summary: String },
        EmergencyStop { reason: String },
        Custom { category: String, message: String },
    }

    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct BlackBoxRecord {
        pub timestamp_us: u64,
        pub tick: u64,
        pub event: BlackBoxEvent,
    }

    pub struct BlackBox;
    impl BlackBox {
        pub fn new(_max_size_mb: usize) -> Self { Self }
        pub fn with_path(self, _dir: PathBuf) -> Self { self }
        pub fn record(&mut self, _event: BlackBoxEvent) {}
        pub fn tick(&mut self) {}
        pub fn events(&self) -> Vec<BlackBoxRecord> { Vec::new() }
        pub fn anomalies(&self) -> Vec<BlackBoxRecord> { Vec::new() }
        pub fn save(&self) -> std::io::Result<()> { Ok(()) }
        pub fn load(&mut self) -> std::io::Result<()> { Ok(()) }
        pub fn clear(&mut self) {}
        pub fn len(&self) -> usize { 0 }
        pub fn is_empty(&self) -> bool { true }
    }

}

// Record/Replay system
pub mod record_replay;

// Deterministic execution (internal)
pub(crate) mod deterministic;

// Node builder for fluent node configuration
pub mod node_builder;

pub use crate::core::rt_node::WCETViolation;
pub use config::{ExecutionMode, RecordingConfigYaml, SchedulerConfig};
pub use safety_monitor::{SafetyState, SafetyStats};
pub use scheduler::{RtDegradation, Scheduler};
pub use types::SchedulerNodeMetrics;

// Re-export blackbox flight recorder
pub use blackbox::BlackBox;
pub use blackbox::BlackBoxEvent;

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

// Re-export fault tolerance
// User-facing: FailurePolicy (NodeBuilder), CircuitState/FailureHandlerStats (introspection)
// Power-user: FailureHandler/FailureAction (direct fault handling in tests/tools)
pub use fault_tolerance::{
    CircuitState, FailureAction, FailureHandler, FailureHandlerStats, FailurePolicy,
};

// Re-export node builder
pub use node_builder::{NodeBuilder, NodeRegistration};
