//! # HORUS Scheduling System
//!
//! Orchestrates node execution with preset configurations:
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
//! // Nodes declare their execution needs
//! let mut scheduler = Scheduler::new().tick_hz(500.0);
//! scheduler.add(motor_ctrl).order(0).budget_us(500).rate_hz(1000.0).done();
//! scheduler.add(planner).order(5).compute().done();
//! scheduler.add(telemetry).order(10).async_io().rate_hz(1.0).done();
//! scheduler.run()?;
//! ```

pub mod config;
pub(crate) mod safety_monitor;
pub mod scheduler;
pub(crate) mod types;

// Low-level execution primitives (NodeRunner, TickResult)
pub(crate) mod primitives;

// Dedicated RT thread executor
pub(crate) mod rt_executor;

// Parallel compute thread pool executor
pub(crate) mod compute_executor;

// Event-driven executor for topic-triggered nodes
pub(crate) mod event_executor;

// Async I/O executor for I/O-bound nodes (tokio spawn_blocking)
pub(crate) mod async_executor;

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
        pub fn from_string(_s: &str) -> Self {
            Self::Local
        }
    }

    pub struct TelemetryManager;
    impl TelemetryManager {
        pub fn new(_ep: TelemetryEndpoint, _interval_ms: u64) -> Self {
            Self
        }
        pub fn set_scheduler_name(&mut self, _name: &str) {}
        pub fn counter(&mut self, _name: &str, _value: u64) {}
        pub fn gauge(&mut self, _name: &str, _value: f64) {}
        pub fn counter_with_labels(
            &mut self,
            _name: &str,
            _value: u64,
            _labels: HashMap<String, String>,
        ) {
        }
        pub fn gauge_with_labels(
            &mut self,
            _name: &str,
            _value: f64,
            _labels: HashMap<String, String>,
        ) {
        }
        pub fn should_export(&self) -> bool {
            false
        }
        pub fn export(&mut self) -> Result<(), String> {
            Ok(())
        }
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
        SchedulerStart {
            name: String,
            node_count: usize,
            config: String,
        },
        SchedulerStop {
            reason: String,
            total_ticks: u64,
        },
        NodeAdded {
            name: String,
            order: u32,
        },
        NodeTick {
            name: String,
            duration_us: u64,
            success: bool,
        },
        NodeError {
            name: String,
            error: String,
        },
        DeadlineMiss {
            name: String,
            deadline_us: u64,
            actual_us: u64,
        },
        BudgetViolation {
            name: String,
            budget_us: u64,
            actual_us: u64,
        },
        LearningComplete {
            duration_ms: u64,
            tier_summary: String,
        },
        EmergencyStop {
            reason: String,
        },
        Custom {
            category: String,
            message: String,
        },
        PodSnapshot {
            data: Vec<u8>,
            type_name: String,
            context: String,
        },
    }

    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct BlackBoxRecord {
        pub timestamp_us: u64,
        pub tick: u64,
        pub event: BlackBoxEvent,
    }

    pub struct BlackBox;
    impl BlackBox {
        pub fn new(_max_size_mb: usize) -> Self {
            Self
        }
        pub fn with_path(self, _dir: PathBuf) -> Self {
            self
        }
        pub fn with_wal_flush_interval(self, _interval: usize) -> Self {
            self
        }
        pub fn flush_wal(&mut self) {}
        pub fn record(&mut self, _event: BlackBoxEvent) {}
        pub fn tick(&mut self) {}
        pub fn events(&self) -> Vec<BlackBoxRecord> {
            Vec::new()
        }
        pub fn anomalies(&self) -> Vec<BlackBoxRecord> {
            Vec::new()
        }
        pub fn save(&self) -> std::io::Result<()> {
            Ok(())
        }
        pub fn load(&mut self) -> std::io::Result<()> {
            Ok(())
        }
        pub fn record_pod_snapshot<T: crate::communication::PodMessage>(
            &mut self,
            _msg: &T,
            _context: &str,
        ) {
        }
        pub fn read_pod_snapshot<T: crate::communication::PodMessage>(_data: &[u8]) -> Option<T> {
            None
        }
        pub fn clear(&mut self) {}
        pub fn len(&self) -> usize {
            0
        }
        pub fn is_empty(&self) -> bool {
            true
        }
    }
}

// Record/Replay system (internal plumbing; user-facing types re-exported below)
pub(crate) mod record_replay;

// Deterministic execution (internal)
pub(crate) mod deterministic;

// Node builder for fluent node configuration (re-exported via #[doc(hidden)] below)
pub(crate) mod node_builder;

// =========================================================================
// Public re-exports — these form the user-facing scheduling API.
// Everything else is pub(crate) or deeper.
// =========================================================================

// The only user-facing type is Scheduler.
pub use scheduler::Scheduler;

// Internal re-exports — accessible but hidden from rustdoc.
#[doc(hidden)]
pub use crate::core::rt_node::BudgetViolation;
#[doc(hidden)]
pub use blackbox::{BlackBox, BlackBoxEvent, BlackBoxRecord};
#[doc(hidden)]
pub use config::{RecordingConfigYaml, SchedulerConfig};
#[doc(hidden)]
pub use fault_tolerance::{FailureHandlerStats, FailurePolicy};
#[doc(hidden)]
pub use node_builder::{NodeBuilder, NodeRegistration};
#[doc(hidden)]
pub use record_replay::{
    diff_recordings, Breakpoint, BreakpointCondition, DebugEvent, DebugSessionState, DebuggerState,
    NodeRecording, NodeReplayer, NodeTickSnapshot, Recording, RecordingConfig, RecordingDiff,
    RecordingManager, ReplayDebugger, SchedulerRecording, WatchExpression, WatchType, WatchValue,
};
#[doc(hidden)]
pub use safety_monitor::{SafetyState, SafetyStats};
#[doc(hidden)]
pub use scheduler::RtFeatureDegradation;
#[doc(hidden)]
pub use types::ExecutionClass;
