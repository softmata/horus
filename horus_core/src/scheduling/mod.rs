//! # HORUS Scheduling System
//!
//! Orchestrates node execution with composable builder methods:
//!
//! ```rust,ignore
//! use horus_core::Scheduler;
//!
//! // Development — lightweight, no syscalls
//! let mut scheduler = Scheduler::new();
//! scheduler.add(sensor_node).order(0).build();
//! scheduler.add(control_node).order(1).build();
//! scheduler.run()?;
//!
//! // Nodes declare their execution needs
//! let mut scheduler = Scheduler::new().tick_rate(500_u64.hz());
//! scheduler.add(motor_ctrl).order(0).rate(1000_u64.hz()).build();
//! scheduler.add(planner).order(5).compute().build();
//! scheduler.add(telemetry).order(10).async_io().rate(1_u64.hz()).build();
//! scheduler.run()?;
//! ```

pub(crate) mod config;
pub mod control;
pub(crate) mod dependency_graph;
pub mod registry;
pub(crate) mod safety_monitor;
pub(crate) mod scheduler;
pub(crate) mod types;

// Low-level execution primitives (NodeRunner, TickResult)
pub(crate) mod primitives;

// Dedicated RT thread executor
pub(crate) mod rt_executor;

/// Cyclic-wait accounting for the RT tick grid — the runtime's own deadline
/// numbers, exported so tests and reports can assert on them.
pub use rt_executor::{rt_wait_stats, RtWaitSnapshot};

// RT readiness report — system audit + jitter benchmark
pub mod rt_report;

// What the RT tick threads actually got from the kernel, as opposed to what
// was asked for. `rt_report` above audits the *system*; this reports the
// *running scheduler's own threads*.
pub mod rt_bandwidth;
pub mod rt_status;

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

    /// Mirror of the real `BlackBoxEvent` in `blackbox.rs`.
    ///
    /// The call sites that build these events (schedulers, executors, horus_net)
    /// are *not* feature-gated — they construct `BlackBoxEvent` unconditionally and
    /// let `record()` throw it away when the feature is off.  That makes this stub a
    /// silent shadow of the real enum: any variant or field added there but not here
    /// compiles fine in the default build and only explodes in a
    /// `--no-default-features` build (E0559/E0599), typically much later in CI.
    /// Keep the two definitions field-for-field identical.
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
            severity: crate::error::Severity,
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
        NetPeerDiscovered {
            peer_addr: String,
            topic_count: usize,
        },
        NetPeerLost {
            peer_addr: String,
            reason: String,
        },
        NetReplicationStarted {
            peer_count: usize,
        },
        NetImportRejected {
            topic: String,
            peer_addr: String,
        },
        Custom {
            category: String,
            message: String,
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
        pub(crate) fn tick(&mut self) {}
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
        pub fn clear(&mut self) {}
        pub fn len(&self) -> usize {
            0
        }
        pub fn is_empty(&self) -> bool {
            true
        }
        /// Always 0 — the stub never buffers, so it can never evict a record.
        pub fn get_loss_count(&self) -> u64 {
            0
        }
    }

    impl Default for BlackBox {
        fn default() -> Self {
            Self::new(0) // Disabled by default
        }
    }

    /// No-op: set a blackbox hook (feature "blackbox" disabled).
    pub fn set_blackbox_hook(_hook: impl Fn(BlackBoxEvent) + Send + Sync + 'static) {}

    /// No-op: record an external event (feature "blackbox" disabled).
    pub fn record_external_event(_event: BlackBoxEvent) {}
}

// Record/Replay system (internal plumbing; user-facing types re-exported below)
pub(crate) mod record_replay;

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
pub use blackbox::{
    record_external_event, set_blackbox_hook, BlackBox, BlackBoxEvent, BlackBoxRecord,
};
#[doc(hidden)]
pub use config::{RecordingConfigYaml, SchedulerConfig};
#[doc(hidden)]
pub use fault_tolerance::{FailureAction, FailureHandler, FailureHandlerStats, FailurePolicy};
#[doc(hidden)]
pub use node_builder::{NodeBuilder, NodeRegistration};
#[doc(hidden)]
pub use record_replay::{
    diff_recordings, Breakpoint, BreakpointCondition, DebugEvent, DebugSessionState, DebuggerState,
    NodeRecording, NodeReplayer, NodeTickSnapshot, Recording, RecordingConfig, RecordingDiff,
    RecordingManager, ReplayDebugger, SchedulerRecording, WatchExpression, WatchType, WatchValue,
};
#[doc(hidden)]
pub use registry::SchedulerRegistry;
/// The best-effort thread class. Re-exported here so `horus_net`, `horus_cpp`
/// and anything else that spawns a thread inside a scheduler process can reach
/// it without depending on the `scheduling::rt` module path.
pub use rt::{enter_best_effort, spawn_best_effort};
pub use rt_bandwidth::{advisory, classify, LoopDuty, RtBandwidthVerdict};
#[doc(hidden)]
pub use safety_monitor::{
    set_emergency_stop_hook, set_safe_state_hook, take_pending_local_estop,
    trigger_external_emergency_stop, trigger_external_safe_state, BudgetPolicy, SafetyState,
    SafetyStats,
};
#[doc(hidden)]
pub use scheduler::{set_network_auto_wire, LifecycleStartFn, RtFeatureDegradation};
#[doc(hidden)]
pub use types::ExecutionClass;
pub use types::StalePolicy;
