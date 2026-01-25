//! # HORUS Core
//!
//! The core runtime system for the HORUS robotics framework.
//!
//! HORUS is a distributed real-time robotics system designed for high-performance
//! applications. This crate provides the fundamental building blocks:
//!
//! - **Nodes**: Independent computational units that process data
//! - **Communication**: Publisher-subscriber message passing between nodes
//! - **Memory**: High-performance shared memory and zero-copy messaging
//! - **Scheduling**: Real-time task scheduling and execution
//! - **Monitoring**: Cross-process system monitoring and diagnostics
//! - **Actions**: Long-running tasks with progress feedback and cancellation
//! - **State Machines**: Hierarchical finite state machines for mode management
//! - **Behavior Trees**: Reactive task orchestration for complex robot behaviors
//! - **Mission Planner**: Goal sequencing with DAG-based task dependencies
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use horus_core::{Node, Scheduler, Topic, hlog};
//!
//! struct ExampleNode {
//!     output: Topic<String>,
//! }
//!
//! impl Node for ExampleNode {
//!     fn name(&self) -> &'static str { "example" }
//!
//!     fn tick(&mut self) {
//!         let _ = self.output.send("Hello HORUS!".into());
//!     }
//! }
//! ```
//!
//! ## Actions
//!
//! Actions provide a pattern for long-running tasks with feedback:
//!
//! ```rust,ignore
//! use horus_core::action;
//!
//! action! {
//!     NavigateToGoal {
//!         goal { target_x: f64, target_y: f64 }
//!         feedback { distance_remaining: f64 }
//!         result { success: bool }
//!     }
//! }
//! ```

pub mod actions;
pub mod behavior_trees;
pub mod communication;
pub mod config;
pub mod core;
pub mod driver;
pub mod error;
pub mod hardware;
pub mod memory;
pub mod mission_planner;
pub mod ml;
pub mod params;
pub mod plugin;
pub mod scheduling;
pub mod state_machines;
pub mod terminal;
pub mod types;

// Re-export commonly used types for easy access
// Unified Topic API - the single way to create IPC channels
pub use communication::{PodMessage, Topic};
pub use core::{
    announce_crashed, announce_started, announce_stopped, detect_isolated_cpus,
    detect_nohz_full_cpus, get_rt_recommended_cpus, pin_thread_to_core, read_announcements,
    HealthStatus, LogSummary, Node, NodeAnnouncement, NodeConfig, NodeEvent, NodeInfo,
    NodeMetrics, NodePresence, NodeState, RtApplyResult, RtConfig, RtConfigBuilder, RtCpuInfo,
    RtDegradation, RtKernelInfo, RtScheduler, TopicMetadata, DISCOVERY_TOPIC,
};
pub use error::{HorusError, HorusResult};
// Clean aliases for user-facing API
pub use error::{Error, Result};
pub use params::RuntimeParams;
pub use scheduling::Scheduler;

// Re-export communication traits for backend-agnostic usage
pub use communication::traits::{Channel, Publisher, Subscriber};

// Re-export driver utilities and traits
pub use driver::{
    Actuator, Driver, DriverCategory, DriverStatus, DriversConfig, Sensor, SingleDriverConfig,
};

// Re-export action types for easy access
pub use actions::{
    Action, ActionClientBuilder, ActionClientNode, ActionError, ActionServerBuilder,
    ActionServerNode, CancelResponse, ClientGoalHandle, GoalId, GoalOutcome, GoalPriority,
    GoalResponse, GoalStatus, PreemptionPolicy, ServerGoalHandle, SyncActionClient,
};

// Re-export state machine types for easy access
pub use state_machines::{
    Event, EventPriority, SharedStateMachine, State, StateId, StateMachine, StateMachineBuilder,
    StateMachineError, Transition, TransitionResult,
};

// Re-export behavior tree types for easy access
pub use behavior_trees::{
    ActionNode, BTNode, BehaviorTree, BehaviorTreeBuilder, BehaviorTreeError, Blackboard,
    BlackboardValue, ConditionNode, DecoratorNode, DecoratorType, NodeId, NodeStatus, NodeType,
    ParallelNode, ParallelPolicy, ReactiveSelectorNode, ReactiveSequenceNode, SelectorNode,
    SequenceNode, SharedBehaviorTree, TickContext, TreeVisualizer,
};

// Re-export mission planner types for easy access
pub use mission_planner::{
    ExecutionContext, ExecutionStatus, GoalFailurePolicy, GoalSpec, GoalState, MissionEvent,
    MissionMetrics, MissionMode, MissionPlanner, MissionPlannerBuilder, MissionPlannerConfig,
    MissionPlannerError, MissionSpec, MissionState, Priority, RetryPolicy, SharedMissionPlanner,
    TaskCondition, TaskExecutor, TaskSpec, TaskState,
};
// Mission planner ID types (aliased to avoid collision with actions::GoalId)
pub use mission_planner::{GoalId as MissionGoalId, MissionId, TaskId};

// Re-export plugin types for driver plugin system
pub use plugin::{
    AutoDetectable, BackendHealth, BackendId, BackendInfo, DriverPlugin, HotReloadable,
    PluginEntryFn, PluginError, PluginFeature, PluginHealth, PluginId, PluginManifest,
    PluginResult, ProbeResult, SystemDependency, PLUGIN_ENTRY_SYMBOL,
};

// Re-export the paste crate for macro usage
pub use paste;

// hlog macro is available at crate root via #[macro_export]
// No need to re-export - it's already at horus_core::hlog

// Re-export serde_json for consistent type usage across crates
pub use serde_json;

// Re-export serde_yaml for consistent type usage across crates
pub use serde_yaml;

// Re-export bytemuck for consistent Pod/Zeroable trait usage
pub use bytemuck;

// Internal types used by macros (not part of public API)
#[doc(hidden)]
pub use types::FixedString;

