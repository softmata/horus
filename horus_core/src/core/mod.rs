//! # Core types and traits for the HORUS framework
//!
//! This module contains the fundamental building blocks of the HORUS system:
//!
//! - **Node**: The base trait for all computational units in HORUS
//! - **NodeInfo**: Internal scheduler bookkeeping for node state tracking
//! - **Contracts**: Message schemas and validation for type-safe communication
//! - **Data Fields**: Structured data types for robotics sensors and actuators
//!
//! ## Node Lifecycle
//!
//! All HORUS nodes follow a consistent lifecycle:
//! 1. **Construction** - Node is created with configuration
//! 2. **Initialization** - `init()` is called to set up resources
//! 3. **Execution** - `tick()` is called repeatedly by the scheduler
//! 4. **Shutdown** - `shutdown()` is called to clean up resources

pub mod discovery;
pub mod hlog;
pub mod log_buffer;
pub mod node;
pub mod presence;
pub mod rt_config;
pub mod rt_node;

pub use discovery::{
    announce_started, announce_stopped, NodeAnnouncement, NodeEvent, DISCOVERY_TOPIC,
};
pub use log_buffer::{LogEntry, LogType, SharedLogBuffer, GLOBAL_LOG_BUFFER};
pub use node::{
    HealthStatus, LogSummary, NetworkStatus, Node, NodeConfig, NodeInfo, NodeMetrics, NodeState,
    TopicMetadata,
};
pub use presence::NodePresence;
pub use rt_config::{
    detect_isolated_cpus, detect_nohz_full_cpus, get_rt_recommended_cpus, pin_thread_to_core,
    prefault_stack, prefault_stack_linear, RtApplyResult, RtConfig, RtConfigBuilder, RtCpuInfo,
    RtDegradation, RtKernelInfo, RtScheduler,
};
pub use rt_node::{
    DeadlineMissPolicy, RtClass, RtNode, RtNodeWrapper, RtPriority, RtStats, WCETViolation,
};
