//! # Core types and traits for the HORUS framework
//!
//! This module contains the fundamental building blocks of the HORUS system:
//!
//! - **Node**: The base trait for all computational units in HORUS
//! - **NodeContext**: Runtime context and utilities provided to nodes during execution
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

pub mod log_buffer;
pub mod node;
pub mod node_info_ext;
pub mod rt_config;
pub mod rt_node;

pub use log_buffer::{LogEntry, LogType, SharedLogBuffer, GLOBAL_LOG_BUFFER};
pub use node::{
    HealthStatus, LogSummary, NetworkStatus, Node, NodeConfig, NodeHeartbeat, NodeInfo,
    NodeMetrics, NodeState, TopicMetadata,
};
pub use node_info_ext::NodeInfoExt;
pub use rt_config::{
    detect_isolated_cpus, detect_nohz_full_cpus, get_rt_recommended_cpus, pin_thread_to_core,
    prefault_stack, prefault_stack_linear, RtApplyResult, RtConfig, RtConfigBuilder,
    RtCpuInfo, RtDegradation, RtKernelInfo, RtScheduler,
};
pub use rt_node::{
    DeadlineMissPolicy, RtClass, RtNode, RtNodeWrapper, RtPriority, RtStats, WCETViolation,
};
