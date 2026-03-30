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

#[doc(hidden)]
pub mod clock;
pub mod duration_ext;
#[doc(hidden)]
pub mod hlog;
#[doc(hidden)]
pub mod log_bridge;
#[doc(hidden)]
pub mod log_buffer;
pub mod node;
#[doc(hidden)]
pub mod presence;
#[doc(hidden)]
pub mod rt_config;
#[doc(hidden)]
pub mod rt_node;
#[doc(hidden)]
pub mod tick_context;
pub mod timer;

// Log buffer types — pub for hlog! macro expansion and CLI log viewing (horus log)
#[doc(hidden)]
pub use log_buffer::{
    start_log_file_drain, LogEntry, LogType, SharedLogBuffer, GLOBAL_ERROR_BUFFER,
    GLOBAL_LOG_BUFFER, GLOBAL_REMOTE_LOG_BUFFER,
};

pub use duration_ext::{DurationExt, Frequency};
#[doc(hidden)]
pub use node::NodeInfo;
pub use node::{HealthStatus, LogSummary, Node, NodeMetrics, NodeState, TopicMetadata};
#[doc(hidden)]
pub use presence::{validate_node_name, NodePresence};
#[doc(hidden)]
pub use rt_node::BudgetViolation;
pub use rt_node::{Miss, RtStats};
pub use timer::{Rate, Stopwatch};
