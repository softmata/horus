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

// Discovery internals (used by scheduler, not user-facing)
pub(crate) use discovery::{announce_started, announce_stopped};

// LogType must be pub for hlog! macro expansion in downstream crates
pub use log_buffer::LogType;

pub use node::{
    HealthStatus, LogSummary, NetworkStatus, Node, NodeConfig, NodeInfo, NodeMetrics, NodeState,
    TopicMetadata,
};
pub use presence::NodePresence;
pub use rt_config::{RtConfig, RtConfigBuilder, RtDegradation};
pub use rt_node::{DeadlineMissPolicy, RtClass, RtNode, RtPriority, RtStats, WCETViolation};
