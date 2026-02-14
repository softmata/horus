//! # Communication layer for HORUS
//!
//! This module provides the unified Topic API for all HORUS IPC needs.
//!
//! ## Topic - The Unified Communication API
//!
//! `Topic<T>` provides a single, consistent interface for all communication patterns.
//! It automatically selects the optimal backend based on topology and access patterns:
//!
//! | Backend | Latency | Detection Criteria |
//! |---------|---------|-------------------|
//! | DirectChannel | ~3ns | same_thread |
//! | SpscIntra | ~18ns | same_process, pubs=1, subs=1 |
//! | SpmcIntra | ~24ns | same_process, pubs=1, subs>1 |
//! | MpscIntra | ~26ns | same_process, pubs>1, subs=1 |
//! | MpmcIntra | ~36ns | same_process, pubs>1, subs>1 |
//! | PodShm | ~50ns | cross_process, is_pod |
//! | SpscShm | ~85ns | cross_process, pubs=1, subs=1, !is_pod |
//! | MpmcShm | ~167ns | cross_process, pubs>1, subs>1 |
//!
//! ## Usage
//!
//! ```rust,ignore
//! use horus_core::communication::Topic;
//!
//! // Just this - backend auto-selected
//! let topic: Topic<String> = Topic::new("sensor_data")?;
//! topic.send("hello".to_string());
//! let msg = topic.recv();
//! ```
//!
//! ## Automatic POD Detection
//!
//! HORUS automatically detects POD types - no registration needed!
//!
//! ```rust,ignore
//! struct MotorCommand { velocity: f32, torque: f32 }
//!
//! // HORUS automatically uses zero-copy path (~50ns)!
//! let topic: Topic<MotorCommand> = Topic::new("motor")?;
//! topic.send(MotorCommand { velocity: 1.0, torque: 0.5 });
//! ```

pub mod topic;
pub mod config;
pub mod network;
pub mod pod;
pub mod storage;
pub mod traits;

// Re-export commonly used types for convenience
pub use topic::{
    BackendHint, BackendMode, ConnectionState, MigrationMetrics, MigrationResult, Topic,
    TopicConfig, TopicDescriptor, TopicMetrics, TopicRole,
};
pub use config::{EndpointConfig, HorusConfig};
pub use pod::{is_pod, PodMessage};
pub use storage::AccessMode;
pub use traits::{Channel, Publisher, Subscriber};
