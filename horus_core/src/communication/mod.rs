//! # Communication layer for HORUS
//!
//! This module provides the unified Topic API for all HORUS IPC needs.
//!
//! ## Topic - The Unified Communication API
//!
//! Topic<T> provides a single, consistent interface for all communication patterns:
//!
//! - **Auto mode** (default): Automatically selects the best backend based on message type
//! - **MPMC mode**: Multi-producer multi-consumer for general pub/sub (167-6994 ns/msg)
//! - **SPSC mode**: Single-producer single-consumer for point-to-point (85-167 ns/msg)
//! - **Pod mode**: Zero-copy POD-only for real-time control (~50 ns)
//!
//! ## Usage
//!
//! ```rust,ignore
//! use horus_core::communication::Topic;
//!
//! // Default MPMC mode - most flexible
//! let topic: Topic<String> = Topic::new("sensor_data").unwrap();
//! topic.send("hello".to_string(), &mut None)?;
//!
//! // SPSC mode - lowest latency for point-to-point
//! let topic: Topic<String, Spsc> = Topic::with_mode("control_cmd").unwrap();
//!
//! // Pod mode - zero-copy for POD types
//! let topic: Topic<CmdVel, Pod> = Topic::with_mode("motor_cmd").unwrap();
//! ```
//!
//! ## PodMessage Trait
//!
//! For ultra-low latency (~50ns), use POD types with the `PodMessage` trait:
//!
//! ```rust,ignore
//! use horus_core::communication::PodMessage;
//! use bytemuck::{Pod, Zeroable};
//!
//! #[repr(C)]
//! #[derive(Clone, Copy, Pod, Zeroable)]
//! pub struct MotorCommand {
//!     pub timestamp_ns: u64,
//!     pub velocity: f32,
//!     pub torque: f32,
//! }
//!
//! unsafe impl PodMessage for MotorCommand {}
//! ```

pub mod config;
pub mod network;
pub mod pod;
pub mod smart_backend;
pub mod smart_detect;
pub mod storage;
pub mod topic;
pub mod traits;

// Re-export commonly used types for convenience
pub use config::{EndpointConfig, HorusConfig};
pub use pod::{PodMessage, is_registered_pod, registered_pod_count, registered_pod_types};
pub use smart_detect::{SmartTopicHeader, DetectedPattern, RecommendedBackend, DetectionResult, smart_detect};
pub use storage::AccessMode;
pub use topic::{
    AccessPattern, Auto, BackendHint, ConnectionState, Mpmc, MpmcIntraBackend, MpscIntraBackend,
    Pod, ProcessTopology, SpmcIntraBackend, Spsc, SpscIntraBackend, Topic, TopicConfig,
    TopicMetrics, TopicMode,
};
pub use traits::{Channel, Publisher, Subscriber};
