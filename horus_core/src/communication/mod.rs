//! # Communication layer for HORUS
//!
//! This module provides the unified Topic API for all HORUS IPC needs.
//!
//! ## Topic - The Unified Communication API
//!
//! `Topic<T>` provides a single, consistent interface for all communication patterns:
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
//! topic.send("hello".to_string())?;
//!
//! // SPSC mode - lowest latency for point-to-point
//! let topic: Topic<String, Spsc> = Topic::with_mode("control_cmd").unwrap();
//!
//! // Pod mode - zero-copy for POD types
//! let topic: Topic<CmdVel, Pod> = Topic::with_mode("motor_cmd").unwrap();
//! ```
//!
//! ## Automatic POD Detection
//!
//! HORUS automatically detects POD types - no registration needed!
//!
//! ```rust,ignore
//! // Just define your struct - HORUS auto-detects it as POD
//! struct MotorCommand {
//!     velocity: f32,
//!     torque: f32,
//! }
//!
//! // Use it directly - zero-copy path selected automatically
//! let topic: Topic<MotorCommand> = Topic::new("motor")?;
//! topic.send(MotorCommand { velocity: 1.0, torque: 0.5 })?;
//! ```
//!
//! Types are POD if `!std::mem::needs_drop::<T>()` - this automatically
//! excludes String, Vec, Box, and any type containing heap pointers.
//!
//! ## PodMessage Trait (Optional)
//!
//! For explicit POD types with bytemuck guarantees, you can still use `PodMessage`:
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

pub mod adaptive_topic;
pub mod config;
pub mod network;
pub mod pod;
pub mod smart_detect;
pub mod storage;
pub mod topic;
pub mod traits;

// Re-export commonly used types for convenience
pub use adaptive_topic::{
    AdaptiveBackendMode, AdaptiveMetrics, AdaptiveTopic, MigrationResult, TopicRole,
};
pub use config::{EndpointConfig, HorusConfig};
pub use pod::{is_pod, PodMessage};
pub use smart_detect::{
    smart_detect, DetectedPattern, DetectionResult, RecommendedBackend, SmartTopicHeader,
};
pub use storage::AccessMode;
pub use topic::{
    AccessPattern, Auto, BackendHint, ConnectionState, Mpmc, MpmcIntraBackend, MpscIntraBackend,
    Pod, ProcessTopology, SpmcIntraBackend, Spsc, SpscIntraBackend, Topic, TopicConfig,
    TopicMetrics, TopicMode,
};
pub use traits::{Channel, Publisher, Subscriber};
