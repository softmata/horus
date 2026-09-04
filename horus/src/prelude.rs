//! The HORUS prelude — everything you need for building robotics applications.
//!
//! ```rust
//! use horus::prelude::*;
//! ```
//!
//! This is the **only import** you need. All user-facing types, traits,
//! macros, and message definitions are included.

// === Node ===
pub use horus_core::core::{HealthStatus, LogSummary, Node, NodeState};

// === Real-time node ===
pub use horus_core::core::rt_config::{
    RtApplyResult, RtConfig, RtConfigBuilder, RtDegradation, RtKernelInfo, RtScheduler,
};
pub use horus_core::core::{DurationExt, Frequency, Miss, RtStats};

// === Rate / Stopwatch ===
pub use horus_core::core::timer::{Rate, Stopwatch};

// === Topic (IPC) ===
pub use horus_core::communication::Topic;

// === Scheduler ===
pub use horus_core::scheduling::Scheduler;

// === Execution configuration ===
pub use horus_core::scheduling::FailurePolicy;

// === Runtime parameters ===
pub use horus_core::params::RuntimeParams;

// === Coordinate transforms ===
// C++ gets `horus::TransformFrame` straight from <horus/horus.hpp> and
// horus_py exposes the same tree, because both crates depend on horus-tf.
// The Rust umbrella crate did not, so `horus::prelude` offered
// `TransformStamped` — the message — but not the frame tree that produces
// it, and a Rust user had to add a pinned git dependency by hand to do
// what the other two languages do out of the box.
pub use horus_tf::{FrameBuilder, Transform, TransformFrame, TransformFrameConfig};

// === Memory (domain types) ===
pub use horus_core::memory::{DepthImage, Image, PointCloud};

// === Universal IPC types (math, diagnostics, time, generic) ===
pub use horus_types::prelude::*;

// === Types (core) ===
/// Internal tensor dtype — prefer `PointCloud::from_xyz()`, `DepthImage::meters()` etc.
#[doc(hidden)]
pub use horus_core::types::TensorDtype;
pub use horus_core::types::{Device, ImageEncoding, PointXYZ, PointXYZI, PointXYZRGB};

// === Standard robotics messages ===
// Kept in the unified prelude so a normal `horus` application can use
// CmdVel, Imu, LaserScan, BatteryState, and related wire types without a
// second explicit dependency.
pub use horus_robotics::prelude::*;

// === Actions ===
pub use horus_core::actions::{
    Action, ActionClient, ActionClientBuilder, ActionClientNode, ActionError, ActionResult,
    ActionServerBuilder, ActionServerNode, CancelResponse, ClientGoalHandle, GoalId, GoalOutcome,
    GoalPriority, GoalResponse, GoalStatus, PreemptionPolicy, ServerGoalHandle,
};

// === Services (request/response RPC) ===
pub use horus_core::services::{
    AsyncServiceClient, Service, ServiceClient, ServiceError, ServiceRequest, ServiceResponse,
    ServiceResult, ServiceServer, ServiceServerBuilder,
};

// === Errors ===
//
// `Error` and `Result` are short aliases (for use inside application code).
// `HorusError` is also exported so callers can pattern-match exhaustively:
//   `use horus::prelude::*;`
//   `if let Err(HorusError::InvalidDescriptor(msg)) = result { ... }`
pub use horus_core::error::{
    retry_transient, CommunicationError, ConfigError, Error, HorusContext, HorusError, MemoryError,
    NodeError, NotFoundError, ParseError, ResourceError, Result, RetryConfig, SerializationError,
    Severity, TimeoutError, TransformError, ValidationError,
};

// === Time API ===
pub use crate::time::TimeStamp;

// === Macros ===
pub use horus_core::action;
pub use horus_core::hlog;
pub use horus_core::hlog_every;
pub use horus_core::hlog_once;
pub use horus_core::message;
pub use horus_core::service;
pub use horus_core::standard_action;
#[cfg(feature = "macros")]
pub use horus_macros::*;
