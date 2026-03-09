//! # HORUS - Hybrid Optimized Robotics Unified System
//!
//! HORUS provides a comprehensive framework for building robotics applications in Rust,
//! with a focus on performance, safety, and developer experience.
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use horus::prelude::*;
//!
//! pub struct MyNode {
//!     publisher: Topic<CmdVel>,
//! }
//!
//! impl Node for MyNode {
//!     fn name(&self) -> &str { "MyNode" }
//!
//!     fn tick(&mut self) {
//!         // Node logic here
//!     }
//! }
//! ```
//!
//! ## Camera Image Example
//!
//! Send and receive images with zero-copy shared memory:
//!
//! ```rust,no_run
//! use horus::prelude::*;
//!
//! // Create a 640x480 RGB image (backed by shared memory)
//! let mut img = Image::new(640, 480, ImageEncoding::Rgb8).unwrap();
//! img.fill(&[0, 0, 255]);           // Blue
//! img.set_pixel(100, 200, &[255, 0, 0]); // Red dot
//!
//! // Send — only the 168-byte descriptor goes through the ring buffer
//! let topic: Topic<Image> = Topic::new("camera/rgb").unwrap();
//! topic.send(&img);
//!
//! // Receive (in another node or process)
//! if let Some(received) = topic.recv() {
//!     let px = received.pixel(100, 200); // Zero-copy read
//! }
//! ```
//!
//! ## Usage
//!
//! Import everything you need from the prelude:
//!
//! ```rust
//! use horus::prelude::*;
//! ```
//!
//! The prelude provides all user-facing types: nodes, topics, schedulers,
//! message types, actions, transforms, and domain types (`Image`,
//! `PointCloud`, `DepthImage`).
//!
//! ## API Quick Reference
//!
//! ### Node
//! ```rust,ignore
//! impl Node for MyNode {
//!     fn name(&self) -> &str { "MyNode" }
//!     fn tick(&mut self) { /* called each cycle */ }
//! }
//! ```
//!
//! ### Topic (pub/sub IPC)
//! ```rust,ignore
//! let topic: Topic<CmdVel> = Topic::new("cmd_vel")?;    // returns Result
//! topic.send(CmdVel::new(1.0, 0.0));
//! if let Some(msg) = topic.recv() { /* ... */ }
//! ```
//!
//! ### Scheduler
//! ```rust,ignore
//! let mut scheduler = Scheduler::new().tick_hz(100.0);
//! scheduler.add(sensor).order(0).rate_hz(100.0).build()?;
//! scheduler.add(controller).order(1).budget_us(200).build()?;
//! scheduler.add(planner).order(5).compute().build()?;
//! scheduler.add(logger).order(10).async_io().rate_hz(1.0).build()?;
//! scheduler.run()?;
//! ```
//!
//! ### Custom Messages
//! ```rust,ignore
//! message! {
//!     MotorCommand { velocity: f32, torque: f32 }
//! }
//! // Ready for Topic<MotorCommand> — no extra traits needed
//! ```
//!
//! ### Services (request/response)
//! ```rust,ignore
//! service! {
//!     AddTwoInts {
//!         request  { a: i64, b: i64 }
//!         response { sum: i64 }
//!     }
//! }
//! ```
//!
//! ### Actions (long-running tasks)
//! ```rust,ignore
//! action! {
//!     Navigate {
//!         goal     { x: f64, y: f64 }
//!         feedback { distance_remaining: f64 }
//!         result   { success: bool }
//!     }
//! }
//! ```
//!
//! ### Real-Time Nodes
//! ```rust,ignore
//! impl Node for MotorCtrl {
//!     fn tick(&mut self) { /* motor control logic */ }
//!     fn tick_budget(&self) -> Option<Duration> { Some(Duration::from_micros(200)) }
//!     fn deadline(&self) -> Duration { Duration::from_millis(1) }
//!     fn deadline_miss_policy(&self) -> DeadlineMissPolicy { DeadlineMissPolicy::Skip }
//! }
//! scheduler.add(MotorCtrl::new()).order(0).build();
//! ```
//!
//! ### Key Message Types
//!
//! | Type | Description |
//! |------|-------------|
//! | `CmdVel` | 2D velocity (linear f32, angular f32) |
//! | `Twist` | 6-DOF velocity ([f64;3] linear + angular) |
//! | `Pose2D` | 2D position + orientation |
//! | `Imu` | Accelerometer + gyroscope + magnetometer |
//! | `LaserScan` | 2D LiDAR scan |
//! | `Image` | Pool-backed image (zero-copy) |
//! | `PointCloud` | Pool-backed 3D points (zero-copy) |
//! | `DepthImage` | Pool-backed depth map (zero-copy) |

// === Internal plumbing (hidden from docs, used by horus_py / macro-generated code / horus_manager) ===
#[doc(hidden)]
pub use horus_core;
#[doc(hidden)]
pub use horus_core::actions;
#[doc(hidden)]
pub use horus_core::bytemuck;
#[doc(hidden)]
pub use horus_core::communication;
#[doc(hidden)]
pub use horus_core::core;
#[doc(hidden)]
pub use horus_core::dlpack;
#[doc(hidden)]
pub use horus_core::error;
#[doc(hidden)]
pub use horus_core::hlog;
#[doc(hidden)]
pub use horus_core::memory;
#[doc(hidden)]
pub use horus_core::params;
#[doc(hidden)]
pub use horus_core::paste;
#[doc(hidden)]
pub use horus_core::scheduling;
#[doc(hidden)]
pub use horus_core::serde_json;
#[doc(hidden)]
pub use horus_core::serde_yaml;
#[doc(hidden)]
pub use horus_core::services;
#[doc(hidden)]
pub use horus_core::types;
#[doc(hidden)]
pub use horus_library as library;
#[doc(hidden)]
pub use serde;

/// The HORUS prelude — everything you need for building robotics applications.
///
/// ```rust
/// use horus::prelude::*;
/// ```
///
/// This is the **only import** you need. All user-facing types, traits,
/// macros, and message definitions are included.
pub mod prelude {
    // === Node ===
    pub use horus_core::core::{LogSummary, Node};

    // === Real-time node ===
    pub use horus_core::core::{DeadlineMissPolicy, RtStats};

    // === Rate / Stopwatch ===
    pub use horus_core::core::timer::{Rate, Stopwatch};

    // === Topic (IPC) ===
    pub use horus_core::communication::Topic;

    // === Scheduler ===
    pub use horus_core::scheduling::Scheduler;

    // === Execution configuration ===
    pub use horus_core::scheduling::{ExecutionClass, FailurePolicy, NodeTier};

    // === Runtime parameters ===
    pub use horus_core::params::RuntimeParams;

    // === Memory (domain types) ===
    pub use horus_core::memory::{DepthImage, Image, PointCloud};

    // === HFrame (coordinate transforms) ===
    #[allow(deprecated)]
    pub use horus_library::hframe::{
        timestamp_now, FrameBuilder, FrameInfo, HFrame, HFrameConfig, HFrameStats,
        StaticFrameBuilder, StaticFrameBuilderWithParent, Transform, TransformQuery,
        TransformQueryFrom,
    };

    // === Message types (all standard robotics messages) ===
    pub use horus_core::types::{
        Device, ImageEncoding, PointXYZ, PointXYZI, PointXYZRGB, TensorDtype,
    };
    pub use horus_library::messages::*;

    // === Actions ===
    pub use horus_core::actions::{
        Action, ActionClientBuilder, ActionClientNode, ActionError, ActionServerBuilder,
        ActionServerNode, CancelResponse, ClientGoalHandle, GoalId, GoalOutcome, GoalPriority,
        GoalResponse, GoalStatus, PreemptionPolicy, ServerGoalHandle, SyncActionClient,
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
        CommunicationError, ConfigError, Error, HorusContext, HorusError, MemoryError, NodeError,
        NotFoundError, ParseError, ResourceError, Result, RetryConfig, SerializationError,
        Severity, TimeoutError, TransformError, ValidationError, retry_transient,
    };

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
}
