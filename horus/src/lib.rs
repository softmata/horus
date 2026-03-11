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
//!     cmd_pub: Topic<CmdVel>,
//! }
//!
//! impl MyNode {
//!     fn new() -> Self {
//!         Self { cmd_pub: Topic::new("cmd_vel").unwrap() }
//!     }
//! }
//!
//! impl Node for MyNode {
//!     fn name(&self) -> &str { "my_node" }
//!
//!     fn tick(&mut self) {
//!         self.cmd_pub.send(CmdVel::new(0.5, 0.0));
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
//! let mut scheduler = Scheduler::new().tick_rate(100.hz());
//! scheduler.add(sensor).order(0).rate(100.hz()).build()?;
//! scheduler.add(controller).order(1).budget(200.us()).build()?;
//! scheduler.add(planner).order(5).compute().build()?;
//! scheduler.add(logger).order(10).async_io().rate(1.hz()).build()?;
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
//!     fn name(&self) -> &str { "MotorCtrl" }
//!     fn tick(&mut self) { /* motor control logic */ }
//! }
//! scheduler.add(MotorCtrl::new())
//!     .order(0)
//!     .budget(200.us())        // 200μs tick budget (auto-marks as RT)
//!     .deadline(1.ms())        // 1ms deadline
//!     .on_miss(Miss::Skip)     // skip tick on deadline miss
//!     .build()?;
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
    pub use horus_core::core::{HealthStatus, LogSummary, Node, NodeState};

    // === Real-time node ===
    pub use horus_core::core::{DurationExt, Frequency, Miss, RtStats};

    // === Rate / Stopwatch ===
    pub use horus_core::core::timer::{Rate, Stopwatch};

    // === Topic (IPC) ===
    pub use horus_core::communication::Topic;

    // === Scheduler ===
    pub use horus_core::scheduling::Scheduler;

    // === Execution configuration ===
    pub use horus_core::scheduling::{ExecutionClass, FailurePolicy};

    // === Runtime parameters ===
    pub use horus_core::params::RuntimeParams;

    // === Memory (domain types) ===
    pub use horus_core::memory::{DepthImage, Image, PointCloud};

    // === Transform Frame (coordinate transforms) ===
    pub use horus_library::transform_frame::{
        timestamp_now, FrameBuilder, FrameInfo, TransformFrame, TransformFrameConfig,
        TransformFrameStats, Transform, TransformQuery, TransformQueryFrom,
    };

    // === Message types (all standard robotics messages) ===
    // Includes: CmdVel, Imu, Odometry, LaserScan, Image, GenericMessage, and 60+ more.
    pub use horus_core::types::{
        Device, ImageEncoding, PointXYZ, PointXYZI, PointXYZRGB, TensorDtype,
    };
    pub use horus_library::messages::*;

    // === Actions ===
    pub use horus_core::actions::{
        Action, ActionClient, ActionClientBuilder, ActionClientNode, ActionError, ActionResult,
        ActionServerBuilder, ActionServerNode, CancelResponse, ClientGoalHandle, GoalId,
        GoalOutcome, GoalPriority, GoalResponse, GoalStatus, PreemptionPolicy, ServerGoalHandle,
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
        retry_transient, CommunicationError, ConfigError, Error, HorusContext, HorusError,
        MemoryError, NodeError, NotFoundError, ParseError, ResourceError, Result, RetryConfig,
        SerializationError, Severity, TimeoutError, TransformError, ValidationError,
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
