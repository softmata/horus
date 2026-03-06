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
//!     fn name(&self) -> &'static str { "MyNode" }
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
//! // Create a 480x640 RGB image (backed by shared memory)
//! let mut img = Image::new(480, 640, ImageEncoding::Rgb8).unwrap();
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

    // === Rate / Stopwatch ===
    pub use horus_core::core::timer::{Rate, Stopwatch};

    // === Topic (IPC) ===
    pub use horus_core::communication::Topic;

    // === Scheduler ===
    pub use horus_core::scheduling::Scheduler;

    // === Memory (domain types) ===
    pub use horus_core::memory::{DepthImage, Image, PointCloud};

    // === HFrame (coordinate transforms) ===
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
    pub use horus_core::error::{Error, HorusError, Result};

    // === Macros ===
    pub use horus_core::hlog;
    pub use horus_core::hlog_every;
    pub use horus_core::hlog_once;
    pub use horus_core::service;
    #[cfg(feature = "macros")]
    pub use horus_macros::*;
}
