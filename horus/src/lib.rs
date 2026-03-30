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
//! let topic: Topic<Image> = Topic::new("camera.rgb").unwrap();
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
//! let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
//! scheduler.add(sensor).order(0).rate(100_u64.hz()).build()?;
//! scheduler.add(controller).order(1).rate(200_u64.hz()).build()?;
//! scheduler.add(planner).order(5).compute().build()?;
//! scheduler.add(logger).order(10).async_io().rate(1_u64.hz()).build()?;
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
//!     .rate(1000_u64.hz())         // 1kHz → auto-derives budget (80%) & deadline (95%)
//!     .on_miss(Miss::Skip)         // skip tick on deadline miss
//!     .build()?;
//! ```
//!
//! ### Execution Classes Quick Reference
//!
//! | Method | Class | When to Use |
//! |--------|-------|-------------|
//! | `.rate(100.hz())` | **Rt** | Motor control, sensor sampling (auto-derives budget/deadline) |
//! | `.compute()` | **Compute** | Path planning, CV, ML inference (parallel thread pool) |
//! | `.on("topic")` | **Event** | Data processors, filters (ticks only on new data) |
//! | `.async_io()` | **AsyncIo** | Network calls, file I/O, logging (tokio pool, no RT impact) |
//! | *(default)* | **BestEffort** | Diagnostics, telemetry (main thread, sequential) |
//!
//! See [`scheduling::node_builder`] for the full decision guide.
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
//!
//! ## Common Mistakes
//!
//! **1. Execution class override** — Calling `.compute()` then `.on("topic")` on the same
//! node: only the LAST execution class applies. The first is silently overridden with a
//! log warning. Pick ONE: `.compute()`, `.on()`, `.async_io()`, or `.rate()`.
//!
//! **2. Budget/deadline units** — `.budget()` and `.deadline()` take `Duration`, not
//! microseconds. Use `200_u64.us()` not `200`. If using `.rate()`, budget (80%) and
//! deadline (95%) are auto-derived — you usually don't need to set them manually.
//!
//! **3. Forgetting `.build()`** — `scheduler.add(node).order(0).rate(100.hz())` without
//! `.build()?` at the end silently drops the node registration. Always end with `.build()?`.
//!
//! **4. `read_latest()` on non-Copy types** — `Topic::read_latest()` requires `T: Copy`
//! for multi-consumer backends. Use `recv()` for non-Copy types like `String` or `Vec`.
//!
//! **5. `.rate()` order doesn't matter** — `.rate(100.hz()).compute()` produces Compute,
//! not Rt. Execution class is determined by the explicit class call, not by `.rate()`.
//! `.rate()` only auto-derives Rt when no explicit class is set.

/// Framework time API — `horus::now()`, `horus::dt()`, `horus::rng()`, etc.
///
/// See [`time`] module documentation for details.
pub mod time;

// === Networking (transparent LAN replication) ===
//
// Auto-wire: when the `net` feature is enabled, a ctor hook registers the
// network auto-start function into horus_core. The scheduler's run() method
// checks for this and starts horus_net automatically — users never need to
// call `horus::net::wire()` manually.
#[cfg(feature = "net")]
#[ctor::ctor]
fn __register_network_auto_wire() {
    horus_core::scheduling::set_network_auto_wire(|scheduler: &mut horus_core::Scheduler| {
        scheduler.on_start(|| match horus_net::start_replicator_default() {
            Some(handle) => {
                eprintln!("[horus_net] Network replication started");
                Some(Box::new(handle))
            }
            None => None,
        });
    });
}

#[cfg(feature = "net")]
pub mod net {
    //! Transparent LAN replication — zero config, same `Topic<T>` API.
    //!
    //! Network replication starts **automatically** when `scheduler.run()` is called.
    //! No manual wiring needed:
    //!
    //! ```rust,ignore
    //! use horus::prelude::*;
    //!
    //! let mut scheduler = Scheduler::new()
    //!     .tick_rate(100_u64.hz());
    //! // ... add nodes ...
    //! scheduler.run()?;
    //! // Network replication starts automatically and stops on shutdown
    //! ```
    //!
    //! ## Opting out
    //!
    //! Disable networking via `.network(false)` or `HORUS_NET_ENABLED=false` env var:
    //!
    //! ```rust,ignore
    //! let mut scheduler = Scheduler::new()
    //!     .network(false)              // <-- disables LAN replication
    //!     .tick_rate(100_u64.hz());
    //! ```
    //!
    //! ## Custom config
    //!
    //! For advanced networking configuration, use [`wire_with_config()`] to override
    //! the automatic default:
    //!
    //! ```rust,ignore
    //! use horus::net::NetConfig;
    //!
    //! let config = NetConfig { port: 9200, ..NetConfig::default() };
    //! horus::net::wire_with_config(&mut scheduler, config);
    //! scheduler.run()?;
    //! ```
    //!
    //! ## Manual control
    //!
    //! Call [`enable()`] before `scheduler.run()` and hold the returned handle:
    //!
    //! ```rust,ignore
    //! let _net = horus::net::enable();
    //! scheduler.run()?;
    //! ```

    /// Enable network replication with default settings.
    ///
    /// Returns a handle that stops the replicator on drop.
    /// Returns `None` if networking is disabled (`HORUS_NO_NETWORK=1`).
    pub fn enable() -> Option<horus_net::ReplicatorHandle> {
        horus_net::start_replicator_default()
    }

    /// Enable network replication with custom config.
    pub fn enable_with_config(
        config: horus_net::config::NetConfig,
    ) -> Option<horus_net::ReplicatorHandle> {
        horus_net::start_replicator(config)
    }

    /// Wire `horus_net` into the scheduler's lifecycle (manual override).
    ///
    /// **Note**: As of HORUS 0.2, networking is wired automatically when
    /// `scheduler.run()` is called — you do NOT need to call this function
    /// for default configuration. It remains available for backward
    /// compatibility and as an explicit opt-in if you want to be explicit.
    ///
    /// If networking was explicitly disabled (`.network(false)` or
    /// `HORUS_NET_ENABLED=false`), this is a no-op.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus::prelude::*;
    ///
    /// let mut scheduler = Scheduler::new()
    ///     .tick_rate(100_u64.hz());
    /// scheduler.add(my_node).build()?;
    ///
    /// // Optional: explicit wire (happens automatically if omitted)
    /// horus::net::wire(&mut scheduler);
    /// scheduler.run()?;
    /// ```
    pub fn wire(scheduler: &mut horus_core::Scheduler) {
        if !scheduler.network_enabled() {
            return;
        }

        scheduler.on_start(|| match horus_net::start_replicator_default() {
            Some(handle) => {
                eprintln!("[horus_net] Network replication started");
                Some(Box::new(handle))
            }
            None => None,
        });
    }

    /// Wire `horus_net` with custom configuration into the scheduler's lifecycle.
    ///
    /// Same as [`wire()`] but allows passing a custom [`NetConfig`].
    /// When called, the custom config takes precedence over the automatic
    /// default wiring.
    pub fn wire_with_config(
        scheduler: &mut horus_core::Scheduler,
        config: horus_net::config::NetConfig,
    ) {
        if !scheduler.network_enabled() {
            return;
        }

        scheduler.on_start(move || match horus_net::start_replicator(config) {
            Some(handle) => {
                eprintln!("[horus_net] Network replication started (custom config)");
                Some(Box::new(handle))
            }
            None => None,
        });
    }

    pub use horus_net::config::NetConfig;
}

// === Internal plumbing (hidden from docs, used by horus_py / macro-generated code / horus_manager) ===
#[doc(hidden)]
pub use horus_core;
#[doc(hidden)]
pub use horus_core::communication;
#[doc(hidden)]
pub use horus_core::core;
/// Hardware configuration and node loading.
///
/// Load hardware nodes from `horus.toml` `[hardware]` section:
///
/// ```rust,ignore
/// use horus::hardware;
///
/// let nodes = hardware::load()?;
/// for (name, node) in nodes {
///     sched.add(node).build()?;
/// }
/// ```
pub use horus_core::drivers as hardware;
// `horus::drivers` removed — use `horus::hardware` instead.
#[doc(hidden)]
pub use horus_core::hlog;
#[doc(hidden)]
pub use horus_core::memory;
/// Register a node factory for `[hardware]` config instantiation.
///
/// ```rust,ignore
/// register_driver!(MyDriver, MyDriver::from_params);
/// ```
pub use horus_core::register_driver;
#[doc(hidden)]
pub use horus_core::scheduling;
#[doc(hidden)]
pub use horus_core::serde_json;
#[doc(hidden)]
pub use horus_types as types;

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
    pub use horus_core::scheduling::FailurePolicy;

    // === Runtime parameters ===
    pub use horus_core::params::RuntimeParams;

    // === Memory (domain types) ===
    pub use horus_core::memory::{DepthImage, Image, PointCloud};

    // === Universal IPC types (math, diagnostics, time, generic) ===
    pub use horus_types::prelude::*;

    // === Types (core) ===
    /// Internal tensor dtype — prefer `PointCloud::from_xyz()`, `DepthImage::meters()` etc.
    #[doc(hidden)]
    pub use horus_core::types::TensorDtype;
    pub use horus_core::types::{Device, ImageEncoding, PointXYZ, PointXYZI, PointXYZRGB};

    // NOTE: TransformFrame and robotics messages are no longer in horus core.
    // Use: `use horus_tf::prelude::*` for transforms
    // Use: `use horus_robotics::prelude::*` for robotics messages (CmdVel, Imu, etc.)

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

    // === Time API ===
    pub use crate::time::TimeStamp;

    // === GPU ===
    pub use horus_core::gpu::{cuda_available, cuda_device_count, gpu_platform, GpuPlatform};

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
