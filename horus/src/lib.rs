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
//! // Any Serialize + Deserialize type is a valid Topic message, and the
//! // ready-made robotics messages (CmdVel, Imu, …) are already in scope:
//! // `horus::prelude` re-exports `horus_robotics::prelude::*`.
//! pub struct MyNode {
//!     cmd_pub: Topic<Twist>,
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
//!         // Twist is the prelude-native velocity command (linear, angular).
//!         self.cmd_pub.send(Twist::new_2d(0.5, 0.0));
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
//! // Send — only the 224-byte descriptor goes through the ring buffer
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
//! let topic: Topic<Twist> = Topic::new("cmd_vel")?;    // returns Result
//! topic.send(Twist::new_2d(1.0, 0.0));
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
//! The builder returned by `Scheduler::add` carries the full set; `.build()` is
//! what registers the node once you have chosen one.
//!
//! ### Key Message Types
//!
//! | Type | Description |
//! |------|-------------|
//! | `CmdVel` | 2D velocity (linear f32, angular f32) |
//! | `Twist` | 6-DOF velocity (`[f64; 3]` linear + angular) |
//! | `Pose2D` | 2D position + orientation |
//! | `Imu` | Accelerometer + gyroscope + magnetometer |
//! | `LaserScan` | 2D LiDAR scan |
//! | `Image` | Pool-backed image (zero-copy) |
//! | `PointCloud` | Pool-backed 3D points (zero-copy) |
//! | `DepthImage` | Pool-backed depth map (zero-copy) |
//!
//! `Twist`, `Pose2D`, `Image`, `PointCloud`, and `DepthImage` are in
//! `horus::prelude`. Robotics-specific messages (`CmdVel`, `Imu`, `LaserScan`,
//! …) live in the `horus_robotics` crate but are re-exported through
//! `horus::prelude` too, so `use horus::prelude::*;` is all you need.
//!
//! When you want to name a message rather than glob one in, every one of them
//! is also at [`msg`] — `horus::msg::CmdVel`, the same path C++ writes. Which
//! crate a message is filed under is a packaging detail; it should not decide
//! what your import lines look like.
//!
//! Do not write `use horus_robotics::prelude::*;` in a project created by
//! `horus new`: the generated manifest depends on `horus` only, so that path
//! fails with `E0433: use of unresolved module or unlinked crate`.
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
//! on every backend, not just the multi-consumer ones: it hands back a copy of bytes
//! another process may overwrite at any moment, so a `T` owning a heap pointer would
//! be a use-after-free. Use `recv()` for non-Copy types like `String` or `Vec`.
//!
//! **5. `.rate()` order doesn't matter** — `.rate(100.hz()).compute()` produces Compute,
//! not Rt. Execution class is determined by the explicit class call, not by `.rate()`.
//! `.rate()` only auto-derives Rt when no explicit class is set.

/// Framework time API — `horus::time::now()`, `horus::time::dt()`,
/// `horus::time::rng()`, and friends.
///
/// These are module functions: call them through `horus::time::`, or bring them
/// in with `use horus::time::{now, dt};`. There is no `horus::now()`,
/// `horus::dt()` or `horus::rng()` — the free functions are not re-exported at
/// the crate root, and the prelude re-exports only [`TimeStamp`](time::TimeStamp).
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
#[ctor::ctor(unsafe)]
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
pub mod net;

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
/// Duration and frequency helpers — `100_u64.hz()`, `5_u64.ms()`, and friends.
///
/// These were reachable only through the prelude, so `use horus::DurationExt;`
/// failed with "unresolved import". Three of the ten shipped examples wrote
/// exactly that and did not compile.
pub use horus_core::core::{DurationExt, Frequency};

/// Error types — `HorusError`, `HorusResult`, and the per-domain sub-errors.
///
/// The prelude exports the short aliases (`Result`, `Error`), which covers most
/// code. This module is here so the long names are reachable without naming an
/// internal crate: the documentation previously said
/// `use horus::horus_core::error::HorusResult;`.
pub use horus_core::error;
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

/// Long-form `Result` alias, for code that prefers it over the prelude's
/// `Result`. Was only reachable as `horus::horus_core::HorusResult`.
pub use horus_core::error::HorusResult;

/// Compile-time topic descriptors declared with the `topics!` macro.
///
/// Was only reachable as `horus::horus_core::topics`.
pub use horus_core::topics;

#[doc(hidden)]
pub use horus_core::horus_internal;

#[doc(hidden)]
pub use horus_core::serde_yaml;

pub mod types;

pub mod msg;

pub mod prelude;

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    /// Naming a type is enough to prove the path resolves; nothing is
    /// constructed, so this stays honest for types without a `Default`.
    fn reachable<T>() {}

    /// Every row of the cross-language compatibility table has to resolve under
    /// the single `horus::msg::` path. Before this module existed the Rust
    /// column sent readers to `horus_robotics::CmdVel`,
    /// `horus_robotics::messages::sensor::Imu` and `horus_types::Pose2D` — three
    /// crates at three depths for one table.
    #[test]
    fn msg_covers_the_cross_language_table() {
        reachable::<crate::msg::CmdVel>();
        reachable::<crate::msg::Imu>();
        reachable::<crate::msg::Pose2D>();
        reachable::<crate::msg::Odometry>();
        reachable::<crate::msg::LaserScan>();
        reachable::<crate::msg::Twist>();
        reachable::<crate::msg::NavGoal>();
    }

    /// The other 60 names C++ declares across `<horus/msg/*.hpp>`. This list is
    /// the C++ headers' own contents, so it fails the moment the two languages
    /// drift. A partial re-export would just recreate the `horus::types` trap —
    /// a path that exists and resolves for half the types a reader tries.
    #[test]
    fn msg_covers_the_whole_cpp_namespace() {
        reachable::<crate::msg::Accel>();
        reachable::<crate::msg::AccelStamped>();
        reachable::<crate::msg::AudioEncoding>();
        reachable::<crate::msg::AudioFrame>();
        reachable::<crate::msg::BatteryState>();
        reachable::<crate::msg::BoundingBox2D>();
        reachable::<crate::msg::BoundingBox3D>();
        reachable::<crate::msg::Clock>();
        reachable::<crate::msg::ContactInfo>();
        reachable::<crate::msg::Detection>();
        reachable::<crate::msg::Detection3D>();
        reachable::<crate::msg::DiagnosticReport>();
        reachable::<crate::msg::DiagnosticStatus>();
        reachable::<crate::msg::DiagnosticValue>();
        reachable::<crate::msg::DifferentialDriveCommand>();
        reachable::<crate::msg::EmergencyStop>();
        reachable::<crate::msg::FluidPressure>();
        reachable::<crate::msg::ForceCommand>();
        reachable::<crate::msg::GoalResult>();
        reachable::<crate::msg::HapticFeedback>();
        reachable::<crate::msg::Heartbeat>();
        reachable::<crate::msg::Illuminance>();
        reachable::<crate::msg::ImpedanceParameters>();
        reachable::<crate::msg::JointCommand>();
        reachable::<crate::msg::JointState>();
        reachable::<crate::msg::JoystickInput>();
        reachable::<crate::msg::KeyboardInput>();
        reachable::<crate::msg::Landmark>();
        reachable::<crate::msg::Landmark3D>();
        reachable::<crate::msg::LandmarkArray>();
        reachable::<crate::msg::MagneticField>();
        reachable::<crate::msg::MotorCommand>();
        reachable::<crate::msg::NavPath>();
        reachable::<crate::msg::NavSatFix>();
        reachable::<crate::msg::NodeHeartbeat>();
        reachable::<crate::msg::PathPlan>();
        reachable::<crate::msg::PidConfig>();
        reachable::<crate::msg::Point3>();
        reachable::<crate::msg::Pose3D>();
        reachable::<crate::msg::PoseStamped>();
        reachable::<crate::msg::PoseWithCovariance>();
        reachable::<crate::msg::Quaternion>();
        reachable::<crate::msg::RangeSensor>();
        reachable::<crate::msg::RateRequest>();
        reachable::<crate::msg::ResourceUsage>();
        reachable::<crate::msg::SafetyStatus>();
        reachable::<crate::msg::SegmentationMask>();
        reachable::<crate::msg::ServoCommand>();
        reachable::<crate::msg::SimSync>();
        reachable::<crate::msg::Temperature>();
        reachable::<crate::msg::TimeReference>();
        reachable::<crate::msg::TrackedObject>();
        reachable::<crate::msg::TrackingHeader>();
        reachable::<crate::msg::TrajectoryPoint>();
        reachable::<crate::msg::TransformStamped>();
        reachable::<crate::msg::TwistWithCovariance>();
        reachable::<crate::msg::Vector3>();
        reachable::<crate::msg::VelocityObstacle>();
        reachable::<crate::msg::Waypoint>();
        reachable::<crate::msg::WrenchStamped>();
    }

    /// The zero-copy payloads and the transform POD. C++ files these in plain
    /// `horus::` rather than `horus::msg::`, which is why the first cut of this
    /// module left them out — and that rebuilt the `horus::types` trap inside
    /// the module written to close it: `horus::msg::Image` was an `E0425` two
    /// lines below a working `horus::msg::LaserScan`. `Image`, `PointCloud` and
    /// `DepthImage` are three of the eight rows in this crate's own "Key
    /// Message Types" table, and the crate docs claim in so many words that
    /// "every one of them is also at [`msg`]" — which was not true for those
    /// three until this re-export existed.
    #[test]
    fn msg_covers_the_pool_backed_payloads() {
        reachable::<crate::msg::Image>();
        reachable::<crate::msg::PointCloud>();
        reachable::<crate::msg::DepthImage>();
        reachable::<crate::msg::Transform>();
        reachable::<crate::msg::ImageEncoding>();
        reachable::<crate::msg::PointXYZ>();
        reachable::<crate::msg::PointXYZI>();
        reachable::<crate::msg::PointXYZRGB>();
    }

    /// `horus::types` is the path a reader guesses before they have heard of
    /// `horus::msg` — the module is literally called "types" and its heading
    /// says "Message and geometry types". It resolved for the geometry half
    /// only, which reads as "right module, missing type" rather than "wrong
    /// module". It now mirrors `msg`.
    #[test]
    fn types_is_no_longer_the_geometry_half() {
        reachable::<crate::types::CmdVel>();
        reachable::<crate::types::Imu>();
        reachable::<crate::types::Odometry>();
        reachable::<crate::types::LaserScan>();
        reachable::<crate::types::NavGoal>();
        reachable::<crate::types::Image>();
        reachable::<crate::types::Transform>();

        // and it did not lose what it already had
        reachable::<crate::types::Tensor>();
        reachable::<crate::types::Pose2D>();
    }

    /// Globbing the prelude and `msg` together is the obvious thing to write,
    /// and `GoalStatus` — the one name `horus_core::actions` and
    /// `horus_robotics::messages::navigation` both define — made it a hard
    /// `E0659` "ambiguous name" error, not a warning. This test is that exact
    /// pair of globs; it cannot compile unless the two agree.
    #[test]
    fn msg_and_prelude_globs_compose() {
        use crate::msg::*;
        use crate::prelude::*;

        let lifecycle: GoalStatus = GoalStatus::Succeeded;
        assert!(lifecycle.is_terminal());

        // The navigation status code is not gone, just where the navigation
        // reference already said it was: under its module.
        assert_eq!(navigation::GoalStatus::Succeeded as u8, 2);

        // A message name from each source crate, and a prelude-only name, so
        // this really is the two-glob situation and not a `msg` glob sitting
        // next to a dead import.
        let _: CmdVel = CmdVel::default();
        let _: Pose2D = Pose2D::default();
        let _: Option<HorusError> = None;
    }

    /// The bare name means the same thing under every path the facade offers,
    /// so moving an import line from the prelude to `msg` cannot change which
    /// enum a match arm is matching on.
    #[test]
    fn goal_status_is_one_enum_under_every_path() {
        fn same<T>(_: &T, _: &T) {}

        let from_msg = crate::msg::GoalStatus::Succeeded;
        let from_prelude = crate::prelude::GoalStatus::Succeeded;
        let from_types = crate::types::GoalStatus::Succeeded;
        same(&from_msg, &from_prelude);
        same(&from_msg, &from_types);

        // The navigation `#[repr(u8)]` code is a different enum and stays
        // reachable through its module, unchanged.
        assert_eq!(crate::msg::navigation::GoalStatus::Succeeded as u8, 2);
        assert_eq!(horus_robotics::GoalStatus::Succeeded as u8, 2);
    }

    /// The pre-existing paths are re-exports, not moves: whatever a user wrote
    /// before still names the same type. `horus::msg` is additive.
    #[test]
    fn msg_is_the_same_type_as_the_old_paths() {
        fn same<T>(_: &T, _: &T) {}

        let a = crate::msg::CmdVel::default();
        let b = horus_robotics::CmdVel::default();
        same(&a, &b);

        let c = crate::msg::Pose2D::default();
        let d = horus_types::Pose2D::default();
        same(&c, &d);

        let e = crate::msg::Imu::default();
        let f = horus_robotics::messages::sensor::Imu::default();
        same(&e, &f);
    }
}
