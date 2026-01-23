//! # HORUS Standard Library
//!
//! The official standard library for the HORUS robotics framework.
//!
//! ## Structure
//!
//! ```text
//! horus_library/
//! ├── messages/       # Shared memory-safe message types
//! ├── hframe/         # HFrame - High-performance transform system
//! ├── nodes/          # Node infrastructure (traits, re-exports)
//! └── drivers/        # Driver infrastructure (traits, re-exports)
//! ```
//!
//! ## What's Included
//!
//! ### Message Types
//! Standard robotics message types for node communication:
//! - **Geometry**: `Pose2D`, `Pose3D`, `Transform`, `Twist`, `Vector3`, `Quaternion`
//! - **Sensors**: `ImuData`, `LaserScan`, `PointCloud`, `BatteryState`, `Temperature`
//! - **Navigation**: `Odometry`, `Path`, `OccupancyGrid`, `Goal`
//! - **Vision**: `Image`, `CameraInfo`, `CompressedImage`
//! - **Control**: `MotorCommand`, `JointState`, `ServoCommand`
//! - **I/O**: `DigitalIO`, `AnalogIO`, `CanFrame`, `ModbusData`, `I2CData`
//!
//! ### HFrame Transform System
//! High-performance coordinate transform system:
//! - `HFrame`, `Transform`, `TransformStamped`
//! - Lock-free concurrent access
//! - Efficient transform tree lookups
//!
//! ## Building Custom Nodes
//!
//! Users implement their own nodes using the HORUS core infrastructure.
//! Node traits and types are provided by `horus_core`:
//!
//! ```rust,ignore
//! use horus_core::{Node, NodeInfo, Topic};
//! use horus_library::messages::{Twist, Odometry};
//!
//! pub struct MyRobotNode {
//!     cmd_vel: Topic<Twist>,
//!     odom: Topic<Odometry>,
//! }
//!
//! impl Node for MyRobotNode {
//!     fn name(&self) -> &'static str { "my_robot" }
//!
//!     fn tick(&mut self) {
//!         // Node logic here
//!     }
//! }
//! ```
//!
//! ## Usage
//!
//! ```rust,ignore
//! // Message types are re-exported at the crate root
//! use horus_library::{
//!     LogSummary,
//!     KeyboardInput, CmdVel, LaserScan, Image, Twist,
//! };
//!
//! // Or import from specific modules
//! use horus_library::messages::{Direction, SnakeState};
//!
//! // Use HFrame for coordinate transforms
//! use horus_library::hframe::{HFrame, Transform};
//!
//! // Use algorithms
//! use horus_library::algorithms::{pid::PID, ekf::EKF};
//!
//! // Use simulators (separate crates)
//! use sim2d::{Sim2DBuilder, RobotConfig};
//! use sim3d::rl::{RLTask, Action, Observation};
//! ```

pub mod drivers;
pub mod hframe;
pub mod messages;
pub mod nodes;

// Note: sim2d and sim3d are separate crates to avoid cyclic dependencies.
// Access them directly via:
//   - Rust: use sim2d::*; or use sim3d::*;
//   - Python: from horus.library.sim2d import Sim2D
//             from horus.library.sim3d import make_env

// Re-export core traits needed for message types
pub use horus_core::core::LogSummary;

// Re-export message types at the crate root for convenience
pub use messages::*;

// Re-export core traits needed for message types
// Note: Users implement their own drivers using traits from horus_core::driver

/// Prelude module for convenient imports
///
/// # Usage
/// ```rust,ignore
/// use horus_library::prelude::*;
///
/// // Build custom nodes using core infrastructure
/// use horus_core::{Node, NodeInfo, Topic};
///
/// // For simulation, import sim2d/sim3d directly:
/// use sim2d::{Sim2DBuilder, RobotConfig};
/// use sim3d::rl::{RLTask, make_env};
/// ```
pub mod prelude {
    // Core traits
    pub use crate::LogSummary;

    // Common message types
    pub use crate::messages::{
        cmd_vel::CmdVel,
        geometry::{Point3, Pose2D, Quaternion, Twist, Vector3},
        sensor::{BatteryState, Imu, LaserScan, NavSatFix, Odometry},
    };

    // HFrame - High-performance transform system
    pub use crate::hframe::{
        timestamp_now, FrameId, FrameRegistry, FrameSlot, HFrame, HFrameConfig, HFrameCore,
        HFrameError, StaticTransformStamped, TFMessage, Transform, TransformStamped,
    };

    // Note: sim2d and sim3d are separate crates to avoid cyclic dependencies.
    // Import them directly: use sim2d::*; or use sim3d::*;
}
