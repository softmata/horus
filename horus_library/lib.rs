//! # HORUS Standard Library
//!
//! The official standard library for the HORUS robotics framework.
//!
//! ## Structure
//!
//! ```text
//! horus_library/
//! ── messages/       # Shared memory-safe messages
//! ── nodes/          # Node infrastructure
//! ── algorithms/     # Common algorithms (via horus-algorithms)
//! ── hframe/         # HFrame - High-performance transform system
//! ```
//!
//! ## Core Nodes
//!
//! Core hardware-independent nodes have been moved to `horus-nodes-core`:
//! - EmergencyStopNode, SafetyMonitorNode (Safety & monitoring)
//! - DifferentialDriveNode, OdometryNode, PidControllerNode (Motion control)
//! - PathPlannerNode, LocalizationNode, CollisionDetectorNode (Navigation)
//!
//! Use `horus-nodes-core` for these nodes.
//!
//! ## Sensor Nodes
//!
//! Sensor nodes have been moved to the `horus-sensors` crate for modularity.
//! Use `horus-sensors` for: CameraNode, ImuNode, LidarNode, GpsNode, etc.
//!
//! ## Actuator Nodes
//!
//! Basic actuator nodes have been moved to the `horus-actuators` crate.
//! Use `horus-actuators` for: DcMotorNode, BldcMotorNode, StepperMotorNode, ServoControllerNode.
//!
//! ## Industrial Nodes
//!
//! Industrial communication nodes have been moved to the `horus-industrial` crate.
//! Use `horus-industrial` for: CanBusNode, I2cBusNode, SpiBusNode, SerialNode, ModbusNode, DigitalIONode.
//!
//! ## Usage
//!
//! ```rust,ignore
//! // Message types and traits are re-exported at the root for convenience
//! use horus_library::{
//!     // Core traits
//!     LogSummary,
//!     // Messages
//!     KeyboardInput, JoystickInput, CmdVel, LaserScan, Image, Twist,
//! };
//!
//! // For core nodes, import from horus-nodes-core:
//! use horus_nodes_core::{DifferentialDriveNode, EmergencyStopNode};
//!
//! // For sensors, import from horus-sensors:
//! use horus_sensors::{CameraNode, LidarNode, ImuNode};
//!
//! // Create and configure nodes with simple constructors
//! let drive = DifferentialDriveNode::new()?;   // Subscribes to "cmd_vel"
//! let emergency = EmergencyStopNode::new()?;   // Emergency stop handler
//!
//! // Or import from specific modules
//! use horus_library::messages::{Direction, SnakeState};
//!
//! // Use HFrame for coordinate transforms
//! use horus_library::hframe::{HFrame, Transform};
//!
//! // Use simulators (separate crates to avoid cyclic deps)
//! use sim2d::{Sim2DBuilder, RobotConfig};
//! use sim3d::rl::{RLTask, Action, Observation};
//! ```

// Re-export algorithms from horus-algorithms crate
pub use horus_algorithms as algorithms;
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

// Re-export driver types
// Note: Motor, servo, bldc, stepper drivers moved to horus-actuators crate
// Note: Bus, serial, modbus, digital_io drivers moved to horus-industrial crate
// Note: Core nodes moved to horus-nodes-core crate

/// Prelude module for convenient imports
///
/// # Usage
/// ```rust,ignore
/// use horus_library::prelude::*;
///
/// // For core nodes, import from horus-nodes-core:
/// use horus_nodes_core::{DifferentialDriveNode, EmergencyStopNode};
///
/// // For simulation, import sim2d/sim3d directly:
/// use sim2d::{Sim2DBuilder, RobotConfig};
/// use sim3d::rl::{RLTask, make_env};  // separate crate
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

    // Note: Core nodes have been moved to horus-nodes-core crate.
    // Import them directly: use horus_nodes_core::*;

    // Note: sim2d and sim3d are separate crates to avoid cyclic dependencies.
    // Import them directly: use sim2d::*; or use sim3d::*;
}
