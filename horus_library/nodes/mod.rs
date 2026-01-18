//! HORUS Library Nodes
//!
//! This module provides access to HORUS node infrastructure.
//!
//! # Node Organization
//!
//! ## Core Nodes (horus-nodes-core crate)
//! 8 hardware-independent nodes have been moved to `horus-nodes-core`:
//! - `EmergencyStopNode`, `SafetyMonitorNode` - Safety & monitoring
//! - `DifferentialDriveNode`, `OdometryNode`, `PidControllerNode` - Motion control
//! - `PathPlannerNode`, `LocalizationNode`, `CollisionDetectorNode` - Navigation
//!
//! Import them directly: `use horus_nodes_core::*;`
//!
//! ## Optional Nodes (Feature-Gated)
//! Additional nodes require hardware features to be enabled:
//! - Sensors: Camera, Lidar, IMU, GPS, etc. (use `horus-sensors` crate)
//! - Actuators: DC/BLDC/Stepper motors, servos (use `horus-actuators` crate)
//! - Industrial: CAN, Modbus, I2C, SPI, Serial (use `horus-industrial` crate)
//! - Machine Learning: ONNX, TFLite, LLM (use `horus-nodes-ml` crate)
//! - Computer Vision: YOLO, Pose, Segmentation (use `horus-nodes-cv` crate)
//! - Dynamixel servos: (use `horus-dynamixel` crate)
//! - Roboclaw motors: (use `horus-roboclaw` crate)
//!
//! # Usage Examples
//!
//! ```rust,ignore
//! // Core nodes - from horus-nodes-core crate
//! use horus_nodes_core::{
//!     DifferentialDriveNode, PidControllerNode,
//!     EmergencyStopNode, SafetyMonitorNode,
//! };
//!
//! let drive = DifferentialDriveNode::new()?;
//! let pid = PidControllerNode::new()?;
//! let emergency = EmergencyStopNode::new()?;
//! ```

// Re-export core HORUS types for convenience
pub use horus_core::{Node, NodeInfo, Topic};

// Processor trait for custom node processing pipelines
// Re-export from horus-nodes-core once available, or define locally
// For now, users can import from horus_nodes_core::processor

// Processor trait for hybrid node pattern - now in horus_nodes_core
// Import from: use horus_nodes_core::{Processor, PassThrough, Pipeline, ClosureProcessor, FilterProcessor};
