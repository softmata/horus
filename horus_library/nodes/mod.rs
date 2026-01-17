//! HORUS Library Nodes
//!
//! This module contains pre-built, high-level nodes for common robotics applications.
//! All nodes follow the same simple API pattern: `NodeName::new()` for default configuration
//! or `NodeName::new_with_topic()` for custom topic names.
//!
//! # Node Organization
//!
//! ## Core Nodes (Always Available)
//! 8 hardware-independent nodes in `core/`:
//! - `EmergencyStopNode`, `SafetyMonitorNode` - Safety & monitoring
//! - `DifferentialDriveNode`, `OdometryNode`, `PidControllerNode` - Motion control
//! - `PathPlannerNode`, `LocalizationNode`, `CollisionDetectorNode` - Navigation
//!
//! ## Optional Nodes (Feature-Gated)
//! Additional nodes require hardware features to be enabled:
//! - Sensors: Camera, Lidar, IMU, GPS, etc. (requires `horus-sensors`)
//! - Actuators: DC/BLDC/Stepper motors, servos (requires `horus-actuators`)
//! - Industrial: CAN, Modbus, I2C, SPI, Serial (requires `horus-industrial`)
//!
//! # Usage Examples
//!
//! ```rust,ignore
//! use horus_library::nodes::*;
//!
//! // Core nodes - always available
//! let drive = DifferentialDriveNode::new();          // Subscribes to "cmd_vel"
//! let pid = PidControllerNode::new();                // Generic PID control
//! let emergency = EmergencyStopNode::new();          // Emergency stop handler
//! let safety = SafetyMonitorNode::new();             // Safety monitoring
//!
//! // Optional nodes - require features
//! #[cfg(feature = "opencv-backend")]
//! let camera = CameraNode::new();
//! ```

// =============================================================================
// CORE NODES - Hardware Independent (8 nodes)
// =============================================================================
// These are fundamental to every robotics application and have no hardware deps.

pub mod core;

// Re-export core nodes at top level for convenience
pub use core::{
    CollisionDetectorNode, DifferentialDriveNode, EmergencyStopNode, LocalizationNode,
    OdometryNode, PathPlannerNode, PidControllerNode, SafetyMonitorNode,
};

// Re-export processor types from core
pub use core::{ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor, ProcessorExt};

// Re-export processor module for backwards compatibility with internal imports
pub use core::processor;

// =============================================================================
// OPTIONAL NODES - Feature-Gated Hardware
// =============================================================================
// These nodes require specific hardware features to be enabled.

// Vision nodes (require camera backends)
#[cfg(any(
    feature = "opencv-backend",
    feature = "v4l2-backend",
    feature = "realsense",
    feature = "zed"
))]
pub mod camera;

#[cfg(feature = "realsense")]
pub mod depth_camera;

#[cfg(feature = "opencv-backend")]
pub mod image_processor;

// Input device nodes
#[cfg(feature = "gilrs")]
pub mod joystick;

#[cfg(feature = "crossterm")]
pub mod keyboard_input;

// Sensor nodes
#[cfg(feature = "i2c-hardware")]
pub mod battery_monitor;

#[cfg(any(feature = "bno055-imu", feature = "mpu6050-imu"))]
pub mod imu;

#[cfg(feature = "nmea-gps")]
pub mod gps;

#[cfg(feature = "rplidar")]
pub mod lidar;

#[cfg(feature = "netft")]
pub mod force_torque_sensor;

#[cfg(feature = "gpio-hardware")]
pub mod encoder;

#[cfg(feature = "gpio-hardware")]
pub mod ultrasonic;

// Motor/Actuator nodes
#[cfg(feature = "gpio-hardware")]
pub mod bldc_motor;

#[cfg(feature = "gpio-hardware")]
pub mod dc_motor;

#[cfg(feature = "gpio-hardware")]
pub mod stepper_motor;

#[cfg(feature = "gpio-hardware")]
pub mod servo_controller;

#[cfg(feature = "serial-hardware")]
pub mod dynamixel;

#[cfg(feature = "serial-hardware")]
pub mod roboclaw_motor;

// Industrial interface nodes
#[cfg(feature = "can-hardware")]
pub mod can_bus;

#[cfg(feature = "gpio-hardware")]
pub mod digital_io;

#[cfg(feature = "i2c-hardware")]
pub mod i2c_bus;

#[cfg(feature = "modbus-hardware")]
pub mod modbus;

#[cfg(feature = "serial-hardware")]
pub mod serial;

#[cfg(feature = "spi-hardware")]
pub mod spi_bus;

// Machine Learning & AI nodes
#[cfg(any(feature = "onnx", feature = "tflite-inference"))]
pub mod ml_inference;

#[cfg(feature = "onnx")]
pub mod cv;

pub mod llm;

// Vision nodes
#[cfg(any(
    feature = "opencv-backend",
    feature = "v4l2-backend",
    feature = "realsense",
    feature = "zed"
))]
pub use camera::CameraNode;

#[cfg(feature = "realsense")]
pub use depth_camera::DepthCameraNode;

#[cfg(feature = "opencv-backend")]
pub use image_processor::ImageProcessorNode;

// Input device nodes
#[cfg(feature = "gilrs")]
pub use joystick::JoystickInputNode;

#[cfg(feature = "crossterm")]
pub use keyboard_input::KeyboardInputNode;

// Sensor nodes
#[cfg(feature = "i2c-hardware")]
pub use battery_monitor::BatteryMonitorNode;

#[cfg(any(feature = "bno055-imu", feature = "mpu6050-imu"))]
pub use imu::{ImuBackend, ImuNode};

#[cfg(feature = "nmea-gps")]
pub use gps::{GpsBackend, GpsNode};

#[cfg(feature = "rplidar")]
pub use lidar::{LidarBackend, LidarNode};

#[cfg(feature = "netft")]
pub use force_torque_sensor::ForceTorqueSensorNode;

#[cfg(feature = "gpio-hardware")]
pub use encoder::EncoderNode;

#[cfg(feature = "gpio-hardware")]
pub use ultrasonic::UltrasonicNode;

// Motor/Actuator nodes
#[cfg(feature = "gpio-hardware")]
pub use bldc_motor::BldcMotorNode;

#[cfg(feature = "gpio-hardware")]
pub use dc_motor::DcMotorNode;

#[cfg(feature = "gpio-hardware")]
pub use stepper_motor::StepperMotorNode;

#[cfg(feature = "gpio-hardware")]
pub use servo_controller::ServoControllerNode;

#[cfg(feature = "serial-hardware")]
pub use dynamixel::DynamixelNode;

#[cfg(feature = "serial-hardware")]
pub use roboclaw_motor::RoboclawMotorNode;

// Industrial interface nodes
#[cfg(feature = "can-hardware")]
pub use can_bus::CanBusNode;

#[cfg(feature = "gpio-hardware")]
pub use digital_io::DigitalIONode;

#[cfg(feature = "i2c-hardware")]
pub use i2c_bus::I2cBusNode;

#[cfg(feature = "modbus-hardware")]
pub use modbus::ModbusNode;

#[cfg(feature = "serial-hardware")]
pub use serial::SerialNode;

#[cfg(feature = "spi-hardware")]
pub use spi_bus::SpiBusNode;

// ML/AI nodes
#[cfg(feature = "onnx")]
pub use ml_inference::{InferenceConfig, ONNXInferenceNode};

#[cfg(feature = "tflite-inference")]
pub use ml_inference::{TFLiteConfig, TFLiteInferenceNode};

#[cfg(feature = "onnx")]
pub use cv::{
    PoseConfig, PoseEstimationNode, PoseModelType, SegmentationConfig, SemanticSegmentationNode,
    YOLOConfig, YOLOv8DetectorNode,
};

// LLM nodes
#[cfg(feature = "ml-inference")]
pub use llm::{CloudLLMNode, LLMConfig, LLMProvider};

// Re-export core HORUS types for convenience
pub use horus_core::{Node, NodeInfo, Topic};
