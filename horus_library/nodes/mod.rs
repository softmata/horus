//! HORUS Library Nodes
//!
//! This module contains pre-built, high-level nodes for common robotics applications.
//! All nodes follow the same simple API pattern: `NodeName::new()` for default configuration
//! or `NodeName::new_with_topic()` for custom topic names.
//!
//! # MVP Node Categories
//!
//! ## Safety & Monitoring (Critical for Industrial Use)
//! - `EmergencyStopNode` - Hardware emergency stop handler
//! - `SafetyMonitorNode` - Critical safety system monitoring
//!
//! ## Sensor Interfaces (Essential Building Blocks)
//! - `CameraNode` - Vision input from cameras
//! - `DepthCameraNode` - RGB-D cameras (RealSense, ZED, Kinect, etc.)
//! - `LidarNode` - LiDAR scanning for mapping/obstacles
//! - `ImuNode` - Inertial measurement unit for orientation
//! - `EncoderNode` - Wheel encoder feedback
//! - `GpsNode` - GPS/GNSS positioning for outdoor navigation
//! - `UltrasonicNode` - Ultrasonic distance sensors (HC-SR04, JSN-SR04T, etc.)
//! - `BatteryMonitorNode` - Battery voltage, current, and health monitoring
//! - `ForceTorqueSensorNode` - 6-axis force/torque sensors (ATI, Robotiq, OnRobot, etc.)
//!
//! ## Control & Actuation (Movement and Control)
//! - `DcMotorNode` - DC motor control with PWM (L298N, TB6612, etc.)
//! - `BldcMotorNode` - Brushless DC motor control (ESC protocols: PWM, DShot, OneShot, CAN)
//! - `StepperMotorNode` - Stepper motor control (A4988, DRV8825, TMC2208, etc.)
//! - `DifferentialDriveNode` - Mobile robot base control
//! - `DynamixelNode` - Dynamixel smart servo control (Protocol 1.0/2.0)
//! - `RoboclawMotorNode` - Roboclaw motor controller (BasicMicro 2x7A to 2x160A models)
//! - `PidControllerNode` - Generic PID control
//! - `ServoControllerNode` - RC/Industrial servo control
//!
//! ## Navigation (Path Planning and Localization)
//! - `PathPlannerNode` - A*/RRT path planning algorithms
//! - `LocalizationNode` - Robot position estimation
//! - `CollisionDetectorNode` - Real-time collision avoidance
//!
//! ## Industrial Integration (Production Ready)
//! - `CanBusNode` - CAN bus communication (SocketCAN, automotive, industrial)
//! - `ModbusNode` - Modbus TCP/RTU protocol handler
//! - `DigitalIONode` - Digital I/O interface
//! - `SerialNode` - UART/Serial communication (GPS, Arduino, sensors)
//! - `I2cBusNode` - I2C bus communication for sensors and peripherals
//!
//! ## Vision & Image Processing
//! - `ImageProcessorNode` - Image preprocessing and filtering
//!
//! ## Input Devices
//! - `KeyboardInputNode` - Keyboard input capture
//! - `JoystickInputNode` - Gamepad/joystick input
//!
//! # Usage Examples
//!
//! ```rust,ignore
//! use horus_library::nodes::*;
//!
//! // Create nodes with simple constructors
//! let camera = CameraNode::new();                    // Uses "camera.image" topic
//! let lidar = LidarNode::new();                      // Uses "scan" topic
//! let drive = DifferentialDriveNode::new();          // Subscribes to "cmd_vel"
//! let pid = PidControllerNode::new();                // Generic PID control
//! let emergency = EmergencyStopNode::new();          // Emergency stop handler
//! let safety = SafetyMonitorNode::new();             // Safety monitoring
//!
//! // Or with custom topics
//! let front_camera = CameraNode::new_with_topic("front_camera");
//! let motor_pid = PidControllerNode::new_with_topics("motor_setpoint", "encoder_feedback", "motor_output", "pid_config");
//!
//! // Configure as needed
//! let mut camera = CameraNode::new();
//! camera.set_resolution(1920, 1080);
//! camera.set_fps(30);
//!
//! let mut drive = DifferentialDriveNode::new();
//! drive.set_wheel_base(0.5);
//! drive.set_velocity_limits(2.0, 3.14);
//! ```

// Declare node modules (each in its own folder with README.md)
//
// Hardware nodes are feature-gated - they only compile when the corresponding
// hardware feature is enabled. This ensures clear compile-time errors if users
// try to use hardware without proper drivers installed.
//
// For simulation/testing, use the sim2d or sim3d tools instead.

// Processor trait and utilities for hybrid node pattern
pub mod processor;

// Hardware-independent nodes (always available)
pub mod collision_detector;
pub mod differential_drive;
pub mod emergency_stop;
pub mod localization;
pub mod odometry;
pub mod path_planner;
pub mod pid_controller;
pub mod safety_monitor;

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

// Re-export node types for convenience
//
// Hardware-independent nodes (always available)
pub use collision_detector::CollisionDetectorNode;
pub use differential_drive::DifferentialDriveNode;
pub use emergency_stop::EmergencyStopNode;
pub use localization::LocalizationNode;
pub use odometry::OdometryNode;
pub use path_planner::PathPlannerNode;
pub use pid_controller::PidControllerNode;
pub use safety_monitor::SafetyMonitorNode;

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
pub use horus_core::{Hub, Node, NodeInfo};

// Re-export processor types for hybrid pattern
pub use processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor, ProcessorExt,
};
