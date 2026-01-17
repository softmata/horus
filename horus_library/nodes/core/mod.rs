//! Core HORUS Nodes - Hardware Independent
//!
//! These 8 nodes form the foundation of every HORUS robotics application.
//! They have zero hardware dependencies and are always available.
//!
//! ## Safety & Monitoring
//! - `EmergencyStopNode` - Hardware emergency stop handler
//! - `SafetyMonitorNode` - Critical safety system monitoring
//!
//! ## Motion & Control
//! - `DifferentialDriveNode` - Mobile robot base control (2-wheel differential)
//! - `OdometryNode` - Wheel odometry calculation
//! - `PidControllerNode` - Generic PID feedback control
//!
//! ## Navigation
//! - `PathPlannerNode` - A*/RRT path planning algorithms
//! - `LocalizationNode` - Robot position estimation
//! - `CollisionDetectorNode` - Real-time collision avoidance
//!
//! These nodes depend only on `horus_core` and pure Rust algorithms.

// Processor trait and utilities for hybrid node pattern
pub mod processor;

// Core nodes - always available, no feature gates
pub mod collision_detector;
pub mod differential_drive;
pub mod emergency_stop;
pub mod localization;
pub mod odometry;
pub mod path_planner;
pub mod pid_controller;
pub mod safety_monitor;

// Re-export node types
pub use collision_detector::CollisionDetectorNode;
pub use differential_drive::DifferentialDriveNode;
pub use emergency_stop::EmergencyStopNode;
pub use localization::LocalizationNode;
pub use odometry::OdometryNode;
pub use path_planner::PathPlannerNode;
pub use pid_controller::PidControllerNode;
pub use safety_monitor::SafetyMonitorNode;

// Re-export processor types for hybrid pattern
pub use processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor, ProcessorExt,
};
