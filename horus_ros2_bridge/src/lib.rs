//! ROS2 Protocol Bridge for HORUS via Zenoh
//!
//! This crate provides ROS2-compatible protocol implementations built on top of
//! Zenoh transport, enabling HORUS nodes to interoperate with ROS2 systems.
//!
//! # Features
//!
//! - **Services**: Request/response patterns compatible with ROS2 services
//! - **Actions**: Goal-based behaviors with feedback and cancellation
//! - **Parameters**: ROS2-style parameter server for runtime configuration
//!
//! # Protocol Implementations
//!
//! All protocols use Zenoh topics with CDR (Common Data Representation) encoding
//! for wire-level compatibility with ROS2/DDS systems.
//!
//! ## ROS2 Services
//!
//! Request/response communication over Zenoh topics:
//! - Request topic: `<service_name>/rq`
//! - Response topic: `<service_name>/rs`
//!
//! ## ROS2 Actions
//!
//! Goal-based behaviors with feedback:
//! - Send goal: `<action_name>/_action/send_goal`
//! - Cancel goal: `<action_name>/_action/cancel_goal`
//! - Get result: `<action_name>/_action/get_result`
//! - Feedback: `<action_name>/_action/feedback`
//! - Status: `<action_name>/_action/status`
//!
//! ## ROS2 Parameters
//!
//! Runtime configuration management:
//! - List parameters: `<node>/list_parameters`
//! - Get parameters: `<node>/get_parameters`
//! - Set parameters: `<node>/set_parameters`
//! - Describe parameters: `<node>/describe_parameters`
//! - Parameter events: `<node>/parameter_events`
//!
//! # Example Usage
//!
//! ```rust,ignore
//! use horus_ros2_bridge::{Ros2ServiceRequest, Ros2ServiceResponse};
//!
//! // Create a ROS2 service client
//! let mut client = Ros2ServiceClient::<MyRequest, MyResponse>::new("my_service").await?;
//!
//! // Call the service
//! let request = MyRequest { ... };
//! let response = client.call(request).await?;
//! ```

// Re-export horus_core for convenience
pub use horus_core;

// ROS2 service protocol
pub mod zenoh_ros2_services;
pub use zenoh_ros2_services::*;

// ROS2 action protocol
pub mod zenoh_ros2_actions;
pub use zenoh_ros2_actions::*;

// ROS2 parameter protocol
pub mod zenoh_ros2_params;
pub use zenoh_ros2_params::*;
