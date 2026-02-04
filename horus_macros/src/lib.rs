//! # HORUS Macros
//!
//! Procedural macros for the HORUS robotics framework.
//!
//! This crate provides derive macros and function-like macros to reduce
//! boilerplate and improve the developer experience when building HORUS applications.
//!
//! ## Available Macros
//!
//! - `node!` - Generate Node trait implementation with automatic topic registration
//! - `message!` - Define message types with automatic zero-copy and String conversion
//!
//! ## Safety
//!
//! These macros generate safe code and use proper error handling with `HorusError`.
//! All generated code follows Rust safety guidelines and avoids undefined behavior.

use proc_macro::TokenStream;

mod message;
mod node;
mod ros2_msg;

/// Generate a HORUS node implementation with automatic topic registration.
///
/// # Example
///
/// ```rust,ignore
/// use horus_macros::node;
/// use horus::prelude::*;
///
/// node! {
///     CameraNode {
///         pub {
///             image: Image -> "camera.image",
///             status: Status -> "camera.status",
///         }
///
///         sub {
///             command: Command -> "camera.command",
///         }
///
///         data {
///             frame_count: u32 = 0,
///             buffer: Vec<u8> = Vec::new(),
///         }
///
///         tick(_ctx) {
///             // Process one message per tick (bounded execution time)
///             if let Some(cmd) = self.command.recv() {
///                 // Process command
///             }
///             self.frame_count += 1;
///             let img = self.capture_frame();
///             self.image.send(img).ok();
///         }
///     }
/// }
/// ```
///
/// This generates:
/// - Complete struct definition with Topic fields
/// - `new()` constructor that creates all Topics
/// - `Node` trait implementation
/// - `Default` trait implementation
/// - Automatic snake_case node naming
///
/// # Sections
///
/// - `pub {}` - Publishers (optional, can be empty)
/// - `sub {}` - Subscribers (optional, can be empty)
/// - `data {}` - Internal state fields (optional)
/// - `tick {}` - Main update logic (required)
/// - `init(ctx) {}` - Initialization (optional)
/// - `shutdown(ctx) {}` - Cleanup (optional)
/// - `impl {}` - Additional methods (optional)
#[proc_macro]
pub fn node(input: TokenStream) -> TokenStream {
    node::impl_node_macro(input)
}

/// Define a HORUS message type with automatic trait implementations.
///
/// This macro generates a message type with all necessary traits:
/// - `Debug`, `Clone`, `Serialize`, `Deserialize`
/// - `LogSummary` (for efficient logging without cloning)
/// - `Pod`, `Zeroable` (automatic zero-copy for Pod-compatible types)
/// - String field conversion to `FixedString<N>` for zero-copy
/// - Auto-generated string accessors for String fields
///
/// # Syntax
///
/// ## Tuple-style (recommended for simple types):
///
/// ```rust,ignore
/// use horus_macros::message;
///
/// message!(Position = (f32, f32));
/// message!(Color = (u8, u8, u8));
/// message!(Command = (u32, bool));
/// ```
///
/// ## Struct-style (for complex types):
///
/// ```rust,ignore
/// message! {
///     RobotStatus {
///         position_x: f32,
///         position_y: f32,
///         battery: u8,
///         is_moving: bool,
///     }
/// }
/// ```
///
/// ## With String fields (automatic conversion):
///
/// ```rust,ignore
/// message! {
///     RobotInfo {
///         #[max_len = 32]
///         name: String,  // Becomes FixedString<32> internally
///
///         id: u32,
///     }
/// }
///
/// // Use with regular strings:
/// let info = RobotInfo::new("turtlebot_01", 1);
/// println!("{}", info.name());  // Returns &str
/// info.set_name("turtlebot_02");
///
/// // Still zero-copy!
/// let bytes = info.as_bytes();
/// ```
///
/// # Generated Code
///
/// For `message!(Position = (f32, f32))`, generates:
///
/// ```rust,ignore
/// #[derive(Debug, Clone, Serialize, Deserialize)]
/// #[repr(C)]
/// pub struct Position(pub f32, pub f32);
///
/// impl LogSummary for Position {
///     fn log_summary(&self) -> String {
///         format!("{:?}", self)
///     }
/// }
///
/// unsafe impl bytemuck::Pod for Position { }
/// unsafe impl bytemuck::Zeroable for Position { }
/// ```
///
/// # Usage with Topic
///
/// ```rust,ignore
/// message!(Position = (f32, f32));
///
/// let topic = Topic::<Position>::new("robot.position")?;
/// topic.send(Position(1.0, 2.0))?;  // Works automatically!
/// ```
///
/// # Benefits
///
/// - **Zero boilerplate**: One line defines everything
/// - **No breaking changes**: LogSummary is auto-implemented
/// - **Performance**: Large messages can override LogSummary manually
/// - **Type safety**: All fields are strongly typed
#[proc_macro]
pub fn message(input: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(input as message::MessageInput);
    let output = message::generate_message(input);
    TokenStream::from(output)
}

/// Generate ROS2-compatible message types with CDR serialization support.
///
/// This macro provides two modes:
///
/// ## Import Mode
///
/// Import well-known ROS2 message types from `horus_ros2_msgs`:
///
/// ```rust,ignore
/// use horus_macros::ros2_msg;
///
/// ros2_msg!(geometry_msgs::Twist);
/// ros2_msg!(sensor_msgs::Imu);
/// ros2_msg!(nav_msgs::Odometry);
/// ```
///
/// ## Define Mode
///
/// Define custom ROS2-compatible messages with inline syntax:
///
/// ```rust,ignore
/// ros2_msg! {
///     my_msgs::RobotState {
///         std_msgs::Header header,
///         geometry_msgs::Pose pose,
///         geometry_msgs::Twist velocity,
///         float64 battery_level,
///         string status_message,
///         float64[] sensor_readings,
///     }
/// }
/// ```
///
/// # Supported Types
///
/// ## Primitives
/// - `bool`, `byte`, `char`
/// - `int8`, `int16`, `int32`, `int64`
/// - `uint8`, `uint16`, `uint32`, `uint64`
/// - `float32`, `float64`
/// - `string`
///
/// ## Arrays
/// - `float64[]` - Unbounded array (becomes `Vec<f64>`)
/// - `float64[36]` - Fixed-size array (becomes `[f64; 36]`)
///
/// ## Nested Messages
/// - `geometry_msgs::Point position` - References horus_ros2_msgs types
///
/// # Generated Code
///
/// The macro generates:
/// - Struct with serde derives for JSON/CDR serialization
/// - `LogSummary` implementation for efficient logging
/// - Constructor method `new(...)`
/// - `Default` implementation
/// - `Copy` derive (when possible - no strings/arrays)
///
/// # CDR Compatibility
///
/// Generated types are wire-compatible with ROS2 when serialized with
/// `cdr-encoding`. This enables direct communication with ROS2 nodes
/// via Zenoh or DDS.
#[proc_macro]
pub fn ros2_msg(input: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(input as ros2_msg::Ros2MsgInput);
    let output = ros2_msg::generate_ros2_msg(input);
    TokenStream::from(output)
}
