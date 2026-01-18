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
//! - `message!` - Define message types with serialization traits
//! - `zero_copy_message!` - Define zero-copy messages with compile-time layout verification
//! - `fixed_string!` - Generate fixed-size string types for zero-copy messages
//!
//! ## Safety
//!
//! These macros generate safe code and use proper error handling with `HorusError`.
//! All generated code follows Rust safety guidelines and avoids undefined behavior.

use proc_macro::TokenStream;

mod endpoint;
mod message;
mod node;
mod zero_copy;

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
///         tick(ctx) {
///             // Process one message per tick (bounded execution time)
///             if let Some(cmd) = self.command.recv(ctx) {
///                 // Process command
///             }
///             self.frame_count += 1;
///             let img = self.capture_frame();
///             self.image.send(img, ctx).ok();
///         }
///     }
/// }
/// ```
///
/// This generates:
/// - Complete struct definition with Hub fields
/// - `new()` constructor that creates all Hubs
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
/// - `Pod`, `Zeroable` (for zero-copy serialization, if fields support it)
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
/// # Usage with Hub
///
/// ```rust,ignore
/// message!(Position = (f32, f32));
///
/// let hub = Hub::<Position>::new("robot.position")?;
/// hub.send(Position(1.0, 2.0), ctx)?;  // Works automatically!
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

/// Define a zero-copy message with compile-time verified layout.
///
/// This macro creates message types optimized for high-performance recording:
/// - **Compile-time size calculation**: Know exact message size at compile time
/// - **Memory alignment guarantees**: Proper alignment for zero-copy access
/// - **bytemuck integration**: Automatic Pod + Zeroable for direct memory access
/// - **Efficient serialization**: Direct memory copy without per-field encoding
///
/// # Example
///
/// ```rust,ignore
/// use horus_macros::zero_copy_message;
///
/// zero_copy_message! {
///     /// IMU sensor reading with fixed layout
///     ImuReading {
///         timestamp_ns: u64,
///         accel: [f32; 3],
///         gyro: [f32; 3],
///         mag: [f32; 3],
///         temperature: f32,
///         status: u8,
///         _padding: [u8; 3],  // Explicit padding for alignment
///     }
/// }
///
/// // Use with zero-copy recording
/// let reading = ImuReading {
///     timestamp_ns: 1234567890,
///     accel: [0.0, 9.8, 0.0],
///     gyro: [0.0, 0.0, 0.0],
///     mag: [0.3, 0.0, 0.5],
///     temperature: 25.0,
///     status: 0x01,
///     _padding: [0; 3],
/// };
///
/// // Get raw bytes for recording (zero-copy)
/// let bytes = reading.as_bytes();
/// assert_eq!(bytes.len(), ImuReading::SIZE);
///
/// // Reconstruct from bytes (zero-copy)
/// let restored = ImuReading::from_bytes(bytes).unwrap();
/// assert_eq!(restored, reading);
/// ```
///
/// # Generated Code
///
/// The macro generates:
/// - `#[repr(C, packed)]` struct with all fields public
/// - `Pod` and `Zeroable` unsafe impl for bytemuck
/// - `Default` impl with zeroed values
/// - `SIZE` constant for compile-time size
/// - `as_bytes()` and `from_bytes()` for zero-copy access
/// - `LogSummary` impl for logging
/// - `Serialize` impl for JSON/MessagePack compatibility
///
/// # Constraints
///
/// All fields must be primitive types or fixed-size arrays of primitives:
/// - Integers: `u8`, `u16`, `u32`, `u64`, `i8`, `i16`, `i32`, `i64`
/// - Floats: `f32`, `f64`
/// - Arrays: `[T; N]` where T is a primitive
/// - Fixed strings: Use `FixedString<N>` (see `fixed_string!` macro)
///
/// # Performance
///
/// Zero-copy messages are ~10-100x faster than serde-based serialization:
/// - No allocation during serialization
/// - Direct memory copy
/// - Compile-time size known
/// - Cache-friendly memory layout
#[proc_macro]
pub fn zero_copy_message(input: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(input as zero_copy::ZeroCopyMessageInput);
    let output = zero_copy::generate_zero_copy_message(input);
    TokenStream::from(output)
}

/// Generate a fixed-size string type for zero-copy messages.
///
/// Fixed-size strings have a known compile-time size, making them suitable
/// for zero-copy message layouts.
///
/// # Example
///
/// ```rust,ignore
/// use horus_macros::fixed_string;
///
/// // Generate FixedString32 type (32 bytes capacity)
/// fixed_string!(32);
///
/// let name = FixedString32::from_str("robot_01");
/// assert_eq!(name.as_str(), "robot_01");
///
/// // Use in zero-copy messages
/// zero_copy_message! {
///     RobotInfo {
///         name: FixedString32,
///         id: u32,
///         status: u8,
///         _padding: [u8; 3],
///     }
/// }
/// ```
///
/// # Generated Type
///
/// For `fixed_string!(32)`, generates `FixedString32` with:
/// - 32 bytes data storage
/// - 1 byte length
/// - `Pod` and `Zeroable` for bytemuck
/// - `from_str()`, `as_str()` conversions
/// - `Debug`, `Display` formatting
#[proc_macro]
pub fn fixed_string(input: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(input as zero_copy::FixedStringInput);
    let output = zero_copy::generate_fixed_string(input);
    TokenStream::from(output)
}

/// Validate a HORUS endpoint string at compile time.
///
/// This macro validates endpoint syntax at compile time, catching errors before
/// your code runs. It also provides IDE hints for valid endpoint formats.
///
/// # Example
///
/// ```rust,ignore
/// use horus_macros::endpoint;
///
/// // Local topic (same process)
/// let ep = endpoint!("sensor_data");
///
/// // Multicast discovery (LAN)
/// let ep = endpoint!("cmd_vel@*");
///
/// // Direct connection
/// let ep = endpoint!("telemetry@192.168.1.50:9000");
///
/// // mDNS hostname
/// let ep = endpoint!("lidar@robot-arm.local");
///
/// // Zenoh transport
/// let ep = endpoint!("odom@zenoh");
/// let ep = endpoint!("/scan@zenoh/ros2");
///
/// // P2P with NAT traversal
/// let ep = endpoint!("video@p2p:a3f7-k2m9-p4n8");
/// let ep = endpoint!("control@p2p:a3f7-k2m9-p4n8/stun");
///
/// // Cloud room
/// let ep = endpoint!("fleet_status@cloud:factory-1");
/// ```
///
/// # Compile-Time Validation
///
/// Invalid endpoints produce compile errors:
///
/// ```compile_fail
/// // Error: Topic name cannot be empty
/// let ep = endpoint!("@192.168.1.1");
///
/// // Error: Invalid peer ID format
/// let ep = endpoint!("topic@p2p:invalid");
///
/// // Error: Invalid IPv4 address
/// let ep = endpoint!("topic@999.0.0.1");
/// ```
///
/// # Supported Formats
///
/// | Pattern | Description |
/// |---------|-------------|
/// | `topic` | Local shared memory |
/// | `topic@localhost` | Same machine (Unix socket) |
/// | `topic@192.168.x.x` | Direct UDP to IP |
/// | `topic@192.168.x.x:port` | Direct UDP with port |
/// | `topic@[ipv6]` | IPv6 address |
/// | `topic@[ipv6]:port` | IPv6 with port |
/// | `topic@*` | Multicast discovery |
/// | `topic@host.local` | mDNS hostname |
/// | `topic@router` | Central router |
/// | `topic@zenoh` | Zenoh mesh |
/// | `topic@zenoh/ros2` | Zenoh ROS2 mode |
/// | `topic@zenoh:cloud` | Zenoh cloud |
/// | `topic@p2p:xxxx-yyyy-zzzz` | P2P with peer ID |
/// | `topic@p2p:xxxx-yyyy-zzzz/stun` | P2P with strategy |
/// | `topic@cloud:room` | Cloud room |
/// | `topic@relay:host` | Self-hosted relay |
/// | `topic@vpn:peer` | VPN connection |
///
/// # IDE Integration
///
/// IDEs with rust-analyzer will show:
/// - Autocomplete suggestions for endpoint patterns
/// - Hover documentation for endpoint syntax
/// - Compile-time error highlighting for invalid endpoints
#[proc_macro]
pub fn endpoint(input: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(input as endpoint::EndpointInput);
    let output = endpoint::generate_endpoint_macro(input);
    TokenStream::from(output)
}
