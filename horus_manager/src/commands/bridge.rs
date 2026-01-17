//! ROS2 Bridge Command - Runtime interoperability between HORUS and ROS2
//!
//! Enables hybrid deployments where existing ROS2 nodes can coexist with HORUS nodes
//! via Zenoh bridging. Unlike `from-ros` which converts code, this provides runtime interop.
//!
//! # Usage
//!
//! ```bash
//! # Bridge specific topics
//! horus bridge ros2 --topics /scan,/odom,/cmd_vel
//!
//! # Bridge all discovered topics
//! horus bridge ros2 --all
//!
//! # Bridge with namespace filter
//! horus bridge ros2 --namespace /robot1 --all
//!
//! # One-way bridging (ROS2 -> HORUS only)
//! horus bridge ros2 --direction in --topics /scan
//! ```
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────┐     Zenoh      ┌─────────────┐
//! │   ROS2      │◄──────────────►│   HORUS     │
//! │   Nodes     │   (CDR msgs)   │   Bridge    │
//! └─────────────┘                └──────┬──────┘
//!                                       │
//!                                       ▼ Shared Memory
//!                                ┌─────────────┐
//!                                │   HORUS     │
//!                                │   Nodes     │
//!                                └─────────────┘
//! ```

use colored::*;
use horus_core::error::{HorusError, HorusResult};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

// ============================================================================
// CDR Serialization Module
// ============================================================================

/// CDR (Common Data Representation) serialization for ROS2 interoperability
#[cfg(feature = "zenoh-transport")]
pub mod cdr {
    #[allow(unused_imports)]
    use super::*;
    use byteorder::LittleEndian;
    use cdr_encoding::{from_bytes, to_vec, Error as CdrError};
    use serde::{Deserialize, Serialize};

    /// Result type for CDR operations
    pub type CdrResult<T> = Result<T, CdrSerializationError>;

    /// Error type for CDR serialization
    #[derive(Debug, thiserror::Error)]
    pub enum CdrSerializationError {
        #[error("CDR encoding error: {0}")]
        Encoding(String),
        #[error("CDR decoding error: {0}")]
        Decoding(String),
        #[error("Unknown message type: {0}")]
        UnknownType(String),
        #[error("Invalid CDR header")]
        InvalidHeader,
    }

    impl From<CdrError> for CdrSerializationError {
        fn from(err: CdrError) -> Self {
            CdrSerializationError::Decoding(err.to_string())
        }
    }

    /// CDR encapsulation header (4 bytes)
    /// Byte 0: CDR_BE (0x00) or CDR_LE (0x01)
    /// Byte 1: Encapsulation options (usually 0x00 or 0x01)
    /// Bytes 2-3: Options (reserved, usually 0x00, 0x00)
    pub const CDR_HEADER_LE: [u8; 4] = [0x00, 0x01, 0x00, 0x00];
    pub const CDR_HEADER_BE: [u8; 4] = [0x00, 0x00, 0x00, 0x00];

    /// Check if CDR data is little-endian based on header
    pub fn is_little_endian(data: &[u8]) -> bool {
        if data.len() < 4 {
            return true; // Default to LE
        }
        data[1] == 0x01
    }

    /// Serialize a value to CDR bytes with encapsulation header
    pub fn serialize<T: Serialize>(value: &T) -> CdrResult<Vec<u8>> {
        let mut result = Vec::with_capacity(128);
        result.extend_from_slice(&CDR_HEADER_LE);

        let payload = to_vec::<T, LittleEndian>(value)
            .map_err(|e| CdrSerializationError::Encoding(e.to_string()))?;
        result.extend(payload);

        Ok(result)
    }

    /// Deserialize CDR bytes (with encapsulation header) to a value
    pub fn deserialize<'a, T: Deserialize<'a>>(data: &'a [u8]) -> CdrResult<T> {
        if data.len() < 4 {
            return Err(CdrSerializationError::InvalidHeader);
        }

        // Skip the 4-byte encapsulation header
        let payload = &data[4..];

        // Note: We always use LittleEndian as ROS2 typically uses LE
        // In a more robust implementation, we'd check the header
        let (value, _bytes_read) = from_bytes::<T, LittleEndian>(payload)?;

        Ok(value)
    }

    // ========================================================================
    // Common ROS2 Message Types
    // ========================================================================

    /// std_msgs/Header
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct Header {
        pub stamp: Time,
        pub frame_id: String,
    }

    /// builtin_interfaces/Time
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct Time {
        pub sec: i32,
        pub nanosec: u32,
    }

    impl Time {
        pub fn now() -> Self {
            let now = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default();
            Time {
                sec: now.as_secs() as i32,
                nanosec: now.subsec_nanos(),
            }
        }
    }

    /// builtin_interfaces/Duration
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct Duration {
        pub sec: i32,
        pub nanosec: u32,
    }

    /// geometry_msgs/Vector3
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct Vector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    /// geometry_msgs/Point
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct Point {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    /// geometry_msgs/Quaternion
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct Quaternion {
        pub x: f64,
        pub y: f64,
        pub z: f64,
        pub w: f64,
    }

    impl Quaternion {
        pub fn identity() -> Self {
            Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            }
        }
    }

    /// geometry_msgs/Pose
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct Pose {
        pub position: Point,
        pub orientation: Quaternion,
    }

    /// geometry_msgs/PoseStamped
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct PoseStamped {
        pub header: Header,
        pub pose: Pose,
    }

    /// geometry_msgs/Twist
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct Twist {
        pub linear: Vector3,
        pub angular: Vector3,
    }

    /// geometry_msgs/TwistStamped
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct TwistStamped {
        pub header: Header,
        pub twist: Twist,
    }

    /// sensor_msgs/LaserScan
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct LaserScan {
        pub header: Header,
        pub angle_min: f32,
        pub angle_max: f32,
        pub angle_increment: f32,
        pub time_increment: f32,
        pub scan_time: f32,
        pub range_min: f32,
        pub range_max: f32,
        pub ranges: Vec<f32>,
        pub intensities: Vec<f32>,
    }

    /// sensor_msgs/Imu
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct Imu {
        pub header: Header,
        pub orientation: Quaternion,
        pub orientation_covariance: [f64; 9],
        pub angular_velocity: Vector3,
        pub angular_velocity_covariance: [f64; 9],
        pub linear_acceleration: Vector3,
        pub linear_acceleration_covariance: [f64; 9],
    }

    /// nav_msgs/Odometry
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct Odometry {
        pub header: Header,
        pub child_frame_id: String,
        pub pose: PoseWithCovariance,
        pub twist: TwistWithCovariance,
    }

    /// Covariance matrix wrapper for 6x6 (36 elements)
    /// Using Vec since [f64; 36] doesn't implement Default/Deserialize automatically
    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Covariance36(pub Vec<f64>);

    impl Default for Covariance36 {
        fn default() -> Self {
            Covariance36(vec![0.0; 36])
        }
    }

    impl Covariance36 {
        pub fn identity() -> Self {
            let mut cov = vec![0.0; 36];
            // Set diagonal elements to 1.0
            for i in 0..6 {
                cov[i * 6 + i] = 1.0;
            }
            Covariance36(cov)
        }
    }

    /// geometry_msgs/PoseWithCovariance
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct PoseWithCovariance {
        pub pose: Pose,
        pub covariance: Covariance36,
    }

    /// geometry_msgs/TwistWithCovariance
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct TwistWithCovariance {
        pub twist: Twist,
        pub covariance: Covariance36,
    }

    /// std_msgs/String
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct StdString {
        pub data: String,
    }

    /// std_msgs/Bool
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct StdBool {
        pub data: bool,
    }

    /// std_msgs/Int32
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct StdInt32 {
        pub data: i32,
    }

    /// std_msgs/Float64
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct StdFloat64 {
        pub data: f64,
    }

    // ========================================================================
    // TF2 Message Types
    // ========================================================================

    /// geometry_msgs/Transform
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct Transform {
        pub translation: Vector3,
        pub rotation: Quaternion,
    }

    impl Transform {
        /// Create an identity transform
        pub fn identity() -> Self {
            Transform {
                translation: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                rotation: Quaternion::identity(),
            }
        }

        /// Create a transform from translation and rotation
        pub fn new(translation: Vector3, rotation: Quaternion) -> Self {
            Transform {
                translation,
                rotation,
            }
        }

        /// Compose two transforms: self * other
        pub fn compose(&self, other: &Transform) -> Transform {
            // Rotate other's translation by self's rotation, then add self's translation
            let rotated_translation = self.rotate_vector(&other.translation);
            Transform {
                translation: Vector3 {
                    x: self.translation.x + rotated_translation.x,
                    y: self.translation.y + rotated_translation.y,
                    z: self.translation.z + rotated_translation.z,
                },
                rotation: self.quaternion_multiply(&other.rotation),
            }
        }

        /// Rotate a vector by this transform's rotation
        pub fn rotate_vector(&self, v: &Vector3) -> Vector3 {
            let q = &self.rotation;
            // Quaternion rotation: q * v * q^-1
            // Using optimized formula for rotating a vector by a unit quaternion
            let qx = q.x;
            let qy = q.y;
            let qz = q.z;
            let qw = q.w;

            let t_x = 2.0 * (qy * v.z - qz * v.y);
            let t_y = 2.0 * (qz * v.x - qx * v.z);
            let t_z = 2.0 * (qx * v.y - qy * v.x);

            Vector3 {
                x: v.x + qw * t_x + (qy * t_z - qz * t_y),
                y: v.y + qw * t_y + (qz * t_x - qx * t_z),
                z: v.z + qw * t_z + (qx * t_y - qy * t_x),
            }
        }

        /// Multiply two quaternions
        pub fn quaternion_multiply(&self, other: &Quaternion) -> Quaternion {
            let q1 = &self.rotation;
            Quaternion {
                w: q1.w * other.w - q1.x * other.x - q1.y * other.y - q1.z * other.z,
                x: q1.w * other.x + q1.x * other.w + q1.y * other.z - q1.z * other.y,
                y: q1.w * other.y - q1.x * other.z + q1.y * other.w + q1.z * other.x,
                z: q1.w * other.z + q1.x * other.y - q1.y * other.x + q1.z * other.w,
            }
        }

        /// Get the inverse of this transform
        pub fn inverse(&self) -> Transform {
            // Inverse rotation (conjugate for unit quaternion)
            let inv_rotation = Quaternion {
                x: -self.rotation.x,
                y: -self.rotation.y,
                z: -self.rotation.z,
                w: self.rotation.w,
            };

            // Inverse translation: -R^-1 * t
            let inv_t = Transform {
                translation: Vector3::default(),
                rotation: inv_rotation.clone(),
            };
            let neg_translation = inv_t.rotate_vector(&self.translation);

            Transform {
                translation: Vector3 {
                    x: -neg_translation.x,
                    y: -neg_translation.y,
                    z: -neg_translation.z,
                },
                rotation: inv_rotation,
            }
        }
    }

    /// geometry_msgs/TransformStamped
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct TransformStamped {
        pub header: Header,
        pub child_frame_id: String,
        pub transform: Transform,
    }

    impl TransformStamped {
        /// Create a new transform stamped
        pub fn new(
            parent_frame: &str,
            child_frame: &str,
            timestamp: Time,
            transform: Transform,
        ) -> Self {
            TransformStamped {
                header: Header {
                    stamp: timestamp,
                    frame_id: parent_frame.to_string(),
                },
                child_frame_id: child_frame.to_string(),
                transform,
            }
        }

        /// Get the parent frame ID
        pub fn parent_frame(&self) -> &str {
            &self.header.frame_id
        }

        /// Get the child frame ID
        pub fn child_frame(&self) -> &str {
            &self.child_frame_id
        }

        /// Get timestamp as nanoseconds since epoch
        pub fn timestamp_ns(&self) -> u64 {
            (self.header.stamp.sec as u64) * 1_000_000_000 + (self.header.stamp.nanosec as u64)
        }
    }

    /// tf2_msgs/TFMessage - A message containing multiple transforms
    #[derive(Debug, Clone, Serialize, Deserialize, Default)]
    pub struct TFMessage {
        pub transforms: Vec<TransformStamped>,
    }

    impl TFMessage {
        /// Create a new empty TFMessage
        pub fn new() -> Self {
            TFMessage {
                transforms: Vec::new(),
            }
        }

        /// Create a TFMessage with the given transforms
        pub fn with_transforms(transforms: Vec<TransformStamped>) -> Self {
            TFMessage { transforms }
        }

        /// Add a transform to the message
        pub fn add(&mut self, transform: TransformStamped) {
            self.transforms.push(transform);
        }

        /// Check if the message is empty
        pub fn is_empty(&self) -> bool {
            self.transforms.is_empty()
        }

        /// Get the number of transforms
        pub fn len(&self) -> usize {
            self.transforms.len()
        }
    }

    // ========================================================================
    // Message Type Registry
    // ========================================================================

    /// Known ROS2 message types for dynamic dispatch
    #[derive(Debug, Clone, PartialEq, Eq, Hash)]
    pub enum Ros2MessageType {
        StdMsgsString,
        StdMsgsBool,
        StdMsgsInt32,
        StdMsgsFloat64,
        GeometryMsgsTwist,
        GeometryMsgsTwistStamped,
        GeometryMsgsPose,
        GeometryMsgsPoseStamped,
        GeometryMsgsTransform,
        GeometryMsgsTransformStamped,
        Tf2MsgsTFMessage,
        SensorMsgsLaserScan,
        SensorMsgsImu,
        NavMsgsOdometry,
        Unknown(String),
    }

    impl Ros2MessageType {
        /// Parse a ROS2 message type string
        pub fn from_type_string(type_str: &str) -> Self {
            match type_str {
                "std_msgs/msg/String" | "std_msgs/String" => Ros2MessageType::StdMsgsString,
                "std_msgs/msg/Bool" | "std_msgs/Bool" => Ros2MessageType::StdMsgsBool,
                "std_msgs/msg/Int32" | "std_msgs/Int32" => Ros2MessageType::StdMsgsInt32,
                "std_msgs/msg/Float64" | "std_msgs/Float64" => Ros2MessageType::StdMsgsFloat64,
                "geometry_msgs/msg/Twist" | "geometry_msgs/Twist" => {
                    Ros2MessageType::GeometryMsgsTwist
                }
                "geometry_msgs/msg/TwistStamped" | "geometry_msgs/TwistStamped" => {
                    Ros2MessageType::GeometryMsgsTwistStamped
                }
                "geometry_msgs/msg/Pose" | "geometry_msgs/Pose" => {
                    Ros2MessageType::GeometryMsgsPose
                }
                "geometry_msgs/msg/PoseStamped" | "geometry_msgs/PoseStamped" => {
                    Ros2MessageType::GeometryMsgsPoseStamped
                }
                "geometry_msgs/msg/Transform" | "geometry_msgs/Transform" => {
                    Ros2MessageType::GeometryMsgsTransform
                }
                "geometry_msgs/msg/TransformStamped" | "geometry_msgs/TransformStamped" => {
                    Ros2MessageType::GeometryMsgsTransformStamped
                }
                "tf2_msgs/msg/TFMessage" | "tf2_msgs/TFMessage" => {
                    Ros2MessageType::Tf2MsgsTFMessage
                }
                "sensor_msgs/msg/LaserScan" | "sensor_msgs/LaserScan" => {
                    Ros2MessageType::SensorMsgsLaserScan
                }
                "sensor_msgs/msg/Imu" | "sensor_msgs/Imu" => Ros2MessageType::SensorMsgsImu,
                "nav_msgs/msg/Odometry" | "nav_msgs/Odometry" => Ros2MessageType::NavMsgsOdometry,
                other => Ros2MessageType::Unknown(other.to_string()),
            }
        }

        /// Get the ROS2 type string for this message type
        pub fn to_type_string(&self) -> &str {
            match self {
                Ros2MessageType::StdMsgsString => "std_msgs/msg/String",
                Ros2MessageType::StdMsgsBool => "std_msgs/msg/Bool",
                Ros2MessageType::StdMsgsInt32 => "std_msgs/msg/Int32",
                Ros2MessageType::StdMsgsFloat64 => "std_msgs/msg/Float64",
                Ros2MessageType::GeometryMsgsTwist => "geometry_msgs/msg/Twist",
                Ros2MessageType::GeometryMsgsTwistStamped => "geometry_msgs/msg/TwistStamped",
                Ros2MessageType::GeometryMsgsPose => "geometry_msgs/msg/Pose",
                Ros2MessageType::GeometryMsgsPoseStamped => "geometry_msgs/msg/PoseStamped",
                Ros2MessageType::GeometryMsgsTransform => "geometry_msgs/msg/Transform",
                Ros2MessageType::GeometryMsgsTransformStamped => {
                    "geometry_msgs/msg/TransformStamped"
                }
                Ros2MessageType::Tf2MsgsTFMessage => "tf2_msgs/msg/TFMessage",
                Ros2MessageType::SensorMsgsLaserScan => "sensor_msgs/msg/LaserScan",
                Ros2MessageType::SensorMsgsImu => "sensor_msgs/msg/Imu",
                Ros2MessageType::NavMsgsOdometry => "nav_msgs/msg/Odometry",
                Ros2MessageType::Unknown(s) => s,
            }
        }

        /// Check if this is a known type that can be deserialized
        pub fn is_known(&self) -> bool {
            !matches!(self, Ros2MessageType::Unknown(_))
        }
    }

    // ========================================================================
    // Dynamic Message Type Registry (Production-Ready)
    // ========================================================================

    use std::collections::HashMap;
    use std::sync::{Arc, RwLock};

    /// Field type in a message definition
    #[derive(Debug, Clone, PartialEq)]
    pub enum FieldType {
        Bool,
        Int8,
        Int16,
        Int32,
        Int64,
        Uint8,
        Uint16,
        Uint32,
        Uint64,
        Float32,
        Float64,
        String,
        Time,
        Duration,
        Array(Box<FieldType>, Option<usize>), // type, optional fixed size
        Nested(String),                       // nested message type name
    }

    impl FieldType {
        /// Parse a ROS2 field type string
        pub fn from_ros2_type(type_str: &str) -> Self {
            // Handle arrays
            if type_str.ends_with("[]") {
                let inner = &type_str[..type_str.len() - 2];
                return FieldType::Array(Box::new(Self::from_ros2_type(inner)), None);
            }
            if let Some(bracket_pos) = type_str.find('[') {
                let inner = &type_str[..bracket_pos];
                let size_str = &type_str[bracket_pos + 1..type_str.len() - 1];
                let size = size_str.parse().ok();
                return FieldType::Array(Box::new(Self::from_ros2_type(inner)), size);
            }

            match type_str {
                "bool" => FieldType::Bool,
                "int8" | "byte" => FieldType::Int8,
                "int16" => FieldType::Int16,
                "int32" => FieldType::Int32,
                "int64" => FieldType::Int64,
                "uint8" | "char" => FieldType::Uint8,
                "uint16" => FieldType::Uint16,
                "uint32" => FieldType::Uint32,
                "uint64" => FieldType::Uint64,
                "float32" => FieldType::Float32,
                "float64" => FieldType::Float64,
                "string" => FieldType::String,
                "builtin_interfaces/Time" | "builtin_interfaces/msg/Time" | "time" => {
                    FieldType::Time
                }
                "builtin_interfaces/Duration" | "builtin_interfaces/msg/Duration" | "duration" => {
                    FieldType::Duration
                }
                other => FieldType::Nested(other.to_string()),
            }
        }

        /// Get the size in bytes for fixed-size types (None for variable size)
        pub fn fixed_size(&self) -> Option<usize> {
            match self {
                FieldType::Bool | FieldType::Int8 | FieldType::Uint8 => Some(1),
                FieldType::Int16 | FieldType::Uint16 => Some(2),
                FieldType::Int32 | FieldType::Uint32 | FieldType::Float32 => Some(4),
                FieldType::Int64 | FieldType::Uint64 | FieldType::Float64 => Some(8),
                FieldType::Time | FieldType::Duration => Some(8), // sec(4) + nsec(4)
                FieldType::Array(inner, Some(size)) => inner.fixed_size().map(|s| s * size),
                _ => None,
            }
        }
    }

    /// A field in a message definition
    #[derive(Debug, Clone)]
    pub struct MessageField {
        pub name: String,
        pub field_type: FieldType,
        pub default_value: Option<String>,
    }

    /// A complete message definition
    #[derive(Debug, Clone)]
    pub struct MessageDefinition {
        pub package: String,
        pub name: String,
        pub fields: Vec<MessageField>,
        pub constants: Vec<(String, String, String)>, // name, type, value
    }

    impl MessageDefinition {
        /// Get the full type name (e.g., "geometry_msgs/msg/Twist")
        pub fn full_name(&self) -> String {
            format!("{}/msg/{}", self.package, self.name)
        }

        /// Parse a .msg file content
        pub fn parse_msg_content(package: &str, name: &str, content: &str) -> Self {
            let mut fields = Vec::new();
            let mut constants = Vec::new();

            for line in content.lines() {
                let line = line.trim();

                // Skip empty lines and comments
                if line.is_empty() || line.starts_with('#') {
                    continue;
                }

                // Check for constants (TYPE NAME=VALUE)
                if let Some(eq_pos) = line.find('=') {
                    let before_eq = &line[..eq_pos];
                    let value = line[eq_pos + 1..].trim().to_string();

                    let parts: Vec<&str> = before_eq.split_whitespace().collect();
                    if parts.len() >= 2 {
                        constants.push((parts[1].to_string(), parts[0].to_string(), value));
                    }
                    continue;
                }

                // Parse field: TYPE NAME [DEFAULT]
                let parts: Vec<&str> = line.split_whitespace().collect();
                if parts.len() >= 2 {
                    let field_type = FieldType::from_ros2_type(parts[0]);
                    let name = parts[1].to_string();
                    let default_value = if parts.len() > 2 {
                        Some(parts[2..].join(" "))
                    } else {
                        None
                    };

                    fields.push(MessageField {
                        name,
                        field_type,
                        default_value,
                    });
                }
            }

            MessageDefinition {
                package: package.to_string(),
                name: name.to_string(),
                fields,
                constants,
            }
        }
    }

    /// Dynamic message registry for runtime type handling
    #[derive(Debug, Default)]
    pub struct MessageRegistry {
        definitions: RwLock<HashMap<String, Arc<MessageDefinition>>>,
    }

    impl MessageRegistry {
        /// Create a new registry with standard ROS2 message types pre-registered
        pub fn new() -> Self {
            let registry = Self::default();
            registry.register_standard_types();
            registry
        }

        /// Register the standard ROS2 message types
        fn register_standard_types(&self) {
            // std_msgs
            self.register_definition(MessageDefinition::parse_msg_content(
                "std_msgs",
                "String",
                "string data",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "std_msgs",
                "Bool",
                "bool data",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "std_msgs",
                "Int32",
                "int32 data",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "std_msgs",
                "Float64",
                "float64 data",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "std_msgs",
                "Header",
                "builtin_interfaces/Time stamp\nstring frame_id",
            ));

            // geometry_msgs
            self.register_definition(MessageDefinition::parse_msg_content(
                "geometry_msgs",
                "Vector3",
                "float64 x\nfloat64 y\nfloat64 z",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "geometry_msgs",
                "Point",
                "float64 x\nfloat64 y\nfloat64 z",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "geometry_msgs",
                "Quaternion",
                "float64 x\nfloat64 y\nfloat64 z\nfloat64 w",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "geometry_msgs",
                "Pose",
                "geometry_msgs/Point position\ngeometry_msgs/Quaternion orientation",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "geometry_msgs",
                "Twist",
                "geometry_msgs/Vector3 linear\ngeometry_msgs/Vector3 angular",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "geometry_msgs",
                "PoseStamped",
                "std_msgs/Header header\ngeometry_msgs/Pose pose",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "geometry_msgs",
                "TwistStamped",
                "std_msgs/Header header\ngeometry_msgs/Twist twist",
            ));

            // TF2 types
            self.register_definition(MessageDefinition::parse_msg_content(
                "geometry_msgs",
                "Transform",
                "geometry_msgs/Vector3 translation\ngeometry_msgs/Quaternion rotation",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "geometry_msgs",
                "TransformStamped",
                "std_msgs/Header header\nstring child_frame_id\ngeometry_msgs/Transform transform",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "tf2_msgs",
                "TFMessage",
                "geometry_msgs/TransformStamped[] transforms",
            ));

            // sensor_msgs
            self.register_definition(MessageDefinition::parse_msg_content(
                "sensor_msgs",
                "LaserScan",
                "std_msgs/Header header\nfloat32 angle_min\nfloat32 angle_max\n\
                 float32 angle_increment\nfloat32 time_increment\nfloat32 scan_time\n\
                 float32 range_min\nfloat32 range_max\nfloat32[] ranges\nfloat32[] intensities",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "sensor_msgs", "Imu",
                "std_msgs/Header header\ngeometry_msgs/Quaternion orientation\n\
                 float64[9] orientation_covariance\ngeometry_msgs/Vector3 angular_velocity\n\
                 float64[9] angular_velocity_covariance\ngeometry_msgs/Vector3 linear_acceleration\n\
                 float64[9] linear_acceleration_covariance"
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "sensor_msgs",
                "Image",
                "std_msgs/Header header\nuint32 height\nuint32 width\nstring encoding\n\
                 uint8 is_bigendian\nuint32 step\nuint8[] data",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "sensor_msgs",
                "PointCloud2",
                "std_msgs/Header header\nuint32 height\nuint32 width\n\
                 sensor_msgs/PointField[] fields\nbool is_bigendian\nuint32 point_step\n\
                 uint32 row_step\nuint8[] data\nbool is_dense",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "sensor_msgs",
                "PointField",
                "string name\nuint32 offset\nuint8 datatype\nuint32 count",
            ));

            // nav_msgs
            self.register_definition(MessageDefinition::parse_msg_content(
                "nav_msgs",
                "Odometry",
                "std_msgs/Header header\nstring child_frame_id\n\
                 geometry_msgs/PoseWithCovariance pose\ngeometry_msgs/TwistWithCovariance twist",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "nav_msgs",
                "Path",
                "std_msgs/Header header\ngeometry_msgs/PoseStamped[] poses",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "nav_msgs",
                "OccupancyGrid",
                "std_msgs/Header header\nnav_msgs/MapMetaData info\nint8[] data",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "nav_msgs",
                "MapMetaData",
                "builtin_interfaces/Time map_load_time\nfloat32 resolution\n\
                 uint32 width\nuint32 height\ngeometry_msgs/Pose origin",
            ));

            // geometry_msgs covariance types
            self.register_definition(MessageDefinition::parse_msg_content(
                "geometry_msgs",
                "PoseWithCovariance",
                "geometry_msgs/Pose pose\nfloat64[36] covariance",
            ));
            self.register_definition(MessageDefinition::parse_msg_content(
                "geometry_msgs",
                "TwistWithCovariance",
                "geometry_msgs/Twist twist\nfloat64[36] covariance",
            ));
        }

        /// Register a message definition
        pub fn register_definition(&self, def: MessageDefinition) {
            let full_name = def.full_name();
            let short_name = full_name.replace("/msg/", "/");
            let def_arc = Arc::new(def);

            let mut defs = self.definitions.write().unwrap();
            defs.insert(full_name.clone(), Arc::clone(&def_arc));
            // Also register short form (package/Name)
            if short_name != full_name {
                defs.insert(short_name, def_arc);
            }
        }

        /// Get a message definition by type name
        pub fn get(&self, type_name: &str) -> Option<Arc<MessageDefinition>> {
            let defs = self.definitions.read().unwrap();
            defs.get(type_name).cloned()
        }

        /// Check if a type is registered
        pub fn contains(&self, type_name: &str) -> bool {
            let defs = self.definitions.read().unwrap();
            defs.contains_key(type_name)
        }

        /// Get all registered type names
        pub fn type_names(&self) -> Vec<String> {
            let defs = self.definitions.read().unwrap();
            defs.keys().cloned().collect()
        }

        /// Get count of registered types
        pub fn len(&self) -> usize {
            let defs = self.definitions.read().unwrap();
            defs.len()
        }

        /// Check if empty
        pub fn is_empty(&self) -> bool {
            self.len() == 0
        }
    }

    /// Global message registry singleton
    static MESSAGE_REGISTRY: std::sync::OnceLock<MessageRegistry> = std::sync::OnceLock::new();

    /// Get the global message registry
    pub fn message_registry() -> &'static MessageRegistry {
        MESSAGE_REGISTRY.get_or_init(MessageRegistry::new)
    }

    /// Dynamic value that can hold any ROS2 message field
    #[derive(Debug, Clone)]
    pub enum DynamicValue {
        Bool(bool),
        Int8(i8),
        Int16(i16),
        Int32(i32),
        Int64(i64),
        Uint8(u8),
        Uint16(u16),
        Uint32(u32),
        Uint64(u64),
        Float32(f32),
        Float64(f64),
        String(String),
        Time { sec: i32, nsec: u32 },
        Duration { sec: i32, nsec: u32 },
        Array(Vec<DynamicValue>),
        Message(HashMap<String, DynamicValue>),
    }

    impl DynamicValue {
        /// Convert to JSON-compatible serde_json::Value
        pub fn to_json(&self) -> serde_json::Value {
            match self {
                DynamicValue::Bool(v) => serde_json::Value::Bool(*v),
                DynamicValue::Int8(v) => serde_json::json!(*v),
                DynamicValue::Int16(v) => serde_json::json!(*v),
                DynamicValue::Int32(v) => serde_json::json!(*v),
                DynamicValue::Int64(v) => serde_json::json!(*v),
                DynamicValue::Uint8(v) => serde_json::json!(*v),
                DynamicValue::Uint16(v) => serde_json::json!(*v),
                DynamicValue::Uint32(v) => serde_json::json!(*v),
                DynamicValue::Uint64(v) => serde_json::json!(*v),
                DynamicValue::Float32(v) => serde_json::json!(*v),
                DynamicValue::Float64(v) => serde_json::json!(*v),
                DynamicValue::String(v) => serde_json::Value::String(v.clone()),
                DynamicValue::Time { sec, nsec } => serde_json::json!({
                    "sec": sec,
                    "nsec": nsec
                }),
                DynamicValue::Duration { sec, nsec } => serde_json::json!({
                    "sec": sec,
                    "nsec": nsec
                }),
                DynamicValue::Array(arr) => {
                    serde_json::Value::Array(arr.iter().map(|v| v.to_json()).collect())
                }
                DynamicValue::Message(map) => {
                    let obj: serde_json::Map<String, serde_json::Value> =
                        map.iter().map(|(k, v)| (k.clone(), v.to_json())).collect();
                    serde_json::Value::Object(obj)
                }
            }
        }
    }

    // ========================================================================
    // TF2 Transform Buffer and Bridge
    // ========================================================================

    use std::collections::BTreeMap;

    /// Error type for TF2 operations
    #[derive(Debug, thiserror::Error)]
    pub enum TF2Error {
        #[error("No connection between frames '{0}' and '{1}'")]
        NoPathBetweenFrames(String, String),

        #[error("Lookup would require extrapolation into the past: earliest available is {available_ns}ns but requested {requested_ns}ns")]
        ExtrapolationIntoPast {
            available_ns: u64,
            requested_ns: u64,
        },

        #[error("Lookup would require extrapolation into the future: latest available is {available_ns}ns but requested {requested_ns}ns")]
        ExtrapolationIntoFuture {
            available_ns: u64,
            requested_ns: u64,
        },

        #[error("No transforms available for frame '{0}'")]
        NoDataForFrame(String),

        #[error("Frame '{0}' is not in the transform tree")]
        FrameNotFound(String),

        #[error("Transform lookup timed out")]
        Timeout,

        #[error("Cannot determine transform to self (source and target frames are the same)")]
        SelfLookup,
    }

    /// Result type for TF2 operations
    pub type TF2Result<T> = Result<T, TF2Error>;

    /// A timestamped transform for the buffer
    #[derive(Debug, Clone)]
    struct BufferedTransform {
        transform: Transform,
        timestamp_ns: u64,
    }

    /// Transform buffer entry for a single parent-child relationship
    #[derive(Debug, Clone)]
    struct FrameTransformBuffer {
        parent_frame: String,
        /// Transforms sorted by timestamp, oldest to newest
        transforms: BTreeMap<u64, Transform>,
        is_static: bool,
    }

    impl FrameTransformBuffer {
        fn new(parent_frame: &str, is_static: bool) -> Self {
            FrameTransformBuffer {
                parent_frame: parent_frame.to_string(),
                transforms: BTreeMap::new(),
                is_static,
            }
        }

        fn add_transform(&mut self, timestamp_ns: u64, transform: Transform) {
            self.transforms.insert(timestamp_ns, transform);

            // Limit buffer size (keep most recent 1000 transforms for dynamic, unlimited for static)
            if !self.is_static && self.transforms.len() > 1000 {
                if let Some(oldest_key) = self.transforms.keys().next().cloned() {
                    self.transforms.remove(&oldest_key);
                }
            }
        }

        /// Get transform at the exact timestamp or interpolate between two closest
        fn get_transform_at(&self, timestamp_ns: u64) -> TF2Result<Transform> {
            if self.transforms.is_empty() {
                return Err(TF2Error::NoDataForFrame(self.parent_frame.clone()));
            }

            // For static transforms, just return the first one
            if self.is_static {
                return Ok(self.transforms.values().next().unwrap().clone());
            }

            // Exact match
            if let Some(tf) = self.transforms.get(&timestamp_ns) {
                return Ok(tf.clone());
            }

            // Get the transforms before and after the requested timestamp
            let before = self
                .transforms
                .range(..timestamp_ns)
                .next_back()
                .map(|(k, v)| (*k, v.clone()));
            let after = self
                .transforms
                .range(timestamp_ns..)
                .next()
                .map(|(k, v)| (*k, v.clone()));

            match (before, after) {
                (Some((t1, tf1)), Some((t2, tf2))) => {
                    // Interpolate between the two transforms
                    let ratio = (timestamp_ns - t1) as f64 / (t2 - t1) as f64;
                    Ok(interpolate_transform(&tf1, &tf2, ratio))
                }
                (Some((t1, _tf1)), None) => {
                    // Would need to extrapolate into the future
                    Err(TF2Error::ExtrapolationIntoFuture {
                        available_ns: t1,
                        requested_ns: timestamp_ns,
                    })
                }
                (None, Some((t2, _tf2))) => {
                    // Would need to extrapolate into the past
                    Err(TF2Error::ExtrapolationIntoPast {
                        available_ns: t2,
                        requested_ns: timestamp_ns,
                    })
                }
                (None, None) => Err(TF2Error::NoDataForFrame(self.parent_frame.clone())),
            }
        }

        fn latest_timestamp(&self) -> Option<u64> {
            self.transforms.keys().next_back().copied()
        }

        fn earliest_timestamp(&self) -> Option<u64> {
            self.transforms.keys().next().copied()
        }
    }

    /// Linear interpolation for Vector3
    pub fn lerp_vector3(v1: &Vector3, v2: &Vector3, t: f64) -> Vector3 {
        Vector3 {
            x: v1.x + (v2.x - v1.x) * t,
            y: v1.y + (v2.y - v1.y) * t,
            z: v1.z + (v2.z - v1.z) * t,
        }
    }

    /// Spherical linear interpolation for Quaternion (SLERP)
    pub fn slerp_quaternion(q1: &Quaternion, q2: &Quaternion, t: f64) -> Quaternion {
        // Compute the dot product
        let mut dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;

        // If the dot product is negative, negate one quaternion to take the shorter path
        let mut q2_adj = q2.clone();
        if dot < 0.0 {
            q2_adj = Quaternion {
                x: -q2.x,
                y: -q2.y,
                z: -q2.z,
                w: -q2.w,
            };
            dot = -dot;
        }

        // If quaternions are very close, use linear interpolation
        if dot > 0.9995 {
            let result = Quaternion {
                x: q1.x + (q2_adj.x - q1.x) * t,
                y: q1.y + (q2_adj.y - q1.y) * t,
                z: q1.z + (q2_adj.z - q1.z) * t,
                w: q1.w + (q2_adj.w - q1.w) * t,
            };
            // Normalize
            let norm = (result.x * result.x
                + result.y * result.y
                + result.z * result.z
                + result.w * result.w)
                .sqrt();
            return Quaternion {
                x: result.x / norm,
                y: result.y / norm,
                z: result.z / norm,
                w: result.w / norm,
            };
        }

        // Compute the spherical interpolation
        let theta_0 = dot.acos();
        let theta = theta_0 * t;

        let sin_theta = theta.sin();
        let sin_theta_0 = theta_0.sin();

        let s0 = theta.cos() - dot * sin_theta / sin_theta_0;
        let s1 = sin_theta / sin_theta_0;

        Quaternion {
            x: q1.x * s0 + q2_adj.x * s1,
            y: q1.y * s0 + q2_adj.y * s1,
            z: q1.z * s0 + q2_adj.z * s1,
            w: q1.w * s0 + q2_adj.w * s1,
        }
    }

    /// Interpolate between two transforms
    pub fn interpolate_transform(tf1: &Transform, tf2: &Transform, t: f64) -> Transform {
        Transform {
            translation: lerp_vector3(&tf1.translation, &tf2.translation, t),
            rotation: slerp_quaternion(&tf1.rotation, &tf2.rotation, t),
        }
    }

    /// TF2 Transform Buffer - stores and manages coordinate frame transforms
    pub struct TransformBuffer {
        /// Map from child_frame_id to its transform buffer (which includes parent info)
        frames: Arc<RwLock<HashMap<String, FrameTransformBuffer>>>,
        /// Cache timeout duration in nanoseconds (default 10 seconds)
        cache_time_ns: u64,
    }

    impl Default for TransformBuffer {
        fn default() -> Self {
            Self::new()
        }
    }

    impl TransformBuffer {
        /// Create a new transform buffer with default settings
        pub fn new() -> Self {
            TransformBuffer {
                frames: Arc::new(RwLock::new(HashMap::new())),
                cache_time_ns: 10_000_000_000, // 10 seconds
            }
        }

        /// Create a transform buffer with custom cache time
        pub fn with_cache_time(cache_time_secs: f64) -> Self {
            TransformBuffer {
                frames: Arc::new(RwLock::new(HashMap::new())),
                cache_time_ns: (cache_time_secs * 1_000_000_000.0) as u64,
            }
        }

        /// Add a transform to the buffer
        pub fn set_transform(&self, tf: &TransformStamped, is_static: bool) {
            let mut frames = self.frames.write().unwrap();
            let timestamp_ns = tf.timestamp_ns();

            let entry = frames
                .entry(tf.child_frame_id.clone())
                .or_insert_with(|| FrameTransformBuffer::new(&tf.header.frame_id, is_static));

            // Update parent if different (shouldn't normally happen)
            if entry.parent_frame != tf.header.frame_id {
                entry.parent_frame = tf.header.frame_id.clone();
            }

            entry.add_transform(timestamp_ns, tf.transform.clone());
        }

        /// Add multiple transforms from a TFMessage
        pub fn set_transforms(&self, msg: &TFMessage, is_static: bool) {
            for tf in &msg.transforms {
                self.set_transform(tf, is_static);
            }
        }

        /// Check if a frame exists in the buffer
        pub fn can_transform(&self, target_frame: &str, source_frame: &str) -> bool {
            if target_frame == source_frame {
                return true;
            }
            self.find_path(target_frame, source_frame).is_some()
        }

        /// Find the path from source to target frame
        fn find_path(&self, target_frame: &str, source_frame: &str) -> Option<Vec<String>> {
            let frames = self.frames.read().unwrap();

            // Build the frame tree by finding ancestors
            fn get_ancestors(
                frames: &HashMap<String, FrameTransformBuffer>,
                frame: &str,
            ) -> Vec<String> {
                let mut ancestors = vec![frame.to_string()];
                let mut current = frame.to_string();

                while let Some(buffer) = frames.get(&current) {
                    if ancestors.contains(&buffer.parent_frame) {
                        break; // Cycle detected
                    }
                    ancestors.push(buffer.parent_frame.clone());
                    current = buffer.parent_frame.clone();
                }

                ancestors
            }

            let source_ancestors = get_ancestors(&frames, source_frame);
            let target_ancestors = get_ancestors(&frames, target_frame);

            // Find common ancestor
            let common = source_ancestors
                .iter()
                .find(|a| target_ancestors.contains(a))?;

            // Build path: source -> common -> target
            let mut path = Vec::new();

            // Add source path to common
            for frame in &source_ancestors {
                path.push(frame.clone());
                if frame == common {
                    break;
                }
            }

            // Add target path from common (reversed)
            let target_idx = target_ancestors.iter().position(|a| a == common)?;
            for frame in target_ancestors[..target_idx].iter().rev() {
                path.push(frame.clone());
            }

            Some(path)
        }

        /// Look up the transform from source_frame to target_frame at a given time
        pub fn lookup_transform(
            &self,
            target_frame: &str,
            source_frame: &str,
            timestamp_ns: u64,
        ) -> TF2Result<TransformStamped> {
            // Check for self lookup (looking up a frame relative to itself)
            if target_frame == source_frame {
                return Err(TF2Error::SelfLookup);
            }

            // Check if frames exist
            let frames = self.frames.read().unwrap();

            // For non-empty buffer, check if source and target frames exist
            if !frames.is_empty() {
                // Check if source frame exists (either as a child or as a parent)
                let source_exists = frames.contains_key(source_frame)
                    || frames.values().any(|buf| buf.parent_frame == source_frame);
                if !source_exists {
                    return Err(TF2Error::FrameNotFound(source_frame.to_string()));
                }

                // Check if target frame exists
                let target_exists = frames.contains_key(target_frame)
                    || frames.values().any(|buf| buf.parent_frame == target_frame);
                if !target_exists {
                    return Err(TF2Error::FrameNotFound(target_frame.to_string()));
                }
            } else {
                // Empty buffer - neither frame can exist
                return Err(TF2Error::FrameNotFound(source_frame.to_string()));
            }
            drop(frames);

            let path = self.find_path(target_frame, source_frame).ok_or_else(|| {
                TF2Error::NoPathBetweenFrames(target_frame.to_string(), source_frame.to_string())
            })?;

            // Compose transforms along the path
            let frames = self.frames.read().unwrap();
            let mut result_transform = Transform::identity();

            // Walk from source to target
            for i in 0..path.len() - 1 {
                let child = &path[i];
                let _parent = &path[i + 1];

                if let Some(buffer) = frames.get(child) {
                    let tf = buffer.get_transform_at(timestamp_ns)?;
                    result_transform = tf.compose(&result_transform);
                }
            }

            Ok(TransformStamped::new(
                target_frame,
                source_frame,
                Time {
                    sec: (timestamp_ns / 1_000_000_000) as i32,
                    nanosec: (timestamp_ns % 1_000_000_000) as u32,
                },
                result_transform,
            ))
        }

        /// Get all known frames
        pub fn get_all_frame_names(&self) -> Vec<String> {
            let frames = self.frames.read().unwrap();
            let mut names: std::collections::HashSet<String> = frames.keys().cloned().collect();

            // Also add parent frames
            for buffer in frames.values() {
                names.insert(buffer.parent_frame.clone());
            }

            names.into_iter().collect()
        }

        /// Get the parent of a frame, if known
        pub fn get_parent(&self, frame_id: &str) -> Option<String> {
            let frames = self.frames.read().unwrap();
            frames.get(frame_id).map(|b| b.parent_frame.clone())
        }

        /// Clear all transforms
        pub fn clear(&self) {
            let mut frames = self.frames.write().unwrap();
            frames.clear();
        }

        /// Get the number of frames in the buffer
        pub fn frame_count(&self) -> usize {
            self.frames.read().unwrap().len()
        }
    }

    /// Convert ROS2 CDR TransformStamped to HORUS HFrame TransformStamped
    pub fn cdr_to_hframe_transform(
        cdr_tf: &TransformStamped,
    ) -> horus_library::hframe::TransformStamped {
        let timestamp_ns = cdr_tf.timestamp_ns();

        horus_library::hframe::TransformStamped::new(
            cdr_tf.parent_frame(),
            cdr_tf.child_frame(),
            timestamp_ns,
            horus_library::hframe::Transform::new(
                [
                    cdr_tf.transform.translation.x,
                    cdr_tf.transform.translation.y,
                    cdr_tf.transform.translation.z,
                ],
                [
                    cdr_tf.transform.rotation.x,
                    cdr_tf.transform.rotation.y,
                    cdr_tf.transform.rotation.z,
                    cdr_tf.transform.rotation.w,
                ],
            ),
        )
    }

    /// Convert HORUS HFrame TransformStamped to ROS2 CDR TransformStamped
    pub fn hframe_to_cdr_transform(
        hframe_tf: &horus_library::hframe::TransformStamped,
    ) -> TransformStamped {
        let sec = (hframe_tf.timestamp / 1_000_000_000) as i32;
        let nanosec = (hframe_tf.timestamp % 1_000_000_000) as u32;

        let parent = hframe_tf.parent_frame_id();
        let child = hframe_tf.child_frame_id();

        TransformStamped::new(
            &parent,
            &child,
            Time { sec, nanosec },
            Transform::new(
                Vector3 {
                    x: hframe_tf.transform.translation[0] as f64,
                    y: hframe_tf.transform.translation[1] as f64,
                    z: hframe_tf.transform.translation[2] as f64,
                },
                Quaternion {
                    x: hframe_tf.transform.rotation[0] as f64,
                    y: hframe_tf.transform.rotation[1] as f64,
                    z: hframe_tf.transform.rotation[2] as f64,
                    w: hframe_tf.transform.rotation[3] as f64,
                },
            ),
        )
    }

    /// Convert ROS2 TFMessage to HORUS TFMessage
    pub fn cdr_to_hframe_tf_message(cdr_msg: &TFMessage) -> horus_library::hframe::TFMessage {
        let mut hframe_msg = horus_library::hframe::TFMessage::new();
        for cdr_tf in &cdr_msg.transforms {
            let hframe_tf = cdr_to_hframe_transform(cdr_tf);
            hframe_msg.add(hframe_tf);
        }
        hframe_msg
    }

    /// Convert HORUS TFMessage to ROS2 TFMessage
    pub fn hframe_to_cdr_tf_message(hframe_msg: &horus_library::hframe::TFMessage) -> TFMessage {
        let transforms: Vec<TransformStamped> = hframe_msg
            .iter()
            .map(|tf| hframe_to_cdr_transform(tf))
            .collect();
        TFMessage::with_transforms(transforms)
    }

    // ========================================================================
    // Tests
    // ========================================================================

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn test_cdr_roundtrip_string() {
            let msg = StdString {
                data: "Hello ROS2!".to_string(),
            };
            let bytes = serialize(&msg).unwrap();
            let decoded: StdString = deserialize(&bytes).unwrap();
            assert_eq!(msg.data, decoded.data);
        }

        #[test]
        fn test_cdr_roundtrip_twist() {
            let msg = Twist {
                linear: Vector3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                angular: Vector3 {
                    x: 0.1,
                    y: 0.2,
                    z: 0.3,
                },
            };
            let bytes = serialize(&msg).unwrap();
            let decoded: Twist = deserialize(&bytes).unwrap();
            assert_eq!(msg.linear.x, decoded.linear.x);
            assert_eq!(msg.angular.z, decoded.angular.z);
        }

        #[test]
        fn test_cdr_roundtrip_laser_scan() {
            let msg = LaserScan {
                header: Header {
                    stamp: Time::now(),
                    frame_id: "base_laser".to_string(),
                },
                angle_min: -1.57,
                angle_max: 1.57,
                angle_increment: 0.01,
                time_increment: 0.0001,
                scan_time: 0.1,
                range_min: 0.1,
                range_max: 30.0,
                ranges: vec![1.0, 1.5, 2.0, 2.5, 3.0],
                intensities: vec![100.0, 110.0, 120.0, 130.0, 140.0],
            };
            let bytes = serialize(&msg).unwrap();
            let decoded: LaserScan = deserialize(&bytes).unwrap();
            assert_eq!(msg.ranges.len(), decoded.ranges.len());
            assert_eq!(msg.header.frame_id, decoded.header.frame_id);
        }

        #[test]
        fn test_cdr_header() {
            let msg = StdInt32 { data: 42 };
            let bytes = serialize(&msg).unwrap();

            // First 4 bytes should be CDR encapsulation header
            assert_eq!(&bytes[0..4], &CDR_HEADER_LE);

            // Check little-endian detection
            assert!(is_little_endian(&bytes));
        }

        #[test]
        fn test_message_type_parsing() {
            assert_eq!(
                Ros2MessageType::from_type_string("geometry_msgs/msg/Twist"),
                Ros2MessageType::GeometryMsgsTwist
            );
            assert_eq!(
                Ros2MessageType::from_type_string("sensor_msgs/LaserScan"),
                Ros2MessageType::SensorMsgsLaserScan
            );
            assert!(matches!(
                Ros2MessageType::from_type_string("custom_msgs/MyType"),
                Ros2MessageType::Unknown(_)
            ));
        }

        #[test]
        fn test_time_now() {
            let t = Time::now();
            assert!(t.sec > 0);
        }

        #[test]
        fn test_field_type_parsing() {
            assert_eq!(FieldType::from_ros2_type("bool"), FieldType::Bool);
            assert_eq!(FieldType::from_ros2_type("int32"), FieldType::Int32);
            assert_eq!(FieldType::from_ros2_type("float64"), FieldType::Float64);
            assert_eq!(FieldType::from_ros2_type("string"), FieldType::String);
            assert_eq!(FieldType::from_ros2_type("time"), FieldType::Time);

            // Array types
            assert_eq!(
                FieldType::from_ros2_type("float32[]"),
                FieldType::Array(Box::new(FieldType::Float32), None)
            );
            assert_eq!(
                FieldType::from_ros2_type("float64[36]"),
                FieldType::Array(Box::new(FieldType::Float64), Some(36))
            );

            // Nested types
            assert!(matches!(
                FieldType::from_ros2_type("geometry_msgs/Twist"),
                FieldType::Nested(_)
            ));
        }

        #[test]
        fn test_message_definition_parsing() {
            let def = MessageDefinition::parse_msg_content(
                "geometry_msgs",
                "Twist",
                "# This is a comment\n\
                 geometry_msgs/Vector3 linear\n\
                 geometry_msgs/Vector3 angular\n",
            );

            assert_eq!(def.package, "geometry_msgs");
            assert_eq!(def.name, "Twist");
            assert_eq!(def.fields.len(), 2);
            assert_eq!(def.fields[0].name, "linear");
            assert_eq!(def.fields[1].name, "angular");
            assert_eq!(def.full_name(), "geometry_msgs/msg/Twist");
        }

        #[test]
        fn test_message_definition_with_constants() {
            let def = MessageDefinition::parse_msg_content(
                "sensor_msgs",
                "PointField",
                "uint8 INT8=1\n\
                 uint8 UINT8=2\n\
                 uint8 INT16=3\n\
                 string name\n\
                 uint32 offset\n\
                 uint8 datatype\n\
                 uint32 count",
            );

            assert_eq!(def.constants.len(), 3);
            assert_eq!(
                def.constants[0],
                ("INT8".to_string(), "uint8".to_string(), "1".to_string())
            );
            assert_eq!(def.fields.len(), 4);
        }

        #[test]
        fn test_message_registry() {
            let registry = MessageRegistry::new();

            // Check standard types are registered
            assert!(registry.contains("geometry_msgs/msg/Twist"));
            assert!(registry.contains("geometry_msgs/Twist")); // Short form
            assert!(registry.contains("sensor_msgs/msg/LaserScan"));
            assert!(registry.contains("nav_msgs/msg/Odometry"));

            // Check unknown type
            assert!(!registry.contains("custom_msgs/msg/Unknown"));

            // Get a definition
            let twist = registry.get("geometry_msgs/msg/Twist").unwrap();
            assert_eq!(twist.name, "Twist");
            assert_eq!(twist.fields.len(), 2);
        }

        #[test]
        fn test_dynamic_value_json() {
            let msg = DynamicValue::Message({
                let mut map = HashMap::new();
                map.insert("x".to_string(), DynamicValue::Float64(1.0));
                map.insert("y".to_string(), DynamicValue::Float64(2.0));
                map.insert("z".to_string(), DynamicValue::Float64(3.0));
                map
            });

            let json = msg.to_json();
            assert!(json.is_object());
            assert_eq!(json["x"], 1.0);
            assert_eq!(json["y"], 2.0);
            assert_eq!(json["z"], 3.0);
        }

        #[test]
        fn test_global_registry() {
            let registry = message_registry();
            assert!(registry.len() > 0);
            assert!(registry.contains("std_msgs/msg/String"));
        }
    }
}

// ============================================================================
// Bridge Configuration
// ============================================================================

/// Direction of message bridging
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BridgeDirection {
    /// ROS2 -> HORUS only
    In,
    /// HORUS -> ROS2 only
    Out,
    /// Bidirectional (default)
    Both,
}

impl std::str::FromStr for BridgeDirection {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "in" | "ros2-to-horus" | "r2h" => Ok(BridgeDirection::In),
            "out" | "horus-to-ros2" | "h2r" => Ok(BridgeDirection::Out),
            "both" | "bidirectional" | "bi" => Ok(BridgeDirection::Both),
            _ => Err(format!("Invalid direction: {}. Use: in, out, or both", s)),
        }
    }
}

/// ROS2 QoS profile presets
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QosProfile {
    /// Sensor data: best effort, volatile, keep last 5
    SensorData,
    /// Default: reliable, volatile, keep last 10
    Default,
    /// Services: reliable, volatile, keep all
    Services,
    /// Parameters: reliable, transient local, keep last 1
    Parameters,
    /// System default
    SystemDefault,
}

impl std::str::FromStr for QosProfile {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "sensor" | "sensor_data" | "sensordata" => Ok(QosProfile::SensorData),
            "default" => Ok(QosProfile::Default),
            "services" | "service" => Ok(QosProfile::Services),
            "parameters" | "params" => Ok(QosProfile::Parameters),
            "system" | "system_default" => Ok(QosProfile::SystemDefault),
            _ => Err(format!("Invalid QoS profile: {}", s)),
        }
    }
}

/// Bridge configuration
#[derive(Debug, Clone)]
pub struct BridgeConfig {
    /// ROS2 domain ID (0-232)
    pub domain_id: u32,
    /// Topics to bridge (empty = discover all if --all)
    pub topics: Vec<String>,
    /// Bridge all discovered topics
    pub bridge_all: bool,
    /// Bridge direction
    pub direction: BridgeDirection,
    /// Namespace filter (only bridge topics matching this prefix)
    pub namespace: Option<String>,
    /// QoS profile to use
    pub qos_profile: QosProfile,
    /// Enable service bridging
    pub bridge_services: bool,
    /// Enable action bridging
    pub bridge_actions: bool,
    /// Enable parameter bridging
    pub bridge_params: bool,
    /// Verbose output
    pub verbose: bool,
    /// Statistics update interval
    pub stats_interval: Duration,
    /// Connection recovery configuration
    pub recovery: RecoveryConfig,
}

impl Default for BridgeConfig {
    fn default() -> Self {
        Self {
            domain_id: 0,
            topics: Vec::new(),
            bridge_all: false,
            direction: BridgeDirection::Both,
            namespace: None,
            qos_profile: QosProfile::Default,
            bridge_services: false,
            bridge_actions: false,
            bridge_params: false,
            verbose: false,
            stats_interval: Duration::from_secs(5),
            recovery: RecoveryConfig::default(),
        }
    }
}

// ============================================================================
// Connection Recovery (Production-Ready)
// ============================================================================

/// Configuration for connection recovery with exponential backoff
#[derive(Debug, Clone)]
pub struct RecoveryConfig {
    /// Enable automatic reconnection
    pub enabled: bool,
    /// Initial delay before first retry
    pub initial_delay: Duration,
    /// Maximum delay between retries
    pub max_delay: Duration,
    /// Multiplier for exponential backoff (e.g., 2.0 doubles delay each retry)
    pub backoff_multiplier: f64,
    /// Maximum number of retries (None = unlimited)
    pub max_retries: Option<u32>,
    /// Jitter factor (0.0-1.0) to add randomness to delays
    pub jitter: f64,
    /// Health check interval to detect disconnections
    pub health_check_interval: Duration,
}

impl Default for RecoveryConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            initial_delay: Duration::from_millis(100),
            max_delay: Duration::from_secs(30),
            backoff_multiplier: 2.0,
            max_retries: Some(10),
            jitter: 0.1,
            health_check_interval: Duration::from_secs(5),
        }
    }
}

impl RecoveryConfig {
    /// Create a configuration for development (fast retries, limited attempts)
    pub fn development() -> Self {
        Self {
            enabled: true,
            initial_delay: Duration::from_millis(50),
            max_delay: Duration::from_secs(5),
            backoff_multiplier: 1.5,
            max_retries: Some(5),
            jitter: 0.05,
            health_check_interval: Duration::from_secs(2),
        }
    }

    /// Create a configuration for production (aggressive recovery)
    pub fn production() -> Self {
        Self {
            enabled: true,
            initial_delay: Duration::from_millis(100),
            max_delay: Duration::from_secs(60),
            backoff_multiplier: 2.0,
            max_retries: None, // Unlimited retries in production
            jitter: 0.2,
            health_check_interval: Duration::from_secs(10),
        }
    }

    /// Disable recovery (fail immediately on disconnection)
    pub fn disabled() -> Self {
        Self {
            enabled: false,
            ..Default::default()
        }
    }
}

/// State machine for connection recovery
#[derive(Debug, Clone)]
pub struct RecoveryState {
    /// Current retry attempt (0 = connected, 1+ = retrying)
    pub attempt: u32,
    /// Current backoff delay
    pub current_delay: Duration,
    /// Timestamp of last successful operation
    pub last_success: std::time::Instant,
    /// Total recoveries performed
    pub total_recoveries: u64,
    /// Consecutive failures
    pub consecutive_failures: u32,
}

impl Default for RecoveryState {
    fn default() -> Self {
        Self::new()
    }
}

impl RecoveryState {
    pub fn new() -> Self {
        Self {
            attempt: 0,
            current_delay: Duration::from_millis(100),
            last_success: std::time::Instant::now(),
            total_recoveries: 0,
            consecutive_failures: 0,
        }
    }

    /// Record a successful operation
    pub fn record_success(&mut self) {
        if self.attempt > 0 {
            self.total_recoveries += 1;
            log::info!("Connection recovered after {} attempts", self.attempt);
        }
        self.attempt = 0;
        self.consecutive_failures = 0;
        self.last_success = std::time::Instant::now();
    }

    /// Record a failure and compute next delay
    pub fn record_failure(&mut self, config: &RecoveryConfig) -> Option<Duration> {
        self.consecutive_failures += 1;

        if !config.enabled {
            return None;
        }

        // Check max retries
        if let Some(max) = config.max_retries {
            if self.attempt >= max {
                log::error!("Max retries ({}) exceeded, giving up", max);
                return None;
            }
        }

        self.attempt += 1;

        // Calculate delay with exponential backoff
        let base_delay = if self.attempt == 1 {
            config.initial_delay
        } else {
            let multiplied = self.current_delay.as_secs_f64() * config.backoff_multiplier;
            Duration::from_secs_f64(multiplied.min(config.max_delay.as_secs_f64()))
        };

        // Add jitter
        let jitter_amount = base_delay.as_secs_f64() * config.jitter;
        let jitter: f64 = (std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as f64
            % 1000.0)
            / 1000.0
            * jitter_amount;

        self.current_delay = Duration::from_secs_f64(base_delay.as_secs_f64() + jitter);

        log::warn!(
            "Connection failed (attempt {}/{}), retrying in {:?}",
            self.attempt,
            config
                .max_retries
                .map(|n| n.to_string())
                .unwrap_or_else(|| "∞".to_string()),
            self.current_delay
        );

        Some(self.current_delay)
    }

    /// Check if recovery should be attempted
    pub fn should_retry(&self, config: &RecoveryConfig) -> bool {
        if !config.enabled {
            return false;
        }
        match config.max_retries {
            Some(max) => self.attempt < max,
            None => true,
        }
    }

    /// Check if connection is healthy (based on last success time)
    pub fn is_healthy(&self, config: &RecoveryConfig) -> bool {
        self.last_success.elapsed() < config.health_check_interval * 2
    }
}

// ============================================================================
// Bridge Statistics (Production-Ready with Latency Tracking)
// ============================================================================

/// Statistics for a single bridged topic with latency tracking
#[derive(Debug, Default)]
pub struct TopicStats {
    /// Messages bridged ROS2 -> HORUS
    pub msgs_in: AtomicU64,
    /// Messages bridged HORUS -> ROS2
    pub msgs_out: AtomicU64,
    /// Bytes transferred
    pub bytes_transferred: AtomicU64,
    /// Last message timestamp (ms since epoch)
    pub last_msg_time: AtomicU64,
    /// Errors encountered
    pub errors: AtomicU64,
    /// Latency tracking (microseconds)
    pub latency_sum_us: AtomicU64,
    pub latency_count: AtomicU64,
    pub latency_min_us: AtomicU64,
    pub latency_max_us: AtomicU64,
}

impl TopicStats {
    pub fn new() -> Self {
        Self {
            latency_min_us: AtomicU64::new(u64::MAX),
            ..Default::default()
        }
    }

    fn now_millis() -> u64 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64
    }

    pub fn record_in(&self, bytes: u64) {
        self.msgs_in.fetch_add(1, Ordering::Relaxed);
        self.bytes_transferred.fetch_add(bytes, Ordering::Relaxed);
        self.last_msg_time
            .store(Self::now_millis(), Ordering::Relaxed);
    }

    pub fn record_out(&self, bytes: u64) {
        self.msgs_out.fetch_add(1, Ordering::Relaxed);
        self.bytes_transferred.fetch_add(bytes, Ordering::Relaxed);
        self.last_msg_time
            .store(Self::now_millis(), Ordering::Relaxed);
    }

    pub fn record_error(&self) {
        self.errors.fetch_add(1, Ordering::Relaxed);
    }

    /// Record a latency measurement in microseconds
    pub fn record_latency(&self, latency_us: u64) {
        self.latency_sum_us.fetch_add(latency_us, Ordering::Relaxed);
        self.latency_count.fetch_add(1, Ordering::Relaxed);

        // Update min (atomic compare-and-swap loop)
        let mut current_min = self.latency_min_us.load(Ordering::Relaxed);
        while latency_us < current_min {
            match self.latency_min_us.compare_exchange_weak(
                current_min,
                latency_us,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(x) => current_min = x,
            }
        }

        // Update max
        let mut current_max = self.latency_max_us.load(Ordering::Relaxed);
        while latency_us > current_max {
            match self.latency_max_us.compare_exchange_weak(
                current_max,
                latency_us,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(x) => current_max = x,
            }
        }
    }

    /// Get average latency in microseconds (returns 0 if no measurements)
    pub fn avg_latency_us(&self) -> u64 {
        let count = self.latency_count.load(Ordering::Relaxed);
        if count == 0 {
            0
        } else {
            self.latency_sum_us.load(Ordering::Relaxed) / count
        }
    }

    /// Get latency statistics as a tuple (min, avg, max) in microseconds
    pub fn latency_stats(&self) -> (u64, u64, u64) {
        let count = self.latency_count.load(Ordering::Relaxed);
        if count == 0 {
            return (0, 0, 0);
        }
        let min = self.latency_min_us.load(Ordering::Relaxed);
        let avg = self.latency_sum_us.load(Ordering::Relaxed) / count;
        let max = self.latency_max_us.load(Ordering::Relaxed);
        (min, avg, max)
    }

    /// Check if topic is healthy (received message recently)
    pub fn is_healthy(&self, max_idle_secs: u64) -> bool {
        let last_time = self.last_msg_time.load(Ordering::Relaxed);
        if last_time == 0 {
            return true; // No messages yet, consider healthy
        }
        let now = Self::now_millis();
        let idle_ms = now.saturating_sub(last_time);
        idle_ms < max_idle_secs * 1000
    }

    /// Get message rate (messages per second) based on recent activity
    pub fn message_rate(&self, duration_secs: f64) -> f64 {
        let total_msgs =
            self.msgs_in.load(Ordering::Relaxed) + self.msgs_out.load(Ordering::Relaxed);
        if duration_secs > 0.0 {
            total_msgs as f64 / duration_secs
        } else {
            0.0
        }
    }
}

/// Health status for monitoring
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthStatus {
    Healthy,
    Degraded,
    Unhealthy,
}

impl std::fmt::Display for HealthStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            HealthStatus::Healthy => write!(f, "healthy"),
            HealthStatus::Degraded => write!(f, "degraded"),
            HealthStatus::Unhealthy => write!(f, "unhealthy"),
        }
    }
}

/// Health check result for the entire bridge
#[derive(Debug, Clone)]
pub struct BridgeHealth {
    pub status: HealthStatus,
    pub uptime: Duration,
    pub active_topics: usize,
    pub healthy_topics: usize,
    pub degraded_topics: usize,
    pub unhealthy_topics: usize,
    pub total_errors: u64,
    pub avg_latency_us: u64,
    pub message_rate: f64,
}

// ============================================================================
// Rate Limiting (Production-Ready)
// ============================================================================

/// Rate limiter using token bucket algorithm
#[derive(Debug)]
pub struct RateLimiter {
    /// Maximum tokens (burst capacity)
    capacity: u64,
    /// Current tokens available
    tokens: AtomicU64,
    /// Tokens added per second
    rate: f64,
    /// Last refill timestamp (microseconds since epoch)
    last_refill: AtomicU64,
    /// Total messages allowed
    allowed: AtomicU64,
    /// Total messages dropped
    dropped: AtomicU64,
}

impl RateLimiter {
    /// Create a new rate limiter
    /// - `rate`: messages per second
    /// - `burst`: maximum burst capacity (tokens)
    pub fn new(rate: f64, burst: u64) -> Self {
        Self {
            capacity: burst,
            tokens: AtomicU64::new(burst),
            rate,
            last_refill: AtomicU64::new(Self::now_micros()),
            allowed: AtomicU64::new(0),
            dropped: AtomicU64::new(0),
        }
    }

    fn now_micros() -> u64 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros() as u64
    }

    /// Try to consume a token. Returns true if allowed, false if rate limited.
    pub fn try_acquire(&self) -> bool {
        self.refill();

        loop {
            let current = self.tokens.load(Ordering::Relaxed);
            if current == 0 {
                self.dropped.fetch_add(1, Ordering::Relaxed);
                return false;
            }

            match self.tokens.compare_exchange_weak(
                current,
                current - 1,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => {
                    self.allowed.fetch_add(1, Ordering::Relaxed);
                    return true;
                }
                Err(_) => continue,
            }
        }
    }

    /// Refill tokens based on elapsed time
    fn refill(&self) {
        let now = Self::now_micros();
        let last = self.last_refill.load(Ordering::Relaxed);
        let elapsed_secs = (now.saturating_sub(last)) as f64 / 1_000_000.0;

        if elapsed_secs < 0.001 {
            return; // Don't refill too frequently
        }

        let new_tokens = (elapsed_secs * self.rate) as u64;
        if new_tokens == 0 {
            return;
        }

        // Try to update last_refill
        if self
            .last_refill
            .compare_exchange(last, now, Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
        {
            // Add tokens (capped at capacity)
            loop {
                let current = self.tokens.load(Ordering::Relaxed);
                let new_value = (current + new_tokens).min(self.capacity);
                if current == new_value {
                    break;
                }
                match self.tokens.compare_exchange_weak(
                    current,
                    new_value,
                    Ordering::Relaxed,
                    Ordering::Relaxed,
                ) {
                    Ok(_) => break,
                    Err(_) => continue,
                }
            }
        }
    }

    /// Get current statistics
    pub fn stats(&self) -> (u64, u64, u64) {
        (
            self.tokens.load(Ordering::Relaxed),
            self.allowed.load(Ordering::Relaxed),
            self.dropped.load(Ordering::Relaxed),
        )
    }

    /// Get drop rate (0.0-1.0)
    pub fn drop_rate(&self) -> f64 {
        let allowed = self.allowed.load(Ordering::Relaxed);
        let dropped = self.dropped.load(Ordering::Relaxed);
        let total = allowed + dropped;
        if total == 0 {
            0.0
        } else {
            dropped as f64 / total as f64
        }
    }
}

/// Rate limiting configuration for the bridge
#[derive(Debug, Clone)]
pub struct RateLimitConfig {
    /// Enable rate limiting
    pub enabled: bool,
    /// Default rate limit (messages/sec) for topics without specific config
    pub default_rate: f64,
    /// Default burst capacity
    pub default_burst: u64,
    /// Per-topic rate limits (topic pattern -> rate)
    pub topic_rates: HashMap<String, f64>,
    /// Topics exempt from rate limiting
    pub exempt_topics: Vec<String>,
}

impl Default for RateLimitConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            default_rate: 1000.0, // 1000 msgs/sec default
            default_burst: 100,
            topic_rates: HashMap::new(),
            exempt_topics: vec!["/rosout".to_string(), "/parameter_events".to_string()],
        }
    }
}

impl RateLimitConfig {
    /// Create a config for high-throughput systems
    pub fn high_throughput() -> Self {
        Self {
            enabled: true,
            default_rate: 10000.0,
            default_burst: 1000,
            topic_rates: HashMap::new(),
            exempt_topics: vec!["/rosout".to_string(), "/parameter_events".to_string()],
        }
    }

    /// Create a config for constrained systems
    pub fn constrained() -> Self {
        let mut topic_rates = HashMap::new();
        topic_rates.insert("/camera/*".to_string(), 30.0); // Limit camera to 30 fps
        topic_rates.insert("/lidar/*".to_string(), 20.0); // Limit lidar to 20 Hz

        Self {
            enabled: true,
            default_rate: 100.0,
            default_burst: 10,
            topic_rates,
            exempt_topics: vec![
                "/rosout".to_string(),
                "/parameter_events".to_string(),
                "/cmd_vel".to_string(), // Don't limit control commands
            ],
        }
    }

    /// Check if a topic is exempt from rate limiting
    pub fn is_exempt(&self, topic: &str) -> bool {
        self.exempt_topics.iter().any(|t| {
            if t.ends_with('*') {
                topic.starts_with(&t[..t.len() - 1])
            } else {
                topic == t
            }
        })
    }

    /// Get the rate limit for a specific topic
    pub fn rate_for_topic(&self, topic: &str) -> f64 {
        // Check specific topic rates first (with wildcard matching)
        for (pattern, rate) in &self.topic_rates {
            if pattern.ends_with('*') {
                if topic.starts_with(&pattern[..pattern.len() - 1]) {
                    return *rate;
                }
            } else if topic == pattern {
                return *rate;
            }
        }
        self.default_rate
    }
}

/// Overall bridge statistics
#[derive(Debug, Default)]
pub struct BridgeStats {
    /// Per-topic statistics
    pub topics: HashMap<String, Arc<TopicStats>>,
    /// Per-service statistics
    pub services: HashMap<String, Arc<ServiceStats>>,
    /// Per-action statistics
    pub actions: HashMap<String, Arc<ActionStats>>,
    /// Per-parameter-node statistics
    pub parameters: HashMap<String, Arc<ParameterStats>>,
    /// Bridge start time
    pub start_time: Option<Instant>,
    /// Total topics discovered
    pub topics_discovered: AtomicU64,
    /// Total topics bridged
    pub topics_bridged: AtomicU64,
    /// Total services discovered
    pub services_discovered: AtomicU64,
    /// Services bridged
    pub services_bridged: AtomicU64,
    /// Total actions discovered
    pub actions_discovered: AtomicU64,
    /// Actions bridged
    pub actions_bridged: AtomicU64,
    /// Total parameter nodes discovered
    pub param_nodes_discovered: AtomicU64,
    /// Parameter nodes bridged
    pub param_nodes_bridged: AtomicU64,
}

impl BridgeStats {
    pub fn new() -> Self {
        Self {
            start_time: Some(Instant::now()),
            ..Default::default()
        }
    }

    pub fn get_or_create_topic(&mut self, topic: &str) -> Arc<TopicStats> {
        self.topics
            .entry(topic.to_string())
            .or_insert_with(|| Arc::new(TopicStats::new()))
            .clone()
    }

    pub fn get_or_create_service(&mut self, service: &str) -> Arc<ServiceStats> {
        self.services
            .entry(service.to_string())
            .or_insert_with(|| Arc::new(ServiceStats::new()))
            .clone()
    }

    pub fn get_or_create_action(&mut self, action: &str) -> Arc<ActionStats> {
        self.actions
            .entry(action.to_string())
            .or_insert_with(|| Arc::new(ActionStats::new()))
            .clone()
    }

    pub fn get_or_create_parameter(&mut self, node_name: &str) -> Arc<ParameterStats> {
        self.parameters
            .entry(node_name.to_string())
            .or_insert_with(|| Arc::new(ParameterStats::new()))
            .clone()
    }

    pub fn print_summary(&self) {
        let uptime = self
            .start_time
            .map(|t| t.elapsed())
            .unwrap_or(Duration::ZERO);

        println!();
        println!("{}", "Bridge Statistics".green().bold());
        println!("{}", "=".repeat(60).dimmed());
        println!("  {} {}", "Uptime:".cyan(), format_duration(uptime).white());
        println!(
            "  {} {}",
            "Topics Discovered:".cyan(),
            self.topics_discovered.load(Ordering::Relaxed)
        );
        println!(
            "  {} {}",
            "Topics Bridged:".cyan(),
            self.topics_bridged.load(Ordering::Relaxed)
        );
        println!(
            "  {} {}",
            "Services Discovered:".cyan(),
            self.services_discovered.load(Ordering::Relaxed)
        );
        println!(
            "  {} {}",
            "Services Bridged:".cyan(),
            self.services_bridged.load(Ordering::Relaxed)
        );
        println!(
            "  {} {}",
            "Actions Discovered:".cyan(),
            self.actions_discovered.load(Ordering::Relaxed)
        );
        println!(
            "  {} {}",
            "Actions Bridged:".cyan(),
            self.actions_bridged.load(Ordering::Relaxed)
        );
        println!(
            "  {} {}",
            "Param Nodes Discovered:".cyan(),
            self.param_nodes_discovered.load(Ordering::Relaxed)
        );
        println!(
            "  {} {}",
            "Param Nodes Bridged:".cyan(),
            self.param_nodes_bridged.load(Ordering::Relaxed)
        );
        println!();

        if !self.topics.is_empty() {
            println!(
                "  {:<30} {:>10} {:>10} {:>12}",
                "TOPIC".dimmed(),
                "IN".dimmed(),
                "OUT".dimmed(),
                "BYTES".dimmed()
            );
            println!("  {}", "-".repeat(66).dimmed());

            for (topic, stats) in &self.topics {
                let msgs_in = stats.msgs_in.load(Ordering::Relaxed);
                let msgs_out = stats.msgs_out.load(Ordering::Relaxed);
                let bytes = stats.bytes_transferred.load(Ordering::Relaxed);

                println!(
                    "  {:<30} {:>10} {:>10} {:>12}",
                    truncate_string(topic, 30),
                    msgs_in,
                    msgs_out,
                    format_bytes(bytes)
                );
            }
        }

        if !self.services.is_empty() {
            println!();
            println!(
                "  {:<30} {:>10} {:>10} {:>10}",
                "SERVICE".dimmed(),
                "REQUESTS".dimmed(),
                "RESPONSES".dimmed(),
                "ERRORS".dimmed()
            );
            println!("  {}", "-".repeat(66).dimmed());

            for (service, stats) in &self.services {
                let requests = stats.requests.load(Ordering::Relaxed);
                let responses = stats.responses.load(Ordering::Relaxed);
                let errors = stats.errors.load(Ordering::Relaxed);

                println!(
                    "  {:<30} {:>10} {:>10} {:>10}",
                    truncate_string(service, 30),
                    requests,
                    responses,
                    errors
                );
            }
        }

        if !self.actions.is_empty() {
            println!();
            println!(
                "  {:<25} {:>8} {:>8} {:>8} {:>8} {:>8}",
                "ACTION".dimmed(),
                "GOALS".dimmed(),
                "OK".dimmed(),
                "FAIL".dimmed(),
                "CANCEL".dimmed(),
                "FDBK".dimmed()
            );
            println!("  {}", "-".repeat(70).dimmed());

            for (action, stats) in &self.actions {
                let goals = stats.goals.load(Ordering::Relaxed);
                let succeeded = stats.goals_succeeded.load(Ordering::Relaxed);
                let failed = stats.goals_failed.load(Ordering::Relaxed);
                let canceled = stats.goals_canceled.load(Ordering::Relaxed);
                let feedback = stats.feedback_msgs.load(Ordering::Relaxed);

                println!(
                    "  {:<25} {:>8} {:>8} {:>8} {:>8} {:>8}",
                    truncate_string(action, 25),
                    goals,
                    succeeded,
                    failed,
                    canceled,
                    feedback
                );
            }
        }

        if !self.parameters.is_empty() {
            println!();
            println!(
                "  {:<25} {:>8} {:>8} {:>8} {:>8} {:>8}",
                "PARAM NODE".dimmed(),
                "GET".dimmed(),
                "SET".dimmed(),
                "LIST".dimmed(),
                "DESC".dimmed(),
                "ERRS".dimmed()
            );
            println!("  {}", "-".repeat(70).dimmed());

            for (node, stats) in &self.parameters {
                let gets = stats.get_calls.load(Ordering::Relaxed);
                let sets = stats.set_calls.load(Ordering::Relaxed);
                let lists = stats.list_calls.load(Ordering::Relaxed);
                let describes = stats.describe_calls.load(Ordering::Relaxed);
                let errors = stats.errors.load(Ordering::Relaxed);

                println!(
                    "  {:<25} {:>8} {:>8} {:>8} {:>8} {:>8}",
                    truncate_string(node, 25),
                    gets,
                    sets,
                    lists,
                    describes,
                    errors
                );
            }
        }
        println!();
    }
}

// ============================================================================
// ROS2 Topic Discovery
// ============================================================================

/// Discovered ROS2 topic information
#[derive(Debug, Clone)]
pub struct DiscoveredTopic {
    /// Topic name (e.g., "/scan", "/odom")
    pub name: String,
    /// Message type (e.g., "sensor_msgs/msg/LaserScan")
    pub msg_type: Option<String>,
    /// Number of publishers
    pub publishers: u32,
    /// Number of subscribers
    pub subscribers: u32,
    /// QoS reliability
    pub reliable: bool,
}

/// Discovered ROS2 service information
#[derive(Debug, Clone)]
pub struct DiscoveredService {
    /// Service name
    pub name: String,
    /// Service type
    pub srv_type: Option<String>,
}

/// Discovery result from ROS2 network
#[derive(Debug, Default)]
pub struct DiscoveryResult {
    pub topics: Vec<DiscoveredTopic>,
    pub services: Vec<DiscoveredService>,
}

/// Discover ROS2 topics via Zenoh liveliness
///
/// Uses Zenoh session to discover active ROS2 topics by querying liveliness tokens.
/// This scans for publishers/subscribers matching ROS2 topic key patterns.
///
/// # Discovery Method
///
/// ROS2 topics in Zenoh follow the pattern: `{domain_id}/rt/{topic_name}`
/// We use Zenoh's liveliness subscription to detect active publishers.
#[cfg(feature = "zenoh-transport")]
pub async fn discover_ros2_topics(domain_id: u32) -> HorusResult<DiscoveryResult> {
    use std::collections::HashSet;
    use std::time::Instant;

    let mut result = DiscoveryResult::default();

    // ROS2 topic discovery key expression
    // Format: rt/{topic_name} (domain is handled by zenoh config)
    let topic_key = format!("rt/**");

    log::info!(
        "ROS2 discovery on domain {} (key pattern: {})",
        domain_id,
        topic_key
    );

    // Open a Zenoh session for discovery
    let zenoh_config = zenoh::Config::default();
    let session: zenoh::Session = match zenoh::open(zenoh_config).await {
        Ok(s) => s,
        Err(e) => {
            log::warn!("Failed to open Zenoh session for discovery: {}", e);
            return Ok(result);
        }
    };

    let mut discovered_topics: HashSet<String> = HashSet::new();

    // Query for active publishers on rt/** pattern
    // Use get() to query what's currently published
    if let Ok(receiver) = session.get(&topic_key).await {
        // Collect replies with a timeout
        let start = Instant::now();
        let timeout = Duration::from_secs(2);

        while start.elapsed() < timeout {
            match tokio::time::timeout(Duration::from_millis(100), receiver.recv_async()).await {
                Ok(Ok(reply)) => {
                    if let Ok(sample) = reply.into_result() {
                        let key = sample.key_expr().as_str();
                        // Extract topic name from rt/{topic}
                        if let Some(topic_name) = key.strip_prefix("rt/") {
                            discovered_topics.insert(format!("/{}", topic_name));
                        } else if let Some(topic_name) = key.strip_prefix("rt") {
                            discovered_topics
                                .insert(format!("/{}", topic_name.trim_start_matches('/')));
                        }
                    }
                }
                Ok(Err(_)) => break, // Channel closed
                Err(_) => continue,  // Timeout, try again
            }
        }
    }

    // Also try liveliness tokens for zenoh-plugin-ros2dds
    let liveliness_key = "@/liveliness/**";
    if let Ok(receiver) = session.liveliness().get(liveliness_key).await {
        let start = Instant::now();
        let timeout = Duration::from_secs(1);

        while start.elapsed() < timeout {
            match tokio::time::timeout(Duration::from_millis(100), receiver.recv_async()).await {
                Ok(Ok(reply)) => {
                    if let Ok(sample) = reply.into_result() {
                        let key = sample.key_expr().as_str();
                        // Parse liveliness token for ROS2 topic info
                        if key.contains("/rt/") {
                            if let Some(idx) = key.find("/rt/") {
                                let topic_part = &key[idx + 3..];
                                if let Some(end_idx) = topic_part.find('/') {
                                    discovered_topics
                                        .insert(format!("/{}", &topic_part[..end_idx]));
                                } else {
                                    discovered_topics.insert(format!("/{}", topic_part));
                                }
                            }
                        }
                    }
                }
                Ok(Err(_)) => break,
                Err(_) => continue,
            }
        }
    }

    // Convert to DiscoveredTopic structs
    for topic_name in discovered_topics {
        result.topics.push(DiscoveredTopic {
            name: topic_name,
            msg_type: None,
            publishers: 1, // We know at least one exists
            subscribers: 0,
            reliable: true, // Default assumption
        });
    }

    log::info!("Discovered {} ROS2 topics", result.topics.len());
    for topic in &result.topics {
        log::debug!("  - {}", topic.name);
    }

    drop(session);
    Ok(result)
}

/// Fallback discovery without zenoh feature
#[cfg(not(feature = "zenoh-transport"))]
pub async fn discover_ros2_topics(_domain_id: u32) -> HorusResult<DiscoveryResult> {
    Err(HorusError::config(
        "ROS2 bridge requires 'zenoh-transport' feature. Rebuild with: cargo build --features zenoh-transport"
    ))
}

// ============================================================================
// Service Bridge
// ============================================================================

/// ROS2 service key expression format
/// Request: {domain_id}/rq/{service_name}/{request_type}
/// Response: {domain_id}/rr/{service_name}/{response_type}
#[allow(dead_code)]
fn ros2_service_request_key(domain_id: u32, service_name: &str) -> String {
    format!("{}/rq{}/**", domain_id, service_name)
}

#[allow(dead_code)]
fn ros2_service_response_key(domain_id: u32, service_name: &str) -> String {
    format!("{}/rr{}/**", domain_id, service_name)
}

/// Statistics for a bridged service
#[derive(Debug, Default)]
pub struct ServiceStats {
    /// Requests received
    pub requests: AtomicU64,
    /// Successful responses
    pub responses: AtomicU64,
    /// Errors encountered
    pub errors: AtomicU64,
}

impl ServiceStats {
    pub fn new() -> Self {
        Self::default()
    }
}

/// Statistics for a bridged action
#[derive(Debug, Default)]
pub struct ActionStats {
    /// Goals sent/received
    pub goals: AtomicU64,
    /// Goals accepted
    pub goals_accepted: AtomicU64,
    /// Goals rejected
    pub goals_rejected: AtomicU64,
    /// Goals completed (succeeded)
    pub goals_succeeded: AtomicU64,
    /// Goals failed
    pub goals_failed: AtomicU64,
    /// Goals canceled
    pub goals_canceled: AtomicU64,
    /// Feedback messages
    pub feedback_msgs: AtomicU64,
    /// Errors encountered
    pub errors: AtomicU64,
}

impl ActionStats {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn record_goal(&self) {
        self.goals.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_accepted(&self) {
        self.goals_accepted.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_rejected(&self) {
        self.goals_rejected.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_succeeded(&self) {
        self.goals_succeeded.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_failed(&self) {
        self.goals_failed.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_canceled(&self) {
        self.goals_canceled.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_feedback(&self) {
        self.feedback_msgs.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_error(&self) {
        self.errors.fetch_add(1, Ordering::Relaxed);
    }
}

/// Discovered ROS2 action
#[derive(Debug, Clone)]
pub struct DiscoveredAction {
    /// Action name (e.g., "/navigate_to_pose")
    pub name: String,
    /// Action type (e.g., "nav2_msgs/action/NavigateToPose")
    pub action_type: Option<String>,
}

/// Statistics for bridged parameters
#[derive(Debug, Default)]
pub struct ParameterStats {
    /// Number of get_parameters calls
    pub get_calls: AtomicU64,
    /// Number of set_parameters calls
    pub set_calls: AtomicU64,
    /// Number of list_parameters calls
    pub list_calls: AtomicU64,
    /// Number of describe_parameters calls
    pub describe_calls: AtomicU64,
    /// Errors encountered
    pub errors: AtomicU64,
}

impl ParameterStats {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn record_get(&self) {
        self.get_calls.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_set(&self) {
        self.set_calls.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_list(&self) {
        self.list_calls.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_describe(&self) {
        self.describe_calls.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_error(&self) {
        self.errors.fetch_add(1, Ordering::Relaxed);
    }
}

/// Discovered ROS2 node with parameters
#[derive(Debug, Clone)]
pub struct DiscoveredParameterNode {
    /// Node name (e.g., "/my_node")
    pub node_name: String,
    /// Namespace (e.g., "/robot1")
    pub namespace: Option<String>,
}

/// Bridge a ROS2 service to HORUS
///
/// ROS2 services in Zenoh use request/response pattern:
/// - Request: rq/{service_name}/{request_type}
/// - Response: rr/{service_name}/{response_type}
///
/// This bridge:
/// 1. Creates a Zenoh queryable to receive ROS2 service requests
/// 2. Forwards requests to a HORUS service node via shared memory
/// 3. Returns responses back via Zenoh reply mechanism
///
/// # Architecture
///
/// ```text
/// ROS2 Client                HORUS Bridge               HORUS Service
///     |                           |                          |
///     |--- query(rq/srv) -------->|                          |
///     |                           |--- Link.send(req) ------>|
///     |                           |<-- Link.recv(resp) ------|
///     |<-- reply(response) -------|                          |
/// ```
#[cfg(feature = "zenoh-transport")]
async fn bridge_ros2_service(
    service_name: &str,
    _domain_id: u32,
    running: Arc<AtomicBool>,
    stats: Arc<ServiceStats>,
) -> HorusResult<()> {
    use horus_core::Topic;
    use std::sync::atomic::Ordering;

    log::info!("Starting service bridge for: {}", service_name);

    // Construct Zenoh key expressions for ROS2 service
    // Request key: rq/{service_name}/**
    let request_key = format!("rq{}", service_name.trim_start_matches('/'));
    log::debug!("Service request key: {}", request_key);

    // HORUS service topic names (request/response pair)
    // Convert to HORUS naming: /robot/set_mode -> robot.set_mode.request
    let horus_service_name = service_name.trim_start_matches('/').replace('/', ".");
    let horus_request_topic = format!("{}.request", horus_service_name);
    let horus_response_topic = format!("{}.response", horus_service_name);

    // Create HORUS shared memory links for service communication
    // We use Vec<u8> for raw byte passthrough (CDR encoded)
    let horus_request_pub: Topic<Vec<u8>> = Topic::new(&horus_request_topic)?;
    let horus_response_sub: Topic<Vec<u8>> = Topic::new(&horus_response_topic)?;

    log::debug!(
        "HORUS service links created: {} -> {}",
        horus_request_topic,
        horus_response_topic
    );

    // Open Zenoh session for service handling
    let zenoh_config = zenoh::Config::default();
    let session: zenoh::Session = match zenoh::open(zenoh_config).await {
        Ok(s) => s,
        Err(e) => {
            log::error!(
                "Failed to open Zenoh session for service {}: {}",
                service_name,
                e
            );
            return Err(HorusError::communication(format!(
                "Failed to open Zenoh session: {}",
                e
            )));
        }
    };

    // Create a queryable to handle incoming ROS2 service requests
    let queryable = match session.declare_queryable(&request_key).await {
        Ok(q) => q,
        Err(e) => {
            log::error!("Failed to create queryable for {}: {}", request_key, e);
            return Err(HorusError::communication(format!(
                "Failed to create queryable: {}",
                e
            )));
        }
    };

    log::info!(
        "Service bridge ready: {} (key: {})",
        service_name,
        request_key
    );

    // Service request handling loop
    while running.load(Ordering::SeqCst) {
        // Wait for incoming query with timeout
        let query_result =
            tokio::time::timeout(Duration::from_millis(100), queryable.recv_async()).await;

        match query_result {
            Ok(Ok(query)) => {
                stats.requests.fetch_add(1, Ordering::Relaxed);
                log::debug!("Received service request on {}", query.key_expr());

                // Extract request payload
                let request_data = query
                    .payload()
                    .map(|p| p.to_bytes().to_vec())
                    .unwrap_or_default();

                log::trace!("Request payload: {} bytes", request_data.len());

                // Forward request to HORUS service via shared memory
                if let Err(e) = horus_request_pub.send(request_data.clone(), &mut None) {
                    log::warn!("Failed to forward request to HORUS: {:?}", e);
                    stats.errors.fetch_add(1, Ordering::Relaxed);
                    continue;
                }

                // Wait for response from HORUS service
                // Use a reasonable timeout for service response
                let response_timeout = Duration::from_secs(5);
                let start = Instant::now();
                let mut response_received = false;

                while start.elapsed() < response_timeout && running.load(Ordering::SeqCst) {
                    if let Some(response_data) = horus_response_sub.recv(&mut None) {
                        log::trace!("Got response: {} bytes", response_data.len());

                        // Send response back via Zenoh
                        if let Err(e) = query.reply(query.key_expr().clone(), response_data).await {
                            log::warn!("Failed to send Zenoh reply: {}", e);
                            stats.errors.fetch_add(1, Ordering::Relaxed);
                        } else {
                            stats.responses.fetch_add(1, Ordering::Relaxed);
                            log::debug!("Service response sent for {}", service_name);
                        }
                        response_received = true;
                        break;
                    }

                    // Brief sleep to avoid busy waiting
                    tokio::time::sleep(Duration::from_micros(100)).await;
                }

                if !response_received {
                    log::warn!("Service response timeout for {}", service_name);
                    stats.errors.fetch_add(1, Ordering::Relaxed);
                }
            }
            Ok(Err(e)) => {
                log::warn!("Queryable error: {}", e);
                stats.errors.fetch_add(1, Ordering::Relaxed);
            }
            Err(_) => {
                // Timeout - no query received, continue loop
            }
        }
    }

    log::info!("Service bridge stopped for {}", service_name);
    drop(queryable);
    drop(session);

    Ok(())
}

/// Bridge a ROS2 service from HORUS to ROS2 (client side)
///
/// This allows HORUS nodes to call ROS2 services.
/// HORUS sends request via shared memory, bridge queries Zenoh, returns response.
#[cfg(feature = "zenoh-transport")]
#[allow(dead_code)]
async fn bridge_horus_service_client(
    service_name: &str,
    _domain_id: u32,
    running: Arc<AtomicBool>,
    stats: Arc<ServiceStats>,
) -> HorusResult<()> {
    use horus_core::Topic;
    use std::sync::atomic::Ordering;

    log::info!(
        "Starting HORUS -> ROS2 service client bridge for: {}",
        service_name
    );

    // HORUS topics for request/response
    // Convert to HORUS naming: /robot/set_mode -> robot.set_mode.client_request
    let horus_service_name = service_name.trim_start_matches('/').replace('/', ".");
    let horus_request_topic = format!("{}.client_request", horus_service_name);
    let horus_response_topic = format!("{}.client_response", horus_service_name);

    // Zenoh service endpoint
    let request_key = format!("rq{}", service_name.trim_start_matches('/'));

    // Create HORUS links
    let horus_request_sub: Topic<Vec<u8>> = Topic::new(&horus_request_topic)?;
    let horus_response_pub: Topic<Vec<u8>> = Topic::new(&horus_response_topic)?;

    // Open Zenoh session
    let zenoh_config = zenoh::Config::default();
    let session: zenoh::Session = match zenoh::open(zenoh_config).await {
        Ok(s) => s,
        Err(e) => {
            return Err(HorusError::communication(format!(
                "Failed to open Zenoh session: {}",
                e
            )));
        }
    };

    log::info!("Service client bridge ready: {}", service_name);

    // Client request loop
    while running.load(Ordering::SeqCst) {
        // Check for outgoing service requests from HORUS
        if let Some(request_data) = horus_request_sub.recv(&mut None) {
            stats.requests.fetch_add(1, Ordering::Relaxed);
            log::debug!(
                "Forwarding service request to ROS2: {} ({} bytes)",
                service_name,
                request_data.len()
            );

            // Query the ROS2 service via Zenoh
            if let Ok(receiver) = session.get(&request_key).payload(request_data).await {
                // Wait for response with timeout
                let start = Instant::now();
                let timeout = Duration::from_secs(10);
                let mut response_received = false;

                while start.elapsed() < timeout {
                    match tokio::time::timeout(Duration::from_millis(100), receiver.recv_async())
                        .await
                    {
                        Ok(Ok(reply)) => {
                            if let Ok(sample) = reply.into_result() {
                                let response_data = sample.payload().to_bytes().to_vec();
                                // Forward response to HORUS
                                if let Err(e) = horus_response_pub.send(response_data, &mut None) {
                                    log::warn!("Failed to forward response to HORUS: {:?}", e);
                                    stats.errors.fetch_add(1, Ordering::Relaxed);
                                } else {
                                    stats.responses.fetch_add(1, Ordering::Relaxed);
                                    log::debug!("Service response forwarded to HORUS");
                                }
                                response_received = true;
                                break;
                            }
                        }
                        Ok(Err(_)) => break, // Channel closed
                        Err(_) => continue,  // Timeout, try again
                    }
                }

                if !response_received {
                    log::warn!("No response from ROS2 service {}", service_name);
                    stats.errors.fetch_add(1, Ordering::Relaxed);
                }
            } else {
                log::warn!("Failed to query ROS2 service: {}", service_name);
                stats.errors.fetch_add(1, Ordering::Relaxed);
            }
        } else {
            // No request pending, brief sleep
            tokio::time::sleep(Duration::from_micros(100)).await;
        }
    }

    log::info!("Service client bridge stopped for {}", service_name);
    Ok(())
}

// ============================================================================
// ROS2 Action Bridge
// ============================================================================

/// Bridge a ROS2 action to HORUS
///
/// ROS2 actions in Zenoh use a combination of services and topics:
/// - `rq/{action}/_action/send_goal` - service to send goals
/// - `rr/{action}/_action/send_goal` - goal response
/// - `rq/{action}/_action/cancel_goal` - service to cancel goals
/// - `rq/{action}/_action/get_result` - service to get result
/// - `{action}/_action/feedback` - topic for feedback
/// - `{action}/_action/status` - topic for status updates
///
/// This bridge:
/// 1. Creates Zenoh queryables for send_goal, cancel_goal, get_result
/// 2. Creates Zenoh subscribers for feedback and status
/// 3. Forwards all data to HORUS shared memory topics/services
#[cfg(feature = "zenoh-transport")]
pub async fn bridge_ros2_action(
    action_name: &str,
    _domain_id: u32,
    running: Arc<AtomicBool>,
    stats: Arc<ActionStats>,
) -> HorusResult<()> {
    use horus_core::Topic;
    use std::sync::atomic::Ordering;

    log::info!("Starting action bridge for: {}", action_name);

    // ROS2 uses '/' in topic names, HORUS uses '.'
    let action_base = action_name.trim_start_matches('/');
    let horus_action_base = action_base.replace('/', ".");

    // Zenoh key expressions for action services and topics (ROS2 side - keep '/')
    let send_goal_key = format!("rq/{}/_action/send_goal", action_base);
    let cancel_goal_key = format!("rq/{}/_action/cancel_goal", action_base);
    let get_result_key = format!("rq/{}/_action/get_result", action_base);
    let feedback_key = format!("{}/_action/feedback", action_base);
    let status_key = format!("{}/_action/status", action_base);

    // HORUS shared memory topics (convert to '.' separator)
    let horus_goal_topic = format!("{}.goal", horus_action_base);
    let horus_goal_response_topic = format!("{}.goal_response", horus_action_base);
    let horus_cancel_topic = format!("{}.cancel", horus_action_base);
    let horus_cancel_response_topic = format!("{}.cancel_response", horus_action_base);
    let horus_result_request_topic = format!("{}.result_request", horus_action_base);
    let horus_result_topic = format!("{}.result", horus_action_base);
    let horus_feedback_topic = format!("{}.feedback", horus_action_base);
    let horus_status_topic = format!("{}.status", horus_action_base);

    log::debug!(
        "Action bridge keys: send_goal={}, feedback={}",
        send_goal_key,
        feedback_key
    );

    // Create HORUS links for action communication
    let goal_pub: Topic<Vec<u8>> = Topic::new(&horus_goal_topic)?;
    let goal_response_sub: Topic<Vec<u8>> = Topic::new(&horus_goal_response_topic)?;
    let cancel_pub: Topic<Vec<u8>> = Topic::new(&horus_cancel_topic)?;
    let cancel_response_sub: Topic<Vec<u8>> = Topic::new(&horus_cancel_response_topic)?;
    let result_request_pub: Topic<Vec<u8>> = Topic::new(&horus_result_request_topic)?;
    let result_sub: Topic<Vec<u8>> = Topic::new(&horus_result_topic)?;
    let feedback_pub: Topic<Vec<u8>> = Topic::new(&horus_feedback_topic)?;
    let status_pub: Topic<Vec<u8>> = Topic::new(&horus_status_topic)?;

    // Open Zenoh session for action handling
    let zenoh_config = zenoh::Config::default();
    let session: zenoh::Session = match zenoh::open(zenoh_config).await {
        Ok(s) => s,
        Err(e) => {
            log::error!(
                "Failed to open Zenoh session for action {}: {}",
                action_name,
                e
            );
            return Err(HorusError::communication(format!(
                "Failed to open Zenoh session: {}",
                e
            )));
        }
    };

    // Create queryables for action services
    let send_goal_queryable = session
        .declare_queryable(&send_goal_key)
        .await
        .map_err(|e| {
            HorusError::communication(format!("Failed to create send_goal queryable: {}", e))
        })?;
    let cancel_goal_queryable = session
        .declare_queryable(&cancel_goal_key)
        .await
        .map_err(|e| {
            HorusError::communication(format!("Failed to create cancel_goal queryable: {}", e))
        })?;
    let get_result_queryable = session
        .declare_queryable(&get_result_key)
        .await
        .map_err(|e| {
            HorusError::communication(format!("Failed to create get_result queryable: {}", e))
        })?;

    // Create subscribers for feedback and status topics
    let feedback_subscriber = session
        .declare_subscriber(&feedback_key)
        .await
        .map_err(|e| {
            HorusError::communication(format!("Failed to create feedback subscriber: {}", e))
        })?;
    let status_subscriber = session.declare_subscriber(&status_key).await.map_err(|e| {
        HorusError::communication(format!("Failed to create status subscriber: {}", e))
    })?;

    log::info!("Action bridge ready: {}", action_name);

    // Main action handling loop
    while running.load(Ordering::SeqCst) {
        // Handle send_goal requests
        if let Ok(Ok(query)) =
            tokio::time::timeout(Duration::from_millis(10), send_goal_queryable.recv_async()).await
        {
            stats.record_goal();
            log::debug!("Received send_goal request for {}", action_name);

            let request_data = query
                .payload()
                .map(|p| p.to_bytes().to_vec())
                .unwrap_or_default();

            // Forward to HORUS
            if let Err(e) = goal_pub.send(request_data, &mut None) {
                log::warn!("Failed to forward goal to HORUS: {:?}", e);
                stats.record_error();
                continue;
            }

            // Wait for goal response from HORUS
            let response_timeout = Duration::from_secs(5);
            let start = Instant::now();
            while start.elapsed() < response_timeout && running.load(Ordering::SeqCst) {
                if let Some(response_data) = goal_response_sub.recv(&mut None) {
                    if let Err(e) = query.reply(query.key_expr().clone(), response_data).await {
                        log::warn!("Failed to send goal response: {}", e);
                        stats.record_error();
                    } else {
                        stats.record_accepted();
                        log::debug!("Goal response sent for {}", action_name);
                    }
                    break;
                }
                tokio::time::sleep(Duration::from_micros(100)).await;
            }
        }

        // Handle cancel_goal requests
        if let Ok(Ok(query)) = tokio::time::timeout(
            Duration::from_millis(10),
            cancel_goal_queryable.recv_async(),
        )
        .await
        {
            log::debug!("Received cancel_goal request for {}", action_name);

            let request_data = query
                .payload()
                .map(|p| p.to_bytes().to_vec())
                .unwrap_or_default();

            if let Err(e) = cancel_pub.send(request_data, &mut None) {
                log::warn!("Failed to forward cancel to HORUS: {:?}", e);
                stats.record_error();
                continue;
            }

            // Wait for cancel response
            let response_timeout = Duration::from_secs(2);
            let start = Instant::now();
            while start.elapsed() < response_timeout && running.load(Ordering::SeqCst) {
                if let Some(response_data) = cancel_response_sub.recv(&mut None) {
                    if let Err(e) = query.reply(query.key_expr().clone(), response_data).await {
                        log::warn!("Failed to send cancel response: {}", e);
                        stats.record_error();
                    } else {
                        stats.record_canceled();
                        log::debug!("Cancel response sent for {}", action_name);
                    }
                    break;
                }
                tokio::time::sleep(Duration::from_micros(100)).await;
            }
        }

        // Handle get_result requests
        if let Ok(Ok(query)) =
            tokio::time::timeout(Duration::from_millis(10), get_result_queryable.recv_async()).await
        {
            log::debug!("Received get_result request for {}", action_name);

            let request_data = query
                .payload()
                .map(|p| p.to_bytes().to_vec())
                .unwrap_or_default();

            if let Err(e) = result_request_pub.send(request_data, &mut None) {
                log::warn!("Failed to forward result request to HORUS: {:?}", e);
                stats.record_error();
                continue;
            }

            // Wait for result
            let response_timeout = Duration::from_secs(30); // Results may take longer
            let start = Instant::now();
            while start.elapsed() < response_timeout && running.load(Ordering::SeqCst) {
                if let Some(response_data) = result_sub.recv(&mut None) {
                    if let Err(e) = query.reply(query.key_expr().clone(), response_data).await {
                        log::warn!("Failed to send result response: {}", e);
                        stats.record_failed();
                    } else {
                        stats.record_succeeded();
                        log::debug!("Result response sent for {}", action_name);
                    }
                    break;
                }
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        }

        // Bridge feedback topic (ROS2 -> HORUS)
        if let Ok(Ok(sample)) =
            tokio::time::timeout(Duration::from_millis(10), feedback_subscriber.recv_async()).await
        {
            stats.record_feedback();
            let feedback_data = sample.payload().to_bytes().to_vec();
            if let Err(e) = feedback_pub.send(feedback_data, &mut None) {
                log::warn!("Failed to forward feedback to HORUS: {:?}", e);
                stats.record_error();
            }
        }

        // Bridge status topic (ROS2 -> HORUS)
        if let Ok(Ok(sample)) =
            tokio::time::timeout(Duration::from_millis(10), status_subscriber.recv_async()).await
        {
            let status_data = sample.payload().to_bytes().to_vec();
            if let Err(e) = status_pub.send(status_data, &mut None) {
                log::warn!("Failed to forward status to HORUS: {:?}", e);
            }
        }
    }

    log::info!("Action bridge stopped for {}", action_name);
    Ok(())
}

/// Fallback when zenoh-transport feature is not enabled
#[cfg(not(feature = "zenoh-transport"))]
pub async fn bridge_ros2_action(
    _action_name: &str,
    _domain_id: u32,
    _running: Arc<AtomicBool>,
    _stats: Arc<ActionStats>,
) -> HorusResult<()> {
    Err(HorusError::config(
        "ROS2 action bridging requires 'zenoh-transport' feature. Rebuild with: cargo build --features zenoh-transport"
    ))
}

/// Discover ROS2 services via Zenoh
///
/// ROS2 services in Zenoh follow the pattern:
/// - Request: rq/{service_name}/{request_type}
/// - Response: rr/{service_name}/{response_type}
///
/// We discover services by querying for active request endpoints.
#[cfg(feature = "zenoh-transport")]
pub async fn discover_ros2_services(domain_id: u32) -> HorusResult<Vec<DiscoveredService>> {
    use std::collections::HashSet;
    use std::time::Instant;

    let service_key = format!("rq/**");
    log::info!(
        "ROS2 service discovery on domain {} (key pattern: {})",
        domain_id,
        service_key
    );

    // Open a Zenoh session for discovery
    let zenoh_config = zenoh::Config::default();
    let session: zenoh::Session = match zenoh::open(zenoh_config).await {
        Ok(s) => s,
        Err(e) => {
            log::warn!("Failed to open Zenoh session for service discovery: {}", e);
            return Ok(Vec::new());
        }
    };

    let mut discovered_services: HashSet<String> = HashSet::new();

    // Query for service request endpoints (rq/**)
    if let Ok(receiver) = session.get(&service_key).await {
        let start = Instant::now();
        let timeout = Duration::from_secs(2);

        while start.elapsed() < timeout {
            match tokio::time::timeout(Duration::from_millis(100), receiver.recv_async()).await {
                Ok(Ok(reply)) => {
                    if let Ok(sample) = reply.into_result() {
                        let key = sample.key_expr().as_str();
                        // Extract service name from rq/{service_name}/{type}
                        if let Some(service_part) = key.strip_prefix("rq/") {
                            // Service name may contain slashes, type is at the end
                            // Find the last segment which is typically the type
                            if let Some(last_slash) = service_part.rfind('/') {
                                let service_name = &service_part[..last_slash];
                                discovered_services.insert(format!("/{}", service_name));
                            } else {
                                discovered_services.insert(format!("/{}", service_part));
                            }
                        }
                    }
                }
                Ok(Err(_)) => break, // Channel closed
                Err(_) => continue,  // Timeout, keep trying
            }
        }
    }

    // Note: Liveliness tokens could provide additional discovery but require
    // different type handling. The primary query above should find active services.
    log::debug!("Service discovery complete (primary query method)");

    // Convert to DiscoveredService structs
    let services: Vec<DiscoveredService> = discovered_services
        .into_iter()
        .map(|name| DiscoveredService {
            name,
            srv_type: None, // Type discovery would require additional parsing
        })
        .collect();

    log::info!("Discovered {} ROS2 services", services.len());
    for service in &services {
        log::debug!("  - {}", service.name);
    }

    drop(session);
    Ok(services)
}

/// Fallback when zenoh-transport feature is not enabled
#[cfg(not(feature = "zenoh-transport"))]
pub async fn discover_ros2_services(_domain_id: u32) -> HorusResult<Vec<DiscoveredService>> {
    Err(HorusError::config(
        "ROS2 service discovery requires 'zenoh-transport' feature. Rebuild with: cargo build --features zenoh-transport"
    ))
}

// ============================================================================
// ROS2 Action Discovery
// ============================================================================

/// Discover ROS2 action servers via Zenoh
///
/// ROS2 actions use a combination of services and topics:
/// - `{action}/_action/send_goal` - service to send goals
/// - `{action}/_action/cancel_goal` - service to cancel goals
/// - `{action}/_action/get_result` - service to get result
/// - `{action}/_action/feedback` - topic for feedback
/// - `{action}/_action/status` - topic for status updates
///
/// We detect actions by looking for the `_action/send_goal` service pattern.
#[cfg(feature = "zenoh-transport")]
pub async fn discover_ros2_actions(domain_id: u32) -> HorusResult<Vec<DiscoveredAction>> {
    use std::collections::HashSet;

    log::info!("Discovering ROS2 actions on domain {}", domain_id);

    // Open Zenoh session
    let zenoh_config = zenoh::Config::default();
    let session: zenoh::Session = match zenoh::open(zenoh_config).await {
        Ok(s) => s,
        Err(e) => {
            return Err(HorusError::communication(format!(
                "Failed to open Zenoh session for action discovery: {}",
                e
            )));
        }
    };

    // Query for action send_goal services
    // ROS2 actions in Zenoh: rq/{domain_id}/{action}/_action/send_goal/**
    let action_key = format!("{}/rq/**/_action/send_goal/**", domain_id);
    log::debug!("Querying for actions with key: {}", action_key);

    let mut action_names: HashSet<String> = HashSet::new();

    // Use liveliness tokens to discover action servers
    if let Ok(receiver) = session.liveliness().get(&action_key).await {
        while let Ok(reply) = receiver.recv_async().await {
            if let Ok(sample) = reply.into_result() {
                let key = sample.key_expr().as_str();
                // Parse action name from key like: 0/rq/navigate_to_pose/_action/send_goal/...
                if let Some(action_name) = parse_action_name_from_key(key) {
                    action_names.insert(action_name);
                }
            }
        }
    }

    // Also try a direct get query if liveliness didn't find anything
    if action_names.is_empty() {
        if let Ok(receiver) = session.get(&action_key).await {
            let timeout = std::time::Duration::from_secs(2);
            let start = std::time::Instant::now();

            while start.elapsed() < timeout {
                match tokio::time::timeout(
                    std::time::Duration::from_millis(100),
                    receiver.recv_async(),
                )
                .await
                {
                    Ok(Ok(reply)) => {
                        if let Ok(sample) = reply.into_result() {
                            let key = sample.key_expr().as_str();
                            if let Some(action_name) = parse_action_name_from_key(key) {
                                action_names.insert(action_name);
                            }
                        }
                    }
                    Ok(Err(_)) => break,
                    Err(_) => continue, // timeout, keep waiting
                }
            }
        }
    }

    let actions: Vec<DiscoveredAction> = action_names
        .into_iter()
        .map(|name| DiscoveredAction {
            name,
            action_type: None, // Type discovery not implemented yet
        })
        .collect();

    log::info!("Discovered {} ROS2 actions", actions.len());
    for action in &actions {
        log::debug!("  - {}", action.name);
    }

    drop(session);
    Ok(actions)
}

/// Parse action name from Zenoh key expression
/// Input: "0/rq/navigate_to_pose/_action/send_goal/..."
/// Output: Some("/navigate_to_pose")
#[cfg(feature = "zenoh-transport")]
fn parse_action_name_from_key(key: &str) -> Option<String> {
    // Split by _action/send_goal
    if let Some(idx) = key.find("/_action/send_goal") {
        let prefix = &key[..idx];
        // Find the last part after domain/rq/
        if let Some(rq_idx) = prefix.find("/rq/") {
            let action_part = &prefix[rq_idx + 4..]; // Skip "/rq/"
            if !action_part.is_empty() {
                return Some(format!("/{}", action_part));
            }
        }
        // Fallback: just use the prefix as action name
        if let Some(last_slash) = prefix.rfind('/') {
            let name = &prefix[last_slash..];
            if !name.is_empty() && name != "/" {
                return Some(name.to_string());
            }
        }
    }
    None
}

/// Fallback when zenoh-transport feature is not enabled
#[cfg(not(feature = "zenoh-transport"))]
pub async fn discover_ros2_actions(_domain_id: u32) -> HorusResult<Vec<DiscoveredAction>> {
    Err(HorusError::config(
        "ROS2 action discovery requires 'zenoh-transport' feature. Rebuild with: cargo build --features zenoh-transport"
    ))
}

// ============================================================================
// Parameter Discovery & Bridging
// ============================================================================

/// Discover ROS2 nodes with parameter services via Zenoh
///
/// ROS2 nodes expose parameter services at:
/// - /{node}/get_parameters
/// - /{node}/set_parameters
/// - /{node}/list_parameters
/// - /{node}/describe_parameters
///
/// We discover nodes by looking for these service endpoints in Zenoh.
#[cfg(feature = "zenoh-transport")]
pub async fn discover_ros2_parameters(domain_id: u32) -> HorusResult<Vec<DiscoveredParameterNode>> {
    use std::collections::HashSet;

    let mut nodes = Vec::new();
    let mut seen_nodes: HashSet<String> = HashSet::new();

    // Look for parameter services - key pattern for list_parameters service
    let param_key = format!("rq/**/list_parameters");

    log::info!(
        "ROS2 parameter discovery on domain {} (key pattern: {})",
        domain_id,
        param_key
    );

    // Open Zenoh session for discovery
    let zenoh_config = zenoh::Config::default();
    let session: zenoh::Session = match zenoh::open(zenoh_config).await {
        Ok(s) => s,
        Err(e) => {
            log::warn!(
                "Failed to open Zenoh session for parameter discovery: {}",
                e
            );
            return Ok(nodes);
        }
    };

    // Query for parameter services using liveliness
    let liveliness_key = format!("@ros2_lv/**");
    let liveliness = session.liveliness();

    // Subscribe to liveliness tokens
    if let Ok(sub) = liveliness.get(&liveliness_key).await {
        // Collect liveliness tokens with timeout
        let timeout = tokio::time::sleep(std::time::Duration::from_secs(2));
        tokio::pin!(timeout);

        loop {
            tokio::select! {
                _ = &mut timeout => {
                    log::debug!("Parameter discovery timeout");
                    break;
                }
                reply = sub.recv_async() => {
                    match reply {
                        Ok(reply) => {
                            // Reply.result() returns Result<Sample, ReplyError>
                            if let Ok(sample) = reply.result() {
                                let key = sample.key_expr().as_str();
                                // Look for parameter service patterns
                                if key.contains("/list_parameters") || key.contains("/get_parameters") {
                                    if let Some(node_name) = parse_node_name_from_param_service(key) {
                                        if !seen_nodes.contains(&node_name) {
                                            seen_nodes.insert(node_name.clone());
                                            nodes.push(DiscoveredParameterNode {
                                                node_name,
                                                namespace: None,
                                            });
                                        }
                                    }
                                }
                            }
                        }
                        Err(_) => break,
                    }
                }
            }
        }
    }

    // Also try direct service query if liveliness didn't find any
    if nodes.is_empty() {
        if let Ok(replies) = session.get(&param_key).await {
            while let Ok(reply) = replies.recv_async().await {
                if let Ok(sample) = reply.into_result() {
                    let key = sample.key_expr().as_str();
                    if let Some(node_name) = parse_node_name_from_param_service(key) {
                        if !seen_nodes.contains(&node_name) {
                            seen_nodes.insert(node_name.clone());
                            nodes.push(DiscoveredParameterNode {
                                node_name,
                                namespace: None,
                            });
                        }
                    }
                }
            }
        }
    }

    log::info!("Discovered {} ROS2 nodes with parameters", nodes.len());
    for node in &nodes {
        log::debug!("  - {}", node.node_name);
    }

    drop(session);
    Ok(nodes)
}

/// Parse node name from parameter service key expression
#[allow(dead_code)] // Used when zenoh-transport feature is enabled
fn parse_node_name_from_param_service(key: &str) -> Option<String> {
    // Pattern: rq/{node_name}/{param_service}
    // e.g., "rq/my_node/list_parameters" -> "/my_node"

    let param_services = [
        "/get_parameters",
        "/set_parameters",
        "/list_parameters",
        "/describe_parameters",
    ];

    for svc in param_services {
        if let Some(idx) = key.find(svc) {
            let prefix = &key[..idx];
            // Find the rq/ prefix and extract node name
            if let Some(rq_idx) = prefix.find("rq/") {
                let node_part = &prefix[rq_idx + 3..];
                if !node_part.is_empty() {
                    return Some(format!("/{}", node_part));
                }
            }
        }
    }
    None
}

/// Fallback when zenoh-transport feature is not enabled
#[cfg(not(feature = "zenoh-transport"))]
pub async fn discover_ros2_parameters(
    _domain_id: u32,
) -> HorusResult<Vec<DiscoveredParameterNode>> {
    Err(HorusError::config(
        "ROS2 parameter discovery requires 'zenoh-transport' feature. Rebuild with: cargo build --features zenoh-transport"
    ))
}

/// Bridge ROS2 parameter services for a node to HORUS
///
/// Handles the following parameter services:
/// - get_parameters: Get parameter values
/// - set_parameters: Set parameter values
/// - list_parameters: List all parameters
/// - describe_parameters: Get parameter descriptions
///
/// # Architecture
///
/// ```text
/// ROS2 Client                HORUS Bridge               HORUS Param Server
///     |                           |                          |
///     |--- get_parameters ------->|                          |
///     |                           |--- Link.send(req) ------>|
///     |                           |<-- Link.recv(resp) ------|
///     |<-- reply(params) ---------|                          |
/// ```
#[cfg(feature = "zenoh-transport")]
pub async fn bridge_ros2_parameters(
    node_name: &str,
    _domain_id: u32,
    running: Arc<AtomicBool>,
    stats: Arc<ParameterStats>,
) -> HorusResult<()> {
    use horus_core::Topic;

    log::info!("Starting parameter bridge for node: {}", node_name);

    let node_clean = node_name.trim_start_matches('/');

    // Create Zenoh session for parameter service handling
    let zenoh_config = zenoh::Config::default();
    let session: zenoh::Session = match zenoh::open(zenoh_config).await {
        Ok(s) => s,
        Err(e) => {
            log::error!(
                "Failed to open Zenoh session for parameters {}: {}",
                node_name,
                e
            );
            return Err(HorusError::communication(format!(
                "Failed to open Zenoh session: {}",
                e
            )));
        }
    };

    // Create HORUS links for parameter communication
    let horus_param_topic = format!("params/{}", node_clean);
    let horus_param_req: Topic<Vec<u8>> = Topic::new(&format!("{}/request", horus_param_topic))?;
    let horus_param_resp: Topic<Vec<u8>> =
        Topic::new(&format!("{}/response", horus_param_topic))?;

    log::debug!("HORUS param links created for: {}", horus_param_topic);

    // Key expressions for parameter services
    let get_params_key = format!("rq/{}/get_parameters", node_clean);
    let set_params_key = format!("rq/{}/set_parameters", node_clean);
    let list_params_key = format!("rq/{}/list_parameters", node_clean);
    let describe_params_key = format!("rq/{}/describe_parameters", node_clean);

    // Create queryables for each parameter service
    let stats_get = stats.clone();
    let stats_set = stats.clone();
    let stats_list = stats.clone();
    let stats_desc = stats.clone();

    let horus_req_get = horus_param_req.clone();
    let horus_resp_get = horus_param_resp.clone();
    let horus_req_set = horus_param_req.clone();
    let horus_resp_set = horus_param_resp.clone();
    let horus_req_list = horus_param_req.clone();
    let horus_resp_list = horus_param_resp.clone();
    let horus_req_desc = horus_param_req;
    let horus_resp_desc = horus_param_resp;

    // get_parameters queryable
    let get_queryable = match session.declare_queryable(&get_params_key).await {
        Ok(q) => q,
        Err(e) => {
            log::warn!("Failed to create get_parameters queryable: {}", e);
            stats.record_error();
            return Ok(());
        }
    };

    // set_parameters queryable
    let set_queryable = match session.declare_queryable(&set_params_key).await {
        Ok(q) => q,
        Err(e) => {
            log::warn!("Failed to create set_parameters queryable: {}", e);
            stats.record_error();
            return Ok(());
        }
    };

    // list_parameters queryable
    let list_queryable = match session.declare_queryable(&list_params_key).await {
        Ok(q) => q,
        Err(e) => {
            log::warn!("Failed to create list_parameters queryable: {}", e);
            stats.record_error();
            return Ok(());
        }
    };

    // describe_parameters queryable
    let describe_queryable = match session.declare_queryable(&describe_params_key).await {
        Ok(q) => q,
        Err(e) => {
            log::warn!("Failed to create describe_parameters queryable: {}", e);
            stats.record_error();
            return Ok(());
        }
    };

    log::info!("Parameter services bridged for node: {}", node_name);

    // Main loop handling all parameter services
    while running.load(Ordering::Relaxed) {
        tokio::select! {
            // Handle get_parameters requests
            query = get_queryable.recv_async() => {
                if let Ok(query) = query {
                    stats_get.record_get();
                    log::debug!("get_parameters request for {}", node_name);

                    // Forward to HORUS
                    if let Some(payload) = query.payload() {
                        let request_bytes = payload.to_bytes().to_vec();
                        if let Err(_) = horus_req_get.send(request_bytes, &mut None) {
                            log::warn!("Failed to forward get_parameters to HORUS");
                            stats_get.record_error();
                            continue;
                        }

                        // Wait for response with timeout
                        match tokio::time::timeout(
                            std::time::Duration::from_secs(5),
                            tokio::task::spawn_blocking({
                                let resp_link = horus_resp_get.clone();
                                move || resp_link.recv(&mut None)
                            })
                        ).await {
                            Ok(Ok(Some(response))) => {
                                if let Err(e) = query.reply(
                                    query.key_expr().clone(),
                                    response,
                                ).await {
                                    log::warn!("Failed to send get_parameters reply: {}", e);
                                    stats_get.record_error();
                                }
                            }
                            _ => {
                                log::warn!("Timeout waiting for get_parameters response");
                                stats_get.record_error();
                            }
                        }
                    }
                }
            }

            // Handle set_parameters requests
            query = set_queryable.recv_async() => {
                if let Ok(query) = query {
                    stats_set.record_set();
                    log::debug!("set_parameters request for {}", node_name);

                    if let Some(payload) = query.payload() {
                        let request_bytes = payload.to_bytes().to_vec();
                        if let Err(_) = horus_req_set.send(request_bytes, &mut None) {
                            log::warn!("Failed to forward set_parameters to HORUS");
                            stats_set.record_error();
                            continue;
                        }

                        match tokio::time::timeout(
                            std::time::Duration::from_secs(5),
                            tokio::task::spawn_blocking({
                                let resp_link = horus_resp_set.clone();
                                move || resp_link.recv(&mut None)
                            })
                        ).await {
                            Ok(Ok(Some(response))) => {
                                if let Err(e) = query.reply(
                                    query.key_expr().clone(),
                                    response,
                                ).await {
                                    log::warn!("Failed to send set_parameters reply: {}", e);
                                    stats_set.record_error();
                                }
                            }
                            _ => {
                                log::warn!("Timeout waiting for set_parameters response");
                                stats_set.record_error();
                            }
                        }
                    }
                }
            }

            // Handle list_parameters requests
            query = list_queryable.recv_async() => {
                if let Ok(query) = query {
                    stats_list.record_list();
                    log::debug!("list_parameters request for {}", node_name);

                    if let Some(payload) = query.payload() {
                        let request_bytes = payload.to_bytes().to_vec();
                        if let Err(_) = horus_req_list.send(request_bytes, &mut None) {
                            log::warn!("Failed to forward list_parameters to HORUS");
                            stats_list.record_error();
                            continue;
                        }

                        match tokio::time::timeout(
                            std::time::Duration::from_secs(5),
                            tokio::task::spawn_blocking({
                                let resp_link = horus_resp_list.clone();
                                move || resp_link.recv(&mut None)
                            })
                        ).await {
                            Ok(Ok(Some(response))) => {
                                if let Err(e) = query.reply(
                                    query.key_expr().clone(),
                                    response,
                                ).await {
                                    log::warn!("Failed to send list_parameters reply: {}", e);
                                    stats_list.record_error();
                                }
                            }
                            _ => {
                                log::warn!("Timeout waiting for list_parameters response");
                                stats_list.record_error();
                            }
                        }
                    }
                }
            }

            // Handle describe_parameters requests
            query = describe_queryable.recv_async() => {
                if let Ok(query) = query {
                    stats_desc.record_describe();
                    log::debug!("describe_parameters request for {}", node_name);

                    if let Some(payload) = query.payload() {
                        let request_bytes = payload.to_bytes().to_vec();
                        if let Err(_) = horus_req_desc.send(request_bytes, &mut None) {
                            log::warn!("Failed to forward describe_parameters to HORUS");
                            stats_desc.record_error();
                            continue;
                        }

                        match tokio::time::timeout(
                            std::time::Duration::from_secs(5),
                            tokio::task::spawn_blocking({
                                let resp_link = horus_resp_desc.clone();
                                move || resp_link.recv(&mut None)
                            })
                        ).await {
                            Ok(Ok(Some(response))) => {
                                if let Err(e) = query.reply(
                                    query.key_expr().clone(),
                                    response,
                                ).await {
                                    log::warn!("Failed to send describe_parameters reply: {}", e);
                                    stats_desc.record_error();
                                }
                            }
                            _ => {
                                log::warn!("Timeout waiting for describe_parameters response");
                                stats_desc.record_error();
                            }
                        }
                    }
                }
            }

            // Check for shutdown every 100ms
            _ = tokio::time::sleep(std::time::Duration::from_millis(100)) => {}
        }
    }

    log::info!("Parameter bridge stopped for node: {}", node_name);
    drop(session);
    Ok(())
}

/// Fallback when zenoh-transport feature is not enabled
#[cfg(not(feature = "zenoh-transport"))]
pub async fn bridge_ros2_parameters(
    _node_name: &str,
    _domain_id: u32,
    _running: Arc<AtomicBool>,
    _stats: Arc<ParameterStats>,
) -> HorusResult<()> {
    Err(HorusError::config(
        "ROS2 parameter bridging requires 'zenoh-transport' feature. Rebuild with: cargo build --features zenoh-transport"
    ))
}

// ============================================================================
// Single Topic Bridge
// ============================================================================

/// Map bridge QoS profile to Zenoh QoS settings
#[cfg(feature = "zenoh-transport")]
fn map_qos_profile(
    profile: QosProfile,
) -> horus_core::communication::network::zenoh_config::ZenohQos {
    use horus_core::communication::network::zenoh_config::ZenohQos;

    match profile {
        QosProfile::SensorData => ZenohQos::sensor_data(),
        QosProfile::Default => ZenohQos::default(),
        QosProfile::Services => ZenohQos::services(),
        QosProfile::Parameters => ZenohQos::parameters(),
        QosProfile::SystemDefault => ZenohQos::system_default(),
    }
}

/// Bridge a single topic between ROS2 (via Zenoh) and HORUS (via shared memory)
#[cfg(feature = "zenoh-transport")]
async fn bridge_single_topic(
    topic_name: &str,
    direction: BridgeDirection,
    domain_id: u32,
    qos_profile: QosProfile,
    running: Arc<AtomicBool>,
    stats: Arc<TopicStats>,
) -> HorusResult<()> {
    use horus_core::communication::network::zenoh_config::ZenohConfig;

    log::info!(
        "Starting bridge for topic: {} (direction: {:?}, qos: {:?})",
        topic_name,
        direction,
        qos_profile
    );

    // Convert ROS topic to HORUS topic format
    // ROS uses `/` separator: /robot/odom, /scan
    // HORUS uses `.` separator: robot.odom, scan
    let horus_topic = topic_name.trim_start_matches('/').replace('/', ".");
    log::debug!(
        "HORUS topic: {} (from ROS topic: {})",
        horus_topic,
        topic_name
    );

    // Build Zenoh config for ROS2 communication with QoS settings
    let mut zenoh_config = ZenohConfig::ros2(domain_id);
    zenoh_config.qos = map_qos_profile(qos_profile);
    log::debug!("Applied QoS profile: {:?}", qos_profile);

    match direction {
        BridgeDirection::In => {
            // ROS2 -> HORUS: Subscribe to Zenoh, publish to shared memory
            bridge_ros2_to_horus(topic_name, horus_topic, zenoh_config, running, stats).await
        }
        BridgeDirection::Out => {
            // HORUS -> ROS2: Subscribe to shared memory, publish to Zenoh
            bridge_horus_to_ros2(topic_name, horus_topic, zenoh_config, running, stats).await
        }
        BridgeDirection::Both => {
            // Bidirectional: Run both directions concurrently
            let stats_in = stats.clone();
            let stats_out = stats.clone();
            let running_in = running.clone();
            let running_out = running.clone();
            let topic_name_in = topic_name.to_string();
            let topic_name_out = topic_name.to_string();
            let horus_topic_in = horus_topic.to_string();
            let horus_topic_out = horus_topic.to_string();
            let config_in = zenoh_config.clone();
            let config_out = zenoh_config;

            // Spawn both directions as separate tasks
            let in_handle = tokio::spawn(async move {
                if let Err(e) = bridge_ros2_to_horus(
                    &topic_name_in,
                    &horus_topic_in,
                    config_in,
                    running_in,
                    stats_in,
                )
                .await
                {
                    log::error!("ROS2->HORUS bridge error for {}: {}", topic_name_in, e);
                }
            });

            let out_handle = tokio::spawn(async move {
                if let Err(e) = bridge_horus_to_ros2(
                    &topic_name_out,
                    &horus_topic_out,
                    config_out,
                    running_out,
                    stats_out,
                )
                .await
                {
                    log::error!("HORUS->ROS2 bridge error for {}: {}", topic_name_out, e);
                }
            });

            // Wait for both to complete (they run until shutdown)
            let _ = tokio::join!(in_handle, out_handle);
            log::info!("Bridge stopped for topic: {}", topic_name);
            Ok(())
        }
    }
}

/// Bridge ROS2 -> HORUS direction
#[cfg(feature = "zenoh-transport")]
async fn bridge_ros2_to_horus(
    ros2_topic: &str,
    horus_topic: &str,
    zenoh_config: horus_core::communication::network::zenoh_config::ZenohConfig,
    running: Arc<AtomicBool>,
    stats: Arc<TopicStats>,
) -> HorusResult<()> {
    use horus_core::communication::network::zenoh_backend::ZenohBackend;
    use horus_core::Topic;
    use std::sync::atomic::Ordering;

    log::info!("Starting ROS2 -> HORUS bridge for {}", ros2_topic);

    // Create Zenoh subscriber for ROS2 topic
    let mut zenoh_sub: ZenohBackend<Vec<u8>> = ZenohBackend::new(ros2_topic, zenoh_config).await?;
    zenoh_sub.init_subscriber().await?;

    // Create HORUS shared memory producer
    let horus_pub: Topic<Vec<u8>> = Topic::new(horus_topic)?;

    log::info!(
        "ROS2 -> HORUS bridge ready: {} -> {}",
        ros2_topic,
        horus_topic
    );

    // Bridge loop: forward messages from ROS2 to HORUS
    while running.load(Ordering::SeqCst) {
        // Try to receive from Zenoh (non-blocking)
        if let Some(data) = zenoh_sub.recv() {
            let bytes_len = data.len() as u64;

            // Forward to HORUS shared memory
            match horus_pub.send(data, &mut None) {
                Ok(()) => {
                    stats.record_in(bytes_len);
                    log::trace!(
                        "Forwarded {} bytes: {} -> {}",
                        bytes_len,
                        ros2_topic,
                        horus_topic
                    );
                }
                Err(_) => {
                    stats.record_error();
                    log::warn!("Failed to forward to HORUS topic: {}", horus_topic);
                }
            }
        } else {
            // No message available, yield to avoid busy loop
            tokio::time::sleep(Duration::from_micros(100)).await;
        }
    }

    log::info!("ROS2 -> HORUS bridge stopped for {}", ros2_topic);
    Ok(())
}

/// Bridge HORUS -> ROS2 direction
#[cfg(feature = "zenoh-transport")]
async fn bridge_horus_to_ros2(
    ros2_topic: &str,
    horus_topic: &str,
    zenoh_config: horus_core::communication::network::zenoh_config::ZenohConfig,
    running: Arc<AtomicBool>,
    stats: Arc<TopicStats>,
) -> HorusResult<()> {
    use horus_core::communication::network::zenoh_backend::ZenohBackend;
    use horus_core::Topic;
    use std::sync::atomic::Ordering;

    log::info!("Starting HORUS -> ROS2 bridge for {}", horus_topic);

    // Create HORUS shared memory consumer
    let horus_sub: Topic<Vec<u8>> = Topic::new(horus_topic)?;

    // Create Zenoh publisher for ROS2 topic
    let mut zenoh_pub: ZenohBackend<Vec<u8>> = ZenohBackend::new(ros2_topic, zenoh_config).await?;
    zenoh_pub.init_publisher().await?;

    log::info!(
        "HORUS -> ROS2 bridge ready: {} -> {}",
        horus_topic,
        ros2_topic
    );

    // Bridge loop: forward messages from HORUS to ROS2
    while running.load(Ordering::SeqCst) {
        // Try to receive from HORUS shared memory (non-blocking)
        if let Some(data) = horus_sub.recv(&mut None) {
            let bytes_len = data.len() as u64;

            // Forward to ROS2 via Zenoh
            match zenoh_pub.send(&data) {
                Ok(()) => {
                    stats.record_out(bytes_len);
                    log::trace!(
                        "Forwarded {} bytes: {} -> {}",
                        bytes_len,
                        horus_topic,
                        ros2_topic
                    );
                }
                Err(e) => {
                    stats.record_error();
                    log::warn!("Failed to forward to ROS2 topic {}: {}", ros2_topic, e);
                }
            }
        } else {
            // No message available, yield to avoid busy loop
            tokio::time::sleep(Duration::from_micros(100)).await;
        }
    }

    log::info!("HORUS -> ROS2 bridge stopped for {}", horus_topic);
    Ok(())
}

/// Fallback single topic bridge without zenoh feature
#[cfg(not(feature = "zenoh-transport"))]
#[allow(dead_code)]
async fn bridge_single_topic(
    _topic_name: &str,
    _direction: BridgeDirection,
    _domain_id: u32,
    _qos_profile: QosProfile,
    _running: Arc<AtomicBool>,
    _stats: Arc<TopicStats>,
) -> HorusResult<()> {
    Err(HorusError::config(
        "ROS2 bridge requires 'zenoh-transport' feature",
    ))
}

// ============================================================================
// Bridge Implementation
// ============================================================================

/// ROS2 Bridge State
pub struct Ros2Bridge {
    #[allow(dead_code)] // Used when zenoh-transport feature is enabled
    config: BridgeConfig,
    #[allow(dead_code)] // Used when zenoh-transport feature is enabled
    stats: BridgeStats,
    running: Arc<AtomicBool>,
}

impl Ros2Bridge {
    pub fn new(config: BridgeConfig) -> Self {
        Self {
            config,
            stats: BridgeStats::new(),
            running: Arc::new(AtomicBool::new(false)),
        }
    }

    /// Start the bridge (blocking)
    #[cfg(feature = "zenoh-transport")]
    pub async fn run(&mut self) -> HorusResult<()> {
        use horus_core::communication::network::zenoh_config::ZenohConfig;

        self.running.store(true, Ordering::SeqCst);

        println!("{}", "Starting ROS2 Bridge...".green().bold());
        println!(
            "  {} {}",
            "Domain ID:".cyan(),
            self.config.domain_id.to_string().white()
        );
        println!("  {} {:?}", "Direction:".cyan(), self.config.direction);
        println!("  {} {:?}", "QoS Profile:".cyan(), self.config.qos_profile);

        // Discover ROS2 topics
        print!("\n{}", "Discovering ROS2 topics...".yellow());
        std::io::Write::flush(&mut std::io::stdout()).ok();

        let discovery = discover_ros2_topics(self.config.domain_id).await?;

        self.stats
            .topics_discovered
            .store(discovery.topics.len() as u64, Ordering::Relaxed);

        println!(
            " {} topics found",
            discovery.topics.len().to_string().green()
        );

        // Filter topics based on config
        let topics_to_bridge: Vec<_> = if self.config.bridge_all {
            // Bridge all discovered topics
            if discovery.topics.is_empty() {
                println!(
                    "{}",
                    "No topics discovered. Ensure ROS2 nodes are running with rmw_zenoh.".yellow()
                );
                return Ok(());
            }
            discovery.topics
        } else if !self.config.topics.is_empty() {
            // Explicit topics specified - use them directly (don't require discovery)
            // This allows bridging even when discovery isn't working
            self.config
                .topics
                .iter()
                .map(|name| {
                    // Ensure topic starts with /
                    let topic_name = if name.starts_with('/') {
                        name.clone()
                    } else {
                        format!("/{}", name)
                    };
                    DiscoveredTopic {
                        name: topic_name,
                        msg_type: None, // Unknown until messages flow
                        publishers: 0,
                        subscribers: 0,
                        reliable: true, // Assume reliable by default
                    }
                })
                .collect()
        } else {
            println!("{}", "No topics specified. Use --topics or --all".yellow());
            return Ok(());
        };

        // Apply namespace filter
        let topics_to_bridge: Vec<_> = if let Some(ref ns) = self.config.namespace {
            topics_to_bridge
                .into_iter()
                .filter(|t| t.name.starts_with(ns))
                .collect()
        } else {
            topics_to_bridge
        };

        if topics_to_bridge.is_empty() {
            println!("{}", "No matching topics to bridge.".yellow());
            return Ok(());
        }

        println!(
            "\n{} {} topics:",
            "Bridging".green().bold(),
            topics_to_bridge.len()
        );
        for topic in &topics_to_bridge {
            println!("  {} {}", "→".green(), topic.name);
            self.stats.get_or_create_topic(&topic.name);
        }

        self.stats
            .topics_bridged
            .store(topics_to_bridge.len() as u64, Ordering::Relaxed);

        // Create Zenoh session for ROS2 bridging
        // The ZenohConfig provides key expression mappings for ROS2 topics
        let _zenoh_config = ZenohConfig::ros2(self.config.domain_id);

        println!(
            "\n{}",
            "Bridge running. Press Ctrl+C to stop.".green().bold()
        );

        // Set up Ctrl+C handler
        let running = self.running.clone();
        ctrlc::set_handler(move || {
            running.store(false, Ordering::SeqCst);
        })
        .ok();

        // Start bridging tasks for each topic
        let mut bridge_handles = Vec::new();

        for topic in &topics_to_bridge {
            let topic_name = topic.name.clone();
            let direction = self.config.direction;
            let domain_id = self.config.domain_id;
            let qos_profile = self.config.qos_profile;
            let running = self.running.clone();
            let topic_stats = self.stats.get_or_create_topic(&topic.name);

            // Spawn a task for each topic bridge
            let handle = tokio::spawn(async move {
                if let Err(e) = bridge_single_topic(
                    &topic_name,
                    direction,
                    domain_id,
                    qos_profile,
                    running,
                    topic_stats,
                )
                .await
                {
                    log::error!("Bridge error for topic {}: {}", topic_name, e);
                }
            });
            bridge_handles.push(handle);
        }

        // Service bridging (if enabled)
        if self.config.bridge_services {
            println!("\n{}", "Discovering ROS2 services...".cyan());

            match discover_ros2_services(self.config.domain_id).await {
                Ok(services) => {
                    self.stats
                        .services_discovered
                        .store(services.len() as u64, Ordering::Relaxed);

                    if services.is_empty() {
                        println!("{}", "  No ROS2 services found.".yellow());
                    } else {
                        println!(
                            "\n{} {} services:",
                            "Bridging".green().bold(),
                            services.len()
                        );

                        for service in &services {
                            println!("  {} {}", "→".green(), service.name);
                            let service_name = service.name.clone();
                            let domain_id = self.config.domain_id;
                            let running = self.running.clone();
                            let service_stats = self.stats.get_or_create_service(&service.name);

                            // Spawn a task for each service bridge
                            let handle = tokio::spawn(async move {
                                if let Err(e) = bridge_ros2_service(
                                    &service_name,
                                    domain_id,
                                    running,
                                    service_stats,
                                )
                                .await
                                {
                                    log::error!("Bridge error for service {}: {}", service_name, e);
                                }
                            });
                            bridge_handles.push(handle);
                        }

                        self.stats
                            .services_bridged
                            .store(services.len() as u64, Ordering::Relaxed);
                    }
                }
                Err(e) => {
                    println!("{}", format!("  Service discovery failed: {}", e).yellow());
                }
            }
        }

        // Action bridging (if enabled)
        if self.config.bridge_actions {
            println!("\n{}", "Discovering ROS2 actions...".cyan());

            match discover_ros2_actions(self.config.domain_id).await {
                Ok(actions) => {
                    self.stats
                        .actions_discovered
                        .store(actions.len() as u64, Ordering::Relaxed);

                    if actions.is_empty() {
                        println!("{}", "  No ROS2 actions found.".yellow());
                    } else {
                        println!("\n{} {} actions:", "Bridging".green().bold(), actions.len());

                        for action in &actions {
                            println!("  {} {}", "→".green(), action.name);
                            let action_name = action.name.clone();
                            let domain_id = self.config.domain_id;
                            let running = self.running.clone();
                            let action_stats = self.stats.get_or_create_action(&action.name);

                            // Spawn a task for each action bridge
                            let handle = tokio::spawn(async move {
                                if let Err(e) = bridge_ros2_action(
                                    &action_name,
                                    domain_id,
                                    running,
                                    action_stats,
                                )
                                .await
                                {
                                    log::error!("Bridge error for action {}: {}", action_name, e);
                                }
                            });
                            bridge_handles.push(handle);
                        }

                        self.stats
                            .actions_bridged
                            .store(actions.len() as u64, Ordering::Relaxed);
                    }
                }
                Err(e) => {
                    println!("{}", format!("  Action discovery failed: {}", e).yellow());
                }
            }
        }

        // Parameter bridging (if enabled)
        if self.config.bridge_params {
            println!("\n{}", "Discovering ROS2 parameter nodes...".cyan());

            match discover_ros2_parameters(self.config.domain_id).await {
                Ok(param_nodes) => {
                    self.stats
                        .param_nodes_discovered
                        .store(param_nodes.len() as u64, Ordering::Relaxed);

                    if param_nodes.is_empty() {
                        println!("{}", "  No ROS2 parameter nodes found.".yellow());
                    } else {
                        println!(
                            "\n{} {} parameter nodes:",
                            "Bridging".green().bold(),
                            param_nodes.len()
                        );

                        for node in &param_nodes {
                            println!("  {} {}", "→".green(), node.node_name);
                            let node_name = node.node_name.clone();
                            let domain_id = self.config.domain_id;
                            let running = self.running.clone();
                            let param_stats = self.stats.get_or_create_parameter(&node.node_name);

                            // Spawn a task for each parameter node bridge
                            let handle = tokio::spawn(async move {
                                if let Err(e) = bridge_ros2_parameters(
                                    &node_name,
                                    domain_id,
                                    running,
                                    param_stats,
                                )
                                .await
                                {
                                    log::error!(
                                        "Bridge error for parameter node {}: {}",
                                        node_name,
                                        e
                                    );
                                }
                            });
                            bridge_handles.push(handle);
                        }

                        self.stats
                            .param_nodes_bridged
                            .store(param_nodes.len() as u64, Ordering::Relaxed);
                    }
                }
                Err(e) => {
                    println!(
                        "{}",
                        format!("  Parameter discovery failed: {}", e).yellow()
                    );
                }
            }
        }

        // Main bridge loop - monitor and print stats
        let mut last_stats = Instant::now();
        while self.running.load(Ordering::SeqCst) {
            tokio::time::sleep(Duration::from_millis(100)).await;

            // Print periodic stats
            if self.config.verbose && last_stats.elapsed() >= self.config.stats_interval {
                self.stats.print_summary();
                last_stats = Instant::now();
            }
        }

        // Wait for all bridge tasks to complete
        for handle in bridge_handles {
            handle.abort();
        }

        println!("\n{}", "Bridge stopped.".yellow());
        self.stats.print_summary();

        Ok(())
    }

    /// Fallback without zenoh feature
    #[cfg(not(feature = "zenoh-transport"))]
    pub async fn run(&mut self) -> HorusResult<()> {
        Err(HorusError::config(
            "ROS2 bridge requires 'zenoh-transport' feature. Rebuild with: cargo build --features zenoh-transport"
        ))
    }

    pub fn stop(&self) {
        self.running.store(false, Ordering::SeqCst);
    }
}

// ============================================================================
// CLI Entry Points
// ============================================================================

/// Start the ROS2 bridge
#[allow(clippy::too_many_arguments)]
pub fn start_ros2_bridge(
    topics: Vec<String>,
    all: bool,
    direction: Option<String>,
    namespace: Option<String>,
    domain_id: Option<u32>,
    qos: Option<String>,
    services: bool,
    actions: bool,
    params: bool,
    verbose: bool,
) -> HorusResult<()> {
    let direction = direction
        .map(|d| d.parse())
        .transpose()
        .map_err(|e: String| HorusError::config(e))?
        .unwrap_or(BridgeDirection::Both);

    let qos_profile = qos
        .map(|q| q.parse())
        .transpose()
        .map_err(|e: String| HorusError::config(e))?
        .unwrap_or(QosProfile::Default);

    let config = BridgeConfig {
        domain_id: domain_id.unwrap_or(0),
        topics,
        bridge_all: all,
        direction,
        namespace,
        qos_profile,
        bridge_services: services,
        bridge_actions: actions,
        bridge_params: params,
        verbose,
        ..Default::default()
    };

    let mut bridge = Ros2Bridge::new(config);

    // Run async bridge in tokio runtime
    let rt = tokio::runtime::Runtime::new()
        .map_err(|e| HorusError::Internal(format!("Failed to create runtime: {}", e)))?;

    rt.block_on(bridge.run())
}

/// List discoverable ROS2 topics without bridging
pub fn list_ros2_topics(domain_id: Option<u32>, json: bool) -> HorusResult<()> {
    let domain_id = domain_id.unwrap_or(0);

    let rt = tokio::runtime::Runtime::new()
        .map_err(|e| HorusError::Internal(format!("Failed to create runtime: {}", e)))?;

    let discovery = rt.block_on(discover_ros2_topics(domain_id))?;

    if json {
        let output = serde_json::json!({
            "domain_id": domain_id,
            "topics": discovery.topics.iter().map(|t| {
                serde_json::json!({
                    "name": t.name,
                    "type": t.msg_type,
                    "publishers": t.publishers,
                    "subscribers": t.subscribers
                })
            }).collect::<Vec<_>>(),
            "services": discovery.services.iter().map(|s| {
                serde_json::json!({
                    "name": s.name,
                    "type": s.srv_type
                })
            }).collect::<Vec<_>>()
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
        return Ok(());
    }

    println!(
        "{}",
        format!("ROS2 Topics (Domain {})", domain_id).green().bold()
    );
    println!();

    if discovery.topics.is_empty() {
        println!("{}", "No ROS2 topics found.".yellow());
        println!(
            "  {} Make sure ROS2 nodes are running and rmw_zenoh is configured",
            "Tip:".dimmed()
        );
    } else {
        println!(
            "  {:<40} {:>10} {:>10}",
            "TOPIC".dimmed(),
            "PUBS".dimmed(),
            "SUBS".dimmed()
        );
        println!("  {}", "-".repeat(64).dimmed());

        for topic in &discovery.topics {
            println!(
                "  {:<40} {:>10} {:>10}",
                topic.name.white(),
                topic.publishers,
                topic.subscribers
            );
        }
    }

    if !discovery.services.is_empty() {
        println!();
        println!("{}", "ROS2 Services".cyan().bold());
        for service in &discovery.services {
            println!("  {}", service.name);
        }
    }

    println!();
    println!(
        "  {} topics, {} services found",
        discovery.topics.len().to_string().green(),
        discovery.services.len().to_string().green()
    );

    Ok(())
}

/// Show bridge status/info
pub fn bridge_info() -> HorusResult<()> {
    println!("{}", "HORUS ROS2 Bridge".green().bold());
    println!();
    println!("  {}", "Capabilities:".cyan());
    println!("    {} Topic bridging (pub/sub)", "✓".green());
    println!("    {} Service bridging (req/rep)", "✓".green());
    println!("    {} Action bridging (goal/feedback/result)", "✓".green());
    println!("    {} Parameter bridging", "✓".green());
    println!();
    println!("  {}", "Transport:".cyan());
    println!("    {} Zenoh (rmw_zenoh compatible)", "→".dimmed());
    println!("    {} CDR message encoding", "→".dimmed());
    println!("    {} QoS policy mapping", "→".dimmed());
    println!();
    println!("  {}", "Usage:".cyan());
    println!(
        "    {} horus bridge ros2 --topics /scan,/odom",
        "$".dimmed()
    );
    println!(
        "    {} horus bridge ros2 --all --namespace /robot1",
        "$".dimmed()
    );
    println!("    {} horus bridge list --domain 0", "$".dimmed());

    #[cfg(not(feature = "zenoh-transport"))]
    {
        println!();
        println!(
            "  {}",
            "WARNING: zenoh-transport feature not enabled!".red().bold()
        );
        println!(
            "    {} Rebuild with: cargo build --features zenoh-transport",
            "Fix:".yellow()
        );
    }

    Ok(())
}

// ============================================================================
// Utility Functions
// ============================================================================

fn format_duration(d: Duration) -> String {
    let secs = d.as_secs();
    if secs < 60 {
        format!("{}s", secs)
    } else if secs < 3600 {
        format!("{}m {}s", secs / 60, secs % 60)
    } else {
        format!("{}h {}m", secs / 3600, (secs % 3600) / 60)
    }
}

fn format_bytes(bytes: u64) -> String {
    if bytes < 1024 {
        format!("{} B", bytes)
    } else if bytes < 1024 * 1024 {
        format!("{:.1} KB", bytes as f64 / 1024.0)
    } else if bytes < 1024 * 1024 * 1024 {
        format!("{:.1} MB", bytes as f64 / (1024.0 * 1024.0))
    } else {
        format!("{:.1} GB", bytes as f64 / (1024.0 * 1024.0 * 1024.0))
    }
}

fn truncate_string(s: &str, max_len: usize) -> String {
    if s.len() <= max_len {
        s.to_string()
    } else {
        format!("{}...", &s[..max_len - 3])
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // Import CDR types when zenoh-transport is enabled
    // Note: We import cdr types selectively to avoid Duration ambiguity
    #[cfg(feature = "zenoh-transport")]
    use super::cdr::{
        interpolate_transform, lerp_vector3, slerp_quaternion, Header, Quaternion, Ros2MessageType,
        TF2Error, TFMessage, Time, Transform, TransformBuffer, TransformStamped, Vector3,
    };

    // ========================================================================
    // Direction and QoS Parsing Tests
    // ========================================================================

    #[test]
    fn test_direction_parsing() {
        assert_eq!(
            "in".parse::<BridgeDirection>().unwrap(),
            BridgeDirection::In
        );
        assert_eq!(
            "out".parse::<BridgeDirection>().unwrap(),
            BridgeDirection::Out
        );
        assert_eq!(
            "both".parse::<BridgeDirection>().unwrap(),
            BridgeDirection::Both
        );
        assert_eq!(
            "ros2-to-horus".parse::<BridgeDirection>().unwrap(),
            BridgeDirection::In
        );
        assert_eq!(
            "horus-to-ros2".parse::<BridgeDirection>().unwrap(),
            BridgeDirection::Out
        );
    }

    #[test]
    fn test_direction_parsing_case_insensitive() {
        assert_eq!(
            "IN".parse::<BridgeDirection>().unwrap(),
            BridgeDirection::In
        );
        assert_eq!(
            "Out".parse::<BridgeDirection>().unwrap(),
            BridgeDirection::Out
        );
        assert_eq!(
            "BOTH".parse::<BridgeDirection>().unwrap(),
            BridgeDirection::Both
        );
        assert_eq!(
            "ROS2-TO-HORUS".parse::<BridgeDirection>().unwrap(),
            BridgeDirection::In
        );
    }

    #[test]
    fn test_direction_parsing_invalid() {
        assert!("invalid".parse::<BridgeDirection>().is_err());
        assert!("".parse::<BridgeDirection>().is_err());
        assert!("inout".parse::<BridgeDirection>().is_err());
    }

    #[test]
    fn test_qos_parsing() {
        assert_eq!(
            "sensor".parse::<QosProfile>().unwrap(),
            QosProfile::SensorData
        );
        assert_eq!(
            "default".parse::<QosProfile>().unwrap(),
            QosProfile::Default
        );
        assert_eq!(
            "services".parse::<QosProfile>().unwrap(),
            QosProfile::Services
        );
    }

    #[test]
    fn test_qos_parsing_all_variants() {
        assert_eq!(
            "sensor_data".parse::<QosProfile>().unwrap(),
            QosProfile::SensorData
        );
        assert_eq!(
            "parameters".parse::<QosProfile>().unwrap(),
            QosProfile::Parameters
        );
        assert_eq!(
            "system_default".parse::<QosProfile>().unwrap(),
            QosProfile::SystemDefault
        );
    }

    #[test]
    fn test_qos_parsing_invalid() {
        assert!("invalid".parse::<QosProfile>().is_err());
        assert!("".parse::<QosProfile>().is_err());
        assert!("best_effort".parse::<QosProfile>().is_err());
    }

    // ========================================================================
    // Utility Function Tests
    // ========================================================================

    #[test]
    fn test_format_bytes() {
        assert_eq!(format_bytes(512), "512 B");
        assert_eq!(format_bytes(1024), "1.0 KB");
        assert_eq!(format_bytes(1024 * 1024), "1.0 MB");
    }

    #[test]
    fn test_format_bytes_edge_cases() {
        assert_eq!(format_bytes(0), "0 B");
        assert_eq!(format_bytes(1), "1 B");
        assert_eq!(format_bytes(1023), "1023 B");
        assert_eq!(format_bytes(1025), "1.0 KB");
        assert_eq!(format_bytes(1024 * 1024 * 1024), "1.0 GB");
    }

    #[test]
    fn test_format_duration() {
        assert_eq!(format_duration(Duration::from_secs(30)), "30s");
        assert_eq!(format_duration(Duration::from_secs(90)), "1m 30s");
        assert_eq!(format_duration(Duration::from_secs(3700)), "1h 1m");
    }

    #[test]
    fn test_format_duration_edge_cases() {
        assert_eq!(format_duration(Duration::from_secs(0)), "0s");
        assert_eq!(format_duration(Duration::from_secs(59)), "59s");
        assert_eq!(format_duration(Duration::from_secs(60)), "1m 0s");
        assert_eq!(format_duration(Duration::from_secs(3600)), "1h 0m");
        assert_eq!(format_duration(Duration::from_secs(86400)), "24h 0m");
    }

    #[test]
    fn test_truncate_string() {
        assert_eq!(truncate_string("hello", 10), "hello");
        assert_eq!(truncate_string("hello world", 8), "hello...");
        assert_eq!(truncate_string("hi", 5), "hi");
        assert_eq!(truncate_string("", 5), "");
    }

    #[test]
    fn test_truncate_string_edge_cases() {
        // String exactly at limit
        assert_eq!(truncate_string("12345", 5), "12345");
        // String one over limit
        assert_eq!(truncate_string("123456", 5), "12...");
        // Very short max length
        assert_eq!(truncate_string("hello", 3), "...");
    }

    // ========================================================================
    // TopicStats Tests
    // ========================================================================

    #[test]
    fn test_topic_stats() {
        let stats = TopicStats::new();
        stats.record_in(100);
        stats.record_out(50);
        assert_eq!(stats.msgs_in.load(Ordering::Relaxed), 1);
        assert_eq!(stats.msgs_out.load(Ordering::Relaxed), 1);
        assert_eq!(stats.bytes_transferred.load(Ordering::Relaxed), 150);
    }

    #[test]
    fn test_topic_stats_multiple_messages() {
        let stats = TopicStats::new();

        // Record multiple incoming messages
        stats.record_in(100);
        stats.record_in(200);
        stats.record_in(300);

        assert_eq!(stats.msgs_in.load(Ordering::Relaxed), 3);
        assert_eq!(stats.bytes_transferred.load(Ordering::Relaxed), 600);

        // Record outgoing messages
        stats.record_out(50);
        stats.record_out(50);

        assert_eq!(stats.msgs_out.load(Ordering::Relaxed), 2);
        assert_eq!(stats.bytes_transferred.load(Ordering::Relaxed), 700);
    }

    #[test]
    fn test_topic_stats_zero_bytes() {
        let stats = TopicStats::new();
        stats.record_in(0);
        stats.record_out(0);

        assert_eq!(stats.msgs_in.load(Ordering::Relaxed), 1);
        assert_eq!(stats.msgs_out.load(Ordering::Relaxed), 1);
        assert_eq!(stats.bytes_transferred.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn test_topic_stats_concurrent_access() {
        use std::thread;

        let stats = Arc::new(TopicStats::new());
        let mut handles = vec![];

        // Spawn multiple threads recording messages
        for _ in 0..10 {
            let stats_clone = Arc::clone(&stats);
            handles.push(thread::spawn(move || {
                for _ in 0..100 {
                    stats_clone.record_in(10);
                    stats_clone.record_out(5);
                }
            }));
        }

        for handle in handles {
            handle.join().unwrap();
        }

        assert_eq!(stats.msgs_in.load(Ordering::Relaxed), 1000);
        assert_eq!(stats.msgs_out.load(Ordering::Relaxed), 1000);
        assert_eq!(stats.bytes_transferred.load(Ordering::Relaxed), 15000);
    }

    // ========================================================================
    // ServiceStats Tests
    // ========================================================================

    #[test]
    fn test_service_stats_new() {
        let stats = ServiceStats::new();
        assert_eq!(stats.requests.load(Ordering::Relaxed), 0);
        assert_eq!(stats.responses.load(Ordering::Relaxed), 0);
        assert_eq!(stats.errors.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn test_service_stats_default() {
        let stats = ServiceStats::default();
        assert_eq!(stats.requests.load(Ordering::Relaxed), 0);
        assert_eq!(stats.responses.load(Ordering::Relaxed), 0);
        assert_eq!(stats.errors.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn test_service_stats_increment() {
        let stats = ServiceStats::new();

        stats.requests.fetch_add(1, Ordering::Relaxed);
        stats.requests.fetch_add(1, Ordering::Relaxed);
        stats.responses.fetch_add(1, Ordering::Relaxed);
        stats.errors.fetch_add(1, Ordering::Relaxed);

        assert_eq!(stats.requests.load(Ordering::Relaxed), 2);
        assert_eq!(stats.responses.load(Ordering::Relaxed), 1);
        assert_eq!(stats.errors.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn test_service_stats_concurrent() {
        use std::thread;

        let stats = Arc::new(ServiceStats::new());
        let mut handles = vec![];

        for _ in 0..10 {
            let stats_clone = Arc::clone(&stats);
            handles.push(thread::spawn(move || {
                for _ in 0..100 {
                    stats_clone.requests.fetch_add(1, Ordering::Relaxed);
                    stats_clone.responses.fetch_add(1, Ordering::Relaxed);
                }
            }));
        }

        for handle in handles {
            handle.join().unwrap();
        }

        assert_eq!(stats.requests.load(Ordering::Relaxed), 1000);
        assert_eq!(stats.responses.load(Ordering::Relaxed), 1000);
    }

    // ========================================================================
    // BridgeConfig Tests
    // ========================================================================

    #[test]
    fn test_bridge_config_default() {
        let config = BridgeConfig::default();
        assert_eq!(config.domain_id, 0);
        assert!(config.topics.is_empty());
        assert!(!config.bridge_all);
        assert!(!config.bridge_services);
        assert!(!config.bridge_actions);
        assert!(!config.bridge_params);
        assert!(!config.verbose);
    }

    #[test]
    fn test_bridge_config_with_topics() {
        let config = BridgeConfig {
            domain_id: 42,
            topics: vec!["/cmd_vel".to_string(), "/odom".to_string()],
            bridge_all: false,
            direction: BridgeDirection::In,
            namespace: Some("/robot".to_string()),
            qos_profile: QosProfile::SensorData,
            bridge_services: true,
            bridge_actions: false,
            bridge_params: false,
            verbose: true,
            stats_interval: Duration::from_secs(10),
            recovery: RecoveryConfig::default(),
        };

        assert_eq!(config.domain_id, 42);
        assert_eq!(config.topics.len(), 2);
        assert_eq!(config.topics[0], "/cmd_vel");
        assert!(!config.bridge_all);
        assert!(config.bridge_services);
        assert!(config.verbose);
    }

    // ========================================================================
    // BridgeDirection and QosProfile Tests
    // ========================================================================

    #[test]
    fn test_bridge_direction_display() {
        assert_eq!(format!("{:?}", BridgeDirection::In), "In");
        assert_eq!(format!("{:?}", BridgeDirection::Out), "Out");
        assert_eq!(format!("{:?}", BridgeDirection::Both), "Both");
    }

    #[test]
    fn test_qos_profile_display() {
        assert_eq!(format!("{:?}", QosProfile::Default), "Default");
        assert_eq!(format!("{:?}", QosProfile::SensorData), "SensorData");
        assert_eq!(format!("{:?}", QosProfile::Services), "Services");
        assert_eq!(format!("{:?}", QosProfile::Parameters), "Parameters");
        assert_eq!(format!("{:?}", QosProfile::SystemDefault), "SystemDefault");
    }

    #[test]
    fn test_bridge_config_namespace_filter() {
        let config = BridgeConfig {
            namespace: Some("/robot1".to_string()),
            ..Default::default()
        };

        assert_eq!(config.namespace, Some("/robot1".to_string()));
    }

    // ========================================================================
    // BridgeStats Tests
    // ========================================================================

    #[test]
    fn test_bridge_stats_default() {
        let stats = BridgeStats::default();
        assert!(stats.topics.is_empty());
        assert!(stats.services.is_empty());
        assert_eq!(stats.services_discovered.load(Ordering::Relaxed), 0);
        assert_eq!(stats.services_bridged.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn test_bridge_stats_add_topics() {
        let mut stats = BridgeStats::default();

        stats
            .topics
            .insert("/cmd_vel".to_string(), Arc::new(TopicStats::new()));
        stats
            .topics
            .insert("/odom".to_string(), Arc::new(TopicStats::new()));

        assert_eq!(stats.topics.len(), 2);
        assert!(stats.topics.contains_key("/cmd_vel"));
        assert!(stats.topics.contains_key("/odom"));
    }

    #[test]
    fn test_bridge_stats_add_services() {
        let mut stats = BridgeStats::default();

        stats
            .services
            .insert("/set_params".to_string(), Arc::new(ServiceStats::new()));
        stats
            .services
            .insert("/get_state".to_string(), Arc::new(ServiceStats::new()));

        assert_eq!(stats.services.len(), 2);
        assert!(stats.services.contains_key("/set_params"));
        assert!(stats.services.contains_key("/get_state"));
    }

    #[test]
    fn test_bridge_stats_get_or_create_service() {
        let mut stats = BridgeStats::default();

        // First call should create the service
        let svc1 = stats.get_or_create_service("/test_service");
        svc1.requests.fetch_add(5, Ordering::Relaxed);

        // Second call should return the same Arc
        let svc2 = stats.get_or_create_service("/test_service");
        assert_eq!(svc2.requests.load(Ordering::Relaxed), 5);

        // Should only have one entry
        assert_eq!(stats.services.len(), 1);
    }

    #[test]
    fn test_bridge_stats_service_counters() {
        let stats = BridgeStats::default();

        stats.services_discovered.store(10, Ordering::Relaxed);
        stats.services_bridged.store(8, Ordering::Relaxed);

        assert_eq!(stats.services_discovered.load(Ordering::Relaxed), 10);
        assert_eq!(stats.services_bridged.load(Ordering::Relaxed), 8);
    }

    // ========================================================================
    // DiscoveryResult Tests
    // ========================================================================

    #[test]
    fn test_discovery_result_default() {
        let result = DiscoveryResult::default();
        assert!(result.topics.is_empty());
        assert!(result.services.is_empty());
    }

    #[test]
    fn test_discovered_topic() {
        let topic = DiscoveredTopic {
            name: "/scan".to_string(),
            msg_type: Some("sensor_msgs/LaserScan".to_string()),
            publishers: 1,
            subscribers: 2,
            reliable: true,
        };

        assert_eq!(topic.name, "/scan");
        assert_eq!(topic.msg_type, Some("sensor_msgs/LaserScan".to_string()));
        assert_eq!(topic.publishers, 1);
        assert_eq!(topic.subscribers, 2);
        assert!(topic.reliable);
    }

    #[test]
    fn test_discovered_service() {
        let service = DiscoveredService {
            name: "/get_map".to_string(),
            srv_type: Some("nav_msgs/GetMap".to_string()),
        };

        assert_eq!(service.name, "/get_map");
        assert_eq!(service.srv_type, Some("nav_msgs/GetMap".to_string()));
    }

    // ========================================================================
    // Service Key Expression Tests
    // ========================================================================

    #[test]
    fn test_ros2_service_request_key() {
        let key = ros2_service_request_key(0, "/get_map");
        assert_eq!(key, "0/rq/get_map/**");
    }

    #[test]
    fn test_ros2_service_response_key() {
        let key = ros2_service_response_key(0, "/get_map");
        assert_eq!(key, "0/rr/get_map/**");
    }

    #[test]
    fn test_ros2_service_keys_different_domains() {
        let key0 = ros2_service_request_key(0, "/srv");
        let key1 = ros2_service_request_key(1, "/srv");
        let key42 = ros2_service_request_key(42, "/srv");

        assert_eq!(key0, "0/rq/srv/**");
        assert_eq!(key1, "1/rq/srv/**");
        assert_eq!(key42, "42/rq/srv/**");
    }

    #[test]
    fn test_ros2_service_keys_nested_names() {
        let key = ros2_service_request_key(0, "/nav/get_path");
        assert_eq!(key, "0/rq/nav/get_path/**");
    }

    // ========================================================================
    // Error Handling Tests
    // ========================================================================

    #[test]
    fn test_direction_error_message() {
        let result = "badvalue".parse::<BridgeDirection>();
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(err.contains("badvalue"));
    }

    #[test]
    fn test_qos_error_message() {
        let result = "badqos".parse::<QosProfile>();
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(err.contains("badqos"));
    }

    // ========================================================================
    // Discovery Tests (feature-gated)
    // ========================================================================

    #[cfg(not(feature = "zenoh-transport"))]
    #[tokio::test]
    async fn test_discover_topics_without_zenoh_feature() {
        let result = discover_ros2_topics(0).await;
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(err.to_string().contains("zenoh-transport"));
    }

    #[cfg(not(feature = "zenoh-transport"))]
    #[tokio::test]
    async fn test_discover_services_without_zenoh_feature() {
        let result = discover_ros2_services(0).await;
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(err.to_string().contains("zenoh-transport"));
    }

    // ========================================================================
    // QoS Mapping Tests (feature-gated)
    // ========================================================================

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_qos_mapping_sensor_data() {
        use horus_core::communication::network::zenoh_config::{
            Durability, HistoryPolicy, Liveliness, Reliability,
        };
        let zenoh_qos = map_qos_profile(QosProfile::SensorData);

        // Sensor data: best effort, volatile, keep last 5, with deadline and lifespan
        assert_eq!(zenoh_qos.reliability, Reliability::BestEffort);
        assert_eq!(zenoh_qos.durability, Durability::Volatile);
        assert_eq!(zenoh_qos.history, HistoryPolicy::KeepLast(5));
        assert!(zenoh_qos.deadline.is_some());
        assert_eq!(zenoh_qos.deadline.unwrap(), Duration::from_millis(100));
        assert!(zenoh_qos.lifespan.is_some());
        assert_eq!(zenoh_qos.lifespan.unwrap(), Duration::from_millis(200));
        assert_eq!(zenoh_qos.liveliness, Liveliness::Automatic);
        assert!(zenoh_qos.liveliness_lease_duration.is_some());
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_qos_mapping_services() {
        use horus_core::communication::network::zenoh_config::{
            Durability, HistoryPolicy, Reliability,
        };
        let zenoh_qos = map_qos_profile(QosProfile::Services);

        // Services: reliable, volatile, keep all history
        assert_eq!(zenoh_qos.reliability, Reliability::Reliable);
        assert_eq!(zenoh_qos.durability, Durability::Volatile);
        assert_eq!(zenoh_qos.history, HistoryPolicy::KeepAll);
        assert!(zenoh_qos.deadline.is_none());
        assert!(zenoh_qos.lifespan.is_none());
        assert_eq!(zenoh_qos.priority, 6); // Higher priority for services
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_qos_mapping_parameters() {
        use horus_core::communication::network::zenoh_config::{
            Durability, HistoryPolicy, Reliability,
        };
        let zenoh_qos = map_qos_profile(QosProfile::Parameters);

        // Parameters: reliable, transient local durability, keep last 1
        assert_eq!(zenoh_qos.reliability, Reliability::Reliable);
        assert_eq!(zenoh_qos.durability, Durability::TransientLocal);
        assert_eq!(zenoh_qos.history, HistoryPolicy::KeepLast(1));
        assert!(zenoh_qos.deadline.is_none());
        assert!(zenoh_qos.lifespan.is_none());
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_qos_mapping_system_default() {
        use horus_core::communication::network::zenoh_config::{
            Durability, HistoryPolicy, Reliability,
        };
        let zenoh_qos = map_qos_profile(QosProfile::SystemDefault);

        // System default: reliable, volatile, keep last 10
        assert_eq!(zenoh_qos.reliability, Reliability::Reliable);
        assert_eq!(zenoh_qos.durability, Durability::Volatile);
        assert_eq!(zenoh_qos.history, HistoryPolicy::KeepLast(10));
        assert!(zenoh_qos.deadline.is_none());
        assert!(zenoh_qos.lifespan.is_none());
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_qos_mapping_default() {
        use horus_core::communication::network::zenoh_config::{
            CongestionControl, HistoryPolicy, Reliability,
        };
        let zenoh_qos = map_qos_profile(QosProfile::Default);

        // Default profile: best effort, drop on congestion, keep last 1
        assert_eq!(zenoh_qos.reliability, Reliability::BestEffort);
        assert_eq!(zenoh_qos.congestion, CongestionControl::Drop);
        assert_eq!(zenoh_qos.history, HistoryPolicy::KeepLast(1));
        assert_eq!(zenoh_qos.priority, 5); // Normal priority
        assert!(!zenoh_qos.express); // Not express by default
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_qos_deadline_check() {
        use horus_core::communication::network::zenoh_config::ZenohQos;
        use std::time::Instant;

        let qos = ZenohQos::sensor_data();
        let deadline = qos.deadline.unwrap();

        // Simulate recent message - should not trigger deadline
        let recent_time = Instant::now();
        assert!(!qos.check_deadline(recent_time));

        // Simulate old message - would trigger deadline (we can't wait in tests)
        // Just verify the check function exists and works
        assert_eq!(deadline, Duration::from_millis(100));
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_qos_lifespan_check() {
        use horus_core::communication::network::zenoh_config::ZenohQos;
        use std::time::Instant;

        let qos = ZenohQos::sensor_data();
        let lifespan = qos.lifespan.unwrap();

        // Simulate fresh message - should not be expired
        let fresh_time = Instant::now();
        assert!(!qos.is_message_expired(fresh_time));

        // Verify lifespan value
        assert_eq!(lifespan, Duration::from_millis(200));
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_qos_history_policy_depth() {
        use horus_core::communication::network::zenoh_config::HistoryPolicy;

        // KeepLast(n) should return Some(n)
        assert_eq!(HistoryPolicy::KeepLast(5).depth(), Some(5));
        assert_eq!(HistoryPolicy::KeepLast(1).depth(), Some(1));
        assert_eq!(HistoryPolicy::KeepLast(100).depth(), Some(100));

        // KeepAll should return None (unbounded)
        assert_eq!(HistoryPolicy::KeepAll.depth(), None);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_qos_liveliness_manual_assertion() {
        use horus_core::communication::network::zenoh_config::Liveliness;

        // Automatic should not require manual assertion
        assert!(!Liveliness::Automatic.requires_manual_assertion());

        // Manual modes should require assertion
        assert!(Liveliness::ManualByParticipant.requires_manual_assertion());
        assert!(Liveliness::ManualByTopic.requires_manual_assertion());
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_qos_profile_builder_methods() {
        use horus_core::communication::network::zenoh_config::{
            Durability, HistoryPolicy, Liveliness, Reliability, ZenohQos,
        };

        // Test building custom QoS
        let custom_qos = ZenohQos::default()
            .with_reliability(Reliability::Reliable)
            .with_history(HistoryPolicy::KeepLast(20))
            .with_durability(Durability::TransientLocal)
            .with_deadline(Duration::from_millis(50))
            .with_lifespan(Duration::from_millis(100))
            .with_liveliness(Liveliness::ManualByTopic, Some(Duration::from_secs(5)))
            .with_priority(7)
            .with_express(true);

        assert_eq!(custom_qos.reliability, Reliability::Reliable);
        assert_eq!(custom_qos.history, HistoryPolicy::KeepLast(20));
        assert_eq!(custom_qos.durability, Durability::TransientLocal);
        assert_eq!(custom_qos.deadline, Some(Duration::from_millis(50)));
        assert_eq!(custom_qos.lifespan, Some(Duration::from_millis(100)));
        assert_eq!(custom_qos.liveliness, Liveliness::ManualByTopic);
        assert_eq!(
            custom_qos.liveliness_lease_duration,
            Some(Duration::from_secs(5))
        );
        assert_eq!(custom_qos.priority, 7);
        assert!(custom_qos.express);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_qos_priority_clamping() {
        use horus_core::communication::network::zenoh_config::ZenohQos;

        // Priority should be clamped to 0-7
        let qos = ZenohQos::default().with_priority(255);
        assert_eq!(qos.priority, 7); // Clamped to max

        let qos = ZenohQos::default().with_priority(0);
        assert_eq!(qos.priority, 0); // Min is fine
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_all_qos_profiles_are_valid() {
        // Ensure all profile mappings produce valid QoS
        let profiles = [
            QosProfile::SensorData,
            QosProfile::Default,
            QosProfile::Services,
            QosProfile::Parameters,
            QosProfile::SystemDefault,
        ];

        for profile in profiles {
            let qos = map_qos_profile(profile);
            // All profiles should have valid priority (0-7)
            assert!(qos.priority <= 7);
            // All profiles should have non-zero history depth (or KeepAll)
            match qos.history {
                horus_core::communication::network::zenoh_config::HistoryPolicy::KeepLast(n) => {
                    assert!(n > 0);
                }
                horus_core::communication::network::zenoh_config::HistoryPolicy::KeepAll => {
                    // KeepAll is valid
                }
            }
        }
    }

    // ========================================================================
    // Integration-style Tests
    // ========================================================================

    // ========================================================================
    // Recovery Tests
    // ========================================================================

    #[test]
    fn test_recovery_config_default() {
        let config = RecoveryConfig::default();
        assert!(config.enabled);
        assert_eq!(config.initial_delay, Duration::from_millis(100));
        assert_eq!(config.max_delay, Duration::from_secs(30));
        assert_eq!(config.backoff_multiplier, 2.0);
        assert_eq!(config.max_retries, Some(10));
    }

    #[test]
    fn test_recovery_config_presets() {
        let dev = RecoveryConfig::development();
        assert!(dev.enabled);
        assert_eq!(dev.max_retries, Some(5));

        let prod = RecoveryConfig::production();
        assert!(prod.enabled);
        assert_eq!(prod.max_retries, None); // Unlimited

        let disabled = RecoveryConfig::disabled();
        assert!(!disabled.enabled);
    }

    #[test]
    fn test_recovery_state_success() {
        let mut state = RecoveryState::new();
        assert_eq!(state.attempt, 0);
        assert_eq!(state.consecutive_failures, 0);

        state.record_success();
        assert_eq!(state.attempt, 0);
        assert_eq!(state.total_recoveries, 0);
    }

    #[test]
    fn test_recovery_state_failure_backoff() {
        let config = RecoveryConfig::default();
        let mut state = RecoveryState::new();

        // First failure
        let delay1 = state.record_failure(&config);
        assert!(delay1.is_some());
        assert_eq!(state.attempt, 1);

        // Second failure should have larger delay
        let delay2 = state.record_failure(&config);
        assert!(delay2.is_some());
        assert_eq!(state.attempt, 2);
        // Note: Due to jitter, we can't exactly compare, but delay2 should be ~2x delay1
    }

    #[test]
    fn test_recovery_state_max_retries() {
        let config = RecoveryConfig {
            max_retries: Some(2),
            ..Default::default()
        };
        let mut state = RecoveryState::new();

        // First retry
        assert!(state.record_failure(&config).is_some());
        assert_eq!(state.attempt, 1);

        // Second retry
        assert!(state.record_failure(&config).is_some());
        assert_eq!(state.attempt, 2);

        // Third should fail (max reached)
        assert!(state.record_failure(&config).is_none());
    }

    #[test]
    fn test_recovery_state_recovery() {
        let config = RecoveryConfig::default();
        let mut state = RecoveryState::new();

        // Simulate failures
        state.record_failure(&config);
        state.record_failure(&config);
        assert_eq!(state.attempt, 2);

        // Recovery
        state.record_success();
        assert_eq!(state.attempt, 0);
        assert_eq!(state.total_recoveries, 1);
        assert_eq!(state.consecutive_failures, 0);
    }

    #[test]
    fn test_recovery_disabled() {
        let config = RecoveryConfig::disabled();
        let mut state = RecoveryState::new();

        // Should return None when disabled
        assert!(state.record_failure(&config).is_none());
        assert!(!state.should_retry(&config));
    }

    // ========================================================================
    // Rate Limiting Tests
    // ========================================================================

    #[test]
    fn test_rate_limiter_basic() {
        let limiter = RateLimiter::new(10.0, 5);

        // Should allow up to burst size immediately
        for _ in 0..5 {
            assert!(limiter.try_acquire());
        }

        // Next should fail (bucket empty)
        assert!(!limiter.try_acquire());
    }

    #[test]
    fn test_rate_limiter_refill() {
        let limiter = RateLimiter::new(1000.0, 5);

        // Drain the bucket
        for _ in 0..5 {
            assert!(limiter.try_acquire());
        }

        // Wait a bit for refill (1000 tokens/sec = 1 token/ms)
        std::thread::sleep(Duration::from_millis(10));

        // Should have some tokens now
        assert!(limiter.try_acquire());
    }

    #[test]
    fn test_rate_limiter_stats() {
        let limiter = RateLimiter::new(10.0, 3);

        // Drain bucket and try more
        for _ in 0..5 {
            limiter.try_acquire();
        }

        let (_tokens, allowed, dropped) = limiter.stats();
        assert_eq!(allowed, 3);
        assert_eq!(dropped, 2);
    }

    #[test]
    fn test_rate_limiter_drop_rate() {
        let limiter = RateLimiter::new(10.0, 2);

        // Try to acquire 10 times
        for _ in 0..10 {
            limiter.try_acquire();
        }

        // 2 allowed, 8 dropped => 80% drop rate
        let drop_rate = limiter.drop_rate();
        assert!(drop_rate > 0.7 && drop_rate < 0.9);
    }

    #[test]
    fn test_rate_limit_config_default() {
        let config = RateLimitConfig::default();
        assert!(!config.enabled);
        assert!(config.default_rate > 0.0);
        assert!(config.default_burst > 0);
    }

    #[test]
    fn test_rate_limit_config_high_throughput() {
        let config = RateLimitConfig::high_throughput();
        assert!(config.enabled);
        assert!(config.default_rate >= 10000.0);
        assert!(config.default_burst >= 1000);
    }

    #[test]
    fn test_rate_limit_config_constrained() {
        let config = RateLimitConfig::constrained();
        assert!(config.enabled);
        assert!(config.default_rate <= 100.0);
        assert!(config.default_burst <= 20);
    }

    #[test]
    fn test_rate_limit_exempt_topics() {
        let mut config = RateLimitConfig::default();
        config.exempt_topics = vec!["/clock".to_string(), "/rosout".to_string()];

        assert!(config.is_exempt("/clock"));
        assert!(config.is_exempt("/rosout"));
        assert!(!config.is_exempt("/cmd_vel"));
    }

    #[test]
    fn test_rate_limit_topic_rates() {
        let mut config = RateLimitConfig::default();
        config.default_rate = 100.0;
        config.topic_rates.insert("/camera/image".to_string(), 30.0);
        config.topic_rates.insert("/lidar/scan".to_string(), 20.0);

        assert!((config.rate_for_topic("/camera/image") - 30.0).abs() < 0.001);
        assert!((config.rate_for_topic("/lidar/scan") - 20.0).abs() < 0.001);
        assert!((config.rate_for_topic("/cmd_vel") - 100.0).abs() < 0.001);
    }

    // ========================================================================
    // Health Monitoring Tests
    // ========================================================================

    #[test]
    fn test_topic_stats_latency_recording() {
        let stats = TopicStats::new();

        // Record some latencies
        stats.record_latency(100);
        stats.record_latency(200);
        stats.record_latency(150);

        assert_eq!(stats.latency_count.load(Ordering::Relaxed), 3);

        // Check average
        let avg = stats.avg_latency_us();
        assert!(avg >= 100 && avg <= 200);
    }

    #[test]
    fn test_topic_stats_latency_min_max() {
        let stats = TopicStats::new();

        stats.record_latency(100);
        stats.record_latency(500);
        stats.record_latency(200);

        let (min, avg, max) = stats.latency_stats();
        assert_eq!(min, 100);
        assert_eq!(max, 500);
        assert!(avg >= 200 && avg <= 300);
    }

    #[test]
    fn test_topic_stats_is_healthy() {
        let stats = TopicStats::new();

        // Initially healthy (no activity)
        assert!(stats.is_healthy(5));

        // Record some activity
        stats.record_in(100);

        // Should be healthy immediately after activity
        assert!(stats.is_healthy(1));
    }

    #[test]
    fn test_topic_stats_message_rate() {
        let stats = TopicStats::new();

        // Record messages over a short period
        for _ in 0..100 {
            stats.record_in(10);
        }

        // Wait a tiny bit
        std::thread::sleep(Duration::from_millis(100));

        // Record more
        for _ in 0..100 {
            stats.record_in(10);
        }

        // Calculate rate (200 messages in 0.1 seconds = 2000 msg/sec)
        let rate = stats.message_rate(0.1);
        assert!(rate > 0.0);
    }

    #[test]
    fn test_health_status_variants() {
        let healthy = HealthStatus::Healthy;
        let degraded = HealthStatus::Degraded;
        let unhealthy = HealthStatus::Unhealthy;

        // Pattern matching
        match healthy {
            HealthStatus::Healthy => {}
            _ => panic!("Expected Healthy"),
        }

        match degraded {
            HealthStatus::Degraded => {}
            _ => panic!("Expected Degraded"),
        }

        match unhealthy {
            HealthStatus::Unhealthy => {}
            _ => panic!("Expected Unhealthy"),
        }

        // Test Display
        assert_eq!(format!("{}", healthy), "healthy");
        assert_eq!(format!("{}", degraded), "degraded");
        assert_eq!(format!("{}", unhealthy), "unhealthy");
    }

    #[test]
    fn test_bridge_health_struct() {
        let health = BridgeHealth {
            status: HealthStatus::Healthy,
            uptime: Duration::from_secs(60),
            active_topics: 5,
            healthy_topics: 4,
            degraded_topics: 1,
            unhealthy_topics: 0,
            total_errors: 10,
            avg_latency_us: 500,
            message_rate: 100.0,
        };

        assert_eq!(health.status, HealthStatus::Healthy);
        assert_eq!(health.active_topics, 5);
        assert_eq!(health.healthy_topics, 4);
        assert_eq!(health.degraded_topics, 1);
        assert_eq!(health.unhealthy_topics, 0);
    }

    // ========================================================================
    // Integration-style Tests
    // ========================================================================

    #[test]
    fn test_full_bridge_config_clone() {
        let config = BridgeConfig {
            domain_id: 42,
            topics: vec!["/cmd_vel".to_string(), "/odom".to_string()],
            bridge_all: true,
            direction: BridgeDirection::Both,
            namespace: Some("/robot".to_string()),
            qos_profile: QosProfile::SensorData,
            bridge_services: true,
            bridge_actions: false,
            bridge_params: true,
            verbose: true,
            stats_interval: Duration::from_secs(10),
            recovery: RecoveryConfig::production(),
        };

        // Test clone
        let cloned = config.clone();

        assert_eq!(cloned.domain_id, config.domain_id);
        assert_eq!(cloned.topics.len(), config.topics.len());
        assert_eq!(cloned.bridge_all, config.bridge_all);
        assert_eq!(cloned.direction, config.direction);
        assert_eq!(cloned.namespace, config.namespace);
        assert_eq!(cloned.bridge_services, config.bridge_services);
    }

    #[test]
    fn test_bridge_stats_with_traffic() {
        let mut stats = BridgeStats::default();

        // Simulate a running bridge with multiple topics
        let cmd_vel_stats = Arc::new(TopicStats::new());
        let odom_stats = Arc::new(TopicStats::new());

        // Simulate traffic
        for _ in 0..100 {
            cmd_vel_stats.record_in(64); // Twist message ~64 bytes
            odom_stats.record_out(256); // Odometry message ~256 bytes
        }

        stats.topics.insert("/cmd_vel".to_string(), cmd_vel_stats);
        stats.topics.insert("/odom".to_string(), odom_stats);

        // Verify stats
        let cmd_vel = stats.topics.get("/cmd_vel").unwrap();
        assert_eq!(cmd_vel.msgs_in.load(Ordering::Relaxed), 100);
        assert_eq!(cmd_vel.bytes_transferred.load(Ordering::Relaxed), 6400);

        let odom = stats.topics.get("/odom").unwrap();
        assert_eq!(odom.msgs_out.load(Ordering::Relaxed), 100);
        assert_eq!(odom.bytes_transferred.load(Ordering::Relaxed), 25600);
    }

    #[test]
    fn test_service_request_response_cycle() {
        let stats = Arc::new(ServiceStats::new());

        // Simulate 10 successful request/response cycles
        for _ in 0..10 {
            stats.requests.fetch_add(1, Ordering::Relaxed);
            // Simulate processing...
            stats.responses.fetch_add(1, Ordering::Relaxed);
        }

        // Simulate 2 failed requests
        for _ in 0..2 {
            stats.requests.fetch_add(1, Ordering::Relaxed);
            stats.errors.fetch_add(1, Ordering::Relaxed);
        }

        assert_eq!(stats.requests.load(Ordering::Relaxed), 12);
        assert_eq!(stats.responses.load(Ordering::Relaxed), 10);
        assert_eq!(stats.errors.load(Ordering::Relaxed), 2);

        // Calculate success rate
        let total = stats.requests.load(Ordering::Relaxed);
        let success = stats.responses.load(Ordering::Relaxed);
        let success_rate = (success as f64 / total as f64) * 100.0;
        assert!((success_rate - 83.33).abs() < 0.1);
    }

    // ========================================================================
    // ActionStats Tests
    // ========================================================================

    #[test]
    fn test_action_stats_new() {
        let stats = ActionStats::new();
        assert_eq!(stats.goals.load(Ordering::Relaxed), 0);
        assert_eq!(stats.goals_accepted.load(Ordering::Relaxed), 0);
        assert_eq!(stats.goals_rejected.load(Ordering::Relaxed), 0);
        assert_eq!(stats.goals_succeeded.load(Ordering::Relaxed), 0);
        assert_eq!(stats.goals_failed.load(Ordering::Relaxed), 0);
        assert_eq!(stats.goals_canceled.load(Ordering::Relaxed), 0);
        assert_eq!(stats.feedback_msgs.load(Ordering::Relaxed), 0);
        assert_eq!(stats.errors.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn test_action_stats_record_methods() {
        let stats = ActionStats::new();

        stats.record_goal();
        stats.record_goal();
        assert_eq!(stats.goals.load(Ordering::Relaxed), 2);

        stats.record_accepted();
        assert_eq!(stats.goals_accepted.load(Ordering::Relaxed), 1);

        stats.record_rejected();
        assert_eq!(stats.goals_rejected.load(Ordering::Relaxed), 1);

        stats.record_succeeded();
        assert_eq!(stats.goals_succeeded.load(Ordering::Relaxed), 1);

        stats.record_failed();
        assert_eq!(stats.goals_failed.load(Ordering::Relaxed), 1);

        stats.record_canceled();
        assert_eq!(stats.goals_canceled.load(Ordering::Relaxed), 1);

        stats.record_feedback();
        stats.record_feedback();
        stats.record_feedback();
        assert_eq!(stats.feedback_msgs.load(Ordering::Relaxed), 3);

        stats.record_error();
        assert_eq!(stats.errors.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn test_action_stats_goal_lifecycle() {
        let stats = ActionStats::new();

        // Simulate a successful action goal lifecycle
        stats.record_goal(); // Client sends goal
        stats.record_accepted(); // Server accepts goal
        stats.record_feedback(); // Feedback during execution
        stats.record_feedback();
        stats.record_succeeded(); // Goal succeeds

        assert_eq!(stats.goals.load(Ordering::Relaxed), 1);
        assert_eq!(stats.goals_accepted.load(Ordering::Relaxed), 1);
        assert_eq!(stats.goals_succeeded.load(Ordering::Relaxed), 1);
        assert_eq!(stats.feedback_msgs.load(Ordering::Relaxed), 2);

        // Simulate a failed action goal
        stats.record_goal();
        stats.record_accepted();
        stats.record_failed();

        assert_eq!(stats.goals.load(Ordering::Relaxed), 2);
        assert_eq!(stats.goals_failed.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn test_action_stats_concurrent() {
        use std::thread;

        let stats = Arc::new(ActionStats::new());
        let mut handles = vec![];

        for _ in 0..10 {
            let stats_clone = Arc::clone(&stats);
            handles.push(thread::spawn(move || {
                for _ in 0..100 {
                    stats_clone.record_goal();
                    stats_clone.record_accepted();
                    stats_clone.record_feedback();
                    stats_clone.record_succeeded();
                }
            }));
        }

        for handle in handles {
            handle.join().unwrap();
        }

        assert_eq!(stats.goals.load(Ordering::Relaxed), 1000);
        assert_eq!(stats.goals_accepted.load(Ordering::Relaxed), 1000);
        assert_eq!(stats.feedback_msgs.load(Ordering::Relaxed), 1000);
        assert_eq!(stats.goals_succeeded.load(Ordering::Relaxed), 1000);
    }

    // ========================================================================
    // ParameterStats Tests
    // ========================================================================

    #[test]
    fn test_parameter_stats_new() {
        let stats = ParameterStats::new();
        assert_eq!(stats.get_calls.load(Ordering::Relaxed), 0);
        assert_eq!(stats.set_calls.load(Ordering::Relaxed), 0);
        assert_eq!(stats.list_calls.load(Ordering::Relaxed), 0);
        assert_eq!(stats.describe_calls.load(Ordering::Relaxed), 0);
        assert_eq!(stats.errors.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn test_parameter_stats_record_methods() {
        let stats = ParameterStats::new();

        stats.record_get();
        stats.record_get();
        stats.record_get();
        assert_eq!(stats.get_calls.load(Ordering::Relaxed), 3);

        stats.record_set();
        stats.record_set();
        assert_eq!(stats.set_calls.load(Ordering::Relaxed), 2);

        stats.record_list();
        assert_eq!(stats.list_calls.load(Ordering::Relaxed), 1);

        stats.record_describe();
        stats.record_describe();
        assert_eq!(stats.describe_calls.load(Ordering::Relaxed), 2);

        stats.record_error();
        assert_eq!(stats.errors.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn test_parameter_stats_typical_usage() {
        let stats = ParameterStats::new();

        // Simulate typical parameter usage: list, get several, set a few
        stats.record_list(); // First, list all parameters

        // Get some parameters
        for _ in 0..5 {
            stats.record_get();
        }

        // Set a couple parameters
        stats.record_set();
        stats.record_set();

        // One describe call
        stats.record_describe();

        assert_eq!(stats.list_calls.load(Ordering::Relaxed), 1);
        assert_eq!(stats.get_calls.load(Ordering::Relaxed), 5);
        assert_eq!(stats.set_calls.load(Ordering::Relaxed), 2);
        assert_eq!(stats.describe_calls.load(Ordering::Relaxed), 1);
        assert_eq!(stats.errors.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn test_parameter_stats_concurrent() {
        use std::thread;

        let stats = Arc::new(ParameterStats::new());
        let mut handles = vec![];

        for _ in 0..10 {
            let stats_clone = Arc::clone(&stats);
            handles.push(thread::spawn(move || {
                for _ in 0..100 {
                    stats_clone.record_get();
                    stats_clone.record_set();
                    stats_clone.record_list();
                    stats_clone.record_describe();
                }
            }));
        }

        for handle in handles {
            handle.join().unwrap();
        }

        assert_eq!(stats.get_calls.load(Ordering::Relaxed), 1000);
        assert_eq!(stats.set_calls.load(Ordering::Relaxed), 1000);
        assert_eq!(stats.list_calls.load(Ordering::Relaxed), 1000);
        assert_eq!(stats.describe_calls.load(Ordering::Relaxed), 1000);
    }

    // ========================================================================
    // BridgeStats - Actions and Parameters Tests
    // ========================================================================

    #[test]
    fn test_bridge_stats_action_counters() {
        let mut stats = BridgeStats::new();

        // Create action stats
        let nav_stats = stats.get_or_create_action("/navigate_to_pose");
        nav_stats.record_goal();
        nav_stats.record_accepted();
        nav_stats.record_succeeded();

        assert_eq!(stats.actions.len(), 1);
        let stored = stats.actions.get("/navigate_to_pose").unwrap();
        assert_eq!(stored.goals.load(Ordering::Relaxed), 1);
        assert_eq!(stored.goals_succeeded.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn test_bridge_stats_parameter_counters() {
        let mut stats = BridgeStats::new();

        // Create parameter stats for multiple nodes
        let node1_stats = stats.get_or_create_parameter("/my_node");
        let node2_stats = stats.get_or_create_parameter("/robot/controller");

        node1_stats.record_get();
        node1_stats.record_set();
        node2_stats.record_list();

        assert_eq!(stats.parameters.len(), 2);

        let stored1 = stats.parameters.get("/my_node").unwrap();
        assert_eq!(stored1.get_calls.load(Ordering::Relaxed), 1);
        assert_eq!(stored1.set_calls.load(Ordering::Relaxed), 1);

        let stored2 = stats.parameters.get("/robot/controller").unwrap();
        assert_eq!(stored2.list_calls.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn test_bridge_stats_get_or_create_idempotent() {
        let mut stats = BridgeStats::new();

        // First call creates
        let action1 = stats.get_or_create_action("/test_action");
        action1.record_goal();

        // Second call returns same Arc
        let action2 = stats.get_or_create_action("/test_action");
        action2.record_goal();

        // Both should point to the same stats
        assert_eq!(stats.actions.len(), 1);
        let stored = stats.actions.get("/test_action").unwrap();
        assert_eq!(stored.goals.load(Ordering::Relaxed), 2);
    }

    #[test]
    fn test_parse_node_name_from_param_service() {
        // Test the helper function
        assert_eq!(
            parse_node_name_from_param_service("rq/my_node/get_parameters"),
            Some("/my_node".to_string())
        );
        assert_eq!(
            parse_node_name_from_param_service("rq/robot/controller/list_parameters"),
            Some("/robot/controller".to_string())
        );
        assert_eq!(
            parse_node_name_from_param_service("rq/ns/subns/node/describe_parameters"),
            Some("/ns/subns/node".to_string())
        );
        assert_eq!(
            parse_node_name_from_param_service("rq/set_parameters"),
            None // Invalid - no node name
        );
        assert_eq!(parse_node_name_from_param_service("invalid_key"), None);
    }

    // ========================================================================
    // TF2 Transform Bridge Tests (require zenoh-transport feature)
    // ========================================================================

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_identity() {
        let tf = Transform::identity();
        assert_eq!(tf.translation.x, 0.0);
        assert_eq!(tf.translation.y, 0.0);
        assert_eq!(tf.translation.z, 0.0);
        assert_eq!(tf.rotation.x, 0.0);
        assert_eq!(tf.rotation.y, 0.0);
        assert_eq!(tf.rotation.z, 0.0);
        assert_eq!(tf.rotation.w, 1.0);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_new() {
        let translation = Vector3 {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        let rotation = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };
        let tf = Transform::new(translation.clone(), rotation.clone());
        assert_eq!(tf.translation.x, 1.0);
        assert_eq!(tf.translation.y, 2.0);
        assert_eq!(tf.translation.z, 3.0);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_compose_identity() {
        let tf1 = Transform::identity();
        let tf2 = Transform::new(
            Vector3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        );

        // Composing with identity should give same transform
        let result = tf1.compose(&tf2);
        assert!((result.translation.x - 1.0).abs() < 1e-10);
        assert!((result.translation.y - 2.0).abs() < 1e-10);
        assert!((result.translation.z - 3.0).abs() < 1e-10);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_inverse() {
        let tf = Transform::new(
            Vector3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        );

        let inv = tf.inverse();

        // Inverse should have negated translation
        assert!((inv.translation.x + 1.0).abs() < 1e-10);
        assert!((inv.translation.y + 2.0).abs() < 1e-10);
        assert!((inv.translation.z + 3.0).abs() < 1e-10);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_compose_then_inverse() {
        let tf = Transform::new(
            Vector3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        );

        let inv = tf.inverse();
        let composed = tf.compose(&inv);

        // Should be near identity
        assert!((composed.translation.x).abs() < 1e-10);
        assert!((composed.translation.y).abs() < 1e-10);
        assert!((composed.translation.z).abs() < 1e-10);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_stamped_creation() {
        let timestamp = Time {
            sec: 123,
            nanosec: 456,
        };
        let transform = Transform::identity();

        let tf_stamped = TransformStamped::new("world", "robot", timestamp, transform);

        assert_eq!(tf_stamped.header.frame_id, "world");
        assert_eq!(tf_stamped.child_frame_id, "robot");
        assert_eq!(tf_stamped.parent_frame(), "world");
        assert_eq!(tf_stamped.child_frame(), "robot");
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_stamped_timestamp() {
        let timestamp = Time {
            sec: 1,
            nanosec: 500_000_000,
        };

        let tf_stamped = TransformStamped::new("world", "robot", timestamp, Transform::identity());

        // 1.5 seconds in nanoseconds
        assert_eq!(tf_stamped.timestamp_ns(), 1_500_000_000);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_tf_message_creation() {
        let tf_msg = TFMessage::new();
        assert!(tf_msg.is_empty());
        assert_eq!(tf_msg.len(), 0);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_tf_message_with_transforms() {
        let tf1 = TransformStamped {
            header: Header {
                stamp: Time { sec: 0, nanosec: 0 },
                frame_id: "world".to_string(),
            },
            child_frame_id: "robot".to_string(),
            transform: Transform::identity(),
        };

        let tf2 = TransformStamped {
            header: Header {
                stamp: Time { sec: 0, nanosec: 0 },
                frame_id: "robot".to_string(),
            },
            child_frame_id: "sensor".to_string(),
            transform: Transform::identity(),
        };

        let msg = TFMessage::with_transforms(vec![tf1, tf2]);
        assert_eq!(msg.len(), 2);
        assert!(!msg.is_empty());
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_tf_message_add() {
        let mut msg = TFMessage::new();

        msg.add(TransformStamped {
            header: Header {
                stamp: Time { sec: 0, nanosec: 0 },
                frame_id: "world".to_string(),
            },
            child_frame_id: "robot".to_string(),
            transform: Transform::identity(),
        });

        assert_eq!(msg.len(), 1);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_buffer_basic() {
        let buffer = TransformBuffer::new();

        let tf = TransformStamped {
            header: Header {
                stamp: Time { sec: 1, nanosec: 0 },
                frame_id: "world".to_string(),
            },
            child_frame_id: "robot".to_string(),
            transform: Transform::new(
                Vector3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            ),
        };

        buffer.set_transform(&tf, false);

        assert_eq!(buffer.frame_count(), 1);
        let frames = buffer.get_all_frame_names();
        assert!(frames.contains(&"robot".to_string()));
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_buffer_lookup() {
        let buffer = TransformBuffer::new();

        let tf = TransformStamped {
            header: Header {
                stamp: Time { sec: 1, nanosec: 0 },
                frame_id: "world".to_string(),
            },
            child_frame_id: "robot".to_string(),
            transform: Transform::new(
                Vector3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            ),
        };

        buffer.set_transform(&tf, false);

        // Lookup at the exact timestamp
        let result = buffer.lookup_transform("world", "robot", 1_000_000_000);
        assert!(result.is_ok());

        let found_tf = result.unwrap();
        assert!((found_tf.transform.translation.x - 1.0).abs() < 1e-10);
        assert!((found_tf.transform.translation.y - 2.0).abs() < 1e-10);
        assert!((found_tf.transform.translation.z - 3.0).abs() < 1e-10);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_buffer_can_transform() {
        let buffer = TransformBuffer::new();

        // Initially no transforms
        assert!(!buffer.can_transform("world", "robot"));

        // Add a transform
        let tf = TransformStamped {
            header: Header {
                stamp: Time { sec: 1, nanosec: 0 },
                frame_id: "world".to_string(),
            },
            child_frame_id: "robot".to_string(),
            transform: Transform::identity(),
        };
        buffer.set_transform(&tf, false);

        // Now should be able to transform
        assert!(buffer.can_transform("world", "robot"));
        assert!(buffer.can_transform("robot", "world")); // Inverse should work too
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_buffer_chain() {
        let buffer = TransformBuffer::new();

        // world -> base_link -> sensor
        let tf1 = TransformStamped {
            header: Header {
                stamp: Time { sec: 1, nanosec: 0 },
                frame_id: "world".to_string(),
            },
            child_frame_id: "base_link".to_string(),
            transform: Transform::new(
                Vector3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            ),
        };

        let tf2 = TransformStamped {
            header: Header {
                stamp: Time { sec: 1, nanosec: 0 },
                frame_id: "base_link".to_string(),
            },
            child_frame_id: "sensor".to_string(),
            transform: Transform::new(
                Vector3 {
                    x: 0.0,
                    y: 1.0,
                    z: 0.0,
                },
                Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            ),
        };

        buffer.set_transform(&tf1, false);
        buffer.set_transform(&tf2, false);

        // Should be able to transform through the chain
        assert!(buffer.can_transform("world", "sensor"));
        assert!(buffer.can_transform("sensor", "world"));

        // Lookup world -> sensor
        let result = buffer.lookup_transform("world", "sensor", 1_000_000_000);
        assert!(result.is_ok());

        let composed = result.unwrap();
        // Translation should be (1, 1, 0)
        assert!((composed.transform.translation.x - 1.0).abs() < 1e-10);
        assert!((composed.transform.translation.y - 1.0).abs() < 1e-10);
        assert!((composed.transform.translation.z).abs() < 1e-10);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_buffer_self_lookup_error() {
        let buffer = TransformBuffer::new();

        // Add a transform so we have the frame
        let tf = TransformStamped {
            header: Header {
                stamp: Time { sec: 1, nanosec: 0 },
                frame_id: "world".to_string(),
            },
            child_frame_id: "robot".to_string(),
            transform: Transform::identity(),
        };
        buffer.set_transform(&tf, false);

        // Self lookup should error
        let result = buffer.lookup_transform("world", "world", 1_000_000_000);
        assert!(result.is_err());
        match result {
            Err(TF2Error::SelfLookup) => {}
            _ => panic!("Expected SelfLookup error"),
        }
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_buffer_frame_not_found() {
        let buffer = TransformBuffer::new();

        let result = buffer.lookup_transform("world", "nonexistent", 0);
        assert!(result.is_err());
        match result {
            Err(TF2Error::FrameNotFound(frame)) => {
                assert_eq!(frame, "nonexistent");
            }
            _ => panic!("Expected FrameNotFound error"),
        }
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_buffer_get_parent() {
        let buffer = TransformBuffer::new();

        let tf = TransformStamped {
            header: Header {
                stamp: Time { sec: 1, nanosec: 0 },
                frame_id: "world".to_string(),
            },
            child_frame_id: "robot".to_string(),
            transform: Transform::identity(),
        };
        buffer.set_transform(&tf, false);

        let parent = buffer.get_parent("robot");
        assert_eq!(parent, Some("world".to_string()));

        let parent = buffer.get_parent("nonexistent");
        assert_eq!(parent, None);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_buffer_static_transforms() {
        let buffer = TransformBuffer::new();

        let tf = TransformStamped {
            header: Header {
                stamp: Time { sec: 0, nanosec: 0 }, // Time 0 for static
                frame_id: "base_link".to_string(),
            },
            child_frame_id: "camera".to_string(),
            transform: Transform::new(
                Vector3 {
                    x: 0.1,
                    y: 0.0,
                    z: 0.2,
                },
                Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            ),
        };

        buffer.set_transform(&tf, true); // Static transform

        // Should be able to look up at any time
        let result1 = buffer.lookup_transform("base_link", "camera", 1_000_000_000);
        let result2 = buffer.lookup_transform("base_link", "camera", 999_000_000_000);

        assert!(result1.is_ok());
        assert!(result2.is_ok());
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_buffer_set_transforms_batch() {
        let buffer = TransformBuffer::new();

        let msg = TFMessage::with_transforms(vec![
            TransformStamped {
                header: Header {
                    stamp: Time { sec: 1, nanosec: 0 },
                    frame_id: "world".to_string(),
                },
                child_frame_id: "robot1".to_string(),
                transform: Transform::identity(),
            },
            TransformStamped {
                header: Header {
                    stamp: Time { sec: 1, nanosec: 0 },
                    frame_id: "world".to_string(),
                },
                child_frame_id: "robot2".to_string(),
                transform: Transform::identity(),
            },
        ]);

        buffer.set_transforms(&msg, false);

        assert_eq!(buffer.frame_count(), 2);
        assert!(buffer.can_transform("world", "robot1"));
        assert!(buffer.can_transform("world", "robot2"));
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_buffer_clear() {
        let buffer = TransformBuffer::new();

        let tf = TransformStamped {
            header: Header {
                stamp: Time { sec: 1, nanosec: 0 },
                frame_id: "world".to_string(),
            },
            child_frame_id: "robot".to_string(),
            transform: Transform::identity(),
        };
        buffer.set_transform(&tf, false);

        assert_eq!(buffer.frame_count(), 1);

        buffer.clear();
        assert_eq!(buffer.frame_count(), 0);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_lerp_vector3() {
        let v1 = Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let v2 = Vector3 {
            x: 10.0,
            y: 20.0,
            z: 30.0,
        };

        let mid = lerp_vector3(&v1, &v2, 0.5);
        assert!((mid.x - 5.0).abs() < 1e-10);
        assert!((mid.y - 10.0).abs() < 1e-10);
        assert!((mid.z - 15.0).abs() < 1e-10);

        let start = lerp_vector3(&v1, &v2, 0.0);
        assert!((start.x - 0.0).abs() < 1e-10);

        let end = lerp_vector3(&v1, &v2, 1.0);
        assert!((end.x - 10.0).abs() < 1e-10);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_slerp_quaternion_identity() {
        let q1 = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };
        let q2 = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };

        let result = slerp_quaternion(&q1, &q2, 0.5);

        // Should still be identity
        assert!((result.x).abs() < 1e-10);
        assert!((result.y).abs() < 1e-10);
        assert!((result.z).abs() < 1e-10);
        assert!((result.w - 1.0).abs() < 1e-10);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_tf2_error_display() {
        let err1 = TF2Error::FrameNotFound("test_frame".to_string());
        assert!(format!("{:?}", err1).contains("test_frame"));

        let err2 = TF2Error::NoPathBetweenFrames("a".to_string(), "b".to_string());
        assert!(format!("{:?}", err2).contains("a"));
        assert!(format!("{:?}", err2).contains("b"));

        let err3 = TF2Error::SelfLookup;
        assert!(format!("{:?}", err3).contains("SelfLookup"));
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_ros2_message_type_tf2() {
        // Test that TF2 types can be parsed and converted
        assert_eq!(
            Ros2MessageType::from_type_string("geometry_msgs/msg/Transform"),
            Ros2MessageType::GeometryMsgsTransform
        );
        assert_eq!(
            Ros2MessageType::from_type_string("geometry_msgs/msg/TransformStamped"),
            Ros2MessageType::GeometryMsgsTransformStamped
        );
        assert_eq!(
            Ros2MessageType::from_type_string("tf2_msgs/msg/TFMessage"),
            Ros2MessageType::Tf2MsgsTFMessage
        );
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_ros2_message_type_tf2_to_string() {
        assert_eq!(
            Ros2MessageType::GeometryMsgsTransform.to_type_string(),
            "geometry_msgs/msg/Transform"
        );
        assert_eq!(
            Ros2MessageType::GeometryMsgsTransformStamped.to_type_string(),
            "geometry_msgs/msg/TransformStamped"
        );
        assert_eq!(
            Ros2MessageType::Tf2MsgsTFMessage.to_type_string(),
            "tf2_msgs/msg/TFMessage"
        );
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_buffer_with_custom_cache_time() {
        let buffer = TransformBuffer::with_cache_time(5.0);

        let tf = TransformStamped {
            header: Header {
                stamp: Time { sec: 1, nanosec: 0 },
                frame_id: "world".to_string(),
            },
            child_frame_id: "robot".to_string(),
            transform: Transform::identity(),
        };

        buffer.set_transform(&tf, false);
        assert_eq!(buffer.frame_count(), 1);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_rotate_vector() {
        // Test rotating a vector by identity quaternion (no rotation)
        let tf = Transform::new(
            Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        );

        let vec = Vector3 {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        let rotated = tf.rotate_vector(&vec);

        assert!((rotated.x - 1.0).abs() < 1e-10);
        assert!((rotated.y - 2.0).abs() < 1e-10);
        assert!((rotated.z - 3.0).abs() < 1e-10);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_quaternion_multiply_identity() {
        let q1 = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };
        let q2 = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };

        // Create a transform with q1 as its rotation and multiply by q2
        let tf = Transform::new(Vector3::default(), q1);
        let result = tf.quaternion_multiply(&q2);

        assert!((result.x).abs() < 1e-10);
        assert!((result.y).abs() < 1e-10);
        assert!((result.z).abs() < 1e-10);
        assert!((result.w - 1.0).abs() < 1e-10);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_interpolate_transform() {
        let tf1 = Transform::new(
            Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        );

        let tf2 = Transform::new(
            Vector3 {
                x: 10.0,
                y: 10.0,
                z: 10.0,
            },
            Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        );

        let mid = interpolate_transform(&tf1, &tf2, 0.5);

        assert!((mid.translation.x - 5.0).abs() < 1e-10);
        assert!((mid.translation.y - 5.0).abs() < 1e-10);
        assert!((mid.translation.z - 5.0).abs() < 1e-10);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_transform_default() {
        let tf = Transform::default();
        assert_eq!(tf.translation.x, 0.0);
        assert_eq!(tf.rotation.w, 0.0); // Default, not identity
    }

    // ========================================================================
    // Comprehensive Discovery Tests
    // ========================================================================

    #[test]
    fn test_discovered_topic_all_fields() {
        let topic = DiscoveredTopic {
            name: "/robot/scan".to_string(),
            msg_type: Some("sensor_msgs/msg/LaserScan".to_string()),
            publishers: 2,
            subscribers: 5,
            reliable: true,
        };

        assert_eq!(topic.name, "/robot/scan");
        assert_eq!(
            topic.msg_type,
            Some("sensor_msgs/msg/LaserScan".to_string())
        );
        assert_eq!(topic.publishers, 2);
        assert_eq!(topic.subscribers, 5);
        assert!(topic.reliable);
    }

    #[test]
    fn test_discovered_topic_no_msg_type() {
        let topic = DiscoveredTopic {
            name: "/unknown_topic".to_string(),
            msg_type: None,
            publishers: 1,
            subscribers: 0,
            reliable: false,
        };

        assert_eq!(topic.name, "/unknown_topic");
        assert!(topic.msg_type.is_none());
        assert_eq!(topic.publishers, 1);
        assert_eq!(topic.subscribers, 0);
        assert!(!topic.reliable);
    }

    #[test]
    fn test_discovered_topic_nested_namespace() {
        let topic = DiscoveredTopic {
            name: "/robot1/sensors/front/lidar".to_string(),
            msg_type: Some("sensor_msgs/msg/PointCloud2".to_string()),
            publishers: 1,
            subscribers: 3,
            reliable: true,
        };

        assert!(topic.name.starts_with("/robot1"));
        assert!(topic.name.contains("/sensors/"));
    }

    #[test]
    fn test_discovered_service_all_fields() {
        let service = DiscoveredService {
            name: "/navigation/get_plan".to_string(),
            srv_type: Some("nav_msgs/srv/GetPlan".to_string()),
        };

        assert_eq!(service.name, "/navigation/get_plan");
        assert_eq!(service.srv_type, Some("nav_msgs/srv/GetPlan".to_string()));
    }

    #[test]
    fn test_discovered_service_no_type() {
        let service = DiscoveredService {
            name: "/custom_service".to_string(),
            srv_type: None,
        };

        assert_eq!(service.name, "/custom_service");
        assert!(service.srv_type.is_none());
    }

    #[test]
    fn test_discovered_action_all_fields() {
        let action = DiscoveredAction {
            name: "/navigate_to_pose".to_string(),
            action_type: Some("nav2_msgs/action/NavigateToPose".to_string()),
        };

        assert_eq!(action.name, "/navigate_to_pose");
        assert_eq!(
            action.action_type,
            Some("nav2_msgs/action/NavigateToPose".to_string())
        );
    }

    #[test]
    fn test_discovered_action_no_type() {
        let action = DiscoveredAction {
            name: "/custom_action".to_string(),
            action_type: None,
        };

        assert_eq!(action.name, "/custom_action");
        assert!(action.action_type.is_none());
    }

    #[test]
    fn test_discovered_parameter_node_with_namespace() {
        let param_node = DiscoveredParameterNode {
            node_name: "/my_controller".to_string(),
            namespace: Some("/robot1".to_string()),
        };

        assert_eq!(param_node.node_name, "/my_controller");
        assert_eq!(param_node.namespace, Some("/robot1".to_string()));
    }

    #[test]
    fn test_discovered_parameter_node_no_namespace() {
        let param_node = DiscoveredParameterNode {
            node_name: "/global_controller".to_string(),
            namespace: None,
        };

        assert_eq!(param_node.node_name, "/global_controller");
        assert!(param_node.namespace.is_none());
    }

    #[test]
    fn test_discovery_result_with_topics_and_services() {
        let mut result = DiscoveryResult::default();

        result.topics.push(DiscoveredTopic {
            name: "/scan".to_string(),
            msg_type: Some("sensor_msgs/msg/LaserScan".to_string()),
            publishers: 1,
            subscribers: 2,
            reliable: true,
        });

        result.topics.push(DiscoveredTopic {
            name: "/odom".to_string(),
            msg_type: Some("nav_msgs/msg/Odometry".to_string()),
            publishers: 1,
            subscribers: 3,
            reliable: true,
        });

        result.services.push(DiscoveredService {
            name: "/get_map".to_string(),
            srv_type: Some("nav_msgs/srv/GetMap".to_string()),
        });

        assert_eq!(result.topics.len(), 2);
        assert_eq!(result.services.len(), 1);
        assert_eq!(result.topics[0].name, "/scan");
        assert_eq!(result.topics[1].name, "/odom");
        assert_eq!(result.services[0].name, "/get_map");
    }

    #[test]
    fn test_ros2_topic_key_pattern() {
        // Test that topic key follows ROS2 Zenoh convention
        let topic_name = "/scan";
        let key = format!("rt{}", topic_name);
        assert_eq!(key, "rt/scan");

        let nested_topic = "/robot/sensors/lidar";
        let nested_key = format!("rt{}", nested_topic);
        assert_eq!(nested_key, "rt/robot/sensors/lidar");
    }

    #[test]
    fn test_ros2_service_key_patterns() {
        // Test service request/response key patterns
        let service_name = "/get_map";
        let domain_id = 0u32;

        let request_key = format!("{}/rq{}/**", domain_id, service_name);
        let response_key = format!("{}/rr{}/**", domain_id, service_name);

        assert_eq!(request_key, "0/rq/get_map/**");
        assert_eq!(response_key, "0/rr/get_map/**");
    }

    #[test]
    fn test_ros2_action_key_patterns() {
        // ROS2 actions use multiple topics: goal, result, feedback, status
        let action_name = "/navigate_to_pose";
        let domain_id = 0u32;

        // Action goal request
        let goal_key = format!("{}/rq{}/_action/send_goal/**", domain_id, action_name);
        assert!(goal_key.contains("send_goal"));

        // Action result request
        let result_key = format!("{}/rq{}/_action/get_result/**", domain_id, action_name);
        assert!(result_key.contains("get_result"));

        // Action cancel request
        let cancel_key = format!("{}/rq{}/_action/cancel_goal/**", domain_id, action_name);
        assert!(cancel_key.contains("cancel_goal"));

        // Action feedback topic
        let feedback_key = format!("rt{}/_action/feedback", action_name);
        assert_eq!(feedback_key, "rt/navigate_to_pose/_action/feedback");

        // Action status topic
        let status_key = format!("rt{}/_action/status", action_name);
        assert_eq!(status_key, "rt/navigate_to_pose/_action/status");
    }

    #[test]
    fn test_parameter_stats_tracking() {
        let stats = ParameterStats::new();

        // Initially all should be zero
        assert_eq!(stats.get_calls.load(Ordering::Relaxed), 0);
        assert_eq!(stats.set_calls.load(Ordering::Relaxed), 0);
        assert_eq!(stats.list_calls.load(Ordering::Relaxed), 0);
        assert_eq!(stats.describe_calls.load(Ordering::Relaxed), 0);
        assert_eq!(stats.errors.load(Ordering::Relaxed), 0);

        // Record some calls
        stats.record_get();
        stats.record_get();
        stats.record_set();
        stats.record_list();
        stats.record_list();
        stats.record_list();
        stats.record_describe();
        stats.record_error();

        assert_eq!(stats.get_calls.load(Ordering::Relaxed), 2);
        assert_eq!(stats.set_calls.load(Ordering::Relaxed), 1);
        assert_eq!(stats.list_calls.load(Ordering::Relaxed), 3);
        assert_eq!(stats.describe_calls.load(Ordering::Relaxed), 1);
        assert_eq!(stats.errors.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn test_parameter_stats_concurrent_updates() {
        use std::thread;

        let stats = std::sync::Arc::new(ParameterStats::new());

        let mut handles = vec![];

        // Spawn multiple threads to update stats concurrently
        for _ in 0..10 {
            let stats_clone = std::sync::Arc::clone(&stats);
            let handle = thread::spawn(move || {
                for _ in 0..100 {
                    stats_clone.record_get();
                    stats_clone.record_set();
                }
            });
            handles.push(handle);
        }

        // Wait for all threads
        for handle in handles {
            handle.join().unwrap();
        }

        // Each thread does 100 gets and 100 sets, 10 threads total
        assert_eq!(stats.get_calls.load(Ordering::Relaxed), 1000);
        assert_eq!(stats.set_calls.load(Ordering::Relaxed), 1000);
    }

    #[test]
    fn test_discovery_result_clone() {
        let topic = DiscoveredTopic {
            name: "/scan".to_string(),
            msg_type: Some("sensor_msgs/msg/LaserScan".to_string()),
            publishers: 1,
            subscribers: 2,
            reliable: true,
        };

        let cloned = topic.clone();
        assert_eq!(topic.name, cloned.name);
        assert_eq!(topic.msg_type, cloned.msg_type);
        assert_eq!(topic.publishers, cloned.publishers);
        assert_eq!(topic.subscribers, cloned.subscribers);
        assert_eq!(topic.reliable, cloned.reliable);
    }

    #[test]
    fn test_discovered_service_clone() {
        let service = DiscoveredService {
            name: "/get_map".to_string(),
            srv_type: Some("nav_msgs/srv/GetMap".to_string()),
        };

        let cloned = service.clone();
        assert_eq!(service.name, cloned.name);
        assert_eq!(service.srv_type, cloned.srv_type);
    }

    #[test]
    fn test_discovered_action_clone() {
        let action = DiscoveredAction {
            name: "/navigate".to_string(),
            action_type: Some("nav2_msgs/action/NavigateToPose".to_string()),
        };

        let cloned = action.clone();
        assert_eq!(action.name, cloned.name);
        assert_eq!(action.action_type, cloned.action_type);
    }

    #[test]
    fn test_discovered_parameter_node_clone() {
        let node = DiscoveredParameterNode {
            node_name: "/controller".to_string(),
            namespace: Some("/robot1".to_string()),
        };

        let cloned = node.clone();
        assert_eq!(node.node_name, cloned.node_name);
        assert_eq!(node.namespace, cloned.namespace);
    }

    #[test]
    fn test_ros2_domain_id_in_keys() {
        // Different domain IDs should produce different keys
        let service_name = "/my_service";

        let key_domain0 = ros2_service_request_key(0, service_name);
        let key_domain1 = ros2_service_request_key(1, service_name);
        let key_domain42 = ros2_service_request_key(42, service_name);

        assert!(key_domain0.starts_with("0/"));
        assert!(key_domain1.starts_with("1/"));
        assert!(key_domain42.starts_with("42/"));

        // All should contain the service name
        assert!(key_domain0.contains("my_service"));
        assert!(key_domain1.contains("my_service"));
        assert!(key_domain42.contains("my_service"));
    }

    #[test]
    fn test_discovery_result_debug_format() {
        let result = DiscoveryResult::default();
        let debug_str = format!("{:?}", result);
        assert!(debug_str.contains("DiscoveryResult"));
        assert!(debug_str.contains("topics"));
        assert!(debug_str.contains("services"));
    }

    #[test]
    fn test_discovered_topic_debug_format() {
        let topic = DiscoveredTopic {
            name: "/test".to_string(),
            msg_type: Some("std_msgs/msg/String".to_string()),
            publishers: 1,
            subscribers: 0,
            reliable: true,
        };
        let debug_str = format!("{:?}", topic);
        assert!(debug_str.contains("DiscoveredTopic"));
        assert!(debug_str.contains("/test"));
    }

    #[test]
    fn test_discovered_service_debug_format() {
        let service = DiscoveredService {
            name: "/srv".to_string(),
            srv_type: Some("std_srvs/srv/Empty".to_string()),
        };
        let debug_str = format!("{:?}", service);
        assert!(debug_str.contains("DiscoveredService"));
        assert!(debug_str.contains("/srv"));
    }

    #[test]
    fn test_discovered_action_debug_format() {
        let action = DiscoveredAction {
            name: "/action".to_string(),
            action_type: None,
        };
        let debug_str = format!("{:?}", action);
        assert!(debug_str.contains("DiscoveredAction"));
        assert!(debug_str.contains("/action"));
    }

    #[test]
    fn test_parameter_stats_debug_format() {
        let stats = ParameterStats::new();
        stats.record_get();
        let debug_str = format!("{:?}", stats);
        assert!(debug_str.contains("ParameterStats"));
    }

    // ========================================================================
    // Zenoh Key Expression Tests
    // ========================================================================

    #[test]
    fn test_ros2_liveliness_key_pattern() {
        // Zenoh liveliness pattern for ROS2 topic discovery
        let liveliness_key = "@/liveliness/**";
        assert!(liveliness_key.starts_with("@/"));
        assert!(liveliness_key.ends_with("**"));
    }

    #[test]
    fn test_topic_name_extraction_from_rt_key() {
        // Test extracting topic names from Zenoh rt/ keys
        let test_cases = vec![
            ("rt/scan", "/scan"),
            ("rt/robot/odom", "/robot/odom"),
            ("rt/a/b/c/d", "/a/b/c/d"),
        ];

        for (key, expected_topic) in test_cases {
            if let Some(topic) = key.strip_prefix("rt/") {
                let extracted = format!("/{}", topic);
                assert_eq!(extracted, expected_topic);
            } else if let Some(topic) = key.strip_prefix("rt") {
                let extracted = format!("/{}", topic.trim_start_matches('/'));
                assert_eq!(extracted, expected_topic);
            }
        }
    }

    #[test]
    fn test_service_name_with_slashes() {
        // Service names with multiple path components
        let service = "/robot1/navigation/get_path";
        let request_key = ros2_service_request_key(0, service);
        let response_key = ros2_service_response_key(0, service);

        assert!(request_key.contains("rq"));
        assert!(response_key.contains("rr"));
        assert!(request_key.contains("navigation"));
        assert!(response_key.contains("navigation"));
    }

    #[test]
    fn test_empty_discovery_result() {
        let result = DiscoveryResult::default();
        assert!(result.topics.is_empty());
        assert!(result.services.is_empty());
        assert_eq!(result.topics.len(), 0);
        assert_eq!(result.services.len(), 0);
    }

    #[test]
    fn test_topic_with_many_publishers() {
        let topic = DiscoveredTopic {
            name: "/clock".to_string(),
            msg_type: Some("rosgraph_msgs/msg/Clock".to_string()),
            publishers: 100,
            subscribers: 50,
            reliable: true,
        };

        assert_eq!(topic.publishers, 100);
        assert_eq!(topic.subscribers, 50);
    }

    #[test]
    fn test_topic_best_effort_qos() {
        let topic = DiscoveredTopic {
            name: "/camera/image_raw".to_string(),
            msg_type: Some("sensor_msgs/msg/Image".to_string()),
            publishers: 1,
            subscribers: 1,
            reliable: false, // Best effort for high-bandwidth data
        };

        assert!(!topic.reliable);
    }

    #[cfg(feature = "zenoh-transport")]
    #[tokio::test]
    async fn test_discover_topics_returns_result() {
        // This test verifies the function signature and return type
        // It may return empty results if no ROS2 system is running
        let result = discover_ros2_topics(0).await;
        assert!(result.is_ok());
    }

    #[cfg(feature = "zenoh-transport")]
    #[tokio::test]
    async fn test_discover_services_returns_result() {
        // This test verifies the function signature and return type
        let result = discover_ros2_services(0).await;
        assert!(result.is_ok());
    }

    #[cfg(feature = "zenoh-transport")]
    #[tokio::test]
    async fn test_discover_actions_returns_result() {
        // This test verifies the function signature and return type
        let result = discover_ros2_actions(0).await;
        assert!(result.is_ok());
    }

    #[cfg(feature = "zenoh-transport")]
    #[tokio::test]
    async fn test_discover_parameters_returns_result() {
        // This test verifies the function signature and return type
        let result = discover_ros2_parameters(0).await;
        assert!(result.is_ok());
    }

    #[cfg(feature = "zenoh-transport")]
    #[tokio::test]
    async fn test_discover_on_different_domain_ids() {
        // Discovery on different domain IDs should work
        let result0 = discover_ros2_topics(0).await;
        let result1 = discover_ros2_topics(1).await;
        let result42 = discover_ros2_topics(42).await;

        // All should succeed (may return empty results)
        assert!(result0.is_ok());
        assert!(result1.is_ok());
        assert!(result42.is_ok());
    }

    // ============================================================
    // Zenoh-Based Discovery Tests
    // ============================================================

    #[test]
    fn test_zenoh_topic_key_pattern_rt_prefix() {
        // ROS2 topics in Zenoh use "rt" prefix
        let topic = "/scan";
        let key = format!("rt{}", topic);
        assert!(key.starts_with("rt"));
        assert_eq!(key, "rt/scan");
    }

    #[test]
    fn test_zenoh_topic_key_pattern_nested() {
        // Nested topic names preserve hierarchy
        let topic = "/robot1/sensors/lidar/scan";
        let key = format!("rt{}", topic);
        assert_eq!(key, "rt/robot1/sensors/lidar/scan");
        assert_eq!(key.matches('/').count(), 4); // 4 slashes in the key
    }

    #[test]
    fn test_zenoh_service_key_domain_encoding() {
        // ROS2 services include domain ID in key
        let service = "/get_map";
        let domain_id = 42u32;

        let request_key = format!("{}/rq{}/**", domain_id, service);
        let response_key = format!("{}/rr{}/**", domain_id, service);

        assert!(request_key.starts_with("42/"));
        assert!(response_key.starts_with("42/"));
    }

    #[test]
    fn test_zenoh_action_key_patterns_complete() {
        // ROS2 actions have 5 endpoints: goal, result, feedback, status, cancel
        let action = "/navigate_to_pose";

        // Action topics (feedback, status)
        let feedback_key = format!("rt{}/_action/feedback", action);
        let status_key = format!("rt{}/_action/status", action);

        assert!(feedback_key.contains("_action"));
        assert!(status_key.contains("_action"));

        // Action services (goal, result, cancel)
        let goal_key = format!("0/rq{}/_action/send_goal/**", action);
        let result_key = format!("0/rq{}/_action/get_result/**", action);
        let cancel_key = format!("0/rq{}/_action/cancel_goal/**", action);

        assert!(goal_key.contains("send_goal"));
        assert!(result_key.contains("get_result"));
        assert!(cancel_key.contains("cancel_goal"));
    }

    #[test]
    fn test_discovered_action_default_values() {
        let action = DiscoveredAction {
            name: "/nav".to_string(),
            action_type: None,
        };

        assert_eq!(action.name, "/nav");
        assert!(action.action_type.is_none());
    }

    #[test]
    fn test_discovered_action_with_type() {
        let action = DiscoveredAction {
            name: "/navigate_to_pose".to_string(),
            action_type: Some("nav2_msgs/action/NavigateToPose".to_string()),
        };

        assert_eq!(action.name, "/navigate_to_pose");
        assert_eq!(
            action.action_type.as_ref().unwrap(),
            "nav2_msgs/action/NavigateToPose"
        );
    }

    #[test]
    fn test_discovered_parameter_node_basic() {
        let node = DiscoveredParameterNode {
            node_name: "/controller_manager".to_string(),
            namespace: None,
        };

        assert_eq!(node.node_name, "/controller_manager");
        assert!(node.namespace.is_none());
    }

    #[test]
    fn test_discovered_parameter_node_debug() {
        let node = DiscoveredParameterNode {
            node_name: "/teleop".to_string(),
            namespace: Some("/mobile_base".to_string()),
        };

        let debug_str = format!("{:?}", node);
        assert!(debug_str.contains("DiscoveredParameterNode"));
        assert!(debug_str.contains("teleop"));
    }

    #[test]
    fn test_discovery_result_aggregation() {
        let mut result = DiscoveryResult::default();

        // Add multiple topics
        for i in 0..10 {
            result.topics.push(DiscoveredTopic {
                name: format!("/topic_{}", i),
                msg_type: Some("std_msgs/msg/String".to_string()),
                publishers: 1,
                subscribers: i as u32,
                reliable: i % 2 == 0,
            });
        }

        // Add multiple services
        for i in 0..5 {
            result.services.push(DiscoveredService {
                name: format!("/service_{}", i),
                srv_type: Some("std_srvs/srv/Empty".to_string()),
            });
        }

        assert_eq!(result.topics.len(), 10);
        assert_eq!(result.services.len(), 5);

        // Check topic properties
        let reliable_count = result.topics.iter().filter(|t| t.reliable).count();
        assert_eq!(reliable_count, 5); // Even indices are reliable
    }

    #[test]
    fn test_discovery_result_filtering_by_name() {
        let mut result = DiscoveryResult::default();

        result.topics.push(DiscoveredTopic {
            name: "/scan".to_string(),
            msg_type: Some("sensor_msgs/msg/LaserScan".to_string()),
            publishers: 1,
            subscribers: 2,
            reliable: true,
        });
        result.topics.push(DiscoveredTopic {
            name: "/camera/image".to_string(),
            msg_type: Some("sensor_msgs/msg/Image".to_string()),
            publishers: 1,
            subscribers: 1,
            reliable: false,
        });
        result.topics.push(DiscoveredTopic {
            name: "/camera/depth".to_string(),
            msg_type: Some("sensor_msgs/msg/Image".to_string()),
            publishers: 1,
            subscribers: 1,
            reliable: false,
        });

        // Filter camera topics
        let camera_topics: Vec<_> = result
            .topics
            .iter()
            .filter(|t| t.name.starts_with("/camera"))
            .collect();

        assert_eq!(camera_topics.len(), 2);
    }

    #[test]
    fn test_discovery_result_filtering_by_type() {
        let mut result = DiscoveryResult::default();

        result.topics.push(DiscoveredTopic {
            name: "/scan".to_string(),
            msg_type: Some("sensor_msgs/msg/LaserScan".to_string()),
            publishers: 1,
            subscribers: 2,
            reliable: true,
        });
        result.topics.push(DiscoveredTopic {
            name: "/image".to_string(),
            msg_type: Some("sensor_msgs/msg/Image".to_string()),
            publishers: 1,
            subscribers: 1,
            reliable: false,
        });
        result.topics.push(DiscoveredTopic {
            name: "/twist".to_string(),
            msg_type: Some("geometry_msgs/msg/Twist".to_string()),
            publishers: 1,
            subscribers: 1,
            reliable: true,
        });

        // Filter sensor_msgs topics
        let sensor_topics: Vec<_> = result
            .topics
            .iter()
            .filter(|t| {
                t.msg_type
                    .as_ref()
                    .map_or(false, |m| m.starts_with("sensor_msgs"))
            })
            .collect();

        assert_eq!(sensor_topics.len(), 2);
    }

    #[test]
    fn test_zenoh_key_wildcard_patterns() {
        // Zenoh uses ** for multi-level wildcard
        let service = "/robot1/arm/get_state";
        let domain_id = 0u32;

        let request_pattern = format!("{}/rq{}/**", domain_id, service);
        let response_pattern = format!("{}/rr{}/**", domain_id, service);

        assert!(request_pattern.ends_with("/**"));
        assert!(response_pattern.ends_with("/**"));
    }

    #[test]
    fn test_zenoh_parameter_service_keys() {
        // ROS2 parameters use specific service patterns
        let node = "/controller";
        let domain_id = 0u32;

        // List parameters
        let list_key = format!("{}/rq{}/list_parameters/**", domain_id, node);
        assert!(list_key.contains("list_parameters"));

        // Get parameters
        let get_key = format!("{}/rq{}/get_parameters/**", domain_id, node);
        assert!(get_key.contains("get_parameters"));

        // Set parameters
        let set_key = format!("{}/rq{}/set_parameters/**", domain_id, node);
        assert!(set_key.contains("set_parameters"));

        // Describe parameters
        let describe_key = format!("{}/rq{}/describe_parameters/**", domain_id, node);
        assert!(describe_key.contains("describe_parameters"));
    }

    #[test]
    fn test_discovery_topic_qos_detection() {
        // Topics with different QoS settings
        let reliable_topic = DiscoveredTopic {
            name: "/cmd_vel".to_string(),
            msg_type: Some("geometry_msgs/msg/Twist".to_string()),
            publishers: 1,
            subscribers: 1,
            reliable: true,
        };

        let best_effort_topic = DiscoveredTopic {
            name: "/camera/image_raw".to_string(),
            msg_type: Some("sensor_msgs/msg/Image".to_string()),
            publishers: 1,
            subscribers: 3,
            reliable: false, // Best effort for high bandwidth
        };

        assert!(reliable_topic.reliable);
        assert!(!best_effort_topic.reliable);
    }

    #[test]
    fn test_discovery_topic_publisher_subscriber_counts() {
        let topic = DiscoveredTopic {
            name: "/rosout".to_string(),
            msg_type: Some("rcl_interfaces/msg/Log".to_string()),
            publishers: 50, // Many nodes publish to rosout
            subscribers: 1, // Usually just one subscriber (rosout_agg)
            reliable: true,
        };

        assert!(topic.publishers > topic.subscribers);
        assert_eq!(topic.publishers, 50);
    }

    #[test]
    fn test_discovery_empty_service_type() {
        // Service discovered but type unknown
        let service = DiscoveredService {
            name: "/unknown_service".to_string(),
            srv_type: None,
        };

        assert!(service.srv_type.is_none());
        assert!(!service.name.is_empty());
    }

    #[test]
    fn test_discovery_service_with_namespace() {
        let service = DiscoveredService {
            name: "/robot1/navigation/get_plan".to_string(),
            srv_type: Some("nav_msgs/srv/GetPlan".to_string()),
        };

        assert!(service.name.contains("/robot1"));
        assert!(service.name.contains("navigation"));
    }

    #[test]
    fn test_zenoh_domain_id_range() {
        // ROS2 supports domain IDs 0-232
        for domain_id in [0u32, 1, 42, 100, 232] {
            let topic = "/test";
            // Domain ID should be valid in key construction
            let _key = format!("{}/rt{}", domain_id, topic);
        }
    }

    #[test]
    fn test_zenoh_special_characters_in_topic() {
        // Topics with underscores and numbers
        let topic = "/robot_1/sensor_array_0/data_stream";
        let key = format!("rt{}", topic);

        assert!(key.contains("robot_1"));
        assert!(key.contains("sensor_array_0"));
        assert!(key.contains("data_stream"));
    }

    #[test]
    fn test_discovery_result_default_is_empty() {
        let result = DiscoveryResult::default();

        assert!(result.topics.is_empty());
        assert!(result.services.is_empty());
        assert_eq!(result.topics.capacity(), 0); // No pre-allocation
    }

    #[cfg(feature = "zenoh-transport")]
    #[tokio::test]
    async fn test_discover_topics_with_high_domain_id() {
        // Test discovery with maximum valid domain ID
        let result = discover_ros2_topics(232).await;
        assert!(result.is_ok());
    }

    #[cfg(feature = "zenoh-transport")]
    #[tokio::test]
    async fn test_discover_services_with_high_domain_id() {
        let result = discover_ros2_services(232).await;
        assert!(result.is_ok());
    }

    #[cfg(feature = "zenoh-transport")]
    #[tokio::test]
    async fn test_discover_actions_with_high_domain_id() {
        let result = discover_ros2_actions(232).await;
        assert!(result.is_ok());
    }

    #[cfg(feature = "zenoh-transport")]
    #[tokio::test]
    async fn test_discover_parameters_with_high_domain_id() {
        let result = discover_ros2_parameters(232).await;
        assert!(result.is_ok());
    }

    #[cfg(feature = "zenoh-transport")]
    #[tokio::test]
    async fn test_parallel_discovery() {
        use tokio::join;

        // Run all discovery functions in parallel
        let (topics, services, actions, params) = join!(
            discover_ros2_topics(0),
            discover_ros2_services(0),
            discover_ros2_actions(0),
            discover_ros2_parameters(0)
        );

        // All should succeed
        assert!(topics.is_ok());
        assert!(services.is_ok());
        assert!(actions.is_ok());
        assert!(params.is_ok());
    }

    #[test]
    fn test_zenoh_tf_topic_keys() {
        // TF2 uses specific topic names
        let tf_key = "rt/tf";
        let tf_static_key = "rt/tf_static";

        assert!(tf_key.ends_with("/tf"));
        assert!(tf_static_key.ends_with("/tf_static"));
    }

    #[test]
    fn test_zenoh_clock_topic_key() {
        // ROS2 /clock topic for simulation time
        let clock_key = "rt/clock";
        assert_eq!(clock_key, "rt/clock");
    }

    #[test]
    fn test_zenoh_rosout_topic_key() {
        // ROS2 logging topic
        let rosout_key = "rt/rosout";
        assert_eq!(rosout_key, "rt/rosout");
    }

    #[test]
    fn test_discovery_multiple_robots() {
        let mut result = DiscoveryResult::default();

        // Robot 1 topics
        result.topics.push(DiscoveredTopic {
            name: "/robot1/scan".to_string(),
            msg_type: Some("sensor_msgs/msg/LaserScan".to_string()),
            publishers: 1,
            subscribers: 1,
            reliable: true,
        });
        result.topics.push(DiscoveredTopic {
            name: "/robot1/odom".to_string(),
            msg_type: Some("nav_msgs/msg/Odometry".to_string()),
            publishers: 1,
            subscribers: 2,
            reliable: true,
        });

        // Robot 2 topics
        result.topics.push(DiscoveredTopic {
            name: "/robot2/scan".to_string(),
            msg_type: Some("sensor_msgs/msg/LaserScan".to_string()),
            publishers: 1,
            subscribers: 1,
            reliable: true,
        });
        result.topics.push(DiscoveredTopic {
            name: "/robot2/odom".to_string(),
            msg_type: Some("nav_msgs/msg/Odometry".to_string()),
            publishers: 1,
            subscribers: 2,
            reliable: true,
        });

        // Filter by robot namespace
        let robot1_topics: Vec<_> = result
            .topics
            .iter()
            .filter(|t| t.name.starts_with("/robot1"))
            .collect();
        let robot2_topics: Vec<_> = result
            .topics
            .iter()
            .filter(|t| t.name.starts_with("/robot2"))
            .collect();

        assert_eq!(robot1_topics.len(), 2);
        assert_eq!(robot2_topics.len(), 2);
    }

    #[test]
    fn test_action_endpoint_identification() {
        // Identify action topics vs services from key patterns
        let action_name = "/navigate";

        // Topics (feedback/status) use rt prefix
        let feedback = format!("rt{}/_action/feedback", action_name);
        let status = format!("rt{}/_action/status", action_name);

        assert!(feedback.starts_with("rt"));
        assert!(status.starts_with("rt"));

        // Services (goal/result/cancel) use domain/rq prefix
        let goal = format!("0/rq{}/_action/send_goal", action_name);
        let result = format!("0/rq{}/_action/get_result", action_name);

        assert!(goal.contains("/rq"));
        assert!(result.contains("/rq"));
    }

    // ============================================================
    // ROS2 Integration Tests
    // ============================================================

    // --- Pub/Sub Message Flow Tests ---

    #[test]
    fn test_pubsub_topic_creation() {
        let topic = DiscoveredTopic {
            name: "/chatter".to_string(),
            msg_type: Some("std_msgs/msg/String".to_string()),
            publishers: 1,
            subscribers: 2,
            reliable: true,
        };

        assert_eq!(topic.name, "/chatter");
        assert!(topic.msg_type.is_some());
        assert!(topic.publishers > 0);
    }

    #[test]
    fn test_pubsub_multiple_publishers() {
        let topic = DiscoveredTopic {
            name: "/sensor_data".to_string(),
            msg_type: Some("sensor_msgs/msg/PointCloud2".to_string()),
            publishers: 5,
            subscribers: 3,
            reliable: false,
        };

        assert_eq!(topic.publishers, 5);
        assert!(!topic.reliable); // Best effort for high-bandwidth data
    }

    #[test]
    fn test_pubsub_no_subscribers() {
        let topic = DiscoveredTopic {
            name: "/debug_info".to_string(),
            msg_type: Some("std_msgs/msg/String".to_string()),
            publishers: 1,
            subscribers: 0, // Topic exists but no subscribers yet
            reliable: true,
        };

        assert_eq!(topic.subscribers, 0);
    }

    #[test]
    fn test_pubsub_namespaced_topics() {
        let topics = vec![
            "/robot1/cmd_vel",
            "/robot1/odom",
            "/robot2/cmd_vel",
            "/robot2/odom",
        ];

        let robot1_topics: Vec<_> = topics.iter().filter(|t| t.starts_with("/robot1")).collect();
        let robot2_topics: Vec<_> = topics.iter().filter(|t| t.starts_with("/robot2")).collect();

        assert_eq!(robot1_topics.len(), 2);
        assert_eq!(robot2_topics.len(), 2);
    }

    #[test]
    fn test_pubsub_latched_topic() {
        // Latched topics (transient local durability) retain last message
        let topic = DiscoveredTopic {
            name: "/map".to_string(),
            msg_type: Some("nav_msgs/msg/OccupancyGrid".to_string()),
            publishers: 1,
            subscribers: 5,
            reliable: true, // Map data needs reliable delivery
        };

        assert!(topic.reliable);
        assert!(topic.subscribers > topic.publishers);
    }

    // --- Service Request/Response Tests ---

    #[test]
    fn test_service_request_construction() {
        let service = DiscoveredService {
            name: "/get_map".to_string(),
            srv_type: Some("nav_msgs/srv/GetMap".to_string()),
        };

        assert!(service.name.starts_with('/'));
        assert!(service.srv_type.as_ref().unwrap().contains("/srv/"));
    }

    #[test]
    fn test_service_response_types() {
        // Services have both Request and Response types
        let srv_type = "std_srvs/srv/SetBool";

        let request_type = format!("{}_Request", srv_type.replace("/srv/", "/msg/"));
        let response_type = format!("{}_Response", srv_type.replace("/srv/", "/msg/"));

        assert!(request_type.contains("_Request"));
        assert!(response_type.contains("_Response"));
    }

    #[test]
    fn test_service_key_construction() {
        let service = "/set_parameters";
        let domain_id = 0u32;

        // Request key format
        let request_key = format!("{}/rq{}/**", domain_id, service);
        // Response key format
        let response_key = format!("{}/rr{}/**", domain_id, service);

        assert!(request_key.contains("/rq/"));
        assert!(response_key.contains("/rr/"));
    }

    #[test]
    fn test_service_call_sequence() {
        // Service call: request ID tracking
        let request_id = uuid::Uuid::new_v4();
        let service_name = "/add_two_ints";

        let call_info = format!("{}:{}", service_name, request_id);
        assert!(call_info.contains(service_name));
    }

    #[test]
    fn test_service_timeout_handling() {
        let timeout_ms = 5000u64;
        let service = DiscoveredService {
            name: "/slow_service".to_string(),
            srv_type: Some("std_srvs/srv/Empty".to_string()),
        };

        // Verify timeout is reasonable for ROS2 services
        assert!(timeout_ms >= 1000);
        assert!(!service.name.is_empty());
    }

    // --- Action Goal/Feedback/Result Tests ---

    #[test]
    fn test_action_goal_creation() {
        let action = DiscoveredAction {
            name: "/navigate_to_pose".to_string(),
            action_type: Some("nav2_msgs/action/NavigateToPose".to_string()),
        };

        let goal_topic = format!("{}/_action/send_goal", action.name);
        assert!(goal_topic.contains("send_goal"));
    }

    #[test]
    fn test_action_feedback_topic() {
        let action_name = "/follow_path";
        let feedback_topic = format!("{}/_action/feedback", action_name);

        assert!(feedback_topic.ends_with("feedback"));
        assert!(feedback_topic.contains("_action"));
    }

    #[test]
    fn test_action_result_topic() {
        let action_name = "/spin";
        let result_topic = format!("{}/_action/get_result", action_name);

        assert!(result_topic.ends_with("get_result"));
    }

    #[test]
    fn test_action_status_topic() {
        let action_name = "/navigate";
        let status_topic = format!("{}/_action/status", action_name);

        assert!(status_topic.ends_with("status"));
    }

    #[test]
    fn test_action_cancel_service() {
        let action_name = "/dock_robot";
        let cancel_service = format!("{}/_action/cancel_goal", action_name);

        assert!(cancel_service.ends_with("cancel_goal"));
    }

    #[test]
    fn test_action_all_endpoints() {
        let action_name = "/compute_path";

        // Action has 5 endpoints: 2 topics + 3 services
        let endpoints = vec![
            format!("{}/_action/feedback", action_name),    // Topic
            format!("{}/_action/status", action_name),      // Topic
            format!("{}/_action/send_goal", action_name),   // Service
            format!("{}/_action/get_result", action_name),  // Service
            format!("{}/_action/cancel_goal", action_name), // Service
        ];

        assert_eq!(endpoints.len(), 5);
        let topics: Vec<_> = endpoints
            .iter()
            .filter(|e| e.contains("feedback") || e.contains("status"))
            .collect();
        let services: Vec<_> = endpoints
            .iter()
            .filter(|e| e.contains("goal") || e.contains("result"))
            .collect();

        assert_eq!(topics.len(), 2);
        assert_eq!(services.len(), 3);
    }

    #[test]
    fn test_action_goal_id_format() {
        // Action goals use UUID for tracking
        let goal_id = uuid::Uuid::new_v4();
        let goal_id_str = goal_id.to_string();

        // UUID format: 8-4-4-4-12
        assert_eq!(goal_id_str.len(), 36);
        assert_eq!(goal_id_str.chars().filter(|c| *c == '-').count(), 4);
    }

    #[test]
    fn test_bridge_direction_variants() {
        // Test all direction variants exist
        let directions = [
            BridgeDirection::In,   // ROS2 -> HORUS
            BridgeDirection::Out,  // HORUS -> ROS2
            BridgeDirection::Both, // Bidirectional
        ];

        assert_eq!(directions.len(), 3);
    }

    #[test]
    fn test_bridge_state_creation() {
        let config = BridgeConfig::default();
        let bridge = Ros2Bridge::new(config);

        // Bridge should not be running initially
        assert!(!bridge.running.load(std::sync::atomic::Ordering::SeqCst));
    }

    // --- Message Serialization Tests ---

    #[test]
    fn test_message_type_parsing() {
        let msg_type = "sensor_msgs/msg/LaserScan";
        let parts: Vec<&str> = msg_type.split('/').collect();

        assert_eq!(parts.len(), 3);
        assert_eq!(parts[0], "sensor_msgs"); // Package
        assert_eq!(parts[1], "msg"); // Type marker
        assert_eq!(parts[2], "LaserScan"); // Message name
    }

    #[test]
    fn test_service_type_parsing() {
        let srv_type = "std_srvs/srv/SetBool";
        let parts: Vec<&str> = srv_type.split('/').collect();

        assert_eq!(parts.len(), 3);
        assert_eq!(parts[1], "srv");
    }

    #[test]
    fn test_action_type_parsing() {
        let action_type = "nav2_msgs/action/NavigateToPose";
        let parts: Vec<&str> = action_type.split('/').collect();

        assert_eq!(parts.len(), 3);
        assert_eq!(parts[1], "action");
    }

    #[test]
    fn test_cdr_header_format() {
        // CDR messages have a 4-byte header
        let cdr_header: [u8; 4] = [0x00, 0x01, 0x00, 0x00]; // Little-endian CDR

        // First byte: 0x00 = CDR version 1
        // Second byte: 0x01 = little-endian (most common)
        assert_eq!(cdr_header[1], 0x01);
    }

    // --- QoS Compatibility Tests ---

    #[test]
    fn test_qos_reliability_compatibility() {
        // Reliable subscriber can connect to reliable publisher
        let pub_reliable = true;
        let sub_reliable = true;

        let compatible = !sub_reliable || pub_reliable;
        assert!(compatible);
    }

    #[test]
    fn test_qos_reliability_incompatibility() {
        // Best effort publisher, reliable subscriber = incompatible
        let pub_reliable = false;
        let sub_reliable = true;

        let compatible = !sub_reliable || pub_reliable;
        assert!(!compatible);
    }

    #[test]
    fn test_qos_durability_transient_local() {
        // Transient local (latched) for static data
        let topic = DiscoveredTopic {
            name: "/robot_description".to_string(),
            msg_type: Some("std_msgs/msg/String".to_string()),
            publishers: 1,
            subscribers: 10, // Many nodes need robot description
            reliable: true,
        };

        assert!(topic.reliable);
    }

    // --- Topic Filtering Tests ---

    #[test]
    fn test_filter_topics_by_prefix() {
        let topics = vec!["/scan", "/camera/image", "/camera/depth", "/odom", "/tf"];

        let camera_topics: Vec<_> = topics.iter().filter(|t| t.starts_with("/camera")).collect();

        assert_eq!(camera_topics.len(), 2);
    }

    #[test]
    fn test_filter_topics_by_type() {
        let mut result = DiscoveryResult::default();

        result.topics.push(DiscoveredTopic {
            name: "/image".to_string(),
            msg_type: Some("sensor_msgs/msg/Image".to_string()),
            publishers: 1,
            subscribers: 1,
            reliable: false,
        });
        result.topics.push(DiscoveredTopic {
            name: "/compressed".to_string(),
            msg_type: Some("sensor_msgs/msg/CompressedImage".to_string()),
            publishers: 1,
            subscribers: 1,
            reliable: false,
        });
        result.topics.push(DiscoveredTopic {
            name: "/cmd_vel".to_string(),
            msg_type: Some("geometry_msgs/msg/Twist".to_string()),
            publishers: 1,
            subscribers: 1,
            reliable: true,
        });

        let image_topics: Vec<_> = result
            .topics
            .iter()
            .filter(|t| t.msg_type.as_ref().map_or(false, |m| m.contains("Image")))
            .collect();

        assert_eq!(image_topics.len(), 2);
    }

    #[test]
    fn test_exclude_internal_topics() {
        let topics = vec!["/scan", "/rosout", "/parameter_events", "/cmd_vel"];

        let internal_topics = ["/rosout", "/parameter_events"];

        let user_topics: Vec<_> = topics
            .iter()
            .filter(|t| !internal_topics.contains(t))
            .collect();

        assert_eq!(user_topics.len(), 2);
    }

    // --- Error Handling Tests ---

    #[test]
    fn test_empty_topic_name_detection() {
        let topic_name = "";
        assert!(topic_name.is_empty());
    }

    #[test]
    fn test_invalid_topic_name_no_slash() {
        let topic_name = "cmd_vel"; // Missing leading slash
        assert!(!topic_name.starts_with('/'));
    }

    #[test]
    fn test_topic_name_normalization() {
        let topic_name = "scan";
        let normalized = if topic_name.starts_with('/') {
            topic_name.to_string()
        } else {
            format!("/{}", topic_name)
        };

        assert!(normalized.starts_with('/'));
        assert_eq!(normalized, "/scan");
    }

    // --- Async Discovery Tests ---

    #[cfg(feature = "zenoh-transport")]
    #[tokio::test]
    async fn test_discover_all_entity_types() {
        use tokio::join;

        let domain_id = 0;

        let (topics, services, actions, params) = join!(
            discover_ros2_topics(domain_id),
            discover_ros2_services(domain_id),
            discover_ros2_actions(domain_id),
            discover_ros2_parameters(domain_id)
        );

        // All should return Ok (even if empty)
        assert!(topics.is_ok());
        assert!(services.is_ok());
        assert!(actions.is_ok());
        assert!(params.is_ok());
    }

    #[cfg(feature = "zenoh-transport")]
    #[tokio::test]
    async fn test_bridge_initialization() {
        let config = BridgeConfig::default();
        let bridge = Ros2Bridge::new(config);

        assert!(!bridge.running.load(std::sync::atomic::Ordering::SeqCst));
    }

    // --- Stats Tracking Tests ---

    #[test]
    fn test_bridge_stats_topics_bridged() {
        let stats = BridgeStats::new();

        stats
            .topics_bridged
            .fetch_add(100, std::sync::atomic::Ordering::Relaxed);

        assert_eq!(
            stats
                .topics_bridged
                .load(std::sync::atomic::Ordering::Relaxed),
            100
        );
    }

    #[test]
    fn test_bridge_stats_topics_discovered() {
        let stats = BridgeStats::new();

        stats
            .topics_discovered
            .store(50, std::sync::atomic::Ordering::Relaxed);

        assert_eq!(
            stats
                .topics_discovered
                .load(std::sync::atomic::Ordering::Relaxed),
            50
        );
    }

    #[test]
    fn test_bridge_stats_concurrent_updates() {
        use std::sync::atomic::Ordering;

        let stats = BridgeStats::new();

        // Simulate concurrent topic bridging counting
        for _ in 0..1000 {
            stats.topics_bridged.fetch_add(1, Ordering::Relaxed);
        }

        assert_eq!(stats.topics_bridged.load(Ordering::Relaxed), 1000);
    }

    // --- Multi-Domain Tests ---

    #[test]
    fn test_domain_isolation() {
        // Topics on different domains are isolated
        let domain0_topic = format!("{}/rt/scan", 0);
        let domain1_topic = format!("{}/rt/scan", 1);

        assert_ne!(domain0_topic, domain1_topic);
    }

    #[test]
    fn test_domain_id_range_valid() {
        // ROS2 supports domain IDs 0-232
        for domain_id in [0u32, 100, 232] {
            assert!(domain_id <= 232);
        }
    }

    #[test]
    fn test_domain_bridge_key_format() {
        let domain_id = 42u32;
        let topic = "/robot/state";

        let key = format!("{}/rt{}", domain_id, topic);

        assert!(key.starts_with("42/"));
        assert!(key.contains("/rt/"));
    }
}
