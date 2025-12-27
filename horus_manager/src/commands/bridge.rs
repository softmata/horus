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
    use horus_core::communication::Link;
    use std::sync::atomic::Ordering;

    log::info!("Starting service bridge for: {}", service_name);

    // Construct Zenoh key expressions for ROS2 service
    // Request key: rq/{service_name}/**
    let request_key = format!("rq{}", service_name.trim_start_matches('/'));
    log::debug!("Service request key: {}", request_key);

    // HORUS service topic names (request/response pair)
    let horus_request_topic = format!("{}/request", service_name.trim_start_matches('/'));
    let horus_response_topic = format!("{}/response", service_name.trim_start_matches('/'));

    // Create HORUS shared memory links for service communication
    // We use Vec<u8> for raw byte passthrough (CDR encoded)
    let horus_request_pub: Link<Vec<u8>> = Link::producer(&horus_request_topic)?;
    let horus_response_sub: Link<Vec<u8>> = Link::consumer(&horus_response_topic)?;

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
    use horus_core::communication::Link;
    use std::sync::atomic::Ordering;

    log::info!(
        "Starting HORUS -> ROS2 service client bridge for: {}",
        service_name
    );

    // HORUS topics for request/response
    let horus_request_topic = format!("{}/client_request", service_name.trim_start_matches('/'));
    let horus_response_topic = format!("{}/client_response", service_name.trim_start_matches('/'));

    // Zenoh service endpoint
    let request_key = format!("rq{}", service_name.trim_start_matches('/'));

    // Create HORUS links
    let horus_request_sub: Link<Vec<u8>> = Link::consumer(&horus_request_topic)?;
    let horus_response_pub: Link<Vec<u8>> = Link::producer(&horus_response_topic)?;

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
    use horus_core::communication::Link;
    use std::sync::atomic::Ordering;

    log::info!("Starting action bridge for: {}", action_name);

    let action_base = action_name.trim_start_matches('/');

    // Zenoh key expressions for action services and topics
    let send_goal_key = format!("rq/{}/_action/send_goal", action_base);
    let cancel_goal_key = format!("rq/{}/_action/cancel_goal", action_base);
    let get_result_key = format!("rq/{}/_action/get_result", action_base);
    let feedback_key = format!("{}/_action/feedback", action_base);
    let status_key = format!("{}/_action/status", action_base);

    // HORUS shared memory topics
    let horus_goal_topic = format!("{}/goal", action_base);
    let horus_goal_response_topic = format!("{}/goal_response", action_base);
    let horus_cancel_topic = format!("{}/cancel", action_base);
    let horus_cancel_response_topic = format!("{}/cancel_response", action_base);
    let horus_result_request_topic = format!("{}/result_request", action_base);
    let horus_result_topic = format!("{}/result", action_base);
    let horus_feedback_topic = format!("{}/feedback", action_base);
    let horus_status_topic = format!("{}/status", action_base);

    log::debug!(
        "Action bridge keys: send_goal={}, feedback={}",
        send_goal_key,
        feedback_key
    );

    // Create HORUS links for action communication
    let goal_pub: Link<Vec<u8>> = Link::producer(&horus_goal_topic)?;
    let goal_response_sub: Link<Vec<u8>> = Link::consumer(&horus_goal_response_topic)?;
    let cancel_pub: Link<Vec<u8>> = Link::producer(&horus_cancel_topic)?;
    let cancel_response_sub: Link<Vec<u8>> = Link::consumer(&horus_cancel_response_topic)?;
    let result_request_pub: Link<Vec<u8>> = Link::producer(&horus_result_request_topic)?;
    let result_sub: Link<Vec<u8>> = Link::consumer(&horus_result_topic)?;
    let feedback_pub: Link<Vec<u8>> = Link::producer(&horus_feedback_topic)?;
    let status_pub: Link<Vec<u8>> = Link::producer(&horus_status_topic)?;

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
    use horus_core::communication::Link;

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
    let horus_param_req: Link<Vec<u8>> = Link::producer(&format!("{}/request", horus_param_topic))?;
    let horus_param_resp: Link<Vec<u8>> =
        Link::consumer(&format!("{}/response", horus_param_topic))?;

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

    // HORUS shared memory topic name (strip leading slash if present)
    let horus_topic = topic_name.trim_start_matches('/');
    log::debug!("HORUS topic: {}", horus_topic);

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
    use horus_core::communication::Link;
    use std::sync::atomic::Ordering;

    log::info!("Starting ROS2 -> HORUS bridge for {}", ros2_topic);

    // Create Zenoh subscriber for ROS2 topic
    let mut zenoh_sub: ZenohBackend<Vec<u8>> = ZenohBackend::new(ros2_topic, zenoh_config).await?;
    zenoh_sub.init_subscriber().await?;

    // Create HORUS shared memory producer
    let horus_pub: Link<Vec<u8>> = Link::producer(horus_topic)?;

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
    use horus_core::communication::Link;
    use std::sync::atomic::Ordering;

    log::info!("Starting HORUS -> ROS2 bridge for {}", horus_topic);

    // Create HORUS shared memory consumer
    let horus_sub: Link<Vec<u8>> = Link::consumer(horus_topic)?;

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
        use horus_core::communication::network::zenoh_config::Reliability;
        let zenoh_qos = map_qos_profile(QosProfile::SensorData);
        // Sensor data should use best effort reliability
        assert_eq!(zenoh_qos.reliability, Reliability::BestEffort);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_qos_mapping_services() {
        use horus_core::communication::network::zenoh_config::Reliability;
        let zenoh_qos = map_qos_profile(QosProfile::Services);
        // Services should use reliable delivery
        assert_eq!(zenoh_qos.reliability, Reliability::Reliable);
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_qos_mapping_default() {
        let zenoh_qos = map_qos_profile(QosProfile::Default);
        // Default should exist and be valid
        let _ = zenoh_qos;
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
}
