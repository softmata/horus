// Message types for the HORUS robotics framework
//
// This module contains all standardized message types used for communication
// between HORUS components (nodes, algorithms, and applications).
//
// # Message Organization
//
// Messages are organized by domain:
// - Geometry: Spatial primitives (Twist, Pose2D, TransformStamped, etc.)
// - Sensor: Sensor data formats (LaserScan, Imu, Odometry, etc.)
// - Control: Actuator commands (MotorCommand, ServoCommand, PID, etc.)
// - Diagnostics: System health (Status, Heartbeat, EmergencyStop, etc.)
// - Input: User input (KeyboardInput, JoystickInput)
// - Application: App-specific messages (CmdVel, etc.)
//
// All message types are re-exported at the crate root for convenience.

/// Implement Pod, Zeroable, and PodMessage for a `#[repr(C)]` message type.
macro_rules! impl_pod_message {
    ($($ty:ty),+ $(,)?) => {
        $(
            unsafe impl horus_core::bytemuck::Pod for $ty {}
            unsafe impl horus_core::bytemuck::Zeroable for $ty {}
            unsafe impl horus_core::communication::PodMessage for $ty {}
        )+
    };
}
pub(crate) use impl_pod_message;

/// Generate `with_frame_id` builder method for types with a `frame_id: [u8; 32]` field.
macro_rules! impl_with_frame_id {
    () => {
        /// Set the frame ID (builder pattern).
        pub fn with_frame_id(mut self, frame_id: &str) -> Self {
            let bytes = frame_id.as_bytes();
            let len = bytes.len().min(31);
            self.frame_id[..len].copy_from_slice(&bytes[..len]);
            self.frame_id[len..].fill(0);
            self
        }
    };
}
pub(crate) use impl_with_frame_id;

// Core message modules
pub mod clock;
pub mod control;
pub mod diagnostics;
pub mod force;
pub mod geometry;
pub mod navigation;
pub mod perception;
pub mod sensor;
pub mod vision;

// Pod/Zeroable types for zero-copy IPC
pub mod detection;
pub mod landmark;
pub mod segmentation;
pub mod tracking;

// Input messages
pub mod joystick_msg;
pub mod keyboard_input_msg;

// Audio
pub mod audio;

// Application-specific messages
pub mod cmd_vel;

// Re-export all message types for convenience
// Geometry
pub use geometry::{
    Accel, AccelStamped, Point3, Pose2D, Pose3D, PoseStamped, PoseWithCovariance, Quaternion,
    TransformStamped, Twist, TwistWithCovariance, Vector3,
};

// Sensor
pub use sensor::{
    BatteryState, FluidPressure, Illuminance, Imu, JointState, LaserScan, MagneticField, NavSatFix,
    Odometry, RangeSensor, Temperature,
};

// Clock & Time
pub use clock::{Clock, TimeReference, SOURCE_REPLAY, SOURCE_SIM, SOURCE_WALL};

// Control
pub use control::{
    DifferentialDriveCommand, JointCommand, MotorCommand, PidConfig, ServoCommand, TrajectoryPoint,
};

// Diagnostics
pub use diagnostics::{
    DiagnosticReport, DiagnosticStatus, DiagnosticValue, EmergencyStop, Heartbeat, NodeHeartbeat,
    NodeStateMsg, ResourceUsage, SafetyStatus, StatusLevel,
};

// Vision
pub use vision::{CameraInfo, CompressedImage, RegionOfInterest};

// Navigation
pub use navigation::{CostMap, NavGoal, NavPath, OccupancyGrid, PathPlan};

// Force
pub use force::{ForceCommand, ImpedanceParameters, WrenchStamped};

// Perception
pub use perception::PlaneDetection;

// Perception types (zero-copy IPC)
pub use detection::{BoundingBox2D, BoundingBox3D, Detection, Detection3D};
pub use landmark::{Landmark, Landmark3D, LandmarkArray};
pub use segmentation::SegmentationMask;
pub use tracking::{TrackedObject, TrackingHeader};

// Input (existing)
pub use joystick_msg::JoystickInput;
pub use keyboard_input_msg::KeyboardInput;

// Audio
pub use audio::{AudioEncoding, AudioFrame, MAX_AUDIO_SAMPLES};

// Application (existing)
pub use cmd_vel::CmdVel;

// Imports for GenericMessage definition
use horus_core::core::LogSummary;
use serde::{Deserialize, Serialize};
use serde_json;

/// Generic message type for cross-language communication
///
/// This message type provides a standardized way to communicate between
/// Rust and Python nodes when type-specific messages are not needed or
/// when dynamic typing is preferred.
///
/// The `data` field contains MessagePack-serialized payload, and the optional
/// `metadata` field can store additional information like message type or timestamp.
///
/// # Example (Rust)
///
/// ```rust,ignore
/// use horus::prelude::*;
/// extern crate rmp_serde;
///
/// let topic = Topic::<GenericMessage>::new("my_topic")?;
///
/// // Send a dict-like structure (requires rmp_serde dependency)
/// let data = rmp_serde::to_vec(&serde_json::json!({
///     "x": 1.0,
///     "y": 2.0
/// }))?;
///
/// let msg = GenericMessage::new(data);
/// topic.send(msg);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
///
/// # Example (Python)
///
/// ```python
/// from horus import Topic
///
/// topic = Topic("my_topic")
///
/// # Send automatically serializes to GenericMessage
/// topic.send({"x": 1.0, "y": 2.0}, node)
///
/// # Receive automatically deserializes
/// msg_bytes = topic.recv(node)
/// if msg_bytes:
///     import msgpack
///     data = msgpack.unpackb(msg_bytes, raw=False)
/// ```
///
/// # Performance Notes
///
/// GenericMessage uses a fixed-size buffer for cross-language safety:
/// - Small messages (≤256 bytes): Optimized fast path (~4.0 μs)
/// - Large messages (>256 bytes): Standard path (~4.4 μs)
/// - Maximum payload size: 4KB (configurable via feature flags)
///
/// For performance-critical paths, use typed messages (Pose2D, CmdVel) which
/// achieve ~200ns latency with zero-copy IPC.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct GenericMessage {
    /// Fast path: Inline buffer for small messages (≤256 bytes)
    /// This covers ~80% of typical use cases and avoids large buffer copies
    inline_data: [u8; 256],
    inline_len: u16,

    /// Overflow buffer for larger messages (256-4096 bytes)
    overflow_data: [u8; 3840], // 4096 - 256
    overflow_len: u32,

    /// Optional metadata (fixed-size buffer)
    metadata: [u8; 256],
    metadata_len: u16,

    /// Reserved for alignment and future use
    _padding: [u8; 2],
}

/// Maximum payload size for GenericMessage (4KB)
pub const MAX_GENERIC_PAYLOAD: usize = 4096;

/// Size of inline buffer for small message optimization
const INLINE_BUFFER_SIZE: usize = 256;

impl GenericMessage {
    /// Create a new GenericMessage with raw bytes
    ///
    /// Returns an error if data exceeds MAX_GENERIC_PAYLOAD (4KB).
    ///
    /// # Performance
    ///
    /// - Small messages (≤256 bytes): Fast path, only copies inline buffer
    /// - Large messages (>256 bytes): Copies both inline and overflow buffers
    pub fn new(data: Vec<u8>) -> horus_core::error::HorusResult<Self> {
        if data.len() > MAX_GENERIC_PAYLOAD {
            return Err(horus_core::error::ValidationError::OutOfRange {
                field: "data".into(),
                actual: data.len().to_string(),
                min: "0".into(),
                max: MAX_GENERIC_PAYLOAD.to_string(),
            }.into());
        }

        let mut msg = Self {
            inline_data: [0; INLINE_BUFFER_SIZE],
            inline_len: 0,
            overflow_data: [0; MAX_GENERIC_PAYLOAD - INLINE_BUFFER_SIZE],
            overflow_len: 0,
            metadata: [0; 256],
            metadata_len: 0,
            _padding: [0; 2],
        };

        if data.len() <= INLINE_BUFFER_SIZE {
            // Fast path: Small message, only use inline buffer
            msg.inline_data[..data.len()].copy_from_slice(&data);
            msg.inline_len = data.len() as u16;
        } else {
            // Slow path: Large message, use both buffers
            msg.inline_data.copy_from_slice(&data[..INLINE_BUFFER_SIZE]);
            msg.inline_len = INLINE_BUFFER_SIZE as u16;

            let overflow_len = data.len() - INLINE_BUFFER_SIZE;
            msg.overflow_data[..overflow_len].copy_from_slice(&data[INLINE_BUFFER_SIZE..]);
            msg.overflow_len = overflow_len as u32;
        }

        Ok(msg)
    }

    /// Create a GenericMessage with metadata
    ///
    /// Metadata is limited to 256 bytes.
    pub fn with_metadata(data: Vec<u8>, metadata: String) -> horus_core::error::HorusResult<Self> {
        let mut msg = Self::new(data)?;

        if metadata.len() > 255 {
            return Err(horus_core::error::ValidationError::OutOfRange {
                field: "metadata".into(),
                actual: metadata.len().to_string(),
                min: "0".into(),
                max: "255".into(),
            }.into());
        }

        let metadata_bytes = metadata.as_bytes();
        msg.metadata[..metadata_bytes.len()].copy_from_slice(metadata_bytes);
        msg.metadata_len = metadata_bytes.len() as u16;

        Ok(msg)
    }

    /// Get the payload data as a slice
    ///
    /// This reconstructs the original data from inline and overflow buffers.
    pub fn data(&self) -> Vec<u8> {
        let inline_len = self.inline_len as usize;
        let overflow_len = self.overflow_len as usize;
        let total_len = inline_len + overflow_len;

        let mut result = Vec::with_capacity(total_len);
        result.extend_from_slice(&self.inline_data[..inline_len]);
        if overflow_len > 0 {
            result.extend_from_slice(&self.overflow_data[..overflow_len]);
        }
        result
    }

    /// Get metadata as a string (if present)
    pub fn metadata(&self) -> Option<String> {
        if self.metadata_len == 0 {
            None
        } else {
            let bytes = &self.metadata[..self.metadata_len as usize];
            String::from_utf8(bytes.to_vec()).ok()
        }
    }

    /// Serialize any serde-compatible type into a GenericMessage
    ///
    /// This is the recommended way to create GenericMessage from structured data.
    /// Users don't need to handle MessagePack serialization directly.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus::prelude::*;
    /// use std::collections::HashMap;
    ///
    /// let mut data = HashMap::new();
    /// data.insert("x", 1.0);
    /// data.insert("y", 2.0);
    ///
    /// let msg = GenericMessage::from_value(&data)?;
    /// ```
    pub fn from_value<T: Serialize>(value: &T) -> horus_core::error::HorusResult<Self> {
        let data = rmp_serde::to_vec(value).map_err(|e| {
            horus_core::error::HorusError::InvalidInput(
                horus_core::error::ValidationError::InvalidFormat {
                    field: "value".into(),
                    expected_format: "MessagePack-serializable".into(),
                    actual: e.to_string(),
                },
            )
        })?;
        Self::new(data)
    }

    /// Deserialize the data field into a typed value
    ///
    /// This is the recommended way to extract structured data from GenericMessage.
    /// Users don't need to handle MessagePack deserialization directly.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus::prelude::*;
    /// use std::collections::HashMap;
    ///
    /// if let Some(msg) = hub.recv(ctx) {
    ///     let data: HashMap<String, f64> = msg.to_value()?;
    ///     println!("x: {}, y: {}", data["x"], data["y"]);
    /// }
    /// ```
    pub fn to_value<T: for<'de> Deserialize<'de>>(&self) -> horus_core::error::HorusResult<T> {
        let data = self.data();
        rmp_serde::from_slice(&data).map_err(|e| {
            horus_core::error::HorusError::InvalidInput(
                horus_core::error::ValidationError::InvalidFormat {
                    field: "data".into(),
                    expected_format: "MessagePack-deserializable".into(),
                    actual: e.to_string(),
                },
            )
        })
    }
}

impl LogSummary for GenericMessage {
    fn log_summary(&self) -> String {
        let total_len = self.inline_len as usize + self.overflow_len as usize;

        // Try to deserialize and show content (with performance cost)
        let data = self.data();

        // Try JSON first (Python Node API uses JSON)
        if let Ok(json_str) = std::str::from_utf8(&data) {
            if let Ok(json_val) = serde_json::from_str::<serde_json::Value>(json_str) {
                let formatted = format!("{}", json_val);
                // Truncate long messages to 200 chars
                if formatted.len() > 200 {
                    return format!("{}... ({} bytes total)", &formatted[..200], total_len);
                }
                return formatted;
            }
        }

        // Try MessagePack next (Rust-to-Rust generic messages)
        if let Ok(msgpack_val) = rmp_serde::from_slice::<serde_json::Value>(&data) {
            let formatted = format!("{}", msgpack_val);
            if formatted.len() > 200 {
                return format!("{}... ({} bytes total)", &formatted[..200], total_len);
            }
            return formatted;
        }

        // Fallback: show hex for binary data (truncated)
        if total_len > 32 {
            let hex_sample = data[..32]
                .iter()
                .map(|b| format!("{:02x}", b))
                .collect::<Vec<_>>()
                .join(" ");
            if let Some(meta) = self.metadata() {
                format!("<{} bytes: {} ..., meta: {}>", total_len, hex_sample, meta)
            } else {
                format!("<{} bytes: {} ...>", total_len, hex_sample)
            }
        } else {
            let hex = data
                .iter()
                .map(|b| format!("{:02x}", b))
                .collect::<Vec<_>>()
                .join(" ");
            if let Some(meta) = self.metadata() {
                format!("<{} bytes: {}, meta: {}>", total_len, hex, meta)
            } else {
                format!("<{} bytes: {}>", total_len, hex)
            }
        }
    }
}

// Manual Serialize implementation - only serialize used portions of buffers
impl Serialize for GenericMessage {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;

        let mut state = serializer.serialize_struct("GenericMessage", 5)?;

        // Serialize only the used portion of inline_data
        let inline_len = self.inline_len as usize;
        state.serialize_field("inline_data", &self.inline_data[..inline_len])?;
        state.serialize_field("inline_len", &self.inline_len)?;

        // Serialize only the used portion of overflow_data
        let overflow_len = self.overflow_len as usize;
        state.serialize_field("overflow_data", &self.overflow_data[..overflow_len])?;
        state.serialize_field("overflow_len", &self.overflow_len)?;

        // Serialize only the used portion of metadata
        let metadata_len = self.metadata_len as usize;
        state.serialize_field("metadata", &self.metadata[..metadata_len])?;

        state.end()
    }
}

// Manual Deserialize implementation - reconstruct from used portions
impl<'de> Deserialize<'de> for GenericMessage {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        use serde::de::{self, MapAccess, Visitor};
        use std::fmt;

        struct GenericMessageVisitor;

        impl<'de> Visitor<'de> for GenericMessageVisitor {
            type Value = GenericMessage;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct GenericMessage")
            }

            fn visit_map<V>(self, mut map: V) -> Result<GenericMessage, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut inline_data: Option<Vec<u8>> = None;
                let mut inline_len: Option<u16> = None;
                let mut overflow_data: Option<Vec<u8>> = None;
                let mut overflow_len: Option<u32> = None;
                let mut metadata: Option<Vec<u8>> = None;

                while let Some(key) = map.next_key::<String>()? {
                    match key.as_str() {
                        "inline_data" => {
                            inline_data = Some(map.next_value()?);
                        }
                        "inline_len" => {
                            inline_len = Some(map.next_value()?);
                        }
                        "overflow_data" => {
                            overflow_data = Some(map.next_value()?);
                        }
                        "overflow_len" => {
                            overflow_len = Some(map.next_value()?);
                        }
                        "metadata" => {
                            metadata = Some(map.next_value()?);
                        }
                        _ => {
                            let _: serde::de::IgnoredAny = map.next_value()?;
                        }
                    }
                }

                let inline_data =
                    inline_data.ok_or_else(|| de::Error::missing_field("inline_data"))?;
                let inline_len =
                    inline_len.ok_or_else(|| de::Error::missing_field("inline_len"))?;
                let overflow_data =
                    overflow_data.ok_or_else(|| de::Error::missing_field("overflow_data"))?;
                let overflow_len =
                    overflow_len.ok_or_else(|| de::Error::missing_field("overflow_len"))?;
                let metadata = metadata.ok_or_else(|| de::Error::missing_field("metadata"))?;

                // Reconstruct the full message
                let mut msg = GenericMessage {
                    inline_data: [0; INLINE_BUFFER_SIZE],
                    inline_len,
                    overflow_data: [0; MAX_GENERIC_PAYLOAD - INLINE_BUFFER_SIZE],
                    overflow_len,
                    metadata: [0; 256],
                    metadata_len: metadata.len() as u16,
                    _padding: [0; 2],
                };

                // Copy the data into fixed buffers
                let inline_copy_len = inline_data.len().min(INLINE_BUFFER_SIZE);
                msg.inline_data[..inline_copy_len].copy_from_slice(&inline_data[..inline_copy_len]);
                // Clamp inline_len to actual copied data to prevent OOB in data()
                msg.inline_len = (msg.inline_len as usize).min(inline_copy_len) as u16;

                let overflow_copy_len = overflow_data
                    .len()
                    .min(MAX_GENERIC_PAYLOAD - INLINE_BUFFER_SIZE);
                msg.overflow_data[..overflow_copy_len]
                    .copy_from_slice(&overflow_data[..overflow_copy_len]);
                // Clamp overflow_len to actual copied data to prevent OOB in data()
                msg.overflow_len = (msg.overflow_len as usize).min(overflow_copy_len) as u32;

                let metadata_copy_len = metadata.len().min(256);
                msg.metadata[..metadata_copy_len].copy_from_slice(&metadata[..metadata_copy_len]);
                msg.metadata_len = (msg.metadata_len as usize).min(metadata_copy_len) as u16;

                Ok(msg)
            }
        }

        const FIELDS: &[&str] = &[
            "inline_data",
            "inline_len",
            "overflow_data",
            "overflow_len",
            "metadata",
        ];
        deserializer.deserialize_struct("GenericMessage", FIELDS, GenericMessageVisitor)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    // ============================================================================
    // GenericMessage Creation Tests
    // ============================================================================

    #[test]
    fn test_generic_message_new_small() {
        let data = vec![1, 2, 3, 4, 5];
        let msg = GenericMessage::new(data.clone()).unwrap();
        assert_eq!(msg.data(), data);
    }

    #[test]
    fn test_generic_message_new_inline_boundary() {
        // Exactly 256 bytes (inline buffer size)
        let data: Vec<u8> = (0..=255).map(|i| i as u8).collect();
        let msg = GenericMessage::new(data.clone()).unwrap();
        assert_eq!(msg.data(), data);
        assert_eq!(msg.inline_len, 256);
        assert_eq!(msg.overflow_len, 0);
    }

    #[test]
    fn test_generic_message_new_overflow() {
        // 300 bytes - requires overflow buffer
        let data: Vec<u8> = (0..300).map(|i| (i % 256) as u8).collect();
        let msg = GenericMessage::new(data.clone()).unwrap();
        assert_eq!(msg.data(), data);
        assert_eq!(msg.inline_len, 256);
        assert_eq!(msg.overflow_len, 44);
    }

    #[test]
    fn test_generic_message_new_max_size() {
        // Maximum payload size (4096 bytes)
        let data: Vec<u8> = vec![0xAB; MAX_GENERIC_PAYLOAD];
        let msg = GenericMessage::new(data.clone()).unwrap();
        assert_eq!(msg.data().len(), MAX_GENERIC_PAYLOAD);
    }

    #[test]
    fn test_generic_message_new_too_large() {
        // Exceeds maximum - should fail
        let data: Vec<u8> = vec![0xAB; MAX_GENERIC_PAYLOAD + 1];
        let result = GenericMessage::new(data);
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("out of range"));
    }

    #[test]
    fn test_generic_message_empty() {
        let msg = GenericMessage::new(vec![]).unwrap();
        assert!(msg.data().is_empty());
        assert_eq!(msg.inline_len, 0);
        assert_eq!(msg.overflow_len, 0);
    }

    // ============================================================================
    // GenericMessage Metadata Tests
    // ============================================================================

    #[test]
    fn test_generic_message_with_metadata() {
        let data = vec![1, 2, 3];
        let msg = GenericMessage::with_metadata(data.clone(), "test_type".to_string()).unwrap();
        assert_eq!(msg.data(), data);
        assert_eq!(msg.metadata(), Some("test_type".to_string()));
    }

    #[test]
    fn test_generic_message_metadata_none() {
        let msg = GenericMessage::new(vec![1, 2, 3]).unwrap();
        assert_eq!(msg.metadata(), None);
    }

    #[test]
    fn test_generic_message_metadata_too_large() {
        let data = vec![1, 2, 3];
        let long_metadata = "x".repeat(256); // Exceeds 255 byte limit
        let result = GenericMessage::with_metadata(data, long_metadata);
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("out of range"));
    }

    // ============================================================================
    // GenericMessage Serialization Tests (from_value/to_value)
    // ============================================================================

    #[test]
    fn test_generic_message_from_value_hashmap() {
        let mut data = HashMap::new();
        data.insert("x", 1.0f64);
        data.insert("y", 2.0f64);

        let msg = GenericMessage::from_value(&data).unwrap();
        let restored: HashMap<String, f64> = msg.to_value().unwrap();
        assert_eq!(restored.get("x"), Some(&1.0));
        assert_eq!(restored.get("y"), Some(&2.0));
    }

    #[test]
    fn test_generic_message_from_value_vec() {
        let data: Vec<i32> = vec![1, 2, 3, 4, 5];
        let msg = GenericMessage::from_value(&data).unwrap();
        let restored: Vec<i32> = msg.to_value().unwrap();
        assert_eq!(restored, data);
    }

    #[test]
    fn test_generic_message_from_value_string() {
        let data = "hello, world!".to_string();
        let msg = GenericMessage::from_value(&data).unwrap();
        let restored: String = msg.to_value().unwrap();
        assert_eq!(restored, data);
    }

    #[test]
    fn test_generic_message_from_value_struct() {
        #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
        struct TestStruct {
            name: String,
            value: f64,
        }

        let data = TestStruct {
            name: "test".to_string(),
            value: 42.5,
        };

        let msg = GenericMessage::from_value(&data).unwrap();
        let restored: TestStruct = msg.to_value().unwrap();
        assert_eq!(restored, data);
    }

    // ============================================================================
    // GenericMessage Serde Serialization Tests
    // ============================================================================

    #[test]
    fn test_generic_message_json_roundtrip() {
        let data = vec![1, 2, 3, 4, 5];
        let msg = GenericMessage::new(data.clone()).unwrap();

        // Serialize to JSON
        let json = serde_json::to_string(&msg).unwrap();

        // Deserialize back
        let restored: GenericMessage = serde_json::from_str(&json).unwrap();
        assert_eq!(restored.data(), data);
    }

    #[test]
    fn test_generic_message_msgpack_roundtrip() {
        // Test that GenericMessage can serialize/deserialize arbitrary data using its internal
        // MessagePack encoding (from_value/to_value), not direct struct serialization
        let original: Vec<i32> = vec![1, 2, 3, 4, 5];
        let msg = GenericMessage::from_value(&original).unwrap();

        // The data is stored as MessagePack bytes inside GenericMessage
        let restored: Vec<i32> = msg.to_value().unwrap();
        assert_eq!(restored, original);
    }

    #[test]
    fn test_generic_message_with_metadata_roundtrip() {
        let data = vec![10, 20, 30];
        let msg = GenericMessage::with_metadata(data.clone(), "my_metadata".to_string()).unwrap();

        let json = serde_json::to_string(&msg).unwrap();
        let restored: GenericMessage = serde_json::from_str(&json).unwrap();

        assert_eq!(restored.data(), data);
        assert_eq!(restored.metadata(), Some("my_metadata".to_string()));
    }

    // ============================================================================
    // GenericMessage LogSummary Tests
    // ============================================================================

    #[test]
    fn test_generic_message_log_summary_json() {
        // Create a message with JSON data
        let json_data = r#"{"key":"value"}"#;
        let msg = GenericMessage::new(json_data.as_bytes().to_vec()).unwrap();
        let summary = msg.log_summary();
        // JSON path should parse and include both key and value
        assert!(summary.contains("key"), "summary should contain key name: {}", summary);
        assert!(summary.contains("value"), "summary should contain value: {}", summary);
        // Should NOT be the hex fallback path
        assert!(!summary.contains("bytes:"), "JSON data should not use hex fallback: {}", summary);
    }

    #[test]
    fn test_generic_message_log_summary_msgpack() {
        // Create a message with MessagePack data
        let value: Vec<i32> = vec![1, 2, 3];
        let msg = GenericMessage::from_value(&value).unwrap();
        let summary = msg.log_summary();
        // MessagePack path should decode the array values
        assert!(summary.contains('1'), "summary should contain value 1: {}", summary);
        assert!(summary.contains('2'), "summary should contain value 2: {}", summary);
        assert!(summary.contains('3'), "summary should contain value 3: {}", summary);
    }

    #[test]
    fn test_generic_message_log_summary_binary() {
        // 0xFF decodes as -1 in MessagePack, so short binary data may be parsed
        // as valid MessagePack. Verify that log_summary produces a meaningful
        // output for such data (either decoded MessagePack or hex fallback).
        let binary_data = vec![0xFF, 0xFE, 0xFD, 0xFC];
        let msg = GenericMessage::new(binary_data).unwrap();
        let summary = msg.log_summary();
        // 0xFF is msgpack negative fixint for -1, so it should decode as "-1"
        assert_eq!(summary, "-1", "0xFF should decode as msgpack -1: {}", summary);

        // Use bytes that are genuinely invalid for both JSON and MessagePack
        // 0xC1 is the "never used" type in MessagePack
        let truly_binary = vec![0xC1, 0xC1, 0xC1, 0xC1];
        let msg2 = GenericMessage::new(truly_binary).unwrap();
        let summary2 = msg2.log_summary();
        assert!(summary2.contains("4 bytes"), "invalid msgpack should use hex fallback: {}", summary2);
        assert!(summary2.contains("c1"), "hex fallback should contain c1: {}", summary2);
    }

    #[test]
    fn test_generic_message_log_summary_large_message() {
        // Create a large JSON message (>200 chars formatted) to test truncation
        let large_json = format!(r#"{{"data":"{}"}}"#, "x".repeat(300));
        let msg = GenericMessage::new(large_json.as_bytes().to_vec()).unwrap();
        let summary = msg.log_summary();
        // JSON string is >200 chars formatted, so truncation with "..." must occur
        assert!(summary.contains("..."), "large message summary should be truncated: {}", summary);
        assert!(summary.contains("bytes total"), "truncated summary should show total bytes: {}", summary);
        // Truncated portion should be at most ~200 chars before the "..."
        let prefix_len = summary.find("...").unwrap();
        assert!(prefix_len <= 210, "prefix before '...' should be ~200 chars, got {}", prefix_len);
    }

    #[test]
    fn test_generic_message_log_summary_empty() {
        // Empty message should produce hex fallback with 0 bytes
        let msg = GenericMessage::new(vec![]).unwrap();
        let summary = msg.log_summary();
        assert!(summary.contains("0 bytes"), "empty message summary should say 0 bytes: {}", summary);
    }

    #[test]
    fn test_generic_message_log_summary_binary_large() {
        // Use 0xC1 bytes (invalid msgpack) to ensure hex fallback path
        // Binary data >32 bytes should be truncated with "..."
        let binary_data: Vec<u8> = vec![0xC1; 64];
        let msg = GenericMessage::new(binary_data).unwrap();
        let summary = msg.log_summary();
        assert!(summary.contains("64 bytes"), "should report 64 bytes: {}", summary);
        assert!(summary.contains("..."), "large binary should show truncated hex: {}", summary);
    }

    #[test]
    fn test_generic_message_log_summary_with_metadata() {
        // Binary data with metadata should include metadata in summary
        let binary_data = vec![0xAB; 4];
        let msg = GenericMessage::with_metadata(binary_data, "my_type".to_string()).unwrap();
        let summary = msg.log_summary();
        assert!(summary.contains("meta: my_type"), "summary should include metadata: {}", summary);
    }

    // ============================================================================
    // Edge Case Tests
    // ============================================================================

    #[test]
    fn test_generic_message_copy_preserves_all_fields() {
        let msg = GenericMessage::with_metadata(vec![10, 20, 30], "test_meta".to_string()).unwrap();
        let copy = msg;
        assert_eq!(copy.data(), msg.data());
        assert_eq!(copy.metadata(), msg.metadata());
        assert_eq!(copy.inline_len, msg.inline_len);
        assert_eq!(copy.overflow_len, msg.overflow_len);
    }

    #[test]
    fn test_generic_message_debug_contains_field_names() {
        let msg = GenericMessage::new(vec![1, 2, 3]).unwrap();
        let debug_str = format!("{:?}", msg);
        // Debug output should reference struct field names
        assert!(debug_str.contains("GenericMessage"), "debug should name the struct: {}", debug_str);
        assert!(debug_str.contains("inline_len"), "debug should include field names: {}", debug_str);
    }

    // ============================================================================
    // Serialization Round-trip Tests
    // ============================================================================

    #[test]
    fn test_generic_message_json_roundtrip_with_overflow() {
        // Test round-trip with data that uses overflow buffer
        let data: Vec<u8> = (0..300).map(|i| (i % 256) as u8).collect();
        let msg = GenericMessage::new(data.clone()).unwrap();
        let json = serde_json::to_string(&msg).unwrap();
        let restored: GenericMessage = serde_json::from_str(&json).unwrap();
        assert_eq!(restored.data(), data);
        assert_eq!(restored.inline_len, 256);
        assert_eq!(restored.overflow_len, 44);
    }

    #[test]
    fn test_generic_message_json_roundtrip_with_metadata() {
        let data = vec![42; 10];
        let msg = GenericMessage::with_metadata(data.clone(), "roundtrip_test".to_string()).unwrap();
        let json = serde_json::to_string(&msg).unwrap();
        let restored: GenericMessage = serde_json::from_str(&json).unwrap();
        assert_eq!(restored.data(), data);
        assert_eq!(restored.metadata(), Some("roundtrip_test".to_string()));
    }

    #[test]
    fn test_generic_message_json_roundtrip_empty() {
        let msg = GenericMessage::new(vec![]).unwrap();
        let json = serde_json::to_string(&msg).unwrap();
        let restored: GenericMessage = serde_json::from_str(&json).unwrap();
        assert!(restored.data().is_empty());
        assert_eq!(restored.inline_len, 0);
        assert_eq!(restored.overflow_len, 0);
        assert_eq!(restored.metadata(), None);
    }

    #[test]
    fn test_generic_message_from_value_roundtrip_nested_struct() {
        // Test from_value/to_value with a nested structure
        #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
        struct Inner {
            values: Vec<f64>,
        }
        #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
        struct Outer {
            name: String,
            inner: Inner,
            count: u32,
        }

        let original = Outer {
            name: "test".into(),
            inner: Inner { values: vec![1.0, 2.5, -3.7] },
            count: 42,
        };
        let msg = GenericMessage::from_value(&original).unwrap();
        let restored: Outer = msg.to_value().unwrap();
        assert_eq!(restored, original);
    }

    #[test]
    fn test_generic_message_new_max_size_preserves_content() {
        // Maximum payload: verify all bytes survive, not just length
        let data: Vec<u8> = (0..MAX_GENERIC_PAYLOAD).map(|i| (i % 256) as u8).collect();
        let msg = GenericMessage::new(data.clone()).unwrap();
        assert_eq!(msg.data(), data, "max-size payload content must survive round-trip");
        assert_eq!(msg.inline_len as usize, INLINE_BUFFER_SIZE);
        assert_eq!(msg.overflow_len as usize, MAX_GENERIC_PAYLOAD - INLINE_BUFFER_SIZE);
    }

    #[test]
    fn test_generic_message_boundary_257_bytes() {
        // Exactly 1 byte into overflow
        let data: Vec<u8> = vec![0xCC; 257];
        let msg = GenericMessage::new(data.clone()).unwrap();
        assert_eq!(msg.data(), data);
        assert_eq!(msg.inline_len as usize, INLINE_BUFFER_SIZE);
        assert_eq!(msg.overflow_len, 1);
    }

    #[test]
    fn test_generic_message_metadata_boundary_255() {
        // Metadata at exact limit (255 bytes)
        let meta = "m".repeat(255);
        let msg = GenericMessage::with_metadata(vec![1], meta.clone()).unwrap();
        assert_eq!(msg.metadata(), Some(meta));
    }

    #[test]
    fn test_generic_message_to_value_wrong_type_fails() {
        // Serialize a string, try to deserialize as Vec<i32> — should fail
        let msg = GenericMessage::from_value(&"hello".to_string()).unwrap();
        let result: Result<Vec<i32>, _> = msg.to_value();
        assert!(result.is_err(), "deserializing string as Vec<i32> should fail");
    }
}
