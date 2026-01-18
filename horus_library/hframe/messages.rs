//! HFrame Message types for inter-node communication
//!
//! Provides message types compatible with HORUS Hub for broadcasting
//! and receiving transform updates.

use super::transform::Transform;
use horus_core::bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

/// Maximum number of transforms in a batch message
pub const MAX_TRANSFORMS_PER_MESSAGE: usize = 32;

/// Frame ID buffer size
pub const FRAME_ID_SIZE: usize = 64;

/// Convert frame ID bytes to string
pub fn frame_id_to_string(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes)
        .trim_end_matches('\0')
        .to_string()
}

/// Copy string to fixed-size frame ID buffer
pub fn string_to_frame_id(s: &str, buffer: &mut [u8]) {
    let bytes = s.as_bytes();
    let len = bytes.len().min(buffer.len() - 1);
    buffer[..len].copy_from_slice(&bytes[..len]);
    buffer[len..].fill(0);
}

/// Stamped transform message (with timestamp)
///
/// Used for dynamic transforms that change over time.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[repr(C)]
pub struct TransformStamped {
    /// Parent frame ID (e.g., "base_link")
    #[serde(with = "serde_arrays")]
    pub parent_frame: [u8; FRAME_ID_SIZE],
    /// Child frame ID (e.g., "camera_frame")
    #[serde(with = "serde_arrays")]
    pub child_frame: [u8; FRAME_ID_SIZE],
    /// Timestamp in nanoseconds since UNIX epoch
    pub timestamp: u64,
    /// The transform from parent to child frame
    pub transform: Transform,
}

unsafe impl Pod for TransformStamped {}
unsafe impl Zeroable for TransformStamped {}

impl Default for TransformStamped {
    fn default() -> Self {
        Self {
            parent_frame: [0u8; FRAME_ID_SIZE],
            child_frame: [0u8; FRAME_ID_SIZE],
            timestamp: 0,
            transform: Transform::identity(),
        }
    }
}

impl TransformStamped {
    /// Create a new stamped transform
    pub fn new(parent: &str, child: &str, timestamp: u64, transform: Transform) -> Self {
        let mut msg = Self::default();
        string_to_frame_id(parent, &mut msg.parent_frame);
        string_to_frame_id(child, &mut msg.child_frame);
        msg.timestamp = timestamp;
        msg.transform = transform;
        msg
    }

    /// Get parent frame ID as string
    pub fn parent_frame_id(&self) -> String {
        frame_id_to_string(&self.parent_frame)
    }

    /// Get child frame ID as string
    pub fn child_frame_id(&self) -> String {
        frame_id_to_string(&self.child_frame)
    }

    /// Set parent frame ID
    pub fn set_parent_frame(&mut self, parent: &str) {
        string_to_frame_id(parent, &mut self.parent_frame);
    }

    /// Set child frame ID
    pub fn set_child_frame(&mut self, child: &str) {
        string_to_frame_id(child, &mut self.child_frame);
    }
}

/// Static transform message (never changes)
///
/// Used for fixed transforms like sensor mounts on a robot.
/// These are only published once and cached by listeners.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[repr(C)]
pub struct StaticTransformStamped {
    /// Parent frame ID
    #[serde(with = "serde_arrays")]
    pub parent_frame: [u8; FRAME_ID_SIZE],
    /// Child frame ID
    #[serde(with = "serde_arrays")]
    pub child_frame: [u8; FRAME_ID_SIZE],
    /// The transform from parent to child frame
    pub transform: Transform,
}

unsafe impl Pod for StaticTransformStamped {}
unsafe impl Zeroable for StaticTransformStamped {}

impl Default for StaticTransformStamped {
    fn default() -> Self {
        Self {
            parent_frame: [0u8; FRAME_ID_SIZE],
            child_frame: [0u8; FRAME_ID_SIZE],
            transform: Transform::identity(),
        }
    }
}

impl StaticTransformStamped {
    /// Create a new static transform
    pub fn new(parent: &str, child: &str, transform: Transform) -> Self {
        let mut msg = Self::default();
        string_to_frame_id(parent, &mut msg.parent_frame);
        string_to_frame_id(child, &mut msg.child_frame);
        msg.transform = transform;
        msg
    }

    /// Get parent frame ID as string
    pub fn parent_frame_id(&self) -> String {
        frame_id_to_string(&self.parent_frame)
    }

    /// Get child frame ID as string
    pub fn child_frame_id(&self) -> String {
        frame_id_to_string(&self.child_frame)
    }

    /// Convert to TransformStamped with given timestamp
    pub fn to_stamped(&self, timestamp: u64) -> TransformStamped {
        TransformStamped {
            parent_frame: self.parent_frame,
            child_frame: self.child_frame,
            timestamp,
            transform: self.transform,
        }
    }
}

/// Batch of transforms for efficient transmission
///
/// Allows sending multiple transforms in a single message.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[repr(C)]
pub struct TFMessage {
    /// Array of transforms
    pub transforms: [TransformStamped; MAX_TRANSFORMS_PER_MESSAGE],
    /// Number of valid transforms in the array
    pub count: u32,
    /// Padding for alignment
    _padding: [u8; 4],
}

unsafe impl Pod for TFMessage {}
unsafe impl Zeroable for TFMessage {}

impl Default for TFMessage {
    fn default() -> Self {
        Self {
            transforms: [TransformStamped::default(); MAX_TRANSFORMS_PER_MESSAGE],
            count: 0,
            _padding: [0; 4],
        }
    }
}

impl TFMessage {
    /// Create a new empty TF message
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a transform to the batch
    ///
    /// Returns false if the batch is full.
    pub fn add(&mut self, transform: TransformStamped) -> bool {
        if (self.count as usize) < MAX_TRANSFORMS_PER_MESSAGE {
            self.transforms[self.count as usize] = transform;
            self.count += 1;
            true
        } else {
            false
        }
    }

    /// Get the number of transforms in the batch
    pub fn len(&self) -> usize {
        self.count as usize
    }

    /// Check if the batch is empty
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Check if the batch is full
    pub fn is_full(&self) -> bool {
        self.count as usize >= MAX_TRANSFORMS_PER_MESSAGE
    }

    /// Clear the batch
    pub fn clear(&mut self) {
        self.count = 0;
    }

    /// Iterate over valid transforms
    pub fn iter(&self) -> impl Iterator<Item = &TransformStamped> {
        self.transforms[..self.count as usize].iter()
    }

    /// Create from a vector of transforms
    pub fn from_vec(transforms: Vec<TransformStamped>) -> Self {
        let mut msg = Self::new();
        for tf in transforms.into_iter().take(MAX_TRANSFORMS_PER_MESSAGE) {
            msg.add(tf);
        }
        msg
    }

    /// Convert to vector of transforms
    pub fn to_vec(&self) -> Vec<TransformStamped> {
        self.transforms[..self.count as usize].to_vec()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transform_stamped_new() {
        let tf = TransformStamped::new(
            "world",
            "robot",
            1234567890,
            Transform::from_translation([1.0, 2.0, 3.0]),
        );

        assert_eq!(tf.parent_frame_id(), "world");
        assert_eq!(tf.child_frame_id(), "robot");
        assert_eq!(tf.timestamp, 1234567890);
    }

    #[test]
    fn test_static_transform_stamped() {
        let tf = StaticTransformStamped::new(
            "base_link",
            "camera",
            Transform::from_translation([0.5, 0.0, 0.2]),
        );

        assert_eq!(tf.parent_frame_id(), "base_link");
        assert_eq!(tf.child_frame_id(), "camera");

        let stamped = tf.to_stamped(12345);
        assert_eq!(stamped.timestamp, 12345);
    }

    #[test]
    fn test_tf_message_batch() {
        let mut batch = TFMessage::new();
        assert!(batch.is_empty());

        let tf1 = TransformStamped::new("a", "b", 1, Transform::identity());
        let tf2 = TransformStamped::new("b", "c", 2, Transform::identity());

        assert!(batch.add(tf1));
        assert!(batch.add(tf2));
        assert_eq!(batch.len(), 2);

        let vec = batch.to_vec();
        assert_eq!(vec.len(), 2);
        assert_eq!(vec[0].parent_frame_id(), "a");
        assert_eq!(vec[1].parent_frame_id(), "b");
    }

    #[test]
    fn test_tf_message_full() {
        let mut batch = TFMessage::new();

        for i in 0..MAX_TRANSFORMS_PER_MESSAGE {
            let tf = TransformStamped::new(
                &format!("f{}", i),
                &format!("f{}", i + 1),
                i as u64,
                Transform::identity(),
            );
            assert!(batch.add(tf));
        }

        assert!(batch.is_full());

        let extra = TransformStamped::new("x", "y", 0, Transform::identity());
        assert!(!batch.add(extra));
    }

    #[test]
    fn test_pod_safety() {
        // Ensure types are Pod-safe (can be safely transmuted)
        let ts = TransformStamped::default();
        let bytes: &[u8] = horus_core::bytemuck::bytes_of(&ts);
        assert!(!bytes.is_empty());

        let sts = StaticTransformStamped::default();
        let bytes: &[u8] = horus_core::bytemuck::bytes_of(&sts);
        assert!(!bytes.is_empty());

        let msg = TFMessage::default();
        let bytes: &[u8] = horus_core::bytemuck::bytes_of(&msg);
        assert!(!bytes.is_empty());
    }

    #[test]
    fn test_frame_id_conversion() {
        let mut buffer = [0u8; 64];
        string_to_frame_id("base_link", &mut buffer);
        assert_eq!(frame_id_to_string(&buffer), "base_link");
    }

    #[test]
    fn test_frame_id_truncation() {
        let mut buffer = [0u8; 8];
        string_to_frame_id("very_long_frame_name", &mut buffer);
        assert_eq!(frame_id_to_string(&buffer), "very_lo");
    }
}
