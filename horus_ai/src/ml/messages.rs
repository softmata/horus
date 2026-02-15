// Machine Learning message types for HORUS
//
// This module contains message types for ML model inference, training,
// and deployment in robotics applications.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Data type for tensor elements
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DataType {
    Float32,
    Float64,
    Int8,
    Int16,
    Int32,
    Int64,
    UInt8,
    UInt16,
    UInt32,
    UInt64,
    Bool,
}

/// Generic tensor for ML model inputs and outputs
///
/// Supports multi-dimensional arrays with various data types.
/// Data is stored as flattened `Vec<f32>` for simplicity - conversion
/// from other types should happen at the node level.
///
/// # Example
/// ```rust,ignore
/// use horus_library::messages::ml::Tensor;
///
/// // Create a 3x3 tensor
/// let data = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
/// let tensor = Tensor::new(data, vec![3, 3], DataType::Float32);
/// ```
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Tensor {
    /// Flattened tensor data (row-major order)
    pub data: Vec<f32>,
    /// Shape of the tensor (e.g., [batch, channels, height, width])
    pub shape: Vec<usize>,
    /// Data type of tensor elements
    pub dtype: DataType,
    /// Optional name for the tensor
    pub name: Option<String>,
}

impl Tensor {
    /// Create a new tensor with given data, shape, and dtype
    pub fn new(data: Vec<f32>, shape: Vec<usize>, dtype: DataType) -> Self {
        Self {
            data,
            shape,
            dtype,
            name: None,
        }
    }

    /// Create a tensor with a name
    pub fn with_name(mut self, name: String) -> Self {
        self.name = Some(name);
        self
    }

    /// Get the total number of elements
    pub fn size(&self) -> usize {
        self.shape.iter().product()
    }

    /// Get the number of dimensions
    pub fn ndim(&self) -> usize {
        self.shape.len()
    }
}

/// Generic predictions from ML models
///
/// Used for classification, detection, or any task that produces
/// class IDs with confidence scores.
///
/// # Example
/// ```rust,ignore
/// use horus_library::messages::ml::Predictions;
///
/// let preds = Predictions {
///     class_ids: vec![0, 1, 2],
///     scores: vec![0.95, 0.88, 0.72],
///     class_names: Some(vec!["person".into(), "car".into(), "dog".into()]),
///     metadata: HashMap::new(),
/// };
/// ```
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Predictions {
    /// Predicted class IDs
    pub class_ids: Vec<u32>,
    /// Confidence scores for each prediction (0.0 to 1.0)
    pub scores: Vec<f32>,
    /// Optional class names
    pub class_names: Option<Vec<String>>,
    /// Additional metadata (model version, preprocessing params, etc.)
    pub metadata: HashMap<String, String>,
}

/// Inference performance metrics
///
/// Tracks timing and throughput for ML model inference.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct InferenceMetrics {
    /// Inference latency in milliseconds
    pub latency_ms: f32,
    /// Throughput in samples per second
    pub throughput: f32,
    /// Model name or identifier
    pub model_name: String,
    /// Batch size used for inference
    pub batch_size: usize,
    /// Timestamp in nanoseconds
    pub timestamp_ns: u64,
}

/// ML model format
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ModelFormat {
    ONNX,
    TFLite,
    PyTorch,
    TensorFlow,
    Tract,
    TensorRT,
    CoreML,
}

/// Model metadata and configuration
///
/// Contains information about a loaded ML model including
/// input/output shapes and runtime configuration.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ModelInfo {
    /// Model name
    pub name: String,
    /// Model version (semver)
    pub version: String,
    /// Model format
    pub format: ModelFormat,
    /// Input tensor shapes
    pub input_shapes: Vec<Vec<usize>>,
    /// Output tensor shapes
    pub output_shapes: Vec<Vec<usize>>,
    /// Input tensor names
    pub input_names: Vec<String>,
    /// Output tensor names
    pub output_names: Vec<String>,
    /// Additional metadata
    pub metadata: HashMap<String, String>,
}

/// ML model detection output
///
/// Represents a single detected object from ML inference with bounding box,
/// class, and confidence score. Named `MlDetection` to distinguish from
/// `detection_pod::Detection` (zero-copy) in horus_library.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MlDetection {
    /// Bounding box [x, y, width, height] in pixels
    pub bbox: [f32; 4],
    /// Class ID
    pub class_id: u32,
    /// Class name (if available)
    pub class_name: Option<String>,
    /// Confidence score (0.0 to 1.0)
    pub confidence: f32,
    /// Optional tracking ID (for multi-object tracking)
    pub track_id: Option<u32>,
}

/// Array of ML model detections
///
/// Used by object detection models (YOLO, SSD, etc.) to return
/// multiple detections from a single image.
///
/// # Example
/// ```rust,ignore
/// use horus_library::messages::ml::{MlDetectionArray, MlDetection};
///
/// let detections = MlDetectionArray {
///     detections: vec![
///         MlDetection {
///             bbox: [100.0, 100.0, 200.0, 300.0],
///             class_id: 0,
///             class_name: Some("person".into()),
///             confidence: 0.95,
///             track_id: None,
///         }
///     ],
///     image_width: 640,
///     image_height: 480,
///     timestamp_ns: 0,
/// };
/// ```
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MlDetectionArray {
    /// List of detections
    pub detections: Vec<MlDetection>,
    /// Image width in pixels
    pub image_width: u32,
    /// Image height in pixels
    pub image_height: u32,
    /// Timestamp in nanoseconds
    pub timestamp_ns: u64,
}

/// Semantic segmentation mask
///
/// Per-pixel class predictions for semantic segmentation models.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SegmentationMask {
    /// Flattened mask data (HxW class IDs)
    pub mask: Vec<u8>,
    /// Mask width in pixels
    pub width: u32,
    /// Mask height in pixels
    pub height: u32,
    /// Number of classes
    pub num_classes: u32,
    /// Class names
    pub class_names: Vec<String>,
    /// Timestamp in nanoseconds
    pub timestamp_ns: u64,
}

/// Human pose keypoint
///
/// Represents a single keypoint in pose estimation. Named `PoseKeypoint`
/// to distinguish from `landmark::Landmark` (zero-copy Pod for IPC).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PoseKeypoint {
    /// X coordinate in pixels
    pub x: f32,
    /// Y coordinate in pixels
    pub y: f32,
    /// Z coordinate (3D pose, optional)
    pub z: Option<f32>,
    /// Confidence score (0.0 to 1.0)
    pub confidence: f32,
    /// Keypoint name (e.g., "nose", "left_shoulder")
    pub name: String,
}

/// Human pose estimation result
///
/// Contains detected keypoints for a single person.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Pose {
    /// List of keypoints (typically 17-33 points depending on model)
    pub keypoints: Vec<PoseKeypoint>,
    /// Overall pose confidence
    pub confidence: f32,
    /// Person/instance ID (for multi-person pose)
    pub person_id: u32,
    /// Bounding box of the person [x, y, width, height]
    pub bbox: Option<[f32; 4]>,
}

/// Multi-person pose estimation result
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PoseArray {
    /// List of detected poses
    pub poses: Vec<Pose>,
    /// Image width in pixels
    pub image_width: u32,
    /// Image height in pixels
    pub image_height: u32,
    /// Timestamp in nanoseconds
    pub timestamp_ns: u64,
}

/// Feature vector for embeddings
///
/// Used for feature extraction, similarity search, and transfer learning.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FeatureVector {
    /// Feature vector (typically 128-2048 dimensions)
    pub features: Vec<f32>,
    /// Source identifier (image path, object ID, etc.)
    pub source: Option<String>,
    /// Timestamp in nanoseconds
    pub timestamp_ns: u64,
}

/// Classification result
///
/// Top-K class predictions with probabilities.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Classification {
    /// Class IDs
    pub class_ids: Vec<u32>,
    /// Class names
    pub class_names: Vec<String>,
    /// Probabilities (0.0 to 1.0, sum to 1.0)
    pub probabilities: Vec<f32>,
    /// Timestamp in nanoseconds
    pub timestamp_ns: u64,
}

/// LLM generation request
///
/// Input for LLM inference nodes.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LLMRequest {
    /// Conversation messages
    pub messages: Vec<ChatMessage>,
}

/// Chat message for LLM conversations
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ChatMessage {
    /// Role: "system", "user", or "assistant"
    pub role: String,
    /// Message content
    pub content: String,
}

/// LLM generation response
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LLMResponse {
    /// Generated response text
    pub response: String,
    /// Total tokens used (input + output)
    pub tokens_used: u64,
    /// Inference time in milliseconds
    pub latency_ms: u64,
    /// Model name/ID
    pub model: String,
    /// Finish reason: "stop", "length", "error"
    pub finish_reason: String,
    /// Timestamp in nanoseconds
    pub timestamp_ns: u64,
}

impl horus_core::core::LogSummary for LLMRequest {
    fn log_summary(&self) -> String {
        if self.messages.is_empty() {
            return "LLMRequest { no messages }".to_string();
        }
        let last_msg = &self.messages[self.messages.len() - 1];
        let preview = if last_msg.content.len() > 50 {
            let end = last_msg
                .content
                .char_indices()
                .take_while(|&(i, _)| i < 50)
                .last()
                .map(|(i, c)| i + c.len_utf8())
                .unwrap_or(0);
            format!("{}...", &last_msg.content[..end])
        } else {
            last_msg.content.clone()
        };
        format!("LLMRequest {{ {}: \"{}\" }}", last_msg.role, preview)
    }
}

impl horus_core::core::LogSummary for LLMResponse {
    fn log_summary(&self) -> String {
        let preview = if self.response.len() > 50 {
            let end = self
                .response
                .char_indices()
                .take_while(|&(i, _)| i < 50)
                .last()
                .map(|(i, c)| i + c.len_utf8())
                .unwrap_or(0);
            format!("{}...", &self.response[..end])
        } else {
            self.response.clone()
        };
        format!(
            "LLMResponse {{ \"{}\", {} tokens, {:.1}ms }}",
            preview, self.tokens_used, self.latency_ms
        )
    }
}

impl horus_core::core::LogSummary for ChatMessage {
    fn log_summary(&self) -> String {
        let preview = if self.content.len() > 40 {
            let end = self
                .content
                .char_indices()
                .take_while(|&(i, _)| i < 40)
                .last()
                .map(|(i, c)| i + c.len_utf8())
                .unwrap_or(0);
            format!("{}...", &self.content[..end])
        } else {
            self.content.clone()
        };
        format!("ChatMessage {{ {}: \"{}\" }}", self.role, preview)
    }
}

/// Training metrics
///
/// Tracks training progress for online learning or fine-tuning.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TrainingMetrics {
    /// Current epoch
    pub epoch: u32,
    /// Current batch/step
    pub step: u32,
    /// Training loss
    pub loss: f32,
    /// Validation accuracy (if available)
    pub accuracy: Option<f32>,
    /// Learning rate
    pub learning_rate: f32,
    /// Timestamp in nanoseconds
    pub timestamp_ns: u64,
}

/// Trajectory point for imitation learning
///
/// Single observation-action pair for behavior cloning.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TrajectoryPoint {
    /// Observation (sensor data, image features, etc.)
    pub observation: Tensor,
    /// Action taken
    pub action: Tensor,
    /// Reward (optional)
    pub reward: Option<f32>,
    /// Done flag (episode termination)
    pub done: bool,
    /// Timestamp in nanoseconds
    pub timestamp_ns: u64,
}

/// Model deployment configuration
///
/// Configuration for deploying ML models in production.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DeploymentConfig {
    /// Model path or URL
    pub model_path: String,
    /// Model format
    pub format: ModelFormat,
    /// Execution provider: "cpu", "cuda", "tensorrt", "coreml"
    pub execution_provider: String,
    /// Batch size
    pub batch_size: usize,
    /// Use half precision (FP16)
    pub use_fp16: bool,
    /// Number of threads (CPU only)
    pub num_threads: Option<usize>,
    /// GPU device ID
    pub device_id: Option<u32>,
}

// LogSummary implementations
impl horus_core::core::LogSummary for InferenceMetrics {
    fn log_summary(&self) -> String {
        format!(
            "InferenceMetrics {{ {}, {:.1}ms, {:.1} fps }}",
            self.model_name, self.latency_ms, self.throughput
        )
    }
}

impl horus_core::core::LogSummary for PoseArray {
    fn log_summary(&self) -> String {
        format!("PoseArray {{ {} poses }}", self.poses.len())
    }
}

impl horus_core::core::LogSummary for SegmentationMask {
    fn log_summary(&self) -> String {
        format!(
            "SegmentationMask {{ {}x{}, {} classes }}",
            self.width, self.height, self.num_classes
        )
    }
}

impl horus_core::core::LogSummary for Tensor {
    fn log_summary(&self) -> String {
        format!(
            "Tensor {{ shape: {:?}, {} elements }}",
            self.shape,
            self.size()
        )
    }
}

impl horus_core::core::LogSummary for Predictions {
    fn log_summary(&self) -> String {
        format!(
            "Predictions {{ {} classes, avg_score: {:.3} }}",
            self.class_ids.len(),
            if self.scores.is_empty() {
                0.0
            } else {
                self.scores.iter().sum::<f32>() / self.scores.len() as f32
            }
        )
    }
}
