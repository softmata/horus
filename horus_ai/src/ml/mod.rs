// Machine Learning Infrastructure for HORUS
//
// Utilities for model management, loading, and deployment.
// All ML message types are canonical in horus_library::messages::ml.

pub mod messages;
pub mod model_loader;
pub mod model_registry;

pub use messages::{
    ChatMessage, Classification, DataType, DeploymentConfig, FeatureVector, InferenceMetrics,
    LLMRequest, LLMResponse, MlDetection, MlDetectionArray, MlTrajectoryPoint, ModelFormat,
    ModelInfo, Pose, PoseArray, PoseKeypoint, Predictions, Tensor, TrainingMetrics,
};
pub use model_loader::ModelLoader;
pub use model_registry::{ModelEntry, ModelRegistry};

// Re-export SegmentationMask from horus_library (the canonical POD version)
pub use horus_library::messages::SegmentationMask;
