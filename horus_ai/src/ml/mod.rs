// Machine Learning Infrastructure for HORUS
//
// Utilities for model management, loading, and deployment.

pub mod messages;
pub mod model_loader;
pub mod model_registry;

pub use messages::{
    ChatMessage, Classification, DataType, DeploymentConfig, FeatureVector, InferenceMetrics,
    LLMRequest, LLMResponse, MlDetection, MlDetectionArray, ModelFormat, ModelInfo, Pose,
    PoseArray, PoseKeypoint, Predictions, SegmentationMask, Tensor, TrainingMetrics,
};
pub use model_loader::ModelLoader;
pub use model_registry::{ModelEntry, ModelRegistry};
