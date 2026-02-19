// Machine Learning message types for HORUS
//
// All ML message types are defined in horus_library::messages::ml (the canonical location).
// This module re-exports them for backwards compatibility.

pub use horus_library::messages::ml::{
    ChatMessage, Classification, DataType, DeploymentConfig, FeatureVector, InferenceMetrics,
    LLMRequest, LLMResponse, MlDetection, MlDetectionArray, MlTrajectoryPoint, ModelFormat,
    ModelInfo, Pose, PoseArray, PoseKeypoint, Predictions, Tensor, TrainingMetrics,
};
