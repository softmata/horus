// ONNX Inference Node for HORUS
//
// Generic ONNX model inference with support for various model types:
// - Image classification (ResNet, MobileNet, EfficientNet)
// - Object detection (YOLO, SSD, Faster R-CNN)
// - Semantic segmentation (DeepLab, U-Net)
// - Pose estimation (OpenPose, MediaPipe)
// - Custom models
//
// # Features
// - CPU and GPU execution providers
// - Batch inference support
// - Dynamic input/output tensor handling
// - Preprocessing pipeline (resize, normalize, etc.)
// - Performance metrics tracking
//
// # Example
// ```rust,ignore
// use horus::prelude::*;
// use horus_library::nodes::ml_inference::ONNXInferenceNode;
//
// fn main() -> Result<()> {
//     let mut scheduler = Scheduler::new();
//
//     let inference_node = ONNXInferenceNode::new(
//         "models/mobilenet_v2.onnx",
//         "camera/raw",
//         "ml/predictions",
//         InferenceConfig::default()
//     )?;
//
//     scheduler.add(Box::new(inference_node), 1, Some(true));
//     scheduler.run()?;
//     Ok(())
// }
// ```

#[cfg(feature = "onnx")]
use ort::session::{builder::GraphOptimizationLevel, Session};
#[cfg(feature = "onnx")]
use ort::value::Tensor as OrtTensor;

use crate::messages::ml::{InferenceMetrics, ModelFormat, ModelInfo, Predictions, Tensor};
use crate::messages::Image;
use horus_core::{HorusError, HorusResult, Node, NodeInfo, Topic};
use std::path::Path;
use std::time::Instant;

#[cfg(feature = "onnx")]
use ndarray::{Array, ArrayD, IxDyn};

/// Configuration for ONNX inference
#[derive(Clone, Debug)]
pub struct InferenceConfig {
    /// Batch size for inference
    pub batch_size: usize,
    /// Use GPU if available
    pub use_gpu: bool,
    /// GPU device ID
    pub device_id: u32,
    /// Confidence threshold for filtering predictions
    pub confidence_threshold: f32,
    /// Enable preprocessing (resize, normalize)
    pub enable_preprocessing: bool,
    /// Target input size [height, width]
    pub input_size: Option<[u32; 2]>,
    /// Normalization mean [R, G, B]
    pub norm_mean: [f32; 3],
    /// Normalization std [R, G, B]
    pub norm_std: [f32; 3],
    /// Graph optimization level
    pub optimization_level: u8,
}

impl Default for InferenceConfig {
    fn default() -> Self {
        Self {
            batch_size: 1,
            use_gpu: false,
            device_id: 0,
            confidence_threshold: 0.5,
            enable_preprocessing: true,
            input_size: None,
            norm_mean: [0.485, 0.456, 0.406], // ImageNet default
            norm_std: [0.229, 0.224, 0.225],  // ImageNet default
            optimization_level: 3,
        }
    }
}

/// ONNX Inference Node
///
/// Generic node for running ONNX model inference on incoming data.
/// Supports both image and tensor inputs.
#[cfg(feature = "onnx")]
pub struct ONNXInferenceNode {
    /// Input hub for images
    image_sub: Option<Topic<Image>>,
    /// Input hub for tensors
    tensor_sub: Option<Topic<Tensor>>,
    /// Output hub for predictions
    predictions_pub: Topic<Predictions>,
    /// Output hub for raw tensors
    tensor_pub: Option<Topic<Tensor>>,
    /// Output hub for metrics
    metrics_pub: Topic<InferenceMetrics>,
    /// ONNX Runtime session
    session: Session,
    /// Inference configuration
    config: InferenceConfig,
    /// Model metadata
    model_info: ModelInfo,
    /// Frame counter
    frame_count: u64,
}

#[cfg(feature = "onnx")]
impl ONNXInferenceNode {
    /// Create a new ONNX inference node with image input
    pub fn new_image(
        model_path: &str,
        input_topic: &str,
        output_topic: &str,
        config: InferenceConfig,
    ) -> HorusResult<Self> {
        let session = Self::load_model(model_path, &config)?;
        let model_info = Self::extract_model_info(&session, model_path)?;

        Ok(Self {
            image_sub: Some(Topic::new(input_topic)?),
            tensor_sub: None,
            predictions_pub: Topic::new(output_topic)?,
            tensor_pub: None,
            metrics_pub: Topic::new(&format!("{}.metrics", output_topic))?,
            session,
            config,
            model_info,
            frame_count: 0,
        })
    }

    /// Create a new ONNX inference node with tensor input
    pub fn new_tensor(
        model_path: &str,
        input_topic: &str,
        output_topic: &str,
        config: InferenceConfig,
    ) -> HorusResult<Self> {
        let session = Self::load_model(model_path, &config)?;
        let model_info = Self::extract_model_info(&session, model_path)?;

        Ok(Self {
            image_sub: None,
            tensor_sub: Some(Topic::new(input_topic)?),
            predictions_pub: Topic::new(output_topic)?,
            tensor_pub: Some(Topic::new(&format!("{}.tensor", output_topic))?),
            metrics_pub: Topic::new(&format!("{}.metrics", output_topic))?,
            session,
            config,
            model_info,
            frame_count: 0,
        })
    }

    /// Load ONNX model and create session
    fn load_model(model_path: &str, config: &InferenceConfig) -> HorusResult<Session> {
        if !Path::new(model_path).exists() {
            return Err(HorusError::Config(format!(
                "Model file not found: {}",
                model_path
            )));
        }

        let optimization_level = match config.optimization_level {
            0 => GraphOptimizationLevel::Disable,
            1 => GraphOptimizationLevel::Level1,
            2 => GraphOptimizationLevel::Level2,
            _ => GraphOptimizationLevel::Level3,
        };

        #[cfg_attr(not(feature = "cuda"), allow(unused_mut))]
        let mut builder = Session::builder()
            .map_err(|e| HorusError::Config(format!("Failed to create session builder: {}", e)))?
            .with_optimization_level(optimization_level)
            .map_err(|e| HorusError::Config(format!("Failed to set optimization level: {}", e)))?;

        // Add GPU execution provider if requested
        #[cfg(feature = "cuda")]
        if config.use_gpu {
            use ort::ExecutionProvider;
            builder = builder
                .with_execution_providers([ExecutionProvider::CUDA(Default::default())])
                .map_err(|e| HorusError::Config(format!("Failed to enable CUDA: {}", e)))?;
        }

        #[cfg(not(feature = "cuda"))]
        if config.use_gpu {
            eprintln!("Warning: GPU requested but CUDA support not compiled. Using CPU.");
        }

        let session = builder
            .commit_from_file(model_path)
            .map_err(|e| HorusError::Config(format!("Failed to load ONNX model: {}", e)))?;

        Ok(session)
    }

    /// Extract model metadata
    fn extract_model_info(session: &Session, model_path: &str) -> HorusResult<ModelInfo> {
        let inputs = &session.inputs;
        let outputs = &session.outputs;

        let input_names: Vec<String> = inputs.iter().map(|i| i.name.to_string()).collect();
        let output_names: Vec<String> = outputs.iter().map(|o| o.name.to_string()).collect();

        // Note: Shape extraction from ValueType is not available in ort 2.0 API
        // Shape information should be obtained from the actual inference outputs
        let input_shapes: Vec<Vec<usize>> = vec![vec![]; inputs.len()];
        let output_shapes: Vec<Vec<usize>> = vec![vec![]; outputs.len()];

        let model_name = Path::new(model_path)
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("unknown")
            .to_string();

        Ok(ModelInfo {
            name: model_name,
            version: "1.0.0".to_string(),
            format: ModelFormat::ONNX,
            input_shapes,
            output_shapes,
            input_names,
            output_names,
            metadata: std::collections::HashMap::new(),
        })
    }

    /// Preprocess image for inference
    fn preprocess_image(&self, image: &Image) -> HorusResult<ArrayD<f32>> {
        let width = image.width as usize;
        let height = image.height as usize;

        // Determine target size
        let (target_h, target_w) = if let Some([h, w]) = self.config.input_size {
            (h as usize, w as usize)
        } else {
            // Use model input size
            if let Some(shape) = self.model_info.input_shapes.first() {
                if shape.len() == 4 {
                    // NCHW format
                    (shape[2], shape[3])
                } else {
                    return Err(HorusError::Config(
                        "Unexpected input shape format".to_string(),
                    ));
                }
            } else {
                return Err(HorusError::Config("No input shape found".to_string()));
            }
        };

        // Convert image data to float and resize if needed
        let mut tensor = if width != target_w || height != target_h {
            // Need to resize - use basic nearest neighbor for now
            // In production, use image crate for better quality
            self.resize_image(&image.data, width, height, target_w, target_h)?
        } else {
            // Direct conversion
            let mut data = Vec::with_capacity(width * height * 3);
            for &pixel in &image.data {
                data.push(pixel as f32 / 255.0);
            }
            Array::from_shape_vec(IxDyn(&[height, width, 3]), data)
                .map_err(|e| HorusError::Config(format!("Array reshape failed: {}", e)))?
        };

        // Normalize if enabled
        if self.config.enable_preprocessing {
            let shape = tensor.shape().to_vec();
            let (mut data, _offset) = tensor.into_raw_vec_and_offset();

            for y in 0..target_h {
                for x in 0..target_w {
                    for c in 0..3 {
                        let idx = y * target_w * 3 + x * 3 + c;
                        data[idx] =
                            (data[idx] - self.config.norm_mean[c]) / self.config.norm_std[c];
                    }
                }
            }

            tensor = Array::from_shape_vec(IxDyn(&shape), data)
                .map_err(|e| HorusError::Config(format!("Array reshape failed: {}", e)))?;
        }

        // Convert HWC to NCHW (batch, channels, height, width)
        let hwc = tensor
            .into_shape_with_order(IxDyn(&[target_h, target_w, 3]))
            .map_err(|e| HorusError::Config(format!("HWC reshape failed: {}", e)))?;

        let mut nchw_data = vec![0.0f32; 1 * 3 * target_h * target_w];

        for c in 0..3 {
            for y in 0..target_h {
                for x in 0..target_w {
                    let src_idx = y * target_w * 3 + x * 3 + c;
                    let dst_idx = c * target_h * target_w + y * target_w + x;
                    nchw_data[dst_idx] = hwc.as_slice().unwrap()[src_idx];
                }
            }
        }

        let nchw = Array::from_shape_vec(IxDyn(&[1, 3, target_h, target_w]), nchw_data)
            .map_err(|e| HorusError::Config(format!("NCHW reshape failed: {}", e)))?;

        Ok(nchw)
    }

    /// Basic image resize (nearest neighbor)
    fn resize_image(
        &self,
        data: &[u8],
        src_w: usize,
        src_h: usize,
        dst_w: usize,
        dst_h: usize,
    ) -> HorusResult<ArrayD<f32>> {
        let mut resized = vec![0.0f32; dst_h * dst_w * 3];

        let x_ratio = src_w as f32 / dst_w as f32;
        let y_ratio = src_h as f32 / dst_h as f32;

        for y in 0..dst_h {
            for x in 0..dst_w {
                let src_x = (x as f32 * x_ratio) as usize;
                let src_y = (y as f32 * y_ratio) as usize;

                for c in 0..3 {
                    let src_idx = (src_y * src_w + src_x) * 3 + c;
                    let dst_idx = (y * dst_w + x) * 3 + c;
                    resized[dst_idx] = data[src_idx] as f32 / 255.0;
                }
            }
        }

        Array::from_shape_vec(IxDyn(&[dst_h, dst_w, 3]), resized)
            .map_err(|e| HorusError::Config(format!("Resize reshape failed: {}", e)))
    }

    /// Run inference on preprocessed tensor
    fn run_inference(&mut self, input: ArrayD<f32>) -> HorusResult<Vec<ArrayD<f32>>> {
        // Create ONNX tensor from ndarray
        let input_tensor = OrtTensor::from_array(input)
            .map_err(|e| HorusError::config(format!("Failed to create input tensor: {}", e)))?;

        // Run inference
        let outputs = self
            .session
            .run(ort::inputs![input_tensor])
            .map_err(|e| HorusError::config(format!("Inference failed: {}", e)))?;

        // Convert outputs to ndarray
        let mut result_tensors = Vec::new();
        for i in 0..outputs.len() {
            let array_view: ndarray::ArrayViewD<f32> =
                outputs[i].try_extract_array().map_err(|e| {
                    HorusError::config(format!("Failed to extract tensor {}: {}", i, e))
                })?;

            result_tensors.push(array_view.to_owned());
        }

        Ok(result_tensors)
    }

    /// Process inference output into predictions
    fn postprocess_output(&self, outputs: Vec<ArrayD<f32>>) -> HorusResult<Predictions> {
        if outputs.is_empty() {
            return Ok(Predictions {
                class_ids: vec![],
                scores: vec![],
                class_names: None,
                metadata: std::collections::HashMap::new(),
            });
        }

        // Simple classification postprocessing
        // For detection/segmentation, this would be more complex
        let output = &outputs[0];
        let data = output
            .as_slice()
            .ok_or_else(|| HorusError::config("Failed to get output slice".to_string()))?;

        // Find top-K predictions
        let mut indexed: Vec<(usize, f32)> =
            data.iter().enumerate().map(|(i, &v)| (i, v)).collect();

        indexed.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

        let top_k = 5.min(indexed.len());
        let class_ids: Vec<u32> = indexed.iter().take(top_k).map(|(i, _)| *i as u32).collect();
        let scores: Vec<f32> = indexed.iter().take(top_k).map(|(_, s)| *s).collect();

        Ok(Predictions {
            class_ids,
            scores,
            class_names: None,
            metadata: std::collections::HashMap::new(),
        })
    }
}

#[cfg(feature = "onnx")]
impl Node for ONNXInferenceNode {
    fn name(&self) -> &'static str {
        "ONNXInferenceNode"
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Try to receive image
        if let Some(ref image_sub) = self.image_sub {
            if let Some(image) = image_sub.recv(&mut ctx) {
                let start = Instant::now();

                // Preprocess
                let input = match self.preprocess_image(&image) {
                    Ok(t) => t,
                    Err(e) => {
                        eprintln!("Preprocessing failed: {}", e);
                        return;
                    }
                };

                // Run inference
                let outputs = match self.run_inference(input) {
                    Ok(o) => o,
                    Err(e) => {
                        eprintln!("Inference failed: {}", e);
                        return;
                    }
                };

                // Postprocess
                let predictions = match self.postprocess_output(outputs) {
                    Ok(p) => p,
                    Err(e) => {
                        eprintln!("Postprocessing failed: {}", e);
                        return;
                    }
                };

                let latency = start.elapsed().as_secs_f32() * 1000.0;

                // Send predictions
                let _ = self.predictions_pub.send(predictions, &mut ctx);

                // Send metrics
                let metrics = InferenceMetrics {
                    latency_ms: latency,
                    throughput: 1000.0 / latency,
                    model_name: self.model_info.name.clone(),
                    batch_size: self.config.batch_size,
                    timestamp_ns: image.timestamp,
                };
                let _ = self.metrics_pub.send(metrics, &mut ctx);

                self.frame_count += 1;
            }
        }

        // Try to receive tensor
        if let Some(ref tensor_sub) = self.tensor_sub {
            if let Some(tensor) = tensor_sub.recv(&mut ctx) {
                let start = Instant::now();

                // Convert HORUS tensor to ndarray
                let shape: Vec<usize> = tensor.shape.clone();
                let input = match Array::from_shape_vec(IxDyn(&shape), tensor.data.clone()) {
                    Ok(arr) => arr,
                    Err(e) => {
                        eprintln!("Tensor conversion failed: {}", e);
                        return;
                    }
                };

                // Run inference
                let outputs = match self.run_inference(input) {
                    Ok(o) => o,
                    Err(e) => {
                        eprintln!("Inference failed: {}", e);
                        return;
                    }
                };

                let latency = start.elapsed().as_secs_f32() * 1000.0;

                // Send raw tensor output if pub exists
                if let Some(ref tensor_pub) = self.tensor_pub {
                    if let Some(output) = outputs.first() {
                        let output_tensor = Tensor {
                            data: output.as_slice().unwrap_or(&[]).to_vec(),
                            shape: output.shape().iter().map(|&d| d as usize).collect(),
                            dtype: crate::messages::ml::DataType::Float32,
                            name: Some(self.model_info.name.clone()),
                        };
                        let _ = tensor_pub.send(output_tensor, &mut ctx);
                    }
                }

                // Send metrics
                let metrics = InferenceMetrics {
                    latency_ms: latency,
                    throughput: 1000.0 / latency,
                    model_name: self.model_info.name.clone(),
                    batch_size: self.config.batch_size,
                    timestamp_ns: 0,
                };
                let _ = self.metrics_pub.send(metrics, &mut ctx);

                self.frame_count += 1;
            }
        }
    }
}

// Stub implementation when ONNX feature is disabled
#[cfg(not(feature = "onnx"))]
pub struct ONNXInferenceNode;

#[cfg(not(feature = "onnx"))]
impl ONNXInferenceNode {
    pub fn new_image(
        _model_path: &str,
        _input_topic: &str,
        _output_topic: &str,
        _config: InferenceConfig,
    ) -> HorusResult<Self> {
        Err(HorusError::Config(
            "ONNX support not compiled. Enable 'onnx' feature.".to_string(),
        ))
    }

    pub fn new_tensor(
        _model_path: &str,
        _input_topic: &str,
        _output_topic: &str,
        _config: InferenceConfig,
    ) -> HorusResult<Self> {
        Err(HorusError::Config(
            "ONNX support not compiled. Enable 'onnx' feature.".to_string(),
        ))
    }
}

#[cfg(not(feature = "onnx"))]
impl Node for ONNXInferenceNode {
    fn name(&self) -> &'static str {
        "ONNXInferenceNode"
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {}
}
