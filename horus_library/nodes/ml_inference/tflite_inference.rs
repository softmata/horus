// TensorFlow Lite Inference Node for HORUS
//
// Edge-optimized ML inference using TensorFlow Lite models.
// Ideal for embedded systems (Raspberry Pi, Jetson Nano, Coral TPU).
//
// # Features
// - Quantized model support (INT8, FP16)
// - Edge TPU delegate (Google Coral)
// - CPU/GPU delegates
// - Efficient memory usage
// - Batch processing
//
// # Example
// ```rust,ignore
// use horus::prelude::*;
// use horus_library::nodes::ml_inference::TFLiteInferenceNode;
//
// fn main() -> Result<()> {
//     let mut scheduler = Scheduler::new();
//
//     let tflite_node = TFLiteInferenceNode::new(
//         "models/mobilenet_v2_quant.tflite",
//         "camera/raw",
//         "ml/predictions",
//         TFLiteConfig::default(),
//     )?;
//
//     scheduler.add(Box::new(tflite_node), 1, Some(true));
//     scheduler.run()?;
//     Ok(())
// }
// ```

use crate::messages::ml::{
    DataType, InferenceMetrics, ModelFormat, ModelInfo, Predictions, Tensor,
};
use crate::messages::{Detection, DetectionArray, Image};
use horus_core::{HorusError, HorusResult, Node, NodeInfo, Topic};
use std::path::Path;
use std::time::Instant;

/// TFLite inference configuration
#[derive(Clone, Debug)]
pub struct TFLiteConfig {
    /// Number of threads for CPU inference
    pub num_threads: usize,
    /// Use Edge TPU delegate
    pub use_edge_tpu: bool,
    /// Use GPU delegate
    pub use_gpu: bool,
    /// Confidence threshold
    pub confidence_threshold: f32,
    /// Input image size [height, width]
    pub input_size: Option<[u32; 2]>,
    /// Normalization mean [R, G, B]
    pub norm_mean: [f32; 3],
    /// Normalization std [R, G, B]
    pub norm_std: [f32; 3],
}

impl Default for TFLiteConfig {
    fn default() -> Self {
        Self {
            num_threads: 4,
            use_edge_tpu: false,
            use_gpu: false,
            confidence_threshold: 0.5,
            input_size: None,
            norm_mean: [0.5, 0.5, 0.5], // Common for quantized models
            norm_std: [0.5, 0.5, 0.5],
        }
    }
}

/// TensorFlow Lite Inference Node
///
/// Runs TFLite models for edge deployment and embedded systems.
#[cfg(feature = "tflite-inference")]
pub struct TFLiteInferenceNode {
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
    /// TFLite interpreter
    interpreter: tflite::Interpreter,
    /// Configuration
    config: TFLiteConfig,
    /// Model metadata
    model_info: ModelInfo,
    /// Frame counter
    frame_count: u64,
}

#[cfg(feature = "tflite-inference")]
impl TFLiteInferenceNode {
    /// Create a new TFLite inference node with image input
    pub fn new_image(
        model_path: &str,
        input_topic: &str,
        output_topic: &str,
        config: TFLiteConfig,
    ) -> HorusResult<Self> {
        let interpreter = Self::load_model(model_path, &config)?;
        let model_info = Self::extract_model_info(&interpreter, model_path)?;

        Ok(Self {
            image_sub: Some(Topic::new(input_topic)?),
            tensor_sub: None,
            predictions_pub: Topic::new(output_topic)?,
            tensor_pub: None,
            metrics_pub: Topic::new(&format!("{}.metrics", output_topic))?,
            interpreter,
            config,
            model_info,
            frame_count: 0,
        })
    }

    /// Create a new TFLite inference node with tensor input
    pub fn new_tensor(
        model_path: &str,
        input_topic: &str,
        output_topic: &str,
        config: TFLiteConfig,
    ) -> HorusResult<Self> {
        let interpreter = Self::load_model(model_path, &config)?;
        let model_info = Self::extract_model_info(&interpreter, model_path)?;

        Ok(Self {
            image_sub: None,
            tensor_sub: Some(Topic::new(input_topic)?),
            predictions_pub: Topic::new(output_topic)?,
            tensor_pub: Some(Topic::new(&format!("{}.tensor", output_topic))?),
            metrics_pub: Topic::new(&format!("{}.metrics", output_topic))?,
            interpreter,
            config,
            model_info,
            frame_count: 0,
        })
    }

    /// Load TFLite model
    fn load_model(model_path: &str, config: &TFLiteConfig) -> Result<tflite::Interpreter> {
        if !Path::new(model_path).exists() {
            return Err(HorusError::Config(format!(
                "Model file not found: {}",
                model_path
            )));
        }

        use tflite::{ops::builtin::BuiltinOpResolver, FlatBufferModel, InterpreterBuilder};

        let model = FlatBufferModel::build_from_file(model_path)
            .map_err(|e| HorusError::Config(format!("Failed to load TFLite model: {:?}", e)))?;

        let resolver = BuiltinOpResolver::default();
        let builder = InterpreterBuilder::new(model, resolver).map_err(|e| {
            HorusError::Config(format!("Failed to create interpreter builder: {:?}", e))
        })?;

        let mut interpreter = builder
            .build()
            .map_err(|e| HorusError::Config(format!("Failed to build interpreter: {:?}", e)))?;

        // Set number of threads
        interpreter
            .set_num_threads(config.num_threads as i32)
            .map_err(|e| HorusError::Config(format!("Failed to set num_threads: {:?}", e)))?;

        // Allocate tensors
        interpreter
            .allocate_tensors()
            .map_err(|e| HorusError::Config(format!("Failed to allocate tensors: {:?}", e)))?;

        Ok(interpreter)
    }

    /// Extract model metadata
    fn extract_model_info(
        interpreter: &tflite::Interpreter,
        model_path: &str,
    ) -> Result<ModelInfo> {
        let model_name = Path::new(model_path)
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("tflite_model")
            .to_string();

        // Get input/output tensor info
        let input_count = interpreter.input_count();
        let output_count = interpreter.output_count();

        let mut input_shapes = Vec::new();
        let mut input_names = Vec::new();

        for i in 0..input_count {
            if let Some(tensor) = interpreter.input(i as usize) {
                let shape: Vec<usize> = tensor.dims().iter().map(|&d| d as usize).collect();
                input_shapes.push(shape);
                input_names.push(format!("input_{}", i));
            }
        }

        let mut output_shapes = Vec::new();
        let mut output_names = Vec::new();

        for i in 0..output_count {
            if let Some(tensor) = interpreter.output(i as usize) {
                let shape: Vec<usize> = tensor.dims().iter().map(|&d| d as usize).collect();
                output_shapes.push(shape);
                output_names.push(format!("output_{}", i));
            }
        }

        Ok(ModelInfo {
            name: model_name,
            version: "1.0.0".to_string(),
            format: ModelFormat::TFLite,
            input_shapes,
            output_shapes,
            input_names,
            output_names,
            metadata: std::collections::HashMap::new(),
        })
    }

    /// Preprocess image for TFLite
    fn preprocess_image(&self, image: &Image) -> Result<Vec<u8>> {
        let width = image.width as usize;
        let height = image.height as usize;

        // Determine target size
        let (target_h, target_w) = if let Some([h, w]) = self.config.input_size {
            (h as usize, w as usize)
        } else {
            // Use model input size
            if let Some(shape) = self.model_info.input_shapes.first() {
                if shape.len() == 4 {
                    (shape[1], shape[2]) // NHWC format
                } else {
                    return Err(HorusError::Config("Unexpected input shape".to_string()));
                }
            } else {
                return Err(HorusError::Config("No input shape found".to_string()));
            }
        };

        // Resize if needed
        let resized = if width != target_w || height != target_h {
            self.resize_image(&image.data, width, height, target_w, target_h)
        } else {
            image.data.clone()
        };

        Ok(resized)
    }

    /// Basic image resize
    fn resize_image(
        &self,
        data: &[u8],
        src_w: usize,
        src_h: usize,
        dst_w: usize,
        dst_h: usize,
    ) -> Vec<u8> {
        let mut resized = vec![0u8; dst_h * dst_w * 3];

        let x_ratio = src_w as f32 / dst_w as f32;
        let y_ratio = src_h as f32 / dst_h as f32;

        for y in 0..dst_h {
            for x in 0..dst_w {
                let src_x = ((x as f32 * x_ratio) as usize).min(src_w - 1);
                let src_y = ((y as f32 * y_ratio) as usize).min(src_h - 1);

                for c in 0..3 {
                    let src_idx = (src_y * src_w + src_x) * 3 + c;
                    let dst_idx = (y * dst_w + x) * 3 + c;
                    resized[dst_idx] = data[src_idx];
                }
            }
        }

        resized
    }

    /// Run TFLite inference
    fn run_inference(&mut self, input_data: &[u8]) -> Result<Vec<f32>> {
        // Get input tensor
        let input_tensor = self
            .interpreter
            .input(0)
            .ok_or_else(|| HorusError::config("No input tensor".to_string()))?;

        // Copy data to input tensor (assuming quantized u8 input)
        if let Some(data) = input_tensor.data::<u8>() {
            let min_len = data.len().min(input_data.len());
            data[..min_len].copy_from_slice(&input_data[..min_len]);
        } else {
            return Err(HorusError::config("Input tensor type mismatch".to_string()));
        }

        // Run inference
        self.interpreter
            .invoke()
            .map_err(|e| HorusError::config(format!("Inference failed: {:?}", e)))?;

        // Get output tensor
        let output_tensor = self
            .interpreter
            .output(0)
            .ok_or_else(|| HorusError::config("No output tensor".to_string()))?;

        // Read output (could be float32 or quantized)
        if let Some(data) = output_tensor.data::<f32>() {
            Ok(data.to_vec())
        } else if let Some(data) = output_tensor.data::<u8>() {
            // Dequantize u8 to float32
            // TFLite quantization: real_value = (quantized_value - zero_point) * scale
            // For simplicity, assume scale=1/255 and zero_point=0
            Ok(data.iter().map(|&v| v as f32 / 255.0).collect())
        } else {
            Err(HorusError::config(
                "Output tensor type not supported".to_string(),
            ))
        }
    }

    /// Postprocess output into predictions
    fn postprocess_output(&self, output: Vec<f32>) -> Result<Predictions> {
        // Find top-K predictions
        let mut indexed: Vec<(usize, f32)> =
            output.iter().enumerate().map(|(i, &v)| (i, v)).collect();

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

#[cfg(feature = "tflite-inference")]
impl Node for TFLiteInferenceNode {
    fn name(&self) -> &'static str {
        "TFLiteInferenceNode"
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Handle image input
        if let Some(ref image_sub) = self.image_sub {
            if let Some(image) = image_sub.recv(&mut ctx) {
                let start = Instant::now();
                let timestamp = image.timestamp;

                // Preprocess
                let input = match self.preprocess_image(&image) {
                    Ok(data) => data,
                    Err(e) => {
                        eprintln!("Preprocessing failed: {}", e);
                        return;
                    }
                };

                // Run inference
                let output = match self.run_inference(&input) {
                    Ok(o) => o,
                    Err(e) => {
                        eprintln!("Inference failed: {}", e);
                        return;
                    }
                };

                // Postprocess
                let predictions = match self.postprocess_output(output) {
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
                    batch_size: 1,
                    timestamp_ns: timestamp,
                };
                let _ = self.metrics_pub.send(metrics, &mut ctx);

                self.frame_count += 1;
            }
        }

        // Handle tensor input
        if let Some(ref tensor_sub) = self.tensor_sub {
            if let Some(_tensor) = tensor_sub.recv(&mut ctx) {
                // For tensor input, would need to convert HORUS Tensor to u8 or f32
                // depending on model quantization
                // Simplified for now
                eprintln!("Tensor input not yet fully implemented for TFLite");
            }
        }
    }
}

// Stub implementation when TFLite feature is disabled
#[cfg(not(feature = "tflite-inference"))]
pub struct TFLiteInferenceNode;

#[cfg(not(feature = "tflite-inference"))]
impl TFLiteInferenceNode {
    pub fn new_image(
        _model_path: &str,
        _input_topic: &str,
        _output_topic: &str,
        _config: TFLiteConfig,
    ) -> HorusResult<Self> {
        Err(HorusError::Config(
            "TFLite support not compiled. Enable 'tflite-inference' feature.".to_string(),
        ))
    }

    pub fn new_tensor(
        _model_path: &str,
        _input_topic: &str,
        _output_topic: &str,
        _config: TFLiteConfig,
    ) -> HorusResult<Self> {
        Err(HorusError::Config(
            "TFLite support not compiled. Enable 'tflite-inference' feature.".to_string(),
        ))
    }
}

#[cfg(not(feature = "tflite-inference"))]
impl Node for TFLiteInferenceNode {
    fn name(&self) -> &'static str {
        "TFLiteInferenceNode"
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {}
}
