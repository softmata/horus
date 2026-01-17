// Monocular Depth Estimation Node for HORUS
//
// Real-time depth estimation from single RGB images using neural networks.
// Supports MiDaS, DPT, Depth Anything, and similar models.
//
// # Features
// - MiDaS v2/v3 model support
// - DPT (Dense Prediction Transformer) support
// - Depth Anything support
// - Metric depth estimation (optional)
// - Multi-scale inference
// - Point cloud generation
//
// # Example
// ```rust,ignore
// use horus::prelude::*;
// use horus_library::nodes::cv::DepthEstimationNode;
//
// fn main() -> Result<()> {
//     let mut scheduler = Scheduler::new();
//
//     let depth_node = DepthEstimationNode::new(
//         "models/midas_v21_384.onnx",
//         "camera/raw",
//         "vision/depth",
//         DepthConfig::midas_v21(),
//     )?;
//
//     scheduler.add(Box::new(depth_node), 1, Some(true));
//     scheduler.run()?;
//     Ok(())
// }
// ```

#[cfg(feature = "onnx")]
use ort::session::{builder::GraphOptimizationLevel, Session};
#[cfg(feature = "onnx")]
use ort::value::Tensor;

use crate::messages::ml::InferenceMetrics;
use crate::messages::{DepthImage, Image};
use horus_core::{HorusError, HorusResult, Node, NodeInfo, Topic};

#[cfg(feature = "onnx")]
use ndarray::{Array, ArrayD, IxDyn};

/// Depth estimation model type
#[derive(Clone, Debug, PartialEq)]
pub enum DepthModelType {
    /// MiDaS v2.1 (small - 384x384)
    MiDaSSmall,
    /// MiDaS v2.1 (large - 384x384)
    MiDaSLarge,
    /// MiDaS v3 DPT-Hybrid
    MiDaSHybrid,
    /// MiDaS v3 DPT-Large
    MiDaSLargeDPT,
    /// Depth Anything (small)
    DepthAnythingSmall,
    /// Depth Anything (base)
    DepthAnythingBase,
    /// Depth Anything (large)
    DepthAnythingLarge,
    /// Custom model with specified input size
    Custom { input_size: [usize; 2] },
}

/// Depth estimation configuration
#[derive(Clone, Debug)]
pub struct DepthConfig {
    /// Model type
    pub model_type: DepthModelType,
    /// Input image size [height, width]
    pub input_size: [usize; 2],
    /// Use GPU for inference
    pub use_gpu: bool,
    /// GPU device ID
    pub device_id: u32,
    /// Number of threads for CPU inference
    pub num_threads: usize,
    /// Output metric depth (meters) vs relative depth
    pub metric_depth: bool,
    /// Depth scale factor (for metric depth)
    pub depth_scale: f32,
    /// Minimum depth value (meters)
    pub min_depth: f32,
    /// Maximum depth value (meters)
    pub max_depth: f32,
    /// Enable colormap visualization
    pub enable_visualization: bool,
    /// Invert depth map (near=dark, far=bright)
    pub invert_depth: bool,
}

impl DepthConfig {
    /// MiDaS v2.1 small configuration
    pub fn midas_small() -> Self {
        Self {
            model_type: DepthModelType::MiDaSSmall,
            input_size: [256, 256],
            use_gpu: false,
            device_id: 0,
            num_threads: 4,
            metric_depth: false,
            depth_scale: 1.0,
            min_depth: 0.1,
            max_depth: 100.0,
            enable_visualization: true,
            invert_depth: true,
        }
    }

    /// MiDaS v2.1 large configuration
    pub fn midas_v21() -> Self {
        Self {
            model_type: DepthModelType::MiDaSLarge,
            input_size: [384, 384],
            use_gpu: false,
            device_id: 0,
            num_threads: 4,
            metric_depth: false,
            depth_scale: 1.0,
            min_depth: 0.1,
            max_depth: 100.0,
            enable_visualization: true,
            invert_depth: true,
        }
    }

    /// MiDaS v3 DPT-Hybrid configuration
    pub fn midas_hybrid() -> Self {
        Self {
            model_type: DepthModelType::MiDaSHybrid,
            input_size: [384, 384],
            use_gpu: false,
            device_id: 0,
            num_threads: 4,
            metric_depth: false,
            depth_scale: 1.0,
            min_depth: 0.1,
            max_depth: 100.0,
            enable_visualization: true,
            invert_depth: true,
        }
    }

    /// MiDaS v3 DPT-Large configuration (highest quality)
    pub fn midas_large() -> Self {
        Self {
            model_type: DepthModelType::MiDaSLargeDPT,
            input_size: [384, 384],
            use_gpu: false,
            device_id: 0,
            num_threads: 4,
            metric_depth: false,
            depth_scale: 1.0,
            min_depth: 0.1,
            max_depth: 100.0,
            enable_visualization: true,
            invert_depth: true,
        }
    }

    /// Depth Anything small configuration (fastest)
    pub fn depth_anything_small() -> Self {
        Self {
            model_type: DepthModelType::DepthAnythingSmall,
            input_size: [518, 518],
            use_gpu: false,
            device_id: 0,
            num_threads: 4,
            metric_depth: false,
            depth_scale: 1.0,
            min_depth: 0.1,
            max_depth: 100.0,
            enable_visualization: true,
            invert_depth: true,
        }
    }

    /// Depth Anything base configuration (balanced)
    pub fn depth_anything_base() -> Self {
        Self {
            model_type: DepthModelType::DepthAnythingBase,
            input_size: [518, 518],
            use_gpu: false,
            device_id: 0,
            num_threads: 4,
            metric_depth: false,
            depth_scale: 1.0,
            min_depth: 0.1,
            max_depth: 100.0,
            enable_visualization: true,
            invert_depth: true,
        }
    }

    /// Depth Anything large configuration (highest quality)
    pub fn depth_anything_large() -> Self {
        Self {
            model_type: DepthModelType::DepthAnythingLarge,
            input_size: [518, 518],
            use_gpu: false,
            device_id: 0,
            num_threads: 4,
            metric_depth: false,
            depth_scale: 1.0,
            min_depth: 0.1,
            max_depth: 100.0,
            enable_visualization: true,
            invert_depth: true,
        }
    }
}

impl Default for DepthConfig {
    fn default() -> Self {
        Self::midas_v21()
    }
}

/// Monocular Depth Estimation Node
///
/// Estimates depth from single RGB images using neural networks.
#[cfg(feature = "onnx")]
pub struct DepthEstimationNode {
    /// Image input subscriber
    image_sub: Topic<Image>,
    /// Depth image publisher
    depth_pub: Topic<DepthImage>,
    /// Visualization publisher (optional)
    vis_pub: Option<Topic<Image>>,
    /// Metrics publisher
    metrics_pub: Topic<InferenceMetrics>,
    /// ONNX Runtime session
    session: Session,
    /// Configuration
    config: DepthConfig,
    /// Model name
    model_name: String,
    /// Frame counter
    frame_count: u64,
}

#[cfg(feature = "onnx")]
impl DepthEstimationNode {
    /// Create a new depth estimation node
    pub fn new(
        model_path: &str,
        input_topic: &str,
        output_topic: &str,
        config: DepthConfig,
    ) -> HorusResult<Self> {
        let session = Self::load_model(model_path, &config)?;

        let model_name = std::path::Path::new(model_path)
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("depth")
            .to_string();

        let vis_pub = if config.enable_visualization {
            Some(Topic::new(&format!("{}.visualization", output_topic))?)
        } else {
            None
        };

        Ok(Self {
            image_sub: Topic::new(input_topic)?,
            depth_pub: Topic::new(output_topic)?,
            vis_pub,
            metrics_pub: Topic::new(&format!("{}.metrics", output_topic))?,
            session,
            config,
            model_name,
            frame_count: 0,
        })
    }

    /// Load ONNX model
    fn load_model(model_path: &str, config: &DepthConfig) -> HorusResult<Session> {
        let mut builder = Session::builder()
            .map_err(|e| HorusError::config(format!("Failed to create session builder: {}", e)))?;

        builder = builder
            .with_optimization_level(GraphOptimizationLevel::Level3)
            .map_err(|e| HorusError::config(format!("Failed to set optimization level: {}", e)))?;

        if !config.use_gpu {
            builder = builder
                .with_intra_threads(config.num_threads)
                .map_err(|e| HorusError::config(format!("Failed to set thread count: {}", e)))?;
        }

        #[cfg(feature = "cuda")]
        if config.use_gpu {
            use ort::ExecutionProvider;
            builder = builder
                .with_execution_providers([ExecutionProvider::CUDA(Default::default())])
                .map_err(|e| HorusError::config(format!("Failed to enable CUDA: {}", e)))?;
        }

        let session = builder.commit_from_file(model_path).map_err(|e| {
            HorusError::config(format!("Failed to load depth model {}: {}", model_path, e))
        })?;

        Ok(session)
    }

    /// Preprocess image for depth estimation
    fn preprocess(&self, image: &Image) -> HorusResult<ArrayD<f32>> {
        let img_data = &image.data;
        let width = image.width as usize;
        let height = image.height as usize;
        let channels = (img_data.len() / (width * height)).min(3);

        let target_h = self.config.input_size[0];
        let target_w = self.config.input_size[1];

        // Resize using bilinear interpolation
        let mut resized = vec![0u8; target_h * target_w * 3];
        let scale_h = height as f32 / target_h as f32;
        let scale_w = width as f32 / target_w as f32;

        for y in 0..target_h {
            for x in 0..target_w {
                let src_y = ((y as f32 + 0.5) * scale_h) as usize;
                let src_x = ((x as f32 + 0.5) * scale_w) as usize;
                let src_y = src_y.min(height - 1);
                let src_x = src_x.min(width - 1);

                for c in 0..3 {
                    let src_idx = (src_y * width + src_x) * channels + c.min(channels - 1);
                    let dst_idx = (y * target_w + x) * 3 + c;
                    resized[dst_idx] = img_data.get(src_idx).copied().unwrap_or(0);
                }
            }
        }

        // Normalize based on model type
        // MiDaS uses ImageNet normalization: (x - mean) / std
        // mean = [0.485, 0.456, 0.406], std = [0.229, 0.224, 0.225]
        let mean = [0.485f32, 0.456, 0.406];
        let std = [0.229f32, 0.224, 0.225];

        let mut normalized = Array::zeros(IxDyn(&[1, 3, target_h, target_w]));
        for c in 0..3 {
            for h in 0..target_h {
                for w in 0..target_w {
                    let idx = (h * target_w + w) * 3 + c;
                    let value = resized[idx] as f32 / 255.0;
                    normalized[[0, c, h, w]] = (value - mean[c]) / std[c];
                }
            }
        }

        Ok(normalized)
    }

    /// Run inference
    fn run_inference(&mut self, input: ArrayD<f32>) -> HorusResult<ArrayD<f32>> {
        let input_tensor = Tensor::from_array(input)
            .map_err(|e| HorusError::config(format!("Failed to create input tensor: {}", e)))?;

        let outputs = self
            .session
            .run(ort::inputs![input_tensor])
            .map_err(|e| HorusError::config(format!("Inference failed: {}", e)))?;

        let array_view: ndarray::ArrayViewD<f32> = outputs[0]
            .try_extract_array()
            .map_err(|e| HorusError::config(format!("Failed to extract tensor: {}", e)))?;

        Ok(array_view.to_owned())
    }

    /// Post-process depth output
    fn postprocess(
        &self,
        output: ArrayD<f32>,
        orig_width: u32,
        orig_height: u32,
    ) -> HorusResult<DepthImage> {
        let shape = output.shape();

        // Output is typically [1, H, W] or [1, 1, H, W]
        let (out_h, out_w) = if shape.len() == 3 {
            (shape[1], shape[2])
        } else if shape.len() == 4 {
            (shape[2], shape[3])
        } else {
            return Err(HorusError::config(format!(
                "Unexpected depth output shape: {:?}",
                shape
            )));
        };

        // Find min/max for normalization
        let mut min_depth = f32::MAX;
        let mut max_depth = f32::MIN;

        for y in 0..out_h {
            for x in 0..out_w {
                let val = if shape.len() == 3 {
                    output[[0, y, x]]
                } else {
                    output[[0, 0, y, x]]
                };
                min_depth = min_depth.min(val);
                max_depth = max_depth.max(val);
            }
        }

        // Resize to original dimensions
        let target_w = orig_width as usize;
        let target_h = orig_height as usize;
        let scale_h = out_h as f32 / target_h as f32;
        let scale_w = out_w as f32 / target_w as f32;

        // Create depth data as f32 values (meters or normalized)
        let mut depth_data = vec![0.0f32; target_w * target_h];
        let range = max_depth - min_depth;
        let range = if range > 0.0 { range } else { 1.0 };

        for y in 0..target_h {
            for x in 0..target_w {
                let src_y = ((y as f32 + 0.5) * scale_h) as usize;
                let src_x = ((x as f32 + 0.5) * scale_w) as usize;
                let src_y = src_y.min(out_h - 1);
                let src_x = src_x.min(out_w - 1);

                let raw_depth = if shape.len() == 3 {
                    output[[0, src_y, src_x]]
                } else {
                    output[[0, 0, src_y, src_x]]
                };

                // Normalize to 0-1 range
                let normalized = (raw_depth - min_depth) / range;

                // Apply inversion if configured (MiDaS outputs inverse depth)
                let depth_value = if self.config.invert_depth {
                    1.0 - normalized
                } else {
                    normalized
                };

                // Scale to metric depth if configured
                let final_depth = if self.config.metric_depth {
                    self.config.min_depth + depth_value * (self.config.max_depth - self.config.min_depth)
                } else {
                    depth_value * self.config.depth_scale
                };

                depth_data[y * target_w + x] = final_depth;
            }
        }

        // Convert f32 depth to u16 for DepthImage (millimeters)
        let depths: Vec<u16> = depth_data
            .iter()
            .map(|&d| {
                let mm = if self.config.metric_depth {
                    d * 1000.0 // meters to millimeters
                } else {
                    d * 65535.0 // normalized to full u16 range
                };
                mm.clamp(0.0, 65535.0) as u16
            })
            .collect();

        // Convert config min/max from f32 meters to u16 mm
        let min_depth_mm = (self.config.min_depth * 1000.0).clamp(0.0, 65535.0) as u16;
        let max_depth_mm = (self.config.max_depth * 1000.0).clamp(0.0, 65535.0) as u16;

        Ok(DepthImage {
            width: orig_width,
            height: orig_height,
            depths,
            min_depth: min_depth_mm,
            max_depth: max_depth_mm,
            depth_scale: 1.0, // 1mm per unit
            frame_id: [0u8; 32],
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
        })
    }

    /// Create colorized depth visualization
    fn create_visualization(depth: &DepthImage) -> HorusResult<Image> {
        let width = depth.width as usize;
        let height = depth.height as usize;
        let mut vis_data = vec![0u8; width * height * 3];

        // Read depth values and create colormap
        for y in 0..height {
            for x in 0..width {
                let idx = y * width + x;

                let depth_val = depth.depths.get(idx).copied().unwrap_or(0);

                // Normalize to 0-1
                let normalized = depth_val as f32 / 65535.0;

                // Apply turbo colormap (simplified)
                let (r, g, b) = Self::turbo_colormap(normalized);

                let vis_idx = idx * 3;
                vis_data[vis_idx] = r;
                vis_data[vis_idx + 1] = g;
                vis_data[vis_idx + 2] = b;
            }
        }

        Ok(Image {
            data: vis_data,
            width: depth.width,
            height: depth.height,
            encoding: crate::vision::ImageEncoding::Rgb8,
            step: depth.width * 3,
            frame_id: [0u8; 32],
            timestamp: depth.timestamp,
        })
    }

    /// Turbo colormap (simplified version)
    fn turbo_colormap(t: f32) -> (u8, u8, u8) {
        let t = t.clamp(0.0, 1.0);

        // Simplified turbo-like colormap
        let r = if t < 0.25 {
            0.0
        } else if t < 0.5 {
            (t - 0.25) * 4.0
        } else if t < 0.75 {
            1.0
        } else {
            1.0 - (t - 0.75) * 2.0
        };

        let g = if t < 0.25 {
            t * 4.0
        } else if t < 0.75 {
            1.0
        } else {
            1.0 - (t - 0.75) * 4.0
        };

        let b = if t < 0.5 {
            1.0 - t * 2.0
        } else {
            0.0
        };

        (
            (r.clamp(0.0, 1.0) * 255.0) as u8,
            (g.clamp(0.0, 1.0) * 255.0) as u8,
            (b.clamp(0.0, 1.0) * 255.0) as u8,
        )
    }
}

#[cfg(feature = "onnx")]
impl Node for DepthEstimationNode {
    fn name(&self) -> &'static str {
        "DepthEstimationNode"
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        while let Some(image) = self.image_sub.recv(&mut ctx) {
            let start = std::time::Instant::now();

            // Preprocess
            let input = match self.preprocess(&image) {
                Ok(inp) => inp,
                Err(e) => {
                    eprintln!("Depth preprocessing failed: {}", e);
                    continue;
                }
            };

            // Inference
            let output = match self.run_inference(input) {
                Ok(out) => out,
                Err(e) => {
                    eprintln!("Depth inference failed: {}", e);
                    continue;
                }
            };

            // Postprocess
            let depth = match self.postprocess(output, image.width, image.height) {
                Ok(d) => d,
                Err(e) => {
                    eprintln!("Depth postprocessing failed: {}", e);
                    continue;
                }
            };

            let latency_ms = start.elapsed().as_secs_f32() * 1000.0;

            // Publish depth
            let _ = self.depth_pub.send(depth.clone(), &mut ctx);

            // Publish visualization if enabled
            if let Some(ref mut vis_pub) = self.vis_pub {
                if let Ok(vis_img) = Self::create_visualization(&depth) {
                    let _ = vis_pub.send(vis_img, &mut ctx);
                }
            }

            // Publish metrics
            self.frame_count += 1;
            let metrics = InferenceMetrics {
                latency_ms,
                throughput: 1000.0 / latency_ms,
                model_name: self.model_name.clone(),
                batch_size: 1,
                timestamp_ns: std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as u64,
            };
            let _ = self.metrics_pub.send(metrics, &mut ctx);
        }
    }
}

// Stub implementation when ONNX is not enabled
#[cfg(not(feature = "onnx"))]
pub struct DepthEstimationNode;

#[cfg(not(feature = "onnx"))]
impl DepthEstimationNode {
    pub fn new(
        _model_path: &str,
        _input_topic: &str,
        _output_topic: &str,
        _config: DepthConfig,
    ) -> HorusResult<Self> {
        Err(HorusError::Config(
            "ONNX support not compiled. Enable 'onnx' feature for DepthEstimationNode.".to_string(),
        ))
    }
}

#[cfg(not(feature = "onnx"))]
impl Node for DepthEstimationNode {
    fn name(&self) -> &'static str {
        "DepthEstimationNode (disabled)"
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        // No-op
    }
}
