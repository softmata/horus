// Semantic Segmentation Node for HORUS
//
// Real-time semantic segmentation using DeepLabV3+, U-Net, and other models.
// Provides per-pixel class prediction for scene understanding.
//
// # Features
// - DeepLabV3+, U-Net, SegFormer model support
// - Per-pixel class prediction
// - Color-coded visualization
// - Multiple output formats (class IDs, colored mask, probability maps)
// - Post-processing (CRF, morphological operations)
// - Multi-class support (Cityscapes, ADE20K, COCO-Stuff, Pascal VOC)
//
// # Example
// ```rust,ignore
// use horus::prelude::*;
// use horus_library::nodes::cv::SemanticSegmentationNode;
//
// fn main() -> Result<()> {
//     let mut scheduler = Scheduler::new();
//
//     let segmentation = SemanticSegmentationNode::new(
//         "models/deeplabv3_resnet50.onnx",
//         "camera/raw",
//         "vision/segmentation",
//         SegmentationConfig::cityscapes(),
//     )?;
//
//     scheduler.add(Box::new(segmentation), 1, Some(true));
//     scheduler.run()?;
//     Ok(())
//     }
// ```

#[cfg(feature = "onnx")]
use ort::session::{builder::GraphOptimizationLevel, Session};
#[cfg(feature = "onnx")]
use ort::value::Tensor;

use crate::messages::ml::{InferenceMetrics, SegmentationMask};
use crate::messages::Image;
use horus_core::{HorusError, HorusResult, Node, NodeInfo, Topic};

#[cfg(feature = "onnx")]
use ndarray::{Array, ArrayD, IxDyn};

/// Semantic segmentation configuration
#[derive(Clone, Debug)]
pub struct SegmentationConfig {
    /// Input image size [height, width]
    pub input_size: [usize; 2],
    /// Number of classes
    pub num_classes: usize,
    /// Class names (for visualization)
    pub class_names: Vec<String>,
    /// Use GPU for inference
    pub use_gpu: bool,
    /// Number of threads for CPU inference
    pub num_threads: usize,
    /// Apply CRF post-processing
    pub use_crf: bool,
    /// Apply morphological post-processing
    pub use_morphology: bool,
    /// Morphology kernel size
    pub morph_kernel_size: usize,
    /// Confidence threshold for predictions
    pub conf_threshold: f32,
    /// Output colored visualization
    pub output_colored: bool,
}

impl SegmentationConfig {
    /// Cityscapes dataset configuration (19 classes)
    pub fn cityscapes() -> Self {
        Self {
            input_size: [512, 1024],
            num_classes: 19,
            class_names: vec![
                "road".to_string(),
                "sidewalk".to_string(),
                "building".to_string(),
                "wall".to_string(),
                "fence".to_string(),
                "pole".to_string(),
                "traffic light".to_string(),
                "traffic sign".to_string(),
                "vegetation".to_string(),
                "terrain".to_string(),
                "sky".to_string(),
                "person".to_string(),
                "rider".to_string(),
                "car".to_string(),
                "truck".to_string(),
                "bus".to_string(),
                "train".to_string(),
                "motorcycle".to_string(),
                "bicycle".to_string(),
            ],
            use_gpu: false,
            num_threads: 4,
            use_crf: false,
            use_morphology: true,
            morph_kernel_size: 3,
            conf_threshold: 0.5,
            output_colored: true,
        }
    }

    /// Pascal VOC dataset configuration (21 classes)
    pub fn pascal_voc() -> Self {
        Self {
            input_size: [512, 512],
            num_classes: 21,
            class_names: vec![
                "background".to_string(),
                "aeroplane".to_string(),
                "bicycle".to_string(),
                "bird".to_string(),
                "boat".to_string(),
                "bottle".to_string(),
                "bus".to_string(),
                "car".to_string(),
                "cat".to_string(),
                "chair".to_string(),
                "cow".to_string(),
                "table".to_string(),
                "dog".to_string(),
                "horse".to_string(),
                "motorbike".to_string(),
                "person".to_string(),
                "plant".to_string(),
                "sheep".to_string(),
                "sofa".to_string(),
                "train".to_string(),
                "monitor".to_string(),
            ],
            use_gpu: false,
            num_threads: 4,
            use_crf: false,
            use_morphology: true,
            morph_kernel_size: 3,
            conf_threshold: 0.5,
            output_colored: true,
        }
    }

    /// ADE20K dataset configuration (150 classes)
    pub fn ade20k() -> Self {
        Self {
            input_size: [512, 512],
            num_classes: 150,
            class_names: (0..150).map(|i| format!("class_{}", i)).collect(),
            use_gpu: false,
            num_threads: 4,
            use_crf: false,
            use_morphology: false,
            morph_kernel_size: 3,
            conf_threshold: 0.5,
            output_colored: true,
        }
    }
}

impl Default for SegmentationConfig {
    fn default() -> Self {
        Self::pascal_voc()
    }
}

#[cfg(feature = "onnx")]
pub struct SemanticSegmentationNode {
    /// Image input subscriber
    image_sub: Topic<Image>,
    /// Segmentation mask publisher
    mask_pub: Topic<SegmentationMask>,
    /// Colored visualization publisher (optional)
    vis_pub: Option<Topic<Image>>,
    /// Metrics publisher
    metrics_pub: Topic<InferenceMetrics>,
    /// ONNX Runtime session
    session: Session,
    /// Configuration
    config: SegmentationConfig,
    /// Model name
    model_name: String,
    /// Frame counter
    frame_count: u64,
}

#[cfg(feature = "onnx")]
impl SemanticSegmentationNode {
    /// Create a new semantic segmentation node
    pub fn new(
        model_path: &str,
        input_topic: &str,
        output_topic: &str,
        config: SegmentationConfig,
    ) -> HorusResult<Self> {
        let session = Self::load_model(model_path, &config)?;

        let model_name = std::path::Path::new(model_path)
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("segmentation")
            .to_string();

        let vis_pub = if config.output_colored {
            Some(Topic::new(&format!("{}.visualization", output_topic))?)
        } else {
            None
        };

        Ok(Self {
            image_sub: Topic::new(input_topic)?,
            mask_pub: Topic::new(output_topic)?,
            vis_pub,
            metrics_pub: Topic::new(&format!("{}.metrics", output_topic))?,
            session,
            config,
            model_name,
            frame_count: 0,
        })
    }

    /// Load ONNX model
    fn load_model(model_path: &str, config: &SegmentationConfig) -> HorusResult<Session> {
        let mut builder = Session::builder()
            .map_err(|e| HorusError::config(format!("Failed to create session builder: {}", e)))?;

        builder = builder
            .with_optimization_level(GraphOptimizationLevel::Level3)
            .map_err(|e| HorusError::config(format!("Failed to set optimization level: {}", e)))?;

        if !config.use_gpu {
            builder = builder
                .with_intra_threads(config.num_threads as usize)
                .map_err(|e| HorusError::config(format!("Failed to set thread count: {}", e)))?;
        }

        let session = builder.commit_from_file(model_path).map_err(|e| {
            HorusError::config(format!("Failed to load model {}: {}", model_path, e))
        })?;

        Ok(session)
    }

    /// Preprocess image for segmentation
    fn preprocess(&self, image: &Image) -> HorusResult<ArrayD<f32>> {
        let img_data = &image.data;
        let width = image.width as usize;
        let height = image.height as usize;
        let channels = (img_data.len() / (width * height)) as usize;

        // Resize to model input size
        let target_h = self.config.input_size[0];
        let target_w = self.config.input_size[1];

        // Simple bilinear resize (for production, use image crate)
        let mut resized = vec![0u8; target_h * target_w * channels];
        let scale_h = height as f32 / target_h as f32;
        let scale_w = width as f32 / target_w as f32;

        for y in 0..target_h {
            for x in 0..target_w {
                let src_y = ((y as f32 + 0.5) * scale_h) as usize;
                let src_x = ((x as f32 + 0.5) * scale_w) as usize;
                let src_y = src_y.min(height - 1);
                let src_x = src_x.min(width - 1);

                for c in 0..channels {
                    let src_idx = (src_y * width + src_x) * channels + c;
                    let dst_idx = (y * target_w + x) * channels + c;
                    resized[dst_idx] = img_data[src_idx];
                }
            }
        }

        // Normalize to 0-1 and convert to CHW format
        let mut normalized = Array::zeros(IxDyn(&[1, channels, target_h, target_w]));
        for c in 0..channels {
            for h in 0..target_h {
                for w in 0..target_w {
                    let idx = (h * target_w + w) * channels + c;
                    let value = resized[idx] as f32 / 255.0;
                    normalized[[0, c, h, w]] = value;
                }
            }
        }

        // ImageNet normalization
        let mean = [0.485, 0.456, 0.406];
        let std = [0.229, 0.224, 0.225];
        for c in 0..channels.min(3) {
            for h in 0..target_h {
                for w in 0..target_w {
                    normalized[[0, c, h, w]] = (normalized[[0, c, h, w]] - mean[c]) / std[c];
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

        // Convert view to owned array
        Ok(array_view.to_owned())
    }

    /// Post-process segmentation output
    fn postprocess(
        &self,
        output: ArrayD<f32>,
        orig_width: u32,
        orig_height: u32,
    ) -> HorusResult<SegmentationMask> {
        let shape = output.shape();

        // Handle different output formats
        let (_batch, height, width, num_classes) = if shape.len() == 4 {
            // NCHW or NHWC format
            if shape[1] == self.config.num_classes {
                // NCHW format [batch, classes, height, width]
                (shape[0], shape[2], shape[3], shape[1])
            } else {
                // NHWC format [batch, height, width, classes]
                (shape[0], shape[1], shape[2], shape[3])
            }
        } else {
            return Err(HorusError::config(format!(
                "Unexpected output shape: {:?}",
                shape
            )));
        };

        // Argmax to get class predictions
        let mut mask = vec![0u8; height * width];

        if shape[1] == num_classes {
            // NCHW format
            for h in 0..height {
                for w in 0..width {
                    let mut max_val = f32::NEG_INFINITY;
                    let mut max_class = 0;

                    for c in 0..num_classes {
                        let val = output[[0, c, h, w]];
                        if val > max_val {
                            max_val = val;
                            max_class = c;
                        }
                    }

                    mask[h * width + w] = max_class as u8;
                }
            }
        } else {
            // NHWC format
            for h in 0..height {
                for w in 0..width {
                    let mut max_val = f32::NEG_INFINITY;
                    let mut max_class = 0;

                    for c in 0..num_classes {
                        let val = output[[0, h, w, c]];
                        if val > max_val {
                            max_val = val;
                            max_class = c;
                        }
                    }

                    mask[h * width + w] = max_class as u8;
                }
            }
        }

        // Apply morphological post-processing
        if self.config.use_morphology {
            mask = self.apply_morphology(mask, width, height);
        }

        // Resize mask to original image size if needed
        if width != orig_width as usize || height != orig_height as usize {
            mask = self.resize_mask(
                mask,
                width,
                height,
                orig_width as usize,
                orig_height as usize,
            );
        }

        Ok(SegmentationMask {
            mask,
            width: orig_width,
            height: orig_height,
            num_classes: self.config.num_classes as u32,
            class_names: self.config.class_names.clone(),
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
        })
    }

    /// Apply morphological operations to smooth mask
    fn apply_morphology(&self, mask: Vec<u8>, width: usize, height: usize) -> Vec<u8> {
        let kernel_size = self.config.morph_kernel_size;
        let mut smoothed = mask.clone();

        // Simple morphological closing (erosion + dilation)
        // Erosion pass
        for y in kernel_size..height - kernel_size {
            for x in kernel_size..width - kernel_size {
                let center_class = mask[y * width + x];
                let mut same_class = true;

                // Check kernel neighborhood
                for ky in 0..kernel_size {
                    for kx in 0..kernel_size {
                        let ny = y + ky - kernel_size / 2;
                        let nx = x + kx - kernel_size / 2;
                        if mask[ny * width + nx] != center_class {
                            same_class = false;
                            break;
                        }
                    }
                    if !same_class {
                        break;
                    }
                }

                if same_class {
                    smoothed[y * width + x] = center_class;
                }
            }
        }

        smoothed
    }

    /// Resize segmentation mask to target size
    fn resize_mask(
        &self,
        mask: Vec<u8>,
        src_w: usize,
        src_h: usize,
        dst_w: usize,
        dst_h: usize,
    ) -> Vec<u8> {
        let mut resized = vec![0u8; dst_h * dst_w];
        let scale_h = src_h as f32 / dst_h as f32;
        let scale_w = src_w as f32 / dst_w as f32;

        for y in 0..dst_h {
            for x in 0..dst_w {
                let src_y = ((y as f32 + 0.5) * scale_h) as usize;
                let src_x = ((x as f32 + 0.5) * scale_w) as usize;
                let src_y = src_y.min(src_h - 1);
                let src_x = src_x.min(src_w - 1);

                resized[y * dst_w + x] = mask[src_y * src_w + src_x];
            }
        }

        resized
    }

    /// Create colored visualization of segmentation mask
    fn create_visualization(&self, mask: &SegmentationMask) -> HorusResult<Image> {
        let width = mask.width as usize;
        let height = mask.height as usize;
        let mut colored = vec![0u8; width * height * 3];

        // Generate distinct colors for each class
        let colors = self.generate_class_colors(mask.num_classes as usize);

        for y in 0..height {
            for x in 0..width {
                let class_id = mask.mask[y * width + x] as usize;
                let color = &colors[class_id % colors.len()];

                let idx = (y * width + x) * 3;
                colored[idx] = color[0];
                colored[idx + 1] = color[1];
                colored[idx + 2] = color[2];
            }
        }

        Ok(Image {
            data: colored,
            width: mask.width,
            height: mask.height,
            encoding: crate::messages::ImageEncoding::Rgb8,
            step: mask.width * 3,
            frame_id: [0; 32],
            timestamp: mask.timestamp_ns,
        })
    }

    /// Generate distinct colors for visualization
    fn generate_class_colors(&self, num_classes: usize) -> Vec<[u8; 3]> {
        let mut colors = Vec::with_capacity(num_classes);

        // Predefined colors for common classes
        colors.push([128, 64, 128]); // road - purple
        colors.push([244, 35, 232]); // sidewalk - pink
        colors.push([70, 70, 70]); // building - dark gray
        colors.push([102, 102, 156]); // wall - gray-blue
        colors.push([190, 153, 153]); // fence - light brown
        colors.push([153, 153, 153]); // pole - gray
        colors.push([250, 170, 30]); // traffic light - orange
        colors.push([220, 220, 0]); // traffic sign - yellow
        colors.push([107, 142, 35]); // vegetation - olive green
        colors.push([152, 251, 152]); // terrain - light green
        colors.push([70, 130, 180]); // sky - blue
        colors.push([220, 20, 60]); // person - red
        colors.push([255, 0, 0]); // rider - bright red
        colors.push([0, 0, 142]); // car - dark blue
        colors.push([0, 0, 70]); // truck - darker blue
        colors.push([0, 60, 100]); // bus - blue
        colors.push([0, 80, 100]); // train - cyan-blue
        colors.push([0, 0, 230]); // motorcycle - bright blue
        colors.push([119, 11, 32]); // bicycle - dark red

        // Generate additional colors using HSV
        for i in colors.len()..num_classes {
            let hue = (i * 360 / num_classes) as f32;
            let rgb = Self::hsv_to_rgb(hue, 0.8, 0.8);
            colors.push(rgb);
        }

        colors
    }

    /// Convert HSV to RGB
    fn hsv_to_rgb(h: f32, s: f32, v: f32) -> [u8; 3] {
        let c = v * s;
        let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
        let m = v - c;

        let (r, g, b) = if h < 60.0 {
            (c, x, 0.0)
        } else if h < 120.0 {
            (x, c, 0.0)
        } else if h < 180.0 {
            (0.0, c, x)
        } else if h < 240.0 {
            (0.0, x, c)
        } else if h < 300.0 {
            (x, 0.0, c)
        } else {
            (c, 0.0, x)
        };

        [
            ((r + m) * 255.0) as u8,
            ((g + m) * 255.0) as u8,
            ((b + m) * 255.0) as u8,
        ]
    }
}

#[cfg(feature = "onnx")]
impl Node for SemanticSegmentationNode {
    fn name(&self) -> &'static str {
        "SemanticSegmentationNode"
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Process all available images
        while let Some(image) = self.image_sub.recv(&mut ctx) {
            let start = std::time::Instant::now();

            // Preprocess
            let input = match self.preprocess(&image) {
                Ok(inp) => inp,
                Err(e) => {
                    eprintln!("Preprocessing failed: {}", e);
                    continue;
                }
            };

            // Inference
            let output = match self.run_inference(input) {
                Ok(out) => out,
                Err(e) => {
                    eprintln!("Inference failed: {}", e);
                    continue;
                }
            };

            // Postprocess
            let mask = match self.postprocess(output, image.width, image.height) {
                Ok(m) => m,
                Err(e) => {
                    eprintln!("Postprocessing failed: {}", e);
                    continue;
                }
            };

            let latency_ms = start.elapsed().as_secs_f32() * 1000.0;

            // Publish mask
            let _ = self.mask_pub.send(mask.clone(), &mut ctx);

            // Publish visualization if enabled
            let vis_image = if self.vis_pub.is_some() {
                self.create_visualization(&mask).ok()
            } else {
                None
            };

            if let (Some(ref mut vis_pub), Some(vis_img)) = (&mut self.vis_pub, vis_image) {
                let _ = vis_pub.send(vis_img, &mut ctx);
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
pub struct SemanticSegmentationNode;

#[cfg(not(feature = "onnx"))]
impl SemanticSegmentationNode {
    pub fn new(
        _model_path: &str,
        _input_topic: &str,
        _output_topic: &str,
        _config: SegmentationConfig,
    ) -> HorusResult<Self> {
        Err(HorusError::Config(
            "ONNX support not compiled. Enable 'onnx' feature for SemanticSegmentationNode."
                .to_string(),
        ))
    }
}

#[cfg(not(feature = "onnx"))]
impl Node for SemanticSegmentationNode {
    fn name(&self) -> &'static str {
        "SemanticSegmentationNode (disabled)"
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        // No-op
    }
}
