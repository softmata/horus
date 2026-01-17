// YOLOv8 Object Detection Node for HORUS
//
// Real-time object detection using YOLOv8 models (n/s/m/l/x variants).
// Supports COCO dataset and custom trained models.
//
// # Features
// - YOLOv8n/s/m/l/x model support
// - Real-time inference (30-60 FPS on GPU)
// - Non-Maximum Suppression (NMS)
// - Confidence filtering
// - Bounding box visualization (optional)
// - Multiple detection formats
//
// # Example
// ```rust,ignore
// use horus::prelude::*;
// use horus_library::nodes::cv::YOLOv8DetectorNode;
//
// fn main() -> Result<()> {
//     let mut scheduler = Scheduler::new();
//
//     let detector = YOLOv8DetectorNode::new(
//         "models/yolov8n.onnx",
//         "camera/raw",
//         "vision/detections",
//         YOLOConfig::default(),
//     )?;
//
//     scheduler.add(Box::new(detector), 1, Some(true));
//     scheduler.run()?;
//     Ok(())
// }
// ```

#[cfg(feature = "onnx")]
use ort::session::{builder::GraphOptimizationLevel, Session};
#[cfg(feature = "onnx")]
use ort::value::Tensor;

use crate::messages::ml::InferenceMetrics;
use crate::messages::{Detection, DetectionArray, Image, RegionOfInterest};
use horus_core::{HorusError, HorusResult, Node, NodeInfo, Topic};
use std::path::Path;
use std::time::Instant;

#[cfg(feature = "onnx")]
use ndarray::{Array, ArrayD, IxDyn};

/// YOLOv8 Configuration
#[derive(Clone, Debug)]
pub struct YOLOConfig {
    /// Confidence threshold (0.0 to 1.0)
    pub conf_threshold: f32,
    /// IoU threshold for NMS (0.0 to 1.0)
    pub iou_threshold: f32,
    /// Maximum number of detections to return
    pub max_detections: usize,
    /// Use GPU if available
    pub use_gpu: bool,
    /// GPU device ID
    pub device_id: u32,
    /// Input size (must match model: 640, 320, etc.)
    pub input_size: u32,
    /// Class names (COCO by default)
    pub class_names: Vec<String>,
    /// Enable visualization output
    pub enable_visualization: bool,
}

impl Default for YOLOConfig {
    fn default() -> Self {
        Self {
            conf_threshold: 0.25,
            iou_threshold: 0.45,
            max_detections: 100,
            use_gpu: false,
            device_id: 0,
            input_size: 640,
            class_names: Self::coco_classes(),
            enable_visualization: false,
        }
    }
}

impl YOLOConfig {
    /// Get COCO dataset class names (80 classes)
    pub fn coco_classes() -> Vec<String> {
        vec![
            "person",
            "bicycle",
            "car",
            "motorcycle",
            "airplane",
            "bus",
            "train",
            "truck",
            "boat",
            "traffic light",
            "fire hydrant",
            "stop sign",
            "parking meter",
            "bench",
            "bird",
            "cat",
            "dog",
            "horse",
            "sheep",
            "cow",
            "elephant",
            "bear",
            "zebra",
            "giraffe",
            "backpack",
            "umbrella",
            "handbag",
            "tie",
            "suitcase",
            "frisbee",
            "skis",
            "snowboard",
            "sports ball",
            "kite",
            "baseball bat",
            "baseball glove",
            "skateboard",
            "surfboard",
            "tennis racket",
            "bottle",
            "wine glass",
            "cup",
            "fork",
            "knife",
            "spoon",
            "bowl",
            "banana",
            "apple",
            "sandwich",
            "orange",
            "broccoli",
            "carrot",
            "hot dog",
            "pizza",
            "donut",
            "cake",
            "chair",
            "couch",
            "potted plant",
            "bed",
            "dining table",
            "toilet",
            "tv",
            "laptop",
            "mouse",
            "remote",
            "keyboard",
            "cell phone",
            "microwave",
            "oven",
            "toaster",
            "sink",
            "refrigerator",
            "book",
            "clock",
            "vase",
            "scissors",
            "teddy bear",
            "hair drier",
            "toothbrush",
        ]
        .iter()
        .map(|s| s.to_string())
        .collect()
    }
}

/// YOLOv8 Object Detector Node
///
/// Performs real-time object detection using YOLOv8 ONNX models.
#[cfg(feature = "onnx")]
pub struct YOLOv8DetectorNode {
    /// Input hub for images
    image_sub: Topic<Image>,
    /// Output hub for detections
    detections_pub: Topic<DetectionArray>,
    /// Output hub for annotated images (optional)
    #[allow(dead_code)]
    annotated_pub: Option<Topic<Image>>,
    /// Output hub for metrics
    metrics_pub: Topic<InferenceMetrics>,
    /// ONNX Runtime session
    session: Session,
    /// YOLO configuration
    config: YOLOConfig,
    /// Model name
    model_name: String,
    /// Frame counter
    frame_count: u64,
}

#[cfg(feature = "onnx")]
impl YOLOv8DetectorNode {
    /// Create a new YOLOv8 detector node
    pub fn new(
        model_path: &str,
        input_topic: &str,
        output_topic: &str,
        config: YOLOConfig,
    ) -> HorusResult<Self> {
        let session = Self::load_model(model_path, &config)?;

        let model_name = Path::new(model_path)
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("yolov8")
            .to_string();

        let annotated_pub = if config.enable_visualization {
            Some(Topic::new(&format!("{}.annotated", output_topic))?)
        } else {
            None
        };

        Ok(Self {
            image_sub: Topic::new(input_topic)?,
            detections_pub: Topic::new(output_topic)?,
            annotated_pub,
            metrics_pub: Topic::new(&format!("{}.metrics", output_topic))?,
            session,
            config,
            model_name,
            frame_count: 0,
        })
    }

    /// Load YOLO ONNX model
    fn load_model(model_path: &str, config: &YOLOConfig) -> HorusResult<Session> {
        if !Path::new(model_path).exists() {
            return Err(HorusError::Config(format!(
                "Model file not found: {}",
                model_path
            )));
        }

        #[cfg_attr(not(feature = "cuda"), allow(unused_mut))]
        let mut builder = Session::builder()
            .map_err(|e| HorusError::Config(format!("Failed to create session builder: {}", e)))?
            .with_optimization_level(GraphOptimizationLevel::Level3)
            .map_err(|e| HorusError::Config(format!("Failed to set optimization level: {}", e)))?;

        // Add GPU support if requested
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
            .map_err(|e| HorusError::Config(format!("Failed to load YOLO model: {}", e)))?;

        Ok(session)
    }

    /// Preprocess image for YOLOv8 (letterbox + normalize)
    fn preprocess(&self, image: &Image) -> HorusResult<(ArrayD<f32>, f32, f32, f32, f32)> {
        let img_w = image.width as usize;
        let img_h = image.height as usize;
        let model_size = self.config.input_size as usize;

        // Calculate letterbox parameters
        let scale = (model_size as f32 / img_w as f32).min(model_size as f32 / img_h as f32);
        let new_w = (img_w as f32 * scale) as usize;
        let new_h = (img_h as f32 * scale) as usize;
        let pad_w = (model_size - new_w) as f32 / 2.0;
        let pad_h = (model_size - new_h) as f32 / 2.0;

        // Resize and pad image
        let mut input_data = vec![0.0f32; model_size * model_size * 3];

        // Simple nearest-neighbor resize with letterbox padding
        for y in 0..new_h {
            for x in 0..new_w {
                let src_x = ((x as f32 / scale) as usize).min(img_w - 1);
                let src_y = ((y as f32 / scale) as usize).min(img_h - 1);

                let dst_x = x + pad_w as usize;
                let dst_y = y + pad_h as usize;

                if dst_x < model_size && dst_y < model_size {
                    for c in 0..3 {
                        let src_idx = (src_y * img_w + src_x) * 3 + c;
                        let dst_idx = c * model_size * model_size + dst_y * model_size + dst_x;

                        if src_idx < image.data.len() {
                            input_data[dst_idx] = image.data[src_idx] as f32 / 255.0;
                        }
                    }
                }
            }
        }

        let input = Array::from_shape_vec(IxDyn(&[1, 3, model_size, model_size]), input_data)
            .map_err(|e| HorusError::Config(format!("Array reshape failed: {}", e)))?;

        Ok((input, scale, pad_w, pad_h, 1.0 / scale))
    }

    /// Run YOLOv8 inference
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

    /// Parse YOLOv8 output format: [batch, 84, 8400] -> detections
    /// YOLOv8 output: [x_center, y_center, width, height, class0_conf, class1_conf, ...]
    fn parse_output(
        &self,
        output: ArrayD<f32>,
        scale: f32,
        pad_w: f32,
        pad_h: f32,
        img_width: u32,
        img_height: u32,
    ) -> HorusResult<Vec<Detection>> {
        let shape = output.shape();

        // YOLOv8 output is typically [1, 84, 8400] for COCO (80 classes + 4 bbox coords)
        // or [1, num_classes+4, num_predictions]
        if shape.len() != 3 || shape[0] != 1 {
            return Err(HorusError::config(format!(
                "Unexpected output shape: {:?}",
                shape
            )));
        }

        let num_preds = shape[2];
        let num_classes = shape[1] - 4;

        let mut detections = Vec::new();

        // Transpose from [1, 84, 8400] to [8400, 84] for easier access
        for pred_idx in 0..num_preds {
            // Extract bbox [x_center, y_center, width, height]
            let x_center = output[[0, 0, pred_idx]];
            let y_center = output[[0, 1, pred_idx]];
            let width = output[[0, 2, pred_idx]];
            let height = output[[0, 3, pred_idx]];

            // Find class with max confidence
            let mut max_conf = 0.0f32;
            let mut max_class = 0usize;

            for c in 0..num_classes {
                let conf = output[[0, 4 + c, pred_idx]];
                if conf > max_conf {
                    max_conf = conf;
                    max_class = c;
                }
            }

            // Filter by confidence threshold
            if max_conf < self.config.conf_threshold {
                continue;
            }

            // Convert from model coordinates to original image coordinates
            let x = (x_center - pad_w) / scale;
            let y = (y_center - pad_h) / scale;
            let w = width / scale;
            let h = height / scale;

            // Convert center format to corner format [x, y, w, h]
            let x1 = (x - w / 2.0).max(0.0).min(img_width as f32);
            let y1 = (y - h / 2.0).max(0.0).min(img_height as f32);
            let box_w = w.min(img_width as f32 - x1);
            let box_h = h.min(img_height as f32 - y1);

            // Convert class name to [u8; 32]
            let mut class_name_bytes = [0u8; 32];
            if let Some(name) = self.config.class_names.get(max_class) {
                let name_bytes = name.as_bytes();
                let copy_len = name_bytes.len().min(31); // Leave room for null terminator
                class_name_bytes[..copy_len].copy_from_slice(&name_bytes[..copy_len]);
            }

            detections.push(Detection {
                class_name: class_name_bytes,
                confidence: max_conf,
                bbox: RegionOfInterest {
                    x_offset: x1 as u32,
                    y_offset: y1 as u32,
                    width: box_w as u32,
                    height: box_h as u32,
                    do_rectify: false,
                },
                pose: None,
                track_id: 0,
                timestamp: std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as u64,
            });
        }

        // Apply NMS
        let filtered = self.non_maximum_suppression(detections);

        Ok(filtered)
    }

    /// Non-Maximum Suppression (NMS)
    fn non_maximum_suppression(&self, mut detections: Vec<Detection>) -> Vec<Detection> {
        if detections.is_empty() {
            return detections;
        }

        // Sort by confidence (descending)
        detections.sort_by(|a, b| b.confidence.partial_cmp(&a.confidence).unwrap());

        let mut keep = Vec::new();
        let mut suppressed = vec![false; detections.len()];

        for i in 0..detections.len() {
            if suppressed[i] {
                continue;
            }

            keep.push(detections[i].clone());

            if keep.len() >= self.config.max_detections {
                break;
            }

            // Suppress overlapping boxes
            for j in (i + 1)..detections.len() {
                if suppressed[j] {
                    continue;
                }

                let iou = Self::calculate_iou(&detections[i].bbox, &detections[j].bbox);
                if iou > self.config.iou_threshold {
                    suppressed[j] = true;
                }
            }
        }

        keep
    }

    /// Calculate Intersection over Union (IoU)
    fn calculate_iou(box1: &RegionOfInterest, box2: &RegionOfInterest) -> f32 {
        let x1_max = (box1.x_offset as f32).max(box2.x_offset as f32);
        let y1_max = (box1.y_offset as f32).max(box2.y_offset as f32);
        let x2_min = ((box1.x_offset + box1.width) as f32).min((box2.x_offset + box2.width) as f32);
        let y2_min =
            ((box1.y_offset + box1.height) as f32).min((box2.y_offset + box2.height) as f32);

        let intersection_w = (x2_min - x1_max).max(0.0);
        let intersection_h = (y2_min - y1_max).max(0.0);
        let intersection = intersection_w * intersection_h;

        let area1 = (box1.width * box1.height) as f32;
        let area2 = (box2.width * box2.height) as f32;
        let union = area1 + area2 - intersection;

        if union > 0.0 {
            intersection / union
        } else {
            0.0
        }
    }
}

#[cfg(feature = "onnx")]
impl Node for YOLOv8DetectorNode {
    fn name(&self) -> &'static str {
        "YOLOv8DetectorNode"
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        if let Some(image) = self.image_sub.recv(&mut ctx) {
            let start = Instant::now();

            let img_width = image.width;
            let img_height = image.height;
            let timestamp = image.timestamp;

            // Preprocess
            let (input, scale, pad_w, pad_h, _inv_scale) = match self.preprocess(&image) {
                Ok(data) => data,
                Err(e) => {
                    eprintln!("Preprocessing failed: {}", e);
                    return;
                }
            };

            // Run inference
            let output = match self.run_inference(input) {
                Ok(o) => o,
                Err(e) => {
                    eprintln!("Inference failed: {}", e);
                    return;
                }
            };

            // Parse detections
            let detections =
                match self.parse_output(output, scale, pad_w, pad_h, img_width, img_height) {
                    Ok(d) => d,
                    Err(e) => {
                        eprintln!("Output parsing failed: {}", e);
                        return;
                    }
                };

            let latency = start.elapsed().as_secs_f32() * 1000.0;

            // Convert Vec<Detection> to fixed array [Detection; 32]
            let count = detections.len().min(32) as u8;
            let mut detection_array_data = [Detection::default(); 32];
            for (i, det) in detections.iter().take(32).enumerate() {
                detection_array_data[i] = det.clone();
            }

            // Send detections
            let detection_array = DetectionArray {
                detections: detection_array_data,
                count,
                image_width: img_width,
                image_height: img_height,
                frame_id: [0u8; 32],
                timestamp,
            };

            let _ = self.detections_pub.send(detection_array, &mut ctx);

            // Send metrics
            let metrics = InferenceMetrics {
                latency_ms: latency,
                throughput: 1000.0 / latency,
                model_name: self.model_name.clone(),
                batch_size: 1,
                timestamp_ns: timestamp,
            };
            let _ = self.metrics_pub.send(metrics, &mut ctx);

            self.frame_count += 1;
        }
    }
}

// Stub implementation when ONNX feature is disabled
#[cfg(not(feature = "onnx"))]
pub struct YOLOv8DetectorNode;

#[cfg(not(feature = "onnx"))]
impl YOLOv8DetectorNode {
    pub fn new(
        _model_path: &str,
        _input_topic: &str,
        _output_topic: &str,
        _config: YOLOConfig,
    ) -> HorusResult<Self> {
        Err(HorusError::Config(
            "ONNX support not compiled. Enable 'onnx' feature.".to_string(),
        ))
    }
}

#[cfg(not(feature = "onnx"))]
impl Node for YOLOv8DetectorNode {
    fn name(&self) -> &'static str {
        "YOLOv8DetectorNode"
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {}
}
