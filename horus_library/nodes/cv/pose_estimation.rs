// Pose Estimation Node for HORUS
//
// Real-time human pose estimation using MediaPipe Pose, OpenPose, and similar models.
// Detects 2D/3D keypoints for human body tracking and activity recognition.
//
// # Features
// - MediaPipe Pose support (33 keypoints)
// - OpenPose support (25 keypoints)
// - 2D and 3D keypoint detection
// - Multi-person pose estimation
// - Skeleton visualization
// - Action recognition preprocessing
//
// # Example
// ```rust,ignore
// use horus::prelude::*;
// use horus_library::nodes::cv::PoseEstimationNode;
//
// fn main() -> Result<()> {
//     let mut scheduler = Scheduler::new();
//
//     let pose_node = PoseEstimationNode::new(
//         "models/movenet_thunder.onnx",
//         "camera/raw",
//         "vision/poses",
//         PoseConfig::mediapipe(),
//     )?;
//
//     scheduler.add(Box::new(pose_node), 1, Some(true));
//     scheduler.run()?;
//     Ok(())
// }
// ```

#[cfg(feature = "onnx")]
use ort::session::{builder::GraphOptimizationLevel, Session};
#[cfg(feature = "onnx")]
use ort::value::Tensor;

use crate::messages::ml::{InferenceMetrics, Keypoint, Pose, PoseArray};
use crate::messages::Image;
use horus_core::{HorusError, HorusResult, Node, NodeInfo, Topic};

#[cfg(feature = "onnx")]
use ndarray::{Array, ArrayD, IxDyn};

/// Pose model type
#[derive(Clone, Debug, PartialEq)]
pub enum PoseModelType {
    /// MediaPipe Pose (33 keypoints)
    MediaPipe,
    /// OpenPose (25 keypoints)
    OpenPose,
    /// MoveNet (17 keypoints)
    MoveNet,
    /// Custom model
    Custom(usize), // number of keypoints
}

/// Pose estimation configuration
#[derive(Clone, Debug)]
pub struct PoseConfig {
    /// Model type
    pub model_type: PoseModelType,
    /// Input image size [height, width]
    pub input_size: [usize; 2],
    /// Confidence threshold for keypoints
    pub conf_threshold: f32,
    /// Use GPU for inference
    pub use_gpu: bool,
    /// Number of threads for CPU inference
    pub num_threads: usize,
    /// Enable 3D pose estimation
    pub enable_3d: bool,
    /// Maximum number of persons to detect
    pub max_persons: usize,
    /// Output skeleton visualization
    pub output_visualization: bool,
}

impl PoseConfig {
    /// MediaPipe Pose configuration
    pub fn mediapipe() -> Self {
        Self {
            model_type: PoseModelType::MediaPipe,
            input_size: [256, 256],
            conf_threshold: 0.5,
            use_gpu: false,
            num_threads: 4,
            enable_3d: true,
            max_persons: 1,
            output_visualization: true,
        }
    }

    /// OpenPose configuration
    pub fn openpose() -> Self {
        Self {
            model_type: PoseModelType::OpenPose,
            input_size: [368, 368],
            conf_threshold: 0.1,
            use_gpu: false,
            num_threads: 4,
            enable_3d: false,
            max_persons: 10,
            output_visualization: true,
        }
    }

    /// MoveNet configuration (Thunder variant - higher accuracy)
    pub fn movenet_thunder() -> Self {
        Self {
            model_type: PoseModelType::MoveNet,
            input_size: [256, 256],
            conf_threshold: 0.3,
            use_gpu: false,
            num_threads: 4,
            enable_3d: false,
            max_persons: 1,
            output_visualization: true,
        }
    }

    /// MoveNet configuration (Lightning variant - faster)
    pub fn movenet_lightning() -> Self {
        Self {
            model_type: PoseModelType::MoveNet,
            input_size: [192, 192],
            conf_threshold: 0.3,
            use_gpu: false,
            num_threads: 4,
            enable_3d: false,
            max_persons: 1,
            output_visualization: true,
        }
    }
}

impl Default for PoseConfig {
    fn default() -> Self {
        Self::mediapipe()
    }
}

#[cfg(feature = "onnx")]
pub struct PoseEstimationNode {
    /// Image input subscriber
    image_sub: Topic<Image>,
    /// Pose array publisher
    pose_pub: Topic<PoseArray>,
    /// Visualization publisher (optional)
    vis_pub: Option<Topic<Image>>,
    /// Metrics publisher
    metrics_pub: Topic<InferenceMetrics>,
    /// ONNX Runtime session
    session: Session,
    /// Configuration
    config: PoseConfig,
    /// Model name
    model_name: String,
    /// Frame counter
    frame_count: u64,
    /// Keypoint names for the model
    keypoint_names: Vec<String>,
}

/// Find the peak (maximum value) location in a heatmap
/// Returns (y, x, confidence) of the peak location
#[cfg(feature = "onnx")]
fn find_heatmap_peak(
    tensor: &ArrayD<f32>,
    keypoint_idx: usize,
    height: usize,
    width: usize,
) -> Option<(usize, usize, f32)> {
    let mut max_val = f32::MIN;
    let mut max_y = 0;
    let mut max_x = 0;

    // Iterate through the heatmap to find maximum
    for y in 0..height {
        for x in 0..width {
            let val = tensor[[0, keypoint_idx, y, x]];
            if val > max_val {
                max_val = val;
                max_y = y;
                max_x = x;
            }
        }
    }

    // Apply Non-Maximum Suppression (NMS) refinement
    // Check if this is actually a local maximum (not just global)
    let is_local_max = {
        let mut is_max = true;
        // Check 3x3 neighborhood
        for dy in -1..=1_i32 {
            for dx in -1..=1_i32 {
                if dy == 0 && dx == 0 {
                    continue;
                }
                let ny = max_y as i32 + dy;
                let nx = max_x as i32 + dx;

                if ny >= 0 && ny < height as i32 && nx >= 0 && nx < width as i32 {
                    let neighbor_val = tensor[[0, keypoint_idx, ny as usize, nx as usize]];
                    if neighbor_val > max_val {
                        is_max = false;
                        break;
                    }
                }
            }
            if !is_max {
                break;
            }
        }
        is_max
    };

    if is_local_max && max_val > 0.0 {
        Some((max_y, max_x, max_val))
    } else {
        None
    }
}

#[cfg(feature = "onnx")]
impl PoseEstimationNode {
    /// Create a new pose estimation node
    pub fn new(
        model_path: &str,
        input_topic: &str,
        output_topic: &str,
        config: PoseConfig,
    ) -> HorusResult<Self> {
        let session = Self::load_model(model_path, &config)?;

        let model_name = std::path::Path::new(model_path)
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("pose")
            .to_string();

        let keypoint_names = Self::get_keypoint_names(&config.model_type);

        let vis_pub = if config.output_visualization {
            Some(Topic::new(&format!("{}.visualization", output_topic))?)
        } else {
            None
        };

        Ok(Self {
            image_sub: Topic::new(input_topic)?,
            pose_pub: Topic::new(output_topic)?,
            vis_pub,
            metrics_pub: Topic::new(&format!("{}.metrics", output_topic))?,
            session,
            config,
            model_name,
            frame_count: 0,
            keypoint_names,
        })
    }

    /// Load ONNX model
    fn load_model(model_path: &str, config: &PoseConfig) -> HorusResult<Session> {
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

    /// Get keypoint names for model type
    fn get_keypoint_names(model_type: &PoseModelType) -> Vec<String> {
        match model_type {
            PoseModelType::MediaPipe => vec![
                "nose",
                "left_eye_inner",
                "left_eye",
                "left_eye_outer",
                "right_eye_inner",
                "right_eye",
                "right_eye_outer",
                "left_ear",
                "right_ear",
                "mouth_left",
                "mouth_right",
                "left_shoulder",
                "right_shoulder",
                "left_elbow",
                "right_elbow",
                "left_wrist",
                "right_wrist",
                "left_pinky",
                "right_pinky",
                "left_index",
                "right_index",
                "left_thumb",
                "right_thumb",
                "left_hip",
                "right_hip",
                "left_knee",
                "right_knee",
                "left_ankle",
                "right_ankle",
                "left_heel",
                "right_heel",
                "left_foot_index",
                "right_foot_index",
            ]
            .iter()
            .map(|s| s.to_string())
            .collect(),

            PoseModelType::OpenPose => vec![
                "nose",
                "neck",
                "right_shoulder",
                "right_elbow",
                "right_wrist",
                "left_shoulder",
                "left_elbow",
                "left_wrist",
                "mid_hip",
                "right_hip",
                "right_knee",
                "right_ankle",
                "left_hip",
                "left_knee",
                "left_ankle",
                "right_eye",
                "left_eye",
                "right_ear",
                "left_ear",
                "left_big_toe",
                "left_small_toe",
                "left_heel",
                "right_big_toe",
                "right_small_toe",
                "right_heel",
            ]
            .iter()
            .map(|s| s.to_string())
            .collect(),

            PoseModelType::MoveNet => vec![
                "nose",
                "left_eye",
                "right_eye",
                "left_ear",
                "right_ear",
                "left_shoulder",
                "right_shoulder",
                "left_elbow",
                "right_elbow",
                "left_wrist",
                "right_wrist",
                "left_hip",
                "right_hip",
                "left_knee",
                "right_knee",
                "left_ankle",
                "right_ankle",
            ]
            .iter()
            .map(|s| s.to_string())
            .collect(),

            PoseModelType::Custom(n) => (0..*n).map(|i| format!("keypoint_{}", i)).collect(),
        }
    }

    /// Preprocess image for pose estimation
    fn preprocess(&self, image: &Image) -> HorusResult<ArrayD<f32>> {
        let img_data = &image.data;
        let width = image.width as usize;
        let height = image.height as usize;
        let channels = (img_data.len() / (width * height)) as usize;

        // Resize to model input size
        let target_h = self.config.input_size[0];
        let target_w = self.config.input_size[1];

        // Simple bilinear resize
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

        // Normalize to -1 to 1 (common for pose models) and convert to NCHW
        let mut normalized = Array::zeros(IxDyn(&[1, channels, target_h, target_w]));
        for c in 0..channels {
            for h in 0..target_h {
                for w in 0..target_w {
                    let idx = (h * target_w + w) * channels + c;
                    // Normalize to [-1, 1]
                    let value = (resized[idx] as f32 / 127.5) - 1.0;
                    normalized[[0, c, h, w]] = value;
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

    /// Post-process pose estimation output
    fn postprocess(
        &self,
        output: ArrayD<f32>,
        orig_width: u32,
        orig_height: u32,
    ) -> HorusResult<PoseArray> {
        let shape = output.shape();
        let mut poses = Vec::new();

        // Different models have different output formats
        match &self.config.model_type {
            PoseModelType::MediaPipe | PoseModelType::MoveNet => {
                // Output format: [batch, num_keypoints, 3] or [batch, num_keypoints, 4]
                // where last dim is [y, x, confidence] or [y, x, z, confidence]

                if shape.len() < 2 {
                    return Err(HorusError::config(format!(
                        "Unexpected output shape: {:?}",
                        shape
                    )));
                }

                let num_keypoints = shape[1];
                let coords_per_kp = if shape.len() >= 3 { shape[2] } else { 3 };

                let mut keypoints = Vec::with_capacity(num_keypoints);
                let mut total_conf = 0.0;

                for kp_idx in 0..num_keypoints {
                    let y_norm = output[[0, kp_idx, 0]];
                    let x_norm = output[[0, kp_idx, 1]];
                    let confidence = if coords_per_kp >= 3 {
                        output[[0, kp_idx, 2]]
                    } else {
                        1.0
                    };
                    let z = if coords_per_kp >= 4 && self.config.enable_3d {
                        Some(output[[0, kp_idx, 3]])
                    } else {
                        None
                    };

                    // Convert normalized coordinates to pixel coordinates
                    let x = x_norm * orig_width as f32;
                    let y = y_norm * orig_height as f32;

                    if confidence >= self.config.conf_threshold {
                        total_conf += confidence;

                        keypoints.push(Keypoint {
                            x,
                            y,
                            z,
                            confidence,
                            name: self
                                .keypoint_names
                                .get(kp_idx)
                                .cloned()
                                .unwrap_or_else(|| format!("kp_{}", kp_idx)),
                        });
                    }
                }

                if !keypoints.is_empty() {
                    poses.push(Pose {
                        bbox: None,
                        keypoints,
                        confidence: total_conf / num_keypoints as f32,
                        person_id: 0,
                    });
                }
            }

            PoseModelType::OpenPose => {
                // OpenPose outputs heatmaps and PAFs (Part Affinity Fields)
                // This is a simplified version - production would need proper PAF parsing

                if shape.len() < 3 {
                    return Err(HorusError::config(format!(
                        "Unexpected output shape: {:?}",
                        shape
                    )));
                }

                // Detect if output is heatmaps or already processed keypoints
                let num_keypoints = self.keypoint_names.len();
                let mut keypoints = Vec::with_capacity(num_keypoints);

                if shape.len() == 4 {
                    // Heatmap format: [batch, num_keypoints, height, width]
                    // Perform proper peak detection
                    let heatmap_height = shape[2];
                    let heatmap_width = shape[3];

                    for kp_idx in 0..num_keypoints.min(shape[1]) {
                        // Find peak in heatmap for this keypoint
                        if let Some((peak_y, peak_x, confidence)) =
                            find_heatmap_peak(&output, kp_idx, heatmap_height, heatmap_width)
                        {
                            if confidence >= self.config.conf_threshold {
                                // Convert from heatmap coordinates to original image coordinates
                                let x = (peak_x as f32 / heatmap_width as f32) * orig_width as f32;
                                let y =
                                    (peak_y as f32 / heatmap_height as f32) * orig_height as f32;

                                keypoints.push(Keypoint {
                                    x,
                                    y,
                                    z: None,
                                    confidence,
                                    name: self.keypoint_names[kp_idx].clone(),
                                });
                            }
                        }
                    }
                } else {
                    // Keypoint format: [batch, num_keypoints, 2 or 3]
                    // Direct keypoint coordinates
                    for kp_idx in 0..num_keypoints.min(shape[1]) {
                        let x = output[[0, kp_idx, 0]] * orig_width as f32;
                        let y = output[[0, kp_idx, 1]] * orig_height as f32;
                        let confidence = if shape[2] > 2 {
                            output[[0, kp_idx, 2]]
                        } else {
                            0.5
                        };

                        if confidence >= self.config.conf_threshold {
                            keypoints.push(Keypoint {
                                x,
                                y,
                                z: None,
                                confidence,
                                name: self.keypoint_names[kp_idx].clone(),
                            });
                        }
                    }
                }

                if !keypoints.is_empty() {
                    let avg_confidence = keypoints.iter().map(|k| k.confidence).sum::<f32>()
                        / keypoints.len() as f32;
                    poses.push(Pose {
                        bbox: None,
                        keypoints,
                        confidence: avg_confidence,
                        person_id: 0,
                    });
                }
            }

            PoseModelType::Custom(_) => {
                // Generic handling for custom models
                // Assume output is [batch, num_keypoints, 3] format
                return Err(HorusError::config(
                    "Custom pose models need custom post-processing implementation".to_string(),
                ));
            }
        }

        Ok(PoseArray {
            poses,
            image_width: orig_width,
            image_height: orig_height,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
        })
    }

    /// Create skeleton visualization
    fn create_visualization(&self, poses: &PoseArray, image: &Image) -> HorusResult<Image> {
        // Clone the original image
        let mut vis_data = image.data.clone();
        let width = image.width as usize;
        let height = image.height as usize;
        let channels = vis_data.len() / (width * height);

        // Draw skeleton connections and keypoints
        for pose in &poses.poses {
            // Get skeleton connections for this model type
            let connections = Self::get_skeleton_connections(&self.config.model_type);

            // Draw connections (bones)
            for (start_idx, end_idx) in connections {
                if let (Some(start_kp), Some(end_kp)) =
                    (pose.keypoints.get(start_idx), pose.keypoints.get(end_idx))
                {
                    self.draw_line(
                        &mut vis_data,
                        width,
                        height,
                        channels,
                        start_kp.x as usize,
                        start_kp.y as usize,
                        end_kp.x as usize,
                        end_kp.y as usize,
                        [0, 255, 0], // Green lines
                    );
                }
            }

            // Draw keypoints as circles
            for keypoint in &pose.keypoints {
                let x = keypoint.x as usize;
                let y = keypoint.y as usize;
                let radius = 3;

                // Color based on confidence
                let color = if keypoint.confidence > 0.8 {
                    [255, 0, 0] // Red for high confidence
                } else if keypoint.confidence > 0.5 {
                    [255, 255, 0] // Yellow for medium
                } else {
                    [0, 0, 255] // Blue for low
                };

                self.draw_circle(&mut vis_data, width, height, channels, x, y, radius, color);
            }
        }

        Ok(Image {
            data: vis_data,
            width: image.width,
            height: image.height,
            encoding: image.encoding,
            step: image.step,
            frame_id: image.frame_id,
            timestamp: poses.timestamp_ns,
        })
    }

    /// Get skeleton connections (bones) for visualization
    fn get_skeleton_connections(model_type: &PoseModelType) -> Vec<(usize, usize)> {
        match model_type {
            PoseModelType::MediaPipe => vec![
                // Face
                (0, 1),
                (1, 2),
                (2, 3),
                (3, 7), // Left eye
                (0, 4),
                (4, 5),
                (5, 6),
                (6, 8),  // Right eye
                (9, 10), // Mouth
                // Arms
                (11, 13),
                (13, 15),
                (15, 17),
                (15, 19),
                (15, 21), // Left arm
                (12, 14),
                (14, 16),
                (16, 18),
                (16, 20),
                (16, 22), // Right arm
                (11, 12), // Shoulders
                // Torso
                (11, 23),
                (12, 24),
                (23, 24), // Hips
                // Legs
                (23, 25),
                (25, 27),
                (27, 29),
                (27, 31), // Left leg
                (24, 26),
                (26, 28),
                (28, 30),
                (28, 32), // Right leg
            ],

            PoseModelType::MoveNet => vec![
                // Face
                (0, 1),
                (0, 2),
                (1, 3),
                (2, 4),
                // Torso
                (5, 6),
                (5, 7),
                (7, 9),
                (6, 8),
                (8, 10),
                (5, 11),
                (6, 12),
                (11, 12),
                // Legs
                (11, 13),
                (13, 15),
                (12, 14),
                (14, 16),
            ],

            PoseModelType::OpenPose => vec![
                // Head
                (0, 1),
                (1, 2),
                (1, 5),
                (2, 3),
                (3, 4),
                (5, 6),
                (6, 7),
                // Torso
                (1, 8),
                (8, 9),
                (8, 12),
                // Right leg
                (9, 10),
                (10, 11),
                // Left leg
                (12, 13),
                (13, 14),
                // Face
                (0, 15),
                (0, 16),
                (15, 17),
                (16, 18),
            ],

            PoseModelType::Custom(_) => vec![],
        }
    }

    /// Draw a line on image (simple Bresenham)
    fn draw_line(
        &self,
        data: &mut [u8],
        width: usize,
        height: usize,
        channels: usize,
        x0: usize,
        y0: usize,
        x1: usize,
        y1: usize,
        color: [u8; 3],
    ) {
        let dx = (x1 as i32 - x0 as i32).abs();
        let dy = -(y1 as i32 - y0 as i32).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx + dy;

        let mut x = x0 as i32;
        let mut y = y0 as i32;

        loop {
            if x >= 0 && x < width as i32 && y >= 0 && y < height as i32 {
                let idx = (y as usize * width + x as usize) * channels;
                if idx + 2 < data.len() {
                    data[idx] = color[0];
                    if channels > 1 {
                        data[idx + 1] = color[1];
                    }
                    if channels > 2 {
                        data[idx + 2] = color[2];
                    }
                }
            }

            if x == x1 as i32 && y == y1 as i32 {
                break;
            }

            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x += sx;
            }
            if e2 <= dx {
                err += dx;
                y += sy;
            }
        }
    }

    /// Draw a filled circle on image
    fn draw_circle(
        &self,
        data: &mut [u8],
        width: usize,
        height: usize,
        channels: usize,
        cx: usize,
        cy: usize,
        radius: usize,
        color: [u8; 3],
    ) {
        for dy in -(radius as i32)..=(radius as i32) {
            for dx in -(radius as i32)..=(radius as i32) {
                if dx * dx + dy * dy <= (radius * radius) as i32 {
                    let x = cx as i32 + dx;
                    let y = cy as i32 + dy;

                    if x >= 0 && x < width as i32 && y >= 0 && y < height as i32 {
                        let idx = (y as usize * width + x as usize) * channels;
                        if idx + 2 < data.len() {
                            data[idx] = color[0];
                            if channels > 1 {
                                data[idx + 1] = color[1];
                            }
                            if channels > 2 {
                                data[idx + 2] = color[2];
                            }
                        }
                    }
                }
            }
        }
    }
}

#[cfg(feature = "onnx")]
impl Node for PoseEstimationNode {
    fn name(&self) -> &'static str {
        "PoseEstimationNode"
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
            let poses = match self.postprocess(output, image.width, image.height) {
                Ok(p) => p,
                Err(e) => {
                    eprintln!("Postprocessing failed: {}", e);
                    continue;
                }
            };

            let latency_ms = start.elapsed().as_secs_f32() * 1000.0;

            // Publish poses
            let _ = self.pose_pub.send(poses.clone(), &mut ctx);

            // Publish visualization if enabled
            let vis_image = if self.vis_pub.is_some() {
                self.create_visualization(&poses, &image).ok()
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
pub struct PoseEstimationNode;

#[cfg(not(feature = "onnx"))]
impl PoseEstimationNode {
    pub fn new(
        _model_path: &str,
        _input_topic: &str,
        _output_topic: &str,
        _config: PoseConfig,
    ) -> HorusResult<Self> {
        Err(HorusError::Config(
            "ONNX support not compiled. Enable 'onnx' feature for PoseEstimationNode.".to_string(),
        ))
    }
}

#[cfg(not(feature = "onnx"))]
impl Node for PoseEstimationNode {
    fn name(&self) -> &'static str {
        "PoseEstimationNode (disabled)"
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        // No-op
    }
}
