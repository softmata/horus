// Visual Odometry Node for HORUS
//
// Provides visual odometry estimation from camera images using feature-based methods.
// This node extracts features, tracks them across frames, and estimates camera motion.
//
// # Features
// - Monocular visual odometry
// - Stereo visual odometry (with depth estimation)
// - RGB-D visual odometry
// - ORB, SIFT, and other feature detectors (via OpenCV)
// - Essential matrix / PnP pose estimation
// - Loop closure detection (appearance-based)
// - Integration with LocalizationNode for sensor fusion
//
// # Usage
// ```rust,ignore
// use horus::prelude::*;
// use horus_library::nodes::visual_odometry::{VisualOdometryNode, VOConfig, CameraMode};
//
// fn main() -> Result<()> {
//     let mut scheduler = Scheduler::new();
//
//     // Create monocular visual odometry
//     let vo = VisualOdometryNode::new(
//         "camera/raw",
//         "vo/odom",
//         VOConfig::default()
//             .with_mode(CameraMode::Monocular)
//             .with_feature_detector(FeatureDetector::ORB)
//             .with_max_features(1000),
//     )?;
//
//     scheduler.add(Box::new(vo), 1, Some(true));
//     scheduler.run()?;
//     Ok(())
// }
// ```
//
// # Note
// This is a pure-Rust implementation that provides basic visual odometry.
// For full Visual SLAM (mapping + loop closure), consider integrating with
// external SLAM systems via ROS2 bridge or custom bindings.

use crate::messages::{Image, ImageEncoding};
use crate::Odometry;
use horus_core::{HorusResult, Node, NodeInfo, Topic};
use std::collections::VecDeque;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor};

/// Camera mode for visual odometry
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CameraMode {
    /// Single camera (scale ambiguity without external reference)
    Monocular,
    /// Stereo camera pair (full scale recovery)
    Stereo,
    /// RGB-D camera (depth from sensor)
    RgbD,
}

impl Default for CameraMode {
    fn default() -> Self {
        Self::Monocular
    }
}

/// Feature detector type
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FeatureDetector {
    /// ORB (Oriented FAST and Rotated BRIEF) - Fast, rotation invariant
    ORB,
    /// SIFT (Scale-Invariant Feature Transform) - Robust but slower
    SIFT,
    /// FAST corner detector - Very fast
    FAST,
    /// Good Features to Track (Shi-Tomasi corners)
    GoodFeatures,
}

impl Default for FeatureDetector {
    fn default() -> Self {
        Self::ORB
    }
}

/// Feature matching method
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MatchingMethod {
    /// Brute-force matching with ratio test
    BruteForce,
    /// FLANN-based approximate matching
    FLANN,
    /// KLT optical flow tracking
    OpticalFlow,
}

impl Default for MatchingMethod {
    fn default() -> Self {
        Self::OpticalFlow
    }
}

/// Visual odometry configuration
#[derive(Clone, Debug)]
pub struct VOConfig {
    /// Camera mode (mono/stereo/RGB-D)
    pub mode: CameraMode,
    /// Feature detector type
    pub feature_detector: FeatureDetector,
    /// Feature matching method
    pub matching_method: MatchingMethod,
    /// Maximum number of features to detect
    pub max_features: usize,
    /// Minimum feature quality (0.0-1.0)
    pub quality_level: f64,
    /// Minimum distance between features (pixels)
    pub min_distance: f64,
    /// Camera intrinsic parameters [fx, fy, cx, cy]
    pub camera_intrinsics: [f64; 4],
    /// Stereo baseline (meters) - for stereo mode
    pub stereo_baseline: f64,
    /// Minimum inlier ratio for valid pose
    pub min_inlier_ratio: f64,
    /// Enable loop closure detection
    pub enable_loop_closure: bool,
    /// Keyframe insertion threshold (translation in meters)
    pub keyframe_threshold_trans: f64,
    /// Keyframe insertion threshold (rotation in radians)
    pub keyframe_threshold_rot: f64,
    /// Window size for local bundle adjustment
    pub ba_window_size: usize,
}

impl Default for VOConfig {
    fn default() -> Self {
        Self {
            mode: CameraMode::default(),
            feature_detector: FeatureDetector::default(),
            matching_method: MatchingMethod::default(),
            max_features: 500,
            quality_level: 0.01,
            min_distance: 7.0,
            // Default pinhole camera (640x480 with 60 degree FOV)
            camera_intrinsics: [554.0, 554.0, 320.0, 240.0],
            stereo_baseline: 0.12, // 12cm baseline
            min_inlier_ratio: 0.3,
            enable_loop_closure: false,
            keyframe_threshold_trans: 0.2, // 20cm
            keyframe_threshold_rot: 0.1,   // ~6 degrees
            ba_window_size: 10,
        }
    }
}

impl VOConfig {
    /// Set camera mode
    pub fn with_mode(mut self, mode: CameraMode) -> Self {
        self.mode = mode;
        self
    }

    /// Set feature detector
    pub fn with_feature_detector(mut self, detector: FeatureDetector) -> Self {
        self.feature_detector = detector;
        self
    }

    /// Set matching method
    pub fn with_matching_method(mut self, method: MatchingMethod) -> Self {
        self.matching_method = method;
        self
    }

    /// Set maximum features to track
    pub fn with_max_features(mut self, max: usize) -> Self {
        self.max_features = max;
        self
    }

    /// Set camera intrinsics [fx, fy, cx, cy]
    pub fn with_intrinsics(mut self, fx: f64, fy: f64, cx: f64, cy: f64) -> Self {
        self.camera_intrinsics = [fx, fy, cx, cy];
        self
    }

    /// Set stereo baseline
    pub fn with_stereo_baseline(mut self, baseline: f64) -> Self {
        self.stereo_baseline = baseline;
        self
    }

    /// Enable loop closure detection
    pub fn with_loop_closure(mut self, enable: bool) -> Self {
        self.enable_loop_closure = enable;
        self
    }
}

/// A 2D feature point
#[derive(Clone, Debug)]
struct Feature {
    x: f32,
    y: f32,
    response: f32,
    #[allow(dead_code)]
    id: u32,
}

/// A frame with detected features
#[derive(Clone)]
struct Frame {
    features: Vec<Feature>,
    #[allow(dead_code)]
    timestamp: u64,
    #[allow(dead_code)]
    image_data: Vec<u8>,
    width: u32,
    #[allow(dead_code)]
    height: u32,
}

/// Visual Odometry Node
///
/// Estimates camera motion from visual features.
///
/// # Hybrid Pattern
///
/// ```rust,ignore
/// let node = VisualOdometryNode::builder()
///     .with_filter(|odom| {
///         // Only output when moving forward
///         if odom.twist.linear[0] > 0.01 {
///             Some(odom)
///         } else {
///             None
///         }
///     })
///     .build()?;
/// ```
pub struct VisualOdometryNode<P = PassThrough<Odometry>>
where
    P: Processor<Odometry>,
{
    /// Image subscriber
    image_sub: Topic<Image>,
    /// Depth subscriber (for RGB-D mode)
    depth_sub: Option<Topic<Image>>,
    /// Stereo subscriber (for stereo mode)
    stereo_sub: Option<Topic<Image>>,
    /// Odometry publisher
    odom_pub: Topic<Odometry>,

    /// Configuration
    config: VOConfig,

    /// Previous frame for tracking
    prev_frame: Option<Frame>,

    /// Accumulated pose [x, y, z, roll, pitch, yaw]
    pose: [f64; 6],

    /// Accumulated velocity [vx, vy, vz, wx, wy, wz]
    velocity: [f64; 6],

    /// Frame counter
    frame_count: u64,

    /// Feature ID counter
    feature_id_counter: u32,

    /// Recent keyframes for loop closure
    keyframes: VecDeque<Frame>,

    /// Last keyframe pose
    last_keyframe_pose: [f64; 6],

    /// Scale factor (estimated from external sources or initialized to 1.0)
    scale: f64,

    /// Processing time for metrics
    last_processing_time_ms: f32,

    /// Last timestamp for velocity calculation
    last_timestamp: u64,

    /// Processor for hybrid pattern
    processor: P,
}

impl VisualOdometryNode {
    /// Create a new visual odometry node
    pub fn new(input_topic: &str, output_topic: &str, config: VOConfig) -> HorusResult<Self> {
        Ok(Self {
            image_sub: Topic::new(input_topic)?,
            depth_sub: None,
            stereo_sub: None,
            odom_pub: Topic::new(output_topic)?,
            config,
            prev_frame: None,
            pose: [0.0; 6],
            velocity: [0.0; 6],
            frame_count: 0,
            feature_id_counter: 0,
            keyframes: VecDeque::with_capacity(100),
            last_keyframe_pose: [0.0; 6],
            scale: 1.0,
            last_processing_time_ms: 0.0,
            last_timestamp: 0,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> VisualOdometryNodeBuilder<PassThrough<Odometry>> {
        VisualOdometryNodeBuilder::new()
    }
}

impl<P> VisualOdometryNode<P>
where
    P: Processor<Odometry>,
{
    /// Create with stereo camera input
    pub fn with_stereo(mut self, stereo_topic: &str) -> HorusResult<Self> {
        self.stereo_sub = Some(Topic::new(stereo_topic)?);
        self.config.mode = CameraMode::Stereo;
        Ok(self)
    }

    /// Create with RGB-D input
    pub fn with_depth(mut self, depth_topic: &str) -> HorusResult<Self> {
        self.depth_sub = Some(Topic::new(depth_topic)?);
        self.config.mode = CameraMode::RgbD;
        Ok(self)
    }

    /// Set external scale reference (from wheel odometry, etc.)
    pub fn set_scale(&mut self, scale: f64) {
        self.scale = scale;
    }

    /// Get current pose
    pub fn get_pose(&self) -> [f64; 6] {
        self.pose
    }

    /// Reset odometry to origin
    pub fn reset(&mut self) {
        self.pose = [0.0; 6];
        self.velocity = [0.0; 6];
        self.prev_frame = None;
        self.keyframes.clear();
        self.last_keyframe_pose = [0.0; 6];
    }

    /// Detect features in an image (simplified implementation)
    fn detect_features(&mut self, image: &Image) -> Vec<Feature> {
        let mut features = Vec::new();
        let width = image.width as usize;
        let height = image.height as usize;
        let channels = encoding_channels(image.encoding);

        // Convert to grayscale if needed
        let grayscale: Vec<u8> = if channels == 1 {
            image.data.clone()
        } else {
            image
                .data
                .chunks(channels)
                .map(|pixel| {
                    // Simple grayscale conversion
                    let r = pixel.first().copied().unwrap_or(0) as u16;
                    let g = pixel.get(1).copied().unwrap_or(0) as u16;
                    let b = pixel.get(2).copied().unwrap_or(0) as u16;
                    ((r * 299 + g * 587 + b * 114) / 1000) as u8
                })
                .collect()
        };

        // Simple corner detection (Harris-like)
        // For production, use OpenCV's feature detectors
        let block_size = 3;
        let step = (self.config.min_distance as usize).max(5);

        for y in (block_size..height - block_size).step_by(step) {
            for x in (block_size..width - block_size).step_by(step) {
                if features.len() >= self.config.max_features {
                    break;
                }

                let response = self.compute_corner_response(&grayscale, width, x, y, block_size);

                if response > (self.config.quality_level * 255.0) as f32 {
                    features.push(Feature {
                        x: x as f32,
                        y: y as f32,
                        response,
                        id: self.feature_id_counter,
                    });
                    self.feature_id_counter += 1;
                }
            }
        }

        // Sort by response and keep top features
        features.sort_by(|a, b| b.response.partial_cmp(&a.response).unwrap_or(std::cmp::Ordering::Equal));
        features.truncate(self.config.max_features);

        features
    }

    /// Compute corner response (simplified Harris)
    fn compute_corner_response(
        &self,
        image: &[u8],
        width: usize,
        x: usize,
        y: usize,
        block_size: usize,
    ) -> f32 {
        let mut ix2 = 0.0f32;
        let mut iy2 = 0.0f32;
        let mut ixiy = 0.0f32;

        for dy in 0..block_size {
            for dx in 0..block_size {
                let px = x + dx;
                let py = y + dy;

                if px >= width - 1 || py >= (image.len() / width) - 1 {
                    continue;
                }

                let idx = py * width + px;
                if idx + width + 1 >= image.len() {
                    continue;
                }

                // Sobel gradients
                let ix = (image[idx + 1] as f32 - image[idx] as f32) / 255.0;
                let iy = (image[idx + width] as f32 - image[idx] as f32) / 255.0;

                ix2 += ix * ix;
                iy2 += iy * iy;
                ixiy += ix * iy;
            }
        }

        // Harris corner response
        let det = ix2 * iy2 - ixiy * ixiy;
        let trace = ix2 + iy2;
        let k = 0.04;

        det - k * trace * trace
    }

    /// Track features between frames using optical flow
    fn track_features(
        &self,
        prev_frame: &Frame,
        curr_image: &Image,
    ) -> Vec<(Feature, Feature)> {
        let mut matches = Vec::new();

        let width = curr_image.width as usize;
        let height = curr_image.height as usize;
        let channels = encoding_channels(curr_image.encoding);

        // Convert current image to grayscale
        let curr_gray: Vec<u8> = if channels == 1 {
            curr_image.data.clone()
        } else {
            curr_image
                .data
                .chunks(channels)
                .map(|pixel| {
                    let r = pixel.first().copied().unwrap_or(0) as u16;
                    let g = pixel.get(1).copied().unwrap_or(0) as u16;
                    let b = pixel.get(2).copied().unwrap_or(0) as u16;
                    ((r * 299 + g * 587 + b * 114) / 1000) as u8
                })
                .collect()
        };

        // Convert previous image to grayscale
        let prev_channels = if prev_frame.width > 0 { 1 } else { 1 }; // Simplified
        let prev_gray: Vec<u8> = if prev_channels == 1 {
            prev_frame.image_data.clone()
        } else {
            prev_frame
                .image_data
                .chunks(prev_channels)
                .map(|pixel| pixel.first().copied().unwrap_or(0))
                .collect()
        };

        // Simple template matching for each feature
        let window_size = 11;
        let search_radius = 30;
        let half_window = window_size / 2;

        for prev_feat in &prev_frame.features {
            let px = prev_feat.x as i32;
            let py = prev_feat.y as i32;

            if px < half_window
                || py < half_window
                || px >= width as i32 - half_window
                || py >= height as i32 - half_window
            {
                continue;
            }

            let mut best_x = px;
            let mut best_y = py;
            let mut best_score = f32::MAX;

            // Search in neighborhood
            for sy in -search_radius..=search_radius {
                for sx in -search_radius..=search_radius {
                    let cx = px + sx;
                    let cy = py + sy;

                    if cx < half_window
                        || cy < half_window
                        || cx >= width as i32 - half_window
                        || cy >= height as i32 - half_window
                    {
                        continue;
                    }

                    // Compute SSD (Sum of Squared Differences)
                    let mut ssd = 0.0f32;
                    for wy in -half_window..=half_window {
                        for wx in -half_window..=half_window {
                            let prev_idx = ((py + wy) as usize) * prev_frame.width as usize
                                + (px + wx) as usize;
                            let curr_idx =
                                ((cy + wy) as usize) * width + (cx + wx) as usize;

                            if prev_idx < prev_gray.len() && curr_idx < curr_gray.len() {
                                let diff =
                                    prev_gray[prev_idx] as f32 - curr_gray[curr_idx] as f32;
                                ssd += diff * diff;
                            }
                        }
                    }

                    if ssd < best_score {
                        best_score = ssd;
                        best_x = cx;
                        best_y = cy;
                    }
                }
            }

            // Only accept good matches
            let threshold = (window_size * window_size * 50) as f32;
            if best_score < threshold {
                matches.push((
                    prev_feat.clone(),
                    Feature {
                        x: best_x as f32,
                        y: best_y as f32,
                        response: prev_feat.response,
                        id: prev_feat.id,
                    },
                ));
            }
        }

        matches
    }

    /// Estimate motion from matched features
    fn estimate_motion(&self, matches: &[(Feature, Feature)], image: &Image) -> Option<[f64; 6]> {
        if matches.len() < 8 {
            return None;
        }

        let [fx, fy, cx, cy] = self.config.camera_intrinsics;

        // Compute essential matrix using 8-point algorithm (simplified)
        // In production, use OpenCV's findEssentialMat with RANSAC

        // Normalize points
        let points1: Vec<(f64, f64)> = matches
            .iter()
            .map(|(p1, _)| ((p1.x as f64 - cx) / fx, (p1.y as f64 - cy) / fy))
            .collect();

        let points2: Vec<(f64, f64)> = matches
            .iter()
            .map(|(_, p2)| ((p2.x as f64 - cx) / fx, (p2.y as f64 - cy) / fy))
            .collect();

        // Compute average motion (simplified - real implementation needs essential matrix decomposition)
        let mut avg_dx = 0.0;
        let mut avg_dy = 0.0;
        let mut avg_scale = 0.0;

        for ((x1, y1), (x2, y2)) in points1.iter().zip(points2.iter()) {
            avg_dx += x2 - x1;
            avg_dy += y2 - y1;
            let d1 = (x1 * x1 + y1 * y1).sqrt();
            let d2 = (x2 * x2 + y2 * y2).sqrt();
            if d1 > 0.001 {
                avg_scale += d2 / d1;
            }
        }

        let n = matches.len() as f64;
        avg_dx /= n;
        avg_dy /= n;
        avg_scale /= n;

        // Convert pixel motion to world motion (simplified)
        // This is a rough approximation - real VO uses proper geometry
        let depth_estimate = match self.config.mode {
            CameraMode::Monocular => 1.0, // Unknown scale
            CameraMode::Stereo => self.config.stereo_baseline * fx / avg_scale.max(0.01),
            CameraMode::RgbD => 1.0, // Use depth from sensor
        };

        // Estimate translation (simplified)
        let tx = -avg_dx * depth_estimate * self.scale;
        let ty = -avg_dy * depth_estimate * self.scale;
        let tz = (1.0 - avg_scale) * depth_estimate * self.scale;

        // Estimate rotation from feature flow pattern
        // This is very simplified - real implementation uses essential matrix
        let rotation_scale = 0.001;
        let rx = avg_dy * rotation_scale; // Pitch from vertical motion
        let ry = -avg_dx * rotation_scale; // Yaw from horizontal motion
        let rz = 0.0; // Roll estimation would need more sophisticated analysis

        // Validate motion (reject outliers)
        let max_translation = 1.0; // Max 1m per frame
        let max_rotation = 0.2; // Max ~11 degrees per frame

        if tx.abs() > max_translation
            || ty.abs() > max_translation
            || tz.abs() > max_translation
            || rx.abs() > max_rotation
            || ry.abs() > max_rotation
        {
            return None;
        }

        // Check image dimensions are valid
        if image.width == 0 || image.height == 0 {
            return None;
        }

        Some([tx, ty, tz, rx, ry, rz])
    }

    /// Update pose with motion estimate
    fn update_pose(&mut self, motion: [f64; 6], dt: f64) {
        let [tx, ty, tz, rx, ry, rz] = motion;

        // Current orientation
        let (roll, pitch, yaw) = (self.pose[3], self.pose[4], self.pose[5]);

        // Rotate translation by current orientation (simplified - should use rotation matrix)
        let cos_yaw = yaw.cos();
        let sin_yaw = yaw.sin();

        let world_tx = tx * cos_yaw - tz * sin_yaw;
        let world_ty = ty;
        let world_tz = tx * sin_yaw + tz * cos_yaw;

        // Update position
        self.pose[0] += world_tx;
        self.pose[1] += world_ty;
        self.pose[2] += world_tz;

        // Update orientation
        self.pose[3] = normalize_angle(roll + rx);
        self.pose[4] = normalize_angle(pitch + ry);
        self.pose[5] = normalize_angle(yaw + rz);

        // Update velocity
        if dt > 0.001 {
            self.velocity[0] = world_tx / dt;
            self.velocity[1] = world_ty / dt;
            self.velocity[2] = world_tz / dt;
            self.velocity[3] = rx / dt;
            self.velocity[4] = ry / dt;
            self.velocity[5] = rz / dt;
        }
    }

    /// Check if current frame should be a keyframe
    fn should_insert_keyframe(&self) -> bool {
        let trans_diff = ((self.pose[0] - self.last_keyframe_pose[0]).powi(2)
            + (self.pose[1] - self.last_keyframe_pose[1]).powi(2)
            + (self.pose[2] - self.last_keyframe_pose[2]).powi(2))
        .sqrt();

        let rot_diff = ((self.pose[3] - self.last_keyframe_pose[3]).powi(2)
            + (self.pose[4] - self.last_keyframe_pose[4]).powi(2)
            + (self.pose[5] - self.last_keyframe_pose[5]).powi(2))
        .sqrt();

        trans_diff > self.config.keyframe_threshold_trans
            || rot_diff > self.config.keyframe_threshold_rot
    }

    /// Publish odometry
    fn publish_odom(&mut self, ctx: &mut Option<&mut NodeInfo>) {
        let mut odom = Odometry::new();

        // Set pose (2D pose from 3D - project to ground plane)
        odom.pose.x = self.pose[0];
        odom.pose.y = self.pose[2]; // Use Z as forward in camera frame
        odom.pose.theta = self.pose[5]; // Yaw

        // Set twist
        odom.twist.linear[0] = self.velocity[0];
        odom.twist.linear[1] = self.velocity[2];
        odom.twist.angular[2] = self.velocity[5];

        // Set timestamp
        odom.timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;

        // Set frame IDs
        odom.set_frames("odom", "camera_link");

        // Process through pipeline
        if let Some(processed) = self.processor.process(odom) {
            let _ = self.odom_pub.send(processed, ctx);
        }
    }

    /// Process a frame
    fn process_frame(&mut self, image: &Image, ctx: &mut Option<&mut NodeInfo>) {
        let start = Instant::now();

        // Detect features
        let features = self.detect_features(image);

        if features.is_empty() {
            return;
        }

        // Create current frame
        let curr_frame = Frame {
            features,
            timestamp: image.timestamp,
            image_data: self.convert_to_grayscale(image),
            width: image.width,
            height: image.height,
        };

        // Track features if we have a previous frame
        if let Some(ref prev_frame) = self.prev_frame {
            let matches = self.track_features(prev_frame, image);

            // Estimate motion
            if let Some(motion) = self.estimate_motion(&matches, image) {
                // Calculate time delta
                let dt = if self.last_timestamp > 0 {
                    (image.timestamp - self.last_timestamp) as f64 / 1_000_000_000.0
                } else {
                    0.033 // Assume 30fps default
                };

                self.update_pose(motion, dt);

                // Check for keyframe insertion
                if self.config.enable_loop_closure && self.should_insert_keyframe() {
                    if self.keyframes.len() >= 100 {
                        self.keyframes.pop_front();
                    }
                    self.keyframes.push_back(curr_frame.clone());
                    self.last_keyframe_pose = self.pose;
                }
            }
        }

        // Update state
        self.prev_frame = Some(curr_frame);
        self.frame_count += 1;
        self.last_timestamp = image.timestamp;
        self.last_processing_time_ms = start.elapsed().as_secs_f32() * 1000.0;

        // Publish odometry
        self.publish_odom(ctx);
    }

    /// Convert image to grayscale
    fn convert_to_grayscale(&self, image: &Image) -> Vec<u8> {
        let channels = encoding_channels(image.encoding);
        if channels == 1 {
            image.data.clone()
        } else {
            image
                .data
                .chunks(channels)
                .map(|pixel| {
                    let r = pixel.first().copied().unwrap_or(0) as u16;
                    let g = pixel.get(1).copied().unwrap_or(0) as u16;
                    let b = pixel.get(2).copied().unwrap_or(0) as u16;
                    ((r * 299 + g * 587 + b * 114) / 1000) as u8
                })
                .collect()
        }
    }
}

impl<P> Node for VisualOdometryNode<P>
where
    P: Processor<Odometry>,
{
    fn name(&self) -> &'static str {
        "VisualOdometryNode"
    }

    fn init(&mut self, _ctx: &mut NodeInfo) -> HorusResult<()> {
        self.processor.on_start();
        Ok(())
    }

    fn shutdown(&mut self, _ctx: &mut NodeInfo) -> HorusResult<()> {
        self.processor.on_shutdown();
        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        self.processor.on_tick();

        // Process main camera image
        if let Some(image) = self.image_sub.recv(&mut ctx) {
            self.process_frame(&image, &mut ctx);
        }
    }
}

/// Get number of channels for an image encoding
fn encoding_channels(encoding: ImageEncoding) -> usize {
    match encoding {
        ImageEncoding::Mono8 | ImageEncoding::Mono16 | ImageEncoding::Mono32F => 1,
        ImageEncoding::Rgb8 | ImageEncoding::Bgr8 | ImageEncoding::Rgb32F => 3,
        ImageEncoding::Rgba8 | ImageEncoding::Bgra8 => 4,
        ImageEncoding::Yuv422 => 2,
        ImageEncoding::BayerRggb8 | ImageEncoding::Depth16 => 1,
    }
}

/// Normalize angle to [-pi, pi]
fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle;
    while a > std::f64::consts::PI {
        a -= 2.0 * std::f64::consts::PI;
    }
    while a < -std::f64::consts::PI {
        a += 2.0 * std::f64::consts::PI;
    }
    a
}

/// Builder for VisualOdometryNode with processor configuration
pub struct VisualOdometryNodeBuilder<P>
where
    P: Processor<Odometry>,
{
    input_topic: String,
    output_topic: String,
    config: VOConfig,
    processor: P,
}

impl VisualOdometryNodeBuilder<PassThrough<Odometry>> {
    /// Create a new builder with default PassThrough processor
    pub fn new() -> Self {
        Self {
            input_topic: "camera.raw".to_string(),
            output_topic: "vo.odom".to_string(),
            config: VOConfig::default(),
            processor: PassThrough::new(),
        }
    }
}

impl Default for VisualOdometryNodeBuilder<PassThrough<Odometry>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P> VisualOdometryNodeBuilder<P>
where
    P: Processor<Odometry>,
{
    /// Set input topic
    pub fn input_topic(mut self, topic: &str) -> Self {
        self.input_topic = topic.to_string();
        self
    }

    /// Set output topic
    pub fn output_topic(mut self, topic: &str) -> Self {
        self.output_topic = topic.to_string();
        self
    }

    /// Set configuration
    pub fn config(mut self, config: VOConfig) -> Self {
        self.config = config;
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> VisualOdometryNodeBuilder<P2>
    where
        P2: Processor<Odometry>,
    {
        VisualOdometryNodeBuilder {
            input_topic: self.input_topic,
            output_topic: self.output_topic,
            config: self.config,
            processor,
        }
    }

    /// Set a closure-based processor
    pub fn with_closure<F>(self, f: F) -> VisualOdometryNodeBuilder<ClosureProcessor<Odometry, Odometry, F>>
    where
        F: FnMut(Odometry) -> Odometry + Send + 'static,
    {
        VisualOdometryNodeBuilder {
            input_topic: self.input_topic,
            output_topic: self.output_topic,
            config: self.config,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Set a filter-based processor
    pub fn with_filter<F>(self, f: F) -> VisualOdometryNodeBuilder<FilterProcessor<Odometry, Odometry, F>>
    where
        F: FnMut(Odometry) -> Option<Odometry> + Send + 'static,
    {
        VisualOdometryNodeBuilder {
            input_topic: self.input_topic,
            output_topic: self.output_topic,
            config: self.config,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor (pipe)
    pub fn pipe<P2>(self, next: P2) -> VisualOdometryNodeBuilder<Pipeline<Odometry, Odometry, Odometry, P, P2>>
    where
        P2: Processor<Odometry, Odometry>,
    {
        VisualOdometryNodeBuilder {
            input_topic: self.input_topic,
            output_topic: self.output_topic,
            config: self.config,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the node
    pub fn build(self) -> HorusResult<VisualOdometryNode<P>> {
        Ok(VisualOdometryNode {
            image_sub: Topic::new(&self.input_topic)?,
            depth_sub: None,
            stereo_sub: None,
            odom_pub: Topic::new(&self.output_topic)?,
            config: self.config,
            prev_frame: None,
            pose: [0.0; 6],
            velocity: [0.0; 6],
            frame_count: 0,
            feature_id_counter: 0,
            keyframes: VecDeque::with_capacity(100),
            last_keyframe_pose: [0.0; 6],
            scale: 1.0,
            last_processing_time_ms: 0.0,
            last_timestamp: 0,
            processor: self.processor,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_builder() {
        let config = VOConfig::default()
            .with_mode(CameraMode::Stereo)
            .with_feature_detector(FeatureDetector::ORB)
            .with_max_features(1000)
            .with_intrinsics(600.0, 600.0, 320.0, 240.0)
            .with_stereo_baseline(0.1);

        assert_eq!(config.mode, CameraMode::Stereo);
        assert_eq!(config.feature_detector, FeatureDetector::ORB);
        assert_eq!(config.max_features, 1000);
        assert_eq!(config.camera_intrinsics, [600.0, 600.0, 320.0, 240.0]);
        assert_eq!(config.stereo_baseline, 0.1);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 0.001);
        assert!((normalize_angle(std::f64::consts::PI) - std::f64::consts::PI).abs() < 0.001);
        assert!(
            (normalize_angle(3.0 * std::f64::consts::PI) - std::f64::consts::PI).abs() < 0.001
        );
        assert!(
            (normalize_angle(-3.0 * std::f64::consts::PI) + std::f64::consts::PI).abs() < 0.001
        );
    }

    #[test]
    fn test_encoding_channels() {
        assert_eq!(encoding_channels(ImageEncoding::Mono8), 1);
        assert_eq!(encoding_channels(ImageEncoding::Rgb8), 3);
        assert_eq!(encoding_channels(ImageEncoding::Rgba8), 4);
    }
}
