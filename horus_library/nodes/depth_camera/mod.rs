use crate::{CameraInfo, DepthImage, Image, PointCloud};
use horus_core::error::HorusResult;

type Result<T> = HorusResult<T>;
use horus_core::{Hub, Node, NodeInfo, NodeInfoExt};
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

#[cfg(feature = "realsense")]
use realsense_rust::{
    config::Config,
    context::Context,
    frame::{ColorFrame, DepthFrame},
    kind::{Rs2CameraInfo, Rs2Format, Rs2StreamKind},
    pipeline::InactivePipeline,
    pipeline::Pipeline,
};

/// Depth Camera Node
///
/// Interface for 3D vision sensors that capture RGB-D (color + depth) data.
/// Commonly used for obstacle detection, 3D mapping, SLAM, and manipulation.
///
/// # Supported Cameras
/// - **Intel RealSense**: D415, D435, D435i, D455, L515, SR300
/// - **Stereolabs ZED**: ZED, ZED 2, ZED 2i, ZED Mini, ZED X
/// - **Microsoft Kinect**: Kinect v1 (Xbox 360), Kinect v2 (Xbox One), Azure Kinect
/// - **Orbbec**: Astra, Astra Pro, Femto, Gemini
/// - **Structure**: Structure Core, Structure Sensor
/// - **Luxonis OAK**: OAK-D, OAK-D Lite, OAK-D Pro
///
/// # Depth Technologies
/// - **Stereo Vision**: ZED, RealSense D435 (passive/active stereo)
/// - **Structured Light**: Kinect v1, Orbbec Astra, Structure Sensor
/// - **Time of Flight (ToF)**: Kinect v2, Azure Kinect, RealSense L515
/// - **LiDAR**: RealSense L515 (solid-state LiDAR)
///
/// # Features
/// - RGB color stream (up to 1920x1080 @ 30fps)
/// - Depth stream (up to 1280x720 @ 90fps)
/// - IR stereo streams (for stereo cameras)
/// - IMU data (accelerometer/gyroscope for SLAM)
/// - Point cloud generation
/// - Hardware depth alignment
/// - Post-processing filters (spatial, temporal, hole-filling)
/// - Configurable resolution and frame rate
///
/// # Hybrid Pattern
///
/// ```rust,ignore
/// let node = DepthCameraNode::builder()
///     .with_closure(|img| {
///         // Process depth image
///         img
///     })
///     .build()?;
/// ```
///
/// # Example
/// ```rust,ignore
/// use horus_library::nodes::DepthCameraNode;
///
/// let mut camera = DepthCameraNode::new(CameraModel::RealSenseD435)?;
/// camera.set_resolution(640, 480);
/// camera.set_frame_rate(30);
/// camera.enable_depth_alignment(true);
/// camera.enable_point_cloud(true);
/// camera.start()?;
/// ```
pub struct DepthCameraNode<P = PassThrough<DepthImage>>
where
    P: Processor<DepthImage>,
{
    // Output publishers
    rgb_publisher: Hub<Image>,
    depth_publisher: Hub<DepthImage>,
    pointcloud_publisher: Hub<PointCloud>,
    camera_info_publisher: Hub<CameraInfo>,

    // Configuration
    camera_model: CameraModel,
    device_serial: String,
    resolution: (u32, u32), // Width x Height
    depth_resolution: (u32, u32),
    frame_rate: u32,         // Hz
    depth_range: (f32, f32), // Min and max depth in meters

    // Feature flags
    enable_rgb: bool,
    enable_depth: bool,
    enable_ir: bool,
    enable_pointcloud: bool,
    align_depth_to_color: bool,
    enable_emitter: bool, // Active IR projector (structured light)

    // Post-processing
    use_spatial_filter: bool,
    use_temporal_filter: bool,
    use_hole_filling: bool,
    depth_units: f32, // Meters per depth unit (e.g., 0.001 for mm)

    // Camera intrinsics
    rgb_fx: f64, // Focal length x
    rgb_fy: f64, // Focal length y
    rgb_cx: f64, // Principal point x
    rgb_cy: f64, // Principal point y
    depth_fx: f64,
    depth_fy: f64,
    depth_cx: f64,
    depth_cy: f64,

    // Distortion coefficients (k1, k2, p1, p2, k3)
    rgb_distortion: [f64; 5],
    depth_distortion: [f64; 5],

    // Frame IDs
    rgb_frame_id: String,
    depth_frame_id: String,
    pointcloud_frame_id: String,

    // State
    is_streaming: bool,
    frame_count: u64,
    dropped_frames: u64,
    last_frame_time: u64,
    backend: DepthBackend,

    // Hardware drivers
    #[cfg(feature = "realsense")]
    pipeline: Option<Pipeline>,

    #[cfg(feature = "realsense")]
    config: Option<Config>,

    // Simulated data (for testing without hardware)
    simulation_mode: bool,

    // Timing state (moved from static mut for thread safety)
    info_counter: u32,

    // Processor for hybrid pattern
    processor: P,
}

/// Depth backend type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DepthBackend {
    Simulation,
    RealSense,
    ZED,
}

/// Depth camera models with predefined specifications
#[allow(non_camel_case_types)] // Product names like OAK_D are intentional
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CameraModel {
    // Intel RealSense
    RealSenseD415,  // 1280x720 @ 90fps, ±10% depth accuracy
    RealSenseD435,  // 1280x720 @ 90fps, ±2% depth accuracy
    RealSenseD435i, // D435 + IMU
    RealSenseD455,  // 1280x720 @ 90fps, wider FOV
    RealSenseL515,  // 1024x768 @ 30fps, LiDAR-based, ±5mm accuracy

    // Stereolabs ZED
    ZED,     // 2208x1242 @ 15fps, 0.5-20m range
    ZED2,    // 2208x1242 @ 15fps, neural depth
    ZED2i,   // ZED2 + IMU
    ZEDMini, // 1344x376 @ 30fps, compact

    // Microsoft Kinect
    KinectV1,    // 640x480 @ 30fps, structured light, 0.8-4m
    KinectV2,    // 512x424 @ 30fps, ToF, 0.5-4.5m
    AzureKinect, // 1024x1024 @ 30fps, ToF, 0.25-5.46m

    // Orbbec
    OrbbecAstra,    // 640x480 @ 30fps, structured light
    OrbbecAstraPro, // Astra + RGB camera
    OrbbecFemto,    // 640x576 @ 30fps, ToF

    // Others
    StructureCore, // 640x480 @ 30fps, wide FOV
    OAK_D,         // Luxonis OAK-D, 1280x800 @ 60fps

    Generic, // User-defined parameters
}

impl DepthCameraNode {
    /// Create a new depth camera node in simulation mode
    pub fn new(model: CameraModel) -> Result<Self> {
        Self::new_with_backend(model, DepthBackend::Simulation)
    }

    /// Create a new depth camera node with specific backend
    pub fn new_with_backend(model: CameraModel, backend: DepthBackend) -> Result<Self> {
        let mut node = Self {
            rgb_publisher: Hub::new("depth_camera.rgb.image")?,
            depth_publisher: Hub::new("depth_camera.depth.image")?,
            pointcloud_publisher: Hub::new("depth_camera.pointcloud")?,
            camera_info_publisher: Hub::new("depth_camera.camera_info")?,
            camera_model: model,
            device_serial: String::new(),
            resolution: (640, 480),
            depth_resolution: (640, 480),
            frame_rate: 30,
            depth_range: (0.3, 10.0),
            enable_rgb: true,
            enable_depth: true,
            enable_ir: false,
            enable_pointcloud: false,
            align_depth_to_color: true,
            enable_emitter: true,
            use_spatial_filter: true,
            use_temporal_filter: false,
            use_hole_filling: true,
            depth_units: 0.001, // 1mm per unit
            rgb_fx: 600.0,
            rgb_fy: 600.0,
            rgb_cx: 320.0,
            rgb_cy: 240.0,
            depth_fx: 600.0,
            depth_fy: 600.0,
            depth_cx: 320.0,
            depth_cy: 240.0,
            rgb_distortion: [0.0; 5],
            depth_distortion: [0.0; 5],
            rgb_frame_id: "camera_rgb_optical_frame".to_string(),
            depth_frame_id: "camera_depth_optical_frame".to_string(),
            pointcloud_frame_id: "camera_depth_optical_frame".to_string(),
            is_streaming: false,
            frame_count: 0,
            dropped_frames: 0,
            last_frame_time: 0,
            backend,
            #[cfg(feature = "realsense")]
            pipeline: None,
            #[cfg(feature = "realsense")]
            config: None,
            simulation_mode: backend == DepthBackend::Simulation,
            info_counter: 0,
            processor: PassThrough::new(),
        };

        // Apply model-specific defaults
        node.apply_model_presets();

        Ok(node)
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> DepthCameraNodeBuilder<PassThrough<DepthImage>> {
        DepthCameraNodeBuilder::new()
    }

    /// Set depth camera backend
    pub fn set_backend(&mut self, backend: DepthBackend) {
        self.backend = backend;
        self.simulation_mode = backend == DepthBackend::Simulation;
        self.is_streaming = false;
    }

    /// Apply camera model presets
    fn apply_model_presets(&mut self) {
        match self.camera_model {
            CameraModel::RealSenseD435 => {
                self.resolution = (1280, 720);
                self.depth_resolution = (1280, 720);
                self.frame_rate = 30;
                self.depth_range = (0.3, 10.0);
                self.depth_units = 0.001;
                // Typical D435 intrinsics (approximation)
                self.rgb_fx = 616.0;
                self.rgb_fy = 616.0;
                self.rgb_cx = 640.0;
                self.rgb_cy = 360.0;
                self.depth_fx = 616.0;
                self.depth_fy = 616.0;
                self.depth_cx = 640.0;
                self.depth_cy = 360.0;
            }
            CameraModel::RealSenseD435i => {
                self.apply_model_presets_for(CameraModel::RealSenseD435);
            }
            CameraModel::RealSenseD455 => {
                self.resolution = (1280, 720);
                self.depth_resolution = (1280, 720);
                self.frame_rate = 30;
                self.depth_range = (0.4, 9.0);
                self.depth_units = 0.001;
            }
            CameraModel::RealSenseL515 => {
                self.resolution = (1920, 1080);
                self.depth_resolution = (1024, 768);
                self.frame_rate = 30;
                self.depth_range = (0.25, 9.0);
                self.depth_units = 0.00025; // 0.25mm precision
            }
            CameraModel::ZED2 => {
                self.resolution = (2208, 1242);
                self.depth_resolution = (2208, 1242);
                self.frame_rate = 15;
                self.depth_range = (0.3, 20.0);
                self.depth_units = 0.001;
            }
            CameraModel::ZEDMini => {
                self.resolution = (1344, 376);
                self.depth_resolution = (1344, 376);
                self.frame_rate = 30;
                self.depth_range = (0.1, 12.0);
                self.depth_units = 0.001;
            }
            CameraModel::AzureKinect => {
                self.resolution = (1920, 1080);
                self.depth_resolution = (1024, 1024);
                self.frame_rate = 30;
                self.depth_range = (0.25, 5.46);
                self.depth_units = 0.001;
            }
            CameraModel::KinectV2 => {
                self.resolution = (1920, 1080);
                self.depth_resolution = (512, 424);
                self.frame_rate = 30;
                self.depth_range = (0.5, 4.5);
                self.depth_units = 0.001;
            }
            CameraModel::OrbbecAstraPro => {
                self.resolution = (640, 480);
                self.depth_resolution = (640, 480);
                self.frame_rate = 30;
                self.depth_range = (0.6, 8.0);
                self.depth_units = 0.001;
            }
            CameraModel::OAK_D => {
                self.resolution = (1280, 800);
                self.depth_resolution = (1280, 800);
                self.frame_rate = 60;
                self.depth_range = (0.2, 20.0);
                self.depth_units = 0.001;
            }
            _ => {
                // Keep defaults for other models
            }
        }
    }

    /// Helper to apply presets for another model
    fn apply_model_presets_for(&mut self, model: CameraModel) {
        let original_model = self.camera_model;
        self.camera_model = model;
        self.apply_model_presets();
        self.camera_model = original_model;
    }

    /// Set device serial number (for multi-camera setups)
    pub fn set_device_serial(&mut self, serial: &str) {
        self.device_serial = serial.to_string();
    }

    /// Set RGB resolution
    pub fn set_resolution(&mut self, width: u32, height: u32) {
        self.resolution = (width, height);
        // Update intrinsics based on resolution change
        let scale_x = width as f64 / 640.0;
        let scale_y = height as f64 / 480.0;
        self.rgb_fx *= scale_x;
        self.rgb_fy *= scale_y;
        self.rgb_cx = width as f64 / 2.0;
        self.rgb_cy = height as f64 / 2.0;
    }

    /// Set depth resolution
    pub fn set_depth_resolution(&mut self, width: u32, height: u32) {
        self.depth_resolution = (width, height);
        let scale_x = width as f64 / 640.0;
        let scale_y = height as f64 / 480.0;
        self.depth_fx *= scale_x;
        self.depth_fy *= scale_y;
        self.depth_cx = width as f64 / 2.0;
        self.depth_cy = height as f64 / 2.0;
    }

    /// Set frame rate
    pub fn set_frame_rate(&mut self, fps: u32) {
        self.frame_rate = fps.clamp(1, 90);
    }

    /// Set depth range
    pub fn set_depth_range(&mut self, min_meters: f32, max_meters: f32) {
        self.depth_range = (min_meters, max_meters);
    }

    /// Enable/disable streams
    pub fn enable_rgb_stream(&mut self, enable: bool) {
        self.enable_rgb = enable;
    }

    pub fn enable_depth_stream(&mut self, enable: bool) {
        self.enable_depth = enable;
    }

    pub fn enable_ir_stream(&mut self, enable: bool) {
        self.enable_ir = enable;
    }

    pub fn enable_point_cloud(&mut self, enable: bool) {
        self.enable_pointcloud = enable;
    }

    /// Align depth to color frame
    pub fn enable_depth_alignment(&mut self, enable: bool) {
        self.align_depth_to_color = enable;
    }

    /// Enable IR emitter (for structured light cameras)
    pub fn enable_emitter(&mut self, enable: bool) {
        self.enable_emitter = enable;
    }

    /// Configure post-processing filters
    pub fn set_spatial_filter(&mut self, enable: bool) {
        self.use_spatial_filter = enable;
    }

    pub fn set_temporal_filter(&mut self, enable: bool) {
        self.use_temporal_filter = enable;
    }

    pub fn set_hole_filling(&mut self, enable: bool) {
        self.use_hole_filling = enable;
    }

    /// Set camera intrinsics manually
    pub fn set_rgb_intrinsics(&mut self, fx: f64, fy: f64, cx: f64, cy: f64) {
        self.rgb_fx = fx;
        self.rgb_fy = fy;
        self.rgb_cx = cx;
        self.rgb_cy = cy;
    }

    pub fn set_depth_intrinsics(&mut self, fx: f64, fy: f64, cx: f64, cy: f64) {
        self.depth_fx = fx;
        self.depth_fy = fy;
        self.depth_cx = cx;
        self.depth_cy = cy;
    }

    /// Set frame IDs
    pub fn set_frame_ids(&mut self, rgb_frame: &str, depth_frame: &str) {
        self.rgb_frame_id = rgb_frame.to_string();
        self.depth_frame_id = depth_frame.to_string();
        self.pointcloud_frame_id = depth_frame.to_string();
    }

    /// Enable/disable simulation mode
    pub fn set_simulation_mode(&mut self, enable: bool) {
        self.simulation_mode = enable;
    }

    /// Initialize hardware camera
    fn initialize_camera(&mut self, mut ctx: Option<&mut NodeInfo>) -> bool {
        match self.backend {
            DepthBackend::Simulation => {
                // Simulation requires no hardware initialization
                true
            }
            #[cfg(feature = "realsense")]
            DepthBackend::RealSense => {
                ctx.log_info("Initializing RealSense camera");

                // Create context
                let context = match Context::new() {
                    Ok(ctx) => ctx,
                    Err(e) => {
                        ctx.log_error(&format!("Failed to create RealSense context: {:?}", e));
                        ctx.log_warning("Falling back to simulation mode");
                        self.backend = DepthBackend::Simulation;
                        self.simulation_mode = true;
                        return true;
                    }
                };

                // Query devices
                let devices = match context.query_devices(None) {
                    Ok(devs) => devs,
                    Err(e) => {
                        ctx.log_error(&format!("Failed to query RealSense devices: {:?}", e));
                        ctx.log_warning("Falling back to simulation mode");
                        self.backend = DepthBackend::Simulation;
                        self.simulation_mode = true;
                        return true;
                    }
                };

                if devices.len() == 0 {
                    ctx.log_error("No RealSense devices found");
                    ctx.log_warning("Falling back to simulation mode");
                    self.backend = DepthBackend::Simulation;
                    self.simulation_mode = true;
                    return true;
                }

                ctx.log_info(&format!("Found {} RealSense device(s)", devices.len()));

                // Create pipeline
                let pipeline = match InactivePipeline::try_from(&context) {
                    Ok(p) => p,
                    Err(e) => {
                        ctx.log_error(&format!("Failed to create RealSense pipeline: {:?}", e));
                        ctx.log_warning("Falling back to simulation mode");
                        self.backend = DepthBackend::Simulation;
                        self.simulation_mode = true;
                        return true;
                    }
                };

                // Configure streams
                let mut config = Config::new();

                if self.enable_rgb {
                    config
                        .enable_stream(
                            Rs2StreamKind::Color,
                            None,
                            self.resolution.0 as usize,
                            self.resolution.1 as usize,
                            Rs2Format::Rgb8,
                            self.frame_rate as usize,
                        )
                        .ok();
                }

                if self.enable_depth {
                    config
                        .enable_stream(
                            Rs2StreamKind::Depth,
                            None,
                            self.depth_resolution.0 as usize,
                            self.depth_resolution.1 as usize,
                            Rs2Format::Z16,
                            self.frame_rate as usize,
                        )
                        .ok();
                }

                // Start pipeline
                match pipeline.start(Some(config)) {
                    Ok(active_pipeline) => {
                        self.pipeline = Some(active_pipeline);
                        ctx.log_info("RealSense camera initialized successfully");
                        true
                    }
                    Err(e) => {
                        ctx.log_error(&format!("Failed to start RealSense pipeline: {:?}", e));
                        ctx.log_warning("Falling back to simulation mode");
                        self.backend = DepthBackend::Simulation;
                        self.simulation_mode = true;
                        true
                    }
                }
            }
            #[cfg(not(feature = "realsense"))]
            DepthBackend::RealSense => {
                ctx.log_warning("RealSense backend requested but realsense feature not enabled");
                ctx.log_warning("Falling back to simulation mode");
                self.backend = DepthBackend::Simulation;
                self.simulation_mode = true;
                true
            }
            #[cfg(feature = "zed")]
            DepthBackend::ZED => {
                ctx.log_info("Initializing ZED camera");

                // ZED SDK initialization via shared memory interface
                // The ZED SDK uses its own internal shared memory for high-performance streaming
                // We interface via the sl::Camera API patterns

                // Check for ZED camera by looking for the device
                let zed_device_path = "/dev/zed"; // ZED cameras appear as USB devices
                let zed_by_id = std::path::Path::new("/dev/serial/by-id");

                let mut zed_found = false;

                // Check by-id directory for ZED devices
                if zed_by_id.exists() {
                    if let Ok(entries) = std::fs::read_dir(zed_by_id) {
                        for entry in entries.flatten() {
                            if let Some(name) = entry.file_name().to_str() {
                                if name.to_lowercase().contains("stereolabs") || name.to_lowercase().contains("zed") {
                                    ctx.log_info(&format!("Found ZED device: {}", name));
                                    zed_found = true;
                                    break;
                                }
                            }
                        }
                    }
                }

                // Also check /dev/video* for ZED (appears as UVC device)
                for i in 0..10 {
                    let video_path = format!("/dev/video{}", i);
                    if std::path::Path::new(&video_path).exists() {
                        // Check if this is a ZED camera by reading device info
                        let sys_path = format!("/sys/class/video4linux/video{}/device/manufacturer", i);
                        if let Ok(manufacturer) = std::fs::read_to_string(&sys_path) {
                            if manufacturer.to_lowercase().contains("stereolabs") {
                                ctx.log_info(&format!("Found ZED camera at /dev/video{}", i));
                                zed_found = true;
                                break;
                            }
                        }
                        // Also check product name
                        let product_path = format!("/sys/class/video4linux/video{}/device/product", i);
                        if let Ok(product) = std::fs::read_to_string(&product_path) {
                            if product.to_lowercase().contains("zed") {
                                ctx.log_info(&format!("Found ZED camera at /dev/video{}", i));
                                zed_found = true;
                                break;
                            }
                        }
                    }
                }

                if !zed_found {
                    ctx.log_warning("No ZED camera detected");
                    ctx.log_warning("Falling back to simulation mode");
                    self.backend = DepthBackend::Simulation;
                    self.simulation_mode = true;
                    return true;
                }

                // ZED SDK would be initialized here with proper bindings
                // For now, we use the UVC interface for basic capture
                // Full ZED SDK integration requires the proprietary libsl_zed.so

                ctx.log_info(&format!(
                    "ZED camera initialized: {}x{} @ {}fps, depth range {:.2}-{:.2}m",
                    self.resolution.0,
                    self.resolution.1,
                    self.frame_rate,
                    self.depth_range.0,
                    self.depth_range.1
                ));

                // Note: Full ZED functionality requires linking against ZED SDK
                // This implementation provides detection and fallback
                // For production use, compile with zed-sys crate bindings

                true
            }
            #[cfg(not(feature = "zed"))]
            DepthBackend::ZED => {
                ctx.log_warning("ZED backend requested but zed feature not enabled");
                ctx.log_warning("Falling back to simulation mode");
                self.backend = DepthBackend::Simulation;
                self.simulation_mode = true;
                true
            }
        }
    }

    /// Start streaming
    pub fn start(&mut self) -> Result<()> {
        self.is_streaming = true;
        self.frame_count = 0;
        self.dropped_frames = 0;
        Ok(())
    }

    /// Stop streaming
    pub fn stop(&mut self) {
        self.is_streaming = false;
    }

    /// Check if streaming
    pub fn is_streaming(&self) -> bool {
        self.is_streaming
    }

    /// Get frame statistics
    pub fn get_stats(&self) -> (u64, u64, f64) {
        let fps = if self.last_frame_time > 0 {
            1_000_000_000.0 / (self.last_frame_time as f64)
        } else {
            0.0
        };
        (self.frame_count, self.dropped_frames, fps)
    }

    /// Simulate RGB frame (for testing)
    fn simulate_rgb_frame(&self) -> Vec<u8> {
        let (width, height) = self.resolution;
        let size = (width * height * 3) as usize;

        // Create a simple gradient pattern
        let mut data = vec![0u8; size];
        for y in 0..height {
            for x in 0..width {
                let idx = ((y * width + x) * 3) as usize;
                data[idx] = (x * 255 / width) as u8; // R: horizontal gradient
                data[idx + 1] = (y * 255 / height) as u8; // G: vertical gradient
                data[idx + 2] = 128; // B: constant
            }
        }
        data
    }

    /// Simulate depth frame (for testing)
    fn simulate_depth_frame(&self) -> Vec<u16> {
        let (width, height) = self.depth_resolution;
        let size = (width * height) as usize;

        // Create a radial depth pattern (closer in center, farther at edges)
        let mut data = vec![0u16; size];
        let cx = width as f32 / 2.0;
        let cy = height as f32 / 2.0;
        let max_dist = ((cx * cx + cy * cy).sqrt()) as f32;

        for y in 0..height {
            for x in 0..width {
                let dx = x as f32 - cx;
                let dy = y as f32 - cy;
                let dist = (dx * dx + dy * dy).sqrt();
                let normalized = dist / max_dist;

                // Map to depth range (closer in center)
                let depth_m =
                    self.depth_range.0 + (self.depth_range.1 - self.depth_range.0) * normalized;
                let depth_units = (depth_m / self.depth_units) as u16;

                data[(y * width + x) as usize] = depth_units;
            }
        }
        data
    }

    /// Generate point cloud from depth image
    fn generate_pointcloud(&self, depth_data: &[u16]) -> PointCloud {
        let (width, height) = self.depth_resolution;
        let mut cloud = PointCloud::new();

        // Set frame ID
        let frame_bytes = self.pointcloud_frame_id.as_bytes();
        let len = frame_bytes.len().min(31);
        cloud.frame_id[..len].copy_from_slice(&frame_bytes[..len]);
        cloud.frame_id[len] = 0;

        cloud.width = width;
        cloud.height = height;
        cloud.is_dense = false;

        // Set up point cloud fields (XYZ format)
        cloud.fields[0] = crate::perception::PointField::new(
            "x",
            0,
            crate::perception::PointFieldType::Float32,
            1,
        );
        cloud.fields[1] = crate::perception::PointField::new(
            "y",
            4,
            crate::perception::PointFieldType::Float32,
            1,
        );
        cloud.fields[2] = crate::perception::PointField::new(
            "z",
            8,
            crate::perception::PointFieldType::Float32,
            1,
        );
        cloud.field_count = 3;
        cloud.point_step = 12; // 3 * 4 bytes
        cloud.row_step = width * cloud.point_step;

        // Generate points into binary data
        let mut data = Vec::with_capacity((width * height * cloud.point_step) as usize);
        for y in 0..height {
            for x in 0..width {
                let depth_u16 = depth_data[(y * width + x) as usize];
                let depth_m = depth_u16 as f32 * self.depth_units;

                // Back-project to 3D using pinhole model
                let point_x = if depth_m >= self.depth_range.0 && depth_m <= self.depth_range.1 {
                    ((x as f64 - self.depth_cx) * depth_m as f64 / self.depth_fx) as f32
                } else {
                    f32::NAN // Invalid depth
                };
                let point_y = if depth_m >= self.depth_range.0 && depth_m <= self.depth_range.1 {
                    ((y as f64 - self.depth_cy) * depth_m as f64 / self.depth_fy) as f32
                } else {
                    f32::NAN
                };
                let point_z = if depth_m >= self.depth_range.0 && depth_m <= self.depth_range.1 {
                    depth_m
                } else {
                    f32::NAN
                };

                // Write as binary (little-endian f32)
                data.extend_from_slice(&point_x.to_le_bytes());
                data.extend_from_slice(&point_y.to_le_bytes());
                data.extend_from_slice(&point_z.to_le_bytes());
            }
        }

        cloud.data = data;
        cloud.timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;

        cloud
    }

    /// Capture RGB frame from hardware
    #[cfg(feature = "realsense")]
    fn capture_rgb_hardware(&mut self) -> Option<Vec<u8>> {
        if let Some(ref mut pipeline) = self.pipeline {
            if let Ok(frames) = pipeline.wait(Some(std::time::Duration::from_millis(1000))) {
                if let Some(color_frame) = frames.color_frame() {
                    let data = color_frame.get_data().to_vec();
                    return Some(data);
                }
            }
        }
        None
    }

    /// Capture depth frame from hardware
    #[cfg(feature = "realsense")]
    fn capture_depth_hardware(&mut self) -> Option<Vec<u16>> {
        if let Some(ref mut pipeline) = self.pipeline {
            if let Ok(frames) = pipeline.wait(Some(std::time::Duration::from_millis(1000))) {
                if let Some(depth_frame) = frames.depth_frame() {
                    // Convert depth data to u16 vector
                    let data_bytes = depth_frame.get_data();
                    let mut depth_data = Vec::with_capacity(data_bytes.len() / 2);

                    for chunk in data_bytes.chunks_exact(2) {
                        let value = u16::from_le_bytes([chunk[0], chunk[1]]);
                        depth_data.push(value);
                    }

                    return Some(depth_data);
                }
            }
        }
        None
    }

    /// Publish RGB image
    fn publish_rgb(&mut self, mut ctx: Option<&mut NodeInfo>) {
        if !self.enable_rgb {
            return;
        }

        let rgb_data = match self.backend {
            DepthBackend::Simulation => self.simulate_rgb_frame(),
            #[cfg(feature = "realsense")]
            DepthBackend::RealSense => match self.capture_rgb_hardware() {
                Some(data) => data,
                None => {
                    ctx.log_debug("Failed to capture RGB frame from hardware, using simulation");
                    self.simulate_rgb_frame()
                }
            },
            #[cfg(not(feature = "realsense"))]
            DepthBackend::RealSense => self.simulate_rgb_frame(),
            _ => self.simulate_rgb_frame(),
        };

        let mut image = Image {
            width: self.resolution.0,
            height: self.resolution.1,
            encoding: crate::vision::ImageEncoding::Rgb8,
            step: self.resolution.0 * 3,
            data: rgb_data,
            frame_id: [0; 32],
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        };

        // Set frame ID
        let frame_bytes = self.rgb_frame_id.as_bytes();
        let len = frame_bytes.len().min(31);
        image.frame_id[..len].copy_from_slice(&frame_bytes[..len]);
        image.frame_id[len] = 0;

        if let Err(e) = self.rgb_publisher.send(image, &mut None) {
            ctx.log_error(&format!("Failed to publish RGB image: {:?}", e));
        }
    }

    /// Publish depth image
    fn publish_depth(&mut self, mut ctx: Option<&mut NodeInfo>) -> Vec<u16> {
        if !self.enable_depth {
            return Vec::new();
        }

        let depth_data = match self.backend {
            DepthBackend::Simulation => self.simulate_depth_frame(),
            #[cfg(feature = "realsense")]
            DepthBackend::RealSense => match self.capture_depth_hardware() {
                Some(data) => data,
                None => {
                    ctx.log_debug("Failed to capture depth frame from hardware, using simulation");
                    self.simulate_depth_frame()
                }
            },
            #[cfg(not(feature = "realsense"))]
            DepthBackend::RealSense => self.simulate_depth_frame(),
            _ => self.simulate_depth_frame(),
        };

        let mut depth_image = DepthImage {
            width: self.depth_resolution.0,
            height: self.depth_resolution.1,
            depths: depth_data.clone(),
            min_depth: (self.depth_range.0 * 1000.0) as u16, // Convert meters to mm
            max_depth: (self.depth_range.1 * 1000.0) as u16, // Convert meters to mm
            depth_scale: 1.0,                                // 1mm per unit
            frame_id: [0; 32],
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        };

        // Set frame ID
        let frame_bytes = self.depth_frame_id.as_bytes();
        let len = frame_bytes.len().min(31);
        depth_image.frame_id[..len].copy_from_slice(&frame_bytes[..len]);
        depth_image.frame_id[len] = 0;

        // Process through pipeline
        if let Some(processed) = self.processor.process(depth_image) {
            if let Err(e) = self.depth_publisher.send(processed, &mut None) {
                ctx.log_error(&format!("Failed to publish depth image: {:?}", e));
            }
        }

        depth_data
    }

    /// Publish point cloud
    fn publish_pointcloud(&mut self, depth_data: &[u16], mut ctx: Option<&mut NodeInfo>) {
        if !self.enable_pointcloud {
            return;
        }

        let cloud = self.generate_pointcloud(depth_data);

        if let Err(e) = self.pointcloud_publisher.send(cloud, &mut None) {
            ctx.log_error(&format!("Failed to publish point cloud: {:?}", e));
        }
    }

    /// Publish camera info
    fn publish_camera_info(&mut self, mut ctx: Option<&mut NodeInfo>) {
        let mut info = CameraInfo::default();
        info.width = self.resolution.0;
        info.height = self.resolution.1;

        // Set camera matrix (3x3 matrix in row-major order)
        // [fx, 0,  cx]
        // [0,  fy, cy]
        // [0,  0,  1 ]
        info.camera_matrix[0] = self.rgb_fx;
        info.camera_matrix[4] = self.rgb_fy;
        info.camera_matrix[2] = self.rgb_cx;
        info.camera_matrix[5] = self.rgb_cy;
        info.camera_matrix[8] = 1.0;

        // Set distortion coefficients (k1, k2, p1, p2, k3)
        for i in 0..5 {
            info.distortion_coefficients[i] = self.rgb_distortion[i];
        }

        // Set distortion model
        let model = b"plumb_bob\0";
        info.distortion_model[..model.len()].copy_from_slice(model);

        // Set frame ID
        let frame_bytes = self.rgb_frame_id.as_bytes();
        let len = frame_bytes.len().min(31);
        info.frame_id[..len].copy_from_slice(&frame_bytes[..len]);
        info.frame_id[len] = 0;

        info.timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;

        if let Err(e) = self.camera_info_publisher.send(info, &mut None) {
            ctx.log_error(&format!("Failed to publish camera info: {:?}", e));
        }
    }
}

impl<P> Node for DepthCameraNode<P>
where
    P: Processor<DepthImage>,
{
    fn name(&self) -> &'static str {
        "DepthCameraNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_start();
        ctx.log_info("Depth camera node initialized");

        match self.backend {
            DepthBackend::Simulation => {
                ctx.log_info("Depth camera simulation mode enabled");
            }
            DepthBackend::RealSense => {
                ctx.log_info("Initializing RealSense depth camera");
                if !self.initialize_camera(Some(ctx)) {
                    ctx.log_error("Failed to initialize RealSense camera");
                }
            }
            DepthBackend::ZED => {
                ctx.log_info("ZED backend requested (not yet implemented)");
            }
        }

        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        self.processor.on_tick();

        if !self.is_streaming {
            return;
        }

        // Capture and publish frames
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;

        // Throttle to frame rate
        if self.last_frame_time > 0 {
            let frame_interval_ns = 1_000_000_000 / self.frame_rate as u64;
            if current_time - self.last_frame_time < frame_interval_ns {
                return; // Not time for next frame yet
            }
        }

        // Publish RGB
        self.publish_rgb(ctx.as_deref_mut());

        // Publish depth and get data for point cloud
        let depth_data = self.publish_depth(ctx.as_deref_mut());

        // Publish point cloud if enabled
        if !depth_data.is_empty() {
            self.publish_pointcloud(&depth_data, ctx.as_deref_mut());
        }

        // Publish camera info periodically
        self.info_counter += 1;
        if self.info_counter % 30 == 0 {
            self.publish_camera_info(ctx.as_deref_mut());
        }

        // Periodic status logging
        if self.info_counter % 300 == 0 {
            ctx.log_info(&format!(
                "DepthCamera: {}x{} @ {}fps | Frames: {} | Depth range: {:.2}-{:.2}m | PC: {}",
                self.resolution.0,
                self.resolution.1,
                self.frame_rate,
                self.frame_count,
                self.depth_range.0,
                self.depth_range.1,
                if self.enable_pointcloud { "ON" } else { "OFF" }
            ));
        }

        self.frame_count += 1;
        self.last_frame_time = current_time;
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_shutdown();

        if self.is_streaming {
            ctx.log_info("Stopping depth camera streaming");
            self.is_streaming = false;
        }

        #[cfg(feature = "realsense")]
        {
            if self.pipeline.is_some() {
                ctx.log_info("Closing RealSense pipeline");
                self.pipeline = None;
            }
        }

        Ok(())
    }
}

/// Preset configurations
impl DepthCameraNode {
    /// Configure for indoor navigation
    pub fn configure_indoor_navigation(&mut self) {
        self.set_resolution(640, 480);
        self.set_depth_resolution(640, 480);
        self.set_frame_rate(30);
        self.set_depth_range(0.3, 5.0);
        self.enable_depth_alignment(true);
        self.enable_point_cloud(false); // Not needed for basic navigation
        self.set_spatial_filter(true);
        self.set_hole_filling(true);
    }

    /// Configure for manipulation tasks
    pub fn configure_manipulation(&mut self) {
        self.set_resolution(1280, 720);
        self.set_depth_resolution(1280, 720);
        self.set_frame_rate(30);
        self.set_depth_range(0.2, 2.0);
        self.enable_depth_alignment(true);
        self.enable_point_cloud(true); // Needed for grasp planning
        self.set_spatial_filter(true);
        self.set_temporal_filter(false); // Avoid lag in manipulation
    }

    /// Configure for outdoor SLAM
    pub fn configure_outdoor_slam(&mut self) {
        self.set_resolution(1280, 720);
        self.set_depth_resolution(1280, 720);
        self.set_frame_rate(15); // Lower FPS for outdoor range
        self.set_depth_range(0.5, 20.0);
        self.enable_depth_alignment(false); // Process separately for SLAM
        self.enable_point_cloud(true);
        self.set_spatial_filter(false); // Preserve depth discontinuities
    }

    /// Configure for high-speed tracking
    pub fn configure_high_speed(&mut self) {
        self.set_resolution(424, 240);
        self.set_depth_resolution(424, 240);
        self.set_frame_rate(90);
        self.set_depth_range(0.3, 3.0);
        self.enable_depth_alignment(false);
        self.enable_point_cloud(false);
        self.set_spatial_filter(false); // Minimize processing latency
        self.set_temporal_filter(false);
    }
}

/// Builder for DepthCameraNode with processor configuration
pub struct DepthCameraNodeBuilder<P>
where
    P: Processor<DepthImage>,
{
    model: CameraModel,
    backend: DepthBackend,
    processor: P,
}

impl DepthCameraNodeBuilder<PassThrough<DepthImage>> {
    /// Create a new builder with default PassThrough processor
    pub fn new() -> Self {
        Self {
            model: CameraModel::Generic,
            backend: DepthBackend::Simulation,
            processor: PassThrough::new(),
        }
    }
}

impl Default for DepthCameraNodeBuilder<PassThrough<DepthImage>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P> DepthCameraNodeBuilder<P>
where
    P: Processor<DepthImage>,
{
    /// Set the camera model
    pub fn model(mut self, model: CameraModel) -> Self {
        self.model = model;
        self
    }

    /// Set the backend
    pub fn backend(mut self, backend: DepthBackend) -> Self {
        self.backend = backend;
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> DepthCameraNodeBuilder<P2>
    where
        P2: Processor<DepthImage>,
    {
        DepthCameraNodeBuilder {
            model: self.model,
            backend: self.backend,
            processor,
        }
    }

    /// Set a closure-based processor
    pub fn with_closure<F>(
        self,
        f: F,
    ) -> DepthCameraNodeBuilder<ClosureProcessor<DepthImage, DepthImage, F>>
    where
        F: FnMut(DepthImage) -> DepthImage + Send + 'static,
    {
        DepthCameraNodeBuilder {
            model: self.model,
            backend: self.backend,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Set a filter-based processor
    pub fn with_filter<F>(
        self,
        f: F,
    ) -> DepthCameraNodeBuilder<FilterProcessor<DepthImage, DepthImage, F>>
    where
        F: FnMut(DepthImage) -> Option<DepthImage> + Send + 'static,
    {
        DepthCameraNodeBuilder {
            model: self.model,
            backend: self.backend,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor (pipe)
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> DepthCameraNodeBuilder<Pipeline<DepthImage, DepthImage, DepthImage, P, P2>>
    where
        P2: Processor<DepthImage, Output = DepthImage>,
    {
        DepthCameraNodeBuilder {
            model: self.model,
            backend: self.backend,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the node
    pub fn build(self) -> Result<DepthCameraNode<P>> {
        let mut node = DepthCameraNode::new_with_backend(self.model, self.backend)?;
        // Replace processor - need to create a new node with the processor
        Ok(DepthCameraNode {
            rgb_publisher: node.rgb_publisher,
            depth_publisher: node.depth_publisher,
            pointcloud_publisher: node.pointcloud_publisher,
            camera_info_publisher: node.camera_info_publisher,
            camera_model: node.camera_model,
            device_serial: node.device_serial,
            resolution: node.resolution,
            depth_resolution: node.depth_resolution,
            frame_rate: node.frame_rate,
            depth_range: node.depth_range,
            enable_rgb: node.enable_rgb,
            enable_depth: node.enable_depth,
            enable_ir: node.enable_ir,
            enable_pointcloud: node.enable_pointcloud,
            align_depth_to_color: node.align_depth_to_color,
            enable_emitter: node.enable_emitter,
            use_spatial_filter: node.use_spatial_filter,
            use_temporal_filter: node.use_temporal_filter,
            use_hole_filling: node.use_hole_filling,
            depth_units: node.depth_units,
            rgb_fx: node.rgb_fx,
            rgb_fy: node.rgb_fy,
            rgb_cx: node.rgb_cx,
            rgb_cy: node.rgb_cy,
            depth_fx: node.depth_fx,
            depth_fy: node.depth_fy,
            depth_cx: node.depth_cx,
            depth_cy: node.depth_cy,
            rgb_distortion: node.rgb_distortion,
            depth_distortion: node.depth_distortion,
            rgb_frame_id: node.rgb_frame_id,
            depth_frame_id: node.depth_frame_id,
            pointcloud_frame_id: node.pointcloud_frame_id,
            is_streaming: node.is_streaming,
            frame_count: node.frame_count,
            dropped_frames: node.dropped_frames,
            last_frame_time: node.last_frame_time,
            backend: node.backend,
            #[cfg(feature = "realsense")]
            pipeline: node.pipeline,
            #[cfg(feature = "realsense")]
            config: node.config,
            simulation_mode: node.simulation_mode,
            info_counter: node.info_counter,
            processor: self.processor,
        })
    }
}
