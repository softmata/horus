//! Camera Node - Generic camera interface for vision input
//!
//! This node reads images from camera sensors and publishes Image messages.
//! It uses the driver abstraction layer to support multiple hardware backends.

use crate::vision::ImageEncoding;
use crate::{CameraInfo, Image};
use horus_core::driver::{Driver, Sensor};
use horus_core::error::HorusResult;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, Topic};
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

// Import driver types
use crate::drivers::camera::{CameraDriver, CameraDriverBackend, SimulationCameraDriver};

#[cfg(feature = "opencv-backend")]
use crate::drivers::camera::OpenCvCameraDriver;

#[cfg(feature = "v4l2-backend")]
use crate::drivers::camera::V4l2CameraDriver;

/// Camera backend type (deprecated - use CameraDriverBackend instead)
///
/// This enum is kept for backward compatibility. New code should use
/// `CameraDriverBackend` from `crate::drivers::camera`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CameraBackend {
    Simulation,
    OpenCv,
    V4l2,
}

impl From<CameraBackend> for CameraDriverBackend {
    fn from(backend: CameraBackend) -> Self {
        match backend {
            CameraBackend::Simulation => CameraDriverBackend::Simulation,
            #[cfg(feature = "opencv-backend")]
            CameraBackend::OpenCv => CameraDriverBackend::OpenCv,
            #[cfg(not(feature = "opencv-backend"))]
            CameraBackend::OpenCv => CameraDriverBackend::Simulation, // Fallback
            #[cfg(feature = "v4l2-backend")]
            CameraBackend::V4l2 => CameraDriverBackend::V4l2,
            #[cfg(not(feature = "v4l2-backend"))]
            CameraBackend::V4l2 => CameraDriverBackend::Simulation, // Fallback
        }
    }
}

/// Camera Node - Generic camera interface for vision input
///
/// Captures images from various camera sources and publishes Image/CompressedImage messages.
/// Supports multiple backends (OpenCV, V4L2) and configurable image parameters.
///
/// # Driver System
///
/// This node uses the HORUS driver abstraction layer. Drivers handle all
/// hardware-specific code, while the node handles HORUS integration (topics,
/// scheduling, lifecycle).
///
/// ## Supported Drivers
///
/// - `SimulationCameraDriver` - Always available, generates synthetic images
/// - `OpenCvCameraDriver` - OpenCV-based camera (requires `opencv-backend` feature)
/// - `V4l2CameraDriver` - Video4Linux2 camera (requires `v4l2-backend` feature)
///
/// # Example
///
/// ```rust,ignore
/// use horus_library::nodes::CameraNode;
/// use horus_library::drivers::SimulationCameraDriver;
///
/// // Using the default simulation driver
/// let node = CameraNode::new()?;
///
/// // Using a specific driver
/// let driver = SimulationCameraDriver::new();
/// let node = CameraNode::with_driver("camera", driver)?;
///
/// // Using the builder for custom configuration
/// let node = CameraNode::builder()
///     .with_topic("custom_camera")
///     .with_backend(CameraBackend::Simulation)
///     .with_closure(|mut img| {
///         // Process image
///         img
///     })
///     .build()?;
/// ```
pub struct CameraNode<D = CameraDriver, P = PassThrough<Image>>
where
    D: Sensor<Output = Image>,
    P: Processor<Image>,
{
    publisher: Topic<Image>,
    info_publisher: Topic<CameraInfo>,

    // Driver (handles hardware abstraction)
    driver: D,

    // Configuration
    width: u32,
    height: u32,
    fps: f32,
    encoding: ImageEncoding,

    // State
    is_initialized: bool,
    frame_count: u64,
    last_frame_time: u64,

    // Processor for hybrid pattern
    processor: P,
}

impl CameraNode<CameraDriver, PassThrough<Image>> {
    /// Create a new camera node with default topic "camera" in simulation mode
    pub fn new() -> Result<Self> {
        Self::new_with_backend("camera", CameraBackend::Simulation)
    }

    /// Create a new camera node with custom topic prefix
    pub fn new_with_topic(topic_prefix: &str) -> Result<Self> {
        Self::new_with_backend(topic_prefix, CameraBackend::Simulation)
    }

    /// Create a new camera node with specific hardware backend
    pub fn new_with_backend(topic_prefix: &str, backend: CameraBackend) -> Result<Self> {
        let driver_backend: CameraDriverBackend = backend.into();
        let driver = CameraDriver::new(driver_backend)?;

        let image_topic = format!("{}.image", topic_prefix);
        let info_topic = format!("{}.camera_info", topic_prefix);

        Ok(Self {
            publisher: Topic::new(&image_topic)?,
            info_publisher: Topic::new(&info_topic)?,
            driver,
            width: 640,
            height: 480,
            fps: 30.0,
            encoding: ImageEncoding::Bgr8,
            is_initialized: false,
            frame_count: 0,
            last_frame_time: 0,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> CameraNodeBuilder<CameraDriver, PassThrough<Image>> {
        CameraNodeBuilder::new()
    }
}

impl<D> CameraNode<D, PassThrough<Image>>
where
    D: Sensor<Output = Image>,
{
    /// Create a new camera node with a custom driver
    ///
    /// This allows using any driver that implements `Sensor<Output = Image>`,
    /// including custom drivers from the marketplace.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// use horus_library::nodes::CameraNode;
    /// use horus_library::drivers::SimulationCameraDriver;
    ///
    /// let driver = SimulationCameraDriver::new();
    /// let node = CameraNode::with_driver("camera", driver)?;
    /// ```
    pub fn with_driver(topic_prefix: &str, driver: D) -> Result<Self> {
        let image_topic = format!("{}.image", topic_prefix);
        let info_topic = format!("{}.camera_info", topic_prefix);

        Ok(Self {
            publisher: Topic::new(&image_topic)?,
            info_publisher: Topic::new(&info_topic)?,
            driver,
            width: 640,
            height: 480,
            fps: 30.0,
            encoding: ImageEncoding::Bgr8,
            is_initialized: false,
            frame_count: 0,
            last_frame_time: 0,
            processor: PassThrough::new(),
        })
    }
}

impl<D, P> CameraNode<D, P>
where
    D: Sensor<Output = Image>,
    P: Processor<Image>,
{
    /// Set image resolution
    pub fn set_resolution(&mut self, width: u32, height: u32) {
        self.width = width;
        self.height = height;
    }

    /// Set capture framerate
    pub fn set_fps(&mut self, fps: f32) {
        self.fps = fps.clamp(1.0, 120.0);
    }

    /// Set image encoding format
    pub fn set_encoding(&mut self, encoding: ImageEncoding) {
        self.encoding = encoding;
    }

    /// Get current frame rate (frames per second)
    pub fn get_actual_fps(&self) -> f32 {
        if self.frame_count < 2 {
            return 0.0;
        }

        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        let time_diff = current_time - self.last_frame_time;
        if time_diff > 0 {
            1000.0 / time_diff as f32
        } else {
            0.0
        }
    }

    /// Get total frames captured
    pub fn get_frame_count(&self) -> u64 {
        self.frame_count
    }

    /// Get the driver's sample rate (if available)
    pub fn get_sample_rate(&self) -> Option<f32> {
        self.driver.sample_rate()
    }

    /// Check if the driver is available
    pub fn is_driver_available(&self) -> bool {
        self.driver.is_available()
    }

    /// Get the driver ID
    pub fn driver_id(&self) -> &str {
        self.driver.id()
    }

    /// Get the driver name
    pub fn driver_name(&self) -> &str {
        self.driver.name()
    }

    fn publish_camera_info(&self) {
        let camera_info = CameraInfo::new(
            self.width,
            self.height,
            800.0,                    // fx
            800.0,                    // fy
            self.width as f64 / 2.0,  // cx
            self.height as f64 / 2.0, // cy
        );
        let _ = self.info_publisher.send(camera_info, &mut None);
    }
}

impl<D, P> Node for CameraNode<D, P>
where
    D: Sensor<Output = Image>,
    P: Processor<Image>,
{
    fn name(&self) -> &'static str {
        "CameraNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        // Initialize the driver
        self.driver.init()?;
        self.is_initialized = true;

        // Initialize processor
        self.processor.on_start();

        // Publish initial camera info
        self.publish_camera_info();

        ctx.log_info(&format!(
            "CameraNode initialized with driver: {} ({})",
            self.driver.name(),
            self.driver.id()
        ));

        Ok(())
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("CameraNode shutting down - releasing camera resources");

        // Call processor shutdown hook
        self.processor.on_shutdown();

        // Shutdown driver
        self.driver.shutdown()?;

        self.is_initialized = false;
        ctx.log_info("Camera resources released safely");
        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        // Call processor tick hook
        self.processor.on_tick();

        // Check if driver has data available
        if !self.driver.has_data() {
            return;
        }

        // Read and publish image data (through processor pipeline)
        match self.driver.read() {
            Ok(image) => {
                self.frame_count += 1;
                self.last_frame_time = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_millis() as u64;

                // Process through pipeline (filter/transform)
                if let Some(processed) = self.processor.process(image) {
                    let _ = self.publisher.send(processed, &mut None);
                }

                // Publish camera info periodically
                if self.frame_count % 30 == 0 {
                    self.publish_camera_info();
                }
            }
            Err(e) => {
                // Log error but continue - sensor might recover
                eprintln!("CameraNode: Failed to read data: {}", e);
            }
        }
    }
}

/// Builder for CameraNode with custom processor
pub struct CameraNodeBuilder<D, P>
where
    D: Sensor<Output = Image>,
    P: Processor<Image>,
{
    topic_prefix: String,
    driver: Option<D>,
    backend: CameraBackend,
    width: u32,
    height: u32,
    fps: f32,
    encoding: ImageEncoding,
    processor: P,
}

impl CameraNodeBuilder<CameraDriver, PassThrough<Image>> {
    /// Create a new builder with default configuration
    pub fn new() -> Self {
        Self {
            topic_prefix: "camera".to_string(),
            driver: None,
            backend: CameraBackend::Simulation,
            width: 640,
            height: 480,
            fps: 30.0,
            encoding: ImageEncoding::Bgr8,
            processor: PassThrough::new(),
        }
    }
}

impl Default for CameraNodeBuilder<CameraDriver, PassThrough<Image>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<D, P> CameraNodeBuilder<D, P>
where
    D: Sensor<Output = Image>,
    P: Processor<Image>,
{
    /// Set the topic prefix for publishing images
    pub fn topic_prefix(mut self, prefix: &str) -> Self {
        self.topic_prefix = prefix.to_string();
        self
    }

    /// Alias for topic_prefix
    pub fn with_topic(self, prefix: &str) -> Self {
        self.topic_prefix(prefix)
    }

    /// Set the camera backend (creates appropriate driver on build)
    pub fn with_backend(mut self, backend: CameraBackend) -> Self {
        self.backend = backend;
        self
    }

    /// Set image resolution
    pub fn resolution(mut self, width: u32, height: u32) -> Self {
        self.width = width;
        self.height = height;
        self
    }

    /// Set capture framerate
    pub fn fps(mut self, fps: f32) -> Self {
        self.fps = fps.clamp(1.0, 120.0);
        self
    }

    /// Set image encoding format
    pub fn encoding(mut self, encoding: ImageEncoding) -> Self {
        self.encoding = encoding;
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> CameraNodeBuilder<D, P2>
    where
        P2: Processor<Image>,
    {
        CameraNodeBuilder {
            topic_prefix: self.topic_prefix,
            driver: self.driver,
            backend: self.backend,
            width: self.width,
            height: self.height,
            fps: self.fps,
            encoding: self.encoding,
            processor,
        }
    }

    /// Add a closure-based processor
    pub fn with_closure<F>(self, f: F) -> CameraNodeBuilder<D, ClosureProcessor<Image, Image, F>>
    where
        F: FnMut(Image) -> Image + Send + 'static,
    {
        CameraNodeBuilder {
            topic_prefix: self.topic_prefix,
            driver: self.driver,
            backend: self.backend,
            width: self.width,
            height: self.height,
            fps: self.fps,
            encoding: self.encoding,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Add a filter processor
    pub fn with_filter<F>(self, f: F) -> CameraNodeBuilder<D, FilterProcessor<Image, Image, F>>
    where
        F: FnMut(Image) -> Option<Image> + Send + 'static,
    {
        CameraNodeBuilder {
            topic_prefix: self.topic_prefix,
            driver: self.driver,
            backend: self.backend,
            width: self.width,
            height: self.height,
            fps: self.fps,
            encoding: self.encoding,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor in a pipeline
    pub fn pipe<P2>(self, next: P2) -> CameraNodeBuilder<D, Pipeline<Image, Image, Image, P, P2>>
    where
        P2: Processor<Image, Output = Image>,
        P: Processor<Image, Output = Image>,
    {
        CameraNodeBuilder {
            topic_prefix: self.topic_prefix,
            driver: self.driver,
            backend: self.backend,
            width: self.width,
            height: self.height,
            fps: self.fps,
            encoding: self.encoding,
            processor: Pipeline::new(self.processor, next),
        }
    }
}

impl<P> CameraNodeBuilder<CameraDriver, P>
where
    P: Processor<Image>,
{
    /// Build the node with CameraDriver (default driver type)
    pub fn build(self) -> Result<CameraNode<CameraDriver, P>> {
        let driver_backend: CameraDriverBackend = self.backend.into();
        let driver = CameraDriver::new(driver_backend)?;

        let image_topic = format!("{}.image", self.topic_prefix);
        let info_topic = format!("{}.camera_info", self.topic_prefix);

        Ok(CameraNode {
            publisher: Topic::new(&image_topic)?,
            info_publisher: Topic::new(&info_topic)?,
            driver,
            width: self.width,
            height: self.height,
            fps: self.fps,
            encoding: self.encoding,
            is_initialized: false,
            frame_count: 0,
            last_frame_time: 0,
            processor: self.processor,
        })
    }
}

// Builder for custom drivers
impl<D, P> CameraNodeBuilder<D, P>
where
    D: Sensor<Output = Image>,
    P: Processor<Image>,
{
    /// Set a custom driver
    pub fn with_driver<D2>(self, driver: D2) -> CameraNodeBuilder<D2, P>
    where
        D2: Sensor<Output = Image>,
    {
        CameraNodeBuilder {
            topic_prefix: self.topic_prefix,
            driver: Some(driver),
            backend: self.backend,
            width: self.width,
            height: self.height,
            fps: self.fps,
            encoding: self.encoding,
            processor: self.processor,
        }
    }

    /// Build the node with a custom driver (requires driver to be set)
    pub fn build_with_driver(self) -> Result<CameraNode<D, P>>
    where
        D: Default,
    {
        let driver = self.driver.unwrap_or_default();

        let image_topic = format!("{}.image", self.topic_prefix);
        let info_topic = format!("{}.camera_info", self.topic_prefix);

        Ok(CameraNode {
            publisher: Topic::new(&image_topic)?,
            info_publisher: Topic::new(&info_topic)?,
            driver,
            width: self.width,
            height: self.height,
            fps: self.fps,
            encoding: self.encoding,
            is_initialized: false,
            frame_count: 0,
            last_frame_time: 0,
            processor: self.processor,
        })
    }
}

// Convenience type aliases for common driver types
/// CameraNode with SimulationCameraDriver
pub type SimulationCameraNode<P = PassThrough<Image>> = CameraNode<SimulationCameraDriver, P>;

#[cfg(feature = "opencv-backend")]
/// CameraNode with OpenCvCameraDriver
pub type OpenCvCameraNode<P = PassThrough<Image>> = CameraNode<OpenCvCameraDriver, P>;

#[cfg(feature = "v4l2-backend")]
/// CameraNode with V4l2CameraDriver
pub type V4l2CameraNode<P = PassThrough<Image>> = CameraNode<V4l2CameraDriver, P>;
