use crate::Image;
use horus_core::error::HorusResult;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, NodeInfoExt, Topic};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

#[cfg(feature = "opencv")]
use opencv::{
    core::{Mat, Size},
    imgproc,
    prelude::*,
};

/// Image backend type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ImageBackend {
    Simulation,
    OpenCV,
}

/// Image Processor Node - Computer vision preprocessing and filtering
///
/// Performs common image processing operations like resizing, filtering,
/// color space conversion, and edge detection. Useful for preparing
/// images for object detection, feature extraction, or visualization.
///
/// Supported backends:
/// - OpenCV (cv::imgproc for actual image processing)
/// - Simulation mode for testing
///
/// # Hybrid Pattern
///
/// ```rust,ignore
/// let node = ImageProcessorNode::builder()
///     .with_closure(|img| {
///         // Add custom post-processing
///         img
///     })
///     .build()?;
/// ```
pub struct ImageProcessorNode<P = PassThrough<Image>>
where
    P: Processor<Image>,
{
    subscriber: Topic<Image>,
    publisher: Topic<Image>,

    // Configuration
    target_width: u32,
    target_height: u32,
    resize_enabled: bool,
    grayscale_enabled: bool,
    gaussian_blur_size: u32,
    edge_detection_enabled: bool,
    brightness_adjustment: f32, // -1.0 to 1.0
    contrast_adjustment: f32,   // 0.5 to 2.0
    backend: ImageBackend,

    // Hardware fields (reserved for caching processed images)
    #[cfg(feature = "opencv")]
    _opencv_mat: Option<Mat>,

    // Statistics
    images_processed: u64,
    processing_time_us: u64,

    // Processor for hybrid pattern
    processor: P,
}

impl ImageProcessorNode {
    /// Create a new image processor node in simulation mode
    pub fn new() -> Result<Self> {
        Self::new_with_backend("camera.image", "camera.processed", ImageBackend::Simulation)
    }

    /// Create with custom input/output topics
    pub fn new_with_topics(input_topic: &str, output_topic: &str) -> Result<Self> {
        Self::new_with_backend(input_topic, output_topic, ImageBackend::Simulation)
    }

    /// Create with specific backend
    pub fn new_with_backend(
        input_topic: &str,
        output_topic: &str,
        backend: ImageBackend,
    ) -> Result<Self> {
        Ok(Self {
            subscriber: Topic::new(input_topic)?,
            publisher: Topic::new(output_topic)?,
            target_width: 640,
            target_height: 480,
            resize_enabled: false,
            grayscale_enabled: false,
            gaussian_blur_size: 0,
            edge_detection_enabled: false,
            brightness_adjustment: 0.0,
            contrast_adjustment: 1.0,
            backend,
            #[cfg(feature = "opencv")]
            _opencv_mat: None,
            images_processed: 0,
            processing_time_us: 0,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> ImageProcessorNodeBuilder<PassThrough<Image>> {
        ImageProcessorNodeBuilder::new()
    }
}

impl<P> ImageProcessorNode<P>
where
    P: Processor<Image>,
{
    /// Set image processing backend
    pub fn set_backend(&mut self, backend: ImageBackend) {
        self.backend = backend;
    }

    /// Enable image resizing
    pub fn enable_resize(&mut self, width: u32, height: u32) {
        self.target_width = width;
        self.target_height = height;
        self.resize_enabled = true;
    }

    /// Disable image resizing
    pub fn disable_resize(&mut self) {
        self.resize_enabled = false;
    }

    /// Enable grayscale conversion
    pub fn enable_grayscale(&mut self) {
        self.grayscale_enabled = true;
    }

    /// Disable grayscale conversion
    pub fn disable_grayscale(&mut self) {
        self.grayscale_enabled = false;
    }

    /// Enable Gaussian blur (size must be odd: 3, 5, 7, etc.)
    pub fn enable_gaussian_blur(&mut self, kernel_size: u32) {
        self.gaussian_blur_size = if kernel_size % 2 == 0 {
            kernel_size + 1
        } else {
            kernel_size
        };
    }

    /// Disable Gaussian blur
    pub fn disable_gaussian_blur(&mut self) {
        self.gaussian_blur_size = 0;
    }

    /// Enable edge detection (Canny, Sobel, etc.)
    pub fn enable_edge_detection(&mut self) {
        self.edge_detection_enabled = true;
    }

    /// Disable edge detection
    pub fn disable_edge_detection(&mut self) {
        self.edge_detection_enabled = false;
    }

    /// Set brightness adjustment (-1.0 to 1.0)
    pub fn set_brightness(&mut self, brightness: f32) {
        self.brightness_adjustment = brightness.clamp(-1.0, 1.0);
    }

    /// Set contrast adjustment (0.5 to 2.0)
    pub fn set_contrast(&mut self, contrast: f32) {
        self.contrast_adjustment = contrast.clamp(0.5, 2.0);
    }

    /// Get processing statistics
    pub fn get_stats(&self) -> (u64, u64) {
        (self.images_processed, self.processing_time_us)
    }

    /// Process an image through the pipeline
    fn process_image(&mut self, image: Image, ctx: Option<&mut NodeInfo>) -> Option<Image> {
        let _start_time = std::time::Instant::now(); // Reserved for timing statistics

        match self.backend {
            ImageBackend::Simulation => self.process_image_simulation(image, ctx),
            #[cfg(feature = "opencv")]
            ImageBackend::OpenCV => self.process_image_opencv(image, ctx),
            #[cfg(not(feature = "opencv"))]
            ImageBackend::OpenCV => {
                ctx.log_warning("OpenCV backend requested but opencv feature not enabled");
                ctx.log_warning("Falling back to simulation mode");
                self.backend = ImageBackend::Simulation;
                self.process_image_simulation(image, ctx)
            }
        }
    }

    /// Process image using simulation (no actual processing)
    fn process_image_simulation(
        &mut self,
        image: Image,
        mut ctx: Option<&mut NodeInfo>,
    ) -> Option<Image> {
        let start_time = std::time::Instant::now();

        // Create output image (start with copy)
        let mut processed = image.clone();

        // Apply processing pipeline (simulation only - no actual pixel manipulation)

        // 1. Resize if enabled
        if self.resize_enabled
            && (image.width != self.target_width || image.height != self.target_height)
        {
            ctx.log_debug(&format!(
                "Resizing from {}x{} to {}x{} (simulation)",
                image.width, image.height, self.target_width, self.target_height
            ));

            // Simulation: just update dimensions
            processed.width = self.target_width;
            processed.height = self.target_height;
        }

        // 2. Convert to grayscale if enabled
        if self.grayscale_enabled {
            ctx.log_debug("Converting to grayscale (simulation)");
        }

        // 3. Apply Gaussian blur if enabled
        if self.gaussian_blur_size > 0 {
            ctx.log_debug(&format!(
                "Applying Gaussian blur (kernel size {}) (simulation)",
                self.gaussian_blur_size
            ));
        }

        // 4. Apply brightness/contrast adjustments
        if self.brightness_adjustment != 0.0 || self.contrast_adjustment != 1.0 {
            ctx.log_debug(&format!(
                "Adjusting brightness={:.2}, contrast={:.2} (simulation)",
                self.brightness_adjustment, self.contrast_adjustment
            ));
        }

        // 5. Apply edge detection if enabled
        if self.edge_detection_enabled {
            ctx.log_debug("Applying edge detection (simulation)");
        }

        // Update statistics
        let elapsed = start_time.elapsed();
        self.processing_time_us = elapsed.as_micros() as u64;
        self.images_processed += 1;

        Some(processed)
    }

    #[cfg(feature = "opencv")]
    /// Process image using OpenCV
    fn process_image_opencv(
        &mut self,
        image: Image,
        mut ctx: Option<&mut NodeInfo>,
    ) -> Option<Image> {
        let start_time = std::time::Instant::now();

        // Convert HORUS Image to OpenCV Mat
        let mat_result = Mat::from_slice(&image.data);
        if mat_result.is_err() {
            ctx.log_error(&format!(
                "Failed to create Mat from image data: {:?}",
                mat_result.err()
            ));
            return None;
        }
        let mat = mat_result.unwrap();

        // Reshape to image dimensions
        let rows = image.height as i32;
        let channels = match image.encoding {
            crate::vision::ImageEncoding::Rgb8 | crate::vision::ImageEncoding::Bgr8 => 3,
            crate::vision::ImageEncoding::Mono8 => 1,
            _ => 3,
        };

        let reshaped = match mat.reshape(channels, rows) {
            Ok(m) => m,
            Err(e) => {
                ctx.log_error(&format!("Failed to reshape Mat: {:?}", e));
                return None;
            }
        };

        let mut working_mat = match reshaped.try_clone() {
            Ok(m) => m,
            Err(e) => {
                ctx.log_error(&format!("Failed to clone Mat: {:?}", e));
                return None;
            }
        };

        // 1. Resize if enabled
        if self.resize_enabled
            && (image.width != self.target_width || image.height != self.target_height)
        {
            ctx.log_debug(&format!(
                "Resizing from {}x{} to {}x{} (OpenCV)",
                image.width, image.height, self.target_width, self.target_height
            ));

            let mut resized = Mat::default();
            let size = Size::new(self.target_width as i32, self.target_height as i32);
            match imgproc::resize(
                &working_mat,
                &mut resized,
                size,
                0.0,
                0.0,
                imgproc::INTER_LINEAR,
            ) {
                Ok(_) => working_mat = resized,
                Err(e) => ctx.log_error(&format!("Resize failed: {:?}", e)),
            }
        }

        // 2. Convert to grayscale if enabled
        if self.grayscale_enabled && channels == 3 {
            ctx.log_debug("Converting to grayscale (OpenCV)");
            let mut gray = Mat::default();
            match imgproc::cvt_color(&working_mat, &mut gray, imgproc::COLOR_BGR2GRAY, 0) {
                Ok(_) => working_mat = gray,
                Err(e) => ctx.log_error(&format!("Grayscale conversion failed: {:?}", e)),
            }
        }

        // 3. Apply Gaussian blur if enabled
        if self.gaussian_blur_size > 0 {
            ctx.log_debug(&format!(
                "Applying Gaussian blur (kernel size {}) (OpenCV)",
                self.gaussian_blur_size
            ));
            let mut blurred = Mat::default();
            let ksize = Size::new(
                self.gaussian_blur_size as i32,
                self.gaussian_blur_size as i32,
            );
            match imgproc::gaussian_blur(
                &working_mat,
                &mut blurred,
                ksize,
                0.0,
                0.0,
                opencv::core::BORDER_DEFAULT,
            ) {
                Ok(_) => working_mat = blurred,
                Err(e) => ctx.log_error(&format!("Gaussian blur failed: {:?}", e)),
            }
        }

        // 4. Apply brightness/contrast adjustments
        if self.brightness_adjustment != 0.0 || self.contrast_adjustment != 1.0 {
            ctx.log_debug(&format!(
                "Adjusting brightness={:.2}, contrast={:.2} (OpenCV)",
                self.brightness_adjustment, self.contrast_adjustment
            ));
            let mut adjusted = Mat::default();
            // convertTo: output = input * alpha + beta
            let alpha = self.contrast_adjustment as f64;
            let beta = self.brightness_adjustment as f64 * 255.0;
            match working_mat.convert_to(&mut adjusted, -1, alpha, beta) {
                Ok(_) => working_mat = adjusted,
                Err(e) => ctx.log_error(&format!("Brightness/contrast adjustment failed: {:?}", e)),
            }
        }

        // 5. Apply edge detection if enabled
        if self.edge_detection_enabled {
            ctx.log_debug("Applying edge detection (OpenCV Canny)");

            // Convert to grayscale if needed
            let mut gray_for_edges = working_mat.clone();
            if working_mat.channels() == 3 {
                let mut temp_gray = Mat::default();
                if imgproc::cvt_color(&working_mat, &mut temp_gray, imgproc::COLOR_BGR2GRAY, 0)
                    .is_ok()
                {
                    gray_for_edges = temp_gray;
                }
            }

            let mut edges = Mat::default();
            match imgproc::canny(&gray_for_edges, &mut edges, 50.0, 150.0, 3, false) {
                Ok(_) => working_mat = edges,
                Err(e) => ctx.log_error(&format!("Canny edge detection failed: {:?}", e)),
            }
        }

        // Convert back to HORUS Image
        let data = match working_mat.data_bytes() {
            Ok(bytes) => bytes.to_vec(),
            Err(e) => {
                ctx.log_error(&format!("Failed to extract Mat data: {:?}", e));
                return None;
            }
        };

        let mut processed = image.clone();
        processed.width = working_mat.cols() as u32;
        processed.height = working_mat.rows() as u32;
        processed.data = data;
        processed.step = (working_mat.cols() * working_mat.channels()) as u32;

        // Update encoding if changed to grayscale
        if working_mat.channels() == 1 {
            processed.encoding = crate::vision::ImageEncoding::Mono8;
        }

        // Update statistics
        let elapsed = start_time.elapsed();
        self.processing_time_us = elapsed.as_micros() as u64;
        self.images_processed += 1;

        ctx.log_debug(&format!(
            "Image processed in {} μs (total: {})",
            self.processing_time_us, self.images_processed
        ));

        Some(processed)
    }
}

impl<P> Node for ImageProcessorNode<P>
where
    P: Processor<Image>,
{
    fn name(&self) -> &'static str {
        "ImageProcessorNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_start();
        ctx.log_info("Image processor node initialized");

        match self.backend {
            ImageBackend::Simulation => {
                ctx.log_info("Image processing simulation mode enabled");
            }
            ImageBackend::OpenCV => {
                #[cfg(feature = "opencv")]
                ctx.log_info("Image processing OpenCV backend enabled");
                #[cfg(not(feature = "opencv"))]
                {
                    ctx.log_warning("OpenCV backend requested but opencv feature not enabled");
                    ctx.log_warning("Falling back to simulation mode");
                    self.backend = ImageBackend::Simulation;
                }
            }
        }

        Ok(())
    }

    fn shutdown(&mut self, _ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_shutdown();
        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        self.processor.on_tick();

        // Process all available images
        while let Some(image) = self.subscriber.recv(&mut None) {
            if let Some(processed) = self.process_image(image, ctx.as_deref_mut()) {
                // Process through pipeline
                if let Some(final_output) = self.processor.process(processed) {
                    let _ = self.publisher.send(final_output, &mut None);
                }
            }
        }
    }
}

/// Builder for ImageProcessorNode with processor configuration
pub struct ImageProcessorNodeBuilder<P>
where
    P: Processor<Image>,
{
    input_topic: String,
    output_topic: String,
    backend: ImageBackend,
    processor: P,
}

impl ImageProcessorNodeBuilder<PassThrough<Image>> {
    /// Create a new builder with default PassThrough processor
    pub fn new() -> Self {
        Self {
            input_topic: "camera.image".to_string(),
            output_topic: "camera.processed".to_string(),
            backend: ImageBackend::Simulation,
            processor: PassThrough::new(),
        }
    }
}

impl Default for ImageProcessorNodeBuilder<PassThrough<Image>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P> ImageProcessorNodeBuilder<P>
where
    P: Processor<Image>,
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

    /// Set image processing backend
    pub fn backend(mut self, backend: ImageBackend) -> Self {
        self.backend = backend;
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> ImageProcessorNodeBuilder<P2>
    where
        P2: Processor<Image>,
    {
        ImageProcessorNodeBuilder {
            input_topic: self.input_topic,
            output_topic: self.output_topic,
            backend: self.backend,
            processor,
        }
    }

    /// Set a closure-based processor
    pub fn with_closure<F>(
        self,
        f: F,
    ) -> ImageProcessorNodeBuilder<ClosureProcessor<Image, Image, F>>
    where
        F: FnMut(Image) -> Image + Send + 'static,
    {
        ImageProcessorNodeBuilder {
            input_topic: self.input_topic,
            output_topic: self.output_topic,
            backend: self.backend,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Set a filter-based processor
    pub fn with_filter<F>(self, f: F) -> ImageProcessorNodeBuilder<FilterProcessor<Image, Image, F>>
    where
        F: FnMut(Image) -> Option<Image> + Send + 'static,
    {
        ImageProcessorNodeBuilder {
            input_topic: self.input_topic,
            output_topic: self.output_topic,
            backend: self.backend,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor (pipe)
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> ImageProcessorNodeBuilder<Pipeline<Image, Image, Image, P, P2>>
    where
        P2: Processor<Image, Image>,
    {
        ImageProcessorNodeBuilder {
            input_topic: self.input_topic,
            output_topic: self.output_topic,
            backend: self.backend,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the node
    pub fn build(self) -> Result<ImageProcessorNode<P>> {
        Ok(ImageProcessorNode {
            subscriber: Topic::new(&self.input_topic)?,
            publisher: Topic::new(&self.output_topic)?,
            target_width: 640,
            target_height: 480,
            resize_enabled: false,
            grayscale_enabled: false,
            gaussian_blur_size: 0,
            edge_detection_enabled: false,
            brightness_adjustment: 0.0,
            contrast_adjustment: 1.0,
            backend: self.backend,
            #[cfg(feature = "opencv")]
            _opencv_mat: None,
            images_processed: 0,
            processing_time_us: 0,
            processor: self.processor,
        })
    }
}
