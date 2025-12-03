use crate::vision::ImageEncoding;
use crate::{CameraInfo, Image};
use horus_core::error::HorusResult;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Hub, Node, NodeInfo};
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

/// Camera Node - Generic camera interface for vision input
///
/// Captures images from various camera sources and publishes Image/CompressedImage messages.
/// Supports multiple backends (OpenCV, V4L2) and configurable image parameters.
///
/// # Hybrid Pattern
///
/// ```rust,ignore
/// let node = CameraNode::builder()
///     .with_closure(|mut img| {
///         // Apply image processing
///         img
///     })
///     .build()?;
/// ```
pub struct CameraNode<P = PassThrough<Image>>
where
    P: Processor<Image>,
{
    publisher: Hub<Image>,
    info_publisher: Hub<CameraInfo>,

    // Configuration
    device_id: u32,
    width: u32,
    height: u32,
    fps: f32,
    encoding: ImageEncoding,
    compress_images: bool,
    quality: u8,

    // State
    is_initialized: bool,
    frame_count: u64,
    last_frame_time: u64,

    #[cfg(feature = "opencv-backend")]
    capture: Option<opencv::videoio::VideoCapture>,

    // V4L2 backend state
    #[cfg(feature = "v4l2-backend")]
    v4l2_fd: Option<i32>,
    #[cfg(feature = "v4l2-backend")]
    v4l2_buffers: Vec<(*mut u8, usize)>,
    #[cfg(feature = "v4l2-backend")]
    v4l2_buffer_count: u32,

    // Processor for hybrid pattern
    processor: P,
}

impl CameraNode {
    /// Create a new camera node with default topic "camera.image"
    pub fn new() -> Result<Self> {
        Self::new_with_topic("camera")
    }

    /// Create a new camera node with custom topic prefix
    pub fn new_with_topic(topic_prefix: &str) -> Result<Self> {
        let image_topic = format!("{}.image", topic_prefix);
        let info_topic = format!("{}.camera_info", topic_prefix);

        Ok(Self {
            publisher: Hub::new(&image_topic)?,
            info_publisher: Hub::new(&info_topic)?,

            device_id: 0,
            width: 640,
            height: 480,
            fps: 30.0,
            encoding: ImageEncoding::Bgr8,
            compress_images: false,
            quality: 90,

            is_initialized: false,
            frame_count: 0,
            last_frame_time: 0,

            #[cfg(feature = "opencv-backend")]
            capture: None,

            #[cfg(feature = "v4l2-backend")]
            v4l2_fd: None,
            #[cfg(feature = "v4l2-backend")]
            v4l2_buffers: Vec::new(),
            #[cfg(feature = "v4l2-backend")]
            v4l2_buffer_count: 4,

            processor: PassThrough::new(),
        })
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> CameraNodeBuilder<PassThrough<Image>> {
        CameraNodeBuilder::new()
    }
}

impl<P> CameraNode<P>
where
    P: Processor<Image>,
{
    /// Set camera device ID (0 for default camera)
    pub fn set_device_id(&mut self, device_id: u32) {
        self.device_id = device_id;
    }

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

    /// Enable/disable image compression
    pub fn set_compression(&mut self, enabled: bool, quality: u8) {
        self.compress_images = enabled;
        self.quality = quality.min(100);
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

    #[cfg(feature = "opencv-backend")]
    fn initialize_opencv(&mut self) -> bool {
        use opencv::prelude::{VideoCaptureTrait, VideoCaptureTraitConst};
        use opencv::videoio::VideoCaptureProperties::{
            CAP_PROP_FPS, CAP_PROP_FRAME_HEIGHT, CAP_PROP_FRAME_WIDTH,
        };
        use opencv::videoio::{VideoCapture, CAP_ANY};

        match VideoCapture::new(self.device_id as i32, CAP_ANY) {
            Ok(mut cap) => {
                // Set camera properties
                let _ = cap.set(CAP_PROP_FRAME_WIDTH.into(), self.width as f64);
                let _ = cap.set(CAP_PROP_FRAME_HEIGHT.into(), self.height as f64);
                let _ = cap.set(CAP_PROP_FPS.into(), self.fps as f64);

                if cap.is_opened().unwrap_or(false) {
                    self.capture = Some(cap);
                    self.publish_camera_info();
                    return true;
                }
            }
            Err(_) => {}
        }
        false
    }

    #[cfg(not(feature = "opencv-backend"))]
    #[allow(dead_code)]
    fn initialize_opencv(&mut self) -> bool {
        false
    }

    #[cfg(feature = "v4l2-backend")]
    #[allow(dead_code)]
    fn initialize_v4l2(&mut self) -> bool {
        use std::ffi::CString;
        use std::os::unix::io::RawFd;

        // V4L2 ioctl constants
        const VIDIOC_QUERYCAP: libc::c_ulong = 0x80685600;
        const VIDIOC_S_FMT: libc::c_ulong = 0xc0d05605;
        const VIDIOC_REQBUFS: libc::c_ulong = 0xc0145608;
        const VIDIOC_QUERYBUF: libc::c_ulong = 0xc0445609;
        const VIDIOC_STREAMON: libc::c_ulong = 0x40045612;
        const VIDIOC_QBUF: libc::c_ulong = 0xc044560f;

        const V4L2_CAP_VIDEO_CAPTURE: u32 = 0x00000001;
        const V4L2_CAP_STREAMING: u32 = 0x04000000;
        const V4L2_BUF_TYPE_VIDEO_CAPTURE: u32 = 1;
        const V4L2_MEMORY_MMAP: u32 = 1;
        const V4L2_PIX_FMT_YUYV: u32 = 0x56595559; // 'YUYV'
        const V4L2_FIELD_NONE: u32 = 1;

        // Open device
        let device_path = format!("/dev/video{}", self.device_id);
        let c_path = match CString::new(device_path.as_str()) {
            Ok(p) => p,
            Err(_) => return false,
        };

        let fd = unsafe { libc::open(c_path.as_ptr(), libc::O_RDWR | libc::O_NONBLOCK) };
        if fd < 0 {
            return false;
        }

        // Query capabilities
        #[repr(C)]
        struct V4l2Capability {
            driver: [u8; 16],
            card: [u8; 32],
            bus_info: [u8; 32],
            version: u32,
            capabilities: u32,
            device_caps: u32,
            reserved: [u32; 3],
        }

        let mut cap: V4l2Capability = unsafe { std::mem::zeroed() };
        if unsafe { libc::ioctl(fd, VIDIOC_QUERYCAP, &mut cap) } < 0 {
            unsafe { libc::close(fd) };
            return false;
        }

        // Check for video capture and streaming support
        if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0 ||
           (cap.capabilities & V4L2_CAP_STREAMING) == 0 {
            unsafe { libc::close(fd) };
            return false;
        }

        // Set format
        #[repr(C)]
        struct V4l2PixFormat {
            width: u32,
            height: u32,
            pixelformat: u32,
            field: u32,
            bytesperline: u32,
            sizeimage: u32,
            colorspace: u32,
            priv_: u32,
            flags: u32,
            hsv_enc_or_ycbcr_enc: u32,
            quantization: u32,
            xfer_func: u32,
        }

        #[repr(C)]
        struct V4l2Format {
            type_: u32,
            fmt: V4l2PixFormat,
            _padding: [u8; 156], // Ensure struct is large enough
        }

        let mut fmt: V4l2Format = unsafe { std::mem::zeroed() };
        fmt.type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.width = self.width;
        fmt.fmt.height = self.height;
        fmt.fmt.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.field = V4L2_FIELD_NONE;

        if unsafe { libc::ioctl(fd, VIDIOC_S_FMT, &mut fmt) } < 0 {
            unsafe { libc::close(fd) };
            return false;
        }

        // Update actual dimensions from driver
        self.width = fmt.fmt.width;
        self.height = fmt.fmt.height;

        // Request buffers
        #[repr(C)]
        struct V4l2RequestBuffers {
            count: u32,
            type_: u32,
            memory: u32,
            capabilities: u32,
            flags: u8,
            reserved: [u8; 3],
        }

        let mut req: V4l2RequestBuffers = unsafe { std::mem::zeroed() };
        req.count = self.v4l2_buffer_count;
        req.type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if unsafe { libc::ioctl(fd, VIDIOC_REQBUFS, &mut req) } < 0 {
            unsafe { libc::close(fd) };
            return false;
        }

        // Map buffers
        #[repr(C)]
        struct V4l2Buffer {
            index: u32,
            type_: u32,
            bytesused: u32,
            flags: u32,
            field: u32,
            timestamp: libc::timeval,
            timecode: [u8; 32], // v4l2_timecode
            sequence: u32,
            memory: u32,
            m_offset: u32, // union with userptr, fd
            length: u32,
            reserved2: u32,
            reserved: u32,
        }

        self.v4l2_buffers.clear();

        for i in 0..req.count {
            let mut buf: V4l2Buffer = unsafe { std::mem::zeroed() };
            buf.type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if unsafe { libc::ioctl(fd, VIDIOC_QUERYBUF, &mut buf) } < 0 {
                // Unmap already mapped buffers
                for (ptr, len) in &self.v4l2_buffers {
                    unsafe { libc::munmap(*ptr as *mut libc::c_void, *len) };
                }
                self.v4l2_buffers.clear();
                unsafe { libc::close(fd) };
                return false;
            }

            let ptr = unsafe {
                libc::mmap(
                    std::ptr::null_mut(),
                    buf.length as usize,
                    libc::PROT_READ | libc::PROT_WRITE,
                    libc::MAP_SHARED,
                    fd,
                    buf.m_offset as libc::off_t,
                )
            };

            if ptr == libc::MAP_FAILED {
                for (ptr, len) in &self.v4l2_buffers {
                    unsafe { libc::munmap(*ptr as *mut libc::c_void, *len) };
                }
                self.v4l2_buffers.clear();
                unsafe { libc::close(fd) };
                return false;
            }

            self.v4l2_buffers.push((ptr as *mut u8, buf.length as usize));

            // Queue buffer
            if unsafe { libc::ioctl(fd, VIDIOC_QBUF, &buf) } < 0 {
                for (ptr, len) in &self.v4l2_buffers {
                    unsafe { libc::munmap(*ptr as *mut libc::c_void, *len) };
                }
                self.v4l2_buffers.clear();
                unsafe { libc::close(fd) };
                return false;
            }
        }

        // Start streaming
        let buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if unsafe { libc::ioctl(fd, VIDIOC_STREAMON, &buf_type) } < 0 {
            for (ptr, len) in &self.v4l2_buffers {
                unsafe { libc::munmap(*ptr as *mut libc::c_void, *len) };
            }
            self.v4l2_buffers.clear();
            unsafe { libc::close(fd) };
            return false;
        }

        self.v4l2_fd = Some(fd);
        self.publish_camera_info();
        true
    }

    #[cfg(not(feature = "v4l2-backend"))]
    #[allow(dead_code)]
    fn initialize_v4l2(&mut self) -> bool {
        false
    }

    fn initialize_camera(&mut self) -> bool {
        if self.is_initialized {
            return true;
        }

        // Try different backends
        #[cfg(feature = "opencv-backend")]
        if self.initialize_opencv() {
            self.is_initialized = true;
            return true;
        }

        #[cfg(feature = "v4l2-backend")]
        if self.initialize_v4l2() {
            self.is_initialized = true;
            return true;
        }

        false
    }

    #[cfg(feature = "opencv-backend")]
    fn capture_opencv_frame(&mut self) -> Option<Vec<u8>> {
        use opencv::core::Mat;
        use opencv::prelude::{MatTraitConst, MatTraitConstManual, VideoCaptureTrait};

        if let Some(ref mut cap) = self.capture {
            let mut frame = Mat::default();
            if cap.read(&mut frame).unwrap_or(false) && !frame.empty() {
                // Convert Mat to Vec<u8>
                if let Ok(bytes) = frame.data_bytes() {
                    return Some(bytes.to_vec());
                }
            }
        }
        None
    }

    #[cfg(not(feature = "opencv-backend"))]
    #[allow(dead_code)]
    fn capture_opencv_frame(&mut self) -> Option<Vec<u8>> {
        None
    }

    #[cfg(feature = "v4l2-backend")]
    fn capture_v4l2_frame(&mut self) -> Option<Vec<u8>> {
        const VIDIOC_DQBUF: libc::c_ulong = 0xc0445611;
        const VIDIOC_QBUF: libc::c_ulong = 0xc044560f;
        const V4L2_BUF_TYPE_VIDEO_CAPTURE: u32 = 1;
        const V4L2_MEMORY_MMAP: u32 = 1;

        let fd = self.v4l2_fd?;

        #[repr(C)]
        struct V4l2Buffer {
            index: u32,
            type_: u32,
            bytesused: u32,
            flags: u32,
            field: u32,
            timestamp: libc::timeval,
            timecode: [u8; 32],
            sequence: u32,
            memory: u32,
            m_offset: u32,
            length: u32,
            reserved2: u32,
            reserved: u32,
        }

        // Dequeue a buffer
        let mut buf: V4l2Buffer = unsafe { std::mem::zeroed() };
        buf.type_ = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if unsafe { libc::ioctl(fd, VIDIOC_DQBUF, &mut buf) } < 0 {
            // Would block or error - no frame available
            return None;
        }

        let idx = buf.index as usize;
        if idx >= self.v4l2_buffers.len() {
            // Re-queue the buffer
            unsafe { libc::ioctl(fd, VIDIOC_QBUF, &buf) };
            return None;
        }

        let (ptr, _len) = self.v4l2_buffers[idx];
        let bytes_used = buf.bytesused as usize;

        // Copy YUYV data
        let yuyv_data = unsafe { std::slice::from_raw_parts(ptr, bytes_used) };

        // Convert YUYV to BGR (OpenCV format)
        let pixel_count = (self.width * self.height) as usize;
        let mut bgr_data = vec![0u8; pixel_count * 3];

        // YUYV to BGR conversion
        // Each 4 bytes (Y0 U Y1 V) produces 2 BGR pixels
        for i in 0..(pixel_count / 2) {
            let yuyv_idx = i * 4;
            if yuyv_idx + 3 >= yuyv_data.len() {
                break;
            }

            let y0 = yuyv_data[yuyv_idx] as f32;
            let u = yuyv_data[yuyv_idx + 1] as f32 - 128.0;
            let y1 = yuyv_data[yuyv_idx + 2] as f32;
            let v = yuyv_data[yuyv_idx + 3] as f32 - 128.0;

            // Pixel 0
            let bgr_idx0 = i * 6;
            bgr_data[bgr_idx0] = (y0 + 1.772 * u).clamp(0.0, 255.0) as u8;         // B
            bgr_data[bgr_idx0 + 1] = (y0 - 0.344 * u - 0.714 * v).clamp(0.0, 255.0) as u8; // G
            bgr_data[bgr_idx0 + 2] = (y0 + 1.402 * v).clamp(0.0, 255.0) as u8;     // R

            // Pixel 1
            let bgr_idx1 = i * 6 + 3;
            if bgr_idx1 + 2 < bgr_data.len() {
                bgr_data[bgr_idx1] = (y1 + 1.772 * u).clamp(0.0, 255.0) as u8;         // B
                bgr_data[bgr_idx1 + 1] = (y1 - 0.344 * u - 0.714 * v).clamp(0.0, 255.0) as u8; // G
                bgr_data[bgr_idx1 + 2] = (y1 + 1.402 * v).clamp(0.0, 255.0) as u8;     // R
            }
        }

        // Re-queue the buffer for next frame
        unsafe { libc::ioctl(fd, VIDIOC_QBUF, &buf) };

        Some(bgr_data)
    }

    #[cfg(not(feature = "v4l2-backend"))]
    #[allow(dead_code)]
    fn capture_v4l2_frame(&mut self) -> Option<Vec<u8>> {
        None
    }

    fn capture_frame(&mut self) -> Option<Vec<u8>> {
        // Hardware-only: no test pattern fallback
        // If backend initialization failed, this returns None
        #[cfg(feature = "opencv-backend")]
        {
            return self.capture_opencv_frame();
        }

        #[cfg(feature = "v4l2-backend")]
        {
            return self.capture_v4l2_frame();
        }

        #[cfg(not(any(feature = "opencv-backend", feature = "v4l2-backend")))]
        {
            // No backend enabled - this should never compile due to module-level feature gate
            compile_error!("CameraNode requires at least one camera backend feature: opencv-backend, v4l2-backend, realsense, or zed");
        }
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

impl<P> Node for CameraNode<P>
where
    P: Processor<Image>,
{
    fn name(&self) -> &'static str {
        "CameraNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_start();
        ctx.log_info("CameraNode initialized");
        Ok(())
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_shutdown();
        ctx.log_info("CameraNode shutting down - releasing camera resources");

        // Release OpenCV capture device
        #[cfg(feature = "opencv-backend")]
        {
            self.capture = None;
        }

        // Release V4L2 resources
        #[cfg(feature = "v4l2-backend")]
        {
            const VIDIOC_STREAMOFF: libc::c_ulong = 0x40045613;
            const V4L2_BUF_TYPE_VIDEO_CAPTURE: u32 = 1;

            if let Some(fd) = self.v4l2_fd.take() {
                // Stop streaming
                let buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                unsafe { libc::ioctl(fd, VIDIOC_STREAMOFF, &buf_type) };

                // Unmap buffers
                for (ptr, len) in self.v4l2_buffers.drain(..) {
                    unsafe { libc::munmap(ptr as *mut libc::c_void, len) };
                }

                // Close device
                unsafe { libc::close(fd) };
            }
        }

        self.is_initialized = false;
        ctx.log_info("Camera resources released safely");
        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        self.processor.on_tick();

        // Initialize camera on first tick
        if !self.is_initialized && !self.initialize_camera() {
            return; // Skip if initialization failed
        }

        // Capture and publish frame
        if let Some(data) = self.capture_frame() {
            self.frame_count += 1;
            self.last_frame_time = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64;

            // Build image message
            let image = Image::new(self.width, self.height, self.encoding, data);

            // Process through pipeline and publish
            if let Some(processed) = self.processor.process(image) {
                let _ = self.publisher.send(processed, &mut None);
            }

            // Publish camera info periodically
            if self.frame_count % 30 == 0 {
                self.publish_camera_info();
            }
        }
    }
}

/// Builder for CameraNode with fluent API for processor configuration
pub struct CameraNodeBuilder<P>
where
    P: Processor<Image>,
{
    topic_prefix: String,
    device_id: u32,
    width: u32,
    height: u32,
    fps: f32,
    encoding: ImageEncoding,
    processor: P,
}

impl CameraNodeBuilder<PassThrough<Image>> {
    /// Create a new builder with default settings
    pub fn new() -> Self {
        Self {
            topic_prefix: "camera".to_string(),
            device_id: 0,
            width: 640,
            height: 480,
            fps: 30.0,
            encoding: ImageEncoding::Bgr8,
            processor: PassThrough::new(),
        }
    }
}

impl<P> CameraNodeBuilder<P>
where
    P: Processor<Image>,
{
    /// Set the topic prefix for publishing images
    pub fn topic_prefix(mut self, prefix: &str) -> Self {
        self.topic_prefix = prefix.to_string();
        self
    }

    /// Set the camera device ID
    pub fn device_id(mut self, id: u32) -> Self {
        self.device_id = id;
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
    pub fn with_processor<P2>(self, processor: P2) -> CameraNodeBuilder<P2>
    where
        P2: Processor<Image>,
    {
        CameraNodeBuilder {
            topic_prefix: self.topic_prefix,
            device_id: self.device_id,
            width: self.width,
            height: self.height,
            fps: self.fps,
            encoding: self.encoding,
            processor,
        }
    }

    /// Add a closure processor for transformations
    pub fn with_closure<F>(self, f: F) -> CameraNodeBuilder<ClosureProcessor<Image, Image, F>>
    where
        F: FnMut(Image) -> Image + Send + 'static,
    {
        CameraNodeBuilder {
            topic_prefix: self.topic_prefix,
            device_id: self.device_id,
            width: self.width,
            height: self.height,
            fps: self.fps,
            encoding: self.encoding,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Add a filter processor
    pub fn with_filter<F>(self, f: F) -> CameraNodeBuilder<FilterProcessor<Image, Image, F>>
    where
        F: FnMut(Image) -> Option<Image> + Send + 'static,
    {
        CameraNodeBuilder {
            topic_prefix: self.topic_prefix,
            device_id: self.device_id,
            width: self.width,
            height: self.height,
            fps: self.fps,
            encoding: self.encoding,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor in a pipeline
    pub fn pipe<P2>(self, next: P2) -> CameraNodeBuilder<Pipeline<Image, Image, Image, P, P2>>
    where
        P2: Processor<Image, Image>,
    {
        CameraNodeBuilder {
            topic_prefix: self.topic_prefix,
            device_id: self.device_id,
            width: self.width,
            height: self.height,
            fps: self.fps,
            encoding: self.encoding,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the CameraNode
    pub fn build(self) -> Result<CameraNode<P>> {
        let image_topic = format!("{}.image", self.topic_prefix);
        let info_topic = format!("{}.camera_info", self.topic_prefix);

        Ok(CameraNode {
            publisher: Hub::new(&image_topic)?,
            info_publisher: Hub::new(&info_topic)?,
            device_id: self.device_id,
            width: self.width,
            height: self.height,
            fps: self.fps,
            encoding: self.encoding,
            compress_images: false,
            quality: 90,
            is_initialized: false,
            frame_count: 0,
            last_frame_time: 0,
            #[cfg(feature = "opencv-backend")]
            capture: None,
            #[cfg(feature = "v4l2-backend")]
            v4l2_fd: None,
            #[cfg(feature = "v4l2-backend")]
            v4l2_buffers: Vec::new(),
            #[cfg(feature = "v4l2-backend")]
            v4l2_buffer_count: 4,
            processor: self.processor,
        })
    }
}
