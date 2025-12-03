# Camera Node

Generic camera interface for vision input with support for multiple backends.

## Overview

The Camera Node provides a unified interface for capturing images from various camera sources. It supports multiple backends (OpenCV, V4L2) and publishes both raw image data and camera calibration information. The node handles camera initialization, frame capture, and configurable image parameters including resolution, framerate, and encoding format.

**Hardware-Only**: This node requires actual camera hardware. It will not compile without a camera backend feature (`opencv-backend`, `v4l2-backend`, `realsense`, or `zed`). For testing without hardware, use the `sim2d` or `sim3d` simulation tools instead.

## Topics

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `camera/image` | `Image` | Raw image frames from camera |
| `camera/camera_info` | `CameraInfo` | Camera calibration and intrinsic parameters |

Note: Topic names can be customized using `new_with_topic()` constructor.

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `device_id` | `u32` | `0` | Camera device identifier (0 for default camera) |
| `width` | `u32` | `640` | Image width in pixels |
| `height` | `u32` | `480` | Image height in pixels |
| `fps` | `f32` | `30.0` | Target capture framerate (1.0 - 120.0 Hz) |
| `encoding` | `ImageEncoding` | `Bgr8` | Pixel encoding format |
| `compress_images` | `bool` | `false` | Enable JPEG/PNG compression |
| `quality` | `u8` | `90` | Compression quality (0-100, higher = better) |

## Message Types

### Image

Raw image data message with metadata:

```rust
pub struct Image {
    pub width: u32,              // Image width in pixels
    pub height: u32,             // Image height in pixels
    pub encoding: ImageEncoding, // Pixel encoding format
    pub step: u32,               // Bytes per row (may include padding)
    pub data: Vec<u8>,           // Image data (row-major order)
    pub frame_id: [u8; 32],      // Camera identifier
    pub timestamp: u64,          // Nanoseconds since epoch
}
```

### ImageEncoding

Supported pixel formats:

```rust
pub enum ImageEncoding {
    Mono8,       // 8-bit grayscale (1 byte/pixel)
    Mono16,      // 16-bit grayscale (2 bytes/pixel)
    Rgb8,        // 8-bit RGB (3 bytes/pixel)
    Bgr8,        // 8-bit BGR - OpenCV format (3 bytes/pixel)
    Rgba8,       // 8-bit RGBA with alpha (4 bytes/pixel)
    Bgra8,       // 8-bit BGRA with alpha (4 bytes/pixel)
    Yuv422,      // YUV 4:2:2 format (2 bytes/pixel)
    Mono32F,     // 32-bit float grayscale (4 bytes/pixel)
    Rgb32F,      // 32-bit float RGB (12 bytes/pixel)
    BayerRggb8,  // Bayer pattern raw sensor data (1 byte/pixel)
    Depth16,     // 16-bit depth in millimeters (2 bytes/pixel)
}
```

### CameraInfo

Camera calibration and intrinsic parameters:

```rust
pub struct CameraInfo {
    pub width: u32,                          // Image width
    pub height: u32,                         // Image height
    pub distortion_model: [u8; 16],          // "plumb_bob", "rational_polynomial"
    pub distortion_coefficients: [f64; 8],   // [k1, k2, p1, p2, k3, k4, k5, k6]
    pub camera_matrix: [f64; 9],             // 3x3 intrinsic matrix [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    pub rectification_matrix: [f64; 9],      // 3x3 rectification matrix
    pub projection_matrix: [f64; 12],        // 3x4 projection matrix
    pub frame_id: [u8; 32],                  // Camera identifier
    pub timestamp: u64,                      // Nanoseconds since epoch
}
```

## Public API

### Construction

```rust
use horus_library::nodes::CameraNode;

// Create with default topic "camera/image"
let mut camera = CameraNode::new()?;

// Create with custom topic prefix
let mut left_camera = CameraNode::new_with_topic("camera/left")?;
// Publishes to "camera/left/image" and "camera/left/camera_info"
```

### Configuration Methods

```rust
// Set camera device (0 = default, 1+ = additional cameras)
camera.set_device_id(0);

// Set image resolution
camera.set_resolution(1920, 1080);

// Set capture framerate (clamped to 1.0 - 120.0 Hz)
camera.set_fps(60.0);

// Set image encoding format
use horus_library::vision::ImageEncoding;
camera.set_encoding(ImageEncoding::Rgb8);

// Enable compression (useful for network transmission)
camera.set_compression(true, 85);  // 85% quality

// Disable compression (raw images)
camera.set_compression(false, 90);
```

### Monitoring Methods

```rust
// Get current actual framerate
let fps = camera.get_actual_fps();
eprintln!("Camera running at {:.1} fps", fps);

// Get total frames captured since initialization
let frames = camera.get_frame_count();
eprintln!("Captured {} frames", frames);
```

## Usage Examples

### Single Camera Setup

```rust
use horus_library::nodes::CameraNode;
use horus_library::vision::ImageEncoding;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create and configure camera
    let mut camera = CameraNode::new()?;
    camera.set_device_id(0);
    camera.set_resolution(640, 480);
    camera.set_fps(30.0);
    camera.set_encoding(ImageEncoding::Bgr8);

    runtime.add_node(camera);
    runtime.run()?;

    Ok(())
}
```

### Stereo Camera Setup

```rust
use horus_library::nodes::CameraNode;
use horus_library::vision::ImageEncoding;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Left camera
    let mut left_camera = CameraNode::new_with_topic("camera/left")?;
    left_camera.set_device_id(0);
    left_camera.set_resolution(1280, 720);
    left_camera.set_fps(30.0);
    left_camera.set_encoding(ImageEncoding::Mono8);  // Grayscale for stereo

    // Right camera
    let mut right_camera = CameraNode::new_with_topic("camera/right")?;
    right_camera.set_device_id(1);
    right_camera.set_resolution(1280, 720);
    right_camera.set_fps(30.0);
    right_camera.set_encoding(ImageEncoding::Mono8);

    runtime.add_node(left_camera);
    runtime.add_node(right_camera);
    runtime.run()?;

    Ok(())
}
```

### High Resolution Setup

```rust
use horus_library::nodes::CameraNode;
use horus_library::vision::ImageEncoding;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // High-resolution camera for detailed capture
    let mut camera = CameraNode::new()?;
    camera.set_device_id(0);
    camera.set_resolution(1920, 1080);  // Full HD
    camera.set_fps(30.0);
    camera.set_encoding(ImageEncoding::Rgb8);

    // Enable compression to reduce bandwidth
    camera.set_compression(true, 90);

    runtime.add_node(camera);
    runtime.run()?;

    Ok(())
}
```

### Low Latency Setup

```rust
use horus_library::nodes::CameraNode;
use horus_library::vision::ImageEncoding;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Low resolution, high framerate for real-time control
    let mut camera = CameraNode::new()?;
    camera.set_device_id(0);
    camera.set_resolution(320, 240);    // Small resolution
    camera.set_fps(120.0);              // High framerate
    camera.set_encoding(ImageEncoding::Mono8);  // Fastest encoding
    camera.set_compression(false, 0);   // No compression overhead

    runtime.add_node(camera);
    runtime.run()?;

    Ok(())
}
```

### Depth Camera Setup

```rust
use horus_library::nodes::CameraNode;
use horus_library::vision::ImageEncoding;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // RGB camera
    let mut rgb_camera = CameraNode::new_with_topic("camera/rgb")?;
    rgb_camera.set_device_id(0);
    rgb_camera.set_resolution(640, 480);
    rgb_camera.set_encoding(ImageEncoding::Rgb8);

    // Depth camera
    let mut depth_camera = CameraNode::new_with_topic("camera/depth")?;
    depth_camera.set_device_id(1);
    depth_camera.set_resolution(640, 480);
    depth_camera.set_encoding(ImageEncoding::Depth16);  // 16-bit depth data

    runtime.add_node(rgb_camera);
    runtime.add_node(depth_camera);
    runtime.run()?;

    Ok(())
}
```

## Supported Formats and Resolutions

### Common Resolutions

| Resolution | Width x Height | Aspect Ratio | Use Case |
|------------|----------------|--------------|----------|
| QVGA | 320 x 240 | 4:3 | Low latency, embedded systems |
| VGA | 640 x 480 | 4:3 | Standard robotics vision |
| SVGA | 800 x 600 | 4:3 | Higher detail standard format |
| HD | 1280 x 720 | 16:9 | HD video, object detection |
| Full HD | 1920 x 1080 | 16:9 | High detail, surveillance |
| 2K | 2048 x 1080 | ~17:9 | Cinema-style capture |
| 4K | 3840 x 2160 | 16:9 | Ultra high detail (requires bandwidth) |

### Encoding Formats

| Encoding | Bytes/Pixel | Color | Use Case |
|----------|-------------|-------|----------|
| `Mono8` | 1 | No | Grayscale, stereo vision, fastest |
| `Mono16` | 2 | No | High-precision grayscale |
| `Rgb8` | 3 | Yes | Standard color, computer vision |
| `Bgr8` | 3 | Yes | OpenCV compatibility |
| `Rgba8` | 4 | Yes | Color with transparency |
| `Bgra8` | 4 | Yes | OpenCV with alpha channel |
| `Yuv422` | 2 | Yes | Video compression, streaming |
| `Mono32F` | 4 | No | HDR, scientific imaging |
| `Rgb32F` | 12 | Yes | HDR color processing |
| `BayerRggb8` | 1 | Yes | Raw sensor data, custom processing |
| `Depth16` | 2 | No | Depth cameras, 3D reconstruction |

### Bandwidth Considerations

| Resolution | Encoding | Uncompressed Size | @ 30 FPS Bandwidth |
|------------|----------|-------------------|--------------------|
| 320x240 | Mono8 | 76 KB/frame | 2.3 MB/s |
| 640x480 | Bgr8 | 900 KB/frame | 27 MB/s |
| 1280x720 | Rgb8 | 2.7 MB/frame | 81 MB/s |
| 1920x1080 | Rgb8 | 6.2 MB/frame | 186 MB/s |

**Note**: Enable compression for high-resolution setups to reduce bandwidth by 10-20x.

## Performance Considerations

### Framerate Guidelines

- **High-speed tracking**: 60-120 FPS, low resolution (320x240)
- **Standard robotics**: 30 FPS, medium resolution (640x480)
- **Object detection**: 15-30 FPS, high resolution (1920x1080)
- **Surveillance**: 10-15 FPS, high resolution

### CPU Usage

CPU usage scales with:
- **Resolution**: Higher resolution = more data to process
- **Framerate**: Linear scaling with FPS
- **Encoding conversion**: Color formats require more processing than grayscale
- **Compression**: JPEG compression adds 5-15% CPU overhead

**Optimization tips**:
```rust
// Use grayscale for algorithms that don't need color
camera.set_encoding(ImageEncoding::Mono8);

// Match resolution to actual needs (don't over-sample)
camera.set_resolution(640, 480);  // Instead of 1920x1080

// Use compression only when transmitting over network
camera.set_compression(false, 0);  // Disable for local processing
```

### Memory Usage

Per-frame memory = `width * height * bytes_per_pixel`

**Examples**:
- 640x480 Mono8: 307 KB
- 640x480 Bgr8: 921 KB
- 1920x1080 Rgb8: 6.2 MB

### Backend Selection

The node automatically selects available backends at compile time:

```toml
# Cargo.toml features
[features]
opencv-backend = ["opencv"]  # Full-featured, cross-platform
v4l2-backend = ["v4l"]       # Linux-native, lower overhead
```

**Performance**:
- OpenCV backend: Good compatibility, moderate overhead
- V4L2 backend: Best performance on Linux, native camera access
- Test pattern fallback: No camera required, for development/testing

## Troubleshooting

### Issue: Camera not found

**Symptoms**:
- Node starts but no frames are published
- "Failed to initialize camera" messages

**Causes**:
1. Wrong device ID
2. Camera not connected
3. Camera in use by another application

**Solutions**:
```rust
// Try different device IDs
camera.set_device_id(0);  // Usually default camera
camera.set_device_id(1);  // Secondary camera

// Check available cameras on Linux
// $ v4l2-ctl --list-devices

// Check available cameras on macOS
// $ system_profiler SPCameraDataType

// Kill processes using camera
// $ lsof /dev/video0
```

### Issue: Permission denied (Linux)

**Symptoms**:
- "Permission denied" when opening camera
- Camera accessible in other applications

**Solution**:
```bash
# Add user to video group
sudo usermod -a -G video $USER

# Or change device permissions (temporary)
sudo chmod 666 /dev/video0

# Then logout and login again
```

### Issue: Low framerate

**Symptoms**:
- `get_actual_fps()` returns lower than configured FPS
- Choppy video

**Causes**:
1. Insufficient USB bandwidth
2. CPU overload
3. Unsupported resolution/FPS combination

**Solutions**:
```rust
// Reduce resolution
camera.set_resolution(320, 240);

// Lower framerate
camera.set_fps(15.0);

// Use grayscale instead of color
camera.set_encoding(ImageEncoding::Mono8);

// Check actual FPS
let actual_fps = camera.get_actual_fps();
eprintln!("Running at {:.1} FPS", actual_fps);
```

### Issue: Wrong colors / inverted colors

**Symptoms**:
- Blue appears red, red appears blue
- Colors look incorrect

**Cause**:
Mismatch between RGB and BGR encoding

**Solution**:
```rust
// If colors are swapped, change encoding
camera.set_encoding(ImageEncoding::Bgr8);  // OpenCV default
// or
camera.set_encoding(ImageEncoding::Rgb8);  // Standard RGB
```

### Issue: Unsupported resolution

**Symptoms**:
- Camera initializes but provides different resolution than requested
- No frames captured

**Solution**:
```bash
# Check supported formats (Linux)
v4l2-ctl --device=/dev/video0 --list-formats-ext

# Common safe resolutions that work on most cameras:
# - 640x480 (VGA)
# - 320x240 (QVGA)
# - 1280x720 (HD)
```

```rust
// Use standard resolutions
camera.set_resolution(640, 480);   // VGA - widely supported
camera.set_resolution(1280, 720);  // HD - common on webcams
```

### Issue: High bandwidth usage

**Symptoms**:
- Network congestion
- High memory usage

**Solution**:
```rust
// Enable compression for network transmission
camera.set_compression(true, 85);

// Reduce resolution
camera.set_resolution(640, 480);

// Reduce framerate
camera.set_fps(15.0);

// Use grayscale if color not needed
camera.set_encoding(ImageEncoding::Mono8);
```

## Integration with Image Processing Nodes

### Object Detection Pipeline

```rust
use horus_library::nodes::CameraNode;
use horus_library::vision::ImageEncoding;
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Camera captures images
    let mut camera = CameraNode::new()?;
    camera.set_resolution(640, 480);
    camera.set_encoding(ImageEncoding::Bgr8);

    // Image processing node subscribes to "camera/image"
    // Detection results published to "detections"

    runtime.add_node(camera);
    // runtime.add_node(object_detector);
    runtime.run()?;

    Ok(())
}
```

### Image Rectification for Stereo Vision

```rust
use horus_library::nodes::CameraNode;
use horus_library::vision::ImageEncoding;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Left camera publishes to "camera/left/image" and "camera/left/camera_info"
    let mut left_camera = CameraNode::new_with_topic("camera/left")?;
    left_camera.set_device_id(0);
    left_camera.set_resolution(1280, 720);
    left_camera.set_encoding(ImageEncoding::Mono8);

    // Right camera publishes to "camera/right/image" and "camera/right/camera_info"
    let mut right_camera = CameraNode::new_with_topic("camera/right")?;
    right_camera.set_device_id(1);
    right_camera.set_resolution(1280, 720);
    right_camera.set_encoding(ImageEncoding::Mono8);

    // Stereo processing node uses camera_info for rectification
    // Subscribes to both image topics and camera_info topics

    runtime.add_node(left_camera);
    runtime.add_node(right_camera);
    // runtime.add_node(stereo_processor);
    runtime.run()?;

    Ok(())
}
```

### Image Filtering and Processing

```rust
use horus_library::nodes::CameraNode;
use horus_library::{Image, vision::ImageEncoding};
use horus_core::{Node, Runtime, Hub, NodeInfo};

// Example image subscriber node
struct ImageProcessor {
    subscriber: Hub<Image>,
}

impl ImageProcessor {
    fn new() -> horus_core::error::HorusResult<Self> {
        Ok(Self {
            subscriber: Hub::new("camera/image")?,
        })
    }
}

impl Node for ImageProcessor {
    fn name(&self) -> &'static str {
        "ImageProcessor"
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Receive images from camera
        if let Some(image) = self.subscriber.recv_latest() {
            if let Some(ctx) = ctx {
                ctx.log_debug(&format!("Received {}x{} image with {} bytes",
                         image.width, image.height, image.data.len()));
            }

            // Process image data here
            // - Apply filters (blur, edge detection)
            // - Color space conversion
            // - Feature extraction
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    let mut camera = CameraNode::new()?;
    camera.set_resolution(640, 480);
    camera.set_encoding(ImageEncoding::Bgr8);

    let processor = ImageProcessor::new()?;

    runtime.add_node(camera);
    runtime.add_node(processor);
    runtime.run()?;

    Ok(())
}
```

### Image Compression for Recording

```rust
use horus_library::nodes::CameraNode;
use horus_library::vision::ImageEncoding;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Camera with compression enabled for recording
    let mut camera = CameraNode::new()?;
    camera.set_resolution(1920, 1080);
    camera.set_fps(30.0);
    camera.set_encoding(ImageEncoding::Rgb8);
    camera.set_compression(true, 90);  // High quality JPEG compression

    // Logger node subscribes to compressed images
    // Writes to disk without additional processing

    runtime.add_node(camera);
    // runtime.add_node(image_logger);
    runtime.run()?;

    Ok(())
}
```

## Camera Calibration

The CameraInfo message provides intrinsic and extrinsic parameters for accurate 3D reconstruction:

```rust
// CameraInfo is published every 30 frames
// Contains:
// - camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
//   - fx, fy: focal lengths in pixels
//   - cx, cy: principal point (image center)
// - distortion_coefficients: [k1, k2, p1, p2, k3, ...]
//   - Radial and tangential distortion parameters
// - projection_matrix: For stereo rectification

// Default values (uncalibrated):
// fx = fy = 800.0
// cx = width / 2
// cy = height / 2
// No distortion
```

**To calibrate your camera**:
1. Use a checkerboard calibration pattern
2. Capture 20-30 images from different angles
3. Run camera calibration software (e.g., OpenCV calibration)
4. Update CameraInfo publisher with calibrated parameters

## Related Nodes

- **ImageProcessorNode**: Subscribe to camera images for filtering, enhancement
- **ObjectDetectionNode**: Visual object detection using camera feed
- **StereoVisionNode**: 3D reconstruction from stereo camera pair
- **ImageLoggerNode**: Record camera images to disk
- **CompressedImageTransport**: Network transmission of compressed images

## See Also

- [Image Message Format](../messages/vision.md)
- [Camera Calibration Guide](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html)
- [V4L2 Camera Control](https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/v4l2.html)
- [OpenCV VideoCapture](https://docs.opencv.org/master/d8/dfe/classcv_1_1VideoCapture.html)
