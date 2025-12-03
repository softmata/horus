# Image Processor Node

Computer vision preprocessing and filtering for object detection, feature extraction, and visual analysis with OpenCV-style operations.

## Overview

The Image Processor Node provides image processing operations for computer vision applications. It supports resizing, filtering, color space conversion, edge detection, and brightness/contrast adjustments to prepare images for object detection, feature extraction, or visualization.

Supports RGB8, BGR8, RGBA8, BGRA8, Mono8, Mono16, YUV422, and other standard image formats.

Key features:
- Image resizing with configurable dimensions
- Grayscale conversion
- Gaussian blur filtering (noise reduction)
- Edge detection (Canny, Sobel)
- Brightness and contrast adjustment
- Color space conversion
- Real-time processing statistics
- Pipeline-based processing architecture

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `camera.image` | `Image` | Input images from camera or other sources |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `camera.processed` | `Image` | Processed images after applying filters and transformations |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `target_width` | `u32` | `640` | Target width for resize operation (pixels) |
| `target_height` | `u32` | `480` | Target height for resize operation (pixels) |
| `resize_enabled` | `bool` | `false` | Enable image resizing |
| `grayscale_enabled` | `bool` | `false` | Enable grayscale conversion |
| `gaussian_blur_size` | `u32` | `0` | Gaussian blur kernel size (must be odd: 3, 5, 7, etc.) |
| `edge_detection_enabled` | `bool` | `false` | Enable edge detection processing |
| `brightness_adjustment` | `f32` | `0.0` | Brightness adjustment (-1.0 to 1.0) |
| `contrast_adjustment` | `f32` | `1.0` | Contrast adjustment (0.5 to 2.0) |

## Message Types

### Image

Raw image data message:

```rust
pub struct Image {
    pub width: u32,              // Image width in pixels
    pub height: u32,             // Image height in pixels
    pub encoding: ImageEncoding, // Pixel encoding format
    pub step: u32,               // Bytes per row (may include padding)
    pub data: Vec<u8>,          // Image data (row-major order)
    pub frame_id: [u8; 32],     // Frame ID (camera identifier)
    pub timestamp: u64,          // Timestamp (ns since epoch)
}
```

### ImageEncoding

Image encoding formats:

```rust
pub enum ImageEncoding {
    Mono8 = 0,         // 8-bit monochrome
    Mono16 = 1,        // 16-bit monochrome
    Rgb8 = 2,          // 8-bit RGB (3 channels)
    Bgr8 = 3,          // 8-bit BGR (3 channels, OpenCV format)
    Rgba8 = 4,         // 8-bit RGBA (4 channels)
    Bgra8 = 5,         // 8-bit BGRA (4 channels)
    Yuv422 = 6,        // YUV 4:2:2 format
    Mono32F = 7,       // 32-bit float monochrome
    Rgb32F = 8,        // 32-bit float RGB
    BayerRggb8 = 9,    // Bayer pattern (raw sensor data)
    Depth16 = 10,      // 16-bit depth image (millimeters)
}
```

### Image Methods

```rust
// Create new image
let image = Image::new(width, height, encoding, data);

// Set frame ID
let image = image.with_frame_id("camera_front");

// Validate image
if image.is_valid() {
    println!("Valid image");
}

// Get pixel at coordinates
if let Some(pixel) = image.get_pixel(x, y) {
    println!("Pixel: {:?}", pixel);
}

// Create region of interest (crop)
if let Some(roi) = image.roi(x, y, width, height) {
    println!("Cropped: {}x{}", roi.width, roi.height);
}

// Get expected data size
let size = image.expected_size();
```

### ImageEncoding Methods

```rust
// Get bytes per pixel
let bpp = encoding.bytes_per_pixel();

// Check if encoding has color
if encoding.is_color() {
    println!("Color image");
}
```

## Public API

### Construction

```rust
use horus_library::nodes::ImageProcessorNode;

// Create with default topics "camera.image" -> "camera.processed"
let mut processor = ImageProcessorNode::new()?;

// Create with custom input/output topics
let mut processor = ImageProcessorNode::new_with_topics(
    "camera.raw",
    "vision.processed"
)?;
```

### Configuration Methods

```rust
// Enable/disable resizing
processor.enable_resize(320, 240);  // width, height
processor.disable_resize();

// Enable/disable grayscale conversion
processor.enable_grayscale();
processor.disable_grayscale();

// Enable Gaussian blur (kernel size must be odd: 3, 5, 7, etc.)
processor.enable_gaussian_blur(5);  // 5x5 kernel
processor.disable_gaussian_blur();

// Enable/disable edge detection
processor.enable_edge_detection();
processor.disable_edge_detection();

// Adjust brightness (-1.0 to 1.0)
processor.set_brightness(0.2);   // Increase brightness by 20%
processor.set_brightness(-0.3);  // Decrease brightness by 30%

// Adjust contrast (0.5 to 2.0)
processor.set_contrast(1.5);  // Increase contrast by 50%
processor.set_contrast(0.8);  // Decrease contrast by 20%
```

### Query Methods

```rust
// Get processing statistics (images_processed, processing_time_us)
let (count, time_us) = processor.get_stats();
println!("Processed {} images, last took {} μs", count, time_us);
```

## Usage Examples

### Basic Image Resizing

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create image processor
    let mut processor = ImageProcessorNode::new()?;

    // Configure resizing
    processor.enable_resize(320, 240);  // Resize to 320x240

    scheduler.add(Box::new(processor), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Grayscale Conversion with Blur

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut processor = ImageProcessorNode::new()?;

    // Convert to grayscale and apply blur
    processor.enable_grayscale();
    processor.enable_gaussian_blur(5);  // 5x5 Gaussian blur

    scheduler.add(Box::new(processor), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Edge Detection Pipeline

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut processor = ImageProcessorNode::new_with_topics(
        "camera.image",
        "vision.edges"
    )?;

    // Prepare for edge detection
    processor.enable_grayscale();        // Convert to grayscale
    processor.enable_gaussian_blur(3);   // Reduce noise
    processor.enable_edge_detection();   // Detect edges

    scheduler.add(Box::new(processor), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Brightness and Contrast Enhancement

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut processor = ImageProcessorNode::new()?;

    // Enhance dark images
    processor.set_brightness(0.3);   // Increase brightness by 30%
    processor.set_contrast(1.2);     // Increase contrast by 20%

    scheduler.add(Box::new(processor), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Multi-Stage Processing

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut processor = ImageProcessorNode::new()?;

    // Complete processing pipeline
    processor.enable_resize(640, 480);       // Standardize size
    processor.set_brightness(0.1);           // Slight brightness boost
    processor.set_contrast(1.1);             // Slight contrast boost
    processor.enable_gaussian_blur(3);       // Reduce noise
    processor.enable_grayscale();            // Convert to grayscale

    scheduler.add(Box::new(processor), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Object Detection Preprocessing

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Preprocess images for object detection
    let mut preprocessor = ImageProcessorNode::new_with_topics(
        "camera.raw",
        "vision.preprocessed"
    )?;

    preprocessor.enable_resize(416, 416);  // YOLO input size
    preprocessor.set_brightness(0.05);
    preprocessor.set_contrast(1.05);

    scheduler.add(Box::new(preprocessor), 1, Some(true));

    // Consumer node
    let detector = node! {
        name: "object_detector",
        tick: |ctx| {
            let hub = Hub::<Image>::new("vision.preprocessed")?;

            while let Some(image) = hub.recv(None) {
                ctx.log_info(&format!(
                    "Processing {}x{} image, {} bytes",
                    image.width, image.height, image.data.len()
                ));

                // Object detection logic here
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(detector), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Quality Monitoring

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut processor = ImageProcessorNode::new()?;
    processor.enable_resize(640, 480);
    processor.enable_grayscale();

    scheduler.add(Box::new(processor), 1, Some(true));

    // Monitor processing performance
    let monitor = node! {
        name: "performance_monitor",
        tick: |ctx| {
            let hub = Hub::<Image>::new("camera.processed")?;

            while let Some(image) = hub.recv(None) {
                if !image.is_valid() {
                    ctx.log_warning("Invalid image received!");
                } else {
                    ctx.log_info(&format!(
                        "Valid {}x{} {:?} image ({} bytes)",
                        image.width, image.height,
                        image.encoding, image.data.len()
                    ));
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(monitor), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Region of Interest Extraction

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut processor = ImageProcessorNode::new()?;
    processor.enable_grayscale();

    scheduler.add(Box::new(processor), 1, Some(true));

    // Extract and process ROI
    let roi_extractor = node! {
        name: "roi_extractor",
        tick: |ctx| {
            let hub = Hub::<Image>::new("camera.processed")?;

            while let Some(image) = hub.recv(None) {
                // Extract center region
                let roi_width = image.width / 2;
                let roi_height = image.height / 2;
                let roi_x = (image.width - roi_width) / 2;
                let roi_y = (image.height - roi_height) / 2;

                if let Some(roi) = image.roi(roi_x, roi_y, roi_width, roi_height) {
                    ctx.log_info(&format!(
                        "ROI extracted: {}x{} from {}x{}",
                        roi.width, roi.height,
                        image.width, image.height
                    ));

                    // Process ROI further
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(roi_extractor), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

## Processing Pipeline

The Image Processor Node applies operations in the following order:

1. **Resize**: Scale image to target dimensions
2. **Grayscale**: Convert color image to grayscale
3. **Gaussian Blur**: Apply blur filter for noise reduction
4. **Brightness/Contrast**: Adjust pixel intensity values
5. **Edge Detection**: Detect edges using Canny or Sobel

Each stage is optional and can be enabled/disabled independently.

### Pipeline Example

```
Input: 1920x1080 RGB8 image
  |
  v
Resize: 640x480 RGB8
  |
  v
Grayscale: 640x480 Mono8
  |
  v
Gaussian Blur: 640x480 Mono8 (smoothed)
  |
  v
Brightness/Contrast: 640x480 Mono8 (enhanced)
  |
  v
Edge Detection: 640x480 Mono8 (edges only)
  |
  v
Output: 640x480 Mono8 processed image
```

## Best Practices

1. **Order processing operations efficiently**:
   ```rust
   // Good: Resize first to reduce processing time
   processor.enable_resize(320, 240);
   processor.enable_gaussian_blur(5);
   processor.enable_edge_detection();
   ```

2. **Use appropriate blur kernel size**:
   ```rust
   // Small images: use smaller kernels
   processor.enable_gaussian_blur(3);

   // Large images: use larger kernels
   processor.enable_gaussian_blur(7);
   ```

3. **Convert to grayscale when color not needed**:
   ```rust
   // Edge detection works best on grayscale
   processor.enable_grayscale();
   processor.enable_edge_detection();
   ```

4. **Validate images before processing**:
   ```rust
   if image.is_valid() {
       // Process image
   } else {
       ctx.log_warning("Invalid image format");
   }
   ```

5. **Monitor processing statistics**:
   ```rust
   let (count, time_us) = processor.get_stats();
   if time_us > 50000 {  // More than 50ms
       ctx.log_warning("Processing too slow!");
   }
   ```

6. **Use appropriate brightness/contrast values**:
   ```rust
   // Subtle adjustments usually work best
   processor.set_brightness(0.1);   // +10%
   processor.set_contrast(1.1);     // +10%

   // Avoid extreme values
   processor.set_brightness(-0.9);  // Too dark
   processor.set_contrast(2.0);     // Too high
   ```

7. **Choose correct encoding for your use case**:
   ```rust
   // Computer vision: BGR8 (OpenCV format)
   let image = Image::new(width, height, ImageEncoding::Bgr8, data);

   // Display: RGB8 (standard format)
   let image = Image::new(width, height, ImageEncoding::Rgb8, data);

   // Memory efficiency: Mono8 (grayscale)
   let image = Image::new(width, height, ImageEncoding::Mono8, data);
   ```

## Performance Considerations

### Memory Usage

Different encodings have different memory requirements:

| Encoding | Bytes per Pixel | 640x480 Image Size |
|----------|----------------|-------------------|
| Mono8    | 1              | 307 KB            |
| Mono16   | 2              | 614 KB            |
| RGB8/BGR8 | 3             | 921 KB            |
| RGBA8/BGRA8 | 4           | 1.23 MB           |
| Rgb32F   | 12             | 3.69 MB           |

### Processing Time

Typical processing times (640x480 image on Raspberry Pi 4):

| Operation | Time |
|-----------|------|
| Resize | 5-15 ms |
| Grayscale conversion | 2-5 ms |
| Gaussian blur (3x3) | 3-8 ms |
| Gaussian blur (5x5) | 5-15 ms |
| Edge detection | 10-25 ms |
| Brightness/contrast | 2-5 ms |

### Optimization Tips

1. **Resize early in pipeline**:
   ```rust
   // Good: Reduces data for subsequent operations
   processor.enable_resize(320, 240);
   processor.enable_gaussian_blur(5);
   ```

2. **Minimize memory allocations**:
   ```rust
   // Process in-place when possible
   processor.enable_grayscale();  // Reduces from 3 to 1 channel
   ```

3. **Use smaller kernel sizes**:
   ```rust
   // 3x3 is much faster than 7x7
   processor.enable_gaussian_blur(3);
   ```

## Troubleshooting

### Images not being processed

**Symptoms:**
```
[INFO] ImageProcessorNode: 0 images processed
```

**Solutions:**
1. Check input topic matches camera output
2. Verify camera node is running
3. Check scheduler execution order
4. Enable debug logging

### Invalid image format errors

**Symptoms:**
```
[WARN] Invalid image received: expected size mismatch
```

**Solutions:**
1. Validate image before sending:
   ```rust
   if image.is_valid() {
       publisher.send(image, &mut None)?;
   }
   ```
2. Check encoding matches actual data
3. Verify step size is correct
4. Ensure data vector has correct size

### Processing too slow

**Symptoms:**
```
[WARN] Image processed in 150000 μs (150ms)
```

**Solutions:**
1. Reduce image size first:
   ```rust
   processor.enable_resize(320, 240);
   ```
2. Use smaller blur kernels:
   ```rust
   processor.enable_gaussian_blur(3);  // Not 7
   ```
3. Disable unnecessary operations
4. Process on separate thread/core

### Memory usage too high

**Solutions:**
1. Use grayscale instead of color:
   ```rust
   processor.enable_grayscale();  // 1/3 memory
   ```
2. Resize to smaller dimensions
3. Use Mono8 instead of Mono16
4. Clear old image data promptly

### Blur kernel size error

**Symptoms:**
```
[WARN] Gaussian blur kernel adjusted: 4 -> 5
```

**Explanation:**
Kernel size must be odd (3, 5, 7, etc.). The node automatically adjusts even values.

**Solution:**
```rust
// Use odd kernel sizes
processor.enable_gaussian_blur(3);   // Good
processor.enable_gaussian_blur(5);   // Good
processor.enable_gaussian_blur(4);   // Auto-adjusted to 5
```

### Brightness/contrast clamping

**Symptoms:**
```
[DEBUG] Brightness clamped: 1.5 -> 1.0
[DEBUG] Contrast clamped: 3.0 -> 2.0
```

**Explanation:**
Values are automatically clamped to valid ranges to prevent overflow.

**Solution:**
```rust
// Use valid ranges
processor.set_brightness(0.5);   // Valid: -1.0 to 1.0
processor.set_contrast(1.5);     // Valid: 0.5 to 2.0

// These get clamped
processor.set_brightness(2.0);   // Clamped to 1.0
processor.set_contrast(3.0);     // Clamped to 2.0
```

## Common Image Operations

### Creating Images

```rust
use horus_library::messages::vision::{Image, ImageEncoding};

// RGB image
let rgb_data = vec![255, 0, 0, 0, 255, 0, 0, 0, 255]; // Red, Green, Blue pixels
let image = Image::new(3, 1, ImageEncoding::Rgb8, rgb_data)
    .with_frame_id("camera_front");

// Grayscale image
let gray_data = vec![128; 640 * 480]; // 640x480 gray image
let image = Image::new(640, 480, ImageEncoding::Mono8, gray_data);

// Depth image
let depth_data = vec![0u8; 640 * 480 * 2]; // 16-bit depth
let image = Image::new(640, 480, ImageEncoding::Depth16, depth_data);
```

### Accessing Pixels

```rust
// Get pixel value
if let Some(pixel) = image.get_pixel(100, 50) {
    match image.encoding {
        ImageEncoding::Rgb8 => {
            println!("R={}, G={}, B={}", pixel[0], pixel[1], pixel[2]);
        }
        ImageEncoding::Mono8 => {
            println!("Gray={}", pixel[0]);
        }
        _ => {}
    }
}
```

### Cropping Images

```rust
// Extract center 200x200 region
let width = 200;
let height = 200;
let x = (image.width - width) / 2;
let y = (image.height - height) / 2;

if let Some(cropped) = image.roi(x, y, width, height) {
    println!("Cropped to {}x{}", cropped.width, cropped.height);
}
```

### Image Validation

```rust
if !image.is_valid() {
    println!("Invalid image:");
    println!("  Size: {}x{}", image.width, image.height);
    println!("  Expected: {} bytes", image.expected_size());
    println!("  Actual: {} bytes", image.data.len());
    return;
}
```

## Integration Examples

### With Camera Node

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Camera captures images
    let camera = CameraNode::new()?;
    scheduler.add(Box::new(camera), 1, Some(true));

    // Processor enhances images
    let mut processor = ImageProcessorNode::new()?;
    processor.enable_resize(640, 480);
    processor.set_brightness(0.1);
    processor.set_contrast(1.1);
    scheduler.add(Box::new(processor), 2, Some(true));

    scheduler.run()?;
    Ok(())
}
```

### With Object Detection

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Preprocess for object detection
    let mut preprocessor = ImageProcessorNode::new_with_topics(
        "camera.raw",
        "vision.preprocessed"
    )?;
    preprocessor.enable_resize(416, 416);  // YOLO size
    preprocessor.enable_grayscale();
    scheduler.add(Box::new(preprocessor), 1, Some(true));

    // Object detection node (simplified)
    let detector = node! {
        name: "yolo_detector",
        tick: |ctx| {
            let hub = Hub::<Image>::new("vision.preprocessed")?;
            while let Some(image) = hub.recv(None) {
                // Run YOLO inference
                ctx.log_info("Running object detection...");
            }
            Ok(())
        }
    };
    scheduler.add(Box::new(detector), 2, Some(true));

    scheduler.run()?;
    Ok(())
}
```

### With Edge Detection Analysis

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Edge detection preprocessing
    let mut edge_processor = ImageProcessorNode::new_with_topics(
        "camera.image",
        "vision.edges"
    )?;
    edge_processor.enable_grayscale();
    edge_processor.enable_gaussian_blur(5);
    edge_processor.enable_edge_detection();
    scheduler.add(Box::new(edge_processor), 1, Some(true));

    // Analyze edges
    let analyzer = node! {
        name: "edge_analyzer",
        tick: |ctx| {
            let hub = Hub::<Image>::new("vision.edges")?;
            while let Some(edges) = hub.recv(None) {
                // Count edge pixels or analyze contours
                ctx.log_info("Analyzing edge image...");
            }
            Ok(())
        }
    };
    scheduler.add(Box::new(analyzer), 2, Some(true));

    scheduler.run()?;
    Ok(())
}
```

## See Also

- [CameraNode](../camera/) - Image capture from cameras
- [DepthCameraNode](../depth_camera/) - 3D depth imaging
- [ObjectDetectorNode](../object_detector/) - YOLO/CNN object detection
- [ArucoDetectorNode](../aruco_detector/) - ArUco marker detection
- [QRCodeReaderNode](../qr_code_reader/) - QR code detection
- [FeatureExtractorNode](../feature_extractor/) - SIFT/ORB/AKAZE features
