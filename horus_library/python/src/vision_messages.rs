// Python wrappers for vision messages
use horus_library::messages::vision;
use pyo3::prelude::*;

/// Image encoding enum
#[pyclass(module = "horus.library._library", name = "ImageEncoding")]
#[derive(Clone, Copy)]
pub struct PyImageEncoding {
    pub(crate) inner: vision::ImageEncoding,
}

#[pymethods]
impl PyImageEncoding {
    #[classattr]
    const MONO8: u8 = 0;
    #[classattr]
    const MONO16: u8 = 1;
    #[classattr]
    const RGB8: u8 = 2;
    #[classattr]
    const BGR8: u8 = 3;
    #[classattr]
    const RGBA8: u8 = 4;
    #[classattr]
    const BGRA8: u8 = 5;
    #[classattr]
    const YUV422: u8 = 6;
    #[classattr]
    const MONO32F: u8 = 7;
    #[classattr]
    const RGB32F: u8 = 8;
    #[classattr]
    const BAYER_RGGB8: u8 = 9;
    #[classattr]
    const DEPTH16: u8 = 10;

    #[new]
    #[pyo3(signature = (encoding=2))]
    fn new(encoding: u8) -> Self {
        let inner = match encoding {
            0 => vision::ImageEncoding::Mono8,
            1 => vision::ImageEncoding::Mono16,
            2 => vision::ImageEncoding::Rgb8,
            3 => vision::ImageEncoding::Bgr8,
            4 => vision::ImageEncoding::Rgba8,
            5 => vision::ImageEncoding::Bgra8,
            6 => vision::ImageEncoding::Yuv422,
            7 => vision::ImageEncoding::Mono32F,
            8 => vision::ImageEncoding::Rgb32F,
            9 => vision::ImageEncoding::BayerRggb8,
            10 => vision::ImageEncoding::Depth16,
            _ => vision::ImageEncoding::Rgb8,
        };
        Self { inner }
    }

    fn bytes_per_pixel(&self) -> u32 {
        self.inner.bytes_per_pixel()
    }

    fn is_color(&self) -> bool {
        self.inner.is_color()
    }

    fn __repr__(&self) -> String {
        format!("{:?}", self.inner)
    }
}

/// Python wrapper for Image
#[pyclass(module = "horus.library._library", name = "Image")]
#[derive(Clone)]
pub struct PyImage {
    pub(crate) inner: vision::Image,
}

#[pymethods]
impl PyImage {
    #[new]
    #[pyo3(signature = (width=0, height=0, encoding=None, data=None))]
    fn new(
        width: u32,
        height: u32,
        encoding: Option<&PyImageEncoding>,
        data: Option<Vec<u8>>,
    ) -> Self {
        let enc = encoding
            .map(|e| e.inner)
            .unwrap_or(vision::ImageEncoding::Rgb8);
        let image_data = data.unwrap_or_default();
        Self {
            inner: vision::Image::new(width, height, enc, image_data),
        }
    }

    #[getter]
    fn width(&self) -> u32 {
        self.inner.width
    }

    #[setter]
    fn set_width(&mut self, value: u32) {
        self.inner.width = value;
    }

    #[getter]
    fn height(&self) -> u32 {
        self.inner.height
    }

    #[setter]
    fn set_height(&mut self, value: u32) {
        self.inner.height = value;
    }

    #[getter]
    fn step(&self) -> u32 {
        self.inner.step
    }

    #[getter]
    fn data(&self) -> Vec<u8> {
        self.inner.data.clone()
    }

    #[setter]
    fn set_data(&mut self, value: Vec<u8>) {
        self.inner.data = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn expected_size(&self) -> usize {
        self.inner.expected_size()
    }

    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    fn __repr__(&self) -> String {
        format!(
            "Image({}x{}, {:?}, {} bytes)",
            self.inner.width,
            self.inner.height,
            self.inner.encoding,
            self.inner.data.len()
        )
    }
}

/// Python wrapper for CompressedImage
#[pyclass(module = "horus.library._library", name = "CompressedImage")]
#[derive(Clone)]
pub struct PyCompressedImage {
    pub(crate) inner: vision::CompressedImage,
}

#[pymethods]
impl PyCompressedImage {
    #[new]
    #[pyo3(signature = (format="jpeg", data=None))]
    fn new(format: &str, data: Option<Vec<u8>>) -> Self {
        Self {
            inner: vision::CompressedImage::new(format, data.unwrap_or_default()),
        }
    }

    #[getter]
    fn format(&self) -> String {
        self.inner.format_str()
    }

    #[getter]
    fn data(&self) -> Vec<u8> {
        self.inner.data.clone()
    }

    #[setter]
    fn set_data(&mut self, value: Vec<u8>) {
        self.inner.data = value;
    }

    #[getter]
    fn width(&self) -> u32 {
        self.inner.width
    }

    #[setter]
    fn set_width(&mut self, value: u32) {
        self.inner.width = value;
    }

    #[getter]
    fn height(&self) -> u32 {
        self.inner.height
    }

    #[setter]
    fn set_height(&mut self, value: u32) {
        self.inner.height = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn __repr__(&self) -> String {
        format!(
            "CompressedImage({}, {}x{}, {} bytes)",
            self.inner.format_str(),
            self.inner.width,
            self.inner.height,
            self.inner.data.len()
        )
    }
}

/// Python wrapper for CameraInfo
#[pyclass(module = "horus.library._library", name = "CameraInfo")]
#[derive(Clone)]
pub struct PyCameraInfo {
    pub(crate) inner: vision::CameraInfo,
}

#[pymethods]
impl PyCameraInfo {
    #[new]
    #[pyo3(signature = (width=640, height=480, fx=525.0, fy=525.0, cx=320.0, cy=240.0))]
    fn new(width: u32, height: u32, fx: f64, fy: f64, cx: f64, cy: f64) -> Self {
        Self {
            inner: vision::CameraInfo::new(width, height, fx, fy, cx, cy),
        }
    }

    #[getter]
    fn width(&self) -> u32 {
        self.inner.width
    }

    #[getter]
    fn height(&self) -> u32 {
        self.inner.height
    }

    #[getter]
    fn camera_matrix(&self) -> [f64; 9] {
        self.inner.camera_matrix
    }

    #[getter]
    fn distortion_coefficients(&self) -> [f64; 8] {
        self.inner.distortion_coefficients
    }

    #[setter]
    fn set_distortion_coefficients(&mut self, value: [f64; 8]) {
        self.inner.distortion_coefficients = value;
    }

    #[getter]
    fn projection_matrix(&self) -> [f64; 12] {
        self.inner.projection_matrix
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn focal_lengths(&self) -> (f64, f64) {
        self.inner.focal_lengths()
    }

    fn principal_point(&self) -> (f64, f64) {
        self.inner.principal_point()
    }

    fn __repr__(&self) -> String {
        let (fx, fy) = self.inner.focal_lengths();
        format!(
            "CameraInfo({}x{}, f={:.1}/{:.1})",
            self.inner.width, self.inner.height, fx, fy
        )
    }
}

/// Python wrapper for RegionOfInterest
#[pyclass(module = "horus.library._library", name = "RegionOfInterest")]
#[derive(Clone, Copy)]
pub struct PyRegionOfInterest {
    pub(crate) inner: vision::RegionOfInterest,
}

#[pymethods]
impl PyRegionOfInterest {
    #[new]
    #[pyo3(signature = (x=0, y=0, width=0, height=0))]
    fn new(x: u32, y: u32, width: u32, height: u32) -> Self {
        Self {
            inner: vision::RegionOfInterest::new(x, y, width, height),
        }
    }

    #[getter]
    fn x_offset(&self) -> u32 {
        self.inner.x_offset
    }

    #[setter]
    fn set_x_offset(&mut self, value: u32) {
        self.inner.x_offset = value;
    }

    #[getter]
    fn y_offset(&self) -> u32 {
        self.inner.y_offset
    }

    #[setter]
    fn set_y_offset(&mut self, value: u32) {
        self.inner.y_offset = value;
    }

    #[getter]
    fn width(&self) -> u32 {
        self.inner.width
    }

    #[setter]
    fn set_width(&mut self, value: u32) {
        self.inner.width = value;
    }

    #[getter]
    fn height(&self) -> u32 {
        self.inner.height
    }

    #[setter]
    fn set_height(&mut self, value: u32) {
        self.inner.height = value;
    }

    fn contains(&self, x: u32, y: u32) -> bool {
        self.inner.contains(x, y)
    }

    fn area(&self) -> u32 {
        self.inner.area()
    }

    fn __repr__(&self) -> String {
        format!(
            "RegionOfInterest(x={}, y={}, {}x{})",
            self.inner.x_offset, self.inner.y_offset, self.inner.width, self.inner.height
        )
    }
}

/// Python wrapper for Detection
#[pyclass(module = "horus.library._library", name = "Detection")]
#[derive(Clone)]
pub struct PyDetection {
    pub(crate) inner: vision::Detection,
}

#[pymethods]
impl PyDetection {
    #[new]
    #[pyo3(signature = (class_name="unknown", confidence=0.0, bbox=None))]
    fn new(class_name: &str, confidence: f32, bbox: Option<&PyRegionOfInterest>) -> Self {
        let roi = bbox.map(|b| b.inner).unwrap_or_default();
        Self {
            inner: vision::Detection::new(class_name, confidence, roi),
        }
    }

    #[getter]
    fn class_name(&self) -> String {
        self.inner.class_str()
    }

    #[getter]
    fn confidence(&self) -> f32 {
        self.inner.confidence
    }

    #[setter]
    fn set_confidence(&mut self, value: f32) {
        self.inner.confidence = value;
    }

    #[getter]
    fn bbox(&self) -> PyRegionOfInterest {
        PyRegionOfInterest {
            inner: self.inner.bbox,
        }
    }

    #[setter]
    fn set_bbox(&mut self, value: &PyRegionOfInterest) {
        self.inner.bbox = value.inner;
    }

    #[getter]
    fn track_id(&self) -> u32 {
        self.inner.track_id
    }

    #[setter]
    fn set_track_id(&mut self, value: u32) {
        self.inner.track_id = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn __repr__(&self) -> String {
        format!(
            "Detection('{}', conf={:.2}, bbox={}x{})",
            self.inner.class_str(),
            self.inner.confidence,
            self.inner.bbox.width,
            self.inner.bbox.height
        )
    }
}

/// Python wrapper for DetectionArray
#[pyclass(module = "horus.library._library", name = "DetectionArray")]
#[derive(Clone)]
pub struct PyDetectionArray {
    pub(crate) inner: vision::DetectionArray,
}

#[pymethods]
impl PyDetectionArray {
    #[new]
    fn new() -> Self {
        Self {
            inner: vision::DetectionArray::new(),
        }
    }

    #[getter]
    fn count(&self) -> u8 {
        self.inner.count
    }

    #[getter]
    fn image_width(&self) -> u32 {
        self.inner.image_width
    }

    #[setter]
    fn set_image_width(&mut self, value: u32) {
        self.inner.image_width = value;
    }

    #[getter]
    fn image_height(&self) -> u32 {
        self.inner.image_height
    }

    #[setter]
    fn set_image_height(&mut self, value: u32) {
        self.inner.image_height = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn add_detection(&mut self, detection: &PyDetection) -> PyResult<()> {
        self.inner
            .add_detection(detection.inner)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e))
    }

    fn get_detections(&self) -> Vec<PyDetection> {
        self.inner
            .get_detections()
            .iter()
            .map(|d| PyDetection { inner: *d })
            .collect()
    }

    fn filter_by_confidence(&self, threshold: f32) -> Vec<PyDetection> {
        self.inner
            .filter_by_confidence(threshold)
            .into_iter()
            .map(|d| PyDetection { inner: d })
            .collect()
    }

    fn __len__(&self) -> usize {
        self.inner.count as usize
    }

    fn __repr__(&self) -> String {
        format!("DetectionArray({} detections)", self.inner.count)
    }
}

/// Python wrapper for StereoInfo
#[pyclass(module = "horus.library._library", name = "StereoInfo")]
#[derive(Clone)]
pub struct PyStereoInfo {
    pub(crate) inner: vision::StereoInfo,
}

#[pymethods]
impl PyStereoInfo {
    #[new]
    fn new() -> Self {
        Self {
            inner: vision::StereoInfo::default(),
        }
    }

    #[getter]
    fn left_camera(&self) -> PyCameraInfo {
        PyCameraInfo {
            inner: self.inner.left_camera,
        }
    }

    #[setter]
    fn set_left_camera(&mut self, value: &PyCameraInfo) {
        self.inner.left_camera = value.inner;
    }

    #[getter]
    fn right_camera(&self) -> PyCameraInfo {
        PyCameraInfo {
            inner: self.inner.right_camera,
        }
    }

    #[setter]
    fn set_right_camera(&mut self, value: &PyCameraInfo) {
        self.inner.right_camera = value.inner;
    }

    #[getter]
    fn baseline(&self) -> f64 {
        self.inner.baseline
    }

    #[setter]
    fn set_baseline(&mut self, value: f64) {
        self.inner.baseline = value;
    }

    fn depth_from_disparity(&self, disparity: f32) -> f32 {
        self.inner.depth_from_disparity(disparity)
    }

    fn disparity_from_depth(&self, depth: f32) -> f32 {
        self.inner.disparity_from_depth(depth)
    }

    fn __repr__(&self) -> String {
        format!(
            "StereoInfo({}x{}, baseline={:.3}m)",
            self.inner.left_camera.width, self.inner.left_camera.height, self.inner.baseline
        )
    }
}
