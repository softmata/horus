//! horus.perception - Python bindings for perception types
//!
//! This module provides Python-friendly wrappers for HORUS perception types:
//! - Detection / Detection3D - Object detection results
//! - PointXYZ / PointXYZRGB / PointXYZI - Point cloud points
//! - Landmark / Landmark3D - Pose estimation keypoints
//! - SegmentationMask - Semantic/instance segmentation
//! - TrackedObject - Multi-object tracking

use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyDict, PyList};

// ============================================
// BoundingBox2D
// ============================================

/// 2D bounding box (x, y, width, height)
#[pyclass(name = "BoundingBox2D")]
#[derive(Debug, Clone)]
pub struct PyBoundingBox2D {
    pub x: f32,
    pub y: f32,
    pub width: f32,
    pub height: f32,
}

#[pymethods]
impl PyBoundingBox2D {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, width=0.0, height=0.0))]
    fn new(x: f32, y: f32, width: f32, height: f32) -> Self {
        Self {
            x,
            y,
            width,
            height,
        }
    }

    /// Create from center coordinates (YOLO format)
    #[staticmethod]
    fn from_center(cx: f32, cy: f32, width: f32, height: f32) -> Self {
        Self {
            x: cx - width / 2.0,
            y: cy - height / 2.0,
            width,
            height,
        }
    }

    /// Get center x coordinate
    #[getter]
    fn center_x(&self) -> f32 {
        self.x + self.width / 2.0
    }

    /// Get center y coordinate
    #[getter]
    fn center_y(&self) -> f32 {
        self.y + self.height / 2.0
    }

    /// Get area
    #[getter]
    fn area(&self) -> f32 {
        self.width * self.height
    }

    /// Calculate IoU with another box
    fn iou(&self, other: &PyBoundingBox2D) -> f32 {
        let x1 = self.x.max(other.x);
        let y1 = self.y.max(other.y);
        let x2 = (self.x + self.width).min(other.x + other.width);
        let y2 = (self.y + self.height).min(other.y + other.height);

        let intersection = (x2 - x1).max(0.0) * (y2 - y1).max(0.0);
        let union = self.area() + other.area() - intersection;

        if union > 0.0 {
            intersection / union
        } else {
            0.0
        }
    }

    #[getter]
    fn x(&self) -> f32 {
        self.x
    }

    #[getter]
    fn y(&self) -> f32 {
        self.y
    }

    #[getter]
    fn width(&self) -> f32 {
        self.width
    }

    #[getter]
    fn height(&self) -> f32 {
        self.height
    }

    /// Convert to tuple (x, y, w, h)
    fn as_tuple(&self) -> (f32, f32, f32, f32) {
        (self.x, self.y, self.width, self.height)
    }

    /// Convert to xyxy format (x1, y1, x2, y2)
    fn as_xyxy(&self) -> (f32, f32, f32, f32) {
        (self.x, self.y, self.x + self.width, self.y + self.height)
    }

    fn __repr__(&self) -> String {
        format!(
            "BoundingBox2D(x={:.1}, y={:.1}, w={:.1}, h={:.1})",
            self.x, self.y, self.width, self.height
        )
    }
}

// ============================================
// Detection
// ============================================

/// 2D object detection result
#[pyclass(name = "Detection")]
#[derive(Debug, Clone)]
pub struct PyDetection {
    bbox: PyBoundingBox2D,
    confidence: f32,
    class_id: u32,
    class_name: String,
    instance_id: u32,
}

#[pymethods]
impl PyDetection {
    #[new]
    #[pyo3(signature = (class_name, confidence, x, y, width, height, class_id=0, instance_id=0))]
    fn new(
        class_name: &str,
        confidence: f32,
        x: f32,
        y: f32,
        width: f32,
        height: f32,
        class_id: u32,
        instance_id: u32,
    ) -> Self {
        Self {
            bbox: PyBoundingBox2D::new(x, y, width, height),
            confidence,
            class_id,
            class_name: class_name.to_string(),
            instance_id,
        }
    }

    /// Create from bounding box
    #[staticmethod]
    #[pyo3(signature = (bbox, class_name, confidence, class_id=0, instance_id=0))]
    fn from_bbox(
        bbox: &PyBoundingBox2D,
        class_name: &str,
        confidence: f32,
        class_id: u32,
        instance_id: u32,
    ) -> Self {
        Self {
            bbox: bbox.clone(),
            confidence,
            class_id,
            class_name: class_name.to_string(),
            instance_id,
        }
    }

    #[getter]
    fn bbox(&self) -> PyBoundingBox2D {
        self.bbox.clone()
    }

    #[getter]
    fn confidence(&self) -> f32 {
        self.confidence
    }

    #[getter]
    fn class_id(&self) -> u32 {
        self.class_id
    }

    #[getter]
    fn class_name(&self) -> &str {
        &self.class_name
    }

    #[getter]
    fn instance_id(&self) -> u32 {
        self.instance_id
    }

    /// Check if detection passes confidence threshold
    fn is_confident(&self, threshold: f32) -> bool {
        self.confidence >= threshold
    }

    /// Convert to bytes for IPC transmission
    fn to_bytes(&self, py: Python<'_>) -> PyResult<Py<PyBytes>> {
        // Create 72-byte Detection struct
        let mut bytes = vec![0u8; 72];

        // BoundingBox2D (16 bytes)
        bytes[0..4].copy_from_slice(&self.bbox.x.to_le_bytes());
        bytes[4..8].copy_from_slice(&self.bbox.y.to_le_bytes());
        bytes[8..12].copy_from_slice(&self.bbox.width.to_le_bytes());
        bytes[12..16].copy_from_slice(&self.bbox.height.to_le_bytes());

        // confidence (4 bytes)
        bytes[16..20].copy_from_slice(&self.confidence.to_le_bytes());

        // class_id (4 bytes)
        bytes[20..24].copy_from_slice(&self.class_id.to_le_bytes());

        // class_name (32 bytes, null-padded)
        let name_bytes = self.class_name.as_bytes();
        let len = name_bytes.len().min(31);
        bytes[24..24 + len].copy_from_slice(&name_bytes[..len]);

        // instance_id (4 bytes)
        bytes[56..60].copy_from_slice(&self.instance_id.to_le_bytes());

        // padding (12 bytes) - already zero

        Ok(PyBytes::new(py, &bytes).into())
    }

    /// Create from bytes (IPC deserialization)
    #[staticmethod]
    fn from_bytes(data: &[u8]) -> PyResult<Self> {
        if data.len() < 72 {
            return Err(pyo3::exceptions::PyValueError::new_err(format!(
                "Detection requires 72 bytes, got {}",
                data.len()
            )));
        }

        let x = f32::from_le_bytes(data[0..4].try_into().unwrap());
        let y = f32::from_le_bytes(data[4..8].try_into().unwrap());
        let width = f32::from_le_bytes(data[8..12].try_into().unwrap());
        let height = f32::from_le_bytes(data[12..16].try_into().unwrap());
        let confidence = f32::from_le_bytes(data[16..20].try_into().unwrap());
        let class_id = u32::from_le_bytes(data[20..24].try_into().unwrap());

        let name_end = data[24..56].iter().position(|&b| b == 0).unwrap_or(32);
        let class_name = String::from_utf8_lossy(&data[24..24 + name_end]).to_string();

        let instance_id = u32::from_le_bytes(data[56..60].try_into().unwrap());

        Ok(Self {
            bbox: PyBoundingBox2D::new(x, y, width, height),
            confidence,
            class_id,
            class_name,
            instance_id,
        })
    }

    fn __repr__(&self) -> String {
        format!(
            "Detection('{}', conf={:.2}, bbox=({:.0}, {:.0}, {:.0}, {:.0}))",
            self.class_name,
            self.confidence,
            self.bbox.x,
            self.bbox.y,
            self.bbox.width,
            self.bbox.height
        )
    }
}

// ============================================
// PointXYZ
// ============================================

/// 3D point (x, y, z)
#[pyclass(name = "PointXYZ")]
#[derive(Debug, Clone, Copy)]
pub struct PyPointXYZ {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[pymethods]
impl PyPointXYZ {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, z=0.0))]
    fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    #[getter]
    fn x(&self) -> f32 {
        self.x
    }

    #[getter]
    fn y(&self) -> f32 {
        self.y
    }

    #[getter]
    fn z(&self) -> f32 {
        self.z
    }

    /// Distance from origin
    fn distance(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Distance to another point
    fn distance_to(&self, other: &PyPointXYZ) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Convert to tuple
    fn as_tuple(&self) -> (f32, f32, f32) {
        (self.x, self.y, self.z)
    }

    /// Convert to numpy array (requires numpy)
    fn to_numpy<'py>(&self, py: Python<'py>) -> PyResult<PyObject> {
        let np = py.import("numpy")?;
        let arr = np.call_method1("array", (vec![self.x, self.y, self.z],))?;
        Ok(arr.unbind())
    }

    fn __repr__(&self) -> String {
        format!("PointXYZ({:.3}, {:.3}, {:.3})", self.x, self.y, self.z)
    }
}

// ============================================
// PointXYZRGB
// ============================================

/// 3D point with RGB color
#[pyclass(name = "PointXYZRGB")]
#[derive(Debug, Clone, Copy)]
pub struct PyPointXYZRGB {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

#[pymethods]
impl PyPointXYZRGB {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, z=0.0, r=255, g=255, b=255))]
    fn new(x: f32, y: f32, z: f32, r: u8, g: u8, b: u8) -> Self {
        Self { x, y, z, r, g, b }
    }

    #[getter]
    fn x(&self) -> f32 {
        self.x
    }

    #[getter]
    fn y(&self) -> f32 {
        self.y
    }

    #[getter]
    fn z(&self) -> f32 {
        self.z
    }

    #[getter]
    fn r(&self) -> u8 {
        self.r
    }

    #[getter]
    fn g(&self) -> u8 {
        self.g
    }

    #[getter]
    fn b(&self) -> u8 {
        self.b
    }

    /// Get RGB as tuple
    fn rgb(&self) -> (u8, u8, u8) {
        (self.r, self.g, self.b)
    }

    /// Get XYZ as PointXYZ
    fn xyz(&self) -> PyPointXYZ {
        PyPointXYZ {
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }

    fn __repr__(&self) -> String {
        format!(
            "PointXYZRGB({:.3}, {:.3}, {:.3}, rgb=({}, {}, {}))",
            self.x, self.y, self.z, self.r, self.g, self.b
        )
    }
}

// ============================================
// Landmark
// ============================================

/// 2D landmark/keypoint for pose estimation
#[pyclass(name = "Landmark")]
#[derive(Debug, Clone, Copy)]
pub struct PyLandmark {
    pub x: f32,
    pub y: f32,
    pub visibility: f32,
    pub index: u32,
}

#[pymethods]
impl PyLandmark {
    #[new]
    #[pyo3(signature = (x, y, visibility=1.0, index=0))]
    fn new(x: f32, y: f32, visibility: f32, index: u32) -> Self {
        Self {
            x,
            y,
            visibility,
            index,
        }
    }

    #[getter]
    fn x(&self) -> f32 {
        self.x
    }

    #[getter]
    fn y(&self) -> f32 {
        self.y
    }

    #[getter]
    fn visibility(&self) -> f32 {
        self.visibility
    }

    #[getter]
    fn index(&self) -> u32 {
        self.index
    }

    /// Check if landmark is visible
    fn is_visible(&self, threshold: f32) -> bool {
        self.visibility >= threshold
    }

    /// Distance to another landmark
    fn distance_to(&self, other: &PyLandmark) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    fn __repr__(&self) -> String {
        format!(
            "Landmark({:.1}, {:.1}, vis={:.2}, idx={})",
            self.x, self.y, self.visibility, self.index
        )
    }
}

// ============================================
// DetectionList - for batch operations
// ============================================

/// List of detections with batch operations
#[pyclass(name = "DetectionList")]
#[derive(Debug, Clone)]
pub struct PyDetectionList {
    detections: Vec<PyDetection>,
}

#[pymethods]
impl PyDetectionList {
    #[new]
    fn new() -> Self {
        Self {
            detections: Vec::new(),
        }
    }

    /// Create from list of detections
    #[staticmethod]
    fn from_list(detections: Vec<PyDetection>) -> Self {
        Self { detections }
    }

    /// Add a detection
    fn append(&mut self, detection: PyDetection) {
        self.detections.push(detection);
    }

    /// Get number of detections
    fn __len__(&self) -> usize {
        self.detections.len()
    }

    /// Get detection by index
    fn __getitem__(&self, index: usize) -> PyResult<PyDetection> {
        self.detections
            .get(index)
            .cloned()
            .ok_or_else(|| pyo3::exceptions::PyIndexError::new_err("Index out of range"))
    }

    /// Filter by confidence threshold
    fn filter_confidence(&self, threshold: f32) -> Self {
        Self {
            detections: self
                .detections
                .iter()
                .filter(|d| d.confidence >= threshold)
                .cloned()
                .collect(),
        }
    }

    /// Filter by class name
    fn filter_class(&self, class_name: &str) -> Self {
        Self {
            detections: self
                .detections
                .iter()
                .filter(|d| d.class_name == class_name)
                .cloned()
                .collect(),
        }
    }

    /// Convert to bytes for IPC (all detections concatenated)
    fn to_bytes(&self, py: Python<'_>) -> PyResult<Py<PyBytes>> {
        let mut bytes = Vec::with_capacity(self.detections.len() * 72);
        for det in &self.detections {
            let det_bytes = det.to_bytes(py)?;
            let borrowed = det_bytes.bind(py);
            bytes.extend_from_slice(borrowed.as_bytes());
        }
        Ok(PyBytes::new(py, &bytes).into())
    }

    /// Create from bytes
    #[staticmethod]
    fn from_bytes(data: &[u8]) -> PyResult<Self> {
        let num_detections = data.len() / 72;
        let mut detections = Vec::with_capacity(num_detections);

        for i in 0..num_detections {
            let start = i * 72;
            let end = start + 72;
            detections.push(PyDetection::from_bytes(&data[start..end])?);
        }

        Ok(Self { detections })
    }

    /// Convert to list of dicts (for JSON serialization)
    fn to_dicts(&self, py: Python<'_>) -> PyResult<Py<PyList>> {
        let list = PyList::empty(py);
        for det in &self.detections {
            let dict = PyDict::new(py);
            dict.set_item("class_name", &det.class_name)?;
            dict.set_item("confidence", det.confidence)?;
            dict.set_item("class_id", det.class_id)?;
            dict.set_item("instance_id", det.instance_id)?;
            dict.set_item("bbox", det.bbox.as_tuple())?;
            list.append(dict)?;
        }
        Ok(list.into())
    }

    fn __repr__(&self) -> String {
        format!("DetectionList({} detections)", self.detections.len())
    }

    fn __iter__(slf: PyRef<'_, Self>) -> PyDetectionListIter {
        PyDetectionListIter {
            inner: slf.detections.clone().into_iter(),
        }
    }
}

#[pyclass]
struct PyDetectionListIter {
    inner: std::vec::IntoIter<PyDetection>,
}

#[pymethods]
impl PyDetectionListIter {
    fn __iter__(slf: PyRef<'_, Self>) -> PyRef<'_, Self> {
        slf
    }

    fn __next__(mut slf: PyRefMut<'_, Self>) -> Option<PyDetection> {
        slf.inner.next()
    }
}

// ============================================
// COCO Pose Constants
// ============================================

/// COCO keypoint indices
#[pyclass(name = "COCOPose")]
pub struct PyCOCOPose;

#[pymethods]
impl PyCOCOPose {
    #[classattr]
    const NOSE: u32 = 0;
    #[classattr]
    const LEFT_EYE: u32 = 1;
    #[classattr]
    const RIGHT_EYE: u32 = 2;
    #[classattr]
    const LEFT_EAR: u32 = 3;
    #[classattr]
    const RIGHT_EAR: u32 = 4;
    #[classattr]
    const LEFT_SHOULDER: u32 = 5;
    #[classattr]
    const RIGHT_SHOULDER: u32 = 6;
    #[classattr]
    const LEFT_ELBOW: u32 = 7;
    #[classattr]
    const RIGHT_ELBOW: u32 = 8;
    #[classattr]
    const LEFT_WRIST: u32 = 9;
    #[classattr]
    const RIGHT_WRIST: u32 = 10;
    #[classattr]
    const LEFT_HIP: u32 = 11;
    #[classattr]
    const RIGHT_HIP: u32 = 12;
    #[classattr]
    const LEFT_KNEE: u32 = 13;
    #[classattr]
    const RIGHT_KNEE: u32 = 14;
    #[classattr]
    const LEFT_ANKLE: u32 = 15;
    #[classattr]
    const RIGHT_ANKLE: u32 = 16;
    #[classattr]
    const NUM_KEYPOINTS: u32 = 17;
}

// ============================================
// TrackedObject
// ============================================

/// Tracked object with ID and velocity
#[pyclass(name = "TrackedObject")]
#[derive(Debug, Clone)]
pub struct PyTrackedObject {
    bbox: PyBoundingBox2D,
    track_id: u64,
    confidence: f32,
    class_id: u32,
    class_name: String,
    velocity_x: f32,
    velocity_y: f32,
    age: u32,
    hits: u32,
    time_since_update: u32,
    state: u32, // 0=tentative, 1=confirmed, 2=deleted
}

#[pymethods]
impl PyTrackedObject {
    #[new]
    #[pyo3(signature = (track_id, bbox, class_name, confidence, class_id=0))]
    fn new(
        track_id: u64,
        bbox: &PyBoundingBox2D,
        class_name: &str,
        confidence: f32,
        class_id: u32,
    ) -> Self {
        Self {
            bbox: bbox.clone(),
            track_id,
            confidence,
            class_id,
            class_name: class_name.to_string(),
            velocity_x: 0.0,
            velocity_y: 0.0,
            age: 0,
            hits: 1,
            time_since_update: 0,
            state: 0,
        }
    }

    #[getter]
    fn bbox(&self) -> PyBoundingBox2D {
        self.bbox.clone()
    }

    #[getter]
    fn track_id(&self) -> u64 {
        self.track_id
    }

    #[getter]
    fn confidence(&self) -> f32 {
        self.confidence
    }

    #[getter]
    fn class_id(&self) -> u32 {
        self.class_id
    }

    #[getter]
    fn class_name(&self) -> &str {
        &self.class_name
    }

    #[getter]
    fn velocity(&self) -> (f32, f32) {
        (self.velocity_x, self.velocity_y)
    }

    #[getter]
    fn age(&self) -> u32 {
        self.age
    }

    #[getter]
    fn hits(&self) -> u32 {
        self.hits
    }

    /// Speed magnitude
    fn speed(&self) -> f32 {
        (self.velocity_x * self.velocity_x + self.velocity_y * self.velocity_y).sqrt()
    }

    /// Check if track is tentative
    fn is_tentative(&self) -> bool {
        self.state == 0
    }

    /// Check if track is confirmed
    fn is_confirmed(&self) -> bool {
        self.state == 1
    }

    /// Check if track is deleted
    fn is_deleted(&self) -> bool {
        self.state == 2
    }

    /// Update with new detection
    fn update(&mut self, bbox: &PyBoundingBox2D, confidence: f32) {
        self.velocity_x = bbox.center_x() - self.bbox.center_x();
        self.velocity_y = bbox.center_y() - self.bbox.center_y();
        self.bbox = bbox.clone();
        self.confidence = confidence;
        self.hits += 1;
        self.time_since_update = 0;
        self.age += 1;
    }

    /// Mark as missed (no detection this frame)
    fn mark_missed(&mut self) {
        self.time_since_update += 1;
        self.age += 1;
    }

    /// Confirm the track
    fn confirm(&mut self) {
        self.state = 1;
    }

    /// Delete the track
    fn delete(&mut self) {
        self.state = 2;
    }

    fn __repr__(&self) -> String {
        format!(
            "TrackedObject(id={}, '{}', conf={:.2}, age={})",
            self.track_id, self.class_name, self.confidence, self.age
        )
    }
}

// ============================================
// PointCloudBuffer - numpy-friendly point cloud
// ============================================

/// Point cloud buffer with numpy integration
#[pyclass(name = "PointCloudBuffer")]
#[derive(Debug)]
pub struct PyPointCloudBuffer {
    points: Vec<f32>, // Flat array: [x0, y0, z0, x1, y1, z1, ...]
    num_points: usize,
    point_type: u32, // 0=XYZ, 1=XYZRGB, 2=XYZI
    frame_id: String,
}

#[pymethods]
impl PyPointCloudBuffer {
    /// Create empty point cloud buffer
    #[new]
    #[pyo3(signature = (capacity=0, frame_id=""))]
    fn new(capacity: usize, frame_id: &str) -> Self {
        Self {
            points: Vec::with_capacity(capacity * 3),
            num_points: 0,
            point_type: 0,
            frame_id: frame_id.to_string(),
        }
    }

    /// Create from numpy array (N, 3) or (N, 4)
    #[staticmethod]
    #[pyo3(signature = (array, frame_id=""))]
    fn from_numpy(py: Python<'_>, array: PyObject, frame_id: &str) -> PyResult<Self> {
        let np = py.import("numpy")?;

        // Get array as contiguous float32
        let arr = np.call_method1("ascontiguousarray", (array,))?;
        let arr = arr.call_method1("astype", ("float32",))?;

        // Get shape
        let shape: Vec<usize> = arr.getattr("shape")?.extract()?;
        if shape.len() != 2 || shape[1] < 3 {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "Array must be (N, 3) or (N, 4)",
            ));
        }

        let num_points = shape[0];
        let point_dim = shape[1];
        let point_type = if point_dim == 3 { 0 } else { 2 }; // XYZ or XYZI

        // Get data as flat array
        let flat = arr.call_method0("flatten")?;
        let data: Vec<f32> = flat.extract()?;

        // For now, just store XYZ (first 3 values per point)
        let mut points = Vec::with_capacity(num_points * 3);
        for i in 0..num_points {
            points.push(data[i * point_dim]);
            points.push(data[i * point_dim + 1]);
            points.push(data[i * point_dim + 2]);
        }

        Ok(Self {
            points,
            num_points,
            point_type,
            frame_id: frame_id.to_string(),
        })
    }

    /// Add a point
    fn add_point(&mut self, x: f32, y: f32, z: f32) {
        self.points.push(x);
        self.points.push(y);
        self.points.push(z);
        self.num_points += 1;
    }

    /// Get number of points
    fn __len__(&self) -> usize {
        self.num_points
    }

    /// Get point by index
    fn __getitem__(&self, index: usize) -> PyResult<PyPointXYZ> {
        if index >= self.num_points {
            return Err(pyo3::exceptions::PyIndexError::new_err(
                "Index out of range",
            ));
        }
        let base = index * 3;
        Ok(PyPointXYZ {
            x: self.points[base],
            y: self.points[base + 1],
            z: self.points[base + 2],
        })
    }

    #[getter]
    fn frame_id(&self) -> &str {
        &self.frame_id
    }

    /// Convert to numpy array (N, 3)
    fn to_numpy<'py>(&self, py: Python<'py>) -> PyResult<PyObject> {
        let np = py.import("numpy")?;

        // Create array from flat data
        let arr = np.call_method1("array", (&self.points,))?;
        let reshaped = arr.call_method1("reshape", ((self.num_points, 3),))?;

        Ok(reshaped.unbind())
    }

    /// Get bytes for IPC transmission
    fn to_bytes(&self, py: Python<'_>) -> PyResult<Py<PyBytes>> {
        // Header (64 bytes) + point data
        let data_size = self.num_points * 12; // 3 floats * 4 bytes
        let mut bytes = vec![0u8; 64 + data_size];

        // Header
        bytes[0..8].copy_from_slice(&(self.num_points as u64).to_le_bytes());
        bytes[8..12].copy_from_slice(&self.point_type.to_le_bytes());
        bytes[12..16].copy_from_slice(&12u32.to_le_bytes()); // point_stride

        // Frame ID (32 bytes at offset 32)
        let frame_bytes = self.frame_id.as_bytes();
        let len = frame_bytes.len().min(31);
        bytes[32..32 + len].copy_from_slice(&frame_bytes[..len]);

        // Point data
        for (i, &val) in self.points.iter().enumerate() {
            let offset = 64 + i * 4;
            bytes[offset..offset + 4].copy_from_slice(&val.to_le_bytes());
        }

        Ok(PyBytes::new(py, &bytes).into())
    }

    fn __repr__(&self) -> String {
        format!(
            "PointCloudBuffer({} points, frame='{}')",
            self.num_points, self.frame_id
        )
    }
}

// ============================================
// Module registration
// ============================================

/// Register perception module classes
pub fn register_perception_module(parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let perception = PyModule::new(parent.py(), "perception")?;

    // Bounding boxes
    perception.add_class::<PyBoundingBox2D>()?;

    // Detection types
    perception.add_class::<PyDetection>()?;
    perception.add_class::<PyDetectionList>()?;

    // Point cloud types
    perception.add_class::<PyPointXYZ>()?;
    perception.add_class::<PyPointXYZRGB>()?;
    perception.add_class::<PyPointCloudBuffer>()?;

    // Landmark types
    perception.add_class::<PyLandmark>()?;

    // Tracking types
    perception.add_class::<PyTrackedObject>()?;

    // Constants
    perception.add_class::<PyCOCOPose>()?;

    // Add docstring
    perception.setattr(
        "__doc__",
        "HORUS perception types - Detection, PointCloud, Landmark for AI/ML pipelines",
    )?;

    parent.add_submodule(&perception)?;

    Ok(())
}
