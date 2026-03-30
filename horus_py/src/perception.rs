//! horus.perception - Python bindings for perception types
//!
//! This module provides Python-friendly wrappers for HORUS perception types:
//! - Detection / Detection3D - Object detection results
//! - PointXYZ / PointXYZRGB / PointXYZI - Point cloud points
//!
// PyO3 constructors expose struct fields as Python kwargs — exceeds Clippy's threshold.
#![allow(clippy::too_many_arguments)]
//! - Landmark / Landmark3D - Pose estimation keypoints
//! - SegmentationMask - Semantic/instance segmentation
//! - TrackedObject - Multi-object tracking

use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyDict, PyList};

use horus_library::messages::detection::BoundingBox2D;

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
        let bb = BoundingBox2D::from_center(cx, cy, width, height);
        Self {
            x: bb.x,
            y: bb.y,
            width: bb.width,
            height: bb.height,
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
        self.as_rust().iou(&other.as_rust())
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

impl PyBoundingBox2D {
    /// Convert to the Rust BoundingBox2D type (for delegating to its methods).
    fn as_rust(&self) -> BoundingBox2D {
        BoundingBox2D::new(self.x, self.y, self.width, self.height)
    }
}

// ============================================
// Detection
// ============================================

/// Byte size of the serialized `Detection` wire format.
///
/// Layout (all little-endian):
/// ```text
///  0.. 4  bbox.x        (f32)
///  4.. 8  bbox.y        (f32)
///  8..12  bbox.width    (f32)
/// 12..16  bbox.height   (f32)
/// 16..20  confidence    (f32)
/// 20..24  class_id      (u32)
/// 24..56  class_name    (32 bytes, null-padded UTF-8)
/// 56..60  instance_id   (u32)
/// 60..72  padding       (12 bytes, reserved)
/// ```
///
/// Any change to `Detection`'s fields must update this constant and the
/// `to_bytes` / `from_bytes` implementations together.
const DETECTION_BYTE_SIZE: usize = 72;

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

    /// Convert to bytes for IPC transmission.
    ///
    /// Produces exactly `DETECTION_BYTE_SIZE` bytes in the documented layout.
    fn to_bytes(&self, py: Python<'_>) -> PyResult<Py<PyBytes>> {
        use std::io::{Cursor, Write};

        let mut bytes = vec![0u8; DETECTION_BYTE_SIZE];
        debug_assert_eq!(
            bytes.len(),
            DETECTION_BYTE_SIZE,
            "to_bytes buffer must be exactly DETECTION_BYTE_SIZE bytes"
        );

        {
            let mut cur = Cursor::new(&mut bytes[..]);

            // BoundingBox2D (16 bytes): x, y, width, height (f32 LE each)
            cur.write_all(&self.bbox.x.to_le_bytes())
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
            cur.write_all(&self.bbox.y.to_le_bytes())
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
            cur.write_all(&self.bbox.width.to_le_bytes())
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
            cur.write_all(&self.bbox.height.to_le_bytes())
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;

            // confidence (4 bytes, f32 LE)
            cur.write_all(&self.confidence.to_le_bytes())
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;

            // class_id (4 bytes, u32 LE)
            cur.write_all(&self.class_id.to_le_bytes())
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;

            // class_name: 32 bytes, null-padded UTF-8; truncated to 31 chars to
            // leave room for the NUL terminator.
            let name_bytes = self.class_name.as_bytes();
            let len = name_bytes.len().min(31);
            cur.write_all(&name_bytes[..len])
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
            // cursor drops here, releasing &mut borrow on `bytes`
        }

        // instance_id (4 bytes, u32 LE) at offset 56.
        bytes[56..60].copy_from_slice(&self.instance_id.to_le_bytes());
        // padding bytes[60..72] are already zero.

        debug_assert_eq!(bytes.len(), DETECTION_BYTE_SIZE);
        Ok(PyBytes::new(py, &bytes).into())
    }

    /// Create from bytes (IPC deserialization).
    ///
    /// Requires exactly `DETECTION_BYTE_SIZE` bytes.  Both too-short and
    /// too-long buffers are rejected to catch format version mismatches early.
    #[staticmethod]
    fn from_bytes(data: &[u8]) -> PyResult<Self> {
        if data.len() != DETECTION_BYTE_SIZE {
            return Err(pyo3::exceptions::PyValueError::new_err(format!(
                "Detection requires exactly {} bytes, got {}",
                DETECTION_BYTE_SIZE,
                data.len()
            )));
        }

        // Use a cursor so reads are sequential and bounds-checked by the Read
        // trait implementation — no manual index arithmetic.
        use std::io::{Cursor, Read};

        let mut cur = Cursor::new(data);

        let mut buf4 = [0u8; 4];

        cur.read_exact(&mut buf4)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
        let x = f32::from_le_bytes(buf4);

        cur.read_exact(&mut buf4)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
        let y = f32::from_le_bytes(buf4);

        cur.read_exact(&mut buf4)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
        let width = f32::from_le_bytes(buf4);

        cur.read_exact(&mut buf4)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
        let height = f32::from_le_bytes(buf4);

        cur.read_exact(&mut buf4)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
        let confidence = f32::from_le_bytes(buf4);

        cur.read_exact(&mut buf4)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
        let class_id = u32::from_le_bytes(buf4);

        // class_name: 32-byte null-padded field.
        let mut name_buf = [0u8; 32];
        cur.read_exact(&mut name_buf)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
        let name_end = name_buf.iter().position(|&b| b == 0).unwrap_or(32);
        let class_name = String::from_utf8_lossy(&name_buf[..name_end]).to_string();

        cur.read_exact(&mut buf4)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e.to_string()))?;
        let instance_id = u32::from_le_bytes(buf4);

        // Remaining 12 bytes are padding — cursor position is at 60, data ends
        // at 72.  We do not read the padding; the length check above ensures
        // the buffer is exactly the right size.

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
    #[pyo3(name = "to_numpy")]
    fn as_numpy<'py>(&self, py: Python<'py>) -> PyResult<Py<PyAny>> {
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
    fn from_numpy(py: Python<'_>, array: Py<PyAny>, frame_id: &str) -> PyResult<Self> {
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
    fn to_numpy<'py>(&self, py: Python<'py>) -> PyResult<Py<PyAny>> {
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

    // Register in sys.modules so `from horus._horus.perception import X` works
    parent
        .py()
        .import("sys")?
        .getattr("modules")?
        .set_item("horus._horus.perception", &perception)?;

    Ok(())
}

#[cfg(test)]
mod detection_serialization_tests {
    use super::*;

    /// Helper: build the raw 72-byte wire buffer for a Detection without a
    /// Python runtime (mirrors the logic in `to_bytes` but returns `Vec<u8>`).
    fn encode(d: &PyDetection) -> Vec<u8> {
        let mut bytes = vec![0u8; DETECTION_BYTE_SIZE];
        bytes[0..4].copy_from_slice(&d.bbox.x.to_le_bytes());
        bytes[4..8].copy_from_slice(&d.bbox.y.to_le_bytes());
        bytes[8..12].copy_from_slice(&d.bbox.width.to_le_bytes());
        bytes[12..16].copy_from_slice(&d.bbox.height.to_le_bytes());
        bytes[16..20].copy_from_slice(&d.confidence.to_le_bytes());
        bytes[20..24].copy_from_slice(&d.class_id.to_le_bytes());
        let name = d.class_name.as_bytes();
        let len = name.len().min(31);
        bytes[24..24 + len].copy_from_slice(&name[..len]);
        bytes[56..60].copy_from_slice(&d.instance_id.to_le_bytes());
        bytes
    }

    /// Round-trip test: encode → decode → verify all fields match.
    #[test]
    fn test_detection_round_trip() {
        let original = PyDetection {
            bbox: PyBoundingBox2D::new(10.0, 20.0, 100.0, 50.0),
            confidence: 0.95,
            class_id: 3,
            class_name: "person".to_string(),
            instance_id: 42,
        };

        let bytes = encode(&original);
        assert_eq!(bytes.len(), DETECTION_BYTE_SIZE);

        let decoded = PyDetection::from_bytes(&bytes).expect("from_bytes failed");

        assert!((decoded.bbox.x - original.bbox.x).abs() < 1e-6);
        assert!((decoded.bbox.y - original.bbox.y).abs() < 1e-6);
        assert!((decoded.bbox.width - original.bbox.width).abs() < 1e-6);
        assert!((decoded.bbox.height - original.bbox.height).abs() < 1e-6);
        assert!((decoded.confidence - original.confidence).abs() < 1e-6);
        assert_eq!(decoded.class_id, original.class_id);
        assert_eq!(decoded.class_name, original.class_name);
        assert_eq!(decoded.instance_id, original.instance_id);
    }

    /// Edge case: zero bounding box and NaN confidence round-trips as NaN.
    #[test]
    fn test_detection_round_trip_nan_confidence() {
        let original = PyDetection {
            bbox: PyBoundingBox2D::new(0.0, 0.0, 0.0, 0.0),
            confidence: f32::NAN,
            class_id: 0,
            class_name: String::new(),
            instance_id: 0,
        };

        let bytes = encode(&original);
        let decoded = PyDetection::from_bytes(&bytes).expect("from_bytes failed");

        assert!(
            decoded.confidence.is_nan(),
            "NaN confidence must survive round-trip"
        );
        assert_eq!(decoded.class_name, "");
        assert_eq!(decoded.class_id, 0);
        assert_eq!(decoded.instance_id, 0);
    }

    /// class_name longer than 31 bytes must be silently truncated.
    #[test]
    fn test_detection_long_class_name_truncated() {
        let long_name = "a".repeat(40);
        let original = PyDetection {
            bbox: PyBoundingBox2D::new(0.0, 0.0, 1.0, 1.0),
            confidence: 1.0,
            class_id: 1,
            class_name: long_name,
            instance_id: 0,
        };

        let bytes = encode(&original);
        assert_eq!(bytes.len(), DETECTION_BYTE_SIZE);
        let decoded = PyDetection::from_bytes(&bytes).expect("from_bytes failed");
        // Truncated to 31 bytes (null terminator takes the 32nd).
        assert_eq!(decoded.class_name.len(), 31);
    }

    /// from_bytes must reject buffers shorter than DETECTION_BYTE_SIZE.
    #[test]
    fn test_from_bytes_rejects_short_buffer() {
        let short = vec![0u8; DETECTION_BYTE_SIZE - 1];
        PyDetection::from_bytes(&short).unwrap_err();
    }

    /// from_bytes must reject buffers longer than DETECTION_BYTE_SIZE.
    #[test]
    fn test_from_bytes_rejects_long_buffer() {
        let long = vec![0u8; DETECTION_BYTE_SIZE + 1];
        PyDetection::from_bytes(&long).unwrap_err();
    }
}
