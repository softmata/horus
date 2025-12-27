// Python wrappers for perception messages
use horus_library::messages::perception;
use pyo3::prelude::*;

use crate::{PyPoint3, PyVector3};

/// Point field type enum
#[pyclass(module = "horus.library._library", name = "PointFieldType")]
#[derive(Clone, Copy)]
pub struct PyPointFieldType {
    pub(crate) inner: perception::PointFieldType,
}

#[pymethods]
impl PyPointFieldType {
    #[classattr]
    const INT8: u8 = 1;
    #[classattr]
    const UINT8: u8 = 2;
    #[classattr]
    const INT16: u8 = 3;
    #[classattr]
    const UINT16: u8 = 4;
    #[classattr]
    const INT32: u8 = 5;
    #[classattr]
    const UINT32: u8 = 6;
    #[classattr]
    const FLOAT32: u8 = 7;
    #[classattr]
    const FLOAT64: u8 = 8;

    #[new]
    #[pyo3(signature = (dtype=7))]
    fn new(dtype: u8) -> Self {
        let inner = match dtype {
            1 => perception::PointFieldType::Int8,
            2 => perception::PointFieldType::UInt8,
            3 => perception::PointFieldType::Int16,
            4 => perception::PointFieldType::UInt16,
            5 => perception::PointFieldType::Int32,
            6 => perception::PointFieldType::UInt32,
            7 => perception::PointFieldType::Float32,
            8 => perception::PointFieldType::Float64,
            _ => perception::PointFieldType::Float32,
        };
        Self { inner }
    }

    fn __repr__(&self) -> String {
        format!("{:?}", self.inner)
    }
}

/// Python wrapper for PointField
#[pyclass(module = "horus.library._library", name = "PointField")]
#[derive(Clone)]
pub struct PyPointField {
    pub(crate) inner: perception::PointField,
}

#[pymethods]
impl PyPointField {
    #[new]
    #[pyo3(signature = (name="x", offset=0, datatype=None, count=1))]
    fn new(name: &str, offset: u32, datatype: Option<&PyPointFieldType>, count: u32) -> Self {
        let dtype = datatype
            .map(|d| d.inner)
            .unwrap_or(perception::PointFieldType::Float32);
        Self {
            inner: perception::PointField::new(name, offset, dtype, count),
        }
    }

    fn name_str(&self) -> String {
        self.inner.name_str()
    }

    #[getter]
    fn offset(&self) -> u32 {
        self.inner.offset
    }

    #[setter]
    fn set_offset(&mut self, value: u32) {
        self.inner.offset = value;
    }

    #[getter]
    fn count(&self) -> u32 {
        self.inner.count
    }

    #[setter]
    fn set_count(&mut self, value: u32) {
        self.inner.count = value;
    }

    fn field_size(&self) -> u32 {
        self.inner.field_size()
    }

    fn __repr__(&self) -> String {
        format!(
            "PointField('{}', offset={}, {:?}, count={})",
            self.inner.name_str(),
            self.inner.offset,
            self.inner.datatype,
            self.inner.count
        )
    }
}

/// Python wrapper for PointCloud
#[pyclass(module = "horus.library._library", name = "PointCloud")]
#[derive(Clone)]
pub struct PyPointCloud {
    pub(crate) inner: perception::PointCloud,
}

#[pymethods]
impl PyPointCloud {
    #[new]
    fn new() -> Self {
        Self {
            inner: perception::PointCloud::new(),
        }
    }

    #[staticmethod]
    fn xyz(points: Vec<(f64, f64, f64)>) -> Self {
        let point3_vec: Vec<horus_library::messages::geometry::Point3> = points
            .iter()
            .map(|(x, y, z)| horus_library::messages::geometry::Point3::new(*x, *y, *z))
            .collect();
        Self {
            inner: perception::PointCloud::xyz(&point3_vec),
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
    fn field_count(&self) -> u8 {
        self.inner.field_count
    }

    #[getter]
    fn is_dense(&self) -> bool {
        self.inner.is_dense
    }

    #[setter]
    fn set_is_dense(&mut self, value: bool) {
        self.inner.is_dense = value;
    }

    #[getter]
    fn point_step(&self) -> u32 {
        self.inner.point_step
    }

    #[getter]
    fn row_step(&self) -> u32 {
        self.inner.row_step
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

    fn point_count(&self) -> u32 {
        self.inner.point_count()
    }

    fn is_valid(&self) -> bool {
        self.inner.is_valid()
    }

    fn extract_xyz(&self) -> Option<Vec<(f64, f64, f64)>> {
        self.inner
            .extract_xyz()
            .map(|points| points.iter().map(|p| (p.x, p.y, p.z)).collect())
    }

    fn __len__(&self) -> usize {
        self.inner.point_count() as usize
    }

    fn __repr__(&self) -> String {
        format!(
            "PointCloud({} points, {} fields, {} bytes)",
            self.inner.point_count(),
            self.inner.field_count,
            self.inner.data.len()
        )
    }
}

/// Python wrapper for BoundingBox3D
#[pyclass(module = "horus.library._library", name = "BoundingBox3D")]
#[derive(Clone)]
pub struct PyBoundingBox3D {
    pub(crate) inner: perception::BoundingBox3D,
}

#[pymethods]
impl PyBoundingBox3D {
    #[new]
    #[pyo3(signature = (center=None, size=None))]
    fn new(center: Option<&PyPoint3>, size: Option<&PyVector3>) -> Self {
        let c = center
            .map(|p| {
                horus_library::messages::geometry::Point3::new(p.inner.x, p.inner.y, p.inner.z)
            })
            .unwrap_or_default();
        let s = size
            .map(|v| {
                horus_library::messages::geometry::Vector3::new(v.inner.x, v.inner.y, v.inner.z)
            })
            .unwrap_or_default();
        Self {
            inner: perception::BoundingBox3D::new(c, s),
        }
    }

    #[getter]
    fn center(&self) -> PyPoint3 {
        PyPoint3 {
            inner: horus_library::messages::geometry::Point3::new(
                self.inner.center.x,
                self.inner.center.y,
                self.inner.center.z,
            ),
        }
    }

    #[getter]
    fn size(&self) -> PyVector3 {
        PyVector3 {
            inner: horus_library::messages::geometry::Vector3::new(
                self.inner.size.x,
                self.inner.size.y,
                self.inner.size.z,
            ),
        }
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

    fn label_str(&self) -> String {
        self.inner.label_str()
    }

    fn volume(&self) -> f64 {
        self.inner.volume()
    }

    fn __repr__(&self) -> String {
        format!(
            "BoundingBox3D('{}', conf={:.2}, vol={:.2})",
            self.inner.label_str(),
            self.inner.confidence,
            self.inner.volume()
        )
    }
}

/// Python wrapper for BoundingBoxArray3D
#[pyclass(module = "horus.library._library", name = "BoundingBoxArray3D")]
#[derive(Clone)]
pub struct PyBoundingBoxArray3D {
    pub(crate) inner: perception::BoundingBoxArray3D,
}

#[pymethods]
impl PyBoundingBoxArray3D {
    #[new]
    fn new() -> Self {
        Self {
            inner: perception::BoundingBoxArray3D::new(),
        }
    }

    #[getter]
    fn count(&self) -> u8 {
        self.inner.count
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn add_box(&mut self, bbox: &PyBoundingBox3D) -> PyResult<()> {
        self.inner
            .add_box(bbox.inner)
            .map_err(|e| pyo3::exceptions::PyValueError::new_err(e))
    }

    fn get_boxes(&self) -> Vec<PyBoundingBox3D> {
        self.inner
            .get_boxes()
            .iter()
            .map(|b| PyBoundingBox3D { inner: *b })
            .collect()
    }

    fn filter_by_confidence(&self, threshold: f32) -> Vec<PyBoundingBox3D> {
        self.inner
            .filter_by_confidence(threshold)
            .into_iter()
            .map(|b| PyBoundingBox3D { inner: b })
            .collect()
    }

    fn filter_by_label(&self, label: &str) -> Vec<PyBoundingBox3D> {
        self.inner
            .filter_by_label(label)
            .into_iter()
            .map(|b| PyBoundingBox3D { inner: b })
            .collect()
    }

    fn __len__(&self) -> usize {
        self.inner.count as usize
    }

    fn __repr__(&self) -> String {
        format!("BoundingBoxArray3D({} boxes)", self.inner.count)
    }
}

/// Python wrapper for DepthImage
#[pyclass(module = "horus.library._library", name = "DepthImage")]
#[derive(Clone)]
pub struct PyDepthImage {
    pub(crate) inner: perception::DepthImage,
}

#[pymethods]
impl PyDepthImage {
    #[new]
    #[pyo3(signature = (width=0, height=0, depths=None))]
    fn new(width: u32, height: u32, depths: Option<Vec<u16>>) -> Self {
        let d = depths.unwrap_or_default();
        Self {
            inner: perception::DepthImage::new(width, height, d),
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
    fn depths(&self) -> Vec<u16> {
        self.inner.depths.clone()
    }

    #[setter]
    fn set_depths(&mut self, value: Vec<u16>) {
        self.inner.depths = value;
    }

    #[getter]
    fn min_depth(&self) -> u16 {
        self.inner.min_depth
    }

    #[setter]
    fn set_min_depth(&mut self, value: u16) {
        self.inner.min_depth = value;
    }

    #[getter]
    fn max_depth(&self) -> u16 {
        self.inner.max_depth
    }

    #[setter]
    fn set_max_depth(&mut self, value: u16) {
        self.inner.max_depth = value;
    }

    #[getter]
    fn depth_scale(&self) -> f32 {
        self.inner.depth_scale
    }

    #[setter]
    fn set_depth_scale(&mut self, value: f32) {
        self.inner.depth_scale = value;
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn get_depth(&self, x: u32, y: u32) -> Option<u16> {
        self.inner.get_depth(x, y)
    }

    fn set_depth(&mut self, x: u32, y: u32, depth: u16) -> bool {
        self.inner.set_depth(x, y, depth)
    }

    fn is_valid_depth(&self, depth: u16) -> bool {
        self.inner.is_valid_depth(depth)
    }

    fn to_point_cloud(&self, fx: f64, fy: f64, cx: f64, cy: f64) -> PyPointCloud {
        PyPointCloud {
            inner: self.inner.to_point_cloud(fx, fy, cx, cy),
        }
    }

    fn depth_statistics(&self) -> (f32, f32, f32) {
        self.inner.depth_statistics()
    }

    fn __repr__(&self) -> String {
        format!(
            "DepthImage({}x{}, {} bytes)",
            self.inner.width,
            self.inner.height,
            self.inner.depths.len() * 2
        )
    }
}

/// Python wrapper for PlaneDetection
#[pyclass(module = "horus.library._library", name = "PlaneDetection")]
#[derive(Clone)]
pub struct PyPlaneDetection {
    pub(crate) inner: perception::PlaneDetection,
}

#[pymethods]
impl PyPlaneDetection {
    #[new]
    fn new() -> Self {
        Self {
            inner: perception::PlaneDetection::default(),
        }
    }

    #[getter]
    fn coefficients(&self) -> [f64; 4] {
        self.inner.coefficients
    }

    #[setter]
    fn set_coefficients(&mut self, value: [f64; 4]) {
        self.inner.coefficients = value;
    }

    #[getter]
    fn center(&self) -> PyPoint3 {
        PyPoint3 {
            inner: horus_library::messages::geometry::Point3::new(
                self.inner.center.x,
                self.inner.center.y,
                self.inner.center.z,
            ),
        }
    }

    #[getter]
    fn normal(&self) -> PyVector3 {
        PyVector3 {
            inner: horus_library::messages::geometry::Vector3::new(
                self.inner.normal.x,
                self.inner.normal.y,
                self.inner.normal.z,
            ),
        }
    }

    #[getter]
    fn size(&self) -> [f64; 2] {
        self.inner.size
    }

    #[setter]
    fn set_size(&mut self, value: [f64; 2]) {
        self.inner.size = value;
    }

    #[getter]
    fn inlier_count(&self) -> u32 {
        self.inner.inlier_count
    }

    #[setter]
    fn set_inlier_count(&mut self, value: u32) {
        self.inner.inlier_count = value;
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
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn plane_type_str(&self) -> String {
        self.inner.plane_type_str()
    }

    fn __repr__(&self) -> String {
        format!(
            "PlaneDetection('{}', {} inliers, conf={:.2})",
            self.inner.plane_type_str(),
            self.inner.inlier_count,
            self.inner.confidence
        )
    }
}

/// Python wrapper for PlaneArray
#[pyclass(module = "horus.library._library", name = "PlaneArray")]
#[derive(Clone)]
pub struct PyPlaneArray {
    pub(crate) inner: perception::PlaneArray,
}

#[pymethods]
impl PyPlaneArray {
    #[new]
    fn new() -> Self {
        Self {
            inner: perception::PlaneArray::default(),
        }
    }

    #[getter]
    fn count(&self) -> u8 {
        self.inner.count
    }

    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn get_planes(&self) -> Vec<PyPlaneDetection> {
        self.inner.planes[..self.inner.count as usize]
            .iter()
            .map(|p| PyPlaneDetection { inner: *p })
            .collect()
    }

    fn __len__(&self) -> usize {
        self.inner.count as usize
    }

    fn __repr__(&self) -> String {
        format!("PlaneArray({} planes)", self.inner.count)
    }
}
