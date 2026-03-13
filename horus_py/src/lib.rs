use pyo3::prelude::*;

mod config;
mod depth_image;
mod dlpack_utils;
pub mod errors;
mod transform_frame;
mod image;
mod messages;
mod node;
mod perception;
mod pointcloud;
mod scheduler;
mod tensor;
mod topic;
mod types;

use config::PySchedulerConfig;
use transform_frame::{PyTransformFrame, PyTransformFrameConfig, PyTransform};
use node::{PyNode, PyNodeInfo, PyNodeState};
use scheduler::{PyMiss, PyScheduler};
use topic::PyTopic;
use types::Priority;

/// HORUS Python Bindings
#[pymodule]
fn _horus(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Core classes
    m.add_class::<PyNode>()?;
    m.add_class::<PyNodeInfo>()?;
    m.add_class::<PyTopic>()?;
    m.add_class::<PyScheduler>()?;
    m.add_class::<PyNodeState>()?;
    m.add_class::<PySchedulerConfig>()?;
    m.add_class::<PyMiss>()?;
    m.add_class::<Priority>()?;

    // TransformFrame transform system
    m.add_class::<PyTransform>()?;
    m.add_class::<PyTransformFrame>()?;
    m.add_class::<PyTransformFrameConfig>()?;
    m.add_function(wrap_pyfunction!(transform_frame::get_timestamp_ns, m)?)?;

    // Domain types
    m.add_class::<image::PyImage>()?;
    m.add_class::<pointcloud::PyPointCloud>()?;
    m.add_class::<depth_image::PyDepthImage>()?;

    // Perception types
    perception::register_perception_module(m)?;

    // Message types
    messages::register_message_classes(m)?;

    // Custom exception types
    m.add(
        "HorusNotFoundError",
        m.py().get_type::<errors::HorusNotFoundError>(),
    )?;
    m.add(
        "HorusTransformError",
        m.py().get_type::<errors::HorusTransformError>(),
    )?;
    m.add(
        "HorusTimeoutError",
        m.py().get_type::<errors::HorusTimeoutError>(),
    )?;

    m.add_function(wrap_pyfunction!(get_version, m)?)?;

    Ok(())
}

/// Get HORUS version information
#[pyfunction]
fn get_version() -> String {
    format!("HORUS Python Bindings v{}", env!("CARGO_PKG_VERSION"))
}
