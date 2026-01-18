// PyO3 methods MUST return PyResult<T> for Python bindings.
// Clippy incorrectly flags this as "useless_conversion" but it's required by PyO3.
// See: https://github.com/PyO3/pyo3/issues/2092
#![allow(clippy::useless_conversion)]
// PyO3 0.27 deprecates PyObject in favor of Py<PyAny> and with_gil in favor of attach.
// These are still functional but will be fixed in a future update.
#![allow(deprecated)]

use pyo3::prelude::*;

mod ai;
mod config;
mod hframe;
mod messages;
mod node;
mod perception;
mod router;
mod scheduler;
mod tensor;
mod topic;
mod types;

use config::{PyRobotPreset, PySchedulerConfig};
use hframe::{PyHFrame, PyHFrameConfig, PyTransform};
use node::{PyNode, PyNodeInfo, PyNodeState};
use router::{PyRouterClient, PyRouterServer};
use scheduler::PyScheduler;
use topic::PyTopic;
use types::Priority;

/// HORUS Python Bindings
///
/// This module provides Python bindings for the HORUS robotics framework,
/// allowing Python developers to create and run distributed robotic systems.
#[pymodule]
fn _horus(m: &Bound<'_, PyModule>) -> PyResult<()> {
    //  USER-FACING: Core classes that users interact with
    m.add_class::<PyNode>()?;
    m.add_class::<PyNodeInfo>()?;
    m.add_class::<PyTopic>()?; // Unified communication API
    m.add_class::<PyRouterClient>()?; // Explicit router connection management
    m.add_class::<PyRouterServer>()?; // Router server management
    m.add_class::<PyScheduler>()?;
    m.add_class::<PyNodeState>()?;

    // Router utility functions
    m.add_function(wrap_pyfunction!(router::default_router_endpoint, m)?)?;
    m.add_function(wrap_pyfunction!(router::router_endpoint, m)?)?;

    // Configuration classes
    m.add_class::<PyRobotPreset>()?;
    m.add_class::<PySchedulerConfig>()?;

    // Priority constants
    m.add_class::<Priority>()?;

    // HFrame - High-Performance Transform System
    m.add_class::<PyTransform>()?;
    m.add_class::<PyHFrame>()?;
    m.add_class::<PyHFrameConfig>()?;
    m.add_function(wrap_pyfunction!(hframe::get_timestamp_ns, m)?)?;

    // Tensor system - zero-copy shared memory tensors
    tensor::register_tensor_classes(m)?;

    // AI/ML utilities - DLPack, dtype, Device
    ai::register_ai_module(m)?;

    // Perception types - Detection, PointCloud, Landmark
    perception::register_perception_module(m)?;

    // Message types for typed Topic communication
    messages::register_message_classes(m)?;

    //  CHANGED: Priority system now uses u32 instead of enum
    // - Priority class provides constants: CRITICAL=0, HIGH=10, NORMAL=50, LOW=80, BACKGROUND=100
    // - Users can pass any u32 value for fine-grained control
    // - Old PyNodePriority enum removed for flexibility

    //  VERSION: Utility function
    m.add_function(wrap_pyfunction!(get_version, m)?)?;

    Ok(())
}

/// Get HORUS version information
#[pyfunction]
fn get_version() -> String {
    format!("HORUS Python Bindings v{}", env!("CARGO_PKG_VERSION"))
}
