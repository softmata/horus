use pyo3::prelude::*;

mod config;
mod depth_image;
mod dlpack_utils;
mod driver_params;
mod drivers;
pub mod errors;
mod image;
mod messages;
mod node;
mod params;
mod perception;
mod pointcloud;
mod rate;
mod scheduler;
mod tensor;
mod topic;
mod transform_frame;
mod types;

use config::PySchedulerConfig;
use node::{PyNodeInfo, PyNodeState};
use scheduler::{PyMiss, PyScheduler};
use topic::PyTopic;
use transform_frame::{PyTransform, PyTransformFrame, PyTransformFrameConfig};
use types::Priority;

/// HORUS Python Bindings
#[pymodule]
fn _horus(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Core classes
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

    // Tensor system (public API)
    m.add_class::<tensor::PyTensorHandle>()?;
    m.add_class::<tensor::PyTensorPool>()?;

    // Runtime parameters
    m.add_class::<params::PyParams>()?;

    // Rate limiter
    m.add_class::<rate::PyRate>()?;

    // Perception types
    perception::register_perception_module(m)?;

    // Drivers module
    drivers::register_drivers_module(m)?;

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

    // Time API — horus.now(), horus.dt(), etc.
    m.add_function(wrap_pyfunction!(time_now, m)?)?;
    m.add_function(wrap_pyfunction!(time_dt, m)?)?;
    m.add_function(wrap_pyfunction!(time_elapsed, m)?)?;
    m.add_function(wrap_pyfunction!(time_tick, m)?)?;
    m.add_function(wrap_pyfunction!(time_budget_remaining, m)?)?;
    m.add_function(wrap_pyfunction!(time_rng_float, m)?)?;

    Ok(())
}

/// Get HORUS version information
#[pyfunction]
fn get_version() -> String {
    format!("HORUS Python Bindings v{}", env!("CARGO_PKG_VERSION"))
}

// ── Time API ──────────────────────────────────────────────────────────────────

/// Current framework time in seconds.
///
/// Normal mode: wall clock. Deterministic mode: virtual SimClock.
#[pyfunction]
fn time_now() -> f64 {
    let ts = horus_core::core::tick_context::ctx_now();
    ts.as_nanos() as f64 / 1_000_000_000.0
}

/// Timestep for this tick in seconds.
///
/// Normal mode: actual elapsed. Deterministic mode: fixed 1/rate.
#[pyfunction]
fn time_dt() -> f64 {
    horus_core::core::tick_context::ctx_dt().as_secs_f64()
}

/// Time elapsed since scheduler start in seconds.
#[pyfunction]
fn time_elapsed() -> f64 {
    horus_core::core::tick_context::ctx_elapsed().as_secs_f64()
}

/// Current tick number.
#[pyfunction]
fn time_tick() -> u64 {
    horus_core::core::tick_context::ctx_tick()
}

/// Time remaining in this tick's budget in seconds.
///
/// Returns float('inf') if no budget configured.
#[pyfunction]
fn time_budget_remaining() -> f64 {
    let remaining = horus_core::core::tick_context::ctx_budget_remaining();
    if remaining == std::time::Duration::MAX {
        f64::INFINITY
    } else {
        remaining.as_secs_f64()
    }
}

/// Random float in [0.0, 1.0) from the deterministic RNG.
///
/// Normal mode: system entropy. Deterministic mode: tick-seeded.
#[pyfunction]
fn time_rng_float() -> f64 {
    horus_core::core::tick_context::ctx_with_rng(|rng| {
        use rand::Rng;
        rng.gen::<f64>()
    })
}
