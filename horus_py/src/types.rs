use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

/// Priority constants for Python users
///
/// Priorities are now represented as u32 values for maximum flexibility.
/// Lower numbers = higher priority. You can use any u32 value, but here
/// are some common presets:
///
/// - PRIORITY_CRITICAL = 0 (highest priority, for emergency/safety nodes)
/// - PRIORITY_HIGH = 10 (for important control loops)
/// - PRIORITY_NORMAL = 50 (default priority for most nodes)
/// - PRIORITY_LOW = 80 (for background processing)
/// - PRIORITY_BACKGROUND = 100 (lowest priority, for logging/telemetry)
///
/// You can also use any custom value between 0-255 for fine-grained control.
#[pyclass]
pub struct Priority;

#[pymethods]
impl Priority {
    #[classattr]
    const CRITICAL: u32 = 0;

    #[classattr]
    const HIGH: u32 = 10;

    #[classattr]
    const NORMAL: u32 = 50;

    #[classattr]
    const LOW: u32 = 80;

    #[classattr]
    const BACKGROUND: u32 = 100;

    /// Parse a priority string into a u32 value
    #[staticmethod]
    fn from_string(priority: String) -> PyResult<u32> {
        match priority.to_lowercase().as_str() {
            "critical" => Ok(0),
            "high" => Ok(10),
            "normal" => Ok(50),
            "low" => Ok(80),
            "background" => Ok(100),
            _ => {
                // Try parsing as number
                priority.parse::<u32>().map_err(|_| {
                    PyValueError::new_err(format!(
                        "Invalid priority '{}'. Use 'critical', 'high', 'normal', 'low', 'background', or a numeric value (0-255)",
                        priority
                    ))
                })
            }
        }
    }

    /// Convert a u32 priority to a descriptive string
    #[staticmethod]
    fn to_string(priority: u32) -> String {
        match priority {
            0 => "critical".to_string(),
            10 => "high".to_string(),
            50 => "normal".to_string(),
            80 => "low".to_string(),
            100 => "background".to_string(),
            n => format!("custom({})", n),
        }
    }
}
