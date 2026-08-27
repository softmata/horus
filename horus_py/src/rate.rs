//! Python binding for [`Rate`] — drift-compensated loop pacing.

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use horus_core::core::timer::Rate;

/// Drift-compensated rate limiter for background threads.
///
/// Unlike ``time.sleep(1/hz)``, Rate compensates for execution time drift
/// to maintain accurate target frequency over many iterations.
///
/// Example:
///
/// ```python
/// from horus import Rate
///
/// rate = Rate(30)  # 30 Hz
/// while running:
///     do_work()
///     rate.sleep()  # sleeps just enough to maintain 30 Hz
///
/// print(f"Actual: {rate.actual_hz():.1f} Hz")
/// ```
///
/// Note: ``sleep()`` blocks the calling thread, but releases the GIL for the
/// duration of the sleep, so other Python threads keep running.
#[pyclass(name = "Rate")]
pub struct PyRate {
    inner: Rate,
}

#[pymethods]
impl PyRate {
    /// Create a rate limiter targeting ``hz`` Hz.
    ///
    /// Raises ``ValueError`` if hz is not positive and finite.
    #[new]
    fn new(hz: f64) -> PyResult<Self> {
        if hz <= 0.0 || !hz.is_finite() {
            return Err(PyValueError::new_err(format!(
                "Rate: frequency must be positive and finite (got {})",
                hz
            )));
        }
        Ok(PyRate {
            inner: Rate::new(hz),
        })
    }

    /// Sleep for the remainder of the current period.
    ///
    /// If work took longer than the period, sleep is skipped and the next
    /// cycle catches up (drift compensation).
    ///
    /// Releases the GIL for the duration of the sleep.
    // This is a real blocking OS sleep of up to one full period. It used to run
    // with the GIL held, which froze every other Python thread for that whole
    // period — the GIL is process-wide, so running the loop in a "dedicated
    // background thread" was no escape from it.
    fn sleep(&mut self, py: Python<'_>) {
        let inner = &mut self.inner;
        py.detach(|| inner.sleep());
    }

    /// Actual achieved frequency in Hz (exponentially smoothed).
    fn actual_hz(&self) -> f64 {
        self.inner.actual_hz()
    }

    /// Target frequency in Hz.
    fn target_hz(&self) -> f64 {
        self.inner.target_hz()
    }

    /// Target period in seconds.
    fn period(&self) -> f64 {
        self.inner.period().as_secs_f64()
    }

    /// Reset cycle start to now (use after a long pause).
    fn reset(&mut self) {
        self.inner.reset();
    }

    /// Whether the current cycle has exceeded the target period.
    fn is_late(&self) -> bool {
        self.inner.is_late()
    }

    fn __repr__(&self) -> String {
        format!(
            "Rate(target={:.1}Hz, actual={:.1}Hz)",
            self.target_hz(),
            self.actual_hz()
        )
    }
}
