//! FFI wrappers for `horus_core::Scheduler`.
//!
//! The Scheduler has two phases:
//! 1. **Building** — chainable builder methods (tick_rate, prefer_rt, etc.)
//!    Rust uses `mut self` (move semantics). For FFI, we rebuild the scheduler
//!    in-place by replacing the inner value.
//! 2. **Running** — run(), tick_once(), stop()
//!    Rust uses `&mut self` / `&self`. FFI passes `&mut FfiScheduler`.
//!
//! All functions are `pub` for use by the CXX bridge (to be wired in later phases).

use std::time::Duration;

use horus_core::core::duration_ext::DurationExt;
use horus_core::scheduling::Scheduler;

use crate::types_ffi::FfiScheduler;

// ─── Constructor ─────────────────────────────────────────────────────────────

/// Create a new Scheduler with default configuration.
///
/// C++ usage: `auto sched = scheduler_new();`
pub fn scheduler_new() -> Box<FfiScheduler> {
    Box::new(FfiScheduler {
        inner: Scheduler::new(),
    })
}

/// Create a new Scheduler with ABI version verification.
///
/// `compiled_abi_version` is the HORUS_CPP_ABI_VERSION constant that was
/// embedded in the C++ headers at compile time. If it doesn't match the
/// runtime version, returns an error with both versions for diagnostics.
pub fn scheduler_new_checked(compiled_abi_version: u32) -> Result<Box<FfiScheduler>, String> {
    if !crate::types_ffi::verify_abi_version(compiled_abi_version) {
        return Err(format!(
            "HORUS ABI version mismatch: C++ headers compiled with ABI v{}, \
             but horus_cpp runtime is ABI v{}. Rebuild your C++ project with: horus build --clean",
            compiled_abi_version,
            crate::types_ffi::HORUS_CPP_ABI_VERSION,
        ));
    }
    Ok(scheduler_new())
}

// ─── Builder Methods ─────────────────────────────────────────────────────────
//
// Rust's builder pattern uses `mut self` (consuming). For FFI, we can't move
// out of a Box, so we use std::mem::replace to swap the inner scheduler
// with the builder-modified version.

/// Set the scheduler tick rate in Hz.
///
/// C++ usage: `scheduler_tick_rate(sched, 100.0);`
pub fn scheduler_tick_rate(sched: &mut FfiScheduler, hz: f64) {
    let inner = std::mem::replace(&mut sched.inner, Scheduler::new());
    sched.inner = inner.tick_rate((hz as u64).hz());
}

/// Set the scheduler name.
pub fn scheduler_name(sched: &mut FfiScheduler, name: &str) {
    let inner = std::mem::replace(&mut sched.inner, Scheduler::new());
    sched.inner = inner.name(name);
}

/// Prefer real-time scheduling (graceful degradation if unavailable).
pub fn scheduler_prefer_rt(sched: &mut FfiScheduler) {
    let inner = std::mem::replace(&mut sched.inner, Scheduler::new());
    sched.inner = inner.prefer_rt();
}

/// Require real-time scheduling (fail if unavailable).
pub fn scheduler_require_rt(sched: &mut FfiScheduler) {
    let inner = std::mem::replace(&mut sched.inner, Scheduler::new());
    sched.inner = inner.require_rt();
}

/// Enable or disable deterministic mode (SimClock + seeded RNG).
pub fn scheduler_deterministic(sched: &mut FfiScheduler, enabled: bool) {
    let inner = std::mem::replace(&mut sched.inner, Scheduler::new());
    sched.inner = inner.deterministic(enabled);
}

/// Enable or disable verbose logging.
pub fn scheduler_verbose(sched: &mut FfiScheduler, enabled: bool) {
    let inner = std::mem::replace(&mut sched.inner, Scheduler::new());
    sched.inner = inner.verbose(enabled);
}

/// Set the global watchdog timeout in microseconds.
pub fn scheduler_watchdog(sched: &mut FfiScheduler, timeout_us: u64) {
    let inner = std::mem::replace(&mut sched.inner, Scheduler::new());
    sched.inner = inner.watchdog(Duration::from_micros(timeout_us));
}

/// Set BlackBox flight recorder size in MB.
pub fn scheduler_blackbox(sched: &mut FfiScheduler, size_mb: usize) {
    let inner = std::mem::replace(&mut sched.inner, Scheduler::new());
    sched.inner = inner.blackbox(size_mb);
}

/// Enable network replication (horus_net).
/// Note: network is on by default. This is kept for C API backward compat.
pub fn scheduler_enable_network(sched: &mut FfiScheduler) {
    let inner = std::mem::replace(&mut sched.inner, Scheduler::new());
    sched.inner = inner.network(true);
}

// ─── Lifecycle ───────────────────────────────────────────────────────────────

/// Run the scheduler (blocks until stopped or error).
///
/// Returns Ok(()) on clean shutdown, Err on failure.
pub fn scheduler_run(sched: &mut FfiScheduler) -> Result<(), String> {
    sched.inner.run().map_err(|e| e.to_string())
}

/// Run the scheduler for a fixed duration (microseconds).
pub fn scheduler_run_for(sched: &mut FfiScheduler, duration_us: u64) -> Result<(), String> {
    sched
        .inner
        .run_for(Duration::from_micros(duration_us))
        .map_err(|e| e.to_string())
}

/// Execute a single tick of all nodes.
pub fn scheduler_tick_once(sched: &mut FfiScheduler) -> Result<(), String> {
    sched.inner.tick_once().map_err(|e| e.to_string())
}

/// Stop the scheduler (can be called from any thread).
pub fn scheduler_stop(sched: &FfiScheduler) {
    sched.inner.stop();
}

// ─── Query ───────────────────────────────────────────────────────────────────

/// Check if the scheduler is still running.
///
/// This flag is set to `false` by:
/// - `scheduler_stop()` (programmatic shutdown from C++)
/// - Ctrl+C signal handler (horus_core's ctrlc handler)
/// - Deadline miss with `Miss::Stop` policy
/// - Emergency stop trigger
///
/// C++ tick callbacks can check this to exit early:
/// ```cpp
/// .tick([&] {
///     if (!scheduler_is_running(sched)) return;
///     // ... normal tick logic
/// })
/// ```
pub fn scheduler_is_running(sched: &FfiScheduler) -> bool {
    sched.inner.is_running()
}

/// Get the scheduler name.
pub fn scheduler_get_name(sched: &FfiScheduler) -> String {
    sched.inner.scheduler_name().to_string()
}

/// Get a human-readable status string.
pub fn scheduler_status(sched: &FfiScheduler) -> String {
    sched.inner.status()
}

/// Check if full RT capabilities are available.
pub fn scheduler_has_full_rt(sched: &FfiScheduler) -> bool {
    sched.inner.has_full_rt()
}

/// Get list of registered node names.
pub fn scheduler_node_list(sched: &FfiScheduler) -> Vec<String> {
    sched.inner.node_list()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_creates_valid_scheduler() {
        let sched = scheduler_new();
        assert!(scheduler_is_running(&sched));
    }

    #[test]
    fn tick_rate_sets_rate() {
        let mut sched = scheduler_new();
        scheduler_tick_rate(&mut sched, 100.0);
        // Scheduler stores config internally — verify no panic
    }

    #[test]
    fn name_sets_name() {
        let mut sched = scheduler_new();
        scheduler_name(&mut sched, "test_scheduler");
        assert_eq!(scheduler_get_name(&sched), "test_scheduler");
    }

    #[test]
    fn builder_chain_works() {
        let mut sched = scheduler_new();
        scheduler_tick_rate(&mut sched, 100.0);
        scheduler_prefer_rt(&mut sched);
        scheduler_deterministic(&mut sched, true);
        scheduler_verbose(&mut sched, false);
        scheduler_watchdog(&mut sched, 5_000_000); // 5 seconds
        // No panic = success
    }

    #[test]
    fn stop_sets_not_running() {
        let sched = scheduler_new();
        assert!(scheduler_is_running(&sched));
        scheduler_stop(&sched);
        assert!(!scheduler_is_running(&sched));
    }

    #[test]
    fn status_returns_string() {
        let sched = scheduler_new();
        let status = scheduler_status(&sched);
        assert!(!status.is_empty());
    }

    #[test]
    fn node_list_empty_initially() {
        let sched = scheduler_new();
        assert!(scheduler_node_list(&sched).is_empty());
    }

    #[test]
    fn tick_once_on_empty_scheduler() {
        let mut sched = scheduler_new();
        // tick_once on empty scheduler should succeed (no nodes to tick)
        let result = scheduler_tick_once(&mut sched);
        assert!(result.is_ok(), "tick_once failed: {:?}", result);
    }
}
