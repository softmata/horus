//! FFI wrappers for node building and the CppNode adapter.
//!
//! Rust's `NodeBuilder<'a>` holds `&'a mut Scheduler` — can't cross FFI.
//! Instead we:
//! 1. Accumulate config in `FfiNodeBuilder` (no lifetime)
//! 2. At `build()`, create a `CppNode` and register it with the scheduler
//!
//! `CppNode` implements `horus_core::Node` and delegates `tick()` to a
//! stored function pointer (set by C++ via `node_builder_set_tick`).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use horus_core::core::duration_ext::DurationExt;
use horus_core::core::Node;
use horus_core::error::HorusResult;

use crate::types_ffi::{FfiNodeBuilder, FfiScheduler, NodeConfig};

// ─── CppNode ─────────────────────────────────────────────────────────────────

/// A Node implementation that delegates tick() to a stored closure.
///
/// This is how C++ tick lambdas cross the FFI boundary:
/// C++ sets a function pointer via `node_builder_set_tick`, which gets
/// stored in a `CppNode`. The scheduler calls `CppNode::tick()` which
/// invokes the closure.
///
/// ## Exception Safety
///
/// `tick()` wraps the callback in `std::panic::catch_unwind`. If the callback
/// panics (e.g., from a Rust abort triggered by invalid FFI state), the panic
/// is caught, logged, and the node is marked as failed. Subsequent ticks on
/// a failed node are no-ops.
///
/// **Important**: C++ exceptions cannot cross `extern "C"` boundaries.
/// The C++ caller MUST catch all exceptions before returning from the callback.
/// If a C++ exception unwinds through `extern "C"`, behavior is undefined.
pub struct CppNode {
    name: String,
    tick_fn: Box<dyn FnMut() + Send>,
    failed: bool,
    fail_count: u32,
}

impl CppNode {
    pub fn new(name: String, tick_fn: Box<dyn FnMut() + Send>) -> Self {
        Self {
            name,
            tick_fn,
            failed: false,
            fail_count: 0,
        }
    }

    /// Returns true if the node has panicked and is in a failed state.
    pub fn is_failed(&self) -> bool {
        self.failed
    }
}

impl Node for CppNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn tick(&mut self) {
        if self.failed {
            return; // Skip ticks on failed nodes
        }

        // SAFETY: catch_unwind prevents panics from unwinding across FFI.
        // This is critical — unwinding through extern "C" is UB.
        let tick_fn = &mut self.tick_fn;
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            (tick_fn)();
        }));

        if let Err(panic_info) = result {
            self.failed = true;
            self.fail_count += 1;

            // Extract panic message for logging
            let msg = if let Some(s) = panic_info.downcast_ref::<&str>() {
                s.to_string()
            } else if let Some(s) = panic_info.downcast_ref::<String>() {
                s.clone()
            } else {
                "unknown panic".to_string()
            };

            eprintln!(
                "[horus_cpp] PANIC in C++ node '{}' tick callback: {}. \
                 Node is now disabled (fail_count={}).",
                self.name, msg, self.fail_count
            );
        }
    }
}

// ─── Node Builder FFI ────────────────────────────────────────────────────────

/// Create a new node builder with the given name.
pub fn node_builder_new(name: &str) -> Box<FfiNodeBuilder> {
    let mut config = NodeConfig::default();
    config.name = name.to_string();
    Box::new(FfiNodeBuilder { config })
}

/// Set the node tick rate in Hz.
pub fn node_builder_rate(builder: &mut FfiNodeBuilder, hz: f64) {
    builder.config.rate_hz = Some(hz);
}

/// Set the tick budget in microseconds.
pub fn node_builder_budget(builder: &mut FfiNodeBuilder, budget_us: u64) {
    builder.config.budget_us = Some(budget_us);
}

/// Set the deadline in microseconds.
pub fn node_builder_deadline(builder: &mut FfiNodeBuilder, deadline_us: u64) {
    builder.config.deadline_us = Some(deadline_us);
}

/// Set the deadline miss policy (0=Warn, 1=Skip, 2=SafeMode, 3=Stop).
pub fn node_builder_on_miss(builder: &mut FfiNodeBuilder, policy: u8) {
    builder.config.miss_policy = policy;
}

/// Set execution class to Compute (parallel thread pool).
pub fn node_builder_compute(builder: &mut FfiNodeBuilder) {
    builder.config.execution_class = 2; // Compute
}

/// Set execution class to AsyncIo (tokio blocking pool).
pub fn node_builder_async_io(builder: &mut FfiNodeBuilder) {
    builder.config.execution_class = 4; // AsyncIo
}

/// Set execution class to Event (triggered by topic).
pub fn node_builder_on_topic(builder: &mut FfiNodeBuilder, topic: &str) {
    builder.config.execution_class = 3; // Event
    builder.config.event_topic = Some(topic.to_string());
}

/// Set execution order (lower = earlier).
pub fn node_builder_order(builder: &mut FfiNodeBuilder, order: u32) {
    builder.config.order = order;
}

/// Pin node to a specific CPU core.
pub fn node_builder_pin_core(builder: &mut FfiNodeBuilder, cpu_id: usize) {
    builder.config.pinned_core = Some(cpu_id);
}

/// Set OS scheduling priority (SCHED_FIFO 1-99).
pub fn node_builder_priority(builder: &mut FfiNodeBuilder, prio: i32) {
    builder.config.os_priority = Some(prio);
}

/// Set per-node watchdog timeout in microseconds.
pub fn node_builder_watchdog(builder: &mut FfiNodeBuilder, timeout_us: u64) {
    builder.config.watchdog_us = Some(timeout_us);
}

/// Set the tick callback — an `extern "C" fn()` that the scheduler invokes each tick.
///
/// This is how C++ tick lambdas cross the FFI boundary. The C++ side captures
/// its state in a static or thread-local, and the function pointer invokes it.
///
/// # Safety
/// The function pointer must remain valid for the lifetime of the node.
/// It must not unwind (no C++ exceptions — catch at the call site).
pub fn node_builder_set_tick(builder: &mut FfiNodeBuilder, callback: extern "C" fn()) {
    builder.config.tick_callback = Some(callback);
}

/// Build the node and add it to the scheduler.
///
/// Creates a `CppNode` with a no-op tick function. The real tick function
/// must be set via `node_builder_set_tick` before calling build.
///
/// For now (Phase 3), uses a no-op tick. Phase 4 (CppNodeAdapter) adds
/// the real C++ callback mechanism.
pub fn node_builder_build(
    builder: Box<FfiNodeBuilder>,
    sched: &mut FfiScheduler,
) -> Result<(), String> {
    let config = builder.config;
    let node_name = config.name.clone();

    // Create node with the C++ tick callback (or no-op if none set).
    // The callback is an extern "C" fn() — panics across extern "C" abort
    // the process (Rust 2024 behavior). Our catch_unwind in CppNode::tick()
    // handles panics from *Rust* closures, but extern "C" panics are uncatchable.
    // The C++ side MUST catch all exceptions before returning.
    let tick_fn: Box<dyn FnMut() + Send> = match config.tick_callback {
        Some(cb) => Box::new(move || cb()),
        None => Box::new(|| {}),
    };
    let node = CppNode::new(node_name, tick_fn);

    // Start the builder chain
    let mut nb = sched.inner.add(node);

    // Apply configuration
    if let Some(hz) = config.rate_hz {
        nb = nb.rate((hz as u64).hz());
    }
    if let Some(us) = config.budget_us {
        nb = nb.budget(Duration::from_micros(us));
    }
    if let Some(us) = config.deadline_us {
        nb = nb.deadline(Duration::from_micros(us));
    }

    // Miss policy
    let miss = match config.miss_policy {
        0 => horus_core::core::Miss::Warn,
        1 => horus_core::core::Miss::Skip,
        2 => horus_core::core::Miss::SafeMode,
        3 => horus_core::core::Miss::Stop,
        _ => horus_core::core::Miss::Warn,
    };
    nb = nb.on_miss(miss);

    // Execution class
    match config.execution_class {
        2 => nb = nb.compute(),
        3 => {
            if let Some(ref topic) = config.event_topic {
                nb = nb.on(topic);
            }
        }
        4 => nb = nb.async_io(),
        _ => {} // BestEffort (0) or Rt (1) — Rt auto-detected from rate
    }

    nb = nb.order(config.order);

    if let Some(cpu) = config.pinned_core {
        nb = nb.core(cpu);
    }
    if let Some(prio) = config.os_priority {
        nb = nb.priority(prio);
    }
    if let Some(us) = config.watchdog_us {
        nb = nb.watchdog(Duration::from_micros(us));
    }

    nb.build().map_err(|e| e.to_string())?;
    Ok(())
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scheduler_ffi::{scheduler_new, scheduler_node_list};

    #[test]
    fn build_adds_node_to_scheduler() {
        let mut sched = scheduler_new();
        let builder = node_builder_new("test_node");
        let result = node_builder_build(builder, &mut sched);
        assert!(result.is_ok(), "build failed: {:?}", result);
        assert!(scheduler_node_list(&sched).contains(&"test_node".to_string()));
    }

    #[test]
    fn builder_with_rt_config() {
        let mut sched = scheduler_new();
        let mut builder = node_builder_new("motor_ctrl");
        node_builder_rate(&mut builder, 1000.0);
        node_builder_budget(&mut builder, 800);
        node_builder_deadline(&mut builder, 950);
        node_builder_on_miss(&mut builder, 1); // Skip
        node_builder_order(&mut builder, 5);
        // Note: rate+budget+deadline → auto RT class (no .compute())

        let result = node_builder_build(builder, &mut sched);
        assert!(result.is_ok(), "build failed: {:?}", result);
        assert!(scheduler_node_list(&sched).contains(&"motor_ctrl".to_string()));
    }

    #[test]
    fn builder_with_compute_config() {
        let mut sched = scheduler_new();
        let mut builder = node_builder_new("planner");
        node_builder_compute(&mut builder);
        node_builder_order(&mut builder, 50);

        let result = node_builder_build(builder, &mut sched);
        assert!(result.is_ok(), "build failed: {:?}", result);
        assert!(scheduler_node_list(&sched).contains(&"planner".to_string()));
    }

    #[test]
    fn builder_minimal_config() {
        let mut sched = scheduler_new();
        let builder = node_builder_new("simple");
        assert!(node_builder_build(builder, &mut sched).is_ok());
    }

    #[test]
    fn multiple_nodes() {
        let mut sched = scheduler_new();

        let b1 = node_builder_new("sensor");
        node_builder_build(b1, &mut sched).unwrap();

        let b2 = node_builder_new("processor");
        node_builder_build(b2, &mut sched).unwrap();

        let b3 = node_builder_new("actuator");
        node_builder_build(b3, &mut sched).unwrap();

        let nodes = scheduler_node_list(&sched);
        assert_eq!(nodes.len(), 3);
        assert!(nodes.contains(&"sensor".to_string()));
        assert!(nodes.contains(&"processor".to_string()));
        assert!(nodes.contains(&"actuator".to_string()));
    }

    #[test]
    fn cpp_node_trait_impl() {
        let mut node = CppNode::new("test".to_string(), Box::new(|| {}));
        assert_eq!(node.name(), "test");
        node.tick(); // no-op, should not panic
    }

    // ─── Tick Callback Tests ──────────────────────────────────────────

    static TICK_COUNTER: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

    extern "C" fn test_tick_callback() {
        TICK_COUNTER.fetch_add(1, Ordering::Relaxed);
    }

    #[test]
    fn set_tick_stores_callback() {
        let mut builder = node_builder_new("callback_node");
        node_builder_set_tick(&mut builder, test_tick_callback);
        assert!(builder.config.tick_callback.is_some());
    }

    #[test]
    fn tick_callback_invoked_by_scheduler() {
        TICK_COUNTER.store(0, Ordering::Relaxed);

        let mut sched = scheduler_new();
        let mut builder = node_builder_new("ticking_node");
        node_builder_set_tick(&mut builder, test_tick_callback);
        node_builder_build(builder, &mut sched).unwrap();

        // tick_once should invoke our callback
        crate::scheduler_ffi::scheduler_tick_once(&mut sched).unwrap();
        assert!(TICK_COUNTER.load(Ordering::Relaxed) >= 1,
            "callback should have been invoked at least once, got {}",
            TICK_COUNTER.load(Ordering::Relaxed));
    }

    #[test]
    fn panic_in_tick_caught_and_node_disabled() {
        let mut node = CppNode::new(
            "panicking".to_string(),
            Box::new(|| panic!("test panic from tick")),
        );

        assert!(!node.is_failed());

        // First tick panics — but catch_unwind catches it
        node.tick();
        assert!(node.is_failed(), "node should be marked failed after panic");

        // Subsequent ticks are no-ops (not panics)
        node.tick();
        node.tick();
        assert!(node.is_failed(), "still failed");
    }

    #[test]
    fn catch_unwind_protects_rust_closures() {
        // catch_unwind protects against panics from Rust closures
        // (not extern "C" panics — those abort in Rust 2024+).
        // This test verifies the Rust closure path used by CppNode.
        let mut node = CppNode::new(
            "rust_panicker".to_string(),
            Box::new(|| panic!("rust closure panic")),
        );

        // Panic caught — process survives
        node.tick();
        assert!(node.is_failed());

        // Subsequent ticks are no-ops
        node.tick();
        node.tick();
    }

    #[test]
    fn no_tick_callback_is_noop() {
        let mut sched = scheduler_new();
        let builder = node_builder_new("noop_node");
        // No set_tick — should use no-op
        node_builder_build(builder, &mut sched).unwrap();
        // tick_once should not panic
        crate::scheduler_ffi::scheduler_tick_once(&mut sched).unwrap();
    }

    #[test]
    fn cpp_node_with_closure() {
        let counter = Arc::new(std::sync::atomic::AtomicU32::new(0));
        let counter_clone = counter.clone();
        let mut node = CppNode::new(
            "counting".to_string(),
            Box::new(move || {
                counter_clone.fetch_add(1, Ordering::Relaxed);
            }),
        );

        assert_eq!(counter.load(Ordering::Relaxed), 0);
        node.tick();
        assert_eq!(counter.load(Ordering::Relaxed), 1);
        node.tick();
        node.tick();
        assert_eq!(counter.load(Ordering::Relaxed), 3);
    }
}
