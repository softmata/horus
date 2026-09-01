//! FFI wrappers for node building and the CppNode adapter.
//!
//! Rust's `NodeBuilder<'a>` holds `&'a mut Scheduler` — can't cross FFI.
//! Instead we:
//! 1. Accumulate config in `FfiNodeBuilder` (no lifetime)
//! 2. At `build()`, create a `CppNode` and register it with the scheduler
//!
//! `CppNode` implements `horus_core::Node` and delegates `tick()` to a
//! stored function pointer (set by C++ via `node_builder_set_tick`).

use std::time::Duration;

use horus_core::core::duration_ext::DurationExt;
use horus_core::core::Node;

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
    init_fn: Option<Box<dyn FnMut() + Send>>,
    safe_state_fn: Option<Box<dyn FnMut() + Send>>,
    shutdown_fn: Option<Box<dyn FnMut() + Send>>,
    failed: bool,
    fail_count: u32,
}

impl CppNode {
    pub fn new(name: String, tick_fn: Box<dyn FnMut() + Send>) -> Self {
        Self {
            name,
            tick_fn,
            init_fn: None,
            safe_state_fn: None,
            shutdown_fn: None,
            failed: false,
            fail_count: 0,
        }
    }

    pub fn with_lifecycle(
        mut self,
        init: Option<Box<dyn FnMut() + Send>>,
        safe_state: Option<Box<dyn FnMut() + Send>>,
        shutdown: Option<Box<dyn FnMut() + Send>>,
    ) -> Self {
        self.init_fn = init;
        self.safe_state_fn = safe_state;
        self.shutdown_fn = shutdown;
        self
    }

    /// Returns true if the node has panicked and is in a failed state.
    pub fn is_failed(&self) -> bool {
        self.failed
    }
}

/// The message carried by a caught panic, whatever payload type it used.
fn panic_message(panic_info: &Box<dyn std::any::Any + Send>) -> String {
    if let Some(s) = panic_info.downcast_ref::<&str>() {
        s.to_string()
    } else if let Some(s) = panic_info.downcast_ref::<String>() {
        s.clone()
    } else {
        "unknown panic".to_string()
    }
}

impl Node for CppNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn init(&mut self) -> horus_core::error::HorusResult<()> {
        // Clear the failure latch: `init()` is what `FailurePolicy::Restart`
        // calls to bring a node back. Now that `tick()` panics while `failed`
        // is set, leaving it latched meant a restarted C++ node re-panicked on
        // its very next tick and escalated straight to `FatalAfterRestarts`,
        // stopping the scheduler — so the policy could never actually restart
        // one. `fail_count` is deliberately kept: it is the node's cumulative
        // record, and the restart budget is counted by the policy, not here.
        self.failed = false;
        if let Some(ref mut init_fn) = self.init_fn {
            // The C++ unwind is caught here so it never crosses the FFI
            // boundary, but the FAILURE is reported, not discarded. This used
            // to `let _ = ...; Ok(())`, which told the scheduler the node had
            // initialised cleanly when its constructor had in fact thrown.
            // The scheduler takes real care over init (it wraps `init()` in its
            // own `catch_unwind` and maps a panic to `NodeError::InitPanic`
            // precisely so a node that cannot start is never marked
            // `initialized`) -- and this adapter defeated all of it for every
            // C++ node. A node whose device handle was never opened then
            // ticked forever against uninitialised state.
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                (init_fn)();
            }));
            if let Err(panic_info) = result {
                self.failed = true;
                self.fail_count += 1;
                let msg = panic_message(&panic_info);
                horus_core::terminal::eprint_line(&format!(
                    "[horus_cpp] PANIC in C++ node '{}' init callback: {}.",
                    self.name, msg
                ));
                return Err(horus_core::HorusError::Node(
                    horus_core::error::NodeError::InitFailed {
                        node: self.name.clone(),
                        reason: msg,
                    },
                ));
            }
        }
        Ok(())
    }

    fn tick(&mut self) {
        // A dead C++ node must not look like a healthy one.
        //
        // This used to `return` quietly once `failed` was set, and to swallow
        // the panic that set it. Both are indistinguishable from a successful
        // tick to everything upstream: the scheduler feeds the node's watchdog
        // on a completed tick, `FailurePolicy` is driven by tick panics and saw
        // none, and `is_failed()` is read nowhere outside this file's own unit
        // tests. So a C++ node that died on its first tick kept a green watchdog
        // and a clean failure record for the life of the process, while its
        // control loop did nothing. On a node driving an actuator that is the
        // worst possible way to fail.
        //
        // The C++ unwind is still caught HERE and never crosses the FFI
        // boundary -- that part was always right and is unchanged. What is new
        // is that the adapter then raises a *Rust* panic, from Rust code, which
        // is exactly what the scheduler's `catch_unwind` around `tick()` is for.
        // The failure becomes visible to the machinery built to react to it
        // instead of being absorbed by the adapter.
        if self.failed {
            panic!(
                "C++ node '{}' is disabled after {} panic(s) in its tick callback",
                self.name, self.fail_count
            );
        }

        let tick_fn = &mut self.tick_fn;
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            (tick_fn)();
        }));

        if let Err(panic_info) = result {
            self.failed = true;
            self.fail_count += 1;
            let msg = panic_message(&panic_info);
            horus_core::terminal::eprint_line(&format!(
                "[horus_cpp] PANIC in C++ node '{}' tick callback: {}. \
                 Node is now disabled (fail_count={}).",
                self.name, msg, self.fail_count
            ));
            panic!(
                "C++ node '{}' panicked in its tick callback: {}",
                self.name, msg
            );
        }
    }

    fn enter_safe_state(&mut self) {
        if let Some(ref mut safe_fn) = self.safe_state_fn {
            let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                (safe_fn)();
            }));
        }
    }

    fn shutdown(&mut self) -> horus_core::error::HorusResult<()> {
        // Invoke the C++ on_shutdown callback when the scheduler stops. Without
        // this override the stored `shutdown_fn` was never called (the trait
        // default no-op ran) — see stub audit 2026-07-13. Mirrors `init`.
        if let Some(ref mut shutdown_fn) = self.shutdown_fn {
            let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                (shutdown_fn)();
            }));
        }
        Ok(())
    }
}

// ─── Node Builder FFI ────────────────────────────────────────────────────────

/// Create a new node builder with the given name.
pub fn node_builder_new(name: &str) -> Box<FfiNodeBuilder> {
    let config = NodeConfig {
        name: name.to_string(),
        ..NodeConfig::default()
    };
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

/// Set the init callback — called once before first tick.
pub fn node_builder_set_init(builder: &mut FfiNodeBuilder, callback: extern "C" fn()) {
    builder.config.init_callback = Some(callback);
}

/// Set the enter_safe_state callback — called by safety monitor.
pub fn node_builder_set_safe_state(builder: &mut FfiNodeBuilder, callback: extern "C" fn()) {
    builder.config.safe_state_callback = Some(callback);
}

/// Set the on_shutdown callback — called when scheduler stops.
pub fn node_builder_set_shutdown(builder: &mut FfiNodeBuilder, callback: extern "C" fn()) {
    builder.config.shutdown_callback = Some(callback);
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
#[allow(clippy::boxed_local)]
pub fn node_builder_build(
    builder: Box<FfiNodeBuilder>,
    sched: &mut FfiScheduler,
) -> Result<(), String> {
    let config = builder.config;
    let node_name = config.name.clone();

    // Validate the rate before touching the scheduler. The rate arrives as an
    // f64 (the C++ `Frequency` carries a double) and used to be applied as
    // `(hz as u64).hz()`: that truncated 2.5 Hz to 2 Hz and truncated anything
    // below 1 Hz to 0, where `u64::hz()` asserts — a panic inside an
    // `extern "C"` frame, which aborts the whole process instead of returning
    // an error C++ can see. Reject only genuinely invalid rates, and keep the
    // f64 so fractional rates survive.
    let rate_hz = match config.rate_hz {
        Some(hz) if !hz.is_finite() || hz <= 0.0 => {
            return Err(format!(
                "node '{node_name}': tick rate must be finite and positive (got {hz})"
            ));
        }
        other => other,
    };

    // Create node with the C++ tick callback (or no-op if none set).
    // The callback is an extern "C" fn() — panics across extern "C" abort
    // the process (Rust 2024 behavior). Our catch_unwind in CppNode::tick()
    // handles panics from *Rust* closures, but extern "C" panics are uncatchable.
    // The C++ side MUST catch all exceptions before returning.
    let tick_fn: Box<dyn FnMut() + Send> = match config.tick_callback {
        Some(cb) => Box::new(move || cb()),
        None => Box::new(|| {}),
    };
    let init_fn = config
        .init_callback
        .map(|cb| -> Box<dyn FnMut() + Send> { Box::new(move || cb()) });
    let safe_fn = config
        .safe_state_callback
        .map(|cb| -> Box<dyn FnMut() + Send> { Box::new(move || cb()) });
    let shutdown_fn = config
        .shutdown_callback
        .map(|cb| -> Box<dyn FnMut() + Send> { Box::new(move || cb()) });
    let node = CppNode::new(node_name, tick_fn).with_lifecycle(init_fn, safe_fn, shutdown_fn);

    // Start the builder chain
    let mut nb = sched.inner.add(node);

    // Apply configuration
    if let Some(hz) = rate_hz {
        nb = nb.rate(hz.hz());
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
    use std::sync::atomic::Ordering;
    use std::sync::Arc;

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

    // Regression: a sub-1 Hz rate used to truncate to 0 and hit the `assert!`
    // in `u64::hz()` — a panic raised inside an `extern "C"` frame, i.e. an
    // abort of the whole process. Fractional rates must build, and invalid
    // ones must come back as an ordinary error.
    #[test]
    fn fractional_rate_builds_and_invalid_rate_is_an_error() {
        let mut sched = scheduler_new();
        let mut slow = node_builder_new("telemetry");
        node_builder_rate(&mut slow, 0.5);
        node_builder_build(slow, &mut sched).unwrap();
        assert!(scheduler_node_list(&sched).contains(&"telemetry".to_string()));

        for bad in [0.0_f64, -1.0, f64::NAN, f64::INFINITY] {
            let mut b = node_builder_new("bad_rate");
            node_builder_rate(&mut b, bad);
            assert!(
                node_builder_build(b, &mut sched).is_err(),
                "rate {bad} should be rejected, not applied"
            );
        }
        assert!(!scheduler_node_list(&sched).contains(&"bad_rate".to_string()));
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
        node_builder_build(builder, &mut sched).unwrap();
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
        assert!(
            TICK_COUNTER.load(Ordering::Relaxed) >= 1,
            "callback should have been invoked at least once, got {}",
            TICK_COUNTER.load(Ordering::Relaxed)
        );
    }

    static SHUTDOWN_COUNTER: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    extern "C" fn test_shutdown_callback() {
        SHUTDOWN_COUNTER.fetch_add(1, Ordering::Relaxed);
    }

    #[test]
    fn shutdown_callback_invoked_when_scheduler_stops() {
        SHUTDOWN_COUNTER.store(0, Ordering::Relaxed);

        let mut sched = scheduler_new();
        let mut builder = node_builder_new("shutdown_node");
        node_builder_set_tick(&mut builder, test_tick_callback);
        node_builder_set_shutdown(&mut builder, test_shutdown_callback);
        node_builder_build(builder, &mut sched).unwrap();

        // Run briefly; on stop the scheduler calls node.shutdown(), which must
        // invoke the registered C shutdown callback (regression: it never did).
        let _ = crate::scheduler_ffi::scheduler_run_for(&mut sched, 20_000);

        assert_eq!(
            SHUTDOWN_COUNTER.load(Ordering::Relaxed),
            1,
            "on_shutdown must fire exactly once when the scheduler stops"
        );
    }

    /// A C++ panic is caught at the boundary and then REPORTED, not absorbed.
    ///
    /// This test used to assert the opposite -- "subsequent ticks are no-ops
    /// (not panics)" -- which is what made a dead C++ node indistinguishable
    /// from a healthy one: a quiet return feeds the watchdog and satisfies
    /// FailurePolicy. The C++ unwind is still caught here and never crosses the
    /// FFI boundary; what escapes now is a Rust panic raised from Rust code,
    /// which the scheduler's catch_unwind around tick() is built to handle.
    #[test]
    fn panic_in_tick_is_caught_at_the_boundary_and_reported_upward() {
        let mut node = CppNode::new(
            "panicking".to_string(),
            Box::new(|| panic!("test panic from tick")),
        );

        assert!(!node.is_failed());

        // The C++ unwind is caught; a Rust panic is raised in its place.
        let first = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| node.tick()));
        assert!(first.is_err(), "the first failure must reach the scheduler");
        assert!(node.is_failed(), "node should be marked failed after panic");

        // And it keeps reporting: a disabled node must never read as a healthy
        // tick, or its watchdog is refreshed forever by a node doing nothing.
        for _ in 0..2 {
            let again = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| node.tick()));
            assert!(
                again.is_err(),
                "a disabled node must keep reporting failure, not return quietly"
            );
        }
        assert!(node.is_failed(), "still failed");
    }

    /// A C++ node whose init callback throws must not report a clean start.
    ///
    /// The adapter used to `let _ = catch_unwind(...)` and return `Ok(())`,
    /// which told the scheduler the node had initialised. That mattered more
    /// than it looks: the scheduler wraps `init()` in its own `catch_unwind`
    /// and maps a panic to `NodeError::InitPanic` specifically so that a node
    /// which cannot start is never marked `initialized` -- and this adapter
    /// silently defeated all of it for every C++ node in existence. A node
    /// whose device handle never opened then ticked forever against
    /// uninitialised state.
    #[test]
    fn panic_in_init_is_reported_as_an_init_failure() {
        let mut node = CppNode::new("boot_fail".to_string(), Box::new(|| {}))
            .with_lifecycle(Some(Box::new(|| panic!("device not present"))), None, None);

        let result = node.init();

        assert!(
            result.is_err(),
            "an init callback that threw must not report a clean start"
        );
        let msg = format!("{}", result.unwrap_err());
        assert!(
            msg.contains("device not present"),
            "the failure must carry the reason the node could not start, got: {msg}"
        );

        // The unwind was contained at the boundary -- this thread is still alive
        // to run the assertion, which is the part that was always correct.
    }

    /// A node that initialises cleanly still reports success, and `init()`
    /// still clears the failure latch so `FailurePolicy::Restart` can work.
    #[test]
    fn a_clean_init_still_succeeds_and_clears_the_failure_latch() {
        let mut node = CppNode::new(
            "restartable".to_string(),
            Box::new(|| panic!("tick always fails")),
        )
        .with_lifecycle(Some(Box::new(|| {})), None, None);

        let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| node.tick()));
        assert!(node.is_failed(), "precondition: the node is latched failed");

        assert!(node.init().is_ok(), "a clean init callback must succeed");
        assert!(
            !node.is_failed(),
            "init() is what Restart calls to bring a node back; it must unlatch"
        );
    }

    #[test]
    fn catch_unwind_protects_rust_closures() {
        // catch_unwind protects against panics from Rust closures
        // (not extern "C" panics — those abort in Rust 2024+).
        // This test verifies the Rust closure path used by CppNode.
        //
        // The property under test is that the callback's unwind is contained at
        // the boundary and the PROCESS survives -- that is unchanged. What the
        // adapter does after containing it changed: it re-raises a Rust panic so
        // the scheduler learns the node is dead, instead of returning quietly
        // and reading as a healthy tick.
        let mut node = CppNode::new(
            "rust_panicker".to_string(),
            Box::new(|| panic!("rust closure panic")),
        );

        // Contained at the boundary: this thread lives to run the assertion.
        let first = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| node.tick()));
        assert!(first.is_err(), "the failure is reported, not absorbed");
        assert!(node.is_failed());

        // Still reporting, still not aborting the process.
        for _ in 0..2 {
            let again = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| node.tick()));
            assert!(again.is_err());
        }
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
