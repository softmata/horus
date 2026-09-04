//! Async I/O executor for I/O-bound nodes.
//!
//! Nodes registered with `.async_io()` are dispatched here. Each node's `tick()`
//! is run via `tokio::task::spawn_blocking()` on a dedicated tokio runtime, keeping
//! I/O-bound work off the RT and compute threads.
//!
//! # Architecture
//!
//! ```text
//!  AsyncExecutor
//!  ┌──────────────────────────────────────────┐
//!  │  Dedicated thread running tokio runtime:  │
//!  │  ┌─ tick loop ─────────────────────────┐ │
//!  │  │  for each ready node:               │ │
//!  │  │    spawn_blocking(node.tick())       │ │
//!  │  │  await all handles                  │ │
//!  │  │  process results                    │ │
//!  │  │  sleep until next tick              │ │
//!  │  └─────────────────────────────────────┘ │
//!  │  shared: running (AtomicBool)            │
//!  └──────────────────────────────────────────┘
//! ```
//!
//! The tokio pool is separate from the compute crossbeam pool, so I/O-bound
//! nodes never steal compute threads. Per-node rate limiting is respected.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::terminal::print_line;

use super::primitives::NodeRunner;
use super::types::{RegisteredNode, SharedMonitors};

/// Async I/O executor that runs node ticks via tokio::task::spawn_blocking.
///
/// Owns async I/O nodes and ticks them on a dedicated tokio runtime.
/// Shutdown is coordinated via the shared `running` flag.
pub(crate) struct AsyncExecutor {
    handle: Option<std::thread::JoinHandle<Vec<RegisteredNode>>>,
}

impl AsyncExecutor {
    /// Start the async I/O executor with the given nodes.
    ///
    /// Spawns a thread with its own tokio runtime. Each tick cycle, ready nodes
    /// are dispatched via `spawn_blocking` and awaited concurrently.
    pub fn start(
        mut nodes: Vec<RegisteredNode>,
        running: Arc<AtomicBool>,
        tick_period: Duration,
        monitors: SharedMonitors,
    ) -> Self {
        nodes.sort_by_key(|n| n.priority);

        let handle = std::thread::Builder::new()
            .name("horus-async-io".to_string())
            .spawn(move || Self::async_thread_main(nodes, running, tick_period, monitors))
            .expect("Failed to spawn async I/O thread");

        Self {
            handle: Some(handle),
        }
    }

    /// Stop the async I/O executor and reclaim its nodes.
    pub fn stop(mut self) -> Vec<RegisteredNode> {
        // Degrade instead of panicking, matching EventExecutor::stop.
        //
        // Both `expect`s used to fire on the MAIN thread during teardown: a
        // panic anywhere inside a async i/o node's tick killed only that
        // executor thread (there is no `panic = "abort"` in the release
        // profile), and the panic then re-surfaced here — turning an orderly
        // scheduler shutdown into a panic at exactly the moment a robot is
        // trying to stop. Any remaining shutdown work, including safing other
        // nodes, was skipped.
        //
        // The nodes owned by a panicked thread cannot be reclaimed, so an empty
        // Vec is the honest result; it is reported rather than swallowed.
        let Some(handle) = self.handle.take() else {
            print_line("[Async I/O] Warning: thread handle already consumed — nothing to join");
            return Vec::new();
        };
        // Bounded join, matching the guarantee RtExecutor::stop documents. A
        // bare `handle.join()` here hung the entire shutdown whenever an async
        // node blocked inside `tick()`, because this loop only re-checks
        // `running` between ticks — and `run_with_filter` calls this before it
        // shuts down or safes any other node.
        super::primitives::join_with_timeout(
            handle,
            "Async I/O",
            super::primitives::SHUTDOWN_TIMEOUT_PER_THREAD,
        )
        .unwrap_or_default()
    }

    /// Main function for the async I/O thread.
    ///
    /// Creates a dedicated tokio runtime and runs the tick loop on it.
    fn async_thread_main(
        mut nodes: Vec<RegisteredNode>,
        running: Arc<AtomicBool>,
        tick_period: Duration,
        monitors: SharedMonitors,
    ) -> Vec<RegisteredNode> {
        let rt = match tokio::runtime::Builder::new_current_thread()
            .enable_time()
            .build()
        {
            Ok(rt) => rt,
            Err(e) => {
                print_line(&format!("[AsyncIO] Failed to create tokio runtime: {}", e));
                return nodes;
            }
        };

        rt.block_on(async {
            print_line(&format!(
                "[AsyncIO] Started with {} nodes, tick period {:?}",
                nodes.len(),
                tick_period
            ));

            while running.load(Ordering::Relaxed) {
                let loop_start = Instant::now();

                // Classify which nodes should tick this cycle
                let mut ready_indices = Vec::new();
                // Safing requested by the main thread's watchdog ladder,
                // applied to EVERY node this executor owns — an Isolated node
                // is precisely one that is not in `ready_indices`.
                for node in nodes.iter_mut() {
                    super::primitives::honor_safe_state_request(node, &monitors);
                }

                for (i, node) in nodes.iter().enumerate() {
                    if !node.initialized
                        || node.is_stopped
                        || node.is_paused
                        || !node.failure_policy_allows_tick()
                    {
                        continue;
                    }

                    // Per-node rate limiting
                    if let Some(rate_hz) = node.rate_hz {
                        if let Some(last_tick) = node.last_tick {
                            let elapsed = loop_start.duration_since(last_tick).as_secs_f64();
                            if rate_hz > 0.0 && elapsed < 1.0 / rate_hz {
                                continue;
                            }
                        }
                    }

                    ready_indices.push(i);
                }

                if ready_indices.is_empty() {
                    let elapsed = loop_start.elapsed();
                    if elapsed < tick_period {
                        tokio::time::sleep(tick_period - elapsed).await;
                    }
                    continue;
                }

                // Update last_tick for rate-limited nodes and begin recording
                let now = Instant::now();
                for &i in &ready_indices {
                    if nodes[i].rate_hz.is_some() {
                        nodes[i].last_tick = Some(now);
                    }
                    if let Some(ref mut ctx) = nodes[i].context {
                        ctx.start_tick();
                    }
                    if let Some(ref mut recorder) = nodes[i].recorder {
                        recorder.begin_tick(0);
                    }
                }

                // Dispatch all ready nodes via spawn_blocking concurrently.
                // We need to send node references across the blocking boundary.
                // Use raw pointers + join handles (same pattern as crossbeam scope
                // but via tokio spawn_blocking).
                struct AsyncResult {
                    index: usize,
                    tick_start: Instant,
                    duration: Duration,
                    result: std::thread::Result<()>,
                }

                let nodes_ptr = nodes.as_mut_ptr();
                let mut handles = Vec::with_capacity(ready_indices.len());

                for &i in &ready_indices {
                    // SAFETY: each task gets a `&mut` to a distinct element of
                    // `nodes`, so the tasks never alias each other. The lifetime
                    // erasure (spawn_blocking demands 'static) is sound only
                    // because EVERY handle below is awaited to completion before
                    // `nodes` is reborrowed or dropped — see the two-phase drain
                    // after this loop. Reborrowing while a task is still running
                    // would invalidate that task's pointer.
                    let node_ref = unsafe { &mut *nodes_ptr.add(i) };
                    // FIX #5: the tick runs on a tokio blocking-pool thread, so the
                    // thread-local context is set INSIDE the closure. spawn_blocking
                    // requires 'static, so move an owned clock Arc clone (cheap).
                    let clock = monitors.clock.clone();
                    let ctx_tick_period = monitors.tick_period;
                    let handle = tokio::task::spawn_blocking(move || {
                        super::primitives::set_node_tick_context(
                            node_ref,
                            &*clock,
                            ctx_tick_period,
                        );
                        let tr = NodeRunner::run_tick(&mut node_ref.node);
                        super::primitives::clear_node_tick_context();
                        AsyncResult {
                            index: i,
                            tick_start: tr.tick_start,
                            duration: tr.duration,
                            result: tr.result,
                        }
                    });
                    handles.push(handle);
                }

                // Two phases, deliberately. This used to await and process one
                // handle at a time, but `&mut nodes[i]` goes through
                // `<Vec<T>>::index_mut`, which takes a unique borrow of the
                // WHOLE buffer — invalidating every pointer the still-running
                // sibling tasks derived from `nodes_ptr`. Aliasing UB, and the
                // resulting `noalias` slice pointer let the compiler reorder
                // reads of a node another blocking thread was still writing.
                // `ComputeExecutor::compute_thread_main` gets this right by
                // joining its crossbeam scope before touching results.
                //
                // Draining first also closes the unwind window: no user code
                // (`on_error`, `apply_failure_policy_after_panic`) runs — and so
                // cannot unwind and drop `nodes` — while a blocking thread still
                // holds a pointer into it.

                // Phase 1: drain every handle, touching `nodes` not at all.
                let mut results = Vec::with_capacity(handles.len());
                for handle in handles {
                    match handle.await {
                        Ok(ar) => results.push(ar),
                        Err(e) => {
                            print_line(&format!("[AsyncIO] spawn_blocking panicked: {}", e));
                        }
                    }
                }

                // Phase 2: no task is live any more, so `nodes` may be reborrowed.
                for ar in results {
                    let tr = super::primitives::TickResult {
                        tick_start: ar.tick_start,
                        duration: ar.duration,
                        result: ar.result,
                    };
                    Self::process_node_result(&mut nodes[ar.index], tr, &running, &monitors);
                }

                // Sleep until next tick period
                let elapsed = loop_start.elapsed();
                if elapsed < tick_period {
                    tokio::time::sleep(tick_period - elapsed).await;
                }
            }

            print_line(&format!(
                "[AsyncIO] Stopped ({} nodes returning to scheduler)",
                nodes.len()
            ));

            nodes
        })
    }

    /// Process the result of a single node tick.
    fn process_node_result(
        node: &mut RegisteredNode,
        tr: super::primitives::TickResult,
        running: &Arc<AtomicBool>,
        monitors: &SharedMonitors,
    ) {
        // Record execution stats
        if let Some(ref mut stats) = node.rt_stats {
            stats.record_execution(tr.duration);
        }

        // Profiler recording (shared with main thread)
        // Use try_lock to avoid priority inversion — skip if contended or poisoned
        if let Ok(mut profiler) = monitors.profiler.try_lock() {
            profiler.record(&node.name, tr.duration);
        }

        // End recording tick
        if let Some(ref mut recorder) = node.recorder {
            recorder.end_tick(tr.duration.as_nanos() as u64);
        }

        // Update live SHM registry (~5ns atomic writes)
        monitors.update_registry(node, tr.duration.as_nanos() as u64);

        match tr.result {
            Ok(_) => {
                if let Some(ref mut ctx) = node.context {
                    ctx.record_tick();
                }
                node.record_tick_success();
                // FIX #2: feed the watchdog after a successful tick, gated on the
                // main loop's critical-node condition (mod.rs:3555). Reached only
                // when the spawn_blocking task RETURNED (a hung task blocks its
                // await so this never runs → hang-detection preserved); the Err
                // arm below is skipped so a panic does not feed either.
                if node.is_rt_node || node.node_watchdog.is_some() {
                    if let Some(ref feeder) = monitors.watchdog {
                        feeder.feed(&node.name);
                    }
                }
            }
            Err(panic_err) => {
                if let Ok(mut profiler) = monitors.profiler.try_lock() {
                    profiler.record_node_failure(&node.name);
                }
                let error_msg = if let Some(s) = panic_err.downcast_ref::<&str>() {
                    format!("[AsyncIO] Node '{}' panicked: {}", node.name, s)
                } else if let Some(s) = panic_err.downcast_ref::<String>() {
                    format!("[AsyncIO] Node '{}' panicked: {}", node.name, s)
                } else {
                    format!("[AsyncIO] Node '{}' panicked (unknown)", node.name)
                };
                // Count the failure. Only the Ok arm recorded metrics, so a node
                // panicking on every tick reported `Health: Healthy, Errors: 0,
                // Total Ticks: 0` while its P99 timing was recorded correctly —
                // making the zero read as "idle" rather than "dead".
                if let Some(ref mut ctx) = node.context {
                    ctx.record_tick_failure(error_msg.clone());
                }

                // Reflect sustained failure in the node's health state. The
                // watchdog/deadline ladder is the only other writer, so a node that
                // panicked every tick but never missed a *timing* target reported
                // `Health: Healthy` forever — 232 errors out of 237 ticks, green.
                //
                // One panic can be transient; 3 consecutive is a state change.
                if let Some(ref ctx) = node.context {
                    if ctx.consecutive_failures() >= super::primitives::FAILURES_BEFORE_UNHEALTHY {
                        node.health_state
                            .store(super::types::NodeHealthState::Unhealthy);
                        monitors.node_controls.set_health(
                            node.name.as_ref(),
                            super::types::NodeHealthState::Unhealthy,
                        );
                    }
                }

                // Record to the blackbox so the flight recorder can see a crash.
                // try_lock mirrors the RT path: never block an executor on it.
                if let Some(ref bb) = monitors.blackbox {
                    if let Ok(mut bb) = bb.try_lock() {
                        bb.record(super::blackbox::BlackBoxEvent::NodeError {
                            name: node.name.to_string(),
                            error: error_msg.clone(),
                            severity: crate::error::Severity::Fatal,
                        });
                    }
                }

                // `record_tick_failure` above already logged this at error level, which
                // reaches both the console and the buffer `horus log` reads. This second
                // copy was gated on `verbose` on the theory that verbose is opt-in — but
                // `MonitoringConfig::verbose` defaults to *true*, so every default run
                // printed the panic twice on two different streams (hlog to stderr, this to
                // stdout), and a third time via the old `Node::on_error` default. With a
                // Python node's traceback attached that is three multi-line blocks for one
                // failure.
                //
                // Panic-guarded: `process_node_result` runs inside `block_on`
                // on the async I/O thread outside any catch_unwind (the tick
                // itself is isolated by `spawn_blocking`), so a bare panic in
                // this advisory callback killed the executor thread — and with
                // it every healthy node it owns — while `run_for` still
                // returned Ok and `stop()` reclaimed nothing.
                if super::primitives::guard_fault_callback(|| node.node.on_error(&error_msg)) {
                    print_line(&format!(
                        "[AsyncIO] Node '{}' also panicked in on_error() — ignoring (advisory callback)",
                        node.name
                    ));
                }

                // Enforce the failure policy (Fatal → safe + stop via shared
                // `running`; Restart → re-init; Skip/Ignore → gated next tick).
                if node.apply_failure_policy_after_panic() {
                    running.store(false, Ordering::SeqCst);
                }
            }
        }
    }
}

impl Drop for AsyncExecutor {
    fn drop(&mut self) {
        // Bounded here too: an early return or a panic can drop the executor
        // without ever calling `stop()`, and an unbounded join on that path
        // hangs exactly as badly.
        if let Some(handle) = self.handle.take() {
            let _ = super::primitives::join_with_timeout(
                handle,
                "Async I/O",
                super::primitives::SHUTDOWN_TIMEOUT_PER_THREAD,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::duration_ext::DurationExt;
    use crate::core::{Miss, Node, NodeInfo};
    use std::sync::atomic::AtomicU64;
    use std::sync::Mutex;

    fn test_monitors() -> SharedMonitors {
        SharedMonitors {
            profiler: Arc::new(Mutex::new(super::super::profiler::RuntimeProfiler::new())),
            blackbox: None,
            verbose: true,
            registry: None,
            registry_slots: Arc::new(std::collections::HashMap::new()),
            node_controls: Arc::new(super::super::types::NodeControlMap::default()),
            clock: Arc::new(crate::core::clock::WallClock::new()),
            tick_period: Duration::from_millis(1),
            watchdog: None,
            estop: None,
            safety: None,
        }
    }

    struct CounterNode {
        name: String,
        count: Arc<AtomicU64>,
    }

    impl Node for CounterNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// A node that simulates blocking I/O by sleeping.
    struct SlowIoNode {
        name: String,
        sleep_ms: u64,
        count: Arc<AtomicU64>,
    }

    impl Node for SlowIoNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.count.fetch_add(1, Ordering::Relaxed);
            std::thread::sleep(self.sleep_ms.ms());
        }
    }

    fn make_async_node(name: &str, count: Arc<AtomicU64>) -> RegisteredNode {
        let node = CounterNode {
            name: name.to_string(),
            count,
        };
        RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(node)),
            name: Arc::from(name),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new(name.to_string())),
            rate_hz: None,
            last_tick: None,
            is_rt_node: false,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            health_probe_counter: 0,
            is_paused: false,
            diag: Default::default(),
            in_safe_mode: false,
            rt_stats: None,
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::AsyncIo,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
            subscription_freshness: Vec::new(),
            use_sched_deadline: false,
            no_alloc: false,
        }
    }

    fn make_slow_io_node(name: &str, sleep_ms: u64, count: Arc<AtomicU64>) -> RegisteredNode {
        let node = SlowIoNode {
            name: name.to_string(),
            sleep_ms,
            count,
        };
        RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(node)),
            name: Arc::from(name),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new(name.to_string())),
            rate_hz: None,
            last_tick: None,
            is_rt_node: false,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            health_probe_counter: 0,
            is_paused: false,
            diag: Default::default(),
            in_safe_mode: false,
            rt_stats: None,
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::AsyncIo,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
            subscription_freshness: Vec::new(),
            use_sched_deadline: false,
            no_alloc: false,
        }
    }

    #[test]
    fn test_async_executor_runs_nodes() {
        let count = Arc::new(AtomicU64::new(0));
        let nodes = vec![make_async_node("async_1", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = AsyncExecutor::start(nodes, running.clone(), 5_u64.ms(), test_monitors());

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 1);
        assert!(count.load(Ordering::Relaxed) > 0);
    }

    /// Regression: a panic in `on_error()` must not take the async I/O thread
    /// with it.
    ///
    /// `process_node_result` runs in phase 2 of the drain loop, inside
    /// `block_on` on the `horus-async-io` thread — the only `catch_unwind` on
    /// this path is `NodeRunner::run_tick` inside `spawn_blocking`, and it
    /// covers the tick only. So a bare `on_error()` panic unwound straight out
    /// of `block_on`, killing the thread and with it every healthy node it
    /// owns, while `stop()`'s `join_with_timeout` saw `Err` from `join` and
    /// returned `None` → `unwrap_or_default()` → zero nodes reclaimed.
    #[test]
    fn on_error_panic_does_not_kill_the_async_thread() {
        use std::sync::atomic::Ordering::Relaxed;

        struct DoubleBoom {
            on_error_calls: Arc<AtomicU64>,
        }
        impl Node for DoubleBoom {
            fn name(&self) -> &str {
                "double_boom"
            }
            fn tick(&mut self) {
                panic!("tick boom");
            }
            fn on_error(&mut self, _msg: &str) {
                // Count BEFORE panicking: this is the test's evidence that the
                // executor actually entered the guarded callback, so the wait
                // below cannot pass by simply never getting there.
                self.on_error_calls.fetch_add(1, Relaxed);
                panic!("on_error boom");
            }
        }

        // Wait on an observed count, never on a fixed sleep. The deadline is an
        // upper bound on "this will never happen", not the thing being
        // measured, so a loaded machine makes this test slower rather than
        // flaky; the failure it reports is a real stall of the I/O thread.
        fn wait_for(what: &str, cond: impl Fn() -> bool) {
            let deadline = Instant::now() + 5_u64.secs();
            while Instant::now() < deadline {
                if cond() {
                    return;
                }
                std::thread::sleep(1_u64.ms());
            }
            panic!("timed out after 5s waiting for {what}");
        }

        let ticks = Arc::new(AtomicU64::new(0));
        let on_error_calls = Arc::new(AtomicU64::new(0));
        let mut boom = make_async_node("double_boom", ticks.clone());
        // Only the healthy node ever increments `ticks` — DoubleBoom replaces
        // the CounterNode that the handle was made for.
        boom.node = super::super::types::NodeKind::new(Box::new(DoubleBoom {
            on_error_calls: on_error_calls.clone(),
        }));
        let nodes = vec![boom, make_async_node("healthy", ticks.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = AsyncExecutor::start(nodes, running.clone(), 1_u64.ms(), test_monitors());

        // 1. The I/O thread has run the panicking on_error at least once.
        wait_for("the first on_error() panic", || {
            on_error_calls.load(Relaxed) > 0
        });
        // 2. The healthy node ticks AFTER that panic. Pre-fix the I/O thread
        //    has already unwound by this point and this never advances.
        let before = ticks.load(Relaxed);
        wait_for(
            "the healthy node to tick again after its neighbour panicked in on_error()",
            || ticks.load(Relaxed) > before,
        );

        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        // The deterministic half: a panicked thread's nodes cannot be
        // reclaimed, so pre-fix this is 0 of 2 regardless of timing.
        assert_eq!(
            returned.len(),
            2,
            "the async I/O thread died in on_error() — its nodes were never reclaimed"
        );
    }

    #[test]
    fn test_async_executor_concurrent_io() {
        // Two slow I/O nodes (each sleeps 30ms). If run sequentially, total would
        // be 60ms+ per cycle. Running concurrently via spawn_blocking, they complete
        // in ~30ms per cycle.
        let count1 = Arc::new(AtomicU64::new(0));
        let count2 = Arc::new(AtomicU64::new(0));
        let nodes = vec![
            make_slow_io_node("io_a", 30, count1.clone()),
            make_slow_io_node("io_b", 30, count2.clone()),
        ];
        let running = Arc::new(AtomicBool::new(true));

        let start = Instant::now();
        let executor = AsyncExecutor::start(nodes, running.clone(), 5_u64.ms(), test_monitors());

        std::thread::sleep(100_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();
        let elapsed = start.elapsed();

        assert_eq!(returned.len(), 2);
        let ticks1 = count1.load(Ordering::Relaxed);
        let ticks2 = count2.load(Ordering::Relaxed);

        // Both should have ticked
        assert!(ticks1 >= 1, "IO node A should tick, got {}", ticks1);
        assert!(ticks2 >= 1, "IO node B should tick, got {}", ticks2);

        // If truly concurrent, both should have similar tick counts
        // (sequential would give one ~2x the other)
        eprintln!(
            "Async IO concurrent test: A={}, B={} in {:?}",
            ticks1, ticks2, elapsed
        );
    }

    #[test]
    fn test_async_executor_rate_limiting() {
        let count = Arc::new(AtomicU64::new(0));
        let mut node = make_async_node("rate_limited_io", count.clone());
        node.rate_hz = Some(10.0); // 10 Hz

        let running = Arc::new(AtomicBool::new(true));
        let executor =
            AsyncExecutor::start(vec![node], running.clone(), 5_u64.ms(), test_monitors());

        std::thread::sleep(250_u64.ms());
        running.store(false, Ordering::SeqCst);
        let _returned = executor.stop();

        let ticks = count.load(Ordering::Relaxed);
        // At 10Hz for 250ms, expect ~2-3 ticks (not hundreds)
        assert!(
            (1..=5).contains(&ticks),
            "Expected 1-5 ticks at 10Hz in 250ms, got {}",
            ticks
        );
    }

    #[test]
    fn test_async_executor_stops_cleanly() {
        let count = Arc::new(AtomicU64::new(0));
        let nodes = vec![make_async_node("async_stop", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = AsyncExecutor::start(nodes, running.clone(), 5_u64.ms(), test_monitors());

        std::thread::sleep(20_u64.ms());
        running.store(false, Ordering::SeqCst);

        let returned = executor.stop();
        assert_eq!(returned.len(), 1);
        // Verifying stop() returns without hanging proves clean shutdown
    }
}
