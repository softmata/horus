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

use super::fault_tolerance::FailureAction;
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
        self.handle
            .take()
            .expect("Async I/O thread handle already consumed")
            .join()
            .expect("Async I/O thread panicked")
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
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_time()
            .build()
            .expect("Failed to create async I/O tokio runtime");

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
                for (i, node) in nodes.iter().enumerate() {
                    if !node.initialized || node.is_stopped || node.is_paused {
                        continue;
                    }

                    // Per-node rate limiting
                    if let Some(rate_hz) = node.rate_hz {
                        if let Some(last_tick) = node.last_tick {
                            let elapsed = loop_start.duration_since(last_tick).as_secs_f64();
                            if elapsed < 1.0 / rate_hz {
                                continue;
                            }
                        }
                    }

                    // Circuit breaker
                    if !node.failure_handler.should_allow() {
                        continue;
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
                    // SAFETY: Each task accesses a distinct index into the nodes vec.
                    // We await all handles before nodes can be mutated again.
                    let node_ref = unsafe { &mut *nodes_ptr.add(i) };
                    let handle = tokio::task::spawn_blocking(move || {
                        let tr = NodeRunner::run_tick(&mut node_ref.node);
                        AsyncResult {
                            index: i,
                            tick_start: tr.tick_start,
                            duration: tr.duration,
                            result: tr.result,
                        }
                    });
                    handles.push(handle);
                }

                // Await all results
                for handle in handles {
                    match handle.await {
                        Ok(ar) => {
                            let tr = super::primitives::TickResult {
                                tick_start: ar.tick_start,
                                duration: ar.duration,
                                result: ar.result,
                            };
                            Self::process_node_result(
                                &mut nodes[ar.index],
                                tr,
                                &running,
                                &monitors,
                            );
                        }
                        Err(e) => {
                            print_line(&format!("[AsyncIO] spawn_blocking panicked: {}", e));
                        }
                    }
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
        monitors
            .profiler
            .lock()
            .unwrap()
            .record(&node.name, tr.duration);

        // End recording tick
        if let Some(ref mut recorder) = node.recorder {
            recorder.end_tick(tr.duration.as_nanos() as u64);
        }

        match tr.result {
            Ok(_) => {
                node.failure_handler.record_success();
                if let Some(ref mut ctx) = node.context {
                    ctx.record_tick();
                }
            }
            Err(panic_err) => {
                let action = node.failure_handler.record_failure();
                monitors
                    .profiler
                    .lock()
                    .unwrap()
                    .record_node_failure(&node.name);
                let error_msg = if let Some(s) = panic_err.downcast_ref::<&str>() {
                    format!("[AsyncIO] Node '{}' panicked: {}", node.name, s)
                } else if let Some(s) = panic_err.downcast_ref::<String>() {
                    format!("[AsyncIO] Node '{}' panicked: {}", node.name, s)
                } else {
                    format!("[AsyncIO] Node '{}' panicked (unknown)", node.name)
                };
                print_line(&error_msg);
                node.node.on_error(&error_msg);

                match action {
                    FailureAction::StopScheduler | FailureAction::FatalAfterRestarts => {
                        print_line(&format!(
                            "[AsyncIO] Fatal failure in '{}' — stopping scheduler",
                            node.name
                        ));
                        running.store(false, Ordering::SeqCst);
                    }
                    FailureAction::RestartNode => {
                        if let Some(ref mut ctx) = node.context {
                            match ctx.restart() {
                                Ok(_) => {
                                    node.initialized = true;
                                }
                                Err(e) => {
                                    print_line(&format!(
                                        "[AsyncIO] Restart failed for '{}': {}",
                                        node.name, e
                                    ));
                                    node.initialized = false;
                                }
                            }
                        }
                    }
                    FailureAction::SkipNode | FailureAction::Continue => {}
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{Node, NodeInfo};
    use std::sync::atomic::AtomicU64;
    use std::sync::Mutex;

    fn test_monitors() -> SharedMonitors {
        SharedMonitors {
            profiler: Arc::new(Mutex::new(super::super::profiler::RuntimeProfiler::new())),
            blackbox: None,
            verbose: true,
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
            std::thread::sleep(Duration::from_millis(self.sleep_ms));
        }
    }

    fn make_async_node(name: &str, count: Arc<AtomicU64>) -> RegisteredNode {
        use super::super::fault_tolerance::FailureHandler;

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
            failure_handler: FailureHandler::new(
                super::super::fault_tolerance::FailurePolicy::Fatal,
            ),
            is_rt_node: false,
            wcet_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: None,
            execution_class: super::super::types::ExecutionClass::AsyncIo,
            has_custom_failure_policy: false,
        }
    }

    fn make_slow_io_node(name: &str, sleep_ms: u64, count: Arc<AtomicU64>) -> RegisteredNode {
        use super::super::fault_tolerance::FailureHandler;

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
            failure_handler: FailureHandler::new(
                super::super::fault_tolerance::FailurePolicy::Fatal,
            ),
            is_rt_node: false,
            wcet_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
            rt_stats: None,
            execution_class: super::super::types::ExecutionClass::AsyncIo,
            has_custom_failure_policy: false,
        }
    }

    #[test]
    fn test_async_executor_runs_nodes() {
        let count = Arc::new(AtomicU64::new(0));
        let nodes = vec![make_async_node("async_1", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = AsyncExecutor::start(
            nodes,
            running.clone(),
            Duration::from_millis(5),
            test_monitors(),
        );

        std::thread::sleep(Duration::from_millis(50));
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 1);
        assert!(count.load(Ordering::Relaxed) > 0);
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
        let executor = AsyncExecutor::start(
            nodes,
            running.clone(),
            Duration::from_millis(5),
            test_monitors(),
        );

        std::thread::sleep(Duration::from_millis(100));
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
        let executor = AsyncExecutor::start(
            vec![node],
            running.clone(),
            Duration::from_millis(5),
            test_monitors(),
        );

        std::thread::sleep(Duration::from_millis(250));
        running.store(false, Ordering::SeqCst);
        let _returned = executor.stop();

        let ticks = count.load(Ordering::Relaxed);
        // At 10Hz for 250ms, expect ~2-3 ticks (not hundreds)
        assert!(
            ticks >= 1 && ticks <= 5,
            "Expected 1-5 ticks at 10Hz in 250ms, got {}",
            ticks
        );
    }

    #[test]
    fn test_async_executor_stops_cleanly() {
        let count = Arc::new(AtomicU64::new(0));
        let nodes = vec![make_async_node("async_stop", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = AsyncExecutor::start(
            nodes,
            running.clone(),
            Duration::from_millis(5),
            test_monitors(),
        );

        std::thread::sleep(Duration::from_millis(20));
        running.store(false, Ordering::SeqCst);

        let returned = executor.stop();
        assert_eq!(returned.len(), 1);
        // Verifying stop() returns without hanging proves clean shutdown
    }
}
