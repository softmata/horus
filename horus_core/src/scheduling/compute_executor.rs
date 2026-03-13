//! Compute thread pool executor.
//!
//! Runs non-RT (compute/best-effort) nodes in a parallel thread pool.
//! The compute pool runs independently of the RT thread — slow compute
//! nodes never block RT execution.
//!
//! # Architecture
//!
//! ```text
//!  Compute Thread (owns all compute nodes)
//!  ┌────────────────────────────────────────┐
//!  │  loop:                                 │
//!  │    ┌─ crossbeam::scope ─────────────┐  │
//!  │    │  thread: node_A.tick()         │  │
//!  │    │  thread: node_B.tick()         │  │
//!  │    │  thread: node_C.tick()         │  │
//!  │    └────────────────────────────────┘  │
//!  │    process results (profiling, etc.)   │
//!  │    sleep until next tick               │
//!  └────────────────────────────────────────┘
//! ```
//!
//! Each compute node is ticked on its own crossbeam thread per tick cycle.
//! Per-node rate limiting is respected — nodes that aren't due yet are skipped.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::terminal::print_line;

use super::primitives::NodeRunner;
use super::types::{RegisteredNode, SharedMonitors};

/// Minimum node order value that qualifies for load shedding.
/// Nodes with `priority >= SHED_THRESHOLD` can be skipped under overload.
/// Maps to the "Background" tier in the order guidelines (200+).
const SHED_THRESHOLD: u32 = 200;

/// Number of consecutive under-budget cycles before shedding deactivates.
/// Prevents rapid on/off thrashing when overloaded nodes are right at the edge.
const SHED_COOLDOWN_CYCLES: u32 = 3;

/// Parallel compute executor for non-RT nodes.
///
/// Owns compute nodes and ticks them in parallel on a dedicated thread.
/// Shutdown is coordinated via the shared `running` flag.
pub(crate) struct ComputeExecutor {
    handle: Option<std::thread::JoinHandle<Vec<RegisteredNode>>>,
}

impl ComputeExecutor {
    /// Start the compute executor with the given nodes.
    ///
    /// Spawns a thread that runs a tick loop, parallelizing node execution
    /// via crossbeam::scope each cycle. `tick_period` controls the cadence
    /// of the compute loop (derived from the scheduler's tick rate).
    pub fn start(
        mut nodes: Vec<RegisteredNode>,
        running: Arc<AtomicBool>,
        tick_period: Duration,
        monitors: SharedMonitors,
    ) -> Self {
        nodes.sort_by_key(|n| n.priority);

        let handle = std::thread::Builder::new()
            .name("horus-compute".to_string())
            .spawn(move || Self::compute_thread_main(nodes, running, tick_period, monitors))
            .expect("Failed to spawn compute thread");

        Self {
            handle: Some(handle),
        }
    }

    /// Stop the compute executor and reclaim its nodes.
    pub fn stop(mut self) -> Vec<RegisteredNode> {
        self.handle
            .take()
            .expect("Compute thread handle already consumed")
            .join()
            .expect("Compute thread panicked")
    }

    /// Main function for the compute thread.
    fn compute_thread_main(
        mut nodes: Vec<RegisteredNode>,
        running: Arc<AtomicBool>,
        tick_period: Duration,
        monitors: SharedMonitors,
    ) -> Vec<RegisteredNode> {
        print_line(&format!(
            "[Compute] Started with {} nodes, tick period {:?}",
            nodes.len(),
            tick_period
        ));

        // Load shedding state: tracks whether we're currently shedding
        let mut shedding_active = false;
        let mut cooldown_remaining: u32 = 0;

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

                // Load shedding: skip sheddable nodes when under overload
                if shedding_active && node.priority >= SHED_THRESHOLD {
                    continue;
                }

                ready_indices.push(i);
            }

            if ready_indices.is_empty() {
                // Nothing to do — sleep and check again
                let elapsed = loop_start.elapsed();
                if elapsed < tick_period {
                    std::thread::sleep(tick_period - elapsed);
                }
                continue;
            }

            // Update last_tick for rate-limited nodes
            let now = Instant::now();
            for &i in &ready_indices {
                if nodes[i].rate_hz.is_some() {
                    nodes[i].last_tick = Some(now);
                }
                // Start context tick
                if let Some(ref mut ctx) = nodes[i].context {
                    ctx.start_tick();
                }
            }

            // Begin recording for all ready nodes
            for &i in &ready_indices {
                if let Some(ref mut recorder) = nodes[i].recorder {
                    recorder.begin_tick(0);
                }
            }

            // Single node: tick directly (no crossbeam overhead)
            if ready_indices.len() == 1 {
                let i = ready_indices[0];
                let tr = NodeRunner::run_tick(&mut nodes[i].node);
                Self::process_node_result(&mut nodes[i], tr, &running, &monitors);
            } else {
                // Parallel tick via crossbeam::scope
                struct ParallelResult {
                    index: usize,
                    tick_start: Instant,
                    duration: Duration,
                    result: std::thread::Result<()>,
                }

                let nodes_ptr = nodes.as_mut_ptr();
                let results: Vec<ParallelResult> = crossbeam::scope(|s| {
                    let handles: Vec<_> = ready_indices
                        .iter()
                        .map(|&i| {
                            // SAFETY: Each thread accesses a distinct index.
                            // crossbeam::scope guarantees threads don't outlive the borrow.
                            let node_ref = unsafe { &mut *nodes_ptr.add(i) };
                            s.spawn(move |_| {
                                let tr = NodeRunner::run_tick(&mut node_ref.node);
                                ParallelResult {
                                    index: i,
                                    tick_start: tr.tick_start,
                                    duration: tr.duration,
                                    result: tr.result,
                                }
                            })
                        })
                        .collect();

                    handles
                        .into_iter()
                        .map(|h| h.join().expect("crossbeam thread panicked"))
                        .collect()
                })
                .expect("crossbeam scope panicked");

                // Process results sequentially
                for pr in results {
                    let tr = super::primitives::TickResult {
                        tick_start: pr.tick_start,
                        duration: pr.duration,
                        result: pr.result,
                    };
                    Self::process_node_result(&mut nodes[pr.index], tr, &running, &monitors);
                }
            }

            // Update load shedding state based on this cycle's duration.
            // Uses hysteresis: shedding activates immediately on overload but
            // requires SHED_COOLDOWN_CYCLES consecutive under-budget cycles to deactivate.
            let elapsed = loop_start.elapsed();
            if elapsed > tick_period {
                if !shedding_active {
                    let shed_count = nodes
                        .iter()
                        .filter(|n| n.priority >= SHED_THRESHOLD)
                        .count();
                    if shed_count > 0 {
                        print_line(&format!(
                            "[Compute] Overload detected (cycle took {:?} > {:?}), shedding {} background nodes (order >= {})",
                            elapsed, tick_period, shed_count, SHED_THRESHOLD
                        ));
                    }
                    shedding_active = true;
                }
                // Reset cooldown on every overload cycle
                cooldown_remaining = SHED_COOLDOWN_CYCLES;
            } else if shedding_active {
                // Under budget — count down cooldown before resuming
                cooldown_remaining = cooldown_remaining.saturating_sub(1);
                if cooldown_remaining == 0 {
                    print_line("[Compute] Load normalized, resuming all nodes");
                    shedding_active = false;
                }
            }

            // Sleep until next tick period
            if elapsed < tick_period {
                std::thread::sleep(tick_period - elapsed);
            }
        }

        print_line(&format!(
            "[Compute] Stopped ({} nodes returning to scheduler)",
            nodes.len()
        ));

        nodes
    }

    /// Process the result of a single node tick.
    fn process_node_result(
        node: &mut RegisteredNode,
        tr: super::primitives::TickResult,
        _running: &Arc<AtomicBool>,
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

        match tr.result {
            Ok(_) => {
                if let Some(ref mut ctx) = node.context {
                    ctx.record_tick();
                }
            }
            Err(panic_err) => {
                if let Ok(mut profiler) = monitors.profiler.try_lock() {
                    profiler.record_node_failure(&node.name);
                }
                let error_msg = if let Some(s) = panic_err.downcast_ref::<&str>() {
                    format!("[Compute] Node '{}' panicked: {}", node.name, s)
                } else if let Some(s) = panic_err.downcast_ref::<String>() {
                    format!("[Compute] Node '{}' panicked: {}", node.name, s)
                } else {
                    format!("[Compute] Node '{}' panicked (unknown)", node.name)
                };
                print_line(&error_msg);
                node.node.on_error(&error_msg);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::duration_ext::DurationExt;
    use crate::core::{Miss, Node, NodeInfo};
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
        count: Arc<std::sync::atomic::AtomicU64>,
    }

    impl Node for CounterNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.count
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
    }

    fn make_compute_node(name: &str, count: Arc<std::sync::atomic::AtomicU64>) -> RegisteredNode {
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
            is_paused: false,
            rt_stats: None,
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::Compute,
            health_state: super::super::types::AtomicHealthState::default(),
        }
    }

    #[test]
    fn test_compute_executor_runs_nodes() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let nodes = vec![make_compute_node("compute_1", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = ComputeExecutor::start(
            nodes,
            running.clone(),
            1_u64.ms(),
            test_monitors(),
        );

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 1);
        assert!(count.load(std::sync::atomic::Ordering::Relaxed) > 0);
    }

    #[test]
    fn test_compute_executor_parallel_nodes() {
        let count1 = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count2 = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count3 = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let nodes = vec![
            make_compute_node("compute_a", count1.clone()),
            make_compute_node("compute_b", count2.clone()),
            make_compute_node("compute_c", count3.clone()),
        ];
        let running = Arc::new(AtomicBool::new(true));

        let executor = ComputeExecutor::start(
            nodes,
            running.clone(),
            1_u64.ms(),
            test_monitors(),
        );

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 3);
        assert!(count1.load(std::sync::atomic::Ordering::Relaxed) > 0);
        assert!(count2.load(std::sync::atomic::Ordering::Relaxed) > 0);
        assert!(count3.load(std::sync::atomic::Ordering::Relaxed) > 0);
    }

    #[test]
    fn test_compute_executor_rate_limiting() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let mut node = make_compute_node("rate_limited", count.clone());
        node.rate_hz = Some(10.0); // 10 Hz = ~100ms between ticks

        let running = Arc::new(AtomicBool::new(true));
        let executor = ComputeExecutor::start(
            vec![node],
            running.clone(),
            1_u64.ms(), // Pool ticks at 1kHz but node only at 10Hz
            test_monitors(),
        );

        std::thread::sleep(250_u64.ms());
        running.store(false, Ordering::SeqCst);
        let _returned = executor.stop();

        let ticks = count.load(std::sync::atomic::Ordering::Relaxed);
        // At 10Hz for 250ms, expect ~2-3 ticks (not hundreds)
        assert!(
            (1..=5).contains(&ticks),
            "Expected 1-5 ticks at 10Hz in 250ms, got {}",
            ticks
        );
    }
}
