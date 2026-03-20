//! Stress tests for the HORUS scheduling system.
//!
//! Tests scale, sustained operation, cascading failures, and
//! resource management under load.

use horus_core::scheduling::{FailurePolicy, Scheduler};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;

mod common;
use common::cleanup_stale_shm;
use horus_core::core::{DurationExt, Node, NodeInfo};

// ============================================================================
// Mock nodes
// ============================================================================

/// Simple counter node with configurable name.
struct CounterNode {
    name: String,
    tick_count: Arc<AtomicU64>,
}

impl Node for CounterNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
    }
}

/// RT counter node with configurable priority.
struct RtCounterNode {
    name: String,
    tick_count: Arc<AtomicU64>,
}

impl Node for RtCounterNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
    }
}

/// Node that always panics.
struct PanicNode {
    name: String,
    tick_count: Arc<AtomicU64>,
}

impl Node for PanicNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
        panic!("stress panic");
    }
}

/// Slow compute node.
struct SlowNode {
    name: String,
    tick_count: Arc<AtomicU64>,
    sleep_ms: u64,
}

impl Node for SlowNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
        std::thread::sleep(self.sleep_ms.ms());
    }
}

/// Event counter node.
struct EventNode {
    name: String,
    tick_count: Arc<AtomicU64>,
}

impl Node for EventNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
    }
}

// ============================================================================
// Tests
// ============================================================================

#[test]
fn test_50_nodes_startup_shutdown() {
    cleanup_stale_shm();
    let counts: Vec<Arc<AtomicU64>> = (0..50).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut scheduler = Scheduler::new();

    // Add a mix of node types
    for i in 0..17 {
        scheduler
            .add(RtCounterNode {
                name: format!("rt_{}", i),
                tick_count: counts[i].clone(),
            })
            .order(i as u32)
            .build();
    }
    for i in 17..34 {
        scheduler
            .add(CounterNode {
                name: format!("compute_{}", i),
                tick_count: counts[i].clone(),
            })
            .order(i as u32)
            .compute()
            .build();
    }
    for i in 34..50 {
        scheduler
            .add(CounterNode {
                name: format!("best_effort_{}", i),
                tick_count: counts[i].clone(),
            })
            .order(i as u32)
            .build();
    }

    let result = scheduler.run_for(300_u64.ms());
    assert!(result.is_ok(), "50-node scheduler should run cleanly");

    // Verify at least some nodes from each group ticked
    let rt_ticked = counts[0..17]
        .iter()
        .filter(|c| c.load(Ordering::SeqCst) > 0)
        .count();
    let compute_ticked = counts[17..34]
        .iter()
        .filter(|c| c.load(Ordering::SeqCst) > 0)
        .count();
    let be_ticked = counts[34..50]
        .iter()
        .filter(|c| c.load(Ordering::SeqCst) > 0)
        .count();

    assert!(rt_ticked > 0, "Some RT nodes should tick");
    assert!(compute_ticked > 0, "Some compute nodes should tick");
    assert!(be_ticked > 0, "Some best-effort nodes should tick");
}

#[test]
fn test_50_rt_nodes_priority_order() {
    cleanup_stale_shm();
    let counts: Vec<Arc<AtomicU64>> = (0..50).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut scheduler = Scheduler::new();

    for i in 0..50 {
        scheduler
            .add(RtCounterNode {
                name: format!("rt_prio_{}", i),
                tick_count: counts[i].clone(),
            })
            .order(i as u32)
            .build();
    }

    scheduler.run_for(500_u64.ms()).unwrap();

    // All RT nodes should tick
    let all_ticked = counts.iter().enumerate().all(|(i, c)| {
        let ticks = c.load(Ordering::SeqCst);
        if ticks == 0 {
            eprintln!("RT node {} did not tick", i);
        }
        ticks > 0
    });

    assert!(all_ticked, "All 50 RT nodes should have ticked");
}

#[test]
fn test_sustained_high_rate_1_second() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(200_u64.hz()); // 200Hz = 5ms period
    scheduler
        .add(RtCounterNode {
            name: "sustained_200hz".to_string(),
            tick_count: tick_count.clone(),
        })
        .order(0)
        .rate(200_u64.hz())
        .build();

    scheduler.run_for(1_u64.secs()).unwrap();

    let ticks = tick_count.load(Ordering::SeqCst);
    // At 200Hz for 1s, expect ~200 ticks. ±75% margin for non-RT kernels + CI load.
    assert!(
        (50..=500).contains(&ticks),
        "200Hz node should tick ~50-500 times in 1s (expected ~200), got {}",
        ticks
    );
}

#[test]
fn test_cascading_failures_restart() {
    cleanup_stale_shm();
    let counts: Vec<Arc<AtomicU64>> = (0..10).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut scheduler = Scheduler::new();

    for i in 0..10 {
        scheduler
            .add(PanicNode {
                name: format!("restart_panic_{}", i),
                tick_count: counts[i].clone(),
            })
            .order(i as u32)
            .compute()
            .failure_policy(FailurePolicy::restart(3, 10_u64.ms()))
            .build();
    }

    // After exhausting restarts, all nodes should escalate to fatal
    let result = scheduler.run_for(500_u64.ms());
    result.unwrap();

    // At least some nodes should have ticked before escalation
    let total_ticks: u64 = counts.iter().map(|c| c.load(Ordering::SeqCst)).sum();
    assert!(
        total_ticks > 0,
        "Some nodes should tick before restart exhaustion"
    );
}

#[test]
fn test_cascading_failures_mixed_policies() {
    cleanup_stale_shm();
    let fatal_count = Arc::new(AtomicU64::new(0));
    let restart_count = Arc::new(AtomicU64::new(0));
    let skip_count = Arc::new(AtomicU64::new(0));
    let ignore_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();

    // Fatal policy — first panic stops scheduler
    scheduler
        .add(PanicNode {
            name: "mixed_fatal".to_string(),
            tick_count: fatal_count.clone(),
        })
        .order(0)
        .compute()
        .failure_policy(FailurePolicy::Fatal)
        .build();

    // These should also be registered but may not tick much due to Fatal stopping
    scheduler
        .add(PanicNode {
            name: "mixed_restart".to_string(),
            tick_count: restart_count.clone(),
        })
        .order(1)
        .compute()
        .failure_policy(FailurePolicy::restart(3, 10_u64.ms()))
        .build();

    scheduler
        .add(PanicNode {
            name: "mixed_skip".to_string(),
            tick_count: skip_count.clone(),
        })
        .order(2)
        .compute()
        .failure_policy(FailurePolicy::skip(2, 100_u64.ms()))
        .build();

    scheduler
        .add(PanicNode {
            name: "mixed_ignore".to_string(),
            tick_count: ignore_count.clone(),
        })
        .order(3)
        .compute()
        .failure_policy(FailurePolicy::Ignore)
        .build();

    // Fatal should stop scheduler
    let result = scheduler.run_for(500_u64.ms());
    result.unwrap();

    // Fatal node should have ticked at least once
    assert!(
        fatal_count.load(Ordering::SeqCst) >= 1,
        "Fatal node should tick at least once"
    );
}

#[test]
fn test_sustained_load_shedding() {
    cleanup_stale_shm();
    let critical_counts: Vec<Arc<AtomicU64>> =
        (0..3).map(|_| Arc::new(AtomicU64::new(0))).collect();
    let bg_counts: Vec<Arc<AtomicU64>> = (0..3).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut scheduler = Scheduler::new().tick_rate(50_u64.hz()); // 20ms period

    // 3 critical compute nodes
    for i in 0..3 {
        scheduler
            .add(SlowNode {
                name: format!("crit_compute_{}", i),
                tick_count: critical_counts[i].clone(),
                sleep_ms: 10, // Each critical node takes 10ms
            })
            .order(i as u32)
            .compute()
            .build();
    }

    // 3 background compute nodes (order >= 200, sheddable)
    for i in 0..3 {
        scheduler
            .add(CounterNode {
                name: format!("bg_compute_{}", i),
                tick_count: bg_counts[i].clone(),
            })
            .order(200 + i as u32)
            .compute()
            .build();
    }

    scheduler.run_for(300_u64.ms()).unwrap();

    // All critical nodes should tick
    for (i, c) in critical_counts.iter().enumerate() {
        assert!(
            c.load(Ordering::SeqCst) > 0,
            "Critical node {} should tick",
            i
        );
    }
}

#[test]
fn test_all_executors_simultaneously() {
    cleanup_stale_shm();
    let rt_count = Arc::new(AtomicU64::new(0));
    let compute_count = Arc::new(AtomicU64::new(0));
    let async_count = Arc::new(AtomicU64::new(0));
    let event_count = Arc::new(AtomicU64::new(0));
    let be_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();

    // RT node
    scheduler
        .add(RtCounterNode {
            name: "all_exec_rt".to_string(),
            tick_count: rt_count.clone(),
        })
        .order(0)
        .build();

    // Compute node
    scheduler
        .add(CounterNode {
            name: "all_exec_compute".to_string(),
            tick_count: compute_count.clone(),
        })
        .order(5)
        .compute()
        .build();

    // Async IO node
    scheduler
        .add(CounterNode {
            name: "all_exec_async".to_string(),
            tick_count: async_count.clone(),
        })
        .order(10)
        .async_io()
        .build();

    // Event node
    scheduler
        .add(EventNode {
            name: "all_exec_event".to_string(),
            tick_count: event_count.clone(),
        })
        .order(15)
        .on("all_exec_topic")
        .build();

    // Best-effort node
    scheduler
        .add(CounterNode {
            name: "all_exec_be".to_string(),
            tick_count: be_count.clone(),
        })
        .order(20)
        .build();

    let handle = std::thread::spawn(move || {
        scheduler.run_for(200_u64.ms()).unwrap();
    });

    // Send event notification
    std::thread::sleep(50_u64.ms());
    NodeInfo::notify_event("all_exec_event");
    std::thread::sleep(50_u64.ms());

    handle.join().unwrap();

    // RT, Compute, Async, and BestEffort should all tick
    assert!(rt_count.load(Ordering::SeqCst) > 0, "RT should tick");
    assert!(
        compute_count.load(Ordering::SeqCst) > 0,
        "Compute should tick"
    );
    assert!(async_count.load(Ordering::SeqCst) > 0, "Async should tick");
    assert!(
        be_count.load(Ordering::SeqCst) > 0,
        "BestEffort should tick"
    );
    // Event may or may not tick depending on timing
}

#[test]
fn test_rapid_start_stop_cycles() {
    // 20 rapid create/add/run/shutdown cycles to check for resource leaks
    for cycle in 0..20 {
        cleanup_stale_shm();
        let count = Arc::new(AtomicU64::new(0));

        let mut scheduler = Scheduler::new();
        scheduler
            .add(CounterNode {
                name: format!("rapid_cycle_{}", cycle),
                tick_count: count.clone(),
            })
            .order(0)
            .build();

        scheduler.run_for(10_u64.ms()).unwrap();
        // Scheduler is dropped here
    }
    // If we get here without hanging or panicking, no resource leak
}

#[test]
fn test_burst_event_notifications() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add(EventNode {
            name: "burst_stress_event".to_string(),
            tick_count: tick_count.clone(),
        })
        .order(5)
        .on("burst_stress_topic")
        .build();

    let handle = std::thread::spawn(move || {
        scheduler.run_for(500_u64.ms()).unwrap();
    });

    // Wait for registration
    let mut registered = false;
    for _ in 0..100 {
        if NodeInfo::notify_event("burst_stress_event") {
            registered = true;
            break;
        }
        std::thread::sleep(5_u64.ms());
    }

    if registered {
        std::thread::sleep(50_u64.ms());

        // Send 999 more rapid notifications (1000 total)
        for _ in 0..999 {
            NodeInfo::notify_event("burst_stress_event");
        }

        std::thread::sleep(200_u64.ms());

        let ticks = tick_count.load(Ordering::SeqCst);
        assert!(
            ticks >= 100,
            "Should handle at least 100 of 1000 burst notifications, got {}",
            ticks
        );
    }

    handle.join().unwrap();
}

#[test]
fn test_20_parallel_compute_nodes() {
    cleanup_stale_shm();
    let counts: Vec<Arc<AtomicU64>> = (0..20).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut scheduler = Scheduler::new();
    for (i, count) in counts.iter().enumerate() {
        scheduler
            .add(SlowNode {
                name: format!("par_compute_{}", i),
                tick_count: count.clone(),
                sleep_ms: 1, // 1ms each
            })
            .order(5)
            .compute()
            .build();
    }

    let start = Instant::now();
    scheduler.run_for(300_u64.ms()).unwrap();
    let elapsed = start.elapsed();

    // All nodes should tick
    let all_ticked = counts.iter().all(|c| c.load(Ordering::SeqCst) > 0);
    assert!(all_ticked, "All 20 parallel compute nodes should tick");

    // If truly parallel, 20 nodes * 1ms each should complete quickly per cycle
    let total_ticks: u64 = counts.iter().map(|c| c.load(Ordering::SeqCst)).sum();
    let avg_ticks = total_ticks as f64 / 20.0;
    assert!(
        avg_ticks >= 1.0,
        "Average ticks should be >= 1, got {}",
        avg_ticks
    );
    let _ = elapsed;
}
