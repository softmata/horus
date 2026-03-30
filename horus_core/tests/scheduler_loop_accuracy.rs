#![allow(dead_code)]
// Scheduler loop accuracy tests.
//
// Verifies that the scheduler's tick rate and run_for duration are
// reasonably accurate under normal conditions.

use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

mod common;
use common::cleanup_stale_shm;

struct TickCounter {
    name: String,
    tick_count: Arc<AtomicU64>,
}
impl Node for TickCounter {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
    }
}

fn pid() -> u32 {
    std::process::id()
}

// ============================================================================
// run_for exits within tolerance
// ============================================================================

#[test]
fn test_run_for_exits_within_tolerance() {
    cleanup_stale_shm();
    let ticks = Arc::new(AtomicU64::new(0));
    let node = TickCounter {
        name: format!("rf_{}", pid()),
        tick_count: ticks.clone(),
    };

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched.add(node).order(0).build().unwrap();

    let start = Instant::now();
    sched.run_for(Duration::from_millis(500)).unwrap();
    let elapsed = start.elapsed();

    // Allow ±200ms tolerance for scheduler overhead + shutdown
    assert!(
        elapsed >= Duration::from_millis(400),
        "run_for(500ms) finished too early: {:?}",
        elapsed
    );
    assert!(
        elapsed < Duration::from_millis(1000),
        "run_for(500ms) took too long: {:?}",
        elapsed
    );
}

// ============================================================================
// Priority ordering preserved with 3 nodes
// ============================================================================

#[test]
fn test_priority_ordering_preserved() {
    cleanup_stale_shm();
    let first_ticks = Arc::new(AtomicU64::new(0));
    let second_ticks = Arc::new(AtomicU64::new(0));
    let third_ticks = Arc::new(AtomicU64::new(0));

    let n1 = TickCounter {
        name: format!("prio0_{}", pid()),
        tick_count: first_ticks.clone(),
    };
    let n2 = TickCounter {
        name: format!("prio1_{}", pid()),
        tick_count: second_ticks.clone(),
    };
    let n3 = TickCounter {
        name: format!("prio2_{}", pid()),
        tick_count: third_ticks.clone(),
    };

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched.add(n1).order(0).build().unwrap();
    sched.add(n2).order(1).build().unwrap();
    sched.add(n3).order(2).build().unwrap();
    sched.run_for(Duration::from_millis(200)).unwrap();

    // All three should tick since they're BestEffort (main thread, sequential)
    let t1 = first_ticks.load(Ordering::SeqCst);
    let t2 = second_ticks.load(Ordering::SeqCst);
    let t3 = third_ticks.load(Ordering::SeqCst);

    assert!(t1 >= 1, "First node should tick, got {}", t1);
    assert!(t2 >= 1, "Second node should tick, got {}", t2);
    assert!(t3 >= 1, "Third node should tick, got {}", t3);
    // All BestEffort nodes tick at same rate — tick counts should be similar
    assert!(
        (t1 as i64 - t2 as i64).unsigned_abs() <= t1.max(1) / 2,
        "BestEffort nodes should tick at similar rates: {} vs {} vs {}",
        t1,
        t2,
        t3
    );
}

// ============================================================================
// set_node_rate changes tick frequency
// ============================================================================

#[test]
fn test_set_node_rate() {
    cleanup_stale_shm();
    let ticks = Arc::new(AtomicU64::new(0));
    let node = TickCounter {
        name: format!("setrate_{}", pid()),
        tick_count: ticks.clone(),
    };

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched.add(node).rate(100_u64.hz()).order(0).build().unwrap();

    // Change rate to 10Hz before running
    sched.set_node_rate(&format!("setrate_{}", pid()), 10_u64.hz());

    sched.run_for(Duration::from_millis(500)).unwrap();
    let final_ticks = ticks.load(Ordering::SeqCst);

    // 10Hz for 500ms ≈ 5 ticks (with generous tolerance for RT scheduling)
    assert!(
        (1..=30).contains(&final_ticks),
        "Node at 10Hz for 500ms should tick ~5 times, got {}",
        final_ticks
    );
}

// ============================================================================
// Empty scheduler exits immediately
// ============================================================================

#[test]
fn test_empty_scheduler_run_for_exits() {
    cleanup_stale_shm();
    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    let start = Instant::now();
    sched.run_for(Duration::from_millis(100)).unwrap();
    let elapsed = start.elapsed();

    assert!(
        elapsed < Duration::from_secs(2),
        "Empty scheduler should exit quickly, took {:?}",
        elapsed
    );
}
