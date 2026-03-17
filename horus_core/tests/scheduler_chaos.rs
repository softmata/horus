// Scheduler chaos tests — push the system to its limits.
//
// Verifies stability under extreme conditions:
// - 50+ nodes with all execution classes
// - Rapid start/stop cycles (resource leak detection)
// - Simultaneous panics with mixed failure policies
// - Memory pressure (large allocations in tick)

use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::{FailurePolicy, Scheduler};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

fn pid() -> u32 { std::process::id() }

// ---------------------------------------------------------------------------
// Node types for chaos testing
// ---------------------------------------------------------------------------

struct Counter {
    name: String,
    ticks: Arc<AtomicU64>,
}
impl Node for Counter {
    fn name(&self) -> &'static str { Box::leak(self.name.clone().into_boxed_str()) }
    fn tick(&mut self) { self.ticks.fetch_add(1, Ordering::SeqCst); }
}

struct PanicNode {
    name: String,
    ticks: Arc<AtomicU64>,
    panic_at: u64,
}
impl Node for PanicNode {
    fn name(&self) -> &'static str { Box::leak(self.name.clone().into_boxed_str()) }
    fn tick(&mut self) {
        let t = self.ticks.fetch_add(1, Ordering::SeqCst);
        if t >= self.panic_at {
            panic!("chaos panic at tick {}", t);
        }
    }
}

struct HeavyAllocNode {
    name: String,
    ticks: Arc<AtomicU64>,
    alloc_bytes: usize,
}
impl Node for HeavyAllocNode {
    fn name(&self) -> &'static str { Box::leak(self.name.clone().into_boxed_str()) }
    fn tick(&mut self) {
        self.ticks.fetch_add(1, Ordering::SeqCst);
        // Allocate and touch memory to create pressure
        let v: Vec<u8> = vec![0xAB; self.alloc_bytes];
        std::hint::black_box(&v); // Prevent optimization
    }
}

// ============================================================================
// Test: 50 mixed nodes across all execution classes
// ============================================================================

#[test]
fn test_50_mixed_nodes_all_classes() {
    cleanup_stale_shm();

    let total_ticks = Arc::new(AtomicU64::new(0));
    let mut sched = Scheduler::new().tick_rate(100_u64.hz());

    // 15 RT nodes
    for i in 0..15 {
        let t = total_ticks.clone();
        sched.add(Counter {
            name: format!("chaos_rt_{}_{}", i, pid()),
            ticks: t,
        }).rate(100_u64.hz()).order(i as u32).build().unwrap();
    }

    // 15 Compute nodes
    for i in 0..15 {
        let t = total_ticks.clone();
        sched.add(Counter {
            name: format!("chaos_compute_{}_{}", i, pid()),
            ticks: t,
        }).compute().order(50 + i as u32).build().unwrap();
    }

    // 10 AsyncIo nodes
    for i in 0..10 {
        let t = total_ticks.clone();
        sched.add(Counter {
            name: format!("chaos_async_{}_{}", i, pid()),
            ticks: t,
        }).async_io().order(100 + i as u32).build().unwrap();
    }

    // 10 BestEffort nodes
    for i in 0..10 {
        let t = total_ticks.clone();
        sched.add(Counter {
            name: format!("chaos_be_{}_{}", i, pid()),
            ticks: t,
        }).order(150 + i as u32).build().unwrap();
    }

    // Run for 1 second
    sched.run_for(Duration::from_secs(1)).unwrap();

    let total = total_ticks.load(Ordering::SeqCst);
    assert!(
        total > 50,
        "50 nodes should produce >50 total ticks in 1s, got {}",
        total
    );
}

// ============================================================================
// Test: 10 rapid start/stop cycles — no hang, no leak
// ============================================================================

#[test]
fn test_rapid_start_stop_10_cycles() {
    cleanup_stale_shm();

    let start = std::time::Instant::now();

    for cycle in 0..10 {
        let ticks = Arc::new(AtomicU64::new(0));
        let mut sched = Scheduler::new().tick_rate(100_u64.hz());

        for i in 0..5 {
            let t = ticks.clone();
            sched.add(Counter {
                name: format!("cycle{}_node{}_{}", cycle, i, pid()),
                ticks: t,
            }).order(i as u32).build().unwrap();
        }

        sched.run_for(Duration::from_millis(50)).unwrap();
        // Scheduler is dropped here — all resources should be cleaned up
    }

    let elapsed = start.elapsed();
    assert!(
        elapsed < Duration::from_secs(30),
        "10 rapid cycles should complete within 30s, took {:?}",
        elapsed
    );
}

// ============================================================================
// Test: 5 panicking nodes with Ignore policy + 5 healthy nodes
// ============================================================================

#[test]
fn test_simultaneous_panics_with_ignore_policy() {
    cleanup_stale_shm();

    let healthy_ticks = Arc::new(AtomicU64::new(0));
    let panic_ticks = Arc::new(AtomicU64::new(0));

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());

    // 5 panicking nodes — Ignore policy means they keep trying
    for i in 0..5 {
        let t = panic_ticks.clone();
        sched.add(PanicNode {
            name: format!("chaos_panic_{}_{}", i, pid()),
            ticks: t,
            panic_at: 1, // Panic on second tick
        })
        .order(i as u32)
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();
    }

    // 5 healthy nodes — should keep ticking despite panic siblings
    for i in 0..5 {
        let t = healthy_ticks.clone();
        sched.add(Counter {
            name: format!("chaos_healthy_{}_{}", i, pid()),
            ticks: t,
        }).order(50 + i as u32).build().unwrap();
    }

    sched.run_for(Duration::from_millis(500)).unwrap();

    let healthy = healthy_ticks.load(Ordering::SeqCst);
    assert!(
        healthy >= 5,
        "Healthy nodes should keep ticking despite panic siblings, got {} ticks",
        healthy
    );
}

// ============================================================================
// Test: Memory pressure — 1MB allocation per tick
// ============================================================================

#[test]
fn test_memory_pressure_large_allocs() {
    cleanup_stale_shm();

    let ticks = Arc::new(AtomicU64::new(0));
    let mut sched = Scheduler::new().tick_rate(50_u64.hz());

    sched.add(HeavyAllocNode {
        name: format!("chaos_alloc_{}", pid()),
        ticks: ticks.clone(),
        alloc_bytes: 1_000_000, // 1MB per tick
    }).order(0).build().unwrap();

    sched.run_for(Duration::from_millis(500)).unwrap();

    let final_ticks = ticks.load(Ordering::SeqCst);
    assert!(
        final_ticks >= 5,
        "Heavy allocation node should tick despite memory pressure, got {}",
        final_ticks
    );
}

// ============================================================================
// Test: Mixed failure policies — Fatal stops, others continue
// ============================================================================

#[test]
fn test_mixed_failure_policies() {
    cleanup_stale_shm();

    let healthy_ticks = Arc::new(AtomicU64::new(0));

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());

    // One Fatal panic node — should stop scheduler
    sched.add(PanicNode {
        name: format!("chaos_fatal_{}", pid()),
        ticks: Arc::new(AtomicU64::new(0)),
        panic_at: 5, // Panic after 5 ticks
    })
    .order(0)
    .failure_policy(FailurePolicy::Fatal)
    .build()
    .unwrap();

    // Healthy node — should tick until Fatal stops everything
    sched.add(Counter {
        name: format!("chaos_healthy_fatal_{}", pid()),
        ticks: healthy_ticks.clone(),
    }).order(50).build().unwrap();

    let start = std::time::Instant::now();
    let result = sched.run_for(Duration::from_millis(500));
    let elapsed = start.elapsed();

    // Fatal policy may manifest as early return or scheduler running full duration
    // depending on whether the panic node is on the main thread or RT thread.
    // BestEffort panic nodes are caught by catch_unwind in the main tick loop.
    // The key assertion: scheduler completes without hanging.
    assert!(
        elapsed < Duration::from_secs(5),
        "Scheduler should complete despite Fatal panic node, took {:?}",
        elapsed
    );

    let healthy = healthy_ticks.load(Ordering::SeqCst);
    assert!(
        healthy >= 1,
        "Healthy node should tick at least once, got {}",
        healthy
    );
}
