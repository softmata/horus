// Integration tests for multi-executor coexistence.
//
// Verifies that RT, Compute, Event, and AsyncIo execution classes can all
// run simultaneously in a single Scheduler without interference.

use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

// ---------------------------------------------------------------------------
// Counter nodes for each execution class
// ---------------------------------------------------------------------------

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

struct ComputeCounterNode {
    name: String,
    tick_count: Arc<AtomicU64>,
    work_ms: u64,
}
impl Node for ComputeCounterNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
        if self.work_ms > 0 {
            std::thread::sleep(Duration::from_millis(self.work_ms));
        }
    }
}

struct AsyncCounterNode {
    name: String,
    tick_count: Arc<AtomicU64>,
    work_ms: u64,
}
impl Node for AsyncCounterNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
        if self.work_ms > 0 {
            std::thread::sleep(Duration::from_millis(self.work_ms));
        }
    }
}

struct BestEffortCounterNode {
    name: String,
    tick_count: Arc<AtomicU64>,
}
impl Node for BestEffortCounterNode {
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
// Test: RT + Compute coexist — RT node is not starved by heavy compute
// ============================================================================

#[test]
fn test_rt_not_starved_by_compute() {
    cleanup_stale_shm();

    let rt_ticks = Arc::new(AtomicU64::new(0));
    let compute_ticks = Arc::new(AtomicU64::new(0));

    let rt_node = RtCounterNode {
        name: format!("mex_rt_{}", pid()),
        tick_count: rt_ticks.clone(),
    };
    let compute_node = ComputeCounterNode {
        name: format!("mex_compute_{}", pid()),
        tick_count: compute_ticks.clone(),
        work_ms: 30, // heavy compute — 30ms per tick
    };

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched
        .add(rt_node)
        .rate(100_u64.hz())
        .order(0)
        .build()
        .unwrap();
    sched
        .add(compute_node)
        .compute()
        .rate(10_u64.hz())
        .order(1)
        .build()
        .unwrap();
    sched.run_for(Duration::from_millis(500)).unwrap();

    let rt_final = rt_ticks.load(Ordering::SeqCst);
    let compute_final = compute_ticks.load(Ordering::SeqCst);

    // RT node should tick many more times than compute node
    // RT at 100Hz for 500ms ≈ 50 ticks (with overhead)
    // Compute at 10Hz with 30ms work ≈ 5 ticks
    assert!(
        rt_final >= 5,
        "RT node should tick multiple times, got {}",
        rt_final
    );
    assert!(
        compute_final >= 1,
        "Compute node should tick at least once, got {}",
        compute_final
    );
    // Key assertion: RT should not be starved (should have more ticks than compute)
    assert!(
        rt_final > compute_final,
        "RT ({}) should tick more than heavy compute ({})",
        rt_final,
        compute_final
    );
}

// ============================================================================
// Test: RT + AsyncIo coexist — async I/O doesn't block RT
// ============================================================================

#[test]
fn test_async_does_not_block_rt() {
    cleanup_stale_shm();

    let rt_ticks = Arc::new(AtomicU64::new(0));
    let async_ticks = Arc::new(AtomicU64::new(0));

    let rt_node = RtCounterNode {
        name: format!("mex_rt_async_{}", pid()),
        tick_count: rt_ticks.clone(),
    };
    let async_node = AsyncCounterNode {
        name: format!("mex_async_{}", pid()),
        tick_count: async_ticks.clone(),
        work_ms: 50, // slow I/O — 50ms per tick
    };

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched
        .add(rt_node)
        .rate(100_u64.hz())
        .order(0)
        .build()
        .unwrap();
    sched
        .add(async_node)
        .async_io()
        .rate(10_u64.hz())
        .order(1)
        .build()
        .unwrap();
    sched.run_for(Duration::from_millis(500)).unwrap();

    let rt_final = rt_ticks.load(Ordering::SeqCst);
    let async_final = async_ticks.load(Ordering::SeqCst);

    assert!(
        rt_final >= 5,
        "RT node should tick independently of async, got {}",
        rt_final
    );
    assert!(
        async_final >= 1,
        "Async node should tick at least once, got {}",
        async_final
    );
}

// ============================================================================
// Test: Multiple execution classes in one scheduler
// ============================================================================

#[test]
fn test_three_execution_classes_coexist() {
    cleanup_stale_shm();

    let rt_ticks = Arc::new(AtomicU64::new(0));
    let compute_ticks = Arc::new(AtomicU64::new(0));
    let best_effort_ticks = Arc::new(AtomicU64::new(0));

    let rt_node = RtCounterNode {
        name: format!("mex3_rt_{}", pid()),
        tick_count: rt_ticks.clone(),
    };
    let compute_node = ComputeCounterNode {
        name: format!("mex3_compute_{}", pid()),
        tick_count: compute_ticks.clone(),
        work_ms: 0,
    };
    let be_node = BestEffortCounterNode {
        name: format!("mex3_be_{}", pid()),
        tick_count: best_effort_ticks.clone(),
    };

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched
        .add(rt_node)
        .rate(100_u64.hz())
        .order(0)
        .build()
        .unwrap();
    sched.add(compute_node).compute().order(1).build().unwrap();
    sched.add(be_node).order(2).build().unwrap();
    sched.run_for(Duration::from_millis(300)).unwrap();

    let rt_final = rt_ticks.load(Ordering::SeqCst);
    let compute_final = compute_ticks.load(Ordering::SeqCst);
    let be_final = best_effort_ticks.load(Ordering::SeqCst);

    // All three should tick
    assert!(rt_final >= 1, "RT should tick, got {}", rt_final);
    assert!(
        compute_final >= 1,
        "Compute should tick, got {}",
        compute_final
    );
    assert!(be_final >= 1, "BestEffort should tick, got {}", be_final);
}

// ============================================================================
// Test: Compute node rate limiting works
// ============================================================================

#[test]
fn test_compute_rate_limiting() {
    cleanup_stale_shm();

    let ticks = Arc::new(AtomicU64::new(0));
    let node = ComputeCounterNode {
        name: format!("mex_comprate_{}", pid()),
        tick_count: ticks.clone(),
        work_ms: 0,
    };

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    // Rate-limited compute node at 10Hz
    sched
        .add(node)
        .compute()
        .rate(10_u64.hz())
        .order(0)
        .build()
        .unwrap();
    sched.run_for(Duration::from_millis(500)).unwrap();

    let final_ticks = ticks.load(Ordering::SeqCst);
    // 10Hz for 500ms ≈ 5 ticks (with tolerance)
    assert!(
        final_ticks >= 2 && final_ticks <= 20,
        "Compute at 10Hz for 500ms should tick ~5 times, got {}",
        final_ticks
    );
}

// ============================================================================
// Test: Async node rate limiting works
// ============================================================================

#[test]
fn test_async_rate_limiting() {
    cleanup_stale_shm();

    let ticks = Arc::new(AtomicU64::new(0));
    let node = AsyncCounterNode {
        name: format!("mex_asyncrate_{}", pid()),
        tick_count: ticks.clone(),
        work_ms: 0,
    };

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched
        .add(node)
        .async_io()
        .rate(10_u64.hz())
        .order(0)
        .build()
        .unwrap();
    sched.run_for(Duration::from_millis(500)).unwrap();

    let final_ticks = ticks.load(Ordering::SeqCst);
    assert!(
        final_ticks >= 2 && final_ticks <= 20,
        "Async at 10Hz for 500ms should tick ~5 times, got {}",
        final_ticks
    );
}

// ============================================================================
// Test: BestEffort node ticks at scheduler tick rate
// ============================================================================

#[test]
fn test_best_effort_ticks_at_scheduler_rate() {
    cleanup_stale_shm();

    let ticks = Arc::new(AtomicU64::new(0));
    let node = BestEffortCounterNode {
        name: format!("mex_be_rate_{}", pid()),
        tick_count: ticks.clone(),
    };

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched.add(node).order(0).build().unwrap();
    sched.run_for(Duration::from_millis(300)).unwrap();

    let final_ticks = ticks.load(Ordering::SeqCst);
    // BestEffort at 100Hz for 300ms ≈ 30 ticks
    assert!(
        final_ticks >= 5,
        "BestEffort should tick at scheduler rate, got {}",
        final_ticks
    );
}
