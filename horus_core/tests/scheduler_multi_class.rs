//! Integration tests for scheduler with multiple execution classes and
//! selective node filtering via `tick_for`.
//!
//! Covers:
//! - All five execution classes coexisting in one scheduler
//! - Selective node filtering with `tick_for`
//! - `tick_once` (all) vs `tick_for` (filtered) counter comparison
//! - `run_for` completion and tick count

use horus_core::core::{DurationExt, Node, NodeInfo};
use horus_core::error::Result;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;

mod common;
use common::cleanup_stale_shm;

// ============================================================================
// Mock nodes
// ============================================================================

/// Simple counter node — increments an atomic on each tick.
struct CounterNode {
    name: String,
    count: Arc<AtomicU64>,
}

impl CounterNode {
    fn new(name: &str) -> (Self, Arc<AtomicU64>) {
        let count = Arc::new(AtomicU64::new(0));
        (
            Self {
                name: name.to_string(),
                count: count.clone(),
            },
            count,
        )
    }
}

impl Node for CounterNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn init(&mut self) -> Result<()> {
        Ok(())
    }

    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::SeqCst);
    }
}

// ============================================================================
// Test 1: All five execution classes coexist
// ============================================================================

/// Create a scheduler with one node per execution class (BestEffort, Compute,
/// RT-like, Event, AsyncIo). Run for 200ms and verify all non-event classes
/// tick at least once. Event node is triggered mid-run via `notify_event`.
#[test]
fn test_all_execution_classes_coexist() {
    cleanup_stale_shm();

    let (be_node, be_count) = CounterNode::new("mc_besteffort");
    let (compute_node, compute_count) = CounterNode::new("mc_compute");
    let (rt_node, rt_count) = CounterNode::new("mc_rt");
    let (event_node, event_count) = CounterNode::new("mc_event");
    let (async_node, async_count) = CounterNode::new("mc_async");

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    // BestEffort (default — no modifier)
    scheduler.add(be_node).order(0).build().unwrap();

    // Compute
    scheduler
        .add(compute_node)
        .order(5)
        .compute()
        .build()
        .unwrap();

    // RT-like (rate + budget auto-promotes to Rt)
    scheduler
        .add(rt_node)
        .order(10)
        .rate(100_u64.hz())
        .budget(1_u64.ms())
        .build()
        .unwrap();

    // Event (triggered by topic)
    scheduler
        .add(event_node)
        .order(15)
        .on("mc_trigger_topic")
        .build()
        .unwrap();

    // AsyncIo
    scheduler
        .add(async_node)
        .order(20)
        .async_io()
        .build()
        .unwrap();

    // Run scheduler in a background thread so we can send event notifications
    let handle = std::thread::spawn(move || {
        scheduler.run_for(200_u64.ms()).unwrap();
    });

    // Give the scheduler time to start, then trigger the event node
    std::thread::sleep(50_u64.ms());
    NodeInfo::notify_event("mc_event");
    std::thread::sleep(50_u64.ms());
    NodeInfo::notify_event("mc_event");

    handle.join().unwrap();

    // All non-event classes must have ticked
    assert!(
        be_count.load(Ordering::SeqCst) > 0,
        "BestEffort node should tick, got {}",
        be_count.load(Ordering::SeqCst)
    );
    assert!(
        compute_count.load(Ordering::SeqCst) > 0,
        "Compute node should tick, got {}",
        compute_count.load(Ordering::SeqCst)
    );
    assert!(
        rt_count.load(Ordering::SeqCst) > 0,
        "RT node should tick, got {}",
        rt_count.load(Ordering::SeqCst)
    );
    assert!(
        async_count.load(Ordering::SeqCst) > 0,
        "AsyncIo node should tick, got {}",
        async_count.load(Ordering::SeqCst)
    );

    // Event node should have ticked from our notifications (timing-dependent,
    // but we sent 2 notifications with generous sleep).
    // Use >= 1 rather than == 2 because event delivery is timing-sensitive.
    assert!(
        event_count.load(Ordering::SeqCst) >= 1,
        "Event node should tick at least once after notifications, got {}",
        event_count.load(Ordering::SeqCst)
    );
}

// ============================================================================
// Test 2: tick_for selective node filtering
// ============================================================================

/// Add 3 nodes named "alpha", "beta", "gamma". Use `tick_for` to run only
/// "alpha" and "gamma" for a short duration. Assert alpha and gamma ticked
/// but beta did NOT.
#[test]
fn test_tick_for_selective() {
    cleanup_stale_shm();

    let (alpha_node, alpha_count) = CounterNode::new("alpha");
    let (beta_node, beta_count) = CounterNode::new("beta");
    let (gamma_node, gamma_count) = CounterNode::new("gamma");

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler.add(alpha_node).order(0).build().unwrap();
    scheduler.add(beta_node).order(1).build().unwrap();
    scheduler.add(gamma_node).order(2).build().unwrap();

    // Run only alpha and gamma for 100ms
    scheduler
        .tick_for(&["alpha", "gamma"], 100_u64.ms())
        .unwrap();

    let alpha_ticks = alpha_count.load(Ordering::SeqCst);
    let beta_ticks = beta_count.load(Ordering::SeqCst);
    let gamma_ticks = gamma_count.load(Ordering::SeqCst);

    assert!(
        alpha_ticks > 0,
        "alpha should have ticked, got {}",
        alpha_ticks
    );
    assert!(
        gamma_ticks > 0,
        "gamma should have ticked, got {}",
        gamma_ticks
    );
    assert_eq!(
        beta_ticks, 0,
        "beta should NOT have ticked when filtered out, got {}",
        beta_ticks
    );
}

// ============================================================================
// Test 3: tick_once (all) vs tick_for (filtered) counter comparison
// ============================================================================

/// Add 3 nodes. Run `tick_once()` (all tick). Then run `tick_for` with only
/// "alpha" for a short duration. Compare counters to verify filtering works:
/// after tick_once, all have 1 tick each. After tick_for, only alpha gains
/// additional ticks.
#[test]
fn test_tick_once_all_vs_tick_for_filtered() {
    cleanup_stale_shm();

    let (alpha_node, alpha_count) = CounterNode::new("tf_alpha");
    let (beta_node, beta_count) = CounterNode::new("tf_beta");
    let (gamma_node, gamma_count) = CounterNode::new("tf_gamma");

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler.add(alpha_node).order(0).build().unwrap();
    scheduler.add(beta_node).order(1).build().unwrap();
    scheduler.add(gamma_node).order(2).build().unwrap();

    // Phase 1: tick_once() — all nodes tick once
    scheduler.tick_once().unwrap();

    let alpha_after_once = alpha_count.load(Ordering::SeqCst);
    let beta_after_once = beta_count.load(Ordering::SeqCst);
    let gamma_after_once = gamma_count.load(Ordering::SeqCst);

    assert_eq!(
        alpha_after_once, 1,
        "alpha should have 1 tick after tick_once, got {}",
        alpha_after_once
    );
    assert_eq!(
        beta_after_once, 1,
        "beta should have 1 tick after tick_once, got {}",
        beta_after_once
    );
    assert_eq!(
        gamma_after_once, 1,
        "gamma should have 1 tick after tick_once, got {}",
        gamma_after_once
    );

    // Phase 2: tick_for with only "tf_alpha" — run for 100ms
    scheduler
        .tick_for(&["tf_alpha"], 100_u64.ms())
        .unwrap();

    let alpha_final = alpha_count.load(Ordering::SeqCst);
    let beta_final = beta_count.load(Ordering::SeqCst);
    let gamma_final = gamma_count.load(Ordering::SeqCst);

    // Alpha should have gained additional ticks beyond the initial 1
    assert!(
        alpha_final > alpha_after_once,
        "alpha should gain ticks from tick_for, before={} after={}",
        alpha_after_once,
        alpha_final
    );

    // Beta and gamma should still be at 1 (unchanged from tick_once)
    assert_eq!(
        beta_final, beta_after_once,
        "beta should not gain ticks during filtered tick_for, before={} after={}",
        beta_after_once, beta_final
    );
    assert_eq!(
        gamma_final, gamma_after_once,
        "gamma should not gain ticks during filtered tick_for, before={} after={}",
        gamma_after_once, gamma_final
    );
}

// ============================================================================
// Test 4: run_for completes and ticks multiple times
// ============================================================================

/// Add 1 node, run_for(100ms). Assert the node ticked multiple times
/// (counter > 5). Assert run_for returns without hanging.
#[test]
fn test_scheduler_run_for_completes() {
    cleanup_stale_shm();

    let (node, count) = CounterNode::new("runfor_node");

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());
    scheduler.add(node).order(0).build().unwrap();

    let timed_out = Arc::new(AtomicBool::new(false));
    let timed_out_clone = timed_out.clone();

    // Run in a thread with a watchdog to detect hangs
    let handle = std::thread::spawn(move || {
        scheduler.run_for(100_u64.ms()).unwrap();
    });

    // Watchdog: if run_for hasn't returned in 5 seconds, something is wrong
    let watchdog = std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_secs(5));
        timed_out_clone.store(true, Ordering::SeqCst);
    });

    handle.join().unwrap();

    assert!(
        !timed_out.load(Ordering::SeqCst),
        "run_for(100ms) should complete well before 5s watchdog"
    );

    let ticks = count.load(Ordering::SeqCst);
    assert!(
        ticks > 5,
        "Node at 100Hz for 100ms should tick more than 5 times, got {}",
        ticks
    );

    // Clean up watchdog (it's still sleeping — just let it go)
    drop(watchdog);
}
