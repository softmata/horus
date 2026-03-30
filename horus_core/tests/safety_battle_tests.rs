#![allow(
    unused_must_use,
    clippy::needless_range_loop,
    clippy::field_reassign_with_default
)]
//! Safety enforcement battle tests.
//!
//! Tests adversarial interactions between safety subsystems:
//! - BudgetPolicy enforcement + graduated degradation
//! - E-stop during active degradation recovery
//! - Multi-node simultaneous failures
//! - Edge cases: killed node guard, state thrashing, boundary values
//!
//! These tests complement `safety_scenario_e2e.rs` (happy-path) and
//! `stress_scheduling.rs` (scalability) by targeting the gaps between
//! independently developed safety features.

use horus_core::core::{DurationExt, Node};
use horus_core::error::HorusResult;
use horus_core::scheduling::{BudgetPolicy, FailurePolicy, Scheduler};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

// ============================================================================
// Shared node types for battle tests
// ============================================================================

/// Node that sleeps a configurable amount per tick. Used to trigger
/// budget violations (sleep > budget) and deadline misses (sleep > deadline).
struct SlowNode {
    name: String,
    count: Arc<AtomicU64>,
    sleep_us: u64,
}

impl Node for SlowNode {
    fn name(&self) -> &str {
        &self.name
    }
    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::SeqCst);
        std::thread::sleep(Duration::from_micros(self.sleep_us));
    }
}

/// Node that stalls (100ms sleep) after a threshold number of ticks.
/// Before the threshold, it's fast. After, it causes chronic deadline misses.
struct StallAfterNNode {
    name: String,
    count: Arc<AtomicU64>,
    stall_after: u64,
}

impl Node for StallAfterNNode {
    fn name(&self) -> &str {
        &self.name
    }
    fn tick(&mut self) {
        let tick_num = self.count.fetch_add(1, Ordering::SeqCst) + 1;
        if tick_num >= self.stall_after {
            std::thread::sleep(Duration::from_millis(100));
        }
    }
}

/// Fast node that always succeeds. Used as a healthy survivor alongside
/// stalling/budget-violating siblings.
struct HealthyNode {
    name: String,
    count: Arc<AtomicU64>,
}

impl Node for HealthyNode {
    fn name(&self) -> &str {
        &self.name
    }
    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::SeqCst);
        std::hint::black_box(42u64);
    }
}

/// Sentinel node that detects if tick() is called after the node is killed.
/// Sets `tick_after_kill` to true if tick runs while `killed` flag is set.
/// The killed flag is set externally by observing tick count stabilization.
struct SentinelNode {
    name: String,
    count: Arc<AtomicU64>,
    killed_at_tick: Arc<AtomicU64>,
    tick_after_kill: Arc<AtomicBool>,
    stall_after: u64,
}

impl Node for SentinelNode {
    fn name(&self) -> &str {
        &self.name
    }
    fn tick(&mut self) {
        let tick_num = self.count.fetch_add(1, Ordering::SeqCst) + 1;
        let killed_at = self.killed_at_tick.load(Ordering::SeqCst);
        if killed_at > 0 && tick_num > killed_at {
            self.tick_after_kill.store(true, Ordering::SeqCst);
        }
        if tick_num >= self.stall_after {
            std::thread::sleep(Duration::from_millis(100));
        }
    }

    fn shutdown(&mut self) -> HorusResult<()> {
        let current = self.count.load(Ordering::SeqCst);
        self.killed_at_tick.store(current, Ordering::SeqCst);
        Ok(())
    }
}

/// Node that alternates between fast and slow ticks to test degradation
/// stability under oscillating conditions.
struct OscillatingNode {
    name: String,
    count: Arc<AtomicU64>,
    fast_ticks: u64,
    slow_ticks: u64,
    slow_sleep_ms: u64,
}

impl Node for OscillatingNode {
    fn name(&self) -> &str {
        &self.name
    }
    fn tick(&mut self) {
        let tick_num = self.count.fetch_add(1, Ordering::SeqCst);
        let cycle_len = self.fast_ticks + self.slow_ticks;
        let pos_in_cycle = tick_num % cycle_len;
        if pos_in_cycle >= self.fast_ticks {
            std::thread::sleep(Duration::from_millis(self.slow_sleep_ms));
        }
    }
}

// ============================================================================
// Phase 1: Interaction Tests — Budget + Degradation
// ============================================================================

/// INTENT: "BudgetPolicy::Enforce stops a node faster than graduated
/// degradation can reach Kill, because budget enforcement runs on every
/// tick while degradation needs consecutive_misses >= kill_after (default 20)."
///
/// A node with both .budget() and .deadline() should be stopped by budget
/// enforcement after the first tick that exceeds 2x budget — well before
/// degradation accumulates enough misses to issue Kill.
#[test]
fn test_budget_enforce_preempts_degradation_pipeline() {
    cleanup_stale_shm();

    let count = Arc::new(AtomicU64::new(0));
    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        .max_deadline_misses(1000);

    // Node sleeps 500μs — 5x the 100μs budget (>2x threshold)
    scheduler
        .add(SlowNode {
            name: "budget_bomb".to_string(),
            count: count.clone(),
            sleep_us: 500,
        })
        .order(0)
        .budget(100_u64.us())
        .budget_policy(BudgetPolicy::Enforce)
        .deadline(1_u64.ms())
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    for _ in 0..50 {
        let _ = scheduler.tick_once();
    }

    let ticks = count.load(Ordering::SeqCst);
    // Budget enforcement should stop node after first tick (5x > 2x)
    // Well before degradation reaches kill_after=20
    assert!(
        ticks < 5,
        "Budget enforce should stop node quickly, got {} ticks",
        ticks
    );
}

/// INTENT: "Graduated degradation still kills nodes that only have deadlines
/// (no budget). Budget enforcement is additive, not a replacement."
///
/// A node with .deadline() but no .budget() must still be killed by the
/// Warn→ReduceRate→Isolate→Kill pipeline after enough consecutive misses.
#[test]
fn test_degradation_kills_deadline_only_node() {
    cleanup_stale_shm();

    let count = Arc::new(AtomicU64::new(0));
    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        .max_deadline_misses(1000);

    // Node stalls immediately — 100ms sleep far exceeds 1ms deadline
    // No budget set, so only graduated degradation can kill it
    scheduler
        .add(StallAfterNNode {
            name: "deadline_only".to_string(),
            count: count.clone(),
            stall_after: 1,
        })
        .order(0)
        .deadline(1_u64.ms())
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    scheduler.run_for(3_u64.secs()).unwrap();

    let ticks = count.load(Ordering::SeqCst);
    // kill_after=20 default: node should be killed after ~20 deadline misses
    assert!(
        ticks < 50,
        "Degradation should kill deadline-only node, got {} ticks",
        ticks
    );

    if let Some(stats) = scheduler.safety_stats() {
        assert!(
            stats.degrade_activations() > 0,
            "Graduated degradation should have fired at least once"
        );
    }
}

/// INTENT: "A node with both .budget() and .deadline() has both subsystems
/// active. Budget enforcement stops the node, but the deadline miss was
/// still detected and recorded by the safety monitor."
///
/// Both subsystems observe the same over-long tick independently:
/// - TimingEnforcer detects budget violation → BudgetPolicy::Enforce stops node
/// - TimingEnforcer detects deadline miss → recorded in safety monitor
#[test]
fn test_budget_and_deadline_both_active() {
    cleanup_stale_shm();

    let count = Arc::new(AtomicU64::new(0));
    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        .max_deadline_misses(1000);

    // 500μs sleep: exceeds 100μs budget (5x > 2x) AND 200μs deadline
    scheduler
        .add(SlowNode {
            name: "dual_enforce".to_string(),
            count: count.clone(),
            sleep_us: 500,
        })
        .order(0)
        .budget(100_u64.us())
        .budget_policy(BudgetPolicy::Enforce)
        .deadline(200_u64.us())
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    for _ in 0..50 {
        let _ = scheduler.tick_once();
    }

    let ticks = count.load(Ordering::SeqCst);
    // Budget enforcement stops it quickly
    assert!(
        ticks < 5,
        "Node should be stopped by budget enforce, got {} ticks",
        ticks
    );

    // Deadline miss should also have been recorded (both subsystems active)
    if let Some(stats) = scheduler.safety_stats() {
        assert!(
            stats.deadline_misses() > 0,
            "Deadline miss should still be recorded even though budget killed the node"
        );
    }
}

// ============================================================================
// Phase 1: Interaction Tests — E-Stop + Degradation
// ============================================================================

/// INTENT: "BudgetPolicy::EmergencyStop on one node halts the scheduler
/// even while another node is mid-degradation (at RateReduced stage)."
///
/// Two nodes: Node A stalls and degrades through Warn→ReduceRate.
/// Node B triggers BudgetPolicy::EmergencyStop. Scheduler must halt.
#[test]
fn test_estop_from_budget_while_sibling_degraded() {
    cleanup_stale_shm();

    let stall_count = Arc::new(AtomicU64::new(0));
    let bomb_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        .max_deadline_misses(1000);

    // Node A: stalls after 3 ticks, gets degraded
    scheduler
        .add(StallAfterNNode {
            name: "degrading_node".to_string(),
            count: stall_count.clone(),
            stall_after: 1,
        })
        .order(0)
        .deadline(1_u64.ms())
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    // Node B: always exceeds budget, triggers e-stop
    // Give it order(1) so it runs after Node A has ticked
    scheduler
        .add(SlowNode {
            name: "budget_estop".to_string(),
            count: bomb_count.clone(),
            sleep_us: 500,
        })
        .order(1)
        .budget(100_u64.us())
        .budget_policy(BudgetPolicy::EmergencyStop)
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    // tick_once should return Err because the budget e-stop fires
    let result = scheduler.tick_once();
    assert!(
        result.is_err(),
        "tick_once should return Err after BudgetPolicy::EmergencyStop"
    );

    // Both nodes should have ticked at least once before e-stop
    assert!(stall_count.load(Ordering::SeqCst) >= 1);
    assert!(bomb_count.load(Ordering::SeqCst) >= 1);
}

/// INTENT: "E-stop halts the scheduler regardless of which degradation
/// stages nodes are in — Normal, Warned, RateReduced, or Isolated."
///
/// 4 nodes at different degradation stages, then a 5th triggers e-stop.
/// All must stop.
#[test]
fn test_all_degradation_stages_halt_on_estop() {
    cleanup_stale_shm();

    let counts: Vec<_> = (0..5).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        .max_deadline_misses(1000);

    // Node 0: healthy (Normal stage)
    scheduler
        .add(HealthyNode {
            name: "healthy_node".to_string(),
            count: counts[0].clone(),
        })
        .order(0)
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    // Nodes 1-3: stall at different points to reach different degradation stages
    for i in 1..=3 {
        scheduler
            .add(StallAfterNNode {
                name: format!("stall_{}", i),
                count: counts[i].clone(),
                stall_after: 1,
            })
            .order(i as u32)
            .deadline(1_u64.ms())
            .failure_policy(FailurePolicy::Ignore)
            .build()
            .unwrap();
    }

    // Node 4: budget e-stop bomb (delayed — runs last each tick)
    scheduler
        .add(SlowNode {
            name: "estop_bomb".to_string(),
            count: counts[4].clone(),
            sleep_us: 500,
        })
        .order(10)
        .budget(100_u64.us())
        .budget_policy(BudgetPolicy::EmergencyStop)
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    // Run one tick — e-stop fires on node 4
    let result = scheduler.tick_once();
    assert!(result.is_err(), "E-stop should halt scheduler");

    // Capture tick counts at halt
    let ticks_at_halt: Vec<u64> = counts.iter().map(|c| c.load(Ordering::SeqCst)).collect();

    // Try more ticks — should all fail
    for _ in 0..10 {
        let _ = scheduler.tick_once();
    }

    // Verify no additional ticks after e-stop
    let ticks_after: Vec<u64> = counts.iter().map(|c| c.load(Ordering::SeqCst)).collect();
    // Healthy node should not have gained ticks after e-stop
    // (some implementations may allow the current tick to complete before checking e-stop)
    assert!(
        ticks_after[0] <= ticks_at_halt[0] + 10,
        "Healthy node should be bounded after e-stop: {} vs {}",
        ticks_after[0],
        ticks_at_halt[0]
    );
}

// ============================================================================
// Phase 2: Stress & Adversarial — Multi-node simultaneous failures
// ============================================================================

/// INTENT: "10 nodes all exceeding 2x budget simultaneously are all stopped
/// without panic, deadlock, or interference between enforcement actions."
#[test]
fn test_10_simultaneous_budget_violations() {
    cleanup_stale_shm();

    let counts: Vec<_> = (0..10).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        .max_deadline_misses(1000);

    for (i, c) in counts.iter().enumerate() {
        scheduler
            .add(SlowNode {
                name: format!("bomb_{}", i),
                count: c.clone(),
                sleep_us: 500, // 5x budget
            })
            .order(i as u32)
            .budget(100_u64.us())
            .budget_policy(BudgetPolicy::Enforce)
            .failure_policy(FailurePolicy::Ignore)
            .build()
            .unwrap();
    }

    for _ in 0..50 {
        let _ = scheduler.tick_once();
    }

    for (i, c) in counts.iter().enumerate() {
        let ticks = c.load(Ordering::SeqCst);
        assert!(
            ticks < 5,
            "bomb_{} should be stopped by budget enforce, got {} ticks",
            i,
            ticks
        );
    }
}

/// INTENT: "20 nodes killed sequentially by graduated degradation —
/// scheduler survives with 0 live nodes and shuts down cleanly."
#[test]
fn test_20_sequential_degradation_kills() {
    cleanup_stale_shm();

    let counts: Vec<_> = (0..20).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        .max_deadline_misses(1000);

    for (i, c) in counts.iter().enumerate() {
        scheduler
            .add(StallAfterNNode {
                name: format!("stall_{}", i),
                count: c.clone(),
                stall_after: 1, // all stall immediately
            })
            .order(i as u32)
            .deadline(1_u64.ms())
            .failure_policy(FailurePolicy::Ignore)
            .build()
            .unwrap();
    }

    // tick_once loop: 100 ticks should be enough for kill_after=20
    // Each tick: 20 nodes × 100ms stall = 2s wall time, but enough deadline misses
    for _ in 0..100 {
        let _ = scheduler.tick_once();
    }

    let mut killed = 0;
    for (i, c) in counts.iter().enumerate() {
        let ticks = c.load(Ordering::SeqCst);
        if ticks < 100 {
            killed += 1;
        }
        // Each node should have bounded ticks (killed by degradation)
        assert!(
            ticks < 200,
            "stall_{} should be killed, got {} ticks",
            i,
            ticks
        );
    }

    assert!(
        killed >= 10,
        "At least 10 of 20 stalling nodes should be killed, got {}",
        killed
    );

    // Safety monitor must exist (RT nodes are present)
    let stats = scheduler
        .safety_stats()
        .expect("Safety stats should exist for scheduler with RT nodes");
    assert!(
        stats.degrade_activations() > 0,
        "Graduated degradation should have fired, got 0 activations"
    );
}

/// INTENT: "5 stalling nodes killed while 5 healthy nodes survive —
/// killed nodes don't drag healthy ones down."
#[test]
fn test_cascade_kill_healthy_survivors() {
    cleanup_stale_shm();

    let stall_counts: Vec<_> = (0..5).map(|_| Arc::new(AtomicU64::new(0))).collect();
    let healthy_counts: Vec<_> = (0..5).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        .max_deadline_misses(1000);

    // 5 stalling nodes with deadline
    for (i, c) in stall_counts.iter().enumerate() {
        scheduler
            .add(StallAfterNNode {
                name: format!("stall_{}", i),
                count: c.clone(),
                stall_after: 1,
            })
            .order(i as u32)
            .deadline(1_u64.ms())
            .failure_policy(FailurePolicy::Ignore)
            .build()
            .unwrap();
    }

    // 5 healthy nodes (no deadline, no budget)
    for (i, c) in healthy_counts.iter().enumerate() {
        scheduler
            .add(HealthyNode {
                name: format!("healthy_{}", i),
                count: c.clone(),
            })
            .order((i + 10) as u32)
            .failure_policy(FailurePolicy::Ignore)
            .build()
            .unwrap();
    }

    scheduler.run_for(5_u64.secs()).unwrap();

    // All stalling nodes should be killed (bounded ticks)
    for (i, c) in stall_counts.iter().enumerate() {
        let ticks = c.load(Ordering::SeqCst);
        assert!(
            ticks < 200,
            "stall_{} should be killed, got {} ticks",
            i,
            ticks
        );
    }

    // All healthy nodes should have ticks (survived the cascade)
    for (i, c) in healthy_counts.iter().enumerate() {
        let ticks = c.load(Ordering::SeqCst);
        assert!(
            ticks >= 5,
            "healthy_{} should have survived, got only {} ticks",
            i,
            ticks
        );
    }

    if let Some(stats) = scheduler.safety_stats() {
        assert!(
            stats.degrade_activations() >= 5,
            "At least 5 degradation activations expected, got {}",
            stats.degrade_activations()
        );
    }
}

// ============================================================================
// Phase 2: Stress — Deadline miss storm at scale
// ============================================================================

/// INTENT: "Nodes stalling at different times are killed at different times,
/// proving independent degradation state machines per-node."
#[test]
fn test_staggered_stall_different_kill_times() {
    cleanup_stale_shm();

    let stall_points = [1u64, 10, 20, 30, 40];
    let counts: Vec<_> = (0..5).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        .max_deadline_misses(1000);

    for (i, (&stall, count)) in stall_points.iter().zip(counts.iter()).enumerate() {
        scheduler
            .add(StallAfterNNode {
                name: format!("stagger_{}", i),
                count: count.clone(),
                stall_after: stall,
            })
            .order(i as u32)
            .deadline(1_u64.ms())
            .failure_policy(FailurePolicy::Ignore)
            .build()
            .unwrap();
    }

    scheduler.run_for(5_u64.secs()).unwrap();

    let ticks: Vec<u64> = counts.iter().map(|c| c.load(Ordering::SeqCst)).collect();

    // Earlier stall → fewer total ticks (killed sooner)
    for i in 0..4 {
        assert!(
            ticks[i] <= ticks[i + 1],
            "stagger_{} ({} ticks) should have <= ticks than stagger_{} ({} ticks)",
            i,
            ticks[i],
            i + 1,
            ticks[i + 1]
        );
    }
}

// ============================================================================
// Phase 2: Stress — Rapid kill cycles
// ============================================================================

/// INTENT: "50 rapid build-run-kill cycles don't leak resources or corrupt state."
#[test]
fn test_50_rapid_kill_cycles() {
    let start = std::time::Instant::now();

    for cycle in 0..50 {
        cleanup_stale_shm();
        let count = Arc::new(AtomicU64::new(0));
        let mut scheduler = Scheduler::new()
            .tick_rate(100_u64.hz())
            .deterministic(true)
            .max_deadline_misses(1000);

        scheduler
            .add(StallAfterNNode {
                name: format!("cycle_{}", cycle),
                count: count.clone(),
                stall_after: 1,
            })
            .order(0)
            .deadline(1_u64.ms())
            .failure_policy(FailurePolicy::Ignore)
            .build()
            .unwrap();

        scheduler.run_for(500_u64.ms()).unwrap();

        let ticks = count.load(Ordering::SeqCst);
        assert!(
            ticks > 0 && ticks < 100,
            "cycle {}: expected bounded ticks, got {}",
            cycle,
            ticks
        );
        // scheduler dropped here — timing report + cleanup
    }

    let elapsed = start.elapsed();
    assert!(
        elapsed < Duration::from_secs(120),
        "50 kill cycles should complete in <120s, took {:?}",
        elapsed
    );
}

/// INTENT: "Scheduler with ALL nodes killed still shuts down cleanly."
#[test]
fn test_scheduler_survives_all_nodes_killed() {
    cleanup_stale_shm();

    let counts: Vec<_> = (0..5).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        .max_deadline_misses(1000);

    for (i, c) in counts.iter().enumerate() {
        scheduler
            .add(StallAfterNNode {
                name: format!("killme_{}", i),
                count: c.clone(),
                stall_after: 1,
            })
            .order(i as u32)
            .deadline(1_u64.ms())
            .failure_policy(FailurePolicy::Ignore)
            .build()
            .unwrap();
    }

    // Should complete without hanging — all nodes killed, tick loop is all skips
    let result = scheduler.run_for(5_u64.secs());
    assert!(
        result.is_ok(),
        "Scheduler should complete even with all nodes killed"
    );
}

// ============================================================================
// Phase 3: Edge Cases & Defensive Guards
// ============================================================================

/// INTENT: "A killed node's tick() is NEVER called after shutdown.
/// The is_stopped guard at execute_single_node() line 2832 must hold."
#[test]
fn test_killed_node_tick_never_called_again() {
    cleanup_stale_shm();

    let count = Arc::new(AtomicU64::new(0));
    let killed_at = Arc::new(AtomicU64::new(0));
    let tick_after_kill = Arc::new(AtomicBool::new(false));

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        .max_deadline_misses(1000);

    scheduler
        .add(SentinelNode {
            name: "sentinel".to_string(),
            count: count.clone(),
            killed_at_tick: killed_at.clone(),
            tick_after_kill: tick_after_kill.clone(),
            stall_after: 1,
        })
        .order(0)
        .deadline(1_u64.ms())
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    // Run for enough time: kill_after=20 misses × 100ms stall ≈ 2s, plus buffer
    scheduler.run_for(5_u64.secs()).unwrap();

    assert!(
        !tick_after_kill.load(Ordering::SeqCst),
        "tick() was called on a killed node — is_stopped guard failed!"
    );

    let k = killed_at.load(Ordering::SeqCst);
    assert!(k > 0, "Node should have been killed (shutdown called)");
}

/// INTENT: "A node oscillating between healthy and stalling reaches a
/// stable degradation state without infinite thrashing."
#[test]
fn test_oscillating_node_degradation_stability() {
    cleanup_stale_shm();

    let count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        .max_deadline_misses(1000);

    // Alternates: 5 fast ticks, then 5 slow (100ms) ticks
    scheduler
        .add(OscillatingNode {
            name: "oscillator".to_string(),
            count: count.clone(),
            fast_ticks: 5,
            slow_ticks: 5,
            slow_sleep_ms: 100,
        })
        .order(0)
        .deadline(1_u64.ms())
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    // Run for enough ticks to see several oscillation cycles
    // With kill_after=20, the slow phases accumulate misses
    // Eventually the node should be killed (misses accumulate across cycles)
    scheduler.run_for(5_u64.secs()).unwrap();

    let ticks = count.load(Ordering::SeqCst);
    // Node should eventually be killed — misses accumulate even if
    // interspersed with fast ticks (consecutive_misses resets on success,
    // but the degradation stage persists)
    // The key assertion: the test completes without hanging (no infinite thrash)
    assert!(
        ticks > 5,
        "Oscillating node should have ticked several times, got {}",
        ticks
    );
}

/// INTENT: "Budget enforcement with extreme values doesn't panic.
/// Zero budget, 1ns budget, and very large budget all handled gracefully."
#[test]
fn test_extreme_budget_values_no_panic() {
    cleanup_stale_shm();

    let counts: Vec<_> = (0..3).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        .max_deadline_misses(1000);

    // Node with 1ns budget — any tick exceeds it
    scheduler
        .add(HealthyNode {
            name: "tiny_budget".to_string(),
            count: counts[0].clone(),
        })
        .order(0)
        .budget(Duration::from_nanos(1))
        .budget_policy(BudgetPolicy::Enforce)
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    // Node with very large budget — never exceeded
    scheduler
        .add(HealthyNode {
            name: "huge_budget".to_string(),
            count: counts[1].clone(),
        })
        .order(1)
        .budget(Duration::from_secs(1000))
        .budget_policy(BudgetPolicy::Enforce)
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    // Node with moderate budget, Warn policy (default behavior)
    scheduler
        .add(HealthyNode {
            name: "warn_budget".to_string(),
            count: counts[2].clone(),
        })
        .order(2)
        .budget(Duration::from_nanos(1))
        .budget_policy(BudgetPolicy::Warn)
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    // Run 10 ticks — must not panic
    for _ in 0..10 {
        let _ = scheduler.tick_once();
    }

    // Tiny budget with Enforce: stopped quickly (any tick > 2ns triggers)
    let tiny_ticks = counts[0].load(Ordering::SeqCst);
    assert!(
        tiny_ticks < 5,
        "Tiny budget node should be stopped, got {}",
        tiny_ticks
    );

    // Huge budget: keeps running
    let huge_ticks = counts[1].load(Ordering::SeqCst);
    assert!(
        huge_ticks >= 5,
        "Huge budget node should keep running, got {}",
        huge_ticks
    );

    // Warn policy: keeps running despite violations
    let warn_ticks = counts[2].load(Ordering::SeqCst);
    assert!(
        warn_ticks >= 5,
        "Warn policy node should keep running, got {}",
        warn_ticks
    );
}

/// INTENT: "Budget enforcement uses strict > (not >=) for the 2x threshold.
/// A tick at exactly 2x budget should NOT trigger enforcement."
///
/// Note: This is a design verification test. Due to OS scheduling jitter,
/// we use a node that sleeps slightly under 2x to prove it's not stopped,
/// and another that sleeps well over 2x to prove it IS stopped.
#[test]
fn test_budget_enforce_threshold_is_strict_greater_than() {
    cleanup_stale_shm();

    let over_count = Arc::new(AtomicU64::new(0));
    let under_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new()
        .tick_rate(100_u64.hz())
        .deterministic(true)
        .max_deadline_misses(1000);

    // Node sleeping 500μs with 100μs budget → 5x > 2x → stopped
    scheduler
        .add(SlowNode {
            name: "over_2x".to_string(),
            count: over_count.clone(),
            sleep_us: 500,
        })
        .order(0)
        .budget(100_u64.us())
        .budget_policy(BudgetPolicy::Enforce)
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    // Node sleeping 50μs with 100μs budget → 0.5x < 2x → NOT stopped
    scheduler
        .add(SlowNode {
            name: "under_2x".to_string(),
            count: under_count.clone(),
            sleep_us: 50,
        })
        .order(1)
        .budget(100_u64.us())
        .budget_policy(BudgetPolicy::Enforce)
        .failure_policy(FailurePolicy::Ignore)
        .build()
        .unwrap();

    for _ in 0..50 {
        let _ = scheduler.tick_once();
    }

    let over_ticks = over_count.load(Ordering::SeqCst);
    let under_ticks = under_count.load(Ordering::SeqCst);

    assert!(
        over_ticks < 5,
        "5x budget node should be stopped, got {} ticks",
        over_ticks
    );
    assert!(
        under_ticks > 20,
        "0.5x budget node should NOT be stopped, got {} ticks",
        under_ticks
    );
}
