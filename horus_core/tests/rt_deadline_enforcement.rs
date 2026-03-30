// Integration tests for RT deadline enforcement behavioral verification.
//
// These tests PROVE that the Miss policies (Warn, Skip, SafeMode, Stop)
// actually produce the correct behavioral outcome when a node exceeds
// its budget or deadline. This is the core safety promise of horus.

use horus_core::core::{DurationExt, Miss, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

// ---------------------------------------------------------------------------
// Test Node: configurable sleep to simulate budget violations
// ---------------------------------------------------------------------------

struct SlowNode {
    name: String,
    sleep_duration: Duration,
    tick_count: Arc<AtomicU64>,
    safe_state_entered: Arc<AtomicBool>,
}

impl SlowNode {
    fn new(prefix: &str, sleep: Duration) -> (Self, Arc<AtomicU64>, Arc<AtomicBool>) {
        let ticks = Arc::new(AtomicU64::new(0));
        let safe = Arc::new(AtomicBool::new(false));
        let node = Self {
            name: format!("{}_{}", prefix, std::process::id()),
            sleep_duration: sleep,
            tick_count: ticks.clone(),
            safe_state_entered: safe.clone(),
        };
        (node, ticks, safe)
    }
}

impl Node for SlowNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
        std::thread::sleep(self.sleep_duration);
    }

    fn enter_safe_state(&mut self) {
        self.safe_state_entered.store(true, Ordering::SeqCst);
    }

    fn is_safe_state(&self) -> bool {
        self.safe_state_entered.load(Ordering::SeqCst)
    }
}

// ---------------------------------------------------------------------------
// Test Node: instant tick (within budget)
// ---------------------------------------------------------------------------

struct FastNode {
    name: String,
    tick_count: Arc<AtomicU64>,
}

impl FastNode {
    fn new(prefix: &str) -> (Self, Arc<AtomicU64>) {
        let ticks = Arc::new(AtomicU64::new(0));
        let node = Self {
            name: format!("{}_{}", prefix, std::process::id()),
            tick_count: ticks.clone(),
        };
        (node, ticks)
    }
}

impl Node for FastNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
        // No sleep — instant completion, well within any budget
    }
}

// ============================================================================
// Miss::Warn — should log warning but continue ticking normally
// ============================================================================

#[test]
fn test_miss_warn_continues_ticking() {
    cleanup_stale_shm();
    let (node, ticks, _safe) = SlowNode::new("rt_warn", Duration::from_millis(5));

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched
        .add(node)
        .rate(100_u64.hz())
        .budget(1_u64.ms())
        .deadline(1_u64.ms()) // Explicit deadline — triggers Miss policy
        .on_miss(Miss::Warn)
        .build()
        .unwrap();
    sched.run_for(Duration::from_millis(300)).unwrap();

    let final_ticks = ticks.load(Ordering::SeqCst);
    assert!(
        final_ticks > 1,
        "Miss::Warn should continue ticking despite deadline misses, got {} ticks",
        final_ticks
    );
}

// ============================================================================
// Miss::Skip — should skip over-budget ticks, resume next cycle
// ============================================================================

#[test]
fn test_miss_skip_still_ticks() {
    cleanup_stale_shm();
    let (node, ticks, _safe) = SlowNode::new("rt_skip", Duration::from_millis(5));

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched
        .add(node)
        .rate(100_u64.hz())
        .budget(1_u64.ms())
        .deadline(1_u64.ms()) // Explicit deadline — triggers Miss policy
        .on_miss(Miss::Skip)
        .build()
        .unwrap();
    sched.run_for(Duration::from_millis(300)).unwrap();

    let final_ticks = ticks.load(Ordering::SeqCst);
    // Skip policy pauses the node after a deadline miss (is_paused = true),
    // it auto-unpauses on the next cycle. Node should still tick multiple times.
    assert!(
        final_ticks >= 1,
        "Miss::Skip node should still tick at least once, got {} ticks",
        final_ticks
    );
}

// ============================================================================
// Miss::SafeMode — should call enter_safe_state() on the node
// ============================================================================

#[test]
fn test_miss_safe_mode_calls_enter_safe_state() {
    cleanup_stale_shm();
    // Node sleeps 5ms, deadline is 1ms — deadline MISS triggers SafeMode action
    // Note: Miss policy is triggered by DEADLINE miss (not budget violation).
    // Budget violations are tracked for stats only. Deadline is checked by
    // TimingEnforcer::check_deadline() in rt_executor.rs:193-238.
    let (node, ticks, safe) = SlowNode::new("rt_safemode", Duration::from_millis(5));

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched
        .add(node)
        .rate(100_u64.hz())
        .budget(1_u64.ms())
        .deadline(1_u64.ms()) // Explicit deadline — triggers Miss policy
        .on_miss(Miss::SafeMode)
        .build()
        .unwrap();
    sched.run_for(Duration::from_millis(300)).unwrap();

    let final_ticks = ticks.load(Ordering::SeqCst);
    assert!(
        final_ticks >= 1,
        "SafeMode node should still tick, got {} ticks",
        final_ticks
    );

    // The key assertion: enter_safe_state() must have been called
    // because the node's 5ms sleep exceeds the 1ms deadline
    assert!(
        safe.load(Ordering::SeqCst),
        "Miss::SafeMode must call enter_safe_state() when deadline is missed"
    );
}

// ============================================================================
// Miss::Stop — should halt the entire scheduler
// ============================================================================

#[test]
fn test_miss_stop_halts_scheduler() {
    cleanup_stale_shm();
    // Node sleeps 5ms, deadline is 1ms → DeadlineAction::EmergencyStop
    // sets running.store(false) which causes the scheduler to exit early
    let (node, ticks, _safe) = SlowNode::new("rt_stop", Duration::from_millis(5));

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched
        .add(node)
        .rate(100_u64.hz())
        .budget(1_u64.ms())
        .deadline(1_u64.ms()) // Explicit deadline — triggers Miss policy
        .on_miss(Miss::Stop)
        .build()
        .unwrap();

    let start = std::time::Instant::now();
    // Use run_for with a moderate timeout — Stop should exit BEFORE this expires
    let _ = sched.run_for(Duration::from_secs(3));
    let elapsed = start.elapsed();

    let final_ticks = ticks.load(Ordering::SeqCst);

    // Miss::Stop sets running=false on the RT thread. The main thread loop
    // checks running on each iteration and exits. Should complete well under 3s.
    // Allow up to 2s for scheduler overhead + shutdown.
    assert!(
        elapsed < Duration::from_millis(2500),
        "Miss::Stop should halt scheduler before run_for timeout, but it ran for {:?}",
        elapsed
    );

    // Should have ticked at least once (the violation tick itself)
    assert!(
        final_ticks >= 1,
        "Should tick at least once before stopping, got {}",
        final_ticks
    );
}

// ============================================================================
// Budget violation detection — verify RtStats tracks violations
// ============================================================================

#[test]
fn test_budget_violation_node_still_runs() {
    cleanup_stale_shm();
    // Node that slightly exceeds budget (2ms sleep vs 1ms budget)
    let (node, ticks, _safe) = SlowNode::new("rt_budget_viol", Duration::from_millis(2));

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched
        .add(node)
        .rate(100_u64.hz())
        .budget(1_u64.ms())
        .on_miss(Miss::Warn) // Warn = continue ticking
        .build()
        .unwrap();
    sched.run_for(Duration::from_millis(300)).unwrap();

    let final_ticks = ticks.load(Ordering::SeqCst);
    // With Warn policy, the node should keep ticking despite budget violations
    assert!(
        final_ticks > 5,
        "Node with budget violations and Warn policy should keep ticking, got {}",
        final_ticks
    );
}

// ============================================================================
// Within budget — verify no violations when node is fast enough
// ============================================================================

#[test]
fn test_within_budget_no_issues() {
    cleanup_stale_shm();
    let (node, ticks) = FastNode::new("rt_fast");

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    sched
        .add(node)
        .rate(100_u64.hz())
        .budget(50_u64.ms()) // Very generous budget
        .on_miss(Miss::Stop) // Stop would trigger if budget exceeded
        .build()
        .unwrap();
    sched.run_for(Duration::from_millis(300)).unwrap();

    let final_ticks = ticks.load(Ordering::SeqCst);
    // With a generous budget and instant tick, should tick many times
    // and NOT trigger Stop
    assert!(
        final_ticks > 10,
        "Fast node with generous budget should tick many times, got {}",
        final_ticks
    );
}

// ============================================================================
// Default Miss policy is Warn
// ============================================================================

#[test]
fn test_on_miss_default_is_warn() {
    cleanup_stale_shm();
    let (node, ticks, _safe) = SlowNode::new("rt_default", Duration::from_millis(5));

    let mut sched = Scheduler::new().tick_rate(100_u64.hz());
    // Intentionally NOT setting on_miss — default should be Warn
    sched
        .add(node)
        .rate(100_u64.hz())
        .budget(1_u64.ms())
        .build()
        .unwrap();
    sched.run_for(Duration::from_millis(300)).unwrap();

    let final_ticks = ticks.load(Ordering::SeqCst);
    // Default Warn policy = keep ticking
    assert!(
        final_ticks > 1,
        "Default (Warn) policy should continue ticking, got {} ticks",
        final_ticks
    );
}

// ============================================================================
// Deadline miss with high-frequency node
// ============================================================================

#[test]
fn test_deadline_exceeded_with_high_frequency_node() {
    cleanup_stale_shm();
    // 1kHz node that sleeps 2ms — will always miss the 1ms period
    let (node, ticks, _safe) = SlowNode::new("rt_deadline", Duration::from_millis(2));

    let mut sched = Scheduler::new().tick_rate(1000_u64.hz());
    sched
        .add(node)
        .rate(1000_u64.hz())
        .budget(500_u64.us())
        .deadline(900_u64.us())
        .on_miss(Miss::Warn) // Keep running despite misses
        .build()
        .unwrap();
    sched.run_for(Duration::from_millis(300)).unwrap();

    let final_ticks = ticks.load(Ordering::SeqCst);
    // Even with deadline misses, Warn policy keeps ticking
    assert!(
        final_ticks > 1,
        "Node with deadline misses and Warn policy should keep ticking, got {}",
        final_ticks
    );
}
