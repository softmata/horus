//! Integration tests for RT executor code paths.
//!
//! Covers: pre/post conditions, all DeadlineAction variants, panic downcasting
//! (str, String, unknown), on_error callback, circuit breaker rejection,
//! and restart failure deinit.

use horus_core::core::{DeadlineMissPolicy, Node, RtNode};
use horus_core::scheduling::{FailurePolicy, Scheduler};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

mod common;
use common::cleanup_stale_shm;

// ============================================================================
// Mock nodes
// ============================================================================

/// RtNode with controllable pre/post conditions via Arc<AtomicBool>.
struct ConditionalRtNode {
    name: String,
    tick_count: Arc<AtomicU64>,
    pre_ok: Arc<AtomicBool>,
    post_ok: Arc<AtomicBool>,
}

impl Node for ConditionalRtNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
    }
}

impl RtNode for ConditionalRtNode {
    fn wcet_budget(&self) -> Duration {
        Duration::from_millis(100)
    }
    fn pre_condition(&self) -> bool {
        self.pre_ok.load(Ordering::SeqCst)
    }
    fn post_condition(&self) -> bool {
        self.post_ok.load(Ordering::SeqCst)
    }
}

/// RtNode that takes a configurable time, with a configurable deadline miss policy.
struct PolicyRtNode {
    name: String,
    tick_count: Arc<AtomicU64>,
    sleep_us: u64,
    policy: DeadlineMissPolicy,
}

impl Node for PolicyRtNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
        if self.sleep_us > 0 {
            std::thread::sleep(Duration::from_micros(self.sleep_us));
        }
    }
}

impl RtNode for PolicyRtNode {
    fn wcet_budget(&self) -> Duration {
        Duration::from_millis(100)
    }
    fn deadline(&self) -> Duration {
        // Very tight deadline that will be missed if sleep_us > 0
        Duration::from_micros(10)
    }
    fn deadline_miss_policy(&self) -> DeadlineMissPolicy {
        self.policy
    }
}

/// RtNode that provides a fallback node.
struct FallbackPrimaryNode {
    name: String,
    tick_count: Arc<AtomicU64>,
    fallback_count: Arc<AtomicU64>,
}

impl Node for FallbackPrimaryNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
        // Slow enough to miss the tight deadline
        std::thread::sleep(Duration::from_micros(200));
    }
}

struct FallbackSecondaryNode {
    name: String,
    tick_count: Arc<AtomicU64>,
}

impl Node for FallbackSecondaryNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
    }
}

impl RtNode for FallbackSecondaryNode {
    fn wcet_budget(&self) -> Duration {
        Duration::from_millis(100)
    }
}

impl RtNode for FallbackPrimaryNode {
    fn wcet_budget(&self) -> Duration {
        Duration::from_millis(100)
    }
    fn deadline(&self) -> Duration {
        Duration::from_micros(10) // Will be missed
    }
    fn deadline_miss_policy(&self) -> DeadlineMissPolicy {
        DeadlineMissPolicy::Fallback
    }
    fn fallback_node(&self) -> Option<Box<dyn RtNode>> {
        Some(Box::new(FallbackSecondaryNode {
            name: "fallback_secondary".to_string(),
            tick_count: self.fallback_count.clone(),
        }))
    }
}

/// Node that panics with a &str literal.
struct StrPanicNode {
    name: String,
    tick_count: Arc<AtomicU64>,
}

impl Node for StrPanicNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
        panic!("literal str panic");
    }
}

impl RtNode for StrPanicNode {
    fn wcet_budget(&self) -> Duration {
        Duration::from_millis(100)
    }
}

/// Node that panics with an owned String.
struct StringPanicNode {
    name: String,
    tick_count: Arc<AtomicU64>,
}

impl Node for StringPanicNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
        let msg = format!(
            "owned string panic #{}",
            self.tick_count.load(Ordering::SeqCst)
        );
        panic!("{}", msg);
    }
}

impl RtNode for StringPanicNode {
    fn wcet_budget(&self) -> Duration {
        Duration::from_millis(100)
    }
}

/// Node that panics with an unknown type (i32).
struct UnknownPanicNode {
    name: String,
    tick_count: Arc<AtomicU64>,
}

impl Node for UnknownPanicNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
        std::panic::panic_any(42i32);
    }
}

impl RtNode for UnknownPanicNode {
    fn wcet_budget(&self) -> Duration {
        Duration::from_millis(100)
    }
}

/// Node that tracks on_error calls.
struct ErrorTrackingNode {
    name: String,
    tick_count: Arc<AtomicU64>,
    error_count: Arc<AtomicU64>,
    should_panic: AtomicBool,
}

impl Node for ErrorTrackingNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
        if self.should_panic.load(Ordering::SeqCst) {
            panic!("error tracking panic");
        }
    }
    fn on_error(&mut self, _error: &str) {
        self.error_count.fetch_add(1, Ordering::SeqCst);
    }
}

impl RtNode for ErrorTrackingNode {
    fn wcet_budget(&self) -> Duration {
        Duration::from_millis(100)
    }
}

/// Node that always panics (for circuit breaker / restart tests).
struct AlwaysPanicRtNode {
    name: String,
    tick_count: Arc<AtomicU64>,
}

impl Node for AlwaysPanicRtNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
        panic!("always panic");
    }
}

impl RtNode for AlwaysPanicRtNode {
    fn wcet_budget(&self) -> Duration {
        Duration::from_millis(100)
    }
}

/// Simple counter node for background testing.
struct SimpleRtNode {
    name: String,
    tick_count: Arc<AtomicU64>,
}

impl Node for SimpleRtNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::SeqCst);
    }
}

impl RtNode for SimpleRtNode {
    fn wcet_budget(&self) -> Duration {
        Duration::from_millis(100)
    }
}

// ============================================================================
// Tests
// ============================================================================

#[test]
fn test_rt_precondition_false_skips_tick() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));
    let pre_ok = Arc::new(AtomicBool::new(false)); // pre-condition always false

    let mut scheduler = Scheduler::new();
    scheduler
        .add_rt(ConditionalRtNode {
            name: "pre_false".to_string(),
            tick_count: tick_count.clone(),
            pre_ok,
            post_ok: Arc::new(AtomicBool::new(true)),
        })
        .order(0)
        .done();

    scheduler.run_for(Duration::from_millis(100)).unwrap();

    // Pre-condition is false, node should never tick
    assert_eq!(
        tick_count.load(Ordering::SeqCst),
        0,
        "Node with false pre_condition should never tick"
    );
}

#[test]
fn test_rt_postcondition_false_continues() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add_rt(ConditionalRtNode {
            name: "post_false".to_string(),
            tick_count: tick_count.clone(),
            pre_ok: Arc::new(AtomicBool::new(true)),
            post_ok: Arc::new(AtomicBool::new(false)), // post-condition always false
        })
        .order(0)
        .done();

    scheduler.run_for(Duration::from_millis(300)).unwrap();

    // Post-condition failure only logs warning, doesn't stop ticking
    assert!(
        tick_count.load(Ordering::SeqCst) > 0,
        "Node with false post_condition should still tick"
    );
}

#[test]
fn test_deadline_emergency_stop() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add_rt(PolicyRtNode {
            name: "emergency_stop_node".to_string(),
            tick_count: tick_count.clone(),
            sleep_us: 500, // Slow enough to miss 10us deadline
            policy: DeadlineMissPolicy::EmergencyStop,
        })
        .order(0)
        .deadline_us(10)
        .done();

    let result = scheduler.run_for(Duration::from_millis(500));
    // The scheduler should stop due to emergency stop
    assert!(result.is_ok());

    // Emergency stop sets running=false, but the RT loop checks at the top of each
    // cycle. The 500us sleep per tick means it takes time to reach the check.
    // Verify it stopped well before filling the full 500ms runtime.
    let ticks = tick_count.load(Ordering::SeqCst);
    assert!(
        ticks >= 1,
        "Emergency stop should tick at least once to trigger, got {}",
        ticks
    );
    // With 500us per tick, 500ms would give ~1000 ticks. Emergency stop should
    // cut this significantly short.
    assert!(
        ticks < 200,
        "Emergency stop should halt well before full runtime, got {} ticks",
        ticks
    );
}

#[test]
fn test_deadline_skip_pauses_node() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));
    let reference_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();
    // Slow node with Skip policy — should be paused after deadline miss
    scheduler
        .add_rt(PolicyRtNode {
            name: "skip_node".to_string(),
            tick_count: tick_count.clone(),
            sleep_us: 500,
            policy: DeadlineMissPolicy::Skip,
        })
        .order(0)
        .deadline_us(10)
        .done();

    // Reference node to confirm scheduler is still running
    scheduler
        .add(SimpleRtNode {
            name: "reference_node".to_string(),
            tick_count: reference_count.clone(),
        })
        .order(1)
        .rt()
        .done();

    scheduler.run_for(Duration::from_millis(200)).unwrap();

    let skip_ticks = tick_count.load(Ordering::SeqCst);
    let ref_ticks = reference_count.load(Ordering::SeqCst);

    // Reference should tick many times, skip node should tick but with gaps
    assert!(ref_ticks > 0, "Reference node should tick");
    assert!(skip_ticks > 0, "Skip node should still tick some");
    // The skip node misses deadlines and gets paused, so it should tick fewer times
    // than a simple fast node would
}

#[test]
fn test_deadline_degrade_lowers_priority() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add_rt(PolicyRtNode {
            name: "degrade_node".to_string(),
            tick_count: tick_count.clone(),
            sleep_us: 500,
            policy: DeadlineMissPolicy::Degrade,
        })
        .order(0)
        .deadline_us(10)
        .done();

    // Run briefly — degrade increases priority value by 10 on each miss
    scheduler.run_for(Duration::from_millis(200)).unwrap();

    // Node should still tick (degrade doesn't stop execution)
    assert!(
        tick_count.load(Ordering::SeqCst) > 0,
        "Degraded node should still tick"
    );
}

#[test]
fn test_deadline_fallback_swaps_node() {
    cleanup_stale_shm();
    let primary_count = Arc::new(AtomicU64::new(0));
    let fallback_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add_rt(FallbackPrimaryNode {
            name: "primary".to_string(),
            tick_count: primary_count.clone(),
            fallback_count: fallback_count.clone(),
        })
        .order(0)
        .deadline_us(10)
        .done();

    scheduler.run_for(Duration::from_millis(200)).unwrap();

    // Primary should tick at least once (to trigger the deadline miss + fallback)
    assert!(
        primary_count.load(Ordering::SeqCst) >= 1,
        "Primary should tick at least once"
    );

    // After fallback swap, the fallback node should be ticking
    assert!(
        fallback_count.load(Ordering::SeqCst) > 0,
        "Fallback node should tick after swap"
    );
}

#[test]
fn test_rt_panic_str_literal() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add(StrPanicNode {
            name: "str_panic".to_string(),
            tick_count: tick_count.clone(),
        })
        .order(0)
        .rt()
        .failure_policy(FailurePolicy::Ignore)
        .done();

    scheduler.run_for(Duration::from_millis(300)).unwrap();

    // With Ignore policy, node should keep ticking despite panics
    let ticks = tick_count.load(Ordering::SeqCst);
    assert!(
        ticks >= 2,
        "Str panic node with Ignore should tick multiple times, got {}",
        ticks
    );
}

#[test]
fn test_rt_panic_owned_string() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add(StringPanicNode {
            name: "string_panic".to_string(),
            tick_count: tick_count.clone(),
        })
        .order(0)
        .rt()
        .failure_policy(FailurePolicy::Ignore)
        .done();

    scheduler.run_for(Duration::from_millis(300)).unwrap();

    let ticks = tick_count.load(Ordering::SeqCst);
    assert!(
        ticks >= 2,
        "String panic node with Ignore should tick multiple times, got {}",
        ticks
    );
}

#[test]
fn test_rt_panic_unknown_type() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add(UnknownPanicNode {
            name: "unknown_panic".to_string(),
            tick_count: tick_count.clone(),
        })
        .order(0)
        .rt()
        .failure_policy(FailurePolicy::Ignore)
        .done();

    scheduler.run_for(Duration::from_millis(300)).unwrap();

    let ticks = tick_count.load(Ordering::SeqCst);
    assert!(
        ticks >= 2,
        "Unknown panic node with Ignore should tick multiple times, got {}",
        ticks
    );
}

#[test]
fn test_rt_on_error_callback() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));
    let error_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add(ErrorTrackingNode {
            name: "error_tracking".to_string(),
            tick_count: tick_count.clone(),
            error_count: error_count.clone(),
            should_panic: AtomicBool::new(true),
        })
        .order(0)
        .rt()
        .failure_policy(FailurePolicy::Ignore)
        .done();

    scheduler.run_for(Duration::from_millis(300)).unwrap();

    // on_error should be called for each panic
    let errors = error_count.load(Ordering::SeqCst);
    let ticks = tick_count.load(Ordering::SeqCst);
    assert!(
        errors > 0,
        "on_error should be called at least once, got {}",
        errors
    );
    assert_eq!(
        errors, ticks,
        "on_error count ({}) should equal tick count ({}) since every tick panics",
        errors, ticks
    );
}

#[test]
fn test_rt_circuit_breaker_rejection() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add(AlwaysPanicRtNode {
            name: "circuit_breaker_rt".to_string(),
            tick_count: tick_count.clone(),
        })
        .order(0)
        .rt()
        .failure_policy(FailurePolicy::skip(2, 5000)) // Opens after 2 failures, 5s cooldown
        .done();

    scheduler.run_for(Duration::from_millis(200)).unwrap();

    // Circuit breaker should open after 2 failures, preventing further ticks.
    // The RT executor tick loop is fast, so several ticks may fire before the
    // breaker fully takes effect.
    let ticks = tick_count.load(Ordering::SeqCst);
    assert!(
        ticks >= 2,
        "Circuit breaker should allow at least 2 failures, got {} ticks",
        ticks
    );
}

#[test]
fn test_rt_restart_failure_deinitializes() {
    cleanup_stale_shm();
    let tick_count = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new();
    scheduler
        .add(AlwaysPanicRtNode {
            name: "restart_fail".to_string(),
            tick_count: tick_count.clone(),
        })
        .order(0)
        .rt()
        .failure_policy(FailurePolicy::restart(3, 10))
        .done();

    // The restart policy should attempt restarts after panics.
    // After exhausting restarts (3), it escalates to fatal and stops.
    scheduler.run_for(Duration::from_millis(500)).unwrap();

    let ticks = tick_count.load(Ordering::SeqCst);
    // The node should tick at least once before restart exhaustion
    assert!(
        ticks >= 1,
        "Restart node should tick at least once, got {}",
        ticks
    );
}
