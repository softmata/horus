#![allow(dead_code)]
//! STRONG enforcement tests for `FailurePolicy` — these distinguish an ENFORCED
//! policy from the old unenforced (log-and-continue) behavior, and verify that a
//! `Fatal` stop actually SAFES the hardware (not just stops ticking).
//!
//! Contrast with `wiring_verification.rs`, whose failure-policy tests are weak
//! (e.g. assert `!is_running()` after `run_for`, which is always true). These
//! assert the discriminating facts: stopped EARLY, the faulted node was
//! `enter_safe_state()`-d, other nodes were `shutdown()`-d, Skip strongly
//! suppresses, and Restart re-runs `init()`.

use horus_core::core::{DurationExt, Node};
use horus_core::error::Result;
use horus_core::scheduling::{FailurePolicy, Scheduler};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;

// A node that panics once it has ticked `panic_at` times, and records whether
// its `enter_safe_state()` was invoked (a Fatal fault must safe it).
struct FaultyNode {
    name: &'static str,
    count: Arc<AtomicU32>,
    panic_at: u32,
    safe_stated: Arc<AtomicU32>,
}
impl Node for FaultyNode {
    fn name(&self) -> &str {
        self.name
    }
    fn tick(&mut self) {
        let c = self.count.fetch_add(1, Ordering::SeqCst);
        if c >= self.panic_at {
            panic!("intentional panic at tick {c}");
        }
    }
    fn enter_safe_state(&mut self) {
        self.safe_stated.fetch_add(1, Ordering::SeqCst);
    }
}

// A healthy "victim" node: counts its ticks and records shutdown() — used to
// prove the scheduler stopped EARLY and that other nodes were safed on stop.
struct VictimNode {
    name: &'static str,
    ticks: Arc<AtomicU32>,
    shutdown_calls: Arc<AtomicU32>,
}
impl Node for VictimNode {
    fn name(&self) -> &str {
        self.name
    }
    fn tick(&mut self) {
        self.ticks.fetch_add(1, Ordering::SeqCst);
    }
    fn shutdown(&mut self) -> Result<()> {
        self.shutdown_calls.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
}

// A node that always panics, counting ticks — used to prove Skip/backoff
// strongly suppresses (few ticks) vs unenforced (ticks every cycle).
struct AlwaysPanic {
    name: &'static str,
    count: Arc<AtomicU32>,
}
impl Node for AlwaysPanic {
    fn name(&self) -> &str {
        self.name
    }
    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::SeqCst);
        panic!("always fails");
    }
}

// A node that fails a few times then would succeed, counting init() calls — used
// to prove Restart re-runs init().
struct RestartableNode {
    name: &'static str,
    inits: Arc<AtomicU32>,
    ticks: Arc<AtomicU32>,
    fail_first: u32,
}
impl Node for RestartableNode {
    fn name(&self) -> &str {
        self.name
    }
    fn init(&mut self) -> Result<()> {
        self.inits.fetch_add(1, Ordering::SeqCst);
        Ok(())
    }
    fn tick(&mut self) {
        let c = self.ticks.fetch_add(1, Ordering::SeqCst);
        if c < self.fail_first {
            panic!("transient failure {c}");
        }
    }
}

// ── Fatal (BestEffort main-thread path) ─────────────────────────────────────
#[test]
fn fatal_stops_early_and_safes_other_nodes() {
    let faulty_ticks = Arc::new(AtomicU32::new(0));
    let faulty_safe = Arc::new(AtomicU32::new(0));
    let victim_ticks = Arc::new(AtomicU32::new(0));
    let victim_shutdown = Arc::new(AtomicU32::new(0));

    let mut sched = Scheduler::new().tick_rate(500_u64.hz());
    sched
        .add(FaultyNode {
            name: "faulty",
            count: faulty_ticks.clone(),
            panic_at: 3,
            safe_stated: faulty_safe.clone(),
        })
        .order(0)
        .failure_policy(FailurePolicy::Fatal)
        .build()
        .unwrap();
    sched
        .add(VictimNode {
            name: "victim",
            ticks: victim_ticks.clone(),
            shutdown_calls: victim_shutdown.clone(),
        })
        .order(1)
        .build()
        .unwrap();

    // Fatal should stop the scheduler ~3 ticks in. Without enforcement the
    // victim ticks for the full 2s (~1000 ticks at 500 Hz).
    sched.run_for(2_u64.secs()).unwrap();

    let vt = victim_ticks.load(Ordering::SeqCst);
    assert!(
        vt < 50,
        "Fatal must stop the scheduler EARLY — victim ticked {vt} (expected a handful; \
         ~1000 means the policy was not enforced)"
    );
    assert!(
        faulty_safe.load(Ordering::SeqCst) >= 1,
        "Fatal fault must safe the faulted node (enter_safe_state was not called)"
    );
    assert!(
        victim_shutdown.load(Ordering::SeqCst) >= 1,
        "A Fatal stop must safe OTHER nodes too — victim.shutdown() was not called"
    );
    assert!(!sched.is_running());
}

// ── Skip (strong suppression) ───────────────────────────────────────────────
#[test]
fn skip_strongly_suppresses_after_threshold() {
    let ticks = Arc::new(AtomicU32::new(0));
    let mut sched = Scheduler::new().tick_rate(200_u64.hz());
    sched
        .add(AlwaysPanic {
            name: "always_panic",
            count: ticks.clone(),
        })
        .order(0)
        .failure_policy(FailurePolicy::skip(3, 100_u64.ms()))
        .build()
        .unwrap();

    sched.run_for(500_u64.ms()).unwrap();

    // skip(3, 100ms): 3 failures then a 100ms cooldown, repeated. Over 500ms at
    // 200 Hz that is ~3 failures/cycle × ~5 cycles ≈ 15-25 ticks. Unenforced it
    // ticks every cycle (~90-100). Assert STRONG suppression, not just "< 100".
    let t = ticks.load(Ordering::SeqCst);
    assert!(
        t < 45,
        "Skip must strongly suppress the panicking node — got {t} ticks (unenforced ≈ 90+)"
    );
}

// ── Restart (re-runs init) ──────────────────────────────────────────────────
#[test]
fn restart_reinitializes_the_node() {
    let inits = Arc::new(AtomicU32::new(0));
    let ticks = Arc::new(AtomicU32::new(0));
    let mut sched = Scheduler::new().tick_rate(200_u64.hz());
    sched
        .add(RestartableNode {
            name: "restartable",
            inits: inits.clone(),
            ticks: ticks.clone(),
            fail_first: 2,
        })
        .order(0)
        .failure_policy(FailurePolicy::restart(5, 5_u64.ms()))
        .build()
        .unwrap();

    sched.run_for(400_u64.ms()).unwrap();

    // init() runs once at startup; each Restart re-runs it. With 2 transient
    // failures, expect at least 2 additional init() calls (>= 3 total). Without
    // enforcement init() is only ever called once.
    let n = inits.load(Ordering::SeqCst);
    assert!(
        n >= 3,
        "Restart must re-run init() after each failure — init() called {n}× (expected >= 3; \
         1 means Restart was not enforced)"
    );
}

// ── Fatal on the RT executor (control-loop path — runs on its own thread) ────
#[test]
fn fatal_on_rt_executor_stops_early_and_safes() {
    let faulty_ticks = Arc::new(AtomicU32::new(0));
    let faulty_safe = Arc::new(AtomicU32::new(0));
    let victim_ticks = Arc::new(AtomicU32::new(0));
    let victim_shutdown = Arc::new(AtomicU32::new(0));

    let mut sched = Scheduler::new().tick_rate(500_u64.hz());
    // `.rate()` classifies these as RT nodes → they run in the RtExecutor.
    sched
        .add(FaultyNode {
            name: "rt_faulty",
            count: faulty_ticks.clone(),
            panic_at: 3,
            safe_stated: faulty_safe.clone(),
        })
        .rate(500_u64.hz())
        .failure_policy(FailurePolicy::Fatal)
        .build()
        .unwrap();
    sched
        .add(VictimNode {
            name: "rt_victim",
            ticks: victim_ticks.clone(),
            shutdown_calls: victim_shutdown.clone(),
        })
        .rate(500_u64.hz())
        .build()
        .unwrap();

    sched.run_for(2_u64.secs()).unwrap();

    let vt = victim_ticks.load(Ordering::SeqCst);
    assert!(
        vt < 80,
        "Fatal on an RT node must stop the scheduler early — victim ticked {vt} (~1000 = unenforced)"
    );
    assert!(
        faulty_safe.load(Ordering::SeqCst) >= 1,
        "Fatal fault on the RT thread must safe the faulted node (enter_safe_state)"
    );
    assert!(
        victim_shutdown.load(Ordering::SeqCst) >= 1,
        "A Fatal stop must safe OTHER nodes too (victim.shutdown())"
    );
    assert!(!sched.is_running());
}

// ── Fatal on the Compute executor (parallel thread pool) ────────────────────
#[test]
fn fatal_on_compute_executor_stops_early() {
    let faulty_safe = Arc::new(AtomicU32::new(0));
    let victim_ticks = Arc::new(AtomicU32::new(0));
    let victim_shutdown = Arc::new(AtomicU32::new(0));

    let mut sched = Scheduler::new().tick_rate(500_u64.hz());
    sched
        .add(FaultyNode {
            name: "cpu_faulty",
            count: Arc::new(AtomicU32::new(0)),
            panic_at: 3,
            safe_stated: faulty_safe.clone(),
        })
        .compute()
        .failure_policy(FailurePolicy::Fatal)
        .build()
        .unwrap();
    sched
        .add(VictimNode {
            name: "cpu_victim",
            ticks: victim_ticks.clone(),
            shutdown_calls: victim_shutdown.clone(),
        })
        .compute()
        .build()
        .unwrap();

    sched.run_for(2_u64.secs()).unwrap();

    let vt = victim_ticks.load(Ordering::SeqCst);
    assert!(
        vt < 200,
        "Fatal on a Compute node must stop the scheduler early — victim ticked {vt}"
    );
    assert!(
        faulty_safe.load(Ordering::SeqCst) >= 1,
        "Fatal fault on the Compute thread must safe the faulted node"
    );
    assert!(!sched.is_running());
}
