//! Whether real-time actually applied is a question you can now ask.
//!
//! Before `rt_status`, the outcome of each RT tick thread's `sched_setscheduler`
//! call lived in a local `bool` that reached one consumer and was dropped. A
//! node running SCHED_FIFO and a node running SCHED_OTHER were identical to
//! every query, stat and report in the crate — and `require_rt()` inspected
//! only the *builder* thread's priority-50 request, so it could return success
//! while every tick thread ran at normal priority.
//!
//! These tests pin both halves: the status is observable, and it is enforced.
//!
//! # On environment dependence
//!
//! Whether SCHED_FIFO is granted depends on `CAP_SYS_NICE` / `RLIMIT_RTPRIO`,
//! which differ between a developer machine, CI, and a tuned robot. A test that
//! hardcoded either outcome would be wrong half the time. These assert the
//! *invariants that must hold either way* — chiefly that the reported policy
//! and the reported reason cannot disagree, and that `require_rt()` succeeds if
//! and only if the tick threads really got a real-time policy.

use horus_core::core::{DurationExt, Node};
use horus_core::error::Result;
use horus_core::scheduling::rt_status::{RtPolicy, RtThreadReport, RtThreadStatus};
use horus_core::scheduling::{Scheduler, SchedulerConfig};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

mod common;
use common::cleanup_stale_shm;

struct Ticker {
    name: &'static str,
    ticks: Arc<AtomicU64>,
}

impl Node for Ticker {
    fn name(&self) -> &'static str {
        self.name
    }
    fn init(&mut self) -> Result<()> {
        Ok(())
    }
    fn tick(&mut self) {
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

fn status_for(name: &'static str, hz: u64) -> (Vec<RtThreadStatus>, u64) {
    let ticks = Arc::new(AtomicU64::new(0));
    let mut sched = Scheduler::new().tick_rate(1000_u64.hz());
    sched
        .add(Ticker {
            name,
            ticks: Arc::clone(&ticks),
        })
        .rate(hz.hz())
        .build()
        .unwrap();
    sched.run_for(120_u64.ms()).unwrap();
    (sched.rt_thread_status(), ticks.load(Ordering::Relaxed))
}

// ---------------------------------------------------------------------------
// The report type itself. Deterministic — no scheduler, no kernel.
// ---------------------------------------------------------------------------

fn st(name: &str, requested: RtPolicy, granted: RtPolicy, locked: bool) -> RtThreadStatus {
    RtThreadStatus {
        thread_name: name.to_string(),
        chain_label: format!("{name}-chain"),
        requested,
        granted,
        priority: if granted.is_realtime() { 80 } else { 0 },
        refusal: if requested.is_realtime() && !granted.is_realtime() {
            Some("EPERM".to_string())
        } else {
            None
        },
        cpus: vec![],
        memory_locked: locked,
    }
}

#[test]
fn a_refused_thread_is_reported_as_degraded() {
    let report = RtThreadReport::new(2);
    report.publish(st("ok", RtPolicy::Fifo, RtPolicy::Fifo, true));
    report.publish(st("refused", RtPolicy::Fifo, RtPolicy::Other, true));

    let degraded = report.degraded();
    assert_eq!(degraded.len(), 1, "exactly one thread was refused");
    assert_eq!(degraded[0].thread_name, "refused");
    assert!(!report.all_realtime());
}

#[test]
fn realtime_on_unlocked_memory_is_reported_but_not_degraded() {
    let report = RtThreadReport::new(1);
    report.publish(st("fifo_unlocked", RtPolicy::Fifo, RtPolicy::Fifo, false));

    assert!(
        report.degraded().is_empty(),
        "the thread got its policy — this is not a refusal"
    );
    assert_eq!(
        report.realtime_without_locked_memory().len(),
        1,
        "but running RT on unlocked memory must still be visible"
    );
    assert!(report.all_realtime());
}

#[test]
fn a_non_realtime_thread_on_unlocked_memory_is_not_flagged_for_memory() {
    // The mlockall warning is about RT threads specifically. A SCHED_OTHER
    // thread on unlocked memory is just an ordinary thread, and flagging it
    // would bury the case that matters in noise.
    let report = RtThreadReport::new(1);
    report.publish(st("plain", RtPolicy::Fifo, RtPolicy::Other, false));
    assert!(report.realtime_without_locked_memory().is_empty());
}

#[test]
fn wait_for_all_returns_false_when_a_thread_never_reports() {
    let report = RtThreadReport::new(2);
    report.publish(st("only_one", RtPolicy::Fifo, RtPolicy::Fifo, true));
    assert!(
        !report.wait_for_all(std::time::Duration::from_millis(50)),
        "one of two threads reported — this must not claim completeness"
    );
    assert_eq!(
        report.statuses().len(),
        1,
        "the partial report is still readable"
    );
}

#[test]
fn wait_for_all_returns_true_once_every_thread_reports() {
    let report = RtThreadReport::new(2);
    let bg = Arc::clone(&report);
    let h = std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_millis(20));
        bg.publish(st("late", RtPolicy::Fifo, RtPolicy::Fifo, true));
    });
    report.publish(st("early", RtPolicy::Fifo, RtPolicy::Fifo, true));
    assert!(
        report.wait_for_all(std::time::Duration::from_secs(5)),
        "both threads reported well within the timeout"
    );
    h.join().unwrap();
}

// ---------------------------------------------------------------------------
// Against a real scheduler.
// ---------------------------------------------------------------------------

#[test]
fn an_rt_node_produces_an_observable_thread_status() {
    let _shm = cleanup_stale_shm();
    let (statuses, ticks) = status_for("observable", 200);

    assert_eq!(
        statuses.len(),
        1,
        "one RT node means one RT thread, and it must report"
    );
    assert!(ticks > 0, "the node must actually have ticked");

    let s = &statuses[0];
    assert!(
        s.thread_name.starts_with("horus-rt"),
        "reported under its OS thread name, got {:?}",
        s.thread_name
    );
    assert_eq!(
        s.chain_label, "observable",
        "the chain is named by its node so this can be matched to a config"
    );
}

#[test]
fn a_scheduler_with_no_rt_nodes_reports_no_rt_threads() {
    let _shm = cleanup_stale_shm();
    let ticks = Arc::new(AtomicU64::new(0));
    let mut sched = Scheduler::new().tick_rate(200_u64.hz());
    sched
        .add(Ticker {
            name: "besteffort",
            ticks: Arc::clone(&ticks),
        })
        .build()
        .unwrap();
    sched.run_for(60_u64.ms()).unwrap();

    assert!(
        sched.rt_thread_status().is_empty(),
        "no .rate() means no RT executor, so there is nothing to report"
    );
    assert!(
        !sched.rt_threads_are_realtime(),
        "'no RT threads' must not read as 'RT is confirmed working'"
    );
}

/// The invariant that the old code could not express.
///
/// Whatever this machine grants, the report may not be self-contradictory: a
/// thread is either running real-time with a priority and no complaint, or it
/// is not running real-time and says why. The failure this pins is the third
/// state the runtime used to be in — not real-time, and silent about it.
#[test]
fn the_reported_policy_and_the_reported_reason_agree() {
    let _shm = cleanup_stale_shm();
    let (statuses, _) = status_for("consistent", 500);
    assert_eq!(statuses.len(), 1);
    let s = &statuses[0];

    if s.granted.is_realtime() {
        // Only SCHED_FIFO carries a static priority; a SCHED_DEADLINE thread
        // is admitted with a budget and reports priority 0.
        if s.granted == RtPolicy::Fifo {
            assert!(
                s.priority > 0,
                "a SCHED_FIFO thread must carry a real priority, got {}",
                s.priority
            );
        }
        assert!(
            s.refusal.is_none(),
            "the kernel granted the policy, so there is nothing to explain: {:?}",
            s.refusal
        );
        assert!(!s.is_degraded());
    } else {
        assert!(
            s.refusal.is_some(),
            "the thread asked for {} and did not get it — that must be explained, \
             not left silent. This is the exact gap rt_status was added to close.",
            s.requested.as_str()
        );
        assert_eq!(s.priority, 0, "no RT policy means no RT priority");
        assert!(s.is_degraded());
    }
}

/// `require_rt()` must track what the TICK threads got.
///
/// It used to inspect only the builder thread's priority-50 request. This
/// asserts the biconditional: the run succeeds exactly when the tick threads
/// really are real-time, so neither a false pass nor a false failure survives.
#[test]
fn require_rt_succeeds_exactly_when_the_tick_threads_are_realtime() {
    let _shm = cleanup_stale_shm();

    // `require_rt()` asserts on capability probing before any of this, and
    // panics on a machine it judges hopeless. That is pre-existing behaviour;
    // catch it so this test reports the interesting case rather than dying.
    let built = std::panic::catch_unwind(|| Scheduler::new().require_rt().tick_rate(1000_u64.hz()));
    let Ok(base) = built else {
        eprintln!("require_rt() refused at build time on this host — nothing to check");
        return;
    };

    let ticks = Arc::new(AtomicU64::new(0));
    let mut sched = base;
    sched
        .add(Ticker {
            name: "required",
            ticks: Arc::clone(&ticks),
        })
        .rate(500_u64.hz())
        .build()
        .unwrap();

    let result = sched.run_for(120_u64.ms());
    let statuses = sched.rt_thread_status();

    match result {
        Ok(()) => {
            assert!(
                !statuses.is_empty(),
                "require_rt() returned success, so there must be RT threads to vouch for it"
            );
            for s in &statuses {
                assert!(
                    s.granted.is_realtime(),
                    "require_rt() succeeded while a tick thread ran {}. This is the false \
                     pass the check was added to prevent: {}",
                    s.granted.as_str(),
                    s.summary()
                );
            }
        }
        Err(e) => {
            let msg = e.to_string();
            assert!(
                statuses.iter().any(|s| !s.granted.is_realtime()),
                "require_rt() failed but every reported tick thread had a real-time \
                 policy — the failure must name a real degradation. Error: {msg}"
            );
            assert!(
                msg.contains("real-time") || msg.contains("Real-time"),
                "the error should say what was missing, got: {msg}"
            );
        }
    }
}

/// The gap itself, reproduced.
///
/// `require_mode` is set through the configuration surface WITHOUT
/// `rt_scheduling_class`, so the thread that builds the scheduler never asks
/// the kernel for anything and records no degradation — `has_full_rt()` is
/// true and the pre-existing check passes cleanly. The RT tick thread still
/// asks for SCHED_FIFO, because `.rate()` alone puts a node on the RT
/// executor.
///
/// That is the exact shape of the hole: builder thread clean, tick thread
/// refused, `run()` returning `Ok(())` while nothing real-time was running.
/// On a host that grants RT the run legitimately succeeds, so the assertion
/// is tied to what the threads actually report rather than to an expected
/// verdict.
#[test]
fn require_mode_fails_when_only_the_tick_thread_is_refused() {
    let _shm = cleanup_stale_shm();

    let ticks = Arc::new(AtomicU64::new(0));
    let mut sched = Scheduler::new().tick_rate(1000_u64.hz());

    let mut cfg = SchedulerConfig::default();
    cfg.realtime.require_mode = true;
    // Deliberately left false. With either of these on, the builder thread
    // makes its own priority-50 request, that failure is recorded, and the
    // older check fires first — which would pass this test without the tick
    // thread ever being consulted.
    cfg.realtime.rt_scheduling_class = false;
    cfg.realtime.memory_locking = false;
    sched.apply_config(cfg);

    sched
        .add(Ticker {
            name: "tick_thread_only",
            ticks: Arc::clone(&ticks),
        })
        .rate(500_u64.hz())
        .build()
        .unwrap();

    let result = sched.run_for(120_u64.ms());
    let statuses = sched.rt_thread_status();
    assert_eq!(statuses.len(), 1, "one RT node, one RT thread, one status");
    let s = &statuses[0];

    if s.granted.is_realtime() {
        assert!(
            result.is_ok(),
            "the tick thread got {} — require_mode must not fail: {:?}",
            s.granted.as_str(),
            result.err()
        );
    } else {
        let err = result.expect_err(
            "the tick thread was refused a real-time policy while require_mode \
             was set. run() returned Ok — this is the false pass: the caller \
             believes RT applied and it did not.",
        );
        let msg = err.to_string();
        assert!(
            msg.contains("tick thread") || msg.contains("real-time"),
            "the error must name what was refused, got: {msg}"
        );
        assert!(
            s.refusal.is_some(),
            "and the status must carry the kernel's reason"
        );
    }
}
