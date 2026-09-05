//! The flight recorder must not serialize JSON and write files on a tick thread.
//!
//! `BlackBox::record` runs `serde_json::to_string`, a `BufWriter` write, and a
//! `flush()` — a `write(2)` — every 64 records, on whichever thread calls it.
//! Every tick-path caller was inside the loop: the RT threads from `tick_node`
//! for budget violations, deadline misses and node errors; the main tick thread
//! for the same plus a **blocking** `lock()` once per loop iteration just to
//! bump a counter; and the compute, event and async workers for node errors.
//!
//! Two things made it worse than the write itself. The write is unthrottled —
//! `DiagThrottle` caps the console line at one per second, but the blackbox
//! record sits outside that gate and fires on every miss, so a 1 kHz node in
//! sustained overload does a thousand serialise-and-write cycles per second on
//! its RT thread. And the loss was silent: every executor site was
//! `if let Ok(mut bb) = bb.try_lock()` with no `else`, so whenever the main
//! thread held the lock — which it did every iteration — the record was dropped
//! and nothing counted it.
//!
//! # What these assert
//!
//! That the events still arrive, and that the executors can no longer reach the
//! recorder at all. The second is now a type-level property —
//! `SharedMonitors::blackbox` is the producer ring — so the strongest test of
//! it is that the events show up having gone through the ring.

use horus_core::core::{DurationExt, Node};
use horus_core::error::Result;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

/// Overruns its budget on every tick, so the deadline/budget machinery fires.
struct Slowpoke {
    ticks: Arc<AtomicU64>,
}

impl Node for Slowpoke {
    fn name(&self) -> &'static str {
        "slowpoke"
    }
    fn init(&mut self) -> Result<()> {
        Ok(())
    }
    fn tick(&mut self) {
        self.ticks.fetch_add(1, Ordering::Relaxed);
        // Comfortably over the budget below, and comfortably inside the
        // period, so the node keeps ticking rather than being enforced away
        // before it has produced anything to record.
        std::thread::sleep(std::time::Duration::from_micros(700));
    }
}

/// Events queued from the tick path must still reach the recorder.
///
/// The ring is only worth having if nothing is lost by it: a flight recorder
/// that drops the events describing a fault is worse than a slow one.
#[test]
fn events_queued_from_the_tick_path_reach_the_recorder() {
    let ticks = Arc::new(AtomicU64::new(0));
    let mut sched = Scheduler::new().tick_rate(200_u64.hz()).blackbox(1);
    sched
        .add(Slowpoke {
            ticks: Arc::clone(&ticks),
        })
        .rate(200_u64.hz())
        .budget(300_u64.us())
        .build()
        .unwrap();
    sched.run_for(400_u64.ms()).unwrap();

    let n = ticks.load(Ordering::Relaxed);
    assert!(
        n > 5,
        "the node must actually have ticked, or there is nothing to record (saw {n})"
    );

    let bb = sched
        .get_blackbox()
        .expect(".blackbox(1) must install a recorder")
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    let events = bb.events();
    assert!(
        !events.is_empty(),
        "the recorder is empty: every budget violation the node produced was \
         queued on the tick thread and none of it was drained"
    );
    assert!(
        events.iter().any(|r| matches!(
            r.event,
            horus_core::scheduling::BlackBoxEvent::BudgetViolation { .. }
        )),
        "a node that overran its budget on every tick must have produced a \
         BudgetViolation record; got {:?}",
        events
            .iter()
            .map(|r| format!("{:?}", r.event))
            .collect::<Vec<_>>()
    );
}

/// The stop marker must be the last thing in the log.
///
/// The drain has to run BEFORE `SchedulerStop` is written, or the events
/// describing why a run ended land after the record that it ended, which is
/// exactly the ordering a post-mortem cannot use.
#[test]
fn the_stop_marker_lands_after_everything_the_run_queued() {
    let ticks = Arc::new(AtomicU64::new(0));
    let mut sched = Scheduler::new().tick_rate(200_u64.hz()).blackbox(1);
    sched
        .add(Slowpoke {
            ticks: Arc::clone(&ticks),
        })
        .rate(200_u64.hz())
        .budget(300_u64.us())
        .build()
        .unwrap();
    sched.run_for(400_u64.ms()).unwrap();

    let bb = sched
        .get_blackbox()
        .expect("recorder")
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    let events = bb.events();
    let stop_at = events.iter().position(|r| {
        matches!(
            r.event,
            horus_core::scheduling::BlackBoxEvent::SchedulerStop { .. }
        )
    });
    let last_violation = events.iter().rposition(|r| {
        matches!(
            r.event,
            horus_core::scheduling::BlackBoxEvent::BudgetViolation { .. }
        )
    });

    // Both must be present, or this test would pass by finding neither — the
    // vacuous-green failure mode that makes an ordering guard worthless.
    let stop = stop_at.expect("a completed run must have written a SchedulerStop marker");
    let violation = last_violation.expect(
        "the node overran its budget on every tick, so at least one BudgetViolation \
         must have been queued and drained",
    );
    assert!(
        violation < stop,
        "a queued event landed AFTER the stop marker (violation at {violation}, stop \
         at {stop}): the drain did not run before the marker was written"
    );
}
