//! `tick_once()` is HORUS's externally-driven step API — the integration point
//! for a fieldbus.
//!
//! A quadruped or humanoid on EtherCAT/CAN does, once per bus cycle:
//!
//! ```text
//!     ec_receive_processdata()  ->  sched.tick_once()  ->  ec_send_processdata()
//! ```
//!
//! The bus owns the clock. `tick_once()` documents exactly this contract: "Does
//! **not** loop. Does **not** sleep. The caller controls timing."
//!
//! For that to work, a node declared `.rate(1000hz)` must actually tick on every
//! 1 kHz bus cycle. It did not: the rate gate compared `elapsed < period` with a
//! strict `<` and no tolerance, so a cycle arriving a hair early — which is the
//! normal case, since a bus crystal is not the host's CLOCK_MONOTONIC and the
//! caller's own work sits inside the measured interval — was refused, and the
//! node fired every *other* cycle at half its declared rate.
//!
//! The RT executor already hit this and fixed it (rt_executor.rs, "halving the
//! effective rate of every RT node"). The externally-driven path never got the
//! same treatment.

use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

struct CycleNode {
    ticks: Arc<AtomicU64>,
}

impl Node for CycleNode {
    fn name(&self) -> &'static str {
        "bus_driven_node"
    }
    fn tick(&mut self) {
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

/// Drive the scheduler from a simulated 1 kHz bus and require that a
/// `.rate(1000hz)` node ticks on (nearly) every cycle.
#[test]
fn rate_node_ticks_once_per_external_cycle() {
    let ticks = Arc::new(AtomicU64::new(0));
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz()).verbose(false);
    let _ = scheduler
        .add(CycleNode {
            ticks: ticks.clone(),
        })
        .rate(1000_u64.hz())
        .build();

    const CYCLES: u64 = 400;
    const PERIOD: Duration = Duration::from_micros(1000);

    // A bus cycle arrives on the BUS's clock, not ours, and a bus master that
    // falls behind does not then deliver a burst of backlogged cycles — it
    // drops them and carries on from the present. Model that, because the naive
    // version (spin to `start + n*PERIOD` for every n) does the opposite: when
    // this thread is preempted it returns to find several targets already past
    // and fires them back to back with no spacing at all. Those bunched cycles
    // are correctly refused by the rate gate, and the test then measures host
    // load rather than the gate. Under load average ~43 it read 66/400 on code
    // that reads 399/400 on an idle box — a flaky gate, and a flaky gate in a
    // required check is worse than no gate.
    //
    // So: skip missed slots, and count only the cycles actually delivered near
    // their slot. `delivered` is the denominator the assertion uses, which makes
    // the ratio independent of how much of the run the host stole.
    let mut delivered: u64 = 0;
    let start = Instant::now();
    let mut target = start;
    for _ in 0..CYCLES {
        let now = Instant::now();
        if now > target + PERIOD {
            // Fell behind by at least a whole cycle: realign to the present
            // instead of replaying the backlog, exactly as a bus master would.
            let behind = now.duration_since(target).as_nanos() as u64;
            let skip = behind / PERIOD.as_nanos() as u64 + 1;
            target += PERIOD * (skip as u32);
            continue;
        }
        while Instant::now() < target {
            std::hint::spin_loop();
        }
        scheduler.tick_once().expect("tick_once");
        delivered += 1;
        target += PERIOD;
    }

    let observed = ticks.load(Ordering::Relaxed);
    eprintln!(
        "{} on-time bus cycles delivered (of {} attempted) -> {} node ticks",
        delivered, CYCLES, observed
    );

    assert!(
        delivered >= 50,
        "host delivered only {delivered} on-time cycles of {CYCLES}; too few to \
         conclude anything about the rate gate"
    );

    // The failure this pins is a systematic refusal of the boundary cycle: the
    // node sees ~50-60% of cycles, or worse. 90% of DELIVERED cycles leaves room
    // for the first cycle and for a stall inside an otherwise on-time run.
    let floor = (delivered as f64 * 0.90) as u64;
    assert!(
        observed >= floor,
        "a .rate(1000hz) node driven by a 1 kHz external cycle ticked only {} \
         times in {} on-time cycles (needed >= {}). The rate gate is refusing \
         cycles that arrive a hair early; the RT executor solved this with a \
         half-period tolerance and the externally-driven path needs the same.",
        observed,
        delivered,
        floor
    );
}

/// The tolerance must not let a SLOWER node run early.
///
/// A 100 Hz node driven by a 1 kHz cycle must still tick ~10ms apart, not on
/// every bus cycle. Widening the gate is only safe if it is bounded by the
/// driving period.
///
/// The property is the SPACING of the node's ticks, not how many it got. A
/// count is a function of how much of the run the host let us have — the same
/// trap the first test fell into — and it is also blunt: a tolerance scaled to
/// the NODE's half-period would admit at 5ms and produce exactly 80 ticks in
/// the nominal 0.4s, which the old `observed <= 80` bound waved through. Host
/// load can only ever push ticks further apart, never closer, so a floor on
/// the smallest observed gap tests the gate and nothing else.
#[test]
fn slower_node_is_not_accelerated_by_the_tolerance() {
    /// Records the gap between its own consecutive ticks.
    struct SpacingNode {
        ticks: Arc<AtomicU64>,
        min_gap_ns: Arc<AtomicU64>,
        last: Option<Instant>,
    }

    impl Node for SpacingNode {
        fn name(&self) -> &'static str {
            "spacing_node"
        }
        fn tick(&mut self) {
            let now = Instant::now();
            if let Some(prev) = self.last {
                let gap = now.duration_since(prev).as_nanos() as u64;
                self.min_gap_ns.fetch_min(gap, Ordering::Relaxed);
            }
            self.last = Some(now);
            self.ticks.fetch_add(1, Ordering::Relaxed);
        }
    }

    let ticks = Arc::new(AtomicU64::new(0));
    let min_gap_ns = Arc::new(AtomicU64::new(u64::MAX));
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz()).verbose(false);
    let _ = scheduler
        .add(SpacingNode {
            ticks: ticks.clone(),
            min_gap_ns: min_gap_ns.clone(),
            last: None,
        })
        .rate(100_u64.hz())
        .build();

    const CYCLES: u64 = 400;
    const PERIOD: Duration = Duration::from_micros(1000);

    // Same delivery model as the first test, for the same reason: a bus master
    // that falls behind drops the missed slots and carries on from the present.
    // Replaying the backlog instead (`target = start + n*PERIOD` for every n)
    // fires bunched calls after a preemption, which makes the tick count a
    // measure of host scheduling rather than of the gate.
    let mut delivered: u64 = 0;
    let start = Instant::now();
    let mut target = start;
    for _ in 0..CYCLES {
        let now = Instant::now();
        if now > target + PERIOD {
            let behind = now.duration_since(target).as_nanos() as u64;
            let skip = behind / PERIOD.as_nanos() as u64 + 1;
            target += PERIOD * (skip as u32);
            continue;
        }
        while Instant::now() < target {
            std::hint::spin_loop();
        }
        scheduler.tick_once().expect("tick_once");
        delivered += 1;
        target += PERIOD;
    }

    let observed = ticks.load(Ordering::Relaxed);
    let min_gap = Duration::from_nanos(min_gap_ns.load(Ordering::Relaxed));
    eprintln!(
        "{} on-time bus cycles delivered (of {} attempted) -> {} ticks of a \
         100Hz node, closest pair {:?} apart",
        delivered, CYCLES, observed, min_gap
    );

    assert!(
        delivered >= 50,
        "host delivered only {delivered} on-time cycles of {CYCLES}; too few to \
         conclude anything about the rate gate"
    );

    // Not starved: with regular delivery a 100 Hz node ticks on one delivered
    // cycle in ten, and sparser delivery only raises that ratio. One in twenty
    // is the floor.
    assert!(
        observed * 20 >= delivered,
        "a 100 Hz node ticked only {} times in {} delivered cycles of a 1 kHz \
         drive; it is being starved",
        observed,
        delivered
    );

    // Not accelerated: the gate admits this node at period - half the DRIVING
    // period = 10ms - 0.5ms, so no two ticks may be closer than 9.5ms. The 9ms
    // floor is that minus slop, since the stamp above is taken inside tick(),
    // a little after the gate that admitted it. A tolerance scaled to the
    // node's own period would admit at 5ms and land here.
    assert!(
        min_gap >= Duration::from_micros(9_000),
        "two ticks of a 100 Hz node were only {:?} apart under a 1 kHz drive \
         (>= 9.5ms expected, allowing 9ms): the tolerance is scaled to the \
         node's period instead of the driving period and is letting slow nodes \
         run early",
        min_gap
    );
}

/// The RT hygiene a bus-driven loop needs must be reachable and must report
/// honestly when it does not get what it asked for.
///
/// `run()` applies all of this to the threads it spawns. `tick_once()` ticks on
/// the caller's thread, which gets none of it — so the externally-driven path
/// needs a way to ask, and a way to find out it was refused. This runs
/// unprivileged in CI, where SCHED_FIFO and mlockall are both denied, so it
/// pins the REPORTING contract: refusal is visible, not silent.
#[test]
fn bus_driven_thread_can_request_rt_and_learn_what_it_got() {
    let ticks = Arc::new(AtomicU64::new(0));
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz()).verbose(false);
    let _ = scheduler
        .add(CycleNode {
            ticks: ticks.clone(),
        })
        .rate(1000_u64.hz())
        .build();

    let degradations = scheduler.prepare_current_thread_rt(80, None, 1 << 20);
    eprintln!("rt setup degradations: {:?}", degradations);

    // Whatever the environment granted, the scheduler must still tick. A
    // failed RT setup is a degradation to report, not a reason to stop.
    for _ in 0..10 {
        scheduler
            .tick_once()
            .expect("tick_once after rt setup attempt");
    }
    assert!(
        ticks.load(Ordering::Relaxed) > 0,
        "scheduler stopped ticking after an RT setup attempt"
    );

    // Every degradation must name the thing that failed, so an integrator can
    // decide whether to arm. An empty string, or a bare "error", is useless at
    // the point where the decision gets made.
    for d in &degradations {
        assert!(
            d.contains(':') && d.len() > 8,
            "degradation {:?} does not say what failed",
            d
        );
    }
}
