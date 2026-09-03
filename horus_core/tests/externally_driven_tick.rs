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

    // A bus cycle arrives on the bus's clock, not ours. Emulate that faithfully:
    // sleep to an absolute grid so the cadence does not accumulate our own
    // overhead, which is exactly how a real master hands off cycles.
    let start = Instant::now();
    for c in 0..CYCLES {
        let target = start + PERIOD * (c as u32);
        while Instant::now() < target {
            std::hint::spin_loop();
        }
        scheduler.tick_once().expect("tick_once");
    }

    let observed = ticks.load(Ordering::Relaxed);
    eprintln!("{} bus cycles -> {} node ticks", CYCLES, observed);

    // The failure this pins is a HALVING (or worse): the strict gate rejects the
    // boundary cycle every time, so the node sees ~50%. Requiring 90% leaves room
    // for the first cycle and for genuine host stalls while still failing hard on
    // a systematic every-other-cycle refusal.
    let floor = (CYCLES as f64 * 0.90) as u64;
    assert!(
        observed >= floor,
        "a .rate(1000hz) node driven by a 1 kHz external cycle ticked only {} \
         times in {} cycles (needed >= {}). The rate gate is refusing cycles \
         that arrive a hair early; the RT executor solved this with a \
         half-period tolerance and the externally-driven path needs the same.",
        observed,
        CYCLES,
        floor
    );
}

/// The tolerance must not let a SLOWER node run early.
///
/// A 100 Hz node driven by a 1 kHz cycle must still see ~100 ticks/second, not
/// 1000. Widening the gate is only safe if it is bounded by the driving period.
#[test]
fn slower_node_is_not_accelerated_by_the_tolerance() {
    let ticks = Arc::new(AtomicU64::new(0));
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz()).verbose(false);
    let _ = scheduler
        .add(CycleNode {
            ticks: ticks.clone(),
        })
        .rate(100_u64.hz())
        .build();

    const CYCLES: u64 = 400;
    const PERIOD: Duration = Duration::from_micros(1000);

    let start = Instant::now();
    for c in 0..CYCLES {
        let target = start + PERIOD * (c as u32);
        while Instant::now() < target {
            std::hint::spin_loop();
        }
        scheduler.tick_once().expect("tick_once");
    }

    let observed = ticks.load(Ordering::Relaxed);
    eprintln!(
        "{} bus cycles at 1kHz -> {} ticks of a 100Hz node",
        CYCLES, observed
    );

    // 400 cycles at 1 kHz is 0.4 s, so a 100 Hz node owes ~40 ticks. Allow a
    // wide band for host jitter, but nothing near the 400 that a tolerance
    // scaled to the NODE's period instead of the DRIVING period would produce.
    assert!(
        observed <= 80,
        "a 100 Hz node ticked {} times in 400 cycles of a 1 kHz drive: the \
         tolerance is scaled wrongly and is letting slow nodes run early",
        observed
    );
    assert!(
        observed >= 20,
        "a 100 Hz node ticked only {} times in 0.4 s; it is being starved",
        observed
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
