//! CI-enforceable timing gates built on the runtime's OWN deadline accounting.
//!
//! # Why this exists
//!
//! Every wall-clock jitter assertion in this repo is skipped in CI, and the
//! comments say why: `integration-tests.yml` records a run that measured 34.24%
//! spread and notes that such a number "is evidence of neighbours, not a
//! regression in HORUS". That reasoning is correct, and the consequence is that
//! HORUS ships timing claims that no gate enforces.
//!
//! The way out is not a looser microsecond bound. It is to assert a property
//! whose truth does not depend on how fast or how contended the host is.
//!
//! `CyclicWaiter` counts, per tick: slots serviced, `overruns` (a wait that
//! found its own scheduled slot already in the past), and `slots_skipped` (how
//! many periods those overruns gave up). So the tick grid maintains an
//! identity:
//!
//! ```text
//!     elapsed / period  ==  slots + slots_skipped
//! ```
//!
//! A slow or preempted host makes `overruns` and `slots_skipped` LARGE. It does
//! not break the identity, because time lost to preemption is *accounted*. What
//! breaks the identity is the grid losing track of real time -- sleeping a
//! relative period instead of to an absolute deadline, mishandling an overrun,
//! or double-counting a catch-up. Those are HORUS defects, and they are exactly
//! what a CI gate should catch.
//!
//! This is the difference between measuring the runner and measuring the
//! runtime. These assertions are safe to run on a shared 2-vCPU runner.

use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::rt_wait_stats;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

/// `rt_wait_stats()` reads process-wide counters, so two of these tests running
/// concurrently in the same binary would each see the other's slots inside its
/// own before/after delta. libtest runs tests in parallel by default; CI passes
/// `--test-threads=1`, but a developer typing `cargo test` does not, and a test
/// that is only correct under a flag someone else remembers to pass is not
/// correct. Same pattern as ESTOP_QUEUE_SERIAL in safety_monitor.rs.
static WAIT_STATS_SERIAL: Mutex<()> = Mutex::new(());

fn serial() -> MutexGuard<'static, ()> {
    WAIT_STATS_SERIAL
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// Burns a fixed slice of each period.
///
/// The work is what makes the conservation test discriminating. With a node
/// that returns immediately, sleeping a RELATIVE period and sleeping to an
/// ABSOLUTE deadline produce the same slot count, so the test would pass
/// against the very defect it exists to catch. At 300 us of work in a 1000 us
/// period, a relative sleep yields a 1300 us effective period -- 77% of the
/// expected slot count over the run, outside the 85% floor below, while an
/// absolute deadline stays at ~100%.
const WORK_PER_TICK: Duration = Duration::from_micros(300);

/// `NodeBuilder::build` validates the registration and returns a `HorusResult`;
/// on failure the node is never added. Every test below early-returns when the
/// cyclic waiter reports zero slots, so a discarded `build()` error would turn
/// each of these gates into a vacuous pass over a scheduler that never ran the
/// probe. Registration failure is a test bug, and it must be loud.
const PROBE_MUST_REGISTER: &str =
    "probe node failed to register; without it this gate asserts nothing";

struct TickNode {
    ticks: Arc<AtomicU64>,
}

impl Node for TickNode {
    fn name(&self) -> &'static str {
        "slot_conservation_probe"
    }
    fn tick(&mut self) {
        self.ticks.fetch_add(1, Ordering::Relaxed);
        let start = Instant::now();
        while start.elapsed() < WORK_PER_TICK {
            std::hint::spin_loop();
        }
    }
}

/// The tick grid must account for every period that elapsed.
///
/// Serviced slots plus explicitly-skipped slots must equal the number of
/// periods that actually passed. Preemption moves work from the first term to
/// the second; it must never make them sum to less than elapsed time, which is
/// what silent drift looks like.
#[test]
fn tick_grid_accounts_for_every_elapsed_period() {
    let _serial = serial();
    let before = rt_wait_stats();
    let ticks = Arc::new(AtomicU64::new(0));

    const RATE_HZ: u64 = 1000;
    const RUN: Duration = Duration::from_secs(3);

    let mut scheduler = Scheduler::new().tick_rate(RATE_HZ.hz()).verbose(false);
    scheduler
        .add(TickNode {
            ticks: ticks.clone(),
        })
        .rate(RATE_HZ.hz())
        .build()
        .expect(PROBE_MUST_REGISTER);

    let started = Instant::now();
    scheduler.run_for(RUN).expect("scheduler run");
    let elapsed = started.elapsed();

    let after = rt_wait_stats();
    let slots = after.slots.saturating_sub(before.slots);
    let skipped = after.slots_skipped.saturating_sub(before.slots_skipped);
    let overruns = after.overruns.saturating_sub(before.overruns);

    if slots == 0 {
        // Nothing used the cyclic waiter (no RT chain on this platform/config).
        // Assert nothing rather than assert vacuously.
        eprintln!("cyclic waiter unused on this platform; nothing to conserve");
        return;
    }

    let period_ns = 1_000_000_000f64 / RATE_HZ as f64;
    let periods_elapsed = elapsed.as_nanos() as f64 / period_ns;
    let accounted = slots as f64 + skipped as f64;

    eprintln!(
        "elapsed {:.0} periods | serviced {} | skipped {} | overruns {} | accounted {:.0}",
        periods_elapsed, slots, skipped, overruns, accounted
    );

    // Generous band: startup and shutdown each cost a partial period, and the
    // run boundary is not slot-aligned. 15% absorbs that on any host. What it
    // does NOT absorb is a grid that drifts. The node burns WORK_PER_TICK of
    // every period precisely so that this band discriminates: relative sleeping
    // would give a 1300 us effective period and land at ~77% of the expected
    // count, well under the floor.
    let lower = periods_elapsed * 0.85;
    let upper = periods_elapsed * 1.15;
    assert!(
        accounted >= lower && accounted <= upper,
        "tick grid lost track of real time: {:.0} periods elapsed but only {:.0} \
         accounted for (serviced {} + skipped {}). The grid must sleep to an \
         ABSOLUTE deadline; a relative sleep compounds `work + period` every \
         tick and drifts away from wall time.",
        periods_elapsed,
        accounted,
        slots,
        skipped
    );
}

/// An overrun must give up whole periods, never fractional or negative ones.
///
/// `slots_skipped` is what the grid discarded to catch back up. If overruns are
/// counted but nothing is skipped, the grid is silently accumulating lateness;
/// if slots are skipped with no overrun recorded, the accounting is lying in
/// the other direction. Neither is affected by host speed.
#[test]
fn overruns_and_skipped_slots_agree() {
    let _serial = serial();
    let before = rt_wait_stats();
    let ticks = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz()).verbose(false);
    scheduler
        .add(TickNode {
            ticks: ticks.clone(),
        })
        .rate(1000_u64.hz())
        .build()
        .expect(PROBE_MUST_REGISTER);
    scheduler
        .run_for(Duration::from_secs(2))
        .expect("scheduler run");

    let after = rt_wait_stats();
    let slots = after.slots.saturating_sub(before.slots);
    let skipped = after.slots_skipped.saturating_sub(before.slots_skipped);
    let overruns = after.overruns.saturating_sub(before.overruns);

    if slots == 0 {
        eprintln!("cyclic waiter unused on this platform");
        return;
    }

    eprintln!("slots {} overruns {} skipped {}", slots, overruns, skipped);

    // Every skipped slot came from an overrun, and each overrun gives up at
    // least one slot. Both directions are structural, not statistical.
    assert!(
        skipped >= overruns,
        "{} overruns gave up only {} slots; an overrun that skips nothing has \
         not caught up and the grid stays behind forever",
        overruns,
        skipped
    );
    assert!(
        overruns > 0 || skipped == 0,
        "{} slots were skipped with zero overruns recorded — the skip counter \
         and the overrun counter disagree",
        skipped
    );
}

/// The wake path must not be late by an order of magnitude beyond a period.
///
/// The bound is ten periods of MEAN lateness, and the name says ten because
/// that is what is enforced. A one-period bound would be a wall-clock jitter
/// gate, which is the thing this file exists to avoid: on a contended runner a
/// neighbour can push mean lateness past a single period while the grid's own
/// accounting -- the property the other two tests assert -- stays intact.
///
/// So this bound's job is only to catch a catastrophic regression (a blocking
/// call or an unbounded wait entering the wake path), not to certify jitter;
/// certifying jitter requires the target hardware, PREEMPT_RT and
/// CAP_SYS_NICE, none of which a CI runner has.
#[test]
fn mean_wake_lateness_stays_within_ten_periods() {
    let _serial = serial();
    let before = rt_wait_stats();
    let ticks = Arc::new(AtomicU64::new(0));

    const RATE_HZ: u64 = 1000;
    // Kept as a period count, not a hardcoded nanosecond figure, so the bound
    // and the name it is documented under cannot drift apart again.
    const LATENESS_BUDGET_PERIODS: f64 = 10.0;

    let mut scheduler = Scheduler::new().tick_rate(RATE_HZ.hz()).verbose(false);
    scheduler
        .add(TickNode {
            ticks: ticks.clone(),
        })
        .rate(RATE_HZ.hz())
        .build()
        .expect(PROBE_MUST_REGISTER);
    scheduler
        .run_for(Duration::from_secs(2))
        .expect("scheduler run");

    let after = rt_wait_stats();
    let slots = after.slots.saturating_sub(before.slots);
    if slots == 0 {
        eprintln!("cyclic waiter unused on this platform");
        return;
    }
    let late_total = after
        .wake_late_total_ns
        .saturating_sub(before.wake_late_total_ns);
    let mean_late_ns = late_total as f64 / slots as f64;

    eprintln!(
        "mean wake lateness {:.1} us over {} slots (max seen {} us)",
        mean_late_ns / 1000.0,
        slots,
        after.wake_late_max_ns / 1000
    );

    // One full period of MEAN lateness already means the grid is a period behind
    // on average, but a loaded shared runner reaches that without any HORUS
    // defect. Ten periods of margin keeps this immune to runner contention
    // while still catching a wake path that has become structurally blocking.
    let period_ns = 1_000_000_000f64 / RATE_HZ as f64;
    let budget_ns = period_ns * LATENESS_BUDGET_PERIODS;
    assert!(
        mean_late_ns < budget_ns,
        "mean wake lateness {:.1} ms over {} slots exceeds the {:.0}-period \
         budget ({:.1} ms): the wake path is structurally late, not merely \
         preempted",
        mean_late_ns / 1_000_000.0,
        slots,
        LATENESS_BUDGET_PERIODS,
        budget_ns / 1_000_000.0
    );
}
