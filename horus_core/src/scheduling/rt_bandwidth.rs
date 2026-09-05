//! Whether Linux RT bandwidth control will dequeue this tick loop, decided
//! from what the kernel actually says rather than from what the code assumes.
//!
//! # The warning that could not be wrong, and was
//!
//! Linux polices the SCHED_FIFO/SCHED_RR class as a whole: `sched_rt_runtime_us`
//! out of every `sched_rt_period_us`. An RT runqueue that exceeds its share is
//! forcibly dequeued for the remainder of the period. That mechanism is the
//! reason the tick loop sleeps to an absolute deadline instead of spinning, and
//! the executor's comments have named it — and its ~50 ms dequeue — for a while.
//!
//! Nothing read the sysctls. So the spin-mode warning fired on
//! `HORUS_RT_WAIT=spin && rt_policy_active`, with the budget not an input at
//! all, and three things followed:
//!
//! * **It fired when the condition was absent.** An operator who had already
//!   set `sched_rt_runtime_us=-1` — the exact remedy the executor's own comment
//!   names — was told on every boot that the kernel would dequeue them for
//!   ~50 ms. It cannot: RT throttling is off. A warning that fires when the
//!   thing cannot happen is how operators learn to skip the warning block.
//! * **Its arithmetic was hardcoded.** "~50 ms" and "~50 missed deadlines at
//!   1kHz" are correct only for 950000/1000000 at a 1 ms tick. At a 250 µs tick
//!   the same budget costs ~200 consecutive misses, and at 500000/1000000 the
//!   window is 500 ms. The two numbers an operator needs are
//!   `period_us - runtime_us` and that divided by their own tick period.
//! * **It had no cgroup case.** Under `CONFIG_RT_GROUP_SCHED` a task in a
//!   non-root cpu cgroup gets `cpu.rt_runtime_us == 0` by default, and
//!   `sched_setscheduler(SCHED_FIFO)` then fails with EPERM. That surfaced as
//!   the generic "could not set SCHED_FIFO" line, which points the operator at
//!   `CAP_SYS_NICE` — the wrong remedy, and one they can spend a long time on.
//!
//! Everything here is a pure function of its arguments, so the
//! finite-budget-plus-spin-mode combination is assertable without a privileged
//! kernel and without a real `/proc`.

use horus_sys::rt::RtBandwidth;
use std::time::Duration;

/// How much of each period the tick loop holds the CPU without blocking.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoopDuty {
    /// `HORUS_RT_WAIT=spin`: the loop never blocks. 100% duty, so it exhausts
    /// any finite RT budget by construction.
    Spin,
    /// Absolute sleep, with a `guard_ns` guard spin out of every `period_ns`.
    Yielding {
        /// Nanoseconds of guard spin at the end of each period.
        guard_ns: u64,
        /// Tick period in nanoseconds.
        period_ns: u64,
    },
}

impl LoopDuty {
    /// The fraction of each period this strategy spends runnable.
    ///
    /// An approximation on purpose, and an under-estimate: it counts only the
    /// wait strategy, not the node's own `tick()`. A loop whose work already
    /// fills the period is over budget regardless of how it waits, and that is
    /// a different diagnostic — the deadline-miss ladder — reported elsewhere.
    pub fn fraction(&self) -> f64 {
        match self {
            Self::Spin => 1.0,
            Self::Yielding {
                guard_ns,
                period_ns,
            } => (*guard_ns as f64 / (*period_ns).max(1) as f64).min(1.0),
        }
    }
}

/// What RT bandwidth control will do to this tick loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtBandwidthVerdict {
    /// No RT policy was granted, so the RT class budget does not police this
    /// thread at all.
    NotPoliced,
    /// The budget could not be read: not Linux, or `/proc/sys` hidden.
    Unknown,
    /// `sched_rt_runtime_us == -1`. No throttle — and no runaway defence.
    Unlimited,
    /// Zero budget. `SCHED_FIFO` cannot be granted in this cgroup at all, which
    /// is a different problem from missing `CAP_SYS_NICE` and has a different
    /// remedy.
    Starved,
    /// Finite budget, and the wait strategy alone stays inside it.
    WithinBudget,
    /// Finite budget, and the wait strategy alone exhausts it. The dequeue case.
    WillThrottle,
}

/// Decide the verdict. Pure.
pub fn classify(bw: RtBandwidth, rt_policy_active: bool, duty: LoopDuty) -> RtBandwidthVerdict {
    if !rt_policy_active {
        return RtBandwidthVerdict::NotPoliced;
    }
    if !bw.is_known() {
        return RtBandwidthVerdict::Unknown;
    }
    if bw.is_unlimited() {
        return RtBandwidthVerdict::Unlimited;
    }
    if bw.is_starved() {
        return RtBandwidthVerdict::Starved;
    }
    match bw.duty_fraction() {
        // `>=`, not `>`: a loop whose duty exactly equals its share has no
        // headroom for the node's own work, and the node's work is the point.
        Some(share) if duty.fraction() >= share => RtBandwidthVerdict::WillThrottle,
        _ => RtBandwidthVerdict::WithinBudget,
    }
}

/// The line to print, or `None` when there is nothing worth saying.
///
/// Every number in it is computed from `bw` and `tick_period`, so it is right
/// for budgets other than the default and tick rates other than 1 kHz.
pub fn advisory(
    verdict: RtBandwidthVerdict,
    bw: RtBandwidth,
    tick_period: Duration,
    contained: bool,
    verbose: bool,
) -> Option<String> {
    match verdict {
        // Nothing to say: the common, healthy case.
        RtBandwidthVerdict::NotPoliced | RtBandwidthVerdict::WithinBudget => None,

        RtBandwidthVerdict::Unknown if verbose => Some(
            "[RT-thread] RT bandwidth budget could not be read \
             (/proc/sys/kernel/sched_rt_runtime_us); whether this thread can be \
             throttled is unknown."
                .to_string(),
        ),
        RtBandwidthVerdict::Unknown => None,

        // Worth one line, and only in verbose: it is what the operator asked
        // for, but it also removes the kernel's only defence against a runaway
        // RT loop taking the machine with it.
        RtBandwidthVerdict::Unlimited if verbose => Some(
            "[RT-thread] RT bandwidth control is disabled (sched_rt_runtime_us = -1): \
             this thread cannot be throttled, and a runaway RT loop has nothing \
             stopping it from locking out the rest of the system."
                .to_string(),
        ),
        RtBandwidthVerdict::Unlimited => None,

        RtBandwidthVerdict::Starved => Some(format!(
            "[RT-thread] WARNING: the RT budget for this task is ZERO ({}/{} us{}). \
             SCHED_FIFO cannot be granted here at all, and the refusal will read as \
             a permission error — but CAP_SYS_NICE is not the remedy. Raise \
             cpu.rt_runtime_us for this cgroup, or run outside it.",
            bw.runtime_us,
            bw.period_us,
            if contained {
                ", from this task's cgroup"
            } else {
                ""
            }
        )),

        RtBandwidthVerdict::WillThrottle => {
            let window = bw.throttle_window().unwrap_or_default();
            let missed = bw.missed_ticks_per_period(tick_period).unwrap_or(0);
            Some(format!(
                "[RT-thread] WARNING: this tick loop holds the CPU for its whole period \
                 while the RT class is limited to {}/{} us ({:.1}%). The kernel will \
                 dequeue it for {:?} out of every {:?} once the share is exhausted — \
                 {} consecutive missed deadlines at this {:?} tick period. Median wake \
                 jitter improves; the worst case gets far worse. Raise \
                 /proc/sys/kernel/sched_rt_runtime_us, or use the default \
                 absolute-sleep wait.",
                bw.runtime_us,
                bw.period_us,
                bw.duty_fraction().unwrap_or(0.0) * 100.0,
                window,
                Duration::from_micros(bw.period_us.max(0) as u64),
                missed,
                tick_period,
            ))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use horus_sys::rt::{RtBandwidth, RtBandwidthSource};

    const DEFAULT: RtBandwidth = RtBandwidth {
        runtime_us: 950_000,
        period_us: 1_000_000,
        source: RtBandwidthSource::ProcSys,
    };
    const UNLIMITED: RtBandwidth = RtBandwidth {
        runtime_us: -1,
        period_us: 1_000_000,
        source: RtBandwidthSource::ProcSys,
    };
    const STARVED: RtBandwidth = RtBandwidth {
        runtime_us: 0,
        period_us: 1_000_000,
        source: RtBandwidthSource::CgroupV1,
    };

    const SLEEPING: LoopDuty = LoopDuty::Yielding {
        guard_ns: 20_000,
        period_ns: 1_000_000,
    };

    /// The false positive the old warning produced, stated as a test.
    #[test]
    fn an_unlimited_budget_is_never_reported_as_a_throttle() {
        assert_eq!(
            classify(UNLIMITED, true, LoopDuty::Spin),
            RtBandwidthVerdict::Unlimited,
            "an operator who set sched_rt_runtime_us=-1 has done the exact thing the \
             executor's comments recommend; telling them the kernel will dequeue them \
             anyway is how a warning block gets ignored"
        );
        assert!(advisory(
            RtBandwidthVerdict::Unlimited,
            UNLIMITED,
            Duration::from_millis(1),
            false,
            false
        )
        .is_none());
    }

    #[test]
    fn spin_mode_against_a_finite_budget_will_throttle() {
        assert_eq!(
            classify(DEFAULT, true, LoopDuty::Spin),
            RtBandwidthVerdict::WillThrottle
        );
        // ...and the default sleeping strategy does not.
        assert_eq!(
            classify(DEFAULT, true, SLEEPING),
            RtBandwidthVerdict::WithinBudget
        );
    }

    #[test]
    fn the_budget_is_not_policed_without_an_rt_policy() {
        assert_eq!(
            classify(DEFAULT, false, LoopDuty::Spin),
            RtBandwidthVerdict::NotPoliced,
            "RT bandwidth control polices the RT class; a SCHED_OTHER thread is not in it"
        );
    }

    #[test]
    fn an_unreadable_budget_is_unknown_not_unlimited() {
        assert_eq!(
            classify(RtBandwidth::UNAVAILABLE, true, LoopDuty::Spin),
            RtBandwidthVerdict::Unknown
        );
        // A garbled read must not be mistaken for "no throttle": that would
        // suppress the warning in precisely the case where nothing about the
        // host is understood.
        let garbled = RtBandwidth::from_raw(950_000, 0, RtBandwidthSource::ProcSys);
        assert_eq!(
            classify(garbled, true, LoopDuty::Spin),
            RtBandwidthVerdict::Unknown
        );
    }

    #[test]
    fn a_zero_budget_names_the_cgroup_and_not_cap_sys_nice() {
        assert_eq!(
            classify(STARVED, true, SLEEPING),
            RtBandwidthVerdict::Starved
        );
        let line = advisory(
            RtBandwidthVerdict::Starved,
            STARVED,
            Duration::from_millis(1),
            true,
            false,
        )
        .expect("a zero budget always warrants a line");
        assert!(line.contains("cpu.rt_runtime_us"), "{line}");
        assert!(
            line.contains("CAP_SYS_NICE is not the remedy"),
            "the EPERM this produces reads exactly like a missing capability, and an \
             operator can spend a long time on the wrong fix: {line}"
        );
    }

    /// The hardcoded arithmetic, replaced.
    #[test]
    fn the_throttle_window_and_missed_ticks_are_computed_not_assumed() {
        assert_eq!(
            DEFAULT.throttle_window(),
            Some(Duration::from_millis(50)),
            "period - runtime, which is where the comments' ~50ms comes from"
        );
        assert_eq!(
            DEFAULT.missed_ticks_per_period(Duration::from_millis(1)),
            Some(50),
            "the familiar '~50 missed deadlines at 1kHz'"
        );
        assert_eq!(
            DEFAULT.missed_ticks_per_period(Duration::from_micros(250)),
            Some(200),
            "at a 250us tick the SAME budget costs 200 consecutive misses — the \
             hardcoded 50 was only ever right for one tick rate"
        );

        let half = RtBandwidth::from_raw(500_000, 1_000_000, RtBandwidthSource::ProcSys);
        assert_eq!(half.throttle_window(), Some(Duration::from_millis(500)));
        assert_eq!(
            half.missed_ticks_per_period(Duration::from_millis(1)),
            Some(500)
        );

        let line = advisory(
            RtBandwidthVerdict::WillThrottle,
            half,
            Duration::from_micros(250),
            false,
            false,
        )
        .expect("a throttling combination always warrants a line");
        assert!(line.contains("2000 consecutive missed deadlines"), "{line}");
    }
}
