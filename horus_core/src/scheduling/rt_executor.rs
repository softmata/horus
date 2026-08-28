#![allow(dead_code)]
//! Dedicated RT thread executor.
//!
//! Runs all RT nodes on an isolated OS thread with its own tick loop.
//! The RT thread is never blocked by compute or event nodes — it ticks
//! RT nodes sequentially in priority order at their declared rates.
//!
//! # Architecture
//!
//! ```text
//!  Main Thread (compute/event)    RT Thread (isolated)
//!  ┌──────────────────────┐       ┌──────────────────────┐
//!  │  non-RT tick loop    │       │  RT tick loop         │
//!  │  ┌────────────────┐  │       │  ┌────────────────┐   │
//!  │  │ compute nodes  │  │       │  │ RT node 0 tick │   │
//!  │  │ (parallel)     │  │       │  │ RT node 1 tick │   │
//!  │  └────────────────┘  │       │  │ RT node 2 tick │   │
//!  │                      │       │  └────────────────┘   │
//!  │  shared: running     │◄──────┤  uses: NodeRunner     │
//!  │  (AtomicBool)        │       │  SCHED_FIFO if avail  │
//!  └──────────────────────┘       └──────────────────────┘
//! ```

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use super::types::Diag;
use crate::error::{Error, Result};
use crate::terminal::print_line;

/// Monotonic nanoseconds for throttling. `Instant` has no epoch to compare
/// against across calls, and `SystemTime` can step backwards under NTP — which
/// would make a throttle window either never expire or expire instantly.
fn monotonic_nanos() -> u64 {
    use std::sync::OnceLock;
    static ORIGIN: OnceLock<std::time::Instant> = OnceLock::new();
    ORIGIN
        .get_or_init(std::time::Instant::now)
        .elapsed()
        .as_nanos() as u64
}

/// `" (+N more in the last second)"`, or nothing when N is zero.
///
/// The count is what keeps throttling honest: an operator sees both that the
/// fault is happening and how often, without the log being destroyed by it.
fn suppressed_suffix(hidden: u32) -> String {
    if hidden == 0 {
        String::new()
    } else {
        format!(" (+{hidden} more in the last second)")
    }
}

// ════════════════════════════════════════════════════════════════════════════
// Cyclic wait: absolute-deadline sleep with a bounded guard spin
// ════════════════════════════════════════════════════════════════════════════
//
// # What this replaces, and why
//
// The tick loop used to close each period with, in essence:
//
// ```text
// let elapsed = loop_start.elapsed();              // loop_start re-sampled each iteration
// if tick_period - elapsed < 1ms { spin until the period is up }
// else { sleep(tick_period - elapsed - 500us); then spin }
// ```
//
// Two defects, in increasing order of severity.
//
// **1. Drift.** The target was `loop_start + tick_period` with `loop_start`
// re-sampled by `Instant::now()` at the top of every iteration — i.e. "now +
// period", not "start + n*period". Every iteration folded its own overshoot
// (spin-exit granularity, loop overhead, wake latency) into the phase and never
// gave it back. At 1 kHz even a 50 ns per-tick overshoot is 50 us/s of phase
// slip, ~4.3 s/day. Anything that assumes a fixed dt — a discrete-time
// controller, a sensor fusion timestamp — inherits that slip. This is the
// classic bug in this code shape, and the old code had it.
//
// **2. An unbounded busy-wait under RT bandwidth control.** The branch tested
// SLACK (`tick_period - elapsed`), not the period, so it took the pure-spin
// path whenever a tick left under 1 ms of slack. That is unconditionally true
// for any tick period at or below 1 ms, and intermittently true at larger
// periods whenever a tick ran long — so a 1 kHz chain busy-waited essentially
// the entire period, every period. Linux RT bandwidth control
// (`sched_rt_runtime_us` / `sched_rt_period_us`, 950 ms / 1000 ms by default)
// forcibly DEQUEUES a SCHED_FIFO thread that exceeds its share for the
// remainder of the RT period: roughly 50 ms, i.e. ~50 consecutive missed
// deadlines at 1 kHz, once per second. SCHED_DEADLINE polices its declared
// runtime even more tightly.
//
// That tail never showed up in test because `set_realtime_priority` fails
// without CAP_SYS_NICE and the thread silently stays SCHED_OTHER, where RT
// bandwidth control does not apply. **The defect only manifests once RT
// priority is actually granted** — on the robot, not on a developer box or in
// CI. `CyclicWaiter::new` therefore takes `rt_policy_active` and warns exactly
// in the configuration where the old behaviour would bite.
//
// # The replacement, and the trade it makes
//
// Sleep to an ABSOLUTE deadline on CLOCK_MONOTONIC (`clock_nanosleep` with
// `TIMER_ABSTIME`) for the bulk of the period, then guard-spin only the last
// few microseconds. This is the standard cyclictest-style cyclic-task pattern.
//
// **TRADE, STATED RATHER THAN BURIED:** median wake jitter rises from the old
// spin's ~100 ns to hrtimer precision — single-digit microseconds on
// PREEMPT_RT, worse on a stock kernel. That is a MEDIAN REGRESSION, bought to
// delete a ~50 ms tail item. It is the right trade for this runtime's figure of
// merit (worst case and jitter first, median second), but it is a real cost, so
// it is explicit here, configurable (`HORUS_RT_WAIT=spin` restores a pure spin,
// `HORUS_RT_SPIN_GUARD_US` retunes the guard), and measured
// (`rt_wait_stats()` publishes the observed wake lateness and spin time).
//
// The fix is deliberately in the code. The other way to stop the throttle is
// `sched_rt_runtime_us=-1`, and that is NOT recommended: it removes the
// kernel's last defence against a runaway RT thread wedging the machine. A loop
// that gives back the CPU it does not need makes that config change
// unnecessary.

/// Width of the busy-wait that guards the final approach to a tick deadline.
///
/// Chosen against the wake-latency distribution the guard exists to hide: on a
/// PREEMPT_RT kernel a pinned SCHED_FIFO thread's hrtimer wakeup lands within
/// single-digit microseconds of the programmed time, so a 20 us guard converts
/// the *typical* wake into a deadline-accurate one. It deliberately does NOT
/// cover the worst case — widening it toward the tail would trade back exactly
/// the RT bandwidth this rewrite reclaims, and buys nothing on a stock kernel,
/// where the overshoot usually exceeds the guard and no spin happens at all.
///
/// Cost ceiling: 20 us is 2 % of a 1 kHz period, and the spin only covers the
/// slice of the guard the sleep did not already consume, so the typical cost is
/// well under that. Override with `HORUS_RT_SPIN_GUARD_US`; `0` disables the
/// spin entirely (lowest CPU, pure hrtimer precision).
const SPIN_GUARD_DEFAULT_NS: u64 = 20_000;

/// The guard spin may never exceed `tick_period >> SPIN_GUARD_PERIOD_SHIFT`
/// — one sixteenth, 6.25 %, of the period.
///
/// This clamp is the structural defence against re-introducing the defect above
/// at high tick rates. A *fixed* guard is 2 % of a 1 ms period but 20 % of a
/// 100 us one, so it would silently walk the loop back toward the kernel's 95 %
/// RT bandwidth ceiling as the rate rises. Bounding the guard as a fraction of
/// the period keeps the loop's unconditional CPU draw at 6.25 % or below at
/// every rate, leaving the remainder of the budget for the nodes' real work.
const SPIN_GUARD_PERIOD_SHIFT: u32 = 4;

/// Minimum slack worth an absolute-sleep syscall, in nanoseconds.
///
/// Arming an hrtimer, switching out, taking the timer interrupt and switching
/// back costs single-digit microseconds. Below this threshold the syscall costs
/// more than the wait it replaces, so the residue is spun instead. The spin is
/// bounded by this constant, so it can only dominate periods shorter than
/// `MIN_SLEEP_SLACK_NS << SPIN_GUARD_PERIOD_SHIFT` (160 us, i.e. above ~6 kHz);
/// `CyclicWaiter::new` warns once when the configured rate is in that regime,
/// because a spin-dominated loop plus a real RT policy is the throttle case.
const MIN_SLEEP_SLACK_NS: u64 = 10_000;

/// How often the per-thread wait counters are published to the process-wide
/// totals. The counters are plain integers in the hot path (a few register
/// adds); only this flush touches shared atomics, so the tick loop never pays
/// for a contended cache line.
const WAIT_STATS_FLUSH_INTERVAL_NS: u64 = 1_000_000_000;

/// Absolute CLOCK_MONOTONIC nanoseconds — the same timebase the absolute sleep
/// targets, so deadline arithmetic and the guard spin cannot disagree.
///
/// Distinct from [`monotonic_nanos`], which is `Instant`-based and exists for
/// log throttling; mixing the two epochs in deadline arithmetic would be a bug.
#[cfg(target_os = "linux")]
#[inline]
fn cyclic_now_ns() -> u64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `ts` is a live, writable `timespec` and CLOCK_MONOTONIC is always
    // a valid clock id, so both documented failure modes (EFAULT for a bad
    // pointer, EINVAL for a bad clock id) are unreachable. On Linux this
    // resolves through the vDSO and does not enter the kernel.
    unsafe {
        libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts);
    }
    (ts.tv_sec as u64)
        .wrapping_mul(1_000_000_000)
        .wrapping_add(ts.tv_nsec as u64)
}

/// Portable monotonic nanoseconds for the non-Linux cyclic path.
#[cfg(not(target_os = "linux"))]
#[inline]
fn cyclic_now_ns() -> u64 {
    monotonic_nanos()
}

/// Sleep until an absolute instant on CLOCK_MONOTONIC.
#[cfg(target_os = "linux")]
#[inline]
fn sleep_until_ns(deadline_ns: u64) {
    let ts = libc::timespec {
        tv_sec: (deadline_ns / 1_000_000_000) as _,
        tv_nsec: (deadline_ns % 1_000_000_000) as _,
    };
    loop {
        // SAFETY: `ts` is a live, valid `timespec` for the duration of the
        // call. With TIMER_ABSTIME the kernel never writes the remaining-time
        // pointer, so passing null is correct. `clock_nanosleep` returns the
        // error number directly and does NOT set `errno`.
        let rc = unsafe {
            libc::clock_nanosleep(
                libc::CLOCK_MONOTONIC,
                libc::TIMER_ABSTIME,
                &ts,
                std::ptr::null_mut(),
            )
        };
        if rc != libc::EINTR {
            return;
        }
        // A signal handler ran. Because the target is ABSOLUTE, re-arming with
        // the identical timespec resumes the same deadline and loses no time.
        // A relative `nanosleep` would have to carry `rmtp` forward and would
        // shed a little phase on every signal — the second reason to use
        // TIMER_ABSTIME, on top of drift.
    }
}

/// Portable fallback: a relative sleep recomputed from the ABSOLUTE target on
/// every period, so oversleep is corrected rather than accumulated.
#[cfg(not(target_os = "linux"))]
#[inline]
fn sleep_until_ns(deadline_ns: u64) {
    let now = cyclic_now_ns();
    if deadline_ns > now {
        std::thread::sleep(Duration::from_nanos(deadline_ns - now));
    }
}

/// How the tick loop waits out the remainder of a period.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum WaitMode {
    /// Sleep to an absolute CLOCK_MONOTONIC deadline (`clock_nanosleep` +
    /// `TIMER_ABSTIME`), then guard-spin the residue. The default, and the
    /// only mode that keeps the loop inside its RT bandwidth share.
    AbsoluteSleep,
    /// Non-Linux fallback: absolute deadlines on `Instant`'s clock, reached
    /// with a relative `thread::sleep` recomputed each period.
    PortableSleep,
    /// Pure busy-wait to the absolute deadline. Opt-in via `HORUS_RT_WAIT=spin`
    /// for the ~100 ns median wake, and DANGEROUS to combine with a real RT
    /// policy: see the section note above. Unlike the code this replaces it is
    /// still phase-absolute, so it does not drift.
    Spin,
}

/// Observed behaviour of the cyclic wait, aggregated across all RT threads.
///
/// This rewrite trades median wake jitter for the removal of a ~50 ms tail, and
/// a trade you cannot see is a trade you cannot defend. These counters are what
/// make both sides of it visible in production: `wake_late_max_ns` is the worst
/// gap ever observed between a tick's scheduled slot and the instant the loop
/// actually resumed, and `overruns` / `slots_skipped` count the periods the
/// executor could not keep up with at all.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub(crate) struct RtWaitSnapshot {
    /// Completed cyclic waits (i.e. tick slots serviced).
    pub slots: u64,
    /// Waits that found their own slot already in the past.
    pub overruns: u64,
    /// Total slots dropped by those overruns.
    pub slots_skipped: u64,
    /// Worst observed `resume - scheduled_slot`. The tail metric for the wake
    /// path, and the honest cost of giving up the busy-wait.
    pub wake_late_max_ns: u64,
    /// Sum of `resume - scheduled_slot`; divide by `slots` for the mean.
    pub wake_late_total_ns: u64,
    /// Total time spent in the guard spin. The RT-bandwidth draw of the wait
    /// itself — what used to be ~99.5 % of every period.
    pub spin_total_ns: u64,
}

static WAIT_SLOTS: AtomicU64 = AtomicU64::new(0);
static WAIT_OVERRUNS: AtomicU64 = AtomicU64::new(0);
static WAIT_SLOTS_SKIPPED: AtomicU64 = AtomicU64::new(0);
static WAIT_LATE_MAX_NS: AtomicU64 = AtomicU64::new(0);
static WAIT_LATE_TOTAL_NS: AtomicU64 = AtomicU64::new(0);
static WAIT_SPIN_TOTAL_NS: AtomicU64 = AtomicU64::new(0);

/// Process-wide cyclic-wait statistics, safe to poll from any thread.
pub(crate) fn rt_wait_stats() -> RtWaitSnapshot {
    RtWaitSnapshot {
        slots: WAIT_SLOTS.load(Ordering::Relaxed),
        overruns: WAIT_OVERRUNS.load(Ordering::Relaxed),
        slots_skipped: WAIT_SLOTS_SKIPPED.load(Ordering::Relaxed),
        wake_late_max_ns: WAIT_LATE_MAX_NS.load(Ordering::Relaxed),
        wake_late_total_ns: WAIT_LATE_TOTAL_NS.load(Ordering::Relaxed),
        spin_total_ns: WAIT_SPIN_TOTAL_NS.load(Ordering::Relaxed),
    }
}

/// Drives one RT thread's tick cadence on an absolute phase grid.
///
/// Deadlines are always `anchor + n * period` for an integer `n`. They are
/// never computed as "now + period": that is the drift bug this type exists to
/// remove, and the arithmetic below is written so the grid survives even a
/// multi-second stall.
struct CyclicWaiter {
    mode: WaitMode,
    period_ns: u64,
    guard_ns: u64,
    /// Phase anchor. Every slot is `anchor_ns + n * period_ns`, exactly.
    anchor_ns: u64,
    /// Absolute time at which the NEXT work pass should begin.
    next_slot_ns: u64,
    /// Counters since the last flush. Plain integers so the per-period cost is
    /// a handful of register adds, not shared-atomic traffic.
    local: RtWaitSnapshot,
    last_flush_ns: u64,
}

impl CyclicWaiter {
    /// `rt_policy_active` is whether SCHED_FIFO or SCHED_DEADLINE was actually
    /// granted. It gates the warnings, because RT bandwidth control — the whole
    /// reason this type exists — only applies when it is true.
    fn new(tick_period: Duration, rt_policy_active: bool, verbose: bool) -> Self {
        // A zero period would make the slot arithmetic divide by zero; clamp to
        // 1 ns, which degenerates to "run flat out" exactly as before.
        let period_ns = (tick_period.as_nanos() as u64).max(1);

        let mut mode = if cfg!(target_os = "linux") {
            WaitMode::AbsoluteSleep
        } else {
            WaitMode::PortableSleep
        };
        match std::env::var("HORUS_RT_WAIT").as_deref() {
            Ok("spin") => mode = WaitMode::Spin,
            Ok("sleep") => {}
            Ok(other) => print_line(&format!(
                "[RT-thread] HORUS_RT_WAIT='{other}' not recognised (expected 'sleep' or 'spin') — using 'sleep'"
            )),
            Err(_) => {}
        }

        let requested_guard_ns = std::env::var("HORUS_RT_SPIN_GUARD_US")
            .ok()
            .and_then(|v| v.parse::<u64>().ok())
            .map(|us| us.saturating_mul(1_000))
            .unwrap_or(SPIN_GUARD_DEFAULT_NS);
        // Never let the guard eat more than 1/16 of the period; see
        // SPIN_GUARD_PERIOD_SHIFT.
        let guard_ns = requested_guard_ns.min(period_ns >> SPIN_GUARD_PERIOD_SHIFT);

        if mode == WaitMode::Spin && rt_policy_active {
            // Unconditional, not verbose-gated: this is the exact combination
            // that produces the ~50 ms dequeue, and an operator who opted into
            // it deserves to be told on every boot.
            print_line(
                "[RT-thread] WARNING: HORUS_RT_WAIT=spin with a real RT policy. The tick loop \
                 will busy-wait every period; Linux RT bandwidth control will dequeue this \
                 thread for ~50ms once its share is exhausted (~50 missed deadlines at 1kHz). \
                 Median wake jitter improves to ~100ns and the worst case gets far worse.",
            );
        } else if period_ns < (MIN_SLEEP_SLACK_NS << SPIN_GUARD_PERIOD_SHIFT) && rt_policy_active {
            print_line(&format!(
                "[RT-thread] WARNING: tick period {}us is below the {}us floor where an \
                 absolute sleep is cheaper than spinning; the final approach will be \
                 spin-dominated and RT bandwidth control may throttle this thread.",
                period_ns / 1000,
                (MIN_SLEEP_SLACK_NS << SPIN_GUARD_PERIOD_SHIFT) / 1000,
            ));
        }

        if verbose {
            print_line(&format!(
                "[RT-thread] Cyclic wait: {:?}, period {}us, guard spin {}ns, RT policy {}",
                mode,
                period_ns / 1000,
                guard_ns,
                if rt_policy_active {
                    "ACTIVE"
                } else {
                    "not granted (SCHED_OTHER)"
                },
            ));
        }

        // Anchor the phase grid LAST: the diagnostics above take a lock and may
        // touch stdout, and folding that into the anchor would make the very
        // first slot arrive already overrun.
        let now = cyclic_now_ns();

        Self {
            mode,
            period_ns,
            guard_ns,
            anchor_ns: now,
            next_slot_ns: now.wrapping_add(period_ns),
            local: RtWaitSnapshot::default(),
            last_flush_ns: now,
        }
    }

    /// Wait out the remainder of the current period and return at the next
    /// slot boundary.
    fn wait(&mut self) {
        let now = cyclic_now_ns();
        self.local.slots += 1;

        if now >= self.next_slot_ns {
            // OVERRUN — the work pass ran past its own slot boundary.
            //
            // POLICY (documented deliberately, because both choices are
            // defensible): the executor SKIPS FORWARD to the next slot that is
            // still in the future, on the ORIGINAL phase grid, and waits for
            // it. Missed slots are dropped, never run back to back.
            //
            // Why skip rather than catch up: a burst of zero-slack ticks is
            // precisely the CPU pattern that trips RT bandwidth control, so a
            // catch-up policy would let a single overrun manufacture the very
            // ~50 ms dequeue this rewrite exists to remove. It would also hand
            // nodes an inter-tick interval far shorter than the fixed dt their
            // control math assumes. Skipping keeps every interval a whole
            // number of periods and keeps the loop phase-locked to its grid.
            //
            // Why it cannot spiral: the number of slots to drop is one
            // division, not a loop, so even a multi-second stall (suspend/
            // resume, a stop-the-world pause, the old RT throttle itself) costs
            // O(1) and lands the loop exactly one slot ahead of `now`. Cost per
            // overrun is bounded and independent of how far behind we were.
            let late_ns = now - self.next_slot_ns;
            let skipped = late_ns / self.period_ns + 1;
            self.local.overruns += 1;
            self.local.slots_skipped += skipped;
            self.next_slot_ns = self
                .next_slot_ns
                .wrapping_add(skipped.wrapping_mul(self.period_ns));
        }

        let target = self.next_slot_ns;
        // Advance the grid BEFORE waiting: the next slot is defined by the
        // phase, not by when this wait happens to return.
        self.next_slot_ns = target.wrapping_add(self.period_ns);

        let mut t = now;
        if self.mode != WaitMode::Spin {
            let sleep_until = target.saturating_sub(self.guard_ns);
            // A `while`, not an `if`: a platform sleep that returns early is
            // re-armed against the same absolute target, which keeps the guard
            // spin below bounded by `guard_ns + MIN_SLEEP_SLACK_NS` instead of
            // letting it absorb the whole period.
            while sleep_until > t && sleep_until - t >= MIN_SLEEP_SLACK_NS {
                sleep_until_ns(sleep_until);
                t = cyclic_now_ns();
            }
        }

        // Guard spin. Bounded by construction: the absolute sleep above returns
        // at or after `target - guard_ns`, so this covers at most `guard_ns` —
        // and nothing at all when the wake already overshot the guard, which is
        // the common case on a stock kernel.
        let spin_start = t;
        while t < target {
            std::hint::spin_loop();
            t = cyclic_now_ns();
        }

        self.local.spin_total_ns += t.saturating_sub(spin_start);
        let late_ns = t.saturating_sub(target);
        self.local.wake_late_total_ns += late_ns;
        if late_ns > self.local.wake_late_max_ns {
            self.local.wake_late_max_ns = late_ns;
        }

        if t.saturating_sub(self.last_flush_ns) >= WAIT_STATS_FLUSH_INTERVAL_NS {
            self.flush();
            self.last_flush_ns = t;
        }
    }

    /// Publish the local counters to the process-wide totals and reset them.
    fn flush(&mut self) {
        WAIT_SLOTS.fetch_add(self.local.slots, Ordering::Relaxed);
        WAIT_OVERRUNS.fetch_add(self.local.overruns, Ordering::Relaxed);
        WAIT_SLOTS_SKIPPED.fetch_add(self.local.slots_skipped, Ordering::Relaxed);
        WAIT_LATE_TOTAL_NS.fetch_add(self.local.wake_late_total_ns, Ordering::Relaxed);
        WAIT_SPIN_TOTAL_NS.fetch_add(self.local.spin_total_ns, Ordering::Relaxed);
        WAIT_LATE_MAX_NS.fetch_max(self.local.wake_late_max_ns, Ordering::Relaxed);
        self.local = RtWaitSnapshot::default();
    }

    /// Final flush plus, under `verbose`, the one line that makes the trade
    /// legible: what the wake path actually cost on this run.
    fn finish(&mut self, verbose: bool) {
        let slots = self.local.slots;
        let late_total = self.local.wake_late_total_ns;
        let late_max = self.local.wake_late_max_ns;
        let overruns = self.local.overruns;
        let spin = self.local.spin_total_ns;
        self.flush();
        if verbose {
            let mean = late_total.checked_div(slots).unwrap_or(0);
            print_line(&format!(
                "[RT-thread] Cyclic wait: {slots} slots, wake late mean {mean}ns / max \
                 {late_max}ns, {overruns} overruns, {spin}ns spun"
            ));
        }
    }
}

use super::primitives::{DeadlineAction, NodeRunner, TimingEnforcer};
use super::types::{RegisteredNode, SharedMonitors};
use crate::core::DurationExt;

/// Dedicated RT thread executor.
///
/// Owns RT nodes and ticks them on isolated OS thread(s) at the rate of
/// the fastest node. Shutdown is coordinated via the shared `running` flag.
///
/// Supports multiple RT threads for independent node chains — each chain
/// gets its own thread with optional CPU pinning and priority.
pub(crate) struct RtExecutor {
    handles: Vec<std::thread::JoinHandle<Vec<RegisteredNode>>>,
}

impl RtExecutor {
    /// Start the RT executor with the given nodes on a dedicated thread.
    ///
    /// `running` is the shared scheduler running flag — when set to `false`,
    /// the RT thread finishes its current tick cycle and returns its nodes.
    ///
    /// The tick period is derived from the fastest RT node's rate. If no node
    /// has a declared rate, falls back to `fallback_period`.
    /// Start the RT executor with multiple independent chains on separate threads.
    ///
    /// Each chain gets its own dedicated RT thread. Independent chains run in
    /// parallel — a slow node in chain 1 cannot block chain 2.
    ///
    /// Falls back to single thread when given 1 chain.
    pub fn start_pool(
        chains: Vec<Vec<RegisteredNode>>,
        running: Arc<AtomicBool>,
        fallback_period: Duration,
        monitors: SharedMonitors,
        rt_cpus: Vec<usize>,
    ) -> Result<Self> {
        let num_chains = chains.len();
        let mut handles = Vec::with_capacity(num_chains);

        for (chain_idx, mut nodes) in chains.into_iter().enumerate() {
            // Sort by priority before handing off to the thread
            nodes.sort_by_key(|n| n.priority);

            // Determine tick period from the fastest node in this chain
            let max_rate_hz = nodes
                .iter()
                .filter_map(|n| n.rate_hz)
                .fold(0.0_f64, f64::max);

            let tick_period = if max_rate_hz > 0.0 {
                max_rate_hz.hz().period()
            } else {
                fallback_period
            };

            // Assign CPU core: round-robin across available RT CPUs
            let thread_cpus = if !rt_cpus.is_empty() {
                vec![rt_cpus[chain_idx % rt_cpus.len()]]
            } else {
                vec![]
            };

            let thread_name = if num_chains == 1 {
                "horus-rt".to_string()
            } else {
                format!("horus-rt-{}", chain_idx)
            };

            let running = running.clone();
            let monitors = monitors.clone();

            let handle = std::thread::Builder::new()
                .name(thread_name)
                .spawn(move || {
                    Self::rt_thread_main(nodes, running, tick_period, monitors, thread_cpus)
                })
                .map_err(|e| Error::Internal {
                    message: format!("Failed to spawn RT thread: {e}"),
                    file: file!(),
                    line: line!(),
                })?;

            handles.push(handle);
        }

        Ok(Self { handles })
    }

    /// Stop the RT executor and reclaim its nodes.
    ///
    /// The caller should have already set `running` to `false` before calling this.
    /// Each RT thread gets up to 3 seconds to exit cleanly. If a thread is stuck
    /// (stalled `tick()`, deadlock, infinite loop), it is detached after the timeout
    /// to prevent the entire scheduler shutdown from hanging.
    ///
    /// # Safety guarantee
    /// Shutdown always completes within `SHUTDOWN_TIMEOUT_PER_THREAD × num_threads`.
    /// A single stalled node cannot block the process from exiting.
    pub fn stop(mut self) -> Vec<RegisteredNode> {
        // The bounded-join loop lives in `primitives::join_with_timeout` so the
        // compute, event and async executors — which used to join unbounded —
        // back the same guarantee from one implementation.
        let mut all_nodes = Vec::new();
        for (i, handle) in std::mem::take(&mut self.handles).into_iter().enumerate() {
            if let Some(nodes) = super::primitives::join_with_timeout(
                handle,
                &format!("RT-thread {i}"),
                super::primitives::SHUTDOWN_TIMEOUT_PER_THREAD,
            ) {
                all_nodes.extend(nodes);
            }
        }
        all_nodes
    }

    /// Process a single node tick with all infrastructure (stats, profiler, budget, deadline).
    ///
    /// Extracted from the RT loop body so the caller can wrap this in `catch_unwind`.
    /// If this function panics, the caller marks the node as stopped.
    fn tick_node(
        node: &mut RegisteredNode,
        monitors: &SharedMonitors,
        running: &Arc<AtomicBool>,
        is_first_tick: bool,
    ) {
        // Failure-policy backoff (Restart) / cooldown (Skip): skip this tick
        // while the node is suppressed.
        if !node.failure_policy_allows_tick() {
            return;
        }

        // Update last tick time
        if node.rate_hz.is_some() {
            node.last_tick = Some(Instant::now());
        }

        // Begin recording tick (before execution)
        if let Some(ref mut recorder) = node.recorder {
            recorder.begin_tick(0); // RT thread has no global tick counter

            // Capture this node's inputs (subscriber topics) into the snapshot,
            // mirroring the single-threaded scheduler path. Without this, RT-node
            // recordings held only tick/timestamp metadata with empty payloads, so
            // `horus record export` produced metadata-only output. Gated on an
            // active recording tick, so there is zero cost when not recording.
            if recorder.is_active_tick() {
                let subscribers =
                    crate::communication::topic_node_registry().subscribers_for_node(&node.name);
                if !subscribers.is_empty() {
                    let topics_dir = crate::memory::platform::shm_topics_dir();
                    for sub in &subscribers {
                        let topic_path = topics_dir.join(&sub.topic_name);
                        if let Some(slot_read) =
                            crate::communication::read_latest_slot_bytes(&topic_path, 0)
                        {
                            recorder.record_input(&sub.topic_name, slot_read.payload);
                        }
                    }
                }
            }
        }

        // Start tick timing in context (required for record_tick() to increment counter)
        if let Some(ref mut ctx) = node.context {
            ctx.start_tick();
        }

        // Enter the allocation-free context if `.no_alloc()` is set — but NOT on
        // the node's first tick, so one-time lazy initialization (e.g. a `Topic`'s
        // SHM backend initializing on its first recv/send) may allocate. Alloc-
        // freedom is a steady-state guarantee, enforced from the second tick on.
        let enforce_no_alloc = node.no_alloc && !is_first_tick;

        // FIX #5: install the per-tick thread-local context (horus::now/dt/
        // elapsed/rng/budget_remaining) on THIS RT thread, mirroring
        // run_node_tick. Set BEFORE entering the alloc-free guard — the first
        // set_tick_context allocates its RNG + node-name string, which must not
        // trip `.no_alloc()`.
        super::primitives::set_node_tick_context(node, &*monitors.clock, monitors.tick_period);

        if enforce_no_alloc {
            crate::memory::rt_allocator::enter_rt_context(&node.name);
        }

        // Execute tick via NodeRunner
        let tr = NodeRunner::run_tick(&mut node.node);

        // Leave the allocation-free context
        if enforce_no_alloc {
            crate::memory::rt_allocator::leave_rt_context();
        }

        // Clear the per-tick context (mirror run_node_tick's clear right after
        // the tick; the following post-tick bookkeeping does not use it).
        super::primitives::clear_node_tick_context();

        // Record execution stats
        if let Some(ref mut stats) = node.rt_stats {
            stats.record_execution(tr.duration);
        }

        // Profiler recording (shared with main thread)
        // Use try_lock to avoid priority inversion — skip if contended
        if let Ok(mut profiler) = monitors.profiler.try_lock() {
            profiler.record(&node.name, tr.duration);
        }

        // Record in node recorder
        if let Some(ref mut recorder) = node.recorder {
            recorder.end_tick(tr.duration.as_nanos() as u64);
        }

        // tick budget check via TimingEnforcer
        if let Some(tick_budget) = node.tick_budget {
            // Budget overrun feeds the monitor too — same reasoning as the
            // deadline path below: this node is not in `Scheduler::nodes`, so
            // the main loop's budget accounting never sees it.
            if let Some(ref monitor) = monitors.safety {
                let _ = monitor.check_tick_budget(&node.name, tr.duration);
            }
            if let Some(budget_result) =
                TimingEnforcer::check_tick_budget(&node.name, tr.duration, tick_budget)
            {
                if monitors.verbose {
                    if let Some(hidden) = node.diag.allow(Diag::BudgetViolation, monotonic_nanos())
                    {
                        print_line(&format!(
                            "[RT-thread] budget violation in '{}': {:?} > {:?}{}",
                            node.name,
                            budget_result.violation.actual(),
                            budget_result.violation.budget(),
                            suppressed_suffix(hidden)
                        ));
                    }
                }
                if let Some(ref mut stats) = node.rt_stats {
                    stats.record_budget_violation();
                }
                // Record to blackbox (try_lock to avoid RT priority inversion)
                if let Some(ref bb) = monitors.blackbox {
                    if let Ok(mut bb) = bb.try_lock() {
                        bb.record(super::blackbox::BlackBoxEvent::BudgetViolation {
                            name: node.name.to_string(),
                            budget_us: tick_budget.as_micros() as u64,
                            actual_us: tr.duration.as_micros() as u64,
                        });
                    }
                }
                // Budget enforcement based on per-node policy.
                // Post-tick enforcement (safe — tick completed, no shared state issues).
                use super::safety_monitor::BudgetPolicy;
                match node.budget_policy {
                    BudgetPolicy::Warn => {
                        // Default: log only (already logged above)
                    }
                    BudgetPolicy::Enforce => {
                        // Stop node if tick exceeded 2x budget
                        if tr.duration > tick_budget * 2 {
                            print_line(&format!(
                                "[RT-thread] BUDGET ENFORCE: '{}' exceeded 2x budget ({:?} > {:?}) — node stopped",
                                node.name, tr.duration, tick_budget * 2
                            ));
                            let _ = node.node.shutdown();
                            node.is_stopped = true;
                        }
                    }
                    BudgetPolicy::EmergencyStop => {
                        // Any budget violation triggers e-stop
                        print_line(&format!(
                            "[RT-thread] BUDGET E-STOP: '{}' budget violation ({:?} > {:?})",
                            node.name, tr.duration, tick_budget
                        ));
                        let _ = node.node.shutdown();
                        node.is_stopped = true;
                        // Latch the SafetyMonitor before signalling the thread to
                        // exit. `running.store(false)` alone is a plain shutdown
                        // flag: it leaves get_state() reporting Normal, writes no
                        // blackbox EmergencyStop, and never populates
                        // PENDING_LOCAL_ESTOP — so horus_net had nothing to
                        // broadcast and peer robots were never told this one
                        // emergency-stopped.
                        if let Some(ref estop) = monitors.estop {
                            estop.trigger(format!(
                                "RT node '{}' budget violation ({:?} > {:?}) with BudgetPolicy::EmergencyStop",
                                node.name, tr.duration, tick_budget
                            ));
                        }
                        // Signal stop via running flag — RT thread will exit
                        running.store(false, Ordering::SeqCst);
                    }
                }
            }
        }

        // Deadline check via TimingEnforcer
        if let Some(deadline) = node.deadline {
            let miss = TimingEnforcer::check_deadline(tr.tick_start, deadline, node.miss_policy);
            if miss.is_none() && node.in_safe_mode {
                // Met the deadline again: clear the latch so a node that
                // recovers can be safed once more if it degrades later.
                // A permanent latch would silently disable the policy after
                // the first miss.
                node.in_safe_mode = false;
                if let Some(hidden) = node.diag.allow(Diag::SafeStateLeave, monotonic_nanos()) {
                    print_line(&format!(
                        "[RT-thread] SafeMode: '{}' met its deadline again, leaving safe state{}",
                        node.name,
                        suppressed_suffix(hidden)
                    ));
                }
            }
            if let Some(dm) = miss {
                if monitors.verbose {
                    if let Some(hidden) = node.diag.allow(Diag::DeadlineMiss, monotonic_nanos()) {
                        print_line(&format!(
                            "[RT-thread] Deadline miss in '{}': {:?} > {:?}{}",
                            node.name,
                            dm.elapsed,
                            dm.deadline,
                            suppressed_suffix(hidden)
                        ));
                    }
                }
                if let Some(ref mut stats) = node.rt_stats {
                    stats.record_deadline_miss();
                }
                // Aggregate into the scheduler's SafetyMonitor.
                //
                // `stats.record_deadline_miss()` above is NODE-LOCAL. The
                // monitor-side accounting — `max_deadline_misses` and the
                // graduated degradation ladder — lives in the main loop's
                // `check_timing_violations`, gated on `is_rt_node`. But the class
                // partition moves every RT node OUT of `Scheduler::nodes` and
                // into this executor, leaving only BestEffort nodes behind, for
                // which that gate is false. So `.rate()` — the very thing that
                // makes a node RT — guaranteed its misses were never counted:
                // the ceiling only ever worked in deterministic/replay mode,
                // where the partition does not happen.
                if let Some(ref monitor) = monitors.safety {
                    monitor.record_deadline_miss(&node.name);
                    let consecutive = monitor.consecutive_misses(&node.name);
                    let action =
                        monitor.evaluate_degradation(&node.name, consecutive, node.rate_hz);
                    if !matches!(action, super::safety_monitor::DegradationAction::None) {
                        print_line(&format!(
                            "[RT-thread] Degradation for '{}': {:?} after {} consecutive misses",
                            node.name, action, consecutive
                        ));

                        // Actually apply it. This used to only PRINT, with a
                        // comment claiming the deadline-action dispatch below
                        // handled anything stronger — it does not: that dispatch
                        // reads `dm.action` (the per-node Miss policy) and knows
                        // nothing about the degradation ladder. So ReduceRate,
                        // Isolate and Kill were computed every time and thrown
                        // away, and the documented graceful-degradation
                        // behaviour never happened for RT nodes — the ones it
                        // exists to protect.
                        super::primitives::apply_degradation_action(node, action, monitors);
                    }
                }
                // Record to blackbox (try_lock to avoid RT priority inversion)
                if let Some(ref bb) = monitors.blackbox {
                    if let Ok(mut bb) = bb.try_lock() {
                        bb.record(super::blackbox::BlackBoxEvent::DeadlineMiss {
                            name: node.name.to_string(),
                            deadline_us: deadline.as_micros() as u64,
                            actual_us: dm.elapsed.as_micros() as u64,
                        });
                    }
                }
                match dm.action {
                    DeadlineAction::Warn => {}
                    DeadlineAction::Skip => {
                        node.is_paused = true;
                    }
                    DeadlineAction::SafeMode => {
                        // Fire the transition once, on the way in. Calling it
                        // on every miss meant a node sleeping 5 ms against a
                        // 1 ms deadline entered its safe state 18 times across
                        // 17 ticks — alternating between commanding and safing
                        // for as long as the overload lasted.
                        if !node.in_safe_mode {
                            node.in_safe_mode = true;
                            if let Some(hidden) =
                                node.diag.allow(Diag::SafeStateEnter, monotonic_nanos())
                            {
                                print_line(&format!(
                                    "[RT-thread] SafeMode: '{}' entering safe state after deadline miss{}",
                                    node.name,
                                    suppressed_suffix(hidden)
                                ));
                            }
                            node.node.enter_safe_state();
                        }
                    }
                    DeadlineAction::EmergencyStop => {
                        print_line(&format!(
                            "[RT-thread] Emergency stop triggered by '{}'",
                            node.name
                        ));
                        // See the budget branch: without this the e-stop is a
                        // silent local shutdown that the fleet never hears about.
                        if let Some(ref estop) = monitors.estop {
                            estop.trigger(format!(
                                "RT node '{}' deadline miss escalated to emergency stop",
                                node.name
                            ));
                        }
                        running.store(false, Ordering::SeqCst);
                    }
                }
            }
        }

        // Handle tick result
        match tr.result {
            Ok(_) => {
                if let Some(ref mut ctx) = node.context {
                    ctx.record_tick();
                }
                node.record_tick_success();
                // FIX #2: feed the watchdog AFTER a successful tick. Mirrors the
                // main loop's critical-node condition (mod.rs:3555): is_rt_node
                // OR an explicit `.watchdog()`. Feeding in the Ok arm (rather than
                // before the tick) preserves hang-detection — a hung tick never
                // returns and a panicking tick lands in Err(_), so neither
                // refreshes the watchdog, and the main-thread `check_watchdogs`
                // still trips expiry. Feeding the wrong (non-critical) node is a
                // harmless no-op (no watchdog registered for it).
                if node.is_rt_node || node.node_watchdog.is_some() {
                    if let Some(ref feeder) = monitors.watchdog {
                        feeder.feed(&node.name);
                    }
                }

                // Degradation RECOVERY. `record_successful_tick` is the only
                // producer of RestoreRate and Deisolate, and its only other
                // call site is the main loop's `process_tick_result`, which
                // post-partition sees BestEffort nodes only. Without this an
                // RT node that had been rate-reduced or isolated stayed that
                // way for the life of the process even after the transient
                // condition cleared — `recovery_ticks` was dead configuration.
                if let Some(ref monitor) = monitors.safety {
                    let action = monitor.record_successful_tick(&node.name);
                    super::primitives::apply_degradation_action(node, action, monitors);
                }
            }
            Err(panic_err) => {
                // Use try_lock to avoid priority inversion — skip if contended
                if let Ok(mut profiler) = monitors.profiler.try_lock() {
                    profiler.record_node_failure(&node.name);
                }
                let error_msg = if let Some(s) = panic_err.downcast_ref::<&str>() {
                    format!("[RT-thread] Node '{}' panicked: {}", node.name, s)
                } else if let Some(s) = panic_err.downcast_ref::<String>() {
                    format!("[RT-thread] Node '{}' panicked: {}", node.name, s)
                } else {
                    format!("[RT-thread] Node '{}' panicked (unknown)", node.name)
                };
                // Count the failure. Only the Ok arm above touched the metrics,
                // so a node panicking on every tick reported
                // `Health: Healthy, Errors: 0, Total Ticks: 0` to both
                // `get_node_stats()` and `horus node info` — while the P99
                // timing next to it was recorded correctly, which made the zero
                // read as "this node is idle" rather than "this node is dead".
                //
                // `record_tick_failure` increments total_ticks and failed_ticks
                // and is what the main-thread path has always used; the RT path
                // simply never called it.
                if let Some(ref mut ctx) = node.context {
                    ctx.record_tick_failure(error_msg.clone());
                }

                // Reflect sustained failure in the node's health state. The
                // watchdog/deadline ladder is the only other writer, so a node that
                // panicked every tick but never missed a *timing* target reported
                // `Health: Healthy` forever — 232 errors out of 237 ticks, green.
                //
                // One panic can be transient; 3 consecutive is a state change.
                if let Some(ref ctx) = node.context {
                    if ctx.consecutive_failures() >= super::primitives::FAILURES_BEFORE_UNHEALTHY {
                        node.health_state
                            .store(super::types::NodeHealthState::Unhealthy);
                        monitors.node_controls.set_health(
                            node.name.as_ref(),
                            super::types::NodeHealthState::Unhealthy,
                        );
                    }
                }

                // `record_tick_failure` above already logged this at error level,
                // which reaches both the console and the buffer `horus log`
                // reads. This second copy was gated on `verbose` on the theory
                // that verbose is opt-in — but `MonitoringConfig::verbose`
                // defaults to *true*, so every default run printed the panic
                // twice on two different streams (hlog to stderr, this to
                // stdout), and a third time via the old `Node::on_error`
                // default. With a Python node's traceback attached that is three
                // multi-line blocks for one failure.

                // Record to the blackbox (try_lock to avoid RT priority
                // inversion, matching the budget/deadline paths above).
                //
                // Without this the flight recorder could not record a crash —
                // its single defining function. `BlackBoxEvent::NodeError` was
                // constructed only in blackbox.rs's own unit tests, so
                // `horus blackbox -e NodeError` after a panic returned
                // "No blackbox events found" while `--help` advertised
                // filtering by exactly that event.
                if let Some(ref bb) = monitors.blackbox {
                    if let Ok(mut bb) = bb.try_lock() {
                        bb.record(super::blackbox::BlackBoxEvent::NodeError {
                            name: node.name.to_string(),
                            error: error_msg.clone(),
                            severity: crate::error::Severity::Fatal,
                        });
                    }
                }

                // Call on_error handler
                node.node.on_error(&error_msg);

                // Enforce the failure policy. Fatal → safe the faulted node and
                // stop the whole scheduler via the shared `running` flag (the
                // same cross-thread mechanism as a deadline EmergencyStop); the
                // main run-loop teardown then shutdown()s every node. Restart →
                // re-init. Skip/Ignore → gated by should_allow on the next tick.
                if node.apply_failure_policy_after_panic() {
                    running.store(false, Ordering::SeqCst);
                }
            }
        }

        // Update live SHM registry AFTER record_tick() so tick_count is current
        monitors.update_registry(node, tr.duration.as_nanos() as u64);
    }

    /// Main function for the RT thread.
    ///
    /// Attempts SCHED_FIFO RT priority, then runs a tight tick loop executing
    /// each RT node sequentially via `NodeRunner::run_tick()`.
    fn rt_thread_main(
        mut nodes: Vec<RegisteredNode>,
        running: Arc<AtomicBool>,
        tick_period: Duration,
        monitors: SharedMonitors,
        rt_cpus: Vec<usize>,
    ) -> Vec<RegisteredNode> {
        // Use per-node priority if any node in this chain has one, otherwise default 80
        let thread_priority = nodes
            .iter()
            .filter_map(|n| n.os_priority)
            .max()
            .unwrap_or(80);

        // Try SCHED_DEADLINE first if any node in this chain requested it
        let use_deadline = nodes.iter().any(|n| n.use_sched_deadline);
        let mut deadline_active = false;
        if use_deadline {
            // Derive kernel params from the fastest node's budget/rate
            if let Some(node) = nodes
                .iter()
                .find(|n| n.tick_budget.is_some() && n.rate_hz.is_some())
            {
                let runtime_ns = node.tick_budget.unwrap().as_nanos() as u64;
                let period_ns = (1_000_000_000.0 / node.rate_hz.unwrap()) as u64;
                let deadline_ns = node
                    .deadline
                    .map(|d| d.as_nanos() as u64)
                    .unwrap_or(period_ns);
                match horus_sys::rt::set_deadline_scheduling(runtime_ns, deadline_ns, period_ns) {
                    Ok(()) => {
                        if monitors.verbose {
                            print_line(&format!(
                                "[RT-thread] SCHED_DEADLINE active (runtime={}us deadline={}us period={}us)",
                                runtime_ns / 1000,
                                deadline_ns / 1000,
                                period_ns / 1000,
                            ));
                        }
                        deadline_active = true;
                    }
                    Err(e) => {
                        print_line(&format!(
                            "[RT-thread] SCHED_DEADLINE failed: {} — falling back to SCHED_FIFO",
                            e
                        ));
                    }
                }
            }
        }

        // SCHED_FIFO fallback (also the default when SCHED_DEADLINE not requested).
        //
        // Whether this SUCCEEDS is load-bearing, not cosmetic: Linux RT
        // bandwidth control only polices SCHED_FIFO/SCHED_DEADLINE threads, so
        // the tick loop's CPU draw only matters once one of them is actually
        // granted. Without CAP_SYS_NICE this fails and the thread stays
        // SCHED_OTHER — which is exactly why the busy-wait defect this file
        // used to have was invisible on developer machines and in CI. Track it
        // so `CyclicWaiter` can warn in the configuration that bites.
        let mut rt_policy_active = deadline_active;
        if !deadline_active {
            match super::rt::set_realtime_priority(thread_priority) {
                Ok(()) => rt_policy_active = true,
                Err(e) => {
                    print_line(&format!(
                        "[RT-thread] Could not set SCHED_FIFO: {} (continuing with normal priority)",
                        e
                    ));
                }
            }
        }

        // Per-node core override: if any node in this chain specifies .core(), use that
        let effective_cpus = {
            let node_core = nodes.iter().filter_map(|n| n.pinned_core).next();
            if let Some(core) = node_core {
                vec![core]
            } else {
                rt_cpus.clone()
            }
        };

        // Pin to recommended RT CPU(s) to avoid cache thrashing and timer interrupts
        let rt_cpus = effective_cpus;
        if !rt_cpus.is_empty() {
            match super::rt::set_thread_affinity(&rt_cpus) {
                Ok(()) => {
                    if monitors.verbose {
                        print_line(&format!("[RT-thread] Pinned to CPU(s) {:?}", rt_cpus));
                    }
                }
                Err(e) => {
                    print_line(&format!(
                        "[RT-thread] Could not pin to CPU(s) {:?}: {} (continuing unpinned)",
                        rt_cpus, e
                    ));
                }
            }
        }

        // Lock CPU governor to "performance" on pinned cores (prevents frequency scaling jitter)
        if !rt_cpus.is_empty() {
            for &cpu in &rt_cpus {
                match horus_sys::rt::set_cpu_governor(cpu, "performance") {
                    Ok(()) => {
                        if monitors.verbose {
                            print_line(&format!("[RT] CPU {} governor → performance", cpu));
                        }
                    }
                    Err(e) => {
                        print_line(&format!(
                            "[RT] Could not set CPU {} governor: {} (continuing)",
                            cpu, e
                        ));
                    }
                }
            }

            // Move hardware interrupts off RT cores (prevents IRQ jitter)
            match horus_sys::rt::move_irqs_off_cpus(&rt_cpus) {
                Ok(n) if n > 0 && monitors.verbose => {
                    print_line(&format!("[RT] Moved {} IRQs off core(s) {:?}", n, rt_cpus));
                }
                Err(e) if monitors.verbose => {
                    print_line(&format!("[RT] Could not move IRQs: {}", e));
                }
                _ => {}
            }
        }

        // Pre-fault 64KB of stack to avoid page faults during first ticks
        crate::core::rt_config::prefault_stack(64 * 1024);
        if monitors.verbose {
            print_line("[RT-thread] Pre-faulted 64KB of stack");
        }

        if monitors.verbose {
            print_line(&format!(
                "[RT-thread] Started with {} nodes, tick period {:?}",
                nodes.len(),
                tick_period
            ));
        }

        // Per-node "has completed at least one tick" flags, aligned with `nodes`.
        // `.no_alloc()` enforcement is skipped on a node's FIRST tick so one-time
        // lazy initialization (e.g. a `Topic`'s SHM backend initializing on its
        // first recv/send) may allocate; steady-state ticks are then enforced.
        let mut warmed = vec![false; nodes.len()];

        // Cyclic cadence on an absolute phase grid. Constructed here, after all
        // the one-time setup above, so the phase anchor is not polluted by
        // affinity/governor/IRQ work that only happens once.
        let mut waiter = CyclicWaiter::new(tick_period, rt_policy_active, monitors.verbose);

        while running.load(Ordering::Relaxed) {
            let loop_start = Instant::now();

            for (idx, node) in nodes.iter_mut().enumerate() {
                if !node.initialized || node.is_stopped {
                    continue;
                }

                // Safing requested by the main thread's watchdog ladder. Runs
                // before the pause/stop gates: an Isolated node must reach its
                // safe state even if it is also about to be stopped.
                super::primitives::honor_safe_state_request(node, &monitors);

                // Check shared control flags (set by CLI: horus node pause/kill)
                if monitors.node_controls.is_stopped(node.name.as_ref()) {
                    node.is_stopped = true;
                    continue;
                }
                if monitors.node_controls.is_paused(node.name.as_ref()) {
                    continue; // Skip tick but don't auto-unpause
                }

                // Auto-unpause (Miss::Skip skips one tick)
                if node.is_paused {
                    node.is_paused = false;
                    continue;
                }

                // Per-node rate limiting.
                //
                // Tolerance is essential, not cosmetic: `loop_start` is sampled
                // at the top of the loop (line ~561) but `last_tick` is stamped
                // *inside* tick_node (after the gate), so the measured
                // `loop_start - last_tick` for an on-time tick lands a hair under
                // one period. With a strict `elapsed < period` and a loop that
                // sleeps exactly `tick_period` (== the fastest node's period),
                // the boundary tick was rejected and the node fired only every
                // *other* loop — halving the effective rate of every RT node.
                //
                // Accept the tick once we are within half a loop period of the
                // target. The tolerance is bounded by tick_period/2 (at most half
                // the fastest node's period), so a slower node can never fire a
                // full period early — it just stops losing its boundary tick.
                if let Some(rate_hz) = node.rate_hz {
                    if rate_hz > 0.0 {
                        if let Some(last_tick) = node.last_tick {
                            let elapsed = loop_start.duration_since(last_tick).as_secs_f64();
                            let period = 1.0 / rate_hz;
                            let tolerance = 0.5 * tick_period.as_secs_f64();
                            if elapsed < period - tolerance {
                                continue;
                            }
                        }
                    }
                }

                // Guard all infrastructure + tick processing against panics.
                // If ANY infrastructure code panics (recorder,
                // timing enforcer, etc.), the node is stopped but the RT thread
                // continues ticking remaining nodes.
                let is_first_tick = !warmed[idx];
                let infra_result = catch_unwind(AssertUnwindSafe(|| {
                    Self::tick_node(node, &monitors, &running, is_first_tick)
                }));
                warmed[idx] = true;

                match infra_result {
                    Ok(()) => {} // normal completion
                    Err(_) => {
                        if monitors.verbose {
                            // Avoid format!() heap allocation in RT path — write directly to stderr
                            crate::terminal::print_line(&format!(
                                "[RT-thread] Infrastructure panic for '{}' — node stopped",
                                node.name
                            ));
                        }
                        node.is_stopped = true;
                    }
                }
            }

            // Wait out the period on an ABSOLUTE deadline grid.
            //
            // What used to be here spun the CPU for the whole period at any
            // tick rate of 1 kHz or above, and computed its target as
            // `loop_start + tick_period` with `loop_start` re-sampled every
            // iteration — a busy-wait that RT bandwidth control punishes with a
            // ~50 ms dequeue, on top of unbounded phase drift. See the
            // `CyclicWaiter` section at the top of this file for the full
            // reasoning and for the median-jitter cost this trade accepts.
            waiter.wait();
        }

        waiter.finish(monitors.verbose);

        if monitors.verbose {
            print_line(&format!(
                "[RT-thread] Stopped ({} nodes returning to scheduler)",
                nodes.len()
            ));
        }

        nodes
    }
}

impl Drop for RtExecutor {
    fn drop(&mut self) {
        for handle in self.handles.drain(..) {
            let _ = handle.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::{Miss, Node};
    use std::sync::Mutex;

    fn test_monitors() -> SharedMonitors {
        SharedMonitors {
            profiler: Arc::new(Mutex::new(super::super::profiler::RuntimeProfiler::new())),
            blackbox: None,
            verbose: true,
            registry: None,
            registry_slots: Arc::new(std::collections::HashMap::new()),
            node_controls: Arc::new(super::super::types::NodeControlMap::default()),
            clock: Arc::new(crate::core::clock::WallClock::new()),
            tick_period: Duration::from_millis(1),
            watchdog: None,
            estop: None,
            safety: None,
        }
    }

    /// The executor half of the graduated watchdog ladder.
    ///
    /// The ladder runs on the main thread (the only one a hung node cannot
    /// block) and cannot call `enter_safe_state()` itself — that needs
    /// `&mut dyn Node`, which the executor owns. So Critical raises a flag in
    /// the shared control map, and the executor must consume it. If it does
    /// not, an executor-hosted node is marked Isolated and never actually
    /// safed: a worse outcome than no ladder, because the state says it was.
    #[test]
    fn test_executor_honors_safe_state_request() {
        use std::sync::atomic::{AtomicBool, AtomicU64};

        struct SafeableNode {
            safed: Arc<AtomicBool>,
        }
        impl Node for SafeableNode {
            fn name(&self) -> &str {
                "safeable"
            }
            fn tick(&mut self) {}
            fn enter_safe_state(&mut self) {
                self.safed.store(true, std::sync::atomic::Ordering::SeqCst);
            }
        }

        let safed = Arc::new(AtomicBool::new(false));
        let mut registered = make_rt_registered("safeable", Arc::new(AtomicU64::new(0)));
        registered.node = super::super::types::NodeKind::new(Box::new(SafeableNode {
            safed: safed.clone(),
        }));

        let monitors = test_monitors();
        monitors.node_controls.register("safeable");

        // Nothing pending → the node is left alone.
        super::super::primitives::honor_safe_state_request(&mut registered, &monitors);
        assert!(!safed.load(std::sync::atomic::Ordering::SeqCst));

        monitors.node_controls.request_safe_state("safeable");
        super::super::primitives::honor_safe_state_request(&mut registered, &monitors);
        assert!(
            safed.load(std::sync::atomic::Ordering::SeqCst),
            "the executor never called enter_safe_state despite a pending request"
        );

        // Honoured exactly once per raise, not on every subsequent pass.
        safed.store(false, std::sync::atomic::Ordering::SeqCst);
        super::super::primitives::honor_safe_state_request(&mut registered, &monitors);
        assert!(!safed.load(std::sync::atomic::Ordering::SeqCst));
    }

    /// The degradation ladder must actually be APPLIED on the executor, not
    /// just computed.
    ///
    /// `evaluate_degradation` was called here and its result only printed, with
    /// a comment claiming the deadline-action dispatch handled anything
    /// stronger — it does not: that dispatch reads the per-node `Miss` policy
    /// and knows nothing about the ladder. So `ReduceRate`, `Isolate` and
    /// `Kill` were produced on every deadline miss and thrown away, and the
    /// documented graceful-degradation behaviour never happened for RT nodes.
    #[test]
    fn test_degradation_action_is_applied_not_just_logged() {
        use super::super::safety_monitor::DegradationAction;
        use super::super::types::NodeHealthState;
        use std::sync::atomic::AtomicU64;

        let monitors = test_monitors();
        let mut node = make_rt_registered("degrader", Arc::new(AtomicU64::new(0)));
        node.rate_hz = Some(100.0);
        monitors.node_controls.register("degrader");

        // ReduceRate must land on the node, not just in the log.
        super::super::primitives::apply_degradation_action(
            &mut node,
            DegradationAction::ReduceRate {
                node: "degrader".to_string(),
                new_rate_hz: 25.0,
            },
            &monitors,
        );
        assert_eq!(
            node.rate_hz,
            Some(25.0),
            "ReduceRate was computed and discarded — the node still runs at full rate"
        );

        // Isolate marks health and safes the node, and the state is mirrored
        // into the shared map so the main thread sees the same value.
        super::super::primitives::apply_degradation_action(
            &mut node,
            DegradationAction::Isolate("degrader".to_string()),
            &monitors,
        );
        assert_eq!(node.health_state.load(), NodeHealthState::Isolated);
        assert_eq!(
            monitors.node_controls.health("degrader"),
            NodeHealthState::Isolated,
            "executor-side health must be visible to the main thread"
        );

        // Recovery is reachable: RestoreRate puts the rate and health back.
        super::super::primitives::apply_degradation_action(
            &mut node,
            DegradationAction::RestoreRate {
                node: "degrader".to_string(),
                original_rate_hz: 100.0,
            },
            &monitors,
        );
        assert_eq!(node.rate_hz, Some(100.0));
        assert_eq!(node.health_state.load(), NodeHealthState::Healthy);
        assert_eq!(
            monitors.node_controls.health("degrader"),
            NodeHealthState::Healthy
        );

        // Kill stops the node — the flag every executor honours.
        super::super::primitives::apply_degradation_action(
            &mut node,
            DegradationAction::Kill("degrader".to_string()),
            &monitors,
        );
        assert!(node.is_stopped, "Kill must actually remove the node");
    }

    /// `ReduceRate` must widen the node's watchdog by the same factor.
    ///
    /// The tick is what feeds the watchdog, so halving a node's rate halves how
    /// often it feeds. With a fixed timeout, the GENTLEST rung of the ladder
    /// becomes an escalation for any node whose watchdog margin was under 2x
    /// its period: 1x, 2x, then Critical — which latches a fleet-wide
    /// emergency stop. Rate-reducing a struggling node must not be a slower
    /// route to halting the robot.
    #[test]
    fn test_reduce_rate_widens_the_watchdog() {
        use super::super::safety_monitor::{DegradationAction, SafetyMonitor};
        use std::sync::atomic::AtomicU64;
        use std::time::Duration;

        let monitor = Arc::new(SafetyMonitor::new(100));
        // Watchdog margin of 1.5x the 100Hz period — survives at full rate,
        // would not survive a halving.
        monitor.add_critical_node("throttled".to_string(), Duration::from_millis(15));

        let mut monitors = test_monitors();
        monitors.safety = Some(monitor.clone());
        monitors.node_controls.register("throttled");

        let mut node = make_rt_registered("throttled", Arc::new(AtomicU64::new(0)));
        node.rate_hz = Some(100.0);

        super::super::primitives::apply_degradation_action(
            &mut node,
            DegradationAction::ReduceRate {
                node: "throttled".to_string(),
                new_rate_hz: 50.0,
            },
            &monitors,
        );
        assert_eq!(node.rate_hz, Some(50.0));
        assert_eq!(
            monitor.watchdog_timeout("throttled"),
            Some(Duration::from_millis(30)),
            "the watchdog window must widen with the halved tick rate, or the \
             node trips its own watchdog for slowing down as instructed"
        );

        // Restoring the rate restores the configured window exactly — the
        // scaling is relative to the configured value, so it cannot compound.
        super::super::primitives::apply_degradation_action(
            &mut node,
            DegradationAction::RestoreRate {
                node: "throttled".to_string(),
                original_rate_hz: 100.0,
            },
            &monitors,
        );
        assert_eq!(
            monitor.watchdog_timeout("throttled"),
            Some(Duration::from_millis(15))
        );
    }

    /// Watchdog-derived health must NOT gate ticking on an executor.
    ///
    /// The executor is what feeds the watchdog, so suppressing on watchdog
    /// health is self-reinforcing: Unhealthy → stops feeding → 3x → e-stop, on
    /// one transient overrun. Suppressing *and* feeding is worse still — it
    /// makes the 3x rung unreachable, capping the ladder one rung short of the
    /// safing it exists to perform. This pins the decision so neither variant
    /// gets reintroduced.
    #[test]
    fn test_unhealthy_rt_node_still_ticks() {
        use super::super::types::NodeHealthState;
        use std::sync::atomic::{AtomicU64, Ordering as AOrd};

        let count = Arc::new(AtomicU64::new(0));
        let mut node = make_rt_registered("slow", count.clone());
        node.health_state.store(NodeHealthState::Unhealthy);

        let monitors = test_monitors();
        RtExecutor::tick_node(&mut node, &monitors, &Arc::new(AtomicBool::new(true)), true);

        assert_eq!(
            count.load(AOrd::Relaxed),
            1,
            "an Unhealthy RT node must keep being ticked — it is the tick that \
             feeds its watchdog, so suppressing it escalates straight to e-stop"
        );
    }

    struct CounterNode {
        name: String,
        count: Arc<std::sync::atomic::AtomicU64>,
    }

    impl Node for CounterNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.count
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
    }

    fn make_rt_registered(name: &str, count: Arc<std::sync::atomic::AtomicU64>) -> RegisteredNode {
        use crate::core::NodeInfo;

        let node = CounterNode {
            name: name.to_string(),
            count,
        };
        RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(node)),
            name: Arc::from(name),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new(name.to_string())),
            rate_hz: None,
            last_tick: None,
            is_rt_node: true,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            health_probe_counter: 0,
            is_paused: false,
            diag: Default::default(),
            in_safe_mode: false,
            rt_stats: None,
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::Rt,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
            subscription_freshness: Vec::new(),
            use_sched_deadline: false,
            no_alloc: false,
        }
    }

    #[test]
    fn test_rt_executor_runs_nodes() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let nodes = vec![make_rt_registered("test_rt", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        // Let it run for a bit
        std::thread::sleep(50_u64.ms());

        // Stop
        running.store(false, Ordering::SeqCst);
        let returned_nodes = executor.stop();

        assert_eq!(returned_nodes.len(), 1);
        assert!(count.load(std::sync::atomic::Ordering::Relaxed) > 0);
    }

    /// Regression guard for the per-node rate gate.
    ///
    /// The gate samples `loop_start` before `last_tick` is stamped and the loop
    /// sleeps exactly one `tick_period`, so a strict `elapsed < period` rejected
    /// every on-time boundary tick and an RT node ran at *half* its configured
    /// rate. This asserts a 100 Hz node ticks close to 100/s, not ~50/s.
    ///
    /// The band [70, 135] over 1s cleanly separates correct (~100) from the old
    /// halving bug (~50) while tolerating scheduler jitter under CI load.
    #[test]
    fn test_rt_executor_honors_configured_rate_not_half() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let mut node = make_rt_registered("rate_100hz", count.clone());
        node.rate_hz = Some(100.0);
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start_pool(
            vec![vec![node]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(1000_u64.ms());
        running.store(false, Ordering::SeqCst);
        executor.stop();

        let ticks = count.load(std::sync::atomic::Ordering::Relaxed);
        assert!(
            ticks >= 70,
            "100 Hz node ticked only {ticks} times in ~1s — the rate gate is \
             halving (or worse-throttling) the configured rate"
        );
        assert!(
            ticks <= 135,
            "100 Hz node ticked {ticks} times in ~1s — gate is over-firing"
        );
    }

    #[test]
    fn test_rt_executor_respects_running_flag() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let nodes = vec![make_rt_registered("test_rt", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        // Stop immediately
        running.store(false, Ordering::SeqCst);
        let returned_nodes = executor.stop();

        assert_eq!(returned_nodes.len(), 1);
        // May have ticked 0 or a few times — that's fine
    }

    #[test]
    fn test_rt_executor_multiple_nodes() {
        let count1 = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count2 = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let nodes = vec![
            make_rt_registered("rt_node_1", count1.clone()),
            make_rt_registered("rt_node_2", count2.clone()),
        ];
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned_nodes = executor.stop();

        assert_eq!(returned_nodes.len(), 2);
        // Both nodes should have ticked
        assert!(count1.load(std::sync::atomic::Ordering::Relaxed) > 0);
        assert!(count2.load(std::sync::atomic::Ordering::Relaxed) > 0);
    }

    struct PanicNode;
    impl Node for PanicNode {
        fn name(&self) -> &str {
            "panic_rt"
        }
        fn tick(&mut self) {
            panic!("intentional RT panic");
        }
    }

    #[test]
    fn test_rt_executor_handles_panic() {
        use crate::core::NodeInfo;

        let node = PanicNode;
        let registered = RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(node)),
            name: Arc::from("panic_rt"),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new("panic_rt".to_string())),
            rate_hz: None,
            last_tick: None,
            is_rt_node: true,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            health_probe_counter: 0,
            is_paused: false,
            diag: Default::default(),
            in_safe_mode: false,
            rt_stats: None,
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::Rt,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
            subscription_freshness: Vec::new(),
            use_sched_deadline: false,
            no_alloc: false,
        };

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![registered]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        // The panic node should log errors and continue (no longer stops scheduler)
        std::thread::sleep(50_u64.ms());

        // Signal the executor to stop before calling stop() (which joins the thread)
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();
        assert_eq!(returned.len(), 1);
    }

    /// Node that just ticks normally -- used to verify siblings survive panicking nodes.
    struct SimpleCounterNode {
        name: String,
        count: Arc<std::sync::atomic::AtomicU64>,
    }

    impl Node for SimpleCounterNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.count
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
    }

    fn make_rt_with_rate(
        name: &str,
        count: Arc<std::sync::atomic::AtomicU64>,
        rate_hz: f64,
    ) -> RegisteredNode {
        let mut node = make_rt_registered(name, count);
        node.rate_hz = Some(rate_hz);
        node
    }

    #[test]
    fn test_multi_rate_timing_wheel() {
        // Simulate IMU at 1kHz and lidar at 10Hz on same RT thread
        let fast_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let slow_count = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let nodes = vec![
            make_rt_with_rate("imu_1khz", fast_count.clone(), 1000.0),
            make_rt_with_rate("lidar_10hz", slow_count.clone(), 10.0),
        ];
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            10_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        // Run for 200ms
        std::thread::sleep(200_u64.ms());
        running.store(false, Ordering::SeqCst);
        let _returned = executor.stop();

        let fast_ticks = fast_count.load(std::sync::atomic::Ordering::Relaxed);
        let slow_ticks = slow_count.load(std::sync::atomic::Ordering::Relaxed);

        // 1kHz for 200ms → ~200 ticks (very wide margin for debug builds on non-RT kernel)
        assert!(
            fast_ticks >= 10,
            "1kHz node should tick at least 10 times in 200ms, got {}",
            fast_ticks
        );

        // 10Hz for 200ms → ~2 ticks
        assert!(
            (1..=10).contains(&slow_ticks),
            "10Hz node should tick 1-10 times in 200ms, got {}",
            slow_ticks
        );

        // Fast node should tick significantly more than slow node
        assert!(
            fast_ticks > slow_ticks * 5,
            "1kHz node ({}) should tick >5x more than 10Hz node ({})",
            fast_ticks,
            slow_ticks
        );
    }

    #[test]
    fn test_multi_rate_three_rates() {
        // Three different rates on same RT thread: 500Hz, 100Hz, 20Hz
        let count_500 = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count_100 = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count_20 = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let nodes = vec![
            make_rt_with_rate("motor_500hz", count_500.clone(), 500.0),
            make_rt_with_rate("sensor_100hz", count_100.clone(), 100.0),
            make_rt_with_rate("planner_20hz", count_20.clone(), 20.0),
        ];
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            10_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        // Run for 200ms
        std::thread::sleep(200_u64.ms());
        running.store(false, Ordering::SeqCst);
        let _returned = executor.stop();

        let ticks_500 = count_500.load(std::sync::atomic::Ordering::Relaxed);
        let ticks_100 = count_100.load(std::sync::atomic::Ordering::Relaxed);
        let ticks_20 = count_20.load(std::sync::atomic::Ordering::Relaxed);

        // Verify ordering: 500Hz > 100Hz > 20Hz
        assert!(
            ticks_500 > ticks_100,
            "500Hz ({}) should tick more than 100Hz ({})",
            ticks_500,
            ticks_100
        );
        assert!(
            ticks_100 > ticks_20,
            "100Hz ({}) should tick more than 20Hz ({})",
            ticks_100,
            ticks_20
        );

        // Verify approximate ratios (with wide margins for non-RT kernel)
        // 500Hz should tick ~5x more than 100Hz
        assert!(
            ticks_500 >= ticks_100 * 2,
            "500Hz ({}) should be at least 2x of 100Hz ({})",
            ticks_500,
            ticks_100
        );
    }

    // ========================================================================
    // Stress tests for production hardening (RT safety & crash resilience)
    // ========================================================================

    /// Stress test: contended profiler lock should not block RT thread.
    ///
    /// A background thread holds the profiler Mutex for 50ms. Because the RT
    /// executor uses try_lock(), it should skip profiling without blocking.
    #[test]
    fn test_stress_contended_profiler_no_deadlock() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let monitors = test_monitors();
        let profiler_arc = monitors.profiler.clone();

        let nodes = vec![make_rt_registered("stress_profiler", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            1_u64.ms(),
            monitors,
            Vec::new(),
        )
        .unwrap();

        // Background thread holds profiler lock for 50ms
        let lock_thread = std::thread::spawn(move || {
            let _guard = profiler_arc.lock().unwrap();
            std::thread::sleep(50_u64.ms());
        });

        // Let RT thread run concurrently with the contended lock
        std::thread::sleep(100_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();
        lock_thread.join().unwrap();

        assert_eq!(returned.len(), 1);
        // RT thread should have been ticking throughout (not blocked)
        let ticks = count.load(std::sync::atomic::Ordering::Relaxed);
        assert!(
            ticks > 10,
            "RT thread should have ticked many times despite profiler contention, got {}",
            ticks
        );
    }

    /// Node that panics on every tick -- used for panic survival tests.
    struct AlwaysPanicNode {
        name: String,
        count: Arc<std::sync::atomic::AtomicU64>,
    }

    impl Node for AlwaysPanicNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.count
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            panic!("simulated persistent panic");
        }
    }

    /// Stress test: tick panic -- node logs error and continues, sibling unaffected.
    ///
    /// The tick panic is caught by NodeRunner::run_tick's catch_unwind. The error
    /// is logged and the node continues. The outer infrastructure catch_unwind
    /// handles panics in post-tick infrastructure (profiler, recorder,
    /// timing enforcer) -- not in the tick itself.
    #[test]
    fn test_stress_tick_panic_continues() {
        use crate::core::NodeInfo;

        let panic_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let normal_count = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let panic_node = AlwaysPanicNode {
            name: "always_panic".to_string(),
            count: panic_count.clone(),
        };
        let panic_registered = RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(panic_node)),
            name: Arc::from("always_panic"),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new("always_panic".to_string())),
            rate_hz: None,
            last_tick: None,
            is_rt_node: true,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            health_probe_counter: 0,
            is_paused: false,
            diag: Default::default(),
            in_safe_mode: false,
            rt_stats: None,
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::Rt,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
            subscription_freshness: Vec::new(),
            use_sched_deadline: false,
            no_alloc: false,
        };

        let normal_registered = make_rt_registered("survivor_node", normal_count.clone());

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![panic_registered, normal_registered]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(100_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        // RT thread should NOT have crashed
        assert_eq!(returned.len(), 2);

        // The panicking node should have attempted multiple ticks
        let panic_ticks = panic_count.load(std::sync::atomic::Ordering::Relaxed);
        assert!(
            panic_ticks > 0,
            "Panicking node should have been ticked (panic caught by NodeRunner)"
        );

        // Normal node should keep ticking regardless
        assert!(
            normal_count.load(std::sync::atomic::Ordering::Relaxed) > 10,
            "Survivor node should keep ticking despite sibling's repeated panics"
        );

        // ── Regression: a panicking node must not report itself healthy ─────
        //
        // Only the Ok arm recorded metrics, so a node panicking on *every* tick
        // reported `Health: Healthy, Errors: 0, Total Ticks: 0` to both
        // `get_node_stats()` and `horus node info` — while its P99 timing was
        // recorded correctly, which made the zero read as "idle" rather than
        // "dead". Monitoring, CI and launch supervisors all saw green.
        let panicked = returned
            .iter()
            .find(|n| &*n.name == "always_panic")
            .expect("panicking node returned from executor");
        let ctx = panicked
            .context
            .as_ref()
            .expect("panicking node keeps its context");

        assert!(
            ctx.metrics().failed_ticks() > 0,
            "failed_ticks must count panics; got {} after {} attempted ticks",
            ctx.metrics().failed_ticks(),
            panic_ticks
        );
        assert!(
            ctx.metrics().errors_count() > 0,
            "errors_count must count panics; got {}",
            ctx.metrics().errors_count()
        );
        assert!(
            ctx.metrics().total_ticks() > 0,
            "total_ticks must include failed ticks; got {}",
            ctx.metrics().total_ticks()
        );
        assert_eq!(
            ctx.metrics().successful_ticks(),
            0,
            "a node that panics every tick has no successful ticks"
        );

        // Health must reflect it. Previously the watchdog/deadline ladder was
        // the only writer of health_state, so a node panicking every tick but
        // never missing a *timing* target reported Healthy indefinitely — the
        // observed case was 232 errors out of 237 ticks, still green.
        assert_eq!(
            panicked.health_state.load(),
            super::super::types::NodeHealthState::Unhealthy,
            "a node failing every tick must not report Healthy"
        );
        assert!(
            ctx.consecutive_failures() >= super::super::primitives::FAILURES_BEFORE_UNHEALTHY,
            "consecutive_failures should have passed the threshold; got {}",
            ctx.consecutive_failures()
        );

        // The survivor's counters must stay clean — the fix must not attribute
        // one node's failures to another.
        let survivor = returned
            .iter()
            .find(|n| &*n.name == "survivor_node")
            .expect("survivor node returned from executor");
        assert_eq!(
            survivor.health_state.load(),
            super::super::types::NodeHealthState::Healthy,
            "the sibling's panics must not degrade a healthy node"
        );
        if let Some(ctx) = survivor.context.as_ref() {
            assert_eq!(
                ctx.metrics().failed_ticks(),
                0,
                "survivor must not inherit the sibling's failures"
            );
            assert_eq!(
                ctx.consecutive_failures(),
                0,
                "a healthy node's failure streak stays at zero"
            );
        }
    }

    /// Stress test: verbose=false suppresses RT thread output.
    ///
    /// We can't easily capture stdout in Rust tests, but we verify the executor
    /// runs correctly with verbose=false and doesn't crash.
    #[test]
    fn test_stress_verbose_false_runs_clean() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let monitors = SharedMonitors {
            profiler: Arc::new(Mutex::new(super::super::profiler::RuntimeProfiler::new())),
            blackbox: None,
            verbose: false,
            registry: None,
            registry_slots: Arc::new(std::collections::HashMap::new()),
            node_controls: Arc::new(super::super::types::NodeControlMap::default()),
            clock: Arc::new(crate::core::clock::WallClock::new()),
            tick_period: Duration::from_millis(1),
            watchdog: None,
            estop: None,
            safety: None,
        };

        let nodes = vec![make_rt_registered("quiet_node", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            1_u64.ms(),
            monitors,
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 1);
        assert!(
            count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "Node should still tick with verbose=false"
        );
    }

    /// Stress test: verbose=false with a panicking node.
    ///
    /// Combines verbose=false with a panicking tick to verify
    /// that the error logging path doesn't crash when verbose is disabled.
    #[test]
    fn test_stress_verbose_false_with_panic() {
        use crate::core::NodeInfo;

        let normal_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let panic_count = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let panic_node = AlwaysPanicNode {
            name: "quiet_panic".to_string(),
            count: panic_count,
        };
        let panic_registered = RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(panic_node)),
            name: Arc::from("quiet_panic"),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new("quiet_panic".to_string())),
            rate_hz: None,
            last_tick: None,
            is_rt_node: true,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            health_probe_counter: 0,
            is_paused: false,
            diag: Default::default(),
            in_safe_mode: false,
            rt_stats: None,
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::Rt,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
            subscription_freshness: Vec::new(),
            use_sched_deadline: false,
            no_alloc: false,
        };

        let normal_registered = make_rt_registered("quiet_normal", normal_count.clone());

        let monitors = SharedMonitors {
            profiler: Arc::new(Mutex::new(super::super::profiler::RuntimeProfiler::new())),
            blackbox: None,
            verbose: false,
            registry: None,
            registry_slots: Arc::new(std::collections::HashMap::new()),
            node_controls: Arc::new(super::super::types::NodeControlMap::default()),
            clock: Arc::new(crate::core::clock::WallClock::new()),
            tick_period: Duration::from_millis(1),
            watchdog: None,
            estop: None,
            safety: None,
        };

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![panic_registered, normal_registered]],
            running.clone(),
            1_u64.ms(),
            monitors,
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(100_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 2);
        assert!(
            normal_count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "Normal node should tick even with verbose=false and sibling panic"
        );
    }

    // ========================================================================
    // Multi-chain parallel execution tests
    // ========================================================================

    #[test]
    fn test_multi_chain_independent_execution() {
        // Two chains on separate threads — both must tick independently
        let count_a = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count_b = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let chain_0 = vec![make_rt_registered("chain0_node", count_a.clone())];
        let chain_1 = vec![make_rt_registered("chain1_node", count_b.clone())];

        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start_pool(
            vec![chain_0, chain_1],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(Duration::from_millis(50));
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        let ticks_a = count_a.load(std::sync::atomic::Ordering::Relaxed);
        let ticks_b = count_b.load(std::sync::atomic::Ordering::Relaxed);

        assert!(ticks_a > 0, "chain 0 must tick: got {}", ticks_a);
        assert!(ticks_b > 0, "chain 1 must tick: got {}", ticks_b);
        assert_eq!(returned.len(), 2, "both nodes returned on stop");
    }

    #[test]
    fn test_multi_chain_panic_in_one_does_not_block_other() {
        // Chain 0 has a panicking node. Chain 1 must still tick.
        struct PanicNode;
        impl Node for PanicNode {
            fn name(&self) -> &str {
                "panic_node"
            }
            fn tick(&mut self) {
                panic!("intentional panic in chain 0");
            }
        }

        let healthy_count = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let panic_node = RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(PanicNode)),
            name: Arc::from("panic_node"),
            priority: 0,
            initialized: true,
            context: Some(crate::core::NodeInfo::new("panic_node".to_string())),
            rate_hz: None,
            last_tick: None,
            is_rt_node: true,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            health_probe_counter: 0,
            is_paused: false,
            diag: Default::default(),
            in_safe_mode: false,
            rt_stats: None,
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::Rt,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
            subscription_freshness: Vec::new(),
            use_sched_deadline: false,
            no_alloc: false,
        };

        let chain_0 = vec![panic_node];
        let chain_1 = vec![make_rt_registered("healthy", healthy_count.clone())];

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![chain_0, chain_1],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(Duration::from_millis(50));
        running.store(false, Ordering::SeqCst);
        let _ = executor.stop();

        let healthy_ticks = healthy_count.load(std::sync::atomic::Ordering::Relaxed);
        assert!(
            healthy_ticks > 0,
            "healthy chain must tick despite panic in other chain: got {}",
            healthy_ticks
        );
    }

    #[test]
    fn test_multi_chain_three_chains_all_tick() {
        let counts: Vec<_> = (0..3)
            .map(|_| Arc::new(std::sync::atomic::AtomicU64::new(0)))
            .collect();

        let chains: Vec<Vec<RegisteredNode>> = counts
            .iter()
            .enumerate()
            .map(|(i, c)| vec![make_rt_registered(&format!("node_{}", i), c.clone())])
            .collect();

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            chains,
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(Duration::from_millis(50));
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        for (i, c) in counts.iter().enumerate() {
            let ticks = c.load(std::sync::atomic::Ordering::Relaxed);
            assert!(ticks > 0, "chain {} must tick: got {}", i, ticks);
        }
        assert_eq!(returned.len(), 3);
    }

    // ========================================================================
    // Budget policy enforcement tests
    // ========================================================================

    struct SlowNode {
        name: String,
        count: Arc<std::sync::atomic::AtomicU64>,
        sleep_us: u64,
    }

    impl Node for SlowNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.count
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            std::thread::sleep(Duration::from_micros(self.sleep_us));
        }
    }

    fn make_slow_rt_node(
        name: &str,
        count: Arc<std::sync::atomic::AtomicU64>,
        sleep_us: u64,
        budget: Duration,
        policy: super::super::safety_monitor::BudgetPolicy,
    ) -> RegisteredNode {
        use crate::core::NodeInfo;

        let node = SlowNode {
            name: name.to_string(),
            count,
            sleep_us,
        };
        RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(node)),
            name: Arc::from(name),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new(name.to_string())),
            rate_hz: Some(1000.0),
            last_tick: None,
            is_rt_node: true,
            tick_budget: Some(budget),
            deadline: None,
            recorder: None,
            is_stopped: false,
            health_probe_counter: 0,
            is_paused: false,
            diag: Default::default(),
            in_safe_mode: false,
            rt_stats: Some(crate::core::RtStats::default()),
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::Rt,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: policy,
            subscription_freshness: Vec::new(),
            use_sched_deadline: false,
            no_alloc: false,
        }
    }

    #[test]
    fn test_budget_policy_warn_does_not_stop_node() {
        use super::super::safety_monitor::BudgetPolicy;

        // Node sleeps 500μs with 100μs budget (5x over) but policy=Warn → keeps running
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let node = make_slow_rt_node(
            "warn_node",
            count.clone(),
            500,
            Duration::from_micros(100),
            BudgetPolicy::Warn,
        );

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![node]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(Duration::from_millis(50));
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        let ticks = count.load(std::sync::atomic::Ordering::Relaxed);
        // With Warn policy, node should tick multiple times (not stopped)
        assert!(
            ticks > 1,
            "Warn policy should NOT stop node — got {} ticks",
            ticks
        );
        // Node should still be alive (not stopped)
        assert!(
            !returned[0].is_stopped,
            "Warn policy should not set is_stopped"
        );
    }

    #[test]
    fn test_budget_policy_enforce_stops_node_on_2x_overrun() {
        use super::super::safety_monitor::BudgetPolicy;

        // Node sleeps 500μs with 100μs budget (5x > 2x threshold) and policy=Enforce → stopped
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let node = make_slow_rt_node(
            "enforce_node",
            count.clone(),
            500,
            Duration::from_micros(100),
            BudgetPolicy::Enforce,
        );

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![node]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(Duration::from_millis(50));
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        // Node should have been stopped after first tick (5x > 2x budget)
        assert!(
            returned[0].is_stopped,
            "Enforce policy should stop node that exceeds 2x budget"
        );
    }

    #[test]
    fn test_budget_policy_emergency_stop_halts_executor() {
        use super::super::safety_monitor::BudgetPolicy;

        // Node sleeps 500μs with 100μs budget and policy=EmergencyStop → halts everything
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let node = make_slow_rt_node(
            "estop_node",
            count.clone(),
            500,
            Duration::from_micros(100),
            BudgetPolicy::EmergencyStop,
        );

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![node]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        // Give it enough time to detect and e-stop
        std::thread::sleep(Duration::from_millis(50));
        // running should be false (set by e-stop)
        assert!(
            !running.load(Ordering::SeqCst),
            "EmergencyStop policy should set running=false"
        );

        let returned = executor.stop();
        assert!(
            returned[0].is_stopped,
            "EmergencyStop policy should stop the node"
        );
    }

    // ========================================================================
    // Edge case tests
    // ========================================================================

    /// Empty executor (zero chains) — start_pool and stop complete without error.
    #[test]
    fn test_empty_executor_no_chains() {
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start_pool(
            Vec::new(),
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();
        assert!(returned.is_empty(), "no chains → no nodes returned");
    }

    /// Single chain with a single empty chain (zero nodes in chain).
    #[test]
    fn test_single_empty_chain() {
        let running = Arc::new(AtomicBool::new(true));

        let executor = RtExecutor::start_pool(
            vec![Vec::new()],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        // Let the empty RT thread loop briefly
        std::thread::sleep(20_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();
        assert!(returned.is_empty(), "empty chain → no nodes returned");
    }

    /// Single chain with exactly one node — verifies the degenerate-chain case.
    #[test]
    fn test_single_chain_single_node() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let node = make_rt_registered("solo_node", count.clone());

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![node]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(30_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 1);
        assert_eq!(&*returned[0].name, "solo_node");
        assert!(
            count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "single node must tick at least once"
        );
    }

    /// Chain with many nodes (12) — all are ticked and returned.
    #[test]
    fn test_chain_with_many_nodes() {
        let counts: Vec<_> = (0..12)
            .map(|_| Arc::new(std::sync::atomic::AtomicU64::new(0)))
            .collect();

        let nodes: Vec<RegisteredNode> = counts
            .iter()
            .enumerate()
            .map(|(i, c)| make_rt_registered(&format!("node_{}", i), c.clone()))
            .collect();

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 12, "all 12 nodes returned");
        for (i, c) in counts.iter().enumerate() {
            let ticks = c.load(std::sync::atomic::Ordering::Relaxed);
            assert!(
                ticks > 0,
                "node_{} must tick at least once, got {}",
                i,
                ticks
            );
        }
    }

    /// Node with a sub-microsecond tick (empty body) — no issues with near-zero durations.
    struct NoOpNode(String);
    impl Node for NoOpNode {
        fn name(&self) -> &str {
            &self.0
        }
        fn tick(&mut self) {
            // Intentionally empty — sub-microsecond tick
        }
    }

    #[test]
    fn test_sub_microsecond_node_runs_without_issue() {
        use crate::core::NodeInfo;

        let node = NoOpNode("noop".to_string());
        let registered = RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(node)),
            name: Arc::from("noop"),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new("noop".to_string())),
            rate_hz: None,
            last_tick: None,
            is_rt_node: true,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            health_probe_counter: 0,
            is_paused: false,
            diag: Default::default(),
            in_safe_mode: false,
            rt_stats: Some(crate::core::RtStats::default()),
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::Rt,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
            subscription_freshness: Vec::new(),
            use_sched_deadline: false,
            no_alloc: false,
        };

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![registered]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(30_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 1);
        // RtStats should have recorded ticks with non-NaN avg
        let stats = returned[0].rt_stats.as_ref().unwrap();
        assert!(
            stats.sampled_ticks() > 0,
            "sub-microsecond node should have recorded ticks"
        );
        assert!(
            stats.avg_execution_us().is_finite(),
            "avg_execution_us must be finite, not NaN/inf"
        );
    }

    /// Chain ordering: nodes are sorted by priority before execution.
    /// Nodes with lower priority values run first.
    #[test]
    fn test_chain_nodes_sorted_by_priority() {
        // Create nodes with intentionally reversed priorities to verify sorting.
        // After sorting, priority 0 runs before priority 10, which runs before priority 20.
        // We use a shared Vec to capture execution order on the first tick cycle.
        let order = Arc::new(Mutex::new(Vec::<String>::new()));

        struct OrderNode {
            name: String,
            order: Arc<Mutex<Vec<String>>>,
        }
        impl Node for OrderNode {
            fn name(&self) -> &str {
                &self.name
            }
            fn tick(&mut self) {
                let mut guard = self.order.lock().unwrap();
                // Only record during the first few ticks to keep deterministic
                if guard.len() < 30 {
                    guard.push(self.name.clone());
                }
            }
        }

        let make_order_node = |name: &str, priority: u32| -> RegisteredNode {
            use crate::core::NodeInfo;
            let node = OrderNode {
                name: name.to_string(),
                order: order.clone(),
            };
            RegisteredNode {
                node: super::super::types::NodeKind::new(Box::new(node)),
                name: Arc::from(name),
                priority,
                initialized: true,
                context: Some(NodeInfo::new(name.to_string())),
                rate_hz: None,
                last_tick: None,
                is_rt_node: true,
                tick_budget: None,
                deadline: None,
                recorder: None,
                is_stopped: false,
                health_probe_counter: 0,
                is_paused: false,
                diag: Default::default(),
                in_safe_mode: false,
                rt_stats: None,
                miss_policy: Miss::Warn,
                execution_class: super::super::types::ExecutionClass::Rt,
                health_state: super::super::types::AtomicHealthState::default(),
                os_priority: None,
                pinned_core: None,
                node_watchdog: None,
                failure_handler: None,
                budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
                subscription_freshness: Vec::new(),
                use_sched_deadline: false,
                no_alloc: false,
            }
        };

        // Provide in REVERSE priority order — start_pool should sort them
        let nodes = vec![
            make_order_node("prio_20", 20),
            make_order_node("prio_0", 0),
            make_order_node("prio_10", 10),
        ];

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![nodes],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(30_u64.ms());
        running.store(false, Ordering::SeqCst);
        let _returned = executor.stop();

        let recorded = order.lock().unwrap();
        // We need at least 3 entries (one full cycle) to check ordering
        assert!(
            recorded.len() >= 3,
            "need at least one full cycle of 3 nodes, got {}",
            recorded.len()
        );
        // Check that within each 3-node cycle, priority order is maintained
        // (prio_0, prio_10, prio_20) repeating
        // `as_chunks::<3>()` rather than `chunks_exact(3)`: the chunk size is a
        // constant, so this yields `&[String; 3]` and the indexing below is
        // bounds-checked at compile time. clippy 1.98 lints the `chunks_exact`
        // form for exactly this reason.
        let (cycles, _partial) = recorded.as_chunks::<3>();
        for chunk in cycles {
            assert_eq!(chunk[0], "prio_0", "first in cycle should be prio_0");
            assert_eq!(chunk[1], "prio_10", "second in cycle should be prio_10");
            assert_eq!(chunk[2], "prio_20", "third in cycle should be prio_20");
        }
    }

    /// Multiple chains with multiple nodes each — all execute.
    #[test]
    fn test_multiple_chains_multiple_nodes_each() {
        let counts: Vec<_> = (0..6)
            .map(|_| Arc::new(std::sync::atomic::AtomicU64::new(0)))
            .collect();

        // 3 chains with 2 nodes each
        let chain_0 = vec![
            make_rt_registered("c0_n0", counts[0].clone()),
            make_rt_registered("c0_n1", counts[1].clone()),
        ];
        let chain_1 = vec![
            make_rt_registered("c1_n0", counts[2].clone()),
            make_rt_registered("c1_n1", counts[3].clone()),
        ];
        let chain_2 = vec![
            make_rt_registered("c2_n0", counts[4].clone()),
            make_rt_registered("c2_n1", counts[5].clone()),
        ];

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![chain_0, chain_1, chain_2],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 6, "all 6 nodes returned across 3 chains");
        for (i, c) in counts.iter().enumerate() {
            let ticks = c.load(std::sync::atomic::Ordering::Relaxed);
            assert!(ticks > 0, "node index {} must tick, got 0", i);
        }
    }

    /// RtStats are correctly populated after execution.
    #[test]
    fn test_rt_stats_populated_after_execution() {
        use crate::core::NodeInfo;

        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let node = CounterNode {
            name: "stats_node".to_string(),
            count: count.clone(),
        };

        let registered = RegisteredNode {
            node: super::super::types::NodeKind::new(Box::new(node)),
            name: Arc::from("stats_node"),
            priority: 0,
            initialized: true,
            context: Some(NodeInfo::new("stats_node".to_string())),
            rate_hz: None,
            last_tick: None,
            is_rt_node: true,
            tick_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            health_probe_counter: 0,
            is_paused: false,
            diag: Default::default(),
            in_safe_mode: false,
            rt_stats: Some(crate::core::RtStats::default()),
            miss_policy: Miss::Warn,
            execution_class: super::super::types::ExecutionClass::Rt,
            health_state: super::super::types::AtomicHealthState::default(),
            os_priority: None,
            pinned_core: None,
            node_watchdog: None,
            failure_handler: None,
            budget_policy: super::super::safety_monitor::BudgetPolicy::default(),
            subscription_freshness: Vec::new(),
            use_sched_deadline: false,
            no_alloc: false,
        };

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![registered]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        let stats = returned[0].rt_stats.as_ref().unwrap();
        let tick_count = count.load(std::sync::atomic::Ordering::Relaxed);

        assert_eq!(
            stats.sampled_ticks(),
            tick_count,
            "sampled_ticks should match actual tick count"
        );
        assert!(
            stats.sampled_ticks() > 0,
            "must have recorded at least one tick"
        );
        assert!(
            stats.worst_execution() >= stats.last_execution() || stats.sampled_ticks() == 1,
            "worst >= last (or only one sample)"
        );
        assert_eq!(stats.deadline_misses(), 0, "no deadline set → 0 misses");
        assert_eq!(stats.budget_violations(), 0, "no budget set → 0 violations");
        assert!(stats.avg_execution_us().is_finite(), "avg must be finite");
        assert!(stats.jitter_us().is_finite(), "jitter must be finite");
    }

    /// Stopped node is skipped — it never gets ticked.
    #[test]
    fn test_stopped_node_is_skipped() {
        let stopped_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let healthy_count = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let mut stopped_node = make_rt_registered("stopped", stopped_count.clone());
        stopped_node.is_stopped = true;

        let healthy_node = make_rt_registered("healthy", healthy_count.clone());

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![stopped_node, healthy_node]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 2);
        assert_eq!(
            stopped_count.load(std::sync::atomic::Ordering::Relaxed),
            0,
            "stopped node must never tick"
        );
        assert!(
            healthy_count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "healthy sibling must still tick"
        );
    }

    /// Uninitialized node is skipped — it never gets ticked.
    #[test]
    fn test_uninitialized_node_is_skipped() {
        let uninit_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let normal_count = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let mut uninit_node = make_rt_registered("uninit", uninit_count.clone());
        uninit_node.initialized = false;

        let normal_node = make_rt_registered("normal", normal_count.clone());

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![uninit_node, normal_node]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 2);
        assert_eq!(
            uninit_count.load(std::sync::atomic::Ordering::Relaxed),
            0,
            "uninitialized node must never tick"
        );
        assert!(
            normal_count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "initialized sibling must still tick"
        );
    }

    /// Paused node auto-unpauses after being skipped once.
    /// The RT loop sets is_paused=false and skips (continue), so the node
    /// misses exactly one tick cycle then resumes.
    #[test]
    fn test_paused_node_auto_unpauses() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let mut node = make_rt_registered("paused_node", count.clone());
        node.is_paused = true;

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![node]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        // Let it run — first cycle skips (unpause), subsequent cycles tick normally
        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 1);
        assert!(
            !returned[0].is_paused,
            "node should be unpaused after running"
        );
        assert!(
            count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "paused node must resume ticking after auto-unpause"
        );
    }

    /// Fallback period is used when no node declares a rate.
    #[test]
    fn test_fallback_period_used_when_no_rate() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        // Node has rate_hz = None
        let node = make_rt_registered("no_rate", count.clone());

        let running = Arc::new(AtomicBool::new(true));
        // Fallback period of 10ms → ~100Hz effective tick rate
        let executor = RtExecutor::start_pool(
            vec![vec![node]],
            running.clone(),
            10_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(100_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        let ticks = count.load(std::sync::atomic::Ordering::Relaxed);
        // At 10ms fallback period over 100ms, expect roughly 10 ticks (wide margin)
        assert!(
            (3..=50).contains(&ticks),
            "with 10ms fallback period over 100ms, expected 3-50 ticks, got {}",
            ticks
        );
        assert_eq!(returned.len(), 1);
    }

    /// Nodes without rt_stats=Some still tick without error (stats recording skipped).
    #[test]
    fn test_node_without_rt_stats() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let mut node = make_rt_registered("no_stats", count.clone());
        // Explicitly set rt_stats to None (this is the default from make_rt_registered)
        node.rt_stats = None;

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![node]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(30_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 1);
        assert!(
            returned[0].rt_stats.is_none(),
            "rt_stats should remain None"
        );
        assert!(
            count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "node without rt_stats must still tick"
        );
    }

    /// CPU affinity round-robin assignment across chains.
    /// Verifies the executor doesn't crash when rt_cpus are provided.
    #[test]
    fn test_cpu_affinity_with_multiple_chains() {
        let count_a = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count_b = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let chain_0 = vec![make_rt_registered("cpu_node_0", count_a.clone())];
        let chain_1 = vec![make_rt_registered("cpu_node_1", count_b.clone())];

        let running = Arc::new(AtomicBool::new(true));
        // Provide CPU cores — may fail to pin (requires root/capabilities) but
        // must not crash. The executor logs and continues unpinned.
        let executor = RtExecutor::start_pool(
            vec![chain_0, chain_1],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            vec![0, 1],
        )
        .unwrap();

        std::thread::sleep(30_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 2);
        assert!(count_a.load(std::sync::atomic::Ordering::Relaxed) > 0);
        assert!(count_b.load(std::sync::atomic::Ordering::Relaxed) > 0);
    }

    /// Immediate stop — running set false before any tick can occur.
    /// The executor must still return all nodes without hanging.
    #[test]
    fn test_immediate_stop_returns_all_nodes() {
        let counts: Vec<_> = (0..4)
            .map(|_| Arc::new(std::sync::atomic::AtomicU64::new(0)))
            .collect();

        let chain = counts
            .iter()
            .enumerate()
            .map(|(i, c)| make_rt_registered(&format!("imm_{}", i), c.clone()))
            .collect();

        let running = Arc::new(AtomicBool::new(false)); // Already false!

        let executor = RtExecutor::start_pool(
            vec![chain],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        let returned = executor.stop();
        assert_eq!(
            returned.len(),
            4,
            "all nodes returned even with immediate stop"
        );
    }

    /// Mixed stopped/uninitialized/paused/healthy in one chain — only healthy ones tick.
    #[test]
    fn test_mixed_node_states_in_chain() {
        let count_stopped = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count_uninit = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count_paused = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count_healthy = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let mut stopped = make_rt_registered("stopped", count_stopped.clone());
        stopped.is_stopped = true;

        let mut uninit = make_rt_registered("uninit", count_uninit.clone());
        uninit.initialized = false;

        let mut paused = make_rt_registered("paused", count_paused.clone());
        paused.is_paused = true;

        let healthy = make_rt_registered("healthy", count_healthy.clone());

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![stopped, uninit, paused, healthy]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 4);
        assert_eq!(
            count_stopped.load(std::sync::atomic::Ordering::Relaxed),
            0,
            "stopped node must not tick"
        );
        assert_eq!(
            count_uninit.load(std::sync::atomic::Ordering::Relaxed),
            0,
            "uninitialized node must not tick"
        );
        // Paused node skips one cycle then resumes
        assert!(
            count_paused.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "paused node must resume after auto-unpause"
        );
        assert!(
            count_healthy.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "healthy node must tick normally"
        );
    }

    // ========================================================================
    // FIX #5: executor installs the per-tick thread-local context
    // ========================================================================

    /// A node that records the ambient `horus::dt()` / `budget_remaining()` it
    /// observes *during* its tick. Used to prove the RT executor installs the
    /// tick context (mirroring run_node_tick) rather than leaving the inert
    /// fallbacks (dt→0, budget_remaining→MAX).
    struct CtxProbeNode {
        name: String,
        observed_dt: Arc<Mutex<Option<Duration>>>,
        observed_budget: Arc<Mutex<Option<Duration>>>,
        ticked: Arc<AtomicBool>,
    }
    impl Node for CtxProbeNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            let dt = crate::core::tick_context::ctx_dt();
            let budget = crate::core::tick_context::ctx_budget_remaining();
            *self.observed_dt.lock().unwrap() = Some(dt);
            *self.observed_budget.lock().unwrap() = Some(budget);
            self.ticked.store(true, Ordering::Relaxed);
        }
    }

    #[test]
    fn test_rt_executor_installs_tick_context() {
        let observed_dt = Arc::new(Mutex::new(None));
        let observed_budget = Arc::new(Mutex::new(None));
        let ticked = Arc::new(AtomicBool::new(false));

        let probe = CtxProbeNode {
            name: "ctx_probe".to_string(),
            observed_dt: observed_dt.clone(),
            observed_budget: observed_budget.clone(),
            ticked: ticked.clone(),
        };

        // 1kHz rate + 800us budget → during tick, dt()==1ms and
        // budget_remaining()<MAX (and <=800us). Without FIX #5 the executor
        // never sets the context, so dt()==ZERO and budget_remaining()==MAX.
        let mut reg =
            make_rt_registered("ctx_probe", Arc::new(std::sync::atomic::AtomicU64::new(0)));
        reg.node = super::super::types::NodeKind::new(Box::new(probe));
        reg.rate_hz = Some(1000.0);
        reg.tick_budget = Some(Duration::from_micros(800));

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![reg]],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
            Vec::new(),
        )
        .unwrap();

        // Wait until the probe has ticked at least once.
        for _ in 0..100 {
            if ticked.load(Ordering::Relaxed) {
                break;
            }
            std::thread::sleep(5_u64.ms());
        }
        running.store(false, Ordering::SeqCst);
        let _ = executor.stop();

        assert!(
            ticked.load(Ordering::Relaxed),
            "probe node must have ticked"
        );
        let dt = observed_dt
            .lock()
            .unwrap()
            .expect("dt must have been recorded");
        let budget = observed_budget
            .lock()
            .unwrap()
            .expect("budget must have been recorded");

        // dt = 1/rate (the exact value run_node_tick computes). RED: dt == ZERO.
        assert_eq!(
            dt,
            Duration::from_secs_f64(1.0 / 1000.0),
            "dt() during executor tick must equal 1/rate, got {:?}",
            dt
        );
        assert!(
            dt > Duration::ZERO,
            "dt() must not be the inert ZERO fallback"
        );
        // budget_remaining = budget - elapsed. RED: budget == Duration::MAX.
        assert!(
            budget < Duration::MAX,
            "budget_remaining() must be real, not the MAX fallback, got {:?}",
            budget
        );
        assert!(
            budget <= Duration::from_micros(800),
            "budget_remaining() must be <= configured budget, got {:?}",
            budget
        );
    }

    // ========================================================================
    // FIX #2: executor feeds the watchdog for its critical nodes
    // ========================================================================

    use super::super::safety_monitor::{SafetyMonitor, WatchdogFeeder};

    fn monitors_with_watchdog(feeder: WatchdogFeeder) -> SharedMonitors {
        let mut m = test_monitors();
        m.watchdog = Some(feeder);
        m
    }

    /// A healthy critical RT node's watchdog must be fed by the executor.
    /// RED (no FIX #2): last_heartbeat_ns never advances past registration →
    /// check_watchdogs would trip a spurious fleet-halting e-stop.
    #[test]
    fn test_rt_executor_feeds_critical_watchdog() {
        let monitor = SafetyMonitor::new(10);
        // Long timeout — we only assert it gets FED, not that it (never) expires.
        monitor.add_critical_node("rt_critical".to_string(), Duration::from_secs(5));
        let h0 = monitor
            .watchdog_last_heartbeat_ns("rt_critical")
            .expect("watchdog registered");

        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let mut reg = make_rt_registered("rt_critical", count.clone());
        reg.rate_hz = Some(1000.0); // is_rt_node already true → critical

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![reg]],
            running.clone(),
            1_u64.ms(),
            monitors_with_watchdog(monitor.watchdog_feeder()),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let _ = executor.stop();

        let h1 = monitor
            .watchdog_last_heartbeat_ns("rt_critical")
            .expect("watchdog registered");
        assert!(
            count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "node must have ticked"
        );
        assert!(
            h1 > h0,
            "critical RT node watchdog must be fed by the executor (h0={h0}, h1={h1})"
        );
    }

    /// A panicking tick must NOT feed the watchdog — this is what pins the
    /// feed-AFTER-successful-tick placement (Ok arm). RED (feed-before): the
    /// pre-tick feed advances the heartbeat even though the tick panics.
    #[test]
    fn test_rt_executor_does_not_feed_panicking_node() {
        let monitor = SafetyMonitor::new(10);
        monitor.add_critical_node("panic_critical".to_string(), Duration::from_secs(5));
        let h0 = monitor
            .watchdog_last_heartbeat_ns("panic_critical")
            .expect("watchdog registered");

        let panic_count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let mut reg = make_rt_registered(
            "panic_critical",
            Arc::new(std::sync::atomic::AtomicU64::new(0)),
        );
        reg.node = super::super::types::NodeKind::new(Box::new(AlwaysPanicNode {
            name: "panic_critical".to_string(),
            count: panic_count.clone(),
        }));
        reg.rate_hz = Some(1000.0);

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![reg]],
            running.clone(),
            1_u64.ms(),
            monitors_with_watchdog(monitor.watchdog_feeder()),
            Vec::new(),
        )
        .unwrap();

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let _ = executor.stop();

        assert!(
            panic_count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "panic node must have been ticked (and panicked)"
        );
        let h1 = monitor
            .watchdog_last_heartbeat_ns("panic_critical")
            .expect("watchdog registered");
        assert_eq!(
            h1, h0,
            "a panicking tick (Err) must NOT refresh the watchdog — feed lives in the Ok arm"
        );
    }

    /// Hang-detection (the dangerous inverse): a genuinely HUNG critical node
    /// (stuck in tick, never returns) is never fed, so its watchdog still
    /// expires and check_watchdogs trips the emergency stop. Uses a controllable
    /// hang (blocks on an AtomicBool) so the test is deterministic and joins
    /// cleanly. The Ok arm is never reached while hung → no feed.
    #[test]
    fn test_rt_executor_hung_node_still_trips_watchdog() {
        struct HangNode {
            name: String,
            entered: Arc<AtomicBool>,
            release: Arc<AtomicBool>,
        }
        impl Node for HangNode {
            fn name(&self) -> &str {
                &self.name
            }
            fn tick(&mut self) {
                self.entered.store(true, Ordering::Relaxed);
                while !self.release.load(Ordering::Relaxed) {
                    std::thread::sleep(Duration::from_millis(1));
                }
            }
        }

        let monitor = SafetyMonitor::new(10);
        monitor.add_critical_node("hang_node".to_string(), Duration::from_millis(30));

        let entered = Arc::new(AtomicBool::new(false));
        let release = Arc::new(AtomicBool::new(false));
        let mut reg =
            make_rt_registered("hang_node", Arc::new(std::sync::atomic::AtomicU64::new(0)));
        reg.node = super::super::types::NodeKind::new(Box::new(HangNode {
            name: "hang_node".to_string(),
            entered: entered.clone(),
            release: release.clone(),
        }));
        // No rate_hz → ticks immediately and hangs on the first tick.

        let running = Arc::new(AtomicBool::new(true));
        let executor = RtExecutor::start_pool(
            vec![vec![reg]],
            running.clone(),
            1_u64.ms(),
            monitors_with_watchdog(monitor.watchdog_feeder()),
            Vec::new(),
        )
        .unwrap();

        // Wait for the tick to enter the hang, then wait well past the timeout.
        for _ in 0..200 {
            if entered.load(Ordering::Relaxed) {
                break;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(
            entered.load(Ordering::Relaxed),
            "hang tick must have started"
        );
        std::thread::sleep(Duration::from_millis(150)); // >> 30ms timeout

        // The scheduler main loop (absent here) would call this each cycle.
        let mut expired = Vec::new();
        monitor.check_watchdogs(&mut expired);
        assert!(
            expired.iter().any(|n| n == "hang_node"),
            "hung node's watchdog must expire (it was never fed): {expired:?}"
        );
        assert!(
            monitor.is_emergency_stop(),
            "an expired critical node must trigger the emergency stop"
        );

        // Release the hang so the RT thread returns and stop() joins cleanly.
        release.store(true, Ordering::SeqCst);
        running.store(false, Ordering::SeqCst);
        let _ = executor.stop();
    }

    // ========================================================================
    // Cyclic wait: absolute phase grid, overrun policy, guard-spin bounds
    // ========================================================================

    /// Operator overrides would invalidate the default-value assertions below.
    fn cyclic_env_overridden() -> bool {
        std::env::var("HORUS_RT_SPIN_GUARD_US").is_ok() || std::env::var("HORUS_RT_WAIT").is_ok()
    }

    /// Deadlines must be `anchor + n*period`, never `now + period`.
    ///
    /// The difference is invisible in a tick *count* — both shapes tick roughly
    /// the right number of times — so this asserts the invariant directly:
    /// every slot the waiter produces is an exact integer multiple of the
    /// period away from the anchor it started on. A "now + period" loop fails
    /// this the moment a wake overshoots, which is every wake.
    #[test]
    fn test_cyclic_waiter_deadlines_stay_on_an_absolute_grid() {
        let mut w = CyclicWaiter::new(Duration::from_millis(2), false, false);
        let anchor = w.anchor_ns;
        for _ in 0..10 {
            w.wait();
        }
        assert_eq!(
            (w.next_slot_ns - anchor) % w.period_ns,
            0,
            "slot {} is off the {}ns grid anchored at {} — deadlines are being \
             computed relative to 'now', which accumulates drift",
            w.next_slot_ns,
            w.period_ns,
            anchor
        );
        let advanced = (w.next_slot_ns - anchor) / w.period_ns;
        assert!(
            advanced >= 11,
            "10 waits must advance the grid by at least 11 slots, got {advanced}"
        );
    }

    /// An overrun drops whole slots and resumes on the ORIGINAL grid.
    ///
    /// It must not replay the missed ticks back to back: a burst of zero-slack
    /// ticks is exactly the CPU pattern RT bandwidth control punishes, so a
    /// catch-up policy would let one overrun manufacture the ~50 ms dequeue
    /// this rewrite exists to remove. Recovery cost must also be O(1) in how
    /// far behind the loop fell — no spiral.
    #[test]
    fn test_cyclic_waiter_overrun_skips_slots_without_catching_up() {
        let mut w = CyclicWaiter::new(Duration::from_millis(1), false, false);
        let anchor = w.anchor_ns;

        // Blow through ~20 slots without servicing any of them.
        std::thread::sleep(Duration::from_millis(20));

        let t0 = Instant::now();
        w.wait();
        let waited = t0.elapsed();

        assert_eq!(w.local.overruns, 1, "the missed slot must be counted");
        assert!(
            w.local.slots_skipped >= 15,
            "expected ~20 dropped slots, got {}",
            w.local.slots_skipped
        );
        assert_eq!(
            (w.next_slot_ns - anchor) % w.period_ns,
            0,
            "an overrun must resume on the original phase grid, not re-anchor"
        );
        assert!(
            waited < Duration::from_millis(10),
            "overrun recovery took {waited:?}; it must cost about one period no \
             matter how far behind the loop fell"
        );
    }

    /// The guard spin is a named constant, and it is additionally capped as a
    /// fraction of the period. A fixed guard is 2 % of a 1 kHz period but 20 %
    /// of a 10 kHz one — it would walk the loop back toward the kernel's 95 %
    /// RT bandwidth ceiling as the tick rate rises.
    #[test]
    fn test_guard_spin_is_clamped_to_a_fraction_of_the_period() {
        let slow = CyclicWaiter::new(Duration::from_millis(1), false, false);
        assert!(
            slow.guard_ns <= slow.period_ns >> SPIN_GUARD_PERIOD_SHIFT,
            "guard {}ns exceeds 1/{} of a 1ms period",
            slow.guard_ns,
            1u32 << SPIN_GUARD_PERIOD_SHIFT
        );

        let fast = CyclicWaiter::new(Duration::from_micros(100), false, false);
        assert!(
            fast.guard_ns <= fast.period_ns >> SPIN_GUARD_PERIOD_SHIFT,
            "guard {}ns exceeds 1/{} of a 100us period",
            fast.guard_ns,
            1u32 << SPIN_GUARD_PERIOD_SHIFT
        );

        if cyclic_env_overridden() {
            return;
        }
        assert_eq!(
            slow.guard_ns, SPIN_GUARD_DEFAULT_NS,
            "a 1ms period has room for the full default guard"
        );
        assert_eq!(
            fast.guard_ns,
            100_000 >> SPIN_GUARD_PERIOD_SHIFT,
            "a 100us period must clamp the default guard down"
        );
        assert!(fast.guard_ns < SPIN_GUARD_DEFAULT_NS);
    }

    /// The default must be the absolute sleep, not the busy-wait it replaced.
    #[test]
    fn test_absolute_sleep_is_the_default_wait_mode() {
        if cyclic_env_overridden() {
            return;
        }
        let w = CyclicWaiter::new(Duration::from_millis(1), false, false);
        let expected = if cfg!(target_os = "linux") {
            WaitMode::AbsoluteSleep
        } else {
            WaitMode::PortableSleep
        };
        assert_eq!(
            w.mode, expected,
            "the tick loop must sleep to an absolute deadline by default; \
             busy-waiting is opt-in via HORUS_RT_WAIT=spin"
        );
    }

    /// The median-for-tail trade must be measurable, not merely asserted in a
    /// comment: the wake path publishes what it actually cost.
    #[test]
    fn test_rt_wait_stats_are_published() {
        let before = rt_wait_stats().slots;
        let mut w = CyclicWaiter::new(Duration::from_millis(1), false, false);
        for _ in 0..5 {
            w.wait();
        }
        w.finish(false);

        let after = rt_wait_stats();
        assert!(
            after.slots >= before + 5,
            "cyclic wait slots must reach the process-wide counters ({} -> {})",
            before,
            after.slots
        );
        assert!(
            after.wake_late_total_ns > 0,
            "wake lateness must be recorded — it is the measured cost of \
             giving up the busy-wait"
        );
    }
}
