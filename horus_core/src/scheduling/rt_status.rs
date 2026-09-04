//! What the RT tick threads actually got from the kernel.
//!
//! # Why this module exists
//!
//! Every RT tick thread asks the kernel for a real-time scheduling policy
//! during startup, and that request can be refused — most commonly for want of
//! `CAP_SYS_NICE`, which is the default state of a developer machine, an
//! unprivileged container, and CI. Before this module, the outcome of that
//! request was a local `bool` in `rt_thread_main` that reached exactly one
//! consumer (`CyclicWaiter`, to pick a wait strategy) and was then dropped.
//!
//! The consequence was that a node running under SCHED_FIFO and a node running
//! under SCHED_OTHER were **indistinguishable to every programmatic consumer**.
//! Nothing in the query API, the registry, the stats, or the RT report could
//! answer "did real-time actually apply?" — the only evidence was a line of
//! text on stdout, and only when the request failed. A caller who wanted to
//! assert on it in a test, gate a control loop on it, or surface it in a health
//! endpoint had nothing to read.
//!
//! That gap also defeated `require_rt()`. Its enforcement path inspects the
//! degradations recorded by `apply_rt_optimizations`, which runs on the thread
//! that *builds* the scheduler and asks for priority 50. The tick threads make
//! a second, entirely separate request at their own priority, and recorded
//! nothing when it was refused — so `require_rt()` could pass while every tick
//! thread ran SCHED_OTHER, which is precisely the "silently running non-RT
//! while you believe otherwise" outcome it exists to prevent.
//!
//! # Why the report is scoped, not global
//!
//! A process-wide static would be simpler and wrong. `libtest` runs every test
//! in a binary as a thread of one process, so two tests that each build a
//! scheduler would write into the same registry and read each other's threads.
//! The report is owned by the `RtExecutor` that created it and handed to the
//! `Scheduler` that started it, so its contents describe that scheduler's
//! threads and no others.
//!
//! # Locking
//!
//! Publishing takes a `Mutex`. That is safe here and nowhere near the tick
//! loop: a thread publishes exactly once, during startup, alongside the
//! affinity/governor/IRQ syscalls it already makes before entering the loop.
//! The tick loop itself remains lock-free — see the `rt_diag` seqlock ring at
//! the top of `rt_executor.rs` for the discipline that applies there.

use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

/// The scheduling policy a thread is actually running under.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtPolicy {
    /// The ordinary time-sharing policy. On Linux, `SCHED_OTHER`.
    ///
    /// A thread here is preemptible by anything and subject to the default
    /// 50 us timer slack unless it was lowered explicitly.
    Other,
    /// `SCHED_FIFO` at some priority — runs until it blocks or yields.
    Fifo,
    /// `SCHED_DEADLINE` — admitted by the kernel with a runtime/period budget.
    Deadline,
}

impl RtPolicy {
    /// Whether this policy is one the kernel treats as real-time.
    ///
    /// This is the question almost every caller actually has, and phrasing it
    /// once here keeps `matches!` chains out of consumer code.
    pub fn is_realtime(self) -> bool {
        matches!(self, RtPolicy::Fifo | RtPolicy::Deadline)
    }

    /// The kernel's name for this policy, for diagnostics.
    pub fn as_str(self) -> &'static str {
        match self {
            RtPolicy::Other => "SCHED_OTHER",
            RtPolicy::Fifo => "SCHED_FIFO",
            RtPolicy::Deadline => "SCHED_DEADLINE",
        }
    }
}

/// What one RT tick thread got, recorded once during its startup.
#[derive(Debug, Clone)]
pub struct RtThreadStatus {
    /// OS thread name (`horus-rt`, or `horus-rt-<n>` when there are several).
    pub thread_name: String,
    /// The chain this thread ticks, named by its highest-priority node — the
    /// same label `check_core_collisions` reports, so diagnostics from the two
    /// can be matched up.
    pub chain_label: String,
    /// The policy the thread asked the kernel for.
    pub requested: RtPolicy,
    /// The policy it is actually running under.
    ///
    /// `granted != requested` is the whole point of this struct.
    pub granted: RtPolicy,
    /// RT priority in effect, or 0 under `SCHED_OTHER`.
    pub priority: i32,
    /// Why the kernel refused, when it did. `None` on success.
    pub refusal: Option<String>,
    /// CPUs this thread is pinned to; empty means unpinned.
    pub cpus: Vec<usize>,
    /// Whether the *process* had `mlockall` applied when this thread started.
    ///
    /// Per-process rather than per-thread because that is what `mlockall` is.
    /// It rides along here because an RT thread without locked memory is the
    /// pairing that matters: a major page fault in a `SCHED_FIFO` thread stalls
    /// it for as long as the I/O takes, which is orders of magnitude past any
    /// deadline the thread was given a real-time policy to meet.
    pub memory_locked: bool,
}

impl RtThreadStatus {
    /// Whether this thread asked for real-time and did not get it.
    pub fn is_degraded(&self) -> bool {
        self.requested.is_realtime() && !self.granted.is_realtime()
    }

    /// Whether this thread got real-time but is running on unlocked memory.
    ///
    /// Not a refusal — the thread has its policy. It is the configuration where
    /// the tail latency the policy was acquired for can still be blown by the
    /// pager, so it is worth naming separately rather than folding into
    /// `is_degraded`.
    pub fn is_realtime_without_locked_memory(&self) -> bool {
        self.granted.is_realtime() && !self.memory_locked
    }

    /// One-line human summary, used by diagnostics and the CLI.
    pub fn summary(&self) -> String {
        let mut s = format!("{}: {}", self.thread_name, self.granted.as_str());
        if self.granted.is_realtime() {
            s.push_str(&format!(" prio {}", self.priority));
        }
        if !self.cpus.is_empty() {
            s.push_str(&format!(" cpu {:?}", self.cpus));
        }
        if let Some(ref why) = self.refusal {
            s.push_str(&format!(
                " (requested {}: {})",
                self.requested.as_str(),
                why
            ));
        }
        if self.is_realtime_without_locked_memory() {
            s.push_str(" [memory not locked]");
        }
        s
    }
}

/// Collects one [`RtThreadStatus`] per RT tick thread.
///
/// Created by `RtExecutor::start_pool` with the number of threads it is about
/// to spawn, so [`wait_for_all`](Self::wait_for_all) knows what "all" means
/// without having to guess from arrival timing.
#[derive(Debug)]
pub struct RtThreadReport {
    expected: usize,
    inner: Mutex<Vec<RtThreadStatus>>,
    arrived: Condvar,
}

impl RtThreadReport {
    /// Create a report expecting `expected` threads to publish.
    pub fn new(expected: usize) -> Arc<Self> {
        Arc::new(Self {
            expected,
            inner: Mutex::new(Vec::with_capacity(expected)),
            arrived: Condvar::new(),
        })
    }

    /// Record what one thread got. Called once per thread, during its startup.
    pub fn publish(&self, status: RtThreadStatus) {
        let mut guard = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        guard.push(status);
        // Notify every waiter, not one: `wait_for_all` is not the only possible
        // consumer and a lost wakeup here would stall a startup path.
        self.arrived.notify_all();
    }

    /// How many threads this report is waiting for.
    pub fn expected(&self) -> usize {
        self.expected
    }

    /// Block until every expected thread has published, or `timeout` elapses.
    ///
    /// Returns `true` if all of them reported. A `false` is not treated as a
    /// failure by callers: a thread that is slow to start is still starting,
    /// and refusing to run over it would turn a diagnostic into an outage. The
    /// enforcement path reads whatever did arrive and says how many that was.
    pub fn wait_for_all(&self, timeout: Duration) -> bool {
        let deadline = Instant::now() + timeout;
        let mut guard = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        while guard.len() < self.expected {
            let Some(remaining) = deadline.checked_duration_since(Instant::now()) else {
                return false;
            };
            let (g, wait) = self
                .arrived
                .wait_timeout(guard, remaining)
                .unwrap_or_else(|e| e.into_inner());
            guard = g;
            if wait.timed_out() && guard.len() < self.expected {
                return false;
            }
        }
        true
    }

    /// Every status published so far.
    pub fn statuses(&self) -> Vec<RtThreadStatus> {
        self.inner.lock().unwrap_or_else(|e| e.into_inner()).clone()
    }

    /// Threads that asked for a real-time policy and were refused.
    pub fn degraded(&self) -> Vec<RtThreadStatus> {
        self.statuses()
            .into_iter()
            .filter(RtThreadStatus::is_degraded)
            .collect()
    }

    /// Threads running real-time on unlocked memory.
    pub fn realtime_without_locked_memory(&self) -> Vec<RtThreadStatus> {
        self.statuses()
            .into_iter()
            .filter(RtThreadStatus::is_realtime_without_locked_memory)
            .collect()
    }

    /// Whether every thread that reported is running a real-time policy.
    ///
    /// Vacuously `true` when nothing has reported yet, so callers that care
    /// about coverage should check [`statuses`](Self::statuses) length or use
    /// [`wait_for_all`](Self::wait_for_all) first.
    pub fn all_realtime(&self) -> bool {
        self.statuses().iter().all(|s| s.granted.is_realtime())
    }
}
