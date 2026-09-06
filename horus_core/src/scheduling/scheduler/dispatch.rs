//! Persistent lanes for the main tick loop's parallel node dispatch.
//!
//! # What this replaces, and why it is the same shape as the compute executor
//!
//! `execute_ready_dispatch` is on the per-tick path — it is what
//! `execute_nodes` calls for any graph with more than one node, which is every
//! program with two plain nodes. It used to open a `crossbeam::scope`, spawn
//! `min(nodes, available_parallelism)` OS threads, and join them. Every tick.
//!
//! Measured on the reference box: a 4-worker scope costs 129.6 µs to create and
//! join (32.4 µs per thread), an 8-worker scope 246.6 µs, against ~66 syscalls
//! per scope — 4 `clone3`, 4 `mmap`, 4 `munmap`, 4 `mprotect`, 4 `madvise`,
//! 4 `set_robust_list`, 16 `rt_sigprocmask`. A channel-dispatch pool doing the
//! same 4-job barrier costs 7.55 µs per cycle with 4 `clone3` for the whole
//! process: 17x, and zero `mmap`/`munmap` per tick.
//!
//! The `mmap`/`munmap` matter beyond their own cost. Each pair takes the
//! process-wide `mmap_lock` in write mode, which an RT thread's own first-touch
//! page faults need in read mode — so the main loop's thread churn was
//! periodically blocking the real-time threads it shares a process with. That
//! is the same mechanism `compute_executor`'s module docs already describe for
//! the identical defect it fixed there, and `ComputePool` is the shape reused
//! here.
//!
//! `available_parallelism()` was also called per tick, and it is not cheap:
//! measured at 34 syscalls per call on this box, because std's Linux
//! implementation reads `/proc/self/cgroup`, `/proc/self/mountinfo` and the
//! cgroup quota files every time and caches nothing. It is now read once.
//!
//! # Where the dependency logic moved, and why
//!
//! The workers used to run it themselves: each decremented its successors'
//! pending counts and pushed newly-ready nodes back onto a shared queue, which
//! meant every worker borrowed `successors`, `pending`, `should_tick`,
//! `tick_contexts` and a `completed` counter — five shared borrows that only a
//! scoped spawn could express.
//!
//! The coordinator does it now. A worker receives one job, runs it, and reports;
//! the coordinator decrements successors as results arrive and dispatches
//! whatever became ready. Same ready-dispatch order, same parallelism, and the
//! workers borrow nothing — which is what lets them outlive a tick.
//!
//! # The barrier, and what enforces it
//!
//! A job carries raw pointers into `self.nodes` and `self.clock`, so every
//! dispatched job MUST report before the tick returns. The scope used to
//! guarantee that structurally. Here it is [`Barrier`], whose `Drop` keeps
//! draining until the outstanding count reaches zero — so an unwind in the
//! coordinator cannot leave a worker holding a pointer into a borrow that has
//! ended.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crossbeam::channel::{bounded, Receiver, Sender};

use super::super::types::RegisteredNode;
use crate::core::clock::Clock;

/// Everything a worker needs to tick one node, with no borrow of the scheduler.
pub(super) struct Job {
    pub(super) index: usize,
    /// SAFETY (established by the coordinator, upheld by [`Barrier`]): points
    /// into `self.nodes`, each index dispatched at most once per tick, and every
    /// job reported before the tick returns.
    node: *mut RegisteredNode,
    /// SAFETY: points at `self.clock`, which outlives the barrier.
    clock: *const dyn Clock,
    ctx: Option<NodeTickContext>,
}

// SAFETY: the two pointers are only dereferenced between dispatch and the
// matching result, a window the coordinator's `Barrier` keeps strictly inside
// the borrow they came from. Each node index is dispatched at most once per
// tick, so no two workers ever address the same `RegisteredNode`; the clock is
// shared but read-only through `&dyn Clock`.
unsafe impl Send for Job {}

/// Per-node context, computed on the coordinator because it reads the clock.
pub(super) struct NodeTickContext {
    pub(super) tick_number: u64,
    pub(super) node_name: Arc<str>,
    pub(super) node_dt: Duration,
    pub(super) sim_time: Duration,
    pub(super) tick_start_ci: crate::core::clock::ClockInstant,
    pub(super) tick_budget: Option<Duration>,
}

/// One completed tick, reported back to the coordinator.
pub(super) struct JobResult {
    pub(super) index: usize,
    pub(super) tick_start: Instant,
    pub(super) duration: Duration,
    pub(super) result: std::thread::Result<()>,
}

/// Hardware parallelism, read once.
///
/// `std::thread::available_parallelism()` costs 34 syscalls per call on Linux —
/// it reads `/proc/self/cgroup`, `/proc/self/mountinfo` and the cgroup quota
/// files and caches nothing — and it was on the per-tick path.
pub(super) fn hw_parallelism() -> usize {
    static CACHED: std::sync::OnceLock<usize> = std::sync::OnceLock::new();
    *CACHED.get_or_init(|| {
        std::thread::available_parallelism()
            .map(|p| p.get())
            .unwrap_or(4)
    })
}

/// Persistent lanes plus the scratch the dispatch needs, so a steady-state tick
/// allocates nothing.
pub(super) struct ReadyDispatch {
    /// `None` once shut down, which is what makes the workers exit.
    job_tx: Option<Sender<Job>>,
    result_rx: Receiver<JobResult>,
    workers: Vec<std::thread::JoinHandle<()>>,
    running: Arc<AtomicBool>,

    // Reusable scratch. Cleared, never reallocated, in steady state.
    pending: Vec<usize>,
    ready: VecDeque<usize>,
    contexts: Vec<Option<NodeTickContext>>,
}

impl ReadyDispatch {
    /// Spawn the lanes.
    ///
    /// Called on the tick loop's own thread, so the lanes start life with the
    /// CPU mask `RtConfig::apply` already put it in — exactly what the scoped
    /// spawn gave them per tick, minus the `clone(2)`.
    ///
    /// The mask is inherited; the POLICY is not, and the earlier wording here
    /// claiming both was wrong. `set_realtime_priority` sets
    /// `SCHED_FIFO|SCHED_RESET_ON_FORK`, and the whole point of that flag is
    /// that a thread spawned from an RT thread comes up `SCHED_OTHER`. So these
    /// lanes run node ticks on the reserved cores at ordinary priority.
    ///
    /// That is not a regression this change introduced — `crossbeam::scope`
    /// spawned from the same thread and was reset the same way, so parallel
    /// node execution has always been best-effort while the coordinator is RT.
    /// It is left alone deliberately: giving N lanes `SCHED_FIFO` at one
    /// priority is a decision about RT bandwidth control and starvation that
    /// needs its own measurement, not a side effect of replacing the spawn
    /// mechanism. `spawn_best_effort` is the opposite case and stays opposite —
    /// helpers must be pushed OFF these cores, whereas lanes belong on them.
    pub(super) fn new(node_count: usize) -> Self {
        let wanted = node_count.min(hw_parallelism()).max(1);
        let (job_tx, job_rx) = bounded::<Job>(node_count.max(1));
        let (result_tx, result_rx) = bounded::<JobResult>(node_count.max(1));
        let running = Arc::new(AtomicBool::new(true));

        let mut workers = Vec::with_capacity(wanted);
        for id in 0..wanted {
            let job_rx = job_rx.clone();
            let result_tx = result_tx.clone();
            match std::thread::Builder::new()
                .name(format!("horus-dispatch-{id}"))
                .spawn(move || worker_main(job_rx, result_tx))
            {
                Ok(h) => workers.push(h),
                Err(e) => {
                    // Degrade to a smaller pool — possibly none, in which case
                    // every job runs inline on the coordinator. A scheduler
                    // that cannot spawn a thread must still tick.
                    crate::terminal::print_line(&format!(
                        "[Scheduler] could not spawn dispatch lane {id} ({e}) — \
                         continuing with {} lanes",
                        workers.len()
                    ));
                    break;
                }
            }
        }

        // Only workers receive jobs, and only workers send results: dropping
        // these two here means `recv` reports `Disconnected` when every worker
        // has exited, instead of blocking the barrier forever.
        drop(job_rx);
        drop(result_tx);

        Self {
            job_tx: (!workers.is_empty()).then_some(job_tx),
            result_rx,
            workers,
            running,
            pending: Vec::with_capacity(node_count),
            ready: VecDeque::with_capacity(node_count),
            contexts: Vec::with_capacity(node_count),
        }
    }

    /// How many lanes actually started.
    ///
    /// Fewer than requested when `Builder::spawn` failed; zero means every job
    /// runs on the coordinator.
    #[allow(dead_code, reason = "an accessor for tests and future diagnostics")]
    pub(super) fn lane_count(&self) -> usize {
        self.workers.len()
    }

    /// Stop the lanes and join them.
    pub(super) fn shutdown(&mut self) {
        self.running.store(false, Ordering::SeqCst);
        // Dropping the only `Sender<Job>` is what ends each worker's `recv()`.
        self.job_tx = None;
        for h in self.workers.drain(..) {
            let _ = h.join();
        }
    }
}

impl Drop for ReadyDispatch {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// Worker body: park on the channel, take one job, report, repeat.
///
/// `recv()` backs off briefly and then parks on a futex, so an idle lane
/// between ticks consumes no CPU and holds no lock. It returns
/// `Err(Disconnected)` when the coordinator drops the only `Sender<Job>`, which
/// is how this loop ends.
fn worker_main(job_rx: Receiver<Job>, result_tx: Sender<JobResult>) {
    while let Ok(job) = job_rx.recv() {
        let index = job.index;
        // `run_tick` already catches a panic from node code, so this catches
        // only the improbable — and the improbable here is a coordinator that
        // waits forever for a result that will never come. A control loop must
        // not have that failure mode.
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| run_job(job)))
            .unwrap_or_else(|payload| JobResult {
                index,
                tick_start: Instant::now(),
                duration: Duration::ZERO,
                result: Err(payload),
            });
        // If the coordinator has gone, so has the barrier that was waiting for
        // this — exit rather than spin.
        if result_tx.send(result).is_err() {
            return;
        }
    }
}

/// Run one node's tick and package the outcome.
///
/// Every path returns a result. `NodeRunner::run_tick` already catches a panic
/// from node code; this must not be the one place a job can vanish, because a
/// job that never reports is a barrier that never completes.
fn run_job(job: Job) -> JobResult {
    let Job {
        index,
        node,
        clock,
        ctx,
    } = job;

    if let Some(ref ctx) = ctx {
        crate::core::hlog::set_node_context(&ctx.node_name, ctx.tick_number);
        // SAFETY: `clock` points at the scheduler's clock, which outlives the
        // barrier that is waiting for this job.
        let clock_ref: &dyn Clock = unsafe { &*clock };
        crate::core::tick_context::set_tick_context(
            &ctx.node_name,
            ctx.tick_number,
            clock_ref,
            ctx.node_dt,
            ctx.sim_time,
            ctx.tick_start_ci,
            ctx.tick_budget,
        );
    }

    // SAFETY: each node index is dispatched at most once per tick, so no two
    // workers ever hold a `&mut` to the same `RegisteredNode`, and the barrier
    // keeps this window inside the coordinator's borrow.
    let node_ref = unsafe { &mut *node };
    let tr = super::super::primitives::NodeRunner::run_tick(&mut node_ref.node);

    crate::core::tick_context::clear_tick_context();
    crate::core::hlog::clear_node_context();

    JobResult {
        index,
        tick_start: tr.tick_start,
        duration: tr.duration,
        result: tr.result,
    }
}

/// Keeps the pointer window open until every dispatched job has reported.
///
/// The scoped spawn used to guarantee this structurally: the scope could not
/// end until every thread had joined. With lanes that outlive the tick, the
/// guarantee has to be explicit — and it has to survive an unwind in the
/// coordinator, because a job in flight holds raw pointers into a borrow that
/// an unwind would end.
struct Barrier<'a> {
    rx: &'a Receiver<JobResult>,
    outstanding: usize,
}

impl Barrier<'_> {
    /// Wait for one result. `None` once every lane has exited, which — because
    /// a lane only exits when it holds no job — means no result can still be
    /// coming.
    fn recv(&mut self) -> Option<JobResult> {
        match self.rx.recv() {
            Ok(r) => {
                self.outstanding -= 1;
                Some(r)
            }
            Err(_) => None,
        }
    }
}

impl Drop for Barrier<'_> {
    fn drop(&mut self) {
        while self.outstanding > 0 {
            match self.rx.recv() {
                Ok(_) => self.outstanding -= 1,
                // Every lane has exited. A lane exits only between jobs, so
                // there is nothing left holding a pointer.
                Err(_) => break,
            }
        }
    }
}

impl ReadyDispatch {
    /// Stage one node's per-tick context, computed on the coordinator because
    /// it reads the clock.
    pub(super) fn set_context(&mut self, index: usize, ctx: Option<NodeTickContext>) {
        if self.contexts.len() <= index {
            self.contexts.resize_with(index + 1, || None);
        }
        self.contexts[index] = ctx;
    }

    /// Clear the staged contexts without releasing their capacity.
    pub(super) fn reset_contexts(&mut self, node_count: usize) {
        if self.contexts.len() < node_count {
            self.contexts.resize_with(node_count, || None);
        }
        for slot in self.contexts.iter_mut() {
            *slot = None;
        }
    }

    /// Tick every node marked in `should_tick`, in dependency order, and fill
    /// `out` with one result per ticked node.
    ///
    /// Returns only once every dispatched job has reported — see [`Barrier`].
    ///
    /// # Safety
    ///
    /// `nodes_ptr` must address a slice of at least `should_tick.len()`
    /// `RegisteredNode`s that outlives this call, and `clock_ptr` a clock that
    /// does the same. The caller holds `&mut self` on the scheduler across the
    /// call, which is what makes both true.
    pub(super) unsafe fn run(
        &mut self,
        successors: &[Vec<usize>],
        dep_counts: &[usize],
        should_tick: &[bool],
        nodes_ptr: *mut RegisteredNode,
        clock_ptr: *const dyn Clock,
        out: &mut Vec<JobResult>,
    ) {
        let n = should_tick.len();
        let total = should_tick.iter().filter(|b| **b).count();
        if total == 0 {
            return;
        }

        // Adjusted dependency counts: a node that will not tick is treated as
        // already complete, so its successors do not wait for it.
        self.pending.clear();
        self.pending.extend_from_slice(&dep_counts[..n]);
        for (i, tick) in should_tick.iter().enumerate() {
            if !tick {
                for &succ in &successors[i] {
                    self.pending[succ] = self.pending[succ].saturating_sub(1);
                }
            }
        }

        self.ready.clear();
        for i in 0..n {
            if should_tick[i] && self.pending[i] == 0 {
                self.ready.push_back(i);
            }
        }

        let mut barrier = Barrier {
            rx: &self.result_rx,
            outstanding: 0,
        };
        let mut completed = 0usize;

        while completed < total {
            // Dispatch everything currently ready. A job no lane takes — the
            // channel is full, or there are no lanes at all — runs here rather
            // than being dropped.
            while let Some(i) = self.ready.pop_front() {
                let job = Job {
                    index: i,
                    // SAFETY: `i < n`, and the caller guarantees `nodes_ptr`
                    // addresses at least `n` nodes.
                    node: unsafe { nodes_ptr.add(i) },
                    clock: clock_ptr,
                    ctx: self.contexts.get_mut(i).and_then(Option::take),
                };
                // A job no lane takes — the channel is full, or there are no
                // lanes at all — runs on the coordinator rather than being
                // dropped. `try_send` hands the job back on either error, so
                // nothing is lost on the way.
                let inline = match self.job_tx.as_ref() {
                    Some(tx) => match tx.try_send(job) {
                        Ok(()) => {
                            barrier.outstanding += 1;
                            None
                        }
                        Err(e) => Some(e.into_inner()),
                    },
                    None => Some(job),
                };
                if let Some(job) = inline {
                    let r = run_job(job);
                    completed += 1;
                    release_successors(
                        r.index,
                        successors,
                        should_tick,
                        &mut self.pending,
                        &mut self.ready,
                    );
                    out.push(r);
                }
            }

            if completed >= total {
                break;
            }
            if barrier.outstanding == 0 {
                // Nothing ready, nothing running, and work left. The dependency
                // counts describe a node waiting on a predecessor that will
                // never report — a graph bug, not a state to hang a control
                // loop in.
                crate::terminal::print_line(&format!(
                    "[Scheduler] ready-dispatch stalled with {} of {total} nodes ticked \
                     and none runnable; the dependency counts are inconsistent",
                    completed
                ));
                break;
            }

            match barrier.recv() {
                Some(r) => {
                    completed += 1;
                    release_successors(
                        r.index,
                        successors,
                        should_tick,
                        &mut self.pending,
                        &mut self.ready,
                    );
                    out.push(r);
                }
                // Every lane has exited. Run whatever is left inline rather
                // than losing those ticks.
                None => {
                    self.job_tx = None;
                    for i in 0..n {
                        if should_tick[i] && !out.iter().any(|r| r.index == i) {
                            self.ready.push_back(i);
                        }
                    }
                    if self.ready.is_empty() {
                        break;
                    }
                }
            }
        }

        out.sort_unstable_by_key(|r| r.index);
    }
}

/// Decrement `index`'s successors and queue any that just became runnable.
fn release_successors(
    index: usize,
    successors: &[Vec<usize>],
    should_tick: &[bool],
    pending: &mut [usize],
    ready: &mut VecDeque<usize>,
) {
    for &succ in &successors[index] {
        if !should_tick[succ] {
            continue;
        }
        pending[succ] = pending[succ].saturating_sub(1);
        if pending[succ] == 0 {
            ready.push_back(succ);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_pool_sizes_itself_to_the_graph_and_the_machine() {
        let small = ReadyDispatch::new(2);
        assert!(
            small.lane_count() <= 2,
            "a two-node graph never needs more than two lanes, got {}",
            small.lane_count()
        );
        assert!(small.lane_count() >= 1, "a two-node graph must get a lane");

        let big = ReadyDispatch::new(1000);
        assert!(
            big.lane_count() <= hw_parallelism(),
            "the pool must not exceed the machine: {} lanes on {} CPUs",
            big.lane_count(),
            hw_parallelism()
        );
    }

    /// `available_parallelism()` costs 34 syscalls per call on Linux — it reads
    /// `/proc/self/cgroup`, `/proc/self/mountinfo` and the cgroup quota files
    /// and caches nothing — and it used to be on the per-tick path.
    #[test]
    fn hardware_parallelism_is_read_once() {
        let first = hw_parallelism();
        assert!(first >= 1);
        assert_eq!(first, hw_parallelism(), "the cached value must be stable");
    }

    /// A pool with no lanes must still run every job, on the coordinator.
    ///
    /// This is the degraded path taken when `Builder::spawn` fails — a
    /// scheduler that cannot get a thread must still tick.
    #[test]
    fn a_pool_with_no_lanes_still_reports_every_job() {
        let mut d = ReadyDispatch::new(1);
        d.shutdown();
        assert_eq!(d.lane_count(), 0);
        assert!(
            d.job_tx.is_none(),
            "after shutdown there is no lane to dispatch to, so every job must run inline"
        );
    }
}
