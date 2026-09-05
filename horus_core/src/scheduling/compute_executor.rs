//! Compute thread pool executor.
//!
//! Runs non-RT (compute/best-effort) nodes on a **persistent** worker pool.
//! The compute pool runs independently of the RT thread — slow compute
//! nodes never block RT execution.
//!
//! # Architecture
//!
//! ```text
//!  Compute coordinator thread (owns all compute nodes)
//!  ┌───────────────────────────────────────────────────────────┐
//!  │  startup: spawn W workers ONCE, park them                 │
//!  │  loop:                                                    │
//!  │    ┌─ dispatch: bounded channel, no thread creation ───┐  │
//!  │    │   worker 0    : node_A.tick()                     │  │
//!  │    │   worker 1    : node_B.tick()                     │  │
//!  │    │   coordinator : node_C.tick()   (it works too)    │  │
//!  │    └─ barrier: drain exactly W' results ───────────────┘  │
//!  │    process results in node order (profiling, policies)    │
//!  │    sleep until next tick                                  │
//!  └───────────────────────────────────────────────────────────┘
//! ```
//!
//! Per-node rate limiting is respected — nodes that aren't due yet are skipped.
//!
//! # Why this is a real pool now (and why the old one hurt the RT thread)
//!
//! This loop used to call `crossbeam::scope` every cycle and `s.spawn` one
//! child per ready node. `crossbeam`'s `spawn` is not a pool: each call is a
//! genuine `clone(2)`, which maps a fresh ~2 MB stack, runs exactly one
//! `tick()`, and unmaps it on join. With N ready compute nodes that is N
//! thread create/join pairs **per cycle**, at roughly 30-80 µs each — and the
//! coordinator paid them serially, before the first child could be joined.
//!
//! The cost that put this in a *latency* effort rather than a throughput one is
//! second order. Every `mmap`/`munmap` pair takes the process-wide
//! `mmap_lock` in **write** mode. The RT thread's own first-touch page faults
//! take that same lock in read mode, so a compute pool churning stacks at the
//! tick rate is a noisy neighbour reaching into the RT thread's tail through
//! the kernel's memory subsystem — unbounded, and invisible to any profile of
//! the compute path itself. Persistent workers pay that cost once, at startup.
//!
//! # Design decisions (deliberate, and each one is a trade)
//!
//! * **Pool size** — see [`ComputePool::worker_count`]. Bounded at startup by
//!   the node count and the machine width, never by the per-cycle ready count.
//! * **Not pinned.** This executor is not told which CPUs the RT executor owns
//!   (`rt_cpus` is passed to `RtExecutor::start`, not here), so any affinity
//!   mask chosen locally could land a compute worker on the RT core — strictly
//!   worse than leaving the workers on the process mask and letting the load
//!   balancer keep them off a core that is running a `SCHED_FIFO` thread.
//! * **Best-effort scheduling class, explicitly.** See
//!   [`set_best_effort_class`]: threads inherit their creator's policy, so on
//!   a hard-RT deployment (`chrt -f 80 ./robot`, or an `RtConfig::apply()` on
//!   the main thread) every compute thread was silently `SCHED_FIFO` at the RT
//!   thread's own priority and could preempt it. Compute nodes are best-effort
//!   by definition of their `ExecutionClass`; they are now put there by hand.
//! * **Workers park, they never spin.** Dispatch is a `crossbeam` bounded
//!   channel whose blocking `recv()` ends in `std::thread::park()` (a futex
//!   wait on Linux) after a short bounded backoff. An idle worker between
//!   ticks costs zero CPU — trading the mmap tail for a spin tail would be no
//!   trade at all.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crossbeam::channel::{Receiver, Sender};

use crate::terminal::print_line;

use super::primitives::NodeRunner;
use super::types::{RegisteredNode, SharedMonitors};

/// Minimum node order value that qualifies for load shedding.
/// Nodes with `priority >= SHED_THRESHOLD` can be skipped under overload.
/// Maps to the "Background" tier in the order guidelines (200+).
const SHED_THRESHOLD: u32 = 200;

/// Number of consecutive under-budget cycles before shedding deactivates.
/// Prevents rapid on/off thrashing when overloaded nodes are right at the edge.
const SHED_COOLDOWN_CYCLES: u32 = 3;

/// Nice increment applied to compute *worker* threads (Linux only).
///
/// `SCHED_FIFO` always preempts `SCHED_OTHER`, so when the RT executor holds
/// real-time privilege this value is irrelevant. It matters in the case that is
/// both common and worst: no `CAP_SYS_NICE` (containers, dev boxes, CI), where
/// the RT loop is itself `SCHED_OTHER` and a busy compute node competes with it
/// for timeslices on equal weight. Five nice levels is roughly a 3x weight
/// reduction, which is the ordering this runtime wants: compute yields to
/// control.
///
/// TRADE: compute nodes get measurably less CPU under contention on a machine
/// without RT privilege. That is the intended direction for this runtime's
/// figure of merit (RT worst case), but it is a real cost to compute
/// throughput, not a free win.
const COMPUTE_WORKER_NICE: i32 = 5;

/// Stack bytes each worker faults in at startup.
///
/// Worker stacks are mapped once now instead of once per cycle, but the pages
/// are still faulted lazily on first touch — and a page fault takes `mmap_lock`
/// in read mode, which is the same lock the RT thread's own faults need.
/// Touching them up front moves that contention to startup. 64 KiB covers the
/// executor frames; a node's own deep stack still faults on first use.
const WORKER_PREFAULT_BYTES: usize = 64 * 1024;

/// Total (not per-thread) budget for joining workers during shutdown.
///
/// Workers hold no borrow of the node vector once the tick barrier has
/// completed — see [`ComputePool::run_cycle`] — so joining them is hygiene
/// rather than a safety requirement, and it must not eat into the compute
/// thread's own `SHUTDOWN_TIMEOUT_PER_THREAD` budget.
const WORKER_JOIN_BUDGET: Duration = Duration::from_millis(500);

/// Poll interval while waiting for workers to exit during shutdown.
const WORKER_JOIN_POLL: Duration = Duration::from_micros(200);

/// One node's tick outcome, as collected by the coordinator each cycle.
///
/// Identical in shape to the value the old `crossbeam::scope` join produced;
/// it just travels back over a channel instead of out of a `JoinHandle`.
struct ParallelResult {
    index: usize,
    tick_start: Instant,
    duration: Duration,
    result: std::thread::Result<()>,
}

/// One unit of per-cycle work: tick the node at `index`.
///
/// Carries a raw pointer rather than `&mut RegisteredNode` because the pool
/// outlives any single cycle and therefore cannot hold a borrow of the node
/// vector in its type. The borrow is re-established, and bounded, by
/// [`ComputePool::run_cycle`]'s barrier.
struct Job {
    index: usize,
    node: *mut RegisteredNode,
}

// SAFETY: the pointer targets one element of the `Vec<RegisteredNode>` owned by
// the coordinator thread. `run_cycle` dispatches each index at most once per
// cycle (so no two workers ever alias the same node) and does not return until
// every dispatched job has reported back, so the pointee outlives the job and
// the coordinator does not touch the vector while a job is live. Both
// invariants are re-stated at their enforcement points.
unsafe impl Send for Job {}

/// Installs the per-tick thread-local context and guarantees it is cleared on
/// the same thread, including if the tick unwinds.
///
/// A pooled worker outlives the tick, so a leaked context would be inherited by
/// the *next* node to land on that worker — it would log under the wrong node
/// name and read another node's `dt()`/budget. With `crossbeam::scope` the
/// thread died at the end of the cycle and hid that class of bug.
struct TickContextGuard;

impl TickContextGuard {
    /// Install the context for `node` on the calling thread.
    ///
    /// Zero-sized and lifetime-free on purpose: the guard borrows nothing, so
    /// the caller can take `&mut node.node` for the tick itself.
    fn install(
        node: &RegisteredNode,
        clock: &dyn crate::core::clock::Clock,
        tick_period: Duration,
    ) -> Self {
        super::primitives::set_node_tick_context(node, clock, tick_period);
        Self
    }
}

impl Drop for TickContextGuard {
    fn drop(&mut self) {
        super::primitives::clear_node_tick_context();
    }
}

/// Run one job to completion on the calling thread.
///
/// Used by both the workers and the coordinator (which executes one job per
/// cycle itself rather than idling at the barrier), which is exactly why the
/// tick context is installed *here*, inside the unit of work, rather than by
/// the dispatcher: `set_node_tick_context` writes thread-locals and MUST run on
/// the thread that will call `tick()` (FIX #5). The old code satisfied that by
/// capturing a non-`'static` `&dyn Clock` inside the `crossbeam::scope`
/// closure; a persistent worker cannot hold a non-`'static` capture, so it owns
/// a clone of the `Arc<dyn Clock>` in its `SharedMonitors` instead — the same
/// clock object, without the borrow.
fn run_job(job: Job, monitors: &SharedMonitors) -> ParallelResult {
    let index = job.index;
    // SAFETY: see `unsafe impl Send for Job`. This job was dispatched by
    // `run_cycle` for a distinct index and is live only until the result below
    // is handed back, which is what releases the coordinator's barrier.
    let node_ref: &mut RegisteredNode = unsafe { &mut *job.node };

    // The inner `catch_unwind` in `NodeRunner::run_tick` isolates a panicking
    // `tick()`. This outer one isolates a panic in the surrounding *machinery*
    // (context install, guard drop) — under `crossbeam::scope` such a panic
    // propagated through `join().expect(...)` and killed the whole compute
    // thread, taking every other compute node's ownership with it. A pooled
    // worker must survive it, so it is attributed to the node whose tick it
    // occurred in and fed through that node's normal failure policy.
    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _ctx = TickContextGuard::install(node_ref, &*monitors.clock, monitors.tick_period);
        NodeRunner::run_tick(&mut node_ref.node)
    }));

    match outcome {
        Ok(tr) => ParallelResult {
            index,
            tick_start: tr.tick_start,
            duration: tr.duration,
            result: tr.result,
        },
        Err(payload) => ParallelResult {
            index,
            tick_start: Instant::now(),
            duration: Duration::ZERO,
            result: Err(payload),
        },
    }
}

/// Put the calling thread in the best-effort scheduling class so it can never
/// preempt the RT executor.
///
/// Threads inherit the scheduling policy of whoever created them. The compute
/// coordinator is spawned from the thread that built the `Scheduler`, which on
/// a hard-RT deployment is already `SCHED_FIFO` — so every compute thread ran
/// at real-time priority, and the module's own promise that "slow compute nodes
/// never block RT execution" was false at the OS level. Demotion to
/// `SCHED_OTHER` is always permitted (it lowers the class) and makes that
/// promise structural: a `SCHED_FIFO` RT thread preempts these threads
/// unconditionally, whatever they are doing.
///
/// TRADE: a deployment that ran the whole process under `chrt -f` and relied on
/// compute nodes inheriting that priority loses it. Nodes that need real-time
/// scheduling belong in the RT executor, which sets its priority explicitly;
/// `ExecutionClass::Compute` is documented as best-effort.
#[cfg(target_os = "linux")]
fn set_best_effort_class(label: &str, nice_increment: i32) {
    // SAFETY: `pid == 0` addresses the calling thread and `sched_getscheduler`
    // only reads.
    if unsafe { libc::sched_getscheduler(0) } != libc::SCHED_OTHER {
        // SAFETY: `sched_param` is a POD struct of integer fields for which
        // all-zero is a valid bit pattern. It is zeroed rather than
        // literal-constructed because musl carries four extra sporadic-server
        // members that glibc does not (the same portability trap documented in
        // horus_sys::rt::linux::sched_param_with_priority); SCHED_OTHER ignores
        // every member but `sched_priority`, which must be 0.
        let rc = unsafe {
            let mut param: libc::sched_param = std::mem::zeroed();
            param.sched_priority = 0;
            libc::sched_setscheduler(0, libc::SCHED_OTHER, &param)
        };
        if rc != 0 {
            // Never silent: if this fails the thread keeps a policy that can
            // preempt the RT thread, which is a latency fact the operator needs.
            print_line(&format!(
                "[Compute] {label}: could not demote to SCHED_OTHER ({}) — thread keeps \
                 its inherited policy and may preempt the RT thread",
                std::io::Error::last_os_error()
            ));
        }
    }

    if nice_increment > 0 {
        // SAFETY: `nice` adjusts only the calling thread's CFS weight (the nice
        // value is a per-thread attribute on Linux/NPTL). Raising it never
        // requires privilege. The return value is deliberately ignored: it
        // cannot be distinguished from a legitimate -1 result without errno
        // handling, and the failure mode — the worker keeps its inherited
        // weight — is not worth a log line on a path that cannot fail.
        let _ = unsafe { libc::nice(nice_increment) };
    }
}

#[cfg(not(target_os = "linux"))]
fn set_best_effort_class(_label: &str, _nice_increment: i32) {
    // No portable equivalent. macOS/Windows deployments of this runtime are
    // development targets, where the RT executor has no hard guarantee either.
}

/// Persistent worker pool for parallel compute ticks.
///
/// Created once per executor run and fed one batch of jobs per cycle. Holds no
/// borrow of the node vector: the coordinator hands it raw pointers per cycle
/// and the tick barrier bounds their validity.
struct ComputePool {
    /// `None` when the pool has no workers (single node, or every spawn
    /// failed) — [`ComputePool::run_cycle`] then runs every job inline.
    job_tx: Option<Sender<Job>>,
    /// The coordinator holds no `Sender<ParallelResult>` clone on purpose: if
    /// every worker were to die, `recv()` reports `Disconnected` instead of
    /// blocking the barrier forever.
    result_rx: Receiver<ParallelResult>,
    workers: Vec<JoinHandle<()>>,
}

impl ComputePool {
    /// Number of worker threads to create for `node_count` compute nodes.
    ///
    /// Bounded by two things, both known at startup:
    ///
    /// * `node_count - 1`, because at most every node is ready in one cycle and
    ///   the coordinator executes one of them itself instead of idling at the
    ///   barrier. More workers than that could never all be busy.
    /// * `available_parallelism() - 1`, reserving one CPU's worth of runnable
    ///   slack for the RT thread and this coordinator. Compute workers beyond
    ///   the machine width buy no parallelism and cost context switches, and
    ///   every runnable compute thread is a thread the scheduler may place on a
    ///   core an RT thread wants.
    ///
    /// TRADE: when more nodes are ready in a cycle than `workers + 1`, their
    /// ticks queue on the channel and are serialized within the cycle, where
    /// the old code gave every ready node its own thread immediately. Compute
    /// nodes are rate-limited best-effort work and the tick barrier is
    /// unchanged, so a cycle takes as long as its slowest lane rather than its
    /// slowest node — the deliberate exchange is peak fan-out for a thread
    /// count that is bounded and paid once.
    fn worker_count(node_count: usize) -> usize {
        if node_count <= 1 {
            return 0;
        }
        let hw = std::thread::available_parallelism()
            .map(|p| p.get())
            .unwrap_or(2);
        let cap = hw.saturating_sub(1).max(1);
        (node_count - 1).min(cap)
    }

    /// Spawn the pool. Called once, before the tick loop starts.
    fn new(node_count: usize, monitors: &SharedMonitors) -> Self {
        let wanted = Self::worker_count(node_count);

        // Bounded, and sized so a send can never block or allocate: a cycle
        // dispatches at most `node_count - 1` jobs and returns at most
        // `node_count` results, and the barrier drains both before the next
        // cycle begins. An unbounded channel would allocate a block on the
        // dispatch path, which is jitter for no benefit.
        let capacity = node_count.max(1);
        let (job_tx, job_rx) = crossbeam::channel::bounded::<Job>(capacity);
        let (result_tx, result_rx) = crossbeam::channel::bounded::<ParallelResult>(capacity);

        let mut workers = Vec::with_capacity(wanted);
        for id in 0..wanted {
            let job_rx = job_rx.clone();
            let result_tx = result_tx.clone();
            let monitors = monitors.clone();
            match std::thread::Builder::new()
                .name(format!("horus-compute-{id}"))
                // Stack size is deliberately left at the platform default so
                // RUST_MIN_STACK still applies: compute nodes include Python
                // interpreters and other deep stacks, and these threads are
                // created once, so the virtual reservation is not a cost worth
                // trading against an overflow.
                .spawn(move || Self::worker_main(id, job_rx, result_tx, monitors))
            {
                Ok(handle) => workers.push(handle),
                Err(e) => {
                    // Degrade to a smaller pool (possibly none) rather than
                    // refusing to run compute nodes at all.
                    print_line(&format!(
                        "[Compute] Failed to spawn worker {id} ({e}) — continuing with {} workers",
                        workers.len()
                    ));
                    break;
                }
            }
        }

        // Only workers receive jobs, and only workers send results.
        drop(job_rx);
        drop(result_tx);

        Self {
            job_tx: (!workers.is_empty()).then_some(job_tx),
            result_rx,
            workers,
        }
    }

    /// Worker thread body: park, take one job, report, repeat.
    fn worker_main(
        id: usize,
        job_rx: Receiver<Job>,
        result_tx: Sender<ParallelResult>,
        monitors: SharedMonitors,
    ) {
        set_best_effort_class(&format!("worker {id}"), COMPUTE_WORKER_NICE);
        crate::core::rt_config::prefault_stack(WORKER_PREFAULT_BYTES);

        // `recv()` tries the channel, backs off for a short bounded spin, then
        // parks via `std::thread::park()` — a futex wait on Linux. An idle
        // worker between ticks therefore consumes no CPU and holds no lock.
        // It returns `Err(Disconnected)` when `shutdown` drops the only
        // `Sender<Job>`, which is how this loop ends.
        while let Ok(job) = job_rx.recv() {
            let result = run_job(job, &monitors);
            if result_tx.send(result).is_err() {
                // Coordinator is gone; nothing can consume further work.
                break;
            }
        }
    }

    /// Tick every node in `ready` and collect one result per node into `out`.
    ///
    /// This is the tick barrier: it does not return until every dispatched job
    /// has reported back, which is what makes the raw pointers in [`Job`] safe
    /// and what preserves the old `crossbeam::scope` semantics — including hang
    /// detection, because a node that never returns from `tick()` blocks this
    /// barrier exactly as it used to block the scope join.
    ///
    /// `out` is reused across cycles by the caller and is refilled in ascending
    /// node order, so downstream processing sees the same order the scoped
    /// version produced (`ready_indices` order) regardless of which lane
    /// happened to finish first — the cycle stays deterministic.
    fn run_cycle(
        &self,
        nodes_ptr: *mut RegisteredNode,
        ready: &[usize],
        monitors: &SharedMonitors,
        out: &mut Vec<ParallelResult>,
    ) {
        out.clear();

        // The coordinator runs the last ready node itself: with one ready node
        // that is the whole cycle (no dispatch, no wakeup — the old "single
        // node: tick directly" fast path, now falling out of the general case),
        // and with more it saves one worker and one park/unpark round trip.
        let Some((&inline_index, dispatched)) = ready.split_last() else {
            return;
        };

        let mut in_flight = 0usize;
        for &i in dispatched {
            // SAFETY: `i` came from an enumeration of the same vector this
            // pointer was taken from, so it is in bounds; `ready` holds each
            // index at most once, so no two jobs alias; and the barrier below
            // keeps the pointee alive and untouched by the coordinator until
            // every job has reported.
            let job = Job {
                index: i,
                node: unsafe { nodes_ptr.add(i) },
            };
            match self.job_tx.as_ref() {
                // `try_send` cannot report `Full` (capacity >= node count), and
                // reports `Disconnected` only if every worker has died. Both
                // fall back to running the job here rather than panicking:
                // degraded to serial is still a running robot.
                Some(tx) => match tx.try_send(job) {
                    Ok(()) => in_flight += 1,
                    Err(err) => out.push(run_job(err.into_inner(), monitors)),
                },
                None => out.push(run_job(job, monitors)),
            }
        }

        // SAFETY: as above — `inline_index` is in bounds and distinct from
        // every dispatched index.
        let inline_job = Job {
            index: inline_index,
            node: unsafe { nodes_ptr.add(inline_index) },
        };
        out.push(run_job(inline_job, monitors));

        for _ in 0..in_flight {
            match self.result_rx.recv() {
                Ok(result) => out.push(result),
                Err(_) => {
                    // Every worker thread is gone. Nothing still holds a job,
                    // so releasing the barrier here is safe; the missing
                    // results simply go unprocessed this cycle.
                    print_line(
                        "[Compute] Worker pool disconnected mid-cycle — results for this \
                         cycle are incomplete; remaining nodes run on the compute thread",
                    );
                    break;
                }
            }
        }

        out.sort_unstable_by_key(|r| r.index);
    }

    /// Stop the workers. Best-effort and time-boxed.
    ///
    /// Safety does not depend on this join: `run_cycle` guarantees no worker
    /// holds a [`Job`] once the barrier has released, and the coordinator only
    /// reaches shutdown between cycles, so no worker can be pointing into the
    /// node vector by the time this runs.
    fn shutdown(mut self) {
        // Dropping the only `Sender<Job>` disconnects the channel and wakes
        // every parked worker with `Err(Disconnected)`.
        self.job_tx = None;

        let deadline = Instant::now() + WORKER_JOIN_BUDGET;
        let mut pending: Vec<JoinHandle<()>> = std::mem::take(&mut self.workers);
        while !pending.is_empty() {
            let mut still_running = Vec::with_capacity(pending.len());
            for handle in pending {
                if handle.is_finished() {
                    let _ = handle.join();
                } else {
                    still_running.push(handle);
                }
            }
            pending = still_running;
            if pending.is_empty() {
                break;
            }
            if Instant::now() >= deadline {
                // A worker is still inside a node's `tick()`. It is detached
                // rather than waited on — the same bargain `join_with_timeout`
                // makes for executor threads, and shutdown must not be held
                // hostage by one stalled node.
                print_line(&format!(
                    "[Compute] {} worker(s) did not exit within {:?} — detaching \
                     (possible stalled tick)",
                    pending.len(),
                    WORKER_JOIN_BUDGET
                ));
                break;
            }
            std::thread::sleep(WORKER_JOIN_POLL);
        }
    }
}

/// Parallel compute executor for non-RT nodes.
///
/// Owns compute nodes and ticks them in parallel on a dedicated thread.
/// Shutdown is coordinated via the shared `running` flag.
pub(crate) struct ComputeExecutor {
    handle: Option<std::thread::JoinHandle<Vec<RegisteredNode>>>,
}

impl ComputeExecutor {
    /// Start the compute executor with the given nodes.
    ///
    /// Spawns the coordinator thread, which creates the persistent worker pool
    /// once and then runs the tick loop. `tick_period` controls the cadence of
    /// the compute loop (derived from the scheduler's tick rate).
    pub fn start(
        mut nodes: Vec<RegisteredNode>,
        running: Arc<AtomicBool>,
        tick_period: Duration,
        monitors: SharedMonitors,
    ) -> Self {
        nodes.sort_by_key(|n| n.priority);

        let handle = std::thread::Builder::new()
            .name("horus-compute".to_string())
            .spawn(move || Self::compute_thread_main(nodes, running, tick_period, monitors))
            .expect("Failed to spawn compute thread");

        Self {
            handle: Some(handle),
        }
    }

    /// Stop the compute executor and reclaim its nodes.
    pub fn stop(mut self) -> Vec<RegisteredNode> {
        // Degrade instead of panicking, matching EventExecutor::stop.
        //
        // Both `expect`s used to fire on the MAIN thread during teardown: a
        // panic anywhere inside a compute node's tick killed only that
        // executor thread (there is no `panic = "abort"` in the release
        // profile), and the panic then re-surfaced here — turning an orderly
        // scheduler shutdown into a panic at exactly the moment a robot is
        // trying to stop. Any remaining shutdown work, including safing other
        // nodes, was skipped.
        //
        // The nodes owned by a panicked thread cannot be reclaimed, so an empty
        // Vec is the honest result; it is reported rather than swallowed.
        let Some(handle) = self.handle.take() else {
            print_line("[Compute] Warning: thread handle already consumed — nothing to join");
            return Vec::new();
        };
        // Bounded join, matching the guarantee RtExecutor::stop documents. A
        // bare `handle.join()` here hung the entire shutdown whenever a compute
        // node blocked inside `tick()`, because this loop only re-checks
        // `running` between ticks — and `run_with_filter` calls this before it
        // shuts down or safes any other node.
        super::primitives::join_with_timeout(
            handle,
            "Compute",
            super::primitives::SHUTDOWN_TIMEOUT_PER_THREAD,
        )
        .unwrap_or_default()
    }

    /// Main function for the compute coordinator thread.
    fn compute_thread_main(
        mut nodes: Vec<RegisteredNode>,
        running: Arc<AtomicBool>,
        tick_period: Duration,
        monitors: SharedMonitors,
    ) -> Vec<RegisteredNode> {
        // Before the workers exist, so they inherit the class rather than each
        // having to change it. No nice increment here: the coordinator owns the
        // tick barrier, the load-shed accounting and the `running` re-check
        // that ends shutdown, and delaying those delays every compute node.
        set_best_effort_class("coordinator", 0);

        let pool = ComputePool::new(nodes.len(), &monitors);

        print_line(&format!(
            "[Compute] Started with {} nodes, {} persistent worker(s) + coordinator, tick period {:?}",
            nodes.len(),
            pool.workers.len(),
            tick_period
        ));

        // Load shedding state: tracks whether we're currently shedding
        let mut shedding_active = false;
        let mut cooldown_remaining: u32 = 0;

        // Hoisted out of the loop: both were allocated and freed every cycle,
        // which is malloc traffic (and arena locking) at the tick rate for
        // buffers whose maximum size is known at startup.
        let mut ready_indices: Vec<usize> = Vec::with_capacity(nodes.len());
        let mut cycle_results: Vec<ParallelResult> = Vec::with_capacity(nodes.len());

        while running.load(Ordering::Relaxed) {
            let loop_start = Instant::now();

            // Classify which nodes should tick this cycle
            ready_indices.clear();
            for (i, node) in nodes.iter().enumerate() {
                if !node.initialized
                    || node.is_stopped
                    || node.is_paused
                    || !node.failure_policy_allows_tick()
                {
                    continue;
                }

                // Per-node rate limiting
                if let Some(rate_hz) = node.rate_hz {
                    if let Some(last_tick) = node.last_tick {
                        let elapsed = loop_start.duration_since(last_tick).as_secs_f64();
                        if rate_hz > 0.0 && elapsed < 1.0 / rate_hz {
                            continue;
                        }
                    }
                }

                // Load shedding: skip sheddable nodes when under overload
                if shedding_active && node.priority >= SHED_THRESHOLD {
                    continue;
                }

                ready_indices.push(i);
            }

            // Safing requested by the main thread's watchdog ladder. Applied
            // to EVERY node this executor owns, not just the ones ticking this
            // pass — an Isolated node is precisely one that is not ticking.
            //
            // ABOVE the `ready_indices.is_empty()` early-continue, not below
            // it. Below, this loop was skipped on exactly the pass the comment
            // describes: when nothing is ready, which is the state a set of
            // isolated or rate-limited nodes is permanently in. A link-loss
            // `request_safe_state_all()` under `safety.on_link_lost =
            // "safe_state"` — the path that deliberately does not latch an
            // e-stop and leaves the scheduler running — would then never be
            // consumed by this executor's nodes at all.
            for node in nodes.iter_mut() {
                super::primitives::honor_safe_state_request(node, &monitors);
                super::primitives::honor_restart_request(node, &monitors);
            }

            if ready_indices.is_empty() {
                // Nothing to do — sleep and check again
                let elapsed = loop_start.elapsed();
                if elapsed < tick_period {
                    std::thread::sleep(tick_period - elapsed);
                }
                continue;
            }

            // Update last_tick for rate-limited nodes
            let now = Instant::now();
            for &i in &ready_indices {
                if nodes[i].rate_hz.is_some() {
                    nodes[i].last_tick = Some(now);
                }
                // Start context tick
                if let Some(ref mut ctx) = nodes[i].context {
                    ctx.start_tick();
                }
            }

            // Begin recording for all ready nodes
            for &i in &ready_indices {
                if let Some(ref mut recorder) = nodes[i].recorder {
                    recorder.begin_tick(0);
                }
            }

            // Tick this cycle's nodes on the pool (plus this thread) and block
            // until all of them have finished. Nothing below may touch `nodes`
            // until `run_cycle` returns — that is the borrow the raw pointers
            // stand in for.
            let nodes_ptr = nodes.as_mut_ptr();
            pool.run_cycle(nodes_ptr, &ready_indices, &monitors, &mut cycle_results);

            // Process results sequentially, in node order. Note the slowest
            // node on the way past: `run_cycle` is a barrier, so the cycle costs
            // whatever the slowest node cost, and when that overruns the period
            // the only actionable fact is which node it was.
            let mut slowest: Option<(usize, std::time::Duration)> = None;
            for pr in cycle_results.drain(..) {
                if slowest.is_none_or(|(_, d)| pr.duration > d) {
                    slowest = Some((pr.index, pr.duration));
                }
                let tr = super::primitives::TickResult {
                    tick_start: pr.tick_start,
                    duration: pr.duration,
                    result: pr.result,
                };
                Self::process_node_result(&mut nodes[pr.index], tr, &running, &monitors);
            }

            // Update load shedding state based on this cycle's duration.
            // Uses hysteresis: shedding activates immediately on overload but
            // requires SHED_COOLDOWN_CYCLES consecutive under-budget cycles to deactivate.
            let elapsed = loop_start.elapsed();
            if elapsed > tick_period {
                if !shedding_active {
                    let shed_count = nodes
                        .iter()
                        .filter(|n| n.priority >= SHED_THRESHOLD)
                        .count();

                    // `run_cycle` is a barrier: every compute node in this cycle
                    // now runs at the slowest one's rate, whether or not anything
                    // is sheddable. This message used to be inside
                    // `if shed_count > 0`, so a pool with no background nodes --
                    // every node below SHED_THRESHOLD, which is the normal shape
                    // for perception plus a learned policy -- overran its period
                    // in complete silence. A 100 ms policy node quietly pulls a
                    // 30 Hz vision node down to 10 Hz and nothing says so.
                    let culprit = slowest
                        .map(|(i, d)| format!(" — slowest was '{}' at {:?}", nodes[i].name, d))
                        .unwrap_or_default();
                    if shed_count > 0 {
                        print_line(&format!(
                            "[Compute] Overload detected (cycle took {:?} > {:?}), shedding {} background nodes (order >= {}){}",
                            elapsed, tick_period, shed_count, SHED_THRESHOLD, culprit
                        ));
                    } else {
                        print_line(&format!(
                            "[Compute] Overload: cycle took {:?} > {:?}, and no node is \
                             sheddable (all below order {}). Every compute node is now \
                             running at this cycle's rate{}",
                            elapsed, tick_period, SHED_THRESHOLD, culprit
                        ));
                    }
                    shedding_active = true;
                }
                // Reset cooldown on every overload cycle
                cooldown_remaining = SHED_COOLDOWN_CYCLES;
            } else if shedding_active {
                // Under budget — count down cooldown before resuming
                cooldown_remaining = cooldown_remaining.saturating_sub(1);
                if cooldown_remaining == 0 {
                    print_line("[Compute] Load normalized, resuming all nodes");
                    shedding_active = false;
                }
            }

            // Sleep until next tick period
            if elapsed < tick_period {
                std::thread::sleep(tick_period - elapsed);
            }
        }

        // Workers are idle here (the barrier released before the loop condition
        // was re-checked), so this only has to wake them and reap them.
        pool.shutdown();

        print_line(&format!(
            "[Compute] Stopped ({} nodes returning to scheduler)",
            nodes.len()
        ));

        nodes
    }

    /// Process the result of a single node tick.
    fn process_node_result(
        node: &mut RegisteredNode,
        tr: super::primitives::TickResult,
        running: &Arc<AtomicBool>,
        monitors: &SharedMonitors,
    ) {
        // Record execution stats
        if let Some(ref mut stats) = node.rt_stats {
            stats.record_execution(tr.duration);
        }

        // Profiler recording (shared with main thread)
        // Use try_lock to avoid priority inversion — skip if contended or poisoned
        if let Ok(mut profiler) = monitors.profiler.try_lock() {
            profiler.record(&node.name, tr.duration);
        }

        // End recording tick
        if let Some(ref mut recorder) = node.recorder {
            recorder.end_tick(tr.duration.as_nanos() as u64);
        }

        // Update live SHM registry (~5ns atomic writes)
        monitors.update_registry(node, tr.duration.as_nanos() as u64);

        match tr.result {
            Ok(_) => {
                if let Some(ref mut ctx) = node.context {
                    ctx.record_tick();
                }
                node.record_tick_success();
                // FIX #2: feed the watchdog after a successful tick, gated on the
                // main loop's critical-node condition (mod.rs:3555). Reached only
                // when the crossbeam child tick RETURNED (a hung child blocks the
                // scope so this never runs → hang-detection preserved); the Err
                // arm below is skipped so a panic does not feed either.
                if node.is_rt_node || node.node_watchdog.is_some() {
                    if let Some(ref feeder) = monitors.watchdog {
                        feeder.feed(&node.name);
                    }
                }
            }
            Err(panic_err) => {
                if let Ok(mut profiler) = monitors.profiler.try_lock() {
                    profiler.record_node_failure(&node.name);
                }
                let error_msg = if let Some(s) = panic_err.downcast_ref::<&str>() {
                    format!("[Compute] Node '{}' panicked: {}", node.name, s)
                } else if let Some(s) = panic_err.downcast_ref::<String>() {
                    format!("[Compute] Node '{}' panicked: {}", node.name, s)
                } else {
                    format!("[Compute] Node '{}' panicked (unknown)", node.name)
                };
                // Count the failure. Only the Ok arm recorded metrics, so a node
                // panicking on every tick reported `Health: Healthy, Errors: 0,
                // Total Ticks: 0` while its P99 timing was recorded correctly —
                // making the zero read as "idle" rather than "dead".
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

                // Record to the blackbox so the flight recorder can see a crash.
                // try_lock mirrors the RT path: never block an executor on it.
                if let Some(ref bb) = monitors.blackbox {
                    if let Ok(mut bb) = bb.try_lock() {
                        bb.record(super::blackbox::BlackBoxEvent::NodeError {
                            name: node.name.to_string(),
                            error: error_msg.clone(),
                            severity: crate::error::Severity::Fatal,
                        });
                    }
                }

                // `record_tick_failure` above already logged this at error level, which
                // reaches both the console and the buffer `horus log` reads. This second
                // copy was gated on `verbose` on the theory that verbose is opt-in — but
                // `MonitoringConfig::verbose` defaults to *true*, so every default run
                // printed the panic twice on two different streams (hlog to stderr, this to
                // stdout), and a third time via the old `Node::on_error` default. With a
                // Python node's traceback attached that is three multi-line blocks for one
                // failure.
                //
                // Panic-guarded: `process_node_result` runs on the compute
                // coordinator thread outside any catch_unwind (the one in
                // `run_job` wraps the tick only), so a bare panic in this
                // advisory callback killed the whole executor thread — and with
                // it every healthy node it owns — while `run_for` still
                // returned Ok and `stop()` reclaimed nothing.
                if super::primitives::guard_fault_callback(|| node.node.on_error(&error_msg)) {
                    print_line(&format!(
                        "[Compute] Node '{}' also panicked in on_error() — ignoring (advisory callback)",
                        node.name
                    ));
                }

                // Enforce the failure policy (Fatal → safe node + stop scheduler
                // via shared `running`; Restart → re-init; Skip/Ignore → gated).
                if node.apply_failure_policy_after_panic() {
                    running.store(false, Ordering::SeqCst);
                }
            }
        }
    }
}

impl Drop for ComputeExecutor {
    fn drop(&mut self) {
        // Bounded here too: an early return or a panic can drop the executor
        // without ever calling `stop()`, and an unbounded join on that path
        // hangs exactly as badly.
        if let Some(handle) = self.handle.take() {
            let _ = super::primitives::join_with_timeout(
                handle,
                "Compute",
                super::primitives::SHUTDOWN_TIMEOUT_PER_THREAD,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::duration_ext::DurationExt;
    use crate::core::{Miss, Node, NodeInfo};
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

    fn make_compute_node(name: &str, count: Arc<std::sync::atomic::AtomicU64>) -> RegisteredNode {
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
            is_rt_node: false,
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
            execution_class: super::super::types::ExecutionClass::Compute,
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
    fn test_compute_executor_runs_nodes() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let nodes = vec![make_compute_node("compute_1", count.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = ComputeExecutor::start(nodes, running.clone(), 1_u64.ms(), test_monitors());

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 1);
        assert!(count.load(std::sync::atomic::Ordering::Relaxed) > 0);
    }

    /// Regression: a panic in `on_error()` must not take the compute thread
    /// with it.
    ///
    /// `process_node_result` runs on the coordinator thread outside any
    /// `catch_unwind` — the guard in `run_job` covers the tick only — so a bare
    /// `on_error()` panic unwound the coordinator, stopping every healthy node
    /// it owns, and `stop()` came back empty because a panicked thread's nodes
    /// cannot be reclaimed. One misbehaving node's advisory callback must not
    /// be able to do that.
    #[test]
    fn on_error_panic_does_not_kill_the_compute_thread() {
        use std::sync::atomic::AtomicU64;
        use std::sync::atomic::Ordering::Relaxed;

        struct DoubleBoom {
            on_error_calls: Arc<AtomicU64>,
        }
        impl Node for DoubleBoom {
            fn name(&self) -> &str {
                "double_boom"
            }
            fn tick(&mut self) {
                panic!("tick boom");
            }
            fn on_error(&mut self, _msg: &str) {
                // Count BEFORE panicking: this is the test's evidence that the
                // coordinator actually entered the guarded callback, so the
                // wait below cannot pass by simply never getting there.
                self.on_error_calls.fetch_add(1, Relaxed);
                panic!("on_error boom");
            }
        }

        // Wait on an observed count, never on a fixed sleep. The deadline is an
        // upper bound on "this will never happen", not the thing being
        // measured, so a loaded machine makes this test slower rather than
        // flaky; the failure it reports is a real stall of the coordinator.
        fn wait_for(what: &str, cond: impl Fn() -> bool) {
            let deadline = Instant::now() + 5_u64.secs();
            while Instant::now() < deadline {
                if cond() {
                    return;
                }
                std::thread::sleep(1_u64.ms());
            }
            panic!("timed out after 5s waiting for {what}");
        }

        let ticks = Arc::new(AtomicU64::new(0));
        let on_error_calls = Arc::new(AtomicU64::new(0));
        let mut boom = make_compute_node("double_boom", ticks.clone());
        // Only the healthy node ever increments `ticks` — DoubleBoom replaces
        // the CounterNode that the handle was made for.
        boom.node = super::super::types::NodeKind::new(Box::new(DoubleBoom {
            on_error_calls: on_error_calls.clone(),
        }));
        let nodes = vec![boom, make_compute_node("healthy", ticks.clone())];
        let running = Arc::new(AtomicBool::new(true));

        let executor = ComputeExecutor::start(nodes, running.clone(), 1_u64.ms(), test_monitors());

        // 1. The coordinator has run the panicking on_error at least once.
        wait_for("the first on_error() panic", || {
            on_error_calls.load(Relaxed) > 0
        });
        // 2. The healthy node ticks AFTER that panic. Pre-fix the coordinator
        //    has already unwound by this point and this never advances.
        let before = ticks.load(Relaxed);
        wait_for(
            "the healthy node to tick again after its neighbour panicked in on_error()",
            || ticks.load(Relaxed) > before,
        );

        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        // The deterministic half: a panicked thread's nodes cannot be
        // reclaimed, so pre-fix this is 0 of 2 regardless of timing.
        assert_eq!(
            returned.len(),
            2,
            "the compute thread died in on_error() — its nodes were never reclaimed"
        );
    }

    #[test]
    fn test_compute_executor_parallel_nodes() {
        let count1 = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count2 = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let count3 = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let nodes = vec![
            make_compute_node("compute_a", count1.clone()),
            make_compute_node("compute_b", count2.clone()),
            make_compute_node("compute_c", count3.clone()),
        ];
        let running = Arc::new(AtomicBool::new(true));

        let executor = ComputeExecutor::start(nodes, running.clone(), 1_u64.ms(), test_monitors());

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 3);
        assert!(count1.load(std::sync::atomic::Ordering::Relaxed) > 0);
        assert!(count2.load(std::sync::atomic::Ordering::Relaxed) > 0);
        assert!(count3.load(std::sync::atomic::Ordering::Relaxed) > 0);
    }

    #[test]
    fn test_compute_executor_rate_limiting() {
        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let mut node = make_compute_node("rate_limited", count.clone());
        node.rate_hz = Some(10.0); // 10 Hz = ~100ms between ticks

        let running = Arc::new(AtomicBool::new(true));
        let executor = ComputeExecutor::start(
            vec![node],
            running.clone(),
            1_u64.ms(), // Pool ticks at 1kHz but node only at 10Hz
            test_monitors(),
        );

        std::thread::sleep(250_u64.ms());
        running.store(false, Ordering::SeqCst);
        let _returned = executor.stop();

        let ticks = count.load(std::sync::atomic::Ordering::Relaxed);
        // At 10Hz for 250ms, expect ~2-3 ticks (not hundreds)
        assert!(
            (1..=5).contains(&ticks),
            "Expected 1-5 ticks at 10Hz in 250ms, got {}",
            ticks
        );
    }

    // ========================================================================
    // FIX #5: executor installs the per-tick thread-local context
    // ========================================================================

    /// Records the ambient `horus::dt()` observed during its tick — proves the
    /// context is set on the thread that actually runs the tick (the
    /// thread-local would be unset if it were set on the dispatching thread
    /// instead).
    struct CtxProbeNode {
        name: String,
        observed_dt: Arc<Mutex<Option<Duration>>>,
    }
    impl Node for CtxProbeNode {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            *self.observed_dt.lock().unwrap() = Some(crate::core::tick_context::ctx_dt());
        }
    }

    #[test]
    fn test_compute_executor_installs_tick_context_on_child_threads() {
        // Two ready nodes force the dispatch path: `run_cycle` sends the first
        // to a pooled WORKER and runs the last on the coordinator, so this
        // covers both lanes at once. Each probe must still see its own
        // dt() == 1/rate. RED (no FIX #5): dt() == ZERO on the worker.
        let dt_a = Arc::new(Mutex::new(None));
        let dt_b = Arc::new(Mutex::new(None));

        let mut reg_a =
            make_compute_node("probe_a", Arc::new(std::sync::atomic::AtomicU64::new(0)));
        reg_a.node = super::super::types::NodeKind::new(Box::new(CtxProbeNode {
            name: "probe_a".to_string(),
            observed_dt: dt_a.clone(),
        }));
        reg_a.rate_hz = Some(500.0);

        let mut reg_b =
            make_compute_node("probe_b", Arc::new(std::sync::atomic::AtomicU64::new(0)));
        reg_b.node = super::super::types::NodeKind::new(Box::new(CtxProbeNode {
            name: "probe_b".to_string(),
            observed_dt: dt_b.clone(),
        }));
        reg_b.rate_hz = Some(500.0);

        let running = Arc::new(AtomicBool::new(true));
        let executor = ComputeExecutor::start(
            vec![reg_a, reg_b],
            running.clone(),
            1_u64.ms(),
            test_monitors(),
        );

        // Give the pool time to run several parallel cycles.
        for _ in 0..100 {
            if dt_a.lock().unwrap().is_some() && dt_b.lock().unwrap().is_some() {
                break;
            }
            std::thread::sleep(5_u64.ms());
        }
        running.store(false, Ordering::SeqCst);
        let _ = executor.stop();

        let observed_a = dt_a.lock().unwrap().expect("probe_a dt must be recorded");
        let observed_b = dt_b.lock().unwrap().expect("probe_b dt must be recorded");
        let expected = Duration::from_secs_f64(1.0 / 500.0);
        assert_eq!(
            observed_a, expected,
            "probe_a dt() on child thread must be 1/rate"
        );
        assert_eq!(
            observed_b, expected,
            "probe_b dt() on child thread must be 1/rate"
        );
    }

    // ========================================================================
    // FIX #2: executor feeds the watchdog only for critical nodes
    // ========================================================================

    use super::super::safety_monitor::{SafetyMonitor, WatchdogFeeder};

    fn monitors_with_watchdog(feeder: WatchdogFeeder) -> SharedMonitors {
        let mut m = test_monitors();
        m.watchdog = Some(feeder);
        m
    }

    /// A NON-critical node (is_rt_node=false, no `.watchdog()`) must not be fed
    /// even if a watchdog exists for its name — guards against blanket-feeding
    /// (mirrors the main loop's `is_rt_node || node_watchdog.is_some()` gate).
    #[test]
    fn test_compute_executor_does_not_feed_non_critical() {
        let monitor = SafetyMonitor::new(10);
        monitor.add_critical_node("plain_compute".to_string(), Duration::from_secs(5));
        let h0 = monitor
            .watchdog_last_heartbeat_ns("plain_compute")
            .expect("watchdog registered");

        let count = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let mut reg = make_compute_node("plain_compute", count.clone());
        reg.rate_hz = Some(1000.0); // is_rt_node stays false, node_watchdog None

        let running = Arc::new(AtomicBool::new(true));
        let executor = ComputeExecutor::start(
            vec![reg],
            running.clone(),
            1_u64.ms(),
            monitors_with_watchdog(monitor.watchdog_feeder()),
        );

        std::thread::sleep(50_u64.ms());
        running.store(false, Ordering::SeqCst);
        let _ = executor.stop();

        assert!(
            count.load(std::sync::atomic::Ordering::Relaxed) > 0,
            "node must have ticked"
        );
        let h1 = monitor
            .watchdog_last_heartbeat_ns("plain_compute")
            .expect("watchdog registered");
        assert_eq!(
            h1, h0,
            "a non-critical node must NOT be fed by the executor"
        );
    }

    // ========================================================================
    // The pool is a POOL: threads are created once, not once per node per tick
    // ========================================================================

    /// Records the id of the thread each of its ticks ran on.
    struct ThreadIdProbe {
        name: String,
        seen: Arc<Mutex<std::collections::HashSet<std::thread::ThreadId>>>,
        ticks: Arc<std::sync::atomic::AtomicU64>,
    }
    impl Node for ThreadIdProbe {
        fn name(&self) -> &str {
            &self.name
        }
        fn tick(&mut self) {
            self.seen
                .lock()
                .unwrap()
                .insert(std::thread::current().id());
            self.ticks
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
    }

    /// The regression this file exists to prevent: the executor used to call
    /// `crossbeam::scope` + `s.spawn` once per ready node per cycle, creating
    /// and destroying real OS threads at the tick rate — an `mmap`/`munmap`
    /// pair per node per cycle, each taking the process-wide mmap lock that the
    /// RT thread's own page faults contend for.
    ///
    /// `ThreadId`s are documented never to be reused, even after a thread
    /// exits, so they are an exact, allocation-free probe: a real pool reuses a
    /// bounded set of them however many cycles run, while the scoped version
    /// produced a fresh pair every cycle (hundreds over this test's window).
    #[test]
    fn test_compute_executor_reuses_pool_threads_across_cycles() {
        let seen = Arc::new(Mutex::new(std::collections::HashSet::new()));
        let ticks = Arc::new(std::sync::atomic::AtomicU64::new(0));

        let mut nodes = Vec::new();
        for name in ["pooled_a", "pooled_b"] {
            let mut reg = make_compute_node(name, Arc::new(std::sync::atomic::AtomicU64::new(0)));
            reg.node = super::super::types::NodeKind::new(Box::new(ThreadIdProbe {
                name: name.to_string(),
                seen: seen.clone(),
                ticks: ticks.clone(),
            }));
            nodes.push(reg);
        }

        let running = Arc::new(AtomicBool::new(true));
        let executor = ComputeExecutor::start(nodes, running.clone(), 1_u64.ms(), test_monitors());

        // Wait for enough cycles that a per-cycle-spawn implementation could
        // not possibly stay under the bound below, with a ceiling so a loaded
        // CI box cannot hang the test.
        const MIN_TICKS: u64 = 40;
        for _ in 0..200 {
            if ticks.load(std::sync::atomic::Ordering::Relaxed) >= MIN_TICKS {
                break;
            }
            std::thread::sleep(10_u64.ms());
        }
        running.store(false, Ordering::SeqCst);
        let returned = executor.stop();

        assert_eq!(returned.len(), 2, "both nodes must come back");
        let observed = ticks.load(std::sync::atomic::Ordering::Relaxed);
        assert!(
            observed >= MIN_TICKS,
            "expected at least {} ticks to exercise many cycles, got {observed}",
            MIN_TICKS
        );

        // Two ready nodes per cycle: the coordinator runs one and a single
        // pooled worker runs the other, so two distinct ids is the expected
        // steady state. The bound is loose enough to tolerate a differently
        // sized pool but far below `observed / 2` cycles' worth of fresh
        // threads.
        let distinct = seen.lock().unwrap().len();
        assert!(
            distinct <= 4,
            "compute ticks ran on {distinct} distinct threads over {} cycles — the pool is \
             creating threads per cycle instead of reusing persistent workers",
            observed / 2
        );
    }
}
