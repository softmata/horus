//! The main tick loop must not create and destroy OS threads every tick.
//!
//! `execute_ready_dispatch` is the per-tick path for any graph with more than
//! one node — which is every program with two plain nodes, because
//! `ExecutionClass` defaults to `BestEffort`. It used to open a
//! `crossbeam::scope`, spawn `min(nodes, available_parallelism)` threads, and
//! join them, on every tick.
//!
//! Measured on the reference box: 129.6 us to create and join a 4-worker scope,
//! 246.6 us for 8, against ~66 syscalls per scope. The `mmap`/`munmap` pairs
//! matter beyond their own cost — each takes the process-wide `mmap_lock` in
//! write mode, which an RT thread's first-touch page faults need in read mode,
//! so the main loop's thread churn was periodically blocking the real-time
//! threads it shares a process with.
//!
//! # Why this counts thread identities rather than timing anything
//!
//! A wall-clock assertion on this machine would measure the host scheduler:
//! SCHED_FIFO is not grantable here and wake jitter is ~253 us mean, three
//! orders of magnitude above the effect. Thread identity is exact, needs no
//! quiet machine, and states the property directly — a pool reuses its lanes,
//! a per-tick scope cannot.

use horus_core::core::{DurationExt, Node};
use horus_core::error::Result;
use horus_core::scheduling::Scheduler;
use std::collections::HashSet;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

/// Records which OS thread ticked it, and how many times.
struct ThreadWatcher {
    name: &'static str,
    seen: Arc<Mutex<HashSet<std::thread::ThreadId>>>,
    ticks: Arc<AtomicU64>,
}

impl Node for ThreadWatcher {
    fn name(&self) -> &'static str {
        self.name
    }
    fn init(&mut self) -> Result<()> {
        Ok(())
    }
    fn tick(&mut self) {
        self.seen
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .insert(std::thread::current().id());
        self.ticks.fetch_add(1, Ordering::Relaxed);
    }
}

#[test]
fn the_ready_dispatch_reuses_its_threads_across_ticks() {
    let seen = Arc::new(Mutex::new(HashSet::new()));
    let ticks = Arc::new(AtomicU64::new(0));

    let mut sched = Scheduler::new().tick_rate(1000_u64.hz());
    for name in ["watcher_a", "watcher_b", "watcher_c", "watcher_d"] {
        sched
            .add(ThreadWatcher {
                name,
                seen: Arc::clone(&seen),
                ticks: Arc::clone(&ticks),
            })
            .build()
            .unwrap();
    }
    sched.run_for(300_u64.ms()).unwrap();

    let total_ticks = ticks.load(Ordering::Relaxed);
    let distinct = seen.lock().unwrap_or_else(|e| e.into_inner()).len();

    // Not a timing assertion: this only establishes that enough ticks happened
    // for "distinct threads" to mean something. Without it the test would pass
    // on a scheduler that never ran.
    assert!(
        total_ticks >= 40,
        "expected the graph to tick many times in 300ms, saw {total_ticks} — the \
         count below would prove nothing"
    );

    // Four nodes, so at most four lanes plus the coordinator running one job
    // inline. A per-tick scope would instead show roughly `ticks * workers`
    // distinct ids: at 4 nodes and ~75 ticks that is several hundred.
    assert!(
        distinct <= 5,
        "the dispatch created {distinct} distinct threads over {total_ticks} node \
         ticks — the lanes are not being reused, so each tick is paying a clone(2) \
         and an mmap/munmap pair per worker"
    );
    assert!(
        distinct >= 1,
        "no thread ever ticked a node: {total_ticks} ticks over {distinct} threads"
    );
}

/// Every node must still tick, in dependency order, once per cycle. A pool that
/// loses a job is worse than a pool that costs a clone.
#[test]
fn every_node_still_ticks_on_the_pooled_dispatch() {
    let seen = Arc::new(Mutex::new(HashSet::new()));
    let counters: Vec<Arc<AtomicU64>> = (0..4).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut sched = Scheduler::new().tick_rate(500_u64.hz());
    for (i, name) in ["n0", "n1", "n2", "n3"].iter().enumerate() {
        sched
            .add(ThreadWatcher {
                name,
                seen: Arc::clone(&seen),
                ticks: Arc::clone(&counters[i]),
            })
            .build()
            .unwrap();
    }
    sched.run_for(200_u64.ms()).unwrap();

    let counts: Vec<u64> = counters.iter().map(|c| c.load(Ordering::Relaxed)).collect();
    assert!(
        counts.iter().all(|&c| c > 0),
        "every node must tick: {counts:?}"
    );
    // All four are independent and run at the loop rate, so their counts must
    // agree to within one in-flight cycle. A dropped job shows up here.
    let min = counts.iter().copied().min().unwrap();
    let max = counts.iter().copied().max().unwrap();
    assert!(
        max - min <= 1,
        "the nodes fell out of step ({counts:?}) — a job was dropped or double-dispatched"
    );
}
