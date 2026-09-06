//! Recording must not map every topic and walk the registry on every tick.
//!
//! With recording enabled, the capture path did, per recording node per tick:
//! one read-lock on the process-global topic registry plus a full
//! O(topics-in-process) walk that allocated a `Vec` and two `String`s per
//! match — and then, for every topic it found, an `open` + `fstat` + `mmap` +
//! `munmap` + `close`. Five syscalls per topic, plus at least two minor page
//! faults, because unmapping tears down the PTEs and the header and payload
//! pages fault back in next tick. Twice over: once for inputs, once for
//! outputs.
//!
//! None of it changes between ticks.
//!
//! # Why counters and not timing
//!
//! "Zero maps per tick" either holds or it does not, with no statistics and no
//! need for a quiet machine — and this box's wake jitter is ~253 µs mean, which
//! would swamp the effect in any wall-clock measurement. Both counters are
//! process-wide and monotonic, in the style of
//! `rt_allocator::violation_count()`.

use horus_core::communication::{shm_map_count, Topic};
use horus_core::core::{DurationExt, Node};
use horus_core::error::Result;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

/// Publishes and subscribes, so it has taps on both sides.
struct Chatty {
    ticks: Arc<AtomicU64>,
    out: Option<Topic<u64>>,
    inp: Option<Topic<u64>>,
}

impl Node for Chatty {
    fn name(&self) -> &'static str {
        "chatty"
    }
    fn init(&mut self) -> Result<()> {
        self.out = Some(Topic::new("rec_storm_out")?);
        self.inp = Some(Topic::new("rec_storm_in")?);
        Ok(())
    }
    fn tick(&mut self) {
        let n = self.ticks.fetch_add(1, Ordering::Relaxed);
        if let Some(ref t) = self.out {
            t.send(n);
        }
        if let Some(ref t) = self.inp {
            let _ = t.try_recv();
        }
    }
}

#[test]
fn a_recording_node_does_not_remap_its_topics_every_tick() {
    let ticks = Arc::new(AtomicU64::new(0));

    let mut sched = Scheduler::new().tick_rate(500_u64.hz()).with_recording();
    sched
        .add(Chatty {
            ticks: Arc::clone(&ticks),
            out: None,
            inp: None,
        })
        .rate(500_u64.hz())
        .build()
        .unwrap();

    // Measured across the run only: startup maps and scans plenty, and this is
    // about the per-tick cost, not the one-time one.
    let maps_before = shm_map_count();
    let scans_before = horus_core::communication::TopicNodeRegistry::scan_count();
    sched.run_for(400_u64.ms()).unwrap();
    let maps = shm_map_count() - maps_before;
    let scans = horus_core::communication::TopicNodeRegistry::scan_count() - scans_before;

    let n = ticks.load(Ordering::Relaxed);
    assert!(
        n >= 40,
        "the node must actually have ticked for the counts below to mean \
         anything; saw {n}"
    );

    // Two taps and two capture sites, so the un-cached path costs exactly 2
    // scans per tick — measured at 384 over 188 ticks with the cache ablated.
    // With it, this run measured 12 scans and 1 map over 186 ticks: bounded by
    // the topology, not by the tick count. The ceiling leaves ~4x headroom over
    // that rather than asserting the exact numbers, which would make the test a
    // record of one machine.
    let ceiling = (n / 4).max(4);
    assert!(
        scans < ceiling,
        "the topic registry was walked {scans} times over {n} ticks — the tap \
         lists are being rebuilt every tick instead of on a version change"
    );
    assert!(
        maps < ceiling,
        "topic regions were mapped {maps} times over {n} ticks — the recorder is \
         re-opening and re-mmapping the same topics every tick instead of \
         holding the mapping"
    );
}
