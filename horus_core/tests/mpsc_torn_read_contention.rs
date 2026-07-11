//! Parallel-contention harness for the brain-1334 MPSC torn read.
//!
//! History: `partial_write_concurrent_writers_no_partial_data` could return a torn
//! `[u64; 4]` (~1/5) ONLY "under full-suite contention" — many unrelated tests
//! hammering SHM in parallel widened the window — and was therefore un-reproducible
//! and un-verifiable standalone. This test MANUFACTURES that contention in one
//! controlled, repeatable place: many topics × many writers × tiny rings, all
//! started at once, so the ring laps constantly and the writer/reader race is
//! maximal. It is the verification tool the serial suite lacked.
//!
//! # What it asserts
//!
//! Every reader receives only INTACT messages: writers send `[v, v, v, v]` (all four
//! lanes equal), so a torn read — a producer overwriting the slot the reader is
//! mid-copy on — shows up as unequal lanes. Across all topics, ZERO torn reads.
//!
//! # Why it is now green (and the gate)
//!
//! The MP-send CAS claim (`mp_send_no_overshoot_corruption`) makes the capacity gate
//! binding, so a producer never laps onto the slot a reader holds: producer-write
//! slot `head & mask` and consumer-read slot `tail & mask` are always distinct while
//! `head - tail < capacity`. Revert that CAS to a bare `fetch_add` and this test goes
//! RED (torn reads reappear under the same contention) — it is the gate that makes
//! 1334 observable, which serial execution could not.

mod common;

use horus_core::communication::Topic;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Barrier};

#[test]
fn mpsc_no_torn_read_under_parallel_contention() {
    let _shm = common::cleanup_stale_shm();

    // Contention knobs. Tiny rings + many writers per topic maximize lap/overwrite
    // races; many topics running at once reproduce the cross-test SHM/cache pressure
    // that used to be needed to trip the bug. Repeated in ROUNDS: one burst catches a
    // regression only ~2/3 of the time, so several rounds make this a RELIABLE gate
    // (RED on broken code) while staying well under a second on fixed code.
    let n_topics = 8usize;
    let n_writers = 4u64;
    let cap: u32 = 8; // tiny ring → the reader is almost always inside the write window
    let msgs_per_writer = 4000u64;
    let rounds = 6u64;

    let torn_total = Arc::new(AtomicU64::new(0));
    let read_total = Arc::new(AtomicU64::new(0));

    for _round in 0..rounds {
        run_contention_round(
            n_topics,
            n_writers,
            cap,
            msgs_per_writer,
            &torn_total,
            &read_total,
        );
    }

    let torn = torn_total.load(Ordering::Relaxed);
    let read = read_total.load(Ordering::Relaxed);
    eprintln!(
        "torn-read harness: {} rounds × {} topics × {} writers, cap {}, read {} messages, {} torn",
        rounds, n_topics, n_writers, cap, read, torn
    );
    assert!(read > 0, "harness delivered nothing — not exercising the path");
    assert_eq!(
        torn, 0,
        "MPSC torn read under parallel contention: {} of {} reads had mismatched lanes \
         (a producer overwrote a slot mid-copy) — brain 1334",
        torn, read
    );
}

/// One contention burst: `n_topics` topics, each with `n_writers` writers + 1 reader,
/// all started together. Accumulates delivered/torn counts.
fn run_contention_round(
    n_topics: usize,
    n_writers: u64,
    cap: u32,
    msgs_per_writer: u64,
    torn_total: &Arc<AtomicU64>,
    read_total: &Arc<AtomicU64>,
) {
    // +1 reader per topic, all threads gated on one barrier so they collide.
    let threads_per_topic = n_writers as usize + 1;
    let barrier = Arc::new(Barrier::new(n_topics * threads_per_topic));

    let mut handles = Vec::new();
    for t in 0..n_topics {
        let name = format!("torn_harness_{}_{}", std::process::id(), t);

        // Writers.
        for w in 0..n_writers {
            let n = name.clone();
            let b = barrier.clone();
            handles.push(std::thread::spawn(move || {
                let topic: Topic<[u64; 4]> =
                    Topic::with_capacity(&n, cap, None).expect("writer topic");
                b.wait();
                for i in 0..msgs_per_writer {
                    // All four lanes equal a value unique to (writer, i). A torn read
                    // (partial overwrite) makes the lanes disagree.
                    let v = (w << 40) | i;
                    // Fire-and-forget: drops under backpressure are fine; we only care
                    // that whatever IS delivered is never torn.
                    let _ = topic.try_send([v, v, v, v]);
                }
            }));
        }

        // Reader.
        {
            let n = name.clone();
            let b = barrier.clone();
            let torn = torn_total.clone();
            let read = read_total.clone();
            handles.push(std::thread::spawn(move || {
                let topic: Topic<[u64; 4]> =
                    Topic::with_capacity(&n, cap, None).expect("reader topic");
                b.wait();
                let total = n_writers * msgs_per_writer;
                let mut seen = 0u64;
                let mut torn_local = 0u64;
                let mut empty_spins = 0u64;
                // Drain until we have seen `total` deliveries' worth of attempts, or the
                // ring stays empty long enough that the writers are clearly done.
                loop {
                    match topic.try_recv() {
                        Some(arr) => {
                            empty_spins = 0;
                            seen += 1;
                            if arr[0] != arr[1] || arr[1] != arr[2] || arr[2] != arr[3] {
                                torn_local += 1;
                            }
                            if seen >= total {
                                break;
                            }
                        }
                        None => {
                            empty_spins += 1;
                            // Writers send `total` messages fast; after a long empty
                            // streak they are done and the ring is drained.
                            if empty_spins > 2_000_000 {
                                break;
                            }
                            std::hint::spin_loop();
                        }
                    }
                }
                read.fetch_add(seen, Ordering::Relaxed);
                torn.fetch_add(torn_local, Ordering::Relaxed);
            }));
        }
    }

    for h in handles {
        h.join().unwrap();
    }
}
