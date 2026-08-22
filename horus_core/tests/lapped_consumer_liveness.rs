//! A consumer that falls behind must still receive data.
//!
//! `recv_never_reorders_or_duplicates_when_lapped` covers the *ordering* half of
//! the broadcast contract. This covers the liveness half, which was broken in a
//! way ordering tests cannot see: the consumer received nothing at all.
//!
//! The `behind > capacity` branch in `recv_shm_pod_broadcast` used to resume at
//! `tail = head`. `head` names the slot the producer writes *next*, so its
//! ready-flag still belonged to the message from a lap ago, the `v1 < tail + 1`
//! test rejected it, and the call returned `None` — every time, for as long as
//! the consumer stayed behind. Measured on a 264-byte POD topic at 1 kHz:
//!
//!     poll   1 ms -> 6000 of 6000
//!     poll 100 ms ->    0 of 6000, last_seq = 0
//!
//! A 10 Hz node reading a 1 kHz sensor is ordinary multi-rate robotics, and
//! seqlock.rs states the intended contract for exactly that shape: the slow
//! reader should get the most recent data.

use horus_core::communication::Topic;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

/// 264 bytes, so `auto_capacity` clamps the ring to its 16-slot minimum and a
/// lap is reachable in well under a second.
#[repr(C)]
#[derive(Clone, Copy, Serialize, Deserialize)]
struct Wide {
    seq: u64,
    _pad: [u64; 32],
}

#[test]
fn a_lapped_broadcast_consumer_keeps_receiving_and_tracks_the_head() {
    let topic_name = "lapped_liveness.probe";
    let received = Arc::new(AtomicU64::new(0));
    let newest = Arc::new(AtomicU64::new(0));
    let stop = Arc::new(AtomicBool::new(false));

    // Two subscribers select the PodShm broadcast backend.
    let mut readers = Vec::new();
    for _ in 0..2 {
        let (r, n, s) = (received.clone(), newest.clone(), stop.clone());
        readers.push(std::thread::spawn(move || {
            let t: Topic<Wide> = Topic::new(topic_name).expect("subscriber");
            while !s.load(Ordering::Relaxed) {
                while let Some(m) = t.recv() {
                    r.fetch_add(1, Ordering::Relaxed);
                    n.fetch_max(m.seq, Ordering::Relaxed);
                }
                // Deliberately far slower than the producer: this is the lap.
                std::thread::sleep(Duration::from_millis(150));
            }
        }));
    }
    std::thread::sleep(Duration::from_millis(120));

    let producer: Topic<Wide> = Topic::new(topic_name).expect("publisher");
    let total: u64 = 3000;
    for i in 1..=total {
        producer.send(Wide {
            seq: i,
            _pad: [0; 32],
        });
        std::thread::sleep(Duration::from_micros(1000)); // ~1 kHz
    }
    std::thread::sleep(Duration::from_millis(300));
    stop.store(true, Ordering::Relaxed);
    for h in readers {
        let _ = h.join();
    }

    let got = received.load(Ordering::Relaxed);
    let last = newest.load(Ordering::Relaxed);

    assert!(
        got > 0,
        "a lapped consumer received nothing at all ({got} messages). It must be \
         given the newest data, not starved — see the module comment."
    );
    // It cannot receive everything (that is what being lapped means), but it
    // must stay near the head rather than stalling early in the stream.
    assert!(
        last >= total / 2,
        "a lapped consumer stalled at seq {last} of {total}; it should track the \
         head, so the last sequence seen should be late in the run"
    );
}
