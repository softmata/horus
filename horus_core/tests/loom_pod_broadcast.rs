//! Loom model for the `PodShm` broadcast ring's torn-read window.
//!
//! `PodShm` is the DEFAULT backend for any POD type (`header.rs`:
//! `(_, _) if is_pod => BackendMode::PodShm`), so this is the most-travelled
//! path in the whole IPC layer. Its producer has **no backpressure** — it
//! `fetch_add`s a sequence and overwrites whatever slot it lands on — so a
//! producer that laps the ring can clobber the very slot a consumer is copying.
//!
//! Before the fix, `send_shm_pod_broadcast` stamped the slot ready only AFTER
//! writing the data, and `recv_shm_pod_broadcast` checked that stamp once and
//! never re-checked. That leaves the classic torn-read window open: the consumer
//! validates the stamp, starts copying, the producer overwrites mid-copy, and
//! the consumer returns a value that is half-old and half-new — on a robot, a
//! pose or velocity that never existed, with nothing to indicate it.
//!
//! The fix is the Boehm seqlock pairing used by `topic::seqlock`, adapted to the
//! `seq + 1` stamp encoding that the ready array already uses (a high
//! `SLOT_WRITING` bit rather than the `pos << 1` tagging, because the array is
//! shared with the MpscShm/SpscShm paths and a PodShm topic can migrate to
//! them):
//!
//!   producer: stamp |= WRITING; release-fence; write data; stamp = seq+1 (Release)
//!   consumer: v1 = stamp (Acquire); read data; acquire-fence; require stamp == v1
//!
//! # What this model proves
//!
//! Every value a consumer accepts is a value the producer actually sent — never
//! a mixture. `NAIVE` reproduces the old protocol and is expected to FAIL under
//! loom; it is kept (ignored) so the model is demonstrably able to catch the bug
//! it was written for, rather than passing vacuously.
//!
//! Run with: `cargo test --test loom_pod_broadcast -- --nocapture`

use loom::sync::atomic::{fence, AtomicU64, Ordering};
use loom::sync::Arc;

/// High bit marking "a write is in progress on this slot".
/// Mirrors `dispatch::SLOT_WRITING`.
const SLOT_WRITING: u64 = 1 << 63;

/// A capacity-`CAP` broadcast ring mirroring `send/recv_shm_pod_broadcast`.
///
/// Data slots are `AtomicU64` so loom does not flag the intentional concurrent
/// access; the stamp protocol is what decides which reads are accepted. Each
/// published value is derived from its position, so a torn read shows up as a
/// value that does not match the position it was delivered for.
struct BroadcastRing<const CAP: usize> {
    head: AtomicU64,
    tail: AtomicU64,
    ready: [AtomicU64; CAP],
    data: [AtomicU64; CAP],
}

/// Value the producer writes for position `pos`. Distinct per position so a
/// mixture of two positions is detectable.
fn value_for(pos: u64) -> u64 {
    0xA5A5_0000_0000_0000 | (pos.wrapping_mul(0x9E37_79B9) & 0xFFFF_FFFF)
}

impl<const CAP: usize> BroadcastRing<CAP> {
    const MASK: u64 = (CAP as u64) - 1;

    fn new() -> Self {
        assert!(CAP.is_power_of_two());
        Self {
            head: AtomicU64::new(0),
            tail: AtomicU64::new(0),
            ready: std::array::from_fn(|_| AtomicU64::new(0)),
            data: std::array::from_fn(|_| AtomicU64::new(0)),
        }
    }

    /// Producer, FIXED protocol — mirrors `send_shm_pod_broadcast`.
    fn publish(&self) {
        let seq = self.head.fetch_add(1, Ordering::Relaxed);
        let idx = (seq & Self::MASK) as usize;
        let stamp = seq.wrapping_add(1);

        self.ready[idx].store(stamp | SLOT_WRITING, Ordering::Relaxed);
        fence(Ordering::Release);
        self.data[idx].store(value_for(seq), Ordering::Relaxed);
        self.ready[idx].store(stamp, Ordering::Release);
    }

    /// Producer, OLD protocol — stamps only after the write, no marker.
    fn publish_naive(&self) {
        let seq = self.head.fetch_add(1, Ordering::Relaxed);
        let idx = (seq & Self::MASK) as usize;
        self.data[idx].store(value_for(seq), Ordering::Relaxed);
        self.ready[idx].store(seq.wrapping_add(1), Ordering::Release);
    }

    /// Consumer, FIXED protocol — mirrors `recv_shm_pod_broadcast`.
    /// Returns `(value, expected_position)` when it accepts a read.
    fn consume(&self) -> Option<(u64, u64)> {
        let tail = self.tail.load(Ordering::Relaxed);
        let idx = (tail & Self::MASK) as usize;

        let v1 = self.ready[idx].load(Ordering::Acquire);
        if v1 & SLOT_WRITING != 0 {
            return None; // mid-write
        }
        if v1 < tail.wrapping_add(1) {
            return None; // nothing published here yet
        }

        let val = self.data[idx].load(Ordering::Relaxed);

        fence(Ordering::Acquire);
        if self.ready[idx].load(Ordering::Relaxed) != v1 {
            return None; // overwritten mid-copy — discard
        }

        self.tail.store(tail.wrapping_add(1), Ordering::Relaxed);
        // v1 is `pos + 1` for whichever position actually occupies the slot,
        // which may be ahead of `tail` after a lap (broadcast is latest-wins).
        Some((val, v1.wrapping_sub(1)))
    }

    /// Consumer, OLD protocol — single stamp check, no re-check.
    fn consume_naive(&self) -> Option<(u64, u64)> {
        let tail = self.tail.load(Ordering::Relaxed);
        let idx = (tail & Self::MASK) as usize;
        let v1 = self.ready[idx].load(Ordering::Acquire);
        if v1 < tail.wrapping_add(1) {
            return None;
        }
        let val = self.data[idx].load(Ordering::Relaxed);
        self.tail.store(tail.wrapping_add(1), Ordering::Relaxed);
        Some((val, v1.wrapping_sub(1)))
    }
}

/// Capacity 2 with 3 publishes forces the producer to lap the ring and land on
/// the slot the consumer is reading — the exact interleaving the fix targets.
/// Loom explores every ordering of that race.
#[test]
fn broadcast_read_never_tears_under_lapping_producer() {
    loom::model(|| {
        let ring = Arc::new(BroadcastRing::<2>::new());

        let producer = {
            let ring = ring.clone();
            loom::thread::spawn(move || {
                for _ in 0..3 {
                    ring.publish();
                }
            })
        };

        let consumer = {
            let ring = ring.clone();
            loom::thread::spawn(move || {
                let mut accepted = Vec::new();
                for _ in 0..2 {
                    if let Some((val, pos)) = ring.consume() {
                        accepted.push((val, pos));
                    }
                }
                accepted
            })
        };

        producer.join().unwrap();
        let accepted = consumer.join().unwrap();

        for (val, pos) in accepted {
            assert_eq!(
                val,
                value_for(pos),
                "TORN READ: consumer accepted {val:#x} for position {pos}, but the \
                 producer wrote {:#x} there. The value is a mixture of two messages.",
                value_for(pos)
            );
        }
    });
}

/// The consumer must never observe a slot while the producer is writing it.
#[test]
fn broadcast_consumer_never_accepts_a_slot_mid_write() {
    loom::model(|| {
        let ring = Arc::new(BroadcastRing::<2>::new());

        let producer = {
            let ring = ring.clone();
            loom::thread::spawn(move || {
                ring.publish();
                ring.publish();
                ring.publish();
            })
        };

        let consumer = {
            let ring = ring.clone();
            loom::thread::spawn(move || {
                let mut seen_writing = false;
                for _ in 0..2 {
                    let tail = ring.tail.load(Ordering::Relaxed);
                    let idx = (tail & BroadcastRing::<2>::MASK) as usize;
                    if ring.ready[idx].load(Ordering::Acquire) & SLOT_WRITING != 0 {
                        seen_writing = true;
                    }
                    // A slot observed mid-write must never be accepted.
                    if let Some((val, pos)) = ring.consume() {
                        assert_eq!(val, value_for(pos), "accepted a mid-write slot");
                    }
                }
                seen_writing
            })
        };

        producer.join().unwrap();
        let _ = consumer.join().unwrap();
    });
}

/// The model must be able to FAIL. This runs the old protocol and is expected to
/// find a torn read; it is `#[ignore]`d so the suite stays green, but it can be
/// run by hand to confirm the model is not passing vacuously:
///
///   cargo test --test loom_pod_broadcast -- --ignored naive
#[test]
#[ignore = "demonstrates the OLD protocol failing; run by hand to validate the model"]
fn naive_protocol_tears_proving_the_model_can_detect_it() {
    loom::model(|| {
        let ring = Arc::new(BroadcastRing::<2>::new());

        let producer = {
            let ring = ring.clone();
            loom::thread::spawn(move || {
                for _ in 0..3 {
                    ring.publish_naive();
                }
            })
        };

        let consumer = {
            let ring = ring.clone();
            loom::thread::spawn(move || {
                let mut accepted = Vec::new();
                for _ in 0..2 {
                    if let Some(r) = ring.consume_naive() {
                        accepted.push(r);
                    }
                }
                accepted
            })
        };

        producer.join().unwrap();
        let accepted = consumer.join().unwrap();

        for (val, pos) in accepted {
            assert_eq!(
                val,
                value_for(pos),
                "torn read under the OLD protocol (expected — this is the bug)"
            );
        }
    });
}
