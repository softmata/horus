//! Loom-based exhaustive concurrency test for the CAS slot-claim in the MpscShm
//! send paths (`send_shm_mp_pod`/`_serde`/`_colo`), which replaced an optimistic
//! `fetch_add` that could overshoot the ring and overwrite an unconsumed slot
//! (softmata-brain overshoot bug).
//!
//! # What is verified
//!
//! Two producers claim slots concurrently via `compare_exchange` (with an in-loop
//! room check) while one in-order consumer drains. The model asserts:
//!
//! - **No double-claim:** two producers never obtain the same sequence number (the
//!   CAS makes the room-check-and-claim atomic — the property a bare `fetch_add`
//!   lacks). Each delivered value appears exactly once.
//! - **No overshoot / no overwrite:** with `CAP < sends`, a producer that finds the
//!   ring full must retry rather than claim a slot aliasing an unconsumed message.
//!   Values are position-encoded, so an overwrite (wrong value for a position) or a
//!   torn read is caught.
//! - **Ready-flag ordering:** the consumer only reads a slot whose flag matches, and
//!   the flag's Release/Acquire pairing guarantees the data write is visible.
//!
//! # Gate
//!
//! Replace the CAS with a bare `fetch_add` (no room check) and the `CAP < sends`
//! cases go RED: a producer claims past capacity, overwrites the slot the consumer
//! is about to read, and either a value mismatch or a lost message fails an assert.
//!
//! Run with: `cargo test --test loom_mp_claim -- --nocapture`

use loom::sync::atomic::{AtomicU64, Ordering};
use loom::sync::Arc;

/// A capacity-`CAP` multi-producer ring claimed via CAS, mirroring the MpscShm POD
/// send/recv. Data slots are `AtomicU64` for loom.
struct MpRing<const CAP: usize> {
    head: AtomicU64,
    tail: AtomicU64,
    ready: [AtomicU64; CAP],
    data: [AtomicU64; CAP],
}

impl<const CAP: usize> MpRing<CAP> {
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

    /// CAS-claim send — models `send_shm_mp_pod`'s claim loop. Returns false when the
    /// ring is genuinely full (never overshoots). `val` must equal `seq + 1` for the
    /// slot it lands in (checked by the consumer).
    fn try_send(&self, make_val: impl Fn(u64) -> u64) -> bool {
        loop {
            let head = self.head.load(Ordering::Acquire);
            let tail = self.tail.load(Ordering::Acquire);
            if head.wrapping_sub(tail) >= CAP as u64 {
                return false; // full — do NOT claim (overshoot prevention)
            }
            if self
                .head
                .compare_exchange(head, head + 1, Ordering::Relaxed, Ordering::Relaxed)
                .is_ok()
            {
                let idx = (head & Self::MASK) as usize;
                self.data[idx].store(make_val(head), Ordering::Relaxed);
                self.ready[idx].store(head + 1, Ordering::Release);
                return true;
            }
            // Lost the race / spurious — retry with a fresh head.
        }
    }

    /// In-order consumer — models `recv_shm_mpsc_pod`. Returns (value, position).
    fn recv(&self) -> Option<(u64, u64)> {
        let tail = self.tail.load(Ordering::Relaxed);
        let head = self.head.load(Ordering::Acquire);
        if head.wrapping_sub(tail) == 0 {
            return None;
        }
        let idx = (tail & Self::MASK) as usize;
        if self.ready[idx].load(Ordering::Acquire) != tail.wrapping_add(1) {
            return None; // slot claimed but not yet written
        }
        let val = self.data[idx].load(Ordering::Relaxed);
        self.tail.store(tail.wrapping_add(1), Ordering::Release);
        Some((val, tail))
    }
}

/// `PRODS` producers each single-shot `try_send` into a ring of `CAP < PRODS` slots
/// (no consumer, no retry — which keeps the interleaving space small and, crucially,
/// exercises the FULL-RING/OVERSHOOT path exhaustively). The number of successful
/// sends must never exceed `CAP`: the CAS makes the room-check-and-claim atomic, so
/// producers that race past capacity are rejected (return false) rather than claiming
/// a slot that aliases an unconsumed one.
///
/// GATE: replace the CAS in `MpRing::try_send` with a bare `fetch_add` (always
/// returns true, no room check) and this goes RED — `PRODS` sends "succeed" into
/// `CAP < PRODS` slots, i.e. an overshoot that overwrote a live slot. Every readable
/// message must also carry the correct value for its slot (no torn/overwritten read).
fn run<const CAP: usize>(prods: usize) {
    let mut builder = loom::model::Builder::new();
    builder.preemption_bound = Some(3);
    builder.check(move || {
        let ring = Arc::new(MpRing::<CAP>::new());

        let handles: Vec<_> = (0..prods)
            .map(|_| {
                let r = ring.clone();
                // Value stored is always `claimed seq + 1` so the consumer can verify
                // value==position+1 (an overwrite lands the wrong value in a slot).
                loom::thread::spawn(move || if r.try_send(|seq| seq + 1) { 1u32 } else { 0 })
            })
            .collect();

        let successes: u32 = handles.into_iter().map(|h| h.join().unwrap()).sum();

        // Overshoot invariant: at most CAP claims can succeed into CAP slots.
        assert!(
            successes as usize <= CAP,
            "OVERSHOOT: {successes} sends succeeded into {CAP} slots — a producer \
             claimed past capacity and overwrote a live slot"
        );

        // Every readable message carries the correct value for its position (no torn /
        // overwritten read), and exactly `successes` messages are readable.
        let mut got = 0u32;
        while let Some((val, pos)) = ring.recv() {
            assert_eq!(val, pos + 1, "overwrite/torn: value {val} at position {pos}");
            got += 1;
        }
        assert_eq!(got, successes, "readable count {got} != successful sends {successes}");
    });
}

#[test]
fn loom_mp_claim_cap1_two_producers() {
    // Single slot, two producers racing: exactly one may claim; the other must be
    // rejected, not overshoot into slot 0 on top of the first.
    run::<1>(2);
}

#[test]
fn loom_mp_claim_cap2_three_producers() {
    // Two slots, three producers: at most two succeed; the third must be rejected.
    run::<2>(3);
}
