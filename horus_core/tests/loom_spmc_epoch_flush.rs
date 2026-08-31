//! Loom model of the SpmcShm epoch flush against competing consumers.
//!
//! # The asymmetry this exists to pin
//!
//! `handle_epoch_change` publishes a handle's consumed frontier to the shared
//! `header.tail` before migrating (`topic/mod.rs`, the `is_cross_process()`
//! block). On SpscShm/MpscShm that word is effectively the one consumer's own
//! position, so a plain store is harmless. On **SpmcShm it is not**:
//! `recv_shm_spmc_pod` CAS-coordinates competing consumers *through* that word,
//! making it a shared claim cursor. `is_cross_process()` includes `SpmcShm`
//! (`types.rs`), so the flush fires there too.
//!
//! The consumer-join flush a few hundred lines earlier publishes the SAME value
//! and uses `fetch_max`, with the reason written out:
//!
//! > `fetch_max` never moves the shared tail backward, so it is safe even if a
//! > concurrent SpmcShm consumer has already advanced it.
//!
//! and names the consequence of getting it wrong:
//!
//! > if a 2nd consumer now joins and CAS-reads from that stale value, it
//! > RE-DELIVERS messages the first consumer already took.
//!
//! Same value, same modes, same hazard — one site guarded, one not.
//!
//! # Why loom rather than an integration test
//!
//! A topic renegotiates its backend: handles opened around a `force_migrate`
//! land back in SpscShm when only one consumer is active, so pinning two live
//! SpmcShm consumers from the outside is unreliable. Loom makes the interleaving
//! explicit and the result deterministic.
//!
//! # Gate
//!
//! `FLUSH_WITH_FETCH_MAX = false` reproduces the plain store and the model goes
//! RED — the cursor moves backward and a message is claimed twice. `true` is
//! the fix and it passes.

use loom::sync::atomic::{AtomicU64, Ordering};
use loom::sync::Arc;

/// Set to `false` to model the plain `store` (the pre-fix code).
const FLUSH_WITH_FETCH_MAX: bool = true;

/// The shared SpmcShm claim cursor plus the per-message claim ledger.
struct Spmc {
    /// `header.tail` — competing consumers CAS through this to claim.
    tail: AtomicU64,
    /// How many messages are available (`header.sequence_or_head`).
    head: AtomicU64,
    /// claims[i] counts how many consumers claimed message i. Must never exceed 1.
    claims: [AtomicU64; 8],
}

impl Spmc {
    fn new(head: u64) -> Self {
        Self {
            tail: AtomicU64::new(0),
            head: AtomicU64::new(head),
            claims: std::array::from_fn(|_| AtomicU64::new(0)),
        }
    }

    /// Mirrors `recv_shm_spmc_pod`: load the cursor, CAS it forward to claim.
    /// Returns the claimed index.
    fn try_claim(&self) -> Option<u64> {
        loop {
            let tail = self.tail.load(Ordering::Acquire);
            if tail >= self.head.load(Ordering::Acquire) {
                return None;
            }
            if self
                .tail
                .compare_exchange_weak(
                    tail,
                    tail.wrapping_add(1),
                    Ordering::AcqRel,
                    Ordering::Relaxed,
                )
                .is_ok()
            {
                self.claims[(tail % 8) as usize].fetch_add(1, Ordering::Relaxed);
                return Some(tail);
            }
        }
    }

    /// Mirrors the `handle_epoch_change` flush of a handle's cached position.
    fn epoch_flush(&self, local_tail: u64) {
        if FLUSH_WITH_FETCH_MAX {
            self.tail.fetch_max(local_tail, Ordering::Release);
        } else {
            self.tail.store(local_tail, Ordering::Release);
        }
    }
}

#[test]
fn spmc_epoch_flush_must_not_redeliver_a_claimed_message() {
    let mut builder = loom::model::Builder::new();
    builder.preemption_bound = Some(3);
    builder.check(|| {
        // Three messages available; two consumers competing.
        let s = Arc::new(Spmc::new(3));

        // Consumer A claims one message, so its cached local_tail is 1 while the
        // shared cursor will run ahead of it once B claims.
        let a_local = s.try_claim().map(|i| i + 1).unwrap_or(0);

        let h = {
            let s = s.clone();
            loom::thread::spawn(move || {
                // Consumer B keeps claiming, pushing the cursor past A.
                while s.try_claim().is_some() {}
            })
        };

        // A crosses an epoch boundary and flushes its now-stale position.
        s.epoch_flush(a_local);

        h.join().unwrap();

        // Drain anything the flush re-opened.
        while s.try_claim().is_some() {}

        // The invariant: SpmcShm hands each message to exactly one consumer.
        for i in 0..3usize {
            let n = s.claims[i].load(Ordering::Relaxed);
            assert!(
                n <= 1,
                "message {i} was claimed {n} times — the epoch flush moved the \
                 shared cursor backward over a claim another consumer had \
                 already made, so the next CAS handed the same message out \
                 again. This is what fetch_max prevents and a plain store does \
                 not."
            );
        }
    });
}
