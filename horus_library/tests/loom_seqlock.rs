//! Loom-based exhaustive concurrency tests for the TransformFrame seqlock.
//!
//! These tests verify that a concurrent writer and reader using the
//! seqlock (version-dance) protocol never produce a torn read. Loom
//! exhaustively explores every possible thread interleaving so this
//! is a true proof-of-correctness for the memory ordering, not just
//! a probabilistic stress test.
//!
//! The seqlock is re-implemented here with loom atomics (`loom::sync::atomic`)
//! so that loom can track every load/store and detect violations.
//!
//! Run with:
//!   cargo test --test loom_seqlock -- --nocapture
//!
//! # Seqlock memory-ordering invariant under test
//!
//! The seqlock requires Release/Acquire on data stores/loads to be formally
//! correct in the C++11 memory model:
//!
//! ```text
//! Writer:
//!   fetch_add(AcqRel)              // version odd → write in progress
//!   x.store(val, Release)          // data stores must be Release...
//!   y.store(val, Release)          // ...so Acquire loads synchronize with them
//!   version.store(even, Release)   // publish: pairs with reader's v1 Acquire
//!
//! Reader:
//!   v1 = version.load(Acquire)     // pairs with writer's version Release
//!   x = x.load(Acquire)            // Acquire: syncs with writer's Release store
//!   y = y.load(Acquire)            // Acquire: syncs with writer's Release store
//!   v2 = version.load(Acquire)     // must be Acquire, not Relaxed or Release
//!   if v1 != v2 { retry }          // writer intervened, discard snapshot
//! ```
//!
//! Why Acquire on v2 (not Relaxed or Release):
//! - Release has no effect on loads (it is a write-side barrier only).
//! - Relaxed on v2 allows the CPU to reorder v2 *before* the data loads,
//!   making the version check race-free on paper but useless in practice.
//! - Acquire on v2 prevents it from floating above the data loads on
//!   ARM64/RISC-V, keeping the version check strictly after the copy.
//!
//! Why Release on data stores / Acquire on data loads (not Relaxed):
//! - With Relaxed data stores, a reader can observe a data value from a
//!   concurrent in-progress write without establishing any synchronization.
//!   The v2 check may then still observe an older version value, producing
//!   a torn read that v1 == v2 fails to catch.
//! - Release/Acquire creates a synchronizes-with edge: if a reader's Acquire
//!   load observes a writer's Release store, all operations sequenced-before
//!   that store (including the version increment to odd) are visible to the
//!   reader. The reader's v2 Acquire load then sees version >= odd, so v1
//!   (even) != v2 and the reader correctly retries.
//!
//! # Relationship to slot.rs
//!
//! `slot.rs` accesses transform data via raw `UnsafeCell` pointer reads (not
//! atomics), which corresponds to Relaxed in the C++11 abstract machine. This
//! is technically undefined behaviour under C++11 but is safe in practice on
//! all supported hardware (x86 TSO, ARMv8) because:
//!
//! - ARMv8 store buffers are FIFO: a core's stores commit to the coherence
//!   domain in program order. `version.fetch_add` (LDAXR/STLXR, AcqRel)
//!   commits version=odd *before* the subsequent data STR instructions
//!   commit. A reader that observes y=new must therefore also observe
//!   version >= odd, causing v1 != v2 and a correct retry.
//! - `slot.rs` also adds `fence(Acquire)` before each v2 check as a
//!   belt-and-suspenders measure, which on ARM64 generates `dmb ishld`
//!   (load barrier) preventing the v2 LDAR from being reordered above
//!   the raw data LDRs.
//!
//! These loom tests verify the seqlock *protocol* with proper C++11 atomics;
//! they serve as a formal proof of the pattern's correctness.

use loom::sync::{
    atomic::{AtomicU64, Ordering},
    Arc,
};

/// A seqlock protecting a two-field payload (x, y) with correct C++11 ordering.
///
/// Data stores use `Release` and data loads use `Acquire` so that a reader
/// observing a data value from an in-progress write always sees version >= odd
/// when it checks v2, correctly triggering a retry.
struct SeqlockPair {
    version: AtomicU64,
    x: AtomicU64,
    y: AtomicU64,
}

impl SeqlockPair {
    fn new() -> Self {
        Self {
            version: AtomicU64::new(0),
            x: AtomicU64::new(0),
            y: AtomicU64::new(0),
        }
    }

    /// Write (x, y) atomically via seqlock write protocol.
    ///
    /// Data stores use `Release` so that a reader's `Acquire` data load
    /// synchronizes with this write and establishes the required
    /// happens-before edge (version-odd happened-before data store
    /// happened-before data load).
    fn write(&self, x: u64, y: u64) {
        // Mark write in progress (odd version).
        let v = self.version.fetch_add(1, Ordering::AcqRel) + 1;
        debug_assert!(v & 1 == 1);

        // Release stores: a reader's Acquire load of either field syncs with
        // this write, making version >= odd visible to that reader's v2 check.
        self.x.store(x, Ordering::Release);
        self.y.store(y, Ordering::Release);

        // Release: makes all data stores visible before the version update.
        self.version.store(v + 1, Ordering::Release);
    }

    /// Read (x, y) via seqlock read protocol.
    ///
    /// Returns a consistent snapshot (x, y) from a single completed write.
    /// Both version loads are Acquire; data loads are Acquire (see module
    /// docs for why Relaxed data loads are insufficient in C++11).
    fn read(&self) -> (u64, u64) {
        loop {
            // v1 Acquire: pairs with writer's version Release, making all
            // data stores from that write visible before we read them.
            let v1 = self.version.load(Ordering::Acquire);
            if v1 & 1 == 1 {
                // Write in progress — spin
                loom::hint::spin_loop();
                continue;
            }

            // Acquire loads: if we observe a value from a concurrent write,
            // the sync edge ensures version >= odd is visible for v2 check.
            let x = self.x.load(Ordering::Acquire);
            let y = self.y.load(Ordering::Acquire);

            // v2 Acquire: prevents this load from being reordered above the
            // data loads (ARM64/RISC-V would reorder with Relaxed/Release).
            let v2 = self.version.load(Ordering::Acquire);
            if v1 == v2 {
                return (x, y);
            }
            // Writer intervened during our read — retry.
        }
    }
}

/// Loom test: one writer (single write) + one concurrent reader.
///
/// Invariant: the reader must see either (0, 0) — the initial state before
/// any write — or (7, 7) — the post-write state.  A torn read would produce
/// x != y (e.g. x=0, y=7), indicating data from two different states was
/// mixed.  Loom exhaustively explores all thread interleavings to verify
/// this never happens.
#[test]
fn test_seqlock_no_torn_reads_one_writer_one_reader() {
    loom::model(|| {
        let lock = Arc::new(SeqlockPair::new());

        let writer_lock = lock.clone();
        let writer = loom::thread::spawn(move || {
            writer_lock.write(7, 7);
        });

        let reader_lock = lock.clone();
        let reader = loom::thread::spawn(move || {
            let (x, y) = reader_lock.read();
            // No torn read: both fields must be from the same logical state
            assert_eq!(x, y, "torn seqlock read: x={} y={}", x, y);
        });

        writer.join().unwrap();
        reader.join().unwrap();
    });
}

/// Loom test: verify the initial (pre-write) state is handled correctly.
///
/// When version is 0 (even, stable) and no data has been written, the reader
/// must return (0, 0) without stalling or panicking.
#[test]
fn test_seqlock_reads_initial_state() {
    loom::model(|| {
        let lock = Arc::new(SeqlockPair::new());
        let (x, y) = lock.read();
        assert_eq!(x, 0);
        assert_eq!(y, 0);
    });
}

/// Loom test: writer writes once; reader sees either (0,0) or (42,42).
///
/// Exhaustively verifies the reader never observes a mix of pre-write and
/// post-write values.
#[test]
fn test_seqlock_single_write_single_read() {
    loom::model(|| {
        let lock = Arc::new(SeqlockPair::new());

        let writer_lock = lock.clone();
        let writer = loom::thread::spawn(move || {
            writer_lock.write(42, 42);
        });

        let reader_lock = lock.clone();
        let reader = loom::thread::spawn(move || {
            let (x, y) = reader_lock.read();
            // Either both zero (pre-write) or both 42 (post-write)
            assert!(
                (x == 0 && y == 0) || (x == 42 && y == 42),
                "torn read: x={} y={}",
                x,
                y
            );
        });

        writer.join().unwrap();
        reader.join().unwrap();
    });
}
