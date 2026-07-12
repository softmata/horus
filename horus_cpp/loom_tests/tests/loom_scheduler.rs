//! Loom model tests for the horus_cpp scheduler + node FFI atomic contract.
//!
//! Loom replaces `std::sync::*` with a virtualized version and exhaustively
//! explores thread interleavings. Requires `--cfg loom`.
//!
//! Run: cd horus_cpp/loom_tests && RUSTFLAGS="--cfg loom" \
//!        LOOM_MAX_PREEMPTIONS=3 cargo test --release
//!
//! Lives in a separate crate from horus_cpp because loom's `--cfg loom` flag
//! causes tokio (a transitive dep of horus_cpp) to fail to compile.

#![cfg(loom)]

use loom::sync::atomic::{AtomicUsize, Ordering};
use loom::sync::Arc;
use loom::thread;

// Models the scheduler+callback atomic contract: a publisher thread
// increments a counter while a callback thread reads it. Property: the
// observed value is always ≤ the final value (no impossible intermediate
// states) and the final value is exactly 2.
#[test]
fn tick_callback_atomic_observability() {
    loom::model(|| {
        let counter = Arc::new(AtomicUsize::new(0));
        let c1 = counter.clone();
        let publisher = thread::spawn(move || {
            c1.fetch_add(1, Ordering::SeqCst);
            c1.fetch_add(1, Ordering::SeqCst);
        });
        let c2 = counter.clone();
        let callback = thread::spawn(move || {
            let v = c2.load(Ordering::SeqCst);
            assert!(v <= 2, "impossible counter value: {}", v);
        });
        publisher.join().unwrap();
        callback.join().unwrap();
        let final_v = counter.load(Ordering::SeqCst);
        assert_eq!(final_v, 2);
    });
}
