//! A rejected attach must not be able to SIGBUS a healthy publisher.
//!
//! Issue #144. HORUS correctly refuses to open an existing topic under a
//! second, incompatible message type — `Imu` is a 304-byte POD needing
//! 312-byte slots, and a region created for `CmdVel` declares 64:
//!
//!   shared header declares slot_size 64 for a 304-byte POD message under the
//!   co-located layout, which needs at least 312 — each write would run past
//!   its slot
//!
//! The refusal is right. What is wrong is what it leaves behind: the publisher
//! that was already on that topic, correctly typed and healthy, then dies with
//! SIGBUS inside `send_shm_mp_pod` on its next send.
//!
//! SIGBUS is a signal, not an unwind, so `catch_unwind` in `NodeRunner::run_tick`,
//! the executors' outer `catch_unwind`, and every `FailurePolicy` are bypassed.
//! There is no `enter_safe_state()` and no blackbox entry — the process is simply
//! gone. On a robot that is a live control publisher killed because some other
//! node mistyped a topic name.

mod common;
use common::{cleanup_stale_shm, unique};

use horus_core::communication::Topic;
use horus_robotics::messages::sensor::Imu;
use horus_robotics::CmdVel;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// Publish on a topic, have a peer attempt an incompatible attach, keep publishing.
///
/// Without the fix this dies with `signal: 7, SIGBUS` rather than failing an
/// assertion — so a clean exit IS the pass condition, and a crash is the bug.
/// IGNORED ON PURPOSE. This test currently CRASHES THE PROCESS with SIGBUS,
/// which no harness can catch and which would take the whole test binary --
/// and any required gate running it -- down with it. It is here as a
/// deterministic reproducer for #144, to be un-ignored by the change that
/// fixes the fault. Run it deliberately:
///
///   cargo test --release -p horus_core \
///     --test rejected_attach_strands_publisher -- --ignored
#[test]
#[ignore]
fn a_refused_attach_must_not_kill_the_publisher() {
    let _g = cleanup_stale_shm();
    let name = unique("issue144");

    // The incumbent: correctly typed, healthy, already publishing.
    let pubv: Topic<CmdVel> = Topic::new(&name).expect("create the CmdVel topic");
    for i in 0..64 {
        pubv.send(CmdVel::new(i as f32, 0.0));
    }

    // The refused attaches happen WHILE the publisher is mid-send. Sequentially
    // this is harmless -- the send that faults is one already in flight against
    // a mapping the refused attach disturbed, so the two have to overlap.
    let stop = Arc::new(AtomicBool::new(false));
    let s2 = stop.clone();
    let n2 = name.clone();
    let attacher = std::thread::spawn(move || {
        let mut refused = 0u32;
        while !s2.load(Ordering::Relaxed) {
            // Every one of these SHOULD be refused: Imu is a 304-byte POD
            // needing 312-byte slots, the region declares 64.
            if Topic::<Imu>::new(&n2).is_err() {
                refused += 1;
            }
        }
        refused
    });

    for i in 0..200_000 {
        pubv.send(CmdVel::new(i as f32, 1.0));
    }

    stop.store(true, Ordering::Relaxed);
    let refused = attacher.join().expect("attacher thread panicked");

    assert!(
        refused > 0,
        "no attach was refused, so this run exercised nothing — the guard that \
         makes this test meaningful may have changed"
    );

    // Reaching here is the assertion. The failure this test is about is a
    // SIGNAL, not a false return: SIGBUS bypasses catch_unwind entirely.
    pubv.send(CmdVel::new(1.0, 1.0));
}
