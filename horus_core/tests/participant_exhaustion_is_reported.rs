//! The 17th participant must be told why it has no comms.
//!
//! A topic holds at most 16 participants. `register_role` builds a precise
//! error for the one that does not fit — it names the limit and says what to do
//! about it — and all three call sites threw it away:
//! `recv_uninitialized` and `read_latest` were `if ....is_err() { return None }`
//! and `send_uninitialized` the same with `Err(msg)`. Those are the only
//! `ensure_consumer`/`ensure_producer` call sites in the tree, so the message
//! reached nothing.
//!
//! Because the role stays `Unregistered`, every later `recv()` re-enters the
//! same cold path and fails identically: `None` forever, from a subscriber that
//! is indistinguishable from an idle one. The identical 16-slot wall one layer
//! down is reported deliberately and loudly, under a comment reading "never let
//! 'all 16 slots live' become a SILENT no-comms endpoint" — with the rate
//! limiter for it sitting in the same file, unused by this path.

use horus_core::communication::Topic;
use std::sync::{Arc, Mutex};

/// The participant table's size. Mirrors `MAX_PARTICIPANTS` in
/// `topic/header.rs`, which is `pub(crate)` and so not importable here.
const MAX_PARTICIPANTS: u32 = 16;
/// Live subscribers needed to fill it, given the producer holds one slot.
const HOLDERS: usize = MAX_PARTICIPANTS as usize - 1;

/// A `tracing::Subscriber` that keeps the formatted fields of every event.
///
/// Hand-rolled against `tracing` core so this test needs no new dependency.
struct Capture(Arc<Mutex<Vec<String>>>);

impl tracing::Subscriber for Capture {
    fn enabled(&self, _: &tracing::Metadata<'_>) -> bool {
        true
    }
    fn new_span(&self, _: &tracing::span::Attributes<'_>) -> tracing::Id {
        tracing::Id::from_u64(1)
    }
    fn record(&self, _: &tracing::Id, _: &tracing::span::Record<'_>) {}
    fn record_follows_from(&self, _: &tracing::Id, _: &tracing::Id) {}
    fn event(&self, event: &tracing::Event<'_>) {
        struct Fields(String);
        impl tracing::field::Visit for Fields {
            fn record_debug(&mut self, _: &tracing::field::Field, value: &dyn std::fmt::Debug) {
                self.0.push_str(&format!("{:?} ", value));
            }
        }
        let mut fields = Fields(String::new());
        event.record(&mut fields);
        self.0
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push(fields.0);
    }
    fn enter(&self, _: &tracing::Id) {}
    fn exit(&self, _: &tracing::Id) {}
}

#[test]
fn a_participant_that_does_not_fit_is_told_so() {
    let name = format!("participant_exhaustion_{}", std::process::id());

    // Process-wide, not `set_default`: the registration that gets refused
    // happens on a spawned thread, and `set_default` is thread-local. A thread
    // is required — the main thread already holds a participant entry for its
    // publisher, and `ensure_consumer` upgrades that entry's role in place
    // rather than claiming a second slot, so it is never refused.
    // This test is alone in its binary, so the global default is free.
    let captured = Arc::new(Mutex::new(Vec::new()));
    tracing::subscriber::set_global_default(Capture(Arc::clone(&captured)))
        .expect("no other subscriber in this test binary");

    let Ok(producer) = Topic::<u64>::new(&name) else {
        eprintln!("skipping: no shared memory available");
        return;
    };
    producer.send(1u64);

    // Fill the participant table with LIVE holders.
    //
    // This used to spawn threads and let them exit, relying on the leak
    // documented in participant_count_leak.rs to keep their slots occupied.
    // That leak is fixed — a dropped handle gives its registration back — so
    // the table stopped filling and this test skipped itself unconditionally:
    // "the participant table did not fill (1/16)". The anti-vacuity guard
    // reported it rather than passing quietly, which is what such a guard is
    // for, but the test proved nothing until it held its registrations open.
    let (ready_tx, ready_rx) = std::sync::mpsc::channel::<()>();
    let release = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let mut holders = Vec::new();
    for _ in 0..HOLDERS {
        let n = name.clone();
        let ready = ready_tx.clone();
        let release = Arc::clone(&release);
        holders.push(std::thread::spawn(move || {
            let sub = Topic::<u64>::new(&n).expect("holder subscriber");
            let _ = sub.try_recv(); // claims this thread's participant slot
            let _ = ready.send(());
            // Hold the registration open until the assertions are done.
            while !release.load(std::sync::atomic::Ordering::Acquire) {
                std::thread::sleep(std::time::Duration::from_millis(2));
            }
            drop(sub);
        }));
    }
    drop(ready_tx);
    for _ in 0..HOLDERS {
        ready_rx
            .recv_timeout(std::time::Duration::from_secs(30))
            .expect("every holder must claim its slot");
    }

    // Anti-vacuity: assert the table really is full BEFORE asking whether the
    // overflow was reported. Without this the test passes on a platform where a
    // slot was always available and nothing was ever refused.
    let occupied = producer.sub_count() + producer.pub_count();

    // One more subscriber, on a thread of its own so it needs a slot of its
    // own. There is none left to give it.
    let n = name.clone();
    let received = std::thread::spawn(move || {
        let sub = Topic::<u64>::new(&n).expect("the handle itself still constructs");
        sub.try_recv()
    })
    .join()
    .expect("late subscriber thread");

    release.store(true, std::sync::atomic::Ordering::Release);
    for h in holders {
        h.join().expect("holder thread");
    }

    let messages = captured.lock().unwrap_or_else(|p| p.into_inner()).clone();

    assert!(
        occupied >= MAX_PARTICIPANTS,
        "precondition: the table must be full for anything to be refused \
         ({occupied}/{MAX_PARTICIPANTS} occupied)"
    );
    assert!(
        received.is_none(),
        "precondition: with the table full this subscriber holds no \
         registration, so it must receive nothing"
    );

    assert!(
        messages.iter().any(|m| m.contains("participant slots")),
        "a subscriber was refused a registration and will receive nothing for \
         the life of its handle, and was told nothing. Warnings seen: {:?}",
        messages
    );
    assert!(
        messages.iter().any(|m| m.contains(&name)),
        "the warning must name the topic — an operator needs to know WHICH \
         subsystem went quiet. Warnings seen: {:?}",
        messages
    );
}
