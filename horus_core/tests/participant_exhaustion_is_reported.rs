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

    // Fill the participant table. Each thread claims its own slot, and — this
    // is the open leak documented in participant_count_leak.rs — the slot is
    // not returned when the thread ends, which is what lets one process fill
    // the table at all. The producer holds one, so 15 subscribers saturate it
    // and every thread after that is refused.
    for _ in 0..20 {
        let n = name.clone();
        std::thread::spawn(move || {
            if let Ok(sub) = Topic::<u64>::new(&n) {
                let _ = sub.try_recv();
            }
        })
        .join()
        .expect("subscriber thread");
    }

    let messages = captured.lock().unwrap_or_else(|p| p.into_inner()).clone();

    // Anti-vacuity: assert the table really did fill BEFORE asking whether the
    // overflow was reported. Without this the test passes on a platform where a
    // slot was always available and nothing was ever refused.
    let occupied = producer.sub_count() + producer.pub_count();
    if occupied < 16 {
        eprintln!("skipping: the participant table did not fill ({occupied}/16)");
        return;
    }

    assert!(
        messages.iter().any(|m| m.contains("participant slots")),
        "five subscribers were refused a registration and will receive nothing \
         for the life of their handles. Not one of them was told. Warnings \
         seen: {:?}",
        messages
    );
    assert!(
        messages.iter().any(|m| m.contains(&name)),
        "the warning must name the topic — an operator needs to know WHICH \
         subsystem went quiet. Warnings seen: {:?}",
        messages
    );
}
