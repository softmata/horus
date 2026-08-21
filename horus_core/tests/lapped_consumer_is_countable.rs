//! A consumer that falls behind must be able to find out that it did.
//!
//! Drop-oldest under overload is by design — a 10 Hz node reading a 1 kHz
//! sensor should get the most recent sample, not a backlog. The problem was
//! that the loss left no trace. `dropped_count()` reports *send* failures, and
//! the broadcast backends overwrite rather than fail, so the publisher records
//! nothing while a subscriber silently loses whole laps.
//!
//! Observed in `multithread_nonpod_subscribers_each_get_full_stream` on a
//! loaded machine: of 400 messages, one subscriber received all 400 and the
//! other received m145..m400 — exactly one 256-slot lap missing — with
//! `dropped_count()` reading 0 on every handle involved.

use horus_core::communication::Topic;
use std::sync::mpsc;

fn unique(p: &str) -> String {
    format!(
        "{}_{}_{}",
        p,
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    )
}

#[test]
fn a_lapped_subscriber_reports_what_it_skipped() {
    let name = unique("missed_count");

    // Two subscribers, each on its own thread, so the backend detector selects a
    // broadcast backend rather than the single-consumer SpscShm that two
    // same-thread handles collapse onto. Broadcast is the case that overwrites
    // instead of applying backpressure, which is where the loss goes unrecorded.
    let (registered_tx, registered_rx) = mpsc::channel();
    let (go_tx, go_rx) = mpsc::channel::<()>();
    let mut go_txs = Vec::new();
    let mut subs = Vec::new();
    for tag in ["stalled", "draining"] {
        let name = name.clone();
        let registered_tx = registered_tx.clone();
        let (my_go_tx, my_go_rx) = mpsc::channel::<()>();
        go_txs.push(my_go_tx);
        subs.push(std::thread::spawn(move || {
            let sub: Topic<u64> = Topic::new(&name).expect("sub");
            let _ = sub.recv(); // registers this thread as a subscriber
            sub.check_migration_now();
            registered_tx.send(()).expect("registered");

            // Both stand still while the producer runs — that is what makes them
            // lapped. `tag` only labels the result.
            my_go_rx.recv().expect("producer finished");

            let mut got = 0u64;
            while sub.recv().is_some() {
                got += 1;
            }
            (tag, got, sub.missed_count(), sub.backend_name().to_string())
        }));
    }
    drop(registered_tx);
    drop(go_rx);

    let producer: Topic<u64> = Topic::new(&name).expect("producer");
    registered_rx.recv().expect("sub 1 registered");
    registered_rx.recv().expect("sub 2 registered");
    producer.check_migration_now();

    // Far more than any ring capacity, so both consumers are lapped repeatedly.
    const SENT: u64 = 20_000;
    for i in 0..SENT {
        producer.send(i);
    }
    for tx in go_txs {
        let _ = tx.send(());
    }
    drop(go_tx);

    for handle in subs {
        let (tag, got, missed, backend) = handle.join().expect("subscriber thread");
        println!("{tag}: backend={backend} got={got} missed={missed}");
        assert!(
            got < SENT,
            "{tag} was supposed to be lapped but received all {SENT} messages — \
             the test is not exercising the path it claims to"
        );
        assert!(
            missed > 0,
            "{tag} received {got} of {SENT} messages and reported missing none. \
             That is the whole defect: the loss is real and invisible."
        );
        assert!(
            missed + got >= SENT / 2,
            "{tag}: skipped + received ({missed} + {got}) should account for most \
             of the {SENT} sent; a count far below that is not a usable signal"
        );
    }
}
