//! A registration is given back when its handle is dropped.
//!
//! It was not, and nothing else inside a living process did it either. The only
//! `decrement_to_floor` calls on `publisher_count`/`subscriber_count` are in
//! `reap_dead_participants_now`, which wants an expired lease AND a dead pid,
//! and in `register_role`'s last-resort pass, which only runs once all 16 slots
//! are occupied. So within one process the counts only ever grew.
//!
//! That is not a bookkeeping wart, because `detect_optimal_backend`
//! (`topic/header.rs`) selects the backend from those counts:
//!
//!     pubs <= 1 && subs <= 1  -> SpscShm    (bounded; try_send can fail)
//!     subs > 1  && is_pod     -> PodShm     (broadcast; overwrites the oldest)
//!
//! So a transient subscriber on a fresh thread -- a worker pool, a scoped task,
//! a one-shot diagnostic read -- permanently converted a command topic from
//! backpressured to overwrite-oldest, with nothing alive to justify it and no
//! path back for the life of the shared-memory segment. It silently voided
//! `try_send`'s full-ring `Err` and `send_blocking`'s backpressure on an e-stop
//! path.
//!
//! # Why the obvious fix was not enough
//!
//! Deregistering in `Drop` against the recorded slot INDEX was implemented and
//! measured, and it was not safe: a slot is not owned for a handle's lifetime.
//! `register_role`'s pass 3 reclaims slots whose thread has ended, so releasing
//! against a stale index charges the drop to somebody else's registration.
//! Guarding with (pid, thread-id hash) narrows the window without closing it --
//! thread ids are reused, and the hash is a u32 that can collide.
//!
//! Measured then, interleaved against unmodified main on the same machine and
//! load, three pairs of the full `horus_core` lib suite:
//!
//!     main: 0, 0, 0 failures      with the naive Drop fix: 0, 3, 2 failures
//!
//! The failures were `multithread_nonpod_subscribers_each_get_a_contiguous_accounted_stream`
//! and `multithread_nonpod_lapped_stream_stays_ordered` -- a subscriber whose
//! registration went missing, so the topic never reached the two-subscriber
//! broadcast backend and one subscriber received nothing.
//!
//! Under-counting is the DANGEROUS direction. `nothing_is_draining` lets a
//! producer retire unread slots once `sub_count()` reads 0, so a lost
//! registration becomes silent message loss on a subscriber that is still
//! reading -- strictly worse than the over-count being fixed.
//!
//! # What makes it safe now
//!
//! `ParticipantEntry` carries a `generation`, bumped on every claim in
//! `finish_claim` and recorded by the handle beside the slot index, so a
//! release can tell its own registration from a later one in the same slot. It
//! also carries a `handles` count, because an entry is keyed on (pid, thread)
//! and several `Topic`s on one thread deliberately share one: the first to drop
//! must not clear an entry the others are still using. `release_participant`
//! refuses -- rather than guesses -- on every mismatch.
//!
//! Both fields came out of the entry's existing padding, so it is still 24
//! bytes and the 640-byte header is unchanged.

use horus_core::communication::Topic;

#[test]
fn transient_subscribers_permanently_flip_a_topic_to_a_lossy_backend() {
    let name = "leak_transient_subs";
    let producer: Topic<u64> = Topic::new(name).expect("producer");
    producer.send(1u64);

    let mut observed = Vec::new();
    for _ in 0..3 {
        let n = name.to_string();
        std::thread::spawn(move || {
            let sub: Topic<u64> = Topic::new(&n).expect("subscriber");
            let _ = sub.try_recv();
        })
        .join()
        .expect("subscriber thread");

        producer.send(2u64);
        observed.push((producer.sub_count(), producer.backend_name().to_string()));
    }

    // No subscriber is alive at any of these points, so the count is 0 or 1
    // and the backend stays SpscShm.
    assert!(
        observed.iter().all(|(c, _)| *c <= 1),
        "sub_count climbed to {:?} with no live subscriber at any sample point",
        observed.iter().map(|(c, _)| *c).collect::<Vec<_>>()
    );
    assert!(
        observed.iter().all(|(_, b)| b == "SpscShm"),
        "a backpressured command topic became lossy with no live subscriber: {observed:?}"
    );
}

/// Several handles on ONE thread share a single registration, and the first to
/// go must not take it away from the others.
///
/// `register_role` keys an entry on (pid, thread), not on the handle: a second
/// `Topic` on the same thread finds the existing entry, ORs in its role bit and
/// deliberately does NOT increment `subscriber_count` again. So a release that
/// matched only on (slot, generation) would clear an entry two other handles
/// were still using — and under-counting is the dangerous direction, because
/// `nothing_is_draining` lets a producer retire unread slots once `sub_count()`
/// reads 0. That is silent message loss on a subscriber that is still reading,
/// strictly worse than the over-count being fixed.
#[test]
fn one_handle_leaving_does_not_deregister_its_siblings_on_the_same_thread() {
    let name = "leak_shared_entry";
    let producer: Topic<u64> = Topic::new(name).expect("producer");
    producer.send(1u64);

    let sub_a: Topic<u64> = Topic::new(name).expect("sub a");
    let sub_b: Topic<u64> = Topic::new(name).expect("sub b");
    let _ = sub_a.try_recv();
    let _ = sub_b.try_recv();

    assert_eq!(
        producer.sub_count(),
        1,
        "precondition: two handles on one thread share one registration"
    );

    drop(sub_a);
    assert_eq!(
        producer.sub_count(),
        1,
        "sub_b is still reading, so the registration must survive its \
         sibling's departure"
    );

    // And it really can still read.
    producer.send(7u64);
    let mut seen = None;
    for _ in 0..1000 {
        if let Some(v) = sub_b.try_recv() {
            seen = Some(v);
            if seen == Some(7) {
                break;
            }
        }
    }
    assert_eq!(
        seen,
        Some(7),
        "the surviving handle must still receive after its sibling dropped"
    );

    drop(sub_b);
    assert_eq!(
        producer.sub_count(),
        0,
        "the last handle to go gives the registration back"
    );
}
