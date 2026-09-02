//! OPEN: participant counts only ever grow within a process, and the backend is
//! chosen from them.
//!
//! `Drop for RingTopic` (`communication/topic/mod.rs`) releases FanoutShm
//! endpoint bits and spill keep-alives and leaves the participant table
//! untouched. The only `decrement_to_floor` calls on `publisher_count` /
//! `subscriber_count` are in `reap_dead_participants_now`, which refuses to reap
//! on an expired lease alone (an idle subscriber spends much of its life looking
//! expired) and otherwise wants a dead pid. Within one living process, nothing
//! ever gives a registration back.
//!
//! That is not a bookkeeping wart, because `detect_optimal_backend`
//! (`topic/header.rs`) selects the backend from those counts:
//!
//!     pubs <= 1 && subs <= 1  -> SpscShm    (bounded; try_send can fail)
//!     subs > 1  && is_pod     -> PodShm     (broadcast; overwrites the oldest)
//!
//! So a transient subscriber on a fresh thread -- a worker pool, a scoped task,
//! a one-shot diagnostic read -- permanently converts a command topic from
//! backpressured to overwrite-oldest, with nothing alive to justify it and no
//! path back. The test below observes exactly that: `sub_count()` climbs 1, 2, 3
//! across three threads that are each fully joined before the next starts, and
//! the backend flips at the second one.
//!
//! # Why this is `#[ignore]` rather than fixed
//!
//! The obvious fix -- deregister in `Drop` -- was implemented and measured, and
//! it is NOT safe as written. A handle records its participant slot index, but a
//! slot is not owned for the handle's lifetime: `register_role`'s later passes
//! reclaim lease-expired slots, and a lease looks expired whenever its owner has
//! been quiet, so a live handle's slot can be taken by another thread. Releasing
//! against a stale index charges the drop to somebody else's registration.
//! Guarding it with the entry's (pid, thread-id hash) narrows the window but does
//! not close it: thread ids are reused after a thread exits, and the hash is a
//! u32 that can collide.
//!
//! Measured, interleaved against unmodified main on the same machine and load,
//! three pairs of the full `horus_core` lib suite:
//!
//!     main: 0, 0, 0 failures      with the Drop fix: 0, 3, 2 failures
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
//! A correct fix needs unambiguous slot ownership: a generation counter on
//! `ParticipantEntry`, bumped on every claim and recorded by the handle, so a
//! release can tell its own registration from a later one in the same slot. The
//! entry has 3 bytes of padding left after `source_host`, which is room for a
//! `u16` generation. That is the shape; it needs building and testing properly,
//! against a machine that is not also running the rest of this suite.

use horus_core::communication::Topic;

#[test]
#[ignore = "documents an open defect: see the module comment. Run with --ignored."]
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

    // What SHOULD hold, and does not: no subscriber is alive at any of these
    // points, so the count should be 0 or 1 and the backend should stay SpscShm.
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
