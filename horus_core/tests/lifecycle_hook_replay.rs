//! A hook installed after topics exist must still learn about them, and must
//! learn which direction each handle is actually used in.
//!
//! `set_topic_lifecycle_hook` is a `OnceLock` set when the network replicator
//! starts, and the replicator starts from `scheduler.run()` — after every node
//! has built its topics in `init()` or in its constructor. Those `Created`
//! events fired into a hook that did not exist yet, so the replicator's topic
//! registry began empty for exactly the topics a robot actually uses, and never
//! recovered: nothing re-announces a topic that is already open.
//!
//! The direction matters too. A `Topic<T>` can send and recv, so `Created`
//! cannot say which a handle is for; without a later signal the network layer
//! records every topic as "both" and its import guard loses the distinction it
//! is built on — either denying all remote data or letting a remote peer
//! overwrite the commands this robot produces itself.

use horus_core::communication::{set_topic_lifecycle_hook, Topic, TopicLifecycleEvent};
use std::sync::{Arc, Mutex};

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

#[derive(Default)]
struct Seen {
    created: Vec<String>,
    published: Vec<String>,
    subscribed: Vec<String>,
}

#[test]
fn a_late_hook_sees_existing_topics_and_the_direction_they_are_used_in() {
    // Built BEFORE any hook exists — the ordinary case, since nodes construct
    // their topics before the scheduler starts the replicator.
    let early = unique("replay_early");
    let early_topic: Topic<f32> = Topic::new(&early).expect("create topic");
    // ...and used before the hook exists too.
    early_topic.send(1.0);

    let sub_only = unique("replay_sub_only");
    let sub_topic: Topic<f32> = Topic::new(&sub_only).expect("create topic");
    let _ = sub_topic.recv();

    let seen = Arc::new(Mutex::new(Seen::default()));
    let sink = seen.clone();
    set_topic_lifecycle_hook(move |event| {
        let mut s = sink.lock().unwrap();
        match event {
            TopicLifecycleEvent::Created { name, .. } => s.created.push(name),
            TopicLifecycleEvent::RoleObserved {
                name, publisher, ..
            } => {
                if publisher {
                    s.published.push(name)
                } else {
                    s.subscribed.push(name)
                }
            }
            TopicLifecycleEvent::Dropped { .. } => {}
        }
    });

    let s = seen.lock().unwrap();
    assert!(
        s.created.contains(&early),
        "a hook installed after the topic was created never learned about it, so \
         the replicator's registry would be missing every topic the robot built \
         during init. created: {:?}",
        s.created
    );
    assert!(
        s.published.contains(&early),
        "a topic that was published to before the hook existed was replayed \
         without its direction, so the import guard cannot tell it apart from \
         one this robot only listens to. published: {:?}",
        s.published
    );
    assert!(
        s.subscribed.contains(&sub_only),
        "a receive-only topic was not reported as subscribed. subscribed: {:?}",
        s.subscribed
    );
    assert!(
        !s.published.contains(&sub_only),
        "a topic that was never sent on was reported as published — the guard \
         would refuse remote data for it. published: {:?}",
        s.published
    );
}
