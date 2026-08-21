//! A hook installed after topics exist must still learn about them.
//!
//! `set_topic_lifecycle_hook` is a `OnceLock` set when the network replicator
//! starts, and the replicator starts from `scheduler.run()` — after every node
//! has built its topics in `init()` or in its constructor. Those `Created`
//! events fired into a hook that did not exist yet, so the replicator's topic
//! registry began empty for exactly the topics a robot actually uses, and never
//! recovered: nothing re-announces a topic that is already open.

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

#[test]
fn a_hook_installed_late_still_sees_topics_that_already_exist() {
    // Built BEFORE any hook exists — the ordinary case, since nodes construct
    // their topics before the scheduler starts the replicator.
    let early = unique("replay_early");
    let _t: Topic<f32> = Topic::new(&early).expect("create topic");

    let seen = Arc::new(Mutex::new(Vec::new()));
    let sink = seen.clone();
    set_topic_lifecycle_hook(move |event| {
        if let TopicLifecycleEvent::Created { name, .. } = event {
            sink.lock().unwrap().push(name);
        }
    });

    let names = seen.lock().unwrap().clone();
    assert!(
        names.iter().any(|n| n == &early),
        "a hook installed after the topic was created never learned about it, so \
         the replicator's registry would be missing every topic the robot built \
         during init. saw: {names:?}"
    );
}
