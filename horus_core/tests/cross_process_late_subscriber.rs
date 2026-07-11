//! Cross-process harness for softmata-brain bug #2: an EARLY subscriber (created
//! before any message is sent) that reads AFTER the publisher process has exited.
//!
//! The in-process analog delivers everything (a dropped producer handle keeps the
//! SHM ring readable). Cross-process, the parent was observed to read 0 — but that
//! test (`cross_process_mpsc_3_pub_1_sub`) is `#[ignore]`, and the spawned children
//! re-exec WITHOUT `--include-ignored`, so they skipped the test and published
//! nothing, which masqueraded as the bug. This harness fixes that: the child re-exec
//! passes `--include-ignored`, so it genuinely runs. Run:
//!   cargo test --test cross_process_late_subscriber -- --include-ignored --nocapture

mod common;

use horus_core::communication::Topic;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

const CHILD_ENV: &str = "HORUS_LATESUB_CHILD";
const TOPIC_ENV: &str = "HORUS_LATESUB_TOPIC";
const COUNT_ENV: &str = "HORUS_LATESUB_COUNT";
const ID_ENV: &str = "HORUS_LATESUB_ID";

fn is_child() -> bool {
    std::env::var(CHILD_ENV).is_ok()
}

/// Child: publish `count` messages tagged with its id, print a marker, then EXIT.
fn child_publish() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV");
    let count: u64 = std::env::var(COUNT_ENV).expect("COUNT_ENV").parse().unwrap();
    let id: u64 = std::env::var(ID_ENV).expect("ID_ENV").parse().unwrap();
    // Let the parent (early subscriber) exist first.
    std::thread::sleep(Duration::from_millis(200));
    let t: Topic<u64> = Topic::new(&topic_name).expect("child topic");
    for i in 0..count {
        t.send(id * 1_000_000 + i); // tag by id so the parent can attribute
    }
    println!("CHILD_SENT:{id}:{count}");
}

fn spawn_child(test_name: &str, topic: &str, count: u64, id: u64) -> std::process::Child {
    let exe = std::env::current_exe().expect("current_exe");
    Command::new(exe)
        // NB: --include-ignored so the child actually runs this #[ignore] test.
        .args([test_name, "--exact", "--nocapture", "--include-ignored"])
        .env(CHILD_ENV, "1")
        .env(TOPIC_ENV, topic)
        .env(COUNT_ENV, count.to_string())
        .env(ID_ENV, id.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn child")
}

// Not #[ignore]: this is a real cross-process regression gate. The child re-exec
// passes --include-ignored (harmless when not ignored) so it keeps working if a future
// change re-ignores it.
#[test]
fn cross_process_late_subscriber_gets_buffered() {
    if is_child() {
        child_publish();
        return;
    }

    let _g = common::cleanup_stale_shm();
    let topic_name = format!("latesub_{}", std::process::id());
    let per_child = 20u64;
    let n_children = 3u64;
    let total = per_child * n_children;

    // Parent is an EARLY subscriber: it creates the topic (and owns the SHM) BEFORE
    // any message is sent.
    let sub: Topic<u64> = Topic::new(&topic_name).expect("parent sub");

    // Spawn N child publishers and WAIT for all to publish and fully EXIT.
    let children: Vec<_> = (0..n_children)
        .map(|id| {
            spawn_child(
                "cross_process_late_subscriber_gets_buffered",
                &topic_name,
                per_child,
                id,
            )
        })
        .collect();
    let mut child_sent = 0u64;
    for c in children {
        let out = c.wait_with_output().expect("child wait");
        if String::from_utf8_lossy(&out.stdout).contains("CHILD_SENT:") {
            child_sent += per_child;
        }
    }

    // All children have exited. Their messages should be buffered in the SHM ring
    // (the parent owns the file). Read them now.
    std::thread::sleep(Duration::from_millis(100));
    sub.check_migration_now();
    let mut got = Vec::new();
    let deadline = Instant::now() + Duration::from_secs(3);
    while (got.len() as u64) < total && Instant::now() < deadline {
        match sub.try_recv() {
            Some(v) => got.push(v),
            None => std::thread::yield_now(),
        }
    }
    let mut ids: std::collections::HashSet<u64> = got.iter().map(|v| v / 1_000_000).collect();
    ids.retain(|&id| id < n_children);

    eprintln!(
        "DIAG early-subscriber got {}/{} ({} children published) from {} distinct children",
        got.len(),
        total,
        child_sent,
        ids.len(),
    );
    assert_eq!(
        got.len() as u64,
        total,
        "early cross-process subscriber lost buffered messages after {} publishers exited (got {} of {})",
        n_children,
        got.len(),
        total
    );
}
