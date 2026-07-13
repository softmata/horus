//! Cross-process gate for the FanoutShm oversized-serde spill fix (roadmap
//! roadmap-mrgqzxyo-ayjt6u).
//!
//! One parent publisher → two child subscriber processes selects FanoutShm
//! broadcast (the proven topology from `cross_process_fanout_1p2s`). The payload
//! is a 50 KiB String — far larger than the fixed 8 KiB fanout slot — so it MUST
//! be spilled: the publisher writes the bytes into the SHM-backed TensorPool and
//! broadcasts only a 40-byte SpillDescriptor; each subscriber process resolves the
//! same pool (deterministic pool_id from the topic name) and reconstructs the
//! bytes. Before the fix this message was silently dropped on FanoutShm.
//!
//! Verifies delivery AND integrity (each received String is compared byte-for-byte
//! to the expected 50 KiB pattern, so a torn/truncated spill is caught).
//!
//! Run: cargo test --no-default-features --test cross_process_fanout_spill -- --test-threads=1 --nocapture

use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use horus_core::communication::topic::Topic;

mod common;
use common::cleanup_stale_shm;

const CHILD_ENV: &str = "HORUS_FANOUT_SPILL_CHILD";
const TEST_ENV: &str = "HORUS_FANOUT_SPILL_TEST";
const TOPIC_ENV: &str = "HORUS_FANOUT_SPILL_TOPIC";

/// 50 KiB — far larger than the 8 KiB fanout slot, so every send MUST spill.
const MSG_LEN: usize = 50_000;
const SENTINEL: &str = "__FANOUT_SPILL_END__";

/// Position-dependent pattern so truncation or corruption is detectable, not just
/// a length check. Publisher and subscriber both regenerate it independently.
fn payload() -> String {
    (0..MSG_LEN).map(|i| (b'A' + (i % 26) as u8) as char).collect()
}

fn is_child() -> bool {
    std::env::var(CHILD_ENV).is_ok()
}

fn unique(prefix: &str) -> String {
    static COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    format!(
        "{}_{}_{}",
        prefix,
        std::process::id(),
        COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
    )
}

/// Subscriber child: receive oversized messages until sentinel/timeout, counting
/// how many arrived intact vs corrupted.
fn child_subscriber() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV");
    let topic: Topic<String> = Topic::new(&topic_name).expect("child subscriber: Topic::new");
    let expected = payload();

    let mut ok = 0usize;
    let mut bad = 0usize;
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut last_check = Instant::now();
    loop {
        if Instant::now() >= deadline {
            break;
        }
        match topic.recv() {
            Some(ref s) if s == SENTINEL => break,
            Some(s) => {
                if s == expected {
                    ok += 1;
                } else {
                    bad += 1;
                }
            }
            None => {
                std::thread::yield_now();
                if last_check.elapsed() > Duration::from_millis(100) {
                    topic.check_migration_now();
                    last_check = Instant::now();
                }
            }
        }
    }
    println!("SUB_OK:{ok} SUB_BAD:{bad} BACKEND:{}", topic.backend_name());
}

fn spawn_sub(test_name: &str, topic: &str, id: &str) -> std::process::Child {
    let exe = std::env::current_exe().expect("current_exe");
    Command::new(exe)
        .args([test_name, "--exact", "--nocapture"])
        .env(CHILD_ENV, "1")
        .env(TEST_ENV, test_name)
        .env(TOPIC_ENV, topic)
        .env("HORUS_FANOUT_SPILL_ID", id)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("failed to spawn child")
}

fn field(stdout: &str, key: &str) -> usize {
    stdout
        .split_whitespace()
        .find_map(|tok| tok.strip_prefix(key))
        .and_then(|v| v.parse().ok())
        .unwrap_or(usize::MAX)
}

#[test]
fn cross_process_fanout_spill_1pub_2sub() {
    if is_child() {
        if std::env::var(TEST_ENV).ok().as_deref() == Some("cross_process_fanout_spill_1pub_2sub") {
            child_subscriber();
        }
        return;
    }

    let _shm_guard = cleanup_stale_shm();
    let topic_name = unique("xproc_fanout_spill");

    // Spawn 2 subscriber children first (they own the recv side).
    let sub1 = spawn_sub("cross_process_fanout_spill_1pub_2sub", &topic_name, "1");
    let sub2 = spawn_sub("cross_process_fanout_spill_1pub_2sub", &topic_name, "2");

    // Let subscribers register before the parent publishes.
    std::thread::sleep(Duration::from_millis(800));

    // Parent: publish the oversized message repeatedly, then the sentinel.
    let topic: Topic<String> = Topic::new(&topic_name).expect("parent: Topic::new");
    std::thread::sleep(Duration::from_millis(200));
    let msg = payload();
    for _ in 0..60 {
        topic.send(msg.clone());
        std::thread::sleep(Duration::from_millis(3));
    }
    topic.send(SENTINEL.to_string());

    let out1 = sub1.wait_with_output().expect("sub1 wait");
    let out2 = sub2.wait_with_output().expect("sub2 wait");
    let s1 = String::from_utf8_lossy(&out1.stdout);
    let s2 = String::from_utf8_lossy(&out2.stdout);
    eprintln!("DIAG parent_backend={} | sub1: {} | sub2: {}", topic.backend_name(), s1.trim(), s2.trim());

    let (ok1, bad1) = (field(&s1, "SUB_OK:"), field(&s1, "SUB_BAD:"));
    let (ok2, bad2) = (field(&s2, "SUB_OK:"), field(&s2, "SUB_BAD:"));

    // No corruption/truncation on either subscriber — the spilled bytes must
    // round-trip byte-for-byte across the process boundary via the pool.
    assert_eq!(bad1, 0, "sub1 received corrupted/truncated spilled messages: {s1}");
    assert_eq!(bad2, 0, "sub2 received corrupted/truncated spilled messages: {s2}");
    // Each subscriber must receive the >8 KiB message at least once (was dropped
    // entirely before the spill fix).
    assert!(ok1 > 0, "sub1 must receive the >8 KiB message intact via spill: {s1}");
    assert!(ok2 > 0, "sub2 must receive the >8 KiB message intact via spill: {s2}");
}

// ===========================================================================
// COMM-H3: spill-pool reclaim — delivery must continue past the pool's slot
// capacity. Every spilled slot used to leak (never released), so after
// `max_slots` (1024) spills the pool filled and every further >8 KiB message
// was silently dropped. This sends MORE than 1024 spilled messages and proves a
// subscriber still receives ones sent *after* the pool would have been
// exhausted — which is only possible if the producer reclaims spilled slots.
// (Delivery past 1024 also proves the slot count stayed bounded — an unbounded
// leak would have stopped delivery at 1024.)
// ===========================================================================

/// > 1024 = TensorPool default `max_slots`. Without reclaim the pool fills here.
const RECLAIM_N: usize = 1200;

/// Payload for message `idx`: a parseable `IDX:<n>:` header + a position- and
/// index-dependent tail, sized > the 8 KiB fanout slot so every send spills and
/// (pre-fix) is dropped once the pool is full. The tail lets a subscriber detect
/// truncation/corruption, not just presence.
fn indexed_payload(idx: usize) -> String {
    let head = format!("IDX:{idx:08}:");
    let tail_len = MSG_LEN - head.len();
    let mut s = String::with_capacity(MSG_LEN);
    s.push_str(&head);
    s.extend((0..tail_len).map(|i| (b'a' + ((i + idx) % 26) as u8) as char));
    s
}

/// Parse+validate an indexed payload. Returns the index only if the tail matches
/// exactly (so a torn/mismatched spill is rejected, not miscounted).
fn parse_indexed(s: &str) -> Option<usize> {
    let rest = s.strip_prefix("IDX:")?;
    let (num, tail) = rest.split_once(':')?;
    let idx: usize = num.parse().ok()?;
    let head_len = format!("IDX:{idx:08}:").len();
    if tail.len() != MSG_LEN - head_len {
        return None;
    }
    if tail
        .bytes()
        .enumerate()
        .any(|(i, c)| c != b'a' + ((i + idx) % 26) as u8)
    {
        return None;
    }
    Some(idx)
}

fn child_subscriber_indexed() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV");
    let topic: Topic<String> = Topic::new(&topic_name).expect("child subscriber: Topic::new");

    let mut max_idx = 0usize;
    let mut ok = 0usize;
    let mut bad = 0usize;
    let deadline = Instant::now() + Duration::from_secs(30);
    let mut last_check = Instant::now();
    loop {
        if Instant::now() >= deadline {
            break;
        }
        match topic.recv() {
            Some(ref s) if s == SENTINEL => break,
            Some(s) => match parse_indexed(&s) {
                Some(idx) => {
                    ok += 1;
                    max_idx = max_idx.max(idx);
                }
                None => bad += 1,
            },
            None => {
                std::thread::yield_now();
                if last_check.elapsed() > Duration::from_millis(100) {
                    topic.check_migration_now();
                    last_check = Instant::now();
                }
            }
        }
    }
    println!("SUB_OK:{ok} SUB_BAD:{bad} SUB_MAX:{max_idx} BACKEND:{}", topic.backend_name());
}

#[test]
fn cross_process_fanout_spill_reclaim_past_pool_capacity() {
    if is_child() {
        if std::env::var(TEST_ENV).ok().as_deref()
            == Some("cross_process_fanout_spill_reclaim_past_pool_capacity")
        {
            child_subscriber_indexed();
        }
        return;
    }

    let _shm_guard = cleanup_stale_shm();
    let topic_name = unique("xproc_fanout_reclaim");
    let test = "cross_process_fanout_spill_reclaim_past_pool_capacity";

    let sub1 = spawn_sub(test, &topic_name, "1");
    let sub2 = spawn_sub(test, &topic_name, "2");

    // Let subscribers register before the parent publishes.
    std::thread::sleep(Duration::from_millis(800));

    let topic: Topic<String> = Topic::new(&topic_name).expect("parent: Topic::new");
    std::thread::sleep(Duration::from_millis(200));

    // Publish RECLAIM_N (> max_slots) distinct oversized messages. The small
    // inter-send sleep gives the drop-oldest subscribers a window to observe
    // intermediate indices (so max is not just the final message).
    for idx in 1..=RECLAIM_N {
        topic.send(indexed_payload(idx));
        std::thread::sleep(Duration::from_millis(1));
    }
    // Let subscribers drain the tail before the (small, non-spilling) sentinel.
    std::thread::sleep(Duration::from_millis(300));
    topic.send(SENTINEL.to_string());

    let out1 = sub1.wait_with_output().expect("sub1 wait");
    let out2 = sub2.wait_with_output().expect("sub2 wait");
    let s1 = String::from_utf8_lossy(&out1.stdout);
    let s2 = String::from_utf8_lossy(&out2.stdout);
    eprintln!(
        "DIAG reclaim parent_backend={} | sub1: {} | sub2: {}",
        topic.backend_name(),
        s1.trim(),
        s2.trim()
    );

    let (bad1, max1) = (field(&s1, "SUB_BAD:"), field(&s1, "SUB_MAX:"));
    let (bad2, max2) = (field(&s2, "SUB_BAD:"), field(&s2, "SUB_MAX:"));

    // No corruption/truncation on either subscriber.
    assert_eq!(bad1, 0, "sub1 received corrupted spilled messages: {s1}");
    assert_eq!(bad2, 0, "sub2 received corrupted spilled messages: {s2}");
    // The load-bearing assertion: a subscriber received a message sent AFTER the
    // 1024-slot pool would have been exhausted. Pre-fix (no reclaim) the pool
    // fills at 1024 and every later >8 KiB message is dropped, capping max at
    // ~1024. Post-fix the producer reclaims overwritten slots, so delivery
    // continues to ~RECLAIM_N. Require clearly past the 1024 boundary.
    assert!(
        max1 > 1050 || max2 > 1050,
        "delivery stalled at the pool capacity — spilled slots were not reclaimed \
         (max1={max1}, max2={max2}); expected a subscriber to see an index > 1050 \
         out of {RECLAIM_N}. sub1: {s1} | sub2: {s2}"
    );
}
