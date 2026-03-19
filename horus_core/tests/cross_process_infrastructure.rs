//! Cross-process infrastructure tests.
//!
//! Tests that BlackBox WAL files and SchedulerRegistry SHM files are
//! readable by external processes. Also tests Topic<[f64; 7]> cross-process
//! (TransformStamped-sized Pod data) to verify TF-equivalent payloads work.
//!
//! Run sequentially: `cargo test --test cross_process_infrastructure -- --test-threads=1`

use horus_core::communication::topic::Topic;
use horus_core::core::DurationExt;
use horus_core::scheduling::{BlackBox, BlackBoxEvent};
use horus_core::scheduling::registry::SchedulerRegistry;
use std::process::{Command, Stdio};
use std::time::Duration;

mod common;
use common::{cleanup_stale_shm, TestTempDir};

// ─── Env vars ────────────────────────────────────────────────────────────────

const CHILD_ENV: &str = "HORUS_INFRA_XPROC_CHILD";
const TEST_ENV: &str = "HORUS_INFRA_XPROC_TEST";
const PATH_ENV: &str = "HORUS_INFRA_XPROC_PATH";
const SCHED_ENV: &str = "HORUS_INFRA_XPROC_SCHED";
const TOPIC_ENV: &str = "HORUS_INFRA_XPROC_TOPIC";

fn is_child() -> bool {
    std::env::var(CHILD_ENV).is_ok()
}

// ─── Child entry points ──────────────────────────────────────────────────────

fn child_read_blackbox_wal() {
    let dir = std::env::var(PATH_ENV).expect("PATH_ENV not set");
    let wal_path = std::path::Path::new(&dir).join("blackbox.wal");

    // Wait for parent to write
    std::thread::sleep(Duration::from_millis(200));

    if !wal_path.exists() {
        println!("WAL_FOUND:false");
        return;
    }

    // Read WAL file (JSON lines)
    let content = std::fs::read_to_string(&wal_path).expect("read WAL");
    let mut event_count = 0;
    for line in content.lines() {
        if !line.trim().is_empty() {
            // Each line should be valid JSON
            let parsed: Result<serde_json::Value, _> = serde_json::from_str(line);
            if parsed.is_ok() {
                event_count += 1;
            }
        }
    }
    println!("WAL_FOUND:true");
    println!("EVENTS:{}", event_count);
}

fn child_read_registry() {
    let sched_name = std::env::var(SCHED_ENV).expect("SCHED_ENV not set");

    // Wait for parent to write
    std::thread::sleep(Duration::from_millis(200));

    // Read registry via the public API
    match SchedulerRegistry::read_all_slots(&sched_name) {
        Some(slots) => {
            println!("REGISTRY_FOUND:true");
            println!("SLOTS:{}", slots.len());
            for slot in &slots {
                println!("NODE:{}", slot.name);
            }
        }
        None => {
            println!("REGISTRY_FOUND:false");
        }
    }
}

fn child_recv_transform_pod() {
    let topic_name = std::env::var(TOPIC_ENV).expect("TOPIC_ENV not set");

    // Open topic for receiving
    let topic: Topic<[f64; 7]> = Topic::new(&topic_name).expect("child: Topic::new");

    let deadline = std::time::Instant::now() + 10_u64.secs();
    let mut received = Vec::new();

    while std::time::Instant::now() < deadline {
        if let Some(data) = topic.recv() {
            received.push(data);
            if received.len() >= 5 {
                break;
            }
        } else {
            std::thread::yield_now();
        }
    }

    println!("RECEIVED:{}", received.len());
    if let Some(first) = received.first() {
        // Print translation (first 3 elements)
        println!("TX:{:.1}", first[0]);
        println!("TY:{:.1}", first[1]);
        println!("TZ:{:.1}", first[2]);
    }
}

// ─── Spawn helper ────────────────────────────────────────────────────────────

fn spawn_child_with_envs(test_name: &str, envs: &[(&str, &str)]) -> std::process::Child {
    let exe = std::env::current_exe().expect("current_exe");
    let mut cmd = Command::new(exe);
    cmd.args([test_name, "--exact", "--nocapture"])
        .env(CHILD_ENV, "1")
        .env(TEST_ENV, test_name)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    for (k, v) in envs {
        cmd.env(k, v);
    }
    cmd.spawn().expect("failed to spawn child")
}

// ═══════════════════════════════════════════════════════════════════════════════
//  1. BlackBox WAL readable by another process
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn cross_process_blackbox_wal_readable() {
    if is_child() {
        if std::env::var(TEST_ENV).ok().as_deref()
            == Some("cross_process_blackbox_wal_readable")
        {
            child_read_blackbox_wal();
        }
        return;
    }

    let tmp = TestTempDir::new("xproc_bb");

    // Parent: create BlackBox, record events, flush
    let mut bb = BlackBox::new(1).with_path(tmp.path().to_path_buf());
    bb.record(BlackBoxEvent::SchedulerStart {
        name: "test_sched".to_string(),
        node_count: 2,
        config: "default".to_string(),
    });
    bb.record(BlackBoxEvent::NodeTick {
        name: "sensor".to_string(),
        duration_us: 42,
        success: true,
    });
    bb.record(BlackBoxEvent::DeadlineMiss {
        name: "slow_node".to_string(),
        deadline_us: 1000,
        actual_us: 2000,
    });
    bb.flush_wal();

    // Spawn child to read the WAL
    let dir_str = tmp.path().to_str().unwrap();
    let child = spawn_child_with_envs(
        "cross_process_blackbox_wal_readable",
        &[(PATH_ENV, dir_str)],
    );
    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);

    assert!(
        output.status.success(),
        "child should succeed. stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    // Parse results
    let wal_found = stdout.lines().find(|l| l.starts_with("WAL_FOUND:"));
    assert_eq!(
        wal_found.map(|l| l.strip_prefix("WAL_FOUND:").unwrap()),
        Some("true"),
        "child should find WAL file"
    );

    let events = stdout
        .lines()
        .find(|l| l.starts_with("EVENTS:"))
        .and_then(|l| l.strip_prefix("EVENTS:"))
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(0);

    assert!(
        events >= 3,
        "child should parse at least 3 events from WAL, found {}",
        events
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  2. SchedulerRegistry readable by another process
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn cross_process_registry_readable() {
    if is_child() {
        if std::env::var(TEST_ENV).ok().as_deref()
            == Some("cross_process_registry_readable")
        {
            child_read_registry();
        }
        return;
    }

    cleanup_stale_shm();

    let sched_name = format!("xproc_reg_{}", std::process::id());

    // Parent: create registry, register nodes
    let registry = SchedulerRegistry::open(&sched_name).expect("open registry");
    let slot0 = registry.register_node("sensor_driver", 0, 100.0, 0);
    let slot1 = registry.register_node("controller", 1, 1000.0, 0);

    // Update some metrics (health, tick_count, error_count, budget_misses, deadline_misses, last_tick_ns, avg_tick_ns, max_tick_ns)
    registry.update_node(slot0, 0, 50, 0, 0, 0, 1000, 500, 800);
    registry.update_node(slot1, 0, 100, 0, 0, 0, 2000, 1000, 1500);

    // Spawn child to read the registry
    let child = spawn_child_with_envs(
        "cross_process_registry_readable",
        &[(SCHED_ENV, &sched_name)],
    );
    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);

    assert!(
        output.status.success(),
        "child should succeed. stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let found = stdout
        .lines()
        .find(|l| l.starts_with("REGISTRY_FOUND:"))
        .and_then(|l| l.strip_prefix("REGISTRY_FOUND:"))
        .unwrap_or("false");
    assert_eq!(found, "true", "child should find registry");

    let slots: usize = stdout
        .lines()
        .find(|l| l.starts_with("SLOTS:"))
        .and_then(|l| l.strip_prefix("SLOTS:"))
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    assert!(
        slots >= 2,
        "child should find at least 2 node slots, found {}",
        slots
    );

    // Verify node names
    let node_names: Vec<&str> = stdout
        .lines()
        .filter_map(|l| l.strip_prefix("NODE:"))
        .collect();
    assert!(
        node_names.iter().any(|n| n.contains("sensor_driver")),
        "should find sensor_driver in registry"
    );
    assert!(
        node_names.iter().any(|n| n.contains("controller")),
        "should find controller in registry"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  3. Transform-sized Pod data via Topic cross-process
// ═══════════════════════════════════════════════════════════════════════════════

#[test]
fn cross_process_transform_pod_topic() {
    if is_child() {
        if std::env::var(TEST_ENV).ok().as_deref()
            == Some("cross_process_transform_pod_topic")
        {
            child_recv_transform_pod();
        }
        return;
    }

    cleanup_stale_shm();

    let topic_name = format!("xproc_tf_{}", std::process::id());

    // Parent: create topic, send transform-sized Pod data
    // [f64; 7] = translation(3) + rotation quaternion(4) = 56 bytes (TransformStamped-sized)
    let topic: Topic<[f64; 7]> = Topic::new(&topic_name).expect("parent: Topic::new");

    // Trigger producer role
    topic.send([1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0]);

    // Spawn child
    let child = spawn_child_with_envs(
        "cross_process_transform_pod_topic",
        &[(TOPIC_ENV, &topic_name)],
    );

    // Wait for child to register, then send more
    std::thread::sleep(Duration::from_millis(500));
    topic.check_migration_now();

    for i in 0..10 {
        topic.send([1.0 + i as f64, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0]);
        std::thread::sleep(Duration::from_millis(50));
    }

    let output = child.wait_with_output().expect("child wait failed");
    let stdout = String::from_utf8_lossy(&output.stdout);

    assert!(
        output.status.success(),
        "child should succeed. stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let received: usize = stdout
        .lines()
        .find(|l| l.starts_with("RECEIVED:"))
        .and_then(|l| l.strip_prefix("RECEIVED:"))
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);

    assert!(
        received > 0,
        "child should receive transform data via Topic"
    );
}
