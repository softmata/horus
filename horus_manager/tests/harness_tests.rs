//! Acceptance tests for the HORUS monitor using the live test harness.
//!
//! These tests create real presence files, SHM topic files, and blackbox WAL
//! entries, then verify the monitor's discovery layer can find them.

mod harness;

use harness::{HorusTestRuntime, TestNodeConfig};

use horus_core::NodePresence;
use std::fs;
use std::time::Duration;

// ── Harness self-tests ──────────────────────────────────────────────────────

/// Verify that adding a node writes a presence file that
/// `NodePresence::read_all()` can discover.
#[test]
fn harness_add_node_discoverable() {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::bare("harness_smoke_node"));

    assert!(
        rt.wait_ready(Duration::from_secs(2)),
        "node should be discoverable within 2 seconds"
    );

    let all = NodePresence::read_all();
    let found = all.iter().any(|p| p.name == "harness_smoke_node");
    assert!(found, "harness_smoke_node not found in read_all()");
}

/// Verify that Drop cleans up all presence and SHM files.
#[test]
fn harness_cleanup_on_drop() {
    let paths;
    let topic_paths;
    {
        let mut rt = HorusTestRuntime::new();
        rt.add_node(TestNodeConfig::bare("harness_drop_test"))
            .add_topic("harness_drop_topic", 1024);
        paths = rt.presence_paths().to_vec();
        topic_paths = rt.topic_paths().to_vec();

        // Files should exist while runtime is alive.
        for p in &paths {
            assert!(p.exists(), "presence file should exist: {}", p.display());
        }
        for p in &topic_paths {
            assert!(p.exists(), "topic file should exist: {}", p.display());
        }
    }
    // After drop, files should be gone.
    for p in &paths {
        assert!(
            !p.exists(),
            "presence file should be removed: {}",
            p.display()
        );
    }
    for p in &topic_paths {
        assert!(!p.exists(), "topic file should be removed: {}", p.display());
    }
}

/// Verify that the sensor preset works and presence fields are correct.
#[test]
fn harness_sensor_preset() {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::sensor(
        "harness_lidar_node",
        "scan_data",
        "LaserScan",
    ));

    assert!(rt.wait_ready(Duration::from_secs(2)));

    let all = NodePresence::read_all();
    let node = all.iter().find(|p| p.name == "harness_lidar_node");
    assert!(node.is_some(), "harness_lidar_node not found");

    let node = node.unwrap();
    assert_eq!(node.publishers.len(), 1);
    assert_eq!(node.publishers[0].topic_name, "scan_data");
    assert_eq!(node.publishers[0].type_name, "LaserScan");
    assert_eq!(node.pid, std::process::id());
}

/// Verify topic file creation under horus_topic/ subdirectory.
#[test]
fn harness_add_topic_creates_file() {
    let mut rt = HorusTestRuntime::new();
    rt.add_topic("harness_test_topic", 2048);

    assert_eq!(rt.topic_paths().len(), 1);
    let path = &rt.topic_paths()[0];
    assert!(path.exists());

    let meta = fs::metadata(path).unwrap();
    assert_eq!(meta.len(), 2048);
}

/// Verify raw topic file creation at the top level.
#[test]
fn harness_add_raw_topic_creates_file() {
    let mut rt = HorusTestRuntime::new();
    rt.add_raw_topic("harness_raw_topic", 512);

    assert_eq!(rt.topic_paths().len(), 1);
    let path = &rt.topic_paths()[0];
    assert!(path.exists());

    let meta = fs::metadata(path).unwrap();
    assert_eq!(meta.len(), 512);
}

/// Verify blackbox event injection writes to WAL.
#[test]
fn harness_blackbox_injection() {
    let mut rt = HorusTestRuntime::new();
    rt.inject_blackbox_custom("test", "hello from harness");

    let dir = rt.blackbox_dir().expect("blackbox dir should exist");
    let wal_path = dir.join("blackbox.wal");
    assert!(wal_path.exists(), "WAL file should exist");

    let content = fs::read_to_string(&wal_path).unwrap();
    assert!(content.contains("hello from harness"));
}

/// Verify log injection does not panic.
#[test]
fn harness_log_injection() {
    let rt = HorusTestRuntime::new();
    rt.inject_log("test_node", "info", "harness log entry");
    rt.inject_log("test_node", "error", "harness error entry");
    rt.inject_log("test_node", "warning", "harness warning entry");
    // If we get here without panicking, the log buffer accepts our entries.
}

/// Verify multi-node scenario with publisher/subscriber graph.
#[test]
fn harness_multi_node() {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::sensor("harness_camera", "image", "Image"))
        .add_node(TestNodeConfig::processor(
            "harness_detector",
            "image",
            "Image",
            "detections",
            "BBoxArray",
        ))
        .add_node(TestNodeConfig::actuator(
            "harness_arm",
            "detections",
            "BBoxArray",
        ));

    assert!(rt.wait_ready(Duration::from_secs(2)));
    assert_eq!(rt.node_names().len(), 3);

    let all = NodePresence::read_all();
    let names: Vec<&str> = all.iter().map(|p| p.name.as_str()).collect();

    for expected in &["harness_camera", "harness_detector", "harness_arm"] {
        assert!(
            names.contains(expected),
            "{} not found in {:?}",
            expected,
            names
        );
    }

    // Verify the processor has both publishers and subscribers.
    let detector = all.iter().find(|p| p.name == "harness_detector").unwrap();
    assert_eq!(detector.publishers.len(), 1);
    assert_eq!(detector.subscribers.len(), 1);
    assert_eq!(detector.publishers[0].topic_name, "detections");
    assert_eq!(detector.subscribers[0].topic_name, "image");
}

/// Verify builder pattern methods on TestNodeConfig.
#[test]
fn harness_builder_pattern() {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(
        TestNodeConfig::bare("harness_builder_node")
            .with_scheduler("test_scheduler")
            .with_rate_hz(50.0)
            .with_priority(42)
            .with_publisher("output_a", "TypeA")
            .with_subscriber("input_b", "TypeB"),
    );

    assert!(rt.wait_ready(Duration::from_secs(2)));

    let all = NodePresence::read_all();
    let node = all
        .iter()
        .find(|p| p.name == "harness_builder_node")
        .expect("harness_builder_node not found");

    assert_eq!(node.scheduler.as_deref(), Some("test_scheduler"));
    assert_eq!(node.rate_hz, Some(50.0));
    assert_eq!(node.priority, 42);
    assert_eq!(node.publishers.len(), 1);
    assert_eq!(node.subscribers.len(), 1);
    assert_eq!(node.publishers[0].topic_name, "output_a");
    assert_eq!(node.subscribers[0].topic_name, "input_b");
}

/// Verify that the actuator preset works.
#[test]
fn harness_actuator_preset() {
    let mut rt = HorusTestRuntime::new();
    rt.add_node(TestNodeConfig::actuator(
        "harness_motor",
        "cmd_vel",
        "Twist",
    ));

    assert!(rt.wait_ready(Duration::from_secs(2)));

    let all = NodePresence::read_all();
    let node = all
        .iter()
        .find(|p| p.name == "harness_motor")
        .expect("harness_motor not found");

    assert!(node.publishers.is_empty());
    assert_eq!(node.subscribers.len(), 1);
    assert_eq!(node.subscribers[0].topic_name, "cmd_vel");
    assert_eq!(node.subscribers[0].type_name, "Twist");
}

/// Verify that multiple blackbox events accumulate in the WAL.
#[test]
fn harness_blackbox_multiple_events() {
    use horus_core::scheduling::BlackBoxEvent;

    let mut rt = HorusTestRuntime::new();
    rt.inject_blackbox_event(BlackBoxEvent::SchedulerStart {
        name: "test_sched".to_string(),
        node_count: 3,
        config: "default".to_string(),
    });
    rt.inject_blackbox_custom("perf", "latency spike");
    rt.inject_blackbox_event(BlackBoxEvent::NodeTick {
        name: "sensor".to_string(),
        duration_us: 123,
        success: true,
    });

    let dir = rt.blackbox_dir().unwrap();
    let wal = fs::read_to_string(dir.join("blackbox.wal")).unwrap();
    let lines: Vec<&str> = wal.lines().collect();
    assert_eq!(lines.len(), 3, "WAL should have 3 entries");
    assert!(wal.contains("test_sched"));
    assert!(wal.contains("latency spike"));
    assert!(wal.contains("sensor"));
}
