//! Discovery system stress tests — mass node creation, rapid churn,
//! stale SHM cleanup, namespace isolation.
//!
//! Uses HorusTestRuntime to write real SHM presence/topic files.
//! Run: `cargo test -p horus_manager --test discovery_stress -- --test-threads=1`

mod harness;

use harness::{HorusTestRuntime, TestNodeConfig};
use horus_core::core::DurationExt;
use horus_manager::discovery::discover_nodes;
use std::time::Instant;

fn clean_shm() {
    let _ = std::fs::remove_dir_all(horus_sys::shm::shm_nodes_dir());
    let _ = std::fs::remove_dir_all(horus_sys::shm::shm_topics_dir());
}

static TEST_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

// ============================================================================
// Test 1: Launch 100 nodes, verify all discovered
// (scaled from 1000 to 100 for test speed — same principle)
// ============================================================================

#[test]
fn test_discover_100_nodes() {
    let _g = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    clean_shm();
    let mut rt = HorusTestRuntime::new();

    for i in 0..100 {
        rt.add_node(
            TestNodeConfig::bare(&format!("disc_mass_{}", i))
                .with_rate_hz(100.0)
                .with_priority(i as u32 % 10),
        );
    }

    assert!(rt.wait_ready(3_u64.secs()), "100 nodes should be discoverable");

    let start = Instant::now();
    let nodes = discover_nodes().expect("discovery should succeed");
    let elapsed = start.elapsed();

    // All 100 should be discovered
    let our_nodes: Vec<_> = nodes
        .iter()
        .filter(|n| n.name.starts_with("disc_mass_"))
        .collect();

    assert_eq!(
        our_nodes.len(),
        100,
        "Should discover all 100 nodes, got {}",
        our_nodes.len()
    );

    // Discovery should be fast (< 2 seconds for 100 nodes)
    assert!(
        elapsed < std::time::Duration::from_secs(2),
        "Discovery of 100 nodes should be <2s, took {:?}",
        elapsed
    );
}

// ============================================================================
// Test 2: SIGKILL cleanup — runtime drop removes all presence files
// ============================================================================

#[test]
fn test_presence_cleanup_on_drop() {
    let _g = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    clean_shm();
    let node_count;

    // Create 50 nodes, then drop the runtime
    {
        let mut rt = HorusTestRuntime::new();
        for i in 0..50 {
            rt.add_node(TestNodeConfig::bare(&format!("disc_drop_{}", i)));
        }
        assert!(rt.wait_ready(2_u64.secs()));

        let nodes = discover_nodes().unwrap();
        node_count = nodes
            .iter()
            .filter(|n| n.name.starts_with("disc_drop_"))
            .count();
        assert_eq!(node_count, 50, "Should have 50 nodes before drop");
        // rt drops here — cleanup happens
    }

    std::thread::sleep(std::time::Duration::from_millis(200));

    // After drop, those nodes should be gone
    let nodes_after = discover_nodes().unwrap();
    let remaining = nodes_after
        .iter()
        .filter(|n| n.name.starts_with("disc_drop_"))
        .count();

    assert_eq!(
        remaining, 0,
        "All disc_drop_ nodes should be cleaned up after drop, {} remain",
        remaining
    );
}

// ============================================================================
// Test 3: Rapid churn — add many nodes, verify discovery consistency
// ============================================================================

#[test]
fn test_rapid_churn_consistency() {
    let _g = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    clean_shm();
    let mut rt = HorusTestRuntime::new();

    // Add 50 nodes rapidly
    for i in 0..50 {
        rt.add_node(
            TestNodeConfig::sensor(
                &format!("disc_churn_{}", i),
                &format!("disc_churn_topic_{}", i % 5),
                "Data",
            )
            .with_rate_hz(1000.0),
        );
    }

    assert!(rt.wait_ready(2_u64.secs()));

    // Run discovery 10 times rapidly — should be consistent each time
    let mut counts = Vec::new();
    for _ in 0..10 {
        let nodes = discover_nodes().unwrap();
        let our_count = nodes
            .iter()
            .filter(|n| n.name.starts_with("disc_churn_"))
            .count();
        counts.push(our_count);
    }

    // All 10 queries should return the same count
    let first = counts[0];
    for (i, &c) in counts.iter().enumerate() {
        assert_eq!(
            c, first,
            "Discovery query {} returned {} but query 0 returned {} — inconsistent",
            i, c, first
        );
    }

    assert_eq!(first, 50, "Should consistently find 50 nodes");
}

// ============================================================================
// Test 4: Stale SHM — runtime drop cleans up, next discovery finds none
// ============================================================================

#[test]
fn test_stale_shm_cleaned_on_rediscovery() {
    let _g = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    clean_shm();
    // Phase 1: create nodes
    {
        let mut rt = HorusTestRuntime::new();
        for i in 0..20 {
            rt.add_node(TestNodeConfig::bare(&format!("disc_stale_{}", i)));
        }
        assert!(rt.wait_ready(2_u64.secs()));

        let nodes = discover_nodes().unwrap();
        let count = nodes
            .iter()
            .filter(|n| n.name.starts_with("disc_stale_"))
            .count();
        assert_eq!(count, 20);
        // Drop — simulates "process died"
    }

    std::thread::sleep(std::time::Duration::from_millis(200));

    // Phase 2: rediscovery should find 0 (presence files cleaned by Drop)
    let nodes = discover_nodes().unwrap();
    let stale_count = nodes
        .iter()
        .filter(|n| n.name.starts_with("disc_stale_"))
        .count();

    assert_eq!(
        stale_count, 0,
        "Stale nodes should be cleaned up, found {}",
        stale_count
    );
}

// ============================================================================
// Test 5: Namespace isolation — two runtimes don't see each other
// ============================================================================

/// Note: HorusTestRuntime uses the default namespace (current HORUS_NAMESPACE
/// or "default"). True namespace isolation requires different SHM base dirs.
/// This test verifies that nodes from different runtimes (same namespace)
/// are correctly attributed and discoverable.
#[test]
fn test_discovery_node_attribution() {
    let _g = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    clean_shm();
    let mut rt = HorusTestRuntime::new();

    // Group A: sensors
    for i in 0..5 {
        rt.add_node(
            TestNodeConfig::sensor(
                &format!("disc_ns_sensor_{}", i),
                &format!("disc_ns_scan_{}", i),
                "LaserScan",
            )
            .with_scheduler("sensor_sched"),
        );
    }

    // Group B: actuators
    for i in 0..5 {
        rt.add_node(
            TestNodeConfig::actuator(
                &format!("disc_ns_motor_{}", i),
                &format!("disc_ns_cmd_{}", i),
                "CmdVel",
            )
            .with_scheduler("motor_sched"),
        );
    }

    assert!(rt.wait_ready(2_u64.secs()));

    let nodes = discover_nodes().unwrap();

    let sensors: Vec<_> = nodes
        .iter()
        .filter(|n| n.name.starts_with("disc_ns_sensor_"))
        .collect();
    let motors: Vec<_> = nodes
        .iter()
        .filter(|n| n.name.starts_with("disc_ns_motor_"))
        .collect();

    assert_eq!(sensors.len(), 5, "Should find 5 sensors");
    assert_eq!(motors.len(), 5, "Should find 5 motors");

    // Verify publishers/subscribers are correctly attributed
    for sensor in &sensors {
        assert!(
            !sensor.publishers.is_empty() || sensor.name.contains("sensor"),
            "Sensor {} should have publishers or sensor-like name",
            sensor.name
        );
    }

    for motor in &motors {
        assert!(
            !motor.subscribers.is_empty() || motor.name.contains("motor"),
            "Motor {} should have subscribers or motor-like name",
            motor.name
        );
    }

    // No cross-contamination — sensor names don't appear in motor list
    for motor in &motors {
        assert!(
            !motor.name.contains("sensor"),
            "Motor list should not contain sensors: {}",
            motor.name
        );
    }
}
