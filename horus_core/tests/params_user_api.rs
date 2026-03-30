//! RuntimeParams user-facing API tests.
//!
//! Tests the exact API a user calls from their nodes to get/set
//! configuration parameters — PID gains, speed limits, sensor config.
//!
//! Run: cargo test --no-default-features -p horus_core \
//!        --test params_user_api -- --nocapture

use horus_core::params::RuntimeParams;

// ════════════════════════════════════════════════════════════════════════
// TEST 1: Basic set/get round-trip for all types
// ════════════════════════════════════════════════════════════════════════

#[test]
fn params_set_get_bool() {
    let params = RuntimeParams::new().expect("create params");
    params.set("enabled", true).unwrap();
    let val: bool = params.get("enabled").expect("should exist");
    assert_eq!(val, true);
}

#[test]
fn params_set_get_i64() {
    let params = RuntimeParams::new().unwrap();
    params.set("max_retries", 42i64).unwrap();
    let val: i64 = params.get("max_retries").expect("should exist");
    assert_eq!(val, 42);
}

#[test]
fn params_set_get_f64() {
    let params = RuntimeParams::new().unwrap();
    params.set("kp", 2.5f64).unwrap();
    let val: f64 = params.get("kp").expect("should exist");
    assert!((val - 2.5).abs() < 1e-10);
}

#[test]
fn params_set_get_string() {
    let params = RuntimeParams::new().unwrap();
    params.set("robot_name", "atlas").unwrap();
    let val: String = params.get("robot_name").expect("should exist");
    assert_eq!(val, "atlas");
}

#[test]
fn params_set_get_vec() {
    let params = RuntimeParams::new().unwrap();
    params.set("joint_ids", vec![1, 2, 3, 4, 5]).unwrap();
    let val: Vec<i64> = params.get("joint_ids").expect("should exist");
    assert_eq!(val, vec![1, 2, 3, 4, 5]);
}

// ════════════════════════════════════════════════════════════════════════
// TEST 2: get_or returns default when key missing
// ════════════════════════════════════════════════════════════════════════

#[test]
fn params_get_or_default() {
    let params = RuntimeParams::new().unwrap();
    let val: f64 = params.get_or("nonexistent_kp", 1.0);
    assert!((val - 1.0).abs() < 1e-10, "should return default");

    // Set it, then get_or should return the set value
    params.set("nonexistent_kp", 2.75).unwrap();
    let val2: f64 = params.get_or("nonexistent_kp", 1.0);
    assert!(
        (val2 - 2.75).abs() < 1e-10,
        "should return set value, not default"
    );
}

// ════════════════════════════════════════════════════════════════════════
// TEST 3: remove deletes parameter
// ════════════════════════════════════════════════════════════════════════

#[test]
fn params_remove() {
    let params = RuntimeParams::new().unwrap();
    params.set("temp_key", "hello").unwrap();
    assert!(params.has("temp_key"));

    let removed = params.remove("temp_key");
    assert!(removed.is_some(), "should return removed value");
    assert!(!params.has("temp_key"), "should no longer exist");

    let val: Option<String> = params.get("temp_key");
    assert!(val.is_none(), "get after remove should return None");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 4: get_all returns full map
// ════════════════════════════════════════════════════════════════════════

#[test]
fn params_get_all() {
    let params = RuntimeParams::new().unwrap();
    params.set("a", 1i64).unwrap();
    params.set("b", 2i64).unwrap();
    params.set("c", 3i64).unwrap();

    let all = params.get_all();
    // Should contain at least our 3 keys (may have defaults too)
    assert!(all.contains_key("a"), "should contain 'a'");
    assert!(all.contains_key("b"), "should contain 'b'");
    assert!(all.contains_key("c"), "should contain 'c'");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 5: list_keys and has
// ════════════════════════════════════════════════════════════════════════

#[test]
fn params_list_keys_and_has() {
    let params = RuntimeParams::new().unwrap();
    params.set("sensor_rate", 100i64).unwrap();
    params.set("motor_kp", 5.0f64).unwrap();

    let keys = params.list_keys();
    assert!(keys.contains(&"sensor_rate".to_string()));
    assert!(keys.contains(&"motor_kp".to_string()));

    assert!(params.has("sensor_rate"));
    assert!(params.has("motor_kp"));
    assert!(!params.has("nonexistent_key"));
}

// ════════════════════════════════════════════════════════════════════════
// TEST 6: Overwrite — set same key twice
// ════════════════════════════════════════════════════════════════════════

#[test]
fn params_overwrite() {
    let params = RuntimeParams::new().unwrap();
    params.set("gain", 1.0f64).unwrap();
    assert!((params.get::<f64>("gain").unwrap() - 1.0).abs() < 1e-10);

    params.set("gain", 5.0f64).unwrap();
    assert!(
        (params.get::<f64>("gain").unwrap() - 5.0).abs() < 1e-10,
        "should be overwritten to 5.0"
    );
}

// ════════════════════════════════════════════════════════════════════════
// TEST 7: Version tracking — set bumps version
// ════════════════════════════════════════════════════════════════════════

#[test]
fn params_version_tracking() {
    let params = RuntimeParams::new().unwrap();
    let v0 = params.get_version("versioned_key");
    assert_eq!(v0, 0, "unset key should have version 0");

    params.set("versioned_key", "first").unwrap();
    let v1 = params.get_version("versioned_key");
    assert!(v1 > v0, "version should increase after set");

    params.set("versioned_key", "second").unwrap();
    let v2 = params.get_version("versioned_key");
    assert!(v2 > v1, "version should increase on each set");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 8: get_typed returns error for wrong type
// ════════════════════════════════════════════════════════════════════════

#[test]
fn params_get_typed_error() {
    let params = RuntimeParams::new().unwrap();
    params.set("string_val", "hello").unwrap();

    // Try to get as i64 — should fail
    let result = params.get_typed::<i64>("string_val");
    assert!(result.is_err(), "getting string as i64 should fail");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 9: Realistic robot parameter workflow
// ════════════════════════════════════════════════════════════════════════

#[test]
fn params_robot_workflow() {
    let params = RuntimeParams::new().unwrap();

    // Operator sets PID gains
    params.set("pid.kp", 2.0f64).unwrap();
    params.set("pid.ki", 0.1f64).unwrap();
    params.set("pid.kd", 0.05f64).unwrap();

    // Operator sets speed limits
    params.set("max_speed", 1.5f64).unwrap();
    params.set("max_angular_speed", 1.0f64).unwrap();

    // Operator sets sensor config
    params.set("lidar.rate_hz", 10i64).unwrap();
    params.set("camera.resolution", "640x480").unwrap();

    // Node reads params at runtime
    let kp: f64 = params.get_or("pid.kp", 1.0);
    let ki: f64 = params.get_or("pid.ki", 0.0);
    let kd: f64 = params.get_or("pid.kd", 0.0);
    let max_v: f64 = params.get_or("max_speed", 0.5);
    let lidar_hz: i64 = params.get_or("lidar.rate_hz", 10);
    let cam_res: String = params.get_or("camera.resolution", "320x240".into());

    assert!((kp - 2.0).abs() < 1e-10);
    assert!((ki - 0.1).abs() < 1e-10);
    assert!((kd - 0.05).abs() < 1e-10);
    assert!((max_v - 1.5).abs() < 1e-10);
    assert_eq!(lidar_hz, 10);
    assert_eq!(cam_res, "640x480");

    // Operator tunes kp at runtime
    params.set("pid.kp", 3.5f64).unwrap();
    let new_kp: f64 = params.get("pid.kp").unwrap();
    assert!((new_kp - 3.5).abs() < 1e-10, "kp should be updated to 3.5");

    println!("✓ params_robot_workflow — PID tuning, speed limits, sensor config all working");
}
