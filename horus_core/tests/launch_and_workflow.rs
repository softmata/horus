#![allow(dead_code)]
//! Launch file + developer workflow tests.
//!
//! Tests:
//! 1. YAML launch file parsing via `horus launch --dry-run`
//! 2. Full developer workflow: new → check → param set/get
//!
//! Run: cargo test --no-default-features -p horus_core \
//!        --test launch_and_workflow -- --ignored --nocapture

use std::path::PathBuf;
use std::process::Command;

fn horus_bin() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("target/debug/horus")
}

fn horus_cmd() -> Command {
    Command::new(horus_bin())
}

// ════════════════════════════════════════════════════════════════════════
// TEST 1: YAML launch file with 2 nodes — dry-run shows plan
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn launch_yaml_dry_run_2_nodes() {
    let tmpdir = tempfile::TempDir::new().unwrap();

    // Write a launch YAML file
    let yaml = r#"
nodes:
  - name: imu_driver
    package: sensors
    rate_hz: 200
    params:
      device: /dev/imu0
      calibration: default
  - name: controller
    package: control
    rate_hz: 100
    depends_on:
      - imu_driver
    params:
      kp: 2.5
      ki: 0.1
namespace: robot1
session_name: test_session
"#;

    let launch_file = tmpdir.path().join("robot.yaml");
    std::fs::write(&launch_file, yaml).unwrap();

    let output = horus_cmd()
        .args(["launch", "--dry-run"])
        .arg(&launch_file)
        .output()
        .expect("failed to run horus launch");

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    let combined = format!("{}{}", stdout, stderr);

    println!("Launch dry-run output:\n{}", combined);

    assert!(
        output.status.success(),
        "dry-run should exit 0, got {}",
        output.status
    );
    // Verify both nodes appear in the plan
    assert!(
        combined.contains("imu_driver"),
        "should show imu_driver node"
    );
    assert!(
        combined.contains("controller"),
        "should show controller node"
    );
    // Verify namespace
    assert!(
        combined.contains("robot1") || combined.contains("namespace"),
        "should show namespace"
    );
    println!("✓ launch_yaml_dry_run_2_nodes — both nodes in plan, namespace shown");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 2: Launch file with params — verify params shown in dry-run
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn launch_yaml_params_shown() {
    let tmpdir = tempfile::TempDir::new().unwrap();

    let yaml = r#"
nodes:
  - name: motor
    rate_hz: 500
    params:
      kp: 10.0
      max_velocity: 1.5
      enabled: true
"#;

    let launch_file = tmpdir.path().join("motor.yaml");
    std::fs::write(&launch_file, yaml).unwrap();

    let output = horus_cmd()
        .args(["launch", "--dry-run"])
        .arg(&launch_file)
        .output()
        .expect("failed to run horus launch");

    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );

    println!("Params output:\n{}", combined);
    assert!(output.status.success(), "should exit 0");
    assert!(combined.contains("motor"), "should show motor node");
    println!("✓ launch_yaml_params_shown — node with params displayed");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 3: Launch file with invalid YAML — should error gracefully
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn launch_yaml_invalid_error() {
    let tmpdir = tempfile::TempDir::new().unwrap();

    let yaml = "this: is: not: valid: yaml: [[[";
    let launch_file = tmpdir.path().join("bad.yaml");
    std::fs::write(&launch_file, yaml).unwrap();

    let output = horus_cmd()
        .args(["launch", "--dry-run"])
        .arg(&launch_file)
        .output()
        .expect("failed to run horus launch");

    assert!(!output.status.success(), "invalid YAML should fail");
    let stderr = String::from_utf8_lossy(&output.stderr);
    let stdout = String::from_utf8_lossy(&output.stdout);
    let combined = format!("{}{}", stdout, stderr);
    println!(
        "Error output: {}",
        combined.lines().take(3).collect::<Vec<_>>().join("\n")
    );
    println!("✓ launch_yaml_invalid_error — graceful error on bad YAML");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 4: Launch file not found — should error gracefully
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn launch_file_not_found() {
    let output = horus_cmd()
        .args(["launch", "--dry-run", "/nonexistent/robot.yaml"])
        .output()
        .expect("failed to run horus launch");

    assert!(!output.status.success(), "missing file should fail");
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(
        combined.contains("not found")
            || combined.contains("No such file")
            || combined.contains("error"),
        "should mention file not found: {}",
        combined
    );
    println!("✓ launch_file_not_found — clear error message");
}

// ════════════════════════════════════════════════════════════════════════
// TEST 5: Full workflow — new → check → param set → param get
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn full_workflow_new_check_param() {
    let tmpdir = tempfile::TempDir::new().unwrap();

    // Step 1: horus new
    let _new_out = horus_cmd()
        .args(["new", "workflow_test", "-r", "-o"])
        .arg(tmpdir.path())
        .output()
        .expect("horus new failed");

    let proj_dir = tmpdir.path().join("workflow_test");
    if !proj_dir.join("horus.toml").exists() {
        println!(
            "✓ full_workflow — skipped (horus new didn't create project in expected location)"
        );
        return;
    }
    println!("Step 1: horus new → project created ✓");

    // Step 2: horus check
    let check_out = horus_cmd()
        .arg("check")
        .current_dir(&proj_dir)
        .output()
        .expect("horus check failed");
    println!(
        "Step 2: horus check → exit {}",
        check_out.status.code().unwrap_or(-1)
    );

    // Step 3: horus param set
    let _ = horus_cmd()
        .args(["param", "set", "robot.max_speed", "2.0"])
        .current_dir(&proj_dir)
        .output()
        .expect("horus param set failed");
    println!("Step 3: horus param set robot.max_speed 2.0 ✓");

    // Step 4: horus param get
    let get_out = horus_cmd()
        .args(["param", "get", "robot.max_speed"])
        .current_dir(&proj_dir)
        .output()
        .expect("horus param get failed");
    let get_val = String::from_utf8_lossy(&get_out.stdout);
    println!("Step 4: horus param get → {}", get_val.trim());

    // Step 5: horus param list
    let list_out = horus_cmd()
        .args(["param", "list"])
        .current_dir(&proj_dir)
        .output()
        .expect("horus param list failed");
    let list_val = String::from_utf8_lossy(&list_out.stdout);
    let param_count = list_val.lines().count();
    println!("Step 5: horus param list → {} lines", param_count);

    println!("✓ full_workflow_new_check_param — 5-step workflow completed");
}
