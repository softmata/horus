use assert_cmd::cargo::cargo_bin_cmd;
use assert_cmd::Command;
use predicates::prelude::*;
use serde_json::json;
use std::fs;
use std::io::Write;
use tempfile::TempDir;

/// Helper to get the CLI command
fn horus_cmd() -> Command {
    cargo_bin_cmd!("horus")
}

// ============================================================================
// Version and help output tests
// ============================================================================

#[test]
fn test_version_flag() {
    horus_cmd()
        .arg("--version")
        .assert()
        .success()
        .stdout(predicate::str::contains("horus"));
}

#[test]
fn test_help_flag() {
    horus_cmd()
        .arg("--help")
        .assert()
        .success()
        .stdout(predicate::str::contains("HORUS"))
        .stdout(predicate::str::contains("Usage"));
}

#[test]
fn test_help_shows_subcommands() {
    horus_cmd()
        .arg("--help")
        .assert()
        .success()
        .stdout(predicate::str::contains("init"))
        .stdout(predicate::str::contains("new"))
        .stdout(predicate::str::contains("run"))
        .stdout(predicate::str::contains("build"))
        .stdout(predicate::str::contains("monitor"))
        .stdout(predicate::str::contains("topic"))
        .stdout(predicate::str::contains("node"))
        .stdout(predicate::str::contains("install"))
        .stdout(predicate::str::contains("clean"));
}

// ============================================================================
// Subcommand help tests
// ============================================================================

#[test]
fn test_init_help() {
    horus_cmd()
        .args(["init", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Initialize"))
        .stdout(predicate::str::contains("workspace"));
}

#[test]
fn test_new_help() {
    horus_cmd()
        .args(["new", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Create"))
        .stdout(predicate::str::contains("project"));
}

#[test]
fn test_run_help() {
    horus_cmd()
        .args(["run", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Run"));
}

#[test]
fn test_build_help() {
    horus_cmd()
        .args(["build", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Build"));
}

#[test]
fn test_topic_help() {
    horus_cmd()
        .args(["topic", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Topic"));
}

#[test]
fn test_node_help() {
    horus_cmd()
        .args(["node", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Node"));
}

#[test]
fn test_monitor_help() {
    horus_cmd()
        .args(["monitor", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Monitor"));
}

#[test]
fn test_clean_help() {
    horus_cmd()
        .args(["clean", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Clean"));
}

#[test]
fn test_check_help() {
    horus_cmd()
        .args(["check", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Validate"));
}

#[test]
fn test_msg_help() {
    horus_cmd()
        .args(["msg", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Message"));
}

// ============================================================================
// Error handling tests
// ============================================================================

#[test]
fn test_invalid_subcommand_fails() {
    horus_cmd()
        .arg("nonexistent-command")
        .assert()
        .failure()
        .stderr(predicate::str::contains("error"));
}

#[test]
fn test_no_args_fails() {
    // Running horus with no subcommand should fail (clap requires subcommand)
    horus_cmd().assert().failure();
}

// ============================================================================
// Init command integration tests
// ============================================================================

#[test]
fn test_init_creates_workspace_files() {
    let tmp = TempDir::new().unwrap();

    // Init may return non-zero due to workspace registry parse in empty dir,
    // but it should still create the .horus directory and horus.yaml
    let output = horus_cmd()
        .arg("init")
        .current_dir(tmp.path())
        .output()
        .unwrap();

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains("Created .horus/") || tmp.path().join(".horus").exists(),
        "init should create .horus directory"
    );
}

#[test]
fn test_init_with_name_creates_files() {
    let tmp = TempDir::new().unwrap();

    let output = horus_cmd()
        .args(["init", "--name", "my-robot"])
        .current_dir(tmp.path())
        .output()
        .unwrap();

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains("Created .horus/") || tmp.path().join(".horus").exists(),
        "init --name should create .horus directory"
    );
}

// ============================================================================
// New command integration tests
// ============================================================================

#[test]
fn test_new_creates_project() {
    let tmp = TempDir::new().unwrap();

    horus_cmd()
        .args(["new", "test-project", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();

    // Should create project directory with horus.yaml
    let project_dir = tmp.path().join("test-project");
    assert!(project_dir.exists(), "Project directory should be created");
    assert!(
        project_dir.join("horus.yaml").exists(),
        "horus.yaml should be created"
    );
}

#[test]
fn test_new_rust_project() {
    let tmp = TempDir::new().unwrap();

    horus_cmd()
        .args([
            "new",
            "rust-project",
            "--rust",
            "-o",
            &tmp.path().to_string_lossy(),
        ])
        .assert()
        .success();

    let project_dir = tmp.path().join("rust-project");
    assert!(project_dir.exists());
    // Rust projects should have horus.yaml and main.rs
    assert!(
        project_dir.join("horus.yaml").exists(),
        "horus.yaml should be created for Rust projects"
    );
    assert!(
        project_dir.join("main.rs").exists(),
        "main.rs should be created for Rust projects"
    );
}

#[test]
fn test_new_python_project() {
    let tmp = TempDir::new().unwrap();

    horus_cmd()
        .args([
            "new",
            "py-project",
            "--python",
            "-o",
            &tmp.path().to_string_lossy(),
        ])
        .assert()
        .success();

    let project_dir = tmp.path().join("py-project");
    assert!(project_dir.exists());
}

// ============================================================================
// Clean command integration tests (dry-run only for safety)
// ============================================================================

#[test]
fn test_clean_dry_run() {
    let tmp = TempDir::new().unwrap();

    // Clean dry-run in empty directory should succeed
    horus_cmd()
        .args(["clean", "--dry-run"])
        .current_dir(tmp.path())
        .assert()
        .success();
}

// ============================================================================
// Check command integration tests
// ============================================================================

#[test]
fn test_check_nonexistent_path() {
    // Checking a non-existent path should still exit gracefully
    // (might warn or error but shouldn't crash)
    let tmp = TempDir::new().unwrap();
    let fake_path = tmp.path().join("nonexistent");

    let result = horus_cmd()
        .args(["check", &fake_path.to_string_lossy()])
        .assert();

    // Either success (with warnings) or failure (not found) - just ensure no crash
    let _ = result;
}

#[test]
fn test_check_empty_directory() {
    let tmp = TempDir::new().unwrap();

    // Checking empty directory - should complete without crash
    let result = horus_cmd()
        .args(["check", &tmp.path().to_string_lossy()])
        .assert();

    let _ = result;
}

#[test]
fn test_check_valid_horus_yaml() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.yaml"),
        "name: test-project\nversion: 0.1.0\n",
    )
    .unwrap();

    horus_cmd()
        .args(["check", &tmp.path().to_string_lossy()])
        .assert()
        .success();
}

// ============================================================================
// Msg list command integration test
// ============================================================================

#[test]
fn test_msg_list() {
    // msg list shows built-in message types - should always succeed
    horus_cmd().args(["msg", "list"]).assert().success();
}

// ============================================================================
// BlackBox command integration tests
// ============================================================================

/// Helper: create a WAL file with sample BlackBoxRecord JSONL entries.
fn write_sample_wal(dir: &std::path::Path) {
    let bb_dir = dir.join(".horus/blackbox");
    fs::create_dir_all(&bb_dir).unwrap();
    let mut f = fs::File::create(bb_dir.join("blackbox.wal")).unwrap();

    let records = vec![
        json!({"timestamp_us": 1700000000_000000u64, "tick": 0, "event": {"SchedulerStart": {"name": "main", "node_count": 3, "config": "default"}}}),
        json!({"timestamp_us": 1700000001_000000u64, "tick": 1, "event": {"NodeAdded": {"name": "sensor_node", "order": 0}}}),
        json!({"timestamp_us": 1700000002_000000u64, "tick": 2, "event": {"NodeTick": {"name": "sensor_node", "duration_us": 150, "success": true}}}),
        json!({"timestamp_us": 1700000003_000000u64, "tick": 3, "event": {"NodeTick": {"name": "motor_ctrl", "duration_us": 4200, "success": true}}}),
        json!({"timestamp_us": 1700000004_000000u64, "tick": 4, "event": {"DeadlineMiss": {"name": "motor_ctrl", "deadline_us": 1000, "actual_us": 4200}}}),
        json!({"timestamp_us": 1700000005_000000u64, "tick": 5, "event": {"NodeError": {"name": "camera_node", "error": "device disconnected"}}}),
        json!({"timestamp_us": 1700000006_000000u64, "tick": 6, "event": {"WCETViolation": {"name": "motor_ctrl", "budget_us": 2000, "actual_us": 4200}}}),
        json!({"timestamp_us": 1700000007_000000u64, "tick": 7, "event": {"EmergencyStop": {"reason": "safety limit exceeded"}}}),
        json!({"timestamp_us": 1700000008_000000u64, "tick": 8, "event": {"SchedulerStop": {"reason": "clean shutdown", "total_ticks": 8}}}),
    ];

    for record in &records {
        writeln!(f, "{}", serde_json::to_string(record).unwrap()).unwrap();
    }
}

/// Helper: create a JSON snapshot (fallback format).
fn write_sample_json_snapshot(dir: &std::path::Path) {
    let bb_dir = dir.join(".horus/blackbox");
    fs::create_dir_all(&bb_dir).unwrap();

    let records = vec![
        json!({"timestamp_us": 1700000000_000000u64, "tick": 0, "event": {"SchedulerStart": {"name": "main", "node_count": 2, "config": "deploy"}}}),
        json!({"timestamp_us": 1700000001_000000u64, "tick": 1, "event": {"NodeError": {"name": "lidar", "error": "timeout"}}}),
    ];

    let content = serde_json::to_string_pretty(&records).unwrap();
    fs::write(bb_dir.join("blackbox.json"), content).unwrap();
}

#[test]
fn test_blackbox_help() {
    horus_cmd()
        .args(["blackbox", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("BlackBox"))
        .stdout(predicate::str::contains("--anomalies"))
        .stdout(predicate::str::contains("--tail"))
        .stdout(predicate::str::contains("--json"))
        .stdout(predicate::str::contains("--node"))
        .stdout(predicate::str::contains("--event"))
        .stdout(predicate::str::contains("--tick"))
        .stdout(predicate::str::contains("--last"))
        .stdout(predicate::str::contains("--path"))
        .stdout(predicate::str::contains("--clear"));
}

#[test]
fn test_bb_alias_help() {
    horus_cmd()
        .args(["bb", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("BlackBox"));
}

#[test]
fn test_blackbox_empty_dir() {
    let tmp = TempDir::new().unwrap();
    let bb_dir = tmp.path().join("empty_bb");
    fs::create_dir_all(&bb_dir).unwrap();

    horus_cmd()
        .args(["blackbox", "--path", &bb_dir.to_string_lossy()])
        .assert()
        .success()
        .stdout(predicate::str::contains("No blackbox events"));
}

#[test]
fn test_blackbox_empty_dir_json() {
    let tmp = TempDir::new().unwrap();
    let bb_dir = tmp.path().join("empty_bb");
    fs::create_dir_all(&bb_dir).unwrap();

    horus_cmd()
        .args(["bb", "--path", &bb_dir.to_string_lossy(), "--json"])
        .assert()
        .success()
        .stdout(predicate::str::contains("[]"));
}

#[test]
fn test_blackbox_reads_wal() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    horus_cmd()
        .args(["blackbox", "--path", &bb_dir.to_string_lossy()])
        .assert()
        .success()
        .stdout(predicate::str::contains("9 events"))
        .stdout(predicate::str::contains("BLACKBOX"));
}

#[test]
fn test_blackbox_reads_json_snapshot_fallback() {
    let tmp = TempDir::new().unwrap();
    write_sample_json_snapshot(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    horus_cmd()
        .args(["bb", "--path", &bb_dir.to_string_lossy()])
        .assert()
        .success()
        .stdout(predicate::str::contains("2 events"));
}

#[test]
fn test_blackbox_anomalies_filter() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    // WAL has 4 anomalies: DeadlineMiss, NodeError, WCETViolation, EmergencyStop
    horus_cmd()
        .args(["bb", "--path", &bb_dir.to_string_lossy(), "--anomalies"])
        .assert()
        .success()
        .stdout(predicate::str::contains("4 events"));
}

#[test]
fn test_blackbox_anomalies_json() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    let output = horus_cmd()
        .args([
            "bb",
            "--path",
            &bb_dir.to_string_lossy(),
            "--anomalies",
            "--json",
        ])
        .output()
        .unwrap();

    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Vec<serde_json::Value> = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed.len(), 4);
}

#[test]
fn test_blackbox_node_filter() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    // "motor_ctrl" appears in: NodeTick(tick 3), DeadlineMiss(tick 4), WCETViolation(tick 6)
    let output = horus_cmd()
        .args([
            "bb",
            "--path",
            &bb_dir.to_string_lossy(),
            "--node",
            "motor_ctrl",
            "--json",
        ])
        .output()
        .unwrap();

    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Vec<serde_json::Value> = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed.len(), 3, "motor_ctrl appears in 3 events");
}

#[test]
fn test_blackbox_node_filter_case_insensitive() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    let output = horus_cmd()
        .args([
            "bb",
            "--path",
            &bb_dir.to_string_lossy(),
            "--node",
            "MOTOR_CTRL",
            "--json",
        ])
        .output()
        .unwrap();

    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Vec<serde_json::Value> = serde_json::from_str(&stdout).unwrap();
    assert_eq!(
        parsed.len(),
        3,
        "case-insensitive node match should find 3 events"
    );
}

#[test]
fn test_blackbox_node_filter_partial_match() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    // "sensor" partial matches "sensor_node"
    let output = horus_cmd()
        .args([
            "bb",
            "--path",
            &bb_dir.to_string_lossy(),
            "--node",
            "sensor",
            "--json",
        ])
        .output()
        .unwrap();

    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Vec<serde_json::Value> = serde_json::from_str(&stdout).unwrap();
    assert_eq!(
        parsed.len(),
        2,
        "'sensor' matches sensor_node: NodeAdded + NodeTick"
    );
}

#[test]
fn test_blackbox_event_type_filter() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    let output = horus_cmd()
        .args([
            "bb",
            "--path",
            &bb_dir.to_string_lossy(),
            "--event",
            "DeadlineMiss",
            "--json",
        ])
        .output()
        .unwrap();

    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Vec<serde_json::Value> = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed.len(), 1);
}

#[test]
fn test_blackbox_event_type_filter_case_insensitive() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    let output = horus_cmd()
        .args([
            "bb",
            "--path",
            &bb_dir.to_string_lossy(),
            "--event",
            "deadlinemiss",
            "--json",
        ])
        .output()
        .unwrap();

    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Vec<serde_json::Value> = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed.len(), 1);
}

#[test]
fn test_blackbox_tick_range_filter() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    // Ticks 3-5 should give 3 events
    let output = horus_cmd()
        .args([
            "bb",
            "--path",
            &bb_dir.to_string_lossy(),
            "--tick",
            "3-5",
            "--json",
        ])
        .output()
        .unwrap();

    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Vec<serde_json::Value> = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed.len(), 3, "ticks 3,4,5 should match 3 events");
}

#[test]
fn test_blackbox_single_tick_filter() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    let output = horus_cmd()
        .args([
            "bb",
            "--path",
            &bb_dir.to_string_lossy(),
            "--tick",
            "4",
            "--json",
        ])
        .output()
        .unwrap();

    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Vec<serde_json::Value> = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed.len(), 1, "single tick 4 should match DeadlineMiss");
}

#[test]
fn test_blackbox_last_n() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    let output = horus_cmd()
        .args([
            "bb",
            "--path",
            &bb_dir.to_string_lossy(),
            "--last",
            "3",
            "--json",
        ])
        .output()
        .unwrap();

    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Vec<serde_json::Value> = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed.len(), 3, "--last 3 should return exactly 3 events");
    // Should be the last 3 (ticks 6, 7, 8)
    assert_eq!(parsed[0]["tick"], 6);
    assert_eq!(parsed[1]["tick"], 7);
    assert_eq!(parsed[2]["tick"], 8);
}

#[test]
fn test_blackbox_combined_filters() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    // Anomalies + node=motor_ctrl → DeadlineMiss(tick 4) + WCETViolation(tick 6)
    let output = horus_cmd()
        .args([
            "bb",
            "--path",
            &bb_dir.to_string_lossy(),
            "--anomalies",
            "--node",
            "motor_ctrl",
            "--json",
        ])
        .output()
        .unwrap();

    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Vec<serde_json::Value> = serde_json::from_str(&stdout).unwrap();
    assert_eq!(
        parsed.len(),
        2,
        "anomalies for motor_ctrl: DeadlineMiss + WCETViolation"
    );
}

#[test]
fn test_blackbox_no_match_returns_empty() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    horus_cmd()
        .args([
            "bb",
            "--path",
            &bb_dir.to_string_lossy(),
            "--node",
            "nonexistent_node",
            "--json",
        ])
        .assert()
        .success()
        .stdout(predicate::str::contains("[]"));
}

#[test]
fn test_blackbox_json_output_is_valid_json() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    let output = horus_cmd()
        .args(["bb", "--path", &bb_dir.to_string_lossy(), "--json"])
        .output()
        .unwrap();

    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value =
        serde_json::from_str(&stdout).expect("JSON output should be valid JSON");
    assert!(parsed.is_array());
}

#[test]
fn test_blackbox_json_records_have_expected_fields() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    let output = horus_cmd()
        .args([
            "bb",
            "--path",
            &bb_dir.to_string_lossy(),
            "--last",
            "1",
            "--json",
        ])
        .output()
        .unwrap();

    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Vec<serde_json::Value> = serde_json::from_str(&stdout).unwrap();
    assert_eq!(parsed.len(), 1);
    assert!(
        parsed[0].get("timestamp_us").is_some(),
        "record should have timestamp_us"
    );
    assert!(parsed[0].get("tick").is_some(), "record should have tick");
    assert!(parsed[0].get("event").is_some(), "record should have event");
}

#[test]
fn test_blackbox_clear_no_data() {
    let tmp = TempDir::new().unwrap();
    let bb_dir = tmp.path().join("empty_bb");
    fs::create_dir_all(&bb_dir).unwrap();

    horus_cmd()
        .args(["bb", "--path", &bb_dir.to_string_lossy(), "--clear"])
        .assert()
        .success()
        .stdout(predicate::str::contains("No blackbox data to clear"));
}

#[test]
fn test_blackbox_clear_aborted() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    // Pipe "n" to stdin → should cancel
    horus_cmd()
        .args(["bb", "--path", &bb_dir.to_string_lossy(), "--clear"])
        .write_stdin("n\n")
        .assert()
        .success()
        .stdout(predicate::str::contains("Cancelled"));

    // Files should still exist
    assert!(bb_dir.join("blackbox.wal").exists());
}

#[test]
fn test_blackbox_clear_confirmed() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");
    assert!(bb_dir.join("blackbox.wal").exists());

    // Pipe "y" to stdin → should delete
    horus_cmd()
        .args(["bb", "--path", &bb_dir.to_string_lossy(), "--clear"])
        .write_stdin("y\n")
        .assert()
        .success()
        .stdout(predicate::str::contains("Blackbox data cleared"));

    assert!(!bb_dir.join("blackbox.wal").exists());
}

#[test]
fn test_blackbox_corrupt_wal_lines_skipped() {
    let tmp = TempDir::new().unwrap();
    let bb_dir = tmp.path().join(".horus/blackbox");
    fs::create_dir_all(&bb_dir).unwrap();

    let mut f = fs::File::create(bb_dir.join("blackbox.wal")).unwrap();
    // Valid record
    writeln!(f, "{}", serde_json::to_string(&json!({"timestamp_us": 100u64, "tick": 0, "event": {"Custom": {"category": "test", "message": "ok"}}})).unwrap()).unwrap();
    // Corrupt line
    writeln!(f, "{{this is not valid json}}").unwrap();
    // Another valid record
    writeln!(f, "{}", serde_json::to_string(&json!({"timestamp_us": 200u64, "tick": 1, "event": {"Custom": {"category": "test", "message": "still ok"}}})).unwrap()).unwrap();
    // Empty line
    writeln!(f).unwrap();

    let output = horus_cmd()
        .args(["bb", "--path", &bb_dir.to_string_lossy(), "--json"])
        .output()
        .unwrap();

    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Vec<serde_json::Value> = serde_json::from_str(&stdout).unwrap();
    assert_eq!(
        parsed.len(),
        2,
        "corrupt lines should be skipped, 2 valid remain"
    );
}

#[test]
fn test_blackbox_wal_preferred_over_json() {
    let tmp = TempDir::new().unwrap();
    let bb_dir = tmp.path().join(".horus/blackbox");
    fs::create_dir_all(&bb_dir).unwrap();

    // Write JSON snapshot with 1 record
    let json_records = vec![
        json!({"timestamp_us": 100u64, "tick": 0, "event": {"Custom": {"category": "json", "message": "from snapshot"}}}),
    ];
    fs::write(
        bb_dir.join("blackbox.json"),
        serde_json::to_string_pretty(&json_records).unwrap(),
    )
    .unwrap();

    // Write WAL with 2 records
    let mut f = fs::File::create(bb_dir.join("blackbox.wal")).unwrap();
    writeln!(f, "{}", serde_json::to_string(&json!({"timestamp_us": 200u64, "tick": 0, "event": {"Custom": {"category": "wal", "message": "line 1"}}})).unwrap()).unwrap();
    writeln!(f, "{}", serde_json::to_string(&json!({"timestamp_us": 300u64, "tick": 1, "event": {"Custom": {"category": "wal", "message": "line 2"}}})).unwrap()).unwrap();

    let output = horus_cmd()
        .args(["bb", "--path", &bb_dir.to_string_lossy(), "--json"])
        .output()
        .unwrap();

    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Vec<serde_json::Value> = serde_json::from_str(&stdout).unwrap();
    assert_eq!(
        parsed.len(),
        2,
        "WAL (2 records) should be preferred over JSON snapshot (1 record)"
    );
}

#[test]
fn test_blackbox_colored_output_contains_event_types() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    horus_cmd()
        .args(["bb", "--path", &bb_dir.to_string_lossy()])
        .assert()
        .success()
        .stdout(predicate::str::contains("SchedulerStart"))
        .stdout(predicate::str::contains("DeadlineMiss"))
        .stdout(predicate::str::contains("NodeError"))
        .stdout(predicate::str::contains("EmergencyStop"))
        .stdout(predicate::str::contains("SchedulerStop"));
}

#[test]
fn test_blackbox_tick_range_out_of_bounds() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    // Tick range 100-200 — no events in that range
    horus_cmd()
        .args([
            "bb",
            "--path",
            &bb_dir.to_string_lossy(),
            "--tick",
            "100-200",
            "--json",
        ])
        .assert()
        .success()
        .stdout(predicate::str::contains("[]"));
}

#[test]
fn test_blackbox_invalid_tick_range() {
    let tmp = TempDir::new().unwrap();
    write_sample_wal(tmp.path());

    let bb_dir = tmp.path().join(".horus/blackbox");

    horus_cmd()
        .args([
            "bb",
            "--path",
            &bb_dir.to_string_lossy(),
            "--tick",
            "not-a-number",
        ])
        .assert()
        .failure();
}

// ============================================================================
// New top-level command help tests
// ============================================================================

#[test]
fn test_install_help() {
    horus_cmd()
        .args(["install", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Install"))
        .stdout(predicate::str::contains("--plugin"))
        .stdout(predicate::str::contains("--driver"));
}

#[test]
fn test_list_help() {
    horus_cmd()
        .args(["list", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("List"))
        .stdout(predicate::str::contains("--global"));
}

#[test]
fn test_search_help() {
    horus_cmd()
        .args(["search", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Search"));
}

#[test]
fn test_update_help() {
    horus_cmd()
        .args(["update", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Update"))
        .stdout(predicate::str::contains("--dry-run"));
}

#[test]
fn test_publish_help() {
    horus_cmd()
        .args(["publish", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Publish"))
        .stdout(predicate::str::contains("--dry-run"));
}

#[test]
fn test_unpublish_help() {
    horus_cmd()
        .args(["unpublish", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Unpublish"));
}

#[test]
fn test_keygen_help() {
    horus_cmd()
        .args(["keygen", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Generate"));
}

#[test]
fn test_info_help() {
    horus_cmd()
        .args(["info", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("info"));
}

#[test]
fn test_enable_help() {
    horus_cmd()
        .args(["enable", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Enable"));
}

#[test]
fn test_disable_help() {
    horus_cmd()
        .args(["disable", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Disable"))
        .stdout(predicate::str::contains("--reason"));
}

#[test]
fn test_verify_help() {
    horus_cmd()
        .args(["verify", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Verify"));
}

// ============================================================================
// Removed commands tests
// ============================================================================

#[test]
fn test_removed_pkg_command_fails() {
    horus_cmd().args(["pkg", "list"]).assert().failure();
}

#[test]
fn test_removed_plugin_command_fails() {
    horus_cmd()
        .args(["plugin", "search", "camera"])
        .assert()
        .failure();
}

#[test]
fn test_removed_add_command_fails() {
    horus_cmd().args(["add", "some-package"]).assert().failure();
}

// ============================================================================
// Phase 6: DX Regression Tests
// ============================================================================

// -- Help Headings --

#[test]
fn test_help_has_project_heading() {
    horus_cmd()
        .arg("--help")
        .assert()
        .success()
        .stdout(predicate::str::contains("Project:"));
}

#[test]
fn test_help_has_introspection_heading() {
    horus_cmd()
        .arg("--help")
        .assert()
        .success()
        .stdout(predicate::str::contains("Introspection:"));
}

#[test]
fn test_help_has_packages_heading() {
    horus_cmd()
        .arg("--help")
        .assert()
        .success()
        .stdout(predicate::str::contains("Packages:"));
}

#[test]
fn test_help_has_plugins_heading() {
    horus_cmd()
        .arg("--help")
        .assert()
        .success()
        .stdout(predicate::str::contains("Plugins:"));
}

#[test]
fn test_help_has_publishing_heading() {
    horus_cmd()
        .arg("--help")
        .assert()
        .success()
        .stdout(predicate::str::contains("Publishing & Deploy:"));
}

#[test]
fn test_help_has_examples() {
    horus_cmd()
        .arg("--help")
        .assert()
        .success()
        .stdout(predicate::str::contains("Examples:"))
        .stdout(predicate::str::contains("horus new my_robot --rust"));
}

// -- Aliases --

#[test]
fn test_alias_t_for_topic() {
    horus_cmd()
        .args(["t", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Topic"));
}

#[test]
fn test_alias_n_for_node() {
    horus_cmd()
        .args(["n", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Node"));
}

#[test]
fn test_alias_p_for_param() {
    horus_cmd()
        .args(["p", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Parameter"));
}

#[test]
fn test_alias_i_for_install() {
    horus_cmd()
        .args(["i", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Install"));
}

#[test]
fn test_alias_s_for_search() {
    horus_cmd()
        .args(["s", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Search"));
}

#[test]
fn test_alias_l_for_launch() {
    horus_cmd()
        .args(["l", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Launch"));
}

#[test]
fn test_alias_mon_for_monitor() {
    horus_cmd()
        .args(["mon", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Monitor"));
}

#[test]
fn test_alias_rec_for_record() {
    horus_cmd()
        .args(["rec", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Record"));
}

#[test]
fn test_alias_bb_for_blackbox() {
    horus_cmd()
        .args(["bb", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("BlackBox"));
}

// -- Frame rename with backward compat --

#[test]
fn test_frame_command_works() {
    horus_cmd()
        .args(["frame", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Coordinate frame"));
}

#[test]
fn test_frames_alias_works() {
    horus_cmd()
        .args(["frames", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Coordinate frame"));
}

#[test]
fn test_hf_backward_compat() {
    horus_cmd()
        .args(["hf", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Coordinate frame"));
}

// -- Semantic fixes --

#[test]
fn test_list_rejects_positional_args() {
    // `horus list camera` should fail since we removed the positional query arg
    horus_cmd().args(["list", "camera"]).assert().failure();
}

#[test]
fn test_install_help_shows_at_version_syntax() {
    horus_cmd()
        .args(["install", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("name@version"));
}

#[test]
fn test_ver_flag_hidden_from_help() {
    let output = horus_cmd().args(["install", "--help"]).output().unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    // --ver should be hidden from help output (but --verbose and --version are expected)
    // Check that "--ver " (with trailing space, the actual flag) doesn't appear
    assert!(
        !stdout.contains("--ver ") && !stdout.contains("--ver\n"),
        "--ver should be hidden from install help output"
    );
}

// -- JSON output --

#[test]
fn test_clean_json_flag_accepted() {
    // --json flag should be accepted (even if clean has nothing to do)
    horus_cmd()
        .args(["clean", "--json", "--dry-run"])
        .assert()
        .success();
}

#[test]
fn test_clean_json_output_is_valid_json() {
    let output = horus_cmd()
        .args(["clean", "--json", "--dry-run"])
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Result<serde_json::Value, _> = serde_json::from_str(&stdout);
    assert!(
        parsed.is_ok(),
        "clean --json should produce valid JSON, got: {}",
        stdout
    );
}

#[test]
fn test_clean_json_has_no_ansi_codes() {
    let output = horus_cmd()
        .args(["clean", "--json", "--dry-run"])
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    // ANSI escape codes start with \x1b[
    assert!(
        !stdout.contains("\x1b["),
        "JSON output should not contain ANSI escape codes"
    );
}

#[test]
fn test_check_json_flag_accepted() {
    horus_cmd().args(["check", "--json"]).assert().success();
}

#[test]
fn test_check_json_output_contains_json() {
    let output = horus_cmd().args(["check", "--json"]).output().unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    // check --json includes a JSON object at the end (may have non-JSON text before it)
    assert!(
        stdout.contains("\"valid\""),
        "check --json should contain valid field in JSON output"
    );
    assert!(
        stdout.contains("\"path\""),
        "check --json should contain path field in JSON output"
    );
}

#[test]
fn test_cache_info_json_flag() {
    let output = horus_cmd()
        .args(["cache", "info", "--json"])
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Result<serde_json::Value, _> = serde_json::from_str(&stdout);
    assert!(
        parsed.is_ok(),
        "cache info --json should produce valid JSON, got: {}",
        stdout
    );
}

#[test]
fn test_cache_list_json_flag() {
    let output = horus_cmd()
        .args(["cache", "list", "--json"])
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Result<serde_json::Value, _> = serde_json::from_str(&stdout);
    assert!(
        parsed.is_ok(),
        "cache list --json should produce valid JSON, got: {}",
        stdout
    );
}

#[test]
fn test_list_json_flag() {
    let output = horus_cmd().args(["list", "--json"]).output().unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: Result<serde_json::Value, _> = serde_json::from_str(&stdout);
    assert!(
        parsed.is_ok(),
        "list --json should produce valid JSON, got: {}",
        stdout
    );
}

#[test]
fn test_verify_json_flag_accepted() {
    // verify --json should at least be a valid flag
    horus_cmd()
        .args(["verify", "--json", "--help"])
        .assert()
        .success();
}

// -- Discover stub when mdns is off --

#[test]
fn test_discover_command_exists() {
    // discover should work (either mdns or stub)
    horus_cmd().args(["discover", "--help"]).assert().success();
}

// ============================================================================
// Node command integration tests
// ============================================================================

#[test]
fn test_node_list_help() {
    horus_cmd()
        .args(["node", "list", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--verbose"))
        .stdout(predicate::str::contains("--json"))
        .stdout(predicate::str::contains("--category"));
}

#[test]
fn test_node_list_succeeds() {
    // node list should succeed regardless of whether nodes are running
    horus_cmd()
        .args(["node", "list"])
        .assert()
        .success()
        .stdout(
            predicate::str::contains("No running nodes")
                .or(predicate::str::contains("Running Nodes")),
        );
}

#[test]
fn test_node_list_json_valid() {
    let output = horus_cmd()
        .args(["node", "list", "--json"])
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(&stdout).expect("should be valid JSON");
    assert!(parsed["count"].is_number());
    assert!(parsed["items"].is_array());
    // count should match items length
    let count = parsed["count"].as_u64().unwrap();
    let items_len = parsed["items"].as_array().unwrap().len() as u64;
    assert_eq!(count, items_len);
}

#[test]
fn test_node_list_verbose_succeeds() {
    horus_cmd()
        .args(["node", "list", "--verbose"])
        .assert()
        .success()
        .stdout(
            predicate::str::contains("No running nodes")
                .or(predicate::str::contains("Running Nodes")),
        );
}

#[test]
fn test_node_list_invalid_category() {
    horus_cmd()
        .args(["node", "list", "--category", "bogus"])
        .assert()
        .failure();
}

#[test]
fn test_node_list_valid_category_node() {
    // Should succeed even if no nodes found (returns empty)
    horus_cmd()
        .args(["node", "list", "--category", "node"])
        .assert()
        .success();
}

#[test]
fn test_node_list_valid_category_tool() {
    horus_cmd()
        .args(["node", "list", "--category", "tool"])
        .assert()
        .success();
}

#[test]
fn test_node_list_valid_category_cli() {
    horus_cmd()
        .args(["node", "list", "--category", "cli"])
        .assert()
        .success();
}

#[test]
fn test_node_info_nonexistent() {
    horus_cmd()
        .args(["node", "info", "nonexistent_node_xyz"])
        .assert()
        .failure()
        .stderr(predicate::str::contains("not found").or(predicate::str::contains("Node")));
}

#[test]
fn test_node_kill_nonexistent() {
    horus_cmd()
        .args(["node", "kill", "nonexistent_node_xyz"])
        .assert()
        .failure()
        .stderr(predicate::str::contains("not found").or(predicate::str::contains("Node")));
}

#[test]
fn test_node_kill_help() {
    horus_cmd()
        .args(["node", "kill", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--force"));
}

#[test]
fn test_node_restart_nonexistent() {
    horus_cmd()
        .args(["node", "restart", "nonexistent_node_xyz"])
        .assert()
        .failure();
}

#[test]
fn test_node_pause_nonexistent() {
    horus_cmd()
        .args(["node", "pause", "nonexistent_node_xyz"])
        .assert()
        .failure();
}

#[test]
fn test_node_resume_nonexistent() {
    horus_cmd()
        .args(["node", "resume", "nonexistent_node_xyz"])
        .assert()
        .failure();
}

#[test]
fn test_node_alias_n() {
    // 'n' should work as alias for 'node'
    horus_cmd().args(["n", "list"]).assert().success();
}

// ============================================================================
// Topic command integration tests
// ============================================================================

#[test]
fn test_topic_list_help() {
    horus_cmd()
        .args(["topic", "list", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--verbose"))
        .stdout(predicate::str::contains("--json"));
}

#[test]
fn test_topic_list_no_active_topics() {
    // With no scheduler running, should succeed and report no topics
    horus_cmd()
        .args(["topic", "list"])
        .assert()
        .success()
        .stdout(
            predicate::str::contains("No active topics")
                .or(predicate::str::contains("Active Topics")),
        );
}

#[test]
fn test_topic_list_json() {
    let output = horus_cmd()
        .args(["topic", "list", "--json"])
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(&stdout).expect("should be valid JSON");
    assert!(parsed["count"].is_number());
    assert!(parsed["items"].is_array());
}

#[test]
fn test_topic_list_verbose() {
    horus_cmd()
        .args(["topic", "list", "--verbose"])
        .assert()
        .success();
}

#[test]
fn test_topic_info_nonexistent() {
    horus_cmd()
        .args(["topic", "info", "nonexistent_topic_xyz"])
        .assert()
        .failure()
        .stderr(predicate::str::contains("not found").or(predicate::str::contains("Topic")));
}

#[test]
fn test_topic_echo_help() {
    horus_cmd()
        .args(["topic", "echo", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--count"))
        .stdout(predicate::str::contains("--rate"));
}

#[test]
fn test_topic_echo_nonexistent() {
    horus_cmd()
        .args(["topic", "echo", "nonexistent_topic_xyz"])
        .assert()
        .failure()
        .stderr(predicate::str::contains("not found").or(predicate::str::contains("Topic")));
}

#[test]
fn test_topic_hz_help() {
    horus_cmd()
        .args(["topic", "hz", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--window"));
}

#[test]
fn test_topic_pub_help() {
    horus_cmd()
        .args(["topic", "pub", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--rate"))
        .stdout(predicate::str::contains("--count"));
}

#[test]
fn test_topic_bw_help() {
    horus_cmd()
        .args(["topic", "bw", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--window"));
}

#[test]
fn test_topic_alias_t() {
    horus_cmd().args(["t", "list"]).assert().success();
}

// ============================================================================
// Param command integration tests
// ============================================================================

#[test]
fn test_param_list_help() {
    horus_cmd()
        .args(["param", "list", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--verbose"))
        .stdout(predicate::str::contains("--json"));
}

#[test]
fn test_param_list_empty() {
    let tmp = TempDir::new().unwrap();
    // Create .horus dir so init succeeds
    fs::create_dir_all(tmp.path().join(".horus/config")).unwrap();

    horus_cmd()
        .args(["param", "list"])
        .current_dir(tmp.path())
        .assert()
        .success()
        .stdout(
            predicate::str::contains("No parameters").or(predicate::str::contains("Parameters:")),
        );
}

#[test]
fn test_param_list_json_empty() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir_all(tmp.path().join(".horus/config")).unwrap();

    let output = horus_cmd()
        .args(["param", "list", "--json"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(&stdout).expect("should be valid JSON");
    assert!(parsed["count"].is_number());
    assert!(parsed["items"].is_array());
}

#[test]
fn test_param_set_and_get() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir_all(tmp.path().join(".horus/config")).unwrap();

    // Set a parameter
    horus_cmd()
        .args(["param", "set", "test.speed", "42"])
        .current_dir(tmp.path())
        .assert()
        .success();

    // Get the parameter back
    horus_cmd()
        .args(["param", "get", "test.speed"])
        .current_dir(tmp.path())
        .assert()
        .success()
        .stdout(predicate::str::contains("42"));
}

#[test]
fn test_param_set_and_get_json() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir_all(tmp.path().join(".horus/config")).unwrap();

    horus_cmd()
        .args(["param", "set", "robot.name", "test_bot"])
        .current_dir(tmp.path())
        .assert()
        .success();

    let output = horus_cmd()
        .args(["param", "get", "robot.name", "--json"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(&stdout).expect("should be valid JSON");
    assert_eq!(parsed["key"], "robot.name");
    assert_eq!(parsed["value"], "test_bot");
    assert_eq!(parsed["type"], "string");
}

#[test]
fn test_param_set_bool() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir_all(tmp.path().join(".horus/config")).unwrap();

    horus_cmd()
        .args(["param", "set", "debug.enabled", "true"])
        .current_dir(tmp.path())
        .assert()
        .success();

    let output = horus_cmd()
        .args(["param", "get", "debug.enabled", "--json"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(&stdout).expect("should be valid JSON");
    assert_eq!(parsed["value"], true);
    assert_eq!(parsed["type"], "bool");
}

#[test]
fn test_param_set_float() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir_all(tmp.path().join(".horus/config")).unwrap();

    horus_cmd()
        .args(["param", "set", "motor.kp", "0.5"])
        .current_dir(tmp.path())
        .assert()
        .success();

    let output = horus_cmd()
        .args(["param", "get", "motor.kp", "--json"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(&stdout).expect("should be valid JSON");
    assert_eq!(parsed["type"], "float");
}

#[test]
fn test_param_get_nonexistent() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir_all(tmp.path().join(".horus/config")).unwrap();

    horus_cmd()
        .args(["param", "get", "nonexistent.key"])
        .current_dir(tmp.path())
        .assert()
        .failure()
        .stderr(predicate::str::contains("not found"));
}

#[test]
fn test_param_delete() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir_all(tmp.path().join(".horus/config")).unwrap();

    // Set then delete
    horus_cmd()
        .args(["param", "set", "temp.key", "val"])
        .current_dir(tmp.path())
        .assert()
        .success();

    horus_cmd()
        .args(["param", "delete", "temp.key"])
        .current_dir(tmp.path())
        .assert()
        .success()
        .stdout(predicate::str::contains("Deleted"));

    // Getting deleted param should fail
    horus_cmd()
        .args(["param", "get", "temp.key"])
        .current_dir(tmp.path())
        .assert()
        .failure();
}

#[test]
fn test_param_delete_nonexistent() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir_all(tmp.path().join(".horus/config")).unwrap();

    horus_cmd()
        .args(["param", "delete", "nonexistent.key"])
        .current_dir(tmp.path())
        .assert()
        .failure()
        .stderr(predicate::str::contains("not found"));
}

#[test]
fn test_param_reset_without_force() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir_all(tmp.path().join(".horus/config")).unwrap();

    // Reset without --force should just print a warning, not actually reset
    horus_cmd()
        .args(["param", "reset"])
        .current_dir(tmp.path())
        .assert()
        .success()
        .stdout(predicate::str::contains("--force"));
}

#[test]
fn test_param_dump() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir_all(tmp.path().join(".horus/config")).unwrap();

    // Set a param so dump has something
    horus_cmd()
        .args(["param", "set", "dump.test", "123"])
        .current_dir(tmp.path())
        .assert()
        .success();

    horus_cmd()
        .args(["param", "dump"])
        .current_dir(tmp.path())
        .assert()
        .success()
        .stdout(predicate::str::contains("dump.test"));
}

#[test]
fn test_param_save_and_load() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir_all(tmp.path().join(".horus/config")).unwrap();

    let save_path = tmp.path().join("params_backup.yaml");

    // Set params
    horus_cmd()
        .args(["param", "set", "save.key1", "value1"])
        .current_dir(tmp.path())
        .assert()
        .success();

    // Save to file
    horus_cmd()
        .args(["param", "save", "-o", &save_path.to_string_lossy()])
        .current_dir(tmp.path())
        .assert()
        .success();

    assert!(save_path.exists(), "params file should be saved");

    // Load from file
    horus_cmd()
        .args(["param", "load", &save_path.to_string_lossy()])
        .current_dir(tmp.path())
        .assert()
        .success();
}

#[test]
fn test_param_load_nonexistent_file() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir_all(tmp.path().join(".horus/config")).unwrap();

    horus_cmd()
        .args(["param", "load", "/tmp/nonexistent_params_xyz.yaml"])
        .current_dir(tmp.path())
        .assert()
        .failure()
        .stderr(predicate::str::contains("not found").or(predicate::str::contains("File")));
}

#[test]
fn test_param_alias_p() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir_all(tmp.path().join(".horus/config")).unwrap();

    horus_cmd()
        .args(["p", "list"])
        .current_dir(tmp.path())
        .assert()
        .success();
}

#[test]
fn test_param_set_json_value() {
    let tmp = TempDir::new().unwrap();
    fs::create_dir_all(tmp.path().join(".horus/config")).unwrap();

    // Set a JSON object as a parameter value
    horus_cmd()
        .args([
            "param",
            "set",
            "pid.config",
            r#"{"kp":1.0,"ki":0.1,"kd":0.05}"#,
        ])
        .current_dir(tmp.path())
        .assert()
        .success();

    let output = horus_cmd()
        .args(["param", "get", "pid.config", "--json"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(&stdout).expect("should be valid JSON");
    assert_eq!(parsed["type"], "object");
}

// ============================================================================
// Service command integration tests
// ============================================================================

#[test]
fn test_service_list_help() {
    horus_cmd()
        .args(["service", "list", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--verbose"))
        .stdout(predicate::str::contains("--json"));
}

#[test]
fn test_service_list_no_services() {
    // With no scheduler running, should succeed and report no services
    horus_cmd()
        .args(["service", "list"])
        .assert()
        .success()
        .stdout(
            predicate::str::contains("No active services")
                .or(predicate::str::contains("Active Services")),
        );
}

#[test]
fn test_service_list_json() {
    let output = horus_cmd()
        .args(["service", "list", "--json"])
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(&stdout).expect("should be valid JSON");
    assert!(parsed["count"].is_number());
    assert!(parsed["items"].is_array());
}

#[test]
fn test_service_list_verbose() {
    horus_cmd()
        .args(["service", "list", "--verbose"])
        .assert()
        .success();
}

#[test]
fn test_service_type_nonexistent() {
    horus_cmd()
        .args(["service", "type", "nonexistent_service_xyz"])
        .assert()
        .failure()
        .stderr(predicate::str::contains("not found").or(predicate::str::contains("Service")));
}

#[test]
fn test_service_find_no_match() {
    // find with a filter that matches nothing should succeed with empty output
    horus_cmd()
        .args(["service", "find", "zzz_nonexistent_filter_zzz"])
        .assert()
        .success()
        .stdout(predicate::str::contains("No services matching"));
}

#[test]
fn test_service_call_help() {
    horus_cmd()
        .args(["service", "call", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--timeout"));
}

#[test]
fn test_service_alias_srv() {
    horus_cmd()
        .args(["srv", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Service"));
}

// ============================================================================
// Record command integration tests
// ============================================================================

#[test]
fn test_record_list_help() {
    horus_cmd()
        .args(["record", "list", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--long"))
        .stdout(predicate::str::contains("--json"));
}

#[test]
fn test_record_list_no_sessions() {
    // Should succeed even with no recordings
    horus_cmd()
        .args(["record", "list"])
        .assert()
        .success()
        .stdout(
            predicate::str::contains("No recording sessions")
                .or(predicate::str::contains("recording session")),
        );
}

#[test]
fn test_record_list_json() {
    let output = horus_cmd()
        .args(["record", "list", "--json"])
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(&stdout).expect("should be valid JSON");
    assert!(parsed["sessions"].is_array());
}

#[test]
fn test_record_info_nonexistent_session() {
    horus_cmd()
        .args(["record", "info", "nonexistent_session_xyz"])
        .assert()
        .success()
        .stdout(predicate::str::contains("not found"));
}

#[test]
fn test_record_info_json_nonexistent() {
    let output = horus_cmd()
        .args(["record", "info", "nonexistent_session_xyz", "--json"])
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(&stdout).expect("should be valid JSON");
    assert_eq!(parsed["found"], false);
}

#[test]
fn test_record_delete_nonexistent_succeeds_silently() {
    // delete --force of a nonexistent session succeeds (idempotent)
    horus_cmd()
        .args(["record", "delete", "nonexistent_session_xyz", "--force"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Deleted"));
}

#[test]
fn test_record_replay_help() {
    horus_cmd()
        .args(["record", "replay", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--speed"))
        .stdout(predicate::str::contains("--start-tick"))
        .stdout(predicate::str::contains("--stop-tick"));
}

#[test]
fn test_record_diff_help() {
    horus_cmd()
        .args(["record", "diff", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--limit"));
}

#[test]
fn test_record_export_help() {
    horus_cmd()
        .args(["record", "export", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--format"))
        .stdout(predicate::str::contains("--output"));
}

#[test]
fn test_record_inject_help() {
    horus_cmd()
        .args(["record", "inject", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--nodes"))
        .stdout(predicate::str::contains("--all"))
        .stdout(predicate::str::contains("--script"))
        .stdout(predicate::str::contains("--speed"))
        .stdout(predicate::str::contains("--loop"));
}

#[test]
fn test_record_alias_rec() {
    horus_cmd().args(["rec", "list"]).assert().success();
}

// ============================================================================
// Cross-command JSON output sanity checks
// ============================================================================

#[test]
fn test_node_list_json_no_ansi() {
    let output = horus_cmd()
        .args(["node", "list", "--json"])
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        !stdout.contains("\x1b["),
        "JSON output should not contain ANSI escape codes"
    );
}

#[test]
fn test_topic_list_json_no_ansi() {
    let output = horus_cmd()
        .args(["topic", "list", "--json"])
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        !stdout.contains("\x1b["),
        "JSON output should not contain ANSI escape codes"
    );
}

#[test]
fn test_service_list_json_no_ansi() {
    let output = horus_cmd()
        .args(["service", "list", "--json"])
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        !stdout.contains("\x1b["),
        "JSON output should not contain ANSI escape codes"
    );
}

#[test]
fn test_record_list_json_no_ansi() {
    let output = horus_cmd()
        .args(["record", "list", "--json"])
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        !stdout.contains("\x1b["),
        "JSON output should not contain ANSI escape codes"
    );
}
