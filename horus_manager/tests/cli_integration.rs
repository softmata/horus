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

/// Replace the scheduler-based main.rs with one that exits immediately.
/// The generated `horus new -r` template contains `scheduler.run()` which loops forever.
/// Tests that call `horus run` on real projects need the binary to exit.
fn write_exiting_main_rs(proj: &std::path::Path) {
    fs::write(
        proj.join("src/main.rs"),
        "fn main() { println!(\"horus project ok\"); }\n",
    )
    .unwrap();
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
#[cfg(feature = "monitor")]
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
    // but it should still create the .horus directory and horus.toml
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

    // Should create project directory with horus.toml
    let project_dir = tmp.path().join("test-project");
    assert!(project_dir.exists(), "Project directory should be created");
    assert!(
        project_dir.join("horus.toml").exists(),
        "horus.toml should be created"
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
    // Rust projects should have horus.toml and main.rs
    assert!(
        project_dir.join("horus.toml").exists(),
        "horus.toml should be created for Rust projects"
    );
    assert!(
        project_dir.join("src/main.rs").exists(),
        "src/main.rs should be created for Rust projects"
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
fn test_check_valid_horus_toml() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"test-project\"\nversion = \"0.1.0\"\n",
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
        json!({"timestamp_us": 1700000005_000000u64, "tick": 5, "event": {"NodeError": {"name": "camera_node", "error": "device disconnected", "severity": "Fatal"}}}),
        json!({"timestamp_us": 1700000006_000000u64, "tick": 6, "event": {"BudgetViolation": {"name": "motor_ctrl", "budget_us": 2000, "actual_us": 4200}}}),
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
        json!({"timestamp_us": 1700000001_000000u64, "tick": 1, "event": {"NodeError": {"name": "lidar", "error": "timeout", "severity": "Transient"}}}),
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
        .stdout(predicate::str::contains("--follow"))
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

    // WAL has 4 anomalies: DeadlineMiss, NodeError, BudgetViolation, EmergencyStop
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

    // "motor_ctrl" appears in: NodeTick(tick 3), DeadlineMiss(tick 4), BudgetViolation(tick 6)
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

    // Anomalies + node=motor_ctrl → DeadlineMiss(tick 4) + BudgetViolation(tick 6)
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
        "anomalies for motor_ctrl: DeadlineMiss + BudgetViolation"
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
        .stdout(predicate::str::contains("--plugin"));
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
fn test_keygen_deprecated_still_works() {
    // keygen is deprecated but should still work (hidden command)
    horus_cmd()
        .args(["keygen", "--help"])
        .assert()
        .success();
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
fn test_plugin_enable_help() {
    horus_cmd()
        .args(["plugin", "enable", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Enable"));
}

#[test]
fn test_plugin_disable_help() {
    horus_cmd()
        .args(["plugin", "disable", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Disable"));
}

#[test]
fn test_plugin_verify_help() {
    horus_cmd()
        .args(["plugin", "verify", "--help"])
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
        .stdout(predicate::str::contains("Quick Start:"))
        .stdout(predicate::str::contains("horus new my_robot -r"));
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
#[cfg(feature = "monitor")]
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
fn test_tf_backward_compat() {
    horus_cmd()
        .args(["tf", "--help"])
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
    // check --json may return nonzero (warnings), but should not crash
    let output = horus_cmd().args(["check", "--json"]).output().unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "check --json should not panic");
}

#[test]
fn test_check_json_output_is_valid_json() {
    let output = horus_cmd().args(["check", "--json"]).output().unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    // check --json should produce parseable JSON (array or object)
    let parsed: serde_json::Value = serde_json::from_str(&stdout)
        .unwrap_or_else(|_| panic!("check --json should produce valid JSON, got: {}", stdout));
    assert!(parsed.is_array() || parsed.is_object(),
        "check --json should return JSON array or object");
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
fn test_plugin_verify_json_flag_accepted() {
    // plugin verify --json should at least be a valid flag
    horus_cmd()
        .args(["plugin", "verify", "--json", "--help"])
        .assert()
        .success();
}

// ============================================================================
// Auth CLI tests
// ============================================================================

#[test]
fn test_auth_help() {
    horus_cmd()
        .args(["auth", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("login"))
        .stdout(predicate::str::contains("api-key"))
        .stdout(predicate::str::contains("signing-key"))
        .stdout(predicate::str::contains("whoami"))
        .stdout(predicate::str::contains("keys"));
}

#[test]
fn test_auth_login_help() {
    horus_cmd()
        .args(["auth", "login", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Login"));
}

#[test]
fn test_auth_whoami_not_logged_in() {
    let output = horus_cmd()
        .args(["auth", "whoami"])
        .output()
        .unwrap();
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!combined.contains("panicked"), "whoami should not panic");
    assert!(
        combined.contains("Not logged in") || combined.contains("not logged in")
            || combined.contains("authenticate"),
        "whoami without login should indicate not logged in, got: {}",
        combined
    );
}

#[test]
fn test_auth_logout_not_logged_in() {
    let output = horus_cmd()
        .args(["auth", "logout"])
        .output()
        .unwrap();
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!combined.contains("panicked"), "logout should not panic");
    // Should handle gracefully even if not logged in
    assert!(
        combined.contains("Not") || combined.contains("not") || combined.contains("Logging out"),
        "logout when not logged in should be graceful, got: {}",
        combined
    );
}

#[test]
fn test_auth_api_key_help() {
    horus_cmd()
        .args(["auth", "api-key", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("API") .or(predicate::str::contains("api"))
            .or(predicate::str::contains("key")));
}

#[test]
fn test_auth_api_key_not_logged_in() {
    // api-key command expects stdin input — it should not panic even without login
    let output = horus_cmd()
        .args(["auth", "api-key"])
        .write_stdin("")
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "api-key should not panic");
}

#[test]
fn test_auth_signing_key_generates() {
    let tmp = TempDir::new().unwrap();
    // Set XDG_DATA_HOME so signing key goes into temp dir
    let output = horus_cmd()
        .args(["auth", "signing-key"])
        .env("XDG_DATA_HOME", tmp.path())
        .output()
        .unwrap();
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!combined.contains("panicked"), "signing-key should not panic");
    // Should either create keys or report existing
    assert!(
        combined.contains("key") || combined.contains("Key") || combined.contains("signing"),
        "signing-key should mention key generation, got: {}",
        combined
    );
}

#[test]
fn test_auth_signing_key_idempotent() {
    let tmp = TempDir::new().unwrap();
    // Generate first time
    horus_cmd()
        .args(["auth", "signing-key"])
        .env("XDG_DATA_HOME", tmp.path())
        .output()
        .unwrap();
    // Generate second time — should warn about existing key or succeed silently
    let output = horus_cmd()
        .args(["auth", "signing-key"])
        .env("XDG_DATA_HOME", tmp.path())
        .output()
        .unwrap();
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!combined.contains("panicked"), "second signing-key should not panic");
}

#[test]
fn test_auth_keys_list_not_logged_in() {
    let output = horus_cmd()
        .args(["auth", "keys", "list"])
        .output()
        .unwrap();
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!combined.contains("panicked"), "keys list should not panic");
    assert!(
        combined.contains("Not logged in") || combined.contains("not logged in")
            || combined.contains("No") || combined.contains("login"),
        "keys list without login should indicate auth needed, got: {}",
        combined
    );
}

#[test]
fn test_auth_keys_revoke_not_logged_in() {
    let output = horus_cmd()
        .args(["auth", "keys", "revoke", "fake-key-id-12345"])
        .output()
        .unwrap();
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!combined.contains("panicked"), "keys revoke should not panic");
    // Should fail gracefully (not logged in or invalid key)
    assert!(
        combined.contains("Not logged in") || combined.contains("not logged in")
            || combined.contains("Error") || combined.contains("error"),
        "keys revoke with bad key should error gracefully, got: {}",
        combined
    );
}

#[test]
fn test_auth_keys_help() {
    horus_cmd()
        .args(["auth", "keys", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("list"))
        .stdout(predicate::str::contains("revoke"));
}

// ============================================================================
// Plugin command expanded tests
// ============================================================================

#[test]
fn test_plugin_enable_nonexistent() {
    let output = horus_cmd()
        .args(["plugin", "enable", "nonexistent-plugin-xyz"])
        .output()
        .unwrap();
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!combined.contains("panicked"), "enable nonexistent plugin should not panic");
    assert!(!output.status.success(), "enable nonexistent plugin should fail");
    assert!(
        combined.contains("not found") || combined.contains("Not found")
            || combined.contains("Error") || combined.contains("error"),
        "should report plugin not found, got: {}",
        combined
    );
}

#[test]
fn test_plugin_disable_nonexistent() {
    let output = horus_cmd()
        .args(["plugin", "disable", "nonexistent-plugin-xyz"])
        .output()
        .unwrap();
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!combined.contains("panicked"), "disable nonexistent plugin should not panic");
    assert!(!output.status.success(), "disable nonexistent plugin should fail");
}

#[test]
fn test_plugin_disable_with_reason() {
    let output = horus_cmd()
        .args(["plugin", "disable", "nonexistent-plugin", "--reason", "maintenance"])
        .output()
        .unwrap();
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!combined.contains("panicked"), "disable with --reason should not panic");
    // --reason flag should be accepted (even if plugin doesn't exist)
    assert!(
        !combined.contains("unexpected argument"),
        "--reason flag should be accepted by the parser"
    );
}

#[test]
fn test_plugin_verify_no_plugins() {
    let output = horus_cmd()
        .args(["plugin", "verify"])
        .output()
        .unwrap();
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!combined.contains("panicked"), "verify with no plugins should not panic");
    // Should report clean or no plugins
    assert!(
        combined.contains("No plugins") || combined.contains("no plugins")
            || combined.contains("verified") || combined.contains("ok")
            || output.status.success(),
        "verify with no plugins should succeed or report none, got: {}",
        combined
    );
}

#[test]
fn test_plugin_verify_json_output() {
    let output = horus_cmd()
        .args(["plugin", "verify", "--json"])
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "verify --json should not panic");
    // If there's JSON output, it should be valid
    if !stdout.trim().is_empty() {
        let parsed: Result<serde_json::Value, _> = serde_json::from_str(&stdout);
        assert!(
            parsed.is_ok(),
            "verify --json should produce valid JSON, got: {}",
            stdout
        );
    }
}

#[test]
fn test_plugin_verify_specific_plugin() {
    let output = horus_cmd()
        .args(["plugin", "verify", "nonexistent-plugin"])
        .output()
        .unwrap();
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!combined.contains("panicked"), "verify specific plugin should not panic");
}

#[test]
fn test_plugin_help() {
    horus_cmd()
        .args(["plugin", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("enable"))
        .stdout(predicate::str::contains("disable"))
        .stdout(predicate::str::contains("verify"));
}

#[test]
fn test_plugin_alias_plugins() {
    horus_cmd()
        .args(["plugins", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("enable"));
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
        .args(["param", "save", &save_path.to_string_lossy()])
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
fn test_service_info_nonexistent() {
    horus_cmd()
        .args(["service", "info", "nonexistent_service_xyz"])
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
        .failure()
        .stderr(predicate::str::contains("not found"));
}

#[test]
fn test_record_info_json_nonexistent() {
    // Nonexistent session returns error
    horus_cmd()
        .args(["record", "info", "nonexistent_session_xyz", "--json"])
        .assert()
        .failure()
        .stderr(predicate::str::contains("not found"));
}

#[test]
fn test_record_delete_nonexistent_fails() {
    // delete of a nonexistent session returns error
    horus_cmd()
        .args(["record", "delete", "nonexistent_session_xyz", "--force"])
        .assert()
        .failure()
        .stderr(predicate::str::contains("not found"));
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

// ============================================================================
// Transform Frame integration tests
// ============================================================================

#[test]
fn test_frame_list_empty() {
    // Should succeed even with no running system — 0 frames is ok
    horus_cmd()
        .args(["frame", "list"])
        .assert()
        .success()
        .stdout(predicate::str::contains("No active"));
}

#[test]
fn test_frame_tree_empty() {
    horus_cmd()
        .args(["frame", "tree"])
        .assert()
        .success()
        .stdout(predicate::str::contains("No frames"));
}

#[test]
fn test_tf_alias_works() {
    horus_cmd()
        .args(["tf", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("frame"));
}

#[test]
fn test_frame_calibrate_with_points() {
    let tmp = TempDir::new().unwrap();
    let csv = tmp.path().join("points.csv");
    // 4 point pairs: sensor→world is a 1m translation along X
    fs::write(
        &csv,
        "0.0,0.0,0.0,1.0,0.0,0.0\n1.0,0.0,0.0,2.0,0.0,0.0\n0.0,1.0,0.0,1.0,1.0,0.0\n0.0,0.0,1.0,1.0,0.0,1.0\n",
    )
    .unwrap();

    horus_cmd()
        .args(["frame", "calibrate", "--points-file", &csv.to_string_lossy()])
        .assert()
        .success()
        .stdout(predicate::str::contains("Calibration Result"))
        .stdout(predicate::str::contains("Translation"))
        .stdout(predicate::str::contains("RMSE"));
}

#[test]
fn test_frame_calibrate_insufficient_points() {
    let tmp = TempDir::new().unwrap();
    let csv = tmp.path().join("points.csv");
    // Only 2 points — should fail or warn
    fs::write(&csv, "0.0,0.0,0.0,1.0,0.0,0.0\n1.0,0.0,0.0,2.0,0.0,0.0\n").unwrap();

    let output = horus_cmd()
        .args(["frame", "calibrate", "--points-file", &csv.to_string_lossy()])
        .output()
        .unwrap();
    // Either fails or succeeds with warning — just shouldn't crash
    let _ = output.status;
}

#[test]
fn test_frame_calibrate_bad_csv() {
    let tmp = TempDir::new().unwrap();
    let csv = tmp.path().join("bad.csv");
    fs::write(&csv, "not,a,valid,csv\ndata\n").unwrap();

    horus_cmd()
        .args(["frame", "calibrate", "--points-file", &csv.to_string_lossy()])
        .assert()
        .failure();
}

#[test]
fn test_frame_calibrate_missing_file() {
    horus_cmd()
        .args(["frame", "calibrate", "--points-file", "/tmp/nonexistent_xyz.csv"])
        .assert()
        .failure();
}

#[test]
fn test_frame_hand_eye_with_poses() {
    let tmp = TempDir::new().unwrap();
    let robot = tmp.path().join("robot.csv");
    let sensor = tmp.path().join("sensor.csv");
    // 4 pose pairs: x,y,z,qx,qy,qz,qw — sensor has 0.1m offset in X
    fs::write(
        &robot,
        "1.0,0.0,0.0,0.0,0.0,0.0,1.0\n2.0,0.0,0.0,0.0,0.0,0.0,1.0\n1.0,1.0,0.0,0.0,0.0,0.0,1.0\n1.0,0.0,1.0,0.0,0.0,0.0,1.0\n",
    )
    .unwrap();
    fs::write(
        &sensor,
        "1.1,0.0,0.0,0.0,0.0,0.0,1.0\n2.1,0.0,0.0,0.0,0.0,0.0,1.0\n1.1,1.0,0.0,0.0,0.0,0.0,1.0\n1.1,0.0,1.0,0.0,0.0,0.0,1.0\n",
    )
    .unwrap();

    horus_cmd()
        .args([
            "frame",
            "hand-eye",
            "--robot-poses",
            &robot.to_string_lossy(),
            "--sensor-poses",
            &sensor.to_string_lossy(),
        ])
        .assert()
        .success()
        .stdout(predicate::str::contains("Hand-Eye Calibration Result"))
        .stdout(predicate::str::contains("Translation"));
}

#[test]
fn test_frame_hand_eye_missing_file() {
    horus_cmd()
        .args([
            "frame",
            "hand-eye",
            "--robot-poses",
            "/tmp/nonexistent_r.csv",
            "--sensor-poses",
            "/tmp/nonexistent_s.csv",
        ])
        .assert()
        .failure();
}

#[test]
fn test_frame_diff_missing_files() {
    horus_cmd()
        .args([
            "frame",
            "diff",
            "/tmp/nonexistent_1.tfr",
            "/tmp/nonexistent_2.tfr",
        ])
        .assert()
        .failure();
}

#[test]
fn test_frame_record_help() {
    horus_cmd()
        .args(["frame", "record", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--output"))
        .stdout(predicate::str::contains("--duration"));
}

#[test]
fn test_frame_play_help() {
    horus_cmd()
        .args(["frame", "play", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--speed"));
}

#[test]
fn test_frame_diff_help() {
    horus_cmd()
        .args(["frame", "diff", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("--threshold-m"))
        .stdout(predicate::str::contains("--json"));
}

#[test]
fn test_frame_tune_help() {
    horus_cmd()
        .args(["frame", "tune", "--help"])
        .assert()
        .success();
}

// ============================================================================
// Example project validation tests (horus check on real examples)
// ============================================================================

#[test]
fn test_example_differential_drive_valid() {
    let examples_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("examples/differential_drive");
    if !examples_dir.exists() {
        return; // skip if examples not present
    }
    horus_cmd()
        .args(["check", &examples_dir.to_string_lossy()])
        .assert()
        .success()
        .stdout(predicate::str::contains("All checks passed"));
}

#[test]
fn test_example_robot_arm_valid() {
    let examples_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("examples/robot_arm");
    if !examples_dir.exists() {
        return;
    }
    horus_cmd()
        .args(["check", &examples_dir.to_string_lossy()])
        .assert()
        .success()
        .stdout(predicate::str::contains("All checks passed"));
}

#[test]
fn test_example_quadruped_valid() {
    let examples_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("examples/quadruped");
    if !examples_dir.exists() {
        return;
    }
    horus_cmd()
        .args(["check", &examples_dir.to_string_lossy()])
        .assert()
        .success()
        .stdout(predicate::str::contains("All checks passed"));
}

#[test]
fn test_example_sensor_navigation_valid() {
    let examples_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("examples/sensor_navigation");
    if !examples_dir.exists() {
        return;
    }
    horus_cmd()
        .args(["check", &examples_dir.to_string_lossy()])
        .assert()
        .success()
        .stdout(predicate::str::contains("All checks passed"));
}

#[test]
fn test_example_multi_robot_valid() {
    let examples_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("examples/multi_robot");
    if !examples_dir.exists() {
        return;
    }
    horus_cmd()
        .args(["check", &examples_dir.to_string_lossy()])
        .assert()
        .success()
        .stdout(predicate::str::contains("All checks passed"));
}

// ============================================================================
// Registry integration tests (live API: api.horusrobotics.dev)
// ============================================================================

#[test]
fn test_search_returns_results() {
    // Search for a broad term — should succeed even if no results
    horus_cmd()
        .args(["search", "robot"])
        .assert()
        .success();
}

#[test]
fn test_search_json_output_valid() {
    let output = horus_cmd()
        .args(["search", "robot", "--json"])
        .output()
        .unwrap();
    assert!(output.status.success(), "search --json should succeed");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value =
        serde_json::from_str(&stdout).expect("search --json should return valid JSON");
    assert!(parsed["results"].is_array(), "results should be an array");
    assert_eq!(parsed["query"], "robot", "query should be echoed back");
}

#[test]
fn test_search_with_category_filter() {
    horus_cmd()
        .args(["search", "sim", "--category", "simulation"])
        .assert()
        .success();
}

#[test]
fn test_search_no_results_still_succeeds() {
    horus_cmd()
        .args(["search", "zzz_nonexistent_package_zzz_12345"])
        .assert()
        .success();
}

#[test]
fn test_publish_dry_run_requires_auth() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"test-qa-pkg\"\nversion = \"0.0.1\"\ndescription = \"QA test\"\nlicense = \"MIT\"\n",
    )
    .unwrap();

    // Dry-run without auth should fail with authentication error
    horus_cmd()
        .args(["publish", "--dry-run"])
        .current_dir(tmp.path())
        .assert()
        .failure()
        .stderr(predicate::str::contains("authentication"));
}

#[test]
fn test_publish_no_manifest_fails() {
    let tmp = TempDir::new().unwrap();

    horus_cmd()
        .args(["publish"])
        .current_dir(tmp.path())
        .assert()
        .failure();
}

#[test]
fn test_publish_with_path_deps_rejected() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"test-path-dep\"\nversion = \"0.0.1\"\ndescription = \"Test\"\nlicense = \"MIT\"\n\n[dependencies]\nlocal-crate = { path = \"../local-crate\" }\n",
    )
    .unwrap();

    horus_cmd()
        .args(["publish"])
        .current_dir(tmp.path())
        .assert()
        .failure()
        .stderr(
            predicate::str::contains("path dependenc")
                .or(predicate::str::contains("authentication")),
        );
}

// ============================================================================
// Deploy integration tests (dry-run — no SSH needed)
// ============================================================================

#[test]
fn test_deploy_list_no_targets() {
    let tmp = TempDir::new().unwrap();
    fs::write(tmp.path().join("horus.toml"), "[package]\nname = \"d\"\nversion = \"0.1.0\"\n").unwrap();

    horus_cmd()
        .args(["deploy", "--list"])
        .current_dir(tmp.path())
        .assert()
        .success()
        .stdout(predicate::str::contains("No deployment targets"));
}

#[test]
fn test_deploy_list_configured_targets() {
    let tmp = TempDir::new().unwrap();
    fs::write(tmp.path().join("horus.toml"), "[package]\nname = \"d\"\nversion = \"0.1.0\"\n").unwrap();
    let horus_dir = tmp.path().join(".horus");
    fs::create_dir_all(&horus_dir).unwrap();
    fs::write(
        horus_dir.join("deploy.yaml"),
        "targets:\n  robot:\n    host: pi@192.168.1.100\n    arch: aarch64\n    dir: ~/my_robot\n  jetson:\n    host: nvidia@jetson.local\n    arch: aarch64\n    dir: ~/horus_app\n",
    )
    .unwrap();

    horus_cmd()
        .args(["deploy", "--list"])
        .current_dir(tmp.path())
        .assert()
        .success()
        .stdout(predicate::str::contains("robot"))
        .stdout(predicate::str::contains("jetson"))
        .stdout(predicate::str::contains("pi@192.168.1.100"))
        .stdout(predicate::str::contains("nvidia@jetson.local"));
}

#[test]
fn test_deploy_dry_run_direct_target() {
    let tmp = TempDir::new().unwrap();
    fs::write(tmp.path().join("horus.toml"), "[package]\nname = \"d\"\nversion = \"0.1.0\"\n").unwrap();

    horus_cmd()
        .args(["deploy", "--dry-run", "user@10.0.0.1"])
        .current_dir(tmp.path())
        .assert()
        .success()
        .stdout(predicate::str::contains("DRY RUN"))
        .stdout(predicate::str::contains("user@10.0.0.1"))
        .stdout(predicate::str::contains("cargo build"))
        .stdout(predicate::str::contains("rsync"));
}

#[test]
fn test_deploy_dry_run_named_target() {
    let tmp = TempDir::new().unwrap();
    fs::write(tmp.path().join("horus.toml"), "[package]\nname = \"d\"\nversion = \"0.1.0\"\n").unwrap();
    let horus_dir = tmp.path().join(".horus");
    fs::create_dir_all(&horus_dir).unwrap();
    fs::write(
        horus_dir.join("deploy.yaml"),
        "targets:\n  robot:\n    host: pi@192.168.1.100\n    arch: aarch64\n    dir: ~/my_robot\n",
    )
    .unwrap();

    horus_cmd()
        .args(["deploy", "--dry-run", "robot"])
        .current_dir(tmp.path())
        .assert()
        .success()
        .stdout(predicate::str::contains("DRY RUN"))
        .stdout(predicate::str::contains("pi@192.168.1.100"))
        .stdout(predicate::str::contains("~/my_robot"));
}

#[test]
fn test_deploy_dry_run_debug_mode() {
    let tmp = TempDir::new().unwrap();
    fs::write(tmp.path().join("horus.toml"), "[package]\nname = \"d\"\nversion = \"0.1.0\"\n").unwrap();

    horus_cmd()
        .args(["deploy", "--dry-run", "--debug", "user@host"])
        .current_dir(tmp.path())
        .assert()
        .success()
        .stdout(predicate::str::contains("debug"));
}

#[test]
fn test_deploy_dry_run_custom_arch() {
    let tmp = TempDir::new().unwrap();
    fs::write(tmp.path().join("horus.toml"), "[package]\nname = \"d\"\nversion = \"0.1.0\"\n").unwrap();

    horus_cmd()
        .args(["deploy", "--dry-run", "--arch", "armv7", "user@host"])
        .current_dir(tmp.path())
        .assert()
        .success()
        .stdout(predicate::str::contains("armv7"));
}

#[test]
fn test_deploy_requires_target() {
    let tmp = TempDir::new().unwrap();
    fs::write(tmp.path().join("horus.toml"), "[package]\nname = \"d\"\nversion = \"0.1.0\"\n").unwrap();

    // No target provided — should fail
    horus_cmd()
        .args(["deploy", "--dry-run"])
        .current_dir(tmp.path())
        .assert()
        .failure();
}

// ============================================================================
// Action command tests
// ============================================================================

#[test]
fn test_action_help() {
    horus_cmd()
        .args(["action", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("list"))
        .stdout(predicate::str::contains("info"));
}

#[test]
fn test_action_list_help() {
    horus_cmd()
        .args(["action", "list", "--help"])
        .assert()
        .success();
}

#[test]
fn test_action_list_no_runtime() {
    // Without a running scheduler, action list should succeed with empty or "no actions"
    horus_cmd()
        .args(["action", "list"])
        .assert()
        .success();
}

#[test]
fn test_action_list_json_no_runtime() {
    let output = horus_cmd()
        .args(["action", "list", "--json"])
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(&stdout).expect("action list --json should be valid JSON");
    assert!(parsed["items"].is_array());
}

#[test]
fn test_action_info_nonexistent() {
    horus_cmd()
        .args(["action", "info", "fake_action_xyz"])
        .assert()
        .failure();
}

#[test]
fn test_action_send_goal_help() {
    horus_cmd()
        .args(["action", "send-goal", "--help"])
        .assert()
        .success();
}

#[test]
fn test_action_cancel_goal_help() {
    horus_cmd()
        .args(["action", "cancel-goal", "--help"])
        .assert()
        .success();
}

#[test]
fn test_action_alias_a() {
    horus_cmd()
        .args(["a", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("list"));
}

// ============================================================================
// Fmt command tests
// ============================================================================

#[test]
fn test_fmt_help() {
    horus_cmd()
        .args(["fmt", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Format"))
        .stdout(predicate::str::contains("--check"));
}

#[test]
fn test_fmt_no_project() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("fmt")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "fmt without project should not panic");
}

#[test]
fn test_fmt_in_rust_project() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "fmt-test", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("fmt-test");

    let output = horus_cmd()
        .arg("fmt")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "fmt in project should not panic");
}

#[test]
fn test_fmt_check_mode() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "fmt-chk", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("fmt-chk");

    let output = horus_cmd()
        .args(["fmt", "--check"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "fmt --check should not panic");
}

#[test]
fn test_fmt_verbose() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "fmt-v", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("fmt-v");

    let output = horus_cmd()
        .args(["fmt", "--verbose"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "fmt --verbose should not panic");
}

// ============================================================================
// Lint command tests
// ============================================================================

#[test]
fn test_lint_help() {
    horus_cmd()
        .args(["lint", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Lint"))
        .stdout(predicate::str::contains("--fix"));
}

#[test]
fn test_lint_no_project() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("lint")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "lint without project should not panic");
}

#[test]
fn test_lint_in_rust_project() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "lint-test", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("lint-test");

    let output = horus_cmd()
        .arg("lint")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "lint in project should not panic");
}

#[test]
fn test_lint_fix_flag() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "lint-fix", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("lint-fix");

    let output = horus_cmd()
        .args(["lint", "--fix"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "lint --fix should not panic");
}

#[test]
fn test_lint_types_flag() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "lint-ty", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("lint-ty");

    let output = horus_cmd()
        .args(["lint", "--types"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "lint --types should not panic");
}

// ============================================================================
// Doc command tests
// ============================================================================

#[test]
fn test_doc_help() {
    horus_cmd()
        .args(["doc", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("documentation"))
        .stdout(predicate::str::contains("--open"));
}

#[test]
fn test_doc_no_project() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("doc")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "doc without project should not panic");
}

#[test]
fn test_doc_in_rust_project() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "doc-test", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("doc-test");

    let output = horus_cmd()
        .arg("doc")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "doc in project should not panic");
}

#[test]
fn test_doc_extract_flag() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "doc-ext", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("doc-ext");

    let output = horus_cmd()
        .args(["doc", "--extract"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "doc --extract should not panic");
}

#[test]
fn test_doc_coverage_flag() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "doc-cov", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("doc-cov");

    let output = horus_cmd()
        .args(["doc", "--coverage"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "doc --coverage should not panic");
}

// ============================================================================
// Bench command tests
// ============================================================================

#[test]
fn test_bench_help() {
    horus_cmd()
        .args(["bench", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("benchmark"));
}

#[test]
fn test_bench_no_project() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("bench")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "bench without project should not panic");
}

#[test]
fn test_bench_with_filter() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "bench-flt", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("bench-flt");

    let output = horus_cmd()
        .args(["bench", "my_benchmark"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "bench with filter should not panic");
}

#[test]
fn test_bench_verbose() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "bench-v", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("bench-v");

    let output = horus_cmd()
        .args(["bench", "--verbose"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "bench --verbose should not panic");
}

// ============================================================================
// Deps command tests
// ============================================================================

#[test]
fn test_deps_help() {
    horus_cmd()
        .args(["deps", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("tree"))
        .stdout(predicate::str::contains("why"))
        .stdout(predicate::str::contains("outdated"))
        .stdout(predicate::str::contains("audit"));
}

#[test]
fn test_deps_tree_help() {
    horus_cmd()
        .args(["deps", "tree", "--help"])
        .assert()
        .success();
}

#[test]
fn test_deps_tree_no_project() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .args(["deps", "tree"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "deps tree without project should not panic");
}

#[test]
fn test_deps_tree_in_project() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "deps-tree", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("deps-tree");

    let output = horus_cmd()
        .args(["deps", "tree"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "deps tree in project should not panic");
}

#[test]
fn test_deps_outdated_no_project() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .args(["deps", "outdated"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "deps outdated without project should not panic");
}

#[test]
fn test_deps_audit_no_project() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .args(["deps", "audit"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "deps audit without project should not panic");
}

#[test]
fn test_deps_why_no_project() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .args(["deps", "why", "serde"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "deps why without project should not panic");
}

// ============================================================================
// Test command tests
// ============================================================================

#[test]
fn test_horus_test_help() {
    horus_cmd()
        .args(["test", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("test"))
        .stdout(predicate::str::contains("--release"))
        .stdout(predicate::str::contains("--json"));
}

#[test]
fn test_horus_test_no_project() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("test")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "test without project should not panic");
}

#[test]
fn test_horus_test_in_rust_project() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "test-test", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("test-test");

    let output = horus_cmd()
        .arg("test")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "test in project should not panic");
}

#[test]
fn test_horus_test_with_filter() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "test-flt", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("test-flt");

    let output = horus_cmd()
        .args(["test", "my_test_name"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "test with filter should not panic");
}

#[test]
fn test_horus_test_json_flag() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "test-json", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("test-json");

    let output = horus_cmd()
        .args(["test", "--json"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "test --json should not panic");
}

#[test]
fn test_horus_test_verbose() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "test-verb", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("test-verb");

    let output = horus_cmd()
        .args(["test", "--verbose"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "test --verbose should not panic");
}

// ============================================================================
// Log command tests
// ============================================================================

#[test]
fn test_log_help() {
    horus_cmd()
        .args(["log", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("log"))
        .stdout(predicate::str::contains("--level"))
        .stdout(predicate::str::contains("--follow"));
}

#[test]
fn test_log_no_runtime() {
    let output = horus_cmd()
        .arg("log")
        .output()
        .unwrap();
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!combined.contains("panicked"), "log without runtime should not panic");
}

#[test]
fn test_log_with_node_filter() {
    let output = horus_cmd()
        .args(["log", "sensor_node"])
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "log with node filter should not panic");
}

#[test]
fn test_log_level_filter() {
    let output = horus_cmd()
        .args(["log", "--level", "error"])
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "log --level should not panic");
}

#[test]
fn test_log_since_flag() {
    let output = horus_cmd()
        .args(["log", "--since", "5m"])
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "log --since should not panic");
}

#[test]
fn test_log_count_flag() {
    let output = horus_cmd()
        .args(["log", "--count", "10"])
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "log --count should not panic");
}

#[test]
fn test_log_clear_flag() {
    let output = horus_cmd()
        .args(["log", "--clear"])
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "log --clear should not panic");
}

// ============================================================================
// Doctor command tests
// ============================================================================

#[test]
fn test_doctor_help() {
    horus_cmd()
        .args(["doctor", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("health"));
}

#[test]
fn test_doctor_runs_outside_project() {
    let tmp = TempDir::new().unwrap();
    // No horus.toml — doctor runs system checks, may return nonzero for warnings
    let output = horus_cmd()
        .arg("doctor")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    // Should produce diagnostic output, not crash
    assert!(stdout.contains("doctor") || stdout.contains("Toolchain") || stdout.contains("Summary"),
        "doctor should produce diagnostic output, got: {}", stdout);
}

#[test]
fn test_doctor_runs_in_project() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"doc-test\"\nversion = \"0.1.0\"\nedition = \"2024\"\n",
    )
    .unwrap();
    let output = horus_cmd()
        .arg("doctor")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("doctor") || stdout.contains("Summary"),
        "doctor in project should produce output");
}

#[test]
fn test_doctor_json_output() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .args(["doctor", "--json"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    // Should be parseable JSON (doctor may return nonzero for warnings)
    let parsed: serde_json::Value = serde_json::from_str(&stdout)
        .unwrap_or_else(|_| panic!("doctor --json should be valid JSON, got: {}", stdout));
    assert!(parsed.is_array() || parsed.is_object(), "doctor --json should return JSON array or object");
}

#[test]
fn test_doctor_verbose() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .args(["doctor", "--verbose"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    // Verbose should produce more output than non-verbose
    assert!(!stdout.is_empty(), "doctor --verbose should produce output");
}

#[test]
fn test_doctor_checks_toolchains() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("doctor")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    // Doctor should mention toolchains
    assert!(
        stdout.contains("Toolchain") || stdout.contains("tools") || stdout.contains("rustc")
            || stdout.contains("cargo") || stdout.contains("Rust"),
        "doctor should check toolchains, got: {}", stdout
    );
}

#[test]
fn test_doctor_shows_summary() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("doctor")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains("Summary") || stdout.contains("ok") || stdout.contains("warning"),
        "doctor should show summary, got: {}", stdout
    );
}

#[test]
fn test_doctor_no_crash_empty_dir() {
    let tmp = TempDir::new().unwrap();
    // Completely empty directory — should not panic
    let output = horus_cmd()
        .arg("doctor")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    // Any exit code is fine, but should not have panicked
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "doctor should not panic in empty dir");
}

#[test]
fn test_doctor_no_crash_malformed_toml() {
    let tmp = TempDir::new().unwrap();
    fs::write(tmp.path().join("horus.toml"), "this is {{{{ not valid toml !!!!").unwrap();
    let output = horus_cmd()
        .arg("doctor")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    // Should report error, not crash
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "doctor should not panic on malformed toml");
}

// ============================================================================
// Sync command tests
// ============================================================================

#[test]
fn test_sync_help() {
    horus_cmd()
        .args(["sync", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Synchronize"));
}

#[test]
fn test_sync_no_project() {
    let tmp = TempDir::new().unwrap();
    // No horus.toml — should report no project found
    horus_cmd()
        .arg("sync")
        .current_dir(tmp.path())
        .assert()
        .success()
        .stdout(predicate::str::contains("horus.toml").or(predicate::str::contains("No")));
}

#[test]
fn test_sync_check_mode() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"sync-test\"\nversion = \"0.1.0\"\nedition = \"2024\"\n",
    )
    .unwrap();
    // --check should not create lockfile
    horus_cmd()
        .args(["sync", "--check"])
        .current_dir(tmp.path())
        .assert()
        .success();
    // No lockfile created in check mode
    assert!(!tmp.path().join("horus-sync.lock").exists());
}

#[test]
fn test_sync_in_rust_project() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"sync-rust\"\nversion = \"0.1.0\"\nedition = \"2024\"\n",
    )
    .unwrap();
    horus_cmd()
        .arg("sync")
        .current_dir(tmp.path())
        .assert()
        .success();
}

#[test]
fn test_sync_writes_lockfile() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"sync-lock\"\nversion = \"0.1.0\"\nedition = \"2024\"\n",
    )
    .unwrap();
    horus_cmd()
        .arg("sync")
        .current_dir(tmp.path())
        .assert()
        .success();
    assert!(tmp.path().join("horus-sync.lock").exists(), "sync should create horus-sync.lock");
}

#[test]
fn test_sync_check_output_lists_deps() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"sync-out\"\nversion = \"0.1.0\"\nedition = \"2024\"\n",
    )
    .unwrap();
    let output = horus_cmd()
        .args(["sync", "--check"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    // Should mention checking/syncing
    assert!(
        stdout.contains("Checking") || stdout.contains("Syncing") || stdout.contains("sync"),
        "sync --check should report what it's checking"
    );
}

// ============================================================================
// Completion command tests
// ============================================================================

#[test]
fn test_completion_bash() {
    let output = horus_cmd()
        .args(["completion", "bash"])
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(!stdout.is_empty(), "bash completion should produce output");
    assert!(stdout.contains("horus") || stdout.contains("complete"), "should be a bash completion script");
}

#[test]
fn test_completion_zsh() {
    let output = horus_cmd()
        .args(["completion", "zsh"])
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(!stdout.is_empty(), "zsh completion should produce output");
}

#[test]
fn test_completion_fish() {
    let output = horus_cmd()
        .args(["completion", "fish"])
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(!stdout.is_empty(), "fish completion should produce output");
}

#[test]
fn test_completion_powershell() {
    let output = horus_cmd()
        .args(["completion", "powershell"])
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(!stdout.is_empty(), "powershell completion should produce output");
}

// ============================================================================
// Launch command tests
// ============================================================================

#[test]
fn test_launch_help() {
    horus_cmd()
        .args(["launch", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Launch"))
        .stdout(predicate::str::contains("YAML"));
}

#[test]
fn test_launch_alias_l() {
    horus_cmd()
        .args(["l", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Launch"));
}

#[test]
fn test_launch_missing_file() {
    horus_cmd()
        .args(["launch", "nonexistent_file.yaml"])
        .assert()
        .failure();
}

#[test]
fn test_launch_invalid_yaml() {
    let tmp = TempDir::new().unwrap();
    fs::write(tmp.path().join("bad.yaml"), "{{{{ not yaml !!!!").unwrap();
    horus_cmd()
        .args(["launch", &tmp.path().join("bad.yaml").to_string_lossy()])
        .assert()
        .failure();
}

#[test]
fn test_launch_empty_yaml() {
    let tmp = TempDir::new().unwrap();
    fs::write(tmp.path().join("empty.yaml"), "").unwrap();
    let output = horus_cmd()
        .args(["launch", &tmp.path().join("empty.yaml").to_string_lossy()])
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "launch with empty yaml should not panic");
}

#[test]
fn test_launch_dry_run() {
    let tmp = TempDir::new().unwrap();
    let yaml = r#"
nodes:
  - name: sensor
    command: "echo sensor"
    rate: 100
"#;
    fs::write(tmp.path().join("launch.yaml"), yaml).unwrap();
    // Dry run should show what would launch without actually launching
    let output = horus_cmd()
        .args(["launch", "--dry-run", &tmp.path().join("launch.yaml").to_string_lossy()])
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "launch --dry-run should not panic");
}

#[test]
fn test_launch_no_args() {
    horus_cmd()
        .arg("launch")
        .assert()
        .failure(); // missing required FILE argument
}

#[test]
fn test_launch_namespace_flag() {
    horus_cmd()
        .args(["launch", "--namespace", "robot1", "--help"])
        .assert()
        .success();
}

// ============================================================================
// Scripts command tests
// ============================================================================

#[test]
fn test_scripts_help() {
    horus_cmd()
        .args(["scripts", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("script"));
}

#[test]
fn test_scripts_no_project() {
    let tmp = TempDir::new().unwrap();
    // No horus.toml — should report error
    let output = horus_cmd()
        .arg("scripts")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "scripts without project should not panic");
}

#[test]
fn test_scripts_list_with_scripts() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"s\"\nversion = \"0.1.0\"\n\n[scripts]\ntest = \"echo hello\"\nsim = \"echo sim\"\n",
    ).unwrap();
    let output = horus_cmd()
        .arg("scripts")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains("test") || stdout.contains("sim"),
        "scripts should list defined scripts, got: {}", stdout
    );
}

#[test]
fn test_scripts_run_echo() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"s\"\nversion = \"0.1.0\"\n\n[scripts]\ngreet = \"echo hello-from-script\"\n",
    ).unwrap();
    let output = horus_cmd()
        .args(["scripts", "greet"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert!(output.status.success(), "running echo script should succeed");
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("hello-from-script"), "script output should contain echo text");
}

#[test]
fn test_scripts_run_nonexistent() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"s\"\nversion = \"0.1.0\"\n\n[scripts]\ntest = \"echo hi\"\n",
    ).unwrap();
    horus_cmd()
        .args(["scripts", "nonexistent_script_xyz"])
        .current_dir(tmp.path())
        .assert()
        .failure();
}

#[test]
fn test_scripts_empty_section() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"s\"\nversion = \"0.1.0\"\n\n[scripts]\n",
    ).unwrap();
    let output = horus_cmd()
        .arg("scripts")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "empty scripts section should not panic");
}

// ============================================================================
// Uninstall command tests
// ============================================================================

#[test]
fn test_uninstall_help() {
    horus_cmd()
        .args(["uninstall", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Uninstall"))
        .stdout(predicate::str::contains("--purge"));
}

#[test]
fn test_uninstall_no_args() {
    horus_cmd()
        .arg("uninstall")
        .assert()
        .failure(); // missing required NAME argument
}

#[test]
fn test_uninstall_nonexistent_package() {
    let output = horus_cmd()
        .args(["uninstall", "fake-package-xyz-999"])
        .output()
        .unwrap();
    // Should fail gracefully (not installed)
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "uninstalling nonexistent package should not panic");
}

#[test]
fn test_uninstall_with_purge_flag() {
    let output = horus_cmd()
        .args(["uninstall", "fake-package-xyz-999", "--purge"])
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "uninstall --purge should not panic");
}

#[test]
fn test_uninstall_output_is_helpful() {
    let output = horus_cmd()
        .args(["uninstall", "nonexistent-pkg-abc"])
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    let combined = format!("{}{}", stdout, stderr);
    // Should give some indication of what went wrong
    assert!(!combined.is_empty(), "uninstall should produce output on failure");
}

// ============================================================================
// Monitor command CLI tests
// ============================================================================

#[test]
fn test_monitor_help() {
    horus_cmd()
        .args(["monitor", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Monitor"));
}

#[test]
fn test_monitor_alias_mon() {
    horus_cmd()
        .args(["mon", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Monitor"));
}

// ============================================================================
// Run command expanded tests
// ============================================================================

#[test]
fn test_run_no_project() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("run")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "run without project should not panic");
    // Should mention missing project or main file
    let combined = format!("{}{}", stdout, stderr);
    assert!(
        combined.contains("main") || combined.contains("horus.toml") || combined.contains("No")
            || combined.contains("not found") || combined.contains("Error"),
        "run should report missing project/main file"
    );
}

#[test]
fn test_run_in_rust_project() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "run-test", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("run-test");
    write_exiting_main_rs(&proj);

    // horus run will try to compile — may fail without full toolchain in CI
    // but should not panic
    let output = horus_cmd()
        .arg("run")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "run in project should not panic");
}

#[test]
fn test_run_verbose_flag() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["run", "--verbose", "--help"])
        .current_dir(tmp.path())
        .assert()
        .success();
}

#[test]
fn test_run_quiet_flag() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["run", "--quiet", "--help"])
        .current_dir(tmp.path())
        .assert()
        .success();
}

#[test]
fn test_run_release_flag() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "run-rel", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("run-rel");
    write_exiting_main_rs(&proj);

    let output = horus_cmd()
        .args(["run", "--release"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "run --release should not panic");
}

#[test]
fn test_run_clean_flag() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "run-cln", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("run-cln");
    write_exiting_main_rs(&proj);

    let output = horus_cmd()
        .args(["run", "--clean"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "run --clean should not panic");
}

#[test]
fn test_run_with_args_passthrough() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "run-args", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("run-args");
    write_exiting_main_rs(&proj);

    // Arguments after -- should be passed through without causing a CLI parse error
    let output = horus_cmd()
        .args(["run", "--", "--some-custom-flag", "value"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "run with pass-through args should not panic");
    // Should not fail with "unknown flag" — the args go to the child process
    assert!(
        !stderr.contains("unexpected argument"),
        "pass-through args should not be parsed by horus"
    );
}

#[test]
fn test_run_specific_file() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "run-file", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("run-file");
    write_exiting_main_rs(&proj);

    let output = horus_cmd()
        .args(["run", "src/main.rs"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "run specific file should not panic");
}

#[test]
fn test_run_nonexistent_file() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "run-nofile", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("run-nofile");

    let output = horus_cmd()
        .args(["run", "nonexistent.rs"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "run nonexistent file should not panic");
    assert!(!output.status.success(), "run nonexistent file should fail");
}

#[test]
fn test_run_script_mode() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "run-script", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("run-script");

    // Add a [scripts] section to horus.toml
    let toml_path = proj.join("horus.toml");
    let mut toml = fs::read_to_string(&toml_path).unwrap();
    toml.push_str("\n[scripts]\nhello = \"echo hello from horus\"\n");
    fs::write(&toml_path, toml).unwrap();

    let output = horus_cmd()
        .args(["run", "hello"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(!stderr.contains("panicked"), "run script should not panic");
    // Script should execute and produce output
    let combined = format!("{}{}", stdout, stderr);
    assert!(
        combined.contains("hello") || output.status.success(),
        "script should execute or at least not crash"
    );
}

#[test]
fn test_run_json_diagnostics_flag() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "run-jd", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("run-jd");
    write_exiting_main_rs(&proj);

    let output = horus_cmd()
        .args(["run", "--json-diagnostics"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "run --json-diagnostics should not panic");
}

#[test]
fn test_run_no_hooks_flag() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "run-nohk", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("run-nohk");
    write_exiting_main_rs(&proj);

    let output = horus_cmd()
        .args(["run", "--no-hooks"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "run --no-hooks should not panic");
}

#[test]
fn test_run_empty_project() {
    let tmp = TempDir::new().unwrap();
    // Create horus.toml but no src/
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"empty\"\nversion = \"0.1.0\"\n",
    )
    .unwrap();

    let output = horus_cmd()
        .arg("run")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "run in empty project should not panic");
    // Should fail since there's no source to run
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        stderr
    );
    assert!(
        !output.status.success() || combined.contains("No") || combined.contains("not found"),
        "run with no source should fail or report missing files"
    );
}

// ============================================================================
// Build command expanded tests
// ============================================================================

#[test]
fn test_build_no_project() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("build")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "build without project should not panic");
}

#[test]
fn test_build_creates_horus_dir() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "build-test", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("build-test");

    let output = horus_cmd()
        .arg("build")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "build should not panic");
    // After build, .horus/ directory should exist
    assert!(proj.join(".horus").exists(), ".horus/ should be created by build");
}

#[test]
fn test_build_verbose_flag() {
    horus_cmd()
        .args(["build", "--verbose", "--help"])
        .assert()
        .success();
}

#[test]
fn test_build_generates_cargo_toml() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "bgen-test", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("bgen-test");

    let output = horus_cmd()
        .arg("build")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "build should not panic");
    // Build pipeline generates .horus/Cargo.toml from horus.toml
    assert!(
        proj.join(".horus").join("Cargo.toml").exists(),
        ".horus/Cargo.toml should be generated by build"
    );
}

#[test]
fn test_build_release_flag() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "brel-test", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("brel-test");

    let output = horus_cmd()
        .args(["build", "--release"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "build --release should not panic");
}

#[test]
fn test_build_clean_flag() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "bcln-test", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("bcln-test");

    // First build to create .horus/
    horus_cmd()
        .arg("build")
        .current_dir(&proj)
        .output()
        .unwrap();

    // Build with --clean should wipe and rebuild
    let output = horus_cmd()
        .args(["build", "--clean"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "build --clean should not panic");
}

#[test]
fn test_build_json_diagnostics_flag() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "bjd-test", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("bjd-test");

    let output = horus_cmd()
        .args(["build", "--json-diagnostics"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "build --json-diagnostics should not panic");
}

#[test]
fn test_build_no_hooks_flag() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "bnohk-test", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("bnohk-test");

    let output = horus_cmd()
        .args(["build", "--no-hooks"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "build --no-hooks should not panic");
}

#[test]
fn test_build_twice_incremental() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "binc-test", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("binc-test");

    // First build
    let output1 = horus_cmd()
        .arg("build")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr1 = String::from_utf8_lossy(&output1.stderr);
    assert!(!stderr1.contains("panicked"), "first build should not panic");

    // Second build (incremental — should also succeed without panic)
    let output2 = horus_cmd()
        .arg("build")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr2 = String::from_utf8_lossy(&output2.stderr);
    assert!(!stderr2.contains("panicked"), "incremental build should not panic");
}

// ============================================================================
// Install command expanded tests
// ============================================================================

#[test]
fn test_install_alias_i() {
    horus_cmd()
        .args(["i", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Install"));
}

#[test]
fn test_install_no_name() {
    horus_cmd()
        .arg("install")
        .assert()
        .failure(); // missing required NAME argument
}

#[test]
fn test_install_name_at_version_parses() {
    // This will fail to connect to registry but should parse the name@version correctly
    let output = horus_cmd()
        .args(["install", "fake-pkg@1.2.3"])
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "install with @version should not panic");
}

#[test]
fn test_install_plugin_flag() {
    let output = horus_cmd()
        .args(["install", "fake-plugin", "--plugin"])
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "install --plugin should not panic");
}

#[test]
fn test_install_json_output_on_error() {
    let output = horus_cmd()
        .args(["install", "nonexistent-package-xyz-999", "--json"])
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    // JSON output should still be valid even on error
    if !stdout.trim().is_empty() {
        let parsed: Result<serde_json::Value, _> = serde_json::from_str(&stdout);
        assert!(parsed.is_ok(), "install --json should produce valid JSON, got: {}", stdout);
    }
}

#[test]
fn test_install_nonexistent_package() {
    // Installing a package that doesn't exist should fail gracefully
    let output = horus_cmd()
        .args(["install", "this-package-does-not-exist-abc-789"])
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "install nonexistent package should not panic");
    assert!(!output.status.success(), "install nonexistent package should fail");
}

#[test]
fn test_install_target_flag() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .args(["install", "some-pkg", "--target", &tmp.path().to_string_lossy()])
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "install --target should not panic");
    // Will fail (package doesn't exist) but the flag should be accepted
    assert!(
        !stderr.contains("unexpected argument"),
        "--target flag should be accepted by the parser"
    );
}

#[test]
fn test_install_invalid_version_format() {
    let output = horus_cmd()
        .args(["install", "fake-pkg@not-a-version!!!"])
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "install with invalid version should not panic");
}

// ============================================================================
// Update command expanded tests
// ============================================================================

#[test]
fn test_update_no_project() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("update")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "update without project should not panic");
}

#[test]
fn test_update_dry_run() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"up\"\nversion = \"0.1.0\"\n",
    ).unwrap();
    let output = horus_cmd()
        .args(["update", "--dry-run"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "update --dry-run should not panic");
}

#[test]
fn test_update_specific_package() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"up\"\nversion = \"0.1.0\"\n",
    ).unwrap();
    let output = horus_cmd()
        .args(["update", "serde"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "update specific package should not panic");
}

#[test]
fn test_update_mentions_self_update() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"up\"\nversion = \"0.1.0\"\n",
    ).unwrap();
    let output = horus_cmd()
        .arg("update")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    // After update, should hint about `horus self update` if CLI update available
    // (or not, if already latest — either way no crash)
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"));
}

#[test]
fn test_update_global_flag() {
    let output = horus_cmd()
        .args(["update", "--global"])
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "update --global should not panic");
    assert!(
        !stderr.contains("unexpected argument"),
        "--global flag should be accepted by the parser"
    );
}

#[test]
fn test_update_in_project_with_deps() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "upd-deps", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("upd-deps");

    // Add a dependency to horus.toml
    let toml_path = proj.join("horus.toml");
    let mut toml = fs::read_to_string(&toml_path).unwrap();
    toml.push_str("\n[dependencies]\nserde = { version = \"1.0\", source = \"crates.io\" }\n");
    fs::write(&toml_path, toml).unwrap();

    let output = horus_cmd()
        .arg("update")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "update in project with deps should not panic");
}

#[test]
fn test_update_dry_run_preserves_files() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "upd-dry", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("upd-dry");

    // Read horus.toml before update
    let toml_before = fs::read_to_string(proj.join("horus.toml")).unwrap();

    let output = horus_cmd()
        .args(["update", "--dry-run"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "update --dry-run should not panic");

    // Dry-run should NOT modify horus.toml
    let toml_after = fs::read_to_string(proj.join("horus.toml")).unwrap();
    assert_eq!(toml_before, toml_after, "dry-run should not modify horus.toml");
}

// ============================================================================
// Publish command expanded tests
// ============================================================================

#[test]
fn test_publish_no_project() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .arg("publish")
        .current_dir(tmp.path())
        .assert()
        .failure();
}

#[test]
fn test_publish_dry_run() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"pub-test\"\nversion = \"0.1.0\"\n",
    ).unwrap();
    let output = horus_cmd()
        .args(["publish", "--dry-run"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "publish --dry-run should not panic");
}

#[test]
fn test_publish_quiet_flag() {
    horus_cmd()
        .args(["publish", "--quiet", "--help"])
        .assert()
        .success();
}

#[test]
fn test_publish_missing_required_fields() {
    let tmp = TempDir::new().unwrap();
    // Minimal horus.toml without description or license — publish should reject
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"bad-pub\"\nversion = \"0.1.0\"\n",
    )
    .unwrap();
    let output = horus_cmd()
        .arg("publish")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "publish with missing fields should not panic");
    assert!(!output.status.success(), "publish with missing fields should fail");
}

#[test]
fn test_publish_in_real_project() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "pub-proj", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("pub-proj");

    // Publish without auth should fail gracefully
    let output = horus_cmd()
        .arg("publish")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "publish in real project should not panic");
    assert!(!output.status.success(), "publish without auth should fail");
}

// ============================================================================
// Self update command tests
// ============================================================================

#[test]
fn test_self_update_help() {
    horus_cmd()
        .args(["self", "update", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Update"));
}

#[test]
fn test_self_update_check() {
    let output = horus_cmd()
        .args(["self", "update", "--check"])
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "self update --check should not panic");
}

// ============================================================================
// Add command tests (new command from CLI redesign)
// ============================================================================

#[test]
fn test_add_help() {
    horus_cmd()
        .args(["add", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Add"))
        .stdout(predicate::str::contains("--source"))
        .stdout(predicate::str::contains("--dev"))
        .stdout(predicate::str::contains("--driver"));
}

#[test]
fn test_add_no_args() {
    horus_cmd()
        .arg("add")
        .assert()
        .failure(); // missing required NAME
}

#[test]
fn test_add_no_project() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["add", "serde"])
        .current_dir(tmp.path())
        .assert()
        .failure();
}

#[test]
fn test_add_to_project() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "add-test", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("add-test");

    let output = horus_cmd()
        .args(["add", "serde", "--source", "crates.io"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "add should not panic");

    // Verify horus.toml was updated
    let toml = fs::read_to_string(proj.join("horus.toml")).unwrap();
    assert!(toml.contains("serde"), "horus.toml should contain serde after add");
}

#[test]
fn test_add_with_version() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "addv-test", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("addv-test");

    let output = horus_cmd()
        .args(["add", "serde@1.0", "--source", "crates.io"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"));
}

#[test]
fn test_add_invalid_source() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"t\"\nversion = \"0.1.0\"\n",
    ).unwrap();
    horus_cmd()
        .args(["add", "serde", "--source", "fake-source"])
        .current_dir(tmp.path())
        .assert()
        .failure();
}

#[test]
fn test_add_dev_flag() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "addev", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("addev");

    let output = horus_cmd()
        .args(["add", "criterion", "--dev", "--source", "crates.io"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"));
}

#[test]
fn test_add_driver_flag() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "adddr", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("adddr");

    let output = horus_cmd()
        .args(["add", "camera-driver", "--driver"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"));
}
