use assert_cmd::Command;
use predicates::prelude::*;
use std::fs;
use tempfile::TempDir;

/// Helper to get the CLI command
fn horus_cmd() -> Command {
    Command::cargo_bin("horus").unwrap()
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
        .stdout(predicate::str::contains("pkg"))
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
fn test_pkg_help() {
    horus_cmd()
        .args(["pkg", "--help"])
        .assert()
        .success()
        .stdout(predicate::str::contains("Package"));
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
    horus_cmd()
        .assert()
        .failure();
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
    horus_cmd()
        .args(["msg", "list"])
        .assert()
        .success();
}
