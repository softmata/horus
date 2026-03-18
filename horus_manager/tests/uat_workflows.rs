//! UAT: End-to-end CLI workflow tests.
//!
//! Each test exercises a complete user journey — multiple CLI commands in sequence,
//! verifying they work together (not just individually). Covers:
//!   1. Rust project full lifecycle (new → check → add → build → fmt → lint → remove → clean)
//!   2. Python project lifecycle (new → add pypi → build → clean)
//!   3. Project health workflow (new → config → sync → doctor)
//!   4. Multi-dependency project (new → add N deps → deps tree → build → test)
//!   5. Project from scratch (init → add → build)
//!   6. Lock and verify (new → lock → check)

use assert_cmd::cargo::cargo_bin_cmd;
use assert_cmd::Command;
use std::fs;
use tempfile::TempDir;

/// Helper to get the CLI command.
fn horus_cmd() -> Command {
    cargo_bin_cmd!("horus")
}

/// Replace the scheduler-based main.rs with one that exits immediately.
fn write_exiting_main_rs(proj: &std::path::Path) {
    fs::write(
        proj.join("src/main.rs"),
        "fn main() { println!(\"horus project ok\"); }\n",
    )
    .unwrap();
}

/// Strip horus dep from horus.toml so compilation doesn't require the horus crate.
fn strip_horus_dep(proj: &std::path::Path) {
    let toml_path = proj.join("horus.toml");
    let toml = fs::read_to_string(&toml_path).unwrap();
    // Remove the [dependencies] section that references horus
    let cleaned: String = toml
        .lines()
        .filter(|l| !l.contains("horus =") && !l.contains("horus_core"))
        .collect::<Vec<_>>()
        .join("\n");
    fs::write(&toml_path, cleaned).unwrap();
}

// ============================================================================
// Workflow 1: Rust Project Full Lifecycle
// ============================================================================

#[test]
fn uat_rust_project_full_lifecycle() {
    let tmp = TempDir::new().unwrap();

    // 1. Create project
    horus_cmd()
        .args(["new", "my-robot", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("my-robot");
    assert!(proj.join("horus.toml").exists(), "horus.toml should exist");
    assert!(
        proj.join("src/main.rs").exists(),
        "src/main.rs should exist"
    );

    // 2. Check project is valid
    let output = horus_cmd()
        .arg("check")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "check should not panic");

    // 3. Add a dependency
    let output = horus_cmd()
        .args(["add", "serde", "--source", "crates.io"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "add should not panic");

    // Verify horus.toml was updated
    let toml = fs::read_to_string(proj.join("horus.toml")).unwrap();
    assert!(
        toml.contains("serde"),
        "horus.toml should contain serde after add"
    );

    // 4. Format code
    let output = horus_cmd().arg("fmt").current_dir(&proj).output().unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "fmt should not panic");

    // 5. Lint code
    let output = horus_cmd().arg("lint").current_dir(&proj).output().unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "lint should not panic");

    // 6. Build (may fail if horus crate not available, but should not panic)
    let output = horus_cmd()
        .arg("build")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "build should not panic");
    // .horus/ directory should be created by build pipeline
    assert!(
        proj.join(".horus").exists(),
        ".horus/ should be created by build"
    );

    // 7. Remove dependency
    let output = horus_cmd()
        .args(["remove", "serde"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "remove should not panic");
    let toml2 = fs::read_to_string(proj.join("horus.toml")).unwrap();
    assert!(
        !toml2.contains("serde"),
        "horus.toml should not contain serde after remove"
    );

    // 8. Clean
    let output = horus_cmd()
        .arg("clean")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "clean should not panic");
}

// ============================================================================
// Workflow 2: Python Project Lifecycle
// ============================================================================

#[test]
fn uat_python_project_lifecycle() {
    let tmp = TempDir::new().unwrap();

    // 1. Create Python project
    horus_cmd()
        .args(["new", "py-robot", "-p", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("py-robot");
    assert!(proj.join("horus.toml").exists(), "horus.toml should exist");
    assert!(
        proj.join("main.py").exists() || proj.join("src/main.py").exists(),
        "main.py should exist for Python project"
    );

    // 2. Check project
    let output = horus_cmd()
        .arg("check")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "check should not panic");

    // 3. Add a PyPI dependency
    let output = horus_cmd()
        .args(["add", "numpy", "--source", "pypi"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        !stderr.contains("panicked"),
        "add pypi dep should not panic"
    );

    // Verify horus.toml was updated
    let toml = fs::read_to_string(proj.join("horus.toml")).unwrap();
    assert!(
        toml.contains("numpy"),
        "horus.toml should contain numpy after add"
    );

    // 4. Build (generates .horus/pyproject.toml)
    let output = horus_cmd()
        .arg("build")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "build should not panic");

    // 5. Clean
    let output = horus_cmd()
        .arg("clean")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "clean should not panic");
}

// ============================================================================
// Workflow 3: Project Health Workflow
// ============================================================================

#[test]
fn uat_project_health_workflow() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args([
            "new",
            "health-test",
            "-r",
            "-o",
            &tmp.path().to_string_lossy(),
        ])
        .assert()
        .success();
    let proj = tmp.path().join("health-test");

    // 1. Config list
    let output = horus_cmd()
        .args(["config", "list"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "config list should not panic");

    // 2. Sync --check (check only, don't install)
    let output = horus_cmd()
        .args(["sync", "--check"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        !stderr.contains("panicked"),
        "sync --check should not panic"
    );

    // 3. Doctor
    let output = horus_cmd()
        .arg("doctor")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "doctor should not panic");
    assert!(
        output.status.success(),
        "doctor should succeed in a valid project"
    );

    // 4. Check --full (comprehensive validation)
    let output = horus_cmd()
        .args(["check", "--full"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        !stderr.contains("panicked"),
        "check --full should not panic"
    );
}

// ============================================================================
// Workflow 4: Multi-Dependency Project
// ============================================================================

#[test]
fn uat_multi_dependency_project() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args([
            "new",
            "multi-dep",
            "-r",
            "-o",
            &tmp.path().to_string_lossy(),
        ])
        .assert()
        .success();
    let proj = tmp.path().join("multi-dep");

    // 1. Add multiple dependencies
    for dep in &["serde", "tokio", "anyhow"] {
        let output = horus_cmd()
            .args(["add", dep, "--source", "crates.io"])
            .current_dir(&proj)
            .output()
            .unwrap();
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert!(!stderr.contains("panicked"), "add {} should not panic", dep);
    }

    // Verify all deps in horus.toml
    let toml = fs::read_to_string(proj.join("horus.toml")).unwrap();
    assert!(toml.contains("serde"), "should contain serde");
    assert!(toml.contains("tokio"), "should contain tokio");
    assert!(toml.contains("anyhow"), "should contain anyhow");

    // 2. Deps tree
    let output = horus_cmd()
        .args(["deps", "tree"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "deps tree should not panic");

    // 3. Build
    let output = horus_cmd()
        .arg("build")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "build should not panic");

    // 4. Test (may have 0 tests, that's fine)
    let output = horus_cmd().arg("test").current_dir(&proj).output().unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "test should not panic");

    // 5. Remove one dep, others remain
    horus_cmd()
        .args(["remove", "tokio"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let toml2 = fs::read_to_string(proj.join("horus.toml")).unwrap();
    assert!(!toml2.contains("tokio"), "tokio should be removed");
    assert!(toml2.contains("serde"), "serde should still be present");
    assert!(toml2.contains("anyhow"), "anyhow should still be present");
}

// ============================================================================
// Workflow 5: Init in Existing Directory
// ============================================================================

#[test]
fn uat_init_existing_directory() {
    let tmp = TempDir::new().unwrap();

    // 1. Init workspace in empty dir
    let output = horus_cmd()
        .arg("init")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "init should not panic");

    // Should create horus.toml
    assert!(
        tmp.path().join("horus.toml").exists(),
        "init should create horus.toml"
    );

    // 2. Check the initialized workspace
    let output = horus_cmd()
        .arg("check")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        !stderr.contains("panicked"),
        "check after init should not panic"
    );

    // 3. Doctor the workspace
    let output = horus_cmd()
        .arg("doctor")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        !stderr.contains("panicked"),
        "doctor after init should not panic"
    );
}

// ============================================================================
// Workflow 6: Run with Quick-Exit Project
// ============================================================================

#[test]
fn uat_build_and_run_quick_exit() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args([
            "new",
            "quick-run",
            "-r",
            "-o",
            &tmp.path().to_string_lossy(),
        ])
        .assert()
        .success();
    let proj = tmp.path().join("quick-run");

    // Replace with quick-exit main and strip horus dep
    write_exiting_main_rs(&proj);
    strip_horus_dep(&proj);

    // 1. Build
    let output = horus_cmd()
        .arg("build")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "build should not panic");

    // 2. Run (should exit immediately with our quick-exit main)
    let output = horus_cmd().arg("run").current_dir(&proj).output().unwrap();
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "run should not panic");
    // If build succeeded, check for our output
    if output.status.success() {
        assert!(
            stdout.contains("horus project ok") || stderr.contains("horus project ok"),
            "run should produce our quick-exit output"
        );
    }
}

// ============================================================================
// Workflow 7: Auth → Signing Key → Publish Attempt
// ============================================================================

#[test]
fn uat_auth_and_publish_workflow() {
    let tmp = TempDir::new().unwrap();
    let key_dir = tmp.path().join("keys");

    // 1. Generate signing key (isolated)
    let output = horus_cmd()
        .args(["auth", "signing-key"])
        .env("XDG_DATA_HOME", &key_dir)
        .output()
        .unwrap();
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(
        !combined.contains("panicked"),
        "signing-key should not panic"
    );

    // 2. Check whoami (not logged in)
    let output = horus_cmd().args(["auth", "whoami"]).output().unwrap();
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(
        combined.contains("Not logged in")
            || combined.contains("not logged in")
            || combined.contains("authenticate"),
        "should report not logged in"
    );

    // 3. Create a project and try to publish (should fail — not logged in)
    horus_cmd()
        .args(["new", "pub-test", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("pub-test");

    let output = horus_cmd()
        .arg("publish")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "publish should not panic");
    assert!(!output.status.success(), "publish without auth should fail");
}

// ============================================================================
// Workflow 8: Dev Dependency Workflow
// ============================================================================

#[test]
fn uat_dev_dependency_workflow() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "dev-dep", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("dev-dep");

    // 1. Add a dev dependency
    let output = horus_cmd()
        .args(["add", "criterion", "--dev", "--source", "crates.io"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "add --dev should not panic");

    // 2. Verify it's in [dev-dependencies]
    let toml = fs::read_to_string(proj.join("horus.toml")).unwrap();
    assert!(
        toml.contains("criterion"),
        "horus.toml should contain criterion"
    );

    // 3. Build should still work (dev deps don't block main build)
    let output = horus_cmd()
        .arg("build")
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        !stderr.contains("panicked"),
        "build with dev deps should not panic"
    );

    // 4. Remove dev dep
    let output = horus_cmd()
        .args(["remove", "criterion"])
        .current_dir(&proj)
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        !stderr.contains("panicked"),
        "remove dev dep should not panic"
    );
    let toml2 = fs::read_to_string(proj.join("horus.toml")).unwrap();
    assert!(!toml2.contains("criterion"), "criterion should be removed");
}
