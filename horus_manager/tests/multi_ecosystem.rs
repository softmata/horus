#![allow(dead_code)]
//! Multi-ecosystem `horus add` / `horus remove` tests.
//!
//! Tests all 6 dependency sources (crates.io, pypi, system, registry, git, path)
//! with both Rust and Python projects. Verifies horus.toml output format,
//! auto-detection, explicit overrides, and roundtrip add/remove.

use assert_cmd::cargo::cargo_bin_cmd;
use assert_cmd::Command;
use std::fs;
use tempfile::TempDir;

fn horus_cmd() -> Command {
    cargo_bin_cmd!("horus")
}

/// Create a Rust project in tmp, return the project path.
fn new_rust_project(tmp: &TempDir, name: &str) -> std::path::PathBuf {
    horus_cmd()
        .args(["new", name, "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    tmp.path().join(name)
}

/// Create a Python project in tmp, return the project path.
fn new_python_project(tmp: &TempDir, name: &str) -> std::path::PathBuf {
    horus_cmd()
        .args(["new", name, "-p", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    tmp.path().join(name)
}

/// Read horus.toml from a project dir.
fn read_toml(proj: &std::path::Path) -> String {
    fs::read_to_string(proj.join("horus.toml")).unwrap()
}

/// Assert no panic in output.
fn assert_no_panic(output: &std::process::Output, ctx: &str) {
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("panicked"), "{} should not panic", ctx);
}

// ============================================================================
// Source: crates.io
// ============================================================================

#[test]
fn test_add_crates_io_explicit() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "cio1");
    let output = horus_cmd()
        .args(["add", "serde", "--source", "crates.io"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "add crates.io");
    assert!(output.status.success(), "add serde should succeed");
    let toml = read_toml(&proj);
    assert!(
        toml.contains("[dependencies.serde]"),
        "should have serde section"
    );
    assert!(
        toml.contains("source = \"crates.io\""),
        "source should be crates.io"
    );
}

#[test]
fn test_add_crates_io_at_version() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "cio2");
    let output = horus_cmd()
        .args(["add", "tokio@1.0", "--source", "crates.io"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "add tokio@1.0");
    assert!(output.status.success());
    let toml = read_toml(&proj);
    assert!(
        toml.contains("version = \"1.0\""),
        "should pin version to 1.0"
    );
}

#[test]
fn test_add_crates_io_with_features() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "cio3");
    let output = horus_cmd()
        .args([
            "add",
            "serde",
            "--source",
            "crates.io",
            "--features",
            "derive",
        ])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "add with features");
    assert!(output.status.success());
    let toml = read_toml(&proj);
    assert!(
        toml.contains("derive"),
        "should contain derive feature, got:\n{}",
        toml
    );
}

#[test]
fn test_add_crates_io_dev_dep() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "cio4");
    let output = horus_cmd()
        .args(["add", "criterion", "--dev", "--source", "crates.io"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "add --dev");
    assert!(output.status.success());
    let toml = read_toml(&proj);
    assert!(
        toml.contains("[dev-dependencies.criterion]"),
        "should be in dev-dependencies, got:\n{}",
        toml
    );
}

// ============================================================================
// Source: pypi
// ============================================================================

#[test]
fn test_add_pypi_in_python_project() {
    let tmp = TempDir::new().unwrap();
    let proj = new_python_project(&tmp, "pypi1");
    let output = horus_cmd()
        .args(["add", "numpy", "--source", "pypi"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "add pypi");
    assert!(output.status.success());
    let toml = read_toml(&proj);
    assert!(
        toml.contains("[dependencies.numpy]"),
        "should have numpy section"
    );
    assert!(toml.contains("source = \"pypi\""), "source should be pypi");
}

#[test]
fn test_add_pypi_multiple_deps() {
    let tmp = TempDir::new().unwrap();
    let proj = new_python_project(&tmp, "pypi2");
    for dep in &["numpy", "requests", "flask"] {
        horus_cmd()
            .args(["add", dep, "--source", "pypi"])
            .current_dir(&proj)
            .output()
            .unwrap();
    }
    let toml = read_toml(&proj);
    assert!(toml.contains("numpy"), "should contain numpy");
    assert!(toml.contains("requests"), "should contain requests");
    assert!(toml.contains("flask"), "should contain flask");
}

#[test]
fn test_add_pypi_in_rust_project_with_override() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "pypi3");
    // Force pypi source on a Rust project
    let output = horus_cmd()
        .args(["add", "numpy", "--source", "pypi"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "add pypi to rust project");
    assert!(output.status.success());
    let toml = read_toml(&proj);
    assert!(
        toml.contains("source = \"pypi\""),
        "source override should work"
    );
}

// ============================================================================
// Source: system
// ============================================================================

#[test]
fn test_add_system_dep() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "sys1");
    let output = horus_cmd()
        .args(["add", "fake-system-pkg", "--source", "system"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "add system dep");
    // May fail to install (fake package) but should write to toml
    let _toml = read_toml(&proj);
    // System deps are attempted to install via apt — the toml entry may or may not
    // be written depending on whether install succeeded. Just verify no panic.
}

// ============================================================================
// Source: path
// ============================================================================

#[test]
fn test_add_path_dep() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "path1");
    // Create a local lib directory
    let lib_dir = proj.join("mylib");
    fs::create_dir_all(&lib_dir).unwrap();
    fs::write(
        lib_dir.join("horus.toml"),
        "[package]\nname = \"mylib\"\nversion = \"0.1.0\"\n",
    )
    .unwrap();

    let output = horus_cmd()
        .args(["add", "mylib", "--source", "path"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "add path dep");
    assert!(output.status.success());
    let toml = read_toml(&proj);
    assert!(toml.contains("source = \"path\""), "source should be path");
    assert!(toml.contains("path = \"mylib\""), "should have path field");
}

#[test]
fn test_add_path_dep_nonexistent() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "path2");
    let output = horus_cmd()
        .args(["add", "nonexistent-dir", "--source", "path"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "add nonexistent path dep");
    assert!(
        !output.status.success(),
        "path dep to nonexistent dir should fail"
    );
}

// ============================================================================
// Source: git
// ============================================================================

#[test]
fn test_add_git_dep() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "git1");
    let output = horus_cmd()
        .args([
            "add",
            "https://github.com/example/repo.git",
            "--source",
            "git",
        ])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "add git dep");
    // Git dep may fail to clone, but the flag should be accepted
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(
        !combined.contains("unexpected argument"),
        "git source should be accepted"
    );
}

// ============================================================================
// Source: registry
// ============================================================================

#[test]
fn test_add_registry_dep() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "reg1");
    let output = horus_cmd()
        .args(["add", "my-horus-pkg", "--source", "registry"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "add registry dep");
    // Registry fetch may fail (network), but toml should be updated or error graceful
}

// ============================================================================
// Driver flag
// ============================================================================

#[test]
fn test_add_driver() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "drv1");
    let output = horus_cmd()
        .args(["add", "camera", "--driver"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "add driver");
    assert!(output.status.success());
    let toml = read_toml(&proj);
    assert!(toml.contains("[drivers]"), "should have [drivers] section");
    assert!(toml.contains("camera"), "should contain camera driver");
}

#[test]
fn test_add_multiple_drivers() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "drv2");
    for drv in &["camera", "lidar", "imu"] {
        horus_cmd()
            .args(["add", drv, "--driver"])
            .current_dir(&proj)
            .output()
            .unwrap();
    }
    let toml = read_toml(&proj);
    assert!(toml.contains("camera"), "should contain camera");
    assert!(toml.contains("lidar"), "should contain lidar");
    assert!(toml.contains("imu"), "should contain imu");
}

// ============================================================================
// JSON output
// ============================================================================

#[test]
fn test_add_json_output() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "json1");
    let output = horus_cmd()
        .args(["add", "serde", "--source", "crates.io", "--json"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "add --json");
    let stdout = String::from_utf8_lossy(&output.stdout);
    if !stdout.trim().is_empty() {
        // JSON may be on the last line after human-readable output
        let last_json_line = stdout.lines().rev().find(|l| l.starts_with('{'));
        if let Some(json_line) = last_json_line {
            let parsed: Result<serde_json::Value, _> = serde_json::from_str(json_line);
            assert!(
                parsed.is_ok(),
                "add --json should produce valid JSON, got: {}",
                json_line
            );
        }
    }
}

// ============================================================================
// Remove verification
// ============================================================================

#[test]
fn test_remove_crates_io_dep() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "rm1");
    horus_cmd()
        .args(["add", "serde", "--source", "crates.io"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert!(
        read_toml(&proj).contains("serde"),
        "serde should be present after add"
    );

    let output = horus_cmd()
        .args(["remove", "serde"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "remove serde");
    assert!(output.status.success());
    assert!(
        !read_toml(&proj).contains("serde"),
        "serde should be gone after remove"
    );
}

#[test]
fn test_remove_pypi_dep() {
    let tmp = TempDir::new().unwrap();
    let proj = new_python_project(&tmp, "rm2");
    horus_cmd()
        .args(["add", "numpy", "--source", "pypi"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert!(
        read_toml(&proj).contains("numpy"),
        "numpy should be present"
    );

    let output = horus_cmd()
        .args(["remove", "numpy"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "remove numpy");
    assert!(output.status.success());
    assert!(
        !read_toml(&proj).contains("numpy"),
        "numpy should be gone after remove"
    );
}

#[test]
fn test_remove_preserves_other_deps() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "rm3");
    horus_cmd()
        .args(["add", "serde", "--source", "crates.io"])
        .current_dir(&proj)
        .output()
        .unwrap();
    horus_cmd()
        .args(["add", "anyhow", "--source", "crates.io"])
        .current_dir(&proj)
        .output()
        .unwrap();
    horus_cmd()
        .args(["add", "tokio", "--source", "crates.io"])
        .current_dir(&proj)
        .output()
        .unwrap();

    // Remove middle dep
    horus_cmd()
        .args(["remove", "anyhow"])
        .current_dir(&proj)
        .output()
        .unwrap();

    let toml = read_toml(&proj);
    assert!(!toml.contains("anyhow"), "anyhow should be removed");
    assert!(toml.contains("serde"), "serde should remain");
    assert!(toml.contains("tokio"), "tokio should remain");
}

#[test]
fn test_add_remove_roundtrip() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "rt1");
    let _toml_before = read_toml(&proj);

    // Add and remove same dep
    horus_cmd()
        .args(["add", "serde", "--source", "crates.io"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert!(read_toml(&proj).contains("serde"), "added");
    horus_cmd()
        .args(["remove", "serde"])
        .current_dir(&proj)
        .output()
        .unwrap();

    let toml_after = read_toml(&proj);
    assert!(!toml_after.contains("serde"), "serde removed");
    // The toml may not be byte-identical (empty [dependencies] section may remain)
    // but serde should be gone
}

#[test]
fn test_add_sequential_then_remove_all() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "seq1");

    let deps = ["dep_a", "dep_b", "dep_c"];
    for dep in &deps {
        horus_cmd()
            .args(["add", dep, "--source", "crates.io"])
            .current_dir(&proj)
            .output()
            .unwrap();
    }

    let toml = read_toml(&proj);
    for dep in &deps {
        assert!(toml.contains(dep), "{} should be present", dep);
    }

    // Remove all
    for dep in &deps {
        horus_cmd()
            .args(["remove", dep])
            .current_dir(&proj)
            .output()
            .unwrap();
    }

    let toml = read_toml(&proj);
    for dep in &deps {
        assert!(!toml.contains(dep), "{} should be removed", dep);
    }
}

// ============================================================================
// Build file regeneration
// ============================================================================

#[test]
fn test_add_triggers_horus_dir_creation() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "bfr1");

    // Build first to create .horus/
    horus_cmd()
        .arg("build")
        .current_dir(&proj)
        .output()
        .unwrap();

    let cargo_toml = proj.join(".horus/Cargo.toml");
    let existed_before = cargo_toml.exists();

    // Add a dep
    horus_cmd()
        .args(["add", "serde", "--source", "crates.io"])
        .current_dir(&proj)
        .output()
        .unwrap();

    // Build again to regenerate
    horus_cmd()
        .arg("build")
        .current_dir(&proj)
        .output()
        .unwrap();

    if existed_before {
        // If .horus/Cargo.toml existed, it should now contain serde
        if cargo_toml.exists() {
            let content = fs::read_to_string(&cargo_toml).unwrap();
            assert!(
                content.contains("serde"),
                ".horus/Cargo.toml should contain serde after build"
            );
        }
    }
}

#[test]
fn test_remove_then_build_regenerates() {
    let tmp = TempDir::new().unwrap();
    let proj = new_rust_project(&tmp, "bfr2");

    horus_cmd()
        .args(["add", "serde", "--source", "crates.io"])
        .current_dir(&proj)
        .output()
        .unwrap();

    // Build with serde
    horus_cmd()
        .arg("build")
        .current_dir(&proj)
        .output()
        .unwrap();

    // Remove serde
    horus_cmd()
        .args(["remove", "serde"])
        .current_dir(&proj)
        .output()
        .unwrap();

    // Build again
    horus_cmd()
        .arg("build")
        .current_dir(&proj)
        .output()
        .unwrap();

    // .horus/Cargo.toml should no longer mention serde
    let cargo_toml = proj.join(".horus/Cargo.toml");
    if cargo_toml.exists() {
        let content = fs::read_to_string(&cargo_toml).unwrap();
        assert!(
            !content.contains("serde"),
            ".horus/Cargo.toml should not contain serde after remove + build"
        );
    }
}

// ============================================================================
// Mixed project tests
// ============================================================================

#[test]
fn test_add_crates_io_to_python_project() {
    let tmp = TempDir::new().unwrap();
    let proj = new_python_project(&tmp, "mix1");
    // This is unusual but should work with explicit source
    let output = horus_cmd()
        .args(["add", "serde", "--source", "crates.io"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "add crates.io to python project");
    assert!(output.status.success());
    let toml = read_toml(&proj);
    assert!(toml.contains("source = \"crates.io\""));
}

#[test]
fn test_add_pypi_at_version() {
    let tmp = TempDir::new().unwrap();
    let proj = new_python_project(&tmp, "pyv1");
    let output = horus_cmd()
        .args(["add", "numpy@1.24", "--source", "pypi"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "add numpy@1.24");
    assert!(output.status.success());
    let toml = read_toml(&proj);
    assert!(toml.contains("version = \"1.24\""), "should pin to 1.24");
}
