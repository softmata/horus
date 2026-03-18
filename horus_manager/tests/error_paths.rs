//! Error path and destructive safety tests.
//!
//! Systematically tests that every error condition produces:
//!   - A clear, helpful error message (not stack traces)
//!   - Nonzero exit code
//!   - No panics
//!
//! Categories:
//!   1. Missing horus.toml — project commands in empty dirs
//!   2. Malformed horus.toml — garbage, missing fields, wrong types
//!   3. Invalid arguments — bad names, unknown flags, empty strings
//!   4. Destructive safety — clean/uninstall/remove preserve what they should
//!   5. Network failure — install/search/publish with unreachable registry
//!   6. No-panic sweep — every command's --help, every command in empty dir

use assert_cmd::cargo::cargo_bin_cmd;
use assert_cmd::Command;
use std::fs;
use tempfile::TempDir;

fn horus_cmd() -> Command {
    cargo_bin_cmd!("horus")
}

/// Assert the command produced no panic and no Rust backtrace.
fn assert_no_panic(output: &std::process::Output, context: &str) {
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        !stderr.contains("panicked") && !stderr.contains("RUST_BACKTRACE"),
        "{} should not panic or show backtrace, got: {}",
        context,
        stderr
    );
}

// ============================================================================
// Category 1: Missing horus.toml
// ============================================================================

#[test]
fn test_run_no_manifest() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("run")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "run without manifest");
}

#[test]
fn test_build_no_manifest() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("build")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "build without manifest");
}

#[test]
fn test_test_no_manifest() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("test")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "test without manifest");
}

#[test]
fn test_check_no_manifest() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("check")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "check without manifest");
}

#[test]
fn test_fmt_no_manifest() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("fmt")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "fmt without manifest");
}

#[test]
fn test_lint_no_manifest() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("lint")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "lint without manifest");
}

#[test]
fn test_doc_no_manifest() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("doc")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "doc without manifest");
}

#[test]
fn test_bench_no_manifest() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("bench")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "bench without manifest");
}

#[test]
fn test_add_no_manifest() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .args(["add", "serde"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "add without manifest");
    assert!(!output.status.success(), "add without manifest should fail");
}

#[test]
fn test_remove_no_manifest() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .args(["remove", "serde"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "remove without manifest");
    assert!(
        !output.status.success(),
        "remove without manifest should fail"
    );
}

#[test]
fn test_clean_no_manifest() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("clean")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "clean without manifest");
}

#[test]
fn test_publish_no_manifest() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("publish")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "publish without manifest");
    assert!(
        !output.status.success(),
        "publish without manifest should fail"
    );
}

#[test]
fn test_update_no_manifest() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("update")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "update without manifest");
}

#[test]
fn test_lock_no_manifest() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("lock")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "lock without manifest");
}

// ============================================================================
// Category 2: Malformed horus.toml
// ============================================================================

#[test]
fn test_run_invalid_toml() {
    let tmp = TempDir::new().unwrap();
    fs::write(tmp.path().join("horus.toml"), "{{{{garbage not toml!!!!").unwrap();
    let output = horus_cmd()
        .arg("run")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "run with garbage toml");
    assert!(
        !output.status.success(),
        "run with garbage toml should fail"
    );
}

#[test]
fn test_build_empty_toml() {
    let tmp = TempDir::new().unwrap();
    fs::write(tmp.path().join("horus.toml"), "").unwrap();
    let output = horus_cmd()
        .arg("build")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "build with empty toml");
}

#[test]
fn test_check_missing_name() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nversion = \"0.1.0\"\n",
    )
    .unwrap();
    let output = horus_cmd()
        .arg("check")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "check with missing name");
}

#[test]
fn test_add_to_corrupt_toml() {
    let tmp = TempDir::new().unwrap();
    fs::write(tmp.path().join("horus.toml"), "not valid toml {{{}}}").unwrap();
    let before = fs::read_to_string(tmp.path().join("horus.toml")).unwrap();
    let output = horus_cmd()
        .args(["add", "serde", "--source", "crates.io"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "add to corrupt toml");
    // Corrupt file should not be silently overwritten
    let after = fs::read_to_string(tmp.path().join("horus.toml")).unwrap();
    assert_eq!(
        before, after,
        "corrupt toml should not be silently overwritten on error"
    );
}

#[test]
fn test_build_missing_src() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"nosrc\"\nversion = \"0.1.0\"\n",
    )
    .unwrap();
    let output = horus_cmd()
        .arg("build")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "build with no src");
}

#[test]
fn test_fmt_invalid_toml() {
    let tmp = TempDir::new().unwrap();
    fs::write(tmp.path().join("horus.toml"), "}}}}invalid").unwrap();
    let output = horus_cmd()
        .arg("fmt")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "fmt with invalid toml");
}

// ============================================================================
// Category 3: Invalid arguments
// ============================================================================

#[test]
fn test_new_invalid_name_dots() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .args([
            "new",
            "../escape",
            "-r",
            "-o",
            &tmp.path().to_string_lossy(),
        ])
        .output()
        .unwrap();
    assert_no_panic(&output, "new with path traversal name");
}

#[test]
fn test_new_existing_directory() {
    let tmp = TempDir::new().unwrap();
    // Create project first
    horus_cmd()
        .args(["new", "exists", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    // Try to create same project again
    let output = horus_cmd()
        .args(["new", "exists", "-r", "-o", &tmp.path().to_string_lossy()])
        .output()
        .unwrap();
    assert_no_panic(&output, "new in existing directory");
}

#[test]
fn test_add_invalid_source_lists_valid() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"t\"\nversion = \"0.1.0\"\n",
    )
    .unwrap();
    let output = horus_cmd()
        .args(["add", "serde", "--source", "fake-source-xyz"])
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "add with invalid source");
    assert!(!output.status.success());
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    // Should suggest valid sources
    assert!(
        combined.contains("crates.io") || combined.contains("pypi") || combined.contains("source"),
        "invalid source should hint at valid options, got: {}",
        combined
    );
}

#[test]
fn test_install_empty_string() {
    let output = horus_cmd().args(["install", ""]).output().unwrap();
    assert_no_panic(&output, "install empty string");
}

#[test]
fn test_remove_nonexistent_dep() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "rm-test", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("rm-test");
    let toml_before = fs::read_to_string(proj.join("horus.toml")).unwrap();

    let output = horus_cmd()
        .args(["remove", "nonexistent-package-xyz"])
        .current_dir(&proj)
        .output()
        .unwrap();
    assert_no_panic(&output, "remove nonexistent dep");

    // horus.toml should be unchanged
    let toml_after = fs::read_to_string(proj.join("horus.toml")).unwrap();
    assert_eq!(
        toml_before, toml_after,
        "removing nonexistent dep should not change horus.toml"
    );
}

#[test]
fn test_invalid_subcommand() {
    let output = horus_cmd()
        .arg("totally-fake-command-xyz")
        .output()
        .unwrap();
    assert_no_panic(&output, "invalid subcommand");
    assert!(!output.status.success(), "invalid subcommand should fail");
}

#[test]
fn test_node_kill_nonexistent() {
    let output = horus_cmd()
        .args(["node", "kill", "fake-node-xyz-999"])
        .output()
        .unwrap();
    assert_no_panic(&output, "node kill nonexistent");
}

// ============================================================================
// Category 4: Destructive command safety
// ============================================================================

#[test]
fn test_clean_preserves_horus_toml() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args([
            "new",
            "clean-safe",
            "-r",
            "-o",
            &tmp.path().to_string_lossy(),
        ])
        .assert()
        .success();
    let proj = tmp.path().join("clean-safe");
    let toml_before = fs::read_to_string(proj.join("horus.toml")).unwrap();

    horus_cmd()
        .arg("clean")
        .current_dir(&proj)
        .output()
        .unwrap();

    assert!(
        proj.join("horus.toml").exists(),
        "clean should never delete horus.toml"
    );
    let toml_after = fs::read_to_string(proj.join("horus.toml")).unwrap();
    assert_eq!(
        toml_before, toml_after,
        "clean should not modify horus.toml"
    );
}

#[test]
fn test_clean_preserves_source() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args([
            "new",
            "clean-src",
            "-r",
            "-o",
            &tmp.path().to_string_lossy(),
        ])
        .assert()
        .success();
    let proj = tmp.path().join("clean-src");
    let main_before = fs::read_to_string(proj.join("src/main.rs")).unwrap();

    horus_cmd()
        .arg("clean")
        .current_dir(&proj)
        .output()
        .unwrap();

    assert!(
        proj.join("src/main.rs").exists(),
        "clean should never delete source files"
    );
    let main_after = fs::read_to_string(proj.join("src/main.rs")).unwrap();
    assert_eq!(
        main_before, main_after,
        "clean should not modify source files"
    );
}

#[test]
fn test_uninstall_nonexistent() {
    let output = horus_cmd()
        .args(["uninstall", "nonexistent-package-xyz-999"])
        .output()
        .unwrap();
    assert_no_panic(&output, "uninstall nonexistent");
    assert!(
        !output.status.success(),
        "uninstall nonexistent should fail"
    );
}

#[test]
fn test_remove_preserves_other_deps() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "rm-other", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("rm-other");

    // Add two deps
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

    // Remove one
    horus_cmd()
        .args(["remove", "serde"])
        .current_dir(&proj)
        .output()
        .unwrap();

    let toml = fs::read_to_string(proj.join("horus.toml")).unwrap();
    assert!(!toml.contains("serde"), "serde should be removed");
    assert!(toml.contains("anyhow"), "anyhow should be preserved");
}

#[test]
fn test_clean_in_empty_dir() {
    let tmp = TempDir::new().unwrap();
    let output = horus_cmd()
        .arg("clean")
        .current_dir(tmp.path())
        .output()
        .unwrap();
    assert_no_panic(&output, "clean in empty dir");
}

#[test]
fn test_unpublish_no_version() {
    let output = horus_cmd()
        .args(["unpublish", "some-pkg"])
        .output()
        .unwrap();
    assert_no_panic(&output, "unpublish without version");
}

#[test]
fn test_update_dry_run_no_changes() {
    let tmp = TempDir::new().unwrap();
    horus_cmd()
        .args(["new", "dry-test", "-r", "-o", &tmp.path().to_string_lossy()])
        .assert()
        .success();
    let proj = tmp.path().join("dry-test");
    let toml_before = fs::read_to_string(proj.join("horus.toml")).unwrap();

    horus_cmd()
        .args(["update", "--dry-run"])
        .current_dir(&proj)
        .output()
        .unwrap();

    let toml_after = fs::read_to_string(proj.join("horus.toml")).unwrap();
    assert_eq!(
        toml_before, toml_after,
        "dry-run should not modify horus.toml"
    );
}

// ============================================================================
// Category 5: Network failure graceful handling
// ============================================================================

#[test]
fn test_install_offline() {
    let output = horus_cmd()
        .args(["install", "fake-pkg-offline-test"])
        .env("HORUS_REGISTRY_URL", "http://127.0.0.1:1")
        .output()
        .unwrap();
    assert_no_panic(&output, "install offline");
    assert!(
        !output.status.success(),
        "install with unreachable registry should fail"
    );
}

#[test]
fn test_search_offline() {
    let output = horus_cmd()
        .args(["search", "robot"])
        .env("HORUS_REGISTRY_URL", "http://127.0.0.1:1")
        .output()
        .unwrap();
    assert_no_panic(&output, "search offline");
}

#[test]
fn test_publish_offline() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"offline-pub\"\nversion = \"0.1.0\"\n",
    )
    .unwrap();
    let output = horus_cmd()
        .arg("publish")
        .current_dir(tmp.path())
        .env("HORUS_REGISTRY_URL", "http://127.0.0.1:1")
        .output()
        .unwrap();
    assert_no_panic(&output, "publish offline");
    assert!(!output.status.success());
}

#[test]
fn test_info_offline() {
    let output = horus_cmd()
        .args(["info", "fake-pkg"])
        .env("HORUS_REGISTRY_URL", "http://127.0.0.1:1")
        .output()
        .unwrap();
    assert_no_panic(&output, "info offline");
}

#[test]
fn test_update_offline() {
    let tmp = TempDir::new().unwrap();
    fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"up\"\nversion = \"0.1.0\"\n",
    )
    .unwrap();
    let output = horus_cmd()
        .arg("update")
        .current_dir(tmp.path())
        .env("HORUS_REGISTRY_URL", "http://127.0.0.1:1")
        .output()
        .unwrap();
    assert_no_panic(&output, "update offline");
}

#[test]
fn test_list_offline() {
    let output = horus_cmd()
        .arg("list")
        .env("HORUS_REGISTRY_URL", "http://127.0.0.1:1")
        .output()
        .unwrap();
    assert_no_panic(&output, "list offline");
}

// ============================================================================
// Category 6: No-panic sweep
// ============================================================================

#[test]
fn test_every_help_flag_succeeds() {
    let commands = [
        "init",
        "new",
        "run",
        "build",
        "lock",
        "test",
        "check",
        "clean",
        "launch",
        "fmt",
        "lint",
        "doc",
        "bench",
        "update",
        "publish",
        "deploy",
        "doctor",
        "config",
        "migrate",
        "install",
        "uninstall",
        "search",
        "info",
        "add",
        "remove",
        "log",
    ];
    for cmd in &commands {
        let output = horus_cmd().args([cmd, "--help"]).output().unwrap();
        assert!(
            output.status.success(),
            "{} --help should succeed (exit code: {})",
            cmd,
            output.status
        );
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert!(
            !stderr.contains("panicked"),
            "{} --help should not panic",
            cmd
        );
    }
}

#[test]
fn test_subcommand_help_flags() {
    let subcommands = [
        &["topic", "--help"][..],
        &["node", "--help"],
        &["service", "--help"],
        &["action", "--help"],
        &["param", "--help"],
        &["frame", "--help"],
        &["msg", "--help"],
        &["blackbox", "--help"],
        &["record", "--help"],
        &["cache", "--help"],
        &["deps", "--help"],
        &["auth", "--help"],
        &["plugin", "--help"],
        &["self", "update", "--help"],
    ];
    for args in &subcommands {
        let output = horus_cmd().args(*args).output().unwrap();
        assert!(output.status.success(), "{:?} should succeed", args);
    }
}

#[test]
fn test_every_command_in_empty_dir() {
    let tmp = TempDir::new().unwrap();
    let commands = [
        "run", "build", "test", "check", "clean", "fmt", "lint", "doc", "bench", "update",
        "publish", "lock",
    ];
    for cmd in &commands {
        let output = horus_cmd()
            .arg(cmd)
            .current_dir(tmp.path())
            .output()
            .unwrap();
        assert_no_panic(&output, &format!("{} in empty dir", cmd));
    }
}

#[test]
fn test_version_always_works() {
    horus_cmd().arg("--version").assert().success();
}
