#![allow(dead_code)]
//! CLI Error Handling Tests
//!
//! Tests for CLI error paths — what happens when commands receive bad input.
//! Verifies that the CLI produces graceful error messages rather than panics
//! or raw stack traces, and that the error wrapper correctly rewrites
//! internal `.horus/` paths to user-facing equivalents.

use assert_cmd::Command;
use predicates::prelude::*;

/// Helper to get the CLI command.
fn horus_cmd() -> Command {
    assert_cmd::cargo::cargo_bin_cmd!("horus")
}

// ============================================================================
// Test 6: msg list with nonexistent HORUS_SOURCE_DIR
// ============================================================================

/// INTENT: "Pointing msg list at a nonexistent source directory produces
/// a graceful error, not a panic or stack trace."
///
/// User guarantee: a new user who hasn't set up HORUS_SOURCE_DIR (or set
/// it to a typo'd path) should get a helpful error message telling them
/// what went wrong — not a Rust panic with a backtrace that scares them
/// away from the framework.
#[test]
fn test_cli_msg_list_with_nonexistent_dir() {
    let result = horus_cmd()
        .args(["msg", "list"])
        .env(
            "HORUS_SOURCE_DIR",
            "/tmp/horus_definitely_does_not_exist_9999",
        )
        .env_remove("HORUS_SOURCE")
        .assert();

    // The command should either succeed (if fallback paths find messages)
    // or fail gracefully with a meaningful error — never a panic
    let output = result.get_output().clone();
    let stderr = String::from_utf8_lossy(&output.stderr);
    let stdout = String::from_utf8_lossy(&output.stdout);

    // Must NOT contain panic indicators
    assert!(
        !stderr.contains("panicked at"),
        "CLI should not panic on nonexistent source dir, stderr: {}",
        stderr
    );
    assert!(
        !stderr.contains("RUST_BACKTRACE"),
        "CLI should not suggest RUST_BACKTRACE on a user error, stderr: {}",
        stderr
    );

    // If it failed, the error message should be informative
    if !output.status.success() {
        let combined = format!("{}{}", stdout, stderr);
        assert!(
            combined.contains("not found")
                || combined.contains("Could not")
                || combined.contains("error")
                || combined.contains("Error"),
            "Error message should be descriptive, got stdout: {} stderr: {}",
            stdout,
            stderr
        );
    }
}

// ============================================================================
// Test 7: msg info with special characters (path traversal attempt)
// ============================================================================

/// INTENT: "Requesting info for a type name with special characters is
/// gracefully rejected."
///
/// User guarantee: even if a user accidentally (or maliciously) passes
/// a path-traversal-style string as a message type name, the CLI must
/// handle it without crashing, without accessing unexpected files, and
/// with a clear "not found" error.
#[test]
fn test_cli_msg_info_with_special_chars() {
    horus_cmd()
        .args(["msg", "info", "../../etc/passwd"])
        .assert()
        .failure()
        .stderr(predicate::str::contains("panicked at").not());

    // Also test with other adversarial inputs
    horus_cmd()
        .args(["msg", "info", "<script>alert(1)</script>"])
        .assert()
        .failure()
        .stderr(predicate::str::contains("panicked at").not());
}

// ============================================================================
// Test 8: Error wrapper strips .horus/ paths
// ============================================================================

/// INTENT: "Error messages containing '.horus/' paths get cleaned up
/// by the error wrapper."
///
/// User guarantee: users should never see raw `.horus/Cargo.toml` in
/// error output — those are generated files that the user should not
/// edit. The error wrapper must rewrite them to point to `horus.toml`,
/// which is the actual source of truth the user should fix.
#[test]
fn test_cli_error_wrapper_strips_horus_paths() {
    use horus_manager::error_wrapper::rewrite_horus_paths;

    // .horus/Cargo.toml -> horus.toml (generated Cargo.toml)
    let input = "error: failed to parse .horus/Cargo.toml at line 5";
    let output = rewrite_horus_paths(input);
    assert!(
        !output.contains(".horus/Cargo.toml"),
        "Should not contain raw .horus/Cargo.toml, got: {}",
        output
    );
    assert!(
        output.contains("horus.toml"),
        "Should reference horus.toml as source of truth, got: {}",
        output
    );

    // .horus/pyproject.toml -> horus.toml (generated pyproject.toml)
    let input = "error in .horus/pyproject.toml: invalid key";
    let output = rewrite_horus_paths(input);
    assert!(
        !output.contains(".horus/pyproject.toml"),
        "Should rewrite .horus/pyproject.toml, got: {}",
        output
    );

    // .horus/CMakeLists.txt -> horus.toml (generated CMakeLists.txt)
    let input = "CMake Error at .horus/CMakeLists.txt:10";
    let output = rewrite_horus_paths(input);
    assert!(
        !output.contains(".horus/CMakeLists.txt"),
        "Should rewrite .horus/CMakeLists.txt, got: {}",
        output
    );

    // .horus/target/ -> build/
    let input = "error in .horus/target/debug/build/foo-hash/output";
    let output = rewrite_horus_paths(input);
    assert!(
        !output.contains(".horus/target/"),
        "Should rewrite .horus/target/ to build/, got: {}",
        output
    );
    assert!(
        output.contains("build/"),
        "Should contain rewritten build/ path, got: {}",
        output
    );

    // .horus/cpp-build/ -> build/
    let input = "linking failed: .horus/cpp-build/CMakeFiles/foo.o";
    let output = rewrite_horus_paths(input);
    assert!(
        !output.contains(".horus/cpp-build/"),
        "Should rewrite .horus/cpp-build/ to build/, got: {}",
        output
    );
}

// ============================================================================
// Test 9: Error wrapper preserves useful info
// ============================================================================

/// INTENT: "Error wrapper keeps the actual error message (doesn't
/// over-strip)."
///
/// User guarantee: while rewriting `.horus/` paths is important, the
/// wrapper must not accidentally remove or corrupt the actual error
/// message, line numbers, or other diagnostic information. An error
/// that says "error at line 5: missing field 'name'" must still say
/// that after rewriting — only the file path changes.
#[test]
fn test_cli_error_wrapper_preserves_useful_info() {
    use horus_manager::error_wrapper::rewrite_horus_paths;

    // Error message without .horus/ paths should pass through unchanged
    let input = "error[E0308]: mismatched types in src/main.rs:42";
    let output = rewrite_horus_paths(input);
    assert_eq!(
        output, input,
        "Messages without .horus/ paths should pass through unchanged"
    );

    // Error message with .horus/ path should preserve the rest of the message
    let input = "error: failed to parse .horus/Cargo.toml at line 5: missing field 'name'";
    let output = rewrite_horus_paths(input);
    assert!(
        output.contains("failed to parse"),
        "Should preserve 'failed to parse', got: {}",
        output
    );
    assert!(
        output.contains("at line 5"),
        "Should preserve 'at line 5', got: {}",
        output
    );
    assert!(
        output.contains("missing field 'name'"),
        "Should preserve the actual error detail, got: {}",
        output
    );

    // Multiple .horus/ paths in the same message should all be rewritten
    let input = "error: .horus/Cargo.toml conflicts with .horus/pyproject.toml";
    let output = rewrite_horus_paths(input);
    assert!(
        !output.contains(".horus/Cargo.toml"),
        "First .horus/ path should be rewritten"
    );
    assert!(
        !output.contains(".horus/pyproject.toml"),
        "Second .horus/ path should be rewritten"
    );
    assert!(
        output.contains("conflicts with"),
        "Should preserve 'conflicts with' between the two paths, got: {}",
        output
    );
}
