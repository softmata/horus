//! `horus check --json` must carry the findings, not just how many there were.
//!
//! Machine-readable output existed but was strictly *less* informative than the
//! terminal:
//!
//! ```text
//! $ horus check              # human
//!   1. Version 'not-a-version' is not valid semver (expected e.g., '0.1.0')
//!
//! $ horus check --json       # machine
//! { "error": "Configuration error: 1 error(s) found", "valid": false }
//! ```
//!
//! CI and editor integrations got a boolean and a count, which defeats the
//! reason the flag exists. The findings were printed to a stdout that `--json`
//! deliberately suppresses, and then discarded.
//!
//! Run: `cargo test -p horus_manager --test check_json_contract`

use std::process::Command;

fn horus() -> &'static str {
    env!("CARGO_BIN_EXE_horus")
}

/// A project whose manifest has one hard error and several unknown keys.
fn broken_project(dir: &std::path::Path) {
    std::fs::write(
        dir.join("horus.toml"),
        "[package]\n\
         name = \"jsonbot\"\n\
         version = \"not-a-version\"\n\
         langauge = \"rust\"\n\
         \n\
         [bogus]\n\
         x = 1\n",
    )
    .unwrap();
    std::fs::create_dir_all(dir.join("src")).unwrap();
    std::fs::write(dir.join("src/main.rs"), "fn main(){}\n").unwrap();
}

fn check_json(dir: &std::path::Path) -> serde_json::Value {
    let out = Command::new(horus())
        .args(["check", "--json"])
        .current_dir(dir)
        .output()
        .expect("horus check --json must run");

    let stdout = String::from_utf8_lossy(&out.stdout);
    serde_json::from_str(&stdout)
        .unwrap_or_else(|e| panic!("--json must emit valid JSON on stdout ({e}):\n{stdout}"))
}

#[test]
fn json_output_carries_the_diagnostics() {
    let tmp = tempfile::tempdir().unwrap();
    broken_project(tmp.path());
    let v = check_json(tmp.path());

    let diags = v
        .get("diagnostics")
        .and_then(|d| d.as_array())
        .expect("--json must include a `diagnostics` array");

    assert!(
        !diags.is_empty(),
        "the human output names the problems; --json must too: {v:#}"
    );
    assert_eq!(v["valid"], serde_json::Value::Bool(false));
}

/// The semver error is the one the human output shows; it must be present.
#[test]
fn the_actual_error_is_reported_not_just_a_count() {
    let tmp = tempfile::tempdir().unwrap();
    broken_project(tmp.path());
    let v = check_json(tmp.path());

    let text = v["diagnostics"].to_string();
    assert!(
        text.contains("not valid semver"),
        "the specific failure must survive into --json, got: {text}"
    );
}

/// Line numbers are what make this usable from an editor.
#[test]
fn unknown_keys_carry_file_and_line() {
    let tmp = tempfile::tempdir().unwrap();
    broken_project(tmp.path());
    let v = check_json(tmp.path());

    let diags = v["diagnostics"].as_array().unwrap();
    let langauge = diags
        .iter()
        .find(|d| d["message"].as_str().unwrap_or("").contains("langauge"))
        .expect("the misspelled key should be reported");

    assert_eq!(langauge["file"], "horus.toml");
    assert_eq!(
        langauge["line"], 4,
        "an editor needs the line to place the squiggle: {langauge:#}"
    );
    assert_eq!(langauge["severity"], "warning");
}

/// Errors and warnings must be distinguishable, or a caller cannot gate on
/// severity.
#[test]
fn severity_separates_errors_from_warnings() {
    let tmp = tempfile::tempdir().unwrap();
    broken_project(tmp.path());
    let v = check_json(tmp.path());
    let diags = v["diagnostics"].as_array().unwrap();

    assert!(
        diags.iter().any(|d| d["severity"] == "error"),
        "the invalid version is an error: {diags:#?}"
    );
    assert!(
        diags.iter().any(|d| d["severity"] == "warning"),
        "the unknown keys are warnings: {diags:#?}"
    );
}

/// A clean project reports valid with nothing to say.
#[test]
fn a_valid_project_reports_no_diagnostics() {
    let tmp = tempfile::tempdir().unwrap();
    std::fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"cleanbot\"\nversion = \"0.1.0\"\n",
    )
    .unwrap();
    std::fs::create_dir_all(tmp.path().join("src")).unwrap();
    std::fs::write(tmp.path().join("src/main.rs"), "fn main(){}\n").unwrap();

    let v = check_json(tmp.path());
    assert_eq!(v["valid"], serde_json::Value::Bool(true), "{v:#}");
    assert!(
        v["diagnostics"].as_array().map(|a| a.is_empty()).unwrap_or(true),
        "a clean project should report nothing: {v:#}"
    );
}

/// stdout must stay pure JSON so `horus check --json | jq` works. The prose
/// error goes to stderr, which is already correct and must not regress.
#[test]
fn stdout_is_pure_json() {
    let tmp = tempfile::tempdir().unwrap();
    broken_project(tmp.path());

    let out = Command::new(horus())
        .args(["check", "--json"])
        .current_dir(tmp.path())
        .output()
        .unwrap();

    let stdout = String::from_utf8_lossy(&out.stdout);
    serde_json::from_str::<serde_json::Value>(&stdout)
        .unwrap_or_else(|e| panic!("stdout must parse as JSON ({e}):\n{stdout}"));
}
