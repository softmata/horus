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

/// A project whose manifest has one hard error, one warning and two unknown
/// keys.
fn broken_project(dir: &std::path::Path) {
    std::fs::write(
        dir.join("horus.toml"),
        "[package]\n\
         name = \"jsonbot\"\n\
         version = \"not-a-version\"\n\
         langauge = \"rust\"\n\
         edition = \"9\"\n\
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
    assert_eq!(
        langauge["column"], 1,
        "the recommended schema asks for a column too: {langauge:#}"
    );
    assert_eq!(
        langauge["code"], "manifest.unknown-key",
        "a consumer filtering one class of finding cannot match on prose: {langauge:#}"
    );
    // A key that does nothing is an error as of 0.3.0, which is what the 0.2.x
    // warning said would happen.
    assert_eq!(langauge["severity"], "error");
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
        diags.iter().any(|d| d["severity"] == "warning"
            && d["message"].as_str().unwrap_or("").contains("edition")),
        "the unknown edition is a warning: {diags:#?}"
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
        v["diagnostics"]
            .as_array()
            .map(|a| a.is_empty())
            .unwrap_or(true),
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

// ─── The paths the first fix missed ─────────────────────────────────────────
//
// "diagnostics on every code path" held for the workspace manifest scan only.
// Everything below reported the problem in the terminal and shipped
// `"diagnostics": []` beside it, which is the same defect this file was
// created for, one path over.

fn check_json_args(dir: &std::path::Path, args: &[&str]) -> serde_json::Value {
    let out = Command::new(horus())
        .args(args)
        .current_dir(dir)
        .output()
        .expect("horus check must run");
    let stdout = String::from_utf8_lossy(&out.stdout);
    serde_json::from_str(&stdout)
        .unwrap_or_else(|e| panic!("--json must emit valid JSON on stdout ({e}):\n{stdout}"))
}

fn find_diag<'a>(v: &'a serde_json::Value, needle: &str) -> &'a serde_json::Value {
    v["diagnostics"]
        .as_array()
        .and_then(|a| {
            a.iter()
                .find(|d| d["message"].as_str().unwrap_or("").contains(needle))
        })
        .unwrap_or_else(|| panic!("no diagnostic mentioning {needle:?}:\n{v:#}"))
}

/// `horus check bad.py --json` -> `"diagnostics": []`, with the whole syntax
/// error sitting in the human-readable `error` string beside it.
#[test]
fn a_single_python_file_reports_its_syntax_error_as_a_diagnostic() {
    let tmp = tempfile::tempdir().unwrap();
    std::fs::write(tmp.path().join("bad.py"), "def f(:\n").unwrap();

    let v = check_json_args(tmp.path(), &["check", "bad.py", "--json"]);
    let d = find_diag(&v, "SyntaxError");
    assert_eq!(d["file"], "bad.py", "{d:#}");
    assert_eq!(
        d["line"], 1,
        "python names the line; it must survive: {d:#}"
    );
    assert_eq!(d["severity"], "error");
    assert_eq!(d["code"], "python.syntax");
}

/// Same hole, the Rust half.
#[test]
fn a_single_rust_file_reports_its_syntax_error_as_a_diagnostic() {
    let tmp = tempfile::tempdir().unwrap();
    std::fs::write(tmp.path().join("bad.rs"), "fn main( {\n").unwrap();

    let v = check_json_args(tmp.path(), &["check", "bad.rs", "--json"]);
    let diags = v["diagnostics"].as_array().expect("array");
    assert!(
        !diags.is_empty(),
        "the terminal named the error and --json shipped nothing:\n{v:#}"
    );
    assert_eq!(diags[0]["file"], "bad.rs", "{v:#}");
    assert_eq!(diags[0]["code"], "rust.syntax", "{v:#}");
}

/// Phase 2 put the *cargo directory* in `file` and no line at all, while
/// Phase 3 (Python) carried a real file and line — so an editor got a jump
/// target for one language and prose for the other.
#[test]
fn a_cargo_error_carries_the_source_file_not_the_project_directory() {
    let tmp = tempfile::tempdir().unwrap();
    std::fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"cargobot\"\nversion = \"0.1.0\"\n",
    )
    .unwrap();
    std::fs::write(
        tmp.path().join("Cargo.toml"),
        "[package]\nname = \"cargobot\"\nversion = \"0.1.0\"\nedition = \"2021\"\n",
    )
    .unwrap();
    std::fs::create_dir_all(tmp.path().join("src")).unwrap();
    std::fs::write(
        tmp.path().join("src/main.rs"),
        "fn main() { let x: u32 = \"not a number\"; }\n",
    )
    .unwrap();

    let v = check_json_args(tmp.path(), &["check", "--json"]);
    let d = find_diag(&v, "E0308");
    assert_eq!(
        d["file"], "src/main.rs",
        "the file the error is in, not the directory cargo ran in: {d:#}"
    );
    assert_eq!(d["line"], 1, "{d:#}");
    assert_eq!(d["column"], 26, "{d:#}");
    assert_eq!(d["code"], "rust.compile", "{d:#}");
}

/// A warning the terminal prints and `--json` drops is the same defect as an
/// error it drops.
#[test]
fn an_unresolved_python_import_reaches_the_json() {
    let tmp = tempfile::tempdir().unwrap();
    std::fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"importbot\"\nversion = \"0.1.0\"\n",
    )
    .unwrap();
    std::fs::write(
        tmp.path().join("main.py"),
        "import totally_missing_module_xyz\nprint(1)\n",
    )
    .unwrap();

    let v = check_json_args(tmp.path(), &["check", "--json"]);
    let d = find_diag(&v, "ModuleNotFoundError");
    assert_eq!(d["severity"], "warning", "{d:#}");
    assert_eq!(d["file"], "main.py", "{d:#}");
    assert_eq!(d["code"], "python.import", "{d:#}");
    // The message exists to name the module. It was printing the literal
    // placeholder `{e.name}` instead, because the script was written as if it
    // went through `format!`.
    assert!(
        d["message"]
            .as_str()
            .unwrap_or_default()
            .contains("totally_missing_module_xyz"),
        "the missing module's name is the whole point of the message: {d:#}"
    );
}

/// The finding's flagship case: one entry with three errors joined into its
/// message and no line, beside unknown-key entries that did carry lines.
#[test]
fn each_validation_error_is_its_own_diagnostic_with_a_position() {
    let tmp = tempfile::tempdir().unwrap();
    std::fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"Bad Name\"\nversion = \"not-a-version\"\n",
    )
    .unwrap();
    std::fs::create_dir_all(tmp.path().join("src")).unwrap();
    std::fs::write(tmp.path().join("src/main.rs"), "fn main(){}\n").unwrap();

    let v = check_json_args(tmp.path(), &["check", "--json"]);
    let name = find_diag(&v, "must contain only lowercase");
    let version = find_diag(&v, "not valid semver");

    assert_eq!(name["line"], 2, "the name is on line 2: {name:#}");
    assert_eq!(version["line"], 3, "the version is on line 3: {version:#}");
    assert_eq!(name["column"], 1, "{name:#}");
    assert_eq!(name["code"], "manifest.invalid", "{name:#}");
    assert!(
        !name["message"]
            .as_str()
            .unwrap_or_default()
            .contains("not valid semver"),
        "two problems must be two entries, not one joined string: {name:#}"
    );
}

/// `check` has no C++ phase. Reporting "All checks passed!" over a project
/// whose only source is C++ reads as "your C++ is fine" rather than "your C++
/// was never looked at".
#[test]
fn a_cpp_project_is_not_told_its_sources_passed() {
    let tmp = tempfile::tempdir().unwrap();
    std::fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"cppbot\"\nversion = \"0.1.0\"\n",
    )
    .unwrap();
    std::fs::create_dir_all(tmp.path().join("src")).unwrap();
    std::fs::write(tmp.path().join("src/main.cpp"), "int main( {\n").unwrap();

    let out = Command::new(horus())
        .arg("check")
        .current_dir(tmp.path())
        .output()
        .expect("horus check must run");
    let text = String::from_utf8_lossy(&out.stdout).into_owned();

    assert!(
        text.contains("not syntax-checked") || text.contains("not among them"),
        "the C++ sources were never checked and the output must say so:\n{text}"
    );
}
