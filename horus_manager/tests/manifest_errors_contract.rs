//! A manifest that fails to load must say what is wrong and where.
//!
//! Every failure — a missing required key and an unclosed string alike —
//! surfaced as the same eight words:
//!
//! ```text
//! > horus.toml
//!     x TOML parse error: Failed to parse ./horus.toml
//! ```
//!
//! No line, no column, no field name, and the wrong class of error: a file that
//! omits `name` parsed perfectly well. Because the two cases were worded
//! identically, a reader could not tell whether to look for a syntax mistake or
//! a missing key, and `horus doctor` printed the same sentence for both with an
//! empty `details` list.
//!
//! `horus check` was fixed first; `doctor`, `config`, and both `--json`
//! surfaces still discarded the diagnostic. These tests pin all of them.
//!
//! Run: `cargo test -p horus_manager --test manifest_errors_contract`

use std::path::Path;
use std::process::Command;

use horus_manager::manifest::{HorusManifest, ManifestErrorKind};

fn horus() -> &'static str {
    env!("CARGO_BIN_EXE_horus")
}

/// Parse `content` as a manifest and return the typed failure.
fn parse_err(content: &str) -> horus_manager::manifest::ManifestError {
    let err = HorusManifest::parse_str(content, Path::new("horus.toml"))
        .expect_err("this fixture must fail to parse");
    err.downcast_ref::<horus_manager::manifest::ManifestError>()
        .expect("manifest failures must carry a ManifestError, not a bare string")
        .clone()
}

fn project(dir: &Path, manifest: &str) {
    std::fs::create_dir_all(dir).expect("mkdir");
    std::fs::write(dir.join("horus.toml"), manifest).expect("write manifest");
}

// ─── Classification ─────────────────────────────────────────────────────────

/// The distinction the original message erased. A missing key is not a parse
/// error, and calling it one sends the reader hunting for a syntax mistake that
/// is not in the file.
#[test]
fn a_missing_key_is_not_reported_as_a_syntax_error() {
    let e = parse_err("[package]\nversion = \"0.1.0\"\n");
    assert!(
        matches!(e.kind, ManifestErrorKind::MissingField { .. }),
        "expected MissingField, got {:?}",
        e.kind
    );
    let text = e.to_string();
    assert!(
        text.contains("missing field `name`"),
        "the message must name the absent field: {text}"
    );
    assert!(
        text.contains("valid TOML"),
        "the message must say the file parsed, or the reader looks for the \
         wrong kind of mistake: {text}"
    );
}

#[test]
fn malformed_toml_is_reported_as_a_syntax_error() {
    let e = parse_err("[package]\nname = \"broken\nversion = \"0.1.0\"\n");
    assert!(
        matches!(e.kind, ManifestErrorKind::Syntax),
        "expected Syntax, got {:?}",
        e.kind
    );
    assert!(
        !e.to_string().contains("valid TOML"),
        "this file is NOT valid TOML; saying so would be worse than saying nothing"
    );
}

/// The two must not render identically — that was the whole defect.
#[test]
fn the_two_failure_classes_read_differently() {
    let missing = parse_err("[package]\nversion = \"0.1.0\"\n").to_string();
    let broken = parse_err("[package]\nname = \"broken\nversion = \"0.1.0\"\n").to_string();
    assert_ne!(missing, broken);
}

#[test]
fn a_wrong_type_is_a_schema_error_not_a_parse_error() {
    let e = parse_err("[package]\nname = \"x\"\nversion = 1\n");
    assert!(
        matches!(e.kind, ManifestErrorKind::Schema),
        "expected Schema, got {:?}: {e}",
        e.kind
    );
}

// ─── Position ───────────────────────────────────────────────────────────────

/// The span must survive, and must point at the offending table rather than at
/// the top of the file.
#[test]
fn the_error_carries_the_line_of_the_table_not_line_one() {
    let e = parse_err(
        "# a comment\n[dependencies]\nserde = \"1.0\"\n\n[package]\nversion = \"0.1.0\"\n",
    );
    assert_eq!(e.line, Some(5), "[package] is on line 5: {e}");
    assert_eq!(e.col, Some(1));
    assert!(e.to_string().contains("horus.toml:5:1"), "{e}");
}

#[test]
fn a_syntax_error_carries_the_column_of_the_mistake() {
    let e = parse_err("[package]\nname = \"broken\nversion = \"0.1.0\"\n");
    assert_eq!(e.line, Some(2), "{e}");
    assert!(e.col.is_some(), "{e}");
}

// ─── Which `name`? ──────────────────────────────────────────────────────────

/// `[package]` and `[robot]` both require `name`, and `toml` reports both as
/// ``missing field `name` `` — it suppresses its own `in `...`` hint whenever a
/// span is available, and `keys` has no public accessor. Without the table the
/// two are indistinguishable.
#[test]
fn the_error_says_which_table_the_missing_key_belongs_to() {
    let pkg = parse_err("[package]\nversion = \"0.1.0\"\n");
    assert!(pkg.to_string().contains("in table [package]"), "{pkg}");

    let robot =
        parse_err("[package]\nname=\"r\"\nversion=\"0.1.0\"\n\n[robot]\nmodel = \"diffbot\"\n");
    assert!(
        robot.to_string().contains("in table [robot]"),
        "a missing [robot].name must not read as a missing [package].name: {robot}"
    );
}

/// Where the span is not a table header, decline to name one. Guessing would be
/// a fabrication, and `None` reads correctly as "cannot tell".
#[test]
fn no_table_is_invented_when_the_span_is_not_a_header() {
    for src in [
        "package = { version = \"0.1.0\" }\n", // inline table
        "package.version = \"0.1.0\"\n",       // dotted key
    ] {
        let e = parse_err(src);
        match &e.kind {
            ManifestErrorKind::MissingField { table, .. } => assert!(
                table.is_none(),
                "span is not a `[header]`, so no table should be claimed; got {table:?} for {src:?}"
            ),
            other => panic!("expected MissingField for {src:?}, got {other:?}"),
        }
        assert!(!e.to_string().contains("in table"), "{e}");
    }
}

// ─── doctor ─────────────────────────────────────────────────────────────────

/// The reported symptom, at its source: doctor collapsed every load failure
/// into one hardcoded sentence with no details.
#[test]
fn doctor_does_not_print_the_bare_failed_to_parse_sentence() {
    let tmp = tempfile::tempdir().expect("tempdir");
    project(tmp.path(), "[package]\nversion = \"0.1.0\"\n");

    let out = Command::new(horus())
        .arg("doctor")
        .current_dir(tmp.path())
        .output()
        .expect("horus doctor must run");
    let text =
        String::from_utf8_lossy(&out.stdout).into_owned() + &String::from_utf8_lossy(&out.stderr);

    assert!(
        !text.contains("Failed to parse horus.toml"),
        "doctor still prints the sentence that names neither the field nor the \
         line:\n{text}"
    );
    assert!(
        text.contains("missing a required field"),
        "doctor must say which class of problem this is:\n{text}"
    );
}

/// The instance that only an adversarial pass found. `find_and_load_from`
/// searches upward; when the load *failed*, the root fell back to the starting
/// directory, so the "does horus.toml exist" test ran against the wrong
/// directory and reported the file absent. Running any command from a
/// subdirectory of a project with a broken manifest denied the project existed.
#[test]
fn doctor_in_a_subdirectory_does_not_claim_the_manifest_is_absent() {
    let tmp = tempfile::tempdir().expect("tempdir");
    project(
        tmp.path(),
        "[package]\nname = \"broken\nversion = \"0.1.0\"\n",
    );
    let sub = tmp.path().join("src").join("nested");
    std::fs::create_dir_all(&sub).expect("mkdir sub");

    let out = Command::new(horus())
        .arg("doctor")
        .current_dir(&sub)
        .output()
        .expect("horus doctor must run");
    let text =
        String::from_utf8_lossy(&out.stdout).into_owned() + &String::from_utf8_lossy(&out.stderr);

    assert!(
        !text.contains("No horus.toml found"),
        "the manifest is one directory up and it is broken, not absent:\n{text}"
    );
    assert!(
        text.contains("not valid TOML"),
        "expected the real diagnosis:\n{text}"
    );
}

/// `check_drivers` loaded `./horus.toml` itself instead of using the context,
/// so it reported "No horus.toml found" from any subdirectory — even of a
/// perfectly valid project.
#[test]
fn the_hardware_check_finds_the_manifest_from_a_subdirectory() {
    let tmp = tempfile::tempdir().expect("tempdir");
    project(
        tmp.path(),
        "[package]\nname = \"ok\"\nversion = \"0.1.0\"\n\n[hardware]\nimu = { port = \"/dev/ttyUSB0\" }\n",
    );
    let sub = tmp.path().join("src");
    std::fs::create_dir_all(&sub).expect("mkdir sub");

    let out = Command::new(horus())
        .arg("doctor")
        .current_dir(&sub)
        .output()
        .expect("horus doctor must run");
    let text = String::from_utf8_lossy(&out.stdout).into_owned();

    let hardware = text
        .lines()
        .find(|l| l.contains("Hardware"))
        .unwrap_or_default()
        .to_string();
    assert!(
        !hardware.contains("No horus.toml found"),
        "the manifest is in the parent directory and is valid:\n{text}"
    );
}

// ─── config ─────────────────────────────────────────────────────────────────

#[test]
fn config_list_reports_the_position_of_a_syntax_error() {
    let tmp = tempfile::tempdir().expect("tempdir");
    project(
        tmp.path(),
        "[package]\nname = \"broken\nversion = \"0.1.0\"\n",
    );

    let out = Command::new(horus())
        .args(["config", "list"])
        .current_dir(tmp.path())
        .output()
        .expect("horus config list must run");
    let text = String::from_utf8_lossy(&out.stderr).into_owned();

    assert!(
        !text.contains("Failed to parse horus.toml"),
        "config discarded toml_edit's span:\n{text}"
    );
    assert!(
        text.contains("horus.toml:2:"),
        "expected a line:column:\n{text}"
    );
}

/// Syntactically fine is not the same as usable. Listing a manifest `horus
/// build` will reject, and exiting 0, tells the user nothing is wrong.
#[test]
fn config_list_warns_when_the_manifest_would_fail_validation() {
    let tmp = tempfile::tempdir().expect("tempdir");
    project(tmp.path(), "[package]\nversion = \"0.1.0\"\n");

    let out = Command::new(horus())
        .args(["config", "list"])
        .current_dir(tmp.path())
        .output()
        .expect("horus config list must run");
    let err = String::from_utf8_lossy(&out.stderr);

    assert!(
        err.contains("missing field `name`"),
        "expected a warning about the missing key:\n{err}"
    );
    assert!(
        out.status.success(),
        "must warn, not fail — `config set` is how the manifest gets repaired"
    );
}

/// The repair path. `config` must keep operating at the syntax level so a
/// manifest that fails *typed* validation can still be fixed with the tool.
#[test]
fn config_set_still_repairs_a_manifest_that_fails_validation() {
    let tmp = tempfile::tempdir().expect("tempdir");
    project(tmp.path(), "[package]\nversion = \"0.1.0\"\n");

    let out = Command::new(horus())
        .args(["config", "set", "package.name", "my-robot"])
        .current_dir(tmp.path())
        .output()
        .expect("horus config set must run");
    assert!(
        out.status.success(),
        "config set must work on an invalid manifest: {}",
        String::from_utf8_lossy(&out.stderr)
    );

    let after = std::fs::read_to_string(tmp.path().join("horus.toml")).expect("read back");
    assert!(after.contains("my-robot"), "{after}");
    assert!(
        HorusManifest::parse_str(&after, Path::new("horus.toml")).is_ok(),
        "the manifest should now load: {after}"
    );
}

// ─── machine-readable surfaces ──────────────────────────────────────────────

/// `check --json` emitted `"diagnostics": []` for every manifest failure, so
/// the position the human output had just computed reached no tool.
#[test]
fn check_json_carries_the_diagnostic() {
    let tmp = tempfile::tempdir().expect("tempdir");
    project(
        tmp.path(),
        "[package]\nname = \"broken\nversion = \"0.1.0\"\n",
    );

    let out = Command::new(horus())
        .args(["check", "--json"])
        .current_dir(tmp.path())
        .output()
        .expect("horus check --json must run");
    let stdout = String::from_utf8_lossy(&out.stdout);
    let json: serde_json::Value = serde_json::from_str(&stdout)
        .unwrap_or_else(|e| panic!("check --json must emit JSON ({e}):\n{stdout}"));

    let diags = json["diagnostics"]
        .as_array()
        .expect("diagnostics must be an array");
    assert!(
        !diags.is_empty(),
        "a manifest that fails to load must produce a diagnostic:\n{stdout}"
    );
    assert_eq!(
        diags[0]["line"].as_u64(),
        Some(2),
        "the diagnostic must carry the line:\n{stdout}"
    );
}

#[test]
fn doctor_json_carries_the_details() {
    let tmp = tempfile::tempdir().expect("tempdir");
    project(tmp.path(), "[package]\nversion = \"0.1.0\"\n");

    let out = Command::new(horus())
        .args(["doctor", "--json"])
        .current_dir(tmp.path())
        .output()
        .expect("horus doctor --json must run");
    let stdout = String::from_utf8_lossy(&out.stdout);
    let json: serde_json::Value = serde_json::from_str(&stdout)
        .unwrap_or_else(|e| panic!("doctor --json must emit JSON ({e}):\n{stdout}"));

    let checks = json
        .as_array()
        .cloned()
        .or_else(|| json["checks"].as_array().cloned())
        .unwrap_or_default();
    let manifest = checks
        .iter()
        .find(|c| c["category"] == "Manifest")
        .unwrap_or_else(|| panic!("no Manifest check in:\n{stdout}"));

    let details = manifest["details"].as_array().expect("details array");
    assert!(
        !details.is_empty(),
        "doctor --json emitted an empty details list, discarding the whole \
         diagnostic:\n{stdout}"
    );
    assert!(
        details.iter().any(|d| d
            .as_str()
            .is_some_and(|s| s.contains("missing field `name`"))),
        "details must carry the field name:\n{stdout}"
    );
}

// ─── documented behaviour ───────────────────────────────────────────────────

/// `#[serde(default)] pub package` means an *entirely absent* `[package]` table
/// is not an error at all — it deserializes to empty strings. Only a table that
/// is present and incomplete reaches the missing-field path. That asymmetry is
/// surprising enough to pin, so a change to it is deliberate.
#[test]
fn an_absent_package_table_defaults_rather_than_failing() {
    let m = HorusManifest::parse_str("[dependencies]\n", Path::new("horus.toml"))
        .expect("an absent [package] is accepted today");
    assert_eq!(m.package.name, "");
    assert_eq!(m.package.version, "0.0.0");
}

// ─── `--json` must say what the human output says (CFG-4) ───────────────────
//
// `horus check --json` exists so a tool can consume the findings. It reported
// `"diagnostics": []` while the terminal listed the errors by name — the
// machine-readable surface was the one that carried nothing.
//
// Fixed in three places, because there are three: the workspace manifest scan,
// the per-language phases, and the single-file path. I fixed the first and
// reported the finding closed; these pin all three.

fn check_json(dir: &Path, args: &[&str]) -> serde_json::Value {
    let out = Command::new(horus())
        .args(args)
        .current_dir(dir)
        .output()
        .expect("horus check must run");
    let stdout = String::from_utf8_lossy(&out.stdout);
    serde_json::from_str(&stdout)
        .unwrap_or_else(|e| panic!("check --json must emit JSON ({e}):\n{stdout}"))
}

fn diagnostics(v: &serde_json::Value) -> Vec<String> {
    v["diagnostics"]
        .as_array()
        .map(|a| {
            a.iter()
                .map(|d| d["message"].as_str().unwrap_or("").to_string())
                .collect()
        })
        .unwrap_or_default()
}

#[test]
fn check_json_reports_a_python_syntax_error() {
    let tmp = tempfile::tempdir().expect("tempdir");
    std::fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"demo\"\nversion = \"0.1.0\"\n",
    )
    .expect("write manifest");
    std::fs::write(tmp.path().join("main.py"), "def broken(:\n").expect("write py");

    let json = check_json(tmp.path(), &["check", "--json"]);
    let msgs = diagnostics(&json);
    assert!(
        msgs.iter().any(|m| m.contains("main.py")),
        "the human output names the file and line; --json carried nothing:\n{msgs:?}"
    );
    let empty = vec![];
    let lines: Vec<u64> = json["diagnostics"]
        .as_array()
        .unwrap_or(&empty)
        .iter()
        .filter_map(|d| d["line"].as_u64())
        .collect();
    assert!(
        lines.contains(&1),
        "Python reports `File \"x.py\", line N`; the number should be a field:\n{json}"
    );
}

#[test]
fn check_json_reports_single_file_validation() {
    let tmp = tempfile::tempdir().expect("tempdir");
    std::fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"x\"\nversion = \"0.1.0\"\n",
    )
    .expect("write");

    let json = check_json(tmp.path(), &["check", "horus.toml", "--json"]);
    let msgs = diagnostics(&json);
    assert!(
        !msgs.is_empty(),
        "`horus check <file> --json` reported no diagnostics while the human \
         output listed the failure:\n{json}"
    );
    assert!(
        msgs.iter().any(|m| m.contains("2-64 characters")),
        "expected the validation failure the human output prints:\n{msgs:?}"
    );
}

/// Warnings count too: a tool that only sees errors cannot show what `horus
/// check` shows.
#[test]
fn check_json_carries_warnings_not_only_errors() {
    let tmp = tempfile::tempdir().expect("tempdir");
    std::fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"demo\"\nversion = \"0.1.0\"\n",
    )
    .expect("write");

    let json = check_json(tmp.path(), &["check", "horus.toml", "--json"]);
    let empty = vec![];
    let severities: Vec<String> = json["diagnostics"]
        .as_array()
        .unwrap_or(&empty)
        .iter()
        .map(|d| d["severity"].as_str().unwrap_or("").to_string())
        .collect();
    assert!(
        severities.iter().any(|s| s == "warning"),
        "a project with no license produces a warning in the human output:\n{json}"
    );
}

// ─── The single-file path (`horus check <file>`) — CFG-3 ────────────────────
//
// The library produced a correctly classified error all along; the *directory*
// path printed it and the *single-file* path re-wrapped it:
//
//   1. Failed to parse manifest: horus.toml:1:1: missing field `name` ...
//      The file is valid TOML — a required key is absent.
//
// A sentence announcing a parse error over a message whose own next line says
// the file parsed. Both classes carried the identical label, so on that path
// they still read the same — the defect the finding is about, on the sibling
// path nobody re-ran.

/// The exact string that made the two classes indistinguishable.
#[test]
fn checking_a_file_does_not_call_a_missing_key_a_parse_failure() {
    let tmp = tempfile::tempdir().expect("tempdir");
    project(tmp.path(), "[package]\nversion = \"0.1.0\"\n");

    let out = Command::new(horus())
        .args(["check", "horus.toml"])
        .current_dir(tmp.path())
        .output()
        .expect("horus check must run");
    let text = String::from_utf8_lossy(&out.stdout).into_owned();

    assert!(
        !text.contains("Failed to parse manifest"),
        "the file parsed; only a key is absent:\n{text}"
    );
    assert!(
        text.contains("missing field `name`"),
        "the field name must survive:\n{text}"
    );
    assert!(
        text.contains("horus.toml:1:1"),
        "and so must the position:\n{text}"
    );
    assert!(
        text.contains("The file is valid TOML"),
        "the explanation that distinguishes the class must be there:\n{text}"
    );
}

/// The other half of "distinct": the syntax case must not read like the
/// missing-key case on this path either.
#[test]
fn the_two_failure_classes_read_differently_on_the_file_path() {
    let tmp = tempfile::tempdir().expect("tempdir");
    std::fs::write(
        tmp.path().join("missing.toml"),
        "[package]\nversion = \"0.1.0\"\n",
    )
    .expect("write");
    std::fs::write(tmp.path().join("broken.toml"), "[package\nname = \"x\"\n").expect("write");

    let read = |file: &str| {
        let out = Command::new(horus())
            .args(["check", file])
            .current_dir(tmp.path())
            .output()
            .expect("horus check must run");
        String::from_utf8_lossy(&out.stdout).into_owned()
    };

    let missing = read("missing.toml");
    let broken = read("broken.toml");

    assert!(
        missing.contains("The file is valid TOML"),
        "missing-key case:\n{missing}"
    );
    assert!(
        !broken.contains("The file is valid TOML"),
        "the file is *not* valid TOML here:\n{broken}"
    );
    assert!(
        broken.contains("broken.toml:1:9"),
        "a syntax error carries the column of the mistake:\n{broken}"
    );
}

/// `horus check main.cpp` was parsed as a manifest, so a C++ file came back
/// with a TOML diagnostic about itself.
#[test]
fn a_source_file_is_not_parsed_as_a_manifest() {
    let tmp = tempfile::tempdir().expect("tempdir");
    std::fs::write(tmp.path().join("main.cpp"), "int main( {\n").expect("write");

    let out = Command::new(horus())
        .args(["check", "main.cpp"])
        .current_dir(tmp.path())
        .output()
        .expect("horus check must run");
    let text = String::from_utf8_lossy(&out.stdout).into_owned();

    assert!(
        !text.contains("parse manifest") && !text.contains("expected `=`"),
        "a C++ file must not be reported as broken TOML:\n{text}"
    );
    assert!(
        text.contains("cannot check main.cpp"),
        "it must say plainly that it does not check this file:\n{text}"
    );
    assert!(
        text.contains("horus build") || text.contains("horus check <directory>"),
        "and name what does check it:\n{text}"
    );
    assert!(
        !out.status.success(),
        "the requested check did not happen, so it must not exit 0:\n{text}"
    );
}

/// A manifest with no `[package]` at all never reaches the typed error — it
/// deserializes with defaults and is caught by hand-written validation, which
/// reported the problem with no file, no line and no column.
#[test]
fn a_manifest_with_no_package_table_is_still_reported_with_a_position() {
    let tmp = tempfile::tempdir().expect("tempdir");
    project(tmp.path(), "[dependencies]\nhorus = \"*\"\n");

    let out = Command::new(horus())
        .args(["check", "horus.toml"])
        .current_dir(tmp.path())
        .output()
        .expect("horus check must run");
    let text = String::from_utf8_lossy(&out.stdout).into_owned();

    assert!(
        text.contains("Missing [package]"),
        "the problem must still be named:\n{text}"
    );
    assert!(
        text.contains("horus.toml:1:1"),
        "with the file and a position, like every other manifest failure:\n{text}"
    );

    let json = check_json(tmp.path(), &["check", "horus.toml", "--json"]);
    let entry = json["diagnostics"]
        .as_array()
        .and_then(|a| {
            a.iter().find(|d| {
                d["message"]
                    .as_str()
                    .unwrap_or("")
                    .contains("Missing [package]")
            })
        })
        .unwrap_or_else(|| panic!("--json must carry it too:\n{json:#}"));
    assert_eq!(entry["file"], "horus.toml", "{entry:#}");
    assert_eq!(entry["line"], 1, "{entry:#}");
    assert_eq!(entry["column"], 1, "{entry:#}");
}

/// `Cargo.toml` is valid TOML, so it parsed — and then every key in it came
/// back as unknown, because `check` was reading another tool's manifest as if
/// it were a horus.toml.
#[test]
fn another_tools_manifest_is_named_rather_than_linted() {
    let tmp = tempfile::tempdir().expect("tempdir");
    std::fs::write(
        tmp.path().join("Cargo.toml"),
        "[package]\nname = \"x\"\nversion = \"0.1.0\"\nedition = \"2021\"\n\n[lib]\npath = \"src/lib.rs\"\n",
    )
    .expect("write");

    let out = Command::new(horus())
        .args(["check", "Cargo.toml"])
        .current_dir(tmp.path())
        .output()
        .expect("horus check must run");
    let text = String::from_utf8_lossy(&out.stdout).into_owned();

    assert!(
        text.contains("is a Cargo manifest"),
        "the file must be named for what it is:\n{text}"
    );
    assert!(
        !text.contains("unknown key"),
        "and not linted key by key against a schema it was never written to:\n{text}"
    );
}
