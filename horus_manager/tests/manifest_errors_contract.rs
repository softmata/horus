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
