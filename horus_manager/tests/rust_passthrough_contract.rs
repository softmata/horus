//! `horus build` must not destroy the only way to configure cargo.
//!
//! `.horus/Cargo.toml` is generated from `horus.toml` and rewritten whenever
//! `horus.toml` is newer, so anything a user adds to it by hand disappears on
//! the next build — silently, with no warning and no record of what was lost.
//! `[cpp]` has forwarded settings to CMake all along. Rust had no equivalent,
//! which left a cargo feature, a `[profile.release]` setting, a lint level or
//! an extra `[patch]` with no supported way to be expressed at all. The
//! documentation's own advice was to edit the generated file.
//!
//! `[rust]` is that equivalent: its tables are spliced into the generated
//! manifest. It covers exactly the sections HORUS does not write itself, so
//! there is nothing to conflict with — except `[patch]`, where HORUS's own
//! entries are load-bearing and win.
//!
//! Run: `cargo test -p horus_manager --test rust_passthrough_contract`

use std::path::Path;
use std::process::Command;

use horus_manager::manifest::HorusManifest;

fn horus() -> &'static str {
    env!("CARGO_BIN_EXE_horus")
}

/// Generate a Rust project, append `extra` to its horus.toml, build, and return
/// the generated Cargo.toml.
fn generated_manifest(extra: &str) -> String {
    let tmp = tempfile::tempdir().expect("tempdir");
    let out = Command::new(horus())
        .args(["new", "demo", "--rust"])
        .current_dir(tmp.path())
        .output()
        .expect("horus new must run");
    assert!(
        out.status.success(),
        "horus new failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );

    let project = tmp.path().join("demo");
    let manifest_path = project.join("horus.toml");
    let mut text = std::fs::read_to_string(&manifest_path).expect("read horus.toml");
    text.push_str(extra);
    std::fs::write(&manifest_path, text).expect("write horus.toml");

    let build = Command::new(horus())
        .arg("build")
        .current_dir(&project)
        .output()
        .expect("horus build must run");

    let generated = project.join(".horus/Cargo.toml");
    assert!(
        generated.is_file(),
        "no generated manifest. build stdout:\n{}\nstderr:\n{}",
        String::from_utf8_lossy(&build.stdout),
        String::from_utf8_lossy(&build.stderr)
    );
    std::fs::read_to_string(generated).expect("read generated manifest")
}

// ─── The schema exists and is understood ────────────────────────────────────

#[test]
fn the_rust_section_parses() {
    let m = HorusManifest::parse_str(
        "[package]\nname = \"x\"\nversion = \"0.1.0\"\n\n\
         [rust]\nedition = \"2024\"\n\n\
         [rust.features]\ndefault = [\"fast\"]\n",
        Path::new("horus.toml"),
    )
    .expect("[rust] must be a known section");

    let rust = m.rust.expect("[rust] must be captured");
    assert_eq!(rust.edition.as_deref(), Some("2024"));
    assert!(rust.features.is_some());
}

/// An unknown key inside `[rust]` must be caught, or a typo silently does
/// nothing — the exact class of failure this feature exists to end.
#[test]
fn a_typo_inside_the_rust_section_is_reported() {
    let tmp = tempfile::tempdir().expect("tempdir");
    std::fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"x\"\nversion = \"0.1.0\"\n\n[rust]\nprofiel = {}\n",
    )
    .expect("write");

    let out = Command::new(horus())
        .arg("check")
        .current_dir(tmp.path())
        .output()
        .expect("horus check must run");
    let text = String::from_utf8_lossy(&out.stdout).into_owned()
        + &String::from_utf8_lossy(&out.stderr);
    assert!(
        text.contains("profiel"),
        "an unknown [rust] key must be reported:\n{text}"
    );
}

// ─── The sections reach the generated manifest ──────────────────────────────

#[test]
fn features_reach_the_generated_manifest() {
    let cargo = generated_manifest("\n[rust.features]\ndefault = [\"fast\"]\nfast = []\n");
    assert!(cargo.contains("[features]"), "{cargo}");
    assert!(cargo.contains("default = [\"fast\"]"), "{cargo}");
}

/// A release profile is the single most common reason to hand-edit.
#[test]
fn a_release_profile_reaches_the_generated_manifest() {
    let cargo = generated_manifest(
        "\n[rust.profile.release]\nlto = \"fat\"\ncodegen-units = 1\npanic = \"abort\"\n",
    );
    assert!(cargo.contains("[profile.release]"), "{cargo}");
    assert!(cargo.contains("lto = \"fat\""), "{cargo}");
    assert!(cargo.contains("codegen-units = 1"), "{cargo}");
}

/// A nested table. Hand-written emission got this wrong and produced an empty
/// `[lints]`, because it skipped anything that reported as a table.
#[test]
fn nested_tables_are_emitted_with_their_full_path() {
    let cargo = generated_manifest("\n[rust.lints.rust]\nunsafe_code = \"forbid\"\n");
    assert!(
        cargo.contains("[lints.rust]"),
        "expected a [lints.rust] header, not a bare [lints]:\n{cargo}"
    );
    assert!(cargo.contains("unsafe_code = \"forbid\""), "{cargo}");
}

#[test]
fn the_edition_can_be_overridden() {
    let cargo = generated_manifest("\n[rust]\nedition = \"2024\"\n");
    assert!(cargo.contains("edition = \"2024\""), "{cargo}");
    assert!(!cargo.contains("edition = \"2021\""), "{cargo}");
}

/// Absent `[rust]` must change nothing.
#[test]
fn no_rust_section_leaves_the_manifest_as_it_was() {
    let cargo = generated_manifest("");
    assert!(cargo.contains("edition = \"2021\""), "{cargo}");
    assert!(!cargo.contains("[features]"), "{cargo}");
    assert!(!cargo.contains("[profile"), "{cargo}");
}

// ─── patch merging ──────────────────────────────────────────────────────────

/// HORUS's own patch entries are load-bearing: without them every generated
/// project fails to resolve `horus_core`. A user cannot displace them.
#[test]
fn horus_patch_entries_survive_a_user_patch_table() {
    let cargo = generated_manifest(
        "\n[rust.patch.\"https://github.com/softmata/horus-robotics.git\"]\n\
         horus_core = { path = \"/definitely/not/here\" }\n",
    );
    assert!(
        cargo.contains("[patch.\"https://github.com/softmata/horus-robotics.git\"]"),
        "{cargo}"
    );
    assert!(
        !cargo.contains("/definitely/not/here"),
        "a user patch must not displace HORUS's own entry:\n{cargo}"
    );
}

/// An inline table is also a table. Emitting these by hand dropped them,
/// because the check that skipped section headers could not tell the two apart.
#[test]
fn a_user_patch_entry_is_emitted_inline() {
    let cargo = generated_manifest(
        "\n[rust.patch.\"https://github.com/example/other.git\"]\n\
         somecrate = { path = \"/tmp/somecrate\" }\n",
    );
    assert!(
        cargo.contains("[patch.\"https://github.com/example/other.git\"]"),
        "an unclaimed patch source must pass through:\n{cargo}"
    );
    assert!(
        cargo.contains("somecrate = { path = \"/tmp/somecrate\" }"),
        "the entry must be emitted, and inline:\n{cargo}"
    );
}

// ─── The generated manifest is still valid ──────────────────────────────────

/// The point is not that the text appears — it is that cargo accepts it.
#[test]
fn the_generated_manifest_is_still_valid_toml_and_a_valid_package() {
    let cargo = generated_manifest(
        "\n[rust]\nedition = \"2021\"\n\n\
         [rust.features]\ndefault = []\n\n\
         [rust.profile.release]\nlto = \"thin\"\n\n\
         [rust.lints.rust]\nunsafe_code = \"forbid\"\n",
    );
    let parsed: toml::Value = toml::from_str(&cargo)
        .unwrap_or_else(|e| panic!("generated manifest is not valid TOML: {e}\n{cargo}"));

    assert!(parsed.get("package").is_some(), "{cargo}");
    assert!(parsed.get("features").is_some(), "{cargo}");
    assert!(
        parsed
            .get("profile")
            .and_then(|p| p.get("release"))
            .and_then(|r| r.get("lto"))
            .is_some(),
        "{cargo}"
    );
}

// ─── The escape hatch is discoverable ───────────────────────────────────────

/// The header said "do not edit manually" and offered no alternative, so the
/// only way to learn `[rust]` exists is for the file to say so.
#[test]
fn the_generated_header_names_the_escape_hatch() {
    let cargo = generated_manifest("");
    let head: String = cargo.lines().take(3).collect::<Vec<_>>().join("\n");
    assert!(
        head.contains("[rust]"),
        "the header must point at the supported alternative:\n{head}"
    );
}

/// With a root Cargo.toml, HORUS builds from it and never generates one — so a
/// `[rust]` section would be parsed, accepted, and silently ignored. That is
/// precisely the failure this feature removes, and it must not be reintroduced
/// by it.
#[test]
fn a_rust_section_that_cannot_take_effect_is_reported() {
    let src = std::fs::read_to_string("src/commands/run/run_rust.rs").expect("read run_rust.rs");
    assert!(
        src.contains("warn_if_rust_section_is_ignored"),
        "no warning exists for the root-Cargo.toml case"
    );
    let at = src
        .find("fn warn_if_rust_section_is_ignored")
        .expect("the helper must exist");
    let body: String = src[at..].lines().take(30).collect::<Vec<_>>().join("\n");
    assert!(
        body.contains("has no") || body.contains("no effect"),
        "the warning must say the section is being ignored:\n{body}"
    );
}
