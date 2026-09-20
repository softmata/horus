//! `horus eject` — hand the build to a Cargo manifest the user owns.
//!
//! A managed project has no root `Cargo.toml`: `horus.toml` is the manifest
//! and `.horus/Cargo.toml` is a generated, gitignored build artefact. That is
//! why plain `cargo build` fails there — there is nothing for cargo to find.
//! `horus env --init` papers over it with a shell function that routes `cargo`
//! through `horus cargo`, but a user who wants cargo itself (an editor, a
//! script, CI, or just muscle memory) has no way to get a real manifest.
//!
//! `horus eject` writes one, from the generated manifest, and stops owning the
//! build: from then on `horus build`/`run`/`test` build from the root
//! `Cargo.toml` directly (the same branch `cargo_gen` already takes when a
//! root manifest exists), and `[rust]` in `horus.toml` becomes inert.

use anyhow::{bail, Context, Result};
use colored::Colorize as _;
use std::fs;
use std::path::Path;

use crate::cli_output;
use crate::manifest::{HorusManifest, HORUS_TOML};

/// The header the ejected manifest carries instead of the generated one.
///
/// The generated header says "do not edit; edits are lost" — the opposite of
/// what is true after an eject, so it is replaced rather than kept.
const EJECT_HEADER: &str = "\
# Cargo manifest written by HORUS (`horus eject` / `horus new --cargo`).
# HORUS builds from it (`horus build`, `horus run`, `horus test`) and will
# never regenerate it. Settings that belong in `[rust]` in horus.toml —
# profile, lints, features — belong in this file instead.
# The HORUS dependencies below are absolute paths to the source tree HORUS
# resolved when it wrote this; moving or upgrading that tree breaks them.

";

/// `horus eject`.
pub fn run_eject(force: bool) -> Result<()> {
    let project_dir = std::env::current_dir()?;
    cli_output::header("Ejecting from HORUS managed mode");
    materialize_root_manifest(&project_dir, force)?;

    println!();
    cli_output::success("Cargo.toml written — this project now builds like any other Rust crate.");
    println!(
        "  {} plain cargo works here: {}",
        "·".dimmed(),
        "cargo build / cargo run / cargo test".cyan()
    );
    println!(
        "  {} {} also build from it",
        "·".dimmed(),
        "horus build / horus run / horus test".cyan()
    );
    println!(
        "  {} {} in horus.toml no longer applies — move profile/lints/features into Cargo.toml",
        "·".dimmed(),
        "[rust]".yellow()
    );
    println!(
        "  {} the manifest is yours now; HORUS will not regenerate it",
        "·".dimmed()
    );
    Ok(())
}

/// Write a root `Cargo.toml` from the project's generated `.horus/Cargo.toml`.
///
/// Shared by `horus eject` and `horus new --cargo`, so a project created
/// cargo-native and one ejected later get byte-identical manifests.
///
/// Refuses a project that already has a root `Cargo.toml` (unless `force`) —
/// that manifest is the user's, and silently replacing it would be the exact
/// class of behaviour this command exists to avoid.
pub fn materialize_root_manifest(project_dir: &Path, force: bool) -> Result<()> {
    let manifest_path = project_dir.join(HORUS_TOML);
    let manifest = HorusManifest::load_from(&manifest_path).with_context(|| {
        format!(
            "{} is not a HORUS project (no {})",
            project_dir.display(),
            HORUS_TOML
        )
    })?;

    if manifest.is_workspace() {
        bail!(
            "workspace projects already build from a generated root Cargo.toml; \
             there is nothing to eject"
        );
    }

    // Ejecting a Python or C++ project would produce a Rust manifest for it.
    // The entry-point check is the same one the build uses.
    let is_rust = project_dir.join("src/main.rs").exists()
        || project_dir.join("main.rs").exists()
        || project_dir.join("src/lib.rs").exists();
    if !is_rust {
        bail!(
            "{} has no Rust entry point (src/main.rs, main.rs or src/lib.rs), \
             so there is no Cargo manifest to eject",
            project_dir.display()
        );
    }

    let root_manifest = project_dir.join("Cargo.toml");
    if root_manifest.exists() && !force {
        bail!(
            "{} already exists — this project already has a Cargo-owned manifest. \
             Delete it first, or pass --force to replace it.",
            root_manifest.display()
        );
    }

    // Regenerate right now, so the ejected copy reflects horus.toml as it is
    // — not as it was at the last build.
    let (generated_path, generated) =
        crate::cargo_gen::generate(&manifest, project_dir, &[], false)
            .context("could not generate .horus/Cargo.toml to eject from")?;

    let ejected = eject_manifest(&generated);
    // Parse before writing: an unparseable root manifest is worse than no
    // eject, because every later `cargo` and `horus build` fails on it.
    let _: toml::Value = toml::from_str(&ejected).with_context(|| {
        format!(
            "the ejected manifest is not valid TOML; refusing to write {}. This is a HORUS bug.",
            root_manifest.display()
        )
    })?;

    fs::write(&root_manifest, &ejected)
        .with_context(|| format!("could not write {}", root_manifest.display()))?;

    // The generated manifest must not linger: with a root manifest present,
    // HORUS builds from that one, and a stale `.horus/Cargo.toml` is a second
    // source of truth that nothing regenerates.
    let _ = fs::remove_file(&generated_path);
    let _ = fs::remove_file(project_dir.join(".horus/Cargo.lock"));

    Ok(())
}

/// Turn a generated `.horus/Cargo.toml` into the root `Cargo.toml` it becomes.
///
/// Two changes, both mechanical:
/// - the generated header (which says the file must not be edited) is replaced
///   by [`EJECT_HEADER`];
/// - every relative path loses exactly one leading `../`. The generated
///   manifest lives one directory below the root, so `../src/main.rs` (a
///   target) and `../../somecrate` (a local path dependency) mean the same
///   file from the root as `src/main.rs` and `../somecrate`. Absolute paths —
///   the HORUS source tree deps — are untouched.
///
/// Split out so the transformation is testable without a project on disk.
fn eject_manifest(generated: &str) -> String {
    let mut out = String::with_capacity(generated.len() + EJECT_HEADER.len());
    out.push_str(EJECT_HEADER);
    for line in generated.lines() {
        if line.starts_with("# Generated by horus") || line.starts_with("# Edits are lost") {
            continue;
        }
        out.push_str(&strip_one_parent(line));
        out.push('\n');
    }
    out
}

/// `path = "../x"` → `path = "x"`, `path = "../../x"` → `path = "../x"`.
///
/// Handles both the standalone form a target uses (`path = "../src/main.rs"`)
/// and the inline form a dependency uses (`local = { path = "../../local" }`).
/// Anything else is returned unchanged, and a bare `path = "../"` is left
/// alone: stripping it would produce an empty path.
fn strip_one_parent(line: &str) -> String {
    const NEEDLE: &str = "path = \"";
    let mut out = String::with_capacity(line.len());
    let mut rest = line;
    while let Some(pos) = rest.find(NEEDLE) {
        let (before, after) = rest.split_at(pos + NEEDLE.len());
        out.push_str(before);
        rest = after;
        if let Some(stripped) = rest.strip_prefix("../") {
            if !stripped.is_empty() && !stripped.starts_with('"') {
                rest = stripped;
            }
        }
    }
    out.push_str(rest);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn generated_manifest() -> String {
        "# Generated by horus from horus.toml — do not edit manually.\n\
         # Edits are lost on the next build; add a [rust] section to horus.toml instead.\n\
         [package]\n\
         name = \"robot\"\n\
         version = \"0.1.0\"\n\
         edition = \"2021\"\n\
         \n\
         [workspace]\n\
         \n\
         [[bin]]\n\
         name = \"robot\"\n\
         path = \"../src/main.rs\"\n\
         \n\
         [[test]]\n\
         name = \"smoke\"\n\
         path = \"../tests/smoke.rs\"\n\
         \n\
         [dependencies]\n\
         horus = { path = \"/opt/horus/horus\" }\n\
         local = { path = \"../../local-crate\" }\n\
         \n\
         [patch.\"https://github.com/softmata/horus-robotics.git\"]\n\
         horus_core = { path = \"/opt/horus/horus_core\" }\n"
            .to_string()
    }

    #[test]
    fn the_generated_header_is_replaced() {
        let out = eject_manifest(&generated_manifest());
        assert!(out.starts_with("# Cargo manifest written by HORUS"));
        assert!(
            !out.contains("do not edit manually"),
            "the generated header survived:\n{out}"
        );
        assert!(
            !out.contains("Edits are lost"),
            "the lost-edits warning survived:\n{out}"
        );
    }

    #[test]
    fn relative_paths_move_up_one_level() {
        let out = eject_manifest(&generated_manifest());
        assert!(
            out.contains("path = \"src/main.rs\""),
            "target path not rewritten:\n{out}"
        );
        assert!(
            out.contains("path = \"tests/smoke.rs\""),
            "test path not rewritten:\n{out}"
        );
        assert!(
            out.contains("path = \"../local-crate\""),
            "a two-level relative dep was not moved exactly one level:\n{out}"
        );
    }

    #[test]
    fn absolute_paths_are_untouched() {
        let out = eject_manifest(&generated_manifest());
        assert!(out.contains("path = \"/opt/horus/horus\""), "{out}");
        assert!(out.contains("path = \"/opt/horus/horus_core\""), "{out}");
    }

    #[test]
    fn the_result_is_valid_toml_and_keeps_the_patch_tables() {
        let out = eject_manifest(&generated_manifest());
        let parsed: toml::Value = toml::from_str(&out).expect("ejected manifest must parse");
        assert!(parsed.get("patch").is_some(), "patch tables were lost");
        assert_eq!(
            parsed["bin"][0]["path"].as_str(),
            Some("src/main.rs"),
            "the [[bin]] path is wrong after the roundtrip"
        );
    }

    #[test]
    fn a_bare_parent_path_is_not_stripped_to_nothing() {
        assert_eq!(strip_one_parent("path = \"../\""), "path = \"../\"");
        assert_eq!(strip_one_parent("path = \"..\""), "path = \"..\"");
    }

    /// End to end on a real project directory: the root manifest appears, its
    /// target paths point at the project (not `.horus/`), the generated
    /// manifest is gone, and a second run refuses to clobber the first.
    #[test]
    fn materialize_writes_the_root_manifest_and_removes_the_generated_one() {
        if crate::commands::run::find_horus_source_dir().is_err() {
            eprintln!("skipping: HORUS source tree not found");
            return;
        }
        let dir = tempfile::tempdir().unwrap();
        let root = dir.path();
        fs::create_dir_all(root.join("src")).unwrap();
        fs::write(
            root.join(HORUS_TOML),
            "[package]\nname = \"ejectme\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        fs::write(root.join("src/main.rs"), "fn main() {}").unwrap();

        materialize_root_manifest(root, false).unwrap();

        let root_manifest = fs::read_to_string(root.join("Cargo.toml")).unwrap();
        assert!(
            root_manifest.contains("path = \"src/main.rs\""),
            "the ejected target path still points into .horus/:\n{root_manifest}"
        );
        assert!(
            !root.join(".horus/Cargo.toml").exists(),
            "the generated manifest was left behind as a second source of truth"
        );

        // Without --force, an existing root manifest is the user's.
        assert!(
            materialize_root_manifest(root, false).is_err(),
            "eject overwrote an existing root Cargo.toml without --force"
        );
    }
}
