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

/// Write the root `Cargo.toml`(s) from the project's generated manifest(s).
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

    let root_manifest = project_dir.join("Cargo.toml");
    if root_manifest.exists() && !force {
        bail!(
            "{} already exists — this project already has a Cargo-owned manifest. \
             Delete it first, or pass --force to replace it.",
            root_manifest.display()
        );
    }

    if manifest.is_workspace() {
        eject_workspace(&manifest, project_dir)
    } else {
        eject_single_crate(&manifest, project_dir)
    }
}

/// One package: `.horus/Cargo.toml` becomes the root `Cargo.toml`.
fn eject_single_crate(manifest: &HorusManifest, project_dir: &Path) -> Result<()> {
    // Ejecting a Python or C++ project would produce a Rust manifest for it.
    // The set matches what `cargo_gen` can turn into a target.
    if !has_rust_entry(project_dir) {
        bail!(
            "{} has no Rust entry point (src/main.rs, main.rs, src/lib.rs or \
             src/bin/*.rs), so there is no Cargo manifest to eject",
            project_dir.display()
        );
    }

    // `include_dev = true`: the ejected manifest is what `cargo test` reads
    // from now on, so dropping the dev-dependencies `horus.toml` declares
    // would silently break the test suite of the project that just ejected.
    let (generated_path, generated) = crate::cargo_gen::generate(manifest, project_dir, &[], true)
        .context("could not generate .horus/Cargo.toml to eject from")?;

    // A manifest with no library or binary is not buildable. `generate` only
    // emits `[lib]` for a lib target, so a stray `src/lib.rs` in a binary
    // project can leave the manifest target-less — cargo would then fail far
    // from here, on the user's first `cargo build`.
    if !(generated.contains("[[bin]]") || generated.contains("[lib]")) {
        bail!(
            "{} has no buildable target (a binary or a library); nothing to eject",
            project_dir.display()
        );
    }

    write_manifest(
        &project_dir.join("Cargo.toml"),
        &eject_manifest(&generated, rebase_generated_path),
    )?;

    let _ = fs::remove_file(&generated_path);
    let _ = fs::remove_file(project_dir.join(".horus/Cargo.lock"));
    finish(project_dir);
    Ok(())
}

/// A workspace: the generated virtual manifest becomes the root `Cargo.toml`
/// and every generated member manifest moves into its member's directory.
///
/// The generated layout is `.horus/Cargo.toml` (virtual, `members = ["<name>"]`
/// pointing at `.horus/<name>/`) plus one manifest per member. The ejected
/// layout is the ordinary one: a root workspace manifest whose `members` name
/// the real directories, and a manifest inside each member directory. Member
/// paths are rebased from `.horus/<name>/` to the member directory, which is
/// not always the same depth — a `members/arm/ws` layout would otherwise get
/// paths that resolve somewhere else.
fn eject_workspace(manifest: &HorusManifest, project_dir: &Path) -> Result<()> {
    let ws = manifest
        .workspace
        .as_ref()
        .expect("is_workspace() was checked by the caller");
    let members = crate::manifest::resolve_workspace_members(ws, project_dir)?;
    if members.is_empty() {
        bail!("this workspace has no members to eject");
    }

    // Regenerate right now, so the ejected copies reflect horus.toml as it is
    // — not as it was at the last build.
    let (_, generated_root) =
        crate::cargo_gen::generate_for_manifest(manifest, project_dir, &[], true)
            .context("could not generate the workspace manifests to eject from")?;

    let names = member_names(&generated_root)?;
    if names.len() != members.len() {
        bail!(
            "the generated workspace lists {} member(s) but {} were resolved; \
             refusing to eject a manifest that does not describe this workspace",
            names.len(),
            members.len()
        );
    }

    for ((member_dir, _), name) in members.iter().zip(&names) {
        // `resolve_workspace_members` returns member directories relative to
        // the project; make them absolute for filesystem work.
        let member_dir = project_dir.join(member_dir);
        let generated_member = project_dir.join(".horus").join(name).join("Cargo.toml");
        let text = fs::read_to_string(&generated_member).with_context(|| {
            format!(
                "generated member manifest {} is missing",
                generated_member.display()
            )
        })?;
        let horus_member_dir = project_dir.join(".horus").join(name);
        let ejected = eject_manifest(&text, |value| {
            rebase_between(value, &horus_member_dir, &member_dir)
        });
        write_manifest(&member_dir.join("Cargo.toml"), &ejected)?;
        let _ = fs::remove_file(&generated_member);
    }

    // The root manifest keeps its `[workspace.dependencies]`, `[patch]` tables
    // and profiles; only the header and the member list change.
    let mut root = eject_manifest(&generated_root, rebase_generated_path);
    // The resolved member paths are already relative to the project root,
    // which is exactly what a root workspace manifest wants.
    let members_line = format!(
        "members = [{}]",
        members
            .iter()
            .map(|(dir, _)| format!("\"{}\"", dir.to_string_lossy().replace('\\', "/")))
            .collect::<Vec<_>>()
            .join(", ")
    );
    root = replace_members_line(&root, &members_line)?;
    write_manifest(&project_dir.join("Cargo.toml"), &root)?;

    let _ = fs::remove_file(project_dir.join(".horus/Cargo.toml"));
    let _ = fs::remove_file(project_dir.join(".horus/Cargo.lock"));
    finish(project_dir);
    Ok(())
}

/// Parse the member names out of a generated virtual workspace manifest.
fn member_names(generated_root: &str) -> Result<Vec<String>> {
    for line in generated_root.lines() {
        let Some(rest) = line.strip_prefix("members = [") else {
            continue;
        };
        let Some(inner) = rest.strip_suffix(']') else {
            continue;
        };
        return Ok(inner
            .split(',')
            .map(|name| name.trim().trim_matches('"').to_string())
            .filter(|name| !name.is_empty())
            .collect());
    }
    bail!("the generated workspace manifest has no members list");
}

/// Replace the `members = [...]` line in a generated workspace manifest.
fn replace_members_line(root: &str, replacement: &str) -> Result<String> {
    let mut out = String::with_capacity(root.len());
    let mut replaced = false;
    for line in root.lines() {
        if line.starts_with("members = [") {
            out.push_str(replacement);
            replaced = true;
        } else {
            out.push_str(line);
        }
        out.push('\n');
    }
    if !replaced {
        bail!("the generated workspace manifest has no members list to rewrite");
    }
    Ok(out)
}

/// Whether the project has a layout `cargo_gen` can point a target at.
fn has_rust_entry(project_dir: &Path) -> bool {
    project_dir.join("src/main.rs").exists()
        || project_dir.join("main.rs").exists()
        || project_dir.join("src/lib.rs").exists()
        || fs::read_dir(project_dir.join("src/bin"))
            .map(|entries| {
                entries.flatten().any(|e| {
                    let path = e.path();
                    // Both forms cargo accepts: `src/bin/tool.rs` and
                    // `src/bin/tool/main.rs`.
                    (path.is_file() && path.extension().is_some_and(|ext| ext == "rs"))
                        || (path.is_dir() && path.join("main.rs").is_file())
                })
            })
            .unwrap_or(false)
}

/// Write an ejected manifest, refusing to write something cargo cannot parse.
fn write_manifest(path: &Path, text: &str) -> Result<()> {
    // Parse before writing: an unparseable root manifest is worse than no
    // eject, because every later `cargo` and `horus build` fails on it.
    let _: toml::Value = toml::from_str(text).with_context(|| {
        format!(
            "the ejected manifest is not valid TOML; refusing to write {}. This is a HORUS bug.",
            path.display()
        )
    })?;
    fs::write(path, text).with_context(|| format!("could not write {}", path.display()))
}

/// Clean up what a managed build leaves behind, once cargo owns the build.
fn finish(project_dir: &Path) {
    // The generated cargo config is only correct while HORUS owns the build;
    // with a root manifest, plain cargo would read `[rust].rustflags` that the
    // documentation says no longer apply.
    crate::cargo_gen::remove_generated_build_configs(project_dir);
}

/// Turn a generated manifest into the user-owned one it becomes.
///
/// The generated header (which says the file must not be edited) is replaced
/// by [`EJECT_HEADER`], and every `path = "…"` value is passed through
/// `rebase`, which moves it from the generated manifest's directory to the
/// ejected one. Absolute paths pass through untouched.
fn eject_manifest(generated: &str, rebase: impl Fn(&str) -> String) -> String {
    let mut out = String::with_capacity(generated.len() + EJECT_HEADER.len());
    out.push_str(EJECT_HEADER);
    for line in generated.lines() {
        if line.starts_with("# Generated by horus") || line.starts_with("# Edits are lost") {
            continue;
        }
        out.push_str(&rewrite_paths(line, &rebase));
        out.push('\n');
    }
    out
}

/// Rewrite every `path = "value"` in a line, including the inline form a
/// dependency uses (`local = { path = "../local" }`).
fn rewrite_paths(line: &str, rebase: &impl Fn(&str) -> String) -> String {
    const NEEDLE: &str = "path = \"";
    let mut out = String::with_capacity(line.len());
    let mut rest = line;
    while let Some(pos) = rest.find(NEEDLE) {
        let (before, after) = rest.split_at(pos + NEEDLE.len());
        out.push_str(before);
        match after.find('"') {
            Some(end) => {
                let (value, tail) = after.split_at(end);
                out.push_str(&rebase(value));
                out.push('"');
                rest = &tail[1..];
            }
            None => {
                rest = after;
            }
        }
    }
    out.push_str(rest);
    out
}

/// Move a path written relative to `.horus/` so it means the same file from
/// the project root.
///
/// The generated manifest lives one directory below the root, so:
/// - `../src/main.rs` (a target, or a local dependency) becomes `src/main.rs`;
/// - `packages/foo` (a registry package auto-installed under `.horus/`)
///   becomes `.horus/packages/foo` — prefixed, not stripped, because the file
///   really does live under `.horus/`.
///
/// Absolute paths are untouched.
fn rebase_generated_path(value: &str) -> String {
    let path = Path::new(value);
    if path.is_absolute() {
        return value.to_string();
    }
    match value {
        "" => String::new(),
        ".." => ".".to_string(),
        "." => ".horus".to_string(),
        _ => match value.strip_prefix("../") {
            Some(rest) if !rest.is_empty() => rest.to_string(),
            _ => format!(".horus/{value}"),
        },
    }
}

/// Rebase a relative path from one directory to another.
fn rebase_between(value: &str, from_dir: &Path, to_dir: &Path) -> String {
    if Path::new(value).is_absolute() {
        return value.to_string();
    }
    let target = from_dir.join(value);
    crate::cargo_gen::pathdiff(&target, to_dir)
        .unwrap_or_else(|_| target.to_string_lossy().to_string())
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
         vendored = { path = \"packages/vendored\" }\n\
         \n\
         [patch.\"https://github.com/softmata/horus-robotics.git\"]\n\
         horus_core = { path = \"/opt/horus/horus_core\" }\n"
            .to_string()
    }

    fn ejected() -> String {
        eject_manifest(&generated_manifest(), rebase_generated_path)
    }

    #[test]
    fn the_generated_header_is_replaced() {
        let out = ejected();
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
        let out = ejected();
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

    /// A registry package lives under `.horus/packages/`; from the root that
    /// is `.horus/packages/…`, not `packages/…`.
    #[test]
    fn generated_internal_paths_keep_the_horus_prefix() {
        let out = ejected();
        assert!(
            out.contains("path = \".horus/packages/vendored\""),
            "a .horus-relative dep was rebased to the wrong place:\n{out}"
        );
    }

    #[test]
    fn absolute_paths_are_untouched() {
        let out = ejected();
        assert!(out.contains("path = \"/opt/horus/horus\""), "{out}");
        assert!(out.contains("path = \"/opt/horus/horus_core\""), "{out}");
    }

    #[test]
    fn the_result_is_valid_toml_and_keeps_the_patch_tables() {
        let out = ejected();
        let parsed: toml::Value = toml::from_str(&out).expect("ejected manifest must parse");
        assert!(parsed.get("patch").is_some(), "patch tables were lost");
        assert_eq!(
            parsed["bin"][0]["path"].as_str(),
            Some("src/main.rs"),
            "the [[bin]] path is wrong after the roundtrip"
        );
    }

    /// Cargo also accepts `src/bin/<name>/main.rs`; the guard must not refuse
    /// a project the generator can represent.
    #[test]
    fn has_rust_entry_accepts_the_src_bin_directory_form() {
        let dir = tempfile::tempdir().unwrap();
        fs::create_dir_all(dir.path().join("src/bin/tool")).unwrap();
        fs::write(dir.path().join("src/bin/tool/main.rs"), "fn main() {}").unwrap();
        assert!(has_rust_entry(dir.path()));
    }

    #[test]
    fn a_bare_parent_path_is_not_stripped_to_nothing() {
        assert_eq!(rebase_generated_path("../"), ".horus/../");
        assert_eq!(rebase_generated_path(".."), ".");
        assert_eq!(rebase_generated_path("."), ".horus");
        assert_eq!(rebase_generated_path("packages/x"), ".horus/packages/x");
    }

    #[test]
    fn workspace_member_names_are_parsed_and_the_line_replaced() {
        let generated = "# Generated by horus from horus.toml — do not edit manually.\n\
                         # Edits are lost on the next build.\n\
                         [workspace]\n\
                         resolver = \"2\"\n\
                         members = [\"arm\", \"base\"]\n\
                         \n\
                         [workspace.dependencies]\n\
                         serde = \"1\"\n";
        let names = member_names(generated).unwrap();
        assert_eq!(names, vec!["arm".to_string(), "base".to_string()]);

        let replaced =
            replace_members_line(generated, "members = [\"crates/arm\", \"crates/base\"]").unwrap();
        assert!(
            replaced.contains("members = [\"crates/arm\", \"crates/base\"]"),
            "{replaced}"
        );
        assert!(!replaced.contains("members = [\"arm\""), "{replaced}");
        let _: toml::Value = toml::from_str(&replaced).expect("still valid TOML");
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

    /// Workspaces eject too: the generated virtual manifest becomes the root
    /// one, and each member manifest moves into its member directory with its
    /// paths rebased.
    #[test]
    fn workspace_members_eject_into_their_own_directories() {
        if crate::commands::run::find_horus_source_dir().is_err() {
            eprintln!("skipping: HORUS source tree not found");
            return;
        }
        let dir = tempfile::tempdir().unwrap();
        let root = dir.path();
        fs::create_dir_all(root.join("crates/arm/src")).unwrap();
        fs::write(
            root.join(HORUS_TOML),
            "[package]\nname = \"root\"\nversion = \"0.1.0\"\n\n\
             [workspace]\nmembers = [\"crates/*\"]\n",
        )
        .unwrap();
        fs::write(
            root.join("crates/arm/horus.toml"),
            "[package]\nname = \"arm\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        fs::write(root.join("crates/arm/src/main.rs"), "fn main() {}").unwrap();

        materialize_root_manifest(root, false).unwrap();

        let root_manifest = fs::read_to_string(root.join("Cargo.toml")).unwrap();
        assert!(
            root_manifest.contains("members = [\"crates/arm\"]"),
            "the ejected workspace does not point at the member directory:\n{root_manifest}"
        );
        let member_manifest = fs::read_to_string(root.join("crates/arm/Cargo.toml")).unwrap();
        assert!(
            member_manifest.contains("path = \"src/main.rs\""),
            "the member target path was not rebased:\n{member_manifest}"
        );
        assert!(
            !root.join(".horus/Cargo.toml").exists()
                && !root.join(".horus/arm/Cargo.toml").exists(),
            "a generated manifest was left behind"
        );
        let _: toml::Value =
            toml::from_str(&root_manifest).expect("ejected workspace manifest must parse");
    }

    /// A `src/bin/`-only project is a valid Rust project; eject must not refuse
    /// it just because there is no `main.rs`.
    #[test]
    fn src_bin_only_projects_can_eject() {
        if crate::commands::run::find_horus_source_dir().is_err() {
            eprintln!("skipping: HORUS source tree not found");
            return;
        }
        let dir = tempfile::tempdir().unwrap();
        let root = dir.path();
        fs::create_dir_all(root.join("src/bin")).unwrap();
        fs::write(
            root.join(HORUS_TOML),
            "[package]\nname = \"binonly\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        fs::write(root.join("src/bin/tool.rs"), "fn main() {}").unwrap();

        materialize_root_manifest(root, false).unwrap();
        let root_manifest = fs::read_to_string(root.join("Cargo.toml")).unwrap();
        assert!(
            root_manifest.contains("path = \"src/bin/tool.rs\""),
            "the src/bin target was not ejected:\n{root_manifest}"
        );
    }
}
