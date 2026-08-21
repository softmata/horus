//! `install.sh` must be able to build what it installs.
//!
//! The script's build command is the only thing standing between a user and a
//! working HORUS, and nothing checked that it compiles. It stopped compiling:
//!
//! ```text
//! $ cargo build --release -p horus_manager --no-default-features
//! error[E0425]: cannot find function `generate_manifest_schema` in module
//!               `horus_manager::manifest`
//!     --> horus_manager/src/main.rs:3163:51
//! note: found an item that was configured out
//! ```
//!
//! `install.sh` passed `--no-default-features`, and `schema` — the crate's only
//! default feature — is what the `horus schema` arm needs. Adding that command
//! broke every source install, and the same flag was on the release workflow's
//! build step, so it broke the published binaries too.
//!
//! Two guards, because either alone leaves the hole open: the code must compile
//! with the feature off, and the script must not turn off the feature the
//! shipped binary is meant to have.
//!
//! Run: `cargo test -p horus_manager --test install_contract`

use std::path::{Path, PathBuf};

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager has a parent")
        .to_path_buf()
}

fn install_sh() -> String {
    std::fs::read_to_string(repo_root().join("install.sh")).expect("install.sh must exist")
}

/// The shipped binary is meant to carry `schema` — the crate comment says so.
/// Disabling default features contradicts that and, for a while, did not build.
#[test]
fn install_sh_does_not_disable_default_features() {
    let text = install_sh();
    for (n, line) in text.lines().enumerate() {
        if line.contains("horus_manager") && line.contains("--no-default-features") {
            panic!(
                "install.sh:{} builds horus_manager without its default features:\n  {}",
                n + 1,
                line.trim()
            );
        }
    }
}

#[test]
fn the_release_workflow_builds_the_same_way_install_sh_does() {
    let wf = std::fs::read_to_string(repo_root().join(".github/workflows/release.yml"))
        .expect("release.yml must exist");
    for (n, line) in wf.lines().enumerate() {
        if line.contains("-p horus_manager") && line.contains("--no-default-features") {
            panic!(
                "release.yml:{} ships a binary without the schema feature:\n  {}",
                n + 1,
                line.trim()
            );
        }
    }
}

/// The other half. A `--no-default-features` build is legitimate — CI does it
/// for the cross-platform matrix — so the code must compile that way even
/// though the shipped binary does not.
#[test]
fn every_command_arm_is_guarded_for_a_no_default_features_build() {
    let main = std::fs::read_to_string(repo_root().join("horus_manager/src/main.rs"))
        .expect("main.rs must exist");

    let at = main
        .find("Commands::Schema { output }")
        .expect("the Schema arm must exist");
    let before = &main[..at];
    let guard = before
        .lines()
        .rev()
        .take(3)
        .any(|l| l.contains("#[cfg(feature = \"schema\")]"));
    assert!(
        guard,
        "the Schema arm calls a function that is compiled out without the \
         `schema` feature, so it must be behind the same cfg"
    );
    assert!(
        main.contains("#[cfg(not(feature = \"schema\"))]"),
        "there must be a fallback arm, or the match is non-exhaustive with the \
         feature off"
    );
}

// ─── The declared toolchain floor ───────────────────────────────────────────

/// One MSRV, in one place. Nine members declared none at all — which means no
/// floor, not the workspace's — and `horus_sys` declared 1.75 against the
/// workspace's 1.92.
#[test]
fn every_crate_inherits_the_workspace_rust_version() {
    let root = repo_root();
    let mut offenders = Vec::new();

    for entry in std::fs::read_dir(&root).expect("read repo root").flatten() {
        // Workspace members only. `horus run`/`horus build` generate a
        // `.horus/Cargo.toml` in whatever directory they are invoked from, and
        // it is gitignored build output — not a crate anyone declares an MSRV
        // for. Scanning it failed this test with ".horus: declares no
        // rust-version" whenever a CLI test had run here first, which is a
        // property of the working directory rather than of the repo.
        let dir_name = entry.file_name().to_string_lossy().to_string();
        if dir_name.starts_with('.') || dir_name == "target" {
            continue;
        }
        let manifest = entry.path().join("Cargo.toml");
        if !manifest.is_file() {
            continue;
        }
        let text = std::fs::read_to_string(&manifest).unwrap_or_default();
        if !text.contains("[package]") {
            continue;
        }
        let name = entry.file_name().to_string_lossy().to_string();
        match text
            .lines()
            .find(|l| l.trim_start().starts_with("rust-version"))
        {
            Some(line) if line.contains("workspace = true") => {}
            Some(line) => offenders.push(format!("{name}: {}", line.trim())),
            None => offenders.push(format!("{name}: declares no rust-version")),
        }
    }

    assert!(
        offenders.is_empty(),
        "these crates do not inherit the workspace MSRV:\n  {}",
        offenders.join("\n  ")
    );
}

/// The floor must be stated where someone deciding whether to install can see
/// it, not only in a manifest they will read after the build fails.
#[test]
fn the_readme_states_the_rust_floor() {
    let readme = std::fs::read_to_string(repo_root().join("README.md")).expect("README.md");
    let root = std::fs::read_to_string(repo_root().join("Cargo.toml")).expect("Cargo.toml");
    let msrv = root
        .lines()
        .find_map(|l| l.trim().strip_prefix("rust-version = "))
        .map(|v| v.trim().trim_matches('"').to_string())
        .expect("workspace must declare rust-version");

    assert!(
        readme.contains(&format!("Rust {msrv}")),
        "README.md does not state the required Rust version ({msrv})"
    );
}

/// install.sh must refuse an old toolchain before spending minutes compiling.
#[test]
fn install_sh_checks_the_rust_version_before_building() {
    let text = install_sh();
    assert!(
        text.contains("check_rust_version"),
        "install.sh does not check the toolchain version"
    );

    let def = text
        .find("check_rust_version() {")
        .expect("the function must be defined");
    let call = text
        .rfind("    check_rust_version")
        .expect("the function must be called");
    assert!(
        def < call,
        "check_rust_version is called before it is defined — it lives in the \
         other branch of the fast/slow path split and would be `command not found`"
    );

    // Read from the manifest, not hardcoded, or it silently rots.
    let body = &text[def..call.min(text.len())];
    assert!(
        body.contains("Cargo.toml"),
        "the required version must come from the workspace manifest:\n{body}"
    );
    assert!(
        body.contains("sort -V"),
        "version comparison must be numeric — string comparison says 1.100 < 1.9"
    );
}

// ─── The framework's own manifest ───────────────────────────────────────────

/// HORUS's own `horus.toml` must pass HORUS's own validator.
///
/// It did not. The package was named `HORUS`, which the charset rule rejects
/// ("must contain only lowercase letters, digits, hyphens, underscores, '@' and
/// '/'"), so `horus doctor` run inside this repository reported:
///
/// ```text
///   x Manifest — horus.toml invalid
///   Summary: 9 ok, 1 warnings, 1 failures
/// ```
///
/// Lowercasing it was not enough either — `horus` is on the reserved list — so
/// the manifest was in a position no name could satisfy without noticing both
/// rules. A diagnostic tool that fails on its own project teaches people to
/// ignore it.
#[test]
fn the_repository_manifest_passes_the_validator_it_ships() {
    let path = repo_root().join("horus.toml");
    let text = std::fs::read_to_string(&path).expect("the repo has a horus.toml");
    let manifest: horus_manager::manifest::HorusManifest =
        toml::from_str(&text).expect("horus.toml must parse");

    if let Err(errors) = manifest.validate() {
        panic!(
            "HORUS's own horus.toml fails HORUS's own validator:\n  {errors}\n\n\
             `horus doctor` reports this as a failure to anyone running it in \
             this repository."
        );
    }
}
