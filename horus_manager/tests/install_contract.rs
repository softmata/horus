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
        let manifest = entry.path().join("Cargo.toml");
        if !manifest.is_file() {
            continue;
        }
        let text = std::fs::read_to_string(&manifest).unwrap_or_default();
        if !text.contains("[package]") {
            continue;
        }
        let name = entry.file_name().to_string_lossy().to_string();
        match text.lines().find(|l| l.trim_start().starts_with("rust-version")) {
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

// ─── Tests must not depend on an external service ───────────────────────────

/// A test that reaches crates.io or PyPI must carry `#[ignore]`.
///
/// The resolver's network tests sat under a comment reading "network tests —
/// ignored by default" while none of them were ignored. crates.io returns HTTP
/// 403 when it throttles, so `cargo test` failed on a machine where nothing was
/// wrong with the code — and a suite that fails for reasons outside the
/// repository teaches contributors to ignore failures.
#[test]
fn network_dependent_tests_are_ignored_by_default() {
    let src = std::fs::read_to_string(repo_root().join("horus_manager/src/source_resolver.rs"))
        .expect("source_resolver.rs must exist");

    let lines: Vec<&str> = src.lines().collect();
    let mut unguarded = Vec::new();

    for (i, line) in lines.iter().enumerate() {
        let Some(name) = line.trim().strip_prefix("fn ").and_then(|r| r.split('(').next()) else {
            continue;
        };
        // The body, up to the next test or the end of the block.
        let body: String = lines[i..]
            .iter()
            .take_while(|l| !l.trim_start().starts_with("#[test]"))
            .cloned()
            .collect::<Vec<_>>()
            .join("\n");

        // Reaching the network means asking the resolver to confirm a real
        // package, or fetching a real version.
        let hits_network = body.contains("resolve_with_network(")
            || body.contains("&DepSource::CratesIo)")
            || body.contains("&DepSource::PyPI)");
        if !hits_network {
            continue;
        }
        // "skips_network" tests are named for asserting the opposite.
        if name.contains("skips_network") {
            continue;
        }

        let preceding: String = lines[i.saturating_sub(3)..i].join("\n");
        if !preceding.contains("#[ignore") {
            unguarded.push(name.to_string());
        }
    }

    assert!(
        unguarded.is_empty(),
        "these tests reach crates.io or PyPI without `#[ignore]`, so a network \
         outage or a rate-limit fails the suite:\n  {}",
        unguarded.join("\n  ")
    );
}

/// Ignoring them is only acceptable if something still runs them.
#[test]
fn ci_runs_the_network_tests_it_ignores() {
    let ci = std::fs::read_to_string(repo_root().join(".github/workflows/ci.yml"))
        .expect("ci.yml must exist");
    assert!(
        ci.contains("source_resolver -- --ignored"),
        "the network tests are ignored by default and nothing in CI runs them, \
         which means they are not run at all"
    );
}

// ─── The container image ────────────────────────────────────────────────────
//
// The image cannot be built here — no Docker daemon — so these check the claims
// it makes against the repository it claims them about. That is less than
// building it, and it is what caught the builder pinning a Rust version the
// workspace no longer declares.

/// The builder image and the workspace must agree on the Rust floor.
#[test]
fn the_dockerfile_pins_the_workspace_msrv() {
    let dockerfile =
        std::fs::read_to_string(repo_root().join("Dockerfile")).expect("Dockerfile must exist");
    let root = std::fs::read_to_string(repo_root().join("Cargo.toml")).expect("Cargo.toml");
    let msrv = root
        .lines()
        .find_map(|l| l.trim().strip_prefix("rust-version = "))
        .map(|v| v.trim().trim_matches('"').to_string())
        .expect("workspace must declare rust-version");

    let pinned = dockerfile
        .lines()
        .find_map(|l| l.trim().strip_prefix("FROM rust:"))
        .and_then(|r| r.split('-').next())
        .map(str::to_string)
        .expect("the builder stage must pin a Rust version");

    assert_eq!(
        pinned, msrv,
        "the Dockerfile builds with Rust {pinned} while the workspace declares \
         {msrv}; pinning the image to the floor is what makes the floor testable"
    );
}

/// Every workspace member's manifest is copied before the source, for dependency
/// caching. A member missing from that list makes the build fail at `cargo
/// build` with a confusing "failed to load manifest" rather than at the COPY.
#[test]
fn the_dockerfile_copies_every_workspace_member() {
    let dockerfile =
        std::fs::read_to_string(repo_root().join("Dockerfile")).expect("Dockerfile");
    let root = std::fs::read_to_string(repo_root().join("Cargo.toml")).expect("Cargo.toml");

    let start = root.find("members").expect("workspace members");
    let body = &root[start..];
    let end = body.find("\n]").expect("members list must close");
    let members: Vec<String> = body[..end]
        .lines()
        .map(|l| l.split('#').next().unwrap_or("")) // drop comments
        .flat_map(|l| l.split(','))
        .map(|t| t.trim().trim_matches('"').to_string())
        .filter(|t| !t.is_empty() && !t.contains('=') && !t.contains('['))
        .collect();

    assert!(members.len() > 5, "parsed too few members: {members:?}");

    let missing: Vec<&String> = members
        .iter()
        .filter(|m| !dockerfile.contains(&format!("{m}/Cargo.toml")))
        .collect();
    assert!(
        missing.is_empty(),
        "these workspace members are not copied into the builder: {missing:?}"
    );
}

/// `--locked` refuses to update the lockfile, so the image cannot build without
/// one committed.
#[test]
fn the_dockerfile_can_use_the_committed_lockfile() {
    let dockerfile =
        std::fs::read_to_string(repo_root().join("Dockerfile")).expect("Dockerfile");
    if !dockerfile.contains("--locked") {
        return;
    }
    assert!(
        repo_root().join("Cargo.lock").is_file(),
        "the builder passes --locked but no Cargo.lock is committed"
    );
}

/// The runtime stage runs `horus man` and `horus completion` to place the files
/// the installer would. Both have to exist, or the image fails to build at a
/// step that has nothing to do with what changed.
#[test]
fn commands_the_dockerfile_runs_exist() {
    let dockerfile =
        std::fs::read_to_string(repo_root().join("Dockerfile")).expect("Dockerfile");
    // Extract the subcommand from every `horus <word>` in the file rather than
    // testing for a fixed list. A `contains("horus man")` check is still true
    // after the command is renamed to `horus manpage`, and passed while the
    // Dockerfile named something that does not exist.
    let mut checked = 0usize;
    let mut missing = Vec::new();
    for line in dockerfile.lines() {
        let mut rest = line;
        while let Some(at) = rest.find("horus ") {
            rest = &rest[at + "horus ".len()..];
            let word: String = rest
                .chars()
                .take_while(|c| c.is_ascii_alphanumeric() || *c == '-')
                .collect();
            if word.is_empty() || word.starts_with('-') {
                continue;
            }
            checked += 1;
            let out = std::process::Command::new(env!("CARGO_BIN_EXE_horus"))
                .args([&word, "--help"])
                .output()
                .expect("horus must run");
            // By exit status: "unrecognized subcommand" also appears inside the
            // help text of commands that have subcommands of their own.
            if !out.status.success() {
                missing.push(word);
            }
        }
    }

    assert!(
        checked > 0,
        "no `horus <command>` invocations found — the extractor is broken and \
         this test is vacuous"
    );
    assert!(
        missing.is_empty(),
        "the Dockerfile runs commands that do not exist: {missing:?}"
    );
}
