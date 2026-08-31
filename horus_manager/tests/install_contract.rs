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
//! What the rest of this file is, and what it is worth. The scans below read
//! `install.sh`, `uninstall.sh`, `release.yml`, `ci.yml`, the `Dockerfile`,
//! `horus.toml` and the workspace manifests as *text*, and they catch what text
//! can catch. What they never caught is whether an install works, because until
//! `runs_install_sh` below, nothing in a file named `install_contract` executed
//! `install.sh`: a green run here proved only that the script contained certain
//! strings. `completion_install_contract.rs` runs the script's tail — from the
//! rc-file detection on — for exactly that reason. `runs_install_sh` gives the
//! front half the same treatment: real `bash`, a throw-away `$HOME`, `curl` and
//! `git` stubbed to serve a locally built release, covering the download, the
//! checksum, the extract, the cached source tree, the tag both halves come
//! from, and the version state files that tell the CLI what is installed.
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

/// The uninstaller is shipped and curl-piped the same way the installer is, so
/// every portability rule that applies to one applies to both.
fn uninstall_sh() -> String {
    std::fs::read_to_string(repo_root().join("uninstall.sh")).expect("uninstall.sh must exist")
}

/// The executable part of a shell line, with any trailing comment removed.
///
/// A `#` opens a comment only at the start of a word, so `${REPO}#ref` is code
/// while `cmd  # note` is not. The cheap `split('#')` used elsewhere in this
/// file is fine for scanners whose pattern never appears in prose, but install.sh
/// quotes the *old, broken* clone guard verbatim in a comment explaining why it
/// no longer does that. A scanner that reads comments fails the script for
/// documenting the very bug it fixed.
fn shell_code(line: &str) -> &str {
    let mut at_word_start = true;
    for (i, c) in line.char_indices() {
        if c == '#' && at_word_start {
            return &line[..i];
        }
        at_word_start = c.is_whitespace();
    }
    line
}

/// Does this line pipe `git clone` into another command?
///
/// `||` is not a pipe. Anything else after `git clone` on the line is, and with
/// `pipefail` off a pipeline reports only its *last* command's status.
fn clone_is_piped(code: &str) -> bool {
    let Some(pos) = code.find("git clone") else {
        return false;
    };
    let bytes = &code.as_bytes()[pos..];
    bytes.iter().enumerate().any(|(i, &b)| {
        b == b'|' && bytes.get(i.wrapping_sub(1)) != Some(&b'|') && bytes.get(i + 1) != Some(&b'|')
    })
}

/// The shipped binary is meant to carry `schema` — the crate comment says so.
/// Disabling default features contradicts that and, for a while, did not build.
#[test]
fn install_sh_does_not_disable_default_features() {
    let text = install_sh();
    // The scan is a search for a line that must not exist, so it also passes
    // when there is no build line left to search. HORUS_BUILD_FROM_SOURCE is a
    // documented escape hatch, so one must always be there.
    assert!(
        text.contains("horus_manager"),
        "install.sh no longer builds horus_manager at all, so this scan proves \
         nothing — and HORUS_BUILD_FROM_SOURCE=1 has nothing left to build"
    );
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
    // Same shape, same hole: with no `-p horus_manager` line anywhere the loop
    // below examines nothing and reports success.
    assert!(
        wf.contains("-p horus_manager"),
        "release.yml does not build horus_manager, so it publishes no CLI and \
         this scan is vacuous"
    );
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

/// macOS still ships Bash 3.2 as `/bin/bash`, so the curl-piped scripts must
/// not use syntax that only exists in Bash 4+.
///
/// Both scripts. The table below has always listed `mapfile`, but the scan read
/// `install.sh` alone, and it is `uninstall.sh` that grew one — to de-duplicate
/// its artifact path lists. On a stock macOS that line is `mapfile: command not
/// found`, and `uninstall.sh` runs `set -e`, so the uninstaller dies at the
/// path table having removed nothing, on the platform where `/bin/bash` is
/// still 3.2 and there is no package manager entry to blame.
#[test]
fn the_shipped_shell_scripts_avoid_bash_four_only_constructs() {
    let forbidden = [
        ("declare -A", "associative arrays require Bash 4"),
        ("readarray", "readarray requires Bash 4"),
        ("mapfile", "mapfile requires Bash 4"),
        ("coproc", "coproc requires Bash 4"),
    ];

    for (name, text) in [
        ("install.sh", install_sh()),
        ("uninstall.sh", uninstall_sh()),
    ] {
        for (n, line) in text.lines().enumerate() {
            let code = shell_code(line);
            for (needle, reason) in forbidden {
                assert!(
                    !code.contains(needle),
                    "{name}:{} uses `{needle}` but {reason}; macOS /bin/bash is \
                     still 3.2:\n  {}",
                    n + 1,
                    line.trim()
                );
            }
            assert!(
                !uses_bash_four_case_modification(code),
                "{name}:{} uses Bash 4 case-modification expansion; macOS \
                 /bin/bash is still 3.2:\n  {}",
                n + 1,
                line.trim()
            );
        }
    }
}

fn uses_bash_four_case_modification(code: &str) -> bool {
    let bytes = code.as_bytes();
    let mut i = 0;
    while i + 1 < bytes.len() {
        if bytes[i] != b'$' || bytes[i + 1] != b'{' {
            i += 1;
            continue;
        }

        let body_start = i + 2;
        let Some(end) = parameter_expansion_end(bytes, body_start) else {
            i += 2;
            continue;
        };
        let expansion = &code[body_start..end];
        if parameter_expansion_uses_case_modification(expansion)
            || uses_bash_four_case_modification(expansion)
        {
            return true;
        }
        i = end + 1;
    }
    false
}

fn parameter_expansion_end(bytes: &[u8], body_start: usize) -> Option<usize> {
    let mut depth = 1usize;
    let mut i = body_start;
    while i < bytes.len() {
        if i + 1 < bytes.len() && bytes[i] == b'$' && bytes[i + 1] == b'{' {
            depth += 1;
            i += 2;
            continue;
        }
        if bytes[i] == b'}' {
            depth -= 1;
            if depth == 0 {
                return Some(i);
            }
        }
        i += 1;
    }
    None
}

fn parameter_expansion_uses_case_modification(expansion: &str) -> bool {
    let bytes = expansion.as_bytes();
    let mut i = 0;

    if bytes.first() == Some(&b'!') {
        i += 1;
    }
    while i < bytes.len() && (bytes[i].is_ascii_alphanumeric() || bytes[i] == b'_') {
        i += 1;
    }
    if bytes.get(i) == Some(&b'[') {
        while i < bytes.len() && bytes[i] != b']' {
            i += 1;
        }
        if i < bytes.len() {
            i += 1;
        }
    }

    matches!(bytes.get(i), Some(b'^' | b','))
}

#[test]
fn bash_four_case_modification_scanner_identifies_constructs() {
    assert!(uses_bash_four_case_modification("${name^}"));
    assert!(uses_bash_four_case_modification("${name^^[a-z]}"));
    assert!(uses_bash_four_case_modification("${name,}"));
    assert!(uses_bash_four_case_modification("${name,,[A-Z]}"));
    assert!(uses_bash_four_case_modification("${outer:-${inner^^}}"));
    // Robustness: an unclosed outer expansion must not hide a later complete
    // nested Bash 4 expansion on the same line.
    assert!(uses_bash_four_case_modification("${broken ${name^}"));

    assert!(!uses_bash_four_case_modification("${name:-fallback}"));
    assert!(!uses_bash_four_case_modification("${name##*/}"));
    assert!(!uses_bash_four_case_modification(
        "${name/pattern/replacement}"
    ));
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
    let mut examined = 0usize;

    for (i, line) in lines.iter().enumerate() {
        let Some(name) = line
            .trim()
            .strip_prefix("fn ")
            .and_then(|r| r.split('(').next())
        else {
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
        examined += 1;
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
    // The detector keys off call syntax in source_resolver.rs. A rename there
    // leaves this loop matching nothing, which reads as "all clear" — the same
    // way the "network tests — ignored by default" comment did while none of
    // them were ignored.
    assert!(
        examined > 0,
        "no network-reaching test was recognised in source_resolver.rs; either \
         they are gone or the detector no longer matches how they call, and \
         this test is vacuous"
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
    let dockerfile = std::fs::read_to_string(repo_root().join("Dockerfile")).expect("Dockerfile");
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
    let dockerfile = std::fs::read_to_string(repo_root().join("Dockerfile")).expect("Dockerfile");
    if !dockerfile.contains("--locked") {
        return;
    }
    assert!(
        repo_root().join("Cargo.lock").is_file(),
        "the builder passes --locked but no Cargo.lock is committed"
    );
}

/// Both stages must exist and the slim one must stay the default.
///
/// Docker's default target is the *last* stage in the file, so appending a
/// stage silently changes what `docker build .` produces. The header says the
/// default is the CLI image; this keeps that true.
#[test]
fn the_slim_image_is_still_the_default_target() {
    let dockerfile = std::fs::read_to_string(repo_root().join("Dockerfile")).expect("Dockerfile");
    let stages: Vec<&str> = dockerfile
        .lines()
        .filter_map(|l| l.trim().strip_prefix("FROM "))
        .filter_map(|r| r.split(" AS ").nth(1))
        .collect();

    assert!(
        stages.contains(&"dev"),
        "no dev stage: the header documents `horus run`, which needs toolchains \
         the slim image does not have"
    );
    assert_eq!(
        stages.last(),
        Some(&"runtime"),
        "the last stage is what `docker build .` builds, and the header says \
         that is the slim CLI image; stages are {stages:?}"
    );
}

/// The header must not advertise against the slim image a command that image
/// cannot run.
///
/// It did: it showed `horus run` under `horus:cli`, and that image has no
/// cargo, so the command fails with "Rust toolchain not installed". Verified by
/// building both images and running the examples.
#[test]
fn the_slim_image_is_not_advertised_for_building_or_running() {
    let dockerfile = std::fs::read_to_string(repo_root().join("Dockerfile")).expect("Dockerfile");

    let cli_section = dockerfile.split("── horus:dev").next().unwrap_or_default();

    for command in [" horus:cli run", " horus:cli build", " horus:cli test"] {
        assert!(
            !cli_section.contains(command),
            "the slim image has no toolchains, so `{}` cannot work — it fails \
             with H060 \"Rust toolchain not installed\"",
            command.trim()
        );
    }
}

/// A container cannot set SCHED_FIFO without the capability, and HORUS is a
/// real-time framework. Measuring it in a container that lacks the capability
/// measures the container.
#[test]
fn the_dockerfile_documents_the_realtime_capability() {
    let dockerfile = std::fs::read_to_string(repo_root().join("Dockerfile")).expect("Dockerfile");
    assert!(
        dockerfile.contains("SYS_NICE"),
        "nothing tells the reader that real-time scheduling in a container needs \
         --cap-add=SYS_NICE; without it every RT node silently runs at normal \
         priority"
    );
}

/// The runtime stage runs `horus man` and `horus completion` to place the files
/// the installer would. Both have to exist, or the image fails to build at a
/// step that has nothing to do with what changed.
#[test]
fn commands_the_dockerfile_runs_exist() {
    let dockerfile = std::fs::read_to_string(repo_root().join("Dockerfile")).expect("Dockerfile");
    // Extract the subcommand from every `horus <word>` in the file rather than
    // testing for a fixed list. A `contains("horus man")` check is still true
    // after the command is renamed to `horus manpage`, and passed while the
    // Dockerfile named something that does not exist.
    let mut checked = 0usize;
    let mut missing = Vec::new();
    // Only RUN lines. The header quotes HORUS's own output — including
    // `horus hint [preflight] H060`, which is a diagnostic, not a subcommand —
    // and scanning comments turned that into a false failure.
    for line in dockerfile.lines().filter(|l| {
        l.trim_start().starts_with("RUN ")
            || l.trim_start().starts_with("&& ")
            || l.contains("&& horus ")
    }) {
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

/// A failed `git clone` must actually stop the install.
///
/// The guard used to read `if ! git clone ... 2>&1 | tail -1; then`. install.sh
/// sets `set -e` but not `set -o pipefail`, so the pipeline reports *tail*'s
/// status — always 0 — and the failure branch was unreachable. A dead clone
/// then fell through to `rm -rf "$HORUS_SRC_DIR"`, destroying a working cached
/// source tree and replacing it with an empty one, while printing
/// "Installation complete!".
#[test]
fn install_sh_tests_the_real_status_of_git_clone() {
    let text = install_sh();

    // A pipeline only masks the clone's status while `pipefail` is off, which is
    // the state install.sh is actually in (`set -e` alone, line 15). If it ever
    // turns pipefail on, piping is no longer the bug this guards.
    let pipefail_is_on = text.lines().any(|l| {
        shell_code(l).contains("set -o pipefail") || shell_code(l).contains("set -eo pipe")
    });

    let mut clones = 0;
    for (n, line) in text.lines().enumerate() {
        let code = shell_code(line);
        if !code.contains("git clone") {
            continue;
        }
        clones += 1;
        if !pipefail_is_on && clone_is_piped(code) {
            panic!(
                "install.sh:{} pipes git clone into another command, so the \
                 pipeline reports that command's status and the clone's failure \
                 is discarded (the script sets `set -e` but not \
                 `set -o pipefail`):\n  {}",
                n + 1,
                line.trim()
            );
        }
    }

    assert!(
        clones > 0,
        "no `git clone` found in install.sh's executable lines — either the \
         script no longer fetches source or this scan is broken, and the test \
         is vacuous"
    );

    // The cache directory is deleted before the fetched tree is moved into it,
    // so the fetch must be proven complete first.
    let marker = text
        .find("horus/Cargo.toml\" ]")
        .expect("install.sh must validate the fetched tree before caching it");
    let destructive = text
        .find("rm -rf \"$HORUS_SRC_DIR\"")
        .expect("install.sh clears the cache dir before moving the new tree in");
    assert!(
        marker < destructive,
        "install.sh deletes the cached source tree before checking that the \
         freshly fetched one is usable"
    );
}

/// install.sh must not assign to `TMPDIR`.
///
/// It did, and then `rm -rf`'d the directory. `TMPDIR` is exported on macOS and
/// in most CI images, so every child the script spawns afterwards — mktemp,
/// rustup's installer, cargo, rustc, cc/ld — inherited a scratch directory that
/// no longer existed, and the build died in the linker with an error that named
/// no cause the user could act on.
#[test]
fn install_sh_does_not_clobber_the_exported_tmpdir() {
    let text = install_sh();
    for (n, line) in text.lines().enumerate() {
        let code = line.split('#').next().unwrap_or("").trim();
        if code.starts_with("TMPDIR=") || code.starts_with("export TMPDIR=") {
            panic!(
                "install.sh:{} overwrites the exported POSIX TMPDIR:\n  {}",
                n + 1,
                line.trim()
            );
        }
    }
}

// ─── Running the installer for real ─────────────────────────────────────────

/// The front half of `install.sh`, executed.
///
/// `completion_install_contract.rs` runs the script from the rc-file detection
/// onwards and says why: "A text search over install.sh would have passed
/// happily on every one of the bugs above." Everything *before* that anchor —
/// resolving what to install, fetching the source, downloading the asset,
/// verifying it against SHA256SUMS, extracting it, and recording what landed —
/// had no execution coverage at all, in either file. The scans above read it as
/// a string and would keep passing while a real install did nothing at all.
///
/// So: real `bash`, a throw-away `$HOME`, and a PATH holding nothing but a
/// symlink farm of the tools the script is allowed to use plus stub `curl`,
/// `git` and `sudo`. The stubs serve a release built here — a tarball with a
/// `SHA256SUMS` computed the way `release.yml:185` computes one — and log every
/// URL and every git invocation, which is what makes "the binary and the source
/// came from the same tag" a thing a test can assert rather than a thing a
/// reviewer has to read for.
#[cfg(unix)]
mod runs_install_sh {
    use super::repo_root;
    use std::fs;
    use std::os::unix::fs::PermissionsExt;
    use std::path::{Path, PathBuf};
    use std::process::Command;

    /// Where `completion_install_contract.rs` begins its slice. Everything
    /// before it is this module's territory; the two slices meet here and
    /// cover the file between them with no gap.
    const TAIL_ANCHOR: &str = "# --- Which rc file does this shell read? ---";

    fn install_head() -> String {
        let src =
            fs::read_to_string(repo_root().join("install.sh")).expect("install.sh must exist");
        let at = src.find(TAIL_ANCHOR).unwrap_or_else(|| {
            panic!(
                "install.sh no longer contains the anchor {TAIL_ANCHOR:?} — which is also \
                 where completion_install_contract.rs starts its slice, so both files stop \
                 covering the script. Re-anchor them together."
            )
        });
        let head = src[..at].to_string();

        // The same vacuity guard the tail slice carries. Without it a moved
        // anchor degrades every test below into running a banner and exiting 0,
        // and the suite goes green on an installer that installs nothing.
        for needed in ["detect_os()", "SHA256SUMS", "HORUS_CACHE", "INSTALL_DIR"] {
            assert!(
                head.contains(needed),
                "the install.sh head slice does not contain {needed:?}, so these tests would \
                 pass without exercising the download, the checksum or the source cache"
            );
        }
        let lines = head.lines().count();
        assert!(
            lines > 150,
            "the install.sh head slice is {lines} lines; the anchor has moved up and this \
             module now covers almost nothing"
        );
        head
    }

    /// The asset name `install.sh` derives from `uname -s` / `uname -m`, and the
    /// one `release.yml`'s matrix publishes.
    fn asset_stem() -> String {
        let os = if cfg!(target_os = "macos") {
            "macos"
        } else {
            "linux"
        };
        let arch = match std::env::consts::ARCH {
            "x86_64" => "amd64",
            "aarch64" => "arm64",
            "arm" => "armv7",
            other => panic!(
                "release.yml publishes no asset for {other}, so install.sh has none to download"
            ),
        };
        format!("horus-{os}-{arch}")
    }

    fn asset_file() -> String {
        format!("{}.tar.gz", asset_stem())
    }

    /// Every external program the script is allowed to reach.
    ///
    /// The run inherits no PATH from the machine. That is what makes "tar and
    /// unzip are both missing" expressible at all, and it stops a test from
    /// silently finding the real `curl`, the real `cargo`, or a `sudo` that
    /// would actually install packages on the developer's laptop.
    const TOOLS: &[&str] = &[
        "uname",
        "mktemp",
        "grep",
        "sed",
        "awk",
        "head",
        "tail",
        "cut",
        "sort",
        "date",
        "mkdir",
        "rm",
        "mv",
        "cp",
        "chmod",
        "cat",
        "id",
        "dirname",
        "basename",
        "sha256sum",
        "shasum",
        "tar",
        "unzip",
        "gzip",
        "tr",
        "ls",
        "touch",
        "find",
        "wc",
        "env",
        "sh",
        "bash",
        "expr",
        "stat",
        "readlink",
        "printf",
        "echo",
        "true",
        "false",
        "xargs",
        "sleep",
        "mktemp",
    ];

    fn which(name: &str) -> Option<PathBuf> {
        std::env::split_paths(&std::env::var_os("PATH")?)
            .map(|d| d.join(name))
            .find(|p| {
                fs::metadata(p)
                    .map(|m| m.is_file() && m.permissions().mode() & 0o111 != 0)
                    .unwrap_or(false)
            })
    }

    fn write_exec(path: &Path, body: &str) {
        fs::write(path, body).expect("write stub");
        fs::set_permissions(path, fs::Permissions::from_mode(0o755)).expect("chmod stub");
    }

    /// A stable 40-hex commit id for a ref, so `install_manifest.toml`'s
    /// `commit` field can be checked for shape without a real repository.
    fn fake_sha(reference: &str) -> String {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for b in reference.bytes() {
            h ^= u64::from(b);
            h = h.wrapping_mul(0x0100_0000_01b3);
        }
        format!("{h:016x}{h:016x}{:08x}", (h >> 32) as u32)
    }

    /// `release.yml:185` is literally `sha256sum *.tar.gz *.zip > SHA256SUMS`,
    /// so produce the file the same way instead of formatting one by hand — a
    /// hand-rolled line with the wrong separator would make every checksum test
    /// pass for the wrong reason.
    fn sha256sums_line(dir: &Path, file: &str) -> Vec<u8> {
        let out = if which("sha256sum").is_some() {
            Command::new("sha256sum")
                .arg(file)
                .current_dir(dir)
                .output()
        } else {
            Command::new("shasum")
                .args(["-a", "256", file])
                .current_dir(dir)
                .output()
        }
        .expect("this test needs sha256sum or shasum, the two install.sh accepts");
        assert!(
            out.status.success(),
            "hashing the fake release asset failed"
        );
        out.stdout
    }

    const CURL_STUB: &str = r##"#!/bin/bash
# Stub curl. Two tables, both under $HORUS_FAKE:
#   routes     glob<TAB>file          what a GET serves
#   redirects  glob<TAB>url           where a request lands, for %{url_effective}
# Every URL asked for is logged, which is what lets a test assert which release
# a run actually reached for rather than reading the script for it.
set -u
out=""
fmt=""
url=""
expect=""
for a in "$@"; do
    case "$expect" in
        o) out="$a"; expect=""; continue ;;
        w) fmt="$a"; expect=""; continue ;;
    esac
    case "$a" in
        -o|--output) expect=o ;;
        -w|--write-out) expect=w ;;
        http://*|https://*) url="$a" ;;
        *) ;;
    esac
done

printf '%s\n' "$url" >> "${HORUS_FAKE}/curl.log"

# install.sh resolves the newest release from the /releases/latest redirect
# rather than the API, because the API rate-limits per IP and answers 403 with
# a JSON body a shell will happily parse a version out of.
case "$fmt" in
    *url_effective*)
        if [ -f "${HORUS_FAKE}/redirects" ]; then
            while IFS=$'\t' read -r glob dest; do
                [ -n "${glob:-}" ] || continue
                case "$url" in
                    $glob) printf '%s' "$dest"; exit 0 ;;
                esac
            done < "${HORUS_FAKE}/redirects"
        fi
        exit 22
        ;;
esac

target=""
while IFS=$'\t' read -r glob file; do
    [ -n "${glob:-}" ] || continue
    case "$url" in
        $glob) target="$file"; break ;;
    esac
done < "${HORUS_FAKE}/routes"

if [ -n "$target" ] && [ -f "$target" ]; then
    if [ -n "$out" ]; then cp "$target" "$out"; else cat "$target"; fi
    case "$fmt" in *http_code*) printf '200' ;; esac
    exit 0
fi

# What real curl does on an HTTP error with -f: nothing on stdout but the -w
# format, and exit 22. install.sh's `|| echo "000"` fallback sees exactly this.
case "$fmt" in *http_code*) printf '404' ;; esac
exit 22
"##;

    const GIT_STUB: &str = r##"#!/bin/bash
# Stub git. Clones come out of $HORUS_FAKE/refs/<ref>, so a test can publish a
# different source tree per tag and then assert which ref the installer took.
set -u
printf '%s\n' "$*" >> "${HORUS_FAKE}/git.log"

repo_dir="."
while [ $# -gt 0 ]; do
    case "$1" in
        -C) repo_dir="$2"; shift 2 ;;
        -c) shift 2 ;;
        --version) echo "git version 2.99.0 (install_contract stub)"; exit 0 ;;
        -*) shift ;;
        *) break ;;
    esac
done
cmd="${1:-}"
if [ $# -gt 0 ]; then shift; fi

case "$cmd" in
    clone)
        ref=""
        url=""
        dest=""
        while [ $# -gt 0 ]; do
            case "$1" in
                -b|--branch) ref="$2"; shift 2 ;;
                --branch=*) ref="${1#--branch=}"; shift ;;
                --depth|--filter|--jobs|-j) shift 2 ;;
                --depth=*|--filter=*) shift ;;
                -*) shift ;;
                *)
                    if [ -z "$url" ]; then url="$1"; else dest="$1"; fi
                    shift
                    ;;
            esac
        done
        if [ -z "$ref" ]; then
            ref="$(cat "${HORUS_FAKE}/default_ref" 2>/dev/null || echo HEAD)"
        fi
        if [ ! -d "${HORUS_FAKE}/refs/${ref}" ]; then
            echo "fatal: Remote branch ${ref} not found in upstream origin" >&2
            exit 128
        fi
        if [ -z "$dest" ]; then dest="$(basename "$url" .git)"; fi
        mkdir -p "$dest"
        cp -R "${HORUS_FAKE}/refs/${ref}/." "${dest}/"
        mkdir -p "${dest}/.git"
        printf '%s\n' "$ref" > "${dest}/.git/horus_ref"
        cat "${HORUS_FAKE}/shas/${ref}" > "${dest}/.git/horus_sha" 2>/dev/null || true
        exit 0
        ;;
    rev-parse)
        case "${1:-HEAD}" in
            --abbrev-ref*) cat "${repo_dir}/.git/horus_ref" 2>/dev/null || exit 128 ;;
            *) cat "${repo_dir}/.git/horus_sha" 2>/dev/null || exit 128 ;;
        esac
        exit 0
        ;;
    describe)
        cat "${repo_dir}/.git/horus_ref" 2>/dev/null || exit 128
        exit 0
        ;;
    ls-remote)
        cat "${HORUS_FAKE}/ls-remote" 2>/dev/null || exit 128
        exit 0
        ;;
    checkout|switch)
        want=""
        for a in "$@"; do
            case "$a" in -*) ;; *) want="$a"; break ;; esac
        done
        if [ ! -d "${HORUS_FAKE}/refs/${want}" ]; then
            echo "fatal: reference is not a tree: ${want}" >&2
            exit 128
        fi
        printf '%s\n' "$want" > "${repo_dir}/.git/horus_ref" 2>/dev/null || true
        exit 0
        ;;
    *)
        exit 0
        ;;
esac
"##;

    const SUDO_STUB: &str = r##"#!/bin/bash
# Nothing in these tests may touch the machine's package manager.
echo "install_contract: refusing 'sudo $*' inside the test sandbox" >&2
exit 1
"##;

    struct Run {
        code: Option<i32>,
        output: String,
    }

    impl Run {
        fn ok(&self) -> bool {
            self.code == Some(0)
        }
    }

    struct Rig {
        home: tempfile::TempDir,
        rig: tempfile::TempDir,
    }

    impl Rig {
        fn new() -> Self {
            let rig = Rig {
                home: tempfile::tempdir().expect("tempdir"),
                rig: tempfile::tempdir().expect("tempdir"),
            };
            for dir in ["bin", "server/refs", "server/shas", "server/files", "tmp"] {
                fs::create_dir_all(rig.rig.path().join(dir)).expect("mkdir");
            }
            fs::write(rig.server().join("routes"), "").expect("routes");
            fs::write(rig.server().join("redirects"), "").expect("redirects");
            write_exec(&rig.rig.path().join("bin/curl"), CURL_STUB);
            write_exec(&rig.rig.path().join("bin/git"), GIT_STUB);
            write_exec(&rig.rig.path().join("bin/sudo"), SUDO_STUB);
            rig
        }

        fn home(&self) -> &Path {
            self.home.path()
        }

        fn server(&self) -> PathBuf {
            self.rig.path().join("server")
        }

        fn route(&self, glob: &str, file: &Path) {
            let routes = self.server().join("routes");
            let mut table = fs::read_to_string(&routes).unwrap_or_default();
            table.push_str(&format!("{glob}\t{}\n", file.display()));
            fs::write(&routes, table).expect("routes");
        }

        /// Where a request lands, for `curl -I -w '%{url_effective}'`.
        fn redirect(&self, glob: &str, destination: &str) {
            let table_path = self.server().join("redirects");
            let mut table = fs::read_to_string(&table_path).unwrap_or_default();
            table.push_str(&format!("{glob}\t{destination}\n"));
            fs::write(&table_path, table).expect("redirects");
        }

        /// The source tree a ref points at: the three files install.sh reads out
        /// of a fetched checkout, plus the constant that decides whether a CLI
        /// and its libraries can talk at all.
        fn publish_branch(&self, reference: &str, version: &str, topic_version: u32) {
            let root = self.server().join("refs").join(reference);
            fs::create_dir_all(root.join("horus")).expect("mkdir");
            fs::create_dir_all(root.join("horus_core/src/communication/topic")).expect("mkdir");
            // The floor is pinned at 1.0.0 rather than the real MSRV on purpose:
            // these tests are about the download, the checksum and the recorded
            // state, and must not start failing on a machine whose rustc is a
            // release behind. `install_sh_checks_the_rust_version_before_building`
            // above is what guards the floor itself.
            fs::write(
                root.join("Cargo.toml"),
                "[workspace]\nmembers = [\"horus\", \"horus_core\"]\n\n[workspace.package]\nrust-version = \"1.0.0\"\n",
            )
            .expect("write");
            fs::write(
                root.join("horus/Cargo.toml"),
                format!("[package]\nname = \"horus\"\nversion = \"{version}\"\n"),
            )
            .expect("write");
            fs::write(
                root.join("horus_core/Cargo.toml"),
                format!("[package]\nname = \"horus_core\"\nversion = \"{version}\"\n"),
            )
            .expect("write");
            // What install.sh's final skew check compares `horus --version`
            // against before it will accept the install.
            fs::create_dir_all(root.join("horus_manager")).expect("mkdir");
            fs::write(
                root.join("horus_manager/Cargo.toml"),
                format!("[package]\nname = \"horus_manager\"\nversion = \"{version}\"\n"),
            )
            .expect("write");
            fs::write(
                root.join("horus_core/src/communication/topic/header.rs"),
                format!("pub(crate) const TOPIC_VERSION: u32 = {topic_version};\n"),
            )
            .expect("write");
            fs::write(
                self.server().join("shas").join(reference),
                format!("{}\n", fake_sha(reference)),
            )
            .expect("write");
        }

        /// A release as `release.yml` publishes one: the tagged source tree, a
        /// tarball holding a single `horus`, and a `SHA256SUMS` over it.
        fn publish(&self, tag: &str, version: &str, topic_version: u32) {
            self.publish_branch(tag, version, topic_version);

            let build = self.server().join("build").join(tag);
            fs::create_dir_all(&build).expect("mkdir");
            // A stub, for the same reason completion_install_contract.rs uses
            // one: these tests are about what the installer does with the asset,
            // not about the binary inside it. It answers --version because
            // install.sh verifies the install by asking.
            write_exec(
                &build.join("horus"),
                &format!(
                    "#!/bin/bash\ncase \"${{1:-}}\" in\n  \
                     --version|-V) echo 'horus {version}' ;;\n  \
                     completion) echo '# stub completion' ;;\n  \
                     man) echo '.TH horus 1' ;;\n  \
                     env) echo 'stub env' ;;\n  \
                     *) exit 1 ;;\nesac\n"
                ),
            );

            let files = self.server().join("files").join(tag);
            fs::create_dir_all(&files).expect("mkdir");
            let asset = asset_file();
            let packed = Command::new("tar")
                .arg("czf")
                .arg(files.join(&asset))
                .arg("-C")
                .arg(&build)
                .arg("horus")
                .status()
                .expect("this test needs tar to build the fake release");
            assert!(packed.success(), "packing the fake release asset failed");
            fs::write(files.join("SHA256SUMS"), sha256sums_line(&files, &asset)).expect("write");

            // SHA256SUMS first: `*/releases/download/<tag>/*` would otherwise
            // swallow it and serve the tarball as the checksum file.
            self.route(
                &format!("*/releases/download/{tag}/SHA256SUMS"),
                &files.join("SHA256SUMS"),
            );
            self.route(&format!("*/releases/download/{tag}/*"), &files.join(&asset));
        }

        /// Make `tag` the newest release, by every route a script might use to
        /// find that out: the `releases/latest/download` redirect, the API, and
        /// `git ls-remote --tags`.
        fn set_latest(&self, tag: &str) {
            let files = self.server().join("files").join(tag);
            self.route(
                "*/releases/latest/download/SHA256SUMS",
                &files.join("SHA256SUMS"),
            );
            self.route("*/releases/latest/download/*", &files.join(asset_file()));

            // The route install.sh takes: /releases/latest redirects to
            // /releases/tag/<T>, which is not rate limited and names the tag in
            // the URL rather than in a JSON body.
            self.redirect(
                "*/releases/latest",
                &format!("https://github.com/softmata/horus/releases/tag/{tag}"),
            );

            let api = self.server().join("api-latest.json");
            fs::write(
                &api,
                format!(
                    "{{\"tag_name\":\"{tag}\",\"name\":\"{tag}\",\"draft\":false,\"prerelease\":false}}\n"
                ),
            )
            .expect("write");
            self.route("https://api.github.com/repos/*/releases/latest", &api);
            fs::write(
                self.server().join("ls-remote"),
                format!("{}\trefs/tags/{tag}\n", fake_sha(tag)),
            )
            .expect("write");
            fs::write(self.server().join("default_ref"), tag).expect("write");
        }

        /// Flip a byte of the published asset *after* SHA256SUMS was computed
        /// over it: a tampered or truncated download, which is the case TLS does
        /// not cover and the whole reason install.sh fetches SHA256SUMS.
        fn corrupt_asset(&self, tag: &str) {
            let path = self.server().join("files").join(tag).join(asset_file());
            let mut bytes = fs::read(&path).expect("read asset");
            let last = bytes.len() - 1;
            bytes[last] ^= 0xff;
            fs::write(&path, bytes).expect("write asset");
        }

        /// The release is there, its checksum file is not — a partial upload, or
        /// a release cut before `release.yml`'s "Compute checksums" step ran.
        ///
        /// A dead route in front of the table rather than a deleted one: with
        /// the SHA256SUMS entry merely removed, `*/releases/download/<tag>/*`
        /// catches the request and hands back the tarball, which install.sh
        /// rejects for a different reason ("SHA256SUMS has no entry for ...")
        /// and the test would be checking the wrong branch.
        fn unpublish_checksums(&self) {
            let routes = self.server().join("routes");
            let existing = fs::read_to_string(&routes).unwrap_or_default();
            let missing = self.server().join("this-file-was-never-uploaded");
            fs::write(
                &routes,
                format!("*SHA256SUMS\t{}\n{existing}", missing.display()),
            )
            .expect("routes");
        }

        fn run(&self, env: &[(&str, &str)]) -> Run {
            self.run_without(&[], env)
        }

        fn run_without(&self, missing: &[&str], env: &[(&str, &str)]) -> Run {
            let tools = self.rig.path().join("tools");
            let _ = fs::remove_dir_all(&tools);
            fs::create_dir_all(&tools).expect("mkdir");
            for tool in TOOLS {
                if missing.contains(tool) || tools.join(tool).exists() {
                    continue;
                }
                if let Some(real) = which(tool) {
                    std::os::unix::fs::symlink(&real, tools.join(tool)).expect("symlink");
                }
            }
            for required in [
                "uname", "mktemp", "grep", "sed", "awk", "date", "mv", "chmod",
            ] {
                assert!(
                    missing.contains(&required) || tools.join(required).exists(),
                    "this test needs `{required}` on the machine running it"
                );
            }

            let script = self.rig.path().join("install-head.sh");
            fs::write(&script, install_head()).expect("write slice");

            let path = format!(
                "{}:{}",
                self.rig.path().join("bin").display(),
                tools.display()
            );
            let mut cmd = Command::new("bash");
            cmd.arg(&script)
                .current_dir(self.rig.path())
                .env_clear()
                .env("HOME", self.home())
                .env("PATH", &path)
                // mktemp, cc/ld, cargo and rustup all read TMPDIR. Point it
                // inside the rig so a run leaves nothing in the machine's /tmp —
                // and so install.sh assigning to it (which it must not; see
                // install_sh_does_not_clobber_the_exported_tmpdir) shows up here
                // as a broken run rather than as someone else's mystery.
                .env("TMPDIR", self.rig.path().join("tmp"))
                .env("SHELL", "/bin/bash")
                .env("TERM", "dumb")
                .env("HORUS_FAKE", self.server())
                // The rc-file half is completion_install_contract.rs's subject
                // and is not in this slice; opt out anyway so a refactor that
                // moves shell integration earlier cannot start editing rc files
                // from under these tests.
                .env("HORUS_NO_SHELL_INTEGRATION", "1");
            for (k, v) in env {
                cmd.env(k, v);
            }
            let out = cmd.output().expect("bash must run");
            Run {
                code: out.status.code(),
                output: format!(
                    "{}{}",
                    String::from_utf8_lossy(&out.stdout),
                    String::from_utf8_lossy(&out.stderr)
                ),
            }
        }

        /// Wherever `find_install_dir` decided to put it.
        fn installed_binary(&self) -> Option<PathBuf> {
            [".cargo/bin/horus", ".local/bin/horus"]
                .iter()
                .map(|rel| self.home().join(rel))
                .find(|p| p.is_file())
        }

        /// The `horus@<version>` directories under `~/.horus/cache` — the exact
        /// names `run_rust.rs:1062` rebuilds from CARGO_PKG_VERSION.
        fn cached_sources(&self) -> Vec<String> {
            let mut found: Vec<String> = fs::read_dir(self.home().join(".horus/cache"))
                .into_iter()
                .flatten()
                .flatten()
                .map(|e| e.file_name().to_string_lossy().into_owned())
                .collect();
            found.sort();
            found
        }

        fn curl_log(&self) -> String {
            fs::read_to_string(self.server().join("curl.log")).unwrap_or_default()
        }

        fn git_log(&self) -> String {
            fs::read_to_string(self.server().join("git.log")).unwrap_or_default()
        }

        /// URLs the run fetched that are release *assets*, as opposed to
        /// whatever it used to work out which release is current.
        fn asset_downloads(&self) -> Vec<String> {
            self.curl_log()
                .lines()
                .filter(|u| u.contains("/releases/") && u.contains("/download/"))
                .map(str::to_string)
                .collect()
        }
    }

    /// The whole point, stated once: a release whose checksum matches installs,
    /// and what lands is a binary that runs.
    ///
    /// Nothing asserted this. `install.sh` could have downloaded a 404 page,
    /// chmod +x'd it and printed "Installation complete!" and every test in this
    /// file would still have passed.
    #[test]
    fn a_verified_release_installs_a_binary_that_runs() {
        let rig = Rig::new();
        rig.publish("v0.4.0", "0.4.0", 4);
        rig.publish_branch("main", "0.4.0", 4);
        rig.set_latest("v0.4.0");

        let run = rig.run(&[]);
        assert!(
            run.ok(),
            "a clean install of a release whose checksum matches exited {:?}:\n{}",
            run.code,
            run.output
        );

        let binary = rig.installed_binary().unwrap_or_else(|| {
            panic!(
                "install.sh exited 0 but wrote no horus under $HOME/.cargo/bin or \
                 $HOME/.local/bin:\n{}",
                run.output
            )
        });
        let mode = fs::metadata(&binary).expect("stat").permissions().mode();
        assert!(
            mode & 0o111 != 0,
            "{} is mode {:o}: the download landed but nothing can execute it",
            binary.display(),
            mode & 0o7777
        );
        let version = Command::new(&binary)
            .arg("--version")
            .output()
            .expect("run the freshly installed binary");
        assert!(
            version.status.success(),
            "`horus --version` on the freshly installed binary exits {:?}; install.sh reported \
             success anyway",
            version.status.code()
        );

        // A binary-only install produces a CLI that cannot build a single Rust
        // project: `horus run` writes .horus/Cargo.toml with horus as *path*
        // dependencies (cargo_gen.rs find_horus_source_dir).
        assert!(
            !rig.cached_sources().is_empty(),
            "nothing was cached under ~/.horus/cache, so no Rust project can be built:\n{}",
            run.output
        );
    }

    /// A byte-flipped asset must stop the install dead.
    ///
    /// TLS does not cover a substituted or truncated artifact, which is why
    /// `release.yml` publishes SHA256SUMS and SECURITY.md promises "checksum
    /// verification". The installer went years fetching neither.
    #[test]
    fn a_tampered_asset_is_refused_and_installs_nothing() {
        let rig = Rig::new();
        rig.publish("v0.4.0", "0.4.0", 4);
        rig.publish_branch("main", "0.4.0", 4);
        rig.set_latest("v0.4.0");
        rig.corrupt_asset("v0.4.0");

        let run = rig.run(&[]);
        assert!(
            !run.ok(),
            "install.sh accepted an asset that does not match the published SHA256SUMS:\n{}",
            run.output
        );
        assert!(
            rig.installed_binary().is_none(),
            "the checksum did not match and a binary was installed anyway:\n{}",
            run.output
        );
        assert!(
            run.output.to_lowercase().contains("checksum"),
            "the run failed, but nothing in its output says the checksum was the reason — a \
             user cannot tell a tampered download from a broken one:\n{}",
            run.output
        );
    }

    /// No SHA256SUMS means nothing to verify against, and the fail-closed answer
    /// is to refuse. Falling back to "install it anyway" turns a missing file on
    /// the release page into an unverified binary on every machine.
    #[test]
    fn a_missing_checksum_file_refuses_the_install() {
        let rig = Rig::new();
        rig.publish("v0.4.0", "0.4.0", 4);
        rig.publish_branch("main", "0.4.0", 4);
        rig.set_latest("v0.4.0");
        rig.unpublish_checksums();

        let run = rig.run(&[]);
        assert!(
            !run.ok(),
            "SHA256SUMS 404'd and install.sh installed the binary anyway:\n{}",
            run.output
        );
        assert!(
            rig.installed_binary().is_none(),
            "SHA256SUMS 404'd and a binary was installed anyway:\n{}",
            run.output
        );
        // The escape hatch has to be named, or the only way out of a broken
        // release page is to read the script.
        assert!(
            run.output.contains("HORUS_BUILD_FROM_SOURCE"),
            "the install refused without telling the user how to proceed:\n{}",
            run.output
        );
    }

    /// With no extractor the install must fail, not half-finish.
    ///
    /// On this platform it is `tar` the script reaches for and `unzip` only on
    /// Windows, so both go: the test then says what it means on either. The
    /// failure mode being guarded is the one where the archive cannot be opened,
    /// the `mv` into INSTALL_DIR is skipped, and the script carries on to report
    /// a successful install of a file that is not there.
    #[test]
    fn an_install_with_no_extractor_fails_instead_of_half_finishing() {
        let rig = Rig::new();
        rig.publish("v0.4.0", "0.4.0", 4);
        rig.publish_branch("main", "0.4.0", 4);
        rig.set_latest("v0.4.0");

        let run = rig.run_without(&["tar", "unzip"], &[]);
        assert!(
            !run.ok(),
            "neither tar nor unzip is on PATH and install.sh exited 0:\n{}",
            run.output
        );
        assert!(
            rig.installed_binary().is_none(),
            "the archive could not be opened and a horus was installed anyway:\n{}",
            run.output
        );
        assert!(
            !run.output.contains("Verified:"),
            "install.sh announced a verified install it could not have performed:\n{}",
            run.output
        );
        let said = run.output.to_lowercase();
        assert!(
            said.contains("tar") || said.contains("unzip") || said.contains("extract"),
            "the install died with nothing in its output naming the missing extractor, so the \
             user has no idea what to install:\n{}",
            run.output
        );
    }

    /// The install has to write down what it installed (contract §2).
    ///
    /// `version.rs:12` reads `~/.horus/installed_version` and compares it with
    /// the CLI's own version. Nothing has written that file since v0.2.0, so the
    /// gate has been dead code — and the remedy the mismatch message printed
    /// (re-run install.sh) could not have fixed it, because install.sh was not a
    /// writer either.
    #[test]
    fn the_install_records_the_version_it_installed() {
        let rig = Rig::new();
        rig.publish("v0.4.0", "0.4.0", 4);
        rig.publish_branch("main", "0.4.0", 4);
        rig.set_latest("v0.4.0");

        let run = rig.run(&[]);
        assert!(run.ok(), "install failed:\n{}", run.output);

        let recorded = fs::read_to_string(rig.home().join(".horus/installed_version"))
            .unwrap_or_else(|e| {
                panic!(
                    "~/.horus/installed_version was not written ({e}), so version.rs sees \"no \
                     version file\" after a successful install and its gate never fires:\n{}",
                    run.output
                )
            });
        assert_eq!(
            recorded.trim(),
            "0.4.0",
            "installed_version must be the bare version string version.rs:12 trims and \
             compares, not {recorded:?}"
        );
    }

    /// The richer record, and specifically the field that matters.
    ///
    /// A version string is not enough to catch RC1: both halves said "0.4.0"
    /// while TOPIC_VERSION was 3 on one and 4 on the other. `topic_version` in
    /// the manifest is the thing that actually breaks, so it has to be recorded.
    #[test]
    fn the_install_records_a_manifest_naming_the_tag_and_the_topic_version() {
        let rig = Rig::new();
        rig.publish("v0.4.0", "0.4.0", 4);
        rig.publish_branch("main", "0.4.0", 4);
        rig.set_latest("v0.4.0");

        let run = rig.run(&[]);
        assert!(run.ok(), "install failed:\n{}", run.output);

        let path = rig.home().join(".horus/install_manifest.toml");
        let text = fs::read_to_string(&path).unwrap_or_else(|e| {
            panic!(
                "~/.horus/install_manifest.toml was not written ({e}); version.rs and doctor.rs \
                 then have nothing richer than a version string to compare:\n{}",
                run.output
            )
        });
        let manifest: toml::Value = toml::from_str(&text)
            .unwrap_or_else(|e| panic!("install_manifest.toml is not TOML ({e}):\n{text}"));

        let field = |key: &str| {
            manifest
                .get(key)
                .and_then(toml::Value::as_str)
                .unwrap_or("")
        };
        assert_eq!(field("version"), "0.4.0", "manifest:\n{text}");
        assert_eq!(field("tag"), "v0.4.0", "manifest:\n{text}");
        assert_eq!(
            field("install_method"),
            "release-binary",
            "a downloaded release binary is not a source build; manifest:\n{text}"
        );
        assert_eq!(
            manifest
                .get("topic_version")
                .and_then(toml::Value::as_integer),
            Some(4),
            "topic_version is the field that decides whether a CLI can read the shm its own \
             libraries write, and it is the one RC1 got wrong while every version string \
             agreed; manifest:\n{text}"
        );

        let commit = field("commit");
        assert!(
            commit.len() == 40 && commit.chars().all(|c| c.is_ascii_hexdigit()),
            "commit must be the 40-hex sha of the source tree that was installed, got \
             {commit:?}; manifest:\n{text}"
        );
        assert!(
            Path::new(field("source_dir")).is_dir(),
            "source_dir {:?} does not exist, so anything that trusts the manifest to find the \
             cached tree is pointed at nothing; manifest:\n{text}",
            field("source_dir")
        );
        assert!(
            Path::new(field("binary")).is_file(),
            "binary {:?} does not exist; manifest:\n{text}",
            field("binary")
        );
        let installed_at = manifest
            .get("installed_at")
            .map(toml::Value::to_string)
            .unwrap_or_default();
        assert!(
            installed_at.contains('T'),
            "installed_at must be an RFC3339 timestamp, got {installed_at:?}; manifest:\n{text}"
        );
    }

    /// RC1, as a test.
    ///
    /// install.sh downloaded the binary from `releases/latest/download/` — a tag
    /// — and cloned the source from `main`. Both trees said `version = "0.4.0"`,
    /// so nothing looked wrong to anyone reading either half, while TOPIC_VERSION
    /// was 3 at the tag and 4 on main, 93 commits apart. The installed CLI could
    /// not read the shared memory its own libraries wrote.
    ///
    /// One resolved tag drives both halves, or this fails.
    #[test]
    fn the_binary_and_the_cached_source_come_from_the_same_tag() {
        let rig = Rig::new();
        rig.publish("v0.4.0", "0.4.0", 4);
        // main has moved on since the tag, exactly as it had when RC1 shipped.
        rig.publish_branch("main", "0.5.0-dev", 5);
        rig.set_latest("v0.4.0");

        let run = rig.run(&[]);
        assert!(run.ok(), "a default install failed:\n{}", run.output);

        assert_eq!(
            rig.cached_sources(),
            vec!["horus@0.4.0".to_string()],
            "the newest release is v0.4.0 and the binary came from it, but ~/.horus/cache holds \
             {:?} — the two halves of the install are at different refs, which is the mismatch \
             that breaks the shm handshake.\ngit calls:\n{}",
            rig.cached_sources(),
            rig.git_log()
        );
        assert!(
            !rig.git_log().contains("main"),
            "a tagged binary was installed while the source was fetched from main:\n{}",
            rig.git_log()
        );

        let binary = rig.installed_binary().expect("a binary was installed");
        let reported = Command::new(&binary)
            .arg("--version")
            .output()
            .expect("run the installed binary");
        let reported = String::from_utf8_lossy(&reported.stdout).trim().to_string();
        assert!(
            reported.contains("0.4.0"),
            "the installed binary reports {reported:?} while the cache holds {:?}",
            rig.cached_sources()
        );
    }

    /// `HORUS_VERSION` pins both halves, with or without the leading `v`.
    ///
    /// A pin that resolves through `releases/latest` is not a pin: it is
    /// whatever the newest release happens to be at the moment the script runs,
    /// which is the failure this variable exists to make impossible.
    #[test]
    fn horus_version_pins_both_halves_to_that_tag() {
        let rig = Rig::new();
        rig.publish("v0.3.0", "0.3.0", 3);
        rig.publish("v0.4.0", "0.4.0", 4);
        rig.publish_branch("main", "0.5.0-dev", 5);
        rig.set_latest("v0.4.0");

        // Without the leading `v`, which the documented interface accepts.
        let run = rig.run(&[("HORUS_VERSION", "0.3.0")]);
        assert!(
            run.ok(),
            "HORUS_VERSION=0.3.0 (no leading v) failed:\n{}",
            run.output
        );
        assert_eq!(
            rig.cached_sources(),
            vec!["horus@0.3.0".to_string()],
            "HORUS_VERSION=0.3.0 cached {:?}\ngit calls:\n{}",
            rig.cached_sources(),
            rig.git_log()
        );
        for url in rig.asset_downloads() {
            assert!(
                !url.contains("/releases/latest/"),
                "HORUS_VERSION was pinned to 0.3.0 and the installer still fetched {url}, which \
                 serves whatever the newest release is"
            );
        }
        let binary = rig.installed_binary().expect("a binary was installed");
        let reported = Command::new(&binary)
            .arg("--version")
            .output()
            .expect("run it");
        assert!(
            String::from_utf8_lossy(&reported.stdout).contains("0.3.0"),
            "pinned to 0.3.0, installed {:?}",
            String::from_utf8_lossy(&reported.stdout)
        );
    }

    /// The developer escape hatch must not be combinable with a release binary.
    ///
    /// That combination *is* RC1: a binary cut from a tag against a source tree
    /// from a branch. `HORUS_INSTALL_BRANCH` therefore implies
    /// `HORUS_BUILD_FROM_SOURCE=1`, and a run that reaches for a prebuilt asset
    /// anyway has reintroduced the bug.
    #[test]
    fn installing_from_a_branch_never_downloads_a_release_binary() {
        let rig = Rig::new();
        rig.publish("v0.4.0", "0.4.0", 4);
        rig.publish_branch("wip", "0.5.0-dev", 5);
        rig.set_latest("v0.4.0");

        // No cargo in the tool farm, so a source build cannot finish here. What
        // the run must not do is reach for a prebuilt binary on the way to
        // failing.
        let run = rig.run(&[("HORUS_INSTALL_BRANCH", "wip")]);

        assert!(
            rig.asset_downloads().is_empty(),
            "HORUS_INSTALL_BRANCH=wip builds from that branch's source, but the run also \
             downloaded {:?} — a tagged binary against a branch source tree is exactly the \
             mismatch this variable must not be able to create.\n{}",
            rig.asset_downloads(),
            run.output
        );
        assert!(
            !run.ok(),
            "a source build with no cargo on PATH reported success:\n{}",
            run.output
        );
        assert!(
            rig.installed_binary().is_none(),
            "no toolchain was available and a binary was installed anyway:\n{}",
            run.output
        );
    }

    /// `HORUS_PREFIX` has to move the whole install, not just the binary.
    ///
    /// The state files are what `version.rs` and `horus self update` read to
    /// decide what is installed, and the cache is what `horus run` builds user
    /// projects against. A prefix that relocated the binary and left those in
    /// `~/.horus` would describe an install that is not where it says it is —
    /// and under `sudo bash`, `~` is /root.
    #[test]
    fn horus_prefix_relocates_the_binary_and_the_state_together() {
        let rig = Rig::new();
        rig.publish("v0.4.0", "0.4.0", 4);
        rig.set_latest("v0.4.0");

        let prefix = rig.home().join("opt/horus");
        let prefix_arg = prefix.display().to_string();
        let run = rig.run(&[("HORUS_PREFIX", prefix_arg.as_str())]);
        assert!(
            run.ok(),
            "an install into HORUS_PREFIX={prefix_arg} failed:\n{}",
            run.output
        );

        for expected in ["bin/horus", "installed_version", "install_manifest.toml"] {
            assert!(
                prefix.join(expected).is_file(),
                "HORUS_PREFIX={prefix_arg} but {expected} is not under it:\n{}",
                run.output
            );
        }
        assert!(
            prefix.join("cache/horus@0.4.0").exists(),
            "the source cache did not follow HORUS_PREFIX, so `horus run` looks for its path \
             dependencies somewhere the installer never wrote:\n{}",
            run.output
        );
        assert!(
            !rig.home().join(".horus").exists(),
            "HORUS_PREFIX was set and the installer wrote to ~/.horus anyway:\n{}",
            run.output
        );
        assert!(
            rig.installed_binary().is_none(),
            "HORUS_PREFIX was set and a binary was installed under $HOME as well:\n{}",
            run.output
        );
    }

    /// The harness itself, checked.
    ///
    /// Everything above is worth exactly as much as the rig is real: if `curl`
    /// silently served nothing, or `git` cloned an empty tree, the failure tests
    /// would still fail for the right-looking reason and the success tests would
    /// be the only ones to notice. Prove the rig serves what it claims before
    /// trusting a red run to mean install.sh is wrong.
    #[test]
    fn the_rig_serves_a_release_the_installer_can_actually_verify() {
        let rig = Rig::new();
        rig.publish("v0.4.0", "0.4.0", 4);
        rig.set_latest("v0.4.0");

        let files = rig.server().join("files/v0.4.0");
        let sums = fs::read_to_string(files.join("SHA256SUMS")).expect("SHA256SUMS");
        assert!(
            sums.contains(&asset_file()),
            "the published SHA256SUMS does not name {}: {sums:?}",
            asset_file()
        );
        // install.sh greps ` <asset>$` out of it; a one-space separator would
        // make every checksum comparison below silently unreachable.
        assert!(
            sums.contains(&format!(" {}", asset_file())),
            "SHA256SUMS is not in `sha256sum` output format, which is what install.sh greps: \
             {sums:?}"
        );

        let source = rig.server().join("refs/v0.4.0");
        for marker in ["horus/Cargo.toml", "horus_core/Cargo.toml", "Cargo.toml"] {
            assert!(
                source.join(marker).is_file(),
                "the published source tree has no {marker}, which install.sh checks for before \
                 it will cache a fetched tree"
            );
        }
    }
}
