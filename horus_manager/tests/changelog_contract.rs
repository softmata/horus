//! The project must record what changed between releases.
//!
//! HORUS shipped nine tagged releases — `alpha` through `v0.2.2`, spanning
//! 2025-11 to 2026-07 — with no CHANGELOG. A team evaluating an upgrade from
//! 0.1.x to 0.2.x had no way to learn what moved except by reading ~2,500
//! commits, and no way at all to find out whether a fix they needed had landed.
//!
//! For a framework businesses build robots on, "what is in this release" is not
//! a nicety; it is the input to a decision about whether to take it.
//!
//! These tests keep the file honest going forward: every tagged version must
//! appear, and the version currently being built must have somewhere to record
//! its changes.
//!
//! Run: `cargo test -p horus_manager --test changelog_contract`

use std::path::{Path, PathBuf};
use std::process::Command;

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager must have a parent")
        .to_path_buf()
}

fn changelog() -> String {
    std::fs::read_to_string(repo_root().join("CHANGELOG.md"))
        .expect("CHANGELOG.md must exist at the repository root")
}

/// Tags as git reports them, empty when git is unavailable (e.g. a vendored
/// source tarball). A missing git is reported, not silently passed.
fn git_tags() -> Vec<String> {
    let out = Command::new("git")
        .args(["tag"])
        .current_dir(repo_root())
        .output();

    match out {
        Ok(o) if o.status.success() => String::from_utf8_lossy(&o.stdout)
            .lines()
            .map(|l| l.trim().to_string())
            .filter(|l| !l.is_empty())
            .collect(),
        _ => Vec::new(),
    }
}

#[test]
fn a_changelog_exists() {
    let text = changelog();
    assert!(
        text.len() > 200,
        "CHANGELOG.md exists but is nearly empty ({} bytes)",
        text.len()
    );
}

/// Every released tag must have a section. A release nobody wrote down is a
/// release nobody can evaluate.
#[test]
fn every_released_tag_appears_in_the_changelog() {
    let tags = git_tags();
    if tags.is_empty() {
        eprintln!("SKIP: no git tags visible (not a git checkout?)");
        return;
    }

    let text = changelog();
    let missing: Vec<&String> = tags
        .iter()
        .filter(|t| {
            let bare = t.trim_start_matches('v');
            !text.contains(&format!("[{bare}]")) && !text.contains(&format!("[{t}]"))
        })
        .collect();

    assert!(
        missing.is_empty(),
        "these tags were released but have no CHANGELOG section: {missing:?}"
    );
}

/// The version being built needs somewhere for its changes to go, whether that
/// is its own released section or `Unreleased`.
#[test]
fn the_current_version_has_a_home() {
    let version = env!("CARGO_PKG_VERSION");
    let text = changelog();

    assert!(
        text.contains(&format!("[{version}]")) || text.contains("## Unreleased"),
        "the workspace is at {version}, but CHANGELOG.md has neither a \
         [{version}] section nor an Unreleased section to collect changes into"
    );
}

/// Newest first. A changelog read bottom-up is a changelog nobody reads.
#[test]
fn releases_are_listed_newest_first() {
    let text = changelog();
    let versions: Vec<&str> = text
        .lines()
        .filter_map(|l| l.strip_prefix("## ["))
        .filter_map(|l| l.split(']').next())
        .collect();

    let parsed: Vec<Vec<u32>> = versions
        .iter()
        .map(|v| {
            v.split(['.', '-'])
                .filter_map(|p| p.parse::<u32>().ok())
                .collect()
        })
        .filter(|p: &Vec<u32>| !p.is_empty())
        .collect();

    for w in parsed.windows(2) {
        assert!(
            w[0] >= w[1],
            "CHANGELOG entries are out of order: {:?} appears above {:?}",
            w[0],
            w[1]
        );
    }
}

/// The file states that older entries are generated from git rather than
/// curated. Removing that note would make derived entries read as if someone
/// had reviewed them.
#[test]
fn the_provenance_of_generated_entries_is_stated() {
    let text = changelog();
    assert!(
        text.contains("derived from the git history"),
        "CHANGELOG.md should say which entries were generated from commit \
         subjects rather than written as release notes, so a reader knows how \
         much curation to expect"
    );
}

// ---------------------------------------------------------------------------
// Packaging: man page, Dockerfile, devcontainer
// ---------------------------------------------------------------------------

/// The man page must be generated from the clap tree, not hand-written.
///
/// HORUS shipped no man page at all — `man horus` found nothing, on a tool
/// whose install script already places a shell completion script. `horus man`
/// renders it from `Cli::command()`, the same tree the binary is built from, so
/// it cannot document a command that does not exist.
#[test]
fn the_man_page_generator_exists() {
    let out = Command::new(env!("CARGO_BIN_EXE_horus"))
        .arg("man")
        .output()
        .expect("horus man must run");

    assert!(out.status.success(), "`horus man` should succeed");

    let page = String::from_utf8_lossy(&out.stdout);
    assert!(
        page.contains(".TH horus 1"),
        "expected a roff man page header, got: {}",
        &page[..page.len().min(200)]
    );
    assert!(page.contains(".SH NAME"), "man page needs a NAME section");
    assert!(
        page.contains(".SH SYNOPSIS"),
        "man page needs a SYNOPSIS section"
    );
}

/// install.sh must actually place it, the way it places completions.
#[test]
fn the_installer_installs_the_man_page() {
    let sh =
        std::fs::read_to_string(repo_root().join("install.sh")).expect("install.sh must exist");
    assert!(
        sh.contains("horus man"),
        "install.sh generates completions but never the man page"
    );
    assert!(
        sh.contains("man1"),
        "the man page needs to land in a man1 directory to be found by `man`"
    );
}

/// The Dockerfile copies each workspace member's manifest to cache dependency
/// compilation. A member added later and not copied breaks the build in a way
/// that only shows up in a Docker build, which no test here can run.
#[test]
fn the_dockerfile_covers_every_workspace_member() {
    let dockerfile =
        std::fs::read_to_string(repo_root().join("Dockerfile")).expect("Dockerfile must exist");
    let cargo =
        std::fs::read_to_string(repo_root().join("Cargo.toml")).expect("Cargo.toml must exist");

    // Take the bracketed members list.
    let start = cargo
        .find("members = [")
        .expect("workspace must declare members");
    let mut depth = 0usize;
    let mut end = start;
    for (i, c) in cargo[start..].char_indices() {
        match c {
            '[' => depth += 1,
            ']' => {
                depth -= 1;
                if depth == 0 {
                    end = start + i;
                    break;
                }
            }
            _ => {}
        }
    }
    let block = &cargo[start..end];
    let members: Vec<&str> = block
        .split('"')
        .skip(1)
        .step_by(2)
        .filter(|m| !m.is_empty() && !m.contains('='))
        .collect();

    assert!(
        members.len() >= 8,
        "failed to parse the workspace members, got {members:?}"
    );

    let missing: Vec<&&str> = members
        .iter()
        .filter(|m| !dockerfile.contains(&format!("COPY {m}/Cargo.toml")))
        .collect();

    assert!(
        missing.is_empty(),
        "the Dockerfile does not copy these workspace members' manifests, so \
         `cargo build --locked` inside the image will fail: {missing:?}"
    );
}

/// A devcontainer that installs a different dependency list from the image is
/// two lists to keep in step. This one builds the image's builder stage.
#[test]
fn the_devcontainer_reuses_the_dockerfile() {
    let json = std::fs::read_to_string(repo_root().join(".devcontainer/devcontainer.json"))
        .expect(".devcontainer/devcontainer.json must exist");
    assert!(
        json.contains("Dockerfile"),
        "the devcontainer should build from the repository Dockerfile rather \
         than duplicating its dependency list"
    );
    assert!(
        json.contains("shm-size"),
        "HORUS nodes talk over shared memory; the default 64 MB /dev/shm is too \
         small for image or point-cloud topics"
    );
}
