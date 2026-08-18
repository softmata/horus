//! The advertised install command must be the one that works.
//!
//! Every README advertised:
//!
//! ```text
//! curl -fsSL https://github.com/softmata/horus/raw/release/install.sh | bash
//! ```
//!
//! There is no `release` branch on origin — tags are cut from `main` — so that
//! URL returned **404**. The single most important command in the project, the
//! one on the front page of six READMEs, did not work.
//!
//! `install.sh` meanwhile defaults to `BRANCH="${HORUS_INSTALL_BRANCH:-main}"`
//! and documents `raw/main` in its own header, so the two halves of the same
//! instruction disagreed about which branch to fetch.
//!
//! These tests compare the files rather than hitting the network, so they work
//! offline and in CI.
//!
//! Run: `cargo test -p horus_manager --test install_contract`

use std::path::{Path, PathBuf};

fn repo_root() -> PathBuf {
    // CARGO_MANIFEST_DIR is horus_manager/; the repo root is its parent.
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager must have a parent directory")
        .to_path_buf()
}

fn read(path: &str) -> Option<String> {
    std::fs::read_to_string(repo_root().join(path)).ok()
}

/// Branch `install.sh` clones by default.
fn installer_branch() -> String {
    let sh = read("install.sh").expect("install.sh must exist");
    let line = sh
        .lines()
        .find(|l| l.trim_start().starts_with("BRANCH="))
        .expect("install.sh must define BRANCH");

    // BRANCH="${HORUS_INSTALL_BRANCH:-main}"
    line.split(":-")
        .nth(1)
        .and_then(|rest| rest.split('}').next())
        .map(|s| s.trim().to_string())
        .unwrap_or_else(|| panic!("could not parse the default from: {line}"))
}

/// Every README that advertises the curl-pipe install.
fn readmes_with_install_url() -> Vec<(String, String)> {
    let mut out = Vec::new();
    for name in [
        "README.md",
        "README.de.md",
        "README.es.md",
        "README.ja.md",
        "README.pt-BR.md",
        "README.zh-CN.md",
    ] {
        if let Some(text) = read(name) {
            if text.contains("install.sh") {
                out.push((name.to_string(), text));
            }
        }
    }
    assert!(!out.is_empty(), "no README advertises install.sh");
    out
}

/// Extract the branch from a `raw/<branch>/install.sh` URL.
fn advertised_branches(text: &str) -> Vec<String> {
    let mut branches = Vec::new();
    for (idx, _) in text.match_indices("/raw/") {
        let rest = &text[idx + "/raw/".len()..];
        if let Some(slash) = rest.find('/') {
            if rest[slash..].starts_with("/install.sh") {
                branches.push(rest[..slash].to_string());
            }
        }
    }
    branches
}

#[test]
fn every_readme_points_at_the_branch_the_installer_uses() {
    let expected = installer_branch();

    for (name, text) in readmes_with_install_url() {
        for branch in advertised_branches(&text) {
            assert_eq!(
                branch, expected,
                "{name} advertises `raw/{branch}/install.sh` but install.sh \
                 clones `{expected}`. For the one command that runs arbitrary \
                 shell from the internet, the two must agree — `raw/release` \
                 returned 404 for exactly this reason."
            );
        }
    }
}

/// The branch has to be one that exists. `release` never did.
#[test]
fn the_installer_branch_is_a_real_branch() {
    let branch = installer_branch();
    assert!(
        matches!(branch.as_str(), "main" | "master"),
        "install.sh clones `{branch}`, which is not a branch this repository \
         has. Tags are cut from main; there is no long-lived release branch."
    );
}

/// install.sh's own header shows the URL users copy, so it must match too.
#[test]
fn the_installer_header_matches_its_own_default() {
    let sh = read("install.sh").expect("install.sh must exist");
    let expected = installer_branch();

    for branch in advertised_branches(&sh) {
        assert_eq!(
            branch, expected,
            "install.sh's header advertises `raw/{branch}/install.sh` while it \
             clones `{expected}`"
        );
    }
}

/// A missing README should not silently pass the check above.
#[test]
fn the_primary_readme_advertises_the_install_command() {
    let text = read("README.md").expect("README.md must exist");
    assert!(
        text.contains("install.sh"),
        "README.md should tell people how to install"
    );
    assert!(
        !advertised_branches(&text).is_empty(),
        "README.md's install URL should be of the form raw/<branch>/install.sh, \
         so this contract can check it"
    );
}
