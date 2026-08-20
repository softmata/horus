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

// ---------------------------------------------------------------------------
// Translated READMEs
// ---------------------------------------------------------------------------

/// The five translated READMEs are abridged overviews, not full translations —
/// 118 lines against the English README's 383. That is a deliberate choice and
/// each one says so, but it only stays honest while the disclaimer is there.
///
/// The audit predicted they would contradict the English README after its
/// performance claims were corrected. They do not: all six carry the same
/// `3-304 ns` figure and the same install one-liner. What is worth guarding is
/// exactly that — the two things a reader acts on, and the notice that tells
/// them where the authoritative version is.
const TRANSLATIONS: &[&str] = &[
    "README.de.md",
    "README.es.md",
    "README.ja.md",
    "README.pt-BR.md",
    "README.zh-CN.md",
];

#[test]
fn every_translation_points_at_the_english_readme() {
    let mut missing = Vec::new();
    for name in TRANSLATIONS {
        let Some(text) = read(name) else {
            missing.push(format!("{name} does not exist"));
            continue;
        };
        // Each disclaimer is in its own language; the invariant is the link.
        if !text.contains("README.md") {
            missing.push(format!(
                "{name} has no link back to the English README, so a reader has \
                 no way to know it is an abridged overview"
            ));
        }
    }
    assert!(missing.is_empty(), "{}", missing.join("\n  "));
}

/// The install command is the one instruction a reader copies verbatim. It must
/// be identical everywhere, or some fraction of users runs a different one.
#[test]
fn every_translation_installs_the_same_way() {
    let english = read("README.md").expect("README.md must exist");
    let expected = advertised_branches(&english);
    assert!(
        !expected.is_empty(),
        "could not find the install URL in README.md"
    );

    let mut wrong = Vec::new();
    for name in TRANSLATIONS {
        let Some(text) = read(name) else { continue };
        let branches = advertised_branches(&text);
        if branches.is_empty() {
            wrong.push(format!("{name} advertises no install command"));
        } else if branches != expected {
            wrong.push(format!(
                "{name} installs from {branches:?} while README.md uses {expected:?}"
            ));
        }
    }
    assert!(wrong.is_empty(), "{}", wrong.join("\n  "));
}

/// A performance figure quoted in a translation must match the English one.
/// Numbers survive translation unchanged, so a mismatch is drift, not localisation.
#[test]
fn quoted_latency_figures_agree_across_translations() {
    let english = read("README.md").expect("README.md must exist");
    let has_figure = english.contains("304 ns");
    assert!(
        has_figure,
        "README.md no longer quotes the 3-304 ns range this test tracks; update \
         it here too so the translations stay checked"
    );

    let mut wrong = Vec::new();
    for name in TRANSLATIONS {
        let Some(text) = read(name) else { continue };
        // A translation may omit the figure entirely — that is abridgement, and
        // it is fine. What it must not do is quote a *different* number.
        if text.contains(" ns") && !text.contains("304 ns") {
            wrong.push(format!(
                "{name} quotes a latency figure that is not the English one"
            ));
        }
    }
    assert!(wrong.is_empty(), "{}", wrong.join("\n  "));
}
