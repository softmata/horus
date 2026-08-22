//! The five translated READMEs must not say something the English one does not.
//!
//! TRUST-3 asked for either refreshed translations or guards that stop them
//! contradicting the source. The refresh never happened — `wc -l` is still 409
//! for README.md against 118 for each translation — and the remediation's only
//! deliverable was three tests in `install_contract.rs`
//! (`every_translation_points_at_the_english_readme`,
//! `every_translation_installs_the_same_way`,
//! `quoted_latency_figures_agree_across_translations`), all three of which were
//! deleted when that file was rewritten for an unrelated finding. `grep -rn
//! 'zh-CN' --include=*.rs` over the repo returned nothing.
//!
//! An abridgement is a legitimate choice: every translation links back to
//! README.md as the authoritative version. What is not legitimate is an
//! abridgement that keeps a number, a claim or an install command the English
//! README has since corrected — which is how "575x faster than ROS2" would have
//! outlived its own removal. These are those three tests, restored, plus a
//! fourth for the claim TRUST-1 removed.

use std::fs;
use std::path::PathBuf;

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager has a parent")
        .to_path_buf()
}

const TRANSLATIONS: [&str; 5] = [
    "README.zh-CN.md",
    "README.pt-BR.md",
    "README.ja.md",
    "README.es.md",
    "README.de.md",
];

fn read(name: &str) -> String {
    fs::read_to_string(repo_root().join(name)).unwrap_or_else(|_| panic!("{name} must exist"))
}

/// A translation that is shorter than its source has to say where the source
/// is, or a reader has no way to know what was left out.
#[test]
fn every_translation_points_at_the_english_readme() {
    for name in TRANSLATIONS {
        let body = read(name);
        assert!(
            body.contains("README.md"),
            "{name} never links back to README.md, and it is an abridgement of it"
        );
    }
}

/// One install command, six files. This is the test that would have caught
/// TRUST-2 in the translations.
#[test]
fn every_translation_installs_the_same_way() {
    let english = read("README.md");
    let install_line = english
        .lines()
        .find(|l| l.contains("curl -fsSL") && l.contains("install.sh"))
        .expect("README.md must show the install one-liner")
        .trim()
        .to_string();

    for name in TRANSLATIONS {
        let body = read(name);
        assert!(
            body.contains(&install_line),
            "{name} does not show the same install command as README.md:\n  \
             expected: {install_line}"
        );
    }
}

/// Pull every `<n>–<m> ns` / `<n>-<m> ns` style latency range out of a README.
fn latency_figures(body: &str) -> Vec<String> {
    let mut out = Vec::new();
    for token in body.split_whitespace() {
        let cleaned: String = token
            .chars()
            .filter(|c| c.is_ascii_digit() || *c == '-' || *c == '\u{2013}')
            .collect();
        if cleaned.contains('-') || cleaned.contains('\u{2013}') {
            let parts: Vec<&str> = cleaned
                .split(|c| c == '-' || c == '\u{2013}')
                .filter(|s| !s.is_empty())
                .collect();
            if parts.len() == 2 && parts.iter().all(|p| p.chars().all(|c| c.is_ascii_digit())) {
                out.push(format!("{}-{}", parts[0], parts[1]));
            }
        }
    }
    out.sort();
    out.dedup();
    out
}

/// The headline latency range is the number the whole project is judged on. If
/// the English README revises it, a translation still quoting the old one is a
/// published contradiction.
#[test]
fn quoted_latency_figures_agree_across_translations() {
    let english = read("README.md");
    let english_figures = latency_figures(&english);
    assert!(
        !english_figures.is_empty(),
        "README.md quotes no latency range — the extractor is broken and this test is vacuous"
    );

    for name in TRANSLATIONS {
        let body = read(name);
        for figure in latency_figures(&body) {
            assert!(
                english_figures.contains(&figure),
                "{name} quotes the range {figure} ns, which README.md does not: \
                 README.md has {english_figures:?}"
            );
        }
    }
}

/// TRUST-1 removed "575x faster than ROS2" and the 550x/585x/875x table because
/// none of it was measured. A translation is a place that claim can survive its
/// own retraction, so no translation may carry a comparison ratio the English
/// README does not.
#[test]
fn no_translation_keeps_a_ros2_ratio_the_english_readme_dropped() {
    let english = read("README.md");
    for name in TRANSLATIONS {
        let body = read(name);
        for claim in ["575x", "550x", "585x", "875x", "550-875x"] {
            if english.contains(claim) {
                continue;
            }
            assert!(
                !body.contains(claim),
                "{name} still claims {claim} faster than ROS2; README.md dropped it \
                 because the number appears in no benchmark, report or citation"
            );
        }
    }
}

/// The language picker at the top of each file has to reach every other
/// translation, or the set silently splits.
#[test]
fn every_readme_links_every_other_translation() {
    let all: Vec<&str> = std::iter::once("README.md").chain(TRANSLATIONS).collect();
    for name in &all {
        let body = read(name);
        for other in &all {
            if other == name {
                continue;
            }
            assert!(
                body.contains(other),
                "{name}'s language picker does not link {other}"
            );
        }
    }
}
