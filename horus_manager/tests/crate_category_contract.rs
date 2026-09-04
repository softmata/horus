//! The published crate metadata must describe the crate that is published.
//!
//! Categories are the one marketing surface that cannot be corrected after the
//! fact: crates.io freezes metadata per version, so a wrong category on a
//! release stays wrong on that release forever. A `cargo yank` does not fix it,
//! and the category pages keep listing it.
//!
//! `horus` and `horus_core` both shipped under `embedded`, which crates.io
//! defines as no-std / bare-metal work. Neither crate has ever had a `no_std`
//! attribute anywhere in the tree; the scheduler's main loop builds a Tokio
//! current-thread runtime as a non-optional dependency, and every topic is
//! backed by a path-named shared-memory file. Someone filtering the category
//! for something to put on a microcontroller would find them, and be wrong.
//!
//! This test is deliberately about the CLAIM, not about becoming embedded. If
//! HORUS ever grows a real `no_std` build, delete the guard in the same commit
//! that adds the attribute.

use std::path::{Path, PathBuf};

/// The name of this source file, excluded from the `no_std` scan below.
const SELF_FILE: &str = "crate_category_contract.rs";

/// Directories neither walker descends into.
///
/// Shared by both so they cannot drift: if the `no_std` scan skipped fewer
/// directories than the manifest scan, a Rust source vendored inside a
/// repo-local virtualenv could make `tree_has_no_std()` true and turn the
/// embedded guard below into a vacuous pass.
macro_rules! skip_dirs {
    () => {
        "target" | ".git" | "node_modules" | "vendor" | ".venv" | "venv"
    };
}

fn repo_root() -> PathBuf {
    // CARGO_MANIFEST_DIR is <root>/horus_manager.
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager must have a parent directory")
        .to_path_buf()
}

/// Every `Cargo.toml` in the workspace, excluding build artefacts and vendored
/// sources — those are not ours to make claims about.
fn workspace_manifests() -> Vec<PathBuf> {
    let root = repo_root();
    let mut found = Vec::new();
    let mut stack = vec![root.clone()];
    while let Some(dir) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            let name = entry.file_name();
            let name = name.to_string_lossy();
            if path.is_dir() {
                if matches!(name.as_ref(), skip_dirs!()) {
                    continue;
                }
                stack.push(path);
            } else if name == "Cargo.toml" {
                found.push(path);
            }
        }
    }
    found.sort();
    found
}

/// The `categories = [...]` array of a manifest, if it declares one.
fn categories_of(manifest: &Path) -> Option<Vec<String>> {
    let text = std::fs::read_to_string(manifest).ok()?;
    // Only the `[package]` categories key matters, and it is a single-line
    // array in every manifest here. Parsed by hand rather than pulling a TOML
    // dependency into a test whose whole job is reading one line.
    for line in text.lines() {
        let line = line.trim();
        let Some(rest) = line.strip_prefix("categories") else {
            continue;
        };
        let Some(rest) = rest.trim_start().strip_prefix('=') else {
            continue;
        };
        let rest = rest.trim();
        let inner = rest.trim_start_matches('[').trim_end_matches(']');
        return Some(
            inner
                .split(',')
                .map(|s| s.trim().trim_matches('"').to_string())
                .filter(|s| !s.is_empty())
                .collect(),
        );
    }
    None
}

/// Whether this source declares `no_std` as a crate ATTRIBUTE.
///
/// The attribute, not the word. `horus_manager/src/error_wrapper.rs` carries
/// the literal `"... but `serde` was built with feature `no_std`"` in an error
/// message, and an earlier version of this check treated that as a no_std
/// build -- which made the guard below return early and pass while `horus`
/// still claimed the embedded category. Caught by ablation, which is the only
/// reason this comment exists.
fn declares_no_std(text: &str) -> bool {
    // `#![no_std]`, or `#![cfg_attr(<anything>, no_std)]` on one attribute.
    // Each inner attribute ends at its first `]`, so the search is bounded by
    // the attribute itself and cannot be fooled by a string literal elsewhere
    // in the file. Cutting at `]` rather than at a fixed byte count also keeps
    // the slice on a character boundary -- a fixed window panicked here on a
    // file whose 160th byte landed inside a multi-byte arrow.
    let mut from = 0usize;
    while let Some(rel) = text[from..].find("#![") {
        let start = from + rel;
        let rest = &text[start..];
        let attr = match rest.find(']') {
            Some(end) => &rest[..end],
            None => rest,
        };
        if attr.contains("no_std") {
            return true;
        }
        from = start + 3;
    }
    false
}

/// Whether any Rust source in the workspace actually declares `no_std`.
fn tree_has_no_std() -> bool {
    let root = repo_root();
    let mut stack = vec![root];
    while let Some(dir) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            let name = entry.file_name();
            let name = name.to_string_lossy();
            if path.is_dir() {
                if matches!(name.as_ref(), skip_dirs!()) {
                    continue;
                }
                stack.push(path);
            } else if name.ends_with(".rs") {
                // This file is not evidence about itself. It contains the
                // literal `#![no_std]` it searches for, so scanning it made
                // `declares_no_std` return true, made the guard below return
                // early, and made it pass while `horus` still claimed the
                // embedded category. The ablation caught it; nothing else
                // would have.
                if name == SELF_FILE {
                    continue;
                }
                if let Ok(text) = std::fs::read_to_string(&path) {
                    if declares_no_std(&text) {
                        return true;
                    }
                }
            }
        }
    }
    false
}

#[test]
fn no_crate_claims_the_embedded_category_without_a_no_std_build() {
    let has_no_std = tree_has_no_std();
    let offenders: Vec<String> = workspace_manifests()
        .into_iter()
        .filter_map(|m| {
            let cats = categories_of(&m)?;
            cats.iter()
                .any(|c| c == "embedded" || c == "no-std" || c.starts_with("no-std::"))
                .then(|| m.display().to_string())
        })
        .collect();

    if has_no_std {
        eprintln!(
            "a no_std build exists in the tree — the embedded category is now defensible, \
             and this guard should be deleted rather than left passing vacuously"
        );
        return;
    }

    assert!(
        offenders.is_empty(),
        "these crates claim an embedded / no-std category while nothing in the workspace \
         declares #![no_std], the scheduler's main loop builds a Tokio runtime as a \
         non-optional dependency, and every topic is backed by a shared-memory file:\n  {}\n\
         crates.io freezes categories per published version, so this cannot be corrected \
         after release. Remove the category, or add the no_std build that would justify it.",
        offenders.join("\n  ")
    );
}

/// Guards the guard: the manifest reader must actually find categories, or the
/// test above would pass by reading nothing.
#[test]
fn the_category_reader_finds_the_categories_it_is_meant_to_check() {
    let manifests = workspace_manifests();
    assert!(
        manifests.len() >= 5,
        "expected the workspace's several crates, found {} manifests",
        manifests.len()
    );

    let with_categories: Vec<_> = manifests
        .iter()
        .filter_map(|m| categories_of(m).map(|c| (m, c)))
        .collect();
    assert!(
        with_categories.len() >= 3,
        "at least the published crates declare categories; found {} that do — if this \
         dropped to zero the embedded guard above would pass by checking nothing",
        with_categories.len()
    );

    // And the values must parse as real category strings, not as fragments of
    // the surrounding TOML.
    for (m, cats) in &with_categories {
        for c in cats {
            assert!(
                !c.contains('[') && !c.contains(']') && !c.contains('"'),
                "{} parsed a malformed category {:?}",
                m.display(),
                c
            );
            assert!(
                c.chars()
                    .all(|ch| ch.is_ascii_lowercase() || ch == '-' || ch == ':'),
                "{} parsed {:?}, which is not a crates.io category slug",
                m.display(),
                c
            );
        }
    }
}
