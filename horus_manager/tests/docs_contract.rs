//! The Configuration Reference must actually reference the configuration.
//!
//! `horus.toml` accepts 15 top-level tables. The page titled "Configuration
//! Reference" documented four of them:
//!
//! ```text
//! documented: package, dependencies, ignore  (+ enable buried in prose)
//! missing:    workspace, robot, dev-dependencies, sim-dependencies,
//!             hardware, drivers, sim-drivers, scripts, cpp, hooks, network
//! ```
//!
//! `[hardware]` is how a robot declares its drivers and `[network]` is how two
//! robots talk — neither appeared. A user's only way to discover them was to
//! read `manifest.rs`, and a user who guessed a key wrong got no error either,
//! because the manifest silently ignored unknown keys (see `manifest_lint`).
//!
//! ## Scope
//!
//! The docs are a separate repository (`horus-docs`), so these tests skip when
//! it is not checked out beside this one. That means they protect a developer
//! working on both, and CI for the docs repo, but they cannot protect a
//! horus-only CI run. The skip is reported, not silent.
//!
//! Run: `cargo test -p horus_manager --test docs_contract`

use horus_manager::manifest_lint::KNOWN_TOP_LEVEL;
use std::path::{Path, PathBuf};

/// `horus-docs` checked out beside `horus`, if it is there.
fn docs_root() -> Option<PathBuf> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()? // horus/
        .parent()? // softmata/
        .join("horus-docs");
    root.is_dir().then_some(root)
}

fn config_reference() -> Option<String> {
    let path = docs_root()?.join("content/docs/package-management/configuration.mdx");
    std::fs::read_to_string(path).ok()
}

/// Every table the manifest accepts must appear in the reference.
#[test]
fn the_configuration_reference_covers_every_manifest_table() {
    let Some(doc) = config_reference() else {
        eprintln!("SKIP: horus-docs is not checked out beside horus");
        return;
    };

    let mut undocumented = Vec::new();
    for key in KNOWN_TOP_LEVEL {
        // `enable` is a bare key, not a table, so it is written `enable = [...]`
        // or as a heading. Everything else appears as `[key]`.
        let documented = doc.contains(&format!("[{key}]")) || doc.contains(&format!("`{key}`"));
        if !documented {
            undocumented.push(*key);
        }
    }

    assert!(
        undocumented.is_empty(),
        "horus.toml accepts these keys but the Configuration Reference never \
         mentions them: {undocumented:?}\n\n\
         An undocumented key is one a user can only find by reading the source. \
         Add a section to content/docs/package-management/configuration.mdx, or \
         remove the key from the manifest."
    );
}

/// The reference should show each table in use, not merely name it in a list.
/// A key that only appears inside the Quick Reference block is not documented.
#[test]
fn each_table_gets_a_section_not_just_a_mention() {
    let Some(doc) = config_reference() else {
        eprintln!("SKIP: horus-docs is not checked out beside horus");
        return;
    };

    // Headings look like `## Hooks` / `### \`[hardware]\` (Optional)`.
    let headings: Vec<&str> = doc
        .lines()
        .filter(|l| l.trim_start().starts_with('#'))
        .collect();
    let heading_text = headings.join("\n").to_lowercase();

    // A handful of keys are documented under a shared heading, which is fine —
    // `[drivers]`/`[sim-drivers]` belong with `[hardware]`, and the three
    // dependency tables belong together. Those are listed here explicitly so
    // that adding a *new* key cannot quietly inherit the exemption.
    let grouped: &[(&str, &str)] = &[
        ("drivers", "hardware"),
        ("sim-drivers", "hardware"),
        ("dev-dependencies", "dependencies"),
        ("sim-dependencies", "dependencies"),
    ];

    let mut sectionless = Vec::new();
    for key in KNOWN_TOP_LEVEL {
        let own = heading_text.contains(&key.to_lowercase());
        let under_group = grouped
            .iter()
            .any(|(k, parent)| k == key && heading_text.contains(parent));
        if !own && !under_group {
            sectionless.push(*key);
        }
    }

    assert!(
        sectionless.is_empty(),
        "these keys are mentioned but have no section of their own: \
         {sectionless:?}\n\n\
         Being listed in an example is not documentation — a reader needs to \
         know what the fields mean."
    );
}

/// The lifecycle hooks are the most recently added surface and the easiest to
/// ship undocumented, so they get their own check.
#[test]
fn every_hook_phase_is_documented() {
    let Some(doc) = config_reference() else {
        eprintln!("SKIP: horus-docs is not checked out beside horus");
        return;
    };

    let mut missing = Vec::new();
    for hook in horus_manager::manifest_lint::KNOWN_HOOKS {
        if !doc.contains(hook) {
            missing.push(*hook);
        }
    }

    assert!(
        missing.is_empty(),
        "these hooks exist but are undocumented: {missing:?}"
    );
}

/// A guard on the guard: if the reference file moves or is renamed, the tests
/// above would skip forever and report nothing. This fails instead.
#[test]
fn the_configuration_reference_is_where_we_think_it_is() {
    let Some(root) = docs_root() else {
        eprintln!("SKIP: horus-docs is not checked out beside horus");
        return;
    };

    let path = root.join("content/docs/package-management/configuration.mdx");
    assert!(
        path.is_file(),
        "the docs repo is present but {} is missing. If the reference moved, \
         update docs_contract.rs — otherwise the coverage tests above silently \
         stop checking anything.",
        path.display()
    );
}

/// A command HORUS tells the user to run must be documented.
///
/// `horus doctor` ended its real-time check with "run `horus setup-rt` for
/// ±20μs". `setup-rt` appeared **zero** times in the entire docs repository, so
/// the one place a user is told about it is a warning line, with nothing to
/// read about what it will do to their machine — and it installs a kernel
/// package and edits `/etc/security/limits.conf`.
#[test]
fn commands_the_cli_suggests_are_documented() {
    let Some(root) = docs_root() else {
        eprintln!("SKIP: horus-docs is not checked out beside horus");
        return;
    };

    // Commands the CLI emits to the user as advice. Add to this list when a new
    // suggestion is introduced; the test then requires it to be documented.
    let suggested = ["setup-rt", "doctor", "check", "lint", "fmt", "doc"];

    let mut corpus = String::new();
    let mut stack = vec![root.clone()];
    while let Some(dir) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                stack.push(path);
            } else if path.extension().is_some_and(|e| e == "mdx" || e == "md") {
                if let Ok(text) = std::fs::read_to_string(&path) {
                    corpus.push_str(&text);
                }
            }
        }
    }

    let missing: Vec<&str> = suggested
        .iter()
        .copied()
        .filter(|c| !corpus.contains(&format!("horus {c}")))
        .collect();

    assert!(
        missing.is_empty(),
        "the CLI tells users to run these, but the documentation never mentions \
         them: {missing:?}\n\nA command that modifies the user's system needs a \
         page they can read first."
    );
}
