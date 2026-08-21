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
        // `[key]` for a table; `key = ` for the one bare key (`enable`).
        //
        // The fallback used to be `contains("`key`")` — the name in backticks
        // anywhere on the page. That is satisfied by any passing mention, and
        // one existed: a language-detection table lists `rust` as a *language*,
        // which made the `[rust]` config section look documented while it was
        // absent entirely. Deleting the whole section still passed this test.
        let documented =
            doc.contains(&format!("[{key}]")) || doc.contains(&format!("{key} = "));
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

/// Every documentation page must be reachable from the navigation.
///
/// 53 of 120 pages were not, including the entire C++ surface: all 25 `cpp/`
/// API pages and all 12 tutorials — every one of which is a C++ tutorial. There
/// was no "C++" section in the sidebar at all, next to the "Rust" and "Python"
/// ones. For a framework that lists C++ as a first-class language, its whole
/// documentation tree existed but could not be navigated to.
///
/// Also asserts the two lists stay identical. `PrevNextNav.tsx` says in a
/// comment that it "must match DocsSidebar.tsx", and nothing checked it.
#[test]
fn every_docs_page_is_reachable_from_the_navigation() {
    let Some(root) = docs_root() else {
        eprintln!("SKIP: horus-docs is not checked out beside horus");
        return;
    };
    let components = root
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("components"));
    let Some(components) = components.filter(|p| p.is_dir()) else {
        eprintln!("SKIP: horus-docs/components not found");
        return;
    };

    let sidebar = std::fs::read_to_string(components.join("DocsSidebar.tsx"))
        .expect("DocsSidebar.tsx must exist");
    let nav = std::fs::read_to_string(components.join("PrevNextNav.tsx"))
        .expect("PrevNextNav.tsx must exist");

    let mut unreachable = Vec::new();
    let mut stack = vec![root.clone()];
    while let Some(dir) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                stack.push(path);
                continue;
            }
            if !path.extension().is_some_and(|e| e == "mdx") {
                continue;
            }
            let rel = path
                .strip_prefix(&root)
                .unwrap_or(&path)
                .to_string_lossy()
                .replace(".mdx", "");

            // A redirect stub is deliberately not in the navigation.
            if let Ok(text) = std::fs::read_to_string(&path) {
                if text.contains("Redirect stub") {
                    continue;
                }
            }

            let href = format!("/{rel}");
            // `foo/index` is reached as `/foo`.
            let parent = rel.strip_suffix("/index").map(|p| format!("/{p}"));

            let linked = sidebar.contains(&format!("\"{href}\""))
                || parent
                    .as_deref()
                    .is_some_and(|p| sidebar.contains(&format!("\"{p}\"")));

            if !linked {
                unreachable.push(rel);
            }
        }
    }

    assert!(
        unreachable.is_empty(),
        "{} documentation page(s) exist but nothing links to them:\n  {}\n\n\
         Add them to `sections` in DocsSidebar.tsx (and `allPages` in \
         PrevNextNav.tsx).",
        unreachable.len(),
        unreachable.join("\n  ")
    );

    // The two lists must agree — PrevNextNav.tsx's own comment requires it.
    let hrefs = |src: &str| -> std::collections::BTreeSet<String> {
        src.split("href: \"")
            .skip(1)
            .filter_map(|s| s.split('"').next())
            .filter(|s| s.starts_with('/'))
            .map(|s| s.to_string())
            .collect()
    };
    let in_sidebar = hrefs(&sidebar);
    let in_nav = hrefs(&nav);

    let only_sidebar: Vec<&String> = in_sidebar.difference(&in_nav).collect();
    let only_nav: Vec<&String> = in_nav.difference(&in_sidebar).collect();

    assert!(
        only_sidebar.is_empty() && only_nav.is_empty(),
        "DocsSidebar.tsx and PrevNextNav.tsx have drifted — the prev/next links \
         will skip or dead-end.\n  only in sidebar: {only_sidebar:?}\n  only in \
         nav: {only_nav:?}"
    );
}

/// A variable the shipped code reads must be documented.
///
/// 26 of the 47 `HORUS_*` variables read by shipped code appeared nowhere in
/// the documentation — including `HORUS_ESTOP_REMOTE`, which controls what a
/// remote emergency stop does locally, and `HORUS_ALLOW_LOCAL_PLUGINS`, which
/// gates executing code from outside the repository. A knob nobody can find is
/// a knob nobody can audit.
#[test]
fn every_environment_variable_is_documented() {
    let Some(root) = docs_root() else {
        eprintln!("SKIP: horus-docs is not checked out beside horus");
        return;
    };

    let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager must have a parent")
        .to_path_buf();

    // Collect HORUS_* names read by shipped code (crate `src/` trees only —
    // a name that appears solely in tests is not a user-facing knob).
    let mut names = std::collections::BTreeSet::new();
    let mut stack: Vec<std::path::PathBuf> = std::fs::read_dir(&repo)
        .expect("repo must be readable")
        .flatten()
        .map(|e| e.path())
        .filter(|p| {
            p.is_dir()
                && p.file_name()
                    .is_some_and(|n| n.to_string_lossy().starts_with("horus"))
        })
        .map(|p| p.join("src"))
        .filter(|p| p.is_dir())
        .collect();

    while let Some(dir) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                stack.push(path);
                continue;
            }
            if !path.extension().is_some_and(|e| e == "rs") {
                continue;
            }
            let Ok(text) = std::fs::read_to_string(&path) else {
                continue;
            };
            // Skip test modules: those knobs are not user-facing.
            for (_, after) in text.match_indices("env::var").map(|(i, _)| (i, &text[i..])) {
                let Some(open) = after.find('"') else {
                    continue;
                };
                let rest = &after[open + 1..];
                let Some(close) = rest.find('"') else {
                    continue;
                };
                let name = &rest[..close];
                if name.starts_with("HORUS_") && name.len() < 60 {
                    names.insert(name.to_string());
                }
            }
        }
    }

    assert!(
        names.len() > 20,
        "failed to collect the environment variables, found {names:?}"
    );

    // Everything in the docs corpus.
    let mut corpus = String::new();
    let mut dstack = vec![root];
    while let Some(dir) = dstack.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                dstack.push(path);
            } else if path.extension().is_some_and(|e| e == "mdx" || e == "md") {
                if let Ok(t) = std::fs::read_to_string(&path) {
                    corpus.push_str(&t);
                }
            }
        }
    }

    let missing: Vec<&String> = names
        .iter()
        .filter(|n| !corpus.contains(n.as_str()))
        .collect();

    assert!(
        missing.is_empty(),
        "{} environment variable(s) are read by shipped code but documented \
         nowhere:\n  {}\n\nAdd them to \
         content/docs/development/environment-variables.mdx — including the \
         internal ones, under the section that says so.",
        missing.len(),
        missing
            .iter()
            .map(|s| s.as_str())
            .collect::<Vec<_>>()
            .join("\n  ")
    );
}

/// Every link the navigation offers must resolve to a page.
///
/// `every_docs_page_is_reachable_from_the_navigation` checks the other
/// direction — that no page is orphaned — and passed the whole time the site
/// footer, which renders on every documentation page, carried five links to
/// pages that do not exist: `/api`, `/architecture`, `/basic-examples`,
/// `/goals`, and `/complete-beginners-guide`. The last one has no correct
/// target at all; there is no beginner's guide, and its presence in the footer
/// is the likely source of the belief that a separate beginner track exists.
///
/// A route resolves as `<slug>.mdx` or `<slug>/index.mdx` — the same two
/// lookups `lib/mdx.tsx` performs.
#[test]
fn every_navigation_link_resolves_to_a_page() {
    let Some(docs) = docs_root() else {
        eprintln!("SKIP: horus-docs not found");
        return;
    };
    let components = docs.join("components");
    let content = docs.join("content/docs");

    let mut dead: Vec<String> = Vec::new();
    let mut checked = 0usize;

    for file in ["DocsSidebar.tsx", "DocsFooter.tsx", "Breadcrumb.tsx"] {
        let Ok(text) = std::fs::read_to_string(components.join(file)) else {
            continue;
        };
        for raw in text.split("href").skip(1) {
            // `href="/x"` and `href: "/x"` both appear.
            let Some(rest) = raw.split_once('"') else {
                continue;
            };
            let Some((href, _)) = rest.1.split_once('"') else {
                continue;
            };
            // Only internal doc routes; external URLs and anchors are not ours.
            if !href.starts_with('/') || href.starts_with("//") || href.len() < 2 {
                continue;
            }
            let slug = href.trim_start_matches('/');
            checked += 1;
            let direct = content.join(format!("{slug}.mdx"));
            let index = content.join(slug).join("index.mdx");
            if !direct.is_file() && !index.is_file() {
                dead.push(format!("{file}: {href}"));
            }
        }
    }

    assert!(
        checked > 50,
        "parsed too few navigation links ({checked}) — the extractor is broken \
         and this test is vacuous"
    );
    dead.sort();
    dead.dedup();
    assert!(
        dead.is_empty(),
        "navigation links that lead nowhere:\n  {}",
        dead.join("\n  ")
    );
}

/// Prev/Next must not be a second, hand-maintained copy of the sidebar order.
///
/// It was, complete with a comment saying it "must match DocsSidebar.tsx". The
/// two held identical page sets and diverged in position by up to 61 places, so
/// a reader in the Rust section was sent past the Rust Guide into Examples and
/// only reached the guide after all of Python.
#[test]
fn prev_next_navigation_is_derived_from_the_sidebar() {
    let Some(docs) = docs_root() else {
        eprintln!("SKIP: horus-docs not found");
        return;
    };
    let text = std::fs::read_to_string(docs.join("components/PrevNextNav.tsx"))
        .expect("PrevNextNav.tsx must exist");

    assert!(
        text.contains("from \"./DocsSidebar\""),
        "PrevNextNav must import the sidebar's order rather than restate it"
    );
    // A hand-copied list is dozens of literal hrefs; a derived one has none.
    let literal_hrefs = text.matches("href: \"/").count();
    assert!(
        literal_hrefs == 0,
        "PrevNextNav still contains {literal_hrefs} hardcoded page links — that \
         is the copy that drifted"
    );
}
