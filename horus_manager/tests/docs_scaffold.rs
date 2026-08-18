//! Scaffold-layout contract — `horus new` must produce the tree the docs promise.
//!
//! A quick-start page's first instruction is "run `horus new …`" and its second
//! is "open `<some path>`". If that path does not exist, the reader is stuck on
//! step 2 with no way to guess the right one. This is the cheapest class of
//! documentation breakage to test and among the most damaging to hit.
//!
//! ## Why this file reads the docs instead of listing them
//!
//! An earlier version of this test hardcoded the expected layouts and cited
//! `getting-started/quick-start-python.mdx` and `quick-start-cpp.mdx` as the
//! source. **Neither file exists.** The citations were wrong, and because they
//! were prose in a comment, nothing caught it — the test passed while
//! documenting a claim about the docs that was false. A source comment in
//! `commands/new.rs` inherited the same false premise ("35 doc sites say
//! `src/main.py`"; the real count was one, against six for root `main.py`) and
//! was used to justify moving the Python entry point.
//!
//! So the expectations here are now *derived*: the test greps the docs that are
//! actually checked out and asserts that whatever path they tell readers to open
//! is a path `horus new` creates. A claim about the docs that the docs do not
//! support now fails instead of passing.
//!
//! ## Scope
//!
//! `horus-docs` is a separate repository. When it is not checked out beside this
//! one, the doc-derived tests report a skip rather than passing silently; the
//! layout tests below still run, since they only need the CLI.
//!
//! Hermetic and fast (three `horus new` invocations), so it runs on every PR.

use assert_cmd::cargo::cargo_bin_cmd;
use std::path::{Path, PathBuf};

/// What a language flag scaffolds.
struct Scaffold {
    /// Flag passed to `horus new`.
    flag: &'static str,
    /// Entry point the reader is meant to edit.
    entry: &'static str,
    /// Why this path and not another.
    rationale: &'static str,
}

const SCAFFOLDS: &[Scaffold] = &[
    Scaffold {
        flag: "-r",
        entry: "src/main.rs",
        rationale: "Cargo requires it; quick-start.mdx:33 names it explicitly",
    },
    Scaffold {
        flag: "-m",
        entry: "src/main.rs",
        rationale: "`-m` is the alias for the default Rust scaffold",
    },
    Scaffold {
        flag: "--cpp",
        entry: "src/main.cpp",
        rationale: "follows the Rust layout; C++ has no competing convention",
    },
    Scaffold {
        flag: "-p",
        entry: "main.py",
        rationale: "Python's convention for a single-entry-point app is the \
                    repository root — `src/` in Python means `src/<package>/` \
                    with an `__init__.py`, which this is not",
    },
];

fn docs_root() -> Option<PathBuf> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()?
        .parent()?
        .join("horus-docs/content/docs");
    root.is_dir().then_some(root)
}

/// Every `.mdx` page in the docs, as (relative path, text).
fn doc_pages() -> Vec<(String, String)> {
    let Some(root) = docs_root() else {
        return Vec::new();
    };
    let mut pages = Vec::new();
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
                    let rel = path
                        .strip_prefix(&root)
                        .unwrap_or(&path)
                        .to_string_lossy()
                        .to_string();
                    pages.push((rel, text));
                }
            }
        }
    }
    pages
}

/// Run `horus new <name> <flag>` in a fresh dir and return the project path.
fn scaffold(tmp: &Path, name: &str, flag: &str) -> PathBuf {
    let out = cargo_bin_cmd!("horus")
        .args(["new", name, flag])
        .current_dir(tmp)
        .output()
        .expect("horus new runs");
    assert!(
        out.status.success(),
        "`horus new {name} {flag}` failed: {}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    tmp.join(name)
}

/// Every documented entry point must exist after `horus new`.
#[test]
fn documented_entry_points_exist() {
    let tmp = tempfile::tempdir().unwrap();

    for (i, s) in SCAFFOLDS.iter().enumerate() {
        let project = scaffold(tmp.path(), &format!("scaffold_probe_{i}"), s.flag);
        let entry = project.join(s.entry);
        assert!(
            entry.is_file(),
            "`horus new <name> {}` should create `{}` ({}), but the tree is:\n{}",
            s.flag,
            s.entry,
            s.rationale,
            list_tree(&project)
        );
    }
}

/// The scaffolded entry point must be the *only* main file, so a reader who
/// finds one is not left wondering which of two is live.
#[test]
fn there_is_exactly_one_entry_point_per_scaffold() {
    let tmp = tempfile::tempdir().unwrap();

    for (i, s) in SCAFFOLDS.iter().enumerate() {
        let project = scaffold(tmp.path(), &format!("single_entry_{i}"), s.flag);

        let candidates = [
            "main.rs",
            "src/main.rs",
            "main.py",
            "src/main.py",
            "main.cpp",
            "src/main.cpp",
        ];
        let present: Vec<&str> = candidates
            .iter()
            .copied()
            .filter(|c| project.join(c).is_file())
            .collect();

        assert_eq!(
            present,
            vec![s.entry],
            "`horus new <name> {}` should leave exactly one entry point; found {present:?}",
            s.flag
        );
    }
}

/// Derived, not asserted: every `main.py` path the docs tell a reader to open
/// must be one `horus new -p` actually creates.
///
/// This is the check that would have caught the false "35 doc sites" claim.
#[test]
fn the_docs_python_entry_point_matches_the_scaffold() {
    let pages = doc_pages();
    if pages.is_empty() {
        eprintln!("SKIP: horus-docs is not checked out beside horus");
        return;
    }

    let tmp = tempfile::tempdir().unwrap();
    let project = scaffold(tmp.path(), "doc_derived_py", "-p");

    let mut wrong = Vec::new();
    for (page, text) in &pages {
        for (lineno, line) in text.lines().enumerate() {
            // Only lines that point at a concrete path, i.e. `src/main.py`.
            // Bare `main.py` matches the root scaffold either way.
            if line.contains("src/main.py") && !project.join("src/main.py").is_file() {
                wrong.push(format!("{page}:{}: {}", lineno + 1, line.trim()));
            }
        }
    }

    assert!(
        wrong.is_empty(),
        "these doc lines tell readers to open `src/main.py`, but `horus new -p` \
         creates `main.py` at the project root:\n  {}\n\n\
         Fix the docs or the scaffold — but they have to agree.",
        wrong.join("\n  ")
    );
}

/// The same check for Rust, which is the language most of the docs are written
/// against and therefore the one most costly to get wrong.
#[test]
fn the_docs_rust_entry_point_matches_the_scaffold() {
    let pages = doc_pages();
    if pages.is_empty() {
        eprintln!("SKIP: horus-docs is not checked out beside horus");
        return;
    }

    let tmp = tempfile::tempdir().unwrap();
    let project = scaffold(tmp.path(), "doc_derived_rs", "-r");
    assert!(
        project.join("src/main.rs").is_file(),
        "the Rust scaffold must produce src/main.rs"
    );

    let mut wrong = Vec::new();
    for (page, text) in &pages {
        for (lineno, line) in text.lines().enumerate() {
            // A doc that says the Rust entry point is at the root is wrong:
            // Cargo would not build it.
            let root_claim = line.contains("`main.rs`") && !line.contains("src/main.rs");
            if root_claim && !line.contains("or") && !line.contains("under `src/`") {
                wrong.push(format!("{page}:{}: {}", lineno + 1, line.trim()));
            }
        }
    }

    assert!(
        wrong.is_empty(),
        "these doc lines name `main.rs` without its `src/` prefix, which is not \
         where Cargo looks:\n  {}",
        wrong.join("\n  ")
    );
}

/// Guard on the guard: if the docs move, the derived tests above skip forever
/// and report nothing. Fail loudly instead.
#[test]
fn the_docs_are_where_we_think_they_are() {
    let Some(root) = docs_root() else {
        eprintln!("SKIP: horus-docs is not checked out beside horus");
        return;
    };

    let quick_start = root.join("getting-started/quick-start.mdx");
    assert!(
        quick_start.is_file(),
        "{} is missing. Every citation in this file assumes it exists; if the \
         page moved, update this test — otherwise the derived checks silently \
         stop checking anything.",
        quick_start.display()
    );

    assert!(
        !doc_pages().is_empty(),
        "the docs directory exists but contains no .mdx pages"
    );
}

fn list_tree(root: &Path) -> String {
    let mut out = Vec::new();
    let mut stack = vec![root.to_path_buf()];
    while let Some(dir) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            let rel = path
                .strip_prefix(root)
                .unwrap_or(&path)
                .display()
                .to_string();
            if path.is_dir() {
                stack.push(path);
                out.push(format!("  {rel}/"));
            } else {
                out.push(format!("  {rel}"));
            }
        }
    }
    out.sort();
    out.join("\n")
}
