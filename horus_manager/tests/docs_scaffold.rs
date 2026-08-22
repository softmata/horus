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

/// The horus repository root (the parent of `horus_manager`).
fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager has a parent")
        .to_path_buf()
}

fn docs_root() -> Option<PathBuf> {
    // `HORUS_DOCS_DIR` first: the docs-contract workflow checks horus-docs out
    // *inside* the horus workspace and points every other doc test at it with
    // this variable. Only looking for a sibling directory meant this file could
    // never see a CI docs checkout, so its doc-derived tests would report SKIP
    // in the one job that has the docs — the same silent-pass this file's own
    // header warns about.
    if let Ok(dir) = std::env::var("HORUS_DOCS_DIR") {
        let root = Path::new(&dir).join("content/docs");
        if root.is_dir() {
            return Some(root);
        }
    }
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()?
        .parent()?
        .join("horus-docs/content/docs");
    root.is_dir().then_some(root)
}

/// One `getting-started` page, as (relative path, text).
fn getting_started_pages() -> Vec<(String, String)> {
    doc_pages()
        .into_iter()
        .filter(|(rel, _)| rel.replace('\\', "/").starts_with("getting-started/"))
        .collect()
}

/// Everything inside ``` fences, concatenated. Prose is what is left over.
fn split_fences(text: &str) -> (String, String) {
    let (mut code, mut prose) = (String::new(), String::new());
    let mut in_fence = false;
    for line in text.lines() {
        if line.trim_start().starts_with("```") {
            in_fence = !in_fence;
            continue;
        }
        let sink = if in_fence { &mut code } else { &mut prose };
        sink.push_str(line);
        sink.push('\n');
    }
    (code, prose)
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

// ════════════════════════════════════════════════════════════════════════════
// README ↔ scaffold contracts
//
// `readme_contract.rs` covers the same ground for Rust, but it is in no CI
// workflow: `ci.yml` runs `--test`-less `--lib`, and `docs-contract.yml`
// enumerates its targets by name and does not list it. These live here because
// this target *is* enumerated (docs-contract.yml, "Scaffold layout contract"),
// runs on every PR, and needs no docs checkout — the README and `horus new`
// both live in this repository.
// ════════════════════════════════════════════════════════════════════════════

fn readme() -> String {
    std::fs::read_to_string(repo_root().join("README.md")).expect("README.md must exist")
}

/// Every ```rust fence in the README, in order.
///
/// `readme_contract.rs` reads only the first, so its style checks stop at the
/// Quick Start and the two later fences drift unobserved.
fn readme_rust_fences() -> Vec<String> {
    let text = readme();
    let mut fences = Vec::new();
    let mut current: Option<String> = None;
    for line in text.lines() {
        match &mut current {
            None => {
                if line.trim_start() == "```rust" {
                    current = Some(String::new());
                }
            }
            Some(buf) => {
                if line.trim_start().starts_with("```") {
                    fences.push(std::mem::take(buf));
                    current = None;
                } else {
                    buf.push_str(line);
                    buf.push('\n');
                }
            }
        }
    }
    fences
}

/// Read one file out of a fresh `horus new <name> <flag>` project.
fn scaffolded_file(flag: &str, entry: &str) -> String {
    let tmp = tempfile::tempdir().unwrap();
    let project = scaffold(tmp.path(), "starter", flag);
    std::fs::read_to_string(project.join(entry))
        .unwrap_or_else(|e| panic!("`horus new starter {flag}` should write {entry}: {e}"))
}

// ─── ONB-2: the README states one Rust floor, and it is the real one ─────────

/// The workspace MSRV, read from the manifest that declares it.
fn workspace_msrv() -> String {
    let manifest =
        std::fs::read_to_string(repo_root().join("Cargo.toml")).expect("workspace Cargo.toml");
    for line in manifest.lines() {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix("rust-version") {
            if let Some(v) = rest.split('"').nth(1) {
                return v.to_string();
            }
        }
    }
    panic!("workspace Cargo.toml has no `rust-version`");
}

/// Every Rust version a README claims, wherever it claims it.
///
/// The previous guard was `readme.contains("Rust 1.90")`, which passes while a
/// second, different floor sits three lines above it — and one did: the badge
/// said `>=1.92` while the prose said 1.90. Containment cannot see a
/// contradiction; enumeration can.
fn rust_versions_claimed(text: &str) -> Vec<(usize, String)> {
    let re = regex::Regex::new(r"(?i)rust[-_ ]*(?:%3E%3D|%3E|>=|≥|v)?\s*(\d+\.\d+(?:\.\d+)?)")
        .expect("valid regex");
    let mut found = Vec::new();
    for (i, line) in text.lines().enumerate() {
        for caps in re.captures_iter(line) {
            found.push((i + 1, caps[1].to_string()));
        }
    }
    found
}

#[test]
fn every_rust_version_the_readmes_claim_is_the_workspace_msrv() {
    let msrv = workspace_msrv();
    let root = repo_root();

    let mut readmes = vec![("README.md".to_string(), readme())];
    for entry in std::fs::read_dir(&root).expect("read repo root").flatten() {
        let name = entry.file_name().to_string_lossy().to_string();
        // README.zh-CN.md, README.ja.md, … — translations are the sibling path
        // this audit keeps finding half-fixed.
        if name.starts_with("README.") && name.ends_with(".md") && name != "README.md" {
            if let Ok(text) = std::fs::read_to_string(entry.path()) {
                readmes.push((name, text));
            }
        }
    }
    assert!(
        readmes.len() > 1,
        "no translated READMEs found — this test would silently cover only English"
    );

    let english = rust_versions_claimed(&readmes[0].1);
    assert!(
        !english.is_empty(),
        "README.md states no Rust version at all; a reader cannot tell whether \
         their toolchain is new enough. (Or this test's extraction broke — it \
         must find the badge and the `Requires Rust …` line.)"
    );

    let mut wrong = Vec::new();
    for (file, text) in &readmes {
        for (line, version) in rust_versions_claimed(text) {
            if version != msrv {
                wrong.push(format!(
                    "{file}:{line}: claims Rust {version}, workspace rust-version is {msrv}"
                ));
            }
        }
    }
    assert!(
        wrong.is_empty(),
        "the READMEs state a Rust floor that is not the workspace MSRV:\n  {}\n\n\
         There is one floor. Every place that names it must name the same one.",
        wrong.join("\n  ")
    );
}

// ─── GEN-1: the README's pointer at `horus new --macro` must be true ────────

/// The paragraph that mentions `--macro`.
fn readme_macro_paragraph() -> String {
    let text = readme();
    let (_, prose) = split_fences(&text);
    for para in prose.split("\n\n") {
        if para.contains("--macro") {
            return para.to_string();
        }
    }
    panic!("the README no longer mentions `--macro`; the DSL must stay reachable");
}

#[test]
fn the_readme_describes_what_horus_new_macro_actually_scaffolds() {
    let generated = scaffolded_file("--macro", "src/main.rs");

    // Derived from the binary, not from a list kept by hand: the struct the DSL
    // declares, and the type/topic it publishes.
    let struct_name = regex::Regex::new(r"node!\s*\{\s*(\w+)")
        .unwrap()
        .captures(&generated)
        .map(|c| c[1].to_string())
        .expect("the --macro template must declare a node!");
    let (msg_type, topic) = regex::Regex::new(r#"(\w+)\s*:\s*(\w+)\s*->\s*"([^"]+)""#)
        .unwrap()
        .captures(&generated)
        .map(|c| (c[2].to_string(), c[3].to_string()))
        .expect("the --macro template must publish something");

    let para = readme_macro_paragraph();
    let mut missing = Vec::new();
    for (what, needle) in [
        ("the struct it declares", struct_name.as_str()),
        ("the message type it publishes", msg_type.as_str()),
        ("the topic it publishes on", topic.as_str()),
    ] {
        if !para.contains(needle) {
            missing.push(format!("{what}: {needle:?}"));
        }
    }
    assert!(
        missing.is_empty(),
        "the README's `--macro` sentence does not describe what the command \
         writes. Missing:\n  {}\n\nThe sentence is:\n{para}\n\n`horus new --macro` \
         actually produces:\n{generated}",
        missing.join("\n  ")
    );

    // The sentence used to read "`horus new --macro` scaffolds the same node
    // with the `node!` macro". It does not: the README's Quick Start is two
    // nodes over message!-declared types at 1 kHz with Miss::SafeMode, and
    // --macro emits one node publishing Twist at 100 Hz with Miss::Warn. The
    // test guarding that line was `readme.contains("--macro")`, which is true
    // however wrong the claim around it is.
    assert!(
        !para.contains("the same node"),
        "`horus new --macro` does not scaffold the node the README's Quick \
         Start shows — it scaffolds `{struct_name}` publishing `{msg_type}` on \
         `{topic}`. Do not claim sameness:\n{para}"
    );
}

// ─── GEN-1/GEN-2: every starter speaks the README's vocabulary ──────────────

/// One language's starter, and how to read the HORUS calls out of it.
///
/// `readme_contract.rs` checks Rust only, so the finding it closed ("a reader
/// who copies the README and a reader who runs the README's own command meet
/// different languages") stayed open for Python and C++ — where the starters
/// call `has_msg` and `horus::log::info`, neither of which the README showed.
struct StarterVocab {
    flag: &'static str,
    entry: &'static str,
    /// (description, regex with the API name in capture group 1, needle template)
    probes: &'static [(&'static str, &'static str, &'static str)],
}

const STARTERS: &[StarterVocab] = &[
    StarterVocab {
        flag: "-r",
        entry: "src/main.rs",
        probes: &[("a call on a stored topic", r"self\.\w+\.(\w+)\(", "{}(")],
    },
    StarterVocab {
        flag: "--macro",
        entry: "src/main.rs",
        probes: &[("a call on a stored topic", r"self\.\w+\.(\w+)\(", "{}(")],
    },
    StarterVocab {
        flag: "-p",
        entry: "main.py",
        probes: &[("a call on the node handle", r"node\.(\w+)\(", "{}(")],
    },
    StarterVocab {
        flag: "--cpp",
        entry: "src/main.cpp",
        probes: &[
            ("a log call", r"horus::log::(\w+)\(", "horus::log::{}("),
            ("a pub/sub factory", r"\b(advertise|subscribe)<", "{}<"),
        ],
    },
];

#[test]
fn the_generated_starters_use_no_horus_api_the_readme_never_shows() {
    let readme = readme();
    let mut unknown = Vec::new();

    for starter in STARTERS {
        let generated = scaffolded_file(starter.flag, starter.entry);
        for (what, pattern, needle_template) in starter.probes {
            let re = regex::Regex::new(pattern).expect("valid regex");
            let names: Vec<String> = re
                .captures_iter(&generated)
                .map(|c| c[1].to_string())
                .collect();
            // A probe that matches nothing proves nothing. Fail loudly rather
            // than pass vacuously if the template stops looking like itself.
            assert!(
                !names.is_empty(),
                "`horus new {} `: found no {what} in {} — the probe {pattern:?} \
                 no longer matches the template, so this test would pass while \
                 checking nothing:\n{generated}",
                starter.flag,
                starter.entry,
            );
            for name in names {
                let needle = needle_template.replace("{}", &name);
                if !readme.contains(&needle) {
                    unknown.push(format!(
                        "`horus new {} ` writes {needle} ({what}) — the README never shows it",
                        starter.flag
                    ));
                }
            }
        }
    }

    unknown.sort();
    unknown.dedup();
    assert!(
        unknown.is_empty(),
        "the starter a reader gets from the README's own `horus new` uses APIs \
         the README never introduces:\n  {}\n\nEither show them in the README or \
         drop them from the template.",
        unknown.join("\n  ")
    );
}

// ─── ONB-1: the README says which of its knobs a beginner may skip ──────────

#[test]
fn every_scheduler_knob_in_the_readme_quick_start_is_explained_in_prose() {
    let fences = readme_rust_fences();
    assert!(
        !fences.is_empty(),
        "the README has no Rust fence; the extraction broke"
    );
    let quick_start = &fences[0];
    let main_body = quick_start
        .find("fn main(")
        .map(|i| &quick_start[i..])
        .expect("the Quick Start must have a main()");

    // Derived from the snippet, so adding `.budget(...)` to it fails this test
    // until the prose accounts for it too.
    //
    // `new`/`hz`/`add`/`run`/`build` are plumbing every example has; the rest
    // are the timing and failure-policy knobs that make the front page feel
    // like it demands ten concepts before "hello world".
    let plumbing = ["hz", "add", "run", "build", "new"];
    let re = regex::Regex::new(r"\.(\w+)\(").unwrap();
    let mut knobs: Vec<String> = re
        .captures_iter(main_body)
        .map(|c| c[1].to_string())
        .filter(|n| !plumbing.contains(&n.as_str()))
        .collect();
    knobs.sort();
    knobs.dedup();
    assert!(
        !knobs.is_empty(),
        "no scheduler knobs found in the Quick Start — extraction broke:\n{main_body}"
    );

    // A bare `prose.contains(knob)` is not enough: "order" is inside
    // "recorder", and "rate" is inside "tick_rate", so two of the four knobs
    // reported themselves explained by words that have nothing to do with them.
    // Require the knob spelled as a call, at an identifier boundary.
    let (_, prose) = split_fences(&readme());
    let explained = |knob: &str| {
        regex::Regex::new(&format!(
            r"(?:^|[^A-Za-z0-9_]){}\(",
            regex::escape(knob)
        ))
        .expect("valid regex")
        .is_match(&prose)
    };
    let unexplained: Vec<&String> = knobs.iter().filter(|k| !explained(k)).collect();
    assert!(
        unexplained.is_empty(),
        "the README's first code sample uses {unexplained:?} and never mentions \
         them outside a code fence. A first-time reader cannot tell which of \
         these they need and which they can skip — say so in prose, or take \
         them out of the front-page sample.\n\nKnobs found: {knobs:?}"
    );
}

// ─── GEN-1: getting-started teaches one builder terminator ──────────────────

/// `.build()?` is what the README, all three templates and the Quick Start's
/// own bullet call canonical; `.done()` is a compatibility alias that returns
/// the same `Result`. A beginner page that writes `.done();` teaches a second
/// spelling *and* silently discards a `Result`.
#[test]
fn getting_started_code_uses_the_canonical_builder_terminator() {
    let pages = getting_started_pages();
    if pages.is_empty() {
        eprintln!("SKIP: horus-docs is not checked out beside horus");
        return;
    }

    let mut offenders = Vec::new();
    for (page, text) in &pages {
        let mut in_fence = false;
        for (i, line) in text.lines().enumerate() {
            if line.trim_start().starts_with("```") {
                in_fence = !in_fence;
                continue;
            }
            // Prose may name `.done()` as the alias it is; code must not use it.
            if in_fence && line.contains(".done()") {
                offenders.push(format!("{page}:{}: {}", i + 1, line.trim()));
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "these Getting Started code samples end a builder with `.done()`; the \
         canonical spelling — the one the README and every `horus new` template \
         emit — is `.build()?`:\n  {}",
        offenders.join("\n  ")
    );
}

// ─── ONB-1: the mistakes list records what the API could prevent ────────────

const API_PREVENTION_MARKER: &str = "**Could the API prevent this?**";

#[test]
fn every_common_mistake_records_whether_the_api_could_prevent_it() {
    let pages = getting_started_pages();
    if pages.is_empty() {
        eprintln!("SKIP: horus-docs is not checked out beside horus");
        return;
    }
    let (_, text) = pages
        .iter()
        .find(|(p, _)| p.replace('\\', "/") == "getting-started/common-mistakes.mdx")
        .expect("getting-started/common-mistakes.mdx must exist");

    // Section boundaries, derived from the page's own numbered headings.
    let heads: Vec<(usize, String)> = text
        .lines()
        .enumerate()
        .filter(|(_, l)| {
            l.starts_with("## ")
                && l[3..]
                    .split('.')
                    .next()
                    .is_some_and(|n| !n.is_empty() && n.chars().all(|c| c.is_ascii_digit()))
        })
        .map(|(i, l)| (i, l.to_string()))
        .collect();
    assert!(
        heads.len() >= 5,
        "found only {} numbered mistakes; the heading format changed and this \
         test would pass while checking nothing",
        heads.len()
    );

    let lines: Vec<&str> = text.lines().collect();
    let mut silent = Vec::new();
    for (idx, (start, head)) in heads.iter().enumerate() {
        let end = heads
            .get(idx + 1)
            .map(|(n, _)| *n)
            .unwrap_or_else(|| lines.len());
        let body = lines[*start..end].join("\n");
        if !body.contains(API_PREVENTION_MARKER) {
            silent.push(head.clone());
        }
    }

    assert!(
        silent.is_empty(),
        "every entry on a 'common mistakes' page is also a design bug report: \
         either the API already makes the mistake impossible, or it could and \
         does not. These entries do not say which — add a \
         {API_PREVENTION_MARKER} line:\n  {}",
        silent.join("\n  ")
    );
}

// ─── ONB-1: Getting Started publishes one reading order ─────────────────────

#[test]
fn the_quick_start_publishes_a_reading_order_that_resolves() {
    let pages = getting_started_pages();
    if pages.is_empty() {
        eprintln!("SKIP: horus-docs is not checked out beside horus");
        return;
    }
    let root = docs_root().expect("docs_root exists when pages do");
    let (_, text) = pages
        .iter()
        .find(|(p, _)| p.replace('\\', "/") == "getting-started/quick-start.mdx")
        .expect("getting-started/quick-start.mdx must exist");

    let section = text
        .split("\n## ")
        .find(|s| s.starts_with("The path through Getting Started"))
        .expect(
            "quick-start.mdx must publish a named reading order — a first-time \
             reader needs one sequence, not a sidebar of ten peers",
        );

    let re = regex::Regex::new(r"\]\((/[a-z0-9\-/]+)\)").unwrap();
    let links: Vec<String> = re
        .captures_iter(section)
        .map(|c| c[1].to_string())
        .collect();
    assert!(
        links.len() >= 3,
        "the reading order lists {} links; that is not a path:\n{section}",
        links.len()
    );

    let dead: Vec<&String> = links
        .iter()
        .filter(|href| {
            let rel = href.trim_start_matches('/');
            !root.join(format!("{rel}.mdx")).is_file() && !root.join(rel).join("index.mdx").is_file()
        })
        .collect();
    assert!(
        dead.is_empty(),
        "the published reading order points at pages that do not exist: {dead:?}"
    );
}
