//! The contributor documentation must keep saying what the repository does.
//!
//! `CONTRIBUTING.md` spent long enough telling contributors that `cargo fmt`
//! was "optional, as it may fail in some cases" and that clippy "warnings are
//! acceptable" — in five places — for the advice to be actively costing people
//! merges. `ci.yml:93` runs `cargo fmt --all -- --check` unconditionally, and
//! `:135-136` runs clippy with `-D warnings` whenever the base branch is
//! `main`. Following the guide got you a red gate.
//!
//! Nothing caught it, and nothing could have: no test in this repository read
//! `CONTRIBUTING.md`. The same was true of every document under `docs/`, which
//! did not exist. Prose about the build is exactly the class of claim that
//! rots — it is written once, against a workflow that then changes — and this
//! repository already knows that, which is why `readme_contract.rs`,
//! `docs_contract.rs` and seven siblings exist.
//!
//! ## What this file asserts, and what it deliberately does not
//!
//! Only set membership: a name that exists in the source must appear in the
//! document that claims to cover it. That catches the failure that actually
//! happens — something is added to the code and the document is not updated —
//! and it is cheap enough to cost nothing, because every check here is a file
//! read and a substring search.
//!
//! It cannot check that the prose around a name is *true*. A test asserting
//! every `ci-success` job is named in `CONTRIBUTING.md` proves the names are
//! present, not that what is written about them is correct. Saying so plainly
//! matters: a suite that quietly implies more coverage than it has is the
//! failure mode `horus-docs/scripts/check-parity-coverage.mjs` was written to
//! warn about, and this file would otherwise invite the same false comfort.
//!
//! ## Why there is no new CI step
//!
//! `integration-tests.yml:127` runs `cargo test --workspace --exclude horus_py
//! --release --test '*'`, which selects every `tests/` target in the
//! workspace. This file is therefore already gated by `Integration Tests
//! Success`, a required context, without adding a job to a merge queue that is
//! the repository's real bottleneck.
//!
//! Run: `cargo test -p horus_manager --test contributor_docs_contract`

use std::fs;
use std::path::{Path, PathBuf};

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager has a parent")
        .to_path_buf()
}

fn read(rel: &str) -> String {
    let p = repo_root().join(rel);
    fs::read_to_string(&p).unwrap_or_else(|e| panic!("{} is not readable: {e}", p.display()))
}

/// `needs: [a, b, c]` for a named job, in declaration order.
fn needs_of(workflow: &str, job: &str) -> Vec<String> {
    let src = read(workflow);
    let anchor = format!("\n  {job}:\n");
    let start = src
        .find(&anchor)
        .unwrap_or_else(|| panic!("{workflow} no longer declares a `{job}:` job"));
    let rest = &src[start..];
    let line = rest
        .lines()
        .find(|l| l.trim_start().starts_with("needs:"))
        .unwrap_or_else(|| panic!("`{job}` in {workflow} has no `needs:` line"));
    let inside = line
        .split_once('[')
        .and_then(|(_, r)| r.split_once(']'))
        .map(|(v, _)| v)
        .unwrap_or_else(|| panic!("`needs:` of `{job}` is not an inline list: {line}"));
    inside
        .split(',')
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .collect()
}

/// Every job gating `CI Success` is named in the merge-gate table.
///
/// The list is checked as a joined string rather than name by name: several of
/// the job ids are ordinary English words (`check`, `test`, `doc`), so finding
/// them individually in a 950-line document would prove nothing at all.
#[test]
fn contributing_lists_the_jobs_that_gate_ci_success() {
    let needs = needs_of(".github/workflows/ci.yml", "ci-success");
    assert!(
        needs.len() >= 5,
        "parsed only {:?} from ci-success — the parser has drifted from the file",
        needs
    );
    let joined = needs.join(", ");
    let doc = read("CONTRIBUTING.md");
    assert!(
        doc.contains(&joined),
        "CONTRIBUTING.md does not list the jobs `CI Success` waits on.\n\
         ci.yml says: {joined}\n\
         Update the merge-gate table in CONTRIBUTING.md, which is the only place \
         a contributor can read what blocks their merge — the required contexts \
         themselves live in the branch-protection API and in no file."
    );
}

/// The specific wrong advice does not come back.
#[test]
fn contributing_does_not_call_formatting_optional_again() {
    let doc = read("CONTRIBUTING.md");
    // Phrases that only make sense as instructions. "warnings are acceptable"
    // is deliberately NOT here: a sentence explaining that the advice used to
    // say that, and does not any more, is a legitimate thing for this document
    // to contain, and banning the words would forbid describing the fix.
    for banned in [
        "cargo fmt` (optional",
        "Optionally run `cargo fmt`",
        "`cargo clippy` (warnings acceptable)",
        "Code formatted if possible",
    ] {
        assert!(
            !doc.contains(banned),
            "CONTRIBUTING.md contains {banned:?} again.\n\
             ci.yml:93 runs `cargo fmt --all -- --check` unconditionally and \
             :135-136 runs clippy with -D warnings against main. Telling \
             contributors otherwise is what this guard exists to stop."
        );
    }
}

/// Every workspace member has somewhere to belong in the source map.
#[test]
fn architecture_covers_every_workspace_member() {
    // Line-oriented, not `split_once(']')`: one member carries the trailing
    // comment "# C++ binding codegen (#[horus_api] proc macro)", and the `]`
    // inside it truncated an earlier version of this parser to four members —
    // which still looked like a working list.
    let root = read("Cargo.toml");
    let members: Vec<String> = root
        .lines()
        .skip_while(|l| !l.trim_start().starts_with("members = ["))
        .skip(1)
        .take_while(|l| l.trim() != "]")
        .filter_map(|l| l.split('"').nth(1))
        .map(str::to_string)
        .collect();
    assert!(
        members.len() >= 8,
        "parsed only {members:?} — the members parser has drifted"
    );

    let arch = read("docs/ARCHITECTURE.md");
    let missing: Vec<&String> = members.iter().filter(|m| !arch.contains(*m)).collect();
    assert!(
        missing.is_empty(),
        "docs/ARCHITECTURE.md never mentions {missing:?}.\n\
         It is the map a contributor reads to answer \"where does my change go\"; \
         a crate it does not name is a crate nobody can be told to put code in."
    );
}

/// Every loom model is accounted for in the testing guide.
///
/// These are the models nobody would otherwise know exist: four of them are
/// named in no workflow file, and run only because
/// `integration-tests.yml:127`'s `--test '*'` selects every target.
#[test]
fn the_testing_guide_names_every_loom_model() {
    let dir = repo_root().join("horus_core/tests");
    let mut models: Vec<String> = fs::read_dir(&dir)
        .expect("horus_core/tests is readable")
        .filter_map(|e| e.ok())
        .filter_map(|e| e.file_name().into_string().ok())
        .filter(|n| n.starts_with("loom_") && n.ends_with(".rs"))
        .map(|n| n.trim_end_matches(".rs").to_string())
        .collect();
    models.sort();
    assert!(
        models.len() >= 8,
        "found only {models:?} under horus_core/tests — the glob has drifted"
    );

    let doc = read("docs/TESTING.md");
    let missing: Vec<&String> = models.iter().filter(|m| !doc.contains(*m)).collect();
    assert!(
        missing.is_empty(),
        "docs/TESTING.md does not name {missing:?}.\n\
         A loom model absent from the guide is a concurrency proof nobody knows \
         to run when they touch the protocol it covers."
    );
}

/// No `blueprint section N` citation points at a section that does not exist.
///
/// Thirteen `horus_net` modules defer their normative behaviour to this
/// document by section number. Before it was written, every one of those
/// citations dangled.
#[test]
fn every_blueprint_section_horus_net_cites_exists() {
    let src_dir = repo_root().join("horus_net/src");
    let mut cited: Vec<u32> = Vec::new();
    let mut stack = vec![src_dir];
    while let Some(d) = stack.pop() {
        for e in fs::read_dir(&d)
            .expect("horus_net/src is readable")
            .flatten()
        {
            let p = e.path();
            if p.is_dir() {
                stack.push(p);
                continue;
            }
            if p.extension().and_then(|s| s.to_str()) != Some("rs") {
                continue;
            }
            let text = fs::read_to_string(&p).unwrap_or_default();
            for (i, _) in text.match_indices("blueprint section ") {
                let tail = &text[i + "blueprint section ".len()..];
                let digits: String = tail.chars().take_while(char::is_ascii_digit).collect();
                if let Ok(n) = digits.parse::<u32>() {
                    cited.push(n);
                }
            }
        }
    }
    cited.sort_unstable();
    cited.dedup();
    assert!(
        !cited.is_empty(),
        "no `blueprint section N` citations found in horus_net/src — either they \
         were all removed, or this scanner no longer sees what it guards"
    );

    let doc = read("horus_net/docs/BLUEPRINT.md");
    let headings: Vec<&str> = doc
        .lines()
        .filter(|l| l.trim_start().starts_with('#'))
        .collect();
    let missing: Vec<u32> = cited
        .iter()
        .copied()
        .filter(|n| {
            let a = format!(" {n}.");
            let b = format!("Section {n}");
            let c = format!(" {n} ");
            !headings.iter().any(|h| {
                h.contains(&a) || h.contains(&b) || h.ends_with(&format!(" {n}")) || h.contains(&c)
            })
        })
        .collect();
    assert!(
        missing.is_empty(),
        "horus_net cites blueprint section(s) {missing:?}, and \
         horus_net/docs/BLUEPRINT.md has no heading for them.\n\
         Cited sections: {cited:?}\n\
         A dangling citation sends a reader to a specification that does not \
         describe the thing they are reading."
    );
}

/// The index links every document beside it.
#[test]
fn the_docs_index_links_every_document() {
    let dir = repo_root().join("docs");
    let mut docs: Vec<String> = fs::read_dir(&dir)
        .expect("docs/ is readable")
        .filter_map(|e| e.ok())
        .filter_map(|e| e.file_name().into_string().ok())
        .filter(|n| n.ends_with(".md") && n != "README.md")
        .collect();
    docs.sort();
    assert!(
        !docs.is_empty(),
        "docs/ holds no documents — the walk is broken"
    );

    let index = read("docs/README.md");
    let missing: Vec<&String> = docs.iter().filter(|d| !index.contains(*d)).collect();
    assert!(
        missing.is_empty(),
        "docs/README.md does not link {missing:?}.\n\
         An unlinked document is one nobody finds."
    );
}
