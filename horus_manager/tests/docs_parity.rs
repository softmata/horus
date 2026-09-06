//! The other direction: what HORUS gained, the documentation has to have gained.
//!
//! # Why this suite exists
//!
//! The docs suites next to this one all run docs → code. `docs_cli_contract`
//! asks whether every flag the documentation *teaches* is accepted;
//! `docs_manifest` asks whether every `HORUS_*` the documentation *tells a
//! reader to set* is read somewhere. Both are the right question, and both are
//! blind to the failure that actually happens: someone adds a flag, an
//! environment variable or a release, and nothing anywhere requires the
//! documentation to catch up. The docs stay true and go incomplete, and no test
//! can tell the difference between "not documented yet" and "documented fine".
//!
//! That blindness is not hypothetical. An audit on 2026-08-29 found nine
//! environment variables read by shipped, non-test code and absent from the page
//! whose entire job is to list them — `HORUS_LOG_LEVEL` among them, whose
//! default is `debug`, so an operator who never heard of it pays the full
//! per-line logging cost inside every tick. It also found six dependency
//! examples pinning `horus = "0.2"` against a 0.4.0 crate, which Cargo reads as
//! `>=0.2.0, <0.3.0` — every one of them uninstallable — and four `cargo run
//! --bin` commands naming benchmark binaries that had been deleted, one of them
//! removed precisely because it manufactured competitor numbers.
//!
//! None of that was a wrong claim. Each was the documentation describing a
//! HORUS that had moved on, which is exactly what a docs → code check cannot
//! see.
//!
//! # What is deliberately not here
//!
//! Only surfaces where "complete" is decidable. Whether a *behavioural*
//! description is still accurate — that a critical node no longer e-stops on its
//! first deadline miss, say — is not something a set difference can answer, and
//! pretending otherwise would produce a test that is either silent or wrong.
//! Those need a reader.
//!
//! Run with a docs checkout:
//!
//! ```text
//! HORUS_DOCS_DIR=../horus-docs cargo test -p horus_manager --test docs_parity -- --ignored
//! ```

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager has a parent")
        .to_path_buf()
}

fn docs_dir() -> Option<PathBuf> {
    if let Ok(p) = std::env::var("HORUS_DOCS_DIR") {
        let p = PathBuf::from(p);
        return p.join("content/docs").is_dir().then_some(p);
    }
    let sibling = repo_root().parent()?.join("horus-docs");
    sibling.join("content/docs").is_dir().then_some(sibling)
}

fn doc_files(docs: &Path) -> Vec<PathBuf> {
    fn walk(dir: &Path, acc: &mut Vec<PathBuf>) {
        let Ok(rd) = std::fs::read_dir(dir) else {
            return;
        };
        for e in rd.flatten() {
            let p = e.path();
            if p.is_dir() {
                walk(&p, acc);
            } else if p.extension().is_some_and(|x| x == "mdx" || x == "md") {
                acc.push(p);
            }
        }
    }
    let mut v = Vec::new();
    walk(&docs.join("content/docs"), &mut v);
    v.sort();
    v
}

fn all_docs_text(docs: &Path) -> String {
    doc_files(docs)
        .iter()
        .filter_map(|f| std::fs::read_to_string(f).ok())
        .collect::<Vec<_>>()
        .join("\n")
}

/// Every `HORUS_*` name in a line, ignoring longer identifiers and bare prefixes.
fn horus_env_names(line: &str) -> Vec<String> {
    let mut out = Vec::new();
    let chars: Vec<char> = line.chars().collect();
    let needle: Vec<char> = "HORUS_".chars().collect();
    let mut i = 0;
    while i + needle.len() <= chars.len() {
        if chars[i..i + needle.len()] != needle[..] {
            i += 1;
            continue;
        }
        if i > 0 && (chars[i - 1].is_alphanumeric() || chars[i - 1] == '_') {
            i += 1;
            continue;
        }
        let mut j = i;
        while j < chars.len()
            && (chars[j].is_ascii_uppercase() || chars[j].is_ascii_digit() || chars[j] == '_')
        {
            j += 1;
        }
        let name: String = chars[i..j].iter().collect();
        if name.len() > "HORUS_".len() && !name.ends_with('_') && !out.contains(&name) {
            out.push(name);
        }
        i = j.max(i + 1);
    }
    out
}

/// The contents of every double-quoted literal on a line.
///
/// Deliberately simple: an escaped quote would end a literal early, which at
/// worst splits one name across two candidates and cannot invent one.
fn string_literals(line: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut inside = false;
    let mut cur = String::new();
    for c in line.chars() {
        if c == '"' {
            if inside {
                out.push(std::mem::take(&mut cur));
            }
            inside = !inside;
        } else if inside {
            cur.push(c);
        }
    }
    out
}

fn rust_sources(root: &Path) -> Vec<PathBuf> {
    fn walk(dir: &Path, acc: &mut Vec<PathBuf>) {
        let Ok(rd) = std::fs::read_dir(dir) else {
            return;
        };
        for e in rd.flatten() {
            let p = e.path();
            let name = e.file_name();
            let name = name.to_string_lossy();
            if p.is_dir() {
                // `target/` is build output and `tests/` is the harness, whose
                // `HORUS_*_CHILD` / `_TOPIC` / `_ROLE` names exist to coordinate
                // one test with the process it spawns. Requiring an operator-
                // facing page to list those would be nonsense.
                if name == "target" || name == "tests" || name.starts_with('.') {
                    continue;
                }
                walk(&p, acc);
            } else if p.extension().is_some_and(|x| x == "rs") {
                acc.push(p);
            }
        }
    }
    let mut v = Vec::new();
    walk(root, &mut v);
    v.sort();
    v
}

/// Names read only by a test, even though they live in a shipped file.
///
/// Each is inside a `#[cfg(test)]` module or gated behind one, so no operator
/// can reach it. Kept explicit rather than pattern-matched on the name: a real
/// variable that happens to contain `TEST` should not disappear from the page
/// because of its spelling.
const TEST_ONLY: &[&str] = &[
    // topic/header.rs — inside a `#[cfg(test)]` module that spawns a child to
    // strand a participant entry; unreachable outside the test binary.
    "HORUS_LIVE5_GHOST_TOPIC",
    "HORUS_LIVE5_GHOST_CAPACITY",
    // horus_cpp/src/c_api.rs — the child half of the FFI emit test.
    "HORUS_CPP_FFI_EMIT_CHILD",
    // scheduling/compute_executor.rs — the child half of the test that a
    // compute-only program still drains its RT diagnostics to stdout. Read
    // inside a `#[cfg(test)]` module; unreachable outside the test binary.
    "HORUS_COMPUTE_DIAG_CHILD",
    // commands/fmt.rs — the switch CI uses to turn a skipped tool into a
    // failure, so a missing clang-format is not silently tolerated.
    "HORUS_TEST_REQUIRE_TOOLS",
];

/// Every `HORUS_*` the shipped code reads must appear in the documentation.
///
/// The failure this catches is silence: a variable is added, it works, and the
/// only way to learn it exists is to read the source. Nine of them had
/// accumulated that way.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn every_env_var_the_code_reads_is_documented() {
    let docs = docs_dir().expect("set HORUS_DOCS_DIR=/path/to/horus-docs");
    let text = all_docs_text(&docs);

    let mut read: BTreeSet<String> = BTreeSet::new();
    for f in rust_sources(&repo_root()) {
        let Ok(src) = std::fs::read_to_string(&f) else {
            continue;
        };
        for line in src.lines() {
            // Only the forms that actually consume a variable. A name in a
            // comment documents the source; it does not read anything.
            if !(line.contains("env::var")
                || line.contains("environ.get")
                || line.contains(": &str ="))
            {
                continue;
            }
            // The name has to come from inside a string literal. Every real form
            // puts it there — `env::var("HORUS_X")`, `const E: &str = "HORUS_X"`,
            // `os.environ.get("HORUS_X")` — while a constant merely *named*
            // HORUS_something does not: `const HORUS_TOML: &str = "horus.toml"`
            // is a filename, and reading the identifier instead of the value
            // reported five of those as undocumented variables.
            for lit in string_literals(line) {
                for name in horus_env_names(&lit) {
                    read.insert(name);
                }
            }
        }
    }
    assert!(
        read.len() > 40,
        "only {} env vars found in the source — the scan is broken",
        read.len()
    );

    let missing: Vec<&String> = read
        .iter()
        .filter(|n| !TEST_ONLY.contains(&n.as_str()))
        .filter(|n| !text.contains(n.as_str()))
        .collect();

    assert!(
        missing.is_empty(),
        "{} environment variable(s) are read by shipped code and appear nowhere in \
         the documentation:\n  {}\n\n\
         Add them to content/docs/development/environment-variables.mdx with their \
         default and what they do. If one is reachable only from a test, add it to \
         TEST_ONLY in this file with the reason.",
        missing.len(),
        missing
            .iter()
            .map(|s| s.as_str())
            .collect::<Vec<_>>()
            .join("\n  ")
    );
}

/// A documented `horus = "x.y"` dependency must resolve to the crate we ship.
///
/// Cargo reads `"0.2"` as `>=0.2.0, <0.3.0`. Against a 0.4.0 crate every such
/// example is uninstallable, and it fails at the reader's first `horus build`
/// rather than anywhere we would see it.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn documented_horus_version_pins_resolve_to_this_crate() {
    let docs = docs_dir().expect("set HORUS_DOCS_DIR=/path/to/horus-docs");

    let manifest = std::fs::read_to_string(repo_root().join("horus/Cargo.toml"))
        .expect("horus/Cargo.toml is readable");
    let version = manifest
        .lines()
        .find_map(|l| l.strip_prefix("version = \""))
        .and_then(|l| l.split('"').next())
        .expect("horus/Cargo.toml declares a version");
    let (major, minor) = {
        let mut it = version.split('.');
        (
            it.next().unwrap_or("0").to_string(),
            it.next().unwrap_or("0").to_string(),
        )
    };

    let mut stale: Vec<String> = Vec::new();
    for f in doc_files(&docs) {
        let Ok(text) = std::fs::read_to_string(&f) else {
            continue;
        };
        for (i, line) in text.lines().enumerate() {
            // `horus = "0.4"`, and the `{ version = "0.4", features = [...] }`
            // form. `horus = "*"` is a deliberate wildcard and always resolves.
            let Some(rest) = line.split_once("horus = ").map(|(_, r)| r) else {
                continue;
            };
            let pin = if let Some(v) = rest.split_once("version = \"") {
                v.1.split('"').next().unwrap_or("")
            } else if rest.starts_with('"') {
                rest.trim_start_matches('"').split('"').next().unwrap_or("")
            } else {
                continue;
            };
            if pin.is_empty() || pin.starts_with('*') {
                continue;
            }
            let mut parts = pin.trim_start_matches(['^', '~', '=']).split('.');
            let (pmaj, pmin) = (parts.next().unwrap_or(""), parts.next().unwrap_or(""));
            // A 0.x crate has its compatibility range keyed on the minor.
            let ok = if major == "0" {
                pmaj == major && (pmin.is_empty() || pmin == minor)
            } else {
                pmaj == major
            };
            if !ok {
                stale.push(format!(
                    "{}:{} — `horus = \"{}\"` cannot resolve to {}",
                    f.file_name().unwrap_or_default().to_string_lossy(),
                    i + 1,
                    pin,
                    version
                ));
            }
        }
    }

    assert!(
        stale.is_empty(),
        "{} documented dependency pin(s) cannot install the crate this repository \
         ships ({version}):\n  {}\n\n\
         Cargo reads \"0.2\" as >=0.2.0,<0.3.0. Update the pin, or use \"*\".",
        stale.len(),
        stale.join("\n  ")
    );
}

/// Every benchmark binary the docs tell a reader to run must exist.
///
/// `cargo run --bin <name>` on a target that was deleted aborts before it
/// compiles anything. Four such commands survived the removal of the binaries
/// they named — including one deleted for manufacturing competitor percentiles,
/// which the docs went on citing as a source.
#[test]
#[ignore = "needs a horus-docs checkout; wired into the docs-contract workflow"]
fn documented_benchmark_binaries_exist() {
    let docs = docs_dir().expect("set HORUS_DOCS_DIR=/path/to/horus-docs");

    let bin_dir = repo_root().join("benchmarks/src/bin");
    let mut real: BTreeSet<String> = BTreeSet::new();
    if let Ok(rd) = std::fs::read_dir(&bin_dir) {
        for e in rd.flatten() {
            if let Some(stem) = e.path().file_stem() {
                real.insert(stem.to_string_lossy().to_string());
            }
        }
    }
    assert!(
        real.len() > 3,
        "only {} benchmark binaries found under {} — the scan is broken",
        real.len(),
        bin_dir.display()
    );

    let mut missing: Vec<String> = Vec::new();
    for f in doc_files(&docs) {
        let Ok(text) = std::fs::read_to_string(&f) else {
            continue;
        };
        for (i, line) in text.lines().enumerate() {
            let Some((_, rest)) = line.split_once("--bin ") else {
                continue;
            };
            let name: String = rest
                .chars()
                .take_while(|c| c.is_alphanumeric() || *c == '_')
                .collect();
            // Only benchmark invocations; `--bin` also appears for a reader's
            // own project binaries, which this repository knows nothing about.
            if name.is_empty() || !line.contains("benchmarks") {
                continue;
            }
            if !real.contains(&name) {
                missing.push(format!(
                    "{}:{} — `--bin {}`",
                    f.file_name().unwrap_or_default().to_string_lossy(),
                    i + 1,
                    name
                ));
            }
        }
    }

    assert!(
        missing.is_empty(),
        "{} documented benchmark command(s) name a binary that does not exist:\n  {}\n\n\
         benchmarks/src/bin holds: {:?}\n\
         Update the command, or restore the binary.",
        missing.len(),
        missing.join("\n  "),
        real
    );
}
