//! Scaffold-layout contract — `horus new` must produce the tree the docs promise.
//!
//! A quick-start page's very first instruction is "run `horus new …`", and its
//! second is "open `<some path>`". If that path does not exist, the reader is
//! stuck on step 2 with no way to guess the right one. This is the cheapest
//! class of documentation breakage to test and among the most damaging to hit.
//!
//! The layouts asserted here are quoted from
//! `content/docs/getting-started/quick-start-python.mdx`, which tabulates the
//! entry point for every language flag, and the sibling quick-start pages.
//!
//! Hermetic and fast (three `horus new` invocations), so it runs on every PR.

use assert_cmd::cargo::cargo_bin_cmd;
use std::path::Path;

/// What a language flag is documented to scaffold.
struct Scaffold {
    /// Flag passed to `horus new`.
    flag: &'static str,
    /// Entry point the docs tell the reader to edit.
    entry: &'static str,
    /// Where the docs say it.
    cited: &'static str,
    /// Set when the CLI is known not to match, with the discrepancy spelled out.
    /// Delete the entry once it is resolved — `quarantine_is_not_stale` fails if
    /// a quarantined layout starts matching.
    quarantined: Option<&'static str>,
}

const SCAFFOLDS: &[Scaffold] = &[
    Scaffold {
        flag: "-r",
        entry: "src/main.rs",
        cited: "getting-started/quick-start-python.mdx:71 (flag table) and quick-start.mdx",
        quarantined: None,
    },
    Scaffold {
        flag: "-m",
        entry: "src/main.rs",
        cited: "getting-started/quick-start-python.mdx:72 (flag table)",
        quarantined: None,
    },
    Scaffold {
        flag: "--cpp",
        entry: "src/main.cpp",
        cited: "getting-started/quick-start-cpp.mdx:34",
        quarantined: None,
    },
    Scaffold {
        flag: "-p",
        entry: "src/main.py",
        cited: "getting-started/quick-start-python.mdx:61 and :70 (flag table)",
        quarantined: Some(
            "`horus new <name> -p` writes main.py at the PROJECT ROOT, not src/main.py \
             (horus_manager/src/commands/new.rs:611). Rust and C++ both scaffold into src/, \
             so Python is the odd one out, and 35 doc sites reference src/main.py. A reader \
             following quick-start-python is told to edit a file that does not exist. \
             Resolving it is a behaviour choice: either move the Python entry point into \
             src/ (dispatch.rs already detects src/main.py — see its test at :1377 — but \
             ~6 tests in new.rs encode the root placement), or correct the 35 doc sites.",
        ),
    },
];

/// Run `horus new <name> <flag>` in a fresh dir and return the project path.
fn scaffold(tmp: &Path, name: &str, flag: &str) -> std::path::PathBuf {
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
    let tmp = tempfile::tempdir().expect("temp dir");
    let mut violations = Vec::new();

    for (i, s) in SCAFFOLDS.iter().enumerate() {
        if s.quarantined.is_some() {
            continue;
        }
        let proj = scaffold(tmp.path(), &format!("proj{i}"), s.flag);
        if !proj.join(s.entry).is_file() {
            let actual: Vec<String> = walk(&proj)
                .iter()
                .filter_map(|p| {
                    p.strip_prefix(&proj)
                        .ok()
                        .map(|r| r.to_string_lossy().to_string())
                })
                .filter(|r| !r.starts_with(".horus"))
                .collect();
            violations.push(format!(
                "  horus new … {}  promises {}  ({})\n      actually created: {:?}",
                s.flag, s.entry, s.cited, actual
            ));
        }
    }

    assert!(
        violations.is_empty(),
        "`horus new` does not produce the documented layout:\n{}\n\n\
         A reader is told to open a file that does not exist, on step 2 of a \
         quick-start. Fix the scaffold or the doc page.",
        violations.join("\n")
    );
}

/// A quarantined layout that starts matching must leave the list.
#[test]
fn quarantine_is_not_stale() {
    let tmp = tempfile::tempdir().expect("temp dir");
    let mut fixed = Vec::new();

    for (i, s) in SCAFFOLDS.iter().enumerate() {
        if s.quarantined.is_none() {
            continue;
        }
        let proj = scaffold(tmp.path(), &format!("q{i}"), s.flag);
        if proj.join(s.entry).is_file() {
            fixed.push(format!("  horus new … {} now creates {}", s.flag, s.entry));
        }
    }

    assert!(
        fixed.is_empty(),
        "these layouts now match the docs but are still quarantined:\n{}\n\n\
         Clear their `quarantined` field in {} so regressions are caught again.",
        fixed.join("\n"),
        file!()
    );
}

/// Every scaffold must at least produce a manifest and a build directory —
/// the two things every quick-start says to expect.
#[test]
fn every_scaffold_has_manifest_and_build_dir() {
    let tmp = tempfile::tempdir().expect("temp dir");
    for (i, s) in SCAFFOLDS.iter().enumerate() {
        let proj = scaffold(tmp.path(), &format!("m{i}"), s.flag);
        assert!(
            proj.join("horus.toml").is_file(),
            "`horus new … {}` created no horus.toml",
            s.flag
        );
        assert!(
            proj.join(".horus").is_dir(),
            "`horus new … {}` created no .horus/ directory",
            s.flag
        );
    }
}

/// The generated manifest must satisfy `horus check`.
///
/// The scaffold and the validator are separate code paths; nothing else stops
/// `horus new` from emitting a manifest `horus check` immediately rejects, which
/// would break the documented `new → check` flow on a brand-new project.
#[test]
fn scaffolded_projects_pass_horus_check() {
    let tmp = tempfile::tempdir().expect("temp dir");
    for (i, s) in SCAFFOLDS.iter().enumerate() {
        let proj = scaffold(tmp.path(), &format!("c{i}"), s.flag);
        let out = cargo_bin_cmd!("horus")
            .args(["check", "."])
            .current_dir(&proj)
            .output()
            .expect("horus check runs");
        assert!(
            out.status.success(),
            "`horus check` rejects a freshly scaffolded `{}` project:\n{}{}",
            s.flag,
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );
    }
}

fn walk(dir: &Path) -> Vec<std::path::PathBuf> {
    let mut out = Vec::new();
    let Ok(rd) = std::fs::read_dir(dir) else {
        return out;
    };
    for e in rd.flatten() {
        let p = e.path();
        if p.is_dir() {
            out.extend(walk(&p));
        } else {
            out.push(p);
        }
    }
    out
}
