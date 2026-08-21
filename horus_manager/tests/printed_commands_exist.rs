//! Every `horus …` command this CLI prints must be a command it has.
//!
//! Messages that tell an operator what to run next are instructions, and a
//! wrong one costs more than no instruction at all: it looks authoritative and
//! ends in `error: unrecognized subcommand`. Four were wrong when this test was
//! written —
//!
//!   horus action send_goal …    (the Rust function's name; it is `send-goal`)
//!   horus auth generate-key     (step 3 of recovering from a failed login)
//!   horus pkg status …          (`pkg` has never been a subcommand)
//!   horus sim start …           (the `[scripts]` example a reader pastes)
//!
//! This scans the source for quoted `horus <sub> <sub>` strings and asks the
//! built binary whether the path resolves.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::process::Command;

fn horus() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_horus"))
}

fn src_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("src")
}

fn rust_sources(dir: &Path, out: &mut Vec<PathBuf>) {
    for entry in std::fs::read_dir(dir).expect("read src").flatten() {
        let p = entry.path();
        if p.is_dir() {
            rust_sources(&p, out);
        } else if p.extension().is_some_and(|e| e == "rs") {
            out.push(p);
        }
    }
}

/// Words that follow `horus ` in prose rather than naming a subcommand.
///
/// The scan is textual, so it also finds sentences like "the horus package" or
/// "a horus project". Listing them here keeps the test about instructions.
const NOT_COMMANDS: &[&str] = &[
    "package",
    "packages",
    "project",
    "projects",
    "prelude",
    "crates",
    "registry",
    "robotics",
    "binary",
    "source",
    "internal",
    "version",
    "workspace",
    "runtime",
    "node",
    "nodes",
    "topic",
    "topics",
    "config",
    "cache",
    "dir",
    "directory",
    "path",
    "prefix",
    "import",
    "from",
    "uses",
    "is",
    "should",
    "creation",
    "dep",
    "dependency",
    "core",
    "types",
    "manager",
    "sys",
    "py",
    "cpp",
    "macros",
    "net",
    "tf",
    "std",
    "toml",
    "lock",
    "home",
    "bin",
    "docs",
    "doc",
    "log",
    "logs",
];

/// Extract `horus <a> [<b>]` occurrences that are presented as commands.
///
/// Only where `horus` opens a quoted string or follows a backtick — how a
/// command is actually shown to a reader. Anything else is prose ("the horus
/// binary", "horus builds …") and matching it would drown the real findings.
fn printed_commands(src: &str) -> BTreeSet<(String, Option<String>)> {
    let mut found = BTreeSet::new();
    // Only printed strings. A comment explaining that `horus pkg` was never a
    // command must not be read as a claim that it is one.
    let src: String = src
        .lines()
        .filter(|l| !l.trim_start().starts_with("//"))
        .collect::<Vec<_>>()
        .join("\n");
    let src = src.as_str();
    let bytes = src.as_bytes();
    for (i, _) in src.match_indices("horus ") {
        let opener = i.checked_sub(1).map(|j| bytes[j]);
        if !matches!(opener, Some(b'"') | Some(b'\'') | Some(b'`')) {
            continue;
        }
        let rest = &src[i + "horus ".len()..];
        let mut words = rest.split(|c: char| !(c.is_ascii_alphanumeric() || c == '-' || c == '_'));
        let Some(first) = words.next().filter(|w| !w.is_empty()) else {
            continue;
        };
        if NOT_COMMANDS.contains(&first) || first.chars().next().is_some_and(|c| c.is_uppercase()) {
            continue;
        }
        let second = words
            .next()
            .filter(|w| !w.is_empty() && !NOT_COMMANDS.contains(w))
            .map(|w| w.to_string());
        found.insert((first.to_string(), second));
    }
    found
}

/// Whether `horus <args> --help` resolves.
fn resolves(args: &[&str]) -> bool {
    let out = Command::new(horus())
        .args(args)
        .arg("--help")
        .output()
        .expect("run horus");
    let text = String::from_utf8_lossy(&out.stderr);
    !text.contains("unrecognized subcommand") && !text.contains("invalid value")
}

#[test]
fn every_printed_horus_command_resolves() {
    let mut files = Vec::new();
    rust_sources(&src_dir(), &mut files);
    assert!(files.len() > 20, "expected to scan the whole crate");

    let mut candidates = BTreeSet::new();
    for f in &files {
        let src = std::fs::read_to_string(f).expect("read source");
        candidates.extend(printed_commands(&src));
    }
    assert!(
        candidates.len() > 20,
        "the scan found almost nothing — it has stopped working"
    );

    let mut bad = Vec::new();
    for (first, second) in &candidates {
        if !resolves(&[first.as_str()]) {
            bad.push(format!("horus {first}"));
            continue;
        }
        if let Some(second) = second {
            if !resolves(&[first.as_str(), second.as_str()]) {
                bad.push(format!("horus {first} {second}"));
            }
        }
    }

    assert!(
        bad.is_empty(),
        "these appear in printed strings but are not commands this CLI has:\n  {}\n\n\
         A wrong instruction costs more than no instruction: it looks \
         authoritative and ends in `error: unrecognized subcommand`.",
        bad.join("\n  ")
    );
}
