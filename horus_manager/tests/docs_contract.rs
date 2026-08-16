//! Docs contract tests — guarantee the CLI never breaks the published docs.
//!
//! # Why this exists
//!
//! Every `horus …` command printed in the documentation at
//! <https://github.com/softmata/horus-docs> is a promise: a user will copy it,
//! paste it into a terminal, and expect it to work. Nothing in this repository
//! used to check those promises.
//!
//! The docs repo *does* verify its own code blocks (`scripts/verify-*.mjs` +
//! `.github/workflows/verify-docs.yml`), but that workflow clones `horus@main`.
//! It answers "did the docs drift from released horus?" — never "does this horus
//! PR break the docs?". A PR that renames a flag turns every affected page into
//! a broken instruction and CI here stays green.
//!
//! These tests close that direction: the docs are treated as an executable
//! contract that the CLI must satisfy.
//!
//! # Two modes
//!
//! * **Live** — set `HORUS_DOCS_DIR`, or check `horus-docs` out beside this repo.
//!   Every `horus …` invocation in every `.mdx` page is re-extracted and verified.
//!   This is the authoritative run (CI does this against a pinned docs commit).
//! * **Hermetic** — with no docs checkout, the tests fall back to the committed
//!   snapshot in `tests/fixtures/docs-cli-contract.json` so they still run on
//!   every PR with no network. Regenerate it with:
//!
//!   ```bash
//!   HORUS_DOCS_DIR=../horus-docs cargo test -p horus_manager \
//!       --test docs_contract -- --ignored regenerate_docs_cli_contract
//!   ```
//!
//! # What counts as a violation
//!
//! A documented flag is satisfied when it appears anywhere in that command's own
//! `--help` or any descendant's. A *pass-through* command (`horus cargo …`,
//! `horus monitor …`) forwards unknown arguments to another tool, so its flags
//! are not clap's to validate and are skipped — see [`PASS_THROUGH`].

use assert_cmd::cargo::cargo_bin_cmd;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::process::Command;

// ─── Contract model ─────────────────────────────────────────────────────────

/// One `horus …` invocation the documentation tells a user to run.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
struct DocInvocation {
    /// Top-level subcommand, e.g. `run`.
    command: String,
    /// Nested subcommand when present, e.g. `list` in `horus topic list`.
    sub: Option<String>,
    /// Long flags used, e.g. `--release`.
    flags: Vec<String>,
    /// Doc page, relative to the docs repo root.
    doc_file: String,
    /// 1-indexed line of the fenced block the command came from.
    doc_line: usize,
}

/// The full committed snapshot.
#[derive(Debug, Serialize, Deserialize)]
struct Contract {
    /// Docs commit this was generated from, when known.
    source_commit: Option<String>,
    invocations: Vec<DocInvocation>,
}

/// Commands that forward unrecognized arguments to another program. Their flags
/// belong to that program, not to clap, so `--help` cannot be used to validate
/// them and a "missing" flag here means nothing.
///
/// `cargo`/`pip`/`cmake` are documented as transparent proxies; `monitor` is a
/// plugin shim that passes `[ARGS]...` to the `horus-monitor` binary.
const PASS_THROUGH: &[&str] = &["cargo", "pip", "cmake", "monitor", "self"];

/// Subcommands provided by optional plugins rather than the core binary. A bare
/// checkout legitimately does not have these, so their absence is not a docs bug.
/// Documented as installed via `horus install --plugin <name>`.
const PLUGIN_PROVIDED: &[&str] = &["sim3d", "sim2d", "viz", "studio"];

/// Pages that teach users to write their own `horus <name>` subcommand. The
/// example names there are deliberately not core commands, so "this subcommand
/// does not exist" is the documented behavior rather than a defect.
fn is_plugin_authoring_page(doc_file: &str) -> bool {
    doc_file.starts_with("content/docs/plugins/")
}

/// Placeholder command names the docs invent to illustrate a user's own plugin.
/// They must NOT resolve in a bare checkout — that is the point of the example.
const EXAMPLE_PLUGIN_NAMES: &[&str] = &[
    "mytool",      // cli-reference.mdx, project-local plugin trust walkthrough
    "topic-stats", // plugins/creating-plugins.mdx, the tutorial's sample plugin
];

/// Documented top-level commands the binary rejects. Same contract as
/// [`QUARANTINED_FLAGS`]: real defects, listed so the suite gates new ones.
/// `quarantine_is_not_stale` fails once a name starts resolving.
/// Documented `horus <command> <sub>` pairs the binary rejects. Same contract as
/// [`QUARANTINED_FLAGS`].
const QUARANTINED_SUBCOMMANDS: &[(&str, &str, &str)] = &[];

const QUARANTINED_COMMANDS: &[(&str, &str)] = &[];

/// Known-broken `(command, flag)` pairs: documented today, rejected by the CLI.
///
/// Every entry is a real user-facing defect found by this harness — a user who
/// copies that line gets `error: unexpected argument`. They are quarantined so
/// the suite can gate *new* regressions instead of staying permanently red, not
/// because they are acceptable.
///
/// **Fix the doc page (or restore the flag), then delete the entry.** The
/// `quarantine_is_not_stale` test fails once a pair starts working, so this list
/// cannot silently rot.
const QUARANTINED_FLAGS: &[(&str, &str, &str)] = &[
    // Empty: every entry was fixed on the docs side. `quarantine_is_not_stale`
    // fails both when a quarantined pair starts working AND when nothing
    // documents it any more, so entries cannot linger here as dead exemptions.
];

fn is_quarantined(command: &str, flag: &str) -> bool {
    QUARANTINED_FLAGS
        .iter()
        .any(|(c, f, _)| *c == command && *f == flag)
}

// ─── Locating things ────────────────────────────────────────────────────────

fn repo_root() -> PathBuf {
    // CARGO_MANIFEST_DIR is horus_manager/; the workspace root is its parent.
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager has a parent")
        .to_path_buf()
}

/// Find a horus-docs checkout, if one is available.
fn docs_dir() -> Option<PathBuf> {
    if let Ok(p) = std::env::var("HORUS_DOCS_DIR") {
        let p = PathBuf::from(p);
        return p.join("content/docs").is_dir().then_some(p);
    }
    // Sibling checkout — the layout CI and most contributors use.
    let sibling = repo_root().parent()?.join("horus-docs");
    sibling.join("content/docs").is_dir().then_some(sibling)
}

fn fixture_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/docs-cli-contract.json")
}

// ─── Extraction ─────────────────────────────────────────────────────────────

/// Pull every `horus …` command out of the shell blocks of one `.mdx` page.
fn extract_from_page(text: &str, rel: &str) -> Vec<DocInvocation> {
    let mut out = Vec::new();
    let mut in_shell = false;
    let mut fence_lang = String::new();

    for (idx, raw) in text.lines().enumerate() {
        let line = raw.trim_end();

        if let Some(rest) = line.trim_start().strip_prefix("```") {
            if in_shell {
                in_shell = false;
                fence_lang.clear();
            } else {
                // ```bash, ```bash:file.sh, ```shell …
                let lang = rest
                    .split([':', ' ', ','])
                    .next()
                    .unwrap_or("")
                    .trim()
                    .to_ascii_lowercase();
                in_shell = matches!(lang.as_str(), "bash" | "sh" | "shell" | "console" | "zsh");
                fence_lang = lang;
            }
            continue;
        }
        if !in_shell || fence_lang.is_empty() {
            continue;
        }

        if let Some(inv) = parse_horus_line(line, rel, idx + 1) {
            out.push(inv);
        }
    }
    out
}

/// Parse a single shell line into an invocation, if it runs `horus`.
fn parse_horus_line(line: &str, rel: &str, lineno: usize) -> Option<DocInvocation> {
    let mut s = line.trim();

    // A comment, checked before prompt-stripping: `#` is also the root prompt,
    // but in documentation it is overwhelmingly a comment marker, and reading
    // `# horus run --release` as a command would treat commented-out examples
    // (and prose about commands) as promises the CLI has to keep.
    if s.starts_with('#') {
        return None;
    }
    // Strip a shell prompt.
    for p in ["$ ", "> "] {
        if let Some(r) = s.strip_prefix(p) {
            s = r.trim();
            break;
        }
    }
    // Ignore anything with shell plumbing — we cannot reason about it reliably.
    if s.contains('|') || s.contains("&&") || s.contains(';') || s.contains('`') {
        return None;
    }
    // Only leading `horus`; `sudo horus` and env-prefixed forms are out of scope.
    let rest = s.strip_prefix("horus ")?;

    // Everything after a bare `--` is forwarded to another tool, not parsed by clap.
    let effective = rest.split(" -- ").next().unwrap_or(rest);

    let mut toks = effective.split_whitespace().filter(|t| !t.is_empty());
    let command = toks.next()?.to_string();
    // `horus --version` / `horus -V` are top-level options, not subcommands.
    if command.starts_with('-') {
        return None;
    }
    if !command
        .chars()
        .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
    {
        return None; // a path, a variable, not a subcommand
    }

    let mut sub = None;
    let mut flags = BTreeSet::new();
    // Subcommands precede flags in every real invocation
    // (`horus topic list --json`, `horus env freeze`). Once a flag appears, any
    // later bare word is that flag's *value*, not a subcommand: in
    // `horus bb --anomalies --last 20` the `20` belongs to `--last`, and
    // reading it as a subcommand reported a correct doc line as a defect.
    let mut seen_flag = false;
    let mut first_positional = true;
    for t in toks {
        if let Some(f) = t.strip_prefix("--") {
            seen_flag = true;
            // `--flag=value` → `--flag`
            let name = f.split('=').next().unwrap_or(f);
            if !name.is_empty() && name.chars().all(|c| c.is_ascii_lowercase() || c == '-') {
                flags.insert(format!("--{name}"));
            }
        } else if t.starts_with('-') {
            seen_flag = true;
        } else if first_positional && !seen_flag {
            first_positional = false;
            // Treat a bare lowercase word as a nested subcommand candidate.
            let looks_like_sub = t
                .chars()
                .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
                && !t.contains('.')
                && !t.contains('/');
            if looks_like_sub {
                sub = Some(t.to_string());
            }
        }
    }

    Some(DocInvocation {
        command,
        sub,
        flags: flags.into_iter().collect(),
        doc_file: rel.to_string(),
        doc_line: lineno,
    })
}

/// Walk `content/docs/**/*.mdx` and extract every documented invocation.
fn extract_all(docs: &Path) -> Vec<DocInvocation> {
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
    let mut files = Vec::new();
    walk(&docs.join("content/docs"), &mut files);
    files.sort();

    let mut out = Vec::new();
    for f in files {
        let Ok(text) = std::fs::read_to_string(&f) else {
            continue;
        };
        let rel = f
            .strip_prefix(docs)
            .unwrap_or(&f)
            .to_string_lossy()
            .replace('\\', "/");
        out.extend(extract_from_page(&text, &rel));
    }
    out
}

// ─── The real CLI surface ───────────────────────────────────────────────────

fn help_for(path: &[&str]) -> Option<String> {
    let mut cmd = cargo_bin_cmd!("horus");
    for p in path {
        cmd.arg(p);
    }
    cmd.arg("--help");
    let out = cmd.output().ok()?;
    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    (out.status.success() && text.contains("Usage:")).then_some(text)
}

/// Long flags mentioned anywhere in a help text.
fn flags_in(help: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    let bytes: Vec<char> = help.chars().collect();
    let mut i = 0;
    while i + 2 < bytes.len() {
        if bytes[i] == '-' && bytes[i + 1] == '-' && bytes[i + 2].is_ascii_alphabetic() {
            // not part of a longer token like `x--y`
            let prev_ok = i == 0 || !bytes[i - 1].is_ascii_alphanumeric();
            let mut j = i + 2;
            while j < bytes.len() && (bytes[j].is_ascii_alphanumeric() || bytes[j] == '-') {
                j += 1;
            }
            if prev_ok {
                out.insert(bytes[i..j].iter().collect::<String>());
            }
            i = j;
        } else {
            i += 1;
        }
    }
    out
}

/// Subcommand names listed in a help text.
///
/// Handles both shapes horus produces:
///  * stock clap — a single `Commands:` block (what nested commands print), and
///  * the top-level hand-written `help_template`, which groups commands under
///    category headers (`Project:`, `Introspection:`, `Plugins:` …) instead.
///
/// Entries are `  name[, alias]   description`. Sections after `Options:` are
/// ignored, which drops the flag list and the `Quick Start:` / `More examples:`
/// prose blocks in `after_help` — those contain full command lines
/// (`horus new my_robot -r`), not subcommand names.
fn children_in(help: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut in_section = false;

    for line in help.lines() {
        let t = line.trim_end();

        // Section header: unindented and ending in ':'.
        if !t.starts_with(' ') && !t.is_empty() {
            let header = t.trim();
            if header.ends_with(':') {
                // Everything from Options: onward is flags or prose examples.
                let lowered = header.to_ascii_lowercase();
                in_section = !matches!(
                    lowered.as_str(),
                    "options:" | "usage:" | "arguments:" | "quick start:" | "more examples:"
                ) && !lowered.starts_with("usage");
            } else {
                in_section = false;
            }
            continue;
        }
        if !in_section || t.trim().is_empty() {
            continue;
        }

        let mut it = t.split_whitespace();
        let Some(first) = it.next() else { continue };
        // `launch, l` → `launch`
        let name = first.trim_end_matches(',');
        // A description must follow on the same line, or this is not an entry.
        if it.next().is_none() {
            continue;
        }
        if name.is_empty()
            || name == "help"
            // `horus new …` / `cd my_robot` — example command lines, not names.
            || name == "horus"
            || name == "cd"
            || !name
                .chars()
                .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
        {
            continue;
        }
        out.push(name.to_string());
    }

    out.sort();
    out.dedup();
    out
}

/// Long flags clap defines but hides from `--help` (`#[arg(hide = true)]`).
///
/// `--help` is a *display* artefact; acceptance is the contract. `horus install
/// --ver 1.2.0` works and is documented, but never appears in help text, so a
/// help-derived universe reported it as a defect. Read from the CLI definition
/// so the two cannot disagree.
fn hidden_long_flags() -> BTreeSet<String> {
    let src = std::fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("src/main.rs"))
        .expect("main.rs is readable");

    let mut out = BTreeSet::new();
    let mut rest = src.as_str();
    while let Some(pos) = rest.find("long = \"") {
        rest = &rest[pos + "long = \"".len()..];
        let Some(name) = rest.split('"').next() else {
            break;
        };
        // Only the ones marked hidden: everything else is already in `--help`,
        // and harvesting all of them would make the check accept any flag on
        // any command.
        let window_end = rest.find(']').unwrap_or(0);
        if rest[..window_end].contains("hide = true") && !name.is_empty() {
            out.insert(format!("--{name}"));
        }
    }
    out
}

/// Every long flag reachable under `horus <command>`, including descendants.
fn flag_universe(command: &str) -> Option<BTreeSet<String>> {
    let top = help_for(&[command])?;
    let mut flags = flags_in(&top);
    flags.extend(hidden_long_flags());
    for child in children_in(&top) {
        if let Some(h2) = help_for(&[command, &child]) {
            flags.extend(flags_in(&h2));
            for gc in children_in(&h2) {
                if let Some(h3) = help_for(&[command, &child, &gc]) {
                    flags.extend(flags_in(&h3));
                }
            }
        }
    }
    Some(flags)
}

// ─── Loading the contract ───────────────────────────────────────────────────

/// Live docs when available, else the committed snapshot.
fn load_contract() -> (Vec<DocInvocation>, &'static str) {
    if let Some(d) = docs_dir() {
        let live = extract_all(&d);
        if !live.is_empty() {
            return (live, "live docs checkout");
        }
    }
    let path = fixture_path();
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "no horus-docs checkout (set HORUS_DOCS_DIR) and no snapshot at {}: {e}",
            path.display()
        )
    });
    let c: Contract = serde_json::from_str(&text).expect("snapshot is valid JSON");
    (c.invocations, "committed snapshot")
}

// ─── Tests ──────────────────────────────────────────────────────────────────

/// Every command the CLI's own `--help` advertises must actually be invocable.
///
/// Fully hermetic — no docs checkout involved. It exists because `horus`'s
/// top-level help uses a hand-written `help_template`, so the advertised command
/// list is not derived from the clap tree and can drift out of sync with it.
/// It already had: `enable`, `disable` and `verify` were moved under
/// `horus plugin …` but stayed listed at top level, so `horus --help` told users
/// to run three commands that answer `error: unrecognized subcommand`. The docs
/// then faithfully copied all three.
///
/// This is the cheapest, highest-signal check in the file: it needs nothing but
/// the binary, and self-inconsistent help is always a bug.
#[test]
fn advertised_subcommands_are_invocable() {
    let top = help_for(&[]).expect("`horus --help` must succeed");
    let advertised = children_in(&top);
    assert!(
        advertised.len() >= 20,
        "parsed only {} commands out of `horus --help` — the parser broke, making \
         this test vacuous:\n{top}",
        advertised.len()
    );

    let broken: Vec<&String> = advertised
        .iter()
        .filter(|c| help_for(&[c]).is_none())
        .collect();

    assert!(
        broken.is_empty(),
        "`horus --help` advertises commands the binary rejects: {broken:?}\n\n\
         The top-level help is a hand-written template (see `help_template` in \
         horus_manager/src/main.rs), so it does not track the real clap tree. \
         Either re-add these commands or correct the template — every one of them \
         is a command users are being told exists.",
    );
}

/// Check names printed by `horus doctor`, read from the binary's real output.
///
/// Parsed rather than grepped out of `doctor.rs`, whose test fixtures define
/// categories like `"Test"` and `"Warn"` that never reach a user.
fn doctor_check_names() -> BTreeSet<String> {
    let tmp = tempfile::tempdir().expect("temp dir");
    // A minimal project, so the manifest/language checks have something to say.
    std::fs::create_dir_all(tmp.path().join("src")).unwrap();
    std::fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"probe\"\nversion = \"0.1.0\"\n",
    )
    .unwrap();
    std::fs::write(tmp.path().join("src/main.rs"), "fn main() {}\n").unwrap();

    let out = cargo_bin_cmd!("horus")
        .arg("doctor")
        .current_dir(tmp.path())
        .output()
        .expect("horus doctor runs");
    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );

    // Rows look like `  * Toolchains — 3/8 tools found` / `  ! Real-Time — …`.
    let mut names = BTreeSet::new();
    for line in text.lines() {
        let t = line.trim_start();
        let Some(rest) = t
            .strip_prefix("* ")
            .or_else(|| t.strip_prefix("! "))
            .or_else(|| t.strip_prefix("x "))
        else {
            continue;
        };
        // Only the em dash separates the name from its detail; check names
        // themselves contain hyphens (`Real-Time`), so splitting on `-` too
        // truncated them to `Real`.
        let name = rest.split('—').next().unwrap_or("").trim().to_string();
        if !name.is_empty() && name.len() < 32 {
            names.insert(name);
        }
    }
    names
}

/// The `horus doctor` table in the install guide must name the real checks.
///
/// `getting-started/installation.mdx:422` tabulates every check with an "OK
/// means" / "FAIL means" column, so a reader consults it by row name when a
/// check fails. A row that does not exist sends them looking for output they
/// will never see; a check that exists but is missing from the table leaves a
/// failure undocumented.
#[test]
fn doctor_check_names_match_the_documented_table() {
    // Transcribed from the table at getting-started/installation.mdx:422.
    const DOCUMENTED: &[&str] = &[
        "Toolchains",
        "Manifest",
        "Shared Memory",
        "Plugins",
        "Disk",
        "Languages",
        "Dependencies",
        "Drivers",
        "System Deps",
    ];
    /// Rows the docs list that `horus doctor` does not print, and the reverse.
    /// Both directions are user-visible defects; quarantined so the suite gates
    /// new drift rather than staying red on drift that predates it.
    const KNOWN_MISMATCH: &[(&str, &str)] = &[
        (
            "Drivers",
            "documented, but the check is called `Hardware` — a reader whose \
             hardware check fails finds no `Drivers` row in their output",
        ),
        ("Hardware", "printed, but the table calls it `Drivers`"),
        (
            "Real-Time",
            "printed on every run (and warns on a stock kernel) but absent from \
             the table, so the most common warning a new user sees is undocumented",
        ),
        ("Network", "printed on every run but absent from the table"),
    ];

    let actual = doctor_check_names();
    assert!(
        actual.len() >= 5,
        "parsed only {actual:?} out of `horus doctor` — the parser broke, which \
         would make this test vacuous"
    );

    let quarantined = |n: &str| KNOWN_MISMATCH.iter().any(|(k, _)| *k == n);

    let missing: Vec<&str> = DOCUMENTED
        .iter()
        .copied()
        .filter(|d| !actual.contains(*d) && !quarantined(d))
        .collect();
    let undocumented: Vec<&String> = actual
        .iter()
        .filter(|a| !DOCUMENTED.contains(&a.as_str()) && !quarantined(a))
        .collect();

    assert!(
        missing.is_empty() && undocumented.is_empty(),
        "the `horus doctor` table at getting-started/installation.mdx:422 no \
         longer matches the binary.\n  documented but never printed: {missing:?}\n  \
         printed but undocumented: {undocumented:?}\n\nactual checks: {actual:?}",
    );
}

/// A quarantined doctor row that starts matching must leave the list.
#[test]
fn doctor_quarantine_is_not_stale() {
    let actual = doctor_check_names();
    // `Drivers` is quarantined precisely because it is documented and absent.
    assert!(
        !actual.contains("Drivers"),
        "`horus doctor` now prints a `Drivers` check, so the docs table is \
         correct — remove the Drivers/Hardware entries from KNOWN_MISMATCH"
    );
}

/// Every documented `horus <command>` must be a command the binary accepts.
#[test]
fn documented_subcommands_exist() {
    let (invocations, source) = load_contract();
    assert!(
        !invocations.is_empty(),
        "extracted no documented commands from {source} — the extractor is broken, \
         which would make every other test in this file vacuous"
    );

    let mut checked = BTreeSet::new();
    let mut missing: BTreeMap<String, Vec<String>> = BTreeMap::new();

    for inv in &invocations {
        if PLUGIN_PROVIDED.contains(&inv.command.as_str()) {
            continue; // shipped by an optional plugin, not the core binary
        }
        // The plugins section teaches users to *author* new `horus <name>`
        // commands, so it necessarily invents names (`horus mytool`,
        // `horus topic-stats`) that the core binary must not have. Those pages
        // are still covered for flags on real commands, and for anything the
        // top-level help advertises via advertised_subcommands_are_invocable.
        if is_plugin_authoring_page(&inv.doc_file)
            || EXAMPLE_PLUGIN_NAMES.contains(&inv.command.as_str())
            || QUARANTINED_COMMANDS.iter().any(|(c, _)| *c == inv.command)
        {
            continue;
        }
        if !checked.insert(inv.command.clone()) {
            continue;
        }
        if help_for(&[&inv.command]).is_none() {
            missing
                .entry(inv.command.clone())
                .or_default()
                .push(format!("{}:{}", inv.doc_file, inv.doc_line));
        }
    }

    assert!(
        missing.is_empty(),
        "documented subcommands the `horus` binary rejects (source: {source}):\n{}\n\n\
         Each is a command a user is told to run that fails immediately. Either restore \
         the subcommand or update the doc page.",
        missing
            .iter()
            .map(|(c, where_)| format!("  horus {c}  — documented at {}", where_.join(", ")))
            .collect::<Vec<_>>()
            .join("\n")
    );
}

/// Whether a help text advertises positional arguments.
fn takes_positional(help: &str) -> bool {
    help.lines().any(|l| l.trim_end() == "Arguments:")
}

/// A documented `horus <command> <sub>` must name a real nested subcommand.
///
/// `documented_subcommands_exist` only checks the top level, so
/// `horus env freeze` passed on the strength of `env` alone while `freeze` does
/// not exist — the binary answers `error: unexpected argument 'freeze' found`.
/// A whole documented command family (`env freeze/restore/list/show`) sat behind
/// that gap.
///
/// Only commands that actually take subcommands are checked. For the rest the
/// first positional is an argument (`horus install rplidar`, `horus new bot`),
/// not a subcommand, and treating it as one would flag every example.
#[test]
fn documented_nested_subcommands_exist() {
    let (invocations, source) = load_contract();
    #[allow(clippy::type_complexity)]
    let mut takes_subcommands: BTreeMap<String, Option<(BTreeSet<String>, bool)>> = BTreeMap::new();
    let mut violations: BTreeMap<(String, String), Vec<String>> = BTreeMap::new();

    for inv in &invocations {
        let Some(sub) = inv.sub.as_deref() else {
            continue;
        };
        if PASS_THROUGH.contains(&inv.command.as_str())
            || PLUGIN_PROVIDED.contains(&inv.command.as_str())
            || EXAMPLE_PLUGIN_NAMES.contains(&inv.command.as_str())
            || QUARANTINED_COMMANDS.iter().any(|(c, _)| *c == inv.command)
            || QUARANTINED_SUBCOMMANDS
                .iter()
                .any(|(c, sc, _)| *c == inv.command && *sc == sub)
        {
            continue;
        }

        let entry = takes_subcommands
            .entry(inv.command.clone())
            .or_insert_with(|| {
                help_for(&[&inv.command])
                    .map(|h| (children_in(&h).into_iter().collect(), takes_positional(&h)))
            });
        let Some((children, positional)) = entry else {
            continue;
        };
        // No `Commands:` block but an `Arguments:` section: this command takes a
        // value, not a subcommand — `horus install rplidar` is fine.
        if children.is_empty() && *positional {
            continue;
        }
        // Neither: it cannot take a bare word at all. `horus env` is
        // `horus env [OPTIONS]`, so `horus env freeze` is always wrong.
        if !children.is_empty() && children.contains(sub) {
            continue;
        }
        {
            violations
                .entry((inv.command.clone(), sub.to_string()))
                .or_default()
                .push(format!("{}:{}", inv.doc_file, inv.doc_line));
        }
    }

    assert!(
        violations.is_empty(),
        "documented subcommands the binary does not have (source: {source}):\n{}\n\n\
         The parent command exists, so a top-level check passes; the reader still \
         gets `error: unexpected argument`.",
        violations
            .iter()
            .map(|((c, sc), where_)| format!(
                "  horus {c} {sc}  — documented at {}",
                where_
                    .iter()
                    .take(2)
                    .cloned()
                    .collect::<Vec<_>>()
                    .join(", ")
            ))
            .collect::<Vec<_>>()
            .join("\n")
    );
}

/// Every documented long flag must exist somewhere in that command's tree.
#[test]
fn documented_flags_exist() {
    let (invocations, source) = load_contract();
    let mut universes: BTreeMap<String, Option<BTreeSet<String>>> = BTreeMap::new();
    let mut violations: BTreeMap<(String, String), Vec<String>> = BTreeMap::new();

    for inv in &invocations {
        if inv.flags.is_empty()
            || PASS_THROUGH.contains(&inv.command.as_str())
            || PLUGIN_PROVIDED.contains(&inv.command.as_str())
        {
            continue;
        }
        let universe = universes
            .entry(inv.command.clone())
            .or_insert_with(|| flag_universe(&inv.command));
        let Some(universe) = universe else { continue }; // missing subcommand: other test owns it

        for f in &inv.flags {
            if !universe.contains(f) && !is_quarantined(&inv.command, f) {
                violations
                    .entry((inv.command.clone(), f.clone()))
                    .or_default()
                    .push(format!("{}:{}", inv.doc_file, inv.doc_line));
            }
        }
    }

    assert!(
        violations.is_empty(),
        "documented flags the `horus` binary rejects (source: {source}):\n{}\n\n\
         A user copying these lines gets `error: unexpected argument`. Either restore \
         the flag or update the doc page.",
        violations
            .iter()
            .map(|((c, f), where_)| format!(
                "  horus {c} {f}  — documented at {}",
                where_.join(", ")
            ))
            .collect::<Vec<_>>()
            .join("\n")
    );
}

/// A quarantined flag that starts working must leave the quarantine list.
///
/// Without this, [`QUARANTINED_FLAGS`] would accumulate entries forever and
/// quietly suppress real regressions: if `horus test --no-cleanup` is restored
/// and later removed again, a stale entry would hide the second break.
#[test]
fn quarantine_is_not_stale() {
    let mut fixed = Vec::new();
    for (command, flag, _why) in QUARANTINED_FLAGS {
        if let Some(universe) = flag_universe(command) {
            if universe.contains(*flag) {
                fixed.push(format!("  horus {command} {flag}"));
            }
        }
    }
    for (command, _why) in QUARANTINED_COMMANDS {
        if help_for(&[command]).is_some() {
            fixed.push(format!("  horus {command}"));
        }
    }

    assert!(
        fixed.is_empty(),
        "these now exist but are still quarantined:\n{}\n\n\
         Remove them from QUARANTINED_FLAGS / QUARANTINED_COMMANDS in {} so \
         future regressions are caught.",
        fixed.join("\n"),
        file!()
    );

    // The other way an entry dies: the docs stop citing it. These defects are
    // usually fixed on the docs side — the flag never starts existing, the page
    // simply stops recommending it — so the "now exists" check above would never
    // retire the entry and the list would accumulate dead weight, each one a
    // standing exemption for a command/flag pair nobody mentions.
    let (invocations, source) = load_contract();
    let mut unused = Vec::new();
    for (command, flag, _why) in QUARANTINED_FLAGS {
        let cited = invocations
            .iter()
            .any(|i| i.command == *command && i.flags.iter().any(|f| f == flag));
        if !cited {
            unused.push(format!("  horus {command} {flag}"));
        }
    }
    for (command, _why) in QUARANTINED_COMMANDS {
        if !invocations.iter().any(|i| i.command == *command) {
            unused.push(format!("  horus {command}"));
        }
    }

    assert!(
        unused.is_empty(),
        "these are quarantined but no longer documented anywhere (source: {source}):\n{}\n\n\
         The doc pages were fixed, so the exemption is dead weight — drop it from \
         {}. Leaving it in silently exempts the pair if a page starts using it again.",
        unused.join("\n"),
        file!()
    );
}

/// The committed snapshot must stay loadable and non-trivial, so the hermetic
/// PR run never silently degrades into checking nothing.
#[test]
fn snapshot_is_present_and_substantial() {
    let path = fixture_path();
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("missing {}: {e}", path.display()));
    let c: Contract = serde_json::from_str(&text).expect("snapshot is valid JSON");
    assert!(
        c.invocations.len() >= 100,
        "snapshot has only {} invocations — expected 100+. Regenerate it with:\n  \
         HORUS_DOCS_DIR=../horus-docs cargo test -p horus_manager --test docs_contract \
         -- --ignored regenerate_docs_cli_contract",
        c.invocations.len()
    );
}

/// Regenerate the committed snapshot from a live docs checkout.
///
/// Not a test — a maintenance task. Run it whenever the docs add commands:
/// ```bash
/// HORUS_DOCS_DIR=../horus-docs cargo test -p horus_manager --test docs_contract \
///     -- --ignored regenerate_docs_cli_contract
/// ```
#[test]
#[ignore = "maintenance task: regenerates tests/fixtures/docs-cli-contract.json"]
fn regenerate_docs_cli_contract() {
    // `#[ignore]` alone does not protect this: `cargo test -- --include-ignored`
    // is the natural way to run the whole suite, and it fired this task during a
    // verification run, silently rewriting the committed snapshot from an
    // unrelated docs checkout. A destructive maintenance task needs an explicit
    // opt-in, not merely an opt-out.
    if std::env::var("HORUS_DOCS_REGENERATE").is_err() {
        eprintln!(
            "skipping: regenerating the snapshot overwrites committed state.\n  \
             Re-run with HORUS_DOCS_REGENERATE=1 to do it deliberately:\n    \
             HORUS_DOCS_REGENERATE=1 HORUS_DOCS_DIR=../horus-docs cargo test \\\n      \
             -p horus_manager --test docs_contract -- --ignored regenerate_docs_cli_contract"
        );
        return;
    }
    let docs = docs_dir().expect(
        "set HORUS_DOCS_DIR=/path/to/horus-docs (or check horus-docs out beside this repo)",
    );
    let mut invocations = extract_all(&docs);
    invocations.sort();
    invocations.dedup();
    assert!(
        !invocations.is_empty(),
        "extracted nothing from {} — refusing to write an empty snapshot",
        docs.display()
    );

    let source_commit = Command::new("git")
        .args(["-C", &docs.to_string_lossy(), "rev-parse", "HEAD"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string());

    let out = fixture_path();
    std::fs::create_dir_all(out.parent().unwrap()).unwrap();
    let contract = Contract {
        source_commit,
        invocations,
    };
    std::fs::write(
        &out,
        serde_json::to_string_pretty(&contract).unwrap() + "\n",
    )
    .unwrap();
    eprintln!(
        "wrote {} ({} invocations)",
        out.display(),
        contract.invocations.len()
    );
}

// ─── Extractor unit tests ───────────────────────────────────────────────────
//
// The extractor is the foundation: if it silently stops finding commands, every
// contract test above passes while checking nothing. These pin its behavior.

#[cfg(test)]
mod extractor {
    use super::*;

    #[test]
    fn extracts_command_and_flags_from_bash_fence() {
        let md = "text\n```bash\nhorus run --release\n```\n";
        let got = extract_from_page(md, "p.mdx");
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].command, "run");
        assert_eq!(got[0].flags, vec!["--release"]);
    }

    #[test]
    fn flag_values_are_not_subcommands() {
        // `20` is the value of --last, not a subcommand. Reading it as one
        // reported development/cli-reference.mdx's correct `horus bb
        // --anomalies --last 20` as a defect.
        let md = "```bash\nhorus bb --anomalies --last 20\n```\n";
        assert_eq!(extract_from_page(md, "p.mdx")[0].sub, None);
        let md2 = "```bash\nhorus bb --node controller --tick 4500-4510\n```\n";
        assert_eq!(extract_from_page(md2, "p.mdx")[0].sub, None);
        // A real subcommand still lands, flags after it notwithstanding.
        let md3 = "```bash\nhorus topic list --json\n```\n";
        assert_eq!(
            extract_from_page(md3, "p.mdx")[0].sub.as_deref(),
            Some("list")
        );
    }

    #[test]
    fn captures_nested_subcommand() {
        let md = "```bash\nhorus topic list --json\n```\n";
        let got = extract_from_page(md, "p.mdx");
        assert_eq!(got[0].command, "topic");
        assert_eq!(got[0].sub.as_deref(), Some("list"));
        assert_eq!(got[0].flags, vec!["--json"]);
    }

    #[test]
    fn ignores_non_shell_fences() {
        // A Rust sample that mentions horus must not be read as a command.
        let md = "```rust\nhorus run --release\n```\n";
        assert!(extract_from_page(md, "p.mdx").is_empty());
    }

    #[test]
    fn ignores_comments_and_prose() {
        let md = "```bash\n# horus run --release\nsome-other-tool --flag\n```\n";
        assert!(extract_from_page(md, "p.mdx").is_empty());
    }

    #[test]
    fn strips_shell_prompt() {
        let md = "```bash\n$ horus build --release\n```\n";
        let got = extract_from_page(md, "p.mdx");
        assert_eq!(got[0].command, "build");
        assert_eq!(got[0].flags, vec!["--release"]);
    }

    #[test]
    fn stops_at_double_dash_passthrough() {
        // `--sample-size` belongs to the benchmark harness, not to clap.
        let md = "```bash\nhorus bench -- --sample-size 100\n```\n";
        let got = extract_from_page(md, "p.mdx");
        assert_eq!(got[0].command, "bench");
        assert!(
            got[0].flags.is_empty(),
            "flags after `--` are forwarded and must not be attributed to horus: {:?}",
            got[0].flags
        );
    }

    #[test]
    fn normalizes_equals_form() {
        let md = "```bash\nhorus new myproj --template=rust\n```\n";
        let got = extract_from_page(md, "p.mdx");
        assert_eq!(got[0].flags, vec!["--template"]);
    }

    #[test]
    fn skips_piped_and_chained_lines() {
        let md = "```bash\nhorus topic list | grep x\nhorus build && horus run\n```\n";
        assert!(extract_from_page(md, "p.mdx").is_empty());
    }

    #[test]
    fn records_doc_location() {
        let md = "a\nb\n```bash\nhorus doctor\n```\n";
        let got = extract_from_page(md, "guide.mdx");
        assert_eq!(got[0].doc_file, "guide.mdx");
        assert_eq!(got[0].doc_line, 4);
    }

    #[test]
    fn children_parses_clap_commands_block() {
        let help = "Usage: horus topic <COMMAND>\n\nCommands:\n  list  List all\n  echo  Echo it\n\nOptions:\n  -h, --help  Print help\n";
        assert_eq!(children_in(help), vec!["echo", "list"]);
    }

    #[test]
    fn children_parses_categorized_top_level_template() {
        // The hand-written top-level template groups by category rather than
        // using a single `Commands:` block.
        let help = "\
horus 0.2.2 — framework

Usage: horus [OPTIONS] <COMMAND>

Project:
  init              Initialize workspace
  launch, l         Launch nodes from YAML

Plugins:
  plugin            Manage plugins

Options:
  -h, --help        Print help

Quick Start:
  horus new my_robot -r           Create a new Rust project
  cd my_robot && horus run        Build and run it
";
        let got = children_in(help);
        assert_eq!(got, vec!["init", "launch", "plugin"], "got {got:?}");
    }

    #[test]
    fn children_ignores_example_and_option_blocks() {
        // Regression: `Quick Start:` lines begin with `horus`, and `Options:`
        // lines begin with `-`; neither is a subcommand name.
        let help =
            "Options:\n  -r, --release  Build release\n\nQuick Start:\n  horus run  Run it\n";
        assert!(children_in(help).is_empty(), "{:?}", children_in(help));
    }

    #[test]
    fn flags_in_finds_long_flags() {
        let help = "Options:\n  -r, --release  Build release\n      --json     As JSON\n";
        let f = flags_in(help);
        assert!(f.contains("--release"), "{f:?}");
        assert!(f.contains("--json"), "{f:?}");
    }
}
