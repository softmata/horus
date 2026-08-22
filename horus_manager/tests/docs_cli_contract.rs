//! Every CLI invocation the documentation teaches must actually work.
//!
//! `.github/workflows/docs-contract.yml` ran two steps named "CLI contract
//! against the committed snapshot" and "CLI contract against live docs", both
//! invoking `--test docs_contract` — whose four tests all read
//! `configuration.mdx` and assert manifest tables. Nothing in that binary
//! touched the CLI. The workflow advertised coverage that did not exist, which
//! is worse than having none: it is why the CLI reference was free to drift.
//!
//! `tests/fixtures/docs-cli-contract.json` holds 657 invocations someone
//! extracted for exactly this test, and nothing read it. It is not used here
//! either, for two reasons: it is a snapshot of an old commit
//! (`source_commit: 1739e51c`), so it cannot describe today's docs; and its
//! extractor mis-parses multi-word commands — it recorded
//! `horus plugin disable sim3d` as `{command: "disable", sub: "sim3d"}` while
//! also recording the same command correctly from another page. Validating
//! against it produces false failures on documentation that is right.
//!
//! So this reads the docs directly and asks clap. Neither side can go stale.
//!
//! Run: `cargo test -p horus_manager --test docs_cli_contract`

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Command;

/// A `horus ...` invocation found in the documentation.
#[derive(Debug, Clone)]
struct Invocation {
    command: String,
    sub: Option<String>,
    flags: Vec<String>,
    doc_file: String,
    doc_line: usize,
}

impl Invocation {
    fn rendered(&self) -> String {
        let mut s = format!("horus {}", self.command);
        if let Some(ref sub) = self.sub {
            s.push(' ');
            s.push_str(sub);
        }
        for f in &self.flags {
            s.push(' ');
            s.push_str(f);
        }
        s
    }

    fn site(&self) -> String {
        format!("{}:{}", self.doc_file, self.doc_line)
    }

    /// Pages under `plugins/` legitimately teach commands that only exist once
    /// a plugin is installed (`horus sim3d`, `horus topic-stats`). Those are not
    /// drift, and this test has no plugins installed.
    fn is_plugin_page(&self) -> bool {
        self.doc_file.contains("plugins/")
    }
}

fn horus() -> &'static str {
    env!("CARGO_BIN_EXE_horus")
}

fn docs_root() -> Option<PathBuf> {
    // Honour HORUS_DOCS_DIR the way docs_examples.rs and friends already do, so
    // CI can point at a checkout it placed somewhere else. Without this the
    // "live docs" CI job silently skipped.
    if let Ok(dir) = std::env::var("HORUS_DOCS_DIR") {
        let p = PathBuf::from(dir);
        if p.is_dir() {
            return Some(p);
        }
    }
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()?
        .parent()?
        .join("horus-docs/content/docs");
    root.is_dir().then_some(root)
}

/// A word that stands for a value the reader supplies, not a literal.
fn is_placeholder(word: &str) -> bool {
    word.starts_with('<')
        || word.starts_with('[')
        || word.starts_with('$')
        || word.starts_with('{')
        || word.contains("...")
        || word.chars().next().is_some_and(|c| c.is_ascii_uppercase())
}

/// Pull every `horus <...>` invocation out of the documentation.
fn extract() -> Vec<Invocation> {
    let Some(root) = docs_root() else {
        return Vec::new();
    };

    let mut out = Vec::new();
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
            if !path.extension().is_some_and(|e| e == "mdx" || e == "md") {
                continue;
            }
            let Ok(text) = std::fs::read_to_string(&path) else {
                continue;
            };
            let rel = path
                .strip_prefix(&root)
                .unwrap_or(&path)
                .to_string_lossy()
                .to_string();

            let mut in_shell_fence = false;
            for (i, line) in text.lines().enumerate() {
                let t = line.trim();
                if t.starts_with("```") {
                    // Only shell fences contain runnable invocations; a Rust
                    // fence mentioning "horus" is not a command.
                    in_shell_fence = t.starts_with("```bash")
                        || t.starts_with("```sh")
                        || t.starts_with("```shell")
                        || t.starts_with("```console");
                    continue;
                }
                if !in_shell_fence {
                    continue;
                }
                let Some(rest) = t.strip_prefix("horus ") else {
                    continue;
                };
                // Stop at a comment or a pipe: `horus run  # comment`.
                let rest = rest.split('#').next().unwrap_or(rest);
                let rest = rest.split('|').next().unwrap_or(rest);
                let words: Vec<&str> = rest.split_whitespace().collect();
                if words.is_empty() {
                    continue;
                }
                let command = words[0];
                if is_placeholder(command) || command.starts_with('-') {
                    continue;
                }
                let sub = words
                    .get(1)
                    .filter(|w| !w.starts_with('-') && !is_placeholder(w))
                    .map(|w| w.to_string());
                // Everything after a bare `--` is forwarded to the underlying
                // tool (`horus fmt -- --edition 2021`), so those are not this
                // command's flags.
                let own: Vec<&str> = words.iter().copied().take_while(|w| *w != "--").collect();
                let flags: Vec<String> = own
                    .iter()
                    .filter(|w| w.starts_with("--"))
                    .map(|w| w.trim_end_matches(['.', ',', ')']).to_string())
                    .collect();

                out.push(Invocation {
                    command: command.to_string(),
                    sub,
                    flags,
                    doc_file: rel.clone(),
                    doc_line: i + 1,
                });
            }
        }
    }
    out
}

/// `horus <args> --help`: the rendered text, and whether clap accepted it.
///
/// Resolution is decided by the exit status, not by grepping the output. An
/// earlier version searched for "unexpected argument" and flagged
/// `horus param set` as missing — because that command's own help text
/// documents that `horus param set min_angle -1.57` used to fail with
/// `unexpected argument '-1' found`. Matching prose inside help output is not a
/// reliable way to ask whether a command exists.
fn help_for(args: &[&str]) -> (bool, String) {
    let mut all: Vec<&str> = args.to_vec();
    all.push("--help");
    let out = Command::new(horus())
        .args(&all)
        .output()
        .expect("horus must run");
    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    (out.status.success(), text)
}

/// Commands that forward everything after their name to a plugin or an
/// underlying tool. Their `[ARGS]...` accepts any flag, so a documented flag
/// that is not in their own `--help` is not drift.
fn is_passthrough(help: &str) -> bool {
    help.contains("[ARGS]...") || help.contains("EXTRA_ARGS")
}

/// The extraction must find something, or every test below passes vacuously.
#[test]
fn the_docs_contain_cli_invocations() {
    if docs_root().is_none() {
        eprintln!("SKIP: horus-docs is not checked out and HORUS_DOCS_DIR is unset");
        return;
    }
    let found = extract();
    assert!(
        found.len() > 100,
        "expected hundreds of documented `horus ...` invocations, found {} —          the extractor is probably broken, which would make the tests below          pass while checking nothing",
        found.len()
    );
}

/// Every top-level command the docs use must exist.
#[test]
fn every_documented_command_resolves() {
    if docs_root().is_none() {
        eprintln!("SKIP: horus-docs is not checked out and HORUS_DOCS_DIR is unset");
        return;
    }

    // One --help per distinct command, not per invocation.
    let mut commands: BTreeMap<String, Invocation> = BTreeMap::new();
    for inv in extract() {
        if inv.is_plugin_page() {
            continue;
        }
        commands.entry(inv.command.clone()).or_insert(inv);
    }

    let mut broken = Vec::new();
    for (name, example) in &commands {
        let (ok, _) = help_for(&[name]);
        if !ok {
            broken.push(format!("`{}` at {}", example.rendered(), example.site()));
        }
    }

    assert!(
        broken.is_empty(),
        "the documentation teaches {} command(s) HORUS does not have:\n  {}",
        broken.len(),
        broken.join("\n  ")
    );
}

/// Every documented subcommand must exist under its parent.
#[test]
fn every_documented_subcommand_resolves() {
    if docs_root().is_none() {
        eprintln!("SKIP: horus-docs is not checked out and HORUS_DOCS_DIR is unset");
        return;
    }

    let mut pairs: BTreeMap<(String, String), Invocation> = BTreeMap::new();
    for inv in extract() {
        if inv.is_plugin_page() {
            continue;
        }
        if let Some(ref sub) = inv.sub {
            pairs
                .entry((inv.command.clone(), sub.clone()))
                .or_insert(inv.clone());
        }
    }

    let mut broken = Vec::new();
    for ((cmd, sub), example) in &pairs {
        let (parent_ok, parent) = help_for(&[cmd]);
        if !parent_ok || !parent.contains("Commands:") {
            // Not a subcommand-taking command: the second word is a value.
            continue;
        }
        let (sub_ok, _) = help_for(&[cmd, sub]);
        if !sub_ok {
            broken.push(format!("`{}` at {}", example.rendered(), example.site()));
        }
    }

    assert!(
        broken.is_empty(),
        "the documentation teaches {} subcommand(s) that do not exist:\n  {}",
        broken.len(),
        broken.join("\n  ")
    );
}

/// Every documented long flag must be accepted where it is shown.
#[test]
fn every_documented_flag_is_accepted() {
    if docs_root().is_none() {
        eprintln!("SKIP: horus-docs is not checked out and HORUS_DOCS_DIR is unset");
        return;
    }

    let mut checked: BTreeMap<(String, String), Invocation> = BTreeMap::new();
    for inv in extract() {
        if inv.is_plugin_page() {
            continue;
        }
        for flag in &inv.flags {
            checked
                .entry((inv.command.clone(), flag.clone()))
                .or_insert(inv.clone());
        }
    }

    let mut broken = Vec::new();
    for ((cmd, flag), example) in &checked {
        let (parent_ok, parent) = help_for(&[cmd]);
        if !parent_ok {
            continue; // reported by every_documented_command_resolves
        }
        if is_passthrough(&parent) {
            continue; // the flag is the wrapped tool's, not ours
        }
        let in_parent = parent.contains(flag.as_str());
        let in_child = example
            .sub
            .as_deref()
            .map(|sub| {
                let (ok, h) = help_for(&[cmd, sub]);
                ok && h.contains(flag.as_str())
            })
            .unwrap_or(false);

        if !in_parent && !in_child {
            broken.push(format!(
                "`{}` — {flag} is not accepted, at {}",
                example.rendered(),
                example.site()
            ));
        }
    }

    assert!(
        broken.is_empty(),
        "the documentation teaches {} flag(s) the CLI does not accept:\n  {}",
        broken.len(),
        broken
            .iter()
            .take(25)
            .cloned()
            .collect::<Vec<_>>()
            .join("\n  ")
    );
}
