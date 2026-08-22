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

// ── The other direction: CLI → docs ─────────────────────────────────────
//
// Everything above asks whether the documentation describes a CLI that exists.
// Nothing asked the reverse, and the reverse is where the reference rotted:
// `horus add`, `horus lock`, `horus list`, `horus info`, `horus search`,
// `horus update`, `horus publish`, `horus unpublish`, `horus scripts`,
// `horus self update` and `horus man` all worked, and none of them had a
// section in cli-reference.mdx — `grep -rn "horus self" content/` returned
// nothing across the entire corpus. A command with no section is a command
// nobody finds, and `horus --help` is one screen, not a reference.

/// The reference page, if the docs are available.
fn cli_reference() -> Option<String> {
    let path = docs_root()?.join("development/cli-reference.mdx");
    std::fs::read_to_string(path).ok()
}

/// Top-level command names clap offers, read out of the generated bash
/// completion script — the only machine-readable view of the command tree an
/// integration test has. Hidden commands are already excluded from it, so this
/// is exactly the set a user can discover.
fn visible_command_names() -> Vec<String> {
    let out = Command::new(horus())
        .args(["completion", "bash"])
        .output()
        .expect("horus completion bash must run");
    let script = String::from_utf8_lossy(&out.stdout);

    let mut lines = script.lines();
    while let Some(line) = lines.next() {
        if line.trim() != "horus)" {
            continue;
        }
        let Some(opts) = lines.next() else { break };
        let Some((_, rest)) = opts.split_once("opts=\"") else {
            break;
        };
        let Some((inner, _)) = rest.split_once('"') else {
            break;
        };
        return inner
            .split_whitespace()
            .filter(|t| !t.starts_with('-'))
            .map(|t| t.to_string())
            .collect();
    }
    Vec::new()
}

/// The canonical name of a command, resolved by clap itself: `horus t --help`
/// renders `Usage: horus topic ...`, so an alias reports the command it is an
/// alias of.
fn canonical_name(name: &str) -> Option<String> {
    let (ok, text) = help_for(&[name]);
    if !ok {
        return None;
    }
    text.lines()
        .find_map(|l| l.strip_prefix("Usage: horus "))
        .and_then(|rest| rest.split_whitespace().next())
        .map(|s| s.to_string())
}

/// The heading and body of the section documenting `horus <name>`, if any.
///
/// A section is a heading whose title contains `` `horus <name>` `` — either a
/// page-level `##` or a `###` inside a grouping section, since the registry
/// lifecycle commands are documented as subsections of one page section.
fn section_for<'a>(doc: &'a str, name: &str) -> Option<Vec<&'a str>> {
    let needle_end = format!("`horus {name}`");
    let needle_more = format!("`horus {name} ");
    let lines: Vec<&str> = doc.lines().collect();
    let start = lines.iter().position(|l| {
        l.starts_with('#')
            && (l.contains(needle_end.as_str()) || l.contains(needle_more.as_str()))
    })?;
    let depth = lines[start].chars().take_while(|c| *c == '#').count();
    let mut end = lines.len();
    // Shell comments inside a fenced block start with `#` too, and reading one
    // as a heading truncates the section at its first example.
    let mut fenced = false;
    for (i, line) in lines.iter().enumerate().skip(start + 1) {
        if line.trim_start().starts_with("```") {
            fenced = !fenced;
            continue;
        }
        if fenced {
            continue;
        }
        let d = line.chars().take_while(|c| *c == '#').count();
        if d > 0 && d <= depth && line.chars().nth(d) == Some(' ') {
            end = i;
            break;
        }
    }
    Some(lines[start..end].to_vec())
}

/// Every command that exists must have a section in the CLI reference.
#[test]
fn every_visible_command_has_a_section_in_the_cli_reference() {
    let Some(doc) = cli_reference() else {
        eprintln!("SKIP: horus-docs is not checked out and HORUS_DOCS_DIR is unset");
        return;
    };
    let names = visible_command_names();
    assert!(
        names.len() > 40,
        "read only {} command names out of the completion script — the \
         generator's output format changed and this test is checking nothing: \
         {names:?}",
        names.len()
    );

    let mut undocumented = Vec::new();
    for name in &names {
        let Some(canonical) = canonical_name(name) else {
            continue; // reported by every_documented_command_resolves
        };
        if &canonical != name {
            continue; // an alias; checked below
        }
        if section_for(&doc, name).is_none() {
            undocumented.push(name.clone());
        }
    }

    assert!(
        undocumented.is_empty(),
        "these commands work but have no section in \
         content/docs/development/cli-reference.mdx, so the only place they are \
         described is one line of `horus --help`: {undocumented:?}\n\
         Add a `## `horus <name>`` section, or a `### `horus <name>`` \
         subsection under the group it belongs to."
    );
}

/// Every alias must be named in the section of the command it aliases.
///
/// `tf`, `srv` and `plugins` appeared nowhere in the reference. `tf` is the
/// word a ROS 2 migrant types first, and `horus plugins` is what the plugin
/// pages themselves use — an alias documented nowhere reads as a typo when a
/// colleague uses it.
#[test]
fn every_alias_is_named_in_its_commands_section() {
    let Some(doc) = cli_reference() else {
        eprintln!("SKIP: horus-docs is not checked out and HORUS_DOCS_DIR is unset");
        return;
    };
    let names = visible_command_names();
    assert!(names.len() > 40, "completion script not parsed: {names:?}");

    let mut aliases = 0usize;
    let mut missing = Vec::new();
    for name in &names {
        let Some(canonical) = canonical_name(name) else {
            continue;
        };
        if &canonical == name {
            continue;
        }
        aliases += 1;
        let Some(section) = section_for(&doc, &canonical) else {
            continue; // reported by the test above
        };
        let mention = format!("horus {name}`");
        if !section.iter().any(|l| l.contains(mention.as_str())) {
            missing.push(format!("{name} (alias of {canonical})"));
        }
    }

    assert!(
        aliases >= 10,
        "found only {aliases} aliases — the resolution through the Usage line \
         is broken and this test is checking nothing"
    );
    assert!(
        missing.is_empty(),
        "these aliases work but the section of the command they alias never \
         mentions them: {missing:?}\n\
         Add `**Alias**: `horus <alias>`` under that command's heading in \
         content/docs/development/cli-reference.mdx."
    );
}

/// The CLI reference must not re-teach the "alias for horus doctor" framing.
///
/// `--health` was described in the CLI as "alias for horus doctor" and nowhere
/// said what either command covers, which is how three entry points to
/// overlapping validation came to have no stated distinction. The CLI help was
/// fixed and guarded (`check_and_doctor_state_what_each_covers` in
/// help_contract.rs); the reference page still carried a pasted copy of the old
/// help text, so the sentence the fix removed was still the one users read.
#[test]
fn the_reference_explains_check_versus_doctor() {
    let Some(doc) = cli_reference() else {
        eprintln!("SKIP: horus-docs is not checked out and HORUS_DOCS_DIR is unset");
        return;
    };
    assert!(
        !doc.contains("alias for horus doctor"),
        "content/docs/development/cli-reference.mdx still describes --health as \
         an alias, the framing that left the check/doctor distinction unstated"
    );

    let section = section_for(&doc, "check").expect("`horus check` must have a section");
    let text = section.join("\n");
    assert!(
        text.contains("horus doctor") && text.contains("machine"),
        "the `horus check` section must say what `horus doctor` covers \
         instead:\n{text}"
    );
}
