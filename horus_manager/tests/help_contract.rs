//! The top-level `--help` text must describe the CLI that actually exists.
//!
//! `horus`'s help is a hand-written `help_template` string in `main.rs`, while
//! the real command set is a clap enum. Nothing checked the two against each
//! other, and they drifted in both directions:
//!
//! * `enable`, `disable` and `verify` were advertised as top-level commands.
//!   All three errored — they are subcommands of `plugin`, which the template
//!   did not mention at all. The did-you-mean was no help either:
//!   `horus enable` suggested `'n'` and `'a'`.
//! * `frame`'s alias was listed as `frames`, which errors. The real alias is
//!   `tf` — the exact word a ROS2 migrant tries first, so the one name that
//!   works was the one the help said did not exist.
//! * `completion`, `scripts` and `setup-rt` all worked and appeared nowhere.
//!   `horus doctor` actively tells users to run `setup-rt`.
//!
//! Keeping the curated grouping (Project / Introspection / Debugging / ...) is
//! worth the hand-written template — a naive render of the parsed tree would
//! flatten forty commands into one alphabetised list. So the template stays and
//! this test guards it, rather than generating it and losing the curation.
//!
//! Run: `cargo test -p horus_manager --test help_contract`

use std::collections::BTreeSet;
use std::process::Command;

/// Names listed in the hand-written help template.
///
/// Parsed from the rendered `--help` output rather than the source string, so
/// this measures what a user actually sees. Stops at the options block, which
/// is clap's own and not part of the curated list.
fn documented_commands(help: &str) -> BTreeSet<String> {
    let mut names = BTreeSet::new();

    for line in help.lines() {
        // Command rows are indented exactly two spaces: `  name, alias  desc`.
        let Some(rest) = line.strip_prefix("  ") else {
            continue;
        };
        if rest.starts_with(' ') || rest.starts_with('-') {
            continue; // continuation, or an option row
        }
        let Some(first) = rest.split_whitespace().next() else {
            continue;
        };
        let name = first.trim_end_matches(',');
        // The after-help "Quick Start" block also uses two-space indentation;
        // its rows begin with the binary name or a shell builtin.
        if name == "horus" || name == "cd" {
            continue;
        }
        if !name.is_empty() && name.chars().all(|c| c.is_ascii_lowercase() || c == '-') {
            names.insert(name.to_string());
        }
    }
    names
}

/// Aliases the template claims, as `(command, alias)` pairs.
fn documented_aliases(help: &str) -> Vec<(String, String)> {
    let mut pairs = Vec::new();
    for line in help.lines() {
        let Some(rest) = line.strip_prefix("  ") else {
            continue;
        };
        if rest.starts_with(' ') || rest.starts_with('-') {
            continue;
        }
        let head: Vec<&str> = rest.split_whitespace().take(2).collect();
        if head.len() == 2 && head[0].ends_with(',') {
            let name = head[0].trim_end_matches(',');
            let alias = head[1];
            if alias.chars().all(|c| c.is_ascii_lowercase() || c == '-') {
                pairs.push((name.to_string(), alias.to_string()));
            }
        }
    }
    pairs
}

fn horus() -> &'static str {
    env!("CARGO_BIN_EXE_horus")
}

fn help_text() -> String {
    let out = Command::new(horus())
        .arg("--help")
        .output()
        .expect("horus --help must run");
    String::from_utf8_lossy(&out.stdout).into_owned()
}

/// `horus <path...> --help` resolves, i.e. clap recognises the whole path.
///
/// Asserting on success plus a rendered `Usage:` line rather than on the
/// absence of the string "unrecognized subcommand": clap words its errors
/// differently depending on how a name fails (an ambiguous prefix, a value
/// where a subcommand was expected, a required argument), and grepping for one
/// phrase makes every other failure read as success.
fn resolves_path(path: &[&str]) -> bool {
    let mut args: Vec<&str> = path.to_vec();
    args.push("--help");
    let out = Command::new(horus())
        .args(&args)
        .output()
        .expect("horus <cmd> --help must run");
    out.status.success() && String::from_utf8_lossy(&out.stdout).contains("Usage:")
}

fn resolves(name: &str) -> bool {
    resolves_path(&[name])
}

/// The first line of `horus <cmd> --help` — the command's own `about`.
fn about_line(name: &str) -> String {
    let out = Command::new(horus())
        .args([name, "--help"])
        .output()
        .expect("horus <cmd> --help must run");
    String::from_utf8_lossy(&out.stdout)
        .lines()
        .find(|l| !l.trim().is_empty())
        .unwrap_or_default()
        .to_string()
}

/// Whether a command has subcommands of its own.
fn has_subcommands(name: &str) -> bool {
    let out = Command::new(horus())
        .args([name, "--help"])
        .output()
        .expect("horus <cmd> --help must run");
    String::from_utf8_lossy(&out.stdout).contains("\nCommands:")
}

/// The `(a, b, c)` hint at the end of a description, when every entry is
/// shaped like a command name.
///
/// Descriptions use parentheses for other things — `(crates.io, PyPI, system,
/// registry)`, `(bash/zsh/fish)`, `(name@version syntax)` — so anything
/// carrying a dot, a slash, a space or a capital is not a subcommand list and
/// yields `None` rather than a pile of false failures.
fn parenthetical(text: &str) -> Option<Vec<String>> {
    let open = text.rfind('(')?;
    let close = text[open..].find(')')? + open;
    let inner = &text[open + 1..close];
    let entries: Vec<String> = inner.split(',').map(|e| e.trim().to_string()).collect();
    if entries.len() < 2 {
        return None;
    }
    if entries.iter().any(|e| {
        e.is_empty()
            || !e
                .chars()
                .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
    }) {
        return None;
    }
    Some(entries)
}

/// Command rows of the template that carry a subcommand hint, as
/// `(parent, [child, ...])`.
fn documented_subcommand_hints(help: &str) -> Vec<(String, Vec<String>)> {
    let mut hints = Vec::new();
    for line in help.lines() {
        let Some(rest) = line.strip_prefix("  ") else {
            continue;
        };
        if rest.starts_with(' ') || rest.starts_with('-') {
            continue;
        }
        let Some(first) = rest.split_whitespace().next() else {
            continue;
        };
        let parent = first.trim_end_matches(',');
        if parent == "horus" || parent == "cd" || parent.is_empty() {
            continue;
        }
        if let Some(children) = parenthetical(rest) {
            hints.push((parent.to_string(), children));
        }
    }
    hints
}

#[test]
fn every_documented_command_exists() {
    let help = help_text();
    let documented = documented_commands(&help);
    assert!(
        documented.len() > 20,
        "parsed too few command rows ({}) — the template format likely changed \
         and this test is no longer reading it",
        documented.len()
    );

    let phantom: Vec<&String> = documented.iter().filter(|n| !resolves(n)).collect();
    assert!(
        phantom.is_empty(),
        "`horus --help` advertises commands that do not exist: {phantom:?}\n\
         Either add them to the clap enum in main.rs, or remove them from the \
         help_template. A user who types one of these gets \
         `error: unrecognized subcommand`."
    );
}

#[test]
fn every_documented_alias_resolves() {
    let help = help_text();
    let broken: Vec<(String, String)> = documented_aliases(&help)
        .into_iter()
        .filter(|(_, alias)| !resolves(alias))
        .collect();

    assert!(
        broken.is_empty(),
        "`horus --help` claims aliases that do not resolve: {broken:?}\n\
         Check `visible_alias` on the command in main.rs. `frame` claimed \
         `frames` for a long time while the real alias was `tf`."
    );
}

/// Commands a user can run must be discoverable.
///
/// Deliberately a fixed list rather than "every visible clap subcommand": the
/// tool proxies (`cargo`, `pip`, `cmake`) are shell-integration internals and
/// are correctly hidden. These four are not internals — `doctor` even tells
/// users to run `setup-rt`.
#[test]
fn user_facing_commands_are_documented() {
    let help = help_text();
    let documented = documented_commands(&help);

    for name in ["completion", "plugin", "scripts", "setup-rt"] {
        assert!(
            resolves(name),
            "{name} should exist — this test needs updating if it was removed"
        );
        assert!(
            documented.contains(name),
            "`{name}` works but is absent from `horus --help`, so nobody can \
             find it. Documented: {documented:?}"
        );
    }
}

/// The completion generator must be reachable and produce a real script.
///
/// `clap_complete` has been a dependency and the command has worked all along,
/// but it was `hide = true` and its own code comment claimed `install.sh` set
/// completions up automatically — `grep -c completion install.sh` returns 0.
/// So for a CLI with ~40 commands and single-letter aliases (`t n p a m s i l
/// srv bb mon rec tf`), nobody has ever had completion.
#[test]
fn completion_generates_a_usable_script() {
    for shell in ["bash", "zsh", "fish"] {
        let out = Command::new(horus())
            .args(["completion", shell])
            .output()
            .unwrap_or_else(|e| panic!("horus completion {shell} must run: {e}"));

        assert!(
            out.status.success(),
            "horus completion {shell} failed: {}",
            String::from_utf8_lossy(&out.stderr)
        );
        let script = String::from_utf8_lossy(&out.stdout);
        assert!(
            script.len() > 200,
            "completion script for {shell} looks empty ({} bytes)",
            script.len()
        );
        assert!(
            script.contains("horus"),
            "completion script for {shell} does not mention the binary"
        );
    }
}

/// Every alias declared in the source must appear in the hand-written help
/// template.
///
/// `--help` is a hardcoded `help_template` string, not generated from clap's
/// command tree, so the two drift silently. `plugins` and `script` were both
/// live `visible_alias`es that the template never mentioned, which is how
/// `horus plugins --help` came to look like a bug: the user typed a name the
/// template does not list, and clap correctly echoed the canonical `plugin` in
/// the usage line.
#[test]
fn every_visible_alias_appears_in_the_help_template() {
    let main_rs = std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/main.rs"),
    )
    .expect("main.rs must be readable");

    // The template is the long string literal at the top of the file; the
    // aliases are declared further down in the Commands enum.
    let aliases: Vec<String> = main_rs
        .lines()
        .filter_map(|l| l.split_once("visible_alias = \""))
        .filter_map(|(_, rest)| rest.split('"').next())
        .map(|s| s.to_string())
        .collect();

    assert!(
        aliases.len() >= 8,
        "expected to find the visible_alias declarations, got {aliases:?}"
    );

    let help = String::from_utf8(
        std::process::Command::new(env!("CARGO_BIN_EXE_horus"))
            .arg("--help")
            .output()
            .expect("horus --help must run")
            .stdout,
    )
    .expect("help output must be utf-8");

    let missing: Vec<&String> = aliases
        .iter()
        .filter(|a| !help.contains(a.as_str()))
        .collect();

    assert!(
        missing.is_empty(),
        "these aliases work but are absent from `horus --help`, so a user has no \
         way to discover them and no way to tell they are aliases: {missing:?}"
    );
}

/// A proxy must not answer for the tool it proxies.
///
/// The five native-tool proxies declare `trailing_var_arg` and
/// `allow_hyphen_values`, but clap still claimed `--help`/`-h` and
/// `--version`/`-V` for itself. Inside a HORUS project — where the shell
/// functions from `horus env --init` delegate — `cargo --version` printed
///
///     horus-cargo 0.2.2
///
/// instead of `cargo 1.97.1`. Anything parsing that output acts on a wrong
/// answer, and the user has no indication a proxy is involved.
#[test]
fn native_tool_proxies_do_not_intercept_version_or_help() {
    let main_rs = std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/main.rs"),
    )
    .expect("main.rs must be readable");

    let mut missing = Vec::new();
    for tool in ["cargo", "pip", "cmake", "conan", "vcpkg"] {
        let marker = format!("name = \"{tool}\"");
        let Some(at) = main_rs.find(&marker) else {
            missing.push(format!("{tool}: proxy declaration not found"));
            continue;
        };
        // The attribute block for this variant. Taken by lines rather than by
        // byte offset: the file contains multi-byte characters and slicing into
        // one panics.
        let block: String = main_rs[at..].lines().take(8).collect::<Vec<_>>().join("\n");
        let block = block.as_str();
        if !block.contains("disable_version_flag") {
            missing.push(format!("{tool}: --version is intercepted by clap"));
        }
        if !block.contains("disable_help_flag") {
            missing.push(format!("{tool}: --help is intercepted by clap"));
        }
    }

    assert!(
        missing.is_empty(),
        "these proxies answer for the tool they proxy:\n  {}",
        missing.join("\n  ")
    );
}

/// `check` and `doctor` must each say what they cover.
///
/// Three entry points led to overlapping validation — `horus check`,
/// `horus check --health` (described only as "alias for horus doctor"), and
/// `horus doctor` — with nothing stating the distinction. It is not a
/// preference: one reads the repository and the other reads the machine, and a
/// user picking the wrong one gets a clean result about something they were not
/// asking about.
#[test]
fn check_and_doctor_state_what_each_covers() {
    let check = String::from_utf8(
        std::process::Command::new(env!("CARGO_BIN_EXE_horus"))
            .args(["check", "--help"])
            .output()
            .expect("horus check --help")
            .stdout,
    )
    .expect("utf-8");
    let doctor = String::from_utf8(
        std::process::Command::new(env!("CARGO_BIN_EXE_horus"))
            .args(["doctor", "--help"])
            .output()
            .expect("horus doctor --help")
            .stdout,
    )
    .expect("utf-8");

    assert!(
        check.contains("doctor"),
        "`horus check --help` should say what the other one does:\n{check}"
    );
    assert!(
        doctor.contains("check"),
        "`horus doctor --help` should say what the other one does:\n{doctor}"
    );
    assert!(
        !check.contains("alias for horus doctor"),
        "describing --health only as an alias is what left the distinction \
         unstated:\n{check}"
    );
}

/// `-n` must mean `--dry-run` wherever a dry run exists.
///
/// Short flags carry different meanings across subcommands — twelve of them do,
/// which is ordinary in a CLI of this size (`git commit -a` and `git branch -a`
/// differ too). What is not ordinary is a *safety* flag that works in four
/// commands and silently does not exist in two others: `--dry-run` had no `-n`
/// on `publish` (which uploads) or `update` (which rewrites dependencies), so
/// the habit formed on `clean`/`deploy`/`launch`/`migrate` failed exactly where
/// the stakes were highest.
///
/// This does not assert the other eleven collisions away. Renaming them is a
/// breaking change to anyone's scripts and needs a deprecation cycle, not a
/// test.
#[test]
fn dry_run_is_always_available_as_n() {
    let horus = env!("CARGO_BIN_EXE_horus");
    let top = String::from_utf8(
        std::process::Command::new(horus)
            .arg("--help")
            .output()
            .expect("horus --help")
            .stdout,
    )
    .expect("utf-8");

    let mut commands: Vec<String> = Vec::new();
    for line in top.lines() {
        let Some(rest) = line.strip_prefix("  ") else {
            continue;
        };
        if rest.starts_with(' ') || rest.starts_with('-') {
            continue;
        }
        let Some(first) = rest.split_whitespace().next() else {
            continue;
        };
        let name = first.trim_end_matches(',');
        if name.chars().all(|c| c.is_ascii_lowercase() || c == '-') && !name.is_empty() {
            commands.push(name.to_string());
        }
    }
    assert!(commands.len() > 20, "parsed too few commands: {commands:?}");

    let mut missing = Vec::new();
    let mut checked = 0usize;
    for cmd in &commands {
        let out = std::process::Command::new(horus)
            .args([cmd, "--help"])
            .output()
            .expect("subcommand help must run");
        let help = String::from_utf8_lossy(&out.stdout);
        if !help.contains("--dry-run") {
            continue;
        }
        checked += 1;
        if !help.contains("-n, --dry-run") {
            missing.push(cmd.clone());
        }
    }

    assert!(
        checked >= 4,
        "found only {checked} commands with --dry-run — the scan is broken"
    );
    assert!(
        missing.is_empty(),
        "these accept --dry-run but not -n, so the habit formed elsewhere fails \
         here: {missing:?}"
    );
}

/// Every subcommand the help *names* must exist, not just every command row.
///
/// `every_documented_command_exists` reads the first token of each row and
/// `every_documented_alias_resolves` the second, so the parenthetical hints —
/// which are most of the template's factual content, nine rows of them — were
/// never checked against anything. `topic, t   Topic interaction (list, echo,
/// publish, bw)` advertised `publish` while the real subcommand is `pub`:
///
///     $ horus topic publish
///     error: unrecognized subcommand 'publish'
///       tip: a similar subcommand exists: 'pub'
///
/// That is the same defect the top of this file describes — the help naming a
/// command that errors — surviving inside the very template the other tests
/// guard.
#[test]
fn every_subcommand_named_in_the_help_exists() {
    let help = help_text();
    let hints = documented_subcommand_hints(&help);
    assert!(
        hints.len() >= 8,
        "parsed too few subcommand hints ({}) — the template format likely \
         changed and this test is no longer reading it: {hints:?}",
        hints.len()
    );

    let mut phantom = Vec::new();
    for (parent, children) in &hints {
        // A hint only claims subcommands for a command that has some; other
        // rows use parentheses to list sources, shells or syntaxes.
        if !has_subcommands(parent) {
            continue;
        }
        for child in children {
            if !resolves_path(&[parent, child]) {
                phantom.push(format!("horus {parent} {child}"));
            }
        }
    }

    assert!(
        phantom.is_empty(),
        "`horus --help` names subcommands that do not exist: {phantom:?}\n\
         Fix the parenthetical in the help_template in main.rs, or add the \
         subcommand. A user who types one of these gets \
         `error: unrecognized subcommand`."
    );
}

/// The two places that describe a command must describe the same command.
///
/// Each of these commands is summarised twice — once in the hand-written
/// top-level template, once in its own `about` — and the two are separate
/// strings hundreds of lines apart. They disagreed: the template said
/// `plugin ... (enable, disable, verify, trust)` while `horus plugin --help`
/// said `(enable, disable, verify)`, leaving `trust` — the whole plugin trust
/// model — visible from one surface and not the other.
#[test]
fn the_help_template_and_each_commands_own_about_agree() {
    let help = help_text();
    let mut disagreements = Vec::new();

    for (parent, template_children) in documented_subcommand_hints(&help) {
        if !has_subcommands(&parent) {
            continue;
        }
        let about = about_line(&parent);
        let Some(about_children) = parenthetical(&about) else {
            continue; // that surface makes no claim
        };
        if about_children != template_children {
            disagreements.push(format!(
                "{parent}: `horus --help` says {template_children:?}, \
                 `horus {parent} --help` says {about_children:?}"
            ));
        }
    }

    assert!(
        disagreements.is_empty(),
        "the two help surfaces describe the same command differently:\n  {}",
        disagreements.join("\n  ")
    );
}

/// The clap → template direction: a working command must be in the help.
///
/// Every other test here runs template → clap, so a command added to the enum
/// and left out of the hand-written template is invisible and no test notices.
/// That is exactly what happened to `man`: added as a visible, working
/// subcommand (`horus man` renders a real troff page, and install.sh installs
/// it), absent from the template, absent from the docs. It was added *after*
/// the fix that unhid `completion`, `plugin`, `scripts` and `setup-rt` — the
/// drift recurred within days, because `user_facing_commands_are_documented`
/// iterates a hardcoded list of those four names and can only ever re-check
/// them.
///
/// The command set is read from the generated completion script, which is
/// clap's own enumeration of the tree — the only machine-readable view of it
/// an integration test has.
#[test]
fn every_visible_command_appears_in_the_help_template() {
    let names = completion_candidates();
    assert!(
        names.len() > 40,
        "parsed only {} names out of the completion script — the generator's \
         output format changed and this test is no longer reading it: {names:?}",
        names.len()
    );

    let help = help_text();
    let mut documented = documented_commands(&help);
    for (_, alias) in documented_aliases(&help) {
        documented.insert(alias);
    }

    let missing: Vec<&String> = names.iter().filter(|n| !documented.contains(*n)).collect();
    assert!(
        missing.is_empty(),
        "these commands exist and are not hidden, but `horus --help` never \
         mentions them, so nobody can find them: {missing:?}\n\
         Add a row to the help_template in main.rs (and a section to \
         content/docs/development/cli-reference.mdx), or mark the command \
         `hide = true` if it is an internal."
    );
}

/// Top-level candidates offered by the generated bash completion script.
fn completion_candidates() -> BTreeSet<String> {
    let out = Command::new(horus())
        .args(["completion", "bash"])
        .output()
        .expect("horus completion bash must run");
    let script = String::from_utf8_lossy(&out.stdout);

    // The generated script dispatches on the command path; the root arm is
    // `horus)` followed by its `opts="..."` line.
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
    BTreeSet::new()
}

/// Completions must not offer what `--help` hides.
///
/// Six subcommands are `hide = true` for a reason: the five native-tool
/// proxies exist only for the shell functions `horus env --init` writes, and
/// `_is-project` is an exit code those functions branch on. `clap_complete`
/// walks `get_subcommands()`, which yields hidden commands too, so installing
/// completions handed every one of them straight back to the user at
/// `horus <TAB>` — undoing the hiding on the machines that ran install.sh.
#[test]
fn completions_do_not_offer_hidden_commands() {
    let names = completion_candidates();
    assert!(
        names.contains("run"),
        "the completion script's candidate list was not found: {names:?}"
    );

    let internals: Vec<&str> = ["cargo", "pip", "cmake", "conan", "vcpkg", "_is-project"]
        .into_iter()
        .filter(|n| names.contains(*n))
        .collect();

    assert!(
        internals.is_empty(),
        "`horus <TAB>` offers hidden internals: {internals:?}\n\
         Generate completions from the visible tree (completion_tree() in \
         main.rs), not from Cli::command()."
    );
}

// ── Short flags ─────────────────────────────────────────────────────────

/// The canonical name of a command: `horus t --help` renders
/// `Usage: horus topic ...`, so an alias reports what it is an alias of.
fn canonical_name(name: &str) -> Option<String> {
    let out = Command::new(horus())
        .args([name, "--help"])
        .output()
        .expect("horus <cmd> --help must run");
    if !out.status.success() {
        return None;
    }
    String::from_utf8_lossy(&out.stdout)
        .lines()
        .find_map(|l| l.strip_prefix("Usage: horus "))
        .and_then(|rest| rest.split_whitespace().next())
        .map(|s| s.to_string())
}

/// Every command path a user can type: each visible top-level command and each
/// of its subcommands.
fn command_paths() -> Vec<Vec<String>> {
    let mut canonical: BTreeSet<String> = BTreeSet::new();
    for name in completion_candidates() {
        if let Some(c) = canonical_name(&name) {
            canonical.insert(c);
        }
    }

    let mut paths = Vec::new();
    for cmd in canonical {
        let out = Command::new(horus())
            .args([cmd.as_str(), "--help"])
            .output()
            .expect("horus <cmd> --help must run");
        let help = String::from_utf8_lossy(&out.stdout).into_owned();
        paths.push(vec![cmd.clone()]);

        let Some(block) = help.split_once("\nCommands:\n") else {
            continue;
        };
        for line in block.1.lines() {
            if line.trim().is_empty() {
                break; // end of the Commands block
            }
            let Some(rest) = line.strip_prefix("  ") else {
                continue;
            };
            let Some(sub) = rest.split_whitespace().next() else {
                continue;
            };
            if sub
                .chars()
                .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-')
                && !sub.is_empty()
            {
                paths.push(vec![cmd.clone(), sub.to_string()]);
            }
        }
    }
    paths
}

/// `-x, --long` rows of a help text, as `(short, long)`. Skips the four flags
/// clap or the root command puts on everything.
fn short_flags(help: &str) -> Vec<(char, String)> {
    let mut out = Vec::new();
    for line in help.lines() {
        let t = line.trim_start();
        let mut chars = t.chars();
        let (Some('-'), Some(short), Some(','), Some(' '), Some('-'), Some('-')) = (
            chars.next(),
            chars.next(),
            chars.next(),
            chars.next(),
            chars.next(),
            chars.next(),
        ) else {
            continue;
        };
        if !short.is_ascii_alphabetic() {
            continue;
        }
        let long: String = chars
            .take_while(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || *c == '-')
            .collect();
        if long.is_empty() {
            continue;
        }
        if matches!(long.as_str(), "help" | "version" | "verbose" | "quiet") {
            continue;
        }
        out.push((short, long));
    }
    out
}

/// A short flag must not quietly acquire a second meaning.
///
/// Fourteen short flags already mean more than one thing, which is ordinary in
/// a CLI this size — `git commit -a` and `git branch -a` differ too. What is
/// not ordinary is a collision between commands used in the same breath:
/// `-r` selected Rust on `horus new` and release on `horus build`, `horus run`
/// and `horus test`, so the habit formed in one crossed silently into the
/// other and built a different thing without erroring. `-p` (Python vs
/// `--package`) and `-c` (C++ vs `--clean`) were the same shape.
///
/// Those three are now `short_alias` on `horus new`: still accepted, no longer
/// advertised, removed in the release named by `NEW_SHORT_FLAG_REMOVAL` in
/// main.rs — which is why `rust`, `python` and `cpp` are absent from the
/// ledger below while `horus new -r` keeps working.
///
/// The ledger is every collision that exists today. A short flag picking up a
/// meaning that is not here fails this test: that is the moment to give it a
/// different letter, while it costs nothing.
#[test]
fn no_short_flag_acquires_a_second_meaning() {
    const LEDGER: &[(char, &[&str])] = &[
        ('a', &["all", "anomalies", "arch"]),
        ('c', &["category", "clean"]),
        ('d', &["dir", "drivers", "duration"]),
        ('e', &["enable", "event"]),
        ('f', &["filter", "follow", "force", "format"]),
        ('i', &["goal-id", "identity"]),
        ('l', &["last", "level", "lib", "long"]),
        ('m', &["macro", "message"]),
        ('n', &["count", "dry-run", "limit", "name", "node", "nodes"]),
        ('p', &["package", "path", "port"]),
        ('r', &["rate", "release"]),
        ('s', &["script", "since", "source", "speed"]),
        ('t', &["target", "tick", "timeout"]),
        ('w', &["wait", "window", "workspace"]),
    ];

    let paths = command_paths();
    assert!(
        paths.len() > 100,
        "scanned only {} command paths — the enumeration is broken and this \
         test is checking nothing",
        paths.len()
    );

    // short -> long -> the command paths that spell it that way
    let mut meanings: std::collections::BTreeMap<
        char,
        std::collections::BTreeMap<String, Vec<String>>,
    > = std::collections::BTreeMap::new();
    for path in &paths {
        let args: Vec<&str> = path.iter().map(|s| s.as_str()).collect();
        let mut with_help = args.clone();
        with_help.push("--help");
        let out = Command::new(horus())
            .args(&with_help)
            .output()
            .expect("horus <path> --help must run");
        let help = String::from_utf8_lossy(&out.stdout);
        for (short, long) in short_flags(&help) {
            meanings
                .entry(short)
                .or_default()
                .entry(long)
                .or_default()
                .push(args.join(" "));
        }
    }
    assert!(
        meanings.len() > 10,
        "parsed almost no short flags ({}) — the help format changed",
        meanings.len()
    );

    let mut problems = Vec::new();
    for (short, longs) in &meanings {
        if longs.len() < 2 {
            continue; // one meaning: not a collision
        }
        let found: Vec<&str> = longs.keys().map(|s| s.as_str()).collect();
        let recorded: Option<&[&str]> = LEDGER
            .iter()
            .find(|(c, _)| c == short)
            .map(|(_, longs)| *longs);
        match recorded {
            None => problems.push(format!(
                "-{short} now means several things and is not in the ledger: {}",
                longs
                    .iter()
                    .map(|(l, cmds)| format!("--{l} ({})", cmds.join(", ")))
                    .collect::<Vec<_>>()
                    .join("; ")
            )),
            Some(recorded) if recorded != found.as_slice() => problems.push(format!(
                "-{short} changed meaning: ledger says {recorded:?}, the CLI says {}",
                longs
                    .iter()
                    .map(|(l, cmds)| format!("--{l} ({})", cmds.join(", ")))
                    .collect::<Vec<_>>()
                    .join("; ")
            )),
            Some(_) => {}
        }
    }

    assert!(
        problems.is_empty(),
        "short flags gained or lost a meaning:\n  {}\n\n\
         If a flag gained one: give it a different letter, or spell it out. \
         Two commands used in the same session must not read the same letter \
         differently. If a collision was deliberately retired, delete it from \
         the ledger in this test.",
        problems.join("\n  ")
    );
}

/// `horus new`'s language flags: deprecated, not broken.
///
/// The fix must do both halves. Advertising `-r` for `--rust` four lines above
/// examples where `-r` means `--release` is what taught the collision, so the
/// help and the Quick Start must stop showing it — and every script, README
/// and CI job that already types `horus new x -r` must keep working, with a
/// notice saying what to write instead and when it stops being accepted.
#[test]
fn the_new_language_short_flags_are_deprecated_not_removed() {
    let new_help = String::from_utf8(
        Command::new(horus())
            .args(["new", "--help"])
            .output()
            .expect("horus new --help")
            .stdout,
    )
    .expect("utf-8");

    for (short, long) in [('r', "rust"), ('p', "python"), ('c', "cpp")] {
        assert!(
            !new_help.contains(&format!("-{short}, --{long}")),
            "`horus new --help` still advertises `-{short}, --{long}`, the \
             collision this fix retires:\n{new_help}"
        );
        assert!(
            new_help.contains(&format!("--{long}")),
            "`--{long}` disappeared from `horus new --help` entirely"
        );
    }

    let top = help_text();
    assert!(
        !top.contains("horus new my_robot -r"),
        "the Quick Start still teaches `-r` for Rust, four lines from examples \
         where `-r` means --release"
    );

    // Still accepted, and it says so.
    let tmp = std::env::temp_dir().join(format!("horus-flag1-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&tmp);
    std::fs::create_dir_all(&tmp).expect("temp dir");
    let out = Command::new(horus())
        .args(["new", "shortflag", "-r", "-y", "-o", &tmp.to_string_lossy()])
        .output()
        .expect("horus new -r must run");
    let stderr = String::from_utf8_lossy(&out.stderr).into_owned();
    let made_rust = tmp.join("shortflag/src/main.rs").exists();
    let _ = std::fs::remove_dir_all(&tmp);

    assert!(
        out.status.success() && made_rust,
        "`horus new <name> -r` must keep creating a Rust project — removing it \
         outright breaks every script that uses it. stderr:\n{stderr}"
    );
    assert!(
        stderr.contains("--rust") && stderr.contains("--release"),
        "`horus new -r` must say what to write instead and what `-r` means \
         elsewhere. stderr was:\n{stderr}"
    );
    assert!(
        stderr.contains("0.4.0"),
        "a deprecation with no removal release is not a deprecation cycle. \
         stderr was:\n{stderr}"
    );
}

// ── Commands that sound like each other ─────────────────────────────────

fn sub_help(args: &[&str]) -> String {
    let mut all: Vec<&str> = args.to_vec();
    all.push("--help");
    String::from_utf8(
        Command::new(horus())
            .args(&all)
            .output()
            .unwrap_or_else(|e| panic!("horus {} --help must run: {e}", args.join(" ")))
            .stdout,
    )
    .expect("utf-8")
}

/// `add` and `install` must each say what the other one does.
///
/// Both take a package name, both talk to the registry, and their one-line
/// helps — "Add a dependency to horus.toml" and "Install a standalone package
/// or plugin from the registry" — never mentioned each other, so nothing told
/// a user which of the two they wanted. The difference is not cosmetic: `add`
/// edits this project's manifest, `install` puts something on the machine, and
/// picking the wrong one leaves either a dependency that is not installed or
/// an installed package no build reads.
#[test]
fn add_and_install_say_how_they_differ() {
    let add = sub_help(&["add"]);
    let install = sub_help(&["install"]);
    let remove = sub_help(&["remove"]);
    let uninstall = sub_help(&["uninstall"]);

    assert!(
        add.contains("horus install") && add.contains("horus remove"),
        "`horus add --help` must name `horus install` (the one it is confused \
         with) and `horus remove` (its undo):\n{add}"
    );
    assert!(
        install.contains("horus add") && install.contains("horus uninstall"),
        "`horus install --help` must name `horus add` and `horus uninstall`:\n{install}"
    );
    assert!(
        remove.contains("horus uninstall"),
        "`horus remove --help` must point at `horus uninstall` for the other \
         kind of removal:\n{remove}"
    );
    assert!(
        uninstall.contains("horus remove"),
        "`horus uninstall --help` must point at `horus remove`:\n{uninstall}"
    );
}

/// Bare `horus list` must say it is not the runtime `list`.
///
/// Seven commands take a `list` subcommand — `topic`, `node`, `param`,
/// `service`, `action`, `frame`, `msg` — and all seven ask a *running* system
/// what it has. Bare `horus list` lists installed packages, and said only
/// "List installed packages and plugins", so a user who ran it while debugging
/// a live system got an unrelated answer with nothing marking it as unrelated.
#[test]
fn bare_list_says_it_is_not_the_runtime_list() {
    let list = sub_help(&["list"]);
    assert!(
        list.contains("topic list") && list.contains("node list"),
        "`horus list --help` must distinguish itself from the runtime \
         `horus topic list` / `horus node list` family:\n{list}"
    );
}

/// `horus publish` must say it is not the `publish()` of the message API.
///
/// `publish()` in the Rust, C++ and Python APIs sends one message on a topic —
/// the first verb anyone learns here. `horus publish` uploads a package to the
/// registry. The audit's suggested rename to `horus registry publish` is
/// declined, in writing, in the command's own long help: six sibling verbs
/// (`unpublish`, `yank`, `unyank`, `deprecate`, `undeprecate`, `owner`) are
/// top-level too. Declining is fine; leaving the collision unmentioned is not.
#[test]
fn publish_says_which_publish_it_is() {
    let publish = sub_help(&["publish"]);
    assert!(
        publish.contains("topic pub"),
        "`horus publish --help` must point at `horus topic pub` for the other \
         meaning of publish:\n{publish}"
    );
    assert!(
        publish.contains("registry publish"),
        "the decision not to rename `publish` to `horus registry publish` must \
         be recorded where a user reading the help can see it:\n{publish}"
    );
}

/// `horus env --help` must name every tool the shell integration shadows.
///
/// `horus env --init` writes shell functions for `cargo`, `pip`, `pip3`,
/// `cmake`, `conan` and `vcpkg`, and every in-CLI description of it said
/// "(cargo, pip, cmake)" — three of six. A user who reads the help has no way
/// to learn that `conan` and `vcpkg` in their shell now route through horus
/// inside a project, which is exactly the surprise `env --init` is criticised
/// for. The list is read from the shell snippet `env --init` actually writes,
/// so it cannot be satisfied by a stale hardcoded list on either side.
#[test]
fn env_help_names_every_tool_the_shell_integration_shadows() {
    let env_rs = std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/commands/env.rs"),
    )
    .expect("env.rs must be readable");

    // The POSIX snippet defines one shell function per shadowed tool:
    // `cargo() {`, `pip3() {`, ...
    let shadowed: BTreeSet<String> = env_rs
        .lines()
        .filter_map(|l| l.trim().strip_suffix("() {"))
        .filter(|n| {
            !n.is_empty()
                && n.chars()
                    .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit())
        })
        .map(|n| n.to_string())
        .collect();

    assert!(
        shadowed.len() >= 5,
        "found {} shell functions in env.rs — the generated snippet's shape \
         changed and this test is reading nothing: {shadowed:?}",
        shadowed.len()
    );

    let help = sub_help(&["env"]);
    let unmentioned: Vec<&String> = shadowed
        .iter()
        .filter(|t| !help.contains(t.as_str()))
        .collect();

    assert!(
        unmentioned.is_empty(),
        "`horus env --init` shadows these commands in the user's shell and \
         `horus env --help` never names them: {unmentioned:?}\n{help}"
    );
}
