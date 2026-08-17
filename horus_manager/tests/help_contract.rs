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

/// `<name> --help` resolves, i.e. clap recognises it.
fn resolves(name: &str) -> bool {
    let out = Command::new(horus())
        .args([name, "--help"])
        .output()
        .expect("horus <cmd> --help must run");
    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    !text.contains("unrecognized subcommand")
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
