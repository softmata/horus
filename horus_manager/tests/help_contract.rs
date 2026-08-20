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
