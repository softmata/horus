//! Generated projects must satisfy the tools HORUS itself ships.
//!
//! `horus new --python` emitted a `main.py` that failed `horus lint` and
//! `horus fmt --check` on the first run — HORUS ships ruff, configures it in
//! the generated `.horus/pyproject.toml`, and then generated code that violated
//! it. Any team with lint in CI was red on commit zero, and the first thing a
//! new user learned was that the tool disagreed with itself:
//!
//! ```text
//! $ horus new lintme --python && cd lintme && horus lint
//! I001 [*] Import block is un-sorted or un-formatted
//! F841 Local variable `sensor_data` is assigned to but never used
//! $ horus fmt --check
//! unformatted: File would be reformatted
//! ```
//!
//! Templates also hard-coded the node name `controller`, so two HORUS projects
//! on one machine collided — `horus new` guaranteed the condition the runtime
//! warns about:
//!
//! ```text
//! [horus] WARNING: node 'controller' already registered by PID 497819
//! (this is PID 498342). Overwriting presence file — duplicate node names
//! cause unreliable discovery.
//! ```
//!
//! Run: `cargo test -p horus_manager --test template_contract`

use std::path::Path;
use std::process::Command;

fn horus() -> &'static str {
    env!("CARGO_BIN_EXE_horus")
}

/// Generate a project, returning its directory.
///
/// `--yes` keeps this non-interactive; without it `horus new` used to block on
/// two prompts that the README never mentions.
fn generate(root: &Path, name: &str, lang: &str) -> std::path::PathBuf {
    let out = Command::new(horus())
        .args(["new", name, lang, "--yes"])
        .current_dir(root)
        .output()
        .expect("horus new must run");
    assert!(
        out.status.success(),
        "horus new {name} {lang} failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    root.join(name)
}

fn run_in(dir: &Path, args: &[&str]) -> (bool, String) {
    let out = Command::new(horus())
        .args(args)
        .current_dir(dir)
        .output()
        .unwrap_or_else(|e| panic!("horus {args:?} must run: {e}"));
    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    (out.status.success(), text)
}

/// Skip rather than fail when the external tool is absent, so the suite still
/// runs on a machine without ruff. The CI image has it.
fn have(tool: &str) -> bool {
    Command::new(tool)
        .arg("--version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

#[test]
fn generated_python_passes_horus_lint() {
    if !have("ruff") {
        eprintln!("skipping: ruff not installed");
        return;
    }
    let tmp = tempfile::tempdir().unwrap();
    let proj = generate(tmp.path(), "lintbot", "--python");

    let (ok, text) = run_in(&proj, &["lint"]);
    assert!(
        ok,
        "a freshly generated project must pass the linter HORUS ships:\n{text}"
    );
}

#[test]
fn generated_python_is_already_formatted() {
    if !have("ruff") {
        eprintln!("skipping: ruff not installed");
        return;
    }
    let tmp = tempfile::tempdir().unwrap();
    let proj = generate(tmp.path(), "fmtbot", "--python");

    let (ok, text) = run_in(&proj, &["fmt", "--check"]);
    assert!(
        ok,
        "a freshly generated project must already satisfy the formatter \
         HORUS ships:\n{text}"
    );
}

/// Two projects on one machine must not collide on the node name.
#[test]
fn generated_node_is_named_after_the_project() {
    let tmp = tempfile::tempdir().unwrap();
    let proj = generate(tmp.path(), "alpha-bot", "--python");
    let src = std::fs::read_to_string(proj.join("src/main.py")).expect("main.py must exist");

    assert!(
        src.contains("alpha_bot_controller"),
        "the node should be named after the project, got:\n{src}"
    );
    assert!(
        !src.contains(r#"name="controller""#),
        "hard-coding `controller` makes every project collide with every other"
    );
}

/// Distinct projects must produce distinct node names — the whole point.
#[test]
fn two_projects_get_distinct_node_names() {
    let tmp = tempfile::tempdir().unwrap();
    let a = generate(tmp.path(), "robot_one", "--python");
    let b = generate(tmp.path(), "robot_two", "--python");

    let sa = std::fs::read_to_string(a.join("src/main.py")).unwrap();
    let sb = std::fs::read_to_string(b.join("src/main.py")).unwrap();

    let name_of = |s: &str| {
        s.lines()
            .find(|l| l.trim_start().starts_with("name="))
            .map(|l| l.trim().to_string())
            .unwrap_or_default()
    };
    assert_ne!(
        name_of(&sa),
        name_of(&sb),
        "two projects must not share a node name"
    );
}

/// `horus new` must not block when nobody is at the keyboard.
///
/// The README's headline is `horus new my_robot && cd my_robot && horus run`,
/// and with no language flag that opened two prompts the README never
/// mentions — so the documented one-liner could not be scripted.
#[test]
fn new_is_non_interactive_without_a_tty() {
    use std::process::Stdio;
    let tmp = tempfile::tempdir().unwrap();

    let out = Command::new(horus())
        .args(["new", "scripted"])
        .current_dir(tmp.path())
        .stdin(Stdio::null())
        .output()
        .expect("horus new must run");

    assert!(
        out.status.success(),
        "horus new with no flags and no tty must complete: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    assert!(
        tmp.path().join("scripted/horus.toml").is_file(),
        "the project should have been created"
    );
}

/// A brand-new project has no tests. pytest exit code 5 means "no tests
/// collected", which is not a failure — reporting it as one made a user's very
/// first `horus test` red in every configuration.
#[test]
fn horus_test_on_a_new_project_is_not_a_failure() {
    if !have("pytest") && !have("py.test") {
        eprintln!("skipping: pytest not installed");
        return;
    }
    let tmp = tempfile::tempdir().unwrap();
    let proj = generate(tmp.path(), "testbot", "--python");

    let (ok, text) = run_in(&proj, &["test"]);
    assert!(
        ok,
        "an empty project should report 'no tests found', not fail:\n{text}"
    );
    assert!(
        text.contains("No Python tests found"),
        "the message should say what to do next:\n{text}"
    );
}
