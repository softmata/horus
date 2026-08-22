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
    let src = std::fs::read_to_string(proj.join("main.py")).expect("main.py must exist");

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

    let sa = std::fs::read_to_string(a.join("main.py")).unwrap();
    let sb = std::fs::read_to_string(b.join("main.py")).unwrap();

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

/// Where the README's Python example lives, relative to this crate.
///
/// The test resolves it rather than hard-coding the shape it expects, because
/// the point is not "lists are correct" — both forms work — but that the two
/// places a new user reads first do not disagree.
fn readme() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../README.md")
}

/// The README's fenced Python examples, with the prose stripped out.
///
/// The prose now says both spellings work and shows one of each, so scanning
/// the whole file would find whichever sentence happened to come first. The
/// fenced example is the code a reader copies, and that is what the template
/// has to match.
fn readme_python(doc: &str) -> String {
    let mut code = String::new();
    let mut inside = false;
    for line in doc.lines() {
        if line.starts_with("```") {
            inside = line.trim_start_matches('`').trim() == "python";
        } else if inside {
            code.push_str(line);
            code.push('\n');
        }
    }
    code
}

/// Is `arg=` handed a list or a bare string at its first *call site*?
///
/// Prose mentions the argument by name too (``pubs=`` in a sentence, the
/// README's own note that both spellings work), so occurrences that are not
/// followed by a value are skipped rather than counted as the answer.
fn arg_form(src: &str, arg: &str) -> Option<&'static str> {
    let needle = format!("{arg}=");
    src.match_indices(&needle)
        .filter_map(|(at, _)| src[at + needle.len()..].chars().next())
        .find_map(|c| match c {
            '[' => Some("a list"),
            '"' | '\'' => Some("a bare string"),
            _ => None,
        })
}

/// GEN-2: the template and the README must spell `pubs=`/`subs=` the same way.
///
/// `horus new --python` wrote `pubs="motors.cmd_vel"` while the README's Python
/// example writes `pubs=["sensor.data"]`. The binding takes either — `pubs` is
/// typed `Optional[Union[List[str], str, Dict[str, Dict]]]` — but neither place
/// said so, so a reader who copied the README and a reader who ran the
/// generator saw two spellings of one argument with no way to tell whether both
/// were legal short of reading the binding source. Both halves were fixed: the
/// README now states that either form builds the same node, and the template
/// shows the same form the example does.
#[test]
fn generated_python_and_the_readme_agree_on_pubs_and_subs() {
    let tmp = tempfile::tempdir().unwrap();
    let proj = generate(tmp.path(), "shapebot", "--python");
    let src = std::fs::read_to_string(proj.join("main.py")).expect("main.py must exist");

    assert_eq!(
        arg_form(&src, "pubs"),
        Some("a list"),
        "the template should pass topics the way the README's example does:\n{src}"
    );
    assert_eq!(
        arg_form(&src, "subs"),
        Some("a list"),
        "the template should pass topics the way the README's example does:\n{src}"
    );

    // Read the README back, so this fails whichever side of the pair drifts.
    let Ok(doc) = std::fs::read_to_string(readme()) else {
        eprintln!(
            "skipping the README half: {} not readable",
            readme().display()
        );
        return;
    };
    let example = readme_python(&doc);
    assert!(
        !example.is_empty(),
        "no fenced ```python block in {}",
        readme().display()
    );
    for arg in ["pubs", "subs"] {
        assert_eq!(
            arg_form(&example, arg),
            arg_form(&src, arg),
            "README.md passes {arg}= {:?} and `horus new --python` passes it {:?} — \
             a new user reads both and cannot tell which is right",
            arg_form(&example, arg),
            arg_form(&src, arg)
        );
    }
}
