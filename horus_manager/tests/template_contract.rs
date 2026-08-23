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

// ---------------------------------------------------------------------------
// `horus new <name> --from <example>`
// ---------------------------------------------------------------------------
//
// There was no supported path from "I like this example" to "this is my
// project" other than `cp -r`, and a raw copy keeps the example's package name
// — the exact collision `two_projects_get_distinct_node_names` above exists to
// prevent, reintroduced by the only route a reader had.

fn repo_root() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager must have a parent")
        .to_path_buf()
}

/// Run `horus new <name> --from <example>` in `root`.
///
/// `HORUS_SOURCE` is pinned to this checkout so the test reads the examples it
/// ships with rather than whatever tree happens to be at ~/horus.
fn new_from(root: &Path, name: &str, example: &str) -> std::process::Output {
    Command::new(horus())
        .args(["new", name, "--from", example])
        .env("HORUS_SOURCE", repo_root())
        .current_dir(root)
        .output()
        .expect("horus new --from must run")
}

/// Every example project shipped under examples/.
fn shipped_examples() -> Vec<String> {
    let mut names: Vec<String> = std::fs::read_dir(repo_root().join("examples"))
        .expect("examples/ must exist")
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.is_dir() && p.join("horus.toml").is_file())
        .filter_map(|p| p.file_name().map(|n| n.to_string_lossy().to_string()))
        .collect();
    names.sort();
    assert!(!names.is_empty(), "no examples found");
    names
}

fn package_name(manifest: &Path) -> String {
    let text = std::fs::read_to_string(manifest)
        .unwrap_or_else(|e| panic!("{} must exist: {e}", manifest.display()));
    for line in text.lines() {
        if let Some(rest) = line.trim().strip_prefix("name") {
            if let Some((_, v)) = rest.split_once('=') {
                return v.trim().trim_matches('"').to_string();
            }
        }
    }
    panic!("no `name` in {}:\n{text}", manifest.display());
}

#[test]
fn new_from_an_example_renames_the_package() {
    let tmp = tempfile::tempdir().unwrap();
    let out = new_from(tmp.path(), "my_robot", "differential_drive");
    assert!(
        out.status.success(),
        "horus new my_robot --from differential_drive failed:\n{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );

    let project = tmp.path().join("my_robot");
    assert_eq!(
        package_name(&project.join("horus.toml")),
        "my_robot",
        "the copy kept the example's package name — so its binary, its cargo \
         package and `horus node list` all still say `differential_drive`"
    );
    // The sources came across, not just a manifest.
    assert!(
        project.join("main.rs").is_file(),
        "main.rs was not copied: {:?}",
        std::fs::read_dir(&project)
            .map(|d| d.flatten().map(|e| e.file_name()).collect::<Vec<_>>())
            .unwrap_or_default()
    );
    assert!(
        project.join("README.md").is_file(),
        "README.md was not copied"
    );
    assert!(
        project.join("robots/diffbot.urdf").is_file(),
        "the robots/ subdirectory was not copied"
    );
}

/// The copy must not carry the example's build output.
///
/// `.horus/` holds a generated Cargo.toml with absolute path dependencies on
/// whichever HORUS tree built the example, plus a target directory that can be
/// hundreds of megabytes; a C++ build additionally symlinks
/// `compile_commands.json` into the project root, and Python leaves
/// `__pycache__/`. All of it describes the example's directory, not the new
/// one.
#[test]
fn new_from_leaves_the_examples_build_output_behind() {
    for example in shipped_examples() {
        let tmp = tempfile::tempdir().unwrap();
        let out = new_from(tmp.path(), "copied", &example);
        assert!(
            out.status.success(),
            "horus new copied --from {example} failed:\n{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        );

        let project = tmp.path().join("copied");
        assert_eq!(
            package_name(&project.join("horus.toml")),
            "copied",
            "in {example}"
        );

        for stale in [
            ".horus/target",
            ".horus/cpp-build",
            ".horus/packages",
            "__pycache__",
            ".ruff_cache",
            ".pytest_cache",
            "compile_commands.json",
            "target",
        ] {
            assert!(
                !project.join(stale).exists(),
                "{example} copied its build output: {stale} is present in the new project"
            );
        }
    }
}

/// An unknown name is the likely mistake, because nothing in the CLI lists the
/// examples. So the error is the list.
#[test]
fn new_from_an_unknown_example_lists_what_is_available() {
    let tmp = tempfile::tempdir().unwrap();
    let out = new_from(tmp.path(), "my_robot", "no_such_example");
    assert!(!out.status.success(), "an unknown example must not succeed");

    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    for example in shipped_examples() {
        assert!(
            text.contains(&example),
            "the error does not mention `{example}`, so the reader still has no \
             way to find out what --from accepts:\n{text}"
        );
    }
    assert!(
        !tmp.path().join("my_robot").exists(),
        "a failed --from left a half-made project behind"
    );
}

/// `--from` names an example, not a path.
///
/// Most traversals are already stopped by the horus.toml probe — there is no
/// manifest at `examples/../../etc` — so the case that discriminates is one
/// that climbs out and lands on a real HORUS project:
/// `../examples/differential_drive` resolves to a directory with a horus.toml
/// and would otherwise be copied, teaching `--from` to take paths.
#[test]
fn new_from_cannot_escape_the_examples_directory() {
    for hostile in [
        "../horus_manager",
        "../../etc",
        "/etc",
        "differential_drive/robots",
        "../examples/differential_drive",
        "./differential_drive",
    ] {
        let tmp = tempfile::tempdir().unwrap();
        let out = new_from(tmp.path(), "escapee", hostile);
        assert!(
            !out.status.success(),
            "--from {hostile} was accepted; it must name a directory under examples/"
        );
        assert!(
            !tmp.path().join("escapee").exists(),
            "--from {hostile} created a project"
        );
    }
}

/// The language comes from the example, so the language flags cannot also
/// apply. Clap rejects them rather than silently picking one.
#[test]
fn new_from_rejects_the_flags_it_would_have_to_ignore() {
    for flag in [
        "--rust",
        "--python",
        "--cpp",
        "--workspace",
        "--lib",
        "--macro",
    ] {
        let tmp = tempfile::tempdir().unwrap();
        let out = Command::new(horus())
            .args(["new", "my_robot", "--from", "differential_drive", flag])
            .env("HORUS_SOURCE", repo_root())
            .current_dir(tmp.path())
            .output()
            .expect("horus new must run");
        assert!(
            !out.status.success(),
            "`horus new --from differential_drive {flag}` was accepted, so {flag} \
             is silently ignored"
        );
    }
}

/// The C++ example is the one where the rename has to reach past cargo: its
/// binary name comes from cmake_gen, not cargo_gen.
#[test]
fn new_from_renames_a_cpp_example_too() {
    let cpp = shipped_examples().into_iter().find(|e| {
        repo_root()
            .join("examples")
            .join(e)
            .join("src/main.cpp")
            .is_file()
    });
    let Some(cpp) = cpp else {
        panic!("no C++ example under examples/ — see examples_contract");
    };

    let tmp = tempfile::tempdir().unwrap();
    let out = new_from(tmp.path(), "my_cpp_robot", &cpp);
    assert!(
        out.status.success(),
        "horus new my_cpp_robot --from {cpp} failed:\n{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );

    let project = tmp.path().join("my_cpp_robot");
    assert_eq!(package_name(&project.join("horus.toml")), "my_cpp_robot");
    assert!(
        project.join("src/main.cpp").is_file(),
        "src/main.cpp was not copied"
    );
    assert!(
        project.join(".gitignore").is_file(),
        "a project outside the HORUS repository cannot rely on the repository's \
         root .gitignore, so --from must leave one behind"
    );
    // `horus fmt --check` on a C++ project with no .clang-format falls back to
    // LLVM style, which does not match the code HORUS itself generates — so a
    // freshly made project fails a check on code its owner has not written yet.
    assert!(
        project.join(".clang-format").is_file(),
        "the C++ copy has no .clang-format, so `horus fmt --check` will fail on it"
    );
}
