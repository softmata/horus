//! What `horus.toml` accepts, and what it says about what it does not.
//!
//! # The defect this suite exists for
//!
//! A key HORUS does not understand is dropped by serde and never mentioned
//! again. The 0.2.x fix reported those keys — but only for the top-level table
//! and four of the nested ones, and only on `horus check`'s *directory* path,
//! and only as a warning, so the audit's own reproduction still ended:
//!
//! ```text
//! $ horus check          # [network] enabeld = true, [robot] descriptoin = "..."
//!   Status: * All checks passed!
//! $ echo $?
//! 0
//! ```
//!
//! `[network]`, `[robot]`, `[workspace]` and `[ignore]` are closed structs
//! exactly like `[package]`; a typo in any of them was silently discarded. The
//! `[network]` one is the expensive case: `sekret` instead of `secret` leaves a
//! fleet unauthenticated while the manifest looks configured.
//!
//! Run: `cargo test -p horus_manager --test manifest_contract`

use horus_manager::manifest_lint::{find_unknown_keys, CLOSED_TABLES};
use std::path::{Path, PathBuf};
use std::process::Command;

fn horus() -> &'static str {
    env!("CARGO_BIN_EXE_horus")
}

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager has a parent")
        .to_path_buf()
}

/// A project whose manifest misspells one key in every closed nested table.
fn project_with_nested_typos(dir: &Path) {
    std::fs::write(
        dir.join("horus.toml"),
        "[package]\n\
         name = \"nestedbot\"\n\
         version = \"0.1.0\"\n\
         \n\
         [robot]\n\
         name = \"arm\"\n\
         descriptoin = \"arm.urdf\"\n\
         \n\
         [workspace]\n\
         members = [\"crates/*\"]\n\
         membrs = [\"crates/*\"]\n\
         \n\
         [ignore]\n\
         fils = [\"debug_*.rs\"]\n\
         \n\
         [network]\n\
         sekret = \"hunter2\"\n\
         \n\
         [network.safety]\n\
         heartbeat_msec = 50\n",
    )
    .unwrap();
    std::fs::create_dir_all(dir.join("src")).unwrap();
    std::fs::write(dir.join("src/main.rs"), "fn main(){}\n").unwrap();
}

fn run(args: &[&str], dir: &Path) -> (String, bool) {
    let out = Command::new(horus())
        .args(args)
        .current_dir(dir)
        .output()
        .unwrap_or_else(|e| panic!("horus {args:?} must run: {e}"));
    (
        format!(
            "{}{}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        ),
        out.status.success(),
    )
}

// ── The nested tables the first fix went blind on ───────────────────────────

#[test]
fn typos_in_every_closed_nested_table_are_reported() {
    let tmp = tempfile::tempdir().unwrap();
    project_with_nested_typos(tmp.path());
    let (text, ok) = run(&["check"], tmp.path());

    for (key, line) in [
        ("robot.descriptoin", 7),
        ("workspace.membrs", 11),
        ("ignore.fils", 14),
        ("network.sekret", 17),
        ("network.safety.heartbeat_msec", 20),
    ] {
        assert!(
            text.contains(key),
            "`{key}` is a misspelling in a closed table and must be reported:\n{text}"
        );
        assert!(
            text.contains(&format!("horus.toml:{line}:")),
            "`{key}` must be reported at line {line}:\n{text}"
        );
    }
    assert!(
        !ok,
        "a manifest whose settings do nothing must not exit 0:\n{text}"
    );
}

/// The did-you-mean has to survive into the nested tables too, or the report
/// is only half the help.
#[test]
fn a_nested_typo_suggests_the_real_key() {
    let tmp = tempfile::tempdir().unwrap();
    project_with_nested_typos(tmp.path());
    let (text, _) = run(&["check"], tmp.path());
    assert!(
        text.contains("did you mean `secret`"),
        "`sekret` is one edit from `secret`:\n{text}"
    );
    assert!(
        text.contains("did you mean `members`"),
        "`membrs` is one edit from `members`:\n{text}"
    );
}

/// `horus check` must never call a manifest valid while reporting keys that do
/// nothing. That sentence, over that file, is the whole finding.
#[test]
fn an_unknown_key_is_an_error_not_a_clean_bill_of_health() {
    let tmp = tempfile::tempdir().unwrap();
    std::fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"typobot\"\nversion = \"0.1.0\"\nlangauge = \"rust\"\n",
    )
    .unwrap();
    std::fs::create_dir_all(tmp.path().join("src")).unwrap();
    std::fs::write(tmp.path().join("src/main.rs"), "fn main(){}\n").unwrap();

    let (text, ok) = run(&["check"], tmp.path());
    assert!(!ok, "exit code must be non-zero:\n{text}");
    assert!(
        !text.contains("manifest valid"),
        "the file was called valid while a key in it does nothing:\n{text}"
    );
    assert!(
        !text.contains("All checks passed"),
        "the check did not pass:\n{text}"
    );
    // `langauge` is not close to any real key; it is a misspelling of a key
    // that does not exist, and the explanation is what helps.
    assert!(
        text.contains("detects the language from your source files"),
        "a near miss on an assumed key must get the tailored explanation:\n{text}"
    );
}

/// The single-file form reads the same file and must reach the same verdict.
#[test]
fn the_single_file_path_reports_unknown_keys_too() {
    let tmp = tempfile::tempdir().unwrap();
    project_with_nested_typos(tmp.path());
    let (text, ok) = run(&["check", "horus.toml"], tmp.path());

    assert!(
        text.contains("network.sekret"),
        "`horus check <file>` skipped the unknown-key check entirely:\n{text}"
    );
    assert!(!ok, "and it exited 0 while doing so:\n{text}");
    assert!(
        !text.contains("All checks passed"),
        "it also reported the file as clean:\n{text}"
    );
}

/// Everything else that loads a manifest has to say so as well — silence at
/// `run`/`build` time is how the typo survives long enough to matter.
#[test]
fn build_warns_about_unknown_keys_on_stderr() {
    let tmp = tempfile::tempdir().unwrap();
    std::fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"typobot\"\nversion = \"0.1.0\"\n\n[network]\nsekret = \"x\"\n",
    )
    .unwrap();
    std::fs::write(tmp.path().join("main.py"), "print('hi')\n").unwrap();

    let out = Command::new(horus())
        .arg("build")
        .current_dir(tmp.path())
        .output()
        .expect("horus build must run");
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.contains("network.sekret"),
        "`horus build` read the manifest and said nothing about the key it \
         ignored (stderr):\n{stderr}"
    );
    assert!(
        stderr.contains("horus.toml:6"),
        "the warning must carry the line, like every other diagnostic:\n{stderr}"
    );
}

/// Guards the *list*, not one entry of it: every table declared closed must
/// actually be linted, so adding one to `CLOSED_TABLES` without wiring it up
/// fails here.
#[test]
fn every_closed_table_is_actually_linted() {
    for (path, known) in CLOSED_TABLES {
        let mut manifest = String::from("[package]\nname = \"x\"\nversion = \"0.1.0\"\n");
        if *path != "package" {
            manifest.push_str(&format!("\n[{path}]\n"));
        }
        // A key no table could plausibly own.
        manifest.push_str("zzz_not_a_key_anywhere = 1\n");
        // `[robot]` and `[workspace]` have required or meaningful fields; the
        // lint runs off the parsed document, so give them enough to parse.
        if *path == "robot" {
            manifest.push_str("name = \"r\"\n");
        }

        let found = find_unknown_keys(&manifest);
        assert!(
            found
                .iter()
                .any(|u| u.path == format!("{path}.zzz_not_a_key_anywhere")),
            "[{path}] is declared closed (its keys are {known:?}) but an unknown \
             key in it was not reported: {found:?}\n--- manifest ---\n{manifest}"
        );
    }
}

// ── Version strings (CFG-2) ─────────────────────────────────────────────────

/// `"latest"` is accepted by `horus add` and the registry, and is copied
/// verbatim into `.horus/Cargo.toml`, where cargo rejects it.
#[test]
fn a_latest_version_on_a_native_dependency_is_an_error() {
    let tmp = tempfile::tempdir().unwrap();
    std::fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"verbot\"\nversion = \"0.1.0\"\n\n\
         [dependencies]\n\
         serde = { version = \"latest\", source = \"crates.io\" }\n",
    )
    .unwrap();
    std::fs::create_dir_all(tmp.path().join("src")).unwrap();
    std::fs::write(tmp.path().join("src/main.rs"), "fn main(){}\n").unwrap();

    let (text, ok) = run(&["check"], tmp.path());
    assert!(
        text.contains("serde") && text.contains("not a version requirement"),
        "a version cargo cannot parse must be reported:\n{text}"
    );
    assert!(
        !ok,
        "the generated Cargo.toml will not load; that is an error:\n{text}"
    );
}

/// The registry does resolve it, so there it is a warning — not silence, and
/// not a broken build.
#[test]
fn a_latest_version_on_a_registry_dependency_is_a_warning() {
    let tmp = tempfile::tempdir().unwrap();
    std::fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"verbot\"\nversion = \"0.1.0\"\n\n\
         [dependencies]\n\
         pid-controller = \"latest\"\n",
    )
    .unwrap();
    std::fs::create_dir_all(tmp.path().join("src")).unwrap();
    std::fs::write(tmp.path().join("src/main.rs"), "fn main(){}\n").unwrap();

    let (text, ok) = run(&["check"], tmp.path());
    assert!(
        text.contains("pid-controller") && text.contains("not a version requirement"),
        "the non-portable spelling must be named:\n{text}"
    );
    assert!(
        ok,
        "but it resolves, so it must not fail the check:\n{text}"
    );
}

/// HORUS's own manifest declared 0.2.0 against a 0.3.0 binary. Nothing tied
/// them together, so it drifted the moment a release was cut.
#[test]
fn the_repository_manifest_states_the_current_version() {
    let manifest = repo_root().join("horus.toml");
    let text = std::fs::read_to_string(&manifest)
        .unwrap_or_else(|e| panic!("{} must be readable: {e}", manifest.display()));
    let parsed: toml::Table = toml::from_str(&text).expect("the repo manifest must be valid TOML");
    let declared = parsed["package"]["version"]
        .as_str()
        .expect("[package] version must be a string");

    assert_eq!(
        declared,
        env!("CARGO_PKG_VERSION"),
        "{} says {declared}, the CLI it ships is {}. HORUS's own manifest is the \
         first one anybody reads.",
        manifest.display(),
        env!("CARGO_PKG_VERSION")
    );
}

/// And it must be a manifest HORUS itself does not warn about.
#[test]
fn the_repository_manifest_has_no_keys_that_do_nothing() {
    let text = std::fs::read_to_string(repo_root().join("horus.toml")).expect("readable");
    let unknown = find_unknown_keys(&text);
    assert!(
        unknown.is_empty(),
        "HORUS's own horus.toml carries keys HORUS ignores: {unknown:?}"
    );
}

// ── [sim-drivers] (CFG-6) ───────────────────────────────────────────────────

/// `[sim-drivers]` parses and is then read by nothing at all. A project that
/// still relies on it runs the *real* driver under `--sim`, which is a moving
/// motor where a simulated one was expected.
#[test]
fn a_sim_drivers_table_is_reported_as_inert() {
    let tmp = tempfile::tempdir().unwrap();
    std::fs::write(
        tmp.path().join("horus.toml"),
        "[package]\nname = \"simbot\"\nversion = \"0.1.0\"\n\n\
         [drivers]\nlidar = \"rplidar\"\n\n\
         [sim-drivers]\nlidar = \"sim3d\"\n",
    )
    .unwrap();
    std::fs::create_dir_all(tmp.path().join("src")).unwrap();
    std::fs::write(tmp.path().join("src/main.rs"), "fn main(){}\n").unwrap();

    let (text, _) = run(&["check"], tmp.path());
    assert!(
        text.contains("[sim-drivers] is no longer consulted"),
        "a table nothing reads must say so where the user is looking:\n{text}"
    );
    assert!(
        text.contains("sim = true"),
        "and it must name the replacement:\n{text}"
    );
}
