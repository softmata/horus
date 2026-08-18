//! Every lifecycle phase that has a `pre_` hook must have a `post_` hook.
//!
//! The hook table shipped as:
//!
//! ```toml
//! [hooks]
//! pre_run   = [...]      # existed
//! pre_build = [...]      # existed
//! pre_test  = [...]      # existed
//! post_test = [...]      # existed
//! # post_run   -- did not exist
//! # post_build -- did not exist
//! ```
//!
//! `test` was the only phase with teardown. For a robotics framework that is
//! precisely backwards: the phase that most needs an "after" hook is `run`,
//! where a crash can leave a CAN bus open, an arm unparked, or a motor
//! controller energized. Users writing `post_run = ["park"]` got no error and
//! no hook — an unknown key in a table nothing validated.
//!
//! These tests pin the symmetry so a future phase cannot be added half-way.
//!
//! Run: `cargo test -p horus_manager --test hooks_contract`

use horus_manager::manifest::HorusManifest;
use horus_manager::manifest_lint::{find_unknown_keys, KNOWN_HOOKS};

/// The property, stated directly: `pre_x` implies `post_x` and vice versa.
#[test]
fn every_phase_has_both_a_pre_and_a_post_hook() {
    let mut missing = Vec::new();

    for hook in KNOWN_HOOKS {
        let counterpart = match hook.split_once('_') {
            Some(("pre", phase)) => format!("post_{phase}"),
            Some(("post", phase)) => format!("pre_{phase}"),
            _ => {
                panic!("hook `{hook}` is neither pre_ nor post_; the naming convention is the API")
            }
        };
        if !KNOWN_HOOKS.contains(&counterpart.as_str()) {
            missing.push(format!("`{hook}` exists but `{counterpart}` does not"));
        }
    }

    assert!(
        missing.is_empty(),
        "the hook table is asymmetric, so users cannot express teardown for \
         some phases:\n  {}\n\nA phase with setup but no teardown is a phase \
         where hardware gets left live after a failure.",
        missing.join("\n  ")
    );
}

/// The three phases the CLI actually has must all be represented.
#[test]
fn run_build_and_test_all_have_hooks() {
    for phase in ["run", "build", "test"] {
        for prefix in ["pre", "post"] {
            let hook = format!("{prefix}_{phase}");
            assert!(
                KNOWN_HOOKS.contains(&hook.as_str()),
                "`horus {phase}` has no `{hook}` hook, so a project cannot \
                 script that point of its lifecycle"
            );
        }
    }
}

/// The manifest must actually deserialize the new keys — the lint knowing a
/// name is worthless if serde drops the value.
#[test]
fn post_hooks_deserialize_into_the_manifest() {
    let manifest: HorusManifest = toml::from_str(
        "[package]\n\
         name = \"armbot\"\n\
         version = \"0.1.0\"\n\
         \n\
         [hooks]\n\
         pre_run = [\"fmt\"]\n\
         post_run = [\"park-arm\"]\n\
         post_build = [\"sign\"]\n",
    )
    .expect("a manifest using post_run/post_build must parse");

    assert_eq!(manifest.hooks.post_run, vec!["park-arm"]);
    assert_eq!(manifest.hooks.post_build, vec!["sign"]);
    assert_eq!(
        manifest.hooks.pre_run,
        vec!["fmt"],
        "existing hooks must not regress"
    );
}

/// A manifest that uses the new hooks must not be flagged as containing
/// unknown keys — the lint and the struct have to agree.
#[test]
fn the_lint_accepts_the_post_hooks() {
    let src = "[package]\n\
               name = \"armbot\"\n\
               version = \"0.1.0\"\n\
               \n\
               [hooks]\n\
               post_run = [\"park-arm\"]\n\
               post_build = [\"sign\"]\n";

    let unknown = find_unknown_keys(src);
    assert!(
        unknown.is_empty(),
        "the lint flags hooks the manifest accepts: {unknown:#?}"
    );
}

/// And a genuine typo in the table must still be caught, or the test above
/// would pass just as well with the lint disabled.
#[test]
fn a_misspelled_hook_is_still_reported() {
    let src = "[package]\n\
               name = \"armbot\"\n\
               version = \"0.1.0\"\n\
               \n\
               [hooks]\n\
               post_rnu = [\"park-arm\"]\n";

    let unknown = find_unknown_keys(src);
    assert!(
        unknown.iter().any(|d| d.message().contains("post_rnu")),
        "`post_rnu` should be reported as unknown, got: {unknown:#?}"
    );
    assert!(
        unknown.iter().any(|d| d.message().contains("post_run")),
        "and the suggestion should name the real hook, got: {unknown:#?}"
    );
}

/// `is_empty()` gates whether the section is serialized at all. If it ignores
/// the new fields, a manifest with only post hooks round-trips to nothing.
#[test]
fn a_manifest_with_only_post_hooks_is_not_considered_empty() {
    let manifest: HorusManifest = toml::from_str(
        "[package]\n\
         name = \"armbot\"\n\
         version = \"0.1.0\"\n\
         \n\
         [hooks]\n\
         post_run = [\"park-arm\"]\n",
    )
    .unwrap();

    assert!(
        !manifest.hooks.is_empty(),
        "hooks.is_empty() ignores post_run, so the section would be dropped \
         when the manifest is written back out"
    );
}

// ---------------------------------------------------------------------------
// End-to-end: the hook has to actually fire, and must not eat the real error.
// ---------------------------------------------------------------------------

/// A project with a `post_run` hook but no source file. `horus run` fails
/// early, before cargo, which keeps this test fast while exercising the branch
/// that matters most: teardown after a *failed* run.
fn project_with_post_run(dir: &std::path::Path) {
    std::fs::write(
        dir.join("horus.toml"),
        "[package]\n\
         name = \"armbot\"\n\
         version = \"0.1.0\"\n\
         \n\
         [scripts]\n\
         park = \"echo PARKED-THE-ARM\"\n\
         \n\
         [hooks]\n\
         post_run = [\"park\"]\n",
    )
    .unwrap();
}

fn run_horus(dir: &std::path::Path, args: &[&str]) -> (bool, String) {
    let out = std::process::Command::new(env!("CARGO_BIN_EXE_horus"))
        .args(args)
        .current_dir(dir)
        .output()
        .expect("horus must execute");
    let combined = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    (out.status.success(), combined)
}

#[test]
fn post_run_fires_even_when_the_run_failed() {
    let tmp = tempfile::tempdir().unwrap();
    project_with_post_run(tmp.path());
    let (_, combined) = run_horus(tmp.path(), &["run"]);

    assert!(
        combined.contains("PARKED-THE-ARM"),
        "the run failed and the teardown hook never ran, which is exactly when \
         hardware gets left live:\n{combined}"
    );
}

/// The teardown must not become the reported error. If it did, a user
/// debugging a crashed run would be shown their own cleanup script instead of
/// the crash.
#[test]
fn the_original_failure_survives_the_teardown_hook() {
    let tmp = tempfile::tempdir().unwrap();
    project_with_post_run(tmp.path());
    let (ok, combined) = run_horus(tmp.path(), &["run"]);

    assert!(!ok, "the run should still fail:\n{combined}");
    assert!(
        combined.contains("No main file detected"),
        "the real cause must still be reported after teardown runs:\n{combined}"
    );
}

/// `--no-hooks` must suppress teardown too, not just setup.
#[test]
fn no_hooks_suppresses_the_post_hook_as_well() {
    let tmp = tempfile::tempdir().unwrap();
    project_with_post_run(tmp.path());
    let (_, combined) = run_horus(tmp.path(), &["run", "--no-hooks"]);

    assert!(
        !combined.contains("PARKED-THE-ARM"),
        "--no-hooks must disable post_run as well as pre_run:\n{combined}"
    );
}
