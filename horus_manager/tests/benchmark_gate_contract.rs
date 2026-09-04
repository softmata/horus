//! The performance gate must have a baseline to compare against.
//!
//! On 2026-09-04 it did not. The newest green trunk run's own gate output read:
//!
//! ```text
//! current   : 988a080b3cb1c02fd1c0c55d1f58e12effa71850 (main)
//! baseline  : none — rolling window of 0 run(s)
//! ```
//!
//! and every metric of every benchmark carried `N/A (no baseline)`. Benchmark CI
//! was green on every pull request while comparing nothing at all. The window had
//! gone 3 -> 1 -> 1 -> 1 over four trunk runs.
//!
//! The cause was not the gate, which is careful, and not the save, which
//! succeeded — the run log says `Cache saved with key:
//! benchmark-history-main-33808800880`. It was **eviction**. Actions caches
//! share a 10 GB per-repository quota and are evicted least-recently-used. This
//! repository sits at ~9.84 GB across ~20 cargo caches of 300 MB-1 GB, each
//! touched by some workflow on nearly every push. The baseline is 4 KB and is
//! read by one job on one branch, so it is always the LRU victim — and six
//! seconds after saving it, the benchmark job saves a ~500 MB
//! `Linux-cargo-bench-*` entry of its own.
//!
//! The baseline therefore travels in the `benchmark-results` **artifact**, which
//! is not subject to the cache quota: every artifact from the period in which
//! every baseline cache had been evicted was still downloadable.
//!
//! These tests pin the mechanism, not the numbers. A gate with no baseline does
//! not fail — it passes, silently, which is why nothing caught this for four
//! runs and why a test is worth more here than a comment.

use std::path::{Path, PathBuf};

fn workflow() -> String {
    let path: PathBuf = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager has a parent")
        .join(".github/workflows/benchmarks.yml");
    std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("{} must be readable: {e}", path.display()))
}

/// The history filename the workflow declares, so these tests follow a rename.
fn bench_history_name(wf: &str) -> String {
    let line = wf
        .lines()
        .find(|l| l.trim_start().starts_with("BENCH_HISTORY:"))
        .expect("benchmarks.yml must define BENCH_HISTORY");
    line.split_once(':')
        .expect("BENCH_HISTORY: <name>")
        .1
        .trim()
        .to_string()
}

#[test]
fn the_baseline_is_not_entrusted_to_the_actions_cache() {
    let wf = workflow();
    let history = bench_history_name(&wf);

    // Every `uses: actions/cache...` block, with the lines that follow it, so a
    // `path:` naming the history file is visible.
    let mut offenders = Vec::new();
    let lines: Vec<&str> = wf.lines().collect();
    for (i, line) in lines.iter().enumerate() {
        if !line.contains("actions/cache") {
            continue;
        }
        let window = lines[i..(i + 8).min(lines.len())].join("\n");
        if window.contains(&history) || window.contains("BENCH_HISTORY") {
            offenders.push(format!("line {}: {}", i + 1, line.trim()));
        }
    }

    assert!(
        offenders.is_empty(),
        "the rolling baseline is back in the Actions cache:\n  {}\n\n\
         It cannot survive there. The cache is a 10 GB LRU pool shared with ~20 \n\
         cargo caches of 300 MB-1 GB; a 4 KB file read by one job on one branch \n\
         is evicted before the next trunk run reads it. The save succeeds and \n\
         the gate then reports `rolling window of 0 run(s)` with `N/A (no \n\
         baseline)` on every metric — green, and comparing nothing.",
        offenders.join("\n  ")
    );
}

#[test]
fn the_baseline_is_restored_from_a_previous_runs_artifact() {
    let wf = workflow();
    let history = bench_history_name(&wf);

    assert!(
        wf.contains("gh run download"),
        "nothing restores the rolling baseline from a previous run's artifact. \
         Without a restore the window is empty on every run and the gate has \
         nothing to compare against."
    );

    // The restore reads the artifact this job uploads; if the upload stops
    // carrying the history file, the restore silently recovers nothing.
    let upload = wf
        .find("upload-artifact")
        .expect("benchmarks.yml must upload its results");
    let tail = &wf[upload..];
    assert!(
        tail.contains(&history) || tail.contains("BENCH_HISTORY"),
        "the uploaded artifact no longer contains {history}, so the next trunk \
         run's restore will find nothing and the window will stay empty."
    );
}

#[test]
fn the_restore_never_reuses_the_current_run() {
    let wf = workflow();
    assert!(
        wf.contains("github.run_id") && wf.contains("select(.databaseId !="),
        "the restore must exclude the current run id. Re-running a workflow run \
         restores that run's own uploaded artifact, which freezes the window at \
         whatever it already held instead of advancing it."
    );
}

/// A trunk run that FAILED the gate still recorded the regression that failed
/// it, and the next run has to inherit that.
///
/// The cache version got this right — it saved `if: always()`, because
/// "a regression that lands on main is a regression the baseline has to know
/// about; otherwise every subsequent build fails for the same reason and the
/// gate gets disabled". Moving the baseline into an artifact makes it easy to
/// lose that by asking `gh run list` only for successful runs, which is exactly
/// what the first draft of this change did.
#[test]
fn the_restore_inherits_from_failed_trunk_runs_too() {
    let wf = workflow();
    let restore = wf
        .split("- name: Restore rolling baseline")
        .nth(1)
        .expect("there is a restore step")
        .split("\n    - name:")
        .next()
        .expect("the step ends at the next one");

    assert!(
        !restore.contains("--status success"),
        "the restore asks only for successful trunk runs. A run that failed the \
         gate holds the recorded regression; skipping it means the next run \
         compares against a pre-regression baseline and fails again for the same \
         reason, which is how a gate gets switched off."
    );
    assert!(
        restore.contains("failure"),
        "the restore must consider trunk runs that failed the gate, not just \
         green ones."
    );
    assert!(
        restore.contains("for PREV in"),
        "the restore must walk candidates rather than taking only the newest: a \
         run can complete without producing a usable history, and that has to \
         fall through to an older run instead of silently emptying the window."
    );
}

/// The gate ships a flag whose whole purpose is to catch the failure that
/// happened, and CI never passed it.
///
/// `regression_gate --help` calls `--require-baseline` "the wiring check": it
/// exits non-zero when the history file holds no runs. Four trunk runs gated
/// nothing without going red because that switch was never armed.
#[test]
fn trunk_runs_require_a_non_empty_baseline() {
    let wf = workflow();
    assert!(
        wf.contains("--require-baseline"),
        "nothing passes --require-baseline, so an empty window stays green. \
         That is the exact state four trunk runs shipped in."
    );

    // Trunk only: on a pull request this would fail a contributor for an
    // infrastructure problem they did not cause.
    //
    // Matched on the assignment, not on any line mentioning the flag -- the
    // comment above it names the flag too, and an earlier version of this test
    // matched that comment and failed.
    let assignment = wf
        .lines()
        .find(|l| l.contains("TRUNK=") && l.contains("--require-baseline"))
        .expect(
            "--require-baseline must be set on the TRUNK variable, which is \
             what gates it to pushes on main",
        );
    assert!(
        assignment.contains("--trunk"),
        "--require-baseline belongs on the trunk invocation, alongside --trunk \
         and --record, not on the pull-request path. Got: {assignment}"
    );
}

#[test]
fn reading_a_previous_artifact_needs_only_read_permissions() {
    let wf = workflow();
    let permissions = wf
        .split("permissions:")
        .nth(1)
        .expect("benchmarks.yml declares permissions")
        .split("\njobs:")
        .next()
        .expect("permissions block ends before jobs");

    assert!(
        permissions.contains("actions: read"),
        "`gh run download` against a previous run needs `actions: read`. \
         Without it the restore fails and the window silently stays empty. \
         Got:\n{permissions}"
    );
    assert!(
        !permissions.contains("contents: write"),
        "carrying the baseline in an artifact is what avoids granting this \
         workflow write access to the repository. If `contents: write` is being \
         added, that is a separate decision and not something this fix needs. \
         Got:\n{permissions}"
    );
}
