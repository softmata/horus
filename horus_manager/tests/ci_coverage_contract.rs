//! A test that every job skips is a test that does not exist.
//!
//! `topic_cross_thread_1p_multi_c_spmc`, `topic_cross_thread_multi_p_multi_c_mpmc`
//! and `topic_cross_thread_mpmc_pre_initialized_99_percent` are plain `#[test]`
//! functions in `horus_core`'s lib target — no `#[ignore]`, nothing conditional.
//! Six of the CI invocations that build that target passed `--skip` for all
//! three (`ci.yml`, `coverage.yml`, `safety.yml` twice, `multi-platform.yml`
//! twice, `integration-tests.yml`) and the remaining two filter by name for
//! something else. The `--test '*'` jobs build integration targets only, so
//! they never reached them either. Net runners: zero, on every platform.
//!
//! The rationale that travelled with the skip list said the three "assert
//! throughput thresholds ("expected at least 50 messages, got 44") that are a
//! property of the runner, not of the code". That was true when it was written
//! and false by the time it was last copied: all three floors had been deleted,
//! and the comments at those asserts in `communication/topic/tests.rs` record
//! the removal in the past tense. The skips outlived their reason and nothing
//! noticed, because nothing was looking.
//!
//! This file looks. It is deliberately about these three names and not about
//! `--skip` in general: most of the other entries in those lists are
//! load-bearing (sanitizer false positives on the deliberately-lossy `send()`
//! path, tick counts that llvm-cov instrumentation cannot hold), and a job is
//! allowed to exclude a test it genuinely cannot run. What is not allowed is
//! for the last job that could run one to drop it silently.

use std::fs;
use std::path::PathBuf;

/// Full libtest paths, not bare function names, because that is what both
/// `--skip` patterns and positional filters are matched against.
///
/// Named rather than globbed as `topic_cross_thread_*`: a new test in that
/// family is not automatically owed a gate, and a glob would make this file
/// pass or fail for reasons nobody chose.
const MUST_RUN_SOMEWHERE: [&str; 3] = [
    "communication::topic::tests::topic_cross_thread_1p_multi_c_spmc",
    "communication::topic::tests::topic_cross_thread_multi_p_multi_c_mpmc",
    "communication::topic::tests::topic_cross_thread_mpmc_pre_initialized_99_percent",
];

/// Cargo flags whose value is a separate token. Needed only to tell a value
/// (`horus_core` in `-p horus_core`) from a bare test-name filter
/// (`source_resolver` in `cargo test -p horus_manager --lib source_resolver`),
/// which changes what a command runs.
const CARGO_FLAGS_TAKING_A_VALUE: [&str; 15] = [
    "-p",
    "--package",
    "--exclude",
    "--target",
    "--features",
    "-F",
    "--test",
    "--bench",
    "--example",
    "--bin",
    "--manifest-path",
    "--target-dir",
    "--profile",
    "--jobs",
    "-j",
];

/// The same thing for libtest, applied only after `--`. Disjoint from the list
/// above on purpose: `--test` is a cargo flag naming a target and takes a
/// value, while libtest has no `--test` at all, and `--test-threads` is the
/// reverse. `--skip` is deliberately absent — it is matched earlier so its
/// value is captured rather than discarded.
///
/// Every `--test-threads` in these workflows is written `--test-threads=1`
/// today, which needs no entry here because the value rides along in the token.
/// The space-separated spelling is equally valid and is what this guards: left
/// unhandled, `-- --test-threads 1` files `1` as a positional filter, and a
/// filter of `1` selects `topic_cross_thread_1p_multi_c_spmc` while excluding
/// `topic_cross_thread_mpmc_pre_initialized_99_percent`, which contains no `1`.
const LIBTEST_FLAGS_TAKING_A_VALUE: [&str; 6] = [
    "--test-threads",
    "--logfile",
    "--format",
    "--color",
    "--shuffle-seed",
    "-Z",
];

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager has a parent")
        .to_path_buf()
}

fn workflows() -> Vec<(String, String)> {
    let dir = repo_root().join(".github/workflows");
    let mut out = Vec::new();
    for entry in fs::read_dir(&dir).expect(".github/workflows must exist") {
        let path = entry.expect("dir entry").path();
        if path.extension().and_then(|e| e.to_str()) == Some("yml") {
            let name = path.file_name().unwrap().to_string_lossy().to_string();
            out.push((name, fs::read_to_string(&path).expect("workflow readable")));
        }
    }
    out.sort();
    assert!(!out.is_empty(), "no workflows found — this test is vacuous");
    out
}

/// Collapse `\`-continued shell lines into one logical command each, drop
/// whole-line `#` comments, and unwrap the one-line `run:` form.
///
/// All three matter. Every `--skip` in these workflows sits on a continuation
/// line, so reading physical lines sees the `cargo test` and its exclusions as
/// unrelated strings. `docs-contract.yml` discusses `cargo test --workspace
/// --lib` inside a prose comment — counting that as a job would make this file
/// pass while nothing actually ran. And a step written `run: cargo test ...`
/// instead of `run: |` is the same invocation.
fn shell_commands(body: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut acc = String::new();
    for line in body.lines() {
        let mut t = line.trim();
        if t.starts_with('#') {
            continue;
        }
        if acc.is_empty() {
            t = t.strip_prefix("run: ").unwrap_or(t);
        }
        if let Some(head) = t.strip_suffix('\\') {
            acc.push_str(head.trim_end());
            acc.push(' ');
        } else {
            acc.push_str(t);
            out.push(std::mem::take(&mut acc));
        }
    }
    if !acc.is_empty() {
        out.push(acc);
    }
    out
}

/// The name filters and `--skip` patterns a command hands to libtest.
///
/// Filters may appear on either side of `--` — cargo forwards its own
/// positional to the harness — so both halves are parsed. `--skip` only ever
/// appears after it.
struct HarnessArgs {
    filters: Vec<String>,
    skips: Vec<String>,
    exact: bool,
}

impl HarnessArgs {
    /// libtest matches both filters and `--skip` patterns as substrings of the
    /// full test path — `--exact` narrows that to equality — and running with
    /// no filter runs everything.
    fn runs(&self, test_path: &str) -> bool {
        let matches = |p: &String| {
            if self.exact {
                p == test_path
            } else {
                test_path.contains(p.as_str())
            }
        };
        let selected = self.filters.is_empty() || self.filters.iter().any(&matches);
        selected && !self.skips.iter().any(&matches)
    }
}

fn harness_args(cmd: &str) -> HarnessArgs {
    let mut filters = Vec::new();
    let mut skips = Vec::new();
    let exact = cmd.split_whitespace().any(|a| a == "--exact");
    let mut args = cmd.split_whitespace().peekable();
    let mut after_dashdash = false;

    // `cargo`, an optional `+toolchain`, then the subcommand(s): `test`,
    // `llvm-cov`, `miri test`. None of these is a filter.
    while let Some(&head) = args.peek() {
        if head.starts_with('-') {
            break;
        }
        args.next();
        if head == "test" {
            break;
        }
        if head == "llvm-cov" {
            // `cargo llvm-cov` with no subcommand implies `test`, so the
            // subcommand is optional and can only be consumed conditionally.
            // Left in place it parses as a positional filter: `cargo llvm-cov
            // test --lib` would run only tests whose path contains `test`.
            if matches!(args.peek(), Some(&"test") | Some(&"nextest")) {
                args.next();
            }
            break;
        }
    }

    while let Some(arg) = args.next() {
        if arg == "--" {
            after_dashdash = true;
        } else if let Some(pattern) = arg.strip_prefix("--skip=") {
            skips.push(pattern.to_string());
        } else if arg == "--skip" {
            skips.extend(args.next().map(str::to_string));
        } else if arg.starts_with('-') {
            let takes_a_separate_value = if after_dashdash {
                LIBTEST_FLAGS_TAKING_A_VALUE.contains(&arg)
            } else {
                CARGO_FLAGS_TAKING_A_VALUE.contains(&arg)
            };
            if takes_a_separate_value {
                args.next();
            }
        } else {
            filters.push(arg.trim_matches('\'').to_string());
        }
    }

    HarnessArgs {
        filters,
        skips,
        exact,
    }
}

/// Every invocation that builds `horus_core`'s lib test target, as
/// `(workflow, command)`.
///
/// `cargo llvm-cov --workspace` in `coverage.yml` also runs that target but
/// carries no `--lib`, so it is not counted. Under-counting is the safe
/// direction: it can only make the assertion below stricter.
fn horus_core_lib_test_commands() -> Vec<(String, String)> {
    let mut out = Vec::new();
    for (name, body) in workflows() {
        for cmd in shell_commands(&body) {
            let is_cargo_test =
                cmd.starts_with("cargo ") && (cmd.contains(" test ") || cmd.contains(" llvm-cov "));
            let builds_horus_core = cmd.contains("-p horus_core") || cmd.contains("--workspace");
            if is_cargo_test
                && builds_horus_core
                && cmd.contains("--lib")
                && !cmd.contains("--exclude horus_core")
            {
                out.push((name.clone(), cmd));
            }
        }
    }
    out
}

#[test]
fn every_cross_thread_topic_test_still_has_a_ci_runner() {
    let cmds = horus_core_lib_test_commands();

    // The failure guarded against is "nothing runs it", which a matcher that
    // has stopped matching reports as success. Eight invocations parse today;
    // if this trips, fix the matcher rather than deleting the file.
    assert!(
        cmds.len() >= 6,
        "only {} invocation(s) of horus_core's lib test target found across \
         .github/workflows — the matcher has drifted and this test is vacuous",
        cmds.len()
    );

    for path in MUST_RUN_SOMEWHERE {
        assert!(
            cmds.iter().any(|(_, cmd)| harness_args(cmd).runs(path)),
            "no CI job runs {path}: all {} invocations that build horus_core's \
             lib target either --skip it or filter it out by name. A test \
             nothing executes is not coverage — give it a runner, or delete it, \
             but do not leave it looking like a gate.",
            cmds.len()
        );
    }
}

#[test]
fn the_pr_gate_is_one_of_those_runners() {
    // The test above is satisfied by any job, the Windows one included, which
    // skips these three for reasons of its own (an open fan-out gap and an open
    // multi-producer data-loss defect, both recorded in multi-platform.yml).
    // Linux is where the ring protocol is actually exercised, so pin the gate
    // every pull request has to pass.
    let ci = fs::read_to_string(repo_root().join(".github/workflows/ci.yml"))
        .expect("ci.yml must exist");
    let cmd = shell_commands(&ci)
        .into_iter()
        .find(|c| c.starts_with("cargo test --workspace") && c.contains("--lib"))
        .expect("ci.yml's test job must run `cargo test --workspace ... --lib`");
    let args = harness_args(&cmd);

    for path in MUST_RUN_SOMEWHERE {
        assert!(
            args.runs(path),
            "ci.yml's workspace lib step does not run {path}. It is the only \
             job that runs the cross-thread topic tests; excluding it here \
             takes them back to zero runners.\n  {cmd}"
        );
    }
}

// ---------------------------------------------------------------------------
// Unit coverage for the two matchers above.
//
// Both assertions in this file reduce to "some parsed command runs this test
// path", so every way the parsers can misread a command surfaces there as a
// bare pass or fail with no indication of which. One direction of that is
// silent: a `harness_args` that stopped recognising `--skip` yields an empty
// `skips`, `runs()` then answers true for everything, and the contract passes
// while nothing executes. The `cmds.len() >= 6` guard does not catch it — that
// checks only that commands were *found*, not that they were read correctly.
//
// So pin the readings directly, including the spellings CI does not use today
// but is free to adopt tomorrow.
// ---------------------------------------------------------------------------

#[test]
fn shell_commands_joins_continuations_and_drops_prose() {
    const BLOCK: &str = r#"
      - name: Run Tests
        run: |
          # cargo test --workspace --lib, named here only in prose
          cargo test --workspace --exclude horus_py --lib --no-fail-fast -- \
            --test-threads=1 \
            --skip test_robotics_autonomous_car_perception
      - name: The one-line form
        run: cargo test -p horus_core --lib -- --skip something
"#;

    let cmds = shell_commands(BLOCK);

    // Every `--skip` in these workflows sits on a continuation line. Read as
    // physical lines, the `cargo test` and its exclusions are unrelated
    // strings and the contract cannot see either.
    assert!(
        cmds.iter().any(|c| c.as_str()
            == "cargo test --workspace --exclude horus_py --lib --no-fail-fast \
                -- --test-threads=1 --skip test_robotics_autonomous_car_perception"),
        "continuation lines were not joined into one command: {cmds:#?}"
    );

    assert!(
        cmds.iter()
            .any(|c| c.as_str() == "cargo test -p horus_core --lib -- --skip something"),
        "the one-line `run:` form was not unwrapped: {cmds:#?}"
    );

    // docs-contract.yml discusses `cargo test --workspace --lib` in a comment.
    // Counting that as a job makes this file pass while nothing runs.
    assert!(
        !cmds.iter().any(|c| c.contains("named here only in prose")),
        "a `#` comment was collected as a command: {cmds:#?}"
    );
}

#[test]
fn harness_args_reads_the_spellings_ci_can_use() {
    let spmc = MUST_RUN_SOMEWHERE[0];
    let mpmc = MUST_RUN_SOMEWHERE[1];
    let pre_init = MUST_RUN_SOMEWHERE[2];

    // ci.yml's gate, as it stands after this change.
    let gate = harness_args(
        "cargo test --workspace --exclude horus_py --lib --no-fail-fast -- \
         --test-threads=1 --skip test_robotics_autonomous_car_perception",
    );
    assert!(
        gate.filters.is_empty(),
        "`horus_py` (the value of --exclude) leaked in as a filter: {:?}",
        gate.filters
    );
    assert_eq!(gate.skips, ["test_robotics_autonomous_car_perception"]);
    assert!(gate.runs(spmc) && gate.runs(mpmc) && gate.runs(pre_init));

    // `--skip` in both spellings, and a pattern that matches by substring.
    let skipped = harness_args(
        "cargo test --no-default-features -p horus_core --lib -- \
         --skip topic_cross_thread_1p_multi_c_spmc --skip=topic_cross_thread_multi_p_multi_c_mpmc",
    );
    assert!(
        !skipped.runs(spmc),
        "the space-separated --skip was dropped"
    );
    assert!(!skipped.runs(mpmc), "the --skip= form was dropped");
    assert!(skipped.runs(pre_init));

    // The space-separated `--test-threads`. libtest accepts it, and without the
    // value being swallowed `1` becomes a positional filter — which selects
    // `..._1p_multi_c_spmc` and silently drops `..._pre_initialized_99_percent`.
    let spaced = harness_args("cargo test -p horus_core --lib -- --test-threads 1");
    assert!(
        spaced.filters.is_empty(),
        "`1`, the value of --test-threads, was filed as a test-name filter: {:?}",
        spaced.filters
    );
    assert!(spaced.runs(spmc) && spaced.runs(mpmc) && spaced.runs(pre_init));

    // `cargo llvm-cov test` — the explicit spelling of coverage.yml's bare
    // `cargo llvm-cov`. `test` is a subcommand, not a filter.
    let cov = harness_args("cargo llvm-cov test --workspace --lib -- --test-threads=1");
    assert!(
        cov.filters.is_empty(),
        "the `test` subcommand was parsed as a test-name filter: {:?}",
        cov.filters
    );
    assert!(cov.runs(spmc) && cov.runs(mpmc) && cov.runs(pre_init));

    // The bare form still parses, and `+toolchain` / `miri` are not filters.
    let bare = harness_args("cargo llvm-cov --workspace --lib -- --skip topic_cross_thread");
    assert!(bare.filters.is_empty() && !bare.runs(spmc));
    let miri = harness_args("cargo +nightly miri test -p horus_core --lib");
    assert!(miri.filters.is_empty() && miri.runs(spmc));

    // A real positional filter must still be read as one — that is the whole
    // reason cargo's value-taking flags are enumerated.
    let named = harness_args("cargo test -p horus_manager --lib source_resolver -- --ignored");
    assert_eq!(named.filters, ["source_resolver"]);
    assert!(!named.runs(spmc));

    // `--exact` turns substring matching into equality, for filters and skips.
    let exact = harness_args("cargo test -p horus_core --lib -- --exact topic_cross_thread");
    assert!(
        !exact.runs(spmc),
        "--exact was ignored: a bare stem matched a full path"
    );
    let exact_hit = harness_args(&format!("cargo test -p horus_core --lib -- --exact {spmc}"));
    assert!(exact_hit.runs(spmc) && !exact_hit.runs(mpmc));
}
