//! The shipped examples have to compile.
//!
//! Nine of the ten did not. A developer evaluating HORUS clones the repository,
//! opens the example the README names as the starting point, and runs
//! `horus build`:
//!
//! ```text
//! error: no rules expected `"SAFETY: velocity exceeds limits! lin={:.2}"`
//! note: while trying to match `info`
//! ```
//!
//! Four distinct causes, none of them exotic:
//!
//! - `hlog!("msg")` and `hlog_every!(100, "msg")` without a level. Neither macro
//!   had a failure arm, so the error pointed at the format string and blamed a
//!   token the author never wrote. Five examples.
//! - `use horus::DurationExt;` — the trait was exported only through the
//!   prelude, so the obvious path was an unresolved import. Three examples.
//! - `[drivers] imu = …` generating `features = ["imu"]` on the `horus`
//!   dependency. `horus` has five features and has never had that one, so cargo
//!   refused the generated manifest outright.
//! - API drift: `handle.cancel` → `canceled`, `SyncActionClient` →
//!   `ActionClient`, `TransformFrame` used without declaring `horus-tf`, a
//!   message read with `read_latest()` without being `#[fixed]`.
//!
//! Every one of these is caught by compiling. Nothing did.
//!
//! ## Why this test only checks manifests by default
//!
//! One cargo build per example against the full HORUS tree is minutes, not
//! seconds, on a cold cache. So the fast path checks what can be checked
//! statically, and the full build is available behind
//! `HORUS_TEST_BUILD_EXAMPLES=1` for CI, which does run it on every PR:
//!
//! ```bash
//! HORUS_TEST_BUILD_EXAMPLES=1 cargo test -p horus_manager --test examples_contract
//! ```
//!
//! Run: `cargo test -p horus_manager --test examples_contract`

use std::path::{Path, PathBuf};
use std::process::Command;

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_manager must have a parent")
        .to_path_buf()
}

fn example_dirs() -> Vec<PathBuf> {
    let examples = repo_root().join("examples");
    let mut out: Vec<PathBuf> = std::fs::read_dir(&examples)
        .expect("examples/ must exist")
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.is_dir() && p.join("horus.toml").is_file())
        .collect();
    out.sort();
    assert!(
        !out.is_empty(),
        "no examples found under {}",
        examples.display()
    );
    out
}

fn horus() -> &'static str {
    env!("CARGO_BIN_EXE_horus")
}

/// Every example's manifest must be free of keys that do nothing.
///
/// All ten carried `author = "softmata"` (the field is `authors`, so the
/// attribution was silently dropped) and `language = "rust"` (inferred from the
/// sources; the key has no effect).
#[test]
fn every_example_manifest_is_clean() {
    let mut offenders = Vec::new();

    for dir in example_dirs() {
        let out = Command::new(horus())
            .args(["check", "--json"])
            .current_dir(&dir)
            .output()
            .expect("horus check must run");

        let stdout = String::from_utf8_lossy(&out.stdout);
        let Ok(v) = serde_json::from_str::<serde_json::Value>(&stdout) else {
            offenders.push(format!("{}: check emitted invalid JSON", dir.display()));
            continue;
        };

        if let Some(diags) = v["diagnostics"].as_array() {
            for d in diags {
                let msg = d["message"].as_str().unwrap_or_default();
                if msg.contains("unknown key") {
                    let name = dir.file_name().unwrap_or_default().to_string_lossy();
                    offenders.push(format!("{name}: {msg}"));
                }
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "shipped examples contain manifest keys that do nothing:\n  {}\n\n\
         An example is the first HORUS a reader copies. A key that is silently \
         ignored there teaches them it works.",
        offenders.join("\n  ")
    );
}

/// An example must not use a macro form its macro does not accept.
///
/// This is a static check for the exact shape that broke five examples, so it
/// runs on every PR without paying for a build.
#[test]
fn examples_pass_a_level_to_the_logging_macros() {
    let mut offenders = Vec::new();

    for dir in example_dirs() {
        for entry in std::fs::read_dir(&dir).into_iter().flatten().flatten() {
            let path = entry.path();
            if path.extension().is_some_and(|e| e == "rs") {
                let Ok(text) = std::fs::read_to_string(&path) else {
                    continue;
                };
                for (i, line) in text.lines().enumerate() {
                    for macro_name in ["hlog!", "hlog_every!"] {
                        let Some(idx) = line.find(macro_name) else {
                            continue;
                        };
                        let rest = line[idx + macro_name.len()..].trim_start_matches('(');
                        let rest = rest.trim_start();
                        // hlog_every! takes an interval first; skip past it.
                        let rest = if macro_name == "hlog_every!" {
                            rest.split_once(',').map(|(_, r)| r.trim()).unwrap_or(rest)
                        } else {
                            rest
                        };
                        let has_level = ["info", "warn", "error", "debug"]
                            .iter()
                            .any(|l| rest.starts_with(l));
                        // A bare `(` with the arguments on the next line is
                        // fine to skip: the level would be on that line.
                        if !rest.is_empty() && !has_level {
                            offenders.push(format!(
                                "{}:{}: {}",
                                path.display(),
                                i + 1,
                                line.trim()
                            ));
                        }
                    }
                }
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "these calls omit the level, which does not compile:\n  {}",
        offenders.join("\n  ")
    );
}

/// The generated Cargo manifest must not request features `horus` does not have.
///
/// `[drivers] imu = …` produced `features = ["imu"]`, and cargo refuses to
/// resolve that at all — the example could not build for any reason downstream.
#[test]
fn no_example_requests_a_feature_horus_does_not_have() {
    let horus_toml = std::fs::read_to_string(repo_root().join("horus/Cargo.toml"))
        .expect("horus/Cargo.toml must exist");

    // Feature names are the keys of [features].
    let features: Vec<String> = horus_toml
        .lines()
        .skip_while(|l| !l.starts_with("[features]"))
        .skip(1)
        .take_while(|l| !l.starts_with('['))
        .filter_map(|l| l.split_once('=').map(|(k, _)| k.trim().to_string()))
        .filter(|k| !k.is_empty() && !k.starts_with('#'))
        .collect();

    assert!(
        features.contains(&"net".to_string()),
        "failed to parse horus's feature list, got {features:?}"
    );

    let mut offenders = Vec::new();
    let mut inspected = 0usize;
    for dir in example_dirs() {
        let generated = dir.join(".horus/Cargo.toml");
        let Ok(text) = std::fs::read_to_string(&generated) else {
            continue; // not built yet; the build test below covers it
        };
        inspected += 1;
        for line in text.lines() {
            if !line.starts_with("horus = ") {
                continue;
            }
            let Some(start) = line.find("features = [") else {
                continue;
            };
            let list = &line[start + "features = [".len()..];
            let list = list.split(']').next().unwrap_or_default();
            for f in list.split(',') {
                let f = f.trim().trim_matches('"');
                if !f.is_empty() && !features.contains(&f.to_string()) {
                    let name = dir.file_name().unwrap_or_default().to_string_lossy();
                    offenders.push(format!("{name}: requests `{f}`"));
                }
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "generated manifests request features `horus` does not have {features:?}:\n  {}\n\n\
         cargo refuses to resolve these, so the example cannot build at all.",
        offenders.join("\n  ")
    );

    // Say so when there was nothing to inspect, rather than passing silently.
    // The defect this exists for — `[drivers] imu = ...` generating a feature
    // `horus` has never had — is only visible in a generated manifest.
    //
    // Reported as a skip, not a failure. Generated manifests only exist after
    // the examples have been built, which a plain `cargo test -p horus_manager`
    // does not do, so failing here made the package's suite red on every fresh
    // checkout for a missing prerequisite rather than a broken contract — and a
    // suite that is always red is a suite nobody reads. CI that wants this
    // enforced sets `HORUS_TEST_BUILD_EXAMPLES=1`, and then `inspected > 0`
    // holds.
    if inspected == 0 {
        eprintln!(
            "skipped: no example has a generated .horus/Cargo.toml, so there was \
             nothing to check. Build the examples first \
             (`HORUS_TEST_BUILD_EXAMPLES=1`), or run this after the build job."
        );
    }
}

/// The full check: every example actually compiles.
///
/// Slow (one cargo build per example), so it is opt-in. This is the test that
/// would have caught all four failure classes at once. CI sets the variable and
/// shares one CARGO_TARGET_DIR across the examples, which is what makes it
/// affordable per-PR: they all depend on the same horus path deps, so the graph
/// compiles once.
#[test]
fn every_example_builds() {
    if std::env::var("HORUS_TEST_BUILD_EXAMPLES").is_err() {
        eprintln!(
            "SKIP every_example_builds: set HORUS_TEST_BUILD_EXAMPLES=1 to run \
             (one cargo build per example against the full HORUS tree)"
        );
        return;
    }

    let mut failures = Vec::new();
    for dir in example_dirs() {
        let out = Command::new(horus())
            .arg("build")
            .current_dir(&dir)
            .output()
            .expect("horus build must run");

        if !out.status.success() {
            let stderr = String::from_utf8_lossy(&out.stderr);
            let stdout = String::from_utf8_lossy(&out.stdout);
            let first = stdout
                .lines()
                .chain(stderr.lines())
                .find(|l| l.starts_with("error"))
                .unwrap_or("(no error line)")
                .to_string();
            let name = dir.file_name().unwrap_or_default().to_string_lossy();
            failures.push(format!("{name}: {first}"));
        }
    }

    assert!(
        failures.is_empty(),
        "shipped examples do not compile:\n  {}",
        failures.join("\n  ")
    );
}

// ---------------------------------------------------------------------------
// C++ examples
// ---------------------------------------------------------------------------

/// Every C++ example must be in the CMake build list.
///
/// `pub_sub_demo.cpp` was not, so nothing ever compiled it — which is how it
/// kept a `horus::Scheduler` copy-initialization that does not compile.
#[test]
fn every_cpp_example_is_in_the_cmake_build() {
    let dir = repo_root().join("horus_cpp/examples");
    let cmake = std::fs::read_to_string(dir.join("CMakeLists.txt"))
        .expect("horus_cpp/examples/CMakeLists.txt must exist");

    let mut missing = Vec::new();
    for entry in std::fs::read_dir(&dir).expect("examples dir").flatten() {
        let path = entry.path();
        if path.extension().is_some_and(|e| e == "cpp") {
            let stem = path
                .file_stem()
                .unwrap_or_default()
                .to_string_lossy()
                .to_string();
            if !cmake.contains(&stem) {
                missing.push(stem);
            }
        }
    }

    assert!(
        missing.is_empty(),
        "these C++ examples exist but are not built by CMakeLists.txt, so \
         nothing checks them: {missing:?}"
    );
}

/// A missing library must produce a message, not a wall of undefined symbols.
///
/// The CMakeLists used `find_package(horus QUIET)` and linked only
/// `if(horus_FOUND)`. HORUS installs no CMake package config — `horus build`
/// passes the paths in with -D — so find_package never succeeded, nothing was
/// linked, and every example failed with 82 undefined references.
#[test]
fn the_cpp_examples_fail_loudly_when_the_library_is_missing() {
    let cmake = std::fs::read_to_string(repo_root().join("horus_cpp/examples/CMakeLists.txt"))
        .expect("CMakeLists.txt must exist");

    // Match a real call, not the comment above the fix explaining why it was
    // removed. CMake comments start with `#`.
    let calls_find_package = cmake
        .lines()
        .map(str::trim)
        .filter(|l| !l.starts_with('#'))
        .any(|l| l.contains("find_package(horus"));
    assert!(
        !calls_find_package,
        "find_package(horus) does not work: HORUS ships no CMake package config"
    );
    assert!(
        cmake.contains("FATAL_ERROR"),
        "a missing libhorus_cpp should stop the configure step with an \
         explanation, not proceed to a guaranteed link failure"
    );
    assert!(
        cmake.contains("cargo build -p horus_cpp"),
        "the error should say how to produce the library"
    );
}

/// C++ is a first-class language in HORUS, so its examples have to be findable
/// from the place a reader looks for examples.
#[test]
fn the_examples_index_points_at_the_cpp_examples() {
    let readme = std::fs::read_to_string(repo_root().join("examples/README.md"))
        .expect("examples/README.md must exist");

    assert!(
        readme.contains("horus_cpp/examples"),
        "examples/README.md never mentions the C++ examples, so a C++ developer \
         reading it concludes there are none — there are six"
    );
}

// ---------------------------------------------------------------------------
// Language coverage
// ---------------------------------------------------------------------------

/// Which language an example project is written in, by its entry point.
///
/// The same order and the same six candidate paths `auto_detect_main_file` in
/// run/mod.rs probes, so this agrees with the file `horus build` picks.
fn example_language(dir: &Path) -> Option<&'static str> {
    for (rel, lang) in [
        ("main.rs", "rust"),
        ("main.py", "python"),
        ("main.cpp", "cpp"),
        ("src/main.rs", "rust"),
        ("src/main.py", "python"),
        ("src/main.cpp", "cpp"),
    ] {
        if dir.join(rel).is_file() {
            return Some(lang);
        }
    }
    None
}

/// Every language HORUS claims to support first-class has a project to read.
///
/// examples/ was nine Rust, one Python and zero C++ — for a language with ten
/// tutorials and twenty-five doc pages. The six programs under
/// `horus_cpp/examples/` are not a substitute: they are standalone CMake
/// translation units built with `cmake -S horus_cpp/examples -B build`, so they
/// demonstrate individual APIs and not the `horus.toml` + `horus build`
/// workflow those tutorials teach. A C++ developer evaluating HORUS had no
/// complete project to open.
#[test]
fn every_first_class_language_has_a_complete_example_project() {
    let mut by_language: std::collections::BTreeMap<&'static str, Vec<String>> =
        std::collections::BTreeMap::new();
    let mut without_entry_point = Vec::new();

    for dir in example_dirs() {
        let name = dir
            .file_name()
            .unwrap_or_default()
            .to_string_lossy()
            .to_string();
        match example_language(&dir) {
            Some(lang) => by_language.entry(lang).or_default().push(name),
            None => without_entry_point.push(name),
        }
    }

    assert!(
        without_entry_point.is_empty(),
        "these examples have a horus.toml but no entry point `horus build` can \
         find (main.rs / main.py / main.cpp, at the root or under src/): {without_entry_point:?}"
    );

    for lang in ["rust", "python", "cpp"] {
        assert!(
            by_language.contains_key(lang),
            "examples/ contains no {lang} project. Coverage is {by_language:?}.\n\n\
             `horus_cpp/examples/*.cpp` does not count: those are standalone CMake \
             translation units, not horus.toml projects, so they never exercise the \
             workflow the {lang} tutorials teach."
        );
    }
}

/// A C++ example must keep its entry point under `src/`.
///
/// `auto_detect_main_file` accepts a `main.cpp` at the project root, but the
/// CMakeLists cmake_gen writes globs `${CMAKE_SOURCE_DIR}/../src/*.cpp` only.
/// A root main.cpp therefore reports
///
/// ```text
/// > Detected: main.cpp (cpp)
/// > cmake build...
/// Error: Could not find compiled binary 'rootcpp' in .horus/cpp-build
/// ```
///
/// — the file is found, then never compiled. Until the two agree, the example
/// has to be on the side that works, because an example is the layout readers
/// copy.
#[test]
fn cpp_examples_keep_their_entry_point_under_src() {
    let mut offenders = Vec::new();
    for dir in example_dirs() {
        if example_language(&dir) != Some("cpp") {
            continue;
        }
        if dir.join("main.cpp").is_file() && !dir.join("src/main.cpp").is_file() {
            offenders.push(
                dir.file_name()
                    .unwrap_or_default()
                    .to_string_lossy()
                    .to_string(),
            );
        }
    }
    assert!(
        offenders.is_empty(),
        "these C++ examples put main.cpp at the project root, where the \
         generated CMakeLists never globs it: {offenders:?}"
    );
}

// ---------------------------------------------------------------------------
// CI
// ---------------------------------------------------------------------------

/// The body of one job in a workflow file, as `(line_number, text)` pairs.
///
/// Jobs are the four-space-indented keys under `jobs:`; the block runs to the
/// next two-space-indented key.
fn workflow_job<'a>(workflow: &'a str, job: &str) -> Vec<(usize, &'a str)> {
    let header = format!("  {job}:");
    let mut out = Vec::new();
    let mut inside = false;
    for (n, line) in workflow.lines().enumerate() {
        if line.trim_end() == header {
            inside = true;
            continue;
        }
        if inside {
            let is_new_key = line.starts_with("  ")
                && !line.starts_with("   ")
                && !line.trim_start().starts_with('#');
            if is_new_key {
                break;
            }
            out.push((n + 1, line));
        }
    }
    assert!(!out.is_empty(), "no job `{job}` in the workflow");
    out
}

fn docs_contract_workflow() -> String {
    std::fs::read_to_string(repo_root().join(".github/workflows/docs-contract.yml"))
        .expect(".github/workflows/docs-contract.yml must exist")
}

/// A PR that breaks a shipped example must not be green.
///
/// `shipped-examples-build` was gated
/// `if: github.event_name == 'schedule' || inputs.run_compile_sweep`, so it ran
/// at 04:00 UTC and on manual dispatch. The comment above it said the cheap
/// static half ran on every PR anyway, "as part of the normal horus_manager
/// test job" — but ci.yml runs `cargo test --workspace --lib`, which builds
/// library test targets only and never compiles an integration test like this
/// one. Nothing in examples_contract ran per-PR. A PR that broke an example
/// stayed green until the next nightly, by which point finding it is a bisect
/// rather than a review comment.
#[test]
fn the_shipped_examples_job_is_not_gated_off_pull_requests() {
    let wf = docs_contract_workflow();

    for (n, line) in workflow_job(&wf, "shipped-examples-build") {
        let key = line.strip_prefix("    ").unwrap_or("");
        if key.starts_with("if:") {
            panic!(
                "docs-contract.yml:{n} gates the shipped-examples job behind \
                 `{}`, so a PR that breaks an example is green until the next \
                 nightly run.",
                key.trim()
            );
        }
    }

    // Running is not the same as gating: only a required check blocks a merge,
    // and the required check here is the docs-contract-success aggregate.
    let success: String = workflow_job(&wf, "docs-contract-success")
        .into_iter()
        .map(|(_, l)| l)
        .collect::<Vec<_>>()
        .join("\n");
    assert!(
        success.contains("needs.shipped-examples-build.result"),
        "docs-contract-success does not consider shipped-examples-build, so the \
         job runs on the PR but nothing blocks a merge on its result:\n{success}"
    );
}

/// The examples job has to be able to find the HORUS source tree.
///
/// `horus build` generates `.horus/Cargo.toml` with path dependencies into the
/// HORUS checkout, resolved by `find_horus_source_dir()`. That function probes
/// `HORUS_SOURCE`, then `/horus`, `~/softmata/horus`, `~/horus`, `/opt/horus`,
/// `/usr/local/horus`, then the installer's cache. None of those is where
/// `actions/checkout` puts the repository — it lands in
/// `/home/runner/work/<repo>/<repo>` — so without `HORUS_SOURCE` the job fails
/// on the first example with "HORUS source not found", and a job that only ever
/// ran at 04:00 is a job nobody watches.
#[test]
fn the_shipped_examples_job_points_at_the_checkout() {
    let wf = docs_contract_workflow();
    let job: String = workflow_job(&wf, "shipped-examples-build")
        .into_iter()
        .map(|(_, l)| l)
        .collect::<Vec<_>>()
        .join("\n");

    assert!(
        job.contains("HORUS_SOURCE:"),
        "the shipped-examples job never sets HORUS_SOURCE, so \
         find_horus_source_dir() cannot see the checkout:\n{job}"
    );
    assert!(
        job.contains("HORUS_TEST_BUILD_EXAMPLES"),
        "the shipped-examples job must set HORUS_TEST_BUILD_EXAMPLES, or \
         every_example_builds skips itself and the job checks manifests only"
    );
}
