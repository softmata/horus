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
//! Building ten examples means ten cargo builds against the full HORUS tree,
//! which is minutes, not seconds — too slow for every PR. So the fast path
//! checks what can be checked statically, and the full build is available
//! behind `HORUS_TEST_BUILD_EXAMPLES=1` for CI:
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
/// Slow (ten cargo builds), so it is opt-in. This is the test that would have
/// caught all four failure classes at once.
#[test]
fn every_example_builds() {
    if std::env::var("HORUS_TEST_BUILD_EXAMPLES").is_err() {
        eprintln!(
            "SKIP every_example_builds: set HORUS_TEST_BUILD_EXAMPLES=1 to run \
             (ten cargo builds against the full HORUS tree)"
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
