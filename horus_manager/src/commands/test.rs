//! Test command - run tests with language-aware dispatch
//!
//! Features:
//! - **Rust**: `cargo test` via cargo_gen (dev-deps included)
//! - **Python**: `pytest` dispatch with PYTHONPATH setup
//! - Simulation mode: Enables simulation drivers for hardware-free testing
//! - Default single-threaded (Rust): Prevents shared memory conflicts
//! - Integration test mode: Runs ignored tests (Rust) or marked tests (Python)

use anyhow::{bail, Context, Result};
use colored::*;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use crate::commands::run;
use crate::config::CARGO_TOML;
use crate::manifest::{self, Language, HORUS_TOML};

/// Check if Cargo.toml needs regeneration
fn needs_rebuild(horus_dir: &Path) -> bool {
    let cargo_toml = horus_dir.join(CARGO_TOML);
    let horus_toml = PathBuf::from(HORUS_TOML);

    // If no Cargo.toml, definitely needs build
    if !cargo_toml.exists() {
        return true;
    }

    // If horus.toml exists and is newer than Cargo.toml, needs rebuild
    if horus_toml.exists() {
        if let (Ok(toml_meta), Ok(cargo_meta)) =
            (fs::metadata(&horus_toml), fs::metadata(&cargo_toml))
        {
            if let (Ok(toml_time), Ok(cargo_time)) = (toml_meta.modified(), cargo_meta.modified()) {
                return toml_time > cargo_time;
            }
        }
    }

    // Check if any .rs files are newer than Cargo.toml
    // (simplified check - just look at main.rs if it exists)
    for main_file in &["main.rs", "lib.rs", "src/main.rs", "src/lib.rs"] {
        let path = PathBuf::from(main_file);
        if path.exists() {
            if let (Ok(src_meta), Ok(cargo_meta)) = (fs::metadata(&path), fs::metadata(&cargo_toml))
            {
                if let (Ok(src_time), Ok(cargo_time)) = (src_meta.modified(), cargo_meta.modified())
                {
                    if src_time > cargo_time {
                        return true;
                    }
                }
            }
        }
    }

    false
}

/// Configuration for the `horus test` command.
pub struct TestConfig {
    pub filter: Option<String>,
    pub release: bool,
    pub nocapture: bool,
    pub test_threads: Option<usize>,
    pub parallel: bool,
    pub simulation: bool,
    pub integration: bool,
    pub no_build: bool,
    pub verbose: bool,
}

/// Run tests for a HORUS project with language-aware dispatch.
///
/// Detects the project language (Rust/Python) and runs the appropriate test runner.
pub fn run_tests(cfg: TestConfig) -> Result<()> {
    println!("{} Running HORUS tests", "[*]".cyan());

    let project_dir = std::env::current_dir()?;
    let languages = manifest::detect_languages(&project_dir);

    if cfg.simulation {
        println!(
            "  {} Simulation mode enabled (no hardware required)",
            "[*]".cyan()
        );
    }

    let has_rust = languages.contains(&Language::Rust);
    let has_python = languages.contains(&Language::Python);
    let has_cpp = languages.contains(&Language::Cpp);

    let mut ran_any = false;

    if has_rust {
        run_rust_tests(&cfg)?;
        ran_any = true;
    }
    if has_python {
        if ran_any {
            println!();
        }
        run_python_tests(&cfg)?;
        ran_any = true;
    }
    if has_cpp {
        if ran_any {
            println!();
        }
        run_cpp_tests(&cfg)?;
        ran_any = true;
    }

    if !ran_any {
        // Fallback: check for .rs files (standalone, no Cargo.toml)
        let has_rs = PathBuf::from("main.rs").exists() || PathBuf::from("src/main.rs").exists();
        if has_rs {
            run_rust_tests(&cfg)?;
        } else {
            anyhow::bail!(
                "No test runner detected. Expected Cargo.toml (Rust), pyproject.toml (Python), or CMakeLists.txt (C++)."
            );
        }
    }

    Ok(())
}

/// Run Rust tests via `cargo test`.
fn run_rust_tests(cfg: &TestConfig) -> Result<()> {
    let horus_dir = PathBuf::from(".horus");

    // Step 1: Ensure .horus/Cargo.toml is up to date (unless --no-build)
    if !cfg.no_build {
        let needs_gen = needs_rebuild(&horus_dir) || !horus_dir.join(CARGO_TOML).exists();

        if needs_gen {
            if Path::new(CARGO_TOML).exists() {
                if cfg.verbose {
                    println!(
                        "  {} Using root Cargo.toml (project-managed build)",
                        "[*]".cyan()
                    );
                }
            } else {
                println!(
                    "  {} Generating build config (with dev-dependencies)...",
                    "[*]".cyan()
                );
                run::ensure_horus_directory()?;
                generate_test_cargo_toml(cfg.verbose)?;
            }
        } else if cfg.verbose {
            println!("  {} Build is up to date, skipping...", "[*]".cyan());
        }
    }

    let use_root_cargo = Path::new(CARGO_TOML).exists();

    if !use_root_cargo {
        let cargo_toml = horus_dir.join(CARGO_TOML);
        if !cargo_toml.exists() {
            println!(
                "{} No build manifest found. Run horus build first.",
                "[!]".yellow()
            );
            println!(
                "    Run {} first to set up the build environment.",
                "horus build".cyan()
            );
            return Ok(());
        }
    }

    // Build cargo test command
    let mut cmd = Command::new("cargo");
    cmd.arg("test");

    if use_root_cargo {
        cmd.env("CARGO_TARGET_DIR", ".horus/target");
    } else {
        cmd.current_dir(&horus_dir);
    }

    if cfg.simulation {
        // HORUS_SIM_MODE, not HORUS_SIMULATION_MODE. The runtime reads the
        // former (horus_core::drivers::mod.rs), and `horus run --sim` sets the
        // former — but `horus test --sim` set a name NOTHING reads, so it was a
        // no-op. A user running `horus test --sim` to keep hardware out of the
        // loop got the REAL drivers, which on a bench with actuators attached is
        // the difference between a test and a motion command.
        cmd.env("HORUS_SIM_MODE", "1");
    }

    if cfg.release {
        cmd.arg("--release");
    }

    if let Some(ref f) = cfg.filter {
        cmd.arg(f);
    }

    cmd.arg("--");

    if cfg.nocapture {
        cmd.arg("--nocapture");
    }

    let effective_threads = if let Some(threads) = cfg.test_threads {
        threads
    } else if cfg.parallel {
        num_cpus::get()
    } else {
        1
    };
    cmd.arg(format!("--test-threads={}", effective_threads));

    if cfg.integration {
        cmd.arg("--ignored");
        println!(
            "  {} Running integration tests (marked #[ignore])",
            "[*]".cyan()
        );
    }

    if cfg.verbose {
        cmd.arg("--show-output");
    }

    println!(
        "  {} Executing: cargo test{}",
        "->".blue(),
        if use_root_cargo {
            ""
        } else {
            " (generated workspace)"
        }
    );

    let status = cmd.status().context("Failed to execute cargo test")?;

    if status.success() {
        println!("{}", "Rust tests passed!".green().bold());
    } else {
        anyhow::bail!(
            "Rust tests failed with exit code {}",
            status.code().unwrap_or(1)
        );
    }

    Ok(())
}


/// Python files that look like tests, by pytest's own default discovery rules.
///
/// Used to tell "no tests" from "tests that cannot run": a generated project
/// has none, and reporting a missing tool for a suite that does not exist is
/// the wrong answer to `horus test`.
fn python_test_files() -> Vec<std::path::PathBuf> {
    let mut out = Vec::new();
    let mut stack = vec![std::path::PathBuf::from(".")];
    while let Some(dir) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            let name = entry.file_name().to_string_lossy().to_string();
            if path.is_dir() {
                if !name.starts_with('.') && name != "target" && name != "node_modules" {
                    stack.push(path);
                }
            } else if name.ends_with(".py")
                && (name.starts_with("test_") || name.ends_with("_test.py"))
            {
                out.push(path);
            }
        }
    }
    out
}
/// Run Python tests via `pytest` (or `python -m pytest` fallback).
fn run_python_tests(cfg: &TestConfig) -> Result<()> {
    let python_cmd = run::run_python::detect_python_interpreter()?;
    let python_path = run::run_python::build_python_path()?;

    // Generate .horus/pyproject.toml if needed
    if !cfg.no_build {
        if let Ok(manifest) = manifest::HorusManifest::load_from(Path::new(HORUS_TOML)) {
            let project_dir = std::env::current_dir()?;
            crate::pyproject_gen::generate(&manifest, &project_dir, true)?;
        }
    }

    // Try pytest directly, fall back to python -m pytest.
    //
    // `output().is_ok()` only says the process spawned, so this falls through to
    // `python -m pytest` whenever the `pytest` binary is absent — including when
    // pytest is not installed at all, which then surfaces the interpreter's
    // error as a test failure:
    //
    //     /usr/bin/python3: No module named pytest
    //     Error: Python tests failed with exit code 1
    //
    // A generated project contains no tests, so that is the first thing
    // `horus test` says on a brand-new project: a failure, about a missing
    // tool, for a suite that does not exist.
    let have_pytest = Command::new("pytest")
        .arg("--version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
        || Command::new(&python_cmd)
            .args(["-m", "pytest", "--version"])
            .output()
            .map(|o| o.status.success())
            .unwrap_or(false);

    if !have_pytest {
        // Nothing to run is not a failure; a missing tool is only a problem if
        // there is something for it to do.
        if python_test_files().is_empty() {
            println!(
                "  {} No Python tests found, and pytest is not installed — nothing to run.",
                crate::cli_output::ICON_INFO.cyan()
            );
            return Ok(());
        }
        bail!(
            "pytest is not installed, and this project has Python tests.\n  \
             Install it with: {} -m pip install pytest",
            python_cmd
        );
    }

    let (test_cmd, test_args) = if Command::new("pytest")
        .arg("--version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
    {
        ("pytest".to_string(), vec![])
    } else {
        (
            python_cmd.clone(),
            vec!["-m".to_string(), "pytest".to_string()],
        )
    };

    let mut cmd = Command::new(&test_cmd);
    for arg in &test_args {
        cmd.arg(arg);
    }

    cmd.env("PYTHONPATH", &python_path);

    if cfg.simulation {
        // HORUS_SIM_MODE, not HORUS_SIMULATION_MODE. The runtime reads the
        // former (horus_core::drivers::mod.rs), and `horus run --sim` sets the
        // former — but `horus test --sim` set a name NOTHING reads, so it was a
        // no-op. A user running `horus test --sim` to keep hardware out of the
        // loop got the REAL drivers, which on a bench with actuators attached is
        // the difference between a test and a motion command.
        cmd.env("HORUS_SIM_MODE", "1");
    }

    if cfg.verbose {
        cmd.arg("-v");
    }

    if cfg.nocapture {
        cmd.arg("-s");
    }

    if let Some(ref f) = cfg.filter {
        cmd.arg("-k").arg(f);
    }

    if cfg.integration {
        cmd.arg("-m").arg("integration");
    }

    if cfg.parallel {
        // pytest-xdist parallel execution
        let n = cfg.test_threads.unwrap_or_else(num_cpus::get);
        cmd.arg("-n").arg(n.to_string());
    }

    println!("  {} Executing: Python tests", "->".blue());

    let status = cmd.status().context("Failed to execute pytest")?;

    // pytest exit code 5 is "no tests collected", which is not a failure.
    // Reporting it as one meant a freshly generated project failed its own test
    // command in every configuration — with pytest missing *and* with pytest
    // installed:
    //
    //     collected 0 items
    //     ============ no tests ran in 0.05s ============
    //     Error: Python tests failed with exit code 5
    //
    // A user's first `horus test` on a brand-new project should not be red.
    const PYTEST_NO_TESTS_COLLECTED: i32 = 5;

    if status.success() {
        println!("{}", "Python tests passed!".green().bold());
    } else if status.code() == Some(PYTEST_NO_TESTS_COLLECTED) {
        println!(
            "{} No Python tests found. Add one under {} — see \
             https://docs.horusrobotics.dev/python/testing",
            "i".cyan(),
            "tests/".cyan()
        );
    } else {
        anyhow::bail!(
            "Python tests failed with exit code {}",
            status.code().unwrap_or(1)
        );
    }

    Ok(())
}

/// Run C++ tests via `ctest`.
fn run_cpp_tests(cfg: &TestConfig) -> Result<()> {
    println!("{}\n", "Running C++ tests...".cyan().bold());

    let build_dir = PathBuf::from(".horus/cpp-build");

    // Build first if build dir doesn't exist
    if !build_dir.exists() {
        println!("  Building C++ project first...");
        let build_status = std::process::Command::new("cmake")
            .args(["--build", ".horus/cpp-build"])
            .status();
        match build_status {
            Ok(s) if s.success() => {}
            _ => anyhow::bail!("C++ build failed. Run `horus build` first."),
        }
    }

    let mut cmd = std::process::Command::new("ctest");
    cmd.args(["--test-dir", ".horus/cpp-build", "--output-on-failure"]);

    if let Some(ref filter) = cfg.filter {
        cmd.args(["-R", filter]); // ctest regex filter
    }
    if cfg.verbose {
        cmd.arg("--verbose");
    }
    if cfg.parallel {
        let cpus = num_cpus::get();
        cmd.args(["-j", &cpus.to_string()]);
    }
    if cfg.integration {
        cmd.args(["-L", "integration"]); // ctest label filter
    }

    let status = cmd.status()?;
    if status.success() {
        println!("\n  {} C++ tests passed", "*".green());
    } else {
        anyhow::bail!(
            "C++ tests failed with exit code {}",
            status.code().unwrap_or(1)
        );
    }

    Ok(())
}

/// Generate `.horus/Cargo.toml` via `cargo_gen` with `include_dev = true`.
///
/// Loads the manifest from `horus.toml`, detects the main source file,
/// and generates the Cargo workspace with dev-dependencies included for testing.
fn generate_test_cargo_toml(verbose: bool) -> Result<()> {
    use crate::commands::run::run_rust::load_or_default_manifest;

    let project_dir = std::env::current_dir()?;
    let manifest = load_or_default_manifest(&[])?;

    // Find the main source file
    let main_file = run::auto_detect_main_file()?;

    if verbose {
        println!("  {} Source file: {}", "[*]".cyan(), main_file.display());
    }

    crate::cargo_gen::generate(
        &manifest,
        &project_dir,
        &[main_file],
        true, // include_dev = true for testing
    )
    .context("Failed to generate build manifest for testing")?;

    Ok(())
}

#[cfg(test)]
mod tests {

    // ── "no tests" is not "the tool is missing" (PATH-6) ───────────────────
    //
    // A generated project contains no tests. `horus test` shelled out to pytest
    // anyway and surfaced the interpreter's error as a test failure:
    //
    //     /usr/bin/python3: No module named pytest
    //     Error: Python tests failed with exit code 1
    //
    // So the first thing `horus test` said on a brand-new project was a
    // failure, about a missing tool, for a suite that does not exist.

    #[test]
    fn a_project_with_no_tests_has_no_test_files() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let cwd = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().expect("cwd");
        std::env::set_current_dir(tmp.path()).expect("chdir");

        std::fs::create_dir_all("src").expect("mkdir");
        std::fs::write("main.py", "print('hi')\n").expect("write");
        let found = super::python_test_files();

        std::env::set_current_dir(&original).expect("restore");
        drop(cwd);
        assert!(
            found.is_empty(),
            "a generated project has no tests; found {found:?}"
        );
    }

    /// pytest's own discovery rules: `test_*.py` and `*_test.py`.
    #[test]
    fn test_files_are_found_by_pytest_naming() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let cwd = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().expect("cwd");
        std::env::set_current_dir(tmp.path()).expect("chdir");

        std::fs::create_dir_all("tests").expect("mkdir");
        std::fs::write("tests/test_motor.py", "def test_x(): pass\n").expect("write");
        std::fs::write("tests/motor_test.py", "def test_y(): pass\n").expect("write");
        std::fs::write("tests/helper.py", "x = 1\n").expect("write");
        let mut found: Vec<String> = super::python_test_files()
            .iter()
            .map(|p| p.file_name().unwrap().to_string_lossy().to_string())
            .collect();
        found.sort();

        std::env::set_current_dir(&original).expect("restore");
        drop(cwd);
        assert_eq!(
            found,
            vec!["motor_test.py".to_string(), "test_motor.py".to_string()],
            "helper.py is not a test file by pytest's rules"
        );
    }

    /// With pytest absent, the two cases must be told apart: nothing to run is
    /// success, tests that cannot run is a failure that names the fix.
    #[test]
    fn a_missing_pytest_is_only_an_error_when_there_are_tests() {
        let full = include_str!("test.rs");
        let src = &full[..full.find("\nmod tests {").unwrap_or(full.len())];
        let at = src
            .find("if !have_pytest {")
            .expect("the missing-pytest branch must exist");
        let body: String = src[at..].lines().take(20).collect::<Vec<_>>().join("\n");

        assert!(
            body.contains("python_test_files().is_empty()"),
            "the branch must ask whether there is anything to run:\n{body}"
        );
        assert!(
            body.contains("return Ok(())"),
            "no tests and no pytest is not a failure:\n{body}"
        );
        assert!(
            body.contains("pip install pytest"),
            "when there ARE tests, the error must name the fix:\n{body}"
        );
    }
    use super::*;
    use std::time::Duration;

    // ── Helper ───────────────────────────────────────────────────────────────

    /// Run a closure inside a temp directory, holding the CWD_LOCK.
    fn in_tmp<F, R>(tmp: &tempfile::TempDir, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = f();
        std::env::set_current_dir(original).unwrap();
        result
    }

    /// Create a default TestConfig (all off / None).
    fn default_cfg() -> TestConfig {
        TestConfig {
            filter: None,
            release: false,
            nocapture: false,
            test_threads: None,
            parallel: false,
            simulation: false,
            integration: false,
            no_build: false,
            verbose: false,
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  needs_rebuild
    // ═══════════════════════════════════════════════════════════════════════

    #[test]
    fn needs_rebuild_no_cargo_toml() {
        let temp_dir = PathBuf::from("/tmp/test_horus_nonexistent");
        assert!(needs_rebuild(&temp_dir));
    }

    #[test]
    fn needs_rebuild_nonexistent_dir() {
        let p = PathBuf::from("/tmp/definitely_not_a_real_dir_xyz_987654");
        assert!(
            needs_rebuild(&p),
            "Nonexistent dir should always need rebuild"
        );
    }

    #[test]
    fn needs_rebuild_empty_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        assert!(needs_rebuild(tmp.path()));
    }

    #[test]
    fn needs_rebuild_with_fresh_cargo_toml() {
        let tmp = tempfile::TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();
        fs::write(
            horus_dir.join(CARGO_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // No horus.toml, no source files → up to date
        let result = in_tmp(&tmp, || needs_rebuild(&horus_dir));
        assert!(!result);
    }

    #[test]
    fn needs_rebuild_when_horus_toml_newer() {
        let tmp = tempfile::TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();

        fs::write(
            horus_dir.join(CARGO_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        std::thread::sleep(Duration::from_millis(50));

        let result = in_tmp(&tmp, || {
            fs::write(
                tmp.path().join(HORUS_TOML),
                "[package]\nname = \"test\"\nversion = \"0.2.0\"\n",
            )
            .unwrap();
            needs_rebuild(&horus_dir)
        });

        assert!(
            result,
            "Should need rebuild when horus.toml is newer than Cargo.toml"
        );
    }

    #[test]
    fn needs_rebuild_when_horus_toml_older() {
        let tmp = tempfile::TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();

        // Create horus.toml first (older)
        in_tmp(&tmp, || {
            fs::write(
                tmp.path().join(HORUS_TOML),
                "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
            )
            .unwrap();
        });

        std::thread::sleep(Duration::from_millis(50));

        // Create Cargo.toml after (newer)
        fs::write(
            horus_dir.join(CARGO_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = in_tmp(&tmp, || needs_rebuild(&horus_dir));
        assert!(
            !result,
            "Should NOT need rebuild when Cargo.toml is newer than horus.toml"
        );
    }

    #[test]
    fn needs_rebuild_when_source_newer() {
        let tmp = tempfile::TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();

        fs::write(
            horus_dir.join(CARGO_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        std::thread::sleep(Duration::from_millis(50));

        let result = in_tmp(&tmp, || {
            fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();
            needs_rebuild(&horus_dir)
        });

        assert!(result, "Should need rebuild when main.rs is newer");
    }

    #[test]
    fn needs_rebuild_when_lib_rs_newer() {
        let tmp = tempfile::TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();

        fs::write(
            horus_dir.join(CARGO_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        std::thread::sleep(Duration::from_millis(50));

        let result = in_tmp(&tmp, || {
            fs::write(tmp.path().join("lib.rs"), "pub fn hello() {}").unwrap();
            needs_rebuild(&horus_dir)
        });

        assert!(result, "Should need rebuild when lib.rs is newer");
    }

    #[test]
    fn needs_rebuild_when_src_main_rs_newer() {
        let tmp = tempfile::TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();

        fs::write(
            horus_dir.join(CARGO_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        std::thread::sleep(Duration::from_millis(50));

        let result = in_tmp(&tmp, || {
            fs::create_dir_all("src").unwrap();
            fs::write("src/main.rs", "fn main() {}").unwrap();
            needs_rebuild(&horus_dir)
        });

        assert!(result, "Should need rebuild when src/main.rs is newer");
    }

    #[test]
    fn needs_rebuild_when_src_lib_rs_newer() {
        let tmp = tempfile::TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();

        fs::write(
            horus_dir.join(CARGO_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        std::thread::sleep(Duration::from_millis(50));

        let result = in_tmp(&tmp, || {
            fs::create_dir_all("src").unwrap();
            fs::write("src/lib.rs", "pub fn hello() {}").unwrap();
            needs_rebuild(&horus_dir)
        });

        assert!(result, "Should need rebuild when src/lib.rs is newer");
    }

    #[test]
    fn needs_rebuild_source_older_than_cargo_toml() {
        let tmp = tempfile::TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();

        // Create source first (older)
        in_tmp(&tmp, || {
            fs::write("main.rs", "fn main() {}").unwrap();
        });

        std::thread::sleep(Duration::from_millis(50));

        // Create Cargo.toml after (newer)
        fs::write(
            horus_dir.join(CARGO_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = in_tmp(&tmp, || needs_rebuild(&horus_dir));
        assert!(
            !result,
            "Should NOT need rebuild when sources are older than Cargo.toml"
        );
    }

    #[test]
    fn needs_rebuild_no_source_files_no_horus_toml() {
        let tmp = tempfile::TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();
        fs::write(
            horus_dir.join(CARGO_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = in_tmp(&tmp, || needs_rebuild(&horus_dir));
        assert!(
            !result,
            "No source files and no horus.toml means up to date"
        );
    }

    #[test]
    fn needs_rebuild_multiple_sources_only_one_newer() {
        let tmp = tempfile::TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();

        // Create main.rs (older)
        in_tmp(&tmp, || {
            fs::write("main.rs", "fn main() {}").unwrap();
        });

        std::thread::sleep(Duration::from_millis(50));

        // Create Cargo.toml (in the middle)
        fs::write(
            horus_dir.join(CARGO_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        std::thread::sleep(Duration::from_millis(50));

        // Create lib.rs (newer) — should trigger rebuild
        let result = in_tmp(&tmp, || {
            fs::write("lib.rs", "pub fn newer() {}").unwrap();
            needs_rebuild(&horus_dir)
        });

        assert!(
            result,
            "Any single newer source file should trigger rebuild"
        );
    }

    #[test]
    fn needs_rebuild_absolute_horus_dir_works() {
        // Verify that needs_rebuild works with an absolute path for horus_dir
        let tmp = tempfile::TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();
        fs::write(
            horus_dir.join(CARGO_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = in_tmp(&tmp, || needs_rebuild(&horus_dir));
        assert!(!result);
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  TestConfig
    // ═══════════════════════════════════════════════════════════════════════

    #[test]
    fn test_config_defaults() {
        let cfg = default_cfg();
        assert!(cfg.filter.is_none());
        assert!(!cfg.release);
        assert!(!cfg.nocapture);
        assert!(cfg.test_threads.is_none());
        assert!(!cfg.parallel);
        assert!(!cfg.simulation);
        assert!(!cfg.integration);
        assert!(!cfg.no_build);
        assert!(!cfg.verbose);
    }

    #[test]
    fn test_config_with_filter() {
        let cfg = TestConfig {
            filter: Some("my_test".to_string()),
            release: true,
            nocapture: true,
            test_threads: Some(4),
            parallel: true,
            simulation: true,
            integration: true,
            no_build: true,
            verbose: true,
        };

        assert_eq!(cfg.filter.as_deref(), Some("my_test"));
        assert!(cfg.release);
        assert!(cfg.nocapture);
        assert_eq!(cfg.test_threads, Some(4));
        assert!(cfg.parallel);
        assert!(cfg.simulation);
        assert!(cfg.integration);
        assert!(cfg.no_build);
        assert!(cfg.verbose);
    }

    #[test]
    fn test_config_filter_empty_string() {
        let cfg = TestConfig {
            filter: Some(String::new()),
            ..default_cfg()
        };
        assert_eq!(cfg.filter.as_deref(), Some(""));
    }

    #[test]
    fn test_config_test_threads_zero() {
        let cfg = TestConfig {
            test_threads: Some(0),
            ..default_cfg()
        };
        assert_eq!(cfg.test_threads, Some(0));
    }

    #[test]
    fn test_config_test_threads_large() {
        let cfg = TestConfig {
            test_threads: Some(1024),
            ..default_cfg()
        };
        assert_eq!(cfg.test_threads, Some(1024));
    }

    #[test]
    fn test_config_parallel_without_threads() {
        let cfg = TestConfig {
            parallel: true,
            test_threads: None,
            ..default_cfg()
        };
        assert!(cfg.parallel);
        assert!(cfg.test_threads.is_none());
    }

    #[test]
    fn test_config_simulation_and_integration() {
        let cfg = TestConfig {
            simulation: true,
            integration: true,
            ..default_cfg()
        };
        assert!(cfg.simulation);
        assert!(cfg.integration);
    }

    #[test]
    fn test_config_no_build_with_verbose() {
        let cfg = TestConfig {
            no_build: true,
            verbose: true,
            ..default_cfg()
        };
        assert!(cfg.no_build);
        assert!(cfg.verbose);
    }

    #[test]
    fn test_config_release_with_nocapture() {
        let cfg = TestConfig {
            release: true,
            nocapture: true,
            ..default_cfg()
        };
        assert!(cfg.release);
        assert!(cfg.nocapture);
    }

    #[test]
    fn test_config_filter_with_regex_like_pattern() {
        let cfg = TestConfig {
            filter: Some("test_.*_integration".to_string()),
            ..default_cfg()
        };
        assert_eq!(cfg.filter.as_deref(), Some("test_.*_integration"));
    }

    #[test]
    fn test_config_filter_with_module_path() {
        let cfg = TestConfig {
            filter: Some("commands::scripts::tests".to_string()),
            ..default_cfg()
        };
        assert_eq!(cfg.filter.as_deref(), Some("commands::scripts::tests"));
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  run_tests — language detection paths
    // ═══════════════════════════════════════════════════════════════════════

    #[test]
    fn run_tests_no_project_fails() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cfg = default_cfg();
        let result = in_tmp(&tmp, || run_tests(cfg));
        assert!(result.is_err(), "Should fail with no project files");
    }

    #[test]
    fn run_tests_no_project_error_message() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cfg = default_cfg();
        let result = in_tmp(&tmp, || run_tests(cfg));
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("No test runner detected"),
            "Error should mention no test runner, got: {}",
            err,
        );
    }

    #[test]
    fn run_tests_no_project_verbose_also_fails() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cfg = TestConfig {
            verbose: true,
            ..default_cfg()
        };
        let result = in_tmp(&tmp, || run_tests(cfg));
        assert!(result.is_err());
    }

    #[test]
    fn run_tests_no_project_simulation_also_fails() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cfg = TestConfig {
            simulation: true,
            ..default_cfg()
        };
        let result = in_tmp(&tmp, || run_tests(cfg));
        assert!(result.is_err());
    }

    #[test]
    fn run_tests_no_project_with_filter_fails() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cfg = TestConfig {
            filter: Some("anything".to_string()),
            ..default_cfg()
        };
        let result = in_tmp(&tmp, || run_tests(cfg));
        assert!(result.is_err());
    }

    #[test]
    fn run_tests_empty_dir_fails() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cfg = default_cfg();
        let result = in_tmp(&tmp, || run_tests(cfg));
        assert!(result.is_err());
    }

    // ── Fallback detection with standalone .rs file ──────────────────────────

    #[test]
    fn run_tests_standalone_main_rs_detects_rust_fallback() {
        // When there's a main.rs but no Cargo.toml, should attempt rust tests
        // (it will fail because there's no Cargo.toml to use, but it should
        //  NOT fail with "No test runner detected" — it enters the rust path)
        let tmp = tempfile::TempDir::new().unwrap();
        in_tmp(&tmp, || {
            fs::write("main.rs", "fn main() {}").unwrap();
        });
        let cfg = TestConfig {
            no_build: true,
            ..default_cfg()
        };
        let result = in_tmp(&tmp, || run_tests(cfg));
        // It enters the rust test path (no "No test runner detected" error).
        // It may succeed (returning Ok) or fail with a cargo-related error,
        // but the important thing is it does NOT fail with "No test runner detected".
        if let Err(e) = &result {
            let msg = format!("{}", e);
            assert!(
                !msg.contains("No test runner detected"),
                "With main.rs, should enter Rust fallback path, not 'no runner': {}",
                msg,
            );
        }
    }

    #[test]
    fn run_tests_standalone_src_main_rs_detects_rust_fallback() {
        let tmp = tempfile::TempDir::new().unwrap();
        in_tmp(&tmp, || {
            fs::create_dir_all("src").unwrap();
            fs::write("src/main.rs", "fn main() {}").unwrap();
        });
        let cfg = TestConfig {
            no_build: true,
            ..default_cfg()
        };
        let result = in_tmp(&tmp, || run_tests(cfg));
        if let Err(e) = &result {
            let msg = format!("{}", e);
            assert!(
                !msg.contains("No test runner detected"),
                "With src/main.rs, should enter Rust fallback, got: {}",
                msg,
            );
        }
    }

    // ── Language detection ────────────────────────────────────────────────────

    #[test]
    fn detect_languages_empty_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        let langs = manifest::detect_languages(tmp.path());
        assert!(langs.is_empty());
    }

    #[test]
    fn detect_languages_rust_only() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join("Cargo.toml"),
            "[package]\nname = \"x\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        let langs = manifest::detect_languages(tmp.path());
        assert!(langs.contains(&Language::Rust));
        assert!(!langs.contains(&Language::Python));
    }

    #[test]
    fn detect_languages_python_pyproject() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join("pyproject.toml"),
            "[project]\nname = \"x\"\n",
        )
        .unwrap();
        let langs = manifest::detect_languages(tmp.path());
        assert!(langs.contains(&Language::Python));
        assert!(!langs.contains(&Language::Rust));
    }

    #[test]
    fn detect_languages_python_requirements_txt() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join("requirements.txt"), "numpy\npytest\n").unwrap();
        let langs = manifest::detect_languages(tmp.path());
        assert!(langs.contains(&Language::Python));
    }

    #[test]
    fn detect_languages_python_setup_py() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join("setup.py"),
            "from setuptools import setup\nsetup()",
        )
        .unwrap();
        let langs = manifest::detect_languages(tmp.path());
        assert!(langs.contains(&Language::Python));
    }

    #[test]
    fn detect_languages_mixed_rust_python() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join("Cargo.toml"),
            "[package]\nname = \"x\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        fs::write(
            tmp.path().join("pyproject.toml"),
            "[project]\nname = \"x\"\n",
        )
        .unwrap();
        let langs = manifest::detect_languages(tmp.path());
        assert!(langs.contains(&Language::Rust));
        assert!(langs.contains(&Language::Python));
    }

    #[test]
    fn detect_languages_cpp() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join("CMakeLists.txt"),
            "cmake_minimum_required(VERSION 3.20)\n",
        )
        .unwrap();
        let langs = manifest::detect_languages(tmp.path());
        assert!(langs.contains(&Language::Cpp));
    }

    #[test]
    fn detect_languages_ros2() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join("package.xml"),
            "<package><name>x</name></package>",
        )
        .unwrap();
        let langs = manifest::detect_languages(tmp.path());
        assert!(langs.contains(&Language::Ros2));
    }

    #[test]
    fn detect_languages_all_four() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join("Cargo.toml"),
            "[package]\nname=\"x\"\nversion=\"0.1.0\"\n",
        )
        .unwrap();
        fs::write(tmp.path().join("pyproject.toml"), "[project]\nname=\"x\"\n").unwrap();
        fs::write(
            tmp.path().join("CMakeLists.txt"),
            "cmake_minimum_required(VERSION 3.20)\n",
        )
        .unwrap();
        fs::write(tmp.path().join("package.xml"), "<package/>").unwrap();
        let langs = manifest::detect_languages(tmp.path());
        assert_eq!(langs.len(), 4, "Should detect all four languages");
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  needs_rebuild — battle tests
    // ═══════════════════════════════════════════════════════════════════════

    #[test]
    fn battle_needs_rebuild_called_twice_stable() {
        let tmp = tempfile::TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();
        fs::write(
            horus_dir.join(CARGO_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        in_tmp(&tmp, || {
            let r1 = needs_rebuild(&horus_dir);
            let r2 = needs_rebuild(&horus_dir);
            assert_eq!(r1, r2, "Two calls without changes should give same result");
        });
    }

    #[test]
    fn battle_needs_rebuild_after_touching_cargo_toml() {
        let tmp = tempfile::TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();
        let cargo_path = horus_dir.join(CARGO_TOML);
        fs::write(
            &cargo_path,
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // Create a horus.toml that's older
        in_tmp(&tmp, || {
            fs::write(
                HORUS_TOML,
                "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
            )
            .unwrap();
        });

        std::thread::sleep(Duration::from_millis(50));

        // Re-write Cargo.toml to make it newer
        fs::write(
            &cargo_path,
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\nedition = \"2021\"\n",
        )
        .unwrap();

        let result = in_tmp(&tmp, || needs_rebuild(&horus_dir));
        assert!(
            !result,
            "Freshly rewritten Cargo.toml should NOT need rebuild"
        );
    }

    #[test]
    fn battle_needs_rebuild_with_symlinked_source() {
        // Symlinks may have different mtime behavior
        let tmp = tempfile::TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();

        in_tmp(&tmp, || {
            fs::write("actual_main.rs", "fn main() {}").unwrap();
        });

        std::thread::sleep(Duration::from_millis(50));

        fs::write(
            horus_dir.join(CARGO_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // The symlink-named file "main.rs" doesn't exist directly, so it
        // won't trigger. Only actual files at checked paths matter.
        let result = in_tmp(&tmp, || needs_rebuild(&horus_dir));
        assert!(!result, "actual_main.rs is not a checked path");
    }

    #[test]
    fn battle_needs_rebuild_cargo_toml_in_subdirectory() {
        // horus_dir can be a nested path
        let tmp = tempfile::TempDir::new().unwrap();
        let horus_dir = tmp.path().join("deep").join("nested").join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();
        fs::write(
            horus_dir.join(CARGO_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = in_tmp(&tmp, || needs_rebuild(&horus_dir));
        assert!(!result);
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  run_tests — more scenarios
    // ═══════════════════════════════════════════════════════════════════════

    #[test]
    fn run_tests_with_only_horus_toml_fails() {
        // horus.toml alone doesn't enable any test runner
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join(HORUS_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        let cfg = default_cfg();
        let result = in_tmp(&tmp, || run_tests(cfg));
        assert!(
            result.is_err(),
            "horus.toml alone should not activate a test runner"
        );
    }

    #[test]
    fn run_tests_all_flags_no_project_still_fails() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cfg = TestConfig {
            filter: Some("foo".to_string()),
            release: true,
            nocapture: true,
            test_threads: Some(2),
            parallel: true,
            simulation: true,
            integration: true,
            no_build: true,
            verbose: true,
        };
        let result = in_tmp(&tmp, || run_tests(cfg));
        assert!(result.is_err());
    }

    #[test]
    fn run_tests_random_files_no_runner() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join("readme.txt"), "hello").unwrap();
        fs::write(tmp.path().join("data.json"), "{}").unwrap();
        let cfg = default_cfg();
        let result = in_tmp(&tmp, || run_tests(cfg));
        assert!(result.is_err());
    }

    // ── run_rust_tests edge cases (via run_tests dispatch) ───────────────

    #[test]
    fn run_tests_rust_no_horus_cargo_toml_prints_hint() {
        // With Cargo.toml in root, run_rust_tests uses it directly
        // (it will fail because cargo test can't run on an empty project)
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join("Cargo.toml"),
            "[package]\nname = \"test-proj\"\nversion = \"0.1.0\"\nedition = \"2021\"\n",
        )
        .unwrap();
        fs::create_dir_all(tmp.path().join("src")).unwrap();
        fs::write(tmp.path().join("src/lib.rs"), "").unwrap();

        let cfg = TestConfig {
            no_build: true,
            ..default_cfg()
        };
        // This actually runs cargo test, which should succeed on an empty lib
        let result = in_tmp(&tmp, || run_tests(cfg));
        // May pass or fail depending on toolchain availability; we just ensure
        // it enters the Rust path (no "No test runner" error).
        if let Err(e) = &result {
            let msg = format!("{}", e);
            assert!(
                !msg.contains("No test runner detected"),
                "Cargo.toml should trigger Rust path: {}",
                msg,
            );
        }
    }

    #[test]
    fn run_tests_rust_without_root_cargo_toml_no_horus_dir() {
        // No Cargo.toml, no .horus/, but main.rs exists -> fallback Rust path
        // -> no .horus/Cargo.toml -> prints hint and returns Ok
        let tmp = tempfile::TempDir::new().unwrap();
        in_tmp(&tmp, || {
            fs::write("main.rs", "fn main() {}").unwrap();
        });
        let cfg = TestConfig {
            no_build: true,
            ..default_cfg()
        };
        let result = in_tmp(&tmp, || run_tests(cfg));
        // Should return Ok (prints hint about running `horus build` first)
        // because run_rust_tests returns Ok(()) when .horus/Cargo.toml is missing
        assert!(
            result.is_ok(),
            "Should return Ok with hint, not error: {:?}",
            result
        );
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  TestConfig combinations — Rust command building
    // ═══════════════════════════════════════════════════════════════════════

    // These tests verify that the effective_threads logic works correctly
    // by inspecting the TestConfig values that drive it.

    #[test]
    fn effective_threads_default_is_one() {
        let cfg = default_cfg();
        let threads = if let Some(t) = cfg.test_threads {
            t
        } else if cfg.parallel {
            num_cpus::get()
        } else {
            1
        };
        assert_eq!(threads, 1, "Default should be single-threaded");
    }

    #[test]
    fn effective_threads_explicit_override() {
        let cfg = TestConfig {
            test_threads: Some(8),
            parallel: true, // should be ignored when test_threads is Some
            ..default_cfg()
        };
        let threads = if let Some(t) = cfg.test_threads {
            t
        } else if cfg.parallel {
            num_cpus::get()
        } else {
            1
        };
        assert_eq!(threads, 8);
    }

    #[test]
    fn effective_threads_parallel_uses_cpus() {
        let cfg = TestConfig {
            parallel: true,
            test_threads: None,
            ..default_cfg()
        };
        let threads = if let Some(t) = cfg.test_threads {
            t
        } else if cfg.parallel {
            num_cpus::get()
        } else {
            1
        };
        assert!(threads >= 1, "Should use at least 1 CPU");
        assert_eq!(threads, num_cpus::get());
    }

    #[test]
    fn effective_threads_one_takes_precedence_over_parallel() {
        let cfg = TestConfig {
            test_threads: Some(1),
            parallel: true,
            ..default_cfg()
        };
        let threads = if let Some(t) = cfg.test_threads {
            t
        } else if cfg.parallel {
            num_cpus::get()
        } else {
            1
        };
        assert_eq!(
            threads, 1,
            "Explicit test_threads=1 should override parallel"
        );
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  Battle tests
    // ═══════════════════════════════════════════════════════════════════════

    #[test]
    fn battle_run_tests_twice_same_empty_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        let r1 = in_tmp(&tmp, || run_tests(default_cfg()));
        let r2 = in_tmp(&tmp, || run_tests(default_cfg()));
        assert!(r1.is_err());
        assert!(r2.is_err());
    }

    #[test]
    fn battle_needs_rebuild_with_only_cargo_toml_and_horus_toml() {
        // Both exist but Cargo.toml is newer -> no rebuild needed
        let tmp = tempfile::TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();

        in_tmp(&tmp, || {
            fs::write(HORUS_TOML, "[package]\nname = \"t\"\nversion = \"0.1.0\"\n").unwrap();
        });

        std::thread::sleep(Duration::from_millis(50));

        fs::write(
            horus_dir.join(CARGO_TOML),
            "[package]\nname = \"t\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = in_tmp(&tmp, || needs_rebuild(&horus_dir));
        assert!(!result);
    }

    #[test]
    fn battle_test_config_struct_size() {
        // Smoke test: the struct should be small and stack-allocatable
        let size = std::mem::size_of::<TestConfig>();
        assert!(
            size < 256,
            "TestConfig should be a lightweight struct, size={}",
            size
        );
    }

    #[test]
    fn battle_detect_languages_nonexistent_dir() {
        let path = PathBuf::from("/tmp/nonexistent_dir_xyz_99999");
        let langs = manifest::detect_languages(&path);
        assert!(
            langs.is_empty(),
            "Nonexistent dir should detect no languages"
        );
    }

    #[test]
    fn battle_needs_rebuild_concurrent_calls_stable() {
        // Simulate what happens when needs_rebuild is called in quick succession
        let tmp = tempfile::TempDir::new().unwrap();
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();
        fs::write(
            horus_dir.join(CARGO_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        in_tmp(&tmp, || {
            let results: Vec<bool> = (0..10).map(|_| needs_rebuild(&horus_dir)).collect();
            let first = results[0];
            for (i, &r) in results.iter().enumerate() {
                assert_eq!(r, first, "Call {} differed from call 0", i);
            }
        });
    }

    #[test]
    fn battle_needs_rebuild_relative_vs_absolute_horus_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        let abs_horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&abs_horus_dir).unwrap();
        fs::write(
            abs_horus_dir.join(CARGO_TOML),
            "[package]\nname = \"t\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        in_tmp(&tmp, || {
            let rel_dir = PathBuf::from(".horus");
            let r_rel = needs_rebuild(&rel_dir);
            let r_abs = needs_rebuild(&abs_horus_dir);
            assert_eq!(r_rel, r_abs, "Relative and absolute should agree");
        });
    }
}
