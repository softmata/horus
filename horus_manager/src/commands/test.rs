//! Test command - run tests with language-aware dispatch
//!
//! Features:
//! - **Rust**: `cargo test` via cargo_gen (dev-deps included)
//! - **Python**: `pytest` dispatch with PYTHONPATH setup
//! - Simulation mode: Enables simulation drivers for hardware-free testing
//! - Default single-threaded (Rust): Prevents shared memory conflicts
//! - Integration test mode: Runs ignored tests (Rust) or marked tests (Python)

use anyhow::{Context, Result};
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

    match (has_rust, has_python) {
        (true, true) => {
            // Mixed project: run both
            println!("  {} Mixed Rust+Python project detected", "[*]".cyan());
            run_rust_tests(&cfg)?;
            println!();
            run_python_tests(&cfg)?;
        }
        (true, false) => run_rust_tests(&cfg)?,
        (false, true) => run_python_tests(&cfg)?,
        (false, false) => {
            // Fallback: check for .rs files (standalone, no Cargo.toml)
            let has_rs = PathBuf::from("main.rs").exists()
                || PathBuf::from("src/main.rs").exists();
            if has_rs {
                run_rust_tests(&cfg)?;
            } else {
                anyhow::bail!(
                    "No test runner detected. Expected Cargo.toml (Rust) or pyproject.toml/requirements.txt (Python)."
                );
            }
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
            println!("{} No .horus/Cargo.toml found.", "[!]".yellow());
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
        cmd.env("HORUS_SIMULATION_MODE", "1");
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
        if use_root_cargo { "" } else { " in .horus/" }
    );

    let status = cmd.status().context("Failed to execute cargo test")?;

    if status.success() {
        println!("{}", "Rust tests passed!".green().bold());
    } else {
        anyhow::bail!("Rust tests failed with exit code {}", status.code().unwrap_or(1));
    }

    Ok(())
}

/// Run Python tests via `pytest` (or `python -m pytest` fallback).
fn run_python_tests(cfg: &TestConfig) -> Result<()> {
    let python_cmd = run::run_python::detect_python_interpreter()?;
    let python_path = run::run_python::build_python_path()?;

    // Generate .horus/pyproject.toml if needed
    if !cfg.no_build {
        if let Ok((manifest, _)) = manifest::HorusManifest::load_from(Path::new(HORUS_TOML)) {
            let project_dir = std::env::current_dir()?;
            crate::pyproject_gen::generate(&manifest, &project_dir, true)?;
        }
    }

    // Try pytest directly, fall back to python -m pytest
    let (test_cmd, test_args) = if Command::new("pytest").arg("--version").output().is_ok() {
        ("pytest".to_string(), vec![])
    } else {
        (python_cmd.clone(), vec!["-m".to_string(), "pytest".to_string()])
    };

    let mut cmd = Command::new(&test_cmd);
    for arg in &test_args {
        cmd.arg(arg);
    }

    cmd.env("PYTHONPATH", &python_path);

    if cfg.simulation {
        cmd.env("HORUS_SIMULATION_MODE", "1");
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

    println!("  {} Executing: {} tests", "->".blue(), "Python");

    let status = cmd.status().context("Failed to execute pytest")?;

    if status.success() {
        println!("{}", "Python tests passed!".green().bold());
    } else {
        anyhow::bail!("Python tests failed with exit code {}", status.code().unwrap_or(1));
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
        println!(
            "  {} Source file: {}",
            "[*]".cyan(),
            main_file.display()
        );
    }

    crate::cargo_gen::generate(
        &manifest,
        &project_dir,
        &[main_file],
        true, // include_dev = true for testing
    )
    .context("Failed to generate .horus/Cargo.toml for testing")?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_needs_rebuild_no_cargo_toml() {
        let temp_dir = PathBuf::from("/tmp/test_horus_nonexistent");
        assert!(needs_rebuild(&temp_dir));
    }
}
