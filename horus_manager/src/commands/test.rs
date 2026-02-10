//! Test command - run cargo test with HORUS-specific enhancements
//!
//! Features beyond cargo test:
//! - Build-before-test: Ensures Cargo.toml is generated/updated
//! - Simulation mode: Enables simulation drivers for hardware-free testing
//! - Default single-threaded: Prevents shared memory conflicts (--parallel to override)
//! - Integration test mode: Runs tests marked `#[ignore]`

use anyhow::{Context, Result};
use colored::*;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use crate::commands::run;

/// Check if Cargo.toml needs regeneration
fn needs_rebuild(horus_dir: &Path) -> bool {
    let cargo_toml = horus_dir.join("Cargo.toml");
    let horus_yaml = PathBuf::from("horus.yaml");

    // If no Cargo.toml, definitely needs build
    if !cargo_toml.exists() {
        return true;
    }

    // If horus.yaml exists and is newer than Cargo.toml, needs rebuild
    if horus_yaml.exists() {
        if let (Ok(yaml_meta), Ok(cargo_meta)) =
            (fs::metadata(&horus_yaml), fs::metadata(&cargo_toml))
        {
            if let (Ok(yaml_time), Ok(cargo_time)) = (yaml_meta.modified(), cargo_meta.modified()) {
                return yaml_time > cargo_time;
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

/// Run tests for a HORUS project with full robotics-aware features
#[allow(clippy::too_many_arguments)]
pub fn run_tests(
    filter: Option<String>,
    release: bool,
    nocapture: bool,
    test_threads: Option<usize>,
    parallel: bool,
    simulation: bool,
    integration: bool,
    no_build: bool,
    _no_cleanup: bool, // Legacy parameter, kept for API compatibility
    verbose: bool,
) -> Result<()> {
    let horus_dir = PathBuf::from(".horus");

    println!("{} Running HORUS tests", "[*]".cyan());

    // Step 1: Build if needed (unless --no-build)
    if !no_build {
        if needs_rebuild(&horus_dir) || !horus_dir.join("Cargo.toml").exists() {
            println!(
                "  {} Generating/updating build configuration...",
                "[*]".cyan()
            );

            // Use the same build logic as `horus build`
            run::execute_build_only(vec![], release, false)
                .context("Failed to build project before testing")?;
        } else if verbose {
            println!("  {} Build is up to date, skipping...", "[*]".cyan());
        }
    }

    // Verify .horus/Cargo.toml exists after potential build
    let cargo_toml = horus_dir.join("Cargo.toml");
    if !cargo_toml.exists() {
        println!("{} No .horus/Cargo.toml found.", "[!]".yellow());
        println!(
            "    Run {} first to set up the build environment.",
            "horus build".cyan()
        );
        return Ok(());
    }

    // Step 2: Set up test environment
    if simulation {
        env::set_var("HORUS_SIMULATION_MODE", "1");
        println!(
            "  {} Simulation mode enabled (no hardware required)",
            "[*]".cyan()
        );
    }

    // Step 3: Build cargo test command
    let mut cmd = Command::new("cargo");
    cmd.arg("test");
    cmd.current_dir(&horus_dir);

    // Pass through environment
    if simulation {
        cmd.env("HORUS_SIMULATION_MODE", "1");
    }

    // Add release flag if requested
    if release {
        cmd.arg("--release");
        if verbose {
            println!("  {} Building in release mode", "[*]".cyan());
        }
    }

    // Add test filter if provided
    if let Some(ref f) = filter {
        cmd.arg(f);
        if verbose {
            println!("  {} Filter: {}", "[*]".cyan(), f);
        }
    }

    // Separator for test runner args
    cmd.arg("--");

    // Add --nocapture if requested
    if nocapture {
        cmd.arg("--nocapture");
    }

    // Thread control: default to single-threaded unless --parallel or explicit -j
    let effective_threads = if let Some(threads) = test_threads {
        // Explicit thread count overrides everything
        threads
    } else if parallel {
        // --parallel means use all cores
        num_cpus::get()
    } else {
        // Default: single-threaded for shared memory safety
        1
    };

    cmd.arg(format!("--test-threads={}", effective_threads));

    if verbose {
        println!(
            "  {} Test threads: {} {}",
            "[*]".cyan(),
            effective_threads,
            if effective_threads == 1 {
                "(single-threaded for shared memory safety)"
                    .dimmed()
                    .to_string()
            } else {
                "".to_string()
            }
        );
    }

    // Integration tests: run ignored tests
    if integration {
        cmd.arg("--ignored");
        println!(
            "  {} Running integration tests (marked #[ignore])",
            "[*]".cyan()
        );
    }

    // Show test output in verbose mode
    if verbose {
        cmd.arg("--show-output");
    }

    println!("  {} Executing: cargo test in .horus/", "->".blue());

    // Step 4: Run the tests
    let status = cmd.status().context("Failed to execute cargo test")?;

    // Step 5: Report results
    if status.success() {
        println!("{}", "Tests passed!".green().bold());
    } else {
        println!("{}", "Some tests failed".red().bold());
        std::process::exit(status.code().unwrap_or(1));
    }

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
