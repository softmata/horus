use anyhow::{bail, Result};
use crate::cli_output;
use colored::*;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

pub(super) fn execute_python_node(file: PathBuf, args: Vec<String>, _release: bool) -> Result<()> {
    eprintln!("{} Setting up Python environment...", cli_output::ICON_INFO.cyan());

    // Check for Python interpreter
    let python_cmd = detect_python_interpreter()?;

    // Setup Python path for horus_py integration
    setup_python_environment()?;

    // Detect if this is a HORUS node or plain Python script
    let uses_horus = detect_horus_usage_python(&file)?;

    if uses_horus {
        // Use scheduler wrapper for HORUS nodes
        eprintln!(
            "{} Executing Python node with HORUS scheduler...",
            cli_output::ICON_INFO.cyan()
        );

        let wrapper_script = create_python_wrapper(&file)?;

        let mut cmd = Command::new(python_cmd);
        cmd.arg(&wrapper_script);
        cmd.args(args);

        // Spawn child process so we can handle Ctrl+C
        let mut child = cmd.spawn()?;
        let child_id = child.id();

        // Setup Ctrl+C handler
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();
        ctrlc::set_handler(move || {
            println!("{}", "\nCtrl+C received, stopping Python process...".red());
            r.store(false, Ordering::SeqCst);
            // Send SIGINT to child process on Unix systems
            #[cfg(unix)]
            // SAFETY: child_id is a valid PID of a child process we spawned. SIGINT requests interruption.
            unsafe {
                libc::kill(child_id as i32, libc::SIGINT);
            }
        })
        .ok();

        // Wait for child to complete
        let status = child.wait()?;

        // Cleanup wrapper script
        fs::remove_file(wrapper_script).ok();

        // Exit with the same code as the Python script
        if !status.success() {
            std::process::exit(status.code().unwrap_or(1));
        }
    } else {
        // Direct execution for plain Python scripts
        eprintln!("{} Executing Python script directly...", cli_output::ICON_INFO.cyan());

        let mut cmd = Command::new(python_cmd);
        cmd.arg(&file);
        cmd.args(args);

        // Spawn child process so we can handle Ctrl+C
        let mut child = cmd.spawn()?;
        let child_id = child.id();

        // Setup Ctrl+C handler
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();
        ctrlc::set_handler(move || {
            println!("{}", "\nCtrl+C received, stopping Python process...".red());
            r.store(false, Ordering::SeqCst);
            // Send SIGINT to child process on Unix systems
            #[cfg(unix)]
            // SAFETY: child_id is a valid PID of a child process we spawned. SIGINT requests interruption.
            unsafe {
                libc::kill(child_id as i32, libc::SIGINT);
            }
        })
        .ok();

        // Wait for child to complete
        let status = child.wait()?;

        // Exit with the same code as the Python script
        if !status.success() {
            std::process::exit(status.code().unwrap_or(1));
        }
    }

    Ok(())
}

pub(crate) fn detect_python_interpreter() -> Result<String> {
    // Use system Python - packages are in PYTHONPATH via .horus/packages/
    for cmd in &["python3", "python"] {
        if Command::new(cmd).arg("--version").output().is_ok() {
            return Ok(cmd.to_string());
        }
    }
    bail!("No Python interpreter found. Install Python 3.7+ and ensure it's in PATH.");
}

pub(super) fn setup_python_environment() -> Result<()> {
    let current_dir = env::current_dir()?;
    let horus_packages = current_dir.join(".horus/packages");

    // Add global cache Python packages to PYTHONPATH
    let global_cache = crate::paths::cache_dir()?;

    let mut python_paths = Vec::new();

    // Collect all global cache Python package lib directories
    if global_cache.exists() {
        if let Ok(entries) = fs::read_dir(&global_cache) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.is_dir() {
                    // Check for lib directory (Python packages)
                    let lib_dir = path.join("lib");
                    if lib_dir.exists() {
                        python_paths.push(lib_dir.display().to_string());
                    }
                }
            }
        }
    }

    // Add local packages
    python_paths.push(horus_packages.display().to_string());

    // Add existing PYTHONPATH
    if let Ok(current_path) = env::var("PYTHONPATH") {
        python_paths.push(current_path);
    }

    // Set the combined PYTHONPATH
    let new_path = python_paths.join(":");
    env::set_var("PYTHONPATH", new_path);

    Ok(())
}

fn detect_horus_usage_python(file: &Path) -> Result<bool> {
    let content = fs::read_to_string(file)?;

    // Check for HORUS imports
    let horus_patterns = [
        "import horus",
        "from horus",
        "import horus_py",
        "from horus_py",
    ];

    for pattern in &horus_patterns {
        if content.contains(pattern) {
            return Ok(true);
        }
    }

    Ok(false)
}

fn create_python_wrapper(original_file: &Path) -> Result<PathBuf> {
    let wrapper_path = env::temp_dir().join(format!(
        "horus_wrapper_{}.py",
        original_file
            .file_stem()
            .unwrap_or_default()
            .to_string_lossy()
    ));

    let wrapper_content = format!(
        r#"#!/usr/bin/env python3
"""
HORUS Python Node Wrapper
Auto-generated wrapper for HORUS scheduler integration
"""
import sys
import os

# HORUS Python bindings are available via the 'horus' package
# Install with: cargo install maturin && maturin develop (from horus_py directory)
# Or: pip install horus-robotics

class HorusSchedulerIntegration:
    def __init__(self):
        self.running = True

    def run_node(self):
        """Run the user's node code with scheduler integration"""
        exit_code = 0
        try:
            # Execute user code in global namespace with proper scope
            # Pass globals() so imports and module-level code are accessible everywhere
            exec(compile(open(r'{}').read(), r'{}', 'exec'), globals())
        except SystemExit as e:
            # Preserve exit code from sys.exit()
            exit_code = e.code if e.code is not None else 0
        except KeyboardInterrupt:
            # Ctrl+C received - exit cleanly
            print("\nGraceful shutdown initiated...", file=sys.stderr)
            exit_code = 0
        except Exception as e:
            print(f" Node execution failed: {{e}}", file=sys.stderr)
            exit_code = 1

        sys.exit(exit_code)

# Initialize HORUS integration
if __name__ == "__main__":
    print(" HORUS Python Node Starting...", file=sys.stderr)
    scheduler = HorusSchedulerIntegration()
    scheduler.run_node()
"#,
        original_file.display(),
        original_file.display()
    );

    fs::write(&wrapper_path, wrapper_content)?;

    Ok(wrapper_path)
}
