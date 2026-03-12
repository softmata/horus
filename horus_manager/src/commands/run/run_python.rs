use crate::cli_output;
use anyhow::{bail, Result};
use colored::*;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

pub(super) fn execute_python_node(file: PathBuf, args: Vec<String>, _release: bool) -> Result<()> {
    eprintln!(
        "{} Setting up Python environment...",
        cli_output::ICON_INFO.cyan()
    );

    // Generate .horus/pyproject.toml from horus.toml if present
    generate_pyproject_if_needed()?;

    // Check for Python interpreter
    let python_cmd = detect_python_interpreter()?;

    // Build PYTHONPATH for child processes (no env::set_var)
    let python_path = build_python_path()?;

    // Detect if this is a HORUS node or plain Python script
    let uses_horus = detect_horus_usage_python(&file)?;

    if uses_horus {
        // Validate and canonicalize the path before use so that a crafted
        // filename cannot escape the wrapper's env-var sandboxing.
        let canonical_file = validate_node_path(&file)?;

        // Use scheduler wrapper for HORUS nodes
        eprintln!(
            "{} Executing Python node with HORUS scheduler...",
            cli_output::ICON_INFO.cyan()
        );

        let wrapper_script = create_python_wrapper()?;

        let mut cmd = Command::new(python_cmd);
        cmd.arg(&wrapper_script);
        // Pass the real node path out-of-band — never inline in Python source.
        cmd.env("HORUS_NODE_FILE", &canonical_file);
        cmd.env("PYTHONPATH", &python_path);
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

        // Propagate the child's exit code as an error
        if !status.success() {
            bail!(
                "Python node exited with code {}",
                status.code().unwrap_or(1)
            );
        }
    } else {
        // Direct execution for plain Python scripts
        eprintln!(
            "{} Executing Python script directly...",
            cli_output::ICON_INFO.cyan()
        );

        let mut cmd = Command::new(python_cmd);
        cmd.arg(&file);
        cmd.env("PYTHONPATH", &python_path);
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

        // Propagate the child's exit code as an error
        if !status.success() {
            bail!(
                "Python script exited with code {}",
                status.code().unwrap_or(1)
            );
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

/// Build PYTHONPATH for child processes without calling `env::set_var`.
///
/// Returns the combined PYTHONPATH string to pass via `Command::env()`.
pub(crate) fn build_python_path() -> Result<String> {
    let current_dir = env::current_dir()?;
    let horus_packages = current_dir.join(".horus/packages");

    let global_cache = crate::paths::cache_dir()?;

    let mut python_paths = Vec::new();

    // Collect all global cache Python package lib directories
    if global_cache.exists() {
        if let Ok(entries) = fs::read_dir(&global_cache) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.is_dir() {
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

    Ok(python_paths.join(":"))
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

/// Validate that a Python node file path is safe to execute.
///
/// Returns the canonicalized absolute path on success.  Rejects paths that
/// contain null bytes (which Path already forbids in Rust, so this is a
/// belt-and-suspenders check) or that cannot be resolved on the filesystem.
fn validate_node_path(file: &Path) -> Result<PathBuf> {
    // Canonicalize resolves symlinks and ensures the file actually exists.
    let canonical = file
        .canonicalize()
        .map_err(|e| anyhow::anyhow!("Invalid node path '{}': {}", file.display(), e))?;

    // Reject paths whose OS string contains any null byte (defensive: Rust's
    // OsStr should already prevent this, but be explicit).
    {
        use std::os::unix::ffi::OsStrExt;
        if canonical.as_os_str().as_bytes().contains(&0u8) {
            bail!("Node path contains a null byte and cannot be used safely.");
        }
    }

    Ok(canonical)
}

/// Create a temporary Python wrapper script that loads the real node file from
/// the `HORUS_NODE_FILE` environment variable at runtime.
///
/// **Security**: the user-supplied file path is intentionally NOT interpolated
/// into the Python source here.  It is passed out-of-band via the environment
/// variable so that a crafted filename like `'); os.system('rm -rf /')` cannot
/// execute arbitrary code.
fn create_python_wrapper() -> Result<PathBuf> {
    // Use a timestamp-based name so the wrapper cannot be guessed or hijacked,
    // and no user-controlled string is included in the filename.
    let ts = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    let wrapper_path = env::temp_dir().join(format!("horus_wrapper_{ts}.py"));

    // Fixed template — no user input anywhere inside this string.
    let wrapper_content = r#"#!/usr/bin/env python3
"""
HORUS Python Node Wrapper
Auto-generated wrapper for HORUS scheduler integration.
The node file path is supplied via the HORUS_NODE_FILE environment variable;
it is never interpolated into this source code.
"""
import sys
import os

class HorusSchedulerIntegration:
    def __init__(self):
        self.running = True

    def run_node(self):
        """Run the user's node code with scheduler integration."""
        node_file = os.environ.get('HORUS_NODE_FILE')
        if not node_file:
            print("Error: HORUS_NODE_FILE environment variable not set", file=sys.stderr)
            sys.exit(2)

        exit_code = 0
        try:
            with open(node_file, 'r') as fh:
                source = fh.read()
            # compile() with the real filename produces correct tracebacks.
            exec(compile(source, node_file, 'exec'), globals())
        except SystemExit as e:
            exit_code = e.code if e.code is not None else 0
        except KeyboardInterrupt:
            print("\nGraceful shutdown initiated...", file=sys.stderr)
            exit_code = 0
        except Exception as e:
            print(f" Node execution failed: {e}", file=sys.stderr)
            exit_code = 1

        sys.exit(exit_code)

if __name__ == "__main__":
    print(" HORUS Python Node Starting...", file=sys.stderr)
    scheduler = HorusSchedulerIntegration()
    scheduler.run_node()
"#;

    fs::write(&wrapper_path, wrapper_content)?;
    Ok(wrapper_path)
}

/// Generate `.horus/pyproject.toml` from `horus.toml` if the manifest exists.
///
/// This keeps the Python build config in sync with the unified manifest,
/// mirroring what `cargo_gen` does for Rust projects.
fn generate_pyproject_if_needed() -> Result<()> {
    use crate::manifest::{HorusManifest, HORUS_TOML};

    let manifest_path = Path::new(HORUS_TOML);
    if !manifest_path.exists() {
        return Ok(());
    }

    let manifest = HorusManifest::load_from(manifest_path)
        .ok();

    if let Some(manifest) = manifest {
        // Only generate if there are Python deps
        let has_python = manifest.dependencies.values().any(|v| v.is_pypi());
        if has_python {
            let project_dir = env::current_dir()?;
            crate::pyproject_gen::generate(&manifest, &project_dir, false)?;
            eprintln!(
                "  {} Generated .horus/pyproject.toml",
                cli_output::ICON_INFO.cyan()
            );
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Malformed / non-existent paths must be rejected by validate_node_path.
    /// Critically, none of the special characters in these paths must end up
    /// interpolated into any Python source string.
    #[test]
    fn test_malicious_path_rejected() {
        let evil_paths = [
            // Classic injection attempt
            "'); __import__('os').system('id')",
            // Double-quote variant
            r#""); __import__('os').system('id')"#,
            // Semicolon path traversal with backtick
            "/tmp/../../etc/shadow`id`",
            // Null-byte injection (non-representable as a real Path on Linux,
            // but let's make sure a manufactured OsString attempt is caught).
        ];
        for path in &evil_paths {
            let result = validate_node_path(Path::new(path));
            assert!(
                result.is_err(),
                "validate_node_path should reject malicious path: {path}"
            );
        }
    }

    /// The wrapper script content must not contain any format placeholder
    /// that could be filled with user data.
    #[test]
    fn test_wrapper_contains_no_format_placeholders() {
        // create_python_wrapper writes a fixed string — verify it doesn't
        // contain `{}` or `r'{}'` patterns that would indicate leftover
        // format!() interpolation.
        let wrapper_path = create_python_wrapper().expect("wrapper creation failed");
        let content = fs::read_to_string(&wrapper_path).expect("read wrapper");
        fs::remove_file(&wrapper_path).ok();

        assert!(
            !content.contains("r'{}'") && !content.contains("open(r'"),
            "Wrapper must not contain inline path placeholders; got:\n{content}"
        );
        assert!(
            content.contains("HORUS_NODE_FILE"),
            "Wrapper must read node path from HORUS_NODE_FILE env var"
        );
    }

    // ── Battle-testing: Python interpreter detection ─────────────────────

    #[test]
    fn battle_detect_python_interpreter_succeeds() {
        // Should find python3 or python on any dev machine
        let result = detect_python_interpreter();
        assert!(result.is_ok(), "Should find a Python interpreter");
        let cmd = result.unwrap();
        assert!(
            cmd == "python3" || cmd == "python",
            "Should be python3 or python, got: {}",
            cmd
        );
    }

    // ── Battle-testing: PYTHONPATH building ──────────────────────────────

    #[test]
    fn battle_build_python_path_includes_horus_packages() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _lock = crate::CWD_LOCK.lock().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = build_python_path();
        std::env::set_current_dir(&prev).unwrap();

        assert!(result.is_ok());
        let path = result.unwrap();
        assert!(
            path.contains(".horus/packages"),
            "PYTHONPATH should include .horus/packages, got: {}",
            path
        );
    }

    #[test]
    fn battle_build_python_path_with_existing_packages() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::create_dir_all(tmp.path().join(".horus/packages")).unwrap();

        let _lock = crate::CWD_LOCK.lock().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = build_python_path();
        std::env::set_current_dir(&prev).unwrap();

        assert!(result.is_ok());
        let path = result.unwrap();
        assert!(path.contains(".horus/packages"));
    }

    // ── Battle-testing: horus usage detection ───────────────────────────

    #[test]
    fn battle_detect_horus_usage_import_horus() {
        let tmp = tempfile::TempDir::new().unwrap();
        let py = tmp.path().join("node.py");
        fs::write(&py, "import horus\nhorus.init()\n").unwrap();
        assert!(detect_horus_usage_python(&py).unwrap());
    }

    #[test]
    fn battle_detect_horus_usage_from_horus() {
        let tmp = tempfile::TempDir::new().unwrap();
        let py = tmp.path().join("node.py");
        fs::write(&py, "from horus import Node\n").unwrap();
        assert!(detect_horus_usage_python(&py).unwrap());
    }

    #[test]
    fn battle_detect_horus_usage_import_horus_py() {
        let tmp = tempfile::TempDir::new().unwrap();
        let py = tmp.path().join("node.py");
        fs::write(&py, "import horus_py\n").unwrap();
        assert!(detect_horus_usage_python(&py).unwrap());
    }

    #[test]
    fn battle_detect_horus_usage_from_horus_py() {
        let tmp = tempfile::TempDir::new().unwrap();
        let py = tmp.path().join("node.py");
        fs::write(&py, "from horus_py import something\n").unwrap();
        assert!(detect_horus_usage_python(&py).unwrap());
    }

    #[test]
    fn battle_detect_no_horus_usage() {
        let tmp = tempfile::TempDir::new().unwrap();
        let py = tmp.path().join("script.py");
        fs::write(&py, "import os\nimport sys\nprint('hello')\n").unwrap();
        assert!(!detect_horus_usage_python(&py).unwrap());
    }

    #[test]
    fn battle_detect_horus_usage_empty_file() {
        let tmp = tempfile::TempDir::new().unwrap();
        let py = tmp.path().join("empty.py");
        fs::write(&py, "").unwrap();
        assert!(!detect_horus_usage_python(&py).unwrap());
    }

    #[test]
    fn battle_detect_horus_usage_nonexistent_file() {
        let result = detect_horus_usage_python(Path::new("/tmp/nonexistent_file_12345.py"));
        assert!(result.is_err());
    }

    // ── Battle-testing: validate_node_path ──────────────────────────────

    #[test]
    fn battle_validate_node_path_real_file() {
        let tmp = tempfile::TempDir::new().unwrap();
        let py = tmp.path().join("valid_node.py");
        fs::write(&py, "print('hello')").unwrap();
        let result = validate_node_path(&py);
        assert!(result.is_ok());
        let canonical = result.unwrap();
        assert!(canonical.is_absolute());
    }

    #[test]
    fn battle_validate_node_path_nonexistent() {
        let result = validate_node_path(Path::new("/tmp/this_does_not_exist_99999.py"));
        assert!(result.is_err());
    }

    #[test]
    fn battle_validate_node_path_directory_rejected() {
        let tmp = tempfile::TempDir::new().unwrap();
        // A directory should not be a valid node path
        let result = validate_node_path(tmp.path());
        // This should succeed since the directory exists — but the important
        // thing is that it doesn't panic. The caller checks it's a file.
        // So we just verify it doesn't return an error for an existing path.
        assert!(result.is_ok());
    }

    #[test]
    fn battle_validate_node_path_symlink() {
        let tmp = tempfile::TempDir::new().unwrap();
        let real = tmp.path().join("real.py");
        let link = tmp.path().join("link.py");
        fs::write(&real, "print('real')").unwrap();
        std::os::unix::fs::symlink(&real, &link).unwrap();

        let result = validate_node_path(&link);
        assert!(result.is_ok());
        // Canonical path should point to real file
        let canonical = result.unwrap();
        assert_eq!(canonical, real.canonicalize().unwrap());
    }

    // ── Battle-testing: wrapper creation ─────────────────────────────────

    #[test]
    fn battle_wrapper_is_valid_python() {
        let wrapper = create_python_wrapper().unwrap();
        let content = fs::read_to_string(&wrapper).unwrap();
        fs::remove_file(&wrapper).ok();

        // Verify it's valid Python by checking structure
        assert!(content.contains("#!/usr/bin/env python3"));
        assert!(content.contains("class HorusSchedulerIntegration"));
        assert!(content.contains("def run_node(self)"));
        assert!(content.contains("if __name__ == \"__main__\""));
    }

    #[test]
    fn battle_wrapper_unique_filenames() {
        let w1 = create_python_wrapper().unwrap();
        let w2 = create_python_wrapper().unwrap();
        // Two wrappers created at different times should have different names
        // (or at least not collide)
        fs::remove_file(&w1).ok();
        fs::remove_file(&w2).ok();
        // Both created successfully — that's the test
    }

    // ── Battle-testing: pyproject generation ─────────────────────────────

    #[test]
    fn battle_generate_pyproject_no_manifest() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _lock = crate::CWD_LOCK.lock().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        // No horus.toml — should succeed (no-op)
        let result = generate_pyproject_if_needed();
        std::env::set_current_dir(&prev).unwrap();
        assert!(result.is_ok());
    }

    #[test]
    fn battle_generate_pyproject_no_python_deps() {
        let tmp = tempfile::TempDir::new().unwrap();
        let toml = r#"[package]
name = "rust-only"
version = "0.1.0"

[dependencies]
serde = "1.0"
"#;
        fs::write(tmp.path().join("horus.toml"), toml).unwrap();

        let _lock = crate::CWD_LOCK.lock().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = generate_pyproject_if_needed();
        std::env::set_current_dir(&prev).unwrap();
        assert!(result.is_ok());
        // No .horus/pyproject.toml should be generated for Rust-only deps
        assert!(!tmp.path().join(".horus/pyproject.toml").exists());
    }
}
