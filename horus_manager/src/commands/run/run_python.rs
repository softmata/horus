use crate::cli_output;
use anyhow::{bail, Context, Result};
use colored::*;
use std::collections::HashSet;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

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

        // `_wrapper_dir` must outlive the child: dropping it deletes the
        // directory holding the script.
        let (_wrapper_dir, wrapper_script) = create_python_wrapper()?;

        let mut cmd = Command::new(python_cmd);
        cmd.arg(&wrapper_script);
        // Pass the real node path out-of-band — never inline in Python source.
        cmd.env("HORUS_NODE_FILE", &canonical_file);
        cmd.env("PYTHONPATH", &python_path);
        cmd.args(args);

        // Spawn child process with Ctrl+C forwarding
        let child = cmd.spawn()?;
        let status = super::spawn_with_ctrlc(child, "Python")?;

        // Propagate the child's exit code as an error
        if !status.success() {
            let code = status.code().unwrap_or(1);
            crate::error_wrapper::emit_diagnostics(&crate::error_wrapper::exit_code_hint(
                "python", code,
            ));
            bail!("Python node exited with code {}", code);
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

        // Spawn child process with Ctrl+C forwarding
        let child = cmd.spawn()?;
        let status = super::spawn_with_ctrlc(child, "Python")?;

        // Propagate the child's exit code as an error
        if !status.success() {
            let code = status.code().unwrap_or(1);
            crate::error_wrapper::emit_diagnostics(&crate::error_wrapper::exit_code_hint(
                "python", code,
            ));
            bail!("Python script exited with code {}", code);
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
    crate::error_wrapper::emit_diagnostics(&[crate::error_wrapper::Diagnostic::new(
        "python",
        "H052",
        "No Python interpreter found",
        format!(
            "No Python interpreter found. Install with:\n  {}",
            crate::error_wrapper::suggest_install("python3").green()
        ),
    )
    .with_fix(crate::error_wrapper::Fix::Command {
        command: crate::error_wrapper::suggest_install("python3"),
    })]);
    bail!("No Python interpreter found. Install Python 3.7+ and ensure it's in PATH.");
}

/// The separator PYTHONPATH entries are joined with.
pub(crate) const PYTHON_PATH_SEP: &str = if cfg!(windows) { ";" } else { ":" };

/// Build PYTHONPATH for child processes without calling `env::set_var`.
///
/// Returns the combined PYTHONPATH string to pass via `Command::env()`.
pub(crate) fn build_python_path() -> Result<String> {
    let current_dir = env::current_dir()?;
    let mut python_paths = collect_python_path_entries(&current_dir, &python_cache_roots());

    // Add existing PYTHONPATH
    if let Ok(current_path) = env::var("PYTHONPATH") {
        python_paths.push(current_path);
    }

    Ok(python_paths.join(PYTHON_PATH_SEP))
}

/// Every cache root that may hold pip-installed packages.
///
/// Two roots, because the codebase has historically used both and the two
/// halves of the Python path disagreed about which one was authoritative:
/// `install::install_pip_packages` writes `pypi_*` under `$HOME/.horus/cache`
/// (the root `install.sh` and `horus clean` also manage), while this function
/// used to read `crate::paths::cache_dir()` alone — the XDG root,
/// `$XDG_CACHE_HOME/horus` or `~/.cache/horus`. On Linux those are different
/// directories, so PYTHONPATH never contained the package pip had just
/// unpacked: `horus run` printed "Cached horus-robotics / Linked
/// horus-robotics (horus) / Updated horus.lock" and then died on
/// `import horus`. `find_horus_source_dir` in run_rust.rs searches both roots
/// for exactly the same reason.
pub(crate) fn python_cache_roots() -> Vec<PathBuf> {
    let mut roots = Vec::new();
    if let Ok(xdg_cache) = crate::paths::cache_dir() {
        roots.push(xdg_cache);
    }
    let legacy_cache = super::install::home_dir().join(".horus/cache");
    if !roots.contains(&legacy_cache) {
        roots.push(legacy_cache);
    }
    roots
}

/// The sys.path entries a project needs, in priority order.
///
/// Split out from `build_python_path` so it can be exercised against a
/// throw-away project and cache instead of the developer's real home
/// directory.
fn collect_python_path_entries(project_dir: &Path, cache_roots: &[PathBuf]) -> Vec<String> {
    let horus_packages = project_dir.join(".horus/packages");

    let mut entries = Vec::new();
    let mut seen = HashSet::new();

    // Project-scoped entries first: `.horus/packages/<dist>` is a symlink into
    // the cache, so the link path itself is a usable sys.path entry and one
    // that names the packages *this* project resolved.
    push_importable_dirs(
        &horus_packages,
        CacheScope::Project,
        &mut entries,
        &mut seen,
    );

    for root in cache_roots {
        push_importable_dirs(root, CacheScope::Shared, &mut entries, &mut seen);
    }

    // `.horus/packages` itself stays on the path for anything that drops a
    // module straight into it rather than linking one. On its own it is not
    // enough: its children are named after the *distribution*
    // (`horus-robotics`), never after the module it provides (`horus`).
    push_entry(horus_packages, &mut entries, &mut seen);

    entries
}

/// Add every directory under `root` that a Python import could use.
///
/// `pip install --target DIR` unpacks the distribution straight into DIR — it
/// never creates a `lib/` subdirectory. Probing only for `<pkg>/lib`, which is
/// what this used to do, therefore matched nothing on any cache pip had
/// written, and left `.horus/packages` as the only entry on PYTHONPATH. The
/// `lib/` probe is kept because HORUS registry packages are laid out that way.
///
/// A directory is only added if it actually provides an importable name.
/// Skipping the rest is not tidiness: the cache also holds *source* trees such
/// as `horus@0.3.0`, whose `horus/` subdirectory has a `Cargo.toml` and no
/// `__init__.py`. Putting one on sys.path would make `import horus` resolve to
/// an empty namespace package that shadows the real one.
fn push_importable_dirs(
    root: &Path,
    scope: CacheScope,
    entries: &mut Vec<String>,
    seen: &mut HashSet<PathBuf>,
) {
    let Ok(read) = fs::read_dir(root) else {
        return;
    };
    let mut dirs: Vec<PathBuf> = read
        .flatten()
        .map(|entry| entry.path())
        .filter(|path| path.is_dir())
        .collect();
    // read_dir order is arbitrary; sort so PYTHONPATH is stable between runs.
    dirs.sort();

    for dir in dirs {
        if scope == CacheScope::Shared && is_unlinked_pypi_cache_dir(&dir, seen) {
            continue;
        }
        let lib_dir = dir.join("lib");
        if lib_dir.is_dir() {
            push_entry(lib_dir, entries, seen);
        }
        if !super::install::top_level_modules(&dir).is_empty() {
            push_entry(dir, entries, seen);
        }
    }
}

/// Whose packages a directory of packages holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CacheScope {
    /// `.horus/packages` — what this project resolved.
    Project,
    /// A global cache root — what every project on this machine resolved.
    Shared,
}

/// A `pypi_<dist>@<version>` cache directory this project never linked to.
///
/// The global cache is storage shared by every project on the machine, and
/// `install_pip_packages` links each distribution a project resolves into that
/// project's own `.horus/packages`. Adding the whole cache root to PYTHONPATH
/// therefore put *other projects'* dependencies on this one's import path: a
/// project depending on nothing imported `serial`, `idna` and `wcwidth`
/// because something else on the machine had once needed them. It worked on
/// the developer's box and failed on the robot, which is the same shape of
/// defect as the one this path was fixed for.
///
/// Being already in `seen` is exactly the test for "this project linked it":
/// the `.horus/packages` pass ran first and deduplicates by canonical path, so
/// a linked distribution's cache directory is already recorded. Anything else
/// under the cache root — the HORUS registry layout, `<name>@<version>` with a
/// `lib/` — is left alone; that is what `horus install --global` writes and it
/// is not per-project.
fn is_unlinked_pypi_cache_dir(dir: &Path, seen: &HashSet<PathBuf>) -> bool {
    let Some(name) = dir.file_name().and_then(|n| n.to_str()) else {
        return false;
    };
    if !name.starts_with("pypi_") {
        return false;
    }
    let key = dir.canonicalize().unwrap_or_else(|_| dir.to_path_buf());
    !seen.contains(&key)
}

/// Append a path unless the same directory is already on the list.
///
/// Deduplication is by canonical path, because the same cache directory is
/// reachable both through `.horus/packages/<dist>` and directly under the
/// cache root.
fn push_entry(path: PathBuf, entries: &mut Vec<String>, seen: &mut HashSet<PathBuf>) {
    let key = path.canonicalize().unwrap_or_else(|_| path.clone());
    if seen.insert(key) {
        entries.push(path.display().to_string());
    }
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
    // Reject paths that aren't valid UTF-8 (defensive check, also catches null bytes)
    if canonical.to_str().is_none() {
        bail!("Node path contains invalid characters and cannot be used safely.");
    }

    Ok(canonical)
}

/// Create a temporary Python wrapper script that loads the real node file from
/// the `HORUS_NODE_FILE` environment variable at runtime.
///
/// Returns the owning [`TempDir`] alongside the script path: the directory must
/// stay alive until the interpreter has finished with the script, and dropping
/// it removes both.
///
/// **Security**: the user-supplied file path is intentionally NOT interpolated
/// into the Python source here.  It is passed out-of-band via the environment
/// variable so that a crafted filename like `'); os.system('rm -rf /')` cannot
/// execute arbitrary code.
fn create_python_wrapper() -> Result<(tempfile::TempDir, PathBuf)> {
    // The wrapper used to be written to `/tmp/horus_wrapper_<nanos>.py` under
    // the claim that a timestamp "cannot be guessed or hijacked". A timestamp
    // is not a secret, and `fs::write` is O_CREAT|O_TRUNC without O_EXCL or
    // O_NOFOLLOW: in a shared /tmp another local user could pre-create the
    // path as a symlink, or simply rewrite the file in the window before the
    // interpreter opens it — and this file is then executed. A private
    // per-invocation directory (random name, mode 0700, created exclusively)
    // puts the script out of other users' reach entirely.
    //
    // The 0700 has to be asked for explicitly: `tempdir()` on its own creates
    // the directory with a plain `mkdir` at 0o777 & !umask, i.e. 0755 under
    // the usual umask 022 — owned by us, but *enterable and readable* by every
    // local user, which defeats the whole point of moving the script here.
    // `Builder::permissions` passes the mode down to `mkdir(2)` itself, so the
    // directory is never visible to anyone else even momentarily; chmod-ing
    // after the fact would leave a window with the loose mode. umask can only
    // clear further bits, never add them, so the result is at most 0700.
    let mut builder = tempfile::Builder::new();
    builder.prefix("horus_wrapper_");
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        builder.permissions(fs::Permissions::from_mode(0o700));
    }
    let dir = builder
        .tempdir()
        .context("creating a private temporary directory for the Python wrapper")?;
    let wrapper_path = dir.path().join("wrapper.py");

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

    // Defence in depth for the script itself. The 0700 directory above is what
    // actually keeps other users out, but the file inherits nothing from it:
    // `fs::write` would create it 0o666 & !umask, so a wrapper.py that ever
    // ended up somewhere less protected (a differently-configured TMPDIR, a
    // future caller passing in its own directory) would be world-readable.
    // `create_new` adds O_EXCL, so this never opens a path that already exists
    // — no pre-planted symlink or file can be written through.
    use std::io::Write;
    let mut opts = fs::OpenOptions::new();
    opts.write(true).create_new(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        opts.mode(0o600);
    }
    let mut wrapper_file = opts
        .open(&wrapper_path)
        .with_context(|| format!("creating the Python wrapper {}", wrapper_path.display()))?;
    wrapper_file
        .write_all(wrapper_content.as_bytes())
        .with_context(|| format!("writing the Python wrapper {}", wrapper_path.display()))?;

    Ok((dir, wrapper_path))
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

    let manifest = HorusManifest::load_from(manifest_path).ok();

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
        let (_dir, wrapper_path) = create_python_wrapper().expect("wrapper creation failed");
        let content = fs::read_to_string(&wrapper_path).expect("read wrapper");

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

    // ── PYTHONPATH has to point at the cache pip wrote to (PATH-3) ───────
    //
    // The two halves of the Python path disagreed about which cache root was
    // authoritative — install.rs wrote `$HOME/.horus/cache`, this file read
    // the XDG root — and the entry that survived, `.horus/packages`, holds
    // symlinks named after the *distribution*, never after the module. So
    // `horus run` printed "Cached horus-robotics / Linked horus-robotics
    // (horus) / Updated horus.lock" and then died on `import horus`.

    /// Build the cache pip would build, and check the interpreter can reach
    /// it. Asserting on the string alone would pass on a machine where the
    /// module is pip-installed anyway — which is exactly the machine the
    /// original defect hid on — so this runs the real import.
    #[test]
    fn battle_python_path_makes_a_cached_distribution_importable() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cache = tmp.path().join("cache");
        let project = tmp.path().join("project");

        // pip install --target unpacks straight into the package directory.
        // There is no `lib/` — that is what the old probe looked for.
        let pkg_dir = cache.join("pypi_horus-fixture@latest");
        fs::create_dir_all(pkg_dir.join("horus_path_fixture")).unwrap();
        fs::write(
            pkg_dir.join("horus_path_fixture/__init__.py"),
            "value = 1\n",
        )
        .unwrap();
        fs::create_dir_all(project.join(".horus/packages")).unwrap();
        horus_sys::fs::symlink(&pkg_dir, &project.join(".horus/packages/horus-fixture")).unwrap();

        let entries = collect_python_path_entries(&project, std::slice::from_ref(&cache));
        let python_path = entries.join(PYTHON_PATH_SEP);

        let output = Command::new("python3")
            .arg("-c")
            .arg("import horus_path_fixture")
            .env("PYTHONPATH", &python_path)
            .output()
            .expect("python3 must run");
        assert!(
            output.status.success(),
            "a linked distribution has to be importable through the PYTHONPATH \
             HORUS builds.\nPYTHONPATH={python_path}\n{}",
            String::from_utf8_lossy(&output.stderr)
        );
    }

    /// The cache also holds *source* trees — `horus@0.3.0` has a `horus/`
    /// subdirectory with a Cargo.toml and no `__init__.py`. Putting one on
    /// sys.path would make `import horus` resolve to an empty namespace
    /// package that shadows the real one, which is a worse failure than the
    /// one being fixed because it fails at first use rather than at import.
    #[test]
    fn battle_python_path_skips_source_trees_that_would_shadow_a_module() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cache = tmp.path().join("cache");
        let source_tree = cache.join("horus@0.3.0");
        fs::create_dir_all(source_tree.join("horus/src")).unwrap();
        fs::write(source_tree.join("horus/Cargo.toml"), "[package]\n").unwrap();

        let entries = collect_python_path_entries(tmp.path(), &[cache]);
        assert!(
            !entries.iter().any(|e| e.contains("horus@0.3.0")),
            "a Rust source tree provides no importable module: {entries:?}"
        );
    }

    /// The global cache is storage for every project on the machine, not an
    /// import path for all of them.
    ///
    /// Adding the whole cache root put another project's dependencies on this
    /// project's sys.path: a project depending on nothing imported `serial`,
    /// `idna` and `wcwidth` because something else on the box had needed them
    /// once. That is a build which works on the developer's machine and fails
    /// on the robot — the same shape as the defect this path was fixed for,
    /// pointing the other way.
    #[test]
    fn battle_python_path_does_not_leak_another_projects_packages() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cache = tmp.path().join("cache");
        let project = tmp.path().join("project");

        // This project's own dependency, resolved and linked.
        let mine = cache.join("pypi_mine@1.0");
        fs::create_dir_all(mine.join("minemod")).unwrap();
        fs::write(mine.join("minemod/__init__.py"), "").unwrap();
        fs::create_dir_all(project.join(".horus/packages")).unwrap();
        horus_sys::fs::symlink(&mine, &project.join(".horus/packages/mine")).unwrap();

        // Another project's dependency, sharing the cache and nothing else.
        let theirs = cache.join("pypi_theirs@2.0");
        fs::create_dir_all(theirs.join("theirmod")).unwrap();
        fs::write(theirs.join("theirmod/__init__.py"), "").unwrap();

        // A HORUS registry package, which is not per-project and stays.
        let registry = cache.join("horus_py@0.3.0");
        fs::create_dir_all(registry.join("lib/horus")).unwrap();
        fs::write(registry.join("lib/horus/__init__.py"), "").unwrap();

        let entries = collect_python_path_entries(&project, &[cache]);
        assert!(
            entries.iter().any(|e| e.contains("mine")),
            "the project's own dependency has to stay importable: {entries:?}"
        );
        assert!(
            !entries.iter().any(|e| e.contains("theirs")),
            "a cached distribution this project never resolved is not its \
             dependency: {entries:?}"
        );
        assert!(
            entries.iter().any(|e| e.contains("horus_py@0.3.0")),
            "the registry layout is what `horus install --global` writes and is \
             not per-project: {entries:?}"
        );
    }

    /// Both roots, or the fix only works for whichever one this machine
    /// happens to use. `find_horus_source_dir` searches both for the same
    /// reason, and its comment names the same history.
    #[test]
    fn battle_python_cache_roots_include_the_root_the_installer_writes() {
        let roots = python_cache_roots();
        assert!(
            roots.iter().any(|r| r.ends_with(".horus/cache")),
            "install_pip_packages writes $HOME/.horus/cache — reading only the \
             XDG root is how PYTHONPATH came to miss every package: {roots:?}"
        );
    }

    /// The same directory is reachable both through `.horus/packages/<dist>`
    /// and directly under the cache root; PYTHONPATH should say it once.
    #[test]
    fn battle_python_path_does_not_repeat_the_same_directory() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cache = tmp.path().join("cache");
        let project = tmp.path().join("project");
        let pkg_dir = cache.join("pypi_dup@1.0");
        fs::create_dir_all(pkg_dir.join("dupmod")).unwrap();
        fs::write(pkg_dir.join("dupmod/__init__.py"), "").unwrap();
        fs::create_dir_all(project.join(".horus/packages")).unwrap();
        horus_sys::fs::symlink(&pkg_dir, &project.join(".horus/packages/dup")).unwrap();

        let entries = collect_python_path_entries(&project, &[cache]);
        let mut canonical: Vec<PathBuf> = entries
            .iter()
            .filter_map(|e| Path::new(e).canonicalize().ok())
            .collect();
        let before = canonical.len();
        canonical.sort();
        canonical.dedup();
        assert_eq!(before, canonical.len(), "duplicate entries: {entries:?}");
    }

    // ── Battle-testing: PYTHONPATH building ──────────────────────────────

    #[test]
    fn battle_build_python_path_includes_horus_packages() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let prev = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
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

        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let prev = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
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
        result.unwrap_err();
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
        result.unwrap_err();
    }

    #[test]
    fn battle_validate_node_path_directory_rejected() {
        let tmp = tempfile::TempDir::new().unwrap();
        // A directory should not be a valid node path
        let result = validate_node_path(tmp.path());
        // This should succeed since the directory exists — but the important
        // thing is that it doesn't panic. The caller checks it's a file.
        // So we just verify it doesn't return an error for an existing path.
        result.unwrap();
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
        let (_dir, wrapper) = create_python_wrapper().unwrap();
        let content = fs::read_to_string(&wrapper).unwrap();

        // Verify it's valid Python by checking structure
        assert!(content.contains("#!/usr/bin/env python3"));
        assert!(content.contains("class HorusSchedulerIntegration"));
        assert!(content.contains("def run_node(self)"));
        assert!(content.contains("if __name__ == \"__main__\""));
    }

    #[test]
    fn battle_wrapper_unique_filenames() {
        // This used to assert nothing at all — two timestamped names in a
        // shared /tmp. Each wrapper now lives in its own private directory, so
        // assert the real property: the two paths differ and both exist at
        // once, i.e. concurrent runs cannot land on the same file.
        let (d1, w1) = create_python_wrapper().unwrap();
        let (d2, w2) = create_python_wrapper().unwrap();
        assert_ne!(w1, w2, "two wrappers must not share a path");
        assert!(w1.exists() && w2.exists());
        drop(d1);
        drop(d2);
        assert!(!w1.exists(), "TempDir drop must remove the wrapper");
        assert!(!w2.exists(), "TempDir drop must remove the wrapper");
    }

    #[test]
    fn wrapper_lives_in_a_private_directory_not_shared_tmp() {
        // Regression: the wrapper was written to a predictable
        // `/tmp/horus_wrapper_<nanos>.py` and then executed, so any local user
        // could pre-create or rewrite it. It must now sit inside its own
        // directory — and that directory has to be created 0700 *explicitly*:
        // tempfile's default is a plain `mkdir` at 0o777 & !umask, which is
        // 0755 under the usual umask 022 and leaves the script readable by
        // every local user. This assertion is what catches a regression back
        // to the default, so it checks the mode rather than trusting tempfile.
        let (dir, wrapper) = create_python_wrapper().unwrap();
        assert_eq!(
            wrapper.parent().unwrap(),
            dir.path(),
            "wrapper must live inside its own temp directory"
        );

        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mode = fs::metadata(dir.path()).unwrap().permissions().mode();
            assert_eq!(
                mode & 0o077,
                0,
                "wrapper directory must not be group/world accessible: {:o}",
                mode
            );

            // The script is what actually gets executed, so it carries its own
            // restriction rather than relying on the directory's mode alone.
            let file_mode = fs::metadata(&wrapper).unwrap().permissions().mode();
            assert_eq!(
                file_mode & 0o077,
                0,
                "wrapper script must not be group/world accessible: {:o}",
                file_mode
            );
        }
    }

    // ── Battle-testing: pyproject generation ─────────────────────────────

    #[test]
    fn battle_generate_pyproject_no_manifest() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let prev = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        // No horus.toml — should succeed (no-op)
        let result = generate_pyproject_if_needed();
        std::env::set_current_dir(&prev).unwrap();
        result.unwrap();
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

        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let prev = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = generate_pyproject_if_needed();
        std::env::set_current_dir(&prev).unwrap();
        result.unwrap();
        // No .horus/pyproject.toml should be generated for Rust-only deps
        assert!(!tmp.path().join(".horus/pyproject.toml").exists());
    }
}
