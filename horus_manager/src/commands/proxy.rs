//! Transparent proxy commands for native tools (cargo, pip, cmake).
//!
//! When invoked as `horus cargo <args>`, `horus pip <args>`, or `horus cmake <args>`,
//! these functions:
//! 1. Ensure `.horus/<native-file>` is up-to-date from `horus.toml`
//! 2. Fingerprint the file before running
//! 3. Execute the real native tool with appropriate path rewrites
//! 4. If the file changed, sync modifications back to `horus.toml`

use anyhow::{bail, Context, Result};
use colored::*;
use std::env;
use std::path::{Path, PathBuf};
use std::process::Command;

use crate::fingerprint::Fingerprints;
use crate::manifest::HorusManifest;
use crate::native_sync::{self, NativeFileType, SyncResult};

const HORUS_TOML: &str = "horus.toml";

/// Find the horus project root by walking up from `start_dir` looking for `horus.toml`.
pub fn find_project_root(start_dir: &Path) -> Option<PathBuf> {
    let mut dir = start_dir.to_path_buf();
    for _ in 0..10 {
        if dir.join(HORUS_TOML).exists() {
            return Some(dir);
        }
        if !dir.pop() {
            break;
        }
    }
    None
}

/// Check whether the current directory is inside a horus project.
/// Used by `horus _is-project` (exit code only, no output).
pub fn run_is_project() -> bool {
    let cwd = env::current_dir().unwrap_or_default();
    find_project_root(&cwd).is_some()
}

/// Cargo subcommands that should NOT get `--manifest-path` (global commands).
/// Everything else gets it — this handles third-party subcommands (cargo-audit,
/// cargo-watch, cargo-expand, etc.) automatically.
const CARGO_NO_MANIFEST_PATH: &[&str] = &[
    "install",
    "uninstall",
    "login",
    "logout",
    "search",
    "help",
    "version",
    "new",
    "init",
    "+stable",
    "+nightly",
    "+beta",
];

/// Find the real native tool binary, skipping any horus shim.
fn find_real_tool(tool_name: &str) -> Result<PathBuf> {
    // Check env var override first (set by horus env.sh)
    let env_key = format!("HORUS_REAL_{}", tool_name.to_uppercase());
    if let Ok(path) = env::var(&env_key) {
        let p = PathBuf::from(&path);
        if p.exists() {
            return Ok(p);
        }
    }

    // Search PATH for the real binary (skip entries that are horus itself)
    let path_var = env::var("PATH").unwrap_or_default();
    for dir in path_var.split(':') {
        let candidate = PathBuf::from(dir).join(tool_name);
        if candidate.exists() {
            // Make sure it's not horus itself
            if let Ok(resolved) = std::fs::canonicalize(&candidate) {
                let resolved_name = resolved.file_name().and_then(|n| n.to_str()).unwrap_or("");
                if resolved_name == "horus" {
                    continue;
                }
            }
            return Ok(candidate);
        }
    }

    bail!("Could not find `{}` in PATH. Is it installed?", tool_name)
}

/// Run the sync-before/after cycle: generate, fingerprint, exec, sync-back.
fn run_with_sync(
    project_dir: &Path,
    file_type: NativeFileType,
    tool_name: &str,
    mut cmd: Command,
) -> Result<i32> {
    let manifest_path = project_dir.join(HORUS_TOML);
    let manifest = HorusManifest::load_from(&manifest_path).context("Failed to load horus.toml")?;

    // 1. Generate native file from horus.toml
    let mut fingerprints = Fingerprints::load(project_dir).unwrap_or_default();
    let content = generate_native(&manifest, project_dir, file_type)?;
    fingerprints.record(file_type.filename(), &content);

    // For workspace projects, also fingerprint each member's generated Cargo.toml
    if file_type.filename() == "Cargo.toml" && manifest.is_workspace() {
        if let Some(ref ws) = manifest.workspace {
            if let Ok(members) = crate::manifest::resolve_workspace_members(ws, project_dir) {
                for (_, member_manifest) in &members {
                    let name = crate::cargo_gen::sanitize_cargo_name(&member_manifest.package.name);
                    let member_path = project_dir.join(".horus").join(&name).join("Cargo.toml");
                    if let Ok(member_content) = std::fs::read_to_string(&member_path) {
                        fingerprints.record(&format!("{}/Cargo.toml", name), &member_content);
                    }
                }
            }
        }
    }

    let _ = fingerprints.save(project_dir);

    // 2. Execute the real tool (may take a long time — no lock held here)
    let status = cmd
        .status()
        .with_context(|| format!("Failed to execute `{}`", tool_name))?;

    // 3. Check if the native file was modified by the tool
    if fingerprints.is_modified(file_type.filename(), project_dir) {
        // Acquire lock for the sync-back cycle to prevent race conditions
        let _lock = crate::fingerprint::SyncLock::try_acquire(
            project_dir,
            std::time::Duration::from_secs(5),
        );

        // Re-load manifest inside lock (another terminal may have modified it)
        let mut manifest = HorusManifest::load_from(&manifest_path)
            .context("Failed to reload horus.toml for sync")?;

        match native_sync::sync_from_native(
            project_dir,
            &mut manifest,
            file_type,
            &mut fingerprints,
        )? {
            SyncResult::Synced {
                added,
                removed,
                modified,
            } => {
                manifest.save_to(&manifest_path)?;
                if added > 0 {
                    eprintln!("  {} Synced {} new dep(s) to horus.toml", "⟳".cyan(), added);
                }
                if removed > 0 {
                    eprintln!(
                        "  {} Removed {} dep(s) from horus.toml",
                        "⟳".cyan(),
                        removed
                    );
                }
                if modified > 0 {
                    eprintln!("  {} Updated {} dep(s) in horus.toml", "⟳".cyan(), modified);
                }
                // Re-generate to normalize the file
                let content = generate_native(&manifest, project_dir, file_type)?;
                fingerprints.record(file_type.filename(), &content);
                let _ = fingerprints.save(project_dir);
            }
            SyncResult::NoChanges => {}
        }
        // _lock dropped here, releasing flock
    }

    Ok(status.code().unwrap_or(1))
}

/// Generate a native file and return its content.
fn generate_native(
    manifest: &HorusManifest,
    project_dir: &Path,
    file_type: NativeFileType,
) -> Result<String> {
    match file_type {
        NativeFileType::Cargo => {
            let (_, content) =
                crate::cargo_gen::generate_for_manifest(manifest, project_dir, &[], false)?;
            Ok(content)
        }
        NativeFileType::Pyproject => {
            let (_, content) = crate::pyproject_gen::generate(manifest, project_dir, false)?;
            Ok(content)
        }
        NativeFileType::Cmake => {
            let (_, content) = crate::cmake_gen::generate(manifest, project_dir, false)?;
            Ok(content)
        }
    }
}

/// Check if a cargo subcommand should be excluded from `--manifest-path` injection.
fn is_cargo_global_command(subcmd: &str) -> bool {
    // Toolchain overrides like +nightly
    if subcmd.starts_with('+') {
        return true;
    }
    // Bare flags like --version, --help, -V, -h
    if subcmd.starts_with('-') {
        return true;
    }
    CARGO_NO_MANIFEST_PATH.contains(&subcmd)
}

// ── Cargo proxy ────────────────────────────────────────────────────────────

/// Transparent cargo proxy: `horus cargo <args>`.
pub fn run_cargo_proxy(args: Vec<String>) -> Result<i32> {
    let cwd = env::current_dir()?;
    let Some(project_dir) = find_project_root(&cwd) else {
        // Not a horus project — pass through to real cargo
        let real = find_real_tool("cargo")?;
        let status = Command::new(&real).args(&args).status()?;
        return Ok(status.code().unwrap_or(1));
    };

    let real_cargo = find_real_tool("cargo")?;
    let manifest_arg = project_dir
        .join(".horus/Cargo.toml")
        .to_string_lossy()
        .to_string();
    let target_dir_arg = project_dir
        .join(".horus/target")
        .to_string_lossy()
        .to_string();

    // Build the command with --manifest-path and CARGO_TARGET_DIR injected
    let mut cmd = Command::new(&real_cargo);

    // Always set CARGO_TARGET_DIR so artifacts go to .horus/target/
    cmd.env("CARGO_TARGET_DIR", &target_dir_arg);

    let subcommand = args.first().map(|s| s.as_str()).unwrap_or("");
    let already_has_manifest = args.iter().any(|a| a.starts_with("--manifest-path"));

    if !is_cargo_global_command(subcommand) && !already_has_manifest {
        // Inject --manifest-path right after the subcommand
        if let Some(subcmd) = args.first() {
            cmd.arg(subcmd);
            cmd.arg("--manifest-path").arg(&manifest_arg);
            cmd.args(&args[1..]);
        }
    } else {
        cmd.args(&args);
    }

    let result = run_with_sync(&project_dir, NativeFileType::Cargo, "cargo", cmd)?;

    // For workspace projects, also sync per-member Cargo.toml changes
    let manifest_path = project_dir.join(HORUS_TOML);
    if let Ok(manifest) = HorusManifest::load_from(&manifest_path) {
        if manifest.is_workspace() {
            let mut fingerprints = Fingerprints::load(&project_dir).unwrap_or_default();
            match native_sync::sync_workspace_members(&project_dir, &manifest, &mut fingerprints) {
                Ok(SyncResult::Synced {
                    added,
                    removed,
                    modified,
                }) => {
                    if added > 0 {
                        eprintln!("  {} Synced {} new member dep(s)", "⟳".cyan(), added);
                    }
                    if removed > 0 {
                        eprintln!("  {} Removed {} member dep(s)", "⟳".cyan(), removed);
                    }
                    if modified > 0 {
                        eprintln!("  {} Updated {} member dep(s)", "⟳".cyan(), modified);
                    }
                    let _ = fingerprints.save(&project_dir);
                }
                Ok(SyncResult::NoChanges) => {}
                Err(e) => log::warn!("Member sync failed: {}", e),
            }
        }
    }

    Ok(result)
}

// ── Pip proxy ──────────────────────────────────────────────────────────────

/// Transparent pip proxy: `horus pip <args>`.
pub fn run_pip_proxy(args: Vec<String>) -> Result<i32> {
    let cwd = env::current_dir()?;
    let Some(project_dir) = find_project_root(&cwd) else {
        let real = find_real_tool("pip")?;
        let status = Command::new(&real).args(&args).status()?;
        return Ok(status.code().unwrap_or(1));
    };

    let real_pip = find_real_tool("pip")?;
    let subcmd = args.first().map(|s| s.as_str()).unwrap_or("");

    // --- pip install ---
    if subcmd == "install" {
        return run_pip_install(&project_dir, &real_pip, &args);
    }

    // --- pip uninstall ---
    if subcmd == "uninstall" {
        return run_pip_uninstall(&project_dir, &real_pip, &args);
    }

    // --- everything else: pass through ---
    let mut cmd = Command::new(&real_pip);
    cmd.args(&args);
    let status = cmd.status()?;
    Ok(status.code().unwrap_or(1))
}

/// Handle `pip install` — add installed packages to horus.toml.
fn run_pip_install(project_dir: &Path, real_pip: &Path, args: &[String]) -> Result<i32> {
    let mut cmd = Command::new(real_pip);

    // Rewrite `pip install -e .` → `pip install -e .horus/`
    let mut rewritten_args: Vec<String> = args.to_vec();
    let is_editable = args.iter().any(|a| a == "-e" || a == "--editable");

    for i in 0..rewritten_args.len() {
        if (rewritten_args[i] == "-e" || rewritten_args[i] == "--editable")
            && rewritten_args.get(i + 1).map(|s| s.as_str()) == Some(".")
        {
            rewritten_args[i + 1] = project_dir.join(".horus").to_string_lossy().to_string();
        }
    }

    cmd.args(&rewritten_args);
    let status = cmd.status()?;

    // Sync installed packages to horus.toml (skip for editable installs of self)
    if status.success() && !is_editable {
        let manifest_path = project_dir.join(HORUS_TOML);
        if let Ok(mut manifest) = HorusManifest::load_from(&manifest_path) {
            let mut synced = 0;

            // Collect all package specs from args + requirements files
            let specs = collect_pip_specs(&args[1..]);
            let is_upgrade = args.iter().any(|a| a == "--upgrade" || a == "-U");

            for spec in &specs {
                let dep = parse_pip_to_dep(spec);
                if let Some((name, dep_value)) = dep {
                    match manifest.dependencies.entry(name) {
                        std::collections::btree_map::Entry::Vacant(e) => {
                            e.insert(dep_value);
                            synced += 1;
                        }
                        std::collections::btree_map::Entry::Occupied(mut e) => {
                            if is_upgrade {
                                // Update existing dep on --upgrade
                                e.insert(dep_value);
                                synced += 1;
                            }
                        }
                    }
                }
            }
            if synced > 0 {
                let _ = manifest.save_to(&manifest_path);
                eprintln!(
                    "  {} Synced {} new dep(s) to horus.toml",
                    "⟳".cyan(),
                    synced
                );
            }
        }
    }

    Ok(status.code().unwrap_or(1))
}

/// Handle `pip uninstall` — remove uninstalled packages from horus.toml.
fn run_pip_uninstall(project_dir: &Path, real_pip: &Path, args: &[String]) -> Result<i32> {
    // Collect package names before running uninstall
    let packages: Vec<String> = args[1..]
        .iter()
        .filter(|a| !a.starts_with('-'))
        .cloned()
        .collect();

    let mut cmd = Command::new(real_pip);
    cmd.args(args);
    let status = cmd.status()?;

    if status.success() && !packages.is_empty() {
        let manifest_path = project_dir.join(HORUS_TOML);
        if let Ok(mut manifest) = HorusManifest::load_from(&manifest_path) {
            let mut removed = 0;
            for pkg in &packages {
                let name = pkg.to_lowercase().replace('-', "_");
                // Try both forms: hyphenated and underscored
                let removed_key = if manifest.dependencies.remove(pkg).is_some() {
                    Some(pkg.clone())
                } else if manifest.dependencies.remove(&name).is_some() {
                    Some(name)
                } else {
                    // Try the other normalization
                    let alt = pkg.replace('_', "-");
                    if manifest.dependencies.remove(&alt).is_some() {
                        Some(alt)
                    } else {
                        None
                    }
                };
                if removed_key.is_some() {
                    removed += 1;
                }
            }
            if removed > 0 {
                let _ = manifest.save_to(&manifest_path);
                eprintln!(
                    "  {} Removed {} dep(s) from horus.toml",
                    "⟳".cyan(),
                    removed
                );
            }
        }
    }

    Ok(status.code().unwrap_or(1))
}

/// Collect all pip package specs from command args, expanding -r requirements files.
fn collect_pip_specs(args: &[String]) -> Vec<String> {
    let mut specs = Vec::new();
    let mut skip_next = false;
    let mut i = 0;
    while i < args.len() {
        let arg = &args[i];
        if skip_next {
            skip_next = false;
            i += 1;
            continue;
        }
        if arg.starts_with('-') {
            if arg == "-r" || arg == "--requirement" {
                // Read the requirements file and add each line as a spec
                if let Some(req_file) = args.get(i + 1) {
                    if let Ok(content) = std::fs::read_to_string(req_file) {
                        for line in content.lines() {
                            let line = line.trim();
                            if line.is_empty() || line.starts_with('#') || line.starts_with('-') {
                                continue;
                            }
                            specs.push(line.to_string());
                        }
                    }
                }
                skip_next = true;
            } else if matches!(
                arg.as_str(),
                "-c" | "--constraint"
                    | "-e"
                    | "--editable"
                    | "-t"
                    | "--target"
                    | "-i"
                    | "--index-url"
                    | "--extra-index-url"
                    | "--prefix"
                    | "--root"
                    | "--src"
            ) {
                skip_next = true;
            }
            i += 1;
            continue;
        }
        specs.push(arg.clone());
        i += 1;
    }
    specs
}

/// Parse a pip spec string into a (name, DependencyValue) pair for horus.toml.
/// Handles regular packages, git+https:// URLs, and ./local-path installs.
fn parse_pip_to_dep(spec: &str) -> Option<(String, crate::manifest::DependencyValue)> {
    let spec = spec.trim();

    // Git URL: git+https://github.com/org/repo
    if spec.starts_with("git+") || spec.starts_with("hg+") || spec.starts_with("svn+") {
        let url = spec
            .strip_prefix("git+")
            .or_else(|| spec.strip_prefix("hg+"))
            .or_else(|| spec.strip_prefix("svn+"))
            .unwrap_or(spec);
        // Extract name from URL (last path segment, strip .git)
        let name = url
            .rsplit('/')
            .next()
            .unwrap_or("unknown")
            .split('@') // strip @branch/tag
            .next()
            .unwrap_or("unknown")
            .trim_end_matches(".git")
            .to_string();
        if name.is_empty() || name == "unknown" {
            return None;
        }
        return Some((
            name,
            crate::manifest::DependencyValue::Detailed(crate::manifest::DetailedDependency {
                source: Some(crate::manifest::DepSource::Git),
                git: Some(url.to_string()),
                ..crate::manifest::DetailedDependency::default()
            }),
        ));
    }

    // Local path: ./something or ../something
    if spec.starts_with("./") || spec.starts_with("../") || spec.starts_with('/') {
        let path = std::path::Path::new(spec);
        let name = path
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("local-dep")
            .to_string();
        return Some((
            name,
            crate::manifest::DependencyValue::Detailed(crate::manifest::DetailedDependency {
                source: Some(crate::manifest::DepSource::Path),
                path: Some(spec.to_string()),
                ..crate::manifest::DetailedDependency::default()
            }),
        ));
    }

    // Regular package: numpy>=1.24 or uvicorn[standard]>=0.20
    let (name, version, extras) = parse_pip_spec(spec);
    if name.is_empty() {
        return None;
    }

    Some((
        name,
        crate::manifest::DependencyValue::Detailed(crate::manifest::DetailedDependency {
            version,
            source: Some(crate::manifest::DepSource::PyPI),
            features: extras,
            ..crate::manifest::DetailedDependency::default()
        }),
    ))
}

/// Parse a pip package specifier like "numpy>=1.24" or "uvicorn[standard]>=0.20"
/// into (name, version, extras).
fn parse_pip_spec(spec: &str) -> (String, Option<String>, Vec<String>) {
    let spec = spec.trim();
    let split_at = spec
        .find(['>', '<', '=', '!', '~', '['])
        .unwrap_or(spec.len());
    let name = spec[..split_at].trim().to_string();

    let mut extras = Vec::new();
    let mut version = None;

    if split_at < spec.len() {
        let rest = &spec[split_at..];
        // Extract extras from [extra1,extra2]
        if rest.starts_with('[') {
            if let Some(bracket_end) = rest.find(']') {
                let extras_str = &rest[1..bracket_end];
                extras = extras_str
                    .split(',')
                    .map(|s| s.trim().to_string())
                    .filter(|s| !s.is_empty())
                    .collect();
                let after_bracket = rest[bracket_end + 1..].trim();
                if !after_bracket.is_empty() {
                    version = Some(after_bracket.to_string());
                }
            }
        } else {
            version = Some(rest.trim().to_string());
        }
    }

    (name, version, extras)
}

// ── CMake proxy ────────────────────────────────────────────────────────────

/// Transparent cmake proxy: `horus cmake <args>`.
pub fn run_cmake_proxy(args: Vec<String>) -> Result<i32> {
    let cwd = env::current_dir()?;
    let Some(project_dir) = find_project_root(&cwd) else {
        let real = find_real_tool("cmake")?;
        let status = Command::new(&real).args(&args).status()?;
        return Ok(status.code().unwrap_or(1));
    };

    let real_cmake = find_real_tool("cmake")?;
    let mut cmd = Command::new(&real_cmake);

    // Rewrite source dir `.` → `.horus/` and ensure build dir
    let mut rewritten_args = args.clone();
    let mut has_build_dir = false;
    for i in 0..rewritten_args.len() {
        if rewritten_args[i] == "-B" {
            has_build_dir = true;
        }
        if rewritten_args[i] == "-S" {
            if let Some(next) = rewritten_args.get(i + 1) {
                if next == "." {
                    rewritten_args[i + 1] =
                        project_dir.join(".horus").to_string_lossy().to_string();
                }
            }
        }
        // Bare "." as source directory
        if rewritten_args[i] == "." && (i == 0 || !rewritten_args[i - 1].starts_with('-')) {
            rewritten_args[i] = project_dir.join(".horus").to_string_lossy().to_string();
        }
    }

    if !has_build_dir {
        rewritten_args.push("-B".to_string());
        rewritten_args.push(
            project_dir
                .join(".horus/cpp-build")
                .to_string_lossy()
                .to_string(),
        );
    }

    cmd.args(&rewritten_args);

    run_with_sync(&project_dir, NativeFileType::Cmake, "cmake", cmd)
}

// ── Conan proxy ────────────────────────────────────────────────────────────

/// Transparent conan proxy: `horus conan <args>`.
pub fn run_conan_proxy(args: Vec<String>) -> Result<i32> {
    let cwd = env::current_dir()?;
    let Some(project_dir) = find_project_root(&cwd) else {
        let real = find_real_tool("conan")?;
        let status = Command::new(&real).args(&args).status()?;
        return Ok(status.code().unwrap_or(1));
    };

    let real_conan = find_real_tool("conan")?;
    let mut cmd = Command::new(&real_conan);
    cmd.args(&args);
    let status = cmd.status()?;

    // Sync on `conan install <reference>` success
    let subcmd = args.first().map(|s| s.as_str()).unwrap_or("");
    if status.success() && subcmd == "install" {
        let manifest_path = project_dir.join(HORUS_TOML);
        if let Ok(mut manifest) = HorusManifest::load_from(&manifest_path) {
            let mut synced = 0;
            for arg in &args[1..] {
                if arg.starts_with('-') {
                    continue;
                }
                // Conan reference: name/version@user/channel or name/version@
                if let Some((name, version)) = parse_conan_ref(arg) {
                    if let std::collections::btree_map::Entry::Vacant(e) =
                        manifest.dependencies.entry(name)
                    {
                        e.insert(crate::manifest::DependencyValue::Detailed(
                            crate::manifest::DetailedDependency {
                                version: Some(version),
                                source: Some(crate::manifest::DepSource::System),
                                lang: Some("cpp".to_string()),
                                ..crate::manifest::DetailedDependency::default()
                            },
                        ));
                        synced += 1;
                    }
                }
            }
            if synced > 0 {
                let _ = manifest.save_to(&manifest_path);
                eprintln!(
                    "  {} Synced {} conan dep(s) to horus.toml",
                    "⟳".cyan(),
                    synced
                );
            }
        }
    }

    Ok(status.code().unwrap_or(1))
}

/// Parse a conan reference like "fmt/10.0.0@" or "boost/1.82.0@user/stable".
fn parse_conan_ref(reference: &str) -> Option<(String, String)> {
    let parts: Vec<&str> = reference.split('/').collect();
    if parts.len() >= 2 {
        let name = parts[0].to_string();
        let version = parts[1].split('@').next().unwrap_or("*").to_string();
        if !name.is_empty() && !version.is_empty() {
            return Some((name, version));
        }
    }
    None
}

// ── vcpkg proxy ────────────────────────────────────────────────────────────

/// Transparent vcpkg proxy: `horus vcpkg <args>`.
pub fn run_vcpkg_proxy(args: Vec<String>) -> Result<i32> {
    let cwd = env::current_dir()?;
    let Some(project_dir) = find_project_root(&cwd) else {
        let real = find_real_tool("vcpkg")?;
        let status = Command::new(&real).args(&args).status()?;
        return Ok(status.code().unwrap_or(1));
    };

    let real_vcpkg = find_real_tool("vcpkg")?;
    let mut cmd = Command::new(&real_vcpkg);
    cmd.args(&args);
    let status = cmd.status()?;

    // Sync on `vcpkg install <package>` success
    let subcmd = args.first().map(|s| s.as_str()).unwrap_or("");
    if status.success() && subcmd == "install" {
        let manifest_path = project_dir.join(HORUS_TOML);
        if let Ok(mut manifest) = HorusManifest::load_from(&manifest_path) {
            let mut synced = 0;
            for arg in &args[1..] {
                if arg.starts_with('-') {
                    continue;
                }
                // vcpkg package: "fmt" or "fmt:x64-linux" (strip triplet)
                let name = arg.split(':').next().unwrap_or(arg).to_string();
                if !name.is_empty() && !manifest.dependencies.contains_key(&name) {
                    manifest.dependencies.insert(
                        name,
                        crate::manifest::DependencyValue::Detailed(
                            crate::manifest::DetailedDependency {
                                source: Some(crate::manifest::DepSource::System),
                                lang: Some("cpp".to_string()),
                                ..crate::manifest::DetailedDependency::default()
                            },
                        ),
                    );
                    synced += 1;
                }
            }
            if synced > 0 {
                let _ = manifest.save_to(&manifest_path);
                eprintln!(
                    "  {} Synced {} vcpkg dep(s) to horus.toml",
                    "⟳".cyan(),
                    synced
                );
            }
        }
    }

    Ok(status.code().unwrap_or(1))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn find_project_root_no_project() {
        let tmp = tempfile::tempdir().unwrap();
        assert!(find_project_root(tmp.path()).is_none());
    }

    #[test]
    fn find_project_root_found() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"",
        )
        .unwrap();
        assert_eq!(
            find_project_root(tmp.path()),
            Some(tmp.path().to_path_buf())
        );
    }

    #[test]
    fn parse_pip_spec_simple() {
        let (name, ver, extras) = parse_pip_spec("numpy");
        assert_eq!(name, "numpy");
        assert!(ver.is_none());
        assert!(extras.is_empty());
    }

    #[test]
    fn parse_pip_spec_versioned() {
        let (name, ver, extras) = parse_pip_spec("numpy>=1.24");
        assert_eq!(name, "numpy");
        assert_eq!(ver, Some(">=1.24".to_string()));
        assert!(extras.is_empty());
    }

    #[test]
    fn parse_pip_spec_extras() {
        let (name, ver, extras) = parse_pip_spec("uvicorn[standard]>=0.20");
        assert_eq!(name, "uvicorn");
        assert_eq!(ver, Some(">=0.20".to_string()));
        assert_eq!(extras, vec!["standard"]);
    }

    #[test]
    fn parse_pip_spec_multiple_extras() {
        let (name, ver, extras) = parse_pip_spec("fastapi[all,jinja]>=0.100");
        assert_eq!(name, "fastapi");
        assert_eq!(ver, Some(">=0.100".to_string()));
        assert_eq!(extras, vec!["all", "jinja"]);
    }

    #[test]
    fn parse_pip_spec_extras_no_version() {
        let (name, ver, extras) = parse_pip_spec("uvicorn[standard]");
        assert_eq!(name, "uvicorn");
        assert!(ver.is_none());
        assert_eq!(extras, vec!["standard"]);
    }

    #[test]
    fn parse_pip_to_dep_git_url() {
        let result = parse_pip_to_dep("git+https://github.com/org/my-lib");
        let (name, dep) = result.unwrap();
        assert_eq!(name, "my-lib");
        assert_eq!(dep.effective_source(), crate::manifest::DepSource::Git);
    }

    #[test]
    fn parse_pip_to_dep_local_path() {
        let result = parse_pip_to_dep("./libs/my-pkg");
        let (name, dep) = result.unwrap();
        assert_eq!(name, "my-pkg");
        assert_eq!(dep.effective_source(), crate::manifest::DepSource::Path);
    }

    #[test]
    fn collect_pip_specs_with_requirements() {
        let tmp = tempfile::tempdir().unwrap();
        let req_path = tmp.path().join("requirements.txt");
        std::fs::write(&req_path, "numpy>=1.24\n# comment\nrequests\n-e .\n").unwrap();
        let args = vec![
            "-r".to_string(),
            req_path.to_string_lossy().to_string(),
            "torch".to_string(),
        ];
        let specs = collect_pip_specs(&args);
        assert_eq!(specs, vec!["numpy>=1.24", "requests", "torch"]);
    }

    #[test]
    fn cargo_global_commands_detected() {
        assert!(is_cargo_global_command("install"));
        assert!(is_cargo_global_command("new"));
        assert!(is_cargo_global_command("+nightly"));
        assert!(is_cargo_global_command("--version"));
        assert!(is_cargo_global_command("-V"));
        assert!(!is_cargo_global_command("build"));
        assert!(!is_cargo_global_command("test"));
        assert!(!is_cargo_global_command("audit")); // third-party, gets manifest-path
        assert!(!is_cargo_global_command("watch")); // third-party
        assert!(!is_cargo_global_command("expand")); // third-party
    }

    #[test]
    fn conan_ref_parsing() {
        let (name, ver) = parse_conan_ref("fmt/10.0.0@").unwrap();
        assert_eq!(name, "fmt");
        assert_eq!(ver, "10.0.0");
    }

    #[test]
    fn conan_ref_with_user_channel() {
        let (name, ver) = parse_conan_ref("boost/1.82.0@user/stable").unwrap();
        assert_eq!(name, "boost");
        assert_eq!(ver, "1.82.0");
    }

    #[test]
    fn conan_ref_invalid() {
        assert!(parse_conan_ref("just-a-name").is_none());
    }

    #[test]
    fn vcpkg_strips_triplet() {
        let name = "fmt:x64-linux".split(':').next().unwrap();
        assert_eq!(name, "fmt");
    }

    #[test]
    fn vcpkg_no_triplet() {
        let name = "fmt".split(':').next().unwrap();
        assert_eq!(name, "fmt");
    }
}
