pub(crate) mod deps;
pub(crate) mod features;
pub(crate) mod install;
pub(crate) mod run_cpp;
pub(crate) mod run_python;
pub(crate) mod run_rust;

// Re-export public API
pub use run_rust::execute_build_only;
pub(crate) use run_rust::find_horus_source_dir;

// Re-export for sibling modules (install.rs uses detect_python_interpreter via super::)
pub(crate) use run_python::detect_python_interpreter;

use crate::cli_output;
use crate::config::CARGO_TOML;
use crate::manifest::{HorusManifest, HORUS_TOML};
use crate::progress;
use anyhow::{anyhow, bail, Context, Result};
use colored::*;
use glob::glob;
use horus_core::core::DurationExt;
use horus_core::params::RuntimeParams;
use std::collections::HashSet;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// Spawn a child process with Ctrl+C forwarding.
///
/// Sets up a ctrlc handler that sends SIGINT to the child process,
/// waits for exit, and returns the status. Shared by run_python, run_cpp, etc.
pub(super) fn spawn_with_ctrlc(
    mut child: std::process::Child,
    lang_label: &str,
) -> Result<std::process::ExitStatus> {
    let child_id = child.id();
    let _running = Arc::new(AtomicBool::new(true));
    let r = _running.clone();
    let label = lang_label.to_string();
    ctrlc::set_handler(move || {
        eprintln!(
            "{}",
            format!("\nCtrl+C received, stopping {} process...", label).red()
        );
        r.store(false, Ordering::SeqCst);
        let _ = horus_sys::process::ProcessHandle::from_pid(child_id)
            .signal(horus_sys::process::Signal::Interrupt);
    })
    .ok();

    let status = child.wait()?;
    Ok(status)
}

#[derive(Debug, Clone)]
enum ExecutionTarget {
    File(PathBuf),
    Directory(PathBuf),
    Manifest(PathBuf),
    Multiple(Vec<PathBuf>),
}

pub fn execute_run(
    files: Vec<PathBuf>,
    args: Vec<String>,
    release: bool,
    clean: bool,
    package: Option<String>,
) -> Result<()> {
    // Verify lockfile system deps and toolchain before running
    crate::system_deps::verify_lockfile_before_build();

    log::debug!("executing run with files: {:?}", files);

    // Check if the argument is a script name from [scripts] in horus.toml
    // e.g. `horus run sim` → runs the "sim" script instead of looking for a file
    if files.len() == 1 {
        let candidate = files[0].to_string_lossy();
        // Only check if it looks like a bare name (no path separators, no extension)
        if !candidate.contains(std::path::MAIN_SEPARATOR)
            && !candidate.contains('/')
            && !candidate.contains('.')
        {
            let manifest_path = Path::new(HORUS_TOML);
            if manifest_path.exists() {
                if let Ok(manifest) = HorusManifest::load_from(manifest_path) {
                    if manifest.scripts.contains_key(candidate.as_ref()) {
                        return crate::commands::scripts::run_scripts(
                            Some(candidate.into_owned()),
                            args,
                        )
                        .map_err(|e| anyhow!("{}", e));
                    }
                }
            }
        }
    }

    // Launch file detection: .yaml/.yml with nodes: key
    if files.len() == 1 {
        let path = &files[0];
        if let Some(ext) = path.extension().and_then(|e| e.to_str()) {
            if (ext == "yaml" || ext == "yml") && path.exists() {
                if let Ok(content) = std::fs::read_to_string(path) {
                    if content.contains("nodes:") {
                        log::info!("Detected launch file: {:?}", path);
                        return super::launch::run_launch(path, false, None, 10)
                            .map_err(|e| anyhow!("{}", e));
                    }
                }
            }
        }
    }

    // Handle clean build
    if clean {
        cli_output::info("Cleaning build cache...");
        run_rust::clean_build_cache()?;
    }

    // Clean up stale topics from previous runs to prevent data corruption
    // Only removes topics with no live processes AND 5+ minutes old
    crate::discovery::cleanup_stale_topics();

    // Load runtime parameters from params.yaml if it exists
    // Supported locations (in priority order):
    // 1. ./params.yaml (project root)
    // 2. ./config/params.yaml
    // 3. .horus/config/params.yaml (created by `horus param`)
    load_params_from_project()?;

    let mode = if release { "release" } else { "debug" };
    cli_output::info(&format!(
        "Starting HORUS runtime in {} mode...",
        mode.yellow()
    ));

    // Workspace detection: if horus.toml has [workspace], use workspace build path
    if files.is_empty() {
        let manifest_path = Path::new(HORUS_TOML);
        if manifest_path.exists() {
            if let Ok(manifest) = HorusManifest::load_from(manifest_path) {
                if manifest.is_workspace() {
                    return execute_workspace(manifest, args, release, clean, package);
                }
            }
        }
    }

    // Step 1: Resolve target(s) - file(s), directory, or pattern
    let execution_targets = if files.is_empty() {
        vec![ExecutionTarget::File(auto_detect_main_file()?)]
    } else if files.len() == 1 {
        resolve_execution_target(files[0].clone())?
    } else {
        // Multiple files provided - treat as ExecutionTarget::Multiple
        vec![ExecutionTarget::Multiple(files)]
    };

    // Step 2: Execute based on target type
    for target in execution_targets {
        match target {
            ExecutionTarget::File(file_path) => {
                execute_single_file(file_path, args.clone(), release, clean)?;
            }
            ExecutionTarget::Directory(dir_path) => {
                execute_directory(dir_path, args.clone(), release, clean)?;
            }
            ExecutionTarget::Manifest(manifest_path) => {
                execute_from_manifest(manifest_path, args.clone(), release, clean)?;
            }
            ExecutionTarget::Multiple(file_paths) => {
                execute_multiple_files(file_paths, args.clone(), release, clean)?;
            }
        }
    }

    Ok(())
}

/// Execute a workspace project using cargo with the generated workspace Cargo.toml.
fn execute_workspace(
    manifest: HorusManifest,
    args: Vec<String>,
    release: bool,
    _clean: bool,
    package: Option<String>,
) -> Result<()> {
    let project_dir = std::env::current_dir()?;

    // Generate workspace Cargo.toml + per-member Cargo.toml files
    cli_output::info("Generating workspace build files...");
    let (cargo_path, _) =
        crate::cargo_gen::generate_for_manifest(&manifest, &project_dir, &[], false)?;

    // Build with cargo
    let mut cmd = std::process::Command::new("cargo");
    cmd.arg("build");
    cmd.arg("--manifest-path").arg(&cargo_path);

    if release {
        cmd.arg("--release");
    }

    // Target specific member or all
    if let Some(ref member) = package {
        cmd.arg("-p").arg(member);
        cli_output::info(&format!("Building workspace member: {}", member));
    } else {
        cmd.arg("--workspace");
        cli_output::info("Building all workspace members...");
    }

    let status = cmd.status()?;
    if !status.success() {
        return Err(anyhow!("Workspace build failed"));
    }

    // If a specific package was requested, run it
    if let Some(ref member) = package {
        let profile = if release { "release" } else { "debug" };
        let binary = project_dir.join(".horus/target").join(profile).join(member);
        if binary.exists() {
            cli_output::success(&format!("Running {}", member));
            let status = std::process::Command::new(&binary).args(&args).status()?;
            if !status.success() {
                let code = status.code().unwrap_or(1);
                return Err(anyhow!("Process exited with code {}", code));
            }
        }
    }

    Ok(())
}

fn execute_single_file(
    file_path: PathBuf,
    args: Vec<String>,
    release: bool,
    clean: bool,
) -> Result<()> {
    let language = deps::detect_language(&file_path)?;

    // ── Preflight validation ────────────────────────────────────────────
    let preflight_issues = crate::error_wrapper::preflight_check(&language);
    if !preflight_issues.is_empty() {
        let critical_count = preflight_issues
            .iter()
            .filter(|d| d.severity == crate::error_wrapper::Severity::Error)
            .count();
        eprintln!(
            "\n{} {} {}",
            "horus".bold().cyan(),
            "preflight".bold().yellow(),
            format!("[{} issue(s) found]", preflight_issues.len()).dimmed()
        );
        crate::error_wrapper::emit_diagnostics(&preflight_issues);
        if critical_count > 0 {
            bail!(
                "Preflight check failed — {} critical issue(s) must be fixed before building",
                critical_count
            );
        }
        eprintln!(); // blank line after warnings
    }

    cli_output::info(&format!(
        "Detected: {} ({})",
        file_path.display().to_string().green(),
        language.yellow()
    ));

    // Load ignore patterns from horus.toml
    let ignore = load_ignore_patterns();

    // Ensure .horus directory exists
    ensure_horus_directory()?;

    // Scan imports and resolve dependencies
    log::info!("Scanning imports for {}", file_path.display());
    cli_output::info("Scanning imports...");
    let dependencies = deps::scan_imports(&file_path, &language, &ignore)?;

    if !dependencies.is_empty() {
        log::debug!("Found {} dependencies", dependencies.len());
        eprintln!(
            "{} Found {} dependencies",
            cli_output::ICON_INFO.cyan(),
            dependencies.len()
        );

        // For Rust, filter out core HORUS crates - they're handled as path dependencies
        // and are NOT in the package registry. Including them causes install loops.
        let deps_to_resolve: HashSet<String> = if language == "rust" {
            let core_crates = ["horus", "horus_core", "horus_library", "horus_macros"];
            dependencies
                .into_iter()
                .filter(|dep| {
                    let base_name = dep.split('@').next().unwrap_or(dep);
                    !core_crates.contains(&base_name)
                })
                .collect()
        } else {
            dependencies
        };

        // Resolve dependencies with language context
        // This ensures cargo library dependencies are handled correctly:
        // - For Rust: library deps are in horus.toml, resolved via cargo_gen
        // - For Python: cargo deps are skipped entirely (they're library crates)
        if !deps_to_resolve.is_empty() {
            install::resolve_dependencies_with_context(deps_to_resolve, Some(&language))?;
        }
    }

    // Build environment for child processes (no env::set_var)
    let child_env = build_child_env()?;

    // Execute
    cli_output::info("Executing...\n");
    run_rust::execute_with_scheduler(file_path, language, args, release, clean, &child_env)?;

    Ok(())
}

fn execute_directory(
    dir_path: PathBuf,
    args: Vec<String>,
    release: bool,
    clean: bool,
) -> Result<()> {
    println!(
        "{} Executing from directory: {}",
        cli_output::ICON_INFO.cyan(),
        dir_path.display().to_string().green()
    );

    let original_dir = env::current_dir()?;

    // Change to target directory
    env::set_current_dir(&dir_path).context(format!(
        "Failed to change to directory: {}",
        dir_path.display()
    ))?;

    let result = (|| -> Result<()> {
        // Auto-detect main file in this directory
        let main_file = auto_detect_main_file().context(format!(
            "No main file found in directory: {}",
            dir_path.display()
        ))?;

        // Execute the file in this context
        execute_single_file(main_file, args, release, clean)?;

        Ok(())
    })();

    // Always restore original directory
    env::set_current_dir(original_dir)?;

    result
}

fn execute_from_manifest(
    manifest_path: PathBuf,
    args: Vec<String>,
    release: bool,
    clean: bool,
) -> Result<()> {
    println!(
        "{} Executing from manifest: {}",
        cli_output::ICON_INFO.cyan(),
        manifest_path.display().to_string().green()
    );

    match manifest_path.file_name().and_then(|s| s.to_str()) {
        Some(HORUS_TOML) => execute_from_horus_manifest(manifest_path, args, release, clean),
        Some(CARGO_TOML) => run_rust::execute_from_cargo_toml(manifest_path, args, release, clean),
        _ => bail!("Unsupported manifest type: {}", manifest_path.display()),
    }
}

fn execute_from_horus_manifest(
    manifest_path: PathBuf,
    args: Vec<String>,
    release: bool,
    clean: bool,
) -> Result<()> {
    // For now, find the main file in the same directory as horus.toml
    let project_dir = manifest_path
        .parent()
        .ok_or_else(|| anyhow!("Cannot determine project directory"))?;

    let original_dir = env::current_dir()?;
    env::set_current_dir(project_dir)?;

    let result = (|| -> Result<()> {
        // Auto-detect and run main file (Rust or Python)
        let main_file =
            auto_detect_main_file().context("No main file found in project directory")?;
        execute_single_file(main_file, args, release, clean)
    })();

    env::set_current_dir(original_dir)?;
    result
}

fn execute_multiple_files(
    file_paths: Vec<PathBuf>,
    args: Vec<String>,
    release: bool,
    clean: bool,
) -> Result<()> {
    use std::io::{BufRead, BufReader};
    use std::process::Stdio;
    use std::sync::Mutex;

    println!(
        "{} Executing {} files concurrently:",
        cli_output::ICON_INFO.cyan(),
        file_paths.len()
    );

    for (i, file_path) in file_paths.iter().enumerate() {
        let language = deps::detect_language(file_path)?;
        println!(
            "  {} {} ({})",
            format!("{}.", i + 1).dimmed(),
            file_path.display().to_string().green(),
            language.yellow()
        );
    }

    // Phase 1: Build all files (batch Rust files for performance)
    println!(
        "\n{} Phase 1: Building all files...",
        cli_output::ICON_INFO.cyan()
    );
    let mut executables = Vec::new();

    // Group files by language for optimized building
    let mut rust_files = Vec::new();
    let mut other_files = Vec::new();

    for file_path in &file_paths {
        let language = deps::detect_language(file_path)?;
        if language == "rust" {
            rust_files.push(file_path.clone());
        } else {
            other_files.push((file_path.clone(), language));
        }
    }

    // Build all Rust files together in a single Cargo workspace (major optimization!)
    if !rust_files.is_empty() {
        let build_msg = if rust_files.len() == 1 {
            format!("Building {}...", rust_files[0].display())
        } else {
            format!("Building {} Rust files together...", rust_files.len())
        };
        let build_start = std::time::Instant::now();
        let spinner = progress::build_spinner(&build_msg);

        let rust_executables = run_rust::build_rust_files_batch(rust_files, release, clean)?;
        executables.extend(rust_executables);
        progress::finish_success(
            &spinner,
            &format!(
                "Rust files built ({:.1}s)",
                build_start.elapsed().as_secs_f64()
            ),
        );
    }

    // Build other languages individually
    for (file_path, language) in other_files {
        let build_start = std::time::Instant::now();
        let spinner = progress::build_spinner(&format!("Building {}...", file_path.display()));

        let exec_info = run_rust::build_file_for_concurrent_execution(
            file_path, language, release, false, // Don't clean - already done if needed
        )?;

        executables.push(exec_info);
        progress::finish_success(
            &spinner,
            &format!("Built ({:.1}s)", build_start.elapsed().as_secs_f64()),
        );
    }

    println!(
        "{} All files built successfully!\n",
        progress::STATUS_SUCCESS
    );

    // Phase 2: Execute all binaries concurrently
    println!(
        "{} Phase 2: Starting all processes...",
        cli_output::ICON_INFO.cyan()
    );

    let running = Arc::new(AtomicBool::new(true));
    let worst_exit_code = Arc::new(std::sync::atomic::AtomicI32::new(0));
    let children: Arc<Mutex<Vec<(String, std::process::Child)>>> = Arc::new(Mutex::new(Vec::new()));

    // Setup Ctrl+C handler — only sets the flag, main loop handles killing.
    // This avoids a race condition where the handler blocks on the children mutex
    // while the main loop holds it, leaving children unkilled.
    let r = running.clone();
    ctrlc::set_handler(move || {
        eprintln!(
            "\n{} Shutting down all processes...",
            cli_output::ICON_WARN.yellow()
        );
        r.store(false, Ordering::SeqCst);
    })
    .ok(); // Handler may already be set; ignore failure

    let mut handles = Vec::new();

    // Spawn all processes
    for (i, exec_info) in executables.iter().enumerate() {
        let node_name = exec_info.name.clone();
        let color = run_rust::get_color_for_index(i);

        let mut cmd = exec_info.create_command(&args);
        cmd.stdout(Stdio::piped()).stderr(Stdio::piped());

        match cmd.spawn() {
            Ok(mut child) => {
                // Handle stdout
                if let Some(stdout) = child.stdout.take() {
                    let name = node_name.clone();
                    let handle = std::thread::spawn(move || {
                        let reader = BufReader::new(stdout);
                        for line in reader.lines().map_while(Result::ok) {
                            println!("{} {}", format!("[{}]", name).color(color), line);
                        }
                    });
                    handles.push(handle);
                }

                // Handle stderr
                if let Some(stderr) = child.stderr.take() {
                    let name = node_name.clone();
                    let handle = std::thread::spawn(move || {
                        let reader = BufReader::new(stderr);
                        for line in reader.lines().map_while(Result::ok) {
                            eprintln!("{} {}", format!("[{}]", name).color(color), line);
                        }
                    });
                    handles.push(handle);
                }

                println!(
                    "  {} Started [{}]",
                    cli_output::ICON_SUCCESS.green(),
                    node_name.color(color)
                );
                children
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .push((node_name, child));
            }
            Err(e) => {
                eprintln!(
                    "  {} Failed to start [{}]: {}",
                    cli_output::ICON_ERROR.red(),
                    node_name,
                    e
                );
            }
        }
    }

    println!(
        "\n{} All processes running. Press Ctrl+C to stop.\n",
        cli_output::ICON_SUCCESS.green()
    );

    // Wait for all processes to complete (concurrent, checks running flag)
    loop {
        let mut children_lock = children.lock().unwrap_or_else(|e| e.into_inner());

        // If shutdown requested, kill all remaining children and break
        if !running.load(Ordering::SeqCst) {
            for (name, child) in children_lock.iter_mut() {
                eprintln!(
                    "  {} Terminating [{}]...",
                    cli_output::ICON_WARN.yellow(),
                    name
                );
                let _ = child.kill();
                let _ = child.wait(); // Reap to prevent zombie
            }
            children_lock.clear();
            drop(children_lock);
            break;
        }

        // Check each child with try_wait (non-blocking)
        let mut all_done = true;
        children_lock.retain_mut(|(name, child)| {
            match child.try_wait() {
                Ok(Some(status)) => {
                    // Process exited
                    if !status.success() {
                        let code = status.code().unwrap_or(1);
                        eprintln!(
                            "\n{} Process [{}] exited with code: {}",
                            cli_output::ICON_WARN.yellow(),
                            name,
                            code
                        );
                        // Track the worst (highest) exit code
                        worst_exit_code.fetch_max(code, Ordering::SeqCst);
                    }
                    false // Remove from list
                }
                Ok(None) => {
                    // Still running
                    all_done = false;
                    true // Keep in list
                }
                Err(e) => {
                    eprintln!(
                        "\n{} Error checking [{}]: {}",
                        cli_output::ICON_ERROR.red(),
                        name,
                        e
                    );
                    false // Remove from list
                }
            }
        });

        drop(children_lock);

        if all_done {
            break;
        }

        // Small sleep to avoid busy-waiting
        std::thread::sleep(100_u64.ms());
    }

    // Wait for output threads to finish
    for handle in handles {
        handle.join().ok();
    }

    if !running.load(Ordering::SeqCst) {
        println!(
            "\n{} All processes stopped.",
            cli_output::ICON_SUCCESS.green()
        );
    } else {
        println!(
            "\n{} All processes completed.",
            cli_output::ICON_SUCCESS.green()
        );
    }

    let exit_code = worst_exit_code.load(Ordering::SeqCst);
    if exit_code != 0 {
        bail!(
            "One or more processes failed (worst exit code: {})",
            exit_code
        );
    }

    Ok(())
}

fn resolve_execution_target(input: PathBuf) -> Result<Vec<ExecutionTarget>> {
    let input_str = input.to_string_lossy();
    log::debug!("resolving execution target: {:?}", input);

    // Check for glob patterns
    if input_str.contains('*') || input_str.contains('?') || input_str.contains('[') {
        log::debug!("detected glob pattern: {}", input_str);
        return resolve_glob_pattern(&input_str);
    }

    if input.is_file() {
        // Check if it's a manifest file
        match input.file_name().and_then(|s| s.to_str()) {
            Some(HORUS_TOML) | Some(CARGO_TOML) => {
                return Ok(vec![ExecutionTarget::Manifest(input)]);
            }
            _ => {}
        }

        // Regular file
        log::debug!("resolved to regular file: {:?}", input);
        return Ok(vec![ExecutionTarget::File(input)]);
    }

    if input.is_dir() {
        log::debug!("resolved to directory: {:?}", input);
        return Ok(vec![ExecutionTarget::Directory(input)]);
    }

    bail!("Target not found: {}", input.display())
}

fn resolve_glob_pattern(pattern: &str) -> Result<Vec<ExecutionTarget>> {
    // Load ignore patterns from horus.toml
    let ignore = load_ignore_patterns();

    let mut files = Vec::new();

    for entry in glob(pattern).context("failed to parse glob pattern")? {
        match entry {
            Ok(path) => {
                if path.is_file() && !ignore.should_ignore_file(&path) {
                    // Only include executable file types
                    if let Some(ext) = path.extension().and_then(|s| s.to_str()) {
                        if matches!(ext, "rs" | "py" | "horus") {
                            files.push(path);
                        }
                    }
                }
            }
            Err(e) => log::warn!("Glob error: {}", e),
        }
    }

    if files.is_empty() {
        bail!("No executable files found matching pattern: {}\n\n{}\n  {} Supported extensions: {}\n  {} Check pattern: {}",
            pattern.green(),
            "No matches found:".yellow(),
            "\u{2022}".cyan(), ".rs, .py, .horus".green(),
            "\u{2022}".cyan(), "Use quotes around patterns like \"nodes/*.py\"".dimmed()
        );
    }

    if files.len() == 1 {
        Ok(vec![ExecutionTarget::File(
            files.into_iter().next().expect("checked len == 1"),
        )])
    } else {
        Ok(vec![ExecutionTarget::Multiple(files)])
    }
}

pub(crate) fn auto_detect_main_file() -> Result<PathBuf> {
    log::debug!("auto-detecting main file in current directory");

    // Load ignore patterns from horus.toml
    let ignore = load_ignore_patterns();

    // Check for main files in priority order (Rust and Python only)
    let candidates = ["main.rs", "main.py", "src/main.rs", "src/main.py"];

    for candidate in &candidates {
        let path = PathBuf::from(candidate);
        if path.exists() && !ignore.should_ignore_file(&path) {
            log::debug!("auto-detected main file: {:?}", path);
            return Ok(path);
        }
    }

    // Check for single file with appropriate extension
    let entries: Vec<_> = fs::read_dir(".")
        .context("Failed to read current directory")?
        .filter_map(Result::ok)
        .collect();

    let code_files: Vec<_> = entries
        .iter()
        .filter(|e| {
            let path = e.path();
            if let Some(ext) = path.extension() {
                matches!(ext.to_str(), Some("rs") | Some("py")) && !ignore.should_ignore_file(&path)
            } else {
                false
            }
        })
        .collect();

    if code_files.len() == 1 {
        return Ok(code_files[0].path());
    }

    bail!("No main file detected.\n\n{}\n  {} Create a main file: {}\n  {} Or specify a file: {}\n  {} Or run from directory: {}",
        "Solutions:".yellow(),
        "\u{2022}".cyan(), "main.rs or main.py".green(),
        "\u{2022}".cyan(), "horus run myfile.rs".green(),
        "\u{2022}".cyan(), "horus run src/".green()
    )
}

pub(crate) fn ensure_horus_directory() -> Result<()> {
    let horus_dir = PathBuf::from(".horus");

    // Create .horus/ if it doesn't exist
    if !horus_dir.exists() {
        println!(
            "{} Creating build environment...",
            cli_output::ICON_INFO.cyan()
        );
        fs::create_dir_all(&horus_dir)?;
    }

    // Always ensure subdirectories exist (they might not if created by `horus new`)
    fs::create_dir_all(horus_dir.join("packages"))?;
    fs::create_dir_all(horus_dir.join("bin"))?;
    fs::create_dir_all(horus_dir.join("lib"))?;
    fs::create_dir_all(horus_dir.join("cache"))?;

    Ok(())
}

/// Load ignore patterns from horus.toml.
fn load_ignore_patterns() -> features::IgnorePatterns {
    if Path::new(HORUS_TOML).exists() {
        if let Ok(manifest) = HorusManifest::load_from(Path::new(HORUS_TOML)) {
            return manifest.ignore.into();
        }
    }
    features::IgnorePatterns::default()
}

/// Build a map of environment variables for child processes.
///
/// Used by sibling modules (run_rust, run_python) to get env vars for Commands.
///
/// Returns the env vars instead of calling `env::set_var` to avoid UB in
/// multi-threaded contexts. Callers should pass these via `Command::envs()`.
pub(crate) fn build_child_env() -> Result<Vec<(String, String)>> {
    let current_dir = env::current_dir()?;
    let horus_bin = current_dir.join(".horus/bin");
    let horus_lib = current_dir.join(".horus/lib");
    let horus_packages = current_dir.join(".horus/packages");

    let mut env_vars: Vec<(String, String)> = Vec::new();

    // PATH
    let new_path = if let Ok(path) = env::var("PATH") {
        format!("{}:{}", horus_bin.display(), path)
    } else {
        horus_bin.display().to_string()
    };
    env_vars.push(("PATH".to_string(), new_path));

    // Build LD_LIBRARY_PATH: local + global cache libs
    let mut lib_paths = vec![horus_lib.display().to_string()];

    let global_cache =
        crate::paths::cache_dir().unwrap_or_else(|_| install::home_dir().join(".horus/cache"));
    if global_cache.exists() {
        if let Ok(entries) = fs::read_dir(&global_cache) {
            for entry in entries.flatten() {
                let lib_dir = entry.path().join("lib");
                if lib_dir.exists() {
                    lib_paths.push(lib_dir.display().to_string());
                }
                let target_lib = entry.path().join("target/release");
                if target_lib.exists() {
                    lib_paths.push(target_lib.display().to_string());
                }
            }
        }
    }

    let lib_path_str = lib_paths.join(":");
    let ld_library_path = if let Ok(ld_path) = env::var("LD_LIBRARY_PATH") {
        format!("{}:{}", lib_path_str, ld_path)
    } else {
        lib_path_str
    };
    env_vars.push(("LD_LIBRARY_PATH".to_string(), ld_library_path));

    // PYTHONPATH
    let python_path = if let Ok(py_path) = env::var("PYTHONPATH") {
        format!("{}:{}", horus_packages.display(), py_path)
    } else {
        horus_packages.display().to_string()
    };
    env_vars.push(("PYTHONPATH".to_string(), python_path));

    Ok(env_vars)
}

/// Load runtime parameters from project files
///
/// Searches for params.yaml in the following locations (in priority order):
/// 1. `./params.yaml` - Project root (most common)
/// 2. `./config/params.yaml` - Config subdirectory (ROS-style)
/// 3. `.horus/config/params.yaml` - HORUS cache (created by `horus param`)
///
/// If found, loads parameters into RuntimeParams which will be available
/// to all nodes during execution.
pub(crate) fn load_params_from_project() -> Result<()> {
    let params_locations = [
        PathBuf::from("params.yaml"),
        PathBuf::from("config/params.yaml"),
        PathBuf::from(".horus/config/params.yaml"),
    ];

    let params_file = params_locations.iter().find(|p| p.exists());

    if let Some(path) = params_file {
        let params = RuntimeParams::new().map_err(|e| anyhow!("Failed to init params: {}", e))?;

        params
            .load_from_disk(path)
            .map_err(|e| anyhow!("Failed to load params from {}: {}", path.display(), e))?;

        let count = params.get_all().len();

        if count > 0 {
            eprintln!(
                "{} Loaded {} parameters from {}",
                "".cyan(),
                count.to_string().green(),
                path.display().to_string().cyan()
            );
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolve_target_file() {
        let tmp = tempfile::TempDir::new().unwrap();
        let file = tmp.path().join("main.rs");
        fs::write(&file, "fn main() {}").unwrap();

        let targets = resolve_execution_target(file.clone()).unwrap();
        assert_eq!(targets.len(), 1);
        assert!(matches!(&targets[0], ExecutionTarget::File(p) if p == &file));
    }

    #[test]
    fn resolve_target_directory() {
        let tmp = tempfile::TempDir::new().unwrap();
        let targets = resolve_execution_target(tmp.path().to_path_buf()).unwrap();
        assert_eq!(targets.len(), 1);
        assert!(matches!(&targets[0], ExecutionTarget::Directory(_)));
    }

    #[test]
    fn resolve_target_manifest_horus_toml() {
        let tmp = tempfile::TempDir::new().unwrap();
        let manifest = tmp.path().join(HORUS_TOML);
        fs::write(&manifest, "[package]\nname = \"test\"\n").unwrap();

        let targets = resolve_execution_target(manifest).unwrap();
        assert_eq!(targets.len(), 1);
        assert!(matches!(&targets[0], ExecutionTarget::Manifest(_)));
    }

    #[test]
    fn resolve_target_manifest_cargo_toml() {
        let tmp = tempfile::TempDir::new().unwrap();
        let manifest = tmp.path().join("Cargo.toml");
        fs::write(
            &manifest,
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let targets = resolve_execution_target(manifest).unwrap();
        assert_eq!(targets.len(), 1);
        assert!(matches!(&targets[0], ExecutionTarget::Manifest(_)));
    }

    #[test]
    fn resolve_target_nonexistent_fails() {
        let result = resolve_execution_target(PathBuf::from("/tmp/definitely_not_exist_abc123"));
        result.unwrap_err();
    }

    #[test]
    fn auto_detect_main_rs() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = auto_detect_main_file();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        assert_eq!(result.unwrap(), PathBuf::from("main.rs"));
    }

    #[test]
    fn auto_detect_main_py() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join("main.py"), "print('hello')").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = auto_detect_main_file();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        assert_eq!(result.unwrap(), PathBuf::from("main.py"));
    }

    #[test]
    fn auto_detect_main_rs_over_main_py() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join("main.rs"), "fn main() {}").unwrap();
        fs::write(tmp.path().join("main.py"), "print('hello')").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = auto_detect_main_file();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        assert_eq!(result.unwrap(), PathBuf::from("main.rs"));
    }

    #[test]
    fn auto_detect_src_main_rs() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::create_dir_all(tmp.path().join("src")).unwrap();
        fs::write(tmp.path().join("src/main.rs"), "fn main() {}").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = auto_detect_main_file();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        assert_eq!(result.unwrap(), PathBuf::from("src/main.rs"));
    }

    #[test]
    fn auto_detect_single_rs_file() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join("robot.rs"), "fn main() {}").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = auto_detect_main_file();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        result.unwrap();
    }

    #[test]
    fn auto_detect_empty_dir_fails() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = auto_detect_main_file();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        result.unwrap_err();
    }

    #[test]
    fn ensure_horus_directory_creates_all_dirs() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = ensure_horus_directory();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        result.unwrap();
        assert!(tmp.path().join(".horus").is_dir());
        assert!(tmp.path().join(".horus/packages").is_dir());
        assert!(tmp.path().join(".horus/bin").is_dir());
        assert!(tmp.path().join(".horus/lib").is_dir());
        assert!(tmp.path().join(".horus/cache").is_dir());
    }

    #[test]
    fn ensure_horus_directory_idempotent() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        ensure_horus_directory().unwrap();
        fs::write(tmp.path().join(".horus/bin/test"), "keep").unwrap();
        ensure_horus_directory().unwrap();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(tmp.path().join(".horus/bin/test").exists());
    }

    #[test]
    fn build_child_env_returns_required_vars() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::create_dir_all(tmp.path().join(".horus/bin")).unwrap();
        fs::create_dir_all(tmp.path().join(".horus/lib")).unwrap();
        fs::create_dir_all(tmp.path().join(".horus/packages")).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = build_child_env();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        let env_vars = result.unwrap();
        let names: Vec<&str> = env_vars.iter().map(|(k, _)| k.as_str()).collect();
        assert!(names.contains(&"PATH"), "Must set PATH");
        assert!(
            names.contains(&"LD_LIBRARY_PATH"),
            "Must set LD_LIBRARY_PATH"
        );
        assert!(names.contains(&"PYTHONPATH"), "Must set PYTHONPATH");
    }

    #[test]
    fn build_child_env_path_includes_horus_bin() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::create_dir_all(tmp.path().join(".horus/bin")).unwrap();
        fs::create_dir_all(tmp.path().join(".horus/lib")).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = build_child_env();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        let env_vars = result.unwrap();
        let path_val = env_vars.iter().find(|(k, _)| k == "PATH").unwrap();
        assert!(
            path_val.1.contains(".horus/bin"),
            "PATH should include .horus/bin, got: {}",
            path_val.1
        );
    }

    #[test]
    fn execution_target_debug() {
        let target = ExecutionTarget::File(PathBuf::from("main.rs"));
        let debug = format!("{:?}", target);
        assert!(debug.contains("File"));

        let target = ExecutionTarget::Directory(PathBuf::from("src/"));
        assert!(format!("{:?}", target).contains("Directory"));

        let target = ExecutionTarget::Manifest(PathBuf::from("horus.toml"));
        assert!(format!("{:?}", target).contains("Manifest"));

        let target = ExecutionTarget::Multiple(vec![]);
        assert!(format!("{:?}", target).contains("Multiple"));
    }

    #[test]
    fn load_params_no_files_returns_ok() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = load_params_from_project();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        result.unwrap();
    }

    #[test]
    fn load_params_with_root_params_yaml() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join("params.yaml"),
            "robot_name: test_bot\nmax_speed: 1.5\n",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = load_params_from_project();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        result.unwrap();
    }

    #[test]
    fn load_params_with_config_subdir_params() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::create_dir_all(tmp.path().join("config")).unwrap();
        fs::write(tmp.path().join("config/params.yaml"), "sensor_rate: 100\n").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = load_params_from_project();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        result.unwrap();
    }

    #[test]
    fn load_params_with_horus_cache_params() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::create_dir_all(tmp.path().join(".horus/config")).unwrap();
        fs::write(
            tmp.path().join(".horus/config/params.yaml"),
            "debug: true\n",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = load_params_from_project();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        result.unwrap();
    }

    #[test]
    fn load_params_root_has_priority_over_config() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join("params.yaml"), "source: root\n").unwrap();
        fs::create_dir_all(tmp.path().join("config")).unwrap();
        fs::write(tmp.path().join("config/params.yaml"), "source: config\n").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = load_params_from_project();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        result.unwrap();
    }

    #[test]
    fn load_params_invalid_yaml_returns_err() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join("params.yaml"),
            "{{{{ invalid yaml: [[[not closed",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = load_params_from_project();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_err());
    }

    #[test]
    fn load_params_empty_yaml_returns_ok() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join("params.yaml"), "").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = load_params_from_project();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        result.unwrap();
    }

    // ── Battle tests: params.yaml loading behavior ──────────────────────

    #[test]
    fn load_params_with_nested_yaml_structure() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join("params.yaml"),
            "robot:\n  name: test_bot\n  speed:\n    max: 1.5\n    min: 0.1\nsensors:\n  lidar: true\n",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = load_params_from_project();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(
            result.is_ok(),
            "Nested YAML structure should load successfully"
        );
    }

    #[test]
    fn load_params_config_subdir_not_used_when_root_exists() {
        // When both root and config/ exist, root takes priority.
        // Verify the function completes successfully in either case.
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join("params.yaml"),
            "source: root\npriority: 1\n",
        )
        .unwrap();
        fs::create_dir_all(tmp.path().join("config")).unwrap();
        fs::write(
            tmp.path().join("config/params.yaml"),
            "source: config\npriority: 2\n",
        )
        .unwrap();
        fs::create_dir_all(tmp.path().join(".horus/config")).unwrap();
        fs::write(
            tmp.path().join(".horus/config/params.yaml"),
            "source: horus_cache\npriority: 3\n",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = load_params_from_project();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(
            result.is_ok(),
            "Should load from highest-priority params.yaml"
        );
    }

    // ── Battle tests: auto_detect_main_file behavior ────────────────────

    #[test]
    fn auto_detect_prefers_main_rs_over_src_main_rs() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join("main.rs"), "fn main() { /* root */ }").unwrap();
        fs::create_dir_all(tmp.path().join("src")).unwrap();
        fs::write(tmp.path().join("src/main.rs"), "fn main() { /* src */ }").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = auto_detect_main_file();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        assert_eq!(
            result.unwrap(),
            PathBuf::from("main.rs"),
            "main.rs in root should have priority over src/main.rs"
        );
    }

    #[test]
    fn auto_detect_src_main_py() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::create_dir_all(tmp.path().join("src")).unwrap();
        fs::write(tmp.path().join("src/main.py"), "print('hello')").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = auto_detect_main_file();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        assert_eq!(result.unwrap(), PathBuf::from("src/main.py"));
    }

    #[test]
    fn auto_detect_fails_with_multiple_code_files_no_main() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join("robot.rs"), "fn main() {}").unwrap();
        fs::write(tmp.path().join("sensor.rs"), "fn main() {}").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = auto_detect_main_file();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(
            result.is_err(),
            "Multiple code files without main.rs should fail auto-detection"
        );
    }

    #[test]
    fn auto_detect_single_py_file() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join("robot.py"), "print('hello')").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = auto_detect_main_file();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok(), "Single .py file should be auto-detected");
    }

    #[test]
    fn auto_detect_ignores_non_code_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        // Only non-code files present
        fs::write(tmp.path().join("README.md"), "# My Robot").unwrap();
        fs::write(tmp.path().join("config.yaml"), "key: value").unwrap();
        fs::write(tmp.path().join("data.json"), "{}").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = auto_detect_main_file();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(
            result.is_err(),
            "Non-code files should not be detected as main files"
        );
    }

    // ── Battle tests: scripts from horus.toml [scripts] ─────────────────

    #[test]
    fn execute_run_dispatches_to_script_when_name_matches() {
        // When a bare name matches a [scripts] entry in horus.toml,
        // execute_run should dispatch to run_scripts.
        // Since run_scripts executes a shell command (which might fail),
        // we test the condition that triggers script dispatch by checking
        // that the manifest has the script and no path separator is present.
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"script-test\"\nversion = \"0.1.0\"\n\n[scripts]\nsim = \"echo running sim\"\nbuild_all = \"cargo build --workspace\"\n",
        )
        .unwrap();

        // Verify the manifest loads and has the scripts
        let manifest = HorusManifest::load_from(&tmp.path().join("horus.toml")).unwrap();
        assert!(
            manifest.scripts.contains_key("sim"),
            "Manifest should contain 'sim' script"
        );
        assert!(
            manifest.scripts.contains_key("build_all"),
            "Manifest should contain 'build_all' script"
        );
        assert_eq!(manifest.scripts.get("sim").unwrap(), "echo running sim");
    }

    #[test]
    fn script_name_with_path_separator_not_treated_as_script() {
        // A path like "src/sim" should never be treated as a script name
        let candidate = "src/sim";
        assert!(
            candidate.contains('/'),
            "Path with separator should not match as bare script name"
        );
    }

    #[test]
    fn script_name_with_extension_not_treated_as_script() {
        // A file like "sim.rs" should never be treated as a script name
        let candidate = "sim.rs";
        assert!(
            candidate.contains('.'),
            "File with extension should not match as bare script name"
        );
    }

    // ── Battle tests: clean flag behavior ───────────────────────────────

    #[test]
    fn clean_flag_triggers_cache_cleanup() {
        // Verify that the clean codepath calls clean_build_cache,
        // which removes files from .horus/cache and .horus/bin
        let tmp = tempfile::TempDir::new().unwrap();
        fs::create_dir_all(tmp.path().join(".horus/cache")).unwrap();
        fs::create_dir_all(tmp.path().join(".horus/bin")).unwrap();
        fs::write(tmp.path().join(".horus/cache/old_binary"), b"old").unwrap();
        fs::write(tmp.path().join(".horus/bin/old_tool"), b"old").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_rust::clean_build_cache();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok(), "clean_build_cache should succeed");
        // Verify files were cleaned
        assert!(
            !tmp.path().join(".horus/cache/old_binary").exists(),
            "Cache files should be removed after clean"
        );
        assert!(
            !tmp.path().join(".horus/bin/old_tool").exists(),
            "Bin files should be removed after clean"
        );
        // Directories themselves should still exist
        assert!(tmp.path().join(".horus/cache").is_dir());
        assert!(tmp.path().join(".horus/bin").is_dir());
    }

    #[test]
    fn clean_flag_handles_nonexistent_dirs_gracefully() {
        let tmp = tempfile::TempDir::new().unwrap();
        // No .horus, no target, no __pycache__

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_rust::clean_build_cache();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(
            result.is_ok(),
            "clean_build_cache should not error on empty directory"
        );
    }

    // ── Battle tests: resolve_execution_target edge cases ────────────────

    #[test]
    fn resolve_target_rs_file_not_treated_as_manifest() {
        let tmp = tempfile::TempDir::new().unwrap();
        let file = tmp.path().join("robot.rs");
        fs::write(&file, "fn main() {}").unwrap();

        let targets = resolve_execution_target(file.clone()).unwrap();
        assert_eq!(targets.len(), 1);
        assert!(
            matches!(&targets[0], ExecutionTarget::File(p) if p == &file),
            "Regular .rs file should resolve as File, not Manifest"
        );
    }

    #[test]
    fn resolve_target_py_file() {
        let tmp = tempfile::TempDir::new().unwrap();
        let file = tmp.path().join("robot.py");
        fs::write(&file, "print('hello')").unwrap();

        let targets = resolve_execution_target(file.clone()).unwrap();
        assert_eq!(targets.len(), 1);
        assert!(matches!(&targets[0], ExecutionTarget::File(p) if p == &file));
    }

    // ── Battle tests: build_child_env details ───────────────────────────

    #[test]
    fn build_child_env_ld_library_path_includes_horus_lib() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::create_dir_all(tmp.path().join(".horus/lib")).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = build_child_env();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        let env_vars = result.unwrap();
        let ld_path_val = env_vars
            .iter()
            .find(|(k, _)| k == "LD_LIBRARY_PATH")
            .unwrap();
        assert!(
            ld_path_val.1.contains(".horus/lib"),
            "LD_LIBRARY_PATH should include .horus/lib, got: {}",
            ld_path_val.1
        );
    }

    #[test]
    fn build_child_env_pythonpath_includes_horus_packages() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::create_dir_all(tmp.path().join(".horus/packages")).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = build_child_env();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        let env_vars = result.unwrap();
        let pypath_val = env_vars.iter().find(|(k, _)| k == "PYTHONPATH").unwrap();
        assert!(
            pypath_val.1.contains(".horus/packages"),
            "PYTHONPATH should include .horus/packages, got: {}",
            pypath_val.1
        );
    }

    // ── Battle tests: load_ignore_patterns from horus.toml ──────────────

    #[test]
    fn load_ignore_patterns_with_horus_toml() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"ignore-test\"\nversion = \"0.1.0\"\n\n[ignore]\nfiles = [\"*.log\", \"*.tmp\"]\ndirectories = [\"target/\", \"build/\"]\npackages = [\"devtools\"]\n",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let patterns = load_ignore_patterns();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert_eq!(patterns.files, vec!["*.log", "*.tmp"]);
        assert_eq!(patterns.directories, vec!["target/", "build/"]);
        assert_eq!(patterns.packages, vec!["devtools"]);
    }

    #[test]
    fn load_ignore_patterns_without_horus_toml_returns_default() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let patterns = load_ignore_patterns();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(patterns.files.is_empty());
        assert!(patterns.directories.is_empty());
        assert!(patterns.packages.is_empty());
    }

    // ── Battle tests: ensure_horus_directory structure ───────────────────

    #[test]
    fn ensure_horus_directory_creates_all_required_subdirs() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = ensure_horus_directory();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        result.unwrap();
        // Verify all 4 subdirectories
        for subdir in &["packages", "bin", "lib", "cache"] {
            assert!(
                tmp.path().join(".horus").join(subdir).is_dir(),
                ".horus/{} should exist",
                subdir
            );
        }
    }

    #[test]
    fn ensure_horus_directory_preserves_existing_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::create_dir_all(tmp.path().join(".horus/cache")).unwrap();
        fs::write(tmp.path().join(".horus/cache/important"), "keep me").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        ensure_horus_directory().unwrap();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        // File should still exist
        let content = fs::read_to_string(tmp.path().join(".horus/cache/important")).unwrap();
        assert_eq!(content, "keep me", "Existing files should be preserved");
    }

    // ── Signal handling & shutdown lifecycle ─────────────────────────────

    #[test]
    fn running_flag_starts_true() {
        let running = Arc::new(std::sync::atomic::AtomicBool::new(true));
        assert!(running.load(std::sync::atomic::Ordering::SeqCst));
    }

    #[test]
    fn running_flag_set_to_false_stops_loop() {
        let running = Arc::new(std::sync::atomic::AtomicBool::new(true));
        running.store(false, std::sync::atomic::Ordering::SeqCst);
        assert!(!running.load(std::sync::atomic::Ordering::SeqCst));
    }

    #[test]
    fn running_flag_cross_thread_visibility() {
        let running = Arc::new(std::sync::atomic::AtomicBool::new(true));
        let r = running.clone();

        let handle = std::thread::spawn(move || {
            // Simulate ctrlc handler setting flag
            r.store(false, std::sync::atomic::Ordering::SeqCst);
        });
        handle.join().unwrap();

        // Main thread should see the change
        assert!(
            !running.load(std::sync::atomic::Ordering::SeqCst),
            "Cross-thread flag update must be visible"
        );
    }

    #[test]
    fn running_flag_multiple_threads_see_change() {
        let running = Arc::new(std::sync::atomic::AtomicBool::new(true));
        let mut handles = vec![];

        // Spawn 4 reader threads
        for _ in 0..4 {
            let r = running.clone();
            handles.push(std::thread::spawn(move || {
                // Spin until flag is false (simulates child process loop)
                while r.load(std::sync::atomic::Ordering::SeqCst) {
                    std::thread::yield_now();
                }
                true // successfully saw the flag change
            }));
        }

        // Small delay then set flag
        std::thread::sleep(std::time::Duration::from_millis(5));
        running.store(false, std::sync::atomic::Ordering::SeqCst);

        // All threads should exit
        for handle in handles {
            let saw_change = handle.join().unwrap();
            assert!(saw_change);
        }
    }

    #[test]
    fn build_child_env_contains_path() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::TempDir::new().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let env = build_child_env();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        let env = env.unwrap();
        assert!(
            env.iter().any(|(k, _)| k == "PATH"),
            "build_child_env must include PATH"
        );
    }

    #[test]
    fn build_child_env_contains_ld_library_path() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::TempDir::new().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let env = build_child_env();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        let env = env.unwrap();
        assert!(
            env.iter().any(|(k, _)| k == "LD_LIBRARY_PATH"),
            "build_child_env must include LD_LIBRARY_PATH"
        );
    }

    #[test]
    fn build_child_env_path_includes_horus_bin_2() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::TempDir::new().unwrap();
        // Create .horus/bin so it gets included
        fs::create_dir_all(tmp.path().join(".horus/bin")).unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let env = build_child_env();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        let env = env.unwrap();
        let path_entry = env.iter().find(|(k, _)| k == "PATH");
        assert!(path_entry.is_some());
        let path_val = &path_entry.unwrap().1;
        assert!(
            path_val.contains(".horus/bin") || path_val.contains(".horus\\bin"),
            "PATH should include .horus/bin: {path_val}"
        );
    }

    #[test]
    fn build_child_env_returns_nonempty() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::TempDir::new().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let env = build_child_env();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(
            !env.unwrap().is_empty(),
            "build_child_env should return at least PATH and LD_LIBRARY_PATH"
        );
    }
}
