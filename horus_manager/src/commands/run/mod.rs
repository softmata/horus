pub(crate) mod deps;
pub(crate) mod features;
pub(crate) mod hardware;
pub(crate) mod install;
mod run_python;
mod run_rust;

// Re-export public API
pub use deps::parse_horus_yaml_dependencies_v2;
pub use hardware::check_hardware_requirements;
pub use run_rust::execute_build_only;

// Re-export for sibling modules (install.rs uses detect_python_interpreter via super::)
pub(crate) use run_python::detect_python_interpreter;

use crate::config::{CARGO_TOML, HORUS_YAML};
use crate::progress;
use anyhow::{anyhow, bail, Context, Result};
use colored::*;
use glob::glob;
use std::collections::HashSet;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

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
) -> Result<()> {
    log::debug!("executing run with files: {:?}", files);

    // Handle clean build
    if clean {
        eprintln!("{} Cleaning build cache...", "[CLEAN]".cyan());
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
    hardware::load_params_from_project()?;

    let mode = if release { "release" } else { "debug" };
    eprintln!(
        "{} Starting HORUS runtime in {} mode...",
        "".cyan(),
        mode.yellow()
    );

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

fn execute_single_file(
    file_path: PathBuf,
    args: Vec<String>,
    release: bool,
    clean: bool,
) -> Result<()> {
    let language = deps::detect_language(&file_path)?;

    eprintln!(
        "{} Detected: {} ({})",
        "".cyan(),
        file_path.display().to_string().green(),
        language.yellow()
    );

    // Load ignore patterns from horus.yaml if it exists
    let ignore = if Path::new(HORUS_YAML).exists() {
        features::parse_horus_yaml_ignore(HORUS_YAML).unwrap_or_default()
    } else {
        features::IgnorePatterns::default()
    };

    // Ensure .horus directory exists
    ensure_horus_directory()?;

    // Scan imports and resolve dependencies
    log::info!("Scanning imports for {}", file_path.display());
    eprintln!("{} Scanning imports...", "".cyan());
    let dependencies = deps::scan_imports(&file_path, &language, &ignore)?;

    // Check hardware requirements
    if let Err(e) = hardware::check_hardware_requirements(&file_path, &language) {
        log::warn!("Hardware check error: {}", e);
        eprintln!("[WARNING] Hardware check error: {}", e);
    }

    if !dependencies.is_empty() {
        log::debug!("Found {} dependencies", dependencies.len());
        eprintln!("{} Found {} dependencies", "".cyan(), dependencies.len());

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
        // - For Rust: cargo deps go to Cargo.toml, not cargo install
        // - For Python: cargo deps are skipped entirely (they're library crates)
        if !deps_to_resolve.is_empty() {
            install::resolve_dependencies_with_context(deps_to_resolve, Some(&language))?;
        }
    }

    // Setup environment
    setup_environment()?;

    // Execute
    eprintln!("{} Executing...\n", "".cyan());
    run_rust::execute_with_scheduler(file_path, language, args, release, clean)?;

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
        "".cyan(),
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
        "".cyan(),
        manifest_path.display().to_string().green()
    );

    match manifest_path.file_name().and_then(|s| s.to_str()) {
        Some(HORUS_YAML) => execute_from_horus_yaml(manifest_path, args, release, clean),
        Some(CARGO_TOML) => run_rust::execute_from_cargo_toml(manifest_path, args, release, clean),
        _ => bail!("Unsupported manifest type: {}", manifest_path.display()),
    }
}

fn execute_from_horus_yaml(
    manifest_path: PathBuf,
    args: Vec<String>,
    release: bool,
    clean: bool,
) -> Result<()> {
    // For now, find the main file in the same directory as horus.yaml
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
        "".cyan(),
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
    println!("\n{} Phase 1: Building all files...", "".cyan());
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
        let spinner = progress::robot_build_spinner(&build_msg);

        let rust_executables = run_rust::build_rust_files_batch(rust_files, release, clean)?;
        executables.extend(rust_executables);
        progress::finish_success(&spinner, "Rust files built");
    }

    // Build other languages individually
    for (file_path, language) in other_files {
        let spinner =
            progress::robot_build_spinner(&format!("Building {}...", file_path.display()));

        let exec_info = run_rust::build_file_for_concurrent_execution(
            file_path, language, release, false, // Don't clean - already done if needed
        )?;

        executables.push(exec_info);
        progress::finish_success(&spinner, "Built");
    }

    println!(
        "{} All files built successfully!\n",
        progress::STATUS_SUCCESS
    );

    // Phase 2: Execute all binaries concurrently
    println!("{} Phase 2: Starting all processes...", "".cyan());

    let running = Arc::new(AtomicBool::new(true));
    let children: Arc<Mutex<Vec<(String, std::process::Child)>>> = Arc::new(Mutex::new(Vec::new()));

    // Setup Ctrl+C handler with access to children
    let r = running.clone();
    let c = children.clone();
    ctrlc::set_handler(move || {
        println!("\n{} Shutting down all processes...", "".yellow());
        r.store(false, Ordering::SeqCst);

        // Kill all child processes
        if let Ok(mut children_lock) = c.lock() {
            for (name, child) in children_lock.iter_mut() {
                println!("  {} Terminating [{}]...", "".yellow(), name);
                let _ = child.kill();
            }
        }
    })
    .expect("Error setting Ctrl-C handler");

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

                println!("  {} Started [{}]", "".green(), node_name.color(color));
                children
                    .lock()
                    .expect("children lock poisoned")
                    .push((node_name, child));
            }
            Err(e) => {
                eprintln!("  {} Failed to start [{}]: {}", "".red(), node_name, e);
            }
        }
    }

    println!(
        "\n{} All processes running. Press Ctrl+C to stop.\n",
        "".green()
    );

    // Wait for all processes to complete (concurrent, checks running flag)
    loop {
        let mut all_done = true;
        let mut children_lock = children.lock().expect("children lock poisoned");

        // Check each child with try_wait (non-blocking)
        children_lock.retain_mut(|(name, child)| {
            match child.try_wait() {
                Ok(Some(status)) => {
                    // Process exited
                    if !status.success() {
                        eprintln!(
                            "\n{} Process [{}] exited with code: {}",
                            "".yellow(),
                            name,
                            status.code().unwrap_or(-1)
                        );
                    }
                    false // Remove from list
                }
                Ok(None) => {
                    // Still running
                    all_done = false;
                    true // Keep in list
                }
                Err(e) => {
                    eprintln!("\n{} Error checking [{}]: {}", "".red(), name, e);
                    false // Remove from list
                }
            }
        });

        let still_running = !children_lock.is_empty();
        drop(children_lock);

        // Exit if all processes done or Ctrl+C was pressed and we killed them
        if all_done || (!running.load(Ordering::SeqCst) && !still_running) {
            break;
        }

        // Small sleep to avoid busy-waiting
        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    // Wait for output threads to finish
    for handle in handles {
        handle.join().ok();
    }

    if !running.load(Ordering::SeqCst) {
        println!("\n{} All processes stopped.", "".green());
    } else {
        println!("\n{} All processes completed.", "".green());
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
        match input.extension().and_then(|s| s.to_str()) {
            Some("yaml") | Some("yml") => {
                if input.file_name().and_then(|s| s.to_str()) == Some(HORUS_YAML) {
                    return Ok(vec![ExecutionTarget::Manifest(input)]);
                }
            }
            Some("toml") => {
                if input.file_name().and_then(|s| s.to_str()) == Some(CARGO_TOML) {
                    return Ok(vec![ExecutionTarget::Manifest(input)]);
                }
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
    // Load ignore patterns from horus.yaml if it exists
    let ignore = if Path::new(HORUS_YAML).exists() {
        features::parse_horus_yaml_ignore(HORUS_YAML).unwrap_or_default()
    } else {
        features::IgnorePatterns::default()
    };

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

fn auto_detect_main_file() -> Result<PathBuf> {
    log::debug!("auto-detecting main file in current directory");

    // Load ignore patterns from horus.yaml if it exists
    let ignore = if Path::new(HORUS_YAML).exists() {
        features::parse_horus_yaml_ignore(HORUS_YAML).unwrap_or_default()
    } else {
        features::IgnorePatterns::default()
    };

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

fn ensure_horus_directory() -> Result<()> {
    let horus_dir = PathBuf::from(".horus");

    // Create .horus/ if it doesn't exist
    if !horus_dir.exists() {
        println!("{} Creating .horus/ environment...", "".cyan());
        fs::create_dir_all(&horus_dir)?;
    }

    // Always ensure subdirectories exist (they might not if created by `horus new`)
    fs::create_dir_all(horus_dir.join("packages"))?;
    fs::create_dir_all(horus_dir.join("bin"))?;
    fs::create_dir_all(horus_dir.join("lib"))?;
    fs::create_dir_all(horus_dir.join("cache"))?;

    Ok(())
}

fn setup_environment() -> Result<()> {
    let current_dir = env::current_dir()?;
    let horus_bin = current_dir.join(".horus/bin");
    let horus_lib = current_dir.join(".horus/lib");
    let horus_packages = current_dir.join(".horus/packages");

    // Update PATH
    if let Ok(path) = env::var("PATH") {
        let new_path = format!("{}:{}", horus_bin.display(), path);
        env::set_var("PATH", new_path);
    }

    // Build LD_LIBRARY_PATH: local + global cache libs
    let mut lib_paths = vec![horus_lib.display().to_string()];

    // Add global cache library paths if they exist
    let global_cache =
        crate::paths::cache_dir().unwrap_or_else(|_| install::home_dir().join(".horus/cache"));
    {
        if global_cache.exists() {
            // Scan for packages with lib/ directories
            if let Ok(entries) = fs::read_dir(&global_cache) {
                for entry in entries.flatten() {
                    let lib_dir = entry.path().join("lib");
                    if lib_dir.exists() {
                        lib_paths.push(lib_dir.display().to_string());
                    }
                    // Also check target/release for Rust packages
                    let target_lib = entry.path().join("target/release");
                    if target_lib.exists() {
                        lib_paths.push(target_lib.display().to_string());
                    }
                }
            }
        }
    }

    // Set LD_LIBRARY_PATH with all paths
    let lib_path_str = lib_paths.join(":");
    if let Ok(ld_path) = env::var("LD_LIBRARY_PATH") {
        let new_path = format!("{}:{}", lib_path_str, ld_path);
        env::set_var("LD_LIBRARY_PATH", new_path);
    } else {
        env::set_var("LD_LIBRARY_PATH", lib_path_str);
    }

    // Update PYTHONPATH for Python imports
    if let Ok(py_path) = env::var("PYTHONPATH") {
        let new_path = format!("{}:{}", horus_packages.display(), py_path);
        env::set_var("PYTHONPATH", new_path);
    } else {
        env::set_var("PYTHONPATH", horus_packages.display().to_string());
    }

    Ok(())
}
