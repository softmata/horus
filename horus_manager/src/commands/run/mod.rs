pub(crate) mod deps;
pub(crate) mod features;
pub(crate) mod hardware;
pub(crate) mod install;

// Re-export public API
pub use deps::parse_horus_yaml_dependencies_v2;
pub use features::{
    parse_horus_yaml_ignore, parse_horus_yaml_drivers, parse_horus_yaml_enable,
    get_active_drivers, get_active_enable, get_all_cargo_features,
    get_cargo_features_from_drivers, get_cargo_features_arg,
    get_cargo_features_from_enable, enable_to_features,
    IgnorePatterns, DriverConfig, EnableConfig,
};
pub use hardware::check_hardware_requirements;

use crate::config::{CARGO_TOML, HORUS_YAML};
use crate::progress::{self, finish_error, finish_success};
use anyhow::{anyhow, bail, Context, Result};
use colored::*;
use glob::glob;
use std::collections::HashSet;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

#[derive(Debug, Clone)]
enum ExecutionTarget {
    File(PathBuf),
    Directory(PathBuf),
    Manifest(PathBuf),
    Multiple(Vec<PathBuf>),
}

pub fn execute_build_only(files: Vec<PathBuf>, release: bool, clean: bool) -> Result<()> {
    // Handle clean build
    if clean {
        println!("{} Cleaning build cache...", "[CLEAN]".cyan());
        clean_build_cache()?;
    }

    let mode = if release { "release" } else { "debug" };
    println!(
        "{} Building project in {} mode (no execution)...",
        "".cyan(),
        mode.yellow()
    );

    // Resolve target file(s)
    let target_files: Vec<PathBuf> = if files.is_empty() {
        vec![auto_detect_main_file()?]
    } else {
        files
    };

    // Bail out - execute_build_only doesn't support multiple files yet
    // For multi-file execution, use execute_run which calls execute_multiple_files
    if target_files.len() > 1 {
        bail!("Build-only mode doesn't support multiple files. Use 'horus run' to execute multiple files concurrently.");
    }

    let target_file = &target_files[0];
    let language = deps::detect_language(target_file)?;
    println!(
        "{} Detected: {} ({})",
        "".cyan(),
        target_file.display().to_string().green(),
        language.yellow()
    );

    // Ensure .horus directory exists
    ensure_horus_directory()?;

    // Build based on language
    match language.as_str() {
        "python" => {
            println!("{} Python is interpreted, no build needed", "[i]".blue());
            println!(
                "  {} File is ready to run: {}",
                "".cyan(),
                target_file.display()
            );
        }
        "rust" => {
            // Setup Rust build using Cargo in .horus workspace
            println!("{} Setting up Cargo workspace...", "".cyan());

            // Parse horus.yaml to get dependencies
            let dependencies = if Path::new(HORUS_YAML).exists() {
                deps::parse_horus_yaml_dependencies(HORUS_YAML)?
            } else {
                HashSet::new()
            };

            // Split dependencies into HORUS packages, pip packages, cargo packages, path and git dependencies
            let (horus_deps, _pip_packages, cargo_packages, path_deps, git_deps) =
                deps::split_dependencies_with_path_context(dependencies.clone(), Some("rust"));

            // Generate Cargo.toml in .horus/ that references source files in parent directory
            let cargo_toml_path = PathBuf::from(".horus/Cargo.toml");

            // Get relative path from .horus/ to the source file
            let source_relative_path = format!("../{}", target_file.display());

            let mut cargo_toml = format!(
                r#"[package]
name = "horus-project"
version = "0.1.9"
edition = "2021"

# Empty workspace to prevent inheriting parent workspace
[workspace]

[[bin]]
name = "horus-project"
path = "{}"

[dependencies]
"#,
                source_relative_path
            );

            // Auto-detect nodes and required features
            use crate::node_detector;
            let auto_features =
                node_detector::detect_features_from_file(target_file).unwrap_or_default();
            if !auto_features.is_empty() {
                eprintln!(
                    "  {} Auto-detected hardware nodes (features: {})",
                    "".cyan(),
                    auto_features.join(", ").yellow()
                );

                // Check system dependencies for detected features
                use crate::system_deps;
                let dep_result = system_deps::check_dependencies(&auto_features);
                let report = system_deps::format_dependency_report(&dep_result, &auto_features);
                if !report.is_empty() {
                    eprintln!("{}", report);
                }
            }

            // Find HORUS source directory
            let horus_source = find_horus_source_dir()?;
            println!(
                "  {} Using HORUS source: {}",
                "".cyan(),
                horus_source.display()
            );

            // Add HORUS dependencies from source
            for dep in &horus_deps {
                // Strip version from dependency name for path lookup
                let dep_name = if let Some(at_pos) = dep.find('@') {
                    &dep[..at_pos]
                } else {
                    dep.as_str()
                };

                let dep_path = horus_source.join(dep_name);

                if dep_path.exists() && dep_path.join(CARGO_TOML).exists() {
                    // Auto-inject features for horus or horus_library
                    if (dep_name == "horus" || dep_name == "horus_library")
                        && !auto_features.is_empty()
                    {
                        cargo_toml.push_str(&format!(
                            "{} = {{ path = \"{}\", features = [{}] }}\n",
                            dep_name,
                            dep_path.display(),
                            auto_features
                                .iter()
                                .map(|f| format!("\"{}\"", f))
                                .collect::<Vec<_>>()
                                .join(", ")
                        ));
                        println!(
                            "  {} Added dependency: {} -> {} (auto-features: {})",
                            "".cyan(),
                            dep,
                            dep_path.display(),
                            auto_features.join(", ").yellow()
                        );
                    } else {
                        cargo_toml.push_str(&format!(
                            "{} = {{ path = \"{}\" }}\n",
                            dep_name,
                            dep_path.display()
                        ));
                        println!(
                            "  {} Added dependency: {} -> {}",
                            "".cyan(),
                            dep,
                            dep_path.display()
                        );
                    }
                } else {
                    eprintln!(
                        "  {} Warning: dependency {} not found at {}",
                        "".yellow(),
                        dep,
                        dep_path.display()
                    );
                }
            }

            // Add cargo dependencies from crates.io
            for pkg in &cargo_packages {
                if !pkg.features.is_empty() {
                    // With features
                    if let Some(ref version) = pkg.version {
                        cargo_toml.push_str(&format!(
                            "{} = {{ version = \"{}\", features = [{}] }}\n",
                            pkg.name,
                            version,
                            pkg.features
                                .iter()
                                .map(|f| format!("\"{}\"", f))
                                .collect::<Vec<_>>()
                                .join(", ")
                        ));
                    } else {
                        cargo_toml.push_str(&format!(
                            "{} = {{ version = \"*\", features = [{}] }}\n",
                            pkg.name,
                            pkg.features
                                .iter()
                                .map(|f| format!("\"{}\"", f))
                                .collect::<Vec<_>>()
                                .join(", ")
                        ));
                        eprintln!(
                            "  {} Warning: Using wildcard version for '{}' - specify a version for reproducibility",
                            "".yellow(),
                            pkg.name
                        );
                    }
                    println!(
                        "  {} Added crates.io dependency: {} (features: {})",
                        "".cyan(),
                        pkg.name,
                        pkg.features.join(", ")
                    );
                } else if let Some(ref version) = pkg.version {
                    cargo_toml.push_str(&format!("{} = \"{}\"\n", pkg.name, version));
                    println!(
                        "  {} Added crates.io dependency: {}@{}",
                        "".cyan(),
                        pkg.name,
                        version
                    );
                } else {
                    cargo_toml.push_str(&format!("{} = \"*\"\n", pkg.name));
                    eprintln!(
                        "  {} Warning: Using wildcard version for '{}' - specify a version for reproducibility",
                        "".yellow(),
                        pkg.name
                    );
                    eprintln!("     Example: 'cargo:{}@1.0' in horus.yaml", pkg.name);
                    println!("  {} Added crates.io dependency: {}", "".cyan(), pkg.name);
                }
            }

            // Add path dependencies
            for (pkg_name, pkg_path) in &path_deps {
                // Convert relative path from current directory to relative from .horus/
                let full_path = PathBuf::from("..").join(pkg_path);
                cargo_toml.push_str(&format!(
                    "{} = {{ path = \"{}\" }}\n",
                    pkg_name,
                    full_path.display()
                ));
                println!(
                    "  {} Added path dependency: {} -> {}",
                    "✓".cyan(),
                    pkg_name,
                    pkg_path
                );
            }

            // Add git dependencies (clone to cache, then use as path dependencies)
            if !git_deps.is_empty() {
                println!("{} Resolving git dependencies...", "📦".cyan());
                let resolved_git_deps = install::resolve_git_dependencies(&git_deps)?;
                for (pkg_name, pkg_path) in &resolved_git_deps {
                    cargo_toml.push_str(&format!(
                        "{} = {{ path = \"{}\" }}\n",
                        pkg_name,
                        pkg_path.display()
                    ));
                    println!(
                        "  {} Added git dependency: {} -> {}",
                        "✓".green(),
                        pkg_name,
                        pkg_path.display()
                    );
                }
            }

            // Also add dependencies directly from horus.yaml (in case some weren't parsed by resolve_dependencies)
            // Track already-added cargo packages to avoid duplicates
            let added_cargo_deps: HashSet<String> =
                cargo_packages.iter().map(|pkg| pkg.name.clone()).collect();

            if Path::new(HORUS_YAML).exists() {
                if let Ok(yaml_content) = fs::read_to_string(HORUS_YAML) {
                    if let Ok(yaml) = serde_yaml::from_str::<serde_yaml::Value>(&yaml_content) {
                        if let Some(serde_yaml::Value::Sequence(list)) = yaml.get("dependencies") {
                            for item in list {
                                if let Some(dep_str) = deps::parse_yaml_cargo_dependency(item) {
                                    // Extract dependency name from the generated string (e.g., "serde = ..." -> "serde")
                                    let dep_name = dep_str.split('=').next().unwrap_or(&dep_str).trim();

                                    // Skip if already added from cargo_packages
                                    if !added_cargo_deps.contains(dep_name) {
                                        cargo_toml.push_str(&format!("{}\n", dep_str));
                                    }
                                }
                            }
                        }
                    }
                }
            }

            fs::write(&cargo_toml_path, cargo_toml)?;
            println!(
                "  {} Generated Cargo.toml (no source copying needed)",
                "".green()
            );

            // Run cargo build in .horus directory
            let spinner = progress::robot_build_spinner("Building with cargo...");
            let mut cmd = Command::new("cargo");
            cmd.arg("build");
            cmd.current_dir(".horus");
            // Capture output to avoid mixing with spinner
            cmd.stdout(std::process::Stdio::piped());
            cmd.stderr(std::process::Stdio::piped());

            if release {
                cmd.arg("--release");
            }

            // Add features from drivers and enable configuration
            if let Some(features) = features::get_all_cargo_features() {
                cmd.arg("--features").arg(&features);
                eprintln!(
                    "  {} Auto-enabling features: {}",
                    "󰢱".cyan(),
                    features.green()
                );
            }

            let output = cmd.output()?;
            if !output.status.success() {
                finish_error(&spinner, "Cargo build failed");
                // Print captured error output
                let stderr = String::from_utf8_lossy(&output.stderr);
                if !stderr.is_empty() {
                    eprintln!("{}", stderr);
                }
                bail!("Cargo build failed");
            }

            let profile = if release { "release" } else { "debug" };
            let binary_path = format!(".horus/target/{}/horus-project", profile);

            finish_success(&spinner, &format!("Built: {}", binary_path));
        }
        _ => bail!("Unsupported language: {}", language),
    }

    Ok(())
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
        clean_build_cache()?;
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
    execute_with_scheduler(file_path, language, args, release, clean)?;

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
        Some(CARGO_TOML) => execute_from_cargo_toml(manifest_path, args, release, clean),
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

fn execute_from_cargo_toml(
    manifest_path: PathBuf,
    args: Vec<String>,
    release: bool,
    clean: bool,
) -> Result<()> {
    // Change to the directory containing Cargo.toml
    let project_dir = manifest_path
        .parent()
        .ok_or_else(|| anyhow!("Cannot determine project directory"))?;

    let original_dir = env::current_dir()?;
    env::set_current_dir(project_dir)?;

    let result = (|| -> Result<()> {
        // Ensure .horus directory exists
        ensure_horus_directory()?;

        // Parse Cargo.toml for HORUS dependencies
        println!("{} Scanning Cargo.toml dependencies...", "".cyan());
        let horus_deps = deps::parse_cargo_dependencies(CARGO_TOML)?;

        if !horus_deps.is_empty() {
            println!(
                "{} Found {} HORUS dependencies",
                "".cyan(),
                horus_deps.len()
            );
            install::resolve_dependencies(horus_deps)?;
        }

        // Setup environment with .horus libraries
        setup_environment()?;

        // For Rust projects, run cargo directly
        let project_name = get_project_name()?;
        let build_dir = if release { "release" } else { "debug" };
        let binary = format!("target/{}/{}", build_dir, project_name);

        if !Path::new(&binary).exists() || clean {
            let spinner = progress::robot_build_spinner(&format!(
                "Building Cargo project ({} mode)...",
                build_dir
            ));
            let mut cmd = Command::new("cargo");
            cmd.arg("build");
            cmd.stdout(std::process::Stdio::piped());
            cmd.stderr(std::process::Stdio::piped());
            if release {
                cmd.arg("--release");
            }

            let output = cmd.output()?;
            if !output.status.success() {
                finish_error(&spinner, "Build failed");
                let stderr = String::from_utf8_lossy(&output.stderr);
                if !stderr.is_empty() {
                    eprintln!("{}", stderr);
                }
                bail!("Build failed");
            }
            finish_success(&spinner, "Build complete");
        }

        // Run the binary with environment
        println!("{} Executing Cargo project...\n", "".cyan());
        let mut cmd = Command::new(binary);
        cmd.args(args);
        let status = cmd.status()?;
        if !status.success() {
            bail!("Execution failed");
        }

        Ok(())
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
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Arc, Mutex};

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

        let rust_executables = build_rust_files_batch(rust_files, release, clean)?;
        executables.extend(rust_executables);
        finish_success(&spinner, "Rust files built");
    }

    // Build other languages individually
    for (file_path, language) in other_files {
        let spinner =
            progress::robot_build_spinner(&format!("Building {}...", file_path.display()));

        let exec_info = build_file_for_concurrent_execution(
            file_path, language, release, false, // Don't clean - already done if needed
        )?;

        executables.push(exec_info);
        finish_success(&spinner, "Built");
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
        let color = get_color_for_index(i);

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
                children.lock().expect("children lock poisoned").push((node_name, child));
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

struct ExecutableInfo {
    name: String,
    command: String,
    args_override: Vec<String>,
}

impl ExecutableInfo {
    fn create_command(&self, user_args: &[String]) -> Command {
        let mut cmd = Command::new(&self.command);

        // Use override args if provided, otherwise use user args
        if !self.args_override.is_empty() {
            cmd.args(&self.args_override);
        } else {
            cmd.args(user_args);
        }

        cmd
    }
}

fn get_color_for_index(index: usize) -> &'static str {
    let colors = ["cyan", "green", "yellow", "magenta", "blue", "red"];
    colors[index % colors.len()]
}

/// Build multiple Rust files in a single Cargo workspace for optimal performance
fn build_rust_files_batch(
    file_paths: Vec<PathBuf>,
    release: bool,
    clean: bool,
) -> Result<Vec<ExecutableInfo>> {
    if file_paths.is_empty() {
        return Ok(Vec::new());
    }

    // Ensure .horus directory exists
    ensure_horus_directory()?;

    // Setup environment
    setup_environment()?;

    // Load ignore patterns from horus.yaml if it exists
    let ignore = if Path::new(HORUS_YAML).exists() {
        features::parse_horus_yaml_ignore(HORUS_YAML).unwrap_or_default()
    } else {
        features::IgnorePatterns::default()
    };

    // Find HORUS source directory
    let horus_source = find_horus_source_dir()?;

    // Collect all dependencies from all Rust files
    let mut all_dependencies = HashSet::new();
    for file_path in &file_paths {
        let dependencies = deps::scan_imports(file_path, "rust", &ignore)?;
        all_dependencies.extend(dependencies);
    }

    // For Rust files, cargo dependencies are handled in Cargo.toml generation
    // So filter them out here to avoid trying to `cargo install` library crates
    let dependencies_to_resolve: HashSet<String> = {
        let (horus_pkgs, pip_pkgs, _cargo_pkgs) =
            deps::split_dependencies_with_context(all_dependencies.clone(), Some("rust"));

        // Filter out core HORUS crates - these are handled as path dependencies in Cargo.toml
        // and are NOT in the package registry. Including them causes install loops.
        // See: Issue with "horus clean -a" followed by "horus run nodes/*"
        let core_crates = ["horus", "horus_core", "horus_library", "horus_macros"];
        let registry_horus_pkgs: Vec<String> = horus_pkgs
            .into_iter()
            .filter(|pkg| {
                // Get base package name (without version)
                let base_name = pkg.split('@').next().unwrap_or(pkg);
                !core_crates.contains(&base_name)
            })
            .collect();

        // Reconstruct set with only HORUS registry packages and pip packages
        registry_horus_pkgs
            .into_iter()
            .chain(pip_pkgs.into_iter().map(|p| {
                if let Some(ref v) = p.version {
                    format!("pip:{}=={}", p.name, v)
                } else {
                    format!("pip:{}", p.name)
                }
            }))
            .collect()
    };

    // Resolve all dependencies once (excluding cargo packages)
    if !dependencies_to_resolve.is_empty() {
        install::resolve_dependencies(dependencies_to_resolve)?;
    }

    // Generate single Cargo.toml with multiple binary targets
    let cargo_toml_path = PathBuf::from(".horus/Cargo.toml");

    let mut cargo_toml = String::from(
        r#"[package]
name = "horus-multi-node"
version = "0.1.9"
edition = "2021"

# Opt out of parent workspace
[workspace]

"#,
    );

    // Add a [[bin]] entry for each Rust file
    let mut binary_names = Vec::new();
    for file_path in &file_paths {
        let name = file_path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("node")
            .to_string();

        let source_relative_path = format!("../{}", file_path.display());

        cargo_toml.push_str(&format!(
            r#"[[bin]]
name = "{}"
path = "{}"

"#,
            name, source_relative_path
        ));

        binary_names.push(name);
    }

    // Add dependencies section
    cargo_toml.push_str("[dependencies]\n");

    // Add HORUS core dependencies (all horus_* crates that might be imported)
    // This fixes Issue #26: users shouldn't need to explicitly add horus_core to horus.yaml
    if horus_source.ends_with(".horus/cache") || horus_source.ends_with(".horus\\cache") {
        let cache_base = horus_source.join("horus@0.1.0");
        cargo_toml.push_str(&format!(
            "horus = {{ path = \"{}\" }}\n",
            cache_base.join("horus").display()
        ));
        cargo_toml.push_str(&format!(
            "horus_core = {{ path = \"{}\" }}\n",
            cache_base.join("horus_core").display()
        ));
        cargo_toml.push_str(&format!(
            "horus_library = {{ path = \"{}\" }}\n",
            cache_base.join("horus_library").display()
        ));
        cargo_toml.push_str(&format!(
            "horus_macros = {{ path = \"{}\" }}\n",
            cache_base.join("horus_macros").display()
        ));
    } else {
        cargo_toml.push_str(&format!(
            "horus = {{ path = \"{}\" }}\n",
            horus_source.join("horus").display()
        ));
        cargo_toml.push_str(&format!(
            "horus_core = {{ path = \"{}\" }}\n",
            horus_source.join("horus_core").display()
        ));
        cargo_toml.push_str(&format!(
            "horus_library = {{ path = \"{}\" }}\n",
            horus_source.join("horus_library").display()
        ));
        cargo_toml.push_str(&format!(
            "horus_macros = {{ path = \"{}\" }}\n",
            horus_source.join("horus_macros").display()
        ));
    }

    // Add dependencies from horus.yaml
    if Path::new(HORUS_YAML).exists() {
        if let Ok(yaml_content) = fs::read_to_string(HORUS_YAML) {
            if let Ok(yaml) = serde_yaml::from_str::<serde_yaml::Value>(&yaml_content) {
                if let Some(serde_yaml::Value::Sequence(list)) = yaml.get("dependencies") {
                    for item in list {
                        if let Some(dep_str) = deps::parse_yaml_cargo_dependency(item) {
                            cargo_toml.push_str(&format!("{}\n", dep_str));
                        }
                    }
                }
            }
        }
    }

    // Write the unified Cargo.toml
    fs::write(&cargo_toml_path, cargo_toml)?;

    // Clean if requested
    if clean {
        let mut clean_cmd = Command::new("cargo");
        clean_cmd.arg("clean").current_dir(".horus");
        clean_cmd.status().ok();
    }

    // Build all binaries with a single cargo build command
    let mut cmd = Command::new("cargo");
    cmd.arg("build").current_dir(".horus");
    if release {
        cmd.arg("--release");
    }

    // Add features from drivers and enable configuration
    if let Some(features) = features::get_all_cargo_features() {
        cmd.arg("--features").arg(&features);
        eprintln!(
            "  {} Auto-enabling features: {}",
            "󰢱".cyan(),
            features.green()
        );
    }

    let status = cmd.status()?;
    if !status.success() {
        bail!("Cargo build failed for batch Rust compilation");
    }

    // Create ExecutableInfo for each binary
    let profile = if release { "release" } else { "debug" };
    let mut executables = Vec::new();

    for name in binary_names {
        let binary_path = format!(".horus/target/{}/{}", profile, name);
        executables.push(ExecutableInfo {
            name,
            command: binary_path,
            args_override: Vec::new(),
        });
    }

    Ok(executables)
}

fn build_file_for_concurrent_execution(
    file_path: PathBuf,
    language: String,
    release: bool,
    clean: bool,
) -> Result<ExecutableInfo> {
    let name = file_path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("node")
        .to_string();

    // Ensure .horus directory exists
    ensure_horus_directory()?;

    // Load ignore patterns from horus.yaml if it exists
    let ignore = if Path::new(HORUS_YAML).exists() {
        features::parse_horus_yaml_ignore(HORUS_YAML).unwrap_or_default()
    } else {
        features::IgnorePatterns::default()
    };

    // Scan imports and resolve dependencies
    let dependencies = deps::scan_imports(&file_path, &language, &ignore)?;
    if !dependencies.is_empty() {
        // Resolve dependencies with language context
        install::resolve_dependencies_with_context(dependencies, Some(&language))?;
    }

    // Setup environment
    setup_environment()?;

    match language.as_str() {
        "rust" => {
            // Build Rust file with Cargo
            let horus_source = find_horus_source_dir()?;
            let cargo_toml_path = PathBuf::from(".horus/Cargo.toml");
            let source_relative_path = format!("../{}", file_path.display());

            let mut cargo_toml = format!(
                r#"[package]
name = "horus-project-{}"
version = "0.1.9"
edition = "2021"

[[bin]]
name = "{}"
path = "{}"

[dependencies]
"#,
                name, name, source_relative_path
            );

            // Add HORUS dependencies (all horus_* crates that might be imported)
            // This fixes Issue #26: users shouldn't need to explicitly add horus_core to horus.yaml
            if horus_source.ends_with(".horus/cache") || horus_source.ends_with(".horus\\cache") {
                let cache_base = horus_source.join("horus@0.1.0");
                cargo_toml.push_str(&format!(
                    "horus = {{ path = \"{}\" }}\n",
                    cache_base.join("horus").display()
                ));
                cargo_toml.push_str(&format!(
                    "horus_core = {{ path = \"{}\" }}\n",
                    cache_base.join("horus_core").display()
                ));
                cargo_toml.push_str(&format!(
                    "horus_library = {{ path = \"{}\" }}\n",
                    cache_base.join("horus_library").display()
                ));
                cargo_toml.push_str(&format!(
                    "horus_macros = {{ path = \"{}\" }}\n",
                    cache_base.join("horus_macros").display()
                ));
            } else {
                cargo_toml.push_str(&format!(
                    "horus = {{ path = \"{}\" }}\n",
                    horus_source.join("horus").display()
                ));
                cargo_toml.push_str(&format!(
                    "horus_core = {{ path = \"{}\" }}\n",
                    horus_source.join("horus_core").display()
                ));
                cargo_toml.push_str(&format!(
                    "horus_library = {{ path = \"{}\" }}\n",
                    horus_source.join("horus_library").display()
                ));
                cargo_toml.push_str(&format!(
                    "horus_macros = {{ path = \"{}\" }}\n",
                    horus_source.join("horus_macros").display()
                ));
            }

            // Add dependencies from horus.yaml
            if Path::new(HORUS_YAML).exists() {
                if let Ok(yaml_content) = fs::read_to_string(HORUS_YAML) {
                    if let Ok(yaml) = serde_yaml::from_str::<serde_yaml::Value>(&yaml_content) {
                        if let Some(serde_yaml::Value::Sequence(list)) = yaml.get("dependencies") {
                            for item in list {
                                if let Some(dep_str) = deps::parse_yaml_cargo_dependency(item) {
                                    cargo_toml.push_str(&format!("{}\n", dep_str));
                                }
                            }
                        }
                    }
                }
            }

            fs::write(&cargo_toml_path, cargo_toml)?;

            if clean {
                let mut clean_cmd = Command::new("cargo");
                clean_cmd.arg("clean").current_dir(".horus");
                clean_cmd.status().ok();
            }

            // Build with Cargo
            let mut cmd = Command::new("cargo");
            cmd.arg("build").current_dir(".horus");
            if release {
                cmd.arg("--release");
            }
            cmd.arg("--bin").arg(&name);

            let status = cmd.status()?;
            if !status.success() {
                bail!("Cargo build failed for {}", name);
            }

            let profile = if release { "release" } else { "debug" };
            let binary_path = format!(".horus/target/{}/{}", profile, name);

            Ok(ExecutableInfo {
                name,
                command: binary_path,
                args_override: Vec::new(),
            })
        }
        "python" => {
            // Python doesn't need building, just setup interpreter
            let python_cmd = detect_python_interpreter()?;
            setup_python_environment()?;

            Ok(ExecutableInfo {
                name,
                command: python_cmd,
                args_override: vec![file_path.to_string_lossy().to_string()],
            })
        }
        _ => bail!("Unsupported language: {}", language),
    }
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
            "•".cyan(), ".rs, .py, .horus".green(),
            "•".cyan(), "Use quotes around patterns like \"nodes/*.py\"".dimmed()
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
        "•".cyan(), "main.rs or main.py".green(),
        "•".cyan(), "horus run myfile.rs".green(),
        "•".cyan(), "horus run src/".green()
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
    let global_cache = crate::paths::cache_dir().unwrap_or_else(|_| install::home_dir().join(".horus/cache"));
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

fn execute_python_node(file: PathBuf, args: Vec<String>, _release: bool) -> Result<()> {
    eprintln!("{} Setting up Python environment...", "".cyan());

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
            "".cyan()
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
        eprintln!("{} Executing Python script directly...", "".cyan());

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

fn setup_python_environment() -> Result<()> {
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

fn clean_build_cache() -> Result<()> {
    // Clean .horus/cache directory (where compiled binaries are stored)
    let cache_dir = PathBuf::from(".horus/cache");
    if cache_dir.exists() {
        for entry in fs::read_dir(&cache_dir)? {
            let entry = entry?;
            fs::remove_file(entry.path()).ok();
        }
        println!("  {} Cleaned .horus/cache/", "".green());
    }

    // Clean .horus/bin directory
    let bin_dir = PathBuf::from(".horus/bin");
    if bin_dir.exists() {
        for entry in fs::read_dir(&bin_dir)? {
            let entry = entry?;
            fs::remove_file(entry.path()).ok();
        }
        println!("  {} Cleaned .horus/bin/", "".green());
    }

    // Clean Rust target directory if exists
    let target_dir = PathBuf::from("target");
    if target_dir.exists() {
        fs::remove_dir_all(&target_dir)?;
        println!("  {} Cleaned target/", "".green());
    }

    // Clean Python __pycache__ in current directory
    let pycache = PathBuf::from("__pycache__");
    if pycache.exists() {
        fs::remove_dir_all(&pycache)?;
        println!("  {} Cleaned __pycache__/", "".green());
    }

    Ok(())
}

fn execute_with_scheduler(
    file: PathBuf,
    language: String,
    args: Vec<String>,
    release: bool,
    clean: bool,
) -> Result<()> {
    match language.as_str() {
        "rust" => {
            // Use Cargo-based compilation (same as horus.yaml path)
            println!("{} Setting up Cargo workspace...", "".cyan());

            // Parse horus.yaml to get dependencies
            let (horus_deps, cargo_packages, path_deps, git_deps) =
                if Path::new(HORUS_YAML).exists() {
                    let dep_set = deps::parse_horus_yaml_dependencies(HORUS_YAML)?;
                    let (horus_pkgs, _pip_pkgs, cargo_pkgs, path_pkgs, git_pkgs) =
                        deps::split_dependencies_with_path_context(dep_set, Some("rust"));
                    (horus_pkgs, cargo_pkgs, path_pkgs, git_pkgs)
                } else {
                    (Vec::new(), Vec::new(), Vec::new(), Vec::new())
                };

            // Find HORUS source directory
            let horus_source = find_horus_source_dir()?;
            println!(
                "  {} Using HORUS source: {}",
                "".cyan(),
                horus_source.display()
            );

            // Generate Cargo.toml in .horus/ that references the source file
            let cargo_toml_path = PathBuf::from(".horus/Cargo.toml");

            // Get relative path from .horus/ to the source file
            let source_relative_path = format!("../{}", file.display());

            let mut cargo_toml = format!(
                r#"[package]
name = "horus-project"
version = "0.1.9"
edition = "2021"

# Empty workspace to prevent inheriting parent workspace
[workspace]

[[bin]]
name = "horus-project"
path = "{}"

[dependencies]
"#,
                source_relative_path
            );

            // Auto-detect nodes and required features
            use crate::node_detector;
            let auto_features = node_detector::detect_features_from_file(&file).unwrap_or_default();
            if !auto_features.is_empty() {
                eprintln!(
                    "  {} Auto-detected hardware nodes (features: {})",
                    "".cyan(),
                    auto_features.join(", ").yellow()
                );

                // Check system dependencies for detected features
                use crate::system_deps;
                let dep_result = system_deps::check_dependencies(&auto_features);
                let report = system_deps::format_dependency_report(&dep_result, &auto_features);
                if !report.is_empty() {
                    eprintln!("{}", report);
                }
            }

            // Add HORUS dependencies from horus.yaml or defaults
            let horus_packages_to_add = if !horus_deps.is_empty() {
                horus_deps
            } else {
                // Default HORUS packages if no horus.yaml
                vec!["horus".to_string(), "horus_library".to_string()]
            };

            for dep in &horus_packages_to_add {
                // Strip version from dependency name for path lookup
                let dep_name = if let Some(at_pos) = dep.find('@') {
                    &dep[..at_pos]
                } else {
                    dep.as_str()
                };

                let dep_path = horus_source.join(dep_name);
                if dep_path.exists() && dep_path.join(CARGO_TOML).exists() {
                    // Auto-inject features for horus or horus_library
                    if (dep_name == "horus" || dep_name == "horus_library")
                        && !auto_features.is_empty()
                    {
                        cargo_toml.push_str(&format!(
                            "{} = {{ path = \"{}\", features = [{}] }}\n",
                            dep_name,
                            dep_path.display(),
                            auto_features
                                .iter()
                                .map(|f| format!("\"{}\"", f))
                                .collect::<Vec<_>>()
                                .join(", ")
                        ));
                        println!(
                            "  {} Added dependency: {} -> {} (auto-features: {})",
                            "".cyan(),
                            dep,
                            dep_path.display(),
                            auto_features.join(", ").yellow()
                        );
                    } else {
                        cargo_toml.push_str(&format!(
                            "{} = {{ path = \"{}\" }}\n",
                            dep_name,
                            dep_path.display()
                        ));
                        println!(
                            "  {} Added dependency: {} -> {}",
                            "".cyan(),
                            dep,
                            dep_path.display()
                        );
                    }
                } else {
                    eprintln!(
                        "  {} Warning: dependency {} not found at {}",
                        "".yellow(),
                        dep,
                        dep_path.display()
                    );
                }
            }

            // Add cargo dependencies from crates.io
            for pkg in &cargo_packages {
                if !pkg.features.is_empty() {
                    // With features
                    if let Some(ref version) = pkg.version {
                        cargo_toml.push_str(&format!(
                            "{} = {{ version = \"{}\", features = [{}] }}\n",
                            pkg.name,
                            version,
                            pkg.features
                                .iter()
                                .map(|f| format!("\"{}\"", f))
                                .collect::<Vec<_>>()
                                .join(", ")
                        ));
                    } else {
                        cargo_toml.push_str(&format!(
                            "{} = {{ version = \"*\", features = [{}] }}\n",
                            pkg.name,
                            pkg.features
                                .iter()
                                .map(|f| format!("\"{}\"", f))
                                .collect::<Vec<_>>()
                                .join(", ")
                        ));
                        eprintln!(
                            "  {} Warning: Using wildcard version for '{}' - specify a version for reproducibility",
                            "".yellow(),
                            pkg.name
                        );
                    }
                    println!(
                        "  {} Added crates.io dependency: {} (features: {})",
                        "".cyan(),
                        pkg.name,
                        pkg.features.join(", ")
                    );
                } else if let Some(ref version) = pkg.version {
                    cargo_toml.push_str(&format!("{} = \"{}\"\n", pkg.name, version));
                    println!(
                        "  {} Added crates.io dependency: {}@{}",
                        "".cyan(),
                        pkg.name,
                        version
                    );
                } else {
                    cargo_toml.push_str(&format!("{} = \"*\"\n", pkg.name));
                    eprintln!(
                        "  {} Warning: Using wildcard version for '{}' - specify a version for reproducibility",
                        "".yellow(),
                        pkg.name
                    );
                    eprintln!("     Example: 'cargo:{}@1.0' in horus.yaml", pkg.name);
                    println!("  {} Added crates.io dependency: {}", "".cyan(), pkg.name);
                }
            }

            // Add path dependencies
            for (pkg_name, pkg_path) in &path_deps {
                // Convert relative path from current directory to relative from .horus/
                let full_path = PathBuf::from("..").join(pkg_path);
                cargo_toml.push_str(&format!(
                    "{} = {{ path = \"{}\" }}\n",
                    pkg_name,
                    full_path.display()
                ));
                println!(
                    "  {} Added path dependency: {} -> {}",
                    "✓".cyan(),
                    pkg_name,
                    pkg_path
                );
            }

            // Add git dependencies (clone to cache, then use as path dependencies)
            if !git_deps.is_empty() {
                println!("{} Resolving git dependencies...", "📦".cyan());
                let resolved_git_deps = install::resolve_git_dependencies(&git_deps)?;
                for (pkg_name, pkg_path) in &resolved_git_deps {
                    cargo_toml.push_str(&format!(
                        "{} = {{ path = \"{}\" }}\n",
                        pkg_name,
                        pkg_path.display()
                    ));
                    println!(
                        "  {} Added git dependency: {} -> {}",
                        "✓".green(),
                        pkg_name,
                        pkg_path.display()
                    );
                }
            }

            // Also add dependencies directly from horus.yaml (in case some weren't parsed by resolve_dependencies)
            // Track already-added cargo packages to avoid duplicates
            let added_cargo_deps: HashSet<String> =
                cargo_packages.iter().map(|pkg| pkg.name.clone()).collect();

            if Path::new(HORUS_YAML).exists() {
                if let Ok(yaml_content) = fs::read_to_string(HORUS_YAML) {
                    if let Ok(yaml) = serde_yaml::from_str::<serde_yaml::Value>(&yaml_content) {
                        if let Some(serde_yaml::Value::Sequence(list)) = yaml.get("dependencies") {
                            for item in list {
                                if let Some(dep_str) = deps::parse_yaml_cargo_dependency(item) {
                                    // Extract dependency name from the generated string (e.g., "serde = ..." -> "serde")
                                    let dep_name = dep_str.split('=').next().unwrap_or(&dep_str).trim();

                                    // Skip if already added from cargo_packages
                                    if !added_cargo_deps.contains(dep_name) {
                                        cargo_toml.push_str(&format!("{}\n", dep_str));
                                    }
                                }
                            }
                        }
                    }
                }
            }

            fs::write(&cargo_toml_path, cargo_toml)?;
            println!("  {} Generated Cargo.toml", "".green());

            // Run cargo clean if requested
            if clean {
                println!("{} Cleaning build artifacts...", "".cyan());
                let mut clean_cmd = Command::new("cargo");
                clean_cmd.arg("clean");
                clean_cmd.current_dir(".horus");
                let status = clean_cmd.status()?;
                if !status.success() {
                    log::warn!("cargo clean failed");
                }
            }

            // Run cargo build in .horus directory
            println!("{} Building with Cargo...", "".cyan());
            let mut cmd = Command::new("cargo");
            cmd.arg("build");
            cmd.current_dir(".horus");
            if release {
                cmd.arg("--release");
            }

            // Add features from drivers and enable configuration
            if let Some(features) = features::get_all_cargo_features() {
                cmd.arg("--features").arg(&features);
                println!(
                    "  {} Auto-enabling features: {}",
                    "󰢱".cyan(),
                    features.green()
                );
            }

            log::debug!("cargo build command: {:?}", cmd);

            let status = cmd.status()?;
            if !status.success() {
                bail!("Cargo build failed");
            }

            // Determine binary path
            let binary_path = if release {
                ".horus/target/release/horus-project"
            } else {
                ".horus/target/debug/horus-project"
            };

            // Execute the binary
            println!("{} Executing...\n", "".cyan());
            let mut cmd = Command::new(binary_path);
            cmd.args(args);

            let status = cmd.status()?;

            // Exit with the same code as the program
            if !status.success() {
                std::process::exit(status.code().unwrap_or(1));
            }
        }
        "python" => {
            execute_python_node(file, args, release)?;
        }
        _ => bail!(
            "Unsupported language: {}. HORUS supports Rust and Python only.",
            language
        ),
    }

    Ok(())
}

fn get_project_name() -> Result<String> {
    // Try to get from Cargo.toml
    if Path::new(CARGO_TOML).exists() {
        let content = fs::read_to_string(CARGO_TOML)?;
        for line in content.lines() {
            if let Some(rest) = line.strip_prefix("name = ") {
                let name = rest.trim_matches('"').trim_matches('\'');
                return Ok(name.to_string());
            }
        }
    }

    // Fallback to directory name
    let current_dir = env::current_dir()?;
    Ok(current_dir
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("main")
        .to_string())
}

// Removed: setup_c_environment() - C support no longer provided
// Removed: find_horus_cpp_library() - C++ bindings no longer supported
// Removed: execute_c_node() - C support no longer provided

/// Find the HORUS source directory by checking common locations
fn find_horus_source_dir() -> Result<PathBuf> {
    log::debug!("searching for HORUS source directory");

    // Check environment variable first
    if let Ok(horus_source) = env::var("HORUS_SOURCE") {
        let path = PathBuf::from(horus_source);
        if path.exists() && path.join("horus/Cargo.toml").exists() {
            log::debug!("found HORUS source via HORUS_SOURCE env: {:?}", path);
            return Ok(path);
        }
    }

    // Check common development locations
    let candidates = vec![
        PathBuf::from("/horus"),
        install::home_dir().join("softmata/horus"),
        install::home_dir().join("horus"),
        PathBuf::from("/opt/horus"),
        PathBuf::from("/usr/local/horus"),
    ];

    for candidate in candidates {
        if candidate.exists() && candidate.join("horus/Cargo.toml").exists() {
            log::debug!("found HORUS source at candidate: {:?}", candidate);
            return Ok(candidate);
        }
    }

    // Fallback: Check for installed packages in cache
    let cache_dir = crate::paths::cache_dir().unwrap_or_else(|_| install::home_dir().join(".horus/cache"));
    if cache_dir.join("horus@0.1.0").exists() {
        log::debug!("found HORUS source in cache: {:?}", cache_dir);
        return Ok(cache_dir);
    }

    log::warn!("HORUS source directory not found in any known location");

    bail!(
        "HORUS source not found. This can happen after running 'horus clean -a'.\n\n\
         To fix this, either:\n\
         1. Re-run the install script: ./install.sh (or curl the installer)\n\
         2. Set HORUS_SOURCE environment variable to your HORUS source directory\n\
         3. Clone HORUS to ~/softmata/horus or ~/horus"
    )
}
