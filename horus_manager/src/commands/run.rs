use crate::commands::driver::resolve_driver_alias;
use crate::dependency_resolver::DependencySpec;
use crate::progress::{self, finish_error, finish_success};
use crate::version;
use anyhow::{anyhow, bail, Context, Result};
use colored::*;
use glob::glob;
use horus_core::params::RuntimeParams;
use std::collections::HashSet;
use std::env;
use std::fs;
use std::io::{self, Write};
#[cfg(unix)]
use std::os::unix::fs::symlink;
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

/// Python package dependency for pip
#[derive(Debug, Clone)]
struct PipPackage {
    name: String,
    version: Option<String>, // None means latest
}

impl PipPackage {
    fn from_string(s: &str) -> Result<Self> {
        // Parse formats:
        // - "numpy>=1.24.0"
        // - "numpy==1.24.0"
        // - "numpy~=1.24"
        // - "numpy@1.24.0" (HORUS-style)
        // - "numpy"

        let s = s.trim();

        // Handle @ separator (HORUS-style: numpy@1.24.0)
        if let Some(at_pos) = s.find('@') {
            let name = s[..at_pos].trim().to_string();
            let version_str = s[at_pos + 1..].trim();
            let version = if !version_str.is_empty() {
                Some(format!("=={}", version_str))
            } else {
                None
            };
            return Ok(PipPackage { name, version });
        }

        // Handle comparison operators (>=, ==, ~=, etc.)
        let operators = [">=", "<=", "==", "~=", ">", "<", "!="];
        for op in &operators {
            if let Some(op_pos) = s.find(op) {
                let name = s[..op_pos].trim().to_string();
                let version = Some(s[op_pos..].trim().to_string());
                return Ok(PipPackage { name, version });
            }
        }

        // No version specified
        Ok(PipPackage {
            name: s.to_string(),
            version: None,
        })
    }

    fn requirement_string(&self) -> String {
        match &self.version {
            Some(v) => format!("{}{}", self.name, v),
            None => self.name.clone(),
        }
    }
}

/// Cargo package dependency for Rust binaries
#[derive(Debug, Clone)]
struct CargoPackage {
    name: String,
    version: Option<String>, // None means latest
    features: Vec<String>,   // Cargo features to enable
}

impl CargoPackage {
    fn from_string(s: &str) -> Result<Self> {
        // Parse formats: "bat@0.24.0:features=derive,serde" or "bat@0.24.0" or "bat"
        let s = s.trim();

        // Check for features
        let (pkg_part, features) = if let Some(colon_pos) = s.find(':') {
            let pkg = &s[..colon_pos];
            let features_part = &s[colon_pos + 1..];

            // Parse features=a,b,c
            let features = if let Some(equals_pos) = features_part.find('=') {
                let features_str = &features_part[equals_pos + 1..].trim();
                if features_str.is_empty() {
                    return Err(anyhow::anyhow!(
                        "Empty features list (remove ':features=' or provide feature names)"
                    ));
                }
                features_str
                    .split(',')
                    .map(|f| f.trim().to_string())
                    .filter(|f| !f.is_empty()) // Filter out empty strings
                    .collect()
            } else if !features_part.is_empty() {
                return Err(anyhow::anyhow!(
                    "Invalid features syntax '{}' - use 'features=feat1,feat2'",
                    features_part
                ));
            } else {
                Vec::new()
            };
            (pkg, features)
        } else {
            (s, Vec::new())
        };

        // Parse name@version
        if let Some(at_pos) = pkg_part.find('@') {
            let name = pkg_part[..at_pos].trim().to_string();
            let version_str = pkg_part[at_pos + 1..].trim();

            // Validate package name
            if name.is_empty() {
                return Err(anyhow::anyhow!("Package name cannot be empty"));
            }

            let version = if !version_str.is_empty() {
                // Basic semver validation
                if version_str.contains(char::is_whitespace) {
                    return Err(anyhow::anyhow!(
                        "Version cannot contain whitespace: '{}'",
                        version_str
                    ));
                }
                // Check for common mistakes
                if version_str == "latest" {
                    return Err(anyhow::anyhow!("Version 'latest' is not valid - specify a version like '1.0' or omit version"));
                }
                Some(version_str.to_string())
            } else {
                None
            };
            return Ok(CargoPackage {
                name,
                version,
                features,
            });
        }

        // No version specified
        let name = pkg_part.trim().to_string();
        if name.is_empty() {
            return Err(anyhow::anyhow!("Package name cannot be empty"));
        }

        Ok(CargoPackage {
            name,
            version: None,
            features,
        })
    }
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
    let language = detect_language(target_file)?;
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
            let dependencies = if Path::new("horus.yaml").exists() {
                parse_horus_yaml_dependencies("horus.yaml")?
            } else {
                HashSet::new()
            };

            // Split dependencies into HORUS packages, pip packages, cargo packages, path and git dependencies
            let (horus_deps, _pip_packages, cargo_packages, path_deps, git_deps) =
                split_dependencies_with_path_context(dependencies.clone(), Some("rust"));

            // Generate Cargo.toml in .horus/ that references source files in parent directory
            let cargo_toml_path = PathBuf::from(".horus/Cargo.toml");

            // Get relative path from .horus/ to the source file
            let source_relative_path = format!("../{}", target_file.display());

            let mut cargo_toml = format!(
                r#"[package]
name = "horus-project"
version = "0.1.8"
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

                if dep_path.exists() && dep_path.join("Cargo.toml").exists() {
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
                let resolved_git_deps = resolve_git_dependencies(&git_deps)?;
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

            if Path::new("horus.yaml").exists() {
                if let Ok(yaml_content) = fs::read_to_string("horus.yaml") {
                    if let Ok(yaml) = serde_yaml::from_str::<serde_yaml::Value>(&yaml_content) {
                        if let Some(serde_yaml::Value::Sequence(list)) = yaml.get("dependencies") {
                            for item in list {
                                if let Some(dep_str) = parse_yaml_cargo_dependency(item) {
                                    // Extract dependency name from the generated string (e.g., "serde = ..." -> "serde")
                                    let dep_name = dep_str.split('=').next().unwrap().trim();

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
            if let Some(features) = get_all_cargo_features() {
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
    load_params_from_project()?;

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
    let language = detect_language(&file_path)?;

    eprintln!(
        "{} Detected: {} ({})",
        "".cyan(),
        file_path.display().to_string().green(),
        language.yellow()
    );

    // Load ignore patterns from horus.yaml if it exists
    let ignore = if Path::new("horus.yaml").exists() {
        parse_horus_yaml_ignore("horus.yaml").unwrap_or_default()
    } else {
        IgnorePatterns::default()
    };

    // Ensure .horus directory exists
    ensure_horus_directory()?;

    // Scan imports and resolve dependencies
    eprintln!("{} Scanning imports...", "".cyan());
    let dependencies = scan_imports(&file_path, &language, &ignore)?;

    // Check hardware requirements
    if let Err(e) = check_hardware_requirements(&file_path, &language) {
        eprintln!("[WARNING] Hardware check error: {}", e);
    }

    if !dependencies.is_empty() {
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
            resolve_dependencies_with_context(deps_to_resolve, Some(&language))?;
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
        Some("horus.yaml") => execute_from_horus_yaml(manifest_path, args, release, clean),
        Some("Cargo.toml") => execute_from_cargo_toml(manifest_path, args, release, clean),
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
        .ok_or_else(|| anyhow::anyhow!("Cannot determine project directory"))?;

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
        .ok_or_else(|| anyhow::anyhow!("Cannot determine project directory"))?;

    let original_dir = env::current_dir()?;
    env::set_current_dir(project_dir)?;

    let result = (|| -> Result<()> {
        // Ensure .horus directory exists
        ensure_horus_directory()?;

        // Parse Cargo.toml for HORUS dependencies
        println!("{} Scanning Cargo.toml dependencies...", "".cyan());
        let horus_deps = parse_cargo_dependencies("Cargo.toml")?;

        if !horus_deps.is_empty() {
            println!(
                "{} Found {} HORUS dependencies",
                "".cyan(),
                horus_deps.len()
            );
            resolve_dependencies(horus_deps)?;
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
        let language = detect_language(file_path)?;
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
        let language = detect_language(file_path)?;
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
                children.lock().unwrap().push((node_name, child));
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
        let mut children_lock = children.lock().unwrap();

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
    let ignore = if Path::new("horus.yaml").exists() {
        parse_horus_yaml_ignore("horus.yaml").unwrap_or_default()
    } else {
        IgnorePatterns::default()
    };

    // Find HORUS source directory
    let horus_source = find_horus_source_dir()?;

    // Collect all dependencies from all Rust files
    let mut all_dependencies = HashSet::new();
    for file_path in &file_paths {
        let dependencies = scan_imports(file_path, "rust", &ignore)?;
        all_dependencies.extend(dependencies);
    }

    // For Rust files, cargo dependencies are handled in Cargo.toml generation
    // So filter them out here to avoid trying to `cargo install` library crates
    let dependencies_to_resolve: HashSet<String> = {
        let (horus_pkgs, pip_pkgs, _cargo_pkgs) =
            split_dependencies_with_context(all_dependencies.clone(), Some("rust"));

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
        resolve_dependencies(dependencies_to_resolve)?;
    }

    // Generate single Cargo.toml with multiple binary targets
    let cargo_toml_path = PathBuf::from(".horus/Cargo.toml");

    let mut cargo_toml = String::from(
        r#"[package]
name = "horus-multi-node"
version = "0.1.8"
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
    if Path::new("horus.yaml").exists() {
        if let Ok(yaml_content) = fs::read_to_string("horus.yaml") {
            if let Ok(yaml) = serde_yaml::from_str::<serde_yaml::Value>(&yaml_content) {
                if let Some(serde_yaml::Value::Sequence(list)) = yaml.get("dependencies") {
                    for item in list {
                        if let Some(dep_str) = parse_yaml_cargo_dependency(item) {
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
    if let Some(features) = get_all_cargo_features() {
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
    let ignore = if Path::new("horus.yaml").exists() {
        parse_horus_yaml_ignore("horus.yaml").unwrap_or_default()
    } else {
        IgnorePatterns::default()
    };

    // Scan imports and resolve dependencies
    let dependencies = scan_imports(&file_path, &language, &ignore)?;
    if !dependencies.is_empty() {
        // Resolve dependencies with language context
        resolve_dependencies_with_context(dependencies, Some(&language))?;
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
version = "0.1.8"
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
            if Path::new("horus.yaml").exists() {
                if let Ok(yaml_content) = fs::read_to_string("horus.yaml") {
                    if let Ok(yaml) = serde_yaml::from_str::<serde_yaml::Value>(&yaml_content) {
                        if let Some(serde_yaml::Value::Sequence(list)) = yaml.get("dependencies") {
                            for item in list {
                                if let Some(dep_str) = parse_yaml_cargo_dependency(item) {
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

    // Check for glob patterns
    if input_str.contains('*') || input_str.contains('?') || input_str.contains('[') {
        return resolve_glob_pattern(&input_str);
    }

    if input.is_file() {
        // Check if it's a manifest file
        match input.extension().and_then(|s| s.to_str()) {
            Some("yaml") | Some("yml") => {
                if input.file_name().and_then(|s| s.to_str()) == Some("horus.yaml") {
                    return Ok(vec![ExecutionTarget::Manifest(input)]);
                }
            }
            Some("toml") => {
                if input.file_name().and_then(|s| s.to_str()) == Some("Cargo.toml") {
                    return Ok(vec![ExecutionTarget::Manifest(input)]);
                }
            }
            _ => {}
        }

        // Regular file
        return Ok(vec![ExecutionTarget::File(input)]);
    }

    if input.is_dir() {
        return Ok(vec![ExecutionTarget::Directory(input)]);
    }

    bail!("Target not found: {}", input.display())
}

fn resolve_glob_pattern(pattern: &str) -> Result<Vec<ExecutionTarget>> {
    // Load ignore patterns from horus.yaml if it exists
    let ignore = if Path::new("horus.yaml").exists() {
        parse_horus_yaml_ignore("horus.yaml").unwrap_or_default()
    } else {
        IgnorePatterns::default()
    };

    let mut files = Vec::new();

    for entry in glob(pattern).context("Failed to parse glob pattern")? {
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
            Err(e) => eprintln!("Warning: Glob error: {}", e),
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
            files.into_iter().next().unwrap(),
        )])
    } else {
        Ok(vec![ExecutionTarget::Multiple(files)])
    }
}

fn auto_detect_main_file() -> Result<PathBuf> {
    // Load ignore patterns from horus.yaml if it exists
    let ignore = if Path::new("horus.yaml").exists() {
        parse_horus_yaml_ignore("horus.yaml").unwrap_or_default()
    } else {
        IgnorePatterns::default()
    };

    // Check for main files in priority order (Rust and Python only)
    let candidates = ["main.rs", "main.py", "src/main.rs", "src/main.py"];

    for candidate in &candidates {
        let path = PathBuf::from(candidate);
        if path.exists() && !ignore.should_ignore_file(&path) {
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

fn detect_language(file: &Path) -> Result<String> {
    match file.extension().and_then(|s| s.to_str()) {
        Some("rs") => Ok("rust".to_string()),
        Some("py") => Ok("python".to_string()),
        _ => bail!(
            "Unsupported file type: {}\n\n{}\n  {} Supported: {}\n  {} Got: {}",
            file.display(),
            "Supported file types:".yellow(),
            "•".cyan(),
            ".rs (Rust), .py (Python)".green(),
            "•".cyan(),
            file.extension()
                .and_then(|s| s.to_str())
                .unwrap_or("no extension")
                .red()
        ),
    }
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

fn scan_imports(file: &Path, language: &str, ignore: &IgnorePatterns) -> Result<HashSet<String>> {
    let content = fs::read_to_string(file)?;
    let mut dependencies = HashSet::new();

    // First, check if horus.yaml exists and use it
    let from_yaml = Path::new("horus.yaml").exists();

    if from_yaml {
        eprintln!("  {} Reading dependencies from horus.yaml", "".cyan());
        let yaml_deps = parse_horus_yaml_dependencies("horus.yaml")?;
        dependencies.extend(yaml_deps);
    } else {
        // Fallback: scan imports from source code
        match language {
            "rust" => {
                // Scan for: use horus::*, use horus_library::*, extern crate
                for line in content.lines() {
                    if let Some(dep) = parse_rust_import(line) {
                        dependencies.insert(dep);
                    }
                }

                // Also check Cargo.toml if exists (legacy support)
                if Path::new("Cargo.toml").exists() {
                    let cargo_deps = parse_cargo_dependencies("Cargo.toml")?;
                    dependencies.extend(cargo_deps);
                }
            }
            "python" => {
                // Scan for ALL imports (not just HORUS)
                for line in content.lines() {
                    if let Some(dep) = parse_all_python_imports(line) {
                        dependencies.insert(dep);
                    }
                }
            }
            _ => {}
        }
    }

    // Filter out ignored packages
    dependencies.retain(|dep| !ignore.should_ignore_package(dep));

    // Auto-create or update horus.yaml if we scanned from source
    if !from_yaml && !dependencies.is_empty() {
        auto_update_horus_yaml(file, language, &dependencies)?;
    }

    Ok(dependencies)
}

fn parse_rust_import(line: &str) -> Option<String> {
    let line = line.trim();

    // use horus_library::*
    if let Some(rest) = line.strip_prefix("use ") {
        let parts: Vec<&str> = rest.split("::").collect();
        if !parts.is_empty() {
            let package = parts[0].trim_end_matches(';');
            if package.starts_with("horus") {
                return Some(package.to_string());
            }
        }
    }

    // extern crate horus_library
    if let Some(rest) = line.strip_prefix("extern crate ") {
        let package = rest.trim_end_matches(';').trim();
        if package.starts_with("horus") {
            return Some(package.to_string());
        }
    }

    None
}

/// Parse ALL Python imports (not just HORUS)
fn parse_all_python_imports(line: &str) -> Option<String> {
    let line = line.trim();

    // Skip comments
    if line.starts_with('#') {
        return None;
    }

    // import numpy
    // import numpy as np
    if let Some(rest) = line.strip_prefix("import ") {
        let package = rest.split_whitespace().next()?.split('.').next()?;
        // Skip relative imports and standard library
        if !is_stdlib_package(package) && !package.starts_with('.') {
            return Some(package.to_string());
        }
    }

    // from numpy import something
    if let Some(rest) = line.strip_prefix("from ") {
        let parts: Vec<&str> = rest.split(" import ").collect();
        if !parts.is_empty() {
            let package = parts[0].trim().split('.').next()?;
            // Skip relative imports and standard library
            if !is_stdlib_package(package) && !package.starts_with('.') {
                return Some(package.to_string());
            }
        }
    }

    None
}

/// Check if package is Python standard library
fn is_stdlib_package(name: &str) -> bool {
    let stdlib = [
        "os",
        "sys",
        "re",
        "json",
        "time",
        "datetime",
        "math",
        "random",
        "collections",
        "itertools",
        "functools",
        "pathlib",
        "typing",
        "asyncio",
        "threading",
        "multiprocessing",
        "subprocess",
        "logging",
        "argparse",
        "configparser",
        "io",
        "pickle",
        "csv",
        "xml",
        "html",
        "http",
        "urllib",
        "socket",
        "email",
        "base64",
        "hashlib",
        "hmac",
        "secrets",
        "uuid",
        "dataclasses",
        "enum",
        "abc",
        "contextlib",
    ];
    stdlib.contains(&name)
}

// Removed: parse_c_include() - C support no longer provided

/// Parse a single YAML dependency and convert to Cargo.toml format
/// Handles: - horus, - name: serde with version: "1" features: [derive]
fn parse_yaml_cargo_dependency(value: &serde_yaml::Value) -> Option<String> {
    match value {
        serde_yaml::Value::String(dep_str) => {
            // Simple string: - horus, - cargo:serde@1.0:features=derive, etc.
            let dep = dep_str.trim();

            // Skip horus packages - they're already added as path dependencies
            if dep == "horus" || dep.starts_with("horus_") || dep.starts_with("horus@") {
                return None;
            }

            // Parse cargo: or pip: prefixed dependencies
            let dep_clean = if let Some(rest) = dep.strip_prefix("cargo:") {
                rest // Remove "cargo:" prefix
            } else if dep.starts_with("pip:") {
                // Skip pip dependencies in Cargo.toml
                return None;
            } else {
                dep
            };

            // Parse package@version:features=feat1,feat2 format
            if let Some(at_pos) = dep_clean.find('@') {
                let pkg_name = dep_clean[..at_pos].trim();
                let rest = &dep_clean[at_pos + 1..];

                // Split version and features
                if let Some(features_pos) = rest.find(":features=") {
                    let version = rest[..features_pos].trim();
                    let features_str = rest[features_pos + 10..].trim(); // Skip ":features="
                    let features: Vec<&str> = features_str.split(',').map(|s| s.trim()).collect();

                    return Some(format!(
                        "{} = {{ version = \"{}\", features = [{}] }}",
                        pkg_name,
                        version,
                        features
                            .iter()
                            .map(|f| format!("\"{}\"", f))
                            .collect::<Vec<_>>()
                            .join(", ")
                    ));
                } else {
                    // Just version, no features
                    let version = rest.trim();
                    return Some(format!("{} = \"{}\"", pkg_name, version));
                }
            }

            // No version specified, use "*"
            Some(format!("{} = \"*\"", dep_clean))
        }
        serde_yaml::Value::Mapping(map) => {
            // Map format: - name: serde, version: "1", features: [derive]
            let name = map.get("name")?.as_str()?;

            if name == "horus" || name.starts_with("horus_") {
                return None; // Skip, already added
            }

            let mut cargo_dep = format!("{} = ", name);

            // Check for simple version string
            if let Some(version) = map.get("version").and_then(|v| v.as_str()) {
                // Check if features exist
                if let Some(features_val) = map.get("features") {
                    if let Some(features_list) = features_val.as_sequence() {
                        let features: Vec<&str> =
                            features_list.iter().filter_map(|f| f.as_str()).collect();

                        if !features.is_empty() {
                            cargo_dep.push_str(&format!(
                                "{{ version = \"{}\", features = [{}] }}",
                                version,
                                features
                                    .iter()
                                    .map(|f| format!("\"{}\"", f))
                                    .collect::<Vec<_>>()
                                    .join(", ")
                            ));
                            return Some(cargo_dep);
                        }
                    }
                }

                // No features, just version
                cargo_dep.push_str(&format!("\"{}\"", version));
                Some(cargo_dep)
            } else {
                // No version specified
                Some(format!("{} = \"*\"", name))
            }
        }
        _ => None,
    }
}

fn parse_horus_yaml_dependencies(path: &str) -> Result<HashSet<String>> {
    let content = fs::read_to_string(path)?;

    // Try to parse as proper YAML first (supports complex table syntax)
    match serde_yaml::from_str::<serde_yaml::Value>(&content) {
        Ok(yaml) => {
            let mut dependencies = HashSet::new();

            // Get language context
            let language = yaml
                .get("language")
                .and_then(|v| v.as_str())
                .map(|s| s.to_string());

            // Parse dependencies
            if let Some(deps_value) = yaml.get("dependencies") {
                match deps_value {
                    // Array format: dependencies: [- pkg, - pkg = "version"]
                    serde_yaml::Value::Sequence(list) => {
                        for item in list {
                            if let Some(dep_str) =
                                parse_dependency_value(item, language.as_deref())?
                            {
                                dependencies.insert(dep_str);
                            }
                        }
                    }

                    // Map format: dependencies: { pkg: version, pkg: {path: ...} }
                    serde_yaml::Value::Mapping(map) => {
                        for (key, value) in map {
                            if let serde_yaml::Value::String(pkg_name) = key {
                                if let Some(dep_str) = parse_dependency_map_entry(
                                    pkg_name,
                                    value,
                                    language.as_deref(),
                                )? {
                                    dependencies.insert(dep_str);
                                }
                            }
                        }
                    }

                    _ => {}
                }
            }

            Ok(dependencies)
        }

        // Fallback to simple line-by-line parsing for malformed YAML
        Err(_) => parse_horus_yaml_dependencies_simple(path),
    }
}

/// Parse a single dependency value from YAML (array item)
fn parse_dependency_value(
    value: &serde_yaml::Value,
    language: Option<&str>,
) -> Result<Option<String>> {
    match value {
        serde_yaml::Value::String(dep_str) => {
            let dep_str = dep_str.trim();
            if dep_str.is_empty() || dep_str.starts_with('#') {
                return Ok(None);
            }

            // Skip normalization for strings that already have package manager prefixes
            // (cargo: or pip:) to avoid corrupting feature syntax like ":features=derive"
            if dep_str.starts_with("cargo:") || dep_str.starts_with("pip:") {
                return Ok(Some(dep_str.to_string()));
            }

            // Normalize "pkg = version" syntax
            if let Some(equals_pos) = dep_str.find('=') {
                let pkg_name = dep_str[..equals_pos].trim();
                let version_part = dep_str[equals_pos + 1..].trim();
                let version = version_part.trim_matches('\'').trim_matches('"').trim();

                if !pkg_name.contains(':') {
                    let prefix = match language {
                        Some("python") => "pip",
                        Some("rust") => "cargo",
                        _ => "cargo",
                    };
                    return Ok(Some(format!("{}:{}@{}", prefix, pkg_name, version)));
                } else {
                    return Ok(Some(format!("{}@{}", pkg_name, version)));
                }
            }

            Ok(Some(dep_str.to_string()))
        }
        _ => Ok(None),
    }
}

/// Parse dependency from map format: pkg: {version: "1.0", features: [...]}
fn parse_dependency_map_entry(
    pkg_name: &str,
    value: &serde_yaml::Value,
    language: Option<&str>,
) -> Result<Option<String>> {
    match value {
        // Simple string version: pkg: "1.0"
        serde_yaml::Value::String(version_str) => {
            let version = version_str.trim_matches('\'').trim_matches('"').trim();
            let prefix = if pkg_name.contains(':') {
                ""
            } else {
                match language {
                    Some("python") => "pip:",
                    Some("rust") => "cargo:",
                    _ => "cargo:",
                }
            };
            Ok(Some(format!("{}{}@{}", prefix, pkg_name, version)))
        }

        // Table format: pkg: {version: "1.0", features: ["full"], path: "...", git: "..."}
        serde_yaml::Value::Mapping(map) => {
            // Check for empty map (pkg: {}) - treat as simple dependency
            if map.is_empty() {
                return Ok(Some(pkg_name.to_string()));
            }

            // Extract version
            let version = map
                .get("version")
                .and_then(|v| v.as_str())
                .map(|s| s.trim_matches('\'').trim_matches('"').trim());

            // Extract features
            let features: Vec<String> = map
                .get("features")
                .and_then(|v| v.as_sequence())
                .map(|seq| {
                    seq.iter()
                        .filter_map(|f| f.as_str().map(|s| s.to_string()))
                        .collect()
                })
                .unwrap_or_default();

            // Check for path dependency
            if let Some(path_value) = map.get("path") {
                if let Some(path_str) = path_value.as_str() {
                    // Path dependencies are now supported! Return special format: path:pkg_name:path_str
                    return Ok(Some(format!("path:{}:{}", pkg_name, path_str)));
                }
            }

            // Check for git dependency
            if let Some(git_value) = map.get("git") {
                if let Some(git_str) = git_value.as_str() {
                    // Extract optional branch, tag, or rev
                    let branch = map.get("branch").and_then(|v| v.as_str());
                    let tag = map.get("tag").and_then(|v| v.as_str());
                    let rev = map.get("rev").and_then(|v| v.as_str());

                    // Build git reference string
                    let ref_str = if let Some(b) = branch {
                        format!(":branch={}", b)
                    } else if let Some(t) = tag {
                        format!(":tag={}", t)
                    } else if let Some(r) = rev {
                        format!(":rev={}", r)
                    } else {
                        String::new()
                    };

                    // Return format: git:pkg_name:git_url[:branch=X|tag=X|rev=X]
                    return Ok(Some(format!("git:{}:{}{}", pkg_name, git_str, ref_str)));
                }
            }

            // Build dependency string
            let prefix = if pkg_name.contains(':') {
                ""
            } else {
                match language {
                    Some("python") => "pip:",
                    Some("rust") => "cargo:",
                    _ => "cargo:",
                }
            };

            if let Some(ver) = version {
                if !features.is_empty() {
                    Ok(Some(format!(
                        "{}{}@{}:features={}",
                        prefix,
                        pkg_name,
                        ver,
                        features.join(",")
                    )))
                } else {
                    Ok(Some(format!("{}{}@{}", prefix, pkg_name, ver)))
                }
            } else {
                // No version specified
                if !features.is_empty() {
                    Ok(Some(format!(
                        "{}{}:features={}",
                        prefix,
                        pkg_name,
                        features.join(",")
                    )))
                } else {
                    Ok(Some(format!("{}{}", prefix, pkg_name)))
                }
            }
        }

        _ => Ok(None),
    }
}

/// Fallback simple line-by-line parser for malformed YAML
fn parse_horus_yaml_dependencies_simple(path: &str) -> Result<HashSet<String>> {
    let content = fs::read_to_string(path)?;
    let mut dependencies = HashSet::new();

    // First, detect language from horus.yaml to determine default prefix
    let mut language = None;
    for line in content.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with("language:") {
            language = trimmed
                .strip_prefix("language:")
                .map(|s| s.trim().to_string());
            break;
        }
    }

    // Simple YAML parsing for dependencies section
    let mut in_deps = false;
    for line in content.lines() {
        let trimmed = line.trim();

        if trimmed.starts_with("dependencies:") {
            in_deps = true;
            continue;
        }

        // Exit dependencies section if we hit another top-level key
        if in_deps
            && !trimmed.is_empty()
            && !trimmed.starts_with("- ")
            && !trimmed.starts_with("#")
            && trimmed.contains(':')
        {
            in_deps = false;
        }

        if in_deps && trimmed.starts_with("- ") {
            // Extract full dependency string "package@version" or "package"
            let dep_str = trimmed[2..].trim();
            if dep_str.starts_with("#") {
                continue; // Skip comments
            }

            // Strip inline comments (handle both quoted and unquoted strings)
            let dep_str = if let Some(comment_pos) = dep_str.find('#') {
                // Check if the # is inside quotes
                let before_comment = &dep_str[..comment_pos];
                let single_quotes = before_comment.matches('\'').count();
                let double_quotes = before_comment.matches('"').count();

                // If quotes are balanced, # is a comment. Otherwise, it's part of the string
                if single_quotes.is_multiple_of(2) && double_quotes.is_multiple_of(2) {
                    dep_str[..comment_pos].trim()
                } else {
                    dep_str
                }
            } else {
                dep_str
            };

            // Remove surrounding quotes if present
            let dep_str = dep_str.trim_matches('\'').trim_matches('"');

            // Normalize package manager syntax: "eframe = \"0.29\"" -> "cargo:eframe@0.29" or "pip:numpy@1.24"
            let dep_str = if let Some(equals_pos) = dep_str.find('=') {
                let pkg_name = dep_str[..equals_pos].trim();
                let version_part = dep_str[equals_pos + 1..].trim();
                // Remove quotes from version
                let version = version_part.trim_matches('\'').trim_matches('"').trim();

                // If no prefix (cargo:/pip:/horus), infer from language context
                if !pkg_name.contains(':') {
                    let prefix = match language.as_deref() {
                        Some("python") => "pip",
                        Some("rust") => "cargo",
                        _ => "cargo", // Default to cargo if unknown
                    };
                    format!("{}:{}@{}", prefix, pkg_name, version)
                } else {
                    // Already has prefix, just reformat
                    format!("{}@{}", pkg_name, version)
                }
            } else {
                dep_str.to_string()
            };

            // Insert the full dependency string (including version)
            // This will be split later into HORUS vs pip packages
            dependencies.insert(dep_str);
        }
    }

    Ok(dependencies)
}

/// Parse horus.yaml dependencies with support for path, git, and registry sources
/// Returns `Vec<DependencySpec>` which includes source information
pub fn parse_horus_yaml_dependencies_v2(path: &str) -> Result<Vec<DependencySpec>> {
    let content = fs::read_to_string(path)?;

    // Try parsing as proper YAML first (supports structured format)
    match serde_yaml::from_str::<serde_yaml::Value>(&content) {
        Ok(yaml) => {
            let mut deps = Vec::new();

            if let Some(deps_value) = yaml.get("dependencies") {
                match deps_value {
                    // List format: dependencies: [- pkg@version]
                    serde_yaml::Value::Sequence(list) => {
                        for item in list {
                            if let serde_yaml::Value::String(dep_str) = item {
                                // Parse simple string format
                                deps.push(DependencySpec::parse(dep_str)?);
                            }
                        }
                    }

                    // Map format: dependencies: { pkg: version, pkg: {path: ...} }
                    serde_yaml::Value::Mapping(map) => {
                        for (key, value) in map {
                            if let serde_yaml::Value::String(name) = key {
                                deps.push(DependencySpec::from_yaml_value(name.clone(), value)?);
                            }
                        }
                    }

                    _ => {}
                }
            }

            Ok(deps)
        }

        // Fallback to simple parsing for backward compatibility
        Err(_) => {
            let old_deps = parse_horus_yaml_dependencies(path)?;
            let mut deps = Vec::new();
            for dep_str in old_deps {
                deps.push(DependencySpec::parse(&dep_str)?);
            }
            Ok(deps)
        }
    }
}

/// Ignore patterns from horus.yaml
#[derive(Debug, Clone, Default)]
pub struct IgnorePatterns {
    pub files: Vec<String>,
    pub directories: Vec<String>,
    pub packages: Vec<String>,
}

impl IgnorePatterns {
    /// Check if a file path should be ignored
    pub fn should_ignore_file(&self, path: &Path) -> bool {
        let path_str = path.to_string_lossy();

        // Check directory patterns first
        for dir_pattern in &self.directories {
            let pattern = dir_pattern.trim_end_matches('/');
            if path_str.contains(pattern) {
                return true;
            }
        }

        // Check file patterns with glob matching
        for file_pattern in &self.files {
            if glob_match(file_pattern, &path_str) {
                return true;
            }
        }

        false
    }

    /// Check if a package should be ignored
    pub fn should_ignore_package(&self, package: &str) -> bool {
        self.packages.iter().any(|p| p == package)
    }
}

/// Simple glob matching for ignore patterns
fn glob_match(pattern: &str, text: &str) -> bool {
    // Handle ** for directory recursion
    if pattern.contains("**/") {
        let parts: Vec<&str> = pattern.split("**/").collect();
        if parts.len() == 2 {
            let suffix = parts[1];
            return text.contains(suffix) || text.ends_with(suffix);
        }
    }

    // Handle * wildcard
    if pattern.contains('*') {
        let parts: Vec<&str> = pattern.split('*').collect();
        if parts.is_empty() {
            return true;
        }

        let mut pos = 0;
        for (i, part) in parts.iter().enumerate() {
            if part.is_empty() {
                continue;
            }

            if i == 0 && !text.starts_with(part) {
                return false;
            }

            if let Some(found_pos) = text[pos..].find(part) {
                pos += found_pos + part.len();
            } else {
                return false;
            }
        }

        // If pattern ends with *, we're good
        // Otherwise, make sure we matched to the end
        if !pattern.ends_with('*') && pos != text.len() {
            return false;
        }

        true
    } else {
        // Exact match or ends_with for simple patterns
        text == pattern || text.ends_with(pattern)
    }
}

/// Parse ignore section from horus.yaml
pub fn parse_horus_yaml_ignore(path: &str) -> Result<IgnorePatterns> {
    let content = fs::read_to_string(path)?;

    match serde_yaml::from_str::<serde_yaml::Value>(&content) {
        Ok(yaml) => {
            let mut ignore = IgnorePatterns::default();

            if let Some(serde_yaml::Value::Mapping(ignore_map)) = yaml.get("ignore") {
                // Parse files
                if let Some(serde_yaml::Value::Sequence(files)) =
                    ignore_map.get(serde_yaml::Value::String("files".to_string()))
                {
                    for file in files {
                        if let serde_yaml::Value::String(pattern) = file {
                            ignore.files.push(pattern.clone());
                        }
                    }
                }

                // Parse directories
                if let Some(serde_yaml::Value::Sequence(dirs)) =
                    ignore_map.get(serde_yaml::Value::String("directories".to_string()))
                {
                    for dir in dirs {
                        if let serde_yaml::Value::String(pattern) = dir {
                            ignore.directories.push(pattern.clone());
                        }
                    }
                }

                // Parse packages
                if let Some(serde_yaml::Value::Sequence(pkgs)) =
                    ignore_map.get(serde_yaml::Value::String("packages".to_string()))
                {
                    for pkg in pkgs {
                        if let serde_yaml::Value::String(package) = pkg {
                            ignore.packages.push(package.clone());
                        }
                    }
                }
            }

            Ok(ignore)
        }
        Err(_) => Ok(IgnorePatterns::default()),
    }
}

/// Driver configuration from horus.yaml
#[derive(Debug, Clone, Default)]
pub struct DriverConfig {
    /// List of drivers to enable (e.g., ["camera", "lidar", "imu"])
    pub drivers: Vec<String>,
    /// Backend overrides (e.g., {"lidar": "rplidar-a2", "imu": "mpu6050"})
    pub backends: std::collections::HashMap<String, String>,
}

/// Parse drivers section from horus.yaml
///
/// Supports two formats:
/// ```yaml
/// # Simple list format
/// drivers:
///   - camera
///   - lidar
///   - imu
///
/// # Or with backend overrides
/// drivers:
///   camera: opencv
///   lidar: rplidar-a2
///   imu: mpu6050
/// ```
pub fn parse_horus_yaml_drivers(path: &str) -> Result<DriverConfig> {
    let content = fs::read_to_string(path)?;

    match serde_yaml::from_str::<serde_yaml::Value>(&content) {
        Ok(yaml) => {
            let mut config = DriverConfig::default();

            if let Some(drivers_value) = yaml.get("drivers") {
                match drivers_value {
                    // List format: drivers: [camera, lidar, imu]
                    serde_yaml::Value::Sequence(list) => {
                        for item in list {
                            if let serde_yaml::Value::String(driver) = item {
                                // Resolve aliases (e.g., "vision" -> ["camera", "depth-camera"])
                                if let Some(expanded) = resolve_driver_alias(driver) {
                                    for d in expanded {
                                        config.drivers.push(d.to_string());
                                    }
                                } else {
                                    config.drivers.push(driver.clone());
                                }
                            }
                        }
                    }

                    // Map format: drivers: { camera: opencv, lidar: rplidar-a2 }
                    serde_yaml::Value::Mapping(map) => {
                        for (key, value) in map {
                            if let serde_yaml::Value::String(driver_name) = key {
                                // Add to drivers list
                                if let Some(expanded) = resolve_driver_alias(driver_name) {
                                    for d in expanded {
                                        config.drivers.push(d.to_string());
                                    }
                                } else {
                                    config.drivers.push(driver_name.clone());
                                }

                                // Store backend override if specified
                                if let serde_yaml::Value::String(backend) = value {
                                    config.backends.insert(driver_name.clone(), backend.clone());
                                }
                            }
                        }
                    }

                    _ => {}
                }
            }

            Ok(config)
        }
        Err(_) => Ok(DriverConfig::default()),
    }
}

/// Get active drivers - combines CLI override, horus.yaml config, and HORUS_DRIVERS env var
pub fn get_active_drivers() -> DriverConfig {
    // Priority: HORUS_DRIVERS env var > CLI --drivers > horus.yaml
    if let Ok(env_drivers) = std::env::var("HORUS_DRIVERS") {
        let drivers: Vec<String> = env_drivers
            .split(',')
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect();

        if !drivers.is_empty() {
            return DriverConfig {
                drivers,
                backends: std::collections::HashMap::new(),
            };
        }
    }

    // Fall back to horus.yaml
    if std::path::Path::new("horus.yaml").exists() {
        parse_horus_yaml_drivers("horus.yaml").unwrap_or_default()
    } else {
        DriverConfig::default()
    }
}

/// Map a driver name and optional backend to Cargo feature(s)
///
/// This is the central mapping from user-friendly driver names to Cargo features.
/// Users never need to know about features - they just specify drivers.
///
/// # Example mappings:
/// - `imu` + `mpu6050` → `["mpu6050-imu"]`
/// - `imu` + `bno055` → `["bno055-imu"]`
/// - `camera` + `opencv` → `["opencv-backend"]`
/// - `lidar` + `rplidar` → `["rplidar"]`
pub fn driver_to_features(driver: &str, backend: Option<&str>) -> Vec<String> {
    match driver.to_lowercase().as_str() {
        // IMU drivers
        "imu" => match backend.map(|s| s.to_lowercase()).as_deref() {
            Some("mpu6050") => vec!["mpu6050-imu".to_string()],
            Some("bno055") => vec!["bno055-imu".to_string()],
            Some("icm20948") => vec!["icm20948-imu".to_string()],
            _ => vec![], // Simulation - no feature needed
        },

        // Camera drivers
        "camera" => match backend.map(|s| s.to_lowercase()).as_deref() {
            Some("opencv") => vec!["opencv-backend".to_string()],
            Some("v4l2") => vec![],
            Some("realsense") => vec![],
            Some("zed") => vec!["zed".to_string()],
            _ => vec![], // Simulation - no feature needed
        },

        // Depth camera
        "depth-camera" => match backend.map(|s| s.to_lowercase()).as_deref() {
            Some("realsense") => vec![],
            Some("zed") => vec![],
            _ => vec![],
        },

        // LiDAR drivers
        "lidar" => match backend.map(|s| s.to_lowercase()).as_deref() {
            Some("rplidar") | Some("rplidar-a2") | Some("rplidar-a3") => {
                vec!["rplidar".to_string()]
            }
            _ => vec![], // Simulation - no feature needed
        },

        // GPS drivers
        "gps" => match backend.map(|s| s.to_lowercase()).as_deref() {
            Some("nmea") => vec!["nmea-gps".to_string()],
            _ => vec![],
        },

        // Input drivers
        "joystick" => vec!["gilrs".to_string()],
        "keyboard" => vec!["crossterm".to_string()],

        // Modbus
        "modbus" => vec!["modbus-hardware".to_string()],

        // Hardware buses
        "i2c" => vec!["i2c-hardware".to_string()],
        "spi" => vec!["spi-hardware".to_string()],
        "serial" => vec!["serial-hardware".to_string()],

        // Motor drivers
        "dc-motor" | "motor" => vec!["gpio-hardware".to_string()],
        "servo" => vec!["gpio-hardware".to_string()],
        "dynamixel" => vec!["serial-hardware".to_string()],

        // Encoder
        "encoder" => vec!["gpio-hardware".to_string()],

        // GPIO
        "gpio" => vec!["gpio-hardware".to_string()],

        // Unknown driver - no features
        _ => vec![],
    }
}

/// Get Cargo features to enable based on driver configuration
///
/// Reads driver config and returns a list of Cargo features to pass to `cargo build --features`.
/// For known built-in drivers, uses the hardcoded mapping.
/// For unknown drivers, queries the HORUS registry for required features.
pub fn get_cargo_features_from_drivers(config: &DriverConfig) -> Vec<String> {
    let mut features = Vec::new();

    for driver in &config.drivers {
        let backend = config.backends.get(driver).map(|s| s.as_str());
        let driver_features = driver_to_features(driver, backend);

        if driver_features.is_empty() {
            // Try registry lookup for unknown drivers
            if let Some(registry_features) = query_registry_for_driver_features(driver, backend) {
                for f in registry_features {
                    if !features.contains(&f) {
                        features.push(f);
                    }
                }
                continue;
            }
        }

        for f in driver_features {
            if !features.contains(&f) {
                features.push(f);
            }
        }
    }

    features
}

/// Query the HORUS registry for driver features
/// Used as fallback when driver_to_features() returns empty for unknown drivers
fn query_registry_for_driver_features(driver: &str, backend: Option<&str>) -> Option<Vec<String>> {
    use crate::registry::RegistryClient;

    // Construct driver name for lookup
    let driver_name = if let Some(b) = backend {
        format!("{}-{}", driver, b)
    } else {
        driver.to_string()
    };

    // Try to query registry (silent failure - returns None if registry unreachable)
    let client = RegistryClient::new();
    client.query_driver_features(&driver_name)
}

/// Get features string for cargo build command
///
/// Returns empty string if no features needed, or `--features "feat1,feat2"` format.
pub fn get_cargo_features_arg(config: &DriverConfig) -> Option<String> {
    let features = get_cargo_features_from_drivers(config);
    if features.is_empty() {
        None
    } else {
        Some(features.join(","))
    }
}

// ============================================================================
// Enable Capabilities Configuration
// ============================================================================

/// Enable configuration from horus.yaml or CLI --enable flag
#[derive(Debug, Clone, Default)]
pub struct EnableConfig {
    /// List of capabilities to enable (e.g., ["cuda", "editor", "python"])
    pub capabilities: Vec<String>,
}

/// Map a capability name to Cargo feature(s)
///
/// This maps user-friendly capability names to Cargo features.
/// Users specify capabilities like "cuda" or "editor" without knowing the underlying features.
///
/// # Example mappings:
/// - `cuda` → `["cuda"]`
/// - `editor` → `["editor"]`
/// - `python` → `["python"]`
/// - `gpu` → `["cuda"]` (alias)
/// - `headless` → `["headless"]`
pub fn enable_to_features(capability: &str) -> Vec<String> {
    match capability.to_lowercase().as_str() {
        // GPU/CUDA capabilities
        "cuda" | "gpu" => vec!["cuda".to_string()],

        // Rendering/UI capabilities
        "editor" => vec!["editor".to_string()],
        "headless" => vec!["headless".to_string()],
        "visual" => vec!["visual".to_string()],

        // Language bindings
        "python" | "py" => vec!["python".to_string()],

        // Hardware interface features (shortcuts without -hardware suffix)
        "gpio" => vec!["gpio-hardware".to_string()],
        "i2c" => vec!["i2c-hardware".to_string()],
        "spi" => vec!["spi-hardware".to_string()],
        "can" => vec!["can-hardware".to_string()],
        "serial" => vec!["serial-hardware".to_string()],

        // Full hardware feature names (pass through)
        "gpio-hardware" => vec!["gpio-hardware".to_string()],
        "i2c-hardware" => vec!["i2c-hardware".to_string()],
        "spi-hardware" => vec!["spi-hardware".to_string()],
        "serial-hardware" => vec!["serial-hardware".to_string()],
        "motor-hardware" => vec!["motor-hardware".to_string()],
        "all-hardware" => vec!["all-hardware".to_string()],

        // Backend features
        "opencv" | "opencv-backend" => vec!["opencv-backend".to_string()],

        // Performance features
        "io-uring" | "io-uring-net" => vec!["io-uring-net".to_string()],
        "ultra-low-latency" => vec!["ultra-low-latency".to_string()],

        // Bundle aliases
        "full" => vec!["full".to_string()],
        "sim" | "simulation" => vec![], // Simulation mode - no hardware features

        // Pass through unknown capabilities as-is (for advanced users)
        other => vec![other.to_string()],
    }
}

/// Get Cargo features to enable based on enable configuration
pub fn get_cargo_features_from_enable(config: &EnableConfig) -> Vec<String> {
    let mut features = Vec::new();

    for capability in &config.capabilities {
        let cap_features = enable_to_features(capability);
        for f in cap_features {
            if !features.contains(&f) {
                features.push(f);
            }
        }
    }

    features
}

/// Parse enable section from horus.yaml
///
/// Supports list format:
/// ```yaml
/// enable:
///   - cuda
///   - editor
///   - python
/// ```
pub fn parse_horus_yaml_enable(path: &str) -> Result<EnableConfig> {
    let content = fs::read_to_string(path)?;

    match serde_yaml::from_str::<serde_yaml::Value>(&content) {
        Ok(yaml) => {
            let mut config = EnableConfig::default();

            if let Some(serde_yaml::Value::Sequence(list)) = yaml.get("enable") {
                for item in list {
                    if let serde_yaml::Value::String(capability) = item {
                        config.capabilities.push(capability.clone());
                    }
                }
            }

            Ok(config)
        }
        Err(_) => Ok(EnableConfig::default()),
    }
}

/// Get active enable config - combines CLI override, horus.yaml config, and HORUS_ENABLE env var
pub fn get_active_enable() -> EnableConfig {
    // Priority: HORUS_ENABLE env var > horus.yaml
    if let Ok(env_enable) = std::env::var("HORUS_ENABLE") {
        let capabilities: Vec<String> = env_enable
            .split(',')
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect();

        if !capabilities.is_empty() {
            return EnableConfig { capabilities };
        }
    }

    // Fall back to horus.yaml
    if std::path::Path::new("horus.yaml").exists() {
        parse_horus_yaml_enable("horus.yaml").unwrap_or_default()
    } else {
        EnableConfig::default()
    }
}

/// Get combined features string from both drivers and enable config
pub fn get_all_cargo_features() -> Option<String> {
    let driver_config = get_active_drivers();
    let enable_config = get_active_enable();

    let mut all_features = Vec::new();

    // Add driver features
    for f in get_cargo_features_from_drivers(&driver_config) {
        if !all_features.contains(&f) {
            all_features.push(f);
        }
    }

    // Add enable features
    for f in get_cargo_features_from_enable(&enable_config) {
        if !all_features.contains(&f) {
            all_features.push(f);
        }
    }

    if all_features.is_empty() {
        None
    } else {
        Some(all_features.join(","))
    }
}

fn parse_cargo_dependencies(path: &str) -> Result<HashSet<String>> {
    let content = fs::read_to_string(path)?;
    let mut dependencies = HashSet::new();

    // Simple TOML parsing for dependencies section
    let mut in_deps = false;
    for line in content.lines() {
        if line.starts_with("[dependencies]") {
            in_deps = true;
            continue;
        }
        if line.starts_with('[') {
            in_deps = false;
        }

        if in_deps {
            if let Some(eq_pos) = line.find('=') {
                let dep = line[..eq_pos].trim();
                // Check if this is a HORUS package or resolvable package
                if dep.starts_with("horus") || is_horus_package(dep) {
                    dependencies.insert(dep.to_string());
                }
            }
        }
    }

    Ok(dependencies)
}

fn is_horus_package(package: &str) -> bool {
    // Only HORUS packages start with "horus" prefix
    // Everything else will be handled by pip integration
    package.starts_with("horus")
}

fn is_cargo_package(package_name: &str) -> bool {
    use reqwest::blocking::Client;

    // Common CLI tools from crates.io
    let common_cli_tools = [
        "bat",
        "fd-find",
        "ripgrep",
        "exa",
        "tokei",
        "hyperfine",
        "starship",
        "zoxide",
        "delta",
        "dust",
        "procs",
        "bottom",
        "tealdeer",
        "sd",
        "grex",
        "xsv",
        "bandwhich",
    ];

    if common_cli_tools.contains(&package_name) {
        return true;
    }

    // Check crates.io API for less common packages
    let client = Client::new();
    let url = format!("https://crates.io/api/v1/crates/{}", package_name);
    if let Ok(response) = client
        .get(&url)
        .header("User-Agent", "horus-pkg-manager")
        .send()
    {
        return response.status().is_success();
    }

    false
}

/// Separate HORUS packages, pip packages, and cargo packages
///
/// # Arguments
/// * `deps` - Set of dependency strings from horus.yaml
/// * `context_language` - Optional language context ("rust", "python", "cpp") to guide auto-detection
fn split_dependencies_with_context(
    deps: HashSet<String>,
    context_language: Option<&str>,
) -> (Vec<String>, Vec<PipPackage>, Vec<CargoPackage>) {
    let mut horus_packages = Vec::new();
    let mut pip_packages = Vec::new();
    let mut cargo_packages = Vec::new();

    for dep in deps {
        let dep = dep.trim();

        // Check for explicit prefixes
        if dep.starts_with("pip:") {
            let pkg_str = dep.strip_prefix("pip:").unwrap();
            match PipPackage::from_string(pkg_str) {
                Ok(pkg) => pip_packages.push(pkg),
                Err(e) => {
                    eprintln!(
                        "  {} Failed to parse pip dependency '{}': {}",
                        "".yellow(),
                        dep,
                        e
                    );
                    eprintln!("     Syntax: pip:PACKAGE@VERSION or pip:PACKAGE");
                    eprintln!("     Example: pip:numpy@1.24.0");
                }
            }
            continue;
        }

        if dep.starts_with("cargo:") {
            let pkg_str = dep.strip_prefix("cargo:").unwrap();
            match CargoPackage::from_string(pkg_str) {
                Ok(pkg) => cargo_packages.push(pkg),
                Err(e) => {
                    eprintln!(
                        "  {} Failed to parse cargo dependency '{}': {}",
                        "".yellow(),
                        dep,
                        e
                    );
                    eprintln!("     Syntax: cargo:PACKAGE@VERSION:features=FEAT1,FEAT2");
                    eprintln!("     Examples:");
                    eprintln!("       - 'cargo:serde@1.0:features=derive'");
                    eprintln!("       - 'cargo:tokio@1.35:features=full,macros'");
                    eprintln!("       - cargo:rand@0.8");
                }
            }
            continue;
        }

        // Auto-detect: if starts with "horus"  HORUS registry
        if dep.starts_with("horus") {
            // Special handling for horus_py: ALWAYS map to 'horus-robotics' pip package
            // This is because horus_py is not a HORUS registry package - it's the Python bindings
            // installed via pip as 'horus-robotics'. This fixes Issue #25.
            if dep == "horus_py" || dep.starts_with("horus_py@") {
                pip_packages.push(PipPackage {
                    name: "horus-robotics".to_string(),
                    version: None,
                });
                continue;
            }

            // For bare "horus" in Python context, also map to horus-robotics pip package
            if context_language == Some("python") && dep == "horus" {
                pip_packages.push(PipPackage {
                    name: "horus-robotics".to_string(),
                    version: None,
                });
                continue;
            }

            horus_packages.push(dep.to_string());
            continue;
        }

        // Check if it's a known HORUS package using registry
        if is_horus_package(dep) {
            horus_packages.push(dep.to_string());
            continue;
        }

        // For unprefixed dependencies, use language context to determine type
        if let Some(lang) = context_language {
            match lang {
                "rust" => {
                    // Rust context: unprefixed deps are cargo packages
                    if let Ok(pkg) = CargoPackage::from_string(dep) {
                        cargo_packages.push(pkg);
                    }
                }
                "python" => {
                    // Python context: check if it's a known cargo crate first
                    let dep_name = if let Some(at_pos) = dep.find('@') {
                        &dep[..at_pos]
                    } else {
                        dep
                    };

                    // Skip Rust library crates when running Python code
                    if is_cargo_package(dep_name) {
                        // This is a Rust crate - skip it for Python runtime
                        continue;
                    }

                    // Otherwise treat as pip package
                    if let Ok(pkg) = PipPackage::from_string(dep) {
                        pip_packages.push(pkg);
                    }
                }
                _ => {
                    // Unknown context: fall back to old auto-detection
                    let dep_name = if let Some(at_pos) = dep.find('@') {
                        &dep[..at_pos]
                    } else {
                        dep
                    };

                    if is_cargo_package(dep_name) {
                        if let Ok(pkg) = CargoPackage::from_string(dep) {
                            cargo_packages.push(pkg);
                        }
                    } else if let Ok(pkg) = PipPackage::from_string(dep) {
                        pip_packages.push(pkg);
                    }
                }
            }
        } else {
            // No context: use old auto-detection behavior
            let dep_name = if let Some(at_pos) = dep.find('@') {
                &dep[..at_pos]
            } else {
                dep
            };

            if is_cargo_package(dep_name) {
                if let Ok(pkg) = CargoPackage::from_string(dep) {
                    cargo_packages.push(pkg);
                }
            } else if let Ok(pkg) = PipPackage::from_string(dep) {
                pip_packages.push(pkg);
            }
        }
    }

    (horus_packages, pip_packages, cargo_packages)
}

/// Git dependency reference type
#[derive(Debug, Clone)]
enum GitRef {
    Branch(String),
    Tag(String),
    Rev(String),
    Default,
}

/// Git package dependency
#[derive(Debug, Clone)]
struct GitPackage {
    name: String,
    url: String,
    git_ref: GitRef,
}

impl GitPackage {
    /// Parse from string format: git:pkg_name:url[:branch=X|tag=X|rev=X]
    fn from_string(s: &str) -> Option<Self> {
        let s = s.strip_prefix("git:")?;
        let parts: Vec<&str> = s.splitn(3, ':').collect();
        if parts.len() < 2 {
            return None;
        }

        let name = parts[0].to_string();
        let url_and_ref = if parts.len() == 3 {
            format!("{}:{}", parts[1], parts[2])
        } else {
            parts[1].to_string()
        };

        // Parse URL and optional ref
        let (url, git_ref) = if let Some(idx) = url_and_ref.find(":branch=") {
            let (url, rest) = url_and_ref.split_at(idx);
            let branch = rest.strip_prefix(":branch=").unwrap();
            (url.to_string(), GitRef::Branch(branch.to_string()))
        } else if let Some(idx) = url_and_ref.find(":tag=") {
            let (url, rest) = url_and_ref.split_at(idx);
            let tag = rest.strip_prefix(":tag=").unwrap();
            (url.to_string(), GitRef::Tag(tag.to_string()))
        } else if let Some(idx) = url_and_ref.find(":rev=") {
            let (url, rest) = url_and_ref.split_at(idx);
            let rev = rest.strip_prefix(":rev=").unwrap();
            (url.to_string(), GitRef::Rev(rev.to_string()))
        } else {
            (url_and_ref, GitRef::Default)
        };

        Some(GitPackage { name, url, git_ref })
    }

    /// Get the cache directory name for this git package
    fn cache_dir_name(&self) -> String {
        // Create a unique cache directory based on URL and ref
        let url_hash = {
            use std::collections::hash_map::DefaultHasher;
            use std::hash::{Hash, Hasher};
            let mut hasher = DefaultHasher::new();
            self.url.hash(&mut hasher);
            format!("{:x}", hasher.finish())[..8].to_string()
        };

        let ref_suffix = match &self.git_ref {
            GitRef::Branch(b) => format!("_branch_{}", b),
            GitRef::Tag(t) => format!("_tag_{}", t),
            GitRef::Rev(r) => format!("_rev_{}", &r[..8.min(r.len())]),
            GitRef::Default => String::new(),
        };

        format!("git_{}_{}{}", self.name, url_hash, ref_suffix)
    }
}

/// Split dependencies including path and git dependencies
/// Returns: (horus_packages, pip_packages, cargo_packages, path_packages, git_packages)
/// path_packages is Vec<(name, path)>
type SplitDependencies = (
    Vec<String>,
    Vec<PipPackage>,
    Vec<CargoPackage>,
    Vec<(String, String)>,
    Vec<GitPackage>,
);

fn split_dependencies_with_path_context(
    deps: HashSet<String>,
    context_language: Option<&str>,
) -> SplitDependencies {
    let mut horus_packages = Vec::new();
    let mut pip_packages = Vec::new();
    let mut cargo_packages = Vec::new();
    let mut path_packages = Vec::new();
    let mut git_packages = Vec::new();

    for dep in deps {
        let dep = dep.trim();

        // Handle path dependencies: path:pkg_name:path_str
        if dep.starts_with("path:") {
            let parts: Vec<&str> = dep.splitn(3, ':').collect();
            if parts.len() == 3 {
                let pkg_name = parts[1].to_string();
                let pkg_path = parts[2].to_string();
                path_packages.push((pkg_name, pkg_path));
                continue;
            }
        }

        // Handle git dependencies: git:pkg_name:url[:branch=X|tag=X|rev=X]
        if dep.starts_with("git:") {
            if let Some(git_pkg) = GitPackage::from_string(dep) {
                git_packages.push(git_pkg);
                continue;
            } else {
                eprintln!(
                    "  {} Failed to parse git dependency '{}'",
                    "⚠".yellow(),
                    dep
                );
            }
        }

        // Check for explicit prefixes
        if dep.starts_with("pip:") {
            let pkg_str = dep.strip_prefix("pip:").unwrap();
            match PipPackage::from_string(pkg_str) {
                Ok(pkg) => pip_packages.push(pkg),
                Err(e) => {
                    eprintln!(
                        "  {} Failed to parse pip dependency '{}': {}",
                        "".yellow(),
                        dep,
                        e
                    );
                }
            }
            continue;
        }

        if dep.starts_with("cargo:") {
            let pkg_str = dep.strip_prefix("cargo:").unwrap();
            match CargoPackage::from_string(pkg_str) {
                Ok(pkg) => cargo_packages.push(pkg),
                Err(e) => {
                    eprintln!(
                        "  {} Failed to parse cargo dependency '{}': {}",
                        "".yellow(),
                        dep,
                        e
                    );
                }
            }
            continue;
        }

        // Auto-detect: if starts with "horus"  HORUS registry
        if dep.starts_with("horus") {
            // Special handling for horus_py: ALWAYS map to 'horus-robotics' pip package
            // This is because horus_py is not a HORUS registry package - it's the Python bindings
            // installed via pip as 'horus-robotics'. This fixes Issue #25.
            if dep == "horus_py" || dep.starts_with("horus_py@") {
                pip_packages.push(PipPackage {
                    name: "horus-robotics".to_string(),
                    version: None,
                });
                continue;
            }

            // For bare "horus" in Python context, also map to horus-robotics pip package
            if context_language == Some("python") && dep == "horus" {
                pip_packages.push(PipPackage {
                    name: "horus-robotics".to_string(),
                    version: None,
                });
                continue;
            }

            horus_packages.push(dep.to_string());
            continue;
        }

        // Check if it's a known HORUS package using registry
        if is_horus_package(dep) {
            horus_packages.push(dep.to_string());
            continue;
        }

        // For unprefixed dependencies, use language context to determine type
        if let Some(lang) = context_language {
            match lang {
                "rust" => {
                    // Rust context: unprefixed deps are cargo packages
                    if let Ok(pkg) = CargoPackage::from_string(dep) {
                        cargo_packages.push(pkg);
                    }
                }
                "python" => {
                    // Python context: check if it's a known cargo crate first
                    let dep_name = if let Some(at_pos) = dep.find('@') {
                        &dep[..at_pos]
                    } else {
                        dep
                    };

                    // Skip Rust library crates when running Python code
                    if is_cargo_package(dep_name) {
                        // This is a Rust crate - skip it for Python runtime
                        continue;
                    }

                    // Otherwise treat as pip package
                    if let Ok(pkg) = PipPackage::from_string(dep) {
                        pip_packages.push(pkg);
                    }
                }
                _ => {}
            }
        }
    }

    (
        horus_packages,
        pip_packages,
        cargo_packages,
        path_packages,
        git_packages,
    )
}

/// Clone a git dependency to the global cache and return the path
/// Returns: (pkg_name, cached_path)
fn clone_git_dependency(git_pkg: &GitPackage) -> Result<(String, PathBuf)> {
    let global_cache = home_dir().join(".horus/cache");
    let cache_dir_name = git_pkg.cache_dir_name();
    let cache_path = global_cache.join(&cache_dir_name);

    // Check if already cached
    if cache_path.exists() && cache_path.join("Cargo.toml").exists() {
        println!(
            "  {} Git dependency '{}' cached at: {}",
            "✓".green(),
            git_pkg.name,
            cache_path.display()
        );
        return Ok((git_pkg.name.clone(), cache_path));
    }

    // Create cache directory
    fs::create_dir_all(&global_cache)?;

    // Clone the repository
    println!(
        "  {} Cloning git dependency: {} from {}",
        "↓".cyan(),
        git_pkg.name,
        git_pkg.url
    );

    // Remove stale directory if exists
    if cache_path.exists() {
        fs::remove_dir_all(&cache_path)?;
    }

    // Build git clone command
    let mut clone_cmd = Command::new("git");
    clone_cmd.args(["clone", "--depth", "1"]);

    // Add branch/tag/rev options
    match &git_pkg.git_ref {
        GitRef::Branch(branch) => {
            clone_cmd.args(["--branch", branch]);
        }
        GitRef::Tag(tag) => {
            clone_cmd.args(["--branch", tag]);
        }
        GitRef::Rev(_) => {
            // For specific rev, we need full clone (can't use --depth 1)
            clone_cmd.args(["--no-single-branch"]);
        }
        GitRef::Default => {}
    }

    clone_cmd.args([&git_pkg.url, &*cache_path.to_string_lossy()]);

    let output = clone_cmd.output().context("Failed to run git clone")?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(anyhow!("Git clone failed: {}", stderr));
    }

    // Checkout specific rev if specified
    if let GitRef::Rev(rev) = &git_pkg.git_ref {
        let checkout_output = Command::new("git")
            .args(["checkout", rev])
            .current_dir(&cache_path)
            .output()
            .context("Failed to checkout git revision")?;

        if !checkout_output.status.success() {
            let stderr = String::from_utf8_lossy(&checkout_output.stderr);
            return Err(anyhow!("Git checkout failed: {}", stderr));
        }
    }

    // Verify it's a valid Rust crate
    if !cache_path.join("Cargo.toml").exists() {
        return Err(anyhow!(
            "Git dependency '{}' doesn't contain a Cargo.toml file",
            git_pkg.name
        ));
    }

    println!("  {} Cloned git dependency: {}", "✓".green(), git_pkg.name);

    Ok((git_pkg.name.clone(), cache_path))
}

/// Clone all git dependencies and return them as path dependencies
fn resolve_git_dependencies(git_deps: &[GitPackage]) -> Result<Vec<(String, PathBuf)>> {
    let mut resolved = Vec::new();

    for git_pkg in git_deps {
        match clone_git_dependency(git_pkg) {
            Ok((name, path)) => resolved.push((name, path)),
            Err(e) => {
                eprintln!(
                    "  {} Failed to clone git dependency '{}': {}",
                    "✗".red(),
                    git_pkg.name,
                    e
                );
                // Continue with other dependencies
            }
        }
    }

    Ok(resolved)
}

/// Install pip packages using global cache (HORUS philosophy)
/// Packages stored at: ~/.horus/cache/pypi_{name}@{version}/
fn install_pip_packages(packages: Vec<PipPackage>) -> Result<()> {
    if packages.is_empty() {
        return Ok(());
    }

    println!("{} Resolving Python packages...", "[PYTHON]".cyan());

    let global_cache = home_dir().join(".horus/cache");
    let local_packages = PathBuf::from(".horus/packages");

    fs::create_dir_all(&global_cache)?;
    fs::create_dir_all(&local_packages)?;

    // Use system Python's pip directly
    let python_cmd = detect_python_interpreter()?;

    for pkg in &packages {
        // Check if package exists in system first
        if let Ok(Some(system_version)) = detect_system_python_package(&pkg.name) {
            // Auto-use system package for horus-robotics (core dependency)
            if pkg.name == "horus-robotics" {
                create_system_reference_python_run(&pkg.name, &system_version)?;
                continue;
            }

            let local_link = local_packages.join(&pkg.name);

            // Skip if already handled
            if local_link.exists() || local_link.read_link().is_ok() {
                continue;
            }

            // Prompt user for choice
            match prompt_system_package_choice_run(&pkg.name, &system_version)? {
                SystemPackageChoiceRun::UseSystem => {
                    create_system_reference_python_run(&pkg.name, &system_version)?;
                    continue;
                }
                SystemPackageChoiceRun::InstallHORUS => {
                    println!("  {} Installing isolated copy to HORUS...", "".blue());
                    // Continue with installation below
                }
                SystemPackageChoiceRun::Cancel => {
                    println!("  {} Skipped {}", "⊘".yellow(), pkg.name);
                    continue;
                }
            }
        }

        // Get actual version by querying PyPI or using installed version
        let version_str = pkg
            .version
            .as_ref()
            .map(|v| {
                v.replace(">=", "")
                    .replace("==", "")
                    .replace("~=", "")
                    .replace(">", "")
                    .replace("<", "")
            })
            .unwrap_or_else(|| "latest".to_string());

        // Cache directory with pypi_ prefix to distinguish from HORUS packages
        let pkg_cache_dir = global_cache.join(format!("pypi_{}@{}", pkg.name, version_str));

        let local_link = local_packages.join(&pkg.name);

        // If already symlinked, skip
        if local_link.exists() || local_link.read_link().is_ok() {
            println!("  {} {} (already linked)", "".green(), pkg.name);
            continue;
        }

        // If not cached, install to global cache
        if !pkg_cache_dir.exists() {
            println!("  {} Installing {} to global cache...", "".cyan(), pkg.name);

            fs::create_dir_all(&pkg_cache_dir)?;

            // Install package with pip to cache directory using system pip
            let mut cmd = Command::new(&python_cmd);
            cmd.args([
                "-m",
                "pip",
                "install",
                "--target",
                &*pkg_cache_dir.to_string_lossy(),
            ]);
            cmd.arg(pkg.requirement_string());

            let output = cmd.output().context("Failed to run pip install")?;

            if !output.status.success() {
                let stderr = String::from_utf8_lossy(&output.stderr);
                bail!("pip install failed for {}: {}", pkg.name, stderr);
            }

            // Create metadata.json for package tracking
            let metadata = serde_json::json!({
                "name": pkg.name,
                "version": version_str,
                "source": "PyPI"
            });
            let metadata_path = pkg_cache_dir.join("metadata.json");
            fs::write(&metadata_path, serde_json::to_string_pretty(&metadata)?)?;

            println!("  {} Cached {}", "".green(), pkg.name);
        } else {
            println!("  {} {} -> global cache", "↗".cyan(), pkg.name);
        }

        // Symlink from local packages to global cache
        symlink(&pkg_cache_dir, &local_link)
            .context(format!("Failed to symlink {} from global cache", pkg.name))?;
        println!("  {} Linked {}", "".green(), pkg.name);
    }

    Ok(())
}

fn install_cargo_packages(packages: Vec<CargoPackage>) -> Result<()> {
    if packages.is_empty() {
        return Ok(());
    }

    println!("{} Resolving Rust binaries...", "[RUST]".cyan());

    let global_cache = home_dir().join(".horus/cache");
    let local_bin = PathBuf::from(".horus/bin");
    let local_packages = PathBuf::from(".horus/packages");

    fs::create_dir_all(&global_cache)?;
    fs::create_dir_all(&local_bin)?;
    fs::create_dir_all(&local_packages)?;

    // Check if cargo is available
    if Command::new("cargo").arg("--version").output().is_err() {
        bail!("cargo not found. Please install Rust toolchain from https://rustup.rs");
    }

    for pkg in &packages {
        // Check if system binary exists first
        if let Ok(Some(system_version)) = detect_system_cargo_binary(&pkg.name) {
            let local_link = local_bin.join(&pkg.name);

            // Skip if already handled
            if local_link.exists() || local_link.read_link().is_ok() {
                continue;
            }

            // Prompt user for choice
            match prompt_system_cargo_choice_run(&pkg.name, &system_version)? {
                SystemPackageChoiceRun::UseSystem => {
                    create_system_reference_cargo_run(&pkg.name, &system_version)?;
                    continue;
                }
                SystemPackageChoiceRun::InstallHORUS => {
                    println!("  {} Installing isolated copy to HORUS...", "".blue());
                    // Continue with installation below
                }
                SystemPackageChoiceRun::Cancel => {
                    println!("  {} Skipped {}", "⊘".yellow(), pkg.name);
                    continue;
                }
            }
        }

        let version_str = pkg
            .version
            .as_ref()
            .unwrap_or(&"latest".to_string())
            .clone();
        let pkg_cache_dir = global_cache.join(format!("cratesio_{}@{}", pkg.name, version_str));
        let local_link = local_bin.join(&pkg.name);

        // If already linked, skip
        if local_link.exists() || local_link.read_link().is_ok() {
            println!("  {} {} (already linked)", "".green(), pkg.name);
            continue;
        }

        // If not cached, install to global cache
        if !pkg_cache_dir.exists() {
            println!("  {} Installing {} to global cache...", "".cyan(), pkg.name);

            fs::create_dir_all(&pkg_cache_dir)?;

            // Install with cargo to cache directory
            let mut cmd = Command::new("cargo");
            cmd.arg("install");

            if let Some(version) = &pkg.version {
                cmd.arg(format!("{}@{}", pkg.name, version));
            } else {
                cmd.arg(&pkg.name);
            }

            cmd.arg("--root").arg(&pkg_cache_dir);

            let output = cmd.output().context("Failed to run cargo install")?;

            if !output.status.success() {
                let stderr = String::from_utf8_lossy(&output.stderr);
                bail!("cargo install failed for {}: {}", pkg.name, stderr);
            }

            // Create metadata.json for package tracking
            let metadata = serde_json::json!({
                "name": pkg.name,
                "version": version_str,
                "source": "CratesIO"
            });
            let metadata_path = pkg_cache_dir.join("metadata.json");
            fs::write(&metadata_path, serde_json::to_string_pretty(&metadata)?)?;

            println!("  {} Cached {}", "".green(), pkg.name);
        } else {
            println!("  {} {} -> global cache", "↗".cyan(), pkg.name);
        }

        // Symlink binary from cache/bin/ to .horus/bin/
        let cached_bin = pkg_cache_dir.join("bin").join(&pkg.name);
        if cached_bin.exists() {
            symlink(&cached_bin, &local_link)
                .context(format!("Failed to symlink {} from global cache", pkg.name))?;
            println!("  {} Linked {}", "".green(), pkg.name);
        } else {
            println!(
                "  {} Warning: Binary {} not found in cache",
                "[WARNING]".yellow(),
                pkg.name
            );
        }
    }

    Ok(())
}

fn resolve_dependencies(dependencies: HashSet<String>) -> Result<()> {
    resolve_dependencies_with_context(dependencies, None)
}

fn resolve_dependencies_with_context(
    dependencies: HashSet<String>,
    context_language: Option<&str>,
) -> Result<()> {
    // Check version compatibility first
    if let Err(e) = version::check_version_compatibility() {
        eprintln!("\n{}", "Hint:".cyan());
        eprintln!("  If you recently updated HORUS, run ./install.sh to update libraries.");
        return Err(e);
    }

    // Split dependencies into HORUS packages, pip packages, and cargo packages
    let (horus_packages, pip_packages, cargo_packages) =
        split_dependencies_with_context(dependencies, context_language);

    // Resolve HORUS packages (existing logic)
    if !horus_packages.is_empty() {
        resolve_horus_packages(horus_packages.into_iter().collect())?;
    }

    // Resolve pip packages
    if !pip_packages.is_empty() {
        install_pip_packages(pip_packages)?;
    }

    // Resolve cargo packages - skip for Python (library crates can't be installed with cargo install)
    // Cargo library dependencies are handled by Cargo.toml for Rust projects
    if !cargo_packages.is_empty() && context_language != Some("python") {
        install_cargo_packages(cargo_packages)?;
    }

    Ok(())
}

fn resolve_horus_packages(dependencies: HashSet<String>) -> Result<()> {
    let global_cache = home_dir().join(".horus/cache");
    let local_packages = PathBuf::from(".horus/packages");

    // Ensure directories exist
    fs::create_dir_all(&global_cache)?;
    fs::create_dir_all(&local_packages)?;

    // Collect missing packages first
    let mut missing_packages = Vec::new();

    for package in &dependencies {
        let local_link = local_packages.join(package);

        // Skip if already linked
        if local_link.exists() {
            println!("  {} {} (already linked)", "".green(), package);
            continue;
        }

        // Check global cache
        let cached_versions = find_cached_versions(&global_cache, package)?;

        if let Some(cached) = cached_versions.first() {
            // Check if we're using a different version than requested
            let cached_name = cached.file_name().and_then(|n| n.to_str()).unwrap_or("");
            let version_mismatch = package.contains('@') && cached_name != package;

            // Special handling for horus_py - the Python package is named "horus"
            if package.starts_with("horus_py") {
                // Check if lib/horus exists in the cached package
                let lib_horus = cached.join("lib/horus");
                if lib_horus.exists() {
                    // Create symlink named "horus" pointing to lib/horus
                    let horus_link = local_packages.join("horus");

                    // Check if symlink already exists
                    if horus_link.exists() {
                        println!("  {} {} (already linked)", "".green(), package);
                        continue;
                    }

                    if version_mismatch {
                        println!(
                            "  {} {} -> {} (using {})",
                            "↗".cyan(),
                            package,
                            "global cache".dimmed(),
                            cached_name.yellow()
                        );
                    } else {
                        println!(
                            "  {} {} -> {}",
                            "↗".cyan(),
                            package,
                            "global cache".dimmed()
                        );
                    }
                    symlink(&lib_horus, &horus_link).context("Failed to symlink horus_py")?;
                    continue;
                }
            }

            // Create symlink to global cache
            if version_mismatch {
                println!(
                    "  {} {} -> {} (using {})",
                    "↗".cyan(),
                    package,
                    "global cache".dimmed(),
                    cached_name.yellow()
                );
            } else {
                println!(
                    "  {} {} -> {}",
                    "↗".cyan(),
                    package,
                    "global cache".dimmed()
                );
            }
            symlink(cached, &local_link).context(format!("Failed to symlink {}", package))?;
        } else {
            // Package not found locally
            missing_packages.push(package.clone());
        }
    }

    // If there are missing packages, ask user if they want to install
    if !missing_packages.is_empty() {
        println!(
            "\n{} Missing {} package(s):",
            "".yellow(),
            missing_packages.len()
        );
        for pkg in &missing_packages {
            println!("  • {}", pkg.yellow());
        }

        print!(
            "\n{} Install missing packages from registry? [Y/n]: ",
            "?".cyan()
        );
        io::stdout().flush()?;

        let mut input = String::new();
        io::stdin().read_line(&mut input)?;
        let input = input.trim().to_lowercase();

        if input.is_empty() || input == "y" || input == "yes" {
            // User wants to install
            println!("\n{} Installing packages...", "".cyan());

            // Import registry client
            use crate::registry::RegistryClient;
            use crate::workspace;
            let client = RegistryClient::new();
            let target = workspace::detect_or_select_workspace(true)?;

            // Try to use structured dependencies from horus.yaml
            let horus_yaml_path = Path::new("horus.yaml");
            let use_structured_deps = horus_yaml_path.exists();

            // Get base directory for resolving relative paths (directory containing horus.yaml)
            let base_dir = horus_yaml_path.parent().or_else(|| Some(Path::new(".")));

            if use_structured_deps {
                // Parse with v2 to get DependencySpecs with source information
                match parse_horus_yaml_dependencies_v2("horus.yaml") {
                    Ok(dep_specs) => {
                        // Create a map of package name -> DependencySpec
                        let mut spec_map: std::collections::HashMap<String, DependencySpec> =
                            dep_specs
                                .into_iter()
                                .map(|spec| (spec.name.clone(), spec))
                                .collect();

                        for package in &missing_packages {
                            if let Some(spec) = spec_map.remove(package) {
                                print!("  {} Installing {}... ", "".cyan(), package.yellow());
                                io::stdout().flush()?;

                                match client.install_dependency_spec(
                                    &spec,
                                    target.clone(),
                                    base_dir,
                                ) {
                                    Ok(_) => {
                                        println!("{}", "".green());
                                    }
                                    Err(e) => {
                                        println!("{}", "".red());
                                        eprintln!(
                                            "    {} Failed to install {}: {}",
                                            "".red(),
                                            package,
                                            e
                                        );
                                        bail!("Failed to install required dependency: {}", package);
                                    }
                                }
                            } else {
                                // Fallback to registry install if spec not found
                                print!(
                                    "  {} Installing {} (from registry)... ",
                                    "".cyan(),
                                    package.yellow()
                                );
                                io::stdout().flush()?;
                                match client.install(package, None) {
                                    Ok(_) => println!("{}", "".green()),
                                    Err(e) => {
                                        println!("{}", "".red());
                                        bail!("Failed to install {}: {}", package, e);
                                    }
                                }
                            }
                        }
                    }
                    Err(_) => {
                        // Fallback to old parser
                        for package in &missing_packages {
                            print!("  {} Installing {}... ", "".cyan(), package.yellow());
                            io::stdout().flush()?;

                            match client.install(package, None) {
                                Ok(_) => {
                                    println!("{}", "".green());
                                }
                                Err(e) => {
                                    println!("{}", "".red());
                                    eprintln!(
                                        "    {} Failed to install {}: {}",
                                        "".red(),
                                        package,
                                        e
                                    );
                                    bail!("Failed to install required dependency: {}", package);
                                }
                            }
                        }
                    }
                }
            } else {
                // No horus.yaml, use old behavior
                for package in &missing_packages {
                    print!("  {} Installing {}... ", "".cyan(), package.yellow());
                    io::stdout().flush()?;

                    match client.install(package, None) {
                        Ok(_) => {
                            println!("{}", "".green());
                        }
                        Err(e) => {
                            println!("{}", "".red());
                            eprintln!("    {} Failed to install {}: {}", "".red(), package, e);
                            bail!("Failed to install required dependency: {}", package);
                        }
                    }
                }
            }

            println!("\n{} All dependencies installed successfully!", "".green());
        } else {
            // User declined
            println!(
                "\n{} Installation cancelled. Cannot proceed without dependencies.",
                "".red()
            );
            bail!(
                "Missing required dependencies: {}",
                missing_packages.join(", ")
            );
        }
    }

    Ok(())
}

fn find_cached_versions(cache_dir: &Path, package: &str) -> Result<Vec<PathBuf>> {
    let mut versions = Vec::new();

    if !cache_dir.exists() {
        return Ok(versions);
    }

    // Parse package name and version if specified (e.g., "horus_py@0.1.0" -> ("horus_py", Some("0.1.5")))
    let (base_package, requested_version) = if let Some(at_pos) = package.find('@') {
        (&package[..at_pos], Some(&package[at_pos + 1..]))
    } else {
        (package, None)
    };

    for entry in fs::read_dir(cache_dir)? {
        let entry = entry?;
        let name = entry.file_name();
        let name_str = name.to_string_lossy();

        // Match base package name
        if name_str == base_package || name_str.starts_with(&format!("{}@", base_package)) {
            // If a specific version was requested, prefer exact match
            if let Some(req_ver) = requested_version {
                if name_str == format!("{}@{}", base_package, req_ver) {
                    // Exact version match - prioritize this
                    versions.insert(0, entry.path());
                } else {
                    // Different version - add to list as fallback
                    versions.push(entry.path());
                }
            } else {
                // No specific version requested - add all
                versions.push(entry.path());
            }
        }
    }

    // Sort by version (newest first), but keep exact match at front if it exists
    if requested_version.is_some() && !versions.is_empty() {
        // First entry is exact match (if found), don't sort it out
        let exact_match = versions.first().cloned();
        let is_exact = exact_match.as_ref().is_some_and(|p| {
            p.file_name().and_then(|n| n.to_str()).is_some_and(|n| {
                requested_version.is_some_and(|v| n == format!("{}@{}", base_package, v))
            })
        });

        if is_exact {
            // Keep exact match at front, sort the rest
            let mut rest = versions.split_off(1);
            rest.sort_by(|a, b| b.cmp(a));
            versions.extend(rest);
        } else {
            // No exact match, sort all by version (newest first)
            versions.sort_by(|a, b| b.cmp(a));
        }
    } else {
        // Sort by version (newest first)
        versions.sort_by(|a, b| b.cmp(a));
    }

    Ok(versions)
}

fn home_dir() -> PathBuf {
    // Cross-platform home directory detection
    dirs::home_dir().unwrap_or_else(|| {
        // Fallback to temp directory if home not found
        std::env::temp_dir()
    })
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
    let home = home_dir();
    let global_cache = home.join(".horus/cache");
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

fn detect_python_interpreter() -> Result<String> {
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
    let home = dirs::home_dir().context("Could not find home directory")?;
    let global_cache = home.join(".horus/cache");

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
                if Path::new("horus.yaml").exists() {
                    let deps = parse_horus_yaml_dependencies("horus.yaml")?;
                    let (horus_pkgs, _pip_pkgs, cargo_pkgs, path_pkgs, git_pkgs) =
                        split_dependencies_with_path_context(deps, Some("rust"));
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
version = "0.1.8"
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
                if dep_path.exists() && dep_path.join("Cargo.toml").exists() {
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
                let resolved_git_deps = resolve_git_dependencies(&git_deps)?;
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

            if Path::new("horus.yaml").exists() {
                if let Ok(yaml_content) = fs::read_to_string("horus.yaml") {
                    if let Ok(yaml) = serde_yaml::from_str::<serde_yaml::Value>(&yaml_content) {
                        if let Some(serde_yaml::Value::Sequence(list)) = yaml.get("dependencies") {
                            for item in list {
                                if let Some(dep_str) = parse_yaml_cargo_dependency(item) {
                                    // Extract dependency name from the generated string (e.g., "serde = ..." -> "serde")
                                    let dep_name = dep_str.split('=').next().unwrap().trim();

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
                    eprintln!("{} Warning: cargo clean failed", "[!]".yellow());
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
            if let Some(features) = get_all_cargo_features() {
                cmd.arg("--features").arg(&features);
                println!(
                    "  {} Auto-enabling features: {}",
                    "󰢱".cyan(),
                    features.green()
                );
            }

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
    if Path::new("Cargo.toml").exists() {
        let content = fs::read_to_string("Cargo.toml")?;
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
    // Check environment variable first
    if let Ok(horus_source) = env::var("HORUS_SOURCE") {
        let path = PathBuf::from(horus_source);
        if path.exists() && path.join("horus/Cargo.toml").exists() {
            return Ok(path);
        }
    }

    // Check common development locations
    let candidates = vec![
        PathBuf::from("/horus"),
        home_dir().join("softmata/horus"),
        home_dir().join("horus"),
        PathBuf::from("/opt/horus"),
        PathBuf::from("/usr/local/horus"),
    ];

    for candidate in candidates {
        if candidate.exists() && candidate.join("horus/Cargo.toml").exists() {
            return Ok(candidate);
        }
    }

    // Fallback: Check for installed packages in cache
    let cache_dir = home_dir().join(".horus/cache");
    if cache_dir.join("horus@0.1.0").exists() {
        return Ok(cache_dir);
    }

    bail!(
        "HORUS source not found. This can happen after running 'horus clean -a'.\n\n\
         To fix this, either:\n\
         1. Re-run the install script: ./install.sh (or curl the installer)\n\
         2. Set HORUS_SOURCE environment variable to your HORUS source directory\n\
         3. Clone HORUS to ~/softmata/horus or ~/horus"
    )
}

#[derive(Debug, Clone, PartialEq)]
enum SystemPackageChoiceRun {
    UseSystem,
    InstallHORUS,
    Cancel,
}

fn detect_system_cargo_binary(package_name: &str) -> Result<Option<String>> {
    use std::process::Command;

    // Check ~/.cargo/bin/
    if let Some(home) = dirs::home_dir() {
        let cargo_bin = home.join(".cargo/bin").join(package_name);
        if cargo_bin.exists() {
            // Try to get version by running --version
            if let Ok(output) = Command::new(&cargo_bin).arg("--version").output() {
                if output.status.success() {
                    let version_str = String::from_utf8_lossy(&output.stdout);
                    // Parse version (usually "name version")
                    let version = version_str
                        .split_whitespace()
                        .nth(1)
                        .unwrap_or("unknown")
                        .to_string();
                    return Ok(Some(version));
                }
            }
            // Binary exists but version unknown
            return Ok(Some("unknown".to_string()));
        }
    }

    Ok(None)
}

fn prompt_system_cargo_choice_run(
    package_name: &str,
    system_version: &str,
) -> Result<SystemPackageChoiceRun> {
    use std::io::{self, Write};

    println!(
        "\n{} crates.io {} found in system (version: {})",
        "[WARNING]".yellow(),
        package_name.green(),
        system_version.cyan()
    );
    println!("\nWhat would you like to do?");
    println!("  [1] {} Use system binary (create reference)", "".green());
    println!(
        "  [2] {} Install to HORUS (isolated environment)",
        "".blue()
    );
    println!("  [3] {} Skip this package", "⊘".yellow());

    print!("\nChoice [1-3]: ");
    io::stdout().flush()?;

    let mut input = String::new();
    io::stdin().read_line(&mut input)?;

    match input.trim() {
        "1" => Ok(SystemPackageChoiceRun::UseSystem),
        "2" => Ok(SystemPackageChoiceRun::InstallHORUS),
        "3" => Ok(SystemPackageChoiceRun::Cancel),
        _ => {
            println!("Invalid choice, defaulting to Install to HORUS");
            Ok(SystemPackageChoiceRun::InstallHORUS)
        }
    }
}

fn create_system_reference_cargo_run(package_name: &str, system_version: &str) -> Result<()> {
    println!("  {} Creating reference to system binary...", "".green());

    // Find actual system binary location
    let home = dirs::home_dir().ok_or_else(|| anyhow!("Could not find home directory"))?;
    let cargo_bin = home.join(".cargo/bin").join(package_name);

    if !cargo_bin.exists() {
        bail!("System binary not found at expected location");
    }

    let packages_dir = PathBuf::from(".horus/packages");
    fs::create_dir_all(&packages_dir)?;

    let metadata_file = packages_dir.join(format!("{}.system.json", package_name));
    let metadata = serde_json::json!({
        "name": package_name,
        "version": system_version,
        "source": "System",
        "system_path": cargo_bin.display().to_string(),
        "package_type": "CratesIO"
    });

    fs::write(&metadata_file, serde_json::to_string_pretty(&metadata)?)?;

    // Create symlink in .horus/bin to system binary (Unix) or copy on Windows
    let bin_dir = PathBuf::from(".horus/bin");
    fs::create_dir_all(&bin_dir)?;

    let bin_link = bin_dir.join(package_name);
    if bin_link.exists() {
        fs::remove_file(&bin_link)?;
    }

    #[cfg(unix)]
    {
        symlink(&cargo_bin, &bin_link)?;
    }
    #[cfg(windows)]
    {
        // On Windows, create a .cmd wrapper instead of symlink
        let cmd_link = bin_dir.join(format!("{}.cmd", package_name));
        fs::write(&cmd_link, format!("@\"{}\"\r\n", cargo_bin.display()))?;
    }

    println!(
        "  {} Using system binary at {}",
        "".green(),
        cargo_bin.display()
    );
    println!(
        "  {} Reference created: {}",
        "".cyan(),
        metadata_file.display()
    );
    println!("  {} Binary linked: {}", "".cyan(), bin_link.display());

    Ok(())
}

fn detect_system_python_package(package_name: &str) -> Result<Option<String>> {
    use std::process::Command;

    // Check if package is installed in system Python using pip show
    let output = Command::new("python3")
        .args(["-m", "pip", "show", package_name])
        .output();

    if let Ok(output) = output {
        if output.status.success() {
            let stdout = String::from_utf8_lossy(&output.stdout);
            // Parse version from pip show output
            for line in stdout.lines() {
                if line.starts_with("Version:") {
                    let version = line.trim_start_matches("Version:").trim().to_string();
                    return Ok(Some(version));
                }
            }
            // Package found but version unknown
            return Ok(Some("unknown".to_string()));
        }
    }

    Ok(None)
}

fn prompt_system_package_choice_run(
    package_name: &str,
    system_version: &str,
) -> Result<SystemPackageChoiceRun> {
    use std::io::{self, Write};

    println!(
        "\n{} PyPI package {} found in system (version: {})",
        "[WARNING]".yellow(),
        package_name.green(),
        system_version.cyan()
    );
    println!("\nWhat would you like to do?");
    println!("  [1] {} Use system package (create reference)", "".green());
    println!(
        "  [2] {} Install to HORUS (isolated environment)",
        "".blue()
    );
    println!("  [3] {} Skip this package", "⊘".yellow());

    print!("\nChoice [1-3]: ");
    io::stdout().flush()?;

    let mut input = String::new();
    io::stdin().read_line(&mut input)?;

    match input.trim() {
        "1" => Ok(SystemPackageChoiceRun::UseSystem),
        "2" => Ok(SystemPackageChoiceRun::InstallHORUS),
        "3" => Ok(SystemPackageChoiceRun::Cancel),
        _ => {
            println!("Invalid choice, defaulting to Install to HORUS");
            Ok(SystemPackageChoiceRun::InstallHORUS)
        }
    }
}

fn create_system_reference_python_run(package_name: &str, system_version: &str) -> Result<()> {
    println!("  {} Creating reference to system package...", "".green());

    let packages_dir = PathBuf::from(".horus/packages");
    fs::create_dir_all(&packages_dir)?;

    let metadata_file = packages_dir.join(format!("{}.system.json", package_name));
    let metadata = serde_json::json!({
        "name": package_name,
        "version": system_version,
        "source": "System",
        "package_type": "PyPI"
    });

    fs::write(&metadata_file, serde_json::to_string_pretty(&metadata)?)?;

    println!("  {} Using system package", "".green());
    println!(
        "  {} Reference created: {}",
        "".cyan(),
        metadata_file.display()
    );

    Ok(())
}

/// Auto-create or update horus.yaml with detected dependencies
fn auto_update_horus_yaml(
    file: &Path,
    language: &str,
    dependencies: &HashSet<String>,
) -> Result<()> {
    let yaml_path = PathBuf::from("horus.yaml");

    if yaml_path.exists() {
        // Update existing horus.yaml
        update_existing_horus_yaml(&yaml_path, language, dependencies)?;
    } else {
        // Create new horus.yaml
        create_horus_yaml(&yaml_path, file, language, dependencies)?;
    }

    Ok(())
}

/// Create new horus.yaml from scratch
fn create_horus_yaml(
    yaml_path: &Path,
    file: &Path,
    language: &str,
    dependencies: &HashSet<String>,
) -> Result<()> {
    // Derive project name from directory or file
    let project_name = std::env::current_dir()
        .ok()
        .and_then(|p| p.file_name().map(|n| n.to_string_lossy().to_string()))
        .unwrap_or_else(|| {
            file.file_stem()
                .and_then(|n| n.to_str())
                .unwrap_or("my-project")
                .to_string()
        });

    // Categorize dependencies based on language context
    let mut horus_deps = Vec::new();
    let mut pip_deps = Vec::new();
    let mut cargo_deps = Vec::new();

    for dep in dependencies {
        if dep.starts_with("horus") {
            horus_deps.push(dep.clone());
        } else {
            // Default based on language
            match language {
                "python" => pip_deps.push(format!("pip:{}", dep)),
                "rust" => cargo_deps.push(format!("cargo:{}", dep)),
                _ => pip_deps.push(format!("pip:{}", dep)), // Default to pip
            }
        }
    }

    // Sort for consistent output
    horus_deps.sort();
    pip_deps.sort();
    cargo_deps.sort();

    // Build YAML content
    let mut content = String::new();
    content.push_str(&format!("name: {}\n", project_name));
    content.push_str("version: 0.1.8\n");
    content.push_str(&format!("language: {}\n", language));
    content.push_str("\ndependencies:\n");

    // Add HORUS packages first
    for dep in horus_deps {
        content.push_str(&format!("  - {}\n", dep));
    }

    // Add pip packages
    for dep in pip_deps {
        content.push_str(&format!("  - {}\n", dep));
    }

    // Add cargo packages
    for dep in cargo_deps {
        content.push_str(&format!("  - {}\n", dep));
    }

    // Write file
    fs::write(yaml_path, content)?;

    eprintln!(
        "  {} Created horus.yaml with {} dependencies",
        "".green(),
        dependencies.len()
    );

    Ok(())
}

/// Update existing horus.yaml with new dependencies
fn update_existing_horus_yaml(
    yaml_path: &Path,
    language: &str,
    new_dependencies: &HashSet<String>,
) -> Result<()> {
    // Parse existing yaml to get current dependencies
    let existing_content = fs::read_to_string(yaml_path)?;
    let existing_deps = parse_horus_yaml_dependencies_from_content(&existing_content)?;

    // Find new dependencies not in existing
    let mut added = Vec::new();
    for dep in new_dependencies {
        let dep_str = if dep.starts_with("horus") {
            dep.clone()
        } else {
            // Categorize based on language context
            match language {
                "python" => format!("pip:{}", dep),
                "rust" => format!("cargo:{}", dep),
                _ => format!("pip:{}", dep), // Default to pip
            }
        };

        // Check if dependency already exists (in any form)
        let base_name = dep_str
            .split(':')
            .next_back()
            .unwrap_or(&dep_str)
            .split('@')
            .next()
            .unwrap_or(&dep_str);

        let already_exists = existing_deps.iter().any(|e| {
            let e_base = e
                .split(':')
                .next_back()
                .unwrap_or(e)
                .split('@')
                .next()
                .unwrap_or(e);
            e_base == base_name
        });

        if !already_exists {
            added.push(dep_str);
        }
    }

    if added.is_empty() {
        return Ok(()); // No new dependencies
    }

    // Append new dependencies to file
    let mut content = existing_content;
    if !content.ends_with('\n') {
        content.push('\n');
    }

    for dep in &added {
        content.push_str(&format!("  - {}\n", dep));
    }

    fs::write(yaml_path, content)?;

    eprintln!(
        "  {} Added {} new dependencies to horus.yaml",
        "".green(),
        added.len()
    );
    for dep in &added {
        eprintln!("     + {}", dep);
    }

    Ok(())
}

/// Parse dependencies from YAML content string
fn parse_horus_yaml_dependencies_from_content(content: &str) -> Result<HashSet<String>> {
    let mut dependencies = HashSet::new();
    let mut in_deps = false;

    for line in content.lines() {
        if line.trim() == "dependencies:" {
            in_deps = true;
            continue;
        }

        if in_deps {
            if line.starts_with("  -") {
                let dep = line.trim_start_matches("  -").trim();
                if !dep.is_empty() && !dep.starts_with('#') {
                    dependencies.insert(dep.to_string());
                }
            } else if !line.starts_with("  ") && !line.trim().is_empty() {
                // End of dependencies section
                in_deps = false;
            }
        }
    }

    Ok(dependencies)
}

/// Detect hardware nodes being used and check if hardware support is properly configured
pub fn check_hardware_requirements(file_path: &Path, language: &str) -> Result<()> {
    // Only check Rust files for now
    if language != "rust" {
        return Ok(());
    }

    let content = fs::read_to_string(file_path)?;

    // Detect hardware nodes being used (platform-specific device paths)
    #[cfg(target_os = "linux")]
    let hardware_nodes = vec![
        (
            "I2cBusNode",
            "i2c-hardware",
            "/dev/i2c-*",
            "sudo apt install i2c-tools",
        ),
        (
            "SpiBusNode",
            "spi-hardware",
            "/dev/spidev*",
            "sudo raspi-config -> Interface Options -> SPI",
        ),
        (
            "UltrasonicNode",
            "gpio-hardware",
            "/sys/class/gpio",
            "sudo apt install libraspberrypi-dev",
        ),
        (
            "StepperMotorNode",
            "gpio-hardware",
            "/sys/class/gpio",
            "sudo apt install libraspberrypi-dev",
        ),
        (
            "BldcMotorNode",
            "gpio-hardware",
            "/sys/class/gpio",
            "sudo apt install libraspberrypi-dev",
        ),
        (
            "DynamixelNode",
            "serial-hardware",
            "/dev/tty*",
            "Serial port access",
        ),
        (
            "RoboclawMotorNode",
            "serial-hardware",
            "/dev/tty*",
            "Serial port access",
        ),
        (
            "BatteryMonitorNode",
            "i2c-hardware",
            "/dev/i2c-*",
            "sudo apt install i2c-tools",
        ),
    ];

    #[cfg(target_os = "macos")]
    let hardware_nodes = vec![
        (
            "DynamixelNode",
            "serial-hardware",
            "/dev/tty.usb*",
            "Serial port access",
        ),
        (
            "RoboclawMotorNode",
            "serial-hardware",
            "/dev/tty.usb*",
            "Serial port access",
        ),
    ];

    #[cfg(target_os = "windows")]
    let hardware_nodes: Vec<(&str, &str, &str, &str)> = vec![
        (
            "DynamixelNode",
            "serial-hardware",
            "COM*",
            "Serial port access",
        ),
        (
            "RoboclawMotorNode",
            "serial-hardware",
            "COM*",
            "Serial port access",
        ),
    ];

    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    let hardware_nodes: Vec<(&str, &str, &str, &str)> = vec![];

    let mut detected_nodes = Vec::new();
    let mut missing_features = Vec::new();
    let mut missing_devices = Vec::new();

    // Scan for hardware node usage
    for (node_name, feature, device_pattern, install_cmd) in &hardware_nodes {
        if content.contains(node_name) {
            detected_nodes.push((*node_name, *feature, *device_pattern, *install_cmd));
        }
    }

    if detected_nodes.is_empty() {
        return Ok(()); // No hardware nodes detected
    }

    // Check if hardware features are enabled
    let features_enabled = check_cargo_features(file_path, &detected_nodes)?;

    // Check if hardware devices exist
    for (node_name, _feature, device_pattern, _install_cmd) in &detected_nodes {
        if !check_device_exists(device_pattern) {
            missing_devices.push((*node_name, *device_pattern));
        }
    }

    // Collect features that should be enabled
    let mut required_features = HashSet::new();
    for (_node, feature, _, _) in &detected_nodes {
        required_features.insert(*feature);
    }

    for feature in &required_features {
        if !features_enabled.contains(*feature) {
            missing_features.push(*feature);
        }
    }

    // Print warnings if issues detected
    if !missing_features.is_empty() || !missing_devices.is_empty() {
        eprintln!(
            "\n{}",
            "[WARNING] Hardware Configuration Check".yellow().bold()
        );

        if !detected_nodes.is_empty() {
            eprintln!("\n{}", "Detected hardware nodes:".cyan());
            for (node, _, _, _) in &detected_nodes {
                eprintln!("  {} {}", "•".dimmed(), node);
            }
        }

        if !missing_features.is_empty() {
            eprintln!("\n{}", "Missing cargo features:".yellow());
            for feature in &missing_features {
                eprintln!("  {} {}", "•".dimmed(), feature);
            }
            eprintln!("\n{}", "To enable hardware support:".green());
            let features_list = missing_features.join(",");
            eprintln!(
                "  {} cargo build --features=\"{}\"",
                ">".cyan(),
                features_list
            );
            eprintln!("\n{}", "Or add to your Cargo.toml:".green());
            eprintln!(
                "  horus_library = {{ version = \"0.1\", features = [{}] }}",
                missing_features
                    .iter()
                    .map(|f| format!("\"{}\"", f))
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        }

        if !missing_devices.is_empty() {
            eprintln!("\n{}", "Hardware devices not found:".yellow());
            for (node, device) in &missing_devices {
                eprintln!("  {} {} requires {}", "•".dimmed(), node, device);
            }
            eprintln!("\n{}", "System packages may be needed:".green());
            let mut printed_cmds = HashSet::new();
            for (_, _, _, install_cmd) in &detected_nodes {
                if printed_cmds.insert(*install_cmd) {
                    eprintln!("  {} {}", ">".cyan(), install_cmd);
                }
            }
        }

        eprintln!("\n{}", "Note: Nodes will automatically fall back to SIMULATION mode if hardware is unavailable.".dimmed());
        eprintln!();
    }

    Ok(())
}

/// Check if cargo features are enabled for hardware nodes
fn check_cargo_features(
    file_path: &Path,
    detected_nodes: &[(&str, &str, &str, &str)],
) -> Result<HashSet<String>> {
    let mut enabled_features = HashSet::new();

    // Check Cargo.toml if it exists
    let cargo_toml = if let Some(parent) = file_path.parent() {
        parent.join("Cargo.toml")
    } else {
        PathBuf::from("Cargo.toml")
    };

    if cargo_toml.exists() {
        if let Ok(content) = fs::read_to_string(&cargo_toml) {
            // Simple parsing - look for horus_library with features
            if content.contains("features") {
                for (_, feature, _, _) in detected_nodes {
                    if content.contains(feature) {
                        enabled_features.insert(feature.to_string());
                    }
                }
            }
        }
    }

    Ok(enabled_features)
}

/// Check if hardware device exists
fn check_device_exists(pattern: &str) -> bool {
    // Simple check - use glob to see if any devices match the pattern
    if let Ok(paths) = glob(pattern) {
        for path in paths {
            if path.is_ok() {
                return true; // At least one device exists
            }
        }
    }
    false
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
fn load_params_from_project() -> Result<()> {
    let params_locations = [
        PathBuf::from("params.yaml"),
        PathBuf::from("config/params.yaml"),
        PathBuf::from(".horus/config/params.yaml"),
    ];

    // Find the first existing params file
    let params_file = params_locations.iter().find(|p| p.exists());

    if let Some(path) = params_file {
        // Initialize RuntimeParams (will also load from .horus/config/params.yaml if exists)
        let params = RuntimeParams::init().map_err(|e| anyhow!("Failed to init params: {}", e))?;

        // Load from the found file
        params
            .load_from_disk(path)
            .map_err(|e| anyhow!("Failed to load params from {}: {}", path.display(), e))?;

        // Count parameters loaded
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
