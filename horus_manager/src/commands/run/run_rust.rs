use crate::cli_output;
use crate::config::CARGO_TOML;
use crate::progress::{self, finish_error, finish_success};
use anyhow::{anyhow, bail, Result};
use colored::*;
use std::collections::HashSet;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use super::{deps, features, install};

pub(super) struct ExecutableInfo {
    pub(super) name: String,
    pub(super) command: String,
    pub(super) args_override: Vec<String>,
    /// Extra env vars to pass to the child process via Command::env().
    pub(super) env_vars: Vec<(String, String)>,
}

impl ExecutableInfo {
    pub(super) fn create_command(&self, user_args: &[String]) -> Command {
        let mut cmd = Command::new(&self.command);

        // Use override args if provided, otherwise use user args
        if !self.args_override.is_empty() {
            cmd.args(&self.args_override);
        } else {
            cmd.args(user_args);
        }

        // Apply extra env vars
        cmd.envs(self.env_vars.iter().cloned());

        cmd
    }
}

pub(super) fn get_color_for_index(index: usize) -> &'static str {
    let colors = ["cyan", "green", "yellow", "magenta", "blue", "red"];
    colors[index % colors.len()]
}

pub fn execute_build_only(files: Vec<PathBuf>, release: bool, clean: bool) -> Result<()> {
    // Handle clean build
    if clean {
        println!("{} Cleaning build cache...", cli_output::ICON_INFO.cyan());
        clean_build_cache()?;
    }

    let mode = if release { "release" } else { "debug" };
    println!(
        "{} Building project in {} mode (no execution)...",
        cli_output::ICON_INFO.cyan(),
        mode.yellow()
    );

    // Resolve target file(s)
    let target_files: Vec<PathBuf> = if files.is_empty() {
        vec![super::auto_detect_main_file()?]
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
        cli_output::ICON_INFO.cyan(),
        target_file.display().to_string().green(),
        language.yellow()
    );

    // Ensure .horus directory exists
    super::ensure_horus_directory()?;

    // Build based on language
    match language.as_str() {
        "python" => {
            println!("{} Python is interpreted, no build needed", "[i]".blue());
            println!(
                "  {} File is ready to run: {}",
                cli_output::ICON_INFO.cyan(),
                target_file.display()
            );
        }
        "rust" => {
            // If a root Cargo.toml exists, build directly from the project root
            // with CARGO_TARGET_DIR=.horus/target. Dependencies live in Cargo.toml.
            if Path::new(CARGO_TOML).exists() {
                println!(
                    "{} Building from root Cargo.toml...",
                    cli_output::ICON_INFO.cyan()
                );

                let spinner = progress::robot_build_spinner("Building with cargo...");
                let mut cmd = Command::new("cargo");
                cmd.arg("build");
                cmd.env("CARGO_TARGET_DIR", ".horus/target");
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
                        "\u{f0cb1}".cyan(),
                        features.green()
                    );
                }

                let output = cmd.output()?;
                if !output.status.success() {
                    finish_error(&spinner, "Cargo build failed");
                    let stderr = String::from_utf8_lossy(&output.stderr);
                    if !stderr.is_empty() {
                        eprintln!("{}", stderr);
                    }
                    bail!("Cargo build failed");
                }

                let profile = if release { "release" } else { "debug" };
                let project_name = get_project_name()?;
                let binary_path = format!(".horus/target/{}/{}", profile, project_name);

                finish_success(&spinner, &format!("Built: {}", binary_path));
            } else {
                // No root Cargo.toml — generate a minimal .horus/Cargo.toml
                // for standalone .rs file compilation.
                println!(
                    "{} Setting up Cargo workspace for standalone file...",
                    cli_output::ICON_INFO.cyan()
                );

                let cargo_toml_path = PathBuf::from(".horus/Cargo.toml");
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
                        cli_output::ICON_INFO.cyan(),
                        auto_features.join(", ").yellow()
                    );

                    use crate::system_deps;
                    let dep_result = system_deps::check_dependencies(&auto_features);
                    let report = system_deps::format_dependency_report(&dep_result, &auto_features);
                    if !report.is_empty() {
                        eprintln!("{}", report);
                    }
                }

                // Find HORUS source directory and add core path deps
                let horus_source = find_horus_source_dir()?;
                println!(
                    "  {} Using HORUS source: {}",
                    cli_output::ICON_INFO.cyan(),
                    horus_source.display()
                );

                // Add default HORUS core path dependencies
                for dep_name in &["horus", "horus_core", "horus_library", "horus_macros"] {
                    let dep_path = horus_source.join(dep_name);
                    if dep_path.exists() && dep_path.join(CARGO_TOML).exists() {
                        if (*dep_name == "horus" || *dep_name == "horus_library")
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
                        } else {
                            cargo_toml.push_str(&format!(
                                "{} = {{ path = \"{}\" }}\n",
                                dep_name,
                                dep_path.display()
                            ));
                        }
                        println!(
                            "  {} Added dependency: {} -> {}",
                            cli_output::ICON_INFO.cyan(),
                            dep_name,
                            dep_path.display()
                        );
                    }
                }

                fs::write(&cargo_toml_path, &cargo_toml)?;
                println!(
                    "  {} Generated Cargo.toml (no source copying needed)",
                    cli_output::ICON_SUCCESS.green()
                );

                // Run cargo build in .horus directory
                let spinner = progress::robot_build_spinner("Building with cargo...");
                let mut cmd = Command::new("cargo");
                cmd.arg("build");
                cmd.current_dir(".horus");
                cmd.stdout(std::process::Stdio::piped());
                cmd.stderr(std::process::Stdio::piped());

                if release {
                    cmd.arg("--release");
                }

                if let Some(features) = features::get_all_cargo_features() {
                    cmd.arg("--features").arg(&features);
                    eprintln!(
                        "  {} Auto-enabling features: {}",
                        "\u{f0cb1}".cyan(),
                        features.green()
                    );
                }

                let output = cmd.output()?;
                if !output.status.success() {
                    finish_error(&spinner, "Cargo build failed");
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
        }
        _ => bail!("Unsupported language: {}", language),
    }

    Ok(())
}

pub(super) fn execute_from_cargo_toml(
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
        super::ensure_horus_directory()?;

        // Parse Cargo.toml for HORUS dependencies
        println!(
            "{} Scanning Cargo.toml dependencies...",
            cli_output::ICON_INFO.cyan()
        );
        let horus_deps = deps::parse_cargo_dependencies(CARGO_TOML)?;

        if !horus_deps.is_empty() {
            println!(
                "{} Found {} HORUS dependencies",
                cli_output::ICON_INFO.cyan(),
                horus_deps.len()
            );
            install::resolve_dependencies(horus_deps)?;
        }

        // Build environment for child processes (no env::set_var)
        let child_env = super::build_child_env()?;

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
            cmd.envs(child_env.iter().cloned());
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
        println!(
            "{} Executing Cargo project...\n",
            cli_output::ICON_INFO.cyan()
        );
        let mut cmd = Command::new(binary);
        cmd.args(args);
        cmd.envs(child_env.iter().cloned());
        let status = cmd.status()?;
        if !status.success() {
            bail!("Execution failed");
        }

        Ok(())
    })();

    env::set_current_dir(original_dir)?;
    result
}

/// Build multiple Rust files in a single Cargo workspace for optimal performance
pub(super) fn build_rust_files_batch(
    file_paths: Vec<PathBuf>,
    release: bool,
    clean: bool,
) -> Result<Vec<ExecutableInfo>> {
    if file_paths.is_empty() {
        return Ok(Vec::new());
    }

    // Ensure .horus directory exists
    super::ensure_horus_directory()?;

    // Load ignore patterns from horus.toml
    let ignore = super::load_ignore_patterns();

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
    // This fixes Issue #26: users shouldn't need to explicitly add horus_core to horus.toml
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

    // Dependencies now live in native build files (Cargo.toml) — no horus.toml bridge needed.

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
            "\u{f0cb1}".cyan(),
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

    let child_env = super::build_child_env()?;
    for name in binary_names {
        let binary_path = format!(".horus/target/{}/{}", profile, name);
        executables.push(ExecutableInfo {
            name,
            command: binary_path,
            args_override: Vec::new(),
            env_vars: child_env.clone(),
        });
    }

    Ok(executables)
}

pub(super) fn build_file_for_concurrent_execution(
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
    super::ensure_horus_directory()?;

    // Load ignore patterns from horus.toml
    let ignore = super::load_ignore_patterns();

    // Scan imports and resolve dependencies
    let dependencies = deps::scan_imports(&file_path, &language, &ignore)?;
    if !dependencies.is_empty() {
        // Resolve dependencies with language context
        install::resolve_dependencies_with_context(dependencies, Some(&language))?;
    }

    // Build environment for child processes (no env::set_var)
    let child_env = super::build_child_env()?;

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
            // This fixes Issue #26: users shouldn't need to explicitly add horus_core to horus.toml
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

            // Dependencies now live in native build files (Cargo.toml) — no horus.toml bridge needed.

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
                env_vars: child_env,
            })
        }
        "python" => {
            // Python doesn't need building, just setup interpreter
            let python_cmd = super::run_python::detect_python_interpreter()?;
            let python_path = super::run_python::build_python_path()?;

            let mut env = child_env;
            env.push(("PYTHONPATH".to_string(), python_path));

            Ok(ExecutableInfo {
                name,
                command: python_cmd,
                args_override: vec![file_path.to_string_lossy().to_string()],
                env_vars: env,
            })
        }
        _ => bail!("Unsupported language: {}", language),
    }
}

pub(super) fn execute_with_scheduler(
    file: PathBuf,
    language: String,
    args: Vec<String>,
    release: bool,
    clean: bool,
    child_env: &[(String, String)],
) -> Result<()> {
    match language.as_str() {
        "rust" => {
            // If a root Cargo.toml exists, build directly from the project root
            // with CARGO_TARGET_DIR=.horus/target. Dependencies live in Cargo.toml.
            if Path::new(CARGO_TOML).exists() {
                println!(
                    "{} Building from root Cargo.toml...",
                    cli_output::ICON_INFO.cyan()
                );

                // Run cargo clean if requested
                if clean {
                    println!(
                        "{} Cleaning build artifacts...",
                        cli_output::ICON_INFO.cyan()
                    );
                    let mut clean_cmd = Command::new("cargo");
                    clean_cmd.arg("clean");
                    clean_cmd.env("CARGO_TARGET_DIR", ".horus/target");
                    clean_cmd.envs(child_env.iter().cloned());
                    let status = clean_cmd.status()?;
                    if !status.success() {
                        log::warn!("cargo clean failed");
                    }
                }

                println!("{} Building with Cargo...", cli_output::ICON_INFO.cyan());
                let mut cmd = Command::new("cargo");
                cmd.arg("build");
                cmd.env("CARGO_TARGET_DIR", ".horus/target");
                cmd.envs(child_env.iter().cloned());
                if release {
                    cmd.arg("--release");
                }

                if let Some(features) = features::get_all_cargo_features() {
                    cmd.arg("--features").arg(&features);
                    println!(
                        "  {} Auto-enabling features: {}",
                        "\u{f0cb1}".cyan(),
                        features.green()
                    );
                }

                log::debug!("cargo build command: {:?}", cmd);

                let status = cmd.status()?;
                if !status.success() {
                    bail!("Cargo build failed");
                }

                let profile = if release { "release" } else { "debug" };
                let project_name = get_project_name()?;
                let binary_path = format!(".horus/target/{}/{}", profile, project_name);

                // Execute the binary
                println!("{} Executing...\n", cli_output::ICON_INFO.cyan());
                let mut cmd = Command::new(binary_path);
                cmd.args(args);
                cmd.envs(child_env.iter().cloned());

                let status = cmd.status()?;

                if !status.success() {
                    bail!("Process exited with code {}", status.code().unwrap_or(1));
                }
            } else {
                // No root Cargo.toml — generate a minimal .horus/Cargo.toml
                // for standalone .rs file compilation.
                println!(
                    "{} Setting up Cargo workspace for standalone file...",
                    cli_output::ICON_INFO.cyan()
                );

                let cargo_toml_path = PathBuf::from(".horus/Cargo.toml");
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
                let auto_features =
                    node_detector::detect_features_from_file(&file).unwrap_or_default();
                if !auto_features.is_empty() {
                    eprintln!(
                        "  {} Auto-detected hardware nodes (features: {})",
                        cli_output::ICON_INFO.cyan(),
                        auto_features.join(", ").yellow()
                    );

                    use crate::system_deps;
                    let dep_result = system_deps::check_dependencies(&auto_features);
                    let report = system_deps::format_dependency_report(&dep_result, &auto_features);
                    if !report.is_empty() {
                        eprintln!("{}", report);
                    }
                }

                // Find HORUS source directory and add core path deps
                let horus_source = find_horus_source_dir()?;
                println!(
                    "  {} Using HORUS source: {}",
                    cli_output::ICON_INFO.cyan(),
                    horus_source.display()
                );

                for dep_name in &["horus", "horus_core", "horus_library", "horus_macros"] {
                    let dep_path = horus_source.join(dep_name);
                    if dep_path.exists() && dep_path.join(CARGO_TOML).exists() {
                        if (*dep_name == "horus" || *dep_name == "horus_library")
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
                        } else {
                            cargo_toml.push_str(&format!(
                                "{} = {{ path = \"{}\" }}\n",
                                dep_name,
                                dep_path.display()
                            ));
                        }
                        println!(
                            "  {} Added dependency: {} -> {}",
                            cli_output::ICON_INFO.cyan(),
                            dep_name,
                            dep_path.display()
                        );
                    }
                }

                fs::write(&cargo_toml_path, &cargo_toml)?;
                println!(
                    "  {} Generated Cargo.toml",
                    cli_output::ICON_SUCCESS.green()
                );

                // Run cargo clean if requested
                if clean {
                    println!(
                        "{} Cleaning build artifacts...",
                        cli_output::ICON_INFO.cyan()
                    );
                    let mut clean_cmd = Command::new("cargo");
                    clean_cmd.arg("clean");
                    clean_cmd.current_dir(".horus");
                    let status = clean_cmd.status()?;
                    if !status.success() {
                        log::warn!("cargo clean failed");
                    }
                }

                println!("{} Building with Cargo...", cli_output::ICON_INFO.cyan());
                let mut cmd = Command::new("cargo");
                cmd.arg("build");
                cmd.current_dir(".horus");
                cmd.envs(child_env.iter().cloned());
                if release {
                    cmd.arg("--release");
                }

                if let Some(features) = features::get_all_cargo_features() {
                    cmd.arg("--features").arg(&features);
                    println!(
                        "  {} Auto-enabling features: {}",
                        "\u{f0cb1}".cyan(),
                        features.green()
                    );
                }

                log::debug!("cargo build command: {:?}", cmd);

                let status = cmd.status()?;
                if !status.success() {
                    bail!("Cargo build failed");
                }

                let binary_path = if release {
                    ".horus/target/release/horus-project"
                } else {
                    ".horus/target/debug/horus-project"
                };

                // Execute the binary
                println!("{} Executing...\n", cli_output::ICON_INFO.cyan());
                let mut cmd = Command::new(binary_path);
                cmd.args(args);
                cmd.envs(child_env.iter().cloned());

                let status = cmd.status()?;

                if !status.success() {
                    bail!("Process exited with code {}", status.code().unwrap_or(1));
                }
            }
        }
        "python" => {
            super::run_python::execute_python_node(file, args, release)?;
        }
        _ => bail!(
            "Unsupported language: {}. HORUS supports Rust and Python only.",
            language
        ),
    }

    Ok(())
}

pub(super) fn get_project_name() -> Result<String> {
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

pub(super) fn clean_build_cache() -> Result<()> {
    // Clean .horus/cache directory (where compiled binaries are stored)
    let cache_dir = PathBuf::from(".horus/cache");
    if cache_dir.exists() {
        for entry in fs::read_dir(&cache_dir)? {
            let entry = entry?;
            fs::remove_file(entry.path()).ok();
        }
        println!(
            "  {} Cleaned .horus/cache/",
            cli_output::ICON_SUCCESS.green()
        );
    }

    // Clean .horus/bin directory
    let bin_dir = PathBuf::from(".horus/bin");
    if bin_dir.exists() {
        for entry in fs::read_dir(&bin_dir)? {
            let entry = entry?;
            fs::remove_file(entry.path()).ok();
        }
        println!("  {} Cleaned .horus/bin/", cli_output::ICON_SUCCESS.green());
    }

    // Clean Rust target directory if exists
    let target_dir = PathBuf::from("target");
    if target_dir.exists() {
        fs::remove_dir_all(&target_dir)?;
        println!("  {} Cleaned target/", cli_output::ICON_SUCCESS.green());
    }

    // Clean Python __pycache__ in current directory
    let pycache = PathBuf::from("__pycache__");
    if pycache.exists() {
        fs::remove_dir_all(&pycache)?;
        println!(
            "  {} Cleaned __pycache__/",
            cli_output::ICON_SUCCESS.green()
        );
    }

    Ok(())
}

/// Find the HORUS source directory by checking common locations
pub(crate) fn find_horus_source_dir() -> Result<PathBuf> {
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
    let cache_dir =
        crate::paths::cache_dir().unwrap_or_else(|_| install::home_dir().join(".horus/cache"));
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
