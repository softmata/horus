//! Deploy command - Deploy HORUS projects to remote robots
//!
//! Handles cross-compilation, file transfer, and remote execution.

use crate::cli_output;
use crate::config::CARGO_TOML;
use crate::manifest::{detect_languages, Language};
use colored::*;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

/// YAML deploy config structure (~/.horus/deploy.yaml or .horus/deploy.yaml)
#[derive(Debug, serde::Deserialize)]
struct DeployYaml {
    targets: HashMap<String, YamlTarget>,
}

/// A named target entry in the deploy YAML
#[derive(Debug, serde::Deserialize)]
struct YamlTarget {
    host: String,
    arch: Option<String>,
    dir: Option<String>,
    port: Option<u16>,
    identity: Option<String>,
}

/// Result of resolving a target string against the YAML config
struct ResolvedTarget {
    host: String,
    arch: Option<String>,
    dir: Option<String>,
    port: Option<u16>,
    identity: Option<PathBuf>,
}

/// Supported target architectures for robotics platforms
#[derive(Debug, Clone)]
pub enum TargetArch {
    /// ARM64 (Raspberry Pi 4/5, Jetson Nano/Xavier/Orin)
    Aarch64,
    /// ARM 32-bit (Raspberry Pi 3, older boards)
    Armv7,
    /// x86_64 (Intel NUC, standard PCs)
    X86_64,
    /// Current host architecture
    Native,
}

impl TargetArch {
    fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "aarch64" | "arm64" | "jetson" | "pi4" | "pi5" => Some(TargetArch::Aarch64),
            "armv7" | "arm" | "pi3" | "pi2" => Some(TargetArch::Armv7),
            "x86_64" | "x64" | "amd64" | "intel" => Some(TargetArch::X86_64),
            "native" | "host" | "local" => Some(TargetArch::Native),
            _ => None,
        }
    }

    fn rust_target(&self) -> &'static str {
        match self {
            TargetArch::Aarch64 => "aarch64-unknown-linux-gnu",
            TargetArch::Armv7 => "armv7-unknown-linux-gnueabihf",
            TargetArch::X86_64 => "x86_64-unknown-linux-gnu",
            TargetArch::Native => "", // Use default
        }
    }

    fn display_name(&self) -> &'static str {
        match self {
            TargetArch::Aarch64 => "ARM64 (aarch64)",
            TargetArch::Armv7 => "ARM32 (armv7)",
            TargetArch::X86_64 => "x86_64",
            TargetArch::Native => "native",
        }
    }
}

/// Deploy configuration
#[derive(Debug)]
pub struct DeployConfig {
    /// Target host (user@host or just host)
    pub target: String,
    /// Remote directory to deploy to
    pub remote_dir: String,
    /// Target architecture
    pub arch: TargetArch,
    /// Whether to run after deploying
    pub run_after: bool,
    /// Whether to build in release mode
    pub release: bool,
    /// SSH port
    pub port: u16,
    /// SSH identity file
    pub identity: Option<PathBuf>,
    /// Extra rsync excludes
    pub excludes: Vec<String>,
    /// Skip interactive confirmation (for fleet deploy where confirmation is done upfront)
    pub skip_confirm: bool,
}

impl Default for DeployConfig {
    fn default() -> Self {
        Self {
            target: String::new(),
            remote_dir: "~/horus_deploy".to_string(),
            arch: TargetArch::Aarch64,
            run_after: false,
            release: true,
            port: 22,
            identity: None,
            excludes: vec![],
            skip_confirm: false,
        }
    }
}

/// Resolve a target string: either a direct `user@host` or a named target from `.horus/deploy.yaml`.
fn resolve_target(target: &str) -> ResolvedTarget {
    // If target contains '@', treat as direct user@host — no YAML lookup
    if target.contains('@') {
        return ResolvedTarget {
            host: target.to_string(),
            arch: None,
            dir: None,
            port: None,
            identity: None,
        };
    }

    // Try to load .horus/deploy.yaml and look up the named target
    if let Some(yaml) = load_deploy_yaml() {
        if let Some(entry) = yaml.targets.get(target) {
            return ResolvedTarget {
                host: entry.host.clone(),
                arch: entry.arch.clone(),
                dir: entry.dir.clone(),
                port: entry.port,
                identity: entry.identity.as_ref().map(PathBuf::from),
            };
        }
    }

    // Not found in YAML — return as-is (could be a bare hostname)
    ResolvedTarget {
        host: target.to_string(),
        arch: None,
        dir: None,
        port: None,
        identity: None,
    }
}

/// Load and parse `.horus/deploy.yaml` from the current directory.
fn load_deploy_yaml() -> Option<DeployYaml> {
    let config_path = Path::new(".horus/deploy.yaml");
    let content = std::fs::read_to_string(config_path).ok()?;
    serde_yaml::from_str(&content).ok()
}

/// CLI arguments for single-target deploy (internal).
pub struct DeployArgs {
    pub target: String,
    pub remote_dir: Option<String>,
    pub arch: Option<String>,
    pub run_after: bool,
    pub release: bool,
    pub port: u16,
    pub identity: Option<PathBuf>,
    pub dry_run: bool,
}

/// CLI arguments for multi-target deploy.
pub struct DeployMultiArgs {
    pub targets: Vec<String>,
    pub all: bool,
    pub parallel: bool,
    pub remote_dir: Option<String>,
    pub arch: Option<String>,
    pub run_after: bool,
    pub release: bool,
    pub port: u16,
    pub identity: Option<PathBuf>,
    pub dry_run: bool,
}

/// Run deploy to one or more targets.
pub fn run_deploy_multi(args: DeployMultiArgs) -> HorusResult<()> {
    let DeployMultiArgs {
        targets,
        all,
        parallel,
        remote_dir,
        arch,
        run_after,
        release,
        port,
        identity,
        dry_run,
    } = args;

    // Resolve target list
    let target_names: Vec<String> = if all {
        // Load all targets from deploy.yaml
        match load_deploy_yaml() {
            Some(yaml) if !yaml.targets.is_empty() => {
                let mut names: Vec<String> = yaml.targets.keys().cloned().collect();
                names.sort();
                println!(
                    "{} Deploying to {} target(s): {}",
                    cli_output::ICON_INFO.cyan(),
                    names.len(),
                    names.join(", ").green()
                );
                names
            }
            _ => {
                return Err(HorusError::Config(ConfigError::Other(
                    "No targets configured in deploy.yaml. Create one with target definitions."
                        .to_string(),
                )));
            }
        }
    } else if targets.is_empty() {
        return Err(HorusError::Config(ConfigError::Other(
            "No target specified. Use a target name, user@host, --all, or --list.".to_string(),
        )));
    } else {
        targets
    };

    if target_names.len() == 1 {
        // Single target — use existing deploy path
        return run_deploy(DeployArgs {
            target: target_names.into_iter().next().unwrap(),
            remote_dir,
            arch,
            run_after,
            release,
            port,
            identity,
            dry_run,
        });
    }

    // ── Multi-target (fleet) deploy ─────────────────────────────────
    let total = target_names.len();
    println!(
        "\n{} {}",
        "HORUS Fleet Deploy".green().bold(),
        format!("({} targets)", total).dimmed()
    );
    for (i, name) in target_names.iter().enumerate() {
        let resolved = resolve_target(name);
        println!(
            "  {} {} → {}",
            format!("{}.", i + 1).dimmed(),
            name.cyan(),
            resolved.host
        );
    }

    // Step 1: Build once (all targets share the same binary if same arch)
    // Use first target's arch for the build
    let first_resolved = resolve_target(&target_names[0]);
    let build_arch_str = arch.clone().or(first_resolved.arch);
    let build_arch = build_arch_str
        .as_ref()
        .and_then(|a| TargetArch::from_str(a))
        .unwrap_or_else(|| detect_target_arch(&first_resolved.host));

    println!(
        "\n{} Step 1: Building for {} (shared across {} targets)...",
        cli_output::ICON_INFO.cyan(),
        build_arch.display_name().yellow(),
        total
    );

    if !dry_run {
        let build_config = DeployConfig {
            target: first_resolved.host.clone(),
            remote_dir: "~/horus_deploy".to_string(),
            arch: build_arch,
            run_after: false,
            release,
            port,
            identity: identity.clone(),
            excludes: vec![],
            skip_confirm: true,
        };
        build_for_target(&build_config)?;
    }

    // Step 2: Sync + run to each target
    println!(
        "\n{} Step 2: Syncing to {} targets{}...",
        cli_output::ICON_INFO.cyan(),
        total,
        if parallel { " (parallel)" } else { "" }
    );

    // For fleet deploy, skip the interactive confirmation per-target.
    // Ask once for the whole fleet.
    if !dry_run {
        println!(
            "  {} This will sync to {} remote hosts (with --delete)",
            cli_output::ICON_WARN.yellow(),
            total
        );
        print!("  Continue? [y/N] ");
        std::io::Write::flush(&mut std::io::stdout()).ok();
        let mut input = String::new();
        std::io::stdin().read_line(&mut input).ok();
        if !input.trim().eq_ignore_ascii_case("y") {
            println!("  Cancelled.");
            return Ok(());
        }
    }

    let mut failures = 0;

    // Sequential deploy (parallel would need thread-safe output — future work)
    for (i, name) in target_names.iter().enumerate() {
        println!(
            "\n{} [{}/{}] {}",
            "---".cyan(),
            i + 1,
            total,
            name.green().bold()
        );

        let result = run_deploy(DeployArgs {
            target: name.clone(),
            remote_dir: remote_dir.clone(),
            arch: arch.clone(),
            run_after,
            release,
            port,
            identity: identity.clone(),
            dry_run,
        });

        match result {
            Ok(()) => println!(
                "{} [{}/{}] {} done",
                cli_output::ICON_SUCCESS.green(),
                i + 1,
                total,
                name
            ),
            Err(e) => {
                println!(
                    "{} [{}/{}] {} failed: {}",
                    cli_output::ICON_ERROR.red(),
                    i + 1,
                    total,
                    name,
                    e
                );
                failures += 1;
                // Continue to next target — don't abort the fleet
            }
        }
    }

    // Summary
    println!("\n{} Fleet Deploy Summary", "===".cyan());
    println!(
        "  {} {}/{} targets deployed successfully",
        if failures == 0 {
            cli_output::ICON_SUCCESS.green()
        } else {
            cli_output::ICON_WARN.yellow()
        },
        total - failures,
        total
    );

    if failures > 0 {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "{} of {} deployments failed",
            failures, total
        ))));
    }

    Ok(())
}

/// Run the deploy command for a single target.
pub fn run_deploy(args: DeployArgs) -> HorusResult<()> {
    let DeployArgs {
        target,
        remote_dir,
        arch,
        run_after,
        release,
        port,
        identity,
        dry_run,
    } = args;
    // Resolve named target from .horus/deploy.yaml (if applicable)
    let resolved = resolve_target(&target);

    // CLI args win over YAML values. For Option fields, None means "not set by user".
    // For port, 22 is the clap default — treat it as "not explicitly set" so YAML can override.
    let effective_arch_str = arch.or(resolved.arch);
    let effective_host = &resolved.host;
    let effective_dir = remote_dir
        .or(resolved.dir)
        .unwrap_or_else(|| "~/horus_deploy".to_string());
    let effective_port = if port != 22 {
        port
    } else {
        resolved.port.unwrap_or(22)
    };
    let effective_identity = identity.or(resolved.identity);

    // Parse target architecture
    let target_arch = effective_arch_str
        .as_ref()
        .and_then(|a| TargetArch::from_str(a))
        .unwrap_or_else(|| detect_target_arch(effective_host));

    let config = DeployConfig {
        target: effective_host.to_string(),
        remote_dir: effective_dir,
        arch: target_arch,
        run_after,
        release,
        port: effective_port,
        identity: effective_identity,
        excludes: vec![
            "target".to_string(),
            ".git".to_string(),
            "node_modules".to_string(),
            "__pycache__".to_string(),
            "*.pyc".to_string(),
        ],
        skip_confirm: false,
    };

    println!("{}", "HORUS Deploy".green().bold());
    println!();
    println!("  {} {}", "Target:".cyan(), config.target);
    println!("  {} {}", "Remote dir:".cyan(), config.remote_dir);
    println!(
        "  {} {}",
        "Architecture:".cyan(),
        config.arch.display_name()
    );
    println!(
        "  {} {}",
        "Build mode:".cyan(),
        if config.release { "release" } else { "debug" }
    );
    println!("  {} {}", "Run after:".cyan(), config.run_after);
    println!();

    if dry_run {
        println!(
            "{}",
            "[DRY RUN] Would perform the following steps:"
                .yellow()
                .bold()
        );
        println!();
        print_deploy_plan(&config);
        return Ok(());
    }

    // Step 1: Build for target
    println!("{}", "Step 1: Building project...".cyan().bold());
    build_for_target(&config)?;

    // Step 2: Sync files
    println!();
    println!("{}", "Step 2: Syncing files to target...".cyan().bold());
    sync_to_target(&config)?;

    // Step 3: Run if requested
    if config.run_after {
        println!();
        println!("{}", "Step 3: Running on target...".cyan().bold());
        run_on_target(&config)?;
    }

    println!();
    println!("{} Deployment complete!", cli_output::ICON_SUCCESS.green());
    println!();
    println!(
        "  {} ssh {}:{} to access your robot",
        "Tip:".dimmed(),
        config.target,
        config.port
    );

    Ok(())
}

/// Print what would be done in dry-run mode
fn print_deploy_plan(config: &DeployConfig) {
    let target = config.arch.rust_target();
    let mode = if config.release { "--release" } else { "" };

    println!("  1. Build:");
    if target.is_empty() {
        println!("     cargo build {}", mode);
    } else {
        println!("     cargo build {} --target {}", mode, target);
    }

    println!();
    println!("  2. Sync files:");
    println!(
        "     rsync -avz --delete -e 'ssh -p {}' ./ {}:{}",
        config.port, config.target, config.remote_dir
    );

    if config.run_after {
        println!();
        println!("  3. Run on target:");
        println!(
            "     ssh -p {} {} 'cd {} && ./target/{}/horus_project'",
            config.port,
            config.target,
            config.remote_dir,
            if config.release { "release" } else { "debug" }
        );
    }
}

/// Detect target architecture based on hostname hints
fn detect_target_arch(target: &str) -> TargetArch {
    let lower = target.to_lowercase();
    if lower.contains("jetson")
        || lower.contains("nano")
        || lower.contains("xavier")
        || lower.contains("orin")
        || lower.contains("pi4")
        || lower.contains("pi5")
        || lower.contains("raspberry")
    {
        TargetArch::Aarch64
    } else if lower.contains("pi3") || lower.contains("pi2") {
        TargetArch::Armv7
    } else {
        // Default to aarch64 as most modern robot boards use it
        TargetArch::Aarch64
    }
}

/// Detect the primary project language for deploy purposes.
///
/// Uses `detect_languages()` and returns the first detected language.
/// Falls back to `Language::Rust` if nothing is detected.
fn detect_deploy_language() -> Language {
    let cwd = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
    let languages = detect_languages(&cwd);
    languages.into_iter().next().unwrap_or(Language::Rust)
}

/// Build the project for target architecture
fn build_for_target(config: &DeployConfig) -> HorusResult<()> {
    let language = detect_deploy_language();

    match language {
        Language::Rust => build_for_target_rust(config),
        Language::Python => {
            println!(
                "  {} Python project — no build step needed",
                cli_output::ICON_INFO.cyan()
            );
            Ok(())
        }
        Language::Cpp => build_for_target_cpp(config),
        Language::Ros2 => {
            // ROS2 projects use colcon; for deploy, treat like C++ with cmake
            build_for_target_cpp(config)
        }
    }
}

/// Build a Rust project for the target architecture
fn build_for_target_rust(config: &DeployConfig) -> HorusResult<()> {
    let target = config.arch.rust_target();

    // Check if cross-compilation target is installed
    if !target.is_empty() {
        print!(
            "  {} Checking target {}... ",
            cli_output::ICON_INFO.cyan(),
            target
        );
        let check = Command::new("rustup")
            .args(["target", "list", "--installed"])
            .output();

        match check {
            Ok(output) => {
                let installed = String::from_utf8_lossy(&output.stdout);
                if !installed.contains(target) {
                    println!("{}", "not installed".yellow());
                    println!("  {} Installing target...", cli_output::ICON_INFO.cyan());

                    let install = Command::new("rustup")
                        .args(["target", "add", target])
                        .status();

                    if install.map(|s| !s.success()).unwrap_or(true) {
                        return Err(HorusError::Config(ConfigError::Other(format!(
                            "Failed to install target {}. Run: rustup target add {}",
                            target, target
                        ))));
                    }
                    println!("  {} Target installed", cli_output::ICON_SUCCESS.green());
                } else {
                    println!("{}", "OK".green());
                }
            }
            Err(_) => {
                println!("{}", "rustup not found".yellow());
            }
        }
    }

    // Build the project
    let mut cmd = Command::new("cargo");
    cmd.arg("build");

    if config.release {
        cmd.arg("--release");
    }

    if !target.is_empty() {
        cmd.args(["--target", target]);
    }

    print!("  {} Building", cli_output::ICON_INFO.cyan());
    if !target.is_empty() {
        print!(" for {}", config.arch.display_name());
    }
    println!("...");

    cmd.stdout(Stdio::inherit());
    cmd.stderr(Stdio::inherit());

    let status = cmd.status().map_err(|e| {
        HorusError::Config(ConfigError::Other(format!("Failed to run cargo: {}", e)))
    })?;

    if !status.success() {
        return Err(HorusError::Config(ConfigError::Other(
            "Build failed".to_string(),
        )));
    }

    println!("  {} Build complete", cli_output::ICON_SUCCESS.green());
    Ok(())
}

/// Build a C++ project using cmake
fn build_for_target_cpp(config: &DeployConfig) -> HorusResult<()> {
    let build_dir = ".horus/cpp-build";
    let build_type = if config.release { "Release" } else { "Debug" };

    // Ensure the build directory exists (run cmake configure if needed)
    if !Path::new(build_dir).join("CMakeCache.txt").exists() {
        println!(
            "  {} Configuring cmake build...",
            cli_output::ICON_INFO.cyan()
        );

        let mut configure_cmd = Command::new("cmake");
        configure_cmd.args([
            "-S",
            ".",
            "-B",
            build_dir,
            &format!("-DCMAKE_BUILD_TYPE={}", build_type),
        ]);
        configure_cmd.stdout(Stdio::inherit());
        configure_cmd.stderr(Stdio::inherit());

        let status = configure_cmd.status().map_err(|e| {
            HorusError::Config(ConfigError::Other(format!(
                "Failed to run cmake configure: {}",
                e
            )))
        })?;

        if !status.success() {
            return Err(HorusError::Config(ConfigError::Other(
                "cmake configure failed".to_string(),
            )));
        }
    }

    print!("  {} Building C++ project", cli_output::ICON_INFO.cyan());
    println!(" ({})...", build_type);

    let mut cmd = Command::new("cmake");
    cmd.args(["--build", build_dir, "--config", build_type]);

    cmd.stdout(Stdio::inherit());
    cmd.stderr(Stdio::inherit());

    let status = cmd.status().map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Failed to run cmake build: {}",
            e
        )))
    })?;

    if !status.success() {
        return Err(HorusError::Config(ConfigError::Other(
            "C++ build failed".to_string(),
        )));
    }

    println!("  {} Build complete", cli_output::ICON_SUCCESS.green());
    Ok(())
}

/// Sync files to target using rsync
fn sync_to_target(config: &DeployConfig) -> HorusResult<()> {
    // Check if rsync is available
    if Command::new("rsync").arg("--version").output().is_err() {
        return Err(HorusError::Config(ConfigError::Other(
            "rsync not found. Please install rsync.".to_string(),
        )));
    }

    // Safety check: show what directory will be synced and confirm
    if !config.skip_confirm {
        let cwd = std::env::current_dir()
            .map(|p| p.display().to_string())
            .unwrap_or_else(|_| ".".to_string());
        println!(
            "  {} Will sync '{}' to {}:{}/ (with --delete)",
            cli_output::ICON_WARN.yellow(),
            cwd,
            config.target,
            config.remote_dir
        );
        println!(
            "  {} Files on remote not present locally will be DELETED",
            cli_output::ICON_WARN.yellow()
        );
        print!("  Continue? [y/N] ");
        std::io::Write::flush(&mut std::io::stdout()).ok();
        let mut input = String::new();
        std::io::stdin().read_line(&mut input).ok();
        if !input.trim().eq_ignore_ascii_case("y") {
            println!("  Cancelled.");
            return Ok(());
        }
    }

    // Build rsync command
    let mut cmd = Command::new("rsync");
    cmd.args(["-avz", "--delete", "--progress"]);

    // Add excludes
    for exclude in &config.excludes {
        cmd.args(["--exclude", exclude]);
    }

    // SSH options with ConnectTimeout and shell-escaped identity path
    let ssh_cmd = if let Some(ref identity) = config.identity {
        let escaped = identity.display().to_string().replace('\'', "'\\''");
        format!(
            "ssh -p {} -o ConnectTimeout=30 -i '{}'",
            config.port, escaped
        )
    } else {
        format!("ssh -p {} -o ConnectTimeout=30", config.port)
    };
    cmd.args(["-e", &ssh_cmd]);

    // Source and destination
    cmd.arg("./");
    cmd.arg(format!("{}:{}/", config.target, config.remote_dir));

    println!("  {} Syncing files...", cli_output::ICON_INFO.cyan());

    cmd.stdout(Stdio::inherit());
    cmd.stderr(Stdio::inherit());

    let status = cmd.status().map_err(|e| {
        HorusError::Config(ConfigError::Other(format!("Failed to run rsync: {}", e)))
    })?;

    if !status.success() {
        return Err(HorusError::Config(ConfigError::Other(
            "rsync failed".to_string(),
        )));
    }

    println!("  {} Files synced", cli_output::ICON_SUCCESS.green());
    Ok(())
}

/// Run the project on the target
fn run_on_target(config: &DeployConfig) -> HorusResult<()> {
    let language = detect_deploy_language();

    let (run_command, display_name) = match language {
        Language::Rust => {
            let binary_name = find_binary_name().unwrap_or_else(|| "horus_project".to_string());
            let target = config.arch.rust_target();
            let mode = if config.release { "release" } else { "debug" };

            let binary_path = if target.is_empty() {
                format!("./target/{}/{}", mode, binary_name)
            } else {
                format!("./target/{}/{}/{}", target, mode, binary_name)
            };
            (
                format!("'{}'", binary_path.replace('\'', "'\\''")),
                binary_path,
            )
        }
        Language::Python => {
            let entry = find_python_entry().unwrap_or_else(|| "main.py".to_string());
            let display = format!("python3 {}", entry);
            (
                format!("python3 '{}'", entry.replace('\'', "'\\''")),
                display,
            )
        }
        Language::Cpp | Language::Ros2 => {
            let binary_name = find_cpp_binary().unwrap_or_else(|| "horus_project".to_string());
            let binary_path = format!(".horus/cpp-build/{}", binary_name);
            let display = format!("build/{}", binary_name);
            (format!("'{}'", binary_path.replace('\'', "'\\''")), display)
        }
    };

    let escaped_dir = config.remote_dir.replace('\'', "'\\''");
    let remote_cmd = format!("cd '{}' && {}", escaped_dir, run_command);

    // Build SSH command with ConnectTimeout
    let mut cmd = Command::new("ssh");
    cmd.args(["-p", &config.port.to_string()]);
    cmd.args(["-o", "ConnectTimeout=30"]);

    if let Some(ref identity) = config.identity {
        cmd.args(["-i", &identity.to_string_lossy()]);
    }

    // Allocate a TTY for interactive use
    cmd.arg("-t");
    cmd.arg(&config.target);
    cmd.arg(&remote_cmd);

    println!(
        "  {} Running: {}",
        cli_output::ICON_INFO.cyan(),
        display_name
    );
    println!("  {} Press Ctrl+C to stop", cli_output::ICON_HINT.dimmed());
    println!();

    cmd.stdout(Stdio::inherit());
    cmd.stderr(Stdio::inherit());
    cmd.stdin(Stdio::inherit());

    let status = cmd
        .status()
        .map_err(|e| HorusError::Config(ConfigError::Other(format!("Failed to run SSH: {}", e))))?;

    if !status.success() {
        // Don't treat interrupt as error
        let code = status.code().unwrap_or(0);
        if code != 130 && code != 0 {
            // 130 = Ctrl+C
            return Err(HorusError::Config(ConfigError::Other(format!(
                "Remote execution failed with code {}",
                code
            ))));
        }
    }

    Ok(())
}

/// Find the Python entry point for the project.
///
/// Checks horus.toml [scripts] for a "run" or "start" entry, then
/// falls back to common entry point paths (src/main.py, main.py).
fn find_python_entry() -> Option<String> {
    use crate::manifest::{HorusManifest, HORUS_TOML};

    // Try horus.toml [scripts] for a run/start entry
    let manifest_path = Path::new(HORUS_TOML);
    if manifest_path.exists() {
        if let Ok(manifest) = HorusManifest::load_from(manifest_path) {
            // Check scripts for python entry hints
            for key in &["run", "start", "main"] {
                if let Some(script) = manifest.scripts.get(*key) {
                    // Extract the Python file from the script command
                    // e.g., "python3 src/app.py" -> "src/app.py"
                    if let Some(py_file) = extract_python_file(script) {
                        return Some(py_file);
                    }
                }
            }
        }
    }

    // Fall back to common entry points
    for candidate in &["src/main.py", "main.py"] {
        if Path::new(candidate).exists() {
            return Some(candidate.to_string());
        }
    }

    None
}

/// Extract a Python file path from a script command string.
///
/// Handles patterns like `python3 src/app.py` or `python src/main.py --verbose`.
fn extract_python_file(script: &str) -> Option<String> {
    let parts: Vec<&str> = script.split_whitespace().collect();
    for (i, part) in parts.iter().enumerate() {
        if (*part == "python3" || *part == "python") && i + 1 < parts.len() {
            let candidate = parts[i + 1];
            if candidate.ends_with(".py") {
                return Some(candidate.to_string());
            }
        }
    }
    // Maybe the script is just a .py file path
    if let Some(first) = parts.first() {
        if first.ends_with(".py") {
            return Some(first.to_string());
        }
    }
    None
}

/// Find the C++ binary name from CMakeLists.txt or cmake build artifacts.
///
/// Looks for `add_executable(name ...)` in CMakeLists.txt, then checks the
/// cmake build directory for executables.
fn find_cpp_binary() -> Option<String> {
    // Try to extract from CMakeLists.txt
    let cmake_path = Path::new("CMakeLists.txt");
    if cmake_path.exists() {
        if let Ok(content) = std::fs::read_to_string(cmake_path) {
            for line in content.lines() {
                let trimmed = line.trim();
                if let Some(rest) = trimmed.strip_prefix("add_executable(") {
                    // add_executable(my_app src/main.cpp)
                    if let Some(name) = rest.split_whitespace().next() {
                        let name = name.trim_end_matches(')');
                        if !name.is_empty() {
                            return Some(name.to_string());
                        }
                    }
                }
            }
        }
    }

    // Fall back to horus.toml package name
    use crate::manifest::{HorusManifest, HORUS_TOML};
    let manifest_path = Path::new(HORUS_TOML);
    if manifest_path.exists() {
        if let Ok(manifest) = HorusManifest::load_from(manifest_path) {
            if !manifest.package.name.is_empty() {
                return Some(manifest.package.name.replace('-', "_"));
            }
        }
    }

    None
}

/// Find the binary name from horus.toml, falling back to Cargo.toml
fn find_binary_name() -> Option<String> {
    // Try horus.toml first (single source of truth)
    use crate::manifest::{HorusManifest, HORUS_TOML};
    let manifest_path = Path::new(HORUS_TOML);
    if manifest_path.exists() {
        if let Ok(manifest) = HorusManifest::load_from(manifest_path) {
            if !manifest.package.name.is_empty() {
                return Some(manifest.package.name.replace('-', "_"));
            }
        }
    }

    // Fall back to Cargo.toml (legacy or non-horus projects)
    let cargo_toml = Path::new(CARGO_TOML);
    if !cargo_toml.exists() {
        return None;
    }

    let content = std::fs::read_to_string(cargo_toml).ok()?;
    let parsed: toml::Value = content.parse().ok()?;

    // Try [[bin]] name first
    if let Some(bins) = parsed.get("bin").and_then(|b| b.as_array()) {
        if let Some(first_bin) = bins.first() {
            if let Some(name) = first_bin.get("name").and_then(|n| n.as_str()) {
                return Some(name.to_string());
            }
        }
    }

    // Fall back to [package].name
    parsed
        .get("package")
        .and_then(|p| p.get("name"))
        .and_then(|n| n.as_str())
        .map(|s| s.to_string())
}

/// List available deployment targets from config
pub fn list_targets() -> HorusResult<()> {
    println!("{}", "Deployment Targets".green().bold());
    println!();

    // Try to parse .horus/deploy.yaml
    if let Some(yaml) = load_deploy_yaml() {
        if yaml.targets.is_empty() {
            println!(
                "  {} .horus/deploy.yaml exists but has no targets.",
                cli_output::ICON_WARN.yellow()
            );
        } else {
            // Collect and sort target names for stable output
            let mut names: Vec<&String> = yaml.targets.keys().collect();
            names.sort();

            // Print formatted table
            println!(
                "  {:<14} {:<28} {:<10} {:<20} {:<6} {}",
                "NAME".bold(),
                "HOST".bold(),
                "ARCH".bold(),
                "DIR".bold(),
                "PORT".bold(),
                "IDENTITY".bold(),
            );
            println!("  {}", "-".repeat(86));

            for name in names {
                let t = &yaml.targets[name];
                println!(
                    "  {:<14} {:<28} {:<10} {:<20} {:<6} {}",
                    name.cyan(),
                    t.host,
                    t.arch.as_deref().unwrap_or("-"),
                    t.dir.as_deref().unwrap_or("~/horus_deploy"),
                    t.port.map(|p| p.to_string()).unwrap_or_else(|| "22".into()),
                    t.identity.as_deref().unwrap_or("-"),
                );
            }

            println!();
            println!(
                "  {} horus deploy <NAME> to deploy to a named target",
                "Usage:".cyan()
            );
        }
    } else {
        println!("  {}", "No deployment targets configured.".dimmed());
        println!();
        println!(
            "  {} Create .horus/deploy.yaml to save targets:",
            "Tip:".cyan()
        );
        println!();
        println!("    targets:");
        println!("      robot:");
        println!("        host: pi@192.168.1.100");
        println!("        arch: aarch64");
        println!("        dir: ~/my_robot");
        println!("      jetson:");
        println!("        host: nvidia@jetson.local");
        println!("        arch: aarch64");
        println!("        dir: ~/horus_app");
    }

    println!();
    println!("  {}", "Supported architectures:".cyan());
    println!("    aarch64  - Raspberry Pi 4/5, Jetson Nano/Xavier/Orin");
    println!("    armv7    - Raspberry Pi 2/3, older ARM boards");
    println!("    x86_64   - Intel NUC, standard PCs");
    println!("    native   - Same as build host");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── TargetArch parsing ───────────────────────────────────────────────

    #[test]
    fn target_arch_aarch64_aliases() {
        for alias in &["aarch64", "arm64", "jetson", "pi4", "pi5"] {
            let arch = TargetArch::from_str(alias);
            assert!(arch.is_some(), "should parse '{}'", alias);
            assert_eq!(arch.unwrap().rust_target(), "aarch64-unknown-linux-gnu");
        }
    }

    #[test]
    fn target_arch_armv7_aliases() {
        for alias in &["armv7", "arm", "pi3", "pi2"] {
            let arch = TargetArch::from_str(alias);
            assert!(arch.is_some(), "should parse '{}'", alias);
            assert_eq!(arch.unwrap().rust_target(), "armv7-unknown-linux-gnueabihf");
        }
    }

    #[test]
    fn target_arch_x86_64_aliases() {
        for alias in &["x86_64", "x64", "amd64", "intel"] {
            let arch = TargetArch::from_str(alias);
            assert!(arch.is_some(), "should parse '{}'", alias);
            assert_eq!(arch.unwrap().rust_target(), "x86_64-unknown-linux-gnu");
        }
    }

    #[test]
    fn target_arch_native_aliases() {
        for alias in &["native", "host", "local"] {
            let arch = TargetArch::from_str(alias);
            assert!(arch.is_some(), "should parse '{}'", alias);
            assert_eq!(arch.unwrap().rust_target(), "");
        }
    }

    #[test]
    fn target_arch_unknown_returns_none() {
        assert!(TargetArch::from_str("mips").is_none());
        assert!(TargetArch::from_str("riscv").is_none());
        assert!(TargetArch::from_str("").is_none());
    }

    #[test]
    fn target_arch_case_insensitive() {
        assert!(TargetArch::from_str("AARCH64").is_some());
        assert!(TargetArch::from_str("Pi4").is_some());
        assert!(TargetArch::from_str("X86_64").is_some());
    }

    #[test]
    fn target_arch_display_names() {
        assert_eq!(TargetArch::Aarch64.display_name(), "ARM64 (aarch64)");
        assert_eq!(TargetArch::Armv7.display_name(), "ARM32 (armv7)");
        assert_eq!(TargetArch::X86_64.display_name(), "x86_64");
        assert_eq!(TargetArch::Native.display_name(), "native");
    }

    // ── DeployYaml parsing ───────────────────────────────────────────────

    #[test]
    fn deploy_yaml_parse() {
        let yaml = r#"
targets:
  robot:
    host: pi@192.168.1.100
    arch: aarch64
    dir: ~/my_robot
    port: 2222
    identity: ~/.ssh/robot_key
  jetson:
    host: nvidia@jetson.local
"#;
        let config: DeployYaml = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.targets.len(), 2);

        let robot = &config.targets["robot"];
        assert_eq!(robot.host, "pi@192.168.1.100");
        assert_eq!(robot.arch.as_deref(), Some("aarch64"));
        assert_eq!(robot.dir.as_deref(), Some("~/my_robot"));
        assert_eq!(robot.port, Some(2222));
        assert_eq!(robot.identity.as_deref(), Some("~/.ssh/robot_key"));

        let jetson = &config.targets["jetson"];
        assert_eq!(jetson.host, "nvidia@jetson.local");
        assert!(jetson.arch.is_none());
        assert!(jetson.dir.is_none());
        assert!(jetson.port.is_none());
    }

    // ── resolve_target ─────────────────────────────────────────────────

    #[test]
    fn resolve_target_direct_user_at_host() {
        let resolved = resolve_target("pi@192.168.1.100");
        assert_eq!(resolved.host, "pi@192.168.1.100");
        assert!(resolved.arch.is_none());
        assert!(resolved.dir.is_none());
        assert!(resolved.port.is_none());
        assert!(resolved.identity.is_none());
    }

    #[test]
    fn resolve_target_bare_hostname() {
        // Without @ and without YAML, returns as-is
        let resolved = resolve_target("unknown-host-xyz");
        assert_eq!(resolved.host, "unknown-host-xyz");
    }

    // ── detect_target_arch ─────────────────────────────────────────────

    #[test]
    fn detect_arch_jetson_keywords() {
        assert!(matches!(
            detect_target_arch("nvidia@jetson"),
            TargetArch::Aarch64
        ));
        assert!(matches!(
            detect_target_arch("user@nano.local"),
            TargetArch::Aarch64
        ));
        assert!(matches!(
            detect_target_arch("xavier-01"),
            TargetArch::Aarch64
        ));
        assert!(matches!(detect_target_arch("orin-nx"), TargetArch::Aarch64));
    }

    #[test]
    fn detect_arch_raspberry_pi_keywords() {
        assert!(matches!(
            detect_target_arch("pi4-robot"),
            TargetArch::Aarch64
        ));
        assert!(matches!(detect_target_arch("pi5"), TargetArch::Aarch64));
        assert!(matches!(
            detect_target_arch("raspberry-pi"),
            TargetArch::Aarch64
        ));
    }

    #[test]
    fn detect_arch_pi3_is_armv7() {
        assert!(matches!(detect_target_arch("pi3-old"), TargetArch::Armv7));
        assert!(matches!(detect_target_arch("pi2"), TargetArch::Armv7));
    }

    #[test]
    fn detect_arch_unknown_defaults_to_aarch64() {
        // Unknown hostnames default to aarch64 (most modern robot boards)
        assert!(matches!(
            detect_target_arch("some-robot"),
            TargetArch::Aarch64
        ));
    }

    // ── DeployConfig defaults ──────────────────────────────────────────

    #[test]
    fn deploy_config_defaults() {
        let config = DeployConfig::default();
        assert_eq!(config.target, "");
        assert_eq!(config.remote_dir, "~/horus_deploy");
        assert!(matches!(config.arch, TargetArch::Aarch64));
        assert!(!config.run_after);
        assert!(config.release);
        assert_eq!(config.port, 22);
        assert!(config.identity.is_none());
        assert!(config.excludes.is_empty());
    }

    // ── find_binary_name ───────────────────────────────────────────────

    #[test]
    fn find_binary_name_no_cargo_toml() {
        // Test runs from workspace root which is a virtual workspace (no [[bin]]).
        // find_binary_name reads ./Cargo.toml for [[bin]] or [package].name.
        let result = find_binary_name();
        // Workspace root has no [[bin]] target, so expect None.
        // If somehow Some, the name must be a valid non-empty identifier.
        if let Some(ref name) = result {
            assert!(!name.is_empty(), "Binary name must not be empty");
            assert!(
                !name.contains(' '),
                "Binary name must not contain spaces: '{}'",
                name
            );
        }
    }

    // ── Rust target strings ────────────────────────────────────────────

    #[test]
    fn rust_target_strings_are_valid() {
        assert_eq!(
            TargetArch::Aarch64.rust_target(),
            "aarch64-unknown-linux-gnu"
        );
        assert_eq!(
            TargetArch::Armv7.rust_target(),
            "armv7-unknown-linux-gnueabihf"
        );
        assert_eq!(TargetArch::X86_64.rust_target(), "x86_64-unknown-linux-gnu");
        assert_eq!(TargetArch::Native.rust_target(), "");
    }

    // ── DeployYaml edge cases ──────────────────────────────────────────

    #[test]
    fn deploy_yaml_empty_targets() {
        let yaml = "targets: {}";
        let config: DeployYaml = serde_yaml::from_str(yaml).unwrap();
        assert!(config.targets.is_empty());
    }

    #[test]
    fn deploy_yaml_minimal_target() {
        let yaml = r#"
targets:
  dev:
    host: localhost
"#;
        let config: DeployYaml = serde_yaml::from_str(yaml).unwrap();
        let dev = &config.targets["dev"];
        assert_eq!(dev.host, "localhost");
        assert!(dev.arch.is_none());
        assert!(dev.dir.is_none());
        assert!(dev.port.is_none());
        assert!(dev.identity.is_none());
    }

    // ── extract_python_file ────────────────────────────────────────────

    #[test]
    fn extract_python_file_standard() {
        assert_eq!(
            extract_python_file("python3 src/app.py"),
            Some("src/app.py".to_string())
        );
    }

    #[test]
    fn extract_python_file_python2_syntax() {
        assert_eq!(
            extract_python_file("python main.py"),
            Some("main.py".to_string())
        );
    }

    #[test]
    fn extract_python_file_with_flags_after() {
        assert_eq!(
            extract_python_file("python3 main.py --verbose --port 8080"),
            Some("main.py".to_string())
        );
    }

    #[test]
    fn extract_python_file_bare_py_file() {
        assert_eq!(extract_python_file("app.py"), Some("app.py".to_string()));
    }

    #[test]
    fn extract_python_file_non_python_returns_none() {
        assert_eq!(extract_python_file("node index.js"), None);
    }

    #[test]
    fn extract_python_file_empty_string() {
        assert_eq!(extract_python_file(""), None);
    }

    #[test]
    fn extract_python_file_python3_without_file() {
        // "python3" alone without a .py file after it
        assert_eq!(extract_python_file("python3 --version"), None);
    }

    #[test]
    fn extract_python_file_nested_path() {
        assert_eq!(
            extract_python_file("python3 src/nodes/sensor.py"),
            Some("src/nodes/sensor.py".to_string())
        );
    }

    // ── find_binary_name (with tempdir) ────────────────────────────────

    #[test]
    fn find_binary_name_from_horus_toml() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"my-robot\"\nversion = \"0.1.0\"\nedition = \"1\"\n",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_binary_name();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        // horus.toml name with hyphens → underscores for binary
        assert_eq!(result, Some("my_robot".to_string()));
    }

    #[test]
    fn find_binary_name_from_cargo_toml_bin() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cargo = r#"
[[bin]]
name = "sensor_node"
path = "src/main.rs"

[package]
name = "my-project"
version = "0.1.0"
"#;
        std::fs::write(tmp.path().join("Cargo.toml"), cargo).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_binary_name();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        // [[bin]].name takes precedence
        assert_eq!(result, Some("sensor_node".to_string()));
    }

    #[test]
    fn find_binary_name_empty_dir_returns_none() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_binary_name();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_none());
    }

    // ── find_python_entry (with tempdir) ───────────────────────────────

    #[test]
    fn find_python_entry_from_src_main_py() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::create_dir_all(tmp.path().join("src")).unwrap();
        std::fs::write(tmp.path().join("src/main.py"), "pass").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_python_entry();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert_eq!(result, Some("src/main.py".to_string()));
    }

    #[test]
    fn find_python_entry_from_root_main_py() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(tmp.path().join("main.py"), "pass").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_python_entry();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert_eq!(result, Some("main.py".to_string()));
    }

    #[test]
    fn find_python_entry_empty_dir_returns_none() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_python_entry();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_none());
    }

    #[test]
    fn find_python_entry_from_horus_toml_scripts() {
        let tmp = tempfile::TempDir::new().unwrap();
        let toml = r#"
[package]
name = "bot"
version = "0.1.0"
edition = "1"

[scripts]
run = "python3 src/robot.py"
"#;
        std::fs::write(tmp.path().join("horus.toml"), toml).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_python_entry();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert_eq!(result, Some("src/robot.py".to_string()));
    }

    // ── find_cpp_binary (with tempdir) ─────────────────────────────────

    #[test]
    fn find_cpp_binary_from_cmake() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("CMakeLists.txt"),
            "add_executable(my_robot src/main.cpp)\n",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_cpp_binary();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert_eq!(result, Some("my_robot".to_string()));
    }

    #[test]
    fn find_cpp_binary_from_horus_toml_fallback() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"cpp-bot\"\nversion = \"0.1.0\"\nedition = \"1\"\n",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_cpp_binary();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert_eq!(result, Some("cpp_bot".to_string()));
    }

    #[test]
    fn find_cpp_binary_empty_dir_returns_none() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = find_cpp_binary();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_none());
    }

    // ── DeployConfig command construction ──────────────────────────────

    #[test]
    fn deploy_config_excludes_default_patterns() {
        let config = DeployConfig {
            excludes: vec![
                "target".to_string(),
                ".git".to_string(),
                "node_modules".to_string(),
                "__pycache__".to_string(),
                "*.pyc".to_string(),
            ],
            ..Default::default()
        };
        assert!(config.excludes.contains(&"target".to_string()));
        assert!(config.excludes.contains(&".git".to_string()));
        assert!(config.excludes.contains(&"__pycache__".to_string()));
        assert_eq!(config.excludes.len(), 5);
    }

    #[test]
    fn deploy_config_with_identity() {
        let config = DeployConfig {
            target: "pi@10.0.0.1".to_string(),
            identity: Some(PathBuf::from("/home/user/.ssh/robot_key")),
            ..Default::default()
        };
        assert_eq!(
            config.identity.unwrap(),
            PathBuf::from("/home/user/.ssh/robot_key")
        );
    }

    #[test]
    fn deploy_config_custom_port() {
        let config = DeployConfig {
            target: "pi@10.0.0.1".to_string(),
            port: 2222,
            ..Default::default()
        };
        assert_eq!(config.port, 2222);
    }

    #[test]
    fn deploy_config_debug_mode() {
        let config = DeployConfig {
            release: false,
            ..Default::default()
        };
        assert!(!config.release);
    }

    #[test]
    fn resolve_target_with_at_sign_bypasses_yaml() {
        // Any target containing '@' is treated as direct user@host
        let resolved = resolve_target("admin@10.0.0.5");
        assert_eq!(resolved.host, "admin@10.0.0.5");
        // No YAML fields should be populated
        assert!(resolved.arch.is_none());
        assert!(resolved.dir.is_none());
        assert!(resolved.port.is_none());
        assert!(resolved.identity.is_none());
    }

    // ── Fleet deployment (load_deploy_yaml + multi-target) ─────────────

    #[test]
    fn load_deploy_yaml_with_valid_file() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::create_dir_all(tmp.path().join(".horus")).unwrap();
        std::fs::write(
            tmp.path().join(".horus/deploy.yaml"),
            "targets:\n  robot:\n    host: pi@10.0.0.1\n    arch: aarch64\n  jetson:\n    host: nvidia@10.0.0.2\n",
        ).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = load_deploy_yaml();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_some());
        let yaml = result.unwrap();
        assert_eq!(yaml.targets.len(), 2);
        assert!(yaml.targets.contains_key("robot"));
        assert!(yaml.targets.contains_key("jetson"));
    }

    #[test]
    fn load_deploy_yaml_missing_file_returns_none() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = load_deploy_yaml();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_none());
    }

    #[test]
    fn load_deploy_yaml_invalid_yaml_returns_none() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::create_dir_all(tmp.path().join(".horus")).unwrap();
        std::fs::write(
            tmp.path().join(".horus/deploy.yaml"),
            "{{{{invalid yaml content",
        )
        .unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = load_deploy_yaml();
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_none());
    }

    #[test]
    fn deploy_multi_args_empty_targets_no_all() {
        let args = DeployMultiArgs {
            targets: vec![],
            all: false,
            parallel: false,
            remote_dir: None,
            arch: None,
            run_after: false,
            release: true,
            port: 22,
            identity: None,
            dry_run: false,
        };
        let result = run_deploy_multi(args);
        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("No target"),
            "Expected 'No target' error, got: {err}"
        );
    }

    #[test]
    fn deploy_multi_args_all_without_yaml_errors() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let args = DeployMultiArgs {
            targets: vec![],
            all: true,
            parallel: false,
            remote_dir: None,
            arch: None,
            run_after: false,
            release: true,
            port: 22,
            identity: None,
            dry_run: false,
        };
        let result = run_deploy_multi(args);
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("No targets configured") || err.contains("deploy.yaml"),
            "Expected deploy.yaml error, got: {err}"
        );
    }

    #[test]
    fn deploy_yaml_target_resolution_merges_fields() {
        let yaml_str = r#"
targets:
  robot:
    host: pi@10.0.0.1
    arch: aarch64
    dir: ~/app
    port: 2222
    identity: ~/.ssh/id_robot
"#;
        let yaml: DeployYaml = serde_yaml::from_str(yaml_str).unwrap();
        let target = &yaml.targets["robot"];
        assert_eq!(target.host, "pi@10.0.0.1");
        assert_eq!(target.arch.as_deref(), Some("aarch64"));
        assert_eq!(target.dir.as_deref(), Some("~/app"));
        assert_eq!(target.port, Some(2222));
        assert_eq!(target.identity.as_deref(), Some("~/.ssh/id_robot"));
    }

    #[test]
    fn deploy_yaml_multiple_targets_sorted() {
        let yaml_str = r#"
targets:
  zebra:
    host: z@z.local
  alpha:
    host: a@a.local
  middle:
    host: m@m.local
"#;
        let yaml: DeployYaml = serde_yaml::from_str(yaml_str).unwrap();
        let mut names: Vec<String> = yaml.targets.keys().cloned().collect();
        names.sort();
        assert_eq!(names, vec!["alpha", "middle", "zebra"]);
    }

    #[test]
    fn deploy_yaml_target_minimal_only_host() {
        let yaml_str = r#"
targets:
  dev:
    host: user@localhost
"#;
        let yaml: DeployYaml = serde_yaml::from_str(yaml_str).unwrap();
        let dev = &yaml.targets["dev"];
        assert_eq!(dev.host, "user@localhost");
        assert!(dev.arch.is_none());
        assert!(dev.dir.is_none());
        assert!(dev.port.is_none());
        assert!(dev.identity.is_none());
    }

    #[test]
    fn deploy_args_dry_run_flag() {
        let args = DeployArgs {
            target: "pi@10.0.0.1".to_string(),
            remote_dir: None,
            arch: None,
            run_after: false,
            release: true,
            port: 22,
            identity: None,
            dry_run: true,
        };
        // dry_run should print plan without executing
        // We can't easily capture stdout, but verify the struct is constructible
        assert!(args.dry_run);
        assert_eq!(args.target, "pi@10.0.0.1");
    }

    #[test]
    fn deploy_multi_args_parallel_flag() {
        let args = DeployMultiArgs {
            targets: vec!["robot1".to_string(), "robot2".to_string()],
            all: false,
            parallel: true,
            remote_dir: None,
            arch: None,
            run_after: false,
            release: true,
            port: 22,
            identity: None,
            dry_run: false,
        };
        assert!(args.parallel);
        assert_eq!(args.targets.len(), 2);
    }

    // ── Intent tests: deploy target parsing ────────────────────────────────

    /// INTENT: Parse a deploy target string "user@192.168.1.100:/opt/robot".
    /// The user, host, and path components must be extracted correctly from
    /// the YAML config format.
    #[test]
    fn test_deploy_target_parsing() {
        let yaml_str = r#"
targets:
  robot:
    host: user@192.168.1.100
    dir: /opt/robot
"#;
        let yaml: DeployYaml = serde_yaml::from_str(yaml_str).unwrap();
        let target = &yaml.targets["robot"];

        // The host field carries user@host
        assert_eq!(target.host, "user@192.168.1.100");
        assert!(
            target.host.contains('@'),
            "Host must contain '@' separator for user@host format"
        );

        // Extract user and host parts
        let parts: Vec<&str> = target.host.splitn(2, '@').collect();
        assert_eq!(parts.len(), 2, "user@host must split into exactly 2 parts");
        assert_eq!(parts[0], "user", "User must be 'user'");
        assert_eq!(parts[1], "192.168.1.100", "Host must be '192.168.1.100'");

        // dir field carries the remote path
        assert_eq!(
            target.dir.as_deref(),
            Some("/opt/robot"),
            "Remote path must be '/opt/robot'"
        );
    }

    /// INTENT: Parse a deploy target with a custom port.
    /// Port must be correctly extracted from the YAML config.
    #[test]
    fn test_deploy_target_with_port() {
        let yaml_str = r#"
targets:
  robot:
    host: user@host
    port: 2222
    dir: /path
"#;
        let yaml: DeployYaml = serde_yaml::from_str(yaml_str).unwrap();
        let target = &yaml.targets["robot"];

        assert_eq!(target.port, Some(2222), "Port must be 2222");
        assert_eq!(target.host, "user@host");
        assert_eq!(target.dir.as_deref(), Some("/path"));
    }

    /// INTENT: Target architecture strings ("aarch64", "armv7", "x86_64")
    /// must all be recognized as valid architectures by TargetArch::from_str.
    /// Unrecognized strings must return None.
    #[test]
    fn test_deploy_arch_detection() {
        // All three primary robot architectures must be recognized
        let aarch64 = TargetArch::from_str("aarch64");
        assert!(
            aarch64.is_some(),
            "'aarch64' must be recognized as a valid architecture"
        );
        assert_eq!(
            aarch64.unwrap().rust_target(),
            "aarch64-unknown-linux-gnu",
            "aarch64 must map to the correct Rust target triple"
        );

        let armv7 = TargetArch::from_str("armv7");
        assert!(
            armv7.is_some(),
            "'armv7' must be recognized as a valid architecture"
        );
        assert_eq!(
            armv7.unwrap().rust_target(),
            "armv7-unknown-linux-gnueabihf",
            "armv7 must map to the correct Rust target triple"
        );

        let x86_64 = TargetArch::from_str("x86_64");
        assert!(
            x86_64.is_some(),
            "'x86_64' must be recognized as a valid architecture"
        );
        assert_eq!(
            x86_64.unwrap().rust_target(),
            "x86_64-unknown-linux-gnu",
            "x86_64 must map to the correct Rust target triple"
        );

        // Unknown architectures must return None (not panic or default)
        let unknown = TargetArch::from_str("riscv64");
        assert!(
            unknown.is_none(),
            "Unrecognized architecture 'riscv64' must return None"
        );

        let empty = TargetArch::from_str("");
        assert!(
            empty.is_none(),
            "Empty string must return None for architecture"
        );
    }
}
