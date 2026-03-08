//! Deploy command - Deploy HORUS projects to remote robots
//!
//! Handles cross-compilation, file transfer, and remote execution.

use crate::cli_output;
use crate::config::CARGO_TOML;
use colored::*;
use horus_core::error::{HorusError, HorusResult};
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

/// Run the deploy command
#[allow(clippy::too_many_arguments)]
pub fn run_deploy(
    target: &str,
    remote_dir: Option<String>,
    arch: Option<String>,
    run_after: bool,
    release: bool,
    port: u16,
    identity: Option<PathBuf>,
    dry_run: bool,
) -> HorusResult<()> {
    // Resolve named target from .horus/deploy.yaml (if applicable)
    let resolved = resolve_target(target);

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

/// Build the project for target architecture
fn build_for_target(config: &DeployConfig) -> HorusResult<()> {
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
                        return Err(HorusError::Config(format!(
                            "Failed to install target {}. Run: rustup target add {}",
                            target, target
                        )));
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

    let status = cmd
        .status()
        .map_err(|e| HorusError::Config(format!("Failed to run cargo: {}", e)))?;

    if !status.success() {
        return Err(HorusError::Config("Build failed".to_string()));
    }

    println!("  {} Build complete", cli_output::ICON_SUCCESS.green());
    Ok(())
}

/// Sync files to target using rsync
fn sync_to_target(config: &DeployConfig) -> HorusResult<()> {
    // Check if rsync is available
    if Command::new("rsync").arg("--version").output().is_err() {
        return Err(HorusError::Config(
            "rsync not found. Please install rsync.".to_string(),
        ));
    }

    // Safety check: show what directory will be synced and confirm
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

    // Build rsync command
    let mut cmd = Command::new("rsync");
    cmd.args(["-avz", "--delete", "--progress"]);

    // Add excludes
    for exclude in &config.excludes {
        cmd.args(["--exclude", exclude]);
    }

    // SSH options with ConnectTimeout and properly quoted identity path
    let ssh_cmd = if let Some(ref identity) = config.identity {
        format!(
            "ssh -p {} -o ConnectTimeout=30 -i '{}'",
            config.port,
            identity.display()
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

    let status = cmd
        .status()
        .map_err(|e| HorusError::Config(format!("Failed to run rsync: {}", e)))?;

    if !status.success() {
        return Err(HorusError::Config("rsync failed".to_string()));
    }

    println!("  {} Files synced", cli_output::ICON_SUCCESS.green());
    Ok(())
}

/// Run the project on the target
fn run_on_target(config: &DeployConfig) -> HorusResult<()> {
    // Find the binary name from Cargo.toml
    let binary_name = find_binary_name().unwrap_or_else(|| "horus_project".to_string());

    let target = config.arch.rust_target();
    let mode = if config.release { "release" } else { "debug" };

    // Build the path to the binary
    let binary_path = if target.is_empty() {
        format!("./target/{}/{}", mode, binary_name)
    } else {
        format!("./target/{}/{}/{}", target, mode, binary_name)
    };

    let remote_cmd = format!("cd {} && {}", config.remote_dir, binary_path);

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
        binary_path
    );
    println!("  {} Press Ctrl+C to stop", cli_output::ICON_HINT.dimmed());
    println!();

    cmd.stdout(Stdio::inherit());
    cmd.stderr(Stdio::inherit());
    cmd.stdin(Stdio::inherit());

    let status = cmd
        .status()
        .map_err(|e| HorusError::Config(format!("Failed to run SSH: {}", e)))?;

    if !status.success() {
        // Don't treat interrupt as error
        let code = status.code().unwrap_or(0);
        if code != 130 && code != 0 {
            // 130 = Ctrl+C
            return Err(HorusError::Config(format!(
                "Remote execution failed with code {}",
                code
            )));
        }
    }

    Ok(())
}

/// Find the binary name from Cargo.toml
fn find_binary_name() -> Option<String> {
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
}
