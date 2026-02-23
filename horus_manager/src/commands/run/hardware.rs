use crate::config::CARGO_TOML;
use anyhow::anyhow;
use anyhow::Result;
use colored::*;
use glob::glob;
use horus_core::params::RuntimeParams;
use std::collections::HashSet;
use std::fs;
use std::path::{Path, PathBuf};

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
pub(crate) fn check_cargo_features(
    file_path: &Path,
    detected_nodes: &[(&str, &str, &str, &str)],
) -> Result<HashSet<String>> {
    let mut enabled_features = HashSet::new();

    // Check Cargo.toml if it exists
    let cargo_toml = if let Some(parent) = file_path.parent() {
        parent.join(CARGO_TOML)
    } else {
        PathBuf::from(CARGO_TOML)
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
pub(crate) fn check_device_exists(pattern: &str) -> bool {
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
pub(crate) fn load_params_from_project() -> Result<()> {
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
