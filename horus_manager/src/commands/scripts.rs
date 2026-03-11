//! Run scripts defined in `horus.toml [scripts]`.
//!
//! Scripts are user-defined shell commands that can be run via `horus scripts <name>`.
//! They are defined in the `[scripts]` section of `horus.toml`:
//!
//! ```toml
//! [scripts]
//! sim = "horus sim start --world warehouse"
//! deploy-pi = "horus deploy robot@192.168.1.5 --release"
//! test-hw = "cargo test --features hardware"
//! ```

use crate::cli_output;
use crate::manifest::{HorusManifest, HORUS_TOML};
use horus_core::error::{ConfigError, HorusError, HorusResult};
use colored::*;
use std::path::Path;
use std::process::Command;

/// Run a named script or list all available scripts.
pub fn run_scripts(name: Option<String>, args: Vec<String>) -> HorusResult<()> {
    let manifest_path = Path::new(HORUS_TOML);
    if !manifest_path.exists() {
        return Err(HorusError::Config(ConfigError::Other(
            "No horus.toml found. Run `horus new` to create a project.".to_string(),
        )));
    }

    let manifest = HorusManifest::load_from(manifest_path)
        .map(|(m, _)| m)
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    match name {
        None => {
            // List all scripts
            if manifest.scripts.is_empty() {
                println!(
                    "{} No scripts defined in horus.toml",
                    cli_output::ICON_INFO.cyan()
                );
                println!(
                    "\n  Add scripts to horus.toml:\n\n  {}\n  {} = \"horus sim start --world warehouse\"\n  {} = \"cargo test --features hardware\"",
                    "[scripts]".cyan(),
                    "sim".green(),
                    "test-hw".green(),
                );
            } else {
                println!(
                    "{} Available scripts ({}):\n",
                    cli_output::ICON_INFO.cyan(),
                    manifest.scripts.len()
                );
                let max_name_len = manifest.scripts.keys().map(|k| k.len()).max().unwrap_or(0);
                for (script_name, command) in &manifest.scripts {
                    println!(
                        "  {}  {}",
                        format!("{:width$}", script_name, width = max_name_len).green(),
                        command.dimmed()
                    );
                }
                println!(
                    "\n  Run with: {}",
                    "horus scripts <name>".cyan()
                );
            }
            Ok(())
        }
        Some(script_name) => {
            // Run the named script
            let script_cmd = manifest.scripts.get(&script_name).ok_or_else(|| {
                let available = manifest
                    .scripts
                    .keys()
                    .map(|k| k.as_str())
                    .collect::<Vec<_>>()
                    .join(", ");
                HorusError::Config(ConfigError::Other(format!(
                    "Script '{}' not found. Available scripts: {}",
                    script_name,
                    if available.is_empty() {
                        "(none)".to_string()
                    } else {
                        available
                    }
                )))
            })?;

            // Append extra args to the command
            let full_cmd = if args.is_empty() {
                script_cmd.clone()
            } else {
                format!("{} {}", script_cmd, args.join(" "))
            };

            println!(
                "{} Running script {}: {}",
                cli_output::ICON_INFO.cyan(),
                script_name.green(),
                full_cmd.dimmed()
            );

            let status = Command::new("sh")
                .arg("-c")
                .arg(&full_cmd)
                .status()
                .map_err(|e| {
                    HorusError::Config(ConfigError::Other(format!(
                        "Failed to execute script: {}",
                        e
                    )))
                })?;

            if !status.success() {
                return Err(HorusError::Config(ConfigError::Other(format!(
                    "Script '{}' exited with code {}",
                    script_name,
                    status.code().unwrap_or(-1)
                ))));
            }

            Ok(())
        }
    }
}
