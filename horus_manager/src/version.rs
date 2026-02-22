use anyhow::{bail, Result};
use colored::*;
use std::fs;
use std::path::{Path, PathBuf};

/// Get the CLI version from Cargo.toml at compile time
pub fn get_cli_version() -> &'static str {
    env!("CARGO_PKG_VERSION")
}

/// Get the installed library version from ~/.horus/installed_version
pub fn get_installed_version() -> Result<Option<String>> {
    let version_file = get_version_file_path();

    if !version_file.exists() {
        return Ok(None);
    }

    let version = fs::read_to_string(&version_file)?.trim().to_string();

    Ok(Some(version))
}

/// Get the path to the version tracking file
fn get_version_file_path() -> PathBuf {
    let home = dirs::home_dir().expect("Could not find home directory");
    home.join(".horus/installed_version")
}

/// Check if the CLI version matches the installed library version
pub fn check_version_compatibility() -> Result<()> {
    let cli_version = get_cli_version();

    match get_installed_version()? {
        Some(installed_version) => {
            if cli_version != installed_version {
                print_version_mismatch(cli_version, &installed_version);
                bail!("Version mismatch detected");
            }
            Ok(())
        }
        None => {
            // No version file found - libraries might not be installed
            Ok(())
        }
    }
}

/// Check version and prompt user if mismatch detected
pub fn check_and_prompt_update() -> Result<()> {
    let cli_version = get_cli_version();

    if let Some(installed_version) = get_installed_version()? {
        if cli_version != installed_version {
            print_version_mismatch(cli_version, &installed_version);

            // Find HORUS source directory
            if let Some(horus_root) = find_horus_source() {
                println!("\n{} To update libraries, run:", "".cyan());
                println!(
                    "  {}",
                    format!("cd {} && ./install.sh", horus_root.display()).cyan()
                );
            } else {
                println!("\n{} To update libraries:", "".cyan());
                println!("  1. Navigate to your HORUS source directory");
                println!("  2. Run: {}", "./install.sh".cyan());
            }

            bail!("Library version mismatch");
        }
    }

    Ok(())
}

/// Print version mismatch warning
fn print_version_mismatch(cli_version: &str, installed_version: &str) {
    eprintln!();
    eprintln!(
        "{} {}",
        "".yellow().bold(),
        "Version mismatch detected!".yellow().bold()
    );
    eprintln!();
    eprintln!("  CLI version:       {}", cli_version.green());
    eprintln!("  Installed libraries: {}", installed_version.red());
    eprintln!();
    eprintln!(
        "{} The CLI and libraries must be the same version.",
        "Note:".cyan()
    );
    eprintln!("  This ensures API compatibility between your code and the runtime.");
}

/// Find the HORUS source directory by looking for install.sh
fn find_horus_source() -> Option<PathBuf> {
    // Common locations to check
    let search_paths = vec![
        PathBuf::from("."),
        PathBuf::from(".."),
        PathBuf::from("../.."),
        dirs::home_dir()?.join("softmata/horus"),
        dirs::home_dir()?.join("HORUS"),
    ];

    for path in search_paths {
        let install_script = path.join("install.sh");
        if install_script.exists() {
            return Some(path);
        }
    }

    None
}

/// Extract version from package directory name (e.g., "horus@0.1.0" -> "0.1.0")
pub fn extract_version_from_path(path: &Path) -> Option<String> {
    path.file_name()?
        .to_str()?
        .split('@')
        .nth(1)
        .map(|s| s.to_string())
}
