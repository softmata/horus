use anyhow::{bail, Context, Result};
use colored::*;
use std::fs;
use std::path::{Path, PathBuf};

/// Get the CLI version from Cargo.toml at compile time
pub fn get_cli_version() -> &'static str {
    env!("CARGO_PKG_VERSION")
}

/// Get the installed library version from ~/.horus/installed_version
pub fn get_installed_version() -> Result<Option<String>> {
    let version_file = get_version_file_path()?;

    if !version_file.exists() {
        return Ok(None);
    }

    let version = fs::read_to_string(&version_file)
        .context("failed to read installed version file")?
        .trim()
        .to_string();

    Ok(Some(version))
}

/// Get the path to the version tracking file
fn get_version_file_path() -> Result<PathBuf> {
    Ok(crate::paths::home_dir()?.join(".horus/installed_version"))
}

/// Check if the CLI version matches the installed library version
pub fn check_version_compatibility() -> Result<()> {
    let cli_version = get_cli_version();
    log::debug!("CLI version: {}", cli_version);

    match get_installed_version()? {
        Some(installed_version) => {
            log::debug!("installed library version: {}", installed_version);
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
    log::warn!(
        "Version mismatch: CLI={}, libraries={}. Run install.sh to update.",
        cli_version,
        installed_version
    );
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

#[cfg(test)]
mod tests {
    use super::*;

    // ========================================================================
    // get_cli_version tests
    // ========================================================================

    #[test]
    fn test_get_cli_version_returns_valid_semver() {
        let version = get_cli_version();
        assert!(!version.is_empty(), "CLI version should not be empty");
        // Should be a valid semver-like string (e.g., "0.1.9")
        let parts: Vec<&str> = version.split('.').collect();
        assert!(
            parts.len() >= 2,
            "Version '{}' should have at least major.minor",
            version
        );
        // Major and minor should be numeric
        assert!(
            parts[0].parse::<u32>().is_ok(),
            "Major version '{}' should be numeric",
            parts[0]
        );
        assert!(
            parts[1].parse::<u32>().is_ok(),
            "Minor version '{}' should be numeric",
            parts[1]
        );
    }

    #[test]
    fn test_get_cli_version_is_stable() {
        // Calling multiple times should return the same value
        let v1 = get_cli_version();
        let v2 = get_cli_version();
        assert_eq!(v1, v2);
    }

    // ========================================================================
    // get_version_file_path tests
    // ========================================================================

    #[test]
    fn test_get_version_file_path_ends_with_expected() {
        let path = get_version_file_path().unwrap();
        assert!(
            path.ends_with(".horus/installed_version"),
            "Path should end with .horus/installed_version, got: {:?}",
            path
        );
    }

    #[test]
    fn test_get_version_file_path_is_absolute() {
        let path = get_version_file_path().unwrap();
        assert!(
            path.is_absolute(),
            "Version file path should be absolute, got: {:?}",
            path
        );
    }

    // ========================================================================
    // extract_version_from_path tests
    // ========================================================================

    #[test]
    fn test_extract_version_simple() {
        let path = Path::new("/packages/horus@0.1.0");
        assert_eq!(extract_version_from_path(path), Some("0.1.0".to_string()));
    }

    #[test]
    fn test_extract_version_complex_semver() {
        let path = Path::new("/packages/my-package@1.2.3-beta.1");
        assert_eq!(
            extract_version_from_path(path),
            Some("1.2.3-beta.1".to_string())
        );
    }

    #[test]
    fn test_extract_version_no_at_symbol() {
        let path = Path::new("/packages/horus");
        assert_eq!(extract_version_from_path(path), None);
    }

    #[test]
    fn test_extract_version_empty_version() {
        let path = Path::new("/packages/horus@");
        // split('@').nth(1) returns Some("") for "horus@"
        assert_eq!(extract_version_from_path(path), Some("".to_string()));
    }

    #[test]
    fn test_extract_version_multiple_at_symbols() {
        // Only the first @ matters for the split
        let path = Path::new("/packages/name@1.0@extra");
        assert_eq!(extract_version_from_path(path), Some("1.0".to_string()));
    }

    #[test]
    fn test_extract_version_nested_path() {
        let path = Path::new("/home/user/.horus/packages/lidar-driver@2.0.0");
        assert_eq!(extract_version_from_path(path), Some("2.0.0".to_string()));
    }

    // ========================================================================
    // get_installed_version tests (with temp files)
    // ========================================================================

    #[test]
    fn test_get_installed_version_no_file() {
        // The actual function reads from ~/.horus/installed_version
        // If the file doesn't exist, it should return Ok(None)
        // This is hard to test in isolation without mocking the filesystem
        // but we can at least verify it doesn't panic
        let result = get_installed_version();
        assert!(
            result.is_ok(),
            "Should not error even if file doesn't exist"
        );
    }

    // ========================================================================
    // find_horus_source tests
    // ========================================================================

    #[test]
    fn test_find_horus_source_returns_option() {
        // find_horus_source searches common locations
        // Result depends on the system, but should not panic
        let result = find_horus_source();
        // If we're running from the horus source tree, it might find it
        if let Some(path) = &result {
            assert!(
                path.join("install.sh").exists(),
                "Found path should contain install.sh"
            );
        }
    }

    // ========================================================================
    // print_version_mismatch tests (smoke test)
    // ========================================================================

    #[test]
    fn test_print_version_mismatch_does_not_panic() {
        // Just verify it doesn't panic with various inputs
        print_version_mismatch("1.0.0", "0.9.0");
        print_version_mismatch("", "");
        print_version_mismatch("1.0.0-beta", "1.0.0");
    }
}
