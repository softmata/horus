use anyhow::{bail, Context, Result};
use colored::*;
use std::fs;
use std::path::PathBuf;

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
#[cfg(test)]
fn extract_version_from_path(path: &std::path::Path) -> Option<String> {
    path.file_name()?
        .to_str()?
        .split('@')
        .nth(1)
        .map(|s| s.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

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
        // Smoke test: print_version_mismatch is a pure side-effect function
        // (writes to stderr). We verify it handles edge cases without panicking,
        // including empty strings and pre-release versions.
        print_version_mismatch("1.0.0", "0.9.0"); // normal mismatch
        print_version_mismatch("", ""); // empty versions
        print_version_mismatch("1.0.0-beta", "1.0.0"); // pre-release vs release
    }

    // ========================================================================
    // Semver comparison tests
    // ========================================================================

    #[test]
    fn test_version_comparison() {
        use semver::Version;

        let v010 = Version::parse("0.1.0").unwrap();
        let v020 = Version::parse("0.2.0").unwrap();
        let v100 = Version::parse("1.0.0").unwrap();

        // Strict ordering: 0.1.0 < 0.2.0 < 1.0.0
        assert!(
            v010 < v020,
            "0.1.0 should be less than 0.2.0"
        );
        assert!(
            v020 < v100,
            "0.2.0 should be less than 1.0.0"
        );
        assert!(
            v010 < v100,
            "0.1.0 should be less than 1.0.0 (transitivity)"
        );

        // Pre-release is less than release
        let v100_beta = Version::parse("1.0.0-beta").unwrap();
        assert!(
            v100_beta < v100,
            "1.0.0-beta should be less than 1.0.0 (pre-release < release)"
        );

        // Patch ordering
        let v019 = Version::parse("0.1.9").unwrap();
        assert!(
            v010 < v019,
            "0.1.0 should be less than 0.1.9"
        );
        assert!(
            v019 < v020,
            "0.1.9 should be less than 0.2.0"
        );
    }

    #[test]
    fn test_version_parse_valid() {
        use semver::Version;

        // Standard semver
        let v1 = Version::parse("1.2.3");
        assert!(
            v1.is_ok(),
            "1.2.3 should parse as valid semver, got: {:?}",
            v1.err()
        );
        let v1 = v1.unwrap();
        assert_eq!(v1.major, 1);
        assert_eq!(v1.minor, 2);
        assert_eq!(v1.patch, 3);

        // Pre-release version
        let v2 = Version::parse("0.1.0-beta");
        assert!(
            v2.is_ok(),
            "0.1.0-beta should parse as valid semver, got: {:?}",
            v2.err()
        );
        let v2 = v2.unwrap();
        assert_eq!(v2.major, 0);
        assert_eq!(v2.minor, 1);
        assert_eq!(v2.patch, 0);
        assert!(
            !v2.pre.is_empty(),
            "Pre-release field should be non-empty for 0.1.0-beta"
        );

        // Build metadata version
        let v3 = Version::parse("2.0.0+build123");
        assert!(
            v3.is_ok(),
            "2.0.0+build123 should parse as valid semver, got: {:?}",
            v3.err()
        );
        let v3 = v3.unwrap();
        assert_eq!(v3.major, 2);
        assert_eq!(v3.minor, 0);
        assert_eq!(v3.patch, 0);
        assert!(
            !v3.build.is_empty(),
            "Build metadata should be non-empty for 2.0.0+build123"
        );

        // Pre-release + build metadata combined
        let v4 = Version::parse("1.0.0-alpha.1+build.456");
        assert!(
            v4.is_ok(),
            "1.0.0-alpha.1+build.456 should parse as valid semver, got: {:?}",
            v4.err()
        );
    }
}
