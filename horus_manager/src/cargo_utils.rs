//! Shared cargo/binary detection utilities.

use anyhow::Result;
use std::path::Path;

/// Detect if a Rust binary exists in the system's `~/.cargo/bin/` directory.
///
/// Returns `Ok(Some(version))` if the binary exists (version may be `"unknown"`),
/// or `Ok(None)` if the binary is not found.
pub fn detect_system_cargo_binary(package_name: &str) -> Result<Option<String>> {
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

/// Check if a path is executable.
#[cfg(unix)]
pub fn is_executable(path: &Path) -> bool {
    use std::os::unix::fs::PermissionsExt;

    if let Ok(metadata) = path.metadata() {
        metadata.permissions().mode() & 0o111 != 0
    } else {
        false
    }
}

/// Check if a path is executable (non-unix: just checks existence).
#[cfg(not(unix))]
pub fn is_executable(path: &Path) -> bool {
    path.exists()
}
