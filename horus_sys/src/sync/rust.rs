//! Rust toolchain synchronization via rustup.

use super::DepStatus;
use std::process::Command;

/// Check if Rust is installed and get its version.
pub fn check_rust(edition: Option<&str>) -> DepStatus {
    let (installed, version) = detect_rust_version();
    let _ = edition; // Edition affects Cargo.toml, not rustup

    DepStatus {
        name: "rust".to_string(),
        required: true,
        installed,
        version,
        install_cmd: if installed {
            None
        } else {
            Some("curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh".to_string())
        },
    }
}

/// Ensure Rust is installed. Returns status.
pub fn ensure_rust(edition: Option<&str>) -> anyhow::Result<DepStatus> {
    let status = check_rust(edition);
    if status.installed {
        return Ok(status);
    }

    // Don't auto-install Rust — too invasive. Report what to do.
    log::warn!(
        "Rust not found. Install via: {}",
        status.install_cmd.as_deref().unwrap_or("rustup")
    );
    Ok(status)
}

fn detect_rust_version() -> (bool, Option<String>) {
    match Command::new("rustc").arg("--version").output() {
        Ok(output) if output.status.success() => {
            let version = String::from_utf8_lossy(&output.stdout)
                .trim()
                .strip_prefix("rustc ")
                .unwrap_or("unknown")
                .split_whitespace()
                .next()
                .unwrap_or("unknown")
                .to_string();
            (true, Some(version))
        }
        _ => (false, None),
    }
}
