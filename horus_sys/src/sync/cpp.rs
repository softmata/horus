//! C++ toolchain synchronization (cmake, compiler).

use super::DepStatus;
use std::process::Command;

/// Check if cmake is installed.
pub fn check_cmake() -> DepStatus {
    let (installed, version) = detect_cmake_version();

    DepStatus {
        name: "cmake".to_string(),
        required: true,
        installed,
        version,
        install_cmd: if installed {
            None
        } else {
            Some(crate::platform::suggest_install("cmake"))
        },
    }
}

/// Check if a C++ compiler is installed.
pub fn check_compiler() -> DepStatus {
    let (installed, version) = detect_cpp_compiler();

    DepStatus {
        name: "c++ compiler".to_string(),
        required: true,
        installed,
        version,
        install_cmd: if installed {
            None
        } else {
            Some(crate::platform::suggest_build_tools())
        },
    }
}

/// Ensure cmake is installed.
pub fn ensure_cmake() -> anyhow::Result<DepStatus> {
    let status = check_cmake();
    if !status.installed {
        log::warn!(
            "cmake not found. Install via: {}",
            status.install_cmd.as_deref().unwrap_or("cmake")
        );
    }
    Ok(status)
}

/// Ensure a C++ compiler is installed.
pub fn ensure_compiler() -> anyhow::Result<DepStatus> {
    let status = check_compiler();
    if !status.installed {
        log::warn!(
            "C++ compiler not found. Install via: {}",
            status.install_cmd.as_deref().unwrap_or("g++")
        );
    }
    Ok(status)
}

fn detect_cmake_version() -> (bool, Option<String>) {
    match Command::new("cmake").arg("--version").output() {
        Ok(output) if output.status.success() => {
            let text = String::from_utf8_lossy(&output.stdout);
            let version = text
                .lines()
                .next()
                .and_then(|l| l.strip_prefix("cmake version "))
                .unwrap_or("unknown")
                .trim()
                .to_string();
            (true, Some(version))
        }
        _ => (false, None),
    }
}

fn detect_cpp_compiler() -> (bool, Option<String>) {
    for cmd in &["g++", "clang++", "c++"] {
        if let Ok(output) = Command::new(cmd).arg("--version").output() {
            if output.status.success() {
                let text = String::from_utf8_lossy(&output.stdout);
                let version = text.lines().next().unwrap_or("unknown").trim().to_string();
                return (true, Some(version));
            }
        }
    }
    (false, None)
}
