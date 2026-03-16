//! Python toolchain synchronization.

use super::DepStatus;
use std::process::Command;

/// Check if Python is installed and get its version.
pub fn check_python(version_req: Option<&str>) -> DepStatus {
    let (installed, version) = detect_python_version();
    let _ = version_req;

    DepStatus {
        name: "python".to_string(),
        required: true,
        installed,
        version,
        install_cmd: if installed {
            None
        } else {
            Some(crate::platform::suggest_install("python3"))
        },
    }
}

/// Ensure Python is installed.
pub fn ensure_python(version_req: Option<&str>) -> anyhow::Result<DepStatus> {
    let status = check_python(version_req);
    if status.installed {
        return Ok(status);
    }

    log::warn!(
        "Python not found. Install via: {}",
        status.install_cmd.as_deref().unwrap_or("python3")
    );
    Ok(status)
}

fn detect_python_version() -> (bool, Option<String>) {
    for cmd in &["python3", "python"] {
        if let Ok(output) = Command::new(cmd).arg("--version").output() {
            if output.status.success() {
                let version = String::from_utf8_lossy(&output.stdout)
                    .trim()
                    .strip_prefix("Python ")
                    .unwrap_or("unknown")
                    .to_string();
                return (true, Some(version));
            }
        }
    }
    (false, None)
}
