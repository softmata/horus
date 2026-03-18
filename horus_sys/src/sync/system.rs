//! System dependency checking via pkg-config and platform package managers.

use super::{DepStatus, SystemDep};
use std::process::Command;

/// Check if a system dependency is installed.
pub fn check_system_dep(dep: &SystemDep) -> DepStatus {
    let (installed, version) = if let Some(ref pkg) = dep.pkg_config {
        detect_pkg_config(pkg)
    } else {
        // Try pkg-config with the logical name
        detect_pkg_config(&dep.name)
    };

    let install_cmd = if installed {
        None
    } else {
        suggest_install_cmd(dep)
    };

    DepStatus {
        name: dep.name.clone(),
        required: true,
        installed,
        version,
        install_cmd,
    }
}

/// Ensure a system dependency is installed.
pub fn ensure_system_dep(dep: &SystemDep) -> anyhow::Result<DepStatus> {
    let status = check_system_dep(dep);
    if !status.installed {
        log::warn!(
            "{} not found. Install via: {}",
            dep.name,
            status.install_cmd.as_deref().unwrap_or("(unknown)")
        );
    }
    Ok(status)
}

fn detect_pkg_config(name: &str) -> (bool, Option<String>) {
    match Command::new("pkg-config")
        .args(["--modversion", name])
        .output()
    {
        Ok(output) if output.status.success() => {
            let version = String::from_utf8_lossy(&output.stdout).trim().to_string();
            (true, Some(version))
        }
        _ => (false, None),
    }
}

fn suggest_install_cmd(dep: &SystemDep) -> Option<String> {
    let distro = crate::platform::detect_distro();

    match distro {
        crate::platform::Distro::Ubuntu | crate::platform::Distro::Debian => dep
            .apt
            .as_ref()
            .map(|pkg| format!("sudo apt install {}", pkg)),
        crate::platform::Distro::Fedora => dep.apt.as_ref().map(|pkg| {
            let pkg = pkg.replace("-dev", "-devel");
            format!("sudo dnf install {}", pkg)
        }),
        crate::platform::Distro::Arch => dep
            .brew
            .as_ref()
            .or(dep.apt.as_ref())
            .map(|pkg| format!("sudo pacman -S {}", pkg)),
        crate::platform::Distro::MacOS => {
            dep.brew.as_ref().map(|pkg| format!("brew install {}", pkg))
        }
        crate::platform::Distro::Windows => dep
            .choco
            .as_ref()
            .map(|pkg| format!("choco install {}", pkg)),
        _ => dep
            .apt
            .as_ref()
            .map(|pkg| crate::platform::suggest_install(pkg)),
    }
}
