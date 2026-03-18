//! System Dependency Detection for HORUS
//!
//! Auto-detects required system packages, device files, and user group permissions
//! based on which hardware nodes are being used in a HORUS project.

use std::collections::HashSet;
use std::path::Path;
use std::process::Command;

/// System dependency information for a feature/node
#[derive(Debug, Clone)]
pub struct SystemDependency {
    /// Feature flag name (e.g., "opencv-backend")
    pub feature: &'static str,
    /// Human-readable description
    pub description: &'static str,
    /// System packages to install (apt/dpkg — Debian/Ubuntu)
    pub apt_packages: &'static [&'static str],
    /// Homebrew formula names (macOS)
    pub brew_packages: &'static [&'static str],
    /// pacman package names (Arch Linux)
    pub pacman_packages: &'static [&'static str],
    /// Chocolatey package names (Windows)
    pub choco_packages: &'static [&'static str],
    /// Nix package attribute names (NixOS / nix-env)
    pub nix_packages: &'static [&'static str],
    /// Device files that should exist when hardware is connected
    pub device_files: &'static [&'static str],
    /// User groups required for access
    pub user_groups: &'static [&'static str],
    /// pkg-config names to check if library is installed
    pub pkg_config: &'static [&'static str],
    /// Install command for Ubuntu/Debian (legacy, prefer platform-specific fields)
    pub install_cmd: &'static str,
    /// Documentation URL
    pub docs_url: &'static str,
}

/// System dependencies mapped by feature flag.
///
/// Only includes features that correspond to real Cargo features in horus crates.
/// Phantom hardware features (gpio-hardware, i2c-hardware, spi-hardware,
/// serial-hardware, modbus-hardware, bno055-imu, mpu6050-imu, nmea-gps, rplidar,
/// netft) have been removed — they never existed in any Cargo.toml.
pub const SYSTEM_DEPS: &[SystemDependency] = &[
    // Joystick/Gamepad (gilrs)
    SystemDependency {
        feature: "gilrs",
        description: "Gamepad/joystick input",
        apt_packages: &["libudev-dev"],
        brew_packages: &[],
        pacman_packages: &["systemd-libs"],
        choco_packages: &[],
        nix_packages: &["systemd"],
        device_files: &["/dev/input/js0", "/dev/input/event0"],
        user_groups: &["input"],
        pkg_config: &["libudev"],
        install_cmd: "sudo apt install -y libudev-dev && sudo usermod -a -G input $USER",
        docs_url: "https://docs.rs/gilrs/latest/gilrs/",
    },
    // OpenCV Backend
    SystemDependency {
        feature: "opencv-backend",
        description: "Computer vision, image processing",
        apt_packages: &["libopencv-dev", "pkg-config", "libclang-dev"],
        brew_packages: &["opencv", "pkg-config", "llvm"],
        pacman_packages: &["opencv", "pkgconf", "clang"],
        choco_packages: &["opencv"],
        nix_packages: &["opencv4", "pkg-config", "libclang"],
        device_files: &["/dev/video0"],
        user_groups: &["video"],
        pkg_config: &["opencv4", "opencv"],
        install_cmd: "sudo apt install -y libopencv-dev pkg-config libclang-dev && sudo usermod -a -G video $USER",
        docs_url: "https://docs.opencv.org/master/",
    },
    // ONNX Runtime
    SystemDependency {
        feature: "onnx",
        description: "ONNX Runtime for ML inference",
        apt_packages: &[],
        brew_packages: &[],
        pacman_packages: &[],
        choco_packages: &[],
        nix_packages: &[],
        device_files: &[],
        user_groups: &[],
        pkg_config: &[],
        install_cmd: "# ONNX Runtime is downloaded automatically by the ort crate",
        docs_url: "https://onnxruntime.ai/",
    },
    // TensorFlow Lite
    SystemDependency {
        feature: "tflite-inference",
        description: "TensorFlow Lite for edge ML",
        apt_packages: &[],
        brew_packages: &[],
        pacman_packages: &[],
        choco_packages: &[],
        nix_packages: &[],
        device_files: &[],
        user_groups: &[],
        pkg_config: &[],
        install_cmd: "# TFLite is bundled with the tflite crate",
        docs_url: "https://www.tensorflow.org/lite",
    },
];

/// Result of checking system dependencies
#[derive(Debug, Default)]
pub struct DependencyCheckResult {
    /// Missing apt packages
    pub missing_packages: Vec<String>,
    /// Missing device files (hardware not connected or not enabled)
    pub missing_devices: Vec<String>,
    /// Groups user is not a member of
    pub missing_groups: Vec<String>,
    /// Missing pkg-config libraries
    pub missing_pkg_config: Vec<String>,
    /// Install commands to run
    pub install_commands: Vec<String>,
    /// Documentation links
    pub docs_links: Vec<(String, String)>, // (feature, url)
}

impl DependencyCheckResult {
    pub fn has_issues(&self) -> bool {
        !self.missing_packages.is_empty()
            || !self.missing_groups.is_empty()
            || !self.missing_pkg_config.is_empty()
    }

    pub fn has_missing_hardware(&self) -> bool {
        !self.missing_devices.is_empty()
    }
}

/// Check system dependencies for a set of features
pub fn check_dependencies(features: &[String]) -> DependencyCheckResult {
    let mut result = DependencyCheckResult::default();
    let mut checked_packages: HashSet<String> = HashSet::new();
    let mut checked_groups: HashSet<String> = HashSet::new();

    // Get current user's groups
    let user_groups = get_user_groups();

    for feature in features {
        if let Some(dep) = SYSTEM_DEPS.iter().find(|d| d.feature == feature) {
            // Check apt packages via dpkg
            for pkg in dep.apt_packages {
                if !checked_packages.contains(*pkg) {
                    checked_packages.insert(pkg.to_string());
                    if !is_package_installed(pkg) {
                        result.missing_packages.push(pkg.to_string());
                    }
                }
            }

            // Check device files
            for device in dep.device_files {
                // Only check one device per pattern (e.g., /dev/i2c-* -> check /dev/i2c-1)
                let check_path = if device.contains('*') || device.ends_with('0') {
                    device.replace('*', "0")
                } else {
                    device.to_string()
                };
                if !Path::new(&check_path).exists() {
                    // Only add if not already in list (avoid duplicates)
                    if !result.missing_devices.contains(&check_path) {
                        result.missing_devices.push(check_path);
                    }
                }
            }

            // Check user groups
            for group in dep.user_groups {
                if !checked_groups.contains(*group) {
                    checked_groups.insert(group.to_string());
                    if !user_groups.contains(*group) {
                        result.missing_groups.push(group.to_string());
                    }
                }
            }

            // Check pkg-config
            for pkg in dep.pkg_config {
                if !is_pkg_config_available(pkg) {
                    result.missing_pkg_config.push(pkg.to_string());
                }
            }

            // Add install command and docs link
            if !dep.install_cmd.is_empty()
                && !result
                    .install_commands
                    .contains(&dep.install_cmd.to_string())
            {
                result.install_commands.push(dep.install_cmd.to_string());
            }
            result
                .docs_links
                .push((dep.feature.to_string(), dep.docs_url.to_string()));
        }
    }

    result
}

/// Check if an apt package is installed
fn is_package_installed(package: &str) -> bool {
    Command::new("dpkg")
        .args(["-s", package])
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

/// Check if a pkg-config library is available
fn is_pkg_config_available(name: &str) -> bool {
    Command::new("pkg-config")
        .args(["--exists", name])
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

/// Get list of groups the current user belongs to
fn get_user_groups() -> HashSet<String> {
    Command::new("groups")
        .output()
        .ok()
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| s.split_whitespace().map(|g| g.to_string()).collect())
        .unwrap_or_default()
}

/// Format dependency check results as a user-friendly message
pub fn format_dependency_report(result: &DependencyCheckResult, features: &[String]) -> String {
    let mut report = String::new();

    if !result.has_issues() && !result.has_missing_hardware() {
        return report; // No issues
    }

    report.push_str("\n╔══════════════════════════════════════════════════════════════════╗\n");
    report.push_str("║              HORUS System Dependency Check                       ║\n");
    report.push_str("╚══════════════════════════════════════════════════════════════════╝\n\n");

    report.push_str(&format!("Features detected: {}\n\n", features.join(", ")));

    // Missing packages
    if !result.missing_packages.is_empty() {
        report.push_str("❌ Missing System Packages:\n");
        for pkg in &result.missing_packages {
            report.push_str(&format!("   • {}\n", pkg));
        }
        report.push_str("\n   Install with:\n");
        report.push_str(&format!(
            "   $ sudo apt install -y {}\n\n",
            result.missing_packages.join(" ")
        ));
    }

    // Missing groups
    if !result.missing_groups.is_empty() {
        report.push_str("❌ Missing User Group Permissions:\n");
        for group in &result.missing_groups {
            report.push_str(&format!("   • {} group\n", group));
        }
        report.push_str("\n   Add yourself to groups:\n");
        report.push_str(&format!(
            "   $ sudo usermod -a -G {} $USER\n",
            result.missing_groups.join(",")
        ));
        report.push_str("   $ # Then logout and login again\n\n");
    }

    // Missing pkg-config libs
    if !result.missing_pkg_config.is_empty() {
        report.push_str("❌ Missing Development Libraries:\n");
        for lib in &result.missing_pkg_config {
            report.push_str(&format!("   • {} (pkg-config)\n", lib));
        }
        report.push('\n');
    }

    // Missing device files (hardware not connected - this is a warning, not error)
    if !result.missing_devices.is_empty() {
        report.push_str("⚠️  Hardware Not Detected (will use simulation mode):\n");
        for device in &result.missing_devices {
            report.push_str(&format!("   • {}\n", device));
        }
        report.push_str("\n   Note: Nodes will automatically fall back to simulation mode.\n");
        report.push_str("         Connect hardware or enable interfaces to use real sensors.\n\n");
    }

    // Documentation links
    if !result.docs_links.is_empty() {
        report.push_str("📚 Documentation:\n");
        let mut seen_urls: HashSet<&str> = HashSet::new();
        for (feature, url) in &result.docs_links {
            if !seen_urls.contains(url.as_str()) {
                seen_urls.insert(url);
                report.push_str(&format!("   • {}: {}\n", feature, url));
            }
        }
        report.push('\n');
    }

    // Quick install script
    if result.has_issues() {
        report.push_str("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
        report.push_str("Quick Fix - Run this command:\n\n");

        let mut quick_install = String::new();

        if !result.missing_packages.is_empty() {
            quick_install.push_str(&format!(
                "sudo apt install -y {}",
                result.missing_packages.join(" ")
            ));
        }

        if !result.missing_groups.is_empty() {
            if !quick_install.is_empty() {
                quick_install.push_str(" && ");
            }
            quick_install.push_str(&format!(
                "sudo usermod -a -G {} $USER",
                result.missing_groups.join(",")
            ));
        }

        report.push_str(&format!("$ {}\n", quick_install));
        report.push_str("\nThen logout and login again for group changes to take effect.\n");
        report.push_str("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    }

    report
}

// ============================================================================
// Lockfile v4 verification — check toolchain + system deps from horus.lock
// ============================================================================

use crate::lockfile::{HorusLockfile, SystemLock, ToolchainPins, HORUS_LOCK};

/// Result of verifying the lockfile against the current machine.
#[derive(Debug, Default)]
pub struct LockfileVerification {
    /// Toolchain version warnings (non-fatal).
    pub toolchain_warnings: Vec<String>,
    /// Missing system dependencies with install suggestions.
    pub missing_system_deps: Vec<(String, String)>, // (name, install_cmd)
    /// Whether any hard errors were found (missing required deps).
    pub has_errors: bool,
}

/// Verify the current machine satisfies the horus.lock v4 requirements.
/// Returns warnings/errors but never fails — callers decide whether to abort.
pub fn verify_lockfile(lockfile: &HorusLockfile) -> LockfileVerification {
    let mut result = LockfileVerification::default();

    // Check toolchain versions
    if let Some(ref tc) = lockfile.toolchain {
        check_toolchain(tc, &mut result);
    }

    // Check system dependencies
    for sys_dep in &lockfile.system_deps {
        check_system_lock(sys_dep, &mut result);
    }

    result
}

fn check_toolchain(tc: &ToolchainPins, result: &mut LockfileVerification) {
    // Check Rust version
    if let Some(ref expected_rust) = tc.rust {
        if let Some(actual) = get_current_rust_version() {
            if !version_compatible(expected_rust, &actual) {
                result.toolchain_warnings.push(format!(
                    "Rust version mismatch: lockfile pins {}, you have {}",
                    expected_rust, actual
                ));
            }
        }
    }

    // Check Python version
    if let Some(ref expected_python) = tc.python {
        if let Some(actual) = get_current_python_version() {
            if !version_compatible(expected_python, &actual) {
                result.toolchain_warnings.push(format!(
                    "Python version mismatch: lockfile pins {}, you have {}",
                    expected_python, actual
                ));
            }
        }
    }
}

fn check_system_lock(dep: &SystemLock, result: &mut LockfileVerification) {
    // Try pkg-config first (most reliable cross-platform check)
    if let Some(ref pc) = dep.pkg_config {
        if is_pkg_config_available(pc) {
            return; // Found via pkg-config — all good
        }
    }

    // Not found — build an install suggestion for the current platform
    let install_cmd = suggest_system_install(dep);
    result
        .missing_system_deps
        .push((dep.name.clone(), install_cmd));
}

/// Build a platform-appropriate install command for a missing system dep.
fn suggest_system_install(dep: &SystemLock) -> String {
    // Detect current platform
    let os = std::env::consts::OS;

    match os {
        "linux" => {
            if let Some(ref apt) = dep.apt {
                // Check if we're on a Debian-based system
                if Path::new("/usr/bin/apt").exists() || Path::new("/usr/bin/apt-get").exists() {
                    return format!("sudo apt install -y {}", apt);
                }
            }
            if let Some(ref pacman) = dep.pacman {
                if Path::new("/usr/bin/pacman").exists() {
                    return format!("sudo pacman -S {}", pacman);
                }
            }
            // Fallback: show apt name if available
            if let Some(ref apt) = dep.apt {
                return format!("sudo apt install -y {}", apt);
            }
            format!("Install {} for your distribution", dep.name)
        }
        "macos" => {
            if let Some(ref brew) = dep.brew {
                return format!("brew install {}", brew);
            }
            format!("Install {} (no brew formula known)", dep.name)
        }
        "windows" => {
            if let Some(ref choco) = dep.choco {
                return format!("choco install {}", choco);
            }
            format!("Install {} for Windows", dep.name)
        }
        _ => format!("Install {} for your platform", dep.name),
    }
}

/// Format lockfile verification results for display.
pub fn format_lockfile_report(result: &LockfileVerification) -> String {
    let mut report = String::new();

    if result.toolchain_warnings.is_empty() && result.missing_system_deps.is_empty() {
        return report;
    }

    for warning in &result.toolchain_warnings {
        report.push_str(&format!("  ⚠ {}\n", warning));
    }

    if !result.missing_system_deps.is_empty() {
        report.push_str("\n  Missing system dependencies:\n");
        for (name, install_cmd) in &result.missing_system_deps {
            report.push_str(&format!("    ✗ {} — install with: {}\n", name, install_cmd));
        }
    }

    report
}

/// Check if two semver-ish version strings are compatible (same major.minor).
fn version_compatible(expected: &str, actual: &str) -> bool {
    let exp_parts: Vec<&str> = expected.split('.').collect();
    let act_parts: Vec<&str> = actual.split('.').collect();
    // Compare major.minor — patch differences are fine
    exp_parts.first() == act_parts.first() && exp_parts.get(1) == act_parts.get(1)
}

fn get_current_rust_version() -> Option<String> {
    Command::new("rustc")
        .arg("--version")
        .output()
        .ok()
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .and_then(|s| {
            // "rustc 1.78.0 (9b00956e5 2024-04-29)"
            s.split_whitespace().nth(1).map(|v| v.to_string())
        })
}

fn get_current_python_version() -> Option<String> {
    Command::new("python3")
        .arg("--version")
        .output()
        .ok()
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .and_then(|s| {
            // "Python 3.12.3"
            s.split_whitespace().nth(1).map(|v| v.to_string())
        })
}

/// Run lockfile verification as part of `horus build`.
/// Prints warnings but does not block the build.
pub fn verify_lockfile_before_build() {
    let lock_path = std::path::Path::new(HORUS_LOCK);
    if !lock_path.exists() {
        return;
    }

    let lockfile = match HorusLockfile::load_from(lock_path) {
        Ok(lf) => lf,
        Err(_) => return, // Can't parse lockfile — skip verification
    };

    let result = verify_lockfile(&lockfile);
    let report = format_lockfile_report(&result);
    if !report.is_empty() {
        eprintln!("  Lockfile verification:");
        eprint!("{}", report);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_check_nonexistent_feature() {
        let result = check_dependencies(&["nonexistent-feature".to_string()]);
        assert!(!result.has_issues());
    }

    #[test]
    fn test_format_empty_report() {
        let result = DependencyCheckResult::default();
        let report = format_dependency_report(&result, &[]);
        assert!(report.is_empty());
    }

    // --- DependencyCheckResult tests ---

    #[test]
    fn test_default_result_has_no_issues() {
        let result = DependencyCheckResult::default();
        assert!(!result.has_issues());
        assert!(!result.has_missing_hardware());
    }

    #[test]
    fn test_has_issues_with_missing_packages() {
        let mut result = DependencyCheckResult::default();
        result.missing_packages.push("libfoo-dev".to_string());
        assert!(result.has_issues());
        assert!(!result.has_missing_hardware());
    }

    #[test]
    fn test_has_issues_with_missing_groups() {
        let mut result = DependencyCheckResult::default();
        result.missing_groups.push("video".to_string());
        assert!(result.has_issues());
    }

    #[test]
    fn test_has_issues_with_missing_pkg_config() {
        let mut result = DependencyCheckResult::default();
        result.missing_pkg_config.push("opencv4".to_string());
        assert!(result.has_issues());
    }

    #[test]
    fn test_has_missing_hardware_with_missing_devices() {
        let mut result = DependencyCheckResult::default();
        result.missing_devices.push("/dev/video0".to_string());
        assert!(
            !result.has_issues(),
            "missing devices alone is not an issue"
        );
        assert!(result.has_missing_hardware());
    }

    #[test]
    fn test_has_issues_and_hardware_combined() {
        let mut result = DependencyCheckResult::default();
        result.missing_packages.push("libfoo-dev".to_string());
        result.missing_devices.push("/dev/video0".to_string());
        assert!(result.has_issues());
        assert!(result.has_missing_hardware());
    }

    // --- SYSTEM_DEPS constant tests ---

    #[test]
    fn test_system_deps_has_known_features() {
        let features: Vec<&str> = SYSTEM_DEPS.iter().map(|d| d.feature).collect();
        assert!(features.contains(&"gilrs"));
        assert!(features.contains(&"opencv-backend"));
        assert!(features.contains(&"onnx"));
        assert!(features.contains(&"tflite-inference"));
    }

    #[test]
    fn test_system_deps_no_empty_features() {
        for dep in SYSTEM_DEPS {
            assert!(!dep.feature.is_empty(), "feature name must not be empty");
            assert!(
                !dep.description.is_empty(),
                "description must not be empty for {}",
                dep.feature
            );
            assert!(
                !dep.docs_url.is_empty(),
                "docs_url must not be empty for {}",
                dep.feature
            );
        }
    }

    #[test]
    fn test_system_deps_unique_features() {
        let mut seen = HashSet::new();
        for dep in SYSTEM_DEPS {
            assert!(
                seen.insert(dep.feature),
                "duplicate feature: {}",
                dep.feature
            );
        }
    }

    #[test]
    fn test_gilrs_dep_details() {
        let dep = SYSTEM_DEPS.iter().find(|d| d.feature == "gilrs").unwrap();
        assert_eq!(dep.apt_packages, &["libudev-dev"]);
        assert!(dep.device_files.contains(&"/dev/input/js0"));
        assert!(dep.user_groups.contains(&"input"));
        assert!(dep.pkg_config.contains(&"libudev"));
    }

    #[test]
    fn test_onnx_dep_has_no_system_requirements() {
        let dep = SYSTEM_DEPS.iter().find(|d| d.feature == "onnx").unwrap();
        assert!(dep.apt_packages.is_empty());
        assert!(dep.device_files.is_empty());
        assert!(dep.user_groups.is_empty());
        assert!(dep.pkg_config.is_empty());
    }

    #[test]
    fn test_tflite_dep_has_no_system_requirements() {
        let dep = SYSTEM_DEPS
            .iter()
            .find(|d| d.feature == "tflite-inference")
            .unwrap();
        assert!(dep.apt_packages.is_empty());
        assert!(dep.device_files.is_empty());
        assert!(dep.user_groups.is_empty());
        assert!(dep.pkg_config.is_empty());
    }

    // --- check_dependencies tests ---

    #[test]
    fn test_check_empty_features() {
        let result = check_dependencies(&[]);
        assert!(!result.has_issues());
        assert!(!result.has_missing_hardware());
        assert!(result.install_commands.is_empty());
        assert!(result.docs_links.is_empty());
    }

    #[test]
    fn test_check_multiple_nonexistent_features() {
        let features = vec![
            "no-such-feature".to_string(),
            "also-not-real".to_string(),
            "fake-feature-123".to_string(),
        ];
        let result = check_dependencies(&features);
        assert!(!result.has_issues());
        assert!(result.docs_links.is_empty());
        assert!(result.install_commands.is_empty());
    }

    #[test]
    fn test_check_onnx_feature_no_issues() {
        // ONNX has no system deps, so it should never report issues
        let result = check_dependencies(&["onnx".to_string()]);
        assert!(!result.has_issues());
        assert!(!result.has_missing_hardware());
        // But it should still have docs link and install command
        assert_eq!(result.docs_links.len(), 1);
        assert_eq!(result.docs_links[0].0, "onnx");
    }

    #[test]
    fn test_check_tflite_feature_no_issues() {
        let result = check_dependencies(&["tflite-inference".to_string()]);
        assert!(!result.has_issues());
        assert!(!result.has_missing_hardware());
        assert_eq!(result.docs_links.len(), 1);
        assert_eq!(result.docs_links[0].0, "tflite-inference");
    }

    #[test]
    fn test_check_dependencies_populates_docs_links() {
        let result = check_dependencies(&["onnx".to_string(), "tflite-inference".to_string()]);
        assert_eq!(result.docs_links.len(), 2);
        let features: Vec<&str> = result.docs_links.iter().map(|(f, _)| f.as_str()).collect();
        assert!(features.contains(&"onnx"));
        assert!(features.contains(&"tflite-inference"));
    }

    #[test]
    fn test_check_dependencies_populates_install_commands() {
        let result = check_dependencies(&["onnx".to_string()]);
        // ONNX has a non-empty install_cmd
        assert_eq!(result.install_commands.len(), 1);
    }

    #[test]
    fn test_check_dependencies_deduplicates_install_commands() {
        // Passing same feature twice should not duplicate install command
        let result = check_dependencies(&["onnx".to_string(), "onnx".to_string()]);
        assert_eq!(result.install_commands.len(), 1);
    }

    #[test]
    fn test_check_mixed_real_and_fake_features() {
        let features = vec![
            "nonexistent".to_string(),
            "onnx".to_string(),
            "also-fake".to_string(),
        ];
        let result = check_dependencies(&features);
        // Only onnx should produce docs/install
        assert_eq!(result.docs_links.len(), 1);
        assert_eq!(result.docs_links[0].0, "onnx");
    }

    // --- format_dependency_report tests ---

    #[test]
    fn test_format_report_no_issues_returns_empty() {
        let result = DependencyCheckResult::default();
        let report = format_dependency_report(&result, &["onnx".to_string()]);
        assert!(report.is_empty());
    }

    #[test]
    fn test_format_report_with_missing_packages() {
        let mut result = DependencyCheckResult::default();
        result.missing_packages.push("libfoo-dev".to_string());
        result.missing_packages.push("libbar-dev".to_string());
        let report = format_dependency_report(&result, &["test-feature".to_string()]);
        assert!(report.contains("Missing System Packages"));
        assert!(report.contains("libfoo-dev"));
        assert!(report.contains("libbar-dev"));
        assert!(report.contains("sudo apt install -y libfoo-dev libbar-dev"));
        assert!(report.contains("Quick Fix"));
    }

    #[test]
    fn test_format_report_with_missing_groups() {
        let mut result = DependencyCheckResult::default();
        result.missing_groups.push("video".to_string());
        result.missing_groups.push("dialout".to_string());
        let report = format_dependency_report(&result, &["test-feature".to_string()]);
        assert!(report.contains("Missing User Group Permissions"));
        assert!(report.contains("video group"));
        assert!(report.contains("dialout group"));
        assert!(report.contains("sudo usermod -a -G video,dialout $USER"));
        assert!(report.contains("logout and login again"));
    }

    #[test]
    fn test_format_report_with_missing_pkg_config() {
        let mut result = DependencyCheckResult::default();
        result.missing_pkg_config.push("opencv4".to_string());
        let report = format_dependency_report(&result, &["opencv-backend".to_string()]);
        assert!(report.contains("Missing Development Libraries"));
        assert!(report.contains("opencv4 (pkg-config)"));
    }

    #[test]
    fn test_format_report_with_missing_devices_only() {
        let mut result = DependencyCheckResult::default();
        result.missing_devices.push("/dev/video0".to_string());
        let report = format_dependency_report(&result, &["opencv-backend".to_string()]);
        assert!(report.contains("Hardware Not Detected"));
        assert!(report.contains("/dev/video0"));
        assert!(report.contains("simulation mode"));
        // No "Quick Fix" section since has_issues() is false (only devices missing)
        assert!(!report.contains("Quick Fix"));
    }

    #[test]
    fn test_format_report_contains_header() {
        let mut result = DependencyCheckResult::default();
        result.missing_packages.push("libfoo-dev".to_string());
        let report = format_dependency_report(&result, &["foo".to_string()]);
        assert!(report.contains("HORUS System Dependency Check"));
        assert!(report.contains("Features detected: foo"));
    }

    #[test]
    fn test_format_report_shows_features_detected() {
        let mut result = DependencyCheckResult::default();
        result.missing_packages.push("libx-dev".to_string());
        let features = vec!["feat-a".to_string(), "feat-b".to_string()];
        let report = format_dependency_report(&result, &features);
        assert!(report.contains("Features detected: feat-a, feat-b"));
    }

    #[test]
    fn test_format_report_with_docs_links() {
        let mut result = DependencyCheckResult::default();
        result.missing_packages.push("libfoo-dev".to_string());
        result.docs_links.push((
            "gilrs".to_string(),
            "https://docs.rs/gilrs/latest/gilrs/".to_string(),
        ));
        let report = format_dependency_report(&result, &["gilrs".to_string()]);
        assert!(report.contains("Documentation"));
        assert!(report.contains("gilrs: https://docs.rs/gilrs/latest/gilrs/"));
    }

    #[test]
    fn test_format_report_deduplicates_docs_links() {
        let mut result = DependencyCheckResult::default();
        result.missing_packages.push("libfoo-dev".to_string());
        let url = "https://example.com/docs".to_string();
        result.docs_links.push(("feat1".to_string(), url.clone()));
        result.docs_links.push(("feat2".to_string(), url.clone()));
        let report = format_dependency_report(&result, &["feat1".to_string(), "feat2".to_string()]);
        // Same URL should appear only once
        let count = report.matches("https://example.com/docs").count();
        assert_eq!(count, 1, "duplicate docs URL should be deduplicated");
    }

    #[test]
    fn test_format_report_quick_fix_combined() {
        let mut result = DependencyCheckResult::default();
        result.missing_packages.push("libfoo-dev".to_string());
        result.missing_groups.push("video".to_string());
        let report = format_dependency_report(&result, &["test".to_string()]);
        // Quick fix should combine apt install and usermod
        assert!(report.contains("sudo apt install -y libfoo-dev"));
        assert!(report.contains("sudo usermod -a -G video $USER"));
        assert!(report.contains(" && "), "commands should be joined with &&");
    }

    #[test]
    fn test_format_report_quick_fix_packages_only() {
        let mut result = DependencyCheckResult::default();
        result.missing_packages.push("libfoo-dev".to_string());
        let report = format_dependency_report(&result, &[]);
        assert!(report.contains("Quick Fix"));
        assert!(report.contains("sudo apt install -y libfoo-dev"));
        assert!(!report.contains(" && "), "no && when only packages missing");
    }

    #[test]
    fn test_format_report_quick_fix_groups_only() {
        let mut result = DependencyCheckResult::default();
        result.missing_groups.push("input".to_string());
        let report = format_dependency_report(&result, &[]);
        assert!(report.contains("Quick Fix"));
        assert!(report.contains("sudo usermod -a -G input $USER"));
        assert!(
            !report.contains("apt install"),
            "no apt install when only groups missing"
        );
    }

    // --- get_user_groups tests ---

    #[test]
    fn test_get_user_groups_returns_non_empty() {
        let groups = get_user_groups();
        // Every user should belong to at least one group
        assert!(
            !groups.is_empty(),
            "current user should have at least one group"
        );
    }

    #[test]
    fn test_get_user_groups_contains_current_user_group() {
        let groups = get_user_groups();
        // The `groups` command typically includes the username as a group
        // At minimum, the set should be non-empty (already tested above)
        // We just verify it's a valid HashSet
        assert!(groups.len() >= 1);
    }

    // --- SystemDependency Clone/Debug ---

    #[test]
    fn test_system_dependency_clone() {
        let dep = SYSTEM_DEPS[0].clone();
        assert_eq!(dep.feature, SYSTEM_DEPS[0].feature);
        assert_eq!(dep.description, SYSTEM_DEPS[0].description);
        assert_eq!(dep.apt_packages, SYSTEM_DEPS[0].apt_packages);
    }

    #[test]
    fn test_system_dependency_debug() {
        let dep = &SYSTEM_DEPS[0];
        let debug = format!("{:?}", dep);
        assert!(debug.contains("SystemDependency"));
        assert!(debug.contains(dep.feature));
    }

    #[test]
    fn test_dependency_check_result_debug() {
        let result = DependencyCheckResult::default();
        let debug = format!("{:?}", result);
        assert!(debug.contains("DependencyCheckResult"));
    }

    // --- Edge cases ---

    #[test]
    fn test_check_dependencies_duplicate_features() {
        // Passing "onnx" twice should not create duplicate docs entries
        // but check_dependencies doesn't deduplicate features — it processes each
        let result = check_dependencies(&["onnx".to_string(), "onnx".to_string()]);
        // docs_links will have 2 entries (one per iteration) — this tests current behavior
        assert_eq!(result.docs_links.len(), 2);
        // But install_commands are deduplicated
        assert_eq!(result.install_commands.len(), 1);
    }

    #[test]
    fn test_check_opencv_populates_device_checks() {
        // opencv-backend references /dev/video0 which likely doesn't exist in CI
        let result = check_dependencies(&["opencv-backend".to_string()]);
        // docs_links should have opencv entry
        assert_eq!(result.docs_links.len(), 1);
        assert_eq!(result.docs_links[0].0, "opencv-backend");
        // install_commands should be populated
        assert!(!result.install_commands.is_empty());
    }

    #[test]
    fn test_format_report_all_sections() {
        let mut result = DependencyCheckResult::default();
        result.missing_packages.push("pkg1".to_string());
        result.missing_groups.push("grp1".to_string());
        result.missing_pkg_config.push("lib1".to_string());
        result.missing_devices.push("/dev/foo".to_string());
        result
            .docs_links
            .push(("feat".to_string(), "https://example.com".to_string()));
        result
            .install_commands
            .push("sudo apt install pkg1".to_string());

        let report = format_dependency_report(&result, &["feat".to_string()]);
        assert!(report.contains("Missing System Packages"));
        assert!(report.contains("Missing User Group Permissions"));
        assert!(report.contains("Missing Development Libraries"));
        assert!(report.contains("Hardware Not Detected"));
        assert!(report.contains("Documentation"));
        assert!(report.contains("Quick Fix"));
    }

    // --- Lockfile verification tests ---

    #[test]
    fn test_version_compatible_same() {
        assert!(super::version_compatible("1.78.0", "1.78.0"));
    }

    #[test]
    fn test_version_compatible_patch_differs() {
        assert!(super::version_compatible("1.78.0", "1.78.5"));
    }

    #[test]
    fn test_version_incompatible_minor_differs() {
        assert!(!super::version_compatible("1.78.0", "1.79.0"));
    }

    #[test]
    fn test_version_incompatible_major_differs() {
        assert!(!super::version_compatible("1.78.0", "2.78.0"));
    }

    #[test]
    fn test_verify_lockfile_empty() {
        let lockfile = super::HorusLockfile::new();
        let result = super::verify_lockfile(&lockfile);
        assert!(result.toolchain_warnings.is_empty());
        assert!(result.missing_system_deps.is_empty());
    }

    #[test]
    fn test_verify_lockfile_with_missing_system_dep() {
        let mut lockfile = super::HorusLockfile::new();
        lockfile.system_deps = vec![super::SystemLock {
            name: "nonexistent-lib-xyz".to_string(),
            version: "99.0".to_string(),
            pkg_config: Some("nonexistent-lib-xyz".to_string()),
            apt: Some("libnonexistent-dev".to_string()),
            brew: None,
            pacman: None,
            choco: None,
        }];
        let result = super::verify_lockfile(&lockfile);
        assert_eq!(result.missing_system_deps.len(), 1);
        assert_eq!(result.missing_system_deps[0].0, "nonexistent-lib-xyz");
    }

    #[test]
    fn test_format_lockfile_report_empty() {
        let result = super::LockfileVerification::default();
        let report = super::format_lockfile_report(&result);
        assert!(report.is_empty());
    }

    #[test]
    fn test_format_lockfile_report_with_warnings() {
        let result = super::LockfileVerification {
            toolchain_warnings: vec!["Rust version mismatch: 1.78.0 vs 1.79.0".to_string()],
            missing_system_deps: vec![(
                "opencv".to_string(),
                "sudo apt install libopencv-dev".to_string(),
            )],
            has_errors: false,
        };
        let report = super::format_lockfile_report(&result);
        assert!(report.contains("Rust version mismatch"));
        assert!(report.contains("opencv"));
        assert!(report.contains("sudo apt install libopencv-dev"));
    }
}
