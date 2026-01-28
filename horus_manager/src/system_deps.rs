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
    /// Feature flag name (e.g., "gpio-hardware")
    pub feature: &'static str,
    /// Human-readable description
    pub description: &'static str,
    /// System packages to install (apt package names)
    pub apt_packages: &'static [&'static str],
    /// Device files that should exist when hardware is connected
    pub device_files: &'static [&'static str],
    /// User groups required for access
    pub user_groups: &'static [&'static str],
    /// pkg-config names to check if library is installed
    pub pkg_config: &'static [&'static str],
    /// Install command for Ubuntu/Debian
    pub install_cmd: &'static str,
    /// Documentation URL
    pub docs_url: &'static str,
}

/// All system dependencies mapped by feature flag
pub const SYSTEM_DEPS: &[SystemDependency] = &[
    // GPIO Hardware (Raspberry Pi)
    SystemDependency {
        feature: "gpio-hardware",
        description: "GPIO access for motors, encoders, ultrasonic sensors",
        apt_packages: &["libraspberrypi-dev"],
        device_files: &["/sys/class/gpio", "/dev/gpiomem"],
        user_groups: &["gpio"],
        pkg_config: &[],
        install_cmd: "sudo apt install -y libraspberrypi-dev && sudo usermod -a -G gpio $USER",
        docs_url: "https://www.raspberrypi.com/documentation/computers/os.html#gpio-and-the-40-pin-header",
    },
    // I2C Hardware
    SystemDependency {
        feature: "i2c-hardware",
        description: "I2C bus for IMUs, battery monitors, displays",
        apt_packages: &["i2c-tools", "libi2c-dev"],
        device_files: &["/dev/i2c-0", "/dev/i2c-1"],
        user_groups: &["i2c"],
        pkg_config: &[],
        install_cmd: "sudo apt install -y i2c-tools libi2c-dev && sudo usermod -a -G i2c $USER",
        docs_url: "https://www.kernel.org/doc/html/latest/i2c/index.html",
    },
    // SPI Hardware
    SystemDependency {
        feature: "spi-hardware",
        description: "SPI bus for ADCs, displays, sensors",
        apt_packages: &["libraspberrypi-dev"],
        device_files: &["/dev/spidev0.0", "/dev/spidev0.1"],
        user_groups: &["spi"],
        pkg_config: &[],
        install_cmd: "sudo apt install -y libraspberrypi-dev && sudo usermod -a -G spi $USER",
        docs_url: "https://www.kernel.org/doc/html/latest/spi/index.html",
    },
    // Serial Hardware
    SystemDependency {
        feature: "serial-hardware",
        description: "Serial/UART for GPS, LiDAR, Dynamixel servos",
        apt_packages: &["libudev-dev"],
        device_files: &["/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyAMA0"],
        user_groups: &["dialout"],
        pkg_config: &["libudev"],
        install_cmd: "sudo apt install -y libudev-dev && sudo usermod -a -G dialout $USER",
        docs_url: "https://www.kernel.org/doc/html/latest/driver-api/serial/index.html",
    },
    // Modbus Hardware
    SystemDependency {
        feature: "modbus-hardware",
        description: "Modbus RTU/TCP for industrial PLCs, sensors",
        apt_packages: &["libudev-dev"],
        device_files: &[],
        user_groups: &["dialout"],
        pkg_config: &[],
        install_cmd: "sudo apt install -y libudev-dev && sudo usermod -a -G dialout $USER",
        docs_url: "https://en.wikipedia.org/wiki/Modbus",
    },
    // Joystick/Gamepad (gilrs)
    SystemDependency {
        feature: "gilrs",
        description: "Gamepad/joystick input",
        apt_packages: &["libudev-dev"],
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
        device_files: &[],
        user_groups: &[],
        pkg_config: &[],
        install_cmd: "# TFLite is bundled with the tflite crate",
        docs_url: "https://www.tensorflow.org/lite",
    },
    // BNO055 IMU
    SystemDependency {
        feature: "bno055-imu",
        description: "Bosch BNO055 9-axis IMU",
        apt_packages: &["i2c-tools", "libi2c-dev"],
        device_files: &["/dev/i2c-1"],
        user_groups: &["i2c"],
        pkg_config: &[],
        install_cmd: "sudo apt install -y i2c-tools libi2c-dev && sudo usermod -a -G i2c $USER",
        docs_url: "https://www.bosch-sensortec.com/products/motion-sensors/imus/bno055/",
    },
    // MPU6050 IMU
    SystemDependency {
        feature: "mpu6050-imu",
        description: "InvenSense MPU6050 6-axis IMU",
        apt_packages: &["i2c-tools", "libi2c-dev"],
        device_files: &["/dev/i2c-1"],
        user_groups: &["i2c"],
        pkg_config: &[],
        install_cmd: "sudo apt install -y i2c-tools libi2c-dev && sudo usermod -a -G i2c $USER",
        docs_url: "https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/",
    },
    // NMEA GPS
    SystemDependency {
        feature: "nmea-gps",
        description: "GPS receivers with NMEA protocol",
        apt_packages: &["libudev-dev"],
        device_files: &["/dev/ttyUSB0", "/dev/ttyACM0"],
        user_groups: &["dialout"],
        pkg_config: &[],
        install_cmd: "sudo apt install -y libudev-dev && sudo usermod -a -G dialout $USER",
        docs_url: "https://en.wikipedia.org/wiki/NMEA_0183",
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

/// Check if running on Raspberry Pi
pub fn is_raspberry_pi() -> bool {
    // Check /proc/device-tree/model
    std::fs::read_to_string("/proc/device-tree/model")
        .map(|s| s.to_lowercase().contains("raspberry pi"))
        .unwrap_or(false)
}

/// Check if running on x86_64 Linux desktop
pub fn is_x86_desktop() -> bool {
    cfg!(target_arch = "x86_64") && cfg!(target_os = "linux")
}

/// Get platform-appropriate default features
pub fn get_platform_defaults() -> Vec<String> {
    let mut features = vec![
        "gilrs".to_string(),           // Joystick - works everywhere with libudev
        "crossterm".to_string(),       // Keyboard - pure Rust
        "serial-hardware".to_string(), // Serial - works everywhere
    ];

    if is_raspberry_pi() {
        // On Raspberry Pi, enable GPIO-based features
        features.push("gpio-hardware".to_string());
        features.push("i2c-hardware".to_string());
        features.push("spi-hardware".to_string());
    }

    features
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
}
