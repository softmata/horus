//! Plugin Discovery - Find and install HORUS plugins
//!
//! This module provides discovery of available plugins from:
//! - Local workspace (horus-* crates in development)
//! - SOFTMATA registry (published plugins)
//! - crates.io (community plugins)
//!
//! Plugins are runtime-loadable extensions that add hardware support,
//! new nodes, or CLI commands without recompilation.

use anyhow::{anyhow, Context, Result};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

/// Available plugin from discovery
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AvailablePlugin {
    /// Plugin package name (e.g., "horus-realsense")
    pub name: String,
    /// Latest version available
    pub version: String,
    /// Short description
    pub description: String,
    /// Plugin category
    pub category: PluginCategory,
    /// Source where plugin is available
    pub source: PluginSourceType,
    /// Supported platforms
    pub platforms: Vec<String>,
    /// HORUS version compatibility
    pub horus_compat: String,
    /// Whether pre-built binaries are available
    pub has_prebuilt: bool,
    /// Required system dependencies
    pub system_deps: Vec<String>,
    /// Features/capabilities provided
    pub features: Vec<String>,
}

/// Plugin category for organization
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum PluginCategory {
    /// Camera/vision drivers
    Camera,
    /// LiDAR/depth sensors
    Lidar,
    /// IMU/motion sensors
    Imu,
    /// Motor controllers
    Motor,
    /// Servo controllers
    Servo,
    /// Communication buses
    Bus,
    /// GPS/localization
    Gps,
    /// Force/torque sensors
    ForceTorque,
    /// Simulation backends
    Simulation,
    /// CLI extensions
    Cli,
    /// Other
    Other,
}

/// Source type for plugin
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum PluginSourceType {
    /// Local workspace (development)
    Local,
    /// SOFTMATA registry
    Registry,
    /// crates.io
    CratesIo,
    /// Git repository
    Git,
}

/// Plugin discovery service
pub struct PluginDiscovery {
    /// Local workspace paths to scan
    workspace_paths: Vec<PathBuf>,
    /// Registry base URL
    registry_url: String,
    /// Cached available plugins
    cache: HashMap<String, AvailablePlugin>,
    /// Cache timestamp
    cache_time: Option<std::time::Instant>,
}

impl Default for PluginDiscovery {
    fn default() -> Self {
        Self::new()
    }
}

impl PluginDiscovery {
    /// Create a new plugin discovery service
    pub fn new() -> Self {
        Self {
            workspace_paths: vec![],
            registry_url: "https://registry.softmata.com/api/v1".to_string(),
            cache: HashMap::new(),
            cache_time: None,
        }
    }

    /// Add a workspace path to scan for local plugins
    pub fn add_workspace_path(&mut self, path: PathBuf) {
        self.workspace_paths.push(path);
    }

    /// Set custom registry URL
    pub fn set_registry_url(&mut self, url: &str) {
        self.registry_url = url.to_string();
    }

    /// Discover all available plugins
    pub fn discover_all(&mut self) -> Result<Vec<AvailablePlugin>> {
        let mut plugins = Vec::new();

        // Discover local workspace plugins
        for path in &self.workspace_paths.clone() {
            plugins.extend(self.discover_local(path)?);
        }

        // Discover from registry (if available)
        if let Ok(registry_plugins) = self.discover_registry() {
            plugins.extend(registry_plugins);
        }

        // Update cache
        for plugin in &plugins {
            self.cache.insert(plugin.name.clone(), plugin.clone());
        }
        self.cache_time = Some(std::time::Instant::now());

        Ok(plugins)
    }

    /// Discover plugins in local workspace
    pub fn discover_local(&self, workspace_root: &Path) -> Result<Vec<AvailablePlugin>> {
        let mut plugins = Vec::new();

        // Scan for horus-* directories that are Cargo crates
        if let Ok(entries) = fs::read_dir(workspace_root) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.is_dir() {
                    if let Some(name) = path.file_name().and_then(|n| n.to_str()) {
                        if name.starts_with("horus-") {
                            if let Some(plugin) = self.parse_local_plugin(&path) {
                                plugins.push(plugin);
                            }
                        }
                    }
                }
            }
        }

        Ok(plugins)
    }

    /// Parse a local plugin from its Cargo.toml
    fn parse_local_plugin(&self, plugin_dir: &Path) -> Option<AvailablePlugin> {
        let cargo_toml = plugin_dir.join("Cargo.toml");
        if !cargo_toml.exists() {
            return None;
        }

        let content = fs::read_to_string(&cargo_toml).ok()?;
        let toml: toml::Table = content.parse().ok()?;

        let package = toml.get("package")?;
        let name = package.get("name")?.as_str()?.to_string();
        let version = package
            .get("version")
            .and_then(|v| v.as_str())
            .unwrap_or("0.1.0")
            .to_string();
        let description = package
            .get("description")
            .and_then(|v| v.as_str())
            .unwrap_or("Local HORUS plugin")
            .to_string();

        // Detect category from name or keywords
        let category = self.detect_category(&name, package);

        // Check for plugin metadata
        let features = self.extract_features(&toml);

        Some(AvailablePlugin {
            name,
            version,
            description,
            category,
            source: PluginSourceType::Local,
            platforms: vec!["linux-x86_64".to_string()],
            horus_compat: ">=0.1.0".to_string(),
            has_prebuilt: false,
            system_deps: vec![],
            features,
        })
    }

    /// Detect plugin category from name and metadata
    fn detect_category(&self, name: &str, _package: &toml::Value) -> PluginCategory {
        let name_lower = name.to_lowercase();

        if name_lower.contains("camera")
            || name_lower.contains("realsense")
            || name_lower.contains("zed")
            || name_lower.contains("oak")
        {
            PluginCategory::Camera
        } else if name_lower.contains("lidar") || name_lower.contains("rplidar") {
            PluginCategory::Lidar
        } else if name_lower.contains("imu") || name_lower.contains("bno") || name_lower.contains("mpu") {
            PluginCategory::Imu
        } else if name_lower.contains("motor") || name_lower.contains("roboclaw") {
            PluginCategory::Motor
        } else if name_lower.contains("servo") || name_lower.contains("dynamixel") {
            PluginCategory::Servo
        } else if name_lower.contains("can")
            || name_lower.contains("i2c")
            || name_lower.contains("spi")
            || name_lower.contains("modbus")
        {
            PluginCategory::Bus
        } else if name_lower.contains("gps") || name_lower.contains("nmea") {
            PluginCategory::Gps
        } else if name_lower.contains("sim") {
            PluginCategory::Simulation
        } else if name_lower.contains("sensors") {
            PluginCategory::Other // sensors is a collection
        } else if name_lower.contains("actuators") {
            PluginCategory::Motor // actuators is motor-related
        } else if name_lower.contains("industrial") {
            PluginCategory::Bus // industrial is bus-related
        } else {
            PluginCategory::Other
        }
    }

    /// Extract features from Cargo.toml
    fn extract_features(&self, toml: &toml::Table) -> Vec<String> {
        toml.get("features")
            .and_then(|f| f.as_table())
            .map(|features| features.keys().cloned().collect())
            .unwrap_or_default()
    }

    /// Discover plugins from SOFTMATA registry
    pub fn discover_registry(&self) -> Result<Vec<AvailablePlugin>> {
        // In a real implementation, this would query the registry API
        // For now, return a curated list of known plugins
        Ok(self.get_known_plugins())
    }

    /// Get list of known/official HORUS plugins
    fn get_known_plugins(&self) -> Vec<AvailablePlugin> {
        vec![
            AvailablePlugin {
                name: "horus-realsense".to_string(),
                version: "0.1.0".to_string(),
                description: "Intel RealSense depth camera support (D400, L500, T200 series)"
                    .to_string(),
                category: PluginCategory::Camera,
                source: PluginSourceType::Registry,
                platforms: vec!["linux-x86_64".to_string(), "linux-aarch64".to_string()],
                horus_compat: ">=0.1.0".to_string(),
                has_prebuilt: true,
                system_deps: vec!["librealsense2".to_string()],
                features: vec![
                    "depth".to_string(),
                    "color".to_string(),
                    "imu".to_string(),
                    "tracking".to_string(),
                ],
            },
            AvailablePlugin {
                name: "horus-zed".to_string(),
                version: "0.1.0".to_string(),
                description: "Stereolabs ZED camera support (ZED, ZED2, ZED Mini)".to_string(),
                category: PluginCategory::Camera,
                source: PluginSourceType::Registry,
                platforms: vec!["linux-x86_64".to_string()],
                horus_compat: ">=0.1.0".to_string(),
                has_prebuilt: true,
                system_deps: vec!["zed-sdk".to_string(), "cuda".to_string()],
                features: vec![
                    "depth".to_string(),
                    "tracking".to_string(),
                    "spatial-mapping".to_string(),
                ],
            },
            AvailablePlugin {
                name: "horus-rplidar".to_string(),
                version: "0.1.0".to_string(),
                description: "Slamtec RPLiDAR support (A1, A2, A3, S1, S2)".to_string(),
                category: PluginCategory::Lidar,
                source: PluginSourceType::Registry,
                platforms: vec!["linux-x86_64".to_string(), "linux-aarch64".to_string()],
                horus_compat: ">=0.1.0".to_string(),
                has_prebuilt: true,
                system_deps: vec![],
                features: vec!["2d-scan".to_string(), "motor-control".to_string()],
            },
            AvailablePlugin {
                name: "horus-velodyne".to_string(),
                version: "0.1.0".to_string(),
                description: "Velodyne 3D LiDAR support (VLP-16, VLP-32, HDL-32E, HDL-64E)"
                    .to_string(),
                category: PluginCategory::Lidar,
                source: PluginSourceType::Registry,
                platforms: vec!["linux-x86_64".to_string()],
                horus_compat: ">=0.1.0".to_string(),
                has_prebuilt: true,
                system_deps: vec![],
                features: vec!["3d-pointcloud".to_string(), "dual-return".to_string()],
            },
            AvailablePlugin {
                name: "horus-odrive".to_string(),
                version: "0.1.0".to_string(),
                description: "ODrive motor controller support (v3.x, S1, Pro)".to_string(),
                category: PluginCategory::Motor,
                source: PluginSourceType::Registry,
                platforms: vec!["linux-x86_64".to_string(), "linux-aarch64".to_string()],
                horus_compat: ">=0.1.0".to_string(),
                has_prebuilt: true,
                system_deps: vec![],
                features: vec![
                    "usb".to_string(),
                    "can".to_string(),
                    "uart".to_string(),
                    "foc".to_string(),
                ],
            },
        ]
    }

    /// Search for plugins matching a query
    pub fn search(&mut self, query: &str) -> Result<Vec<AvailablePlugin>> {
        let all = self.discover_all()?;
        let query_lower = query.to_lowercase();

        Ok(all
            .into_iter()
            .filter(|p| {
                p.name.to_lowercase().contains(&query_lower)
                    || p.description.to_lowercase().contains(&query_lower)
                    || p.features
                        .iter()
                        .any(|f| f.to_lowercase().contains(&query_lower))
            })
            .collect())
    }

    /// Search by category
    pub fn search_by_category(&mut self, category: PluginCategory) -> Result<Vec<AvailablePlugin>> {
        let all = self.discover_all()?;
        Ok(all.into_iter().filter(|p| p.category == category).collect())
    }

    /// Get plugin by name
    pub fn get_plugin(&mut self, name: &str) -> Result<Option<AvailablePlugin>> {
        // Check cache first
        if let Some(plugin) = self.cache.get(name) {
            return Ok(Some(plugin.clone()));
        }

        // Refresh and try again
        self.discover_all()?;
        Ok(self.cache.get(name).cloned())
    }

    /// Check if a plugin is compatible with current HORUS version
    pub fn is_compatible(&self, plugin: &AvailablePlugin) -> bool {
        // Parse the compatibility string (e.g., ">=0.1.0")
        let horus_version = env!("CARGO_PKG_VERSION");
        let current = match semver::Version::parse(horus_version) {
            Ok(v) => v,
            Err(_) => return true, // Assume compatible if version unparseable
        };

        // Simple compatibility check
        if plugin.horus_compat.starts_with(">=") {
            if let Ok(min) = semver::Version::parse(plugin.horus_compat.trim_start_matches(">=")) {
                return current >= min;
            }
        }

        true
    }

    /// Get installation instructions for a plugin
    pub fn get_install_instructions(&self, plugin: &AvailablePlugin) -> String {
        match plugin.source {
            PluginSourceType::Local => {
                format!(
                    "# Local development plugin\ncd {} && cargo build --release",
                    plugin.name
                )
            }
            PluginSourceType::Registry => {
                let mut instructions = format!("horus plugins install {}", plugin.name);
                if !plugin.system_deps.is_empty() {
                    instructions.push_str(&format!(
                        "\n\n# System dependencies required:\nsudo apt install {}",
                        plugin.system_deps.join(" ")
                    ));
                }
                instructions
            }
            PluginSourceType::CratesIo => {
                format!("cargo install {}", plugin.name)
            }
            PluginSourceType::Git => {
                format!("horus plugins install --git <repository-url>")
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    #[test]
    fn test_discover_local_plugin() {
        let temp_dir = TempDir::new().unwrap();
        let plugin_dir = temp_dir.path().join("horus-test");
        fs::create_dir_all(&plugin_dir).unwrap();

        let cargo_toml = r#"
[package]
name = "horus-test"
version = "0.1.0"
description = "Test plugin"

[features]
default = []
hardware = []
"#;
        fs::write(plugin_dir.join("Cargo.toml"), cargo_toml).unwrap();

        let discovery = PluginDiscovery::new();
        let plugins = discovery.discover_local(temp_dir.path()).unwrap();

        assert_eq!(plugins.len(), 1);
        assert_eq!(plugins[0].name, "horus-test");
        assert_eq!(plugins[0].source, PluginSourceType::Local);
    }

    #[test]
    fn test_category_detection() {
        let discovery = PluginDiscovery::new();
        let dummy = toml::Value::Table(toml::Table::new());

        assert_eq!(
            discovery.detect_category("horus-realsense", &dummy),
            PluginCategory::Camera
        );
        assert_eq!(
            discovery.detect_category("horus-rplidar", &dummy),
            PluginCategory::Lidar
        );
        assert_eq!(
            discovery.detect_category("horus-dynamixel", &dummy),
            PluginCategory::Servo
        );
        assert_eq!(
            discovery.detect_category("horus-sim3d", &dummy),
            PluginCategory::Simulation
        );
    }

    #[test]
    fn test_search() {
        let mut discovery = PluginDiscovery::new();
        let results = discovery.search("camera").unwrap();

        // Should find RealSense and ZED
        assert!(results.iter().any(|p| p.name.contains("realsense")));
        assert!(results.iter().any(|p| p.name.contains("zed")));
    }

    #[test]
    fn test_known_plugins() {
        let discovery = PluginDiscovery::new();
        let known = discovery.get_known_plugins();

        assert!(!known.is_empty());
        assert!(known.iter().any(|p| p.name == "horus-realsense"));
    }
}
