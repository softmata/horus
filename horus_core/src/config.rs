//! HORUS Project Configuration
//!
//! This module provides typed configuration for `horus.yaml` project files.
//! It includes settings for drivers, plugins, scheduling, and more.
//!
//! # Example horus.yaml
//!
//! ```yaml
//! name: my-robot
//! version: 0.1.0
//!
//! # Driver mode: static, dynamic, or hybrid
//! driver_mode: hybrid
//!
//! # Plugin configuration
//! plugins:
//!   search_paths:
//!     - ~/.horus/drivers
//!     - /usr/local/lib/horus/drivers
//!   static:  # Drivers to compile in (hybrid mode)
//!     - imu
//!     - motor
//!   dynamic:  # Drivers to load at runtime (hybrid mode)
//!     - camera-fancy
//!     - lidar-velodyne
//!
//! # Driver settings
//! drivers:
//!   - camera
//!   - lidar
//!   - imu
//!
//! # Backend overrides
//! backends:
//!   imu: mpu6050
//!   lidar: rplidar-a2
//!
//! # Dependencies
//! dependencies:
//!   - numpy@latest
//!   - opencv@4.8
//! ```

use crate::plugin::{DriverLoaderConfig, DriverMode};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::PathBuf;

/// HORUS project configuration from horus.yaml
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct ProjectConfig {
    /// Project name
    pub name: Option<String>,

    /// Project version
    pub version: Option<String>,

    /// Project description
    pub description: Option<String>,

    /// Driver loading mode: static, dynamic, or hybrid
    #[serde(default)]
    pub driver_mode: DriverMode,

    /// Plugin configuration
    #[serde(default)]
    pub plugins: PluginConfig,

    /// List of drivers to enable
    #[serde(default)]
    pub drivers: Vec<String>,

    /// Backend overrides (driver_id -> backend_id)
    #[serde(default)]
    pub backends: HashMap<String, String>,

    /// Dependencies
    #[serde(default)]
    pub dependencies: Vec<DependencyEntry>,

    /// Scheduling configuration
    #[serde(default)]
    pub scheduling: Option<SchedulingConfig>,

    /// Network configuration
    #[serde(default)]
    pub network: Option<NetworkConfig>,

    /// Extra configuration (for custom keys)
    #[serde(flatten)]
    pub extra: HashMap<String, serde_yaml::Value>,
}

/// Plugin configuration section
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct PluginConfig {
    /// Search paths for dynamic plugins
    #[serde(default)]
    pub search_paths: Vec<PathBuf>,

    /// Drivers to load statically (for hybrid mode)
    #[serde(default, rename = "static")]
    pub static_drivers: Vec<String>,

    /// Drivers to load dynamically (for hybrid mode)
    #[serde(default, rename = "dynamic")]
    pub dynamic_drivers: Vec<String>,

    /// Whether to auto-discover drivers in search paths
    #[serde(default = "default_true")]
    pub auto_discover: bool,
}

fn default_true() -> bool {
    true
}

impl From<PluginConfig> for DriverLoaderConfig {
    fn from(config: PluginConfig) -> Self {
        DriverLoaderConfig {
            mode: DriverMode::default(),
            search_paths: config.search_paths,
            static_drivers: config.static_drivers,
            dynamic_drivers: config.dynamic_drivers,
            auto_discover: config.auto_discover,
        }
    }
}

/// Dependency entry in horus.yaml
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum DependencyEntry {
    /// Simple string format: "package@version"
    Simple(String),

    /// Structured format with path or features
    Structured {
        name: Option<String>,
        version: Option<String>,
        path: Option<PathBuf>,
        features: Option<Vec<String>>,
    },
}

impl DependencyEntry {
    /// Get the package name from the entry
    pub fn package_name(&self) -> Option<&str> {
        match self {
            DependencyEntry::Simple(s) => {
                // Parse "package@version" or just "package"
                s.split('@').next()
            }
            DependencyEntry::Structured { name, .. } => name.as_deref(),
        }
    }

    /// Get the version from the entry
    pub fn version(&self) -> Option<&str> {
        match self {
            DependencyEntry::Simple(s) => {
                // Parse "package@version"
                let parts: Vec<&str> = s.split('@').collect();
                if parts.len() >= 2 {
                    Some(parts[1])
                } else {
                    None
                }
            }
            DependencyEntry::Structured { version, .. } => version.as_deref(),
        }
    }
}

/// Scheduling configuration
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct SchedulingConfig {
    /// Scheduler type: realtime, cooperative, etc.
    pub scheduler: Option<String>,

    /// Target frequency in Hz
    pub frequency: Option<f64>,

    /// CPU pinning settings
    pub cpu_affinity: Option<Vec<usize>>,

    /// Priority settings
    pub priority: Option<i32>,
}

/// Network configuration
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct NetworkConfig {
    /// Transport type: shm, tcp, udp, quic
    pub transport: Option<String>,

    /// Bind address
    pub bind: Option<String>,

    /// Remote hosts
    pub remotes: Option<Vec<String>>,

    /// Enable TLS
    pub tls: Option<bool>,
}

impl ProjectConfig {
    /// Load configuration from a YAML string
    pub fn from_yaml(content: &str) -> Result<Self, serde_yaml::Error> {
        serde_yaml::from_str(content)
    }

    /// Load configuration from a file
    pub fn from_file(path: &std::path::Path) -> std::io::Result<Self> {
        let content = std::fs::read_to_string(path)?;
        serde_yaml::from_str(&content)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
    }

    /// Convert to a DriverLoaderConfig
    pub fn to_driver_loader_config(&self) -> DriverLoaderConfig {
        DriverLoaderConfig {
            mode: self.driver_mode,
            search_paths: self.plugins.search_paths.clone(),
            static_drivers: self.plugins.static_drivers.clone(),
            dynamic_drivers: self.plugins.dynamic_drivers.clone(),
            auto_discover: self.plugins.auto_discover,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_config() {
        let yaml = r#"
name: test-robot
version: 0.1.0
driver_mode: static
drivers:
  - camera
  - lidar
"#;
        let config: ProjectConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.name, Some("test-robot".to_string()));
        assert_eq!(config.driver_mode, DriverMode::Static);
        assert_eq!(config.drivers, vec!["camera", "lidar"]);
    }

    #[test]
    fn test_parse_hybrid_config() {
        let yaml = r#"
name: test-robot
driver_mode: hybrid
plugins:
  search_paths:
    - ~/.horus/drivers
  static:
    - imu
    - motor
  dynamic:
    - camera-fancy
backends:
  imu: mpu6050
"#;
        let config: ProjectConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.driver_mode, DriverMode::Hybrid);
        assert_eq!(config.plugins.static_drivers, vec!["imu", "motor"]);
        assert_eq!(config.plugins.dynamic_drivers, vec!["camera-fancy"]);
        assert_eq!(config.backends.get("imu"), Some(&"mpu6050".to_string()));
    }

    #[test]
    fn test_parse_dependencies() {
        let yaml = r#"
dependencies:
  - numpy@1.24
  - name: opencv
    version: "4.8"
    features:
      - highgui
"#;
        let config: ProjectConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.dependencies.len(), 2);

        assert_eq!(config.dependencies[0].package_name(), Some("numpy"));
        assert_eq!(config.dependencies[0].version(), Some("1.24"));

        assert_eq!(config.dependencies[1].package_name(), Some("opencv"));
    }

    #[test]
    fn test_default_driver_mode() {
        let yaml = r#"
name: minimal
"#;
        let config: ProjectConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.driver_mode, DriverMode::Static);
    }

    #[test]
    fn test_to_driver_loader_config() {
        let yaml = r#"
driver_mode: hybrid
plugins:
  search_paths:
    - /custom/path
  static:
    - imu
  auto_discover: false
"#;
        let config: ProjectConfig = serde_yaml::from_str(yaml).unwrap();
        let loader_config = config.to_driver_loader_config();

        assert_eq!(loader_config.mode, DriverMode::Hybrid);
        assert_eq!(
            loader_config.search_paths,
            vec![PathBuf::from("/custom/path")]
        );
        assert_eq!(loader_config.static_drivers, vec!["imu"]);
        assert!(!loader_config.auto_discover);
    }
}
