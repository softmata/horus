//! Plugin type definitions for HORUS driver plugins
//!
//! This module defines the core types used by the driver plugin system:
//!
//! - [`PluginManifest`]: Metadata describing a plugin
//! - [`BackendInfo`]: Information about a specific driver backend
//! - [`ProbeResult`]: Hardware detection results
//! - [`PluginHealth`]: Plugin health status
//! - [`SystemDependency`]: External system requirements

use crate::driver::DriverCategory;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Compare two version strings numerically (e.g., "1.10.0" > "1.2.0").
/// Returns negative if a < b, zero if equal, positive if a > b.
/// Falls back to lexicographic comparison for non-numeric parts.
fn compare_version_parts(a: &str, b: &str) -> i32 {
    let mut a_parts = a.split('.');
    let mut b_parts = b.split('.');
    loop {
        match (a_parts.next(), b_parts.next()) {
            (Some(ap), Some(bp)) => {
                let an = ap.parse::<u64>().unwrap_or(0);
                let bn = bp.parse::<u64>().unwrap_or(0);
                if an != bn {
                    return if an > bn { 1 } else { -1 };
                }
            }
            (Some(_), None) => return 1,
            (None, Some(_)) => return -1,
            (None, None) => return 0,
        }
    }
}

/// Unique identifier for a plugin
pub type PluginId = String;

/// Unique identifier for a backend within a plugin
pub type BackendId = String;

/// Plugin manifest containing all metadata about a driver plugin
///
/// This manifest is embedded in the plugin and provides all necessary
/// information for discovery, loading, and compatibility checking.
///
/// # Example
///
/// ```rust,ignore
/// use horus_core::plugin::{PluginManifest, BackendInfo, SystemDependency};
/// use horus_core::driver::DriverCategory;
///
/// let manifest = PluginManifest {
///     id: "horus-imu-mpu6050".into(),
///     name: "MPU6050 IMU Driver".into(),
///     version: "1.0.0".into(),
///     category: DriverCategory::Sensor,
///     description: "Driver for MPU6050 6-axis IMU".into(),
///     author: Some("HORUS Team".into()),
///     license: Some("MIT OR Apache-2.0".into()),
///     repository: Some("https://github.com/softmata/horus".into()),
///     backends: vec![BackendInfo {
///         id: "mpu6050".into(),
///         name: "MPU6050".into(),
///         description: "MPU6050 via I2C".into(),
///         hardware_ids: vec!["i2c:0x68".into(), "i2c:0x69".into()],
///     }],
///     system_deps: vec![SystemDependency {
///         name: "i2c-dev".into(),
///         kind: "kernel-module".into(),
///         required: true,
///         check_cmd: Some("lsmod | grep -q i2c_dev".into()),
///     }],
///     horus_version: ">=0.5.0, <1.0.0".into(),
///     platforms: vec!["linux".into()],
///     features: vec![],
///     extra: HashMap::new(),
/// };
/// ```
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PluginManifest {
    /// Unique plugin identifier (e.g., "horus-imu-mpu6050")
    pub id: PluginId,

    /// Human-readable name
    pub name: String,

    /// Plugin version (semver string, e.g., "1.0.0")
    pub version: String,

    /// Driver category
    pub category: DriverCategory,

    /// Brief description of the plugin
    pub description: String,

    /// Plugin author or maintainer
    #[serde(default)]
    pub author: Option<String>,

    /// License (SPDX identifier)
    #[serde(default)]
    pub license: Option<String>,

    /// Source repository URL
    #[serde(default)]
    pub repository: Option<String>,

    /// Available backends provided by this plugin
    pub backends: Vec<BackendInfo>,

    /// System dependencies required by the plugin
    #[serde(default)]
    pub system_deps: Vec<SystemDependency>,

    /// Required HORUS version range (semver requirement)
    pub horus_version: String,

    /// Supported platforms (e.g., ["linux", "macos"])
    #[serde(default = "default_platforms")]
    pub platforms: Vec<String>,

    /// Optional feature flags the plugin supports
    #[serde(default)]
    pub features: Vec<PluginFeature>,

    /// Additional metadata (for extensibility)
    #[serde(default, flatten)]
    pub extra: HashMap<String, serde_json::Value>,
}

fn default_platforms() -> Vec<String> {
    vec!["linux".into()]
}

impl PluginManifest {
    /// Check if this plugin supports the current platform
    pub fn supports_current_platform(&self) -> bool {
        let current = if cfg!(target_os = "linux") {
            "linux"
        } else if cfg!(target_os = "macos") {
            "macos"
        } else if cfg!(target_os = "windows") {
            "windows"
        } else {
            "unknown"
        };

        self.platforms.iter().any(|p| p == current || p == "any")
    }

    /// Check if this plugin is compatible with the given HORUS version
    ///
    /// This performs a simple version prefix check. For more sophisticated
    /// semver matching, use the `semver` crate externally.
    pub fn is_compatible_with(&self, horus_version: &str) -> bool {
        // Simple compatibility check:
        // - ">=X.Y.Z" requires version >= X.Y.Z
        // - "X.Y" requires version starts with X.Y
        // - "*" matches any version
        let req = self.horus_version.trim();

        if req == "*" || req.is_empty() {
            return true;
        }

        if let Some(min_version) = req.strip_prefix(">=") {
            let min = min_version.trim();
            return compare_version_parts(horus_version, min) >= 0;
        }

        if let Some(prefix) = req.strip_prefix('^') {
            // Caret: compatible with major version
            let prefix = prefix.trim();
            if let Some((major, _)) = prefix.split_once('.') {
                if let Some((v_major, _)) = horus_version.split_once('.') {
                    return major == v_major;
                }
            }
        }

        // Default: exact prefix match
        horus_version.starts_with(req.trim_start_matches('=').trim())
    }

    /// Get backend info by ID
    pub fn get_backend(&self, id: &str) -> Option<&BackendInfo> {
        self.backends.iter().find(|b| b.id == id)
    }

    /// List all backend IDs
    pub fn backend_ids(&self) -> Vec<&str> {
        self.backends.iter().map(|b| b.id.as_str()).collect()
    }
}

/// Information about a specific driver backend
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BackendInfo {
    /// Backend identifier (e.g., "mpu6050", "bno055")
    pub id: BackendId,

    /// Human-readable name
    pub name: String,

    /// Description of this backend
    pub description: String,

    /// Hardware identifiers for auto-detection
    /// Format: "bus:id" (e.g., "i2c:0x68", "usb:1234:5678", "serial:/dev/ttyUSB*")
    #[serde(default)]
    pub hardware_ids: Vec<String>,
}

/// External system dependency
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemDependency {
    /// Dependency name (e.g., "libudev", "i2c-dev")
    pub name: String,

    /// Dependency kind: "library", "kernel-module", "binary", "service"
    pub kind: String,

    /// Whether this dependency is required (vs optional/recommended)
    #[serde(default = "default_required")]
    pub required: bool,

    /// Shell command to check if dependency is available
    /// Returns exit code 0 if available
    #[serde(default)]
    pub check_cmd: Option<String>,
}

fn default_required() -> bool {
    true
}

/// Optional feature that a plugin can support
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PluginFeature {
    /// Feature name
    pub name: String,

    /// Feature description
    pub description: String,

    /// Whether this feature is enabled by default
    #[serde(default)]
    pub default: bool,
}

/// Result of probing for hardware
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProbeResult {
    /// Backend ID that detected the hardware
    pub backend_id: BackendId,

    /// Whether hardware was detected
    pub detected: bool,

    /// Detection confidence (0.0 - 1.0)
    pub confidence: f32,

    /// Device path or identifier
    #[serde(default)]
    pub device_path: Option<String>,

    /// Additional detection metadata
    #[serde(default)]
    pub metadata: HashMap<String, String>,

    /// Human-readable status message
    #[serde(default)]
    pub message: Option<String>,
}

impl ProbeResult {
    /// Create a successful probe result
    pub fn detected(backend_id: impl Into<BackendId>, device_path: impl Into<String>) -> Self {
        Self {
            backend_id: backend_id.into(),
            detected: true,
            confidence: 1.0,
            device_path: Some(device_path.into()),
            metadata: HashMap::new(),
            message: None,
        }
    }

    /// Create a probe result with confidence
    pub fn with_confidence(
        backend_id: impl Into<BackendId>,
        confidence: f32,
        device_path: Option<String>,
    ) -> Self {
        Self {
            backend_id: backend_id.into(),
            detected: confidence > 0.0,
            confidence,
            device_path,
            metadata: HashMap::new(),
            message: None,
        }
    }

    /// Create a not-detected probe result
    pub fn not_detected(backend_id: impl Into<BackendId>) -> Self {
        Self {
            backend_id: backend_id.into(),
            detected: false,
            confidence: 0.0,
            device_path: None,
            metadata: HashMap::new(),
            message: None,
        }
    }

    /// Add metadata
    pub fn with_metadata(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.metadata.insert(key.into(), value.into());
        self
    }

    /// Add status message
    pub fn with_message(mut self, message: impl Into<String>) -> Self {
        self.message = Some(message.into());
        self
    }
}

/// Health status of a loaded plugin
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PluginHealth {
    /// Plugin ID
    pub plugin_id: PluginId,

    /// Whether the plugin is healthy
    pub healthy: bool,

    /// Health status message
    pub message: String,

    /// Per-backend health status
    #[serde(default)]
    pub backend_status: HashMap<BackendId, BackendHealth>,

    /// Timestamp of health check
    #[serde(default)]
    pub checked_at: Option<u64>,
}

impl PluginHealth {
    /// Create a healthy status
    pub fn healthy(plugin_id: impl Into<PluginId>) -> Self {
        Self {
            plugin_id: plugin_id.into(),
            healthy: true,
            message: "Plugin is healthy".into(),
            backend_status: HashMap::new(),
            checked_at: Some(
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_secs(),
            ),
        }
    }

    /// Create an unhealthy status
    pub fn unhealthy(plugin_id: impl Into<PluginId>, message: impl Into<String>) -> Self {
        Self {
            plugin_id: plugin_id.into(),
            healthy: false,
            message: message.into(),
            backend_status: HashMap::new(),
            checked_at: Some(
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_secs(),
            ),
        }
    }
}

/// Health status for a specific backend
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BackendHealth {
    /// Whether the backend is operational
    pub operational: bool,

    /// Status message
    pub message: String,

    /// Number of active instances
    #[serde(default)]
    pub active_instances: u32,
}

/// Plugin loading error
#[derive(Debug, Clone)]
pub enum PluginError {
    /// Plugin not found at path
    NotFound(String),

    /// Invalid plugin format or missing symbols
    InvalidPlugin(String),

    /// Version incompatibility
    VersionMismatch { required: String, found: String },

    /// Platform not supported
    PlatformNotSupported(String),

    /// Missing system dependency
    MissingDependency(String),

    /// Failed to load dynamic library
    LoadError(String),

    /// Backend not found in plugin
    BackendNotFound(String),

    /// Driver creation failed
    CreationFailed(String),

    /// Plugin is already loaded
    AlreadyLoaded(String),
}

impl std::fmt::Display for PluginError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotFound(path) => write!(f, "Plugin not found: {}", path),
            Self::InvalidPlugin(msg) => write!(f, "Invalid plugin: {}", msg),
            Self::VersionMismatch { required, found } => {
                write!(
                    f,
                    "Version mismatch: requires {}, found {}",
                    required, found
                )
            }
            Self::PlatformNotSupported(platform) => {
                write!(f, "Platform not supported: {}", platform)
            }
            Self::MissingDependency(dep) => write!(f, "Missing dependency: {}", dep),
            Self::LoadError(msg) => write!(f, "Failed to load plugin: {}", msg),
            Self::BackendNotFound(id) => write!(f, "Backend not found: {}", id),
            Self::CreationFailed(msg) => write!(f, "Driver creation failed: {}", msg),
            Self::AlreadyLoaded(id) => write!(f, "Plugin already loaded: {}", id),
        }
    }
}

impl std::error::Error for PluginError {}

impl From<crate::error::HorusError> for PluginError {
    fn from(err: crate::error::HorusError) -> Self {
        PluginError::LoadError(err.to_string())
    }
}

/// Result type for plugin operations
pub type PluginResult<T> = Result<T, PluginError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_plugin_manifest() {
        let manifest = PluginManifest {
            id: "test-plugin".into(),
            name: "Test Plugin".into(),
            version: "1.0.0".into(),
            category: DriverCategory::Sensor,
            description: "A test plugin".into(),
            author: None,
            license: None,
            repository: None,
            backends: vec![BackendInfo {
                id: "test".into(),
                name: "Test Backend".into(),
                description: "Test".into(),
                hardware_ids: vec![],
            }],
            system_deps: vec![],
            horus_version: ">=0.5.0".into(),
            platforms: vec!["linux".into()],
            features: vec![],
            extra: HashMap::new(),
        };

        assert!(manifest.supports_current_platform());
        assert!(manifest.is_compatible_with("0.5.0"));
        assert!(manifest.is_compatible_with("0.6.0"));
        assert!(!manifest.is_compatible_with("0.4.0"));

        // Test multi-digit version comparison (would fail with string comparison)
        let manifest2 = PluginManifest {
            horus_version: ">=1.10.0".into(),
            ..manifest.clone()
        };
        assert!(manifest2.is_compatible_with("1.10.0"));
        assert!(manifest2.is_compatible_with("1.11.0"));
        assert!(manifest2.is_compatible_with("2.0.0"));
        assert!(!manifest2.is_compatible_with("1.2.0")); // 1.2 < 1.10
        assert!(!manifest2.is_compatible_with("1.9.0")); // 1.9 < 1.10
    }

    #[test]
    fn test_probe_result() {
        let detected = ProbeResult::detected("mpu6050", "/dev/i2c-1")
            .with_metadata("address", "0x68")
            .with_message("MPU6050 found");

        assert!(detected.detected);
        assert_eq!(detected.confidence, 1.0);
        assert_eq!(detected.device_path, Some("/dev/i2c-1".into()));
        assert_eq!(detected.metadata.get("address"), Some(&"0x68".into()));

        let not_detected = ProbeResult::not_detected("bno055");
        assert!(!not_detected.detected);
        assert_eq!(not_detected.confidence, 0.0);
    }

    #[test]
    fn test_plugin_health() {
        let healthy = PluginHealth::healthy("test-plugin");
        assert!(healthy.healthy);

        let unhealthy = PluginHealth::unhealthy("test-plugin", "Device disconnected");
        assert!(!unhealthy.healthy);
        assert_eq!(unhealthy.message, "Device disconnected");
    }
}
