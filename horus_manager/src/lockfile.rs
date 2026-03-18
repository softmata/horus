//! Unified lockfile for deterministic builds (`horus.lock`).
//!
//! Tracks pinned versions for all dependency sources (registry, crates.io, PyPI)
//! alongside toolchain versions, system dependencies, and feature flags.
//! This is the single source of truth for reproducible builds across machines.
//!
//! ## Format (v4)
//!
//! ```toml
//! version = 4
//! config_hash = "sha256:..."
//! features = ["cuda", "monitor"]
//!
//! [toolchain]
//! rust = "1.78.0"
//! python = "3.12.3"
//!
//! [[package]]
//! name = "rplidar"
//! version = "1.2.0"
//! source = "registry"
//!
//! [[package]]
//! name = "serde"
//! version = "1.0.215"
//! source = "crates.io"
//! checksum = "sha256:abc..."
//!
//! [[system]]
//! name = "opencv"
//! version = "4.8.1"
//! pkg_config = "opencv4"
//! apt = "libopencv-dev"
//! brew = "opencv"
//! ```

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

/// Lockfile filename.
pub const HORUS_LOCK: &str = "horus.lock";

/// Current lockfile schema version.
const CURRENT_VERSION: u32 = 4;

/// Pinned toolchain versions for reproducible builds.
#[derive(Debug, Clone, Serialize, Deserialize, Default, PartialEq, Eq)]
pub struct ToolchainPins {
    /// Rust toolchain version (e.g., "1.78.0").
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rust: Option<String>,

    /// Python version (e.g., "3.12.3").
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub python: Option<String>,

    /// CMake version (e.g., "3.28.0"), if C++ deps exist.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cmake: Option<String>,
}

/// A pinned system dependency with cross-platform package names.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct SystemLock {
    /// Canonical name (e.g., "opencv").
    pub name: String,

    /// Pinned version (e.g., "4.8.1").
    pub version: String,

    /// pkg-config name for detection (e.g., "opencv4").
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pkg_config: Option<String>,

    /// apt package name (Debian/Ubuntu).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub apt: Option<String>,

    /// Homebrew formula name (macOS).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub brew: Option<String>,

    /// pacman package name (Arch Linux).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pacman: Option<String>,

    /// Chocolatey package name (Windows).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub choco: Option<String>,
}

/// The typed representation of a `horus.lock` file.
///
/// Tracks config hash for staleness detection, pinned package versions,
/// toolchain versions, system dependencies, and active feature flags.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HorusLockfile {
    /// Schema version (4 = unified deps + toolchain + system + features).
    pub version: u32,

    /// SHA-256 hash of the `horus.toml` config + detected imports.
    /// Used for fast staleness detection.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub config_hash: Option<String>,

    /// Pinned toolchain versions (Rust, Python, CMake).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub toolchain: Option<ToolchainPins>,

    /// Active feature flags at lock time.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub features: Vec<String>,

    /// Pinned package versions across all sources.
    #[serde(default, skip_serializing_if = "Vec::is_empty", rename = "package")]
    pub packages: Vec<LockedPackage>,

    /// Pinned system dependencies with cross-platform package names.
    #[serde(default, skip_serializing_if = "Vec::is_empty", rename = "system")]
    pub system_deps: Vec<SystemLock>,
}

/// A single pinned dependency in the lockfile.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LockedPackage {
    /// Package name.
    pub name: String,

    /// Pinned version string.
    pub version: String,

    /// Source: "registry", "crates.io", "pypi", "path", "git".
    pub source: String,

    /// Optional integrity checksum (for crates.io / registry packages).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub checksum: Option<String>,
}

impl Default for HorusLockfile {
    fn default() -> Self {
        Self::new()
    }
}

impl HorusLockfile {
    /// Create a new empty lockfile.
    pub fn new() -> Self {
        Self {
            version: CURRENT_VERSION,
            config_hash: None,
            toolchain: None,
            features: Vec::new(),
            packages: Vec::new(),
            system_deps: Vec::new(),
        }
    }

    /// Load lockfile from disk.
    pub fn load_from(path: &Path) -> Result<Self> {
        let content = fs::read_to_string(path)
            .with_context(|| format!("Failed to read lockfile: {}", path.display()))?;

        let lockfile: HorusLockfile = toml::from_str(&content)
            .with_context(|| format!("Failed to parse lockfile: {}", path.display()))?;

        anyhow::ensure!(
            lockfile.version >= 3,
            "Outdated lockfile version {} (expected 3+). Delete horus.lock and re-run to regenerate.\n\
             Note: v3 lockfiles are supported but will be upgraded to v4 on next write.",
            lockfile.version
        );

        Ok(lockfile)
    }

    /// Save lockfile to disk.
    pub fn save_to(&self, path: &Path) -> Result<()> {
        let content = toml::to_string_pretty(self).context("Failed to serialize lockfile")?;
        fs::write(path, content)
            .with_context(|| format!("Failed to write lockfile: {}", path.display()))?;
        Ok(())
    }

    /// Check if the lockfile is stale relative to a config hash.
    pub fn is_stale(&self, current_config_hash: &str) -> bool {
        match &self.config_hash {
            Some(hash) => hash != current_config_hash,
            None => true, // No hash means always stale
        }
    }

    /// Add or update a pinned package.
    pub fn pin(&mut self, name: &str, version: &str, source: &str, checksum: Option<String>) {
        // Update existing entry or insert new one
        if let Some(existing) = self
            .packages
            .iter_mut()
            .find(|p| p.name == name && p.source == source)
        {
            existing.version = version.to_string();
            existing.checksum = checksum;
        } else {
            self.packages.push(LockedPackage {
                name: name.to_string(),
                version: version.to_string(),
                source: source.to_string(),
                checksum,
            });
        }
        // Keep sorted for deterministic output
        self.packages
            .sort_by(|a, b| a.source.cmp(&b.source).then_with(|| a.name.cmp(&b.name)));
    }
}

#[cfg(test)]
impl HorusLockfile {
    /// Remove a pinned package.
    pub fn unpin(&mut self, name: &str, source: &str) {
        self.packages
            .retain(|p| !(p.name == name && p.source == source));
    }

    /// Get the pinned version for a package, if any.
    pub fn get_pinned(&self, name: &str, source: &str) -> Option<&str> {
        self.packages
            .iter()
            .find(|p| p.name == name && p.source == source)
            .map(|p| p.version.as_str())
    }

    /// Get all pinned packages for a given source.
    pub fn packages_by_source(&self, source: &str) -> Vec<&LockedPackage> {
        self.packages
            .iter()
            .filter(|p| p.source == source)
            .collect()
    }

    /// Merge pins from the manifest's resolved dependencies.
    ///
    /// Updates existing pins and adds new ones. Does not remove pins
    /// for packages no longer in the manifest (use `retain_only` for that).
    pub fn merge_pins(&mut self, pins: &[(String, String, String)]) {
        for (name, version, source) in pins {
            self.pin(name, version, source, None);
        }
    }

    /// Retain only packages that are in the given set (name, source).
    /// Removes pins for packages no longer declared in horus.toml.
    pub fn retain_only(&mut self, keep: &[(String, String)]) {
        self.packages.retain(|p| {
            keep.iter()
                .any(|(name, source)| p.name == *name && p.source == *source)
        });
    }
}

/// Compute a SHA-256 hash of the horus.toml config for staleness detection.
pub fn hash_config(config_content: &str) -> String {
    use sha2::{Digest, Sha256};
    let mut hasher = Sha256::new();
    hasher.update(config_content.as_bytes());
    format!("{:x}", hasher.finalize())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_lockfile_defaults() {
        let lock = HorusLockfile::new();
        assert_eq!(lock.version, CURRENT_VERSION);
        assert!(lock.config_hash.is_none());
        assert!(lock.packages.is_empty());
    }

    #[test]
    fn lockfile_roundtrip() {
        let mut lock = HorusLockfile::new();
        lock.config_hash = Some("deadbeef".to_string());
        lock.pin(
            "serde",
            "1.0.215",
            "crates.io",
            Some("sha256:abc".to_string()),
        );
        lock.pin("numpy", "1.26.4", "pypi", None);
        lock.pin("rplidar", "1.2.0", "registry", None);

        let serialized = toml::to_string_pretty(&lock).unwrap();
        let deserialized: HorusLockfile = toml::from_str(&serialized).unwrap();
        assert_eq!(deserialized.version, CURRENT_VERSION);
        assert_eq!(deserialized.config_hash, Some("deadbeef".to_string()));
        assert_eq!(deserialized.packages.len(), 3);
    }

    #[test]
    fn staleness_detection() {
        let lock = HorusLockfile {
            version: CURRENT_VERSION,
            config_hash: Some("abc123".to_string()),
            toolchain: None,
            features: Vec::new(),
            packages: Vec::new(),
            system_deps: Vec::new(),
        };
        assert!(!lock.is_stale("abc123"));
        assert!(lock.is_stale("different"));

        let no_hash = HorusLockfile::new();
        assert!(no_hash.is_stale("anything"));
    }

    #[test]
    fn save_and_load_file() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("horus.lock");

        let mut lock = HorusLockfile::new();
        lock.config_hash = Some("hash123".to_string());
        lock.pin("serde", "1.0.0", "crates.io", None);

        lock.save_to(&path).unwrap();
        let loaded = HorusLockfile::load_from(&path).unwrap();
        assert_eq!(loaded.version, CURRENT_VERSION);
        assert_eq!(loaded.config_hash, Some("hash123".to_string()));
        assert_eq!(loaded.packages.len(), 1);
        assert_eq!(loaded.packages[0].name, "serde");
    }

    #[test]
    fn load_v1_lockfile_is_rejected() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("horus.lock");

        let legacy_content = "version = 1\nmanifest_hash = \"oldhash\"\n";
        fs::write(&path, legacy_content).unwrap();

        let result = HorusLockfile::load_from(&path);
        assert!(result.is_err());
        let err_msg = result.unwrap_err().to_string();
        assert!(err_msg.contains("Outdated lockfile version"));
    }

    #[test]
    fn load_v2_lockfile_is_rejected() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("horus.lock");

        let v2_content = "version = 2\nconfig_hash = \"abc\"\n";
        fs::write(&path, v2_content).unwrap();

        let result = HorusLockfile::load_from(&path);
        assert!(result.is_err());
        let err_msg = result.unwrap_err().to_string();
        assert!(err_msg.contains("Outdated lockfile version"));
    }

    #[test]
    fn hash_config_deterministic() {
        let content = "[package]\nname = \"test\"\nversion = \"0.1.0\"\n";
        let hash1 = hash_config(content);
        let hash2 = hash_config(content);
        assert_eq!(hash1, hash2);
    }

    #[test]
    fn hash_config_differs_on_change() {
        let hash1 = hash_config("version = 1");
        let hash2 = hash_config("version = 2");
        assert_ne!(hash1, hash2);
    }

    #[test]
    fn pin_and_get() {
        let mut lock = HorusLockfile::new();
        lock.pin("serde", "1.0.0", "crates.io", None);

        assert_eq!(lock.get_pinned("serde", "crates.io"), Some("1.0.0"));
        assert_eq!(lock.get_pinned("serde", "pypi"), None);
        assert_eq!(lock.get_pinned("nonexistent", "crates.io"), None);
    }

    #[test]
    fn pin_update_existing() {
        let mut lock = HorusLockfile::new();
        lock.pin("serde", "1.0.0", "crates.io", None);
        lock.pin(
            "serde",
            "1.0.215",
            "crates.io",
            Some("sha256:xyz".to_string()),
        );

        assert_eq!(lock.packages.len(), 1);
        assert_eq!(lock.packages[0].version, "1.0.215");
        assert_eq!(lock.packages[0].checksum, Some("sha256:xyz".to_string()));
    }

    #[test]
    fn unpin() {
        let mut lock = HorusLockfile::new();
        lock.pin("serde", "1.0.0", "crates.io", None);
        lock.pin("numpy", "1.26.4", "pypi", None);

        lock.unpin("serde", "crates.io");
        assert_eq!(lock.packages.len(), 1);
        assert_eq!(lock.packages[0].name, "numpy");
    }

    #[test]
    fn packages_sorted_by_source_then_name() {
        let mut lock = HorusLockfile::new();
        lock.pin("tokio", "1.0", "crates.io", None);
        lock.pin("serde", "1.0", "crates.io", None);
        lock.pin("numpy", "1.26", "pypi", None);
        lock.pin("rplidar", "1.2", "registry", None);

        assert_eq!(lock.packages[0].name, "serde"); // crates.io, s
        assert_eq!(lock.packages[1].name, "tokio"); // crates.io, t
        assert_eq!(lock.packages[2].name, "numpy"); // pypi
        assert_eq!(lock.packages[3].name, "rplidar"); // registry
    }

    #[test]
    fn packages_by_source_filter() {
        let mut lock = HorusLockfile::new();
        lock.pin("serde", "1.0", "crates.io", None);
        lock.pin("tokio", "1.0", "crates.io", None);
        lock.pin("numpy", "1.26", "pypi", None);

        let crates = lock.packages_by_source("crates.io");
        assert_eq!(crates.len(), 2);

        let pypi = lock.packages_by_source("pypi");
        assert_eq!(pypi.len(), 1);

        let registry = lock.packages_by_source("registry");
        assert!(registry.is_empty());
    }

    #[test]
    fn retain_only_keeps_matching() {
        let mut lock = HorusLockfile::new();
        lock.pin("serde", "1.0", "crates.io", None);
        lock.pin("tokio", "1.0", "crates.io", None);
        lock.pin("numpy", "1.26", "pypi", None);

        lock.retain_only(&[
            ("serde".to_string(), "crates.io".to_string()),
            ("numpy".to_string(), "pypi".to_string()),
        ]);

        assert_eq!(lock.packages.len(), 2);
        assert!(lock.get_pinned("tokio", "crates.io").is_none());
        assert!(lock.get_pinned("serde", "crates.io").is_some());
    }

    #[test]
    fn merge_pins_adds_and_updates() {
        let mut lock = HorusLockfile::new();
        lock.pin("serde", "1.0.0", "crates.io", None);

        lock.merge_pins(&[
            (
                "serde".to_string(),
                "1.0.215".to_string(),
                "crates.io".to_string(),
            ),
            (
                "numpy".to_string(),
                "1.26.4".to_string(),
                "pypi".to_string(),
            ),
        ]);

        assert_eq!(lock.packages.len(), 2);
        assert_eq!(lock.get_pinned("serde", "crates.io"), Some("1.0.215"));
        assert_eq!(lock.get_pinned("numpy", "pypi"), Some("1.26.4"));
    }

    // --- v4 lockfile tests ---

    #[test]
    fn v4_lockfile_roundtrip() {
        let mut lock = HorusLockfile::new();
        lock.config_hash = Some("deadbeef".to_string());
        lock.toolchain = Some(ToolchainPins {
            rust: Some("1.78.0".to_string()),
            python: Some("3.12.3".to_string()),
            cmake: None,
        });
        lock.features = vec!["cuda".to_string(), "monitor".to_string()];
        lock.pin(
            "serde",
            "1.0.215",
            "crates.io",
            Some("sha256:abc".to_string()),
        );
        lock.system_deps = vec![SystemLock {
            name: "opencv".to_string(),
            version: "4.8.1".to_string(),
            pkg_config: Some("opencv4".to_string()),
            apt: Some("libopencv-dev".to_string()),
            brew: Some("opencv".to_string()),
            pacman: Some("opencv".to_string()),
            choco: None,
        }];

        let serialized = toml::to_string_pretty(&lock).unwrap();
        let deserialized: HorusLockfile = toml::from_str(&serialized).unwrap();

        assert_eq!(deserialized.version, 4);
        assert_eq!(
            deserialized.toolchain.as_ref().unwrap().rust,
            Some("1.78.0".to_string())
        );
        assert_eq!(
            deserialized.toolchain.as_ref().unwrap().python,
            Some("3.12.3".to_string())
        );
        assert_eq!(deserialized.toolchain.as_ref().unwrap().cmake, None);
        assert_eq!(deserialized.features, vec!["cuda", "monitor"]);
        assert_eq!(deserialized.packages.len(), 1);
        assert_eq!(deserialized.system_deps.len(), 1);
        assert_eq!(deserialized.system_deps[0].name, "opencv");
        assert_eq!(
            deserialized.system_deps[0].apt,
            Some("libopencv-dev".to_string())
        );
        assert_eq!(deserialized.system_deps[0].brew, Some("opencv".to_string()));
    }

    #[test]
    fn v3_lockfile_backward_compat() {
        // A v3 lockfile has no toolchain, features, or system_deps sections.
        // It should parse successfully with defaults.
        let v3_content = r#"
version = 3
config_hash = "abc123"

[[package]]
name = "serde"
version = "1.0.215"
source = "crates.io"
"#;
        let lock: HorusLockfile = toml::from_str(v3_content).unwrap();
        assert_eq!(lock.version, 3);
        assert!(lock.toolchain.is_none());
        assert!(lock.features.is_empty());
        assert!(lock.system_deps.is_empty());
        assert_eq!(lock.packages.len(), 1);
        assert_eq!(lock.packages[0].name, "serde");
    }

    #[test]
    fn v4_save_and_load_file() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("horus.lock");

        let mut lock = HorusLockfile::new();
        lock.toolchain = Some(ToolchainPins {
            rust: Some("1.78.0".to_string()),
            python: None,
            cmake: None,
        });
        lock.features = vec!["monitor".to_string()];
        lock.system_deps = vec![SystemLock {
            name: "libudev".to_string(),
            version: "252".to_string(),
            pkg_config: Some("libudev".to_string()),
            apt: Some("libudev-dev".to_string()),
            brew: None,
            pacman: Some("systemd-libs".to_string()),
            choco: None,
        }];
        lock.pin("serde", "1.0.0", "crates.io", None);

        lock.save_to(&path).unwrap();
        let loaded = HorusLockfile::load_from(&path).unwrap();

        assert_eq!(loaded.version, 4);
        assert_eq!(
            loaded.toolchain.as_ref().unwrap().rust,
            Some("1.78.0".to_string())
        );
        assert_eq!(loaded.features, vec!["monitor"]);
        assert_eq!(loaded.system_deps.len(), 1);
        assert_eq!(loaded.system_deps[0].name, "libudev");
        assert_eq!(
            loaded.system_deps[0].pacman,
            Some("systemd-libs".to_string())
        );
        assert_eq!(loaded.packages.len(), 1);
    }

    #[test]
    fn toolchain_pins_default() {
        let pins = ToolchainPins::default();
        assert!(pins.rust.is_none());
        assert!(pins.python.is_none());
        assert!(pins.cmake.is_none());
    }

    #[test]
    fn toolchain_pins_roundtrip() {
        let pins = ToolchainPins {
            rust: Some("1.78.0".to_string()),
            python: Some("3.12.3".to_string()),
            cmake: Some("3.28.0".to_string()),
        };
        let serialized = toml::to_string_pretty(&pins).unwrap();
        let deserialized: ToolchainPins = toml::from_str(&serialized).unwrap();
        assert_eq!(pins, deserialized);
    }

    #[test]
    fn system_lock_minimal() {
        let lock = SystemLock {
            name: "cuda".to_string(),
            version: "12.4".to_string(),
            pkg_config: None,
            apt: Some("nvidia-cuda-toolkit".to_string()),
            brew: None,
            pacman: None,
            choco: None,
        };
        let serialized = toml::to_string_pretty(&lock).unwrap();
        assert!(serialized.contains("nvidia-cuda-toolkit"));
        assert!(!serialized.contains("brew"));
        assert!(!serialized.contains("pacman"));
        assert!(!serialized.contains("choco"));

        let deserialized: SystemLock = toml::from_str(&serialized).unwrap();
        assert_eq!(lock, deserialized);
    }
}
