//! Unified lockfile for deterministic builds (`horus.lock`).
//!
//! Tracks pinned versions for all dependency sources (registry, crates.io, PyPI)
//! alongside a config hash for fast staleness detection. This replaces relying
//! solely on `Cargo.lock` / `pip freeze` with a single source of truth.
//!
//! ## Format (v3)
//!
//! ```toml
//! version = 3
//! config_hash = "sha256:..."
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
//! [[package]]
//! name = "numpy"
//! version = "1.26.4"
//! source = "pypi"
//! ```

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

/// Lockfile filename.
pub const HORUS_LOCK: &str = "horus.lock";

/// Current lockfile schema version.
const CURRENT_VERSION: u32 = 3;

/// The typed representation of a `horus.lock` file.
///
/// Tracks config hash for staleness detection and pinned package versions
/// across all dependency sources (registry, crates.io, PyPI).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HorusLockfile {
    /// Schema version (3 = unified deps).
    pub version: u32,

    /// SHA-256 hash of the `horus.toml` config + detected imports.
    /// Used for fast staleness detection.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub config_hash: Option<String>,

    /// Pinned package versions across all sources.
    #[serde(default, skip_serializing_if = "Vec::is_empty", rename = "package")]
    pub packages: Vec<LockedPackage>,
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
            packages: Vec::new(),
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
            "Outdated lockfile version {} (expected 3+). Delete horus.lock and re-run to regenerate.",
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
        self.packages.sort_by(|a, b| {
            a.source.cmp(&b.source).then_with(|| a.name.cmp(&b.name))
        });
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
        lock.pin("serde", "1.0.215", "crates.io", Some("sha256:abc".to_string()));
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
            packages: Vec::new(),
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
        lock.pin("serde", "1.0.215", "crates.io", Some("sha256:xyz".to_string()));

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
            ("serde".to_string(), "1.0.215".to_string(), "crates.io".to_string()),
            ("numpy".to_string(), "1.26.4".to_string(), "pypi".to_string()),
        ]);

        assert_eq!(lock.packages.len(), 2);
        assert_eq!(lock.get_pinned("serde", "crates.io"), Some("1.0.215"));
        assert_eq!(lock.get_pinned("numpy", "pypi"), Some("1.26.4"));
    }
}
