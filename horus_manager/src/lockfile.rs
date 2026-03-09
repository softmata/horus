//! Lockfile system for deterministic builds (`horus.lock`).
//!
//! The lockfile records exact resolved versions, checksums, and sources for
//! every transitive dependency. It lives next to `horus.toml` and should be
//! committed to version control.
//!
//! ## Format
//!
//! ```toml
//! version = 1
//!
//! [[package]]
//! name = "numpy"
//! version = "1.24.3"
//! source = "pypi"
//! checksum = "sha256:abc123..."
//!
//! [[package]]
//! name = "horus_library"
//! version = "0.2.1"
//! source = "registry"
//! checksum = "sha256:def456..."
//! dependencies = ["horus_core@0.2.1"]
//! ```

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

/// Lockfile filename.
pub const HORUS_LOCK: &str = "horus.lock";

/// The typed representation of a `horus.lock` file.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HorusLockfile {
    /// Schema version (currently 1).
    pub version: u32,

    /// All locked packages, sorted by name for deterministic output.
    #[serde(default, rename = "package")]
    pub packages: Vec<LockedPackage>,

    /// SHA-256 hash of the `[dependencies]` section from `horus.toml`.
    /// Used for fast staleness detection.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub manifest_hash: Option<String>,
}

/// A single locked dependency entry.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct LockedPackage {
    /// Package name.
    pub name: String,

    /// Exact resolved version.
    pub version: String,

    /// Source identifier.
    #[serde(default = "default_source")]
    pub source: String,

    /// Content hash for verification.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub checksum: Option<String>,

    /// Direct dependencies as `["name@version", ...]`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub dependencies: Vec<String>,
}

fn default_source() -> String {
    "registry".to_string()
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
            version: 1,
            packages: Vec::new(),
            manifest_hash: None,
        }
    }

    /// Load lockfile from disk.
    pub fn load_from(path: &Path) -> Result<Self> {
        let content = fs::read_to_string(path)
            .with_context(|| format!("Failed to read lockfile: {}", path.display()))?;
        let lockfile: HorusLockfile = toml::from_str(&content)
            .with_context(|| format!("Failed to parse lockfile: {}", path.display()))?;
        Ok(lockfile)
    }

    /// Save lockfile to disk in deterministic order.
    pub fn save_to(&self, path: &Path) -> Result<()> {
        let mut sorted = self.clone();
        sorted.packages.sort_by(|a, b| a.name.cmp(&b.name));

        let content = toml::to_string_pretty(&sorted)
            .context("Failed to serialize lockfile")?;
        fs::write(path, content)
            .with_context(|| format!("Failed to write lockfile: {}", path.display()))?;
        Ok(())
    }

    /// Check if the lockfile is stale relative to a manifest hash.
    pub fn is_stale(&self, current_manifest_hash: &str) -> bool {
        match &self.manifest_hash {
            Some(hash) => hash != current_manifest_hash,
            None => true, // No hash means always stale
        }
    }

    /// Find a locked package by name.
    pub fn find_package(&self, name: &str) -> Option<&LockedPackage> {
        self.packages.iter().find(|p| p.name == name)
    }

    /// Get all locked packages as (name, version) pairs for installation.
    pub fn pinned_versions(&self) -> BTreeMap<String, String> {
        self.packages
            .iter()
            .map(|p| (p.name.clone(), p.version.clone()))
            .collect()
    }
}

/// Compute a SHA-256 hash of the dependencies section for staleness detection.
pub fn hash_manifest_deps(manifest_content: &str) -> String {
    use sha2::{Digest, Sha256};
    let mut hasher = Sha256::new();
    // Hash just the dependency-related lines for change detection
    // This is a simplified approach — hash the entire file for now
    hasher.update(manifest_content.as_bytes());
    format!("{:x}", hasher.finalize())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_lockfile_is_empty() {
        let lock = HorusLockfile::new();
        assert_eq!(lock.version, 1);
        assert!(lock.packages.is_empty());
    }

    #[test]
    fn lockfile_roundtrip() {
        let lock = HorusLockfile {
            version: 1,
            packages: vec![
                LockedPackage {
                    name: "horus".to_string(),
                    version: "0.2.1".to_string(),
                    source: "registry".to_string(),
                    checksum: Some("sha256:abc123".to_string()),
                    dependencies: vec![],
                },
                LockedPackage {
                    name: "numpy".to_string(),
                    version: "1.24.3".to_string(),
                    source: "pypi".to_string(),
                    checksum: None,
                    dependencies: vec![],
                },
            ],
            manifest_hash: Some("deadbeef".to_string()),
        };

        let serialized = toml::to_string_pretty(&lock).unwrap();
        let deserialized: HorusLockfile = toml::from_str(&serialized).unwrap();
        assert_eq!(deserialized.packages.len(), 2);
        assert_eq!(deserialized.manifest_hash, Some("deadbeef".to_string()));
    }

    #[test]
    fn staleness_detection() {
        let lock = HorusLockfile {
            version: 1,
            packages: vec![],
            manifest_hash: Some("abc123".to_string()),
        };
        assert!(!lock.is_stale("abc123"));
        assert!(lock.is_stale("different"));

        let no_hash = HorusLockfile::new();
        assert!(no_hash.is_stale("anything"));
    }

    #[test]
    fn pinned_versions_map() {
        let lock = HorusLockfile {
            version: 1,
            packages: vec![
                LockedPackage {
                    name: "a".to_string(),
                    version: "1.0.0".to_string(),
                    source: "registry".to_string(),
                    checksum: None,
                    dependencies: vec![],
                },
                LockedPackage {
                    name: "b".to_string(),
                    version: "2.0.0".to_string(),
                    source: "pypi".to_string(),
                    checksum: None,
                    dependencies: vec![],
                },
            ],
            manifest_hash: None,
        };

        let pinned = lock.pinned_versions();
        assert_eq!(pinned.get("a"), Some(&"1.0.0".to_string()));
        assert_eq!(pinned.get("b"), Some(&"2.0.0".to_string()));
    }

    #[test]
    fn save_and_load_file() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("horus.lock");

        let lock = HorusLockfile {
            version: 1,
            packages: vec![LockedPackage {
                name: "test".to_string(),
                version: "1.0.0".to_string(),
                source: "registry".to_string(),
                checksum: Some("sha256:test".to_string()),
                dependencies: vec!["dep@0.1.0".to_string()],
            }],
            manifest_hash: Some("hash123".to_string()),
        };

        lock.save_to(&path).unwrap();
        let loaded = HorusLockfile::load_from(&path).unwrap();
        assert_eq!(loaded.packages.len(), 1);
        assert_eq!(loaded.packages[0].name, "test");
        assert_eq!(loaded.packages[0].dependencies, vec!["dep@0.1.0"]);
    }

    #[test]
    fn deterministic_output() {
        let lock = HorusLockfile {
            version: 1,
            packages: vec![
                LockedPackage {
                    name: "zeta".to_string(),
                    version: "1.0.0".to_string(),
                    source: "registry".to_string(),
                    checksum: None,
                    dependencies: vec![],
                },
                LockedPackage {
                    name: "alpha".to_string(),
                    version: "2.0.0".to_string(),
                    source: "registry".to_string(),
                    checksum: None,
                    dependencies: vec![],
                },
            ],
            manifest_hash: None,
        };

        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("horus.lock");
        lock.save_to(&path).unwrap();

        let content = fs::read_to_string(&path).unwrap();
        let alpha_pos = content.find("alpha").unwrap();
        let zeta_pos = content.find("zeta").unwrap();
        assert!(alpha_pos < zeta_pos, "Packages should be sorted alphabetically");
    }
}
