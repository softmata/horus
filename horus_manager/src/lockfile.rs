//! Lockfile system for deterministic builds (`horus.lock`).
//!
//! With the manifest V2 redesign, dependencies live in native build files
//! (`Cargo.toml` / `Cargo.lock` for Rust, `pyproject.toml` for Python).
//! The horus lockfile now only tracks whether the horus config state has
//! changed, so that dependency resolution can be skipped when nothing changed.
//!
//! ## Format
//!
//! ```toml
//! version = 2
//! config_hash = "sha256:..."
//! ```

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

/// Lockfile filename.
pub const HORUS_LOCK: &str = "horus.lock";

/// The typed representation of a `horus.lock` file.
///
/// Tracks whether the horus config has changed since the last resolution.
/// Actual dependency locking is delegated to native tools (Cargo.lock, etc.).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HorusLockfile {
    /// Schema version (2 = config-only).
    pub version: u32,

    /// SHA-256 hash of the `horus.toml` config file.
    /// Used for fast staleness detection.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub config_hash: Option<String>,
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
            version: 2,
            config_hash: None,
        }
    }

    /// Load lockfile from disk.
    pub fn load_from(path: &Path) -> Result<Self> {
        let content = fs::read_to_string(path)
            .with_context(|| format!("Failed to read lockfile: {}", path.display()))?;

        let lockfile: HorusLockfile = toml::from_str(&content)
            .with_context(|| format!("Failed to parse lockfile: {}", path.display()))?;

        anyhow::ensure!(
            lockfile.version >= 2,
            "Unsupported lockfile version {}. Delete horus.lock and re-run to regenerate.",
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
        assert_eq!(lock.version, 2);
        assert!(lock.config_hash.is_none());
    }

    #[test]
    fn lockfile_roundtrip() {
        let lock = HorusLockfile {
            version: 2,
            config_hash: Some("deadbeef".to_string()),
        };

        let serialized = toml::to_string_pretty(&lock).unwrap();
        let deserialized: HorusLockfile = toml::from_str(&serialized).unwrap();
        assert_eq!(deserialized.version, 2);
        assert_eq!(deserialized.config_hash, Some("deadbeef".to_string()));
    }

    #[test]
    fn staleness_detection() {
        let lock = HorusLockfile {
            version: 2,
            config_hash: Some("abc123".to_string()),
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

        let lock = HorusLockfile {
            version: 2,
            config_hash: Some("hash123".to_string()),
        };

        lock.save_to(&path).unwrap();
        let loaded = HorusLockfile::load_from(&path).unwrap();
        assert_eq!(loaded.version, 2);
        assert_eq!(loaded.config_hash, Some("hash123".to_string()));
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
        assert!(err_msg.contains("Unsupported lockfile version"));
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
}
