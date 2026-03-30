//! Fingerprint tracking for generated build files.
//!
//! Stores SHA-256 hashes of generated files (`.horus/Cargo.toml`, etc.) so we
//! can detect when a native tool (cargo, pip, cmake) modifies them externally.

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

const FINGERPRINTS_FILE: &str = ".horus/fingerprints.json";

/// Per-file fingerprint entry.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FileFingerprint {
    /// SHA-256 hex digest of the file content when horus last generated it.
    pub hash: String,
    /// ISO 8601 timestamp of last generation.
    pub generated_at: String,
}

/// Collection of fingerprints for all managed files.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Fingerprints {
    pub version: u32,
    pub files: BTreeMap<String, FileFingerprint>,
}

impl Fingerprints {
    /// Load fingerprints from `.horus/fingerprints.json`, returning empty if missing.
    pub fn load(project_dir: &Path) -> Result<Self> {
        let path = project_dir.join(FINGERPRINTS_FILE);
        if !path.exists() {
            return Ok(Self::default());
        }
        let content = fs::read_to_string(&path).context("Failed to read fingerprints.json")?;
        serde_json::from_str(&content).context("Failed to parse fingerprints.json")
    }

    /// Save fingerprints to `.horus/fingerprints.json`.
    pub fn save(&self, project_dir: &Path) -> Result<()> {
        let path = project_dir.join(FINGERPRINTS_FILE);
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }
        let content = serde_json::to_string_pretty(self)?;
        fs::write(&path, content).context("Failed to write fingerprints.json")?;
        Ok(())
    }

    /// Record the fingerprint of generated content for a file.
    pub fn record(&mut self, filename: &str, content: &str) {
        self.version = 1;
        self.files.insert(
            filename.to_string(),
            FileFingerprint {
                hash: hash_content(content),
                generated_at: chrono::Utc::now().to_rfc3339(),
            },
        );
    }

    /// Check whether a file on disk differs from the last-generated fingerprint.
    ///
    /// Returns `true` if the file was modified externally (hash mismatch).
    /// Returns `false` if the file matches or doesn't exist on disk.
    pub fn is_modified(&self, filename: &str, project_dir: &Path) -> bool {
        let Some(fp) = self.files.get(filename) else {
            return false;
        };
        // The file lives inside .horus/
        let file_path = project_dir.join(".horus").join(filename);
        let Ok(content) = fs::read_to_string(&file_path) else {
            return false;
        };
        hash_content(&content) != fp.hash
    }
}

/// Compute SHA-256 hex digest of content.
pub fn hash_content(content: &str) -> String {
    let mut hasher = Sha256::new();
    hasher.update(content.as_bytes());
    format!("{:x}", hasher.finalize())
}

// ─── Sync lock ──────────────────────────────────────────────────────────────

/// RAII guard for advisory file lock on `.horus/.sync.lock`.
///
/// Prevents concurrent proxy instances from overwriting each other's
/// horus.toml changes. The lock is released when the guard is dropped.
/// Uses `horus_sys::fs::FileLock` for cross-platform support (Unix flock / Windows LockFileEx).
pub struct SyncLock {
    _lock: horus_sys::fs::FileLock,
}

impl SyncLock {
    /// Try to acquire the sync lock. Returns `None` if the lock is held by
    /// another process after `timeout` duration.
    pub fn try_acquire(project_dir: &Path, timeout: std::time::Duration) -> Option<Self> {
        let lock_path = project_dir.join(".horus/.sync.lock");
        if let Some(parent) = lock_path.parent() {
            let _ = fs::create_dir_all(parent);
        }

        let file = std::fs::OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(false)
            .open(&lock_path)
            .ok()?;

        // Try non-blocking first
        if let Some(lock) = horus_sys::fs::FileLock::try_exclusive(&file).ok().flatten() {
            return Some(Self { _lock: lock });
        }

        // Retry with backoff up to timeout
        let start = std::time::Instant::now();
        while start.elapsed() < timeout {
            std::thread::sleep(std::time::Duration::from_millis(50));
            if let Some(lock) = horus_sys::fs::FileLock::try_exclusive(&file).ok().flatten() {
                return Some(Self { _lock: lock });
            }
        }

        log::warn!("Could not acquire sync lock after {:?}, proceeding without lock", timeout);
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hash_is_deterministic() {
        assert_eq!(hash_content("hello"), hash_content("hello"));
    }

    #[test]
    fn hash_differs_for_different_content() {
        assert_ne!(hash_content("hello"), hash_content("world"));
    }

    #[test]
    fn record_and_check() {
        let mut fp = Fingerprints::default();
        fp.record("Cargo.toml", "[package]\nname = \"test\"");
        assert_eq!(fp.version, 1);
        assert!(fp.files.contains_key("Cargo.toml"));
    }

    #[test]
    fn load_missing_returns_empty() {
        let tmp = tempfile::tempdir().unwrap();
        let fp = Fingerprints::load(tmp.path()).unwrap();
        assert!(fp.files.is_empty());
    }

    #[test]
    fn round_trip() {
        let tmp = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(tmp.path().join(".horus")).unwrap();

        let mut fp = Fingerprints::default();
        fp.record("Cargo.toml", "content");
        fp.save(tmp.path()).unwrap();

        let loaded = Fingerprints::load(tmp.path()).unwrap();
        assert_eq!(
            loaded.files["Cargo.toml"].hash,
            fp.files["Cargo.toml"].hash
        );
    }

    #[test]
    fn test_fingerprints_save_load_roundtrip() {
        let tmp = tempfile::tempdir().unwrap();

        let mut fp = Fingerprints::default();
        fp.record("Cargo.toml", "[package]\nname = \"test\"");
        fp.record("pyproject.toml", "[project]\nname = \"test\"");
        fp.record("CMakeLists.txt", "cmake_minimum_required(VERSION 3.20)");
        fp.save(tmp.path()).unwrap();

        let loaded = Fingerprints::load(tmp.path()).unwrap();

        // Version preserved
        assert_eq!(loaded.version, fp.version);
        assert_eq!(loaded.version, 1);

        // All three files present
        assert_eq!(loaded.files.len(), 3);
        assert!(loaded.files.contains_key("Cargo.toml"));
        assert!(loaded.files.contains_key("pyproject.toml"));
        assert!(loaded.files.contains_key("CMakeLists.txt"));

        // Hashes match exactly
        for (key, original_fp) in &fp.files {
            let loaded_fp = &loaded.files[key];
            assert_eq!(loaded_fp.hash, original_fp.hash, "Hash mismatch for {}", key);
            assert_eq!(
                loaded_fp.generated_at, original_fp.generated_at,
                "Timestamp mismatch for {}",
                key
            );
        }
    }

    #[test]
    fn test_fingerprints_load_missing_file_returns_default() {
        let tmp = tempfile::tempdir().unwrap();
        // Do NOT create .horus/ or fingerprints.json — path does not exist

        let fp = Fingerprints::load(tmp.path()).unwrap();

        assert_eq!(fp.version, 0);
        assert!(fp.files.is_empty());
    }

    #[test]
    fn test_fingerprints_is_modified_detects_changes() {
        let tmp = tempfile::tempdir().unwrap();
        let horus_dir = tmp.path().join(".horus");
        std::fs::create_dir_all(&horus_dir).unwrap();

        let original_content = "[package]\nname = \"my-robot\"\nversion = \"0.1.0\"";
        let cargo_path = horus_dir.join("Cargo.toml");
        std::fs::write(&cargo_path, original_content).unwrap();

        // Record fingerprint matching current content
        let mut fp = Fingerprints::default();
        fp.record("Cargo.toml", original_content);

        // File matches fingerprint — should NOT be modified
        assert!(
            !fp.is_modified("Cargo.toml", tmp.path()),
            "File should not be detected as modified when content matches fingerprint"
        );

        // Now modify the file on disk (simulating an external tool like cargo)
        let modified_content =
            "[package]\nname = \"my-robot\"\nversion = \"0.1.0\"\n\n[dependencies]\nserde = \"1.0\"";
        std::fs::write(&cargo_path, modified_content).unwrap();

        // File no longer matches fingerprint — SHOULD be modified
        assert!(
            fp.is_modified("Cargo.toml", tmp.path()),
            "File should be detected as modified after external changes"
        );

        // A file not tracked in fingerprints should NOT report as modified
        assert!(
            !fp.is_modified("pyproject.toml", tmp.path()),
            "Untracked file should not be detected as modified"
        );

        // A tracked file that doesn't exist on disk should NOT report as modified
        fp.record("missing.toml", "some content");
        assert!(
            !fp.is_modified("missing.toml", tmp.path()),
            "Missing file on disk should not be detected as modified"
        );
    }
}
