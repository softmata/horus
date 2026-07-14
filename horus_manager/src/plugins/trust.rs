//! Plugin trust store — the out-of-repo provenance anchor for plugin execution.
//!
//! # Why this exists (SEC2 / SEC3)
//!
//! A plugin binary is executed when a user runs `horus <cmd>` (see
//! [`super::executor::PluginExecutor`]). Discovery prefers the **project**
//! `.horus/` directory, which is part of any cloned repository and therefore
//! fully attacker-controlled: a hostile repo can ship `.horus/plugins.lock` +
//! `.horus/bin/horus-<cmd>` and get its binary auto-executed the moment the
//! victim types `horus <cmd>` in the checkout.
//!
//! The old integrity check compared the binary against a checksum stored **in
//! that same attacker-controlled lockfile** — a vacuous check (the attacker
//! simply writes the matching hash). A checksum proves *integrity*, never
//! *provenance*.
//!
//! This store is the missing **out-of-repo trust record**. It lives under the
//! user's data directory (`~/.local/share/horus/trusted_plugins.json` on Linux)
//! — a location a cloned repository cannot write to. A project-local plugin is
//! allowed to execute **only** if the SHA-256 of its binary is recorded here.
//! Because the key is the content hash, this record simultaneously proves:
//!   * **provenance** — the entry was written by a deliberate
//!     `horus plugin install --local` / `horus plugin trust` on *this* machine,
//!     not by cloning a repo; and
//!   * **integrity** — swapping the binary changes its hash, so a tampered
//!     binary is no longer in the store and is refused.
//!
//! Trust is *conferred at install time* (the registry install path performs the
//! Ed25519 signature / checksum verification — SEC1). This store is the runtime
//! enforcement point: "only binaries that were trusted through a deliberate,
//! out-of-repo action may run."

use anyhow::{Context, Result};
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

/// Current on-disk schema version for the trust store.
const TRUST_STORE_VERSION: u32 = 1;

fn default_version() -> u32 {
    TRUST_STORE_VERSION
}

/// A single trusted-plugin record, keyed in [`TrustStore::trusted`] by the
/// binary's SHA-256 checksum (`"sha256:<hex>"`).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct TrustRecord {
    /// CLI command this binary was trusted as (best-effort label for humans).
    pub command: String,

    /// Path the binary lived at when it was trusted (informational only — the
    /// trust decision is made purely on the content hash, never the path).
    pub path: PathBuf,

    /// When the trust was granted.
    pub added_at: DateTime<Utc>,
}

/// The out-of-repo set of plugin binaries the user has deliberately trusted.
///
/// Membership is by binary content hash, so a trusted record survives a rename
/// but is invalidated the instant the binary's bytes change.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrustStore {
    /// Schema version for forward compatibility.
    #[serde(default = "default_version")]
    pub version: u32,

    /// Trusted binaries: `"sha256:<hex>"` -> record.
    #[serde(default)]
    pub trusted: BTreeMap<String, TrustRecord>,
}

impl Default for TrustStore {
    fn default() -> Self {
        Self {
            version: TRUST_STORE_VERSION,
            trusted: BTreeMap::new(),
        }
    }
}

impl TrustStore {
    /// Load the trust store from `path`.
    ///
    /// A missing file is **not** an error — it yields an empty store (nothing
    /// trusted yet). A present-but-corrupt file **is** an error; callers on the
    /// execution hot path must treat that as "trust nothing" (fail closed)
    /// rather than crashing or, worse, trusting everything.
    pub fn load(path: &Path) -> Result<Self> {
        if !path.exists() {
            return Ok(Self::default());
        }
        let content = fs::read_to_string(path)
            .with_context(|| format!("failed to read trust store from {}", path.display()))?;
        let store: Self = serde_json::from_str(&content)
            .with_context(|| format!("failed to parse trust store at {}", path.display()))?;
        Ok(store)
    }

    /// Persist the trust store to `path` (atomically via temp + rename).
    pub fn save(&self, path: &Path) -> Result<()> {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).with_context(|| {
                format!("failed to create trust store directory {}", parent.display())
            })?;
        }
        let content = serde_json::to_string_pretty(self)?;
        // Temp-then-rename so a concurrent reader never sees a half-written file.
        let tmp = path.with_extension("json.tmp");
        fs::write(&tmp, content)
            .with_context(|| format!("failed to write trust store to {}", tmp.display()))?;
        fs::rename(&tmp, path)
            .with_context(|| format!("failed to finalize trust store at {}", path.display()))?;
        Ok(())
    }

    /// Is this binary checksum trusted? An empty checksum is never trusted.
    pub fn contains(&self, checksum: &str) -> bool {
        !checksum.is_empty() && self.trusted.contains_key(checksum)
    }

    /// Record trust for `checksum` (idempotent — re-trusting refreshes the
    /// record). An empty checksum is rejected so a hashing bug cannot silently
    /// trust "everything with no hash."
    pub fn trust(&mut self, checksum: &str, command: &str, path: &Path) -> Result<()> {
        if checksum.is_empty() {
            anyhow::bail!("refusing to trust a plugin with an empty checksum");
        }
        self.trusted.insert(
            checksum.to_string(),
            TrustRecord {
                command: command.to_string(),
                path: path.to_path_buf(),
                added_at: Utc::now(),
            },
        );
        Ok(())
    }

    /// Remove every record whose `command` label matches `command`.
    /// Returns the number of records removed.
    pub fn untrust_command(&mut self, command: &str) -> usize {
        let before = self.trusted.len();
        self.trusted.retain(|_, rec| rec.command != command);
        before - self.trusted.len()
    }

    /// Iterate over `(checksum, record)` pairs, sorted by command name.
    pub fn records(&self) -> Vec<(&String, &TrustRecord)> {
        let mut v: Vec<_> = self.trusted.iter().collect();
        v.sort_by(|a, b| a.1.command.cmp(&b.1.command).then(a.0.cmp(b.0)));
        v
    }

    /// Number of trusted binaries.
    pub fn len(&self) -> usize {
        self.trusted.len()
    }

    /// Whether the store is empty.
    pub fn is_empty(&self) -> bool {
        self.trusted.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    fn store_path(tmp: &TempDir) -> PathBuf {
        tmp.path().join("trusted_plugins.json")
    }

    #[test]
    fn load_missing_file_returns_empty_not_error() {
        let tmp = TempDir::new().unwrap();
        let store = TrustStore::load(&store_path(&tmp)).unwrap();
        assert!(store.is_empty());
        assert_eq!(store.version, TRUST_STORE_VERSION);
    }

    #[test]
    fn load_corrupt_file_is_error() {
        let tmp = TempDir::new().unwrap();
        let path = store_path(&tmp);
        fs::write(&path, b"this is not json {{{").unwrap();
        // A corrupt store must surface as Err so the caller can fail closed;
        // it must NOT silently deserialize into a permissive/empty store.
        TrustStore::load(&path).unwrap_err();
    }

    #[test]
    fn trust_then_contains_and_roundtrips() {
        let tmp = TempDir::new().unwrap();
        let path = store_path(&tmp);

        let mut store = TrustStore::default();
        store
            .trust("sha256:deadbeef", "nav2", Path::new("/proj/.horus/bin/horus-nav2"))
            .unwrap();
        assert!(store.contains("sha256:deadbeef"));
        assert!(!store.contains("sha256:other"));
        store.save(&path).unwrap();

        let reloaded = TrustStore::load(&path).unwrap();
        assert!(reloaded.contains("sha256:deadbeef"));
        assert_eq!(reloaded.trusted["sha256:deadbeef"].command, "nav2");
    }

    #[test]
    fn empty_checksum_is_never_trusted() {
        let mut store = TrustStore::default();
        assert!(!store.contains(""));
        // trusting an empty checksum is refused
        assert!(store.trust("", "x", Path::new("/x")).is_err());
        assert!(store.is_empty());
    }

    #[test]
    fn untrust_removes_by_command() {
        let mut store = TrustStore::default();
        store.trust("sha256:a", "cmd1", Path::new("/a")).unwrap();
        store.trust("sha256:b", "cmd1", Path::new("/b")).unwrap();
        store.trust("sha256:c", "cmd2", Path::new("/c")).unwrap();
        assert_eq!(store.len(), 3);

        let removed = store.untrust_command("cmd1");
        assert_eq!(removed, 2);
        assert_eq!(store.len(), 1);
        assert!(store.contains("sha256:c"));
        assert!(!store.contains("sha256:a"));

        // untrusting a missing command removes nothing
        assert_eq!(store.untrust_command("nope"), 0);
    }

    #[test]
    fn save_is_atomic_and_leaves_no_temp_file() {
        let tmp = TempDir::new().unwrap();
        let path = store_path(&tmp);
        let mut store = TrustStore::default();
        store.trust("sha256:z", "z", Path::new("/z")).unwrap();
        store.save(&path).unwrap();
        assert!(path.exists());
        assert!(!path.with_extension("json.tmp").exists());
    }
}
