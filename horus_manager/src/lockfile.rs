//! Unified lockfile for deterministic builds (`horus.lock`).
//!
//! Tracks pinned versions for all dependency sources (registry, crates.io, PyPI)
//! alongside toolchain versions, system dependencies, and feature flags.
//!
//! ## What the pins enforce, and what they still do not
//!
//! `[[package]]` entries are applied, not merely recorded. Every installer in
//! `commands::run::install` asks [`HorusLockfile::get_pinned`] for a version
//! before resolving one, so a checkout of a committed `horus.lock` installs
//! what the file names instead of whatever `latest` happened to resolve to on
//! that machine on that day. An ordinary `horus run`/`horus build` may *add* a
//! pin for a package that has none; it may not *move* one that is already
//! there. Only `horus lock`, which is an explicit request to re-resolve, and a
//! manifest that names a version the pin cannot satisfy, change an existing
//! pin.
//!
//! [`LockedPackage::checksum`] carries a `sha256:<hex>` digest for the sources
//! that publish one, and [`HorusLockfile::checksum_conflict`] refuses an
//! artifact whose digest disagrees with the pin at the same version:
//!
//! * `registry` — the digest `registry::install` already verified against the
//!   server, read back out of the package's `metadata.json`.
//! * `pypi` — the digest pip reports for the artifact it actually chose
//!   (`pip install --report`, pip 22.2+). Older pip cannot report one, so the
//!   field stays `None` rather than being filled with something weaker.
//! * `crates.io` — `cargo install` neither reports the `.crate` digest nor
//!   leaves it on disk, so this stays `None` for every crates.io binary. Its
//!   absence must not be read as "verified".
//!
//! Still unenforced, deliberately named here so the gap is not rediscovered:
//! `[toolchain]` and `[[system]]` are recorded and *reported*
//! (`system_deps::verify_lockfile`, which by contract never fails), but nothing
//! installs or switches to them; `features` is a record of what was active at
//! lock time, not a constraint on the next build.
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
use std::fmt;
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

    /// Integrity checksum as `sha256:<hex>`, for the sources that publish one.
    ///
    /// `None` is "no digest was available", never "verified" — see the
    /// module-level note for which sources can fill this in.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub checksum: Option<String>,
}

impl LockedPackage {
    /// Build an entry from the pieces the installers read off disk.
    pub fn new(name: &str, version: &str, source: &str, checksum: Option<String>) -> Self {
        Self {
            name: name.to_string(),
            version: version.to_string(),
            source: source.to_string(),
            checksum,
        }
    }
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
        // Atomic write: serialize into a sibling temp file, then rename it into place.
        // `rename(2)` is atomic within a filesystem, so an interrupt (Ctrl-C, crash,
        // ENOSPC mid-write) can never leave a truncated/corrupt lockfile — a reader
        // sees either the old file or the complete new one.
        let tmp = path.with_file_name(format!(
            "{}.tmp",
            path.file_name().and_then(|n| n.to_str()).unwrap_or("lock")
        ));
        fs::write(&tmp, &content)
            .with_context(|| format!("Failed to write lockfile: {}", tmp.display()))?;
        fs::rename(&tmp, path)
            .with_context(|| format!("Failed to persist lockfile: {}", path.display()))?;
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

    /// Get the pinned version for a package, if any.
    ///
    /// This lived behind `#[cfg(test)]`, which is why the pins were written
    /// and never enforced: nothing outside the test module could read them.
    /// Every installer calls it before choosing a version.
    pub fn get_pinned(&self, name: &str, source: &str) -> Option<&str> {
        self.get_locked(name, source).map(|p| p.version.as_str())
    }

    /// The whole `[[package]]` entry for a dependency, if it is pinned.
    ///
    /// Callers that need the checksum as well as the version take this rather
    /// than looking the package up twice.
    pub fn get_locked(&self, name: &str, source: &str) -> Option<&LockedPackage> {
        self.packages
            .iter()
            .find(|p| p.name == name && p.source == source)
    }

    /// Fill in a checksum for a pin that has none, without touching the
    /// version or an existing checksum.
    ///
    /// Completing a pin is not the same as moving one: a lockfile written
    /// before `--report` was available (or by a pip too old to produce one)
    /// records the version and no digest, and the next resolve that *can*
    /// produce a digest should record it. Returns whether anything changed.
    pub fn complete_checksum(&mut self, name: &str, source: &str, checksum: &str) -> bool {
        match self
            .packages
            .iter_mut()
            .find(|p| p.name == name && p.source == source)
        {
            Some(entry) if entry.checksum.is_none() => {
                entry.checksum = Some(checksum.to_string());
                true
            }
            _ => false,
        }
    }

    /// The pinned checksum an installed artifact contradicts, if it does.
    ///
    /// Answers `Some(pinned)` only when the pin and the artifact agree on the
    /// *version* and disagree on the digest — the one shape that means the
    /// bytes behind a version changed under the project. A pin with no
    /// checksum, an artifact with no checksum, or a version that differs are
    /// all `None`: the first two are the ordinary "no digest available" state
    /// this file documents, and the third is a version question, reported
    /// separately by [`HorusLockfile::drift`].
    pub fn checksum_conflict(&self, installed: &LockedPackage) -> Option<&str> {
        let pinned = self.get_locked(&installed.name, &installed.source)?;
        if pinned.version != installed.version {
            return None;
        }
        let (want, got) = (pinned.checksum.as_deref()?, installed.checksum.as_deref()?);
        (want != got).then_some(want)
    }

    /// Every way this lockfile fails to describe what is installed.
    ///
    /// Two shapes count, and a third deliberately does not:
    ///
    /// * a package installed at a version the lock does not name — the drift
    ///   the file exists to catch;
    /// * a package installed and not pinned at all — the lock is incomplete,
    ///   so a second machine cannot reproduce this one.
    ///
    /// A pin with *nothing installed for it* is not drift. That is the state
    /// of every fresh checkout before the first `horus run`, and the permanent
    /// state of a Rust project whose crates.io libraries cargo builds rather
    /// than HORUS installing. Failing `horus lock --check` on it would make the
    /// gate the docs tell you to put in CI fire on a clean clone.
    pub fn drift(&self, installed: &[LockedPackage]) -> Vec<Drift> {
        let mut drift = Vec::new();
        for pkg in installed {
            match self.get_locked(&pkg.name, &pkg.source) {
                Some(pinned) if pinned.version != pkg.version => drift.push(Drift::Version {
                    name: pkg.name.clone(),
                    source: pkg.source.clone(),
                    pinned: pinned.version.clone(),
                    installed: pkg.version.clone(),
                }),
                Some(pinned) => {
                    if let (Some(want), Some(got)) =
                        (pinned.checksum.as_deref(), pkg.checksum.as_deref())
                    {
                        if want != got {
                            drift.push(Drift::Checksum {
                                name: pkg.name.clone(),
                                source: pkg.source.clone(),
                                version: pkg.version.clone(),
                                pinned: want.to_string(),
                                installed: got.to_string(),
                            });
                        }
                    }
                }
                None => drift.push(Drift::Unpinned {
                    name: pkg.name.clone(),
                    source: pkg.source.clone(),
                    installed: pkg.version.clone(),
                }),
            }
        }
        drift
    }
}

/// One way `horus.lock` and the installed tree disagree.
///
/// Rendered by `horus lock --check`, which exits non-zero when this list is
/// non-empty. Every variant names the package, so the message says which
/// dependency to look at rather than "the lockfile is out of date".
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Drift {
    /// Pinned at one version, installed at another.
    Version {
        name: String,
        source: String,
        pinned: String,
        installed: String,
    },
    /// Same version, different bytes.
    Checksum {
        name: String,
        source: String,
        version: String,
        pinned: String,
        installed: String,
    },
    /// Installed and not named by the lockfile at all.
    Unpinned {
        name: String,
        source: String,
        installed: String,
    },
}

impl fmt::Display for Drift {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Version {
                name,
                source,
                pinned,
                installed,
            } => write!(
                f,
                "{name} ({source}): horus.lock pins {pinned}, {installed} is installed"
            ),
            Self::Checksum {
                name,
                source,
                version,
                pinned,
                installed,
            } => write!(
                f,
                "{name} ({source}) {version}: horus.lock records {pinned}, the installed \
                 artifact is {installed}"
            ),
            Self::Unpinned {
                name,
                source,
                installed,
            } => write!(
                f,
                "{name} ({source}): {installed} is installed and horus.lock does not pin it"
            ),
        }
    }
}

#[cfg(test)]
impl HorusLockfile {
    /// Remove a pinned package.
    pub fn unpin(&mut self, name: &str, source: &str) {
        self.packages
            .retain(|p| !(p.name == name && p.source == source));
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

/// The packages `.horus/` says are installed in a project.
///
/// Read back off disk rather than collected as the installers run, so the
/// answer covers packages a previous run resolved and this one skipped as
/// already-linked, rather than only what this particular invocation happened
/// to do. That is the set a second machine has to reproduce, so it is both
/// what gets written into `[[package]]` and what `horus lock --check` compares
/// the pins against.
///
/// Anything whose version cannot be established is left out: a pin that says
/// `version = "latest"` is not a pin, and writing one would make the lockfile
/// look authoritative while promising nothing.
///
/// This lives here rather than beside the installers because the lockfile is
/// the thing that has to answer "does the file describe reality", and
/// `horus lock --check` reaches it from `main.rs`, which cannot see inside
/// `commands::run`.
pub fn installed_packages(project_dir: &Path) -> Vec<LockedPackage> {
    let mut pins: Vec<LockedPackage> = Vec::new();

    // Two passes over `.horus/packages`, because a package can be represented
    // there both by a directory and by a JSON marker beside it and the
    // directory is the better evidence.
    //
    // Pass 1 — directories. A link into the global cache, or, for
    // `horus install <pkg>` against a local workspace target, a real directory
    // that pip unpacked in place (registry/install.rs installs to
    // `.horus/packages/<name>` and never links). This used to call
    // `fs::read_link` on everything, so the real-directory layout failed the
    // very first step and was silently left out of the lockfile: `horus run`
    // put the package on PYTHONPATH and horus.lock never named it.
    if let Ok(entries) = fs::read_dir(project_dir.join(".horus/packages")) {
        for entry in entries.flatten() {
            let path = entry.path();
            if !path.is_dir() {
                continue;
            }

            // `metadata.json` is written by whichever installer produced the
            // directory, and for a registry package it carries the sha256 that
            // `registry::install` already verified against the server. Reading
            // it back is how a verified download becomes a recorded pin
            // without a second network round trip.
            let checksum = metadata_checksum(&path);

            match link_target_name(&path) {
                // A link into the global cache, whose directory name carries
                // both the distribution and the version.
                Some(target) => {
                    if let Some((name, version)) =
                        target.strip_prefix("pypi_").and_then(split_name_version)
                    {
                        // pip is asked for `latest` when the manifest names no
                        // version, so the directory name cannot answer this one
                        // — the installed dist-info can.
                        let version = if version == "latest" {
                            match dist_info_version(&path) {
                                Some(v) => v,
                                None => continue,
                            }
                        } else {
                            version.to_string()
                        };
                        pins.push(LockedPackage::new(name, &version, "pypi", checksum));
                    } else if let Some((name, version)) = split_name_version(target.as_str()) {
                        pins.push(LockedPackage::new(name, version, "registry", checksum));
                    }
                }
                // Installed in place. The directory is named after the
                // distribution alone, so the version comes from the metadata
                // the installer wrote or from pip's own dist-info.
                None => {
                    let name = entry.file_name().to_string_lossy().to_string();
                    if let Some(version) =
                        installed_directory_version(&path).or_else(|| dist_info_version(&path))
                    {
                        pins.push(LockedPackage::new(&name, &version, "pypi", checksum));
                    }
                }
            }
        }
    }

    // Pass 2 — markers, for the packages no directory accounted for. A
    // reference to a package already installed in the system interpreter has no
    // directory at all; `<name>.pypi.json` normally sits beside a directory
    // pass 1 already pinned, and is the fallback when that directory is the one
    // form pass 1 could not read a version out of.
    if let Ok(entries) = fs::read_dir(project_dir.join(".horus/packages")) {
        for entry in entries.flatten() {
            let path = entry.path();
            let file_name = entry.file_name().to_string_lossy().to_string();
            let Some(name) = file_name
                .strip_suffix(".system.json")
                .or_else(|| file_name.strip_suffix(".pypi.json"))
            else {
                continue;
            };
            if pins.iter().any(|pinned| pinned.name == name) {
                continue;
            }
            if let Some(version) = marker_version(&path) {
                // A reference to the interpreter's own copy. There is no
                // artifact HORUS downloaded, so there is nothing to digest.
                pins.push(LockedPackage::new(name, &version, "pypi", None));
            }
        }
    }

    if let Ok(entries) = fs::read_dir(project_dir.join(".horus/bin")) {
        for entry in entries.flatten() {
            // `.horus/bin/<name>` links to `<cache>/cratesio_<name>@<ver>/bin/<name>`,
            // so the version lives two levels up from the link target.
            let Ok(target) = fs::read_link(entry.path()) else {
                continue;
            };
            let Some(pkg_dir) = target.parent().and_then(|bin| bin.parent()) else {
                continue;
            };
            let dir_name = pkg_dir.file_name().unwrap_or_default().to_string_lossy();
            if let Some((name, version)) = dir_name
                .strip_prefix("cratesio_")
                .and_then(split_name_version)
                .filter(|(_, version)| *version != "latest")
            {
                // No checksum: `cargo install` does not report the `.crate`
                // digest and leaves nothing on disk that carries it, so there
                // is nothing honest to put here. See the module note.
                pins.push(LockedPackage::new(name, version, "crates.io", None));
            }
        }
    }

    pins.sort_by(|a, b| {
        a.source
            .cmp(&b.source)
            .then_with(|| a.name.cmp(&b.name))
            .then_with(|| a.version.cmp(&b.version))
    });
    pins.dedup_by(|a, b| a.name == b.name && a.source == b.source && a.version == b.version);
    pins
}

/// Split a cache directory's `name@version` suffix.
fn split_name_version(dir_name: &str) -> Option<(&str, &str)> {
    dir_name
        .rsplit_once('@')
        .filter(|(name, _)| !name.is_empty())
}

/// The cache directory a `.horus/packages` entry links to, by name.
fn link_target_name(link: &Path) -> Option<String> {
    let target = fs::read_link(link).ok()?;
    Some(target.file_name()?.to_string_lossy().to_string())
}

/// The version recorded in the `metadata.json` an installer writes into a
/// package directory it unpacked in place.
///
/// `registry/install.rs` writes this file last, after pip reports success, and
/// it carries the version pip resolved — which for a `@latest` request is the
/// only place it is written down other than the dist-info.
fn installed_directory_version(pkg_dir: &Path) -> Option<String> {
    marker_version(&pkg_dir.join("metadata.json"))
}

/// The version recorded in a `<name>.system.json` or `<name>.pypi.json` marker,
/// or in a package directory's `metadata.json` — all three are the same
/// `{"name", "version", "source"}` shape.
fn marker_version(marker: &Path) -> Option<String> {
    read_json(marker)?
        .get("version")
        .and_then(|v| v.as_str())
        .filter(|v| !v.is_empty() && *v != "unknown")
        .map(|v| v.to_string())
}

/// The digest an installer recorded beside a package it wrote.
///
/// `registry::install` stores the sha256 it verified against the server as bare
/// hex; the pip path stores pip's own `sha256:<hex>`. Both are normalised here,
/// and anything that is not a sha256 is dropped rather than copied into the
/// lockfile as if it were one.
fn metadata_checksum(pkg_dir: &Path) -> Option<String> {
    let raw = read_json(&pkg_dir.join("metadata.json"))?
        .get("checksum")
        .and_then(|v| v.as_str())?
        .to_string();
    normalize_checksum(&raw)
}

fn read_json(path: &Path) -> Option<serde_json::Value> {
    serde_json::from_str(&fs::read_to_string(path).ok()?).ok()
}

/// Canonicalise a digest to `sha256:<lowercase hex>`.
///
/// Accepts the three spellings that reach this code — bare hex from the
/// registry, `sha256=<hex>` from pip's report, and the canonical form on a
/// round trip — and returns `None` for anything else. A digest that is not a
/// sha256 must not be written into `checksum`, because everything that reads
/// that field treats it as one.
pub fn normalize_checksum(raw: &str) -> Option<String> {
    let hex = raw
        .strip_prefix("sha256:")
        .or_else(|| raw.strip_prefix("sha256="))
        .unwrap_or(raw)
        .trim();
    if hex.len() != 64 || !hex.chars().all(|c| c.is_ascii_hexdigit()) {
        return None;
    }
    Some(format!("sha256:{}", hex.to_ascii_lowercase()))
}

/// The version pip actually installed, from the `*.dist-info` it leaves behind.
///
/// Wheel metadata directories are named `{distribution}-{version}.dist-info`,
/// which is the only place the resolved version is written down when the
/// manifest asked for no particular one.
fn dist_info_version(pkg_dir: &Path) -> Option<String> {
    let entries = fs::read_dir(pkg_dir).ok()?;
    for entry in entries.flatten() {
        let name = entry.file_name().to_string_lossy().to_string();
        if let Some(stem) = name.strip_suffix(".dist-info") {
            if let Some((_, version)) = stem.rsplit_once('-') {
                if !version.is_empty() {
                    return Some(version.to_string());
                }
            }
        }
    }
    None
}

/// Compute a SHA-256 hash of the horus.toml config for staleness detection.
pub fn hash_config(config_content: &str) -> String {
    use sha2::{Digest, Sha256};
    let mut hasher = Sha256::new();
    hasher.update(config_content.as_bytes());
    hex::encode(hasher.finalize())
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
    fn save_to_is_atomic_and_leaves_no_temp_file() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("horus.lock");
        let mut lock = HorusLockfile::new();
        lock.pin("serde", "1.0.215", "crates.io", None);
        lock.save_to(&path).unwrap();

        // Written and round-trips through load_from.
        assert!(path.exists());
        assert_eq!(HorusLockfile::load_from(&path).unwrap().packages.len(), 1);

        // The sibling temp file must have been renamed into place, not left behind
        // (guards the tmp+rename mechanics — a broken rename would strand horus.lock.tmp).
        assert!(
            !dir.path().join("horus.lock.tmp").exists(),
            "atomic save must not leave a .tmp file behind"
        );
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

    #[test]
    fn test_lockfile_roundtrip_with_multiple_sources() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("horus.lock");

        let mut lock = HorusLockfile::new();
        lock.config_hash = Some("multi_source_hash".to_string());
        lock.toolchain = Some(ToolchainPins {
            rust: Some("1.78.0".to_string()),
            python: Some("3.12.3".to_string()),
            cmake: None,
        });

        // Pin deps from all 6 source types
        lock.pin(
            "rplidar",
            "1.2.0",
            "registry",
            Some("sha256:reg_check".to_string()),
        );
        lock.pin(
            "serde",
            "1.0.215",
            "crates.io",
            Some("sha256:crate_check".to_string()),
        );
        lock.pin("numpy", "1.26.4", "pypi", None);
        lock.pin("libudev", "252", "system", None);
        lock.pin("my_local_lib", "0.1.0", "path", None);
        lock.pin("horus_utils", "0.2.0", "git", None);

        lock.save_to(&path).unwrap();
        let loaded = HorusLockfile::load_from(&path).unwrap();

        // All 6 packages preserved
        assert_eq!(loaded.packages.len(), 6);

        // Verify each source is preserved
        assert_eq!(loaded.get_pinned("rplidar", "registry"), Some("1.2.0"));
        assert_eq!(loaded.get_pinned("serde", "crates.io"), Some("1.0.215"));
        assert_eq!(loaded.get_pinned("numpy", "pypi"), Some("1.26.4"));
        assert_eq!(loaded.get_pinned("libudev", "system"), Some("252"));
        assert_eq!(loaded.get_pinned("my_local_lib", "path"), Some("0.1.0"));
        assert_eq!(loaded.get_pinned("horus_utils", "git"), Some("0.2.0"));

        // Checksums preserved where set
        let registry_pkg = loaded
            .packages
            .iter()
            .find(|p| p.source == "registry")
            .unwrap();
        assert_eq!(registry_pkg.checksum, Some("sha256:reg_check".to_string()));
        let crates_pkg = loaded
            .packages
            .iter()
            .find(|p| p.source == "crates.io")
            .unwrap();
        assert_eq!(crates_pkg.checksum, Some("sha256:crate_check".to_string()));
        let pypi_pkg = loaded.packages.iter().find(|p| p.source == "pypi").unwrap();
        assert!(pypi_pkg.checksum.is_none());

        // Toolchain preserved
        assert_eq!(
            loaded.toolchain.as_ref().unwrap().rust,
            Some("1.78.0".to_string())
        );
        assert_eq!(
            loaded.toolchain.as_ref().unwrap().python,
            Some("3.12.3".to_string())
        );
    }

    #[test]
    fn test_lockfile_version_3_accepted() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("horus.lock");

        let v3_content = r#"
version = 3
config_hash = "v3_test_hash"

[[package]]
name = "tokio"
version = "1.36.0"
source = "crates.io"

[[package]]
name = "rplidar"
version = "1.0.0"
source = "registry"
"#;
        fs::write(&path, v3_content).unwrap();

        let loaded = HorusLockfile::load_from(&path);
        assert!(loaded.is_ok(), "v3 lockfile should be accepted");
        let lock = loaded.unwrap();
        assert_eq!(lock.version, 3);
        assert_eq!(lock.config_hash, Some("v3_test_hash".to_string()));
        assert_eq!(lock.packages.len(), 2);
        assert_eq!(lock.packages[0].name, "tokio");
        assert_eq!(lock.packages[1].name, "rplidar");
        // v3 has no toolchain, features, or system_deps
        assert!(lock.toolchain.is_none());
        assert!(lock.features.is_empty());
        assert!(lock.system_deps.is_empty());
    }

    #[test]
    fn test_lockfile_old_version_rejected() {
        let dir = tempfile::tempdir().unwrap();

        // v1 rejected
        let path_v1 = dir.path().join("v1.lock");
        fs::write(&path_v1, "version = 1\nmanifest_hash = \"old\"\n").unwrap();
        let result = HorusLockfile::load_from(&path_v1);
        assert!(result.is_err(), "v1 lockfile must be rejected");
        assert!(
            result
                .unwrap_err()
                .to_string()
                .contains("Outdated lockfile version"),
            "Error should mention outdated version"
        );

        // v2 rejected
        let path_v2 = dir.path().join("v2.lock");
        fs::write(&path_v2, "version = 2\nconfig_hash = \"old\"\n").unwrap();
        let result = HorusLockfile::load_from(&path_v2);
        assert!(result.is_err(), "v2 lockfile must be rejected");
        assert!(
            result
                .unwrap_err()
                .to_string()
                .contains("Outdated lockfile version"),
            "Error should mention outdated version"
        );
    }

    #[test]
    fn test_lockfile_check_detects_stale() {
        // Create a lockfile with a known config hash
        let original_config = "[package]\nname = \"bot\"\nversion = \"0.1.0\"\n";
        let original_hash = hash_config(original_config);

        let mut lock = HorusLockfile::new();
        lock.config_hash = Some(original_hash.clone());
        lock.pin("serde", "1.0.0", "crates.io", None);

        // Same config => not stale
        assert!(
            !lock.is_stale(&original_hash),
            "Lockfile with matching hash should not be stale"
        );

        // Modified config (added a dep) => different hash => stale
        let modified_config =
            "[package]\nname = \"bot\"\nversion = \"0.1.0\"\n\n[dependencies]\ntokio = \"1.0\"\n";
        let modified_hash = hash_config(modified_config);
        assert_ne!(
            original_hash, modified_hash,
            "Different configs must produce different hashes"
        );
        assert!(
            lock.is_stale(&modified_hash),
            "Lockfile should be stale after manifest modification"
        );
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
