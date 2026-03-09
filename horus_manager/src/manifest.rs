//! Typed manifest for HORUS projects (`horus.toml`).
//!
//! This module is the **single source of truth** for the manifest schema.
//! Every command that needs project metadata loads a [`HorusManifest`] via
//! [`HorusManifest::find_and_load`] or [`HorusManifest::load_from`].
//!
//! ## File format
//!
//! ```toml
//! [package]
//! name = "my-robot"
//! version = "0.1.0"
//! description = "A mobile robot controller"
//! language = "rust"
//!
//! [dependencies]
//! horus = "^0.2"
//! numpy = { version = "^1.24", source = "pypi" }
//!
//! [dev-dependencies]
//! proptest = "^1.4"
//! ```

use anyhow::{anyhow, Context, Result};
use colored::*;
use semver::VersionReq;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::fmt;
use std::fs;
use std::path::{Path, PathBuf};

#[cfg(feature = "schema")]
use schemars::JsonSchema;

use crate::dependency_resolver::{DependencySource, DependencySpec};

// ─── Constants ───────────────────────────────────────────────────────────────

/// Primary manifest filename (new format).
pub const HORUS_TOML: &str = "horus.toml";


// ─── Top-level manifest ─────────────────────────────────────────────────────

/// The typed representation of a `horus.toml` manifest.
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HorusManifest {
    /// `[package]` — project metadata.
    pub package: PackageInfo,

    /// `[dependencies]` — runtime dependencies.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub dependencies: BTreeMap<String, DependencyValue>,

    /// `[dev-dependencies]` — test/dev-only dependencies.
    #[serde(
        default,
        rename = "dev-dependencies",
        skip_serializing_if = "BTreeMap::is_empty"
    )]
    pub dev_dependencies: BTreeMap<String, DependencyValue>,

    /// `[build-dependencies]` — build-time only dependencies.
    #[serde(
        default,
        rename = "build-dependencies",
        skip_serializing_if = "BTreeMap::is_empty"
    )]
    pub build_dependencies: BTreeMap<String, DependencyValue>,

    /// `[drivers]` — hardware driver configuration.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub drivers: BTreeMap<String, DriverValue>,

    /// `[ignore]` — patterns to exclude during scanning.
    #[serde(default, skip_serializing_if = "IgnoreConfig::is_empty")]
    pub ignore: IgnoreConfig,

    /// `enable = ["cuda", "editor"]` — capabilities to enable.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub enable: Vec<String>,
}

// ─── [package] ──────────────────────────────────────────────────────────────

/// Project metadata under `[package]`.
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PackageInfo {
    /// Package name (2-64 chars, lowercase alphanumeric + hyphens/underscores).
    pub name: String,

    /// Semantic version string (e.g., `"0.1.0"`).
    pub version: String,

    /// Human-readable description.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,

    /// List of authors (e.g., `["Jane Doe <jane@example.com>"]`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub authors: Vec<String>,

    /// SPDX license identifier (e.g., `"Apache-2.0"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub license: Option<String>,

    /// Primary language: `"rust"` or `"python"`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub language: Option<String>,

    /// Manifest schema edition for forward compatibility.
    #[serde(default = "default_edition", skip_serializing_if = "is_default_edition")]
    pub edition: String,

    /// Source code repository URL.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub repository: Option<String>,

    /// Package type for the registry.
    #[serde(default, skip_serializing_if = "Option::is_none", rename = "package-type")]
    pub package_type: Option<PackageType>,

    /// Registry categories (comma-separated or array).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub categories: Vec<String>,
}

fn default_edition() -> String {
    "1".to_string()
}

fn is_default_edition(s: &str) -> bool {
    s == "1"
}

/// Package type for registry classification.
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PackageType {
    Node,
    Driver,
    Tool,
    Algorithm,
    Model,
    Message,
    App,
}

impl fmt::Display for PackageType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Node => write!(f, "node"),
            Self::Driver => write!(f, "driver"),
            Self::Tool => write!(f, "tool"),
            Self::Algorithm => write!(f, "algorithm"),
            Self::Model => write!(f, "model"),
            Self::Message => write!(f, "message"),
            Self::App => write!(f, "app"),
        }
    }
}

// ─── Dependency values ──────────────────────────────────────────────────────

/// A dependency specification in the manifest.
///
/// Two forms:
/// - **Simple**: `horus = "^0.2"` → [`DependencyValue::Simple`]
/// - **Detailed**: `numpy = { version = "^1.24", source = "pypi" }` → [`DependencyValue::Detailed`]
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum DependencyValue {
    /// Version requirement string, e.g. `"^0.2"`, `"*"`, `"1.0"`.
    Simple(String),

    /// Structured dependency with optional source, path, git, features, etc.
    Detailed(DetailedDependency),
}

/// Structured dependency with all possible fields.
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetailedDependency {
    /// Version requirement (e.g., `"^1.24"`). Optional for path/git deps.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,

    /// Dependency source override.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<DepSource>,

    /// Local filesystem path (relative to manifest directory).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,

    /// Git repository URL.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub git: Option<String>,

    /// Git branch.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub branch: Option<String>,

    /// Git tag.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tag: Option<String>,

    /// Git revision (commit hash).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rev: Option<String>,

    /// Crate/package features to enable.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub features: Vec<String>,

    /// Target platform filter (e.g., `"linux-x86_64"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target: Option<String>,
}

/// Dependency source identifier.
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum DepSource {
    /// HORUS registry (default).
    Registry,
    /// Python Package Index.
    Pypi,
    /// Rust crates.io.
    CratesIo,
    /// System package manager (apt, brew, etc.).
    System,
}

impl fmt::Display for DepSource {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Registry => write!(f, "registry"),
            Self::Pypi => write!(f, "pypi"),
            Self::CratesIo => write!(f, "crates-io"),
            Self::System => write!(f, "system"),
        }
    }
}

// ─── Driver values ──────────────────────────────────────────────────────────

/// Driver configuration value.
///
/// - Simple: `camera = "opencv"` (backend name)
/// - Or just: `camera = true` (enable with default backend)
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum DriverValue {
    /// Backend name string, e.g., `"opencv"`.
    Backend(String),
    /// Enable with default backend.
    Enabled(bool),
}

// ─── [ignore] ───────────────────────────────────────────────────────────────

/// Patterns to exclude during file scanning and dependency resolution.
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct IgnoreConfig {
    /// File glob patterns to ignore (e.g., `["debug_*.py", "**/experiments/**"]`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub files: Vec<String>,

    /// Directory names to ignore (e.g., `["old/", "experiments/"]`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub directories: Vec<String>,

    /// Package names to skip during auto-install (e.g., `["ipython"]`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub packages: Vec<String>,
}

impl IgnoreConfig {
    pub fn is_empty(&self) -> bool {
        self.files.is_empty() && self.directories.is_empty() && self.packages.is_empty()
    }
}

// ─── Conversion: DependencyValue → DependencySpec ───────────────────────────

impl DependencyValue {
    /// Convert a named dependency value into a [`DependencySpec`] for the resolver.
    pub fn to_spec(&self, name: &str) -> Result<DependencySpec> {
        // Special case: horus_py always maps to pip:horus-robotics
        if name == "horus_py" {
            return Ok(DependencySpec {
                name: "horus_py".to_string(),
                requirement: VersionReq::STAR,
                source: DependencySource::Pip {
                    package_name: "horus-robotics".to_string(),
                },
                target: None,
            });
        }

        match self {
            DependencyValue::Simple(version_str) => {
                let requirement = parse_version_req(version_str)?;
                Ok(DependencySpec {
                    name: name.to_string(),
                    requirement,
                    source: DependencySource::Registry,
                    target: None,
                })
            }

            DependencyValue::Detailed(detail) => {
                let requirement = if let Some(ref v) = detail.version {
                    parse_version_req(v)?
                } else {
                    VersionReq::STAR
                };

                let source = if let Some(ref path_str) = detail.path {
                    DependencySource::Path(PathBuf::from(path_str))
                } else if let Some(ref git_url) = detail.git {
                    DependencySource::Git {
                        url: git_url.clone(),
                        branch: detail.branch.clone(),
                        tag: detail.tag.clone(),
                        rev: detail.rev.clone(),
                    }
                } else if let Some(ref src) = detail.source {
                    match src {
                        DepSource::Registry => DependencySource::Registry,
                        DepSource::Pypi => DependencySource::Pip {
                            package_name: name.to_string(),
                        },
                        DepSource::CratesIo => DependencySource::CratesIO,
                        DepSource::System => DependencySource::System,
                    }
                } else {
                    DependencySource::Registry
                };

                Ok(DependencySpec {
                    name: name.to_string(),
                    requirement,
                    source,
                    target: detail.target.clone(),
                })
            }
        }
    }
}

/// Parse a version requirement string, treating `"latest"` and `"*"` as `VersionReq::STAR`.
fn parse_version_req(s: &str) -> Result<VersionReq> {
    if s == "latest" || s == "*" {
        Ok(VersionReq::STAR)
    } else {
        VersionReq::parse(s).map_err(|e| anyhow!("Invalid version constraint '{}': {}", s, e))
    }
}

// ─── HorusManifest: loading ─────────────────────────────────────────────────

/// The format the manifest was loaded from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ManifestOrigin {
    Toml,
}

impl HorusManifest {
    /// Load manifest from a TOML file path.
    pub fn load_from(path: &Path) -> Result<(Self, ManifestOrigin)> {
        let content = fs::read_to_string(path)
            .with_context(|| format!("Failed to read manifest: {}", path.display()))?;

        let manifest: HorusManifest = toml::from_str(&content)
            .with_context(|| format!("Failed to parse {}", path.display()))?;
        Ok((manifest, ManifestOrigin::Toml))
    }

    /// Search upward from the current directory to find and load a manifest.
    ///
    /// Searches for `horus.toml` in the current directory and parent directories.
    /// Returns the manifest and the directory it was found in.
    pub fn find_and_load() -> Result<(Self, PathBuf, ManifestOrigin)> {
        Self::find_and_load_from(std::env::current_dir()?)
    }

    /// Search upward from `start_dir` to find and load a manifest.
    pub fn find_and_load_from(start_dir: PathBuf) -> Result<(Self, PathBuf, ManifestOrigin)> {
        let mut current = start_dir;

        for _ in 0..10 {
            let toml_path = current.join(HORUS_TOML);
            if toml_path.exists() {
                let (manifest, origin) = Self::load_from(&toml_path)?;
                return Ok((manifest, current, origin));
            }

            if let Some(parent) = current.parent() {
                current = parent.to_path_buf();
            } else {
                break;
            }
        }

        Err(anyhow!(
            "No horus.toml found. Run {} to create a project.",
            "horus new".cyan(),
        ))
    }

    /// Convert all `[dependencies]` to `Vec<DependencySpec>` for the resolver.
    pub fn dependencies_as_specs(&self) -> Result<Vec<DependencySpec>> {
        self.deps_to_specs(&self.dependencies)
    }

    /// Convert all `[dev-dependencies]` to `Vec<DependencySpec>`.
    pub fn dev_dependencies_as_specs(&self) -> Result<Vec<DependencySpec>> {
        self.deps_to_specs(&self.dev_dependencies)
    }

    /// Convert all `[build-dependencies]` to `Vec<DependencySpec>`.
    pub fn build_dependencies_as_specs(&self) -> Result<Vec<DependencySpec>> {
        self.deps_to_specs(&self.build_dependencies)
    }

    /// Convert all dependencies (runtime + dev) to specs. Used by `horus run` and `horus test`.
    pub fn all_dependencies_as_specs(&self) -> Result<Vec<DependencySpec>> {
        let mut specs = self.dependencies_as_specs()?;
        specs.extend(self.dev_dependencies_as_specs()?);
        Ok(specs)
    }

    fn deps_to_specs(&self, deps: &BTreeMap<String, DependencyValue>) -> Result<Vec<DependencySpec>> {
        deps.iter()
            .map(|(name, value)| {
                value
                    .to_spec(name)
                    .with_context(|| format!("Invalid dependency '{}'", name))
            })
            .collect()
    }

    /// Save manifest as TOML to a file path.
    pub fn save_to(&self, path: &Path) -> Result<()> {
        let content = toml::to_string_pretty(self)
            .context("Failed to serialize manifest to TOML")?;
        fs::write(path, content)
            .with_context(|| format!("Failed to write {}", path.display()))?;
        Ok(())
    }

    // ── Validation ──────────────────────────────────────────────────────

    /// Validate the manifest and return all errors (not fail-fast).
    pub fn validate(&self) -> Result<Vec<String>> {
        let mut warnings = Vec::new();
        let mut errors = Vec::new();

        // Name validation
        if self.package.name.len() < 2 || self.package.name.len() > 64 {
            errors.push(format!(
                "Package name '{}' must be 2-64 characters",
                self.package.name
            ));
        }

        // Check for valid characters
        if !self.package.name.chars().all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-' || c == '_' || c == '@' || c == '/') {
            errors.push(format!(
                "Package name '{}' must contain only lowercase letters, digits, hyphens, and underscores",
                self.package.name
            ));
        }

        // Reserved names
        let reserved = [
            "horus", "core", "std", "lib", "test", "main", "admin", "api",
            "root", "system", "internal", "config", "setup", "install",
        ];
        if reserved.contains(&self.package.name.as_str()) {
            errors.push(format!(
                "Package name '{}' is reserved",
                self.package.name
            ));
        }

        // Version validation
        if semver::Version::parse(&self.package.version).is_err() {
            errors.push(format!(
                "Version '{}' is not valid semver (expected e.g., '0.1.0')",
                self.package.version
            ));
        }

        // Edition validation
        let known_editions = ["1"];
        if !known_editions.contains(&self.package.edition.as_str()) {
            warnings.push(format!(
                "Unknown edition '{}'. Known editions: {}",
                self.package.edition,
                known_editions.join(", ")
            ));
        }

        // Dependency validation
        for (name, value) in &self.dependencies {
            if let Err(e) = value.to_spec(name) {
                errors.push(format!("[dependencies] {}: {}", name, e));
            }
        }
        for (name, value) in &self.dev_dependencies {
            if let Err(e) = value.to_spec(name) {
                errors.push(format!("[dev-dependencies] {}: {}", name, e));
            }
        }
        for (name, value) in &self.build_dependencies {
            if let Err(e) = value.to_spec(name) {
                errors.push(format!("[build-dependencies] {}: {}", name, e));
            }
        }

        if !errors.is_empty() {
            let msg = errors
                .iter()
                .enumerate()
                .map(|(i, e)| format!("  {}. {}", i + 1, e))
                .collect::<Vec<_>>()
                .join("\n");
            return Err(anyhow!("Manifest validation failed:\n{}", msg));
        }

        Ok(warnings)
    }
}

// ─── Manifest editing with toml_edit (preserves comments) ───────────────────

/// Add a dependency to a manifest file, preserving comments and formatting.
pub fn add_dependency_to_file(
    manifest_path: &Path,
    name: &str,
    value: &DependencyValue,
    section: &str, // "dependencies", "dev-dependencies", or "build-dependencies"
) -> Result<()> {
    let content = fs::read_to_string(manifest_path)
        .with_context(|| format!("Failed to read {}", manifest_path.display()))?;
    let mut doc = content
        .parse::<toml_edit::DocumentMut>()
        .with_context(|| format!("Failed to parse {}", manifest_path.display()))?;

    // Ensure section exists
    if doc.get(section).is_none() {
        doc[section] = toml_edit::Item::Table(toml_edit::Table::new());
    }

    // Insert dependency
    match value {
        DependencyValue::Simple(version) => {
            doc[section][name] = toml_edit::value(version.as_str());
        }
        DependencyValue::Detailed(detail) => {
            let mut table = toml_edit::InlineTable::new();
            if let Some(ref v) = detail.version {
                table.insert("version", v.as_str().into());
            }
            if let Some(ref s) = detail.source {
                table.insert("source", s.to_string().as_str().into());
            }
            if let Some(ref p) = detail.path {
                table.insert("path", p.as_str().into());
            }
            if let Some(ref g) = detail.git {
                table.insert("git", g.as_str().into());
            }
            if let Some(ref b) = detail.branch {
                table.insert("branch", b.as_str().into());
            }
            if let Some(ref t) = detail.tag {
                table.insert("tag", t.as_str().into());
            }
            if let Some(ref r) = detail.rev {
                table.insert("rev", r.as_str().into());
            }
            doc[section][name] = toml_edit::value(table);
        }
    }

    fs::write(manifest_path, doc.to_string())
        .with_context(|| format!("Failed to write {}", manifest_path.display()))?;
    Ok(())
}

/// Remove a dependency from a manifest file, preserving comments and formatting.
pub fn remove_dependency_from_file(
    manifest_path: &Path,
    name: &str,
    section: &str,
) -> Result<bool> {
    let content = fs::read_to_string(manifest_path)
        .with_context(|| format!("Failed to read {}", manifest_path.display()))?;
    let mut doc = content
        .parse::<toml_edit::DocumentMut>()
        .with_context(|| format!("Failed to parse {}", manifest_path.display()))?;

    let removed = if let Some(table) = doc.get_mut(section).and_then(|v| v.as_table_mut()) {
        table.remove(name).is_some()
    } else {
        false
    };

    if removed {
        fs::write(manifest_path, doc.to_string())
            .with_context(|| format!("Failed to write {}", manifest_path.display()))?;
    }

    Ok(removed)
}

// ─── Schema generation ──────────────────────────────────────────────────────

/// Generate a JSON Schema for the `horus.toml` manifest format.
///
/// Requires the `schema` feature to be enabled.
#[cfg(feature = "schema")]
pub fn generate_manifest_schema() -> String {
    let schema = schemars::schema_for!(HorusManifest);
    serde_json::to_string_pretty(&schema).expect("Failed to serialize JSON Schema")
}

#[cfg(all(test, feature = "schema"))]
#[test]
fn schema_generation_works() {
    let schema = generate_manifest_schema();
    assert!(schema.contains("HorusManifest"));
    assert!(schema.contains("package"));
    assert!(schema.contains("dependencies"));
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── TOML parsing ────────────────────────────────────────────────────

    #[test]
    fn parse_minimal_toml() {
        let toml = r#"
[package]
name = "my-robot"
version = "0.1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        assert_eq!(manifest.package.name, "my-robot");
        assert_eq!(manifest.package.version, "0.1.0");
        assert_eq!(manifest.package.edition, "1");
        assert!(manifest.dependencies.is_empty());
    }

    #[test]
    fn parse_full_toml() {
        let toml = r#"
[package]
name = "advanced-robot"
version = "1.0.0"
description = "A mobile robot controller"
authors = ["Jane Doe <jane@example.com>"]
license = "Apache-2.0"
language = "rust"
edition = "1"
repository = "https://github.com/org/robot"
package-type = "node"
categories = ["robotics", "navigation"]

[dependencies]
horus = "^0.2"
horus_library = "^0.2"

[dependencies.numpy]
version = "^1.24"
source = "pypi"

[dependencies.my-driver]
path = "../drivers/my-driver"

[dependencies.my-algo]
git = "https://github.com/org/algo.git"
tag = "v2.0"

[dev-dependencies]
proptest = "^1.4"

[drivers]
camera = "opencv"
lidar = "rplidar-a2"

[ignore]
files = ["debug_*.py"]
directories = ["old/"]
packages = ["ipython"]
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        assert_eq!(manifest.package.name, "advanced-robot");
        assert_eq!(manifest.package.package_type, Some(PackageType::Node));
        assert_eq!(manifest.dependencies.len(), 5); // horus, horus_library, numpy, my-driver, my-algo
        assert_eq!(manifest.dev_dependencies.len(), 1);
        assert_eq!(manifest.drivers.len(), 2);
        assert_eq!(manifest.ignore.files.len(), 1);
    }

    #[test]
    fn parse_simple_dependency() {
        let toml = r#"
[package]
name = "test"
version = "0.1.0"

[dependencies]
horus = "^0.2"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        let specs = manifest.dependencies_as_specs().unwrap();
        assert_eq!(specs.len(), 1);
        assert_eq!(specs[0].name, "horus");
        assert_eq!(specs[0].source, DependencySource::Registry);
    }

    #[test]
    fn parse_detailed_pypi_dependency() {
        let toml = r#"
[package]
name = "test"
version = "0.1.0"

[dependencies.numpy]
version = "^1.24"
source = "pypi"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        let specs = manifest.dependencies_as_specs().unwrap();
        assert_eq!(specs.len(), 1);
        assert_eq!(specs[0].name, "numpy");
        assert!(matches!(specs[0].source, DependencySource::Pip { .. }));
    }

    #[test]
    fn parse_path_dependency() {
        let toml = r#"
[package]
name = "test"
version = "0.1.0"

[dependencies.my-driver]
path = "../drivers/my-driver"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        let specs = manifest.dependencies_as_specs().unwrap();
        assert_eq!(specs.len(), 1);
        assert!(matches!(specs[0].source, DependencySource::Path(_)));
    }

    #[test]
    fn parse_git_dependency() {
        let toml = r#"
[package]
name = "test"
version = "0.1.0"

[dependencies.my-algo]
git = "https://github.com/org/algo.git"
branch = "main"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        let specs = manifest.dependencies_as_specs().unwrap();
        assert_eq!(specs.len(), 1);
        assert!(matches!(specs[0].source, DependencySource::Git { .. }));
    }

    #[test]
    fn parse_crates_io_dependency() {
        let toml = r#"
[package]
name = "test"
version = "0.1.0"

[dependencies.serde]
version = "1.0"
source = "crates-io"
features = ["derive"]
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        let specs = manifest.dependencies_as_specs().unwrap();
        assert_eq!(specs.len(), 1);
        assert_eq!(specs[0].source, DependencySource::CratesIO);
    }

    #[test]
    fn parse_system_dependency() {
        let toml = r#"
[package]
name = "test"
version = "0.1.0"

[dependencies.libusb]
source = "system"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        let specs = manifest.dependencies_as_specs().unwrap();
        assert_eq!(specs.len(), 1);
        assert_eq!(specs[0].source, DependencySource::System);
    }

    #[test]
    fn parse_star_and_latest_versions() {
        let toml = r#"
[package]
name = "test"
version = "0.1.0"

[dependencies]
a = "*"
b = "latest"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        let specs = manifest.dependencies_as_specs().unwrap();
        assert_eq!(specs.len(), 2);
        assert_eq!(specs[0].requirement, VersionReq::STAR);
        assert_eq!(specs[1].requirement, VersionReq::STAR);
    }

    #[test]
    fn horus_py_special_case() {
        let toml = r#"
[package]
name = "test"
version = "0.1.0"

[dependencies]
horus_py = "*"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        let specs = manifest.dependencies_as_specs().unwrap();
        assert_eq!(specs[0].name, "horus_py");
        assert!(matches!(
            specs[0].source,
            DependencySource::Pip { ref package_name } if package_name == "horus-robotics"
        ));
    }

    #[test]
    fn dev_dependencies_separate() {
        let toml = r#"
[package]
name = "test"
version = "0.1.0"

[dependencies]
horus = "^0.2"

[dev-dependencies]
proptest = "^1.4"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        assert_eq!(manifest.dependencies.len(), 1);
        assert_eq!(manifest.dev_dependencies.len(), 1);

        let all = manifest.all_dependencies_as_specs().unwrap();
        assert_eq!(all.len(), 2);
    }

    // ── Validation ──────────────────────────────────────────────────────

    #[test]
    fn validate_valid_manifest() {
        let toml = r#"
[package]
name = "my-robot"
version = "0.1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        assert!(manifest.validate().is_ok());
    }

    #[test]
    fn validate_rejects_short_name() {
        let toml = r#"
[package]
name = "x"
version = "0.1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        assert!(manifest.validate().is_err());
    }

    #[test]
    fn validate_rejects_reserved_name() {
        let toml = r#"
[package]
name = "horus"
version = "0.1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        assert!(manifest.validate().is_err());
    }

    #[test]
    fn validate_rejects_bad_version() {
        let toml = r#"
[package]
name = "my-robot"
version = "not-semver"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        assert!(manifest.validate().is_err());
    }

    #[test]
    fn validate_rejects_bad_dependency() {
        let toml = r#"
[package]
name = "my-robot"
version = "0.1.0"

[dependencies]
bad-dep = "not>>a>>version"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        assert!(manifest.validate().is_err());
    }

    // ── Round-trip ──────────────────────────────────────────────────────

    #[test]
    fn toml_roundtrip() {
        let manifest = HorusManifest {
            package: PackageInfo {
                name: "test-project".to_string(),
                version: "0.1.0".to_string(),
                description: Some("A test project".to_string()),
                authors: vec!["Test Author".to_string()],
                license: Some("MIT".to_string()),
                language: Some("rust".to_string()),
                edition: "1".to_string(),
                repository: None,
                package_type: None,
                categories: vec![],
            },
            dependencies: {
                let mut deps = BTreeMap::new();
                deps.insert("horus".to_string(), DependencyValue::Simple("^0.2".to_string()));
                deps
            },
            dev_dependencies: BTreeMap::new(),
            build_dependencies: BTreeMap::new(),
            drivers: BTreeMap::new(),
            ignore: IgnoreConfig::default(),
            enable: Vec::new(),
        };

        let serialized = toml::to_string_pretty(&manifest).unwrap();
        let deserialized: HorusManifest = toml::from_str(&serialized).unwrap();
        assert_eq!(deserialized.package.name, "test-project");
        assert_eq!(deserialized.dependencies.len(), 1);
    }

    // ── toml_edit operations ────────────────────────────────────────────

    #[test]
    fn add_and_remove_dependency() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("horus.toml");
        fs::write(
            &path,
            r#"[package]
name = "test"
version = "0.1.0"

[dependencies]
horus = "^0.2"
"#,
        )
        .unwrap();

        // Add
        add_dependency_to_file(
            &path,
            "numpy",
            &DependencyValue::Detailed(DetailedDependency {
                version: Some("^1.24".to_string()),
                source: Some(DepSource::Pypi),
                path: None, git: None, branch: None, tag: None, rev: None,
                features: vec![], target: None,
            }),
            "dependencies",
        )
        .unwrap();

        let content = fs::read_to_string(&path).unwrap();
        assert!(content.contains("numpy"));
        assert!(content.contains("horus")); // preserved

        // Remove
        let removed = remove_dependency_from_file(&path, "numpy", "dependencies").unwrap();
        assert!(removed);

        let content = fs::read_to_string(&path).unwrap();
        assert!(!content.contains("numpy"));
        assert!(content.contains("horus")); // still preserved
    }

    // ── E2E lifecycle ─────────────────────────────────────────────────

    #[test]
    fn e2e_lifecycle_create_load_edit_save_reload() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(HORUS_TOML);

        // 1. Create manifest programmatically and save
        let manifest = HorusManifest {
            package: PackageInfo {
                name: "lifecycle-test".to_string(),
                version: "0.1.0".to_string(),
                description: Some("E2E lifecycle test".to_string()),
                authors: vec![],
                license: None,
                language: Some("rust".to_string()),
                edition: "1".to_string(),
                repository: None,
                package_type: None,
                categories: vec![],
            },
            dependencies: {
                let mut deps = BTreeMap::new();
                deps.insert("horus".into(), DependencyValue::Simple("^0.2".into()));
                deps.insert("serde".into(), DependencyValue::Detailed(DetailedDependency {
                    version: Some("1.0".into()),
                    source: Some(DepSource::CratesIo),
                    features: vec!["derive".into()],
                    path: None, git: None, branch: None, tag: None, rev: None, target: None,
                }));
                deps
            },
            dev_dependencies: BTreeMap::new(),
            build_dependencies: BTreeMap::new(),
            drivers: {
                let mut d = BTreeMap::new();
                d.insert("camera".into(), DriverValue::Enabled(true));
                d.insert("lidar".into(), DriverValue::Backend("rplidar-a2".into()));
                d
            },
            ignore: IgnoreConfig::default(),
            enable: vec!["cuda".into()],
        };
        manifest.save_to(&path).unwrap();
        assert!(path.exists());

        // 2. Load from disk
        let (loaded, origin) = HorusManifest::load_from(&path).unwrap();
        assert!(matches!(origin, ManifestOrigin::Toml));
        assert_eq!(loaded.package.name, "lifecycle-test");
        assert_eq!(loaded.dependencies.len(), 2);
        assert_eq!(loaded.drivers.len(), 2);
        assert_eq!(loaded.enable, vec!["cuda"]);

        // 3. Validate
        let warnings = loaded.validate().unwrap();
        assert!(warnings.is_empty(), "unexpected warnings: {:?}", warnings);

        // 4. Convert to DependencySpecs
        let specs = loaded.dependencies_as_specs().unwrap();
        assert_eq!(specs.len(), 2);
        let serde_spec = specs.iter().find(|s| s.name == "serde").unwrap();
        assert_eq!(serde_spec.source, DependencySource::CratesIO);

        // 5. Edit via toml_edit: add a dependency
        add_dependency_to_file(
            &path, "tokio",
            &DependencyValue::Simple("^1.35".into()),
            "dependencies",
        ).unwrap();

        // 6. Reload and verify edit preserved existing data
        let (reloaded, _) = HorusManifest::load_from(&path).unwrap();
        assert_eq!(reloaded.dependencies.len(), 3);
        assert!(reloaded.dependencies.contains_key("tokio"));
        assert!(reloaded.dependencies.contains_key("horus"));
        assert!(reloaded.dependencies.contains_key("serde"));
        assert_eq!(reloaded.package.description.as_deref(), Some("E2E lifecycle test"));
        assert_eq!(reloaded.drivers.len(), 2);

        // 7. Remove a dependency
        let removed = remove_dependency_from_file(&path, "tokio", "dependencies").unwrap();
        assert!(removed);
        let (final_manifest, _) = HorusManifest::load_from(&path).unwrap();
        assert_eq!(final_manifest.dependencies.len(), 2);
        assert!(!final_manifest.dependencies.contains_key("tokio"));
    }
}
