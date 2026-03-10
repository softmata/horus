//! Typed manifest for HORUS projects (`horus.toml`).
//!
//! `horus.toml` is a **config-only** manifest for horus-specific settings.
//! Dependencies live in native build files (`Cargo.toml`, `pyproject.toml`).
//! Language is auto-detected from which build files exist in the project.
//!
//! ## File format
//!
//! ```toml
//! [package]
//! name = "my-robot"
//! version = "0.1.0"
//! description = "A mobile robot controller"
//! package-type = "node"
//! categories = ["robotics", "navigation"]
//!
//! [drivers]
//! camera = "opencv"
//! lidar = "rplidar-a2"
//!
//! [ignore]
//! files = ["debug_*.py"]
//!
//! enable = ["cuda"]
//! ```

use anyhow::{anyhow, Context, Result};
use colored::*;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::fmt;
use std::fs;
use std::path::{Path, PathBuf};

#[cfg(feature = "schema")]
use schemars::JsonSchema;

// ─── Constants ───────────────────────────────────────────────────────────────

/// Primary manifest filename.
pub const HORUS_TOML: &str = "horus.toml";

// ─── Language detection ──────────────────────────────────────────────────────

/// Project language, auto-detected from native build files.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Language {
    Rust,
    Python,
    Cpp,
    Ros2,
}

impl fmt::Display for Language {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Rust => write!(f, "rust"),
            Self::Python => write!(f, "python"),
            Self::Cpp => write!(f, "cpp"),
            Self::Ros2 => write!(f, "ros2"),
        }
    }
}

/// Detect project languages from native build files in the given directory.
///
/// Returns all detected languages. An empty Vec means no recognized build files exist.
pub fn detect_languages(project_dir: &Path) -> Vec<Language> {
    let mut languages = Vec::new();

    if project_dir.join("Cargo.toml").exists() {
        languages.push(Language::Rust);
    }
    if project_dir.join("pyproject.toml").exists()
        || project_dir.join("setup.py").exists()
        || project_dir.join("requirements.txt").exists()
    {
        languages.push(Language::Python);
    }
    if project_dir.join("CMakeLists.txt").exists() {
        languages.push(Language::Cpp);
    }
    if project_dir.join("package.xml").exists() {
        languages.push(Language::Ros2);
    }

    languages
}

/// Detect languages or return an error with a helpful message.
pub fn detect_languages_or_error(project_dir: &Path) -> Result<Vec<Language>> {
    let languages = detect_languages(project_dir);
    if languages.is_empty() {
        Err(anyhow!(
            "No build files found in {}. Expected one of: {}, {}, {}, {}",
            project_dir.display(),
            "Cargo.toml".cyan(),
            "pyproject.toml".cyan(),
            "CMakeLists.txt".cyan(),
            "package.xml".cyan(),
        ))
    } else {
        Ok(languages)
    }
}

// ─── Top-level manifest ─────────────────────────────────────────────────────

/// The typed representation of a `horus.toml` manifest.
///
/// Contains horus-specific config only. Dependencies live in native build files
/// (`Cargo.toml`, `pyproject.toml`, etc.).
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HorusManifest {
    /// `[package]` -- project metadata.
    pub package: PackageInfo,

    /// `[drivers]` -- hardware driver configuration.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub drivers: BTreeMap<String, DriverValue>,

    /// `[ignore]` -- patterns to exclude during scanning.
    #[serde(default, skip_serializing_if = "IgnoreConfig::is_empty")]
    pub ignore: IgnoreConfig,

    /// `enable = ["cuda", "editor"]` -- capabilities to enable.
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

    /// Manifest schema edition for forward compatibility.
    #[serde(
        default = "default_edition",
        skip_serializing_if = "is_default_edition"
    )]
    pub edition: String,

    /// Source code repository URL.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub repository: Option<String>,

    /// Package type for the registry.
    #[serde(
        default,
        skip_serializing_if = "Option::is_none",
        rename = "package-type"
    )]
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

/// Patterns to exclude during file scanning.
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

    /// Save manifest as TOML to a file path.
    pub fn save_to(&self, path: &Path) -> Result<()> {
        let content =
            toml::to_string_pretty(self).context("Failed to serialize manifest to TOML")?;
        fs::write(path, content).with_context(|| format!("Failed to write {}", path.display()))?;
        Ok(())
    }

    // ── Validation ──────────────────────────────────────────────────────

    /// Validate the manifest and return warnings (errors are returned as Err).
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
        if !self.package.name.chars().all(|c| {
            c.is_ascii_lowercase()
                || c.is_ascii_digit()
                || c == '-'
                || c == '_'
                || c == '@'
                || c == '/'
        }) {
            errors.push(format!(
                "Package name '{}' must contain only lowercase letters, digits, hyphens, and underscores",
                self.package.name
            ));
        }

        // Reserved names
        let reserved = [
            "horus", "core", "std", "lib", "test", "main", "admin", "api", "root", "system",
            "internal", "config", "setup", "install",
        ];
        if reserved.contains(&self.package.name.as_str()) {
            errors.push(format!("Package name '{}' is reserved", self.package.name));
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
    assert!(schema.contains("drivers"));
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
        assert!(manifest.drivers.is_empty());
        assert!(manifest.enable.is_empty());
    }

    #[test]
    fn parse_full_toml() {
        let toml = r#"
enable = ["cuda", "editor"]

[package]
name = "advanced-robot"
version = "1.0.0"
description = "A mobile robot controller"
authors = ["Jane Doe <jane@example.com>"]
license = "Apache-2.0"
edition = "1"
repository = "https://github.com/org/robot"
package-type = "node"
categories = ["robotics", "navigation"]

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
        assert_eq!(manifest.package.categories, vec!["robotics", "navigation"]);
        assert_eq!(manifest.drivers.len(), 2);
        assert_eq!(manifest.ignore.files.len(), 1);
        assert_eq!(manifest.ignore.directories.len(), 1);
        assert_eq!(manifest.ignore.packages.len(), 1);
        assert_eq!(manifest.enable, vec!["cuda", "editor"]);
    }

    #[test]
    fn parse_drivers_both_forms() {
        let toml = r#"
[package]
name = "test"
version = "0.1.0"

[drivers]
camera = "opencv"
gps = true
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        assert_eq!(manifest.drivers.len(), 2);
        assert!(matches!(manifest.drivers["camera"], DriverValue::Backend(ref s) if s == "opencv"));
        assert!(matches!(
            manifest.drivers["gps"],
            DriverValue::Enabled(true)
        ));
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
        manifest.validate().unwrap();
    }

    #[test]
    fn validate_rejects_short_name() {
        let toml = r#"
[package]
name = "x"
version = "0.1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        manifest.validate().unwrap_err();
    }

    #[test]
    fn validate_rejects_reserved_name() {
        let toml = r#"
[package]
name = "horus"
version = "0.1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        manifest.validate().unwrap_err();
    }

    #[test]
    fn validate_rejects_bad_version() {
        let toml = r#"
[package]
name = "my-robot"
version = "not-semver"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        manifest.validate().unwrap_err();
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
                edition: "1".to_string(),
                repository: None,
                package_type: None,
                categories: vec![],
            },
            drivers: {
                let mut d = BTreeMap::new();
                d.insert("camera".into(), DriverValue::Backend("opencv".into()));
                d
            },
            ignore: IgnoreConfig::default(),
            enable: vec!["cuda".into()],
        };

        let serialized = toml::to_string_pretty(&manifest).unwrap();
        let deserialized: HorusManifest = toml::from_str(&serialized).unwrap();
        assert_eq!(deserialized.package.name, "test-project");
        assert_eq!(deserialized.drivers.len(), 1);
        assert_eq!(deserialized.enable, vec!["cuda"]);
    }

    // ── E2E lifecycle ─────────────────────────────────────────────────

    #[test]
    fn e2e_lifecycle_create_load_save_reload() {
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
                edition: "1".to_string(),
                repository: None,
                package_type: Some(PackageType::Node),
                categories: vec!["robotics".into()],
            },
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
        assert_eq!(loaded.drivers.len(), 2);
        assert_eq!(loaded.enable, vec!["cuda"]);
        assert_eq!(loaded.package.package_type, Some(PackageType::Node));

        // 3. Validate
        let warnings = loaded.validate().unwrap();
        assert!(warnings.is_empty(), "unexpected warnings: {:?}", warnings);
    }

    // ── Language detection ───────────────────────────────────────────────

    #[test]
    fn detect_rust_project() {
        let dir = tempfile::tempdir().unwrap();
        fs::write(
            dir.path().join("Cargo.toml"),
            "[package]\nname = \"test\"\n",
        )
        .unwrap();
        let langs = detect_languages(dir.path());
        assert_eq!(langs, vec![Language::Rust]);
    }

    #[test]
    fn detect_python_project_pyproject() {
        let dir = tempfile::tempdir().unwrap();
        fs::write(
            dir.path().join("pyproject.toml"),
            "[project]\nname = \"test\"\n",
        )
        .unwrap();
        let langs = detect_languages(dir.path());
        assert_eq!(langs, vec![Language::Python]);
    }

    #[test]
    fn detect_python_project_requirements() {
        let dir = tempfile::tempdir().unwrap();
        fs::write(dir.path().join("requirements.txt"), "numpy\n").unwrap();
        let langs = detect_languages(dir.path());
        assert_eq!(langs, vec![Language::Python]);
    }

    #[test]
    fn detect_mixed_project() {
        let dir = tempfile::tempdir().unwrap();
        fs::write(dir.path().join("Cargo.toml"), "[package]\n").unwrap();
        fs::write(dir.path().join("pyproject.toml"), "[project]\n").unwrap();
        let langs = detect_languages(dir.path());
        assert_eq!(langs, vec![Language::Rust, Language::Python]);
    }

    #[test]
    fn detect_cpp_project() {
        let dir = tempfile::tempdir().unwrap();
        fs::write(
            dir.path().join("CMakeLists.txt"),
            "cmake_minimum_required(VERSION 3.20)\n",
        )
        .unwrap();
        let langs = detect_languages(dir.path());
        assert_eq!(langs, vec![Language::Cpp]);
    }

    #[test]
    fn detect_ros2_project() {
        let dir = tempfile::tempdir().unwrap();
        fs::write(dir.path().join("package.xml"), "<package>\n").unwrap();
        let langs = detect_languages(dir.path());
        assert_eq!(langs, vec![Language::Ros2]);
    }

    #[test]
    fn detect_no_build_files() {
        let dir = tempfile::tempdir().unwrap();
        let langs = detect_languages(dir.path());
        assert!(langs.is_empty());
    }

    #[test]
    fn detect_languages_or_error_with_no_files() {
        let dir = tempfile::tempdir().unwrap();
        detect_languages_or_error(dir.path()).unwrap_err();
    }

    #[test]
    fn detect_languages_or_error_with_cargo() {
        let dir = tempfile::tempdir().unwrap();
        fs::write(dir.path().join("Cargo.toml"), "[package]\n").unwrap();
        let langs = detect_languages_or_error(dir.path()).unwrap();
        assert_eq!(langs, vec![Language::Rust]);
    }

    #[test]
    fn old_format_with_dependencies_silently_ignored() {
        let old_toml = r#"
[package]
name = "my-robot"
version = "0.1.0"

[dependencies]
serde = "1.0"
tokio = { version = "1.0", features = ["full"] }

[dev-dependencies]
criterion = "0.5"
"#;
        let manifest: HorusManifest = toml::from_str(old_toml).unwrap();
        assert_eq!(manifest.package.name, "my-robot");
        assert_eq!(manifest.package.version, "0.1.0");
        // [dependencies] and [dev-dependencies] are silently ignored — no error
    }
}
