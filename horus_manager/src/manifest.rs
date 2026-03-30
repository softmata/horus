//! Typed manifest for HORUS projects (`horus.toml`).
//!
//! `horus.toml` is the **single source of truth** for a horus project:
//! metadata, dependencies, drivers, ignores, scripts, and feature flags.
//!
//! Dependencies are declared here and native build files (`.horus/Cargo.toml`,
//! `.horus/pyproject.toml`) are generated from them automatically.
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
//! [dependencies]
//! horus_library = "0.2.0"
//! serde = { version = "1.0", features = ["derive"], source = "crates.io" }
//! numpy = { version = ">=1.24", source = "pypi" }
//!
//! [dev-dependencies]
//! criterion = { version = "0.5", source = "crates.io" }
//!
//! [drivers]
//! camera = "opencv"
//! lidar = "rplidar-a2"
//!
//! [scripts]
//! sim = "horus sim start --world warehouse"
//! deploy-pi = "horus deploy robot@192.168.1.5 --release"
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

    // Rust: root Cargo.toml (legacy), .horus/Cargo.toml (generated), or .rs source files
    let has_rust = project_dir.join("Cargo.toml").exists()
        || project_dir.join(".horus/Cargo.toml").exists()
        || has_files_with_ext(project_dir, "rs");
    if has_rust {
        languages.push(Language::Rust);
    }

    // Python: root pyproject.toml (legacy), .horus/pyproject.toml (generated), setup.py,
    // requirements.txt, or .py source files
    let has_python = project_dir.join("pyproject.toml").exists()
        || project_dir.join(".horus/pyproject.toml").exists()
        || project_dir.join("setup.py").exists()
        || project_dir.join("requirements.txt").exists()
        || has_files_with_ext(project_dir, "py");
    if has_python {
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

/// Check if a directory contains files with the given extension (root + src/).
fn has_files_with_ext(dir: &Path, ext: &str) -> bool {
    let check = |d: &Path| -> bool {
        if let Ok(entries) = std::fs::read_dir(d) {
            for entry in entries.flatten() {
                if let Some(e) = entry.path().extension() {
                    if e == ext {
                        return true;
                    }
                }
            }
        }
        false
    };
    check(dir) || check(&dir.join("src"))
}

/// Detect languages or return an error with a helpful message.
#[cfg(test)]
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
/// Single source of truth for a horus project: metadata, dependencies,
/// drivers, scripts, ignore patterns, and feature flags.
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HorusManifest {
    /// `[package]` -- project metadata. Required for single-package projects.
    /// For virtual workspaces (only `[workspace]`), this defaults to empty.
    #[serde(default)]
    pub package: PackageInfo,

    /// `[workspace]` -- multi-crate workspace configuration (optional).
    /// When present, this manifest is a workspace root.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub workspace: Option<WorkspaceConfig>,

    /// `[robot]` -- robot model configuration (optional).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub robot: Option<RobotConfig>,

    /// `[dependencies]` -- project dependencies (all sources).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub dependencies: BTreeMap<String, DependencyValue>,

    /// `[dev-dependencies]` -- dev-only dependencies (not included in publish/deploy).
    #[serde(
        default,
        skip_serializing_if = "BTreeMap::is_empty",
        rename = "dev-dependencies"
    )]
    pub dev_dependencies: BTreeMap<String, DependencyValue>,

    /// `[sim-dependencies]` -- simulation asset dependencies (auto-installed on `horus sim3d`).
    ///
    /// Package types: robot, world, sensor-preset, task, actuator-preset.
    /// Resolved from horus-registry and cached in `.horus/packages/`.
    #[serde(
        default,
        skip_serializing_if = "BTreeMap::is_empty",
        rename = "sim-dependencies"
    )]
    pub sim_dependencies: BTreeMap<String, DependencyValue>,

    /// `[hardware]` -- hardware node configuration (preferred).
    /// Falls back to `[drivers]` for backward compatibility.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub hardware: BTreeMap<String, DriverValue>,

    /// `[drivers]` -- legacy hardware driver configuration (use [hardware] instead).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub drivers: BTreeMap<String, DriverValue>,

    /// `[sim-drivers]` -- legacy simulation overrides (use `sim = true` in [hardware] instead).
    #[serde(
        default,
        skip_serializing_if = "BTreeMap::is_empty",
        rename = "sim-drivers"
    )]
    pub sim_drivers: BTreeMap<String, DriverValue>,

    /// `[scripts]` -- custom project commands (like npm scripts / just).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub scripts: BTreeMap<String, String>,

    /// `[ignore]` -- patterns to exclude during scanning.
    #[serde(default, skip_serializing_if = "IgnoreConfig::is_empty")]
    pub ignore: IgnoreConfig,

    /// `enable = ["cuda", "editor"]` -- capabilities to enable.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub enable: Vec<String>,

    /// `[cpp]` -- C++ build configuration (compiler override, cmake args, toolchain).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cpp: Option<CppConfig>,

    /// `[hooks]` -- pre/post action hooks for run/build/test.
    #[serde(default, skip_serializing_if = "HooksConfig::is_empty")]
    pub hooks: HooksConfig,

    /// `[network]` -- LAN replication configuration (optional).
    /// When present, configures horus_net for cross-machine topic sharing.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub network: Option<NetworkConfig>,
}

/// Network replication configuration for horus_net.
///
/// Parsed from `[network]` in horus.toml. All fields optional — defaults are
/// sensible for zero-config LAN operation.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct NetworkConfig {
    /// Enable/disable networking. Default: true. Env override: HORUS_NO_NETWORK=1.
    #[serde(default)]
    pub enabled: Option<bool>,
    /// Import control: "auto" (default), "deny", or list of topic patterns.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub import: Option<toml::Value>,
    /// Export deny patterns (e.g., ["camera.*", "debug.*"]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub deny_export: Option<Vec<String>>,
    /// Shared secret for peer filtering (NOT security).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub secret: Option<String>,
    /// Enabled optimizers (e.g., ["fusion", "delta"]).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub optimize: Option<Vec<String>>,
    /// Safety heartbeat settings.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub safety: Option<NetworkSafetyConfig>,
}

/// Safety heartbeat configuration within [network].
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct NetworkSafetyConfig {
    /// Heartbeat interval in ms (default: 50).
    pub heartbeat_ms: Option<u64>,
    /// Missed heartbeats before link-dead (default: 3).
    pub missed_threshold: Option<u32>,
    /// Action on link loss: "warn", "safe_state", "stop" (default: "warn").
    pub on_link_lost: Option<String>,
}

#[cfg(test)]
impl HorusManifest {
    /// Get all crates.io dependencies (for Cargo.toml generation).
    pub fn crates_io_deps(&self) -> BTreeMap<&str, &DependencyValue> {
        self.dependencies
            .iter()
            .filter(|(_, v)| v.is_crates_io())
            .map(|(k, v)| (k.as_str(), v))
            .collect()
    }

    /// Get all PyPI dependencies (for pyproject.toml generation).
    pub fn pypi_deps(&self) -> BTreeMap<&str, &DependencyValue> {
        self.dependencies
            .iter()
            .filter(|(_, v)| v.is_pypi())
            .map(|(k, v)| (k.as_str(), v))
            .collect()
    }

    /// Get all registry dependencies.
    pub fn registry_deps(&self) -> BTreeMap<&str, &DependencyValue> {
        self.dependencies
            .iter()
            .filter(|(_, v)| v.is_registry())
            .map(|(k, v)| (k.as_str(), v))
            .collect()
    }

    /// Get all path dependencies.
    pub fn path_deps(&self) -> BTreeMap<&str, &DependencyValue> {
        self.dependencies
            .iter()
            .filter(|(_, v)| v.is_path())
            .map(|(k, v)| (k.as_str(), v))
            .collect()
    }

    /// Detect languages from declared dependencies.
    ///
    /// If any dep has source = "crates.io", the project uses Rust.
    /// If any dep has source = "pypi", the project uses Python.
    pub fn languages_from_deps(&self) -> Vec<Language> {
        let mut langs = Vec::new();
        let has_rust = self.dependencies.values().any(|v| v.is_crates_io());
        let has_python = self.dependencies.values().any(|v| v.is_pypi());
        if has_rust {
            langs.push(Language::Rust);
        }
        if has_python {
            langs.push(Language::Python);
        }
        langs
    }
}

// ─── [robot] ───────────────────────────────────────────────────────────────

/// Robot model configuration under `[robot]`.
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotConfig {
    /// Robot name used in topic naming (e.g., `"turtlebot"`).
    pub name: String,
    /// Path to URDF/Xacro file, relative to project root.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Simulator plugin to use with `horus run --sim`.
    /// When absent, defaults to `"sim3d"` at runtime.
    /// Examples: `"sim3d"`, `"mujoco"`, `"isaac"`, `"gazebo"`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub simulator: Option<String>,
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

    /// C++ language standard (e.g., `"c++17"`, `"c++20"`, `"c++23"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub standard: Option<String>,

    /// Rust edition override (e.g., `"2021"`). Used by cargo_gen.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rust_edition: Option<String>,

    /// Crate target type: "bin" (default), "lib", or "both".
    #[serde(
        default,
        skip_serializing_if = "TargetType::is_default",
        rename = "type"
    )]
    pub target_type: TargetType,
}

impl Default for PackageInfo {
    fn default() -> Self {
        Self {
            name: String::new(),
            version: "0.0.0".to_string(),
            description: None,
            authors: vec![],
            license: None,
            edition: default_edition(),
            repository: None,
            package_type: None,
            categories: vec![],
            standard: None,
            rust_edition: None,
            target_type: TargetType::default(),
        }
    }
}

/// Crate target type.
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum TargetType {
    #[default]
    Bin,
    Lib,
    Both,
}

impl TargetType {
    fn is_default(&self) -> bool {
        *self == Self::Bin
    }
}

/// Multi-crate workspace configuration under `[workspace]`.
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorkspaceConfig {
    /// Glob patterns for workspace members (e.g., `["crates/*"]`).
    #[serde(default)]
    pub members: Vec<String>,

    /// Glob patterns to exclude from membership.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub exclude: Vec<String>,

    /// Shared dependencies inherited by members via `workspace = true`.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub dependencies: BTreeMap<String, DependencyValue>,
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

// ─── C++ configuration ──────────────────────────────────────────────────────

/// C++ build configuration under `[cpp]` in horus.toml.
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CppConfig {
    /// Override the C++ compiler (e.g., `"clang++"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub compiler: Option<String>,

    /// Additional CMake arguments injected into the generated CMakeLists.txt.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub cmake_args: Vec<String>,

    /// Cross-compilation toolchain target (e.g., `"aarch64"`, `"armv7"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub toolchain: Option<String>,
}

// ─── Dependency types ────────────────────────────────────────────────────────

/// Source of a dependency.
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
#[derive(Default)]
pub enum DepSource {
    /// Horus registry (default when no source specified).
    #[default]
    Registry,
    /// Rust crates from crates.io.
    #[serde(rename = "crates.io")]
    CratesIo,
    /// Python packages from PyPI.
    #[serde(rename = "pypi")]
    PyPI,
    /// System package (apt, brew, etc.).
    System,
    /// Local path dependency.
    Path,
    /// Git repository dependency.
    Git,
}

impl std::fmt::Display for DepSource {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Registry => write!(f, "registry"),
            Self::CratesIo => write!(f, "crates.io"),
            Self::PyPI => write!(f, "pypi"),
            Self::System => write!(f, "system"),
            Self::Path => write!(f, "path"),
            Self::Git => write!(f, "git"),
        }
    }
}

/// Detailed dependency specification.
///
/// Used when a dependency needs more than just a version string.
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct DetailedDependency {
    /// Version requirement (e.g., "1.0", ">=1.24", "^2.0").
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub version: Option<String>,

    /// Dependency source.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<DepSource>,

    /// Cargo/Python features to enable.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub features: Vec<String>,

    /// Whether this dependency is optional.
    #[serde(default, skip_serializing_if = "is_false")]
    pub optional: bool,

    /// Local path (for source = "path").
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,

    /// Git repository URL (for source = "git").
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub git: Option<String>,

    /// Git branch (for source = "git").
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub branch: Option<String>,

    /// Git tag (for source = "git").
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tag: Option<String>,

    /// Git revision (for source = "git").
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rev: Option<String>,

    /// Apt package name for system dependencies (e.g., `"libeigen3-dev"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub apt: Option<String>,

    /// CMake package name for `find_package()` (e.g., `"Eigen3"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cmake_package: Option<String>,

    /// Language this dependency belongs to (e.g., `"cpp"`, `"rust"`, `"python"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lang: Option<String>,

    /// Inherit version/source/features from `[workspace.dependencies]`.
    #[serde(default, skip_serializing_if = "is_false")]
    pub workspace: bool,
}

fn is_false(b: &bool) -> bool {
    !*b
}

impl DetailedDependency {
    /// Resolve the effective source of this dependency.
    ///
    /// If `source` is explicitly set, use it.
    /// Otherwise, infer from `path` / `git` fields, defaulting to Registry.
    pub fn effective_source(&self) -> DepSource {
        if let Some(ref s) = self.source {
            return s.clone();
        }
        if self.path.is_some() {
            return DepSource::Path;
        }
        if self.git.is_some() {
            return DepSource::Git;
        }
        DepSource::Registry
    }
}

/// Dependency value: either a simple version string or a detailed table.
///
/// - Simple: `serde = "1.0"` → `DependencyValue::Simple("1.0")`
/// - Detailed: `serde = { version = "1.0", features = ["derive"] }` → `DependencyValue::Detailed(...)`
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum DependencyValue {
    /// Version string only (e.g., `"1.0"`, `">=1.24"`, `"^2.0"`).
    /// Defaults to horus registry source.
    Simple(String),

    /// Full dependency specification.
    Detailed(DetailedDependency),
}

impl DependencyValue {
    /// Get the version string, if any.
    pub fn version(&self) -> Option<&str> {
        match self {
            Self::Simple(v) => Some(v),
            Self::Detailed(d) => d.version.as_deref(),
        }
    }

    /// Get the effective source of this dependency.
    pub fn effective_source(&self) -> DepSource {
        match self {
            Self::Simple(_) => DepSource::Registry,
            Self::Detailed(d) => d.effective_source(),
        }
    }

    /// Get features, if any.
    pub fn features(&self) -> &[String] {
        match self {
            Self::Simple(_) => &[],
            Self::Detailed(d) => &d.features,
        }
    }

    /// Check if this is a crates.io dependency.
    pub fn is_crates_io(&self) -> bool {
        self.effective_source() == DepSource::CratesIo
    }

    /// Check if this is a PyPI dependency.
    pub fn is_pypi(&self) -> bool {
        self.effective_source() == DepSource::PyPI
    }

    /// Check if this is a registry dependency.
    pub fn is_registry(&self) -> bool {
        self.effective_source() == DepSource::Registry
    }

    /// Check if this is a path dependency.
    pub fn is_path(&self) -> bool {
        self.effective_source() == DepSource::Path
    }
}

// ─── Driver values ──────────────────────────────────────────────────────────

/// Driver configuration value.
///
/// Three forms:
/// - Config table: `[drivers.arm]` with `terra`/`package`/`node` key + params
/// - Simple string: `camera = "opencv"` (backend name → feature flags)
/// - Enable bool: `camera = true` (enable with default backend)
///
/// The `Config` variant must be first — `#[serde(untagged)]` tries variants
/// in order, and TOML tables must match before scalar strings/bools.
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum DriverValue {
    /// Config table: `[drivers.arm]` with terra/package/node key + params.
    Config(DriverTableConfig),
    /// Backend name string, e.g., `"opencv"`.
    Backend(String),
    /// Enable with default backend.
    Enabled(bool),
}

/// Structured driver configuration from a `[drivers.NAME]` TOML table.
///
/// Exactly one of `terra`, `package`, or `node` should be present to identify
/// the driver source. All other keys are captured in `params` and passed to
/// the driver factory at runtime.
///
/// # Examples
///
/// ```toml
/// # Terra driver (pre-built hardware support)
/// [drivers.arm]
/// terra = "dynamixel"
/// port = "/dev/ttyUSB0"
/// baudrate = 1000000
/// servo_ids = [1, 2, 3, 4, 5, 6]
///
/// # Registry package (community/vendor driver)
/// [drivers.force_sensor]
/// package = "horus-driver-ati-netft"
/// address = "192.168.1.100"
///
/// # Local code (user's own driver registered via register_driver!)
/// [drivers.conveyor]
/// node = "ConveyorDriver"
/// port = "/dev/ttyACM0"
/// baudrate = 57600
/// ```
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DriverTableConfig {
    /// Node type name — the new unified source key (e.g., `"dynamixel"`, `"rplidar"`).
    /// Replaces the 6 legacy keys (terra, package, node, crate, pip, exec).
    #[serde(default, skip_serializing_if = "Option::is_none", rename = "use")]
    pub use_name: Option<String>,
    /// Simulation flag — when true, replaced by sim stub when `horus run --sim`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sim: Option<bool>,
    /// Terra driver shortname (legacy — use `use` instead).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub terra: Option<String>,
    /// Registry package name (e.g., `"horus-driver-ati-netft"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub package: Option<String>,
    /// Local node struct name registered via `register_driver!` (e.g., `"ConveyorDriver"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub node: Option<String>,
    /// HORUS topic for sensor data output (e.g., `"sensors/imu"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub topic: Option<String>,
    /// HORUS topic for joint/state output (e.g., `"arm/joint_states"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub topic_state: Option<String>,
    /// HORUS topic for command input (e.g., `"arm/joint_commands"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub topic_command: Option<String>,
    /// Rust crate name from crates.io — `crate = "rplidar-driver"`.
    #[serde(default, skip_serializing_if = "Option::is_none", rename = "crate")]
    pub crate_name: Option<String>,
    /// Crate source identifier (e.g., `"crates-io"`). Implied when `crate` is present.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub source: Option<String>,
    /// Python package name from PyPI — `pip = "adafruit-bno055"`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pip: Option<String>,
    /// External binary path — `exec = "./realsense_bridge"`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exec: Option<String>,
    /// Simulated by a simulator plugin — `simulated = true`.
    #[serde(default, skip_serializing_if = "Option::is_none", alias = "sim3d")]
    pub simulated: Option<bool>,
    /// CLI arguments for exec drivers.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub args: Option<Vec<String>>,
    /// All remaining keys — passed to the driver factory as `DriverParams`.
    #[serde(flatten)]
    pub params: std::collections::HashMap<String, toml::Value>,
}

// ─── [hooks] ────────────────────────────────────────────────────────────────

/// Pre/post action hooks that run automatically before/after commands.
#[cfg_attr(feature = "schema", derive(JsonSchema))]
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct HooksConfig {
    /// Hooks to run before `horus run` (e.g., `["fmt", "lint"]`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub pre_run: Vec<String>,

    /// Hooks to run before `horus build`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub pre_build: Vec<String>,

    /// Hooks to run before `horus test`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub pre_test: Vec<String>,

    /// Hooks to run after `horus test`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub post_test: Vec<String>,
}

impl HooksConfig {
    pub fn is_empty(&self) -> bool {
        self.pre_run.is_empty()
            && self.pre_build.is_empty()
            && self.pre_test.is_empty()
            && self.post_test.is_empty()
    }
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

impl HorusManifest {
    /// Load manifest from a TOML file path.
    pub fn load_from(path: &Path) -> Result<Self> {
        let content = fs::read_to_string(path)
            .with_context(|| format!("Failed to read manifest: {}", path.display()))?;

        let manifest: HorusManifest = toml::from_str(&content)
            .with_context(|| format!("Failed to parse {}", path.display()))?;
        Ok(manifest)
    }

    /// Search upward from the current directory to find and load a manifest.
    ///
    /// Searches for `horus.toml` in the current directory and parent directories.
    /// Returns the manifest and the directory it was found in.
    pub fn find_and_load() -> Result<(Self, PathBuf)> {
        Self::find_and_load_from(std::env::current_dir()?)
    }

    /// Search upward from `start_dir` to find and load a manifest.
    pub fn find_and_load_from(start_dir: PathBuf) -> Result<(Self, PathBuf)> {
        let mut current = start_dir;

        for _ in 0..10 {
            let toml_path = current.join(HORUS_TOML);
            if toml_path.exists() {
                let manifest = Self::load_from(&toml_path)?;
                return Ok((manifest, current));
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

        // Virtual workspace: skip package validation, validate workspace config
        if self.is_virtual_workspace() {
            let ws = self.workspace.as_ref().unwrap();
            if ws.members.is_empty() {
                errors.push("Workspace has no members".to_string());
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
            return Ok(warnings);
        }

        // No workspace and no package name → invalid
        if self.package.name.is_empty() && self.workspace.is_none() {
            errors.push("Missing [package] section with a name".to_string());
        }

        // Name validation
        if !self.package.name.is_empty()
            && (self.package.name.len() < 2 || self.package.name.len() > 64)
        {
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
                "Package name '{}' must contain only lowercase letters, digits, hyphens, underscores, '@', and '/'",
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

    // ── Workspace helpers ──────────────────────────────────────────────────

    /// Whether this manifest represents a workspace (virtual or root-package).
    pub fn is_workspace(&self) -> bool {
        self.workspace.is_some()
    }

    /// Whether this is a virtual workspace (has `[workspace]` but no `[package]` name).
    pub fn is_virtual_workspace(&self) -> bool {
        self.workspace.is_some() && self.package.name.is_empty()
    }
}

// ─── Workspace resolution ───────────────────────────────────────────────────

/// Expand workspace member glob patterns and load each member's `horus.toml`.
///
/// Returns `(relative_path, member_manifest)` pairs.
pub fn resolve_workspace_members(
    workspace: &WorkspaceConfig,
    project_dir: &Path,
) -> Result<Vec<(PathBuf, HorusManifest)>> {
    let mut members = Vec::new();
    let mut seen = std::collections::HashSet::new();

    for pattern in &workspace.members {
        let full_pattern = project_dir.join(pattern);
        let pattern_str = full_pattern.to_string_lossy().to_string();

        for entry in glob::glob(&pattern_str)
            .with_context(|| format!("Invalid glob pattern: {}", pattern))?
        {
            let entry = entry?;
            if !entry.is_dir() {
                continue;
            }

            let member_toml = entry.join(HORUS_TOML);
            if !member_toml.exists() {
                continue;
            }

            let relative = entry
                .strip_prefix(project_dir)
                .unwrap_or(&entry)
                .to_path_buf();
            let rel_str = relative.to_string_lossy().to_string();

            // Check exclude patterns
            let excluded = workspace.exclude.iter().any(|exc| {
                glob::Pattern::new(exc)
                    .map(|p| p.matches(&rel_str))
                    .unwrap_or(false)
            });
            if excluded {
                continue;
            }

            if seen.insert(entry.clone()) {
                let manifest = HorusManifest::load_from(&member_toml)?;
                members.push((relative, manifest));
            }
        }
    }

    Ok(members)
}

/// Resolve a `workspace = true` dependency from `[workspace.dependencies]`.
///
/// Merges member-specific features on top of the workspace dep.
pub fn resolve_workspace_dep(
    name: &str,
    member_dep: &DetailedDependency,
    workspace_deps: &BTreeMap<String, DependencyValue>,
) -> Result<DependencyValue> {
    let ws_dep = workspace_deps.get(name).ok_or_else(|| {
        anyhow!(
            "Dependency '{}' uses `workspace = true` but not found in [workspace.dependencies]",
            name
        )
    })?;

    if member_dep.features.is_empty() {
        return Ok(ws_dep.clone());
    }

    // Merge member features on top of workspace features
    let mut resolved = ws_dep.clone();
    match &mut resolved {
        DependencyValue::Simple(v) => {
            resolved = DependencyValue::Detailed(DetailedDependency {
                version: Some(v.clone()),
                features: member_dep.features.clone(),
                ..DetailedDependency::default()
            });
        }
        DependencyValue::Detailed(d) => {
            for f in &member_dep.features {
                if !d.features.contains(f) {
                    d.features.push(f.clone());
                }
            }
        }
    }
    Ok(resolved)
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
                standard: None,
                rust_edition: None,
                target_type: TargetType::default(),
            },
            workspace: None,
            robot: None,
            dependencies: BTreeMap::new(),
            dev_dependencies: BTreeMap::new(),
            sim_dependencies: BTreeMap::new(),
            hardware: BTreeMap::new(),
            drivers: {
                let mut d = BTreeMap::new();
                d.insert("camera".into(), DriverValue::Backend("opencv".into()));
                d
            },
            sim_drivers: BTreeMap::new(),
            scripts: BTreeMap::new(),
            ignore: IgnoreConfig::default(),
            enable: vec!["cuda".into()],
            cpp: None,
            hooks: Default::default(),
            network: None,
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
                standard: None,
                rust_edition: None,
                target_type: TargetType::default(),
            },
            workspace: None,
            robot: None,
            dependencies: BTreeMap::new(),
            dev_dependencies: BTreeMap::new(),
            sim_dependencies: BTreeMap::new(),
            hardware: BTreeMap::new(),
            drivers: {
                let mut d = BTreeMap::new();
                d.insert("camera".into(), DriverValue::Enabled(true));
                d.insert("lidar".into(), DriverValue::Backend("rplidar-a2".into()));
                d
            },
            sim_drivers: BTreeMap::new(),
            scripts: BTreeMap::new(),
            ignore: IgnoreConfig::default(),
            enable: vec!["cuda".into()],
            cpp: None,
            hooks: Default::default(),
            network: None,
        };
        manifest.save_to(&path).unwrap();
        assert!(path.exists());

        // 2. Load from disk
        let loaded = HorusManifest::load_from(&path).unwrap();
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

    // ── Dependency parsing ────────────────────────────────────────────

    #[test]
    fn parse_dependencies_simple() {
        let toml_str = r#"
[package]
name = "my-robot"
version = "0.1.0"

[dependencies]
horus_library = "0.2.0"
my-sensor = "^2.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(manifest.dependencies.len(), 2);

        match &manifest.dependencies["horus_library"] {
            DependencyValue::Simple(v) => assert_eq!(v, "0.2.0"),
            _ => panic!("expected simple dep"),
        }
    }

    #[test]
    fn parse_dependencies_detailed() {
        let toml_str = r#"
[package]
name = "my-robot"
version = "0.1.0"

[dependencies]
serde = { version = "1.0", features = ["derive"], source = "crates.io" }
numpy = { version = ">=1.24", source = "pypi" }
my-lib = { path = "../my-lib" }
some-fork = { git = "https://github.com/org/repo", branch = "main" }
libudev-dev = { source = "system" }
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(manifest.dependencies.len(), 5);

        // serde is crates.io
        assert!(manifest.dependencies["serde"].is_crates_io());
        assert_eq!(manifest.dependencies["serde"].features(), &["derive"]);

        // numpy is pypi
        assert!(manifest.dependencies["numpy"].is_pypi());
        assert_eq!(manifest.dependencies["numpy"].version(), Some(">=1.24"));

        // my-lib is path
        assert!(manifest.dependencies["my-lib"].is_path());

        // libudev-dev is system
        assert_eq!(
            manifest.dependencies["libudev-dev"].effective_source(),
            DepSource::System
        );
    }

    #[test]
    fn parse_dev_dependencies() {
        let toml_str = r#"
[package]
name = "my-robot"
version = "0.1.0"

[dev-dependencies]
criterion = { version = "0.5", source = "crates.io" }
pytest = { version = ">=7.0", source = "pypi" }
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(manifest.dev_dependencies.len(), 2);
        assert!(manifest.dev_dependencies["criterion"].is_crates_io());
        assert!(manifest.dev_dependencies["pytest"].is_pypi());
    }

    #[test]
    fn parse_scripts() {
        let toml_str = r#"
[package]
name = "my-robot"
version = "0.1.0"

[scripts]
sim = "horus sim start --world warehouse"
deploy-pi = "horus deploy robot@192.168.1.5 --release"
test-hw = "cargo test --features hardware"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(manifest.scripts.len(), 3);
        assert_eq!(manifest.scripts["sim"], "horus sim start --world warehouse");
    }

    #[test]
    fn dep_source_inference() {
        // path dep inferred from path field
        let dep = DependencyValue::Detailed(DetailedDependency {
            version: None,
            source: None,
            features: vec![],
            optional: false,
            path: Some("../my-lib".to_string()),
            git: None,
            branch: None,
            tag: None,
            rev: None,
            apt: None,
            cmake_package: None,
            lang: None,
            workspace: false,
        });
        assert_eq!(dep.effective_source(), DepSource::Path);

        // git dep inferred from git field
        let dep = DependencyValue::Detailed(DetailedDependency {
            version: None,
            source: None,
            features: vec![],
            optional: false,
            path: None,
            git: Some("https://github.com/org/repo".to_string()),
            branch: Some("main".to_string()),
            tag: None,
            rev: None,
            apt: None,
            cmake_package: None,
            lang: None,
            workspace: false,
        });
        assert_eq!(dep.effective_source(), DepSource::Git);

        // explicit source overrides inference
        let dep = DependencyValue::Detailed(DetailedDependency {
            version: Some("1.0".to_string()),
            source: Some(DepSource::CratesIo),
            features: vec![],
            optional: false,
            path: None,
            git: None,
            branch: None,
            tag: None,
            rev: None,
            apt: None,
            cmake_package: None,
            lang: None,
            workspace: false,
        });
        assert_eq!(dep.effective_source(), DepSource::CratesIo);

        // simple dep defaults to registry
        let dep = DependencyValue::Simple("1.0".to_string());
        assert_eq!(dep.effective_source(), DepSource::Registry);
    }

    #[test]
    fn languages_from_deps_detection() {
        let toml_str = r#"
[package]
name = "mixed-bot"
version = "0.1.0"

[dependencies]
serde = { version = "1.0", source = "crates.io" }
numpy = { version = ">=1.24", source = "pypi" }
my-pkg = "^2.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        let langs = manifest.languages_from_deps();
        assert!(langs.contains(&Language::Rust));
        assert!(langs.contains(&Language::Python));
    }

    #[test]
    fn filter_deps_by_source() {
        let toml_str = r#"
[package]
name = "mixed-bot"
version = "0.1.0"

[dependencies]
horus_library = "0.2.0"
serde = { version = "1.0", source = "crates.io" }
tokio = { version = "1.0", features = ["full"], source = "crates.io" }
numpy = { version = ">=1.24", source = "pypi" }
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(manifest.crates_io_deps().len(), 2);
        assert_eq!(manifest.pypi_deps().len(), 1);
        assert_eq!(manifest.registry_deps().len(), 1);
    }

    #[test]
    fn backward_compat_no_deps_section() {
        // Old horus.toml without [dependencies] should still load fine
        let toml_str = r#"
[package]
name = "old-project"
version = "0.1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert!(manifest.dependencies.is_empty());
        assert!(manifest.dev_dependencies.is_empty());
        assert!(manifest.scripts.is_empty());
    }

    // ── DependencyValue helpers ──────────────────────────────────────

    #[test]
    fn dep_value_simple_version() {
        let dep = DependencyValue::Simple("1.0".to_string());
        assert_eq!(dep.version(), Some("1.0"));
        assert!(dep.features().is_empty());
        assert!(dep.is_registry());
        assert!(!dep.is_crates_io());
        assert!(!dep.is_pypi());
        assert!(!dep.is_path());
    }

    #[test]
    fn dep_value_detailed_optional() {
        let dep = DependencyValue::Detailed(DetailedDependency {
            version: Some("1.0".to_string()),
            source: Some(DepSource::CratesIo),
            features: vec!["derive".to_string(), "serde".to_string()],
            optional: true,
            path: None,
            git: None,
            branch: None,
            tag: None,
            rev: None,
            apt: None,
            cmake_package: None,
            lang: None,
            workspace: false,
        });
        assert_eq!(dep.version(), Some("1.0"));
        assert_eq!(dep.features().len(), 2);
        assert!(dep.is_crates_io());
        match dep {
            DependencyValue::Detailed(d) => assert!(d.optional),
            _ => panic!("expected Detailed"),
        }
    }

    #[test]
    fn dep_value_no_version() {
        let dep = DependencyValue::Detailed(DetailedDependency {
            version: None,
            source: Some(DepSource::System),
            features: vec![],
            optional: false,
            path: None,
            git: None,
            branch: None,
            tag: None,
            rev: None,
            apt: None,
            cmake_package: None,
            lang: None,
            workspace: false,
        });
        assert!(dep.version().is_none());
        assert_eq!(dep.effective_source(), DepSource::System);
    }

    // ── Display impls ─────────────────────────────────────────────────

    #[test]
    fn language_display() {
        assert_eq!(format!("{}", Language::Rust), "rust");
        assert_eq!(format!("{}", Language::Python), "python");
        assert_eq!(format!("{}", Language::Cpp), "cpp");
        assert_eq!(format!("{}", Language::Ros2), "ros2");
    }

    #[test]
    fn dep_source_display() {
        assert_eq!(format!("{}", DepSource::Registry), "registry");
        assert_eq!(format!("{}", DepSource::CratesIo), "crates.io");
        assert_eq!(format!("{}", DepSource::PyPI), "pypi");
        assert_eq!(format!("{}", DepSource::System), "system");
        assert_eq!(format!("{}", DepSource::Path), "path");
        assert_eq!(format!("{}", DepSource::Git), "git");
    }

    #[test]
    fn dep_source_default_is_registry() {
        assert_eq!(DepSource::default(), DepSource::Registry);
    }

    #[test]
    fn package_type_display() {
        assert_eq!(format!("{}", PackageType::Node), "node");
        assert_eq!(format!("{}", PackageType::Driver), "driver");
        assert_eq!(format!("{}", PackageType::Tool), "tool");
        assert_eq!(format!("{}", PackageType::Algorithm), "algorithm");
        assert_eq!(format!("{}", PackageType::Model), "model");
        assert_eq!(format!("{}", PackageType::Message), "message");
        assert_eq!(format!("{}", PackageType::App), "app");
    }

    // ── File I/O ──────────────────────────────────────────────────────

    #[test]
    fn load_from_nonexistent_fails() {
        let result = HorusManifest::load_from(Path::new("/tmp/nonexistent_horus_toml_xyz.toml"));
        assert!(result.is_err());
    }

    #[test]
    fn load_from_invalid_toml_fails() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(HORUS_TOML);
        fs::write(&path, "this is not valid toml {{{{").unwrap();
        let result = HorusManifest::load_from(&path);
        assert!(result.is_err());
    }

    #[test]
    fn save_and_reload() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(HORUS_TOML);

        let manifest = HorusManifest {
            package: PackageInfo {
                name: "save-test".to_string(),
                version: "1.2.3".to_string(),
                description: None,
                authors: vec![],
                license: None,
                edition: "1".to_string(),
                repository: None,
                package_type: None,
                categories: vec![],
                standard: None,
                rust_edition: None,
                target_type: TargetType::default(),
            },
            workspace: None,
            robot: None,
            dependencies: BTreeMap::new(),
            dev_dependencies: BTreeMap::new(),
            sim_dependencies: BTreeMap::new(),
            hardware: BTreeMap::new(),
            drivers: BTreeMap::new(),
            sim_drivers: BTreeMap::new(),
            scripts: BTreeMap::new(),
            ignore: IgnoreConfig::default(),
            enable: vec![],
            cpp: None,
            hooks: Default::default(),
            network: None,
        };

        manifest.save_to(&path).unwrap();
        let loaded = HorusManifest::load_from(&path).unwrap();
        assert_eq!(loaded.package.name, "save-test");
        assert_eq!(loaded.package.version, "1.2.3");
    }

    // ── Validation edge cases ─────────────────────────────────────────

    #[test]
    fn validate_rejects_empty_name() {
        let toml = r#"
[package]
name = ""
version = "0.1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        manifest.validate().unwrap_err();
    }

    #[test]
    fn validate_name_at_min_length() {
        let toml = r#"
[package]
name = "ab"
version = "0.1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        // 2-char name should pass
        assert!(manifest.validate().is_ok());
    }

    #[test]
    fn validate_rejects_core_reserved() {
        let toml = r#"
[package]
name = "core"
version = "0.1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml).unwrap();
        manifest.validate().unwrap_err();
    }

    // ── path_deps filter ──────────────────────────────────────────────

    #[test]
    fn path_deps_filter() {
        let toml_str = r#"
[package]
name = "dep-filter"
version = "0.1.0"

[dependencies]
my-lib = { path = "../my-lib" }
serde = { version = "1.0", source = "crates.io" }
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        let path = manifest.path_deps();
        assert_eq!(path.len(), 1);
        assert!(path.contains_key("my-lib"));
    }

    #[test]
    fn dep_roundtrip_serialization() {
        let toml_str = r#"
[package]
name = "roundtrip-test"
version = "0.1.0"

[dependencies]
horus_library = "0.2.0"
serde = { version = "1.0", features = ["derive"], source = "crates.io" }
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        let serialized = toml::to_string_pretty(&manifest).unwrap();
        let reloaded: HorusManifest = toml::from_str(&serialized).unwrap();
        assert_eq!(reloaded.dependencies.len(), 2);
        assert!(reloaded.dependencies["serde"].is_crates_io());
        assert_eq!(reloaded.dependencies["serde"].features(), &["derive"]);
    }

    // ── Battle-testing: edge cases & robustness ──────────────────────

    /// 1. Parse a manifest with ALL sections populated.
    #[test]
    fn battle_parse_all_sections_populated() {
        let toml_str = r#"
enable = ["cuda", "editor", "profiling"]

[package]
name = "mega-robot"
version = "2.3.1"
description = "All sections populated"
authors = ["Alice <alice@example.com>", "Bob <bob@example.com>"]
license = "MIT"
edition = "1"
repository = "https://github.com/softmata/mega-robot"
package-type = "app"
categories = ["robotics", "planning", "simulation"]

[dependencies]
horus_library = "0.2.0"
serde = { version = "1.0", features = ["derive"], source = "crates.io" }
numpy = { version = ">=1.24", source = "pypi" }
my-lib = { path = "../my-lib" }
some-fork = { git = "https://github.com/org/repo", branch = "dev" }
libudev-dev = { source = "system" }
my-registry-pkg = { version = "^3.0", source = "registry" }

[dev-dependencies]
criterion = { version = "0.5", source = "crates.io" }
pytest = { version = ">=7.0", source = "pypi" }

[drivers]
camera = "opencv"
lidar = "rplidar-a2"
gps = true
imu = false

[scripts]
sim = "horus sim start --world warehouse"
deploy-pi = "horus deploy robot@192.168.1.5 --release"
test-hw = "cargo test --features hardware"
lint = "cargo clippy -- -D warnings"

[ignore]
files = ["debug_*.py", "*.bak"]
directories = ["old/", "experiments/"]
packages = ["ipython", "debugpy"]
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();

        // Package
        assert_eq!(manifest.package.name, "mega-robot");
        assert_eq!(manifest.package.version, "2.3.1");
        assert_eq!(
            manifest.package.description.as_deref(),
            Some("All sections populated")
        );
        assert_eq!(manifest.package.authors.len(), 2);
        assert_eq!(manifest.package.license.as_deref(), Some("MIT"));
        assert_eq!(
            manifest.package.repository.as_deref(),
            Some("https://github.com/softmata/mega-robot")
        );
        assert_eq!(manifest.package.package_type, Some(PackageType::App));
        assert_eq!(manifest.package.categories.len(), 3);

        // Dependencies — 7 total with mixed sources
        assert_eq!(manifest.dependencies.len(), 7);
        assert!(manifest.dependencies["horus_library"].is_registry());
        assert!(manifest.dependencies["serde"].is_crates_io());
        assert!(manifest.dependencies["numpy"].is_pypi());
        assert!(manifest.dependencies["my-lib"].is_path());
        assert_eq!(
            manifest.dependencies["some-fork"].effective_source(),
            DepSource::Git
        );
        assert_eq!(
            manifest.dependencies["libudev-dev"].effective_source(),
            DepSource::System
        );
        assert!(manifest.dependencies["my-registry-pkg"].is_registry());

        // Dev deps
        assert_eq!(manifest.dev_dependencies.len(), 2);

        // Drivers — 4 total
        assert_eq!(manifest.drivers.len(), 4);
        assert!(matches!(
            manifest.drivers["camera"],
            DriverValue::Backend(_)
        ));
        assert!(matches!(
            manifest.drivers["gps"],
            DriverValue::Enabled(true)
        ));
        assert!(matches!(
            manifest.drivers["imu"],
            DriverValue::Enabled(false)
        ));

        // Scripts
        assert_eq!(manifest.scripts.len(), 4);
        assert_eq!(manifest.scripts["lint"], "cargo clippy -- -D warnings");

        // Ignore
        assert_eq!(manifest.ignore.files.len(), 2);
        assert_eq!(manifest.ignore.directories.len(), 2);
        assert_eq!(manifest.ignore.packages.len(), 2);

        // Enable
        assert_eq!(manifest.enable, vec!["cuda", "editor", "profiling"]);

        // Validate
        let warnings = manifest.validate().unwrap();
        assert!(warnings.is_empty());
    }

    /// 2. Validate rejects names with uppercase letters.
    #[test]
    fn battle_validate_rejects_uppercase_name() {
        let toml_str = r#"
[package]
name = "MyRobot"
version = "0.1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        let err = manifest.validate().unwrap_err();
        let msg = format!("{}", err);
        assert!(
            msg.contains("lowercase"),
            "Error should mention lowercase requirement: {}",
            msg
        );
    }

    #[test]
    fn battle_validate_rejects_mixed_case_name() {
        let toml_str = r#"
[package]
name = "my-Robot"
version = "0.1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert!(manifest.validate().is_err());
    }

    /// 3. Validate rejects names with dots, spaces, or special characters.
    #[test]
    fn battle_validate_rejects_name_with_dot() {
        let toml_str = r#"
[package]
name = "my.robot"
version = "0.1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert!(manifest.validate().is_err());
    }

    #[test]
    fn battle_validate_rejects_name_with_space() {
        let toml_str = r#"
[package]
name = "my robot"
version = "0.1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert!(manifest.validate().is_err());
    }

    #[test]
    fn battle_validate_rejects_name_with_special_chars() {
        for name in &[
            "my!robot", "my#robot", "my$robot", "my%robot", "my+robot", "my=robot",
        ] {
            let toml_str = format!(
                r#"
[package]
name = "{}"
version = "0.1.0"
"#,
                name
            );
            let manifest: HorusManifest = toml::from_str(&toml_str).unwrap();
            assert!(
                manifest.validate().is_err(),
                "Name '{}' should be rejected",
                name
            );
        }
    }

    /// 4. Dependencies with ALL source types parse correctly.
    #[test]
    fn battle_all_dep_source_types() {
        let toml_str = r#"
[package]
name = "all-sources"
version = "0.1.0"

[dependencies]
reg-pkg = "1.0"
crate-pkg = { version = "2.0", source = "crates.io" }
pypi-pkg = { version = ">=3.0", source = "pypi" }
path-pkg = { path = "../local-lib" }
git-pkg = { git = "https://github.com/org/repo" }
sys-pkg = { source = "system" }
explicit-registry = { version = "^4.0", source = "registry" }
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(manifest.dependencies.len(), 7);

        assert_eq!(
            manifest.dependencies["reg-pkg"].effective_source(),
            DepSource::Registry
        );
        assert_eq!(
            manifest.dependencies["crate-pkg"].effective_source(),
            DepSource::CratesIo
        );
        assert_eq!(
            manifest.dependencies["pypi-pkg"].effective_source(),
            DepSource::PyPI
        );
        assert_eq!(
            manifest.dependencies["path-pkg"].effective_source(),
            DepSource::Path
        );
        assert_eq!(
            manifest.dependencies["git-pkg"].effective_source(),
            DepSource::Git
        );
        assert_eq!(
            manifest.dependencies["sys-pkg"].effective_source(),
            DepSource::System
        );
        assert_eq!(
            manifest.dependencies["explicit-registry"].effective_source(),
            DepSource::Registry
        );
    }

    /// 5. Optional deps parse and round-trip correctly.
    #[test]
    fn battle_optional_dep_roundtrip() {
        let toml_str = r#"
[package]
name = "optional-test"
version = "0.1.0"

[dependencies]
serde = { version = "1.0", source = "crates.io", optional = true }
tokio = { version = "1.0", source = "crates.io", optional = false }
simple-dep = "2.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();

        // Check optional flag parsed
        match &manifest.dependencies["serde"] {
            DependencyValue::Detailed(d) => assert!(d.optional, "serde should be optional"),
            _ => panic!("serde should be Detailed"),
        }
        match &manifest.dependencies["tokio"] {
            DependencyValue::Detailed(d) => assert!(!d.optional, "tokio should not be optional"),
            _ => panic!("tokio should be Detailed"),
        }

        // Round-trip
        let serialized = toml::to_string_pretty(&manifest).unwrap();
        let reloaded: HorusManifest = toml::from_str(&serialized).unwrap();

        match &reloaded.dependencies["serde"] {
            DependencyValue::Detailed(d) => {
                assert!(d.optional, "serde should be optional after roundtrip");
                assert_eq!(d.version.as_deref(), Some("1.0"));
                assert_eq!(d.source, Some(DepSource::CratesIo));
            }
            _ => panic!("serde should remain Detailed after roundtrip"),
        }
        // `optional = false` is skip_serializing_if = is_false, so it gets omitted
        // and re-parsed as default (false) — still correct
        match &reloaded.dependencies["tokio"] {
            DependencyValue::Detailed(d) => assert!(!d.optional),
            _ => panic!("tokio should remain Detailed after roundtrip"),
        }
    }

    /// 6. Git deps with tag, rev, branch all parse correctly.
    #[test]
    fn battle_git_deps_tag_rev_branch() {
        let toml_str = r#"
[package]
name = "git-variants"
version = "0.1.0"

[dependencies]
branch-dep = { git = "https://github.com/org/branch-repo", branch = "develop" }
tag-dep = { git = "https://github.com/org/tag-repo", tag = "v1.2.3" }
rev-dep = { git = "https://github.com/org/rev-repo", rev = "abc123def456" }
bare-git = { git = "https://github.com/org/bare-repo" }
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();

        // All are git deps
        for name in &["branch-dep", "tag-dep", "rev-dep", "bare-git"] {
            assert_eq!(
                manifest.dependencies[*name].effective_source(),
                DepSource::Git,
                "{} should be Git source",
                name
            );
        }

        // Check specific git fields
        match &manifest.dependencies["branch-dep"] {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.git.as_deref(), Some("https://github.com/org/branch-repo"));
                assert_eq!(d.branch.as_deref(), Some("develop"));
                assert!(d.tag.is_none());
                assert!(d.rev.is_none());
            }
            _ => panic!("expected Detailed"),
        }
        match &manifest.dependencies["tag-dep"] {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.tag.as_deref(), Some("v1.2.3"));
                assert!(d.branch.is_none());
                assert!(d.rev.is_none());
            }
            _ => panic!("expected Detailed"),
        }
        match &manifest.dependencies["rev-dep"] {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.rev.as_deref(), Some("abc123def456"));
                assert!(d.branch.is_none());
                assert!(d.tag.is_none());
            }
            _ => panic!("expected Detailed"),
        }
        match &manifest.dependencies["bare-git"] {
            DependencyValue::Detailed(d) => {
                assert!(d.branch.is_none());
                assert!(d.tag.is_none());
                assert!(d.rev.is_none());
            }
            _ => panic!("expected Detailed"),
        }
    }

    /// 7. `find_and_load_from` searches upward and finds horus.toml in parent.
    #[test]
    fn battle_find_and_load_from_parent_directory() {
        let root = tempfile::tempdir().unwrap();
        let parent_dir = root.path();
        let child_dir = parent_dir.join("src").join("nested");
        fs::create_dir_all(&child_dir).unwrap();

        // Put horus.toml in the parent (root)
        let toml_content = r#"
[package]
name = "found-in-parent"
version = "0.1.0"
"#;
        fs::write(parent_dir.join(HORUS_TOML), toml_content).unwrap();

        // Search from nested child
        let (manifest, found_dir) = HorusManifest::find_and_load_from(child_dir).unwrap();
        assert_eq!(manifest.package.name, "found-in-parent");
        assert_eq!(found_dir, parent_dir);
    }

    /// 8. `find_and_load_from` returns error when no horus.toml within 10 levels.
    #[test]
    fn battle_find_and_load_from_not_found() {
        let root = tempfile::tempdir().unwrap();
        // Create a deeply nested directory (no horus.toml anywhere)
        let mut deep = root.path().to_path_buf();
        for i in 0..12 {
            deep = deep.join(format!("level{}", i));
        }
        fs::create_dir_all(&deep).unwrap();

        let result = HorusManifest::find_and_load_from(deep);
        assert!(result.is_err());
        let err_msg = format!("{}", result.unwrap_err());
        assert!(
            err_msg.contains("No horus.toml found"),
            "Expected 'No horus.toml found' in error: {}",
            err_msg
        );
    }

    /// 9. Filter methods: crates_io_deps, pypi_deps, registry_deps, path_deps.
    #[test]
    fn battle_filter_deps_comprehensive() {
        let toml_str = r#"
[package]
name = "filter-test"
version = "0.1.0"

[dependencies]
reg-a = "1.0"
reg-b = { version = "2.0", source = "registry" }
crate-a = { version = "1.0", source = "crates.io" }
crate-b = { version = "2.0", source = "crates.io", features = ["full"] }
crate-c = { version = "3.0", source = "crates.io" }
pypi-a = { version = ">=1.0", source = "pypi" }
pypi-b = { version = ">=2.0", source = "pypi" }
path-a = { path = "../lib-a" }
path-b = { path = "../lib-b" }
path-c = { path = "../lib-c" }
path-d = { path = "../lib-d" }
git-dep = { git = "https://github.com/org/repo" }
sys-dep = { source = "system" }
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(manifest.dependencies.len(), 13);

        let crates = manifest.crates_io_deps();
        assert_eq!(crates.len(), 3);
        assert!(crates.contains_key("crate-a"));
        assert!(crates.contains_key("crate-b"));
        assert!(crates.contains_key("crate-c"));

        let pypi = manifest.pypi_deps();
        assert_eq!(pypi.len(), 2);
        assert!(pypi.contains_key("pypi-a"));
        assert!(pypi.contains_key("pypi-b"));

        let registry = manifest.registry_deps();
        assert_eq!(registry.len(), 2);
        assert!(registry.contains_key("reg-a"));
        assert!(registry.contains_key("reg-b"));

        let path = manifest.path_deps();
        assert_eq!(path.len(), 4);
        assert!(path.contains_key("path-a"));
        assert!(path.contains_key("path-b"));
        assert!(path.contains_key("path-c"));
        assert!(path.contains_key("path-d"));

        // git and system deps should not appear in any of the above filters
        assert!(!crates.contains_key("git-dep"));
        assert!(!pypi.contains_key("git-dep"));
        assert!(!registry.contains_key("git-dep"));
        assert!(!path.contains_key("git-dep"));
        assert!(!crates.contains_key("sys-dep"));
    }

    /// 10. Save -> Load round-trip preserves all fields including optional ones.
    #[test]
    fn battle_full_roundtrip_preserves_all_fields() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(HORUS_TOML);

        let manifest = HorusManifest {
            package: PackageInfo {
                name: "roundtrip-full".to_string(),
                version: "3.2.1".to_string(),
                description: Some("Full roundtrip test".to_string()),
                authors: vec![
                    "Alice <alice@example.com>".to_string(),
                    "Bob <bob@example.com>".to_string(),
                ],
                license: Some("Apache-2.0".to_string()),
                edition: "1".to_string(),
                repository: Some("https://github.com/softmata/roundtrip".to_string()),
                package_type: Some(PackageType::Algorithm),
                categories: vec!["planning".to_string(), "ml".to_string()],
                standard: None,
                rust_edition: None,
                target_type: TargetType::default(),
            },
            workspace: None,
            robot: None,
            dependencies: {
                let mut deps = BTreeMap::new();
                deps.insert(
                    "simple-reg".to_string(),
                    DependencyValue::Simple("1.0".to_string()),
                );
                deps.insert(
                    "crate-dep".to_string(),
                    DependencyValue::Detailed(DetailedDependency {
                        version: Some("2.0".to_string()),
                        source: Some(DepSource::CratesIo),
                        features: vec!["feat-a".to_string(), "feat-b".to_string()],
                        optional: true,
                        path: None,
                        git: None,
                        branch: None,
                        tag: None,
                        rev: None,
                        apt: None,
                        cmake_package: None,
                        lang: None,
                        workspace: false,
                    }),
                );
                deps.insert(
                    "git-dep".to_string(),
                    DependencyValue::Detailed(DetailedDependency {
                        version: None,
                        source: Some(DepSource::Git),
                        features: vec![],
                        optional: false,
                        path: None,
                        git: Some("https://github.com/org/repo".to_string()),
                        branch: None,
                        tag: Some("v1.0.0".to_string()),
                        rev: None,
                        apt: None,
                        cmake_package: None,
                        lang: None,
                        workspace: false,
                    }),
                );
                deps.insert(
                    "path-dep".to_string(),
                    DependencyValue::Detailed(DetailedDependency {
                        version: None,
                        source: None,
                        features: vec![],
                        optional: false,
                        path: Some("../local".to_string()),
                        git: None,
                        branch: None,
                        tag: None,
                        rev: None,
                        apt: None,
                        cmake_package: None,
                        lang: None,
                        workspace: false,
                    }),
                );
                deps
            },
            dev_dependencies: {
                let mut dd = BTreeMap::new();
                dd.insert(
                    "test-crate".to_string(),
                    DependencyValue::Detailed(DetailedDependency {
                        version: Some("0.5".to_string()),
                        source: Some(DepSource::CratesIo),
                        features: vec![],
                        optional: false,
                        path: None,
                        git: None,
                        branch: None,
                        tag: None,
                        rev: None,
                        apt: None,
                        cmake_package: None,
                        lang: None,
                        workspace: false,
                    }),
                );
                dd
            },
            sim_dependencies: BTreeMap::new(),
            hardware: BTreeMap::new(),
            drivers: {
                let mut d = BTreeMap::new();
                d.insert(
                    "camera".to_string(),
                    DriverValue::Backend("opencv".to_string()),
                );
                d.insert("gps".to_string(), DriverValue::Enabled(true));
                d
            },
            sim_drivers: BTreeMap::new(),
            scripts: {
                let mut s = BTreeMap::new();
                s.insert("build".to_string(), "cargo build --release".to_string());
                s.insert("test".to_string(), "cargo test".to_string());
                s
            },
            ignore: IgnoreConfig {
                files: vec!["*.bak".to_string(), "debug_*".to_string()],
                directories: vec!["tmp/".to_string()],
                packages: vec!["ipython".to_string()],
            },
            enable: vec!["cuda".to_string(), "profiling".to_string()],
            cpp: None,
            hooks: Default::default(),
            network: None,
        };

        manifest.save_to(&path).unwrap();
        let loaded = HorusManifest::load_from(&path).unwrap();

        // Package fields
        assert_eq!(loaded.package.name, "roundtrip-full");
        assert_eq!(loaded.package.version, "3.2.1");
        assert_eq!(
            loaded.package.description.as_deref(),
            Some("Full roundtrip test")
        );
        assert_eq!(loaded.package.authors.len(), 2);
        assert_eq!(loaded.package.license.as_deref(), Some("Apache-2.0"));
        assert_eq!(
            loaded.package.repository.as_deref(),
            Some("https://github.com/softmata/roundtrip")
        );
        assert_eq!(loaded.package.package_type, Some(PackageType::Algorithm));
        assert_eq!(loaded.package.categories, vec!["planning", "ml"]);

        // Dependencies preserved
        assert_eq!(loaded.dependencies.len(), 4);
        assert!(loaded.dependencies["simple-reg"].is_registry());
        assert!(loaded.dependencies["crate-dep"].is_crates_io());
        match &loaded.dependencies["crate-dep"] {
            DependencyValue::Detailed(d) => {
                assert!(d.optional);
                assert_eq!(d.features, vec!["feat-a", "feat-b"]);
            }
            _ => panic!("expected Detailed"),
        }
        match &loaded.dependencies["git-dep"] {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.tag.as_deref(), Some("v1.0.0"));
                assert_eq!(d.git.as_deref(), Some("https://github.com/org/repo"));
            }
            _ => panic!("expected Detailed"),
        }
        assert!(loaded.dependencies["path-dep"].is_path());

        // Dev deps
        assert_eq!(loaded.dev_dependencies.len(), 1);
        assert!(loaded.dev_dependencies["test-crate"].is_crates_io());

        // Drivers
        assert_eq!(loaded.drivers.len(), 2);

        // Scripts
        assert_eq!(loaded.scripts.len(), 2);
        assert_eq!(loaded.scripts["build"], "cargo build --release");

        // Ignore
        assert_eq!(loaded.ignore.files.len(), 2);
        assert_eq!(loaded.ignore.directories, vec!["tmp/"]);
        assert_eq!(loaded.ignore.packages, vec!["ipython"]);

        // Enable
        assert_eq!(loaded.enable, vec!["cuda", "profiling"]);
    }

    /// 11. Empty sections are omitted in serialized output (skip_serializing_if).
    #[test]
    fn battle_empty_sections_omitted_in_serialized() {
        let manifest = HorusManifest {
            package: PackageInfo {
                name: "minimal-out".to_string(),
                version: "0.1.0".to_string(),
                description: None,
                authors: vec![],
                license: None,
                edition: "1".to_string(),
                repository: None,
                package_type: None,
                categories: vec![],
                standard: None,
                rust_edition: None,
                target_type: TargetType::default(),
            },
            workspace: None,
            robot: None,
            dependencies: BTreeMap::new(),
            dev_dependencies: BTreeMap::new(),
            sim_dependencies: BTreeMap::new(),
            hardware: BTreeMap::new(),
            drivers: BTreeMap::new(),
            sim_drivers: BTreeMap::new(),
            scripts: BTreeMap::new(),
            ignore: IgnoreConfig::default(),
            enable: vec![],
            cpp: None,
            hooks: Default::default(),
            network: None,
        };

        let serialized = toml::to_string_pretty(&manifest).unwrap();

        // Empty sections should NOT appear
        assert!(
            !serialized.contains("[dependencies]"),
            "Empty dependencies should be omitted: {}",
            serialized
        );
        assert!(
            !serialized.contains("[dev-dependencies]"),
            "Empty dev-dependencies should be omitted: {}",
            serialized
        );
        assert!(
            !serialized.contains("[drivers]"),
            "Empty drivers should be omitted: {}",
            serialized
        );
        assert!(
            !serialized.contains("[scripts]"),
            "Empty scripts should be omitted: {}",
            serialized
        );
        assert!(
            !serialized.contains("[ignore]"),
            "Empty ignore should be omitted: {}",
            serialized
        );
        assert!(
            !serialized.contains("enable"),
            "Empty enable should be omitted: {}",
            serialized
        );

        // Optional package fields should also be omitted
        assert!(
            !serialized.contains("description"),
            "None description should be omitted: {}",
            serialized
        );
        assert!(
            !serialized.contains("license"),
            "None license should be omitted: {}",
            serialized
        );
        assert!(
            !serialized.contains("repository"),
            "None repository should be omitted: {}",
            serialized
        );
        assert!(
            !serialized.contains("package-type"),
            "None package_type should be omitted: {}",
            serialized
        );
        assert!(
            !serialized.contains("edition"),
            "Default edition '1' should be omitted: {}",
            serialized
        );

        // But [package], name, version must exist
        assert!(serialized.contains("[package]"));
        assert!(serialized.contains("minimal-out"));
        assert!(serialized.contains("0.1.0"));
    }

    /// 12. Manifest with no dependencies at all is valid.
    #[test]
    fn battle_no_deps_is_valid() {
        let toml_str = r#"
[package]
name = "no-deps"
version = "0.1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        let warnings = manifest.validate().unwrap();
        assert!(warnings.is_empty());
        assert!(manifest.dependencies.is_empty());
        assert!(manifest.dev_dependencies.is_empty());
        assert_eq!(manifest.crates_io_deps().len(), 0);
        assert_eq!(manifest.pypi_deps().len(), 0);
        assert_eq!(manifest.registry_deps().len(), 0);
        assert_eq!(manifest.path_deps().len(), 0);
        assert!(manifest.languages_from_deps().is_empty());
    }

    /// 13. Multiple drivers with both Backend and Enabled forms.
    #[test]
    fn battle_multiple_drivers_mixed_forms() {
        let toml_str = r#"
[package]
name = "multi-driver"
version = "0.1.0"

[drivers]
camera = "opencv"
lidar = "rplidar-a2"
gps = true
imu = false
depth-camera = "realsense"
motor-controller = true
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(manifest.drivers.len(), 6);

        // Backend forms
        match &manifest.drivers["camera"] {
            DriverValue::Backend(s) => assert_eq!(s, "opencv"),
            _ => panic!("expected Backend"),
        }
        match &manifest.drivers["lidar"] {
            DriverValue::Backend(s) => assert_eq!(s, "rplidar-a2"),
            _ => panic!("expected Backend"),
        }
        match &manifest.drivers["depth-camera"] {
            DriverValue::Backend(s) => assert_eq!(s, "realsense"),
            _ => panic!("expected Backend"),
        }

        // Enabled forms
        assert!(matches!(
            manifest.drivers["gps"],
            DriverValue::Enabled(true)
        ));
        assert!(matches!(
            manifest.drivers["imu"],
            DriverValue::Enabled(false)
        ));
        assert!(matches!(
            manifest.drivers["motor-controller"],
            DriverValue::Enabled(true)
        ));

        // Round-trip drivers
        let serialized = toml::to_string_pretty(&manifest).unwrap();
        let reloaded: HorusManifest = toml::from_str(&serialized).unwrap();
        assert_eq!(reloaded.drivers.len(), 6);
        assert!(matches!(
            reloaded.drivers["camera"],
            DriverValue::Backend(_)
        ));
        assert!(matches!(
            reloaded.drivers["gps"],
            DriverValue::Enabled(true)
        ));
    }

    // ── Driver table config (DriverTableConfig) ──────────────────────

    /// Parse [drivers.arm] table with terra key + params.
    #[test]
    fn parse_driver_table_config_terra() {
        let toml_str = r#"
[package]
name = "test"
version = "0.1.0"

[drivers.arm]
terra = "dynamixel"
port = "/dev/ttyUSB0"
baudrate = 1000000
servo_ids = [1, 2, 3, 4, 5, 6]
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(manifest.drivers.len(), 1);
        match &manifest.drivers["arm"] {
            DriverValue::Config(cfg) => {
                assert_eq!(cfg.terra.as_deref(), Some("dynamixel"));
                assert!(cfg.package.is_none());
                assert!(cfg.node.is_none());
                assert_eq!(
                    cfg.params.get("port").and_then(|v| v.as_str()),
                    Some("/dev/ttyUSB0")
                );
                assert_eq!(
                    cfg.params.get("baudrate").and_then(|v| v.as_integer()),
                    Some(1000000)
                );
                let ids = cfg.params.get("servo_ids").unwrap().as_array().unwrap();
                assert_eq!(ids.len(), 6);
            }
            other => panic!("expected Config, got {:?}", other),
        }
    }

    /// Parse [drivers.sensor] table with package key.
    #[test]
    fn parse_driver_table_config_package() {
        let toml_str = r#"
[package]
name = "test"
version = "0.1.0"

[drivers.force_sensor]
package = "horus-driver-ati-netft"
address = "192.168.1.100"
filter_hz = 500
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        match &manifest.drivers["force_sensor"] {
            DriverValue::Config(cfg) => {
                assert!(cfg.terra.is_none());
                assert_eq!(cfg.package.as_deref(), Some("horus-driver-ati-netft"));
                assert!(cfg.node.is_none());
                assert_eq!(
                    cfg.params.get("address").and_then(|v| v.as_str()),
                    Some("192.168.1.100")
                );
                assert_eq!(
                    cfg.params.get("filter_hz").and_then(|v| v.as_integer()),
                    Some(500)
                );
            }
            other => panic!("expected Config, got {:?}", other),
        }
    }

    /// Parse [drivers.conveyor] table with node key (local driver).
    #[test]
    fn parse_driver_table_config_node() {
        let toml_str = r#"
[package]
name = "test"
version = "0.1.0"

[drivers.conveyor]
node = "ConveyorDriver"
port = "/dev/ttyACM0"
baudrate = 57600
belt_length_mm = 2400
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        match &manifest.drivers["conveyor"] {
            DriverValue::Config(cfg) => {
                assert!(cfg.terra.is_none());
                assert!(cfg.package.is_none());
                assert_eq!(cfg.node.as_deref(), Some("ConveyorDriver"));
                assert_eq!(
                    cfg.params.get("baudrate").and_then(|v| v.as_integer()),
                    Some(57600)
                );
            }
            other => panic!("expected Config, got {:?}", other),
        }
    }

    /// Mix simple string/bool drivers with table config drivers.
    #[test]
    fn parse_driver_mixed_simple_and_table() {
        let toml_str = r#"
[package]
name = "test"
version = "0.1.0"

[drivers]
camera = "opencv"
gps = true

[drivers.arm]
terra = "dynamixel"
port = "/dev/ttyUSB0"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(manifest.drivers.len(), 3);
        assert!(matches!(manifest.drivers["camera"], DriverValue::Backend(ref s) if s == "opencv"));
        assert!(matches!(
            manifest.drivers["gps"],
            DriverValue::Enabled(true)
        ));
        assert!(matches!(manifest.drivers["arm"], DriverValue::Config(_)));
    }

    /// Round-trip: DriverTableConfig survives serialize → deserialize.
    #[test]
    fn driver_table_config_round_trip() {
        let toml_str = r#"
[package]
name = "test"
version = "0.1.0"

[drivers.lidar]
terra = "rplidar"
port = "/dev/ttyUSB1"
scan_mode = "sensitivity"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        let serialized = toml::to_string_pretty(&manifest).unwrap();
        let reloaded: HorusManifest = toml::from_str(&serialized).unwrap();
        match &reloaded.drivers["lidar"] {
            DriverValue::Config(cfg) => {
                assert_eq!(cfg.terra.as_deref(), Some("rplidar"));
                assert_eq!(
                    cfg.params.get("port").and_then(|v| v.as_str()),
                    Some("/dev/ttyUSB1")
                );
                assert_eq!(
                    cfg.params.get("scan_mode").and_then(|v| v.as_str()),
                    Some("sensitivity")
                );
            }
            other => panic!("expected Config after round-trip, got {:?}", other),
        }
    }

    // ── Additional robustness edge cases ─────────────────────────────

    /// Name with allowed special chars: @ and / (scoped names).
    #[test]
    fn battle_validate_accepts_scoped_name() {
        let toml_str = r#"
[package]
name = "@softmata/my-robot"
version = "0.1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert!(manifest.validate().is_ok());
    }

    /// Name at maximum length (64 chars).
    #[test]
    fn battle_validate_name_at_max_length() {
        // 64 chars of lowercase
        let name = "a".repeat(64);
        let toml_str = format!(
            r#"
[package]
name = "{}"
version = "0.1.0"
"#,
            name
        );
        let manifest: HorusManifest = toml::from_str(&toml_str).unwrap();
        assert!(manifest.validate().is_ok());
    }

    /// Name exceeding maximum length (65 chars).
    #[test]
    fn battle_validate_name_exceeds_max_length() {
        let name = "a".repeat(65);
        let toml_str = format!(
            r#"
[package]
name = "{}"
version = "0.1.0"
"#,
            name
        );
        let manifest: HorusManifest = toml::from_str(&toml_str).unwrap();
        assert!(manifest.validate().is_err());
    }

    /// Git dep with version + features together.
    #[test]
    fn battle_git_dep_with_version_and_features() {
        let toml_str = r#"
[package]
name = "git-feat"
version = "0.1.0"

[dependencies]
complex-git = { git = "https://github.com/org/repo", tag = "v2.0", version = "2.0", features = ["async", "tls"] }
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        match &manifest.dependencies["complex-git"] {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.git.as_deref(), Some("https://github.com/org/repo"));
                assert_eq!(d.tag.as_deref(), Some("v2.0"));
                assert_eq!(d.version.as_deref(), Some("2.0"));
                assert_eq!(d.features, vec!["async", "tls"]);
                assert_eq!(d.effective_source(), DepSource::Git);
            }
            _ => panic!("expected Detailed"),
        }
    }

    /// Verify find_and_load_from finds horus.toml in current dir (not just parent).
    #[test]
    fn battle_find_and_load_from_current_directory() {
        let dir = tempfile::tempdir().unwrap();
        let toml_content = r#"
[package]
name = "found-here"
version = "0.1.0"
"#;
        fs::write(dir.path().join(HORUS_TOML), toml_content).unwrap();

        let (manifest, found_dir) =
            HorusManifest::find_and_load_from(dir.path().to_path_buf()).unwrap();
        assert_eq!(manifest.package.name, "found-here");
        assert_eq!(found_dir, dir.path());
    }

    /// All reserved names are rejected.
    #[test]
    fn battle_validate_all_reserved_names() {
        let reserved = [
            "horus", "core", "std", "lib", "test", "main", "admin", "api", "root", "system",
            "internal", "config", "setup", "install",
        ];
        for name in &reserved {
            let toml_str = format!(
                r#"
[package]
name = "{}"
version = "0.1.0"
"#,
                name
            );
            let manifest: HorusManifest = toml::from_str(&toml_str).unwrap();
            assert!(
                manifest.validate().is_err(),
                "Reserved name '{}' should be rejected",
                name
            );
        }
    }

    /// Verify that multiple validation errors are aggregated.
    #[test]
    fn battle_validate_multiple_errors_aggregated() {
        // Name too short AND has uppercase → both errors should be reported
        let toml_str = r#"
[package]
name = "X"
version = "not-semver"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        let err = manifest.validate().unwrap_err();
        let msg = format!("{}", err);
        // Should contain at least 2 error items (name length, name chars, bad version)
        assert!(
            msg.contains("2.") || msg.contains("3."),
            "Should have multiple numbered errors: {}",
            msg
        );
    }

    // ── Workspace TOML parsing ───────────────────────────────────────

    /// 1. Parse a virtual workspace (only [workspace], no [package]).
    #[test]
    fn workspace_parse_virtual_workspace() {
        let toml_str = r#"
[workspace]
members = ["crates/*"]
exclude = ["crates/experimental"]
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert!(manifest.is_workspace());
        assert!(manifest.is_virtual_workspace());
        let ws = manifest.workspace.as_ref().unwrap();
        assert_eq!(ws.members, vec!["crates/*"]);
        assert_eq!(ws.exclude, vec!["crates/experimental"]);
        // Virtual workspace has no package name (defaults to empty)
        assert!(manifest.package.name.is_empty());
    }

    /// 2. Parse a root package with workspace (both [package] and [workspace]).
    #[test]
    fn workspace_parse_root_package_with_workspace() {
        let toml_str = r#"
[package]
name = "my-robot"
version = "0.1.0"

[workspace]
members = ["crates/*"]
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert!(manifest.is_workspace());
        assert!(!manifest.is_virtual_workspace());
        assert_eq!(manifest.package.name, "my-robot");
        assert_eq!(manifest.package.version, "0.1.0");
        let ws = manifest.workspace.as_ref().unwrap();
        assert_eq!(ws.members, vec!["crates/*"]);
    }

    /// 3. Parse [workspace.dependencies] section with Simple and Detailed deps.
    #[test]
    fn workspace_parse_workspace_dependencies() {
        let toml_str = r#"
[workspace]
members = ["crates/*"]

[workspace.dependencies]
serde = { version = "1.0", source = "crates.io", features = ["derive"] }
tokio = "1.35"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert!(manifest.is_workspace());
        let ws = manifest.workspace.as_ref().unwrap();
        assert_eq!(ws.dependencies.len(), 2);

        // serde is Detailed with features
        match &ws.dependencies["serde"] {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.version.as_deref(), Some("1.0"));
                assert_eq!(d.source, Some(DepSource::CratesIo));
                assert_eq!(d.features, vec!["derive"]);
            }
            _ => panic!("expected Detailed for serde"),
        }

        // tokio is Simple
        match &ws.dependencies["tokio"] {
            DependencyValue::Simple(v) => assert_eq!(v, "1.35"),
            _ => panic!("expected Simple for tokio"),
        }
    }

    /// 4. Parse a member manifest with workspace = true dep.
    #[test]
    fn workspace_parse_member_with_workspace_dep() {
        let toml_str = r#"
[package]
name = "my-messages"
version = "0.1.0"
type = "lib"

[dependencies]
serde = { workspace = true }
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(manifest.package.name, "my-messages");
        assert_eq!(manifest.package.target_type, TargetType::Lib);

        match &manifest.dependencies["serde"] {
            DependencyValue::Detailed(d) => {
                assert!(d.workspace, "serde should have workspace = true");
                assert!(
                    d.version.is_none(),
                    "workspace dep should have no local version"
                );
            }
            _ => panic!("expected Detailed for serde"),
        }
    }

    /// 5. All TargetType values parse correctly; default is Bin.
    #[test]
    fn workspace_parse_target_type_variants() {
        // type = "lib"
        let toml_str = r#"
[package]
name = "my-lib"
version = "0.1.0"
type = "lib"
"#;
        let m: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(m.package.target_type, TargetType::Lib);

        // type = "bin"
        let toml_str = r#"
[package]
name = "my-bin"
version = "0.1.0"
type = "bin"
"#;
        let m: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(m.package.target_type, TargetType::Bin);

        // type = "both"
        let toml_str = r#"
[package]
name = "my-both"
version = "0.1.0"
type = "both"
"#;
        let m: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(m.package.target_type, TargetType::Both);

        // No type → Bin (default)
        let toml_str = r#"
[package]
name = "my-default"
version = "0.1.0"
"#;
        let m: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(m.package.target_type, TargetType::Bin);
    }

    /// 6. No [workspace] section means single-package project.
    #[test]
    fn workspace_absent_means_single_package() {
        let toml_str = r#"
[package]
name = "solo-bot"
version = "0.1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert!(!manifest.is_workspace());
        assert!(!manifest.is_virtual_workspace());
        assert!(manifest.workspace.is_none());
    }

    // ── Backward compatibility ───────────────────────────────────────

    /// 7. A full existing-style manifest with all sections parses; workspace is None.
    #[test]
    fn workspace_backward_compat_existing_manifest() {
        let toml_str = r#"
enable = ["cuda"]

[package]
name = "compat-bot"
version = "1.0.0"
description = "Full backward compat test"
authors = ["Alice <alice@example.com>"]
license = "MIT"
edition = "1"
package-type = "node"
categories = ["robotics"]

[dependencies]
serde = { version = "1.0", source = "crates.io", features = ["derive"] }
horus_library = "0.2.0"

[dev-dependencies]
criterion = { version = "0.5", source = "crates.io" }

[drivers]
camera = "opencv"
lidar = true

[scripts]
sim = "horus sim start"
deploy = "horus deploy"

[ignore]
files = ["*.bak"]
directories = ["tmp/"]
packages = ["debugpy"]

[hooks]
pre_run = ["fmt"]
pre_build = ["lint"]
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert!(
            manifest.workspace.is_none(),
            "workspace should be None for existing-style manifest"
        );
        assert_eq!(manifest.package.name, "compat-bot");
        assert_eq!(manifest.dependencies.len(), 2);
        assert_eq!(manifest.dev_dependencies.len(), 1);
        assert_eq!(manifest.drivers.len(), 2);
        assert_eq!(manifest.scripts.len(), 2);
        assert_eq!(manifest.ignore.files, vec!["*.bak"]);
        assert_eq!(manifest.ignore.directories, vec!["tmp/"]);
        assert_eq!(manifest.ignore.packages, vec!["debugpy"]);
        assert_eq!(manifest.hooks.pre_run, vec!["fmt"]);
        assert_eq!(manifest.hooks.pre_build, vec!["lint"]);
        assert_eq!(manifest.enable, vec!["cuda"]);
    }

    /// 8. Parse → serialize → re-parse roundtrip: all fields match, no [workspace] appears.
    #[test]
    fn workspace_backward_compat_roundtrip() {
        let toml_str = r#"
[package]
name = "roundtrip-compat"
version = "2.0.0"
description = "Roundtrip test"

[dependencies]
serde = { version = "1.0", source = "crates.io" }

[dev-dependencies]
criterion = { version = "0.5", source = "crates.io" }

[drivers]
camera = "opencv"

[scripts]
build = "cargo build"

[ignore]
files = ["*.tmp"]
"#;
        let original: HorusManifest = toml::from_str(toml_str).unwrap();
        let serialized = toml::to_string_pretty(&original).unwrap();

        // [workspace] must NOT appear in serialized output
        assert!(
            !serialized.contains("[workspace]"),
            "Serialized output should not contain [workspace]: {}",
            serialized
        );

        let reparsed: HorusManifest = toml::from_str(&serialized).unwrap();
        assert_eq!(reparsed.package.name, original.package.name);
        assert_eq!(reparsed.package.version, original.package.version);
        assert_eq!(reparsed.package.description, original.package.description);
        assert_eq!(reparsed.dependencies.len(), original.dependencies.len());
        assert_eq!(
            reparsed.dev_dependencies.len(),
            original.dev_dependencies.len()
        );
        assert_eq!(reparsed.drivers.len(), original.drivers.len());
        assert_eq!(reparsed.scripts.len(), original.scripts.len());
        assert_eq!(reparsed.ignore.files, original.ignore.files);
        assert!(reparsed.workspace.is_none());
    }

    // ── Workspace validation ─────────────────────────────────────────

    /// 9. Virtual workspace with members validates OK.
    #[test]
    fn workspace_validate_virtual_ok() {
        let toml_str = r#"
[workspace]
members = ["crates/*"]
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        let warnings = manifest.validate().unwrap();
        assert!(warnings.is_empty());
    }

    /// 10. Virtual workspace with empty members fails validation.
    #[test]
    fn workspace_validate_virtual_no_members_fails() {
        let toml_str = r#"
[workspace]
members = []
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        let err = manifest.validate().unwrap_err();
        let msg = format!("{}", err);
        assert!(
            msg.contains("no members"),
            "Error should mention no members: {}",
            msg
        );
    }

    /// 11. Empty manifest (no [package], no [workspace]) fails validation.
    #[test]
    fn workspace_validate_no_package_no_workspace_fails() {
        // A completely empty TOML still creates a default HorusManifest
        // with empty package name and no workspace.
        let toml_str = "";
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert!(!manifest.is_workspace());
        assert!(manifest.package.name.is_empty());
        let err = manifest.validate().unwrap_err();
        let msg = format!("{}", err);
        assert!(
            msg.contains("Missing") || msg.contains("package"),
            "Error should mention missing package: {}",
            msg
        );
    }

    // ── resolve_workspace_members ────────────────────────────────────

    /// 12. Resolve members from glob pattern, skipping dirs without horus.toml.
    #[test]
    fn workspace_resolve_members_glob() {
        let dir = tempfile::tempdir().unwrap();
        let root = dir.path();

        // Root workspace manifest
        let ws_toml = r#"
[workspace]
members = ["crates/*"]
"#;
        fs::write(root.join(HORUS_TOML), ws_toml).unwrap();

        // Member: messages
        let messages_dir = root.join("crates").join("messages");
        fs::create_dir_all(&messages_dir).unwrap();
        fs::write(
            messages_dir.join(HORUS_TOML),
            r#"
[package]
name = "messages"
version = "0.1.0"
"#,
        )
        .unwrap();

        // Member: driver
        let driver_dir = root.join("crates").join("driver");
        fs::create_dir_all(&driver_dir).unwrap();
        fs::write(
            driver_dir.join(HORUS_TOML),
            r#"
[package]
name = "driver"
version = "0.1.0"
"#,
        )
        .unwrap();

        // Dir with no horus.toml — should be skipped
        let notoml_dir = root.join("crates").join("notoml");
        fs::create_dir_all(&notoml_dir).unwrap();

        let ws_manifest: HorusManifest = toml::from_str(ws_toml).unwrap();
        let ws = ws_manifest.workspace.as_ref().unwrap();
        let members = resolve_workspace_members(ws, root).unwrap();

        assert_eq!(members.len(), 2, "Should find 2 members, got {:?}", members);
        let names: Vec<&str> = members
            .iter()
            .map(|(_, m)| m.package.name.as_str())
            .collect();
        assert!(
            names.contains(&"messages"),
            "Missing 'messages' in {:?}",
            names
        );
        assert!(names.contains(&"driver"), "Missing 'driver' in {:?}", names);
    }

    /// 13. Exclude pattern filters out matching members.
    #[test]
    fn workspace_resolve_members_exclude() {
        let dir = tempfile::tempdir().unwrap();
        let root = dir.path();

        // Member: messages
        let messages_dir = root.join("crates").join("messages");
        fs::create_dir_all(&messages_dir).unwrap();
        fs::write(
            messages_dir.join(HORUS_TOML),
            r#"
[package]
name = "messages"
version = "0.1.0"
"#,
        )
        .unwrap();

        // Member: driver (will be excluded)
        let driver_dir = root.join("crates").join("driver");
        fs::create_dir_all(&driver_dir).unwrap();
        fs::write(
            driver_dir.join(HORUS_TOML),
            r#"
[package]
name = "driver"
version = "0.1.0"
"#,
        )
        .unwrap();

        let ws = WorkspaceConfig {
            members: vec!["crates/*".to_string()],
            exclude: vec!["crates/driver".to_string()],
            dependencies: BTreeMap::new(),
        };

        let members = resolve_workspace_members(&ws, root).unwrap();
        assert_eq!(members.len(), 1);
        assert_eq!(members[0].1.package.name, "messages");
    }

    /// 14. No matching directories returns empty vec.
    #[test]
    fn workspace_resolve_members_empty_dir() {
        let dir = tempfile::tempdir().unwrap();
        let root = dir.path();

        let ws = WorkspaceConfig {
            members: vec!["crates/*".to_string()],
            exclude: vec![],
            dependencies: BTreeMap::new(),
        };

        let members = resolve_workspace_members(&ws, root).unwrap();
        assert!(members.is_empty());
    }

    // ── resolve_workspace_dep ────────────────────────────────────────

    /// 15. Simple workspace dep resolves to version string.
    #[test]
    fn workspace_resolve_dep_simple() {
        let mut ws_deps = BTreeMap::new();
        ws_deps.insert(
            "tokio".to_string(),
            DependencyValue::Simple("1.35".to_string()),
        );

        let member_dep = DetailedDependency {
            workspace: true,
            ..DetailedDependency::default()
        };

        let resolved = resolve_workspace_dep("tokio", &member_dep, &ws_deps).unwrap();
        assert_eq!(resolved.version(), Some("1.35"));
    }

    /// 16. Workspace dep with features merges member-specific features on top.
    #[test]
    fn workspace_resolve_dep_with_features() {
        let mut ws_deps = BTreeMap::new();
        ws_deps.insert(
            "serde".to_string(),
            DependencyValue::Detailed(DetailedDependency {
                version: Some("1.0".to_string()),
                source: Some(DepSource::CratesIo),
                features: vec!["derive".to_string()],
                ..DetailedDependency::default()
            }),
        );

        let member_dep = DetailedDependency {
            workspace: true,
            features: vec!["alloc".to_string()],
            ..DetailedDependency::default()
        };

        let resolved = resolve_workspace_dep("serde", &member_dep, &ws_deps).unwrap();
        match resolved {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.version.as_deref(), Some("1.0"));
                assert_eq!(d.source, Some(DepSource::CratesIo));
                assert!(
                    d.features.contains(&"derive".to_string()),
                    "Should contain workspace feature 'derive': {:?}",
                    d.features
                );
                assert!(
                    d.features.contains(&"alloc".to_string()),
                    "Should contain member feature 'alloc': {:?}",
                    d.features
                );
            }
            _ => panic!("expected Detailed resolved dep"),
        }
    }

    /// 17. Requesting a nonexistent workspace dep returns Err.
    #[test]
    fn workspace_resolve_dep_missing_fails() {
        let ws_deps = BTreeMap::new();
        let member_dep = DetailedDependency {
            workspace: true,
            ..DetailedDependency::default()
        };

        let result = resolve_workspace_dep("nonexistent", &member_dep, &ws_deps);
        assert!(result.is_err());
        let msg = format!("{}", result.unwrap_err());
        assert!(
            msg.contains("nonexistent") && msg.contains("workspace"),
            "Error should mention dep name and workspace: {}",
            msg
        );
    }

    // ── Workspace battle tests ───────────────────────────────────────

    /// 18. Full workspace manifest roundtrip: create, serialize, re-parse, verify.
    #[test]
    fn battle_workspace_toml_full_roundtrip() {
        let manifest = HorusManifest {
            package: PackageInfo {
                name: "ws-root".to_string(),
                version: "1.0.0".to_string(),
                description: Some("Workspace root".to_string()),
                authors: vec!["Dev <dev@example.com>".to_string()],
                license: Some("MIT".to_string()),
                edition: "1".to_string(),
                repository: Some("https://github.com/org/ws".to_string()),
                package_type: Some(PackageType::App),
                categories: vec!["robotics".to_string()],
                standard: None,
                rust_edition: None,
                target_type: TargetType::Lib,
            },
            workspace: Some(WorkspaceConfig {
                members: vec!["crates/*".to_string(), "tools/*".to_string()],
                exclude: vec!["crates/experimental".to_string()],
                dependencies: {
                    let mut deps = BTreeMap::new();
                    deps.insert(
                        "serde".to_string(),
                        DependencyValue::Detailed(DetailedDependency {
                            version: Some("1.0".to_string()),
                            source: Some(DepSource::CratesIo),
                            features: vec!["derive".to_string()],
                            ..DetailedDependency::default()
                        }),
                    );
                    deps.insert(
                        "tokio".to_string(),
                        DependencyValue::Simple("1.35".to_string()),
                    );
                    deps
                },
            }),
            robot: None,
            dependencies: {
                let mut deps = BTreeMap::new();
                deps.insert(
                    "horus_library".to_string(),
                    DependencyValue::Simple("0.2.0".to_string()),
                );
                deps
            },
            dev_dependencies: BTreeMap::new(),
            sim_dependencies: BTreeMap::new(),
            hardware: BTreeMap::new(),
            drivers: BTreeMap::new(),
            sim_drivers: BTreeMap::new(),
            scripts: {
                let mut s = BTreeMap::new();
                s.insert(
                    "build-all".to_string(),
                    "cargo build --workspace".to_string(),
                );
                s
            },
            ignore: IgnoreConfig::default(),
            enable: vec!["cuda".to_string()],
            cpp: None,
            hooks: Default::default(),
            network: None,
        };

        let serialized = toml::to_string_pretty(&manifest).unwrap();
        let reparsed: HorusManifest = toml::from_str(&serialized).unwrap();

        // Package fields
        assert_eq!(reparsed.package.name, "ws-root");
        assert_eq!(reparsed.package.version, "1.0.0");
        assert_eq!(
            reparsed.package.description.as_deref(),
            Some("Workspace root")
        );
        assert_eq!(reparsed.package.target_type, TargetType::Lib);

        // Workspace config
        assert!(reparsed.is_workspace());
        assert!(!reparsed.is_virtual_workspace());
        let ws = reparsed.workspace.as_ref().unwrap();
        assert_eq!(ws.members, vec!["crates/*", "tools/*"]);
        assert_eq!(ws.exclude, vec!["crates/experimental"]);
        assert_eq!(ws.dependencies.len(), 2);

        // Workspace deps
        match &ws.dependencies["serde"] {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.version.as_deref(), Some("1.0"));
                assert_eq!(d.features, vec!["derive"]);
            }
            _ => panic!("expected Detailed serde"),
        }
        match &ws.dependencies["tokio"] {
            DependencyValue::Simple(v) => assert_eq!(v, "1.35"),
            _ => panic!("expected Simple tokio"),
        }

        // Root deps
        assert_eq!(reparsed.dependencies.len(), 1);
        assert!(reparsed.dependencies["horus_library"].is_registry());

        // Scripts
        assert_eq!(reparsed.scripts.len(), 1);
        assert_eq!(reparsed.scripts["build-all"], "cargo build --workspace");

        // Enable
        assert_eq!(reparsed.enable, vec!["cuda"]);
    }

    /// 19. TargetType::Bin (default) is omitted in serialization; Lib and Both appear.
    #[test]
    fn battle_target_type_default_omitted_in_serialization() {
        // Bin (default) should be omitted
        let manifest_bin = HorusManifest {
            package: PackageInfo {
                name: "tt-bin".to_string(),
                version: "0.1.0".to_string(),
                target_type: TargetType::Bin,
                ..PackageInfo::default()
            },
            workspace: None,
            robot: None,
            dependencies: BTreeMap::new(),
            dev_dependencies: BTreeMap::new(),
            sim_dependencies: BTreeMap::new(),
            hardware: BTreeMap::new(),
            drivers: BTreeMap::new(),
            sim_drivers: BTreeMap::new(),
            scripts: BTreeMap::new(),
            ignore: IgnoreConfig::default(),
            enable: vec![],
            cpp: None,
            hooks: Default::default(),
            network: None,
        };
        let ser_bin = toml::to_string_pretty(&manifest_bin).unwrap();
        assert!(
            !ser_bin.contains("type"),
            "Default Bin target_type should be omitted: {}",
            ser_bin
        );

        // Lib should appear
        let manifest_lib = HorusManifest {
            package: PackageInfo {
                name: "tt-lib".to_string(),
                version: "0.1.0".to_string(),
                target_type: TargetType::Lib,
                ..PackageInfo::default()
            },
            workspace: None,
            robot: None,
            dependencies: BTreeMap::new(),
            dev_dependencies: BTreeMap::new(),
            sim_dependencies: BTreeMap::new(),
            hardware: BTreeMap::new(),
            drivers: BTreeMap::new(),
            sim_drivers: BTreeMap::new(),
            scripts: BTreeMap::new(),
            ignore: IgnoreConfig::default(),
            enable: vec![],
            cpp: None,
            hooks: Default::default(),
            network: None,
        };
        let ser_lib = toml::to_string_pretty(&manifest_lib).unwrap();
        assert!(
            ser_lib.contains("type = \"lib\""),
            "Lib target_type should appear in serialized output: {}",
            ser_lib
        );

        // Both should appear
        let manifest_both = HorusManifest {
            package: PackageInfo {
                name: "tt-both".to_string(),
                version: "0.1.0".to_string(),
                target_type: TargetType::Both,
                ..PackageInfo::default()
            },
            workspace: None,
            robot: None,
            dependencies: BTreeMap::new(),
            dev_dependencies: BTreeMap::new(),
            sim_dependencies: BTreeMap::new(),
            hardware: BTreeMap::new(),
            drivers: BTreeMap::new(),
            sim_drivers: BTreeMap::new(),
            scripts: BTreeMap::new(),
            ignore: IgnoreConfig::default(),
            enable: vec![],
            cpp: None,
            hooks: Default::default(),
            network: None,
        };
        let ser_both = toml::to_string_pretty(&manifest_both).unwrap();
        assert!(
            ser_both.contains("type = \"both\""),
            "Both target_type should appear in serialized output: {}",
            ser_both
        );
    }

    /// 20. DetailedDependency with workspace: false should omit "workspace" in serialized output.
    #[test]
    fn battle_workspace_dep_bool_false_omitted() {
        let dep = DetailedDependency {
            version: Some("1.0".to_string()),
            source: Some(DepSource::CratesIo),
            workspace: false,
            ..DetailedDependency::default()
        };
        let serialized = toml::to_string_pretty(&dep).unwrap();
        assert!(
            !serialized.contains("workspace"),
            "workspace = false should be omitted (skip_serializing_if = is_false): {}",
            serialized
        );

        // workspace = true should appear
        let dep_ws = DetailedDependency {
            workspace: true,
            ..DetailedDependency::default()
        };
        let serialized_ws = toml::to_string_pretty(&dep_ws).unwrap();
        assert!(
            serialized_ws.contains("workspace = true"),
            "workspace = true should appear in serialized output: {}",
            serialized_ws
        );
    }

    // ── Additional manifest parsing tests ────────────────────────────

    #[test]
    fn test_manifest_parse_minimal_toml() {
        let toml_str = "[package]\nname = \"test\"\nversion = \"0.1.0\"";
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(manifest.package.name, "test");
        assert_eq!(manifest.package.version, "0.1.0");
        // Defaults are applied for optional fields
        assert!(manifest.package.description.is_none());
        assert!(manifest.package.authors.is_empty());
        assert!(manifest.dependencies.is_empty());
        assert!(manifest.scripts.is_empty());
    }

    #[test]
    fn test_manifest_parse_with_dependencies() {
        let toml_str = r#"
[package]
name = "test-deps"
version = "0.1.0"

[dependencies]
serde = "1.0"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(manifest.dependencies.len(), 1);
        match &manifest.dependencies["serde"] {
            DependencyValue::Simple(v) => assert_eq!(v, "1.0"),
            other => panic!("expected Simple(\"1.0\"), got {:?}", other),
        }
        // Simple deps default to Registry source
        assert_eq!(
            manifest.dependencies["serde"].effective_source(),
            DepSource::Registry
        );
    }

    #[test]
    fn test_manifest_parse_with_detailed_dep() {
        let toml_str = r#"
[package]
name = "test-detailed"
version = "0.1.0"

[dependencies.tokio]
version = "1.0"
features = ["full"]
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(manifest.dependencies.len(), 1);
        match &manifest.dependencies["tokio"] {
            DependencyValue::Detailed(d) => {
                assert_eq!(d.version.as_deref(), Some("1.0"));
                assert_eq!(d.features, vec!["full"]);
            }
            other => panic!(
                "expected Detailed with version and features, got {:?}",
                other
            ),
        }
    }

    #[test]
    fn test_manifest_parse_with_scripts() {
        let toml_str = r#"
[package]
name = "test-scripts"
version = "0.1.0"

[scripts]
test = "cargo test"
"#;
        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert_eq!(manifest.scripts.len(), 1);
        assert_eq!(manifest.scripts["test"], "cargo test");
    }
}
