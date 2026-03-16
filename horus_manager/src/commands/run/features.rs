use crate::manifest::{DriverValue, HorusManifest, IgnoreConfig, HORUS_TOML};
use std::path::Path;

impl From<IgnoreConfig> for IgnorePatterns {
    fn from(cfg: IgnoreConfig) -> Self {
        Self {
            files: cfg.files,
            directories: cfg.directories,
            packages: cfg.packages,
        }
    }
}

/// Resolve driver alias names to their constituent driver IDs.
/// Used when parsing `[drivers]` section in horus.toml.
pub(crate) fn resolve_driver_alias(alias: &str) -> Option<Vec<&'static str>> {
    match alias {
        "vision" => Some(vec!["camera", "depth-camera"]),
        "navigation" => Some(vec!["lidar", "gps", "imu"]),
        "manipulation" => Some(vec!["servo", "motor", "force-torque"]),
        "locomotion" => Some(vec!["motor", "encoder", "imu"]),
        "sensing" => Some(vec!["camera", "lidar", "ultrasonic", "imu"]),
        _ => None,
    }
}

/// Ignore patterns from horus.toml
#[derive(Debug, Clone, Default)]
pub struct IgnorePatterns {
    pub files: Vec<String>,
    pub directories: Vec<String>,
    pub packages: Vec<String>,
}

impl IgnorePatterns {
    /// Check if a file path should be ignored
    pub fn should_ignore_file(&self, path: &Path) -> bool {
        let path_str = path.to_string_lossy();

        // Check directory patterns first
        for dir_pattern in &self.directories {
            let pattern = dir_pattern.trim_end_matches('/');
            if path_str.contains(pattern) {
                return true;
            }
        }

        // Check file patterns with glob matching
        for file_pattern in &self.files {
            if glob_match(file_pattern, &path_str) {
                return true;
            }
        }

        false
    }

    /// Check if a package should be ignored
    pub fn should_ignore_package(&self, package: &str) -> bool {
        self.packages.iter().any(|p| p == package)
    }
}

/// Simple glob matching for ignore patterns
pub(crate) fn glob_match(pattern: &str, text: &str) -> bool {
    // Handle ** for directory recursion
    if pattern.contains("**/") {
        let parts: Vec<&str> = pattern.split("**/").collect();
        if parts.len() == 2 {
            let suffix = parts[1];
            return text.contains(suffix) || text.ends_with(suffix);
        }
    }

    // Handle * wildcard
    if pattern.contains('*') {
        let parts: Vec<&str> = pattern.split('*').collect();
        if parts.is_empty() {
            return true;
        }

        let mut pos = 0;
        for (i, part) in parts.iter().enumerate() {
            if part.is_empty() {
                continue;
            }

            if i == 0 && !text.starts_with(part) {
                return false;
            }

            if let Some(found_pos) = text[pos..].find(part) {
                pos += found_pos + part.len();
            } else {
                return false;
            }
        }

        // If pattern ends with *, we're good
        // Otherwise, make sure we matched to the end
        if !pattern.ends_with('*') && pos != text.len() {
            return false;
        }

        true
    } else {
        // Exact match or ends_with for simple patterns
        text == pattern || text.ends_with(pattern)
    }
}

/// Driver configuration from horus.toml `[drivers]` section
#[derive(Debug, Clone, Default)]
pub struct DriverConfig {
    /// List of drivers to enable (e.g., ["camera", "lidar", "imu"])
    pub drivers: Vec<String>,
    /// Backend overrides (e.g., {"lidar": "rplidar-a2", "imu": "mpu6050"})
    pub backends: std::collections::HashMap<String, String>,
}

impl DriverConfig {
    /// Build from manifest's `[drivers]` section.
    pub fn from_manifest(manifest: &HorusManifest) -> Self {
        let mut config = Self::default();
        for (name, value) in &manifest.drivers {
            match value {
                DriverValue::Config(cfg) => {
                    // Config table — extract driver name from terra/package/node key
                    config.drivers.push(name.clone());
                    if let Some(terra) = &cfg.terra {
                        config.backends.insert(name.clone(), terra.clone());
                    } else if let Some(package) = &cfg.package {
                        config.backends.insert(name.clone(), package.clone());
                    } else if let Some(node) = &cfg.node {
                        config.backends.insert(name.clone(), node.clone());
                    }
                }
                DriverValue::Backend(backend) => {
                    // Simple string — resolve aliases, store backend
                    if let Some(expanded) = resolve_driver_alias(name) {
                        for d in expanded {
                            config.drivers.push(d.to_string());
                        }
                    } else {
                        config.drivers.push(name.clone());
                    }
                    config.backends.insert(name.clone(), backend.clone());
                }
                DriverValue::Enabled(true) => {
                    // Bool enable — resolve aliases
                    if let Some(expanded) = resolve_driver_alias(name) {
                        for d in expanded {
                            config.drivers.push(d.to_string());
                        }
                    } else {
                        config.drivers.push(name.clone());
                    }
                }
                DriverValue::Enabled(false) => {
                    // Explicitly disabled — skip
                }
            }
        }
        config
    }
}

/// Get active drivers - combines HORUS_DRIVERS env var and horus.toml
pub fn get_active_drivers() -> DriverConfig {
    // Priority: HORUS_DRIVERS env var > horus.toml
    if let Ok(env_drivers) = std::env::var("HORUS_DRIVERS") {
        let drivers: Vec<String> = env_drivers
            .split(',')
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect();

        if !drivers.is_empty() {
            return DriverConfig {
                drivers,
                backends: std::collections::HashMap::new(),
            };
        }
    }

    // Fall back to horus.toml
    load_manifest()
        .map(|m| DriverConfig::from_manifest(&m))
        .unwrap_or_default()
}

/// Get Cargo features to enable based on driver configuration
///
/// Queries the HORUS registry for each driver's required features.
/// Driver metadata (required_features, cargo_dependencies, etc.) lives in the
/// registry — not hardcoded in the CLI.
pub fn get_cargo_features_from_drivers(config: &DriverConfig) -> Vec<String> {
    use crate::registry::RegistryClient;

    let mut features = Vec::new();
    let client = RegistryClient::new();

    for driver in &config.drivers {
        let backend = config.backends.get(driver).map(|s| s.as_str());

        // Construct driver name for registry lookup (e.g., "lidar" + "rplidar" -> "lidar-rplidar")
        let driver_name = if let Some(b) = backend {
            format!("{}-{}", driver, b)
        } else {
            driver.to_string()
        };

        // Query registry for required features (silent failure if registry unreachable)
        if let Some(registry_features) = client.query_driver_features(&driver_name) {
            for f in registry_features {
                if !features.contains(&f) {
                    features.push(f);
                }
            }
        }
    }

    features
}

// ============================================================================
// Enable Capabilities Configuration
// ============================================================================

/// Enable configuration from horus.toml or CLI --enable flag
#[derive(Debug, Clone, Default)]
pub struct EnableConfig {
    /// List of capabilities to enable (e.g., ["cuda", "editor", "python"])
    pub capabilities: Vec<String>,
}

/// Map a capability name to Cargo feature(s)
///
/// This maps user-friendly capability names to Cargo features.
/// Users specify capabilities like "cuda" or "editor" without knowing the underlying features.
///
/// # Example mappings:
/// - `cuda` → `["cuda"]`
/// - `editor` → `["editor"]`
/// - `python` → `["python"]`
/// - `gpu` → `["cuda"]` (alias)
/// - `headless` → `["headless"]`
pub fn enable_to_features(capability: &str) -> Vec<String> {
    match capability.to_lowercase().as_str() {
        // GPU/CUDA capabilities
        "cuda" | "gpu" => vec!["cuda".to_string()],

        // Rendering/UI capabilities
        "editor" => vec!["editor".to_string()],
        "headless" => vec!["headless".to_string()],
        "visual" => vec!["visual".to_string()],

        // Language bindings
        "python" | "py" => vec!["python".to_string()],

        // Backend features
        "opencv" | "opencv-backend" => vec!["opencv-backend".to_string()],

        // Performance features
        "io-uring" | "io-uring-net" => vec!["io-uring-net".to_string()],
        "ultra-low-latency" => vec!["ultra-low-latency".to_string()],

        // Bundle aliases
        "full" => vec!["full".to_string()],
        "sim" | "simulation" => vec![], // Simulation mode - no hardware features

        // Pass through unknown capabilities as-is (for advanced users)
        other => vec![other.to_string()],
    }
}

/// Get Cargo features to enable based on enable configuration
pub fn get_cargo_features_from_enable(config: &EnableConfig) -> Vec<String> {
    let mut features = Vec::new();

    for capability in &config.capabilities {
        let cap_features = enable_to_features(capability);
        for f in cap_features {
            if !features.contains(&f) {
                features.push(f);
            }
        }
    }

    features
}

/// Get active enable config - combines HORUS_ENABLE env var and horus.toml
pub fn get_active_enable() -> EnableConfig {
    // Priority: HORUS_ENABLE env var > horus.toml
    if let Ok(env_enable) = std::env::var("HORUS_ENABLE") {
        let capabilities: Vec<String> = env_enable
            .split(',')
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
            .collect();

        if !capabilities.is_empty() {
            return EnableConfig { capabilities };
        }
    }

    // Fall back to horus.toml
    load_manifest()
        .map(|m| EnableConfig {
            capabilities: m.enable,
        })
        .unwrap_or_default()
}

/// Get combined features string from both drivers and enable config
pub fn get_all_cargo_features() -> Option<String> {
    let driver_config = get_active_drivers();
    let enable_config = get_active_enable();

    let mut all_features = Vec::new();

    // Add driver features
    for f in get_cargo_features_from_drivers(&driver_config) {
        if !all_features.contains(&f) {
            all_features.push(f);
        }
    }

    // Add enable features
    for f in get_cargo_features_from_enable(&enable_config) {
        if !all_features.contains(&f) {
            all_features.push(f);
        }
    }

    if all_features.is_empty() {
        None
    } else {
        Some(all_features.join(","))
    }
}

/// Load manifest from horus.toml if it exists.
fn load_manifest() -> Option<HorusManifest> {
    let path = Path::new(HORUS_TOML);
    if path.exists() {
        HorusManifest::load_from(path).ok()
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::manifest::{IgnoreConfig, PackageInfo};
    use std::collections::BTreeMap;
    use std::path::Path;

    /// Helper: build a minimal HorusManifest for testing.
    fn minimal_manifest() -> HorusManifest {
        HorusManifest {
            package: PackageInfo {
                name: "test-pkg".to_string(),
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
            },
            dependencies: BTreeMap::new(),
            dev_dependencies: BTreeMap::new(),
            drivers: BTreeMap::new(),
            scripts: BTreeMap::new(),
            ignore: IgnoreConfig::default(),
            enable: vec![],
            cpp: None,
            hooks: Default::default(),
        }
    }

    // ── resolve_driver_alias ────────────────────────────────────────────────

    #[test]
    fn resolve_driver_alias_vision() {
        let drivers = resolve_driver_alias("vision").unwrap();
        assert_eq!(drivers, vec!["camera", "depth-camera"]);
    }

    #[test]
    fn resolve_driver_alias_navigation() {
        let drivers = resolve_driver_alias("navigation").unwrap();
        assert_eq!(drivers, vec!["lidar", "gps", "imu"]);
    }

    #[test]
    fn resolve_driver_alias_manipulation() {
        let drivers = resolve_driver_alias("manipulation").unwrap();
        assert_eq!(drivers, vec!["servo", "motor", "force-torque"]);
    }

    #[test]
    fn resolve_driver_alias_locomotion() {
        let drivers = resolve_driver_alias("locomotion").unwrap();
        assert_eq!(drivers, vec!["motor", "encoder", "imu"]);
    }

    #[test]
    fn resolve_driver_alias_sensing() {
        let drivers = resolve_driver_alias("sensing").unwrap();
        assert_eq!(drivers, vec!["camera", "lidar", "ultrasonic", "imu"]);
    }

    #[test]
    fn resolve_driver_alias_unknown_returns_none() {
        assert!(resolve_driver_alias("unknown").is_none());
        assert!(resolve_driver_alias("").is_none());
        assert!(resolve_driver_alias("camera").is_none());
    }

    // ── glob_match ──────────────────────────────────────────────────────────

    #[test]
    fn glob_match_double_star_directory_recursion() {
        assert!(glob_match("**/test.py", "src/foo/test.py"));
        assert!(glob_match("**/test.py", "test.py"));
    }

    #[test]
    fn glob_match_double_star_literal_suffix() {
        // When the suffix after **/ contains no wildcards, it matches via contains/ends_with.
        assert!(glob_match("**/Cargo.toml", "deep/nested/Cargo.toml"));
        assert!(glob_match("**/build/", "project/build/output"));
    }

    #[test]
    fn glob_match_double_star_wildcard_suffix_limitation() {
        // NOTE: The current glob_match implementation treats the suffix after **/
        // as a literal string, so **/*.log checks for literal "*.log" in the text.
        // This means **/ + wildcard suffix does NOT work as a recursive glob.
        assert!(!glob_match("**/*.log", "a/b/c/debug.log"));
    }

    #[test]
    fn glob_match_single_star_wildcard() {
        assert!(glob_match("*.py", "main.py"));
        assert!(glob_match("test_*", "test_hello"));
        assert!(glob_match("test_*.py", "test_module.py"));
    }

    #[test]
    fn glob_match_exact_match_no_wildcards() {
        assert!(glob_match("main.py", "main.py"));
        assert!(glob_match("Cargo.toml", "Cargo.toml"));
    }

    #[test]
    fn glob_match_ends_with_for_simple_patterns() {
        // Without wildcards, glob_match also matches via ends_with
        assert!(glob_match("main.py", "src/main.py"));
    }

    #[test]
    fn glob_match_no_match() {
        assert!(!glob_match("*.py", "main.rs"));
        assert!(!glob_match("test_*.py", "hello.py"));
    }

    #[test]
    fn glob_match_leading_star() {
        assert!(glob_match("*.rs", "lib.rs"));
        assert!(glob_match("*.rs", "deeply/nested/file.rs"));
    }

    #[test]
    fn glob_match_trailing_star() {
        assert!(glob_match("debug_*", "debug_log"));
        assert!(glob_match("debug_*", "debug_"));
    }

    #[test]
    fn glob_match_pattern_without_wildcards_no_match() {
        assert!(!glob_match("main.py", "other.py"));
        assert!(!glob_match("Cargo.toml", "pyproject.toml"));
    }

    #[test]
    fn glob_match_star_in_middle() {
        assert!(glob_match("test_*.py", "test_foo.py"));
        assert!(!glob_match("test_*.py", "test_foo.rs"));
    }

    // ── IgnorePatterns::should_ignore_file ──────────────────────────────────

    #[test]
    fn should_ignore_file_directory_match() {
        let patterns = IgnorePatterns {
            files: vec![],
            directories: vec!["target/".to_string(), "node_modules".to_string()],
            packages: vec![],
        };
        assert!(patterns.should_ignore_file(Path::new("project/target/debug/main")));
        assert!(patterns.should_ignore_file(Path::new("node_modules/foo/index.js")));
    }

    #[test]
    fn should_ignore_file_glob_match() {
        let patterns = IgnorePatterns {
            files: vec!["*.log".to_string(), "debug_*".to_string()],
            directories: vec![],
            packages: vec![],
        };
        assert!(patterns.should_ignore_file(Path::new("output.log")));
        assert!(patterns.should_ignore_file(Path::new("debug_trace")));
    }

    #[test]
    fn should_ignore_file_no_match() {
        let patterns = IgnorePatterns {
            files: vec!["*.log".to_string()],
            directories: vec!["target/".to_string()],
            packages: vec![],
        };
        assert!(!patterns.should_ignore_file(Path::new("src/main.rs")));
        assert!(!patterns.should_ignore_file(Path::new("Cargo.toml")));
    }

    #[test]
    fn should_ignore_file_directory_trailing_slash_stripped() {
        // The impl trims trailing '/' before checking contains()
        let patterns = IgnorePatterns {
            files: vec![],
            directories: vec!["build/".to_string()],
            packages: vec![],
        };
        assert!(patterns.should_ignore_file(Path::new("build/output.bin")));
        assert!(patterns.should_ignore_file(Path::new("project/build/output.bin")));
    }

    // ── IgnorePatterns::should_ignore_package ───────────────────────────────

    #[test]
    fn should_ignore_package_exact_match() {
        let patterns = IgnorePatterns {
            files: vec![],
            directories: vec![],
            packages: vec!["ipython".to_string(), "pytest".to_string()],
        };
        assert!(patterns.should_ignore_package("ipython"));
        assert!(patterns.should_ignore_package("pytest"));
    }

    #[test]
    fn should_ignore_package_no_match() {
        let patterns = IgnorePatterns {
            files: vec![],
            directories: vec![],
            packages: vec!["ipython".to_string()],
        };
        assert!(!patterns.should_ignore_package("numpy"));
        assert!(!patterns.should_ignore_package("ipython3")); // not an exact match
        assert!(!patterns.should_ignore_package(""));
    }

    #[test]
    fn should_ignore_package_empty_list() {
        let patterns = IgnorePatterns::default();
        assert!(!patterns.should_ignore_package("anything"));
    }

    // ── DriverConfig::from_manifest ─────────────────────────────────────────

    #[test]
    fn driver_config_empty_drivers() {
        let manifest = minimal_manifest();
        let config = DriverConfig::from_manifest(&manifest);
        assert!(config.drivers.is_empty());
        assert!(config.backends.is_empty());
    }

    #[test]
    fn driver_config_alias_expansion() {
        let mut manifest = minimal_manifest();
        manifest
            .drivers
            .insert("vision".to_string(), DriverValue::Enabled(true));
        let config = DriverConfig::from_manifest(&manifest);
        assert_eq!(config.drivers, vec!["camera", "depth-camera"]);
        assert!(config.backends.is_empty());
    }

    #[test]
    fn driver_config_direct_driver_name() {
        let mut manifest = minimal_manifest();
        manifest
            .drivers
            .insert("lidar".to_string(), DriverValue::Enabled(true));
        let config = DriverConfig::from_manifest(&manifest);
        assert_eq!(config.drivers, vec!["lidar"]);
    }

    #[test]
    fn driver_config_backend_override() {
        let mut manifest = minimal_manifest();
        manifest.drivers.insert(
            "lidar".to_string(),
            DriverValue::Backend("rplidar-a2".to_string()),
        );
        let config = DriverConfig::from_manifest(&manifest);
        assert_eq!(config.drivers, vec!["lidar"]);
        assert_eq!(config.backends.get("lidar").unwrap(), "rplidar-a2");
    }

    #[test]
    fn driver_config_alias_with_backend_stores_alias_key() {
        // When an alias like "vision" has a Backend value, the backend is stored
        // under the alias key, and the alias is expanded into individual drivers.
        let mut manifest = minimal_manifest();
        manifest.drivers.insert(
            "vision".to_string(),
            DriverValue::Backend("realsense".to_string()),
        );
        let config = DriverConfig::from_manifest(&manifest);
        assert_eq!(config.drivers, vec!["camera", "depth-camera"]);
        assert_eq!(config.backends.get("vision").unwrap(), "realsense");
    }

    #[test]
    fn driver_config_multiple_entries() {
        let mut manifest = minimal_manifest();
        // BTreeMap iterates in sorted order: "camera" then "navigation"
        manifest
            .drivers
            .insert("camera".to_string(), DriverValue::Enabled(true));
        manifest
            .drivers
            .insert("navigation".to_string(), DriverValue::Enabled(true));
        let config = DriverConfig::from_manifest(&manifest);
        // "camera" (direct) then "navigation" expands to lidar, gps, imu
        assert_eq!(config.drivers, vec!["camera", "lidar", "gps", "imu"]);
    }

    // ── enable_to_features ──────────────────────────────────────────────────

    #[test]
    fn enable_to_features_cuda() {
        assert_eq!(enable_to_features("cuda"), vec!["cuda"]);
    }

    #[test]
    fn enable_to_features_gpu_alias() {
        assert_eq!(enable_to_features("gpu"), vec!["cuda"]);
    }

    #[test]
    fn enable_to_features_python() {
        assert_eq!(enable_to_features("python"), vec!["python"]);
    }

    #[test]
    fn enable_to_features_py_alias() {
        assert_eq!(enable_to_features("py"), vec!["python"]);
    }

    #[test]
    fn enable_to_features_sim_empty() {
        let features = enable_to_features("sim");
        assert!(features.is_empty(), "sim should map to empty vec");
    }

    #[test]
    fn enable_to_features_simulation_alias_empty() {
        let features = enable_to_features("simulation");
        assert!(features.is_empty(), "simulation should map to empty vec");
    }

    #[test]
    fn enable_to_features_full() {
        assert_eq!(enable_to_features("full"), vec!["full"]);
    }

    #[test]
    fn enable_to_features_editor() {
        assert_eq!(enable_to_features("editor"), vec!["editor"]);
    }

    #[test]
    fn enable_to_features_headless() {
        assert_eq!(enable_to_features("headless"), vec!["headless"]);
    }

    #[test]
    fn enable_to_features_visual() {
        assert_eq!(enable_to_features("visual"), vec!["visual"]);
    }

    #[test]
    fn enable_to_features_opencv() {
        assert_eq!(enable_to_features("opencv"), vec!["opencv-backend"]);
        assert_eq!(
            enable_to_features("opencv-backend"),
            vec!["opencv-backend"]
        );
    }

    #[test]
    fn enable_to_features_performance() {
        assert_eq!(enable_to_features("io-uring"), vec!["io-uring-net"]);
        assert_eq!(enable_to_features("io-uring-net"), vec!["io-uring-net"]);
        assert_eq!(
            enable_to_features("ultra-low-latency"),
            vec!["ultra-low-latency"]
        );
    }

    #[test]
    fn enable_to_features_unknown_passthrough() {
        assert_eq!(
            enable_to_features("my-custom-feature"),
            vec!["my-custom-feature"]
        );
    }

    #[test]
    fn enable_to_features_case_insensitive() {
        assert_eq!(enable_to_features("CUDA"), vec!["cuda"]);
        assert_eq!(enable_to_features("Python"), vec!["python"]);
        assert_eq!(enable_to_features("GPU"), vec!["cuda"]);
    }

    // ── get_cargo_features_from_enable ──────────────────────────────────────

    #[test]
    fn get_cargo_features_from_enable_empty() {
        let config = EnableConfig {
            capabilities: vec![],
        };
        let features = get_cargo_features_from_enable(&config);
        assert!(features.is_empty());
    }

    #[test]
    fn get_cargo_features_from_enable_single() {
        let config = EnableConfig {
            capabilities: vec!["cuda".to_string()],
        };
        let features = get_cargo_features_from_enable(&config);
        assert_eq!(features, vec!["cuda"]);
    }

    #[test]
    fn get_cargo_features_from_enable_deduplication() {
        // "cuda" and "gpu" both map to "cuda" -- should appear only once
        let config = EnableConfig {
            capabilities: vec!["cuda".to_string(), "gpu".to_string()],
        };
        let features = get_cargo_features_from_enable(&config);
        assert_eq!(features, vec!["cuda"]);
    }

    #[test]
    fn get_cargo_features_from_enable_mixed() {
        let config = EnableConfig {
            capabilities: vec![
                "cuda".to_string(),
                "python".to_string(),
                "editor".to_string(),
            ],
        };
        let features = get_cargo_features_from_enable(&config);
        assert_eq!(features, vec!["cuda", "python", "editor"]);
    }

    #[test]
    fn get_cargo_features_from_enable_sim_adds_nothing() {
        let config = EnableConfig {
            capabilities: vec!["sim".to_string()],
        };
        let features = get_cargo_features_from_enable(&config);
        assert!(features.is_empty());
    }

    #[test]
    fn get_cargo_features_from_enable_sim_mixed_with_others() {
        let config = EnableConfig {
            capabilities: vec!["sim".to_string(), "cuda".to_string()],
        };
        let features = get_cargo_features_from_enable(&config);
        assert_eq!(features, vec!["cuda"]);
    }

    #[test]
    fn get_cargo_features_from_enable_dedup_python_py() {
        // "python" and "py" both map to "python"
        let config = EnableConfig {
            capabilities: vec!["python".to_string(), "py".to_string()],
        };
        let features = get_cargo_features_from_enable(&config);
        assert_eq!(features, vec!["python"]);
    }

    // ── IgnorePatterns from IgnoreConfig ────────────────────────────────────

    #[test]
    fn ignore_patterns_from_config() {
        let cfg = IgnoreConfig {
            files: vec!["*.log".to_string()],
            directories: vec!["target/".to_string()],
            packages: vec!["ipython".to_string()],
        };
        let patterns = IgnorePatterns::from(cfg);
        assert_eq!(patterns.files, vec!["*.log"]);
        assert_eq!(patterns.directories, vec!["target/"]);
        assert_eq!(patterns.packages, vec!["ipython"]);
    }

    #[test]
    fn ignore_patterns_default_is_empty() {
        let patterns = IgnorePatterns::default();
        assert!(patterns.files.is_empty());
        assert!(patterns.directories.is_empty());
        assert!(patterns.packages.is_empty());
        assert!(!patterns.should_ignore_file(Path::new("anything.rs")));
        assert!(!patterns.should_ignore_package("anything"));
    }
}
