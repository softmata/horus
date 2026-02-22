use crate::config::HORUS_YAML;
use anyhow::Result;
use std::fs;
use std::path::Path;

/// Resolve driver alias names to their constituent driver IDs.
/// Used when parsing `drivers:` section in horus.yaml.
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

/// Ignore patterns from horus.yaml
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

/// Parse ignore section from horus.yaml
pub fn parse_horus_yaml_ignore(path: &str) -> Result<IgnorePatterns> {
    let content = fs::read_to_string(path)?;

    match serde_yaml::from_str::<serde_yaml::Value>(&content) {
        Ok(yaml) => {
            let mut ignore = IgnorePatterns::default();

            if let Some(serde_yaml::Value::Mapping(ignore_map)) = yaml.get("ignore") {
                // Parse files
                if let Some(serde_yaml::Value::Sequence(files)) =
                    ignore_map.get(serde_yaml::Value::String("files".to_string()))
                {
                    for file in files {
                        if let serde_yaml::Value::String(pattern) = file {
                            ignore.files.push(pattern.clone());
                        }
                    }
                }

                // Parse directories
                if let Some(serde_yaml::Value::Sequence(dirs)) =
                    ignore_map.get(serde_yaml::Value::String("directories".to_string()))
                {
                    for dir in dirs {
                        if let serde_yaml::Value::String(pattern) = dir {
                            ignore.directories.push(pattern.clone());
                        }
                    }
                }

                // Parse packages
                if let Some(serde_yaml::Value::Sequence(pkgs)) =
                    ignore_map.get(serde_yaml::Value::String("packages".to_string()))
                {
                    for pkg in pkgs {
                        if let serde_yaml::Value::String(package) = pkg {
                            ignore.packages.push(package.clone());
                        }
                    }
                }
            }

            Ok(ignore)
        }
        Err(_) => Ok(IgnorePatterns::default()),
    }
}

/// Driver configuration from horus.yaml
#[derive(Debug, Clone, Default)]
pub struct DriverConfig {
    /// List of drivers to enable (e.g., ["camera", "lidar", "imu"])
    pub drivers: Vec<String>,
    /// Backend overrides (e.g., {"lidar": "rplidar-a2", "imu": "mpu6050"})
    pub backends: std::collections::HashMap<String, String>,
}

/// Parse drivers section from horus.yaml
///
/// Supports two formats:
/// ```yaml
/// # Simple list format
/// drivers:
///   - camera
///   - lidar
///   - imu
///
/// # Or with backend overrides
/// drivers:
///   camera: opencv
///   lidar: rplidar-a2
///   imu: mpu6050
/// ```
pub fn parse_horus_yaml_drivers(path: &str) -> Result<DriverConfig> {
    let content = fs::read_to_string(path)?;

    match serde_yaml::from_str::<serde_yaml::Value>(&content) {
        Ok(yaml) => {
            let mut config = DriverConfig::default();

            if let Some(drivers_value) = yaml.get("drivers") {
                match drivers_value {
                    // List format: drivers: [camera, lidar, imu]
                    serde_yaml::Value::Sequence(list) => {
                        for item in list {
                            if let serde_yaml::Value::String(driver) = item {
                                // Resolve aliases (e.g., "vision" -> ["camera", "depth-camera"])
                                if let Some(expanded) = resolve_driver_alias(driver) {
                                    for d in expanded {
                                        config.drivers.push(d.to_string());
                                    }
                                } else {
                                    config.drivers.push(driver.clone());
                                }
                            }
                        }
                    }

                    // Map format: drivers: { camera: opencv, lidar: rplidar-a2 }
                    serde_yaml::Value::Mapping(map) => {
                        for (key, value) in map {
                            if let serde_yaml::Value::String(driver_name) = key {
                                // Add to drivers list
                                if let Some(expanded) = resolve_driver_alias(driver_name) {
                                    for d in expanded {
                                        config.drivers.push(d.to_string());
                                    }
                                } else {
                                    config.drivers.push(driver_name.clone());
                                }

                                // Store backend override if specified
                                if let serde_yaml::Value::String(backend) = value {
                                    config.backends.insert(driver_name.clone(), backend.clone());
                                }
                            }
                        }
                    }

                    _ => {}
                }
            }

            Ok(config)
        }
        Err(_) => Ok(DriverConfig::default()),
    }
}

/// Get active drivers - combines CLI override, horus.yaml config, and HORUS_DRIVERS env var
pub fn get_active_drivers() -> DriverConfig {
    // Priority: HORUS_DRIVERS env var > CLI --drivers > horus.yaml
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

    // Fall back to horus.yaml
    if std::path::Path::new(HORUS_YAML).exists() {
        parse_horus_yaml_drivers(HORUS_YAML).unwrap_or_default()
    } else {
        DriverConfig::default()
    }
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

/// Get features string for cargo build command
///
/// Returns empty string if no features needed, or `--features "feat1,feat2"` format.
pub fn get_cargo_features_arg(config: &DriverConfig) -> Option<String> {
    let features = get_cargo_features_from_drivers(config);
    if features.is_empty() {
        None
    } else {
        Some(features.join(","))
    }
}

// ============================================================================
// Enable Capabilities Configuration
// ============================================================================

/// Enable configuration from horus.yaml or CLI --enable flag
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

        // Hardware interface features (shortcuts without -hardware suffix)
        "gpio" => vec!["gpio-hardware".to_string()],
        "i2c" => vec!["i2c-hardware".to_string()],
        "spi" => vec!["spi-hardware".to_string()],
        "can" => vec!["can-hardware".to_string()],
        "serial" => vec!["serial-hardware".to_string()],

        // Full hardware feature names (pass through)
        "gpio-hardware" => vec!["gpio-hardware".to_string()],
        "i2c-hardware" => vec!["i2c-hardware".to_string()],
        "spi-hardware" => vec!["spi-hardware".to_string()],
        "serial-hardware" => vec!["serial-hardware".to_string()],
        "motor-hardware" => vec!["motor-hardware".to_string()],
        "all-hardware" => vec!["all-hardware".to_string()],

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

/// Parse enable section from horus.yaml
///
/// Supports list format:
/// ```yaml
/// enable:
///   - cuda
///   - editor
///   - python
/// ```
pub fn parse_horus_yaml_enable(path: &str) -> Result<EnableConfig> {
    let content = fs::read_to_string(path)?;

    match serde_yaml::from_str::<serde_yaml::Value>(&content) {
        Ok(yaml) => {
            let mut config = EnableConfig::default();

            if let Some(serde_yaml::Value::Sequence(list)) = yaml.get("enable") {
                for item in list {
                    if let serde_yaml::Value::String(capability) = item {
                        config.capabilities.push(capability.clone());
                    }
                }
            }

            Ok(config)
        }
        Err(_) => Ok(EnableConfig::default()),
    }
}

/// Get active enable config - combines CLI override, horus.yaml config, and HORUS_ENABLE env var
pub fn get_active_enable() -> EnableConfig {
    // Priority: HORUS_ENABLE env var > horus.yaml
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

    // Fall back to horus.yaml
    if std::path::Path::new(HORUS_YAML).exists() {
        parse_horus_yaml_enable(HORUS_YAML).unwrap_or_default()
    } else {
        EnableConfig::default()
    }
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
