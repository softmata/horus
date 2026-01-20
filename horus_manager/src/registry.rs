// Simple registry client for HORUS package management
// Keeps complexity low - just HTTP calls to registry

use crate::dependency_resolver::{DependencySpec, PackageProvider};
use crate::progress::{self, finish_error, finish_success};
use anyhow::{anyhow, bail, Result};
use chrono::{DateTime, Utc};
use colored::*;
use flate2::read::GzDecoder;
use flate2::write::GzEncoder;
use flate2::Compression;
use reqwest::blocking::Client;
use semver::Version;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::fs;
use std::path::{Path, PathBuf};
use tar::Archive;
use tar::Builder;

#[derive(Debug, Serialize, Deserialize)]
pub struct Package {
    pub name: String,
    pub version: String,
    pub description: Option<String>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct PackageMetadata {
    pub name: String,
    pub version: String,
    pub checksum: Option<String>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct LockedPackage {
    pub name: String,
    pub version: String,
    pub checksum: String,
    pub source: PackageSource,
}

#[derive(Debug, Serialize, Deserialize, Clone, PartialEq)]
pub enum PackageSource {
    Registry, // HORUS registry (Rust, Python curated packages)
    PyPI,     // Python Package Index (external Python packages)
    CratesIO, // Rust crates.io (future)
    System,   // System packages (apt, brew, etc.)
    Path {
        // Local filesystem path (for development)
        path: String,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub enum SystemPackageChoice {
    UseSystem,    // Use existing system package
    InstallHORUS, // Install fresh copy to HORUS
    Cancel,       // Cancel installation
}

#[derive(Debug, Serialize, Deserialize)]
pub struct SystemInfo {
    pub os: String,
    pub arch: String,
    pub python_version: Option<String>,
    pub rust_version: Option<String>,
    pub gcc_version: Option<String>,
    pub cuda_version: Option<String>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct EnvironmentManifest {
    pub horus_id: String,
    pub name: Option<String>,
    pub description: Option<String>,
    pub packages: Vec<LockedPackage>,
    pub system: SystemInfo,
    pub created_at: DateTime<Utc>,
    pub horus_version: String,
}

/// Driver metadata from the registry API
/// Contains required features and dependencies for automatic installation
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct DriverMetadata {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bus_type: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub driver_category: Option<String>,
    /// Required Cargo features for this driver (e.g., ["serial-hardware", "i2c-hardware"])
    #[serde(skip_serializing_if = "Option::is_none")]
    pub required_features: Option<Vec<String>>,
    /// Cargo crate dependencies (e.g., ["serialport@4.2", "i2cdev@0.6"])
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cargo_dependencies: Option<Vec<String>>,
    /// Python package dependencies (e.g., ["pyserial>=3.5", "smbus2"])
    #[serde(skip_serializing_if = "Option::is_none")]
    pub python_dependencies: Option<Vec<String>>,
    /// System packages required (e.g., ["libudev-dev", "libusb-1.0-0-dev"])
    #[serde(skip_serializing_if = "Option::is_none")]
    pub system_dependencies: Option<Vec<String>>,
}

/// Response from the driver metadata API endpoint
#[derive(Debug, Deserialize)]
struct DriverMetadataResponse {
    #[serde(flatten)]
    pub driver_metadata: Option<DriverMetadata>,
}

/// Response from the driver list API endpoint
#[derive(Debug, Deserialize)]
struct DriverListResponse {
    pub drivers: Vec<DriverListEntry>,
}

/// Entry in the driver list response
#[derive(Debug, Clone, Deserialize)]
pub struct DriverListEntry {
    pub name: String,
    pub description: Option<String>,
    pub bus_type: Option<String>,
    pub category: Option<String>,
    #[serde(flatten)]
    pub driver_metadata: Option<DriverMetadata>,
}

/// URL-encode a package name for API calls
/// Kept for safety but package names should be simple alphanumeric now
pub fn url_encode_package_name(name: &str) -> String {
    name.to_string()
}

/// Convert a package name to a safe filesystem path component
pub fn package_name_to_path(name: &str) -> String {
    name.to_string()
}

pub struct RegistryClient {
    client: Client,
    base_url: String,
}

impl Default for RegistryClient {
    fn default() -> Self {
        Self::new()
    }
}

impl RegistryClient {
    pub fn new() -> Self {
        let base_url = std::env::var("HORUS_REGISTRY_URL")
            .unwrap_or_else(|_| "https://horus-marketplace-api.onrender.com".to_string());

        Self {
            client: Client::new(),
            base_url,
        }
    }

    /// Get a reference to the HTTP client
    pub fn http_client(&self) -> &Client {
        &self.client
    }

    /// Get the base URL of the registry
    pub fn base_url(&self) -> &str {
        &self.base_url
    }

    /// Fetch driver metadata for a package from the registry
    /// Returns error if the package is not a driver or not found in the registry
    pub fn fetch_driver_metadata(&self, package_name: &str) -> Result<DriverMetadata> {
        let encoded_name = url_encode_package_name(package_name);
        let url = format!("{}/api/drivers/{}", self.base_url, encoded_name);

        let response = self
            .client
            .get(&url)
            .send()
            .map_err(|e| anyhow!("Failed to fetch driver metadata: {}", e))?;

        if !response.status().is_success() {
            return Err(anyhow!("Driver '{}' not found in registry", package_name));
        }

        // Try to parse the driver metadata response
        let resp: DriverMetadataResponse = response
            .json()
            .map_err(|e| anyhow!("Failed to parse driver metadata: {}", e))?;

        resp.driver_metadata
            .ok_or_else(|| anyhow!("No driver metadata for '{}'", package_name))
    }

    /// Fetch driver metadata, returning Option (for backwards compatibility)
    pub fn fetch_driver_metadata_opt(&self, package_name: &str) -> Option<DriverMetadata> {
        self.fetch_driver_metadata(package_name).ok()
    }

    /// Fetch package type from registry (node, driver, plugin, tool, etc.)
    pub fn fetch_package_type(&self, package_name: &str) -> Result<String> {
        let encoded_name = url_encode_package_name(package_name);
        let url = format!("{}/api/packages/{}", self.base_url, encoded_name);

        let response = self
            .client
            .get(&url)
            .send()
            .map_err(|e| anyhow!("Failed to fetch package info: {}", e))?;

        if !response.status().is_success() {
            return Err(anyhow!("Package '{}' not found in registry", package_name));
        }

        // Parse the response to get package_type
        let resp: serde_json::Value = response
            .json()
            .map_err(|e| anyhow!("Failed to parse package info: {}", e))?;

        // Extract package_type from response, default to "node"
        let pkg_type = resp
            .get("package_type")
            .and_then(|v| v.as_str())
            .unwrap_or("node")
            .to_string();

        Ok(pkg_type)
    }

    /// Fetch driver metadata by querying the drivers list API
    pub fn query_driver_features(&self, driver_name: &str) -> Option<Vec<String>> {
        // Try direct driver metadata endpoint first
        if let Some(meta) = self.fetch_driver_metadata_opt(driver_name) {
            return meta.required_features;
        }

        // Try searching drivers by name
        let url = format!("{}/api/drivers?search={}", self.base_url, driver_name);
        if let Ok(response) = self.client.get(&url).send() {
            if response.status().is_success() {
                if let Ok(list) = response.json::<DriverListResponse>() {
                    // Find exact or best match
                    for driver in list.drivers {
                        if driver.name.eq_ignore_ascii_case(driver_name) {
                            return driver.driver_metadata.and_then(|m| m.required_features);
                        }
                    }
                }
            }
        }

        None
    }

    /// List all drivers from the registry, optionally filtered by category
    pub fn list_drivers(&self, category: Option<&str>) -> Result<Vec<DriverListEntry>> {
        let url = if let Some(cat) = category {
            format!("{}/api/drivers?category={}", self.base_url, cat)
        } else {
            format!("{}/api/drivers", self.base_url)
        };

        let response = self
            .client
            .get(&url)
            .send()
            .map_err(|e| anyhow!("Failed to fetch drivers: {}", e))?;

        if !response.status().is_success() {
            return Err(anyhow!("Registry returned error: {}", response.status()));
        }

        // Try to parse as DriverListResponse or as array directly
        let text = response
            .text()
            .map_err(|e| anyhow!("Failed to read response: {}", e))?;

        // Try parsing as { "drivers": [...] } first
        if let Ok(list) = serde_json::from_str::<DriverListResponse>(&text) {
            return Ok(list.drivers);
        }

        // Try parsing as direct array
        if let Ok(drivers) = serde_json::from_str::<Vec<DriverListEntry>>(&text) {
            return Ok(drivers);
        }

        Ok(vec![])
    }

    /// Search for drivers by query string and optional bus type
    pub fn search_drivers(
        &self,
        query: &str,
        bus_type: Option<&str>,
    ) -> Result<Vec<DriverListEntry>> {
        let mut url = format!("{}/api/drivers?search={}", self.base_url, query);
        if let Some(bus) = bus_type {
            url.push_str(&format!("&bus_type={}", bus));
        }

        let response = self
            .client
            .get(&url)
            .send()
            .map_err(|e| anyhow!("Failed to search drivers: {}", e))?;

        if !response.status().is_success() {
            return Err(anyhow!("Registry returned error: {}", response.status()));
        }

        let text = response
            .text()
            .map_err(|e| anyhow!("Failed to read response: {}", e))?;

        // Try parsing as { "drivers": [...] } first
        if let Ok(list) = serde_json::from_str::<DriverListResponse>(&text) {
            return Ok(list.drivers);
        }

        // Try parsing as direct array
        if let Ok(drivers) = serde_json::from_str::<Vec<DriverListEntry>>(&text) {
            return Ok(drivers);
        }

        Ok(vec![])
    }

    // Install a package to a specific target (used by install_to_target)
    // Returns the actual installed version
    pub fn install(&self, package_name: &str, version: Option<&str>) -> Result<String> {
        // Default: auto-detect global/local
        use crate::workspace;
        let target = workspace::detect_or_select_workspace(true)?;
        self.install_to_target(package_name, version, target)
    }

    // Install a package from registry to a specific target
    // Returns the actual installed version
    pub fn install_to_target(
        &self,
        package_name: &str,
        version: Option<&str>,
        target: crate::workspace::InstallTarget,
    ) -> Result<String> {
        // Detect package source
        let source = self.detect_package_source(package_name)?;

        match source {
            PackageSource::Registry => self.install_from_registry(package_name, version, target),
            PackageSource::PyPI => self.install_from_pypi(package_name, version, target),
            PackageSource::CratesIO => self.install_from_cratesio(package_name, version, target),
            PackageSource::System => Err(anyhow!(
                "System packages not supported via horus pkg install"
            )),
            PackageSource::Path { .. } => Err(anyhow!(
                "Path dependencies must be specified in horus.yaml.\n\
                     Use 'horus run' to install dependencies from horus.yaml."
            )),
        }
    }

    fn detect_package_source(&self, package_name: &str) -> Result<PackageSource> {
        // Scoped packages (@org/name) are always from HORUS registry
        if package_name.starts_with('@') {
            return Ok(PackageSource::Registry);
        }

        // Check if it's a HORUS package
        if package_name.starts_with("horus") {
            return Ok(PackageSource::Registry);
        }

        // Try HORUS registry first (URL encode for safety)
        let encoded_name = url_encode_package_name(package_name);
        let url = format!("{}/api/packages/{}", self.base_url, encoded_name);
        if let Ok(response) = self.client.get(&url).send() {
            if response.status().is_success() {
                return Ok(PackageSource::Registry);
            }
        }

        // Check BOTH PyPI and crates.io to detect ambiguity
        let in_pypi = self.check_pypi_exists(package_name);
        let in_crates = self.check_crates_exists(package_name);

        // Handle ambiguity - package exists in both registries
        if in_pypi && in_crates {
            println!(
                "\n{} Package '{}' found in BOTH PyPI and crates.io",
                "[WARNING]".yellow(),
                package_name.green()
            );
            return self.prompt_package_source_choice(package_name);
        }

        // Package only in crates.io
        if in_crates {
            return Ok(PackageSource::CratesIO);
        }

        // Package only in PyPI or not found (default to PyPI)
        Ok(PackageSource::PyPI)
    }

    fn check_pypi_exists(&self, package_name: &str) -> bool {
        // Check PyPI API
        let pypi_url = format!("https://pypi.org/pypi/{}/json", package_name);
        if let Ok(response) = self.client.get(&pypi_url).send() {
            return response.status().is_success();
        }
        false
    }

    fn check_crates_exists(&self, package_name: &str) -> bool {
        // Check crates.io API
        let crates_url = format!("https://crates.io/api/v1/crates/{}", package_name);
        if let Ok(response) = self
            .client
            .get(&crates_url)
            .header("User-Agent", "horus-pkg-manager")
            .send()
        {
            return response.status().is_success();
        }
        false
    }

    fn prompt_package_source_choice(&self, _package_name: &str) -> Result<PackageSource> {
        use std::io::{self, Write};

        println!("\nWhich package source do you want to use?");
        println!("  [1] {} PyPI (Python package)", "[PYTHON]".cyan());
        println!("  [2] {} crates.io (Rust binary)", "[RUST]".cyan());
        println!("  [3] {} Cancel installation", "[FAIL]".red());

        print!("\nChoice [1-3]: ");
        io::stdout().flush()?;

        let mut input = String::new();
        io::stdin().read_line(&mut input)?;

        match input.trim() {
            "1" => {
                println!("   Using PyPI (Python)");
                Ok(PackageSource::PyPI)
            }
            "2" => {
                println!("   Using crates.io (Rust)");
                Ok(PackageSource::CratesIO)
            }
            "3" => {
                bail!("Installation cancelled by user")
            }
            _ => {
                println!("Invalid choice, defaulting to PyPI");
                Ok(PackageSource::PyPI)
            }
        }
    }

    // Install a dependency from DependencySpec (supports path and registry)
    pub fn install_dependency_spec(
        &self,
        spec: &crate::dependency_resolver::DependencySpec,
        target: crate::workspace::InstallTarget,
        base_dir: Option<&Path>,
    ) -> Result<()> {
        use crate::dependency_resolver::DependencySource;

        match &spec.source {
            DependencySource::Registry => {
                // For registry dependencies, use version from requirement if specific
                let version_str = if spec.requirement.to_string() != "*" {
                    Some(spec.requirement.to_string())
                } else {
                    None
                };
                self.install_from_registry(&spec.name, version_str.as_deref(), target)
                    .map(|_| ()) // Ignore version for dependency spec
            }
            DependencySource::Path(path) => {
                self.install_from_path(&spec.name, path, target, base_dir)
                    .map(|_| ()) // Ignore version for dependency spec
            }
            DependencySource::Git {
                url,
                branch,
                tag,
                rev,
            } => {
                // Git dependencies are handled by horus run - skip for pkg install
                // They are cloned to ~/.horus/cache/git_* and used as path deps
                eprintln!(
                    "  {} Git dependency '{}' from {} - use 'horus run' to clone automatically",
                    "ℹ".cyan(),
                    spec.name,
                    url
                );
                if let Some(b) = branch {
                    eprintln!("      Branch: {}", b);
                } else if let Some(t) = tag {
                    eprintln!("      Tag: {}", t);
                } else if let Some(r) = rev {
                    eprintln!("      Rev: {}", r);
                }
                Ok(())
            }
            DependencySource::Pip { package_name } => {
                // Install from PyPI
                let version_str = if spec.requirement.to_string() != "*" {
                    Some(spec.requirement.to_string())
                } else {
                    None
                };
                self.install_from_pypi(package_name, version_str.as_deref(), target)
                    .map(|_| ()) // Ignore version for dependency spec
            }
        }
    }

    fn install_from_registry(
        &self,
        package_name: &str,
        version: Option<&str>,
        target: crate::workspace::InstallTarget,
    ) -> Result<String> {
        let spinner = progress::robot_download_spinner(&format!(
            "Downloading {} from HORUS registry...",
            package_name
        ));

        let version_str = version.unwrap_or("latest");
        // URL-encode scoped package names for API calls
        let encoded_name = url_encode_package_name(package_name);
        let url = format!(
            "{}/api/packages/{}/{}/download",
            self.base_url, encoded_name, version_str
        );

        // Download package
        let response = self.client.get(&url).send()?;

        if !response.status().is_success() {
            return Err(anyhow!("Package not found: {}", package_name));
        }

        let bytes = response.bytes()?;

        // Calculate checksum
        let mut hasher = Sha256::new();
        hasher.update(&bytes);
        let checksum = format!("{:x}", hasher.finalize());

        // Convert scoped package name to safe path (e.g., @org/pkg -> org--pkg)
        let safe_pkg_name = package_name_to_path(package_name);

        // Determine installation directory based on target
        use crate::workspace::InstallTarget;
        let home = dirs::home_dir().ok_or_else(|| anyhow!("Could not find home directory"))?;
        let global_cache = home.join(".horus/cache");

        let (install_dir, install_type, local_packages_dir) = match &target {
            InstallTarget::Global => {
                // Force global installation
                fs::create_dir_all(&global_cache)?;
                let current_local = PathBuf::from(".horus/packages");
                (global_cache.clone(), "global", Some(current_local))
            }
            InstallTarget::Local(workspace_path) => {
                // Install to specific workspace
                let local_packages = workspace_path.join(".horus/packages");
                fs::create_dir_all(&local_packages)?;

                // Check if any version exists in global cache (use safe name)
                let has_global_versions = check_global_versions(&global_cache, &safe_pkg_name)?;

                if has_global_versions {
                    // Install to global and symlink
                    fs::create_dir_all(&global_cache)?;
                    (global_cache.clone(), "global", Some(local_packages))
                } else {
                    // Install locally
                    (local_packages.clone(), "local", None)
                }
            }
        };

        // Create package directory with version
        let tar = GzDecoder::new(&bytes[..]);
        let mut archive = Archive::new(tar);

        // Extract to temporary location first to detect version (use safe name)
        let temp_dir = std::env::temp_dir().join(format!("horus_pkg_{}", safe_pkg_name));
        fs::create_dir_all(&temp_dir)?;
        archive.unpack(&temp_dir)?;

        // Get actual version from package info (for "latest" downloads)
        let actual_version = if version_str == "latest" {
            detect_package_version(&temp_dir).unwrap_or_else(|| version_str.to_string())
        } else {
            version_str.to_string()
        };

        // Move to final location with version info (use safe path name)
        let package_dir = if install_type == "global" {
            install_dir.join(format!("{}@{}", safe_pkg_name, actual_version))
        } else {
            install_dir.join(&safe_pkg_name)
        };

        // Remove existing if present
        if package_dir.exists() {
            fs::remove_dir_all(&package_dir)?;
        }
        fs::create_dir_all(&package_dir)?;

        // Move from temp to final location
        copy_dir_all(&temp_dir, &package_dir)?;
        fs::remove_dir_all(&temp_dir)?;

        // Create metadata.json for tracking
        let metadata = PackageMetadata {
            name: package_name.to_string(),
            version: actual_version.clone(),
            checksum: Some(checksum),
        };

        let metadata_path = package_dir.join("metadata.json");
        fs::write(&metadata_path, serde_json::to_string_pretty(&metadata)?)?;

        // If installed to global, create symlink in local workspace
        if install_type == "global" {
            if let Some(local_pkg_dir) = local_packages_dir {
                fs::create_dir_all(&local_pkg_dir)?;
                let local_link = local_pkg_dir.join(&safe_pkg_name);

                // Remove existing symlink/dir if present
                if local_link.exists() || local_link.symlink_metadata().is_ok() {
                    #[cfg(unix)]
                    {
                        if local_link.symlink_metadata()?.is_symlink() {
                            fs::remove_file(&local_link)?;
                        } else {
                            fs::remove_dir_all(&local_link)?;
                        }
                    }
                    #[cfg(windows)]
                    {
                        if local_link.is_dir() {
                            fs::remove_dir_all(&local_link)?;
                        } else {
                            fs::remove_file(&local_link)?;
                        }
                    }
                }

                // Create symlink
                #[cfg(unix)]
                std::os::unix::fs::symlink(&package_dir, &local_link)?;
                #[cfg(windows)]
                std::os::windows::fs::symlink_dir(&package_dir, &local_link)?;

                finish_success(
                    &spinner,
                    &format!("Installed {} v{}", package_name, actual_version),
                );
                println!(
                    "   {} Linked: {} -> {}",
                    "".dimmed(),
                    local_link.display(),
                    package_dir.display()
                );
            } else {
                finish_success(
                    &spinner,
                    &format!("Installed {} v{}", package_name, actual_version),
                );
                println!("   {} Location: {}", "".dimmed(), package_dir.display());
            }
        } else {
            finish_success(
                &spinner,
                &format!("Installed {} v{} locally", package_name, actual_version),
            );
            println!("   {} Location: {}", "".dimmed(), package_dir.display());
        }

        // Pre-compile if installed to global cache and is Rust/C package
        if install_type == "global" {
            if let Err(e) = precompile_package(&package_dir) {
                println!("  {} Pre-compilation skipped: {}", "".yellow(), e);
            }
        }

        // Resolve transitive dependencies
        if let Ok(deps) = extract_package_dependencies(&package_dir) {
            if !deps.is_empty() {
                println!("  {} Found {} dependencies", "".cyan(), deps.len());
                for dep in &deps {
                    println!("    • {} {}", dep.name, dep.requirement);
                }

                // Recursively install dependencies
                self.install_dependencies(&deps, &target)?;
            }
        }

        // Fetch and apply driver metadata if this is a driver package
        if let Some(driver_meta) = self.fetch_driver_metadata_opt(package_name) {
            self.apply_driver_requirements(&driver_meta, &target)?;
        }

        Ok(actual_version)
    }

    /// Apply driver requirements (features, dependencies, system packages)
    fn apply_driver_requirements(
        &self,
        driver_meta: &DriverMetadata,
        target: &crate::workspace::InstallTarget,
    ) -> Result<()> {
        use crate::workspace::InstallTarget;

        // Get workspace path for local installations
        let workspace_path = match target {
            InstallTarget::Local(path) => Some(path.clone()),
            InstallTarget::Global => None,
        };

        // Apply required Cargo features
        if let Some(features) = &driver_meta.required_features {
            if !features.is_empty() {
                println!("  {} Driver requires features:", "[BUILD]".cyan());
                for feature in features {
                    println!("    • {}", feature.yellow());
                }

                // Update Cargo.toml if we have a workspace path
                if let Some(ws_path) = &workspace_path {
                    if let Err(e) = add_features_to_cargo_toml(ws_path, features) {
                        println!(
                            "  {} Could not auto-add features to Cargo.toml: {}",
                            "[WARN]".yellow(),
                            e
                        );
                        println!(
                            "    Add manually: horus_library = {{ features = {:?} }}",
                            features
                        );
                    } else {
                        println!("  {} Added features to Cargo.toml", "[OK]".green());
                    }
                }
            }
        }

        // Add Cargo dependencies (crates.io)
        if let Some(cargo_deps) = &driver_meta.cargo_dependencies {
            if !cargo_deps.is_empty() {
                println!("  {} Cargo dependencies required:", "[PKG]".cyan());
                for dep in cargo_deps {
                    println!("    • {}", dep.yellow());
                }

                // Update Cargo.toml if we have a workspace path
                if let Some(ws_path) = &workspace_path {
                    if let Err(e) = add_cargo_deps_to_cargo_toml(ws_path, cargo_deps) {
                        println!(
                            "  {} Could not auto-add dependencies to Cargo.toml: {}",
                            "[WARN]".yellow(),
                            e
                        );
                        println!("    Add manually to [dependencies]");
                    } else {
                        println!("  {} Added dependencies to Cargo.toml", "[OK]".green());
                    }
                }
            }
        }

        // Install Python dependencies (PyPI)
        if let Some(py_deps) = &driver_meta.python_dependencies {
            if !py_deps.is_empty() {
                println!("  {} Python dependencies required:", "[PY]".cyan());
                for dep in py_deps {
                    println!("    • {}", dep);
                }
                // Auto-install with pip
                for dep in py_deps {
                    let status = std::process::Command::new("pip3")
                        .args(["install", "--quiet", dep])
                        .status();
                    if status.is_ok() {
                        println!("  {} Installed {} via pip", "[OK]".green(), dep);
                    }
                }
            }
        }

        // Handle system dependencies with smart detection
        if let Some(sys_deps) = &driver_meta.system_dependencies {
            if !sys_deps.is_empty() {
                handle_system_dependencies(sys_deps)?;
            }
        }

        Ok(())
    }

    fn install_from_pypi(
        &self,
        package_name: &str,
        version: Option<&str>,
        target: crate::workspace::InstallTarget,
    ) -> Result<String> {
        use std::process::Command;
        let spinner =
            progress::robot_download_spinner(&format!("Installing {} from PyPI...", package_name));

        // Check if package exists in system first
        if let Ok(Some(system_version)) = self.detect_system_python_package(package_name) {
            let choice =
                self.prompt_system_package_choice(package_name, &system_version, "PyPI")?;

            match choice {
                SystemPackageChoice::Cancel => {
                    return Err(anyhow!("Installation cancelled by user"));
                }
                SystemPackageChoice::UseSystem => {
                    // Create reference to system package instead of installing
                    return self.create_system_reference_python(
                        package_name,
                        &system_version,
                        &target,
                    );
                }
                SystemPackageChoice::InstallHORUS => {
                    // Continue with installation below
                    println!("  {} Installing isolated copy to HORUS...", "".blue());
                }
            }
        }

        // Determine installation location based on target
        use crate::workspace::InstallTarget;
        let home = dirs::home_dir().ok_or_else(|| anyhow!("Could not find home directory"))?;
        let global_cache = home.join(".horus/cache");

        let (install_dir, is_global, local_packages_dir) = match &target {
            InstallTarget::Global => {
                // Install to global cache
                fs::create_dir_all(&global_cache)?;
                let current_local = PathBuf::from(".horus/packages");
                (global_cache.clone(), true, Some(current_local))
            }
            InstallTarget::Local(workspace_path) => {
                // Install to workspace packages
                let local_packages = workspace_path.join(".horus/packages");
                fs::create_dir_all(&local_packages)?;
                (local_packages.clone(), false, None)
            }
        };

        // Create temp venv for pip operations
        let temp_venv = PathBuf::from(".horus/venv");
        if !temp_venv.exists() {
            fs::create_dir_all(&temp_venv)?;
            let python_cmd = if Command::new("python3").arg("--version").output().is_ok() {
                "python3"
            } else {
                "python"
            };
            Command::new(python_cmd)
                .args(["-m", "venv", temp_venv.to_str().unwrap()])
                .status()?;
        }

        let pip_path = temp_venv.join("bin/pip");

        // Build version string
        let version_str = version.unwrap_or("latest");
        let requirement = if version_str == "latest" {
            package_name.to_string()
        } else {
            format!("{}=={}", package_name, version_str)
        };

        // Install to target directory
        let pkg_dir = if is_global {
            install_dir.join(format!("pypi_{}@{}", package_name, version_str))
        } else {
            install_dir.join(package_name)
        };

        if pkg_dir.exists() {
            fs::remove_dir_all(&pkg_dir)?;
        }
        fs::create_dir_all(&pkg_dir)?;

        spinner.set_message(format!("Installing {} with pip...", package_name));
        let output = Command::new(&pip_path)
            .args(["install", "--target", pkg_dir.to_str().unwrap()])
            .arg(&requirement)
            .output()?;

        if !output.status.success() {
            finish_error(
                &spinner,
                &format!("pip install failed for {}", package_name),
            );
            let stderr = String::from_utf8_lossy(&output.stderr);
            return Err(anyhow!("pip install failed:\n{}", stderr));
        }

        // Detect actual installed version from .dist-info directory
        let actual_version = detect_pypi_installed_version(&pkg_dir, package_name)
            .unwrap_or_else(|| version_str.to_string());

        // Create metadata.json
        let metadata = serde_json::json!({
            "name": package_name,
            "version": actual_version,
            "source": "PyPI"
        });
        let metadata_path = pkg_dir.join("metadata.json");
        fs::write(&metadata_path, serde_json::to_string_pretty(&metadata)?)?;

        // If global, create symlink
        if is_global {
            if let Some(local_pkg_dir) = local_packages_dir {
                fs::create_dir_all(&local_pkg_dir)?;
                let local_link = local_pkg_dir.join(package_name);

                // Remove existing
                if local_link.exists() || local_link.symlink_metadata().is_ok() {
                    #[cfg(unix)]
                    {
                        if local_link.symlink_metadata()?.is_symlink() {
                            fs::remove_file(&local_link)?;
                        } else {
                            fs::remove_dir_all(&local_link)?;
                        }
                    }
                    #[cfg(windows)]
                    {
                        if local_link.is_dir() {
                            fs::remove_dir_all(&local_link)?;
                        } else {
                            fs::remove_file(&local_link)?;
                        }
                    }
                }

                // Create symlink
                #[cfg(unix)]
                std::os::unix::fs::symlink(&pkg_dir, &local_link)?;
                #[cfg(windows)]
                std::os::windows::fs::symlink_dir(&pkg_dir, &local_link)?;

                finish_success(
                    &spinner,
                    &format!("Installed {} v{}", package_name, actual_version),
                );
                println!(
                    "   {} Linked: {} -> {}",
                    "".dimmed(),
                    local_link.display(),
                    pkg_dir.display()
                );
            }
        } else {
            finish_success(
                &spinner,
                &format!("Installed {} v{} locally", package_name, actual_version),
            );
            println!("   {} Location: {}", "".dimmed(), pkg_dir.display());
        }

        Ok(actual_version)
    }

    fn install_from_cratesio(
        &self,
        package_name: &str,
        version: Option<&str>,
        target: crate::workspace::InstallTarget,
    ) -> Result<String> {
        use std::process::Command;
        let spinner = progress::robot_download_spinner(&format!(
            "Installing {} from crates.io...",
            package_name
        ));

        // Check if cargo is available
        if Command::new("cargo").arg("--version").output().is_err() {
            return Err(anyhow!(
                "cargo not found. Please install Rust toolchain from https://rustup.rs"
            ));
        }

        // Check if binary exists in system first
        if let Ok(Some(system_version)) = self.detect_system_cargo_binary(package_name) {
            let choice =
                self.prompt_system_package_choice(package_name, &system_version, "crates.io")?;

            match choice {
                SystemPackageChoice::Cancel => {
                    return Err(anyhow!("Installation cancelled by user"));
                }
                SystemPackageChoice::UseSystem => {
                    // Create reference to system binary instead of installing
                    return self.create_system_reference_cargo(
                        package_name,
                        &system_version,
                        &target,
                    );
                }
                SystemPackageChoice::InstallHORUS => {
                    // Continue with installation below
                    println!("  {} Installing isolated copy to HORUS...", "".blue());
                }
            }
        }

        // Determine installation location based on target
        use crate::workspace::InstallTarget;
        let home = dirs::home_dir().ok_or_else(|| anyhow!("Could not find home directory"))?;
        let global_cache = home.join(".horus/cache");

        let (install_root, is_global, local_packages_dir) = match &target {
            InstallTarget::Global => {
                // Install to global cache
                fs::create_dir_all(&global_cache)?;
                let current_local = PathBuf::from(".horus/packages");
                (global_cache.clone(), true, Some(current_local))
            }
            InstallTarget::Local(workspace_path) => {
                // Install to workspace packages
                let local_packages = workspace_path.join(".horus/packages");
                fs::create_dir_all(&local_packages)?;
                (local_packages.clone(), false, None)
            }
        };

        // Build version string
        let version_str = version.unwrap_or("latest");
        let crate_spec = if version_str == "latest" {
            package_name.to_string()
        } else {
            format!("{}@{}", package_name, version_str)
        };

        // Install directory
        let pkg_dir = if is_global {
            install_root.join(format!("cratesio_{}@{}", package_name, version_str))
        } else {
            install_root.join(package_name)
        };

        if pkg_dir.exists() {
            fs::remove_dir_all(&pkg_dir)?;
        }
        fs::create_dir_all(&pkg_dir)?;

        spinner.set_message(format!("Installing {} with cargo...", package_name));

        // Use cargo install with --root to install to specific directory
        let mut cmd = Command::new("cargo");
        cmd.arg("install");
        cmd.arg(&crate_spec);
        cmd.arg("--root");
        cmd.arg(&pkg_dir);

        let output = cmd.output()?;

        if !output.status.success() {
            finish_error(
                &spinner,
                &format!("cargo install failed for {}", package_name),
            );
            let stderr = String::from_utf8_lossy(&output.stderr);
            return Err(anyhow!("cargo install failed:\n{}", stderr));
        }

        // Detect actual installed version from the binary
        let actual_version = detect_cargo_installed_version(&pkg_dir, package_name)
            .unwrap_or_else(|| version_str.to_string());

        // Create metadata.json
        let metadata = serde_json::json!({
            "name": package_name,
            "version": actual_version,
            "source": "CratesIO"
        });
        let metadata_path = pkg_dir.join("metadata.json");
        fs::write(&metadata_path, serde_json::to_string_pretty(&metadata)?)?;

        // If global, create symlink
        if is_global {
            if let Some(local_pkg_dir) = local_packages_dir {
                fs::create_dir_all(&local_pkg_dir)?;
                let local_link = local_pkg_dir.join(package_name);

                // Remove existing
                if local_link.exists() || local_link.symlink_metadata().is_ok() {
                    #[cfg(unix)]
                    {
                        if local_link.symlink_metadata()?.is_symlink() {
                            fs::remove_file(&local_link)?;
                        } else {
                            fs::remove_dir_all(&local_link)?;
                        }
                    }
                    #[cfg(windows)]
                    {
                        if local_link.is_dir() {
                            fs::remove_dir_all(&local_link)?;
                        } else {
                            fs::remove_file(&local_link)?;
                        }
                    }
                }

                // Create symlink
                #[cfg(unix)]
                std::os::unix::fs::symlink(&pkg_dir, &local_link)?;
                #[cfg(windows)]
                std::os::windows::fs::symlink_dir(&pkg_dir, &local_link)?;

                finish_success(
                    &spinner,
                    &format!("Installed {} v{}", package_name, actual_version),
                );
                println!(
                    "   {} Linked: {} -> {}",
                    "".dimmed(),
                    local_link.display(),
                    pkg_dir.display()
                );
                println!("   {} Binaries: {}/bin/", "".dimmed(), pkg_dir.display());
            }
        } else {
            finish_success(
                &spinner,
                &format!("Installed {} v{} locally", package_name, actual_version),
            );
            println!("   {} Location: {}", "".dimmed(), pkg_dir.display());
            println!("   {} Binaries: {}/bin/", "".dimmed(), pkg_dir.display());
        }

        Ok(actual_version)
    }

    // Install multiple dependencies recursively
    fn install_dependencies(
        &self,
        dependencies: &[DependencySpec],
        target: &crate::workspace::InstallTarget,
    ) -> Result<()> {
        // Use dependency resolver for version resolution
        use crate::dependency_resolver::{DependencyResolver, ResolvedDependency};

        println!("  {} Resolving dependency versions...", "".cyan());

        // Create resolver with this registry client as provider
        let mut resolver = DependencyResolver::new(self);

        // Resolve all dependencies with version constraints
        let resolved: Vec<ResolvedDependency> = match resolver.resolve(dependencies.to_vec()) {
            Ok(r) => r,
            Err(e) => {
                println!("  {} Dependency resolution failed: {}", "".red(), e);
                println!("  {} Falling back to simple installation...", "".yellow());

                // Fallback: install without version resolution
                for dep in dependencies {
                    let dep_name = &dep.name;

                    // Check if already installed
                    let is_installed = match target {
                        crate::workspace::InstallTarget::Global => {
                            let home = dirs::home_dir()
                                .ok_or_else(|| anyhow!("Could not find home directory"))?;
                            let global_cache = home.join(".horus/cache");
                            check_global_versions(&global_cache, dep_name)?
                        }
                        crate::workspace::InstallTarget::Local(workspace_path) => {
                            let local_packages = workspace_path.join(".horus/packages");
                            local_packages.join(dep_name).exists()
                        }
                    };

                    if is_installed {
                        println!("  {} {} (already installed)", "".green(), dep_name);
                        continue;
                    }

                    // Install latest version
                    println!("  {} Installing dependency: {}...", "".cyan(), dep_name);
                    self.install_to_target(dep_name, None, target.clone())?;
                }
                return Ok(());
            }
        };

        // Install resolved versions
        for resolved_dep in resolved {
            let version_str = resolved_dep.version.to_string();
            let home = dirs::home_dir().ok_or_else(|| anyhow!("Could not find home directory"))?;
            let global_cache = home.join(".horus/cache");

            // Check if already installed
            let (is_installed_local, is_installed_global) = match &target {
                crate::workspace::InstallTarget::Global => {
                    let has_global = check_global_versions(&global_cache, &resolved_dep.name)?;
                    (has_global, has_global)
                }
                crate::workspace::InstallTarget::Local(workspace_path) => {
                    let local_packages = workspace_path.join(".horus/packages");
                    let has_local = local_packages.join(&resolved_dep.name).exists();
                    let has_global =
                        check_global_versions(&global_cache, &resolved_dep.name).unwrap_or(false);
                    (has_local, has_global)
                }
            };

            if is_installed_local {
                println!(
                    "  {} {} v{} (already installed)",
                    "".green(),
                    resolved_dep.name,
                    resolved_dep.version
                );
                continue;
            }

            // If package exists in global cache but not local, create symlink instead of downloading
            if !is_installed_local && is_installed_global {
                if let crate::workspace::InstallTarget::Local(workspace_path) = &target {
                    println!(
                        "  {} Linking {} v{} from global cache...",
                        "".cyan(),
                        resolved_dep.name,
                        resolved_dep.version
                    );

                    // Find the global package directory
                    let package_dir_name = format!("{}@{}", resolved_dep.name, version_str);
                    let global_package_dir = global_cache.join(&package_dir_name);

                    if global_package_dir.exists() {
                        let local_packages = workspace_path.join(".horus/packages");
                        fs::create_dir_all(&local_packages)?;
                        let local_link = local_packages.join(&resolved_dep.name);

                        // Create symlink
                        #[cfg(unix)]
                        std::os::unix::fs::symlink(&global_package_dir, &local_link)?;
                        #[cfg(windows)]
                        std::os::windows::fs::symlink_dir(&global_package_dir, &local_link)?;

                        println!(
                            "  {} {} v{} (linked from global cache)",
                            "".green(),
                            resolved_dep.name,
                            resolved_dep.version
                        );
                        continue;
                    }
                }
            }

            // Install the resolved version from registry
            println!(
                "  {} Installing {} v{}...",
                "".cyan(),
                resolved_dep.name,
                resolved_dep.version
            );
            self.install_to_target(&resolved_dep.name, Some(&version_str), target.clone())?;
        }

        Ok(())
    }

    // Publish a package to registry
    pub fn publish(&self, path: Option<&Path>) -> Result<()> {
        let current_dir = path.unwrap_or_else(|| Path::new("."));

        // Simple detection - just get name, version, description, license
        let (name, version, description, license) = detect_package_info(current_dir)?;

        // Validate dependencies - check for path/git deps before publishing
        let yaml_path = current_dir.join("horus.yaml");
        if yaml_path.exists() {
            use crate::commands::run::parse_horus_yaml_dependencies_v2;
            use crate::dependency_resolver::DependencySource;

            match parse_horus_yaml_dependencies_v2(yaml_path.to_str().unwrap()) {
                Ok(deps) => {
                    let mut has_path_deps = false;

                    for dep in deps {
                        if let DependencySource::Path(p) = dep.source {
                            println!(
                                "\n{} Cannot publish package with path dependencies!",
                                "Error:".red()
                            );
                            println!("  Path dependency: {} -> {}", dep.name, p.display());
                            println!(
                                "\n{}",
                                "Path dependencies are not reproducible and cannot be published."
                                    .yellow()
                            );
                            println!("{}", "Please publish the path dependency to the registry first, then update horus.yaml.".yellow());
                            has_path_deps = true;
                        }
                    }

                    if has_path_deps {
                        return Err(anyhow!("Cannot publish package with path dependencies"));
                    }
                }
                Err(_) => {
                    // If parsing fails, continue (might be old format or no deps)
                }
            }
        }

        println!(" Publishing {} v{}...", name, version);

        // Read API key from auth config (with helpful error message)
        let api_key = match get_api_key() {
            Ok(key) => key,
            Err(_) => {
                println!("\n Not authenticated with HORUS registry.");
                println!("\nTo publish packages, you need to authenticate:");
                println!("  1. Run: horus auth login");
                println!("  2. Authorize in your browser");
                println!("  3. The registry will show your API key");
                println!("  4. Save it to ~/.horus/auth.json");
                println!("\nThen try publishing again!");
                return Err(anyhow!("Authentication required"));
            }
        };

        // Create tar.gz of the package (use safe path for temp file)
        let safe_name = package_name_to_path(&name);
        let tar_path = std::env::temp_dir().join(format!("{}-{}.tar.gz", safe_name, version));

        // Create tarball in a scope to ensure proper flushing
        {
            let tar_file = fs::File::create(&tar_path)?;
            let encoder = GzEncoder::new(tar_file, Compression::default());
            let mut tar_builder = Builder::new(encoder);

            // Add all files to tar (excluding .git, target, node_modules)
            tar_builder.append_dir_all(".", current_dir)?;
            tar_builder.finish()?;

            // Explicitly drop to flush encoder before reading
        } // encoder and tar_builder dropped here, ensuring flush

        // Read the tar file after it's fully written
        let package_data = fs::read(&tar_path)?;
        fs::remove_file(&tar_path)?; // Clean up temp file

        // Simple multipart form - just like the original
        let form = reqwest::blocking::multipart::Form::new()
            .text("name", name.clone())
            .text("version", version.clone())
            .text("description", description.unwrap_or_default())
            .text(
                "license",
                license.unwrap_or_else(|| "Apache-2.0".to_string()),
            )
            .part(
                "package",
                reqwest::blocking::multipart::Part::bytes(package_data)
                    .file_name(format!("{}-{}.tar.gz", safe_name, version)),
            );

        // Upload to registry with API key authentication
        let response = self
            .client
            .post(format!("{}/api/packages/upload", self.base_url))
            .header("Authorization", format!("Bearer {}", api_key))
            .multipart(form)
            .send()?;

        if !response.status().is_success() {
            let status = response.status();
            let error_text = response
                .text()
                .unwrap_or_else(|_| "Unknown error".to_string());

            if status == reqwest::StatusCode::UNAUTHORIZED {
                println!("\n Authentication failed!");
                println!("\nYour API key may be invalid or expired.");
                println!("\nTo fix this:");
                println!("  1. Run: horus auth login");
                println!("  2. Get a new API key from the registry");
                println!("  3. Try publishing again");
                return Err(anyhow!("Unauthorized - invalid or expired API key"));
            }

            return Err(anyhow!("Failed to publish: {} - {}", status, error_text));
        }

        // Parse the response to check verification status
        let response_text = response.text().unwrap_or_default();
        let response_json: serde_json::Value =
            serde_json::from_str(&response_text).unwrap_or(serde_json::json!({}));

        // Check if verification failed
        if response_json.get("success") == Some(&serde_json::json!(false)) {
            let error_type = response_json
                .get("error")
                .and_then(|v| v.as_str())
                .unwrap_or("unknown");
            let message = response_json
                .get("message")
                .and_then(|v| v.as_str())
                .unwrap_or("Package verification failed");

            println!("\n{} Package verification failed!", "Error:".red());
            println!("   {}", message);

            if error_type == "verification_failed" {
                println!(
                    "\n{}",
                    "The package failed pre-upload verification.".yellow()
                );
                println!("{}", "Please fix the issues above and try again.".yellow());

                // Show warnings if any
                if let Some(warnings) = response_json.get("warnings").and_then(|v| v.as_array()) {
                    if !warnings.is_empty() {
                        println!("\n{}", "Warnings:".yellow());
                        for warning in warnings {
                            if let Some(w) = warning.as_str() {
                                println!("   - {}", w);
                            }
                        }
                    }
                }
            }

            return Err(anyhow!("Package verification failed: {}", message));
        }

        println!(" Published {} v{} successfully!", name, version);

        // Show verification status if available
        if let Some(verification) = response_json.get("verification") {
            let status = verification
                .get("status")
                .and_then(|v| v.as_str())
                .unwrap_or("unknown");
            let job_id = verification.get("job_id").and_then(|v| v.as_str());

            match status {
                "pending" => {
                    println!("\n{} Build verification in progress...", "".cyan());
                    if let Some(id) = job_id {
                        println!(
                            "   Track status: horus pkg status {} v{} --verification",
                            name, version
                        );
                        println!("   Job ID: {}", id);
                    }
                    println!(
                        "\n{}",
                        "Your package will be fully available once verification completes.".cyan()
                    );
                }
                "passed" => {
                    println!("\n{} Build verification passed!", "".green());
                }
                "failed" => {
                    println!("\n{} Build verification failed!", "".red());
                    if let Some(msg) = verification.get("message").and_then(|v| v.as_str()) {
                        println!("   {}", msg);
                    }
                }
                _ => {}
            }
        }
        let encoded_name = url_encode_package_name(&name);
        println!("   View at: {}/packages/{}", self.base_url, encoded_name);

        // Interactive prompts for documentation and source (optional metadata)
        println!("\n{}", "[#] Package Metadata (optional)".cyan().bold());
        println!("   Help users discover and use your package by adding:");

        let (docs_url, docs_type, source_url, categories, package_type) =
            prompt_package_metadata(current_dir)?;

        // If user provided docs, source, categories, or package_type, update the package
        if !docs_url.is_empty()
            || !source_url.is_empty()
            || !categories.is_empty()
            || !package_type.is_empty()
        {
            println!("\n{} Updating package metadata...", "".cyan());
            self.update_package_metadata(
                &name,
                &version,
                &docs_url,
                &docs_type,
                &source_url,
                &categories,
                &package_type,
                &api_key,
            )?;
            println!(" Package metadata updated!");
        }

        Ok(())
    }

    // Update package metadata (docs/source URLs, categories, and package_type)
    #[allow(clippy::too_many_arguments)]
    fn update_package_metadata(
        &self,
        name: &str,
        version: &str,
        docs_url: &str,
        docs_type: &str,
        source_url: &str,
        categories: &str,
        package_type: &str,
        api_key: &str,
    ) -> Result<()> {
        let mut form = reqwest::blocking::multipart::Form::new()
            .text("docs_url", docs_url.to_string())
            .text("docs_type", docs_type.to_string())
            .text("source_url", source_url.to_string());

        // Add categories if provided
        if !categories.is_empty() {
            form = form.text("categories", categories.to_string());
        }

        // Add package_type if provided
        if !package_type.is_empty() {
            form = form.text("package_type", package_type.to_string());
        }

        let response = self
            .client
            .post(format!(
                "{}/api/packages/{}/{}/metadata",
                self.base_url, name, version
            ))
            .header("Authorization", format!("Bearer {}", api_key))
            .multipart(form)
            .send()?;

        if !response.status().is_success() {
            return Err(anyhow!("Failed to update package metadata"));
        }

        Ok(())
    }

    // Unpublish a package from registry
    pub fn unpublish(&self, package_name: &str, version: &str) -> Result<()> {
        // Get API key
        let api_key = match get_api_key() {
            Ok(key) => key,
            Err(_) => {
                println!("\n Not authenticated with HORUS registry.");
                println!("\nTo unpublish packages, you need to authenticate:");
                println!("  1. Run: horus auth login");
                println!("  2. Authorize in your browser");
                println!("  3. The registry will show your API key");
                println!("  4. Save it to ~/.horus/auth.json");
                return Err(anyhow!("Authentication required"));
            }
        };

        // Call DELETE endpoint
        let url = format!(
            "{}/api/packages/{}/{}",
            self.base_url, package_name, version
        );
        let response = self
            .client
            .delete(&url)
            .header("Authorization", format!("Bearer {}", api_key))
            .send()?;

        if !response.status().is_success() {
            let status = response.status();
            let error_text = response
                .text()
                .unwrap_or_else(|_| "Unknown error".to_string());

            if status == reqwest::StatusCode::UNAUTHORIZED {
                return Err(anyhow!(
                    "Authentication failed - invalid or expired API key"
                ));
            } else if status == reqwest::StatusCode::FORBIDDEN {
                return Err(anyhow!("You do not have permission to unpublish this package. Only the package owner can unpublish it."));
            } else if status == reqwest::StatusCode::NOT_FOUND {
                return Err(anyhow!(
                    "Package {} v{} not found in registry",
                    package_name,
                    version
                ));
            }

            return Err(anyhow!("Failed to unpublish: {} - {}", status, error_text));
        }

        Ok(())
    }

    // Search for packages
    pub fn search(&self, query: &str) -> Result<Vec<Package>> {
        let url = format!("{}/api/packages/search?q={}", self.base_url, query);

        let response = self.client.get(&url).send()?;

        if !response.status().is_success() {
            return Err(anyhow!("Search failed"));
        }

        // The registry returns a wrapped response with results, total, query, etc.
        #[derive(Deserialize)]
        struct SearchResponse {
            results: Vec<SearchResultItem>,
        }

        #[derive(Deserialize)]
        struct SearchResultItem {
            name: String,
            version: Option<String>,
            metadata: Option<SearchMetadata>,
        }

        #[derive(Deserialize)]
        struct SearchMetadata {
            description: Option<String>,
        }

        let search_response: SearchResponse = response.json()?;

        // Convert to Package structs
        let packages = search_response
            .results
            .into_iter()
            .map(|item| Package {
                name: item.name,
                version: item.version.unwrap_or_else(|| "latest".to_string()),
                description: item.metadata.and_then(|m| m.description),
            })
            .collect();

        Ok(packages)
    }

    // Resolve an import name to a package name via registry
    pub fn resolve_import(&self, import_name: &str, language: &str) -> Result<Option<String>> {
        let url = format!(
            "{}/api/imports/resolve?import={}&language={}",
            self.base_url, import_name, language
        );

        let response = self.client.get(&url).send()?;

        if !response.status().is_success() {
            return Ok(None);
        }

        #[derive(Deserialize)]
        struct ResolveResult {
            package_name: String,
        }

        let result: Option<ResolveResult> = response.json()?;
        Ok(result.map(|r| r.package_name))
    }

    // Freeze current environment to a manifest
    pub fn freeze(&self) -> Result<EnvironmentManifest> {
        // Scan .horus/packages/ directory for installed packages
        let packages_dir = PathBuf::from(".horus/packages");
        let mut locked_packages = Vec::new();

        if packages_dir.exists() {
            for entry in fs::read_dir(&packages_dir)? {
                let entry = entry?;
                let entry_path = entry.path();

                // Check for path package metadata (*.path.json)
                if entry_path.extension().and_then(|s| s.to_str()) == Some("json")
                    && entry_path.to_string_lossy().contains(".path.")
                {
                    let content = fs::read_to_string(&entry_path)?;
                    let metadata: serde_json::Value = serde_json::from_str(&content)?;

                    let name = metadata["name"].as_str().unwrap_or("unknown").to_string();
                    let version = metadata["version"].as_str().unwrap_or("dev").to_string();
                    let path = metadata["source_path"].as_str().unwrap_or("").to_string();

                    locked_packages.push(LockedPackage {
                        name,
                        version,
                        checksum: String::new(), // Path deps don't have checksums
                        source: PackageSource::Path { path },
                    });
                    continue;
                }

                // Check for system package references (*.system.json)
                if entry_path.extension().and_then(|s| s.to_str()) == Some("json")
                    && entry_path.to_string_lossy().contains(".system.")
                {
                    // This is a system package reference
                    let content = fs::read_to_string(&entry_path)?;
                    let metadata: serde_json::Value = serde_json::from_str(&content)?;

                    let name = metadata["name"].as_str().unwrap_or("unknown").to_string();
                    let version = metadata["version"]
                        .as_str()
                        .unwrap_or("unknown")
                        .to_string();

                    locked_packages.push(LockedPackage {
                        name,
                        version,
                        checksum: String::new(),
                        source: PackageSource::System,
                    });
                    continue;
                }

                // Check if it's a symlink or directory
                let is_package = entry.file_type()?.is_dir() || entry.file_type()?.is_symlink();

                if is_package {
                    let package_name = entry.file_name().to_string_lossy().to_string();

                    // Resolve symlink to actual path if needed
                    let actual_path = if entry_path.is_symlink() {
                        entry_path.read_link().unwrap_or(entry_path.clone())
                    } else {
                        entry_path.clone()
                    };

                    // Try to read package metadata
                    let metadata_path = actual_path.join("metadata.json");
                    if metadata_path.exists() {
                        let content = fs::read_to_string(&metadata_path)?;
                        let metadata_value: serde_json::Value = serde_json::from_str(&content)?;

                        let name = metadata_value["name"]
                            .as_str()
                            .unwrap_or(&package_name)
                            .to_string();
                        let version = metadata_value["version"]
                            .as_str()
                            .unwrap_or("unknown")
                            .to_string();
                        let checksum = metadata_value["checksum"]
                            .as_str()
                            .unwrap_or("")
                            .to_string();
                        let source_str = metadata_value["source"].as_str().unwrap_or("Registry");

                        // Determine package source from metadata or path
                        let source = if source_str == "PyPI"
                            || actual_path.to_string_lossy().contains("pypi_")
                        {
                            PackageSource::PyPI
                        } else {
                            PackageSource::Registry
                        };

                        locked_packages.push(LockedPackage {
                            name,
                            version,
                            checksum,
                            source,
                        });
                    } else {
                        // Fallback: Determine source from path
                        let source = if actual_path.to_string_lossy().contains("pypi_") {
                            PackageSource::PyPI
                        } else {
                            PackageSource::Registry
                        };

                        let version = detect_package_version(&actual_path)
                            .unwrap_or_else(|| "unknown".to_string());

                        locked_packages.push(LockedPackage {
                            name: package_name.clone(),
                            version,
                            checksum: String::new(),
                            source,
                        });
                    }
                }
            }
        }

        // Get system information
        let system_info = SystemInfo {
            os: std::env::consts::OS.to_string(),
            arch: std::env::consts::ARCH.to_string(),
            python_version: get_python_version(),
            rust_version: get_rust_version(),
            gcc_version: get_gcc_version(),
            cuda_version: get_cuda_version(),
        };

        // Generate horus_id (hash of all content)
        let mut hasher = Sha256::new();
        for pkg in &locked_packages {
            hasher.update(&pkg.name);
            hasher.update(&pkg.version);
            hasher.update(&pkg.checksum);
        }
        hasher.update(&system_info.os);
        hasher.update(&system_info.arch);
        let horus_id = format!("env-{}", &format!("{:x}", hasher.finalize())[..12]);

        let manifest = EnvironmentManifest {
            horus_id,
            name: None,
            description: Some("Frozen environment manifest".to_string()),
            packages: locked_packages,
            system: system_info,
            created_at: chrono::Utc::now(),
            horus_version: env!("CARGO_PKG_VERSION").to_string(),
        };

        Ok(manifest)
    }

    // Save environment manifest to registry
    pub fn save_environment(&self, manifest: &EnvironmentManifest) -> Result<()> {
        // No auth for now - server doesn't validate yet
        let response = self
            .client
            .post(format!("{}/api/environments", self.base_url))
            .json(manifest)
            .send()?;

        if !response.status().is_success() {
            let error_text = response
                .text()
                .unwrap_or_else(|_| "Unknown error".to_string());
            return Err(anyhow!("Failed to save environment: {}", error_text));
        }

        println!(" Environment saved with ID: {}", manifest.horus_id);
        Ok(())
    }

    // Restore environment from manifest
    pub fn restore_environment(&self, horus_id: &str) -> Result<()> {
        println!(" Restoring environment {}...", horus_id);

        // Fetch environment manifest from registry
        let url = format!("{}/api/environments/{}", self.base_url, horus_id);
        let response = self.client.get(&url).send()?;

        if !response.status().is_success() {
            return Err(anyhow!("Environment not found: {}", horus_id));
        }

        let manifest: EnvironmentManifest = response.json()?;

        // Install each package
        for package in &manifest.packages {
            println!("  Installing {} v{}...", package.name, package.version);
            self.install(&package.name, Some(&package.version))?;
        }

        println!(" Environment {} restored successfully!", horus_id);
        Ok(())
    }

    pub fn upload_environment(&self, manifest: &EnvironmentManifest) -> Result<()> {
        println!(
            "Publishing environment {} to registry...",
            manifest.horus_id
        );

        // Get API key
        let api_key = get_api_key()?;

        // Upload to registry
        let url = format!("{}/api/environments", self.base_url);
        let response = self
            .client
            .post(&url)
            .header("x-api-key", api_key)
            .json(&serde_json::json!({
                "horus_id": manifest.horus_id,
                "name": manifest.name,
                "description": manifest.description,
                "manifest": manifest
            }))
            .send()?;

        if !response.status().is_success() {
            let error_text = response
                .text()
                .unwrap_or_else(|_| "Unknown error".to_string());
            return Err(anyhow!("Failed to publish environment: {}", error_text));
        }

        println!(" Environment published successfully!");
        println!(
            "   Anyone can now restore with: horus env restore {}",
            manifest.horus_id
        );
        Ok(())
    }
}

// Helper functions for system detection
fn get_python_version() -> Option<String> {
    std::process::Command::new("python3")
        .arg("--version")
        .output()
        .ok()
        .and_then(|output| {
            String::from_utf8(output.stdout)
                .ok()
                .map(|s| s.trim().replace("Python ", ""))
        })
}

fn get_rust_version() -> Option<String> {
    std::process::Command::new("rustc")
        .arg("--version")
        .output()
        .ok()
        .and_then(|output| {
            String::from_utf8(output.stdout)
                .ok()
                .and_then(|s| s.split_whitespace().nth(1).map(|v| v.to_string()))
        })
}

fn get_gcc_version() -> Option<String> {
    std::process::Command::new("gcc")
        .arg("--version")
        .output()
        .ok()
        .and_then(|output| {
            String::from_utf8(output.stdout).ok().and_then(|s| {
                s.lines()
                    .next()
                    .and_then(|line| line.split_whitespace().last())
                    .map(|v| v.to_string())
            })
        })
}

fn get_cuda_version() -> Option<String> {
    // Try nvcc first (CUDA compiler)
    if let Some(version) = std::process::Command::new("nvcc")
        .arg("--version")
        .output()
        .ok()
        .and_then(|output| {
            String::from_utf8(output.stdout).ok().and_then(|s| {
                // Parse output like: "Cuda compilation tools, release 11.8, V11.8.89"
                // Look for "release X.Y" pattern
                s.lines()
                    .find(|line| line.contains("release"))
                    .and_then(|line| {
                        line.split("release")
                            .nth(1)
                            .and_then(|part| part.split(',').next())
                            .map(|v| v.trim().to_string())
                    })
            })
        })
    {
        return Some(version);
    }

    // Fallback to nvidia-smi
    std::process::Command::new("nvidia-smi")
        .output()
        .ok()
        .and_then(|output| {
            String::from_utf8(output.stdout).ok().and_then(|s| {
                // Parse output to find CUDA Version line
                // Format: "CUDA Version: 12.0"
                s.lines()
                    .find(|line| line.contains("CUDA Version"))
                    .and_then(|line| {
                        line.split("CUDA Version:")
                            .nth(1)
                            .map(|v| v.split_whitespace().next().unwrap_or("").to_string())
                    })
            })
        })
}

// Check if any version of a package exists in global cache
fn check_global_versions(cache_dir: &Path, package_name: &str) -> Result<bool> {
    if !cache_dir.exists() {
        return Ok(false);
    }

    for entry in fs::read_dir(cache_dir)? {
        let entry = entry?;
        let name = entry.file_name();
        let name_str = name.to_string_lossy();

        // Match package@version pattern
        if name_str == package_name || name_str.starts_with(&format!("{}@", package_name)) {
            return Ok(true);
        }
    }

    Ok(false)
}

/// Add required features to the project's Cargo.toml for horus_library
fn add_features_to_cargo_toml(workspace_path: &Path, features: &[String]) -> Result<()> {
    use toml_edit::{DocumentMut, Item, Value};

    let cargo_toml_path = workspace_path.join("Cargo.toml");
    if !cargo_toml_path.exists() {
        return Err(anyhow!("Cargo.toml not found in workspace"));
    }

    let content = fs::read_to_string(&cargo_toml_path)?;
    let mut doc = content
        .parse::<DocumentMut>()
        .map_err(|e| anyhow!("Failed to parse Cargo.toml: {}", e))?;

    // Look for horus_library in dependencies
    let deps = doc
        .get_mut("dependencies")
        .ok_or_else(|| anyhow!("No [dependencies] section"))?;

    if let Some(horus_lib) = deps.get_mut("horus_library") {
        // Get existing features or create empty array
        match horus_lib {
            Item::Value(Value::String(_)) => {
                // Convert simple version string to table with features
                let version = horus_lib.as_str().unwrap_or("*").to_string();
                let mut table = toml_edit::InlineTable::new();
                table.insert("version", version.into());
                let mut features_array = toml_edit::Array::new();
                for f in features {
                    features_array.push(f.as_str());
                }
                table.insert("features", Value::Array(features_array));
                *horus_lib = Item::Value(Value::InlineTable(table));
            }
            Item::Value(Value::InlineTable(table)) => {
                // Add features to existing inline table
                let existing_features = table
                    .get("features")
                    .and_then(|v| v.as_array())
                    .map(|a| {
                        a.iter()
                            .filter_map(|v| v.as_str().map(String::from))
                            .collect::<Vec<_>>()
                    })
                    .unwrap_or_default();

                let mut new_features = toml_edit::Array::new();
                for f in &existing_features {
                    new_features.push(f.as_str());
                }
                for f in features {
                    if !existing_features.contains(f) {
                        new_features.push(f.as_str());
                    }
                }
                table.insert("features", Value::Array(new_features));
            }
            Item::Table(table) => {
                // Add features to existing table
                let existing_features = table
                    .get("features")
                    .and_then(|v| v.as_array())
                    .map(|a| {
                        a.iter()
                            .filter_map(|v| v.as_str().map(String::from))
                            .collect::<Vec<_>>()
                    })
                    .unwrap_or_default();

                let mut new_features = toml_edit::Array::new();
                for f in &existing_features {
                    new_features.push(f.as_str());
                }
                for f in features {
                    if !existing_features.contains(f) {
                        new_features.push(f.as_str());
                    }
                }
                table.insert("features", toml_edit::value(Value::Array(new_features)));
            }
            _ => {
                return Err(anyhow!("Unexpected horus_library format in Cargo.toml"));
            }
        }
    } else if let Some(horus) = deps.get_mut("horus") {
        // Also check for "horus" dependency (the unified crate)
        // Similar logic as above
        match horus {
            Item::Value(Value::String(_)) => {
                let version = horus.as_str().unwrap_or("*").to_string();
                let mut table = toml_edit::InlineTable::new();
                table.insert("version", version.into());
                let mut features_array = toml_edit::Array::new();
                for f in features {
                    features_array.push(f.as_str());
                }
                table.insert("features", Value::Array(features_array));
                *horus = Item::Value(Value::InlineTable(table));
            }
            Item::Value(Value::InlineTable(table)) => {
                let existing_features = table
                    .get("features")
                    .and_then(|v| v.as_array())
                    .map(|a| {
                        a.iter()
                            .filter_map(|v| v.as_str().map(String::from))
                            .collect::<Vec<_>>()
                    })
                    .unwrap_or_default();

                let mut new_features = toml_edit::Array::new();
                for f in &existing_features {
                    new_features.push(f.as_str());
                }
                for f in features {
                    if !existing_features.contains(f) {
                        new_features.push(f.as_str());
                    }
                }
                table.insert("features", Value::Array(new_features));
            }
            Item::Table(table) => {
                let existing_features = table
                    .get("features")
                    .and_then(|v| v.as_array())
                    .map(|a| {
                        a.iter()
                            .filter_map(|v| v.as_str().map(String::from))
                            .collect::<Vec<_>>()
                    })
                    .unwrap_or_default();

                let mut new_features = toml_edit::Array::new();
                for f in &existing_features {
                    new_features.push(f.as_str());
                }
                for f in features {
                    if !existing_features.contains(f) {
                        new_features.push(f.as_str());
                    }
                }
                table.insert("features", toml_edit::value(Value::Array(new_features)));
            }
            _ => {}
        }
    } else {
        // No horus or horus_library dependency found - skip
        return Ok(());
    }

    // Write back to file
    fs::write(&cargo_toml_path, doc.to_string())?;
    Ok(())
}

/// Detect the system package manager
fn detect_package_manager() -> Option<(&'static str, &'static str, bool)> {
    // Returns (command, install_subcommand, needs_sudo)
    use std::process::Command;

    // Check for apt (Debian/Ubuntu)
    if Command::new("apt").arg("--version").output().is_ok() {
        return Some(("apt", "install -y", true));
    }
    // Check for dnf (Fedora)
    if Command::new("dnf").arg("--version").output().is_ok() {
        return Some(("dnf", "install -y", true));
    }
    // Check for pacman (Arch)
    if Command::new("pacman").arg("--version").output().is_ok() {
        return Some(("pacman", "-S --noconfirm", true));
    }
    // Check for brew (macOS)
    if Command::new("brew").arg("--version").output().is_ok() {
        return Some(("brew", "install", false));
    }
    // Check for apk (Alpine)
    if Command::new("apk").arg("--version").output().is_ok() {
        return Some(("apk", "add", true));
    }
    None
}

/// Check if a system package is installed
fn is_system_package_installed(pkg: &str, pkg_manager: &str) -> bool {
    use std::process::Command;

    let result = match pkg_manager {
        "apt" => Command::new("dpkg").args(["-s", pkg]).output(),
        "dnf" => Command::new("rpm").args(["-q", pkg]).output(),
        "pacman" => Command::new("pacman").args(["-Q", pkg]).output(),
        "brew" => Command::new("brew").args(["list", pkg]).output(),
        "apk" => Command::new("apk").args(["info", "-e", pkg]).output(),
        _ => return false,
    };

    result.map(|o| o.status.success()).unwrap_or(false)
}

/// Handle system dependencies with smart detection and optional installation
fn handle_system_dependencies(deps: &[String]) -> Result<()> {
    use colored::*;
    use std::io::{self, Write};
    use std::process::Command;

    let Some((pkg_manager, install_cmd, needs_sudo)) = detect_package_manager() else {
        // No known package manager, just notify
        println!("  {} System packages required:", "[PKG]".cyan());
        for dep in deps {
            println!("    • {}", dep);
        }
        println!("    Please install these packages manually");
        return Ok(());
    };

    // Check which packages are missing
    let missing: Vec<&String> = deps
        .iter()
        .filter(|pkg| !is_system_package_installed(pkg, pkg_manager))
        .collect();

    if missing.is_empty() {
        println!("  {} All system packages already installed", "[OK]".green());
        return Ok(());
    }

    println!(
        "  {} System packages required ({}):",
        "[PKG]".cyan(),
        pkg_manager
    );
    for dep in deps {
        let status = if is_system_package_installed(dep, pkg_manager) {
            format!("{}", "[OK]".green())
        } else {
            format!("{}", "missing".yellow())
        };
        println!("    • {} [{}]", dep, status);
    }

    // Build install command
    let install_args: Vec<&str> = install_cmd.split_whitespace().collect();
    let full_cmd = if needs_sudo {
        format!(
            "sudo {} {} {}",
            pkg_manager,
            install_cmd,
            missing
                .iter()
                .map(|s| s.as_str())
                .collect::<Vec<_>>()
                .join(" ")
        )
    } else {
        format!(
            "{} {} {}",
            pkg_manager,
            install_cmd,
            missing
                .iter()
                .map(|s| s.as_str())
                .collect::<Vec<_>>()
                .join(" ")
        )
    };

    // Prompt user
    print!("  {} Install missing packages? [Y/n]: ", "?".cyan());
    io::stdout().flush().ok();

    let mut input = String::new();
    if io::stdin().read_line(&mut input).is_ok() {
        let input = input.trim().to_lowercase();
        if input.is_empty() || input == "y" || input == "yes" {
            println!("  {} Running: {}", "->".cyan(), full_cmd);

            let status = if needs_sudo {
                Command::new("sudo")
                    .arg(pkg_manager)
                    .args(&install_args)
                    .args(missing.iter().map(|s| s.as_str()))
                    .status()
            } else {
                Command::new(pkg_manager)
                    .args(&install_args)
                    .args(missing.iter().map(|s| s.as_str()))
                    .status()
            };

            match status {
                Ok(s) if s.success() => {
                    println!("  {} System packages installed", "[OK]".green());
                }
                Ok(_) => {
                    println!(
                        "  {} Installation failed. Run manually: {}",
                        "[WARN]".yellow(),
                        full_cmd
                    );
                }
                Err(e) => {
                    println!("  {} Could not run installer: {}", "[WARN]".yellow(), e);
                    println!("    Run manually: {}", full_cmd);
                }
            }
        } else {
            println!("  {} Skipped. Run manually: {}", "->".cyan(), full_cmd);
        }
    }

    Ok(())
}

/// Add cargo dependencies to Cargo.toml
/// Dependencies are in format "crate_name@version" e.g., "serialport@4.2"
fn add_cargo_deps_to_cargo_toml(workspace_path: &Path, deps: &[String]) -> Result<()> {
    use toml_edit::DocumentMut;

    let cargo_toml_path = workspace_path.join("Cargo.toml");
    if !cargo_toml_path.exists() {
        return Err(anyhow!("Cargo.toml not found in workspace"));
    }

    let content = fs::read_to_string(&cargo_toml_path)?;
    let mut doc = content
        .parse::<DocumentMut>()
        .map_err(|e| anyhow!("Failed to parse Cargo.toml: {}", e))?;

    // Ensure dependencies section exists
    if doc.get("dependencies").is_none() {
        doc["dependencies"] = toml_edit::table();
    }
    let deps_section = doc
        .get_mut("dependencies")
        .ok_or_else(|| anyhow!("No [dependencies] section"))?;

    for dep_spec in deps {
        // Parse "crate_name@version" or just "crate_name"
        let (name, version) = if let Some((n, v)) = dep_spec.split_once('@') {
            (n.trim(), Some(v.trim()))
        } else {
            (dep_spec.trim(), None)
        };

        // Skip if already present
        if deps_section.get(name).is_some() {
            continue;
        }

        // Add the dependency
        let version_str = version.unwrap_or("*");
        deps_section[name] = toml_edit::value(version_str);
    }

    // Write back to file
    fs::write(&cargo_toml_path, doc.to_string())?;
    Ok(())
}

// Copy directory recursively
fn copy_dir_all(src: &Path, dst: &Path) -> Result<()> {
    fs::create_dir_all(dst)?;

    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let ty = entry.file_type()?;
        let src_path = entry.path();
        let dst_path = dst.join(entry.file_name());

        if ty.is_dir() {
            copy_dir_all(&src_path, &dst_path)?;
        } else {
            fs::copy(&src_path, &dst_path)?;
        }
    }

    Ok(())
}

// Helper function to detect package version from directory
fn detect_package_version(dir: &Path) -> Option<String> {
    // Try horus.yaml first (primary HORUS manifest)
    let horus_yaml = dir.join("horus.yaml");
    if horus_yaml.exists() {
        if let Ok(content) = fs::read_to_string(&horus_yaml) {
            // Simple YAML parsing for version
            for line in content.lines() {
                let trimmed = line.trim();
                if trimmed.starts_with("version:") {
                    let version = trimmed.trim_start_matches("version:").trim().to_string();
                    return Some(version);
                }
            }
        }
    }

    // Try package.json
    let package_json = dir.join("package.json");
    if package_json.exists() {
        if let Ok(content) = fs::read_to_string(&package_json) {
            if let Ok(json) = serde_json::from_str::<serde_json::Value>(&content) {
                if let Some(version) = json.get("version").and_then(|v| v.as_str()) {
                    return Some(version.to_string());
                }
            }
        }
    }

    // Try Cargo.toml
    let cargo_toml = dir.join("Cargo.toml");
    if cargo_toml.exists() {
        if let Ok(content) = fs::read_to_string(&cargo_toml) {
            if let Ok(toml) = toml::from_str::<toml::Value>(&content) {
                if let Some(package) = toml.get("package") {
                    if let Some(version) = package.get("version").and_then(|v| v.as_str()) {
                        return Some(version.to_string());
                    }
                }
            }
        }
    }

    None
}

// Detect installed version from PyPI package .dist-info directory
fn detect_pypi_installed_version(pkg_dir: &Path, package_name: &str) -> Option<String> {
    // PyPI packages create .dist-info directories with the format: package_name-version.dist-info
    // We need to find this directory and extract the version

    if let Ok(entries) = fs::read_dir(pkg_dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                if let Some(dir_name) = path.file_name().and_then(|n| n.to_str()) {
                    // Look for .dist-info directories
                    if dir_name.ends_with(".dist-info") {
                        // Extract package name and version from directory name
                        // Format: package_name-version.dist-info
                        let without_suffix = dir_name.trim_end_matches(".dist-info");

                        // Handle package name normalization (PyPI normalizes names)
                        let normalized_pkg = package_name
                            .replace("-", "_")
                            .replace(".", "_")
                            .to_lowercase();
                        let normalized_dir = without_suffix
                            .replace("-", "_")
                            .replace(".", "_")
                            .to_lowercase();

                        if normalized_dir.starts_with(&normalized_pkg) {
                            // Try to extract version after package name
                            // Format is usually: package_name-version
                            let parts: Vec<&str> = without_suffix.rsplitn(2, '-').collect();
                            if parts.len() == 2 {
                                return Some(parts[0].to_string());
                            }
                        }
                    }
                }
            }
        }
    }

    None
}

// Detect installed version from cargo binary
fn detect_cargo_installed_version(pkg_dir: &Path, package_name: &str) -> Option<String> {
    // Cargo install creates binaries in bin/ subdirectory
    // Try to run the binary with --version flag

    let bin_path = pkg_dir.join("bin").join(package_name);

    if bin_path.exists() {
        use std::process::Command;

        if let Ok(output) = Command::new(&bin_path).arg("--version").output() {
            if output.status.success() {
                let version_output = String::from_utf8_lossy(&output.stdout);
                // Parse version from output (usually: "package_name version")
                let parts: Vec<&str> = version_output.split_whitespace().collect();
                if parts.len() >= 2 {
                    return Some(parts[1].to_string());
                }
            }
        }
    }

    None
}

fn detect_package_info(dir: &Path) -> Result<(String, String, Option<String>, Option<String>)> {
    // HORUS uses horus.yaml as the primary package manifest
    let horus_yaml = dir.join("horus.yaml");

    if !horus_yaml.exists() {
        return Err(anyhow!("No horus.yaml found. This doesn't appear to be a HORUS package.\nRun 'horus new <name>' to create a new package."));
    }

    let content = fs::read_to_string(&horus_yaml)?;

    // Simple YAML parsing for name, version, description, license
    let mut name = String::from("unknown");
    let mut version = String::from("0.1.7");
    let mut description: Option<String> = None;
    let mut license: Option<String> = None;

    for line in content.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with("name:") {
            name = trimmed.trim_start_matches("name:").trim().to_string();
        } else if trimmed.starts_with("version:") {
            version = trimmed.trim_start_matches("version:").trim().to_string();
        } else if trimmed.starts_with("description:") {
            description = Some(
                trimmed
                    .trim_start_matches("description:")
                    .trim()
                    .to_string(),
            );
        } else if trimmed.starts_with("license:") {
            license = Some(trimmed.trim_start_matches("license:").trim().to_string());
        }
    }

    Ok((name, version, description, license))
}

// Extract HORUS dependencies from package metadata
fn extract_package_dependencies(dir: &Path) -> Result<Vec<DependencySpec>> {
    let mut dependencies = Vec::new();

    // Try Cargo.toml
    if dir.join("Cargo.toml").exists() {
        let content = fs::read_to_string(dir.join("Cargo.toml"))?;
        let toml: toml::Value = toml::from_str(&content)?;

        // Extract dependencies from [dependencies] section
        if let Some(deps) = toml.get("dependencies").and_then(|v| v.as_table()) {
            for (dep_name, dep_value) in deps {
                // Only include HORUS packages (start with "horus")
                if dep_name.starts_with("horus") {
                    // Extract version requirement if present
                    let spec_str = if let Some(version) = dep_value.as_str() {
                        format!("{}@{}", dep_name, version)
                    } else if let Some(table) = dep_value.as_table() {
                        if let Some(version) = table.get("version").and_then(|v| v.as_str()) {
                            format!("{}@{}", dep_name, version)
                        } else {
                            dep_name.to_string()
                        }
                    } else {
                        dep_name.to_string()
                    };

                    if let Ok(spec) = DependencySpec::parse(&spec_str) {
                        dependencies.push(spec);
                    }
                }
            }
        }
    }

    // Try package.json
    if dir.join("package.json").exists() {
        let content = fs::read_to_string(dir.join("package.json"))?;
        let json: serde_json::Value = serde_json::from_str(&content)?;

        // Extract dependencies
        if let Some(deps) = json.get("dependencies").and_then(|v| v.as_object()) {
            for (dep_name, dep_value) in deps {
                // Only include HORUS packages
                if dep_name.starts_with("horus") {
                    let spec_str = if let Some(version) = dep_value.as_str() {
                        format!("{}@{}", dep_name, version)
                    } else {
                        dep_name.to_string()
                    };

                    if let Ok(spec) = DependencySpec::parse(&spec_str) {
                        dependencies.push(spec);
                    }
                }
            }
        }
    }

    // Try horus.yaml
    if dir.join("horus.yaml").exists() {
        let content = fs::read_to_string(dir.join("horus.yaml"))?;
        // Simple YAML parsing for dependencies
        for line in content.lines() {
            let trimmed = line.trim();
            if trimmed.starts_with("- ") && !trimmed.contains(':') {
                // Simple list item
                let dep = trimmed[2..].trim();
                if dep.starts_with("horus") {
                    if let Ok(spec) = DependencySpec::parse(dep) {
                        dependencies.push(spec);
                    }
                }
            } else if trimmed.starts_with("dependencies:") {
                // Dependencies section marker, items come next
                continue;
            }
        }
    }

    Ok(dependencies)
}

// Pre-compile Rust packages (Python packages don't need pre-compilation)
fn precompile_package(package_dir: &Path) -> Result<()> {
    use std::process::Command;

    // Detect package language - HORUS supports Rust and Python packages only
    let has_cargo_toml = package_dir.join("Cargo.toml").exists();

    if has_cargo_toml {
        // Rust package - compile with cargo
        println!("  {} Pre-compiling Rust package...", "".cyan());

        let status = Command::new("cargo")
            .arg("build")
            .arg("--release")
            .arg("--lib")
            .current_dir(package_dir)
            .status()?;

        if !status.success() {
            return Err(anyhow!("Cargo build failed"));
        }

        // Copy compiled artifacts to lib/ directory for easy access
        let target_dir = package_dir.join("target/release");
        let lib_dir = package_dir.join("lib");
        fs::create_dir_all(&lib_dir)?;

        // Copy .rlib and .so files
        if target_dir.exists() {
            for entry in fs::read_dir(&target_dir)? {
                let entry = entry?;
                let path = entry.path();
                let name = entry.file_name();
                let name_str = name.to_string_lossy();

                if name_str.ends_with(".rlib")
                    || name_str.ends_with(".so")
                    || name_str.ends_with(".a")
                {
                    let dest = lib_dir.join(&name);
                    fs::copy(&path, &dest)?;
                }
            }

            // Also check deps directory
            let deps_dir = target_dir.join("deps");
            if deps_dir.exists() {
                for entry in fs::read_dir(&deps_dir)? {
                    let entry = entry?;
                    let path = entry.path();
                    let name = entry.file_name();
                    let name_str = name.to_string_lossy();

                    if name_str.ends_with(".rlib")
                        || name_str.ends_with(".so")
                        || name_str.ends_with(".a")
                    {
                        let dest = lib_dir.join(&name);
                        fs::copy(&path, &dest)?;
                    }
                }
            }
        }

        println!("  {} Rust package pre-compiled", "".green());
    } else {
        // Not a compiled package (Python packages don't need pre-compilation)
        return Err(anyhow!("Not a compiled package"));
    }

    Ok(())
}

// Get API key from ~/.horus/auth.json
fn get_api_key() -> Result<String> {
    let home = dirs::home_dir().ok_or_else(|| anyhow!("Could not find home directory"))?;
    let auth_file = home.join(".horus/auth.json");

    if !auth_file.exists() {
        return Err(anyhow!("Not authenticated. Please run: horus auth login"));
    }

    let content = fs::read_to_string(&auth_file)?;
    let auth: serde_json::Value = serde_json::from_str(&content)?;

    let api_key = auth
        .get("api_key")
        .and_then(|v| v.as_str())
        .ok_or_else(|| anyhow!("API key not found in auth.json"))?;

    Ok(api_key.to_string())
}

// Interactive prompts for package documentation, source URLs, categories, and package type
fn prompt_package_metadata(dir: &Path) -> Result<(String, String, String, String, String)> {
    use std::io::{self, Write};

    let mut docs_url = String::new();
    let mut docs_type = String::new();
    let mut source_url = String::new();
    let mut categories = String::new();
    let mut package_type = String::new();

    // Check if /docs folder exists with .md files
    let docs_dir = dir.join("docs");
    let has_local_docs = docs_dir.exists() && docs_dir.is_dir() && {
        fs::read_dir(&docs_dir)
            .map(|entries| {
                entries
                    .filter_map(Result::ok)
                    .any(|e| e.path().extension().and_then(|s| s.to_str()) == Some("md"))
            })
            .unwrap_or(false)
    };

    // Try to auto-detect Git remote URL
    let git_config_path = dir.join(".git/config");
    let detected_git_url = if git_config_path.exists() {
        fs::read_to_string(&git_config_path)
            .ok()
            .and_then(|content| {
                // Extract URL from git config
                for line in content.lines() {
                    let trimmed = line.trim();
                    if trimmed.starts_with("url = ") {
                        let url = trimmed.trim_start_matches("url = ");
                        // Convert git@github.com:user/repo.git to https://github.com/user/repo
                        if url.starts_with("git@github.com:") {
                            let repo = url
                                .trim_start_matches("git@github.com:")
                                .trim_end_matches(".git");
                            return Some(format!("https://github.com/{}", repo));
                        } else if url.starts_with("https://") {
                            return Some(url.trim_end_matches(".git").to_string());
                        }
                    }
                }
                None
            })
    } else {
        None
    };

    // 1. Documentation prompt
    println!("\n{}", "Documentation".cyan().bold());
    if has_local_docs {
        println!(
            "   {} Found local /docs folder with markdown files",
            "".green()
        );
    }
    print!("   Add documentation? (y/n): ");
    io::stdout().flush()?;

    let mut add_docs = String::new();
    io::stdin().read_line(&mut add_docs)?;

    if add_docs.trim().to_lowercase() == "y" {
        println!("\n   Documentation options:");
        println!(
            "     {} External URL - Link to online documentation (e.g., https://docs.example.com)",
            "1.".cyan()
        );
        println!(
            "     {} Local /docs - Bundle markdown files in a /docs folder",
            "2.".cyan()
        );

        if has_local_docs {
            println!(
                "\n   {} Your /docs folder should contain .md files organized as:",
                "[i]".blue()
            );
            println!("      /docs/README.md          (main documentation)");
            println!("      /docs/getting-started.md (guides)");
            println!("      /docs/api.md             (API reference)");
        } else {
            println!(
                "\n   {} To use local docs, create a /docs folder with .md files:",
                "[i]".blue()
            );
            println!("      • Add README.md as the main page");
            println!("      • Use markdown formatting");
            println!("      • Organize by topic (getting-started.md, api.md, etc.)");
        }

        print!("\n   Choose option (1/2/skip): ");
        io::stdout().flush()?;

        let mut docs_choice = String::new();
        io::stdin().read_line(&mut docs_choice)?;

        match docs_choice.trim() {
            "1" => {
                print!("   Enter documentation URL: ");
                io::stdout().flush()?;
                io::stdin().read_line(&mut docs_url)?;
                docs_url = docs_url.trim().to_string();
                docs_type = "external".to_string();

                if !docs_url.is_empty() {
                    println!("   {} Documentation URL: {}", "".green(), docs_url);
                }
            }
            "2" => {
                if has_local_docs {
                    docs_url = "docs/".to_string();
                    docs_type = "local".to_string();
                    println!(
                        "   {} Will bundle local /docs folder with package",
                        "".green()
                    );
                } else {
                    println!(
                        "   {} No /docs folder found. Please create one with .md files first.",
                        "".yellow()
                    );
                }
            }
            _ => {
                println!("   {} Skipping documentation", "".dimmed());
            }
        }
    }

    // 2. Source repository prompt
    println!("\n{}", "Source Repository".cyan().bold());
    if let Some(ref git_url) = detected_git_url {
        println!("   {} Auto-detected: {}", "".green(), git_url);
    }
    print!("   Add source repository? (y/n): ");
    io::stdout().flush()?;

    let mut add_source = String::new();
    io::stdin().read_line(&mut add_source)?;

    if add_source.trim().to_lowercase() == "y" {
        if let Some(git_url) = detected_git_url {
            print!("   Use detected URL? (y/n): ");
            io::stdout().flush()?;

            let mut use_detected = String::new();
            io::stdin().read_line(&mut use_detected)?;

            if use_detected.trim().to_lowercase() == "y" {
                source_url = git_url;
                println!("   {} Source repository: {}", "".green(), source_url);
            } else {
                print!("   Enter source repository URL (e.g., https://github.com/user/repo): ");
                io::stdout().flush()?;
                io::stdin().read_line(&mut source_url)?;
                source_url = source_url.trim().to_string();

                if !source_url.is_empty() {
                    println!("   {} Source repository: {}", "".green(), source_url);
                }
            }
        } else {
            println!(
                "   {} Enter the URL where your code is hosted:",
                "[i]".blue()
            );
            println!("      • GitHub: https://github.com/username/repo");
            println!("      • GitLab: https://gitlab.com/username/repo");
            println!("      • Other: Any public repository URL");
            print!("\n   Enter source repository URL: ");
            io::stdout().flush()?;
            io::stdin().read_line(&mut source_url)?;
            source_url = source_url.trim().to_string();

            if !source_url.is_empty() {
                println!("   {} Source repository: {}", "".green(), source_url);
            }
        }
    }

    // 3. Categories prompt
    println!("\n{}", "Categories".cyan().bold());
    println!(
        "   {} Help users discover your package by selecting relevant categories",
        "[i]".blue()
    );
    println!("   Available categories:");
    println!(
        "     {} Navigation    - Path planning, localization, mapping",
        "1.".cyan()
    );
    println!(
        "     {} Vision        - Computer vision, image processing",
        "2.".cyan()
    );
    println!(
        "     {} Perception    - Sensor fusion, object detection",
        "3.".cyan()
    );
    println!(
        "     {} Control       - Motion control, PID, dynamics",
        "4.".cyan()
    );
    println!(
        "     {} App           - Complete applications, demos",
        "5.".cyan()
    );
    println!(
        "     {} Manipulation  - Arm control, grasping, kinematics",
        "6.".cyan()
    );
    println!(
        "     {} Simulation    - Simulators, testing tools",
        "7.".cyan()
    );
    println!(
        "     {} Utilities     - Tools, helpers, common functions",
        "8.".cyan()
    );
    print!("\n   Select categories (comma-separated numbers, e.g., 1,3,5) or skip: ");
    io::stdout().flush()?;

    let mut category_input = String::new();
    io::stdin().read_line(&mut category_input)?;
    let category_input = category_input.trim();

    if !category_input.is_empty() {
        let category_map = [
            "Navigation",
            "Vision",
            "Perception",
            "Control",
            "App",
            "Manipulation",
            "Simulation",
            "Utilities",
        ];

        let selected: Vec<&str> = category_input
            .split(',')
            .filter_map(|s| {
                let num = s.trim().parse::<usize>().ok()?;
                if num > 0 && num <= category_map.len() {
                    Some(category_map[num - 1])
                } else {
                    None
                }
            })
            .collect();

        if !selected.is_empty() {
            categories = selected.join(",");
            println!(
                "   {} Selected categories: {}",
                "".green(),
                selected.join(", ")
            );
        }
    }

    // 4. Package Type prompt
    println!("\n{}", "Package Type".cyan().bold());
    println!("   {} What type of package is this?", "[i]".blue());
    println!("   Available types:");
    println!(
        "     {} node       - HORUS node that processes data (pub/sub)",
        "1.".cyan()
    );
    println!(
        "     {} driver     - Hardware driver (sensors, actuators)",
        "2.".cyan()
    );
    println!("     {} tool       - CLI tool or utility", "3.".cyan());
    println!(
        "     {} algorithm  - Reusable algorithm implementation",
        "4.".cyan()
    );
    println!(
        "     {} model      - AI/ML model (ONNX, TensorFlow, etc.)",
        "5.".cyan()
    );
    println!("     {} message    - Message type definitions", "6.".cyan());
    println!(
        "     {} app        - Complete multi-node application",
        "7.".cyan()
    );
    print!("\n   Select package type (1-7) or skip for default [node]: ");
    io::stdout().flush()?;

    let mut type_input = String::new();
    io::stdin().read_line(&mut type_input)?;
    let type_input = type_input.trim();

    if !type_input.is_empty() {
        let type_map = [
            "node",
            "driver",
            "tool",
            "algorithm",
            "model",
            "message",
            "app",
        ];

        if let Ok(num) = type_input.parse::<usize>() {
            if num > 0 && num <= type_map.len() {
                package_type = type_map[num - 1].to_string();
                println!("   {} Package type: {}", "".green(), package_type);
            }
        }
    }

    // Default to "node" if not specified
    if package_type.is_empty() {
        package_type = "node".to_string();
        println!("   {} Using default package type: node", "".blue());
    }

    Ok((docs_url, docs_type, source_url, categories, package_type))
}

// Implement PackageProvider trait for RegistryClient to enable dependency resolution
impl PackageProvider for RegistryClient {
    fn get_available_versions(&self, package: &str) -> Result<Vec<Version>> {
        // Query registry for available versions
        let url = format!("{}/api/packages/{}/versions", self.base_url, package);

        let response = self.client.get(&url).send();

        match response {
            Ok(resp) if resp.status().is_success() => {
                #[derive(Deserialize)]
                struct VersionsResponse {
                    versions: Vec<String>,
                }

                let versions_resp: VersionsResponse =
                    resp.json().unwrap_or(VersionsResponse { versions: vec![] });

                // Parse version strings to semver::Version
                let mut versions: Vec<Version> = versions_resp
                    .versions
                    .iter()
                    .filter_map(|v| Version::parse(v).ok())
                    .collect();

                versions.sort();

                // If registry has versions, return them
                // If empty, fall back to local cache (for built-in packages like "horus")
                if !versions.is_empty() {
                    return Ok(versions);
                }

                // Fall through to cache check below
            }
            _ => {}
        }

        // Fallback: check local/global cache for versions
        let home = dirs::home_dir().ok_or_else(|| anyhow!("Could not find home directory"))?;
        let global_cache = home.join(".horus/cache");
        let local_packages = PathBuf::from(".horus/packages");

        let mut versions = Vec::new();

        // Check global cache
        if let Ok(entries) = fs::read_dir(&global_cache) {
            for entry in entries.flatten() {
                let name = entry.file_name();
                let name_str = name.to_string_lossy();

                // Match "package@version" pattern
                if name_str.starts_with(&format!("{}@", package)) {
                    if let Some(version_str) = name_str.split('@').nth(1) {
                        if let Ok(version) = Version::parse(version_str) {
                            versions.push(version);
                        }
                    }
                }
            }
        }

        // Check local packages
        if let Ok(entries) = fs::read_dir(&local_packages) {
            for entry in entries.flatten() {
                let name = entry.file_name();
                let name_str = name.to_string_lossy();

                if name_str == package {
                    // Read version from metadata
                    if let Some(version) = detect_package_version(&entry.path()) {
                        if let Ok(v) = Version::parse(&version) {
                            versions.push(v);
                        }
                    }
                }
            }
        }

        versions.sort();
        versions.dedup();

        if versions.is_empty() {
            Err(anyhow!("No versions found for package: {}", package))
        } else {
            Ok(versions)
        }
    }

    fn get_dependencies(&self, package: &str, version: &Version) -> Result<Vec<DependencySpec>> {
        // Query registry for package dependencies
        let url = format!(
            "{}/api/packages/{}/{}/metadata",
            self.base_url, package, version
        );

        let response = self.client.get(&url).send();

        match response {
            Ok(resp) if resp.status().is_success() => {
                #[derive(Deserialize)]
                struct MetadataResponse {
                    dependencies: Option<Vec<DependencyInfo>>,
                }

                #[derive(Deserialize)]
                struct DependencyInfo {
                    name: String,
                    version_req: Option<String>,
                }

                let metadata: MetadataResponse = resp
                    .json()
                    .unwrap_or(MetadataResponse { dependencies: None });

                let deps: Vec<DependencySpec> = metadata
                    .dependencies
                    .unwrap_or_default()
                    .into_iter()
                    .filter_map(|dep| {
                        let spec_str = if let Some(req) = dep.version_req {
                            format!("{}@{}", dep.name, req)
                        } else {
                            dep.name
                        };
                        DependencySpec::parse(&spec_str).ok()
                    })
                    .collect();

                Ok(deps)
            }
            _ => {
                // Fallback: read from local cache
                let home =
                    dirs::home_dir().ok_or_else(|| anyhow!("Could not find home directory"))?;
                let global_cache = home.join(".horus/cache");
                let package_dir_name = format!("{}@{}", package, version);
                let package_dir = global_cache.join(&package_dir_name);

                if package_dir.exists() {
                    extract_package_dependencies(&package_dir)
                } else {
                    // Check local
                    let local_packages = PathBuf::from(".horus/packages");
                    let local_dir = local_packages.join(package);

                    if local_dir.exists() {
                        extract_package_dependencies(&local_dir)
                    } else {
                        Ok(vec![]) // No dependencies
                    }
                }
            }
        }
    }
}

// Additional methods for path and git dependencies
impl RegistryClient {
    // Install a package from local filesystem path
    // Returns the detected version
    pub fn install_from_path(
        &self,
        package_name: &str,
        path: &Path,
        target: crate::workspace::InstallTarget,
        base_dir: Option<&Path>,
    ) -> Result<String> {
        use crate::workspace::InstallTarget;

        println!(
            " Installing {} from path: {}...",
            package_name,
            path.display()
        );

        // Resolve relative path to absolute
        let source_path = if path.is_absolute() {
            path.to_path_buf()
        } else {
            // Resolve relative to base_dir (horus.yaml location) or current directory
            let base = base_dir
                .map(|p| p.to_path_buf())
                .or_else(|| std::env::current_dir().ok())
                .unwrap_or_else(|| PathBuf::from("."));
            base.join(path)
        };

        if !source_path.exists() {
            return Err(anyhow!("Path does not exist: {}", source_path.display()));
        }

        if !source_path.is_dir() {
            return Err(anyhow!(
                "Path is not a directory: {}",
                source_path.display()
            ));
        }

        // Detect version from package manifest
        let version = detect_package_version(&source_path).unwrap_or_else(|| "dev".to_string());

        // Determine packages directory based on target
        let packages_dir = match &target {
            InstallTarget::Global => {
                let current = PathBuf::from(".horus/packages");
                fs::create_dir_all(&current)?;
                current
            }
            InstallTarget::Local(workspace_path) => {
                let local = workspace_path.join(".horus/packages");
                fs::create_dir_all(&local)?;
                local
            }
        };

        let link_path = packages_dir.join(package_name);

        // Remove existing symlink/directory if present
        if link_path.exists() || link_path.symlink_metadata().is_ok() {
            #[cfg(unix)]
            {
                if link_path.symlink_metadata()?.is_symlink() {
                    fs::remove_file(&link_path)?;
                } else {
                    fs::remove_dir_all(&link_path)?;
                }
            }
            #[cfg(windows)]
            {
                if link_path.is_dir() {
                    fs::remove_dir_all(&link_path)?;
                } else {
                    fs::remove_file(&link_path)?;
                }
            }
        }

        // Create symlink to source path
        #[cfg(unix)]
        std::os::unix::fs::symlink(&source_path, &link_path)?;
        #[cfg(windows)]
        std::os::windows::fs::symlink_dir(&source_path, &link_path)?;

        // Create metadata for tracking
        let metadata = serde_json::json!({
            "name": package_name,
            "version": version,
            "source": "Path",
            "source_path": source_path.display().to_string()
        });

        let metadata_file = packages_dir.join(format!("{}.path.json", package_name));
        fs::write(&metadata_file, serde_json::to_string_pretty(&metadata)?)?;

        println!(" Installed {} v{} from path", package_name, version);
        println!(
            "   Link: {} -> {}",
            link_path.display(),
            source_path.display()
        );
        println!(
            "   {} Path dependencies are live-linked - changes take effect immediately",
            "ℹ".cyan()
        );

        Ok(version)
    }
}

// Helper methods for system package detection (not part of PackageProvider trait)
impl RegistryClient {
    // Detect if a Python package exists in system site-packages
    fn detect_system_python_package(&self, package_name: &str) -> Result<Option<String>> {
        use std::process::Command;

        // Try to find package using pip show
        let output = Command::new("python3")
            .args(["-m", "pip", "show", package_name])
            .output();

        if let Ok(output) = output {
            if output.status.success() {
                let stdout = String::from_utf8_lossy(&output.stdout);
                // Parse version from pip show output
                for line in stdout.lines() {
                    if line.starts_with("Version:") {
                        let version = line.trim_start_matches("Version:").trim();
                        return Ok(Some(version.to_string()));
                    }
                }
            }
        }

        // Fallback: check site-packages directly
        let mut site_packages_paths = vec![
            PathBuf::from(format!(
                "/usr/lib/python3.12/site-packages/{}",
                package_name
            )),
            PathBuf::from(format!(
                "/usr/local/lib/python3.12/site-packages/{}",
                package_name
            )),
        ];

        if let Some(home) = dirs::home_dir() {
            site_packages_paths.push(home.join(format!(
                ".local/lib/python3.12/site-packages/{}",
                package_name
            )));
        }

        for path in site_packages_paths {
            if path.exists() {
                // Try to get version from __init__.py or metadata
                let version_file = path.join("__init__.py");
                if version_file.exists() {
                    // Found package, but version unknown
                    return Ok(Some("unknown".to_string()));
                }
            }
        }

        Ok(None)
    }

    // Detect if a Rust binary exists in system cargo bin
    fn detect_system_cargo_binary(&self, package_name: &str) -> Result<Option<String>> {
        use std::process::Command;

        // Check ~/.cargo/bin/
        if let Some(home) = dirs::home_dir() {
            let cargo_bin = home.join(".cargo/bin").join(package_name);
            if cargo_bin.exists() {
                // Try to get version by running --version
                if let Ok(output) = Command::new(&cargo_bin).arg("--version").output() {
                    if output.status.success() {
                        let version_str = String::from_utf8_lossy(&output.stdout);
                        // Parse version (usually "name version")
                        let version = version_str
                            .split_whitespace()
                            .nth(1)
                            .unwrap_or("unknown")
                            .to_string();
                        return Ok(Some(version));
                    }
                }
                // Binary exists but version unknown
                return Ok(Some("unknown".to_string()));
            }
        }

        Ok(None)
    }

    // Prompt user for what to do with system package
    fn prompt_system_package_choice(
        &self,
        package_name: &str,
        system_version: &str,
        source_type: &str, // "PyPI" or "crates.io"
    ) -> Result<SystemPackageChoice> {
        use std::io::{self, Write};

        println!(
            "\n{} {} {} found in system (version: {})",
            "".yellow(),
            source_type,
            package_name.green(),
            system_version.cyan()
        );
        println!("\nWhat would you like to do?");
        println!("  [1] {} Use system package (create reference)", "".green());
        println!(
            "  [2] {} Install to HORUS (isolated environment)",
            "".blue()
        );
        println!("  [3] {} Cancel installation", "[FAIL]".red());

        print!("\nChoice [1-3]: ");
        io::stdout().flush()?;

        let mut input = String::new();
        io::stdin().read_line(&mut input)?;

        match input.trim() {
            "1" => Ok(SystemPackageChoice::UseSystem),
            "2" => Ok(SystemPackageChoice::InstallHORUS),
            "3" => Ok(SystemPackageChoice::Cancel),
            _ => {
                println!("Invalid choice, defaulting to Install to HORUS");
                Ok(SystemPackageChoice::InstallHORUS)
            }
        }
    }

    // Create reference to system Python package
    fn create_system_reference_python(
        &self,
        package_name: &str,
        system_version: &str,
        target: &crate::workspace::InstallTarget,
    ) -> Result<String> {
        use crate::workspace::InstallTarget;
        use std::process::Command;

        println!("  {} Creating reference to system package...", "".green());

        // Find actual system package location
        let output = Command::new("python3")
            .args([
                "-c",
                &format!("import {}; print({}.__file__)", package_name, package_name),
            ])
            .output()?;

        if !output.status.success() {
            return Err(anyhow!("Failed to locate system package"));
        }

        let package_file = String::from_utf8_lossy(&output.stdout).trim().to_string();
        let package_path = PathBuf::from(&package_file)
            .parent()
            .ok_or_else(|| anyhow!("Invalid package path"))?
            .to_path_buf();

        // Create metadata file in .horus/packages/ with system reference
        let packages_dir = match target {
            InstallTarget::Global => {
                let current = PathBuf::from(".horus/packages");
                fs::create_dir_all(&current)?;
                current
            }
            InstallTarget::Local(workspace_path) => {
                let local = workspace_path.join(".horus/packages");
                fs::create_dir_all(&local)?;
                local
            }
        };

        let metadata_file = packages_dir.join(format!("{}.system.json", package_name));
        let metadata = serde_json::json!({
            "name": package_name,
            "version": system_version,
            "source": "System",
            "system_path": package_path.display().to_string(),
            "package_type": "PyPI"
        });

        fs::write(&metadata_file, serde_json::to_string_pretty(&metadata)?)?;

        println!(
            "  {} Using system package at {}",
            "".green(),
            package_path.display()
        );
        println!(
            "  {} Reference created: {}",
            "".cyan(),
            metadata_file.display()
        );

        Ok(system_version.to_string())
    }

    // Create reference to system cargo binary
    fn create_system_reference_cargo(
        &self,
        package_name: &str,
        system_version: &str,
        target: &crate::workspace::InstallTarget,
    ) -> Result<String> {
        use crate::workspace::InstallTarget;

        println!("  {} Creating reference to system binary...", "".green());

        // Find actual system binary location
        let home = dirs::home_dir().ok_or_else(|| anyhow!("Could not find home directory"))?;
        let cargo_bin = home.join(".cargo/bin").join(package_name);

        if !cargo_bin.exists() {
            return Err(anyhow!("System binary not found at expected location"));
        }

        // Create metadata file in .horus/packages/ with system reference
        let packages_dir = match target {
            InstallTarget::Global => {
                let current = PathBuf::from(".horus/packages");
                fs::create_dir_all(&current)?;
                current
            }
            InstallTarget::Local(workspace_path) => {
                let local = workspace_path.join(".horus/packages");
                fs::create_dir_all(&local)?;
                local
            }
        };

        let metadata_file = packages_dir.join(format!("{}.system.json", package_name));
        let metadata = serde_json::json!({
            "name": package_name,
            "version": system_version,
            "source": "System",
            "system_path": cargo_bin.display().to_string(),
            "package_type": "CratesIO"
        });

        fs::write(&metadata_file, serde_json::to_string_pretty(&metadata)?)?;

        // Create symlink in .horus/bin to system binary
        let bin_dir = match target {
            InstallTarget::Global => PathBuf::from(".horus/bin"),
            InstallTarget::Local(workspace_path) => workspace_path.join(".horus/bin"),
        };
        fs::create_dir_all(&bin_dir)?;

        let bin_link = bin_dir.join(package_name);
        if bin_link.exists() {
            fs::remove_file(&bin_link)?;
        }
        #[cfg(unix)]
        std::os::unix::fs::symlink(&cargo_bin, &bin_link)?;
        #[cfg(windows)]
        {
            // On Windows, create a .cmd wrapper for executables
            let cmd_link = bin_dir.join(format!("{}.cmd", package_name));
            let cmd_content = format!("@echo off\n\"{}\" %*\n", cargo_bin.display());
            fs::write(&cmd_link, cmd_content)?;
        }

        println!(
            "  {} Using system binary at {}",
            "".green(),
            cargo_bin.display()
        );
        println!(
            "  {} Reference created: {}",
            "".cyan(),
            metadata_file.display()
        );
        println!("  {} Binary linked: {}", "".cyan(), bin_link.display());

        Ok(system_version.to_string())
    }
}

// ============================================================================
// Public wrapper functions for use from main.rs
// ============================================================================

/// Public wrapper to add features to Cargo.toml (for horus driver add)
pub fn add_features_to_cargo_toml_pub(workspace_path: &Path, features: &[String]) -> Result<()> {
    add_features_to_cargo_toml(workspace_path, features)
}

/// Public wrapper to add cargo dependencies to Cargo.toml (for horus driver add)
pub fn add_cargo_deps_pub(workspace_path: &Path, deps: &[String]) -> Result<()> {
    add_cargo_deps_to_cargo_toml(workspace_path, deps)
}

/// Public wrapper to handle system dependencies (for horus driver add)
pub fn handle_system_deps_pub(deps: &[String]) -> Result<()> {
    handle_system_dependencies(deps)
}

/// Public wrapper to fetch package type from registry
pub fn fetch_package_type(package_name: &str) -> Result<String> {
    let client = RegistryClient::new();
    client.fetch_package_type(package_name)
}

/// Public wrapper to fetch driver metadata from registry
pub fn fetch_driver_metadata(package_name: &str) -> Result<DriverMetadata> {
    let client = RegistryClient::new();
    client.fetch_driver_metadata(package_name)
}
