mod helpers;
mod install;
mod publish;

#[cfg(test)]
mod tests;

// Re-export all public types and functions
pub use helpers::generate_signing_keypair;

use crate::config::CARGO_TOML;
use crate::manifest::HORUS_TOML;
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

/// Environment list item from registry API (includes full manifest)
#[derive(Debug, Serialize, Deserialize)]
pub struct EnvironmentListItem {
    pub horus_id: String,
    pub name: Option<String>,
    pub description: Option<String>,
    pub manifest: EnvironmentManifest,
    pub created_at: String,
}

/// Response wrapper for environment list
#[derive(Debug, Serialize, Deserialize)]
pub struct EnvironmentListResponse {
    pub environments: Vec<EnvironmentListItem>,
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
    /// Cargo crate dependencies (e.g., `["serialport@4.2", "i2cdev@0.6"]`)
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
pub(crate) struct DriverMetadataResponse {
    #[serde(flatten)]
    pub driver_metadata: Option<DriverMetadata>,
}

/// Response from the driver list API endpoint (GET /api/drivers)
#[derive(Debug, Deserialize)]
pub(crate) struct DriverListResponse {
    pub drivers: Vec<DriverListEntry>,
}

/// Response from the driver search API endpoint (GET /api/drivers/search)
#[derive(Debug, Deserialize)]
pub(crate) struct DriverSearchResponse {
    pub results: Vec<DriverListEntry>,
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

/// Validate a package name against registry rules (mirrors server-side validation).
/// Scoped packages (@org/name) bypass normal validation.
pub fn validate_package_name(name: &str) -> Result<()> {
    // Scoped packages have their own format
    if name.starts_with('@') {
        if !name.contains('/') {
            return Err(anyhow!("Scoped package name must be in @org/name format"));
        }
        return Ok(());
    }

    // Length check
    if name.len() < 2 || name.len() > 64 {
        return Err(anyhow!(
            "Package name must be between 2 and 64 characters (got {})",
            name.len()
        ));
    }

    // Format check: must start with lowercase letter, only lowercase/digits/hyphens/underscores
    let mut chars = name.chars();
    if let Some(first) = chars.next() {
        if !first.is_ascii_lowercase() {
            return Err(anyhow!(
                "Package name must start with a lowercase letter, got '{}'",
                first
            ));
        }
    }
    for ch in name.chars() {
        if !ch.is_ascii_lowercase() && !ch.is_ascii_digit() && ch != '-' && ch != '_' {
            return Err(anyhow!(
                "Package name contains invalid character '{}'. Only lowercase letters, digits, hyphens, and underscores are allowed",
                ch
            ));
        }
    }

    // Path traversal check
    if name.contains("..") || name.contains('/') || name.contains('\\') {
        return Err(anyhow!("Package name contains invalid path characters"));
    }

    // Reserved names
    const RESERVED: &[&str] = &[
        "horus", "core", "std", "lib", "test", "main", "mod", "pub", "use",
        "crate", "self", "super", "extern", "fn", "let", "const", "static",
        "mut", "ref", "type", "impl", "trait", "struct", "enum", "union",
        "admin", "api", "www", "mail", "ftp", "localhost", "root", "system",
    ];
    if RESERVED.contains(&name) {
        return Err(anyhow!("Package name '{}' is reserved", name));
    }

    Ok(())
}

/// URL-encode a package name for API calls
/// Encodes characters unsafe in URL path segments (@ and / for scoped packages)
pub fn url_encode_package_name(name: &str) -> String {
    name.replace('%', "%25")
        .replace('@', "%40")
        .replace('/', "%2F")
        .replace(' ', "%20")
        .replace('#', "%23")
        .replace('?', "%3F")
}

/// Convert a package name to a safe filesystem path component
/// Scoped names like @org/package become org__package
pub fn package_name_to_path(name: &str) -> String {
    name.replace('@', "")
        .replace('/', "__")
}

pub struct RegistryClient {
    pub(crate) client: Client,
    pub(crate) base_url: String,
}

impl Default for RegistryClient {
    fn default() -> Self {
        Self::new()
    }
}

impl RegistryClient {
    pub fn new() -> Self {
        let base_url = crate::config::registry_url();

        let client = Client::builder()
            .connect_timeout(std::time::Duration::from_secs(30))
            .timeout(std::time::Duration::from_secs(120))
            .user_agent("horus-pkg-manager")
            .build()
            .unwrap_or_else(|_| Client::new());

        Self {
            client,
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
}
