mod helpers;
mod install;
mod publish;

#[cfg(test)]
mod tests;

// Re-export all public types and functions
pub use helpers::generate_signing_keypair;

use crate::config::{CARGO_TOML, HORUS_YAML};
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

/// Response from the driver list API endpoint
#[derive(Debug, Deserialize)]
pub(crate) struct DriverListResponse {
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
}
