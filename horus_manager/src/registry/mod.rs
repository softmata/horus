pub mod helpers;
mod install;
mod publish;

#[cfg(test)]
mod tests;

// Re-export all public types and functions
pub use helpers::generate_signing_keypair;

use crate::config::CARGO_TOML;
use crate::dependency_resolver::{DependencySpec, PackageProvider};
use crate::manifest::HORUS_TOML;
use crate::progress::{self, finish_error, finish_success};
use anyhow::{anyhow, bail, Result};
use colored::*;
use flate2::read::GzDecoder;
use flate2::write::GzEncoder;
use flate2::Compression;
use horus_core::core::DurationExt;
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
/// Supports both unscoped ("my-pkg") and scoped ("@org/my-pkg") package names.
pub fn validate_package_name(name: &str) -> Result<()> {
    // Scoped packages: @org/name
    if let Some(rest) = name.strip_prefix('@') {
        // Total length check
        if name.len() < 4 || name.len() > 128 {
            return Err(anyhow!(
                "Scoped package name must be between 4 and 128 characters (got {})",
                name.len()
            ));
        }

        // strip @
        let Some((org, pkg)) = rest.split_once('/') else {
            return Err(anyhow!("Scoped package name must be in @org/name format"));
        };

        // Validate org segment
        if org.len() < 2 || org.len() > 64 {
            return Err(anyhow!(
                "Organization name must be between 2 and 64 characters (got {})",
                org.len()
            ));
        }
        validate_name_segment(org, "Organization name")?;

        // Validate name segment
        if pkg.len() < 2 || pkg.len() > 64 {
            return Err(anyhow!(
                "Package name must be between 2 and 64 characters (got {})",
                pkg.len()
            ));
        }
        validate_name_segment(pkg, "Package name")?;

        // Reserved names on both segments
        if RESERVED_NAMES.contains(&org) {
            return Err(anyhow!("Organization name '{}' is reserved", org));
        }
        if RESERVED_NAMES.contains(&pkg) {
            return Err(anyhow!("Package name '{}' is reserved", pkg));
        }

        return Ok(());
    }

    // Unscoped packages
    if name.len() < 2 || name.len() > 64 {
        return Err(anyhow!(
            "Package name must be between 2 and 64 characters (got {})",
            name.len()
        ));
    }

    validate_name_segment(name, "Package name")?;

    // Path traversal check
    if name.contains("..") || name.contains('\\') {
        return Err(anyhow!("Package name contains invalid path characters"));
    }

    // Reserved names
    if RESERVED_NAMES.contains(&name) {
        return Err(anyhow!("Package name '{}' is reserved", name));
    }

    Ok(())
}

/// Reserved names that cannot be used as package or organization names.
const RESERVED_NAMES: &[&str] = &[
    "horus",
    "core",
    "std",
    "lib",
    "test",
    "main",
    "mod",
    "pub",
    "use",
    "crate",
    "self",
    "super",
    "extern",
    "fn",
    "let",
    "const",
    "static",
    "mut",
    "ref",
    "type",
    "impl",
    "trait",
    "struct",
    "enum",
    "union",
    "admin",
    "api",
    "www",
    "mail",
    "ftp",
    "localhost",
    "root",
    "system",
];

/// Validate a name segment (org or package name part).
/// Must start with lowercase letter, contain only lowercase letters, digits, hyphens, underscores.
fn validate_name_segment(segment: &str, label: &str) -> Result<()> {
    let mut chars = segment.chars();
    if let Some(first) = chars.next() {
        if !first.is_ascii_lowercase() {
            return Err(anyhow!(
                "{} must start with a lowercase letter, got '{}'",
                label,
                first
            ));
        }
    }
    for ch in segment.chars() {
        if !ch.is_ascii_lowercase() && !ch.is_ascii_digit() && ch != '-' && ch != '_' {
            return Err(anyhow!(
                "{} contains invalid character '{}'. Only lowercase letters, digits, hyphens, and underscores are allowed",
                label, ch
            ));
        }
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
    name.replace('@', "").replace('/', "__")
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
            .connect_timeout(30_u64.secs())
            .timeout(120_u64.secs())
            .user_agent("horus-pkg-manager")
            .build()
            .unwrap_or_else(|_| Client::new());

        Self { client, base_url }
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

#[cfg(test)]
mod dispatch_tests {
    use super::*;

    // ── validate_package_name: reserved names ───────────────────────────

    #[test]
    fn all_reserved_names_rejected() {
        for name in RESERVED_NAMES {
            let result = validate_package_name(name);
            assert!(
                result.is_err(),
                "Reserved name '{}' should be rejected",
                name
            );
        }
    }

    #[test]
    fn reserved_names_count() {
        // Ensure we have the expected count of reserved names
        assert!(
            RESERVED_NAMES.len() >= 30,
            "Should have at least 30 reserved names, got {}",
            RESERVED_NAMES.len()
        );
    }

    // ── validate_package_name: edge cases ───────────────────────────────

    #[test]
    fn validate_name_too_short() {
        assert!(validate_package_name("a").is_err());
    }

    #[test]
    fn validate_name_exact_min_length() {
        validate_package_name("ab").unwrap();
    }

    #[test]
    fn validate_name_too_long() {
        let name = "a".repeat(65);
        assert!(validate_package_name(&name).is_err());
    }

    #[test]
    fn validate_name_exact_max_length() {
        let name = "a".repeat(64);
        validate_package_name(&name).unwrap();
    }

    #[test]
    fn validate_name_uppercase_rejected() {
        assert!(validate_package_name("MyPackage").is_err());
    }

    #[test]
    fn validate_name_starts_with_digit_rejected() {
        assert!(validate_package_name("123pkg").is_err());
    }

    #[test]
    fn validate_name_path_traversal_rejected() {
        assert!(validate_package_name("..evil").is_err());
    }

    #[test]
    fn validate_name_backslash_rejected() {
        assert!(validate_package_name("evil\\pkg").is_err());
    }

    #[test]
    fn validate_name_valid_with_hyphens_underscores() {
        validate_package_name("my-cool_package").unwrap();
    }

    #[test]
    fn validate_name_valid_with_digits() {
        validate_package_name("horus-driver-v2").unwrap();
    }

    // ── scoped package names ────────────────────────────────────────────

    #[test]
    fn validate_scoped_name_valid() {
        validate_package_name("@myorg/my-package").unwrap();
    }

    #[test]
    fn validate_scoped_name_reserved_org() {
        assert!(validate_package_name("@horus/my-package").is_err());
    }

    #[test]
    fn validate_scoped_name_reserved_pkg() {
        assert!(validate_package_name("@myorg/core").is_err());
    }

    #[test]
    fn validate_scoped_name_missing_slash() {
        assert!(validate_package_name("@myorg-package").is_err());
    }

    // ── url_encode_package_name ─────────────────────────────────────────

    #[test]
    fn url_encode_simple_name() {
        assert_eq!(url_encode_package_name("my-package"), "my-package");
    }

    #[test]
    fn url_encode_scoped_name() {
        assert_eq!(url_encode_package_name("@org/package"), "%40org%2Fpackage");
    }

    // ── package_name_to_path ────────────────────────────────────────────

    #[test]
    fn path_simple_name() {
        assert_eq!(package_name_to_path("my-package"), "my-package");
    }

    #[test]
    fn path_scoped_name() {
        assert_eq!(package_name_to_path("@org/package"), "org__package");
    }

    // ── PackageSource enum ──────────────────────────────────────────────

    #[test]
    fn package_source_all_variants() {
        let _ = PackageSource::Registry;
        let _ = PackageSource::PyPI;
        let _ = PackageSource::CratesIO;
        let _ = PackageSource::System;
        let _ = PackageSource::Path {
            path: "/tmp".to_string(),
        };
    }

    #[test]
    fn package_source_equality() {
        assert_eq!(PackageSource::Registry, PackageSource::Registry);
        assert_ne!(PackageSource::Registry, PackageSource::PyPI);
    }

    // ── RegistryClient ──────────────────────────────────────────────────

    #[test]
    fn registry_client_default_url() {
        let client = RegistryClient::new();
        assert!(
            !client.base_url().is_empty(),
            "Base URL should not be empty"
        );
    }

    #[test]
    fn registry_client_has_http_client() {
        let client = RegistryClient::new();
        let _ = client.http_client(); // should not panic
    }

    // ── LockedPackage ───────────────────────────────────────────────────

    #[test]
    fn locked_package_construction() {
        let pkg = LockedPackage {
            name: "serde".to_string(),
            version: "1.0.215".to_string(),
            checksum: "sha256:abc".to_string(),
            source: PackageSource::CratesIO,
        };
        assert_eq!(pkg.name, "serde");
        assert_eq!(pkg.version, "1.0.215");
    }

    // ── DriverMetadata ──────────────────────────────────────────────────

    #[test]
    fn driver_metadata_default() {
        let meta = DriverMetadata::default();
        assert!(meta.bus_type.is_none());
        assert!(meta.required_features.is_none());
        assert!(meta.cargo_dependencies.is_none());
        assert!(meta.python_dependencies.is_none());
        assert!(meta.system_dependencies.is_none());
    }
}
