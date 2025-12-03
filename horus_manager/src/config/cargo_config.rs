//! Cargo.toml configuration parsing
//! Handles package metadata and HORUS-specific metadata for registry/marketplace

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

/// Root structure of Cargo.toml
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CargoConfig {
    pub package: PackageSection,
    #[serde(default)]
    pub dependencies: toml::Table,
    #[serde(rename = "package.metadata")]
    pub metadata: Option<PackageMetadata>,
}

/// `[package]` section of Cargo.toml
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PackageSection {
    pub name: String,
    pub version: String,
    #[serde(default)]
    pub authors: Option<Vec<String>>,
    pub description: Option<String>,
    pub license: Option<String>,
    pub repository: Option<String>,
    pub homepage: Option<String>,
    pub documentation: Option<String>,
    pub readme: Option<String>,
    pub keywords: Option<Vec<String>>,
    pub categories: Option<Vec<String>>,
    pub edition: Option<String>,
}

/// [package.metadata] section for tool-specific metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PackageMetadata {
    pub horus: Option<HorusMetadata>,
}

/// [package.metadata.horus] - HORUS-specific metadata for registry/marketplace
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HorusMetadata {
    /// Marketplace tags for discovery
    pub tags: Option<Vec<String>>,

    /// Capabilities this package provides
    pub capabilities: Option<Vec<String>>,

    /// Hardware requirements/compatibility
    pub hardware: Option<Vec<String>>,

    /// Minimum HORUS version required
    pub min_horus_version: Option<String>,

    /// Maximum tested HORUS version
    pub max_horus_version: Option<String>,

    /// Whether this package can be published to registry
    #[serde(default = "default_true")]
    pub publishable: bool,

    /// Package tier for marketplace (free, pro, enterprise)
    pub tier: Option<String>,

    /// Preview image URL for marketplace
    pub preview_image: Option<String>,

    /// Video demo URL
    pub demo_video: Option<String>,

    /// Example configurations
    pub examples: Option<Vec<ExampleConfig>>,

    /// Dependencies on other HORUS packages
    pub horus_dependencies: Option<Vec<HorusDependency>>,

    // Runtime configuration (previously in horus.toml)
    pub tick_rate: Option<u32>,
    pub backend: Option<String>,
    pub logging_level: Option<String>,
    pub priority: Option<i32>,
}

fn default_true() -> bool {
    true
}

/// Example configuration for documentation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExampleConfig {
    pub name: String,
    pub description: String,
    pub config_file: Option<String>,
    pub command: Option<String>,
}

/// HORUS package dependency specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HorusDependency {
    pub name: String,
    pub version: Option<String>,
    pub registry: Option<String>,
    pub features: Option<Vec<String>>,
}

impl CargoConfig {
    /// Load Cargo.toml from file
    pub fn load(path: &Path) -> Result<Self> {
        let contents =
            fs::read_to_string(path).with_context(|| format!("Failed to read {:?}", path))?;

        // Parse as raw TOML first
        let cargo_toml: toml::Value = toml::from_str(&contents)
            .with_context(|| format!("Failed to parse {:?} as TOML", path))?;

        // Extract package.metadata.horus if it exists
        let metadata = cargo_toml
            .get("package")
            .and_then(|p| p.get("metadata"))
            .and_then(|m| m.get("horus"))
            .and_then(|h| serde_json::to_value(h).ok())
            .and_then(|v| serde_json::from_value::<HorusMetadata>(v).ok());

        // Parse the main structure
        let package = cargo_toml
            .get("package")
            .ok_or_else(|| anyhow::anyhow!("Missing [package] section"))?
            .clone();

        let package_section: PackageSection = package
            .try_into()
            .context("Failed to parse [package] section")?;

        let dependencies = cargo_toml
            .get("dependencies")
            .and_then(|d| d.as_table())
            .cloned()
            .unwrap_or_default();

        Ok(Self {
            package: package_section,
            dependencies,
            metadata: metadata.map(|h| PackageMetadata { horus: Some(h) }),
        })
    }

    /// Create a minimal Cargo.toml for a new package
    pub fn new_minimal(name: &str) -> Self {
        Self {
            package: PackageSection {
                name: name.to_string(),
                version: "0.1.6".to_string(),
                edition: Some("2021".to_string()),
                authors: None,
                description: Some("HORUS package".to_string()),
                license: Some("Apache-2.0".to_string()),
                repository: None,
                homepage: None,
                documentation: None,
                readme: None,
                keywords: None,
                categories: None,
            },
            dependencies: toml::Table::new(),
            metadata: Some(PackageMetadata {
                horus: Some(HorusMetadata {
                    tags: Some(vec!["horus".to_string()]),
                    capabilities: None,
                    hardware: None,
                    min_horus_version: Some("0.1.6".to_string()),
                    max_horus_version: None,
                    publishable: true,
                    tier: Some("free".to_string()),
                    preview_image: None,
                    demo_video: None,
                    examples: None,
                    horus_dependencies: None,
                    backend: None,
                    logging_level: None,
                    priority: None,
                    tick_rate: None,
                }),
            }),
        }
    }
}
