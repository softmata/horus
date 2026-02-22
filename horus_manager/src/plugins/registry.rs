//! Plugin Registry - manages plugins.lock file
//!
//! The registry tracks installed plugins with their metadata, checksums,
//! and compatibility information.

use anyhow::{anyhow, Context, Result};
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

use super::{HORUS_VERSION, SCHEMA_VERSION};

/// Plugin registry stored in plugins.lock
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PluginRegistry {
    /// Schema version for forward compatibility
    pub schema_version: String,

    /// Scope of this registry (global or project)
    pub scope: PluginScope,

    /// Project name (only for project-scoped registries)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub project_name: Option<String>,

    /// HORUS version when registry was last updated
    pub horus_version: String,

    /// Last update timestamp
    pub updated_at: DateTime<Utc>,

    /// Installed plugins (command name -> plugin entry)
    pub plugins: HashMap<String, PluginEntry>,

    /// Disabled plugins (command name -> disabled info)
    #[serde(default, skip_serializing_if = "HashMap::is_empty")]
    pub disabled: HashMap<String, DisabledPlugin>,

    /// Whether to inherit plugins from global registry (project-only)
    #[serde(default = "default_inherit_global")]
    pub inherit_global: bool,
}

fn default_inherit_global() -> bool {
    true
}

/// Scope of the plugin registry
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PluginScope {
    Global,
    Project,
}

/// A single plugin entry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PluginEntry {
    /// Package name that provides this plugin
    pub package: String,

    /// Installed version
    pub version: String,

    /// Where the plugin came from
    pub source: PluginSource,

    /// Path to the plugin binary
    pub binary: PathBuf,

    /// SHA256 checksum of the binary
    pub checksum: String,

    /// Optional GPG/minisign signature
    #[serde(skip_serializing_if = "Option::is_none")]
    pub signature: Option<String>,

    /// Installation timestamp
    pub installed_at: DateTime<Utc>,

    /// HORUS version when installed
    pub installed_by: String,

    /// Compatibility constraints
    pub compatibility: Compatibility,

    /// Available subcommands
    pub commands: Vec<CommandInfo>,

    /// Required permissions
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub permissions: Vec<String>,
}

/// Source of the plugin
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum PluginSource {
    /// From HORUS registry
    Registry,

    /// From Git repository
    Git {
        url: String,
        #[serde(skip_serializing_if = "Option::is_none")]
        rev: Option<String>,
    },

    /// From local path
    Local { path: PathBuf },

    /// From crates.io
    CratesIo,

    /// From PyPI
    PyPI,
}

/// Version compatibility constraints
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Compatibility {
    /// Minimum HORUS version required
    pub horus_min: String,

    /// Maximum HORUS version supported (exclusive)
    pub horus_max: String,

    /// Supported platforms
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub platforms: Vec<String>,
}

impl Default for Compatibility {
    fn default() -> Self {
        Self {
            horus_min: "0.1.0".to_string(),
            horus_max: "2.0.0".to_string(),
            platforms: vec![],
        }
    }
}

/// Information about a plugin subcommand
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandInfo {
    /// Subcommand name
    pub name: String,

    /// Brief description
    pub description: String,
}

/// Information about a disabled plugin
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DisabledPlugin {
    /// Original plugin entry
    pub plugin: PluginEntry,

    /// Reason for disabling
    pub reason: String,

    /// When it was disabled
    pub disabled_at: DateTime<Utc>,
}

impl Default for PluginRegistry {
    fn default() -> Self {
        Self {
            schema_version: SCHEMA_VERSION.to_string(),
            scope: PluginScope::Global,
            project_name: None,
            horus_version: HORUS_VERSION.to_string(),
            updated_at: Utc::now(),
            plugins: HashMap::new(),
            disabled: HashMap::new(),
            inherit_global: true,
        }
    }
}

impl PluginRegistry {
    /// Create a new global registry
    pub fn new_global() -> Self {
        Self {
            scope: PluginScope::Global,
            ..Default::default()
        }
    }

    /// Create a new project registry
    pub fn new_project(project_name: &str) -> Self {
        Self {
            scope: PluginScope::Project,
            project_name: Some(project_name.to_string()),
            ..Default::default()
        }
    }

    /// Get the global registry path
    pub fn global_path() -> Result<PathBuf> {
        Ok(crate::paths::horus_dir()?.join("plugins.lock"))
    }

    /// Get the global bin directory path
    pub fn global_bin_dir() -> Result<PathBuf> {
        Ok(crate::paths::horus_dir()?.join("bin"))
    }

    /// Get the project registry path for a given project root
    pub fn project_path(project_root: &Path) -> PathBuf {
        project_root.join(".horus").join("plugins.lock")
    }

    /// Get the project bin directory path
    pub fn project_bin_dir(project_root: &Path) -> PathBuf {
        project_root.join(".horus").join("bin")
    }

    /// Load registry from a path
    pub fn load(path: &Path) -> Result<Self> {
        let content = fs::read_to_string(path)
            .with_context(|| format!("Failed to read plugins.lock from {}", path.display()))?;
        let registry: Self = serde_json::from_str(&content)
            .with_context(|| format!("failed to parse plugins.lock from {}", path.display()))?;
        Ok(registry)
    }

    /// Load registry from path, or return default if not found
    pub fn load_or_default(path: &Path) -> Self {
        Self::load(path).unwrap_or_default()
    }

    /// Load global registry
    pub fn load_global() -> Result<Self> {
        let path = Self::global_path()?;
        if path.exists() {
            Self::load(&path)
        } else {
            Ok(Self::new_global())
        }
    }

    /// Load project registry
    pub fn load_project(project_root: &Path) -> Option<Self> {
        let path = Self::project_path(project_root);
        if path.exists() {
            Self::load(&path).ok()
        } else {
            None
        }
    }

    /// Save registry to its appropriate location
    pub fn save(&self) -> Result<()> {
        let path = match self.scope {
            PluginScope::Global => Self::global_path()?,
            PluginScope::Project => {
                // For project registries, we need the project root
                // This should be called with save_to() instead
                return Err(anyhow!(
                    "Use save_to() for project registries or ensure project path is set"
                ));
            }
        };
        self.save_to(&path)
    }

    /// Save registry to a specific path
    pub fn save_to(&self, path: &Path) -> Result<()> {
        // Ensure parent directory exists
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }

        let content = serde_json::to_string_pretty(self)?;
        fs::write(path, content)?;
        Ok(())
    }

    /// Register a new plugin
    pub fn register_plugin(&mut self, command: &str, entry: PluginEntry) {
        // Remove from disabled if it was there
        self.disabled.remove(command);

        // Add to active plugins
        self.plugins.insert(command.to_string(), entry);
        self.updated_at = Utc::now();
        self.horus_version = HORUS_VERSION.to_string();
    }

    /// Unregister a plugin
    pub fn unregister_plugin(&mut self, command: &str) -> Option<PluginEntry> {
        self.updated_at = Utc::now();
        self.plugins.remove(command)
    }

    /// Disable a plugin without removing it
    pub fn disable_plugin(&mut self, command: &str, reason: &str) -> Result<()> {
        let entry = self
            .plugins
            .remove(command)
            .ok_or_else(|| anyhow!("Plugin '{}' not found", command))?;

        self.disabled.insert(
            command.to_string(),
            DisabledPlugin {
                plugin: entry,
                reason: reason.to_string(),
                disabled_at: Utc::now(),
            },
        );

        self.updated_at = Utc::now();
        Ok(())
    }

    /// Enable a previously disabled plugin
    pub fn enable_plugin(&mut self, command: &str) -> Result<()> {
        let disabled = self
            .disabled
            .remove(command)
            .ok_or_else(|| anyhow!("Plugin '{}' is not disabled", command))?;

        self.plugins.insert(command.to_string(), disabled.plugin);
        self.updated_at = Utc::now();
        Ok(())
    }

    /// Get a plugin by command name
    pub fn get_plugin(&self, command: &str) -> Option<&PluginEntry> {
        self.plugins.get(command)
    }

    /// Check if a plugin is disabled
    pub fn is_disabled(&self, command: &str) -> bool {
        self.disabled.contains_key(command)
    }

    /// Get all active plugin commands
    pub fn active_commands(&self) -> Vec<&str> {
        self.plugins.keys().map(|s| s.as_str()).collect()
    }

    /// Get all disabled plugin commands
    pub fn disabled_commands(&self) -> Vec<&str> {
        self.disabled.keys().map(|s| s.as_str()).collect()
    }

    /// Verify plugin binary checksum
    pub fn verify_plugin(&self, command: &str) -> Result<bool> {
        let entry = self
            .plugins
            .get(command)
            .ok_or_else(|| anyhow!("Plugin '{}' not found", command))?;

        if !entry.binary.exists() {
            return Err(anyhow!(
                "Plugin binary not found: {}",
                entry.binary.display()
            ));
        }

        let content = fs::read(&entry.binary)?;
        let mut hasher = Sha256::new();
        hasher.update(&content);
        let computed = format!("sha256:{:x}", hasher.finalize());

        Ok(computed == entry.checksum)
    }

    /// Check if plugin is compatible with current HORUS version
    pub fn is_compatible(&self, entry: &PluginEntry) -> bool {
        let current = match semver::Version::parse(HORUS_VERSION) {
            Ok(v) => v,
            Err(_) => return true, // Permissive if version unparseable
        };

        let min = semver::Version::parse(&entry.compatibility.horus_min).ok();
        let max = semver::Version::parse(&entry.compatibility.horus_max).ok();

        match (min, max) {
            (Some(min), Some(max)) => current >= min && current < max,
            (Some(min), None) => current >= min,
            (None, Some(max)) => current < max,
            (None, None) => true,
        }
    }

    /// Calculate checksum for a binary file
    pub fn calculate_checksum(path: &Path) -> Result<String> {
        let content = fs::read(path)?;
        let mut hasher = Sha256::new();
        hasher.update(&content);
        Ok(format!("sha256:{:x}", hasher.finalize()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    #[test]
    fn test_registry_default() {
        let registry = PluginRegistry::default();
        assert_eq!(registry.schema_version, SCHEMA_VERSION);
        assert_eq!(registry.scope, PluginScope::Global);
        assert!(registry.plugins.is_empty());
        assert!(registry.inherit_global);
    }

    #[test]
    fn test_new_project_registry() {
        let registry = PluginRegistry::new_project("my_robot");
        assert_eq!(registry.scope, PluginScope::Project);
        assert_eq!(registry.project_name, Some("my_robot".to_string()));
    }

    #[test]
    fn test_register_and_get_plugin() {
        let mut registry = PluginRegistry::default();

        let entry = PluginEntry {
            package: "nav2-lite".to_string(),
            version: "1.0.0".to_string(),
            source: PluginSource::Registry,
            binary: PathBuf::from("/usr/local/bin/horus-nav2"),
            checksum: "sha256:abc123".to_string(),
            signature: None,
            installed_at: Utc::now(),
            installed_by: HORUS_VERSION.to_string(),
            compatibility: Compatibility::default(),
            commands: vec![CommandInfo {
                name: "run".to_string(),
                description: "Run navigation".to_string(),
            }],
            permissions: vec![],
        };

        registry.register_plugin("nav2", entry.clone());

        assert!(registry.get_plugin("nav2").is_some());
        assert_eq!(registry.get_plugin("nav2").unwrap().package, "nav2-lite");
    }

    #[test]
    fn test_disable_enable_plugin() {
        let mut registry = PluginRegistry::default();

        let entry = PluginEntry {
            package: "test-plugin".to_string(),
            version: "1.0.0".to_string(),
            source: PluginSource::Registry,
            binary: PathBuf::from("/bin/horus-test"),
            checksum: "sha256:test".to_string(),
            signature: None,
            installed_at: Utc::now(),
            installed_by: HORUS_VERSION.to_string(),
            compatibility: Compatibility::default(),
            commands: vec![],
            permissions: vec![],
        };

        registry.register_plugin("test", entry);
        assert!(registry.get_plugin("test").is_some());

        // Disable
        registry.disable_plugin("test", "Testing").unwrap();
        assert!(registry.get_plugin("test").is_none());
        assert!(registry.is_disabled("test"));

        // Enable
        registry.enable_plugin("test").unwrap();
        assert!(registry.get_plugin("test").is_some());
        assert!(!registry.is_disabled("test"));
    }

    #[test]
    fn test_save_and_load() {
        let temp_dir = TempDir::new().unwrap();
        let path = temp_dir.path().join("plugins.lock");

        let mut registry = PluginRegistry::new_global();
        registry.register_plugin(
            "test",
            PluginEntry {
                package: "test-pkg".to_string(),
                version: "1.0.0".to_string(),
                source: PluginSource::Registry,
                binary: PathBuf::from("/bin/horus-test"),
                checksum: "sha256:test".to_string(),
                signature: None,
                installed_at: Utc::now(),
                installed_by: HORUS_VERSION.to_string(),
                compatibility: Compatibility::default(),
                commands: vec![],
                permissions: vec![],
            },
        );

        registry.save_to(&path).unwrap();

        let loaded = PluginRegistry::load(&path).unwrap();
        assert_eq!(loaded.plugins.len(), 1);
        assert!(loaded.get_plugin("test").is_some());
    }

    #[test]
    fn test_compatibility_check() {
        let registry = PluginRegistry::default();

        let compatible_entry = PluginEntry {
            package: "test".to_string(),
            version: "1.0.0".to_string(),
            source: PluginSource::Registry,
            binary: PathBuf::from("/bin/test"),
            checksum: "sha256:test".to_string(),
            signature: None,
            installed_at: Utc::now(),
            installed_by: HORUS_VERSION.to_string(),
            compatibility: Compatibility {
                horus_min: "0.1.0".to_string(),
                horus_max: "2.0.0".to_string(),
                platforms: vec![],
            },
            commands: vec![],
            permissions: vec![],
        };

        assert!(registry.is_compatible(&compatible_entry));
    }
}
