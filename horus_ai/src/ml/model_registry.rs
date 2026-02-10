// Model Registry for version management
//
// Manages model metadata, versioning, and deployment configuration.

use horus_core::error::{HorusError, HorusResult};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

/// Model metadata entry
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ModelEntry {
    /// Model name
    pub name: String,
    /// Model version (semver)
    pub version: String,
    /// Model format (onnx, tflite, pytorch, etc.)
    pub format: String,
    /// Path or URL to model file
    pub path: String,
    /// SHA256 hash of model file
    pub hash: Option<String>,
    /// Model size in bytes
    pub size_bytes: Option<u64>,
    /// Model metrics (mAP, accuracy, latency, etc.)
    pub metrics: HashMap<String, f64>,
    /// Status: "stable", "latest", "deprecated"
    pub status: String,
    /// Training date
    pub trained_date: Option<String>,
    /// Dataset used for training
    pub dataset: Option<String>,
    /// Additional metadata
    pub metadata: HashMap<String, String>,
}

/// Model Registry
///
/// Manages a collection of models with versioning and metadata.
pub struct ModelRegistry {
    /// Path to registry configuration file (models.yaml)
    config_path: PathBuf,
    /// Registry entries
    entries: HashMap<String, Vec<ModelEntry>>,
}

impl ModelRegistry {
    /// Create a new model registry from a configuration file
    pub fn new(config_path: PathBuf) -> HorusResult<Self> {
        let entries = if config_path.exists() {
            Self::load_from_file(&config_path)?
        } else {
            HashMap::new()
        };

        Ok(Self {
            config_path,
            entries,
        })
    }

    /// Load registry from YAML file
    fn load_from_file(path: &Path) -> HorusResult<HashMap<String, Vec<ModelEntry>>> {
        let contents = fs::read_to_string(path)
            .map_err(|e| HorusError::Config(format!("Failed to read registry file: {}", e)))?;

        #[derive(Deserialize)]
        struct RegistryFile {
            models: HashMap<String, ModelsConfig>,
        }

        #[derive(Deserialize)]
        struct ModelsConfig {
            versions: Vec<ModelEntry>,
        }

        let registry: RegistryFile = serde_yaml::from_str(&contents)
            .map_err(|e| HorusError::Config(format!("Failed to parse registry YAML: {}", e)))?;

        let mut entries = HashMap::new();
        for (name, config) in registry.models {
            entries.insert(name, config.versions);
        }

        Ok(entries)
    }

    /// Save registry to YAML file
    pub fn save(&self) -> HorusResult<()> {
        #[derive(Serialize)]
        struct RegistryFile {
            models: HashMap<String, ModelsConfig>,
        }

        #[derive(Serialize)]
        struct ModelsConfig {
            versions: Vec<ModelEntry>,
        }

        let mut models = HashMap::new();
        for (name, versions) in &self.entries {
            models.insert(
                name.clone(),
                ModelsConfig {
                    versions: versions.clone(),
                },
            );
        }

        let registry = RegistryFile { models };

        let yaml = serde_yaml::to_string(&registry)
            .map_err(|e| HorusError::Config(format!("Failed to serialize registry: {}", e)))?;

        fs::write(&self.config_path, yaml)
            .map_err(|e| HorusError::Config(format!("Failed to write registry file: {}", e)))?;

        Ok(())
    }

    /// Get a specific model version
    pub fn get_model(&self, name: &str, version: Option<&str>) -> HorusResult<ModelEntry> {
        let versions = self
            .entries
            .get(name)
            .ok_or_else(|| HorusError::Config(format!("Model '{}' not found in registry", name)))?;

        if let Some(ver) = version {
            // Find specific version
            for entry in versions {
                if entry.version == ver {
                    return Ok(entry.clone());
                }
            }

            Err(HorusError::Config(format!(
                "Model '{}' version '{}' not found",
                name, ver
            )))
        } else {
            // Return latest version (first one marked as "latest" or first in list)
            for entry in versions {
                if entry.status == "latest" {
                    return Ok(entry.clone());
                }
            }

            // Fallback to first version
            versions.first().cloned().ok_or_else(|| {
                HorusError::Config(format!("No versions found for model '{}'", name))
            })
        }
    }

    /// List all models in registry
    pub fn list_models(&self) -> Vec<String> {
        self.entries.keys().cloned().collect()
    }

    /// List all versions of a model
    pub fn list_versions(&self, name: &str) -> Vec<String> {
        self.entries
            .get(name)
            .map(|versions| versions.iter().map(|e| e.version.clone()).collect())
            .unwrap_or_default()
    }

    /// Register a new model version
    pub fn register_model(&mut self, entry: ModelEntry) -> HorusResult<()> {
        let name = entry.name.clone();

        let versions = self.entries.entry(name.clone()).or_default();

        // Check if version already exists
        for existing in versions.iter() {
            if existing.version == entry.version {
                return Err(HorusError::Config(format!(
                    "Model '{}' version '{}' already exists",
                    name, entry.version
                )));
            }
        }

        versions.push(entry);

        Ok(())
    }

    /// Deprecate a model version
    pub fn deprecate_model(&mut self, name: &str, version: &str) -> HorusResult<()> {
        let versions = self
            .entries
            .get_mut(name)
            .ok_or_else(|| HorusError::Config(format!("Model '{}' not found", name)))?;

        for entry in versions.iter_mut() {
            if entry.version == version {
                entry.status = "deprecated".to_string();
                return Ok(());
            }
        }

        Err(HorusError::Config(format!(
            "Model '{}' version '{}' not found",
            name, version
        )))
    }

    /// Set a version as "latest"
    pub fn set_latest(&mut self, name: &str, version: &str) -> HorusResult<()> {
        let versions = self
            .entries
            .get_mut(name)
            .ok_or_else(|| HorusError::Config(format!("Model '{}' not found", name)))?;

        // Remove "latest" from all versions
        for entry in versions.iter_mut() {
            if entry.status == "latest" {
                entry.status = "stable".to_string();
            }
        }

        // Set specified version as latest
        for entry in versions.iter_mut() {
            if entry.version == version {
                entry.status = "latest".to_string();
                return Ok(());
            }
        }

        Err(HorusError::Config(format!(
            "Model '{}' version '{}' not found",
            name, version
        )))
    }

    /// Get default registry path (~/.horus/models.yaml)
    pub fn default_path() -> HorusResult<PathBuf> {
        let home = dirs::home_dir()
            .ok_or_else(|| HorusError::Config("Could not determine home directory".to_string()))?;

        Ok(home.join(".horus").join("models.yaml"))
    }
}

impl Default for ModelRegistry {
    fn default() -> Self {
        let config_path =
            Self::default_path().unwrap_or_else(|_| PathBuf::from(".horus/models.yaml"));
        Self::new(config_path.clone()).unwrap_or_else(|_| Self {
            config_path,
            entries: HashMap::new(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_register_and_get_model() {
        let temp_path = std::env::temp_dir().join("test_models.yaml");
        let mut registry = ModelRegistry::new(temp_path.clone()).unwrap();

        let entry = ModelEntry {
            name: "yolov8n".to_string(),
            version: "1.0.0".to_string(),
            format: "onnx".to_string(),
            path: "models/yolov8n.onnx".to_string(),
            hash: Some("abc123".to_string()),
            size_bytes: Some(6000000),
            metrics: [("mAP".to_string(), 0.372)].iter().cloned().collect(),
            status: "latest".to_string(),
            trained_date: Some("2024-01-15".to_string()),
            dataset: Some("COCO".to_string()),
            metadata: HashMap::new(),
        };

        registry.register_model(entry.clone()).unwrap();

        let retrieved = registry.get_model("yolov8n", None).unwrap();
        assert_eq!(retrieved.version, "1.0.0");
        assert_eq!(retrieved.status, "latest");

        // Cleanup
        fs::remove_file(temp_path).ok();
    }
}
