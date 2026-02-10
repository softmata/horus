// Model Loading and Caching for HORUS
//
// Utilities for loading ML models from local paths or URLs with automatic caching.

use horus_core::error::{HorusError, HorusResult};
use sha2::{Digest, Sha256};
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::Read;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

/// Model loader with caching support
pub struct ModelLoader {
    /// Cache directory for downloaded models
    cache_dir: PathBuf,
    /// Verify file checksums
    verify_checksums: bool,
    /// Expected checksums for models (URL/path -> SHA256 hex)
    expected_checksums: HashMap<String, String>,
    /// Cache of loaded model paths
    loaded_models: Arc<Mutex<HashMap<String, PathBuf>>>,
}

impl ModelLoader {
    /// Create a new model loader
    pub fn new(cache_dir: PathBuf) -> Self {
        Self {
            cache_dir,
            verify_checksums: false,
            expected_checksums: HashMap::new(),
            loaded_models: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    /// Create with checksum verification enabled
    pub fn with_verification(mut self) -> Self {
        self.verify_checksums = true;
        self
    }

    /// Add an expected checksum for a model URL or path
    ///
    /// The checksum should be a lowercase hex-encoded SHA256 hash.
    /// When verification is enabled, the model will be validated against this hash.
    pub fn with_checksum(mut self, path_or_url: &str, sha256_hex: &str) -> Self {
        self.expected_checksums
            .insert(path_or_url.to_string(), sha256_hex.to_lowercase());
        self
    }

    /// Compute SHA256 hash of a file
    fn compute_sha256(path: &Path) -> HorusResult<String> {
        let mut file = File::open(path)
            .map_err(|e| HorusError::Config(format!("Failed to open file for hashing: {}", e)))?;

        let mut hasher = Sha256::new();
        let mut buffer = [0u8; 8192];

        loop {
            let bytes_read = file
                .read(&mut buffer)
                .map_err(|e| HorusError::Config(format!("Failed to read file: {}", e)))?;
            if bytes_read == 0 {
                break;
            }
            hasher.update(&buffer[..bytes_read]);
        }

        let hash = hasher.finalize();
        Ok(format!("{:x}", hash))
    }

    /// Verify a file's checksum against the expected value
    fn verify_checksum(&self, path_or_url: &str, file_path: &Path) -> HorusResult<()> {
        if let Some(expected) = self.expected_checksums.get(path_or_url) {
            let actual = Self::compute_sha256(file_path)?;
            if actual != *expected {
                return Err(HorusError::Config(format!(
                    "Checksum mismatch for {}: expected {}, got {}",
                    path_or_url, expected, actual
                )));
            }
            println!("Checksum verified: {}", path_or_url);
        }
        Ok(())
    }

    /// Get default cache directory (~/.horus/models/)
    pub fn default_cache_dir() -> HorusResult<PathBuf> {
        let home = dirs::home_dir()
            .ok_or_else(|| HorusError::Config("Could not determine home directory".to_string()))?;

        let cache_dir = home.join(".horus").join("models");

        if !cache_dir.exists() {
            fs::create_dir_all(&cache_dir).map_err(|e| {
                HorusError::Config(format!("Failed to create cache directory: {}", e))
            })?;
        }

        Ok(cache_dir)
    }

    /// Load model from path or URL
    ///
    /// If path_or_url is a URL (starts with http:// or https://),
    /// downloads the model to cache_dir.
    ///
    /// If it's a local path, uses it directly.
    pub fn load(&self, path_or_url: &str) -> HorusResult<PathBuf> {
        // Check if already loaded
        {
            let loaded = self.loaded_models.lock().unwrap();
            if let Some(cached_path) = loaded.get(path_or_url) {
                if cached_path.exists() {
                    return Ok(cached_path.clone());
                }
            }
        }

        // Determine if URL or local path
        if path_or_url.starts_with("http://") || path_or_url.starts_with("https://") {
            self.download_model(path_or_url)
        } else {
            self.load_local(path_or_url)
        }
    }

    /// Load model from local path
    fn load_local(&self, path: &str) -> HorusResult<PathBuf> {
        let model_path = PathBuf::from(path);

        if !model_path.exists() {
            return Err(HorusError::Config(format!(
                "Model file not found: {}",
                path
            )));
        }

        // Cache the path
        {
            let mut loaded = self.loaded_models.lock().unwrap();
            loaded.insert(path.to_string(), model_path.clone());
        }

        Ok(model_path)
    }

    /// Download model from URL to cache
    fn download_model(&self, url: &str) -> HorusResult<PathBuf> {
        // Extract filename from URL
        let filename = url
            .rsplit('/')
            .next()
            .ok_or_else(|| HorusError::Config("Invalid URL: no filename".to_string()))?;

        let cache_path = self.cache_dir.join(filename);

        // Check if already cached
        if cache_path.exists() {
            println!("Using cached model: {:?}", cache_path);

            // Verify checksum if enabled and we have an expected hash
            if self.verify_checksums {
                self.verify_checksum(url, &cache_path)?;
            }

            // Cache the path
            {
                let mut loaded = self.loaded_models.lock().unwrap();
                loaded.insert(url.to_string(), cache_path.clone());
            }

            return Ok(cache_path);
        }

        println!("Downloading model from {} to {:?}", url, cache_path);

        // Download the file
        #[cfg(feature = "reqwest")]
        {
            let response = reqwest::blocking::get(url)
                .map_err(|e| HorusError::Config(format!("Download failed: {}", e)))?;

            if !response.status().is_success() {
                return Err(HorusError::Config(format!(
                    "Download failed with status: {}",
                    response.status()
                )));
            }

            let bytes = response
                .bytes()
                .map_err(|e| HorusError::Config(format!("Failed to read response: {}", e)))?;

            fs::write(&cache_path, bytes)
                .map_err(|e| HorusError::Config(format!("Failed to write model file: {}", e)))?;

            println!("Download complete");

            // Verify checksum after download if enabled
            if self.verify_checksums {
                self.verify_checksum(url, &cache_path)?;
            }

            // Cache the path
            {
                let mut loaded = self.loaded_models.lock().unwrap();
                loaded.insert(url.to_string(), cache_path.clone());
            }

            Ok(cache_path)
        }

        #[cfg(not(feature = "reqwest"))]
        {
            Err(HorusError::Config(
                "HTTP download not supported. Enable 'reqwest' feature.".to_string(),
            ))
        }
    }

    /// Clear the model cache
    pub fn clear_cache(&self) -> HorusResult<()> {
        if self.cache_dir.exists() {
            fs::remove_dir_all(&self.cache_dir)
                .map_err(|e| HorusError::Config(format!("Failed to clear cache: {}", e)))?;

            fs::create_dir_all(&self.cache_dir)
                .map_err(|e| HorusError::Config(format!("Failed to recreate cache dir: {}", e)))?;
        }

        // Clear loaded models cache
        {
            let mut loaded = self.loaded_models.lock().unwrap();
            loaded.clear();
        }

        Ok(())
    }

    /// Get cache directory path
    pub fn cache_dir(&self) -> &Path {
        &self.cache_dir
    }

    /// List cached models
    pub fn list_cached(&self) -> HorusResult<Vec<String>> {
        let mut models = Vec::new();

        if !self.cache_dir.exists() {
            return Ok(models);
        }

        for entry in fs::read_dir(&self.cache_dir)
            .map_err(|e| HorusError::Config(format!("Failed to read cache dir: {}", e)))?
        {
            let entry =
                entry.map_err(|e| HorusError::Config(format!("Failed to read entry: {}", e)))?;

            if entry.path().is_file() {
                if let Some(filename) = entry.file_name().to_str() {
                    models.push(filename.to_string());
                }
            }
        }

        Ok(models)
    }
}

impl Default for ModelLoader {
    fn default() -> Self {
        let cache_dir =
            Self::default_cache_dir().unwrap_or_else(|_| PathBuf::from(".horus/models"));
        Self::new(cache_dir)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn test_load_local_model() {
        // Create a temporary model file
        let temp_dir = std::env::temp_dir();
        let model_path = temp_dir.join("test_model.onnx");
        fs::write(&model_path, b"fake model data").unwrap();

        let loader = ModelLoader::new(temp_dir.join("cache"));
        let loaded_path = loader.load(model_path.to_str().unwrap()).unwrap();

        assert_eq!(loaded_path, model_path);

        // Cleanup
        fs::remove_file(model_path).ok();
    }

    #[test]
    fn test_cache_directory_creation() {
        let temp_dir = std::env::temp_dir().join("test_cache");
        if temp_dir.exists() {
            fs::remove_dir_all(&temp_dir).ok();
        }

        let _loader = ModelLoader::new(temp_dir.clone());
        // Cache dir is not created until first use

        // Cleanup
        fs::remove_dir_all(&temp_dir).ok();
    }

    #[test]
    fn test_checksum_verification() {
        let temp_dir = std::env::temp_dir();
        let model_path = temp_dir.join("test_checksum_model.bin");
        let model_data = b"test model content for checksum";
        fs::write(&model_path, model_data).unwrap();

        // Compute expected SHA256
        let expected_hash = ModelLoader::compute_sha256(&model_path).unwrap();

        // Test with correct checksum
        let loader = ModelLoader::new(temp_dir.join("cache"))
            .with_verification()
            .with_checksum(model_path.to_str().unwrap(), &expected_hash);

        // This should verify the checksum internally
        let result = loader.verify_checksum(model_path.to_str().unwrap(), &model_path);
        assert!(result.is_ok());

        // Test with wrong checksum
        let loader_bad = ModelLoader::new(temp_dir.join("cache"))
            .with_verification()
            .with_checksum(model_path.to_str().unwrap(), "badhash123");

        let result = loader_bad.verify_checksum(model_path.to_str().unwrap(), &model_path);
        assert!(result.is_err());

        // Cleanup
        fs::remove_file(model_path).ok();
    }

    #[test]
    fn test_sha256_computation() {
        let temp_dir = std::env::temp_dir();
        let test_file = temp_dir.join("test_sha256.txt");
        fs::write(&test_file, b"hello world").unwrap();

        let hash = ModelLoader::compute_sha256(&test_file).unwrap();

        // Known SHA256 of "hello world"
        assert_eq!(
            hash,
            "b94d27b9934d3e08a52e52d7da7dabfac484efe37a5380ee9088f7ace2efcde9"
        );

        // Cleanup
        fs::remove_file(test_file).ok();
    }
}
