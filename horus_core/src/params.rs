//! Simple runtime parameter system for HORUS
//!
//! Provides a straightforward key-value store for runtime configuration

use crate::error::{HorusError, HorusResult};
use regex;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, RwLock};

/// Validation rules for parameter values
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ValidationRule {
    /// Minimum value for numbers
    MinValue(f64),
    /// Maximum value for numbers
    MaxValue(f64),
    /// Range for numbers (min, max)
    Range(f64, f64),
    /// Regex pattern for strings (stored as string pattern)
    RegexPattern(String),
    /// Allowed enum values (list of acceptable strings)
    Enum(Vec<String>),
    /// Minimum length for strings/arrays
    MinLength(usize),
    /// Maximum length for strings/arrays
    MaxLength(usize),
    /// Required object keys
    RequiredKeys(Vec<String>),
}

/// Parameter metadata including validation rules
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParamMetadata {
    /// Human-readable description
    pub description: Option<String>,
    /// Unit of measurement (e.g., "m/s", "Hz")
    pub unit: Option<String>,
    /// Validation rules
    pub validation: Vec<ValidationRule>,
    /// Whether this parameter is read-only
    pub read_only: bool,
}

/// Simple runtime parameter store
pub struct RuntimeParams {
    /// Parameter storage - BTreeMap maintains sorted order
    params: Arc<RwLock<BTreeMap<String, Value>>>,
    /// Parameter metadata for validation
    metadata: Arc<RwLock<BTreeMap<String, ParamMetadata>>>,
    /// Version tracking for optimistic locking (concurrent edit protection)
    versions: Arc<RwLock<BTreeMap<String, u64>>>,
    /// Optional persistence path
    persist_path: Option<PathBuf>,
}

impl RuntimeParams {
    /// Canonical set of default parameters (single source of truth).
    fn default_params() -> BTreeMap<String, Value> {
        let mut params = BTreeMap::new();

        // System defaults
        params.insert("tick_rate".to_string(), Value::from(30));
        params.insert("max_memory_mb".to_string(), Value::from(512));

        // Motion defaults
        params.insert("max_speed".to_string(), Value::from(1.0));
        params.insert("max_angular_speed".to_string(), Value::from(1.0));
        params.insert("acceleration_limit".to_string(), Value::from(0.5));

        // Sensor defaults
        params.insert("lidar_rate".to_string(), Value::from(10));
        params.insert("camera_fps".to_string(), Value::from(30));
        params.insert("sensor_timeout_ms".to_string(), Value::from(1000));

        // Safety defaults
        params.insert("emergency_stop_distance".to_string(), Value::from(0.3));
        params.insert("collision_threshold".to_string(), Value::from(0.5));

        // PID defaults
        params.insert("pid_kp".to_string(), Value::from(1.0));
        params.insert("pid_ki".to_string(), Value::from(0.1));
        params.insert("pid_kd".to_string(), Value::from(0.05));

        params
    }

    /// Create new parameter store with defaults
    pub fn init() -> HorusResult<Self> {
        let mut initial_params = BTreeMap::new();

        // Try to load from .horus/config/params.yaml in current project
        let params_file = PathBuf::from(".horus/config/params.yaml");
        if params_file.exists() {
            if let Ok(yaml_str) = std::fs::read_to_string(&params_file) {
                if let Ok(loaded) = serde_yaml::from_str::<BTreeMap<String, Value>>(&yaml_str) {
                    initial_params = loaded;
                }
            }
        }

        // If empty, set defaults
        if initial_params.is_empty() {
            initial_params = Self::default_params();
        }

        Ok(Self {
            params: Arc::new(RwLock::new(initial_params)),
            metadata: Arc::new(RwLock::new(BTreeMap::new())),
            versions: Arc::new(RwLock::new(BTreeMap::new())),
            persist_path: Some(params_file),
        })
    }

    /// Set default parameters
    fn set_defaults(&self) -> Result<(), HorusError> {
        for (key, value) in Self::default_params() {
            self.set(&key, value)?;
        }
        Ok(())
    }

    /// Get a parameter value
    pub fn get<T: for<'de> Deserialize<'de>>(&self, key: &str) -> Option<T> {
        let params = self.params.read().ok()?;
        let value = params.get(key)?;
        serde_json::from_value(value.clone()).ok()
    }

    /// Get parameter with default
    pub fn get_or<T: for<'de> Deserialize<'de>>(&self, key: &str, default: T) -> T {
        self.get(key).unwrap_or(default)
    }

    /// Set a parameter value with validation
    pub fn set<T: Serialize>(&self, key: &str, value: T) -> Result<(), HorusError> {
        let json_value = serde_json::to_value(value)?;

        // Check if parameter is read-only
        if let Some(meta) = self.get_metadata(key) {
            if meta.read_only {
                return Err(HorusError::InvalidInput(format!(
                    "Parameter '{}' is read-only",
                    key
                )));
            }

            // Validate against rules
            self.validate_value(key, &json_value, &meta.validation)?;
        }

        // Get old value for audit log
        let old_value = {
            let params = self.params.read()?;
            params.get(key).cloned()
        };

        // Set new value and increment version
        let mut params = self.params.write()?;
        params.insert(key.to_string(), json_value.clone());
        drop(params);

        // Increment version for concurrent edit protection
        let mut versions = self.versions.write()?;
        let version = versions.entry(key.to_string()).or_insert(0);
        *version += 1;
        drop(versions);

        // Log change
        self.log_change(key, old_value.as_ref(), &json_value);

        Ok(())
    }

    /// Get all parameters
    pub fn get_all(&self) -> BTreeMap<String, Value> {
        self.params.read().map(|p| p.clone()).unwrap_or_default()
    }

    /// List all parameter keys
    pub fn list_keys(&self) -> Vec<String> {
        self.params
            .read()
            .map(|p| p.keys().cloned().collect())
            .unwrap_or_default()
    }

    /// Check if a parameter exists
    pub fn has(&self, key: &str) -> bool {
        self.params
            .read()
            .map(|p| p.contains_key(key))
            .unwrap_or(false)
    }

    /// Remove a parameter
    pub fn remove(&self, key: &str) -> Option<Value> {
        self.params.write().ok()?.remove(key)
    }

    /// Clear all parameters and reset to defaults
    pub fn reset(&self) -> Result<(), HorusError> {
        let mut params = self.params.write()?;
        params.clear();
        drop(params);
        self.set_defaults()?;
        Ok(())
    }

    /// Set metadata for a parameter
    pub fn set_metadata(&self, key: &str, metadata: ParamMetadata) -> Result<(), HorusError> {
        let mut meta_map = self.metadata.write()?;
        meta_map.insert(key.to_string(), metadata);
        Ok(())
    }

    /// Get metadata for a parameter
    pub fn get_metadata(&self, key: &str) -> Option<ParamMetadata> {
        self.metadata.read().ok()?.get(key).cloned()
    }

    /// Get version for a parameter (for concurrent edit protection)
    pub fn get_version(&self, key: &str) -> u64 {
        self.versions
            .read()
            .ok()
            .and_then(|v| v.get(key).copied())
            .unwrap_or(0)
    }

    /// Set a parameter value with version validation (optimistic locking)
    /// Returns error if the expected_version doesn't match the current version
    pub fn set_with_version<T: Serialize>(
        &self,
        key: &str,
        value: T,
        expected_version: u64,
    ) -> Result<(), HorusError> {
        // First, check the version
        let current_version = self.get_version(key);
        if current_version != expected_version {
            return Err(HorusError::InvalidInput(format!(
                "Version mismatch for '{}': expected {}, current {}. The parameter was modified by another user.",
                key, expected_version, current_version
            )));
        }

        // If version matches, proceed with normal set operation
        self.set(key, value)
    }

    /// Log parameter change to audit log
    fn log_change(&self, key: &str, old_value: Option<&Value>, new_value: &Value) {
        // Only log if we have a persist path (indicates we're in a project)
        if let Some(ref persist_path) = self.persist_path {
            if let Some(parent) = persist_path.parent() {
                let log_dir = parent.parent().unwrap_or(parent).join("logs");
                let log_file = log_dir.join("param_changes.log");

                // Create logs directory if it doesn't exist
                let _ = std::fs::create_dir_all(&log_dir);

                // Format log entry
                let timestamp = chrono::Local::now().format("%Y-%m-%d %H:%M:%S");
                let old_str = old_value
                    .map(|v| v.to_string())
                    .unwrap_or_else(|| "(none)".to_string());
                let new_str = new_value.to_string();

                let log_entry = format!("[{}] {}: {} -> {}\n", timestamp, key, old_str, new_str);

                // Append to log file (ignore errors to not block parameter updates)
                use std::io::Write;
                if let Ok(mut file) = std::fs::OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open(log_file)
                {
                    let _ = file.write_all(log_entry.as_bytes());
                }
            }
        }
    }

    /// Validate a value against validation rules
    fn validate_value(
        &self,
        key: &str,
        value: &Value,
        rules: &[ValidationRule],
    ) -> Result<(), HorusError> {
        for rule in rules {
            match rule {
                ValidationRule::MinValue(min) => {
                    if let Some(num) = value.as_f64() {
                        if num < *min {
                            return Err(HorusError::InvalidInput(format!(
                                "Parameter '{}' value {} is below minimum {}",
                                key, num, min
                            )));
                        }
                    }
                }
                ValidationRule::MaxValue(max) => {
                    if let Some(num) = value.as_f64() {
                        if num > *max {
                            return Err(HorusError::InvalidInput(format!(
                                "Parameter '{}' value {} exceeds maximum {}",
                                key, num, max
                            )));
                        }
                    }
                }
                ValidationRule::Range(min, max) => {
                    if let Some(num) = value.as_f64() {
                        if num < *min || num > *max {
                            return Err(HorusError::InvalidInput(format!(
                                "Parameter '{}' value {} is outside range [{}, {}]",
                                key, num, min, max
                            )));
                        }
                    }
                }
                ValidationRule::RegexPattern(pattern) => {
                    if let Some(s) = value.as_str() {
                        let re = regex::Regex::new(pattern).map_err(|e| {
                            HorusError::InvalidInput(format!("Invalid regex: {}", e))
                        })?;
                        if !re.is_match(s) {
                            return Err(HorusError::InvalidInput(format!(
                                "Parameter '{}' value '{}' does not match pattern '{}'",
                                key, s, pattern
                            )));
                        }
                    }
                }
                ValidationRule::Enum(allowed) => {
                    if let Some(s) = value.as_str() {
                        if !allowed.contains(&s.to_string()) {
                            return Err(HorusError::InvalidInput(format!(
                                "Parameter '{}' value '{}' not in allowed values: {:?}",
                                key, s, allowed
                            )));
                        }
                    }
                }
                ValidationRule::MinLength(min_len) => {
                    let len = if let Some(s) = value.as_str() {
                        s.len()
                    } else if let Some(arr) = value.as_array() {
                        arr.len()
                    } else {
                        0
                    };
                    if len < *min_len {
                        return Err(HorusError::InvalidInput(format!(
                            "Parameter '{}' length {} is below minimum {}",
                            key, len, min_len
                        )));
                    }
                }
                ValidationRule::MaxLength(max_len) => {
                    let len = if let Some(s) = value.as_str() {
                        s.len()
                    } else if let Some(arr) = value.as_array() {
                        arr.len()
                    } else {
                        0
                    };
                    if len > *max_len {
                        return Err(HorusError::InvalidInput(format!(
                            "Parameter '{}' length {} exceeds maximum {}",
                            key, len, max_len
                        )));
                    }
                }
                ValidationRule::RequiredKeys(required) => {
                    if let Some(obj) = value.as_object() {
                        for req_key in required {
                            if !obj.contains_key(req_key) {
                                return Err(HorusError::InvalidInput(format!(
                                    "Parameter '{}' missing required key '{}'",
                                    key, req_key
                                )));
                            }
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// Save parameters to YAML file
    pub fn save_to_disk(&self) -> Result<(), HorusError> {
        let path = self
            .persist_path
            .clone()
            .unwrap_or_else(|| PathBuf::from(".horus/config/params.yaml"));

        // Create .horus directory if it doesn't exist
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }

        let params = self.params.read()?;
        let yaml = serde_yaml::to_string(&*params)?;
        std::fs::write(path, yaml)?;
        Ok(())
    }

    /// Load parameters from YAML file
    pub fn load_from_disk(&self, path: &Path) -> Result<(), HorusError> {
        if path.exists() {
            let yaml_str = std::fs::read_to_string(path)?;
            let loaded: BTreeMap<String, Value> = serde_yaml::from_str(&yaml_str)?;

            let mut params = self.params.write()?;
            *params = loaded;
        }
        Ok(())
    }
}

impl Clone for RuntimeParams {
    fn clone(&self) -> Self {
        Self {
            params: self.params.clone(),
            metadata: self.metadata.clone(),
            versions: self.versions.clone(),
            persist_path: self.persist_path.clone(),
        }
    }
}

impl Default for RuntimeParams {
    fn default() -> Self {
        Self::init().unwrap_or_else(|e| {
            eprintln!(
                "Failed to initialize RuntimeParams: {}. Using empty params.",
                e
            );
            Self {
                params: Arc::new(RwLock::new(BTreeMap::new())),
                metadata: Arc::new(RwLock::new(BTreeMap::new())),
                versions: Arc::new(RwLock::new(BTreeMap::new())),
                persist_path: None,
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    fn create_test_params() -> RuntimeParams {
        RuntimeParams {
            params: Arc::new(RwLock::new(BTreeMap::new())),
            metadata: Arc::new(RwLock::new(BTreeMap::new())),
            versions: Arc::new(RwLock::new(BTreeMap::new())),
            persist_path: None,
        }
    }

    #[test]
    fn test_set_and_get_string() {
        let params = create_test_params();
        params.set("test_key", "test_value").unwrap();

        let value: Option<String> = params.get("test_key");
        assert_eq!(value, Some("test_value".to_string()));
    }

    #[test]
    fn test_set_and_get_number() {
        let params = create_test_params();
        params.set("number_key", 42).unwrap();

        let value: Option<i32> = params.get("number_key");
        assert_eq!(value, Some(42));
    }

    #[test]
    fn test_set_and_get_float() {
        let params = create_test_params();
        params.set("float_key", 1.234).unwrap();

        let value: Option<f64> = params.get("float_key");
        assert_eq!(value, Some(1.234));
    }

    #[test]
    fn test_set_and_get_bool() {
        let params = create_test_params();
        params.set("bool_key", true).unwrap();

        let value: Option<bool> = params.get("bool_key");
        assert_eq!(value, Some(true));
    }

    #[test]
    fn test_set_and_get_array() {
        let params = create_test_params();
        let test_array = vec![1, 2, 3, 4, 5];
        params.set("array_key", &test_array).unwrap();

        let value: Option<Vec<i32>> = params.get("array_key");
        assert_eq!(value, Some(test_array));
    }

    #[test]
    fn test_set_and_get_object() {
        let params = create_test_params();
        let mut test_obj = BTreeMap::new();
        test_obj.insert("name".to_string(), Value::String("test".to_string()));
        test_obj.insert("count".to_string(), Value::from(42));

        params.set("object_key", &test_obj).unwrap();

        let value: Option<BTreeMap<String, Value>> = params.get("object_key");
        assert_eq!(value, Some(test_obj));
    }

    #[test]
    fn test_get_nonexistent_key() {
        let params = create_test_params();
        let value: Option<String> = params.get("nonexistent");
        assert_eq!(value, None);
    }

    #[test]
    fn test_get_or_with_default() {
        let params = create_test_params();
        let value: i32 = params.get_or("missing_key", 99);
        assert_eq!(value, 99);
    }

    #[test]
    fn test_get_or_existing_value() {
        let params = create_test_params();
        params.set("existing_key", 42).unwrap();

        let value: i32 = params.get_or("existing_key", 99);
        assert_eq!(value, 42);
    }

    #[test]
    fn test_get_f64() {
        let params = create_test_params();
        params.set("test_float", 1.23456).unwrap();

        let value: f64 = params.get_or("test_float", 0.0);
        assert_eq!(value, 1.23456);
    }

    #[test]
    fn test_get_i32() {
        let params = create_test_params();
        params.set("age", 25).unwrap();

        let value: i32 = params.get_or("age", 0);
        assert_eq!(value, 25);
    }

    #[test]
    fn test_get_bool() {
        let params = create_test_params();
        params.set("enabled", true).unwrap();

        let value: bool = params.get_or("enabled", false);
        assert!(value);
    }

    #[test]
    fn test_get_string() {
        let params = create_test_params();
        params.set("name", "HORUS").unwrap();

        let value: String = params.get_or("name", "default".to_string());
        assert_eq!(value, "HORUS");
    }

    #[test]
    fn test_has() {
        let params = create_test_params();
        params.set("key1", "value1").unwrap();

        assert!(params.has("key1"));
        assert!(!params.has("key2"));
    }

    #[test]
    fn test_remove() {
        let params = create_test_params();
        params.set("to_remove", 123).unwrap();

        assert!(params.has("to_remove"));

        let removed = params.remove("to_remove");
        assert!(removed.is_some());
        assert!(!params.has("to_remove"));
    }

    #[test]
    fn test_remove_nonexistent() {
        let params = create_test_params();
        let removed = params.remove("nonexistent");
        assert!(removed.is_none());
    }

    #[test]
    fn test_list_keys() {
        let params = create_test_params();
        params.set("key1", "value1").unwrap();
        params.set("key2", "value2").unwrap();
        params.set("key3", "value3").unwrap();

        let keys = params.list_keys();
        assert_eq!(keys.len(), 3);
        assert!(keys.contains(&"key1".to_string()));
        assert!(keys.contains(&"key2".to_string()));
        assert!(keys.contains(&"key3".to_string()));
    }

    #[test]
    fn test_get_all() {
        let params = create_test_params();
        params.set("key1", "value1").unwrap();
        params.set("key2", 42).unwrap();

        let all = params.get_all();
        assert_eq!(all.len(), 2);
        assert!(all.contains_key("key1"));
        assert!(all.contains_key("key2"));
    }

    #[test]
    fn test_reset() {
        let params = create_test_params();
        params.set("custom_key", "custom_value").unwrap();

        params.reset().unwrap();

        // After reset, custom key should be gone
        assert!(!params.has("custom_key"));

        // But defaults should be present
        assert!(params.has("tick_rate"));
        assert!(params.has("max_speed"));
    }

    #[test]
    fn test_save_and_load() {
        let temp_dir = TempDir::new().unwrap();
        let test_file = temp_dir.path().join("test_params.yaml");

        // Create params and save
        let params1 = RuntimeParams {
            params: Arc::new(RwLock::new(BTreeMap::new())),
            metadata: Arc::new(RwLock::new(BTreeMap::new())),
            versions: Arc::new(RwLock::new(BTreeMap::new())),
            persist_path: Some(test_file.clone()),
        };

        params1.set("test_key", "test_value").unwrap();
        params1.set("test_number", 42).unwrap();
        params1.save_to_disk().unwrap();

        // Load into new instance
        let params2 = RuntimeParams {
            params: Arc::new(RwLock::new(BTreeMap::new())),
            metadata: Arc::new(RwLock::new(BTreeMap::new())),
            versions: Arc::new(RwLock::new(BTreeMap::new())),
            persist_path: Some(test_file.clone()),
        };

        params2.load_from_disk(&test_file).unwrap();

        // Verify loaded values
        let value1: Option<String> = params2.get("test_key");
        let value2: Option<i32> = params2.get("test_number");

        assert_eq!(value1, Some("test_value".to_string()));
        assert_eq!(value2, Some(42));
    }

    #[test]
    fn test_concurrent_access() {
        use std::thread;

        let params = Arc::new(create_test_params());
        let mut handles = vec![];

        // Spawn multiple threads writing different keys
        for i in 0..10 {
            let params_clone = Arc::clone(&params);
            let handle = thread::spawn(move || {
                let key = format!("key_{}", i);
                params_clone.set(&key, i).unwrap();
            });
            handles.push(handle);
        }

        // Wait for all threads to complete
        for handle in handles {
            handle.join().unwrap();
        }

        // Verify all keys were set
        for i in 0..10 {
            let key = format!("key_{}", i);
            assert!(params.has(&key));
            let value: Option<i32> = params.get(&key);
            assert_eq!(value, Some(i));
        }
    }

    #[test]
    fn test_overwrite_existing_key() {
        let params = create_test_params();
        params.set("key", "original").unwrap();

        let value1: Option<String> = params.get("key");
        assert_eq!(value1, Some("original".to_string()));

        params.set("key", "updated").unwrap();

        let value2: Option<String> = params.get("key");
        assert_eq!(value2, Some("updated".to_string()));
    }

    #[test]
    fn test_type_conversion() {
        let params = create_test_params();
        params.set("number", 42).unwrap();

        // Should be able to get as f64
        let as_f64: Option<f64> = params.get("number");
        assert_eq!(as_f64, Some(42.0));

        // Should be able to get as i32
        let as_i32: Option<i32> = params.get("number");
        assert_eq!(as_i32, Some(42));
    }

    #[test]
    fn test_clone() {
        let params1 = create_test_params();
        params1.set("key1", "value1").unwrap();

        let params2 = params1.clone();

        // Both should share the same underlying data (Arc)
        let value: Option<String> = params2.get("key1");
        assert_eq!(value, Some("value1".to_string()));

        // Modifying one should affect the other
        params2.set("key2", "value2").unwrap();
        assert!(params1.has("key2"));
    }

    #[test]
    fn test_sorted_order() {
        let params = create_test_params();
        params.set("zebra", 1).unwrap();
        params.set("alpha", 2).unwrap();
        params.set("beta", 3).unwrap();

        let keys = params.list_keys();

        // BTreeMap should maintain sorted order
        assert_eq!(keys, vec!["alpha", "beta", "zebra"]);
    }

    // Validation tests
    #[test]
    fn test_validation_min_value() {
        let params = create_test_params();
        let metadata = ParamMetadata {
            description: Some("Speed parameter".to_string()),
            unit: Some("m/s".to_string()),
            validation: vec![ValidationRule::MinValue(0.0)],
            read_only: false,
        };

        params.set_metadata("speed", metadata).unwrap();

        // Valid value
        assert!(params.set("speed", 10.0).is_ok());

        // Invalid value (below minimum)
        assert!(params.set("speed", -1.0).is_err());
    }

    #[test]
    fn test_validation_max_value() {
        let params = create_test_params();
        let metadata = ParamMetadata {
            description: None,
            unit: None,
            validation: vec![ValidationRule::MaxValue(100.0)],
            read_only: false,
        };

        params.set_metadata("temp", metadata).unwrap();

        // Valid value
        assert!(params.set("temp", 50.0).is_ok());

        // Invalid value (above maximum)
        assert!(params.set("temp", 150.0).is_err());
    }

    #[test]
    fn test_validation_range() {
        let params = create_test_params();
        let metadata = ParamMetadata {
            description: None,
            unit: None,
            validation: vec![ValidationRule::Range(0.0, 100.0)],
            read_only: false,
        };

        params.set_metadata("percentage", metadata).unwrap();

        // Valid values
        assert!(params.set("percentage", 0.0).is_ok());
        assert!(params.set("percentage", 50.0).is_ok());
        assert!(params.set("percentage", 100.0).is_ok());

        // Invalid values
        assert!(params.set("percentage", -1.0).is_err());
        assert!(params.set("percentage", 101.0).is_err());
    }

    #[test]
    fn test_validation_regex() {
        let params = create_test_params();
        let metadata = ParamMetadata {
            description: None,
            unit: None,
            validation: vec![ValidationRule::RegexPattern("^[a-z]+$".to_string())],
            read_only: false,
        };

        params.set_metadata("name", metadata).unwrap();

        // Valid value
        assert!(params.set("name", "hello").is_ok());

        // Invalid values
        assert!(params.set("name", "Hello").is_err()); // Capital letter
        assert!(params.set("name", "hello123").is_err()); // Contains digits
    }

    #[test]
    fn test_validation_enum() {
        let params = create_test_params();
        let metadata = ParamMetadata {
            description: None,
            unit: None,
            validation: vec![ValidationRule::Enum(vec![
                "low".to_string(),
                "medium".to_string(),
                "high".to_string(),
            ])],
            read_only: false,
        };

        params.set_metadata("priority", metadata).unwrap();

        // Valid values
        assert!(params.set("priority", "low").is_ok());
        assert!(params.set("priority", "medium").is_ok());
        assert!(params.set("priority", "high").is_ok());

        // Invalid value
        assert!(params.set("priority", "critical").is_err());
    }

    #[test]
    fn test_validation_min_length() {
        let params = create_test_params();
        let metadata = ParamMetadata {
            description: None,
            unit: None,
            validation: vec![ValidationRule::MinLength(3)],
            read_only: false,
        };

        params.set_metadata("name", metadata).unwrap();

        // Valid value
        assert!(params.set("name", "abc").is_ok());
        assert!(params.set("name", "abcd").is_ok());

        // Invalid value (too short)
        assert!(params.set("name", "ab").is_err());
    }

    #[test]
    fn test_validation_max_length() {
        let params = create_test_params();
        let metadata = ParamMetadata {
            description: None,
            unit: None,
            validation: vec![ValidationRule::MaxLength(5)],
            read_only: false,
        };

        params.set_metadata("code", metadata).unwrap();

        // Valid values
        assert!(params.set("code", "abc").is_ok());
        assert!(params.set("code", "abcde").is_ok());

        // Invalid value (too long)
        assert!(params.set("code", "abcdef").is_err());
    }

    #[test]
    fn test_validation_required_keys() {
        let params = create_test_params();
        let metadata = ParamMetadata {
            description: None,
            unit: None,
            validation: vec![ValidationRule::RequiredKeys(vec![
                "x".to_string(),
                "y".to_string(),
            ])],
            read_only: false,
        };

        params.set_metadata("position", metadata).unwrap();

        // Valid value (has required keys)
        let mut valid_obj = serde_json::Map::new();
        valid_obj.insert("x".to_string(), Value::from(10));
        valid_obj.insert("y".to_string(), Value::from(20));
        assert!(params.set("position", valid_obj).is_ok());

        // Invalid value (missing required key)
        let mut invalid_obj = serde_json::Map::new();
        invalid_obj.insert("x".to_string(), Value::from(10));
        assert!(params.set("position", invalid_obj).is_err());
    }

    #[test]
    fn test_read_only_parameter() {
        let params = create_test_params();
        let metadata = ParamMetadata {
            description: Some("Read-only system version".to_string()),
            unit: None,
            validation: vec![],
            read_only: true,
        };

        params.set_metadata("version", metadata).unwrap();

        // Should not be able to set read-only parameter
        assert!(params.set("version", "1.0.0").is_err());
    }

    #[test]
    fn test_metadata_get_set() {
        let params = create_test_params();
        let metadata = ParamMetadata {
            description: Some("Test param".to_string()),
            unit: Some("units".to_string()),
            validation: vec![ValidationRule::MinValue(0.0)],
            read_only: false,
        };

        params.set_metadata("test_key", metadata.clone()).unwrap();

        let retrieved = params.get_metadata("test_key").unwrap();
        assert_eq!(retrieved.description, metadata.description);
        assert_eq!(retrieved.unit, metadata.unit);
        assert_eq!(retrieved.read_only, metadata.read_only);
    }
}
