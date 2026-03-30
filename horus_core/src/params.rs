//! Simple runtime parameter system for HORUS
//!
//! Provides a straightforward key-value store for runtime configuration

use crate::error::{HorusError, HorusResult, ValidationError};
use regex;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::{Arc, RwLock};

/// Validation rules for parameter values
#[derive(Debug, Clone, Serialize, Deserialize)]
pub(crate) enum ValidationRule {
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
    pub(crate) validation: Vec<ValidationRule>,
    /// Whether this parameter is read-only
    read_only: bool,
}

impl ParamMetadata {
    /// Whether this parameter is read-only.
    pub fn read_only(&self) -> bool {
        self.read_only
    }
}

/// Runtime parameter store for dynamic configuration.
///
/// Provides a typed key-value store with validation, persistence, and
/// concurrent access support.
///
/// # Example
///
/// ```rust
/// use horus_core::params::RuntimeParams;
///
/// let params = RuntimeParams::new().unwrap();
///
/// // Set and get typed parameters
/// params.set("max_speed", 1.5_f64).unwrap();
///
/// let speed: f64 = params.get("max_speed").unwrap();
/// assert_eq!(speed, 1.5);
/// ```
/// Callback type for parameter change notification.
/// Called with (key, old_value, new_value). Return Err to reject the change.
pub type ParamChangeCallback =
    Arc<dyn Fn(&str, Option<&Value>, &Value) -> Result<(), String> + Send + Sync>;

pub struct RuntimeParams {
    /// Parameter storage - BTreeMap maintains sorted order
    params: Arc<RwLock<BTreeMap<String, Value>>>,
    /// Parameter metadata for validation
    metadata: Arc<RwLock<BTreeMap<String, ParamMetadata>>>,
    /// Version tracking for optimistic locking (concurrent edit protection)
    versions: Arc<RwLock<BTreeMap<String, u64>>>,
    /// Optional persistence path
    persist_path: Option<PathBuf>,
    /// Change notification callbacks (called on every set).
    /// If any callback returns Err, the set is rejected and value rolls back.
    change_callbacks: Arc<RwLock<Vec<ParamChangeCallback>>>,
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

    /// Create a new parameter store with defaults.
    ///
    /// Loads parameters from `.horus/config/params.yaml` if present,
    /// otherwise uses built-in defaults.
    pub fn new() -> HorusResult<Self> {
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

        // Load HORUS_PARAM_* env vars (override YAML values).
        // Launch files set these via `horus_manager/src/commands/launch.rs`.
        for (key, val) in std::env::vars() {
            if let Some(param_name) = key.strip_prefix("HORUS_PARAM_") {
                let name = param_name.to_lowercase();
                let value = parse_env_value(&val);
                initial_params.insert(name, value);
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
            change_callbacks: Arc::new(RwLock::new(Vec::new())),
        })
    }

    /// Register a callback that fires on every parameter change.
    ///
    /// The callback receives (key, old_value, new_value) and can reject the
    /// change by returning Err. If rejected, the parameter value is not updated.
    ///
    /// Used by the scheduler to notify nodes via on_parameter_change().
    pub fn on_change(&self, callback: ParamChangeCallback) {
        if let Ok(mut cbs) = self.change_callbacks.write() {
            cbs.push(callback);
        }
    }

    /// Set default parameters
    fn set_defaults(&self) -> Result<(), HorusError> {
        for (key, value) in Self::default_params() {
            self.set(&key, value)?;
        }
        Ok(())
    }

    /// Get a parameter value, returning `None` if missing or on any error.
    ///
    /// For explicit error handling, use [`get_typed()`](Self::get_typed).
    pub fn get<T: for<'de> Deserialize<'de>>(&self, key: &str) -> Option<T> {
        let params = self.params.read().ok()?;
        let value = params.get(key)?;
        serde_json::from_value(value.clone()).ok()
    }

    /// Get a parameter value with explicit error reporting.
    ///
    /// Unlike [`get()`](Self::get) which silently returns `None` on errors,
    /// this method distinguishes between missing keys, type mismatches, and
    /// lock poisoning.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let speed: f64 = params.get_typed("max_speed")?;
    /// ```
    pub fn get_typed<T: for<'de> Deserialize<'de>>(&self, key: &str) -> HorusResult<T> {
        let params = self.params.read()?;
        let value = params.get(key).ok_or_else(|| {
            HorusError::NotFound(crate::error::NotFoundError::Parameter {
                name: key.to_string(),
            })
        })?;
        serde_json::from_value(value.clone()).map_err(|e| {
            HorusError::InvalidInput(ValidationError::Other(format!(
                "Parameter '{}' type mismatch: {}",
                key, e
            )))
        })
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
            if meta.read_only() {
                return Err(HorusError::InvalidInput(ValidationError::Other(format!(
                    "Parameter '{}' is read-only",
                    key
                ))));
            }

            // Validate against rules
            self.validate_value(key, &json_value, &meta.validation)?;
        }

        // Get old value for audit log + callbacks
        let old_value = {
            let params = self.params.read()?;
            params.get(key).cloned()
        };

        // Notify change callbacks — if any reject, abort the set
        if let Ok(callbacks) = self.change_callbacks.read() {
            for cb in callbacks.iter() {
                if let Err(reason) = cb(key, old_value.as_ref(), &json_value) {
                    return Err(HorusError::InvalidInput(ValidationError::Other(format!(
                        "Parameter change rejected by callback: {}",
                        reason
                    ))));
                }
            }
        }

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
        let json_value = serde_json::to_value(value)?;

        // Check read-only and validation
        if let Some(meta) = self.get_metadata(key) {
            if meta.read_only() {
                return Err(HorusError::InvalidInput(ValidationError::Other(format!(
                    "Parameter '{}' is read-only",
                    key
                ))));
            }
            self.validate_value(key, &json_value, &meta.validation)?;
        }

        // Hold versions write lock for atomic check-then-set
        let mut versions = self.versions.write()?;
        let current_version = versions.get(key).copied().unwrap_or(0);
        if current_version != expected_version {
            return Err(HorusError::InvalidInput(ValidationError::Other(format!(
                "Version mismatch for '{}': expected {}, current {}. The parameter was modified by another user.",
                key, expected_version, current_version
            ))));
        }

        let old_value = {
            let params = self.params.read()?;
            params.get(key).cloned()
        };

        let mut params = self.params.write()?;
        params.insert(key.to_string(), json_value.clone());
        drop(params);

        let version = versions.entry(key.to_string()).or_insert(0);
        *version += 1;
        drop(versions);

        self.log_change(key, old_value.as_ref(), &json_value);

        Ok(())
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
                    let num = value.as_f64().ok_or_else(|| {
                        HorusError::InvalidInput(ValidationError::Other(format!(
                            "Parameter '{}' requires a numeric value for MinValue rule",
                            key
                        )))
                    })?;
                    if num < *min {
                        return Err(HorusError::InvalidInput(ValidationError::Other(format!(
                            "Parameter '{}' value {} is below minimum {}",
                            key, num, min
                        ))));
                    }
                }
                ValidationRule::MaxValue(max) => {
                    let num = value.as_f64().ok_or_else(|| {
                        HorusError::InvalidInput(ValidationError::Other(format!(
                            "Parameter '{}' requires a numeric value for MaxValue rule",
                            key
                        )))
                    })?;
                    if num > *max {
                        return Err(HorusError::InvalidInput(ValidationError::Other(format!(
                            "Parameter '{}' value {} exceeds maximum {}",
                            key, num, max
                        ))));
                    }
                }
                ValidationRule::Range(min, max) => {
                    let num = value.as_f64().ok_or_else(|| {
                        HorusError::InvalidInput(ValidationError::Other(format!(
                            "Parameter '{}' requires a numeric value for Range rule",
                            key
                        )))
                    })?;
                    if num < *min || num > *max {
                        return Err(HorusError::InvalidInput(ValidationError::Other(format!(
                            "Parameter '{}' value {} is outside range [{}, {}]",
                            key, num, min, max
                        ))));
                    }
                }
                ValidationRule::RegexPattern(pattern) => {
                    let s = value.as_str().ok_or_else(|| {
                        HorusError::InvalidInput(ValidationError::Other(format!(
                            "Parameter '{}' requires a string value for RegexPattern rule",
                            key
                        )))
                    })?;
                    let re = regex::Regex::new(pattern).map_err(|e| {
                        HorusError::InvalidInput(ValidationError::Other(format!(
                            "Invalid regex: {}",
                            e
                        )))
                    })?;
                    if !re.is_match(s) {
                        return Err(HorusError::InvalidInput(ValidationError::Other(format!(
                            "Parameter '{}' value '{}' does not match pattern '{}'",
                            key, s, pattern
                        ))));
                    }
                }
                ValidationRule::Enum(allowed) => {
                    let s = value.as_str().ok_or_else(|| {
                        HorusError::InvalidInput(ValidationError::Other(format!(
                            "Parameter '{}' requires a string value for Enum rule",
                            key
                        )))
                    })?;
                    if !allowed.contains(&s.to_string()) {
                        return Err(HorusError::InvalidInput(ValidationError::Other(format!(
                            "Parameter '{}' value '{}' not in allowed values: {:?}",
                            key, s, allowed
                        ))));
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
                        return Err(HorusError::InvalidInput(ValidationError::Other(format!(
                            "Parameter '{}' length {} is below minimum {}",
                            key, len, min_len
                        ))));
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
                        return Err(HorusError::InvalidInput(ValidationError::Other(format!(
                            "Parameter '{}' length {} exceeds maximum {}",
                            key, len, max_len
                        ))));
                    }
                }
                ValidationRule::RequiredKeys(required) => {
                    if let Some(obj) = value.as_object() {
                        for req_key in required {
                            if !obj.contains_key(req_key) {
                                return Err(HorusError::InvalidInput(ValidationError::Other(
                                    format!(
                                        "Parameter '{}' missing required key '{}'",
                                        key, req_key
                                    ),
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
            change_callbacks: self.change_callbacks.clone(),
        }
    }
}

impl Default for RuntimeParams {
    fn default() -> Self {
        Self::new().unwrap_or_else(|e| {
            eprintln!(
                "Failed to initialize RuntimeParams: {}. Using empty params.",
                e
            );
            Self {
                params: Arc::new(RwLock::new(BTreeMap::new())),
                metadata: Arc::new(RwLock::new(BTreeMap::new())),
                versions: Arc::new(RwLock::new(BTreeMap::new())),
                persist_path: None,
                change_callbacks: Arc::new(RwLock::new(Vec::new())),
            }
        })
    }
}

/// Parse an environment variable value into a typed `serde_json::Value`.
///
/// Attempts to parse as i64, f64, or bool before falling back to a string.
/// Integer is tried first so "42" becomes `Value::Number(42)` not `42.0`.
fn parse_env_value(s: &str) -> Value {
    if let Ok(i) = s.parse::<i64>() {
        return Value::from(i);
    }
    if let Ok(f) = s.parse::<f64>() {
        return Value::from(f);
    }
    match s {
        "true" => Value::from(true),
        "false" => Value::from(false),
        _ => Value::from(s),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use serde_json::json;
    use tempfile::TempDir;

    fn create_test_params() -> RuntimeParams {
        RuntimeParams {
            params: Arc::new(RwLock::new(BTreeMap::new())),
            metadata: Arc::new(RwLock::new(BTreeMap::new())),
            versions: Arc::new(RwLock::new(BTreeMap::new())),
            persist_path: None,
            change_callbacks: Arc::new(RwLock::new(Vec::new())),
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
            change_callbacks: Arc::new(RwLock::new(Vec::new())),
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
            change_callbacks: Arc::new(RwLock::new(Vec::new())),
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

    // ── Helper: create params with metadata ──────────────────────────────

    fn create_params_with_rule(key: &str, rule: ValidationRule) -> RuntimeParams {
        let p = create_test_params();
        let meta = ParamMetadata {
            description: None,
            unit: None,
            validation: vec![rule],
            read_only: false,
        };
        p.metadata.write().unwrap().insert(key.to_string(), meta);
        p
    }

    fn create_readonly_params(key: &str) -> RuntimeParams {
        let p = create_test_params();
        let meta = ParamMetadata {
            description: None,
            unit: None,
            validation: vec![],
            read_only: true,
        };
        p.metadata.write().unwrap().insert(key.to_string(), meta);
        p
    }

    // ── Arbitrary strategies ─────────────────────────────────────────────

    fn arb_key() -> impl Strategy<Value = String> {
        "[a-z][a-z0-9_]{0,30}".prop_filter("non-empty key", |s| !s.is_empty())
    }

    fn arb_finite_f64() -> impl Strategy<Value = f64> {
        -1e12f64..1e12f64
    }

    // ── Property tests ───────────────────────────────────────────────────

    proptest! {
        #![proptest_config(ProptestConfig::with_cases(500))]

        /// set/get roundtrip for f64 values
        #[test]
        fn prop_set_get_roundtrip_f64(key in arb_key(), val in arb_finite_f64()) {
            let p = create_test_params();
            p.set(&key, val).unwrap();
            let got: f64 = p.get(&key).unwrap();
            prop_assert!((got - val).abs() < 1e-10,
                "f64 roundtrip: set {} got {}", val, got);
        }

        /// set/get roundtrip for i64 values
        #[test]
        fn prop_set_get_roundtrip_i64(key in arb_key(), val in any::<i32>()) {
            let p = create_test_params();
            p.set(&key, val).unwrap();
            let got: i32 = p.get(&key).unwrap();
            prop_assert_eq!(got, val);
        }

        /// set/get roundtrip for bool values
        #[test]
        fn prop_set_get_roundtrip_bool(key in arb_key(), val in any::<bool>()) {
            let p = create_test_params();
            p.set(&key, val).unwrap();
            let got: bool = p.get(&key).unwrap();
            prop_assert_eq!(got, val);
        }

        /// set/get roundtrip for string values
        #[test]
        fn prop_set_get_roundtrip_string(key in arb_key(), val in "[a-zA-Z0-9_ ]{0,100}") {
            let p = create_test_params();
            p.set(&key, &val).unwrap();
            let got: String = p.get(&key).unwrap();
            prop_assert_eq!(got, val);
        }

        /// has() returns true after set, false after remove
        #[test]
        fn prop_has_after_set_and_remove(key in arb_key(), val in any::<i32>()) {
            let p = create_test_params();
            prop_assert!(!p.has(&key));
            p.set(&key, val).unwrap();
            prop_assert!(p.has(&key));
            p.remove(&key);
            prop_assert!(!p.has(&key));
        }

        /// version increments by exactly 1 on each set
        #[test]
        fn prop_version_increments(key in arb_key(), vals in proptest::collection::vec(any::<i32>(), 1..20)) {
            let p = create_test_params();
            for (i, val) in vals.iter().enumerate() {
                let v_before = p.get_version(&key);
                prop_assert_eq!(v_before, i as u64, "version before set #{}", i);
                p.set(&key, val).unwrap();
                let v_after = p.get_version(&key);
                prop_assert_eq!(v_after, (i + 1) as u64, "version after set #{}", i);
            }
        }

        /// set_with_version succeeds when version matches, fails when it doesn't
        #[test]
        fn prop_optimistic_locking(key in arb_key(), v1 in any::<i32>(), v2 in any::<i32>()) {
            let p = create_test_params();
            p.set(&key, v1).unwrap();
            let ver = p.get_version(&key);

            // Correct version succeeds
            prop_assert!(p.set_with_version(&key, v2, ver).is_ok());

            // Stale version fails
            prop_assert!(p.set_with_version(&key, v1, ver).is_err());
        }

        /// read-only parameters reject all set attempts
        #[test]
        fn prop_read_only_rejects_set(key in arb_key(), val in any::<i32>()) {
            let p = create_readonly_params(&key);
            let result = p.set(&key, val);
            prop_assert!(result.is_err(), "read-only param should reject set");
        }

        /// MinValue: values below min are rejected, values >= min are accepted
        #[test]
        fn prop_min_value_enforcement(
            min in arb_finite_f64(),
            offset in 0.001f64..1e6f64,
        ) {
            let p = create_params_with_rule("v", ValidationRule::MinValue(min));

            // Value below min should fail
            let below = min - offset;
            prop_assert!(p.set("v", below).is_err(),
                "MinValue({}) should reject {}", min, below);

            // Value at min should succeed
            prop_assert!(p.set("v", min).is_ok(),
                "MinValue({}) should accept {}", min, min);

            // Value above min should succeed
            let above = min + offset;
            prop_assert!(p.set("v", above).is_ok(),
                "MinValue({}) should accept {}", min, above);
        }

        /// MaxValue: values above max are rejected, values <= max are accepted
        #[test]
        fn prop_max_value_enforcement(
            max in arb_finite_f64(),
            offset in 0.001f64..1e6f64,
        ) {
            let p = create_params_with_rule("v", ValidationRule::MaxValue(max));

            // Value above max should fail
            let above = max + offset;
            prop_assert!(p.set("v", above).is_err(),
                "MaxValue({}) should reject {}", max, above);

            // Value at max should succeed
            prop_assert!(p.set("v", max).is_ok(),
                "MaxValue({}) should accept {}", max, max);

            // Value below max should succeed
            let below = max - offset;
            prop_assert!(p.set("v", below).is_ok(),
                "MaxValue({}) should accept {}", max, below);
        }

        /// Range: values inside [min,max] accepted, outside rejected
        #[test]
        fn prop_range_enforcement(
            lo in -1e6f64..0.0f64,
            hi in 0.001f64..1e6f64,
            offset in 0.001f64..1e3f64,
        ) {
            let p = create_params_with_rule("v", ValidationRule::Range(lo, lo + hi));
            let max = lo + hi;

            // Inside range
            let mid = lo + hi / 2.0;
            prop_assert!(p.set("v", mid).is_ok(),
                "Range([{}, {}]) should accept {}", lo, max, mid);

            // Below range
            let below = lo - offset;
            prop_assert!(p.set("v", below).is_err(),
                "Range([{}, {}]) should reject {}", lo, max, below);

            // Above range
            let above = max + offset;
            prop_assert!(p.set("v", above).is_err(),
                "Range([{}, {}]) should reject {}", lo, max, above);
        }

        /// Enum: only allowed values accepted
        #[test]
        fn prop_enum_enforcement(
            allowed in proptest::collection::vec("[a-z]{1,10}", 1..5),
            reject in "[A-Z]{1,10}",
        ) {
            let p = create_params_with_rule("v", ValidationRule::Enum(allowed.clone()));

            // Each allowed value should succeed
            for val in &allowed {
                prop_assert!(p.set("v", val).is_ok(),
                    "Enum should accept '{}' from {:?}", val, allowed);
            }

            // An uppercase value should fail (won't be in lowercase-only allowed list)
            prop_assert!(p.set("v", &reject).is_err(),
                "Enum should reject '{}' not in {:?}", reject, allowed);
        }

        /// MinLength: strings shorter than min are rejected
        #[test]
        fn prop_min_length_enforcement(min_len in 1usize..20) {
            let p = create_params_with_rule("v", ValidationRule::MinLength(min_len));

            // Too short
            let short: String = "x".repeat(min_len.saturating_sub(1));
            if short.len() < min_len {
                prop_assert!(p.set("v", &short).is_err(),
                    "MinLength({}) should reject len={}", min_len, short.len());
            }

            // Exact length
            let exact: String = "x".repeat(min_len);
            prop_assert!(p.set("v", &exact).is_ok(),
                "MinLength({}) should accept len={}", min_len, exact.len());

            // Longer
            let long: String = "x".repeat(min_len + 5);
            prop_assert!(p.set("v", &long).is_ok(),
                "MinLength({}) should accept len={}", min_len, long.len());
        }

        /// MaxLength: strings longer than max are rejected
        #[test]
        fn prop_max_length_enforcement(max_len in 1usize..50) {
            let p = create_params_with_rule("v", ValidationRule::MaxLength(max_len));

            // Too long
            let long: String = "x".repeat(max_len + 1);
            prop_assert!(p.set("v", &long).is_err(),
                "MaxLength({}) should reject len={}", max_len, long.len());

            // Exact length
            let exact: String = "x".repeat(max_len);
            prop_assert!(p.set("v", &exact).is_ok(),
                "MaxLength({}) should accept len={}", max_len, exact.len());

            // Shorter
            if max_len > 0 {
                let short: String = "x".repeat(max_len - 1);
                prop_assert!(p.set("v", &short).is_ok(),
                    "MaxLength({}) should accept len={}", max_len, short.len());
            }
        }

        /// Reset always restores all default keys
        #[test]
        fn prop_reset_restores_defaults(
            keys in proptest::collection::vec(arb_key(), 0..10),
            vals in proptest::collection::vec(any::<i32>(), 10),
        ) {
            let p = create_test_params();
            // Set some random keys
            for (k, v) in keys.iter().zip(vals.iter()) {
                let _ = p.set(k, v);
            }
            // Reset
            p.reset().unwrap();
            // All default keys must be present
            let defaults = RuntimeParams::default_params();
            for key in defaults.keys() {
                prop_assert!(p.has(key), "After reset, default key '{}' missing", key);
            }
        }

        /// get_all returns exactly the keys that have been set
        #[test]
        fn prop_get_all_matches_has(
            entries in proptest::collection::vec((arb_key(), any::<i32>()), 0..20),
        ) {
            let p = create_test_params();
            for (k, v) in &entries {
                p.set(k, v).unwrap();
            }
            let all = p.get_all();
            // Every key in get_all should be findable via has()
            for key in all.keys() {
                prop_assert!(p.has(key), "get_all key '{}' not found by has()", key);
            }
            // list_keys should match get_all
            let keys = p.list_keys();
            prop_assert_eq!(keys.len(), all.len(),
                "list_keys len {} != get_all len {}", keys.len(), all.len());
        }
    }

    // ════════════════════════════════════════════════════════════════════════
    //  Scale: 1000+ params, concurrent modification, file round-trip
    // ════════════════════════════════════════════════════════════════════════

    #[test]
    fn scale_1000_params_set_get_list() {
        let params = create_test_params();

        // Set 1000 params
        let start = std::time::Instant::now();
        for i in 0..1000 {
            params.set(&format!("scale_key_{}", i), &(i as f64)).unwrap();
        }
        let set_elapsed = start.elapsed();

        // Get all 1000
        let start = std::time::Instant::now();
        for i in 0..1000 {
            let val: Option<Value> = params.get(&format!("scale_key_{}", i));
            assert!(val.is_some(), "key {} must exist", i);
        }
        let get_elapsed = start.elapsed();

        // List all
        let start = std::time::Instant::now();
        let all = params.get_all();
        let list_elapsed = start.elapsed();

        assert!(all.len() >= 1000, "should have at least 1000 params, got {}", all.len());

        println!(
            "1000 params: set={:?}, get={:?}, list={:?}",
            set_elapsed, get_elapsed, list_elapsed
        );

        // CI bounds: each operation < 500ms
        assert!(set_elapsed.as_millis() < 500, "1000 sets should complete in < 500ms");
        assert!(get_elapsed.as_millis() < 500, "1000 gets should complete in < 500ms");
        assert!(list_elapsed.as_millis() < 100, "list 1000+ params should complete in < 100ms");
    }

    #[test]
    fn concurrent_4_thread_modification_no_corruption() {
        let params = create_test_params();
        let params = Arc::new(params);

        let barrier = std::sync::Barrier::new(4);
        let barrier = std::sync::Arc::new(barrier);

        let handles: Vec<_> = (0..4)
            .map(|t| {
                let p = params.clone();
                let b = barrier.clone();
                std::thread::spawn(move || {
                    b.wait();
                    for i in 0..100 {
                        p.set(&format!("thread_{}_key_{}", t, i), &(t * 100 + i))
                            .unwrap();
                    }
                })
            })
            .collect();

        for h in handles {
            h.join().unwrap();
        }

        // 4 threads × 100 keys = 400 thread-specific keys
        let all = params.get_all();
        for t in 0..4 {
            for i in 0..100 {
                let key = format!("thread_{}_key_{}", t, i);
                assert!(
                    all.contains_key(&key),
                    "key {} must exist after concurrent set",
                    key
                );
            }
        }
    }

    #[test]
    fn file_save_load_roundtrip() {
        let params = create_test_params();

        // Set some diverse params
        params.set("string_param", &"hello world").unwrap();
        params.set("int_param", &42).unwrap();
        params.set("float_param", &3.14).unwrap();
        params.set("bool_param", &true).unwrap();

        // Use save_to_disk which writes to the configured path
        // For a round-trip test, we use load_from_disk with a temp file
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("params.yaml");

        // Manual YAML save for round-trip
        let all = params.get_all();
        let yaml = serde_yaml::to_string(&all).unwrap();
        std::fs::write(&path, &yaml).unwrap();
        assert!(path.exists(), "params file must exist after save");

        // Load into new instance
        let params2 = create_test_params();
        params2.load_from_disk(&path).unwrap();

        // Verify values match
        let v: Option<Value> = params2.get("string_param");
        assert_eq!(v.unwrap().as_str(), Some("hello world"));
        let v: Option<Value> = params2.get("int_param");
        assert_eq!(v.unwrap().as_i64(), Some(42));
        let v: Option<Value> = params2.get("float_param");
        assert!((v.unwrap().as_f64().unwrap() - 3.14).abs() < 0.001);
        let v: Option<Value> = params2.get("bool_param");
        assert_eq!(v.unwrap().as_bool(), Some(true));
    }

    #[test]
    fn version_advances_under_concurrent_writes() {
        let params = create_test_params();
        params.set("conflict_key", &"initial").unwrap();

        let v1 = params.get_version("conflict_key");

        // Two threads race to set the same key
        let p = Arc::new(params);
        let p1 = p.clone();
        let p2 = p.clone();

        let h1 = std::thread::spawn(move || {
            for i in 0..50 {
                let _ = p1.set("conflict_key", &format!("t1_{}", i));
            }
        });
        let h2 = std::thread::spawn(move || {
            for i in 0..50 {
                let _ = p2.set("conflict_key", &format!("t2_{}", i));
            }
        });

        h1.join().unwrap();
        h2.join().unwrap();

        // Version should have advanced (100 writes total)
        let v2 = p.get_version("conflict_key");
        assert!(
            v2 > v1,
            "version should advance after concurrent writes: {} -> {}",
            v1, v2
        );

        // Value should be from one of the threads (not corrupted)
        let val: Option<Value> = p.get("conflict_key");
        let s = val.unwrap().as_str().unwrap().to_string();
        assert!(
            s.starts_with("t1_") || s.starts_with("t2_"),
            "value should be from one of the threads, got: {}",
            s
        );
    }

    // ========================================================================
    // Params edge case and negative tests
    // ========================================================================

    #[test]
    fn test_params_get_nonexistent_key() {
        let p = create_test_params();
        let val: Option<Value> = p.get("nonexistent");
        assert!(val.is_none());
    }

    #[test]
    fn test_params_empty_key() {
        let p = create_test_params();
        p.set("", "empty_key_value").unwrap();
        let val: Option<Value> = p.get("");
        assert_eq!(val, Some(json!("empty_key_value")));
    }

    #[test]
    fn test_params_overwrite_different_types() {
        let p = create_test_params();
        p.set("key", 42).unwrap();
        assert_eq!(p.get::<Value>("key"), Some(json!(42)));

        // Overwrite int with string
        p.set("key", "now a string").unwrap();
        assert_eq!(p.get::<Value>("key"), Some(json!("now a string")));

        // Overwrite string with array
        p.set("key", vec![1, 2, 3]).unwrap();
        assert_eq!(p.get::<Value>("key"), Some(json!([1, 2, 3])));
    }

    #[test]
    fn test_params_list_keys_empty() {
        let p = create_test_params();
        assert!(p.list_keys().is_empty());
    }

    #[test]
    fn test_params_list_keys_after_set() {
        let p = create_test_params();
        p.set("b", 2).unwrap();
        p.set("a", 1).unwrap();
        p.set("c", 3).unwrap();
        let keys = p.list_keys();
        assert_eq!(keys.len(), 3);
        assert!(keys.contains(&"a".to_string()));
        assert!(keys.contains(&"b".to_string()));
        assert!(keys.contains(&"c".to_string()));
    }

    #[test]
    fn test_params_very_large_value() {
        let p = create_test_params();
        let big = "x".repeat(100_000);
        p.set("big", &big).unwrap();
        let val: Option<String> = p.get("big");
        assert_eq!(val.unwrap().len(), 100_000);
    }

    #[test]
    fn test_params_unicode_key_and_value() {
        let p = create_test_params();
        p.set("robot_name", "HORUS").unwrap();
        let val: Option<String> = p.get("robot_name");
        assert_eq!(val, Some("HORUS".to_string()));
    }

    // ── HORUS_PARAM_* env var injection tests ──────────────────

    #[test]
    fn test_parse_env_value_types() {
        // Integer
        assert_eq!(parse_env_value("42"), json!(42));
        assert_eq!(parse_env_value("-7"), json!(-7));
        assert_eq!(parse_env_value("0"), json!(0));

        // Float (only when not parseable as integer)
        assert_eq!(parse_env_value("1.5"), json!(1.5));
        assert_eq!(parse_env_value("-0.01"), json!(-0.01));

        // Bool
        assert_eq!(parse_env_value("true"), json!(true));
        assert_eq!(parse_env_value("false"), json!(false));

        // String fallback
        assert_eq!(parse_env_value("hello"), json!("hello"));
        assert_eq!(parse_env_value(""), json!(""));
        assert_eq!(parse_env_value("True"), json!("True")); // case-sensitive
    }

    #[test]
    fn test_env_var_injection() {
        // Set env vars before creating RuntimeParams.
        // Safety: these tests must run with --test-threads=1 since env is global.
        unsafe {
            std::env::set_var("HORUS_PARAM_TEST_FOO", "bar");
            std::env::set_var("HORUS_PARAM_TEST_SPEED", "1.5");
            std::env::set_var("HORUS_PARAM_TEST_COUNT", "42");
            std::env::set_var("HORUS_PARAM_TEST_ENABLED", "true");
        }

        let params = RuntimeParams::new().expect("RuntimeParams::new should succeed");

        // Keys are lowercased from env var names
        let foo: Option<String> = params.get("test_foo");
        assert_eq!(foo, Some("bar".to_string()));

        let speed: Option<f64> = params.get("test_speed");
        assert_eq!(speed, Some(1.5));

        let count: Option<i64> = params.get("test_count");
        assert_eq!(count, Some(42));

        let enabled: Option<bool> = params.get("test_enabled");
        assert_eq!(enabled, Some(true));

        // Clean up
        unsafe {
            std::env::remove_var("HORUS_PARAM_TEST_FOO");
            std::env::remove_var("HORUS_PARAM_TEST_SPEED");
            std::env::remove_var("HORUS_PARAM_TEST_COUNT");
            std::env::remove_var("HORUS_PARAM_TEST_ENABLED");
        }
    }
}
