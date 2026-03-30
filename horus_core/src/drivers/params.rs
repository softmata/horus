//! Typed access to node configuration parameters.
//!
//! [`NodeParams`] wraps the `HashMap<String, toml::Value>` from a
//! `[hardware.NAME]` table in `horus.toml` and provides typed getters.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus::hardware::NodeParams;
//!
//! fn from_params(params: &NodeParams) -> HorusResult<MyNode> {
//!     let port = params.get::<String>("port")?;
//!     let baud = params.get_or("baudrate", 115200u32);
//!     let ids = params.get::<Vec<u8>>("servo_ids")?;
//!     // ...
//! }
//! ```

use std::collections::HashMap;

use crate::error::{ConfigError, HorusResult};

/// Typed access to node config params from `horus.toml`.
///
/// Wraps the flattened params HashMap from a `[hardware.NAME]` table.
/// Provides `get::<T>()` for required params and `get_or()` for optional
/// params with defaults.
#[derive(Debug, Clone)]
pub struct NodeParams {
    params: HashMap<String, toml::Value>,
}

/// Backward-compatibility alias.
pub type DriverParams = NodeParams;

impl NodeParams {
    /// Create from a raw params map.
    pub fn new(params: HashMap<String, toml::Value>) -> Self {
        Self { params }
    }

    /// Create empty params (for testing or nodes with no config).
    pub fn empty() -> Self {
        Self {
            params: HashMap::new(),
        }
    }

    /// Get a required param, converting from TOML value to the target type.
    ///
    /// Returns an error if the key is missing or the value can't be converted.
    ///
    /// # Example
    /// ```rust,ignore
    /// let port: String = params.get("port")?;
    /// let baud: u32 = params.get("baudrate")?;
    /// ```
    pub fn get<T: FromToml>(&self, key: &str) -> HorusResult<T> {
        let value = self.params.get(key).ok_or_else(|| {
            ConfigError::Other(format!("driver param '{}': required but not found", key))
        })?;
        T::from_toml(value)
            .map_err(|e| ConfigError::Other(format!("driver param '{}': {}", key, e)).into())
    }

    /// Get an optional param with a default value.
    ///
    /// Returns the default if the key is missing or conversion fails.
    ///
    /// # Example
    /// ```rust,ignore
    /// let timeout: u32 = params.get_or("timeout_ms", 1000);
    /// ```
    pub fn get_or<T: FromToml>(&self, key: &str, default: T) -> T {
        self.params
            .get(key)
            .and_then(|v| T::from_toml(v).ok())
            .unwrap_or(default)
    }

    /// Check if a param key exists.
    pub fn has(&self, key: &str) -> bool {
        self.params.contains_key(key)
    }

    /// Iterate over all param keys.
    pub fn keys(&self) -> impl Iterator<Item = &str> {
        self.params.keys().map(|s| s.as_str())
    }

    /// Number of params.
    pub fn len(&self) -> usize {
        self.params.len()
    }

    /// Whether there are no params.
    pub fn is_empty(&self) -> bool {
        self.params.is_empty()
    }

    /// Get the raw TOML value for a key.
    pub fn raw(&self, key: &str) -> Option<&toml::Value> {
        self.params.get(key)
    }
}

/// Trait for converting `toml::Value` to a Rust type.
///
/// Implemented for common types used in driver configs: strings, integers,
/// floats, booleans, and arrays.
pub trait FromToml: Sized {
    /// Convert a TOML value to this type.
    fn from_toml(value: &toml::Value) -> HorusResult<Self>;
}

impl FromToml for String {
    fn from_toml(value: &toml::Value) -> HorusResult<Self> {
        value.as_str().map(|s| s.to_string()).ok_or_else(|| {
            ConfigError::Other(format!("expected string, got {}", value.type_str())).into()
        })
    }
}

impl FromToml for bool {
    fn from_toml(value: &toml::Value) -> HorusResult<Self> {
        value.as_bool().ok_or_else(|| {
            ConfigError::Other(format!("expected bool, got {}", value.type_str())).into()
        })
    }
}

impl FromToml for i64 {
    fn from_toml(value: &toml::Value) -> HorusResult<Self> {
        value.as_integer().ok_or_else(|| {
            ConfigError::Other(format!("expected integer, got {}", value.type_str())).into()
        })
    }
}

impl FromToml for i32 {
    fn from_toml(value: &toml::Value) -> HorusResult<Self> {
        let v = i64::from_toml(value)?;
        i32::try_from(v)
            .map_err(|_| ConfigError::Other(format!("integer {} out of i32 range", v)).into())
    }
}

impl FromToml for u64 {
    fn from_toml(value: &toml::Value) -> HorusResult<Self> {
        let v = i64::from_toml(value)?;
        u64::try_from(v).map_err(|_| {
            ConfigError::Other(format!("integer {} is negative, expected unsigned", v)).into()
        })
    }
}

impl FromToml for u32 {
    fn from_toml(value: &toml::Value) -> HorusResult<Self> {
        let v = i64::from_toml(value)?;
        u32::try_from(v)
            .map_err(|_| ConfigError::Other(format!("integer {} out of u32 range", v)).into())
    }
}

impl FromToml for u8 {
    fn from_toml(value: &toml::Value) -> HorusResult<Self> {
        let v = i64::from_toml(value)?;
        u8::try_from(v).map_err(|_| {
            ConfigError::Other(format!("integer {} out of u8 range (0-255)", v)).into()
        })
    }
}

impl FromToml for f64 {
    fn from_toml(value: &toml::Value) -> HorusResult<Self> {
        // Try float first, then integer (TOML 1000 is integer, 1000.0 is float)
        value
            .as_float()
            .or_else(|| value.as_integer().map(|i| i as f64))
            .ok_or_else(|| {
                ConfigError::Other(format!("expected number, got {}", value.type_str())).into()
            })
    }
}

impl FromToml for f32 {
    fn from_toml(value: &toml::Value) -> HorusResult<Self> {
        f64::from_toml(value).map(|v| v as f32)
    }
}

impl<T: FromToml> FromToml for Vec<T> {
    fn from_toml(value: &toml::Value) -> HorusResult<Self> {
        let arr = value.as_array().ok_or_else(|| {
            ConfigError::Other(format!("expected array, got {}", value.type_str()))
        })?;
        arr.iter()
            .enumerate()
            .map(|(i, v)| {
                T::from_toml(v)
                    .map_err(|e| ConfigError::Other(format!("array element [{}]: {}", i, e)).into())
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_params(pairs: &[(&str, toml::Value)]) -> DriverParams {
        let mut map = HashMap::new();
        for (k, v) in pairs {
            map.insert(k.to_string(), v.clone());
        }
        DriverParams::new(map)
    }

    #[test]
    fn get_string() {
        let params = make_params(&[("port", toml::Value::String("/dev/ttyUSB0".into()))]);
        let port: String = params.get("port").unwrap();
        assert_eq!(port, "/dev/ttyUSB0");
    }

    #[test]
    fn get_u32() {
        let params = make_params(&[("baudrate", toml::Value::Integer(1_000_000))]);
        let baud: u32 = params.get("baudrate").unwrap();
        assert_eq!(baud, 1_000_000);
    }

    #[test]
    fn get_i32() {
        let params = make_params(&[("offset", toml::Value::Integer(-42))]);
        let offset: i32 = params.get("offset").unwrap();
        assert_eq!(offset, -42);
    }

    #[test]
    fn get_u64() {
        let params = make_params(&[("big", toml::Value::Integer(5_000_000_000))]);
        let big: u64 = params.get("big").unwrap();
        assert_eq!(big, 5_000_000_000);
    }

    #[test]
    fn get_f64() {
        let params = make_params(&[("gain", toml::Value::Float(2.75))]);
        let gain: f64 = params.get("gain").unwrap();
        assert!((gain - 2.75).abs() < 1e-10);
    }

    #[test]
    fn get_f64_from_integer() {
        // TOML integers should be convertible to f64
        let params = make_params(&[("rate", toml::Value::Integer(100))]);
        let rate: f64 = params.get("rate").unwrap();
        assert!((rate - 100.0).abs() < 1e-10);
    }

    #[test]
    fn get_bool() {
        let params = make_params(&[("enabled", toml::Value::Boolean(true))]);
        let enabled: bool = params.get("enabled").unwrap();
        assert!(enabled);
    }

    #[test]
    fn get_vec_u8() {
        let ids = vec![
            toml::Value::Integer(1),
            toml::Value::Integer(2),
            toml::Value::Integer(3),
        ];
        let params = make_params(&[("servo_ids", toml::Value::Array(ids))]);
        let servo_ids: Vec<u8> = params.get("servo_ids").unwrap();
        assert_eq!(servo_ids, vec![1u8, 2, 3]);
    }

    #[test]
    fn get_vec_string() {
        let topics = vec![
            toml::Value::String("imu".into()),
            toml::Value::String("gps".into()),
        ];
        let params = make_params(&[("topics", toml::Value::Array(topics))]);
        let topics: Vec<String> = params.get("topics").unwrap();
        assert_eq!(topics, vec!["imu", "gps"]);
    }

    #[test]
    fn get_or_returns_value_when_present() {
        let params = make_params(&[("timeout", toml::Value::Integer(500))]);
        let timeout: u32 = params.get_or("timeout", 1000);
        assert_eq!(timeout, 500);
    }

    #[test]
    fn get_or_returns_default_when_missing() {
        let params = make_params(&[]);
        let timeout: u32 = params.get_or("timeout", 1000);
        assert_eq!(timeout, 1000);
    }

    #[test]
    fn get_or_returns_default_on_type_mismatch() {
        let params = make_params(&[("timeout", toml::Value::String("not a number".into()))]);
        let timeout: u32 = params.get_or("timeout", 1000);
        assert_eq!(timeout, 1000);
    }

    #[test]
    fn get_missing_key_errors() {
        let params = make_params(&[]);
        let result = params.get::<String>("nonexistent");
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("nonexistent"),
            "error should mention key: {}",
            err
        );
        assert!(
            err.contains("required"),
            "error should mention required: {}",
            err
        );
    }

    #[test]
    fn get_type_mismatch_errors() {
        let params = make_params(&[("port", toml::Value::Integer(42))]);
        let result = params.get::<String>("port");
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("string"),
            "error should mention expected type: {}",
            err
        );
    }

    #[test]
    fn has_key() {
        let params = make_params(&[("port", toml::Value::String("/dev/ttyUSB0".into()))]);
        assert!(params.has("port"));
        assert!(!params.has("missing"));
    }

    #[test]
    fn keys_iterator() {
        let params = make_params(&[
            ("a", toml::Value::Integer(1)),
            ("b", toml::Value::Integer(2)),
            ("c", toml::Value::Integer(3)),
        ]);
        let mut keys: Vec<&str> = params.keys().collect();
        keys.sort();
        assert_eq!(keys, vec!["a", "b", "c"]);
    }

    #[test]
    fn empty_params() {
        let params = DriverParams::empty();
        assert!(params.is_empty());
        assert_eq!(params.len(), 0);
        assert!(!params.has("anything"));
    }

    #[test]
    fn len_and_is_empty() {
        let params = make_params(&[
            ("a", toml::Value::Integer(1)),
            ("b", toml::Value::Integer(2)),
        ]);
        assert_eq!(params.len(), 2);
        assert!(!params.is_empty());
    }

    #[test]
    fn raw_returns_toml_value() {
        let params = make_params(&[("port", toml::Value::String("/dev/ttyUSB0".into()))]);
        let raw = params.raw("port").unwrap();
        assert_eq!(raw.as_str(), Some("/dev/ttyUSB0"));
        assert!(params.raw("missing").is_none());
    }

    #[test]
    fn u8_out_of_range_errors() {
        let params = make_params(&[("val", toml::Value::Integer(300))]);
        let result = params.get::<u8>("val");
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("u8 range"));
    }

    #[test]
    fn negative_u32_errors() {
        let params = make_params(&[("val", toml::Value::Integer(-1))]);
        let result = params.get::<u32>("val");
        assert!(result.is_err());
    }
}
