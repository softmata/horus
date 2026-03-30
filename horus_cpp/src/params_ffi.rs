//! FFI wrappers for RuntimeParams (dynamic configuration).
//!
//! Params use JSON values for type-erased get/set across FFI.

use horus_core::params::RuntimeParams;

/// Opaque RuntimeParams wrapper.
pub struct FfiParams {
    inner: RuntimeParams,
}

/// Create a new empty params store.
pub fn params_new() -> Box<FfiParams> {
    Box::new(FfiParams {
        inner: RuntimeParams::new().unwrap_or_default(),
    })
}

/// Get a parameter value as JSON string. Returns None if not found.
pub fn params_get(params: &FfiParams, key: &str) -> Option<String> {
    params.inner.get::<serde_json::Value>(key)
        .map(|v| v.to_string())
}

/// Set a parameter from a JSON value string.
pub fn params_set(params: &FfiParams, key: &str, json_value: &str) -> Result<(), String> {
    let value: serde_json::Value = serde_json::from_str(json_value)
        .map_err(|e| format!("Invalid JSON value: {}", e))?;
    params.inner.set(key, value).map_err(|e| e.to_string())
}

/// Get a parameter as f64 (convenience).
pub fn params_get_f64(params: &FfiParams, key: &str) -> Option<f64> {
    params.inner.get::<f64>(key)
}

/// Get a parameter as i64 (convenience).
pub fn params_get_i64(params: &FfiParams, key: &str) -> Option<i64> {
    params.inner.get::<i64>(key)
}

/// Get a parameter as bool (convenience).
pub fn params_get_bool(params: &FfiParams, key: &str) -> Option<bool> {
    params.inner.get::<bool>(key)
}

/// Get a parameter as string (convenience).
pub fn params_get_string(params: &FfiParams, key: &str) -> Option<String> {
    params.inner.get::<String>(key)
}

/// Set a parameter as f64 (convenience).
pub fn params_set_f64(params: &FfiParams, key: &str, value: f64) -> Result<(), String> {
    params.inner.set(key, value).map_err(|e| e.to_string())
}

/// Set a parameter as i64 (convenience).
pub fn params_set_i64(params: &FfiParams, key: &str, value: i64) -> Result<(), String> {
    params.inner.set(key, value).map_err(|e| e.to_string())
}

/// Set a parameter as bool (convenience).
pub fn params_set_bool(params: &FfiParams, key: &str, value: bool) -> Result<(), String> {
    params.inner.set(key, value).map_err(|e| e.to_string())
}

/// Set a parameter as string (convenience).
pub fn params_set_string(params: &FfiParams, key: &str, value: &str) -> Result<(), String> {
    params.inner.set(key, value.to_string()).map_err(|e| e.to_string())
}

/// Check if a parameter exists.
pub fn params_has(params: &FfiParams, key: &str) -> bool {
    params.inner.get::<serde_json::Value>(key).is_some()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create_params() {
        let _p = params_new();
    }

    #[test]
    fn set_and_get_f64() {
        let p = params_new();
        params_set_f64(&p, "max_speed", 1.5).unwrap();
        assert_eq!(params_get_f64(&p, "max_speed"), Some(1.5));
    }

    #[test]
    fn set_and_get_i64() {
        let p = params_new();
        params_set_i64(&p, "count", 42).unwrap();
        assert_eq!(params_get_i64(&p, "count"), Some(42));
    }

    #[test]
    fn set_and_get_bool() {
        let p = params_new();
        params_set_bool(&p, "enabled", true).unwrap();
        assert_eq!(params_get_bool(&p, "enabled"), Some(true));
    }

    #[test]
    fn set_and_get_string() {
        let p = params_new();
        params_set_string(&p, "name", "robot1").unwrap();
        assert_eq!(params_get_string(&p, "name"), Some("robot1".to_string()));
    }

    #[test]
    fn get_missing_returns_none() {
        let p = params_new();
        assert_eq!(params_get_f64(&p, "missing"), None);
        assert!(!params_has(&p, "missing"));
    }

    #[test]
    fn set_and_get_json() {
        let p = params_new();
        params_set(&p, "config", r#"{"a": 1, "b": "hello"}"#).unwrap();
        let json = params_get(&p, "config");
        assert!(json.is_some());
        let val: serde_json::Value = serde_json::from_str(&json.unwrap()).unwrap();
        assert_eq!(val["a"], 1);
        assert_eq!(val["b"], "hello");
    }

    #[test]
    fn has_returns_true_after_set() {
        let p = params_new();
        params_set_f64(&p, "x", 1.0).unwrap();
        assert!(params_has(&p, "x"));
    }

    #[test]
    fn invalid_json_returns_error() {
        let p = params_new();
        let result = params_set(&p, "bad", "not json{");
        assert!(result.is_err());
    }
}
