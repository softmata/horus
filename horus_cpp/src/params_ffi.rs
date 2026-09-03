//! FFI wrappers for RuntimeParams (dynamic configuration).
//!
//! Params use JSON values for type-erased get/set across FFI.

use horus_core::params::RuntimeParams;

/// Opaque RuntimeParams wrapper.
pub struct FfiParams {
    inner: RuntimeParams,
}

/// Create a new params store. Returns `None` if `.horus/config/params.yaml`
/// exists but cannot be read or parsed.
///
/// This used to be `RuntimeParams::new().unwrap_or_default()`, the only one of
/// the 13 `RuntimeParams` construction sites in the workspace that swallowed
/// the error — `horus_manager` (including the `horus run` launcher) and
/// `horus_py` all propagate it. `RuntimeParams::default()` scopes its fallback
/// to "a caller that reached Default has no way to handle the error", which is
/// not true here: the C ABI can return null. The cost of the escape hatch was
/// that a typo in params.yaml restored the built-in `max_speed = 1.0` and
/// `emergency_stop_distance = 0.3` under the operator's name, with no way for
/// a C++ caller to tell that had happened.
///
/// The reason goes to stderr because the C ABI carries no error channel; the
/// `horus::Params` constructor turns the null into a `horus::Error`.
pub fn params_new() -> Option<Box<FfiParams>> {
    match RuntimeParams::new() {
        Ok(inner) => Some(Box::new(FfiParams { inner })),
        Err(e) => {
            horus_core::terminal::eprint_line(&format!("[PARAMS] {e}"));
            None
        }
    }
}

/// Get a parameter value as JSON string. Returns None if not found.
pub fn params_get(params: &FfiParams, key: &str) -> Option<String> {
    params
        .inner
        .get::<serde_json::Value>(key)
        .map(|v| v.to_string())
}

/// Set a parameter from a JSON value string.
pub fn params_set(params: &FfiParams, key: &str, json_value: &str) -> Result<(), String> {
    let value: serde_json::Value =
        serde_json::from_str(json_value).map_err(|e| format!("Invalid JSON value: {}", e))?;
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
    params
        .inner
        .set(key, value.to_string())
        .map_err(|e| e.to_string())
}

/// Check if a parameter exists.
pub fn params_has(params: &FfiParams, key: &str) -> bool {
    params.inner.get::<serde_json::Value>(key).is_some()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// These tests exercise the getters and setters, not the constructor, but
    /// `params_new` is the only way to build an `FfiParams` from outside this
    /// module: `RuntimeParams` has no path-taking constructor, and its
    /// `Default` is `new().unwrap_or_else(..)`, which reads the same
    /// cwd-relative file. So they inherit its dependency on the ambient
    /// `.horus/config/params.yaml` being absent or well-formed.
    ///
    /// Deliberately NOT addressed by chdir'ing to a temp directory:
    /// `set_current_dir` is process-global and libtest runs this binary's tests
    /// on parallel threads, several of which do cwd-relative I/O — every
    /// `params_set_*` below appends to `.horus/logs/param_changes.log`. A chdir
    /// from one test thread would redirect the others' writes. The malformed
    /// file is covered instead by `tests/params_malformed_yaml.rs`, a separate
    /// binary that owns the process cwd.
    fn params() -> Box<FfiParams> {
        params_new().unwrap_or_else(|| {
            panic!(
                "params_new returned None: a malformed or unreadable \
                 .horus/config/params.yaml exists under the directory cargo ran \
                 this test from (reason on stderr above). Fix or remove that \
                 file — this test is about the accessors, not the config file."
            )
        })
    }

    #[test]
    fn create_params() {
        assert!(
            params_new().is_some(),
            "params_new refused to build a store: a malformed or unreadable \
             .horus/config/params.yaml exists under the directory cargo ran this \
             test from (reason on stderr above). The refusal itself is the \
             intended behaviour and is covered by tests/params_malformed_yaml.rs."
        );
    }

    #[test]
    fn set_and_get_f64() {
        let p = params();
        params_set_f64(&p, "max_speed", 1.5).unwrap();
        assert_eq!(params_get_f64(&p, "max_speed"), Some(1.5));
    }

    #[test]
    fn set_and_get_i64() {
        let p = params();
        params_set_i64(&p, "count", 42).unwrap();
        assert_eq!(params_get_i64(&p, "count"), Some(42));
    }

    #[test]
    fn set_and_get_bool() {
        let p = params();
        params_set_bool(&p, "enabled", true).unwrap();
        assert_eq!(params_get_bool(&p, "enabled"), Some(true));
    }

    #[test]
    fn set_and_get_string() {
        let p = params();
        params_set_string(&p, "name", "robot1").unwrap();
        assert_eq!(params_get_string(&p, "name"), Some("robot1".to_string()));
    }

    #[test]
    fn get_missing_returns_none() {
        let p = params();
        assert_eq!(params_get_f64(&p, "missing"), None);
        assert!(!params_has(&p, "missing"));
    }

    #[test]
    fn set_and_get_json() {
        let p = params();
        params_set(&p, "config", r#"{"a": 1, "b": "hello"}"#).unwrap();
        let json = params_get(&p, "config");
        assert!(json.is_some());
        let val: serde_json::Value = serde_json::from_str(&json.unwrap()).unwrap();
        assert_eq!(val["a"], 1);
        assert_eq!(val["b"], "hello");
    }

    #[test]
    fn has_returns_true_after_set() {
        let p = params();
        params_set_f64(&p, "x", 1.0).unwrap();
        assert!(params_has(&p, "x"));
    }

    #[test]
    fn invalid_json_returns_error() {
        let p = params();
        let result = params_set(&p, "bad", "not json{");
        assert!(result.is_err());
    }
}
