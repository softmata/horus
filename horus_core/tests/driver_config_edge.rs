//! Edge-case tests for hardware configuration: type coercion, empty configs,
//! and unknown node types.

use horus_core::drivers::params::NodeParams;
use std::collections::HashMap;

// ============================================================================
// Helper
// ============================================================================

fn make_params(pairs: &[(&str, toml::Value)]) -> NodeParams {
    let mut map = HashMap::new();
    for (k, v) in pairs {
        map.insert(k.to_string(), v.clone());
    }
    NodeParams::new(map)
}

// ============================================================================
// Test 1: NodeParams type coercion between TOML types
// ============================================================================

#[test]
fn test_node_params_type_coercion_int_to_f64() {
    // TOML integer (e.g., `rate = 100`) should coerce to f64 via FromToml.
    let params = make_params(&[("rate", toml::Value::Integer(100))]);
    let rate: f64 = params.get("rate").unwrap();
    assert!((rate - 100.0).abs() < 1e-10, "i64 -> f64 coercion failed");
}

#[test]
fn test_node_params_type_coercion_int_to_f32() {
    // TOML integer should also coerce to f32.
    let params = make_params(&[("gain", toml::Value::Integer(42))]);
    let gain: f32 = params.get("gain").unwrap();
    assert!((gain - 42.0).abs() < 1e-5, "i64 -> f32 coercion failed");
}

#[test]
fn test_node_params_type_coercion_float_to_f32() {
    // TOML float to f32 (lossy but valid via FromToml).
    let params = make_params(&[("val", toml::Value::Float(2.5))]);
    let val: f32 = params.get("val").unwrap();
    assert!((val - 2.5_f32).abs() < 1e-4, "f64 -> f32 coercion failed");
}

#[test]
fn test_node_params_type_coercion_float_no_cross_to_int() {
    // TOML float should NOT coerce to integer types (prevent silent truncation).
    let params = make_params(&[("count", toml::Value::Float(3.5))]);
    let result = params.get::<u32>("count");
    assert!(result.is_err(), "float -> u32 should NOT coerce");
}

#[test]
fn test_node_params_type_coercion_string_no_cross_to_int() {
    // TOML string should NOT coerce to integer types.
    let params = make_params(&[("val", toml::Value::String("42".into()))]);
    let result = params.get::<u32>("val");
    assert!(result.is_err(), "string -> u32 should NOT coerce");
}

#[test]
fn test_node_params_type_coercion_bool_no_cross_to_int() {
    // TOML boolean should NOT coerce to integer.
    let params = make_params(&[("val", toml::Value::Boolean(true))]);
    let result = params.get::<u32>("val");
    assert!(result.is_err(), "bool -> u32 should NOT coerce");
}

// ============================================================================
// Test 2: Empty config
// ============================================================================

#[test]
fn test_empty_node_params() {
    let params = NodeParams::empty();
    assert!(params.is_empty());
    assert_eq!(params.len(), 0);
    assert!(!params.has("anything"));
}

#[test]
fn test_empty_params_get_errors() {
    let params = NodeParams::empty();
    assert!(params.get::<String>("port").is_err());
    assert!(params.get::<u32>("baudrate").is_err());
    assert!(params.get::<f64>("rate").is_err());
    assert!(params.get::<bool>("enabled").is_err());
}

#[test]
fn test_empty_params_get_or_defaults() {
    let params = NodeParams::empty();
    assert_eq!(params.get_or("port", "default".to_string()), "default");
    assert_eq!(params.get_or("baud", 115200u32), 115200);
    assert_eq!(params.get_or("enabled", true), true);
}

// ============================================================================
// Test 3: Integer range edge cases
// ============================================================================

#[test]
fn test_u8_max_value() {
    let params = make_params(&[("val", toml::Value::Integer(255))]);
    let val: u8 = params.get("val").unwrap();
    assert_eq!(val, 255);
}

#[test]
fn test_u8_overflow() {
    let params = make_params(&[("val", toml::Value::Integer(256))]);
    let result = params.get::<u8>("val");
    assert!(result.is_err());
    assert!(result.unwrap_err().to_string().contains("u8 range"));
}

#[test]
fn test_u32_negative() {
    let params = make_params(&[("val", toml::Value::Integer(-1))]);
    let result = params.get::<u32>("val");
    assert!(result.is_err());
}

#[test]
fn test_i32_overflow() {
    let params = make_params(&[("val", toml::Value::Integer(i64::from(i32::MAX) + 1))]);
    let result = params.get::<i32>("val");
    assert!(result.is_err());
    assert!(result.unwrap_err().to_string().contains("i32 range"));
}

// ============================================================================
// Test 4: Array params
// ============================================================================

#[test]
fn test_vec_u8() {
    let ids = vec![
        toml::Value::Integer(1),
        toml::Value::Integer(2),
        toml::Value::Integer(255),
    ];
    let params = make_params(&[("servo_ids", toml::Value::Array(ids))]);
    let servo_ids: Vec<u8> = params.get("servo_ids").unwrap();
    assert_eq!(servo_ids, vec![1u8, 2, 255]);
}

#[test]
fn test_vec_string() {
    let topics = vec![
        toml::Value::String("imu".into()),
        toml::Value::String("gps".into()),
    ];
    let params = make_params(&[("topics", toml::Value::Array(topics))]);
    let topics: Vec<String> = params.get("topics").unwrap();
    assert_eq!(topics, vec!["imu", "gps"]);
}

#[test]
fn test_vec_with_bad_element() {
    let arr = vec![
        toml::Value::Integer(1),
        toml::Value::String("bad".into()),
        toml::Value::Integer(3),
    ];
    let params = make_params(&[("vals", toml::Value::Array(arr))]);
    let result = params.get::<Vec<u8>>("vals");
    assert!(result.is_err());
    assert!(result.unwrap_err().to_string().contains("array element"));
}

// ============================================================================
// Test 5: raw() access
// ============================================================================

#[test]
fn test_raw_value() {
    let params = make_params(&[("port", toml::Value::String("/dev/ttyUSB0".into()))]);
    let raw = params.raw("port").unwrap();
    assert_eq!(raw.as_str(), Some("/dev/ttyUSB0"));
    assert!(params.raw("missing").is_none());
}

// ============================================================================
// Test 6: DriverParams backward-compat alias
// ============================================================================

#[test]
fn test_driver_params_alias() {
    use horus_core::drivers::params::DriverParams;
    let params = DriverParams::empty();
    assert!(params.is_empty());
}
