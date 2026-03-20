//! Edge-case tests for driver configuration: type coercion, empty configs,
//! and unknown driver types.

use horus_core::drivers::params::DriverParams;
use horus_core::drivers::{HardwareSet, DriverType};
use std::collections::HashMap;

// ============================================================================
// Helper
// ============================================================================

fn make_params(pairs: &[(&str, toml::Value)]) -> DriverParams {
    let mut map = HashMap::new();
    for (k, v) in pairs {
        map.insert(k.to_string(), v.clone());
    }
    DriverParams::new(map)
}

fn parse_toml_table(s: &str) -> toml::value::Table {
    let value: toml::Value = toml::from_str(s).unwrap();
    value.as_table().unwrap().clone()
}

// ============================================================================
// Test 1: DriverParams type coercion between TOML types
// ============================================================================

#[test]
fn test_driver_params_type_coercion_int_to_f64() {
    // TOML integer (e.g., `rate = 100`) should coerce to f64 via FromToml.
    let params = make_params(&[("rate", toml::Value::Integer(100))]);
    let rate: f64 = params.get("rate").unwrap();
    assert!((rate - 100.0).abs() < 1e-10, "i64 -> f64 coercion failed");
}

#[test]
fn test_driver_params_type_coercion_int_to_f32() {
    // TOML integer should also coerce to f32.
    let params = make_params(&[("gain", toml::Value::Integer(42))]);
    let gain: f32 = params.get("gain").unwrap();
    assert!((gain - 42.0).abs() < 1e-5, "i64 -> f32 coercion failed");
}

#[test]
fn test_driver_params_type_coercion_float_to_f32() {
    // TOML float to f32 (lossy but valid via FromToml).
    let params = make_params(&[("val", toml::Value::Float(3.14159))]);
    let val: f32 = params.get("val").unwrap();
    assert!((val - 3.14159_f32).abs() < 1e-4, "f64 -> f32 coercion failed");
}

#[test]
fn test_driver_params_string_to_int_fails() {
    // A string value should NOT coerce to u32 — this must return an error.
    let params = make_params(&[("port_num", toml::Value::String("8080".into()))]);
    let result = params.get::<u32>("port_num");
    assert!(
        result.is_err(),
        "string -> u32 should fail; TOML types are strict"
    );
}

#[test]
fn test_driver_params_float_to_int_fails() {
    // A float value should NOT coerce to u32 (TOML distinguishes int and float).
    let params = make_params(&[("count", toml::Value::Float(3.14))]);
    let result = params.get::<u32>("count");
    assert!(
        result.is_err(),
        "float -> u32 should fail; TOML does not auto-coerce float to int"
    );
}

#[test]
fn test_driver_params_int_to_string_fails() {
    // An integer should NOT coerce to String.
    let params = make_params(&[("port", toml::Value::Integer(9090))]);
    let result = params.get::<String>("port");
    assert!(
        result.is_err(),
        "int -> String should fail"
    );
}

#[test]
fn test_driver_params_bool_to_string_fails() {
    // A bool should NOT coerce to String.
    let params = make_params(&[("flag", toml::Value::Boolean(true))]);
    let result = params.get::<String>("flag");
    assert!(
        result.is_err(),
        "bool -> String should fail"
    );
}

#[test]
fn test_driver_params_int_to_bool_fails() {
    // An integer should NOT coerce to bool.
    let params = make_params(&[("flag", toml::Value::Integer(1))]);
    let result = params.get::<bool>("flag");
    assert!(
        result.is_err(),
        "int -> bool should fail"
    );
}

#[test]
fn test_driver_params_bool_to_int_fails() {
    // A bool should NOT coerce to u32.
    let params = make_params(&[("val", toml::Value::Boolean(true))]);
    let result = params.get::<u32>("val");
    assert!(
        result.is_err(),
        "bool -> u32 should fail"
    );
}

#[test]
fn test_driver_params_get_or_handles_type_mismatch() {
    // get_or should return the default on type mismatch (not error).
    let params = make_params(&[("rate", toml::Value::String("fast".into()))]);
    let rate: u32 = params.get_or("rate", 100);
    assert_eq!(rate, 100, "get_or should return default on type mismatch");
}

#[test]
fn test_driver_params_narrow_int_coercion_boundaries() {
    // i64 -> u8 boundary: 0 and 255 pass, 256 fails.
    let params_ok = make_params(&[("ch", toml::Value::Integer(255))]);
    assert_eq!(params_ok.get::<u8>("ch").unwrap(), 255);

    let params_over = make_params(&[("ch", toml::Value::Integer(256))]);
    assert!(params_over.get::<u8>("ch").is_err());

    // i64 -> i32 boundary
    let params_big = make_params(&[("val", toml::Value::Integer(i64::from(i32::MAX) + 1))]);
    assert!(params_big.get::<i32>("val").is_err());

    // Negative -> u64 fails
    let params_neg = make_params(&[("val", toml::Value::Integer(-1))]);
    assert!(params_neg.get::<u64>("val").is_err());
}

// ============================================================================
// Test 2: Empty [drivers] section should produce an empty HardwareSet
// ============================================================================

#[test]
fn test_driver_empty_config() {
    // An empty TOML table should parse to an empty HardwareSet with no errors.
    let table = toml::value::Table::new();
    let hw = HardwareSet::from_toml_table(&table).unwrap();

    assert!(hw.is_empty(), "empty config should produce empty HardwareSet");
    assert_eq!(hw.len(), 0);
    assert!(hw.list().is_empty());
    assert!(!hw.has("anything"));
    assert!(hw.params("anything").is_none());
    assert!(hw.driver_type("anything").is_none());
    assert!(hw.topic_mapping("anything").is_none());
}

#[test]
fn test_driver_empty_config_from_toml_string() {
    // A TOML file with an empty [drivers] section (simulated by parsing
    // a table that has no entries).
    let toml_str = "";
    let value: toml::Value = toml::from_str(toml_str).unwrap_or(toml::Value::Table(Default::default()));
    let table = value.as_table().cloned().unwrap_or_default();
    let hw = HardwareSet::from_toml_table(&table).unwrap();

    assert!(hw.is_empty());
    assert_eq!(hw.len(), 0);
}

#[test]
fn test_driver_empty_config_accessors_return_errors() {
    // All typed accessors on an empty HardwareSet should return errors (not panic).
    let table = toml::value::Table::new();
    let mut hw = HardwareSet::from_toml_table(&table).unwrap();

    assert!(hw.dynamixel("arm").is_err());
    assert!(hw.rplidar("lidar").is_err());
    assert!(hw.realsense("cam").is_err());
    assert!(hw.i2c("bus").is_err());
    assert!(hw.serial("port").is_err());
    assert!(hw.can("bus").is_err());
    assert!(hw.gpio("pin").is_err());
    assert!(hw.pwm("motor").is_err());
    assert!(hw.usb("dev").is_err());
    assert!(hw.webcam("cam").is_err());
    assert!(hw.input("joy").is_err());
    assert!(hw.bluetooth("ble").is_err());
    assert!(hw.net("sock").is_err());
    assert!(hw.ethercat("ec").is_err());
    assert!(hw.spi("spi").is_err());
    assert!(hw.adc("adc").is_err());
    assert!(hw.raw("anything").is_err());
    assert!(hw.node("anything").is_err());
    assert!(hw.local("anything").is_err());
    assert!(hw.package("anything").is_err());
}

// ============================================================================
// Test 3: Unknown driver type in config table (missing source key)
// ============================================================================

#[test]
fn test_driver_unknown_type_handled() {
    // A config table without 'terra', 'package', or 'node' should return
    // an error, not panic.
    let table = parse_toml_table(
        r#"
        [mystery_sensor]
        port = "/dev/ttyUSB0"
        baudrate = 115200
    "#,
    );
    let result = HardwareSet::from_toml_table(&table);
    assert!(
        result.is_err(),
        "config without terra/package/node key should return error"
    );
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("mystery_sensor"),
        "error should mention driver name; got: {}",
        err
    );
    assert!(
        err.contains("terra") && err.contains("package") && err.contains("node"),
        "error should list required keys; got: {}",
        err
    );
}

#[test]
fn test_driver_invalid_value_type_integer() {
    // A driver entry that is an integer (not table, string, or bool).
    let mut table = toml::value::Table::new();
    table.insert("bad".to_string(), toml::Value::Integer(42));
    let result = HardwareSet::from_toml_table(&table);
    assert!(
        result.is_err(),
        "integer driver value should return error, not panic"
    );
    let err = result.unwrap_err().to_string();
    assert!(err.contains("bad"), "error should mention driver name");
}

#[test]
fn test_driver_invalid_value_type_float() {
    // A driver entry that is a float.
    let mut table = toml::value::Table::new();
    table.insert("bad".to_string(), toml::Value::Float(1.23));
    let result = HardwareSet::from_toml_table(&table);
    assert!(
        result.is_err(),
        "float driver value should return error, not panic"
    );
}

#[test]
fn test_driver_invalid_value_type_array() {
    // A driver entry that is an array.
    let mut table = toml::value::Table::new();
    table.insert(
        "bad".to_string(),
        toml::Value::Array(vec![toml::Value::Integer(1)]),
    );
    let result = HardwareSet::from_toml_table(&table);
    assert!(
        result.is_err(),
        "array driver value should return error, not panic"
    );
}

#[test]
fn test_driver_unknown_type_among_valid_drivers() {
    // A mix of valid and invalid drivers — the invalid one should cause the
    // entire parse to fail (fail-fast behavior).
    let table = parse_toml_table(
        r#"
        [good_arm]
        terra = "dynamixel"
        port = "/dev/ttyUSB0"

        [broken_sensor]
        port = "/dev/ttyUSB1"
        rate = 100
    "#,
    );
    let result = HardwareSet::from_toml_table(&table);
    assert!(
        result.is_err(),
        "config with one invalid driver should fail"
    );
    let err = result.unwrap_err().to_string();
    assert!(
        err.contains("broken_sensor"),
        "error should mention the broken driver; got: {}",
        err
    );
}

#[test]
fn test_driver_legacy_string_and_bool_accepted() {
    // Verify that legacy string and bool values are still accepted
    // (they produce DriverType::Legacy, not errors).
    let table = parse_toml_table(
        r#"
        camera = "opencv"
        gps = true
        imu = false
    "#,
    );
    let hw = HardwareSet::from_toml_table(&table).unwrap();
    assert_eq!(hw.len(), 3);

    assert!(matches!(hw.driver_type("camera"), Some(DriverType::Legacy)));
    assert!(matches!(hw.driver_type("gps"), Some(DriverType::Legacy)));
    assert!(matches!(hw.driver_type("imu"), Some(DriverType::Legacy)));
}

#[test]
fn test_driver_empty_table_with_source_key_accepted() {
    // A config table with a source key but no extra params should work.
    let table = parse_toml_table(
        r#"
        [minimal]
        terra = "serial"
    "#,
    );
    let hw = HardwareSet::from_toml_table(&table).unwrap();
    assert!(hw.has("minimal"));
    assert!(matches!(
        hw.driver_type("minimal"),
        Some(DriverType::Terra(t)) if t == "serial"
    ));
    let params = hw.params("minimal").unwrap();
    assert!(params.is_empty(), "no extra params beyond the source key");
}
