//! Parameter management commands
//!
//! Provides CLI commands for getting, setting, and listing runtime parameters.
//!
//! Parameters are stored in `.horus/config/params.yaml` and can be modified
//! at runtime using these commands.

use colored::*;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use horus_core::params::RuntimeParams;
use horus_core::serde_json::{self, Value};
use horus_core::serde_yaml;
use std::path::Path;

/// List all parameters
pub fn list_params(verbose: bool, json: bool) -> HorusResult<()> {
    let params = RuntimeParams::new()?;
    let all_params = params.get_all();

    if json {
        let items: Vec<_> = all_params
            .iter()
            .map(|(key, value)| {
                serde_json::json!({
                    "name": key,
                    "value": value,
                    "type": value_type(value)
                })
            })
            .collect();
        let output = serde_json::json!({
            "count": items.len(),
            "items": items
        });
        println!("{}", serde_json::to_string_pretty(&output)?);
        return Ok(());
    }

    if all_params.is_empty() {
        println!("{}", "No parameters found.".yellow());
        println!(
            "  {} Use 'horus param set <key> <value>' to add parameters",
            "Tip:".dimmed()
        );
        return Ok(());
    }

    println!("{}", "Parameters:".green().bold());
    println!();

    if verbose {
        for (key, value) in &all_params {
            println!("  {} {}", "Key:".cyan(), key.white().bold());
            println!("    {} {}", "Value:".dimmed(), format_value(value));
            println!("    {} {}", "Type:".dimmed(), value_type(value));
            if let Some(meta) = params.get_metadata(key) {
                if let Some(ref desc) = meta.description {
                    println!("    {} {}", "Description:".dimmed(), desc);
                }
                if let Some(ref unit) = meta.unit {
                    println!("    {} {}", "Unit:".dimmed(), unit);
                }
                if meta.read_only() {
                    println!("    {} {}", "Read-only:".dimmed(), "Yes".yellow());
                }
            }
            println!();
        }
    } else {
        // Compact table view
        println!(
            "  {:<35} {:>15} {:>10}",
            "KEY".dimmed(),
            "VALUE".dimmed(),
            "TYPE".dimmed()
        );
        println!("  {}", "-".repeat(64).dimmed());

        for (key, value) in &all_params {
            let value_str = format_value_compact(value);
            let type_str = value_type(value);
            println!("  {:<35} {:>15} {:>10}", key, value_str, type_str);
        }
    }

    println!();
    println!("  {} {} parameter(s)", "Total:".dimmed(), all_params.len());
    println!();
    println!(
        "  {} Stored in: {}",
        "Location:".dimmed(),
        ".horus/config/params.yaml".cyan()
    );

    Ok(())
}

/// Get a single parameter value
pub fn get_param(key: &str, json: bool) -> HorusResult<()> {
    let params = RuntimeParams::new()?;

    if let Some(value) = params.get_all().get(key) {
        if json {
            let output = serde_json::json!({
                "key": key,
                "value": value,
                "type": value_type(value)
            });
            println!("{}", serde_json::to_string_pretty(&output)?);
        } else {
            println!("{}: {}", key.cyan(), format_value(value).white());
        }
        Ok(())
    } else {
        Err(HorusError::Config(ConfigError::Other(format!(
            "Parameter '{}' not found. Use 'horus param list' to see available parameters.",
            key
        ))))
    }
}

/// Set a parameter value
pub fn set_param(key: &str, value: &str) -> HorusResult<()> {
    let params = RuntimeParams::new()?;

    // Try to parse as JSON first (for complex types)
    let json_value: Value = if let Ok(parsed) = serde_json::from_str(value) {
        parsed
    } else {
        // Try to infer type from string
        if value == "true" {
            Value::Bool(true)
        } else if value == "false" {
            Value::Bool(false)
        } else if let Ok(num) = value.parse::<i64>() {
            Value::Number(num.into())
        } else if let Ok(num) = value.parse::<f64>() {
            Value::Number(serde_json::Number::from_f64(num).unwrap_or_else(|| 0.into()))
        } else {
            Value::String(value.to_string())
        }
    };

    // Get old value for display
    let old_value = params.get_all().get(key).cloned();

    // Set the new value
    params.set(key, &json_value)?;

    // Save to disk
    params.save_to_disk()?;

    // Display result
    if let Some(old) = old_value {
        println!(
            "{} Updated {} {} -> {}",
            "".green(),
            key.cyan(),
            format_value_compact(&old).dimmed(),
            format_value_compact(&json_value).white()
        );
    } else {
        println!(
            "{} Set {} = {}",
            "".green(),
            key.cyan(),
            format_value_compact(&json_value).white()
        );
    }

    Ok(())
}

/// Delete a parameter
pub fn delete_param(key: &str) -> HorusResult<()> {
    let params = RuntimeParams::new()?;

    if params.has(key) {
        let old_value = params.remove(key);
        params.save_to_disk()?;

        if let Some(val) = old_value {
            println!(
                "{} Deleted {} (was: {})",
                "".green(),
                key.cyan(),
                format_value_compact(&val).dimmed()
            );
        }
        Ok(())
    } else {
        Err(HorusError::Config(ConfigError::Other(format!(
            "Parameter '{}' not found.",
            key
        ))))
    }
}

/// Reset all parameters to defaults
pub fn reset_params(force: bool) -> HorusResult<()> {
    if !force {
        println!("{}", "This will reset all parameters to defaults.".yellow());
        println!("  Use --force to confirm.");
        return Ok(());
    }

    let params = RuntimeParams::new()?;
    params.reset()?;
    params.save_to_disk()?;

    println!("{} Reset all parameters to defaults", "".green());
    Ok(())
}

/// Load parameters from a YAML file
pub fn load_params(file: &Path) -> HorusResult<()> {
    if !file.exists() {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "File not found: {}",
            file.display()
        ))));
    }

    let params = RuntimeParams::new()?;
    params.load_from_disk(file)?;
    params.save_to_disk()?;

    let count = params.get_all().len();
    println!(
        "{} Loaded {} parameters from {}",
        "".green(),
        count,
        file.display()
    );

    Ok(())
}

/// Save parameters to a YAML file
pub fn save_params(file: Option<&Path>) -> HorusResult<()> {
    let params = RuntimeParams::new()?;

    if let Some(path) = file {
        // Create parent directories if needed
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }

        let all_params = params.get_all();
        let yaml = serde_yaml::to_string(&all_params)?;
        std::fs::write(path, yaml)?;

        println!(
            "{} Saved {} parameters to {}",
            "".green(),
            all_params.len(),
            path.display()
        );
    } else {
        params.save_to_disk()?;
        println!(
            "{} Saved parameters to .horus/config/params.yaml",
            "".green()
        );
    }

    Ok(())
}

/// Dump all parameters as YAML to stdout
pub fn dump_params() -> HorusResult<()> {
    let params = RuntimeParams::new()?;
    let all_params = params.get_all();
    let yaml = serde_yaml::to_string(&all_params)?;
    println!("{}", yaml);
    Ok(())
}

// Helper functions

fn format_value(value: &Value) -> String {
    match value {
        Value::String(s) => format!("\"{}\"", s),
        Value::Number(n) => n.to_string(),
        Value::Bool(b) => b.to_string(),
        Value::Array(arr) => format!("[{} items]", arr.len()),
        Value::Object(obj) => format!("{{{}}} keys", obj.len()),
        Value::Null => "null".to_string(),
    }
}

fn format_value_compact(value: &Value) -> String {
    match value {
        Value::String(s) => {
            if s.len() > 20 {
                format!("\"{}\"", crate::cli_output::safe_truncate(s, 20))
            } else {
                format!("\"{}\"", s)
            }
        }
        Value::Number(n) => n.to_string(),
        Value::Bool(b) => b.to_string(),
        Value::Array(arr) => format!("[{}]", arr.len()),
        Value::Object(obj) => format!("{{{}}}", obj.len()),
        Value::Null => "null".to_string(),
    }
}

fn value_type(value: &Value) -> &'static str {
    match value {
        Value::String(_) => "string",
        Value::Number(n) => {
            if n.is_f64() {
                "float"
            } else {
                "int"
            }
        }
        Value::Bool(_) => "bool",
        Value::Array(_) => "array",
        Value::Object(_) => "object",
        Value::Null => "null",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_format_value() {
        assert_eq!(format_value(&Value::Bool(true)), "true");
        assert_eq!(format_value(&Value::Number(42.into())), "42");
        assert_eq!(format_value(&Value::String("test".into())), "\"test\"");
    }

    #[test]
    fn test_format_value_null() {
        assert_eq!(format_value(&Value::Null), "null");
    }

    #[test]
    fn test_format_value_array() {
        let arr = Value::Array(vec![Value::Bool(true), Value::Bool(false)]);
        assert_eq!(format_value(&arr), "[2 items]");
    }

    #[test]
    fn test_format_value_object() {
        let mut map = serde_json::Map::new();
        map.insert("a".into(), Value::Number(1.into()));
        let obj = Value::Object(map);
        assert_eq!(format_value(&obj), "{1} keys");
    }

    #[test]
    fn test_format_value_compact_long_string() {
        let long_str = Value::String("a".repeat(50));
        let result = format_value_compact(&long_str);
        // Should be truncated (plus quotes)
        assert!(result.len() < 50 + 2);
    }

    #[test]
    fn test_format_value_compact_short_string() {
        let short = Value::String("hi".to_string());
        assert_eq!(format_value_compact(&short), "\"hi\"");
    }

    #[test]
    fn test_value_type() {
        assert_eq!(value_type(&Value::Bool(true)), "bool");
        assert_eq!(value_type(&Value::Number(42.into())), "int");
        assert_eq!(value_type(&Value::String("test".into())), "string");
    }

    #[test]
    fn test_value_type_float() {
        let float = Value::Number(serde_json::Number::from_f64(3.15).unwrap());
        assert_eq!(value_type(&float), "float");
    }

    #[test]
    fn test_value_type_null() {
        assert_eq!(value_type(&Value::Null), "null");
    }

    #[test]
    fn test_value_type_array() {
        assert_eq!(value_type(&Value::Array(vec![])), "array");
    }

    #[test]
    fn test_value_type_object() {
        assert_eq!(value_type(&Value::Object(serde_json::Map::new())), "object");
    }

    #[test]
    fn test_format_value_compact_array_and_object() {
        let arr = Value::Array(vec![Value::Null; 3]);
        assert_eq!(format_value_compact(&arr), "[3]");

        let mut map = serde_json::Map::new();
        map.insert("x".into(), Value::Null);
        map.insert("y".into(), Value::Null);
        assert_eq!(format_value_compact(&Value::Object(map)), "{2}");
    }

    // ── Battle tests: format_value edge cases ─────────────────────────────

    #[test]
    fn format_value_empty_string() {
        assert_eq!(format_value(&Value::String(String::new())), "\"\"");
    }

    #[test]
    fn format_value_empty_array() {
        assert_eq!(format_value(&Value::Array(vec![])), "[0 items]");
    }

    #[test]
    fn format_value_empty_object() {
        assert_eq!(
            format_value(&Value::Object(serde_json::Map::new())),
            "{0} keys"
        );
    }

    #[test]
    fn format_value_negative_number() {
        assert_eq!(format_value(&Value::Number((-42).into())), "-42");
    }

    #[test]
    fn format_value_large_number() {
        let big = Value::Number(i64::MAX.into());
        let s = format_value(&big);
        assert_eq!(s, i64::MAX.to_string());
    }

    #[test]
    fn format_value_float_zero() {
        let zero = Value::Number(serde_json::Number::from_f64(0.0).unwrap());
        assert_eq!(format_value(&zero), "0.0");
    }

    #[test]
    fn format_value_nested_array() {
        let nested = Value::Array(vec![
            Value::Array(vec![Value::Null]),
            Value::Array(vec![Value::Null, Value::Null]),
        ]);
        // format_value only counts top-level items
        assert_eq!(format_value(&nested), "[2 items]");
    }

    #[test]
    fn format_value_string_with_special_chars() {
        let val = Value::String("hello \"world\" \n\t".to_string());
        let s = format_value(&val);
        assert!(s.starts_with('"'), "Should start with quote");
        assert!(s.ends_with('"'), "Should end with quote");
        assert!(s.contains("hello"), "Should contain content");
    }

    // ── Battle tests: format_value_compact edge cases ─────────────────────

    #[test]
    fn format_value_compact_empty_string() {
        assert_eq!(format_value_compact(&Value::String(String::new())), "\"\"");
    }

    #[test]
    fn format_value_compact_exactly_20_chars() {
        let s = Value::String("a".repeat(20));
        let result = format_value_compact(&s);
        // Exactly 20 chars should NOT be truncated
        assert_eq!(result, format!("\"{}\"", "a".repeat(20)));
    }

    #[test]
    fn format_value_compact_21_chars_truncates() {
        let s = Value::String("a".repeat(21));
        let result = format_value_compact(&s);
        // Should be truncated — the result should be shorter than the full 21+2 chars
        assert!(result.len() < 23, "Should be truncated, got: '{}'", result);
        assert!(
            result.contains("..."),
            "Should contain ellipsis, got: '{}'",
            result
        );
    }

    #[test]
    fn format_value_compact_null() {
        assert_eq!(format_value_compact(&Value::Null), "null");
    }

    #[test]
    fn format_value_compact_bool_true() {
        assert_eq!(format_value_compact(&Value::Bool(true)), "true");
    }

    #[test]
    fn format_value_compact_bool_false() {
        assert_eq!(format_value_compact(&Value::Bool(false)), "false");
    }

    #[test]
    fn format_value_compact_negative_float() {
        let val = Value::Number(serde_json::Number::from_f64(-2.75).unwrap());
        let result = format_value_compact(&val);
        assert!(
            result.contains("-2.75"),
            "Should contain -2.75, got: '{}'",
            result
        );
    }

    // ── Battle tests: value_type comprehensive ────────────────────────────

    #[test]
    fn value_type_negative_int() {
        assert_eq!(value_type(&Value::Number((-1).into())), "int");
    }

    #[test]
    fn value_type_zero_int() {
        assert_eq!(value_type(&Value::Number(0.into())), "int");
    }

    #[test]
    fn value_type_float_negative() {
        let val = Value::Number(serde_json::Number::from_f64(-1.5).unwrap());
        assert_eq!(value_type(&val), "float");
    }

    #[test]
    fn value_type_float_very_small() {
        let val = Value::Number(serde_json::Number::from_f64(0.0001).unwrap());
        assert_eq!(value_type(&val), "float");
    }

    // ── Battle tests: set_param value inference (pure logic) ──────────────

    /// Test the value inference logic that set_param uses, extracted inline.
    fn infer_value(value: &str) -> Value {
        if let Ok(parsed) = serde_json::from_str::<Value>(value) {
            parsed
        } else if value == "true" {
            Value::Bool(true)
        } else if value == "false" {
            Value::Bool(false)
        } else if let Ok(num) = value.parse::<i64>() {
            Value::Number(num.into())
        } else if let Ok(num) = value.parse::<f64>() {
            Value::Number(serde_json::Number::from_f64(num).unwrap_or_else(|| 0.into()))
        } else {
            Value::String(value.to_string())
        }
    }

    #[test]
    fn infer_value_bool_true() {
        assert_eq!(infer_value("true"), Value::Bool(true));
    }

    #[test]
    fn infer_value_bool_false() {
        assert_eq!(infer_value("false"), Value::Bool(false));
    }

    #[test]
    fn infer_value_integer() {
        assert_eq!(infer_value("42"), Value::Number(42.into()));
    }

    #[test]
    fn infer_value_negative_integer() {
        assert_eq!(infer_value("-7"), Value::Number((-7).into()));
    }

    #[test]
    fn infer_value_float() {
        let result = infer_value("2.75");
        match result {
            Value::Number(n) => {
                let f = n.as_f64().unwrap();
                assert!((f - 2.75).abs() < 1e-10, "Expected 2.75, got {}", f);
            }
            other => panic!("Expected Number, got {:?}", other),
        }
    }

    #[test]
    fn infer_value_string_fallback() {
        assert_eq!(
            infer_value("hello world"),
            Value::String("hello world".to_string())
        );
    }

    #[test]
    fn infer_value_json_object() {
        let result = infer_value("{\"key\": \"val\"}");
        assert!(result.is_object(), "Should parse as object");
        assert_eq!(result["key"], "val");
    }

    #[test]
    fn infer_value_json_array() {
        let result = infer_value("[1, 2, 3]");
        assert!(result.is_array(), "Should parse as array");
        assert_eq!(result.as_array().unwrap().len(), 3);
    }

    #[test]
    fn infer_value_empty_string_stays_string() {
        // Empty string is not valid JSON, not bool, not number
        assert_eq!(infer_value(""), Value::String(String::new()));
    }

    #[test]
    fn infer_value_zero() {
        assert_eq!(infer_value("0"), Value::Number(0.into()));
    }

    #[test]
    fn infer_value_true_uppercase_is_string() {
        // "True" != "true", should be a string
        assert_eq!(infer_value("True"), Value::String("True".to_string()));
    }

    #[test]
    fn infer_value_false_uppercase_is_string() {
        assert_eq!(infer_value("False"), Value::String("False".to_string()));
    }

    #[test]
    fn infer_value_null_parsed_as_json() {
        // "null" is valid JSON
        let result = infer_value("null");
        assert!(result.is_null(), "Should parse as null");
    }

    #[test]
    fn infer_value_scientific_notation() {
        let result = infer_value("1e10");
        match result {
            Value::Number(n) => {
                let f = n.as_f64().unwrap();
                assert!((f - 1e10).abs() < 1.0, "Expected 1e10, got {}", f);
            }
            other => panic!("Expected Number, got {:?}", other),
        }
    }

    #[test]
    fn infer_value_special_chars_string() {
        let val = infer_value("/robot/joint_1/position");
        assert_eq!(val, Value::String("/robot/joint_1/position".to_string()));
    }

    // ── Battle tests: load_params file not found ──────────────────────────

    #[test]
    fn load_params_nonexistent_file_returns_error() {
        let result = load_params(Path::new("/tmp/__nonexistent_horus_params_12345__.yaml"));
        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("not found") || err.contains("File not found"),
            "Should say file not found, got: {}",
            err
        );
    }
}
