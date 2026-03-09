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
    let params = RuntimeParams::init()?;
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
                if meta.read_only {
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
    let params = RuntimeParams::init()?;

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
    let params = RuntimeParams::init()?;

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
    let params = RuntimeParams::init()?;

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

    let params = RuntimeParams::init()?;
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

    let params = RuntimeParams::init()?;
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
    let params = RuntimeParams::init()?;

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
    let params = RuntimeParams::init()?;
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
}
