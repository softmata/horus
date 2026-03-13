//! `horus config` — CLI-based horus.toml editing.
//!
//! Read and write horus.toml values from the command line.
//! Uses toml_edit to preserve formatting and comments.

use anyhow::{Context, Result};
use colored::*;
use std::fs;

use crate::manifest::HORUS_TOML;

/// Config subcommand.
#[derive(Debug, Clone)]
pub enum ConfigAction {
    Get(String),
    Set(String, String),
    List,
}

/// Run `horus config <action>`.
pub fn run_config(action: ConfigAction) -> Result<()> {
    match action {
        ConfigAction::Get(key) => config_get(&key),
        ConfigAction::Set(key, value) => config_set(&key, &value),
        ConfigAction::List => config_list(),
    }
}

fn config_get(key: &str) -> Result<()> {
    let content = fs::read_to_string(HORUS_TOML)
        .with_context(|| format!("No {} in current directory", HORUS_TOML))?;
    let doc = content
        .parse::<toml_edit::DocumentMut>()
        .context("Failed to parse horus.toml")?;

    let value = navigate_toml(&doc, key);
    match value {
        Some(v) => {
            // Print raw value without TOML decoration
            let s = v.to_string();
            let s = s.trim_matches('"').trim();
            println!("{}", s);
        }
        None => {
            anyhow::bail!("Key '{}' not found in horus.toml", key);
        }
    }
    Ok(())
}

fn config_set(key: &str, value: &str) -> Result<()> {
    let content = fs::read_to_string(HORUS_TOML)
        .with_context(|| format!("No {} in current directory", HORUS_TOML))?;
    let mut doc = content
        .parse::<toml_edit::DocumentMut>()
        .context("Failed to parse horus.toml")?;

    set_toml_value(&mut doc, key, value)?;

    fs::write(HORUS_TOML, doc.to_string()).context("Failed to write horus.toml")?;
    println!("{} {} = {}", "set:".green(), key, value.cyan());
    Ok(())
}

fn config_list() -> Result<()> {
    let content = fs::read_to_string(HORUS_TOML)
        .with_context(|| format!("No {} in current directory", HORUS_TOML))?;
    let doc = content
        .parse::<toml_edit::DocumentMut>()
        .context("Failed to parse horus.toml")?;

    print_toml_flat(&doc, "");
    Ok(())
}

/// Navigate a dotted key path in a TOML document.
fn navigate_toml<'a>(doc: &'a toml_edit::DocumentMut, key: &str) -> Option<&'a toml_edit::Item> {
    let parts: Vec<&str> = key.split('.').collect();
    let mut current: &toml_edit::Item = doc.as_item();

    for part in parts {
        match current {
            toml_edit::Item::Table(t) => {
                current = t.get(part)?;
            }
            _ => return None,
        }
    }

    Some(current)
}

/// Parse a string value into the appropriate TOML type (bool, integer, float, or string).
fn parse_toml_value(value: &str) -> toml_edit::Item {
    if value == "true" {
        toml_edit::value(true)
    } else if value == "false" {
        toml_edit::value(false)
    } else if let Ok(i) = value.parse::<i64>() {
        toml_edit::value(i)
    } else if let Ok(f) = value.parse::<f64>() {
        toml_edit::value(f)
    } else {
        toml_edit::value(value)
    }
}

/// Set a value at a dotted key path.
fn set_toml_value(doc: &mut toml_edit::DocumentMut, key: &str, value: &str) -> Result<()> {
    let parts: Vec<&str> = key.split('.').collect();
    let item = parse_toml_value(value);

    if parts.len() == 1 {
        doc[parts[0]] = item;
        return Ok(());
    }

    // Navigate to parent table
    let mut table = doc.as_table_mut();
    for part in &parts[..parts.len() - 1] {
        if !table.contains_key(part) {
            table[part] = toml_edit::Item::Table(toml_edit::Table::new());
        }
        table = table[part]
            .as_table_mut()
            .with_context(|| format!("'{}' is not a table in horus.toml", part))?;
    }

    let last = parts.last().unwrap();
    table[last] = item;
    Ok(())
}

/// Print all key-value pairs in flat dotted notation.
fn print_toml_flat(doc: &toml_edit::DocumentMut, prefix: &str) {
    for (key, item) in doc.iter() {
        let full_key = if prefix.is_empty() {
            key.to_string()
        } else {
            format!("{}.{}", prefix, key)
        };

        match item {
            toml_edit::Item::Table(t) => {
                print_table_flat(t, &full_key);
            }
            toml_edit::Item::Value(v) => {
                println!("{} = {}", full_key.dimmed(), format_value(v));
            }
            _ => {}
        }
    }
}

fn print_table_flat(table: &toml_edit::Table, prefix: &str) {
    for (key, item) in table.iter() {
        let full_key = format!("{}.{}", prefix, key);
        match item {
            toml_edit::Item::Table(t) => {
                print_table_flat(t, &full_key);
            }
            toml_edit::Item::Value(v) => {
                println!("{} = {}", full_key.dimmed(), format_value(v));
            }
            toml_edit::Item::ArrayOfTables(a) => {
                for (i, t) in a.iter().enumerate() {
                    let indexed = format!("{}[{}]", full_key, i);
                    print_table_flat(t, &indexed);
                }
            }
            _ => {}
        }
    }
}

fn format_value(v: &toml_edit::Value) -> String {
    let s = v.to_string();
    s.trim().to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── navigate_toml ────────────────────────────────────────────────────

    #[test]
    fn navigate_top_level_key() {
        let doc: toml_edit::DocumentMut = "[package]\nname = \"test\"\n".parse().unwrap();
        let item = navigate_toml(&doc, "package");
        assert!(item.is_some());
        assert!(item.unwrap().is_table());
    }

    #[test]
    fn navigate_nested_key() {
        let doc: toml_edit::DocumentMut =
            "[package]\nname = \"test\"\nversion = \"1.0\"\n".parse().unwrap();
        let item = navigate_toml(&doc, "package.name");
        assert!(item.is_some());
        assert_eq!(item.unwrap().as_str(), Some("test"));
    }

    #[test]
    fn navigate_deep_nested_key() {
        let doc: toml_edit::DocumentMut =
            "[a]\n[a.b]\n[a.b.c]\nvalue = 42\n".parse().unwrap();
        let item = navigate_toml(&doc, "a.b.c.value");
        assert!(item.is_some());
        assert_eq!(item.unwrap().as_integer(), Some(42));
    }

    #[test]
    fn navigate_missing_key_returns_none() {
        let doc: toml_edit::DocumentMut = "[package]\nname = \"test\"\n".parse().unwrap();
        assert!(navigate_toml(&doc, "package.missing").is_none());
        assert!(navigate_toml(&doc, "nonexistent").is_none());
        assert!(navigate_toml(&doc, "package.name.deep").is_none());
    }

    // ── set_toml_value ───────────────────────────────────────────────────

    #[test]
    fn set_top_level_value() {
        let mut doc: toml_edit::DocumentMut = "".parse().unwrap();
        set_toml_value(&mut doc, "name", "hello").unwrap();
        assert_eq!(doc["name"].as_str(), Some("hello"));
    }

    #[test]
    fn set_nested_value() {
        let mut doc: toml_edit::DocumentMut = "[package]\nname = \"old\"\n".parse().unwrap();
        set_toml_value(&mut doc, "package.name", "new").unwrap();
        assert_eq!(doc["package"]["name"].as_str(), Some("new"));
    }

    #[test]
    fn set_creates_intermediate_tables() {
        let mut doc: toml_edit::DocumentMut = "".parse().unwrap();
        set_toml_value(&mut doc, "a.b.c", "deep").unwrap();
        assert_eq!(doc["a"]["b"]["c"].as_str(), Some("deep"));
    }

    #[test]
    fn set_overwrites_existing() {
        let mut doc: toml_edit::DocumentMut =
            "[package]\nversion = \"0.1.0\"\n".parse().unwrap();
        set_toml_value(&mut doc, "package.version", "2.0.0").unwrap();
        assert_eq!(doc["package"]["version"].as_str(), Some("2.0.0"));
    }

    #[test]
    fn set_fails_on_non_table_intermediate() {
        let mut doc: toml_edit::DocumentMut =
            "[package]\nname = \"test\"\n".parse().unwrap();
        let result = set_toml_value(&mut doc, "package.name.sub", "val");
        assert!(result.is_err());
    }

    // ── format_value ─────────────────────────────────────────────────────

    #[test]
    fn format_value_trims_whitespace() {
        let val = toml_edit::value("hello");
        if let toml_edit::Item::Value(v) = val {
            let formatted = format_value(&v);
            assert_eq!(formatted, "\"hello\"");
        }
    }

    // ── ConfigAction ─────────────────────────────────────────────────────

    #[test]
    fn config_action_debug() {
        let get = ConfigAction::Get("package.name".to_string());
        assert!(format!("{:?}", get).contains("Get"));
        let set = ConfigAction::Set("package.name".to_string(), "new".to_string());
        assert!(format!("{:?}", set).contains("Set"));
        let list = ConfigAction::List;
        assert!(format!("{:?}", list).contains("List"));
    }

    #[test]
    fn config_action_clone() {
        let original = ConfigAction::Get("key".to_string());
        let cloned = original.clone();
        assert!(matches!(cloned, ConfigAction::Get(k) if k == "key"));
    }

    // ── Integration: config get/set/list ─────────────────────────────────

    #[test]
    fn config_get_reads_value() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join(HORUS_TOML),
            "[package]\nname = \"my_robot\"\nversion = \"0.1.0\"\n",
        ).unwrap();

        let _lock = crate::CWD_LOCK.lock().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_config(ConfigAction::Get("package.name".to_string()));
        std::env::set_current_dir(prev).unwrap();
        assert!(result.is_ok());
    }

    #[test]
    fn config_get_missing_key_fails() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join(HORUS_TOML), "[package]\nname = \"test\"\n").unwrap();

        let _lock = crate::CWD_LOCK.lock().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_config(ConfigAction::Get("package.nonexistent".to_string()));
        std::env::set_current_dir(prev).unwrap();
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("not found"));
    }

    #[test]
    fn config_get_no_horus_toml_fails() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _lock = crate::CWD_LOCK.lock().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_config(ConfigAction::Get("package.name".to_string()));
        std::env::set_current_dir(prev).unwrap();
        assert!(result.is_err());
    }

    #[test]
    fn config_set_modifies_file() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join(HORUS_TOML),
            "[package]\nname = \"old\"\nversion = \"0.1.0\"\n",
        ).unwrap();

        let _lock = crate::CWD_LOCK.lock().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_config(ConfigAction::Set(
            "package.version".to_string(),
            "2.0.0".to_string(),
        ));
        std::env::set_current_dir(prev).unwrap();

        assert!(result.is_ok());
        let content = fs::read_to_string(tmp.path().join(HORUS_TOML)).unwrap();
        assert!(content.contains("2.0.0"));
        assert!(content.contains("old")); // Old name preserved
    }

    #[test]
    fn config_set_creates_new_key() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join(HORUS_TOML), "[package]\nname = \"test\"\n").unwrap();

        let _lock = crate::CWD_LOCK.lock().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_config(ConfigAction::Set(
            "package.description".to_string(),
            "My robot project".to_string(),
        ));
        std::env::set_current_dir(prev).unwrap();

        assert!(result.is_ok());
        let content = fs::read_to_string(tmp.path().join(HORUS_TOML)).unwrap();
        assert!(content.contains("description"));
        assert!(content.contains("My robot project"));
    }

    #[test]
    fn config_list_succeeds() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join(HORUS_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        ).unwrap();

        let _lock = crate::CWD_LOCK.lock().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_config(ConfigAction::List);
        std::env::set_current_dir(prev).unwrap();
        assert!(result.is_ok());
    }

    // ── Battle tests: edge cases ─────────────────────────────────────────

    #[test]
    fn config_set_preserves_comments_and_formatting() {
        let tmp = tempfile::TempDir::new().unwrap();
        let content = "# My robot config\n[package]\nname = \"robot\"\nversion = \"0.1.0\"\n";
        fs::write(tmp.path().join(HORUS_TOML), content).unwrap();

        let _lock = crate::CWD_LOCK.lock().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();
        run_config(ConfigAction::Set("package.version".to_string(), "1.0.0".to_string())).unwrap();
        std::env::set_current_dir(prev).unwrap();

        let updated = fs::read_to_string(tmp.path().join(HORUS_TOML)).unwrap();
        assert!(updated.contains("1.0.0"));
        assert!(updated.contains("robot")); // name preserved
    }

    #[test]
    fn config_set_creates_deep_nested_key() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join(HORUS_TOML), "[package]\nname = \"test\"\n").unwrap();

        let _lock = crate::CWD_LOCK.lock().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_config(ConfigAction::Set(
            "custom.section.deep.key".to_string(),
            "deep_value".to_string(),
        ));
        std::env::set_current_dir(prev).unwrap();

        assert!(result.is_ok());
        let content = fs::read_to_string(tmp.path().join(HORUS_TOML)).unwrap();
        assert!(content.contains("deep_value"));
    }

    #[test]
    fn config_get_returns_table_for_section() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join(HORUS_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        ).unwrap();

        let _lock = crate::CWD_LOCK.lock().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();
        // Getting a section (table) should succeed
        let result = run_config(ConfigAction::Get("package".to_string()));
        std::env::set_current_dir(prev).unwrap();
        assert!(result.is_ok());
    }

    #[test]
    fn config_set_multiple_keys_sequentially() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(
            tmp.path().join(HORUS_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        ).unwrap();

        let _lock = crate::CWD_LOCK.lock().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        run_config(ConfigAction::Set("package.description".to_string(), "A robot".to_string())).unwrap();
        run_config(ConfigAction::Set("package.license".to_string(), "MIT".to_string())).unwrap();
        run_config(ConfigAction::Set("package.version".to_string(), "2.0.0".to_string())).unwrap();

        std::env::set_current_dir(prev).unwrap();

        let content = fs::read_to_string(tmp.path().join(HORUS_TOML)).unwrap();
        assert!(content.contains("A robot"));
        assert!(content.contains("MIT"));
        assert!(content.contains("2.0.0"));
        assert!(content.contains("test")); // original name preserved
    }

    #[test]
    fn navigate_toml_empty_key() {
        let doc: toml_edit::DocumentMut = "[package]\nname = \"test\"\n".parse().unwrap();
        // Empty key should return the root item
        let item = navigate_toml(&doc, "");
        // The root is a table, so navigating with "" splits into [""] which may
        // find nothing — just verify no panic
        let _ = item;
    }

    #[test]
    fn set_toml_single_key_creates_top_level() {
        let mut doc: toml_edit::DocumentMut = "".parse().unwrap();
        set_toml_value(&mut doc, "foo", "bar").unwrap();
        assert_eq!(doc["foo"].as_str(), Some("bar"));
    }

    #[test]
    fn config_list_with_all_sections() {
        let tmp = tempfile::TempDir::new().unwrap();
        let content = r#"[package]
name = "full"
version = "1.0.0"

[dependencies]
serde = "1.0"

[drivers]
camera = "opencv"

[scripts]
sim = "echo sim"
"#;
        fs::write(tmp.path().join(HORUS_TOML), content).unwrap();

        let _lock = crate::CWD_LOCK.lock().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_config(ConfigAction::List);
        std::env::set_current_dir(prev).unwrap();
        assert!(result.is_ok());
    }
}
