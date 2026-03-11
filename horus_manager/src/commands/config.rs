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

/// Set a value at a dotted key path.
fn set_toml_value(doc: &mut toml_edit::DocumentMut, key: &str, value: &str) -> Result<()> {
    let parts: Vec<&str> = key.split('.').collect();

    if parts.len() == 1 {
        doc[parts[0]] = toml_edit::value(value);
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
    table[last] = toml_edit::value(value);
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
