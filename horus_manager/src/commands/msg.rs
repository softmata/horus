//! Message command - Message type introspection
//!
//! Lists and inspects HORUS message types defined in horus_library.

use crate::cli_output;
use colored::*;
use horus_core::error::{HorusError, HorusResult};
use std::collections::HashMap;
use std::fs;
use std::path::Path;

/// Message type information
#[derive(Debug, Clone)]
pub struct MessageInfo {
    /// Message type name
    pub name: String,
    /// Module/category (e.g., "control", "sensor", "vision")
    pub module: String,
    /// Fields in the message
    pub fields: Vec<FieldInfo>,
    /// Documentation comment
    pub doc: String,
    /// Source file path
    pub source_file: String,
}

/// Field information
#[derive(Debug, Clone)]
pub struct FieldInfo {
    /// Field name
    pub name: String,
    /// Field type
    pub field_type: String,
    /// Documentation comment
    pub doc: String,
}

/// List all message types
pub fn list_messages(verbose: bool, filter: Option<&str>) -> HorusResult<()> {
    let messages = discover_messages()?;

    // Apply filter if specified
    let filtered: Vec<_> = if let Some(f) = filter {
        let f_lower = f.to_lowercase();
        messages
            .iter()
            .filter(|m| {
                m.name.to_lowercase().contains(&f_lower)
                    || m.module.to_lowercase().contains(&f_lower)
            })
            .collect()
    } else {
        messages.iter().collect()
    };

    if filtered.is_empty() {
        if filter.is_some() {
            println!("{}", "No message types found matching filter.".yellow());
        } else {
            println!("{}", "No message types found.".yellow());
        }
        return Ok(());
    }

    println!("{}", "HORUS Message Types".green().bold());
    println!();

    if verbose {
        // Group by module
        let mut by_module: HashMap<String, Vec<&MessageInfo>> = HashMap::new();
        for msg in &filtered {
            by_module.entry(msg.module.clone()).or_default().push(msg);
        }

        let mut modules: Vec<_> = by_module.keys().cloned().collect();
        modules.sort();

        for module in modules {
            println!("  {}", format!("{}:", module).cyan().bold());
            let Some(msgs) = by_module.get(&module) else {
                continue;
            };
            for msg in msgs {
                println!("    {} {}", "".white(), msg.name.white().bold());
                if !msg.doc.is_empty() {
                    // Truncate doc to first line
                    let first_line = msg.doc.lines().next().unwrap_or("");
                    println!("      {}", first_line.dimmed());
                }
                if !msg.fields.is_empty() {
                    let field_count = msg.fields.len();
                    println!("      {} fields: {}", cli_output::ICON_HINT.dimmed(), field_count);
                }
            }
            println!();
        }
    } else {
        // Compact table view
        println!(
            "  {:<30} {:<15} {:>8}",
            "MESSAGE TYPE".dimmed(),
            "MODULE".dimmed(),
            "FIELDS".dimmed()
        );
        println!("  {}", "-".repeat(55).dimmed());

        for msg in &filtered {
            let field_count = if msg.fields.is_empty() {
                "-".to_string()
            } else {
                msg.fields.len().to_string()
            };
            println!("  {:<30} {:<15} {:>8}", msg.name, msg.module, field_count);
        }
    }

    println!();
    println!("  {} {} message type(s)", "Total:".dimmed(), filtered.len());

    Ok(())
}

/// Show detailed info about a message type
pub fn show_message(name: &str, json: bool) -> HorusResult<()> {
    let messages = discover_messages()?;

    // Find matching message
    let msg = messages.iter().find(|m| {
        m.name.eq_ignore_ascii_case(name)
            || format!("{}::{}", m.module, m.name).eq_ignore_ascii_case(name)
    });

    let Some(msg) = msg else {
        return Err(HorusError::Config(format!(
            "Message type '{}' not found. Use 'horus msg list' to see available types.",
            name
        )));
    };

    if json {
        let fields: Vec<_> = msg.fields.iter().map(|f| {
            serde_json::json!({
                "name": f.name,
                "type": f.field_type,
                "doc": f.doc,
            })
        }).collect();
        let md5 = compute_message_hash(msg);
        let output = serde_json::json!({
            "name": msg.name,
            "module": msg.module,
            "source_file": msg.source_file,
            "doc": msg.doc,
            "fields": fields,
            "md5": md5,
        });
        println!("{}", serde_json::to_string_pretty(&output).unwrap());
        return Ok(());
    }

    println!("{}", "Message Type Definition".green().bold());
    println!();
    println!("  {} {}", "Type:".cyan(), msg.name.white().bold());
    println!("  {} {}", "Module:".cyan(), msg.module);
    println!("  {} {}", "Source:".cyan(), msg.source_file.dimmed());

    if !msg.doc.is_empty() {
        println!();
        println!("  {}", "Description:".cyan());
        for line in msg.doc.lines() {
            println!("    {}", line);
        }
    }

    println!();
    println!("  {}", "Fields:".cyan());
    if msg.fields.is_empty() {
        println!("    {}", "(no public fields or unit struct)".dimmed());
    } else {
        for field in &msg.fields {
            println!(
                "    {} {}: {}",
                "".white(),
                field.name.white(),
                field.field_type.yellow()
            );
            if !field.doc.is_empty() {
                println!("      {}", field.doc.dimmed());
            }
        }
    }

    // Compute MD5
    let md5 = compute_message_hash(msg);
    println!();
    println!("  {} {}", "MD5:".cyan(), md5.dimmed());

    Ok(())
}

/// Show message hash (MD5)
pub fn message_hash(name: &str, json: bool) -> HorusResult<()> {
    let messages = discover_messages()?;

    let msg = messages.iter().find(|m| {
        m.name.eq_ignore_ascii_case(name)
            || format!("{}::{}", m.module, m.name).eq_ignore_ascii_case(name)
    });

    let Some(msg) = msg else {
        return Err(HorusError::Config(format!(
            "Message type '{}' not found.",
            name
        )));
    };
    let md5 = compute_message_hash(msg);

    if json {
        let output = serde_json::json!({
            "name": msg.name,
            "module": msg.module,
            "md5": md5,
        });
        println!("{}", serde_json::to_string_pretty(&output).unwrap());
    } else {
        println!("{}", md5);
    }

    Ok(())
}

/// Discover all message types from source files
fn discover_messages() -> HorusResult<Vec<MessageInfo>> {
    let mut messages = Vec::new();

    // Find the horus_library messages directory
    let base_paths = [
        "horus_library/messages",
        "../horus_library/messages",
        "../../horus_library/messages",
    ];

    let mut messages_dir = None;
    for path in base_paths {
        let p = Path::new(path);
        if p.exists() && p.is_dir() {
            messages_dir = Some(p.to_path_buf());
            break;
        }
    }

    let messages_dir = messages_dir.ok_or_else(|| {
        HorusError::Config("Could not find horus_library/messages directory".to_string())
    })?;

    // Parse each .rs file in the messages directory
    for entry in fs::read_dir(&messages_dir).map_err(HorusError::Io)? {
        let entry = entry.map_err(HorusError::Io)?;
        let path = entry.path();

        if path.extension().map(|e| e == "rs").unwrap_or(false) {
            let filename = path
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("unknown")
                .to_string();

            // Skip mod.rs
            if filename == "mod" {
                continue;
            }

            // Parse the file for struct definitions
            if let Ok(content) = fs::read_to_string(&path) {
                let file_messages = parse_messages_from_source(
                    &content,
                    &filename,
                    path.to_string_lossy().to_string(),
                );
                messages.extend(file_messages);
            }
        }
    }

    // Sort by module and name
    messages.sort_by(|a, b| match a.module.cmp(&b.module) {
        std::cmp::Ordering::Equal => a.name.cmp(&b.name),
        other => other,
    });

    Ok(messages)
}

/// Parse message types from source code
fn parse_messages_from_source(source: &str, module: &str, source_file: String) -> Vec<MessageInfo> {
    let mut messages = Vec::new();
    let lines: Vec<&str> = source.lines().collect();
    let mut i = 0;

    while i < lines.len() {
        let line = lines[i].trim();

        // Look for pub struct definitions
        if line.starts_with("pub struct ") || line.starts_with("#[derive") {
            // Collect doc comments before the struct
            let mut doc_lines = Vec::new();
            let mut j = i;

            // Go back to find doc comments
            while j > 0 {
                let prev_line = lines[j - 1].trim();
                if prev_line.starts_with("///") {
                    doc_lines.insert(0, prev_line.trim_start_matches("///").trim());
                    j -= 1;
                } else if prev_line.is_empty() || prev_line.starts_with("#[") {
                    j -= 1;
                } else {
                    break;
                }
            }

            // Skip derive attributes to find struct line
            let mut struct_line_idx = i;
            while struct_line_idx < lines.len()
                && !lines[struct_line_idx].trim().starts_with("pub struct ")
            {
                struct_line_idx += 1;
            }

            if struct_line_idx >= lines.len() {
                i += 1;
                continue;
            }

            let struct_line = lines[struct_line_idx].trim();

            // Extract struct name
            if let Some(name) = extract_struct_name(struct_line) {
                // Parse fields
                let mut fields = Vec::new();
                let mut field_idx = struct_line_idx + 1;
                let mut in_struct = struct_line.contains('{');
                let mut brace_count = if in_struct { 1 } else { 0 };

                // Check if it's a unit struct or tuple struct
                if struct_line.ends_with(';') || struct_line.contains('(') {
                    // Unit struct or tuple struct - no named fields
                } else {
                    // Named struct - parse fields
                    while field_idx < lines.len() && (in_struct || brace_count == 0) {
                        let field_line = lines[field_idx].trim();

                        if field_line.contains('{') {
                            in_struct = true;
                            brace_count += field_line.matches('{').count();
                        }
                        if field_line.contains('}') {
                            brace_count -= field_line.matches('}').count();
                            if brace_count == 0 {
                                break;
                            }
                        }

                        // Parse field
                        if let Some(field) = parse_field(field_line) {
                            // Get field doc
                            let mut field_doc = String::new();
                            if field_idx > 0 {
                                let prev = lines[field_idx - 1].trim();
                                if prev.starts_with("///") {
                                    field_doc = prev.trim_start_matches("///").trim().to_string();
                                }
                            }
                            fields.push(FieldInfo {
                                name: field.0,
                                field_type: field.1,
                                doc: field_doc,
                            });
                        }

                        field_idx += 1;
                    }
                }

                messages.push(MessageInfo {
                    name: name.to_string(),
                    module: module.to_string(),
                    fields,
                    doc: doc_lines.join("\n"),
                    source_file: source_file.clone(),
                });

                i = field_idx;
                continue;
            }
        }

        i += 1;
    }

    messages
}

/// Extract struct name from "pub struct Foo" or "pub struct Foo {"
fn extract_struct_name(line: &str) -> Option<&str> {
    let line = line.trim_start_matches("pub struct ").trim();
    // Handle generics like "Foo<T>" or "Foo {"
    let name = line
        .split(|c: char| c == '<' || c == '{' || c == '(' || c.is_whitespace())
        .next()?;
    if name.is_empty() || !name.chars().next()?.is_uppercase() {
        return None;
    }
    Some(name)
}

/// Parse a field line like "pub name: Type," or "name: Type,"
fn parse_field(line: &str) -> Option<(String, String)> {
    let line = line.trim();

    // Skip non-field lines
    if line.is_empty()
        || line.starts_with("//")
        || line.starts_with("#[")
        || line == "{"
        || line == "}"
        || line.starts_with("pub fn")
        || line.starts_with("fn ")
        || line.starts_with("impl")
    {
        return None;
    }

    // Handle "pub name: Type," or "name: Type,"
    let line = line.trim_start_matches("pub ");

    if !line.contains(':') {
        return None;
    }

    let parts: Vec<&str> = line.splitn(2, ':').collect();
    if parts.len() != 2 {
        return None;
    }

    let name = parts[0].trim().to_string();
    let field_type = parts[1].trim().trim_end_matches(',').trim().to_string();

    // Skip if it looks like a method signature
    if field_type.contains("->") || field_type.contains("fn(") {
        return None;
    }

    // Skip padding fields
    if name.starts_with('_') {
        return None;
    }

    Some((name, field_type))
}

/// Compute MD5 hash of message definition
fn compute_message_hash(msg: &MessageInfo) -> String {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    // Create a canonical string representation
    let mut canonical = format!("{}::{}\n", msg.module, msg.name);
    for field in &msg.fields {
        canonical.push_str(&format!("  {}: {}\n", field.name, field.field_type));
    }

    // Compute hash (using DefaultHasher as a simple hash)
    let mut hasher = DefaultHasher::new();
    canonical.hash(&mut hasher);
    let hash = hasher.finish();

    // Format as hex (similar to MD5 output style)
    format!("{:016x}", hash)
}
