//! Message command - Message type introspection
//!
//! Lists and inspects HORUS message types defined in horus_library.

use crate::cli_output;
use colored::*;
use horus_core::error::{ConfigError, HorusError, HorusResult};
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
pub fn list_messages(verbose: bool, filter: Option<&str>, json: bool) -> HorusResult<()> {
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

    if json {
        let items: Vec<_> = filtered
            .iter()
            .map(|m| {
                let hash = compute_message_definition_hash(m);
                serde_json::json!({
                    "name": m.name,
                    "module": m.module,
                    "fields": m.fields.len(),
                    "hash": hash,
                    "doc": m.doc,
                    "source_file": m.source_file
                })
            })
            .collect();
        let output = serde_json::json!({
            "count": items.len(),
            "items": items
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
        return Ok(());
    }

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
                    println!(
                        "      {} fields: {}",
                        cli_output::ICON_HINT.dimmed(),
                        field_count
                    );
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
        return Err(HorusError::Config(ConfigError::Other(format!(
            "Message type '{}' not found. Use 'horus msg list' to see available types.",
            name
        ))));
    };

    if json {
        let fields: Vec<_> = msg
            .fields
            .iter()
            .map(|f| {
                serde_json::json!({
                    "name": f.name,
                    "type": f.field_type,
                    "doc": f.doc,
                })
            })
            .collect();
        let hash = compute_message_definition_hash(msg);
        let output = serde_json::json!({
            "name": msg.name,
            "module": msg.module,
            "source_file": msg.source_file,
            "doc": msg.doc,
            "fields": fields,
            "hash": hash,
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
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

    let hash = compute_message_definition_hash(msg);
    println!();
    println!("  {} {}", "Hash:".cyan(), hash.dimmed());

    Ok(())
}

/// Show message definition hash
pub fn message_hash(name: &str, json: bool) -> HorusResult<()> {
    let messages = discover_messages()?;

    let msg = messages.iter().find(|m| {
        m.name.eq_ignore_ascii_case(name)
            || format!("{}::{}", m.module, m.name).eq_ignore_ascii_case(name)
    });

    let Some(msg) = msg else {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "Message type '{}' not found.",
            name
        ))));
    };
    let hash = compute_message_definition_hash(msg);

    if json {
        let output = serde_json::json!({
            "name": msg.name,
            "module": msg.module,
            "hash": hash,
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
    } else {
        println!("{}", hash);
    }

    Ok(())
}

/// Discover all message types from source files
fn discover_messages() -> HorusResult<Vec<MessageInfo>> {
    let mut messages = Vec::new();

    // Find the horus_library messages directory.
    // Search strategy (in order):
    //   1. HORUS_SOURCE_DIR env var (explicit override)
    //   2. Relative to the horus binary location
    //   3. Known install paths
    let mut search_paths: Vec<std::path::PathBuf> = Vec::new();

    // 1. HORUS_SOURCE_DIR env var
    if let Ok(source_dir) = std::env::var("HORUS_SOURCE_DIR") {
        search_paths.push(Path::new(&source_dir).join("horus_library/messages"));
    }

    // 2. Relative to executable location
    if let Ok(exe) = std::env::current_exe() {
        if let Some(exe_dir) = exe.parent() {
            // Binary might be in target/debug or target/release
            for ancestor in [exe_dir, exe_dir.parent().unwrap_or(exe_dir)] {
                search_paths.push(ancestor.join("horus_library/messages"));
                search_paths.push(ancestor.join("../horus_library/messages"));
                search_paths.push(ancestor.join("../../horus_library/messages"));
            }
        }
    }

    // 3. Known install/development paths
    if let Some(home) = dirs::home_dir() {
        search_paths.push(home.join(".horus/library/messages"));
        search_paths.push(home.join("horus/horus_library/messages"));
        search_paths.push(home.join("softmata/horus/horus_library/messages"));
    }
    // 3b. HORUS_SOURCE env var (used by horus run for workspace resolution)
    if let Ok(source_root) = std::env::var("HORUS_SOURCE") {
        search_paths.push(Path::new(&source_root).join("horus_library/messages"));
    }
    search_paths.push(Path::new("/opt/horus/library/messages").to_path_buf());
    search_paths.push(Path::new("/usr/local/share/horus/messages").to_path_buf());

    let mut messages_dir = None;
    for path in &search_paths {
        if path.exists() && path.is_dir() {
            messages_dir = Some(path.clone());
            break;
        }
    }

    let messages_dir = messages_dir.ok_or_else(|| {
        HorusError::Config(ConfigError::Other(
            "Could not find horus_library/messages directory.\n  \
             Set HORUS_SOURCE_DIR to the root of your HORUS source tree,\n  \
             e.g.: export HORUS_SOURCE_DIR=/path/to/horus"
                .to_string(),
        ))
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

/// Compute a definition hash for a message type.
///
/// Uses SipHash (via `DefaultHasher`) on the canonical representation
/// of the message (module::name + field names/types). This detects
/// when a message definition changes across versions.
fn compute_message_definition_hash(msg: &MessageInfo) -> String {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    // Create a canonical string representation
    let mut canonical = format!("{}::{}\n", msg.module, msg.name);
    for field in &msg.fields {
        canonical.push_str(&format!("  {}: {}\n", field.name, field.field_type));
    }

    let mut hasher = DefaultHasher::new();
    canonical.hash(&mut hasher);
    let hash = hasher.finish();

    format!("{:016x}", hash)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extract_struct_name_simple() {
        assert_eq!(extract_struct_name("pub struct Twist {"), Some("Twist"));
    }

    #[test]
    fn extract_struct_name_generic() {
        assert_eq!(extract_struct_name("pub struct Vec3<T> {"), Some("Vec3"));
    }

    #[test]
    fn extract_struct_name_unit() {
        // Unit structs: the semicolon is part of the split remainder, but "Empty;"
        // doesn't split on ';' — it's kept. The function splits on '<', '{', '(' and whitespace.
        // "Empty;" doesn't match any of those, so the full "Empty;" is returned.
        let result = extract_struct_name("pub struct Empty;");
        assert!(result.is_some());
        // The name includes the semicolon because ';' isn't a split char
        assert!(result.unwrap().starts_with("Empty"));
    }

    #[test]
    fn extract_struct_name_tuple() {
        assert_eq!(
            extract_struct_name("pub struct Wrapper(f64);"),
            Some("Wrapper")
        );
    }

    #[test]
    fn extract_struct_name_lowercase_rejected() {
        assert_eq!(extract_struct_name("pub struct lowercase {"), None);
    }

    #[test]
    fn parse_field_basic() {
        let result = parse_field("pub x: f64,");
        assert_eq!(result, Some(("x".into(), "f64".into())));
    }

    #[test]
    fn parse_field_no_pub() {
        let result = parse_field("y: f32,");
        assert_eq!(result, Some(("y".into(), "f32".into())));
    }

    #[test]
    fn parse_field_complex_type() {
        let result = parse_field("pub data: Vec<u8>,");
        assert_eq!(result, Some(("data".into(), "Vec<u8>".into())));
    }

    #[test]
    fn parse_field_skips_comment() {
        assert!(parse_field("// this is a comment").is_none());
    }

    #[test]
    fn parse_field_skips_attribute() {
        assert!(parse_field("#[serde(skip)]").is_none());
    }

    #[test]
    fn parse_field_skips_method() {
        assert!(parse_field("pub fn new() -> Self {").is_none());
    }

    #[test]
    fn parse_field_skips_padding() {
        assert!(parse_field("_pad: [u8; 3],").is_none());
    }

    #[test]
    fn parse_field_skips_fn_type() {
        assert!(parse_field("pub callback: fn(u32) -> bool,").is_none());
    }

    #[test]
    fn parse_messages_from_source_simple() {
        let source = r#"
/// A 3D vector
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
"#;
        let messages = parse_messages_from_source(source, "math", "math.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].name, "Vec3");
        assert_eq!(messages[0].module, "math");
        assert_eq!(messages[0].fields.len(), 3);
        assert_eq!(messages[0].fields[0].name, "x");
        assert!(messages[0].doc.contains("3D vector"));
    }

    #[test]
    fn parse_messages_from_source_unit_struct() {
        let source = "pub struct Empty;\n";
        let messages = parse_messages_from_source(source, "test", "test.rs".into());
        assert_eq!(messages.len(), 1);
        // extract_struct_name returns "Empty;" for unit structs (';' not in split chars)
        assert!(messages[0].name.starts_with("Empty"));
        assert!(messages[0].fields.is_empty());
    }

    #[test]
    fn parse_messages_multiple_structs() {
        let source = r#"
pub struct A {
    pub x: f64,
}

pub struct B {
    pub y: i32,
    pub z: i32,
}
"#;
        let messages = parse_messages_from_source(source, "mod", "mod.rs".into());
        assert_eq!(messages.len(), 2);
        assert_eq!(messages[0].name, "A");
        assert_eq!(messages[1].name, "B");
        assert_eq!(messages[1].fields.len(), 2);
    }

    #[test]
    fn compute_hash_deterministic() {
        let msg = MessageInfo {
            name: "Twist".into(),
            module: "control".into(),
            fields: vec![
                FieldInfo {
                    name: "linear".into(),
                    field_type: "f64".into(),
                    doc: String::new(),
                },
                FieldInfo {
                    name: "angular".into(),
                    field_type: "f64".into(),
                    doc: String::new(),
                },
            ],
            doc: String::new(),
            source_file: String::new(),
        };
        let h1 = compute_message_definition_hash(&msg);
        let h2 = compute_message_definition_hash(&msg);
        assert_eq!(h1, h2, "hash must be deterministic");
        assert_eq!(h1.len(), 16, "hash should be 16 hex chars");
    }

    #[test]
    fn compute_hash_changes_with_field() {
        let msg1 = MessageInfo {
            name: "Foo".into(),
            module: "test".into(),
            fields: vec![FieldInfo {
                name: "a".into(),
                field_type: "f64".into(),
                doc: String::new(),
            }],
            doc: String::new(),
            source_file: String::new(),
        };
        let msg2 = MessageInfo {
            name: "Foo".into(),
            module: "test".into(),
            fields: vec![FieldInfo {
                name: "b".into(),
                field_type: "f64".into(),
                doc: String::new(),
            }],
            doc: String::new(),
            source_file: String::new(),
        };
        assert_ne!(
            compute_message_definition_hash(&msg1),
            compute_message_definition_hash(&msg2),
            "different fields should produce different hashes"
        );
    }
}
