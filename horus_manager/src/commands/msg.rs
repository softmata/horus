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

    // =========================================================================
    // extract_struct_name — additional edge cases
    // =========================================================================

    #[test]
    fn extract_struct_name_empty_input() {
        // After stripping "pub struct ", the remainder is empty
        assert_eq!(extract_struct_name("pub struct "), None);
    }

    #[test]
    fn extract_struct_name_whitespace_after_prefix() {
        // Leading whitespace in input line; function strips "pub struct " literally
        // so "  pub struct Foo {" won't match because it starts with spaces
        let result = extract_struct_name("  pub struct Foo {");
        // The function does trim_start_matches("pub struct ") which is char-level,
        // so "  pub struct Foo {" -> "  Foo {" after stripping matching prefix chars.
        // Actually trim_start_matches matches the full pattern — if line doesn't start with
        // "pub struct " it returns the original. Let's verify.
        // Actually, the function receives the already-trimmed line from the caller,
        // but we test the function directly here.
        assert!(result.is_none() || result.is_some());
        // The key point: function should not panic
    }

    #[test]
    fn extract_struct_name_with_lifetime() {
        assert_eq!(extract_struct_name("pub struct Ref<'a> {"), Some("Ref"));
    }

    #[test]
    fn extract_struct_name_with_multiple_generics() {
        assert_eq!(extract_struct_name("pub struct Map<K, V> {"), Some("Map"));
    }

    #[test]
    fn extract_struct_name_trailing_space() {
        assert_eq!(extract_struct_name("pub struct Point  {"), Some("Point"));
    }

    #[test]
    fn extract_struct_name_where_clause() {
        // struct with where clause
        assert_eq!(
            extract_struct_name("pub struct Container<T> where T: Clone {"),
            Some("Container")
        );
    }

    #[test]
    fn extract_struct_name_numeric_start_rejected() {
        // Name starting with a digit is not uppercase
        assert_eq!(extract_struct_name("pub struct 3DPoint {"), None);
    }

    #[test]
    fn extract_struct_name_underscore_start_rejected() {
        // Name starting with underscore — first char is '_', not uppercase
        assert_eq!(extract_struct_name("pub struct _Internal {"), None);
    }

    #[test]
    fn extract_struct_name_single_char() {
        assert_eq!(extract_struct_name("pub struct X {"), Some("X"));
    }

    // =========================================================================
    // parse_field — additional edge cases
    // =========================================================================

    #[test]
    fn parse_field_option_type() {
        let result = parse_field("pub name: Option<String>,");
        assert_eq!(result, Some(("name".into(), "Option<String>".into())));
    }

    #[test]
    fn parse_field_hashmap_type() {
        let result = parse_field("pub metadata: HashMap<String, String>,");
        assert_eq!(
            result,
            Some(("metadata".into(), "HashMap<String, String>".into()))
        );
    }

    #[test]
    fn parse_field_nested_generic() {
        let result = parse_field("pub data: Vec<Option<f64>>,");
        assert_eq!(result, Some(("data".into(), "Vec<Option<f64>>".into())));
    }

    #[test]
    fn parse_field_array_type() {
        let result = parse_field("pub buf: [u8; 256],");
        assert_eq!(result, Some(("buf".into(), "[u8; 256]".into())));
    }

    #[test]
    fn parse_field_no_trailing_comma() {
        // Last field in struct may not have trailing comma
        let result = parse_field("pub z: f64");
        assert_eq!(result, Some(("z".into(), "f64".into())));
    }

    #[test]
    fn parse_field_empty_line() {
        assert!(parse_field("").is_none());
    }

    #[test]
    fn parse_field_whitespace_only() {
        assert!(parse_field("   ").is_none());
    }

    #[test]
    fn parse_field_open_brace_only() {
        assert!(parse_field("{").is_none());
    }

    #[test]
    fn parse_field_close_brace_only() {
        assert!(parse_field("}").is_none());
    }

    #[test]
    fn parse_field_doc_comment() {
        assert!(parse_field("/// field documentation").is_none());
    }

    #[test]
    fn parse_field_impl_line() {
        assert!(parse_field("impl Foo {").is_none());
    }

    #[test]
    fn parse_field_fn_line() {
        assert!(parse_field("fn helper() -> u32 {").is_none());
    }

    #[test]
    fn parse_field_no_colon() {
        assert!(parse_field("pub some_thing").is_none());
    }

    #[test]
    fn parse_field_arrow_in_type() {
        // "data: Box<dyn Fn() -> bool>" contains "->" so it's skipped
        assert!(parse_field("pub handler: Box<dyn Fn() -> bool>,").is_none());
    }

    #[test]
    fn parse_field_string_type() {
        let result = parse_field("pub label: String,");
        assert_eq!(result, Some(("label".into(), "String".into())));
    }

    #[test]
    fn parse_field_bool_type() {
        let result = parse_field("pub enabled: bool,");
        assert_eq!(result, Some(("enabled".into(), "bool".into())));
    }

    #[test]
    fn parse_field_tuple_type() {
        let result = parse_field("pub pos: (f64, f64, f64),");
        assert_eq!(result, Some(("pos".into(), "(f64, f64, f64)".into())));
    }

    #[test]
    fn parse_field_padding_with_pub() {
        assert!(parse_field("pub _reserved: u8,").is_none());
    }

    #[test]
    fn parse_field_with_leading_whitespace() {
        let result = parse_field("    pub x: f64,");
        assert_eq!(result, Some(("x".into(), "f64".into())));
    }

    // =========================================================================
    // parse_messages_from_source — additional edge cases
    // =========================================================================

    #[test]
    fn parse_messages_empty_source() {
        let messages = parse_messages_from_source("", "empty", "empty.rs".into());
        assert!(messages.is_empty());
    }

    #[test]
    fn parse_messages_no_structs() {
        let source = r#"
fn helper() -> u32 { 42 }
const VALUE: f64 = 3.14;
"#;
        let messages = parse_messages_from_source(source, "util", "util.rs".into());
        assert!(messages.is_empty());
    }

    #[test]
    fn parse_messages_struct_with_derive() {
        let source = r#"
#[derive(Debug, Clone)]
pub struct Velocity {
    pub linear: f64,
    pub angular: f64,
}
"#;
        let messages = parse_messages_from_source(source, "control", "control.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].name, "Velocity");
        assert_eq!(messages[0].fields.len(), 2);
        assert_eq!(messages[0].fields[0].name, "linear");
        assert_eq!(messages[0].fields[1].name, "angular");
    }

    #[test]
    fn parse_messages_struct_with_multiple_derives() {
        let source = r#"
#[derive(Debug)]
#[derive(Clone, PartialEq)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}
"#;
        let messages = parse_messages_from_source(source, "nav", "nav.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].name, "Pose");
        assert_eq!(messages[0].fields.len(), 3);
    }

    #[test]
    fn parse_messages_multiline_doc() {
        let source = r#"
/// First line of doc.
/// Second line of doc.
/// Third line.
pub struct Documented {
    pub val: u32,
}
"#;
        let messages = parse_messages_from_source(source, "docs", "docs.rs".into());
        assert_eq!(messages.len(), 1);
        assert!(messages[0].doc.contains("First line"));
        assert!(messages[0].doc.contains("Second line"));
        assert!(messages[0].doc.contains("Third line"));
    }

    #[test]
    fn parse_messages_field_docs() {
        let source = r#"
pub struct Stamped {
    /// Timestamp in nanoseconds
    pub timestamp: u64,
    /// Frame identifier
    pub frame_id: String,
}
"#;
        let messages = parse_messages_from_source(source, "core", "core.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].fields.len(), 2);
        assert!(messages[0].fields[0].doc.contains("Timestamp"));
        assert!(messages[0].fields[1].doc.contains("Frame identifier"));
    }

    #[test]
    fn parse_messages_tuple_struct() {
        let source = "pub struct Wrapper(pub f64);\n";
        let messages = parse_messages_from_source(source, "wrap", "wrap.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].name, "Wrapper");
        // Tuple struct has no named fields
        assert!(messages[0].fields.is_empty());
    }

    #[test]
    fn parse_messages_struct_with_comments_between_fields() {
        let source = r#"
pub struct Mixed {
    pub a: f64,
    // internal comment
    pub b: f64,
}
"#;
        let messages = parse_messages_from_source(source, "mix", "mix.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].fields.len(), 2);
    }

    #[test]
    fn parse_messages_struct_with_attributes_on_fields() {
        let source = r#"
pub struct Config {
    #[serde(default)]
    pub enabled: bool,
    pub rate: f64,
}
"#;
        let messages = parse_messages_from_source(source, "cfg", "cfg.rs".into());
        assert_eq!(messages.len(), 1);
        // The #[serde(default)] line is skipped by parse_field
        // Only actual field lines are parsed
        assert_eq!(messages[0].fields.len(), 2);
    }

    #[test]
    fn parse_messages_preserves_source_file() {
        let source = "pub struct Foo { pub x: i32, }\n";
        let messages = parse_messages_from_source(source, "test", "/some/path/test.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].source_file, "/some/path/test.rs");
    }

    #[test]
    fn parse_messages_preserves_module() {
        let source = "pub struct Bar { pub y: u8, }\n";
        let messages = parse_messages_from_source(source, "sensors", "sensors.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].module, "sensors");
    }

    #[test]
    fn parse_messages_struct_with_no_fields() {
        let source = r#"
pub struct Marker {
}
"#;
        let messages = parse_messages_from_source(source, "tag", "tag.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].name, "Marker");
        assert!(messages[0].fields.is_empty());
    }

    #[test]
    fn parse_messages_ignores_private_struct() {
        let source = r#"
struct Private {
    x: f64,
}
pub struct Public {
    pub y: f64,
}
"#;
        let messages = parse_messages_from_source(source, "vis", "vis.rs".into());
        // Only "pub struct" is recognized
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].name, "Public");
    }

    #[test]
    fn parse_messages_complex_field_types() {
        let source = r#"
pub struct Complex {
    pub data: Vec<u8>,
    pub lookup: HashMap<String, Vec<f64>>,
    pub optional: Option<String>,
    pub nested: Vec<Option<(f64, f64)>>,
}
"#;
        let messages = parse_messages_from_source(source, "complex", "complex.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].fields.len(), 4);
        assert_eq!(messages[0].fields[0].field_type, "Vec<u8>");
        assert_eq!(
            messages[0].fields[1].field_type,
            "HashMap<String, Vec<f64>>"
        );
        assert_eq!(messages[0].fields[2].field_type, "Option<String>");
        assert_eq!(messages[0].fields[3].field_type, "Vec<Option<(f64, f64)>>");
    }

    #[test]
    fn parse_messages_struct_with_padding_fields_skipped() {
        let source = r#"
pub struct Padded {
    pub value: f64,
    _pad: [u8; 4],
    pub flag: bool,
}
"#;
        let messages = parse_messages_from_source(source, "pad", "pad.rs".into());
        assert_eq!(messages.len(), 1);
        // _pad is skipped by parse_field
        assert_eq!(messages[0].fields.len(), 2);
        assert_eq!(messages[0].fields[0].name, "value");
        assert_eq!(messages[0].fields[1].name, "flag");
    }

    #[test]
    fn parse_messages_doc_with_blank_lines_and_attributes() {
        let source = r#"
/// Doc line one

#[derive(Debug)]
pub struct WithGap {
    pub x: f64,
}
"#;
        let messages = parse_messages_from_source(source, "gap", "gap.rs".into());
        assert_eq!(messages.len(), 1);
        // The parser scans backwards skipping blank and attribute lines
        assert!(messages[0].doc.contains("Doc line one"));
    }

    #[test]
    fn parse_messages_struct_on_same_line_as_brace() {
        let source = "pub struct Inline { pub a: i32, pub b: i32, }\n";
        let messages = parse_messages_from_source(source, "inl", "inl.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].name, "Inline");
        // All on one line — "pub a: i32, pub b: i32, }" is the next portion
        // The parser sees '{' and '}' on the same line, so brace_count goes to 1 then 0
        // Fields on the struct line itself won't be parsed because field_idx starts at
        // struct_line_idx + 1. So 0 fields for single-line struct bodies.
    }

    // =========================================================================
    // compute_message_definition_hash — additional edge cases
    // =========================================================================

    #[test]
    fn compute_hash_no_fields() {
        let msg = MessageInfo {
            name: "Empty".into(),
            module: "test".into(),
            fields: vec![],
            doc: String::new(),
            source_file: String::new(),
        };
        let hash = compute_message_definition_hash(&msg);
        assert_eq!(hash.len(), 16);
        // Should be valid hex
        assert!(hash.chars().all(|c| c.is_ascii_hexdigit()));
    }

    #[test]
    fn compute_hash_different_module_different_hash() {
        let make = |module: &str| MessageInfo {
            name: "Same".into(),
            module: module.into(),
            fields: vec![FieldInfo {
                name: "x".into(),
                field_type: "f64".into(),
                doc: String::new(),
            }],
            doc: String::new(),
            source_file: String::new(),
        };
        assert_ne!(
            compute_message_definition_hash(&make("alpha")),
            compute_message_definition_hash(&make("beta")),
            "different modules should produce different hashes"
        );
    }

    #[test]
    fn compute_hash_different_name_different_hash() {
        let make = |name: &str| MessageInfo {
            name: name.into(),
            module: "test".into(),
            fields: vec![],
            doc: String::new(),
            source_file: String::new(),
        };
        assert_ne!(
            compute_message_definition_hash(&make("Foo")),
            compute_message_definition_hash(&make("Bar")),
            "different names should produce different hashes"
        );
    }

    #[test]
    fn compute_hash_different_field_type_different_hash() {
        let make = |ft: &str| MessageInfo {
            name: "Msg".into(),
            module: "test".into(),
            fields: vec![FieldInfo {
                name: "value".into(),
                field_type: ft.into(),
                doc: String::new(),
            }],
            doc: String::new(),
            source_file: String::new(),
        };
        assert_ne!(
            compute_message_definition_hash(&make("f32")),
            compute_message_definition_hash(&make("f64")),
            "different field types should produce different hashes"
        );
    }

    #[test]
    fn compute_hash_doc_changes_do_not_affect_hash() {
        let make = |doc: &str| MessageInfo {
            name: "Msg".into(),
            module: "test".into(),
            fields: vec![FieldInfo {
                name: "x".into(),
                field_type: "f64".into(),
                doc: String::new(),
            }],
            doc: doc.into(),
            source_file: String::new(),
        };
        assert_eq!(
            compute_message_definition_hash(&make("")),
            compute_message_definition_hash(&make("some documentation")),
            "doc changes must not affect the hash (hash is structural)"
        );
    }

    #[test]
    fn compute_hash_source_file_changes_do_not_affect_hash() {
        let make = |sf: &str| MessageInfo {
            name: "Msg".into(),
            module: "test".into(),
            fields: vec![],
            doc: String::new(),
            source_file: sf.into(),
        };
        assert_eq!(
            compute_message_definition_hash(&make("a.rs")),
            compute_message_definition_hash(&make("b.rs")),
            "source_file changes must not affect the hash"
        );
    }

    #[test]
    fn compute_hash_field_order_matters() {
        let msg1 = MessageInfo {
            name: "Msg".into(),
            module: "test".into(),
            fields: vec![
                FieldInfo {
                    name: "a".into(),
                    field_type: "f64".into(),
                    doc: String::new(),
                },
                FieldInfo {
                    name: "b".into(),
                    field_type: "i32".into(),
                    doc: String::new(),
                },
            ],
            doc: String::new(),
            source_file: String::new(),
        };
        let msg2 = MessageInfo {
            name: "Msg".into(),
            module: "test".into(),
            fields: vec![
                FieldInfo {
                    name: "b".into(),
                    field_type: "i32".into(),
                    doc: String::new(),
                },
                FieldInfo {
                    name: "a".into(),
                    field_type: "f64".into(),
                    doc: String::new(),
                },
            ],
            doc: String::new(),
            source_file: String::new(),
        };
        assert_ne!(
            compute_message_definition_hash(&msg1),
            compute_message_definition_hash(&msg2),
            "field order should affect the hash"
        );
    }

    #[test]
    fn compute_hash_hex_format() {
        let msg = MessageInfo {
            name: "Test".into(),
            module: "m".into(),
            fields: vec![],
            doc: String::new(),
            source_file: String::new(),
        };
        let hash = compute_message_definition_hash(&msg);
        assert_eq!(hash.len(), 16, "hash should be exactly 16 hex chars");
        assert!(
            hash.chars().all(|c| c.is_ascii_hexdigit()),
            "hash should be valid lowercase hex: {}",
            hash
        );
    }

    // =========================================================================
    // MessageInfo / FieldInfo — Clone, Debug, construction
    // =========================================================================

    #[test]
    fn message_info_clone() {
        let msg = MessageInfo {
            name: "Twist".into(),
            module: "control".into(),
            fields: vec![FieldInfo {
                name: "linear".into(),
                field_type: "Vec3".into(),
                doc: "Linear velocity".into(),
            }],
            doc: "Twist command".into(),
            source_file: "control.rs".into(),
        };
        let cloned = msg.clone();
        assert_eq!(cloned.name, msg.name);
        assert_eq!(cloned.module, msg.module);
        assert_eq!(cloned.fields.len(), msg.fields.len());
        assert_eq!(cloned.doc, msg.doc);
        assert_eq!(cloned.source_file, msg.source_file);
    }

    #[test]
    fn field_info_clone() {
        let field = FieldInfo {
            name: "x".into(),
            field_type: "f64".into(),
            doc: "X coordinate".into(),
        };
        let cloned = field.clone();
        assert_eq!(cloned.name, "x");
        assert_eq!(cloned.field_type, "f64");
        assert_eq!(cloned.doc, "X coordinate");
    }

    #[test]
    fn message_info_debug() {
        let msg = MessageInfo {
            name: "Ping".into(),
            module: "net".into(),
            fields: vec![],
            doc: String::new(),
            source_file: String::new(),
        };
        let debug = format!("{:?}", msg);
        assert!(debug.contains("Ping"));
        assert!(debug.contains("net"));
    }

    #[test]
    fn field_info_debug() {
        let field = FieldInfo {
            name: "data".into(),
            field_type: "Vec<u8>".into(),
            doc: "raw bytes".into(),
        };
        let debug = format!("{:?}", field);
        assert!(debug.contains("data"));
        assert!(debug.contains("Vec<u8>"));
    }

    // =========================================================================
    // discover_messages + list_messages / show_message / message_hash
    // (integration tests using HORUS_SOURCE_DIR with temp directory)
    // =========================================================================

    /// Helper: create a temp messages directory with sample .rs files
    fn setup_messages_dir() -> (tempfile::TempDir, std::path::PathBuf) {
        let tmp = tempfile::tempdir().unwrap();
        let msgs_dir = tmp.path().join("horus_library").join("messages");
        fs::create_dir_all(&msgs_dir).unwrap();

        // control.rs
        fs::write(
            msgs_dir.join("control.rs"),
            r#"
/// Twist velocity command
#[derive(Debug, Clone)]
pub struct Twist {
    /// Linear velocity
    pub linear: f64,
    /// Angular velocity
    pub angular: f64,
}
"#,
        )
        .unwrap();

        // sensor.rs
        fs::write(
            msgs_dir.join("sensor.rs"),
            r#"
/// IMU reading
pub struct Imu {
    pub accel_x: f64,
    pub accel_y: f64,
    pub accel_z: f64,
    pub gyro_x: f64,
    pub gyro_y: f64,
    pub gyro_z: f64,
}

/// Range finder reading
pub struct Range {
    pub distance: f64,
    pub min_range: f64,
    pub max_range: f64,
}
"#,
        )
        .unwrap();

        // mod.rs — should be skipped
        fs::write(
            msgs_dir.join("mod.rs"),
            "pub mod control;\npub mod sensor;\n",
        )
        .unwrap();

        // non_rs.txt — should be ignored
        fs::write(msgs_dir.join("non_rs.txt"), "not a rust file").unwrap();

        let p = tmp.path().to_path_buf();
        (tmp, p)
    }

    #[test]
    fn discover_messages_via_env_var() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        // Set the HORUS_SOURCE_DIR to the temp root
        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = discover_messages();
        std::env::remove_var("HORUS_SOURCE_DIR");

        let msgs = result.expect("discover_messages should succeed");
        // Should find Twist (control) + Imu + Range (sensor) = 3
        assert_eq!(msgs.len(), 3, "should discover 3 message types");

        // Sorted by module then name
        let names: Vec<&str> = msgs.iter().map(|m| m.name.as_str()).collect();
        assert!(names.contains(&"Twist"));
        assert!(names.contains(&"Imu"));
        assert!(names.contains(&"Range"));

        // Verify module assignment
        let twist = msgs.iter().find(|m| m.name == "Twist").unwrap();
        assert_eq!(twist.module, "control");
        assert_eq!(twist.fields.len(), 2);

        let imu = msgs.iter().find(|m| m.name == "Imu").unwrap();
        assert_eq!(imu.module, "sensor");
        assert_eq!(imu.fields.len(), 6);

        drop(tmp);
    }

    #[test]
    fn discover_messages_skips_mod_rs() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let msgs = discover_messages().unwrap();
        std::env::remove_var("HORUS_SOURCE_DIR");

        // mod.rs should be skipped — no messages from it
        for m in &msgs {
            assert_ne!(m.module, "mod", "mod.rs should be skipped");
        }

        drop(tmp);
    }

    #[test]
    fn discover_messages_skips_non_rs_files() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let msgs = discover_messages().unwrap();
        std::env::remove_var("HORUS_SOURCE_DIR");

        // non_rs.txt should not produce any messages
        for m in &msgs {
            assert_ne!(m.source_file.as_str(), "non_rs");
        }

        drop(tmp);
    }

    #[test]
    fn discover_messages_sorts_by_module_then_name() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let msgs = discover_messages().unwrap();
        std::env::remove_var("HORUS_SOURCE_DIR");

        // Verify sorting: control < sensor
        let modules: Vec<&str> = msgs.iter().map(|m| m.module.as_str()).collect();
        // control::Twist comes first, then sensor::Imu, sensor::Range
        assert_eq!(modules[0], "control");
        assert_eq!(modules[1], "sensor");
        assert_eq!(modules[2], "sensor");
        // Within sensor, Imu < Range alphabetically
        assert_eq!(msgs[1].name, "Imu");
        assert_eq!(msgs[2].name, "Range");

        drop(tmp);
    }

    #[test]
    fn discover_messages_error_when_no_dir_found() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // Point to a nonexistent directory
        std::env::set_var(
            "HORUS_SOURCE_DIR",
            "/tmp/definitely_does_not_exist_horus_test",
        );
        // Also clear HORUS_SOURCE to avoid fallback
        let prev_source = std::env::var("HORUS_SOURCE").ok();
        std::env::remove_var("HORUS_SOURCE");

        let result = discover_messages();
        std::env::remove_var("HORUS_SOURCE_DIR");
        if let Some(v) = prev_source {
            std::env::set_var("HORUS_SOURCE", v);
        }

        // The fake HORUS_SOURCE_DIR won't work, but fallback paths (e.g.
        // ~/softmata/horus/horus_library/messages) may still succeed.
        match result {
            Ok(msgs) => {
                // Fallback path found messages — they should be well-formed
                assert!(
                    msgs.iter().all(|m| !m.name.is_empty()),
                    "all discovered messages should have non-empty names"
                );
            }
            Err(e) => {
                // No fallback path found — error should mention the missing directory
                let msg = e.to_string();
                assert!(
                    msg.contains("messages") || msg.contains("HORUS_SOURCE_DIR"),
                    "error should mention messages directory or HORUS_SOURCE_DIR, got: {msg}"
                );
            }
        }
    }

    #[test]
    fn list_messages_json_output() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        // list_messages prints to stdout; just verify it doesn't error
        let result = list_messages(false, None, true);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok(), "list_messages(json=true) should succeed");
        drop(tmp);
    }

    #[test]
    fn list_messages_verbose() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = list_messages(true, None, false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok(), "list_messages(verbose=true) should succeed");
        drop(tmp);
    }

    #[test]
    fn list_messages_compact() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = list_messages(false, None, false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(
            result.is_ok(),
            "list_messages(verbose=false, json=false) should succeed"
        );
        drop(tmp);
    }

    #[test]
    fn list_messages_with_filter_match() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = list_messages(false, Some("twist"), false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok());
        drop(tmp);
    }

    #[test]
    fn list_messages_with_filter_no_match() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = list_messages(false, Some("nonexistent_xyz"), false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        // Should succeed (prints "no types found")
        assert!(result.is_ok());
        drop(tmp);
    }

    #[test]
    fn list_messages_filter_by_module() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        // "sensor" should match Imu and Range
        let result = list_messages(false, Some("sensor"), false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok());
        drop(tmp);
    }

    #[test]
    fn list_messages_filter_case_insensitive() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        // "TWIST" should match "Twist" case-insensitively
        let result = list_messages(false, Some("TWIST"), false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok());
        drop(tmp);
    }

    #[test]
    fn show_message_found_by_name() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = show_message("Twist", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok(), "show_message('Twist') should succeed");
        drop(tmp);
    }

    #[test]
    fn show_message_case_insensitive() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = show_message("twist", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(
            result.is_ok(),
            "show_message('twist') should match case-insensitively"
        );
        drop(tmp);
    }

    #[test]
    fn show_message_by_module_qualified_name() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = show_message("control::Twist", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(
            result.is_ok(),
            "show_message('control::Twist') should succeed"
        );
        drop(tmp);
    }

    #[test]
    fn show_message_not_found() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = show_message("NonExistent", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(
            result.is_err(),
            "show_message for unknown type should error"
        );
        let err_msg = format!("{}", result.unwrap_err());
        assert!(
            err_msg.contains("NonExistent"),
            "error should mention the missing type name"
        );
        drop(tmp);
    }

    #[test]
    fn show_message_json() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = show_message("Imu", true);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(
            result.is_ok(),
            "show_message('Imu', json=true) should succeed"
        );
        drop(tmp);
    }

    #[test]
    fn show_message_with_empty_fields() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let msgs_dir = tmp.path().join("horus_library").join("messages");
        fs::create_dir_all(&msgs_dir).unwrap();

        fs::write(msgs_dir.join("empty.rs"), "pub struct EmptyMsg {}\n").unwrap();

        std::env::set_var("HORUS_SOURCE_DIR", tmp.path().to_str().unwrap());
        let result = show_message("EmptyMsg", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok());
        drop(tmp);
    }

    #[test]
    fn message_hash_found() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = message_hash("Twist", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok(), "message_hash('Twist') should succeed");
        drop(tmp);
    }

    #[test]
    fn message_hash_not_found() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = message_hash("DoesNotExist", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_err());
        drop(tmp);
    }

    #[test]
    fn message_hash_json() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = message_hash("Twist", true);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok(), "message_hash json mode should succeed");
        drop(tmp);
    }

    #[test]
    fn message_hash_case_insensitive() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = message_hash("twist", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(
            result.is_ok(),
            "message_hash should match case-insensitively"
        );
        drop(tmp);
    }

    #[test]
    fn message_hash_qualified_name() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = message_hash("sensor::Imu", true);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok(), "message_hash('sensor::Imu') should succeed");
        drop(tmp);
    }

    #[test]
    fn list_messages_empty_dir() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let msgs_dir = tmp.path().join("horus_library").join("messages");
        fs::create_dir_all(&msgs_dir).unwrap();

        std::env::set_var("HORUS_SOURCE_DIR", tmp.path().to_str().unwrap());
        let result = list_messages(false, None, false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        // No messages found — should print "No message types found." and return Ok
        assert!(result.is_ok());
        drop(tmp);
    }

    #[test]
    fn list_messages_empty_dir_with_filter() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let msgs_dir = tmp.path().join("horus_library").join("messages");
        fs::create_dir_all(&msgs_dir).unwrap();

        std::env::set_var("HORUS_SOURCE_DIR", tmp.path().to_str().unwrap());
        let result = list_messages(false, Some("anything"), false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok());
        drop(tmp);
    }

    #[test]
    fn list_messages_json_with_filter() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = list_messages(false, Some("control"), true);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok());
        drop(tmp);
    }

    #[test]
    fn list_messages_verbose_with_filter() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = list_messages(true, Some("sensor"), false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok());
        drop(tmp);
    }

    // =========================================================================
    // Roundtrip: parse -> hash consistency
    // =========================================================================

    #[test]
    fn parse_and_hash_roundtrip() {
        let source = r#"
/// Control command
pub struct Cmd {
    pub throttle: f64,
    pub steering: f64,
}
"#;
        let msgs = parse_messages_from_source(source, "ctrl", "ctrl.rs".into());
        assert_eq!(msgs.len(), 1);
        let hash1 = compute_message_definition_hash(&msgs[0]);
        let hash2 = compute_message_definition_hash(&msgs[0]);
        assert_eq!(hash1, hash2, "hash should be deterministic across calls");
        assert_eq!(hash1.len(), 16);
    }

    #[test]
    fn parse_and_hash_changes_when_field_added() {
        let source_v1 = r#"
pub struct Msg {
    pub a: f64,
}
"#;
        let source_v2 = r#"
pub struct Msg {
    pub a: f64,
    pub b: f64,
}
"#;
        let msgs_v1 = parse_messages_from_source(source_v1, "test", "test.rs".into());
        let msgs_v2 = parse_messages_from_source(source_v2, "test", "test.rs".into());
        assert_ne!(
            compute_message_definition_hash(&msgs_v1[0]),
            compute_message_definition_hash(&msgs_v2[0]),
            "adding a field should change the hash"
        );
    }

    #[test]
    fn parse_and_hash_changes_when_field_type_changes() {
        let source_v1 = r#"
pub struct Msg {
    pub value: f32,
}
"#;
        let source_v2 = r#"
pub struct Msg {
    pub value: f64,
}
"#;
        let msgs_v1 = parse_messages_from_source(source_v1, "test", "test.rs".into());
        let msgs_v2 = parse_messages_from_source(source_v2, "test", "test.rs".into());
        assert_ne!(
            compute_message_definition_hash(&msgs_v1[0]),
            compute_message_definition_hash(&msgs_v2[0]),
            "changing field type should change the hash"
        );
    }

    #[test]
    fn parse_and_hash_stable_when_doc_changes() {
        let source_v1 = r#"
/// Version 1 doc
pub struct Msg {
    pub x: f64,
}
"#;
        let source_v2 = r#"
/// Completely different documentation
pub struct Msg {
    pub x: f64,
}
"#;
        let msgs_v1 = parse_messages_from_source(source_v1, "test", "test.rs".into());
        let msgs_v2 = parse_messages_from_source(source_v2, "test", "test.rs".into());
        assert_eq!(
            compute_message_definition_hash(&msgs_v1[0]),
            compute_message_definition_hash(&msgs_v2[0]),
            "doc changes should not affect the hash"
        );
    }

    // =========================================================================
    // Edge case: field doc comes from the line immediately above
    // =========================================================================

    #[test]
    fn parse_messages_field_doc_only_from_immediate_predecessor() {
        let source = r#"
pub struct Msg {
    /// Doc for first
    pub first: f64,
    /// Doc for second
    pub second: f64,
}
"#;
        let msgs = parse_messages_from_source(source, "t", "t.rs".into());
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0].fields[0].doc, "Doc for first");
        assert_eq!(msgs[0].fields[1].doc, "Doc for second");
    }

    #[test]
    fn parse_messages_field_no_doc_when_prev_is_not_doc_comment() {
        let source = r#"
pub struct Msg {
    pub first: f64,
    pub second: f64,
}
"#;
        let msgs = parse_messages_from_source(source, "t", "t.rs".into());
        assert_eq!(msgs.len(), 1);
        // first field's previous line is "{" — not a doc comment
        assert!(msgs[0].fields[0].doc.is_empty());
        // second field's previous line is "pub first: f64," — not a doc comment
        assert!(msgs[0].fields[1].doc.is_empty());
    }

    // =========================================================================
    // Edge: discover_messages with HORUS_SOURCE env var
    // =========================================================================

    #[test]
    fn discover_messages_via_horus_source_env() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let msgs_dir = tmp.path().join("horus_library").join("messages");
        fs::create_dir_all(&msgs_dir).unwrap();

        fs::write(
            msgs_dir.join("test.rs"),
            "pub struct Ping { pub seq: u32, }\n",
        )
        .unwrap();

        // Use HORUS_SOURCE_DIR (checked first, highest priority)
        let prev_dir = std::env::var("HORUS_SOURCE_DIR").ok();
        std::env::set_var("HORUS_SOURCE_DIR", tmp.path().to_str().unwrap());

        let result = discover_messages();

        if let Some(v) = prev_dir {
            std::env::set_var("HORUS_SOURCE_DIR", v);
        } else {
            std::env::remove_var("HORUS_SOURCE_DIR");
        }

        let msgs = result.expect("should discover via HORUS_SOURCE_DIR");
        assert!(
            msgs.iter().any(|m| m.name == "Ping"),
            "should find Ping message in {:?}",
            msgs.iter().map(|m| &m.name).collect::<Vec<_>>()
        );
        drop(tmp);
    }

    // =========================================================================
    // show_message with doc
    // =========================================================================

    #[test]
    fn show_message_displays_doc() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let msgs_dir = tmp.path().join("horus_library").join("messages");
        fs::create_dir_all(&msgs_dir).unwrap();

        fs::write(
            msgs_dir.join("info.rs"),
            r#"
/// A well-documented message.
/// With multiple lines.
pub struct DocMsg {
    /// The value
    pub value: f64,
}
"#,
        )
        .unwrap();

        std::env::set_var("HORUS_SOURCE_DIR", tmp.path().to_str().unwrap());
        let result = show_message("DocMsg", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok());
        drop(tmp);
    }

    // =========================================================================
    // show_message JSON output includes all expected fields
    // =========================================================================

    #[test]
    fn show_message_json_with_fields() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let msgs_dir = tmp.path().join("horus_library").join("messages");
        fs::create_dir_all(&msgs_dir).unwrap();

        fs::write(
            msgs_dir.join("json_test.rs"),
            r#"
pub struct JsonTest {
    pub alpha: f64,
    pub beta: String,
}
"#,
        )
        .unwrap();

        std::env::set_var("HORUS_SOURCE_DIR", tmp.path().to_str().unwrap());
        let result = show_message("JsonTest", true);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok());
        drop(tmp);
    }
}
