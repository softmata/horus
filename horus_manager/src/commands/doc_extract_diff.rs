//! API diff — compare current API surface against a saved baseline.
//!
//! Detects added, removed, changed, and deprecated symbols.
//! Reports breaking changes and exits non-zero for CI gating.

use super::doc_extract::*;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::fmt::Write;

// ─── Types ──────────────────────────────────────────────────────────────────

/// Full API diff result.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApiDiff {
    pub added: Vec<DiffEntry>,
    pub removed: Vec<DiffEntry>,
    pub changed: Vec<ChangedEntry>,
    pub deprecated: Vec<DiffEntry>,
    pub topics_added: Vec<String>,
    pub topics_removed: Vec<String>,
    pub breaking_changes: usize,
    pub summary: DiffSummary,
}

/// A symbol that was added or removed.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DiffEntry {
    pub name: String,
    pub kind: String,
    pub file: String,
    pub signature: Option<String>,
    pub line: Option<usize>,
}

/// A symbol whose signature changed.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ChangedEntry {
    pub name: String,
    pub kind: String,
    pub file: String,
    pub old_signature: String,
    pub new_signature: String,
    pub is_breaking: bool,
    pub description: String,
}

/// Summary counts.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DiffSummary {
    pub added: usize,
    pub removed: usize,
    pub changed: usize,
    pub deprecated: usize,
}

// ─── Symbol Key ─────────────────────────────────────────────────────────────

/// Identity of a symbol: (file_stem, name, kind).
type SymbolKey = (String, String, String);

struct SymbolInfo {
    signature: String,
    deprecated: Option<String>,
    file: String,
    line: usize,
    kind: String,
}

fn symbol_kind_str(sym: &SymbolDoc) -> &'static str {
    match sym {
        SymbolDoc::Function(_) => "function",
        SymbolDoc::Struct(_) => "struct",
        SymbolDoc::Enum(_) => "enum",
        SymbolDoc::Trait(_) => "trait",
        SymbolDoc::TypeAlias(_) => "type_alias",
        SymbolDoc::Constant(_) => "constant",
        SymbolDoc::HorusMessage(_) => "message",
        SymbolDoc::HorusService(_) => "service",
        SymbolDoc::HorusAction(_) => "action",
    }
}

fn symbol_signature(sym: &SymbolDoc) -> String {
    match sym {
        SymbolDoc::Function(f) => f.signature.clone(),
        SymbolDoc::Struct(s) => {
            let fields: Vec<String> = s.fields.iter().map(|f| {
                format!("{}: {}", f.name, f.type_str.as_deref().unwrap_or("?"))
            }).collect();
            format!("struct {} {{ {} }}", s.name, fields.join(", "))
        }
        SymbolDoc::Enum(e) => {
            let variants: Vec<&str> = e.variants.iter().map(|v| v.name.as_str()).collect();
            format!("enum {} {{ {} }}", e.name, variants.join(", "))
        }
        SymbolDoc::Trait(t) => {
            let methods: Vec<&str> = t.required_methods.iter().map(|m| m.name.as_str()).collect();
            format!("trait {} {{ {} }}", t.name, methods.join(", "))
        }
        SymbolDoc::TypeAlias(t) => format!("type {} = {}", t.name, t.target_type),
        SymbolDoc::Constant(c) => format!("const {}: {}", c.name, c.type_str),
        SymbolDoc::HorusMessage(m) => {
            let fields: Vec<String> = m.fields.iter().map(|f| {
                format!("{}: {}", f.name, f.type_str.as_deref().unwrap_or("?"))
            }).collect();
            format!("message {} {{ {} }}", m.name, fields.join(", "))
        }
        SymbolDoc::HorusService(s) => format!("service {}", s.name),
        SymbolDoc::HorusAction(a) => format!("action {}", a.name),
    }
}

fn build_symbol_map(doc: &ProjectDoc) -> HashMap<SymbolKey, SymbolInfo> {
    let mut map = HashMap::new();
    for module in &doc.modules {
        let file_stem = module.path.file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_default();
        for sym in &module.symbols {
            let key = (file_stem.clone(), sym.name().to_string(), symbol_kind_str(sym).to_string());
            let info = SymbolInfo {
                signature: symbol_signature(sym),
                deprecated: sym.deprecated().map(|s| s.to_string()),
                file: module.path.display().to_string(),
                line: sym.location().map(|l| l.line).unwrap_or(0),
                kind: symbol_kind_str(sym).to_string(),
            };
            map.insert(key, info);
        }
    }
    map
}

// ─── Diff Computation ───────────────────────────────────────────────────────

/// Compare baseline against current and produce an ApiDiff.
pub fn compute_diff(baseline: &ProjectDoc, current: &ProjectDoc) -> ApiDiff {
    let baseline_map = build_symbol_map(baseline);
    let current_map = build_symbol_map(current);

    let mut added = Vec::new();
    let mut removed = Vec::new();
    let mut changed = Vec::new();
    let mut deprecated = Vec::new();
    let mut breaking = 0;

    // Added: in current but not baseline
    for (key, info) in &current_map {
        if !baseline_map.contains_key(key) {
            added.push(DiffEntry {
                name: key.1.clone(),
                kind: info.kind.clone(),
                file: info.file.clone(),
                signature: Some(info.signature.clone()),
                line: Some(info.line),
            });
        }
    }

    // Removed: in baseline but not current (BREAKING)
    for (key, info) in &baseline_map {
        if !current_map.contains_key(key) {
            removed.push(DiffEntry {
                name: key.1.clone(),
                kind: info.kind.clone(),
                file: info.file.clone(),
                signature: Some(info.signature.clone()),
                line: Some(info.line),
            });
            breaking += 1;
        }
    }

    // Changed: in both but signature differs
    for (key, baseline_info) in &baseline_map {
        if let Some(current_info) = current_map.get(key) {
            // Signature changed
            if baseline_info.signature != current_info.signature {
                let is_breaking = detect_breaking_signature_change(
                    &baseline_info.signature,
                    &current_info.signature,
                    &baseline_info.kind,
                );
                let description = describe_change(
                    &baseline_info.signature,
                    &current_info.signature,
                    is_breaking,
                );
                if is_breaking {
                    breaking += 1;
                }
                changed.push(ChangedEntry {
                    name: key.1.clone(),
                    kind: current_info.kind.clone(),
                    file: current_info.file.clone(),
                    old_signature: baseline_info.signature.clone(),
                    new_signature: current_info.signature.clone(),
                    is_breaking,
                    description,
                });
            }

            // Newly deprecated
            if baseline_info.deprecated.is_none() && current_info.deprecated.is_some() {
                deprecated.push(DiffEntry {
                    name: key.1.clone(),
                    kind: current_info.kind.clone(),
                    file: current_info.file.clone(),
                    signature: current_info.deprecated.clone(),
                    line: Some(current_info.line),
                });
            }
        }
    }

    // Topic changes
    let baseline_topics: HashSet<String> = baseline.message_graph.as_ref()
        .map(|g| g.topics.iter().map(|t| t.name.clone()).collect())
        .unwrap_or_default();
    let current_topics: HashSet<String> = current.message_graph.as_ref()
        .map(|g| g.topics.iter().map(|t| t.name.clone()).collect())
        .unwrap_or_default();

    let topics_added: Vec<String> = current_topics.difference(&baseline_topics).cloned().collect();
    let topics_removed: Vec<String> = baseline_topics.difference(&current_topics).cloned().collect();

    let summary = DiffSummary {
        added: added.len(),
        removed: removed.len(),
        changed: changed.len(),
        deprecated: deprecated.len(),
    };

    ApiDiff {
        added,
        removed,
        changed,
        deprecated,
        topics_added,
        topics_removed,
        breaking_changes: breaking,
        summary,
    }
}

// ─── Breaking Change Detection ──────────────────────────────────────────────

fn detect_breaking_signature_change(old: &str, new: &str, kind: &str) -> bool {
    match kind {
        "function" => {
            // Parameter removed or type changed is breaking
            // Simple heuristic: if old has more params or different param types
            let old_params = extract_param_count(old);
            let new_params = extract_param_count(new);
            if new_params < old_params {
                return true; // param removed
            }
            // Return type changed
            let old_ret = extract_return_type_str(old);
            let new_ret = extract_return_type_str(new);
            if old_ret != new_ret {
                return true;
            }
            // Param types changed (conservative: any difference is breaking)
            old_params != new_params || old != new
        }
        "struct" => {
            // Field removed is breaking
            let old_fields = extract_brace_contents(old);
            let new_fields = extract_brace_contents(new);
            // Check if any old field is missing from new
            for field in old_fields.split(',') {
                let field_name = field.split(':').next().unwrap_or("").trim();
                if !field_name.is_empty() && !new_fields.contains(field_name) {
                    return true;
                }
            }
            false
        }
        "enum" => {
            // Variant removed is breaking
            let old_variants = extract_brace_contents(old);
            let new_variants = extract_brace_contents(new);
            for variant in old_variants.split(',') {
                let v = variant.trim();
                if !v.is_empty() && !new_variants.contains(v) {
                    return true;
                }
            }
            false
        }
        "trait" => {
            // New required method added is breaking
            let old_methods = extract_brace_contents(old);
            let new_methods = extract_brace_contents(new);
            let old_count = old_methods.split(',').filter(|s| !s.trim().is_empty()).count();
            let new_count = new_methods.split(',').filter(|s| !s.trim().is_empty()).count();
            new_count > old_count
        }
        _ => false,
    }
}

fn extract_param_count(sig: &str) -> usize {
    if let Some(start) = sig.find('(') {
        if let Some(end) = sig.rfind(')') {
            let params = &sig[start + 1..end];
            if params.trim().is_empty() {
                return 0;
            }
            // Count commas, accounting for generic angle brackets
            let mut count = 1;
            let mut depth = 0;
            for ch in params.chars() {
                match ch {
                    '<' => depth += 1,
                    '>' => depth -= 1,
                    ',' if depth == 0 => count += 1,
                    _ => {}
                }
            }
            return count;
        }
    }
    0
}

fn extract_return_type_str(sig: &str) -> &str {
    if let Some(pos) = sig.rfind("->") {
        sig[pos + 2..].trim()
    } else {
        ""
    }
}

fn extract_brace_contents(s: &str) -> &str {
    if let Some(start) = s.find('{') {
        if let Some(end) = s.rfind('}') {
            return s[start + 1..end].trim();
        }
    }
    ""
}

fn describe_change(_old: &str, _new: &str, is_breaking: bool) -> String {
    let breaking_str = if is_breaking { " (BREAKING)" } else { "" };
    format!("signature changed{breaking_str}")
}

// ─── Formatters ─────────────────────────────────────────────────────────────

/// Format diff as human-readable text.
pub fn format_diff_text(diff: &ApiDiff) -> String {
    let mut out = String::new();

    if diff.added.is_empty()
        && diff.removed.is_empty()
        && diff.changed.is_empty()
        && diff.deprecated.is_empty()
        && diff.topics_added.is_empty()
        && diff.topics_removed.is_empty()
    {
        let _ = writeln!(out, "No API changes detected.");
        return out;
    }

    let _ = writeln!(out, "API Changes:\n");

    if !diff.added.is_empty() {
        let _ = writeln!(out, "  Added:");
        for entry in &diff.added {
            let sig = entry.signature.as_deref().unwrap_or("");
            let _ = writeln!(out, "    + {}: {} {}", entry.file, entry.kind, sig);
        }
        let _ = writeln!(out);
    }

    if !diff.removed.is_empty() {
        let _ = writeln!(out, "  Removed:  \u{26a0} BREAKING");
        for entry in &diff.removed {
            let sig = entry.signature.as_deref().unwrap_or("");
            let _ = writeln!(out, "    - {}: {} {}", entry.file, entry.kind, sig);
        }
        let _ = writeln!(out);
    }

    if !diff.changed.is_empty() {
        let _ = writeln!(out, "  Changed:");
        for entry in &diff.changed {
            let breaking = if entry.is_breaking { " \u{26a0} BREAKING" } else { "" };
            let _ = writeln!(out, "    ~ {}: {} {}{}", entry.file, entry.kind, entry.name, breaking);
            let _ = writeln!(out, "      was: {}", entry.old_signature);
            let _ = writeln!(out, "      now: {}", entry.new_signature);
        }
        let _ = writeln!(out);
    }

    if !diff.deprecated.is_empty() {
        let _ = writeln!(out, "  Deprecated:");
        for entry in &diff.deprecated {
            let note = entry.signature.as_deref().unwrap_or("");
            let _ = writeln!(out, "    \u{26a0} {}: {} {} — {}", entry.file, entry.kind, entry.name, note);
        }
        let _ = writeln!(out);
    }

    if !diff.topics_added.is_empty() {
        let _ = writeln!(out, "  Topics added: {}", diff.topics_added.join(", "));
    }
    if !diff.topics_removed.is_empty() {
        let _ = writeln!(out, "  Topics removed: {}", diff.topics_removed.join(", "));
    }

    let _ = writeln!(
        out,
        "\n  Summary: +{} added, -{} removed, ~{} changed, \u{26a0}{} deprecated, {} breaking changes",
        diff.summary.added, diff.summary.removed, diff.summary.changed,
        diff.summary.deprecated, diff.breaking_changes
    );

    out
}

/// Format diff as JSON.
pub fn format_diff_json(diff: &ApiDiff) -> String {
    serde_json::to_string_pretty(diff).unwrap_or_else(|e| format!("{{\"error\": \"{e}\"}}"))
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn empty_doc(name: &str) -> ProjectDoc {
        ProjectDoc {
            project: name.to_string(),
            version: "0.1.0".to_string(),
            languages: vec!["rust".to_string()],
            module_tree: ModuleTree { name: "root".to_string(), children: vec![] },
            modules: vec![],
            relationships: vec![],
            message_graph: None,
            entry_points: vec![],
            todos: vec![],
            stats: DocStats {
                total_files: 0, total_symbols: 0, documented_symbols: 0,
                deprecated_symbols: 0, documentation_coverage: 1.0,
                horus_nodes: 0, horus_messages: 0, horus_services: 0,
                horus_actions: 0, topics_discovered: 0, todos: 0, fixmes: 0,
            },
        }
    }

    fn make_fn_symbol(name: &str, sig: &str, deprecated: Option<&str>) -> SymbolDoc {
        SymbolDoc::Function(FunctionDoc {
            name: name.to_string(),
            visibility: Visibility::Public,
            location: SourceLocation { file: PathBuf::from("src/lib.rs"), line: 1, end_line: None },
            signature: sig.to_string(),
            doc: None,
            deprecated: deprecated.map(|s| s.to_string()),
            params: vec![],
            returns: None,
            is_async: false,
            generic_params: vec![],
            examples: vec![],
        })
    }

    fn make_struct_symbol(name: &str, fields: &[(&str, &str)]) -> SymbolDoc {
        SymbolDoc::Struct(StructDoc {
            name: name.to_string(),
            visibility: Visibility::Public,
            location: SourceLocation { file: PathBuf::from("src/lib.rs"), line: 1, end_line: None },
            doc: None, deprecated: None, generic_params: vec![],
            fields: fields.iter().map(|(n, t)| FieldDoc {
                name: n.to_string(), type_str: Some(t.to_string()), doc: None,
            }).collect(),
            methods: vec![], trait_impls: vec![], derives: vec![], examples: vec![],
        })
    }

    fn doc_with_symbols(symbols: Vec<SymbolDoc>) -> ProjectDoc {
        let mut doc = empty_doc("test");
        doc.modules = vec![ModuleDoc {
            path: PathBuf::from("src/lib.rs"),
            language: "rust".to_string(),
            module_doc: None,
            imports: vec![],
            symbols,
        }];
        doc.stats.total_symbols = doc.modules[0].symbols.len();
        doc
    }

    #[test]
    fn test_diff_added_symbol() {
        let baseline = empty_doc("test");
        let current = doc_with_symbols(vec![
            make_fn_symbol("new_fn", "pub fn new_fn()", None),
        ]);
        let diff = compute_diff(&baseline, &current);
        assert_eq!(diff.added.len(), 1);
        assert_eq!(diff.added[0].name, "new_fn");
        assert_eq!(diff.removed.len(), 0);
        assert_eq!(diff.breaking_changes, 0);
    }

    #[test]
    fn test_diff_removed_symbol() {
        let baseline = doc_with_symbols(vec![
            make_fn_symbol("old_fn", "pub fn old_fn()", None),
        ]);
        let current = empty_doc("test");
        let diff = compute_diff(&baseline, &current);
        assert_eq!(diff.removed.len(), 1);
        assert_eq!(diff.removed[0].name, "old_fn");
        assert_eq!(diff.breaking_changes, 1);
    }

    #[test]
    fn test_diff_changed_signature() {
        let baseline = doc_with_symbols(vec![
            make_fn_symbol("compute", "pub fn compute(x: f64) -> f64", None),
        ]);
        let current = doc_with_symbols(vec![
            make_fn_symbol("compute", "pub fn compute(x: f64, dt: f64) -> f64", None),
        ]);
        let diff = compute_diff(&baseline, &current);
        assert_eq!(diff.changed.len(), 1);
        assert_eq!(diff.changed[0].name, "compute");
        assert!(diff.changed[0].old_signature.contains("x: f64)"));
        assert!(diff.changed[0].new_signature.contains("dt: f64"));
    }

    #[test]
    fn test_diff_newly_deprecated() {
        let baseline = doc_with_symbols(vec![
            make_fn_symbol("old_api", "pub fn old_api()", None),
        ]);
        let current = doc_with_symbols(vec![
            make_fn_symbol("old_api", "pub fn old_api()", Some("use new_api")),
        ]);
        let diff = compute_diff(&baseline, &current);
        assert_eq!(diff.deprecated.len(), 1);
        assert_eq!(diff.deprecated[0].name, "old_api");
    }

    #[test]
    fn test_diff_no_changes() {
        let doc = doc_with_symbols(vec![
            make_fn_symbol("stable", "pub fn stable()", None),
        ]);
        let diff = compute_diff(&doc, &doc);
        assert!(diff.added.is_empty());
        assert!(diff.removed.is_empty());
        assert!(diff.changed.is_empty());
        assert!(diff.deprecated.is_empty());
        assert_eq!(diff.breaking_changes, 0);
    }

    #[test]
    fn test_diff_breaking_field_removed() {
        let baseline = doc_with_symbols(vec![
            make_struct_symbol("Config", &[("name", "String"), ("verbose", "bool")]),
        ]);
        let current = doc_with_symbols(vec![
            make_struct_symbol("Config", &[("name", "String")]),
        ]);
        let diff = compute_diff(&baseline, &current);
        assert_eq!(diff.changed.len(), 1);
        assert!(diff.changed[0].is_breaking, "removing a field should be breaking");
        assert!(diff.breaking_changes > 0);
    }

    #[test]
    fn test_diff_breaking_return_type_changed() {
        let baseline = doc_with_symbols(vec![
            make_fn_symbol("get", "pub fn get() -> String", None),
        ]);
        let current = doc_with_symbols(vec![
            make_fn_symbol("get", "pub fn get() -> Option<String>", None),
        ]);
        let diff = compute_diff(&baseline, &current);
        assert_eq!(diff.changed.len(), 1);
        assert!(diff.changed[0].is_breaking);
    }

    #[test]
    fn test_diff_against_empty_baseline() {
        let baseline = empty_doc("test");
        let current = doc_with_symbols(vec![
            make_fn_symbol("a", "pub fn a()", None),
            make_fn_symbol("b", "pub fn b()", None),
        ]);
        let diff = compute_diff(&baseline, &current);
        assert_eq!(diff.added.len(), 2);
        assert_eq!(diff.removed.len(), 0);
        assert_eq!(diff.breaking_changes, 0);
    }

    #[test]
    fn test_diff_identical() {
        let doc = doc_with_symbols(vec![
            make_fn_symbol("f", "pub fn f()", None),
        ]);
        let diff = compute_diff(&doc, &doc);
        let text = format_diff_text(&diff);
        assert!(text.contains("No API changes"));
    }

    #[test]
    fn test_format_diff_text_markers() {
        let baseline = doc_with_symbols(vec![
            make_fn_symbol("old", "pub fn old()", None),
        ]);
        let current = doc_with_symbols(vec![
            make_fn_symbol("new", "pub fn new()", None),
        ]);
        let diff = compute_diff(&baseline, &current);
        let text = format_diff_text(&diff);
        assert!(text.contains("+"), "should have + for added");
        assert!(text.contains("-"), "should have - for removed");
        assert!(text.contains("BREAKING"));
    }

    #[test]
    fn test_format_diff_json_valid() {
        let baseline = empty_doc("test");
        let current = doc_with_symbols(vec![
            make_fn_symbol("f", "pub fn f()", None),
        ]);
        let diff = compute_diff(&baseline, &current);
        let json = format_diff_json(&diff);
        let parsed: serde_json::Value = serde_json::from_str(&json).expect("should be valid JSON");
        assert!(parsed.is_object());
        assert!(parsed["added"].is_array());
        assert!(parsed["breaking_changes"].is_number());
    }

    #[test]
    fn test_diff_topic_added() {
        let baseline = empty_doc("test");
        let mut current = empty_doc("test");
        current.message_graph = Some(MessageGraph {
            nodes: vec!["A".to_string()],
            topics: vec![MessageGraphTopic {
                name: "cmd_vel".to_string(),
                message_type: "CmdVel".to_string(),
                publishers: vec!["A".to_string()],
                subscribers: vec![],
            }],
        });
        let diff = compute_diff(&baseline, &current);
        assert_eq!(diff.topics_added, vec!["cmd_vel"]);
    }

    #[test]
    fn test_diff_topic_removed() {
        let mut baseline = empty_doc("test");
        baseline.message_graph = Some(MessageGraph {
            nodes: vec!["A".to_string()],
            topics: vec![MessageGraphTopic {
                name: "odom".to_string(),
                message_type: "Odom".to_string(),
                publishers: vec!["A".to_string()],
                subscribers: vec![],
            }],
        });
        let current = empty_doc("test");
        let diff = compute_diff(&baseline, &current);
        assert_eq!(diff.topics_removed, vec!["odom"]);
    }

    #[test]
    fn test_extract_param_count() {
        assert_eq!(extract_param_count("pub fn f()"), 0);
        assert_eq!(extract_param_count("pub fn f(x: i32)"), 1);
        assert_eq!(extract_param_count("pub fn f(x: i32, y: f64)"), 2);
        assert_eq!(extract_param_count("pub fn f(x: Vec<i32>, y: f64)"), 2);
        assert_eq!(extract_param_count("pub fn f(&self, x: i32)"), 2);
    }

    #[test]
    fn test_extract_return_type() {
        assert_eq!(extract_return_type_str("pub fn f() -> String"), "String");
        assert_eq!(extract_return_type_str("pub fn f()"), "");
        assert_eq!(extract_return_type_str("pub fn f() -> Option<String>"), "Option<String>");
    }

    #[test]
    fn test_diff_breaking_enum_variant_removed() {
        let baseline = doc_with_symbols(vec![
            SymbolDoc::Enum(EnumDoc {
                name: "Color".to_string(),
                visibility: Visibility::Public,
                location: SourceLocation { file: PathBuf::from("src/lib.rs"), line: 1, end_line: None },
                doc: None,
                deprecated: None,
                variants: vec![
                    VariantDoc { name: "Red".to_string(), doc: None, fields: vec![] },
                    VariantDoc { name: "Green".to_string(), doc: None, fields: vec![] },
                    VariantDoc { name: "Blue".to_string(), doc: None, fields: vec![] },
                ],
                methods: vec![],
            }),
        ]);
        let current = doc_with_symbols(vec![
            SymbolDoc::Enum(EnumDoc {
                name: "Color".to_string(),
                visibility: Visibility::Public,
                location: SourceLocation { file: PathBuf::from("src/lib.rs"), line: 1, end_line: None },
                doc: None,
                deprecated: None,
                variants: vec![
                    VariantDoc { name: "Red".to_string(), doc: None, fields: vec![] },
                    VariantDoc { name: "Green".to_string(), doc: None, fields: vec![] },
                ],
                methods: vec![],
            }),
        ]);
        let diff = compute_diff(&baseline, &current);
        assert_eq!(diff.changed.len(), 1, "should detect enum change");
        assert!(diff.changed[0].is_breaking, "removing an enum variant should be breaking");
        assert!(diff.breaking_changes > 0);
    }

    #[test]
    fn test_diff_breaking_trait_method_added() {
        let baseline = doc_with_symbols(vec![
            SymbolDoc::Trait(TraitDoc {
                name: "Driver".to_string(),
                visibility: Visibility::Public,
                location: SourceLocation { file: PathBuf::from("src/lib.rs"), line: 1, end_line: None },
                doc: None,
                deprecated: None,
                generic_params: vec![],
                required_methods: vec![
                    FunctionDoc {
                        name: "init".to_string(),
                        visibility: Visibility::Public,
                        location: SourceLocation { file: PathBuf::from("src/lib.rs"), line: 2, end_line: None },
                        signature: "fn init(&mut self)".to_string(),
                        doc: None, deprecated: None, params: vec![],
                        returns: None, is_async: false, generic_params: vec![], examples: vec![],
                    },
                ],
                provided_methods: vec![],
                implementors: vec![],
            }),
        ]);
        let current = doc_with_symbols(vec![
            SymbolDoc::Trait(TraitDoc {
                name: "Driver".to_string(),
                visibility: Visibility::Public,
                location: SourceLocation { file: PathBuf::from("src/lib.rs"), line: 1, end_line: None },
                doc: None,
                deprecated: None,
                generic_params: vec![],
                required_methods: vec![
                    FunctionDoc {
                        name: "init".to_string(),
                        visibility: Visibility::Public,
                        location: SourceLocation { file: PathBuf::from("src/lib.rs"), line: 2, end_line: None },
                        signature: "fn init(&mut self)".to_string(),
                        doc: None, deprecated: None, params: vec![],
                        returns: None, is_async: false, generic_params: vec![], examples: vec![],
                    },
                    FunctionDoc {
                        name: "shutdown".to_string(),
                        visibility: Visibility::Public,
                        location: SourceLocation { file: PathBuf::from("src/lib.rs"), line: 5, end_line: None },
                        signature: "fn shutdown(&mut self)".to_string(),
                        doc: None, deprecated: None, params: vec![],
                        returns: None, is_async: false, generic_params: vec![], examples: vec![],
                    },
                ],
                provided_methods: vec![],
                implementors: vec![],
            }),
        ]);
        let diff = compute_diff(&baseline, &current);
        assert_eq!(diff.changed.len(), 1, "should detect trait change");
        assert!(diff.changed[0].is_breaking, "adding a required trait method should be breaking");
        assert!(diff.breaking_changes > 0);
    }

    #[test]
    fn test_diff_doc_only_not_breaking() {
        let baseline = doc_with_symbols(vec![
            make_fn_symbol("stable", "pub fn stable(x: i32) -> bool", None),
        ]);
        // Same name, same signature — only doc differs (which is not in the signature)
        let current = doc_with_symbols(vec![
            make_fn_symbol("stable", "pub fn stable(x: i32) -> bool", None),
        ]);
        let diff = compute_diff(&baseline, &current);
        assert!(diff.changed.is_empty(), "doc-only change should not appear as changed since signatures match");
        assert_eq!(diff.breaking_changes, 0);
    }

    // ─── Level 2: Integration Tests ─────────────────────────────────────────

    #[test]
    fn test_integration_diff_multiple_changes() {
        // Baseline: fn a, fn b, struct C
        let baseline = doc_with_symbols(vec![
            make_fn_symbol("a", "pub fn a()", None),
            make_fn_symbol("b", "pub fn b(x: i32) -> i32", None),
            make_struct_symbol("C", &[("field1", "String")]),
        ]);
        // Current: remove fn a, change fn b signature, add fn d
        let current = doc_with_symbols(vec![
            make_fn_symbol("b", "pub fn b(x: i32, y: i32) -> i32", None),
            make_struct_symbol("C", &[("field1", "String")]),
            make_fn_symbol("d", "pub fn d() -> bool", None),
        ]);

        let diff = compute_diff(&baseline, &current);

        // Removed: fn a
        assert!(
            diff.removed.iter().any(|e| e.name == "a"),
            "removed should contain 'a', got: {:?}",
            diff.removed.iter().map(|e| &e.name).collect::<Vec<_>>()
        );
        // Changed: fn b (signature changed)
        assert!(
            diff.changed.iter().any(|e| e.name == "b"),
            "changed should contain 'b', got: {:?}",
            diff.changed.iter().map(|e| &e.name).collect::<Vec<_>>()
        );
        // Added: fn d
        assert!(
            diff.added.iter().any(|e| e.name == "d"),
            "added should contain 'd', got: {:?}",
            diff.added.iter().map(|e| &e.name).collect::<Vec<_>>()
        );
        // Removed fn is breaking
        assert!(
            diff.breaking_changes >= 1,
            "removing fn a should be breaking, got {} breaking changes",
            diff.breaking_changes
        );
    }

    #[test]
    fn test_integration_diff_format_roundtrip() {
        let baseline = doc_with_symbols(vec![
            make_fn_symbol("old", "pub fn old()", None),
        ]);
        let current = doc_with_symbols(vec![
            make_fn_symbol("new_fn", "pub fn new_fn(x: f64) -> f64", None),
        ]);
        let diff = compute_diff(&baseline, &current);

        // Serialize to JSON
        let json = format_diff_json(&diff);
        // Parse back as ApiDiff
        let roundtripped: ApiDiff = serde_json::from_str(&json)
            .expect("JSON produced by format_diff_json should parse back into ApiDiff");

        assert_eq!(roundtripped.added.len(), diff.added.len());
        assert_eq!(roundtripped.removed.len(), diff.removed.len());
        assert_eq!(roundtripped.changed.len(), diff.changed.len());
        assert_eq!(roundtripped.deprecated.len(), diff.deprecated.len());
        assert_eq!(roundtripped.breaking_changes, diff.breaking_changes);
        assert_eq!(roundtripped.summary.added, diff.summary.added);
        assert_eq!(roundtripped.summary.removed, diff.summary.removed);
        assert_eq!(roundtripped.summary.changed, diff.summary.changed);
        assert_eq!(roundtripped.summary.deprecated, diff.summary.deprecated);
    }

    // ─── Level 5: Error Path Tests ──────────────────────────────────────────

    #[test]
    fn test_error_diff_empty_baseline_empty_current() {
        let baseline = empty_doc("empty-baseline");
        let current = empty_doc("empty-current");
        let diff = compute_diff(&baseline, &current);

        assert!(diff.added.is_empty(), "no symbols to add");
        assert!(diff.removed.is_empty(), "no symbols to remove");
        assert!(diff.changed.is_empty(), "no symbols to change");
        assert!(diff.deprecated.is_empty(), "no symbols to deprecate");
        assert!(diff.topics_added.is_empty(), "no topics to add");
        assert!(diff.topics_removed.is_empty(), "no topics to remove");
        assert_eq!(diff.breaking_changes, 0, "empty diff has no breaking changes");

        // Summary counts must all be zero
        assert_eq!(diff.summary.added, 0);
        assert_eq!(diff.summary.removed, 0);
        assert_eq!(diff.summary.changed, 0);
        assert_eq!(diff.summary.deprecated, 0);

        // Text format should say "No API changes"
        let text = format_diff_text(&diff);
        assert!(text.contains("No API changes"), "empty-vs-empty should report no changes");
    }
}
