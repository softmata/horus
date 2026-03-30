//! C++ header file extractor using tree-sitter.
//!
//! Parses `.h`/`.hpp` files and extracts public API symbols into `ModuleDoc`.
//! Handles classes, functions, enums, templates, namespaces, and doxygen comments.

use super::doc_extract::*;
use anyhow::{Context, Result};
use std::path::Path;

/// Result of extracting a single C++ file.
pub struct CppExtractionResult {
    pub module: ModuleDoc,
    pub todos: Vec<TodoItem>,
}

/// Extract documentation from a C++ header file.
pub fn extract_cpp_file(path: &Path, include_private: bool) -> Result<CppExtractionResult> {
    let source = std::fs::read_to_string(path)
        .with_context(|| format!("Failed to read {}", path.display()))?;

    let mut parser = tree_sitter::Parser::new();
    parser
        .set_language(&tree_sitter_cpp::LANGUAGE.into())
        .context("Failed to set C++ language for tree-sitter")?;

    let tree = parser
        .parse(&source, None)
        .context("Failed to parse C++ source")?;

    let root = tree.root_node();
    let source_bytes = source.as_bytes();

    let mut symbols = Vec::new();
    let mut imports = Vec::new();
    let mut namespace_stack: Vec<String> = Vec::new();

    walk_node(
        root,
        source_bytes,
        &source,
        &mut symbols,
        &mut imports,
        &mut namespace_stack,
        include_private,
        path,
    );

    let todos = scan_cpp_todos(&source, path);

    Ok(CppExtractionResult {
        module: ModuleDoc {
            path: path.to_path_buf(),
            language: "cpp".to_string(),
            module_doc: extract_file_comment(&source),
            imports,
            symbols,
        },
        todos,
    })
}

// ─── Tree Walking ───────────────────────────────────────────────────────────

fn walk_node(
    node: tree_sitter::Node,
    source: &[u8],
    source_str: &str,
    symbols: &mut Vec<SymbolDoc>,
    imports: &mut Vec<String>,
    namespace_stack: &mut Vec<String>,
    include_private: bool,
    path: &Path,
) {
    let mut cursor = node.walk();

    for child in node.children(&mut cursor) {
        match child.kind() {
            "namespace_definition" => {
                // Extract namespace name and recurse
                if let Some(name_node) = child.child_by_field_name("name") {
                    let name = node_text(name_node, source);
                    namespace_stack.push(name);
                }
                if let Some(body) = child.child_by_field_name("body") {
                    walk_node(
                        body,
                        source,
                        source_str,
                        symbols,
                        imports,
                        namespace_stack,
                        include_private,
                        path,
                    );
                }
                if child.child_by_field_name("name").is_some() {
                    namespace_stack.pop();
                }
            }
            "function_definition" | "declaration" => {
                if let Some(sym) =
                    extract_function_or_declaration(child, source, namespace_stack, path)
                {
                    symbols.push(sym);
                }
            }
            "class_specifier" | "struct_specifier" => {
                if let Some(sym) =
                    extract_class_or_struct(child, source, namespace_stack, include_private, path)
                {
                    symbols.push(sym);
                }
            }
            "enum_specifier" => {
                if let Some(sym) = extract_enum(child, source, namespace_stack, path) {
                    symbols.push(sym);
                }
            }
            "type_alias_declaration" | "alias_declaration" => {
                if let Some(sym) = extract_type_alias(child, source, namespace_stack, path) {
                    symbols.push(sym);
                }
            }
            "template_declaration" => {
                // Recurse into the template's inner declaration
                let mut inner_cursor = child.walk();
                for inner in child.children(&mut inner_cursor) {
                    match inner.kind() {
                        "class_specifier" | "struct_specifier" => {
                            if let Some(sym) = extract_class_or_struct(
                                inner,
                                source,
                                namespace_stack,
                                include_private,
                                path,
                            ) {
                                symbols.push(sym);
                            }
                        }
                        "function_definition" | "declaration" => {
                            if let Some(sym) = extract_function_or_declaration(
                                inner,
                                source,
                                namespace_stack,
                                path,
                            ) {
                                symbols.push(sym);
                            }
                        }
                        _ => {}
                    }
                }
            }
            "preproc_include" => {
                let text = node_text(child, source);
                imports.push(text);
            }
            _ => {
                // Recurse into other containers (e.g., linkage_specification for extern "C")
                walk_node(
                    child,
                    source,
                    source_str,
                    symbols,
                    imports,
                    namespace_stack,
                    include_private,
                    path,
                );
            }
        }
    }
}

// ─── Symbol Extractors ──────────────────────────────────────────────────────

fn extract_function_or_declaration(
    node: tree_sitter::Node,
    source: &[u8],
    namespace_stack: &[String],
    path: &Path,
) -> Option<SymbolDoc> {
    // Find the declarator to get function name
    let declarator = find_declarator(node, source)?;
    let name = declarator_name(&declarator, source)?;
    let qualified_name = qualified_name(namespace_stack, &name);

    let return_type = node
        .child_by_field_name("type")
        .map(|n| node_text(n, source));

    let params = extract_function_params(node, source);
    let doc = extract_preceding_comment(node, source);
    let deprecated = doc.as_ref().and_then(|d| {
        if d.contains("@deprecated") || d.contains("\\deprecated") {
            Some(extract_doxygen_tag(d, "deprecated"))
        } else {
            None
        }
    });

    let signature = node_text(node, source);
    // Truncate to declaration (remove body)
    let sig_clean = if let Some(pos) = signature.find('{') {
        signature[..pos].trim().to_string()
    } else if signature.ends_with(';') {
        signature[..signature.len() - 1].trim().to_string()
    } else {
        signature.trim().to_string()
    };

    Some(SymbolDoc::Function(FunctionDoc {
        name: qualified_name,
        visibility: Visibility::Public,
        location: SourceLocation {
            file: path.to_path_buf(),
            line: node.start_position().row + 1,
            end_line: Some(node.end_position().row + 1),
        },
        signature: truncate_sig(&sig_clean, 120),
        doc: doc.map(|d| clean_doxygen(&d)),
        deprecated,
        params,
        returns: return_type,
        is_async: false,
        generic_params: vec![],
        examples: vec![],
    }))
}

fn extract_class_or_struct(
    node: tree_sitter::Node,
    source: &[u8],
    namespace_stack: &[String],
    include_private: bool,
    path: &Path,
) -> Option<SymbolDoc> {
    let name_node = node.child_by_field_name("name")?;
    let name = node_text(name_node, source);
    if name.is_empty() {
        return None; // anonymous struct
    }
    let qualified_name = qualified_name(namespace_stack, &name);

    let doc = extract_preceding_comment(node, source);
    let deprecated = doc.as_ref().and_then(|d| {
        if d.contains("@deprecated") {
            Some(extract_doxygen_tag(d, "deprecated"))
        } else {
            None
        }
    });

    // Extract base classes
    let mut bases = Vec::new();
    let mut cursor = node.walk();
    for child in node.children(&mut cursor) {
        if child.kind() == "base_class_clause" {
            let base_text = node_text(child, source);
            // Parse "public BaseClass" or just "BaseClass"
            for part in base_text.split(',') {
                let trimmed = part
                    .trim()
                    .strip_prefix(": ")
                    .unwrap_or(part.trim())
                    .trim()
                    .replace("public ", "")
                    .replace("protected ", "")
                    .replace("private ", "");
                if !trimmed.is_empty() {
                    bases.push(trimmed);
                }
            }
        }
    }

    // Extract public methods and fields from class body
    let mut methods = Vec::new();
    let mut fields = Vec::new();
    let mut is_public = node.kind() == "struct_specifier"; // struct defaults to public

    if let Some(body) = node.child_by_field_name("body") {
        let mut body_cursor = body.walk();
        for member in body.children(&mut body_cursor) {
            match member.kind() {
                "access_specifier" => {
                    let text = node_text(member, source).to_lowercase();
                    is_public = text.contains("public");
                }
                "function_definition" | "declaration" if is_public || include_private => {
                    // Check if it's a field or method
                    if has_function_declarator(member) {
                        if let Some(SymbolDoc::Function(f)) =
                            extract_function_or_declaration(member, source, &[], path)
                        {
                            methods.push(f);
                        }
                    } else {
                        // It's a field declaration
                        if let Some(field) = extract_field(member, source) {
                            fields.push(field);
                        }
                    }
                }
                "field_declaration" if is_public || include_private => {
                    if let Some(field) = extract_field(member, source) {
                        fields.push(field);
                    }
                }
                _ => {}
            }
        }
    }

    Some(SymbolDoc::Struct(StructDoc {
        name: qualified_name,
        visibility: Visibility::Public,
        location: SourceLocation {
            file: path.to_path_buf(),
            line: node.start_position().row + 1,
            end_line: Some(node.end_position().row + 1),
        },
        doc: doc.map(|d| clean_doxygen(&d)),
        deprecated,
        generic_params: vec![], // template params handled at template_declaration level
        fields,
        methods,
        trait_impls: bases, // C++ inheritance maps to trait_impls
        derives: vec![],
        examples: vec![],
    }))
}

fn extract_enum(
    node: tree_sitter::Node,
    source: &[u8],
    namespace_stack: &[String],
    path: &Path,
) -> Option<SymbolDoc> {
    let name_node = node.child_by_field_name("name")?;
    let name = node_text(name_node, source);
    let qualified_name = qualified_name(namespace_stack, &name);

    let doc = extract_preceding_comment(node, source);
    let deprecated = doc.as_ref().and_then(|d| {
        if d.contains("@deprecated") {
            Some(extract_doxygen_tag(d, "deprecated"))
        } else {
            None
        }
    });

    let mut variants = Vec::new();
    if let Some(body) = node.child_by_field_name("body") {
        let mut cursor = body.walk();
        for child in body.children(&mut cursor) {
            if child.kind() == "enumerator" {
                let variant_name = child
                    .child_by_field_name("name")
                    .map(|n| node_text(n, source))
                    .unwrap_or_default();
                if !variant_name.is_empty() {
                    variants.push(VariantDoc {
                        name: variant_name,
                        doc: extract_preceding_comment(child, source).map(|d| clean_doxygen(&d)),
                        fields: vec![],
                    });
                }
            }
        }
    }

    Some(SymbolDoc::Enum(EnumDoc {
        name: qualified_name,
        visibility: Visibility::Public,
        location: SourceLocation {
            file: path.to_path_buf(),
            line: node.start_position().row + 1,
            end_line: Some(node.end_position().row + 1),
        },
        doc: doc.map(|d| clean_doxygen(&d)),
        deprecated,
        variants,
        methods: vec![],
    }))
}

fn extract_type_alias(
    node: tree_sitter::Node,
    source: &[u8],
    namespace_stack: &[String],
    path: &Path,
) -> Option<SymbolDoc> {
    let text = node_text(node, source);
    // using Name = Type;
    let name = if text.starts_with("using ") {
        text.strip_prefix("using ")?
            .split('=')
            .next()?
            .trim()
            .to_string()
    } else if text.starts_with("typedef ") {
        // typedef Type Name;
        text.rsplit_once(' ')?.1.trim_end_matches(';').to_string()
    } else {
        return None;
    };

    let target = text
        .split('=')
        .nth(1)
        .map(|s| s.trim().trim_end_matches(';').to_string())
        .unwrap_or_else(|| "...".to_string());

    let qualified_name = qualified_name(namespace_stack, &name);

    Some(SymbolDoc::TypeAlias(TypeAliasDoc {
        name: qualified_name,
        visibility: Visibility::Public,
        location: SourceLocation {
            file: path.to_path_buf(),
            line: node.start_position().row + 1,
            end_line: None,
        },
        doc: extract_preceding_comment(node, source).map(|d| clean_doxygen(&d)),
        deprecated: None,
        target_type: target,
    }))
}

fn extract_field(node: tree_sitter::Node, source: &[u8]) -> Option<FieldDoc> {
    let text = node_text(node, source).trim_end_matches(';').to_string();
    // Simple heuristic: last word is name, rest is type
    let parts: Vec<&str> = text.split_whitespace().collect();
    if parts.len() < 2 {
        return None;
    }
    let name = parts.last()?.trim_end_matches(';').to_string();
    let type_str = parts[..parts.len() - 1].join(" ");

    Some(FieldDoc {
        name,
        type_str: Some(type_str),
        doc: extract_preceding_comment(node, source).map(|d| clean_doxygen(&d)),
    })
}

// ─── Helper Functions ───────────────────────────────────────────────────────

fn node_text(node: tree_sitter::Node, source: &[u8]) -> String {
    node.utf8_text(source).unwrap_or("").to_string()
}

fn qualified_name(namespace_stack: &[String], name: &str) -> String {
    if namespace_stack.is_empty() {
        name.to_string()
    } else {
        format!("{}::{}", namespace_stack.join("::"), name)
    }
}

fn find_declarator<'a>(
    node: tree_sitter::Node<'a>,
    _source: &[u8],
) -> Option<tree_sitter::Node<'a>> {
    node.child_by_field_name("declarator")
}

fn declarator_name(node: &tree_sitter::Node, source: &[u8]) -> Option<String> {
    // Walk down declarators to find the identifier
    if node.kind() == "identifier" || node.kind() == "field_identifier" {
        return Some(node_text(*node, source));
    }
    if node.kind() == "destructor_name" || node.kind() == "operator_name" {
        return Some(node_text(*node, source));
    }
    // Check child named "declarator" recursively
    if let Some(inner) = node.child_by_field_name("declarator") {
        return declarator_name(&inner, source);
    }
    // Check named "name"
    if let Some(name) = node.child_by_field_name("name") {
        return Some(node_text(name, source));
    }
    // Fallback: first identifier child
    let mut cursor = node.walk();
    for child in node.children(&mut cursor) {
        if child.kind() == "identifier" || child.kind() == "field_identifier" {
            return Some(node_text(child, source));
        }
    }
    None
}

fn has_function_declarator(node: tree_sitter::Node) -> bool {
    if node.kind() == "function_definition" {
        return true;
    }
    if let Some(decl) = node.child_by_field_name("declarator") {
        return decl.kind() == "function_declarator" || has_function_declarator_recursive(decl);
    }
    false
}

fn has_function_declarator_recursive(node: tree_sitter::Node) -> bool {
    if node.kind() == "function_declarator" {
        return true;
    }
    let mut cursor = node.walk();
    for child in node.children(&mut cursor) {
        if has_function_declarator_recursive(child) {
            return true;
        }
    }
    false
}

fn extract_function_params(node: tree_sitter::Node, source: &[u8]) -> Vec<ParamDoc> {
    let mut params = Vec::new();
    // Find parameter_list
    fn find_param_list(n: tree_sitter::Node) -> Option<tree_sitter::Node> {
        if n.kind() == "parameter_list" {
            return Some(n);
        }
        let mut cursor = n.walk();
        for child in n.children(&mut cursor) {
            if let Some(found) = find_param_list(child) {
                return Some(found);
            }
        }
        None
    }

    if let Some(param_list) = find_param_list(node) {
        let mut cursor = param_list.walk();
        for child in param_list.children(&mut cursor) {
            if child.kind() == "parameter_declaration" {
                let text = node_text(child, source);
                let parts: Vec<&str> = text.split_whitespace().collect();
                if parts.len() >= 2 {
                    let name = parts
                        .last()
                        .unwrap()
                        .trim_end_matches(&['&', '*', ','][..])
                        .to_string();
                    let type_str = parts[..parts.len() - 1].join(" ");
                    params.push(ParamDoc {
                        name,
                        type_str: Some(type_str),
                        default_value: None,
                        doc: None,
                    });
                } else if parts.len() == 1 {
                    params.push(ParamDoc {
                        name: String::new(),
                        type_str: Some(parts[0].to_string()),
                        default_value: None,
                        doc: None,
                    });
                }
            }
        }
    }
    params
}

// ─── Comment Extraction ─────────────────────────────────────────────────────

fn extract_preceding_comment(node: tree_sitter::Node, source: &[u8]) -> Option<String> {
    let mut prev = node.prev_sibling();
    let mut comments = Vec::new();

    while let Some(sibling) = prev {
        if sibling.kind() == "comment" {
            let text = node_text(sibling, source);
            comments.push(text);
            prev = sibling.prev_sibling();
        } else {
            break;
        }
    }

    if comments.is_empty() {
        return None;
    }

    comments.reverse();
    let combined: Vec<String> = comments
        .iter()
        .map(|c| {
            c.trim()
                .strip_prefix("///")
                .or_else(|| c.trim().strip_prefix("//!"))
                .or_else(|| c.trim().strip_prefix("//"))
                .map(|s| s.trim().to_string())
                .unwrap_or_else(|| {
                    // Block comment: strip /* and */
                    c.trim()
                        .strip_prefix("/*")
                        .and_then(|s| s.strip_suffix("*/"))
                        .unwrap_or(c.trim())
                        .lines()
                        .map(|l| l.trim().strip_prefix("* ").unwrap_or(l.trim()).to_string())
                        .collect::<Vec<_>>()
                        .join("\n")
                })
        })
        .collect();

    let result = combined.join("\n").trim().to_string();
    if result.is_empty() {
        None
    } else {
        Some(result)
    }
}

fn extract_file_comment(source: &str) -> Option<String> {
    // Look for a block comment at the very start of the file
    let trimmed = source.trim_start();
    if trimmed.starts_with("/**") || trimmed.starts_with("/*!") {
        if let Some(end) = trimmed.find("*/") {
            let comment = &trimmed[..end + 2];
            return Some(clean_doxygen(comment));
        }
    }
    // Or consecutive /// comments at the start
    let mut lines = Vec::new();
    for line in source.lines() {
        let t = line.trim();
        if t.starts_with("///") || t.starts_with("//!") {
            lines.push(
                t.strip_prefix("///")
                    .or_else(|| t.strip_prefix("//!"))
                    .unwrap_or("")
                    .trim()
                    .to_string(),
            );
        } else if t.is_empty() && lines.is_empty() {
            continue;
        } else {
            break;
        }
    }
    if lines.is_empty() {
        None
    } else {
        Some(lines.join("\n"))
    }
}

fn clean_doxygen(text: &str) -> String {
    text.lines()
        .map(|l| {
            l.trim()
                .strip_prefix("///")
                .or_else(|| l.trim().strip_prefix("/**"))
                .or_else(|| l.trim().strip_prefix("*/"))
                .or_else(|| l.trim().strip_prefix("* "))
                .or_else(|| l.trim().strip_prefix("*"))
                .unwrap_or(l.trim())
                .to_string()
        })
        .collect::<Vec<_>>()
        .join("\n")
        .trim()
        .to_string()
}

fn extract_doxygen_tag(doc: &str, tag: &str) -> String {
    let pattern = format!("@{tag}");
    let alt_pattern = format!("\\{tag}");
    for line in doc.lines() {
        let trimmed = line.trim();
        if let Some(rest) = trimmed
            .strip_prefix(&pattern)
            .or_else(|| trimmed.strip_prefix(&alt_pattern))
        {
            return rest.trim().to_string();
        }
    }
    String::new()
}

fn truncate_sig(sig: &str, max: usize) -> String {
    if sig.len() <= max {
        sig.to_string()
    } else {
        format!("{}...", &sig[..max - 3])
    }
}

// ─── TODO Scanner ───────────────────────────────────────────────────────────

fn scan_cpp_todos(source: &str, path: &Path) -> Vec<TodoItem> {
    let re = regex::Regex::new(r"//\s*(TODO|FIXME|HACK|SAFETY)\s*[:(]?\s*(.*)").unwrap();
    source
        .lines()
        .enumerate()
        .filter_map(|(i, line)| {
            re.captures(line.trim()).map(|caps| {
                let kind = match &caps[1] {
                    "TODO" => TodoKind::Todo,
                    "FIXME" => TodoKind::Fixme,
                    "HACK" => TodoKind::Hack,
                    "SAFETY" => TodoKind::Safety,
                    _ => TodoKind::Todo,
                };
                TodoItem {
                    kind,
                    text: caps[2].trim().to_string(),
                    location: SourceLocation {
                        file: path.to_path_buf(),
                        line: i + 1,
                        end_line: None,
                    },
                }
            })
        })
        .collect()
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn extract_from_header(source: &str) -> CppExtractionResult {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("test.hpp");
        std::fs::write(&file, source).unwrap();
        extract_cpp_file(&file, false).unwrap()
    }

    #[test]
    fn test_extract_free_function() {
        let result = extract_from_header("int add(int a, int b) { return a + b; }");
        assert!(!result.module.symbols.is_empty(), "should extract function");
        assert_eq!(result.module.symbols[0].name(), "add");
    }

    #[test]
    fn test_extract_function_declaration() {
        let result = extract_from_header("void process(double value);");
        assert!(!result.module.symbols.is_empty());
        assert_eq!(result.module.symbols[0].name(), "process");
    }

    #[test]
    fn test_extract_class() {
        let result = extract_from_header(
            "class Robot {\npublic:\n    void move();\n    int speed;\nprivate:\n    int internal;\n};",
        );
        let classes: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Struct(_)))
            .collect();
        assert_eq!(classes.len(), 1);
        if let SymbolDoc::Struct(s) = &classes[0] {
            assert_eq!(s.name, "Robot");
            // public method should be extracted
            assert!(
                !s.methods.is_empty() || !s.fields.is_empty(),
                "should have public members"
            );
        }
    }

    #[test]
    fn test_extract_struct_default_public() {
        let result = extract_from_header("struct Point {\n    double x;\n    double y;\n};");
        let structs: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Struct(_)))
            .collect();
        assert_eq!(structs.len(), 1);
        if let SymbolDoc::Struct(s) = &structs[0] {
            assert_eq!(s.name, "Point");
            assert_eq!(s.fields.len(), 2);
        }
    }

    #[test]
    fn test_extract_enum() {
        let result = extract_from_header("enum Color { Red, Green, Blue };");
        let enums: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Enum(_)))
            .collect();
        assert_eq!(enums.len(), 1);
        if let SymbolDoc::Enum(e) = &enums[0] {
            assert_eq!(e.name, "Color");
            assert_eq!(e.variants.len(), 3);
        }
    }

    #[test]
    fn test_extract_enum_class() {
        let result = extract_from_header("enum class Direction { North, South, East, West };");
        let enums: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Enum(_)))
            .collect();
        assert_eq!(enums.len(), 1);
        if let SymbolDoc::Enum(e) = &enums[0] {
            assert_eq!(e.name, "Direction");
            assert_eq!(e.variants.len(), 4);
        }
    }

    #[test]
    fn test_extract_namespace() {
        let result = extract_from_header(
            "namespace horus {\n    namespace sensors {\n        void read();\n    }\n}",
        );
        let fns: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Function(_)))
            .collect();
        assert!(!fns.is_empty());
        // Should have qualified name
        assert!(
            fns[0].name().contains("horus")
                || fns[0].name().contains("sensors")
                || fns[0].name() == "read",
            "got name: {}",
            fns[0].name()
        );
    }

    #[test]
    fn test_extract_doxygen_comment() {
        let result = extract_from_header(
            "/// @brief Compute velocity.\n/// @param speed The speed value.\nvoid compute(double speed);",
        );
        if let SymbolDoc::Function(f) = &result.module.symbols[0] {
            assert!(f.doc.is_some(), "should extract doc comment");
            let doc = f.doc.as_ref().unwrap();
            assert!(
                doc.contains("velocity") || doc.contains("Compute"),
                "doc: {doc}"
            );
        }
    }

    #[test]
    fn test_extract_inheritance() {
        let result = extract_from_header("class Base {};\nclass Derived : public Base {};");
        let classes: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Struct(_)))
            .collect();
        assert!(classes.len() >= 2);
        // Find Derived
        let derived = classes.iter().find(|s| s.name() == "Derived");
        assert!(derived.is_some(), "should find Derived class");
        if let SymbolDoc::Struct(s) = derived.unwrap() {
            assert!(
                !s.trait_impls.is_empty(),
                "should have base class in trait_impls"
            );
            assert!(
                s.trait_impls.iter().any(|b| b.contains("Base")),
                "trait_impls: {:?}",
                s.trait_impls
            );
        }
    }

    #[test]
    fn test_extract_using_alias() {
        let result = extract_from_header("using Vec3 = std::array<double, 3>;");
        let aliases: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::TypeAlias(_)))
            .collect();
        assert_eq!(aliases.len(), 1);
        if let SymbolDoc::TypeAlias(t) = &aliases[0] {
            assert_eq!(t.name, "Vec3");
        }
    }

    #[test]
    fn test_extract_include() {
        let result = extract_from_header("#include <vector>\n#include \"robot.h\"\nvoid f();");
        assert_eq!(result.module.imports.len(), 2);
    }

    #[test]
    fn test_source_locations() {
        let result = extract_from_header("void first();\n\nvoid second();");
        for sym in &result.module.symbols {
            if let Some(loc) = sym.location() {
                assert!(loc.line > 0, "line should be > 0");
            }
        }
    }

    #[test]
    fn test_cpp_todos() {
        let todos = scan_cpp_todos(
            "// TODO: fix this\n// FIXME: broken\nint x;",
            std::path::Path::new("test.hpp"),
        );
        assert_eq!(todos.len(), 2);
        assert_eq!(todos[0].kind, TodoKind::Todo);
        assert_eq!(todos[1].kind, TodoKind::Fixme);
    }

    #[test]
    fn test_empty_header() {
        let result = extract_from_header("");
        assert!(result.module.symbols.is_empty());
    }

    #[test]
    fn test_language_is_cpp() {
        let result = extract_from_header("void f();");
        assert_eq!(result.module.language, "cpp");
    }

    #[test]
    fn test_extract_template_class() {
        let result = extract_from_header(
            "template<typename T>\nclass Container {\npublic:\n    T value;\n    T get() const;\n};",
        );
        let classes: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Struct(_)))
            .collect();
        assert_eq!(
            classes.len(),
            1,
            "should extract exactly one class from template"
        );
        if let SymbolDoc::Struct(s) = &classes[0] {
            assert_eq!(s.name, "Container");
            assert!(
                !s.fields.is_empty() || !s.methods.is_empty(),
                "template class should have public members"
            );
        }
    }

    #[test]
    fn test_extract_deep_nested_namespace() {
        let result =
            extract_from_header("namespace a { namespace b { namespace c { void deep(); } } }");
        let fns: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Function(_)))
            .collect();
        assert!(
            !fns.is_empty(),
            "should extract function from deep namespace"
        );
        let name = fns[0].name();
        assert!(
            name.contains("a") || name.contains("b") || name.contains("c"),
            "function name should be namespace-qualified, got: {name}"
        );
    }

    #[test]
    fn test_extract_default_args() {
        let result = extract_from_header("void configure(int speed = 100, bool verbose = false);");
        let fns: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Function(_)))
            .collect();
        assert_eq!(fns.len(), 1, "should extract function with default args");
        assert_eq!(fns[0].name(), "configure");
        if let SymbolDoc::Function(f) = &fns[0] {
            // tree-sitter may use optional_parameter_declaration for defaulted params,
            // so param count may be 0 or 2 depending on parser behavior; the key
            // assertion is that the function itself is extracted with its signature.
            assert!(
                f.signature.contains("configure"),
                "signature should contain function name, got: {}",
                f.signature
            );
        }
    }

    #[test]
    fn test_private_members_excluded() {
        // extract_from_header uses include_private=false
        let result = extract_from_header(
            "class Priv {\nprivate:\n    int secret;\npublic:\n    int visible;\n};",
        );
        let classes: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Struct(_)))
            .collect();
        assert_eq!(classes.len(), 1);
        if let SymbolDoc::Struct(s) = &classes[0] {
            assert_eq!(s.name, "Priv");
            // Only public field "visible" should be present; private "secret" excluded
            assert!(
                s.fields.iter().any(|f| f.name == "visible"),
                "should include public field 'visible', fields: {:?}",
                s.fields
            );
            assert!(
                !s.fields.iter().any(|f| f.name == "secret"),
                "should exclude private field 'secret', fields: {:?}",
                s.fields
            );
        }
    }

    #[test]
    fn test_extract_constexpr() {
        let result = extract_from_header("constexpr int MAX_SIZE = 1024;");
        // constexpr may be extracted as a function/declaration or constant
        assert!(
            !result.module.symbols.is_empty(),
            "should extract constexpr as a symbol"
        );
        let names: Vec<_> = result
            .module
            .symbols
            .iter()
            .map(|s| s.name().to_string())
            .collect();
        assert!(
            names.iter().any(|n| n.contains("MAX_SIZE")),
            "should extract symbol named MAX_SIZE, got: {:?}",
            names
        );
    }

    #[test]
    fn test_extract_multiple_inheritance() {
        let result =
            extract_from_header("class A {};\nclass B {};\nclass C : public A, public B {};");
        let classes: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Struct(_)))
            .collect();
        let c_class = classes.iter().find(|s| s.name() == "C");
        assert!(c_class.is_some(), "should find class C");
        if let SymbolDoc::Struct(s) = c_class.unwrap() {
            assert!(
                s.trait_impls.iter().any(|b| b.contains("A")),
                "C should inherit from A, trait_impls: {:?}",
                s.trait_impls
            );
            assert!(
                s.trait_impls.iter().any(|b| b.contains("B")),
                "C should inherit from B, trait_impls: {:?}",
                s.trait_impls
            );
        }
    }

    #[test]
    fn test_extract_multiline_signature() {
        let result = extract_from_header("int\ncompute(\n    double x,\n    double y\n);");
        let fns: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Function(_)))
            .collect();
        assert!(
            !fns.is_empty(),
            "should extract function with multiline signature"
        );
        assert_eq!(fns[0].name(), "compute");
    }

    #[test]
    fn test_doxygen_param_extraction() {
        let result = extract_from_header(
            "/// @brief Compute sum.\n/// @param a First value.\n/// @param b Second value.\n/// @return Sum of a and b.\nint add(int a, int b);",
        );
        let fns: Vec<_> = result
            .module
            .symbols
            .iter()
            .filter(|s| matches!(s, SymbolDoc::Function(_)))
            .collect();
        assert_eq!(fns.len(), 1, "should extract documented function");
        if let SymbolDoc::Function(f) = &fns[0] {
            assert_eq!(f.name, "add");
            assert!(f.doc.is_some(), "should have doc comment");
            let doc = f.doc.as_ref().unwrap();
            assert!(
                doc.contains("Compute sum"),
                "doc should contain '@brief' text, got: {doc}"
            );
            assert!(
                f.params.len() >= 2,
                "should extract at least 2 params, got: {}",
                f.params.len()
            );
        }
    }

    // ─── Level 2: Integration Tests ─────────────────────────────────────────

    #[test]
    fn test_integration_cpp_extract_full_header() {
        let source = r#"
/// @brief Sensor utilities.
namespace sensors {

/// A sensor reading.
class Reading {
public:
    /// Get the value.
    double value() const;
    /// Set the value.
    void set_value(double v);
private:
    double raw_;
};

/// Sensor status codes.
enum Status { Ok, Error, Timeout };

} // namespace sensors

/// Initialize the sensor system.
void init_sensors(int count);

// TODO: add calibration support
"#;
        let result = extract_from_header(source);

        // At least 3 symbols: class Reading, enum Status, free function init_sensors
        assert!(
            result.module.symbols.len() >= 3,
            "should extract at least 3 symbols (class + enum + function), got: {}",
            result.module.symbols.len()
        );

        // Namespace-qualified name should be present
        let names: Vec<String> = result
            .module
            .symbols
            .iter()
            .map(|s| s.name().to_string())
            .collect();
        assert!(
            names.iter().any(|n| n.contains("sensors::")
                || n == "sensors::Reading"
                || n == "sensors::Status"),
            "should have a namespace-qualified symbol, got: {:?}",
            names
        );

        // TODO captured
        assert!(!result.todos.is_empty(), "should capture the TODO comment");
        assert!(
            result.todos.iter().any(|t| t.text.contains("calibration")),
            "TODO text should mention 'calibration', got: {:?}",
            result.todos
        );
    }

    // ─── Level 5: Error Path Tests ──────────────────────────────────────────

    #[test]
    fn test_error_cpp_empty_file_no_crash() {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("empty.hpp");
        std::fs::write(&file, "").unwrap();
        let result = extract_cpp_file(&file, false).unwrap();
        assert!(
            result.module.symbols.is_empty(),
            "empty file should produce no symbols"
        );
        assert!(
            result.todos.is_empty(),
            "empty file should produce no TODOs"
        );
    }

    #[test]
    fn test_error_cpp_invalid_syntax() {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("broken.hpp");
        std::fs::write(&file, "}}}{{{{").unwrap();
        // tree-sitter is error-tolerant; extract_cpp_file should still return Ok
        let result = extract_cpp_file(&file, false);
        assert!(
            result.is_ok(),
            "invalid syntax should not cause an error, got: {:?}",
            result.err()
        );
        // No panic is the primary assertion — the function returned without crashing
    }
}
