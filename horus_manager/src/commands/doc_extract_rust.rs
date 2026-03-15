//! Rust source file extractor using `syn`.
//!
//! Parses `.rs` files and extracts all public symbols into `ModuleDoc`.
//! Handles functions, structs, enums, traits, type aliases, constants,
//! impl blocks, doc comments, `#[deprecated]`, and source locations.

use super::doc_extract::*;
use anyhow::{Context, Result};
use std::path::Path;
use syn::{self, Attribute, Fields, GenericParam, Generics, Item, Meta, ReturnType, Type};

/// Result of extracting a single Rust file.
pub struct RustExtractionResult {
    pub module: ModuleDoc,
    pub entry_points: Vec<EntryPoint>,
    pub topics: Vec<TopicInfo>,
    pub todos: Vec<TodoItem>,
}

/// Extract documentation from a Rust source file.
pub fn extract_rust_file(path: &Path, include_private: bool) -> Result<RustExtractionResult> {
    let source = std::fs::read_to_string(path)
        .with_context(|| format!("Failed to read {}", path.display()))?;
    let file = syn::parse_file(&source)
        .with_context(|| format!("Failed to parse {}", path.display()))?;

    let module_doc = extract_module_doc(&file.attrs);
    let mut symbols = Vec::new();
    let mut imports = Vec::new();
    let mut entry_points = Vec::new();

    // Collect impl blocks for later association
    let mut impl_records: Vec<ImplRecord> = Vec::new();

    for item in &file.items {
        match item {
            Item::Fn(f) if include_private || is_public(&f.vis) => {
                symbols.push(SymbolDoc::Function(extract_fn_item(f, path)));
            }
            Item::Struct(s) if include_private || is_public(&s.vis) => {
                symbols.push(SymbolDoc::Struct(extract_struct_item(s, path)));
            }
            Item::Enum(e) if include_private || is_public(&e.vis) => {
                symbols.push(SymbolDoc::Enum(extract_enum_item(e, path)));
            }
            Item::Trait(t) if include_private || is_public(&t.vis) => {
                symbols.push(SymbolDoc::Trait(extract_trait_item(t, path)));
            }
            Item::Type(t) if include_private || is_public(&t.vis) => {
                symbols.push(SymbolDoc::TypeAlias(extract_type_alias(t, path)));
            }
            Item::Const(c) if include_private || is_public(&c.vis) => {
                symbols.push(SymbolDoc::Constant(extract_const_item(c, path)));
            }
            Item::Static(s) if include_private || is_public(&s.vis) => {
                symbols.push(SymbolDoc::Constant(extract_static_item(s, path)));
            }
            Item::Impl(i) => {
                let type_name = type_name_from_path(&i.self_ty);
                let trait_name = i.trait_.as_ref().map(|(_, p, _)| path_to_string(p));
                let methods = extract_impl_methods(i, path, include_private);

                // Detect impl Node for X → entry point
                if let Some(ref tn) = trait_name {
                    if tn == "Node" || tn.ends_with("::Node") {
                        entry_points.push(EntryPoint {
                            kind: EntryPointKind::HorusNode,
                            name: type_name.clone(),
                            file: path.to_path_buf(),
                            details: None, // Topic scanning done in macro task
                        });
                    }
                }

                impl_records.push(ImplRecord {
                    type_name,
                    trait_name,
                    methods,
                });
            }
            Item::Use(u) if include_private || is_public(&u.vis) => {
                imports.push(use_tree_to_string(&u.tree));
            }
            Item::Macro(m) => {
                if let Some(msgs) = extract_message_macro(m, path) {
                    symbols.extend(msgs.into_iter().map(SymbolDoc::HorusMessage));
                } else if let Some(svc) = extract_service_macro(m, path) {
                    symbols.push(SymbolDoc::HorusService(svc));
                } else if let Some(act) = extract_action_macro(m, path) {
                    symbols.push(SymbolDoc::HorusAction(act));
                }
            }
            _ => {}
        }
    }

    // Associate impl methods with their types
    associate_impl_methods(&mut symbols, &impl_records);

    // Scan for TODOs
    let todos = scan_todos(&source, path);

    Ok(RustExtractionResult {
        module: ModuleDoc {
            path: path.to_path_buf(),
            language: "rust".to_string(),
            module_doc,
            imports,
            symbols,
        },
        entry_points,
        topics: vec![],
        todos,
    })
}

// ─── Impl Record ────────────────────────────────────────────────────────────

struct ImplRecord {
    type_name: String,
    trait_name: Option<String>,
    methods: Vec<FunctionDoc>,
}

// ─── Extraction Helpers ─────────────────────────────────────────────────────

fn extract_fn_item(f: &syn::ItemFn, path: &Path) -> FunctionDoc {
    let sig = &f.sig;
    FunctionDoc {
        name: sig.ident.to_string(),
        visibility: vis_to_enum(&f.vis),
        location: span_to_location(path, &f.sig.ident),
        signature: signature_to_string(&f.vis, sig),
        doc: extract_doc_comment(&f.attrs),
        deprecated: extract_deprecated(&f.attrs),
        params: extract_params(sig),
        returns: extract_return_type(&sig.output),
        is_async: sig.asyncness.is_some(),
        generic_params: extract_generics(&sig.generics),
        examples: extract_examples_from_doc(&f.attrs),
    }
}

fn extract_struct_item(s: &syn::ItemStruct, path: &Path) -> StructDoc {
    let fields = match &s.fields {
        Fields::Named(named) => named
            .named
            .iter()
            .filter(|f| {
                matches!(&f.vis, syn::Visibility::Public(_) | syn::Visibility::Inherited)
            })
            .map(|f| FieldDoc {
                name: f
                    .ident
                    .as_ref()
                    .map(|i| i.to_string())
                    .unwrap_or_default(),
                type_str: Some(type_to_string(&f.ty)),
                doc: extract_doc_comment(&f.attrs),
            })
            .collect(),
        Fields::Unnamed(unnamed) => unnamed
            .unnamed
            .iter()
            .enumerate()
            .map(|(i, f)| FieldDoc {
                name: format!("{i}"),
                type_str: Some(type_to_string(&f.ty)),
                doc: None,
            })
            .collect(),
        Fields::Unit => vec![],
    };

    let derives = extract_derives(&s.attrs);

    StructDoc {
        name: s.ident.to_string(),
        visibility: vis_to_enum(&s.vis),
        location: span_to_location(path, &s.ident),
        doc: extract_doc_comment(&s.attrs),
        deprecated: extract_deprecated(&s.attrs),
        generic_params: extract_generics(&s.generics),
        fields,
        methods: vec![], // filled by associate_impl_methods
        trait_impls: vec![], // filled by associate_impl_methods
        derives,
        examples: extract_examples_from_doc(&s.attrs),
    }
}

fn extract_enum_item(e: &syn::ItemEnum, path: &Path) -> EnumDoc {
    let variants = e
        .variants
        .iter()
        .map(|v| {
            let fields = match &v.fields {
                Fields::Named(named) => named
                    .named
                    .iter()
                    .map(|f| FieldDoc {
                        name: f.ident.as_ref().map(|i| i.to_string()).unwrap_or_default(),
                        type_str: Some(type_to_string(&f.ty)),
                        doc: extract_doc_comment(&f.attrs),
                    })
                    .collect(),
                Fields::Unnamed(unnamed) => unnamed
                    .unnamed
                    .iter()
                    .enumerate()
                    .map(|(i, f)| FieldDoc {
                        name: format!("{i}"),
                        type_str: Some(type_to_string(&f.ty)),
                        doc: None,
                    })
                    .collect(),
                Fields::Unit => vec![],
            };
            VariantDoc {
                name: v.ident.to_string(),
                doc: extract_doc_comment(&v.attrs),
                fields,
            }
        })
        .collect();

    EnumDoc {
        name: e.ident.to_string(),
        visibility: vis_to_enum(&e.vis),
        location: span_to_location(path, &e.ident),
        doc: extract_doc_comment(&e.attrs),
        deprecated: extract_deprecated(&e.attrs),
        variants,
        methods: vec![], // filled by associate_impl_methods
    }
}

fn extract_trait_item(t: &syn::ItemTrait, path: &Path) -> TraitDoc {
    let mut required = Vec::new();
    let mut provided = Vec::new();

    for item in &t.items {
        if let syn::TraitItem::Fn(m) = item {
            let func = FunctionDoc {
                name: m.sig.ident.to_string(),
                visibility: Visibility::Public,
                location: span_to_location(path, &m.sig.ident),
                signature: format!("fn {}", sig_to_string_no_vis(&m.sig)),
                doc: extract_doc_comment(&m.attrs),
                deprecated: extract_deprecated(&m.attrs),
                params: extract_params(&m.sig),
                returns: extract_return_type(&m.sig.output),
                is_async: m.sig.asyncness.is_some(),
                generic_params: extract_generics(&m.sig.generics),
                examples: vec![],
            };
            if m.default.is_some() {
                provided.push(func);
            } else {
                required.push(func);
            }
        }
    }

    TraitDoc {
        name: t.ident.to_string(),
        visibility: vis_to_enum(&t.vis),
        location: span_to_location(path, &t.ident),
        doc: extract_doc_comment(&t.attrs),
        deprecated: extract_deprecated(&t.attrs),
        generic_params: extract_generics(&t.generics),
        required_methods: required,
        provided_methods: provided,
        implementors: vec![], // filled by cross-reference pass
    }
}

fn extract_type_alias(t: &syn::ItemType, path: &Path) -> TypeAliasDoc {
    TypeAliasDoc {
        name: t.ident.to_string(),
        visibility: vis_to_enum(&t.vis),
        location: span_to_location(path, &t.ident),
        doc: extract_doc_comment(&t.attrs),
        deprecated: extract_deprecated(&t.attrs),
        target_type: type_to_string(&t.ty),
    }
}

fn extract_const_item(c: &syn::ItemConst, path: &Path) -> ConstantDoc {
    ConstantDoc {
        name: c.ident.to_string(),
        visibility: vis_to_enum(&c.vis),
        location: span_to_location(path, &c.ident),
        doc: extract_doc_comment(&c.attrs),
        deprecated: extract_deprecated(&c.attrs),
        type_str: type_to_string(&c.ty),
        value: Some(expr_to_string(&c.expr)),
    }
}

fn extract_static_item(s: &syn::ItemStatic, path: &Path) -> ConstantDoc {
    ConstantDoc {
        name: s.ident.to_string(),
        visibility: vis_to_enum(&s.vis),
        location: span_to_location(path, &s.ident),
        doc: extract_doc_comment(&s.attrs),
        deprecated: extract_deprecated(&s.attrs),
        type_str: type_to_string(&s.ty),
        value: Some(expr_to_string(&s.expr)),
    }
}

fn extract_impl_methods(
    i: &syn::ItemImpl,
    path: &Path,
    include_private: bool,
) -> Vec<FunctionDoc> {
    i.items
        .iter()
        .filter_map(|item| {
            if let syn::ImplItem::Fn(m) = item {
                if include_private || is_impl_method_public(&m.vis) {
                    Some(FunctionDoc {
                        name: m.sig.ident.to_string(),
                        visibility: vis_to_enum(&m.vis),
                        location: span_to_location(path, &m.sig.ident),
                        signature: signature_to_string(&m.vis, &m.sig),
                        doc: extract_doc_comment(&m.attrs),
                        deprecated: extract_deprecated(&m.attrs),
                        params: extract_params(&m.sig),
                        returns: extract_return_type(&m.sig.output),
                        is_async: m.sig.asyncness.is_some(),
                        generic_params: extract_generics(&m.sig.generics),
                        examples: extract_examples_from_doc(&m.attrs),
                    })
                } else {
                    None
                }
            } else {
                None
            }
        })
        .collect()
}

// ─── Horus Macro Extractors ─────────────────────────────────────────────────

/// Extract `message! { Name { field: Type, ... } ... }` invocations.
/// Returns multiple messages (one block can define several).
fn extract_message_macro(m: &syn::ItemMacro, path: &Path) -> Option<Vec<HorusMessageDoc>> {
    let mac_path = &m.mac.path;
    if !mac_path.is_ident("message") {
        return None;
    }

    let tokens: Vec<proc_macro2::TokenTree> = m.mac.tokens.clone().into_iter().collect();
    let location = span_to_location(path, &mac_path.segments.first()?.ident);
    let doc = extract_doc_comment(&m.attrs);
    let mut messages = Vec::new();
    let mut i = 0;

    while i < tokens.len() {
        // Look for: Ident { field: Type, ... }
        if let proc_macro2::TokenTree::Ident(name) = &tokens[i] {
            let msg_name = name.to_string();
            // Check for optional doc comments preceding the name (as #[...] attrs in tokens)
            if i + 1 < tokens.len() {
                if let proc_macro2::TokenTree::Group(group) = &tokens[i + 1] {
                    if group.delimiter() == proc_macro2::Delimiter::Brace {
                        let fields = parse_struct_fields_from_tokens(group.stream());
                        messages.push(HorusMessageDoc {
                            name: msg_name,
                            location: location.clone(),
                            doc: doc.clone(),
                            deprecated: None,
                            fields,
                        });
                        i += 2;
                        continue;
                    }
                }
            }
        }
        i += 1;
    }

    if messages.is_empty() {
        None
    } else {
        Some(messages)
    }
}

/// Extract `service! { Name { request { ... } response { ... } } }` invocations.
fn extract_service_macro(m: &syn::ItemMacro, path: &Path) -> Option<HorusServiceDoc> {
    let mac_path = &m.mac.path;
    if !mac_path.is_ident("service") {
        return None;
    }

    let tokens: Vec<proc_macro2::TokenTree> = m.mac.tokens.clone().into_iter().collect();
    let location = span_to_location(path, &mac_path.segments.first()?.ident);
    let doc = extract_doc_comment(&m.attrs);

    // Pattern: Name { request { fields } response { fields } }
    let mut i = 0;
    while i < tokens.len() {
        if let proc_macro2::TokenTree::Ident(name) = &tokens[i] {
            let svc_name = name.to_string();
            if svc_name == "request" || svc_name == "response" {
                i += 1;
                continue;
            }
            if i + 1 < tokens.len() {
                if let proc_macro2::TokenTree::Group(outer) = &tokens[i + 1] {
                    if outer.delimiter() == proc_macro2::Delimiter::Brace {
                        let (req, resp) = parse_service_blocks(outer.stream());
                        return Some(HorusServiceDoc {
                            name: svc_name,
                            location,
                            doc,
                            request_fields: req,
                            response_fields: resp,
                        });
                    }
                }
            }
        }
        i += 1;
    }
    None
}

/// Extract `action! { Name { goal { ... } feedback { ... } result { ... } } }` invocations.
fn extract_action_macro(m: &syn::ItemMacro, path: &Path) -> Option<HorusActionDoc> {
    let mac_path = &m.mac.path;
    if !mac_path.is_ident("action") {
        return None;
    }

    let tokens: Vec<proc_macro2::TokenTree> = m.mac.tokens.clone().into_iter().collect();
    let location = span_to_location(path, &mac_path.segments.first()?.ident);
    let doc = extract_doc_comment(&m.attrs);

    let mut i = 0;
    while i < tokens.len() {
        if let proc_macro2::TokenTree::Ident(name) = &tokens[i] {
            let act_name = name.to_string();
            if act_name == "goal" || act_name == "feedback" || act_name == "result" {
                i += 1;
                continue;
            }
            if i + 1 < tokens.len() {
                if let proc_macro2::TokenTree::Group(outer) = &tokens[i + 1] {
                    if outer.delimiter() == proc_macro2::Delimiter::Brace {
                        let (goal, feedback, result) = parse_action_blocks(outer.stream());
                        return Some(HorusActionDoc {
                            name: act_name,
                            location,
                            doc,
                            goal_fields: goal,
                            feedback_fields: feedback,
                            result_fields: result,
                        });
                    }
                }
            }
        }
        i += 1;
    }
    None
}

/// Parse `field: Type, field: Type` from a brace-delimited token group.
fn parse_struct_fields_from_tokens(tokens: proc_macro2::TokenStream) -> Vec<FieldDoc> {
    let toks: Vec<proc_macro2::TokenTree> = tokens.into_iter().collect();
    let mut fields = Vec::new();
    let mut i = 0;

    while i < toks.len() {
        // Skip attributes (#[...])
        if let proc_macro2::TokenTree::Punct(p) = &toks[i] {
            if p.as_char() == '#' {
                i += 1;
                if i < toks.len() {
                    if let proc_macro2::TokenTree::Group(_) = &toks[i] {
                        i += 1;
                    }
                }
                continue;
            }
        }

        // Look for: ident : type_tokens [, | end]
        if let proc_macro2::TokenTree::Ident(field_name) = &toks[i] {
            let name = field_name.to_string();
            // Skip if next is not ':'
            if i + 1 < toks.len() {
                if let proc_macro2::TokenTree::Punct(p) = &toks[i + 1] {
                    if p.as_char() == ':' {
                        // Collect type tokens until comma or end
                        let mut type_tokens = Vec::new();
                        let mut j = i + 2;
                        let mut angle_depth = 0;
                        while j < toks.len() {
                            if let proc_macro2::TokenTree::Punct(p) = &toks[j] {
                                if p.as_char() == '<' {
                                    angle_depth += 1;
                                } else if p.as_char() == '>' {
                                    angle_depth -= 1;
                                } else if p.as_char() == ',' && angle_depth == 0 {
                                    break;
                                } else if p.as_char() == '=' && angle_depth == 0 {
                                    // Default value — stop type, skip to comma
                                    break;
                                }
                            }
                            type_tokens.push(toks[j].clone());
                            j += 1;
                        }
                        let type_str = type_tokens
                            .iter()
                            .map(|t| t.to_string())
                            .collect::<Vec<_>>()
                            .join("")
                            .replace(" :: ", "::")
                            .replace("< ", "<")
                            .replace(" >", ">");

                        fields.push(FieldDoc {
                            name,
                            type_str: Some(type_str),
                            doc: None,
                        });

                        i = j + 1; // skip past comma
                        continue;
                    }
                }
            }
        }
        i += 1;
    }

    fields
}

/// Parse service blocks: `request { fields } response { fields }`.
fn parse_service_blocks(tokens: proc_macro2::TokenStream) -> (Vec<FieldDoc>, Vec<FieldDoc>) {
    let toks: Vec<proc_macro2::TokenTree> = tokens.into_iter().collect();
    let mut request_fields = Vec::new();
    let mut response_fields = Vec::new();
    let mut i = 0;

    while i < toks.len() {
        if let proc_macro2::TokenTree::Ident(ident) = &toks[i] {
            let name = ident.to_string();
            if (name == "request" || name == "response") && i + 1 < toks.len() {
                if let proc_macro2::TokenTree::Group(group) = &toks[i + 1] {
                    if group.delimiter() == proc_macro2::Delimiter::Brace {
                        let fields = parse_struct_fields_from_tokens(group.stream());
                        if name == "request" {
                            request_fields = fields;
                        } else {
                            response_fields = fields;
                        }
                        i += 2;
                        continue;
                    }
                }
            }
        }
        i += 1;
    }

    (request_fields, response_fields)
}

/// Parse action blocks: `goal { fields } feedback { fields } result { fields }`.
fn parse_action_blocks(
    tokens: proc_macro2::TokenStream,
) -> (Vec<FieldDoc>, Vec<FieldDoc>, Vec<FieldDoc>) {
    let toks: Vec<proc_macro2::TokenTree> = tokens.into_iter().collect();
    let mut goal = Vec::new();
    let mut feedback = Vec::new();
    let mut result = Vec::new();
    let mut i = 0;

    while i < toks.len() {
        if let proc_macro2::TokenTree::Ident(ident) = &toks[i] {
            let name = ident.to_string();
            if (name == "goal" || name == "feedback" || name == "result") && i + 1 < toks.len() {
                if let proc_macro2::TokenTree::Group(group) = &toks[i + 1] {
                    if group.delimiter() == proc_macro2::Delimiter::Brace {
                        let fields = parse_struct_fields_from_tokens(group.stream());
                        match name.as_str() {
                            "goal" => goal = fields,
                            "feedback" => feedback = fields,
                            "result" => result = fields,
                            _ => {}
                        }
                        i += 2;
                        continue;
                    }
                }
            }
        }
        i += 1;
    }

    (goal, feedback, result)
}

// ─── Association ────────────────────────────────────────────────────────────

fn associate_impl_methods(symbols: &mut Vec<SymbolDoc>, impls: &[ImplRecord]) {
    for imp in impls {
        // Find matching struct/enum and add methods + trait_impls
        for sym in symbols.iter_mut() {
            match sym {
                SymbolDoc::Struct(s) if s.name == imp.type_name => {
                    s.methods.extend(imp.methods.clone());
                    if let Some(ref tn) = imp.trait_name {
                        if !s.trait_impls.contains(tn) {
                            s.trait_impls.push(tn.clone());
                        }
                    }
                }
                SymbolDoc::Enum(e) if e.name == imp.type_name => {
                    e.methods.extend(imp.methods.clone());
                }
                _ => {}
            }
        }
    }
}

// ─── Attribute Extractors ───────────────────────────────────────────────────

fn extract_doc_comment(attrs: &[Attribute]) -> Option<String> {
    let docs: Vec<String> = attrs
        .iter()
        .filter_map(|attr| {
            if attr.path().is_ident("doc") {
                if let Meta::NameValue(nv) = &attr.meta {
                    if let syn::Expr::Lit(lit) = &nv.value {
                        if let syn::Lit::Str(s) = &lit.lit {
                            return Some(s.value());
                        }
                    }
                }
            }
            None
        })
        .collect();

    if docs.is_empty() {
        return None;
    }

    let combined = docs.join("\n");
    let trimmed = combined
        .lines()
        .map(|l| l.strip_prefix(' ').unwrap_or(l))
        .collect::<Vec<_>>()
        .join("\n")
        .trim()
        .to_string();

    if trimmed.is_empty() {
        None
    } else {
        Some(trimmed)
    }
}

fn extract_module_doc(attrs: &[Attribute]) -> Option<String> {
    // Module docs use the same #[doc = "..."] attributes
    extract_doc_comment(attrs)
}

fn extract_deprecated(attrs: &[Attribute]) -> Option<String> {
    for attr in attrs {
        if attr.path().is_ident("deprecated") {
            // #[deprecated] — no message
            if matches!(attr.meta, Meta::Path(_)) {
                return Some(String::new());
            }
            // #[deprecated = "message"] or #[deprecated(note = "message")]
            if let Meta::NameValue(nv) = &attr.meta {
                if let syn::Expr::Lit(lit) = &nv.value {
                    if let syn::Lit::Str(s) = &lit.lit {
                        return Some(s.value());
                    }
                }
            }
            // #[deprecated(since = "...", note = "...")]
            if let Meta::List(_) = &attr.meta {
                let text = attr
                    .meta
                    .require_list()
                    .ok()
                    .map(|list| list.tokens.to_string());
                if let Some(t) = text {
                    // Extract note = "..." from the token string
                    if let Some(note_start) = t.find("note") {
                        let rest = &t[note_start..];
                        if let Some(start) = rest.find('"') {
                            let after = &rest[start + 1..];
                            if let Some(end) = after.find('"') {
                                return Some(after[..end].to_string());
                            }
                        }
                    }
                    return Some(t);
                }
            }
            return Some(String::new());
        }
    }
    None
}

fn extract_derives(attrs: &[Attribute]) -> Vec<String> {
    let mut derives = Vec::new();
    for attr in attrs {
        if attr.path().is_ident("derive") {
            if let Ok(list) = attr.meta.require_list() {
                let tokens = list.tokens.to_string();
                for part in tokens.split(',') {
                    let trimmed = part.trim();
                    if !trimmed.is_empty() {
                        derives.push(trimmed.to_string());
                    }
                }
            }
        }
    }
    derives
}

fn extract_examples_from_doc(attrs: &[Attribute]) -> Vec<String> {
    let doc = match extract_doc_comment(attrs) {
        Some(d) => d,
        None => return vec![],
    };

    let mut examples = Vec::new();
    let mut in_example = false;
    let mut current = String::new();

    for line in doc.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with("```") && !in_example {
            in_example = true;
            current.clear();
        } else if trimmed.starts_with("```") && in_example {
            in_example = false;
            if !current.trim().is_empty() {
                examples.push(current.trim().to_string());
            }
            current.clear();
        } else if in_example {
            current.push_str(line);
            current.push('\n');
        }
    }

    examples
}

// ─── Signature Helpers ──────────────────────────────────────────────────────

fn signature_to_string(vis: &syn::Visibility, sig: &syn::Signature) -> String {
    let vis_str = match vis {
        syn::Visibility::Public(_) => "pub ",
        syn::Visibility::Restricted(_) => "pub(crate) ",
        syn::Visibility::Inherited => "",
    };
    let async_str = if sig.asyncness.is_some() {
        "async "
    } else {
        ""
    };
    let generics = if sig.generics.params.is_empty() {
        String::new()
    } else {
        let params: Vec<String> = sig.generics.params.iter().map(generic_to_string).collect();
        format!("<{}>", params.join(", "))
    };
    let params: Vec<String> = sig
        .inputs
        .iter()
        .map(|arg| match arg {
            syn::FnArg::Receiver(r) => {
                let mut s = String::new();
                if r.reference.is_some() {
                    s.push('&');
                    if r.mutability.is_some() {
                        s.push_str("mut ");
                    }
                }
                s.push_str("self");
                s
            }
            syn::FnArg::Typed(t) => {
                let name = pat_to_string(&t.pat);
                let ty = type_to_string(&t.ty);
                format!("{name}: {ty}")
            }
        })
        .collect();
    let ret = match &sig.output {
        ReturnType::Default => String::new(),
        ReturnType::Type(_, ty) => format!(" -> {}", type_to_string(ty)),
    };
    format!(
        "{vis_str}{async_str}fn {}{generics}({}){}",
        sig.ident,
        params.join(", "),
        ret
    )
}

fn sig_to_string_no_vis(sig: &syn::Signature) -> String {
    let async_str = if sig.asyncness.is_some() {
        "async "
    } else {
        ""
    };
    let params: Vec<String> = sig
        .inputs
        .iter()
        .map(|arg| match arg {
            syn::FnArg::Receiver(r) => {
                let mut s = String::new();
                if r.reference.is_some() {
                    s.push('&');
                    if r.mutability.is_some() {
                        s.push_str("mut ");
                    }
                }
                s.push_str("self");
                s
            }
            syn::FnArg::Typed(t) => {
                let name = pat_to_string(&t.pat);
                let ty = type_to_string(&t.ty);
                format!("{name}: {ty}")
            }
        })
        .collect();
    let ret = match &sig.output {
        ReturnType::Default => String::new(),
        ReturnType::Type(_, ty) => format!(" -> {}", type_to_string(ty)),
    };
    format!("{async_str}{}({}){}", sig.ident, params.join(", "), ret)
}

fn extract_params(sig: &syn::Signature) -> Vec<ParamDoc> {
    sig.inputs
        .iter()
        .filter_map(|arg| match arg {
            syn::FnArg::Receiver(_) => None,
            syn::FnArg::Typed(t) => Some(ParamDoc {
                name: pat_to_string(&t.pat),
                type_str: Some(type_to_string(&t.ty)),
                default_value: None,
                doc: None,
            }),
        })
        .collect()
}

fn extract_return_type(output: &ReturnType) -> Option<String> {
    match output {
        ReturnType::Default => None,
        ReturnType::Type(_, ty) => Some(type_to_string(ty)),
    }
}

fn extract_generics(generics: &Generics) -> Vec<String> {
    generics.params.iter().map(generic_to_string).collect()
}

fn generic_to_string(param: &GenericParam) -> String {
    match param {
        GenericParam::Type(t) => {
            let mut s = t.ident.to_string();
            if !t.bounds.is_empty() {
                let bounds: Vec<String> = t.bounds.iter().map(type_param_bound_to_string).collect();
                s.push_str(": ");
                s.push_str(&bounds.join(" + "));
            }
            s
        }
        GenericParam::Lifetime(l) => format!("'{}", l.lifetime.ident),
        GenericParam::Const(c) => format!("const {}: {}", c.ident, type_to_string(&c.ty)),
    }
}

// ─── Type/Path Helpers ──────────────────────────────────────────────────────

fn type_to_string(ty: &Type) -> String {
    match ty {
        Type::Path(tp) => {
            let mut s = String::new();
            if tp.qself.is_some() {
                s.push_str("<...>::");
            }
            s.push_str(&path_to_string_with_args(&tp.path));
            s
        }
        Type::Reference(r) => {
            let mut s = String::from("&");
            if let Some(lt) = &r.lifetime {
                s.push_str(&format!("'{} ", lt.ident));
            }
            if r.mutability.is_some() {
                s.push_str("mut ");
            }
            s.push_str(&type_to_string(&r.elem));
            s
        }
        Type::Tuple(t) => {
            let inner: Vec<String> = t.elems.iter().map(type_to_string).collect();
            format!("({})", inner.join(", "))
        }
        Type::Slice(s) => format!("[{}]", type_to_string(&s.elem)),
        Type::Array(a) => {
            format!("[{}; ...]", type_to_string(&a.elem))
        }
        Type::Ptr(p) => {
            let m = if p.mutability.is_some() { "mut" } else { "const" };
            format!("*{m} {}", type_to_string(&p.elem))
        }
        Type::ImplTrait(i) => {
            let bounds: Vec<String> = i.bounds.iter().map(type_param_bound_to_string).collect();
            format!("impl {}", bounds.join(" + "))
        }
        Type::Paren(p) => format!("({})", type_to_string(&p.elem)),
        Type::Never(_) => "!".to_string(),
        Type::Infer(_) => "_".to_string(),
        _ => "...".to_string(),
    }
}

fn type_name_from_path(ty: &Type) -> String {
    match ty {
        Type::Path(tp) => path_to_string(&tp.path),
        _ => type_to_string(ty),
    }
}

fn path_to_string(path: &syn::Path) -> String {
    path.segments
        .iter()
        .map(|s| s.ident.to_string())
        .collect::<Vec<_>>()
        .join("::")
}

fn path_to_string_with_args(path: &syn::Path) -> String {
    path.segments
        .iter()
        .map(|s| {
            let name = s.ident.to_string();
            match &s.arguments {
                syn::PathArguments::None => name,
                syn::PathArguments::AngleBracketed(ab) => {
                    let args: Vec<String> = ab.args.iter().map(|a| match a {
                        syn::GenericArgument::Type(t) => type_to_string(t),
                        syn::GenericArgument::Lifetime(l) => format!("'{}", l.ident),
                        _ => "...".to_string(),
                    }).collect();
                    format!("{}<{}>", name, args.join(", "))
                }
                syn::PathArguments::Parenthesized(p) => {
                    let inputs: Vec<String> = p.inputs.iter().map(type_to_string).collect();
                    let ret = match &p.output {
                        ReturnType::Default => String::new(),
                        ReturnType::Type(_, ty) => format!(" -> {}", type_to_string(ty)),
                    };
                    format!("{}({}){}", name, inputs.join(", "), ret)
                }
            }
        })
        .collect::<Vec<_>>()
        .join("::")
}

fn type_param_bound_to_string(bound: &syn::TypeParamBound) -> String {
    match bound {
        syn::TypeParamBound::Trait(t) => path_to_string_with_args(&t.path),
        syn::TypeParamBound::Lifetime(l) => format!("'{}", l.ident),
        _ => "...".to_string(),
    }
}

fn pat_to_string(pat: &syn::Pat) -> String {
    match pat {
        syn::Pat::Ident(pi) => pi.ident.to_string(),
        syn::Pat::Wild(_) => "_".to_string(),
        syn::Pat::Reference(r) => {
            let m = if r.mutability.is_some() { "mut " } else { "" };
            format!("&{m}{}", pat_to_string(&r.pat))
        }
        syn::Pat::Tuple(t) => {
            let inner: Vec<String> = t.elems.iter().map(pat_to_string).collect();
            format!("({})", inner.join(", "))
        }
        _ => "_".to_string(),
    }
}

fn expr_to_string(expr: &syn::Expr) -> String {
    match expr {
        syn::Expr::Lit(lit) => match &lit.lit {
            syn::Lit::Str(s) => format!("\"{}\"", s.value()),
            syn::Lit::Int(i) => i.base10_digits().to_string(),
            syn::Lit::Float(f) => f.base10_digits().to_string(),
            syn::Lit::Bool(b) => b.value.to_string(),
            syn::Lit::Char(c) => format!("'{}'", c.value()),
            _ => "...".to_string(),
        },
        syn::Expr::Path(p) => path_to_string(&p.path),
        syn::Expr::Unary(u) => {
            let op = match u.op {
                syn::UnOp::Neg(_) => "-",
                syn::UnOp::Not(_) => "!",
                syn::UnOp::Deref(_) => "*",
                _ => "?",
            };
            format!("{op}{}", expr_to_string(&u.expr))
        }
        syn::Expr::Reference(r) => {
            let m = if r.mutability.is_some() { "mut " } else { "" };
            format!("&{m}{}", expr_to_string(&r.expr))
        }
        syn::Expr::Call(c) => {
            let func = expr_to_string(&c.func);
            let args: Vec<String> = c.args.iter().map(expr_to_string).collect();
            format!("{func}({})", args.join(", "))
        }
        _ => "...".to_string(),
    }
}

fn use_tree_to_string(tree: &syn::UseTree) -> String {
    match tree {
        syn::UseTree::Path(p) => format!("{}::{}", p.ident, use_tree_to_string(&p.tree)),
        syn::UseTree::Name(n) => n.ident.to_string(),
        syn::UseTree::Rename(r) => format!("{} as {}", r.ident, r.rename),
        syn::UseTree::Glob(_) => "*".to_string(),
        syn::UseTree::Group(g) => {
            let items: Vec<String> = g.items.iter().map(use_tree_to_string).collect();
            format!("{{{}}}", items.join(", "))
        }
    }
}

// ─── Visibility Helpers ─────────────────────────────────────────────────────

fn is_public(vis: &syn::Visibility) -> bool {
    matches!(vis, syn::Visibility::Public(_) | syn::Visibility::Restricted(_))
}

fn is_impl_method_public(vis: &syn::Visibility) -> bool {
    matches!(vis, syn::Visibility::Public(_))
}

fn vis_to_enum(vis: &syn::Visibility) -> Visibility {
    match vis {
        syn::Visibility::Public(_) => Visibility::Public,
        syn::Visibility::Restricted(_) => Visibility::PublicCrate,
        syn::Visibility::Inherited => Visibility::Private,
    }
}

// ─── Source Location ────────────────────────────────────────────────────────

fn span_to_location(path: &Path, _ident: &syn::Ident) -> SourceLocation {
    // proc_macro2 spans require `span-locations` feature for line info.
    // Without it, we record line 0. Source locations will be populated
    // accurately when span-locations is enabled or via line-counting fallback.
    // For now, line numbers are best-effort.
    SourceLocation {
        file: path.to_path_buf(),
        line: 0,
        end_line: None,
    }
}

// ─── TODO Scanner ───────────────────────────────────────────────────────────

/// Scan source text for TODO/FIXME/HACK/SAFETY comments.
pub fn scan_todos(source: &str, path: &Path) -> Vec<TodoItem> {
    let re = regex::Regex::new(r"//\s*(TODO|FIXME|HACK|SAFETY)\s*[:(]?\s*(.*)").unwrap();
    source
        .lines()
        .enumerate()
        .filter_map(|(i, line)| {
            let trimmed = line.trim();
            re.captures(trimmed).map(|caps| {
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
    fn extract_from_source(source: &str) -> RustExtractionResult {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("test.rs");
        std::fs::write(&file, source).unwrap();
        extract_rust_file(&file, false).unwrap()
    }

    fn extract_from_source_all(source: &str) -> RustExtractionResult {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("test.rs");
        std::fs::write(&file, source).unwrap();
        extract_rust_file(&file, true).unwrap()
    }

    #[test]
    fn test_extract_pub_fn() {
        let result = extract_from_source(
            "pub fn hello(name: &str) -> String { name.to_string() }",
        );
        assert_eq!(result.module.symbols.len(), 1);
        let sym = &result.module.symbols[0];
        assert_eq!(sym.name(), "hello");
        if let SymbolDoc::Function(f) = sym {
            assert!(f.signature.contains("pub fn hello"));
            assert!(f.signature.contains("-> String"));
            assert_eq!(f.params.len(), 1);
            assert_eq!(f.params[0].name, "name");
            assert_eq!(f.returns.as_deref(), Some("String"));
            assert!(!f.is_async);
        } else {
            panic!("expected Function");
        }
    }

    #[test]
    fn test_extract_pub_struct() {
        let result = extract_from_source(
            "#[derive(Debug, Clone)]\npub struct Config {\n    pub verbose: bool,\n    pub name: String,\n}",
        );
        assert_eq!(result.module.symbols.len(), 1);
        if let SymbolDoc::Struct(s) = &result.module.symbols[0] {
            assert_eq!(s.name, "Config");
            assert_eq!(s.fields.len(), 2);
            assert_eq!(s.fields[0].name, "verbose");
            assert!(s.derives.contains(&"Debug".to_string()));
            assert!(s.derives.contains(&"Clone".to_string()));
        } else {
            panic!("expected Struct");
        }
    }

    #[test]
    fn test_extract_pub_enum() {
        let result = extract_from_source(
            "pub enum Color { Red, Green, Blue }",
        );
        assert_eq!(result.module.symbols.len(), 1);
        if let SymbolDoc::Enum(e) = &result.module.symbols[0] {
            assert_eq!(e.name, "Color");
            assert_eq!(e.variants.len(), 3);
            assert_eq!(e.variants[0].name, "Red");
        } else {
            panic!("expected Enum");
        }
    }

    #[test]
    fn test_extract_pub_trait() {
        let result = extract_from_source(
            "pub trait Greet {\n    fn hello(&self) -> String;\n    fn goodbye(&self) -> String { String::new() }\n}",
        );
        assert_eq!(result.module.symbols.len(), 1);
        if let SymbolDoc::Trait(t) = &result.module.symbols[0] {
            assert_eq!(t.name, "Greet");
            assert_eq!(t.required_methods.len(), 1);
            assert_eq!(t.required_methods[0].name, "hello");
            assert_eq!(t.provided_methods.len(), 1);
            assert_eq!(t.provided_methods[0].name, "goodbye");
        } else {
            panic!("expected Trait");
        }
    }

    #[test]
    fn test_extract_pub_type_alias() {
        let result = extract_from_source("pub type Result<T> = std::result::Result<T, Error>;");
        assert_eq!(result.module.symbols.len(), 1);
        if let SymbolDoc::TypeAlias(t) = &result.module.symbols[0] {
            assert_eq!(t.name, "Result");
            assert!(t.target_type.contains("Result"));
        } else {
            panic!("expected TypeAlias");
        }
    }

    #[test]
    fn test_extract_pub_const() {
        let result = extract_from_source("pub const MAX: u32 = 100;");
        assert_eq!(result.module.symbols.len(), 1);
        if let SymbolDoc::Constant(c) = &result.module.symbols[0] {
            assert_eq!(c.name, "MAX");
            assert_eq!(c.type_str, "u32");
            assert_eq!(c.value.as_deref(), Some("100"));
        } else {
            panic!("expected Constant");
        }
    }

    #[test]
    fn test_extract_doc_comment() {
        let result = extract_from_source(
            "/// This is a documented function.\n/// It does things.\npub fn documented() {}",
        );
        if let SymbolDoc::Function(f) = &result.module.symbols[0] {
            let doc = f.doc.as_ref().expect("should have doc");
            assert!(doc.contains("documented function"));
            assert!(doc.contains("does things"));
        } else {
            panic!("expected Function");
        }
    }

    #[test]
    fn test_extract_deprecated() {
        let result = extract_from_source(
            "#[deprecated(note = \"use new_fn instead\")]\npub fn old_fn() {}",
        );
        if let SymbolDoc::Function(f) = &result.module.symbols[0] {
            assert_eq!(f.deprecated.as_deref(), Some("use new_fn instead"));
        } else {
            panic!("expected Function");
        }
    }

    #[test]
    fn test_extract_deprecated_bare() {
        let result = extract_from_source(
            "#[deprecated]\npub fn old_fn() {}",
        );
        if let SymbolDoc::Function(f) = &result.module.symbols[0] {
            assert!(f.deprecated.is_some());
        } else {
            panic!("expected Function");
        }
    }

    #[test]
    fn test_extract_async_fn() {
        let result = extract_from_source("pub async fn fetch() -> String { String::new() }");
        if let SymbolDoc::Function(f) = &result.module.symbols[0] {
            assert!(f.is_async);
            assert!(f.signature.contains("async"));
        } else {
            panic!("expected Function");
        }
    }

    #[test]
    fn test_extract_generic_fn() {
        let result = extract_from_source("pub fn identity<T: Clone>(val: T) -> T { val }");
        if let SymbolDoc::Function(f) = &result.module.symbols[0] {
            assert!(!f.generic_params.is_empty());
            assert!(f.signature.contains("<T"));
        } else {
            panic!("expected Function");
        }
    }

    #[test]
    fn test_private_excluded_by_default() {
        let result = extract_from_source("fn private_fn() {}\npub fn public_fn() {}");
        assert_eq!(result.module.symbols.len(), 1);
        assert_eq!(result.module.symbols[0].name(), "public_fn");
    }

    #[test]
    fn test_private_included_with_all() {
        let result = extract_from_source_all("fn private_fn() {}\npub fn public_fn() {}");
        assert_eq!(result.module.symbols.len(), 2);
    }

    #[test]
    fn test_impl_methods_associated() {
        let result = extract_from_source(
            "pub struct Foo;\nimpl Foo {\n    pub fn bar(&self) {}\n}",
        );
        if let SymbolDoc::Struct(s) = &result.module.symbols[0] {
            assert_eq!(s.name, "Foo");
            assert_eq!(s.methods.len(), 1);
            assert_eq!(s.methods[0].name, "bar");
        } else {
            panic!("expected Struct");
        }
    }

    #[test]
    fn test_trait_impl_recorded() {
        let result = extract_from_source(
            "pub struct MyNode;\npub trait Node { fn tick(&mut self); }\nimpl Node for MyNode { fn tick(&mut self) {} }",
        );
        if let SymbolDoc::Struct(s) = &result.module.symbols[0] {
            assert!(s.trait_impls.contains(&"Node".to_string()));
        } else {
            panic!("expected Struct");
        }
    }

    #[test]
    fn test_node_impl_entry_point() {
        let result = extract_from_source(
            "pub struct MyNode;\npub trait Node { fn tick(&mut self); }\nimpl Node for MyNode { fn tick(&mut self) {} }",
        );
        assert_eq!(result.entry_points.len(), 1);
        assert_eq!(result.entry_points[0].kind, EntryPointKind::HorusNode);
        assert_eq!(result.entry_points[0].name, "MyNode");
    }

    #[test]
    fn test_use_imports() {
        let result = extract_from_source("pub use std::collections::HashMap;");
        assert_eq!(result.module.imports.len(), 1);
        assert!(result.module.imports[0].contains("HashMap"));
    }

    #[test]
    fn test_scan_todos() {
        let todos = scan_todos(
            "// TODO: fix this\n// FIXME: broken\n// normal comment\n// HACK: workaround\n// SAFETY: pointer is valid",
            Path::new("test.rs"),
        );
        assert_eq!(todos.len(), 4);
        assert_eq!(todos[0].kind, TodoKind::Todo);
        assert_eq!(todos[0].text, "fix this");
        assert_eq!(todos[0].location.line, 1);
        assert_eq!(todos[1].kind, TodoKind::Fixme);
        assert_eq!(todos[2].kind, TodoKind::Hack);
        assert_eq!(todos[3].kind, TodoKind::Safety);
    }

    #[test]
    fn test_scan_todos_no_match() {
        let todos = scan_todos("// This is a normal comment\nlet x = 1;", Path::new("test.rs"));
        assert!(todos.is_empty());
    }

    #[test]
    fn test_extract_examples() {
        let result = extract_from_source(
            "/// A function.\n/// \n/// # Examples\n/// ```\n/// let x = hello();\n/// ```\npub fn hello() {}",
        );
        if let SymbolDoc::Function(f) = &result.module.symbols[0] {
            assert_eq!(f.examples.len(), 1);
            assert!(f.examples[0].contains("let x = hello()"));
        } else {
            panic!("expected Function");
        }
    }

    #[test]
    fn test_module_doc() {
        let result = extract_from_source("//! This is the module doc.\n//! It has two lines.\npub fn f() {}");
        assert!(result.module.module_doc.is_some());
        let doc = result.module.module_doc.unwrap();
        assert!(doc.contains("module doc"));
    }

    #[test]
    fn test_empty_file() {
        let result = extract_from_source("");
        assert!(result.module.symbols.is_empty());
        assert!(result.module.module_doc.is_none());
    }

    #[test]
    fn test_enum_with_fields() {
        let result = extract_from_source(
            "pub enum Shape {\n    Circle { radius: f64 },\n    Rectangle { width: f64, height: f64 },\n}",
        );
        if let SymbolDoc::Enum(e) = &result.module.symbols[0] {
            assert_eq!(e.variants.len(), 2);
            assert_eq!(e.variants[0].fields.len(), 1);
            assert_eq!(e.variants[1].fields.len(), 2);
        } else {
            panic!("expected Enum");
        }
    }

    #[test]
    fn test_static_item() {
        let result = extract_from_source("pub static GLOBAL: u32 = 42;");
        if let SymbolDoc::Constant(c) = &result.module.symbols[0] {
            assert_eq!(c.name, "GLOBAL");
            assert_eq!(c.value.as_deref(), Some("42"));
        } else {
            panic!("expected Constant");
        }
    }

    // ── Horus Macro Extraction Tests ────────────────────────────────────────

    #[test]
    fn test_extract_message_single() {
        let result = extract_from_source(
            "message! {\n    TemperatureReading {\n        sensor_id: u32,\n        celsius: f64,\n    }\n}",
        );
        let msgs: Vec<_> = result.module.symbols.iter().filter(|s| matches!(s, SymbolDoc::HorusMessage(_))).collect();
        assert_eq!(msgs.len(), 1, "should extract 1 message");
        if let SymbolDoc::HorusMessage(m) = &msgs[0] {
            assert_eq!(m.name, "TemperatureReading");
            assert_eq!(m.fields.len(), 2);
            assert_eq!(m.fields[0].name, "sensor_id");
            assert_eq!(m.fields[0].type_str.as_deref(), Some("u32"));
            assert_eq!(m.fields[1].name, "celsius");
            assert_eq!(m.fields[1].type_str.as_deref(), Some("f64"));
        }
    }

    #[test]
    fn test_extract_message_multiple() {
        let result = extract_from_source(
            "message! {\n    Foo { x: f64 }\n    Bar { y: u32, z: String }\n}",
        );
        let msgs: Vec<_> = result.module.symbols.iter().filter(|s| matches!(s, SymbolDoc::HorusMessage(_))).collect();
        assert_eq!(msgs.len(), 2, "should extract 2 messages");
        assert_eq!(msgs[0].name(), "Foo");
        assert_eq!(msgs[1].name(), "Bar");
    }

    #[test]
    fn test_extract_service() {
        let result = extract_from_source(
            "service! {\n    AddTwoInts {\n        request {\n            a: i64,\n            b: i64,\n        }\n        response {\n            sum: i64,\n        }\n    }\n}",
        );
        let svcs: Vec<_> = result.module.symbols.iter().filter(|s| matches!(s, SymbolDoc::HorusService(_))).collect();
        assert_eq!(svcs.len(), 1);
        if let SymbolDoc::HorusService(s) = &svcs[0] {
            assert_eq!(s.name, "AddTwoInts");
            assert_eq!(s.request_fields.len(), 2);
            assert_eq!(s.request_fields[0].name, "a");
            assert_eq!(s.response_fields.len(), 1);
            assert_eq!(s.response_fields[0].name, "sum");
        }
    }

    #[test]
    fn test_extract_action() {
        let result = extract_from_source(
            "action! {\n    Navigate {\n        goal {\n            x: f64,\n            y: f64,\n        }\n        feedback {\n            progress: f32,\n        }\n        result {\n            success: bool,\n        }\n    }\n}",
        );
        let acts: Vec<_> = result.module.symbols.iter().filter(|s| matches!(s, SymbolDoc::HorusAction(_))).collect();
        assert_eq!(acts.len(), 1);
        if let SymbolDoc::HorusAction(a) = &acts[0] {
            assert_eq!(a.name, "Navigate");
            assert_eq!(a.goal_fields.len(), 2);
            assert_eq!(a.goal_fields[0].name, "x");
            assert_eq!(a.feedback_fields.len(), 1);
            assert_eq!(a.feedback_fields[0].name, "progress");
            assert_eq!(a.result_fields.len(), 1);
            assert_eq!(a.result_fields[0].name, "success");
        }
    }

    #[test]
    fn test_no_false_positive_macro() {
        let result = extract_from_source("println!(\"hello world\");");
        let msgs: Vec<_> = result.module.symbols.iter().filter(|s| matches!(s, SymbolDoc::HorusMessage(_))).collect();
        assert!(msgs.is_empty(), "println should not be detected as message macro");
    }

    #[test]
    fn test_message_with_generic_type() {
        let result = extract_from_source(
            "message! {\n    Scan {\n        points: Vec<f32>,\n        header: Option<String>,\n    }\n}",
        );
        let msgs: Vec<_> = result.module.symbols.iter().filter(|s| matches!(s, SymbolDoc::HorusMessage(_))).collect();
        assert_eq!(msgs.len(), 1);
        if let SymbolDoc::HorusMessage(m) = &msgs[0] {
            assert_eq!(m.fields.len(), 2);
            // Verify generic types parsed correctly (angle brackets handled)
            let points_type = m.fields[0].type_str.as_deref().unwrap();
            assert!(points_type.contains("Vec"), "got: {points_type}");
        }
    }

    #[test]
    fn test_extract_tuple_struct() {
        let result = extract_from_source("pub struct Point(pub f64, pub f64);");
        assert_eq!(result.module.symbols.len(), 1);
        if let SymbolDoc::Struct(s) = &result.module.symbols[0] {
            assert_eq!(s.name, "Point");
            assert_eq!(s.fields.len(), 2);
            assert_eq!(s.fields[0].name, "0");
            assert_eq!(s.fields[1].name, "1");
            assert_eq!(s.fields[0].type_str.as_deref(), Some("f64"));
            assert_eq!(s.fields[1].type_str.as_deref(), Some("f64"));
        } else {
            panic!("expected Struct");
        }
    }

    #[test]
    fn test_extract_unit_struct() {
        let result = extract_from_source("pub struct Marker;");
        assert_eq!(result.module.symbols.len(), 1);
        if let SymbolDoc::Struct(s) = &result.module.symbols[0] {
            assert_eq!(s.name, "Marker");
            assert!(s.fields.is_empty());
        } else {
            panic!("expected Struct");
        }
    }

    #[test]
    fn test_extract_pub_crate_fn() {
        // pub(crate) is Restricted visibility — is_public returns true for it
        let result = extract_from_source("pub(crate) fn internal() {}");
        assert_eq!(result.module.symbols.len(), 1);
        if let SymbolDoc::Function(f) = &result.module.symbols[0] {
            assert_eq!(f.name, "internal");
            assert_eq!(f.visibility, Visibility::PublicCrate);
        } else {
            panic!("expected Function");
        }
    }

    #[test]
    fn test_extract_lifetime_fn() {
        let result = extract_from_source(
            "pub fn borrow<'a>(s: &'a str) -> &'a str { s }",
        );
        assert_eq!(result.module.symbols.len(), 1);
        if let SymbolDoc::Function(f) = &result.module.symbols[0] {
            assert_eq!(f.name, "borrow");
            assert!(!f.generic_params.is_empty(), "should have generic params");
            assert!(
                f.generic_params.iter().any(|g| g.contains("'a")),
                "generic_params should contain lifetime 'a, got: {:?}",
                f.generic_params
            );
        } else {
            panic!("expected Function");
        }
    }

    #[test]
    fn test_extract_where_clause() {
        let result = extract_from_source(
            "pub fn complex<T>(v: T) -> T where T: Clone + std::fmt::Debug { v }",
        );
        assert_eq!(result.module.symbols.len(), 1);
        if let SymbolDoc::Function(f) = &result.module.symbols[0] {
            assert_eq!(f.name, "complex");
            assert_eq!(f.params.len(), 1);
            assert_eq!(f.params[0].name, "v");
            assert_eq!(f.returns.as_deref(), Some("T"));
            // Function with where clause is extracted correctly even though
            // the signature builder omits the where clause text
            assert!(f.signature.contains("pub fn complex"), "signature: {}", f.signature);
            assert!(f.signature.contains("-> T"), "return type in signature: {}", f.signature);
        } else {
            panic!("expected Function");
        }
    }

    #[test]
    fn test_extract_nested_generics() {
        let result = extract_from_source(
            "pub fn nested() -> std::collections::HashMap<String, Vec<u32>> { todo!() }",
        );
        assert_eq!(result.module.symbols.len(), 1);
        if let SymbolDoc::Function(f) = &result.module.symbols[0] {
            assert_eq!(f.name, "nested");
            let ret = f.returns.as_deref().expect("should have return type");
            assert!(
                ret.contains("HashMap"),
                "return type should contain HashMap, got: {ret}"
            );
        } else {
            panic!("expected Function");
        }
    }

    #[test]
    fn test_multiple_impl_blocks() {
        let result = extract_from_source(
            "pub struct Foo;\nimpl Foo { pub fn a(&self) {} }\nimpl Foo { pub fn b(&self) {} }",
        );
        assert_eq!(result.module.symbols.len(), 1);
        if let SymbolDoc::Struct(s) = &result.module.symbols[0] {
            assert_eq!(s.name, "Foo");
            assert_eq!(s.methods.len(), 2, "Foo should have 2 methods from separate impl blocks");
            let method_names: Vec<&str> = s.methods.iter().map(|m| m.name.as_str()).collect();
            assert!(method_names.contains(&"a"), "should contain method 'a'");
            assert!(method_names.contains(&"b"), "should contain method 'b'");
        } else {
            panic!("expected Struct");
        }
    }

    #[test]
    fn test_extract_trait_impl_methods() {
        let result = extract_from_source(
            "pub struct Bar;\npub trait Baz { fn do_thing(&self); }\nimpl Baz for Bar { fn do_thing(&self) {} }",
        );
        // Should have Bar (struct) and Baz (trait)
        let structs: Vec<_> = result.module.symbols.iter()
            .filter_map(|s| if let SymbolDoc::Struct(st) = s { Some(st) } else { None })
            .collect();
        let traits: Vec<_> = result.module.symbols.iter()
            .filter_map(|s| if let SymbolDoc::Trait(t) = s { Some(t) } else { None })
            .collect();
        assert_eq!(structs.len(), 1, "should have 1 struct");
        assert_eq!(traits.len(), 1, "should have 1 trait");

        // Bar should list Baz in trait_impls
        assert!(
            structs[0].trait_impls.contains(&"Baz".to_string()),
            "Bar.trait_impls should contain Baz, got: {:?}",
            structs[0].trait_impls
        );

        // Baz trait should have do_thing as required method
        assert_eq!(traits[0].name, "Baz");
        assert_eq!(traits[0].required_methods.len(), 1);
        assert_eq!(traits[0].required_methods[0].name, "do_thing");
    }

    // ── Level 2: Integration tests (extract → verify cross-module contract) ──

    #[test]
    fn test_integration_rust_extract_produces_valid_project_doc_symbols() {
        // Create a realistic file with a struct implementing a trait, a message!
        // macro, and a TODO comment — then verify ALL extractor features work
        // together on a single file.
        let source = r#"
//! Module-level docs for the integration test.

// TODO: refactor this module later

pub trait Node {
    fn tick(&mut self);
}

/// A sensor node that reads temperature.
pub struct TempSensor {
    pub id: u32,
    pub celsius: f64,
}

impl Node for TempSensor {
    fn tick(&mut self) {}
}

impl TempSensor {
    pub fn new(id: u32) -> Self {
        Self { id, celsius: 0.0 }
    }
}

message! {
    SensorReading {
        sensor_id: u32,
        value: f64,
    }
}
"#;
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("test.rs");
        std::fs::write(&file, source).unwrap();
        let result = extract_rust_file(&file, false).unwrap();

        // StructDoc should have trait_impls populated
        let structs: Vec<_> = result.module.symbols.iter()
            .filter_map(|s| if let SymbolDoc::Struct(st) = s { Some(st) } else { None })
            .collect();
        assert!(!structs.is_empty(), "should extract at least one struct");
        let temp_sensor = structs.iter().find(|s| s.name == "TempSensor")
            .expect("TempSensor struct should be extracted");
        assert!(
            temp_sensor.trait_impls.contains(&"Node".to_string()),
            "TempSensor.trait_impls should contain Node, got: {:?}",
            temp_sensor.trait_impls
        );
        // Should also have the `new` method from inherent impl
        assert!(
            temp_sensor.methods.iter().any(|m| m.name == "new"),
            "TempSensor should have 'new' method"
        );

        // HorusMessageDoc should have fields
        let messages: Vec<_> = result.module.symbols.iter()
            .filter_map(|s| if let SymbolDoc::HorusMessage(m) = s { Some(m) } else { None })
            .collect();
        assert_eq!(messages.len(), 1, "should extract 1 message");
        assert_eq!(messages[0].name, "SensorReading");
        assert_eq!(messages[0].fields.len(), 2, "SensorReading should have 2 fields");

        // TodoItem should be captured
        assert!(!result.todos.is_empty(), "should capture TODO comment");
        assert!(
            result.todos.iter().any(|t| t.text.contains("refactor")),
            "TODO text should contain 'refactor'"
        );

        // entry_points should contain HorusNode because `impl Node` exists
        assert!(
            result.entry_points.iter().any(|ep| ep.kind == EntryPointKind::HorusNode && ep.name == "TempSensor"),
            "entry_points should contain TempSensor as HorusNode, got: {:?}",
            result.entry_points
        );

        // Module doc captured
        assert!(result.module.module_doc.is_some(), "module doc should be captured");
        assert!(
            result.module.module_doc.as_ref().unwrap().contains("integration test"),
            "module doc should contain 'integration test'"
        );
    }

    #[test]
    fn test_integration_rust_extract_multiple_files_in_dir() {
        use crate::commands::doc_extract::{extract_project, ExtractConfig};

        let dir = tempfile::tempdir().unwrap();
        let src_dir = dir.path().join("src");
        std::fs::create_dir_all(&src_dir).unwrap();

        // lib.rs — a struct and a function
        std::fs::write(
            src_dir.join("lib.rs"),
            "/// Library root.\npub struct AppConfig {\n    pub debug: bool,\n}\npub fn init() {}",
        ).unwrap();

        // helper.rs — a different function and constant
        std::fs::write(
            src_dir.join("helper.rs"),
            "/// Helper utilities.\npub fn compute(x: f64) -> f64 { x * 2.0 }\npub const VERSION: u32 = 1;",
        ).unwrap();

        let config = ExtractConfig {
            json: false, md: false, html: false, brief: false, full: false,
            all: false, lang: None, coverage: false,
            output: None, watch: false, diff: None, fail_under: None,
        };
        let doc = extract_project(dir.path(), &config).unwrap();

        // Should have 2 modules (one per file)
        assert_eq!(
            doc.modules.len(), 2,
            "should extract 2 modules, got {}",
            doc.modules.len()
        );

        // Collect all symbol names across modules
        let all_names: Vec<&str> = doc.modules.iter()
            .flat_map(|m| m.symbols.iter().map(|s| s.name()))
            .collect();

        assert!(all_names.contains(&"AppConfig"), "should find AppConfig from lib.rs");
        assert!(all_names.contains(&"init"), "should find init from lib.rs");
        assert!(all_names.contains(&"compute"), "should find compute from helper.rs");
        assert!(all_names.contains(&"VERSION"), "should find VERSION from helper.rs");
    }

    // ── Level 5: Error path tests ────────────────────────────────────────────

    #[test]
    fn test_error_rust_syntax_error_returns_err() {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("bad.rs");
        std::fs::write(&file, "pub fn broken( { }}}}}").unwrap();
        let result = extract_rust_file(&file, false);
        assert!(result.is_err(), "invalid Rust syntax should return Err, not panic");
    }

    #[test]
    fn test_error_rust_nonexistent_file() {
        let path = std::path::Path::new("/tmp/does_not_exist_ever_abc123xyz.rs");
        let result = extract_rust_file(path, false);
        assert!(result.is_err(), "nonexistent file should return Err");
    }

    #[test]
    fn test_error_rust_binary_file() {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("binary.rs");
        // Write bytes that are not valid UTF-8
        std::fs::write(&file, &[0xFF, 0xFE, 0x00, 0x01, 0x80, 0x81, 0xCC, 0xDD]).unwrap();
        let result = extract_rust_file(&file, false);
        assert!(result.is_err(), "binary/non-UTF-8 file should return Err, not panic");
    }
}
