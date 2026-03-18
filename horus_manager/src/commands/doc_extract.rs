//! Machine-readable documentation extraction for AI-assisted development.
//!
//! `horus doc --extract` parses source code across Rust, C++, and Python
//! and outputs structured API maps (JSON, brief text, markdown, HTML)
//! that give AI agents complete understanding of a project's public surface.

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::fmt::Write;
use std::path::{Path, PathBuf};

// ─── Source Location ────────────────────────────────────────────────────────

/// Source location for any symbol — enables click-to-source and agent navigation.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct SourceLocation {
    pub file: PathBuf,
    pub line: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub end_line: Option<usize>,
}

// ─── Top-Level Types ────────────────────────────────────────────────────────

/// Top-level project documentation.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct ProjectDoc {
    pub project: String,
    pub version: String,
    pub languages: Vec<String>,
    pub module_tree: ModuleTree,
    pub modules: Vec<ModuleDoc>,
    pub relationships: Vec<Relationship>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub message_graph: Option<MessageGraph>,
    pub entry_points: Vec<EntryPoint>,
    pub todos: Vec<TodoItem>,
    pub stats: DocStats,
}

/// A single source file's extracted documentation.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct ModuleDoc {
    pub path: PathBuf,
    pub language: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub module_doc: Option<String>,
    pub imports: Vec<String>,
    pub symbols: Vec<SymbolDoc>,
}

// ─── Symbol Types ───────────────────────────────────────────────────────────

/// Any extractable symbol.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum SymbolDoc {
    Function(FunctionDoc),
    Struct(StructDoc),
    Enum(EnumDoc),
    Trait(TraitDoc),
    TypeAlias(TypeAliasDoc),
    Constant(ConstantDoc),
    HorusMessage(HorusMessageDoc),
    HorusService(HorusServiceDoc),
    HorusAction(HorusActionDoc),
}

impl SymbolDoc {
    /// Get the symbol name regardless of variant.
    pub fn name(&self) -> &str {
        match self {
            SymbolDoc::Function(f) => &f.name,
            SymbolDoc::Struct(s) => &s.name,
            SymbolDoc::Enum(e) => &e.name,
            SymbolDoc::Trait(t) => &t.name,
            SymbolDoc::TypeAlias(t) => &t.name,
            SymbolDoc::Constant(c) => &c.name,
            SymbolDoc::HorusMessage(m) => &m.name,
            SymbolDoc::HorusService(s) => &s.name,
            SymbolDoc::HorusAction(a) => &a.name,
        }
    }

    /// Get the doc comment regardless of variant.
    pub fn doc(&self) -> Option<&str> {
        match self {
            SymbolDoc::Function(f) => f.doc.as_deref(),
            SymbolDoc::Struct(s) => s.doc.as_deref(),
            SymbolDoc::Enum(e) => e.doc.as_deref(),
            SymbolDoc::Trait(t) => t.doc.as_deref(),
            SymbolDoc::TypeAlias(t) => t.doc.as_deref(),
            SymbolDoc::Constant(c) => c.doc.as_deref(),
            SymbolDoc::HorusMessage(m) => m.doc.as_deref(),
            SymbolDoc::HorusService(s) => s.doc.as_deref(),
            SymbolDoc::HorusAction(a) => a.doc.as_deref(),
        }
    }

    /// Get the deprecated annotation regardless of variant.
    pub fn deprecated(&self) -> Option<&str> {
        match self {
            SymbolDoc::Function(f) => f.deprecated.as_deref(),
            SymbolDoc::Struct(s) => s.deprecated.as_deref(),
            SymbolDoc::Enum(e) => e.deprecated.as_deref(),
            SymbolDoc::Trait(t) => t.deprecated.as_deref(),
            SymbolDoc::TypeAlias(t) => t.deprecated.as_deref(),
            SymbolDoc::Constant(c) => c.deprecated.as_deref(),
            SymbolDoc::HorusMessage(m) => m.deprecated.as_deref(),
            SymbolDoc::HorusService(_) => None,
            SymbolDoc::HorusAction(_) => None,
        }
    }

    /// Get the source location regardless of variant.
    pub fn location(&self) -> Option<&SourceLocation> {
        match self {
            SymbolDoc::Function(f) => Some(&f.location),
            SymbolDoc::Struct(s) => Some(&s.location),
            SymbolDoc::Enum(e) => Some(&e.location),
            SymbolDoc::Trait(t) => Some(&t.location),
            SymbolDoc::TypeAlias(t) => Some(&t.location),
            SymbolDoc::Constant(c) => Some(&c.location),
            SymbolDoc::HorusMessage(m) => Some(&m.location),
            SymbolDoc::HorusService(s) => Some(&s.location),
            SymbolDoc::HorusAction(a) => Some(&a.location),
        }
    }
}

/// Visibility of a symbol.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum Visibility {
    Public,
    PublicCrate,
    Private,
}

/// A function or method.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct FunctionDoc {
    pub name: String,
    pub visibility: Visibility,
    pub location: SourceLocation,
    pub signature: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub doc: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub deprecated: Option<String>,
    pub params: Vec<ParamDoc>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub returns: Option<String>,
    pub is_async: bool,
    pub generic_params: Vec<String>,
    pub examples: Vec<String>,
}

/// A function parameter.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct ParamDoc {
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub type_str: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub default_value: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub doc: Option<String>,
}

/// A struct or class.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct StructDoc {
    pub name: String,
    pub visibility: Visibility,
    pub location: SourceLocation,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub doc: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub deprecated: Option<String>,
    pub generic_params: Vec<String>,
    pub fields: Vec<FieldDoc>,
    pub methods: Vec<FunctionDoc>,
    pub trait_impls: Vec<String>,
    pub derives: Vec<String>,
    pub examples: Vec<String>,
}

/// A struct or class field.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct FieldDoc {
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub type_str: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub doc: Option<String>,
}

/// An enum type.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct EnumDoc {
    pub name: String,
    pub visibility: Visibility,
    pub location: SourceLocation,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub doc: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub deprecated: Option<String>,
    pub variants: Vec<VariantDoc>,
    pub methods: Vec<FunctionDoc>,
}

/// An enum variant.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct VariantDoc {
    pub name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub doc: Option<String>,
    pub fields: Vec<FieldDoc>,
}

/// A trait or interface.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct TraitDoc {
    pub name: String,
    pub visibility: Visibility,
    pub location: SourceLocation,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub doc: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub deprecated: Option<String>,
    pub generic_params: Vec<String>,
    pub required_methods: Vec<FunctionDoc>,
    pub provided_methods: Vec<FunctionDoc>,
    pub implementors: Vec<String>,
}

/// A type alias.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct TypeAliasDoc {
    pub name: String,
    pub visibility: Visibility,
    pub location: SourceLocation,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub doc: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub deprecated: Option<String>,
    pub target_type: String,
}

/// A constant or static value.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct ConstantDoc {
    pub name: String,
    pub visibility: Visibility,
    pub location: SourceLocation,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub doc: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub deprecated: Option<String>,
    pub type_str: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub value: Option<String>,
}

// ─── Horus-Specific Types ───────────────────────────────────────────────────

/// Extracted from `message! {}` macro invocation.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct HorusMessageDoc {
    pub name: String,
    pub location: SourceLocation,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub doc: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub deprecated: Option<String>,
    pub fields: Vec<FieldDoc>,
}

/// Extracted from `service! {}` macro invocation.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct HorusServiceDoc {
    pub name: String,
    pub location: SourceLocation,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub doc: Option<String>,
    pub request_fields: Vec<FieldDoc>,
    pub response_fields: Vec<FieldDoc>,
}

/// Extracted from `action! {}` macro invocation.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct HorusActionDoc {
    pub name: String,
    pub location: SourceLocation,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub doc: Option<String>,
    pub goal_fields: Vec<FieldDoc>,
    pub feedback_fields: Vec<FieldDoc>,
    pub result_fields: Vec<FieldDoc>,
}

// ─── TODO/FIXME ─────────────────────────────────────────────────────────────

/// A TODO/FIXME/HACK comment found in source.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct TodoItem {
    pub kind: TodoKind,
    pub text: String,
    pub location: SourceLocation,
}

/// Kind of TODO comment.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum TodoKind {
    Todo,
    Fixme,
    Hack,
    Safety,
}

// ─── Module Hierarchy ───────────────────────────────────────────────────────

/// Hierarchical module structure (crate → module → submodule).
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct ModuleTree {
    pub name: String,
    pub children: Vec<ModuleTreeNode>,
}

/// A node in the module hierarchy tree.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct ModuleTreeNode {
    pub name: String,
    pub path: PathBuf,
    pub children: Vec<ModuleTreeNode>,
    pub symbol_count: usize,
}

// ─── Relationships ──────────────────────────────────────────────────────────

/// Cross-symbol relationships discovered during extraction.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Relationship {
    /// struct X implements trait Y
    Implements {
        implementor: String,
        trait_name: String,
        file: PathBuf,
    },
    /// class X inherits from Y (C++/Python)
    Inherits {
        child: String,
        parent: String,
        file: PathBuf,
    },
    /// module X imports symbol Y from module Z
    Uses {
        user: String,
        used: String,
        from_module: String,
    },
    /// node X publishes message Y on topic Z
    Publishes {
        node: String,
        message_type: String,
        topic: String,
    },
    /// node X subscribes to message Y on topic Z
    Subscribes {
        node: String,
        message_type: String,
        topic: String,
    },
    /// service client in X calls service Y
    CallsService { caller: String, service: String },
}

// ─── Message Graph ──────────────────────────────────────────────────────────

/// The message flow graph: which nodes talk to which via what topics.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct MessageGraph {
    pub nodes: Vec<String>,
    pub topics: Vec<MessageGraphTopic>,
}

/// A topic in the message flow graph.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct MessageGraphTopic {
    pub name: String,
    pub message_type: String,
    pub publishers: Vec<String>,
    pub subscribers: Vec<String>,
}

// ─── Entry Points ───────────────────────────────────────────────────────────

/// A discovered entry point (main function, horus node, ROS2 node).
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct EntryPoint {
    pub kind: EntryPointKind,
    pub name: String,
    pub file: PathBuf,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<EntryPointDetails>,
}

/// Kind of entry point.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum EntryPointKind {
    MainFunction,
    HorusNode,
    Ros2Node,
    PythonScript,
}

/// Details for a horus/ROS2 node entry point.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct EntryPointDetails {
    pub publishes: Vec<TopicInfo>,
    pub subscribes: Vec<TopicInfo>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tick_rate: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub execution_class: Option<String>,
}

/// Topic publication or subscription info.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct TopicInfo {
    pub name: String,
    pub message_type: String,
    pub direction: String,
}

// ─── Statistics ─────────────────────────────────────────────────────────────

/// Documentation statistics for the project.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct DocStats {
    pub total_files: usize,
    pub total_symbols: usize,
    pub documented_symbols: usize,
    pub deprecated_symbols: usize,
    pub documentation_coverage: f32,
    pub horus_nodes: usize,
    pub horus_messages: usize,
    pub horus_services: usize,
    pub horus_actions: usize,
    pub topics_discovered: usize,
    pub todos: usize,
    pub fixmes: usize,
}

// ─── Extraction Config ──────────────────────────────────────────────────────

/// Configuration for the extraction pipeline.
pub struct ExtractConfig {
    pub json: bool,
    pub md: bool,
    pub html: bool,
    pub brief: bool,
    pub full: bool,
    pub all: bool,
    pub lang: Option<String>,
    pub coverage: bool,
    pub output: Option<PathBuf>,
    pub watch: bool,
    pub diff: Option<PathBuf>,
    pub fail_under: Option<u32>,
}

// ─── Formatters ─────────────────────────────────────────────────────────────

/// Format ProjectDoc as pretty-printed JSON.
pub fn format_json(doc: &ProjectDoc) -> String {
    serde_json::to_string_pretty(doc).unwrap_or_else(|e| format!("{{\"error\": \"{e}\"}}"))
}

/// Format ProjectDoc as brief one-liner-per-symbol text.
pub fn format_brief(doc: &ProjectDoc) -> String {
    let mut out = String::new();
    let _ = writeln!(
        out,
        "# {} v{} — {} symbols, {:.0}% documented",
        doc.project,
        doc.version,
        doc.stats.total_symbols,
        doc.stats.documentation_coverage * 100.0
    );

    // Horus summary
    if doc.stats.horus_nodes > 0 || doc.stats.horus_messages > 0 {
        let _ = writeln!(
            out,
            "# {} nodes, {} messages, {} topics",
            doc.stats.horus_nodes, doc.stats.horus_messages, doc.stats.topics_discovered
        );
    }
    let _ = writeln!(out);

    // Message Types section
    let messages: Vec<&HorusMessageDoc> = doc
        .modules
        .iter()
        .flat_map(|m| m.symbols.iter())
        .filter_map(|s| match s {
            SymbolDoc::HorusMessage(m) => Some(m),
            _ => None,
        })
        .collect();
    if !messages.is_empty() {
        let _ = writeln!(out, "## Message Types");
        for msg in &messages {
            let fields: Vec<String> = msg
                .fields
                .iter()
                .map(|f| format!("{}: {}", f.name, f.type_str.as_deref().unwrap_or("?")))
                .collect();
            let _ = writeln!(out, "  {} {{ {} }}", msg.name, fields.join(", "));
        }
        let _ = writeln!(out);
    }

    // Nodes section
    let nodes: Vec<&EntryPoint> = doc
        .entry_points
        .iter()
        .filter(|e| e.kind == EntryPointKind::HorusNode)
        .collect();
    if !nodes.is_empty() {
        let _ = writeln!(out, "## Nodes");
        for node in &nodes {
            let mut desc = format!("  {} (impl Node)", node.name);
            if let Some(ref details) = node.details {
                if let Some(ref rate) = details.tick_rate {
                    desc.push_str(&format!(" [{rate}]"));
                }
            }
            let _ = writeln!(out, "{desc}");
            if let Some(ref details) = node.details {
                for t in &details.publishes {
                    let _ = writeln!(out, "    pub → {}: {}", t.name, t.message_type);
                }
                for t in &details.subscribes {
                    let _ = writeln!(out, "    sub ← {}: {}", t.name, t.message_type);
                }
            }
        }
        let _ = writeln!(out);
    }

    // Topic Graph section
    if let Some(ref graph) = doc.message_graph {
        if !graph.topics.is_empty() {
            let _ = writeln!(out, "## Topic Graph");
            for topic in &graph.topics {
                let pubs = topic.publishers.join(", ");
                let subs = topic.subscribers.join(", ");
                let _ = writeln!(
                    out,
                    "  {}: {}    {} → {}",
                    topic.name, topic.message_type, pubs, subs
                );
            }
            let _ = writeln!(out);
        }
    }

    // Per-file sections
    for module in &doc.modules {
        let path_str = module.path.display();
        if let Some(ref mdoc) = module.module_doc {
            let _ = writeln!(out, "## {path_str} — {mdoc}");
        } else {
            let _ = writeln!(out, "## {path_str}");
        }
        for sym in &module.symbols {
            let dep_marker = if sym.deprecated().is_some() {
                "⚠ "
            } else {
                ""
            };
            match sym {
                SymbolDoc::Function(f) => {
                    let _ = writeln!(out, "  {dep_marker}{}", f.signature);
                }
                SymbolDoc::Struct(s) => {
                    let fields: Vec<String> = s
                        .fields
                        .iter()
                        .map(|f| format!("{}: {}", f.name, f.type_str.as_deref().unwrap_or("?")))
                        .collect();
                    let _ = writeln!(
                        out,
                        "  {dep_marker}struct {} {{ {} }}",
                        s.name,
                        fields.join(", ")
                    );
                    for ti in &s.trait_impls {
                        let _ = writeln!(out, "    impl {ti}");
                    }
                    for m in &s.methods {
                        let _ = writeln!(out, "    {}", m.signature);
                    }
                }
                SymbolDoc::Enum(e) => {
                    let variants: Vec<&str> = e.variants.iter().map(|v| v.name.as_str()).collect();
                    let _ = writeln!(
                        out,
                        "  {dep_marker}enum {} {{ {} }}",
                        e.name,
                        variants.join(", ")
                    );
                }
                SymbolDoc::Trait(t) => {
                    let methods: Vec<&str> =
                        t.required_methods.iter().map(|m| m.name.as_str()).collect();
                    let _ = writeln!(
                        out,
                        "  {dep_marker}trait {} {{ {} }}",
                        t.name,
                        methods.join(", ")
                    );
                }
                SymbolDoc::TypeAlias(t) => {
                    let _ = writeln!(out, "  {dep_marker}type {} = {}", t.name, t.target_type);
                }
                SymbolDoc::Constant(c) => {
                    let val = c.value.as_deref().unwrap_or("...");
                    let _ = writeln!(
                        out,
                        "  {dep_marker}const {}: {} = {val}",
                        c.name, c.type_str
                    );
                }
                SymbolDoc::HorusMessage(_) => {} // already printed above
                SymbolDoc::HorusService(s) => {
                    let _ = writeln!(out, "  service {}", s.name);
                }
                SymbolDoc::HorusAction(a) => {
                    let _ = writeln!(out, "  action {}", a.name);
                }
            }
        }
        let _ = writeln!(out);
    }

    out
}

/// Format ProjectDoc as markdown suitable for LLM context.
pub fn format_markdown(doc: &ProjectDoc) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "# {} v{} — API Reference\n", doc.project, doc.version);
    let _ = writeln!(
        out,
        "**{} symbols** across {} files, **{:.0}% documented**\n",
        doc.stats.total_symbols,
        doc.stats.total_files,
        doc.stats.documentation_coverage * 100.0
    );

    // Include full brief output
    let brief = format_brief(doc);
    // Add doc comments under each symbol for --full/--md mode
    for module in &doc.modules {
        let path_str = module.path.display();
        let _ = writeln!(out, "### {path_str}\n");
        if let Some(ref mdoc) = module.module_doc {
            let _ = writeln!(out, "{mdoc}\n");
        }
        for sym in &module.symbols {
            let _ = writeln!(out, "#### {}\n", sym.name());
            if let Some(doc_text) = sym.doc() {
                let _ = writeln!(out, "{doc_text}\n");
            }
            if let Some(dep) = sym.deprecated() {
                let _ = writeln!(out, "> **Deprecated**: {dep}\n");
            }
        }
    }

    // Append brief for structure
    let _ = writeln!(out, "---\n\n{brief}");
    out
}

/// Format documentation coverage report.
pub fn format_coverage(doc: &ProjectDoc) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "Documentation Coverage Report");
    let _ = writeln!(out, "=============================\n");

    for module in &doc.modules {
        let total = module.symbols.len();
        let documented = module.symbols.iter().filter(|s| s.doc().is_some()).count();
        let pct = if total > 0 {
            (documented as f32 / total as f32 * 100.0) as u32
        } else {
            100
        };
        let flag = if pct < 50 { "  ← needs docs" } else { "" };
        let _ = writeln!(
            out,
            "  {:<40} {}/{} symbols documented  ({}%){flag}",
            module.path.display(),
            documented,
            total,
            pct
        );
    }

    let _ = writeln!(
        out,
        "\n  Total: {}/{} ({:.0}%)",
        doc.stats.documented_symbols,
        doc.stats.total_symbols,
        doc.stats.documentation_coverage * 100.0
    );

    // Horus-specific
    if doc.stats.horus_nodes > 0 || doc.stats.horus_messages > 0 || doc.stats.horus_services > 0 {
        let _ = writeln!(out, "\n  Horus-specific:");
        let _ = writeln!(out, "    Nodes: {}", doc.stats.horus_nodes);
        let _ = writeln!(out, "    Messages: {}", doc.stats.horus_messages);
        let _ = writeln!(out, "    Services: {}", doc.stats.horus_services);
        let _ = writeln!(out, "    Actions: {}", doc.stats.horus_actions);
        let _ = writeln!(
            out,
            "    Topics: {} discovered",
            doc.stats.topics_discovered
        );
    }

    // Undocumented
    let undocumented: Vec<(String, String)> = doc
        .modules
        .iter()
        .flat_map(|m| {
            m.symbols
                .iter()
                .filter(|s| s.doc().is_none())
                .map(move |s| (m.path.display().to_string(), s.name().to_string()))
        })
        .collect();
    if !undocumented.is_empty() {
        let _ = writeln!(out, "\n  Undocumented:");
        for (file, name) in &undocumented {
            let _ = writeln!(out, "    {file}: {name}");
        }
    }

    out
}

// ─── Stats Computation ──────────────────────────────────────────────────────

/// Compute documentation statistics from extracted modules and entry points.
pub fn compute_stats(
    modules: &[ModuleDoc],
    entry_points: &[EntryPoint],
    todos: &[TodoItem],
) -> DocStats {
    let total_files = modules.len();
    let total_symbols: usize = modules.iter().map(|m| m.symbols.len()).sum();
    let documented_symbols: usize = modules
        .iter()
        .flat_map(|m| m.symbols.iter())
        .filter(|s| s.doc().is_some())
        .count();
    let deprecated_symbols: usize = modules
        .iter()
        .flat_map(|m| m.symbols.iter())
        .filter(|s| s.deprecated().is_some())
        .count();
    let documentation_coverage = if total_symbols > 0 {
        documented_symbols as f32 / total_symbols as f32
    } else {
        1.0
    };

    let mut horus_messages = 0;
    let mut horus_services = 0;
    let mut horus_actions = 0;
    for module in modules {
        for sym in &module.symbols {
            match sym {
                SymbolDoc::HorusMessage(_) => horus_messages += 1,
                SymbolDoc::HorusService(_) => horus_services += 1,
                SymbolDoc::HorusAction(_) => horus_actions += 1,
                _ => {}
            }
        }
    }

    let horus_nodes = entry_points
        .iter()
        .filter(|e| e.kind == EntryPointKind::HorusNode)
        .count();
    let topics_discovered = entry_points
        .iter()
        .filter_map(|e| e.details.as_ref())
        .map(|d| d.publishes.len() + d.subscribes.len())
        .sum();

    let todo_count = todos.iter().filter(|t| t.kind == TodoKind::Todo).count();
    let fixme_count = todos.iter().filter(|t| t.kind == TodoKind::Fixme).count();

    DocStats {
        total_files,
        total_symbols,
        documented_symbols,
        deprecated_symbols,
        documentation_coverage,
        horus_nodes,
        horus_messages,
        horus_services,
        horus_actions,
        topics_discovered,
        todos: todo_count,
        fixmes: fixme_count,
    }
}

// ─── Orchestrator ───────────────────────────────────────────────────────────

/// Run the extraction pipeline.
pub fn run_extract(config: ExtractConfig) -> Result<()> {
    // Handle --watch mode
    if config.watch {
        return run_extract_watch(config);
    }

    let project_dir = std::env::current_dir()?;

    // Handle --diff mode
    if let Some(ref baseline_path) = config.diff {
        let baseline_json = std::fs::read_to_string(baseline_path)
            .with_context(|| format!("Failed to read baseline {}", baseline_path.display()))?;
        let baseline: ProjectDoc = serde_json::from_str(&baseline_json).with_context(|| {
            format!(
                "Failed to parse baseline JSON from {}",
                baseline_path.display()
            )
        })?;
        let current = extract_project(&project_dir, &config)?;
        let diff = crate::commands::doc_extract_diff::compute_diff(&baseline, &current);
        let output = if config.json {
            super::doc_extract_diff::format_diff_json(&diff)
        } else {
            super::doc_extract_diff::format_diff_text(&diff)
        };
        write_output(&output, &config)?;
        if diff.breaking_changes > 0 {
            eprintln!("{} breaking API changes detected", diff.breaking_changes);
            std::process::exit(1);
        }
        return Ok(());
    }

    let doc = extract_project(&project_dir, &config)?;
    let output = format_output(&doc, &config);
    write_output(&output, &config)?;

    // Check --fail-under threshold
    if config.coverage {
        if let Some(threshold) = config.fail_under {
            let coverage_pct = (doc.stats.documentation_coverage * 100.0) as u32;
            if coverage_pct < threshold {
                eprintln!("Documentation coverage {coverage_pct}% is below threshold {threshold}%");
                std::process::exit(1);
            }
        }
    }

    Ok(())
}

/// Extract the full project documentation.
pub fn extract_project(project_dir: &Path, config: &ExtractConfig) -> Result<ProjectDoc> {
    let ctx = crate::dispatch::detect_context(project_dir);
    let manifest_path = project_dir.join(crate::manifest::HORUS_TOML);
    let manifest = crate::manifest::HorusManifest::load_from(&manifest_path).ok();

    let project_name = manifest
        .as_ref()
        .map(|m| m.package.name.clone())
        .unwrap_or_else(|| {
            project_dir
                .file_name()
                .map(|n| n.to_string_lossy().to_string())
                .unwrap_or_else(|| "project".to_string())
        });
    let version = manifest
        .as_ref()
        .map(|m| m.package.version.clone())
        .unwrap_or_default();

    let mut modules = Vec::new();
    let mut entry_points = Vec::new();
    let mut todos = Vec::new();

    // Collect source directories to walk
    let mut walk_dirs = Vec::new();
    let src_dir = project_dir.join("src");
    if src_dir.exists() {
        walk_dirs.push(src_dir);
    }
    let include_dir = project_dir.join("include");
    if include_dir.exists() {
        walk_dirs.push(include_dir);
    }

    // Walk source files
    for dir in &walk_dirs {
        for entry in walkdir::WalkDir::new(dir)
            .into_iter()
            .filter_map(|e| e.ok())
        {
            let path = entry.path();
            if !path.is_file() {
                continue;
            }
            if let Some(ext) = path.extension().and_then(|e| e.to_str()) {
                let lang_filter = config.lang.as_deref();
                match ext {
                    "rs" if lang_filter.is_none() || lang_filter == Some("rust") => {
                        match super::doc_extract_rust::extract_rust_file(path, config.all) {
                            Ok(result) => {
                                modules.push(result.module);
                                entry_points.extend(result.entry_points);
                                todos.extend(result.todos);
                            }
                            Err(e) => {
                                log::warn!("Failed to extract {}: {}", path.display(), e);
                            }
                        }
                    }
                    "h" | "hpp" | "hxx" if lang_filter.is_none() || lang_filter == Some("cpp") => {
                        match super::doc_extract_cpp::extract_cpp_file(path, config.all) {
                            Ok(result) => {
                                modules.push(result.module);
                                todos.extend(result.todos);
                            }
                            Err(e) => {
                                log::warn!("Failed to extract C++ {}: {}", path.display(), e);
                            }
                        }
                    }
                    "py" if lang_filter.is_none() || lang_filter == Some("python") => {
                        match super::doc_extract_python::extract_python_file(path) {
                            Ok(result) => {
                                modules.push(result.module);
                                todos.extend(result.todos);
                            }
                            Err(e) => {
                                log::warn!("Failed to extract Python {}: {}", path.display(), e);
                            }
                        }
                    }
                    _ => {}
                }
            }
        }
    }

    let languages: Vec<String> = ctx
        .languages
        .iter()
        .map(|l| format!("{:?}", l).to_lowercase())
        .collect();

    let stats = compute_stats(&modules, &entry_points, &todos);

    let mut doc = ProjectDoc {
        project: project_name,
        version,
        languages,
        module_tree: ModuleTree {
            name: "root".to_string(),
            children: vec![],
        },
        modules,
        relationships: vec![],
        message_graph: None,
        entry_points,
        todos,
        stats,
    };

    // Cross-reference pass: match trait impls, build message graph
    cross_reference_pass(&mut doc);

    Ok(doc)
}

// ─── Watch Mode ─────────────────────────────────────────────────────────────

/// Run extraction in watch mode — regenerate on file changes.
fn run_extract_watch(config: ExtractConfig) -> Result<()> {
    use notify::{RecursiveMode, Watcher};
    use std::sync::mpsc;
    use std::time::{Duration, Instant};

    let project_dir = std::env::current_dir()?;

    // Initial extraction
    let doc = extract_project(&project_dir, &config)?;
    let output = format_output(&doc, &config);
    write_output(&output, &config)?;

    eprintln!("Watching for changes... (Ctrl+C to stop)");

    let (tx, rx) = mpsc::channel();
    let mut watcher = notify::recommended_watcher(move |res| {
        if let Ok(event) = res {
            let _ = tx.send(event);
        }
    })?;

    // Watch src/ and include/
    for dir_name in &["src", "include"] {
        let dir = project_dir.join(dir_name);
        if dir.exists() {
            watcher.watch(&dir, RecursiveMode::Recursive)?;
        }
    }

    let debounce = Duration::from_millis(200);
    let mut last_rebuild = Instant::now();

    loop {
        match rx.recv() {
            Ok(event) => {
                if !is_source_file_event(&event) {
                    continue;
                }

                // Debounce
                if last_rebuild.elapsed() < debounce {
                    std::thread::sleep(debounce.saturating_sub(last_rebuild.elapsed()));
                }
                // Drain queued events
                while rx.try_recv().is_ok() {}

                eprintln!("Changes detected, regenerating...");
                match extract_project(&project_dir, &config) {
                    Ok(doc) => {
                        let output = format_output(&doc, &config);
                        if let Err(e) = write_output_atomic(&output, &config) {
                            eprintln!("Write error: {e}");
                        }
                    }
                    Err(e) => eprintln!("Extraction error: {e}"),
                }
                last_rebuild = Instant::now();
            }
            Err(_) => break, // Channel closed (Ctrl+C)
        }
    }

    Ok(())
}

fn is_source_file_event(event: &notify::Event) -> bool {
    event.paths.iter().any(|p| {
        matches!(
            p.extension().and_then(|e| e.to_str()),
            Some("rs" | "py" | "h" | "hpp" | "cpp" | "cc" | "cxx" | "toml")
        )
    })
}

fn write_output_atomic(output: &str, config: &ExtractConfig) -> Result<()> {
    if let Some(ref path) = config.output {
        let tmp = path.with_extension("tmp");
        std::fs::write(&tmp, output)?;
        std::fs::rename(&tmp, path)?;
    } else {
        // Clear terminal and reprint
        print!("\x1b[2J\x1b[H{output}");
    }
    Ok(())
}

// ─── Cross-Reference Pass ───────────────────────────────────────────────────

/// Post-extraction cross-reference pass. Matches trait impls to types,
/// builds the message flow graph, and populates the relationships vec.
pub fn cross_reference_pass(doc: &mut ProjectDoc) {
    let mut relationships = Vec::new();

    // 1. Collect trait impls from structs → Relationship::Implements
    for module in &doc.modules {
        for sym in &module.symbols {
            if let SymbolDoc::Struct(s) = sym {
                for trait_name in &s.trait_impls {
                    relationships.push(Relationship::Implements {
                        implementor: s.name.clone(),
                        trait_name: trait_name.clone(),
                        file: module.path.clone(),
                    });
                }
            }
        }
    }

    // 2. Populate TraitDoc.implementors from the relationships
    let impl_map: std::collections::HashMap<String, Vec<String>> = {
        let mut map = std::collections::HashMap::new();
        for rel in &relationships {
            if let Relationship::Implements {
                implementor,
                trait_name,
                ..
            } = rel
            {
                map.entry(trait_name.clone())
                    .or_insert_with(Vec::new)
                    .push(implementor.clone());
            }
        }
        map
    };

    for module in &mut doc.modules {
        for sym in &mut module.symbols {
            if let SymbolDoc::Trait(t) = sym {
                if let Some(impls) = impl_map.get(&t.name) {
                    for imp in impls {
                        if !t.implementors.contains(imp) {
                            t.implementors.push(imp.clone());
                        }
                    }
                }
            }
        }
    }

    // 3. Build message graph from entry point topic info
    let graph = build_message_graph(&doc.entry_points);

    // 4. Add topic relationships
    for ep in &doc.entry_points {
        if let Some(ref details) = ep.details {
            for topic in &details.publishes {
                relationships.push(Relationship::Publishes {
                    node: ep.name.clone(),
                    message_type: topic.message_type.clone(),
                    topic: topic.name.clone(),
                });
            }
            for topic in &details.subscribes {
                relationships.push(Relationship::Subscribes {
                    node: ep.name.clone(),
                    message_type: topic.message_type.clone(),
                    topic: topic.name.clone(),
                });
            }
        }
    }

    doc.relationships = relationships;
    doc.message_graph = if graph.topics.is_empty() {
        None
    } else {
        Some(graph)
    };
}

/// Build message flow graph from entry point topic info.
fn build_message_graph(entry_points: &[EntryPoint]) -> MessageGraph {
    let mut nodes = Vec::new();
    let mut topic_map: std::collections::HashMap<String, (String, Vec<String>, Vec<String>)> =
        std::collections::HashMap::new();

    for ep in entry_points {
        if !nodes.contains(&ep.name) {
            nodes.push(ep.name.clone());
        }
        if let Some(ref details) = ep.details {
            for topic in &details.publishes {
                let entry = topic_map
                    .entry(topic.name.clone())
                    .or_insert_with(|| (topic.message_type.clone(), vec![], vec![]));
                if !entry.1.contains(&ep.name) {
                    entry.1.push(ep.name.clone());
                }
            }
            for topic in &details.subscribes {
                let entry = topic_map
                    .entry(topic.name.clone())
                    .or_insert_with(|| (topic.message_type.clone(), vec![], vec![]));
                if !entry.2.contains(&ep.name) {
                    entry.2.push(ep.name.clone());
                }
            }
        }
    }

    let topics = topic_map
        .into_iter()
        .map(|(name, (msg_type, pubs, subs))| MessageGraphTopic {
            name,
            message_type: msg_type,
            publishers: pubs,
            subscribers: subs,
        })
        .collect();

    MessageGraph { nodes, topics }
}

/// Select the appropriate formatter based on config.
pub fn format_output(doc: &ProjectDoc, config: &ExtractConfig) -> String {
    if config.json {
        format_json(doc)
    } else if config.html {
        crate::commands::doc_extract_html::format_html(doc)
    } else if config.md {
        format_markdown(doc)
    } else if config.coverage {
        format_coverage(doc)
    } else {
        format_brief(doc)
    }
}

/// Write output to file or stdout.
pub fn write_output(output: &str, config: &ExtractConfig) -> Result<()> {
    if let Some(ref path) = config.output {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)
                .with_context(|| format!("Failed to create directory {}", parent.display()))?;
        }
        std::fs::write(path, output)
            .with_context(|| format!("Failed to write to {}", path.display()))?;
        eprintln!("Wrote documentation to {}", path.display());
    } else {
        print!("{output}");
    }
    Ok(())
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_project_doc() -> ProjectDoc {
        ProjectDoc {
            project: "test-project".to_string(),
            version: "0.1.0".to_string(),
            languages: vec!["rust".to_string()],
            module_tree: ModuleTree {
                name: "root".to_string(),
                children: vec![],
            },
            modules: vec![ModuleDoc {
                path: PathBuf::from("src/lib.rs"),
                language: "rust".to_string(),
                module_doc: Some("Test module.".to_string()),
                imports: vec![],
                symbols: vec![
                    SymbolDoc::Function(FunctionDoc {
                        name: "hello".to_string(),
                        visibility: Visibility::Public,
                        location: SourceLocation {
                            file: PathBuf::from("src/lib.rs"),
                            line: 5,
                            end_line: Some(10),
                        },
                        signature: "pub fn hello(name: &str) -> String".to_string(),
                        doc: Some("Greet someone.".to_string()),
                        deprecated: None,
                        params: vec![ParamDoc {
                            name: "name".to_string(),
                            type_str: Some("&str".to_string()),
                            default_value: None,
                            doc: None,
                        }],
                        returns: Some("String".to_string()),
                        is_async: false,
                        generic_params: vec![],
                        examples: vec![],
                    }),
                    SymbolDoc::Struct(StructDoc {
                        name: "Config".to_string(),
                        visibility: Visibility::Public,
                        location: SourceLocation {
                            file: PathBuf::from("src/lib.rs"),
                            line: 15,
                            end_line: None,
                        },
                        doc: None,
                        deprecated: Some("use ConfigV2 instead".to_string()),
                        generic_params: vec![],
                        fields: vec![FieldDoc {
                            name: "verbose".to_string(),
                            type_str: Some("bool".to_string()),
                            doc: None,
                        }],
                        methods: vec![],
                        trait_impls: vec!["Debug".to_string()],
                        derives: vec!["Debug".to_string(), "Clone".to_string()],
                        examples: vec![],
                    }),
                ],
            }],
            relationships: vec![],
            message_graph: None,
            entry_points: vec![],
            todos: vec![TodoItem {
                kind: TodoKind::Todo,
                text: "add error handling".to_string(),
                location: SourceLocation {
                    file: PathBuf::from("src/lib.rs"),
                    line: 42,
                    end_line: None,
                },
            }],
            stats: DocStats {
                total_files: 1,
                total_symbols: 2,
                documented_symbols: 1,
                deprecated_symbols: 1,
                documentation_coverage: 0.5,
                horus_nodes: 0,
                horus_messages: 0,
                horus_services: 0,
                horus_actions: 0,
                topics_discovered: 0,
                todos: 1,
                fixmes: 0,
            },
        }
    }

    #[test]
    fn test_project_doc_json_roundtrip() {
        let doc = sample_project_doc();
        let json = format_json(&doc);
        let parsed: ProjectDoc = serde_json::from_str(&json).expect("should parse back");
        assert_eq!(parsed.project, doc.project);
        assert_eq!(parsed.version, doc.version);
        assert_eq!(parsed.modules.len(), doc.modules.len());
        assert_eq!(parsed.stats.total_symbols, doc.stats.total_symbols);
    }

    #[test]
    fn test_symbol_doc_tagged_serialization() {
        let sym = SymbolDoc::Function(FunctionDoc {
            name: "test_fn".to_string(),
            visibility: Visibility::Public,
            location: SourceLocation {
                file: PathBuf::from("test.rs"),
                line: 1,
                end_line: None,
            },
            signature: "pub fn test_fn()".to_string(),
            doc: None,
            deprecated: None,
            params: vec![],
            returns: None,
            is_async: false,
            generic_params: vec![],
            examples: vec![],
        });
        let json = serde_json::to_string(&sym).unwrap();
        assert!(
            json.contains("\"kind\":\"function\""),
            "should have tagged kind: {json}"
        );
    }

    #[test]
    fn test_source_location_serde() {
        let loc = SourceLocation {
            file: PathBuf::from("src/main.rs"),
            line: 42,
            end_line: Some(50),
        };
        let json = serde_json::to_string(&loc).unwrap();
        let parsed: SourceLocation = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.file, loc.file);
        assert_eq!(parsed.line, 42);
        assert_eq!(parsed.end_line, Some(50));
    }

    #[test]
    fn test_empty_project_doc() {
        let doc = ProjectDoc {
            project: "empty".to_string(),
            version: "0.0.0".to_string(),
            languages: vec![],
            module_tree: ModuleTree {
                name: "root".to_string(),
                children: vec![],
            },
            modules: vec![],
            relationships: vec![],
            message_graph: None,
            entry_points: vec![],
            todos: vec![],
            stats: DocStats {
                total_files: 0,
                total_symbols: 0,
                documented_symbols: 0,
                deprecated_symbols: 0,
                documentation_coverage: 1.0,
                horus_nodes: 0,
                horus_messages: 0,
                horus_services: 0,
                horus_actions: 0,
                topics_discovered: 0,
                todos: 0,
                fixmes: 0,
            },
        };
        let json = serde_json::to_string_pretty(&doc).unwrap();
        assert!(json.contains("\"empty\""));
        // Roundtrip
        let parsed: ProjectDoc = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.project, "empty");
    }

    #[test]
    fn test_format_brief_header() {
        let doc = sample_project_doc();
        let brief = format_brief(&doc);
        assert!(brief.contains("test-project v0.1.0"));
        assert!(brief.contains("2 symbols"));
        assert!(brief.contains("50%"));
    }

    #[test]
    fn test_format_brief_function() {
        let doc = sample_project_doc();
        let brief = format_brief(&doc);
        assert!(brief.contains("pub fn hello(name: &str) -> String"));
    }

    #[test]
    fn test_format_brief_deprecated_marker() {
        let doc = sample_project_doc();
        let brief = format_brief(&doc);
        assert!(
            brief.contains("⚠ struct Config"),
            "deprecated should have ⚠ marker: {brief}"
        );
    }

    #[test]
    fn test_format_coverage_percentages() {
        let doc = sample_project_doc();
        let cov = format_coverage(&doc);
        assert!(cov.contains("1/2 symbols documented"));
        assert!(cov.contains("50%"));
    }

    #[test]
    fn test_format_coverage_undocumented_list() {
        let doc = sample_project_doc();
        let cov = format_coverage(&doc);
        assert!(
            cov.contains("Config"),
            "undocumented Config should be listed"
        );
    }

    #[test]
    fn test_format_json_valid() {
        let doc = sample_project_doc();
        let json = format_json(&doc);
        let parsed: serde_json::Value =
            serde_json::from_str(&json).expect("format_json should produce valid JSON");
        assert!(parsed.is_object());
    }

    #[test]
    fn test_compute_stats() {
        let modules = vec![ModuleDoc {
            path: PathBuf::from("src/lib.rs"),
            language: "rust".to_string(),
            module_doc: None,
            imports: vec![],
            symbols: vec![
                SymbolDoc::Function(FunctionDoc {
                    name: "f1".to_string(),
                    visibility: Visibility::Public,
                    location: SourceLocation {
                        file: PathBuf::from("src/lib.rs"),
                        line: 1,
                        end_line: None,
                    },
                    signature: "pub fn f1()".to_string(),
                    doc: Some("Documented.".to_string()),
                    deprecated: None,
                    params: vec![],
                    returns: None,
                    is_async: false,
                    generic_params: vec![],
                    examples: vec![],
                }),
                SymbolDoc::Function(FunctionDoc {
                    name: "f2".to_string(),
                    visibility: Visibility::Public,
                    location: SourceLocation {
                        file: PathBuf::from("src/lib.rs"),
                        line: 5,
                        end_line: None,
                    },
                    signature: "pub fn f2()".to_string(),
                    doc: None,
                    deprecated: Some("removed".to_string()),
                    params: vec![],
                    returns: None,
                    is_async: false,
                    generic_params: vec![],
                    examples: vec![],
                }),
            ],
        }];
        let stats = compute_stats(&modules, &[], &[]);
        assert_eq!(stats.total_files, 1);
        assert_eq!(stats.total_symbols, 2);
        assert_eq!(stats.documented_symbols, 1);
        assert_eq!(stats.deprecated_symbols, 1);
        assert!((stats.documentation_coverage - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_symbol_doc_name_accessor() {
        let sym = SymbolDoc::Struct(StructDoc {
            name: "MyStruct".to_string(),
            visibility: Visibility::Public,
            location: SourceLocation {
                file: PathBuf::from("test.rs"),
                line: 1,
                end_line: None,
            },
            doc: None,
            deprecated: None,
            generic_params: vec![],
            fields: vec![],
            methods: vec![],
            trait_impls: vec![],
            derives: vec![],
            examples: vec![],
        });
        assert_eq!(sym.name(), "MyStruct");
    }

    #[test]
    fn test_todo_kind_serde() {
        let todo = TodoItem {
            kind: TodoKind::Fixme,
            text: "broken".to_string(),
            location: SourceLocation {
                file: PathBuf::from("test.rs"),
                line: 1,
                end_line: None,
            },
        };
        let json = serde_json::to_string(&todo).unwrap();
        assert!(json.contains("\"fixme\""));
        let parsed: TodoItem = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.kind, TodoKind::Fixme);
    }

    #[test]
    fn test_relationship_tagged_serde() {
        let rel = Relationship::Implements {
            implementor: "MyNode".to_string(),
            trait_name: "Node".to_string(),
            file: PathBuf::from("src/node.rs"),
        };
        let json = serde_json::to_string(&rel).unwrap();
        assert!(json.contains("\"kind\":\"implements\""));
        let parsed: Relationship = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed, rel);
    }

    #[test]
    fn test_format_output_selects_json() {
        let doc = sample_project_doc();
        let config = ExtractConfig {
            json: true,
            md: false,
            html: false,
            brief: false,
            full: false,
            all: false,
            lang: None,
            coverage: false,
            output: None,
            watch: false,
            diff: None,
            fail_under: None,
        };
        let output = format_output(&doc, &config);
        // JSON output should be parseable
        let _: serde_json::Value = serde_json::from_str(&output).expect("should be valid JSON");
    }

    #[test]
    fn test_format_output_selects_coverage() {
        let doc = sample_project_doc();
        let config = ExtractConfig {
            json: false,
            md: false,
            html: false,
            brief: false,
            full: false,
            all: false,
            lang: None,
            coverage: true,
            output: None,
            watch: false,
            diff: None,
            fail_under: None,
        };
        let output = format_output(&doc, &config);
        assert!(output.contains("Coverage Report"));
    }

    // ── Cross-Reference Pass Tests ──────────────────────────────────────

    #[test]
    fn test_cross_ref_trait_impl() {
        let mut doc = ProjectDoc {
            project: "test".to_string(),
            version: "0.1.0".to_string(),
            languages: vec![],
            module_tree: ModuleTree {
                name: "root".to_string(),
                children: vec![],
            },
            modules: vec![ModuleDoc {
                path: PathBuf::from("src/lib.rs"),
                language: "rust".to_string(),
                module_doc: None,
                imports: vec![],
                symbols: vec![
                    SymbolDoc::Struct(StructDoc {
                        name: "MyNode".to_string(),
                        visibility: Visibility::Public,
                        location: SourceLocation {
                            file: PathBuf::from("src/lib.rs"),
                            line: 1,
                            end_line: None,
                        },
                        doc: None,
                        deprecated: None,
                        generic_params: vec![],
                        fields: vec![],
                        methods: vec![],
                        trait_impls: vec!["Node".to_string()],
                        derives: vec![],
                        examples: vec![],
                    }),
                    SymbolDoc::Trait(TraitDoc {
                        name: "Node".to_string(),
                        visibility: Visibility::Public,
                        location: SourceLocation {
                            file: PathBuf::from("src/lib.rs"),
                            line: 10,
                            end_line: None,
                        },
                        doc: None,
                        deprecated: None,
                        generic_params: vec![],
                        required_methods: vec![],
                        provided_methods: vec![],
                        implementors: vec![],
                    }),
                ],
            }],
            relationships: vec![],
            message_graph: None,
            entry_points: vec![],
            todos: vec![],
            stats: DocStats {
                total_files: 1,
                total_symbols: 2,
                documented_symbols: 0,
                deprecated_symbols: 0,
                documentation_coverage: 0.0,
                horus_nodes: 0,
                horus_messages: 0,
                horus_services: 0,
                horus_actions: 0,
                topics_discovered: 0,
                todos: 0,
                fixmes: 0,
            },
        };

        cross_reference_pass(&mut doc);

        // Relationship should be created
        assert!(!doc.relationships.is_empty());
        assert!(
            matches!(&doc.relationships[0], Relationship::Implements { implementor, trait_name, .. } if implementor == "MyNode" && trait_name == "Node")
        );

        // TraitDoc.implementors should be populated
        if let SymbolDoc::Trait(t) = &doc.modules[0].symbols[1] {
            assert!(t.implementors.contains(&"MyNode".to_string()));
        } else {
            panic!("expected Trait");
        }
    }

    #[test]
    fn test_message_graph_basic() {
        let entry_points = vec![
            EntryPoint {
                kind: EntryPointKind::HorusNode,
                name: "SensorNode".to_string(),
                file: PathBuf::from("src/sensor.rs"),
                details: Some(EntryPointDetails {
                    publishes: vec![TopicInfo {
                        name: "temperature".to_string(),
                        message_type: "TempReading".to_string(),
                        direction: "pub".to_string(),
                    }],
                    subscribes: vec![],
                    tick_rate: None,
                    execution_class: None,
                }),
            },
            EntryPoint {
                kind: EntryPointKind::HorusNode,
                name: "MonitorNode".to_string(),
                file: PathBuf::from("src/monitor.rs"),
                details: Some(EntryPointDetails {
                    publishes: vec![],
                    subscribes: vec![TopicInfo {
                        name: "temperature".to_string(),
                        message_type: "TempReading".to_string(),
                        direction: "sub".to_string(),
                    }],
                    tick_rate: None,
                    execution_class: None,
                }),
            },
        ];

        let graph = build_message_graph(&entry_points);
        assert_eq!(graph.nodes.len(), 2);
        assert_eq!(graph.topics.len(), 1);
        assert_eq!(graph.topics[0].name, "temperature");
        assert_eq!(graph.topics[0].publishers, vec!["SensorNode"]);
        assert_eq!(graph.topics[0].subscribers, vec!["MonitorNode"]);
    }

    #[test]
    fn test_message_graph_empty() {
        let graph = build_message_graph(&[]);
        assert!(graph.nodes.is_empty());
        assert!(graph.topics.is_empty());
    }

    // ── format_markdown tests ───────────────────────────────────────

    #[test]
    fn test_format_markdown_contains_project_header() {
        let doc = sample_project_doc();
        let md = format_markdown(&doc);
        assert!(
            md.contains("# test-project v0.1.0"),
            "markdown should have project header"
        );
        assert!(
            md.contains("API Reference"),
            "markdown should have API Reference label"
        );
    }

    #[test]
    fn test_format_markdown_includes_doc_comments() {
        let doc = sample_project_doc();
        let md = format_markdown(&doc);
        assert!(
            md.contains("Greet someone."),
            "markdown should include doc comments from symbols"
        );
    }

    #[test]
    fn test_format_markdown_shows_deprecated() {
        let doc = sample_project_doc();
        let md = format_markdown(&doc);
        assert!(
            md.contains("Deprecated"),
            "markdown should flag deprecated symbols"
        );
        assert!(
            md.contains("use ConfigV2 instead"),
            "markdown should show deprecation message"
        );
    }

    // ── format_output selection tests ───────────────────────────────

    #[test]
    fn test_format_output_selects_html() {
        let doc = sample_project_doc();
        let config = ExtractConfig {
            json: false,
            md: false,
            html: true,
            brief: false,
            full: false,
            all: false,
            lang: None,
            coverage: false,
            output: None,
            watch: false,
            diff: None,
            fail_under: None,
        };
        let output = format_output(&doc, &config);
        assert!(
            output.contains("<!DOCTYPE html>"),
            "html flag should produce HTML output"
        );
    }

    #[test]
    fn test_format_output_selects_md() {
        let doc = sample_project_doc();
        let config = ExtractConfig {
            json: false,
            md: true,
            html: false,
            brief: false,
            full: false,
            all: false,
            lang: None,
            coverage: false,
            output: None,
            watch: false,
            diff: None,
            fail_under: None,
        };
        let output = format_output(&doc, &config);
        assert!(
            output.contains("API Reference"),
            "md flag should produce markdown output"
        );
    }

    #[test]
    fn test_format_output_default_is_brief() {
        let doc = sample_project_doc();
        let config = ExtractConfig {
            json: false,
            md: false,
            html: false,
            brief: false,
            full: false,
            all: false,
            lang: None,
            coverage: false,
            output: None,
            watch: false,
            diff: None,
            fail_under: None,
        };
        let output = format_output(&doc, &config);
        assert!(
            output.starts_with("# test-project"),
            "default should be brief format"
        );
    }

    // ── write_output tests ──────────────────────────────────────────

    #[test]
    fn test_write_output_to_file() {
        let dir = tempfile::tempdir().unwrap();
        let out_path = dir.path().join("api.json");
        let config = ExtractConfig {
            json: true,
            md: false,
            html: false,
            brief: false,
            full: false,
            all: false,
            lang: None,
            coverage: false,
            output: Some(out_path.clone()),
            watch: false,
            diff: None,
            fail_under: None,
        };
        let doc = sample_project_doc();
        let output_str = format_output(&doc, &config);
        write_output(&output_str, &config).unwrap();

        assert!(out_path.exists(), "output file should be created");
        let content = std::fs::read_to_string(&out_path).unwrap();
        assert!(
            content.contains("test-project"),
            "file should contain project data"
        );
        // Verify it's valid JSON
        let _: serde_json::Value = serde_json::from_str(&content).expect("should be valid JSON");
    }

    #[test]
    fn test_write_output_creates_parent_dirs() {
        let dir = tempfile::tempdir().unwrap();
        let out_path = dir.path().join("nested/dir/api.json");
        let config = ExtractConfig {
            json: true,
            md: false,
            html: false,
            brief: false,
            full: false,
            all: false,
            lang: None,
            coverage: false,
            output: Some(out_path.clone()),
            watch: false,
            diff: None,
            fail_under: None,
        };
        write_output("{}", &config).unwrap();
        assert!(out_path.exists(), "should create nested directories");
    }

    // ── is_source_file_event tests ──────────────────────────────────

    #[test]
    fn test_is_source_file_event_rs() {
        let event = notify::Event {
            kind: notify::EventKind::Modify(notify::event::ModifyKind::Data(
                notify::event::DataChange::Content,
            )),
            paths: vec![std::path::PathBuf::from("src/main.rs")],
            attrs: Default::default(),
        };
        assert!(is_source_file_event(&event));
    }

    #[test]
    fn test_is_source_file_event_py() {
        let event = notify::Event {
            kind: notify::EventKind::Modify(notify::event::ModifyKind::Data(
                notify::event::DataChange::Content,
            )),
            paths: vec![std::path::PathBuf::from("src/main.py")],
            attrs: Default::default(),
        };
        assert!(is_source_file_event(&event));
    }

    #[test]
    fn test_is_source_file_event_hpp() {
        let event = notify::Event {
            kind: notify::EventKind::Modify(notify::event::ModifyKind::Data(
                notify::event::DataChange::Content,
            )),
            paths: vec![std::path::PathBuf::from("include/robot.hpp")],
            attrs: Default::default(),
        };
        assert!(is_source_file_event(&event));
    }

    #[test]
    fn test_is_source_file_event_non_source() {
        let event = notify::Event {
            kind: notify::EventKind::Modify(notify::event::ModifyKind::Data(
                notify::event::DataChange::Content,
            )),
            paths: vec![std::path::PathBuf::from(".gitignore")],
            attrs: Default::default(),
        };
        assert!(
            !is_source_file_event(&event),
            ".gitignore should not be a source file"
        );
    }

    #[test]
    fn test_is_source_file_event_toml() {
        let event = notify::Event {
            kind: notify::EventKind::Modify(notify::event::ModifyKind::Data(
                notify::event::DataChange::Content,
            )),
            paths: vec![std::path::PathBuf::from("horus.toml")],
            attrs: Default::default(),
        };
        assert!(
            is_source_file_event(&event),
            "toml files should trigger rebuild"
        );
    }

    // ── write_output_atomic tests ───────────────────────────────────

    #[test]
    fn test_write_output_atomic_to_file() {
        let dir = tempfile::tempdir().unwrap();
        let out_path = dir.path().join("output.txt");
        let config = ExtractConfig {
            json: false,
            md: false,
            html: false,
            brief: false,
            full: false,
            all: false,
            lang: None,
            coverage: false,
            output: Some(out_path.clone()),
            watch: false,
            diff: None,
            fail_under: None,
        };
        write_output_atomic("test content", &config).unwrap();
        assert!(out_path.exists());
        assert_eq!(std::fs::read_to_string(&out_path).unwrap(), "test content");
        // .tmp should not remain
        assert!(
            !dir.path().join("output.tmp").exists(),
            "tmp file should be cleaned up"
        );
    }

    // ── compute_stats edge cases ────────────────────────────────────

    #[test]
    fn test_compute_stats_empty() {
        let stats = compute_stats(&[], &[], &[]);
        assert_eq!(stats.total_files, 0);
        assert_eq!(stats.total_symbols, 0);
        assert_eq!(
            stats.documentation_coverage, 1.0,
            "empty project = 100% coverage"
        );
    }

    #[test]
    fn test_compute_stats_counts_horus_types() {
        let modules = vec![ModuleDoc {
            path: PathBuf::from("src/lib.rs"),
            language: "rust".to_string(),
            module_doc: None,
            imports: vec![],
            symbols: vec![
                SymbolDoc::HorusMessage(HorusMessageDoc {
                    name: "Msg".to_string(),
                    location: SourceLocation {
                        file: PathBuf::from("src/lib.rs"),
                        line: 1,
                        end_line: None,
                    },
                    doc: None,
                    deprecated: None,
                    fields: vec![],
                }),
                SymbolDoc::HorusService(HorusServiceDoc {
                    name: "Svc".to_string(),
                    location: SourceLocation {
                        file: PathBuf::from("src/lib.rs"),
                        line: 5,
                        end_line: None,
                    },
                    doc: None,
                    request_fields: vec![],
                    response_fields: vec![],
                }),
                SymbolDoc::HorusAction(HorusActionDoc {
                    name: "Act".to_string(),
                    location: SourceLocation {
                        file: PathBuf::from("src/lib.rs"),
                        line: 10,
                        end_line: None,
                    },
                    doc: None,
                    goal_fields: vec![],
                    feedback_fields: vec![],
                    result_fields: vec![],
                }),
            ],
        }];
        let stats = compute_stats(&modules, &[], &[]);
        assert_eq!(stats.horus_messages, 1);
        assert_eq!(stats.horus_services, 1);
        assert_eq!(stats.horus_actions, 1);
    }

    // ── Level 2: Integration — extract → format pipeline ────────────

    #[test]
    fn test_integration_extract_rust_then_format_json_roundtrip() {
        // Integration test: extract a real Rust file, format as JSON, parse back
        let dir = tempfile::tempdir().unwrap();
        let src_dir = dir.path().join("src");
        std::fs::create_dir_all(&src_dir).unwrap();
        std::fs::write(
            src_dir.join("lib.rs"),
            "/// A helper.\npub fn helper() -> u32 { 42 }",
        )
        .unwrap();

        let config = ExtractConfig {
            json: false,
            md: false,
            html: false,
            brief: false,
            full: false,
            all: false,
            lang: None,
            coverage: false,
            output: None,
            watch: false,
            diff: None,
            fail_under: None,
        };
        let doc = extract_project(dir.path(), &config).unwrap();

        // Verify extraction found the function
        assert!(
            !doc.modules.is_empty(),
            "should extract at least one module"
        );
        let total_symbols: usize = doc.modules.iter().map(|m| m.symbols.len()).sum();
        assert!(total_symbols > 0, "should extract at least one symbol");

        // Format as JSON and roundtrip
        let json = format_json(&doc);
        let parsed: ProjectDoc = serde_json::from_str(&json).expect("JSON should roundtrip");
        assert_eq!(parsed.modules.len(), doc.modules.len());
    }

    #[test]
    fn test_integration_extract_then_format_all_formats() {
        // Integration: extract project, verify all formatters produce non-empty output
        let dir = tempfile::tempdir().unwrap();
        let src_dir = dir.path().join("src");
        std::fs::create_dir_all(&src_dir).unwrap();
        std::fs::write(src_dir.join("lib.rs"), "pub struct Foo;").unwrap();

        let config = ExtractConfig {
            json: false,
            md: false,
            html: false,
            brief: false,
            full: false,
            all: false,
            lang: None,
            coverage: false,
            output: None,
            watch: false,
            diff: None,
            fail_under: None,
        };
        let doc = extract_project(dir.path(), &config).unwrap();

        assert!(!format_json(&doc).is_empty());
        assert!(!format_brief(&doc).is_empty());
        assert!(!format_markdown(&doc).is_empty());
        assert!(!format_coverage(&doc).is_empty());
        assert!(!crate::commands::doc_extract_html::format_html(&doc).is_empty());
    }

    #[test]
    fn test_integration_extract_then_diff_against_self() {
        // Integration: extract project, diff against itself = no changes
        let dir = tempfile::tempdir().unwrap();
        let src_dir = dir.path().join("src");
        std::fs::create_dir_all(&src_dir).unwrap();
        std::fs::write(src_dir.join("lib.rs"), "pub fn stable() {}").unwrap();

        let config = ExtractConfig {
            json: false,
            md: false,
            html: false,
            brief: false,
            full: false,
            all: false,
            lang: None,
            coverage: false,
            output: None,
            watch: false,
            diff: None,
            fail_under: None,
        };
        let doc = extract_project(dir.path(), &config).unwrap();
        let diff = crate::commands::doc_extract_diff::compute_diff(&doc, &doc);
        assert_eq!(
            diff.breaking_changes, 0,
            "self-diff should have no breaking changes"
        );
        assert!(diff.added.is_empty());
        assert!(diff.removed.is_empty());
        assert!(diff.changed.is_empty());
    }

    // ── Level 5: Error path tests ───────────────────────────────────

    #[test]
    fn test_extract_project_nonexistent_dir() {
        let config = ExtractConfig {
            json: false,
            md: false,
            html: false,
            brief: false,
            full: false,
            all: false,
            lang: None,
            coverage: false,
            output: None,
            watch: false,
            diff: None,
            fail_under: None,
        };
        // Should succeed with empty result (no src/ dir)
        let dir = tempfile::tempdir().unwrap();
        let doc = extract_project(dir.path(), &config).unwrap();
        assert!(doc.modules.is_empty());
        assert_eq!(doc.stats.total_symbols, 0);
    }

    #[test]
    fn test_write_output_invalid_path() {
        let config = ExtractConfig {
            json: false,
            md: false,
            html: false,
            brief: false,
            full: false,
            all: false,
            lang: None,
            coverage: false,
            output: Some(PathBuf::from(
                "/nonexistent/deeply/nested/impossible/path/file.json",
            )),
            watch: false,
            diff: None,
            fail_under: None,
        };
        let result = write_output("test", &config);
        assert!(result.is_err(), "writing to impossible path should fail");
    }
}
