//! Self-contained HTML report generator for doc extraction.
//!
//! Produces a single `.html` file with embedded CSS and JavaScript.
//! No external dependencies — works offline, safe to email or upload as CI artifact.

use super::doc_extract::*;
use std::fmt::Write;

/// Generate a self-contained HTML report from ProjectDoc.
pub fn format_html(doc: &ProjectDoc) -> String {
    let mut html = String::with_capacity(64 * 1024);

    write_head(&mut html, doc);
    write_header(&mut html, doc);
    write_search_bar(&mut html);

    // Topic graph
    if let Some(ref graph) = doc.message_graph {
        if !graph.topics.is_empty() {
            write_topic_graph_svg(&mut html, graph);
        }
    }

    // Messages section
    write_messages_section(&mut html, doc);

    // Nodes section
    write_nodes_section(&mut html, doc);

    // Full API reference
    write_api_reference(&mut html, doc);

    // Coverage table
    write_coverage_table(&mut html, doc);

    // TODOs section
    write_todos_section(&mut html, doc);

    // Search JS
    html.push_str(SEARCH_JS);
    html.push_str("\n</body>\n</html>\n");

    html
}

// ─── HTML Sections ──────────────────────────────────────────────────────────

fn write_head(html: &mut String, doc: &ProjectDoc) {
    let _ = write!(
        html,
        r#"<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{} v{} — API Documentation</title>
<style>
{CSS}
</style>
</head>
<body>
"#,
        escape_html(&doc.project),
        escape_html(&doc.version)
    );
}

fn write_header(html: &mut String, doc: &ProjectDoc) {
    let langs: Vec<String> = doc
        .languages
        .iter()
        .map(|l| format!(r#"<span class="badge">{}</span>"#, escape_html(l)))
        .collect();

    let coverage_class = if doc.stats.documentation_coverage >= 0.8 {
        "badge badge-green"
    } else if doc.stats.documentation_coverage >= 0.5 {
        "badge badge-yellow"
    } else {
        "badge badge-red"
    };

    let _ = write!(
        html,
        r#"<header>
<h1>{} <small>v{}</small></h1>
<div class="badges">
{} <span class="{}">{:.0}% documented</span>
<span class="badge">{} symbols</span>
"#,
        escape_html(&doc.project),
        escape_html(&doc.version),
        langs.join(" "),
        coverage_class,
        doc.stats.documentation_coverage * 100.0,
        doc.stats.total_symbols
    );

    if doc.stats.horus_nodes > 0 {
        let _ = write!(
            html,
            r#"<span class="badge">{} nodes</span>"#,
            doc.stats.horus_nodes
        );
    }
    if doc.stats.horus_messages > 0 {
        let _ = write!(
            html,
            r#"<span class="badge">{} messages</span>"#,
            doc.stats.horus_messages
        );
    }
    if doc.stats.topics_discovered > 0 {
        let _ = write!(
            html,
            r#"<span class="badge">{} topics</span>"#,
            doc.stats.topics_discovered
        );
    }

    html.push_str("</div>\n</header>\n");
}

fn write_search_bar(html: &mut String) {
    html.push_str(
        r#"<div class="search-container">
<input type="text" id="search" placeholder="Search symbols..." autocomplete="off">
</div>
"#,
    );
}

fn write_topic_graph_svg(html: &mut String, graph: &MessageGraph) {
    html.push_str("<section>\n<h2>Topic Graph</h2>\n");

    // Simple SVG layout: nodes on top, topics as arrows below
    let node_count = graph.nodes.len();
    let width = (node_count.max(1) * 180).max(400);
    let height = 120 + graph.topics.len() * 40;

    let _ = write!(
        html,
        r#"<svg viewBox="0 0 {width} {height}" class="topic-graph">"#
    );

    // Draw nodes
    for (i, node) in graph.nodes.iter().enumerate() {
        let x = 20 + i * 170;
        let _ = write!(
            html,
            r#"<rect x="{x}" y="10" width="150" height="40" rx="8" class="graph-node"/>"#
        );
        let _ = write!(
            html,
            r#"<text x="{}" y="35" class="graph-label">{}</text>"#,
            x + 75,
            escape_html(node)
        );
    }

    // Draw topics as labeled arrows
    for (i, topic) in graph.topics.iter().enumerate() {
        let y = 80 + i * 40;
        let label = format!("{}: {}", topic.name, topic.message_type);
        let pubs = topic.publishers.join(", ");
        let subs = topic.subscribers.join(", ");
        let _ = write!(
            html,
            r#"<text x="20" y="{y}" class="graph-topic">{} -&gt; {}</text>"#,
            escape_html(&pubs),
            escape_html(&subs)
        );
        let _ = write!(
            html,
            r#"<text x="20" y="{}" class="graph-topic-label">{}</text>"#,
            y + 14,
            escape_html(&label)
        );
    }

    html.push_str("</svg>\n</section>\n");
}

fn write_messages_section(html: &mut String, doc: &ProjectDoc) {
    let messages: Vec<&HorusMessageDoc> = doc
        .modules
        .iter()
        .flat_map(|m| m.symbols.iter())
        .filter_map(|s| match s {
            SymbolDoc::HorusMessage(m) => Some(m),
            _ => None,
        })
        .collect();

    if messages.is_empty() {
        return;
    }

    html.push_str("<section>\n<h2>Message Types</h2>\n<div class=\"cards\">\n");
    for msg in &messages {
        let _ = write!(
            html,
            "<div class=\"card symbol\">\n<h3>{}</h3>\n",
            escape_html(&msg.name)
        );
        if let Some(ref d) = msg.doc {
            let _ = write!(html, "<p>{}</p>\n", escape_html(d));
        }
        html.push_str("<table>\n");
        for field in &msg.fields {
            let _ = write!(
                html,
                "<tr><td><code>{}</code></td><td><code>{}</code></td></tr>\n",
                escape_html(&field.name),
                escape_html(field.type_str.as_deref().unwrap_or("?"))
            );
        }
        html.push_str("</table>\n</div>\n");
    }
    html.push_str("</div>\n</section>\n");
}

fn write_nodes_section(html: &mut String, doc: &ProjectDoc) {
    let nodes: Vec<&EntryPoint> = doc
        .entry_points
        .iter()
        .filter(|e| e.kind == EntryPointKind::HorusNode)
        .collect();

    if nodes.is_empty() {
        return;
    }

    html.push_str("<section>\n<h2>Nodes</h2>\n<div class=\"cards\">\n");
    for node in &nodes {
        let _ = write!(
            html,
            "<div class=\"card symbol\">\n<h3>{}</h3>\n",
            escape_html(&node.name)
        );
        if let Some(ref details) = node.details {
            if let Some(ref rate) = details.tick_rate {
                let _ = write!(html, "<span class=\"badge\">{rate}</span>\n");
            }
            if !details.publishes.is_empty() {
                html.push_str("<div class=\"topics\">\n");
                for t in &details.publishes {
                    let _ = write!(
                        html,
                        "<div>pub -&gt; <code>{}: {}</code></div>\n",
                        escape_html(&t.name),
                        escape_html(&t.message_type)
                    );
                }
                html.push_str("</div>\n");
            }
            if !details.subscribes.is_empty() {
                html.push_str("<div class=\"topics\">\n");
                for t in &details.subscribes {
                    let _ = write!(
                        html,
                        "<div>sub &lt;- <code>{}: {}</code></div>\n",
                        escape_html(&t.name),
                        escape_html(&t.message_type)
                    );
                }
                html.push_str("</div>\n");
            }
        }
        html.push_str("</div>\n");
    }
    html.push_str("</div>\n</section>\n");
}

fn write_api_reference(html: &mut String, doc: &ProjectDoc) {
    html.push_str("<section>\n<h2>API Reference</h2>\n");

    for module in &doc.modules {
        let path_str = module.path.display().to_string();
        let _ = write!(
            html,
            "<details class=\"collapsible\">\n<summary>{}",
            escape_html(&path_str)
        );
        if let Some(ref mdoc) = module.module_doc {
            let _ = write!(html, " — <em>{}</em>", escape_html(mdoc));
        }
        let _ = write!(
            html,
            " <span class=\"count\">({})</span></summary>\n",
            module.symbols.len()
        );

        html.push_str("<div class=\"symbols\">\n");
        for sym in &module.symbols {
            let dep_class = if sym.deprecated().is_some() {
                " deprecated"
            } else {
                ""
            };
            let _ = write!(html, "<div class=\"symbol{dep_class}\">\n");

            match sym {
                SymbolDoc::Function(f) => {
                    let _ = write!(
                        html,
                        "<code class=\"sig\">{}</code>\n",
                        escape_html(&f.signature)
                    );
                    if let Some(ref d) = f.doc {
                        let _ = write!(html, "<p class=\"doc\">{}</p>\n", escape_html(d));
                    }
                }
                SymbolDoc::Struct(s) => {
                    let _ = write!(html, "<strong>struct {}</strong>\n", escape_html(&s.name));
                    if !s.trait_impls.is_empty() {
                        let impls = s.trait_impls.join(", ");
                        let _ = write!(html, "<span class=\"impls\">impl {impls}</span>\n");
                    }
                    if let Some(ref d) = s.doc {
                        let _ = write!(html, "<p class=\"doc\">{}</p>\n", escape_html(d));
                    }
                    for m in &s.methods {
                        let _ = write!(
                            html,
                            "<div class=\"method\"><code>{}</code></div>\n",
                            escape_html(&m.signature)
                        );
                    }
                }
                SymbolDoc::Enum(e) => {
                    let variants: Vec<&str> = e.variants.iter().map(|v| v.name.as_str()).collect();
                    let _ = write!(
                        html,
                        "<strong>enum {}</strong> {{ {} }}\n",
                        escape_html(&e.name),
                        escape_html(&variants.join(", "))
                    );
                }
                SymbolDoc::Trait(t) => {
                    let _ = write!(html, "<strong>trait {}</strong>\n", escape_html(&t.name));
                }
                SymbolDoc::TypeAlias(t) => {
                    let _ = write!(
                        html,
                        "<code>type {} = {}</code>\n",
                        escape_html(&t.name),
                        escape_html(&t.target_type)
                    );
                }
                SymbolDoc::Constant(c) => {
                    let val = c.value.as_deref().unwrap_or("...");
                    let _ = write!(
                        html,
                        "<code>const {}: {} = {}</code>\n",
                        escape_html(&c.name),
                        escape_html(&c.type_str),
                        escape_html(val)
                    );
                }
                _ => {
                    let _ = write!(html, "<strong>{}</strong>\n", escape_html(sym.name()));
                }
            }

            if let Some(dep) = sym.deprecated() {
                let _ = write!(
                    html,
                    "<div class=\"deprecated-note\">\u{26a0} Deprecated: {}</div>\n",
                    escape_html(dep)
                );
            }

            html.push_str("</div>\n");
        }
        html.push_str("</div>\n</details>\n");
    }

    html.push_str("</section>\n");
}

fn write_coverage_table(html: &mut String, doc: &ProjectDoc) {
    html.push_str("<section>\n<h2>Documentation Coverage</h2>\n");
    html.push_str(
        "<table class=\"coverage\">\n<tr><th>File</th><th>Documented</th><th>Coverage</th></tr>\n",
    );

    for module in &doc.modules {
        let total = module.symbols.len();
        let documented = module.symbols.iter().filter(|s| s.doc().is_some()).count();
        let pct = if total > 0 {
            documented as f32 / total as f32 * 100.0
        } else {
            100.0
        };
        let color = if pct >= 80.0 {
            "green"
        } else if pct >= 50.0 {
            "orange"
        } else {
            "red"
        };

        let _ = write!(html, "<tr><td>{}</td><td>{}/{}</td><td><span style=\"color:{color}\">{:.0}%</span></td></tr>\n",
            escape_html(&module.path.display().to_string()), documented, total, pct);
    }

    let _ = write!(html, "<tr class=\"total\"><td><strong>Total</strong></td><td><strong>{}/{}</strong></td><td><strong>{:.0}%</strong></td></tr>\n",
        doc.stats.documented_symbols, doc.stats.total_symbols,
        doc.stats.documentation_coverage * 100.0);

    html.push_str("</table>\n</section>\n");
}

fn write_todos_section(html: &mut String, doc: &ProjectDoc) {
    if doc.todos.is_empty() {
        return;
    }

    html.push_str("<section>\n<h2>TODOs &amp; FIXMEs</h2>\n<table>\n<tr><th>Kind</th><th>File</th><th>Line</th><th>Text</th></tr>\n");
    for todo in &doc.todos {
        let kind_str = match todo.kind {
            TodoKind::Todo => "TODO",
            TodoKind::Fixme => "FIXME",
            TodoKind::Hack => "HACK",
            TodoKind::Safety => "SAFETY",
        };
        let _ = write!(
            html,
            "<tr><td>{kind_str}</td><td>{}</td><td>{}</td><td>{}</td></tr>\n",
            escape_html(&todo.location.file.display().to_string()),
            todo.location.line,
            escape_html(&todo.text)
        );
    }
    html.push_str("</table>\n</section>\n");
}

// ─── Helpers ────────────────────────────────────────────────────────────────

fn escape_html(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

// ─── Embedded CSS ───────────────────────────────────────────────────────────

const CSS: &str = r#"
:root {
    --bg: #fafafa; --fg: #1a1a2e; --accent: #0f3460; --accent-light: #e8f0fe;
    --border: #e0e0e0; --code-bg: #f5f5f5; --card-bg: #fff; --card-shadow: rgba(0,0,0,0.08);
}
@media (prefers-color-scheme: dark) {
    :root {
        --bg: #1a1a2e; --fg: #e0e0e0; --accent: #53a8b6; --accent-light: #16213e;
        --border: #333; --code-bg: #16213e; --card-bg: #1e1e3a; --card-shadow: rgba(0,0,0,0.3);
    }
}
* { box-sizing: border-box; margin: 0; padding: 0; }
body { font-family: system-ui, -apple-system, sans-serif; background: var(--bg); color: var(--fg); max-width: 1100px; margin: 0 auto; padding: 2rem 1rem; line-height: 1.6; }
h1 { font-size: 1.8rem; margin-bottom: 0.5rem; } h1 small { color: var(--accent); font-weight: normal; font-size: 0.6em; }
h2 { font-size: 1.3rem; margin: 1.5rem 0 0.8rem; border-bottom: 2px solid var(--accent); padding-bottom: 0.3rem; }
h3 { font-size: 1.1rem; margin-bottom: 0.3rem; }
header { margin-bottom: 2rem; }
.badges { display: flex; flex-wrap: wrap; gap: 0.5rem; margin-top: 0.5rem; }
.badge { display: inline-block; padding: 0.15rem 0.6rem; border-radius: 12px; font-size: 0.8rem; background: var(--accent-light); color: var(--accent); border: 1px solid var(--border); }
.badge-green { background: #d4edda; color: #155724; } .badge-yellow { background: #fff3cd; color: #856404; } .badge-red { background: #f8d7da; color: #721c24; }
@media (prefers-color-scheme: dark) { .badge-green { background: #1a3a2a; color: #7dcea0; } .badge-yellow { background: #3a3520; color: #f0c040; } .badge-red { background: #3a1a1a; color: #f08080; } }
.search-container { margin: 1rem 0; }
#search { width: 100%; padding: 0.6rem 1rem; border: 1px solid var(--border); border-radius: 8px; font-size: 1rem; background: var(--card-bg); color: var(--fg); }
.cards { display: grid; grid-template-columns: repeat(auto-fill, minmax(280px, 1fr)); gap: 1rem; }
.card { background: var(--card-bg); border: 1px solid var(--border); border-radius: 8px; padding: 1rem; box-shadow: 0 1px 3px var(--card-shadow); }
.topics { margin-top: 0.5rem; font-size: 0.9rem; }
code, .sig { font-family: 'SF Mono', 'Fira Code', monospace; font-size: 0.85rem; background: var(--code-bg); padding: 0.1rem 0.3rem; border-radius: 3px; }
.sig { display: block; padding: 0.4rem 0.6rem; margin: 0.3rem 0; overflow-x: auto; white-space: nowrap; }
.doc { color: var(--fg); opacity: 0.85; margin: 0.3rem 0; font-size: 0.9rem; }
.method { margin-left: 1.5rem; padding: 0.2rem 0; }
.impls { font-size: 0.85rem; color: var(--accent); }
.symbol { padding: 0.5rem 0; border-bottom: 1px solid var(--border); }
.symbol:last-child { border-bottom: none; }
.deprecated { opacity: 0.6; }
.deprecated-note { color: #e67e22; font-size: 0.85rem; margin-top: 0.2rem; }
details { margin: 0.5rem 0; }
summary { cursor: pointer; padding: 0.5rem; background: var(--accent-light); border-radius: 6px; font-weight: 600; }
summary:hover { opacity: 0.9; }
.count { font-weight: normal; color: var(--accent); font-size: 0.85rem; }
.symbols { padding: 0.5rem 1rem; }
table { width: 100%; border-collapse: collapse; margin: 0.5rem 0; font-size: 0.9rem; }
th, td { text-align: left; padding: 0.4rem 0.8rem; border-bottom: 1px solid var(--border); }
th { background: var(--accent-light); font-weight: 600; }
.total td { font-weight: bold; border-top: 2px solid var(--accent); }
.coverage { max-width: 600px; }
.topic-graph { width: 100%; max-height: 300px; margin: 1rem 0; }
.graph-node { fill: var(--accent-light); stroke: var(--accent); stroke-width: 2; }
.graph-label { font-size: 13px; text-anchor: middle; fill: var(--fg); font-weight: 600; }
.graph-topic { font-size: 12px; fill: var(--fg); opacity: 0.7; }
.graph-topic-label { font-size: 11px; fill: var(--accent); font-style: italic; }
.hidden { display: none; }
"#;

// ─── Embedded JS ────────────────────────────────────────────────────────────

const SEARCH_JS: &str = r#"<script>
document.getElementById('search').addEventListener('input', function(e) {
    var q = e.target.value.toLowerCase();
    document.querySelectorAll('.symbol').forEach(function(el) {
        el.style.display = el.textContent.toLowerCase().indexOf(q) !== -1 ? '' : 'none';
    });
    // Auto-expand details that contain matches
    document.querySelectorAll('details').forEach(function(det) {
        var hasVisible = det.querySelector('.symbol:not([style*="display: none"])');
        if (q && hasVisible) { det.open = true; } else if (!q) { det.open = false; }
    });
});
</script>"#;

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn sample_doc() -> ProjectDoc {
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
                symbols: vec![SymbolDoc::Function(FunctionDoc {
                    name: "hello".to_string(),
                    visibility: Visibility::Public,
                    location: SourceLocation {
                        file: PathBuf::from("src/lib.rs"),
                        line: 1,
                        end_line: None,
                    },
                    signature: "pub fn hello() -> String".to_string(),
                    doc: Some("Say hello.".to_string()),
                    deprecated: None,
                    params: vec![],
                    returns: Some("String".to_string()),
                    is_async: false,
                    generic_params: vec![],
                    examples: vec![],
                })],
            }],
            relationships: vec![],
            message_graph: None,
            entry_points: vec![],
            todos: vec![TodoItem {
                kind: TodoKind::Todo,
                text: "fix this".to_string(),
                location: SourceLocation {
                    file: PathBuf::from("src/lib.rs"),
                    line: 42,
                    end_line: None,
                },
            }],
            stats: DocStats {
                total_files: 1,
                total_symbols: 1,
                documented_symbols: 1,
                deprecated_symbols: 0,
                documentation_coverage: 1.0,
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
    fn test_html_valid_structure() {
        let html = format_html(&sample_doc());
        assert!(html.contains("<!DOCTYPE html>"));
        assert!(html.contains("<html"));
        assert!(html.contains("<body>"));
        assert!(html.contains("</html>"));
    }

    #[test]
    fn test_html_self_contained() {
        let html = format_html(&sample_doc());
        assert!(!html.contains("http://"), "should not contain http:// URLs");
        assert!(
            !html.contains("https://"),
            "should not contain https:// URLs"
        );
    }

    #[test]
    fn test_html_search_input() {
        let html = format_html(&sample_doc());
        assert!(html.contains("id=\"search\""));
        assert!(html.contains("<input"));
    }

    #[test]
    fn test_html_dark_mode() {
        let html = format_html(&sample_doc());
        assert!(html.contains("prefers-color-scheme: dark"));
    }

    #[test]
    fn test_html_project_title() {
        let html = format_html(&sample_doc());
        assert!(html.contains("test-project"));
        assert!(html.contains("0.1.0"));
    }

    #[test]
    fn test_html_function_rendered() {
        let html = format_html(&sample_doc());
        assert!(html.contains("pub fn hello()"));
        assert!(html.contains("Say hello."));
    }

    #[test]
    fn test_html_coverage_table() {
        let html = format_html(&sample_doc());
        assert!(html.contains("Documentation Coverage"));
        assert!(html.contains("100%"));
    }

    #[test]
    fn test_html_todos_section() {
        let html = format_html(&sample_doc());
        assert!(html.contains("TODOs"));
        assert!(html.contains("fix this"));
    }

    #[test]
    fn test_html_collapsible() {
        let html = format_html(&sample_doc());
        assert!(html.contains("<details"));
        assert!(html.contains("<summary"));
    }

    #[test]
    fn test_html_js_search() {
        let html = format_html(&sample_doc());
        assert!(html.contains("<script>"));
        assert!(html.contains("getElementById('search')"));
    }

    #[test]
    fn test_html_empty_project() {
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
        let html = format_html(&doc);
        assert!(html.contains("<!DOCTYPE html>"));
        assert!(html.contains("empty"));
    }

    #[test]
    fn test_html_escapes_special_chars() {
        let mut doc = sample_doc();
        doc.project = "test<script>alert(1)</script>".to_string();
        let html = format_html(&doc);
        assert!(
            !html.contains("<script>alert(1)</script>"),
            "should escape HTML"
        );
        assert!(html.contains("&lt;script&gt;"));
    }

    #[test]
    fn test_html_svg_with_graph() {
        let mut doc = sample_doc();
        doc.message_graph = Some(MessageGraph {
            nodes: vec!["A".to_string(), "B".to_string()],
            topics: vec![MessageGraphTopic {
                name: "cmd_vel".to_string(),
                message_type: "CmdVel".to_string(),
                publishers: vec!["A".to_string()],
                subscribers: vec!["B".to_string()],
            }],
        });
        let html = format_html(&doc);
        assert!(html.contains("<svg"));
        assert!(html.contains("cmd_vel"));
    }

    #[test]
    fn test_html_no_svg_without_graph() {
        let html = format_html(&sample_doc());
        assert!(!html.contains("<svg"), "no graph = no SVG");
    }

    #[test]
    fn test_html_size_reasonable() {
        let html = format_html(&sample_doc());
        assert!(
            html.len() < 50_000,
            "simple project HTML should be < 50KB, got {} bytes",
            html.len()
        );
    }

    #[test]
    fn test_html_deprecated_rendering() {
        let mut doc = sample_doc();
        doc.modules[0]
            .symbols
            .push(SymbolDoc::Function(FunctionDoc {
                name: "old_api".to_string(),
                visibility: Visibility::Public,
                location: SourceLocation {
                    file: PathBuf::from("src/lib.rs"),
                    line: 10,
                    end_line: None,
                },
                signature: "pub fn old_api()".to_string(),
                doc: None,
                deprecated: Some("use new_api instead".to_string()),
                params: vec![],
                returns: None,
                is_async: false,
                generic_params: vec![],
                examples: vec![],
            }));
        let html = format_html(&doc);
        assert!(
            html.contains("deprecated"),
            "should contain 'deprecated' class"
        );
        assert!(
            html.contains("\u{26a0}"),
            "should contain warning symbol for deprecated"
        );
        assert!(
            html.contains("use new_api instead"),
            "should render deprecation message"
        );
    }

    #[test]
    fn test_html_message_section() {
        let mut doc = sample_doc();
        doc.modules[0]
            .symbols
            .push(SymbolDoc::HorusMessage(HorusMessageDoc {
                name: "CmdVel".to_string(),
                location: SourceLocation {
                    file: PathBuf::from("src/lib.rs"),
                    line: 20,
                    end_line: None,
                },
                doc: Some("Velocity command.".to_string()),
                deprecated: None,
                fields: vec![
                    FieldDoc {
                        name: "linear".to_string(),
                        type_str: Some("Vec3".to_string()),
                        doc: None,
                    },
                    FieldDoc {
                        name: "angular".to_string(),
                        type_str: Some("Vec3".to_string()),
                        doc: None,
                    },
                ],
            }));
        doc.stats.horus_messages = 1;
        let html = format_html(&doc);
        assert!(
            html.contains("Message Types"),
            "should contain 'Message Types' heading"
        );
        assert!(html.contains("CmdVel"), "should render message name");
        assert!(html.contains("linear"), "should render message fields");
    }

    #[test]
    fn test_html_node_section() {
        let mut doc = sample_doc();
        doc.entry_points.push(EntryPoint {
            kind: EntryPointKind::HorusNode,
            name: "MotorController".to_string(),
            file: PathBuf::from("src/motor.rs"),
            details: Some(EntryPointDetails {
                publishes: vec![TopicInfo {
                    name: "motor_status".to_string(),
                    message_type: "MotorStatus".to_string(),
                    direction: "publish".to_string(),
                }],
                subscribes: vec![],
                tick_rate: Some("100 Hz".to_string()),
                execution_class: None,
            }),
        });
        doc.stats.horus_nodes = 1;
        let html = format_html(&doc);
        assert!(html.contains("Nodes"), "should contain 'Nodes' heading");
        assert!(html.contains("MotorController"), "should render node name");
        assert!(
            html.contains("motor_status"),
            "should render published topic"
        );
    }

    #[test]
    fn test_html_struct_with_methods() {
        let mut doc = sample_doc();
        doc.modules[0].symbols.push(SymbolDoc::Struct(StructDoc {
            name: "Robot".to_string(),
            visibility: Visibility::Public,
            location: SourceLocation {
                file: PathBuf::from("src/lib.rs"),
                line: 30,
                end_line: None,
            },
            doc: Some("A robot.".to_string()),
            deprecated: None,
            generic_params: vec![],
            fields: vec![FieldDoc {
                name: "name".to_string(),
                type_str: Some("String".to_string()),
                doc: None,
            }],
            methods: vec![FunctionDoc {
                name: "move_to".to_string(),
                visibility: Visibility::Public,
                location: SourceLocation {
                    file: PathBuf::from("src/lib.rs"),
                    line: 35,
                    end_line: None,
                },
                signature: "pub fn move_to(&mut self, x: f64, y: f64)".to_string(),
                doc: None,
                deprecated: None,
                params: vec![],
                returns: None,
                is_async: false,
                generic_params: vec![],
                examples: vec![],
            }],
            trait_impls: vec![],
            derives: vec![],
            examples: vec![],
        }));
        let html = format_html(&doc);
        assert!(html.contains("struct Robot"), "should render struct name");
        assert!(
            html.contains("pub fn move_to"),
            "should render method signature in HTML"
        );
    }

    // ─── Level 2: Integration Tests ─────────────────────────────────────────

    #[test]
    fn test_integration_html_full_project() {
        let mut doc = sample_doc();

        // Add a message type
        doc.modules[0]
            .symbols
            .push(SymbolDoc::HorusMessage(HorusMessageDoc {
                name: "Odometry".to_string(),
                location: SourceLocation {
                    file: PathBuf::from("src/lib.rs"),
                    line: 50,
                    end_line: None,
                },
                doc: Some("Odometry message.".to_string()),
                deprecated: None,
                fields: vec![
                    FieldDoc {
                        name: "x".to_string(),
                        type_str: Some("f64".to_string()),
                        doc: None,
                    },
                    FieldDoc {
                        name: "y".to_string(),
                        type_str: Some("f64".to_string()),
                        doc: None,
                    },
                ],
            }));
        doc.stats.horus_messages = 1;

        // Add a node
        doc.entry_points.push(EntryPoint {
            kind: EntryPointKind::HorusNode,
            name: "NavigationNode".to_string(),
            file: PathBuf::from("src/nav.rs"),
            details: Some(EntryPointDetails {
                publishes: vec![TopicInfo {
                    name: "cmd_vel".to_string(),
                    message_type: "CmdVel".to_string(),
                    direction: "publish".to_string(),
                }],
                subscribes: vec![TopicInfo {
                    name: "odom".to_string(),
                    message_type: "Odometry".to_string(),
                    direction: "subscribe".to_string(),
                }],
                tick_rate: Some("50 Hz".to_string()),
                execution_class: None,
            }),
        });
        doc.stats.horus_nodes = 1;

        // Add a struct (already have fn from sample_doc)
        doc.modules[0].symbols.push(SymbolDoc::Struct(StructDoc {
            name: "Config".to_string(),
            visibility: Visibility::Public,
            location: SourceLocation {
                file: PathBuf::from("src/lib.rs"),
                line: 60,
                end_line: None,
            },
            doc: None,
            deprecated: None,
            generic_params: vec![],
            fields: vec![FieldDoc {
                name: "rate".to_string(),
                type_str: Some("f64".to_string()),
                doc: None,
            }],
            methods: vec![],
            trait_impls: vec![],
            derives: vec![],
            examples: vec![],
        }));

        // Add an enum
        doc.modules[0].symbols.push(SymbolDoc::Enum(EnumDoc {
            name: "DriveMode".to_string(),
            visibility: Visibility::Public,
            location: SourceLocation {
                file: PathBuf::from("src/lib.rs"),
                line: 70,
                end_line: None,
            },
            doc: None,
            deprecated: None,
            variants: vec![
                VariantDoc {
                    name: "Manual".to_string(),
                    doc: None,
                    fields: vec![],
                },
                VariantDoc {
                    name: "Auto".to_string(),
                    doc: None,
                    fields: vec![],
                },
            ],
            methods: vec![],
        }));

        // Add a deprecated function
        doc.modules[0]
            .symbols
            .push(SymbolDoc::Function(FunctionDoc {
                name: "old_drive".to_string(),
                visibility: Visibility::Public,
                location: SourceLocation {
                    file: PathBuf::from("src/lib.rs"),
                    line: 80,
                    end_line: None,
                },
                signature: "pub fn old_drive()".to_string(),
                doc: None,
                deprecated: Some("use new_drive instead".to_string()),
                params: vec![],
                returns: None,
                is_async: false,
                generic_params: vec![],
                examples: vec![],
            }));
        doc.stats.deprecated_symbols = 1;

        // Add a topic graph with SVG
        doc.message_graph = Some(MessageGraph {
            nodes: vec!["NavigationNode".to_string(), "MotorDriver".to_string()],
            topics: vec![MessageGraphTopic {
                name: "cmd_vel".to_string(),
                message_type: "CmdVel".to_string(),
                publishers: vec!["NavigationNode".to_string()],
                subscribers: vec!["MotorDriver".to_string()],
            }],
        });
        doc.stats.topics_discovered = 1;

        // Add a TODO (sample_doc already has one, add another)
        doc.todos.push(TodoItem {
            kind: TodoKind::Fixme,
            text: "handle timeout".to_string(),
            location: SourceLocation {
                file: PathBuf::from("src/nav.rs"),
                line: 99,
                end_line: None,
            },
        });
        doc.stats.fixmes = 1;

        let html = format_html(&doc);

        // Verify all major sections present
        assert!(
            html.contains("Message Types"),
            "should have Message Types section"
        );
        assert!(html.contains("Nodes"), "should have Nodes section");
        assert!(
            html.contains("API Reference"),
            "should have API Reference section"
        );
        assert!(
            html.contains("Documentation Coverage"),
            "should have Coverage section"
        );
        assert!(html.contains("TODOs"), "should have TODOs section");

        // SVG for topic graph
        assert!(html.contains("<svg"), "should have SVG for topic graph");

        // Deprecated markers
        assert!(
            html.contains("deprecated"),
            "should have deprecated markers"
        );
        assert!(
            html.contains("use new_drive instead"),
            "should render deprecation message"
        );

        // Verify content from each category
        assert!(html.contains("Odometry"), "should render message name");
        assert!(html.contains("NavigationNode"), "should render node name");
        assert!(html.contains("Config"), "should render struct name");
        assert!(html.contains("DriveMode"), "should render enum name");
        assert!(html.contains("handle timeout"), "should render FIXME text");
    }

    #[test]
    fn test_html_multiple_modules() {
        let mut doc = sample_doc();
        // Replace modules with 3 distinct modules
        doc.modules = vec![
            ModuleDoc {
                path: PathBuf::from("src/alpha.rs"),
                language: "rust".to_string(),
                module_doc: Some("Alpha module.".to_string()),
                imports: vec![],
                symbols: vec![SymbolDoc::Function(FunctionDoc {
                    name: "alpha_fn".to_string(),
                    visibility: Visibility::Public,
                    location: SourceLocation {
                        file: PathBuf::from("src/alpha.rs"),
                        line: 1,
                        end_line: None,
                    },
                    signature: "pub fn alpha_fn()".to_string(),
                    doc: None,
                    deprecated: None,
                    params: vec![],
                    returns: None,
                    is_async: false,
                    generic_params: vec![],
                    examples: vec![],
                })],
            },
            ModuleDoc {
                path: PathBuf::from("src/beta.rs"),
                language: "rust".to_string(),
                module_doc: Some("Beta module.".to_string()),
                imports: vec![],
                symbols: vec![SymbolDoc::Function(FunctionDoc {
                    name: "beta_fn".to_string(),
                    visibility: Visibility::Public,
                    location: SourceLocation {
                        file: PathBuf::from("src/beta.rs"),
                        line: 1,
                        end_line: None,
                    },
                    signature: "pub fn beta_fn()".to_string(),
                    doc: None,
                    deprecated: None,
                    params: vec![],
                    returns: None,
                    is_async: false,
                    generic_params: vec![],
                    examples: vec![],
                })],
            },
            ModuleDoc {
                path: PathBuf::from("src/gamma.rs"),
                language: "rust".to_string(),
                module_doc: Some("Gamma module.".to_string()),
                imports: vec![],
                symbols: vec![SymbolDoc::Function(FunctionDoc {
                    name: "gamma_fn".to_string(),
                    visibility: Visibility::Public,
                    location: SourceLocation {
                        file: PathBuf::from("src/gamma.rs"),
                        line: 1,
                        end_line: None,
                    },
                    signature: "pub fn gamma_fn()".to_string(),
                    doc: None,
                    deprecated: None,
                    params: vec![],
                    returns: None,
                    is_async: false,
                    generic_params: vec![],
                    examples: vec![],
                })],
            },
        ];
        doc.stats.total_symbols = 3;
        doc.stats.total_files = 3;

        let html = format_html(&doc);

        // Count <details class="collapsible"> occurrences — one per module
        let details_count = html.matches("<details class=\"collapsible\">").count();
        assert_eq!(
            details_count, 3,
            "should have 3 <details> sections, one per module, got {details_count}"
        );

        // Each module path should appear
        assert!(
            html.contains("src/alpha.rs"),
            "should contain alpha module path"
        );
        assert!(
            html.contains("src/beta.rs"),
            "should contain beta module path"
        );
        assert!(
            html.contains("src/gamma.rs"),
            "should contain gamma module path"
        );
    }

    #[test]
    fn test_html_enum_rendering() {
        let mut doc = sample_doc();
        doc.modules[0].symbols.push(SymbolDoc::Enum(EnumDoc {
            name: "Direction".to_string(),
            visibility: Visibility::Public,
            location: SourceLocation {
                file: PathBuf::from("src/lib.rs"),
                line: 40,
                end_line: None,
            },
            doc: None,
            deprecated: None,
            variants: vec![
                VariantDoc {
                    name: "North".to_string(),
                    doc: None,
                    fields: vec![],
                },
                VariantDoc {
                    name: "South".to_string(),
                    doc: None,
                    fields: vec![],
                },
                VariantDoc {
                    name: "East".to_string(),
                    doc: None,
                    fields: vec![],
                },
                VariantDoc {
                    name: "West".to_string(),
                    doc: None,
                    fields: vec![],
                },
            ],
            methods: vec![],
        }));

        let html = format_html(&doc);

        // Enum name rendered
        assert!(
            html.contains("enum Direction"),
            "should render enum name 'Direction'"
        );
        // All variant names present
        assert!(html.contains("North"), "should render variant 'North'");
        assert!(html.contains("South"), "should render variant 'South'");
        assert!(html.contains("East"), "should render variant 'East'");
        assert!(html.contains("West"), "should render variant 'West'");
    }
}
