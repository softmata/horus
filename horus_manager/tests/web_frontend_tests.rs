//! Phase 6: Web Frontend (JavaScript) Tests
//!
//! These tests verify that the generated HTML/JavaScript frontend is correct
//! without requiring a browser runtime.  Since the JS is embedded in Rust
//! string literals (`monitor::html::generate_html`), we validate:
//!
//! - **Infrastructure**: All required DOM element IDs, JS functions, and CSS
//!   classes are present in the generated HTML.
//! - **Fetch/polling cycle**: API endpoints return JSON with the exact field
//!   names the JS code destructures (contract alignment).
//! - **DOM rendering**: HTML templates reference correct element IDs and CSS
//!   classes; empty-state placeholders are present.
//! - **WebSocket client handling**: The JS WebSocket code processes server
//!   messages correctly (validated via server-side message structure).
//! - **Error display & auto-refresh**: Error CSS classes, error elements, and
//!   auto-refresh interval constants are present.

mod monitor_tests;

use monitor_tests::builders;
use monitor_tests::helpers::{assert_json_ok, get_request};
use monitor_tests::shm_fixtures::TestShmGuard;
use tower::ServiceExt;

// ═══════════════════════════════════════════════════════════════════════════════
//  Helper: Get generated HTML from index handler
// ═══════════════════════════════════════════════════════════════════════════════

/// Fetch the monitor HTML via the index handler (auth disabled → returns
/// monitor page, not login page).
async fn get_monitor_html() -> String {
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/")).await.unwrap();
    assert_eq!(resp.status(), axum::http::StatusCode::OK);

    let body = axum::body::to_bytes(resp.into_body(), 5 * 1024 * 1024)
        .await
        .expect("reading index response body must not fail");
    String::from_utf8(body.to_vec()).expect("index response must be valid UTF-8")
}

/// Fetch the login HTML via the index handler (auth enabled → returns login
/// page for unauthenticated requests).
async fn get_login_html() -> String {
    let state = builders::test_app_state_with_auth();
    let app = builders::test_router_with_state(state);
    let resp = app.oneshot(get_request("/")).await.unwrap();
    assert_eq!(resp.status(), axum::http::StatusCode::OK);

    let body = axum::body::to_bytes(resp.into_body(), 2 * 1024 * 1024)
        .await
        .expect("reading login response body must not fail");
    String::from_utf8(body.to_vec()).expect("login response must be valid UTF-8")
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TASK 1: JavaScript test infrastructure — HTML/JS validation helpers
// ═══════════════════════════════════════════════════════════════════════════════

/// Assert that the HTML contains a `<script>` block.
fn assert_has_script(html: &str) {
    assert!(
        html.contains("<script>"),
        "generated HTML must contain a <script> block"
    );
    assert!(
        html.contains("</script>"),
        "generated HTML must contain a closing </script> tag"
    );
}

/// Assert that the HTML contains a specific JS function definition.
fn assert_has_js_function(html: &str, fn_name: &str) {
    let patterns = [
        format!("function {}(", fn_name),
        format!("function {} (", fn_name),
        format!("async function {}(", fn_name),
        format!("async function {} (", fn_name),
    ];
    let found = patterns.iter().any(|p| html.contains(p));
    assert!(
        found,
        "generated HTML must contain JS function '{}'",
        fn_name
    );
}

/// Assert that the HTML contains an element with the given ID.
fn assert_has_element_id(html: &str, id: &str) {
    let pattern = format!("id=\"{}\"", id);
    let pattern_single = format!("id='{}'", id);
    assert!(
        html.contains(&pattern) || html.contains(&pattern_single),
        "generated HTML must contain element with id='{}'",
        id
    );
}

/// Assert that the HTML contains a CSS class definition or reference.
fn assert_has_css_class(html: &str, class: &str) {
    // Check in style blocks (CSS selectors), class attributes, and JS strings
    let patterns = [
        format!(".{}", class),             // CSS selector: .node-item
        format!("class=\"{}", class),      // First in class attr: class="node-item..."
        format!(" {}\"", class),           // Last in class attr: ...node-item"
        format!(" {} ", class),            // Middle of class attr: ...foo node-item bar...
        format!("'{}'", class),            // JS string: '.node-item'
        format!(".closest('.{}')", class), // JS delegation: .closest('.node-item')
    ];
    let found = patterns.iter().any(|p| html.contains(p.as_str()));
    assert!(
        found,
        "generated HTML must reference CSS class '{}' in a CSS selector, class attribute, or JS string",
        class
    );
}

// ─── Infrastructure: monitor page ──────────────────────────────────────────

#[tokio::test]
async fn infra_monitor_html_contains_script_block() {
    let html = get_monitor_html().await;
    assert_has_script(&html);
}

#[tokio::test]
async fn infra_monitor_html_contains_doctype_and_structure() {
    let html = get_monitor_html().await;
    assert!(html.contains("<!DOCTYPE html>"), "must have DOCTYPE");
    assert!(html.contains("<html"), "must have <html> tag");
    assert!(html.contains("</html>"), "must have closing </html>");
    assert!(html.contains("<head>"), "must have <head>");
    assert!(html.contains("<body>"), "must have <body>");
    assert!(html.contains("HORUS Monitor"), "must have title");
}

#[tokio::test]
async fn infra_monitor_html_has_all_core_js_functions() {
    let html = get_monitor_html().await;

    // Tab/view switching
    assert_has_js_function(&html, "switchTab");
    assert_has_js_function(&html, "switchMonitorView");

    // Data fetching
    assert_has_js_function(&html, "updateStatus");
    assert_has_js_function(&html, "updateNodes");
    assert_has_js_function(&html, "updateTopics");
    assert_has_js_function(&html, "updateGraphData");
    assert_has_js_function(&html, "updateAll");
    assert_has_js_function(&html, "refreshMonitorData");

    // Tooltip updates
    assert_has_js_function(&html, "updateNodesToolTip");
    assert_has_js_function(&html, "updateTopicsToolTip");

    // WebSocket
    assert_has_js_function(&html, "connectWebSocket");

    // Log panel
    assert_has_js_function(&html, "showNodeLogs");
    assert_has_js_function(&html, "showTopicLogs");
    assert_has_js_function(&html, "closeLogPanel");

    // Graph
    assert_has_js_function(&html, "renderGraph");
    assert_has_js_function(&html, "resetGraphLayout");
    assert_has_js_function(&html, "initGraphInteraction");

    // Theme
    assert_has_js_function(&html, "toggleTheme");
    assert_has_js_function(&html, "loadTheme");

    // Packages
    assert_has_js_function(&html, "searchRegistry");
    assert_has_js_function(&html, "loadEnvironments");
    assert_has_js_function(&html, "switchPackageView");
    assert_has_js_function(&html, "loadGlobalEnvironment");
    assert_has_js_function(&html, "loadLocalEnvironments");
    assert_has_js_function(&html, "confirmInstall");
    assert_has_js_function(&html, "showInstallDialog");
    assert_has_js_function(&html, "closeInstallDialog");

    // Parameters
    assert_has_js_function(&html, "refreshParams");
    assert_has_js_function(&html, "renderParams");
    assert_has_js_function(&html, "saveParam");
    assert_has_js_function(&html, "deleteParam");
    assert_has_js_function(&html, "editParam");
    assert_has_js_function(&html, "showAddParamDialog");
    assert_has_js_function(&html, "closeParamDialog");
    assert_has_js_function(&html, "exportParams");
    assert_has_js_function(&html, "importParams");

    // Help
    assert_has_js_function(&html, "toggleHelp");
}

#[tokio::test]
async fn infra_monitor_html_has_all_required_dom_element_ids() {
    let html = get_monitor_html().await;

    // Core layout elements
    assert_has_element_id(&html, "nodes-list");
    assert_has_element_id(&html, "topics-list");
    assert_has_element_id(&html, "graph-canvas");

    // Status bar elements
    assert_has_element_id(&html, "node-count");
    assert_has_element_id(&html, "topic-count");

    // Tooltip elements
    assert_has_element_id(&html, "nodes-tooltip-content");
    assert_has_element_id(&html, "topics-tooltip-content");

    // Log panel
    assert_has_element_id(&html, "log-panel");
    assert_has_element_id(&html, "log-panel-title");
    assert_has_element_id(&html, "log-panel-content");

    // Theme
    assert_has_element_id(&html, "theme-icon");

    // Help modal
    assert_has_element_id(&html, "help-modal");

    // Tabs
    assert_has_element_id(&html, "tab-packages");
    assert_has_element_id(&html, "tab-params");

    // Package management
    assert_has_element_id(&html, "install-dialog");
    assert_has_element_id(&html, "install-pkg-name");
    assert_has_element_id(&html, "install-option-global");
    assert_has_element_id(&html, "install-option-local");
    assert_has_element_id(&html, "radio-global");
    assert_has_element_id(&html, "radio-local");
    assert_has_element_id(&html, "local-package-select");

    // Parameter management
    assert_has_element_id(&html, "params-table-body");
    assert_has_element_id(&html, "param-dialog");
    assert_has_element_id(&html, "dialog-title");
    assert_has_element_id(&html, "param-key");
    assert_has_element_id(&html, "param-type");
    assert_has_element_id(&html, "param-value");
    assert_has_element_id(&html, "param-search");

    // Import/Export
    assert_has_element_id(&html, "import-dialog");
    assert_has_element_id(&html, "import-format");
    assert_has_element_id(&html, "import-data");
}

#[tokio::test]
async fn infra_monitor_html_has_required_css_classes() {
    let html = get_monitor_html().await;

    // Layout classes
    assert_has_css_class(&html, "container");
    assert_has_css_class(&html, "sidebar");
    assert_has_css_class(&html, "tab-content");
    assert_has_css_class(&html, "nav-item");

    // Node/topic rendering
    assert_has_css_class(&html, "node-item");
    assert_has_css_class(&html, "node-header");
    assert_has_css_class(&html, "node-name");
    assert_has_css_class(&html, "node-status");
    assert_has_css_class(&html, "node-details");
    assert_has_css_class(&html, "topic-item");
    assert_has_css_class(&html, "topic-header");
    assert_has_css_class(&html, "topic-name");
    assert_has_css_class(&html, "topic-details");

    // Health status colors
    assert_has_css_class(&html, "status-running");

    // Log panel
    assert_has_css_class(&html, "log-panel");
    assert_has_css_class(&html, "log-panel-header");
    assert_has_css_class(&html, "log-panel-content");
    assert_has_css_class(&html, "log-panel-title");

    // Views
    assert_has_css_class(&html, "monitor-view");
    assert_has_css_class(&html, "view-btn");
}

// ─── Infrastructure: login page ────────────────────────────────────────────

#[tokio::test]
async fn infra_login_html_has_form_and_script() {
    let html = get_login_html().await;

    assert!(html.contains("<!DOCTYPE html>"), "must have DOCTYPE");
    assert!(html.contains("HORUS"), "must have HORUS branding");
    assert_has_element_id(&html, "loginForm");
    assert_has_element_id(&html, "password");
    assert_has_element_id(&html, "error");
    assert_has_script(&html);
    // Login form submits to /api/login
    assert!(
        html.contains("/api/login"),
        "login page must reference /api/login endpoint"
    );
}

#[tokio::test]
async fn infra_login_error_display_present() {
    let html = get_login_html().await;
    // Error div with 'show' class toggling
    assert!(
        html.contains("classList.add('show')"),
        "login JS must add 'show' class on error"
    );
    assert!(
        html.contains("classList.remove('show')"),
        "login JS must remove 'show' class on submit"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TASK 2: Test fetch/polling cycle correctness
// ═══════════════════════════════════════════════════════════════════════════════

// The frontend JS calls these endpoints and destructures specific field names.
// These tests verify the API contract matches what the JS expects.

#[tokio::test]
async fn polling_status_endpoint_has_nodes_and_topics_fields() {
    // JS: `data.nodes` and `data.topics` in updateStatus()
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/status")).await.unwrap();
    let json = assert_json_ok(resp).await;

    assert!(
        json["nodes"].is_number(),
        "status response must have numeric 'nodes' field (JS: data.nodes)"
    );
    assert!(
        json["topics"].is_number(),
        "status response must have numeric 'topics' field (JS: data.topics)"
    );
}

#[tokio::test]
async fn polling_nodes_endpoint_has_expected_node_fields() {
    // JS destructures: node.name, node.health, node.health_color, node.pid,
    //                   node.cpu, node.memory
    // Inject a test node so the endpoint returns non-empty data
    let mut guard = TestShmGuard::new();
    guard.create_presence("wf_nf", std::process::id());
    // Wait for discovery cache to expire (250ms TTL)
    tokio::time::sleep(std::time::Duration::from_millis(350)).await;

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/nodes")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let nodes = json["nodes"]
        .as_array()
        .expect("nodes response must have 'nodes' array");
    assert!(
        !nodes.is_empty(),
        "nodes array must not be empty (test fixture injected)"
    );

    // Validate all returned nodes have the fields JS destructures
    for node in nodes {
        let required_fields = ["name", "health", "health_color", "cpu", "memory"];
        for field in &required_fields {
            assert!(
                node.get(field).is_some(),
                "node object must have '{}' field (used by JS template), got: {}",
                field,
                node
            );
        }
    }

    drop(guard);
}

#[tokio::test]
async fn polling_topics_endpoint_has_expected_topic_fields() {
    // JS destructures: topic.name, topic.active, topic.size, topic.processes
    // Inject a test topic so the endpoint returns non-empty data
    let mut guard = TestShmGuard::new();
    guard.create_topic("wf_tf", 65536);
    tokio::time::sleep(std::time::Duration::from_millis(350)).await;

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/topics")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let topics = json["topics"]
        .as_array()
        .expect("topics response must have 'topics' array");
    assert!(
        !topics.is_empty(),
        "topics array must not be empty (test fixture injected)"
    );

    // Validate all returned topics have the fields JS destructures
    for topic in topics {
        let required_fields = ["name", "active", "size", "processes"];
        for field in &required_fields {
            assert!(
                topic.get(field).is_some(),
                "topic object must have '{}' field (used by JS template), got: {}",
                field,
                topic
            );
        }
    }

    drop(guard);
}

#[tokio::test]
async fn polling_graph_endpoint_has_nodes_and_edges() {
    // JS: data.nodes, data.edges in updateGraphData()
    // Inject interconnected nodes so graph has both nodes and edges
    let mut guard = TestShmGuard::new();
    guard.create_presence_with_topics(
        "wf_gf_pub",
        std::process::id(),
        &[("wf_gf_topic", "SensorData")],
        &[],
    );
    guard.create_presence_with_topics(
        "wf_gf_sub",
        std::process::id(),
        &[],
        &[("wf_gf_topic", "SensorData")],
    );
    guard.create_topic("wf_gf_topic", 4096);
    tokio::time::sleep(std::time::Duration::from_millis(350)).await;

    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/graph")).await.unwrap();
    let json = assert_json_ok(resp).await;

    let nodes = json["nodes"]
        .as_array()
        .expect("graph response must have 'nodes' array");
    let edges = json["edges"]
        .as_array()
        .expect("graph response must have 'edges' array");

    assert!(
        !nodes.is_empty(),
        "graph nodes must not be empty (test fixtures injected)"
    );

    // Validate graph node fields
    for node in nodes {
        for field in &["id", "label", "type"] {
            assert!(
                node.get(field).is_some(),
                "graph node must have '{}' field (used by renderGraph), got: {}",
                field,
                node
            );
        }
        // type must be "process" or "topic"
        let t = node["type"]
            .as_str()
            .expect("graph node 'type' must be a string");
        assert!(
            t == "process" || t == "topic",
            "graph node type must be 'process' or 'topic', got '{}'",
            t
        );
    }

    // Validate edge fields (edges may be empty if graph topology doesn't generate them)
    for edge in edges {
        for field in &["from", "to", "type"] {
            assert!(
                edge.get(field).is_some(),
                "graph edge must have '{}' field (used by renderGraph), got: {}",
                field,
                edge
            );
        }
    }

    drop(guard);
}

#[tokio::test]
async fn polling_js_references_correct_api_paths() {
    let html = get_monitor_html().await;

    // Verify all API paths referenced in fetch() calls exist in the JS
    let expected_api_paths = [
        "/api/status",
        "/api/nodes",
        "/api/topics",
        "/api/graph",
        "/api/logs/node/",
        "/api/logs/topic/",
        "/api/packages/environments",
        "/api/packages/registry",
        "/api/packages/install",
        "/api/packages/uninstall",
        "/api/packages/publish",
        "/api/params",
        "/api/params/export",
        "/api/params/import",
    ];

    for path in &expected_api_paths {
        assert!(
            html.contains(path),
            "JS must reference API path '{}' in fetch() calls",
            path
        );
    }
}

#[tokio::test]
async fn polling_updateall_calls_all_update_functions() {
    // Verify updateAll() calls all the individual update functions
    let html = get_monitor_html().await;

    // Extract the updateAll function body to verify calls are inside it
    let update_all_start = html
        .find("function updateAll()")
        .or_else(|| html.find("async function updateAll()"))
        .expect("JS must define updateAll() function");

    // Find the function body by tracking brace depth
    let after_fn = &html[update_all_start..];
    let body_start = after_fn
        .find('{')
        .expect("updateAll must have a function body");
    let fn_body_start = update_all_start + body_start;

    // Find matching closing brace
    let mut depth = 0u32;
    let mut fn_body_end = fn_body_start;
    for (i, ch) in html[fn_body_start..].char_indices() {
        match ch {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    fn_body_end = fn_body_start + i;
                    break;
                }
            }
            _ => {}
        }
    }
    assert!(
        fn_body_end > fn_body_start,
        "could not find updateAll body end"
    );

    let update_all_body = &html[fn_body_start..=fn_body_end];

    // These functions must be called INSIDE updateAll
    let expected_calls = [
        "updateStatus()",
        "updateNodes()",
        "updateTopics()",
        "updateGraphData()",
        "updateNodesToolTip()",
        "updateTopicsToolTip()",
    ];

    for call in &expected_calls {
        assert!(
            update_all_body.contains(call),
            "updateAll() body must call '{}' — not found in function body",
            call
        );
    }
}

#[tokio::test]
async fn polling_fallback_interval_present() {
    let html = get_monitor_html().await;

    // Polling fallback: setInterval(updateAll, 1000)
    assert!(
        html.contains("setInterval(updateAll, 1000)"),
        "JS must set 1-second polling interval as WebSocket fallback"
    );
}

#[tokio::test]
async fn polling_logs_endpoint_has_expected_fields() {
    // JS destructures: data.logs (array), log.timestamp, log.log_type,
    //                   log.message, log.topic, log.tick_us, log.ipc_ns
    let app = builders::test_router();
    let resp = app
        .oneshot(get_request("/api/logs/node/nonexistent_node"))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;

    assert!(
        json["logs"].is_array(),
        "node logs response must have 'logs' array"
    );
    assert!(
        json["node"].is_string(),
        "node logs response must echo 'node' name"
    );
}

#[tokio::test]
async fn polling_params_endpoint_has_expected_fields() {
    // JS: data.success, data.params
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/params")).await.unwrap();
    let json = assert_json_ok(resp).await;

    assert!(
        json["success"].is_boolean(),
        "params response must have boolean 'success' field"
    );
    assert!(
        json["params"].is_array(),
        "params response must have 'params' array"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TASK 3: Test DOM rendering of node and topic data
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn dom_node_template_uses_correct_data_attributes() {
    let html = get_monitor_html().await;

    // JS template string uses data-node-name for click delegation
    assert!(
        html.contains("data-node-name"),
        "node template must use 'data-node-name' attribute"
    );

    // JS template references these interpolations: ${node.name}, ${node.health},
    // ${node.health_color}, ${node.pid}, ${node.cpu}, ${node.memory}
    let node_template_fields = [
        "node.name",
        "node.health",
        "node.health_color",
        "node.cpu",
        "node.memory",
    ];
    for field in &node_template_fields {
        assert!(
            html.contains(field),
            "node template must reference '{}' for rendering",
            field
        );
    }
}

#[tokio::test]
async fn dom_topic_template_uses_correct_data_attributes() {
    let html = get_monitor_html().await;

    // JS template uses data-topic-name for click delegation
    assert!(
        html.contains("data-topic-name"),
        "topic template must use 'data-topic-name' attribute"
    );

    // JS template references: ${topic.name}, ${topic.active}, ${topic.size},
    // ${topic.processes}
    let topic_template_fields = [
        "topic.name",
        "topic.active",
        "topic.size",
        "topic.processes",
    ];
    for field in &topic_template_fields {
        assert!(
            html.contains(field),
            "topic template must reference '{}' for rendering",
            field
        );
    }
}

#[tokio::test]
async fn dom_empty_state_placeholders_present() {
    let html = get_monitor_html().await;

    // Node empty state
    assert!(
        html.contains("No active nodes detected"),
        "must have node empty-state placeholder text"
    );

    // Topic empty state
    assert!(
        html.contains("No topics available"),
        "must have topic empty-state placeholder text"
    );
}

#[tokio::test]
async fn dom_event_delegation_for_node_and_topic_clicks() {
    let html = get_monitor_html().await;

    // Event delegation uses .closest() to find clickable items
    assert!(
        html.contains(".closest('.node-item')"),
        "JS must use event delegation with .closest('.node-item')"
    );
    assert!(
        html.contains(".closest('.topic-item')"),
        "JS must use event delegation with .closest('.topic-item')"
    );

    // getAttribute for extracting names
    assert!(
        html.contains("getAttribute('data-node-name')"),
        "JS must extract node name via getAttribute"
    );
    assert!(
        html.contains("getAttribute('data-topic-name')"),
        "JS must extract topic name via getAttribute"
    );
}

#[tokio::test]
async fn dom_log_panel_has_open_close_mechanism() {
    let html = get_monitor_html().await;

    // Opening: classList.add('open')
    assert!(
        html.contains("classList.add('open')"),
        "log panel must use 'open' class to show"
    );
    // Closing: classList.remove('open')
    assert!(
        html.contains("classList.remove('open')"),
        "log panel must remove 'open' class to hide"
    );
    // Close button
    assert!(
        html.contains("closeLogPanel()"),
        "must have closeLogPanel() call"
    );
}

#[tokio::test]
async fn dom_graph_canvas_element_present() {
    let html = get_monitor_html().await;
    assert_has_element_id(&html, "graph-canvas");
    assert!(
        html.contains("<canvas"),
        "must have a <canvas> element for graph rendering"
    );
}

#[tokio::test]
async fn dom_tooltip_rendering_templates_present() {
    let html = get_monitor_html().await;

    // Nodes tooltip template
    assert!(
        html.contains("tooltip-node-item"),
        "must have tooltip node item class"
    );
    assert!(
        html.contains("tooltip-node-name"),
        "must have tooltip node name class"
    );

    // Topics tooltip template
    assert!(
        html.contains("tooltip-topic-item"),
        "must have tooltip topic item class"
    );
    assert!(
        html.contains("tooltip-topic-name"),
        "must have tooltip topic name class"
    );
}

#[tokio::test]
async fn dom_cached_elements_helper_present() {
    let html = get_monitor_html().await;

    // getCachedElements() caches DOM lookups for performance
    assert_has_js_function(&html, "getCachedElements");
    assert!(
        html.contains("cachedDOMElements"),
        "must have cachedDOMElements variable"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TASK 4: Test WebSocket client message handling
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn ws_client_constructs_correct_url() {
    let html = get_monitor_html().await;

    // Protocol detection: ws: or wss:
    assert!(
        html.contains("wss:") && html.contains("ws:"),
        "JS must handle both ws: and wss: protocols"
    );

    // URL construction: /api/ws
    assert!(
        html.contains("/api/ws"),
        "JS must connect to /api/ws endpoint"
    );
}

#[tokio::test]
async fn ws_client_handles_open_event() {
    let html = get_monitor_html().await;

    // ws.onopen sets wsConnected = true
    assert!(
        html.contains("ws.onopen"),
        "JS must handle WebSocket onopen event"
    );
    assert!(
        html.contains("wsConnected = true"),
        "JS must set wsConnected to true on open"
    );

    // Should clear polling fallback on successful WS connection
    assert!(
        html.contains("clearInterval(pollingInterval)"),
        "JS must clear polling interval when WS connects"
    );
}

#[tokio::test]
async fn ws_client_handles_message_event() {
    let html = get_monitor_html().await;

    // ws.onmessage processes JSON
    assert!(
        html.contains("ws.onmessage"),
        "JS must handle WebSocket onmessage event"
    );

    // Parses JSON: JSON.parse(event.data)
    assert!(
        html.contains("JSON.parse(event.data)"),
        "JS must parse WS message as JSON"
    );

    // Checks message type: update.type === 'update'
    assert!(
        html.contains("update.type === 'update'"),
        "JS must check for message type 'update'"
    );
}

#[tokio::test]
async fn ws_client_processes_node_updates() {
    let html = get_monitor_html().await;

    // WS handler updates nodes from update.data.nodes
    assert!(
        html.contains("update.data.nodes"),
        "JS must process update.data.nodes from WS"
    );

    // Full refresh when count changes
    assert!(
        html.contains("existingNodes.length !== update.data.nodes.length"),
        "JS must detect node count changes for full refresh"
    );
}

#[tokio::test]
async fn ws_client_processes_topic_updates() {
    let html = get_monitor_html().await;

    // WS handler updates topics from update.data.topics
    assert!(
        html.contains("update.data.topics"),
        "JS must process update.data.topics from WS"
    );

    // Full refresh when count changes
    assert!(
        html.contains("existingTopics.length !== update.data.topics.length"),
        "JS must detect topic count changes for full refresh"
    );
}

#[tokio::test]
async fn ws_client_processes_graph_updates() {
    let html = get_monitor_html().await;

    // WS handler updates graph data
    assert!(
        html.contains("update.data.graph"),
        "JS must process update.data.graph from WS"
    );
}

#[tokio::test]
async fn ws_client_handles_close_and_reconnect() {
    let html = get_monitor_html().await;

    // ws.onclose triggers reconnect
    assert!(
        html.contains("ws.onclose"),
        "JS must handle WebSocket onclose event"
    );

    // Reconnect after timeout
    assert!(
        html.contains("setTimeout(connectWebSocket, 5000)"),
        "JS must reconnect after 5 seconds on close"
    );

    // Fallback to polling on close
    assert!(
        html.contains("pollingInterval = setInterval(updateAll, 1000)"),
        "JS must start polling fallback on WS close"
    );
}

#[tokio::test]
async fn ws_client_handles_error_event() {
    let html = get_monitor_html().await;

    // ws.onerror
    assert!(
        html.contains("ws.onerror"),
        "JS must handle WebSocket onerror event"
    );
    assert!(
        html.contains("wsConnected = false"),
        "JS must set wsConnected to false on error"
    );
}

#[tokio::test]
async fn ws_client_message_parse_error_handling() {
    let html = get_monitor_html().await;

    // try/catch around JSON.parse
    assert!(
        html.contains("WebSocket message parse error"),
        "JS must log parse errors from WebSocket messages"
    );
}

#[tokio::test]
async fn ws_server_endpoint_exists_and_rejects_non_upgrade() {
    // Verify the /api/ws endpoint is routed (not 404/405).
    // A plain GET without proper upgrade headers returns 4xx (not 404),
    // confirming the route exists.  Full WS upgrade testing is covered in
    // uat_websocket_live.rs which uses a real TCP listener.
    let app = builders::test_router();
    let resp = app.oneshot(get_request("/api/ws")).await.unwrap();

    // Must NOT be 404 (route exists) or 405 (method allowed)
    let status = resp.status().as_u16();
    assert_ne!(status, 404, "/api/ws route must exist (got 404)");
    assert_ne!(status, 405, "/api/ws must accept GET method (got 405)");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TASK 5: Test web frontend error display and auto-refresh behavior
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn error_css_classes_defined() {
    let html = get_monitor_html().await;

    // Error color CSS custom property must be defined
    assert!(
        html.contains("--error"),
        "CSS must define --error custom property"
    );
    // Verify error color is actually assigned (not just referenced)
    assert!(
        html.contains("--error:") || html.contains("--error :"),
        "CSS must assign a value to --error (e.g., --error: #ff4444)"
    );
    // Error color must appear in an error-related styling context
    assert!(
        html.contains("#ff4444"),
        "CSS must define error color #ff4444"
    );
}

#[tokio::test]
async fn error_display_in_log_panel() {
    let html = get_monitor_html().await;

    // Log panel shows error messages when fetch fails
    assert!(
        html.contains("Error loading logs:"),
        "log panel must display error message on fetch failure"
    );
    assert!(
        html.contains("color: #ff4444"),
        "error text must have red color styling"
    );
}

#[tokio::test]
async fn error_display_in_package_views() {
    let html = get_monitor_html().await;

    // Package views show errors
    assert!(
        html.contains("Failed to load global packages:"),
        "global packages view must show fetch errors"
    );
    assert!(
        html.contains("Failed to load local environments:"),
        "local environments view must show fetch errors"
    );
    assert!(
        html.contains("Search failed:"),
        "registry search must show search errors"
    );
}

#[tokio::test]
async fn error_console_logging_present() {
    let html = get_monitor_html().await;

    // All async functions should have console.error for failures
    let error_logs = [
        "Failed to fetch status:",
        "Failed to fetch nodes:",
        "Failed to fetch topics:",
        "Failed to fetch graph:",
        "Failed to update nodes tooltip:",
        "Failed to update topics tooltip:",
        "Failed to fetch params:",
    ];

    for msg in &error_logs {
        assert!(html.contains(msg), "JS must log '{}' on fetch failure", msg);
    }
}

#[tokio::test]
async fn auto_refresh_log_panel_interval() {
    let html = get_monitor_html().await;

    // Log panel auto-refreshes at 200ms (5 times per second)
    assert!(
        html.contains("setInterval(updateLogs, 200)"),
        "log panel must auto-refresh logs at 200ms interval"
    );
}

#[tokio::test]
async fn auto_refresh_log_panel_guard_prevents_stale_updates() {
    let html = get_monitor_html().await;

    // The JS must track current log view state
    assert!(
        html.contains("currentLogView"),
        "JS must have currentLogView variable for log panel state"
    );

    // Guard pattern: the log update functions must check currentLogView.type
    // to avoid updating the wrong panel (node logs vs topic logs)
    assert!(
        html.contains("currentLogView.type"),
        "log update must check currentLogView.type for stale view guard"
    );

    // Both node and topic type guards must exist (not just one)
    assert!(
        html.contains("'node'") && html.contains("'topic'"),
        "JS must guard against both 'node' and 'topic' view types"
    );
}

#[tokio::test]
async fn auto_refresh_initial_load_on_startup() {
    let html = get_monitor_html().await;

    // Initial data load via updateAll() on page load
    // This should be called at the end of the script
    assert!(
        html.contains("updateAll();"),
        "JS must call updateAll() for initial data load on startup"
    );
}

#[tokio::test]
async fn auto_refresh_ws_reconnect_clears_polling() {
    let html = get_monitor_html().await;

    // When WS reconnects, polling should stop
    assert!(
        html.contains("clearInterval(pollingInterval)"),
        "JS must clear polling interval when WebSocket reconnects"
    );
    assert!(
        html.contains("pollingInterval = null"),
        "JS must null out pollingInterval after clearing"
    );
}

#[tokio::test]
async fn auto_refresh_params_tab_observer() {
    let html = get_monitor_html().await;

    // MutationObserver watches for params tab activation
    assert!(
        html.contains("MutationObserver"),
        "JS must use MutationObserver for params tab activation"
    );
    assert!(
        html.contains("paramsTabObserver"),
        "JS must have paramsTabObserver variable"
    );
}

#[tokio::test]
async fn error_login_page_network_error_handling() {
    let html = get_login_html().await;

    // Login page handles network errors
    assert!(
        html.contains("Network error"),
        "login page must show 'Network error' message"
    );
}

#[tokio::test]
async fn error_login_page_invalid_password_handling() {
    let html = get_login_html().await;

    // Login page shows server error messages
    assert!(
        html.contains("data.error || 'Invalid password'"),
        "login page must show server error or 'Invalid password' fallback"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Cross-cutting: Theme persistence
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn theme_persistence_uses_localstorage() {
    let html = get_monitor_html().await;

    assert!(
        html.contains("localStorage.setItem('horus-theme'"),
        "theme toggle must persist to localStorage"
    );
    assert!(
        html.contains("localStorage.getItem('horus-theme')"),
        "theme load must read from localStorage"
    );
}

#[tokio::test]
async fn theme_supports_light_and_dark() {
    let html = get_monitor_html().await;

    // Dark is the default base theme; light overrides via [data-theme="light"]
    assert!(
        html.contains("[data-theme=\"light\"]"),
        "CSS must define [data-theme=\"light\"] selector for light theme overrides"
    );

    // JS must handle both 'dark' and 'light' theme values
    assert!(
        html.contains("'dark'"),
        "JS must reference 'dark' theme value"
    );
    assert!(
        html.contains("'light'"),
        "JS must reference 'light' theme value"
    );

    // Theme toggle must switch between the two values
    assert!(
        html.contains("setAttribute('data-theme'"),
        "JS must set data-theme attribute to switch themes"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Cross-cutting: Keyboard shortcuts
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn keyboard_shortcuts_present() {
    let html = get_monitor_html().await;

    // '?' opens help
    assert!(
        html.contains("e.key === '?'"),
        "JS must handle '?' key for help"
    );
    // Escape closes modal
    assert!(
        html.contains("e.key === 'Escape'"),
        "JS must handle 'Escape' key to close modals"
    );
}
