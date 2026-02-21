use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        Path, Query, State,
    },
    http::StatusCode,
    middleware,
    response::{Html, IntoResponse, Response},
    routing::{get, post},
    Json, Router,
};
use futures::{sink::SinkExt, stream::StreamExt};
use qrcode::{render::unicode, QrCode};
use serde::Deserialize;
use std::net::UdpSocket;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};
use tower_http::cors::{AllowOrigin, CorsLayer};

// Security imports
use crate::security::{security_headers_middleware, AuthService};

/// Validate a path component (session name, filename) to prevent directory traversal.
///
/// Rejects components containing `..`, `/`, `\`, or null bytes.
fn is_safe_path_component(component: &str) -> bool {
    !component.is_empty()
        && !component.contains("..")
        && !component.contains('/')
        && !component.contains('\\')
        && !component.contains('\0')
}

/// Cache for workspace discovery to avoid repeated filesystem scanning
struct WorkspaceCache {
    workspaces: Vec<crate::workspace::DiscoveredWorkspace>,
    last_scan: Instant,
    base_path: Option<std::path::PathBuf>,
}

impl WorkspaceCache {
    fn new() -> Self {
        Self {
            workspaces: Vec::new(),
            last_scan: Instant::now() - Duration::from_secs(1000), // Force initial scan
            base_path: None,
        }
    }

    /// Get workspaces from cache or refresh if stale/different path
    fn get_or_refresh(
        &mut self,
        current: &Option<std::path::PathBuf>,
    ) -> Vec<crate::workspace::DiscoveredWorkspace> {
        const TTL: Duration = Duration::from_secs(300); // 5 minutes

        // Refresh if cache is stale or path changed
        let needs_refresh = self.last_scan.elapsed() > TTL || self.base_path != *current;

        if needs_refresh {
            self.workspaces = crate::workspace::discover_all_workspaces(current);
            self.last_scan = Instant::now();
            self.base_path = current.clone();
        }

        self.workspaces.clone()
    }
}

/// Global workspace cache - shared across all requests (using std::sync::OnceLock)
fn workspace_cache() -> &'static RwLock<WorkspaceCache> {
    static CACHE: std::sync::OnceLock<RwLock<WorkspaceCache>> = std::sync::OnceLock::new();
    CACHE.get_or_init(|| RwLock::new(WorkspaceCache::new()))
}

#[derive(Clone)]
pub struct AppState {
    pub port: u16,
    pub params: Arc<horus_core::RuntimeParams>,
    pub auth_service: Arc<AuthService>,
    pub current_workspace: Option<std::path::PathBuf>,
    pub auth_disabled: bool,
}

/// Get local IP address for network access
fn get_local_ip() -> Option<String> {
    // Create a UDP socket to determine local IP
    // This doesn't actually send data, just connects to determine routing
    let socket = UdpSocket::bind("0.0.0.0:0").ok()?;
    socket.connect("8.8.8.8:80").ok()?;
    socket.local_addr().ok().map(|addr| addr.ip().to_string())
}

/// Generate and display QR code for a URL
fn display_qr_code(url: &str) {
    use colored::Colorize;

    match QrCode::new(url) {
        Ok(code) => {
            let qr_string = code
                .render::<unicode::Dense1x2>()
                .dark_color(unicode::Dense1x2::Light)
                .light_color(unicode::Dense1x2::Dark)
                .build();

            println!(
                "\n   {} Scan QR code with your phone:",
                "[QR CODE]".cyan().bold()
            );
            for line in qr_string.lines() {
                println!("   {}", line);
            }
        }
        Err(_) => {
            // Silently skip QR code if generation fails
        }
    }
}

/// Session validation middleware for monitor using AppState
async fn monitor_session_middleware(
    State(state): State<Arc<AppState>>,
    req: axum::http::Request<axum::body::Body>,
    next: axum::middleware::Next,
) -> Result<axum::response::Response, StatusCode> {
    // Use the session middleware from security module
    crate::security::middleware::session_middleware(State(state.auth_service.clone()), req, next)
        .await
}

/// Run the web monitor server
pub async fn run(port: u16) -> anyhow::Result<()> {
    // Check if password is configured, if not prompt for setup
    let password_hash = if !crate::security::auth::is_password_configured() {
        crate::security::auth::prompt_for_password_setup()?
    } else {
        crate::security::auth::load_password_hash()?
    };

    // Check if authentication is disabled (empty password)
    let auth_disabled = password_hash.is_empty();

    // Initialize authentication service with password
    let auth_service = Arc::new(AuthService::new(password_hash)?);

    let params = Arc::new(
        horus_core::RuntimeParams::init().unwrap_or_else(|_| horus_core::RuntimeParams::default()),
    );

    // Detect current workspace (if running from within a workspace)
    let current_workspace = crate::workspace::find_workspace_root();

    let state = Arc::new(AppState {
        port,
        params,
        auth_service: auth_service.clone(),
        current_workspace: current_workspace.clone(),
        auth_disabled,
    });

    // API routes
    let mut api_routes = Router::new()
        .route("/api/status", get(status_handler))
        .route("/api/nodes", get(nodes_handler))
        .route("/api/topics", get(topics_handler))
        .route("/api/graph", get(graph_handler))
        .route("/api/network", get(network_handler))
        .route("/api/logs/all", get(logs_all_handler))
        .route("/api/logs/node/:name", get(logs_node_handler))
        .route("/api/logs/topic/:name", get(logs_topic_handler))
        .route("/api/packages/registry", get(packages_registry_handler))
        .route(
            "/api/packages/environments",
            get(packages_environments_handler),
        )
        .route("/api/packages/install", post(packages_install_handler))
        .route("/api/packages/uninstall", post(packages_uninstall_handler))
        .route("/api/packages/publish", post(packages_publish_handler))
        .route("/api/params", get(params_list_handler))
        .route("/api/params/:key", get(params_get_handler))
        .route("/api/params/:key", post(params_set_handler))
        .route(
            "/api/params/:key",
            axum::routing::delete(params_delete_handler),
        )
        .route("/api/params/export", post(params_export_handler))
        .route("/api/params/import", post(params_import_handler))
        // Recording API endpoints
        .route("/api/recordings", get(recordings_list_handler))
        .route("/api/recordings/:session", get(recordings_info_handler))
        .route(
            "/api/recordings/:session",
            axum::routing::delete(recordings_delete_handler),
        )
        // Debugging API endpoints
        .route("/api/debug/sessions", get(debug_sessions_list_handler))
        .route("/api/debug/sessions", post(debug_session_create_handler))
        .route("/api/debug/sessions/:id", get(debug_session_get_handler))
        .route(
            "/api/debug/sessions/:id",
            axum::routing::delete(debug_session_delete_handler),
        )
        .route(
            "/api/debug/sessions/:id/breakpoints",
            post(debug_add_breakpoint_handler),
        )
        .route(
            "/api/debug/sessions/:id/breakpoints/:bp_id",
            axum::routing::delete(debug_remove_breakpoint_handler),
        )
        .route(
            "/api/debug/sessions/:id/watches",
            post(debug_add_watch_handler),
        )
        .route(
            "/api/debug/sessions/:id/watches/:watch_id",
            axum::routing::delete(debug_remove_watch_handler),
        )
        .route(
            "/api/debug/sessions/:id/step-forward",
            post(debug_step_forward_handler),
        )
        .route(
            "/api/debug/sessions/:id/step-backward",
            post(debug_step_backward_handler),
        )
        .route(
            "/api/debug/sessions/:id/continue",
            post(debug_continue_handler),
        )
        .route("/api/debug/sessions/:id/pause", post(debug_pause_handler))
        .route("/api/debug/sessions/:id/seek", post(debug_seek_handler))
        .route("/api/debug/sessions/:id/reset", post(debug_reset_handler))
        .route(
            "/api/debug/sessions/:id/snapshot",
            get(debug_snapshot_handler),
        )
        .route(
            "/api/debug/sessions/:id/watches/values",
            get(debug_watches_values_handler),
        )
        .route("/api/ws", get(websocket_handler))
        .route("/api/logout", post(logout_handler));

    // Only add authentication middleware if password is set
    if !auth_disabled {
        api_routes = api_routes.layer(middleware::from_fn_with_state(
            state.clone(),
            monitor_session_middleware,
        ));
    }

    // Public routes (no authentication required)
    let public_routes = Router::new()
        .route("/", get(index_handler))
        .route("/api/login", post(login_handler));

    // Combine all routes
    let app = Router::new()
        .merge(api_routes)
        .merge(public_routes)
        .with_state(state.clone())
        .layer(middleware::from_fn(security_headers_middleware))
        .layer(
            CorsLayer::new()
                .allow_origin(AllowOrigin::predicate(
                    |origin: &axum::http::HeaderValue, _| {
                        // Allow localhost and local network IPs
                        let origin_str = origin.to_str().unwrap_or("");
                        origin_str.starts_with("http://localhost")
                            || origin_str.starts_with("https://localhost")
                            || origin_str.starts_with("http://127.0.0.1")
                            || origin_str.starts_with("https://127.0.0.1")
                            || origin_str.starts_with("http://192.168.")
                            || origin_str.starts_with("https://192.168.")
                            || origin_str.starts_with("http://10.")
                            || origin_str.starts_with("https://10.")
                    },
                ))
                .allow_methods([
                    axum::http::Method::GET,
                    axum::http::Method::POST,
                    axum::http::Method::DELETE,
                ])
                .allow_headers([
                    axum::http::header::AUTHORIZATION,
                    axum::http::header::CONTENT_TYPE,
                ]),
        );

    let addr = format!("0.0.0.0:{}", port);

    use colored::Colorize;

    // Get local IP addresses
    let local_ip = get_local_ip();

    println!("{}", "HORUS Web Monitor is running!".green().bold());

    println!("\n{}:", "Access URLs".cyan().bold());
    println!(
        "   • Local:    {}",
        format!("http://localhost:{}", port).bright_blue()
    );
    if let Some(ref ip) = local_ip {
        let network_url = format!("http://{}:{}", ip, port);
        println!("   • Network:  {}", network_url.bright_blue());

        // Display QR code for network URL
        display_qr_code(&network_url);
    }

    println!("\n{}:", "Security".cyan().bold());
    if auth_disabled {
        println!(
            "   {} {}",
            "⚠".yellow(),
            "Authentication DISABLED - No password required"
                .yellow()
                .bold()
        );
        println!("   • Accessible to anyone on your network");
        println!(
            "   {} To enable password protection: {}",
            "[TIP]".cyan(),
            "horus monitor -r".bright_blue()
        );
    } else {
        println!("   • Password-based authentication with session management");
        println!("   • Accessible from local network devices");
        println!("   • Rate limiting enabled (5 login attempts per minute)");
        println!(
            "   {}",
            "[NOTE] For production use, consider SSH tunneling or reverse proxy with HTTPS"
                .dimmed()
        );
    }

    println!("\n{}:", "Features".cyan().bold());
    println!("   • Real-time node monitoring");
    println!("   • Topic visualization");
    println!("   • Performance metrics");
    println!("   • Package management");
    println!("   • Accessible from any device on your local network");

    // Display workspace context if detected
    if let Some(ref ws_path) = current_workspace {
        let ws_name = ws_path
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("unknown");
        println!("\n{}:", "Workspace Context".cyan().bold());
        println!("   • Running in workspace: {}", ws_name.yellow());
        println!("   • Path: {}", ws_path.display().to_string().dimmed());
    } else {
        println!("\n{}:", "Workspace Context".cyan().bold());
        println!("   • Not running in a workspace (monitoring all HORUS processes)");
    }

    println!("\n   Press {} to stop", "Ctrl+C".bright_red());

    // Start HTTP server
    let listener = tokio::net::TcpListener::bind(&addr).await?;
    axum::serve(listener, app).await?;

    Ok(())
}

// ============================================================================
// Authentication Handlers
// ============================================================================

#[derive(serde::Deserialize)]
pub struct LoginRequest {
    password: String,
}

#[derive(serde::Serialize)]
struct LoginResponse {
    success: bool,
    session_token: Option<String>,
    error: Option<String>,
}

/// Login handler - validates password and creates session
pub async fn login_handler(
    State(state): State<Arc<AppState>>,
    Json(login_req): Json<LoginRequest>,
) -> impl IntoResponse {
    // Extract client IP from connection (for rate limiting)
    // Note: In production, you'd extract this from X-Forwarded-For header
    let ip_address = Some("127.0.0.1".to_string());

    match state.auth_service.login(&login_req.password, ip_address) {
        Ok(Some(token)) => {
            // Set session cookie
            let cookie = format!(
                "session_token={}; Path=/; HttpOnly; SameSite=Strict; Max-Age=3600",
                token
            );

            (
                StatusCode::OK,
                [(axum::http::header::SET_COOKIE, cookie)],
                Json(LoginResponse {
                    success: true,
                    session_token: Some(token),
                    error: None,
                }),
            )
        }
        Ok(None) => {
            // Invalid password
            (
                StatusCode::UNAUTHORIZED,
                [(axum::http::header::SET_COOKIE, String::new())],
                Json(LoginResponse {
                    success: false,
                    session_token: None,
                    error: Some("Invalid password".to_string()),
                }),
            )
        }
        Err(e) => {
            // Rate limited or other error
            (
                StatusCode::TOO_MANY_REQUESTS,
                [(axum::http::header::SET_COOKIE, String::new())],
                Json(LoginResponse {
                    success: false,
                    session_token: None,
                    error: Some(e.to_string()),
                }),
            )
        }
    }
}

#[derive(serde::Deserialize)]
pub struct LogoutRequest {
    pub session_token: String,
}

/// Logout handler - invalidates session
pub async fn logout_handler(
    State(state): State<Arc<AppState>>,
    Json(logout_req): Json<LogoutRequest>,
) -> impl IntoResponse {
    state.auth_service.logout(&logout_req.session_token);

    // Clear session cookie
    let cookie = "session_token=; Path=/; HttpOnly; SameSite=Strict; Max-Age=0";

    (
        StatusCode::OK,
        [(axum::http::header::SET_COOKIE, cookie.to_string())],
        Json(serde_json::json!({ "success": true })),
    )
}

// ============================================================================
// Monitor Handlers
// ============================================================================

async fn index_handler(
    State(state): State<Arc<AppState>>,
    req: axum::http::Request<axum::body::Body>,
) -> Response {
    // If authentication is disabled, go straight to monitor
    let is_authenticated = if state.auth_disabled {
        true // Skip authentication entirely
    } else {
        // Check if user is authenticated by looking for session cookie
        if let Some(cookie_header) = req.headers().get(axum::http::header::COOKIE) {
            if let Ok(cookie_str) = cookie_header.to_str() {
                // Extract session token from cookies
                let token = cookie_str.split(';').find_map(|cookie| {
                    let cookie = cookie.trim();
                    cookie.strip_prefix("session_token=")
                });

                // Validate session if token exists
                if let Some(token) = token {
                    state.auth_service.validate_session(token)
                } else {
                    false
                }
            } else {
                false
            }
        } else {
            false
        }
    };

    if is_authenticated {
        // User is logged in - show monitor
        Html(generate_html(state.port)).into_response()
    } else {
        // User is not logged in - show login page
        Html(generate_login_html()).into_response()
    }
}

pub async fn status_handler(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    use horus_core::core::HealthStatus;

    // Get all nodes and their health
    let nodes = crate::discovery::discover_nodes().unwrap_or_default();
    let nodes_count = nodes.len();

    let topics_count = crate::discovery::discover_shared_memory()
        .map(|t| t.len())
        .unwrap_or(0);

    // Calculate system-wide health by aggregating all node health
    let (system_status, system_health, health_color) = if nodes_count == 0 {
        ("Idle".to_string(), "No nodes running".to_string(), "gray")
    } else {
        // Count nodes by health status
        let mut healthy = 0;
        let mut warning = 0;
        let mut error = 0;
        let mut critical = 0;
        let mut unknown = 0;

        for node in &nodes {
            match node.health {
                HealthStatus::Healthy => healthy += 1,
                HealthStatus::Warning => warning += 1,
                HealthStatus::Error => error += 1,
                HealthStatus::Critical => critical += 1,
                HealthStatus::Unknown => unknown += 1,
            }
        }

        // System health is determined by worst node health
        let (status, color) = if critical > 0 {
            ("Critical", "red")
        } else if error > 0 {
            ("Degraded", "orange")
        } else if warning > 0 {
            ("Warning", "yellow")
        } else if unknown > 0 && healthy == 0 {
            ("Unknown", "gray")
        } else {
            ("Healthy", "green")
        };

        // Build detailed health summary
        let mut details = Vec::new();
        if critical > 0 {
            details.push(format!("{} critical", critical));
        }
        if error > 0 {
            details.push(format!("{} error", error));
        }
        if warning > 0 {
            details.push(format!("{} warning", warning));
        }
        if healthy > 0 {
            details.push(format!("{} healthy", healthy));
        }
        if unknown > 0 {
            details.push(format!("{} unknown", unknown));
        }

        let health_summary = if details.is_empty() {
            format!("{} nodes", nodes_count)
        } else {
            details.join(", ")
        };

        (status.to_string(), health_summary, color)
    };

    // Build workspace info
    let workspace_info = if let Some(ref ws_path) = state.current_workspace {
        let ws_name = ws_path
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("unknown");
        serde_json::json!({
            "name": ws_name,
            "path": ws_path.display().to_string(),
            "detected": true
        })
    } else {
        serde_json::json!({
            "detected": false
        })
    };

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "status": system_status,
            "health": system_health,
            "health_color": health_color,
            "version": "0.1.9",
            "nodes": nodes_count,
            "topics": topics_count,
            "workspace": workspace_info
        })),
    )
        .into_response()
}

pub async fn nodes_handler() -> impl IntoResponse {
    // Use unified backend from monitor module
    let nodes = crate::discovery::discover_nodes()
        .unwrap_or_default()
        .into_iter()
        .map(|n| {
            serde_json::json!({
                "name": n.name,
                "pid": n.process_id,
                "status": n.status,
                "health": n.health.as_str(),
                "health_color": n.health.color(),
                "cpu": format!("{:.1}%", n.cpu_usage),
                "memory": format!("{} MB", n.memory_usage / 1024 / 1024),
                "tick_count": n.tick_count,
                "error_count": n.error_count,
                "tick_rate": n.actual_rate_hz,
                "scheduler_name": n.scheduler_name,
            })
        })
        .collect::<Vec<_>>();

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "nodes": nodes
        })),
    )
        .into_response()
}

pub async fn topics_handler() -> impl IntoResponse {
    // Use unified backend from monitor module
    let topics = crate::discovery::discover_shared_memory()
        .unwrap_or_default()
        .into_iter()
        .map(|t| {
            // Convert IPC name to original format for display
            // Topic names use dot notation - just strip horus_ prefix
            let display_name = t
                .topic_name
                .strip_prefix("horus_")
                .unwrap_or(&t.topic_name)
                .to_string();

            serde_json::json!({
                "name": display_name,
                "size": format!("{} KB", t.size_bytes / 1024),
                "active": t.active,
                "processes": t.accessing_processes.len(),
            })
        })
        .collect::<Vec<_>>();

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "topics": topics
        })),
    )
        .into_response()
}

pub async fn graph_handler() -> impl IntoResponse {
    // Use graph module to get nodes and edges
    let (nodes, edges) = crate::graph::discover_graph_data();

    let graph_nodes = nodes
        .into_iter()
        .map(|n| {
            serde_json::json!({
                "id": n.id,
                "label": n.label,
                "type": match n.node_type {
                    crate::graph::NodeType::Process => "process",
                    crate::graph::NodeType::Topic => "topic",
                },
                "pid": n.pid,
                "active": n.active,
            })
        })
        .collect::<Vec<_>>();

    let graph_edges = edges
        .into_iter()
        .map(|e| {
            serde_json::json!({
                "from": e.from,
                "to": e.to,
                "type": match e.edge_type {
                    crate::graph::EdgeType::Publish => "publish",
                    crate::graph::EdgeType::Subscribe => "subscribe",
                },
                "active": e.active,
            })
        })
        .collect::<Vec<_>>();

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "nodes": graph_nodes,
            "edges": graph_edges
        })),
    )
        .into_response()
}

pub async fn network_handler() -> impl IntoResponse {
    let summary = crate::discovery::get_network_summary();

    let node_statuses = summary
        .node_statuses
        .iter()
        .map(|s| {
            serde_json::json!({
                "node_name": s.node_name,
                "transport_type": s.transport_type,
                "local_endpoint": s.local_endpoint,
                "remote_endpoints": s.remote_endpoints,
                "network_topics_pub": s.network_topics_pub,
                "network_topics_sub": s.network_topics_sub,
                "bytes_sent": s.bytes_sent,
                "bytes_received": s.bytes_received,
                "packets_sent": s.packets_sent,
                "packets_received": s.packets_received,
                "timestamp_secs": s.timestamp_secs,
            })
        })
        .collect::<Vec<_>>();

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "total_nodes": summary.total_nodes,
            "total_bytes_sent": summary.total_bytes_sent,
            "total_bytes_received": summary.total_bytes_received,
            "total_packets_sent": summary.total_packets_sent,
            "total_packets_received": summary.total_packets_received,
            "transport_breakdown": summary.transport_breakdown,
            "unique_endpoints": summary.unique_endpoints,
            "node_statuses": node_statuses,
        })),
    )
        .into_response()
}

pub async fn logs_all_handler() -> impl IntoResponse {
    use horus_core::core::log_buffer::GLOBAL_LOG_BUFFER;

    let logs = GLOBAL_LOG_BUFFER.get_all();

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "logs": logs
        })),
    )
        .into_response()
}

pub async fn logs_node_handler(Path(node_name): Path<String>) -> impl IntoResponse {
    use horus_core::core::log_buffer::GLOBAL_LOG_BUFFER;

    let logs = GLOBAL_LOG_BUFFER.for_node(&node_name);

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "node": node_name,
            "logs": logs
        })),
    )
        .into_response()
}

pub async fn logs_topic_handler(Path(topic_name): Path<String>) -> impl IntoResponse {
    use horus_core::core::log_buffer::GLOBAL_LOG_BUFFER;

    // Topic names use dot notation - just strip horus_ prefix
    let original_topic = topic_name
        .strip_prefix("horus_")
        .unwrap_or(&topic_name)
        .to_string();

    let logs = GLOBAL_LOG_BUFFER.for_topic(&original_topic);

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "topic": topic_name,
            "logs": logs
        })),
    )
        .into_response()
}

// Marketplace handlers
#[derive(serde::Deserialize)]
pub struct SearchQuery {
    pub q: String,
}

// Registry: Search available packages from remote registry
pub async fn packages_registry_handler(Query(query): Query<SearchQuery>) -> impl IntoResponse {
    use crate::registry::RegistryClient;

    let result = tokio::task::spawn_blocking(move || {
        let client = RegistryClient::new();
        client.search(&query.q)
    })
    .await;

    match result {
        Ok(Ok(packages)) => {
            let pkgs = packages
                .into_iter()
                .map(|p| {
                    serde_json::json!({
                        "name": p.name,
                        "version": p.version,
                        "description": p.description.unwrap_or_default(),
                    })
                })
                .collect::<Vec<_>>();

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "packages": pkgs
                })),
            )
                .into_response()
        }
        _ => (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({
                "error": "Failed to search packages"
            })),
        )
            .into_response(),
    }
}

// Environments: Show global packages and local environments
pub async fn packages_environments_handler() -> impl IntoResponse {
    use std::collections::HashSet;
    use std::fs;

    let result = tokio::task::spawn_blocking(move || {
        let mut global_packages_set: HashSet<String> = HashSet::new();
        let mut global_packages = Vec::new();
        let mut local_envs = Vec::new();

        // 1. Global Environment: All packages in ~/.horus/cache (deduplicated)
        if let Some(home) = dirs::home_dir() {
            let global_cache = home.join(".horus/cache");
            if global_cache.exists() {
                if let Ok(entries) = fs::read_dir(&global_cache) {
                    for entry in entries.flatten() {
                        if entry.file_type().map(|t| t.is_dir()).unwrap_or(false) {
                            let name = entry.file_name().to_string_lossy().to_string();

                            // Skip if already added
                            if global_packages_set.contains(&name) {
                                continue;
                            }

                            // Try to read metadata
                            let metadata_path = entry.path().join("metadata.json");
                            let version = if metadata_path.exists() {
                                fs::read_to_string(&metadata_path)
                                    .ok()
                                    .and_then(|s| {
                                        serde_json::from_str::<serde_json::Value>(&s).ok()
                                    })
                                    .and_then(|j| {
                                        j.get("version")
                                            .and_then(|v| v.as_str())
                                            .map(|s| s.to_string())
                                    })
                                    .unwrap_or_else(|| "unknown".to_string())
                            } else {
                                "unknown".to_string()
                            };

                            global_packages_set.insert(name.clone());
                            global_packages.push(serde_json::json!({
                                "name": name,
                                "version": version,
                            }));
                        }
                    }
                }
            }
        }

        // 2. Local Environments: Use cached workspace discovery (5min TTL)
        let current_workspace = crate::workspace::find_workspace_root();

        // Use cache to avoid repeated scanning (16ms → <1ms after first load)
        let discovered_workspaces = workspace_cache()
            .write()
            .unwrap()
            .get_or_refresh(&current_workspace);

        for ws in discovered_workspaces {
            let env_path = ws.path;
            let env_name = ws.name;
            let horus_dir = env_path.join(".horus");

            // Try to read dependencies from horus.yaml
            let horus_yaml_path = env_path.join("horus.yaml");
            let yaml_dependencies = if horus_yaml_path.exists() {
                fs::read_to_string(&horus_yaml_path)
                    .ok()
                    .and_then(|content| serde_yaml::from_str::<serde_yaml::Value>(&content).ok())
                    .and_then(|yaml| {
                        yaml.get("dependencies")
                            .and_then(|deps| deps.as_sequence())
                            .map(|seq| {
                                seq.iter()
                                    .filter_map(|v| v.as_str().map(|s| s.to_string()))
                                    .collect::<Vec<String>>()
                            })
                    })
                    .unwrap_or_default()
            } else {
                Vec::new()
            };

            // Get packages inside this environment (deduplicated)
            let packages_dir = horus_dir.join("packages");
            let mut packages = Vec::new();
            let mut local_packages_set: HashSet<String> = HashSet::new();

            if packages_dir.exists() {
                if let Ok(pkg_entries) = fs::read_dir(&packages_dir) {
                    for pkg_entry in pkg_entries.flatten() {
                        // Check if it's a directory OR a symlink (include broken symlinks to show them)
                        let file_type = pkg_entry.file_type().ok();
                        let is_dir = file_type.as_ref().map(|t| t.is_dir()).unwrap_or(false);
                        let is_symlink =
                            file_type.as_ref().map(|t| t.is_symlink()).unwrap_or(false);
                        let is_pkg_entry = is_dir || is_symlink;

                        if is_pkg_entry {
                            let pkg_name = pkg_entry.file_name().to_string_lossy().to_string();
                            let pkg_path = pkg_entry.path();

                            // Skip if already added
                            if local_packages_set.contains(&pkg_name) {
                                continue;
                            }

                            // Check if symlink target exists (for broken symlink detection)
                            let symlink_broken = is_symlink && !pkg_path.exists();

                            // Try to get version from metadata.json (follow symlinks if valid)
                            let metadata_path = pkg_path.join("metadata.json");
                            let version = if symlink_broken {
                                "broken link".to_string()
                            } else if metadata_path.exists() {
                                fs::read_to_string(&metadata_path)
                                    .ok()
                                    .and_then(|s| {
                                        serde_json::from_str::<serde_json::Value>(&s).ok()
                                    })
                                    .and_then(|j| {
                                        j.get("version")
                                            .and_then(|v| v.as_str())
                                            .map(|s| s.to_string())
                                    })
                                    .unwrap_or_else(|| "unknown".to_string())
                            } else {
                                "unknown".to_string()
                            };

                            local_packages_set.insert(pkg_name.clone());
                            packages.push(serde_json::json!({
                                "name": pkg_name,
                                "version": version,
                            }));
                        }
                    }
                }
            }

            // workspace_dependencies: Only list dependencies declared in horus.yaml
            // that are not already in the packages list (to avoid duplication)
            let workspace_dependencies: Vec<serde_json::Value> = yaml_dependencies
                .iter()
                .filter_map(|dep_str| {
                    let dep_name = dep_str.split('@').next().unwrap_or(dep_str);

                    // Skip if already in packages list
                    if local_packages_set.contains(dep_name) {
                        return None;
                    }

                    // dependency format: "package_name@version" or just "package_name"
                    let dep_path = packages_dir.join(dep_str);
                    if dep_path.exists() || dep_path.read_link().is_ok() {
                        // Try to read metadata.json (follow symlinks)
                        let mut metadata_path = dep_path.join("metadata.json");

                        // Follow symlink if necessary
                        if let Ok(real_path) = std::fs::canonicalize(&dep_path) {
                            metadata_path = real_path.join("metadata.json");
                        }

                        let version = if metadata_path.exists() {
                            fs::read_to_string(&metadata_path)
                                .ok()
                                .and_then(|s| serde_json::from_str::<serde_json::Value>(&s).ok())
                                .and_then(|j| {
                                    j.get("version")
                                        .and_then(|v| v.as_str())
                                        .map(|s| s.to_string())
                                })
                                .unwrap_or_else(|| {
                                    dep_str.split('@').nth(1).unwrap_or("unknown").to_string()
                                })
                        } else {
                            dep_str.split('@').nth(1).unwrap_or("unknown").to_string()
                        };

                        Some(serde_json::json!({
                            "name": dep_name,
                            "version": version,
                        }))
                    } else {
                        None
                    }
                })
                .collect();

            local_envs.push(serde_json::json!({
                "name": env_name,
                "path": env_path.to_string_lossy(),
                "packages": packages,
                "package_count": packages.len(),
                "dependencies": workspace_dependencies,
            }));
        }

        serde_json::json!({
            "global": global_packages,
            "local": local_envs
        })
    })
    .await;

    match result {
        Ok(data) => (StatusCode::OK, Json(data)).into_response(),
        Err(_) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({
                "error": "Failed to list environments"
            })),
        )
            .into_response(),
    }
}

#[derive(serde::Deserialize)]
pub struct InstallRequest {
    pub package: String,
    #[serde(default)]
    pub target: Option<String>,
}

pub async fn packages_install_handler(Json(req): Json<InstallRequest>) -> impl IntoResponse {
    use crate::registry::RegistryClient;
    use std::path::PathBuf;

    // Validate package name to prevent directory traversal
    if !is_safe_path_component(&req.package) {
        return (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "error": "Invalid package name"
            })),
        )
            .into_response();
    }

    let package_name = req.package.clone();
    let target = req.target.clone();

    let result = tokio::task::spawn_blocking(move || -> anyhow::Result<(String, String)> {
        let client = RegistryClient::new();

        // Determine target based on input and horus.yaml path
        let (horus_yaml_path, version) = if let Some(target_str) = &target {
            if target_str == "global" {
                // Install globally - no horus.yaml to update
                let version = client.install_to_target(
                    &req.package,
                    None,
                    crate::workspace::InstallTarget::Global,
                )?;
                (None, version)
            } else {
                // Use specified path - find horus.yaml in parent package
                let target_path = PathBuf::from(target_str);

                // Extract parent package path (remove /.horus/packages/package_name)
                // target_path format: /path/to/project/.horus/packages/parent_package
                let parent_path = if target_path.ends_with(".horus/packages") {
                    target_path.parent().and_then(|p| p.parent())
                } else {
                    // Likely: /path/.horus/packages/parent_package
                    target_path
                        .parent()
                        .and_then(|p| p.parent()) // Remove parent_package
                        .and_then(|p| p.parent()) // Remove packages
                        .and_then(|p| p.parent()) // Remove .horus
                };

                let yaml_path = parent_path.map(|p| p.join("horus.yaml"));

                let version = client.install_to_target(
                    &req.package,
                    None,
                    crate::workspace::InstallTarget::Local(target_path),
                )?;
                (yaml_path, version)
            }
        } else {
            // Default: auto-detect - look for horus.yaml in current dir
            let yaml_path = PathBuf::from("horus.yaml");
            let yaml_path = if yaml_path.exists() {
                Some(yaml_path)
            } else {
                None
            };
            let version = client.install(&req.package, None)?;
            (yaml_path, version)
        };

        // Update horus.yaml if path exists
        if let Some(yaml_path) = horus_yaml_path {
            if yaml_path.exists() {
                crate::yaml_utils::add_dependency_to_horus_yaml(
                    &yaml_path,
                    &req.package,
                    &version,
                )?;
            }
        }

        Ok((req.package.clone(), version))
    })
    .await;

    match result {
        Ok(Ok(_)) => (
            StatusCode::OK,
            Json(serde_json::json!({
                "success": true,
                "message": format!("Successfully installed {}", package_name)
            })),
        )
            .into_response(),
        Ok(Err(e)) => (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "success": false,
                "error": e.to_string()
            })),
        )
            .into_response(),
        Err(e) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({
                "success": false,
                "error": format!("Task failed: {}", e)
            })),
        )
            .into_response(),
    }
}

#[derive(serde::Deserialize)]
pub struct UninstallRequest {
    pub parent_package: String,
    pub package: String,
}

pub async fn packages_uninstall_handler(Json(req): Json<UninstallRequest>) -> impl IntoResponse {
    use std::fs;
    use std::path::PathBuf;

    // Validate package names to prevent directory traversal
    if !is_safe_path_component(&req.parent_package) || !is_safe_path_component(&req.package) {
        return (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "error": "Invalid package name"
            })),
        )
            .into_response();
    }

    let parent_package = req.parent_package.clone();
    let package = req.package.clone();
    let parent_package_msg = parent_package.clone();
    let package_msg = package.clone();

    let result = tokio::task::spawn_blocking(move || -> anyhow::Result<()> {
        // Find the parent package in local environments
        let search_paths = vec![PathBuf::from("."), dirs::home_dir().unwrap_or_default()];

        for base_path in search_paths {
            if !base_path.exists() {
                continue;
            }

            if let Ok(entries) = fs::read_dir(&base_path) {
                for entry in entries.flatten() {
                    if !entry.file_type().map(|t| t.is_dir()).unwrap_or(false) {
                        continue;
                    }

                    let horus_dir = entry.path().join(".horus");
                    if !horus_dir.exists() {
                        continue;
                    }

                    // Check if this environment has the parent package
                    let parent_pkg_path = horus_dir.join("packages").join(&parent_package);
                    if !parent_pkg_path.exists() {
                        continue;
                    }

                    // Found the parent package, now uninstall the nested package
                    let nested_pkg_path = parent_pkg_path.join(".horus/packages").join(&package);
                    if nested_pkg_path.exists() {
                        fs::remove_dir_all(&nested_pkg_path)?;

                        // Update horus.yaml of the parent package
                        // The parent package directory structure is: <project_root>/.horus/packages/<parent_package>
                        // We need to go up to the project root and find horus.yaml
                        let project_root = parent_pkg_path.parent().and_then(|p| p.parent());
                        if let Some(root) = project_root {
                            let horus_yaml_path = root.join("horus.yaml");
                            if horus_yaml_path.exists() {
                                // Ignore errors in updating horus.yaml - package is already uninstalled
                                let _ = crate::yaml_utils::remove_dependency_from_horus_yaml(
                                    &horus_yaml_path,
                                    &package,
                                );
                            }
                        }

                        return Ok(());
                    }
                }
            }
        }

        anyhow::bail!("Package not found")
    })
    .await;

    match result {
        Ok(Ok(_)) => (
            StatusCode::OK,
            Json(serde_json::json!({
                "success": true,
                "message": format!("Successfully uninstalled {} from {}", package_msg, parent_package_msg)
            }))
        ).into_response(),
        Ok(Err(e)) => (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "success": false,
                "error": e.to_string()
            }))
        ).into_response(),
        Err(e) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({
                "success": false,
                "error": format!("Task failed: {}", e)
            }))
        ).into_response(),
    }
}

pub async fn packages_publish_handler() -> impl IntoResponse {
    use crate::registry::RegistryClient;

    let result = tokio::task::spawn_blocking(move || {
        let client = RegistryClient::new();
        client.publish(None, false, None)
    })
    .await;

    match result {
        Ok(Ok(_)) => (
            StatusCode::OK,
            Json(serde_json::json!({
                "success": true,
                "message": "Package published successfully"
            })),
        )
            .into_response(),
        Ok(Err(e)) => (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "success": false,
                "error": e.to_string()
            })),
        )
            .into_response(),
        Err(e) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({
                "success": false,
                "error": format!("Task failed: {}", e)
            })),
        )
            .into_response(),
    }
}

// === Parameter Management Handlers ===

/// List all parameters
pub async fn params_list_handler(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let params_map = state.params.get_all();

    let params_list: Vec<_> = params_map
        .iter()
        .map(|(key, value)| {
            // Determine type using accessor methods to avoid version conflicts
            let type_str = if value.is_number() {
                "number"
            } else if value.is_string() {
                "string"
            } else if value.is_boolean() {
                "boolean"
            } else if value.is_array() {
                "array"
            } else if value.is_object() {
                "object"
            } else {
                "null"
            };

            serde_json::json!({
                "key": key,
                "value": value,
                "type": type_str,
                "version": state.params.get_version(key)
            })
        })
        .collect();

    (
        StatusCode::OK,
        serde_json::json!({
            "success": true,
            "params": params_list,
            "count": params_list.len()
        })
        .to_string(),
    )
}

/// Get a specific parameter
pub async fn params_get_handler(
    State(state): State<Arc<AppState>>,
    Path(key): Path<String>,
) -> impl IntoResponse {
    match state.params.get::<serde_json::Value>(&key) {
        Some(value) => (
            StatusCode::OK,
            serde_json::json!({
                "success": true,
                "key": key,
                "value": value,
                "version": state.params.get_version(&key)
            })
            .to_string(),
        ),
        None => (
            StatusCode::NOT_FOUND,
            serde_json::json!({
                "success": false,
                "error": format!("Parameter '{}' not found", key)
            })
            .to_string(),
        ),
    }
}

#[derive(serde::Deserialize)]
pub struct SetParamRequest {
    pub value: serde_json::Value,
    pub version: Option<u64>, // Optional version for optimistic locking
}

/// Set a parameter
pub async fn params_set_handler(
    State(state): State<Arc<AppState>>,
    Path(key): Path<String>,
    Json(req): Json<SetParamRequest>,
) -> impl IntoResponse {
    // Use version-aware set if version is provided (optimistic locking)
    let result = if let Some(expected_version) = req.version {
        state
            .params
            .set_with_version(&key, req.value.clone(), expected_version)
    } else {
        state.params.set(&key, req.value.clone())
    };

    match result {
        Ok(_) => {
            // Save to disk
            let _ = state.params.save_to_disk();

            let new_version = state.params.get_version(&key);

            (
                StatusCode::OK,
                serde_json::json!({
                    "success": true,
                    "message": format!("Parameter '{}' updated", key),
                    "key": key,
                    "value": req.value,
                    "version": new_version
                })
                .to_string(),
            )
        }
        Err(e) => {
            // Check if it's a version mismatch error
            let status_code = if e.to_string().contains("Version mismatch") {
                StatusCode::CONFLICT // 409 Conflict for version mismatch
            } else {
                StatusCode::INTERNAL_SERVER_ERROR
            };

            (
                status_code,
                serde_json::json!({
                    "success": false,
                    "error": e.to_string()
                })
                .to_string(),
            )
        }
    }
}

/// Delete a parameter
pub async fn params_delete_handler(
    State(state): State<Arc<AppState>>,
    Path(key): Path<String>,
) -> impl IntoResponse {
    match state.params.remove(&key) {
        Some(old_value) => {
            // Save to disk
            let _ = state.params.save_to_disk();

            (
                StatusCode::OK,
                serde_json::json!({
                    "success": true,
                    "message": format!("Parameter '{}' deleted", key),
                    "key": key,
                    "old_value": old_value
                })
                .to_string(),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            serde_json::json!({
                "success": false,
                "error": format!("Parameter '{}' not found", key)
            })
            .to_string(),
        ),
    }
}

/// Export all parameters
pub async fn params_export_handler(State(state): State<Arc<AppState>>) -> impl IntoResponse {
    let params = state.params.get_all();

    match serde_yaml::to_string(&params) {
        Ok(yaml) => (
            StatusCode::OK,
            serde_json::json!({
                "success": true,
                "format": "yaml",
                "data": yaml
            })
            .to_string(),
        ),
        Err(e) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            serde_json::json!({
                "success": false,
                "error": e.to_string()
            })
            .to_string(),
        ),
    }
}

#[derive(serde::Deserialize)]
pub struct ImportParamsRequest {
    pub data: String,
    pub format: String, // "yaml" or "json"
}

/// Import parameters
pub async fn params_import_handler(
    State(state): State<Arc<AppState>>,
    Json(req): Json<ImportParamsRequest>,
) -> impl IntoResponse {
    let import_result: Result<
        std::collections::BTreeMap<String, serde_json::Value>,
        Box<dyn std::error::Error>,
    > =
        match req.format.as_str() {
            "yaml" => serde_yaml::from_str(&req.data)
                .map_err(|e| Box::new(e) as Box<dyn std::error::Error>),
            "json" => serde_json::from_str(&req.data)
                .map_err(|e| Box::new(e) as Box<dyn std::error::Error>),
            _ => {
                return (
                    StatusCode::BAD_REQUEST,
                    serde_json::json!({
                        "success": false,
                        "error": "Invalid format. Use 'yaml' or 'json'"
                    })
                    .to_string(),
                );
            }
        };

    match import_result {
        Ok(params_map) => {
            let mut count = 0;
            for (key, value) in params_map {
                if state.params.set(&key, value).is_ok() {
                    count += 1;
                }
            }

            // Save to disk
            let _ = state.params.save_to_disk();

            (
                StatusCode::OK,
                serde_json::json!({
                    "success": true,
                    "message": format!("Imported {} parameters", count),
                    "count": count
                })
                .to_string(),
            )
        }
        Err(e) => (
            StatusCode::BAD_REQUEST,
            serde_json::json!({
                "success": false,
                "error": format!("Failed to parse {}: {}", req.format, e)
            })
            .to_string(),
        ),
    }
}

// ============================================================================
// Recording API Handlers
// ============================================================================

/// Response structure for recording list
#[derive(serde::Serialize)]
struct RecordingListItem {
    session_name: String,
    started_at: String,
    ended_at: Option<String>,
    total_ticks: u64,
    node_count: usize,
    size_bytes: u64,
}

/// List all recordings
pub async fn recordings_list_handler() -> impl IntoResponse {
    let recordings_dir = dirs::home_dir()
        .unwrap_or_else(|| std::path::PathBuf::from("."))
        .join(".horus/recordings");

    if !recordings_dir.exists() {
        return (
            StatusCode::OK,
            Json(serde_json::json!({
                "recordings": [],
                "count": 0
            })),
        );
    }

    let mut recordings = Vec::new();

    // Read all session directories
    if let Ok(entries) = std::fs::read_dir(&recordings_dir) {
        for entry in entries.flatten() {
            if entry.file_type().map(|t| t.is_dir()).unwrap_or(false) {
                let session_name = entry.file_name().to_string_lossy().to_string();
                let session_dir = entry.path();

                // Find scheduler recording file
                let scheduler_file = session_dir
                    .read_dir()
                    .ok()
                    .and_then(|mut d| {
                        d.find(|e| {
                            e.as_ref()
                                .map(|e| e.file_name().to_string_lossy().starts_with("scheduler@"))
                                .unwrap_or(false)
                        })
                    })
                    .and_then(|r| r.ok());

                // Count node recordings
                let node_count = session_dir
                    .read_dir()
                    .map(|d| {
                        d.filter(|e| {
                            e.as_ref()
                                .map(|e| {
                                    let name = e.file_name();
                                    let name_str = name.to_string_lossy();
                                    name_str.ends_with(".horus")
                                        && !name_str.starts_with("scheduler@")
                                })
                                .unwrap_or(false)
                        })
                        .count()
                    })
                    .unwrap_or(0);

                // Calculate total size
                let size_bytes = session_dir
                    .read_dir()
                    .map(|d| {
                        d.filter_map(|e| e.ok())
                            .filter_map(|e| e.metadata().ok())
                            .map(|m| m.len())
                            .sum()
                    })
                    .unwrap_or(0u64);

                // Try to load scheduler recording for timing info
                let (started_at, ended_at, total_ticks) = if let Some(sf) = scheduler_file {
                    use horus_core::scheduling::SchedulerRecording;
                    if let Ok(sr) = SchedulerRecording::load(&sf.path()) {
                        let started = chrono::DateTime::from_timestamp_micros(sr.started_at as i64)
                            .map(|dt| dt.format("%Y-%m-%d %H:%M:%S").to_string())
                            .unwrap_or_else(|| "Unknown".to_string());
                        let ended = sr.ended_at.and_then(|e| {
                            chrono::DateTime::from_timestamp_micros(e as i64)
                                .map(|dt| dt.format("%Y-%m-%d %H:%M:%S").to_string())
                        });
                        (started, ended, sr.total_ticks)
                    } else {
                        ("Unknown".to_string(), None, 0)
                    }
                } else {
                    // Fallback to directory modification time
                    let started = entry
                        .metadata()
                        .ok()
                        .and_then(|m| m.created().ok())
                        .map(|t| {
                            chrono::DateTime::<chrono::Utc>::from(t)
                                .format("%Y-%m-%d %H:%M:%S")
                                .to_string()
                        })
                        .unwrap_or_else(|| "Unknown".to_string());
                    (started, None, 0)
                };

                recordings.push(RecordingListItem {
                    session_name,
                    started_at,
                    ended_at,
                    total_ticks,
                    node_count,
                    size_bytes,
                });
            }
        }
    }

    // Sort by started_at descending (newest first)
    recordings.sort_by(|a, b| b.started_at.cmp(&a.started_at));

    let count = recordings.len();

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "recordings": recordings,
            "count": count
        })),
    )
}

/// Get detailed info about a specific recording
pub async fn recordings_info_handler(Path(session): Path<String>) -> impl IntoResponse {
    if !is_safe_path_component(&session) {
        return (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "error": "Invalid session name"
            })),
        );
    }

    let recordings_dir = dirs::home_dir()
        .unwrap_or_else(|| std::path::PathBuf::from("."))
        .join(".horus/recordings")
        .join(&session);

    if !recordings_dir.exists() {
        return (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Recording session '{}' not found", session)
            })),
        );
    }

    // Find and load scheduler recording
    let scheduler_file = recordings_dir
        .read_dir()
        .ok()
        .and_then(|mut d| {
            d.find(|e| {
                e.as_ref()
                    .map(|e| e.file_name().to_string_lossy().starts_with("scheduler@"))
                    .unwrap_or(false)
            })
        })
        .and_then(|r| r.ok());

    // List node recordings
    let mut node_recordings: Vec<serde_json::Value> = recordings_dir
        .read_dir()
        .map(|d| {
            d.filter_map(|e| e.ok())
                .filter(|e| {
                    let name = e.file_name();
                    let name_str = name.to_string_lossy();
                    name_str.ends_with(".horus") && !name_str.starts_with("scheduler@")
                })
                .map(|e| {
                    let size = e.metadata().map(|m| m.len()).unwrap_or(0);
                    let filename = e.file_name().to_string_lossy().to_string();
                    // Parse node name from filename: "NodeName@id.horus"
                    let node_name = filename.split('@').next().unwrap_or(&filename).to_string();
                    serde_json::json!({
                        "filename": filename,
                        "node_name": node_name,
                        "size_bytes": size,
                        "size_human": format_size(size)
                    })
                })
                .collect()
        })
        .unwrap_or_default();

    node_recordings.sort_by(|a, b| a["node_name"].as_str().cmp(&b["node_name"].as_str()));

    // Load scheduler info if available
    let scheduler_info = if let Some(sf) = scheduler_file {
        use horus_core::scheduling::SchedulerRecording;
        if let Ok(sr) = SchedulerRecording::load(&sf.path()) {
            let started = chrono::DateTime::from_timestamp_micros(sr.started_at as i64)
                .map(|dt| dt.format("%Y-%m-%d %H:%M:%S UTC").to_string())
                .unwrap_or_else(|| "Unknown".to_string());
            let ended = sr.ended_at.and_then(|e| {
                chrono::DateTime::from_timestamp_micros(e as i64)
                    .map(|dt| dt.format("%Y-%m-%d %H:%M:%S UTC").to_string())
            });
            let duration_secs = sr
                .ended_at
                .map(|e| (e - sr.started_at) / 1_000_000)
                .unwrap_or(0);

            serde_json::json!({
                "scheduler_id": sr.scheduler_id,
                "session_name": sr.session_name,
                "started_at": started,
                "ended_at": ended,
                "duration_secs": duration_secs,
                "total_ticks": sr.total_ticks,
                "execution_order_ticks": sr.execution_order.len(),
                "config": sr.config
            })
        } else {
            serde_json::json!(null)
        }
    } else {
        serde_json::json!(null)
    };

    // Calculate total size
    let total_size: u64 = recordings_dir
        .read_dir()
        .map(|d| {
            d.filter_map(|e| e.ok())
                .filter_map(|e| e.metadata().ok())
                .map(|m| m.len())
                .sum()
        })
        .unwrap_or(0);

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "session_name": session,
            "path": recordings_dir.to_string_lossy(),
            "scheduler": scheduler_info,
            "node_recordings": node_recordings,
            "total_size_bytes": total_size,
            "total_size_human": format_size(total_size)
        })),
    )
}

/// Delete a recording session
pub async fn recordings_delete_handler(Path(session): Path<String>) -> impl IntoResponse {
    if !is_safe_path_component(&session) {
        return (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "error": "Invalid session name"
            })),
        );
    }

    let recordings_dir = dirs::home_dir()
        .unwrap_or_else(|| std::path::PathBuf::from("."))
        .join(".horus/recordings")
        .join(&session);

    if !recordings_dir.exists() {
        return (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Recording session '{}' not found", session)
            })),
        );
    }

    match std::fs::remove_dir_all(&recordings_dir) {
        Ok(_) => (
            StatusCode::OK,
            Json(serde_json::json!({
                "success": true,
                "message": format!("Deleted recording session '{}'", session)
            })),
        ),
        Err(e) => (
            StatusCode::INTERNAL_SERVER_ERROR,
            Json(serde_json::json!({
                "error": format!("Failed to delete recording: {}", e)
            })),
        ),
    }
}

/// Format bytes to human-readable size
fn format_size(bytes: u64) -> String {
    const KB: u64 = 1024;
    const MB: u64 = KB * 1024;
    const GB: u64 = MB * 1024;

    if bytes >= GB {
        format!("{:.2} GB", bytes as f64 / GB as f64)
    } else if bytes >= MB {
        format!("{:.2} MB", bytes as f64 / MB as f64)
    } else if bytes >= KB {
        format!("{:.2} KB", bytes as f64 / KB as f64)
    } else {
        format!("{} B", bytes)
    }
}

// ============================================================================
// Debug Session API Handlers
// ============================================================================

use horus_core::scheduling::{
    BreakpointCondition, DebugSessionState, NodeRecording, ReplayDebugger, WatchExpression,
    WatchType,
};
use std::sync::Mutex;

lazy_static::lazy_static! {
    /// Global debug sessions storage
    static ref DEBUG_SESSIONS: Mutex<std::collections::HashMap<String, DebugSessionData>> =
        Mutex::new(std::collections::HashMap::new());
}

/// Debug session data with debugger and state
struct DebugSessionData {
    debugger: ReplayDebugger,
    state: DebugSessionState,
}

/// Request body for creating a debug session
#[derive(Debug, Deserialize)]
pub struct CreateDebugSessionRequest {
    /// Recording session name
    recording_session: String,
    /// Node recording filename within the session
    recording_file: String,
    /// Optional debug session name
    session_name: Option<String>,
}

/// Request body for adding a breakpoint
#[derive(Debug, Deserialize)]
pub struct AddBreakpointRequest {
    /// Breakpoint type
    breakpoint_type: String,
    /// Tick number (for tick-based breakpoints)
    tick: Option<u64>,
    /// Topic name (for topic-based breakpoints)
    topic: Option<String>,
    /// Pattern bytes (for pattern matching)
    pattern: Option<Vec<u8>>,
    /// Optional name
    name: Option<String>,
}

/// Request body for adding a watch expression
#[derive(Debug, Deserialize)]
pub struct AddWatchRequest {
    /// Watch ID
    id: String,
    /// Watch name
    name: String,
    /// Topic to watch
    topic: String,
    /// Watch type: "input" or "output"
    watch_type: String,
    /// Optional byte offset
    byte_offset: Option<usize>,
    /// Optional byte length
    byte_length: Option<usize>,
}

/// Request body for seeking
#[derive(Debug, Deserialize)]
pub struct SeekRequest {
    tick: u64,
}

/// List all debug sessions
pub async fn debug_sessions_list_handler() -> impl IntoResponse {
    let sessions = DEBUG_SESSIONS.lock().unwrap();

    let session_list: Vec<serde_json::Value> = sessions
        .iter()
        .map(|(id, data)| {
            serde_json::json!({
                "id": id,
                "session_name": data.state.session_name,
                "recording_path": data.state.recording_path.to_string_lossy(),
                "current_tick": data.state.current_tick,
                "breakpoint_count": data.state.breakpoints.len(),
                "watch_count": data.state.watches.len(),
                "state": format!("{:?}", data.debugger.state()),
                "created_at": data.state.created_at,
                "updated_at": data.state.updated_at,
            })
        })
        .collect();

    (
        StatusCode::OK,
        Json(serde_json::json!({
            "sessions": session_list,
            "count": session_list.len()
        })),
    )
}

/// Create a new debug session
pub async fn debug_session_create_handler(
    Json(req): Json<CreateDebugSessionRequest>,
) -> impl IntoResponse {
    if !is_safe_path_component(&req.recording_session)
        || !is_safe_path_component(&req.recording_file)
    {
        return (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({
                "error": "Invalid recording session or file name"
            })),
        );
    }

    let recordings_dir = dirs::home_dir()
        .unwrap_or_else(|| std::path::PathBuf::from("."))
        .join(".horus/recordings")
        .join(&req.recording_session);

    let recording_path = recordings_dir.join(&req.recording_file);

    if !recording_path.exists() {
        return (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Recording file not found: {}", recording_path.display())
            })),
        );
    }

    // Load the recording and create a debugger
    let recording = match NodeRecording::load(&recording_path) {
        Ok(r) => r,
        Err(e) => {
            return (
                StatusCode::INTERNAL_SERVER_ERROR,
                Json(serde_json::json!({
                    "error": format!("Failed to load recording: {}", e)
                })),
            );
        }
    };

    let replayer = horus_core::scheduling::NodeReplayer::from_recording(recording);
    let debugger = ReplayDebugger::new(replayer);

    let session_name = req.session_name.unwrap_or_else(|| {
        format!(
            "debug_{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs()
        )
    });

    let state = DebugSessionState::new(recording_path.clone(), &session_name);

    // Generate unique ID
    let session_id = format!(
        "{:x}",
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos()
    );

    let mut sessions = DEBUG_SESSIONS.lock().unwrap();
    sessions.insert(session_id.clone(), DebugSessionData { debugger, state });

    (
        StatusCode::CREATED,
        Json(serde_json::json!({
            "id": session_id,
            "session_name": session_name,
            "recording_path": recording_path.to_string_lossy(),
            "message": "Debug session created"
        })),
    )
}

/// Get debug session details
pub async fn debug_session_get_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let sessions = DEBUG_SESSIONS.lock().unwrap();

    match sessions.get(&session_id) {
        Some(data) => {
            let recording = data.debugger.recording();
            let current_snapshot = data.debugger.current_snapshot();

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "id": session_id,
                    "session_name": data.state.session_name,
                    "recording_path": data.state.recording_path.to_string_lossy(),
                    "state": format!("{:?}", data.debugger.state()),
                    "current_tick": data.debugger.current_tick(),
                    "total_ticks": recording.snapshots.len(),
                    "first_tick": recording.first_tick,
                    "last_tick": recording.last_tick,
                    "node_name": recording.node_name,
                    "node_id": recording.node_id,
                    "breakpoints": data.debugger.breakpoints().iter().map(|bp| {
                        serde_json::json!({
                            "id": bp.id,
                            "condition": format!("{:?}", bp.condition),
                            "enabled": bp.enabled,
                            "hit_count": bp.hit_count,
                            "name": bp.name
                        })
                    }).collect::<Vec<_>>(),
                    "watches": data.debugger.watches().iter().map(|w| {
                        serde_json::json!({
                            "id": w.id,
                            "name": w.name,
                            "topic": w.topic,
                            "watch_type": format!("{:?}", w.watch_type),
                            "byte_offset": w.byte_offset,
                            "byte_length": w.byte_length
                        })
                    }).collect::<Vec<_>>(),
                    "current_snapshot": current_snapshot.map(|s| {
                        serde_json::json!({
                            "tick": s.tick,
                            "timestamp_us": s.timestamp_us,
                            "duration_ns": s.duration_ns,
                            "input_topics": s.inputs.keys().collect::<Vec<_>>(),
                            "output_topics": s.outputs.keys().collect::<Vec<_>>(),
                            "has_state": s.state.is_some()
                        })
                    }),
                    "recent_events": data.debugger.recent_events(10).iter().map(|e| {
                        format!("{:?}", e)
                    }).collect::<Vec<_>>()
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Delete a debug session
pub async fn debug_session_delete_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().unwrap();

    match sessions.remove(&session_id) {
        Some(_) => (
            StatusCode::OK,
            Json(serde_json::json!({
                "success": true,
                "message": format!("Debug session '{}' deleted", session_id)
            })),
        ),
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Add a breakpoint to a debug session
pub async fn debug_add_breakpoint_handler(
    Path(session_id): Path<String>,
    Json(req): Json<AddBreakpointRequest>,
) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().unwrap();

    match sessions.get_mut(&session_id) {
        Some(data) => {
            let condition = match req.breakpoint_type.as_str() {
                "at_tick" => {
                    let tick = req.tick.unwrap_or(0);
                    BreakpointCondition::AtTick(tick)
                }
                "topic_has_data" => {
                    let topic = req.topic.unwrap_or_default();
                    BreakpointCondition::TopicHasData(topic)
                }
                "output_matches" => {
                    let topic = req.topic.unwrap_or_default();
                    let pattern = req.pattern.unwrap_or_default();
                    BreakpointCondition::OutputMatches { topic, pattern }
                }
                "on_error" => BreakpointCondition::OnError,
                "after_ticks" => {
                    let n = req.tick.unwrap_or(1);
                    BreakpointCondition::AfterTicks(n)
                }
                _ => {
                    return (
                        StatusCode::BAD_REQUEST,
                        Json(serde_json::json!({
                            "error": format!("Unknown breakpoint type: {}", req.breakpoint_type)
                        })),
                    );
                }
            };

            let bp_id = if let Some(name) = req.name {
                data.debugger.add_named_breakpoint(&name, condition)
            } else {
                data.debugger.add_breakpoint(condition)
            };

            // Update state
            data.state.update_from_debugger(&data.debugger);

            (
                StatusCode::CREATED,
                Json(serde_json::json!({
                    "success": true,
                    "breakpoint_id": bp_id,
                    "message": "Breakpoint added"
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Remove a breakpoint from a debug session
pub async fn debug_remove_breakpoint_handler(
    Path((session_id, bp_id)): Path<(String, u32)>,
) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().unwrap();

    match sessions.get_mut(&session_id) {
        Some(data) => {
            if data.debugger.remove_breakpoint(bp_id) {
                data.state.update_from_debugger(&data.debugger);
                (
                    StatusCode::OK,
                    Json(serde_json::json!({
                        "success": true,
                        "message": format!("Breakpoint {} removed", bp_id)
                    })),
                )
            } else {
                (
                    StatusCode::NOT_FOUND,
                    Json(serde_json::json!({
                        "error": format!("Breakpoint {} not found", bp_id)
                    })),
                )
            }
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Add a watch expression to a debug session
pub async fn debug_add_watch_handler(
    Path(session_id): Path<String>,
    Json(req): Json<AddWatchRequest>,
) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().unwrap();

    match sessions.get_mut(&session_id) {
        Some(data) => {
            let watch_type = match req.watch_type.as_str() {
                "input" => WatchType::Input,
                "output" => WatchType::Output,
                _ => {
                    return (
                        StatusCode::BAD_REQUEST,
                        Json(serde_json::json!({
                            "error": format!("Unknown watch type: {}", req.watch_type)
                        })),
                    );
                }
            };

            let mut watch = WatchExpression::new(&req.id, &req.name, &req.topic, watch_type);

            if let (Some(offset), Some(length)) = (req.byte_offset, req.byte_length) {
                watch = watch.with_range(offset, length);
            }

            data.debugger.add_watch(watch);
            data.state.update_from_debugger(&data.debugger);

            (
                StatusCode::CREATED,
                Json(serde_json::json!({
                    "success": true,
                    "watch_id": req.id,
                    "message": "Watch expression added"
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Remove a watch expression from a debug session
pub async fn debug_remove_watch_handler(
    Path((session_id, watch_id)): Path<(String, String)>,
) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().unwrap();

    match sessions.get_mut(&session_id) {
        Some(data) => {
            if data.debugger.remove_watch(&watch_id) {
                data.state.update_from_debugger(&data.debugger);
                (
                    StatusCode::OK,
                    Json(serde_json::json!({
                        "success": true,
                        "message": format!("Watch '{}' removed", watch_id)
                    })),
                )
            } else {
                (
                    StatusCode::NOT_FOUND,
                    Json(serde_json::json!({
                        "error": format!("Watch '{}' not found", watch_id)
                    })),
                )
            }
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Step forward one tick
pub async fn debug_step_forward_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().unwrap();

    match sessions.get_mut(&session_id) {
        Some(data) => {
            let success = data.debugger.step_forward();
            data.state.update_from_debugger(&data.debugger);

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "success": success,
                    "state": format!("{:?}", data.debugger.state()),
                    "current_tick": data.debugger.current_tick(),
                    "recent_events": data.debugger.recent_events(5).iter().map(|e| {
                        format!("{:?}", e)
                    }).collect::<Vec<_>>()
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Step backward one tick
pub async fn debug_step_backward_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().unwrap();

    match sessions.get_mut(&session_id) {
        Some(data) => {
            let success = data.debugger.step_backward();
            data.state.update_from_debugger(&data.debugger);

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "success": success,
                    "state": format!("{:?}", data.debugger.state()),
                    "current_tick": data.debugger.current_tick(),
                    "recent_events": data.debugger.recent_events(5).iter().map(|e| {
                        format!("{:?}", e)
                    }).collect::<Vec<_>>()
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Continue execution until breakpoint
pub async fn debug_continue_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().unwrap();

    match sessions.get_mut(&session_id) {
        Some(data) => {
            let _event = data.debugger.continue_execution();
            data.state.update_from_debugger(&data.debugger);

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "state": format!("{:?}", data.debugger.state()),
                    "current_tick": data.debugger.current_tick(),
                    "recent_events": data.debugger.recent_events(10).iter().map(|e| {
                        format!("{:?}", e)
                    }).collect::<Vec<_>>()
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Pause execution
pub async fn debug_pause_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().unwrap();

    match sessions.get_mut(&session_id) {
        Some(data) => {
            data.debugger.pause();
            data.state.update_from_debugger(&data.debugger);

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "success": true,
                    "state": format!("{:?}", data.debugger.state()),
                    "current_tick": data.debugger.current_tick()
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Seek to a specific tick
pub async fn debug_seek_handler(
    Path(session_id): Path<String>,
    Json(req): Json<SeekRequest>,
) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().unwrap();

    match sessions.get_mut(&session_id) {
        Some(data) => {
            let success = data.debugger.seek(req.tick);
            data.state.update_from_debugger(&data.debugger);

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "success": success,
                    "current_tick": data.debugger.current_tick(),
                    "state": format!("{:?}", data.debugger.state())
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Reset to the beginning
pub async fn debug_reset_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().unwrap();

    match sessions.get_mut(&session_id) {
        Some(data) => {
            data.debugger.reset();
            data.state.update_from_debugger(&data.debugger);

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "success": true,
                    "current_tick": data.debugger.current_tick(),
                    "state": format!("{:?}", data.debugger.state())
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Get current snapshot data
pub async fn debug_snapshot_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let sessions = DEBUG_SESSIONS.lock().unwrap();

    match sessions.get(&session_id) {
        Some(data) => {
            match data.debugger.current_snapshot() {
                Some(snapshot) => {
                    // Convert inputs and outputs to displayable format
                    let inputs: serde_json::Map<String, serde_json::Value> = snapshot
                        .inputs
                        .iter()
                        .map(|(k, v)| {
                            (
                                k.clone(),
                                serde_json::json!({
                                    "size": v.len(),
                                    "hex": format!("{:02x?}", &v[..v.len().min(64)]),
                                    "truncated": v.len() > 64
                                }),
                            )
                        })
                        .collect();

                    let outputs: serde_json::Map<String, serde_json::Value> = snapshot
                        .outputs
                        .iter()
                        .map(|(k, v)| {
                            (
                                k.clone(),
                                serde_json::json!({
                                    "size": v.len(),
                                    "hex": format!("{:02x?}", &v[..v.len().min(64)]),
                                    "truncated": v.len() > 64
                                }),
                            )
                        })
                        .collect();

                    (
                        StatusCode::OK,
                        Json(serde_json::json!({
                            "tick": snapshot.tick,
                            "timestamp_us": snapshot.timestamp_us,
                            "duration_ns": snapshot.duration_ns,
                            "duration_human": format!("{:.3} ms", snapshot.duration_ns as f64 / 1_000_000.0),
                            "inputs": inputs,
                            "outputs": outputs,
                            "has_state": snapshot.state.is_some(),
                            "state_size": snapshot.state.as_ref().map(|s| s.len()).unwrap_or(0)
                        })),
                    )
                }
                None => (
                    StatusCode::OK,
                    Json(serde_json::json!({
                        "snapshot": null,
                        "message": "No snapshot at current position"
                    })),
                ),
            }
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

/// Get current watch values
pub async fn debug_watches_values_handler(Path(session_id): Path<String>) -> impl IntoResponse {
    let mut sessions = DEBUG_SESSIONS.lock().unwrap();

    match sessions.get_mut(&session_id) {
        Some(data) => {
            let values = data.debugger.evaluate_watches();

            let values_json: Vec<serde_json::Value> = values
                .iter()
                .map(|v| {
                    serde_json::json!({
                        "expression_id": v.expression_id,
                        "tick": v.tick,
                        "display_value": v.display_value,
                        "raw_bytes_hex": format!("{:02x?}", &v.raw_bytes[..v.raw_bytes.len().min(32)]),
                        "raw_bytes_size": v.raw_bytes.len()
                    })
                })
                .collect();

            (
                StatusCode::OK,
                Json(serde_json::json!({
                    "values": values_json,
                    "count": values_json.len(),
                    "current_tick": data.debugger.current_tick()
                })),
            )
        }
        None => (
            StatusCode::NOT_FOUND,
            Json(serde_json::json!({
                "error": format!("Debug session '{}' not found", session_id)
            })),
        ),
    }
}

// WebSocket handler for real-time updates
async fn websocket_handler(ws: WebSocketUpgrade) -> impl IntoResponse {
    ws.on_upgrade(handle_websocket)
}

async fn handle_websocket(socket: WebSocket) {
    let (mut sender, _receiver) = socket.split();

    // Stream updates every 250ms (4 FPS) for reasonable real-time monitoring
    // This reduces backend load and network traffic significantly
    let mut interval = tokio::time::interval(tokio::time::Duration::from_millis(250));

    loop {
        interval.tick().await;

        // Gather all data in parallel
        let (nodes_result, topics_result, graph_result) = tokio::join!(
            tokio::task::spawn_blocking(|| {
                crate::discovery::discover_nodes()
                    .unwrap_or_default()
                    .into_iter()
                    .map(|n| {
                        serde_json::json!({
                            "name": n.name,
                            "pid": n.process_id,
                            "status": n.status,
                            "health": n.health.as_str(),
                            "health_color": n.health.color(),
                            "cpu": format!("{:.1}%", n.cpu_usage),
                            "memory": format!("{} MB", n.memory_usage / 1024 / 1024),
                            "scheduler_name": n.scheduler_name,
                        })
                    })
                    .collect::<Vec<_>>()
            }),
            tokio::task::spawn_blocking(|| {
                crate::discovery::discover_shared_memory()
                    .unwrap_or_default()
                    .into_iter()
                    .map(|t| {
                        serde_json::json!({
                            "name": t.topic_name,
                            "size": format!("{} KB", t.size_bytes / 1024),
                            "active": t.active,
                            "processes": t.accessing_processes.len(),
                        })
                    })
                    .collect::<Vec<_>>()
            }),
            tokio::task::spawn_blocking(|| {
                let (nodes, edges) = crate::graph::discover_graph_data();
                (nodes, edges)
            })
        );

        // Unwrap results
        let nodes = nodes_result.unwrap_or_default();
        let topics = topics_result.unwrap_or_default();
        let (graph_nodes, graph_edges) = graph_result.unwrap_or_default();

        // Convert graph data
        let graph_nodes_json = graph_nodes
            .into_iter()
            .map(|n| {
                serde_json::json!({
                    "id": n.id,
                    "label": n.label,
                    "type": match n.node_type {
                        crate::graph::NodeType::Process => "process",
                        crate::graph::NodeType::Topic => "topic",
                    },
                    "pid": n.pid,
                    "active": n.active,
                })
            })
            .collect::<Vec<_>>();

        let graph_edges_json = graph_edges
            .into_iter()
            .map(|e| {
                serde_json::json!({
                    "from": e.from,
                    "to": e.to,
                    "type": match e.edge_type {
                        crate::graph::EdgeType::Publish => "publish",
                        crate::graph::EdgeType::Subscribe => "subscribe",
                    },
                    "active": e.active,
                })
            })
            .collect::<Vec<_>>();

        // Build update message
        let update = serde_json::json!({
            "type": "update",
            "timestamp": chrono::Utc::now().to_rfc3339(),
            "data": {
                "nodes": nodes,
                "topics": topics,
                "graph": {
                    "nodes": graph_nodes_json,
                    "edges": graph_edges_json
                }
            }
        });

        // Send to client
        if sender
            .send(Message::Text(update.to_string()))
            .await
            .is_err()
        {
            break; // Client disconnected
        }
    }
}

fn generate_login_html() -> String {
    r#"<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>HORUS Monitor - Login</title>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@400;700;900&display=swap');

        * { margin: 0; padding: 0; box-sizing: border-box; }

        body {
            font-family: 'Orbitron', monospace;
            background: #0a0a0f;
            min-height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            padding: 20px;
            position: relative;
            overflow: hidden;
        }

        body::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background-image:
                linear-gradient(rgba(0, 255, 255, 0.1) 1px, transparent 1px),
                linear-gradient(90deg, rgba(0, 255, 255, 0.1) 1px, transparent 1px);
            background-size: 50px 50px;
            animation: gridMove 20s linear infinite;
            z-index: 0;
        }

        @keyframes gridMove {
            0% { transform: translate(0, 0); }
            100% { transform: translate(50px, 50px); }
        }

        body::after {
            content: '';
            position: absolute;
            width: 500px;
            height: 500px;
            background: radial-gradient(circle, rgba(0, 255, 136, 0.2), transparent);
            border-radius: 50%;
            top: -250px;
            right: -250px;
            animation: pulse 4s ease-in-out infinite;
            z-index: 0;
        }

        @keyframes pulse {
            0%, 100% { opacity: 0.5; transform: scale(1); }
            50% { opacity: 0.8; transform: scale(1.1); }
        }

        .login-container {
            background: rgba(15, 15, 25, 0.9);
            padding: 40px;
            border-radius: 20px;
            border: 2px solid #00ffff;
            box-shadow:
                0 0 20px rgba(0, 255, 255, 0.5),
                0 0 40px rgba(0, 255, 136, 0.3),
                inset 0 0 60px rgba(0, 255, 255, 0.1);
            max-width: 400px;
            width: 100%;
            position: relative;
            z-index: 1;
            backdrop-filter: blur(10px);
        }

        .logo {
            text-align: center;
            margin-bottom: 30px;
        }

        .logo h1 {
            color: #00ffff;
            font-size: 42px;
            margin-bottom: 8px;
            text-shadow:
                0 0 10px #00ffff,
                0 0 20px #00ffff,
                0 0 30px #00ffff;
            font-weight: 900;
            letter-spacing: 4px;
            animation: glitch 3s infinite;
        }

        @keyframes glitch {
            0%, 90%, 100% { transform: translate(0); }
            92% { transform: translate(-2px, 2px); }
            94% { transform: translate(2px, -2px); }
            96% { transform: translate(-2px, -2px); }
            98% { transform: translate(2px, 2px); }
        }

        .logo p {
            color: #00ff88;
            font-size: 14px;
            text-transform: uppercase;
            letter-spacing: 3px;
            text-shadow: 0 0 10px #00ff88;
        }

        .form-group {
            margin-bottom: 20px;
        }

        label {
            display: block;
            color: #00ffff;
            font-weight: 600;
            margin-bottom: 8px;
            font-size: 14px;
            text-transform: uppercase;
            letter-spacing: 2px;
        }

        input[type="password"] {
            width: 100%;
            padding: 12px 16px;
            background: rgba(0, 0, 0, 0.5);
            border: 2px solid #00ffff;
            border-radius: 8px;
            font-size: 16px;
            color: #fff;
            font-family: 'Orbitron', monospace;
            transition: all 0.3s;
        }

        input[type="password"]:focus {
            outline: none;
            border-color: #00ff88;
            box-shadow:
                0 0 10px #00ff88,
                inset 0 0 10px rgba(0, 255, 136, 0.2);
        }

        .btn {
            width: 100%;
            padding: 14px;
            background: linear-gradient(135deg, #00ffff 0%, #00ff88 100%);
            color: #0a0a0f;
            border: none;
            border-radius: 8px;
            font-size: 16px;
            font-weight: 700;
            cursor: pointer;
            transition: all 0.3s;
            text-transform: uppercase;
            letter-spacing: 2px;
            font-family: 'Orbitron', monospace;
            box-shadow: 0 0 20px rgba(0, 255, 255, 0.5);
        }

        .btn:hover {
            transform: translateY(-2px);
            box-shadow:
                0 0 30px #00ffff,
                0 0 40px #00ff88;
        }

        .btn:active {
            transform: translateY(0);
        }

        .error {
            background: rgba(255, 0, 100, 0.2);
            color: #ff0066;
            padding: 12px;
            border-radius: 8px;
            border: 1px solid #ff0066;
            margin-bottom: 20px;
            font-size: 14px;
            display: none;
            text-shadow: 0 0 5px #ff0066;
        }

        .error.show {
            display: block;
            animation: errorPulse 0.5s;
        }

        @keyframes errorPulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }

        .info {
            text-align: center;
            color: #00ff88;
            font-size: 11px;
            margin-top: 20px;
            text-transform: uppercase;
            letter-spacing: 1px;
            text-shadow: 0 0 5px #00ff88;
        }
    
        /* Mobile Responsive Styles */
        @media (max-width: 768px) {
            body {
                padding: 10px;
            }

            body::before {
                background-size: 30px 30px;
            }

            body::after {
                width: 300px;
                height: 300px;
                top: -150px;
                right: -150px;
            }

            .login-container {
                padding: 30px 20px;
                max-width: 100%;
                border-radius: 16px;
            }

            .logo h1 {
                font-size: 36px;
                letter-spacing: 3px;
            }

            .logo p {
                font-size: 12px;
                letter-spacing: 2px;
            }

            label {
                font-size: 12px;
                letter-spacing: 1.5px;
            }

            input[type="password"] {
                padding: 14px 16px;
                font-size: 16px;
            }

            .btn {
                padding: 16px;
                font-size: 15px;
            }

            .info {
                font-size: 10px;
            }
        }

        @media (max-width: 480px) {
            .login-container {
                padding: 25px 15px;
            }

            .logo h1 {
                font-size: 32px;
            }
        }

    </style>
</head>
<body>
    <div class="login-container">
        <div class="logo">
            <h1>HORUS</h1>
            <p>Robotics Monitor</p>
        </div>

        <div id="error" class="error"></div>

        <form id="loginForm">
            <div class="form-group">
                <label for="password">Password</label>
                <input type="password" id="password" name="password" required autofocus>
            </div>

            <button type="submit" class="btn">Login</button>
        </form>

        <div class="info">
            Secure Connection Established
        </div>
    </div>

    <script>
        const form = document.getElementById('loginForm');
        const errorDiv = document.getElementById('error');

        form.addEventListener('submit', async (e) => {
            e.preventDefault();
            errorDiv.classList.remove('show');

            const password = document.getElementById('password').value;

            try {
                const response = await fetch('/api/login', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({ password })
                });

                const data = await response.json();

                if (data.success) {
                    window.location.reload();
                } else {
                    errorDiv.textContent = data.error || 'Invalid password';
                    errorDiv.classList.add('show');
                    document.getElementById('password').value = '';
                    document.getElementById('password').focus();
                }
            } catch (error) {
                errorDiv.textContent = 'Network error. Please try again.';
                errorDiv.classList.add('show');
            }
        });
    </script>
</body>
</html>"#.to_string()
}

fn generate_html(port: u16) -> String {
    format!(
        r#"<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>HORUS Monitor</title>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;600;700&display=swap');

        :root {{
            --primary: #0a0a0f;
            --accent: #00ffff;
            --success: #00ff88;
            --warning: #ffaa00;
            --error: #ff0066;
            --gray: #64748B;
            --dark-bg: #0a0a0f;
            --card-bg: rgba(15, 15, 25, 0.8);
            --surface: rgba(15, 15, 25, 0.8);
            --surface-hover: rgba(30, 30, 45, 0.9);
            --border: rgba(0, 255, 255, 0.3);
            --text-primary: #00ffff;
            --text-secondary: #94A3B8;
            --text-tertiary: #00ff88;
            --neon-cyan: #00ffff;
            --neon-green: #00ff88;
            --neon-green: #00ff88;
        }}

        /* Light theme variables */
        [data-theme="light"] {{
            --primary: #1E293B;
            --accent: #0369A1;
            --success: #059669;
            --gray: #64748B;
            --dark-bg: #F8FAFC;
            --card-bg: #FFFFFF;
            --surface: #FFFFFF;
            --surface-hover: #F1F5F9;
            --border: rgba(3, 105, 161, 0.2);
            --text-primary: #1E293B;
            --text-secondary: #475569;
            --text-tertiary: #64748B;
        }}

        [data-theme="light"] body {{
            background-image: repeating-linear-gradient(
                0deg,
                transparent,
                transparent 2px,
                rgba(3, 105, 161, 0.05) 2px,
                rgba(3, 105, 161, 0.05) 4px
            );
        }}

        [data-theme="light"] .logo h1 {{
            background: linear-gradient(135deg, #0369A1, #EA580C);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
        }}

        [data-theme="light"] .status-value {{
            color: #EA580C;
            text-shadow: 0 0 10px rgba(0, 0, 0, 0.4), 0 0 20px rgba(0, 0, 0, 0.2);
        }}

        * {{
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }}

        body {{
            font-family: 'JetBrains Mono', monospace;
            background-color: var(--dark-bg);
            background-image: repeating-linear-gradient(
                0deg,
                transparent,
                transparent 2px,
                rgba(0, 212, 255, 0.03) 2px,
                rgba(0, 212, 255, 0.03) 4px
            );
            color: var(--text-primary);
            min-height: 100vh;
        }}

        .container {{
            display: flex;
            min-height: 100vh;
            padding: 0;
        }}

        .sidebar {{
            width: 250px;
            background: rgba(22, 24, 28, 0.9);
            
            border-right: 1px solid var(--border);
            padding: 2rem 0;
            position: fixed;
            height: 100vh;
            overflow-y: auto;
            
        }}

        [data-theme="light"] .sidebar {{
            background: rgba(248, 250, 252, 0.9);
        }}

        .logo {{
            padding: 0 1.5rem;
            margin-bottom: 2rem;
            display: flex;
            align-items: center;
            gap: 0.75rem;
        }}

        .logo h1 {{
            font-size: 1.5rem;
            font-weight: 800;
            background: linear-gradient(135deg, #00D4FF, #00FF88);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
            
        }}

        .main-content {{
            margin-left: 250px;
            flex: 1;
            padding: 2rem;
        }}

        h1 {{
            font-size: 2rem;
            font-weight: 800;
            color: var(--text-primary);
            margin-bottom: 1.5rem;
        }}

        .status-bar {{
            background: var(--surface);
            border: 1px solid var(--border);
            border-radius: 8px;
            padding: 1.5rem;
            margin-bottom: 2rem;
            display: flex;
            gap: 2rem;
            align-items: center;
        }}

        .status-item {{
            display: flex;
            flex-direction: column;
        }}

        .status-label {{
            color: var(--text-secondary);
            font-size: 0.875rem;
            margin-bottom: 0.5rem;
        }}

        .status-value {{
            color: var(--success);
            font-size: 1.5rem;
            font-weight: 600;
            text-shadow: 0 0 20px var(--success);
        }}

        /* Status item with tooltip */
        .status-item-with-tooltip {{
            position: relative;
            cursor: pointer;
        }}

        .status-item-with-tooltip:hover {{
            background: var(--surface-hover);
            border-radius: 8px;
            padding: 0.5rem;
            margin: -0.5rem;
        }}

        /* Tooltip container */
        .status-tooltip {{
            display: none;
            position: absolute;
            top: 100%;
            left: 50%;
            transform: translateX(-50%);
            margin-top: 0.75rem;
            background: var(--card-bg);
            border: 1px solid var(--accent);
            border-radius: 8px;
            
            padding: 0;
            min-width: 250px;
            max-width: 350px;
            z-index: 1000;
            
        }}

            to {{
                opacity: 1;
                transform: translateX(-50%) translateY(0);
            }}
        }}

        .status-item-with-tooltip:hover .status-tooltip {{
            display: block;
        }}

        .tooltip-header {{
            background: var(--accent);
            color: var(--primary);
            padding: 0.75rem 1rem;
            font-weight: 600;
            font-size: 0.875rem;
            border-radius: 8px 8px 0 0;
        }}

        .tooltip-content {{
            padding: 0.75rem;
            max-height: 300px;
            overflow-y: auto;
        }}

        .tooltip-node-item, .tooltip-topic-item {{
            display: flex;
            align-items: center;
            gap: 0.5rem;
            padding: 0.5rem;
            border-radius: 4px;
            margin-bottom: 0.25rem;
            font-size: 0.875rem;
        }}

        .tooltip-node-item:hover, .tooltip-topic-item:hover {{
            background: var(--surface-hover);
        }}

        .tooltip-node-health {{
            width: 8px;
            height: 8px;
            border-radius: 50%;
            flex-shrink: 0;
        }}

        .tooltip-node-health.health-green {{ background: #00FF88;  }}
        .tooltip-node-health.health-yellow {{ background: #FFC107;  }}
        .tooltip-node-health.health-orange {{ background: #FF9800;  }}
        .tooltip-node-health.health-red {{ background: #F44336;  }}
        .tooltip-node-health.health-gray {{ background: #9E9E9E;  }}

        .tooltip-node-name {{
            color: var(--text-primary);
            font-weight: 500;
            flex: 1;
        }}

        .tooltip-node-status {{
            color: var(--text-secondary);
            font-size: 0.75rem;
        }}

        .tooltip-topic-bullet {{
            color: var(--accent);
            font-weight: bold;
        }}

        .tooltip-topic-name {{
            color: var(--text-primary);
        }}

        .tooltip-loading {{
            color: var(--text-secondary);
            font-style: italic;
            text-align: center;
            padding: 1rem;
        }}

        .grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
            gap: 1.5rem;
            margin-bottom: 2rem;
        }}

        .card {{
            background: var(--surface);
            border: 1px solid rgba(0, 212, 255, 0.2);
            border-radius: 8px;
            padding: 1.5rem;
            
            position: relative;
            overflow: hidden;
        }}

        .card::before {{
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 2px;
            background: linear-gradient(90deg, transparent, #00D4FF, transparent);
            
        }}


        .card:hover {{
            transform: translateY(-5px);
            border-color: var(--accent);
            
        }}

        .card h2 {{
            color: var(--accent);
            font-size: 1.5rem;
            margin-bottom: 1rem;
            border-bottom: 2px solid var(--border);
            padding-bottom: 0.5rem;
        }}

        /* Scrollable lists for nodes and topics */
        #nodes-list, #topics-list {{
            max-height: 600px;
            overflow-y: auto;
            overflow-x: hidden;
        }}

        #nodes-list::-webkit-scrollbar, #topics-list::-webkit-scrollbar {{
            width: 8px;
        }}

        #nodes-list::-webkit-scrollbar-track, #topics-list::-webkit-scrollbar-track {{
            background: var(--dark-bg);
            border-radius: 4px;
        }}

        #nodes-list::-webkit-scrollbar-thumb, #topics-list::-webkit-scrollbar-thumb {{
            background: var(--accent);
            border-radius: 4px;
        }}

        #nodes-list::-webkit-scrollbar-thumb:hover, #topics-list::-webkit-scrollbar-thumb:hover {{
            background: #00B8E6;
        }}

        .placeholder {{
            color: var(--text-secondary);
            font-style: italic;
            padding: 2rem;
            text-align: center;
        }}

        .pulse {{
            display: inline-block;
            width: 8px;
            height: 8px;
            background: var(--success);
            border-radius: 50%;
            
            margin-right: 0.5rem;
        }}


        .command {{
            background: var(--dark-bg);
            border: 1px solid var(--border);
            border-radius: 4px;
            padding: 1rem;
            font-family: 'JetBrains Mono', monospace;
            color: var(--text-secondary);
            margin-top: 1rem;
            cursor: pointer;
            
        }}

        .command:hover {{
            border-color: var(--success);
            background: var(--surface-hover);
        }}

        .command-prompt {{
            color: var(--success);
            margin-right: 0.5rem;
        }}

        [data-theme="light"] .command-prompt {{
            color: #EA580C;
            text-shadow: 0 0 8px rgba(0, 0, 0, 0.5), 0 0 16px rgba(0, 0, 0, 0.3);
        }}

        .theme-toggle {{
            position: fixed;
            bottom: 2rem;
            left: 1rem;
            background: var(--surface);
            border: 1px solid var(--border);
            border-radius: 6px;
            padding: 0.5rem;
            cursor: pointer;
            font-size: 1.5rem;
            
            z-index: 1001;
            width: 48px;
            height: 48px;
            display: flex;
            align-items: center;
            justify-content: center;
            color: var(--text-secondary);
        }}

        .theme-toggle:hover {{
            background: var(--surface-hover);
            border-color: var(--accent);
            color: var(--accent);
        }}

        /* Help Button */
        .help-button {{
            position: fixed;
            bottom: 2rem;
            left: 5rem;
            background: var(--surface);
            border: 1px solid var(--border);
            border-radius: 6px;
            padding: 0.5rem;
            cursor: pointer;
            font-size: 1.5rem;
            font-weight: bold;
            
            z-index: 1001;
            width: 48px;
            height: 48px;
            display: flex;
            align-items: center;
            justify-content: center;
            color: var(--text-secondary);
        }}

        .help-button:hover {{
            background: var(--surface-hover);
            border-color: var(--accent);
            color: var(--accent);
            transform: scale(1.05);
        }}

        /* Help Modal */
        .help-modal {{
            display: none;
            position: fixed;
            z-index: 2000;
            left: 0;
            top: 0;
            width: 100%;
            height: 100%;
            overflow: auto;
            background-color: rgba(0, 0, 0, 0.7);
            
        }}

        .help-modal.active {{
            display: block;
        }}


        .help-modal-content {{
            background-color: var(--card-bg);
            margin: 3% auto;
            border: 1px solid var(--border);
            border-radius: 12px;
            width: 90%;
            max-width: 900px;
            max-height: 85vh;
            display: flex;
            flex-direction: column;
            
        }}

            to {{
                transform: translateY(0);
                opacity: 1;
            }}
        }}

        .help-modal-header {{
            background: linear-gradient(135deg, var(--accent), #00A8CC);
            color: var(--primary);
            padding: 1.5rem 2rem;
            border-radius: 12px 12px 0 0;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }}

        .help-modal-header h2 {{
            margin: 0;
            font-size: 1.75rem;
            font-weight: 700;
        }}

        .help-close {{
            background: none;
            border: none;
            color: var(--primary);
            font-size: 2.5rem;
            cursor: pointer;
            
            padding: 0;
            width: 40px;
            height: 40px;
            display: flex;
            align-items: center;
            justify-content: center;
        }}

        .help-close:hover {{
            transform: rotate(90deg);
        }}

        .help-modal-body {{
            padding: 2rem;
            overflow-y: auto;
            flex: 1;
        }}

        .help-section {{
            margin-bottom: 2.5rem;
        }}

        .help-section h3 {{
            color: var(--accent);
            margin-bottom: 1.5rem;
            font-size: 1.5rem;
            border-bottom: 2px solid var(--border);
            padding-bottom: 0.5rem;
        }}

        .help-item {{
            margin-bottom: 1.5rem;
            padding: 1rem;
            background: var(--surface);
            border-radius: 8px;
            border-left: 3px solid var(--accent);
        }}

        .help-status {{
            display: flex;
            align-items: center;
            gap: 0.75rem;
            margin-bottom: 0.5rem;
        }}

        .status-dot {{
            width: 12px;
            height: 12px;
            border-radius: 50%;
            flex-shrink: 0;
        }}

        .status-dot.health-green {{ background: #00FF88;  }}
        .status-dot.health-yellow {{ background: #FFC107;  }}
        .status-dot.health-orange {{ background: #FF9800;  }}
        .status-dot.health-red {{ background: #F44336;  }}
        .status-dot.health-gray {{ background: #9E9E9E;  }}

        .help-item strong {{
            color: var(--text-primary);
            font-size: 1.1rem;
        }}

        .help-item p {{
            color: var(--text-secondary);
            margin: 0.5rem 0;
        }}

        .help-item ul, .help-item ol {{
            margin: 0.75rem 0;
            padding-left: 1.5rem;
        }}

        .help-item li {{
            color: var(--text-secondary);
            margin: 0.5rem 0;
        }}

        .help-item code {{
            background: var(--dark-bg);
            border: 1px solid var(--border);
            padding: 0.25rem 0.5rem;
            border-radius: 4px;
            font-family: 'JetBrains Mono', monospace;
            font-size: 0.9rem;
            color: var(--accent);
        }}

        .help-item kbd {{
            background: var(--surface-hover);
            border: 1px solid var(--border);
            border-radius: 4px;
            padding: 0.25rem 0.5rem;
            font-family: 'JetBrains Mono', monospace;
            font-size: 0.9rem;
            color: var(--accent);
            
        }}

        .nav {{
            list-style: none;
            padding: 0;
            margin: 0;
        }}

        .nav-item {{
            display: block;
            width: 100%;
            padding: 1rem 1.5rem;
            background: transparent;
            border: none;
            border-left: 3px solid transparent;
            color: var(--text-secondary);
            cursor: pointer;
            font-family: 'Courier New', monospace;
            font-size: 1rem;
            
            text-align: left;
        }}

        .nav-item:hover {{
            background: var(--dark-bg);
            color: var(--accent);
            border-left-color: var(--accent);
        }}

        .nav-item.active {{
            background: var(--dark-bg);
            color: var(--accent);
            border-left-color: var(--accent);
        }}

        .tab-content {{
            display: none;
        }}

        .tab-content.active {{
            display: block;
        }}

        .view-selector {{
            display: flex;
            gap: 1rem;
            margin-bottom: 1.5rem;
            padding: 0.5rem;
            background: var(--surface);
            border-radius: 8px;
            border: 1px solid var(--border);
        }}

        .view-btn {{
            padding: 0.75rem 1.5rem;
            background: transparent;
            border: none;
            color: var(--text-secondary);
            font-size: 0.875rem;
            font-weight: 600;
            cursor: pointer;
            border-radius: 6px;
            
            font-family: 'JetBrains Mono', monospace;
        }}

        .view-btn:hover {{
            background: var(--surface-hover);
            color: var(--accent);
        }}

        .view-btn.active {{
            background: var(--accent);
            color: var(--primary);
            
        }}

        .refresh-btn {{
            margin-left: auto;
            background: rgba(0, 212, 255, 0.1);
            border: 1px solid var(--accent);
            color: var(--accent);
        }}

        .refresh-btn:hover {{
            background: rgba(0, 212, 255, 0.2);
            
        }}

        .monitor-view {{
            display: none;
        }}

        .monitor-view.active {{
            display: flex;
            flex-direction: column;
            height: calc(100vh - 200px);
        }}

        .graph-card {{
            display: flex;
            flex-direction: column;
            height: 100%;
        }}

        .graph-card h2 {{
            margin-bottom: 1rem;
            flex-shrink: 0;
        }}

        .graph-card canvas {{
            flex: 1;
            min-height: 0;
        }}

        .remote-view-btn {{
            padding: 0.75rem 1.5rem;
            background: transparent;
            border: none;
            color: var(--text-secondary);
            font-size: 0.875rem;
            font-weight: 600;
            cursor: pointer;
            border-radius: 6px;
            
            font-family: 'JetBrains Mono', monospace;
        }}

        .remote-view-btn:hover {{
            background: var(--surface-hover);
            color: var(--accent);
        }}

        .remote-view-btn.active {{
            background: var(--accent);
            color: var(--primary);
            
        }}

        .remote-view {{
            display: none;
        }}

        .remote-view.active {{
            display: block;
        }}

        .node-item, .topic-item {{
            background: var(--dark-bg);
            border: 1px solid var(--border);
            border-radius: 4px;
            padding: 1rem;
            margin-bottom: 0.5rem;
            cursor: pointer;
            
            user-select: none;
        }}

        .node-item:hover, .topic-item:hover {{
            border-color: var(--accent);
            background: var(--surface);
            
        }}

        .node-header, .topic-header {{
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 0.5rem;
        }}

        .node-name, .topic-name {{
            color: var(--accent);
            font-weight: 600;
        }}

        .node-status {{
            padding: 0.25rem 0.75rem;
            border-radius: 4px;
            font-size: 0.875rem;
        }}

        .status-running {{
            background: rgba(0, 255, 136, 0.2);
            color: var(--success);
        }}

        /* Health status colors */
        .status-green {{
            background: rgba(0, 255, 136, 0.2);
            color: #00FF88;
        }}
        .status-yellow {{
            background: rgba(255, 193, 7, 0.2);
            color: #FFC107;
        }}
        .status-orange {{
            background: rgba(255, 152, 0, 0.2);
            color: #FF9800;
        }}
        .status-red {{
            background: rgba(244, 67, 54, 0.2);
            color: #F44336;
        }}
        .status-gray {{
            background: rgba(158, 158, 158, 0.2);
            color: #9E9E9E;
        }}

        .node-details, .topic-details {{
            display: flex;
            gap: 1.5rem;
            font-size: 0.875rem;
            color: var(--text-secondary);
        }}

        /* Mobile Responsive Styles */
        @media (max-width: 768px) {{
            .container {{
                flex-direction: column;
            }}

            .sidebar {{
                width: 100%;
                height: auto;
                position: relative;
                border-right: none;
                border-bottom: 1px solid var(--border);
                padding: 1rem 0;
            }}

            .logo {{
                padding: 0 1rem;
                margin-bottom: 1rem;
            }}

            .logo h1 {{
                font-size: 1.25rem;
            }}

            .nav {{
                display: flex;
                flex-direction: row;
                overflow-x: auto;
                padding: 0 1rem;
                gap: 0.5rem;
            }}

            .nav li {{
                margin: 0;
            }}

            .nav-item {{
                white-space: nowrap;
                padding: 0.75rem 1rem;
                font-size: 0.875rem;
            }}

            .main-content {{
                margin-left: 0;
                padding: 1rem;
            }}

            .status-bar {{
                flex-wrap: wrap;
                gap: 1rem;
                padding: 1rem;
            }}

            .status-item {{
                flex: 1;
                min-width: calc(50% - 0.5rem);
            }}

            .grid {{
                grid-template-columns: 1fr;
                gap: 1rem;
            }}

            .view-selector {{
                padding: 0.25rem;
                gap: 0.5rem;
            }}

            .view-btn {{
                padding: 0.5rem 1rem;
                font-size: 0.75rem;
            }}

            /* Graph canvas height now managed by flexbox */

            .card {{
                padding: 1rem;
            }}

            .card h2 {{
                font-size: 1.25rem;
            }}

            .node-details, .topic-details {{
                flex-direction: column;
                gap: 0.5rem;
            }}
        }}

        /* iPhone specific optimizations */
        @media (max-width: 430px) {{
            .logo h1 {{
                font-size: 1rem;
            }}

            .status-item {{
                min-width: 100%;
            }}

            .status-value {{
                font-size: 1.25rem;
            }}

            .nav-item {{
                padding: 0.5rem 0.75rem;
                font-size: 0.75rem;
            }}

            .view-btn {{
                padding: 0.5rem 0.75rem;
                font-size: 0.7rem;
            }}

            /* Graph canvas height now managed by flexbox */
        }}

        /* Log Panel - Slides from right */
        .log-panel {{
            position: fixed;
            top: 0;
            right: -500px;
            width: 500px;
            height: 100vh;
            background: var(--surface);
            border-left: 2px solid var(--border);
            
            
            z-index: 1000;
            display: flex;
            flex-direction: column;
            pointer-events: none;
        }}

        .log-panel.open {{
            right: 0;
            pointer-events: auto;
        }}

        .log-panel-header {{
            padding: 1.5rem;
            border-bottom: 1px solid var(--border);
            display: flex;
            justify-content: space-between;
            align-items: center;
            background: var(--dark-bg);
        }}

        .log-panel-title {{
            font-size: 1.2rem;
            font-weight: 600;
            color: var(--accent);
        }}

        .log-panel-close {{
            background: transparent;
            border: 1px solid var(--border);
            color: var(--text-secondary);
            padding: 0.5rem 1rem;
            border-radius: 4px;
            cursor: pointer;
            
        }}

        .log-panel-close:hover {{
            border-color: var(--accent);
            color: var(--accent);
        }}

        .log-panel-content {{
            flex: 1;
            overflow-y: auto;
            padding: 1rem;
        }}

        .log-entry {{
            background: var(--dark-bg);
            border: 1px solid var(--border);
            border-radius: 4px;
            padding: 0.75rem;
            margin-bottom: 0.5rem;
            font-size: 0.85rem;
        }}

        .log-entry-header {{
            display: flex;
            justify-content: space-between;
            margin-bottom: 0.5rem;
            font-size: 0.75rem;
        }}

        .log-timestamp {{
            color: var(--text-tertiary);
        }}

        .log-type {{
            padding: 0.2rem 0.5rem;
            border-radius: 3px;
            font-size: 0.7rem;
            font-weight: 600;
        }}

        .log-type-publish {{ background: rgba(0, 212, 255, 0.2); color: var(--accent); }}
        .log-type-subscribe {{ background: rgba(0, 255, 136, 0.2); color: var(--success); }}
        .log-type-info {{ background: rgba(100, 116, 139, 0.2); color: var(--text-secondary); }}
        .log-type-warning {{ background: rgba(255, 165, 0, 0.2); color: #ffa500; }}
        .log-type-error {{ background: rgba(255, 68, 68, 0.2); color: #ff4444; }}
        .log-type-topicread {{ background: rgba(0, 255, 136, 0.2); color: var(--success); }}
        .log-type-topicwrite {{ background: rgba(0, 212, 255, 0.2); color: var(--accent); }}
        .log-type-topicmap {{ background: rgba(138, 43, 226, 0.2); color: #c792ea; }}
        .log-type-topicunmap {{ background: rgba(255, 136, 0, 0.2); color: #ff8800; }}

        .log-message {{
            color: var(--text-primary);
            margin-top: 0.5rem;
            word-break: break-word;
        }}

        @media (max-width: 768px) {{
            .log-panel {{
                width: 100%;
                right: -100%;
            }}
        }}

        /* Install Dialog Modal */
        .install-dialog {{
            display: none;
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(0, 0, 0, 0.7);
            z-index: 2000;
            align-items: center;
            justify-content: center;
        }}

        .install-dialog.active {{
            display: flex;
        }}

        .install-dialog-content {{
            background: var(--surface);
            border: 2px solid var(--accent);
            border-radius: 12px;
            width: 90%;
            max-width: 500px;
            max-height: 80vh;
            display: flex;
            flex-direction: column;
            
        }}

        .install-dialog-header {{
            padding: 1.5rem;
            border-bottom: 1px solid var(--border);
            display: flex;
            justify-content: space-between;
            align-items: center;
        }}

        .install-dialog-header h3 {{
            color: var(--accent);
            margin: 0;
            font-size: 1.3rem;
        }}

        .install-dialog-body {{
            padding: 1.5rem;
            overflow-y: auto;
        }}

        .install-dialog-footer {{
            padding: 1rem 1.5rem;
            border-top: 1px solid var(--border);
            display: flex;
            gap: 10px;
            justify-content: flex-end;
        }}

        .install-option {{
            padding: 1rem;
            background: var(--dark-bg);
            border: 2px solid var(--border);
            border-radius: 8px;
            cursor: pointer;
            
            margin-bottom: 10px;
        }}

        .install-option:hover {{
            border-color: var(--accent);
            background: var(--primary);
        }}

        .install-option.selected {{
            border-color: var(--accent);
            background: var(--primary);
        }}

        .install-option input[type="radio"] {{
            margin-right: 10px;
        }}

        .local-packages-select {{
            width: 100%;
            padding: 10px;
            background: var(--dark-bg);
            border: 1px solid var(--border);
            border-radius: 6px;
            color: var(--text-primary);
            font-family: 'JetBrains Mono', monospace;
            margin-top: 10px;
        }}
    </style>
</head>
<body>
    <button class="theme-toggle" onclick="toggleTheme()" id="theme-toggle" title="Toggle theme">
        <svg id="theme-icon" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <!-- Moon icon (default for dark theme) -->
            <path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z"></path>
        </svg>
    </button>

    <button class="help-button" onclick="toggleHelp()" id="help-button" title="Help (Press ?)">
        ?
    </button>

    <!-- Help Modal -->
    <div class="help-modal" id="help-modal">
        <div class="help-modal-content">
            <div class="help-modal-header">
                <h2>HORUS Monitor Guide</h2>
                <button class="help-close" onclick="toggleHelp()">&times;</button>
            </div>
            <div class="help-modal-body">
                <!-- Health Status Section -->
                <div class="help-section">
                    <h3> Node Health Statuses</h3>
                    <div class="help-item">
                        <div class="help-status">
                            <span class="status-dot health-green"></span>
                            <strong>Healthy</strong>
                        </div>
                        <p>Node operating normally with no issues</p>
                        <ul>
                            <li>No errors detected</li>
                            <li>Fast execution (< 100ms per tick)</li>
                            <li>All systems functioning as expected</li>
                        </ul>
                    </div>

                    <div class="help-item">
                        <div class="help-status">
                            <span class="status-dot health-yellow"></span>
                            <strong>Warning</strong>
                        </div>
                        <p>Performance degraded, attention recommended</p>
                        <ul>
                            <li>Slow tick execution (> 100ms)</li>
                            <li>Missed deadlines or timing issues</li>
                            <li>System still functional but not optimal</li>
                        </ul>
                    </div>

                    <div class="help-item">
                        <div class="help-status">
                            <span class="status-dot health-orange"></span>
                            <strong>Error</strong>
                        </div>
                        <p>Errors occurring, investigation needed</p>
                        <ul>
                            <li>3-10 errors have occurred</li>
                            <li>Node still running but unreliable</li>
                            <li>Check logs for error details</li>
                        </ul>
                    </div>

                    <div class="help-item">
                        <div class="help-status">
                            <span class="status-dot health-red"></span>
                            <strong>Critical</strong>
                        </div>
                        <p>Severe issues - immediate action required</p>
                        <ul>
                            <li>10+ errors detected</li>
                            <li>Node may crash or become unresponsive</li>
                            <li>System stability at risk</li>
                        </ul>
                    </div>

                    <div class="help-item">
                        <div class="help-status">
                            <span class="status-dot health-gray"></span>
                            <strong>Unknown</strong>
                        </div>
                        <p>Unable to determine health status</p>
                        <ul>
                            <li>No heartbeat received (> 5 seconds)</li>
                            <li>Node may be frozen or deadlocked</li>
                            <li>Process might need restart</li>
                        </ul>
                    </div>
                </div>

                <!-- Monitor Features Section -->
                <div class="help-section">
                    <h3>[#] Monitor Features</h3>

                    <div class="help-item">
                        <strong>Status Bar</strong>
                        <p>Top bar showing system overview:</p>
                        <ul>
                            <li><strong>Active Nodes</strong> - Hover to see node list with health indicators</li>
                            <li><strong>Active Topics</strong> - Hover to see all topic names</li>
                            <li><strong>Port</strong> - Monitor server port number</li>
                        </ul>
                    </div>

                    <div class="help-item">
                        <strong>Monitor Tab</strong>
                        <p>View running nodes and topics</p>
                        <ul>
                            <li><strong>List View</strong> - Detailed list of nodes/topics with stats</li>
                            <li><strong>Graph View</strong> - Visual network topology showing connections</li>
                            <li><strong>Click nodes/topics</strong> - View detailed logs</li>
                        </ul>
                    </div>

                    <div class="help-item">
                        <strong>Parameters Tab</strong>
                        <p>Manage runtime parameters</p>
                        <ul>
                            <li>Add, edit, or delete parameters</li>
                            <li>Export parameters to YAML/JSON</li>
                            <li>Import parameter configurations</li>
                        </ul>
                    </div>

                    <div class="help-item">
                        <strong>Packages Tab</strong>
                        <p>Manage HORUS packages</p>
                        <ul>
                            <li><strong>Search</strong> - Find packages in registry</li>
                            <li><strong>Global</strong> - View globally installed packages</li>
                            <li><strong>Local</strong> - View local environment packages</li>
                            <li><strong>Registry</strong> - Search and install packages</li>
                        </ul>
                    </div>

                </div>

                <!-- Tips Section -->
                <div class="help-section">
                    <h3> Tips & Tricks</h3>

                    <div class="help-item">
                        <strong>Real-time Updates</strong>
                        <p>Monitor updates automatically via WebSocket (20 FPS). No refresh needed!</p>
                    </div>

                    <div class="help-item">
                        <strong>View Logs</strong>
                        <p>Click any node or topic in the Monitor tab to see detailed logs</p>
                    </div>

                    <div class="help-item">
                        <strong>Health Indicators</strong>
                        <p>Colored dots show node health at a glance. Hover over "Active Nodes" for quick status check</p>
                    </div>

                    <div class="help-item">
                        <strong>Dark/Light Theme</strong>
                        <p>Toggle between themes using the moon/sun button (bottom left)</p>
                    </div>
                </div>

                <!-- Keyboard Shortcuts Section -->
                <div class="help-section">
                    <h3>Keyboard Shortcuts</h3>

                    <div class="help-item">
                        <ul>
                            <li><kbd>?</kbd> - Open/close this help guide</li>
                            <li><kbd>Esc</kbd> - Close help modal</li>
                        </ul>
                    </div>
                </div>

                <!-- Getting Started Section -->
                <div class="help-section">
                    <h3> Getting Started</h3>

                    <div class="help-item">
                        <strong>Running Your First Node</strong>
                        <ol>
                            <li>Create a HORUS node file (e.g., <code>my_node.rs</code>)</li>
                            <li>Run: <code>horus run my_node.rs</code></li>
                            <li>Watch it appear in the Monitor tab with health status</li>
                            <li>Click the node to view logs in real-time</li>
                        </ol>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <div class="container">
        <!-- Left Sidebar Navigation -->
        <nav class="sidebar">
            <div class="logo">
                <h1>HORUS MONITOR</h1>
            </div>

            <ul class="nav">
                <li><button class="nav-item active" onclick="switchTab('monitor')">Monitor</button></li>
                <li><button class="nav-item" onclick="switchTab('params')">Parameters</button></li>
                <li><button class="nav-item" onclick="switchTab('packages')">Packages</button></li>
            </ul>
        </nav>

        <!-- Main Content Area -->
        <div class="main-content">
            <div class="status-bar">
                <div class="status-item status-item-with-tooltip" id="nodes-status-item">
                    <div class="status-label">Active Nodes</div>
                    <div class="status-value">
                        <span class="pulse"></span>
                        <span id="node-count">0</span>
                    </div>
                    <!-- Tooltip for node list -->
                    <div class="status-tooltip" id="nodes-tooltip">
                        <div class="tooltip-header">Active Nodes</div>
                        <div class="tooltip-content" id="nodes-tooltip-content">
                            <div class="tooltip-loading">No nodes running</div>
                        </div>
                    </div>
                </div>
                <div class="status-item status-item-with-tooltip" id="topics-status-item">
                    <div class="status-label">Active Topics</div>
                    <div class="status-value">
                        <span id="topic-count">0</span>
                    </div>
                    <!-- Tooltip for topic list -->
                    <div class="status-tooltip" id="topics-tooltip">
                        <div class="tooltip-header">Active Topics</div>
                        <div class="tooltip-content" id="topics-tooltip-content">
                            <div class="tooltip-loading">No topics available</div>
                        </div>
                    </div>
                </div>
                <div class="status-item">
                    <div class="status-label">Port</div>
                    <div class="status-value">{port}</div>
                </div>
            </div>

        <!-- Monitor Tab -->
        <div id="tab-monitor" class="tab-content active">
            <!-- View Selector -->
            <div class="view-selector">
                <button class="view-btn active" onclick="switchMonitorView('list')">List View</button>
                <button class="view-btn" onclick="switchMonitorView('graph')">Graph View</button>
                <button class="view-btn refresh-btn" onclick="refreshMonitorData()">Refresh</button>
            </div>

            <!-- List View -->
            <div id="monitor-view-list" class="monitor-view active">
                <div class="grid">
                    <div class="card">
                        <h2>Nodes</h2>
                        <div id="nodes-list"></div>
                    </div>

                    <div class="card">
                        <h2>Topics</h2>
                        <div id="topics-list"></div>
                    </div>
                </div>
            </div>

            <!-- Graph View -->
            <div id="monitor-view-graph" class="monitor-view">
                <div class="card graph-card">
                    <h2>System Graph</h2>
                    <canvas id="graph-canvas" width="1200" height="500" style="width: 100%; height: 100%; background: var(--dark-bg); border-radius: 4px; border: 1px solid var(--border);"></canvas>
                </div>
            </div>
        </div>

        <!-- Parameters Tab -->
        <div id="tab-params" class="tab-content">
            <div class="card">
                <h2>Runtime Parameters</h2>

                <!-- Actions Bar -->
                <div style="display: flex; gap: 1rem; margin-bottom: 1.5rem; flex-wrap: wrap;">
                    <input
                        type="text"
                        id="param-search"
                        placeholder="Search parameters..."
                        style="flex: 1; min-width: 200px; padding: 0.5rem 1rem; border: 1px solid var(--border); border-radius: 8px; background: var(--surface); color: var(--text-primary); font-family: 'JetBrains Mono', monospace;"
                    />
                    <button onclick="refreshParams()" style="padding: 0.5rem 1.5rem; background: var(--accent); color: white; border: none; border-radius: 8px; cursor: pointer; font-weight: 600;">
                        Refresh
                    </button>
                    <button onclick="showAddParamDialog()" style="padding: 0.5rem 1.5rem; background: var(--success); color: white; border: none; border-radius: 8px; cursor: pointer; font-weight: 600;">
                        Add Parameter
                    </button>
                    <button onclick="exportParams()" style="padding: 0.5rem 1.5rem; background: var(--warning); color: white; border: none; border-radius: 8px; cursor: pointer; font-weight: 600;">
                        Export
                    </button>
                    <button onclick="showImportDialog()" style="padding: 0.5rem 1.5rem; background: var(--info); color: white; border: none; border-radius: 8px; cursor: pointer; font-weight: 600;">
                        Import
                    </button>
                </div>

                <!-- Parameters Table -->
                <div id="params-container" style="overflow-x: auto;">
                    <table style="width: 100%; border-collapse: collapse; font-family: 'JetBrains Mono', monospace; font-size: 0.9rem;">
                        <thead>
                            <tr style="border-bottom: 2px solid var(--border); text-align: left;">
                                <th style="padding: 0.75rem; color: var(--text-secondary); font-weight: 600;">Key</th>
                                <th style="padding: 0.75rem; color: var(--text-secondary); font-weight: 600;">Value</th>
                                <th style="padding: 0.75rem; color: var(--text-secondary); font-weight: 600;">Type</th>
                                <th style="padding: 0.75rem; color: var(--text-secondary); font-weight: 600; text-align: right;">Actions</th>
                            </tr>
                        </thead>
                        <tbody id="params-table-body">
                            <tr>
                                <td colspan="4" style="padding: 2rem; text-align: center; color: var(--text-tertiary);">
                                    Loading parameters...
                                </td>
                            </tr>
                        </tbody>
                    </table>
                </div>
            </div>

            <!-- Add/Edit Parameter Dialog -->
            <div id="param-dialog" style="display: none; position: fixed; top: 0; left: 0; right: 0; bottom: 0; background: rgba(0,0,0,0.7); z-index: 1000; align-items: center; justify-content: center;">
                <div style="background: var(--surface); border-radius: 12px; padding: 2rem; max-width: 500px; width: 90%; border: 1px solid var(--border);">
                    <h3 id="dialog-title" style="margin-top: 0; color: var(--text-primary);">Add Parameter</h3>
                    <div style="margin-bottom: 1rem;">
                        <label style="display: block; margin-bottom: 0.5rem; color: var(--text-secondary); font-weight: 600;">Key</label>
                        <input type="text" id="param-key" placeholder="parameter_name" style="width: 100%; padding: 0.5rem; border: 1px solid var(--border); border-radius: 8px; background: var(--bg); color: var(--text-primary); font-family: 'JetBrains Mono', monospace;" />
                    </div>
                    <div style="margin-bottom: 1rem;">
                        <label style="display: block; margin-bottom: 0.5rem; color: var(--text-secondary); font-weight: 600;">Type</label>
                        <select id="param-type" onchange="updateValueInput()" style="width: 100%; padding: 0.5rem; border: 1px solid var(--border); border-radius: 8px; background: var(--bg); color: var(--text-primary);">
                            <option value="number">Number</option>
                            <option value="string">String</option>
                            <option value="boolean">Boolean</option>
                            <option value="array">Array (JSON)</option>
                            <option value="object">Object (JSON)</option>
                        </select>
                    </div>
                    <div style="margin-bottom: 1.5rem;">
                        <label style="display: block; margin-bottom: 0.5rem; color: var(--text-secondary); font-weight: 600;">Value</label>
                        <input type="text" id="param-value" placeholder="Enter value" style="width: 100%; padding: 0.5rem; border: 1px solid var(--border); border-radius: 8px; background: var(--bg); color: var(--text-primary); font-family: 'JetBrains Mono', monospace;" />
                    </div>
                    <div style="display: flex; gap: 1rem; justify-content: flex-end;">
                        <button onclick="closeParamDialog()" style="padding: 0.5rem 1.5rem; background: var(--surface); color: var(--text-primary); border: 1px solid var(--border); border-radius: 8px; cursor: pointer; font-weight: 600;">
                            Cancel
                        </button>
                        <button onclick="saveParam()" style="padding: 0.5rem 1.5rem; background: var(--accent); color: white; border: none; border-radius: 8px; cursor: pointer; font-weight: 600;">
                            Save
                        </button>
                    </div>
                </div>
            </div>

            <!-- Import Dialog -->
            <div id="import-dialog" style="display: none; position: fixed; top: 0; left: 0; right: 0; bottom: 0; background: rgba(0,0,0,0.7); z-index: 1000; align-items: center; justify-content: center;">
                <div style="background: var(--surface); border-radius: 12px; padding: 2rem; max-width: 600px; width: 90%; border: 1px solid var(--border);">
                    <h3 style="margin-top: 0; color: var(--text-primary);">Import Parameters</h3>
                    <div style="margin-bottom: 1rem;">
                        <label style="display: block; margin-bottom: 0.5rem; color: var(--text-secondary); font-weight: 600;">Format</label>
                        <select id="import-format" style="width: 100%; padding: 0.5rem; border: 1px solid var(--border); border-radius: 8px; background: var(--bg); color: var(--text-primary);">
                            <option value="yaml">YAML</option>
                            <option value="json">JSON</option>
                        </select>
                    </div>
                    <div style="margin-bottom: 1.5rem;">
                        <label style="display: block; margin-bottom: 0.5rem; color: var(--text-secondary); font-weight: 600;">Data</label>
                        <textarea id="import-data" rows="10" placeholder="Paste YAML or JSON here..." style="width: 100%; padding: 0.5rem; border: 1px solid var(--border); border-radius: 8px; background: var(--bg); color: var(--text-primary); font-family: 'JetBrains Mono', monospace; resize: vertical;"></textarea>
                    </div>
                    <div style="display: flex; gap: 1rem; justify-content: flex-end;">
                        <button onclick="closeImportDialog()" style="padding: 0.5rem 1.5rem; background: var(--surface); color: var(--text-primary); border: 1px solid var(--border); border-radius: 8px; cursor: pointer; font-weight: 600;">
                            Cancel
                        </button>
                        <button onclick="importParams()" style="padding: 0.5rem 1.5rem; background: var(--accent); color: white; border: none; border-radius: 8px; cursor: pointer; font-weight: 600;">
                            Import
                        </button>
                    </div>
                </div>
            </div>
        </div>

        <!-- Packages Tab -->
        <div id="tab-packages" class="tab-content">
            <!-- View Selector -->
            <div class="view-selector">
                <button class="view-btn active" onclick="switchPackageView('global')">Global Env</button>
                <button class="view-btn" onclick="switchPackageView('local')">Local Env</button>
                <button class="view-btn" onclick="switchPackageView('registry')">Registry</button>
            </div>

            <!-- Global Environment View -->
            <div id="package-global" class="package-view active">
                <div class="card">
                    <h2>Global Environment</h2>
                    <div id="global-packages-list">
                        <p style="color: var(--text-secondary);">Loading global packages...</p>
                    </div>
                </div>
            </div>

            <!-- Local Environment View -->
            <div id="package-local" class="package-view" style="display: none;">
                <div class="card">
                    <h2>Local Environments</h2>
                    <div id="local-environments-list">
                        <p style="color: var(--text-secondary);">Loading local environments...</p>
                    </div>
                </div>
            </div>

            <!-- Registry View -->
            <div id="package-registry" class="package-view" style="display: none;">
                <div class="card">
                    <h2> Package Registry</h2>
                    <div style="display: flex; gap: 10px; margin-bottom: 20px;">
                        <input
                            type="text"
                            id="registry-search-input"
                            placeholder="Search registry packages..."
                            style="flex: 1; padding: 10px; background: var(--surface); border: 1px solid var(--border); border-radius: 6px; color: var(--text-primary); font-family: 'JetBrains Mono', monospace;"
                        />
                        <button
                            onclick="searchRegistry()"
                            style="padding: 10px 20px; background: var(--accent); color: var(--primary); border: none; border-radius: 6px; cursor: pointer; font-weight: 600; font-family: 'JetBrains Mono', monospace;"
                        >
                            Search
                        </button>
                    </div>
                    <div id="registry-results">
                        <p style="color: var(--text-secondary);">Search for packages above</p>
                    </div>
                </div>
            </div>
        </div>


        </div> <!-- end main-content -->
    </div> <!-- end container -->

    <script>
        // Tab switching
        function switchTab(tabName) {{
            // Hide all tab contents
            document.querySelectorAll('.tab-content').forEach(content => {{
                content.classList.remove('active');
            }});

            // Remove active class from all nav items
            document.querySelectorAll('.nav-item').forEach(item => {{
                item.classList.remove('active');
            }});

            // Show selected tab content
            document.getElementById('tab-' + tabName).classList.add('active');

            // Add active class to clicked nav item
            event.target.classList.add('active');

            // Initialize packages tab if switching to it
            if (tabName === 'packages') {{
                onPackagesTabActivate();
            }}
        }}

        // Switch monitor view (list/graph)
        function switchMonitorView(viewName) {{
            // Hide all monitor views
            document.querySelectorAll('.monitor-view').forEach(view => {{
                view.classList.remove('active');
            }});

            // Remove active class from all view buttons
            document.querySelectorAll('.view-btn').forEach(btn => {{
                btn.classList.remove('active');
            }});

            // Show selected view
            document.getElementById('monitor-view-' + viewName).classList.add('active');

            // Add active class to clicked button
            event.target.classList.add('active');

            // Ensure canvas is properly sized when switching to graph view
            if (viewName === 'graph') {{
                // Ensure canvas is properly sized after becoming visible
                setTimeout(() => {{
                    const canvas = document.getElementById('graph-canvas');
                    if (canvas) {{
                        const container = canvas.parentElement;
                        const rect = canvas.getBoundingClientRect();
                        canvas.width = rect.width || container.clientWidth || 1200;
                        canvas.height = rect.height || container.clientHeight || 600;
                        // Re-render graph with new dimensions
                        if (window.graphState && window.graphState.nodes) {{
                            renderGraph(window.graphState.nodes, window.graphState.edges);
                        }}
                    }}
                }}, 100);
            }}
        }}

        // Reset graph layout to default positions
        function resetGraphLayout() {{
            graphState.nodePositions = {{}};
            graphState.nodeVelocities = {{}};
            graphState.hoveredNode = null;
            graphState.draggedNode = null;
        }}


        // Auto-refresh status
        async function updateStatus() {{
            try {{
                const response = await fetch('/api/status');
                const data = await response.json();

                const {{ nodeCount, topicCount }} = getCachedElements();
                if (nodeCount) nodeCount.textContent = data.nodes;
                if (topicCount) topicCount.textContent = data.topics;
            }} catch (error) {{
                console.error('Failed to fetch status:', error);
            }}
        }}

        // Update nodes tooltip
        async function updateNodesToolTip() {{
            try {{
                const response = await fetch('/api/nodes');
                const data = await response.json();
                const tooltipContent = document.getElementById('nodes-tooltip-content');

                if (data.nodes.length === 0) {{
                    tooltipContent.innerHTML = '<div class="tooltip-loading">No nodes running</div>';
                }} else {{
                    tooltipContent.innerHTML = data.nodes.map(node => `
                        <div class="tooltip-node-item">
                            <div class="tooltip-node-health health-${{node.health_color || 'gray'}}"></div>
                            <div class="tooltip-node-name">${{node.name}}</div>
                            <div class="tooltip-node-status">${{node.health}}</div>
                        </div>
                    `).join('');
                }}
            }} catch (error) {{
                console.error('Failed to update nodes tooltip:', error);
            }}
        }}

        // Update topics tooltip
        async function updateTopicsToolTip() {{
            try {{
                const response = await fetch('/api/topics');
                const data = await response.json();
                const tooltipContent = document.getElementById('topics-tooltip-content');

                if (data.topics.length === 0) {{
                    tooltipContent.innerHTML = '<div class="tooltip-loading">No topics available</div>';
                }} else {{
                    tooltipContent.innerHTML = data.topics.map(topic => `
                        <div class="tooltip-topic-item">
                            <span class="tooltip-topic-bullet">•</span>
                            <span class="tooltip-topic-name">${{topic.name}}</span>
                        </div>
                    `).join('');
                }}
            }} catch (error) {{
                console.error('Failed to update topics tooltip:', error);
            }}
        }}

        // Fetch and display nodes
        async function updateNodes() {{
            try {{
                const response = await fetch('/api/nodes');
                const data = await response.json();
                const nodesList = document.getElementById('nodes-list');

                if (data.nodes.length === 0) {{
                    nodesList.innerHTML = '<div class=\"placeholder\">No active nodes detected.<br><div class=\"command\" style=\"margin-top: 1rem;\"><span class=\"command-prompt\">$</span> horus run your_node.rs</div></div>';
                }} else {{
                    nodesList.innerHTML = data.nodes.map(node => `
                        <div class=\"node-item\" data-node-name=\"${{node.name}}\" title=\"Click to view logs\">
                            <div class=\"node-header\">
                                <span class=\"node-name\">${{node.name}}</span>
                                <span class=\"node-status status-${{node.health_color || 'gray'}}\">${{node.health}}</span>
                            </div>
                            <div class=\"node-details\">
                                <span>PID: ${{node.pid}}</span>
                                <span>CPU: ${{node.cpu}}</span>
                                <span>Memory: ${{node.memory}}</span>
                            </div>
                        </div>
                    `).join('');
                }}
            }} catch (error) {{
                console.error('Failed to fetch nodes:', error);
            }}
        }}

        // Fetch and display topics
        async function updateTopics() {{
            try {{
                const response = await fetch('/api/topics');
                const data = await response.json();
                const topicsList = document.getElementById('topics-list');

                if (data.topics.length === 0) {{
                    topicsList.innerHTML = '<div class=\"placeholder\">No topics available.</div>';
                }} else {{
                    const topicsHtml = data.topics.map(topic => `
                        <div class=\"topic-item\" data-topic-name=\"${{topic.name}}\" title=\"Click to view logs\">
                            <div class=\"topic-header\">
                                <span class=\"topic-name\">${{topic.name}}</span>
                                <span class=\"node-status status-running\">${{topic.active ? 'Active' : 'Inactive'}}</span>
                            </div>
                            <div class=\"topic-details\">
                                <span>Size: ${{topic.size}}</span>
                                <span>Processes: ${{topic.processes}}</span>
                            </div>
                        </div>
                    `).join('');
                    topicsList.innerHTML = topicsHtml;
                }}
            }} catch (error) {{
                console.error('Failed to fetch topics:', error);
            }}
        }}

        // Interactive graph state (global for access from renderGraph and resize)
        window.graphState = {{
            nodePositions: {{}},
            nodes: [],
            edges: [],
            hoveredNode: null,
            draggedNode: null,
            mouseX: 0,
            mouseY: 0,
            offsetX: 0,
            offsetY: 0,
            isDragging: false,
            dragStartX: 0,
            dragStartY: 0
        }};
        const graphState = window.graphState; // Local alias for backward compatibility

        // Initialize canvas event listeners
        function initGraphInteraction() {{
            const canvas = document.getElementById('graph-canvas');
            if (!canvas) return;

            canvas.addEventListener('mousemove', (e) => {{
                const rect = canvas.getBoundingClientRect();
                const scaleX = canvas.width / rect.width;
                const scaleY = canvas.height / rect.height;
                graphState.mouseX = (e.clientX - rect.left) * scaleX;
                graphState.mouseY = (e.clientY - rect.top) * scaleY;

                // Handle dragging
                if (graphState.draggedNode) {{
                    graphState.nodePositions[graphState.draggedNode].x = graphState.mouseX - graphState.offsetX;
                    graphState.nodePositions[graphState.draggedNode].y = graphState.mouseY - graphState.offsetY;

                    // Mark as dragging if mouse moved more than 5 pixels
                    const dx = graphState.mouseX - graphState.dragStartX;
                    const dy = graphState.mouseY - graphState.dragStartY;
                    if (Math.sqrt(dx*dx + dy*dy) > 5) {{
                        graphState.isDragging = true;
                    }}
                }} else {{
                    // Check for hovered node (works for both circles and triangles)
                    graphState.hoveredNode = null;
                    Object.keys(graphState.nodePositions).forEach(nodeId => {{
                        const pos = graphState.nodePositions[nodeId];
                        const dx = graphState.mouseX - pos.x;
                        const dy = graphState.mouseY - pos.y;
                        const dist = Math.sqrt(dx * dx + dy * dy);
                        // Use larger hit area (30px) to account for triangle shapes
                        if (dist < 30) {{
                            graphState.hoveredNode = nodeId;
                        }}
                    }});
                    canvas.style.cursor = graphState.hoveredNode ? 'pointer' : 'default';
                }}
            }});

            canvas.addEventListener('mousedown', (e) => {{
                if (graphState.hoveredNode) {{
                    graphState.draggedNode = graphState.hoveredNode;
                    const pos = graphState.nodePositions[graphState.draggedNode];
                    graphState.offsetX = graphState.mouseX - pos.x;
                    graphState.offsetY = graphState.mouseY - pos.y;
                    graphState.dragStartX = graphState.mouseX;
                    graphState.dragStartY = graphState.mouseY;
                    graphState.isDragging = false; // Reset drag flag
                    canvas.style.cursor = 'grabbing';
                    e.preventDefault(); // Prevent text selection while dragging
                }}
            }});

            canvas.addEventListener('mouseup', () => {{
                graphState.draggedNode = null;
                canvas.style.cursor = graphState.hoveredNode ? 'pointer' : 'default';
            }});

            canvas.addEventListener('mouseleave', () => {{
                graphState.draggedNode = null;
                graphState.hoveredNode = null;
                canvas.style.cursor = 'default';
            }});

            canvas.addEventListener('click', (e) => {{
                // Don't open log panel if user was dragging
                if (graphState.isDragging) {{
                    graphState.isDragging = false; // Reset for next interaction
                    return;
                }}

                if (graphState.hoveredNode) {{
                    // Find the node data
                    const clickedNode = graphData.nodes.find(n => n.id === graphState.hoveredNode);
                    if (clickedNode) {{
                        if (clickedNode.type === 'process') {{
                            showNodeLogs(clickedNode.label);
                        }} else if (clickedNode.type === 'topic') {{
                            showTopicLogs(clickedNode.label);
                        }}
                    }}
                }}
            }});
        }}

        // Enhanced graph renderer with glowing nodes
        function renderGraph(nodes, edges) {{
            const canvas = document.getElementById('graph-canvas');
            if (!canvas) return;

            const ctx = canvas.getContext('2d');
            const width = canvas.width;
            const height = canvas.height;

            // Debug: Log canvas dimensions on first render
            if (!window.graphDebugLogged) {{
                window.graphDebugLogged = true;
            }}

            // Clear canvas
            ctx.fillStyle = 'rgb(10, 11, 13)';
            ctx.fillRect(0, 0, width, height);

            // Barycenter Heuristic Layout: Minimizes edge crossings in bipartite graph
            const processNodes = nodes.filter(n => n.type === 'process');
            const topicNodes = nodes.filter(n => n.type === 'topic');

            if (topicNodes.length === 0) {{
                console.warn(' No topic nodes found! Node types:', nodes.map(n => `${{n.id}}:${{n.type}}`));
            }}
            if (edges.length === 0) {{
                console.warn(' No edges found! Check if processes are publishing/subscribing to topics.');
            }}

            // Ensure nodePositions object exists
            if (!graphState.nodePositions) {{
                graphState.nodePositions = {{}};
            }}

            // Initialize positions only if not already set (preserve drag positions)
            // OR if we have new nodes that don't have positions yet
            const needsLayout = Object.keys(graphState.nodePositions).length === 0 ||
                                nodes.some(n => !graphState.nodePositions[n.id]);

            if (needsLayout) {{

                // Step 1: Build adjacency maps
                const processToTopics = {{}};  // process_id -> [topic_ids]
                const topicToProcesses = {{}}; // topic_id -> [process_ids]

                edges.forEach(edge => {{
                    const isProcessToTopic = processNodes.some(p => p.id === edge.from);
                    if (isProcessToTopic) {{
                        if (!processToTopics[edge.from]) processToTopics[edge.from] = [];
                        processToTopics[edge.from].push(edge.to);
                        if (!topicToProcesses[edge.to]) topicToProcesses[edge.to] = [];
                        topicToProcesses[edge.to].push(edge.from);
                    }} else {{
                        if (!topicToProcesses[edge.from]) topicToProcesses[edge.from] = [];
                        topicToProcesses[edge.from].push(edge.to);
                        if (!processToTopics[edge.to]) processToTopics[edge.to] = [];
                        processToTopics[edge.to].push(edge.from);
                    }}
                }});

                // Step 2: Initial ordering (by ID for deterministic results)
                let processOrder = [...processNodes].sort((a, b) => a.id.localeCompare(b.id));
                let topicOrder = [...topicNodes].sort((a, b) => a.id.localeCompare(b.id));

                // Step 3: Barycenter iterations (5 iterations for convergence)
                for (let iter = 0; iter < 5; iter++) {{
                    // 3a. Reorder topics based on average Y of connected processes
                    topicOrder = topicOrder.map(topic => {{
                        const connectedProcesses = topicToProcesses[topic.id] || [];
                        if (connectedProcesses.length === 0) return {{ node: topic, barycenter: 0 }};

                        const avgIndex = connectedProcesses.reduce((sum, procId) => {{
                            const index = processOrder.findIndex(p => p.id === procId);
                            return sum + (index >= 0 ? index : 0);
                        }}, 0) / connectedProcesses.length;

                        return {{ node: topic, barycenter: avgIndex }};
                    }}).sort((a, b) => a.barycenter - b.barycenter).map(item => item.node);

                    // 3b. Reorder processes based on average Y of connected topics
                    processOrder = processOrder.map(process => {{
                        const connectedTopics = processToTopics[process.id] || [];
                        if (connectedTopics.length === 0) return {{ node: process, barycenter: 0 }};

                        const avgIndex = connectedTopics.reduce((sum, topicId) => {{
                            const index = topicOrder.findIndex(t => t.id === topicId);
                            return sum + (index >= 0 ? index : 0);
                        }}, 0) / connectedTopics.length;

                        return {{ node: process, barycenter: avgIndex }};
                    }}).sort((a, b) => a.barycenter - b.barycenter).map(item => item.node);
                }}

                // Step 4: Calculate final positions with even spacing
                const margin = 80;
                const processX = 180;
                const topicX = width - 250;

                // Calculate optimal spacing
                const processSpacing = processOrder.length > 1
                    ? Math.min(120, (height - 2 * margin) / (processOrder.length - 1))
                    : 0;
                const topicSpacing = topicOrder.length > 1
                    ? Math.min(100, (height - 2 * margin) / (topicOrder.length - 1))
                    : 0;

                // Position processes (only if they don't already have positions)
                const processTotalHeight = (processOrder.length - 1) * processSpacing;
                const processStartY = (height - processTotalHeight) / 2;

                processOrder.forEach((node, i) => {{
                    // Skip if already positioned (preserve manual drag positions)
                    if (!graphState.nodePositions[node.id]) {{
                        const y = processOrder.length === 1
                            ? height / 2
                            : processStartY + i * processSpacing;
                        graphState.nodePositions[node.id] = {{
                            x: processX,
                            y: Math.max(margin, Math.min(height - margin, y))
                        }};
                    }}
                }});

                // Position topics (only if they don't already have positions)
                const topicTotalHeight = (topicOrder.length - 1) * topicSpacing;
                const topicStartY = (height - topicTotalHeight) / 2;

                topicOrder.forEach((node, i) => {{
                    // Skip if already positioned (preserve manual drag positions)
                    if (!graphState.nodePositions[node.id]) {{
                        const y = topicOrder.length === 1
                            ? height / 2
                            : topicStartY + i * topicSpacing;
                        graphState.nodePositions[node.id] = {{
                            x: topicX,
                            y: Math.max(margin, Math.min(height - margin, y))
                        }};
                    }}
                }});

            }}

            // Draw edges with smooth Bézier curves (like rqt_graph)
            edges.forEach(edge => {{
                const from = graphState.nodePositions[edge.from];
                const to = graphState.nodePositions[edge.to];
                if (!from || !to) {{
                    console.warn(` Edge ${{edge.from}} -> ${{edge.to}} missing positions. From: ${{!!from}}, To: ${{!!to}}`);
                    return;
                }}

                // Calculate control points for smooth S-curve (Bézier curve)
                const dx = to.x - from.x;
                const dy = to.y - from.y;
                const controlDistance = Math.abs(dx) * 0.5; // Horizontal control distance

                // Control points create the smooth curve
                const cp1x = from.x + controlDistance;
                const cp1y = from.y;
                const cp2x = to.x - controlDistance;
                const cp2y = to.y;

                // Edge styling based on type
                const edgeColor = edge.type === 'publish' ? 'rgba(0, 212, 255, 0.8)' : 'rgba(0, 255, 136, 0.8)';
                const arrowColor = edge.type === 'publish' ? 'rgba(0, 212, 255, 1.0)' : 'rgba(0, 255, 136, 1.0)';

                // Draw smooth Bézier curve
                ctx.beginPath();
                ctx.moveTo(from.x, from.y);
                ctx.bezierCurveTo(cp1x, cp1y, cp2x, cp2y, to.x, to.y);
                ctx.strokeStyle = edgeColor;
                ctx.lineWidth = 2;
                ctx.stroke();

                // Calculate arrow position and angle at the end of the curve
                // We need to find the tangent angle at t=1 (end of curve)
                // For cubic Bézier: tangent = 3*(1-t)²*(P1-P0) + 6*(1-t)*t*(P2-P1) + 3*t²*(P3-P2)
                // At t=1: tangent = 3*(P3-P2) = direction from last control point to end
                const tangentX = to.x - cp2x;
                const tangentY = to.y - cp2y;
                const angle = Math.atan2(tangentY, tangentX);

                const arrowHeadLen = 12;
                const nodeRadius = 20;

                // Position arrowhead at node boundary
                const arrowTipX = to.x - nodeRadius * Math.cos(angle);
                const arrowTipY = to.y - nodeRadius * Math.sin(angle);

                // Draw filled arrowhead
                ctx.beginPath();
                ctx.moveTo(arrowTipX, arrowTipY);
                ctx.lineTo(
                    arrowTipX - arrowHeadLen * Math.cos(angle - Math.PI / 7),
                    arrowTipY - arrowHeadLen * Math.sin(angle - Math.PI / 7)
                );
                ctx.lineTo(
                    arrowTipX - arrowHeadLen * Math.cos(angle + Math.PI / 7),
                    arrowTipY - arrowHeadLen * Math.sin(angle + Math.PI / 7)
                );
                ctx.closePath();
                ctx.fillStyle = arrowColor;
                ctx.fill();
            }});

            // Draw nodes - RQT style: Circles for processes, Triangles for topics
            nodes.forEach(node => {{
                const pos = graphState.nodePositions[node.id];
                if (!pos) return;

                const nodeSize = 15;

                // Different colors for processes vs topics
                const color = node.type === 'process'
                    ? {{ r: 0, g: 255, b: 136 }}     // Green for processes
                    : {{ r: 255, g: 20, b: 147 }};   // Pink for topics

                if (node.type === 'process') {{
                    // PROCESSES: Draw as circles (no glow, no shadows)
                    ctx.beginPath();
                    ctx.arc(pos.x, pos.y, nodeSize, 0, 2 * Math.PI);
                    ctx.fillStyle = `rgb(${{color.r}}, ${{color.g}}, ${{color.b}})`;
                    ctx.fill();
                }} else {{
                    // TOPICS: Draw as rectangles (RQT style, no glow, no shadows)
                    // Topic names use dot notation - no conversion needed
                    const topicName = node.label;

                    // Dynamically size rectangle based on text length
                    ctx.font = '10px JetBrains Mono, monospace';
                    const textWidth = ctx.measureText(topicName).width;
                    const rectWidth = Math.max(nodeSize * 3, textWidth + 20);
                    const rectHeight = nodeSize * 1.5;

                    // Main rectangle (no glow, no shadows)
                    ctx.fillStyle = `rgb(${{color.r}}, ${{color.g}}, ${{color.b}})`;
                    ctx.fillRect(
                        pos.x - rectWidth/2,
                        pos.y - rectHeight/2,
                        rectWidth,
                        rectHeight
                    );

                    // Draw topic name INSIDE the rectangle
                    ctx.fillStyle = 'rgb(10, 11, 13)';
                    ctx.font = '400 10px JetBrains Mono, monospace';
                    ctx.textAlign = 'center';
                    ctx.textBaseline = 'middle';
                    ctx.fillText(topicName, pos.x, pos.y);
                }}

                // Draw label for processes only (below the circle, no shadows)
                if (node.type === 'process') {{
                    ctx.fillStyle = 'rgb(226, 232, 240)';
                    ctx.font = '400 11px JetBrains Mono, monospace';
                    ctx.textAlign = 'center';
                    ctx.textBaseline = 'top';
                    const label = node.label.length > 15 ? node.label.substring(0, 12) + '...' : node.label;
                    ctx.fillText(label, pos.x, pos.y + 25);
                }}
            }});
        }}

        // Initialize interaction once
        initGraphInteraction();

        // Handle window resize for canvas
        window.addEventListener('resize', () => {{
            const canvas = document.getElementById('graph-canvas');
            if (canvas && canvas.offsetParent !== null) {{ // Only resize if visible
                const rect = canvas.getBoundingClientRect();
                const oldWidth = canvas.width;
                const oldHeight = canvas.height;
                canvas.width = rect.width;
                canvas.height = rect.height;

                // Re-render if dimensions changed
                if (oldWidth !== canvas.width || oldHeight !== canvas.height) {{
                    if (window.graphState && window.graphState.nodes) {{
                        renderGraph(window.graphState.nodes, window.graphState.edges);
                    }}
                }}
            }}
        }});

        // Simple graph rendering - no animations
        let graphData = {{ nodes: [], edges: [] }};

        // Update graph data
        async function updateGraphData() {{
            try {{
                const response = await fetch('/api/graph');
                const data = await response.json();
                if (data.edges && data.edges.length > 0) {{
                }} else {{
                    console.warn(' No edges found! Cannot draw connection lines.');
                }}
                graphData = data;
                // Store in global state for resize handler
                window.graphState.nodes = data.nodes || [];
                window.graphState.edges = data.edges || [];

                // Render once when data changes
                if (graphData.nodes.length > 0) {{
                    renderGraph(graphData.nodes, graphData.edges);
                }}
            }} catch (error) {{
                console.error('Failed to fetch graph:', error);
            }}
        }}

        // Refresh all monitor data (triggers backend re-scan + updates frontend)
        // Backend performs fresh system scan on each API call - no caching
        async function refreshMonitorData() {{

            // Reset graph layout to default positions
            resetGraphLayout();

            await Promise.all([
                updateNodes(),      // Re-scans processes
                updateTopics(),     // Re-scans shared memory
                updateGraphData()   // Re-builds graph from fresh data
            ]);
        }}

        // Cache frequently accessed DOM elements for performance
        let cachedDOMElements = null;
        function getCachedElements() {{
            if (!cachedDOMElements) {{
                cachedDOMElements = {{
                    logPanel: document.getElementById('log-panel'),
                    logPanelTitle: document.getElementById('log-panel-title'),
                    logPanelContent: document.getElementById('log-panel-content'),
                    nodeCount: document.getElementById('node-count'),
                    topicCount: document.getElementById('topic-count')
                }};
            }}
            return cachedDOMElements;
        }}

        // Track current log view for auto-updates
        let currentLogView = {{ type: null, name: null, interval: null }};

        // Log panel functions (defined early so onclick handlers can use them)
        async function showNodeLogs(nodeName) {{
            const {{ logPanel: panel, logPanelTitle: title, logPanelContent: content }} = getCachedElements();

            if (!panel || !title || !content) {{
                return;
            }}

            // Stop previous auto-update and cancel any pending requests
            if (currentLogView.interval) {{
                clearInterval(currentLogView.interval);
                currentLogView.interval = null;
            }}

            // Set new view context BEFORE starting updates
            currentLogView = {{ type: 'node', name: nodeName, interval: null }};

            title.textContent = `Logs: ${{nodeName}} (live)`;
            content.innerHTML = '<p style="color: var(--text-secondary);">Loading logs...</p>';
            panel.classList.add('open');

            async function updateLogs() {{
                // Guard: Only update if this is still the active view
                if (currentLogView.type !== 'node' || currentLogView.name !== nodeName) {{
                    return; // User switched to a different log view
                }}

                try {{
                    const response = await fetch(`/api/logs/node/${{encodeURIComponent(nodeName)}}`);
                    const data = await response.json();

                    // Guard again after async operation
                    if (currentLogView.type !== 'node' || currentLogView.name !== nodeName) {{
                        return; // View changed during fetch
                    }}

                    if (data.logs && data.logs.length > 0) {{
                        const wasScrolledToBottom = content.scrollHeight - content.scrollTop <= content.clientHeight + 50;

                        content.innerHTML = data.logs.slice(-100).map(log => `
                            <div class="log-entry">
                                <div class="log-entry-header">
                                    <span class="log-timestamp">${{log.timestamp}}</span>
                                    <span class="log-type log-type-${{log.log_type.toLowerCase()}}">${{log.log_type}}</span>
                                </div>
                                ${{log.topic ? `<div style="color: var(--text-tertiary); font-size: 0.75rem;">Topic: ${{log.topic}}</div>` : ''}}
                                <div class="log-message">${{log.message}}</div>
                                <div style="color: var(--text-tertiary); font-size: 0.7rem; margin-top: 0.5rem;">
                                    Tick: ${{log.tick_us}}μs | IPC: ${{log.ipc_ns}}ns
                                </div>
                            </div>
                        `).join('');

                        if (wasScrolledToBottom) {{
                            content.scrollTop = content.scrollHeight;
                        }}
                    }} else {{
                        content.innerHTML = '<p style="color: var(--text-secondary);">No logs found for this node</p>';
                    }}
                }} catch (error) {{
                    content.innerHTML = `<p style="color: #ff4444;">Error loading logs: ${{error.message}}</p>`;
                }}
            }}

            await updateLogs();
            // Update logs every 200ms (5 times per second) - reasonable balance
            currentLogView.interval = setInterval(updateLogs, 200);
        }}

        async function showTopicLogs(topicName) {{
            const {{ logPanel: panel, logPanelTitle: title, logPanelContent: content }} = getCachedElements();

            if (!panel || !title || !content) {{
                return;
            }}

            // Stop previous auto-update and cancel any pending requests
            if (currentLogView.interval) {{
                clearInterval(currentLogView.interval);
                currentLogView.interval = null;
            }}

            // Set new view context BEFORE starting updates
            currentLogView = {{ type: 'topic', name: topicName, interval: null }};

            title.textContent = `Logs: ${{topicName}} (live)`;
            content.innerHTML = '<p style="color: var(--text-secondary);">Loading logs...</p>';
            panel.classList.add('open');

            async function updateLogs() {{
                // Guard: Only update if this is still the active view
                if (currentLogView.type !== 'topic' || currentLogView.name !== topicName) {{
                    return; // User switched to a different log view
                }}

                try {{
                    const response = await fetch(`/api/logs/topic/${{encodeURIComponent(topicName)}}`);
                    const data = await response.json();

                    // Guard again after async operation
                    if (currentLogView.type !== 'topic' || currentLogView.name !== topicName) {{
                        return; // View changed during fetch
                    }}

                    if (data.logs && data.logs.length > 0) {{
                        const wasScrolledToBottom = content.scrollHeight - content.scrollTop <= content.clientHeight + 50;

                        content.innerHTML = data.logs.slice(-100).map(log => {{
                            // Convert log type to topic-centric description
                            let operation = log.log_type;
                            if (log.log_type === 'Publish') {{
                                operation = 'Write';
                            }} else if (log.log_type === 'Subscribe') {{
                                operation = 'Read';
                            }} else if (log.log_type === 'TopicMap') {{
                                operation = 'Map';
                            }} else if (log.log_type === 'TopicUnmap') {{
                                operation = 'Unmap';
                            }}

                            return `
                            <div class="log-entry">
                                <div class="log-entry-header">
                                    <span class="log-timestamp">${{log.timestamp}}</span>
                                    <span class="log-type log-type-${{log.log_type.toLowerCase()}}">${{operation}}</span>
                                </div>
                                <div style="color: var(--accent); font-size: 0.85rem; font-weight: 500;">by ${{log.node_name}}</div>
                                <div class="log-message">${{log.message}}</div>
                                ${{log.ipc_ns > 0 ? `<div style="color: var(--text-tertiary); font-size: 0.7rem; margin-top: 0.5rem;">
                                    Tick: ${{log.tick_us}}μs | IPC: ${{log.ipc_ns}}ns
                                </div>` : ''}}
                            </div>
                        `;
                        }}).join('');

                        if (wasScrolledToBottom) {{
                            content.scrollTop = content.scrollHeight;
                        }}
                    }} else {{
                        content.innerHTML = '<p style="color: var(--text-secondary);">No logs found for this topic</p>';
                    }}
                }} catch (error) {{
                    content.innerHTML = `<p style="color: #ff4444;">Error loading logs: ${{error.message}}</p>`;
                }}
            }}

            await updateLogs();
            // Update logs every 200ms (5 times per second) - reasonable balance
            currentLogView.interval = setInterval(updateLogs, 200);
        }}

        function closeLogPanel() {{
            // Stop auto-updates when panel closes
            if (currentLogView.interval) {{
                cancelAnimationFrame(currentLogView.interval);
                currentLogView = {{ type: null, name: null, interval: null }};
            }}
            document.getElementById('log-panel').classList.remove('open');
        }}

        // WebSocket connection for real-time updates
        let ws = null;
        let wsConnected = false;
        let pollingInterval = null;

        function connectWebSocket() {{
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${{protocol}}//${{window.location.host}}/api/ws`;

            ws = new WebSocket(wsUrl);

            ws.onopen = () => {{
                wsConnected = true;

                // Clear polling fallback if it was running
                if (pollingInterval) {{
                    clearInterval(pollingInterval);
                    pollingInterval = null;
                }}
            }};

            ws.onmessage = (event) => {{
                try {{
                    const update = JSON.parse(event.data);
                    if (update.type === 'update') {{
                        // Update nodes - create if missing, update if exists
                        if (update.data.nodes) {{
                            const nodesList = document.getElementById('nodes-list');
                            const existingNodes = nodesList.querySelectorAll('.node-item');

                            // If node count changed, do full refresh
                            if (existingNodes.length !== update.data.nodes.length) {{
                                if (update.data.nodes.length === 0) {{
                                    nodesList.innerHTML = '<div class=\"placeholder\">No active nodes detected.<br><div class=\"command\" style=\"margin-top: 1rem;\"><span class=\"command-prompt\">$</span> horus run your_node.rs</div></div>';
                                }} else {{
                                    nodesList.innerHTML = update.data.nodes.map(node => `
                                        <div class=\"node-item\" data-node-name=\"${{node.name}}\" title=\"Click to view logs\">
                                            <div class=\"node-header\">
                                                <span class=\"node-name\">${{node.name}}</span>
                                                <span class=\"node-status status-${{node.health_color || 'gray'}}\">${{node.health}}</span>
                                            </div>
                                            <div class=\"node-details\">
                                                <span>PID: ${{node.pid}}</span>
                                                <span>CPU: ${{node.cpu}}</span>
                                                <span>Memory: ${{node.memory}}</span>
                                            </div>
                                        </div>
                                    `).join('');
                                }}
                            }} else {{
                                // Just update stats for existing nodes
                                update.data.nodes.forEach(node => {{
                                    const nodeItem = document.querySelector(`[data-node-name="${{node.name}}"]`);
                                    if (nodeItem) {{
                                        // Update node details
                                        const details = nodeItem.querySelector('.node-details');
                                        if (details) {{
                                            details.innerHTML = `
                                                <span>PID: ${{node.pid}}</span>
                                                <span>CPU: ${{node.cpu}}</span>
                                                <span>Memory: ${{node.memory}}</span>
                                            `;
                                        }}

                                        // Update status badge color
                                        const statusBadge = nodeItem.querySelector('.node-status');
                                        if (statusBadge) {{
                                            statusBadge.className = `node-status status-${{node.health_color || 'gray'}}`;
                                            statusBadge.textContent = node.health;
                                        }}
                                    }}
                                }});
                            }}
                        }}

                        // Update topics - create if missing, update if exists
                        if (update.data.topics) {{
                            const topicsList = document.getElementById('topics-list');
                            const existingTopics = topicsList.querySelectorAll('.topic-item');

                            // If topic count changed, do full refresh
                            if (existingTopics.length !== update.data.topics.length) {{
                                if (update.data.topics.length === 0) {{
                                    topicsList.innerHTML = '<div class=\"placeholder\">No topics available.</div>';
                                }} else {{
                                    topicsList.innerHTML = update.data.topics.map(topic => `
                                        <div class=\"topic-item\" data-topic-name=\"${{topic.name}}\" title=\"Click to view logs\">
                                            <div class=\"topic-header\">
                                                <span class=\"topic-name\">${{topic.name}}</span>
                                                <span class=\"node-status status-running\">${{topic.active ? 'Active' : 'Inactive'}}</span>
                                            </div>
                                            <div class=\"topic-details\">
                                                <span>Size: ${{topic.size}}</span>
                                                <span>Processes: ${{topic.processes}}</span>
                                            </div>
                                        </div>
                                    `).join('');
                                }}
                            }} else {{
                                // Just update stats for existing topics
                                update.data.topics.forEach(topic => {{
                                    const topicItem = document.querySelector(`[data-topic-name="${{topic.name}}"]`);
                                    if (topicItem) {{
                                        const details = topicItem.querySelector('.topic-details');
                                        if (details) {{
                                            details.innerHTML = `
                                                <span>Size: ${{topic.size}}</span>
                                                <span>Processes: ${{topic.processes}}</span>
                                            `;
                                        }}
                                    }}
                                }});
                            }}
                        }}

                        // Update graph data
                        if (update.data.graph) {{
                            graphData = update.data.graph;
                            window.graphState.nodes = update.data.graph.nodes || [];
                            window.graphState.edges = update.data.graph.edges || [];

                            // Render once when data arrives
                            if (graphData.nodes.length > 0) {{
                                renderGraph(graphData.nodes, graphData.edges);
                            }}
                        }}

                        // Update status bar and tooltips
                        updateStatus();
                        updateNodesToolTip();
                        updateTopicsToolTip();
                    }}
                }} catch (error) {{
                    console.error('WebSocket message parse error:', error);
                }}
            }};

            ws.onerror = (error) => {{
                console.warn(' WebSocket error, falling back to polling');
                wsConnected = false;
            }};

            ws.onclose = () => {{
                wsConnected = false;

                // Fallback to polling (1 second interval to reduce load)
                if (!pollingInterval) {{
                    pollingInterval = setInterval(updateAll, 1000);
                }}

                // Try to reconnect after 5 seconds
                setTimeout(connectWebSocket, 5000);
            }};
        }}

        // Update all data (polling fallback)
        function updateAll() {{
            updateStatus();
            updateNodes();
            updateTopics();
            updateGraphData();
            updateNodesToolTip();
            updateTopicsToolTip();
        }}

        // Event delegation for node and topic clicks - SET UP EARLY!

        try {{
            document.addEventListener('click', (e) => {{
                // Check if click is on a node item
                const nodeItem = e.target.closest('.node-item');
                if (nodeItem) {{
                    e.preventDefault();
                    e.stopPropagation();
                    const nodeName = nodeItem.getAttribute('data-node-name');
                    if (nodeName) {{
                        showNodeLogs(nodeName);
                    }}
                    return;
                }}

                // Check if click is on a topic item
                const topicItem = e.target.closest('.topic-item');
                if (topicItem) {{
                    e.preventDefault();
                    e.stopPropagation();
                    const topicName = topicItem.getAttribute('data-topic-name');
                    if (topicName) {{
                        showTopicLogs(topicName);
                    }}
                    return;
                }}
            }}, true);

        }} catch (err) {{
            console.error('Failed to attach event delegation:', err);
        }}

        // Try WebSocket first, fallback to polling
        connectWebSocket();

        // Initial load via polling (in case WebSocket takes time to connect)
        updateAll();

        // Theme toggle functionality
        function toggleTheme() {{
            const html = document.documentElement;
            const currentTheme = html.getAttribute('data-theme');
            const newTheme = currentTheme === 'light' ? 'dark' : 'light';
            const themeIcon = document.getElementById('theme-icon');

            html.setAttribute('data-theme', newTheme);

            // Update icon: Moon for dark theme, Sun for light theme
            if (newTheme === 'light') {{
                // Sun icon
                themeIcon.innerHTML = '<circle cx="12" cy="12" r="5"></circle><line x1="12" y1="1" x2="12" y2="3"></line><line x1="12" y1="21" x2="12" y2="23"></line><line x1="4.22" y1="4.22" x2="5.64" y2="5.64"></line><line x1="18.36" y1="18.36" x2="19.78" y2="19.78"></line><line x1="1" y1="12" x2="3" y2="12"></line><line x1="21" y1="12" x2="23" y2="12"></line><line x1="4.22" y1="19.78" x2="5.64" y2="18.36"></line><line x1="18.36" y1="5.64" x2="19.78" y2="4.22"></line>';
            }} else {{
                // Moon icon
                themeIcon.innerHTML = '<path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z"></path>';
            }}

            // Save preference to localStorage
            localStorage.setItem('horus-theme', newTheme);
        }}

        // Load saved theme preference
        function loadTheme() {{
            const savedTheme = localStorage.getItem('horus-theme') || 'dark';
            const html = document.documentElement;
            const themeIcon = document.getElementById('theme-icon');

            html.setAttribute('data-theme', savedTheme);

            // Set correct icon based on saved theme
            if (savedTheme === 'light') {{
                // Sun icon
                themeIcon.innerHTML = '<circle cx="12" cy="12" r="5"></circle><line x1="12" y1="1" x2="12" y2="3"></line><line x1="12" y1="21" x2="12" y2="23"></line><line x1="4.22" y1="4.22" x2="5.64" y2="5.64"></line><line x1="18.36" y1="18.36" x2="19.78" y2="19.78"></line><line x1="1" y1="12" x2="3" y2="12"></line><line x1="21" y1="12" x2="23" y2="12"></line><line x1="4.22" y1="19.78" x2="5.64" y2="18.36"></line><line x1="18.36" y1="5.64" x2="19.78" y2="4.22"></line>';
            }} else {{
                // Moon icon
                themeIcon.innerHTML = '<path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z"></path>';
            }}
        }}

        // Load theme on page load
        loadTheme();

        // Help modal functions
        function toggleHelp() {{
            const modal = document.getElementById('help-modal');
            modal.classList.toggle('active');
        }}

        // Keyboard shortcuts
        document.addEventListener('keydown', function(e) {{
            // Press '?' to open help
            if (e.key === '?' && !e.ctrlKey && !e.altKey && !e.metaKey) {{
                const activeElement = document.activeElement;
                // Don't trigger if typing in an input
                if (activeElement.tagName !== 'INPUT' && activeElement.tagName !== 'TEXTAREA') {{
                    e.preventDefault();
                    toggleHelp();
                }}
            }}

            // Press 'Esc' to close help modal
            if (e.key === 'Escape') {{
                const modal = document.getElementById('help-modal');
                if (modal.classList.contains('active')) {{
                    toggleHelp();
                }}
            }}
        }});

        // Close modal when clicking outside of it
        document.getElementById('help-modal').addEventListener('click', function(e) {{
            if (e.target === this) {{
                toggleHelp();
            }}
        }});

        // Package view switching
        function switchPackageView(view) {{
            // Hide all package views
            document.querySelectorAll('.package-view').forEach(v => v.style.display = 'none');

            // Remove active from all view buttons
            const packageTab = document.getElementById('tab-packages');
            packageTab.querySelectorAll('.view-btn').forEach(btn => btn.classList.remove('active'));

            // Show selected view and activate button
            if (view === 'global') {{
                document.getElementById('package-global').style.display = 'block';
                event.target.classList.add('active');
                loadGlobalEnvironment();
            }} else if (view === 'local') {{
                document.getElementById('package-local').style.display = 'block';
                event.target.classList.add('active');
                loadLocalEnvironments();
            }} else if (view === 'registry') {{
                document.getElementById('package-registry').style.display = 'block';
                event.target.classList.add('active');
            }}
        }}

        async function loadEnvironments() {{
            const container = document.getElementById('environments-list');
            try {{
                const response = await fetch('/api/packages/environments');
                const data = await response.json();

                let html = '';

                // Global Environment Section
                html += '<div style="margin-bottom: 30px;">';
                html += '<h3 style="color: var(--accent); margin-bottom: 15px; display: flex; align-items: center; gap: 8px;">';
                html += 'Global Environment';
                html += `<span style="font-size: 0.8em; color: var(--text-secondary); font-weight: normal;">(${{data.global?.length || 0}} packages)</span>`;
                html += '</h3>';

                if (data.global && data.global.length > 0) {{
                    html += data.global.map(pkg => `
                        <div style="padding: 12px 15px; background: var(--surface); border: 1px solid var(--border); border-radius: 6px; margin-bottom: 8px;">
                            <div style="display: flex; justify-content: space-between; align-items: center;">
                                <div>
                                    <span style="font-weight: 600; color: var(--text-primary);">${{pkg.name}}</span>
                                    <span style="color: var(--text-secondary); margin-left: 10px; font-size: 0.9em;">v${{pkg.version}}</span>
                                </div>
                            </div>
                        </div>
                    `).join('');
                }} else {{
                    html += '<p style="color: var(--text-secondary); font-size: 0.9em;">No global packages installed</p>';
                }}
                html += '</div>';

                // Local Environments Section
                html += '<div>';
                html += '<h3 style="color: var(--success); margin-bottom: 15px; display: flex; align-items: center; gap: 8px;">';
                html += 'Local Environments';
                html += `<span style="font-size: 0.8em; color: var(--text-secondary); font-weight: normal;">(${{data.local?.length || 0}} environments)</span>`;
                html += '</h3>';

                if (data.local && data.local.length > 0) {{
                    html += data.local.map((env, index) => {{
                        let expandableContent = '';

                        const hasDependencies = env.dependencies && env.dependencies.length > 0;

                        if (hasDependencies) {{
                            expandableContent = `
                                <div id="env-details-${{index}}" style="display: none; margin-top: 10px; padding-top: 10px; border-top: 1px solid var(--border);">
                                    <div style="color: var(--text-secondary); margin-bottom: 8px; font-size: 0.9em; font-weight: 600;">
                                        Dependencies (horus.yaml):
                                    </div>
                                    <div style="display: flex; flex-direction: column; gap: 4px;">
                                        ${{env.dependencies.map(dep => `
                                            <div style="padding: 6px 10px; background: var(--surface); border: 1px solid var(--success, #4ade80); border-radius: 4px; display: flex; justify-content: space-between; align-items: center; gap: 10px;">
                                                <div style="display: flex; align-items: center; gap: 10px; flex: 1;">
                                                    <span style="color: var(--text-primary); font-size: 0.85em;">${{dep.name}}</span>
                                                    <span style="color: var(--text-tertiary); font-size: 0.75em;">v${{dep.version}}</span>
                                                </div>
                                                <span style="color: var(--text-tertiary); font-size: 0.7em;">installed</span>
                                            </div>
                                        `).join('')}}
                                    </div>
                                </div>
                            `;
                        }} else {{
                            expandableContent = `
                                <div id="env-details-${{index}}" style="display: none; margin-top: 10px; padding-top: 10px; border-top: 1px solid var(--border);">
                                    <div style="color: var(--text-tertiary); font-size: 0.85em; font-style: italic;">
                                        No dependencies declared in horus.yaml
                                    </div>
                                </div>
                            `;
                        }}

                        const depCount = env.dependencies?.length || 0;
                        const hasContent = depCount > 0;

                        return `
                            <div class="package-item" style="padding: 15px; background: var(--surface); border: 1px solid var(--border); border-radius: 6px; margin-bottom: 10px; ${{hasContent ? 'cursor: pointer;' : ''}}">
                                <div style="display: flex; justify-content: space-between; align-items: center;" onclick="${{hasContent ? `toggleEnvDetails(${{index}})` : ''}}">
                                    <div style="flex: 1;">
                                        <div style="display: flex; align-items: center; gap: 8px;">
                                            <span style="font-weight: 600; color: var(--text-primary); font-size: 1.05em;">
                                                ${{env.name}}
                                            </span>
                                            ${{hasContent ? '<span id="arrow-' + index + '" style="color: var(--accent); font-size: 0.9em;">▶</span>' : ''}}
                                        </div>
                                        <div style="color: var(--text-secondary); margin-top: 5px; font-size: 0.85em;">
                                            ${{env.path}} • ${{depCount}} dependencies
                                        </div>
                                    </div>
                                </div>
                                ${{expandableContent}}
                            </div>
                        `;
                    }}).join('');
                }} else {{
                    html += '<p style="color: var(--text-secondary); font-size: 0.9em;">No local environments found</p>';
                }}
                html += '</div>';

                container.innerHTML = html;
            }} catch (error) {{
                container.innerHTML = `<p style="color: var(--error);">Failed to load environments: ${{error.message}}</p>`;
            }}
        }}

        function toggleEnvDetails(index) {{
            const detailsDiv = document.getElementById(`env-details-${{index}}`);
            const arrow = document.getElementById(`arrow-${{index}}`);

            if (detailsDiv) {{
                const isVisible = detailsDiv.style.display !== 'none';
                detailsDiv.style.display = isVisible ? 'none' : 'block';
                if (arrow) {{
                    arrow.textContent = isVisible ? '▶' : '▼';
                }}
            }}
        }}

        function togglePackageDetails(envIndex, pkgIndex) {{
            const detailsDiv = document.getElementById(`pkg-details-${{envIndex}}-${{pkgIndex}}`);
            const arrow = document.getElementById(`pkg-arrow-${{envIndex}}-${{pkgIndex}}`);

            if (detailsDiv) {{
                const isVisible = detailsDiv.style.display !== 'none';
                detailsDiv.style.display = isVisible ? 'none' : 'block';
                if (arrow) {{
                    arrow.textContent = isVisible ? '▶' : '▼';
                }}
            }}
        }}

        async function searchRegistry() {{
            const query = document.getElementById('registry-search-input').value;
            const container = document.getElementById('registry-results');

            container.innerHTML = '<p style="color: var(--text-secondary);">Searching...</p>';

            try {{
                const response = await fetch(`/api/packages/registry?q=${{encodeURIComponent(query)}}`);
                const data = await response.json();

                if (data.error) {{
                    container.innerHTML = `<p style="color: var(--error);">${{data.error}}</p>`;
                    return;
                }}

                if (!data.packages || data.packages.length === 0) {{
                    container.innerHTML = '<p style="color: var(--text-secondary);">No packages found</p>';
                    return;
                }}

                const html = data.packages.map(pkg => `
                    <div class="package-item" style="padding: 15px; background: var(--surface); border: 1px solid var(--border); border-radius: 6px; margin-bottom: 10px;">
                        <div style="display: flex; justify-content: space-between; align-items: center;">
                            <div style="flex: 1;">
                                <div style="font-weight: 600; color: var(--text-primary); font-size: 1.1em;">
                                    ${{pkg.name}}
                                </div>
                                <div style="color: var(--text-secondary); margin-top: 5px; font-size: 0.9em;">
                                    Version: ${{pkg.version}}
                                </div>
                                <div style="color: var(--text-secondary); margin-top: 5px; font-size: 0.9em;">
                                    ${{pkg.description}}
                                </div>
                            </div>
                            <button
                                onclick="showInstallDialog('${{pkg.name}}')"
                                style="padding: 8px 16px; background: var(--accent); color: var(--primary); border: none; border-radius: 4px; cursor: pointer; font-weight: 600; font-family: 'JetBrains Mono', monospace;"
                            >
                                Install
                            </button>
                        </div>
                    </div>
                `).join('');

                container.innerHTML = html;
            }} catch (error) {{
                container.innerHTML = `<p style="color: var(--error);">Search failed: ${{error.message}}</p>`;
            }}
        }}

        // Install dialog state
        let currentInstallPackage = null;
        let currentInstallLocation = 'global';

        async function showInstallDialog(packageName) {{
            currentInstallPackage = packageName;
            document.getElementById('install-pkg-name').textContent = packageName;
            document.getElementById('install-dialog').classList.add('active');

            // Reset to global selection
            selectInstallLocation('global');

            // Load local packages
            await loadLocalPackagesForInstall();
        }}

        function closeInstallDialog() {{
            document.getElementById('install-dialog').classList.remove('active');
            currentInstallPackage = null;
            currentInstallLocation = 'global';
        }}

        function selectInstallLocation(location) {{
            currentInstallLocation = location;

            // Update radio buttons
            document.getElementById('radio-global').checked = (location === 'global');
            document.getElementById('radio-local').checked = (location === 'local');

            // Update visual selection
            document.getElementById('install-option-global').classList.toggle('selected', location === 'global');
            document.getElementById('install-option-local').classList.toggle('selected', location === 'local');

            // Show/hide local package dropdown
            const dropdown = document.getElementById('local-package-select');
            dropdown.style.display = (location === 'local') ? 'block' : 'none';
        }}

        async function loadLocalPackagesForInstall() {{
            try {{
                const response = await fetch('/api/packages/environments');
                const data = await response.json();

                const dropdown = document.getElementById('local-package-select');
                dropdown.innerHTML = '<option value="">Select a package...</option>';

                if (data.local && data.local.length > 0) {{
                    data.local.forEach(env => {{
                        if (env.packages && env.packages.length > 0) {{
                            env.packages.forEach(pkg => {{
                                const option = document.createElement('option');
                                option.value = `${{env.path}}/.horus/packages/${{pkg.name}}`;
                                option.textContent = `${{env.name}}  ${{pkg.name}}`;
                                dropdown.appendChild(option);
                            }});
                        }}
                    }});
                }}
            }} catch (error) {{
                console.error('Failed to load local packages:', error);
            }}
        }}

        async function confirmInstall() {{
            if (!currentInstallPackage) return;

            let target = 'global';

            if (currentInstallLocation === 'local') {{
                const dropdown = document.getElementById('local-package-select');
                target = dropdown.value;

                if (!target) {{
                    alert('Please select a package to install into');
                    return;
                }}
            }}

            closeInstallDialog();

            try {{
                const response = await fetch('/api/packages/install', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify({{
                        package: currentInstallPackage,
                        target: target
                    }})
                }});

                const data = await response.json();

                if (data.success) {{
                    alert(`Successfully installed ${{currentInstallPackage}}`);
                    loadEnvironments();
                }} else {{
                    alert(`Failed to install: ${{data.error}}`);
                }}
            }} catch (error) {{
                alert(`Installation failed: ${{error.message}}`);
            }}
        }}

        async function installPackage(packageName) {{
            try {{
                const response = await fetch('/api/packages/install', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify({{ package: packageName }})
                }});

                const data = await response.json();

                if (data.success) {{
                    alert(`Successfully installed ${{packageName}}`);
                    loadEnvironments();
                }} else {{
                    alert(`Failed to install: ${{data.error}}`);
                }}
            }} catch (error) {{
                alert(`Installation failed: ${{error.message}}`);
            }}
        }}

        async function uninstallPackage(parentPackage, packageName, event) {{
            // Stop event propagation to prevent toggling the package details
            event.stopPropagation();

            if (!confirm(`Uninstall ${{packageName}} from ${{parentPackage}}?`)) return;

            try {{
                const response = await fetch('/api/packages/uninstall', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify({{
                        parent_package: parentPackage,
                        package: packageName
                    }})
                }});

                const data = await response.json();

                if (data.success) {{
                    alert(`Successfully uninstalled ${{packageName}}`);
                    loadEnvironments();
                }} else {{
                    alert(`Failed to uninstall: ${{data.error}}`);
                }}
            }} catch (error) {{
                alert(`Uninstallation failed: ${{error.message}}`);
            }}
        }}

        async function publishPackage() {{
            if (!confirm('Publish current directory as a package?')) return;

            try {{
                const response = await fetch('/api/packages/publish', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify({{}})
                }});

                const data = await response.json();

                if (data.success) {{
                    alert('Package published successfully!');
                    loadEnvironments();
                }} else {{
                    alert(`Failed to publish: ${{data.error}}`);
                }}
            }} catch (error) {{
                alert(`Error: ${{error.message}}`);
            }}
        }}

        // Initialize on tab switch
        function onPackagesTabActivate() {{
            loadGlobalEnvironment();
        }}

        async function loadGlobalEnvironment() {{
            const container = document.getElementById('global-packages-list');
            try {{
                const response = await fetch('/api/packages/environments');
                const data = await response.json();

                if (data.global && data.global.length > 0) {{
                    const html = data.global.map(pkg => `
                        <div style="padding: 12px 15px; background: var(--surface); border: 1px solid var(--border); border-radius: 6px; margin-bottom: 8px;">
                            <div style="display: flex; justify-content: space-between; align-items: center;">
                                <div>
                                    <span style="font-weight: 600; color: var(--text-primary);">${{pkg.name}}</span>
                                    <span style="color: var(--text-secondary); margin-left: 10px; font-size: 0.9em;">v${{pkg.version}}</span>
                                </div>
                            </div>
                        </div>
                    `).join('');
                    container.innerHTML = html;
                }} else {{
                    container.innerHTML = '<p style="color: var(--text-secondary); font-size: 0.9em;">No global packages installed in ~/.horus/cache</p>';
                }}
            }} catch (error) {{
                container.innerHTML = `<p style="color: var(--error);">Failed to load global packages: ${{error.message}}</p>`;
            }}
        }}

        async function loadLocalEnvironments() {{
            const container = document.getElementById('local-environments-list');
            try {{
                const response = await fetch('/api/packages/environments');
                const data = await response.json();

                if (data.local && data.local.length > 0) {{
                    const html = data.local.map((env, index) => {{
                        let expandableContent = '';

                        const hasDependencies = env.dependencies && env.dependencies.length > 0;

                        if (hasDependencies) {{
                            expandableContent = `
                                <div id="env-details-${{index}}" style="display: none; margin-top: 10px; padding-top: 10px; border-top: 1px solid var(--border);">
                                    <div style="color: var(--text-secondary); margin-bottom: 8px; font-size: 0.9em; font-weight: 600;">
                                        Dependencies (horus.yaml):
                                    </div>
                                    <div style="display: flex; flex-direction: column; gap: 4px;">
                                        ${{env.dependencies.map(dep => `
                                            <div style="padding: 6px 10px; background: var(--surface); border: 1px solid var(--success, #4ade80); border-radius: 4px; display: flex; justify-content: space-between; align-items: center; gap: 10px;">
                                                <div style="display: flex; align-items: center; gap: 10px; flex: 1;">
                                                    <span style="color: var(--text-primary); font-size: 0.85em;">${{dep.name}}</span>
                                                    <span style="color: var(--text-tertiary); font-size: 0.75em;">v${{dep.version}}</span>
                                                </div>
                                                <span style="color: var(--text-tertiary); font-size: 0.7em;">installed</span>
                                            </div>
                                        `).join('')}}
                                    </div>
                                </div>
                            `;
                        }} else {{
                            expandableContent = `
                                <div id="env-details-${{index}}" style="display: none; margin-top: 10px; padding-top: 10px; border-top: 1px solid var(--border);">
                                    <div style="color: var(--text-tertiary); font-size: 0.85em; font-style: italic;">
                                        No dependencies declared in horus.yaml
                                    </div>
                                </div>
                            `;
                        }}

                        const depCount = env.dependencies?.length || 0;
                        const hasContent = depCount > 0;

                        return `
                            <div class="package-item" style="padding: 15px; background: var(--surface); border: 1px solid var(--border); border-radius: 6px; margin-bottom: 10px; ${{hasContent ? 'cursor: pointer;' : ''}}">
                                <div style="display: flex; justify-content: space-between; align-items: center;" onclick="${{hasContent ? `toggleEnvDetails(${{index}})` : ''}}">
                                    <div style="flex: 1;">
                                        <div style="display: flex; align-items: center; gap: 8px;">
                                            <span style="font-weight: 600; color: var(--text-primary); font-size: 1.05em;">
                                                ${{env.name}}
                                            </span>
                                            ${{hasContent ? '<span id="arrow-' + index + '" style="color: var(--accent); font-size: 0.9em;">▶</span>' : ''}}
                                        </div>
                                        <div style="color: var(--text-secondary); margin-top: 5px; font-size: 0.85em;">
                                            ${{env.path}} • ${{depCount}} dependencies
                                        </div>
                                    </div>
                                </div>
                                ${{expandableContent}}
                            </div>
                        `;
                    }}).join('');
                    container.innerHTML = html;
                }} else {{
                    container.innerHTML = '<p style="color: var(--text-secondary); font-size: 0.9em;">No local HORUS environments found</p>';
                }}
            }} catch (error) {{
                container.innerHTML = `<p style="color: var(--error);">Failed to load local environments: ${{error.message}}</p>`;
            }}
        }}


        // Enter key support for search
        document.getElementById('search-input')?.addEventListener('keypress', (e) => {{
            if (e.key === 'Enter') searchPackages();
        }});

        // === Parameter Management Functions ===
        let allParams = [];
        let editingParam = null;

        async function refreshParams() {{
            try {{
                const response = await fetch('/api/params');
                const data = await response.json();

                if (data.success) {{
                    allParams = data.params;
                    renderParams(allParams);
                }}
            }} catch (error) {{
                console.error('Failed to fetch params:', error);
            }}
        }}

        function renderParams(params) {{
            const tbody = document.getElementById('params-table-body');

            if (params.length === 0) {{
                tbody.innerHTML = `
                    <tr>
                        <td colspan="4" style="padding: 2rem; text-align: center; color: var(--text-tertiary);">
                            No parameters found. Click "Add Parameter" to create one.
                        </td>
                    </tr>
                `;
                return;
            }}

            tbody.innerHTML = params.map(param => {{
                const valueDisplay = typeof param.value === 'object'
                    ? JSON.stringify(param.value)
                    : String(param.value);

                return `
                    <tr style="border-bottom: 1px solid var(--border); " onmouseover="this.style.background='var(--surface)'" onmouseout="this.style.background='transparent'">
                        <td style="padding: 0.75rem; font-weight: 600; color: var(--accent);">${{param.key}}</td>
                        <td style="padding: 0.75rem; color: var(--text-primary); font-family: 'JetBrains Mono', monospace;">${{valueDisplay}}</td>
                        <td style="padding: 0.75rem; color: var(--text-secondary);">
                            <span style="background: var(--surface); padding: 0.25rem 0.5rem; border-radius: 4px; font-size: 0.85rem;">${{param.type}}</span>
                        </td>
                        <td style="padding: 0.75rem; text-align: right;">
                            <button onclick="editParam('${{param.key}}')" style="padding: 0.25rem 0.75rem; background: var(--accent); color: white; border: none; border-radius: 6px; cursor: pointer; margin-right: 0.5rem; font-size: 0.85rem;">Edit</button>
                            <button onclick="deleteParam('${{param.key}}')" style="padding: 0.25rem 0.75rem; background: var(--error); color: white; border: none; border-radius: 6px; cursor: pointer; font-size: 0.85rem;">Delete</button>
                        </td>
                    </tr>
                `;
            }}).join('');
        }}

        function showAddParamDialog() {{
            editingParam = null;
            document.getElementById('dialog-title').textContent = 'Add Parameter';
            document.getElementById('param-key').value = '';
            document.getElementById('param-key').disabled = false;
            document.getElementById('param-type').value = 'number';
            document.getElementById('param-value').value = '';
            document.getElementById('param-dialog').style.display = 'flex';
        }}

        async function editParam(key) {{
            editingParam = key;
            const param = allParams.find(p => p.key === key);

            if (param) {{
                document.getElementById('dialog-title').textContent = 'Edit Parameter';
                document.getElementById('param-key').value = key;
                document.getElementById('param-key').disabled = true;
                document.getElementById('param-type').value = param.type;
                document.getElementById('param-value').value = typeof param.value === 'object'
                    ? JSON.stringify(param.value)
                    : String(param.value);
                document.getElementById('param-dialog').style.display = 'flex';
            }}
        }}

        function closeParamDialog() {{
            document.getElementById('param-dialog').style.display = 'none';
            editingParam = null;
        }}

        function updateValueInput() {{
            const type = document.getElementById('param-type').value;
            const valueInput = document.getElementById('param-value');

            if (type === 'boolean') {{
                valueInput.value = 'false';
                valueInput.placeholder = 'true or false';
            }} else if (type === 'number') {{
                valueInput.value = '0';
                valueInput.placeholder = 'Enter number';
            }} else if (type === 'array') {{
                valueInput.value = '[]';
                valueInput.placeholder = '[1, 2, 3] or ["a", "b"]';
            }} else if (type === 'object') {{
                valueInput.value = '{{}}';
                valueInput.placeholder = '{{"key": "value", "num": 42}}';
            }} else {{
                valueInput.value = '';
                valueInput.placeholder = 'Enter text';
            }}
        }}

        async function saveParam() {{
            const key = document.getElementById('param-key').value.trim();
            const type = document.getElementById('param-type').value;
            const valueStr = document.getElementById('param-value').value.trim();

            if (!key) {{
                alert('Parameter key is required');
                return;
            }}

            let value;
            try {{
                if (type === 'number') {{
                    value = parseFloat(valueStr);
                    if (isNaN(value)) throw new Error('Invalid number');
                }} else if (type === 'boolean') {{
                    value = valueStr.toLowerCase() === 'true';
                }} else if (type === 'array' || type === 'object') {{
                    // Parse JSON for arrays and objects
                    value = JSON.parse(valueStr);
                    // Validate type matches
                    if (type === 'array' && !Array.isArray(value)) {{
                        throw new Error('Value must be a valid JSON array');
                    }}
                    if (type === 'object' && (Array.isArray(value) || typeof value !== 'object')) {{
                        throw new Error('Value must be a valid JSON object');
                    }}
                }} else {{
                    value = valueStr;
                }}
            }} catch (e) {{
                alert('Invalid value for type ' + type + ': ' + e.message);
                return;
            }}

            try {{
                const response = await fetch(`/api/params/${{encodeURIComponent(key)}}`, {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify({{ value }})
                }});

                const data = await response.json();

                if (data.success) {{
                    closeParamDialog();
                    refreshParams();
                }} else {{
                    alert('Error: ' + data.error);
                }}
            }} catch (error) {{
                alert('Failed to save parameter: ' + error.message);
            }}
        }}

        async function deleteParam(key) {{
            if (!confirm(`Are you sure you want to delete parameter "${{key}}"?`)) {{
                return;
            }}

            try {{
                const response = await fetch(`/api/params/${{encodeURIComponent(key)}}`, {{
                    method: 'DELETE'
                }});

                const data = await response.json();

                if (data.success) {{
                    refreshParams();
                }} else {{
                    alert('Error: ' + data.error);
                }}
            }} catch (error) {{
                alert('Failed to delete parameter: ' + error.message);
            }}
        }}

        async function exportParams() {{
            try {{
                const response = await fetch('/api/params/export', {{ method: 'POST' }});
                const data = await response.json();

                if (data.success) {{
                    const blob = new Blob([data.data], {{ type: 'text/yaml' }});
                    const url = URL.createObjectURL(blob);
                    const a = document.createElement('a');
                    a.href = url;
                    a.download = 'horus_params_' + new Date().toISOString().split('T')[0] + '.yaml';
                    a.click();
                    URL.revokeObjectURL(url);
                }} else {{
                    alert('Export failed: ' + data.error);
                }}
            }} catch (error) {{
                alert('Failed to export parameters: ' + error.message);
            }}
        }}

        function showImportDialog() {{
            document.getElementById('import-format').value = 'yaml';
            document.getElementById('import-data').value = '';
            document.getElementById('import-dialog').style.display = 'flex';
        }}

        function closeImportDialog() {{
            document.getElementById('import-dialog').style.display = 'none';
        }}

        async function importParams() {{
            const format = document.getElementById('import-format').value;
            const data = document.getElementById('import-data').value.trim();

            if (!data) {{
                alert('Please paste data to import');
                return;
            }}

            try {{
                const response = await fetch('/api/params/import', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify({{ format, data }})
                }});

                const result = await response.json();

                if (result.success) {{
                    alert(result.message);
                    closeImportDialog();
                    refreshParams();
                }} else {{
                    alert('Import failed: ' + result.error);
                }}
            }} catch (error) {{
                alert('Failed to import parameters: ' + error.message);
            }}
        }}

        // Search parameters
        document.getElementById('param-search')?.addEventListener('input', (e) => {{
            const query = e.target.value.toLowerCase();

            if (!query) {{
                renderParams(allParams);
                return;
            }}

            const filtered = allParams.filter(param =>
                param.key.toLowerCase().includes(query) ||
                String(param.value).toLowerCase().includes(query)
            );

            renderParams(filtered);
        }});

        // Load parameters when params tab is shown
        const paramsTabObserver = new MutationObserver((mutations) => {{
            mutations.forEach((mutation) => {{
                if (mutation.target.classList.contains('active') &&
                    mutation.target.id === 'tab-params') {{
                    refreshParams();
                }}
            }});
        }});

        const paramsTab = document.getElementById('tab-params');
        if (paramsTab) {{
            paramsTabObserver.observe(paramsTab, {{
                attributes: true,
                attributeFilter: ['class']
            }});
        }}
    </script>

    <!-- Install Dialog -->
    <div class="install-dialog" id="install-dialog">
        <div class="install-dialog-content">
            <div class="install-dialog-header">
                <h3>Install Package: <span id="install-pkg-name"></span></h3>
                <button onclick="closeInstallDialog()" style="background: transparent; border: none; color: var(--text-secondary); font-size: 1.5rem; cursor: pointer; padding: 0; width: 30px; height: 30px;">&times;</button>
            </div>
            <div class="install-dialog-body">
                <p style="color: var(--text-secondary); margin-bottom: 1rem;">Where would you like to install this package?</p>

                <div class="install-option" onclick="selectInstallLocation('global')" id="install-option-global">
                    <input type="radio" name="install-location" id="radio-global" value="global">
                    <label for="radio-global" style="cursor: pointer; color: var(--text-primary); font-weight: 600;">
                        Global Installation
                    </label>
                    <div style="color: var(--text-secondary); font-size: 0.85em; margin-top: 0.5rem; margin-left: 24px;">
                        Available to all HORUS projects
                    </div>
                </div>

                <div class="install-option" onclick="selectInstallLocation('local')" id="install-option-local">
                    <input type="radio" name="install-location" id="radio-local" value="local">
                    <label for="radio-local" style="cursor: pointer; color: var(--text-primary); font-weight: 600;">
                        Local Installation
                    </label>
                    <div style="color: var(--text-secondary); font-size: 0.85em; margin-top: 0.5rem; margin-left: 24px;">
                        Install into a specific package
                    </div>
                    <select id="local-package-select" class="local-packages-select" style="display: none;">
                        <option value="">Select a package...</option>
                    </select>
                </div>
            </div>
            <div class="install-dialog-footer">
                <button onclick="closeInstallDialog()" style="padding: 10px 20px; background: var(--surface); color: var(--text-secondary); border: 1px solid var(--border); border-radius: 6px; cursor: pointer; font-weight: 600;">
                    Cancel
                </button>
                <button onclick="confirmInstall()" style="padding: 10px 20px; background: var(--accent); color: var(--primary); border: none; border-radius: 6px; cursor: pointer; font-weight: 600;">
                    Install
                </button>
            </div>
        </div>
    </div>

    <!-- Log Panel -->
    <div id="log-panel" class="log-panel">
        <div class="log-panel-header">
            <div id="log-panel-title" class="log-panel-title">Logs</div>
            <button class="log-panel-close" onclick="closeLogPanel()">[X] Close</button>
        </div>
        <div id="log-panel-content" class="log-panel-content">
            <!-- Logs will be loaded here -->
        </div>
    </div>
</body>
</html>"#,
        port = port
    )
}
