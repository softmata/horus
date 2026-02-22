mod handlers;
mod html;

use axum::{
    extract::State,
    http::StatusCode,
    middleware,
    routing::{get, post},
    Router,
};
use qrcode::{render::unicode, QrCode};
use std::net::UdpSocket;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};
use tower_http::cors::{AllowOrigin, CorsLayer};

// Security imports
use crate::security::{security_headers_middleware, AuthService};

// Re-export handlers for router setup
use handlers::*;

/// Validate a path component (session name, filename) to prevent directory traversal.
///
/// Rejects components containing `..`, `/`, `\`, or null bytes.
pub(crate) fn is_safe_path_component(component: &str) -> bool {
    !component.is_empty()
        && !component.contains("..")
        && !component.contains('/')
        && !component.contains('\\')
        && !component.contains('\0')
}

/// Cache for workspace discovery to avoid repeated filesystem scanning
pub(crate) struct WorkspaceCache {
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
        const TTL: Duration = Duration::from_secs(crate::config::WORKSPACE_CACHE_TTL_SECS);

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
pub(crate) fn workspace_cache() -> &'static RwLock<WorkspaceCache> {
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
            println!("\n{}", "  Scan to access on mobile:".cyan().bold());
            println!("{}", qr_string);
        }
        Err(e) => {
            log::warn!("Failed to generate QR code: {}", e);
        }
    }
}

/// Session validation middleware for monitor using AppState
async fn monitor_session_middleware(
    State(state): State<Arc<AppState>>,
    req: axum::http::Request<axum::body::Body>,
    next: axum::middleware::Next,
) -> Result<axum::response::Response, StatusCode> {
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

    // Public routes (no auth required)
    let public_routes = Router::new()
        .route("/", get(index_handler))
        .route("/api/login", post(login_handler));

    let cors = CorsLayer::new()
        .allow_origin(AllowOrigin::any())
        .allow_methods([
            axum::http::Method::GET,
            axum::http::Method::POST,
            axum::http::Method::DELETE,
        ])
        .allow_headers([
            axum::http::header::CONTENT_TYPE,
            axum::http::header::AUTHORIZATION,
        ]);

    let app = public_routes
        .merge(api_routes)
        .layer(cors)
        .layer(middleware::from_fn(security_headers_middleware))
        .with_state(state);

    // Display startup info
    use colored::Colorize;
    println!("\n{}", "╔════════════════════════════════════════════╗".cyan());
    println!(
        "{}",
        "║         HORUS Web Monitor                  ║".cyan()
    );
    println!("{}", "╠════════════════════════════════════════════╣".cyan());
    println!(
        "{}  http://localhost:{}",
        "║  Local:".green(),
        port
    );

    if let Some(ip) = get_local_ip() {
        let network_url = format!("http://{}:{}", ip, port);
        println!("{}  {}", "║  Network:".green(), network_url);

        // Show QR code for mobile access
        display_qr_code(&network_url);
    }

    if auth_disabled {
        println!(
            "{}  {}",
            "║  Auth:".yellow(),
            "DISABLED (no password set)".yellow()
        );
    } else {
        println!("{}  Password protected", "║  Auth:".green());
    }

    // Show workspace info
    if let Some(ref ws) = current_workspace {
        println!(
            "{}  {}",
            "║  Workspace:".green(),
            ws.display()
        );
    }

    println!("{}", "╚════════════════════════════════════════════╝".cyan());
    println!(
        "{}",
        "  Press Ctrl+C to stop the server\n".bright_black()
    );

    let listener = tokio::net::TcpListener::bind(format!("0.0.0.0:{}", port)).await?;
    axum::serve(listener, app).await?;

    Ok(())
}
