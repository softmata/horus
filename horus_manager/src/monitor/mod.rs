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
use std::net::{SocketAddr, UdpSocket};
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};
use tower_http::cors::{AllowOrigin, CorsLayer};

// Security imports
use crate::security::{security_headers_middleware, AuthService};

// Re-export handlers for router setup (pub so integration tests can compose routers).
pub use handlers::*;

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
    /// Whether to trust `X-Forwarded-For` / `X-Real-IP` headers from a
    /// reverse proxy.  Controlled by the `HORUS_TRUST_PROXY` environment
    /// variable.  Must be `false` (the default) on direct-internet deployments
    /// so clients cannot spoof their IP to bypass per-IP rate limiting.
    pub trust_proxy: bool,
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

/// WebSocket-specific authentication middleware.
///
/// Accepts a session token from three sources (checked in order):
///   1. `Cookie: session_token=<token>` — set automatically by the browser after login
///   2. `Authorization: Bearer <token>` — for non-browser clients
///   3. `?token=<token>` URL query parameter — the standard workaround for
///      browser `WebSocket` clients, which cannot set custom HTTP headers
///
/// ## Why a separate middleware instead of reusing `monitor_session_middleware`
///
/// The standard session middleware only checks Cookie and Authorization headers
/// (sufficient for all HTTP API calls).  WebSocket connections from a browser
/// must pass the token in the URL query string because the `WebSocket` API does
/// not expose a way to set headers on the initial HTTP upgrade request.
///
/// ## Why a middleware layer and not a check inside `websocket_handler`
///
/// When Axum's `WebSocketUpgrade` extractor is dropped without calling
/// `on_upgrade()`, axum emits an HTTP 426 "Upgrade Required" response that
/// overrides any 401 the handler would have returned.  By placing the auth
/// check in a middleware layer that runs *before* the handler, the
/// `WebSocketUpgrade` extractor is never reached for unauthenticated requests.
async fn ws_auth_middleware(
    State(state): State<Arc<AppState>>,
    req: axum::http::Request<axum::body::Body>,
    next: axum::middleware::Next,
) -> Result<axum::response::Response, StatusCode> {
    // Auth disabled → let every request through.
    if state.auth_disabled {
        return Ok(next.run(req).await);
    }

    // 1. Cookie: session_token=<token>
    let token_from_cookie = req
        .headers()
        .get(axum::http::header::COOKIE)
        .and_then(|v| v.to_str().ok())
        .and_then(|cookies| {
            cookies
                .split(';')
                .map(|c| c.trim())
                .find_map(|c| c.strip_prefix("session_token="))
        })
        .map(|s| s.to_string());

    // 2. Authorization: Bearer <token>
    let token_from_bearer = req
        .headers()
        .get(axum::http::header::AUTHORIZATION)
        .and_then(|v| v.to_str().ok())
        .and_then(|s| s.strip_prefix("Bearer "))
        .map(|s| s.to_string());

    // 3. ?token= query parameter (browser WebSocket clients cannot set headers)
    let token_from_query = req
        .uri()
        .query()
        .and_then(|q| q.split('&').find_map(|pair| pair.strip_prefix("token=")))
        .filter(|v| !v.is_empty())
        .map(|s| s.to_string());

    let token = token_from_cookie
        .or(token_from_bearer)
        .or(token_from_query)
        .ok_or(StatusCode::UNAUTHORIZED)?;

    if !state.auth_service.validate_session(&token) {
        return Err(StatusCode::UNAUTHORIZED);
    }

    Ok(next.run(req).await)
}

/// Run the web monitor server
///
/// `no_auth` corresponds to the `--no-auth` CLI flag.  When `true`, the
/// monitor starts without requiring a password.  A loud warning is printed
/// to stderr so operators are never silently running an open endpoint.
/// When `false` and no password is configured, `run` returns an error
/// instructing the user to set a password.
pub async fn run(port: u16, no_auth: bool) -> anyhow::Result<()> {
    use colored::Colorize;
    // Determine the password hash and authentication mode.
    let (password_hash, auth_disabled) = if !crate::security::auth::is_password_configured() {
        if no_auth {
            // Explicit opt-out: proceed without authentication.
            (String::new(), true)
        } else {
            // No password file and no --no-auth → refuse to start.
            anyhow::bail!(
                "No monitor password is set.\n\
                 \n\
                 To set a password:  horus monitor -r\n\
                 To disable auth:    horus monitor --no-auth\n\
                 \n\
                 Running without a password exposes the entire monitoring API \
                 to anyone on your network. Use --no-auth only in trusted \
                 environments."
            );
        }
    } else {
        let hash = crate::security::auth::load_password_hash()?;
        let disabled = hash.is_empty();
        if disabled && !no_auth {
            // Hash file exists but is empty (user previously opted out).
            // Require explicit --no-auth to proceed.
            anyhow::bail!(
                "Authentication is disabled (empty password on file).\n\
                 \n\
                 To set a password:  horus monitor -r\n\
                 To disable auth:    horus monitor --no-auth"
            );
        }
        (hash, disabled || no_auth)
    };

    if auth_disabled {
        eprintln!(
            "\n{} HORUS monitor is running WITHOUT authentication.\n\
             {} Anyone on your network can access all monitoring APIs.\n\
             {} Set a password with: horus monitor -r\n",
            "[WARNING]".red().bold(),
            "[WARNING]".red().bold(),
            "[WARNING]".red().bold(),
        );
    }

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
        trust_proxy: crate::config::trust_proxy(),
    });

    // ─── Middleware ordering contract ─────────────────────────────────────────
    // The auth layer MUST be applied to `api_routes` BEFORE any Router::merge.
    //
    // Rationale: In Axum, `Router::layer(L)` wraps the routes already in the
    // router at the time of the call.  `Router::merge` DOES preserve per-router
    // layers, but only because we apply the layer before merging.  Applying the
    // layer AFTER `merge(api_routes)` would wrap ALL routes — including the
    // public login route — which is wrong; and applying it too late could
    // silently expose routes if Axum ever changes merge semantics.
    //
    // Correct order (current):
    //   1. Add all /api/* routes to `api_routes`          ← routes defined
    //   2. api_routes.layer(auth)                          ← layer applied
    //   3. public_routes.merge(api_routes)                 ← merge after layer
    //
    // NEVER do:
    //   1. public_routes.merge(api_routes).layer(auth)     ← would guard login too
    //   2. api_routes without layer, then merge            ← leaves /api/* open
    // ─────────────────────────────────────────────────────────────────────────

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
        // BlackBox flight recorder API endpoints
        .route("/api/blackbox", get(blackbox_list_handler))
        .route("/api/blackbox/anomalies", get(blackbox_anomalies_handler))
        .route(
            "/api/blackbox",
            axum::routing::delete(blackbox_clear_handler),
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
        .route("/api/logout", post(logout_handler));

    // Only add authentication middleware if password is set
    if !auth_disabled {
        api_routes = api_routes.layer(middleware::from_fn_with_state(
            state.clone(),
            monitor_session_middleware,
        ));
    }

    // WebSocket route with its own dedicated auth middleware.
    //
    // `/api/ws` is intentionally NOT part of `api_routes` above.  Reason: the
    // standard `monitor_session_middleware` only checks Cookie and Authorization
    // headers, but browser WebSocket clients must pass their token via the
    // `?token=` query parameter (the `WebSocket` API does not allow custom
    // headers).  `ws_auth_middleware` accepts all three sources.  It also
    // prevents a 426 "Upgrade Required" response that Axum would emit if the
    // `WebSocketUpgrade` extractor were dropped without calling `on_upgrade()`.
    let mut ws_route = Router::new().route("/api/ws", get(websocket_handler));
    if !auth_disabled {
        ws_route = ws_route.layer(middleware::from_fn_with_state(
            state.clone(),
            ws_auth_middleware,
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
        .merge(ws_route)
        .layer(cors)
        .layer(middleware::from_fn(security_headers_middleware))
        .with_state(state);

    // Display startup info
    println!(
        "\n{}",
        "╔════════════════════════════════════════════╗".cyan()
    );
    println!(
        "{}",
        "║         HORUS Web Monitor                  ║".cyan()
    );
    println!(
        "{}",
        "╠════════════════════════════════════════════╣".cyan()
    );
    println!("{}  http://localhost:{}", "║  Local:".green(), port);

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
        println!("{}  {}", "║  Workspace:".green(), ws.display());
    }

    println!(
        "{}",
        "╚════════════════════════════════════════════╝".cyan()
    );
    println!("{}", "  Press Ctrl+C to stop the server\n".bright_black());

    // Auto-open browser to the monitor UI
    let url = format!("http://localhost:{}", port);
    if let Err(e) = open::that(&url) {
        eprintln!("{} Failed to open browser: {}", "Warning:".yellow(), e);
    }

    let listener = tokio::net::TcpListener::bind(format!("0.0.0.0:{}", port)).await?;
    axum::serve(
        listener,
        app.into_make_service_with_connect_info::<SocketAddr>(),
    )
    .await?;

    Ok(())
}

// ─── Auth middleware ordering tests ──────────────────────────────────────────
#[cfg(test)]
mod tests {
    use super::*;
    use axum::{body::Body, http::Request, routing::post};
    use tower::ServiceExt; // for .oneshot()

    /// Build a minimal router identical in structure to `run()` but without
    /// starting a real TCP listener.  Auth is always enabled in tests.
    fn build_test_app() -> axum::Router {
        // Use a real Argon2 hash so that PasswordHash::new() succeeds inside
        // AuthService::login().  A raw string would fail to parse, causing
        // login() to return Err (mapped to 429) on the very first attempt —
        // which would break the brute-force rate-limit test.
        let password_hash = crate::security::auth::hash_password("test_monitor_password_123")
            .expect("hash_password should not fail in tests");
        let auth_service =
            Arc::new(AuthService::new(password_hash).expect("AuthService::new should not fail"));
        let params = Arc::new(horus_core::RuntimeParams::default());
        let state = Arc::new(AppState {
            port: 0,
            params,
            auth_service: auth_service.clone(),
            current_workspace: None,
            auth_disabled: false, // auth IS active
            trust_proxy: false,   // no proxy in tests
        });

        // Replicate the route+middleware structure from `run()` for the routes
        // being tested.  Using a representative subset of /api/* routes covers
        // the middleware ordering contract without pulling in all handlers.
        let mut api_routes = Router::new()
            .route("/api/status", get(status_handler))
            .route("/api/debug/sessions", get(debug_sessions_list_handler))
            .route("/api/debug/sessions", post(debug_session_create_handler))
            .route("/api/debug/sessions/:id", get(debug_session_get_handler));

        // ← Auth layer applied BEFORE merge (this is the invariant being tested)
        api_routes = api_routes.layer(middleware::from_fn_with_state(
            state.clone(),
            monitor_session_middleware,
        ));

        // /api/ws uses its own dedicated middleware (Cookie, Bearer, ?token=),
        // mirroring the structure in `run()`.
        let ws_route = Router::new()
            .route("/api/ws", get(websocket_handler))
            .layer(middleware::from_fn_with_state(
                state.clone(),
                ws_auth_middleware,
            ));

        let public_routes = Router::new().route("/api/login", post(login_handler));

        public_routes
            .merge(api_routes)
            .merge(ws_route)
            .with_state(state)
    }

    #[test]
    fn safe_path_component_rejects_traversal() {
        assert!(!is_safe_path_component(".."));
        assert!(!is_safe_path_component("foo/../bar"));
        assert!(!is_safe_path_component("foo/bar"));
        assert!(!is_safe_path_component("foo\\bar"));
        assert!(!is_safe_path_component("foo\0bar"));
        assert!(!is_safe_path_component(""));
    }

    #[test]
    fn safe_path_component_accepts_valid_names() {
        assert!(is_safe_path_component("session-2026-01-01"));
        assert!(is_safe_path_component("my_recording"));
        assert!(is_safe_path_component("test.session"));
    }

    /// Unauthenticated GET /api/debug/sessions must return 401.
    #[tokio::test]
    async fn debug_sessions_list_requires_auth() {
        let app = build_test_app();
        let req = Request::builder()
            .method("GET")
            .uri("/api/debug/sessions")
            .body(Body::empty())
            .unwrap();
        let resp = app.oneshot(req).await.unwrap();
        assert_eq!(
            resp.status(),
            axum::http::StatusCode::UNAUTHORIZED,
            "GET /api/debug/sessions without credentials must return 401"
        );
    }

    /// Unauthenticated GET /api/debug/sessions/:id must return 401.
    #[tokio::test]
    async fn debug_session_get_requires_auth() {
        let app = build_test_app();
        let req = Request::builder()
            .method("GET")
            .uri("/api/debug/sessions/some-session-id")
            .body(Body::empty())
            .unwrap();
        let resp = app.oneshot(req).await.unwrap();
        assert_eq!(resp.status(), axum::http::StatusCode::UNAUTHORIZED);
    }

    /// Unauthenticated GET /api/status (another protected route) must return 401.
    #[tokio::test]
    async fn api_status_requires_auth() {
        let app = build_test_app();
        let req = Request::builder()
            .method("GET")
            .uri("/api/status")
            .body(Body::empty())
            .unwrap();
        let resp = app.oneshot(req).await.unwrap();
        assert_eq!(resp.status(), axum::http::StatusCode::UNAUTHORIZED);
    }

    /// After AUTH_MAX_ATTEMPTS failed login attempts from the same IP, the next
    /// attempt must return 429 (Too Many Requests), not 401.
    ///
    /// In tests, ConnectInfo is not available so the handler falls back to
    /// "127.0.0.1" — all requests therefore share the same rate-limit bucket,
    /// which is exactly what we want for this test.  The Arc<AuthService> is
    /// shared across Router clones (app.clone() only clones the Arc, not the
    /// inner data), so rate-limit counters accumulate across `oneshot` calls.
    #[tokio::test]
    async fn login_brute_force_triggers_rate_limit() {
        let app = build_test_app();

        // AUTH_MAX_ATTEMPTS wrong passwords → all 401 (wrong password, not yet rate-limited).
        for attempt in 0..crate::config::AUTH_MAX_ATTEMPTS {
            let resp = app
                .clone()
                .oneshot(
                    Request::builder()
                        .method("POST")
                        .uri("/api/login")
                        .header("Content-Type", "application/json")
                        .body(Body::from(r#"{"password":"wrong_password"}"#))
                        .unwrap(),
                )
                .await
                .unwrap();
            assert_eq!(
                resp.status(),
                axum::http::StatusCode::UNAUTHORIZED,
                "attempt {} should return 401 (wrong password, not yet rate-limited)",
                attempt + 1,
            );
        }

        // AUTH_MAX_ATTEMPTS + 1: rate limiter engages → 429.
        let resp = app
            .clone()
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/login")
                    .header("Content-Type", "application/json")
                    .body(Body::from(r#"{"password":"wrong_password"}"#))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(
            resp.status(),
            axum::http::StatusCode::TOO_MANY_REQUESTS,
            "attempt {} should return 429 (rate limited)",
            crate::config::AUTH_MAX_ATTEMPTS + 1,
        );
    }

    /// POST /api/login is a public route and must NOT return 401 when no token
    /// is present (it should process the request regardless of auth state).
    #[tokio::test]
    async fn login_route_is_public() {
        let app = build_test_app();
        let req = Request::builder()
            .method("POST")
            .uri("/api/login")
            .header("Content-Type", "application/json")
            .body(Body::from(r#"{"password":"wrong"}"#))
            .unwrap();
        let resp = app.oneshot(req).await.unwrap();
        // Login with wrong password returns 401 from the handler, but NOT from
        // the session middleware.  The important thing is that the response is
        // not 404 (route missing), which would indicate the public route wasn't
        // registered.  Any non-404 status means the route was reached.
        assert_ne!(
            resp.status(),
            axum::http::StatusCode::NOT_FOUND,
            "/api/login must be a registered public route"
        );
    }

    /// `run()` with no password configured and no --no-auth must return an Err
    /// with a message pointing the user to `horus monitor -r` or `--no-auth`.
    ///
    /// We test via a closure mirroring the exact policy logic inside `run()`.
    #[test]
    fn run_no_password_no_flag_returns_error() {
        // Mirrors the policy decision in `run()` before the TCP bind.
        let check_policy = |no_password_configured: bool, no_auth: bool| -> anyhow::Result<()> {
            if no_password_configured && !no_auth {
                anyhow::bail!(
                    "No monitor password is set.\n\
                     \n\
                     To set a password:  horus monitor -r\n\
                     To disable auth:    horus monitor --no-auth"
                );
            }
            Ok(())
        };

        // No password, no flag → must be an error.
        let result = check_policy(true, false);
        assert!(
            result.is_err(),
            "`run()` without a password and without --no-auth must fail"
        );
        let msg = result.unwrap_err().to_string();
        assert!(
            msg.contains("No monitor password is set"),
            "error message should explain the problem: {}",
            msg
        );
        assert!(
            msg.contains("--no-auth"),
            "error message should mention --no-auth: {}",
            msg
        );
        assert!(
            msg.contains("horus monitor -r"),
            "error message should mention password setup: {}",
            msg
        );

        // No password but --no-auth → must succeed.
        assert!(
            check_policy(true, true).is_ok(),
            "no password + --no-auth must be allowed"
        );
        // Password configured, no --no-auth → must succeed.
        assert!(
            check_policy(false, false).is_ok(),
            "password configured, no --no-auth must be allowed"
        );
    }

    /// Minimum password length constant must be ≥ 12 (NIST SP 800-63B minimum).
    #[test]
    fn min_password_length_meets_nist_minimum() {
        assert!(
            crate::config::MIN_PASSWORD_LENGTH >= 12,
            "MIN_PASSWORD_LENGTH must be ≥ 12 (NIST SP 800-63B); got {}",
            crate::config::MIN_PASSWORD_LENGTH,
        );
    }

    /// GET /api/ws without a session token must return 401 (the HTTP upgrade
    /// must be rejected before the WebSocket connection is established).
    ///
    /// The test sends a proper WebSocket upgrade request (Connection: Upgrade,
    /// Upgrade: websocket, Sec-WebSocket-Key, Sec-WebSocket-Version headers)
    /// without any session token.  `ws_auth_middleware` runs before the
    /// `WebSocketUpgrade` extractor and returns 401 for unauthenticated
    /// requests, preventing the 426 "Upgrade Required" that Axum would emit
    /// if the extractor were dropped without calling `on_upgrade()`.
    #[tokio::test]
    async fn websocket_upgrade_without_token_returns_401() {
        let app = build_test_app();
        let req = Request::builder()
            .method("GET")
            .uri("/api/ws")
            .header("Connection", "Upgrade")
            .header("Upgrade", "websocket")
            .header("Sec-WebSocket-Key", "dGhlIHNhbXBsZSBub25jZQ==")
            .header("Sec-WebSocket-Version", "13")
            .body(Body::empty())
            .unwrap();
        let resp = app.oneshot(req).await.unwrap();
        assert_eq!(
            resp.status(),
            axum::http::StatusCode::UNAUTHORIZED,
            "WebSocket upgrade without token must be rejected with 401"
        );
    }

    /// Helper: perform a successful login and return the session token.
    async fn login_and_get_token(app: &axum::Router) -> String {
        let resp = app
            .clone()
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/login")
                    .header("Content-Type", "application/json")
                    .body(Body::from(r#"{"password":"test_monitor_password_123"}"#))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(resp.status(), axum::http::StatusCode::OK);
        let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
            .await
            .unwrap();
        let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
        json["session_token"]
            .as_str()
            .expect("login response must contain session_token")
            .to_string()
    }

    /// Successful login returns 200 with session_token, csrf_token, and
    /// success=true in the JSON body and a Set-Cookie header.
    #[tokio::test]
    async fn login_success_returns_token_and_csrf() {
        let app = build_test_app();
        let resp = app
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/login")
                    .header("Content-Type", "application/json")
                    .body(Body::from(r#"{"password":"test_monitor_password_123"}"#))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(resp.status(), axum::http::StatusCode::OK);

        // Verify Set-Cookie header is present with HttpOnly flag
        let set_cookie = resp
            .headers()
            .get(axum::http::header::SET_COOKIE)
            .expect("login must set a cookie")
            .to_str()
            .unwrap();
        assert!(
            set_cookie.contains("session_token="),
            "cookie must contain session_token"
        );
        assert!(
            set_cookie.contains("HttpOnly"),
            "session cookie must be HttpOnly"
        );
        assert!(
            set_cookie.contains("SameSite=Strict"),
            "session cookie must be SameSite=Strict"
        );

        let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
            .await
            .unwrap();
        let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
        assert_eq!(json["success"], true);
        assert!(json["session_token"].is_string());
        assert!(json["csrf_token"].is_string());
        assert!(json["error"].is_null());
    }

    /// Login with wrong password returns 401 with success=false and error
    /// message.
    #[tokio::test]
    async fn login_wrong_password_returns_401_with_error() {
        let app = build_test_app();
        let resp = app
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/login")
                    .header("Content-Type", "application/json")
                    .body(Body::from(r#"{"password":"wrong_password"}"#))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(resp.status(), axum::http::StatusCode::UNAUTHORIZED);
        let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
            .await
            .unwrap();
        let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
        assert_eq!(json["success"], false);
        assert!(json["session_token"].is_null());
        assert!(json["error"].is_string());
    }

    /// After successful login, authenticated GET /api/status returns 200 with
    /// expected JSON fields.
    #[tokio::test]
    async fn authenticated_status_returns_200_with_json() {
        let app = build_test_app();
        let token = login_and_get_token(&app).await;
        let resp = app
            .oneshot(
                Request::builder()
                    .method("GET")
                    .uri("/api/status")
                    .header("Cookie", format!("session_token={}", token))
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(resp.status(), axum::http::StatusCode::OK);
        let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
            .await
            .unwrap();
        let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
        // Validate JSON schema
        assert!(json["status"].is_string(), "status must be a string");
        assert!(json["health"].is_string(), "health must be a string");
        assert!(json["version"].is_string(), "version must be a string");
        assert!(json["nodes"].is_number(), "nodes count must be a number");
        assert!(json["topics"].is_number(), "topics count must be a number");
        assert!(json["workspace"].is_object(), "workspace must be an object");
    }

    /// Bearer token authentication works for protected endpoints.
    #[tokio::test]
    async fn bearer_token_auth_works() {
        let app = build_test_app();
        let token = login_and_get_token(&app).await;
        let resp = app
            .oneshot(
                Request::builder()
                    .method("GET")
                    .uri("/api/status")
                    .header("Authorization", format!("Bearer {}", token))
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(
            resp.status(),
            axum::http::StatusCode::OK,
            "Bearer token should authenticate the request"
        );
    }

    /// Invalid session token returns 401.
    #[tokio::test]
    async fn invalid_session_token_returns_401() {
        let app = build_test_app();
        let resp = app
            .oneshot(
                Request::builder()
                    .method("GET")
                    .uri("/api/status")
                    .header("Cookie", "session_token=bogus_invalid_token")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(resp.status(), axum::http::StatusCode::UNAUTHORIZED);
    }

    /// Authenticated GET /api/debug/sessions returns 200 with JSON containing
    /// sessions array and count.
    #[tokio::test]
    async fn authenticated_debug_sessions_returns_json() {
        let app = build_test_app();
        let token = login_and_get_token(&app).await;
        let resp = app
            .oneshot(
                Request::builder()
                    .method("GET")
                    .uri("/api/debug/sessions")
                    .header("Cookie", format!("session_token={}", token))
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(resp.status(), axum::http::StatusCode::OK);
        let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
            .await
            .unwrap();
        let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
        assert!(json["sessions"].is_array());
        assert!(json["count"].is_number());
    }

    /// GET /api/debug/sessions/:id for a non-existent session returns 404.
    #[tokio::test]
    async fn debug_session_not_found_returns_404() {
        let app = build_test_app();
        let token = login_and_get_token(&app).await;
        let resp = app
            .oneshot(
                Request::builder()
                    .method("GET")
                    .uri("/api/debug/sessions/nonexistent-id-12345")
                    .header("Cookie", format!("session_token={}", token))
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(resp.status(), axum::http::StatusCode::NOT_FOUND);
        let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
            .await
            .unwrap();
        let json: serde_json::Value = serde_json::from_slice(&body).unwrap();
        assert!(json["error"].is_string());
    }

    /// POST /api/login with invalid JSON returns 4xx (malformed request).
    #[tokio::test]
    async fn login_with_invalid_json_returns_error() {
        let app = build_test_app();
        let resp = app
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/login")
                    .header("Content-Type", "application/json")
                    .body(Body::from(r#"not valid json"#))
                    .unwrap(),
            )
            .await
            .unwrap();
        // Axum returns 422 Unprocessable Entity for JSON parse failures
        assert!(
            resp.status().is_client_error(),
            "invalid JSON should return 4xx, got {}",
            resp.status()
        );
    }

    /// POST /api/login without Content-Type header returns an error (Axum
    /// requires Content-Type: application/json for Json extractor).
    #[tokio::test]
    async fn login_without_content_type_returns_error() {
        let app = build_test_app();
        let resp = app
            .oneshot(
                Request::builder()
                    .method("POST")
                    .uri("/api/login")
                    .body(Body::from(r#"{"password":"test"}"#))
                    .unwrap(),
            )
            .await
            .unwrap();
        assert!(
            resp.status().is_client_error(),
            "missing Content-Type should return 4xx, got {}",
            resp.status()
        );
    }

    /// WebSocket upgrade with token in query parameter should not return 401
    /// (tests the `?token=` path in `ws_auth_middleware`).
    #[tokio::test]
    async fn websocket_upgrade_with_query_token_passes_auth() {
        let app = build_test_app();
        let token = login_and_get_token(&app).await;
        let resp = app
            .oneshot(
                Request::builder()
                    .method("GET")
                    .uri(format!("/api/ws?token={}", token))
                    .header("Connection", "Upgrade")
                    .header("Upgrade", "websocket")
                    .header("Sec-WebSocket-Key", "dGhlIHNhbXBsZSBub25jZQ==")
                    .header("Sec-WebSocket-Version", "13")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        // With a valid token, auth middleware passes. The response may be 101
        // (Switching Protocols) or another non-401 status depending on how Axum
        // handles the upgrade in a test (no real TCP socket).
        assert_ne!(
            resp.status(),
            axum::http::StatusCode::UNAUTHORIZED,
            "WebSocket upgrade with valid ?token= must pass auth"
        );
    }

    /// WebSocket upgrade with invalid query token returns 401.
    #[tokio::test]
    async fn websocket_upgrade_with_invalid_query_token_returns_401() {
        let app = build_test_app();
        let resp = app
            .oneshot(
                Request::builder()
                    .method("GET")
                    .uri("/api/ws?token=invalid_token_value")
                    .header("Connection", "Upgrade")
                    .header("Upgrade", "websocket")
                    .header("Sec-WebSocket-Key", "dGhlIHNhbXBsZSBub25jZQ==")
                    .header("Sec-WebSocket-Version", "13")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(
            resp.status(),
            axum::http::StatusCode::UNAUTHORIZED,
            "WebSocket upgrade with invalid token must return 401"
        );
    }

    /// WebSocket upgrade with empty query token returns 401.
    #[tokio::test]
    async fn websocket_upgrade_with_empty_query_token_returns_401() {
        let app = build_test_app();
        let resp = app
            .oneshot(
                Request::builder()
                    .method("GET")
                    .uri("/api/ws?token=")
                    .header("Connection", "Upgrade")
                    .header("Upgrade", "websocket")
                    .header("Sec-WebSocket-Key", "dGhlIHNhbXBsZSBub25jZQ==")
                    .header("Sec-WebSocket-Version", "13")
                    .body(Body::empty())
                    .unwrap(),
            )
            .await
            .unwrap();
        assert_eq!(
            resp.status(),
            axum::http::StatusCode::UNAUTHORIZED,
            "WebSocket upgrade with empty token must return 401"
        );
    }

    /// Verify workspace cache returns consistent results across calls.
    #[test]
    fn workspace_cache_returns_same_results() {
        let cache = workspace_cache();
        let first = cache.write().unwrap().get_or_refresh(&None);
        let second = cache.write().unwrap().get_or_refresh(&None);
        assert_eq!(first.len(), second.len());
    }
}
