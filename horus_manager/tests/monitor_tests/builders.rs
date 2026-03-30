//! Test builders for `AppState` and the Axum router.
//!
//! These functions construct a realistic `AppState` and router that mirror the
//! production configuration in `monitor::run()` but are suitable for unit and
//! integration tests:
//!
//! - No TCP listener is started.
//! - Authentication is disabled by default (configurable).
//! - No security-headers middleware (keeps test assertions simple).

use axum::{
    routing::{get, post},
    Router,
};
use horus_manager::{
    monitor::{
        blackbox_anomalies_handler, blackbox_clear_handler, blackbox_list_handler,
        debug_add_breakpoint_handler, debug_add_watch_handler, debug_continue_handler,
        debug_pause_handler, debug_remove_breakpoint_handler, debug_remove_watch_handler,
        debug_reset_handler, debug_seek_handler, debug_session_create_handler,
        debug_session_delete_handler, debug_session_get_handler, debug_sessions_list_handler,
        debug_snapshot_handler, debug_step_backward_handler, debug_step_forward_handler,
        debug_watches_values_handler, graph_handler, index_handler, login_handler, logout_handler,
        logs_all_handler, logs_errors_handler, logs_node_handler, logs_topic_handler,
        nodes_handler,
        packages_environments_handler, packages_install_handler, packages_publish_handler,
        packages_registry_handler, packages_uninstall_handler, params_delete_handler,
        params_export_handler, params_get_handler, params_import_handler, params_list_handler,
        params_set_handler, recordings_delete_handler, recordings_info_handler,
        recordings_list_handler, status_handler, topics_handler, websocket_handler, AppState,
    },
    security::{auth::hash_password, AuthService},
};
use std::sync::Arc;

/// Default test password used by [`test_app_state_with_auth`].
pub const TEST_PASSWORD: &str = "test_monitor_password_123";

// ─── AppState builders ──────────────────────────────────────────────────────

/// Create an `AppState` with authentication **disabled**.
///
/// This is the most common configuration for handler unit tests where you want
/// to call endpoints directly without needing to log in first.
pub fn test_app_state() -> Arc<AppState> {
    // AuthService still needs a valid (but unused) password hash so that the
    // struct can be constructed.  We use an empty string — login will simply
    // fail if anyone tries, but that is fine because auth is disabled.
    let auth_service = Arc::new(
        AuthService::new(String::new()).expect("AuthService::new with empty hash must not fail"),
    );
    let params = Arc::new(horus_core::RuntimeParams::default());

    Arc::new(AppState {
        port: 0,
        params,
        auth_service,
        current_workspace: None,
        auth_disabled: true,
        trust_proxy: false,
    })
}

/// Create an `AppState` with authentication **enabled** and [`TEST_PASSWORD`]
/// as the configured password.
///
/// Use this when testing login/logout flows, session validation, or
/// middleware ordering.
pub fn test_app_state_with_auth() -> Arc<AppState> {
    let password_hash = hash_password(TEST_PASSWORD).expect("hash_password must not fail in tests");
    let auth_service =
        Arc::new(AuthService::new(password_hash).expect("AuthService::new must not fail"));
    let params = Arc::new(horus_core::RuntimeParams::default());

    Arc::new(AppState {
        port: 0,
        params,
        auth_service,
        current_workspace: None,
        auth_disabled: false,
        trust_proxy: false,
    })
}

/// Create an `AppState` with a specific workspace path for testing
/// workspace-related handler behaviour.
pub fn test_app_state_with_workspace(workspace: std::path::PathBuf) -> Arc<AppState> {
    let auth_service = Arc::new(
        AuthService::new(String::new()).expect("AuthService::new with empty hash must not fail"),
    );
    let params = Arc::new(horus_core::RuntimeParams::default());

    Arc::new(AppState {
        port: 0,
        params,
        auth_service,
        current_workspace: Some(workspace),
        auth_disabled: true,
        trust_proxy: false,
    })
}

// ─── Router builders ────────────────────────────────────────────────────────

/// Build a test router with **all** monitor routes and **no** auth middleware.
///
/// This mirrors the route layout from `monitor::run()` so that integration
/// tests can call any endpoint via `tower::ServiceExt::oneshot`.
///
/// Authentication is disabled at the `AppState` level (auth_disabled = true),
/// and no session middleware is applied.
pub fn test_router() -> Router {
    test_router_with_state(test_app_state())
}

/// Build a test router with a custom `AppState`.
///
/// Useful when you need auth enabled or a specific workspace path.
pub fn test_router_with_state(state: Arc<AppState>) -> Router {
    let api_routes = Router::new()
        .route("/api/status", get(status_handler))
        .route("/api/nodes", get(nodes_handler))
        .route("/api/topics", get(topics_handler))
        .route("/api/graph", get(graph_handler))
        .route("/api/logs/all", get(logs_all_handler))
        .route("/api/logs/errors", get(logs_errors_handler))
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
        // Recording endpoints
        .route("/api/recordings", get(recordings_list_handler))
        .route("/api/recordings/:session", get(recordings_info_handler))
        .route(
            "/api/recordings/:session",
            axum::routing::delete(recordings_delete_handler),
        )
        // BlackBox endpoints
        .route("/api/blackbox", get(blackbox_list_handler))
        .route("/api/blackbox/anomalies", get(blackbox_anomalies_handler))
        .route(
            "/api/blackbox",
            axum::routing::delete(blackbox_clear_handler),
        )
        // Debug endpoints
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

    // WebSocket route (separate so it can optionally get its own middleware)
    let ws_route = Router::new().route("/api/ws", get(websocket_handler));

    // Public routes
    let public_routes = Router::new()
        .route("/", get(index_handler))
        .route("/api/login", post(login_handler));

    public_routes
        .merge(api_routes)
        .merge(ws_route)
        .with_state(state)
}

// ─── Auth helper ────────────────────────────────────────────────────────────

/// Perform a login against the given `app` and return `(session_token, csrf_token)`.
///
/// Panics if the login fails.  Requires the router to have been built with
/// [`test_app_state_with_auth`] (or equivalent) and the password to be
/// [`TEST_PASSWORD`].
pub async fn login(app: &Router) -> (String, String) {
    use axum::body::Body;
    use axum::http::Request;
    use tower::ServiceExt;

    let resp = app
        .clone()
        .oneshot(
            Request::builder()
                .method("POST")
                .uri("/api/login")
                .header("Content-Type", "application/json")
                .body(Body::from(format!(r#"{{"password":"{}"}}"#, TEST_PASSWORD)))
                .unwrap(),
        )
        .await
        .expect("login request must not fail");

    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "login with TEST_PASSWORD must succeed"
    );

    let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
        .await
        .expect("body read must not fail");
    let json: serde_json::Value =
        serde_json::from_slice(&body).expect("login response must be valid JSON");

    let session_token = json["session_token"]
        .as_str()
        .expect("login response must contain session_token")
        .to_string();
    let csrf_token = json["csrf_token"]
        .as_str()
        .expect("login response must contain csrf_token")
        .to_string();

    (session_token, csrf_token)
}
