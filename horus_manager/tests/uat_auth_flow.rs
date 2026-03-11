//! UAT: Auth flow E2E with real web server.
//!
//! Verifies the full authentication lifecycle:
//! login, session access, CSRF enforcement on mutations, logout, and
//! expired session rejection.
//!
//! Tests use a router with production-like auth middleware.
#![cfg(feature = "monitor")]

mod harness;
mod monitor_tests;

use monitor_tests::builders;

use axum::middleware;
use tower::ServiceExt;

/// Session middleware wrapper that extracts AuthService from AppState.
/// Mirrors `monitor_session_middleware` in production.
async fn test_session_middleware(
    axum::extract::State(state): axum::extract::State<
        std::sync::Arc<horus_manager::monitor::AppState>,
    >,
    req: axum::http::Request<axum::body::Body>,
    next: axum::middleware::Next,
) -> Result<axum::response::Response, axum::http::StatusCode> {
    horus_manager::security::session_middleware(
        axum::extract::State(state.auth_service.clone()),
        req,
        next,
    )
    .await
}

/// Build a test router with auth middleware applied (mirrors production layout).
fn auth_test_router() -> axum::Router {
    use axum::routing::{get, post};
    use horus_manager::monitor::*;

    let state = builders::test_app_state_with_auth();

    // Protected API routes — require valid session + CSRF for mutations
    let api_routes = axum::Router::new()
        .route("/api/status", get(status_handler))
        .route("/api/nodes", get(nodes_handler))
        .route("/api/topics", get(topics_handler))
        .route("/api/graph", get(graph_handler))
        .route("/api/params", get(params_list_handler))
        .route("/api/params/:key", get(params_get_handler))
        .route("/api/params/:key", post(params_set_handler))
        .route(
            "/api/params/:key",
            axum::routing::delete(params_delete_handler),
        )
        .route("/api/logout", post(logout_handler))
        .layer(middleware::from_fn_with_state(
            state.clone(),
            test_session_middleware,
        ));

    // Public routes — no auth required
    let public_routes = axum::Router::new()
        .route("/", get(index_handler))
        .route("/api/login", post(login_handler));

    public_routes.merge(api_routes).with_state(state)
}

/// Helper: perform login and return (session_token, csrf_token).
async fn do_login(app: &axum::Router) -> (String, String) {
    let resp = app
        .clone()
        .oneshot(
            axum::http::Request::builder()
                .method("POST")
                .uri("/api/login")
                .header("Content-Type", "application/json")
                .body(axum::body::Body::from(format!(
                    r#"{{"password":"{}"}}"#,
                    builders::TEST_PASSWORD
                )))
                .unwrap(),
        )
        .await
        .unwrap();

    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "login must succeed"
    );

    let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();

    let session_token = json["session_token"]
        .as_str()
        .expect("must have session_token")
        .to_string();
    let csrf_token = json["csrf_token"]
        .as_str()
        .expect("must have csrf_token")
        .to_string();

    (session_token, csrf_token)
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Unauthenticated access — protected endpoints return 401
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn unauthenticated_get_nodes_returns_401() {
    let app = auth_test_router();
    let resp = app
        .oneshot(
            axum::http::Request::builder()
                .method("GET")
                .uri("/api/nodes")
                .body(axum::body::Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::UNAUTHORIZED,
        "unauthenticated GET /api/nodes must return 401"
    );
}

#[tokio::test]
async fn unauthenticated_get_status_returns_401() {
    let app = auth_test_router();
    let resp = app
        .oneshot(
            axum::http::Request::builder()
                .method("GET")
                .uri("/api/status")
                .body(axum::body::Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(resp.status(), axum::http::StatusCode::UNAUTHORIZED,);
}

#[tokio::test]
async fn unauthenticated_get_topics_returns_401() {
    let app = auth_test_router();
    let resp = app
        .oneshot(
            axum::http::Request::builder()
                .method("GET")
                .uri("/api/topics")
                .body(axum::body::Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(resp.status(), axum::http::StatusCode::UNAUTHORIZED);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Login — wrong password returns 401
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn login_wrong_password_returns_401() {
    let app = auth_test_router();
    let resp = app
        .oneshot(
            axum::http::Request::builder()
                .method("POST")
                .uri("/api/login")
                .header("Content-Type", "application/json")
                .body(axum::body::Body::from(
                    r#"{"password":"wrong_password_xyz"}"#,
                ))
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::UNAUTHORIZED,
        "wrong password must return 401"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Login — correct password returns session + CSRF tokens
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn login_correct_password_returns_tokens() {
    let app = auth_test_router();

    let resp = app
        .oneshot(
            axum::http::Request::builder()
                .method("POST")
                .uri("/api/login")
                .header("Content-Type", "application/json")
                .body(axum::body::Body::from(format!(
                    r#"{{"password":"{}"}}"#,
                    builders::TEST_PASSWORD
                )))
                .unwrap(),
        )
        .await
        .unwrap();

    // Check Set-Cookie header is present and HttpOnly
    let set_cookie = resp
        .headers()
        .get(axum::http::header::SET_COOKIE)
        .expect("login response must set cookie")
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

    assert_eq!(resp.status(), axum::http::StatusCode::OK);

    let body = axum::body::to_bytes(resp.into_body(), 1024 * 1024)
        .await
        .unwrap();
    let json: serde_json::Value = serde_json::from_slice(&body).unwrap();

    assert_eq!(json["success"], true);
    assert!(
        json["session_token"].is_string(),
        "must return session_token"
    );
    assert!(json["csrf_token"].is_string(), "must return csrf_token");
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Authenticated GET — session cookie grants access
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn authenticated_get_nodes_returns_200() {
    let app = auth_test_router();
    let (session_token, _csrf) = do_login(&app).await;

    let resp = app
        .oneshot(
            axum::http::Request::builder()
                .method("GET")
                .uri("/api/nodes")
                .header("Cookie", format!("session_token={}", session_token))
                .body(axum::body::Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "authenticated GET /api/nodes must return 200"
    );
}

#[tokio::test]
async fn authenticated_via_bearer_returns_200() {
    let app = auth_test_router();
    let (session_token, _csrf) = do_login(&app).await;

    let resp = app
        .oneshot(
            axum::http::Request::builder()
                .method("GET")
                .uri("/api/status")
                .header("Authorization", format!("Bearer {}", session_token))
                .body(axum::body::Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "Bearer token auth must also work"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  CSRF enforcement — mutations without CSRF token return 403
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn post_without_csrf_returns_403() {
    let app = auth_test_router();
    let (session_token, _csrf) = do_login(&app).await;

    let resp = app
        .oneshot(
            axum::http::Request::builder()
                .method("POST")
                .uri("/api/params/test_csrf_key")
                .header("Content-Type", "application/json")
                .header("Cookie", format!("session_token={}", session_token))
                // NO X-CSRF-Token header
                .body(axum::body::Body::from(r#"{"value": 1}"#))
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::FORBIDDEN,
        "POST without CSRF token must return 403"
    );
}

#[tokio::test]
async fn post_with_wrong_csrf_returns_403() {
    let app = auth_test_router();
    let (session_token, _csrf) = do_login(&app).await;

    let resp = app
        .oneshot(
            axum::http::Request::builder()
                .method("POST")
                .uri("/api/params/test_csrf_key2")
                .header("Content-Type", "application/json")
                .header("Cookie", format!("session_token={}", session_token))
                .header("X-CSRF-Token", "wrong_csrf_token_xyz")
                .body(axum::body::Body::from(r#"{"value": 1}"#))
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::FORBIDDEN,
        "POST with wrong CSRF token must return 403"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  CSRF enforcement — correct CSRF token allows mutation
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn post_with_correct_csrf_succeeds() {
    let app = auth_test_router();
    let (session_token, csrf_token) = do_login(&app).await;

    let resp = app
        .oneshot(
            axum::http::Request::builder()
                .method("POST")
                .uri("/api/params/test_csrf_pass")
                .header("Content-Type", "application/json")
                .header("Cookie", format!("session_token={}", session_token))
                .header("X-CSRF-Token", csrf_token)
                .body(axum::body::Body::from(r#"{"value": 42}"#))
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "POST with correct CSRF token must succeed"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Logout — invalidates session
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logout_clears_session() {
    let app = auth_test_router();
    let (session_token, _csrf) = do_login(&app).await;

    // Verify session works before logout
    let resp = app
        .clone()
        .oneshot(
            axum::http::Request::builder()
                .method("GET")
                .uri("/api/nodes")
                .header("Cookie", format!("session_token={}", session_token))
                .body(axum::body::Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "pre-logout access must work"
    );

    // Logout (no CSRF required for /api/logout)
    let resp = app
        .clone()
        .oneshot(
            axum::http::Request::builder()
                .method("POST")
                .uri("/api/logout")
                .header("Content-Type", "application/json")
                .header("Cookie", format!("session_token={}", session_token))
                .body(axum::body::Body::from(format!(
                    r#"{{"session_token":"{}"}}"#,
                    session_token
                )))
                .unwrap(),
        )
        .await
        .unwrap();

    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "logout must succeed"
    );

    // Check that the Set-Cookie clears the session
    let set_cookie = resp
        .headers()
        .get(axum::http::header::SET_COOKIE)
        .expect("logout must set cookie")
        .to_str()
        .unwrap();
    assert!(
        set_cookie.contains("Max-Age=0"),
        "logout cookie must have Max-Age=0 to clear it"
    );

    // Verify old session token is now rejected
    let resp = app
        .oneshot(
            axum::http::Request::builder()
                .method("GET")
                .uri("/api/nodes")
                .header("Cookie", format!("session_token={}", session_token))
                .body(axum::body::Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::UNAUTHORIZED,
        "old session token must be rejected after logout"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Invalid session token returns 401
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn invalid_session_token_returns_401() {
    let app = auth_test_router();

    let resp = app
        .oneshot(
            axum::http::Request::builder()
                .method("GET")
                .uri("/api/nodes")
                .header("Cookie", "session_token=totally_bogus_token")
                .body(axum::body::Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::UNAUTHORIZED,
        "invalid session token must be rejected"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  No-auth mode — all endpoints accessible without login
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn no_auth_mode_allows_unauthenticated_access() {
    // The standard test_router() has auth disabled
    let app = builders::test_router();

    let resp = app
        .clone()
        .oneshot(
            axum::http::Request::builder()
                .method("GET")
                .uri("/api/nodes")
                .body(axum::body::Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "no-auth mode must allow GET /api/nodes without login"
    );

    let resp = app
        .clone()
        .oneshot(
            axum::http::Request::builder()
                .method("GET")
                .uri("/api/topics")
                .body(axum::body::Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "no-auth mode must allow GET /api/topics without login"
    );

    let resp = app
        .oneshot(
            axum::http::Request::builder()
                .method("GET")
                .uri("/api/status")
                .body(axum::body::Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "no-auth mode must allow GET /api/status without login"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Login page accessible without auth
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn login_page_accessible_without_auth() {
    let app = auth_test_router();

    let resp = app
        .oneshot(
            axum::http::Request::builder()
                .method("GET")
                .uri("/")
                .body(axum::body::Body::empty())
                .unwrap(),
        )
        .await
        .unwrap();
    assert_eq!(
        resp.status(),
        axum::http::StatusCode::OK,
        "index page must be accessible without auth"
    );
}
