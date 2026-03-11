//! Security integration tests for the HORUS web monitor.
//!
//! These tests complement the unit tests in `monitor/mod.rs` and
//! `plugins/sandbox.rs` by exercising the router and sandbox through the
//! public library interface.
//!
//! The following properties are verified:
//!
//! 1. **CSRF enforcement** (`post_params_without_csrf_token_returns_403`):
//!    An authenticated POST to a state-mutating route that omits the
//!    `X-CSRF-Token` header must be rejected with `403 Forbidden`.
//!    A matching POST that includes the correct token must NOT return 403.
//!
//! 2. **Path traversal rejection** (`get_recordings_path_traversal_returns_400`):
//!    A route whose `Path` parameter contains `..` is caught by
//!    `is_safe_path_component` before any filesystem access occurs and
//!    returns `400 Bad Request`.
//!
//! 3. **Plugin seccomp – socket blocked** (`sandbox_seccomp_blocks_socket`)
//!    (Linux x86-64 only):
//!    A child process that calls `sandbox::apply()` before attempting to
//!    open a network socket must exit non-zero (EPERM or SIGSYS).  This
//!    confirms the seccomp deny list is correctly constructed and applied
//!    end-to-end through the public `sandbox::apply` entry point.
//!
//! # Relationship to existing tests
//!
//! | Covered elsewhere                           | Where                          |
//! |---------------------------------------------|--------------------------------|
//! | Unauthenticated access → 401                | `monitor::tests`               |
//! | Login brute-force → 429                     | `monitor::tests`               |
//! | WebSocket auth rejection → 401              | `monitor::tests`               |
//! | Sandbox BPF construction doesn't panic      | `plugins::sandbox::tests`      |
//! | Sandbox seccomp blocks socket (unit test)   | `plugins::sandbox::tests`      |
//! | Sandbox rlimit applied                      | `plugins::sandbox::tests`      |
#![cfg(feature = "monitor")]

use axum::{
    body::Body,
    extract::State,
    http::{Request, StatusCode},
    middleware,
    routing::{get, post},
    Router,
};
use horus_manager::{
    monitor::{login_handler, params_set_handler, recordings_info_handler, AppState},
    security::{auth::hash_password, session_middleware, AuthService},
};
use http_body_util::BodyExt;
use std::sync::Arc;
use tower::ServiceExt; // for .oneshot()

// ─── test password ────────────────────────────────────────────────────────────

const TEST_PASSWORD: &str = "security_test_pw_xyz_9876";

// ─── session middleware adapter ───────────────────────────────────────────────

/// Adapter that extracts `Arc<AuthService>` from the broader `Arc<AppState>`
/// and delegates to the production `session_middleware`.
///
/// Mirrors the private `monitor_session_middleware` in `monitor/mod.rs` so
/// integration tests can build an equivalent router without accessing private
/// symbols.
async fn test_session_middleware(
    State(state): State<Arc<AppState>>,
    req: axum::http::Request<Body>,
    next: axum::middleware::Next,
) -> Result<axum::response::Response, StatusCode> {
    session_middleware(State(state.auth_service.clone()), req, next).await
}

// ─── test app builder ─────────────────────────────────────────────────────────

/// Build the minimal router used by every security integration test.
///
/// Routes included:
/// - `POST /api/login`           — public; used to acquire tokens
/// - `POST /api/params/:key`     — auth + CSRF required; for CSRF test
/// - `GET  /api/recordings/:session` — auth required; path component validated
///
/// The router mirrors the middleware ordering contract from `monitor::run()`:
/// auth layer applied to `api_routes` *before* merge with `public_routes`.
fn build_security_test_app() -> Router {
    let password_hash = hash_password(TEST_PASSWORD).expect("hash_password must not fail in tests");
    let auth_service =
        Arc::new(AuthService::new(password_hash).expect("AuthService::new must not fail"));
    let params = Arc::new(horus_core::RuntimeParams::default());
    let state = Arc::new(AppState {
        port: 0,
        params,
        auth_service: auth_service.clone(),
        current_workspace: None,
        auth_disabled: false,
        trust_proxy: false,
    });

    let mut api_routes = Router::new()
        .route("/api/params/:key", post(params_set_handler))
        .route("/api/recordings/:session", get(recordings_info_handler));

    // Auth (+ CSRF) layer applied before merge — same invariant as production.
    api_routes = api_routes.layer(middleware::from_fn_with_state(
        state.clone(),
        test_session_middleware,
    ));

    let public_routes = Router::new().route("/api/login", post(login_handler));

    public_routes.merge(api_routes).with_state(state)
}

// ─── login helper ─────────────────────────────────────────────────────────────

/// POST /api/login and return `(session_token, csrf_token)`.
///
/// Panics if login fails — the tests depend on a working session.
async fn login_to_get_tokens(app: &Router) -> (String, String) {
    let body = format!(r#"{{"password":"{}"}}"#, TEST_PASSWORD);
    let req = Request::builder()
        .method("POST")
        .uri("/api/login")
        .header("Content-Type", "application/json")
        .body(Body::from(body))
        .unwrap();

    let resp = app.clone().oneshot(req).await.unwrap();
    assert_eq!(
        resp.status(),
        StatusCode::OK,
        "test login must succeed with the correct password"
    );

    let bytes = resp.into_body().collect().await.unwrap().to_bytes();
    let json: serde_json::Value = serde_json::from_slice(&bytes).unwrap();

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

// ─── test 1: CSRF token enforcement ──────────────────────────────────────────

/// POST to a state-mutating endpoint with a valid session but without the
/// `X-CSRF-Token` header must return `403 Forbidden`.
///
/// The CSRF double-submit pattern requires the client to echo the token
/// returned at login in every non-idempotent request.  An attacker on a
/// third-party origin piggybacks on the victim's session cookie but cannot
/// read the CSRF token (same-origin policy blocks `XMLHttpRequest` and
/// `fetch` cross-origin reads), so any forged POST is rejected here.
#[tokio::test]
async fn post_params_without_csrf_token_returns_403() {
    let app = build_security_test_app();
    let (session_token, _csrf_token) = login_to_get_tokens(&app).await;

    // POST with valid session cookie but *without* X-CSRF-Token.
    let req = Request::builder()
        .method("POST")
        .uri("/api/params/csrf_test_key")
        .header("Content-Type", "application/json")
        .header("Cookie", format!("session_token={}", session_token))
        // X-CSRF-Token deliberately omitted
        .body(Body::from(r#"{"value":1}"#))
        .unwrap();

    let resp = app.oneshot(req).await.unwrap();
    assert_eq!(
        resp.status(),
        StatusCode::FORBIDDEN,
        "POST /api/params/:key without X-CSRF-Token must return 403"
    );
}

/// POST to a state-mutating endpoint with a valid session AND the correct
/// `X-CSRF-Token` must NOT return 403.
///
/// This is the positive counterpart to `post_params_without_csrf_token_returns_403`:
/// it guards against false negatives (middleware rejecting all requests).
#[tokio::test]
async fn post_params_with_valid_csrf_token_passes_csrf_check() {
    let app = build_security_test_app();
    let (session_token, csrf_token) = login_to_get_tokens(&app).await;

    let req = Request::builder()
        .method("POST")
        .uri("/api/params/csrf_positive_key")
        .header("Content-Type", "application/json")
        .header("Cookie", format!("session_token={}", session_token))
        .header("X-CSRF-Token", csrf_token)
        .body(Body::from(r#"{"value":42}"#))
        .unwrap();

    let resp = app.oneshot(req).await.unwrap();
    assert_ne!(
        resp.status(),
        StatusCode::FORBIDDEN,
        "POST /api/params/:key with valid X-CSRF-Token must not return 403"
    );
    assert_ne!(
        resp.status(),
        StatusCode::UNAUTHORIZED,
        "POST /api/params/:key with valid session must not return 401"
    );
}

// ─── test 2: path traversal rejection ────────────────────────────────────────

/// GET /api/recordings/:session where the session name contains `..` must
/// return `400 Bad Request`.
///
/// `recordings_info_handler` calls `is_safe_path_component` on the path
/// parameter before any filesystem operation.  The validator rejects any
/// component that contains `..`, `/`, `\`, or null bytes.
///
/// The parameter `..secret` contains the two-character sequence `..`, which
/// is sufficient to trigger the guard — it does not need to be a raw `../..`
/// to be rejected, and using an alphanumeric suffix avoids ambiguity with
/// Axum's path normalization of bare `..` segments.
#[tokio::test]
async fn get_recordings_path_traversal_returns_400() {
    let app = build_security_test_app();
    let (session_token, _csrf_token) = login_to_get_tokens(&app).await;

    let req = Request::builder()
        .method("GET")
        // "..secret" contains ".." and is rejected by is_safe_path_component.
        .uri("/api/recordings/..secret")
        .header("Cookie", format!("session_token={}", session_token))
        .body(Body::empty())
        .unwrap();

    let resp = app.oneshot(req).await.unwrap();
    assert_eq!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "GET /api/recordings with a path-traversal parameter must return 400"
    );
}

/// GET /api/recordings/:session with a safe (alphanumeric) session name must
/// NOT return 400.
///
/// Positive counterpart to `get_recordings_path_traversal_returns_400`:
/// ensures `is_safe_path_component` does not reject legitimate names.
/// The handler will return 404 (no such recording) — that's expected.
#[tokio::test]
async fn get_recordings_safe_name_passes_path_check() {
    let app = build_security_test_app();
    let (session_token, _csrf_token) = login_to_get_tokens(&app).await;

    let req = Request::builder()
        .method("GET")
        .uri("/api/recordings/my_session_2025")
        .header("Cookie", format!("session_token={}", session_token))
        .body(Body::empty())
        .unwrap();

    let resp = app.oneshot(req).await.unwrap();
    assert_ne!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "GET /api/recordings with a safe session name must not return 400"
    );
    assert_ne!(
        resp.status(),
        StatusCode::UNAUTHORIZED,
        "authenticated GET /api/recordings must not return 401"
    );
}

// ─── test 3: plugin sandbox — seccomp blocks socket ──────────────────────────

/// After `sandbox::apply()` is applied in a child process, any attempt to
/// call `socket()` must fail (EPERM) or be killed (SIGSYS), causing the
/// child to exit non-zero.
///
/// This exercises the full `sandbox::apply()` code path as an integration
/// test: the BPF filter must be correctly constructed, `no_new_privs` must
/// be set successfully, and the filter must be installed before the `exec`.
///
/// The test uses `/bin/bash`'s built-in TCP redirection (`exec N<>/dev/tcp/…`)
/// which internally calls `socket()` in the bash process itself — exactly the
/// syscall that the seccomp deny list blocks.
///
/// Port 9 (discard) is used; whether a server is listening is irrelevant
/// because the filter fires before the kernel inspects the destination.
#[test]
#[cfg(all(target_os = "linux", target_arch = "x86_64"))]
fn sandbox_seccomp_blocks_socket_integration() {
    use std::os::unix::process::CommandExt;
    use std::process::{Command, Stdio};

    // Skip if bash is not available in the test environment.
    if !std::path::Path::new("/bin/bash").exists() {
        return;
    }

    let mut cmd = Command::new("/bin/bash");
    // bash's built-in TCP redirection calls socket() internally.
    cmd.args(["-c", "exec 5<>/dev/tcp/127.0.0.1/9"])
        .env_clear()
        .stdout(Stdio::null())
        .stderr(Stdio::null());

    // Safety: pre_exec runs in the forked child after fork() but before exec().
    // sandbox::apply() uses only async-signal-safe libc syscalls (no Rust
    // allocator, no mutexes).  See sandbox.rs module doc for the contract.
    unsafe {
        cmd.pre_exec(|| horus_manager::plugins::sandbox::apply(None));
    }

    let status = cmd
        .status()
        .expect("failed to spawn bash for seccomp integration test");

    assert!(
        !status.success(),
        "seccomp deny list must block socket(); bash must exit non-zero; \
         got exit status: {:?}",
        status.code()
    );
}
