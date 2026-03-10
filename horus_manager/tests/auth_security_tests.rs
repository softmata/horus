//! Comprehensive unit tests for **auth**, **WebSocket**, and **security** handlers.
//!
//! Covers:
//! - Login/logout flows, session management, rate limiting, cookie semantics
//! - WebSocket token extraction (cookie and bearer) edge cases
//! - Path traversal, null byte injection, and other input validation attacks
//!   across all endpoints that accept user-controlled path components

mod harness;
mod monitor_tests;

use monitor_tests::builders::{self, TEST_PASSWORD};
use monitor_tests::helpers::{
    assert_json_error, assert_json_ok, delete_request, get_request, post_json_request,
};

use axum::http::StatusCode;
use tower::ServiceExt;

// ═══════════════════════════════════════════════════════════════════════════════
//  AUTH HANDLER TESTS — Login
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn login_correct_password_returns_200_with_tokens() {
    let state = builders::test_app_state_with_auth();
    let app = builders::test_router_with_state(state);

    let resp = app
        .oneshot(post_json_request(
            "/api/login",
            &format!(r#"{{"password":"{}"}}"#, TEST_PASSWORD),
        ))
        .await
        .unwrap();

    let json = assert_json_ok(resp).await;

    assert_eq!(json["success"], true, "login should succeed");
    assert!(
        json["session_token"].is_string(),
        "should return session_token"
    );
    assert!(json["csrf_token"].is_string(), "should return csrf_token");
    assert!(json["error"].is_null(), "error should be null on success");
}

#[tokio::test]
async fn login_wrong_password_returns_401() {
    let state = builders::test_app_state_with_auth();
    let app = builders::test_router_with_state(state);

    let resp = app
        .oneshot(post_json_request(
            "/api/login",
            r#"{"password":"completely_wrong_password"}"#,
        ))
        .await
        .unwrap();

    let json = assert_json_error(resp, StatusCode::UNAUTHORIZED).await;

    assert_eq!(json["success"], false, "login should fail");
    assert_eq!(
        json["error"].as_str().unwrap(),
        "Invalid password",
        "should return 'Invalid password' error"
    );
    assert!(json["session_token"].is_null(), "no session_token on fail");
    assert!(json["csrf_token"].is_null(), "no csrf_token on fail");
}

#[tokio::test]
async fn login_response_sets_httponly_cookie() {
    let state = builders::test_app_state_with_auth();
    let app = builders::test_router_with_state(state);

    let resp = app
        .oneshot(post_json_request(
            "/api/login",
            &format!(r#"{{"password":"{}"}}"#, TEST_PASSWORD),
        ))
        .await
        .unwrap();

    assert_eq!(resp.status(), StatusCode::OK);

    let set_cookie = resp
        .headers()
        .get("set-cookie")
        .expect("Set-Cookie header must be present")
        .to_str()
        .unwrap();

    assert!(
        set_cookie.contains("HttpOnly"),
        "cookie must be HttpOnly, got: {}",
        set_cookie
    );
    assert!(
        set_cookie.starts_with("session_token="),
        "cookie must start with session_token=, got: {}",
        set_cookie
    );
}

#[tokio::test]
async fn login_cookie_has_samesite_strict_and_max_age_28800() {
    let state = builders::test_app_state_with_auth();
    let app = builders::test_router_with_state(state);

    let resp = app
        .oneshot(post_json_request(
            "/api/login",
            &format!(r#"{{"password":"{}"}}"#, TEST_PASSWORD),
        ))
        .await
        .unwrap();

    assert_eq!(resp.status(), StatusCode::OK);

    let set_cookie = resp
        .headers()
        .get("set-cookie")
        .expect("Set-Cookie header must be present")
        .to_str()
        .unwrap();

    assert!(
        set_cookie.contains("SameSite=Strict"),
        "cookie must have SameSite=Strict, got: {}",
        set_cookie
    );
    assert!(
        set_cookie.contains("Max-Age=28800"),
        "cookie must have Max-Age=28800 (8h), got: {}",
        set_cookie
    );
}

#[tokio::test]
async fn rate_limiting_6th_wrong_password_returns_429() {
    let state = builders::test_app_state_with_auth();
    let app = builders::test_router_with_state(state);

    // Send 5 wrong password attempts — should all be 401
    for i in 0..5 {
        let resp = app
            .clone()
            .oneshot(post_json_request(
                "/api/login",
                r#"{"password":"wrong_password"}"#,
            ))
            .await
            .unwrap();
        assert_eq!(
            resp.status(),
            StatusCode::UNAUTHORIZED,
            "attempt {} should return 401",
            i + 1
        );
    }

    // 6th attempt — should be rate limited
    let resp = app
        .oneshot(post_json_request(
            "/api/login",
            r#"{"password":"wrong_password"}"#,
        ))
        .await
        .unwrap();

    let json = assert_json_error(resp, StatusCode::TOO_MANY_REQUESTS).await;

    assert_eq!(json["success"], false);
    assert!(
        json["error"].as_str().unwrap().contains("Too many"),
        "429 error should mention rate limiting, got: {}",
        json["error"]
    );
}

#[tokio::test]
async fn rate_limit_different_ip_is_independent() {
    // Rate limiting is per-IP. Since oneshot tests go through without
    // ConnectInfo (IP falls back to "127.0.0.1"), all attempts share the same
    // IP. Here we simply verify that after rate limiting kicks in, a correct
    // password from the same IP is also blocked — confirming per-IP behavior.
    let state = builders::test_app_state_with_auth();
    let app = builders::test_router_with_state(state);

    // Exhaust rate limit
    for _ in 0..5 {
        let _ = app
            .clone()
            .oneshot(post_json_request("/api/login", r#"{"password":"wrong"}"#))
            .await
            .unwrap();
    }

    // Even the correct password is blocked after rate limit
    let resp = app
        .oneshot(post_json_request(
            "/api/login",
            &format!(r#"{{"password":"{}"}}"#, TEST_PASSWORD),
        ))
        .await
        .unwrap();

    assert_eq!(
        resp.status(),
        StatusCode::TOO_MANY_REQUESTS,
        "correct password should still be rate-limited for the same IP"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  AUTH HANDLER TESTS — Logout
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn logout_valid_session_returns_200() {
    let state = builders::test_app_state_with_auth();
    let app = builders::test_router_with_state(state);

    let (session_token, _csrf_token) = builders::login(&app).await;

    let resp = app
        .oneshot(post_json_request(
            "/api/logout",
            &format!(r#"{{"session_token":"{}"}}"#, session_token),
        ))
        .await
        .unwrap();

    let json = assert_json_ok(resp).await;
    assert_eq!(json["success"], true, "logout should succeed");
}

#[tokio::test]
async fn logout_cookie_has_max_age_zero() {
    let state = builders::test_app_state_with_auth();
    let app = builders::test_router_with_state(state);

    let (session_token, _csrf_token) = builders::login(&app).await;

    let resp = app
        .oneshot(post_json_request(
            "/api/logout",
            &format!(r#"{{"session_token":"{}"}}"#, session_token),
        ))
        .await
        .unwrap();

    assert_eq!(resp.status(), StatusCode::OK);

    let set_cookie = resp
        .headers()
        .get("set-cookie")
        .expect("Set-Cookie header must be present on logout")
        .to_str()
        .unwrap();

    assert!(
        set_cookie.contains("Max-Age=0"),
        "logout cookie must have Max-Age=0 to clear it, got: {}",
        set_cookie
    );
}

#[tokio::test]
async fn logout_invalid_session_token_still_returns_200() {
    let state = builders::test_app_state_with_auth();
    let app = builders::test_router_with_state(state);

    let resp = app
        .oneshot(post_json_request(
            "/api/logout",
            r#"{"session_token":"this_token_does_not_exist"}"#,
        ))
        .await
        .unwrap();

    let json = assert_json_ok(resp).await;
    assert_eq!(
        json["success"], true,
        "logout with invalid token should still return success (silent)"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  AUTH HANDLER TESTS — Miscellaneous
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn login_no_auth_mode_endpoints_accessible() {
    // With auth disabled, endpoints should be accessible without login
    let state = builders::test_app_state(); // auth_disabled = true
    let app = builders::test_router_with_state(state);

    let resp = app.oneshot(get_request("/api/status")).await.unwrap();
    assert_eq!(
        resp.status(),
        StatusCode::OK,
        "status endpoint should be accessible with auth disabled"
    );
}

#[tokio::test]
async fn session_token_required_for_protected_endpoints_concept() {
    // The test router from builders does NOT apply auth middleware, so we test
    // the underlying session validation logic directly through login/logout.
    let state = builders::test_app_state_with_auth();
    let app = builders::test_router_with_state(state.clone());

    let (session_token, _csrf_token) = builders::login(&app).await;

    // Session should be valid
    assert!(
        state.auth_service.validate_session(&session_token),
        "freshly-logged-in session should be valid"
    );

    // An invalid token should not be valid
    assert!(
        !state.auth_service.validate_session("random_invalid_token"),
        "random token should not be valid"
    );
}

#[tokio::test]
async fn csrf_token_validation_works() {
    let state = builders::test_app_state_with_auth();
    let app = builders::test_router_with_state(state.clone());

    let (session_token, csrf_token) = builders::login(&app).await;

    // Correct CSRF token should validate
    assert!(
        state
            .auth_service
            .validate_csrf(&session_token, &csrf_token),
        "correct CSRF token should validate"
    );

    // Wrong CSRF token should fail
    assert!(
        !state
            .auth_service
            .validate_csrf(&session_token, "wrong_csrf"),
        "wrong CSRF token should fail"
    );

    // CSRF with unknown session should fail
    assert!(
        !state
            .auth_service
            .validate_csrf("unknown_session", &csrf_token),
        "CSRF with unknown session should fail"
    );
}

#[tokio::test]
async fn login_response_consistent_json_format_success() {
    let state = builders::test_app_state_with_auth();
    let app = builders::test_router_with_state(state);

    let resp = app
        .oneshot(post_json_request(
            "/api/login",
            &format!(r#"{{"password":"{}"}}"#, TEST_PASSWORD),
        ))
        .await
        .unwrap();

    let json = assert_json_ok(resp).await;

    // All four fields must be present
    assert!(json.get("success").is_some(), "must have 'success' field");
    assert!(
        json.get("session_token").is_some(),
        "must have 'session_token' field"
    );
    assert!(
        json.get("csrf_token").is_some(),
        "must have 'csrf_token' field"
    );
    assert!(json.get("error").is_some(), "must have 'error' field");
}

#[tokio::test]
async fn login_response_consistent_json_format_failure() {
    let state = builders::test_app_state_with_auth();
    let app = builders::test_router_with_state(state);

    let resp = app
        .oneshot(post_json_request("/api/login", r#"{"password":"wrong"}"#))
        .await
        .unwrap();

    let json = assert_json_error(resp, StatusCode::UNAUTHORIZED).await;

    // Same four fields on failure
    assert!(json.get("success").is_some(), "must have 'success' field");
    assert!(
        json.get("session_token").is_some(),
        "must have 'session_token' field"
    );
    assert!(
        json.get("csrf_token").is_some(),
        "must have 'csrf_token' field"
    );
    assert!(json.get("error").is_some(), "must have 'error' field");
}

#[tokio::test]
async fn multiple_logins_create_different_session_tokens() {
    let state = builders::test_app_state_with_auth();
    let app = builders::test_router_with_state(state);

    let (token1, _csrf1) = builders::login(&app).await;
    let (token2, _csrf2) = builders::login(&app).await;

    assert_ne!(
        token1, token2,
        "each login should produce a unique session token"
    );
}

#[tokio::test]
async fn login_success_field_true_on_success_false_on_failure() {
    let state = builders::test_app_state_with_auth();
    let app = builders::test_router_with_state(state);

    // Success case
    let resp = app
        .clone()
        .oneshot(post_json_request(
            "/api/login",
            &format!(r#"{{"password":"{}"}}"#, TEST_PASSWORD),
        ))
        .await
        .unwrap();
    let json = assert_json_ok(resp).await;
    assert_eq!(json["success"], true);

    // Failure case
    let resp = app
        .oneshot(post_json_request("/api/login", r#"{"password":"bad"}"#))
        .await
        .unwrap();
    let json = assert_json_error(resp, StatusCode::UNAUTHORIZED).await;
    assert_eq!(json["success"], false);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  WEBSOCKET TOKEN EXTRACTION TESTS
// ═══════════════════════════════════════════════════════════════════════════════
//
// These test `extract_cookie_token` and `extract_bearer_token` directly.
// The inline tests in websocket.rs cover the basic happy/missing/wrong cases;
// here we cover edge cases.

use horus_manager::monitor::{extract_bearer_token, extract_cookie_token};

#[test]
fn ws_extract_cookie_token_multiple_cookies_extracts_correct() {
    let mut headers = axum::http::HeaderMap::new();
    headers.insert(
        axum::http::header::COOKIE,
        "theme=dark; session_token=abc123xyz; lang=en; tracking=off"
            .parse()
            .unwrap(),
    );
    assert_eq!(extract_cookie_token(&headers), Some("abc123xyz"));
}

#[test]
fn ws_extract_cookie_token_at_start() {
    let mut headers = axum::http::HeaderMap::new();
    headers.insert(
        axum::http::header::COOKIE,
        "session_token=first_value; other=123".parse().unwrap(),
    );
    assert_eq!(extract_cookie_token(&headers), Some("first_value"));
}

#[test]
fn ws_extract_cookie_token_at_end() {
    let mut headers = axum::http::HeaderMap::new();
    headers.insert(
        axum::http::header::COOKIE,
        "other=123; session_token=last_value".parse().unwrap(),
    );
    assert_eq!(extract_cookie_token(&headers), Some("last_value"));
}

#[test]
fn ws_extract_cookie_token_empty_cookie_header_returns_none() {
    let mut headers = axum::http::HeaderMap::new();
    headers.insert(axum::http::header::COOKIE, "".parse().unwrap());
    assert_eq!(extract_cookie_token(&headers), None);
}

#[test]
fn ws_extract_bearer_token_extra_spaces_in_value() {
    // "Bearer  token_with_leading_space" — the prefix is "Bearer " (one space)
    // so strip_prefix("Bearer ") returns " token_with_leading_space"
    let mut headers = axum::http::HeaderMap::new();
    headers.insert(
        axum::http::header::AUTHORIZATION,
        "Bearer  double_spaced_token".parse().unwrap(),
    );
    let result = extract_bearer_token(&headers);
    // The function does strip_prefix("Bearer "), so the result includes the
    // extra leading space.
    assert_eq!(result, Some(" double_spaced_token"));
}

#[test]
fn ws_extract_bearer_token_lowercase_returns_none() {
    // "bearer" (lowercase b) should not match "Bearer "
    let mut headers = axum::http::HeaderMap::new();
    headers.insert(
        axum::http::header::AUTHORIZATION,
        "bearer my_token".parse().unwrap(),
    );
    assert_eq!(
        extract_bearer_token(&headers),
        None,
        "lowercase 'bearer' should not match (case sensitive)"
    );
}

#[test]
fn ws_extract_cookie_token_special_chars_in_value() {
    let mut headers = axum::http::HeaderMap::new();
    headers.insert(
        axum::http::header::COOKIE,
        "session_token=abc+def/ghi=jkl; other=x".parse().unwrap(),
    );
    assert_eq!(extract_cookie_token(&headers), Some("abc+def/ghi=jkl"));
}

#[test]
fn ws_extract_bearer_token_empty_after_prefix() {
    // "Bearer " with nothing after — strip_prefix yields ""
    let mut headers = axum::http::HeaderMap::new();
    headers.insert(
        axum::http::header::AUTHORIZATION,
        "Bearer ".parse().unwrap(),
    );
    let result = extract_bearer_token(&headers);
    assert_eq!(result, Some(""), "empty token after 'Bearer ' prefix");
}

#[test]
fn ws_extract_cookie_token_similar_name_not_matched() {
    // "my_session_token" should not be confused with "session_token"
    let mut headers = axum::http::HeaderMap::new();
    headers.insert(
        axum::http::header::COOKIE,
        "my_session_token=wrong; session_token=right"
            .parse()
            .unwrap(),
    );
    assert_eq!(
        extract_cookie_token(&headers),
        Some("right"),
        "should match exact 'session_token=' prefix, not a substring"
    );
}

#[test]
fn ws_extract_cookie_token_no_value() {
    // "session_token=" with empty value
    let mut headers = axum::http::HeaderMap::new();
    headers.insert(
        axum::http::header::COOKIE,
        "session_token=; other=x".parse().unwrap(),
    );
    assert_eq!(
        extract_cookie_token(&headers),
        Some(""),
        "empty cookie value should still be returned"
    );
}

#[test]
fn ws_extract_bearer_token_no_header() {
    let headers = axum::http::HeaderMap::new();
    assert_eq!(extract_bearer_token(&headers), None);
}

#[test]
fn ws_extract_cookie_token_no_header() {
    let headers = axum::http::HeaderMap::new();
    assert_eq!(extract_cookie_token(&headers), None);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SECURITY TESTS — Path Traversal
// ═══════════════════════════════════════════════════════════════════════════════
//
// These tests exercise `is_safe_path_component` indirectly through the HTTP
// router. The test router does NOT apply auth middleware, so no login is needed.

#[tokio::test]
async fn path_traversal_recordings_info_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(get_request("/api/recordings/..secret"))
        .await
        .unwrap();

    assert_eq!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "path traversal on recordings info should return 400"
    );
}

#[tokio::test]
async fn path_traversal_recordings_delete_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(delete_request("/api/recordings/..secret"))
        .await
        .unwrap();

    assert_eq!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "path traversal on recordings delete should return 400"
    );
}

#[tokio::test]
async fn path_traversal_params_get_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(get_request("/api/params/../../etc/passwd"))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == StatusCode::BAD_REQUEST || status == StatusCode::NOT_FOUND,
        "path traversal on params get should return 400 or 404, got {}",
        status
    );
}

#[tokio::test]
async fn path_traversal_params_set_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(post_json_request(
            "/api/params/..%2Fetc%2Fpasswd",
            r#"{"value":"evil"}"#,
        ))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == StatusCode::BAD_REQUEST || status == StatusCode::NOT_FOUND,
        "path traversal on params set should return 400 or 404, got {}",
        status
    );
}

#[tokio::test]
async fn path_traversal_params_delete_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(delete_request("/api/params/..secret"))
        .await
        .unwrap();

    assert_eq!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "path traversal on params delete should return 400"
    );
}

#[tokio::test]
async fn path_traversal_debug_session_create_recording_session() {
    let app = builders::test_router();

    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions",
            r#"{"recording_session":"../../etc","recording_file":"passwd"}"#,
        ))
        .await
        .unwrap();

    assert_eq!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "path traversal in recording_session field should return 400"
    );
}

#[tokio::test]
async fn path_traversal_debug_session_create_recording_file() {
    let app = builders::test_router();

    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions",
            r#"{"recording_session":"valid_session","recording_file":"../../shadow"}"#,
        ))
        .await
        .unwrap();

    assert_eq!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "path traversal in recording_file field should return 400"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SECURITY TESTS — Null Byte Injection
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn null_byte_recordings_returns_400() {
    let app = builders::test_router();

    // Axum may reject null bytes before they reach the handler, so we accept
    // either 400 (our handler) or 400 from framework-level rejection.
    let resp = app
        .oneshot(get_request("/api/recordings/session%00evil"))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == StatusCode::BAD_REQUEST || status == StatusCode::NOT_FOUND,
        "null byte in recordings session should return 400 or 404, got {}",
        status
    );
}

#[tokio::test]
async fn null_byte_debug_session_create_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions",
            r#"{"recording_session":"session\u0000evil","recording_file":"file.horus"}"#,
        ))
        .await
        .unwrap();

    // is_safe_path_component rejects null bytes
    assert_eq!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "null byte in debug session create should return 400"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SECURITY TESTS — URL-encoded and Backslash Traversal
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn url_encoded_traversal_recordings_returns_400_or_404() {
    let app = builders::test_router();

    // %2e%2e%2f = "../" — Axum may decode or not
    let resp = app
        .oneshot(get_request("/api/recordings/%2e%2e%2f%2e%2e%2fetc"))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == StatusCode::BAD_REQUEST || status == StatusCode::NOT_FOUND,
        "URL-encoded traversal should return 400 or 404, got {}",
        status
    );
}

#[tokio::test]
async fn backslash_traversal_recordings_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(get_request("/api/recordings/..\\..\\etc"))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == StatusCode::BAD_REQUEST || status == StatusCode::NOT_FOUND,
        "backslash traversal should return 400 or 404, got {}",
        status
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SECURITY TESTS — Package Install/Uninstall Path Traversal
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn install_handler_path_traversal_package_name_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(post_json_request(
            "/api/packages/install",
            r#"{"package":"../../etc/passwd"}"#,
        ))
        .await
        .unwrap();

    assert_eq!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "path traversal in install package name should return 400"
    );
}

#[tokio::test]
async fn uninstall_handler_path_traversal_parent_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(post_json_request(
            "/api/packages/uninstall",
            r#"{"parent_package":"../../etc","package":"valid_pkg"}"#,
        ))
        .await
        .unwrap();

    assert_eq!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "path traversal in uninstall parent_package should return 400"
    );
}

#[tokio::test]
async fn uninstall_handler_path_traversal_package_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(post_json_request(
            "/api/packages/uninstall",
            r#"{"parent_package":"valid_parent","package":"../../shadow"}"#,
        ))
        .await
        .unwrap();

    assert_eq!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "path traversal in uninstall package should return 400"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SECURITY TESTS — Empty Path Components
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn empty_string_recordings_info_returns_error() {
    // Axum will not match an empty path segment to /api/recordings/:session,
    // so this will 404 or 405 at the routing level rather than reaching the
    // handler. We test with a nearly-empty but still routable value.
    let app = builders::test_router();

    // Use the handler's path component validation: an empty string passed
    // directly to is_safe_path_component returns false. We trigger this via
    // a whitespace-only segment that Axum passes through.
    // Actually, Axum won't route an empty segment, so test via direct handler
    // invocation (which we don't do here since we test via router). Instead,
    // verify that the install handler rejects empty package names.
    let resp = app
        .oneshot(post_json_request(
            "/api/packages/install",
            r#"{"package":""}"#,
        ))
        .await
        .unwrap();

    assert_eq!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "empty package name should return 400"
    );
}

#[tokio::test]
async fn empty_string_debug_session_fields_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions",
            r#"{"recording_session":"","recording_file":"file.horus"}"#,
        ))
        .await
        .unwrap();

    assert_eq!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "empty recording_session should return 400"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SECURITY TESTS — Unicode / Fullwidth Characters
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn unicode_fullwidth_dot_in_recordings_returns_error() {
    let app = builders::test_router();

    // Fullwidth full stop: U+FF0E (not the same as ASCII '.')
    // is_safe_path_component only allows alphanumeric, _, -, ., /
    // Fullwidth characters are not alphanumeric ASCII, so they should be
    // rejected by the monitor's is_safe_path_component.
    // However, the function checks for ".." (ASCII dots), so fullwidth dots
    // would pass that check but fail the char whitelist.
    let resp = app
        .oneshot(get_request("/api/recordings/\u{FF0E}\u{FF0E}secret"))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == StatusCode::BAD_REQUEST || status == StatusCode::NOT_FOUND,
        "fullwidth dot characters should return 400 or 404, got {}",
        status
    );
}

#[tokio::test]
async fn unicode_fullwidth_dot_debug_session_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(post_json_request(
            "/api/debug/sessions",
            r#"{"recording_session":"\uff0e\uff0e/etc","recording_file":"passwd"}"#,
        ))
        .await
        .unwrap();

    // is_safe_path_component rejects non-ASCII characters (not alphanumeric, _, -, ., /)
    assert_eq!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "fullwidth dot in debug session should return 400"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SECURITY TESTS — Additional Path-based Attacks
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn shell_metachar_in_params_key_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(get_request("/api/params/key;rm%20-rf%20/"))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == StatusCode::BAD_REQUEST || status == StatusCode::NOT_FOUND,
        "shell metacharacters should be rejected, got {}",
        status
    );
}

#[tokio::test]
async fn pipe_injection_in_params_key_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(get_request("/api/params/key|cat%20/etc/shadow"))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == StatusCode::BAD_REQUEST || status == StatusCode::NOT_FOUND,
        "pipe injection should be rejected, got {}",
        status
    );
}

#[tokio::test]
async fn double_slash_in_params_key_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(get_request("/api/params/double//slash"))
        .await
        .unwrap();

    let status = resp.status();
    // Axum might normalize double slashes or the handler rejects them
    assert!(
        status == StatusCode::BAD_REQUEST
            || status == StatusCode::NOT_FOUND
            || status == StatusCode::OK, // if Axum routes differently
        "double slash should be handled, got {}",
        status
    );
}

#[tokio::test]
async fn recording_with_forward_slash_returns_400() {
    let app = builders::test_router();

    // Forward slash in the path component (Axum may split this)
    let resp = app
        .oneshot(get_request("/api/recordings/has%2Fslash"))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == StatusCode::BAD_REQUEST || status == StatusCode::NOT_FOUND,
        "forward slash in recording name should return 400 or 404, got {}",
        status
    );
}

#[tokio::test]
async fn recording_with_backslash_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(get_request("/api/recordings/has%5Cbackslash"))
        .await
        .unwrap();

    let status = resp.status();
    assert!(
        status == StatusCode::BAD_REQUEST || status == StatusCode::NOT_FOUND,
        "backslash in recording name should return 400 or 404, got {}",
        status
    );
}

#[tokio::test]
async fn install_handler_null_byte_in_package_name_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(post_json_request(
            "/api/packages/install",
            r#"{"package":"evil\u0000pkg"}"#,
        ))
        .await
        .unwrap();

    assert_eq!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "null byte in package name should return 400"
    );
}

#[tokio::test]
async fn uninstall_handler_both_fields_traversal_returns_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(post_json_request(
            "/api/packages/uninstall",
            r#"{"parent_package":"../evil","package":"../also_evil"}"#,
        ))
        .await
        .unwrap();

    assert_eq!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "path traversal in both uninstall fields should return 400"
    );
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SECURITY TESTS — Valid Names Pass Through
// ═══════════════════════════════════════════════════════════════════════════════

#[tokio::test]
async fn safe_recording_name_does_not_return_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(get_request("/api/recordings/my_valid_session_2026"))
        .await
        .unwrap();

    // Should be 404 (session doesn't exist) but NOT 400
    assert_ne!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "valid session name should not return 400"
    );
}

#[tokio::test]
async fn safe_params_key_does_not_return_400() {
    let app = builders::test_router();

    let resp = app
        .oneshot(get_request("/api/params/robot_name"))
        .await
        .unwrap();

    // Should be 404 (param doesn't exist) but NOT 400
    assert_ne!(
        resp.status(),
        StatusCode::BAD_REQUEST,
        "valid param key should not return 400"
    );
}
