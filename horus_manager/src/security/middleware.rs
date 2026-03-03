//! Security middleware for Axum - session validation and security headers

use super::auth::AuthService;
use axum::{
    body::Body,
    extract::State,
    http::{header, Request, StatusCode},
    middleware::Next,
    response::Response,
};
use std::sync::Arc;

/// Session validation middleware.
///
/// Checks for a valid session token (Cookie or Authorization header) on every
/// request.  Additionally, for state-mutating methods (POST, PUT, DELETE,
/// PATCH), it enforces the CSRF double-submit pattern: the client must supply
/// the `X-CSRF-Token` header with the token that was returned in the login
/// JSON response.
///
/// The `/api/logout` POST route is excluded from CSRF checking because it is
/// self-directed and idempotent (at worst an attacker can log the victim out,
/// which is harmless compared to the alternative of requiring a CSRF token
/// before allowing logout).
pub async fn session_middleware(
    State(auth_service): State<Arc<AuthService>>,
    req: Request<Body>,
    next: Next,
) -> Result<Response, StatusCode> {
    // Extract session token from Cookie header or Authorization header.
    // The lifetime of `token` is tied to `req`, so it must not outlive `req`.
    let token = extract_session_token(&req).ok_or(StatusCode::UNAUTHORIZED)?;

    // Validate session (idle + absolute timeout check).
    if !auth_service.validate_session(token) {
        return Err(StatusCode::UNAUTHORIZED);
    }

    // CSRF check for state-mutating methods.
    let method = req.method();
    if matches!(
        *method,
        axum::http::Method::POST
            | axum::http::Method::PUT
            | axum::http::Method::DELETE
            | axum::http::Method::PATCH
    ) {
        // /api/logout only needs a valid session; skip CSRF for it.
        let path = req.uri().path();
        if path != "/api/logout" {
            let csrf_token = extract_csrf_token(&req).ok_or(StatusCode::FORBIDDEN)?;
            if !auth_service.validate_csrf(token, csrf_token) {
                return Err(StatusCode::FORBIDDEN);
            }
        }
    }

    Ok(next.run(req).await)
}

/// Extract session token from request (checks both Cookie and Authorization headers)
fn extract_session_token(req: &Request<Body>) -> Option<&str> {
    // Try Cookie header first
    if let Some(cookie_header) = req.headers().get(header::COOKIE) {
        if let Ok(cookie_str) = cookie_header.to_str() {
            for cookie in cookie_str.split(';') {
                let cookie = cookie.trim();
                if let Some(value) = cookie.strip_prefix("session_token=") {
                    return Some(value);
                }
            }
        }
    }

    // Try Authorization header as fallback (Bearer token)
    if let Some(auth_header) = req.headers().get(header::AUTHORIZATION) {
        if let Ok(auth_str) = auth_header.to_str() {
            if let Some(token) = auth_str.strip_prefix("Bearer ") {
                return Some(token);
            }
        }
    }

    None
}

/// Extract CSRF token from the `X-CSRF-Token` request header.
fn extract_csrf_token(req: &Request<Body>) -> Option<&str> {
    req.headers()
        .get("X-CSRF-Token")
        .and_then(|v| v.to_str().ok())
}

/// Security headers middleware
pub async fn security_headers_middleware(req: Request<Body>, next: Next) -> Response {
    let mut response = next.run(req).await;
    let headers = response.headers_mut();

    // Content Security Policy
    headers.insert(
        "Content-Security-Policy",
        "default-src 'self' 'unsafe-inline' 'unsafe-eval'; connect-src 'self'"
            .parse()
            .expect("valid static header"),
    );

    // Prevent clickjacking
    headers.insert(
        "X-Frame-Options",
        "DENY".parse().expect("valid static header"),
    );

    // Prevent MIME sniffing
    headers.insert(
        "X-Content-Type-Options",
        "nosniff".parse().expect("valid static header"),
    );

    // Enable XSS protection
    headers.insert(
        "X-XSS-Protection",
        "1; mode=block".parse().expect("valid static header"),
    );

    // Referrer policy
    headers.insert(
        "Referrer-Policy",
        "strict-origin-when-cross-origin"
            .parse()
            .expect("valid static header"),
    );

    // HSTS (only for HTTPS)
    headers.insert(
        "Strict-Transport-Security",
        "max-age=31536000; includeSubDomains"
            .parse()
            .expect("valid static header"),
    );

    response
}
