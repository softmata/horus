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

/// Session validation middleware - checks for session token in cookie or Authorization header
pub async fn session_middleware(
    State(auth_service): State<Arc<AuthService>>,
    req: Request<Body>,
    next: Next,
) -> Result<Response, StatusCode> {
    // Extract session token from Cookie header or Authorization header
    let token = extract_session_token(&req).ok_or(StatusCode::UNAUTHORIZED)?;

    // Validate session
    if !auth_service.validate_session(token) {
        return Err(StatusCode::UNAUTHORIZED);
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
