use axum::{
    extract::{ConnectInfo, State},
    http::{HeaderMap, StatusCode},
    response::IntoResponse,
    Json,
};
use std::net::SocketAddr;
use std::sync::Arc;

use crate::monitor::AppState;

/// Extract the real client IP address.
///
/// When `trust_proxy` is `true`, the function first checks the
/// `X-Forwarded-For` header (taking the leftmost — i.e. original client —
/// address) and then `X-Real-IP` before falling back to the TCP peer address.
///
/// When `trust_proxy` is `false` (the default), proxy headers are ignored
/// entirely so a remote client cannot craft a spoofed `X-Forwarded-For` value
/// to impersonate a trusted IP and bypass per-IP rate limiting.
///
/// Falls back to `"127.0.0.1"` when no peer address is available, which
/// happens in unit tests that use `Router::oneshot` without
/// `into_make_service_with_connect_info`.
fn extract_client_ip(
    peer_addr: Option<SocketAddr>,
    headers: &HeaderMap,
    trust_proxy: bool,
) -> String {
    if trust_proxy {
        // X-Forwarded-For: client, proxy1, proxy2  →  take the first entry.
        if let Some(xff) = headers.get("x-forwarded-for") {
            if let Ok(val) = xff.to_str() {
                if let Some(first) = val.split(',').next() {
                    let ip = first.trim().to_string();
                    if !ip.is_empty() {
                        return ip;
                    }
                }
            }
        }
        // X-Real-IP: single client address set by some proxies (e.g. nginx).
        if let Some(xri) = headers.get("x-real-ip") {
            if let Ok(val) = xri.to_str() {
                let ip = val.trim().to_string();
                if !ip.is_empty() {
                    return ip;
                }
            }
        }
    }
    // Use the actual TCP peer address, or fall back for test environments.
    peer_addr
        .map(|a| a.ip().to_string())
        .unwrap_or_else(|| "127.0.0.1".to_string())
}

#[derive(serde::Deserialize)]
pub struct LoginRequest {
    password: String,
}

#[derive(serde::Serialize)]
struct LoginResponse {
    success: bool,
    /// The session token.  Clients should store this in their state but the
    /// browser also receives it via the `HttpOnly` cookie — it is included in
    /// the body for non-browser API clients.
    session_token: Option<String>,
    /// CSRF token bound to this session.  Clients MUST send this value in the
    /// `X-CSRF-Token` header for every POST / PUT / DELETE / PATCH request.
    csrf_token: Option<String>,
    error: Option<String>,
}

/// Login handler - validates password and creates session
pub async fn login_handler(
    State(state): State<Arc<AppState>>,
    connect_info: Option<ConnectInfo<SocketAddr>>,
    headers: HeaderMap,
    Json(login_req): Json<LoginRequest>,
) -> impl IntoResponse {
    let ip_address = Some(extract_client_ip(
        connect_info.map(|ci| ci.0),
        &headers,
        state.trust_proxy,
    ));

    match state.auth_service.login(&login_req.password, ip_address) {
        Ok(Some((token, csrf_token))) => {
            // Session cookie: HttpOnly so JavaScript cannot read it (mitigates
            // XSS-based token theft).  Max-Age matches the absolute session
            // lifetime (8 hours).
            let cookie = format!(
                "session_token={}; Path=/; HttpOnly; SameSite=Strict; Max-Age=28800",
                token
            );

            (
                StatusCode::OK,
                [(axum::http::header::SET_COOKIE, cookie)],
                Json(LoginResponse {
                    success: true,
                    session_token: Some(token),
                    csrf_token: Some(csrf_token),
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
                    csrf_token: None,
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
                    csrf_token: None,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extract_client_ip_no_proxy_ignores_xff() {
        let mut headers = HeaderMap::new();
        headers.insert("x-forwarded-for", "10.0.0.1".parse().unwrap());
        let ip = extract_client_ip(None, &headers, false);
        assert_eq!(ip, "127.0.0.1", "should ignore XFF when trust_proxy=false");
    }

    #[test]
    fn extract_client_ip_trust_proxy_uses_xff() {
        let mut headers = HeaderMap::new();
        headers.insert("x-forwarded-for", "10.0.0.1, 192.168.1.1".parse().unwrap());
        let ip = extract_client_ip(None, &headers, true);
        assert_eq!(ip, "10.0.0.1", "should use first XFF entry");
    }

    #[test]
    fn extract_client_ip_trust_proxy_uses_x_real_ip() {
        let mut headers = HeaderMap::new();
        headers.insert("x-real-ip", "172.16.0.5".parse().unwrap());
        let ip = extract_client_ip(None, &headers, true);
        assert_eq!(ip, "172.16.0.5");
    }

    #[test]
    fn extract_client_ip_xff_takes_priority_over_x_real_ip() {
        let mut headers = HeaderMap::new();
        headers.insert("x-forwarded-for", "10.0.0.1".parse().unwrap());
        headers.insert("x-real-ip", "172.16.0.5".parse().unwrap());
        let ip = extract_client_ip(None, &headers, true);
        assert_eq!(ip, "10.0.0.1", "XFF should take priority over X-Real-IP");
    }

    #[test]
    fn extract_client_ip_falls_back_to_peer_addr() {
        let headers = HeaderMap::new();
        let peer: SocketAddr = "192.168.1.100:12345".parse().unwrap();
        let ip = extract_client_ip(Some(peer), &headers, false);
        assert_eq!(ip, "192.168.1.100");
    }

    #[test]
    fn extract_client_ip_no_info_returns_localhost() {
        let ip = extract_client_ip(None, &HeaderMap::new(), false);
        assert_eq!(ip, "127.0.0.1");
    }
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
