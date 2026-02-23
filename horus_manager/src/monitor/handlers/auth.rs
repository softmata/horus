use axum::{extract::State, http::StatusCode, response::IntoResponse, Json};
use std::sync::Arc;

use crate::monitor::AppState;

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
