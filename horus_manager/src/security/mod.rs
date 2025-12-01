//! Security module for HORUS dashboard
//!
//! Provides password-based authentication and security middleware.

pub mod auth;
pub mod middleware;

pub use auth::AuthService;
pub use middleware::{security_headers_middleware, session_middleware};
