//! Centralized configuration for horus_manager.
//!
//! All infrastructure URLs and defaults live here. Each can be overridden
//! by environment variables, enabling self-hosted registry and offline use.

/// Default HORUS registry API URL.
/// Override with `HORUS_REGISTRY_URL` environment variable.
pub const DEFAULT_REGISTRY_URL: &str = "https://horus-marketplace-api.onrender.com";

/// Default HORUS cloud URL.
/// Override with `HORUS_CLOUD_URL` environment variable.
pub const DEFAULT_CLOUD_URL: &str = "https://cloud.horus.dev";

/// Default plugin registry URL.
/// Override with `HORUS_PLUGIN_REGISTRY_URL` environment variable.
pub const DEFAULT_PLUGIN_REGISTRY_URL: &str = "https://registry.softmata.com/api/v1";

/// PyPI JSON API base URL.
pub const PYPI_API_URL: &str = "https://pypi.org/pypi";

/// Crates.io API base URL.
pub const CRATES_IO_API_URL: &str = "https://crates.io/api/v1/crates";

/// Get the registry URL from env var or default.
pub fn registry_url() -> String {
    std::env::var("HORUS_REGISTRY_URL").unwrap_or_else(|_| DEFAULT_REGISTRY_URL.to_string())
}

/// Get the cloud URL from env var or default.
pub fn cloud_url() -> String {
    std::env::var("HORUS_CLOUD_URL").unwrap_or_else(|_| DEFAULT_CLOUD_URL.to_string())
}

/// Get the plugin registry URL from env var or default.
pub fn plugin_registry_url() -> String {
    std::env::var("HORUS_PLUGIN_REGISTRY_URL")
        .unwrap_or_else(|_| DEFAULT_PLUGIN_REGISTRY_URL.to_string())
}

// === Security Constants ===

/// Maximum login attempts before rate limiting kicks in.
pub const AUTH_MAX_ATTEMPTS: usize = 5;

/// Rate limit window duration in seconds.
pub const AUTH_RATE_LIMIT_WINDOW_SECS: u64 = 60;

/// Session inactivity timeout in seconds (1 hour).
pub const SESSION_TIMEOUT_SECS: u64 = 3600;

// === Cache & Monitoring Constants ===

/// Workspace discovery cache TTL in seconds (5 minutes).
pub const WORKSPACE_CACHE_TTL_SECS: u64 = 300;

/// Discovery cache duration in milliseconds for real-time updates.
pub const DISCOVERY_CACHE_MS: u64 = 250;

/// Default poll interval for TUI event loop in milliseconds.
pub const TUI_POLL_INTERVAL_MS: u64 = 100;

/// Default UI refresh interval in milliseconds.
pub const TUI_REFRESH_INTERVAL_MS: u64 = 250;

/// WebSocket broadcast interval in milliseconds.
pub const WS_BROADCAST_INTERVAL_MS: u64 = 250;

// === File Name Constants ===

/// HORUS workspace manifest file name.
pub const HORUS_YAML: &str = "horus.yaml";

/// Cargo manifest file name.
pub const CARGO_TOML: &str = "Cargo.toml";

/// Environment freeze file name.
pub const HORUS_FREEZE_YAML: &str = "horus-freeze.yaml";

/// Shared memory path prefix on Linux.
pub const SHM_PATH: &str = "/dev/shm";

/// HORUS shared memory topic prefix.
pub const SHM_HORUS_PREFIX: &str = "/dev/shm/horus_";
