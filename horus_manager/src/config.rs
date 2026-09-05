//! Centralized configuration for horus_manager.
//!
//! All infrastructure URLs and defaults live here. Each can be overridden
//! by environment variables, enabling self-hosted registry and offline use.

/// Default HORUS registry API URL, or `None` when no public registry is running.
///
/// This is `None` today. The hosted registry at `api.horusrobotics.dev` is
/// suspended and answers every request with HTTP 503, and
/// `plugins.horusrobotics.dev` has no DNS record at all. Shipping those URLs as
/// compiled-in defaults did not degrade gracefully — it made an outage
/// indistinguishable from an empty ecosystem, because a failed search was
/// discarded and reported as "no results" with exit code 0. Absent is honest;
/// a dead URL is not.
///
/// Self-hosted registries are unaffected: set `HORUS_REGISTRY_URL` and every
/// registry command works exactly as before. Restoring a public registry is a
/// one-line change here — put the URL back in the `Some(..)` and nothing else
/// needs to move.
///
/// Tracked in <https://github.com/softmata/horus/issues/173>.
pub const DEFAULT_REGISTRY_URL: Option<&str> = None;

/// Default plugin registry URL, or `None` when no public registry is running.
///
/// `None` for the same reason as [`DEFAULT_REGISTRY_URL`]; override with
/// `HORUS_PLUGIN_REGISTRY_URL`.
pub const DEFAULT_PLUGIN_REGISTRY_URL: Option<&str> = None;

/// What every registry-backed command prints when no registry is configured.
///
/// One string so the CLI cannot drift into saying different things in different
/// places about the same condition.
pub const NO_REGISTRY_CONFIGURED: &str = "\
no package registry is configured.
  The public HORUS registry is not running yet (tracking:
  https://github.com/softmata/horus/issues/173).

  Packages still work without it:
    - `path` and `git` dependencies in horus.toml are unaffected
    - set HORUS_REGISTRY_URL=<url> to use a self-hosted registry";

/// PyPI JSON API base URL.
pub const PYPI_API_URL: &str = "https://pypi.org/pypi";

/// Crates.io API base URL.
pub const CRATES_IO_API_URL: &str = "https://crates.io/api/v1/crates";

/// The registry URL from the environment, else the compiled-in default.
///
/// `None` means no registry is configured and registry-backed commands must say
/// so rather than guess. An env var set to whitespace counts as unset: an empty
/// base URL would otherwise build request paths like `/api/packages/search` and
/// fail as a confusing relative-URL parse error rather than as "not configured".
pub fn registry_url() -> Option<String> {
    resolve_registry_url("HORUS_REGISTRY_URL", DEFAULT_REGISTRY_URL)
}

/// The plugin registry URL from the environment, else the compiled-in default.
pub fn plugin_registry_url() -> Option<String> {
    resolve_registry_url("HORUS_PLUGIN_REGISTRY_URL", DEFAULT_PLUGIN_REGISTRY_URL)
}

/// Resolve a registry base URL, normalised so callers can join paths freely.
///
/// The trailing slash is stripped HERE rather than at each use. Every caller
/// builds paths as `format!("{base}/api/...")`, and of roughly two dozen such
/// sites exactly two remembered to `trim_end_matches('/')` — so a
/// `HORUS_REGISTRY_URL` ending in `/` produced `//api/...` almost everywhere
/// and the correct URL in two places. Normalising once means no site has to
/// remember, and the two that did can stop.
///
/// `RegistryClient::with_base_url` deliberately stores whatever it is handed;
/// that contract is unchanged. This is the layer that decides what to hand it.
fn resolve_registry_url(var: &str, fallback: Option<&str>) -> Option<String> {
    fn normalise(url: &str) -> String {
        url.trim().trim_end_matches('/').to_string()
    }
    match std::env::var(var) {
        Ok(url) if !url.trim().is_empty() => Some(normalise(&url)),
        _ => fallback.map(normalise),
    }
}

// === Security Constants ===

/// Maximum login attempts before rate limiting kicks in.
pub const AUTH_MAX_ATTEMPTS: usize = 5;

/// Minimum password length for the monitor.
///
/// Passwords shorter than this are rejected at setup/reset time.
/// 12 characters is the NIST SP 800-63B recommended minimum for user-chosen
/// passwords without additional complexity requirements.
pub const MIN_PASSWORD_LENGTH: usize = 12;

/// Rate limit window duration in seconds.
pub const AUTH_RATE_LIMIT_WINDOW_SECS: u64 = 60;

/// Session inactivity timeout in seconds (1 hour).
pub const SESSION_TIMEOUT_SECS: u64 = 3600;

/// Absolute session lifetime in seconds (8 hours), regardless of activity.
///
/// An attacker who captures a session token cannot extend access beyond this
/// window by simply replaying requests; the session is unconditionally revoked
/// after 8 hours even if it has been continuously active.
pub const SESSION_ABSOLUTE_TIMEOUT_SECS: u64 = 8 * 3600; // 28_800

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

/// Cargo manifest file name.
pub const CARGO_TOML: &str = "Cargo.toml";

// === Reverse Proxy Configuration ===

/// Whether to trust reverse-proxy IP headers (X-Forwarded-For, X-Real-IP).
///
/// Set `HORUS_TRUST_PROXY=1` (or `true` / `yes`) when horus-monitor is
/// deployed behind a reverse proxy that rewrites forwarded headers.
/// When disabled (the default), those headers are ignored so a remote client
/// cannot spoof an arbitrary IP address to bypass per-IP rate limiting.
pub fn trust_proxy() -> bool {
    matches!(
        std::env::var("HORUS_TRUST_PROXY").as_deref(),
        Ok("1") | Ok("true") | Ok("yes")
    )
}

#[cfg(test)]
#[allow(clippy::assertions_on_constants)]
mod tests {
    use super::*;

    // ── Constants validation ─────────────────────────────────────────

    // Both defaults are `None` while the public registry is down (#173). These
    // stay as conditionals rather than being deleted, so that whoever restores
    // a URL still cannot land a plaintext one.

    #[test]
    fn default_registry_url_is_https_if_set() {
        if let Some(url) = DEFAULT_REGISTRY_URL {
            assert!(
                url.starts_with("https://"),
                "Registry URL must use HTTPS, got {url}"
            );
        }
    }

    #[test]
    fn default_plugin_registry_url_is_https_if_set() {
        if let Some(url) = DEFAULT_PLUGIN_REGISTRY_URL {
            assert!(
                url.starts_with("https://"),
                "Plugin registry URL must use HTTPS, got {url}"
            );
        }
    }

    // ── Registry resolution (#173) ───────────────────────────────────
    //
    // These drive `resolve_registry_url` directly rather than
    // `registry_url()`. The public wrappers read fixed env var names, and
    // libtest runs the module's tests in one process on many threads, so two
    // tests touching HORUS_REGISTRY_URL race each other. Passing a unique var
    // name per test removes the shared state instead of trying to lock it.

    #[test]
    fn the_public_registry_default_is_absent() {
        // Deliberate, not an oversight: `api.horusrobotics.dev` is suspended and
        // answers 503, and `plugins.horusrobotics.dev` has no DNS record. A
        // default pointing at either one turns an outage into a silent "no
        // results". Restoring one is the intended way to close #173 -- this test
        // is what makes that a conscious edit rather than a drive-by.
        assert_eq!(DEFAULT_REGISTRY_URL, None);
        assert_eq!(DEFAULT_PLUGIN_REGISTRY_URL, None);
    }

    #[test]
    fn resolve_returns_none_when_unset_and_no_default() {
        std::env::remove_var("HORUS_TEST_REG_UNSET");
        assert_eq!(resolve_registry_url("HORUS_TEST_REG_UNSET", None), None);
    }

    #[test]
    fn resolve_falls_back_to_the_compiled_default() {
        std::env::remove_var("HORUS_TEST_REG_FALLBACK");
        assert_eq!(
            resolve_registry_url("HORUS_TEST_REG_FALLBACK", Some("https://d.example")),
            Some("https://d.example".to_string())
        );
    }

    #[test]
    fn env_override_wins_over_the_compiled_default() {
        std::env::set_var("HORUS_TEST_REG_OVERRIDE", "https://mine.example");
        assert_eq!(
            resolve_registry_url("HORUS_TEST_REG_OVERRIDE", Some("https://d.example")),
            Some("https://mine.example".to_string())
        );
    }

    #[test]
    fn a_blank_env_override_is_treated_as_unset() {
        // An empty base URL is the dangerous case: it does not fail loudly, it
        // builds request paths like "/api/packages/search" that fail later as an
        // opaque relative-URL parse error. Blank means "not configured".
        for blank in ["", "   ", "\t", "\n"] {
            std::env::set_var("HORUS_TEST_REG_BLANK", blank);
            assert_eq!(
                resolve_registry_url("HORUS_TEST_REG_BLANK", None),
                None,
                "{blank:?} should count as unset"
            );
            assert_eq!(
                resolve_registry_url("HORUS_TEST_REG_BLANK", Some("https://d.example")),
                Some("https://d.example".to_string()),
                "{blank:?} should fall through to the default"
            );
        }
        std::env::remove_var("HORUS_TEST_REG_BLANK");
    }

    #[test]
    fn surrounding_whitespace_is_trimmed_from_an_override() {
        // A trailing newline is what a `HORUS_REGISTRY_URL=$(cat url.txt)` gives
        // you, and it would otherwise be pasted straight into the request URL.
        std::env::set_var("HORUS_TEST_REG_TRIM", "  https://mine.example\n");
        assert_eq!(
            resolve_registry_url("HORUS_TEST_REG_TRIM", None),
            Some("https://mine.example".to_string())
        );
        std::env::remove_var("HORUS_TEST_REG_TRIM");
    }

    #[test]
    fn the_no_registry_message_stays_actionable() {
        // The whole point of this message is that a user who hits it knows what
        // to do next and where to look. If someone trims it down to "registry
        // unavailable", this fails.
        let msg = NO_REGISTRY_CONFIGURED;
        assert!(
            msg.contains("HORUS_REGISTRY_URL"),
            "must name the escape hatch: {msg}"
        );
        assert!(
            msg.contains("issues/173"),
            "must link the tracking issue: {msg}"
        );
        assert!(
            msg.contains("path") && msg.contains("git"),
            "must say what still works without a registry: {msg}"
        );
    }

    #[test]
    fn pypi_api_url_is_valid() {
        assert!(PYPI_API_URL.starts_with("https://"));
        assert!(PYPI_API_URL.contains("pypi.org"));
    }

    #[test]
    fn crates_io_api_url_is_valid() {
        assert!(CRATES_IO_API_URL.starts_with("https://"));
        assert!(CRATES_IO_API_URL.contains("crates.io"));
    }

    // ── Security constants ───────────────────────────────────────────

    #[test]
    fn auth_max_attempts_is_reasonable() {
        assert!(
            AUTH_MAX_ATTEMPTS >= 3 && AUTH_MAX_ATTEMPTS <= 10,
            "Auth max attempts should be 3-10, got {}",
            AUTH_MAX_ATTEMPTS
        );
    }

    #[test]
    fn min_password_length_meets_nist_minimum() {
        // NIST SP 800-63B recommends at least 8 characters
        assert!(
            MIN_PASSWORD_LENGTH >= 8,
            "Password minimum should be at least 8 (NIST), got {}",
            MIN_PASSWORD_LENGTH
        );
    }

    #[test]
    fn rate_limit_window_is_reasonable() {
        assert!(
            AUTH_RATE_LIMIT_WINDOW_SECS > 0 && AUTH_RATE_LIMIT_WINDOW_SECS <= 300,
            "Rate limit window should be 1-300s, got {}",
            AUTH_RATE_LIMIT_WINDOW_SECS
        );
    }

    #[test]
    fn session_timeout_is_reasonable() {
        // Session should expire between 15 minutes and 24 hours
        assert!(SESSION_TIMEOUT_SECS >= 900, "Session timeout too short");
        assert!(SESSION_TIMEOUT_SECS <= 86400, "Session timeout too long");
    }

    #[test]
    fn session_absolute_timeout_exceeds_inactivity_timeout() {
        assert!(
            SESSION_ABSOLUTE_TIMEOUT_SECS > SESSION_TIMEOUT_SECS,
            "Absolute timeout must be longer than inactivity timeout"
        );
    }

    // ── Cache & monitoring constants ─────────────────────────────────

    #[test]
    fn workspace_cache_ttl_is_reasonable() {
        assert!(WORKSPACE_CACHE_TTL_SECS > 0);
        assert!(WORKSPACE_CACHE_TTL_SECS <= 3600);
    }

    #[test]
    fn discovery_cache_is_sub_second() {
        assert!(DISCOVERY_CACHE_MS < 1000, "Discovery cache should be < 1s");
    }

    #[test]
    fn tui_intervals_are_reasonable() {
        assert!(TUI_POLL_INTERVAL_MS > 0 && TUI_POLL_INTERVAL_MS <= 1000);
        assert!(TUI_REFRESH_INTERVAL_MS > 0 && TUI_REFRESH_INTERVAL_MS <= 1000);
    }

    #[test]
    fn ws_broadcast_interval_is_reasonable() {
        assert!(WS_BROADCAST_INTERVAL_MS > 0 && WS_BROADCAST_INTERVAL_MS <= 5000);
    }

    // ── File name constants ──────────────────────────────────────────

    #[test]
    fn cargo_toml_filename_correct() {
        assert_eq!(CARGO_TOML, "Cargo.toml");
    }

    // ── Environment variable functions ───────────────────────────────

    #[test]
    fn registry_url_returns_default_when_unset() {
        // Temporarily clear env var if set
        let original = std::env::var("HORUS_REGISTRY_URL").ok();
        std::env::remove_var("HORUS_REGISTRY_URL");

        let url = registry_url();
        assert_eq!(url.as_deref(), DEFAULT_REGISTRY_URL);

        // Restore if was set
        if let Some(val) = original {
            std::env::set_var("HORUS_REGISTRY_URL", val);
        }
    }

    #[test]
    fn plugin_registry_url_returns_default_when_unset() {
        let original = std::env::var("HORUS_PLUGIN_REGISTRY_URL").ok();
        std::env::remove_var("HORUS_PLUGIN_REGISTRY_URL");

        let url = plugin_registry_url();
        assert_eq!(url.as_deref(), DEFAULT_PLUGIN_REGISTRY_URL);

        if let Some(val) = original {
            std::env::set_var("HORUS_PLUGIN_REGISTRY_URL", val);
        }
    }

    #[test]
    fn trust_proxy_defaults_to_false() {
        let original = std::env::var("HORUS_TRUST_PROXY").ok();
        std::env::remove_var("HORUS_TRUST_PROXY");

        assert!(!trust_proxy(), "trust_proxy should default to false");

        if let Some(val) = original {
            std::env::set_var("HORUS_TRUST_PROXY", val);
        }
    }

    // ── Config default values ───────────────────────────────────────────

    #[test]
    fn test_config_default_values() {
        // A configured default must be HTTPS and non-empty. `None` -- the
        // current state while the public registry is down -- is also valid;
        // what is NOT valid is a default that is present but unusable.
        for (label, url) in [
            ("registry", DEFAULT_REGISTRY_URL),
            ("plugin registry", DEFAULT_PLUGIN_REGISTRY_URL),
        ] {
            if let Some(url) = url {
                assert!(
                    url.starts_with("https://"),
                    "default {label} URL should use HTTPS"
                );
                assert!(!url.is_empty(), "default {label} URL should not be empty");
            }
        }

        // Security defaults are reasonable
        assert!(
            AUTH_MAX_ATTEMPTS >= 3,
            "Max auth attempts should be at least 3"
        );
        assert!(
            MIN_PASSWORD_LENGTH >= 8,
            "Min password length should meet NIST minimum of 8"
        );
        assert!(
            SESSION_TIMEOUT_SECS >= 60,
            "Session timeout should be at least 1 minute"
        );

        // Cache TTL is set and positive
        assert!(
            WORKSPACE_CACHE_TTL_SECS > 0,
            "Workspace cache TTL should be positive"
        );

        // TUI intervals are non-zero
        assert!(
            TUI_POLL_INTERVAL_MS > 0,
            "TUI poll interval should be positive"
        );
        assert!(
            TUI_REFRESH_INTERVAL_MS > 0,
            "TUI refresh interval should be positive"
        );
    }

    // ── Config set/get roundtrip ────────────────────────────────────────

    #[test]
    fn test_config_set_get_roundtrip() {
        // Set registry URL via env var, then read it back
        let original = std::env::var("HORUS_REGISTRY_URL").ok();
        let test_url = "https://custom.registry.example.com";
        std::env::set_var("HORUS_REGISTRY_URL", test_url);

        let url = registry_url();
        assert_eq!(
            url.as_deref(),
            Some(test_url),
            "registry_url() should return env override"
        );

        // Set plugin registry URL via env var, then read it back
        let original_plugin = std::env::var("HORUS_PLUGIN_REGISTRY_URL").ok();
        let test_plugin_url = "https://custom.plugins.example.com/api/v2";
        std::env::set_var("HORUS_PLUGIN_REGISTRY_URL", test_plugin_url);

        let plugin_url = plugin_registry_url();
        assert_eq!(
            plugin_url.as_deref(),
            Some(test_plugin_url),
            "plugin_registry_url() should return env override"
        );

        // Set trust_proxy via env var, then read it back
        let original_proxy = std::env::var("HORUS_TRUST_PROXY").ok();
        std::env::set_var("HORUS_TRUST_PROXY", "1");
        assert!(
            trust_proxy(),
            "trust_proxy should return true when set to '1'"
        );

        std::env::set_var("HORUS_TRUST_PROXY", "true");
        assert!(
            trust_proxy(),
            "trust_proxy should return true when set to 'true'"
        );

        std::env::set_var("HORUS_TRUST_PROXY", "yes");
        assert!(
            trust_proxy(),
            "trust_proxy should return true when set to 'yes'"
        );

        std::env::set_var("HORUS_TRUST_PROXY", "0");
        assert!(
            !trust_proxy(),
            "trust_proxy should return false when set to '0'"
        );

        // Restore original values
        match original {
            Some(val) => std::env::set_var("HORUS_REGISTRY_URL", val),
            None => std::env::remove_var("HORUS_REGISTRY_URL"),
        }
        match original_plugin {
            Some(val) => std::env::set_var("HORUS_PLUGIN_REGISTRY_URL", val),
            None => std::env::remove_var("HORUS_PLUGIN_REGISTRY_URL"),
        }
        match original_proxy {
            Some(val) => std::env::set_var("HORUS_TRUST_PROXY", val),
            None => std::env::remove_var("HORUS_TRUST_PROXY"),
        }
    }

    // ── Config save/load roundtrip ──────────────────────────────────────
    //
    // config.rs exposes constants and env-var-backed functions, not a
    // serializable struct. We prove the roundtrip by writing the values
    // to a TOML file, reading them back, and asserting equality.

    #[test]
    fn test_config_save_load_roundtrip() {
        use std::io::Write;

        let tmp = tempfile::tempdir().unwrap();
        let path = tmp.path().join("config_test.toml");

        // "Save" the current config values as TOML
        let contents = format!(
            "[urls]\nregistry = \"{}\"\nplugin_registry = \"{}\"\npypi = \"{}\"\ncrates_io = \"{}\"\n\n[security]\nmax_attempts = {}\nmin_password_length = {}\nrate_limit_window_secs = {}\nsession_timeout_secs = {}\nsession_absolute_timeout_secs = {}\n",
            DEFAULT_REGISTRY_URL.unwrap_or(""),
            DEFAULT_PLUGIN_REGISTRY_URL.unwrap_or(""),
            PYPI_API_URL,
            CRATES_IO_API_URL,
            AUTH_MAX_ATTEMPTS,
            MIN_PASSWORD_LENGTH,
            AUTH_RATE_LIMIT_WINDOW_SECS,
            SESSION_TIMEOUT_SECS,
            SESSION_ABSOLUTE_TIMEOUT_SECS,
        );
        let mut file = std::fs::File::create(&path).unwrap();
        file.write_all(contents.as_bytes()).unwrap();

        // "Load" it back
        let loaded = std::fs::read_to_string(&path).unwrap();
        // toml 1.1: `FromStr for Value` parses a value, not a document.
        let parsed: toml::Value = toml::from_str(&loaded).unwrap();

        // Assert all fields match
        let urls = parsed.get("urls").unwrap();
        assert_eq!(
            urls.get("registry").unwrap().as_str().unwrap(),
            DEFAULT_REGISTRY_URL.unwrap_or("")
        );
        assert_eq!(
            urls.get("plugin_registry").unwrap().as_str().unwrap(),
            DEFAULT_PLUGIN_REGISTRY_URL.unwrap_or("")
        );
        assert_eq!(urls.get("pypi").unwrap().as_str().unwrap(), PYPI_API_URL);
        assert_eq!(
            urls.get("crates_io").unwrap().as_str().unwrap(),
            CRATES_IO_API_URL
        );

        let security = parsed.get("security").unwrap();
        assert_eq!(
            security.get("max_attempts").unwrap().as_integer().unwrap() as usize,
            AUTH_MAX_ATTEMPTS
        );
        assert_eq!(
            security
                .get("min_password_length")
                .unwrap()
                .as_integer()
                .unwrap() as usize,
            MIN_PASSWORD_LENGTH
        );
        assert_eq!(
            security
                .get("rate_limit_window_secs")
                .unwrap()
                .as_integer()
                .unwrap() as u64,
            AUTH_RATE_LIMIT_WINDOW_SECS
        );
        assert_eq!(
            security
                .get("session_timeout_secs")
                .unwrap()
                .as_integer()
                .unwrap() as u64,
            SESSION_TIMEOUT_SECS
        );
        assert_eq!(
            security
                .get("session_absolute_timeout_secs")
                .unwrap()
                .as_integer()
                .unwrap() as u64,
            SESSION_ABSOLUTE_TIMEOUT_SECS
        );
    }
}
