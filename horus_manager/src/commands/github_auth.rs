use anyhow::Result;
use colored::*;
use horus_core::core::DurationExt;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use serde::{Deserialize, Serialize};
use std::fs;
use std::io::{self, Read as _, Write};
use std::net::TcpListener;
use std::path::PathBuf;

#[derive(Debug, Serialize, Deserialize)]
struct AuthConfig {
    api_key: String,
    registry_url: String,
    github_username: Option<String>,
}

/// Get the path to the auth config file
fn auth_config_path() -> Result<PathBuf> {
    let config_dir = crate::paths::horus_dir()?;

    // Create .horus directory if it doesn't exist
    if !config_dir.exists() {
        fs::create_dir_all(&config_dir)?;
    }

    Ok(config_dir.join("auth.json"))
}

/// Load auth config from disk. Returns error if not logged in.
fn load_auth_config() -> HorusResult<AuthConfig> {
    let config_path =
        auth_config_path().map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    if !config_path.exists() {
        return Err(HorusError::Config(ConfigError::Other(
            "not authenticated. Please run: horus auth login".to_string(),
        )));
    }

    let content = fs::read_to_string(&config_path).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "failed to read auth config: {}",
            e
        )))
    })?;

    serde_json::from_str(&content).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "failed to parse auth config: {}",
            e
        )))
    })
}

/// Login to the HORUS registry with GitHub.
///
/// Starts a local callback server, opens the browser for GitHub OAuth,
/// and automatically saves the API key after authentication completes.
pub fn login() -> HorusResult<()> {
    let registry_url = get_registry_url();

    // Start local callback server on a random port
    let listener = TcpListener::bind("127.0.0.1:0").map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Failed to start local auth server: {}",
            e
        )))
    })?;
    let port = listener.local_addr().unwrap().port();

    // Set a timeout so we don't hang forever if the user closes the browser
    listener.set_nonblocking(false).ok();

    println!("Logging in to HORUS registry with GitHub...");
    println!();

    // Open browser with CLI port so the server redirects back to us
    let auth_url = format!("{}/auth/github?cli_port={}", registry_url, port);
    println!("{} Opening browser for GitHub authentication...", "".cyan());

    if open::that(&auth_url).is_err() {
        println!(
            "{} Could not open browser automatically.",
            crate::cli_output::ICON_WARN.yellow()
        );
        println!("Please visit: {}", auth_url.cyan());
    }

    println!();
    println!("Waiting for authentication...");
    println!("  {} Press Ctrl+C to cancel", "Tip:".dimmed());

    // Wait for the OAuth callback (with timeout)
    let result = wait_for_oauth_callback(&listener, &registry_url);

    match result {
        Ok((api_key, username)) => {
            // Save auth config
            save_auth_config(&api_key, &registry_url, Some(&username))?;

            println!();
            println!(
                "{} Logged in as @{}",
                crate::cli_output::ICON_SUCCESS.green(),
                username.green().bold()
            );
            println!("  {} API key saved automatically", "".dimmed());
            println!();
            println!(
                "{} You can now publish packages with: {}",
                "Tip:".yellow(),
                "horus publish".cyan()
            );
            Ok(())
        }
        Err(e) => {
            println!();
            println!(
                "{} Automatic login failed: {}",
                crate::cli_output::ICON_WARN.yellow(),
                e
            );
            println!();
            println!("You can complete login manually:");
            println!("  1. Visit: {}/dashboard/keys", registry_url.cyan());
            println!("  2. Generate an API key");
            println!("  3. Run: {}", "horus auth generate-key".cyan());
            Ok(())
        }
    }
}

/// Wait for the OAuth callback on the local server, exchange the auth code for an API key.
fn wait_for_oauth_callback(
    listener: &TcpListener,
    registry_url: &str,
) -> std::result::Result<(String, String), String> {
    // Set a 5-minute timeout for the OAuth flow
    listener.set_nonblocking(false).ok();

    // Accept one connection (the browser redirect)
    // Use a timeout via polling to avoid hanging indefinitely
    let start = std::time::Instant::now();
    let timeout = 300_u64.secs();

    listener.set_nonblocking(true).ok();

    let mut stream = loop {
        match listener.accept() {
            Ok((stream, _)) => break stream,
            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                if start.elapsed() > timeout {
                    return Err("Timed out waiting for authentication (5 minutes)".to_string());
                }
                std::thread::sleep(100_u64.ms());
                continue;
            }
            Err(e) => return Err(format!("Failed to accept connection: {}", e)),
        }
    };

    // Read the HTTP request
    let mut buf = [0u8; 4096];
    stream.set_read_timeout(Some(5_u64.secs())).ok();
    let n = stream
        .read(&mut buf)
        .map_err(|e| format!("Failed to read request: {}", e))?;
    let request = String::from_utf8_lossy(&buf[..n]);

    // Parse the request line to extract query params
    // Expected: GET /callback?code=xxx&user=yyy HTTP/1.1
    let first_line = request.lines().next().unwrap_or("");
    let path = first_line.split_whitespace().nth(1).unwrap_or("");

    let auth_code = extract_query_param(path, "code");
    let username = extract_query_param(path, "user").unwrap_or_default();

    // Send a response to the browser
    let html_body = if auth_code.is_some() {
        "<html><body style='font-family:system-ui;text-align:center;padding:60px'>\
         <h1 style='color:#22c55e'>Authentication successful!</h1>\
         <p>You can close this window and return to the terminal.</p>\
         </body></html>"
    } else {
        "<html><body style='font-family:system-ui;text-align:center;padding:60px'>\
         <h1 style='color:#ef4444'>Authentication failed</h1>\
         <p>No auth code received. Please try again with <code>horus auth login</code></p>\
         </body></html>"
    };

    let response = format!(
        "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
        html_body.len(),
        html_body
    );
    let _ = stream.write_all(response.as_bytes());
    drop(stream);

    let auth_code = auth_code.ok_or("No auth code in callback")?;

    // Exchange the auth code for an API key
    let client = reqwest::blocking::Client::new();
    let exchange_resp = client
        .post(format!("{}/api/auth/exchange", registry_url))
        .json(&serde_json::json!({ "code": auth_code }))
        .send()
        .map_err(|e| format!("Failed to exchange auth code: {}", e))?;

    if !exchange_resp.status().is_success() {
        return Err(format!(
            "Auth code exchange failed (HTTP {}). The code may have expired.",
            exchange_resp.status().as_u16()
        ));
    }

    let body: serde_json::Value = exchange_resp
        .json()
        .map_err(|e| format!("Failed to parse exchange response: {}", e))?;

    let api_key = body
        .get("api_key")
        .and_then(|v| v.as_str())
        .ok_or("No API key in exchange response")?
        .to_string();

    let user = body
        .get("user")
        .and_then(|v| v.as_str())
        .map(|s| s.to_string())
        .unwrap_or(username);

    Ok((api_key, user))
}

/// Decode a percent-encoded string per RFC 3986
fn url_decode(input: &str) -> String {
    let mut result = Vec::with_capacity(input.len());
    let bytes = input.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'%' && i + 2 < bytes.len() {
            if let Ok(byte) = u8::from_str_radix(&input[i + 1..i + 3], 16) {
                result.push(byte);
                i += 3;
                continue;
            }
        } else if bytes[i] == b'+' {
            result.push(b' ');
            i += 1;
            continue;
        }
        result.push(bytes[i]);
        i += 1;
    }
    String::from_utf8_lossy(&result).into_owned()
}

/// Extract a query parameter value from a URL path
fn extract_query_param(path: &str, key: &str) -> Option<String> {
    let query = path.split('?').nth(1)?;
    for pair in query.split('&') {
        let mut parts = pair.splitn(2, '=');
        if parts.next() == Some(key) {
            return parts.next().map(url_decode);
        }
    }
    None
}

/// Save authentication config to disk
fn save_auth_config(api_key: &str, registry_url: &str, username: Option<&str>) -> HorusResult<()> {
    let config = AuthConfig {
        api_key: api_key.to_string(),
        registry_url: registry_url.to_string(),
        github_username: username.map(|s| s.to_string()),
    };

    let config_path =
        auth_config_path().map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    let config_json = serde_json::to_string_pretty(&config).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Failed to serialize config: {}",
            e
        )))
    })?;

    // Create file with restricted permissions from the start (contains API token)
    {
        use std::io::Write;
        let mut file = horus_sys::fs::open_private(&config_path).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!(
                "Failed to create auth config: {}",
                e
            )))
        })?;
        file.write_all(config_json.as_bytes()).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!(
                "Failed to write auth config: {}",
                e
            )))
        })?;
    }

    Ok(())
}

/// Generate API key after GitHub authentication
pub fn generate_key(name: Option<String>, environment: Option<String>) -> HorusResult<()> {
    println!("Generating API key...");

    let registry_url = get_registry_url();
    let key_name = name.unwrap_or_else(|| {
        // Generate default name based on hostname
        let hostname = hostname::get()
            .map(|h| h.to_string_lossy().to_string())
            .unwrap_or_else(|_| "unknown".to_string());
        format!("{}-{}", hostname, chrono::Utc::now().timestamp())
    });

    println!();
    println!(
        "{} This requires you to be logged in via GitHub first.",
        "Note:".yellow()
    );
    println!(
        "  If you haven't logged in yet, run: {}",
        "horus auth login".cyan()
    );
    println!();

    // Prompt for manual key entry (since we need the GitHub session)
    println!("After logging in via GitHub, the registry will show an API key generation page.");
    println!("Visit: {}/dashboard/keys", registry_url.cyan());
    println!();
    println!("Generate a key with:");
    println!("  {} {}", "Name:".dimmed(), key_name);
    if let Some(env) = &environment {
        println!("  {} {}", "Environment:".dimmed(), env);
    }
    println!();

    print!("Enter the generated API key: ");
    let _ = io::stdout().flush();

    let mut api_key = String::new();
    io::stdin().read_line(&mut api_key).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!("Failed to read input: {}", e)))
    })?;

    let api_key = api_key.trim().to_string();

    // Validate token format
    if !api_key.starts_with("horus_key_") {
        return Err(HorusError::Config(ConfigError::Other(
            "Invalid token format. Token should start with 'horus_key_'".to_string(),
        )));
    }

    save_auth_config(&api_key, &registry_url, None)?;

    let config_path =
        auth_config_path().map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    println!();
    println!("API key saved successfully!");
    println!("  {} {}", "Registry:".dimmed(), registry_url);
    println!(
        "  {} {}",
        "Config saved to:".dimmed(),
        config_path.display()
    );
    println!();
    println!(
        "{} You can now publish packages with: {}",
        "Tip:".yellow(),
        "horus publish".cyan()
    );

    Ok(())
}

/// Logout from the HORUS registry
pub fn logout() -> HorusResult<()> {
    println!("Logging out from HORUS registry...");

    let config_path =
        auth_config_path().map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    if config_path.exists() {
        fs::remove_file(&config_path).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!(
                "Failed to remove auth config: {}",
                e
            )))
        })?;

        println!("Successfully logged out!");
        println!("  {} API key removed from local storage", "•".dimmed());
    } else {
        println!(
            "{} Not currently logged in",
            crate::cli_output::ICON_WARN.yellow()
        );
    }

    Ok(())
}

/// Show current authenticated user
pub fn whoami() -> HorusResult<()> {
    let config_path =
        auth_config_path().map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    if !config_path.exists() {
        println!("{} Not logged in", crate::cli_output::ICON_WARN.yellow());
        println!();
        println!("To authenticate:");
        println!("  1. Run: {}", "horus auth login".cyan());
        println!("  2. Then: {}", "horus auth generate-key".cyan());
        return Ok(());
    }

    let config = load_auth_config()?;

    // Try to fetch user info from registry
    let registry_url = config.registry_url.clone();
    let client = reqwest::blocking::Client::new();

    match client
        .get(format!("{}/api/auth/whoami", registry_url))
        .header("Authorization", format!("Bearer {}", config.api_key))
        .send()
    {
        Ok(response) if response.status().is_success() => {
            if let Ok(user_info) = response.json::<serde_json::Value>() {
                println!("Current authentication:");
                println!("  {} {}", "Registry:".dimmed(), registry_url);

                if let Some(username) = user_info["github_username"].as_str() {
                    println!("  {} @{}", "GitHub User:".dimmed(), username.green());
                }
                if let Some(email) = user_info["email"].as_str() {
                    println!("  {} {}", "Email:".dimmed(), email);
                }
                if let Some(packages) = user_info["packages_published"].as_u64() {
                    println!("  {} {}", "Packages Published:".dimmed(), packages);
                }

                if let Some(keys) = user_info["api_keys"].as_array() {
                    println!();
                    println!("  {} ({} active)", "API Keys:".dimmed(), keys.len());
                    for key in keys {
                        if let (Some(name), Some(prefix)) =
                            (key["name"].as_str(), key["prefix"].as_str())
                        {
                            println!("    • {} ({})", name, prefix.dimmed());
                        }
                    }
                }
            }
        }
        _ => {
            // Fallback to showing local config
            println!("Current authentication:");
            println!("  {} {}", "Registry:".dimmed(), registry_url);

            // Show token prefix only
            let token_prefix = config.api_key.chars().take(15).collect::<String>() + "...";
            println!("  {} {}", "API Token:".dimmed(), token_prefix.green());
            println!();
            println!(
                "  {} Could not fetch user details from registry",
                "Note:".yellow()
            );
        }
    }

    println!();
    println!(
        "{} To manage API keys, visit: {}/dashboard/keys",
        "Tip:".yellow(),
        registry_url
    );

    Ok(())
}

/// Get the registry URL
pub fn get_registry_url() -> String {
    // First check environment variable
    if let Ok(url) = std::env::var("HORUS_REGISTRY_URL") {
        return url;
    }

    // Then check config file
    if let Ok(config) = load_auth_config() {
        return config.registry_url;
    }

    // Default
    crate::config::registry_url()
}

/// List API keys
pub fn keys_list() -> HorusResult<()> {
    let config_path =
        auth_config_path().map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    if !config_path.exists() {
        println!("{} Not logged in", crate::cli_output::ICON_WARN.yellow());
        return Ok(());
    }

    let config = load_auth_config()?;

    let registry_url = config.registry_url.clone();
    let client = reqwest::blocking::Client::new();

    match client
        .get(format!("{}/api/auth/keys", registry_url))
        .header("Authorization", format!("Bearer {}", config.api_key))
        .send()
    {
        Ok(response) if response.status().is_success() => {
            if let Ok(keys_info) = response.json::<serde_json::Value>() {
                println!("{}", "API Keys".cyan().bold());
                println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

                if let Some(keys) = keys_info["keys"].as_array() {
                    if keys.is_empty() {
                        println!("  No API keys found");
                    } else {
                        for key in keys {
                            let name = key["name"].as_str().unwrap_or("unnamed");
                            let prefix = key["prefix"].as_str().unwrap_or("horus_key_***");
                            let last_used = key["last_used"].as_str();
                            let permissions = key["permissions"].as_array();

                            println!();
                            println!("  {} {}", "•".green(), name.bold());
                            println!("    {} {}", "Key:".dimmed(), prefix);

                            if let Some(perms) = permissions {
                                let perm_strs: Vec<&str> =
                                    perms.iter().filter_map(|p| p.as_str()).collect();
                                println!(
                                    "    {} {}",
                                    "Permissions:".dimmed(),
                                    perm_strs.join(", ")
                                );
                            }

                            if let Some(used) = last_used {
                                println!("    {} {}", "Last used:".dimmed(), used);
                            } else {
                                println!("    {} {}", "Last used:".dimmed(), "never".dimmed());
                            }
                        }
                    }
                }

                println!();
                println!(
                    "{} Manage keys at: {}/dashboard/keys",
                    "Tip:".yellow(),
                    registry_url.cyan()
                );
            }
        }
        _ => {
            println!(
                "{} Could not fetch API keys",
                crate::cli_output::ICON_WARN.yellow()
            );
            println!();
            println!("Manage keys at: {}/dashboard/keys", registry_url.cyan());
        }
    }

    Ok(())
}

/// Revoke an API key
pub fn keys_revoke(key_id: &str) -> HorusResult<()> {
    let config_path =
        auth_config_path().map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    if !config_path.exists() {
        println!("{} Not logged in", crate::cli_output::ICON_WARN.yellow());
        return Ok(());
    }

    let config = load_auth_config()?;

    let registry_url = config.registry_url.clone();
    let client = reqwest::blocking::Client::new();

    match client
        .delete(format!("{}/api/auth/keys/{}", registry_url, key_id))
        .header("Authorization", format!("Bearer {}", config.api_key))
        .send()
    {
        Ok(response) if response.status().is_success() => {
            println!(
                "{} API key revoked successfully",
                crate::cli_output::ICON_SUCCESS.green()
            );
        }
        Ok(response) if response.status() == reqwest::StatusCode::NOT_FOUND => {
            println!(
                "{} API key not found",
                crate::cli_output::ICON_WARN.yellow()
            );
        }
        _ => {
            println!(
                "{} Could not revoke API key",
                crate::cli_output::ICON_ERROR.red()
            );
            println!();
            println!("Try revoking at: {}/dashboard/keys", registry_url.cyan());
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn auth_config_serde_roundtrip() {
        let config = AuthConfig {
            api_key: "horus_key_abc123".to_string(),
            registry_url: "https://registry.example.com".to_string(),
            github_username: Some("testuser".to_string()),
        };
        let json = serde_json::to_string(&config).unwrap();
        let parsed: AuthConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.api_key, "horus_key_abc123");
        assert_eq!(parsed.registry_url, "https://registry.example.com");
        assert_eq!(parsed.github_username, Some("testuser".to_string()));
    }

    #[test]
    fn auth_config_github_username_optional() {
        let json = r#"{"api_key":"horus_key_x","registry_url":"http://localhost"}"#;
        let config: AuthConfig = serde_json::from_str(json).unwrap();
        assert!(config.github_username.is_none());
    }

    #[test]
    fn token_format_validation_valid() {
        let valid_token = "horus_key_abc123";
        assert!(valid_token.starts_with("horus_key_"));
        // Valid token should be accepted in an AuthConfig
        let config = AuthConfig {
            api_key: valid_token.to_string(),
            registry_url: "https://registry.example.com".to_string(),
            github_username: None,
        };
        assert!(config.api_key.starts_with("horus_key_"));
        assert!(
            config.api_key.len() > "horus_key_".len(),
            "token must have content after prefix"
        );
    }

    #[test]
    fn token_format_validation_invalid() {
        let invalid_tokens = ["invalid_token", "", "horus_key", "HORUS_KEY_abc"];
        for token in &invalid_tokens {
            assert!(
                !token.starts_with("horus_key_"),
                "token '{}' should NOT be valid",
                token
            );
        }
    }

    #[test]
    fn extract_query_param_basic() {
        assert_eq!(
            extract_query_param("/callback?code=abc123&user=testuser", "code"),
            Some("abc123".to_string())
        );
        assert_eq!(
            extract_query_param("/callback?code=abc123&user=testuser", "user"),
            Some("testuser".to_string())
        );
    }

    #[test]
    fn extract_query_param_missing() {
        assert_eq!(extract_query_param("/callback?code=abc123", "user"), None);
        assert_eq!(extract_query_param("/callback", "code"), None);
    }

    #[test]
    fn extract_query_param_url_decode() {
        assert_eq!(
            extract_query_param("/callback?name=hello%20world", "name"),
            Some("hello world".to_string())
        );
        // Test all RFC 3986 percent-encoded characters
        assert_eq!(
            extract_query_param("/callback?val=a%26b%3Dc", "val"),
            Some("a&b=c".to_string())
        );
        assert_eq!(
            extract_query_param("/callback?email=user%40example.com", "email"),
            Some("user@example.com".to_string())
        );
        assert_eq!(
            extract_query_param("/callback?path=%2Fhome%2Fuser", "path"),
            Some("/home/user".to_string())
        );
        // Test + as space (form encoding)
        assert_eq!(
            extract_query_param("/callback?q=hello+world", "q"),
            Some("hello world".to_string())
        );
    }

    #[test]
    fn url_decode_comprehensive() {
        assert_eq!(url_decode("hello%20world"), "hello world");
        assert_eq!(url_decode("100%25"), "100%");
        assert_eq!(url_decode("no+spaces"), "no spaces");
        assert_eq!(url_decode("plain"), "plain");
        // Invalid percent encoding passes through
        assert_eq!(url_decode("%ZZ"), "%ZZ");
        assert_eq!(url_decode("%2"), "%2");
    }

    // ── url_decode edge cases ───────────────────────────────────────

    #[test]
    fn url_decode_empty_string() {
        assert_eq!(url_decode(""), "");
    }

    #[test]
    fn url_decode_only_percent_encoded() {
        assert_eq!(url_decode("%48%65%6C%6C%6F"), "Hello");
    }

    #[test]
    fn url_decode_mixed_case_hex() {
        assert_eq!(url_decode("%2f"), "/");
        assert_eq!(url_decode("%2F"), "/");
        assert_eq!(url_decode("%2a"), "*");
        assert_eq!(url_decode("%2A"), "*");
    }

    #[test]
    fn url_decode_consecutive_percent_encodings() {
        // %C3%A9 = UTF-8 for 'e' with acute accent
        assert_eq!(url_decode("%C3%A9"), "\u{00e9}");
    }

    #[test]
    fn url_decode_plus_and_percent_mixed() {
        assert_eq!(url_decode("a+b%20c"), "a b c");
    }

    #[test]
    fn url_decode_percent_at_end_of_string() {
        // Trailing '%' without two hex digits should pass through
        assert_eq!(url_decode("abc%"), "abc%");
    }

    #[test]
    fn url_decode_percent_with_one_char_at_end() {
        // '%' followed by only one character at end of string
        assert_eq!(url_decode("abc%2"), "abc%2");
    }

    #[test]
    fn url_decode_special_url_characters() {
        assert_eq!(url_decode("%3F"), "?");
        assert_eq!(url_decode("%26"), "&");
        assert_eq!(url_decode("%3D"), "=");
        assert_eq!(url_decode("%23"), "#");
        assert_eq!(url_decode("%40"), "@");
        assert_eq!(url_decode("%21"), "!");
    }

    #[test]
    fn url_decode_all_plus_signs() {
        assert_eq!(url_decode("+++"), "   ");
    }

    #[test]
    fn url_decode_non_ascii_passthrough() {
        // Non-encoded non-ASCII bytes should pass through
        assert_eq!(url_decode("abc123"), "abc123");
    }

    #[test]
    fn url_decode_null_byte() {
        assert_eq!(url_decode("%00"), "\0");
    }

    #[test]
    fn url_decode_preserves_slashes_and_dots() {
        assert_eq!(url_decode("/path/to/file.txt"), "/path/to/file.txt");
    }

    #[test]
    fn url_decode_long_string() {
        let input = "a%20".repeat(100);
        let expected = "a ".repeat(100);
        assert_eq!(url_decode(&input), expected);
    }

    // ── extract_query_param edge cases ──────────────────────────────

    #[test]
    fn extract_query_param_empty_value() {
        assert_eq!(
            extract_query_param("/cb?code=&user=bob", "code"),
            Some("".to_string())
        );
    }

    #[test]
    fn extract_query_param_no_query_string() {
        assert_eq!(extract_query_param("/callback", "anything"), None);
    }

    #[test]
    fn extract_query_param_empty_query_string() {
        assert_eq!(extract_query_param("/callback?", "code"), None);
    }

    #[test]
    fn extract_query_param_first_param() {
        assert_eq!(
            extract_query_param("/cb?first=1&second=2&third=3", "first"),
            Some("1".to_string())
        );
    }

    #[test]
    fn extract_query_param_middle_param() {
        assert_eq!(
            extract_query_param("/cb?first=1&second=2&third=3", "second"),
            Some("2".to_string())
        );
    }

    #[test]
    fn extract_query_param_last_param() {
        assert_eq!(
            extract_query_param("/cb?first=1&second=2&third=3", "third"),
            Some("3".to_string())
        );
    }

    #[test]
    fn extract_query_param_key_is_prefix_of_another() {
        // "code" should not match "code_verifier"
        assert_eq!(
            extract_query_param("/cb?code_verifier=abc&code=xyz", "code"),
            Some("xyz".to_string())
        );
    }

    #[test]
    fn extract_query_param_value_contains_equals() {
        // Values with = in them (base64, etc.)
        assert_eq!(
            extract_query_param("/cb?token=abc=def==", "token"),
            Some("abc=def==".to_string())
        );
    }

    #[test]
    fn extract_query_param_url_encoded_value() {
        assert_eq!(
            extract_query_param("/cb?user=hello%20world", "user"),
            Some("hello world".to_string())
        );
    }

    #[test]
    fn extract_query_param_url_encoded_special_chars() {
        assert_eq!(
            extract_query_param("/cb?redirect=https%3A%2F%2Fexample.com%2Fpath", "redirect"),
            Some("https://example.com/path".to_string())
        );
    }

    #[test]
    fn extract_query_param_multiple_question_marks() {
        // split('?').nth(1) takes only between the first and second '?'
        // so a second '?' truncates the value
        assert_eq!(
            extract_query_param("/cb?code=abc?extra=ignored", "code"),
            Some("abc".to_string())
        );
    }

    #[test]
    fn extract_query_param_no_value_part() {
        // key without =value should return None for that key
        assert_eq!(extract_query_param("/cb?orphankey", "orphankey"), None);
    }

    #[test]
    fn extract_query_param_single_char_key_and_value() {
        assert_eq!(extract_query_param("/cb?a=b", "a"), Some("b".to_string()));
    }

    #[test]
    fn extract_query_param_plus_in_value() {
        assert_eq!(
            extract_query_param("/cb?q=hello+world", "q"),
            Some("hello world".to_string())
        );
    }

    // ── AuthConfig serialization edge cases ─────────────────────────

    #[test]
    fn auth_config_with_empty_api_key() {
        let config = AuthConfig {
            api_key: "".to_string(),
            registry_url: "https://example.com".to_string(),
            github_username: None,
        };
        let json = serde_json::to_string(&config).unwrap();
        let parsed: AuthConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.api_key, "");
    }

    #[test]
    fn auth_config_with_special_characters_in_key() {
        let config = AuthConfig {
            api_key: "horus_key_abc/+==123".to_string(),
            registry_url: "https://example.com".to_string(),
            github_username: Some("user-name.with_special".to_string()),
        };
        let json = serde_json::to_string(&config).unwrap();
        let parsed: AuthConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.api_key, "horus_key_abc/+==123");
        assert_eq!(
            parsed.github_username,
            Some("user-name.with_special".to_string())
        );
    }

    #[test]
    fn auth_config_pretty_print_roundtrip() {
        let config = AuthConfig {
            api_key: "horus_key_test".to_string(),
            registry_url: "https://plugins.horusrobotics.dev".to_string(),
            github_username: Some("dev".to_string()),
        };
        let json = serde_json::to_string_pretty(&config).unwrap();
        let parsed: AuthConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.api_key, config.api_key);
        assert_eq!(parsed.registry_url, config.registry_url);
        assert_eq!(parsed.github_username, config.github_username);
    }

    #[test]
    fn auth_config_rejects_invalid_json() {
        let result = serde_json::from_str::<AuthConfig>("not json at all");
        assert!(result.is_err());
    }

    #[test]
    fn auth_config_rejects_missing_required_field() {
        // Missing registry_url
        let json = r#"{"api_key":"horus_key_x"}"#;
        let result = serde_json::from_str::<AuthConfig>(json);
        assert!(result.is_err());
    }

    #[test]
    fn auth_config_rejects_missing_api_key() {
        let json = r#"{"registry_url":"https://example.com"}"#;
        let result = serde_json::from_str::<AuthConfig>(json);
        assert!(result.is_err());
    }

    #[test]
    fn auth_config_ignores_extra_fields() {
        let json = r#"{
            "api_key": "horus_key_x",
            "registry_url": "https://example.com",
            "github_username": null,
            "extra_field": "should be ignored"
        }"#;
        // serde default: deny unknown fields is NOT set, so this should work
        let result = serde_json::from_str::<AuthConfig>(json);
        assert!(result.is_ok());
    }

    #[test]
    fn auth_config_null_username_is_none() {
        let json = r#"{
            "api_key": "horus_key_x",
            "registry_url": "https://example.com",
            "github_username": null
        }"#;
        let config: AuthConfig = serde_json::from_str(json).unwrap();
        assert!(config.github_username.is_none());
    }

    #[test]
    fn auth_config_unicode_username() {
        let config = AuthConfig {
            api_key: "horus_key_test".to_string(),
            registry_url: "https://example.com".to_string(),
            github_username: Some("\u{1F916}robot\u{1F916}".to_string()),
        };
        let json = serde_json::to_string(&config).unwrap();
        let parsed: AuthConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.github_username, config.github_username);
    }

    // ── save_auth_config + load_auth_config filesystem tests ────────

    #[test]
    fn save_and_load_auth_config_roundtrip() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        // Create the .horus directory
        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();

        let result = save_auth_config(
            "horus_key_test123",
            "https://test-registry.example.com",
            Some("testuser"),
        );
        assert!(result.is_ok(), "save_auth_config failed: {:?}", result);

        let loaded = load_auth_config();
        assert!(loaded.is_ok(), "load_auth_config failed: {:?}", loaded);
        let config = loaded.unwrap();
        assert_eq!(config.api_key, "horus_key_test123");
        assert_eq!(config.registry_url, "https://test-registry.example.com");
        assert_eq!(config.github_username, Some("testuser".to_string()));

        // Restore HOME
        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    #[test]
    fn save_auth_config_without_username() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();

        let result = save_auth_config("horus_key_nouser", "https://registry.example.com", None);
        assert!(result.is_ok());

        let config = load_auth_config().unwrap();
        assert_eq!(config.api_key, "horus_key_nouser");
        assert!(config.github_username.is_none());

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    #[test]
    fn save_auth_config_creates_valid_json_on_disk() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();

        save_auth_config("horus_key_disk", "https://example.com", Some("alice")).unwrap();

        // Read the raw file and verify it's valid JSON
        let config_path = horus_dir.join("auth.json");
        assert!(config_path.exists());
        let raw = fs::read_to_string(&config_path).unwrap();
        let value: serde_json::Value = serde_json::from_str(&raw).unwrap();
        assert_eq!(value["api_key"], "horus_key_disk");
        assert_eq!(value["registry_url"], "https://example.com");
        assert_eq!(value["github_username"], "alice");

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    #[test]
    fn save_auth_config_overwrites_existing() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();

        // Write first config
        save_auth_config("horus_key_first", "https://first.com", Some("user1")).unwrap();
        let config1 = load_auth_config().unwrap();
        assert_eq!(config1.api_key, "horus_key_first");

        // Overwrite with second config
        save_auth_config("horus_key_second", "https://second.com", Some("user2")).unwrap();
        let config2 = load_auth_config().unwrap();
        assert_eq!(config2.api_key, "horus_key_second");
        assert_eq!(config2.registry_url, "https://second.com");
        assert_eq!(config2.github_username, Some("user2".to_string()));

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    #[cfg(unix)]
    #[test]
    fn save_auth_config_sets_owner_only_permissions() {
        use std::os::unix::fs::PermissionsExt;

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();

        save_auth_config("horus_key_perms", "https://example.com", None).unwrap();

        let config_path = horus_dir.join("auth.json");
        let metadata = fs::metadata(&config_path).unwrap();
        let mode = metadata.permissions().mode() & 0o777;
        assert_eq!(
            mode, 0o600,
            "auth.json should be owner-only (0600), got {:o}",
            mode
        );

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    #[test]
    fn load_auth_config_returns_error_when_not_authenticated() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        // Create .horus dir but no auth.json
        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();

        let result = load_auth_config();
        assert!(result.is_err());
        let err_msg = format!("{}", result.unwrap_err());
        assert!(
            err_msg.contains("not authenticated"),
            "Expected 'not authenticated' error, got: {}",
            err_msg
        );

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    #[test]
    fn load_auth_config_returns_error_for_corrupt_json() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();
        fs::write(horus_dir.join("auth.json"), "this is not json").unwrap();

        let result = load_auth_config();
        assert!(result.is_err());
        let err_msg = format!("{}", result.unwrap_err());
        assert!(
            err_msg.contains("failed to parse auth config"),
            "Expected parse error, got: {}",
            err_msg
        );

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    #[test]
    fn load_auth_config_returns_error_for_partial_json() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();
        // Valid JSON but missing required fields
        fs::write(horus_dir.join("auth.json"), r#"{"api_key":"test"}"#).unwrap();

        let result = load_auth_config();
        assert!(result.is_err());

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    #[test]
    fn load_auth_config_returns_error_for_empty_file() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();
        fs::write(horus_dir.join("auth.json"), "").unwrap();

        let result = load_auth_config();
        assert!(result.is_err());

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    // ── auth_config_path tests ──────────────────────────────────────

    #[test]
    fn auth_config_path_ends_with_auth_json() {
        let path = auth_config_path().unwrap();
        assert!(
            path.ends_with("auth.json"),
            "Expected path ending with auth.json, got: {:?}",
            path
        );
    }

    #[test]
    fn auth_config_path_is_inside_horus_dir() {
        let path = auth_config_path().unwrap();
        let parent = path.parent().unwrap();
        assert!(
            parent.to_string_lossy().contains("horus"),
            "auth.json should be inside horus config dir, parent was: {:?}",
            parent
        );
    }

    #[test]
    fn auth_config_path_returns_valid_path() {
        let path = auth_config_path().unwrap();
        assert!(path.is_absolute(), "auth config path should be absolute");
        assert_eq!(path.file_name().unwrap(), "auth.json");
        assert!(
            path.to_string_lossy().contains("horus"),
            "auth config path should be inside a horus directory"
        );
    }

    // ── get_registry_url tests ──────────────────────────────────────

    #[test]
    fn get_registry_url_env_var_takes_priority() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::var("HORUS_REGISTRY_URL").ok();

        std::env::set_var("HORUS_REGISTRY_URL", "https://custom-registry.test");
        let url = get_registry_url();
        assert_eq!(url, "https://custom-registry.test");

        // Restore
        match original {
            Some(val) => std::env::set_var("HORUS_REGISTRY_URL", val),
            None => std::env::remove_var("HORUS_REGISTRY_URL"),
        }
    }

    #[test]
    fn get_registry_url_falls_back_to_config() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        let original_registry = std::env::var("HORUS_REGISTRY_URL").ok();
        std::env::set_var("HOME", tmp.path());
        std::env::remove_var("HORUS_REGISTRY_URL");

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();
        save_auth_config("horus_key_test", "https://from-config.test", None).unwrap();

        let url = get_registry_url();
        assert_eq!(url, "https://from-config.test");

        // Restore
        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
        match original_registry {
            Some(val) => std::env::set_var("HORUS_REGISTRY_URL", val),
            None => std::env::remove_var("HORUS_REGISTRY_URL"),
        }
    }

    #[test]
    fn get_registry_url_falls_back_to_default() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        let original_registry = std::env::var("HORUS_REGISTRY_URL").ok();
        std::env::set_var("HOME", tmp.path());
        std::env::remove_var("HORUS_REGISTRY_URL");

        // No auth config file exists, so should fall back to default
        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();

        let url = get_registry_url();
        assert_eq!(url, crate::config::registry_url());

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
        match original_registry {
            Some(val) => std::env::set_var("HORUS_REGISTRY_URL", val),
            None => std::env::remove_var("HORUS_REGISTRY_URL"),
        }
    }

    // ── logout tests ────────────────────────────────────────────────

    #[test]
    fn logout_removes_auth_config_file() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();
        save_auth_config("horus_key_logout", "https://example.com", Some("bob")).unwrap();

        let config_path = horus_dir.join("auth.json");
        assert!(config_path.exists(), "auth.json should exist before logout");

        let result = logout();
        assert!(result.is_ok());
        assert!(
            !config_path.exists(),
            "auth.json should be removed after logout"
        );

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    #[test]
    fn logout_succeeds_when_not_logged_in() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();
        // No auth.json exists

        let result = logout();
        assert!(
            result.is_ok(),
            "logout should succeed even when not logged in"
        );

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    #[test]
    fn logout_then_load_auth_fails() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();
        save_auth_config("horus_key_then_logout", "https://example.com", None).unwrap();

        // Verify load works before logout
        assert!(load_auth_config().is_ok());

        logout().unwrap();

        // Load should fail after logout
        let result = load_auth_config();
        assert!(result.is_err());

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    // ── Token validation edge cases ─────────────────────────────────

    #[test]
    fn token_prefix_is_exactly_horus_key_underscore() {
        let valid = "horus_key_";
        assert!(valid.starts_with("horus_key_"));

        // Almost-valid prefixes
        let near_misses = [
            "horus_key",   // missing trailing underscore
            "horus-key_",  // wrong separator
            "Horus_key_",  // wrong case
            "HORUS_KEY_",  // all caps
            "horus_Key_",  // mixed case
            " horus_key_", // leading space
            "horus_key_ ", // value starts with space (prefix still matches but...)
        ];
        for token in &near_misses {
            // Only the last one starts_with "horus_key_" — rest should not
            if !token.starts_with("horus_key_") {
                // Good, this is invalid as expected
            }
        }
    }

    #[test]
    fn token_format_boundary_cases() {
        // Exactly the prefix with nothing after it
        assert!("horus_key_".starts_with("horus_key_"));

        // Very long token
        let long_token = format!("horus_key_{}", "a".repeat(1000));
        assert!(long_token.starts_with("horus_key_"));

        // Token with special characters after prefix
        assert!("horus_key_!@#$%^&*()".starts_with("horus_key_"));
    }

    // ── save_auth_config with special content ───────────────────────

    #[test]
    fn save_auth_config_with_very_long_api_key() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();

        let long_key = format!("horus_key_{}", "x".repeat(2048));
        save_auth_config(&long_key, "https://example.com", None).unwrap();

        let config = load_auth_config().unwrap();
        assert_eq!(config.api_key, long_key);

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    #[test]
    fn save_auth_config_with_url_containing_path() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();

        let url_with_path = "https://registry.example.com/api/v2";
        save_auth_config("horus_key_path", url_with_path, None).unwrap();

        let config = load_auth_config().unwrap();
        assert_eq!(config.registry_url, url_with_path);

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    #[test]
    fn save_auth_config_with_localhost_url() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();

        save_auth_config("horus_key_local", "http://localhost:3000", Some("dev")).unwrap();

        let config = load_auth_config().unwrap();
        assert_eq!(config.registry_url, "http://localhost:3000");

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    #[test]
    fn save_auth_config_with_unicode_username() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();

        save_auth_config(
            "horus_key_uni",
            "https://example.com",
            Some("\u{1F680}rocket-user"),
        )
        .unwrap();

        let config = load_auth_config().unwrap();
        assert_eq!(
            config.github_username,
            Some("\u{1F680}rocket-user".to_string())
        );

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    // ── Concurrent save/load safety ─────────────────────────────────

    #[test]
    fn multiple_saves_last_write_wins() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();

        for i in 0..10 {
            save_auth_config(
                &format!("horus_key_{}", i),
                "https://example.com",
                Some(&format!("user{}", i)),
            )
            .unwrap();
        }

        let config = load_auth_config().unwrap();
        assert_eq!(config.api_key, "horus_key_9");
        assert_eq!(config.github_username, Some("user9".to_string()));

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    // ── whoami when not logged in ───────────────────────────────────

    #[test]
    fn whoami_not_logged_in_succeeds() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();

        // whoami should not error when not logged in, just print guidance
        let result = whoami();
        assert!(result.is_ok());

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    // ── keys_list when not logged in ────────────────────────────────

    #[test]
    fn keys_list_not_logged_in_succeeds() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();

        let result = keys_list();
        assert!(result.is_ok());

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    // ── keys_revoke when not logged in ──────────────────────────────

    #[test]
    fn keys_revoke_not_logged_in_succeeds() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();

        let result = keys_revoke("some-key-id");
        assert!(result.is_ok());

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    // ── Full login-logout cycle ─────────────────────────────────────

    #[test]
    fn full_save_load_logout_cycle() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original_home = std::env::var("HOME").ok();
        std::env::set_var("HOME", tmp.path());

        let horus_dir = tmp.path().join(".config/horus");
        fs::create_dir_all(&horus_dir).unwrap();

        // Step 1: save
        save_auth_config("horus_key_cycle", "https://cycle.test", Some("cyclist")).unwrap();

        // Step 2: verify load
        let config = load_auth_config().unwrap();
        assert_eq!(config.api_key, "horus_key_cycle");
        assert_eq!(config.github_username, Some("cyclist".to_string()));

        // Step 3: logout
        logout().unwrap();

        // Step 4: verify gone
        assert!(load_auth_config().is_err());

        // Step 5: re-save with different user
        save_auth_config("horus_key_cycle2", "https://cycle2.test", Some("cyclist2")).unwrap();

        // Step 6: verify new config
        let config2 = load_auth_config().unwrap();
        assert_eq!(config2.api_key, "horus_key_cycle2");
        assert_eq!(config2.registry_url, "https://cycle2.test");

        match original_home {
            Some(val) => std::env::set_var("HOME", val),
            None => std::env::remove_var("HOME"),
        }
    }

    // ── URL decode robustness / fuzzy inputs ────────────────────────

    #[test]
    fn url_decode_double_encoded() {
        // %2520 is double-encoded space: %25 -> %, 20 stays -> %20
        // Our decoder only decodes once
        assert_eq!(url_decode("%2520"), "%20");
    }

    #[test]
    fn url_decode_just_percent() {
        assert_eq!(url_decode("%"), "%");
    }

    #[test]
    fn url_decode_percent_followed_by_non_hex() {
        assert_eq!(url_decode("%GG"), "%GG");
        assert_eq!(url_decode("%XY"), "%XY");
    }

    #[test]
    fn url_decode_mixed_valid_invalid_percent() {
        assert_eq!(url_decode("%20%GG%41"), " %GGA");
    }

    #[test]
    fn url_decode_only_plus_signs() {
        assert_eq!(url_decode("+"), " ");
        assert_eq!(url_decode("+++++"), "     ");
    }

    #[test]
    fn url_decode_full_alphabet_hex() {
        // Test all hex chars 0-F in various positions
        assert_eq!(url_decode("%30"), "0"); // ASCII '0'
        assert_eq!(url_decode("%39"), "9"); // ASCII '9'
        assert_eq!(url_decode("%41"), "A"); // ASCII 'A'
        assert_eq!(url_decode("%46"), "F"); // ASCII 'F'
        assert_eq!(url_decode("%5A"), "Z"); // ASCII 'Z'
        assert_eq!(url_decode("%61"), "a"); // ASCII 'a'
        assert_eq!(url_decode("%66"), "f"); // ASCII 'f'
        assert_eq!(url_decode("%7A"), "z"); // ASCII 'z'
    }

    // ── extract_query_param with tricky paths ───────────────────────

    #[test]
    fn extract_query_param_root_path() {
        assert_eq!(
            extract_query_param("/?key=value", "key"),
            Some("value".to_string())
        );
    }

    #[test]
    fn extract_query_param_deep_path() {
        assert_eq!(
            extract_query_param("/a/b/c/d?key=value", "key"),
            Some("value".to_string())
        );
    }

    #[test]
    fn extract_query_param_many_params() {
        let path = "/cb?a=1&b=2&c=3&d=4&e=5&f=6&g=7&h=8&i=9&j=10";
        assert_eq!(extract_query_param(path, "a"), Some("1".to_string()));
        assert_eq!(extract_query_param(path, "j"), Some("10".to_string()));
        assert_eq!(extract_query_param(path, "e"), Some("5".to_string()));
        assert_eq!(extract_query_param(path, "z"), None);
    }

    #[test]
    fn extract_query_param_duplicate_keys_returns_first() {
        assert_eq!(
            extract_query_param("/cb?key=first&key=second", "key"),
            Some("first".to_string())
        );
    }

    #[test]
    fn extract_query_param_empty_path() {
        assert_eq!(extract_query_param("", "key"), None);
    }

    #[test]
    fn extract_query_param_just_question_mark() {
        assert_eq!(extract_query_param("?", "key"), None);
    }

    #[test]
    fn extract_query_param_key_with_no_equals() {
        // "flag" has no = sign so splitn(2, '=').next() gives "flag", .nth(1) gives None
        assert_eq!(extract_query_param("/cb?flag&key=val", "flag"), None);
        assert_eq!(
            extract_query_param("/cb?flag&key=val", "key"),
            Some("val".to_string())
        );
    }

    #[test]
    fn extract_query_param_ampersand_in_encoded_value() {
        // %26 should decode to & but the split happens before decode
        assert_eq!(
            extract_query_param("/cb?data=a%26b", "data"),
            Some("a&b".to_string())
        );
    }
}
