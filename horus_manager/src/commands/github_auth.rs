use anyhow::Result;
use colored::*;
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
    let timeout = std::time::Duration::from_secs(300);

    listener.set_nonblocking(true).ok();

    let mut stream = loop {
        match listener.accept() {
            Ok((stream, _)) => break stream,
            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                if start.elapsed() > timeout {
                    return Err("Timed out waiting for authentication (5 minutes)".to_string());
                }
                std::thread::sleep(std::time::Duration::from_millis(100));
                continue;
            }
            Err(e) => return Err(format!("Failed to accept connection: {}", e)),
        }
    };

    // Read the HTTP request
    let mut buf = [0u8; 4096];
    stream
        .set_read_timeout(Some(std::time::Duration::from_secs(5)))
        .ok();
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

    fs::write(&config_path, config_json).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Failed to save auth config: {}",
            e
        )))
    })?;

    // Restrict permissions to owner-only (contains API token)
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let _ = fs::set_permissions(&config_path, fs::Permissions::from_mode(0o600));
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
        assert!("horus_key_abc123".starts_with("horus_key_"));
    }

    #[test]
    fn token_format_validation_invalid() {
        assert!(!"invalid_token".starts_with("horus_key_"));
        assert!(!"".starts_with("horus_key_"));
        assert!(!"horus_key".starts_with("horus_key_"));
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
}
