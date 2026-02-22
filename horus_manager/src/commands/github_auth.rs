use anyhow::{anyhow, Result};
use colored::*;
use dirs::home_dir;
use horus_core::error::{HorusError, HorusResult};
use serde::{Deserialize, Serialize};
use std::fs;
use std::io::{self, Write};
use std::path::PathBuf;

#[derive(Debug, Serialize, Deserialize)]
struct AuthConfig {
    api_key: String,
    registry_url: String,
    #[serde(default = "default_cloud_url")]
    cloud_url: String,
    github_username: Option<String>,
    organization_id: Option<String>,
    organization_name: Option<String>,
    plan: Option<String>,
}

fn default_cloud_url() -> String {
    crate::config::DEFAULT_CLOUD_URL.to_string()
}

/// Get the path to the auth config file
fn auth_config_path() -> Result<PathBuf> {
    let home = home_dir().ok_or_else(|| anyhow!("Could not find home directory"))?;
    let config_dir = home.join(".horus");

    // Create .horus directory if it doesn't exist
    if !config_dir.exists() {
        fs::create_dir_all(&config_dir)?;
    }

    Ok(config_dir.join("auth.json"))
}

/// Login to the HORUS registry with GitHub
pub fn login() -> HorusResult<()> {
    let registry_url = get_registry_url();

    // GitHub OAuth flow
    println!("Logging in to HORUS registry with GitHub...");
    println!();
    println!("{} Opening browser for GitHub authentication...", "".cyan());
    println!("  {} {}/auth/github", "URL:".dimmed(), registry_url);
    println!();

    // Open browser for GitHub OAuth
    let auth_url = format!("{}/auth/github", registry_url);
    if open::that(&auth_url).is_err() {
        println!("{} Could not open browser automatically.", "!".yellow());
        println!("Please visit: {}", auth_url.cyan());
    }

    println!();
    println!(
        "{} After authenticating with GitHub:",
        "Next steps:".green()
    );
    println!("  1. You'll be redirected back to the registry");
    println!("  2. Copy the displayed instructions");
    println!("  3. Run: {}", "horus auth generate-key".cyan());
    println!();
    println!("GitHub authentication initiated!");

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
    io::stdin()
        .read_line(&mut api_key)
        .map_err(|e| HorusError::Config(format!("Failed to read input: {}", e)))?;

    let api_key = api_key.trim().to_string();

    // Validate token format
    if !api_key.starts_with("horus_key_") {
        return Err(HorusError::Config(
            "Invalid token format. Token should start with 'horus_key_'".to_string(),
        ));
    }

    // Save auth config (unified for Registry + Cloud)
    let config = AuthConfig {
        api_key: api_key.clone(),
        registry_url: registry_url.clone(),
        cloud_url: get_cloud_url(),
        github_username: None, // Will be populated on first API call
        organization_id: None, // Will be populated on first API call
        organization_name: None,
        plan: None,
    };

    let config_path = auth_config_path().map_err(|e| HorusError::Config(e.to_string()))?;

    let config_json = serde_json::to_string_pretty(&config)
        .map_err(|e| HorusError::Config(format!("Failed to serialize config: {}", e)))?;

    fs::write(&config_path, config_json)
        .map_err(|e| HorusError::Config(format!("Failed to save auth config: {}", e)))?;

    // Restrict permissions to owner-only (contains API token)
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let _ = fs::set_permissions(&config_path, fs::Permissions::from_mode(0o600));
    }

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

    let config_path = auth_config_path().map_err(|e| HorusError::Config(e.to_string()))?;

    if config_path.exists() {
        fs::remove_file(&config_path)
            .map_err(|e| HorusError::Config(format!("Failed to remove auth config: {}", e)))?;

        println!("Successfully logged out!");
        println!("  {} API key removed from local storage", "•".dimmed());
    } else {
        println!("{} Not currently logged in", "!".yellow());
    }

    Ok(())
}

/// Show current authenticated user
pub fn whoami() -> HorusResult<()> {
    let config_path = auth_config_path().map_err(|e| HorusError::Config(e.to_string()))?;

    if !config_path.exists() {
        println!("{} Not logged in", "!".yellow());
        println!();
        println!("To authenticate:");
        println!("  1. Run: {}", "horus auth login".cyan());
        println!("  2. Then: {}", "horus auth generate-key".cyan());
        return Ok(());
    }

    let config_content = fs::read_to_string(&config_path)
        .map_err(|e| HorusError::Config(format!("Failed to read auth config: {}", e)))?;

    let config: AuthConfig = serde_json::from_str(&config_content)
        .map_err(|e| HorusError::Config(format!("Failed to parse auth config: {}", e)))?;

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

/// Get the current auth token (used by other commands)
pub fn get_auth_token() -> Option<String> {
    // First check environment variable
    if let Ok(token) = std::env::var("HORUS_API_KEY") {
        return Some(token);
    }

    // Then check config file
    if let Ok(config_path) = auth_config_path() {
        if config_path.exists() {
            if let Ok(content) = fs::read_to_string(&config_path) {
                if let Ok(config) = serde_json::from_str::<AuthConfig>(&content) {
                    return Some(config.api_key);
                }
            }
        }
    }

    None
}

/// Get the registry URL
pub fn get_registry_url() -> String {
    // First check environment variable
    if let Ok(url) = std::env::var("HORUS_REGISTRY_URL") {
        return url;
    }

    // Then check config file
    if let Ok(config_path) = auth_config_path() {
        if config_path.exists() {
            if let Ok(content) = fs::read_to_string(&config_path) {
                if let Ok(config) = serde_json::from_str::<AuthConfig>(&content) {
                    return config.registry_url;
                }
            }
        }
    }

    // Default
    crate::config::registry_url()
}

/// Get the cloud URL
pub fn get_cloud_url() -> String {
    // First check environment variable
    if let Ok(url) = std::env::var("HORUS_CLOUD_URL") {
        return url;
    }

    // Then check config file
    if let Ok(config_path) = auth_config_path() {
        if config_path.exists() {
            if let Ok(content) = fs::read_to_string(&config_path) {
                if let Ok(config) = serde_json::from_str::<AuthConfig>(&content) {
                    return config.cloud_url;
                }
            }
        }
    }

    // Default
    crate::config::cloud_url()
}

/// Show current organization info
pub fn org() -> HorusResult<()> {
    let config_path = auth_config_path().map_err(|e| HorusError::Config(e.to_string()))?;

    if !config_path.exists() {
        println!("{} Not logged in", "!".yellow());
        println!();
        println!("Run: {}", "horus auth login".cyan());
        return Ok(());
    }

    let config_content = fs::read_to_string(&config_path)
        .map_err(|e| HorusError::Config(format!("Failed to read auth config: {}", e)))?;

    let config: AuthConfig = serde_json::from_str(&config_content)
        .map_err(|e| HorusError::Config(format!("Failed to parse auth config: {}", e)))?;

    // Try to fetch org info from cloud
    let cloud_url = config.cloud_url.clone();
    let client = reqwest::blocking::Client::new();

    match client
        .get(format!("{}/api/auth/organization", cloud_url))
        .header("Authorization", format!("Bearer {}", config.api_key))
        .send()
    {
        Ok(response) if response.status().is_success() => {
            if let Ok(org_info) = response.json::<serde_json::Value>() {
                println!("{}", "Organization".cyan().bold());
                println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

                if let Some(name) = org_info["name"].as_str() {
                    println!("  {} {}", "Name:".dimmed(), name.green());
                }
                if let Some(slug) = org_info["slug"].as_str() {
                    println!("  {} {}", "Slug:".dimmed(), slug);
                }
                if let Some(plan) = org_info["plan"].as_str() {
                    let plan_display = match plan {
                        "free" => plan.dimmed().to_string(),
                        "team" => plan.blue().to_string(),
                        "business" => plan.magenta().to_string(),
                        "enterprise" => plan.yellow().bold().to_string(),
                        _ => plan.to_string(),
                    };
                    println!("  {} {}", "Plan:".dimmed(), plan_display);
                }

                if let Some(members) = org_info["member_count"].as_u64() {
                    println!("  {} {}", "Members:".dimmed(), members);
                }
                if let Some(robots) = org_info["robot_count"].as_u64() {
                    println!("  {} {}", "Robots:".dimmed(), robots);
                }

                println!();
                println!(
                    "{} Manage organization at: {}/settings",
                    "Tip:".yellow(),
                    cloud_url.cyan()
                );
            }
        }
        _ => {
            // Fallback to local config
            println!("{}", "Organization".cyan().bold());
            println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

            if let Some(name) = &config.organization_name {
                println!("  {} {}", "Name:".dimmed(), name.green());
            } else {
                println!("  {} {}", "Name:".dimmed(), "(not set)".dimmed());
            }

            if let Some(plan) = &config.plan {
                println!("  {} {}", "Plan:".dimmed(), plan);
            }

            println!();
            println!(
                "{} Could not fetch details from cloud. Check your connection.",
                "Note:".yellow()
            );
        }
    }

    Ok(())
}

/// Show current usage for the month
pub fn usage() -> HorusResult<()> {
    let config_path = auth_config_path().map_err(|e| HorusError::Config(e.to_string()))?;

    if !config_path.exists() {
        println!("{} Not logged in", "!".yellow());
        return Ok(());
    }

    let config_content = fs::read_to_string(&config_path)
        .map_err(|e| HorusError::Config(format!("Failed to read auth config: {}", e)))?;

    let config: AuthConfig = serde_json::from_str(&config_content)
        .map_err(|e| HorusError::Config(format!("Failed to parse auth config: {}", e)))?;

    let cloud_url = config.cloud_url.clone();
    let client = reqwest::blocking::Client::new();

    match client
        .get(format!("{}/api/billing/usage", cloud_url))
        .header("Authorization", format!("Bearer {}", config.api_key))
        .send()
    {
        Ok(response) if response.status().is_success() => {
            if let Ok(usage_info) = response.json::<serde_json::Value>() {
                println!("{}", "Current Month Usage".cyan().bold());
                println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

                if let Some(period) = usage_info["period"].as_str() {
                    println!("  {} {}", "Period:".dimmed(), period);
                }

                println!();
                println!("  {}", "Resources".green());

                if let (Some(robots), Some(limit)) = (
                    usage_info["robots"]["current"].as_u64(),
                    usage_info["robots"]["limit"].as_u64(),
                ) {
                    let pct = if limit > 0 {
                        (robots as f64 / limit as f64 * 100.0) as u64
                    } else {
                        0
                    };
                    let bar = usage_bar(pct);
                    println!(
                        "    {} {}/{} robots  {}",
                        "Robots:".dimmed(),
                        robots,
                        limit,
                        bar
                    );
                }

                if let (Some(events), Some(limit)) = (
                    usage_info["telemetry"]["current"].as_u64(),
                    usage_info["telemetry"]["limit"].as_u64(),
                ) {
                    let pct = if limit > 0 {
                        (events as f64 / limit as f64 * 100.0) as u64
                    } else {
                        0
                    };
                    let bar = usage_bar(pct);
                    let events_display = format_number(events);
                    let limit_display = format_number(limit);
                    println!(
                        "    {} {}/{} events  {}",
                        "Telemetry:".dimmed(),
                        events_display,
                        limit_display,
                        bar
                    );
                }

                if let Some(cost) = usage_info["estimated_cost"].as_f64() {
                    println!();
                    println!("  {} ${:.2}", "Estimated Cost:".dimmed(), cost);
                }

                println!();
                println!(
                    "{} View detailed usage at: {}/billing",
                    "Tip:".yellow(),
                    cloud_url.cyan()
                );
            }
        }
        Ok(response) if response.status() == reqwest::StatusCode::PAYMENT_REQUIRED => {
            println!("{} No active subscription", "!".yellow());
            println!();
            println!("Upgrade your plan at: {}/billing", cloud_url.cyan());
        }
        _ => {
            println!("{} Could not fetch usage data", "!".yellow());
            println!();
            println!("Check your connection and try again.");
        }
    }

    Ok(())
}

/// Show current plan details
pub fn plan() -> HorusResult<()> {
    let config_path = auth_config_path().map_err(|e| HorusError::Config(e.to_string()))?;

    if !config_path.exists() {
        println!("{} Not logged in", "!".yellow());
        return Ok(());
    }

    let config_content = fs::read_to_string(&config_path)
        .map_err(|e| HorusError::Config(format!("Failed to read auth config: {}", e)))?;

    let config: AuthConfig = serde_json::from_str(&config_content)
        .map_err(|e| HorusError::Config(format!("Failed to parse auth config: {}", e)))?;

    let cloud_url = config.cloud_url.clone();
    let client = reqwest::blocking::Client::new();

    match client
        .get(format!("{}/api/billing/plan", cloud_url))
        .header("Authorization", format!("Bearer {}", config.api_key))
        .send()
    {
        Ok(response) if response.status().is_success() => {
            if let Ok(plan_info) = response.json::<serde_json::Value>() {
                let plan_name = plan_info["name"].as_str().unwrap_or("free");

                println!("{}", "Current Plan".cyan().bold());
                println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

                let plan_display = match plan_name {
                    "free" => "Free".dimmed().to_string(),
                    "team" => "Team".blue().bold().to_string(),
                    "business" => "Business".magenta().bold().to_string(),
                    "enterprise" => "Enterprise".yellow().bold().to_string(),
                    _ => plan_name.to_string(),
                };
                println!("  {} {}", "Plan:".dimmed(), plan_display);

                if let Some(price) = plan_info["price"].as_f64() {
                    if price > 0.0 {
                        println!("  {} ${:.0}/month", "Price:".dimmed(), price);
                    } else {
                        println!("  {} Free", "Price:".dimmed());
                    }
                }

                println!();
                println!("  {}", "Limits".green());

                if let Some(robots) = plan_info["limits"]["robots"].as_u64() {
                    let display = if robots == 0 {
                        "Unlimited".to_string()
                    } else {
                        robots.to_string()
                    };
                    println!("    {} {}", "Robots:".dimmed(), display);
                }

                if let Some(events) = plan_info["limits"]["telemetry_events"].as_u64() {
                    let display = if events == 0 {
                        "Unlimited".to_string()
                    } else {
                        format_number(events)
                    };
                    println!("    {} {}/month", "Telemetry:".dimmed(), display);
                }

                if let Some(retention) = plan_info["limits"]["data_retention_days"].as_u64() {
                    println!("    {} {} days", "Data Retention:".dimmed(), retention);
                }

                if let Some(users) = plan_info["limits"]["users"].as_u64() {
                    let display = if users == 0 {
                        "Unlimited".to_string()
                    } else {
                        users.to_string()
                    };
                    println!("    {} {}", "Users:".dimmed(), display);
                }

                println!();

                // Show features
                if let Some(features) = plan_info["features"].as_array() {
                    println!("  {}", "Features".green());
                    for feature in features {
                        if let Some(name) = feature.as_str() {
                            println!("    {} {}", "✓".green(), name);
                        }
                    }
                    println!();
                }

                println!(
                    "{} Upgrade or manage plan at: {}/billing",
                    "Tip:".yellow(),
                    cloud_url.cyan()
                );
            }
        }
        _ => {
            // Fallback - show generic plan info
            println!("{}", "Current Plan".cyan().bold());
            println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

            let plan_name = config.plan.as_deref().unwrap_or("free");
            println!("  {} {}", "Plan:".dimmed(), plan_name);
            println!();
            println!(
                "{} View plan details at: {}/billing",
                "Tip:".yellow(),
                cloud_url.cyan()
            );
        }
    }

    Ok(())
}

/// List API keys
pub fn keys_list() -> HorusResult<()> {
    let config_path = auth_config_path().map_err(|e| HorusError::Config(e.to_string()))?;

    if !config_path.exists() {
        println!("{} Not logged in", "!".yellow());
        return Ok(());
    }

    let config_content = fs::read_to_string(&config_path)
        .map_err(|e| HorusError::Config(format!("Failed to read auth config: {}", e)))?;

    let config: AuthConfig = serde_json::from_str(&config_content)
        .map_err(|e| HorusError::Config(format!("Failed to parse auth config: {}", e)))?;

    // Try registry first, then cloud
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
            println!("{} Could not fetch API keys", "!".yellow());
            println!();
            println!("Manage keys at: {}/dashboard/keys", registry_url.cyan());
        }
    }

    Ok(())
}

/// Revoke an API key
pub fn keys_revoke(key_id: &str) -> HorusResult<()> {
    let config_path = auth_config_path().map_err(|e| HorusError::Config(e.to_string()))?;

    if !config_path.exists() {
        println!("{} Not logged in", "!".yellow());
        return Ok(());
    }

    let config_content = fs::read_to_string(&config_path)
        .map_err(|e| HorusError::Config(format!("Failed to read auth config: {}", e)))?;

    let config: AuthConfig = serde_json::from_str(&config_content)
        .map_err(|e| HorusError::Config(format!("Failed to parse auth config: {}", e)))?;

    let registry_url = config.registry_url.clone();
    let client = reqwest::blocking::Client::new();

    match client
        .delete(format!("{}/api/auth/keys/{}", registry_url, key_id))
        .header("Authorization", format!("Bearer {}", config.api_key))
        .send()
    {
        Ok(response) if response.status().is_success() => {
            println!("{} API key revoked successfully", "✓".green());
        }
        Ok(response) if response.status() == reqwest::StatusCode::NOT_FOUND => {
            println!("{} API key not found", "!".yellow());
        }
        _ => {
            println!("{} Could not revoke API key", "!".red());
            println!();
            println!("Try revoking at: {}/dashboard/keys", registry_url.cyan());
        }
    }

    Ok(())
}

// Helper function to create a usage bar
fn usage_bar(percent: u64) -> String {
    let filled = (percent / 10) as usize;
    let empty = 10 - filled;

    let bar_char = if percent >= 90 {
        "█".red()
    } else if percent >= 70 {
        "█".yellow()
    } else {
        "█".green()
    };

    format!(
        "[{}{}] {}%",
        bar_char.to_string().repeat(filled),
        "░".dimmed().to_string().repeat(empty),
        percent
    )
}

// Helper function to format large numbers
fn format_number(n: u64) -> String {
    if n >= 1_000_000_000 {
        format!("{:.1}B", n as f64 / 1_000_000_000.0)
    } else if n >= 1_000_000 {
        format!("{:.1}M", n as f64 / 1_000_000.0)
    } else if n >= 1_000 {
        format!("{:.1}K", n as f64 / 1_000.0)
    } else {
        n.to_string()
    }
}
