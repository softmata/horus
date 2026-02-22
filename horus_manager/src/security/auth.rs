//! Password-based authentication for HORUS monitor
//!
//! Security through password authentication with Argon2 hashing and session tokens.

use anyhow::{Context, Result};
use argon2::{
    password_hash::{rand_core::OsRng, PasswordHash, PasswordHasher, PasswordVerifier, SaltString},
    Argon2,
};
use rand::Rng;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

/// Authentication service with password-based authentication
pub struct AuthService {
    password_hash: String,
    sessions: Arc<RwLock<HashMap<String, SessionInfo>>>,
    rate_limiter: Arc<RwLock<RateLimiter>>,
}

/// Session information
#[derive(Clone)]
struct SessionInfo {
    _created_at: Instant,
    last_used: Instant,
    _ip_address: Option<String>,
}

/// Rate limiter for login attempts
struct RateLimiter {
    attempts: HashMap<String, Vec<Instant>>,
    max_attempts: usize,
    window: Duration,
}

impl AuthService {
    /// Create authentication service with existing password hash
    pub fn new(password_hash: String) -> Result<Self> {
        Ok(Self {
            password_hash,
            sessions: Arc::new(RwLock::new(HashMap::new())),
            rate_limiter: Arc::new(RwLock::new(RateLimiter::new(
                crate::config::AUTH_MAX_ATTEMPTS,
                Duration::from_secs(crate::config::AUTH_RATE_LIMIT_WINDOW_SECS),
            ))),
        })
    }

    /// Verify password and create session token
    pub fn login(&self, password: &str, ip_address: Option<String>) -> Result<Option<String>> {
        // Check rate limiting
        if let Some(ip) = &ip_address {
            if !self.rate_limiter.write().expect("rate_limiter lock poisoned").check_attempt(ip) {
                anyhow::bail!("Too many login attempts. Please wait a minute.");
            }
        }

        // Verify password
        let parsed_hash = PasswordHash::new(&self.password_hash)
            .map_err(|e| anyhow::anyhow!("Invalid stored password hash: {}", e))?;

        if Argon2::default()
            .verify_password(password.as_bytes(), &parsed_hash)
            .is_ok()
        {
            // Password correct - generate session token
            let session_token = generate_session_token();

            // Store session
            self.sessions.write().expect("sessions lock poisoned").insert(
                session_token.clone(),
                SessionInfo {
                    _created_at: Instant::now(),
                    last_used: Instant::now(),
                    _ip_address: ip_address,
                },
            );

            Ok(Some(session_token))
        } else {
            Ok(None)
        }
    }

    /// Validate session token
    pub fn validate_session(&self, token: &str) -> bool {
        let mut sessions = self.sessions.write().expect("sessions lock poisoned");

        if let Some(session) = sessions.get_mut(token) {
            // Check if session is expired (1 hour of inactivity)
            if session.last_used.elapsed() > Duration::from_secs(crate::config::SESSION_TIMEOUT_SECS) {
                sessions.remove(token);
                return false;
            }

            // Update last used time
            session.last_used = Instant::now();
            true
        } else {
            false
        }
    }

    /// Logout - invalidate session token
    pub fn logout(&self, token: &str) {
        self.sessions.write().expect("sessions lock poisoned").remove(token);
    }
}

impl RateLimiter {
    fn new(max_attempts: usize, window: Duration) -> Self {
        Self {
            attempts: HashMap::new(),
            max_attempts,
            window,
        }
    }

    fn check_attempt(&mut self, ip: &str) -> bool {
        let now = Instant::now();

        // Clean old attempts
        let entry = self.attempts.entry(ip.to_string()).or_default();
        entry.retain(|&timestamp| now.duration_since(timestamp) < self.window);

        // Check if under limit
        if entry.len() >= self.max_attempts {
            return false;
        }

        // Record this attempt
        entry.push(now);
        true
    }
}

/// Generate a cryptographically secure session token
fn generate_session_token() -> String {
    let mut rng = rand::thread_rng();
    let random_bytes: Vec<u8> = (0..32).map(|_| rng.gen()).collect();

    // Convert to base64 URL-safe encoding
    use base64::Engine;
    base64::engine::general_purpose::URL_SAFE_NO_PAD.encode(&random_bytes)
}

/// Hash a password using Argon2
pub fn hash_password(password: &str) -> Result<String> {
    let salt = SaltString::generate(&mut OsRng);
    let argon2 = Argon2::default();

    argon2
        .hash_password(password.as_bytes(), &salt)
        .map(|hash| hash.to_string())
        .map_err(|e| anyhow::anyhow!("Failed to hash password: {}", e))
}

/// Verify a password against a hash
pub fn verify_password(password: &str, hash: &str) -> Result<bool> {
    let parsed_hash = PasswordHash::new(hash)
        .map_err(|e| anyhow::anyhow!("Invalid password hash format: {}", e))?;

    Ok(Argon2::default()
        .verify_password(password.as_bytes(), &parsed_hash)
        .is_ok())
}

/// Get the path to the password hash file
pub fn get_password_file_path() -> Result<PathBuf> {
    let horus_dir = dirs::home_dir()
        .context("Could not find home directory")?
        .join(".horus");

    std::fs::create_dir_all(&horus_dir).context("Failed to create .horus directory")?;

    Ok(horus_dir.join("dashboard_password.hash"))
}

/// Load password hash from file
pub fn load_password_hash() -> Result<String> {
    let path = get_password_file_path()?;
    std::fs::read_to_string(&path).context("Failed to read password hash file")
}

/// Save password hash to file
pub fn save_password_hash(hash: &str) -> Result<()> {
    let path = get_password_file_path()?;
    std::fs::write(&path, hash).context("Failed to write password hash file")?;

    // Restrict permissions to owner-only (contains password hash)
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let _ = std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o600));
    }

    Ok(())
}

/// Check if password has been set up
pub fn is_password_configured() -> bool {
    get_password_file_path()
        .map(|path| path.exists())
        .unwrap_or(false)
}

/// Prompt user to set up password (for CLI)
pub fn prompt_for_password_setup() -> Result<String> {
    use colored::Colorize;
    use std::io::{self, Write};

    println!(
        "\n{} HORUS Monitor - First Time Setup",
        "[SECURITY]".cyan().bold()
    );
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    println!("Set a monitor password (or press Enter for no password):");
    println!(
        "{} Without a password, anyone on your network can access the monitor",
        "[NOTE]".yellow()
    );

    loop {
        print!("Password: ");
        io::stdout().flush()?;
        let password = rpassword::read_password()?;

        // Allow empty password (no authentication)
        if password.is_empty() {
            println!(
                "{} No password set - monitor will be accessible without login",
                "[WARNING]".yellow().bold()
            );
            println!(
                "{} You can add a password later with: {}",
                "[TIP]".cyan(),
                "horus monitor -r".bright_blue()
            );
            println!();

            // Save empty hash to indicate no password
            save_password_hash("")?;
            return Ok(String::new());
        }

        if password.len() < 8 {
            println!(
                "{} Password must be at least 8 characters. Please try again.",
                "[ERROR]".red().bold()
            );
            println!("{} Or press Enter for no password", "[TIP]".cyan());
            continue;
        }

        print!("Confirm password: ");
        io::stdout().flush()?;
        let confirm = rpassword::read_password()?;

        if password != confirm {
            println!(
                "{} Passwords don't match. Please try again.",
                "[ERROR]".red().bold()
            );
            continue;
        }

        // Hash and save password
        let hash = hash_password(&password)?;
        save_password_hash(&hash)?;

        println!("{} Password set successfully!", "[SUCCESS]".green().bold());
        println!();

        return Ok(hash);
    }
}

/// Prompt user for password (for CLI login)
pub fn prompt_for_password() -> Result<String> {
    use std::io::{self, Write};

    print!("Monitor password: ");
    io::stdout().flush()?;
    Ok(rpassword::read_password()?)
}

/// Prompt user to reset password
pub fn reset_password() -> Result<String> {
    use colored::Colorize;
    use std::io::{self, Write};

    println!(
        "\n{} HORUS Monitor - Password Reset",
        "[SECURITY]".cyan().bold()
    );
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    println!("Enter a new monitor password (or press Enter to disable password):");
    println!(
        "{} Without a password, anyone on your network can access the monitor",
        "[NOTE]".yellow()
    );

    loop {
        print!("New password: ");
        io::stdout().flush()?;
        let password = rpassword::read_password()?;

        // Allow empty password (no authentication)
        if password.is_empty() {
            println!(
                "{} Password removed - monitor will be accessible without login",
                "[WARNING]".yellow().bold()
            );
            println!();

            // Save empty hash to indicate no password
            save_password_hash("")?;
            return Ok(String::new());
        }

        if password.len() < 8 {
            println!(
                "{} Password must be at least 8 characters. Please try again.",
                "[ERROR]".red().bold()
            );
            println!("{} Or press Enter to disable password", "[TIP]".cyan());
            continue;
        }

        print!("Confirm password: ");
        io::stdout().flush()?;
        let confirm = rpassword::read_password()?;

        if password != confirm {
            println!(
                "{} Passwords don't match. Please try again.",
                "[ERROR]".red().bold()
            );
            continue;
        }

        // Hash and save password
        let hash = hash_password(&password)?;
        save_password_hash(&hash)?;

        println!(
            "{} Password reset successfully!",
            "[SUCCESS]".green().bold()
        );
        println!();

        return Ok(hash);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_password_hashing() {
        let password = "SecurePassword123";
        let hash = hash_password(password).unwrap();

        assert!(verify_password(password, &hash).unwrap());
        assert!(!verify_password("WrongPassword", &hash).unwrap());
    }

    #[test]
    fn test_session_generation() {
        let token1 = generate_session_token();
        let token2 = generate_session_token();

        assert_ne!(token1, token2);
        assert!(token1.len() > 32);
    }

    #[test]
    fn test_session_validation() {
        let hash = hash_password("testpass").unwrap();
        let auth = AuthService::new(hash).unwrap();

        let token = auth.login("testpass", None).unwrap().unwrap();
        assert!(auth.validate_session(&token));
        assert!(!auth.validate_session("invalid_token"));
    }

    #[test]
    fn test_rate_limiting() {
        let hash = hash_password("testpass").unwrap();
        let auth = AuthService::new(hash).unwrap();

        // Make 5 failed login attempts (should succeed)
        for _ in 0..5 {
            let _ = auth.login("wrongpass", Some("127.0.0.1".to_string()));
        }

        // 6th attempt should be rate limited
        let result = auth.login("testpass", Some("127.0.0.1".to_string()));
        assert!(result.is_err());
    }

    #[test]
    fn test_session_expiration() {
        let hash = hash_password("testpass").unwrap();
        let auth = AuthService::new(hash).unwrap();

        let token = auth.login("testpass", None).unwrap().unwrap();
        assert!(auth.validate_session(&token));

        // Manually expire the session
        {
            let mut sessions = auth.sessions.write().unwrap();
            if let Some(session) = sessions.get_mut(&token) {
                session.last_used = Instant::now() - Duration::from_secs(3601);
            }
        }

        assert!(!auth.validate_session(&token));
    }

    #[test]
    fn test_logout() {
        let hash = hash_password("testpass").unwrap();
        let auth = AuthService::new(hash).unwrap();

        let token = auth.login("testpass", None).unwrap().unwrap();
        assert!(auth.validate_session(&token));

        auth.logout(&token);
        assert!(!auth.validate_session(&token));
    }
}
