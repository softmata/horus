//! Password-based authentication for HORUS monitor
//!
//! Security through password authentication with Argon2 hashing and session tokens.

use anyhow::{anyhow, Context, Result};
use argon2::{
    password_hash::{
        rand_core::{OsRng, RngCore},
        PasswordHash, PasswordHasher, PasswordVerifier, SaltString,
    },
    Argon2,
};
use horus_core::core::DurationExt;
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
    /// When the session was first created.  Used for absolute timeout enforcement.
    created_at: Instant,
    last_used: Instant,
    _ip_address: Option<String>,
    /// CSRF token bound to this session.  The client must echo it in the
    /// `X-CSRF-Token` request header for every state-mutating request.
    csrf_token: String,
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
                crate::config::AUTH_RATE_LIMIT_WINDOW_SECS.secs(),
            ))),
        })
    }

    /// Verify password and create a session.
    ///
    /// Returns `Ok(Some((session_token, csrf_token)))` on success.
    /// The caller must deliver both tokens to the client:
    /// - `session_token`: stored in an `HttpOnly` cookie for automatic transport
    /// - `csrf_token`: returned in the JSON response body so the client can
    ///   echo it in the `X-CSRF-Token` header on every state-mutating request
    pub fn login(
        &self,
        password: &str,
        ip_address: Option<String>,
    ) -> Result<Option<(String, String)>> {
        // Check rate limiting
        if let Some(ip) = &ip_address {
            if !self
                .rate_limiter
                .write()
                .expect("rate_limiter lock poisoned")
                .check_attempt(ip)
            {
                anyhow::bail!("Too many login attempts. Please wait a minute.");
            }
        }

        // Verify password
        let parsed_hash = PasswordHash::new(&self.password_hash)
            .map_err(|e| anyhow!("Invalid stored password hash: {}", e))?;

        if Argon2::default()
            .verify_password(password.as_bytes(), &parsed_hash)
            .is_ok()
        {
            // Password correct — generate independent session and CSRF tokens,
            // both sourced from OS entropy (OsRng, not thread_rng).
            let session_token = generate_secure_token();
            let csrf_token = generate_secure_token();

            // Store session
            self.sessions
                .write()
                .expect("sessions lock poisoned")
                .insert(
                    session_token.clone(),
                    SessionInfo {
                        created_at: Instant::now(),
                        last_used: Instant::now(),
                        _ip_address: ip_address,
                        csrf_token: csrf_token.clone(),
                    },
                );

            Ok(Some((session_token, csrf_token)))
        } else {
            Ok(None)
        }
    }

    /// Validate session token.
    ///
    /// A session is rejected if:
    /// - It does not exist (already logged out or never created).
    /// - It has been idle longer than `SESSION_TIMEOUT_SECS` (1 hour).
    /// - It has existed longer than `SESSION_ABSOLUTE_TIMEOUT_SECS` (8 hours)
    ///   regardless of activity — prevents indefinite session hijacking.
    pub fn validate_session(&self, token: &str) -> bool {
        let mut sessions = self.sessions.write().expect("sessions lock poisoned");

        if let Some(session) = sessions.get_mut(token) {
            // Absolute session lifetime — expire regardless of activity.
            if session.created_at.elapsed() > crate::config::SESSION_ABSOLUTE_TIMEOUT_SECS.secs() {
                sessions.remove(token);
                return false;
            }

            // Idle timeout — expire after 1 hour of inactivity.
            if session.last_used.elapsed() > crate::config::SESSION_TIMEOUT_SECS.secs() {
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

    /// Validate a CSRF token against the session it was issued with.
    ///
    /// The comparison is done in constant time to prevent timing-based
    /// enumeration of valid CSRF tokens.
    ///
    /// Returns `false` if the session does not exist, has expired, or the
    /// provided CSRF token does not match the one issued at login.
    pub fn validate_csrf(&self, session_token: &str, csrf_token: &str) -> bool {
        let sessions = self.sessions.read().expect("sessions lock poisoned");
        if let Some(session) = sessions.get(session_token) {
            ct_eq(csrf_token.as_bytes(), session.csrf_token.as_bytes())
        } else {
            false
        }
    }

    /// Logout - invalidate session token
    pub fn logout(&self, token: &str) {
        self.sessions
            .write()
            .expect("sessions lock poisoned")
            .remove(token);
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

/// Generate a cryptographically secure 32-byte token from the OS entropy source.
///
/// Uses `OsRng` (backed by `getrandom`) rather than `thread_rng`, which is a
/// userspace PRNG that may be predictable from observable seed state.
fn generate_secure_token() -> String {
    let mut random_bytes = [0u8; 32];
    OsRng.fill_bytes(&mut random_bytes);
    use base64::Engine;
    base64::engine::general_purpose::URL_SAFE_NO_PAD.encode(random_bytes)
}

/// Constant-time byte comparison to prevent timing side-channels.
///
/// An attacker that can precisely measure how long comparison takes could
/// enumerate valid tokens one byte at a time.  This implementation XORs all
/// bytes and folds them to a single value so the runtime does not reveal which
/// byte first differed.
fn ct_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    a.iter()
        .zip(b.iter())
        .fold(0u8, |acc, (x, y)| acc | (x ^ y))
        == 0
}

/// Hash a password using Argon2
pub fn hash_password(password: &str) -> Result<String> {
    let salt = SaltString::generate(&mut OsRng);
    let argon2 = Argon2::default();

    argon2
        .hash_password(password.as_bytes(), &salt)
        .map(|hash| hash.to_string())
        .map_err(|e| anyhow!("Failed to hash password: {}", e))
}

/// Verify a password against a hash
#[cfg(test)]
fn verify_password(password: &str, hash: &str) -> Result<bool> {
    let parsed_hash =
        PasswordHash::new(hash).map_err(|e| anyhow!("Invalid password hash format: {}", e))?;

    Ok(Argon2::default()
        .verify_password(password.as_bytes(), &parsed_hash)
        .is_ok())
}

/// Get the path to the password hash file
pub fn get_password_file_path() -> Result<PathBuf> {
    let horus_dir = crate::paths::horus_dir()?;
    std::fs::create_dir_all(&horus_dir).context("failed to create .horus directory")?;
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

    // Create file with restricted permissions from the start to avoid TOCTOU race
    {
        use std::io::Write;
        let mut file =
            horus_sys::fs::open_private(&path).context("Failed to create password hash file")?;
        file.write_all(hash.as_bytes())
            .context("Failed to write password hash file")?;
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
///
/// An empty password (press Enter) is still accepted here to let the user
/// explicitly opt out of authentication; the caller (`monitor::run`) is
/// responsible for checking that the `--no-auth` flag was also provided
/// before allowing the monitor to start without a password.
pub fn prompt_for_password_setup() -> Result<String> {
    use colored::Colorize;
    use std::io::{self, Write};

    println!(
        "\n{} HORUS Monitor - First Time Setup",
        "[SECURITY]".cyan().bold()
    );
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    println!(
        "Set a monitor password (minimum {} characters, or press Enter for no password):",
        crate::config::MIN_PASSWORD_LENGTH
    );
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
                crate::cli_output::ICON_HINT.cyan(),
                "horus monitor -r".bright_blue()
            );
            println!();

            // Save empty hash to indicate no password
            save_password_hash("")?;
            return Ok(String::new());
        }

        if password.len() < crate::config::MIN_PASSWORD_LENGTH {
            println!(
                "{} Password must be at least {} characters. Please try again.",
                crate::cli_output::ICON_ERROR.red().bold(),
                crate::config::MIN_PASSWORD_LENGTH,
            );
            println!(
                "{} Or press Enter for no password",
                crate::cli_output::ICON_HINT.cyan()
            );
            continue;
        }

        print!("Confirm password: ");
        io::stdout().flush()?;
        let confirm = rpassword::read_password()?;

        if password != confirm {
            println!(
                "{} Passwords don't match. Please try again.",
                crate::cli_output::ICON_ERROR.red().bold()
            );
            continue;
        }

        // Hash and save password
        let hash = hash_password(&password)?;
        save_password_hash(&hash)?;

        println!(
            "{} Password set successfully!",
            crate::cli_output::ICON_SUCCESS.green().bold()
        );
        println!();

        return Ok(hash);
    }
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

        if password.len() < crate::config::MIN_PASSWORD_LENGTH {
            println!(
                "{} Password must be at least {} characters. Please try again.",
                crate::cli_output::ICON_ERROR.red().bold(),
                crate::config::MIN_PASSWORD_LENGTH,
            );
            println!(
                "{} Or press Enter to disable password",
                crate::cli_output::ICON_HINT.cyan()
            );
            continue;
        }

        print!("Confirm password: ");
        io::stdout().flush()?;
        let confirm = rpassword::read_password()?;

        if password != confirm {
            println!(
                "{} Passwords don't match. Please try again.",
                crate::cli_output::ICON_ERROR.red().bold()
            );
            continue;
        }

        // Hash and save password
        let hash = hash_password(&password)?;
        save_password_hash(&hash)?;

        println!(
            "{} Password reset successfully!",
            crate::cli_output::ICON_SUCCESS.green().bold()
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
        let token1 = generate_secure_token();
        let token2 = generate_secure_token();

        assert_ne!(token1, token2);
        // 32 bytes → 43 base64url chars (no padding)
        assert_eq!(token1.len(), 43);
        assert_eq!(token2.len(), 43);
    }

    #[test]
    fn test_session_validation() {
        let hash = hash_password("testpass").unwrap();
        let auth = AuthService::new(hash).unwrap();

        let (token, _csrf) = auth.login("testpass", None).unwrap().unwrap();
        assert!(auth.validate_session(&token));
        assert!(!auth.validate_session("invalid_token"));
    }

    #[test]
    fn test_csrf_validation() {
        let hash = hash_password("testpass").unwrap();
        let auth = AuthService::new(hash).unwrap();

        let (token, csrf) = auth.login("testpass", None).unwrap().unwrap();

        // Correct CSRF token must be accepted
        assert!(auth.validate_csrf(&token, &csrf));

        // Wrong CSRF token must be rejected
        assert!(!auth.validate_csrf(&token, "wrong_csrf_token"));

        // Unknown session must be rejected
        assert!(!auth.validate_csrf("bad_session", &csrf));
    }

    #[test]
    fn test_ct_eq() {
        assert!(ct_eq(b"hello", b"hello"));
        assert!(!ct_eq(b"hello", b"world"));
        assert!(!ct_eq(b"hello", b"hell"));
        assert!(!ct_eq(b"", b"x"));
        assert!(ct_eq(b"", b""));
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
        result.unwrap_err();
    }

    #[test]
    fn test_session_idle_expiration() {
        let hash = hash_password("testpass").unwrap();
        let auth = AuthService::new(hash).unwrap();

        let (token, _csrf) = auth.login("testpass", None).unwrap().unwrap();
        assert!(auth.validate_session(&token));

        // Manually expire the session by pushing last_used into the past
        {
            let mut sessions = auth.sessions.write().unwrap();
            if let Some(session) = sessions.get_mut(&token) {
                session.last_used = Instant::now() - 3601_u64.secs();
            }
        }

        assert!(!auth.validate_session(&token));
    }

    #[test]
    fn test_session_absolute_expiration() {
        let hash = hash_password("testpass").unwrap();
        let auth = AuthService::new(hash).unwrap();

        let (token, _csrf) = auth.login("testpass", None).unwrap().unwrap();
        assert!(auth.validate_session(&token));

        // Simulate a session older than 8 hours (both created_at and last_used
        // are kept fresh, but created_at is too old).
        {
            let mut sessions = auth.sessions.write().unwrap();
            if let Some(session) = sessions.get_mut(&token) {
                session.created_at =
                    Instant::now() - (crate::config::SESSION_ABSOLUTE_TIMEOUT_SECS + 1).secs();
            }
        }

        assert!(
            !auth.validate_session(&token),
            "session must be rejected after absolute timeout even if still active"
        );
    }

    #[test]
    fn test_logout() {
        let hash = hash_password("testpass").unwrap();
        let auth = AuthService::new(hash).unwrap();

        let (token, _csrf) = auth.login("testpass", None).unwrap().unwrap();
        assert!(auth.validate_session(&token));

        auth.logout(&token);
        assert!(!auth.validate_session(&token));
    }
}
