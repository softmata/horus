//! Common path utilities for HORUS directory structure.
//!
//! Centralizes all `.horus` directory path construction to avoid
//! duplicating `dirs::home_dir().ok_or_else(...)?.join(".horus/...")` everywhere.

use anyhow::{anyhow, Result};
use std::path::PathBuf;

/// Get the user's home directory or return an error.
pub fn home_dir() -> Result<PathBuf> {
    dirs::home_dir().ok_or_else(|| anyhow!("could not find home directory"))
}

/// Get `~/.horus` — the global HORUS config directory.
pub fn horus_dir() -> Result<PathBuf> {
    let dir = home_dir()?.join(".horus");
    log::debug!("horus dir: {:?}", dir);
    Ok(dir)
}

/// Get `~/.horus/cache` — the global package cache directory.
pub fn cache_dir() -> Result<PathBuf> {
    Ok(home_dir()?.join(".horus/cache"))
}

/// Get `~/.horus/recordings` — the recordings directory.
pub fn recordings_dir() -> Result<PathBuf> {
    Ok(home_dir()?.join(".horus/recordings"))
}

/// Get `~/.horus/keys` — the signing keys directory.
pub fn keys_dir() -> Result<PathBuf> {
    Ok(home_dir()?.join(".horus/keys"))
}

/// Get `~/.horus/auth.json` — the authentication config file.
pub fn auth_config_path() -> Result<PathBuf> {
    Ok(home_dir()?.join(".horus/auth.json"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_home_dir_returns_absolute_path() {
        let path = home_dir().unwrap();
        assert!(path.is_absolute());
    }

    #[test]
    fn test_horus_dir_ends_with_dot_horus() {
        let path = horus_dir().unwrap();
        assert!(path.ends_with(".horus"));
    }

    #[test]
    fn test_cache_dir_ends_with_cache() {
        let path = cache_dir().unwrap();
        assert!(path.ends_with(".horus/cache"));
    }

    #[test]
    fn test_recordings_dir_ends_with_recordings() {
        let path = recordings_dir().unwrap();
        assert!(path.ends_with(".horus/recordings"));
    }

    #[test]
    fn test_keys_dir_ends_with_keys() {
        let path = keys_dir().unwrap();
        assert!(path.ends_with(".horus/keys"));
    }

    #[test]
    fn test_auth_config_path_ends_with_auth_json() {
        let path = auth_config_path().unwrap();
        assert!(path.ends_with(".horus/auth.json"));
    }
}
