//! Common path utilities for HORUS directory structure.
//!
//! Delegates to [`horus_sys::platform`] for cross-platform directory resolution.
//! All paths are platform-appropriate:
//! - Linux: `~/.config/horus/`, `~/.cache/horus/`, `~/.local/share/horus/`
//! - macOS: `~/Library/Application Support/horus/`, `~/Library/Caches/horus/`
//! - Windows: `%APPDATA%\horus\`, `%LOCALAPPDATA%\horus\`

use anyhow::{anyhow, Result};
use std::path::PathBuf;

/// Get the user's home directory or return an error.
pub fn home_dir() -> Result<PathBuf> {
    dirs::home_dir().ok_or_else(|| anyhow!("could not find home directory"))
}

/// Get the global HORUS config directory (platform-appropriate).
pub fn horus_dir() -> Result<PathBuf> {
    let dir = horus_sys::platform::config_dir();
    log::debug!("horus dir: {:?}", dir);
    Ok(dir)
}

/// Get the global package cache directory (platform-appropriate).
pub fn cache_dir() -> Result<PathBuf> {
    Ok(horus_sys::platform::cache_dir())
}

/// Get the recordings directory.
pub fn recordings_dir() -> Result<PathBuf> {
    Ok(horus_sys::platform::data_dir().join("recordings"))
}

/// Get the blackbox directory — tries `.horus/blackbox/` in CWD first, falls back to data dir.
pub fn blackbox_dir() -> Result<PathBuf> {
    let local = std::path::Path::new(".horus/blackbox");
    if local.is_dir() {
        return Ok(local.to_path_buf());
    }
    Ok(horus_sys::platform::data_dir().join("blackbox"))
}

/// Get the signing keys directory.
pub fn keys_dir() -> Result<PathBuf> {
    Ok(horus_sys::platform::data_dir().join("keys"))
}

/// Get the authentication config file path.
pub fn auth_config_path() -> Result<PathBuf> {
    Ok(horus_sys::platform::config_dir().join("auth.json"))
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Basic contract: all paths work and are absolute ──────────────

    #[test]
    fn test_home_dir_returns_absolute_path() {
        let path = home_dir().unwrap();
        assert!(path.is_absolute());
    }

    #[test]
    fn test_home_dir_exists_on_disk() {
        let path = home_dir().unwrap();
        assert!(path.exists());
        assert!(path.is_dir());
    }

    #[test]
    fn test_home_dir_consistent_across_calls() {
        assert_eq!(home_dir().unwrap(), home_dir().unwrap());
    }

    #[test]
    fn test_horus_dir_is_absolute() {
        assert!(horus_dir().unwrap().is_absolute());
    }

    #[test]
    fn test_cache_dir_is_absolute() {
        assert!(cache_dir().unwrap().is_absolute());
    }

    #[test]
    fn test_recordings_dir_is_absolute() {
        assert!(recordings_dir().unwrap().is_absolute());
    }

    #[test]
    fn test_keys_dir_is_absolute() {
        assert!(keys_dir().unwrap().is_absolute());
    }

    #[test]
    fn test_auth_config_path_is_absolute() {
        assert!(auth_config_path().unwrap().is_absolute());
    }

    // ── Path structure: all contain "horus" and correct leaf names ───

    #[test]
    fn test_horus_dir_contains_horus() {
        let path = horus_dir().unwrap();
        assert!(
            path.to_string_lossy().contains("horus"),
            "horus_dir should contain 'horus': {:?}",
            path
        );
    }

    #[test]
    fn test_horus_dir_file_name() {
        // XDG: ~/.config/horus → file_name = "horus"
        let path = horus_dir().unwrap();
        assert_eq!(path.file_name().unwrap(), "horus");
    }

    #[test]
    fn test_cache_dir_contains_horus() {
        let path = cache_dir().unwrap();
        assert!(path.to_string_lossy().contains("horus"));
    }

    #[test]
    fn test_cache_dir_file_name() {
        // XDG: ~/.cache/horus → file_name = "horus"
        let path = cache_dir().unwrap();
        assert_eq!(path.file_name().unwrap(), "horus");
    }

    #[test]
    fn test_recordings_dir_file_name() {
        let path = recordings_dir().unwrap();
        assert_eq!(path.file_name().unwrap(), "recordings");
    }

    #[test]
    fn test_keys_dir_file_name() {
        let path = keys_dir().unwrap();
        assert_eq!(path.file_name().unwrap(), "keys");
    }

    #[test]
    fn test_auth_config_path_file_name() {
        assert_eq!(
            auth_config_path().unwrap().file_name().unwrap(),
            "auth.json"
        );
    }

    #[test]
    fn test_auth_config_path_extension() {
        assert_eq!(auth_config_path().unwrap().extension().unwrap(), "json");
    }

    // ── Parent relationships ────────────────────────────────────────

    #[test]
    fn test_auth_config_parent_is_horus_dir() {
        let auth = auth_config_path().unwrap();
        let horus = horus_dir().unwrap();
        assert_eq!(auth.parent().unwrap(), horus);
    }

    #[test]
    fn test_recordings_parent_is_data_dir() {
        // recordings_dir = data_dir / "recordings"
        let rec = recordings_dir().unwrap();
        let data = horus_sys::platform::data_dir();
        assert_eq!(rec.parent().unwrap(), data);
    }

    #[test]
    fn test_keys_parent_is_data_dir() {
        let keys = keys_dir().unwrap();
        let data = horus_sys::platform::data_dir();
        assert_eq!(keys.parent().unwrap(), data);
    }

    // ── Blackbox (local .horus/blackbox fallback) ───────────────────

    #[test]
    fn test_blackbox_dir_is_absolute_or_relative() {
        let path = blackbox_dir().unwrap();
        if path.is_absolute() {
            assert!(
                path.to_string_lossy().contains("blackbox"),
                "absolute blackbox dir should contain 'blackbox': {:?}",
                path
            );
        } else {
            assert_eq!(path, std::path::PathBuf::from(".horus/blackbox"));
        }
    }

    #[test]
    fn test_blackbox_dir_fallback_is_absolute() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let path = blackbox_dir().unwrap();
        assert!(path.is_absolute(), "fallback should be absolute");
        assert!(path.to_string_lossy().contains("blackbox"));

        std::env::set_current_dir(original).unwrap();
    }

    #[test]
    fn test_blackbox_dir_uses_local_when_exists() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::fs::create_dir_all(tmp.path().join(".horus/blackbox")).unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let path = blackbox_dir().unwrap();
        assert_eq!(path, std::path::PathBuf::from(".horus/blackbox"));

        std::env::set_current_dir(original).unwrap();
    }

    // ── All paths under home ────────────────────────────────────────

    #[test]
    fn test_all_paths_share_home_prefix() {
        let home = home_dir().unwrap();
        let paths = vec![
            horus_dir().unwrap(),
            cache_dir().unwrap(),
            recordings_dir().unwrap(),
            keys_dir().unwrap(),
            auth_config_path().unwrap(),
        ];
        for p in &paths {
            assert!(
                p.starts_with(&home),
                "{:?} should start with home dir {:?}",
                p,
                home
            );
        }
    }

    // ── All paths are distinct ──────────────────────────────────────

    #[test]
    fn test_all_leaf_paths_are_distinct() {
        let paths = vec![
            cache_dir().unwrap(),
            recordings_dir().unwrap(),
            keys_dir().unwrap(),
            auth_config_path().unwrap(),
        ];
        for i in 0..paths.len() {
            for j in (i + 1)..paths.len() {
                assert_ne!(paths[i], paths[j]);
            }
        }
    }
}
