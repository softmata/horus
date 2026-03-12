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

/// Get the blackbox directory — tries `.horus/blackbox/` in CWD first, falls back to home dir.
pub fn blackbox_dir() -> Result<PathBuf> {
    let local = std::path::Path::new(".horus/blackbox");
    if local.is_dir() {
        return Ok(local.to_path_buf());
    }
    Ok(home_dir()?.join(".horus/blackbox"))
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

    // --- home_dir deeper tests ---

    #[test]
    fn test_home_dir_exists_on_disk() {
        let path = home_dir().unwrap();
        assert!(path.exists(), "home directory should exist");
        assert!(path.is_dir(), "home directory should be a directory");
    }

    #[test]
    fn test_home_dir_consistent_across_calls() {
        let p1 = home_dir().unwrap();
        let p2 = home_dir().unwrap();
        assert_eq!(p1, p2, "home_dir() should return the same path on repeated calls");
    }

    // --- horus_dir deeper tests ---

    #[test]
    fn test_horus_dir_is_absolute() {
        let path = horus_dir().unwrap();
        assert!(path.is_absolute());
    }

    #[test]
    fn test_horus_dir_parent_is_home() {
        let horus = horus_dir().unwrap();
        let home = home_dir().unwrap();
        assert_eq!(horus.parent().unwrap(), home);
    }

    #[test]
    fn test_horus_dir_file_name() {
        let path = horus_dir().unwrap();
        assert_eq!(path.file_name().unwrap(), ".horus");
    }

    // --- cache_dir deeper tests ---

    #[test]
    fn test_cache_dir_is_absolute() {
        let path = cache_dir().unwrap();
        assert!(path.is_absolute());
    }

    #[test]
    fn test_cache_dir_parent_is_horus_dir() {
        let cache = cache_dir().unwrap();
        let horus = horus_dir().unwrap();
        assert_eq!(cache.parent().unwrap(), horus);
    }

    #[test]
    fn test_cache_dir_file_name() {
        let path = cache_dir().unwrap();
        assert_eq!(path.file_name().unwrap(), "cache");
    }

    // --- recordings_dir deeper tests ---

    #[test]
    fn test_recordings_dir_is_absolute() {
        let path = recordings_dir().unwrap();
        assert!(path.is_absolute());
    }

    #[test]
    fn test_recordings_dir_parent_is_horus_dir() {
        let rec = recordings_dir().unwrap();
        let horus = horus_dir().unwrap();
        assert_eq!(rec.parent().unwrap(), horus);
    }

    #[test]
    fn test_recordings_dir_file_name() {
        let path = recordings_dir().unwrap();
        assert_eq!(path.file_name().unwrap(), "recordings");
    }

    // --- keys_dir deeper tests ---

    #[test]
    fn test_keys_dir_is_absolute() {
        let path = keys_dir().unwrap();
        assert!(path.is_absolute());
    }

    #[test]
    fn test_keys_dir_parent_is_horus_dir() {
        let keys = keys_dir().unwrap();
        let horus = horus_dir().unwrap();
        assert_eq!(keys.parent().unwrap(), horus);
    }

    #[test]
    fn test_keys_dir_file_name() {
        let path = keys_dir().unwrap();
        assert_eq!(path.file_name().unwrap(), "keys");
    }

    // --- auth_config_path deeper tests ---

    #[test]
    fn test_auth_config_path_is_absolute() {
        let path = auth_config_path().unwrap();
        assert!(path.is_absolute());
    }

    #[test]
    fn test_auth_config_path_parent_is_horus_dir() {
        let auth = auth_config_path().unwrap();
        let horus = horus_dir().unwrap();
        assert_eq!(auth.parent().unwrap(), horus);
    }

    #[test]
    fn test_auth_config_path_file_name() {
        let path = auth_config_path().unwrap();
        assert_eq!(path.file_name().unwrap(), "auth.json");
    }

    #[test]
    fn test_auth_config_path_extension() {
        let path = auth_config_path().unwrap();
        assert_eq!(path.extension().unwrap(), "json");
    }

    // --- blackbox_dir tests ---

    #[test]
    fn test_blackbox_dir_is_absolute_or_relative() {
        // blackbox_dir returns a relative path if .horus/blackbox exists in CWD,
        // otherwise returns an absolute path under home
        let path = blackbox_dir().unwrap();
        // Either it's the local relative path or an absolute fallback
        if path.is_absolute() {
            assert!(path.ends_with(".horus/blackbox"));
        } else {
            assert_eq!(path, std::path::PathBuf::from(".horus/blackbox"));
        }
    }

    #[test]
    fn test_blackbox_dir_fallback_ends_with_blackbox() {
        // When run from a directory without .horus/blackbox, should fall back to home
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let path = blackbox_dir().unwrap();
        // No .horus/blackbox in tmp, so should fall back
        assert!(path.is_absolute(), "fallback should be absolute");
        assert!(path.ends_with(".horus/blackbox"));

        std::env::set_current_dir(original).unwrap();
    }

    #[test]
    fn test_blackbox_dir_uses_local_when_exists() {
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap();

        // Create .horus/blackbox in the temp dir
        std::fs::create_dir_all(tmp.path().join(".horus/blackbox")).unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let path = blackbox_dir().unwrap();
        assert_eq!(path, std::path::PathBuf::from(".horus/blackbox"));

        std::env::set_current_dir(original).unwrap();
    }

    // --- All paths are under home ---

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

    // --- All paths are distinct ---

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
                assert_ne!(paths[i], paths[j], "{:?} and {:?} should differ", paths[i], paths[j]);
            }
        }
    }

    // --- Path depth / structure ---

    #[test]
    fn test_horus_dir_is_exactly_two_levels_from_root() {
        // home is e.g. /home/user, horus_dir is /home/user/.horus
        let horus = horus_dir().unwrap();
        let home = home_dir().unwrap();
        // horus_dir should have exactly one more component than home
        assert_eq!(
            horus.components().count(),
            home.components().count() + 1
        );
    }

    #[test]
    fn test_subdirs_are_three_levels_from_root() {
        let home = home_dir().unwrap();
        let subdirs = vec![
            cache_dir().unwrap(),
            recordings_dir().unwrap(),
            keys_dir().unwrap(),
        ];
        for dir in &subdirs {
            assert_eq!(
                dir.components().count(),
                home.components().count() + 2,
                "{:?} should be 2 levels deeper than home",
                dir,
            );
        }
    }
}
