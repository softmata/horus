//! Platform Module Behavioral Parity Tests
//!
//! Verifies OS detection, directories, disk space, hostname, shell
//! produce identical observable behavior on all platforms.

use horus_sys::platform;

// ═══════════════════════════════════════════════════════════════════════════
// OS Detection
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_detect_returns_valid_os() {
    let info = platform::detect();
    // OS should match compile-time target
    match info.os {
        platform::Os::Linux => assert!(cfg!(target_os = "linux")),
        platform::Os::MacOS => assert!(cfg!(target_os = "macos")),
        platform::Os::Windows => assert!(cfg!(target_os = "windows")),
    }
}

#[test]
fn test_detect_returns_valid_arch() {
    let info = platform::detect();
    match info.arch {
        platform::Arch::X86_64 => assert!(cfg!(target_arch = "x86_64")),
        platform::Arch::Aarch64 => assert!(cfg!(target_arch = "aarch64")),
        _ => {} // Other arches are acceptable
    }
}

#[test]
fn test_detect_is_cached() {
    let info1 = platform::detect();
    let info2 = platform::detect();
    // Same pointer — OnceLock returns &'static
    assert!(
        std::ptr::eq(info1, info2),
        "detect() should return cached value"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Standard Directories
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_config_dir_is_absolute() {
    assert!(platform::config_dir().is_absolute());
}

#[test]
fn test_cache_dir_is_absolute() {
    assert!(platform::cache_dir().is_absolute());
}

#[test]
fn test_data_dir_is_absolute() {
    assert!(platform::data_dir().is_absolute());
}

#[test]
fn test_temp_dir_is_absolute() {
    assert!(platform::temp_dir().is_absolute());
}

#[test]
fn test_standard_dirs_contain_horus() {
    assert!(platform::config_dir().to_string_lossy().contains("horus"));
    assert!(platform::cache_dir().to_string_lossy().contains("horus"));
    assert!(platform::data_dir().to_string_lossy().contains("horus"));
    assert!(platform::temp_dir().to_string_lossy().contains("horus"));
}

#[test]
fn test_standard_dirs_are_distinct() {
    let config = platform::config_dir();
    let cache = platform::cache_dir();
    let temp = platform::temp_dir();
    assert_ne!(config, cache, "config and cache should differ");
    assert_ne!(config, temp, "config and temp should differ");
    assert_ne!(cache, temp, "cache and temp should differ");
}

// ═══════════════════════════════════════════════════════════════════════════
// Disk Space
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_disk_available_mb_returns_some() {
    let result = platform::disk_available_mb(&std::env::temp_dir());
    assert!(
        result.is_some(),
        "temp_dir should have measurable space on all platforms"
    );
}

#[test]
fn test_disk_available_mb_is_positive() {
    if let Some(mb) = platform::disk_available_mb(&std::env::temp_dir()) {
        assert!(mb > 0, "available space should be positive");
    }
}

#[test]
fn test_disk_available_mb_nonexistent_returns_none() {
    let result = platform::disk_available_mb(std::path::Path::new("/nonexistent/path/xyz"));
    assert!(result.is_none(), "nonexistent path should return None");
}

// ═══════════════════════════════════════════════════════════════════════════
// Hostname
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_hostname_is_nonempty() {
    let h = platform::hostname();
    assert!(!h.is_empty(), "hostname must not be empty on any platform");
}

#[test]
fn test_hostname_is_stable() {
    let h1 = platform::hostname();
    let h2 = platform::hostname();
    assert_eq!(h1, h2, "hostname must be stable across calls");
}

// ═══════════════════════════════════════════════════════════════════════════
// Shell Detection
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_default_shell_is_nonempty() {
    let shell = platform::default_shell();
    assert!(
        !shell.as_os_str().is_empty(),
        "default shell should be non-empty on all platforms"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Package Manager Suggestions
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_suggest_install_contains_package_name() {
    let cmd = platform::suggest_install("libssl");
    assert!(
        cmd.contains("libssl"),
        "install suggestion should contain package name"
    );
}

#[test]
fn test_suggest_build_tools_nonempty() {
    let cmd = platform::suggest_build_tools();
    assert!(
        !cmd.is_empty(),
        "build tools suggestion should be non-empty"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Terminal Colors
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_init_terminal_colors_does_not_panic() {
    let _ = platform::init_terminal_colors();
}
