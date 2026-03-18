//! Filesystem Module Behavioral Parity Tests
//!
//! Verifies symlinks, file locking, permissions, dev_null
//! produce identical observable behavior on all platforms.

use horus_sys::fs;
use std::io::Write;

fn test_dir(name: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("horus_parity_fs_{}_{}", name, std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

// ═══════════════════════════════════════════════════════════════════════════
// Symlinks
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_symlink_create_and_read() {
    let dir = test_dir("symlink");
    let src = dir.join("source.txt");
    let dst = dir.join("link.txt");
    std::fs::write(&src, "hello parity").unwrap();

    fs::symlink(&src, &dst).unwrap();
    assert!(dst.exists(), "symlink should exist");
    let content = std::fs::read_to_string(&dst).unwrap();
    assert_eq!(
        content, "hello parity",
        "symlink should read source content"
    );

    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_symlink_overwrites_existing() {
    let dir = test_dir("sym_overwrite");
    let src = dir.join("source.txt");
    let dst = dir.join("link.txt");
    std::fs::write(&src, "new content").unwrap();
    std::fs::write(&dst, "old content").unwrap();

    fs::symlink(&src, &dst).unwrap();
    let content = std::fs::read_to_string(&dst).unwrap();
    assert_eq!(content, "new content", "symlink should overwrite old file");

    let _ = std::fs::remove_dir_all(&dir);
}

// ═══════════════════════════════════════════════════════════════════════════
// File Locking
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_file_lock_exclusive_acquire_release() {
    let path = std::env::temp_dir().join(format!("horus_parity_lock_{}", std::process::id()));
    let file = std::fs::File::create(&path).unwrap();
    let lock = fs::FileLock::exclusive(&file).unwrap();
    drop(lock); // release
    let _ = std::fs::remove_file(&path);
}

#[test]
fn test_file_lock_shared_allows_multiple() {
    let path = std::env::temp_dir().join(format!("horus_parity_shared_{}", std::process::id()));
    let file1 = std::fs::File::create(&path).unwrap();
    let file2 = std::fs::File::open(&path).unwrap();
    let _lock1 = fs::FileLock::shared(&file1).unwrap();
    let _lock2 = fs::FileLock::shared(&file2).unwrap();
    // Both should coexist without deadlock
    let _ = std::fs::remove_file(&path);
}

// ═══════════════════════════════════════════════════════════════════════════
// Permissions
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_is_executable_nonexistent_false() {
    assert!(!fs::is_executable(std::path::Path::new("/nonexistent/xyz")));
}

#[test]
fn test_set_executable_then_is_executable() {
    let path = std::env::temp_dir().join(format!("horus_parity_exec_{}", std::process::id()));
    std::fs::write(&path, "#!/bin/sh\necho hi").unwrap();
    fs::set_executable(&path).unwrap();

    // On Unix: execute bit is set. On Windows: is_executable checks existence.
    // Either way, after set_executable, is_executable should return true.
    #[cfg(unix)]
    assert!(
        fs::is_executable(&path),
        "should be executable after set_executable"
    );

    let _ = std::fs::remove_file(&path);
}

#[test]
fn test_set_private_does_not_error() {
    let path = std::env::temp_dir().join(format!("horus_parity_priv_{}", std::process::id()));
    std::fs::write(&path, "secret").unwrap();
    fs::set_private(&path).unwrap();
    // Verify file still readable by owner
    let content = std::fs::read_to_string(&path).unwrap();
    assert_eq!(content, "secret");
    let _ = std::fs::remove_file(&path);
}

// ═══════════════════════════════════════════════════════════════════════════
// Secure Creation
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_create_dir_secure_creates_directory() {
    let dir = std::env::temp_dir().join(format!("horus_parity_secure_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    fs::create_dir_secure(&dir).unwrap();
    assert!(dir.is_dir(), "secure dir should exist");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn test_open_private_creates_writable_file() {
    let path = std::env::temp_dir().join(format!("horus_parity_openpriv_{}", std::process::id()));
    {
        let mut file = fs::open_private(&path).unwrap();
        write!(file, "private data").unwrap();
    }
    let content = std::fs::read_to_string(&path).unwrap();
    assert_eq!(content, "private data");
    let _ = std::fs::remove_file(&path);
}

// ═══════════════════════════════════════════════════════════════════════════
// Dev Null
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_dev_null_exists() {
    let path = fs::dev_null();
    assert!(path.exists(), "dev_null path must exist on all platforms");
}

#[test]
fn test_dev_null_writable() {
    let path = fs::dev_null();
    let mut file = std::fs::File::create(path).unwrap();
    write!(file, "discarded").unwrap();
}

// ═══════════════════════════════════════════════════════════════════════════
// Stdout Suppression
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn test_suppress_stdout_and_restore() {
    let guard = fs::suppress_stdout().unwrap();
    println!("this should not appear");
    drop(guard);
    // After drop, stdout should be restored (no way to assert in test, just verify no crash)
}
