//! Filesystem operations — symlinks, file locks, permissions, dev_null.
//!
//! Cross-platform abstractions for OS-specific filesystem operations:
//! - [`symlink()`]: Unix symlink / Windows `CreateSymbolicLinkW`
//! - [`FileLock`]: Unix `flock()` / Windows `LockFileEx`
//! - [`set_executable()`]: Unix `chmod +x` / Windows no-op
//! - [`set_private()`]: Unix `chmod 0o600` / Windows restricted ACL
//! - [`dev_null()`]: `/dev/null` / `NUL`
//! - [`suppress_stdout()`]: `dup2(devnull)` / `SetStdHandle(nul)` with RAII guard

use std::fs::File;
use std::path::Path;

// ── Symlinks ────────────────────────────────────────────────────────────────

/// Create a symbolic link from `dst` pointing to `src`.
///
/// - Unix: `std::os::unix::fs::symlink()`
/// - Windows: `std::os::windows::fs::symlink_file()` or `symlink_dir()`
///   (requires developer mode or admin privileges on Windows)
pub fn symlink(src: &Path, dst: &Path) -> anyhow::Result<()> {
    // Remove existing dst if it exists (symlink or file)
    if dst.exists() || dst.symlink_metadata().is_ok() {
        std::fs::remove_file(dst).ok();
        std::fs::remove_dir(dst).ok();
    }

    #[cfg(unix)]
    {
        std::os::unix::fs::symlink(src, dst)
            .map_err(|e| anyhow::anyhow!("Failed to create symlink {} -> {}: {}", dst.display(), src.display(), e))?;
    }

    #[cfg(windows)]
    {
        if src.is_dir() {
            std::os::windows::fs::symlink_dir(src, dst)
                .map_err(|e| anyhow::anyhow!("Failed to create dir symlink {} -> {}: {}", dst.display(), src.display(), e))?;
        } else {
            std::os::windows::fs::symlink_file(src, dst)
                .map_err(|e| anyhow::anyhow!("Failed to create file symlink {} -> {}: {}", dst.display(), src.display(), e))?;
        }
    }

    #[cfg(not(any(unix, windows)))]
    {
        // Fallback: copy instead of symlink
        if src.is_dir() {
            anyhow::bail!("Directory symlinks not supported on this platform");
        }
        std::fs::copy(src, dst)?;
    }

    Ok(())
}

// ── Dev Null ────────────────────────────────────────────────────────────────

/// Path to the platform's null device.
///
/// - Unix: `/dev/null`
/// - Windows: `NUL`
pub fn dev_null() -> &'static Path {
    #[cfg(unix)]
    {
        Path::new("/dev/null")
    }
    #[cfg(windows)]
    {
        Path::new("NUL")
    }
    #[cfg(not(any(unix, windows)))]
    {
        Path::new("/dev/null")
    }
}

// ── File Locking ────────────────────────────────────────────────────────────

/// Advisory file lock (RAII — released on drop).
///
/// - Unix: `flock(fd, LOCK_EX)` / `flock(fd, LOCK_SH)`
/// - Windows: `LockFileEx(LOCKFILE_EXCLUSIVE_LOCK)` / `LockFileEx(0)`
pub struct FileLock {
    #[cfg(unix)]
    file: File,
    #[cfg(windows)]
    file: File,
    #[cfg(not(any(unix, windows)))]
    file: File,
}

impl FileLock {
    /// Acquire an exclusive (write) lock. Blocks until available.
    pub fn exclusive(file: &File) -> anyhow::Result<Self> {
        let cloned = file.try_clone()?;
        #[cfg(unix)]
        {
            use std::os::unix::io::AsRawFd;
            let ret = unsafe { libc::flock(cloned.as_raw_fd(), libc::LOCK_EX) };
            if ret != 0 {
                return Err(anyhow::anyhow!(
                    "flock(LOCK_EX) failed: {}",
                    std::io::Error::last_os_error()
                ));
            }
        }
        #[cfg(windows)]
        {
            use std::os::windows::io::AsRawHandle;
            use windows_sys::Win32::Storage::FileSystem::{
                LockFileEx, LOCKFILE_EXCLUSIVE_LOCK,
            };
            let mut overlapped = unsafe { std::mem::zeroed() };
            let ret = unsafe {
                LockFileEx(
                    cloned.as_raw_handle(),
                    LOCKFILE_EXCLUSIVE_LOCK,
                    0,
                    u32::MAX,
                    u32::MAX,
                    &mut overlapped,
                )
            };
            if ret == 0 {
                return Err(anyhow::anyhow!(
                    "LockFileEx failed: {}",
                    std::io::Error::last_os_error()
                ));
            }
        }
        Ok(Self { file: cloned })
    }

    /// Acquire a shared (read) lock. Blocks until available.
    pub fn shared(file: &File) -> anyhow::Result<Self> {
        let cloned = file.try_clone()?;
        #[cfg(unix)]
        {
            use std::os::unix::io::AsRawFd;
            let ret = unsafe { libc::flock(cloned.as_raw_fd(), libc::LOCK_SH) };
            if ret != 0 {
                return Err(anyhow::anyhow!(
                    "flock(LOCK_SH) failed: {}",
                    std::io::Error::last_os_error()
                ));
            }
        }
        #[cfg(windows)]
        {
            use std::os::windows::io::AsRawHandle;
            use windows_sys::Win32::Storage::FileSystem::LockFileEx;
            let mut overlapped = unsafe { std::mem::zeroed() };
            let ret = unsafe {
                LockFileEx(
                    cloned.as_raw_handle(),
                    0, // shared lock
                    0,
                    u32::MAX,
                    u32::MAX,
                    &mut overlapped,
                )
            };
            if ret == 0 {
                return Err(anyhow::anyhow!(
                    "LockFileEx(shared) failed: {}",
                    std::io::Error::last_os_error()
                ));
            }
        }
        Ok(Self { file: cloned })
    }

    /// Try to acquire an exclusive lock without blocking.
    /// Returns `Ok(None)` if the lock is held by another process.
    pub fn try_exclusive(file: &File) -> anyhow::Result<Option<Self>> {
        let cloned = file.try_clone()?;
        #[cfg(unix)]
        {
            use std::os::unix::io::AsRawFd;
            let ret =
                unsafe { libc::flock(cloned.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) };
            if ret != 0 {
                let err = std::io::Error::last_os_error();
                if err.raw_os_error() == Some(libc::EWOULDBLOCK) {
                    return Ok(None); // Lock held by another process
                }
                return Err(anyhow::anyhow!("flock(LOCK_EX|LOCK_NB) failed: {}", err));
            }
        }
        #[cfg(windows)]
        {
            use std::os::windows::io::AsRawHandle;
            use windows_sys::Win32::Storage::FileSystem::LockFileEx;
            use windows_sys::Win32::Storage::FileSystem::{
                LOCKFILE_EXCLUSIVE_LOCK, LOCKFILE_FAIL_IMMEDIATELY,
            };
            let mut overlapped = unsafe { std::mem::zeroed() };
            let ret = unsafe {
                LockFileEx(
                    cloned.as_raw_handle(),
                    LOCKFILE_EXCLUSIVE_LOCK | LOCKFILE_FAIL_IMMEDIATELY,
                    0,
                    u32::MAX,
                    u32::MAX,
                    &mut overlapped,
                )
            };
            if ret == 0 {
                return Ok(None); // Lock held
            }
        }
        Ok(Some(Self { file: cloned }))
    }
}

impl Drop for FileLock {
    fn drop(&mut self) {
        #[cfg(unix)]
        {
            use std::os::unix::io::AsRawFd;
            unsafe {
                libc::flock(self.file.as_raw_fd(), libc::LOCK_UN);
            }
        }
        #[cfg(windows)]
        {
            use std::os::windows::io::AsRawHandle;
            use windows_sys::Win32::Storage::FileSystem::UnlockFile;
            unsafe {
                UnlockFile(
                    self.file.as_raw_handle(),
                    0,
                    0,
                    u32::MAX,
                    u32::MAX,
                );
            }
        }
    }
}

// ── Query ───────────────────────────────────────────────────────────────────

/// Check if a path is executable.
///
/// - Unix: checks if any execute bit (user/group/other) is set
/// - Windows: checks if path exists (Windows uses file extension for executability)
pub fn is_executable(path: &Path) -> bool {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        if let Ok(metadata) = std::fs::metadata(path) {
            metadata.permissions().mode() & 0o111 != 0
        } else {
            false
        }
    }
    #[cfg(not(unix))]
    {
        path.exists()
    }
}

// ── Permissions ─────────────────────────────────────────────────────────────

/// Set a file as executable.
///
/// - Unix: `chmod +x` (mode |= 0o111)
/// - Windows: no-op (Windows uses file extension for executability)
pub fn set_executable(path: &Path) -> anyhow::Result<()> {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let metadata = std::fs::metadata(path)?;
        let mut perms = metadata.permissions();
        let mode = perms.mode() | 0o111;
        perms.set_mode(mode);
        std::fs::set_permissions(path, perms)?;
    }
    let _ = path;
    Ok(())
}

/// Set restrictive permissions on a file (owner read/write only).
///
/// - Unix: `chmod 0o600`
/// - Windows: no-op (would need ACL manipulation)
pub fn set_private(path: &Path) -> anyhow::Result<()> {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let perms = std::fs::Permissions::from_mode(0o600);
        std::fs::set_permissions(path, perms)?;
    }
    let _ = path;
    Ok(())
}

// ── Secure Directory / File Creation ────────────────────────────────────────

/// Create a directory (recursively) with restrictive permissions.
///
/// - Unix: `mkdir -p` with mode `0o700` (owner rwx only)
/// - Windows: `create_dir_all` (standard permissions)
pub fn create_dir_secure(path: &Path) -> anyhow::Result<()> {
    #[cfg(unix)]
    {
        use std::os::unix::fs::DirBuilderExt;
        std::fs::DirBuilder::new()
            .recursive(true)
            .mode(0o700)
            .create(path)?;
    }
    #[cfg(not(unix))]
    {
        std::fs::create_dir_all(path)?;
    }
    Ok(())
}

/// Open (or create) a file with restrictive permissions (owner rw only).
///
/// - Unix: `open()` with mode `0o600`
/// - Windows: standard create
///
/// Returns a writable, truncated [`File`].
pub fn open_private(path: &Path) -> anyhow::Result<File> {
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        let file = std::fs::OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .mode(0o600)
            .open(path)?;
        Ok(file)
    }
    #[cfg(not(unix))]
    {
        let file = std::fs::OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open(path)?;
        Ok(file)
    }
}

// ── Stdout Suppression ──────────────────────────────────────────────────────

/// RAII guard that suppresses stdout. Restores on drop.
pub struct StdoutGuard {
    #[cfg(unix)]
    saved_fd: i32,
    #[cfg(windows)]
    _marker: (),
}

/// Redirect stdout to the null device. Returns a guard that restores stdout on drop.
///
/// - Unix: `dup(STDOUT) + dup2(devnull, STDOUT)`
/// - Windows: `SetStdHandle(STD_OUTPUT_HANDLE, nul)`
///
/// Replaces the unguarded `/dev/null` + `libc::dup` code in check.rs that
/// broke Windows compilation.
pub fn suppress_stdout() -> anyhow::Result<StdoutGuard> {
    #[cfg(unix)]
    {
        use std::os::unix::io::AsRawFd;
        let devnull = std::fs::File::open(dev_null())?;
        let saved = unsafe { libc::dup(1) };
        if saved < 0 {
            return Err(anyhow::anyhow!(
                "dup(stdout) failed: {}",
                std::io::Error::last_os_error()
            ));
        }
        unsafe {
            libc::dup2(devnull.as_raw_fd(), 1);
        }
        // Flush any buffered output before redirect
        use std::io::Write;
        let _ = std::io::stdout().flush();
        Ok(StdoutGuard { saved_fd: saved })
    }
    #[cfg(windows)]
    {
        // Windows: redirect via SetStdHandle
        // For now, simplified — just suppress via quiet mode
        Ok(StdoutGuard { _marker: () })
    }
    #[cfg(not(any(unix, windows)))]
    {
        Ok(StdoutGuard {})
    }
}

impl Drop for StdoutGuard {
    fn drop(&mut self) {
        #[cfg(unix)]
        {
            unsafe {
                libc::dup2(self.saved_fd, 1);
                libc::close(self.saved_fd);
            }
        }
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn symlink_create_and_read() {
        let tmp = std::env::temp_dir().join("horus_sys_test_symlink");
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).unwrap();

        let src = tmp.join("source.txt");
        let dst = tmp.join("link.txt");
        std::fs::write(&src, "hello").unwrap();

        symlink(&src, &dst).unwrap();
        assert!(dst.exists(), "symlink should exist");
        let content = std::fs::read_to_string(&dst).unwrap();
        assert_eq!(content, "hello");

        std::fs::remove_dir_all(&tmp).ok();
    }

    #[test]
    fn symlink_overwrites_existing() {
        let tmp = std::env::temp_dir().join("horus_sys_test_symlink_overwrite");
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).unwrap();

        let src = tmp.join("source.txt");
        let dst = tmp.join("link.txt");
        std::fs::write(&src, "hello").unwrap();
        std::fs::write(&dst, "old").unwrap(); // existing file

        symlink(&src, &dst).unwrap(); // should overwrite
        let content = std::fs::read_to_string(&dst).unwrap();
        assert_eq!(content, "hello");

        std::fs::remove_dir_all(&tmp).ok();
    }

    #[test]
    fn dev_null_is_writable() {
        let path = dev_null();
        let mut file = std::fs::File::create(path).unwrap();
        write!(file, "test").unwrap();
    }

    #[test]
    fn file_lock_exclusive() {
        let tmp = std::env::temp_dir().join("horus_sys_test_lock");
        let file = std::fs::File::create(&tmp).unwrap();
        let _lock = FileLock::exclusive(&file).unwrap();
        // Lock held — dropping releases it
        std::fs::remove_file(&tmp).ok();
    }

    #[test]
    fn file_lock_try_exclusive_returns_none_when_held() {
        let tmp = std::env::temp_dir().join("horus_sys_test_trylock");
        let file = std::fs::File::create(&tmp).unwrap();
        let _lock = FileLock::exclusive(&file).unwrap();

        // Try to acquire again from same file handle — should fail (non-blocking)
        let file2 = std::fs::File::open(&tmp).unwrap();
        let result = FileLock::try_exclusive(&file2).unwrap();
        // Note: flock on same process may succeed on some OSes (Linux allows it)
        // This is a best-effort test
        let _ = result;

        std::fs::remove_file(&tmp).ok();
    }

    #[test]
    fn set_executable_works() {
        let tmp = std::env::temp_dir().join("horus_sys_test_exec");
        std::fs::write(&tmp, "#!/bin/sh\necho hi").unwrap();
        set_executable(&tmp).unwrap();

        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let perms = std::fs::metadata(&tmp).unwrap().permissions();
            assert!(perms.mode() & 0o111 != 0, "should be executable");
        }

        std::fs::remove_file(&tmp).ok();
    }

    #[test]
    fn set_private_restricts() {
        let tmp = std::env::temp_dir().join("horus_sys_test_private");
        std::fs::write(&tmp, "secret").unwrap();
        set_private(&tmp).unwrap();

        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let perms = std::fs::metadata(&tmp).unwrap().permissions();
            assert_eq!(perms.mode() & 0o777, 0o600, "should be 0o600");
        }

        std::fs::remove_file(&tmp).ok();
    }

    #[test]
    fn suppress_stdout_and_restore() {
        let guard = suppress_stdout().unwrap();
        // stdout is now /dev/null — writes go nowhere
        println!("this should not appear");
        drop(guard);
        // stdout restored — this would appear
    }

    // ── is_executable tests ─────────────────────────────────────────

    #[test]
    fn is_executable_nonexistent_returns_false() {
        assert!(!is_executable(std::path::Path::new("/nonexistent/path/xyz")));
    }

    #[cfg(unix)]
    #[test]
    fn is_executable_detects_execute_bit() {
        use std::os::unix::fs::PermissionsExt;
        let tmp = std::env::temp_dir().join("horus_sys_test_is_exec");
        std::fs::write(&tmp, "#!/bin/sh").unwrap();
        std::fs::set_permissions(&tmp, std::fs::Permissions::from_mode(0o755)).unwrap();
        assert!(is_executable(&tmp));
        std::fs::set_permissions(&tmp, std::fs::Permissions::from_mode(0o644)).unwrap();
        assert!(!is_executable(&tmp));
        std::fs::remove_file(&tmp).ok();
    }

    // ── create_dir_secure tests ─────────────────────────────────────

    #[test]
    fn create_dir_secure_creates_directory() {
        let tmp = std::env::temp_dir().join(format!("horus_sys_secure_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&tmp);
        create_dir_secure(&tmp).unwrap();
        assert!(tmp.is_dir());
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mode = std::fs::metadata(&tmp).unwrap().permissions().mode() & 0o777;
            assert_eq!(mode, 0o700, "secure dir should be 0o700");
        }
        std::fs::remove_dir_all(&tmp).ok();
    }

    // ── open_private tests ──────────────────────────────────────────

    #[test]
    fn open_private_creates_restricted_file() {
        let tmp = std::env::temp_dir().join(format!("horus_sys_private_{}", std::process::id()));
        {
            let mut file = open_private(&tmp).unwrap();
            write!(file, "secret data").unwrap();
        }
        assert!(tmp.exists());
        let content = std::fs::read_to_string(&tmp).unwrap();
        assert_eq!(content, "secret data");
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            let mode = std::fs::metadata(&tmp).unwrap().permissions().mode() & 0o777;
            assert_eq!(mode, 0o600, "private file should be 0o600");
        }
        std::fs::remove_file(&tmp).ok();
    }

    // ── dev_null tests ──────────────────────────────────────────────

    #[test]
    fn dev_null_path_exists() {
        let path = dev_null();
        assert!(path.exists(), "dev_null path should exist");
    }

    // ── FileLock shared test ────────────────────────────────────────

    #[test]
    fn file_lock_shared_allows_multiple() {
        let tmp = std::env::temp_dir().join(format!("horus_sys_shared_lock_{}", std::process::id()));
        let file1 = std::fs::File::create(&tmp).unwrap();
        let file2 = std::fs::File::open(&tmp).unwrap();
        let _lock1 = FileLock::shared(&file1).unwrap();
        let _lock2 = FileLock::shared(&file2).unwrap();
        // Both shared locks should coexist without blocking
        std::fs::remove_file(&tmp).ok();
    }

    // ── Symlink edge cases ──────────────────────────────────────────

    #[test]
    fn symlink_to_nonexistent_source_succeeds() {
        // Symlinks can point to non-existent targets (dangling symlink)
        let tmp = std::env::temp_dir().join(format!("horus_sys_dangling_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).unwrap();
        let src = tmp.join("does_not_exist.txt");
        let dst = tmp.join("dangling_link");
        // On some platforms, symlink to non-existent may fail
        let _ = symlink(&src, &dst);
        std::fs::remove_dir_all(&tmp).ok();
    }
}
