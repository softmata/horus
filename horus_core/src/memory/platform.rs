// Cross-platform shared memory - each platform uses its optimal mechanism
//
// Linux: /dev/shm/horus (tmpfs - RAM-backed, fastest) via file mmap
// macOS: POSIX shm_open() (Mach shared memory - RAM-backed) - NOT file-based!
// Windows: CreateFileMappingW (pagefile-backed - optimized for IPC) - NOT temp files!
//
// Note: macOS and Windows no longer use filesystem paths for shared memory.
// The path functions below are kept for Linux and backward compatibility only.

use std::path::PathBuf;

/// Get the base directory for HORUS shared memory
///
/// This returns a platform-appropriate path for shared memory:
/// - Linux: `/dev/shm/horus` (tmpfs for maximum performance)
/// - macOS: `/tmp/horus` (no /dev/shm, but /tmp is still fast)
/// - Windows: `%TEMP%\horus` (system temp directory)
pub fn shm_base_dir() -> PathBuf {
    #[cfg(target_os = "linux")]
    {
        PathBuf::from("/dev/shm/horus")
    }

    #[cfg(target_os = "macos")]
    {
        // macOS doesn't have /dev/shm, use /tmp instead
        // For better performance, could use shm_open() in the future
        PathBuf::from("/tmp/horus")
    }

    #[cfg(target_os = "windows")]
    {
        // Windows uses temp directory
        std::env::temp_dir().join("horus")
    }

    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        // Fallback for other Unix-like systems (BSD, etc.)
        PathBuf::from("/tmp/horus")
    }
}

/// Get the topics directory for shared memory message passing
pub fn shm_topics_dir() -> PathBuf {
    shm_base_dir().join("topics")
}

/// Get the nodes directory for node presence files
///
/// Each running node creates a presence file here at startup and removes it at shutdown.
/// Monitor scans this directory to discover all active nodes.
///
/// Structure: `/dev/shm/horus/nodes/{node_name}.json`
pub fn shm_nodes_dir() -> PathBuf {
    shm_base_dir().join("nodes")
}

/// Get the network status directory for transport monitoring
pub fn shm_network_dir() -> PathBuf {
    shm_base_dir().join("network")
}

/// Get the params directory for cross-process runtime parameters
///
/// Parameters stored here can be read/written by multiple processes,
/// enabling dynamic runtime configuration (e.g., PID tuning).
///
/// Structure: `/dev/shm/horus/params/{param_name}` (one file per param)
pub fn shm_params_dir() -> PathBuf {
    shm_base_dir().join("params")
}

/// Get the control directory for node lifecycle commands
///
/// Control files allow external processes (like `horus node kill`) to
/// send commands to running nodes without killing the entire scheduler.
///
/// Structure: `/dev/shm/horus/control/{node_name}.cmd`
/// Commands: "stop", "restart", "pause", "resume"
pub fn shm_control_dir() -> PathBuf {
    shm_base_dir().join("control")
}

/// Get the logs shared memory path
pub fn shm_logs_path() -> PathBuf {
    // Logs are at the same level as horus dir, not inside it
    #[cfg(target_os = "linux")]
    {
        PathBuf::from("/dev/shm/horus_logs")
    }

    #[cfg(target_os = "macos")]
    {
        PathBuf::from("/tmp/horus_logs")
    }

    #[cfg(target_os = "windows")]
    {
        std::env::temp_dir().join("horus_logs")
    }

    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        PathBuf::from("/tmp/horus_logs")
    }
}

/// Check if a process with given PID is running
pub fn is_process_running(pid: u32) -> bool {
    #[cfg(unix)]
    {
        // SAFETY: kill(pid, 0) sends no signal; only checks if process exists. No memory is accessed.
        unsafe { libc::kill(pid as i32, 0) == 0 }
    }

    #[cfg(windows)]
    {
        // On Windows, try to open the process using windows-sys
        use windows_sys::Win32::Foundation::CloseHandle;
        use windows_sys::Win32::System::Threading::{
            OpenProcess, PROCESS_QUERY_LIMITED_INFORMATION,
        };
        // SAFETY: OpenProcess with limited query rights is safe for existence check.
        // Handle is closed immediately after.
        unsafe {
            let handle = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, 0, pid);
            if handle == 0 {
                false
            } else {
                CloseHandle(handle);
                true
            }
        }
    }

    #[cfg(not(any(unix, windows)))]
    {
        // Fallback: check /proc on Linux-like systems
        std::path::Path::new(&format!("/proc/{}", pid)).exists()
    }
}

/// Check if we're running on a platform with true RAM-backed shared memory
///
/// All major platforms now use optimal shared memory:
/// - Linux: /dev/shm (tmpfs - RAM)
/// - macOS: shm_open() (Mach shared memory - RAM)
/// - Windows: CreateFileMapping (pagefile-backed - OS optimized)
pub fn has_native_shm() -> bool {
    #[cfg(any(target_os = "linux", target_os = "macos", target_os = "windows"))]
    {
        true
    }

    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        false // BSD and others still use file-based fallback
    }
}

/// Get platform name for logging/diagnostics
pub fn platform_name() -> &'static str {
    #[cfg(target_os = "linux")]
    {
        "Linux"
    }

    #[cfg(target_os = "macos")]
    {
        "macOS"
    }

    #[cfg(target_os = "windows")]
    {
        "Windows"
    }

    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        "Unix"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_shm_paths_are_valid() {
        let base = shm_base_dir();
        assert!(!base.as_os_str().is_empty());

        let topics = shm_topics_dir();
        assert!(topics.starts_with(&base));

        let nodes = shm_nodes_dir();
        assert!(nodes.starts_with(&base));

        let params = shm_params_dir();
        assert!(params.starts_with(&base));
    }
}
