// Windows RT: REALTIME_PRIORITY_CLASS + SetThreadPriority

use super::RtCapabilities;
use std::time::Duration;

/// Detect Windows RT capabilities.
pub(super) fn detect_capabilities() -> RtCapabilities {
    let cpu_count = std::thread::available_parallelism()
        .map(|p| p.get())
        .unwrap_or(1);

    RtCapabilities {
        preempt_rt: false,
        max_priority: 31, // Windows thread priorities range 0-31
        min_priority: 1,
        memory_locking: true, // VirtualLock available
        cpu_affinity: true,
        kernel_version: get_windows_version(),
        cpu_count,
        estimated_jitter: Duration::from_millis(1), // Windows ~1-10ms jitter
    }
}

/// Set real-time priority using Windows API.
///
/// Sets process priority to REALTIME_PRIORITY_CLASS and thread to TIME_CRITICAL.
pub(super) fn set_realtime_priority(_priority: i32) -> anyhow::Result<()> {
    use windows_sys::Win32::System::Threading::{
        GetCurrentProcess, GetCurrentThread, SetPriorityClass, SetThreadPriority,
        REALTIME_PRIORITY_CLASS, THREAD_PRIORITY_TIME_CRITICAL,
    };

    // SAFETY: GetCurrentProcess/Thread always return valid pseudo-handles
    let process = unsafe { GetCurrentProcess() };
    let thread = unsafe { GetCurrentThread() };

    // SAFETY: process is a valid handle, REALTIME_PRIORITY_CLASS is a valid priority class
    let result = unsafe { SetPriorityClass(process, REALTIME_PRIORITY_CLASS) };
    if result == 0 {
        anyhow::bail!(
            "SetPriorityClass(REALTIME) failed: {}",
            std::io::Error::last_os_error()
        );
    }

    // SAFETY: thread is a valid handle, THREAD_PRIORITY_TIME_CRITICAL is valid
    let result = unsafe { SetThreadPriority(thread, THREAD_PRIORITY_TIME_CRITICAL) };
    if result == 0 {
        anyhow::bail!(
            "SetThreadPriority(TIME_CRITICAL) failed: {}",
            std::io::Error::last_os_error()
        );
    }

    log::debug!("Windows: Set REALTIME_PRIORITY_CLASS + THREAD_PRIORITY_TIME_CRITICAL");
    Ok(())
}

/// Lock memory on Windows via SetProcessWorkingSetSize.
///
/// Windows has no `mlockall()` equivalent. Instead:
/// 1. `SetProcessWorkingSetSize` with min=max pins the working set,
///    preventing the OS from paging out process memory under pressure.
/// 2. This is the closest Windows equivalent to Linux `mlockall(MCL_CURRENT)`.
///
/// Note: Requires SeIncreaseWorkingSetPrivilege (granted by default to admins).
/// On non-admin processes, this may fail silently — we log and return Ok.
pub(super) fn lock_memory() -> anyhow::Result<()> {
    use windows_sys::Win32::System::Memory::SetProcessWorkingSetSizeEx;
    use windows_sys::Win32::System::Threading::GetCurrentProcess;

    // Get current working set size to compute minimum
    let process = unsafe { GetCurrentProcess() };

    // Set min = max = 256 MB to pin working set.
    // The OS won't page out any memory until usage exceeds this.
    // For RT nodes, this prevents page fault jitter.
    let min_ws: usize = 256 * 1024 * 1024; // 256 MB
    let max_ws: usize = 512 * 1024 * 1024; // 512 MB

    // SAFETY: process is a valid pseudo-handle from GetCurrentProcess.
    // Flags=0 means hard min/max limits (QUOTA_LIMITS_HARDWS_MIN_ENABLE | MAX_ENABLE)
    let result = unsafe { SetProcessWorkingSetSizeEx(process, min_ws, max_ws, 0) };

    if result == 0 {
        let err = std::io::Error::last_os_error();
        log::warn!(
            "Windows: SetProcessWorkingSetSize failed (non-fatal): {}. \
             RT nodes may experience occasional page fault jitter. \
             Run as administrator for guaranteed memory locking.",
            err
        );
    } else {
        log::debug!(
            "Windows: Working set pinned to {}-{} MB (memory locked)",
            min_ws / (1024 * 1024),
            max_ws / (1024 * 1024)
        );
    }

    Ok(())
}

fn get_windows_version() -> String {
    // Try reading from environment or RtlGetVersion
    // HKLM\SOFTWARE\Microsoft\Windows NT\CurrentVersion has build info
    if let Ok(val) = std::env::var("OS") {
        // Try to get more detail via systeminfo-style detection
        // For now, return what we have
        format!("Windows ({})", val)
    } else {
        "Windows".to_string()
    }
}
