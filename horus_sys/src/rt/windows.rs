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

/// Lock memory on Windows (best-effort via SetProcessWorkingSetSize).
pub(super) fn lock_memory() -> anyhow::Result<()> {
    // Windows doesn't have mlockall. SetProcessWorkingSetSize can prevent paging
    // but requires careful tuning. This is a best-effort stub.
    log::debug!("Windows: Memory locking is best-effort (no mlockall equivalent)");
    Ok(())
}

fn get_windows_version() -> String {
    // Read from registry or use GetVersionEx
    format!(
        "Windows (build {})",
        std::env::var("OS").unwrap_or_default()
    )
}
