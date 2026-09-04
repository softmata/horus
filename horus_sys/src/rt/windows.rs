// Windows RT: REALTIME_PRIORITY_CLASS + SetThreadPriority

use super::RtCapabilities;
use std::time::Duration;

/// Detect Windows RT capabilities.
pub(super) fn detect_capabilities() -> RtCapabilities {
    let cpu_count = std::thread::available_parallelism()
        .map(|p| p.get())
        .unwrap_or(1);

    RtCapabilities {
        // Derived from the one function that owns this policy rather than
        // restated here, so the two cannot drift if the Windows probe ever
        // learns to check SeIncreaseBasePriorityPrivilege.
        // (`super::can_set_rt_priority` does not dispatch back into this
        // module, so there is no recursion.)
        rt_priority_permitted: super::can_set_rt_priority(),
        preempt_rt: false,
        max_priority: 31, // Windows thread priorities range 0-31
        min_priority: 1,
        // `lock_memory()` below pins the working set with
        // SetProcessWorkingSetSizeEx; nothing in this backend calls VirtualLock,
        // and the pin needs SeIncreaseWorkingSetPrivilege, so this advertises
        // only that a mechanism exists — not that it will succeed unelevated.
        memory_locking: true,
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

/// Lock memory on Windows via SetProcessWorkingSetSizeEx.
///
/// Windows has no `mlockall()` equivalent. Instead:
/// 1. `SetProcessWorkingSetSizeEx` with hard min/max limits pins the working
///    set, preventing the OS from paging out process memory under pressure.
/// 2. This is the closest Windows equivalent to Linux `mlockall(MCL_CURRENT)`,
///    though still weaker: it bounds the working set rather than pinning every
///    page.
///
/// Requires SeIncreaseWorkingSetPrivilege (granted by default to admins).
/// Failure returns `Err` rather than `Ok`: the `Result` is the only channel by
/// which `RtConfig::apply()` learns memory was not locked, and swallowing the
/// error here made a non-elevated process report `RtApplyResult::FullSuccess`
/// while its RT nodes still took page-fault stalls. Returning `Err` lets
/// RtConfig record an honest `RtDegradation::MemoryLockUnavailable`, matching
/// the macOS stub.
pub(super) fn lock_memory() -> anyhow::Result<()> {
    use windows_sys::Win32::System::Memory::{
        SetProcessWorkingSetSizeEx, QUOTA_LIMITS_HARDWS_MAX_ENABLE, QUOTA_LIMITS_HARDWS_MIN_ENABLE,
    };
    use windows_sys::Win32::System::Threading::GetCurrentProcess;

    // Get current working set size to compute minimum
    let process = unsafe { GetCurrentProcess() };

    // Set min = max = 256 MB to pin working set.
    // The OS won't page out any memory until usage exceeds this.
    // For RT nodes, this prevents page fault jitter.
    let min_ws: usize = 256 * 1024 * 1024; // 256 MB
    let max_ws: usize = 512 * 1024 * 1024; // 512 MB

    // Flags 0 requests *soft* limits, which the OS may ignore under pressure —
    // the opposite of what the previous comment here claimed. MIN_ENABLE |
    // MAX_ENABLE is what actually enforces the limits as hard.
    let flags = QUOTA_LIMITS_HARDWS_MIN_ENABLE | QUOTA_LIMITS_HARDWS_MAX_ENABLE;

    // SAFETY: process is a valid pseudo-handle from GetCurrentProcess.
    let result = unsafe { SetProcessWorkingSetSizeEx(process, min_ws, max_ws, flags) };

    if result == 0 {
        let err = std::io::Error::last_os_error();
        anyhow::bail!(
            "SetProcessWorkingSetSizeEx failed: {}. Windows working-set pinning \
             requires SeIncreaseWorkingSetPrivilege; run elevated for memory locking.",
            err
        );
    }

    log::debug!(
        "Windows: Working set pinned to {}-{} MB (hard limits)",
        min_ws / (1024 * 1024),
        max_ws / (1024 * 1024)
    );

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
