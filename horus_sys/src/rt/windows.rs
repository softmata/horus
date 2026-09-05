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

/// Environment variable that opts a deployment back into the process-wide
/// `REALTIME_PRIORITY_CLASS`.
pub(super) const REALTIME_CLASS_ENV: &str = "HORUS_WIN_REALTIME_CLASS";

/// Whether the process-wide realtime class has already been raised, so that N
/// RT tick threads raise it once rather than N times.
static REALTIME_CLASS_RAISED: std::sync::Once = std::sync::Once::new();

/// Give the calling thread real-time priority.
///
/// # Why this no longer raises `REALTIME_PRIORITY_CLASS` by default
///
/// It is a **process** attribute. Raising it moved every thread in the process
/// — the helpers, the log drain, the tokio blocking pool, and threads that
/// already existed — into the 16-31 band, above most driver threads, and no
/// per-thread `SetThreadPriority` brings one back out: even
/// `THREAD_PRIORITY_IDLE` inside that class is base 16. So the Windows arm of
/// this backend reproduced defect 02's Linux shape by a different mechanism,
/// and unlike the Linux one it could not be undone per thread.
///
/// `THREAD_PRIORITY_TIME_CRITICAL` inside the inherited class gives base 15
/// under `NORMAL_PRIORITY_CLASS`, which preempts every ordinary thread on the
/// box while leaving the kernel's own alone — the same *relative* guarantee the
/// Linux `SCHED_FIFO` path gives, and the guarantee
/// [`set_best_effort_class`] can actually restore a helper out of.
///
/// Set `HORUS_WIN_REALTIME_CLASS=1` on a machine genuinely dedicated to the
/// robot to get the old behaviour.
pub(super) fn set_realtime_priority(_priority: i32) -> anyhow::Result<()> {
    use windows_sys::Win32::System::Threading::{
        GetCurrentProcess, GetCurrentThread, SetPriorityClass, SetThreadPriority,
        REALTIME_PRIORITY_CLASS, THREAD_PRIORITY_TIME_CRITICAL,
    };

    // SAFETY: GetCurrentProcess/Thread always return valid pseudo-handles
    let thread = unsafe { GetCurrentThread() };

    if std::env::var(REALTIME_CLASS_ENV).as_deref() == Ok("1") {
        let mut failure: Option<std::io::Error> = None;
        REALTIME_CLASS_RAISED.call_once(|| {
            // SAFETY: pseudo-handle as above; REALTIME_PRIORITY_CLASS is a
            // valid priority class.
            let process = unsafe { GetCurrentProcess() };
            if unsafe { SetPriorityClass(process, REALTIME_PRIORITY_CLASS) } == 0 {
                failure = Some(std::io::Error::last_os_error());
            }
        });
        if let Some(e) = failure {
            anyhow::bail!("SetPriorityClass(REALTIME) failed: {}", e);
        }
    }

    // SAFETY: thread is a valid handle, THREAD_PRIORITY_TIME_CRITICAL is valid
    let result = unsafe { SetThreadPriority(thread, THREAD_PRIORITY_TIME_CRITICAL) };
    if result == 0 {
        anyhow::bail!(
            "SetThreadPriority(TIME_CRITICAL) failed: {}",
            std::io::Error::last_os_error()
        );
    }

    log::debug!("Windows: Set THREAD_PRIORITY_TIME_CRITICAL");
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

/// Put the calling thread back on the ordinary priority band and give it back
/// the helper CPU set.
///
/// # What this can and cannot undo
///
/// `SetThreadPriority(THREAD_PRIORITY_NORMAL)` restores the *relative*
/// ordering: an RT tick thread at `THREAD_PRIORITY_TIME_CRITICAL` preempts
/// this one again. That is the same relative guarantee the Linux path gives.
///
/// What it cannot undo is `REALTIME_PRIORITY_CLASS`, which is a **process**
/// attribute: once raised, every thread in the process sits in the 16-31 band
/// and no per-thread call brings one back out (even `THREAD_PRIORITY_IDLE`
/// inside that class is base 16). That is why [`set_realtime_priority`] no
/// longer raises the class unless the operator opts in.
pub(super) fn set_best_effort_class(
    _nice_increment: i32,
    target: &[usize],
) -> super::BestEffortReport {
    use windows_sys::Win32::System::Threading::{
        GetCurrentThread, SetThreadAffinityMask, SetThreadPriority, THREAD_PRIORITY_NORMAL,
    };

    let mut report = super::BestEffortReport::default();

    // SAFETY: `GetCurrentThread` returns a pseudo-handle that needs no close,
    // and `SetThreadPriority` only writes the calling thread's priority.
    let ok = unsafe { SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL) };
    if ok != 0 {
        report.policy_changed = true;
    } else {
        report.refusal = Some(unsafe { windows_sys::Win32::Foundation::GetLastError() as i32 });
    }

    // `SetThreadAffinityMask` takes a bitmask over the process's processor
    // group, so CPUs at or beyond `usize::BITS` cannot be expressed and are
    // dropped rather than silently aliased onto a lower bit.
    if !target.is_empty() {
        let mut mask: usize = 0;
        for &cpu in target {
            if cpu < usize::BITS as usize {
                mask |= 1 << cpu;
            }
        }
        if mask != 0 {
            // SAFETY: pseudo-handle as above; the mask is a plain integer.
            let prev = unsafe { SetThreadAffinityMask(GetCurrentThread(), mask) };
            if prev != 0 {
                report.affinity_changed = true;
            } else if report.refusal.is_none() {
                report.refusal =
                    Some(unsafe { windows_sys::Win32::Foundation::GetLastError() as i32 });
            }
        }
    }

    report
}
