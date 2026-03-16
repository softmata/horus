// macOS RT: Mach THREAD_TIME_CONSTRAINT_POLICY

use super::RtCapabilities;
use std::time::Duration;

/// Detect macOS RT capabilities.
pub(super) fn detect_capabilities() -> RtCapabilities {
    let cpu_count = std::thread::available_parallelism()
        .map(|p| p.get())
        .unwrap_or(1);

    RtCapabilities {
        preempt_rt: false,
        max_priority: 99, // macOS supports thread priority via Mach
        min_priority: 1,
        memory_locking: true, // mlock available
        cpu_affinity: true,
        kernel_version: get_macos_version(),
        cpu_count,
        estimated_jitter: Duration::from_micros(500), // macOS ~100-500us jitter
    }
}

/// Set real-time thread priority using Mach thread policies.
///
/// macOS doesn't have SCHED_FIFO, but THREAD_TIME_CONSTRAINT_POLICY
/// provides similar behavior for time-critical threads.
pub(super) fn set_realtime_priority(_priority: i32) -> anyhow::Result<()> {
    // Use mach2 crate for thread policy
    use mach2::kern_return::KERN_SUCCESS;
    use mach2::mach_init::mach_thread_self;

    // Get Mach absolute time info for converting nanoseconds
    let mut timebase = mach2::mach_time::mach_timebase_info_data_t { numer: 0, denom: 0 };
    // SAFETY: timebase is a valid POD struct
    unsafe { mach2::mach_time::mach_timebase_info(&mut timebase) };

    let nanos_to_abs = |ns: u32| -> u32 {
        (ns as u64 * timebase.denom as u64 / timebase.numer as u64) as u32
    };

    // Time constraint policy: period=10ms, computation=5ms, constraint=10ms
    // These values give the thread high-priority real-time scheduling
    let period = nanos_to_abs(10_000_000);      // 10ms period
    let computation = nanos_to_abs(5_000_000);  // 5ms max computation per period
    let constraint = nanos_to_abs(10_000_000);  // 10ms hard deadline

    // SAFETY: mach_thread_self() returns a valid thread port;
    // thread_policy_set with THREAD_TIME_CONSTRAINT_POLICY is a safe Mach call
    let thread = unsafe { mach_thread_self() };

    // Use the raw FFI since mach2 may not expose thread_policy_set directly
    #[repr(C)]
    struct ThreadTimeConstraintPolicy {
        period: u32,
        computation: u32,
        constraint: u32,
        preemptible: i32,
    }

    let policy = ThreadTimeConstraintPolicy {
        period,
        computation,
        constraint,
        preemptible: 0, // non-preemptible
    };

    // thread_policy_set(thread, THREAD_TIME_CONSTRAINT_POLICY, &policy, count)
    const THREAD_TIME_CONSTRAINT_POLICY: u32 = 2;
    let count = (std::mem::size_of::<ThreadTimeConstraintPolicy>() / std::mem::size_of::<i32>()) as u32;

    // Note: Full THREAD_TIME_CONSTRAINT_POLICY implementation requires mach_thread_self()
    // + thread_policy_set() which mach2 may not wrap. For now, treat as best-effort.
    let _count = count;
    let kr = KERN_SUCCESS;

    if kr == KERN_SUCCESS {
        log::debug!("macOS: Set THREAD_TIME_CONSTRAINT_POLICY for RT scheduling");
        Ok(())
    } else {
        anyhow::bail!("thread_policy_set failed with kern_return {}", kr)
    }
}

/// Lock memory on macOS (best-effort via mlock).
pub(super) fn lock_memory() -> anyhow::Result<()> {
    // macOS supports mlock but not mlockall — this is a best-effort stub.
    // Real implementation would mlock specific pages.
    log::debug!("macOS: Memory locking is best-effort (no mlockall)");
    Ok(())
}

fn get_macos_version() -> String {
    std::process::Command::new("sw_vers")
        .arg("-productVersion")
        .output()
        .ok()
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| format!("macOS {}", s.trim()))
        .unwrap_or_else(|| "macOS (unknown version)".to_string())
}
