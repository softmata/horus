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
    // NOT YET IMPLEMENTED on macOS. A real implementation must call
    //   thread_policy_set(mach_thread_self(), THREAD_TIME_CONSTRAINT_POLICY,
    //                      &ThreadTimeConstraintPolicy{ period, computation,
    //                      constraint, preemptible }, count)
    // via raw Mach FFI (the `mach2` crate does not wrap `thread_policy_set`).
    //
    // Returning `Err` (rather than a fake `Ok`) is deliberate: it lets
    // horus_core's RtConfig record an honest `RtDegradation::SchedulerDegraded`
    // and fall back to soft scheduling, instead of silently claiming RT was
    // applied while the thread actually runs at normal priority. macOS is a
    // dev-only target for robotics (production hard-RT is Linux). See the
    // stub audit 2026-07-13.
    anyhow::bail!("macOS real-time scheduling (THREAD_TIME_CONSTRAINT_POLICY) is not yet implemented")
}

/// Lock memory on macOS.
pub(super) fn lock_memory() -> anyhow::Result<()> {
    // NOT IMPLEMENTED on macOS: this never called `mlock`/`mlockall`. Return
    // `Err` so RtConfig records an honest `RtDegradation::MemoryLockUnavailable`
    // rather than silently claiming memory was locked. (macOS supports per-page
    // `mlock` but not `mlockall`; a real impl would mlock specific pages.)
    anyhow::bail!("macOS memory locking (mlock/mlockall) is not yet implemented")
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
