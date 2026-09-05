// macOS RT: Mach THREAD_TIME_CONSTRAINT_POLICY

use super::RtCapabilities;
use std::time::Duration;

/// Detect macOS RT capabilities.
pub(super) fn detect_capabilities() -> RtCapabilities {
    let cpu_count = std::thread::available_parallelism()
        .map(|p| p.get())
        .unwrap_or(1);

    RtCapabilities {
        // Derived from the one function that owns this policy rather than
        // restated here, so the two cannot drift if the macOS probe ever stops
        // being "no special privilege required". (`super::can_set_rt_priority`
        // does not dispatch back into this module, so there is no recursion.)
        //
        // Both this and `memory_locking` below used to report `true` while the
        // functions they describe -- `set_realtime_priority` and `lock_memory`,
        // twenty lines down in this same file -- returned `Err` on every call.
        // That is not a cosmetic inconsistency. `Scheduler::require_rt()`
        // asserts on `rt_priority_available || mlockall_permitted`, so two
        // hardcoded `true`s made `require_rt()` pass unconditionally on macOS
        // and then apply neither RT scheduling nor memory locking -- the exact
        // "silently running non-RT while you believe otherwise" outcome
        // `require_rt()` exists to prevent.
        rt_priority_permitted: super::can_set_rt_priority(),
        preempt_rt: false,
        max_priority: 99, // macOS supports thread priority via Mach
        min_priority: 1,
        // `lock_memory()` below is unimplemented and always returns `Err`.
        // Report that honestly; flip this back to `true` in the same commit
        // that makes `lock_memory()` actually call `mlock`.
        memory_locking: false,
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
    anyhow::bail!(
        "macOS real-time scheduling (THREAD_TIME_CONSTRAINT_POLICY) is not yet implemented"
    )
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

/// Put the calling thread on the ordinary time-shared class — an honest no-op
/// on macOS.
///
/// Nothing here is left undone by omission:
///
/// * There is no policy to demote from. [`set_realtime_priority`] on this
///   backend returns `Err` on every call — `THREAD_TIME_CONSTRAINT_POLICY` is
///   not wired up — so no thread in a HORUS process is ever promoted, and a
///   child has nothing to inherit.
/// * There is no placement to undo. Mach offers `THREAD_AFFINITY_POLICY`, an
///   advisory *cache affinity tag* rather than a placement constraint, and it
///   is unimplemented on Apple Silicon. `core_affinity` sets that tag; it does
///   not confine a thread to a CPU, so a helper is not crowding an RT core the
///   way it would on Linux.
/// * `nice(2)` on Darwin adjusts the **process**, not the calling thread.
///   Applying the increment here would slow the RT thread down along with the
///   helper — the opposite of the intent.
///
/// [`crate::rt::helper_thread_cpus`] and
/// [`crate::rt::capture_helper_baseline_cpus`] still work on this platform, so
/// the cross-platform tests have something to assert on.
pub(super) fn set_best_effort_class(
    _nice_increment: i32,
    _target: &[usize],
) -> super::BestEffortReport {
    super::BestEffortReport::default()
}
