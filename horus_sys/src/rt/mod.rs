//! Real-time scheduling — priority, memory locking, CPU affinity.
//!
//! Provides platform-specific RT primitives:
//! - **Linux**: `SCHED_FIFO` + `mlockall` + `sched_setaffinity`
//! - **macOS**: `THREAD_TIME_CONSTRAINT_POLICY` via Mach
//! - **Windows**: `REALTIME_PRIORITY_CLASS` + `SetThreadPriority`
//!
//! Higher-level types (RtConfig, RuntimeCapabilities) remain in horus_core.
//! This module provides the raw platform primitives they call.

use std::time::Duration;

#[cfg(target_os = "linux")]
mod linux;
#[cfg(target_os = "macos")]
mod macos;
#[cfg(target_os = "windows")]
mod windows;

// ============================================================================
// RT Capabilities (platform-aware detection)
// ============================================================================

/// Platform RT capabilities detected at startup.
#[derive(Debug, Clone)]
pub struct RtCapabilities {
    /// PREEMPT_RT kernel detected (Linux) or RT-capable scheduler
    pub preempt_rt: bool,
    /// Maximum RT priority available (0 if none)
    pub max_priority: i32,
    /// Minimum RT priority
    pub min_priority: i32,
    /// Whether memory locking is permitted
    pub memory_locking: bool,
    /// Whether CPU affinity can be set
    pub cpu_affinity: bool,
    /// Kernel version string
    pub kernel_version: String,
    /// Number of available CPUs
    pub cpu_count: usize,
    /// Estimated scheduling jitter
    pub estimated_jitter: Duration,
}

impl Default for RtCapabilities {
    fn default() -> Self {
        Self {
            preempt_rt: false,
            max_priority: 0,
            min_priority: 0,
            memory_locking: false,
            cpu_affinity: true, // core_affinity crate is cross-platform
            kernel_version: String::new(),
            cpu_count: std::thread::available_parallelism()
                .map(|p| p.get())
                .unwrap_or(1),
            estimated_jitter: Duration::from_millis(10),
        }
    }
}

/// Detect platform RT capabilities.
pub fn detect_capabilities() -> RtCapabilities {
    #[cfg(target_os = "linux")]
    {
        linux::detect_capabilities()
    }
    #[cfg(target_os = "macos")]
    {
        macos::detect_capabilities()
    }
    #[cfg(target_os = "windows")]
    {
        windows::detect_capabilities()
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        RtCapabilities::default()
    }
}

// ============================================================================
// RT Scheduling Primitives
// ============================================================================

/// Set real-time scheduling priority for the current thread.
///
/// - Linux: `sched_setscheduler(SCHED_FIFO, priority)`
/// - macOS: `thread_policy_set(THREAD_TIME_CONSTRAINT_POLICY)`
/// - Windows: `SetPriorityClass(REALTIME_PRIORITY_CLASS)` + `SetThreadPriority(TIME_CRITICAL)`
pub fn set_realtime_priority(priority: i32) -> anyhow::Result<()> {
    #[cfg(target_os = "linux")]
    {
        linux::set_realtime_priority(priority)
    }
    #[cfg(target_os = "macos")]
    {
        macos::set_realtime_priority(priority)
    }
    #[cfg(target_os = "windows")]
    {
        windows::set_realtime_priority(priority)
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        let _ = priority;
        anyhow::bail!("RT scheduling not supported on this platform")
    }
}

/// Lock all current and future memory pages (prevent swapping).
///
/// - Linux: `mlockall(MCL_CURRENT | MCL_FUTURE)`
/// - macOS: best-effort mlock
/// - Windows: best-effort VirtualLock
pub fn lock_memory() -> anyhow::Result<()> {
    #[cfg(target_os = "linux")]
    {
        linux::lock_memory()
    }
    #[cfg(target_os = "macos")]
    {
        macos::lock_memory()
    }
    #[cfg(target_os = "windows")]
    {
        windows::lock_memory()
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        anyhow::bail!("Memory locking not supported on this platform")
    }
}

/// Pin the current thread to a specific CPU core.
///
/// Uses the `core_affinity` crate which is cross-platform.
pub fn pin_to_core(core_id: usize) -> anyhow::Result<()> {
    let core_ids =
        core_affinity::get_core_ids().ok_or_else(|| anyhow::anyhow!("Failed to get core IDs"))?;

    if core_id >= core_ids.len() {
        anyhow::bail!(
            "Core {} does not exist (max: {})",
            core_id,
            core_ids.len() - 1
        );
    }

    if core_affinity::set_for_current(core_ids[core_id]) {
        Ok(())
    } else {
        anyhow::bail!("Failed to pin to core {}", core_id)
    }
}

/// Pin the current thread to any of the specified cores (first success wins).
pub fn pin_to_cores(cores: &[usize]) -> anyhow::Result<()> {
    if cores.is_empty() {
        return Ok(());
    }

    let core_ids =
        core_affinity::get_core_ids().ok_or_else(|| anyhow::anyhow!("Failed to get core IDs"))?;

    for &core_idx in cores {
        if core_idx < core_ids.len() && core_affinity::set_for_current(core_ids[core_idx]) {
            return Ok(());
        }
    }

    anyhow::bail!("Failed to pin to any of cores {:?}", cores)
}

/// Get the list of available CPU core IDs.
pub fn available_cores() -> Vec<usize> {
    core_affinity::get_core_ids()
        .map(|ids| (0..ids.len()).collect())
        .unwrap_or_else(|| vec![0])
}

/// Detect isolated CPUs (Linux `isolcpus` parameter).
pub fn isolated_cores() -> Vec<usize> {
    #[cfg(target_os = "linux")]
    {
        linux::detect_isolated_cpus()
    }
    #[cfg(not(target_os = "linux"))]
    {
        Vec::new()
    }
}

/// Detect nohz_full CPUs (Linux tickless kernel).
pub fn nohz_full_cores() -> Vec<usize> {
    #[cfg(target_os = "linux")]
    {
        linux::detect_nohz_full_cpus()
    }
    #[cfg(not(target_os = "linux"))]
    {
        Vec::new()
    }
}

/// Detect the CPU frequency governor (Linux only).
pub fn cpu_governor() -> Option<String> {
    #[cfg(target_os = "linux")]
    {
        linux::detect_cpu_governor()
    }
    #[cfg(not(target_os = "linux"))]
    {
        None
    }
}

/// Set the CPU frequency governor for a specific core.
///
/// Sets `/sys/devices/system/cpu/cpu{N}/cpufreq/scaling_governor`.
/// No-op on non-Linux platforms.
pub fn set_cpu_governor(cpu_id: usize, governor: &str) -> anyhow::Result<()> {
    #[cfg(target_os = "linux")]
    {
        linux::set_cpu_governor(cpu_id, governor)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = (cpu_id, governor);
        Ok(())
    }
}

/// Move hardware interrupts off the specified CPU cores.
///
/// Returns the number of IRQs whose affinity was changed.
/// No-op on non-Linux platforms.
pub fn move_irqs_off_cpus(cpus: &[usize]) -> anyhow::Result<usize> {
    #[cfg(target_os = "linux")]
    {
        linux::move_irqs_off_cpus(cpus)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = cpus;
        Ok(0)
    }
}

/// Set SCHED_DEADLINE (EDF) scheduling for the current thread.
///
/// Gives kernel-guaranteed CPU bandwidth: `runtime_ns` of CPU every `period_ns`.
/// Falls back gracefully — callers should try SCHED_FIFO on failure.
pub fn set_deadline_scheduling(
    runtime_ns: u64,
    deadline_ns: u64,
    period_ns: u64,
) -> anyhow::Result<()> {
    #[cfg(target_os = "linux")]
    {
        linux::set_deadline_scheduling(runtime_ns, deadline_ns, period_ns)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = (runtime_ns, deadline_ns, period_ns);
        anyhow::bail!("SCHED_DEADLINE is Linux-only")
    }
}

/// Check if SCHED_DEADLINE is available on this kernel.
pub fn has_deadline_capability() -> bool {
    #[cfg(target_os = "linux")]
    {
        linux::has_deadline_capability()
    }
    #[cfg(not(target_os = "linux"))]
    {
        false
    }
}

/// Check whether RT priority can actually be set (dry-run test).
pub fn can_set_rt_priority() -> bool {
    #[cfg(target_os = "linux")]
    {
        linux::can_set_rt_priority()
    }
    #[cfg(target_os = "macos")]
    {
        true // macOS doesn't require special privileges for thread policy
    }
    #[cfg(target_os = "windows")]
    {
        true // Will try SetPriorityClass at runtime
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        false
    }
}

/// Get the RLIMIT_MEMLOCK soft limit in bytes (Linux only, u64::MAX if unlimited).
pub fn memlock_limit_bytes() -> u64 {
    #[cfg(target_os = "linux")]
    {
        linux::get_memlock_limit()
    }
    #[cfg(not(target_os = "linux"))]
    {
        0
    }
}

/// Pre-fault stack memory to avoid page faults during execution.
///
/// Platform-agnostic: touches stack pages using recursive allocation + `black_box`.
#[inline(never)]
pub fn prefault_stack(size: usize) {
    const PAGE_SIZE: usize = 4096;
    let num_pages = size.div_ceil(PAGE_SIZE);
    prefault_stack_recursive(num_pages, 0);
}

#[inline(never)]
fn prefault_stack_recursive(remaining_pages: usize, depth: usize) {
    if remaining_pages == 0 || depth >= 4096 {
        return;
    }

    let mut buffer: [u8; 4096] = [0u8; 4096];
    for i in (0..4096).step_by(64) {
        buffer[i] = std::hint::black_box(i as u8);
    }
    std::hint::black_box(&buffer);

    prefault_stack_recursive(remaining_pages - 1, depth + 1);
}

/// Parse a CPU list string like "0-3,7,9-11" into individual CPU indices.
pub fn parse_cpu_list(s: &str) -> Vec<usize> {
    let mut cpus = Vec::new();
    for part in s.split(',') {
        let part = part.trim();
        if part.is_empty() {
            continue;
        }
        if let Some((start, end)) = part.split_once('-') {
            if let (Ok(s), Ok(e)) = (start.trim().parse::<usize>(), end.trim().parse::<usize>()) {
                cpus.extend(s..=e);
            }
        } else if let Ok(cpu) = part.parse::<usize>() {
            cpus.push(cpu);
        }
    }
    cpus
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_capabilities() {
        let caps = detect_capabilities();
        assert!(caps.cpu_count >= 1);
        assert!(!caps.kernel_version.is_empty() || !cfg!(target_os = "linux"));
    }

    #[test]
    fn test_available_cores() {
        let cores = available_cores();
        assert!(!cores.is_empty());
    }

    #[test]
    fn test_parse_cpu_list() {
        assert_eq!(parse_cpu_list("0-3"), vec![0, 1, 2, 3]);
        assert_eq!(parse_cpu_list("0,2,4"), vec![0, 2, 4]);
        assert_eq!(parse_cpu_list("0-2,5,7-9"), vec![0, 1, 2, 5, 7, 8, 9]);
        assert_eq!(parse_cpu_list(""), Vec::<usize>::new());
    }

    #[test]
    fn test_prefault_stack_does_not_panic() {
        prefault_stack(8192); // 2 pages
    }

    #[test]
    fn test_can_set_rt_priority_returns() {
        let _ = can_set_rt_priority(); // just verify it doesn't panic
    }

    #[test]
    fn test_pin_to_cores_empty_is_ok() {
        pin_to_cores(&[]).unwrap();
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn test_capabilities_linux() {
        let caps = detect_capabilities();
        assert!(caps.cpu_count >= 1);
        assert!(caps.max_priority > 0); // Linux always has RT priorities
        assert!(!caps.kernel_version.is_empty());
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn test_isolated_and_nohz_cores() {
        // These may be empty on most systems, but shouldn't panic
        let _ = isolated_cores();
        let _ = nohz_full_cores();
    }

    // ── Capability detection intent tests ────────────────────────────

    #[test]
    fn test_capabilities_cpu_count_matches_parallelism() {
        let caps = detect_capabilities();
        let expected = std::thread::available_parallelism()
            .map(|p| p.get())
            .unwrap_or(1);
        assert_eq!(
            caps.cpu_count, expected,
            "cpu_count should match available_parallelism"
        );
    }

    #[test]
    fn test_capabilities_cpu_affinity_is_true() {
        // core_affinity crate works on all platforms
        let caps = detect_capabilities();
        assert!(
            caps.cpu_affinity,
            "cpu_affinity should be true (core_affinity crate is cross-platform)"
        );
    }

    #[test]
    fn test_capabilities_jitter_is_positive() {
        let caps = detect_capabilities();
        assert!(
            !caps.estimated_jitter.is_zero(),
            "estimated jitter should be > 0"
        );
    }

    // ── parse_cpu_list edge cases ───────────────────────────────────

    #[test]
    fn test_parse_cpu_list_single_cpu() {
        assert_eq!(parse_cpu_list("7"), vec![7]);
    }

    #[test]
    fn test_parse_cpu_list_whitespace() {
        assert_eq!(parse_cpu_list(" 0 - 2 , 5 "), vec![0, 1, 2, 5]);
    }

    #[test]
    fn test_parse_cpu_list_invalid_entries_skipped() {
        assert_eq!(parse_cpu_list("0,abc,2"), vec![0, 2]);
    }

    #[test]
    fn test_parse_cpu_list_trailing_comma() {
        assert_eq!(parse_cpu_list("0,1,"), vec![0, 1]);
    }

    // ── CPU pinning tests ───────────────────────────────────────────

    #[test]
    fn test_pin_to_core_zero_succeeds() {
        // Core 0 should always exist
        pin_to_core(0).unwrap();
    }

    #[test]
    fn test_pin_to_core_invalid_fails() {
        let result = pin_to_core(99999);
        assert!(result.is_err(), "pinning to nonexistent core should fail");
    }

    #[test]
    fn test_pin_to_cores_first_valid_wins() {
        // First core (99999) doesn't exist, second (0) does
        pin_to_cores(&[99999, 0]).unwrap();
    }

    // ── prefault_stack intent test ──────────────────────────────────

    #[test]
    fn test_prefault_stack_larger_size() {
        // 64KB = 16 pages — shouldn't overflow stack or panic
        prefault_stack(65536);
    }

    // ── memlock_limit ───────────────────────────────────────────────

    #[test]
    fn test_memlock_limit_returns_value() {
        let limit = memlock_limit_bytes();
        // On Linux: some positive value or u64::MAX. On non-Linux: 0
        #[cfg(target_os = "linux")]
        assert!(limit > 0, "Linux should have a memlock limit");
        let _ = limit;
    }

    // ── cpu_governor ────────────────────────────────────────────────

    #[test]
    fn test_cpu_governor_does_not_panic() {
        let gov = cpu_governor();
        // May be Some("performance") or None, but shouldn't panic
        let _ = gov;
    }
}
