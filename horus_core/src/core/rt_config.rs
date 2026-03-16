//! Real-time configuration builder for HORUS nodes.
//!
//! This module provides an ergonomic builder API for configuring real-time
//! execution parameters including memory locking, thread priorities, and
//! CPU affinity. The configuration is optional and gracefully degrades
//! on systems without PREEMPT_RT kernel support.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::core::{RtConfig, RtScheduler};
//!
//! // Create RT configuration for a critical control loop
//! let config = RtConfig::builder()
//!     .memory_locked(true)
//!     .priority(80)
//!     .scheduler(RtScheduler::Fifo)
//!     .cpu_affinity(&[2, 3])
//!     .build();
//!
//! // Apply to current thread
//! config.apply().expect("Failed to apply RT config");
//! ```
//!
//! # Zero-Cost Abstraction
//!
//! When no RT features are enabled, the configuration is essentially a no-op
//! with zero runtime overhead.

use std::hint::black_box;
use std::io;

/// Real-time scheduler policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) enum RtScheduler {
    /// Normal (non-RT) scheduler - SCHED_OTHER
    #[default]
    Normal,
    /// First-In-First-Out RT scheduler - SCHED_FIFO
    /// Highest priority, no time slicing
    Fifo,
}

/// Result of applying RT configuration.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum RtApplyResult {
    /// All requested features were applied successfully
    FullSuccess,
    /// Configuration applied with some features degraded
    Degraded(Vec<RtDegradation>),
}

/// Describes a degradation from the requested configuration.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum RtDegradation {
    /// Memory locking was requested but not available
    MemoryLockUnavailable(String),
    /// Requested priority was clamped to system maximum
    PriorityClamped { requested: i32, actual: i32 },
    /// RT scheduler not available, using normal scheduler
    SchedulerDegraded(String),
    /// CPU affinity could not be set
    AffinityUnavailable(String),
    /// PREEMPT_RT kernel not detected
    NoPreemptRt,
}

/// Information about kernel RT support.
#[derive(Debug, Clone, Default)]
pub(crate) struct RtKernelInfo {
    /// Whether PREEMPT_RT patch is detected
    pub preempt_rt: bool,
    /// Kernel version string
    pub kernel_version: String,
    /// Maximum RT priority available
    pub max_rt_priority: i32,
    /// Minimum RT priority available
    pub min_rt_priority: i32,
    /// Whether memory locking is permitted
    pub mlockall_permitted: bool,
    /// Available CPU count (used in tests and for diagnostics)
    #[allow(dead_code)]
    pub cpu_count: usize,
}

impl RtKernelInfo {
    /// Detect kernel RT capabilities (delegates to horus_sys::rt).
    pub fn detect() -> Self {
        let caps = horus_sys::rt::detect_capabilities();
        Self {
            preempt_rt: caps.preempt_rt,
            kernel_version: caps.kernel_version,
            max_rt_priority: caps.max_priority,
            min_rt_priority: caps.min_priority,
            mlockall_permitted: caps.memory_locking,
            cpu_count: caps.cpu_count,
        }
    }
}

/// Builder for real-time configuration.
///
/// Use this builder to construct an [`RtConfig`] with the desired
/// real-time parameters. The builder uses sensible defaults and
/// gracefully degrades on systems without full RT support.
#[derive(Debug, Clone, Default)]
pub(crate) struct RtConfigBuilder {
    memory_locked: bool,
    priority: Option<i32>,
    scheduler: RtScheduler,
    cpu_affinity: Option<Vec<usize>>,
    warn_on_degradation: bool,
}

impl RtConfigBuilder {
    /// Create a new RT configuration builder with default settings.
    pub fn new() -> Self {
        Self::default()
    }

    /// Enable or disable memory locking via `mlockall()`.
    ///
    /// When enabled, all current and future memory pages are locked
    /// into RAM, preventing page faults during execution.
    ///
    /// Requires appropriate permissions (typically `CAP_IPC_LOCK` or
    /// running as root).
    pub fn memory_locked(mut self, locked: bool) -> Self {
        self.memory_locked = locked;
        self
    }

    /// Set the thread priority (1-99 for RT schedulers).
    ///
    /// Higher values indicate higher priority. The value will be
    /// clamped to the system's allowed range.
    pub fn priority(mut self, priority: i32) -> Self {
        self.priority = Some(priority);
        self
    }

    /// Set the scheduler policy.
    ///
    /// - [`RtScheduler::Normal`]: Standard Linux scheduler (SCHED_OTHER)
    /// - [`RtScheduler::Fifo`]: Real-time FIFO (SCHED_FIFO)
    /// - [`RtScheduler::RoundRobin`]: Real-time round-robin (SCHED_RR)
    pub fn scheduler(mut self, scheduler: RtScheduler) -> Self {
        self.scheduler = scheduler;
        self
    }

    /// Set CPU affinity to pin the thread to specific cores.
    ///
    /// This is useful for ensuring consistent cache behavior and
    /// avoiding migration overhead.
    pub fn cpu_affinity(mut self, cpus: &[usize]) -> Self {
        self.cpu_affinity = Some(cpus.to_vec());
        self
    }

    /// Log warnings when configuration is degraded.
    pub fn warn_on_degradation(mut self, warn: bool) -> Self {
        self.warn_on_degradation = warn;
        self
    }

    /// Build the RT configuration.
    pub fn build(self) -> RtConfig {
        RtConfig {
            memory_locked: self.memory_locked,
            priority: self.priority,
            scheduler: self.scheduler,
            cpu_affinity: self.cpu_affinity,
            warn_on_degradation: self.warn_on_degradation,
        }
    }
}

/// Real-time configuration for HORUS nodes.
///
/// This struct holds the configuration parameters and provides
/// methods to apply them to the current thread or process.
#[derive(Debug, Clone, Default)]
pub(crate) struct RtConfig {
    memory_locked: bool,
    priority: Option<i32>,
    scheduler: RtScheduler,
    cpu_affinity: Option<Vec<usize>>,
    warn_on_degradation: bool,
}

impl RtConfig {
    /// Create a new RT configuration builder.
    pub fn builder() -> RtConfigBuilder {
        RtConfigBuilder::new()
    }

    /// Apply this configuration to the current thread.
    ///
    /// Delegates platform-specific operations to horus_sys::rt.
    pub fn apply(&self) -> Result<RtApplyResult, io::Error> {
        let mut degradations = Vec::new();
        let kernel_info = RtKernelInfo::detect();

        // Check for PREEMPT_RT if using RT scheduler
        if self.scheduler != RtScheduler::Normal && !kernel_info.preempt_rt {
            degradations.push(RtDegradation::NoPreemptRt);
            if self.warn_on_degradation {
                eprintln!("Warning: PREEMPT_RT kernel not detected, RT guarantees may not be met");
            }
        }

        // Check CPU governor when RT features are requested
        let has_rt_features = self.scheduler != RtScheduler::Normal
            || self.memory_locked
            || self.cpu_affinity.is_some();
        if has_rt_features {
            if let Some(gov) = horus_sys::rt::cpu_governor() {
                if gov != "performance" && self.warn_on_degradation {
                    eprintln!(
                        "Warning: CPU governor is '{}' (want 'performance'). \
                         Run: sudo ./scripts/setup-realtime.sh",
                        gov
                    );
                }
            }
        }

        // Apply memory locking via horus_sys
        if self.memory_locked {
            if let Err(e) = horus_sys::rt::lock_memory() {
                let msg = format!("memory lock failed: {}", e);
                degradations.push(RtDegradation::MemoryLockUnavailable(msg.clone()));
                if self.warn_on_degradation {
                    eprintln!("Warning: {}", msg);
                }
            }
        }

        // Apply scheduler and priority via horus_sys
        if let Some(priority) = self.priority {
            if self.scheduler != RtScheduler::Normal {
                let actual = priority.clamp(kernel_info.min_rt_priority, kernel_info.max_rt_priority);
                match horus_sys::rt::set_realtime_priority(actual) {
                    Ok(()) => {
                        if actual != priority {
                            degradations.push(RtDegradation::PriorityClamped {
                                requested: priority,
                                actual,
                            });
                        }
                    }
                    Err(e) => {
                        let msg = format!("Scheduler setup failed: {}", e);
                        degradations.push(RtDegradation::SchedulerDegraded(msg.clone()));
                        if self.warn_on_degradation {
                            eprintln!("Warning: {}", msg);
                        }
                    }
                }
            }
        }

        // Apply CPU affinity via horus_sys
        if let Some(ref cpus) = self.cpu_affinity {
            if let Err(e) = horus_sys::rt::pin_to_cores(cpus) {
                let msg = format!("CPU affinity failed: {}", e);
                degradations.push(RtDegradation::AffinityUnavailable(msg.clone()));
                if self.warn_on_degradation {
                    eprintln!("Warning: {}", msg);
                }
            }
        }

        if degradations.is_empty() {
            Ok(RtApplyResult::FullSuccess)
        } else {
            Ok(RtApplyResult::Degraded(degradations))
        }
    }
}

// ============================================================================
// CPU Governor Detection
// ============================================================================

// CPU governor detection delegated to horus_sys::rt::cpu_governor()

#[cfg(test)]
impl RtConfig {
    /// Get the current thread's scheduler and priority (test-only).
    #[cfg(target_os = "linux")]
    pub fn get_current_scheduler() -> Result<(RtScheduler, i32), io::Error> {
        // SAFETY: sched_getscheduler and sched_getparam are safe libc calls
        unsafe {
            let policy = libc::sched_getscheduler(0);
            if policy < 0 {
                return Err(io::Error::last_os_error());
            }

            let mut param = libc::sched_param { sched_priority: 0 };
            if libc::sched_getparam(0, &mut param) < 0 {
                return Err(io::Error::last_os_error());
            }

            let scheduler = match policy {
                libc::SCHED_FIFO => RtScheduler::Fifo,
                _ => RtScheduler::Normal,
            };

            Ok((scheduler, param.sched_priority))
        }
    }

    #[cfg(not(target_os = "linux"))]
    pub fn get_current_scheduler() -> Result<(RtScheduler, i32), io::Error> {
        Ok((RtScheduler::Normal, 0))
    }

    /// Get the current thread's CPU affinity (test-only).
    #[cfg(target_os = "linux")]
    pub fn get_current_affinity() -> Result<Vec<usize>, io::Error> {
        // SAFETY: sched_getaffinity and CPU_ISSET are safe libc calls
        unsafe {
            let mut cpuset: libc::cpu_set_t = std::mem::zeroed();

            let result =
                libc::sched_getaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &mut cpuset);

            if result < 0 {
                return Err(io::Error::last_os_error());
            }

            let cpu_count = std::thread::available_parallelism()
                .map(|p| p.get())
                .unwrap_or(1);

            let mut cpus = Vec::new();
            for cpu in 0..cpu_count {
                if libc::CPU_ISSET(cpu, &cpuset) {
                    cpus.push(cpu);
                }
            }

            Ok(cpus)
        }
    }

    #[cfg(not(target_os = "linux"))]
    pub fn get_current_affinity() -> Result<Vec<usize>, io::Error> {
        let cpu_count = std::thread::available_parallelism()
            .map(|p| p.get())
            .unwrap_or(1);
        Ok((0..cpu_count).collect())
    }
}

/// Prefault the stack by touching all pages up to the specified size.
///
/// This function allocates a buffer on the stack and touches every page
/// to ensure all stack pages are mapped into physical memory. This prevents
/// minor page faults during RT-critical code execution.
///
/// # Arguments
/// * `size` - Stack size in bytes to prefault. This should match or be smaller
///   than your thread's actual stack size.
///
/// # Performance
/// - Uses `black_box` to prevent compiler optimization from removing the touches
/// - Touches pages in 4KB increments (standard page size)
/// - Runs in O(size / 4096) time
///
/// # Example
/// ```rust,ignore
/// use horus_core::core::prefault_stack;
///
/// // Prefault 512KB of stack space
/// prefault_stack(512 * 1024);
/// ```
///
/// # Safety
/// This function is safe but should be called at thread initialization
/// before entering RT-critical sections, not during normal execution.
#[inline(never)] // Prevent inlining to ensure stack allocation happens
pub(crate) fn prefault_stack(size: usize) {
    // Page size is typically 4KB on most systems
    const PAGE_SIZE: usize = 4096;

    // Calculate number of pages to touch
    let num_pages = size.div_ceil(PAGE_SIZE);

    // We use a recursive approach with a small fixed-size buffer per call
    // to avoid stack overflow from trying to allocate too much at once
    prefault_stack_recursive(num_pages, 0);
}

/// Recursive helper for stack prefaulting.
///
/// Touches stack pages by allocating small buffers recursively.
/// Each recursion level touches one page worth of stack.
#[inline(never)]
fn prefault_stack_recursive(remaining_pages: usize, depth: usize) {
    if remaining_pages == 0 {
        return;
    }

    // Prevent infinite recursion in case of bugs
    const MAX_DEPTH: usize = 4096; // ~16MB max stack
    if depth >= MAX_DEPTH {
        return;
    }

    // Allocate a page-sized buffer on the stack
    // This forces the OS to map this stack page
    let mut buffer: [u8; 4096] = [0u8; 4096];

    // Touch the buffer to ensure the page is actually faulted in
    // Use black_box to prevent the compiler from optimizing this away
    for i in (0..4096).step_by(64) {
        // Touch every cache line (64 bytes) to ensure full page is loaded
        buffer[i] = black_box(i as u8);
    }

    // Use black_box on the buffer to prevent dead code elimination
    black_box(&buffer);

    // Recurse to prefault the next page
    prefault_stack_recursive(remaining_pages - 1, depth + 1);
}

// ============================================================================
// STANDALONE HELPER FUNCTIONS FOR CPU ISOLATION DETECTION
// ============================================================================

/// Detect CPUs isolated via `isolcpus` kernel parameter (delegates to horus_sys::rt).
pub(crate) fn detect_isolated_cpus() -> Vec<usize> {
    horus_sys::rt::isolated_cores()
}

/// Detect CPUs with `nohz_full` tickless kernel (delegates to horus_sys::rt).
pub(crate) fn detect_nohz_full_cpus() -> Vec<usize> {
    horus_sys::rt::nohz_full_cores()
}

/// Get recommended CPUs for real-time HORUS tasks.
///
/// Returns CPUs in order of preference for RT workloads:
/// 1. CPUs that are both isolated AND nohz_full (best)
/// 2. CPUs that are isolated only (good)
/// 3. CPUs that are nohz_full only (acceptable)
/// 4. Higher-numbered CPUs if no special configuration (least interference)
///
/// # Arguments
/// * `count` - Number of CPUs needed (returns up to this many)
///
/// # Returns
/// * `Vec<usize>` - Recommended CPU indices in preference order
///
/// # Example
/// ```rust,ignore
/// use horus_core::core::get_rt_recommended_cpus;
///
/// // Get 2 CPUs for RT tasks
/// let rt_cpus = get_rt_recommended_cpus(2);
/// for (i, cpu) in rt_cpus.iter().enumerate() {
///     println!("RT thread {} -> CPU {}", i, cpu);
/// }
/// ```
pub(crate) fn get_rt_recommended_cpus(count: usize) -> Vec<usize> {
    let isolated = detect_isolated_cpus();
    let nohz = detect_nohz_full_cpus();

    let mut recommended = Vec::with_capacity(count);

    // Priority 1: CPUs that are both isolated AND nohz_full
    for &cpu in &isolated {
        if nohz.contains(&cpu) && !recommended.contains(&cpu) {
            recommended.push(cpu);
            if recommended.len() >= count {
                return recommended;
            }
        }
    }

    // Priority 2: CPUs that are isolated only
    for &cpu in &isolated {
        if !recommended.contains(&cpu) {
            recommended.push(cpu);
            if recommended.len() >= count {
                return recommended;
            }
        }
    }

    // Priority 3: CPUs that are nohz_full only
    for &cpu in &nohz {
        if !recommended.contains(&cpu) {
            recommended.push(cpu);
            if recommended.len() >= count {
                return recommended;
            }
        }
    }

    // Priority 4: Higher-numbered CPUs (less likely to be used by system services)
    let cpu_count = std::thread::available_parallelism()
        .map(|p| p.get())
        .unwrap_or(1);

    // Start from highest CPU and work down
    for cpu in (0..cpu_count).rev() {
        if !recommended.contains(&cpu) {
            recommended.push(cpu);
            if recommended.len() >= count {
                return recommended;
            }
        }
    }

    recommended
}

/// Get detailed RT CPU configuration information.
///
/// Returns a summary of the system's real-time CPU configuration
/// including isolated CPUs, nohz_full CPUs, and recommendations.
#[derive(Debug, Clone)]
pub(crate) struct RtCpuInfo {
    /// Total number of CPUs
    pub total_cpus: usize,
    /// CPUs isolated via isolcpus kernel parameter
    pub isolated_cpus: Vec<usize>,
    /// CPUs with tickless operation (nohz_full)
    pub nohz_full_cpus: Vec<usize>,
    /// Recommended CPUs for RT workloads (in preference order)
    pub recommended_rt_cpus: Vec<usize>,
}

impl RtCpuInfo {
    /// Detect current system RT CPU configuration.
    pub fn detect() -> Self {
        let total_cpus = std::thread::available_parallelism()
            .map(|p| p.get())
            .unwrap_or(1);
        let isolated_cpus = detect_isolated_cpus();
        let nohz_full_cpus = detect_nohz_full_cpus();
        let recommended_rt_cpus = get_rt_recommended_cpus(total_cpus);

        RtCpuInfo {
            total_cpus,
            isolated_cpus,
            nohz_full_cpus,
            recommended_rt_cpus,
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_builder_default() {
        let config = RtConfig::builder().build();
        assert!(!config.memory_locked);
        assert_eq!(config.priority, None);
        assert_eq!(config.scheduler, RtScheduler::Normal);
        assert!(config.cpu_affinity.is_none());
    }

    #[test]
    fn test_builder_full() {
        let config = RtConfig::builder()
            .memory_locked(true)
            .priority(80)
            .scheduler(RtScheduler::Fifo)
            .cpu_affinity(&[0, 1])
            .warn_on_degradation(true)
            .build();

        assert!(config.memory_locked);
        assert_eq!(config.priority, Some(80));
        assert_eq!(config.scheduler, RtScheduler::Fifo);
        assert_eq!(config.cpu_affinity, Some(vec![0, 1]));
        assert!(config.warn_on_degradation);
    }

    #[test]
    fn test_kernel_info_detect() {
        let info = RtKernelInfo::detect();
        assert!(info.cpu_count > 0);
        assert!(
            !info.kernel_version.is_empty(),
            "kernel_version should be non-empty"
        );
        // Verify RT priority range is sane
        assert!(
            info.max_rt_priority >= info.min_rt_priority,
            "max_rt_priority ({}) should >= min_rt_priority ({})",
            info.max_rt_priority,
            info.min_rt_priority
        );
    }

    #[test]
    fn test_get_current_scheduler() {
        let (scheduler, priority) = RtConfig::get_current_scheduler().unwrap();
        // Priority should be non-negative
        assert!(priority >= 0, "Priority should be non-negative, got {}", priority);
        // Verify the enum variant is one of the known policies
        match scheduler {
            RtScheduler::Normal | RtScheduler::Fifo => {}
        }
    }

    #[test]
    fn test_get_current_affinity() {
        // Should not fail
        let result = RtConfig::get_current_affinity();
        assert!(result.is_ok());
        let cpus = result.unwrap();
        assert!(!cpus.is_empty());
    }

    #[test]
    fn test_default_config_apply() {
        // Default config should always succeed
        let config = RtConfig::default();
        let result = config.apply();
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), RtApplyResult::FullSuccess);
    }

    #[test]
    fn test_prefault_stack_small() {
        // Prefault a small stack (16KB = 4 pages)
        // This should not panic
        super::prefault_stack(16 * 1024);
    }

    #[test]
    fn test_prefault_stack_medium() {
        // Prefault a medium stack (128KB = 32 pages)
        // This should not panic
        super::prefault_stack(128 * 1024);
    }
}
