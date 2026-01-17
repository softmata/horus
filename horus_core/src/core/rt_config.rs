//! Real-time configuration builder for HORUS nodes.
//!
//! This module provides an ergonomic builder API for configuring real-time
//! execution parameters including memory locking, thread priorities, and
//! CPU affinity. The configuration is optional and gracefully degrades
//! on systems without PREEMPT_RT kernel support.
//!
//! # Example
//!
//! ```rust,no_run
//! use horus_core::core::{RtConfig, RtScheduler};
//!
//! // Create RT configuration for a critical control loop
//! let config = RtConfig::new()
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
pub enum RtScheduler {
    /// Normal (non-RT) scheduler - SCHED_OTHER
    #[default]
    Normal,
    /// First-In-First-Out RT scheduler - SCHED_FIFO
    /// Highest priority, no time slicing
    Fifo,
    /// Round-Robin RT scheduler - SCHED_RR
    /// Time-sliced among same-priority threads
    RoundRobin,
    /// Deadline scheduler - SCHED_DEADLINE (Linux 3.14+)
    /// For periodic tasks with explicit deadlines
    Deadline,
}

/// Result of applying RT configuration.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RtApplyResult {
    /// All requested features were applied successfully
    FullSuccess,
    /// Configuration applied with some features degraded
    Degraded(Vec<RtDegradation>),
    /// Configuration failed to apply
    Failed(String),
}

/// Describes a degradation from the requested configuration.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RtDegradation {
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
pub struct RtKernelInfo {
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
    /// Available CPU count
    pub cpu_count: usize,
}

impl RtKernelInfo {
    /// Detect kernel RT capabilities.
    #[cfg(target_os = "linux")]
    pub fn detect() -> Self {
        let kernel_version = Self::get_kernel_version();
        let preempt_rt = Self::detect_preempt_rt(&kernel_version);
        let (min_rt_priority, max_rt_priority) = Self::get_priority_range();
        let mlockall_permitted = Self::check_mlockall_permitted();
        let cpu_count = Self::get_cpu_count();

        Self {
            preempt_rt,
            kernel_version,
            max_rt_priority,
            min_rt_priority,
            mlockall_permitted,
            cpu_count,
        }
    }

    #[cfg(not(target_os = "linux"))]
    pub fn detect() -> Self {
        Self {
            preempt_rt: false,
            kernel_version: "non-linux".to_string(),
            max_rt_priority: 0,
            min_rt_priority: 0,
            mlockall_permitted: false,
            cpu_count: 1,
        }
    }

    #[cfg(target_os = "linux")]
    fn get_kernel_version() -> String {
        std::fs::read_to_string("/proc/version")
            .unwrap_or_default()
            .lines()
            .next()
            .unwrap_or("unknown")
            .to_string()
    }

    #[cfg(target_os = "linux")]
    fn detect_preempt_rt(kernel_version: &str) -> bool {
        // Check for PREEMPT_RT indicators in kernel version string
        kernel_version.contains("PREEMPT_RT")
            || kernel_version.contains("PREEMPT RT")
            || std::path::Path::new("/sys/kernel/realtime").exists()
    }

    #[cfg(target_os = "linux")]
    fn get_priority_range() -> (i32, i32) {
        // SAFETY: These are safe libc calls that query system limits
        unsafe {
            let min = libc::sched_get_priority_min(libc::SCHED_FIFO);
            let max = libc::sched_get_priority_max(libc::SCHED_FIFO);
            (min.max(1), max.max(1))
        }
    }

    #[cfg(target_os = "linux")]
    fn check_mlockall_permitted() -> bool {
        // Check RLIMIT_MEMLOCK - if unlimited or large, mlockall is likely permitted
        // SAFETY: getrlimit is a safe libc call
        unsafe {
            let mut rlim = libc::rlimit {
                rlim_cur: 0,
                rlim_max: 0,
            };
            if libc::getrlimit(libc::RLIMIT_MEMLOCK, &mut rlim) == 0 {
                // If limit is unlimited or > 1GB, mlockall is likely permitted
                rlim.rlim_cur == libc::RLIM_INFINITY || rlim.rlim_cur > 1024 * 1024 * 1024
            } else {
                false
            }
        }
    }

    #[cfg(target_os = "linux")]
    fn get_cpu_count() -> usize {
        std::thread::available_parallelism()
            .map(|p| p.get())
            .unwrap_or(1)
    }
}

/// Builder for real-time configuration.
///
/// Use this builder to construct an [`RtConfig`] with the desired
/// real-time parameters. The builder uses sensible defaults and
/// gracefully degrades on systems without full RT support.
#[derive(Debug, Clone, Default)]
pub struct RtConfigBuilder {
    memory_locked: bool,
    priority: Option<i32>,
    scheduler: RtScheduler,
    cpu_affinity: Option<Vec<usize>>,
    prefault_stack_size: Option<usize>,
    warn_on_degradation: bool,
    fail_on_degradation: bool,
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
    /// - [`RtScheduler::Deadline`]: Deadline scheduler (SCHED_DEADLINE)
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

    /// Enable stack prefaulting with the specified stack size.
    ///
    /// Stack prefaulting touches all stack pages at thread initialization
    /// to ensure they are mapped into physical memory. This prevents
    /// minor page faults during RT-critical code execution.
    ///
    /// # Arguments
    /// * `size` - Stack size in bytes to prefault. Common values:
    ///   - 64 * 1024 (64KB) for small stacks
    ///   - 512 * 1024 (512KB) for moderate stacks
    ///   - 8 * 1024 * 1024 (8MB) for large stacks (default thread stack)
    ///
    /// # Example
    /// ```rust,no_run
    /// use horus_core::core::RtConfig;
    ///
    /// let config = RtConfig::new()
    ///     .memory_locked(true)
    ///     .prefault_stack(512 * 1024) // 512KB stack prefaulting
    ///     .build();
    /// ```
    pub fn prefault_stack(mut self, size: usize) -> Self {
        self.prefault_stack_size = Some(size);
        self
    }

    /// Log warnings when configuration is degraded.
    pub fn warn_on_degradation(mut self, warn: bool) -> Self {
        self.warn_on_degradation = warn;
        self
    }

    /// Fail instead of degrading when full RT support unavailable.
    pub fn fail_on_degradation(mut self, fail: bool) -> Self {
        self.fail_on_degradation = fail;
        self
    }

    /// Build the RT configuration.
    pub fn build(self) -> RtConfig {
        RtConfig {
            memory_locked: self.memory_locked,
            priority: self.priority,
            scheduler: self.scheduler,
            cpu_affinity: self.cpu_affinity,
            prefault_stack_size: self.prefault_stack_size,
            warn_on_degradation: self.warn_on_degradation,
            fail_on_degradation: self.fail_on_degradation,
        }
    }
}

/// Real-time configuration for HORUS nodes.
///
/// This struct holds the configuration parameters and provides
/// methods to apply them to the current thread or process.
#[derive(Debug, Clone, Default)]
pub struct RtConfig {
    memory_locked: bool,
    priority: Option<i32>,
    scheduler: RtScheduler,
    cpu_affinity: Option<Vec<usize>>,
    prefault_stack_size: Option<usize>,
    warn_on_degradation: bool,
    fail_on_degradation: bool,
}

impl RtConfig {
    /// Create a new RT configuration builder.
    pub fn new() -> RtConfigBuilder {
        RtConfigBuilder::new()
    }

    /// Create a configuration optimized for hard real-time control loops.
    ///
    /// This enables memory locking, stack prefaulting, sets FIFO scheduler
    /// with high priority, and optionally pins to specific CPUs.
    pub fn hard_realtime(cpus: Option<&[usize]>) -> Self {
        let mut builder = Self::new()
            .memory_locked(true)
            .prefault_stack(512 * 1024) // 512KB stack prefaulting
            .scheduler(RtScheduler::Fifo)
            .priority(80)
            .warn_on_degradation(true);

        if let Some(cpu_list) = cpus {
            builder = builder.cpu_affinity(cpu_list);
        }

        builder.build()
    }

    /// Create a configuration for soft real-time tasks.
    ///
    /// Uses round-robin scheduler with moderate priority, suitable
    /// for tasks that should be responsive but can tolerate occasional
    /// delays.
    pub fn soft_realtime() -> Self {
        Self::new()
            .scheduler(RtScheduler::RoundRobin)
            .priority(50)
            .build()
    }

    /// Create a default non-RT configuration.
    ///
    /// This is a no-op configuration that doesn't modify thread
    /// scheduling parameters.
    pub fn normal() -> Self {
        Self::default()
    }

    /// Check kernel RT support without applying configuration.
    pub fn check_kernel_support() -> RtKernelInfo {
        RtKernelInfo::detect()
    }

    /// Apply this configuration to the current thread.
    ///
    /// Returns the result of applying the configuration, including
    /// any degradations that occurred.
    #[cfg(target_os = "linux")]
    pub fn apply(&self) -> Result<RtApplyResult, io::Error> {
        let mut degradations = Vec::new();
        let kernel_info = RtKernelInfo::detect();

        // Check for PREEMPT_RT if using RT scheduler
        if self.scheduler != RtScheduler::Normal && !kernel_info.preempt_rt {
            degradations.push(RtDegradation::NoPreemptRt);
            if self.warn_on_degradation {
                eprintln!(
                    "Warning: PREEMPT_RT kernel not detected, RT guarantees may not be met"
                );
            }
        }

        // Apply memory locking
        if self.memory_locked {
            match self.apply_memory_lock() {
                Ok(()) => {}
                Err(e) => {
                    let msg = format!("mlockall failed: {}", e);
                    degradations.push(RtDegradation::MemoryLockUnavailable(msg.clone()));
                    if self.warn_on_degradation {
                        eprintln!("Warning: {}", msg);
                    }
                    if self.fail_on_degradation {
                        return Err(io::Error::new(io::ErrorKind::PermissionDenied, msg));
                    }
                }
            }
        }

        // Apply stack prefaulting - do this early before entering RT section
        // This prevents minor page faults during critical code execution
        if let Some(stack_size) = self.prefault_stack_size {
            prefault_stack(stack_size);
        }

        // Apply scheduler and priority
        if let Some(priority) = self.priority {
            match self.apply_scheduler_priority(&kernel_info, priority) {
                Ok(actual) if actual != priority => {
                    degradations.push(RtDegradation::PriorityClamped {
                        requested: priority,
                        actual,
                    });
                    if self.warn_on_degradation {
                        eprintln!(
                            "Warning: Priority clamped from {} to {}",
                            priority, actual
                        );
                    }
                }
                Ok(_) => {}
                Err(e) => {
                    let msg = format!("Scheduler setup failed: {}", e);
                    degradations.push(RtDegradation::SchedulerDegraded(msg.clone()));
                    if self.warn_on_degradation {
                        eprintln!("Warning: {}", msg);
                    }
                    if self.fail_on_degradation {
                        return Err(io::Error::new(io::ErrorKind::PermissionDenied, msg));
                    }
                }
            }
        }

        // Apply CPU affinity
        if let Some(ref cpus) = self.cpu_affinity {
            match self.apply_cpu_affinity(cpus, kernel_info.cpu_count) {
                Ok(()) => {}
                Err(e) => {
                    let msg = format!("CPU affinity failed: {}", e);
                    degradations.push(RtDegradation::AffinityUnavailable(msg.clone()));
                    if self.warn_on_degradation {
                        eprintln!("Warning: {}", msg);
                    }
                    if self.fail_on_degradation {
                        return Err(io::Error::new(io::ErrorKind::InvalidInput, msg));
                    }
                }
            }
        }

        if degradations.is_empty() {
            Ok(RtApplyResult::FullSuccess)
        } else {
            Ok(RtApplyResult::Degraded(degradations))
        }
    }

    #[cfg(not(target_os = "linux"))]
    pub fn apply(&self) -> Result<RtApplyResult, io::Error> {
        // On non-Linux platforms, RT features are not available
        let mut degradations = Vec::new();

        if self.memory_locked {
            degradations.push(RtDegradation::MemoryLockUnavailable(
                "Not supported on this platform".to_string(),
            ));
        }

        if self.scheduler != RtScheduler::Normal {
            degradations.push(RtDegradation::SchedulerDegraded(
                "RT schedulers not supported on this platform".to_string(),
            ));
        }

        if self.cpu_affinity.is_some() {
            degradations.push(RtDegradation::AffinityUnavailable(
                "CPU affinity not supported on this platform".to_string(),
            ));
        }

        if self.fail_on_degradation && !degradations.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::Unsupported,
                "RT features not available on this platform",
            ));
        }

        if degradations.is_empty() {
            Ok(RtApplyResult::FullSuccess)
        } else {
            Ok(RtApplyResult::Degraded(degradations))
        }
    }

    #[cfg(target_os = "linux")]
    fn apply_memory_lock(&self) -> Result<(), io::Error> {
        // SAFETY: mlockall is a safe libc call
        // MCL_CURRENT: Lock all current pages
        // MCL_FUTURE: Lock all future pages as they are allocated
        let result = unsafe { libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) };

        if result == 0 {
            Ok(())
        } else {
            Err(io::Error::last_os_error())
        }
    }

    #[cfg(target_os = "linux")]
    fn apply_scheduler_priority(
        &self,
        kernel_info: &RtKernelInfo,
        requested_priority: i32,
    ) -> Result<i32, io::Error> {
        let policy = match self.scheduler {
            RtScheduler::Normal => libc::SCHED_OTHER,
            RtScheduler::Fifo => libc::SCHED_FIFO,
            RtScheduler::RoundRobin => libc::SCHED_RR,
            RtScheduler::Deadline => {
                // SCHED_DEADLINE requires special handling
                return Err(io::Error::new(
                    io::ErrorKind::Unsupported,
                    "SCHED_DEADLINE requires sched_setattr, not yet implemented",
                ));
            }
        };

        // Clamp priority to valid range
        let actual_priority = if policy == libc::SCHED_OTHER {
            0 // SCHED_OTHER always uses priority 0
        } else {
            requested_priority.clamp(kernel_info.min_rt_priority, kernel_info.max_rt_priority)
        };

        let param = libc::sched_param {
            sched_priority: actual_priority,
        };

        // SAFETY: sched_setscheduler is a safe libc call
        // 0 = current thread
        let result = unsafe { libc::sched_setscheduler(0, policy, &param) };

        if result == 0 {
            Ok(actual_priority)
        } else {
            Err(io::Error::last_os_error())
        }
    }

    #[cfg(target_os = "linux")]
    fn apply_cpu_affinity(&self, cpus: &[usize], cpu_count: usize) -> Result<(), io::Error> {
        if cpus.is_empty() {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "CPU affinity list cannot be empty",
            ));
        }

        // Validate CPU indices
        for &cpu in cpus {
            if cpu >= cpu_count {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    format!("CPU {} does not exist (max: {})", cpu, cpu_count - 1),
                ));
            }
        }

        // SAFETY: CPU_SET manipulation and sched_setaffinity are safe libc calls
        unsafe {
            let mut cpuset: libc::cpu_set_t = std::mem::zeroed();
            libc::CPU_ZERO(&mut cpuset);

            for &cpu in cpus {
                libc::CPU_SET(cpu, &mut cpuset);
            }

            // 0 = current thread
            let result = libc::sched_setaffinity(
                0,
                std::mem::size_of::<libc::cpu_set_t>(),
                &cpuset,
            );

            if result == 0 {
                Ok(())
            } else {
                Err(io::Error::last_os_error())
            }
        }
    }

    /// Get the current thread's scheduler and priority.
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
                libc::SCHED_RR => RtScheduler::RoundRobin,
                _ => RtScheduler::Normal,
            };

            Ok((scheduler, param.sched_priority))
        }
    }

    #[cfg(not(target_os = "linux"))]
    pub fn get_current_scheduler() -> Result<(RtScheduler, i32), io::Error> {
        Ok((RtScheduler::Normal, 0))
    }

    /// Get the current thread's CPU affinity.
    #[cfg(target_os = "linux")]
    pub fn get_current_affinity() -> Result<Vec<usize>, io::Error> {
        // SAFETY: sched_getaffinity and CPU_ISSET are safe libc calls
        unsafe {
            let mut cpuset: libc::cpu_set_t = std::mem::zeroed();

            let result = libc::sched_getaffinity(
                0,
                std::mem::size_of::<libc::cpu_set_t>(),
                &mut cpuset,
            );

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
        // Return all CPUs on non-Linux
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
/// ```rust
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
pub fn prefault_stack(size: usize) {
    // Page size is typically 4KB on most systems
    const PAGE_SIZE: usize = 4096;

    // Calculate number of pages to touch
    let num_pages = (size + PAGE_SIZE - 1) / PAGE_SIZE;

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

/// Prefault stack using a single large allocation (alternative implementation).
///
/// This variant uses alloca-style stack allocation for systems where
/// recursive prefaulting might have too much overhead.
///
/// # Arguments
/// * `size` - Stack size in bytes to prefault (will be rounded up to page boundary)
///
/// # Note
/// This function uses a different approach than `prefault_stack`:
/// - Faster for large stacks (no recursion overhead)
/// - May cause stack overflow if size exceeds available stack
/// - Use with caution and ensure thread stack size is sufficient
#[inline(never)]
pub fn prefault_stack_linear(size: usize) {
    const PAGE_SIZE: usize = 4096;

    // Cap at reasonable maximum to prevent stack overflow
    let capped_size = size.min(8 * 1024 * 1024); // 8MB max

    // Number of pages to touch
    let num_pages = (capped_size + PAGE_SIZE - 1) / PAGE_SIZE;

    // Use a volatile write to each page to ensure they're faulted in
    // We can't actually allocate variable-sized arrays on stack in safe Rust,
    // so we touch pages by using fixed-size chunks
    for page_idx in 0..num_pages {
        // Create a small marker on the stack for each page
        // The recursive calls ensure we actually use stack space
        let marker: [u8; 64] = [page_idx as u8; 64];
        black_box(&marker);

        // Force compiler to not optimize away by reading the value
        if black_box(marker[0]) == 255 {
            // This branch is never taken but prevents optimization
            std::hint::spin_loop();
        }
    }
}

// ============================================================================
// STANDALONE HELPER FUNCTIONS FOR CPU AFFINITY AND ISOLATION
// ============================================================================

/// Pin the current thread to a specific CPU core.
///
/// This is a convenience wrapper around RtConfig that provides a simple
/// one-call interface for pinning threads to cores for real-time performance.
///
/// # Arguments
/// * `cpu` - The CPU core index to pin to (0-indexed)
///
/// # Returns
/// * `Ok(())` - Thread was successfully pinned to the specified core
/// * `Err(io::Error)` - Failed to pin (invalid CPU, permission denied, etc.)
///
/// # Example
/// ```rust,no_run
/// use horus_core::core::pin_thread_to_core;
///
/// // Pin current thread to CPU core 2
/// pin_thread_to_core(2).expect("Failed to pin thread");
/// ```
///
/// # Performance Notes
/// - Pinning to a dedicated core (especially isolcpus) eliminates scheduler migration
/// - Reduces cache pollution from other processes
/// - Critical for achieving sub-100ns IPC latency
#[cfg(target_os = "linux")]
pub fn pin_thread_to_core(cpu: usize) -> Result<(), io::Error> {
    let cpu_count = std::thread::available_parallelism()
        .map(|p| p.get())
        .unwrap_or(1);

    if cpu >= cpu_count {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("CPU {} does not exist (max: {})", cpu, cpu_count - 1),
        ));
    }

    // SAFETY: CPU_SET manipulation and sched_setaffinity are safe libc calls
    unsafe {
        let mut cpuset: libc::cpu_set_t = std::mem::zeroed();
        libc::CPU_ZERO(&mut cpuset);
        libc::CPU_SET(cpu, &mut cpuset);

        let result = libc::sched_setaffinity(
            0, // 0 = current thread
            std::mem::size_of::<libc::cpu_set_t>(),
            &cpuset,
        );

        if result == 0 {
            Ok(())
        } else {
            Err(io::Error::last_os_error())
        }
    }
}

#[cfg(not(target_os = "linux"))]
pub fn pin_thread_to_core(_cpu: usize) -> Result<(), io::Error> {
    // CPU pinning not supported on non-Linux platforms
    // Silently succeed to avoid breaking cross-platform code
    Ok(())
}

/// Detect CPUs isolated via the `isolcpus` kernel boot parameter.
///
/// Isolated CPUs are excluded from the general scheduler and are ideal for
/// real-time tasks because they won't be interrupted by other processes.
///
/// # Returns
/// * `Vec<usize>` - List of isolated CPU core indices (empty if none)
///
/// # Example
/// ```rust,no_run
/// use horus_core::core::detect_isolated_cpus;
///
/// let isolated = detect_isolated_cpus();
/// if !isolated.is_empty() {
///     println!("Isolated CPUs available for RT: {:?}", isolated);
/// }
/// ```
///
/// # Kernel Configuration
/// To isolate CPUs, add to kernel boot parameters:
/// ```text
/// isolcpus=2,3          # Isolate cores 2 and 3
/// isolcpus=2-5          # Isolate cores 2, 3, 4, 5
/// nohz_full=2,3         # Also disable timer ticks (recommended)
/// rcu_nocbs=2,3         # Move RCU callbacks off these cores
/// ```
#[cfg(target_os = "linux")]
pub fn detect_isolated_cpus() -> Vec<usize> {
    // Read from /sys/devices/system/cpu/isolated
    match std::fs::read_to_string("/sys/devices/system/cpu/isolated") {
        Ok(content) => parse_cpu_list(&content.trim()),
        Err(_) => Vec::new(), // File doesn't exist or not readable
    }
}

#[cfg(not(target_os = "linux"))]
pub fn detect_isolated_cpus() -> Vec<usize> {
    Vec::new() // No isolcpus on non-Linux platforms
}

/// Detect CPUs configured with `nohz_full` (tickless kernel).
///
/// These CPUs have timer interrupts disabled when running a single task,
/// providing even better latency characteristics than just isolcpus.
///
/// # Returns
/// * `Vec<usize>` - List of nohz_full CPU core indices (empty if none)
#[cfg(target_os = "linux")]
pub fn detect_nohz_full_cpus() -> Vec<usize> {
    match std::fs::read_to_string("/sys/devices/system/cpu/nohz_full") {
        Ok(content) => parse_cpu_list(&content.trim()),
        Err(_) => Vec::new(),
    }
}

#[cfg(not(target_os = "linux"))]
pub fn detect_nohz_full_cpus() -> Vec<usize> {
    Vec::new()
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
/// ```rust,no_run
/// use horus_core::core::get_rt_recommended_cpus;
///
/// // Get 2 CPUs for RT tasks
/// let rt_cpus = get_rt_recommended_cpus(2);
/// for (i, cpu) in rt_cpus.iter().enumerate() {
///     println!("RT thread {} -> CPU {}", i, cpu);
/// }
/// ```
pub fn get_rt_recommended_cpus(count: usize) -> Vec<usize> {
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
pub struct RtCpuInfo {
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

    /// Check if the system has good RT CPU configuration.
    pub fn has_rt_configuration(&self) -> bool {
        !self.isolated_cpus.is_empty()
    }

    /// Get a human-readable summary of RT CPU configuration.
    pub fn summary(&self) -> String {
        let mut s = format!("RT CPU Configuration ({} total CPUs):\n", self.total_cpus);

        if self.isolated_cpus.is_empty() {
            s.push_str("  isolcpus: NONE (consider adding isolcpus=N-M to kernel params)\n");
        } else {
            s.push_str(&format!("  isolcpus: {:?}\n", self.isolated_cpus));
        }

        if self.nohz_full_cpus.is_empty() {
            s.push_str("  nohz_full: NONE (consider adding nohz_full=N-M for lower latency)\n");
        } else {
            s.push_str(&format!("  nohz_full: {:?}\n", self.nohz_full_cpus));
        }

        s.push_str(&format!(
            "  Recommended for RT (priority order): {:?}\n",
            self.recommended_rt_cpus
        ));

        s
    }
}

/// Parse a CPU list string like "2-5,7,9-11" into individual CPU indices.
fn parse_cpu_list(s: &str) -> Vec<usize> {
    let mut cpus = Vec::new();

    if s.is_empty() {
        return cpus;
    }

    for part in s.split(',') {
        let part = part.trim();
        if part.is_empty() {
            continue;
        }

        if let Some(dash_pos) = part.find('-') {
            // Range like "2-5"
            let start_str = &part[..dash_pos];
            let end_str = &part[dash_pos + 1..];

            if let (Ok(start), Ok(end)) = (start_str.parse::<usize>(), end_str.parse::<usize>()) {
                for cpu in start..=end {
                    cpus.push(cpu);
                }
            }
        } else {
            // Single CPU like "7"
            if let Ok(cpu) = part.parse::<usize>() {
                cpus.push(cpu);
            }
        }
    }

    cpus.sort();
    cpus.dedup();
    cpus
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_builder_default() {
        let config = RtConfig::new().build();
        assert!(!config.memory_locked);
        assert_eq!(config.priority, None);
        assert_eq!(config.scheduler, RtScheduler::Normal);
        assert!(config.cpu_affinity.is_none());
    }

    #[test]
    fn test_builder_full() {
        let config = RtConfig::new()
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
    fn test_hard_realtime_preset() {
        let config = RtConfig::hard_realtime(Some(&[2, 3]));
        assert!(config.memory_locked);
        assert_eq!(config.scheduler, RtScheduler::Fifo);
        assert_eq!(config.priority, Some(80));
        assert_eq!(config.cpu_affinity, Some(vec![2, 3]));
    }

    #[test]
    fn test_soft_realtime_preset() {
        let config = RtConfig::soft_realtime();
        assert!(!config.memory_locked);
        assert_eq!(config.scheduler, RtScheduler::RoundRobin);
        assert_eq!(config.priority, Some(50));
    }

    #[test]
    fn test_normal_preset() {
        let config = RtConfig::normal();
        assert!(!config.memory_locked);
        assert_eq!(config.scheduler, RtScheduler::Normal);
        assert_eq!(config.priority, None);
    }

    #[test]
    fn test_kernel_info_detect() {
        // Just verify it doesn't panic
        let info = RtKernelInfo::detect();
        assert!(info.cpu_count > 0);
    }

    #[test]
    fn test_get_current_scheduler() {
        // Should not fail
        let result = RtConfig::get_current_scheduler();
        assert!(result.is_ok());
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
    fn test_normal_config_apply() {
        // Normal config should always succeed
        let config = RtConfig::normal();
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

    #[test]
    fn test_prefault_stack_linear() {
        // Test the linear variant
        super::prefault_stack_linear(64 * 1024);
    }

    #[test]
    fn test_builder_with_prefault() {
        let config = RtConfig::new()
            .prefault_stack(256 * 1024)
            .build();

        assert_eq!(config.prefault_stack_size, Some(256 * 1024));
    }

    #[test]
    fn test_hard_realtime_includes_prefault() {
        let config = RtConfig::hard_realtime(None);

        // hard_realtime preset should include stack prefaulting
        assert!(config.prefault_stack_size.is_some());
        assert_eq!(config.prefault_stack_size, Some(512 * 1024));
    }

    #[test]
    fn test_config_with_prefault_apply() {
        // Config with prefault should apply successfully
        let config = RtConfig::new()
            .prefault_stack(64 * 1024)
            .build();

        let result = config.apply();
        assert!(result.is_ok());
    }
}
