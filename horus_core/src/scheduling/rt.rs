//! Runtime features and capability detection for the HORUS scheduler.
//!
//! This module combines:
//! - **OS-level RT primitives**: CPU affinity, memory locking, RT scheduling, NUMA
//! - **Capability detection**: Auto-detect what the system supports at startup
//!
//! # Usage
//!
//! ```rust,ignore
//! use horus_core::scheduling::RuntimeCapabilities;
//!
//! // Detect all capabilities (one-time ~30-100μs startup cost)
//! let caps = RuntimeCapabilities::detect();
//!
//! if caps.has_rt_support() {
//!     println!("RT scheduling available with priority up to {}", caps.max_rt_priority);
//! }
//! ```

use crate::core::rt_config::{RtCpuInfo, RtKernelInfo};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// ============================================================================
// Runtime Error Types
// ============================================================================

/// Result type for runtime operations
#[doc(hidden)]
pub type RuntimeResult<T> = Result<T, RuntimeError>;

/// Errors from runtime operations
#[doc(hidden)]
#[derive(Debug, thiserror::Error)]
pub enum RuntimeError {
    /// Failed to set CPU affinity
    #[error("CPU affinity error: {0}")]
    AffinityError(String),
    /// Failed to lock memory
    #[error("Memory lock error: {0}")]
    MemoryLockError(String),
    /// Failed to set RT scheduling
    #[error("Scheduling error: {0}")]
    SchedulingError(String),
    /// Feature not supported on this platform
    #[error("Not supported: {0}")]
    NotSupported(String),
    /// Permission denied
    #[error("Permission denied: {0}")]
    PermissionDenied(String),
}

impl From<RuntimeError> for crate::error::HorusError {
    fn from(err: RuntimeError) -> Self {
        match err {
            RuntimeError::PermissionDenied(msg) => {
                crate::error::HorusError::Resource(crate::error::ResourceError::PermissionDenied {
                    resource: "RT scheduling".to_string(),
                    required_permission: msg,
                })
            }
            RuntimeError::NotSupported(msg) => {
                crate::error::HorusError::Resource(crate::error::ResourceError::Unsupported {
                    feature: "RT scheduling".to_string(),
                    reason: msg,
                })
            }
            other => crate::error::HorusError::Config(crate::error::ConfigError::Other(
                other.to_string(),
            )),
        }
    }
}

// ============================================================================
// CPU Core Affinity
// ============================================================================

/// Pin the current thread to specific CPU cores
#[doc(hidden)]
pub fn set_thread_affinity(cores: &[usize]) -> RuntimeResult<()> {
    if cores.is_empty() {
        return Ok(());
    }

    let core_ids = core_affinity::get_core_ids()
        .ok_or_else(|| RuntimeError::AffinityError("Failed to get core IDs".to_string()))?;

    // Find the requested cores
    for &core_idx in cores {
        if core_idx < core_ids.len() && core_affinity::set_for_current(core_ids[core_idx]) {
            println!("[RT] Pinned thread to core {}", core_idx);
            return Ok(());
        }
    }

    Err(RuntimeError::AffinityError(format!(
        "Failed to pin to cores {:?}",
        cores
    )))
}

// ============================================================================
// Memory Locking
// ============================================================================

/// Lock all current and future memory pages (prevents swapping).
///
/// Delegates to [`horus_sys::rt::lock_memory()`] which handles per-platform implementation.
#[doc(hidden)]
pub fn lock_all_memory() -> RuntimeResult<()> {
    horus_sys::rt::lock_memory().map_err(|e| {
        let msg = e.to_string();
        if msg.contains("CAP_IPC_LOCK") || msg.contains("root") || msg.contains("EPERM") {
            RuntimeError::PermissionDenied(msg)
        } else if msg.contains("not supported") {
            RuntimeError::NotSupported(msg)
        } else {
            RuntimeError::MemoryLockError(msg)
        }
    })?;
    println!("[RT] All memory locked");
    Ok(())
}

/// Pre-fault stack memory to avoid page faults during execution.
///
/// Delegates to [`crate::core::rt_config::prefault_stack`] which uses
/// recursive stack touching with `black_box` for reliable prefaulting.
#[doc(hidden)]
pub fn prefault_stack(stack_size: usize) -> RuntimeResult<()> {
    crate::core::rt_config::prefault_stack(stack_size);
    println!("[RT] Pre-faulted {}KB of stack", stack_size / 1024);
    Ok(())
}

// ============================================================================
// Real-Time Scheduling
// ============================================================================

/// Set real-time scheduling policy for current thread.
///
/// Delegates to [`horus_sys::rt::set_realtime_priority()`] which handles per-platform implementation.
#[doc(hidden)]
pub fn set_realtime_priority(priority: i32) -> RuntimeResult<()> {
    horus_sys::rt::set_realtime_priority(priority).map_err(|e| {
        let msg = e.to_string();
        if msg.contains("CAP_SYS_NICE") || msg.contains("root") || msg.contains("EPERM") {
            RuntimeError::PermissionDenied(msg)
        } else if msg.contains("not supported") {
            RuntimeError::NotSupported(msg)
        } else {
            RuntimeError::SchedulingError(msg)
        }
    })?;
    println!("[RT] Set RT priority {}", priority);
    Ok(())
}

// ============================================================================
// NUMA Awareness
// ============================================================================

/// Get NUMA node count (returns 1 if NUMA not available).
///
/// Reserved for future NUMA-aware thread pinning in the scheduler.
#[cfg(test)]
#[cfg(target_os = "linux")]
pub(crate) fn get_numa_node_count() -> usize {
    if let Ok(entries) = std::fs::read_dir("/sys/devices/system/node/") {
        entries
            .filter_map(|e| e.ok())
            .filter(|e| e.file_name().to_string_lossy().starts_with("node"))
            .count()
            .max(1)
    } else {
        1
    }
}

#[cfg(test)]
#[cfg(not(target_os = "linux"))]
pub(crate) fn get_numa_node_count() -> usize {
    1
}

// ============================================================================
// Runtime Capabilities Detection
// ============================================================================

/// Comprehensive runtime capabilities for automatic Scheduler optimization.
///
/// This struct aggregates all detectable system capabilities that affect
/// scheduling performance. The Scheduler uses this to auto-configure
/// RT priority, memory locking, CPU affinity, and other optimizations.
///
/// # Fields
///
/// All fields are public for inspection, but prefer using the helper methods
/// (e.g., `has_rt_support()`, `can_lock_memory()`) for cleaner code.
#[doc(hidden)]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RuntimeCapabilities {
    // =========================================================================
    // RT Scheduling Capabilities
    // =========================================================================
    /// Whether PREEMPT_RT kernel patch is detected
    pub preempt_rt: bool,

    /// Whether RT scheduling (SCHED_FIFO/SCHED_RR) is available AND permitted
    /// This is true if max_rt_priority > 0 and the process can set RT priority
    pub rt_priority_available: bool,

    /// Maximum RT priority this process can use (typically 1-99, 0 if unavailable)
    pub max_rt_priority: i32,

    /// Minimum RT priority (typically 1)
    pub min_rt_priority: i32,

    // =========================================================================
    // Memory Locking Capabilities
    // =========================================================================
    /// Whether mlockall() is permitted (via RLIMIT_MEMLOCK)
    pub mlockall_permitted: bool,

    /// Current RLIMIT_MEMLOCK soft limit in bytes (0 if unavailable)
    pub memlock_limit_bytes: u64,

    // =========================================================================
    // CPU Topology
    // =========================================================================
    /// Total number of logical CPUs
    pub cpu_count: usize,

    /// CPUs isolated via isolcpus kernel parameter (excluded from general scheduling)
    pub isolated_cpus: Vec<usize>,

    /// CPUs with tickless operation (nohz_full) - minimal timer interrupts
    pub nohz_full_cpus: Vec<usize>,

    /// Recommended CPUs for RT tasks, in preference order:
    /// 1. Isolated + nohz_full (best)
    /// 2. Isolated only
    /// 3. nohz_full only
    /// 4. Higher-numbered CPUs (least system interference)
    pub recommended_rt_cpus: Vec<usize>,

    // =========================================================================
    // NUMA Topology
    // =========================================================================
    /// Number of NUMA nodes (1 for UMA systems)
    pub numa_node_count: usize,

    /// CPUs belonging to each NUMA node (node_id -> cpu_ids)
    pub numa_topology: HashMap<usize, Vec<usize>>,

    // =========================================================================
    // Kernel Information
    // =========================================================================
    /// Full kernel version string
    pub kernel_version: String,

    /// Whether this is a Linux system
    pub is_linux: bool,

    // =========================================================================
    // Detection Metadata
    // =========================================================================
    /// Time taken to detect capabilities (microseconds)
    pub detection_time_us: u64,
}

impl Default for RuntimeCapabilities {
    fn default() -> Self {
        Self {
            preempt_rt: false,
            rt_priority_available: false,
            max_rt_priority: 0,
            min_rt_priority: 0,
            mlockall_permitted: false,
            memlock_limit_bytes: 0,
            cpu_count: 1,
            isolated_cpus: Vec::new(),
            nohz_full_cpus: Vec::new(),
            recommended_rt_cpus: Vec::new(),
            numa_node_count: 1,
            numa_topology: HashMap::new(),
            kernel_version: String::new(),
            is_linux: cfg!(target_os = "linux"),
            detection_time_us: 0,
        }
    }
}

impl RuntimeCapabilities {
    /// Detect all runtime capabilities.
    ///
    /// This performs one-time detection of all system capabilities:
    /// - RT scheduling availability and permissions
    /// - Memory locking permissions
    /// - CPU topology (isolated, nohz_full, NUMA)
    ///
    /// # Performance
    ///
    /// Detection takes approximately 30-100μs depending on system.
    /// Call once at Scheduler construction, not per-tick.
    pub fn detect() -> Self {
        let start = std::time::Instant::now();

        // Detect RT kernel capabilities
        let kernel_info = RtKernelInfo::detect();

        // Detect CPU topology
        let cpu_info = RtCpuInfo::detect();

        // Detect NUMA topology
        let (numa_node_count, numa_topology) = Self::detect_numa_topology();

        // Get memlock limit
        let memlock_limit_bytes = Self::get_memlock_limit();

        // Determine if RT priority is actually available
        // (need both kernel support AND permission)
        let rt_priority_available =
            kernel_info.max_rt_priority > 0 && Self::can_set_rt_priority_impl();

        let detection_time_us = start.elapsed().as_micros() as u64;

        Self {
            preempt_rt: kernel_info.preempt_rt,
            rt_priority_available,
            max_rt_priority: kernel_info.max_rt_priority,
            min_rt_priority: kernel_info.min_rt_priority,
            mlockall_permitted: kernel_info.mlockall_permitted,
            memlock_limit_bytes,
            cpu_count: cpu_info.total_cpus,
            isolated_cpus: cpu_info.isolated_cpus,
            nohz_full_cpus: cpu_info.nohz_full_cpus,
            recommended_rt_cpus: cpu_info.recommended_rt_cpus,
            numa_node_count,
            numa_topology,
            kernel_version: kernel_info.kernel_version,
            is_linux: cfg!(target_os = "linux"),
            detection_time_us,
        }
    }

    // =========================================================================
    // Convenience Query Methods
    // =========================================================================

    /// Check if RT scheduling is available and permitted.
    #[inline]
    pub fn has_rt_support(&self) -> bool {
        self.rt_priority_available
    }

    /// Check if full hard-realtime is available.
    ///
    /// Returns true if PREEMPT_RT kernel is present, RT priority is available,
    /// and memory locking is permitted.
    #[inline]
    pub fn has_hard_rt_support(&self) -> bool {
        self.preempt_rt && self.rt_priority_available && self.mlockall_permitted
    }

    /// Check if memory locking (mlockall) is permitted.
    #[inline]
    pub fn can_lock_memory(&self) -> bool {
        self.mlockall_permitted
    }

    /// Check if isolated CPUs are available for RT pinning.
    #[inline]
    pub fn has_isolated_cpus(&self) -> bool {
        !self.isolated_cpus.is_empty()
    }

    /// Check if this is a multi-NUMA system.
    #[inline]
    pub fn is_numa_system(&self) -> bool {
        self.numa_node_count > 1
    }

    /// Get the best CPUs for RT tasks (up to `count` CPUs).
    pub fn get_rt_cpus(&self, count: usize) -> Vec<usize> {
        self.recommended_rt_cpus
            .iter()
            .take(count)
            .copied()
            .collect()
    }

    /// Get the recommended RT priority for production use.
    ///
    /// Returns 80 if RT is available, 0 otherwise.
    pub fn recommended_rt_priority(&self) -> i32 {
        if self.rt_priority_available {
            80.min(self.max_rt_priority)
        } else {
            0
        }
    }

    /// Get CPUs on a specific NUMA node.
    pub fn cpus_on_numa_node(&self, node: usize) -> Vec<usize> {
        self.numa_topology.get(&node).cloned().unwrap_or_default()
    }

    /// Get a human-readable summary of capabilities.
    pub fn summary(&self) -> String {
        let mut s = String::new();

        s.push_str(&format!(
            "RuntimeCapabilities (detected in {}μs):\n",
            self.detection_time_us
        ));
        s.push_str(&format!(
            "  Kernel: {} {}\n",
            if self.preempt_rt {
                "PREEMPT_RT"
            } else {
                "standard"
            },
            &self.kernel_version[..self.kernel_version.len().min(50)]
        ));
        s.push_str(&format!(
            "  RT Priority: {} (max: {})\n",
            if self.rt_priority_available {
                "available"
            } else {
                "UNAVAILABLE"
            },
            self.max_rt_priority
        ));
        s.push_str(&format!(
            "  Memory Lock: {}\n",
            if self.mlockall_permitted {
                "permitted"
            } else {
                "DENIED"
            }
        ));
        s.push_str(&format!("  CPUs: {} total", self.cpu_count));

        if !self.isolated_cpus.is_empty() {
            s.push_str(&format!(", isolated: {:?}", self.isolated_cpus));
        }
        if !self.nohz_full_cpus.is_empty() {
            s.push_str(&format!(", nohz_full: {:?}", self.nohz_full_cpus));
        }
        s.push('\n');

        if self.numa_node_count > 1 {
            s.push_str(&format!("  NUMA: {} nodes\n", self.numa_node_count));
        }

        if !self.recommended_rt_cpus.is_empty() {
            s.push_str(&format!(
                "  Recommended RT CPUs: {:?}\n",
                &self.recommended_rt_cpus[..self.recommended_rt_cpus.len().min(8)]
            ));
        }

        s
    }

    // =========================================================================
    // Private Detection Helpers
    // =========================================================================

    /// Detect NUMA topology (node count and CPU mapping).
    /// On Linux, reads from /sys/devices/system/node/. Non-Linux: single node.
    fn detect_numa_topology() -> (usize, HashMap<usize, Vec<usize>>) {
        // NUMA detection: read /sys/devices/system/node/ (Linux).
        // Falls through to single-node on non-Linux or when /sys is unavailable.
        let mut topology = HashMap::new();
        let mut max_node = 0;
        let node_base = std::path::Path::new("/sys/devices/system/node");
        if node_base.exists() {
            if let Ok(entries) = std::fs::read_dir(node_base) {
                for entry in entries.flatten() {
                    let name = entry.file_name();
                    let name = name.to_string_lossy();
                    if let Some(node_str) = name.strip_prefix("node") {
                        if let Ok(node_id) = node_str.parse::<usize>() {
                            max_node = max_node.max(node_id + 1);
                            let cpulist_path = entry.path().join("cpulist");
                            if let Ok(cpulist) = std::fs::read_to_string(&cpulist_path) {
                                let cpus = horus_sys::rt::parse_cpu_list(cpulist.trim());
                                if !cpus.is_empty() {
                                    topology.insert(node_id, cpus);
                                }
                            }
                        }
                    }
                }
            }
        }
        if max_node > 0 {
            return (max_node, topology);
        }
        // Fallback: single NUMA node with all CPUs
        let cpu_count = std::thread::available_parallelism().map(|p| p.get()).unwrap_or(1);
        topology.insert(0, (0..cpu_count).collect());
        (1, topology)
    }

    /// Parse a CPU list string like "0-3,7,9-11" into individual CPU indices.
    #[allow(dead_code)]
    fn parse_cpu_list(s: &str) -> Vec<usize> {
        horus_sys::rt::parse_cpu_list(s)
    }

    /// Get the current RLIMIT_MEMLOCK soft limit (delegates to horus_sys::rt).
    fn get_memlock_limit() -> u64 {
        horus_sys::rt::memlock_limit_bytes()
    }

    /// Check if we can actually set RT priority (delegates to horus_sys::rt).
    fn can_set_rt_priority_impl() -> bool {
        horus_sys::rt::can_set_rt_priority()
    }
}

#[cfg(test)]
mod runtime_tests {
    use super::*;

    #[test]
    fn test_numa_node_count() {
        let count = get_numa_node_count();
        assert!(count >= 1);
        // Cross-check against capabilities detection
        let caps = RuntimeCapabilities::detect();
        assert_eq!(
            count, caps.numa_node_count,
            "get_numa_node_count() should match RuntimeCapabilities::detect().numa_node_count"
        );
    }

    #[test]
    fn test_prefault_stack() {
        let result = prefault_stack(64 * 1024); // 64KB
        result.unwrap();
    }
}

#[cfg(test)]
mod capabilities_tests {
    use super::*;

    #[test]
    fn test_detect_capabilities() {
        let caps = RuntimeCapabilities::detect();
        assert!(caps.cpu_count > 0);
        assert!(!caps.kernel_version.is_empty(), "kernel_version should be detected");
        assert!(caps.numa_node_count >= 1, "At least 1 NUMA node expected");
        assert!(caps.detection_time_us > 0, "Detection should take measurable time");
        // recommended_rt_cpus should be a subset of available CPUs
        for &cpu in &caps.recommended_rt_cpus {
            assert!(cpu < caps.cpu_count, "Recommended RT CPU {} exceeds cpu_count {}", cpu, caps.cpu_count);
        }
    }

    #[test]
    fn test_default() {
        let caps = RuntimeCapabilities::default();
        assert_eq!(caps.cpu_count, 1);
        assert!(!caps.has_rt_support());
        assert!(caps.recommended_rt_cpus.is_empty());
    }

    #[test]
    fn test_numa_detection() {
        let caps = RuntimeCapabilities::detect();
        assert!(caps.numa_node_count >= 1);
        let node0_cpus = caps.cpus_on_numa_node(0);
        assert!(!node0_cpus.is_empty());
    }

    #[test]
    fn test_get_rt_cpus() {
        let caps = RuntimeCapabilities::detect();
        let cpus = caps.get_rt_cpus(2);
        assert!(cpus.len() <= 2);
        let cpus = caps.get_rt_cpus(0);
        assert!(cpus.is_empty());
    }

    #[test]
    fn test_parse_cpu_list() {
        assert_eq!(RuntimeCapabilities::parse_cpu_list("0-3"), vec![0, 1, 2, 3]);
        assert_eq!(RuntimeCapabilities::parse_cpu_list("0,2,4"), vec![0, 2, 4]);
        assert_eq!(
            RuntimeCapabilities::parse_cpu_list("0-2,5,7-8"),
            vec![0, 1, 2, 5, 7, 8]
        );
        assert_eq!(RuntimeCapabilities::parse_cpu_list(""), Vec::<usize>::new());
    }

    #[test]
    fn test_recommended_priority() {
        let caps = RuntimeCapabilities::detect();
        let prio = caps.recommended_rt_priority();
        if caps.has_rt_support() {
            assert!(prio > 0);
            assert!(prio <= caps.max_rt_priority);
        } else {
            assert_eq!(prio, 0);
        }
    }

    #[test]
    fn test_helper_methods() {
        let caps = RuntimeCapabilities::detect();
        // Verify logical consistency between capability flags
        if caps.has_hard_rt_support() {
            assert!(caps.has_rt_support(), "hard RT implies basic RT support");
        }
        // NUMA system iff multiple nodes
        assert_eq!(
            caps.is_numa_system(),
            caps.numa_node_count > 1,
            "is_numa_system() should match numa_node_count > 1"
        );
        // isolated_cpus implies they exist in the CPU range
        if caps.has_isolated_cpus() {
            assert!(
                !caps.recommended_rt_cpus.is_empty(),
                "Isolated CPUs should produce RT CPU recommendations"
            );
        }
    }

    #[test]
    fn test_summary_output() {
        let caps = RuntimeCapabilities::detect();
        let summary = caps.summary();
        assert!(summary.contains("RT Priority:"));
        assert!(summary.contains("Memory Lock:"));
        assert!(summary.contains("CPUs:"));
    }

    #[test]
    fn test_detection_time() {
        let caps = RuntimeCapabilities::detect();
        assert!(caps.detection_time_us < 100_000);
    }
}
