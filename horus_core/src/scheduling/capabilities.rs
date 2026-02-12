//! Runtime capabilities detection for automatic Scheduler optimization.
//!
//! This module provides comprehensive detection of runtime capabilities that the
//! Scheduler uses to auto-optimize for maximum performance without manual configuration.
//!
//! # Key Capabilities Detected
//!
//! - **RT Priority**: SCHED_FIFO/SCHED_RR availability and permission
//! - **Memory Locking**: mlockall() permission via RLIMIT_MEMLOCK
//! - **CPU Topology**: Isolated CPUs, nohz_full, NUMA nodes
//! - **Kernel Features**: PREEMPT_RT patch, kernel version
//! - **Platform**: Hardware platform (Jetson, RPi, etc.) and capabilities
//!
//! # Usage
//!
//! ```rust,ignore
//! use horus_core::scheduling::RuntimeCapabilities;
//!
//! // Detect all capabilities (one-time ~30-100μs startup cost)
//! let caps = RuntimeCapabilities::detect();
//!
//! // Check what's available
//! if caps.has_rt_support() {
//!     println!("RT scheduling available with priority up to {}", caps.max_rt_priority);
//! }
//!
//! // Get recommended CPUs for RT tasks
//! if !caps.recommended_rt_cpus.is_empty() {
//!     println!("Pin to CPUs: {:?}", caps.recommended_rt_cpus);
//! }
//! ```
//!
//! # Zero Runtime Overhead
//!
//! Detection runs once at Scheduler construction. After that, all capability
//! checks are simple field reads with zero overhead.

use crate::core::rt_config::{parse_cpu_list, RtCpuInfo, RtKernelInfo};
use crate::hardware::{Platform, PlatformCapabilities, PlatformDetector};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

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
    // Platform Information
    // =========================================================================
    /// Detected hardware platform (Jetson, RPi, x86, etc.)
    pub platform: Platform,

    /// Platform-specific capabilities (GPIO, CUDA, NPU, etc.)
    /// Note: Skipped in serialization as PlatformCapabilities doesn't implement Serde
    #[serde(skip)]
    pub platform_capabilities: PlatformCapabilities,

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
            platform: Platform::Unknown,
            platform_capabilities: PlatformCapabilities::default(),
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
    /// - Hardware platform detection
    ///
    /// # Performance
    ///
    /// Detection takes approximately 30-100μs depending on system.
    /// Call once at Scheduler construction, not per-tick.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let caps = RuntimeCapabilities::detect();
    /// println!("RT available: {}", caps.has_rt_support());
    /// println!("Recommended CPUs: {:?}", caps.recommended_rt_cpus);
    /// ```
    pub fn detect() -> Self {
        let start = std::time::Instant::now();

        // Detect RT kernel capabilities
        let kernel_info = RtKernelInfo::detect();

        // Detect CPU topology
        let cpu_info = RtCpuInfo::detect();

        // Detect NUMA topology
        let (numa_node_count, numa_topology) = Self::detect_numa_topology();

        // Detect platform
        let platform = PlatformDetector::detect();
        let platform_capabilities = PlatformDetector::capabilities(&platform);

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
            platform,
            platform_capabilities,
            detection_time_us,
        }
    }

    // =========================================================================
    // Convenience Query Methods
    // =========================================================================

    /// Check if RT scheduling is available and permitted.
    ///
    /// Returns true if the system supports RT priorities (SCHED_FIFO/SCHED_RR)
    /// AND the current process has permission to use them.
    #[inline]
    pub fn has_rt_support(&self) -> bool {
        self.rt_priority_available
    }

    /// Check if full hard-realtime is available.
    ///
    /// Returns true if:
    /// - PREEMPT_RT kernel is present
    /// - RT priority is available
    /// - Memory locking is permitted
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

    /// Check if CUDA is available (Jetson platforms).
    #[inline]
    pub fn has_cuda(&self) -> bool {
        self.platform_capabilities.cuda_available
    }

    /// Check if NPU/AI accelerator is available.
    #[inline]
    pub fn has_npu(&self) -> bool {
        self.platform_capabilities.npu_available
    }

    /// Get the best CPUs for RT tasks (up to `count` CPUs).
    ///
    /// Returns CPUs in order of preference for RT workloads.
    /// Empty if no suitable CPUs are available.
    pub fn get_rt_cpus(&self, count: usize) -> Vec<usize> {
        self.recommended_rt_cpus
            .iter()
            .take(count)
            .copied()
            .collect()
    }

    /// Get the recommended RT priority for production use.
    ///
    /// Returns a priority that's high but leaves room for system tasks:
    /// - 80 if full RT is available
    /// - 0 if RT is not available
    pub fn recommended_rt_priority(&self) -> i32 {
        if self.rt_priority_available {
            // Use 80 out of 99 - high but not maximum
            80.min(self.max_rt_priority)
        } else {
            0
        }
    }

    /// Get CPUs on a specific NUMA node.
    ///
    /// Returns empty vec if node doesn't exist or on non-NUMA systems.
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
            "  Platform: {:?} ({})\n",
            self.platform,
            self.platform.name()
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

        if self.platform_capabilities.cuda_available {
            s.push_str("  CUDA: available\n");
        }
        if self.platform_capabilities.npu_available {
            if let Some(tops) = self.platform_capabilities.npu_tops {
                s.push_str(&format!("  NPU: available ({} TOPS)\n", tops));
            } else {
                s.push_str("  NPU: available\n");
            }
        }

        s
    }

    // =========================================================================
    // Private Detection Helpers
    // =========================================================================

    /// Detect NUMA topology (node count and CPU mapping).
    fn detect_numa_topology() -> (usize, HashMap<usize, Vec<usize>>) {
        #[cfg(target_os = "linux")]
        {
            let mut topology = HashMap::new();
            let mut max_node = 0;

            // Try to read NUMA node information
            let node_base = std::path::Path::new("/sys/devices/system/node");
            if node_base.exists() {
                if let Ok(entries) = std::fs::read_dir(node_base) {
                    for entry in entries.flatten() {
                        let name = entry.file_name();
                        let name = name.to_string_lossy();
                        if let Some(node_str) = name.strip_prefix("node") {
                            if let Ok(node_id) = node_str.parse::<usize>() {
                                max_node = max_node.max(node_id + 1);

                                // Read CPUs for this node
                                let cpulist_path = entry.path().join("cpulist");
                                if let Ok(cpulist) = std::fs::read_to_string(&cpulist_path) {
                                    let cpus = Self::parse_cpu_list(cpulist.trim());
                                    if !cpus.is_empty() {
                                        topology.insert(node_id, cpus);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // If we found NUMA info, use it
            if max_node > 0 {
                return (max_node, topology);
            }

            // Otherwise, assume single NUMA node with all CPUs
            let cpu_count = std::thread::available_parallelism()
                .map(|p| p.get())
                .unwrap_or(1);
            topology.insert(0, (0..cpu_count).collect());
            (1, topology)
        }

        #[cfg(not(target_os = "linux"))]
        {
            let cpu_count = std::thread::available_parallelism()
                .map(|p| p.get())
                .unwrap_or(1);
            let mut topology = HashMap::new();
            topology.insert(0, (0..cpu_count).collect());
            (1, topology)
        }
    }

    /// Parse a CPU list string like "0-3,7,9-11" into individual CPU indices.
    fn parse_cpu_list(s: &str) -> Vec<usize> {
        parse_cpu_list(s)
    }

    /// Get the current RLIMIT_MEMLOCK soft limit.
    fn get_memlock_limit() -> u64 {
        #[cfg(target_os = "linux")]
        {
            // SAFETY: getrlimit is a safe libc call
            unsafe {
                let mut rlim = libc::rlimit {
                    rlim_cur: 0,
                    rlim_max: 0,
                };
                if libc::getrlimit(libc::RLIMIT_MEMLOCK, &mut rlim) == 0 {
                    if rlim.rlim_cur == libc::RLIM_INFINITY {
                        u64::MAX
                    } else {
                        rlim.rlim_cur
                    }
                } else {
                    0
                }
            }
        }

        #[cfg(not(target_os = "linux"))]
        {
            0
        }
    }

    /// Check if we can actually set RT priority (not just that kernel supports it).
    fn can_set_rt_priority_impl() -> bool {
        #[cfg(target_os = "linux")]
        {
            // Try to query if we could set SCHED_FIFO
            // This checks CAP_SYS_NICE or RLIMIT_RTPRIO
            // SAFETY: getrlimit/geteuid are always safe to call with valid constants
            unsafe {
                // Check RLIMIT_RTPRIO
                let mut rlim = libc::rlimit {
                    rlim_cur: 0,
                    rlim_max: 0,
                };
                if libc::getrlimit(libc::RLIMIT_RTPRIO, &mut rlim) == 0 {
                    // If limit > 0, we can use RT priorities
                    if rlim.rlim_cur > 0 {
                        return true;
                    }
                }

                // If RLIMIT_RTPRIO is 0, we might still have CAP_SYS_NICE
                // Check by trying to get the current scheduler (which always works)
                // and seeing if we're running as root
                let euid = libc::geteuid();
                if euid == 0 {
                    return true; // Root can always set RT priority
                }

                // On non-root systems without RLIMIT_RTPRIO, RT is likely unavailable
                false
            }
        }

        #[cfg(not(target_os = "linux"))]
        {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_capabilities() {
        let caps = RuntimeCapabilities::detect();

        // Should detect something
        assert!(caps.cpu_count > 0);
        println!("Detected capabilities:\n{}", caps.summary());
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

        // Should have at least one NUMA node
        assert!(caps.numa_node_count >= 1);

        // Node 0 should have CPUs
        let node0_cpus = caps.cpus_on_numa_node(0);
        assert!(!node0_cpus.is_empty());
    }

    #[test]
    fn test_get_rt_cpus() {
        let caps = RuntimeCapabilities::detect();

        // Should return up to the requested count
        let cpus = caps.get_rt_cpus(2);
        assert!(cpus.len() <= 2);

        // With count 0, should return empty
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

        // If RT available, priority should be reasonable (1-99)
        // If not available, should be 0
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

        // These should not panic
        let _ = caps.has_rt_support();
        let _ = caps.has_hard_rt_support();
        let _ = caps.can_lock_memory();
        let _ = caps.has_isolated_cpus();
        let _ = caps.is_numa_system();
        let _ = caps.has_cuda();
        let _ = caps.has_npu();
    }

    #[test]
    fn test_summary_output() {
        let caps = RuntimeCapabilities::detect();
        let summary = caps.summary();

        // Should contain key information
        assert!(summary.contains("Platform:"));
        assert!(summary.contains("RT Priority:"));
        assert!(summary.contains("Memory Lock:"));
        assert!(summary.contains("CPUs:"));
    }

    #[test]
    fn test_detection_time() {
        let caps = RuntimeCapabilities::detect();

        // Detection should be reasonably fast (< 100ms)
        assert!(caps.detection_time_us < 100_000);

        println!("Detection took {}μs", caps.detection_time_us);
    }
}
