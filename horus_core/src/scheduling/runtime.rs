//! Runtime configuration and OS-level features
//!
//! This module provides real implementations for:
//! - CPU core affinity (thread pinning)
//! - Memory locking (mlockall)
//! - Real-time scheduling (SCHED_FIFO)
//! - NUMA awareness

use std::io;

/// Result type for runtime operations
pub type RuntimeResult<T> = Result<T, RuntimeError>;

/// Errors from runtime operations
#[derive(Debug)]
pub enum RuntimeError {
    /// Failed to set CPU affinity
    AffinityError(String),
    /// Failed to lock memory
    MemoryLockError(String),
    /// Failed to set RT scheduling
    SchedulingError(String),
    /// Feature not supported on this platform
    NotSupported(String),
    /// Permission denied
    PermissionDenied(String),
}

impl std::fmt::Display for RuntimeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RuntimeError::AffinityError(msg) => write!(f, "CPU affinity error: {}", msg),
            RuntimeError::MemoryLockError(msg) => write!(f, "Memory lock error: {}", msg),
            RuntimeError::SchedulingError(msg) => write!(f, "Scheduling error: {}", msg),
            RuntimeError::NotSupported(msg) => write!(f, "Not supported: {}", msg),
            RuntimeError::PermissionDenied(msg) => write!(f, "Permission denied: {}", msg),
        }
    }
}

impl std::error::Error for RuntimeError {}

// ============================================================================
// CPU Core Affinity
// ============================================================================

/// Pin the current thread to specific CPU cores
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

/// Get available CPU core count
pub fn get_core_count() -> usize {
    core_affinity::get_core_ids()
        .map(|ids| ids.len())
        .unwrap_or_else(num_cpus::get)
}

/// Pin a thread by its native handle (for spawned threads)
#[cfg(target_os = "linux")]
pub fn set_thread_affinity_by_id(thread_id: libc::pthread_t, core: usize) -> RuntimeResult<()> {
    // SAFETY: thread_id is a valid pthread handle; cpuset is properly zeroed and initialized.
    unsafe {
        let mut cpuset: libc::cpu_set_t = std::mem::zeroed();
        libc::CPU_ZERO(&mut cpuset);
        libc::CPU_SET(core, &mut cpuset);

        let result = libc::pthread_setaffinity_np(
            thread_id,
            std::mem::size_of::<libc::cpu_set_t>(),
            &cpuset,
        );

        if result == 0 {
            Ok(())
        } else {
            Err(RuntimeError::AffinityError(format!(
                "pthread_setaffinity_np failed: {}",
                io::Error::last_os_error()
            )))
        }
    }
}

#[cfg(not(target_os = "linux"))]
pub fn set_thread_affinity_by_id(_thread_id: u64, _core: usize) -> RuntimeResult<()> {
    Err(RuntimeError::NotSupported(
        "Thread affinity by ID only supported on Linux".to_string(),
    ))
}

// ============================================================================
// Memory Locking
// ============================================================================

/// Lock all current and future memory pages (prevents swapping)
/// Requires CAP_IPC_LOCK capability or root on Linux
#[cfg(target_os = "linux")]
pub fn lock_all_memory() -> RuntimeResult<()> {
    // SAFETY: MCL_CURRENT | MCL_FUTURE are valid POSIX flag constants for mlockall.
    unsafe {
        // MCL_CURRENT | MCL_FUTURE
        let flags = libc::MCL_CURRENT | libc::MCL_FUTURE;
        let result = libc::mlockall(flags);

        if result == 0 {
            println!("[RT] All memory locked (mlockall)");
            Ok(())
        } else {
            let err = io::Error::last_os_error();
            if err.raw_os_error() == Some(libc::EPERM) {
                Err(RuntimeError::PermissionDenied(
                    "mlockall requires CAP_IPC_LOCK or root".to_string(),
                ))
            } else {
                Err(RuntimeError::MemoryLockError(format!(
                    "mlockall failed: {}",
                    err
                )))
            }
        }
    }
}

#[cfg(not(target_os = "linux"))]
pub fn lock_all_memory() -> RuntimeResult<()> {
    Err(RuntimeError::NotSupported(
        "Memory locking only supported on Linux".to_string(),
    ))
}

/// Unlock all memory pages
#[cfg(target_os = "linux")]
pub fn unlock_all_memory() -> RuntimeResult<()> {
    // SAFETY: munlockall requires no arguments and is always safe to call.
    unsafe {
        let result = libc::munlockall();
        if result == 0 {
            println!("[RT] Memory unlocked (munlockall)");
            Ok(())
        } else {
            Err(RuntimeError::MemoryLockError(format!(
                "munlockall failed: {}",
                io::Error::last_os_error()
            )))
        }
    }
}

#[cfg(not(target_os = "linux"))]
pub fn unlock_all_memory() -> RuntimeResult<()> {
    Ok(()) // No-op on non-Linux
}

/// Pre-fault stack memory to avoid page faults during execution
pub fn prefault_stack(stack_size: usize) -> RuntimeResult<()> {
    // Allocate and touch stack memory to force page allocation
    let mut stack_buffer = vec![0u8; stack_size];

    // Touch every page (typically 4KB)
    let page_size = 4096;
    for i in (0..stack_size).step_by(page_size) {
        stack_buffer[i] = 1;
    }

    // Prevent optimization from removing the touches
    std::hint::black_box(&stack_buffer);

    println!("[RT] Pre-faulted {}KB of stack", stack_size / 1024);
    Ok(())
}

// ============================================================================
// Real-Time Scheduling
// ============================================================================

/// Set real-time scheduling policy for current thread
/// Requires CAP_SYS_NICE capability or root on Linux
#[cfg(target_os = "linux")]
pub fn set_realtime_priority(priority: i32) -> RuntimeResult<()> {
    // SAFETY: pid 0 = current thread; sched_param is properly initialized with valid priority.
    unsafe {
        let param = libc::sched_param {
            sched_priority: priority,
        };

        // SCHED_FIFO for real-time scheduling
        let result = libc::sched_setscheduler(0, libc::SCHED_FIFO, &param);

        if result == 0 {
            println!("[RT] Set SCHED_FIFO with priority {}", priority);
            Ok(())
        } else {
            let err = io::Error::last_os_error();
            if err.raw_os_error() == Some(libc::EPERM) {
                Err(RuntimeError::PermissionDenied(
                    "SCHED_FIFO requires CAP_SYS_NICE or root".to_string(),
                ))
            } else {
                Err(RuntimeError::SchedulingError(format!(
                    "sched_setscheduler failed: {}",
                    err
                )))
            }
        }
    }
}

#[cfg(not(target_os = "linux"))]
pub fn set_realtime_priority(_priority: i32) -> RuntimeResult<()> {
    Err(RuntimeError::NotSupported(
        "SCHED_FIFO only supported on Linux".to_string(),
    ))
}

/// Get the maximum real-time priority available
#[cfg(target_os = "linux")]
pub fn get_max_rt_priority() -> i32 {
    // SAFETY: SCHED_FIFO is a valid scheduling policy constant.
    unsafe { libc::sched_get_priority_max(libc::SCHED_FIFO) }
}

#[cfg(not(target_os = "linux"))]
pub fn get_max_rt_priority() -> i32 {
    99 // Default Linux max
}

/// Get the minimum real-time priority available
#[cfg(target_os = "linux")]
pub fn get_min_rt_priority() -> i32 {
    // SAFETY: SCHED_FIFO is a valid scheduling policy constant.
    unsafe { libc::sched_get_priority_min(libc::SCHED_FIFO) }
}

#[cfg(not(target_os = "linux"))]
pub fn get_min_rt_priority() -> i32 {
    1 // Default Linux min
}

/// Set thread to round-robin scheduling (less aggressive than FIFO)
#[cfg(target_os = "linux")]
pub fn set_round_robin_priority(priority: i32) -> RuntimeResult<()> {
    // SAFETY: pid 0 = current thread; sched_param is properly initialized with valid priority.
    unsafe {
        let param = libc::sched_param {
            sched_priority: priority,
        };

        let result = libc::sched_setscheduler(0, libc::SCHED_RR, &param);

        if result == 0 {
            println!("[RT] Set SCHED_RR with priority {}", priority);
            Ok(())
        } else {
            Err(RuntimeError::SchedulingError(format!(
                "sched_setscheduler (RR) failed: {}",
                io::Error::last_os_error()
            )))
        }
    }
}

#[cfg(not(target_os = "linux"))]
pub fn set_round_robin_priority(_priority: i32) -> RuntimeResult<()> {
    Err(RuntimeError::NotSupported(
        "SCHED_RR only supported on Linux".to_string(),
    ))
}

/// Reset to normal scheduling
#[cfg(target_os = "linux")]
pub fn reset_to_normal_scheduling() -> RuntimeResult<()> {
    // SAFETY: pid 0 = current thread; sched_param with priority 0 is valid for SCHED_OTHER.
    unsafe {
        let param = libc::sched_param { sched_priority: 0 };
        let result = libc::sched_setscheduler(0, libc::SCHED_OTHER, &param);

        if result == 0 {
            println!("[RT] Reset to SCHED_OTHER");
            Ok(())
        } else {
            Err(RuntimeError::SchedulingError(format!(
                "Reset scheduling failed: {}",
                io::Error::last_os_error()
            )))
        }
    }
}

#[cfg(not(target_os = "linux"))]
pub fn reset_to_normal_scheduling() -> RuntimeResult<()> {
    Ok(())
}

// ============================================================================
// NUMA Awareness
// ============================================================================

/// Get NUMA node count (returns 1 if NUMA not available)
#[cfg(target_os = "linux")]
pub fn get_numa_node_count() -> usize {
    // Try to read from /sys/devices/system/node/
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

#[cfg(not(target_os = "linux"))]
pub fn get_numa_node_count() -> usize {
    1
}

/// Get CPUs belonging to a NUMA node
#[cfg(target_os = "linux")]
pub fn get_numa_node_cpus(node: usize) -> Vec<usize> {
    let path = format!("/sys/devices/system/node/node{}/cpulist", node);
    if let Ok(content) = std::fs::read_to_string(&path) {
        parse_cpu_list(content.trim())
    } else {
        Vec::new()
    }
}

#[cfg(not(target_os = "linux"))]
pub fn get_numa_node_cpus(_node: usize) -> Vec<usize> {
    Vec::new()
}

/// Parse CPU list format (e.g., "0-3,8-11" -> [0,1,2,3,8,9,10,11])
fn parse_cpu_list(s: &str) -> Vec<usize> {
    let mut cpus = Vec::new();
    for part in s.split(',') {
        let part = part.trim();
        if part.contains('-') {
            let mut range = part.split('-');
            if let (Some(start), Some(end)) = (range.next(), range.next()) {
                if let (Ok(start), Ok(end)) = (start.parse::<usize>(), end.parse::<usize>()) {
                    cpus.extend(start..=end);
                }
            }
        } else if let Ok(cpu) = part.parse::<usize>() {
            cpus.push(cpu);
        }
    }
    cpus
}

// ============================================================================
// Combined RT Setup
// ============================================================================

/// Apply all real-time optimizations at once
pub fn apply_rt_optimizations(
    cpu_cores: Option<&[usize]>,
    lock_memory: bool,
    rt_priority: Option<i32>,
    prefault_stack_kb: Option<usize>,
) -> Vec<RuntimeError> {
    let mut errors = Vec::new();

    // 1. Set CPU affinity
    if let Some(cores) = cpu_cores {
        if let Err(e) = set_thread_affinity(cores) {
            eprintln!("[RT] Warning: {}", e);
            errors.push(e);
        }
    }

    // 2. Lock memory
    if lock_memory {
        if let Err(e) = lock_all_memory() {
            eprintln!("[RT] Warning: {}", e);
            errors.push(e);
        }
    }

    // 3. Set RT priority
    if let Some(priority) = rt_priority {
        if let Err(e) = set_realtime_priority(priority) {
            eprintln!("[RT] Warning: {}", e);
            errors.push(e);
        }
    }

    // 4. Pre-fault stack
    if let Some(kb) = prefault_stack_kb {
        if let Err(e) = prefault_stack(kb * 1024) {
            eprintln!("[RT] Warning: {}", e);
            errors.push(e);
        }
    }

    if errors.is_empty() {
        println!("[RT] All real-time optimizations applied successfully");
    } else {
        println!(
            "[RT] Applied with {} warnings (may need root/capabilities)",
            errors.len()
        );
    }

    errors
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_core_count() {
        let count = get_core_count();
        assert!(count > 0);
        println!("Core count: {}", count);
    }

    #[test]
    fn test_parse_cpu_list() {
        assert_eq!(parse_cpu_list("0-3"), vec![0, 1, 2, 3]);
        assert_eq!(parse_cpu_list("0,2,4"), vec![0, 2, 4]);
        assert_eq!(parse_cpu_list("0-2,4-5"), vec![0, 1, 2, 4, 5]);
    }

    #[test]
    fn test_numa_node_count() {
        let count = get_numa_node_count();
        assert!(count >= 1);
        println!("NUMA nodes: {}", count);
    }

    #[test]
    fn test_prefault_stack() {
        // Should work without privileges
        let result = prefault_stack(64 * 1024); // 64KB
        assert!(result.is_ok());
    }
}
