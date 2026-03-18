// Linux RT: SCHED_FIFO + mlockall + sched_setaffinity + /proc detection

use super::RtCapabilities;
use std::time::Duration;

/// Detect Linux RT capabilities.
pub(super) fn detect_capabilities() -> RtCapabilities {
    let kernel_version = get_kernel_version();
    let preempt_rt = detect_preempt_rt(&kernel_version);
    let (min_priority, max_priority) = get_priority_range();
    let memory_locking = check_mlockall_permitted();
    let cpu_count = std::thread::available_parallelism()
        .map(|p| p.get())
        .unwrap_or(1);

    RtCapabilities {
        preempt_rt,
        max_priority,
        min_priority,
        memory_locking,
        cpu_affinity: true,
        kernel_version,
        cpu_count,
        estimated_jitter: if preempt_rt {
            Duration::from_micros(10)
        } else {
            Duration::from_micros(100)
        },
    }
}

/// Set SCHED_FIFO priority for the current thread.
pub(super) fn set_realtime_priority(priority: i32) -> anyhow::Result<()> {
    // SAFETY: pid 0 = current thread; sched_param is properly initialized
    let param = libc::sched_param {
        sched_priority: priority,
    };
    let result = unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) };

    if result == 0 {
        Ok(())
    } else {
        let err = std::io::Error::last_os_error();
        if err.raw_os_error() == Some(libc::EPERM) {
            anyhow::bail!("SCHED_FIFO requires CAP_SYS_NICE or root: {}", err)
        } else {
            anyhow::bail!(
                "sched_setscheduler(SCHED_FIFO, {}) failed: {}",
                priority,
                err
            )
        }
    }
}

/// Lock all memory pages.
pub(super) fn lock_memory() -> anyhow::Result<()> {
    // SAFETY: MCL_CURRENT | MCL_FUTURE are valid POSIX flags
    let result = unsafe { libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) };

    if result == 0 {
        Ok(())
    } else {
        let err = std::io::Error::last_os_error();
        if err.raw_os_error() == Some(libc::EPERM) {
            anyhow::bail!("mlockall requires CAP_IPC_LOCK or root: {}", err)
        } else {
            anyhow::bail!("mlockall failed: {}", err)
        }
    }
}

/// Check whether RT priority can be set (dry-run via sched_setscheduler).
pub(super) fn can_set_rt_priority() -> bool {
    // Try to set a low RT priority — if it succeeds, we have permission.
    // Immediately restore to SCHED_OTHER afterward.
    let param = libc::sched_param { sched_priority: 1 };
    // SAFETY: pid 0 = current thread; params are valid
    let result = unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) };

    if result == 0 {
        // Restore normal scheduling
        let normal_param = libc::sched_param { sched_priority: 0 };
        // SAFETY: restoring to SCHED_OTHER
        unsafe { libc::sched_setscheduler(0, libc::SCHED_OTHER, &normal_param) };
        true
    } else {
        false
    }
}

/// Detect isolated CPUs (isolcpus kernel parameter).
pub(super) fn detect_isolated_cpus() -> Vec<usize> {
    match std::fs::read_to_string("/sys/devices/system/cpu/isolated") {
        Ok(content) => super::parse_cpu_list(content.trim()),
        Err(_) => Vec::new(),
    }
}

/// Detect nohz_full CPUs (tickless kernel).
pub(super) fn detect_nohz_full_cpus() -> Vec<usize> {
    match std::fs::read_to_string("/sys/devices/system/cpu/nohz_full") {
        Ok(content) => super::parse_cpu_list(content.trim()),
        Err(_) => Vec::new(),
    }
}

/// Detect the CPU frequency governor.
pub(super) fn detect_cpu_governor() -> Option<String> {
    std::fs::read_to_string("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
        .ok()
        .map(|s| s.trim().to_string())
}

/// Get the RLIMIT_MEMLOCK soft limit.
pub(super) fn get_memlock_limit() -> u64 {
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
                rlim.rlim_cur as u64
            }
        } else {
            0
        }
    }
}

// ── Private helpers ─────────────────────────────────────────────────────────

fn get_kernel_version() -> String {
    std::fs::read_to_string("/proc/version")
        .unwrap_or_default()
        .lines()
        .next()
        .unwrap_or("unknown")
        .to_string()
}

fn detect_preempt_rt(kernel_version: &str) -> bool {
    kernel_version.contains("PREEMPT_RT")
        || kernel_version.contains("PREEMPT RT")
        || std::path::Path::new("/sys/kernel/realtime").exists()
}

fn get_priority_range() -> (i32, i32) {
    // SAFETY: safe libc calls that query system limits
    unsafe {
        let min = libc::sched_get_priority_min(libc::SCHED_FIFO);
        let max = libc::sched_get_priority_max(libc::SCHED_FIFO);
        (min.max(1), max.max(1))
    }
}

fn check_mlockall_permitted() -> bool {
    // SAFETY: getrlimit is a safe libc call
    unsafe {
        let mut rlim = libc::rlimit {
            rlim_cur: 0,
            rlim_max: 0,
        };
        if libc::getrlimit(libc::RLIMIT_MEMLOCK, &mut rlim) == 0 {
            rlim.rlim_cur == libc::RLIM_INFINITY || rlim.rlim_cur > 1024 * 1024 * 1024
        } else {
            false
        }
    }
}
