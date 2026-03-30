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

// ── SCHED_DEADLINE (EDF kernel scheduler) ──────────────────────────────

/// Kernel sched_attr struct for sched_setattr() syscall.
/// Not in libc crate — must be defined manually.
#[repr(C)]
struct SchedAttr {
    size: u32,
    sched_policy: u32,
    sched_flags: u64,
    sched_nice: i32,
    sched_priority: u32,
    sched_runtime: u64,
    sched_deadline: u64,
    sched_period: u64,
}

const SCHED_DEADLINE_POLICY: u32 = 6;

/// Syscall number for sched_setattr (not in libc crate).
#[cfg(target_arch = "x86_64")]
const SYS_SCHED_SETATTR: libc::c_long = 314;
#[cfg(target_arch = "aarch64")]
const SYS_SCHED_SETATTR: libc::c_long = 274;
#[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
const SYS_SCHED_SETATTR: libc::c_long = -1; // unsupported

/// Set SCHED_DEADLINE (EDF) scheduling for the current thread.
///
/// The kernel guarantees this thread gets `runtime_ns` of CPU time within
/// every `period_ns` window, completing by `deadline_ns` after the period
/// start. Admission control rejects requests that would overcommit CPU.
///
/// - `runtime_ns`: maximum CPU time per period (the "budget")
/// - `deadline_ns`: absolute deadline within the period (usually ≤ period)
/// - `period_ns`: period length
///
/// Requires `CAP_SYS_NICE` or root. Returns specific errors for:
/// - EPERM: insufficient privileges
/// - EBUSY: admission rejected (CPU overcommitted)
/// - ENOSYS: kernel doesn't support SCHED_DEADLINE
pub(super) fn set_deadline_scheduling(
    runtime_ns: u64,
    deadline_ns: u64,
    period_ns: u64,
) -> anyhow::Result<()> {
    if SYS_SCHED_SETATTR < 0 {
        anyhow::bail!("SCHED_DEADLINE not supported on this architecture");
    }

    let attr = SchedAttr {
        size: std::mem::size_of::<SchedAttr>() as u32,
        sched_policy: SCHED_DEADLINE_POLICY,
        sched_flags: 0,
        sched_nice: 0,
        sched_priority: 0, // must be 0 for SCHED_DEADLINE
        sched_runtime: runtime_ns,
        sched_deadline: deadline_ns,
        sched_period: period_ns,
    };

    // SAFETY: sched_setattr is a Linux syscall. pid=0 means current thread.
    // The SchedAttr struct is properly initialized with correct size field.
    let result = unsafe { libc::syscall(SYS_SCHED_SETATTR, 0i32, &attr as *const _, 0u32) };

    if result == 0 {
        Ok(())
    } else {
        let err = std::io::Error::last_os_error();
        match err.raw_os_error() {
            Some(libc::EPERM) => {
                anyhow::bail!("SCHED_DEADLINE requires CAP_SYS_NICE or root: {}", err)
            }
            Some(libc::EBUSY) => anyhow::bail!(
                "SCHED_DEADLINE admission rejected (CPU overcommitted): runtime={}ns deadline={}ns period={}ns",
                runtime_ns, deadline_ns, period_ns
            ),
            Some(libc::ENOSYS) => {
                anyhow::bail!("SCHED_DEADLINE not supported by this kernel (requires Linux 3.14+)")
            }
            Some(libc::EINVAL) => anyhow::bail!(
                "SCHED_DEADLINE invalid parameters: runtime={}ns must be ≤ deadline={}ns ≤ period={}ns",
                runtime_ns, deadline_ns, period_ns
            ),
            _ => anyhow::bail!("sched_setattr(SCHED_DEADLINE) failed: {}", err),
        }
    }
}

/// Check if SCHED_DEADLINE is available on this kernel.
pub(super) fn has_deadline_capability() -> bool {
    // Try with minimal valid params — kernel will reject with EPERM (no privilege)
    // or EBUSY (admission) but NOT ENOSYS (unsupported). ENOSYS means no support.
    let attr = SchedAttr {
        size: std::mem::size_of::<SchedAttr>() as u32,
        sched_policy: SCHED_DEADLINE_POLICY,
        sched_flags: 0,
        sched_nice: 0,
        sched_priority: 0,
        sched_runtime: 1_000_000,  // 1ms
        sched_deadline: 2_000_000, // 2ms
        sched_period: 2_000_000,   // 2ms
    };
    // SAFETY: dry-run probe — we expect this to fail (EPERM) but not ENOSYS.
    let result = unsafe { libc::syscall(SYS_SCHED_SETATTR, 0i32, &attr as *const _, 0u32) };
    if result == 0 {
        // Unexpectedly succeeded — restore to SCHED_OTHER
        let normal = libc::sched_param {
            sched_priority: 0,
        };
        unsafe {
            libc::sched_setscheduler(0, libc::SCHED_OTHER, &normal);
        }
        return true;
    }
    let err = std::io::Error::last_os_error();
    // ENOSYS = kernel doesn't have it. Anything else = kernel has it but we lack permission.
    err.raw_os_error() != Some(libc::ENOSYS)
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

/// Set the CPU frequency governor for a specific core.
///
/// Writes to `/sys/devices/system/cpu/cpu{N}/cpufreq/scaling_governor`.
/// Requires root or appropriate sysfs permissions.
pub(super) fn set_cpu_governor(cpu_id: usize, governor: &str) -> anyhow::Result<()> {
    let path = format!(
        "/sys/devices/system/cpu/cpu{}/cpufreq/scaling_governor",
        cpu_id
    );
    std::fs::write(&path, governor).map_err(|e| {
        anyhow::anyhow!(
            "Failed to set CPU {} governor to '{}': {}",
            cpu_id,
            governor,
            e
        )
    })
}

/// Move hardware interrupts off the specified CPU cores.
///
/// Iterates `/proc/irq/*/smp_affinity` and clears the bits for the given cores.
/// Returns the number of IRQs whose affinity was successfully changed.
/// Requires root or CAP_SYS_ADMIN.
pub(super) fn move_irqs_off_cpus(cpus: &[usize]) -> anyhow::Result<usize> {
    let irq_dir = std::path::Path::new("/proc/irq");
    if !irq_dir.exists() {
        return Ok(0);
    }
    let total_cpus = std::thread::available_parallelism()
        .map(|p| p.get())
        .unwrap_or(1);
    // Build mask with RT cores cleared
    let mut mask: u64 = if total_cpus >= 64 {
        u64::MAX
    } else {
        (1u64 << total_cpus) - 1
    };
    for &cpu in cpus {
        if cpu < 64 {
            mask &= !(1u64 << cpu);
        }
    }
    if mask == 0 {
        return Ok(0); // can't clear all cores
    }
    let mask_str = format!("{:x}", mask);
    let mut moved = 0usize;
    for entry in std::fs::read_dir(irq_dir)?.flatten() {
        let affinity_path = entry.path().join("smp_affinity");
        if affinity_path.exists() && std::fs::write(&affinity_path, &mask_str).is_ok() {
            moved += 1;
        }
    }
    Ok(moved)
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
                rlim.rlim_cur
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
