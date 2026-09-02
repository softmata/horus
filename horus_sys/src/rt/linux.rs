// Linux RT: SCHED_FIFO + mlockall + sched_setaffinity + /proc detection

use super::RtCapabilities;
use std::time::Duration;

/// Build a `sched_param` carrying just a priority.
///
/// `sched_param` is not field-compatible across libcs: glibc exposes only
/// `sched_priority`, while musl also carries the four POSIX sporadic-server
/// fields (`sched_ss_low_priority`, `sched_ss_repl_period`,
/// `sched_ss_init_budget`, `sched_ss_max_repl`). A struct literal naming only
/// `sched_priority` therefore fails to compile on musl (E0063), which is what
/// broke the Alpine image in docker-distro-tests. Zero-initialising and then
/// setting the one field we care about is correct on both: the sporadic-server
/// members are ignored by SCHED_FIFO and SCHED_OTHER.
fn sched_param_with_priority(priority: i32) -> libc::sched_param {
    // SAFETY: sched_param is a plain C struct of integer fields; all-zero is a
    // valid bit pattern for it.
    let mut param: libc::sched_param = unsafe { std::mem::zeroed() };
    param.sched_priority = priority;
    param
}

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

/// Set this thread's timer slack, in nanoseconds.
///
/// Linux gives every thread 50 us of timer slack by default (see
/// `/proc/self/timerslack_ns`). The kernel is then free to delay any
/// `nanosleep`, `clock_nanosleep`, `futex` timeout or poll wake by up to that
/// much, so it can batch wakeups and save power. For a periodic control loop
/// that is a 50 us error added to every single wake.
///
/// It applies to SCHED_OTHER threads. A SCHED_FIFO/RR thread already gets zero
/// slack from the kernel, so this call is a no-op for a fully privileged RT
/// deployment -- and exactly what is needed for the one that could not get
/// SCHED_FIFO, which is the common case (it needs CAP_SYS_NICE, and a plain
/// `cargo test` or an unprivileged container does not have it). HORUS
/// deliberately continues at normal priority when that happens, and this is
/// what makes that fallback behave.
///
/// Measured on an idle 12-core box, 3000 iterations of a 1 kHz
/// `clock_nanosleep(TIMER_ABSTIME)` loop, wake lateness:
///
/// ```text
///   50000 ns slack (default):  p50 55.8 us   p90 63.8 us   p99 101-271 us
///       1 ns slack:            p50  4.3 us   p90 16.7 us   p99  25-116 us
/// ```
pub(super) fn set_timer_slack(nanoseconds: u64) -> anyhow::Result<()> {
    // SAFETY: PR_SET_TIMERSLACK takes a single unsigned long argument and
    // affects only the calling thread. A value of 0 means "restore the
    // default", which is why the caller is expected to pass >= 1.
    let result = unsafe { libc::prctl(libc::PR_SET_TIMERSLACK, nanoseconds as libc::c_ulong) };
    if result == 0 {
        Ok(())
    } else {
        Err(std::io::Error::last_os_error())
            .map_err(|e| anyhow::anyhow!("prctl(PR_SET_TIMERSLACK, {}) failed: {}", nanoseconds, e))
    }
}

/// This thread's current timer slack in nanoseconds, if it can be read.
pub(super) fn timer_slack_ns() -> Option<u64> {
    std::fs::read_to_string("/proc/self/timerslack_ns")
        .ok()?
        .trim()
        .parse()
        .ok()
}

/// Set SCHED_FIFO priority for the current thread.
pub(super) fn set_realtime_priority(priority: i32) -> anyhow::Result<()> {
    // SAFETY: pid 0 = current thread; sched_param is properly initialized
    let param = sched_param_with_priority(priority);
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

/// Snapshot of the calling thread's scheduling policy and parameters.
///
/// Capability probes here mutate the *calling* thread, and the caller is not a
/// throwaway: `RuntimeCapabilities::detect()` runs on whichever thread builds a
/// `Scheduler`, which on a hard-RT deployment (`chrt -f 80 ./node`, or an
/// earlier `RtConfig::apply()`) is already SCHED_FIFO. The probes used to
/// "restore" a hardcoded SCHED_OTHER/priority 0, so detection silently demoted
/// the very thread it then reported as RT-capable. Every probe must therefore
/// either be reversible against what was really there, or not run at all.
struct SchedSnapshot {
    policy: i32,
    param: libc::sched_param,
}

/// Read the current thread's scheduling policy and parameters.
///
/// Returns `None` when either query fails, which is the signal to decline the
/// probe rather than guess at a restore target.
fn snapshot_scheduling() -> Option<SchedSnapshot> {
    // SAFETY: pid 0 = current thread; both calls only read, and `param` is a
    // local POD struct for which all-zero is a valid bit pattern.
    unsafe {
        let policy = libc::sched_getscheduler(0);
        if policy < 0 {
            return None;
        }
        let mut param: libc::sched_param = std::mem::zeroed();
        if libc::sched_getparam(0, &mut param) != 0 {
            return None;
        }
        Some(SchedSnapshot { policy, param })
    }
}

/// Put the calling thread back exactly where `snapshot_scheduling` found it.
fn restore_scheduling(snapshot: &SchedSnapshot) {
    // SAFETY: pid 0 = current thread; policy and param came from this thread.
    let ret = unsafe { libc::sched_setscheduler(0, snapshot.policy, &snapshot.param) };
    if ret != 0 {
        // A demotion must never be silent — it costs the caller its RT guarantee.
        log::warn!(
            "RT capability probe failed to restore scheduling policy {} priority {}: {}",
            snapshot.policy,
            snapshot.param.sched_priority,
            std::io::Error::last_os_error()
        );
    }
}

/// Check if SCHED_DEADLINE is available on this kernel.
pub(super) fn has_deadline_capability() -> bool {
    // A thread already running SCHED_DEADLINE proves the kernel supports it,
    // and probing would clobber its reservation.
    // SAFETY: pid 0 = current thread; sched_getscheduler only reads.
    if unsafe { libc::sched_getscheduler(0) } == SCHED_DEADLINE_POLICY as i32 {
        return true;
    }

    // The probe below can unexpectedly succeed and move this thread onto
    // SCHED_DEADLINE, so it is only safe to run if it can be undone exactly.
    let Some(snapshot) = snapshot_scheduling() else {
        return false;
    };

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
        // Unexpectedly succeeded — put back the policy the thread actually had,
        // not a hardcoded SCHED_OTHER.
        restore_scheduling(&snapshot);
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

/// Check whether RT priority can be set.
///
/// Answered read-only wherever possible. The previous implementation probed by
/// calling `sched_setscheduler(0, SCHED_FIFO, 1)` on the caller and then forcing
/// SCHED_OTHER priority 0, so a thread deployed the normal hard-RT way was
/// stripped of its real-time class by the very check that reported RT as
/// available — and the scheduler's tick loop, which runs on that thread, then
/// ran time-shared while `rt_priority_available` said otherwise.
pub(super) fn can_set_rt_priority() -> bool {
    // A thread already on an RT policy demonstrably holds the privilege, so
    // there is nothing to probe — and probing is exactly what used to break it.
    // SAFETY: pid 0 = current thread; sched_getscheduler only reads.
    let current_policy = unsafe { libc::sched_getscheduler(0) };
    if current_policy == libc::SCHED_FIFO
        || current_policy == libc::SCHED_RR
        || current_policy == SCHED_DEADLINE_POLICY as i32
    {
        return true;
    }

    // Root can always raise the scheduling class.
    // SAFETY: geteuid cannot fail and mutates nothing.
    if unsafe { libc::geteuid() } == 0 {
        return true;
    }

    // A non-zero RLIMIT_RTPRIO ceiling means unprivileged SCHED_FIFO is allowed.
    // SAFETY: getrlimit only writes into the local rlimit struct.
    let rtprio_allowed = unsafe {
        let mut rlim = libc::rlimit {
            rlim_cur: 0,
            rlim_max: 0,
        };
        libc::getrlimit(libc::RLIMIT_RTPRIO, &mut rlim) == 0 && rlim.rlim_cur > 0
    };
    if rtprio_allowed {
        return true;
    }

    // Inconclusive — e.g. CAP_SYS_NICE granted with no RTPRIO ceiling. Fall back
    // to the live probe, but only when it can be undone exactly; if the thread's
    // current policy cannot be read back, decline to probe rather than risk
    // leaving the caller on a scheduling class it never chose.
    let Some(snapshot) = snapshot_scheduling() else {
        return false;
    };

    let param = sched_param_with_priority(1);
    // SAFETY: pid 0 = current thread; params are valid
    let result = unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) };

    if result == 0 {
        restore_scheduling(&snapshot);
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
//
// `clippy::useless_conversion` fires on the `u64::from(rlim.rlim_cur)` below,
// but only when checked for a 64-bit target. `rlim_t` is `u32` on
// armv7-unknown-linux-gnueabihf, which `horus deploy --arch armv7` targets and
// which is an installed rust-std here; dropping the conversion there fails with
// `error[E0308]: expected u64, found u32`. The conversion is width-portable and
// must stay.
#[allow(clippy::useless_conversion)]
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
                // `rlim_t` is 32-bit on armv7-unknown-linux-gnueabihf and
                // 64-bit on x86_64/aarch64. `u64::from` covers both widths;
                // returning the field directly fails to compile for 32-bit ARM,
                // which `horus deploy --arch armv7` targets.
                u64::from(rlim.rlim_cur)
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
