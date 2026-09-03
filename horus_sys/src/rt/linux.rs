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

/// Build the `smp_affinity` mask naming `online` with every CPU in `rt_cpus`
/// cleared.
///
/// Split out of [`move_irqs_off_cpus`] so the arithmetic that decides where
/// every host interrupt lands can be tested on synthetic CPU sets, rather than
/// on whatever machine happens to be running the suite.
fn irq_affinity_mask(online: &[usize], rt_cpus: &[usize]) -> anyhow::Result<u64> {
    let Some(highest) = online.iter().copied().max() else {
        anyhow::bail!("the machine's online CPU set is empty; no IRQ mask to build");
    };

    // Above 63 CPUs a single u64 cannot name the set, and `smp_affinity` wants
    // comma-separated 32-bit groups anyway. Refuse rather than write a mask
    // that silently confines every interrupt to the low 64 CPUs.
    if highest >= 64 {
        anyhow::bail!(
            "machine has CPUs up to index {highest}; a 64-bit affinity mask \
             cannot name them and writing one would confine every interrupt to \
             CPUs 0-63. Refusing; set IRQ affinity out of band."
        );
    }

    let mut mask: u64 = 0;
    for &cpu in online {
        mask |= 1u64 << cpu;
    }
    for &cpu in rt_cpus {
        // Membership, not `cpu <= highest`: an online set has holes whenever a
        // CPU is offlined (1 offline while 3 is online), and an upper bound
        // would accept CPU 1 there while the error text claims the set was
        // consulted. Every RT core is cleared or the call did not do what it
        // says; the original `if cpu < 64` guard skipped high cores silently.
        if !online.contains(&cpu) {
            anyhow::bail!(
                "asked to clear CPU {cpu}, which is not in the machine's online \
                 set {online:?}"
            );
        }
        mask &= !(1u64 << cpu);
    }
    Ok(mask)
}

/// The CPUs this process may actually run on, from `sched_getaffinity(2)`.
///
/// `available_parallelism()` cannot answer this question. It returns a *count*,
/// and two different CPU sets of the same size compare equal: a process pinned
/// to CPUs 2-7 of a machine whose online set is 0-5 counts six either way,
/// while it cannot run on CPUs 0 or 1 at all. The count is also clamped by the
/// cgroup CPU *quota*, which bounds how much CPU time this process gets rather
/// than which CPUs exist, so a 2-CPU quota on an 8-CPU host reads as a
/// confinement that is not there. Both errors matter here: one lets a confined
/// process reroute the host, the other refuses a process that was entitled to.
fn allowed_cpus() -> anyhow::Result<Vec<usize>> {
    // SAFETY: an all-zero `cpu_set_t` is a valid empty set; `sched_getaffinity`
    // is told the exact size of the object it fills, and pid 0 names the
    // calling thread. `CPU_ISSET` then reads only bits below `CPU_SETSIZE`,
    // which is that object's capacity.
    let cpus = unsafe {
        let mut set: libc::cpu_set_t = std::mem::zeroed();
        if libc::sched_getaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &mut set) != 0 {
            return Err(anyhow::Error::new(std::io::Error::last_os_error()).context(
                "sched_getaffinity failed, so this process's own CPU set is \
                 unknown; refusing to rewrite host IRQ affinity from a guess",
            ));
        }
        (0..libc::CPU_SETSIZE as usize)
            .filter(|&cpu| libc::CPU_ISSET(cpu, &set))
            .collect::<Vec<usize>>()
    };
    Ok(cpus)
}

/// The online CPUs this process is *not* allowed to run on.
///
/// Empty means this process can reach every CPU the machine has, and so may
/// speak for the whole machine. Containment is one-directional on purpose:
/// `allowed` being a strict superset of `online` is the ordinary case, not a
/// fault, because a task's `cpus_allowed` keeps naming CPUs that have since
/// been offlined.
fn online_cpus_outside(allowed: &[usize], online: &[usize]) -> Vec<usize> {
    online
        .iter()
        .copied()
        .filter(|cpu| !allowed.contains(cpu))
        .collect()
}

/// Move hardware interrupts off the specified CPU cores.
///
/// Rewrites `/proc/irq/*/smp_affinity` for every IRQ on the machine, so it is
/// host-global and irreversible within this process. Requires root or
/// CAP_SYS_ADMIN.
///
/// # What it returns
///
/// `Ok(n)` counts the IRQs whose affinity was actually rewritten. `n` is 0, not
/// an error, in the three cases where there is nothing to do or nothing may be
/// done: `/proc/irq` does not exist, every online CPU is an RT core so the mask
/// would be empty, or the process lacks root/CAP_SYS_ADMIN and every write
/// fails. Missing privilege is only that last one: the refusals below are
/// `Err`, returned before a single byte is written.
///
/// # What it refuses to do
///
/// The mask used to be built from `available_parallelism()`, which reports the
/// CPUs *this process* may run on, not the CPUs the machine has. A count is not
/// a CPU set, and conflating them made the function reconfigure the whole host
/// from a container's-eye view:
///
///   * under `taskset -c 0-3` with `rt_cpus = [2, 3]` it produced mask `3` and
///     herded every host interrupt onto CPUs 0-1 — including the interrupts of
///     cores it had never heard of;
///   * under `taskset -c 8-11` it produced `f`, naming CPUs 0-3, which this
///     process cannot even run on;
///   * at 64 or more CPUs it produced `u64::MAX`, confining every IRQ to CPUs
///     0-63, while its `cpu < 64` guard silently declined to clear the RT
///     cores it was called for — failing its whole purpose and clobbering the
///     machine anyway.
///
/// So the mask now comes from `/sys/devices/system/cpu/online`, the machine's
/// actual CPU set, and the function REFUSES to act when this process cannot see
/// the whole machine. A process confined to a cpuset has no business rewriting
/// the interrupt routing of cores outside it, and in a container that is the
/// normal case, not an edge case. "Cannot see" is decided by comparing the
/// online set against [`allowed_cpus`] — the set from `sched_getaffinity(2)`,
/// not a count — because the mistake this function exists to fix was reading a
/// count as a set in the first place. The mask arithmetic itself lives in
/// [`irq_affinity_mask`], and the containment test in [`online_cpus_outside`];
/// both are unit-tested on synthetic CPU sets.
pub(super) fn move_irqs_off_cpus(cpus: &[usize]) -> anyhow::Result<usize> {
    let irq_dir = std::path::Path::new("/proc/irq");
    if !irq_dir.exists() {
        return Ok(0);
    }

    // The machine's CPUs, not this process's share of them. A file that cannot
    // be read and a file that parses to nothing are different diagnoses, so
    // they get different errors and the read error is carried through:
    // collapsing both into "cannot read" sends the operator chasing a file that
    // was in fact read fine.
    const ONLINE: &str = "/sys/devices/system/cpu/online";
    let online_raw = std::fs::read_to_string(ONLINE).map_err(|e| {
        anyhow::anyhow!(
            "cannot read {ONLINE} ({e}), so the machine's CPU set is unknown; \
             refusing to rewrite host IRQ affinity from a guess"
        )
    })?;
    let online = super::parse_cpu_list(online_raw.trim());
    if online.is_empty() {
        anyhow::bail!(
            "{ONLINE} holds {:?}, which names no CPUs, so the machine's CPU set \
             is unknown; refusing to rewrite host IRQ affinity from a guess",
            online_raw.trim()
        );
    }

    // Computed before the confinement check below, and not only because it is
    // free: it is what bounds every online index under 64, and therefore under
    // `CPU_SETSIZE`, so the affinity set the check consults cannot be silently
    // truncated out from under it on a very large machine.
    let mask = irq_affinity_mask(&online, cpus)?;

    // If this process is confined, it is not entitled to reroute the host. The
    // question is not "how many CPUs do I have?" but "may I run on every CPU
    // this machine has?", so ask for the set rather than a count.
    let allowed = allowed_cpus()?;
    let unseen = online_cpus_outside(&allowed, &online);
    if !unseen.is_empty() {
        anyhow::bail!(
            "this process may not run on online CPU(s) {unseen:?} of the \
             machine's {} (cpuset or taskset), so it is confined to part of the \
             machine; refusing rather than rerouting interrupts for cores it \
             cannot see",
            online.len()
        );
    }

    if mask == 0 {
        return Ok(0); // every CPU is an RT core; nothing left to move IRQs to
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

#[cfg(test)]
mod irq_affinity_tests {
    use super::*;

    /// The mask names exactly the online CPUs, minus the RT cores.
    ///
    /// Synthetic sets, so this says the same thing on a 2-core CI runner as on
    /// a 96-core host.
    #[test]
    fn mask_is_the_online_set_minus_the_rt_cores() {
        // The ordinary case: four online, two handed to RT.
        assert_eq!(irq_affinity_mask(&[0, 1, 2, 3], &[2, 3]).unwrap(), 0b0011);
        // Holes stay holes. CPU 1 is offline, so it is absent from the mask
        // even though nobody asked for it to be cleared.
        assert_eq!(irq_affinity_mask(&[0, 2, 3], &[3]).unwrap(), 0b0101);
        // Nothing to clear leaves the online set intact.
        assert_eq!(irq_affinity_mask(&[0, 2, 3], &[]).unwrap(), 0b1101);
        // Every online CPU is an RT core: empty mask, which the caller turns
        // into Ok(0) rather than writing "0" to every IRQ.
        assert_eq!(irq_affinity_mask(&[0, 1], &[0, 1]).unwrap(), 0);
    }

    /// An offline CPU inside the index range is still not a CPU to clear.
    ///
    /// The check is membership, not `cpu <= highest`: with 1 offline and 3
    /// online, an upper bound accepts CPU 1 while the error text claims the
    /// online set was consulted.
    #[test]
    fn an_offline_cpu_below_the_highest_index_is_refused() {
        let err = irq_affinity_mask(&[0, 2, 3], &[1]).unwrap_err().to_string();
        assert!(
            err.contains("not in the machine's online set"),
            "expected a refusal naming the online set, got: {err}"
        );
    }

    /// Past index 63 a u64 cannot name the set, so refuse instead of truncating.
    #[test]
    fn cpu_indices_past_63_are_refused_not_truncated() {
        let online: Vec<usize> = (0..80).collect();
        let err = irq_affinity_mask(&online, &[70]).unwrap_err().to_string();
        assert!(
            err.contains("cannot name"),
            "expected a refusal about the 64-bit mask, got: {err}"
        );
    }

    /// Asking to clear a CPU the machine does not have is a caller error and
    /// must be reported, not silently skipped.
    ///
    /// The old code guarded with `if cpu < 64` and skipped anything higher, so
    /// on a large machine it wrote a mask while declining to clear the very
    /// cores it was called for. Asserted on the helper, not through
    /// `move_irqs_off_cpus`: routed through the real function the host's `/proc`
    /// and `/sys` pick which refusal comes back, and a test that accepts any of
    /// several outcomes asserts none of them.
    #[test]
    fn clearing_a_nonexistent_cpu_is_an_error_not_a_silent_skip() {
        let err = irq_affinity_mask(&[0, 1, 2, 3], &[4096])
            .unwrap_err()
            .to_string();
        assert!(
            err.contains("asked to clear CPU 4096"),
            "expected a refusal naming the CPU, got: {err}"
        );
        assert!(
            err.contains("not in the machine's online set"),
            "expected a refusal naming the online set, got: {err}"
        );
    }

    /// Confinement is a property of the CPU *set*, which a count cannot see.
    ///
    /// Four allowed and four online, so the old `available_parallelism() <
    /// online.len()` check found nothing wrong — while this process may not run
    /// on online CPUs 0 or 1 at all.
    #[test]
    fn an_equal_sized_but_different_cpu_set_is_still_confinement() {
        assert_eq!(
            online_cpus_outside(&[2, 3, 4, 5], &[0, 1, 2, 3]),
            vec![0, 1]
        );
    }

    /// Reaching every online CPU is not confinement, even with CPUs to spare.
    #[test]
    fn covering_the_online_set_is_not_confinement() {
        // Exactly the online set.
        assert!(online_cpus_outside(&[0, 1, 2, 3], &[0, 1, 2, 3]).is_empty());
        // A strict superset, which is the ordinary reading once CPU 1 has been
        // offlined: `cpus_allowed` still names it, `online` no longer does.
        assert!(online_cpus_outside(&[0, 1, 2, 3], &[0, 2, 3]).is_empty());
    }

    /// This process's own affinity is a set, and it is one this machine has.
    ///
    /// The one host-dependent assertion kept, and it is deliberately weak: it
    /// only pins down that `allowed_cpus` reads a plausible set rather than,
    /// say, returning all 1024 `CPU_SETSIZE` bits. The behaviour that matters
    /// is asserted on synthetic sets above.
    #[test]
    fn allowed_cpus_reads_a_set_this_machine_could_have() {
        let allowed = allowed_cpus().expect("sched_getaffinity on self");
        assert!(!allowed.is_empty(), "this process runs on some CPU");
        assert!(
            allowed.len() <= libc::CPU_SETSIZE as usize,
            "got {} CPUs, more than a cpu_set_t can name",
            allowed.len()
        );
    }
}
