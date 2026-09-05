// Linux RT: SCHED_FIFO + mlockall + sched_setaffinity + /proc detection

use super::{
    BestEffortReport, Clocksource, PreemptInfo, PreemptModel, PreemptSource, RtBandwidth,
    RtBandwidthSource, RtCapabilities,
};
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
    let preempt = read_preempt_info();
    let preempt_rt = preempt.model == PreemptModel::PreemptRt;
    let clocksource = read_clocksource();
    let (min_priority, max_priority) = get_priority_range();
    let memory_locking = check_mlockall_permitted();
    let cpu_count = std::thread::available_parallelism()
        .map(|p| p.get())
        .unwrap_or(1);

    RtCapabilities {
        preempt_rt,
        max_priority,
        // Probed, not inferred from `max_priority` — that is a kernel constant
        // and is non-zero even where the call is refused with EPERM.
        rt_priority_permitted: can_set_rt_priority(),
        min_priority,
        memory_locking,
        cpu_affinity: true,
        kernel_version,
        cpu_count,
        // Derived from the MODEL, not from the bool. The bool put a
        // `preempt=none` kernel — worst-case scheduling latency in the
        // milliseconds — in the same bucket as a fully preemptible one, and
        // quoted 100 us for both.
        estimated_jitter: match preempt.model {
            PreemptModel::PreemptRt => Duration::from_micros(10),
            PreemptModel::Full | PreemptModel::Lazy => Duration::from_micros(100),
            PreemptModel::Voluntary => Duration::from_micros(1_000),
            PreemptModel::None => Duration::from_micros(10_000),
            // An unknown model is not a claim about latency. 100 us is what
            // this reported before the model was read at all, so an
            // undetectable kernel keeps the number it always had.
            PreemptModel::Unknown | PreemptModel::NotApplicable => Duration::from_micros(100),
        },
        preempt,
        deepest_idle_state: deepest_idle_exit_latency_at(std::path::Path::new(CPU_SYSFS_ROOT), 0),
        clocksource,
    }
}

/// Set this thread's timer slack, in nanoseconds.
///
/// A thread starts with 50 us of timer slack: 50000 ns is the value the kernel
/// gives `init_task`, and every task inherits its parent's current slack as its
/// own default across both fork and exec, so 50 us is what anything launched
/// from an ordinary shell begins with. It is inherited rather than fixed, so a
/// process started under something that had already lowered its own slack
/// starts lower. The kernel is then free to delay any `nanosleep`,
/// `clock_nanosleep`, `futex` timeout or poll wake by up to that much, so it can
/// batch wakeups and save power. For a periodic control loop that is a 50 us
/// error added to every single wake.
///
/// `nanoseconds` of 0 is not "no slack": the kernel reads a non-positive
/// argument as "restore this thread's inherited default" and puts the 50 us
/// back. That is why the callers in this workspace pass 1.
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
    // The prctl argument is an `unsigned long`, which is 32 bits on
    // armv7-unknown-linux-gnueabihf -- a target multi-platform.yml checks on
    // every PR. A bare `as` cast wraps there instead of failing, so a caller
    // asking for 5_000_000_000 ns would silently arm 705 ms and still be told
    // it succeeded. Reject what the target cannot represent.
    let slack = libc::c_ulong::try_from(nanoseconds).map_err(|_| {
        anyhow::anyhow!(
            "timer slack {} ns does not fit this target's unsigned long (max {} ns)",
            nanoseconds,
            libc::c_ulong::MAX
        )
    })?;

    // SAFETY: PR_SET_TIMERSLACK takes a single unsigned long argument and
    // affects only the calling thread.
    let result = unsafe { libc::prctl(libc::PR_SET_TIMERSLACK, slack) };
    if result == 0 {
        Ok(())
    } else {
        Err(std::io::Error::last_os_error())
            .map_err(|e| anyhow::anyhow!("prctl(PR_SET_TIMERSLACK, {}) failed: {}", nanoseconds, e))
    }
}

/// This thread's current timer slack in nanoseconds, if it can be read.
///
/// Asks `PR_GET_TIMERSLACK` rather than reading `/proc/self/timerslack_ns`,
/// because slack is per-thread and that file is not:
///
/// - `/proc/self` is the thread-group leader's directory, so every thread but
///   the leader would be reporting some other task's slack;
/// - the kernel only lets a task read another task's `timerslack_ns` with
///   CAP_SYS_NICE, so in practice the read does not even return the wrong
///   number off the leader -- it fails with EPERM and this returns `None` for a
///   thread whose slack is perfectly well set;
/// - `timerslack_ns` exists only in the tgid directory, so there is no
///   `/proc/self/task/<tid>/timerslack_ns` to read instead.
///
/// prctl returns the value through a C `int`, so a slack above `i32::MAX`
/// (2.1 s, far past anything a timed wait would want) cannot come back through
/// it and yields `None`.
pub(super) fn timer_slack_ns() -> Option<u64> {
    // SAFETY: PR_GET_TIMERSLACK takes no further arguments and reports the
    // calling thread's slack as the return value.
    let result = unsafe { libc::prctl(libc::PR_GET_TIMERSLACK) };
    if result < 0 {
        None
    } else {
        Some(result as u64)
    }
}

// ── The reset-on-fork flag, and the masking it forces on every readback ──
//
// A thread's scheduling policy is inherited. `copy_process()` duplicates
// `policy` and `rt_priority` into every `clone(2)` child, and glibc's
// `pthread_create` defaults to `PTHREAD_INHERIT_SCHED`, so nothing at the libc
// layer undoes it either. RT setup runs before the lifecycle hooks, on the
// thread that builds the scheduler — so the net replicator, the telemetry HTTP
// thread, the log drain and every per-goal action thread came up at real-time
// priority without asking for it.
//
// `SCHED_RESET_ON_FORK` is the kernel's opt-out: a child of a thread carrying
// it starts at SCHED_OTHER, nice 0. It is a backstop, not the fix — it does
// nothing for CPU affinity, which is inherited through a separate channel with
// no equivalent flag (measured: a `SCHED_BATCH|SCHED_RESET_ON_FORK` parent
// pinned to one CPU produces a child with `policy=SCHED_OTHER` and a
// one-CPU mask). The real guarantee is the explicit demotion in
// `set_best_effort_class`, which resets both.

/// `SCHED_RESET_ON_FORK` from the kernel's `include/uapi/linux/sched.h`.
///
/// Spelled out rather than reached for through `libc::`: in libc 0.2.189 the
/// constant exists only under `linux_like/android` and `linux_l4re_shared`,
/// neither of which is compiled for `*-linux-gnu` or `*-linux-musl`.
pub(super) const SCHED_RESET_ON_FORK: libc::c_int = 0x4000_0000;

/// Strip the reset-on-fork bit from a `sched_getscheduler` return value.
///
/// The kernel ORs the flag INTO the readback — a thread set to
/// `SCHED_FIFO|SCHED_RESET_ON_FORK` reads back `0x40000001`, not `1`. Every
/// `== SCHED_FIFO`-shaped comparison in this workspace therefore has to go
/// through here, or setting the flag silently stops that comparison
/// recognising the very threads it exists to recognise: `can_set_rt_priority`
/// would stop short-circuiting on an already-RT thread and start probing it,
/// `has_deadline_capability` would re-probe a live SCHED_DEADLINE reservation,
/// and `RtConfig::get_current_scheduler` would report `Normal` for a
/// SCHED_FIFO thread.
pub(super) fn policy_of(raw: libc::c_int) -> libc::c_int {
    raw & !SCHED_RESET_ON_FORK
}

/// This thread's scheduling policy with the reset-on-fork bit removed.
pub(super) fn current_policy() -> libc::c_int {
    // SAFETY: pid 0 = current thread; `sched_getscheduler` only reads.
    policy_of(unsafe { libc::sched_getscheduler(0) })
}

/// Set SCHED_FIFO priority for the current thread.
pub(super) fn set_realtime_priority(priority: i32) -> anyhow::Result<()> {
    // SAFETY: pid 0 = current thread; sched_param is properly initialized
    let param = sched_param_with_priority(priority);
    // `|SCHED_RESET_ON_FORK` so a thread spawned from this one does not silently
    // start at real-time priority. Kernels before 2.6.32, and some seccomp
    // filters, reject the unknown policy bit with EINVAL — fall back rather
    // than lose the RT policy entirely, since the explicit demotion in
    // `set_best_effort_class` is the real guarantee and this is the backstop.
    let mut result =
        unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO | SCHED_RESET_ON_FORK, &param) };
    if result != 0 && std::io::Error::last_os_error().raw_os_error() == Some(libc::EINVAL) {
        result = unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) };
    }

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
        // Same reasoning as the `SCHED_RESET_ON_FORK` on the SCHED_FIFO path: a
        // child of a SCHED_DEADLINE thread would otherwise inherit the
        // reservation. `SCHED_FLAG_RESET_ON_FORK` is the `sched_attr` spelling
        // of the same request, and libc does export this one.
        sched_flags: libc::SCHED_FLAG_RESET_ON_FORK as u64,
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
    // Deliberately NOT run through `policy_of`: this value is a restore target,
    // and `restore_scheduling` must put the reset-on-fork bit back exactly as
    // it found it. Masking here would quietly strip the flag off every thread a
    // capability probe touched.
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
    if current_policy() == SCHED_DEADLINE_POLICY as i32 {
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
    let current_policy = current_policy();
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

/// Put the calling thread on the ordinary time-shared class and give it back
/// the helper CPU set.
///
/// Both halves are conditional on the thread actually being somewhere else, so
/// the common case — a helper spawned in a process where no RT setup ever ran —
/// issues ZERO syscalls beyond the two reads. That is what makes the
/// syscall-count test meaningful.
pub(super) fn set_best_effort_class(nice_increment: i32, target: &[usize]) -> BestEffortReport {
    let mut report = BestEffortReport {
        // SAFETY: pid 0 = current thread; `sched_getscheduler` only reads.
        prior_policy: policy_of(unsafe { libc::sched_getscheduler(0) }),
        ..BestEffortReport::default()
    };

    if report.prior_policy != libc::SCHED_OTHER {
        // SAFETY: `sched_param` is a POD struct of integer fields for which
        // all-zero is a valid bit pattern. It is zeroed rather than
        // literal-constructed because musl carries four extra sporadic-server
        // members that glibc does not (the same portability trap documented on
        // `sched_param_with_priority`); SCHED_OTHER ignores every member but
        // `sched_priority`, which must be 0.
        let rc = unsafe {
            let mut param: libc::sched_param = std::mem::zeroed();
            param.sched_priority = 0;
            libc::sched_setscheduler(0, libc::SCHED_OTHER, &param)
        };
        if rc == 0 {
            report.policy_changed = true;
        } else {
            report.refusal = std::io::Error::last_os_error().raw_os_error();
        }
    }

    if nice_increment > 0 {
        // SAFETY: `nice` adjusts only the calling thread's CFS weight (the nice
        // value is a per-thread attribute on Linux/NPTL). Raising it never
        // requires privilege. The return value is deliberately ignored: it
        // cannot be distinguished from a legitimate -1 result without errno
        // handling, and the failure mode — the helper keeps its inherited
        // weight — is not worth a diagnostic on a path that cannot fail.
        let _ = unsafe { libc::nice(nice_increment) };
    }

    // The affinity half. There is no kernel flag for this: `cpus_allowed` is
    // copied by `dup_task_struct` and `SCHED_RESET_ON_FORK` explicitly does not
    // reset it, so a helper spawned after `apply_rt_optimizations` stays pinned
    // to the reserved RT core no matter what its scheduling class is.
    if !target.is_empty() {
        match current_affinity() {
            Ok(now) if now == target => {}
            Ok(_) => match set_affinity(target) {
                Ok(_) => report.affinity_changed = true,
                Err(_) => {
                    report.refusal = report
                        .refusal
                        .or(std::io::Error::last_os_error().raw_os_error());
                }
            },
            Err(_) => {
                report.refusal = report
                    .refusal
                    .or(std::io::Error::last_os_error().raw_os_error());
            }
        }
    }

    report
}

// ── CPU affinity, addressed by kernel CPU id ────────────────────────────
//
// The distinction this section exists to enforce: a CPU *id* is what the kernel
// calls a processor, and it is what `isolcpus=6,7`, `taskset -c 6`, `/proc/stat`
// and every operator instruction mean. A CPU *index* is a position in some
// vector this process happens to hold. They coincide on an untuned machine and
// diverge on exactly the tuned host an RT deployment runs on: with
// `isolcpus=6,7` the process inherits an affinity mask of {0..5}, so a list of
// "the CPUs I may run on" has six entries and position 6 does not exist. Code
// that pins "to core 6" by indexing that list therefore silently declines to
// pin at all, and the isolated cores the operator rebooted for are never used.
//
// `sched_setaffinity` takes ids, and — this is the part that makes the whole
// approach work — it SUCCEEDS on an isolated CPU. `isolcpus` changes the mask
// init hands out; it is not a permission. So a request for CPU 6 on an
// `isolcpus=6,7` host is granted, while the same request expressed as an index
// into the inherited mask is refused.

/// The CPU ids the kernel has enumerated, from `/sys/devices/system/cpu/present`.
///
/// `present` rather than `online` or the inherited affinity mask: it is the set
/// that validates a *request*. An offline CPU can come back and an isolated CPU
/// is deliberately absent from the inherited mask, so both are legitimate pin
/// targets; a CPU that is not present never will be, and a request naming one
/// is a configuration error worth reporting.
///
/// Empty when the file cannot be read (a container with a trimmed `/sys`, a
/// non-standard kernel). Callers treat empty as "cannot validate" and let the
/// kernel arbitrate rather than refusing the pin.
pub(super) fn present_cpus() -> Vec<usize> {
    match std::fs::read_to_string("/sys/devices/system/cpu/present") {
        Ok(content) => super::parse_cpu_list(content.trim()),
        Err(_) => Vec::new(),
    }
}

/// The CPUs the calling thread may currently run on, as kernel CPU ids.
///
/// This is the read-back that makes a pin verifiable. `sched_setaffinity`
/// intersects the requested mask with what a cpuset cgroup allows and reports
/// success on a partial application, so "the call returned 0" and "the thread
/// is on the CPUs I asked for" are different statements.
pub(super) fn current_affinity() -> anyhow::Result<Vec<usize>> {
    // SAFETY: `set` is a stack-allocated `cpu_set_t` for which all-zero is a
    // valid bit pattern, and `sched_getaffinity` only writes into it. `pid == 0`
    // addresses the calling thread.
    let mut set: libc::cpu_set_t = unsafe { std::mem::zeroed() };
    let rc =
        unsafe { libc::sched_getaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &mut set) };
    if rc != 0 {
        anyhow::bail!(
            "sched_getaffinity failed: {}",
            std::io::Error::last_os_error()
        );
    }

    let mut cpus = Vec::new();
    for cpu in 0..libc::CPU_SETSIZE as usize {
        // SAFETY: `cpu` is below `CPU_SETSIZE`, which is the bound the macro
        // indexes within, and `set` was filled by the call above.
        if unsafe { libc::CPU_ISSET(cpu, &set) } {
            cpus.push(cpu);
        }
    }
    Ok(cpus)
}

/// Pin the calling thread to exactly `cpus`, addressed by kernel CPU id.
///
/// Returns the mask the kernel actually installed, read back with
/// [`current_affinity`]. The return value is the point: a caller that reports
/// "pinned to {2,3}" because the call returned `Ok` is reporting its request,
/// not the outcome, and that is how a thread ends up running everywhere while
/// the log says it is isolated.
///
/// Errors when the request is empty, names a CPU outside `cpu_set_t`, names a
/// CPU that is not present, or when the kernel refuses the call outright.
pub(super) fn set_affinity(cpus: &[usize]) -> anyhow::Result<Vec<usize>> {
    if cpus.is_empty() {
        anyhow::bail!("refusing to install an empty CPU affinity mask");
    }

    let setsize = libc::CPU_SETSIZE as usize;
    if let Some(&too_big) = cpus.iter().find(|&&c| c >= setsize) {
        anyhow::bail!(
            "CPU {} does not fit in a cpu_set_t (CPU_SETSIZE = {})",
            too_big,
            setsize
        );
    }

    // Validate against `present` when it is readable. Skipping the check when
    // the file is missing is deliberate: refusing a pin because `/sys` was not
    // mounted would break containers that can pin perfectly well.
    let present = present_cpus();
    if !present.is_empty() {
        let missing: Vec<usize> = cpus
            .iter()
            .copied()
            .filter(|c| !present.contains(c))
            .collect();
        if !missing.is_empty() {
            anyhow::bail!(
                "CPU(s) {:?} are not present on this machine (present: {:?})",
                missing,
                present
            );
        }
    }

    // SAFETY: `set` is a stack-allocated `cpu_set_t` for which all-zero is a
    // valid bit pattern. Every `cpu` was bounds-checked against `CPU_SETSIZE`
    // above, which is the range `CPU_SET` indexes within. `pid == 0` addresses
    // the calling thread and `sched_setaffinity` only reads through the pointer.
    let rc = unsafe {
        let mut set: libc::cpu_set_t = std::mem::zeroed();
        libc::CPU_ZERO(&mut set);
        for &cpu in cpus {
            libc::CPU_SET(cpu, &mut set);
        }
        libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set)
    };

    if rc != 0 {
        let err = std::io::Error::last_os_error();
        match err.raw_os_error() {
            Some(libc::EPERM) => anyhow::bail!(
                "sched_setaffinity({:?}) refused: {} — the process lacks permission to \
                 move this thread (a cpuset cgroup or a seccomp filter, not isolcpus: \
                 isolcpus changes the inherited mask, it is not a permission)",
                cpus,
                err
            ),
            Some(libc::EINVAL) => anyhow::bail!(
                "sched_setaffinity({:?}) rejected the mask: {} — none of the requested \
                 CPUs is available to this process (check the cpuset cgroup)",
                cpus,
                err
            ),
            _ => anyhow::bail!("sched_setaffinity({:?}) failed: {}", cpus, err),
        }
    }

    current_affinity()
}

// ── CPU idle-state exit latency ─────────────────────────────────────────
//
// The absolute-sleep tick loop deliberately gives the core back for most of
// every period — which is exactly the condition under which the cpuidle
// governor promotes it into a deep C-state. The `menu` governor admits a state
// when predicted idle time exceeds that state's target residency, so the SLOWER
// the loop the DEEPER the state it licenses, and a 100 Hz mobile base is worse
// off than a 4 kHz drone.
//
// On the reference box (`intel_idle`, `menu`): C1 exit 1 us, C2 exit 151 us
// with a 453 us residency target, C3 exit 1034 us with 3102 us. Against a guard
// spin of 20 us, that is 7.5x at 1 kHz and 51x at 100 Hz — and the guard cannot
// prevent it, because the guard runs AFTER the wake the exit has already
// delayed.
//
// Setting the cpufreq governor to `performance`, which this crate does do, is a
// DIFFERENT SUBSYSTEM. It addresses frequency scaling and has no effect
// whatsoever on idle states. The two are routinely conflated, including in this
// project's own setup script until now.

pub(super) const CPU_DMA_LATENCY_DEV: &str = "/dev/cpu_dma_latency";
pub(super) const CPU_SYSFS_ROOT: &str = "/sys/devices/system/cpu";

/// The four bytes for a `/dev/cpu_dma_latency` write.
///
/// NATIVE endian: the kernel does a raw `copy_from_user` of an `s32`, so this
/// follows the target rather than a wire format. Exactly four bytes — a short
/// write is a failure, never a partial constraint.
pub(super) fn cpu_dma_latency_bytes(budget_us: u32) -> [u8; 4] {
    (budget_us as i32).to_ne_bytes()
}

/// The string for a per-CPU `pm_qos_resume_latency_us` write.
///
/// Refuses 0. The two interfaces disagree about that value: 0 written to
/// `/dev/cpu_dma_latency` means "tolerate no exit latency at all", pinning
/// every core in POLL; 0 in `pm_qos_resume_latency_us` means "no constraint" —
/// the exact opposite. Passing a zero budget to the per-CPU path would silently
/// REMOVE the bound and report success.
pub(super) fn resume_latency_value(budget_us: u32) -> anyhow::Result<String> {
    if budget_us == 0 {
        anyhow::bail!(
            "a 0 us budget means `no constraint` to pm_qos_resume_latency_us and \
             `tolerate nothing` to /dev/cpu_dma_latency; refusing rather than silently \
             removing the bound"
        );
    }
    Ok(budget_us.to_string())
}

/// Open `dev_path`, write the budget, and return the STILL-OPEN file.
///
/// The constraint lives exactly as long as the returned handle — the kernel
/// releases it the instant the fd closes, including on SIGKILL. That is why the
/// handle is returned rather than dropped, and why this needs no cleanup path.
pub(super) fn bound_idle_latency_global_at(
    dev_path: &std::path::Path,
    budget_us: u32,
) -> std::io::Result<std::fs::File> {
    use std::io::Write;
    let mut f = std::fs::OpenOptions::new().write(true).open(dev_path)?;
    let bytes = cpu_dma_latency_bytes(budget_us);
    f.write_all(&bytes)?;
    f.flush()?;
    Ok(f)
}

/// A per-CPU `pm_qos_resume_latency_us` hold, with the previous values kept so
/// `Drop` restores them verbatim.
///
/// Unlike the global device, this is sysfs state: it does NOT go away when the
/// process dies, so a hard kill leaks it. That is the reason `Global` is the
/// default and this is opt-in.
#[derive(Debug)]
pub struct PerCpuIdleHold {
    cpu_root: std::path::PathBuf,
    /// `(cpu, previous file contents)`.
    previous: Vec<(usize, String)>,
}

fn resume_latency_path(cpu_root: &std::path::Path, cpu: usize) -> std::path::PathBuf {
    cpu_root
        .join(format!("cpu{cpu}"))
        .join("power")
        .join("pm_qos_resume_latency_us")
}

/// Write the budget into each CPU's `pm_qos_resume_latency_us`, reading back to
/// confirm sysfs took the value.
pub(super) fn bound_idle_latency_per_cpu_at(
    cpu_root: &std::path::Path,
    cpus: &[usize],
    budget_us: u32,
) -> std::io::Result<PerCpuIdleHold> {
    let value = resume_latency_value(budget_us)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidInput, e.to_string()))?;
    let mut hold = PerCpuIdleHold {
        cpu_root: cpu_root.to_path_buf(),
        previous: Vec::with_capacity(cpus.len()),
    };
    for &cpu in cpus {
        let path = resume_latency_path(cpu_root, cpu);
        let prev = std::fs::read_to_string(&path)?;
        std::fs::write(&path, &value)?;
        // Read back: sysfs can accept a write and clamp it, and a constraint
        // that was silently clamped is a constraint the operator does not have.
        let now = std::fs::read_to_string(&path)?;
        if now.trim() != value {
            // Undo what we did before reporting, so a partial application does
            // not outlive the failure.
            let _ = std::fs::write(&path, &prev);
            for (c, p) in hold.previous.drain(..) {
                let _ = std::fs::write(resume_latency_path(cpu_root, c), p);
            }
            return Err(std::io::Error::other(format!(
                "wrote {value} to {} but it reads back {}",
                path.display(),
                now.trim()
            )));
        }
        hold.previous.push((cpu, prev));
    }
    Ok(hold)
}

impl Drop for PerCpuIdleHold {
    fn drop(&mut self) {
        for (cpu, prev) in self.previous.drain(..) {
            let _ = std::fs::write(resume_latency_path(&self.cpu_root, cpu), prev);
        }
    }
}

/// The deepest ENABLED idle state under `cpu_root/cpu{n}/cpuidle`, as
/// `(name, exit latency in microseconds)`.
///
/// "Deepest" is the largest exit latency, which is also the worst case a wake
/// can pay. States whose `disable` file reads non-zero are skipped: the
/// governor cannot enter them, so counting them would overstate the exposure.
pub(super) fn deepest_idle_exit_latency_at(
    cpu_root: &std::path::Path,
    cpu: usize,
) -> Option<(String, u32)> {
    let dir = cpu_root.join(format!("cpu{cpu}")).join("cpuidle");
    let mut best: Option<(String, u32)> = None;
    for entry in std::fs::read_dir(dir).ok()? {
        let Ok(entry) = entry else { continue };
        let path = entry.path();
        if std::fs::read_to_string(path.join("disable"))
            .map(|s| s.trim() != "0")
            .unwrap_or(false)
        {
            continue;
        }
        let Some(latency) = std::fs::read_to_string(path.join("latency"))
            .ok()
            .and_then(|s| s.trim().parse::<u32>().ok())
        else {
            continue;
        };
        let name = std::fs::read_to_string(path.join("name"))
            .map(|s| s.trim().to_string())
            .unwrap_or_else(|_| "unknown".to_string());
        if best.as_ref().is_none_or(|(_, l)| latency > *l) {
            best = Some((name, latency));
        }
    }
    best
}

/// What is holding an idle-latency bound open.
///
/// Neither payload is ever read, and that is the point: the VALUE is the
/// mechanism. An open `/dev/cpu_dma_latency` fd holds the constraint for as
/// long as it is open, and `PerCpuIdleHold` restores the previous sysfs values
/// in its `Drop`. Dropping this enum is what releases the bound.
#[derive(Debug)]
#[allow(dead_code, reason = "held for its lifetime and its Drop, never read")]
pub(super) enum IdleHold {
    /// The open `/dev/cpu_dma_latency` fd. Closing it releases the constraint.
    Global(std::fs::File),
    /// Per-CPU sysfs writes, restored on drop.
    PerCpu(PerCpuIdleHold),
}

// ── RT bandwidth control ────────────────────────────────────────────────

fn read_i64(path: &str) -> Option<i64> {
    std::fs::read_to_string(path).ok()?.trim().parse().ok()
}

/// This task's cgroup cpu-controller path, from `/proc/self/cgroup`.
///
/// Split out from the file read so both formats are testable on a string: a
/// unified v2 line `0::/user.slice/...`, and a v1 line
/// `4:cpu,cpuacct:/docker/<id>`.
pub(crate) fn cgroup_cpu_path(proc_self_cgroup: &str) -> Option<String> {
    let mut unified: Option<String> = None;
    for line in proc_self_cgroup.lines() {
        let mut parts = line.splitn(3, ':');
        let (_hier, controllers, path) = (parts.next()?, parts.next()?, parts.next()?);
        if controllers.split(',').any(|c| c == "cpu") {
            return Some(path.to_string());
        }
        if controllers.is_empty() {
            unified = Some(path.to_string());
        }
    }
    unified
}

/// `cpu.rt_runtime_us` / `cpu.rt_period_us` for this task's cgroup.
///
/// Present only where `CONFIG_RT_GROUP_SCHED` is compiled in, which most
/// distributions do not do — the files being absent is the NORMAL case, not an
/// error.
fn read_cgroup_rt_budget() -> Option<RtBandwidth> {
    let path = std::fs::read_to_string("/proc/self/cgroup")
        .ok()
        .and_then(|s| cgroup_cpu_path(&s))?;
    for root in ["/sys/fs/cgroup/cpu", "/sys/fs/cgroup/cpu,cpuacct"] {
        let base = format!("{root}{path}");
        if let (Some(r), Some(p)) = (
            read_i64(&format!("{base}/cpu.rt_runtime_us")),
            read_i64(&format!("{base}/cpu.rt_period_us")),
        ) {
            return Some(RtBandwidth::from_raw(r, p, RtBandwidthSource::CgroupV1));
        }
    }
    None
}

/// The RT class budget that actually binds this task.
pub(super) fn read_rt_bandwidth() -> RtBandwidth {
    let global = match (
        read_i64("/proc/sys/kernel/sched_rt_runtime_us"),
        read_i64("/proc/sys/kernel/sched_rt_period_us"),
    ) {
        (Some(r), Some(p)) => RtBandwidth::from_raw(r, p, RtBandwidthSource::ProcSys),
        _ => return RtBandwidth::UNAVAILABLE,
    };

    // A cgroup budget, where one exists, is the tighter of the two and is what
    // the kernel will enforce. `cpu.rt_runtime_us == 0` is the default for a
    // non-root group and is why `sched_setscheduler(SCHED_FIFO)` fails with
    // EPERM inside an ordinary container.
    match read_cgroup_rt_budget() {
        Some(cg) if tighter_than(cg, global) => cg,
        _ => global,
    }
}

/// Whether `a` permits strictly less RT execution than `b`.
fn tighter_than(a: RtBandwidth, b: RtBandwidth) -> bool {
    if !a.is_known() {
        return false;
    }
    if !b.is_known() || b.is_unlimited() {
        return !a.is_unlimited();
    }
    match (a.duty_fraction(), b.duty_fraction()) {
        (Some(x), Some(y)) => x < y,
        _ => false,
    }
}

// ── Preemption model: a ladder, with the rung recorded ──────────────────

const DEBUGFS_PREEMPT: &str = "/sys/kernel/debug/sched/preempt";
const REALTIME_SYSFS: &str = "/sys/kernel/realtime";
const CLOCKSOURCE_PATH: &str = "/sys/devices/system/clocksource/clocksource0/current_clocksource";

/// Parse `/sys/kernel/debug/sched/preempt`.
///
/// The kernel prints every mode with the ACTIVE one in parentheses:
/// `none voluntary (full) lazy`. Returns `None` when nothing is parenthesised —
/// a format change must read as "unknown", never as a guess.
pub(crate) fn parse_debugfs_preempt(s: &str) -> Option<PreemptModel> {
    s.split_whitespace()
        .find_map(|tok| tok.strip_prefix('(')?.strip_suffix(')'))
        .and_then(model_from_mode_name)
}

/// Map a kernel mode name to a model.
fn model_from_mode_name(name: &str) -> Option<PreemptModel> {
    match name {
        "none" => Some(PreemptModel::None),
        "voluntary" => Some(PreemptModel::Voluntary),
        "full" => Some(PreemptModel::Full),
        "lazy" => Some(PreemptModel::Lazy),
        _ => None,
    }
}

/// Parse a `preempt=` boot override out of `/proc/cmdline`.
///
/// Whole-token match on whitespace-split fields, so `nopreempt=full` cannot
/// match.
pub(crate) fn parse_cmdline_preempt(cmdline: &str) -> Option<PreemptModel> {
    cmdline
        .split_whitespace()
        .find_map(|tok| tok.strip_prefix("preempt="))
        .and_then(model_from_mode_name)
}

/// Map `CONFIG_PREEMPT_*` to the model the kernel boots with.
///
/// Follows the kernel's own `preempt_dynamic_init()` if/else chain exactly:
/// RT > NONE > VOLUNTARY > LAZY > PREEMPT(full).
///
/// Matches whole `CONFIG_X=y` lines only. Both traps this avoids are live on an
/// ordinary Ubuntu kernel: `# CONFIG_PREEMPT is not set` must not satisfy the
/// `CONFIG_PREEMPT` arm, and `CONFIG_PREEMPT_BUILD=y` must not masquerade as
/// `CONFIG_PREEMPT=y`.
pub(crate) fn model_from_kernel_config(config: &str) -> Option<PreemptModel> {
    let has = |key: &str| config.lines().any(|l| l.trim() == format!("{key}=y"));
    if has("CONFIG_PREEMPT_RT") {
        Some(PreemptModel::PreemptRt)
    } else if has("CONFIG_PREEMPT_NONE") {
        Some(PreemptModel::None)
    } else if has("CONFIG_PREEMPT_VOLUNTARY") {
        Some(PreemptModel::Voluntary)
    } else if has("CONFIG_PREEMPT_LAZY") {
        Some(PreemptModel::Lazy)
    } else if has("CONFIG_PREEMPT") {
        Some(PreemptModel::Full)
    } else {
        None
    }
}

/// Whether the kernel config says the model is runtime-settable.
pub(crate) fn config_is_dynamic(config: &str) -> bool {
    config
        .lines()
        .any(|l| l.trim() == "CONFIG_PREEMPT_DYNAMIC=y")
}

/// The `PREEMPT*` token in `/proc/version`, for a kernel with no other source.
///
/// Returns `None` for a bare `PREEMPT_DYNAMIC` — that token announces
/// runtime-settability, not a mode — and `None` for no token at all, because
/// `none` and `voluntary` both emit nothing and are indistinguishable here.
/// Guessing between them is exactly the one-bucket error this ladder exists to
/// remove.
pub(crate) fn model_from_version_token(v: &str) -> Option<PreemptModel> {
    if version_says_rt(v) {
        return Some(PreemptModel::PreemptRt);
    }
    let has = |t: &str| v.split_whitespace().any(|tok| tok == t);
    if has("PREEMPT_LAZY") {
        Some(PreemptModel::Lazy)
    } else if has("PREEMPT") {
        Some(PreemptModel::Full)
    } else {
        None
    }
}

pub(crate) fn version_says_dynamic(v: &str) -> bool {
    v.split_whitespace().any(|tok| tok == "PREEMPT_DYNAMIC")
}

pub(crate) fn version_says_rt(v: &str) -> bool {
    v.contains("PREEMPT_RT") || v.contains("PREEMPT RT")
}

/// `(model, dynamic)` from `/boot/config-<osrelease>`, read at most once.
///
/// `/proc/config.gz` is deliberately NOT read: it needs gzip, and adding a
/// decompressor to a crate every HORUS binary links, for a diagnostic present
/// on a minority of distros, is the wrong trade.
fn kernel_config_facts() -> &'static Option<(Option<PreemptModel>, bool)> {
    static FACTS: std::sync::OnceLock<Option<(Option<PreemptModel>, bool)>> =
        std::sync::OnceLock::new();
    FACTS.get_or_init(|| {
        let release = std::fs::read_to_string("/proc/sys/kernel/osrelease").ok()?;
        let config = std::fs::read_to_string(format!("/boot/config-{}", release.trim())).ok()?;
        Some((
            model_from_kernel_config(&config),
            config_is_dynamic(&config),
        ))
    })
}

/// Establish the preemption model, recording which rung answered.
pub(super) fn read_preempt_info() -> PreemptInfo {
    let version = get_kernel_version();
    let config = kernel_config_facts();
    let dynamic = config.map(|(_, d)| d).unwrap_or(false) || version_says_dynamic(&version);

    // 1. The running value. Root-only in practice, and the only rung that
    //    survives a runtime `echo full > .../preempt`.
    if let Some(model) = std::fs::read_to_string(DEBUGFS_PREEMPT)
        .ok()
        .and_then(|s| parse_debugfs_preempt(&s))
    {
        return PreemptInfo {
            model,
            dynamic,
            source: PreemptSource::Debugfs,
        };
    }

    // 2. PREEMPT_RT is not dynamic — a kernel either has sleeping spinlocks or
    //    it does not — so these two rungs are conclusive wherever they fire.
    if version_says_rt(&version) || std::path::Path::new(REALTIME_SYSFS).exists() {
        return PreemptInfo {
            model: PreemptModel::PreemptRt,
            dynamic: false,
            source: if version_says_rt(&version) {
                PreemptSource::ProcVersion
            } else {
                PreemptSource::RealtimeSysfs
            },
        };
    }

    // 3. The boot override, if the operator set one.
    if let Some(model) = std::fs::read_to_string("/proc/cmdline")
        .ok()
        .and_then(|s| parse_cmdline_preempt(&s))
    {
        return PreemptInfo {
            model,
            dynamic,
            source: PreemptSource::Cmdline,
        };
    }

    // 4. The build default.
    if let Some((Some(model), _)) = config {
        return PreemptInfo {
            model: *model,
            dynamic,
            source: PreemptSource::KernelConfig,
        };
    }

    // 5. The token, which can only distinguish full and lazy from each other.
    if let Some(model) = model_from_version_token(&version) {
        return PreemptInfo {
            model,
            dynamic,
            source: PreemptSource::ProcVersion,
        };
    }

    PreemptInfo {
        model: PreemptModel::Unknown,
        dynamic,
        source: PreemptSource::Unavailable,
    }
}

/// The clocksource currently serving `clock_gettime`.
pub(super) fn read_clocksource() -> Clocksource {
    match std::fs::read_to_string(CLOCKSOURCE_PATH) {
        Ok(s) => Clocksource::from_name(s.trim()),
        Err(_) => Clocksource::unknown(),
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
/// normal case, not an edge case. The mask arithmetic itself lives in
/// [`irq_affinity_mask`], where it is unit-tested on synthetic CPU sets.
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

    // If this process is confined, it is not entitled to reroute the host.
    let visible = std::thread::available_parallelism()
        .map(|p| p.get())
        .unwrap_or(1);
    if visible < online.len() {
        anyhow::bail!(
            "this process sees {visible} of the machine's {} CPUs (cpuset or \
             taskset), so it cannot compute a correct host-wide IRQ mask; \
             refusing rather than rerouting interrupts for cores it cannot see",
            online.len()
        );
    }

    let mask = irq_affinity_mask(&online, cpus)?;
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
mod tests {
    // ── Idle-state exit latency ─────────────────────────────────────────

    /// The two kernel interfaces disagree about what 0 means, and the
    /// disagreement is a silent-wrong-answer trap.
    #[test]
    fn a_zero_budget_is_refused_by_the_per_cpu_path_and_encoded_by_the_global_one() {
        // /dev/cpu_dma_latency: 0 means "tolerate no exit latency at all".
        assert_eq!(cpu_dma_latency_bytes(0), 0i32.to_ne_bytes());
        assert_eq!(cpu_dma_latency_bytes(20), 20i32.to_ne_bytes());
        assert_eq!(cpu_dma_latency_bytes(1_034), 1_034i32.to_ne_bytes());

        // pm_qos_resume_latency_us: 0 means "no constraint" — the opposite.
        // Writing it would silently REMOVE the bound and report success.
        assert_eq!(resume_latency_value(20).unwrap(), "20");
        let err = resume_latency_value(0).unwrap_err().to_string();
        assert!(
            err.contains("no constraint"),
            "the refusal must say why the two interfaces cannot share a 0: {err}"
        );
    }

    /// The per-CPU path is sysfs state: it must restore what it found, and it
    /// must not leave a partial application behind on failure.
    #[test]
    fn the_per_cpu_bound_restores_the_previous_values_on_drop() {
        let tmp = std::env::temp_dir().join(format!(
            "horus_idle_test_{}_{}",
            std::process::id(),
            line!()
        ));
        let cpus = [2usize, 3];
        for cpu in cpus {
            let dir = tmp.join(format!("cpu{cpu}")).join("power");
            std::fs::create_dir_all(&dir).unwrap();
            std::fs::write(dir.join("pm_qos_resume_latency_us"), "0\n").unwrap();
        }

        {
            let _hold = bound_idle_latency_per_cpu_at(&tmp, &cpus, 20).expect("bound");
            for cpu in cpus {
                let v = std::fs::read_to_string(
                    tmp.join(format!("cpu{cpu}"))
                        .join("power")
                        .join("pm_qos_resume_latency_us"),
                )
                .unwrap();
                assert_eq!(v.trim(), "20", "cpu{cpu} must carry the bound while held");
            }
        }

        for cpu in cpus {
            let v = std::fs::read_to_string(
                tmp.join(format!("cpu{cpu}"))
                    .join("power")
                    .join("pm_qos_resume_latency_us"),
            )
            .unwrap();
            assert_eq!(
                v.trim(),
                "0",
                "cpu{cpu} must be back to what it was — this is sysfs, so a value left \
                 behind outlives the process"
            );
        }
        let _ = std::fs::remove_dir_all(&tmp);
    }

    /// "Deepest" means the largest exit latency, and a state the governor
    /// cannot enter must not be counted — that would overstate the exposure.
    #[test]
    fn the_deepest_idle_state_skips_disabled_states() {
        let tmp = std::env::temp_dir().join(format!(
            "horus_cpuidle_test_{}_{}",
            std::process::id(),
            line!()
        ));
        // The reference box's real states.
        let states = [
            ("state0", "POLL", "0", "0"),
            ("state1", "C1_ACPI", "1", "0"),
            ("state2", "C2_ACPI", "151", "0"),
            ("state3", "C3_ACPI", "1034", "1"), // disabled
        ];
        for (dir, name, latency, disable) in states {
            let d = tmp.join("cpu0").join("cpuidle").join(dir);
            std::fs::create_dir_all(&d).unwrap();
            std::fs::write(d.join("name"), format!("{name}\n")).unwrap();
            std::fs::write(d.join("latency"), format!("{latency}\n")).unwrap();
            std::fs::write(d.join("disable"), format!("{disable}\n")).unwrap();
        }

        assert_eq!(
            deepest_idle_exit_latency_at(&tmp, 0),
            Some(("C2_ACPI".to_string(), 151)),
            "C3 is disabled, so the governor cannot enter it and counting its 1034us \
             exit would overstate what this machine is exposed to"
        );

        // Re-enable it and the answer changes.
        std::fs::write(
            tmp.join("cpu0")
                .join("cpuidle")
                .join("state3")
                .join("disable"),
            "0\n",
        )
        .unwrap();
        assert_eq!(
            deepest_idle_exit_latency_at(&tmp, 0),
            Some(("C3_ACPI".to_string(), 1034))
        );

        // A host with no cpuidle sysfs at all — a VM, or `cpuidle.off=1`.
        assert_eq!(deepest_idle_exit_latency_at(&tmp, 99), None);
        let _ = std::fs::remove_dir_all(&tmp);
    }

    /// The global path holds the constraint by keeping the fd OPEN. A version
    /// that wrote and closed would report success and constrain nothing.
    #[test]
    fn the_global_bound_writes_four_native_endian_bytes() {
        let tmp =
            std::env::temp_dir().join(format!("horus_dma_test_{}_{}", std::process::id(), line!()));
        std::fs::write(&tmp, b"").unwrap();

        let file = bound_idle_latency_global_at(&tmp, 20).expect("write");
        drop(file);
        let written = std::fs::read(&tmp).unwrap();
        assert_eq!(
            written,
            20i32.to_ne_bytes(),
            "the kernel copy_from_user's an s32, so this follows the target's endianness \
             rather than a wire format, and it is exactly four bytes"
        );
        let _ = std::fs::remove_file(&tmp);
    }

    // ── RT bandwidth ────────────────────────────────────────────────────

    #[test]
    fn the_cgroup_cpu_path_is_parsed_in_both_formats() {
        // cgroup v2: one unified line, empty controller field.
        assert_eq!(
            cgroup_cpu_path("0::/user.slice/user-1000.slice/app.slice/x.scope\n"),
            Some("/user.slice/user-1000.slice/app.slice/x.scope".to_string())
        );
        // cgroup v1: the cpu controller may be joined with cpuacct, and the
        // matching line is not the first.
        assert_eq!(
            cgroup_cpu_path("11:devices:/docker/abc\n4:cpu,cpuacct:/docker/abc\n0::/init.scope\n"),
            Some("/docker/abc".to_string()),
            "a v1 cpu line must win over the unified fallback"
        );
        assert_eq!(cgroup_cpu_path(""), None);
        assert_eq!(
            cgroup_cpu_path("11:devices:/docker/abc\n"),
            None,
            "no cpu controller and no unified line means no path, not a guess"
        );
    }

    /// The read must be self-consistent on whatever host runs the suite.
    #[test]
    fn reading_the_rt_budget_on_this_machine_is_self_consistent() {
        let bw = read_rt_bandwidth();
        if bw.is_known() {
            assert!(bw.period_us > 0);
            assert!(bw.runtime_us >= -1);
            // Exactly one of the three states holds.
            let states = [bw.is_unlimited(), bw.is_finite()];
            assert_eq!(
                states.iter().filter(|b| **b).count(),
                1,
                "unlimited and finite are mutually exclusive: {bw:?}"
            );
            if bw.is_finite() {
                assert!(bw.throttle_window().is_some());
                assert!(bw.duty_fraction().is_some());
            }
        }
        assert!(!bw.describe().is_empty());
    }

    /// A garbled or unreadable budget must not be mistaken for "no throttle".
    #[test]
    fn an_inconsistent_budget_is_unknown_rather_than_unlimited() {
        let garbled = RtBandwidth::from_raw(-1, 0, RtBandwidthSource::ProcSys);
        assert!(!garbled.is_known());
        assert!(
            !garbled.is_unlimited(),
            "period_us == 0 is not a documented kernel state; reading it as unlimited \
             would suppress the throttle warning in exactly the case where nothing \
             about the host is understood"
        );
        assert_eq!(garbled.describe(), "unknown");
        assert!(!RtBandwidth::UNAVAILABLE.is_known());
        assert!(!RtBandwidth::UNAVAILABLE.is_unlimited());
    }

    #[test]
    fn a_tighter_cgroup_budget_wins_over_the_host_wide_one() {
        let host = RtBandwidth::from_raw(950_000, 1_000_000, RtBandwidthSource::ProcSys);
        let container = RtBandwidth::from_raw(0, 1_000_000, RtBandwidthSource::CgroupV1);
        assert!(
            tighter_than(container, host),
            "a zero cgroup budget is what actually binds — it is why \
             sched_setscheduler(SCHED_FIFO) fails with EPERM inside a container"
        );
        assert!(!tighter_than(host, container));

        // An unlimited host budget loses to any finite cgroup budget.
        let unlimited = RtBandwidth::from_raw(-1, 1_000_000, RtBandwidthSource::ProcSys);
        assert!(tighter_than(container, unlimited));
        assert!(!tighter_than(unlimited, host));
    }

    // ── Preemption model: the parsers, each rung on its own ─────────────

    #[test]
    fn debugfs_names_the_active_mode_by_its_parentheses() {
        assert_eq!(
            parse_debugfs_preempt("none voluntary (full) lazy"),
            Some(PreemptModel::Full)
        );
        assert_eq!(
            parse_debugfs_preempt("(none) voluntary full lazy"),
            Some(PreemptModel::None)
        );
        assert_eq!(
            parse_debugfs_preempt("none voluntary full (lazy)\n"),
            Some(PreemptModel::Lazy)
        );
        // A format change must read as "unknown", never as a guess: this file
        // is the only authoritative source, so a wrong answer from it is worse
        // than no answer.
        assert_eq!(parse_debugfs_preempt("none voluntary full lazy"), None);
        assert_eq!(parse_debugfs_preempt(""), None);
        assert_eq!(parse_debugfs_preempt("(banana)"), None);
    }

    #[test]
    fn a_boot_override_is_matched_as_a_whole_token() {
        assert_eq!(
            parse_cmdline_preempt("ro quiet preempt=voluntary splash"),
            Some(PreemptModel::Voluntary)
        );
        assert_eq!(parse_cmdline_preempt("ro quiet splash"), None);
        assert_eq!(
            parse_cmdline_preempt("nopreempt=full"),
            None,
            "a substring match would read `nopreempt=full` as a request for full \
             preemption, which is the opposite of what it says"
        );
    }

    /// Both traps here are live on an ordinary Ubuntu kernel config.
    #[test]
    fn kernel_config_is_matched_line_wise_not_by_substring() {
        // The real shape of /boot/config-* on the reference box.
        let ubuntu = "\
CONFIG_PREEMPT_BUILD=y
# CONFIG_PREEMPT is not set
CONFIG_PREEMPT_LAZY=y
# CONFIG_PREEMPT_RT is not set
CONFIG_PREEMPT_DYNAMIC=y
";
        assert_eq!(
            model_from_kernel_config(ubuntu),
            Some(PreemptModel::Lazy),
            "`# CONFIG_PREEMPT is not set` must not satisfy the CONFIG_PREEMPT arm, \
             and CONFIG_PREEMPT_BUILD=y must not masquerade as CONFIG_PREEMPT=y"
        );
        assert!(config_is_dynamic(ubuntu));

        assert_eq!(
            model_from_kernel_config("CONFIG_PREEMPT_RT=y\nCONFIG_PREEMPT=y\n"),
            Some(PreemptModel::PreemptRt),
            "the kernel's own preempt_dynamic_init() chain tries RT first"
        );
        assert_eq!(
            model_from_kernel_config("CONFIG_PREEMPT_NONE=y\nCONFIG_PREEMPT_LAZY=y\n"),
            Some(PreemptModel::None),
            "NONE precedes LAZY in preempt_dynamic_init()"
        );
        assert_eq!(model_from_kernel_config("CONFIG_HZ=250\n"), None);
        assert!(!config_is_dynamic("# CONFIG_PREEMPT_DYNAMIC is not set\n"));
    }

    #[test]
    fn the_proc_version_token_refuses_to_guess() {
        let dynamic = "Linux version 7.0.0-30-generic #30-Ubuntu SMP PREEMPT_DYNAMIC Fri Jul 31";
        assert_eq!(
            model_from_version_token(dynamic),
            None,
            "PREEMPT_DYNAMIC announces runtime-settability, not a mode — reading it \
             as `full` is exactly the one-bucket error this ladder exists to remove"
        );
        assert!(version_says_dynamic(dynamic));

        assert_eq!(
            model_from_version_token("Linux version 6.1.0 #1 SMP PREEMPT Wed"),
            Some(PreemptModel::Full)
        );
        assert_eq!(
            model_from_version_token("Linux version 6.14.0 #1 SMP PREEMPT_LAZY Wed"),
            Some(PreemptModel::Lazy)
        );
        assert_eq!(
            model_from_version_token("Linux version 6.1.0-rt7 #1 SMP PREEMPT_RT Wed"),
            Some(PreemptModel::PreemptRt)
        );
        assert_eq!(
            model_from_version_token("Linux version 6.1.0 #1 SMP Wed"),
            None,
            "`none` and `voluntary` both emit no token and are indistinguishable here"
        );
    }

    /// The ladder must not claim to know a runtime value it never read.
    #[test]
    fn an_inferred_dynamic_model_is_not_authoritative() {
        let inferred = PreemptInfo {
            model: PreemptModel::Lazy,
            dynamic: true,
            source: PreemptSource::KernelConfig,
        };
        assert!(!inferred.is_authoritative());
        assert!(inferred.describe().contains("inferred"));

        let running = PreemptInfo {
            source: PreemptSource::Debugfs,
            ..inferred
        };
        assert!(running.is_authoritative());

        // A non-dynamic kernel cannot have its model changed underneath us, so
        // the build default IS the running value.
        let fixed = PreemptInfo {
            model: PreemptModel::Full,
            dynamic: false,
            source: PreemptSource::KernelConfig,
        };
        assert!(fixed.is_authoritative());
    }

    #[test]
    fn reading_the_preempt_model_on_this_machine_answers_something_defensible() {
        let info = read_preempt_info();
        // No assertion on WHICH model — that is a property of the test host.
        // What must hold is that an unknown model never claims a source, and a
        // known one always names where it came from.
        if info.model == PreemptModel::Unknown {
            assert_eq!(info.source, PreemptSource::Unavailable);
        } else {
            assert_ne!(info.source, PreemptSource::Unavailable);
        }
        assert!(!info.describe().is_empty());
    }

    // ── Clocksource ─────────────────────────────────────────────────────

    #[test]
    fn the_clocksource_table_separates_vdso_reads_from_syscalls() {
        assert_eq!(Clocksource::from_name("tsc").vdso_fast(), Some(true));
        assert_eq!(Clocksource::from_name("kvm-clock").vdso_fast(), Some(true));
        assert_eq!(
            Clocksource::from_name("arch_sys_counter").vdso_fast(),
            Some(true)
        );
        // hpet lost its vDSO page in 4.20; acpi_pm never had one.
        assert_eq!(Clocksource::from_name("hpet").vdso_fast(), Some(false));
        assert_eq!(Clocksource::from_name("acpi_pm").vdso_fast(), Some(false));
        // An unrecognised name has an unknown cost, and saying so beats
        // guessing in either direction.
        assert_eq!(Clocksource::from_name("something_new").vdso_fast(), None);
        assert_eq!(Clocksource::unknown().name(), "unknown");
        assert_eq!(Clocksource::unknown().vdso_fast(), None);
    }

    #[test]
    fn reading_the_clocksource_on_this_machine_does_not_panic() {
        let cs = read_clocksource();
        assert!(!cs.describe().is_empty());
    }

    /// The reset-on-fork bit must not break policy readback.
    ///
    /// The kernel ORs `SCHED_RESET_ON_FORK` INTO the value `sched_getscheduler`
    /// returns. Setting the flag without masking every readback would have
    /// silently stopped `can_set_rt_priority` short-circuiting on an already-RT
    /// thread (sending it to the live probe that used to strip a deployed
    /// thread of its policy), made `has_deadline_capability` re-probe a live
    /// SCHED_DEADLINE reservation, and made `RtConfig::get_current_scheduler`
    /// report `Normal` for a SCHED_FIFO thread.
    ///
    /// SCHED_BATCH stands in for SCHED_FIFO: same inheritance mechanism, and
    /// settable without CAP_SYS_NICE.
    #[test]
    fn reset_on_fork_bit_does_not_break_policy_readback() {
        std::thread::spawn(|| {
            // SAFETY: `sched_param` is a POD struct of integer fields for which
            // all-zero is a valid bit pattern; SCHED_BATCH requires priority 0.
            let rc = unsafe {
                let mut param: libc::sched_param = std::mem::zeroed();
                param.sched_priority = 0;
                libc::sched_setscheduler(0, libc::SCHED_BATCH | SCHED_RESET_ON_FORK, &param)
            };
            assert_eq!(
                rc,
                0,
                "SCHED_BATCH|SCHED_RESET_ON_FORK must be settable unprivileged: {}",
                std::io::Error::last_os_error()
            );

            // SAFETY: pid 0 = current thread; `sched_getscheduler` only reads.
            let raw = unsafe { libc::sched_getscheduler(0) };
            assert_eq!(
                raw,
                libc::SCHED_BATCH | SCHED_RESET_ON_FORK,
                "the kernel reports the flag OR'd into the policy — that is the whole \
                 reason `policy_of` exists"
            );
            assert_eq!(policy_of(raw), libc::SCHED_BATCH);
            assert_eq!(current_policy(), libc::SCHED_BATCH);

            // The snapshot is a restore target and must keep the raw value, or
            // a capability probe would quietly strip the flag off every thread
            // it touched.
            let snapshot = snapshot_scheduling().expect("snapshot");
            assert_eq!(snapshot.policy, raw, "snapshot_scheduling must not mask");
            restore_scheduling(&snapshot);
            // SAFETY: as above.
            assert_eq!(
                unsafe { libc::sched_getscheduler(0) },
                raw,
                "restore must put the reset-on-fork bit back"
            );
        })
        .join()
        .expect("the probe thread must not panic");
    }

    use super::*;

    /// The property the whole pair exists for. `timer_slack_ns` used to read
    /// `/proc/self/timerslack_ns`, which is the thread-group leader's file: the
    /// spawned thread's read returned EPERM (`None`) instead of its own 7777,
    /// because the kernel guards another task's `timerslack_ns` behind
    /// CAP_SYS_NICE. The RT thread that `set_timer_slack` is called on is never
    /// the leader, so that is the only thread anyone would ask about.
    #[test]
    fn timer_slack_is_per_thread() {
        let original = timer_slack_ns().expect("read this thread's slack");
        set_timer_slack(4321).expect("set slack on this thread");

        let spawned = std::thread::spawn(|| {
            set_timer_slack(7777).expect("set slack on the spawned thread");
            timer_slack_ns()
        })
        .join()
        .expect("spawned thread panicked");

        assert_eq!(spawned, Some(7777), "spawned thread reported another task");
        assert_eq!(
            timer_slack_ns(),
            Some(4321),
            "this thread's slack changed when another thread set its own"
        );

        // Under `--test-threads=1` (how CI runs these) this is the harness
        // thread every other test also runs on, so hand it back unchanged.
        set_timer_slack(original).expect("restore slack");
    }

    /// 0 is the kernel's "restore the inherited default", not "no slack" --
    /// the contract the doc comment promises and the reason callers pass 1.
    #[test]
    fn zero_restores_the_default_rather_than_setting_zero() {
        std::thread::spawn(|| {
            set_timer_slack(1).expect("set 1 ns");
            assert_eq!(timer_slack_ns(), Some(1));

            set_timer_slack(0).expect("set 0");
            let restored = timer_slack_ns().expect("read slack back");
            assert_ne!(restored, 0, "0 must not arm zero slack");
            assert_ne!(restored, 1, "0 must not leave the previous value in place");
        })
        .join()
        .expect("spawned thread panicked");
    }

    /// On armv7 (a required multi-platform.yml job) `c_ulong` is 32 bits and
    /// the `as` cast this used to do would wrap a wider request into a small
    /// one and still report success. The guard makes this a no-op on 64-bit
    /// targets, where every `u64` fits and there is nothing to reject.
    #[test]
    fn a_value_too_wide_for_c_ulong_is_rejected_not_wrapped() {
        if libc::c_ulong::try_from(u64::MAX).is_err() {
            let err = set_timer_slack(u64::MAX).expect_err("should not have been accepted");
            assert!(
                err.to_string().contains("does not fit"),
                "unexpected error: {err}"
            );
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
    /// cores it was called for.
    #[test]
    fn clearing_a_nonexistent_cpu_is_an_error_not_a_silent_skip() {
        // 4096 is beyond any plausible online set here.
        let r = move_irqs_off_cpus(&[4096]);
        match r {
            Err(e) => {
                let m = e.to_string();
                assert!(
                    m.contains("online set") || m.contains("cannot name") || m.contains("refusing"),
                    "expected a refusal naming the reason, got: {m}"
                );
            }
            Ok(n) => {
                // Only acceptable outcome without /proc/irq present.
                assert_eq!(
                    n, 0,
                    "clearing a CPU that does not exist must not report IRQs moved"
                );
            }
        }
    }
}
