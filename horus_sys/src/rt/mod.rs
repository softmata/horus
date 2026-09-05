//! Real-time scheduling — priority, memory locking, CPU affinity, and
//! priority-inheriting locking.
//!
//! Provides platform-specific RT primitives:
//! - **Linux**: `SCHED_FIFO` + `mlockall` + `sched_setaffinity`
//! - **macOS**: `THREAD_TIME_CONSTRAINT_POLICY` via Mach
//! - **Windows**: `REALTIME_PRIORITY_CLASS` + `SetThreadPriority`
//!
//! Plus [`PiMutex`], the priority-inheriting mutex. Setting `SCHED_FIFO` on a
//! control thread is only half the job: a plain futex lock shared with a
//! lower-priority thread reintroduces unbounded blocking no matter how the
//! scheduler is configured, so the lock belongs next to the scheduling calls.
//!
//! Higher-level types (RtConfig, RuntimeCapabilities) remain in horus_core.
//! This module provides the raw platform primitives they call.

use serde::{Deserialize, Serialize};
use std::time::Duration;

#[cfg(target_os = "linux")]
mod linux;
#[cfg(target_os = "macos")]
mod macos;
pub mod pi_mutex;
#[cfg(target_os = "windows")]
mod windows;

pub use pi_mutex::{priority_inheritance_available, PiMutex, PiMutexGuard};

// ============================================================================
// RT Capabilities (platform-aware detection)
// ============================================================================

// ── Preemption model and clocksource ────────────────────────────────────
//
// Neither removes a nanosecond of latency. They are here because every other
// number this framework prints is conditional on them, and until now nothing
// read either one.
//
// The preemption model was reduced to a single `bool` from a substring test on
// `/proc/version`, so every non-PREEMPT_RT kernel landed in one bucket —
// `none`, `voluntary`, `full` and `lazy` alike, whose worst-case scheduling
// latencies differ by roughly two orders of magnitude end to end. And on a
// `CONFIG_PREEMPT_DYNAMIC` kernel the effective model is a runtime setting that
// `/proc/version` cannot report at all: the token there announces
// runtime-settability, not a mode.
//
// The clocksource was never read. Only clocksources with a non-`NONE`
// `vdso_clock_mode` are served from the vDSO — `tsc`, `kvm-clock`, the Hyper-V
// TSC page, arm64's `arch_sys_counter`. `hpet` lost its vDSO page in 4.20 and
// `acpi_pm` never had one, so on either every `clock_gettime` is a syscall plus
// an uncached device read. The kernel demotes without asking
// ("clocksource: Marking TSC unstable due to clocksource watchdog"), and the RT
// guard spin reads the clock once per iteration.

/// The preemption model a kernel is running under.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum PreemptModel {
    /// `CONFIG_PREEMPT_NONE`. Kernel code yields only at explicit reschedule
    /// points; worst-case scheduling latency is milliseconds.
    None,
    /// `CONFIG_PREEMPT_VOLUNTARY`. Adds `might_resched()` points. Hundreds of µs.
    Voluntary,
    /// `CONFIG_PREEMPT`. Preemptible outside critical sections. Tens of µs.
    Full,
    /// `CONFIG_PREEMPT_LAZY` (6.13+). Full preemption for RT and deadline tasks;
    /// SCHED_OTHER gets a lazy reschedule instead. For an RT thread this
    /// behaves as [`Full`](Self::Full).
    Lazy,
    /// `CONFIG_PREEMPT_RT`. Sleeping spinlocks, threaded IRQs. Single-digit µs.
    PreemptRt,
    /// Linux, but nothing readable said which model is in force.
    ///
    /// An admission, not a value — and deliberately distinct from
    /// [`NotApplicable`](Self::NotApplicable).
    #[default]
    Unknown,
    /// The platform has no preemption model to read (macOS, Windows).
    NotApplicable,
}

impl PreemptModel {
    /// The name the kernel itself uses.
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Voluntary => "voluntary",
            Self::Full => "full",
            Self::Lazy => "lazy",
            Self::PreemptRt => "preempt_rt",
            Self::Unknown => "unknown",
            Self::NotApplicable => "n/a",
        }
    }

    /// Whether an RT thread is preemptible promptly under this model.
    ///
    /// `Lazy` counts: its laziness applies to SCHED_OTHER, and RT and deadline
    /// tasks still preempt immediately.
    pub fn is_rt_friendly(&self) -> bool {
        matches!(self, Self::Full | Self::Lazy | Self::PreemptRt)
    }
}

/// Where a [`PreemptModel`] came from — that is, how much to believe it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum PreemptSource {
    /// `/sys/kernel/debug/sched/preempt` — the RUNNING value, and the only
    /// source that survives a runtime `echo full > .../preempt`. Root-only in
    /// practice: `/sys/kernel/debug` is mode 0700, owned by root.
    Debugfs,
    /// `preempt=` on `/proc/cmdline` — the boot override. A later debugfs write
    /// is invisible here.
    Cmdline,
    /// `CONFIG_PREEMPT_*` in `/boot/config-<release>` — the build default. On a
    /// dynamic kernel this is only what the kernel booted with.
    KernelConfig,
    /// The `PREEMPT*` token in `/proc/version`.
    ProcVersion,
    /// `/sys/kernel/realtime` exists.
    RealtimeSysfs,
    /// Nothing readable said. Pairs only with [`PreemptModel::Unknown`] or
    /// [`PreemptModel::NotApplicable`].
    #[default]
    Unavailable,
}

impl PreemptSource {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Debugfs => "/sys/kernel/debug/sched/preempt",
            Self::Cmdline => "/proc/cmdline",
            Self::KernelConfig => "kernel config",
            Self::ProcVersion => "/proc/version",
            Self::RealtimeSysfs => "/sys/kernel/realtime",
            Self::Unavailable => "unavailable",
        }
    }
}

/// The preemption model, and how it was established.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct PreemptInfo {
    /// The effective model, as best as could be established.
    pub model: PreemptModel,
    /// `CONFIG_PREEMPT_DYNAMIC`: the model is a runtime setting that root can
    /// change with one write, so anything but [`PreemptSource::Debugfs`] can be
    /// stale.
    pub dynamic: bool,
    /// Which rung of the ladder answered.
    pub source: PreemptSource,
}

impl PreemptInfo {
    /// The platform has no preemption model.
    pub fn not_applicable() -> Self {
        Self {
            model: PreemptModel::NotApplicable,
            dynamic: false,
            source: PreemptSource::Unavailable,
        }
    }

    /// Whether this reports the RUNNING value rather than an inference.
    ///
    /// False on a dynamic kernel whose debugfs is closed — which is the common
    /// unprivileged case, and the reason `source` is carried at all: it lets a
    /// report say "this is the boot default, confirm with debugfs" instead of
    /// asserting a runtime value it never read.
    pub fn is_authoritative(&self) -> bool {
        !self.dynamic || self.source == PreemptSource::Debugfs
    }

    /// One-line human summary.
    pub fn describe(&self) -> String {
        let mut s = self.model.as_str().to_string();
        match (self.dynamic, self.is_authoritative()) {
            (true, true) => s.push_str(" (dynamic, running value)"),
            (true, false) => s.push_str(&format!(
                " (dynamic, inferred from {})",
                self.source.as_str()
            )),
            (false, _) if self.model == PreemptModel::Unknown => {}
            (false, _) => s.push_str(&format!(" (from {})", self.source.as_str())),
        }
        s
    }
}

/// The clocksource the kernel is using to serve `clock_gettime`.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct Clocksource(String);

impl Clocksource {
    /// Wrap a clocksource name as the kernel spells it.
    pub fn from_name(name: impl Into<String>) -> Self {
        Self(name.into())
    }

    /// Nothing could be read.
    pub fn unknown() -> Self {
        Self(String::new())
    }

    /// The kernel's name for it, or `"unknown"`.
    pub fn name(&self) -> &str {
        if self.0.is_empty() {
            "unknown"
        } else {
            &self.0
        }
    }

    /// Whether a `clock_gettime` on this clocksource is served from userspace.
    ///
    /// `Some(true)`: served from the vDSO — a ~25 ns read.
    /// `Some(false)`: every read traps into the kernel AND does an uncached
    /// device access. Measured on the reference box, 2M iterations each: 25.4 ns
    /// through the vDSO against 187.3 ns for the bare syscall on the *same* tsc
    /// clocksource — a 7.4x penalty before the device read is added on top.
    /// `None`: not a name this table knows, so the cost is genuinely unknown
    /// and saying so beats guessing.
    pub fn vdso_fast(&self) -> Option<bool> {
        match self.0.as_str() {
            // Non-NONE `vdso_clock_mode` in the kernel's clocksource table.
            "tsc"
            | "kvm-clock"
            | "hyperv_clocksource_tsc_page"
            | "arch_sys_counter"
            | "mach_absolute_time" => Some(true),
            // `hpet` lost its vDSO page in 4.20; `acpi_pm` never had one.
            "hpet" | "acpi_pm" | "jiffies" | "pit" | "xen" => Some(false),
            _ => None,
        }
    }

    /// One-line human summary.
    pub fn describe(&self) -> String {
        match self.vdso_fast() {
            Some(true) => format!("{} (userspace read)", self.name()),
            Some(false) => format!("{} (every read is a syscall)", self.name()),
            None => format!("{} (read cost unknown)", self.name()),
        }
    }
}

// ── CPU idle-state exit latency ─────────────────────────────────────────
//
// See `linux::deepest_idle_exit_latency_at` for the mechanism and the measured
// numbers. The short version: the absolute-sleep design gives the core back for
// most of every period, which is precisely what licenses the cpuidle governor
// to promote it into a deep C-state whose exit dwarfs the guard spin — and
// setting the cpufreq governor to `performance`, which this crate does do, is a
// different subsystem and does nothing about it.

/// Which CPUs an idle-latency bound applies to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IdleLatencyScope {
    /// Every CPU, via `/dev/cpu_dma_latency`.
    ///
    /// The kernel releases the constraint the instant the fd closes — including
    /// on SIGKILL — so it cannot leak. The default for that reason.
    Global,
    /// Only these CPUs, via `cpu{N}/power/pm_qos_resume_latency_us`.
    ///
    /// KERNEL CPU IDS, not indices into a core list. Lets the perception cores
    /// keep their power savings on a shared box — but it is sysfs state, so it
    /// must be restored on drop and it LEAKS if the process is killed. Opt-in
    /// for that reason.
    PerCpu(Vec<usize>),
}

impl IdleLatencyScope {
    pub fn describe(&self) -> String {
        match self {
            Self::Global => "all CPUs, via /dev/cpu_dma_latency".to_string(),
            Self::PerCpu(cpus) => format!("CPUs {cpus:?}, via pm_qos_resume_latency_us"),
        }
    }
}

/// A live idle-latency bound. The constraint lasts exactly as long as this
/// value.
#[derive(Debug)]
pub struct IdleLatencyGuard {
    budget_us: u32,
    scope: IdleLatencyScope,
    #[cfg(target_os = "linux")]
    _hold: linux::IdleHold,
}

impl IdleLatencyGuard {
    /// The exit latency, in microseconds, the governor was told to stay under.
    pub fn budget_us(&self) -> u32 {
        self.budget_us
    }

    pub fn scope(&self) -> &IdleLatencyScope {
        &self.scope
    }

    pub fn describe(&self) -> String {
        format!("{} us — {}", self.budget_us, self.scope.describe())
    }
}

/// The outcome of asking the kernel to bound idle-state exit latency.
#[derive(Debug)]
pub enum IdleLatencyOutcome {
    /// In force until the guard is dropped.
    Bounded(IdleLatencyGuard),
    /// This platform has no idle-state QoS interface.
    ///
    /// NOT a degradation: nothing was refused, and there is nothing the
    /// operator could do about it.
    Unsupported {
        platform: &'static str,
        note: &'static str,
    },
    /// The interface exists and the kernel said no. THIS is the degradation —
    /// the machine has deep idle states, they will be entered, and the operator
    /// believes otherwise unless told.
    Refused { path: String, error: String },
}

/// Ask the kernel to keep CPU idle-state exit latency under `budget_us`.
///
/// Hold the returned guard for as long as the RT loop runs. On Linux the global
/// scope is an open fd; dropping it releases the constraint immediately, which
/// is the whole reason the fd is carried rather than closed after the write.
pub fn bound_idle_latency(budget_us: u32, scope: IdleLatencyScope) -> IdleLatencyOutcome {
    #[cfg(target_os = "linux")]
    {
        match &scope {
            IdleLatencyScope::Global => {
                let path = std::path::Path::new(linux::CPU_DMA_LATENCY_DEV);
                match linux::bound_idle_latency_global_at(path, budget_us) {
                    Ok(f) => IdleLatencyOutcome::Bounded(IdleLatencyGuard {
                        budget_us,
                        scope,
                        _hold: linux::IdleHold::Global(f),
                    }),
                    Err(e) => IdleLatencyOutcome::Refused {
                        path: linux::CPU_DMA_LATENCY_DEV.to_string(),
                        error: e.to_string(),
                    },
                }
            }
            IdleLatencyScope::PerCpu(cpus) => {
                let root = std::path::Path::new(linux::CPU_SYSFS_ROOT);
                match linux::bound_idle_latency_per_cpu_at(root, cpus, budget_us) {
                    Ok(h) => IdleLatencyOutcome::Bounded(IdleLatencyGuard {
                        budget_us,
                        scope,
                        _hold: linux::IdleHold::PerCpu(h),
                    }),
                    Err(e) => IdleLatencyOutcome::Refused {
                        path: format!(
                            "{}/cpuN/power/pm_qos_resume_latency_us",
                            linux::CPU_SYSFS_ROOT
                        ),
                        error: e.to_string(),
                    },
                }
            }
        }
    }
    #[cfg(target_os = "macos")]
    {
        let _ = (budget_us, scope);
        IdleLatencyOutcome::Unsupported {
            platform: "macOS",
            note: "Darwin exposes no PM QoS interface for idle-state exit latency; \
                   the closest equivalent is a power-assertion, which governs sleep \
                   rather than per-core C-states",
        }
    }
    #[cfg(target_os = "windows")]
    {
        let _ = (budget_us, scope);
        IdleLatencyOutcome::Unsupported {
            platform: "Windows",
            note: "idle-state depth is governed by the active power plan \
                   (`powercfg /setacvalueindex ... IDLEDISABLE`), a machine-wide \
                   setting with no per-process interface",
        }
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        let _ = (budget_us, scope);
        IdleLatencyOutcome::Unsupported {
            platform: "this platform",
            note: "no known idle-state QoS interface",
        }
    }
}

/// The deepest idle state the cpuidle governor may currently enter on `cpu`,
/// and its exit latency in microseconds.
///
/// `None` where cpuidle sysfs is absent — non-Linux, `cpuidle.off=1`, and most
/// VMs.
pub fn deepest_idle_exit_latency_us(cpu: usize) -> Option<(String, u32)> {
    #[cfg(target_os = "linux")]
    {
        linux::deepest_idle_exit_latency_at(std::path::Path::new(linux::CPU_SYSFS_ROOT), cpu)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = cpu;
        None
    }
}

// ── RT bandwidth control ────────────────────────────────────────────────
//
// Linux polices the SCHED_FIFO/SCHED_RR class as a whole: `sched_rt_runtime_us`
// out of every `sched_rt_period_us`, 950 ms out of 1000 ms by default. An RT
// runqueue that exceeds its share is forcibly DEQUEUED for the remainder of the
// period — 50 ms with the defaults, which at 1 kHz is fifty consecutive missed
// deadlines.
//
// The tick loop's own comments have named these two sysctls, and the ~50 ms
// dequeue that motivated the absolute-sleep rewrite, for a while. Nothing ever
// read them. So the spin-mode warning was REASONED rather than MEASURED, with
// three consequences: it fired even for an operator who had already set
// `sched_rt_runtime_us=-1` — the exact remedy the comment recommends — which is
// how operators learn to ignore a warning block; its "~50 ms" and "~50 missed
// deadlines at 1kHz" were hardcoded, and are wrong for any other budget or tick
// rate; and under `CONFIG_RT_GROUP_SCHED` a task in a non-root cpu cgroup has
// `cpu.rt_runtime_us == 0`, so `sched_setscheduler(SCHED_FIFO)` fails with
// EPERM and surfaced as a generic refusal pointing at CAP_SYS_NICE — the wrong
// remedy entirely.

/// Where an [`RtBandwidth`] reading came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum RtBandwidthSource {
    /// `/proc/sys/kernel/sched_rt_{runtime,period}_us` — the host-wide budget.
    ProcSys,
    /// `cpu.rt_runtime_us` in this task's cgroup-v1 cpu controller. Present
    /// only where `CONFIG_RT_GROUP_SCHED` is compiled in, and TIGHTER than the
    /// host-wide budget when it is, so it is what actually binds.
    CgroupV1,
    /// Nothing readable — not Linux, or `/proc/sys` hidden.
    #[default]
    Unavailable,
}

/// The real-time class's CPU budget.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct RtBandwidth {
    /// Microseconds of RT execution permitted per period. `-1` is unlimited.
    pub runtime_us: i64,
    /// Length of the accounting period in microseconds.
    pub period_us: i64,
    /// Which file answered.
    pub source: RtBandwidthSource,
}

impl Default for RtBandwidth {
    fn default() -> Self {
        Self::UNAVAILABLE
    }
}

impl RtBandwidth {
    /// Nothing could be read.
    pub const UNAVAILABLE: Self = Self {
        runtime_us: 0,
        period_us: 0,
        source: RtBandwidthSource::Unavailable,
    };

    pub fn from_raw(runtime_us: i64, period_us: i64, source: RtBandwidthSource) -> Self {
        Self {
            runtime_us,
            period_us,
            source,
        }
    }

    /// Readable AND self-consistent.
    ///
    /// `period_us <= 0` and `runtime_us < -1` are not documented kernel states.
    /// They must report unknown rather than unlimited: treating a garbled read
    /// as "no throttle" would suppress the warning in exactly the case where
    /// nothing is understood about the host.
    pub fn is_known(&self) -> bool {
        self.source != RtBandwidthSource::Unavailable && self.period_us > 0 && self.runtime_us >= -1
    }

    /// `sched_rt_runtime_us == -1`: no throttle — and no runaway defence either.
    pub fn is_unlimited(&self) -> bool {
        self.is_known() && self.runtime_us == -1
    }

    /// A finite share of each period.
    pub fn is_finite(&self) -> bool {
        self.is_known() && self.runtime_us >= 0
    }

    /// Zero budget: `sched_setscheduler(SCHED_FIFO)` cannot succeed here at all.
    pub fn is_starved(&self) -> bool {
        self.is_known() && self.runtime_us == 0
    }

    /// Fraction of each period the RT class may execute for.
    pub fn duty_fraction(&self) -> Option<f64> {
        self.is_finite()
            .then(|| (self.runtime_us as f64 / self.period_us as f64).min(1.0))
    }

    /// `period_us - runtime_us`: the window, per RT period, during which an
    /// over-budget RT runqueue is dequeued. 50 ms for the 950000/1000000
    /// default — the number the tick loop's comments quote.
    pub fn throttle_window(&self) -> Option<Duration> {
        self.is_finite()
            .then(|| Duration::from_micros((self.period_us - self.runtime_us).max(0) as u64))
    }

    /// Consecutive deadlines missed per RT period, at a given tick rate.
    ///
    /// The hardcoded "~50 missed deadlines at 1kHz" is only true for the
    /// default budget at a 1 ms tick; at a 250 µs tick the same budget costs
    /// ~200.
    pub fn missed_ticks_per_period(&self, tick_period: Duration) -> Option<u64> {
        let window = self.throttle_window()?.as_nanos() as u64;
        let tick = (tick_period.as_nanos() as u64).max(1);
        Some(window / tick)
    }

    /// One line for a status block.
    pub fn describe(&self) -> String {
        if !self.is_known() {
            return "unknown".to_string();
        }
        if self.is_unlimited() {
            return "unlimited (sched_rt_runtime_us = -1: no throttle, and no runaway defence)"
                .to_string();
        }
        let pct = self.duty_fraction().unwrap_or(0.0) * 100.0;
        let window = self.throttle_window().unwrap_or_default();
        format!(
            "{}/{} us ({:.1}% of each period; over-budget RT is dequeued for {:?}) from {}",
            self.runtime_us,
            self.period_us,
            pct,
            window,
            match self.source {
                RtBandwidthSource::ProcSys => "/proc/sys/kernel",
                RtBandwidthSource::CgroupV1 => "the cgroup cpu controller",
                RtBandwidthSource::Unavailable => "nowhere",
            }
        )
    }
}

/// The real-time class's CPU budget, as it applies to THIS task.
///
/// Reads the host-wide sysctls, then prefers a tighter cgroup-v1
/// `cpu.rt_runtime_us` when one exists — that is the budget that actually
/// binds. Every read is world-readable and unprivileged; a failure is
/// [`RtBandwidth::UNAVAILABLE`], never an error.
///
/// Not applicable off Linux, which has no equivalent policing of a real-time
/// class.
pub fn rt_bandwidth() -> RtBandwidth {
    // Cached: every RT chain asks, and they would otherwise each open the same
    // three files. A process that changes the sysctl mid-run keeps the value
    // read at startup, which is the right scope for a startup diagnostic — and
    // the alternative, re-reading, would put `open`/`read`/`close` on a path
    // reached once per RT thread during the phase-anchor window.
    static CACHED: std::sync::OnceLock<RtBandwidth> = std::sync::OnceLock::new();
    *CACHED.get_or_init(|| {
        #[cfg(target_os = "linux")]
        {
            linux::read_rt_bandwidth()
        }
        #[cfg(not(target_os = "linux"))]
        {
            RtBandwidth::UNAVAILABLE
        }
    })
}

/// Platform RT capabilities detected at startup.
#[derive(Debug, Clone)]
pub struct RtCapabilities {
    /// PREEMPT_RT kernel detected (Linux) or RT-capable scheduler
    pub preempt_rt: bool,
    /// Maximum RT priority the KERNEL defines. Privilege-independent.
    ///
    /// This is a kernel constant, not a permission. `sched_get_priority_max`
    /// answers the same on a host that will refuse the call, and Linux floors
    /// the range with `.max(1)` while macOS and Windows hardcode 99 and 31 — so
    /// `max_priority > 0` is a tautology on every supported platform and says
    /// nothing about whether this process may actually use it. Use
    /// [`Self::rt_priority_permitted`] for that.
    pub max_priority: i32,
    /// Whether this process can actually obtain an RT policy.
    ///
    /// The distinction matters more than it looks: SCHED_FIFO is the
    /// tail-dominant scheduling lever. Measured on one unprivileged 12-core
    /// box, 1 kHz wake lateness over 5000 samples — SCHED_OTHER (with timer
    /// slack already at 1 ns) p99 1276-2494 us / max 3.3-6.4 ms, versus
    /// `chrt -f 80` p99 16.5-114 us / max 0.11-0.84 ms. Reporting it as
    /// available when it is refused hides a 20-100x improvement behind a
    /// one-line fix.
    pub rt_priority_permitted: bool,
    /// Minimum RT priority
    pub min_priority: i32,
    /// Whether memory locking is permitted
    pub memory_locking: bool,
    /// Whether CPU affinity can be set
    pub cpu_affinity: bool,
    /// Kernel version string
    pub kernel_version: String,
    /// Number of available CPUs
    pub cpu_count: usize,
    /// Estimated scheduling jitter
    pub estimated_jitter: Duration,
    /// The kernel's preemption model, and how confident we are in it.
    ///
    /// [`preempt_rt`](Self::preempt_rt) is `preempt.model == PreemptRt`; both
    /// are kept so no existing consumer breaks. The bool alone put `none`,
    /// `voluntary`, `full` and `lazy` in one bucket spanning two orders of
    /// magnitude of scheduling latency.
    pub preempt: PreemptInfo,
    /// The deepest idle state the cpuidle driver may enter on CPU 0, and its
    /// exit latency in microseconds.
    ///
    /// `None` where cpuidle sysfs is absent: non-Linux, `cpuidle.off=1`, and
    /// most VMs.
    pub deepest_idle_state: Option<(String, u32)>,
    /// The clocksource in force AT DETECTION TIME.
    ///
    /// The kernel can demote it mid-run without asking, which is why this is
    /// dated rather than treated as a constant.
    pub clocksource: Clocksource,
}

impl Default for RtCapabilities {
    fn default() -> Self {
        Self {
            preempt_rt: false,
            max_priority: 0,
            rt_priority_permitted: false,
            min_priority: 0,
            memory_locking: false,
            cpu_affinity: true, // core_affinity crate is cross-platform
            kernel_version: String::new(),
            cpu_count: std::thread::available_parallelism()
                .map(|p| p.get())
                .unwrap_or(1),
            estimated_jitter: Duration::from_millis(10),
            preempt: PreemptInfo::not_applicable(),
            deepest_idle_state: None,
            clocksource: Clocksource::unknown(),
        }
    }
}

/// The kernel's preemption model, and how it was established.
///
/// Not applicable off Linux: macOS and Windows have no such setting to read.
pub fn preempt_info() -> PreemptInfo {
    #[cfg(target_os = "linux")]
    {
        linux::read_preempt_info()
    }
    #[cfg(not(target_os = "linux"))]
    {
        PreemptInfo::not_applicable()
    }
}

/// The clocksource serving `clock_gettime`.
///
/// The macOS and Windows names are OUR labels, not strings either OS returns —
/// do not go looking for them in the platform headers. `mach_absolute_time` is
/// a userspace read (commpage on x86, `CNTVCT_EL0` on Apple Silicon). `qpc` is
/// reported as unknown-cost on purpose: QueryPerformanceCounter is usually
/// served from `KUSER_SHARED_DATA`, but the HAL may select HPET or the PM
/// timer, and Windows exposes that choice nowhere ordinary — so "unknown" is
/// the only honest answer.
pub fn clocksource() -> Clocksource {
    #[cfg(target_os = "linux")]
    {
        linux::read_clocksource()
    }
    #[cfg(target_os = "macos")]
    {
        Clocksource::from_name("mach_absolute_time")
    }
    #[cfg(target_os = "windows")]
    {
        Clocksource::from_name("qpc")
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        Clocksource::unknown()
    }
}

/// Detect platform RT capabilities.
pub fn detect_capabilities() -> RtCapabilities {
    #[cfg(target_os = "linux")]
    {
        linux::detect_capabilities()
    }
    #[cfg(target_os = "macos")]
    {
        macos::detect_capabilities()
    }
    #[cfg(target_os = "windows")]
    {
        windows::detect_capabilities()
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        RtCapabilities::default()
    }
}

// ============================================================================
// RT Scheduling Primitives
// ============================================================================

/// Set real-time scheduling priority for the current thread.
///
/// - Linux: `sched_setscheduler(SCHED_FIFO, priority)`
/// - macOS: `thread_policy_set(THREAD_TIME_CONSTRAINT_POLICY)`
/// - Windows: `SetPriorityClass(REALTIME_PRIORITY_CLASS)` + `SetThreadPriority(TIME_CRITICAL)`
pub fn set_realtime_priority(priority: i32) -> anyhow::Result<()> {
    #[cfg(target_os = "linux")]
    {
        linux::set_realtime_priority(priority)
    }
    #[cfg(target_os = "macos")]
    {
        macos::set_realtime_priority(priority)
    }
    #[cfg(target_os = "windows")]
    {
        windows::set_realtime_priority(priority)
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        let _ = priority;
        anyhow::bail!("RT scheduling not supported on this platform")
    }
}

/// Lock all current and future memory pages (prevent swapping).
///
/// - Linux: `mlockall(MCL_CURRENT | MCL_FUTURE)`
/// - macOS: best-effort mlock
/// - Windows: best-effort VirtualLock
pub fn lock_memory() -> anyhow::Result<()> {
    #[cfg(target_os = "linux")]
    {
        linux::lock_memory()
    }
    #[cfg(target_os = "macos")]
    {
        macos::lock_memory()
    }
    #[cfg(target_os = "windows")]
    {
        windows::lock_memory()
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        anyhow::bail!("Memory locking not supported on this platform")
    }
}

/// Set this thread's timer slack in nanoseconds (Linux only).
///
/// Linux defaults to 50 us of slack per thread, which the kernel may add to any
/// timed wait. That is 5 % of a 1 kHz period, applied to every wake. Setting it
/// to 1 ns is what RT applications do.
///
/// A SCHED_FIFO/RR thread already gets zero slack, so this matters precisely
/// for the degraded path where `set_realtime_priority` was refused for lack of
/// CAP_SYS_NICE and HORUS continued at normal priority.
///
/// `nanoseconds` is the slack itself, and 0 is not "no slack": Linux reads a
/// non-positive argument as "restore this thread's inherited default" and hands
/// back the 50 us. Pass 1 for the tightest setting. A value too wide for the
/// target's `unsigned long` (32 bits on armv7) is refused with an error rather
/// than wrapped into a smaller one.
///
/// Returns `Ok(())` and does nothing on platforms without the concept, so
/// callers do not need to branch.
pub fn set_timer_slack(nanoseconds: u64) -> anyhow::Result<()> {
    #[cfg(target_os = "linux")]
    {
        linux::set_timer_slack(nanoseconds)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = nanoseconds;
        Ok(())
    }
}

/// This thread's current timer slack in nanoseconds, or `None` where the
/// platform has no such setting.
///
/// Per-thread, like [`set_timer_slack`]: it reports the slack of whichever
/// thread calls it, so call it from the thread you are asking about rather than
/// from the one that configured it.
pub fn timer_slack_ns() -> Option<u64> {
    #[cfg(target_os = "linux")]
    {
        linux::timer_slack_ns()
    }
    #[cfg(not(target_os = "linux"))]
    {
        None
    }
}

// ── The best-effort class, and what a helper thread inherits ────────────
//
// A thread inherits BOTH its creator's scheduling policy and its creator's CPU
// mask, through two channels with different remedies. RT setup runs before the
// lifecycle hooks and on the same thread, so by the time the net replicator,
// the telemetry HTTP server, the log drain, the diagnostic drain and every
// per-goal action thread are spawned, the spawning thread is already
// SCHED_FIFO and already pinned to the reserved cores — and all of them came up
// that way without asking.
//
// The policy channel has a kernel opt-out (`SCHED_RESET_ON_FORK`, set on the
// RT path). The affinity channel has none: `cpus_allowed` is copied by
// `dup_task_struct` and no flag resets it. So the guarantee has to be an
// explicit demotion in each helper's own thread body, which is what
// [`set_best_effort_class`] is, and [`crate::rt`]'s callers reach it through
// `horus_core::scheduling::rt::spawn_best_effort`.

/// What a demotion actually did. All-false is the good, common case: a helper
/// spawned in a process where no RT setup ever ran had nothing to undo.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BestEffortReport {
    /// The scheduling policy the thread inherited, reset-on-fork bit stripped.
    /// `libc::SCHED_OTHER` (0) on the good path; a Linux-only detail reported
    /// as 0 on other platforms.
    pub prior_policy: i32,
    /// Whether a `sched_setscheduler` was actually issued.
    pub policy_changed: bool,
    /// Whether a `sched_setaffinity` was actually issued.
    pub affinity_changed: bool,
    /// `errno` from whichever half was refused, when one was.
    ///
    /// A refusal is reported rather than propagated: the alternative is a
    /// helper thread that does not start, and a helper on the wrong scheduling
    /// class is strictly better than no helper.
    pub refusal: Option<i32>,
}

/// The CPU set this process held before HORUS narrowed anything.
///
/// Snapshotted rather than reconstructed. A deployment under `taskset` or a
/// cpuset cgroup has a mask HORUS did not choose, and handing helpers CPUs the
/// cgroup forbids fails with EINVAL — so the restore target must be what the
/// process was actually given.
static HELPER_BASELINE: std::sync::OnceLock<Vec<usize>> = std::sync::OnceLock::new();

/// CPUs dedicated to RT tick threads, accumulated across every pin site.
static RESERVED_RT_CPUS: std::sync::Mutex<Vec<usize>> = std::sync::Mutex::new(Vec::new());

/// Demotions that actually issued a syscall. A test hook, and the number that
/// tells an operator whether the inheritance was real on their machine.
static BEST_EFFORT_DEMOTIONS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// Record the CPU set this process holds, as the target helper threads are
/// restored to.
///
/// Idempotent, and the FIRST call wins — so it must run before anything
/// narrows the mask. Called from `Scheduler::new`, from `RtConfig::apply` and
/// from `set_thread_affinity`, all of which are cheap and any of which may be
/// the earliest in a given process.
pub fn capture_helper_baseline_cpus() {
    let _ = HELPER_BASELINE.get_or_init(current_affinity);
}

/// Record CPUs dedicated to RT tick threads. Additive and idempotent.
pub fn reserve_rt_cpus(cpus: &[usize]) {
    if cpus.is_empty() {
        return;
    }
    let mut reserved = RESERVED_RT_CPUS.lock().unwrap_or_else(|e| e.into_inner());
    for &cpu in cpus {
        if !reserved.contains(&cpu) {
            reserved.push(cpu);
        }
    }
    reserved.sort_unstable();
}

/// The CPU set helper threads belong on: the captured baseline minus the
/// reserved RT CPUs.
///
/// Falls back to the whole baseline when that subtraction would be empty —
/// `sched_setaffinity` rejects an empty mask with EINVAL, and an operator who
/// gave the entire machine to RT still needs the helpers to run somewhere.
/// Empty only when no baseline was ever captured, which means "do not touch
/// this thread's affinity".
pub fn helper_thread_cpus() -> Vec<usize> {
    let Some(baseline) = HELPER_BASELINE.get() else {
        return Vec::new();
    };
    let reserved = RESERVED_RT_CPUS.lock().unwrap_or_else(|e| e.into_inner());
    let helpers: Vec<usize> = baseline
        .iter()
        .copied()
        .filter(|c| !reserved.contains(c))
        .collect();
    if helpers.is_empty() {
        baseline.clone()
    } else {
        helpers
    }
}

/// Process-wide count of demotions that actually issued a syscall.
pub fn best_effort_demotions() -> u64 {
    BEST_EFFORT_DEMOTIONS.load(std::sync::atomic::Ordering::Relaxed)
}

/// Put the CALLING thread on the OS's ordinary time-shared class and give it
/// back [`helper_thread_cpus`].
///
/// Call it as the first statement of every non-RT thread body. It never fails
/// hard; see [`BestEffortReport::refusal`].
pub fn set_best_effort_class(nice_increment: i32) -> BestEffortReport {
    let target = helper_thread_cpus();

    #[cfg(target_os = "linux")]
    let report = linux::set_best_effort_class(nice_increment, &target);

    #[cfg(target_os = "macos")]
    let report = macos::set_best_effort_class(nice_increment, &target);

    #[cfg(target_os = "windows")]
    let report = windows::set_best_effort_class(nice_increment, &target);

    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    let report = {
        let _ = (nice_increment, &target);
        BestEffortReport::default()
    };

    if report.policy_changed || report.affinity_changed {
        BEST_EFFORT_DEMOTIONS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    }
    report
}

/// A `sched_getscheduler` return value with the reset-on-fork bit stripped.
///
/// Exported for `horus_core`, which compares the raw policy in
/// `RtConfig::get_current_scheduler`. The kernel ORs the flag into the
/// readback, so an unmasked comparison reports a SCHED_FIFO thread as
/// `Normal`.
#[cfg(target_os = "linux")]
pub fn policy_without_reset_flag(raw: i32) -> i32 {
    linux::policy_of(raw)
}

/// Pin the current thread to one CPU, addressed by kernel CPU id.
///
/// "Id", not "index into a list of the CPUs this process may use" — see
/// [`pin_to_cores`] for why that distinction is the whole point of this
/// function and what it used to get wrong.
pub fn pin_to_core(core_id: usize) -> anyhow::Result<()> {
    let installed = pin_to_cores(&[core_id])?;
    if installed != [core_id] {
        anyhow::bail!(
            "asked to pin to CPU {} but the thread ended up on {:?}",
            core_id,
            installed
        );
    }
    Ok(())
}

/// Pin the current thread to **all** of `cores`, addressed by kernel CPU id,
/// and return the mask the kernel actually installed.
///
/// # Two things this deliberately does not do
///
/// **It does not index a list.** It used to: it fetched the CPUs the process
/// was already allowed to run on and used the caller's number as a *position*
/// in that vector. That is correct only when the inherited mask is exactly
/// `0..N-1`, which is true on a developer laptop and false on the tuned host an
/// RT deployment runs on. Boot a machine with `isolcpus=6,7` and init's mask is
/// `{0..5}`: a request for CPU 6 evaluated `6 < 6`, fell through, and the
/// thread ran unpinned across the housekeeping cores — competing with every
/// other process on the box, while the log said only "continuing unpinned" and
/// the isolated cores the operator rebooted for went unused. `sched_setaffinity`
/// takes ids and succeeds on an isolated CPU, because `isolcpus` changes the
/// mask init hands out rather than granting or withholding a permission.
///
/// **It does not stop at the first core that works.** It used to do that too,
/// so `.cores([2, 3])` pinned to CPU 2 alone and the second core was silently
/// dropped. A multi-core request is a request for a mask.
///
/// # Return value
///
/// The read-back affinity, not the request. `sched_setaffinity` intersects the
/// mask with whatever a cpuset cgroup permits and reports success on a partial
/// application, so a caller that logs its own request as the outcome can
/// announce an isolation it did not get. Compare the return value against what
/// you asked for.
///
/// An empty `cores` is "no affinity requested" and returns an empty mask
/// without touching the thread.
pub fn pin_to_cores(cores: &[usize]) -> anyhow::Result<Vec<usize>> {
    if cores.is_empty() {
        return Ok(Vec::new());
    }

    // Snapshot the pre-narrowing mask here rather than relying on every caller
    // to remember. A pin path added later cannot bypass the capture, which is
    // the difference between "helpers are restored to what the operator gave
    // this process" and "helpers are restored to whatever mask happened to be
    // in force". Idempotent; the first call wins.
    capture_helper_baseline_cpus();

    #[cfg(target_os = "linux")]
    {
        linux::set_affinity(cores)
    }

    // No `sched_setaffinity` outside Linux. `core_affinity` addresses cores by
    // the id carried in `CoreId`, which is the processor number on both macOS
    // (where it is advisory — Darwin offers affinity *tags*, not placement) and
    // Windows (`SetThreadAffinityMask`). Pin to each requested core in turn and
    // report the ones that took; there is no mask-setting call to make this one
    // operation.
    #[cfg(not(target_os = "linux"))]
    {
        let core_ids = core_affinity::get_core_ids()
            .ok_or_else(|| anyhow::anyhow!("Failed to get core IDs"))?;
        let mut installed = Vec::new();
        for &want in cores {
            if let Some(id) = core_ids.iter().find(|c| c.id == want) {
                if core_affinity::set_for_current(*id) {
                    installed.push(want);
                }
            }
        }
        if installed.is_empty() {
            anyhow::bail!("Failed to pin to any of cores {:?}", cores);
        }
        Ok(installed)
    }
}

/// The CPUs the calling thread may currently run on, as kernel CPU ids.
///
/// This is what a pin should be verified against; see [`pin_to_cores`].
pub fn current_affinity() -> Vec<usize> {
    #[cfg(target_os = "linux")]
    {
        linux::current_affinity().unwrap_or_else(|_| available_cores())
    }
    #[cfg(not(target_os = "linux"))]
    {
        available_cores()
    }
}

/// The CPU ids the kernel has enumerated, whether or not this process may use
/// them.
///
/// On Linux this is `/sys/devices/system/cpu/present`, which unlike
/// [`available_cores`] still lists a CPU that `isolcpus` has taken out of the
/// inherited mask — so it is the right set to validate a pin *request* against.
/// Empty when it cannot be determined.
pub fn present_cpus() -> Vec<usize> {
    #[cfg(target_os = "linux")]
    {
        linux::present_cpus()
    }
    #[cfg(not(target_os = "linux"))]
    {
        available_cores()
    }
}

/// The CPU **ids** this process may currently run on.
///
/// Ids, not positions. This returned `0..n` — a dense count of the allowed
/// CPUs presented as if those were their numbers — which on an `isolcpus=6,7`
/// host reported `[0, 1, 2, 3, 4, 5]` for a machine whose RT cores are 6 and 7,
/// and handed every caller a number that meant nothing to `taskset`, to
/// `/proc/stat`, or to the operator.
pub fn available_cores() -> Vec<usize> {
    core_affinity::get_core_ids()
        .map(|ids| {
            let mut cpus: Vec<usize> = ids.iter().map(|c| c.id).collect();
            cpus.sort_unstable();
            cpus
        })
        .unwrap_or_else(|| vec![0])
}

/// Detect isolated CPUs (Linux `isolcpus` parameter).
pub fn isolated_cores() -> Vec<usize> {
    #[cfg(target_os = "linux")]
    {
        linux::detect_isolated_cpus()
    }
    #[cfg(not(target_os = "linux"))]
    {
        Vec::new()
    }
}

/// Detect nohz_full CPUs (Linux tickless kernel).
pub fn nohz_full_cores() -> Vec<usize> {
    #[cfg(target_os = "linux")]
    {
        linux::detect_nohz_full_cpus()
    }
    #[cfg(not(target_os = "linux"))]
    {
        Vec::new()
    }
}

/// Detect the CPU frequency governor (Linux only).
pub fn cpu_governor() -> Option<String> {
    #[cfg(target_os = "linux")]
    {
        linux::detect_cpu_governor()
    }
    #[cfg(not(target_os = "linux"))]
    {
        None
    }
}

/// Set the CPU frequency governor for a specific core.
///
/// Sets `/sys/devices/system/cpu/cpu{N}/cpufreq/scaling_governor`.
/// No-op on non-Linux platforms.
pub fn set_cpu_governor(cpu_id: usize, governor: &str) -> anyhow::Result<()> {
    #[cfg(target_os = "linux")]
    {
        linux::set_cpu_governor(cpu_id, governor)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = (cpu_id, governor);
        Ok(())
    }
}

/// Move hardware interrupts off the specified CPU cores.
///
/// Returns the number of IRQs whose affinity was changed.
/// No-op on non-Linux platforms.
pub fn move_irqs_off_cpus(cpus: &[usize]) -> anyhow::Result<usize> {
    #[cfg(target_os = "linux")]
    {
        linux::move_irqs_off_cpus(cpus)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = cpus;
        Ok(0)
    }
}

/// Set SCHED_DEADLINE (EDF) scheduling for the current thread.
///
/// Gives kernel-guaranteed CPU bandwidth: `runtime_ns` of CPU every `period_ns`.
/// Falls back gracefully — callers should try SCHED_FIFO on failure.
pub fn set_deadline_scheduling(
    runtime_ns: u64,
    deadline_ns: u64,
    period_ns: u64,
) -> anyhow::Result<()> {
    #[cfg(target_os = "linux")]
    {
        linux::set_deadline_scheduling(runtime_ns, deadline_ns, period_ns)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = (runtime_ns, deadline_ns, period_ns);
        anyhow::bail!("SCHED_DEADLINE is Linux-only")
    }
}

/// Check if SCHED_DEADLINE is available on this kernel.
pub fn has_deadline_capability() -> bool {
    #[cfg(target_os = "linux")]
    {
        linux::has_deadline_capability()
    }
    #[cfg(not(target_os = "linux"))]
    {
        false
    }
}

/// Check whether RT priority can actually be set (dry-run test).
pub fn can_set_rt_priority() -> bool {
    #[cfg(target_os = "linux")]
    {
        linux::can_set_rt_priority()
    }
    #[cfg(target_os = "macos")]
    {
        // False because `macos::set_realtime_priority` is a stub that returns
        // `Err` on every call -- there is no privilege question to answer while
        // the operation is unimplemented. This said `true` ("macOS doesn't
        // require special privileges for thread policy"), which was true about
        // macOS and false about this crate: the call it was predicting could
        // never succeed. Flip it back when THREAD_TIME_CONSTRAINT_POLICY is
        // actually wired up.
        false
    }
    #[cfg(target_os = "windows")]
    {
        true // Will try SetPriorityClass at runtime
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        false
    }
}

/// Get the RLIMIT_MEMLOCK soft limit in bytes (Linux only, u64::MAX if unlimited).
pub fn memlock_limit_bytes() -> u64 {
    #[cfg(target_os = "linux")]
    {
        linux::get_memlock_limit()
    }
    #[cfg(not(target_os = "linux"))]
    {
        0
    }
}

/// Pre-fault stack memory to avoid page faults during execution.
///
/// Platform-agnostic: touches stack pages using recursive allocation + `black_box`.
#[inline(never)]
pub fn prefault_stack(size: usize) {
    const PAGE_SIZE: usize = 4096;
    let num_pages = size.div_ceil(PAGE_SIZE);
    prefault_stack_recursive(num_pages, 0);
}

#[inline(never)]
fn prefault_stack_recursive(remaining_pages: usize, depth: usize) {
    if remaining_pages == 0 || depth >= 4096 {
        return;
    }

    let mut buffer: [u8; 4096] = [0u8; 4096];
    for i in (0..4096).step_by(64) {
        buffer[i] = std::hint::black_box(i as u8);
    }
    std::hint::black_box(&buffer);

    prefault_stack_recursive(remaining_pages - 1, depth + 1);
}

/// Parse a CPU list string like "0-3,7,9-11" into individual CPU indices.
pub fn parse_cpu_list(s: &str) -> Vec<usize> {
    let mut cpus = Vec::new();
    for part in s.split(',') {
        let part = part.trim();
        if part.is_empty() {
            continue;
        }
        if let Some((start, end)) = part.split_once('-') {
            if let (Ok(s), Ok(e)) = (start.trim().parse::<usize>(), end.trim().parse::<usize>()) {
                cpus.extend(s..=e);
            }
        } else if let Ok(cpu) = part.parse::<usize>() {
            cpus.push(cpu);
        }
    }
    cpus
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_capabilities() {
        let caps = detect_capabilities();
        assert!(caps.cpu_count >= 1);
        assert!(!caps.kernel_version.is_empty() || !cfg!(target_os = "linux"));
    }

    #[test]
    fn test_available_cores() {
        let cores = available_cores();
        assert!(!cores.is_empty());
        assert!(
            cores.windows(2).all(|w| w[0] < w[1]),
            "available_cores must be sorted and free of duplicates: {cores:?}"
        );
    }

    /// `available_cores` reports CPU **ids**, and it used to report `0..n`.
    ///
    /// On a host with isolated cores that is not a cosmetic difference: it
    /// answered `[0, 1, 2, 3, 4, 5]` for a machine whose RT cores are 6 and 7,
    /// and every consumer — `taskset`, `/proc/stat`, the operator — reads those
    /// numbers as CPU ids.
    #[cfg(target_os = "linux")]
    #[test]
    fn available_cores_reports_ids_not_a_dense_count() {
        std::thread::spawn(|| {
            let all = current_affinity();
            if all.len() < 4 {
                eprintln!("skipped: needs >= 4 usable CPUs, have {all:?}");
                return;
            }
            let upper: Vec<usize> = all[all.len() / 2..].to_vec();
            pin_to_cores(&upper).expect("restricting to the upper half");
            assert_eq!(
                available_cores(),
                upper,
                "available_cores must name the CPUs this thread may use, not count them"
            );
        })
        .join()
        .unwrap();
    }

    #[test]
    fn test_parse_cpu_list() {
        assert_eq!(parse_cpu_list("0-3"), vec![0, 1, 2, 3]);
        assert_eq!(parse_cpu_list("0,2,4"), vec![0, 2, 4]);
        assert_eq!(parse_cpu_list("0-2,5,7-9"), vec![0, 1, 2, 5, 7, 8, 9]);
        assert_eq!(parse_cpu_list(""), Vec::<usize>::new());
    }

    #[test]
    fn test_prefault_stack_does_not_panic() {
        prefault_stack(8192); // 2 pages
    }

    #[test]
    fn test_can_set_rt_priority_returns() {
        let _ = can_set_rt_priority(); // just verify it doesn't panic
    }

    /// The capability probe must not change the caller's scheduling class.
    ///
    /// `RuntimeCapabilities::detect()` calls this on whichever thread builds a
    /// `Scheduler`, and the scheduler's tick loop then runs on that same thread.
    /// The probe used to set SCHED_FIFO and "restore" a hardcoded SCHED_OTHER
    /// priority 0, so a process deployed with `chrt -f 80` had its tick loop
    /// silently demoted to the time-sharing class while the detection it just
    /// ran reported RT as available.
    #[cfg(target_os = "linux")]
    #[test]
    fn test_can_set_rt_priority_leaves_thread_policy_untouched() {
        // A scratch thread, so a failure here cannot disturb the rest of the
        // suite. If the process happens to be privileged the thread is put on
        // SCHED_FIFO first, which is the case that used to be destroyed.
        std::thread::spawn(|| {
            // SAFETY: pid 0 = current thread; sched_param is a POD struct for
            // which all-zero is a valid bit pattern.
            unsafe {
                let mut param: libc::sched_param = std::mem::zeroed();
                param.sched_priority = 10;
                libc::sched_setscheduler(0, libc::SCHED_FIFO, &param);
            }

            // SAFETY: pid 0 = current thread; both calls only read.
            let (policy_before, prio_before) = unsafe {
                let mut param: libc::sched_param = std::mem::zeroed();
                libc::sched_getparam(0, &mut param);
                (libc::sched_getscheduler(0), param.sched_priority)
            };

            let _ = can_set_rt_priority();

            // SAFETY: same read-only queries as above.
            let (policy_after, prio_after) = unsafe {
                let mut param: libc::sched_param = std::mem::zeroed();
                libc::sched_getparam(0, &mut param);
                (libc::sched_getscheduler(0), param.sched_priority)
            };

            assert_eq!(
                policy_before, policy_after,
                "can_set_rt_priority() changed the calling thread's scheduling policy"
            );
            assert_eq!(
                prio_before, prio_after,
                "can_set_rt_priority() changed the calling thread's priority"
            );
        })
        .join()
        .unwrap();
    }

    /// The best-effort contract, asserted on every platform.
    ///
    /// This is the test that has to run in the Multi-Platform gate: the macOS
    /// and Windows arms of `set_best_effort_class` are honest no-ops and a
    /// near-no-op respectively, and "compiles" is not the same claim as
    /// "behaves sanely".
    #[test]
    fn best_effort_contract_holds_on_every_platform() {
        capture_helper_baseline_cpus();
        let first = helper_thread_cpus();
        capture_helper_baseline_cpus();
        assert_eq!(
            first,
            helper_thread_cpus(),
            "capture_helper_baseline_cpus must be idempotent — the first call wins"
        );
        assert!(
            !first.is_empty(),
            "every platform must offer helper threads somewhere to run"
        );

        // Reserve the entire baseline. `sched_setaffinity` rejects an empty
        // mask with EINVAL, and an operator who gave the whole machine to RT
        // still needs the helpers to run somewhere — so the subtraction has to
        // fall back to the full baseline rather than to nothing.
        reserve_rt_cpus(&first);
        assert_eq!(
            helper_thread_cpus(),
            first,
            "reserving every CPU must fall back to the whole baseline, not to an \
             empty mask that sched_setaffinity would refuse"
        );

        let report = set_best_effort_class(0);
        assert!(
            report.refusal.is_none(),
            "entering the best-effort class must not be refused on a thread that is \
             already in it: {report:?}"
        );
        assert_eq!(BestEffortReport::default().refusal, None);
        assert!(!BestEffortReport::default().policy_changed);
        assert!(!BestEffortReport::default().affinity_changed);
    }

    #[test]
    fn test_pin_to_cores_empty_is_ok() {
        pin_to_cores(&[]).unwrap();
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn test_capabilities_linux() {
        let caps = detect_capabilities();
        assert!(caps.cpu_count >= 1);
        assert!(caps.max_priority > 0); // Linux always has RT priorities
        assert!(!caps.kernel_version.is_empty());
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn test_isolated_and_nohz_cores() {
        // These may be empty on most systems, but shouldn't panic
        let _ = isolated_cores();
        let _ = nohz_full_cores();
    }

    // ── Capability detection intent tests ────────────────────────────

    #[test]
    fn test_capabilities_cpu_count_matches_parallelism() {
        let caps = detect_capabilities();
        let expected = std::thread::available_parallelism()
            .map(|p| p.get())
            .unwrap_or(1);
        assert_eq!(
            caps.cpu_count, expected,
            "cpu_count should match available_parallelism"
        );
    }

    #[test]
    fn test_capabilities_cpu_affinity_is_true() {
        // core_affinity crate works on all platforms
        let caps = detect_capabilities();
        assert!(
            caps.cpu_affinity,
            "cpu_affinity should be true (core_affinity crate is cross-platform)"
        );
    }

    #[test]
    fn test_capabilities_jitter_is_positive() {
        let caps = detect_capabilities();
        assert!(
            !caps.estimated_jitter.is_zero(),
            "estimated jitter should be > 0"
        );
    }

    // ── parse_cpu_list edge cases ───────────────────────────────────

    #[test]
    fn test_parse_cpu_list_single_cpu() {
        assert_eq!(parse_cpu_list("7"), vec![7]);
    }

    #[test]
    fn test_parse_cpu_list_whitespace() {
        assert_eq!(parse_cpu_list(" 0 - 2 , 5 "), vec![0, 1, 2, 5]);
    }

    #[test]
    fn test_parse_cpu_list_invalid_entries_skipped() {
        assert_eq!(parse_cpu_list("0,abc,2"), vec![0, 2]);
    }

    #[test]
    fn test_parse_cpu_list_trailing_comma() {
        assert_eq!(parse_cpu_list("0,1,"), vec![0, 1]);
    }

    // ── CPU pinning tests ───────────────────────────────────────────

    #[test]
    #[cfg(target_os = "linux")]
    fn test_pin_to_core_zero_succeeds() {
        // Not literally core 0: a cpuset cgroup can exclude it, and then
        // "core 0 should always exist" is true about the machine and false
        // about this process. Pin to a CPU this thread is demonstrably allowed
        // to use, on a thread of its own so the rest of the binary keeps its
        // affinity.
        std::thread::spawn(|| {
            let usable = current_affinity();
            let cpu = *usable.first().expect("a thread runs on at least one CPU");
            pin_to_core(cpu).unwrap();
        })
        .join()
        .unwrap();
    }

    #[test]
    fn test_pin_to_core_invalid_fails() {
        let result = pin_to_core(99999);
        assert!(result.is_err(), "pinning to nonexistent core should fail");
    }

    #[test]
    #[cfg(target_os = "linux")]
    fn pinning_to_a_missing_cpu_is_an_error_not_a_silent_fallback() {
        // This asserted the opposite: that `pin_to_cores(&[99999, 0])`
        // succeeded, because the implementation walked the list and stopped at
        // the first core that worked. That is the behaviour that made
        // `.cores([2, 3])` pin to CPU 2 alone, and it turned a typo'd CPU
        // number into a silent placement on some other core.
        let err = pin_to_cores(&[99999, 0]).unwrap_err().to_string();
        assert!(
            err.contains("99999"),
            "the error must name the CPU that does not exist, got: {err}"
        );
    }

    /// The isolcpus shape, reproduced without isolcpus and without privilege.
    ///
    /// Shrink this thread's own affinity mask so the CPU ids it may use no
    /// longer start at zero, then pin to one of them **by id**. Under the old
    /// implementation — which used the caller's number as a position in the
    /// list of allowed CPUs — the request evaluated `id < len` against a
    /// shortened list, fell through, and the pin silently did not happen. That
    /// is exactly what `isolcpus=6,7` did to a request for CPU 6.
    #[cfg(target_os = "linux")]
    #[test]
    fn pinning_addresses_a_cpu_id_not_a_position_in_the_allowed_set() {
        std::thread::spawn(|| {
            let all = current_affinity();
            if all.len() < 4 {
                eprintln!("skipped: needs >= 4 usable CPUs, have {all:?}");
                return;
            }
            // The upper half: ids that no longer begin at 0.
            let upper: Vec<usize> = all[all.len() / 2..].to_vec();
            let installed = pin_to_cores(&upper).expect("restricting to the upper half");
            assert_eq!(installed, upper, "the restriction itself must take");

            // `target`'s POSITION in the shrunken mask is `upper.len() - 1`,
            // which is strictly less than its ID. Indexing by position would
            // therefore refuse this pin (or take the wrong CPU).
            let target = *upper.last().unwrap();
            assert!(
                target >= upper.len(),
                "the test needs a CPU whose id exceeds its position; got id {target}                  in a {}-CPU mask",
                upper.len()
            );

            let installed = pin_to_cores(&[target]).expect("pinning to a CPU id in the mask");
            assert_eq!(installed, vec![target]);
            assert_eq!(
                current_affinity(),
                vec![target],
                "the kernel's read-back must agree with what pin_to_cores reported"
            );
        })
        .join()
        .unwrap();
    }

    /// A multi-core request is a request for a mask, not for whichever core
    /// happens to work first.
    #[cfg(target_os = "linux")]
    #[test]
    fn pinning_to_several_cpus_installs_all_of_them() {
        std::thread::spawn(|| {
            let all = current_affinity();
            if all.len() < 2 {
                eprintln!("skipped: needs >= 2 usable CPUs, have {all:?}");
                return;
            }
            let want = vec![all[0], all[1]];
            let installed = pin_to_cores(&want).expect("pinning to two CPUs");
            assert_eq!(
                installed, want,
                "both CPUs must be in the mask; stopping at the first one that                  works is what silently halved `.cores([a, b])`"
            );
            assert_eq!(current_affinity(), want);
        })
        .join()
        .unwrap();
    }

    // ── prefault_stack intent test ──────────────────────────────────

    #[test]
    fn test_prefault_stack_larger_size() {
        // 64KB = 16 pages — shouldn't overflow stack or panic
        prefault_stack(65536);
    }

    // ── memlock_limit ───────────────────────────────────────────────

    #[test]
    fn test_memlock_limit_returns_value() {
        let limit = memlock_limit_bytes();
        // On Linux: some positive value or u64::MAX. On non-Linux: 0
        #[cfg(target_os = "linux")]
        assert!(limit > 0, "Linux should have a memlock limit");
        let _ = limit;
    }

    // ── cpu_governor ────────────────────────────────────────────────

    #[test]
    fn test_cpu_governor_does_not_panic() {
        let gov = cpu_governor();
        // May be Some("performance") or None, but shouldn't panic
        let _ = gov;
    }
}

#[cfg(test)]
mod capability_tests {
    use super::*;

    /// `max_priority` must never be used as a stand-in for permission.
    ///
    /// It is a kernel constant: Linux floors the range with `.max(1)`, macOS
    /// hardcodes 99 and Windows 31. So `max_priority > 0` is true on every
    /// supported platform regardless of privilege, which made the three CLI
    /// surfaces that branched on it report SCHED_FIFO as available on a host
    /// that refuses it — and made their "not available" branches unreachable.
    #[test]
    fn max_priority_is_a_constant_not_a_permission() {
        let caps = detect_capabilities();
        assert!(
            caps.max_priority > 0,
            "every supported platform reports a non-zero max_priority, which is \
             exactly why it cannot be used to decide availability (got {})",
            caps.max_priority
        );
    }

    /// The permission field must agree with the probe it is derived from.
    #[test]
    fn rt_priority_permitted_matches_the_probe() {
        let caps = detect_capabilities();
        assert_eq!(
            caps.rt_priority_permitted,
            can_set_rt_priority(),
            "rt_priority_permitted must report what can_set_rt_priority() \
             actually found, not a kernel constant"
        );
    }
}
