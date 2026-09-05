//! RT Readiness Report — proves horus RT capability to customers.
//!
//! Usage from code:
//!   let report = RtReport::generate(Duration::from_secs(5));
//!   report.print();
//!   assert!(report.is_production_ready());
//!
//! Usage from CLI:
//!   horus doctor --rt   (detailed RT audit)

use std::time::{Duration, Instant};

/// Complete RT readiness assessment.
#[derive(Debug)]
pub struct RtReport {
    // System capabilities
    pub kernel: String,
    pub preempt_rt: bool,
    /// The kernel's preemption model, and how confident we are in it.
    pub preempt: horus_sys::rt::PreemptInfo,
    /// The real-time class's CPU budget, as it applies to this task.
    ///
    /// The sysctls the tick loop's comments blame for the ~50 ms dequeue, read
    /// rather than assumed — and preferring a tighter cgroup budget, which is
    /// what actually binds inside a container.
    pub rt_bandwidth: horus_sys::rt::RtBandwidth,
    /// The deepest idle state the cpuidle governor may enter, and its exit
    /// latency in microseconds. `None` where cpuidle sysfs is absent.
    pub deepest_idle_state: Option<(String, u32)>,
    /// The clocksource in force when the report was generated.
    ///
    /// Read again after the benchmark: a clocksource that changed *during* the
    /// measurement is the single most important thing this report can say, and
    /// costs two file reads in a command that already runs for seconds.
    pub clocksource: horus_sys::rt::Clocksource,
    /// Whether this process may actually obtain an RT policy.
    ///
    /// Not "the kernel defines RT priorities" — that is always true. Filling
    /// this from `max_priority > 0` printed `SCHED_FIFO ✓` in the SYSTEM block
    /// of the very report that listed "SCHED_FIFO refused" under ISSUES.
    pub sched_fifo: bool,
    pub memory_locking: bool,
    pub cpu_count: usize,
    pub isolated_cpus: Vec<usize>,

    // Measured jitter (from live benchmark)
    pub jitter_samples: usize,
    pub jitter_min_us: f64,
    pub jitter_mean_us: f64,
    pub jitter_p50_us: f64,
    pub jitter_p99_us: f64,
    pub jitter_p999_us: f64,
    pub jitter_max_us: f64,
    pub target_period_us: f64,
    pub actual_rate_hz: f64,

    // IPC benchmark
    pub ipc_latency_ns: f64,
    pub ipc_throughput_msg_per_sec: f64,

    // Verdict
    pub grade: RtGrade,
    pub issues: Vec<String>,
    pub recommendations: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RtGrade {
    /// Full RT: PREEMPT_RT + SCHED_FIFO + mlockall + isolated CPUs
    /// Jitter <50μs P99. Ready for safety-critical deployment.
    Production,
    /// Partial RT: SCHED_FIFO available, jitter <500μs P99.
    /// Suitable for most robotics applications.
    Standard,
    /// Limited: No SCHED_FIFO or high jitter. Development only.
    Development,
}

impl RtReport {
    /// Run a complete RT readiness assessment.
    ///
    /// `benchmark_duration`: How long to run the jitter benchmark.
    /// Longer = more accurate P99/P999 numbers.
    pub fn generate(benchmark_duration: Duration) -> Self {
        let caps = horus_sys::rt::detect_capabilities();
        let isolated = horus_sys::rt::isolated_cores();

        // Run jitter benchmark
        let target_hz = 1000u64; // 1kHz — standard RT benchmark rate
        let target_period_us = 1_000_000.0 / target_hz as f64;
        let clocksource_before = horus_sys::rt::clocksource();
        let jitter = measure_jitter(target_hz, benchmark_duration);
        let clocksource_after = horus_sys::rt::clocksource();

        // Run IPC benchmark
        let (ipc_lat, ipc_tput) = measure_ipc();

        // Assess
        let mut issues = Vec::new();
        let mut recs = Vec::new();

        if !caps.preempt_rt {
            issues.push("PREEMPT_RT kernel not detected".into());
            recs.push("Install PREEMPT_RT kernel for <20μs jitter: https://wiki.linuxfoundation.org/realtime".into());
        }

        // The preemption model, which `preempt_rt` alone could not distinguish.
        // `none` and `voluntary` have worst-case scheduling latencies in the
        // milliseconds — they are a different problem from "not PREEMPT_RT",
        // and reporting only the latter left the operator with no idea which
        // one they had.
        match caps.preempt.model {
            horus_sys::rt::PreemptModel::None | horus_sys::rt::PreemptModel::Voluntary => {
                issues.push(format!(
                    "Kernel preemption model is `{}`: worst-case scheduling latency is \
                     milliseconds, not microseconds",
                    caps.preempt.model.as_str()
                ));
                recs.push(if caps.preempt.dynamic {
                    "Boot with `preempt=full` — this kernel is CONFIG_PREEMPT_DYNAMIC, so \
                     the model is a runtime setting and needs no rebuild"
                        .into()
                } else {
                    "Install a kernel built with CONFIG_PREEMPT or CONFIG_PREEMPT_RT".into()
                });
            }
            horus_sys::rt::PreemptModel::Unknown => {
                recs.push(
                    "The preemption model could not be determined: \
                     /sys/kernel/debug/sched/preempt needs root, no readable kernel config \
                     was found, and /proc/version carries no distinguishing token — `none` \
                     and `voluntary` are indistinguishable there"
                        .into(),
                );
            }
            _ => {}
        }
        // Never an issue, always a recommendation: an inferred value is not
        // evidence of a problem, only of a question this process could not
        // answer without privilege.
        if !caps.preempt.is_authoritative() {
            recs.push(format!(
                "The effective preemption mode is a runtime setting; this report inferred \
                 `{}` from {}. Confirm with `sudo cat /sys/kernel/debug/sched/preempt`.",
                caps.preempt.model.as_str(),
                caps.preempt.source.as_str()
            ));
        }

        // The clocksource. Every latency figure below is conditional on it.
        match caps.clocksource.vdso_fast() {
            Some(false) => {
                issues.push(format!(
                    "Clocksource is `{}`: every clock read is a syscall plus a device \
                     access instead of a ~25ns vDSO read, and the RT tick loop reads the \
                     clock once per guard-spin iteration. Every latency figure in this \
                     report is inflated by it.",
                    caps.clocksource.name()
                ));
                recs.push(
                    "echo tsc | sudo tee \
                     /sys/devices/system/clocksource/clocksource0/current_clocksource, and \
                     check `dmesg | grep -i clocksource` for 'Marking TSC unstable'"
                        .into(),
                );
            }
            None => {
                recs.push(format!(
                    "Clocksource `{}` is not one this build recognises, so the cost of a \
                     clock read is unknown rather than assumed",
                    caps.clocksource.name()
                ));
            }
            Some(true) => {}
        }
        // RT bandwidth control. Narrow on purpose: a finite budget is the
        // correct default on almost every host and must not become an issue,
        // or every machine would grow one.
        let bandwidth = horus_sys::rt::rt_bandwidth();
        if bandwidth.is_starved() {
            issues.push(
                "The RT budget for this task is ZERO — SCHED_FIFO cannot be granted here \
                 at all. The refusal reads as a permission error, but CAP_SYS_NICE is not \
                 the remedy."
                    .into(),
            );
            recs.push(
                "Raise cpu.rt_runtime_us for this cgroup, or run outside it. This is the \
                 default state of a non-root cpu cgroup on a kernel built with \
                 CONFIG_RT_GROUP_SCHED."
                    .into(),
            );
        } else if bandwidth.is_finite() && std::env::var("HORUS_RT_WAIT").as_deref() == Ok("spin") {
            let window = bandwidth.throttle_window().unwrap_or_default();
            recs.push(format!(
                "HORUS_RT_WAIT=spin against a finite RT budget ({}/{} µs): the kernel will \
                 dequeue the tick thread for {:?} out of every RT period once its share is \
                 exhausted. Raise /proc/sys/kernel/sched_rt_runtime_us or use the default \
                 absolute-sleep wait.",
                bandwidth.runtime_us, bandwidth.period_us, window
            ));
        }

        if clocksource_before != clocksource_after {
            issues.push(format!(
                "The clocksource changed DURING this benchmark, from `{}` to `{}` — the \
                 kernel demoted it mid-measurement (see `dmesg | grep -i clocksource`), so \
                 the numbers below span two different clock-read costs",
                clocksource_before.name(),
                clocksource_after.name()
            ));
        }
        // `max_priority` is a kernel constant and is never 0 on any supported
        // platform, so this issue and its recommendation could never fire — and
        // `horus doctor --rt` would report a p99 jitter figure while suppressing
        // the single explanation for it.
        if !caps.rt_priority_permitted {
            // Say what was observed, not why. `rt_priority_permitted` is one
            // bool: `can_set_rt_priority()` returns false for a missing
            // CAP_SYS_NICE, for RLIMIT_RTPRIO=0, for a seccomp filter that
            // rejects sched_setscheduler, and for a thread whose policy could
            // not be read back — naming a single cause would be a guess in
            // three of those four cases. The remedies below are listed as
            // remedies, not as diagnoses.
            issues.push("SCHED_FIFO refused — this process may not set an RT policy".into());
            recs.push(
                "Grant RT scheduling: sudo setcap cap_sys_nice+ep $(which horus), or run \
                 `horus setup-rt`, which raises RLIMIT_RTPRIO"
                    .into(),
            );
        }
        if !caps.memory_locking {
            issues.push("Memory locking not permitted (page faults possible)".into());
            recs.push("Add to /etc/security/limits.conf: * - memlock unlimited".into());
        }
        if isolated.is_empty() {
            recs.push(
                "Isolate CPUs for RT: add 'isolcpus=2,3 nohz_full=2,3' to kernel cmdline".into(),
            );
        }
        if jitter.p99 > 500.0 {
            issues.push(format!(
                "P99 jitter {:.0}μs exceeds 500μs threshold",
                jitter.p99
            ));
            // Two causes, ordered by likelihood, and deliberately no longer
            // one line. The governor line alone conflated two different
            // subsystems: `performance` addresses FREQUENCY SCALING and has no
            // effect at all on idle-state exit latency, which on this class of
            // machine is the larger term by an order of magnitude.
            recs.push(
                "Check CPU idle-state exit latency: a deep C-state exit can cost \
                 hundreds of microseconds, and the cpufreq governor does not affect it. \
                 HORUS bounds it automatically when it can open /dev/cpu_dma_latency; \
                 run `horus setup-rt` or scripts/setup-realtime.sh to grant that access."
                    .into(),
            );
            recs.push(
                "Check for CPU frequency scaling: cpupower frequency-set -g performance \
                 (a different subsystem from the above)"
                    .into(),
            );
        }

        // A state whose exit the guard spin cannot absorb is worth naming with
        // its own numbers, whatever the measured jitter came out at.
        if let Some((name, exit_us)) = &caps.deepest_idle_state {
            let guard_us = target_period_us / 16.0;
            if (*exit_us as f64) > guard_us {
                issues.push(format!(
                    "cpuidle may enter {name}, whose exit costs {exit_us}us — {:.0}% of \
                     the {target_period_us:.0}us period, against a {guard_us:.0}us guard \
                     spin that runs AFTER the wake it would have to absorb",
                    *exit_us as f64 * 100.0 / target_period_us
                ));
            }
        }
        if jitter.p99 > 50.0 && caps.preempt_rt {
            issues.push(format!(
                "P99 jitter {:.0}μs high for PREEMPT_RT (expected <50μs)",
                jitter.p99
            ));
            recs.push("Check for SMI interrupts: sudo rdmsr 0x34".into());
        }

        // Both arms gate on permission, not on `caps.max_priority > 0` — that
        // is a kernel constant and is non-zero everywhere, so the grade could
        // not drop to `Development` for a missing SCHED_FIFO no matter how the
        // host was configured.
        let grade = if caps.preempt_rt
            && caps.memory_locking
            && caps.rt_priority_permitted
            && jitter.p99 < 50.0
        {
            RtGrade::Production
        } else if caps.rt_priority_permitted && jitter.p99 < 500.0 {
            RtGrade::Standard
        } else {
            RtGrade::Development
        };

        RtReport {
            kernel: caps.kernel_version,
            preempt_rt: caps.preempt_rt,
            preempt: caps.preempt,
            rt_bandwidth: horus_sys::rt::rt_bandwidth(),
            deepest_idle_state: horus_sys::rt::deepest_idle_exit_latency_us(0),
            clocksource: clocksource_after,
            sched_fifo: caps.rt_priority_permitted,
            memory_locking: caps.memory_locking,
            cpu_count: caps.cpu_count,
            isolated_cpus: isolated,
            jitter_samples: jitter.samples,
            jitter_min_us: jitter.min,
            jitter_mean_us: jitter.mean,
            jitter_p50_us: jitter.p50,
            jitter_p99_us: jitter.p99,
            jitter_p999_us: jitter.p999,
            jitter_max_us: jitter.max,
            target_period_us,
            actual_rate_hz: jitter.actual_hz,
            ipc_latency_ns: ipc_lat,
            ipc_throughput_msg_per_sec: ipc_tput,
            grade,
            issues,
            recommendations: recs,
        }
    }

    pub fn is_production_ready(&self) -> bool {
        self.grade == RtGrade::Production
    }

    pub fn print(&self) {
        let grade_str = match self.grade {
            RtGrade::Production => "PRODUCTION ★★★",
            RtGrade::Standard => "STANDARD ★★☆",
            RtGrade::Development => "DEVELOPMENT ★☆☆",
        };
        let check = |b: bool| if b { "✓" } else { "✗" };

        crate::terminal::print_line(
            "╔══════════════════════════════════════════════════════════════╗",
        );
        crate::terminal::print_line(
            "║               HORUS RT READINESS REPORT                     ║",
        );
        crate::terminal::print_line(&format!(
            "║               Grade: {:20}                  ║",
            grade_str
        ));
        crate::terminal::print_line(
            "╠══════════════════════════════════════════════════════════════╣",
        );
        crate::terminal::print_line(
            "║  SYSTEM                                                     ║",
        );
        crate::terminal::print_line(&format!(
            "║    Kernel:         {}",
            if self.kernel.len() > 40 {
                &self.kernel[..40]
            } else {
                &self.kernel
            }
        ));
        crate::terminal::print_line(&format!(
            "║    PREEMPT_RT:     {}                                        ║",
            check(self.preempt_rt)
        ));
        // The two facts every other number in this report is conditional on.
        crate::terminal::print_line(&format!(
            "║    Preempt:        {:<40} ║",
            self.preempt.describe()
        ));
        crate::terminal::print_line(&format!(
            "║    Clocksource:    {:<40} ║",
            self.clocksource.describe()
        ));
        crate::terminal::print_line(&format!(
            "║    RT budget:      {:<40} ║",
            self.rt_bandwidth.describe()
        ));
        crate::terminal::print_line(&format!(
            "║    Deepest idle:   {:<40} ║",
            match &self.deepest_idle_state {
                Some((name, us)) => format!("{name} (exit {us}us)"),
                None => "unknown (no cpuidle sysfs)".to_string(),
            }
        ));
        crate::terminal::print_line(&format!(
            "║    SCHED_FIFO:     {}                                        ║",
            check(self.sched_fifo)
        ));
        crate::terminal::print_line(&format!(
            "║    Memory lock:    {}                                        ║",
            check(self.memory_locking)
        ));
        crate::terminal::print_line(&format!(
            "║    CPUs:           {} total, {} isolated                     ║",
            self.cpu_count,
            self.isolated_cpus.len()
        ));
        crate::terminal::print_line(
            "╠══════════════════════════════════════════════════════════════╣",
        );
        crate::terminal::print_line(&format!(
            "║  JITTER BENCHMARK @ 1kHz ({} samples)                       ║",
            self.jitter_samples
        ));
        crate::terminal::print_line(&format!(
            "║    Min:    {:8.1} μs                                       ║",
            self.jitter_min_us
        ));
        crate::terminal::print_line(&format!(
            "║    Mean:   {:8.1} μs                                       ║",
            self.jitter_mean_us
        ));
        crate::terminal::print_line(&format!(
            "║    P50:    {:8.1} μs                                       ║",
            self.jitter_p50_us
        ));
        crate::terminal::print_line(&format!(
            "║    P99:    {:8.1} μs                                       ║",
            self.jitter_p99_us
        ));
        crate::terminal::print_line(&format!(
            "║    P999:   {:8.1} μs                                       ║",
            self.jitter_p999_us
        ));
        crate::terminal::print_line(&format!(
            "║    Max:    {:8.1} μs                                       ║",
            self.jitter_max_us
        ));
        crate::terminal::print_line(&format!(
            "║    Rate:   {:8.1} Hz (target: 1000 Hz)                     ║",
            self.actual_rate_hz
        ));
        crate::terminal::print_line(
            "╠══════════════════════════════════════════════════════════════╣",
        );
        crate::terminal::print_line(
            "║  IPC BENCHMARK                                              ║",
        );
        crate::terminal::print_line(&format!(
            "║    Latency:    {:8.0} ns per message                       ║",
            self.ipc_latency_ns
        ));
        crate::terminal::print_line(&format!(
            "║    Throughput: {:8.0} msg/sec                              ║",
            self.ipc_throughput_msg_per_sec
        ));
        crate::terminal::print_line(
            "╠══════════════════════════════════════════════════════════════╣",
        );

        if self.issues.is_empty() {
            crate::terminal::print_line(
                "║  No issues found — system is RT-ready.                     ║",
            );
        } else {
            crate::terminal::print_line(&format!(
                "║  ISSUES ({})                                                ║",
                self.issues.len()
            ));
            for issue in &self.issues {
                crate::terminal::print_line(&format!("║    ✗ {}", issue));
            }
        }
        if !self.recommendations.is_empty() {
            crate::terminal::print_line(
                "║  RECOMMENDATIONS                                            ║",
            );
            for rec in &self.recommendations {
                crate::terminal::print_line(&format!("║    → {}", rec));
            }
        }
        crate::terminal::print_line(
            "╚══════════════════════════════════════════════════════════════╝",
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════
// Jitter benchmark — measures actual tick-to-tick timing
// ═══════════════════════════════════════════════════════════════════════

struct JitterResult {
    samples: usize,
    min: f64,
    mean: f64,
    p50: f64,
    p99: f64,
    p999: f64,
    max: f64,
    actual_hz: f64,
}

/// Absolute deviation of each inter-tick interval from the nominal period, in us.
///
/// Split out from [`measure_jitter`] so the arithmetic can be tested without a
/// live clock -- the bug this replaced could not be caught any other way, since
/// the wrong answer is only wrong relative to a period the caller knows and the
/// function did not use.
fn deviations_us(intervals_us: &[f64], period_us: f64) -> Vec<f64> {
    intervals_us
        .iter()
        .map(|iv| (iv - period_us).abs())
        .collect()
}

fn measure_jitter(target_hz: u64, duration: Duration) -> JitterResult {
    let period = Duration::from_nanos(1_000_000_000 / target_hz);
    let mut timestamps =
        Vec::with_capacity((target_hz as usize) * (duration.as_secs() as usize + 1));

    let start = Instant::now();
    let mut next_tick = start + period;

    while start.elapsed() < duration {
        // Spin-wait to target (same technique as RT executor)
        while Instant::now() < next_tick {
            std::hint::spin_loop();
        }
        timestamps.push(Instant::now());
        next_tick += period;
    }

    if timestamps.len() < 2 {
        return JitterResult {
            samples: 0,
            min: 0.0,
            mean: 0.0,
            p50: 0.0,
            p99: 0.0,
            p999: 0.0,
            max: 0.0,
            actual_hz: 0.0,
        };
    }

    let intervals_us: Vec<f64> = timestamps
        .windows(2)
        .map(|w| w[1].duration_since(w[0]).as_nanos() as f64 / 1000.0)
        .collect();
    let actual_hz = intervals_us.len() as f64 / duration.as_secs_f64();

    // Jitter is the DEVIATION from the nominal period, not the interval itself.
    // This used to report the raw interval, which at the 1 kHz benchmark rate is
    // ~1000us on a perfectly idle machine. Every threshold downstream reads this
    // as a deviation: `RtGrade::Production` wants p99 < 50, `Standard` wants
    // p99 < 500, and an issue fires above 500. Against a ~1000us interval all
    // three are decided before the machine is even measured -- Production and
    // Standard were arithmetically unreachable on every host on earth, and every
    // report carried a spurious "P99 jitter 1000us exceeds 500us threshold".
    // `target_period_us` was already being computed and stored in the report,
    // which is the tell that deviation was always the intent.
    let mut deltas_us = deviations_us(&intervals_us, period.as_nanos() as f64 / 1000.0);
    deltas_us.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let n = deltas_us.len();

    JitterResult {
        samples: n,
        min: deltas_us[0],
        mean: deltas_us.iter().sum::<f64>() / n as f64,
        p50: deltas_us[n / 2],
        p99: deltas_us[(n as f64 * 0.99) as usize],
        p999: deltas_us[std::cmp::min((n as f64 * 0.999) as usize, n - 1)],
        max: deltas_us[n - 1],
        actual_hz,
    }
}

// ═══════════════════════════════════════════════════════════════════════
// IPC benchmark — measures Topic send/recv latency
// ═══════════════════════════════════════════════════════════════════════

fn measure_ipc() -> (f64, f64) {
    use crate::communication::topic::Topic;

    let topic_name = format!("_rt_bench_{}", std::process::id());
    let topic: Result<Topic<u64>, _> = Topic::new(&topic_name);

    match topic {
        Ok(t) => {
            // Warmup
            for i in 0..100u64 {
                t.send(i);
            }
            while t.recv().is_some() {}

            // Measure
            let iterations = 10_000u64;
            let start = Instant::now();
            for i in 0..iterations {
                t.send(i);
                let _ = t.recv();
            }
            let elapsed = start.elapsed();

            let latency_ns = elapsed.as_nanos() as f64 / iterations as f64;
            let throughput = iterations as f64 / elapsed.as_secs_f64();

            (latency_ns, throughput)
        }
        Err(_) => (0.0, 0.0),
    }
}

#[cfg(test)]
mod jitter_arithmetic_tests {
    use super::*;

    /// Jitter is a deviation, not an interval.
    ///
    /// A 1 kHz loop ticking perfectly has intervals of 1000us and jitter of ZERO.
    /// Reporting 1000 here is what made `RtGrade::Production` (p99 < 50) and
    /// `RtGrade::Standard` (p99 < 500) unreachable on every machine.
    #[test]
    fn a_perfect_thousand_hz_loop_has_no_jitter() {
        let period_us = 1000.0;
        let intervals = vec![1000.0; 64];
        let dev = deviations_us(&intervals, period_us);
        assert!(
            dev.iter().all(|d| *d == 0.0),
            "a perfectly paced loop must report zero jitter, got {:?}",
            &dev[..4]
        );
    }

    /// Late and early ticks are both jitter, and both count positively.
    #[test]
    fn deviation_is_symmetric_and_absolute() {
        let dev = deviations_us(&[1040.0, 960.0, 1000.0], 1000.0);
        assert_eq!(dev, vec![40.0, 40.0, 0.0]);
    }

    /// The grades have to be reachable by a machine that actually behaves.
    ///
    /// This is the assertion the old code could not satisfy: feed it the timing
    /// of a well-tuned host (within 20us of nominal) and the p99 must land under
    /// the 50us Production bar, not at the 1000us interval.
    #[test]
    fn a_well_tuned_host_clears_the_production_bar() {
        let period_us = 1000.0;
        let mut intervals: Vec<f64> = (0..1000)
            .map(|i| period_us + ((i % 40) as f64 - 20.0))
            .collect();
        // One bad tick, so p99 is exercised rather than max.
        intervals.push(period_us + 900.0);

        let mut dev = deviations_us(&intervals, period_us);
        dev.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let p99 = dev[(dev.len() as f64 * 0.99) as usize];

        assert!(
            p99 < 50.0,
            "p99 deviation {p99}us should clear the 50us Production bar; the old \
             code reported the interval (~{period_us}us) and could never clear it"
        );
    }
}
