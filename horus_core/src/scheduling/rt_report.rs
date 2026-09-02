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
        let jitter = measure_jitter(target_hz, benchmark_duration);

        // Run IPC benchmark
        let (ipc_lat, ipc_tput) = measure_ipc();

        // Assess
        let mut issues = Vec::new();
        let mut recs = Vec::new();

        if !caps.preempt_rt {
            issues.push("PREEMPT_RT kernel not detected".into());
            recs.push("Install PREEMPT_RT kernel for <20μs jitter: https://wiki.linuxfoundation.org/realtime".into());
        }
        if caps.max_priority == 0 {
            issues.push("SCHED_FIFO not available (no CAP_SYS_NICE)".into());
            recs.push("Run: sudo setcap cap_sys_nice+ep $(which horus)".into());
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
            recs.push(
                "Check for CPU frequency scaling: cpupower frequency-set -g performance".into(),
            );
        }
        if jitter.p99 > 50.0 && caps.preempt_rt {
            issues.push(format!(
                "P99 jitter {:.0}μs high for PREEMPT_RT (expected <50μs)",
                jitter.p99
            ));
            recs.push("Check for SMI interrupts: sudo rdmsr 0x34".into());
        }

        let grade =
            if caps.preempt_rt && caps.memory_locking && caps.max_priority > 0 && jitter.p99 < 50.0
            {
                RtGrade::Production
            } else if caps.max_priority > 0 && jitter.p99 < 500.0 {
                RtGrade::Standard
            } else {
                RtGrade::Development
            };

        RtReport {
            kernel: caps.kernel_version,
            preempt_rt: caps.preempt_rt,
            sched_fifo: caps.max_priority > 0,
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

/// Deviation of each observed interval from the target period.
///
/// This is what "jitter" means everywhere else in this module: the fields are
/// named `jitter_*_us`, `RtGrade::Production` documents "Jitter <50us P99", and
/// the issue text reads "P99 jitter ... exceeds 500us threshold".
///
/// The measurement used to return the raw INTERVAL instead -- ~1000us for a 1kHz
/// loop -- so the grade compared a period against a jitter budget. `p99 < 50.0`
/// could never hold at any rate, and `p99 < 500.0` could not hold below 2kHz, so
/// RtGrade::Production and RtGrade::Standard were both unreachable and
/// `is_production_ready()` returned false on every machine including a correctly
/// configured PREEMPT_RT one.
fn jitter_from_intervals(intervals_us: &[f64], period_us: f64) -> Vec<f64> {
    intervals_us
        .iter()
        .map(|interval| (interval - period_us).abs())
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

    // Derived from `period`, not recomputed from `target_hz`. `period` is
    // integer nanoseconds, so for rates that do not divide 1e9 evenly it is
    // truncated (7 Hz: 142857142 ns, not 142857142.857), and computing the
    // jitter baseline independently in floating point would measure against a
    // period the loop never actually slept for. The gap is at most 0.857 ns
    // against a 50 us Production threshold, so no grade was ever affected —
    // this is one source of truth rather than a numeric fix.
    let period_us = period.as_nanos() as f64 / 1000.0;
    let intervals_us: Vec<f64> = timestamps
        .windows(2)
        .map(|w| w[1].duration_since(w[0]).as_nanos() as f64 / 1000.0)
        .collect();
    let mut deltas_us = jitter_from_intervals(&intervals_us, period_us);
    deltas_us.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let n = deltas_us.len();
    let actual_hz = n as f64 / duration.as_secs_f64();

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
mod jitter_tests {
    use super::*;

    /// A perfectly paced loop has ZERO jitter. Before the fix this returned the
    /// period itself (1000us at 1kHz), which is what made every RT grade
    /// unreachable.
    ///
    /// GATE: revert `jitter_from_intervals` to returning the interval unchanged
    /// and this fails with 1000 != 0.
    #[test]
    fn a_perfectly_paced_loop_has_no_jitter() {
        let period_us = 1000.0;
        let intervals = vec![1000.0; 64];
        let jitter = jitter_from_intervals(&intervals, period_us);
        // Epsilon rather than `== 0.0`: `period_us` is derived from a Duration
        // in the caller, so a future rate whose nanosecond period does not
        // divide evenly can leave sub-nanosecond residue. The gate is unharmed
        // — reverting `jitter_from_intervals` yields 1000.0, six orders of
        // magnitude above this bound.
        const EPS_US: f64 = 1e-6;
        assert!(
            jitter.iter().all(|j| j.abs() <= EPS_US),
            "a loop hitting its period exactly must report zero jitter, got {:?}",
            &jitter[..4]
        );
    }

    /// Jitter is the magnitude of the miss, in either direction: running early
    /// is as much a deadline violation as running late.
    #[test]
    fn jitter_is_absolute_deviation_in_both_directions() {
        let jitter = jitter_from_intervals(&[1200.0, 800.0, 1000.0], 1000.0);
        assert_eq!(jitter, vec![200.0, 200.0, 0.0]);
    }

    /// The grade thresholds must be reachable by a well-behaved 1kHz loop.
    /// This is the property the bug actually broke.
    #[test]
    fn a_good_1khz_loop_can_reach_the_production_jitter_budget() {
        // 1kHz with +/-10us of scheduling noise: comfortably inside the 50us
        // Production budget and the 500us Standard budget.
        let intervals: Vec<f64> = (0..100)
            .map(|i| if i % 2 == 0 { 1010.0 } else { 990.0 })
            .collect();
        let mut jitter = jitter_from_intervals(&intervals, 1000.0);
        jitter.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let p99 = jitter[(jitter.len() as f64 * 0.99) as usize];
        assert!(
            p99 < 50.0,
            "p99 jitter {p99}us must fit the Production budget (<50us); before \
             the fix this was ~1000us because the raw period was returned"
        );
    }
}
