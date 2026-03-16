//! Profiling API for measuring scheduler tick timing with percentile statistics.
//!
//! Provides `Scheduler::profile(n)` which runs N ticks and returns a
//! `ProfileReport` with min/median/p95/p99/max percentiles, per-node
//! breakdown, and deadline/budget violation counts.
//!
//! Not to be confused with `horus bench` (CLI) which runs user-written
//! benchmark functions via cargo bench / pytest. This profiles the
//! scheduler's own execution timing.

use super::*;
use std::fmt;

/// Structured benchmark results with percentile timing statistics.
///
/// All timing fields use `std::time::Duration` (not f64 ms).
/// Use the `Display` impl for a formatted table, or access fields directly
/// for CI assertions.
///
/// # Example
/// ```rust,ignore
/// let report = scheduler.profile(1000)?;
/// println!("{report}");
///
/// // CI gate
/// assert!(report.p99 < 1_u64.ms());
/// assert_eq!(report.deadline_misses, 0);
/// ```
pub struct ProfileReport {
    /// Number of ticks measured (after warmup).
    pub ticks: u64,
    /// Total wall-clock duration of measured ticks.
    pub duration: Duration,
    /// Minimum tick duration.
    pub min: Duration,
    /// Median (50th percentile) tick duration.
    pub median: Duration,
    /// 95th percentile tick duration.
    pub p95: Duration,
    /// 99th percentile tick duration.
    pub p99: Duration,
    /// Maximum tick duration.
    pub max: Duration,
    /// Total deadline misses during benchmark (including warmup).
    pub deadline_misses: u64,
    /// Total budget overruns during benchmark (including warmup).
    pub budget_overruns: u64,
    /// Per-node timing breakdown.
    pub nodes: Vec<NodeTiming>,
}

/// Per-node timing from a benchmark run.
pub struct NodeTiming {
    /// Node name.
    pub name: String,
    /// Median tick duration for this node (proxy: uses avg from metrics).
    pub median: Duration,
    /// 99th percentile tick duration for this node (proxy: uses max from metrics).
    pub p99: Duration,
    /// Fraction of budget used (0.0 to 1.0). None if no budget set.
    pub budget_used: Option<f64>,
}

impl Scheduler {
    /// Run `n` ticks and return structured timing analysis.
    ///
    /// First 10% of ticks are warmup (discarded from statistics).
    /// Collects per-tick timing for percentile computation.
    ///
    /// Per-node data uses existing `metrics()` avg/max as proxies for
    /// median/p99. True per-node percentiles would require per-tick
    /// per-node timing collection (a future enhancement).
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus::prelude::*;
    ///
    /// let report = scheduler.profile(1000)?;
    /// println!("{report}");
    ///
    /// assert!(report.p99 < 1_u64.ms(), "p99 too high: {:?}", report.p99);
    /// assert_eq!(report.deadline_misses, 0);
    ///
    /// for node in &report.nodes {
    ///     println!("{}: median={:?} p99={:?}", node.name, node.median, node.p99);
    /// }
    /// ```
    pub fn profile(&mut self, n: u64) -> HorusResult<ProfileReport> {
        if n == 0 {
            return Ok(ProfileReport {
                ticks: 0,
                duration: Duration::ZERO,
                min: Duration::ZERO,
                median: Duration::ZERO,
                p95: Duration::ZERO,
                p99: Duration::ZERO,
                max: Duration::ZERO,
                deadline_misses: 0,
                budget_overruns: 0,
                nodes: vec![],
            });
        }

        let warmup = n / 10;
        let measure = n - warmup;

        // Warmup phase (discarded from statistics)
        for _ in 0..warmup {
            self.tick_once()?;
        }

        // Measurement phase — collect per-tick durations
        let mut tick_times = Vec::with_capacity(measure as usize);
        let start_total = Instant::now();

        for _ in 0..measure {
            let tick_start = Instant::now();
            self.tick_once()?;
            tick_times.push(tick_start.elapsed());
        }

        let total_duration = start_total.elapsed();

        // Compute percentiles
        tick_times.sort();
        let len = tick_times.len();

        // Per-node data from metrics() + registered node budget
        let node_timings: Vec<NodeTiming> = self
            .metrics()
            .iter()
            .map(|m| {
                let avg_dur = Duration::from_secs_f64(m.avg_tick_duration_ms() / 1000.0);
                let max_dur = Duration::from_secs_f64(m.max_tick_duration_ms() / 1000.0);

                // Compute budget utilization: avg_tick_duration / tick_budget
                let budget_used = self
                    .nodes
                    .iter()
                    .find(|n| &*n.name == m.name())
                    .and_then(|n| n.tick_budget)
                    .map(|budget| {
                        if budget.is_zero() {
                            0.0
                        } else {
                            avg_dur.as_secs_f64() / budget.as_secs_f64()
                        }
                    });

                NodeTiming {
                    name: m.name().to_string(),
                    median: avg_dur,
                    p99: max_dur,
                    budget_used,
                }
            })
            .collect();

        // Safety stats
        let (dm, bo) = self
            .safety_stats()
            .map(|s| (s.deadline_misses(), s.budget_overruns()))
            .unwrap_or((0, 0));

        Ok(ProfileReport {
            ticks: measure,
            duration: total_duration,
            min: tick_times[0],
            median: tick_times[len / 2],
            p95: tick_times[len * 95 / 100],
            p99: tick_times[len * 99 / 100],
            max: tick_times[len - 1],
            deadline_misses: dm,
            budget_overruns: bo,
            nodes: node_timings,
        })
    }
}

fn format_duration_short(d: Duration) -> String {
    let us = d.as_micros();
    if us < 1000 {
        format!("{}μs", us)
    } else if us < 1_000_000 {
        format!("{:.1}ms", us as f64 / 1000.0)
    } else {
        format!("{:.2}s", d.as_secs_f64())
    }
}

impl fmt::Display for ProfileReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "Benchmark: {} ticks in {}",
            self.ticks,
            format_duration_short(self.duration)
        )?;
        writeln!(f)?;
        writeln!(
            f,
            "  min={}  median={}  p95={}  p99={}  max={}",
            format_duration_short(self.min),
            format_duration_short(self.median),
            format_duration_short(self.p95),
            format_duration_short(self.p99),
            format_duration_short(self.max),
        )?;

        if !self.nodes.is_empty() {
            writeln!(f)?;
            for node in &self.nodes {
                let budget_str = node
                    .budget_used
                    .map(|b| format!("  budget: {:.0}%", b * 100.0))
                    .unwrap_or_default();
                writeln!(
                    f,
                    "  {:<20} median={}  p99={}{}",
                    node.name,
                    format_duration_short(node.median),
                    format_duration_short(node.p99),
                    budget_str,
                )?;
            }
        }

        writeln!(f)?;
        let dm_pct = if self.ticks > 0 {
            self.deadline_misses as f64 / self.ticks as f64 * 100.0
        } else {
            0.0
        };
        let bo_pct = if self.ticks > 0 {
            self.budget_overruns as f64 / self.ticks as f64 * 100.0
        } else {
            0.0
        };
        writeln!(
            f,
            "  Deadline misses: {}/{} ({:.1}%)",
            self.deadline_misses, self.ticks, dm_pct,
        )?;
        writeln!(
            f,
            "  Budget overruns: {}/{} ({:.1}%)",
            self.budget_overruns, self.ticks, bo_pct,
        )?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lock_scheduler() -> std::sync::MutexGuard<'static, ()> {
        // Re-use the same lock as the main scheduler tests
        static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
        LOCK.lock().unwrap_or_else(|e| e.into_inner())
    }

    struct TickNode {
        name: &'static str,
    }

    impl TickNode {
        fn new(name: &'static str) -> Self {
            Self { name }
        }
    }

    impl crate::core::Node for TickNode {
        fn name(&self) -> &str {
            self.name
        }
        fn tick(&mut self) {
            // Minimal work — just burns a tiny bit of time
            std::hint::black_box(42);
        }
    }

    #[test]
    fn test_profile_returns_correct_tick_count() {
        let _guard = lock_scheduler();
        let mut scheduler = Scheduler::new();
        scheduler.add(TickNode::new("node_a")).order(0).build();

        let report = scheduler.profile(100).unwrap();
        // 100 total, 10% warmup = 90 measured
        assert_eq!(report.ticks, 90, "should measure 90 ticks (10% warmup)");
    }

    #[test]
    fn test_profile_percentiles_ordered() {
        let _guard = lock_scheduler();
        let mut scheduler = Scheduler::new();
        scheduler.add(TickNode::new("node_b")).order(0).build();

        let report = scheduler.profile(200).unwrap();
        assert!(report.min <= report.median, "min <= median");
        assert!(report.median <= report.p95, "median <= p95");
        assert!(report.p95 <= report.p99, "p95 <= p99");
        assert!(report.p99 <= report.max, "p99 <= max");
    }

    #[test]
    fn test_profile_display_format() {
        let _guard = lock_scheduler();
        let mut scheduler = Scheduler::new();
        scheduler.add(TickNode::new("motor")).order(0).build();

        let report = scheduler.profile(50).unwrap();
        let output = format!("{report}");

        assert!(output.contains("Benchmark:"), "should contain header");
        assert!(output.contains("median="), "should contain median");
        assert!(output.contains("p99="), "should contain p99");
        assert!(
            output.contains("Deadline misses:"),
            "should contain deadline line"
        );
        assert!(
            output.contains("Budget overruns:"),
            "should contain budget line"
        );
        assert!(output.contains("motor"), "should contain node name");
    }

    #[test]
    fn test_profile_zero_ticks() {
        let _guard = lock_scheduler();
        let mut scheduler = Scheduler::new();
        scheduler.add(TickNode::new("node_c")).order(0).build();

        let report = scheduler.profile(0).unwrap();
        assert_eq!(report.ticks, 0);
        assert_eq!(report.duration, Duration::ZERO);
        assert_eq!(report.min, Duration::ZERO);
    }

    #[test]
    fn test_profile_node_timings_populated() {
        let _guard = lock_scheduler();
        let mut scheduler = Scheduler::new();
        scheduler.add(TickNode::new("alpha")).order(0).build();
        scheduler.add(TickNode::new("beta")).order(1).build();

        let report = scheduler.profile(100).unwrap();
        assert_eq!(report.nodes.len(), 2, "should have 2 node timings");
        assert_eq!(report.nodes[0].name, "alpha");
        assert_eq!(report.nodes[1].name, "beta");
    }

    #[test]
    fn test_format_duration_short() {
        assert_eq!(format_duration_short(Duration::from_micros(42)), "42μs");
        assert_eq!(format_duration_short(Duration::from_micros(1500)), "1.5ms");
        assert_eq!(
            format_duration_short(Duration::from_secs_f64(1.234)),
            "1.23s"
        );
    }

    #[test]
    fn test_profile_budget_used_computed() {
        let _guard = lock_scheduler();
        let mut scheduler = Scheduler::new();
        scheduler
            .add(TickNode::new("budgeted"))
            .rate(100_u64.hz())
            .build();

        let report = scheduler.profile(100).unwrap();
        assert_eq!(report.nodes.len(), 1);
        // Node has a rate → auto-derived budget → budget_used should be Some
        assert!(
            report.nodes[0].budget_used.is_some(),
            "node with rate should have budget_used computed"
        );
        let used = report.nodes[0].budget_used.unwrap();
        assert!(
            used >= 0.0 && used <= 1.0,
            "budget_used should be 0.0-1.0, got {}",
            used
        );
    }

    #[test]
    fn test_profile_no_budget_is_none() {
        let _guard = lock_scheduler();
        let mut scheduler = Scheduler::new();
        // No .rate() → no budget → budget_used should be None
        scheduler.add(TickNode::new("no_budget")).order(0).build();

        let report = scheduler.profile(50).unwrap();
        assert!(
            report.nodes[0].budget_used.is_none(),
            "node without budget should have budget_used = None"
        );
    }
}
