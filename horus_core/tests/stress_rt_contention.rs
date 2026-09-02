#![allow(dead_code)]
//! Stress test: RT executor under CPU contention.
//!
//! Verifies that HORUS RT nodes maintain timing consistency when competing
//! cores are saturated. Spawns busy-loop threads on all available cores,
//! then runs RT nodes measuring tick-to-tick jitter and consistency.
//!
//! # Rate aliasing note
//!
//! The RT executor's per-node rate limiter can alias with the loop period
//! when both match exactly (e.g., 1kHz node on a 1kHz loop yields ~500Hz
//! effective rate). This is expected behavior on non-RT kernels where
//! `Instant::now()` granularity creates small timing offsets. The tests
//! account for this by using generous tick-count thresholds and measuring
//! jitter relative to the *mean observed interval*, not the declared rate.
//!
//! # What the gates bound
//!
//! Spread and tail are gated separately, because a spread number cannot fail
//! on a tail. Averaged over ~3000 intervals, one stall of any length moves
//! mean-absolute-deviation by a fraction of a percent, so for a control loop
//! -- where the whole question is the worst cycle, not the average one -- it
//! is the wrong gate to stand alone. It stood alone here: the baseline test
//! passed at 5.92% while its own report showed 3451.7 us max jitter on a
//! 1000 us period. Contention made the number *better*, not worse (3.94%
//! with eleven hog threads against 5.92% idle) while max jitter went up 57%.
//!
//! So `check_timing` bounds three things: spread (`max_mad_percent`), the
//! single worst excursion in units of the declared period
//! (`max_jitter_periods`), and how often intervals run past 3x the mean
//! (`max_stall_rate`), and how close the achieved rate came to the declared one
//! (`min_rate_fraction`). That last one exists because measuring jitter against
//! the mean observed interval -- which is what makes it aliasing-robust -- also
//! makes rate error invisible: a 1 kHz node delivering 200 Hz keeps a flawless
//! jitter profile around its own wrong period, and the tick-count asserts let
//! it through, since `ticks >= 500` over 5 s permits exactly that 100 Hz.
//! Every limit is set from measurements recorded at its call site, not from a
//! target someone hoped for.
//!
//! # Where these run
//!
//! The four timing tests run nowhere in CI -- on a shared runner they measure
//! the runner. Run them locally when touching the tick path. The gate-logic
//! tests at the bottom of this file are pure and deterministic, and do run in
//! CI, so the gate cannot quietly lose its teeth even though the measurements
//! are developer-run.

use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

mod common;
use common::cleanup_stale_shm;

// ============================================================================
// Jitter-recording RT node
// ============================================================================

/// RT node that records every tick timestamp for post-hoc jitter analysis.
struct JitterNode {
    name: String,
    tick_count: Arc<AtomicU64>,
    timestamps: Arc<Mutex<Vec<Instant>>>,
}

impl Node for JitterNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::Relaxed);
        if let Ok(mut ts) = self.timestamps.lock() {
            ts.push(Instant::now());
        }
    }
}

// ============================================================================
// CPU contention helpers
// ============================================================================

/// Spawn busy-loop threads on N cores to saturate the CPU.
/// Returns a stop flag that, when set to false, causes all threads to exit.
fn spawn_cpu_hogs(num_threads: usize) -> (Arc<AtomicBool>, Vec<std::thread::JoinHandle<()>>) {
    let running = Arc::new(AtomicBool::new(true));
    let mut handles = Vec::with_capacity(num_threads);

    for i in 0..num_threads {
        let flag = running.clone();
        let h = std::thread::Builder::new()
            .name(format!("cpu-hog-{}", i))
            .spawn(move || {
                while flag.load(Ordering::Relaxed) {
                    std::hint::spin_loop();
                }
            })
            .expect("failed to spawn CPU hog thread");
        handles.push(h);
    }

    (running, handles)
}

/// Compute jitter statistics from tick timestamps.
///
/// Measures jitter as deviation from the *mean observed interval*, which is
/// robust against rate aliasing. Also reports deviation from `expected_period`
/// for informational purposes.
fn compute_jitter_stats(timestamps: &[Instant], expected_period: Duration) -> JitterReport {
    if timestamps.len() < 2 {
        return JitterReport::default();
    }

    let mut intervals_us: Vec<f64> = Vec::with_capacity(timestamps.len() - 1);

    for pair in timestamps.windows(2) {
        let interval = pair[1].duration_since(pair[0]);
        let interval_us = interval.as_secs_f64() * 1_000_000.0;
        intervals_us.push(interval_us);
    }

    let n = intervals_us.len();
    let mean_interval_us: f64 = intervals_us.iter().sum::<f64>() / n as f64;
    let expected_us = expected_period.as_secs_f64() * 1_000_000.0;

    // Jitter = deviation from MEAN observed interval (robust against rate aliasing)
    let mut jitters_us: Vec<f64> = intervals_us
        .iter()
        .map(|&interval| (interval - mean_interval_us).abs())
        .collect();
    jitters_us.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let min = jitters_us[0];
    let max = jitters_us[n - 1];
    let p50 = jitters_us[n / 2];
    let p99 = jitters_us[std::cmp::min((n as f64 * 0.99) as usize, n - 1)];
    let mean_jitter: f64 = jitters_us.iter().sum::<f64>() / n as f64;

    // Mean absolute deviation, relative to the mean interval.
    //
    // This is a SPREAD statistic, not a tail statistic, and it is deliberately
    // no longer the only gate. Averaging hides excursions: one interval four
    // periods long moves this by 3 * period / n microseconds, which at n = 3000
    // is under a thousandth of a percent. Measured on an idle 12-core box, the
    // baseline test reported 5.92% here -- well inside its 10% limit -- while
    // the same report showed 3451.7 us max jitter and a 4474.9 us worst
    // interval on a 1000 us period. Four missed control cycles is precisely the
    // failure these tests exist to catch, so `check_timing` bounds the tail
    // separately via `max_jitter_periods` and `max_stall_rate`.
    let mad_percent = (mean_jitter / mean_interval_us) * 100.0;

    // Worst/best raw intervals
    let worst_interval = intervals_us.iter().cloned().fold(0.0_f64, f64::max);
    let best_interval = intervals_us.iter().cloned().fold(f64::INFINITY, f64::min);

    // Count intervals exceeding 3x the mean (severe outliers)
    let severe_outliers = intervals_us
        .iter()
        .filter(|&&i| i > mean_interval_us * 3.0)
        .count();

    JitterReport {
        total_ticks: timestamps.len() as u64,
        total_intervals: n as u64,
        mean_interval_us,
        expected_interval_us: expected_us,
        effective_rate_hz: 1_000_000.0 / mean_interval_us,
        min_jitter_us: min,
        max_jitter_us: max,
        p50_jitter_us: p50,
        p99_jitter_us: p99,
        mean_jitter_us: mean_jitter,
        mad_percent,
        worst_interval_us: worst_interval,
        best_interval_us: best_interval,
        severe_outliers: severe_outliers as u64,
    }
}

#[derive(Debug, Default)]
struct JitterReport {
    total_ticks: u64,
    total_intervals: u64,
    mean_interval_us: f64,
    expected_interval_us: f64,
    effective_rate_hz: f64,
    min_jitter_us: f64,
    max_jitter_us: f64,
    p50_jitter_us: f64,
    p99_jitter_us: f64,
    mean_jitter_us: f64,
    mad_percent: f64,
    worst_interval_us: f64,
    best_interval_us: f64,
    severe_outliers: u64,
}

impl JitterReport {
    /// Fraction of intervals longer than 3x the mean interval.
    fn stall_rate(&self) -> f64 {
        if self.total_intervals == 0 {
            0.0
        } else {
            self.severe_outliers as f64 / self.total_intervals as f64
        }
    }
}

/// Limits for one timing gate.
///
/// Tail limits are expressed in units of the node's declared period so they
/// stay meaningful if a test's rate changes.
struct TimingLimits {
    /// Mean-absolute-deviation over the mean interval, in percent. `None` where
    /// the environment makes spread uninformative (the contention tests).
    max_mad_percent: Option<f64>,
    /// Largest single deviation from the mean interval, in periods. This is the
    /// bound that spread cannot express: a lone stall of any size is invisible
    /// to `max_mad_percent` and trips this immediately.
    max_jitter_periods: f64,
    /// Fraction of intervals longer than 3x the mean interval.
    max_stall_rate: f64,
    /// Lowest acceptable effective rate, as a fraction of the declared rate.
    ///
    /// Jitter is measured against the *mean observed* interval, which makes it
    /// robust to rate aliasing but also blind to rate error: a node declared at
    /// 1 kHz that actually runs at 200 Hz has a beautiful jitter profile around
    /// its own wrong period. The tick-count asserts alone were far too loose to
    /// catch that -- `ticks >= 500` over 5 s permits 100 Hz from a 1 kHz node,
    /// a 10x rate collapse -- so state the rate as a fraction and let it fail.
    min_rate_fraction: f64,
}

/// Check a jitter report against `limits`, returning every violation.
///
/// Split out from the tests so the gate itself can be tested on synthetic
/// distributions -- see `mod gate_tests`. Without that, a gate that silently
/// stopped being able to fail would look exactly like a passing test.
fn check_timing(report: &JitterReport, limits: &TimingLimits) -> Result<(), String> {
    let period_us = report.expected_interval_us;
    let mut problems: Vec<String> = Vec::new();

    if let Some(limit) = limits.max_mad_percent {
        if report.mad_percent >= limit {
            problems.push(format!(
                "spread: MAD {:.2}% of the mean interval, limit {:.2}%",
                report.mad_percent, limit
            ));
        }
    }

    let jitter_periods = if period_us > 0.0 {
        report.max_jitter_us / period_us
    } else {
        0.0
    };
    if jitter_periods >= limits.max_jitter_periods {
        problems.push(format!(
            "tail: max jitter {:.1} us = {:.1} periods, limit {:.1} periods \
             (worst interval {:.1} us on a {:.0} us period)",
            report.max_jitter_us,
            jitter_periods,
            limits.max_jitter_periods,
            report.worst_interval_us,
            period_us
        ));
    }

    let stall_rate = report.stall_rate();
    if stall_rate >= limits.max_stall_rate {
        problems.push(format!(
            "stalls: {:.2}% of intervals exceeded 3x the mean ({} of {}), limit {:.2}%",
            stall_rate * 100.0,
            report.severe_outliers,
            report.total_intervals,
            limits.max_stall_rate * 100.0
        ));
    }

    let rate_fraction = if report.mean_interval_us > 0.0 {
        period_us / report.mean_interval_us
    } else {
        0.0
    };
    if rate_fraction < limits.min_rate_fraction {
        problems.push(format!(
            "rate: {:.1} Hz effective against {:.0} Hz declared ({:.0}% of it), \
             floor {:.0}%",
            report.effective_rate_hz,
            1_000_000.0 / period_us,
            rate_fraction * 100.0,
            limits.min_rate_fraction * 100.0
        ));
    }

    if problems.is_empty() {
        Ok(())
    } else {
        Err(problems.join("\n  "))
    }
}

/// Panic with the full report when `check_timing` fails.
fn assert_timing(report: &JitterReport, limits: &TimingLimits, context: &str) {
    if let Err(problems) = check_timing(report, limits) {
        panic!(
            "{context} timing gate failed:\n  {problems}\n\n{report}\n\
             If this is a fresh checkout rather than a regression, check the \
             environment first: SCHED_FIFO needs CAP_SYS_NICE (`horus setup-rt`), \
             this kernel may not be PREEMPT_RT, and a debug build is 10-50x \
             slower than release."
        );
    }
}

impl std::fmt::Display for JitterReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "=== RT Contention Jitter Report ===")?;
        writeln!(f, "Total ticks:       {}", self.total_ticks)?;
        writeln!(
            f,
            "Expected rate:     {:.0} Hz",
            1_000_000.0 / self.expected_interval_us
        )?;
        writeln!(f, "Effective rate:    {:.1} Hz", self.effective_rate_hz)?;
        writeln!(f, "Mean interval:     {:.1} us", self.mean_interval_us)?;
        writeln!(f, "Jitter (deviation from mean interval):")?;
        writeln!(f, "  min:   {:.1} us", self.min_jitter_us)?;
        writeln!(f, "  p50:   {:.1} us", self.p50_jitter_us)?;
        writeln!(f, "  p99:   {:.1} us", self.p99_jitter_us)?;
        writeln!(f, "  max:   {:.1} us", self.max_jitter_us)?;
        writeln!(f, "  mean:  {:.1} us", self.mean_jitter_us)?;
        writeln!(
            f,
            "  MAD:   {:.2}% of mean interval (spread only -- blind to tails)",
            self.mad_percent
        )?;
        writeln!(f, "Interval range:")?;
        writeln!(f, "  best:  {:.1} us", self.best_interval_us)?;
        writeln!(f, "  worst: {:.1} us", self.worst_interval_us)?;
        writeln!(f, "Severe outliers (>3x mean): {}", self.severe_outliers)?;
        writeln!(f, "=================================")
    }
}

// ============================================================================
// Tests
// ============================================================================

/// RT node at 1kHz with all available CPUs saturated by busy-loop threads.
///
/// Measures tick-to-tick jitter and verifies RT isolation keeps the node
/// on schedule despite heavy CPU contention on competing cores.
#[test]
fn stress_rt_1khz_under_cpu_contention() {
    let _shm_guard = cleanup_stale_shm();

    let tick_count = Arc::new(AtomicU64::new(0));
    let timestamps = Arc::new(Mutex::new(Vec::with_capacity(6000)));

    let num_cpus = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(4);
    let hog_count = num_cpus.saturating_sub(1).max(1);

    eprintln!(
        "[stress_rt_contention] CPUs: {}, CPU hog threads: {}",
        num_cpus, hog_count
    );

    // Start CPU hogs BEFORE the scheduler to ensure contention is active
    let (hog_running, hog_handles) = spawn_cpu_hogs(hog_count);
    std::thread::sleep(Duration::from_millis(100));

    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz()).verbose(false);

    let _ = scheduler
        .add(JitterNode {
            name: "rt_jitter_1khz".to_string(),
            tick_count: tick_count.clone(),
            timestamps: timestamps.clone(),
        })
        .rate(1000_u64.hz())
        .budget(800_u64.us())
        .deadline(2_u64.ms())
        .build();

    let result = scheduler.run_for(5_u64.secs());
    assert!(result.is_ok(), "Scheduler should complete without error");

    // Stop CPU hogs
    hog_running.store(false, Ordering::SeqCst);
    for h in hog_handles {
        let _ = h.join();
    }

    // Analyze jitter
    let ts = timestamps.lock().unwrap();
    let report = compute_jitter_stats(&ts, 1_u64.ms());
    eprintln!("{}", report);

    // Node should tick consistently (accounting for rate-limiter aliasing)
    let ticks = tick_count.load(Ordering::SeqCst);
    assert!(
        ticks >= 500,
        "Expected at least 500 ticks in 5s under contention, got {}",
        ticks
    );

    // Under CPU contention without SCHED_FIFO, the RT thread gets preempted
    // by busy-loop hog threads during spin-wait, causing jitter spikes.
    // This is expected on non-RT kernels. The important thing is:
    // 1. The RT node continues ticking (not starved)
    // 2. Jitter is REPORTED for documentation (the report above)
    // 3. The worst-case doesn't exceed 100ms (catastrophic scheduler stall)
    assert!(
        report.worst_interval_us < 100_000.0,
        "Worst interval {:.1}us exceeds 100ms — possible scheduler stall",
        report.worst_interval_us
    );

    // Spread is not gated here: with 11 hog threads it is actively
    // misleading. The same box measured MAD 3.94% under full contention
    // against 5.92% idle -- the contended run scored "better" while its max
    // jitter was 57% worse (5407.7 us vs 3451.7 us). Bound the tail instead.
    // Measured under contention: max jitter 5407.7 us, stalls 0.44%.
    assert_timing(
        &report,
        &TimingLimits {
            max_mad_percent: None,
            max_jitter_periods: 15.0,
            max_stall_rate: 0.05,
            min_rate_fraction: 0.40,
        },
        "1kHz under CPU contention",
    );
}

/// Baseline: same RT node at 1kHz WITHOUT CPU contention.
/// Establishes the timing floor for comparison and validates
/// that the jitter measurement infrastructure works correctly.
#[test]
fn stress_rt_1khz_baseline_no_contention() {
    let _shm_guard = cleanup_stale_shm();

    let tick_count = Arc::new(AtomicU64::new(0));
    let timestamps = Arc::new(Mutex::new(Vec::with_capacity(4000)));

    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz()).verbose(false);

    let _ = scheduler
        .add(JitterNode {
            name: "rt_baseline_1khz".to_string(),
            tick_count: tick_count.clone(),
            timestamps: timestamps.clone(),
        })
        .rate(1000_u64.hz())
        .budget(800_u64.us())
        .deadline(2_u64.ms())
        .build();

    let result = scheduler.run_for(3_u64.secs());
    assert!(result.is_ok(), "Baseline scheduler should complete");

    let ts = timestamps.lock().unwrap();
    let report = compute_jitter_stats(&ts, 1_u64.ms());
    eprintln!("{}", report);

    let ticks = tick_count.load(Ordering::SeqCst);
    assert!(
        ticks >= 500,
        "Expected at least 500 ticks in 3s at 1kHz, got {}",
        ticks
    );

    // Baseline should have very consistent timing.
    //
    // Limits below are measured, not aspirational: five runs on an idle
    // 12-core non-PREEMPT_RT box (debug build, no SCHED_FIFO) gave MAD
    // 4.80-7.18%, max jitter 2873.6-3551.4 us, and stalls 0.24-0.58%. The
    // tail bound is set at 8 periods, roughly 2.3x the worst observed max,
    // so it passes that environment while still failing any single stall
    // past 8 ms -- the class of regression (a lock, an allocation, a syscall
    // on the tick path) that the spread number cannot see at all.
    assert_timing(
        &report,
        &TimingLimits {
            max_mad_percent: Some(10.0),
            max_jitter_periods: 8.0,
            max_stall_rate: 0.02,
            min_rate_fraction: 0.40,
        },
        "baseline (no contention)",
    );
}

/// Multiple RT nodes at different rates under heavy contention.
/// Validates the RT executor handles multi-rate scheduling correctly
/// even with saturated CPUs.
#[test]
fn stress_rt_multi_rate_under_contention() {
    let _shm_guard = cleanup_stale_shm();

    let count_1khz = Arc::new(AtomicU64::new(0));
    let count_500hz = Arc::new(AtomicU64::new(0));
    let count_100hz = Arc::new(AtomicU64::new(0));
    let ts_1khz = Arc::new(Mutex::new(Vec::with_capacity(6000)));

    let num_cpus = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(4);
    let hog_count = num_cpus.saturating_sub(1).max(1);

    let (hog_running, hog_handles) = spawn_cpu_hogs(hog_count);
    std::thread::sleep(Duration::from_millis(100));

    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz()).verbose(false);

    let _ = scheduler
        .add(JitterNode {
            name: "rt_1khz".to_string(),
            tick_count: count_1khz.clone(),
            timestamps: ts_1khz.clone(),
        })
        .rate(1000_u64.hz())
        .order(0)
        .build();

    let _ = scheduler
        .add(JitterNode {
            name: "rt_500hz".to_string(),
            tick_count: count_500hz.clone(),
            timestamps: Arc::new(Mutex::new(Vec::new())),
        })
        .rate(500_u64.hz())
        .order(1)
        .build();

    let _ = scheduler
        .add(JitterNode {
            name: "rt_100hz".to_string(),
            tick_count: count_100hz.clone(),
            timestamps: Arc::new(Mutex::new(Vec::new())),
        })
        .rate(100_u64.hz())
        .order(2)
        .build();

    let result = scheduler.run_for(5_u64.secs());
    assert!(result.is_ok(), "Multi-rate scheduler should complete");

    hog_running.store(false, Ordering::SeqCst);
    for h in hog_handles {
        let _ = h.join();
    }

    let ticks_1k = count_1khz.load(Ordering::SeqCst);
    let ticks_500 = count_500hz.load(Ordering::SeqCst);
    let ticks_100 = count_100hz.load(Ordering::SeqCst);

    eprintln!(
        "[multi-rate] 1kHz: {} ticks, 500Hz: {} ticks, 100Hz: {} ticks",
        ticks_1k, ticks_500, ticks_100
    );

    // All nodes should tick (accounting for rate aliasing on non-RT kernels)
    assert!(ticks_1k >= 500, "1kHz node should tick (got {})", ticks_1k);
    assert!(
        ticks_500 >= 250,
        "500Hz node should tick (got {})",
        ticks_500
    );
    assert!(
        ticks_100 >= 100,
        "100Hz node should tick (got {})",
        ticks_100
    );

    // Rate ordering: faster nodes should tick more than slower ones
    assert!(
        ticks_1k > ticks_500,
        "1kHz ({}) should tick more than 500Hz ({})",
        ticks_1k,
        ticks_500
    );
    assert!(
        ticks_500 > ticks_100,
        "500Hz ({}) should tick more than 100Hz ({})",
        ticks_500,
        ticks_100
    );

    // Jitter consistency for the fastest node
    let ts = ts_1khz.lock().unwrap();
    let report = compute_jitter_stats(&ts, 1_u64.ms());
    eprintln!("1kHz node jitter:\n{}", report);

    // Under contention, spread can be high due to OS preemption of the
    // spin-wait. Assert stability, not tight timing -- but assert it against
    // the period rather than only the 100 ms catastrophic-stall floor, which
    // a 1 kHz loop can miss a hundred deadlines under and still pass.
    // Measured here: max jitter 10342.3 us, stalls 1.50%.
    assert!(
        report.worst_interval_us < 100_000.0,
        "Worst interval {:.1}us exceeds 100ms — possible scheduler stall",
        report.worst_interval_us
    );
    assert_timing(
        &report,
        &TimingLimits {
            max_mad_percent: None,
            max_jitter_periods: 25.0,
            max_stall_rate: 0.05,
            min_rate_fraction: 0.40,
        },
        "1kHz node in multi-rate mix",
    );
}

/// RT node with compute-class competitors: RT thread should not be starved
/// by heavy compute nodes on the main thread.
#[test]
fn stress_rt_isolation_from_compute_nodes() {
    let _shm_guard = cleanup_stale_shm();

    let rt_count = Arc::new(AtomicU64::new(0));
    let rt_ts = Arc::new(Mutex::new(Vec::with_capacity(6000)));
    let compute_counts: Vec<Arc<AtomicU64>> = (0..8).map(|_| Arc::new(AtomicU64::new(0))).collect();

    /// Compute node that does expensive work each tick.
    struct HeavyComputeNode {
        name: String,
        tick_count: Arc<AtomicU64>,
    }

    impl Node for HeavyComputeNode {
        fn name(&self) -> &'static str {
            Box::leak(self.name.clone().into_boxed_str())
        }
        fn tick(&mut self) {
            self.tick_count.fetch_add(1, Ordering::Relaxed);
            // Simulate 2ms of compute work
            let start = Instant::now();
            while start.elapsed() < Duration::from_millis(2) {
                std::hint::spin_loop();
            }
        }
    }

    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz()).verbose(false);

    // RT node at 1kHz
    let _ = scheduler
        .add(JitterNode {
            name: "rt_isolated".to_string(),
            tick_count: rt_count.clone(),
            timestamps: rt_ts.clone(),
        })
        .rate(1000_u64.hz())
        .order(0)
        .build();

    // 8 heavy compute nodes (each burns 2ms per tick)
    for (i, count) in compute_counts.iter().enumerate() {
        let _ = scheduler
            .add(HeavyComputeNode {
                name: format!("heavy_compute_{}", i),
                tick_count: count.clone(),
            })
            .compute()
            .build();
    }

    let result = scheduler.run_for(5_u64.secs());
    assert!(
        result.is_ok(),
        "Scheduler with RT + compute should complete"
    );

    let rt_ticks = rt_count.load(Ordering::SeqCst);
    eprintln!("[rt_isolation] RT ticks: {}", rt_ticks);

    // RT node runs on dedicated thread — should tick regardless of compute load
    assert!(
        rt_ticks >= 500,
        "RT node should get >=500 ticks in 5s despite 8 heavy compute nodes, got {}",
        rt_ticks
    );

    let ts = rt_ts.lock().unwrap();
    let report = compute_jitter_stats(&ts, 1_u64.ms());
    eprintln!("RT isolation jitter:\n{}", report);

    // RT thread is independent of compute — timing should stay consistent.
    // Measured with 8 heavy compute nodes running: MAD 12.42%, max jitter
    // 3420.2 us, stalls 1.02%.
    assert_timing(
        &report,
        &TimingLimits {
            max_mad_percent: Some(15.0),
            max_jitter_periods: 10.0,
            max_stall_rate: 0.02,
            min_rate_fraction: 0.40,
        },
        "RT node isolated from compute nodes",
    );
}

// ============================================================================
// Gate tests
//
// The gate these stress tests hang on used to be a single spread number, and a
// spread number cannot fail on a tail. That is not a hypothetical: on an idle
// 12-core box the baseline test passed at "CV 5.92%" while the very same report
// printed 3451.7 us max jitter and a 4474.9 us worst interval on a 1000 us
// period. The tests below feed `check_timing` synthetic distributions so the
// gate's reach is pinned deterministically, rather than being inferred from
// whatever the host machine happened to do that afternoon.
// ============================================================================

/// Build a timestamp series from explicit inter-tick intervals.
fn synth(intervals_us: &[f64]) -> Vec<Instant> {
    let base = Instant::now();
    let mut out = Vec::with_capacity(intervals_us.len() + 1);
    let mut acc = Duration::ZERO;
    out.push(base);
    for &us in intervals_us {
        acc += Duration::from_nanos((us * 1000.0) as u64);
        out.push(base + acc);
    }
    out
}

fn spread_only(limit: f64) -> TimingLimits {
    TimingLimits {
        max_mad_percent: Some(limit),
        max_jitter_periods: f64::INFINITY,
        max_stall_rate: 1.0,
        min_rate_fraction: 0.0,
    }
}

/// The regression that motivated the tail gate: 2999 perfect 1 ms intervals and
/// one 20 ms stall. Twenty missed deadlines in a row is a control-loop failure
/// in any robot, and the spread gate rates the run at 1.3%.
#[test]
fn spread_gate_alone_cannot_see_a_single_long_stall() {
    let mut intervals = vec![1000.0; 2999];
    intervals.push(20_000.0);
    let report = compute_jitter_stats(&synth(&intervals), Duration::from_millis(1));

    assert!(
        report.max_jitter_us > 18_000.0,
        "expected the 20 ms stall to show as max jitter, got {:.1} us",
        report.max_jitter_us
    );

    // The historical gate. It does not merely pass — it passes by 8x.
    assert!(
        report.mad_percent < 10.0,
        "MAD was {:.2}%, expected it to sail under the old 10% limit",
        report.mad_percent
    );
    assert!(
        check_timing(&report, &spread_only(10.0)).is_ok(),
        "spread-only gate should be blind to this; if it now fails, the \
         demonstration below no longer demonstrates anything"
    );

    // The gate the stress tests actually use now.
    let err = check_timing(
        &report,
        &TimingLimits {
            max_mad_percent: Some(10.0),
            max_jitter_periods: 8.0,
            max_stall_rate: 0.02,
            min_rate_fraction: 0.40,
        },
    )
    .expect_err("a 20 ms stall on a 1 ms period must fail the gate");
    assert!(
        err.contains("tail:"),
        "expected the tail limit to fire: {err}"
    );
    assert!(
        !err.contains("spread:"),
        "spread must NOT be what caught this — that is the whole point: {err}"
    );
}

/// Broad degradation with no single dramatic outlier: 100 intervals at 6 ms
/// among 2900 at 1 ms. Under 5 periods of jitter, so the tail bound stays
/// quiet; the stall rate is what catches it.
#[test]
fn stall_rate_catches_degradation_below_the_tail_bound() {
    let mut intervals = vec![1000.0; 2900];
    intervals.resize(3000, 6000.0);
    let report = compute_jitter_stats(&synth(&intervals), Duration::from_millis(1));

    assert!(
        report.max_jitter_us / report.expected_interval_us < 8.0,
        "this case is only interesting if the tail bound stays quiet, but \
         max jitter was {:.1} us",
        report.max_jitter_us
    );

    let err = check_timing(
        &report,
        &TimingLimits {
            max_mad_percent: None,
            max_jitter_periods: 8.0,
            max_stall_rate: 0.02,
            min_rate_fraction: 0.40,
        },
    )
    .expect_err("3.3% of intervals at 6x the period must fail the gate");
    assert!(
        err.contains("stalls:"),
        "expected the stall-rate limit to fire: {err}"
    );
    assert!(
        !err.contains("tail:"),
        "tail must not be what caught this: {err}"
    );
}

/// The spread limit is kept, not replaced — a genuinely jittery run still trips
/// it even when no single interval is long enough to reach the tail bound.
#[test]
fn spread_gate_still_fires_on_broadly_noisy_timing() {
    let intervals: Vec<f64> = (0..3000)
        .map(|i| if i % 2 == 0 { 500.0 } else { 1500.0 })
        .collect();
    let report = compute_jitter_stats(&synth(&intervals), Duration::from_millis(1));

    let err = check_timing(
        &report,
        &TimingLimits {
            max_mad_percent: Some(10.0),
            max_jitter_periods: 8.0,
            max_stall_rate: 0.02,
            min_rate_fraction: 0.40,
        },
    )
    .expect_err("+/-50% alternating intervals must fail the spread limit");
    assert!(
        err.contains("spread:"),
        "expected the spread limit to fire: {err}"
    );
}

/// A clean run passes every limit. Without this, all three tests above would
/// still pass if `check_timing` simply always returned `Err`.
#[test]
fn clean_timing_passes_every_limit() {
    let intervals: Vec<f64> = (0..3000)
        .map(|i| 1000.0 + ((i % 7) as f64 - 3.0) * 2.0)
        .collect();
    let report = compute_jitter_stats(&synth(&intervals), Duration::from_millis(1));

    check_timing(
        &report,
        &TimingLimits {
            max_mad_percent: Some(10.0),
            max_jitter_periods: 8.0,
            max_stall_rate: 0.02,
            min_rate_fraction: 0.40,
        },
    )
    .expect("a run within +/-6 us of a 1 ms period must pass");
}

/// A node running at a fifth of its declared rate has excellent jitter — around
/// its own wrong period. Neither spread nor tail nor stalls can see that, and
/// `ticks >= 500` over 5 s cannot either: a 1 kHz node delivering 100 Hz clears
/// it. Only the rate floor does.
#[test]
fn rate_floor_catches_a_node_running_at_the_wrong_rate() {
    // 200 Hz, metronomically. Every interval identical, so spread is zero.
    let intervals = vec![5000.0; 3000];
    let report = compute_jitter_stats(&synth(&intervals), Duration::from_millis(1));

    assert!(
        report.mad_percent < 0.01,
        "this case is only interesting if the timing looks flawless, but MAD \
         was {:.4}%",
        report.mad_percent
    );
    assert!(
        report.max_jitter_us < 1.0,
        "and if the tail looks flawless, but max jitter was {:.1} us",
        report.max_jitter_us
    );

    let err = check_timing(
        &report,
        &TimingLimits {
            max_mad_percent: Some(10.0),
            max_jitter_periods: 8.0,
            max_stall_rate: 0.02,
            min_rate_fraction: 0.40,
        },
    )
    .expect_err("a 1 kHz node delivering 200 Hz must fail the gate");
    assert!(
        err.contains("rate:"),
        "expected the rate floor to fire: {err}"
    );
    assert!(
        !err.contains("spread:") && !err.contains("tail:") && !err.contains("stalls:"),
        "nothing but the rate floor can see this: {err}"
    );
}
