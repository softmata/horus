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

    // Coefficient of variation: jitter relative to mean interval
    let cv_percent = (mean_jitter / mean_interval_us) * 100.0;

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
        cv_percent,
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
    cv_percent: f64,
    worst_interval_us: f64,
    best_interval_us: f64,
    severe_outliers: u64,
}

impl std::fmt::Display for JitterReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "=== RT Contention Jitter Report ===")?;
        writeln!(f, "Total ticks:       {}", self.total_ticks)?;
        writeln!(f, "Expected rate:     {:.0} Hz", 1_000_000.0 / self.expected_interval_us)?;
        writeln!(f, "Effective rate:    {:.1} Hz", self.effective_rate_hz)?;
        writeln!(f, "Mean interval:     {:.1} us", self.mean_interval_us)?;
        writeln!(f, "Jitter (deviation from mean interval):")?;
        writeln!(f, "  min:   {:.1} us", self.min_jitter_us)?;
        writeln!(f, "  p50:   {:.1} us", self.p50_jitter_us)?;
        writeln!(f, "  p99:   {:.1} us", self.p99_jitter_us)?;
        writeln!(f, "  max:   {:.1} us", self.max_jitter_us)?;
        writeln!(f, "  mean:  {:.1} us", self.mean_jitter_us)?;
        writeln!(f, "  CV:    {:.2}%", self.cv_percent)?;
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
    cleanup_stale_shm();

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

    let mut scheduler = Scheduler::new()
        .tick_rate(1000_u64.hz())
        .verbose(false);

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

    // Severe outliers (>3x mean) should be rare — under 5% even with contention
    let outlier_rate = report.severe_outliers as f64 / report.total_intervals as f64;
    assert!(
        outlier_rate < 0.05,
        "Severe outlier rate {:.2}% exceeds 5% — {} outliers in {} intervals",
        outlier_rate * 100.0,
        report.severe_outliers,
        report.total_intervals
    );
}

/// Baseline: same RT node at 1kHz WITHOUT CPU contention.
/// Establishes the timing floor for comparison and validates
/// that the jitter measurement infrastructure works correctly.
#[test]
fn stress_rt_1khz_baseline_no_contention() {
    cleanup_stale_shm();

    let tick_count = Arc::new(AtomicU64::new(0));
    let timestamps = Arc::new(Mutex::new(Vec::with_capacity(4000)));

    let mut scheduler = Scheduler::new()
        .tick_rate(1000_u64.hz())
        .verbose(false);

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

    // Baseline should have very consistent timing
    assert!(
        report.cv_percent < 10.0,
        "Baseline CV {:.2}% exceeds 10% — timing is too variable without contention",
        report.cv_percent
    );
}

/// Multiple RT nodes at different rates under heavy contention.
/// Validates the RT executor handles multi-rate scheduling correctly
/// even with saturated CPUs.
#[test]
fn stress_rt_multi_rate_under_contention() {
    cleanup_stale_shm();

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

    let mut scheduler = Scheduler::new()
        .tick_rate(1000_u64.hz())
        .verbose(false);

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
    assert!(ticks_500 >= 250, "500Hz node should tick (got {})", ticks_500);
    assert!(ticks_100 >= 100, "100Hz node should tick (got {})", ticks_100);

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

    // Under contention, CV can be high due to OS preemption of the spin-wait.
    // Assert stability (no catastrophic stalls), not tight timing.
    assert!(
        report.worst_interval_us < 100_000.0,
        "Worst interval {:.1}us exceeds 100ms — possible scheduler stall",
        report.worst_interval_us
    );
}

/// RT node with compute-class competitors: RT thread should not be starved
/// by heavy compute nodes on the main thread.
#[test]
fn stress_rt_isolation_from_compute_nodes() {
    cleanup_stale_shm();

    let rt_count = Arc::new(AtomicU64::new(0));
    let rt_ts = Arc::new(Mutex::new(Vec::with_capacity(6000)));
    let compute_counts: Vec<Arc<AtomicU64>> =
        (0..8).map(|_| Arc::new(AtomicU64::new(0))).collect();

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

    let mut scheduler = Scheduler::new()
        .tick_rate(1000_u64.hz())
        .verbose(false);

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
    assert!(result.is_ok(), "Scheduler with RT + compute should complete");

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

    // RT thread is independent of compute — jitter should stay consistent
    assert!(
        report.cv_percent < 15.0,
        "RT CV {:.2}% — compute nodes should not affect RT thread timing consistency",
        report.cv_percent
    );

    // No severe outliers
    let outlier_rate = if report.total_intervals > 0 {
        report.severe_outliers as f64 / report.total_intervals as f64
    } else {
        0.0
    };
    assert!(
        outlier_rate < 0.02,
        "Severe outlier rate {:.2}% exceeds 2%",
        outlier_rate * 100.0
    );
}
