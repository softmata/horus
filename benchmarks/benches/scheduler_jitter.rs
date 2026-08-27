//! Scheduler RT Jitter Benchmarks
//!
//! Measures RT node timing accuracy using the node-declared API
//! (Scheduler::new() with RT nodes) under heavy compute load.
//!
//! Run with: cargo bench -- scheduler_jitter

use criterion::{criterion_group, criterion_main, Criterion};
use horus_core::core::{DurationExt, Node};
use horus_core::error::HorusResult as Result;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

// ============================================================================
// Benchmark Nodes
// ============================================================================

// `name` is a `&'static str`, not a `String`. Both nodes here used to store a
// `String` and implement `Node::name` as
// `Box::leak(self.name.clone().into_boxed_str())`, which allocated and leaked a
// fresh string every time the scheduler asked a node for its name — once per
// tick, inside the loop these benchmarks measure. `Node::name` returns `&str`,
// so the leak was never required, and every construction site passes a literal.
struct JitterMeasureNode {
    name: &'static str,
    tick_count: Arc<AtomicU64>,
    timestamps: Arc<std::sync::Mutex<Vec<Instant>>>,
}

impl JitterMeasureNode {
    fn new(name: &'static str) -> (Self, Arc<AtomicU64>, Arc<std::sync::Mutex<Vec<Instant>>>) {
        let tick_count = Arc::new(AtomicU64::new(0));
        let timestamps = Arc::new(std::sync::Mutex::new(Vec::with_capacity(2048)));
        (
            Self {
                name,
                tick_count: tick_count.clone(),
                timestamps: timestamps.clone(),
            },
            tick_count,
            timestamps,
        )
    }
}

impl Node for JitterMeasureNode {
    fn name(&self) -> &str {
        self.name
    }
    fn init(&mut self) -> Result<()> {
        Ok(())
    }
    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::Relaxed);
        self.timestamps.lock().unwrap().push(Instant::now());
    }
}

struct HeavyComputeNode {
    name: &'static str,
    work_us: u64,
}

impl HeavyComputeNode {
    fn new(name: &'static str, work_us: u64) -> Self {
        Self { name, work_us }
    }
}

impl Node for HeavyComputeNode {
    fn name(&self) -> &str {
        self.name
    }
    fn init(&mut self) -> Result<()> {
        Ok(())
    }
    fn tick(&mut self) {
        // Simulate CPU-bound work by busy-waiting
        let start = Instant::now();
        while start.elapsed().as_micros() < self.work_us as u128 {
            std::hint::spin_loop();
        }
    }
}

// ============================================================================
// Jitter Analysis
// ============================================================================

struct JitterStats {
    tick_count: u64,
    mean_interval_us: f64,
    max_interval_us: f64,
    p99_interval_us: f64,
    jitter_us: f64, // max - min
}

fn analyze_jitter(timestamps: &[Instant]) -> Option<JitterStats> {
    if timestamps.len() < 3 {
        return None;
    }

    let mut intervals_us: Vec<f64> = timestamps
        .windows(2)
        .map(|w| w[1].duration_since(w[0]).as_micros() as f64)
        .collect();

    intervals_us.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let min = intervals_us[0];
    let max = *intervals_us.last().unwrap();
    let mean = intervals_us.iter().sum::<f64>() / intervals_us.len() as f64;
    let p99_idx = (intervals_us.len() as f64 * 0.99) as usize;
    let p99 = intervals_us[p99_idx.min(intervals_us.len() - 1)];

    Some(JitterStats {
        tick_count: timestamps.len() as u64,
        mean_interval_us: mean,
        max_interval_us: max,
        p99_interval_us: p99,
        jitter_us: max - min,
    })
}

/// The p99 tick interval, as the `Duration` `iter_custom` wants back.
///
/// The failure sentinel is a large finite value, not `f64::MAX`: `f64::MAX as
/// u64` saturates and would have made a run that collected too few timestamps
/// indistinguishable from one that collected none.
fn measured_p99(stats: Option<JitterStats>) -> Duration {
    match stats {
        Some(s) => Duration::from_nanos((s.p99_interval_us * 1000.0) as u64),
        None => Duration::from_secs(1),
    }
}

// ============================================================================
// Benchmarks
// ============================================================================

fn bench_new_api_rt_under_compute_load(c: &mut Criterion) {
    let mut group = c.benchmark_group("scheduler_rt_jitter");
    group.measurement_time(5_u64.secs());
    group.sample_size(10);

    // `iter_custom`, not `iter`.
    //
    // These two used to call `b.iter(|| { ...; run_for(200ms); stats.p99 })`.
    // Criterion times the closure and does nothing with its return value, so
    // the reported number was the wall-clock duration of the whole closure —
    // two `remove_dir_all`s, three node constructions, the scheduler build and
    // a `run_for` that pins the total at a constant 200 ms by definition. Both
    // variants therefore reported ~200 ms with near-zero variance and were
    // indistinguishable however badly the RT node was jittering; the
    // `analyze_jitter` result, the only quantity of interest, was thrown away.
    //
    // `iter_custom` hands back a `Duration` covering `iters` executions, which
    // Criterion divides by `iters`. Only the measured p99 interval is added to
    // the total, so setup and teardown no longer contaminate the number.
    group.bench_function("new_api_rt_500hz_with_compute_load", |b| {
        b.iter_custom(|iters| {
            let mut total = Duration::ZERO;
            for _ in 0..iters {
                let _ = std::fs::remove_dir_all(horus_core::memory::shm_base_dir().join("topics"));
                let _ = std::fs::remove_dir_all(horus_core::memory::shm_base_dir().join("nodes"));

                let (rt_node, _count, timestamps) = JitterMeasureNode::new("bench_rt");
                let compute_a = HeavyComputeNode::new("bench_compute_a", 5000); // 5ms work
                let compute_b = HeavyComputeNode::new("bench_compute_b", 5000);

                let mut scheduler = Scheduler::new().tick_rate(500_u64.hz());
                let _ = scheduler.add(rt_node).order(0).rate(500_u64.hz()).build();
                let _ = scheduler.add(compute_a).order(10).compute().build();
                let _ = scheduler.add(compute_b).order(11).compute().build();

                scheduler.run_for(200_u64.ms()).unwrap();

                let ts = timestamps.lock().unwrap();
                total += measured_p99(analyze_jitter(&ts));
            }
            total
        });
    });

    group.bench_function("new_api_rt_500hz_no_load", |b| {
        b.iter_custom(|iters| {
            let mut total = Duration::ZERO;
            for _ in 0..iters {
                let _ = std::fs::remove_dir_all(horus_core::memory::shm_base_dir().join("topics"));
                let _ = std::fs::remove_dir_all(horus_core::memory::shm_base_dir().join("nodes"));

                let (rt_node, _count, timestamps) = JitterMeasureNode::new("bench_rt_solo");

                let mut scheduler = Scheduler::new().tick_rate(500_u64.hz());
                let _ = scheduler.add(rt_node).order(0).rate(500_u64.hz()).build();

                scheduler.run_for(200_u64.ms()).unwrap();

                let ts = timestamps.lock().unwrap();
                total += measured_p99(analyze_jitter(&ts));
            }
            total
        });
    });

    group.finish();
}

fn bench_jitter_report(c: &mut Criterion) {
    let mut group = c.benchmark_group("scheduler_jitter_report");
    group.measurement_time(3_u64.secs());
    group.sample_size(10);

    group.bench_function("rt_isolation_proof", |b| {
        b.iter(|| {
            let _ = std::fs::remove_dir_all(horus_core::memory::shm_base_dir().join("topics"));
            let _ = std::fs::remove_dir_all(horus_core::memory::shm_base_dir().join("nodes"));

            let (rt_node, rt_count, rt_timestamps) = JitterMeasureNode::new("proof_rt");
            let slow = HeavyComputeNode::new("proof_slow", 50_000); // 50ms blocking work

            let mut scheduler = Scheduler::new().tick_rate(500_u64.hz());
            let _ = scheduler.add(rt_node).order(0).rate(500_u64.hz()).build();
            let _ = scheduler.add(slow).order(10).compute().rate(10_u64.hz()).build();

            scheduler.run_for(500_u64.ms()).unwrap();

            let ticks = rt_count.load(Ordering::Relaxed);
            let ts = rt_timestamps.lock().unwrap();

            if let Some(stats) = analyze_jitter(&ts) {
                eprintln!(
                    "  RT isolation: ticks={}, mean={:.0}us, p99={:.0}us, max={:.0}us, jitter={:.0}us",
                    stats.tick_count,
                    stats.mean_interval_us,
                    stats.p99_interval_us,
                    stats.max_interval_us,
                    stats.jitter_us,
                );
                // Key assertion: if RT was blocked by 50ms compute, max would be >50000us
                assert!(
                    stats.max_interval_us < 20_000.0,
                    "RT max interval {:.0}us should be < 20ms (proving isolation from 50ms compute)",
                    stats.max_interval_us
                );
            }

            ticks
        });
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_new_api_rt_under_compute_load,
    bench_jitter_report,
);
criterion_main!(benches);
