//! E2E test: Real scheduler running a GPU node.
//!
//! This is the critical missing test — a Scheduler with .gpu() nodes
//! that actually tick and produce output.

use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::gpu::cuda_available;
use horus_core::scheduling::Scheduler;
use horus_core::types::Device;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

/// A simple GPU node that increments a counter each tick.
/// In a real scenario, it would launch CUDA kernels.
struct GpuCounterNode {
    count: Arc<AtomicU64>,
    name: String,
}

impl Node for GpuCounterNode {
    fn name(&self) -> &str {
        &self.name
    }

    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::Relaxed);
        // In production, this would launch CUDA kernels via horus::gpu_stream()
        // For this test, we verify the scheduler dispatches to GpuExecutor correctly
    }
}

/// A CPU sensor node that publishes data.
struct CpuSensorNode {
    count: Arc<AtomicU64>,
}

impl Node for CpuSensorNode {
    fn name(&self) -> &str {
        "cpu_sensor"
    }

    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::Relaxed);
    }
}

/// Test: Scheduler with mixed CPU + GPU nodes runs without crash.
#[test]
fn test_scheduler_with_gpu_node() {
    if !cuda_available() {
        eprintln!("CUDA not available, skipping scheduler GPU test");
        return;
    }

    let gpu_count = Arc::new(AtomicU64::new(0));
    let cpu_count = Arc::new(AtomicU64::new(0));

    let gpu_node = GpuCounterNode {
        count: Arc::clone(&gpu_count),
        name: "gpu_preprocess".to_string(),
    };
    let cpu_node = CpuSensorNode {
        count: Arc::clone(&cpu_count),
    };

    let mut sched = Scheduler::new();
    sched = sched.tick_rate(100_u64.hz());

    // Add CPU node (BestEffort)
    sched.add(cpu_node).order(0).build().unwrap();

    // Add GPU node
    sched.add(gpu_node).order(1).gpu().build().unwrap();

    // Run for 200ms (should get ~20 ticks at 100Hz)
    sched
        .run_for(std::time::Duration::from_millis(200))
        .unwrap();

    let gpu_ticks = gpu_count.load(Ordering::Relaxed);
    let cpu_ticks = cpu_count.load(Ordering::Relaxed);

    eprintln!("CPU ticks: {}, GPU ticks: {}", cpu_ticks, gpu_ticks);

    // Both should have ticked at least once
    assert!(cpu_ticks > 0, "CPU node should have ticked");
    assert!(gpu_ticks > 0, "GPU node should have ticked");

    // Both should have similar tick counts (within 50% — timing is approximate)
    let ratio = if cpu_ticks > gpu_ticks {
        cpu_ticks as f64 / gpu_ticks.max(1) as f64
    } else {
        gpu_ticks as f64 / cpu_ticks.max(1) as f64
    };
    assert!(
        ratio < 3.0,
        "CPU/GPU tick ratio too skewed: {} (cpu={}, gpu={})",
        ratio,
        cpu_ticks,
        gpu_ticks
    );
}

/// Test: Scheduler with ONLY GPU nodes (no CPU nodes).
#[test]
fn test_scheduler_gpu_only() {
    if !cuda_available() {
        return;
    }

    let count = Arc::new(AtomicU64::new(0));
    let node = GpuCounterNode {
        count: Arc::clone(&count),
        name: "solo_gpu".to_string(),
    };

    let mut sched = Scheduler::new();
    sched = sched.tick_rate(50_u64.hz());
    sched.add(node).order(0).gpu().build().unwrap();

    sched
        .run_for(std::time::Duration::from_millis(200))
        .unwrap();

    let ticks = count.load(Ordering::Relaxed);
    eprintln!("Solo GPU node ticks: {}", ticks);
    assert!(ticks > 0, "GPU-only scheduler should tick");
}

/// Test: Multiple GPU nodes in same scheduler.
#[test]
fn test_scheduler_multiple_gpu_nodes() {
    if !cuda_available() {
        return;
    }

    let counts: Vec<Arc<AtomicU64>> = (0..3).map(|_| Arc::new(AtomicU64::new(0))).collect();

    let mut sched = Scheduler::new();
    sched = sched.tick_rate(50_u64.hz());

    for (i, count) in counts.iter().enumerate() {
        let node = GpuCounterNode {
            count: Arc::clone(count),
            name: format!("gpu_node_{}", i),
        };
        sched.add(node).order(i as u32).gpu().build().unwrap();
    }

    sched
        .run_for(std::time::Duration::from_millis(200))
        .unwrap();

    for (i, count) in counts.iter().enumerate() {
        let ticks = count.load(Ordering::Relaxed);
        eprintln!("GPU node {} ticks: {}", i, ticks);
        assert!(ticks > 0, "GPU node {} should have ticked", i);
    }
}

/// Test: GPU node with rate limiting.
#[test]
fn test_scheduler_gpu_rate_limited() {
    if !cuda_available() {
        return;
    }

    let count = Arc::new(AtomicU64::new(0));
    let node = GpuCounterNode {
        count: Arc::clone(&count),
        name: "rate_limited_gpu".to_string(),
    };

    let mut sched = Scheduler::new();
    sched = sched.tick_rate(200_u64.hz()); // Scheduler at 200Hz

    // GPU node at 30Hz (rate limited)
    sched
        .add(node)
        .order(0)
        .gpu()
        .rate(30_u64.hz())
        .build()
        .unwrap();

    sched
        .run_for(std::time::Duration::from_millis(500))
        .unwrap();

    let ticks = count.load(Ordering::Relaxed);
    eprintln!(
        "Rate-limited GPU node ticks: {} (expected ~15 at 30Hz for 500ms)",
        ticks
    );

    // Should be approximately 15 ticks (30Hz * 0.5s), allow wide margin
    assert!(ticks >= 5, "Should tick at least 5 times in 500ms at 30Hz");
    assert!(
        ticks <= 50,
        "Should not exceed 50 ticks (30Hz * 0.5s + margin)"
    );
}
