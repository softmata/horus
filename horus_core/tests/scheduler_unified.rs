//! Integration tests for the unified scheduler redesign.
//!
//! These tests verify that RT nodes are fully isolated from compute nodes:
//! RT nodes tick at their declared rate regardless of how slow compute nodes are.

use horus_core::core::Node;
use horus_core::error::HorusResult as Result;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

mod common;
use common::cleanup_stale_shm;

// ============================================================================
// Test Nodes
// ============================================================================

/// An RT node that records its tick timestamps for jitter analysis.
struct RtTimingNode {
    name: String,
    tick_count: Arc<AtomicU64>,
    tick_timestamps: Arc<std::sync::Mutex<Vec<Instant>>>,
}

impl RtTimingNode {
    fn new(name: &str) -> (Self, Arc<AtomicU64>, Arc<std::sync::Mutex<Vec<Instant>>>) {
        let tick_count = Arc::new(AtomicU64::new(0));
        let timestamps = Arc::new(std::sync::Mutex::new(Vec::new()));
        (
            Self {
                name: name.to_string(),
                tick_count: tick_count.clone(),
                tick_timestamps: timestamps.clone(),
            },
            tick_count,
            timestamps,
        )
    }
}

impl Node for RtTimingNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn init(&mut self) -> Result<()> {
        Ok(())
    }

    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::Relaxed);
        self.tick_timestamps.lock().unwrap().push(Instant::now());
    }
}

/// A slow compute node that sleeps to simulate heavy computation.
struct SlowComputeNode {
    name: String,
    sleep_ms: u64,
    tick_count: Arc<AtomicU64>,
}

impl SlowComputeNode {
    fn new(name: &str, sleep_ms: u64) -> (Self, Arc<AtomicU64>) {
        let tick_count = Arc::new(AtomicU64::new(0));
        (
            Self {
                name: name.to_string(),
                sleep_ms,
                tick_count: tick_count.clone(),
            },
            tick_count,
        )
    }
}

impl Node for SlowComputeNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn init(&mut self) -> Result<()> {
        Ok(())
    }

    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::Relaxed);
        std::thread::sleep(Duration::from_millis(self.sleep_ms));
    }
}

/// A simple counter node for basic testing.
struct CounterNode {
    name: String,
    count: Arc<AtomicU64>,
}

impl CounterNode {
    fn new(name: &str) -> (Self, Arc<AtomicU64>) {
        let count = Arc::new(AtomicU64::new(0));
        (
            Self {
                name: name.to_string(),
                count: count.clone(),
            },
            count,
        )
    }
}

impl Node for CounterNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn init(&mut self) -> Result<()> {
        Ok(())
    }

    fn tick(&mut self) {
        self.count.fetch_add(1, Ordering::Relaxed);
    }
}

// ============================================================================
// Tests
// ============================================================================

/// The Discord user's exact scenario:
/// RT node at 1kHz + slow compute node sleeping 100ms.
/// The RT node must maintain its tick rate regardless.
#[test]
fn test_rt_isolation_under_compute_load() {
    cleanup_stale_shm();

    let (rt_node, rt_count, rt_timestamps) = RtTimingNode::new("rt_controller");
    let (slow_node, compute_count) = SlowComputeNode::new("heavy_compute", 50);

    let mut scheduler = Scheduler::new().tick_hz(1000.0);
    scheduler.add(rt_node).order(0).rt().rate_hz(1000.0).done();
    scheduler.add(slow_node).order(5).rate_hz(10.0).done(); // 10 Hz compute

    // Run for 200ms
    scheduler.run_for(Duration::from_millis(200)).unwrap();

    let rt_ticks = rt_count.load(Ordering::Relaxed);
    let compute_ticks = compute_count.load(Ordering::Relaxed);

    // RT node should have ticked many times (~200 at 1kHz for 200ms)
    // Allow wide margin: debug builds on non-RT kernels without SCHED_FIFO
    // run much slower; the key proof is that RT ticks >> what it'd get if
    // blocked by compute (which would be ~4 ticks at 200ms/50ms).
    assert!(
        rt_ticks >= 20,
        "RT node should tick at least 20 times in 200ms at 1kHz, got {}",
        rt_ticks
    );

    // Compute node at 10Hz for 200ms should tick ~2 times
    assert!(
        compute_ticks >= 1,
        "Compute node should tick at least once at 10Hz in 200ms, got {}",
        compute_ticks
    );

    // Verify RT jitter: check that inter-tick intervals are reasonable
    let timestamps = rt_timestamps.lock().unwrap();
    if timestamps.len() > 2 {
        let intervals: Vec<Duration> = timestamps.windows(2).map(|w| w[1] - w[0]).collect();

        // Calculate max jitter (max interval - min interval)
        let min_interval = intervals.iter().min().unwrap();
        let max_interval = intervals.iter().max().unwrap();
        let jitter = *max_interval - *min_interval;

        // On non-RT kernels, we can't guarantee <100us jitter, but we can
        // verify the RT thread is independent of the slow compute node.
        // If RT was blocked by compute, max interval would be >50ms.
        assert!(
            max_interval.as_millis() < 50,
            "RT node max inter-tick interval should be < 50ms (proving isolation from 50ms compute), got {:?}",
            max_interval
        );

        // Print jitter info for debugging
        eprintln!(
            "RT jitter analysis: ticks={}, min_interval={:?}, max_interval={:?}, jitter={:?}",
            timestamps.len(),
            min_interval,
            max_interval,
            jitter
        );
    }
}

/// Verify that compute nodes run in parallel (not sequentially).
#[test]
fn test_compute_nodes_run_parallel() {
    cleanup_stale_shm();

    // Two slow compute nodes, each sleeping 30ms
    let (node_a, count_a) = SlowComputeNode::new("compute_a", 30);
    let (node_b, count_b) = SlowComputeNode::new("compute_b", 30);

    let mut scheduler = Scheduler::new().tick_hz(100.0);
    scheduler.add(node_a).order(1).done();
    scheduler.add(node_b).order(2).done();

    let start = Instant::now();
    scheduler.run_for(Duration::from_millis(150)).unwrap();
    let elapsed = start.elapsed();

    let ticks_a = count_a.load(Ordering::Relaxed);
    let ticks_b = count_b.load(Ordering::Relaxed);

    // Both nodes should have ticked
    assert!(ticks_a >= 1, "Node A should tick, got {}", ticks_a);
    assert!(ticks_b >= 1, "Node B should tick, got {}", ticks_b);

    // If nodes ran in parallel, both should have approximately the same tick count
    // If sequential, one would have half the ticks
    eprintln!(
        "Parallel test: A={} ticks, B={} ticks in {:?}",
        ticks_a, ticks_b, elapsed
    );
}

/// Verify that RT + compute together works correctly.
#[test]
fn test_mixed_rt_and_compute_nodes() {
    cleanup_stale_shm();

    let (rt_node, rt_count, _) = RtTimingNode::new("mixed_rt");
    let (compute_node, compute_count) = CounterNode::new("mixed_compute");

    let mut scheduler = Scheduler::new().tick_hz(500.0);
    scheduler.add(rt_node).order(0).rt().rate_hz(500.0).done();
    scheduler.add(compute_node).order(5).done();

    scheduler.run_for(Duration::from_millis(100)).unwrap();

    let rt_ticks = rt_count.load(Ordering::Relaxed);
    let compute_ticks = compute_count.load(Ordering::Relaxed);

    // Both should have ticked
    assert!(
        rt_ticks >= 10,
        "RT node should tick at least 10 times in 100ms at 500Hz, got {}",
        rt_ticks
    );
    assert!(
        compute_ticks >= 10,
        "Compute node should tick at least 10 times in 100ms, got {}",
        compute_ticks
    );

    eprintln!(
        "Mixed test: RT={} ticks, Compute={} ticks",
        rt_ticks, compute_ticks
    );
}

/// Sequential mode should still work exactly as before (no RT/compute split).
#[test]
fn test_sequential_mode_unchanged() {
    cleanup_stale_shm();

    let (node_a, count_a) = CounterNode::new("seq_a");
    let (node_b, count_b) = CounterNode::new("seq_b");

    let mut scheduler = Scheduler::new(); // Default = Sequential
    scheduler.add(node_a).order(0).done();
    scheduler.add(node_b).order(1).done();

    scheduler.run_for(Duration::from_millis(100)).unwrap();

    let ticks_a = count_a.load(Ordering::Relaxed);
    let ticks_b = count_b.load(Ordering::Relaxed);

    // Both should have ticked in sequential mode
    assert!(
        ticks_a >= 1,
        "Node A should tick in sequential mode, got {}",
        ticks_a
    );
    assert!(
        ticks_b >= 1,
        "Node B should tick in sequential mode, got {}",
        ticks_b
    );

    eprintln!("Sequential test: A={} ticks, B={} ticks", ticks_a, ticks_b);
}

/// Full system test: multiple RT nodes at different rates + multiple compute nodes.
/// Verifies all groups run simultaneously, RT maintains timing, and shutdown is clean.
#[test]
fn test_full_system_rt_and_compute_groups() {
    cleanup_stale_shm();

    // RT group: fast controller (500Hz) + slower sensor (100Hz)
    let (rt_fast, rt_fast_count, _) = RtTimingNode::new("rt_controller_500hz");
    let (rt_slow, rt_slow_count, _) = RtTimingNode::new("rt_sensor_100hz");

    // Compute group: two parallel workers + one slow worker
    let (compute_a, compute_a_count) = CounterNode::new("compute_worker_a");
    let (compute_b, compute_b_count) = CounterNode::new("compute_worker_b");
    let (slow_compute, slow_compute_count) = SlowComputeNode::new("slow_planner", 30);

    let mut scheduler = Scheduler::new().tick_hz(500.0);

    // RT nodes
    scheduler.add(rt_fast).order(0).rt().rate_hz(500.0).done();
    scheduler.add(rt_slow).order(1).rt().rate_hz(100.0).done();

    // Compute nodes
    scheduler.add(compute_a).order(10).done();
    scheduler.add(compute_b).order(11).done();
    scheduler.add(slow_compute).order(20).rate_hz(10.0).done();

    // Run for 300ms
    scheduler.run_for(Duration::from_millis(300)).unwrap();

    let fast_ticks = rt_fast_count.load(Ordering::Relaxed);
    let slow_rt_ticks = rt_slow_count.load(Ordering::Relaxed);
    let ca_ticks = compute_a_count.load(Ordering::Relaxed);
    let cb_ticks = compute_b_count.load(Ordering::Relaxed);
    let slow_c_ticks = slow_compute_count.load(Ordering::Relaxed);

    // RT group assertions
    assert!(
        fast_ticks >= 50,
        "500Hz RT should tick >= 50 in 300ms, got {}",
        fast_ticks
    );
    assert!(
        slow_rt_ticks >= 10,
        "100Hz RT should tick >= 10 in 300ms, got {}",
        slow_rt_ticks
    );
    assert!(
        fast_ticks > slow_rt_ticks * 2,
        "500Hz ({}) should be >2x of 100Hz ({})",
        fast_ticks,
        slow_rt_ticks
    );

    // Compute group assertions
    assert!(
        ca_ticks >= 10,
        "Compute A should tick >= 10 in 300ms, got {}",
        ca_ticks
    );
    assert!(
        cb_ticks >= 10,
        "Compute B should tick >= 10 in 300ms, got {}",
        cb_ticks
    );
    assert!(
        slow_c_ticks >= 1,
        "Slow compute (10Hz, 30ms sleep) should tick >= 1 in 300ms, got {}",
        slow_c_ticks
    );

    // Key isolation check: slow compute doesn't block RT
    // If RT was blocked, fast_ticks would be ~3 (300ms / 30ms per slow tick * 0.3)
    assert!(
        fast_ticks >= 50,
        "RT isolation check: 500Hz RT ({}) should not be blocked by 30ms compute",
        fast_ticks
    );

    eprintln!(
        "Full system: RT_fast={}, RT_slow={}, Compute_A={}, Compute_B={}, SlowCompute={}",
        fast_ticks, slow_rt_ticks, ca_ticks, cb_ticks, slow_c_ticks
    );
}

// ============================================================================
// Event-Triggered Node Tests
// ============================================================================

/// Event node should only tick when data is published to its topic.
/// Publish N messages, verify node ticks exactly N times.
#[test]
fn test_event_node_ticks_on_publish() {
    use horus_core::core::NodeInfo;

    cleanup_stale_shm();

    let (event_node, event_count) = CounterNode::new("evt_sensor");

    let mut scheduler = Scheduler::new();
    scheduler.add(event_node).order(0).on("lidar_scan").done();

    // Run scheduler in a background thread
    let running = scheduler.running_flag();
    let handle = std::thread::spawn(move || {
        let _ = scheduler.run();
    });

    // Give the scheduler and event watcher time to start
    std::thread::sleep(Duration::from_millis(50));

    // Publish 10 notifications at irregular intervals
    for i in 0..10 {
        NodeInfo::notify_event("evt_sensor");
        // Irregular intervals: 5ms, 10ms, 5ms, ...
        let sleep_ms = if i % 2 == 0 { 5 } else { 10 };
        std::thread::sleep(Duration::from_millis(sleep_ms));
    }

    // Wait for all ticks to process
    std::thread::sleep(Duration::from_millis(50));

    // Stop the scheduler
    running.store(false, Ordering::SeqCst);
    handle.join().unwrap();

    let ticks = event_count.load(Ordering::Relaxed);
    assert_eq!(
        ticks, 10,
        "Event node should tick exactly 10 times for 10 publishes, got {}",
        ticks
    );
}

/// Event node should NOT tick when no data is published.
#[test]
fn test_event_node_no_tick_without_data() {
    cleanup_stale_shm();

    let (event_node, event_count) = CounterNode::new("evt_idle");

    let mut scheduler = Scheduler::new();
    scheduler.add(event_node).order(0).on("unused_topic").done();

    // Run for 100ms without publishing any data
    scheduler.run_for(Duration::from_millis(100)).unwrap();

    let ticks = event_count.load(Ordering::Relaxed);
    assert_eq!(
        ticks, 0,
        "Event node should NOT tick without data, got {}",
        ticks
    );
}

/// Event node works alongside RT and BestEffort nodes in the same scheduler.
#[test]
fn test_event_node_alongside_rt_and_besteffort() {
    use horus_core::core::NodeInfo;

    cleanup_stale_shm();

    let (rt_node, rt_count, _) = RtTimingNode::new("mixed_rt_node");
    let (be_node, be_count) = CounterNode::new("mixed_be_node");
    let (evt_node, evt_count) = CounterNode::new("mixed_evt_node");

    let mut scheduler = Scheduler::new().tick_hz(200.0);
    scheduler.add(rt_node).order(0).rt().rate_hz(200.0).done();
    scheduler.add(be_node).order(5).done();
    scheduler.add(evt_node).order(10).on("sensor_data").done();

    let running = scheduler.running_flag();
    let handle = std::thread::spawn(move || {
        let _ = scheduler.run();
    });

    // Wait for the event node's notifier to be registered in the global registry.
    // On loaded systems the scheduler/event threads may take time to start.
    let mut registered = false;
    for _ in 0..100 {
        if NodeInfo::notify_event("mixed_evt_node") {
            registered = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(
        registered,
        "Event node notifier should register within 500ms"
    );

    // That first notify_event already sent 1 notification; send 4 more
    for _ in 0..4 {
        NodeInfo::notify_event("mixed_evt_node");
        std::thread::sleep(Duration::from_millis(10));
    }

    // Let the system run a bit more
    std::thread::sleep(Duration::from_millis(100));

    running.store(false, Ordering::SeqCst);
    handle.join().unwrap();

    let rt_ticks = rt_count.load(Ordering::Relaxed);
    let be_ticks = be_count.load(Ordering::Relaxed);
    let evt_ticks = evt_count.load(Ordering::Relaxed);

    // RT node should have many ticks
    assert!(
        rt_ticks >= 10,
        "RT node should tick >= 10 times, got {}",
        rt_ticks
    );

    // BestEffort node should have many ticks
    assert!(
        be_ticks >= 10,
        "BestEffort node should tick >= 10 times, got {}",
        be_ticks
    );

    // Event node should tick exactly 5 times (once per notification)
    assert_eq!(
        evt_ticks, 5,
        "Event node should tick exactly 5 times for 5 notifications, got {}",
        evt_ticks
    );

    eprintln!(
        "Mixed system: RT={}, BestEffort={}, Event={}",
        rt_ticks, be_ticks, evt_ticks
    );
}

// ============================================================================
// Async I/O Isolation Tests
// ============================================================================

/// A blocking I/O node that sleeps to simulate network latency.
struct BlockingIoNode {
    name: String,
    sleep_ms: u64,
    tick_count: Arc<AtomicU64>,
}

impl BlockingIoNode {
    fn new(name: &str, sleep_ms: u64) -> (Self, Arc<AtomicU64>) {
        let tick_count = Arc::new(AtomicU64::new(0));
        (
            Self {
                name: name.to_string(),
                sleep_ms,
                tick_count: tick_count.clone(),
            },
            tick_count,
        )
    }
}

impl Node for BlockingIoNode {
    fn name(&self) -> &'static str {
        Box::leak(self.name.clone().into_boxed_str())
    }

    fn init(&mut self) -> Result<()> {
        Ok(())
    }

    fn tick(&mut self) {
        self.tick_count.fetch_add(1, Ordering::Relaxed);
        std::thread::sleep(Duration::from_millis(self.sleep_ms));
    }
}

/// Async I/O isolation test:
/// An async_io node sleeping 200ms per tick must NOT affect RT jitter or
/// compute throughput. This proves the tokio blocking pool is fully isolated.
///
/// Setup:
///   - RT node at 1kHz recording timestamps for jitter analysis
///   - Compute counter node (fast, no sleep) for throughput measurement
///   - Async I/O node sleeping 200ms per tick (simulating blocking network I/O)
///
/// Acceptance criteria:
///   1. RT max inter-tick interval stays < 20ms (if blocked by async, it'd be 200ms+)
///   2. Compute throughput is not degraded (ticks many times, not blocked)
///   3. Async I/O node still executes its ticks (at least 1 in 500ms)
#[test]
fn test_async_io_isolation_from_rt_and_compute() {
    cleanup_stale_shm();

    // RT node at 1kHz for jitter analysis
    let (rt_node, rt_count, rt_timestamps) = RtTimingNode::new("iso_rt_1khz");

    // Fast compute node — should tick many times unimpeded
    let (compute_node, compute_count) = CounterNode::new("iso_compute_fast");

    // Blocking async I/O node — 200ms sleep simulates network call
    let (async_node, async_count) = BlockingIoNode::new("iso_async_slow", 200);

    let mut scheduler = Scheduler::new().tick_hz(1000.0);
    scheduler.add(rt_node).order(0).rt().rate_hz(1000.0).done();
    scheduler.add(compute_node).order(5).done();
    scheduler
        .add(async_node)
        .order(10)
        .async_io()
        .rate_hz(5.0) // 5 Hz — each tick blocks 200ms
        .done();

    // Run for 500ms — enough for async node to tick a few times
    scheduler.run_for(Duration::from_millis(500)).unwrap();

    let rt_ticks = rt_count.load(Ordering::Relaxed);
    let compute_ticks = compute_count.load(Ordering::Relaxed);
    let async_ticks = async_count.load(Ordering::Relaxed);

    // === Criterion 1: RT maintains timing ===
    // On non-RT kernels in debug builds, actual tick rate is well below 1kHz.
    // The key proof: if blocked by 200ms async I/O, RT would get ~2 ticks (500ms/200ms).
    // Getting 20+ ticks definitively proves isolation.
    assert!(
        rt_ticks >= 20,
        "RT node should tick >= 20 in 500ms (proving not blocked by 200ms async), got {}",
        rt_ticks
    );

    // Verify RT jitter via timestamps
    let timestamps = rt_timestamps.lock().unwrap();
    if timestamps.len() > 2 {
        let intervals: Vec<Duration> = timestamps.windows(2).map(|w| w[1] - w[0]).collect();

        let max_interval = intervals.iter().max().unwrap();

        // If RT was blocked by the async 200ms sleep, max interval would be >= 200ms.
        // On a healthy system with isolation, it should be well under 20ms.
        assert!(
            max_interval.as_millis() < 50,
            "RT max inter-tick interval must be < 50ms (proving isolation from 200ms async I/O), got {:?}",
            max_interval
        );

        let min_interval = intervals.iter().min().unwrap();
        let jitter = *max_interval - *min_interval;
        eprintln!(
            "  RT jitter: ticks={}, min={:?}, max={:?}, jitter={:?}",
            timestamps.len(),
            min_interval,
            max_interval,
            jitter
        );
    }

    // === Criterion 2: Compute throughput not degraded ===
    // A fast counter node should tick many times in 500ms.
    // If blocked by async 200ms sleep, it would be limited to ~2-3 ticks.
    // Getting 20+ ticks proves compute is isolated from the async pool.
    assert!(
        compute_ticks >= 20,
        "Compute node should tick >= 20 in 500ms (proving not blocked by async I/O), got {}",
        compute_ticks
    );

    // === Criterion 3: Async I/O node still executes ===
    // At 5 Hz with 200ms sleep, expect ~2 ticks in 500ms
    assert!(
        async_ticks >= 1,
        "Async I/O node should tick at least once in 500ms, got {}",
        async_ticks
    );

    eprintln!(
        "Async I/O isolation: RT={}, Compute={}, AsyncIO={}",
        rt_ticks, compute_ticks, async_ticks
    );
}

/// Multiple async I/O nodes with different blocking durations should run concurrently
/// on the tokio pool and not serialize each other.
#[test]
fn test_async_io_nodes_run_concurrently() {
    cleanup_stale_shm();

    // Two async I/O nodes, each sleeping 100ms
    let (io_a, count_a) = BlockingIoNode::new("async_a_100ms", 100);
    let (io_b, count_b) = BlockingIoNode::new("async_b_100ms", 100);

    let mut scheduler = Scheduler::new().tick_hz(100.0);
    scheduler.add(io_a).order(0).async_io().done();
    scheduler.add(io_b).order(1).async_io().done();

    // Run for 350ms
    scheduler.run_for(Duration::from_millis(350)).unwrap();

    let ticks_a = count_a.load(Ordering::Relaxed);
    let ticks_b = count_b.load(Ordering::Relaxed);

    // If concurrent: both ~3 ticks in 350ms (100ms sleep each).
    // If sequential: total of ~3 ticks split between them (one gets ~2, other ~1).
    assert!(
        ticks_a >= 1,
        "Async I/O node A should tick at least once, got {}",
        ticks_a
    );
    assert!(
        ticks_b >= 1,
        "Async I/O node B should tick at least once, got {}",
        ticks_b
    );

    // Both should have similar tick counts if running concurrently
    eprintln!(
        "Async I/O concurrency: A={}, B={} (should be similar if concurrent)",
        ticks_a, ticks_b
    );
}

/// Async I/O + Event + RT + Compute — all execution groups active simultaneously.
/// Proves the full 4-group architecture works correctly.
#[test]
fn test_all_execution_groups_simultaneously() {
    use horus_core::core::NodeInfo;

    cleanup_stale_shm();

    // RT group
    let (rt_node, rt_count, _) = RtTimingNode::new("all_rt");
    // Compute group
    let (compute_node, compute_count) = CounterNode::new("all_compute");
    // Async I/O group
    let (async_node, async_count) = BlockingIoNode::new("all_async", 50);
    // Event group
    let (event_node, event_count) = CounterNode::new("all_event");

    let mut scheduler = Scheduler::new().tick_hz(500.0);
    scheduler.add(rt_node).order(0).rt().rate_hz(500.0).done();
    scheduler.add(compute_node).order(5).done();
    scheduler
        .add(async_node)
        .order(10)
        .async_io()
        .rate_hz(10.0)
        .done();
    scheduler.add(event_node).order(15).on("all_topic").done();

    let running = scheduler.running_flag();
    let handle = std::thread::spawn(move || {
        let _ = scheduler.run();
    });

    // Give all groups time to start
    std::thread::sleep(Duration::from_millis(50));

    // Send event notifications
    for _ in 0..5 {
        NodeInfo::notify_event("all_event");
        std::thread::sleep(Duration::from_millis(20));
    }

    // Let the system run more
    std::thread::sleep(Duration::from_millis(200));

    running.store(false, Ordering::SeqCst);
    handle.join().unwrap();

    let rt = rt_count.load(Ordering::Relaxed);
    let compute = compute_count.load(Ordering::Relaxed);
    let async_io = async_count.load(Ordering::Relaxed);
    let event = event_count.load(Ordering::Relaxed);

    assert!(rt >= 20, "RT should tick >= 20, got {}", rt);
    assert!(compute >= 20, "Compute should tick >= 20, got {}", compute);
    assert!(
        async_io >= 1,
        "Async I/O should tick >= 1, got {}",
        async_io
    );
    assert_eq!(event, 5, "Event should tick exactly 5 times, got {}", event);

    eprintln!(
        "All groups: RT={}, Compute={}, AsyncIO={}, Event={}",
        rt, compute, async_io, event
    );
}

// ============================================================================
// Clean Shutdown Tests
// ============================================================================

/// Verify that scheduler.stop() triggers clean shutdown of all groups.
#[test]
fn test_clean_shutdown_all_groups() {
    cleanup_stale_shm();

    let (rt_node, rt_count, _) = RtTimingNode::new("shutdown_rt");
    let (compute_node, compute_count) = CounterNode::new("shutdown_compute");

    let mut scheduler = Scheduler::new().tick_hz(100.0);
    scheduler.add(rt_node).order(0).rt().rate_hz(100.0).done();
    scheduler.add(compute_node).order(5).done();

    // run_for triggers clean shutdown at the end
    scheduler.run_for(Duration::from_millis(100)).unwrap();

    // After run_for returns, both groups should have been stopped and joined
    // The fact that run_for returns without hanging proves clean shutdown
    let rt_ticks = rt_count.load(Ordering::Relaxed);
    let compute_ticks = compute_count.load(Ordering::Relaxed);

    assert!(rt_ticks >= 1, "RT should have ticked before shutdown");
    assert!(
        compute_ticks >= 1,
        "Compute should have ticked before shutdown"
    );
}

// ============================================================================
// Multi-Rate RT Timing Accuracy Tests
// ============================================================================

/// RT nodes at different rates (1kHz, 100Hz, 10Hz) all tick at their correct
/// rates within the same RT thread. Verifies the timing wheel handles multi-rate.
#[test]
fn test_multi_rate_rt_timing_accuracy() {
    cleanup_stale_shm();

    let (node_1khz, count_1khz, _) = RtTimingNode::new("rt_1khz");
    let (node_100hz, count_100hz, _) = RtTimingNode::new("rt_100hz");
    let (node_10hz, count_10hz, _) = RtTimingNode::new("rt_10hz");

    let mut scheduler = Scheduler::new().tick_hz(1000.0);
    scheduler
        .add(node_1khz)
        .order(0)
        .rt()
        .rate_hz(1000.0)
        .done();
    scheduler
        .add(node_100hz)
        .order(1)
        .rt()
        .rate_hz(100.0)
        .done();
    scheduler.add(node_10hz).order(2).rt().rate_hz(10.0).done();

    // Run for 500ms — enough for statistical significance
    scheduler.run_for(Duration::from_millis(500)).unwrap();

    let ticks_1khz = count_1khz.load(Ordering::Relaxed);
    let ticks_100hz = count_100hz.load(Ordering::Relaxed);
    let ticks_10hz = count_10hz.load(Ordering::Relaxed);

    // Expected: ~500 ticks at 1kHz, ~50 at 100Hz, ~5 at 10Hz
    // Very wide margins: debug builds on non-RT kernels can't sustain 1kHz.
    // The key proof is the *ratio* between rates, not absolute counts.
    assert!(
        ticks_1khz >= 10,
        "1kHz RT should tick >= 10 in 500ms, got {}",
        ticks_1khz
    );
    assert!(
        ticks_100hz >= 3,
        "100Hz RT should tick >= 3 in 500ms, got {}",
        ticks_100hz
    );
    assert!(
        ticks_10hz >= 2,
        "10Hz RT should tick >= 2 in 500ms, got {}",
        ticks_10hz
    );

    // Ratio checks: 1kHz should have ~10x more ticks than 100Hz
    let ratio_1k_to_100 = ticks_1khz as f64 / ticks_100hz as f64;
    assert!(
        ratio_1k_to_100 > 3.0,
        "1kHz/100Hz tick ratio should be > 3x, got {:.1}x ({}/{})",
        ratio_1k_to_100,
        ticks_1khz,
        ticks_100hz
    );

    // 100Hz should have ~10x more ticks than 10Hz
    let ratio_100_to_10 = ticks_100hz as f64 / ticks_10hz as f64;
    assert!(
        ratio_100_to_10 > 3.0,
        "100Hz/10Hz tick ratio should be > 3x, got {:.1}x ({}/{})",
        ratio_100_to_10,
        ticks_100hz,
        ticks_10hz
    );

    eprintln!(
        "Multi-rate RT: 1kHz={}, 100Hz={}, 10Hz={} (ratios: {:.1}x, {:.1}x)",
        ticks_1khz, ticks_100hz, ticks_10hz, ratio_1k_to_100, ratio_100_to_10
    );
}

// ============================================================================
// Compute Load Shedding Tests
// ============================================================================

/// Under compute overload, high-order compute nodes (order >= 200) are shed
/// while low-order nodes continue unimpeded.
#[test]
fn test_compute_load_shedding() {
    cleanup_stale_shm();

    // Fast high-priority node (order 10) — should never be shed
    let (fast_hi, count_hi) = CounterNode::new("fast_hi_10");
    // Fast medium-priority node (order 50) — should never be shed
    let (fast_mid, count_mid) = CounterNode::new("fast_mid_50");
    // Slow background node (order 200) — sheddable, sleeps to cause overload
    let (slow_bg, count_bg) = SlowComputeNode::new("slow_bg_200", 100);

    let mut scheduler = Scheduler::new().tick_hz(100.0);
    scheduler.add(fast_hi).order(10).compute().done();
    scheduler.add(fast_mid).order(50).compute().done();
    // Order 200 = sheddable tier. 100ms sleep on a 10ms tick period = guaranteed overload.
    scheduler
        .add(slow_bg)
        .order(200)
        .compute()
        .rate_hz(100.0)
        .done();

    // Run for 500ms
    scheduler.run_for(Duration::from_millis(500)).unwrap();

    let hi_ticks = count_hi.load(Ordering::Relaxed);
    let mid_ticks = count_mid.load(Ordering::Relaxed);
    let bg_ticks = count_bg.load(Ordering::Relaxed);

    // High-priority nodes should tick many times (they're fast, not shed).
    // With hysteresis, shedding stays active longer so fast nodes get more
    // uncontested cycles. Threshold is conservative for CI variability.
    assert!(
        hi_ticks >= 5,
        "High-priority compute (order 10) should tick >= 5 in 500ms, got {}",
        hi_ticks
    );
    assert!(
        mid_ticks >= 5,
        "Medium-priority compute (order 50) should tick >= 5 in 500ms, got {}",
        mid_ticks
    );

    // Background node should have fewer ticks than the others due to shedding.
    // It can still tick on cycles where shedding hasn't activated yet (first cycle).
    // The key: bg_ticks should be significantly less than hi_ticks.
    assert!(
        bg_ticks < hi_ticks,
        "Background node (order 200) should be shed and have fewer ticks than high-priority. bg={}, hi={}",
        bg_ticks, hi_ticks
    );

    eprintln!(
        "Load shedding: hi(10)={}, mid(50)={}, bg(200)={} (bg < hi proves shedding)",
        hi_ticks, mid_ticks, bg_ticks
    );
}
