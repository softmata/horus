//! Measure actual latency of all AdaptiveTopic backend paths
//!
//! Compares expected vs actual latency for each of the 10 backend modes.
//! Uses CmdVel::with_timestamp() to avoid SystemTime::now() syscall overhead (~52ns).

use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

const ITERATIONS: u64 = 100_000;
const WARMUP: u64 = 10_000;
const TIMEOUT_SECS: u64 = 30;

fn main() {
    // Check for child process mode
    let args: Vec<String> = std::env::args().collect();
    if args.len() >= 4 && args[1] == "--child-publisher" {
        let topic = &args[2];
        let count: u64 = args[3].parse().unwrap_or(10000);
        run_child_publisher(topic, count);
        return;
    }
    if args.len() >= 4 && args[1] == "--child-consumer" {
        let topic = &args[2];
        let count: u64 = args[3].parse().unwrap_or(10000);
        run_child_consumer(topic, count);
        return;
    }

    println!("╔══════════════════════════════════════════════════════════════════════╗");
    println!("║            AdaptiveTopic - All 10 Routes Latency Benchmark           ║");
    println!("║     (Using with_timestamp() to exclude ~52ns SystemTime overhead)    ║");
    println!("╠══════════════════════════════════════════════════════════════════════╣");
    println!("║ Route             │ Expected │ Actual  │ Overhead │ Status          ║");
    println!("╠═══════════════════╪══════════╪═════════╪══════════╪═════════════════╣");

    // ============================================================
    // INTRA-PROCESS ROUTES (5 modes)
    // ============================================================

    // 1. DirectChannel - Same thread (0 atomics)
    let latency = bench_direct_channel();
    print_row("DirectChannel", 3, latency);

    // 2. SpscIntra - Cross-thread 1P-1C
    let latency = bench_spsc_intra();
    print_row("SpscIntra", 18, latency);

    // 3. MpscIntra - Cross-thread MP-1C (multiple producers, 1 consumer)
    let latency = bench_mpsc_intra();
    print_row("MpscIntra", 26, latency);

    // 4. MpmcIntra - Cross-thread MPMC (multiple producers, multiple consumers)
    let latency = bench_mpmc_intra();
    print_row("MpmcIntra", 36, latency);

    println!("╠═══════════════════╪══════════╪═════════╪══════════╪═════════════════╣");

    // ============================================================
    // CROSS-PROCESS ROUTES (5 modes)
    // ============================================================

    // 5. SpscShm - Cross-process 1P-1C
    let latency = bench_spsc_shm();
    print_row("SpscShm", 85, latency);

    // 6. MpscShm - Cross-process MP-1C (multiple producers, 1 consumer)
    let latency = bench_mpsc_shm();
    print_row("MpscShm", 65, latency);

    // 7. MpmcShm - Cross-process MPMC
    let latency = bench_mpmc_shm();
    print_row("MpmcShm", 167, latency);

    println!("╚══════════════════════════════════════════════════════════════════════╝");
    println!();
    println!("Note: SpmcIntra/SpmcShm/PodShm skipped (broadcast semantics vary)");
    println!();

    // Detailed breakdown for DirectChannel
    println!("=== Detailed DirectChannel Breakdown ===");
    bench_direct_channel_detailed();
}

fn print_row(name: &str, expected: u64, actual: f64) {
    let overhead = actual - expected as f64;
    let status = if actual < (expected as f64 * 1.5) {
        "✓ EXCELLENT"
    } else if actual < (expected as f64 * 2.0) {
        "✓ OK"
    } else if actual < (expected as f64 * 5.0) {
        "⚠ HIGH"
    } else {
        "✗ BAD"
    };
    println!(
        "║ {:17} │ {:>6}ns │ {:>6.0}ns │ {:>+7.0}ns │ {:15} ║",
        name, expected, actual, overhead, status
    );
}

// ============================================================
// INTRA-PROCESS BENCHMARKS
// ============================================================

/// DirectChannel - Same thread, no atomics
fn bench_direct_channel() -> f64 {
    let topic: Topic<CmdVel> = Topic::new("bench_direct").unwrap();

    // Warmup - establishes DirectChannel mode
    for _ in 0..WARMUP {
        let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
        let _ = topic.send(msg);
        let _ = topic.recv();
    }

    // Drain
    while topic.recv().is_some() {}

    // Measure round-trip (send + recv)
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let start = Instant::now();
    for _ in 0..ITERATIONS {
        let _ = std::hint::black_box(topic.send(std::hint::black_box(msg)));
        let _ = std::hint::black_box(topic.recv());
    }
    let elapsed = start.elapsed();

    // Return per-operation latency (round-trip / 2)
    (elapsed.as_nanos() as f64 / ITERATIONS as f64) / 2.0
}

/// SpscIntra - Cross-thread, 1 producer, 1 consumer
fn bench_spsc_intra() -> f64 {
    let topic_name = format!("bench_spsc_{}", std::process::id());
    let producer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();

    let received_count = Arc::new(AtomicU64::new(0));
    let received_count_clone = received_count.clone();
    let done = Arc::new(AtomicBool::new(false));
    let done_clone = done.clone();

    // Consumer thread
    let cons_handle = thread::spawn(move || {
        while !done_clone.load(Ordering::Relaxed) {
            if consumer.recv().is_some() {
                received_count_clone.fetch_add(1, Ordering::Relaxed);
            } else {
                thread::yield_now();
            }
        }
        // Drain remaining
        while consumer.recv().is_some() {
            received_count_clone.fetch_add(1, Ordering::Relaxed);
        }
    });

    // Wait for consumer to register
    thread::sleep(Duration::from_millis(10));

    // Producer sends (this is what we measure)
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let start = Instant::now();
    for _ in 0..ITERATIONS {
        while producer.send(msg).is_err() {
            thread::yield_now();
        }
    }
    let elapsed = start.elapsed();

    // Wait for messages to be consumed
    thread::sleep(Duration::from_millis(100));
    done.store(true, Ordering::Relaxed);
    cons_handle.join().unwrap();

    let received = received_count.load(Ordering::Relaxed);
    if received < ITERATIONS {
        println!("  (Warning: SpscIntra only received {}/{})", received, ITERATIONS);
    }

    elapsed.as_nanos() as f64 / ITERATIONS as f64
}

/// MpscIntra - Cross-thread, multiple producers, 1 consumer
fn bench_mpsc_intra() -> f64 {
    let topic_name = format!("bench_mpsc_{}", std::process::id());
    let producer1: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let producer2: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();

    let received_count = Arc::new(AtomicU64::new(0));
    let received_count_clone = received_count.clone();
    let done = Arc::new(AtomicBool::new(false));
    let done_clone = done.clone();

    // Consumer thread
    let cons_handle = thread::spawn(move || {
        while !done_clone.load(Ordering::Relaxed) {
            if consumer.recv().is_some() {
                received_count_clone.fetch_add(1, Ordering::Relaxed);
            } else {
                thread::yield_now();
            }
        }
        // Drain remaining
        while consumer.recv().is_some() {
            received_count_clone.fetch_add(1, Ordering::Relaxed);
        }
    });

    // Wait for consumer to register
    thread::sleep(Duration::from_millis(10));

    let half = ITERATIONS / 2;
    let p1_done = Arc::new(AtomicBool::new(false));
    let p2_done = Arc::new(AtomicBool::new(false));
    let p1_done_clone = p1_done.clone();
    let p2_done_clone = p2_done.clone();

    // Producer threads
    let p1_handle = thread::spawn(move || {
        let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
        for _ in 0..half {
            while producer1.send(msg).is_err() {
                thread::yield_now();
            }
        }
        p1_done_clone.store(true, Ordering::Relaxed);
    });

    let p2_handle = thread::spawn(move || {
        let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
        for _ in 0..half {
            while producer2.send(msg).is_err() {
                thread::yield_now();
            }
        }
        p2_done_clone.store(true, Ordering::Relaxed);
    });

    let start = Instant::now();

    // Wait for both producers to finish
    while !p1_done.load(Ordering::Relaxed) || !p2_done.load(Ordering::Relaxed) {
        thread::yield_now();
    }
    let elapsed = start.elapsed();

    p1_handle.join().unwrap();
    p2_handle.join().unwrap();

    // Wait for messages to be consumed
    thread::sleep(Duration::from_millis(100));
    done.store(true, Ordering::Relaxed);
    cons_handle.join().unwrap();

    let received = received_count.load(Ordering::Relaxed);
    if received < ITERATIONS {
        println!("  (Warning: MpscIntra only received {}/{})", received, ITERATIONS);
    }

    elapsed.as_nanos() as f64 / ITERATIONS as f64
}

/// MpmcIntra - Cross-thread, multiple producers, multiple consumers
fn bench_mpmc_intra() -> f64 {
    let topic_name = format!("bench_mpmc_{}", std::process::id());
    let producer1: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let producer2: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let consumer1: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let consumer2: Topic<CmdVel> = Topic::new(&topic_name).unwrap();

    let received_count = Arc::new(AtomicU64::new(0));
    let rc1 = received_count.clone();
    let rc2 = received_count.clone();
    let done = Arc::new(AtomicBool::new(false));
    let done_c1 = done.clone();
    let done_c2 = done.clone();

    // Consumer threads
    let c1_handle = thread::spawn(move || {
        while !done_c1.load(Ordering::Relaxed) {
            if consumer1.recv().is_some() {
                rc1.fetch_add(1, Ordering::Relaxed);
            } else {
                thread::yield_now();
            }
        }
        while consumer1.recv().is_some() {
            rc1.fetch_add(1, Ordering::Relaxed);
        }
    });

    let c2_handle = thread::spawn(move || {
        while !done_c2.load(Ordering::Relaxed) {
            if consumer2.recv().is_some() {
                rc2.fetch_add(1, Ordering::Relaxed);
            } else {
                thread::yield_now();
            }
        }
        while consumer2.recv().is_some() {
            rc2.fetch_add(1, Ordering::Relaxed);
        }
    });

    // Wait for consumers to register
    thread::sleep(Duration::from_millis(10));

    let half = ITERATIONS / 2;
    let p1_done = Arc::new(AtomicBool::new(false));
    let p2_done = Arc::new(AtomicBool::new(false));
    let p1_done_clone = p1_done.clone();
    let p2_done_clone = p2_done.clone();

    // Producer threads
    let p1_handle = thread::spawn(move || {
        let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
        for _ in 0..half {
            while producer1.send(msg).is_err() {
                thread::yield_now();
            }
        }
        p1_done_clone.store(true, Ordering::Relaxed);
    });

    let p2_handle = thread::spawn(move || {
        let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
        for _ in 0..half {
            while producer2.send(msg).is_err() {
                thread::yield_now();
            }
        }
        p2_done_clone.store(true, Ordering::Relaxed);
    });

    let start = Instant::now();

    // Wait for both producers to finish
    while !p1_done.load(Ordering::Relaxed) || !p2_done.load(Ordering::Relaxed) {
        thread::yield_now();
    }
    let elapsed = start.elapsed();

    p1_handle.join().unwrap();
    p2_handle.join().unwrap();

    // Wait for messages to be consumed
    thread::sleep(Duration::from_millis(100));
    done.store(true, Ordering::Relaxed);
    c1_handle.join().unwrap();
    c2_handle.join().unwrap();

    let received = received_count.load(Ordering::Relaxed);
    if received < ITERATIONS {
        println!("  (Warning: MpmcIntra only received {}/{})", received, ITERATIONS);
    }

    elapsed.as_nanos() as f64 / ITERATIONS as f64
}

// ============================================================
// CROSS-PROCESS BENCHMARKS
// ============================================================

/// SpscShm - Cross-process 1P-1C
fn bench_spsc_shm() -> f64 {
    use std::process::{Command, Stdio};

    let topic_name = format!("bench_spsc_shm_{}", std::process::id());
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();

    // Spawn publisher child
    let mut child = Command::new(std::env::current_exe().unwrap())
        .args(["--child-publisher", &topic_name, &ITERATIONS.to_string()])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("Failed to spawn child");

    thread::sleep(Duration::from_millis(100));

    let start = Instant::now();
    let mut received = 0u64;
    let timeout = start + Duration::from_secs(TIMEOUT_SECS);
    while received < ITERATIONS && Instant::now() < timeout {
        if consumer.recv().is_some() {
            received += 1;
        }
    }
    let elapsed = start.elapsed();

    child.wait().ok();

    if received < ITERATIONS {
        println!(
            "  (Warning: SpscShm only received {}/{})",
            received, ITERATIONS
        );
    }

    elapsed.as_nanos() as f64 / received.max(1) as f64
}

/// MpscShm - Cross-process MP-1C (2 child publishers, 1 consumer in parent)
fn bench_mpsc_shm() -> f64 {
    use std::process::{Command, Stdio};

    let topic_name = format!("bench_mpsc_shm_{}", std::process::id());
    let consumer: Topic<CmdVel> = Topic::new(&topic_name).unwrap();

    let half = ITERATIONS / 2;

    // Spawn two publisher children
    let mut child1 = Command::new(std::env::current_exe().unwrap())
        .args(["--child-publisher", &topic_name, &half.to_string()])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("Failed to spawn child 1");

    let mut child2 = Command::new(std::env::current_exe().unwrap())
        .args(["--child-publisher", &topic_name, &half.to_string()])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("Failed to spawn child 2");

    thread::sleep(Duration::from_millis(100));

    let start = Instant::now();
    let mut received = 0u64;
    let timeout = start + Duration::from_secs(TIMEOUT_SECS);
    while received < ITERATIONS && Instant::now() < timeout {
        if consumer.recv().is_some() {
            received += 1;
        }
    }
    let elapsed = start.elapsed();

    child1.wait().ok();
    child2.wait().ok();

    if received < ITERATIONS {
        println!(
            "  (Warning: MpscShm only received {}/{})",
            received, ITERATIONS
        );
    }

    elapsed.as_nanos() as f64 / received.max(1) as f64
}

/// MpmcShm - Cross-process MPMC (2 child publishers, 2 consumer threads)
fn bench_mpmc_shm() -> f64 {
    use std::process::{Command, Stdio};

    let topic_name = format!("bench_mpmc_shm_{}", std::process::id());
    let consumer1: Topic<CmdVel> = Topic::new(&topic_name).unwrap();
    let consumer2: Topic<CmdVel> = Topic::new(&topic_name).unwrap();

    let half = ITERATIONS / 2;
    let received_count = Arc::new(AtomicU64::new(0));
    let rc1 = received_count.clone();
    let rc2 = received_count.clone();
    let done = Arc::new(AtomicBool::new(false));
    let done_c1 = done.clone();
    let done_c2 = done.clone();

    // Start consumer threads
    let c1_handle = thread::spawn(move || {
        while !done_c1.load(Ordering::Relaxed) {
            if consumer1.recv().is_some() {
                rc1.fetch_add(1, Ordering::Relaxed);
            } else {
                thread::yield_now();
            }
        }
        while consumer1.recv().is_some() {
            rc1.fetch_add(1, Ordering::Relaxed);
        }
    });

    let c2_handle = thread::spawn(move || {
        while !done_c2.load(Ordering::Relaxed) {
            if consumer2.recv().is_some() {
                rc2.fetch_add(1, Ordering::Relaxed);
            } else {
                thread::yield_now();
            }
        }
        while consumer2.recv().is_some() {
            rc2.fetch_add(1, Ordering::Relaxed);
        }
    });

    thread::sleep(Duration::from_millis(50));

    // Spawn two publisher children
    let start = Instant::now();
    let mut child1 = Command::new(std::env::current_exe().unwrap())
        .args(["--child-publisher", &topic_name, &half.to_string()])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("Failed to spawn child 1");

    let mut child2 = Command::new(std::env::current_exe().unwrap())
        .args(["--child-publisher", &topic_name, &half.to_string()])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .expect("Failed to spawn child 2");

    child1.wait().ok();
    child2.wait().ok();
    let elapsed = start.elapsed();

    thread::sleep(Duration::from_millis(100));
    done.store(true, Ordering::Relaxed);

    c1_handle.join().unwrap();
    c2_handle.join().unwrap();

    let received = received_count.load(Ordering::Relaxed);
    if received < ITERATIONS {
        println!(
            "  (Warning: MpmcShm only received {}/{})",
            received, ITERATIONS
        );
    }

    elapsed.as_nanos() as f64 / ITERATIONS as f64
}

// ============================================================
// DETAILED BREAKDOWN
// ============================================================

fn bench_direct_channel_detailed() {
    let topic: Topic<CmdVel> = Topic::new("bench_detailed").unwrap();

    // Warmup
    for _ in 0..WARMUP {
        let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
        let _ = topic.send(msg);
        let _ = topic.recv();
    }
    while topic.recv().is_some() {}

    println!("Backend type: {}", topic.backend_type());

    // Send only (pre-create message outside loop)
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let start = Instant::now();
    for _ in 0..ITERATIONS {
        let _ = topic.send(msg);
    }
    let send_time = start.elapsed();
    println!(
        "  Send only:     {:>6.1}ns",
        send_time.as_nanos() as f64 / ITERATIONS as f64
    );

    // Recv only (drain what we just sent)
    let start = Instant::now();
    let mut count = 0u64;
    while topic.recv().is_some() {
        count += 1;
    }
    let recv_time = start.elapsed();
    println!(
        "  Recv only:     {:>6.1}ns (got {})",
        recv_time.as_nanos() as f64 / count as f64,
        count
    );

    // With black_box
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let start = Instant::now();
    for _ in 0..ITERATIONS {
        let _ = std::hint::black_box(topic.send(std::hint::black_box(msg)));
    }
    let send_bb = start.elapsed();
    println!(
        "  Send (bb):     {:>6.1}ns",
        send_bb.as_nanos() as f64 / ITERATIONS as f64
    );

    while topic.recv().is_some() {}

    // Round-trip with black_box
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let start = Instant::now();
    for _ in 0..ITERATIONS {
        let _ = std::hint::black_box(topic.send(std::hint::black_box(msg)));
        let _ = std::hint::black_box(topic.recv());
    }
    let rt_time = start.elapsed();
    println!(
        "  Round-trip:    {:>6.1}ns",
        rt_time.as_nanos() as f64 / ITERATIONS as f64
    );

    // Compare with SystemTime overhead (using new())
    println!("\n=== SystemTime::now() Overhead Comparison ===");
    let start = Instant::now();
    for _ in 0..ITERATIONS {
        let msg = CmdVel::new(1.5, 0.8); // Has SystemTime::now() call
        let _ = topic.send(msg);
    }
    let with_syscall = start.elapsed();
    println!(
        "  Send with new():          {:>6.1}ns (includes ~52ns syscall)",
        with_syscall.as_nanos() as f64 / ITERATIONS as f64
    );

    while topic.recv().is_some() {}

    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);
    let start = Instant::now();
    for _ in 0..ITERATIONS {
        let _ = topic.send(msg);
    }
    let without_syscall = start.elapsed();
    println!(
        "  Send with with_timestamp(): {:>6.1}ns (pure IPC)",
        without_syscall.as_nanos() as f64 / ITERATIONS as f64
    );

    let overhead =
        with_syscall.as_nanos() as f64 / ITERATIONS as f64 - without_syscall.as_nanos() as f64 / ITERATIONS as f64;
    println!("  SystemTime overhead:      {:>6.1}ns", overhead);
}

// ============================================================
// CHILD PROCESS ENTRY POINTS
// ============================================================

fn run_child_publisher(topic_name: &str, count: u64) {
    let topic: Topic<CmdVel> = Topic::new(topic_name).unwrap();
    let msg = CmdVel::with_timestamp(1.5, 0.8, 0);

    for _ in 0..count {
        while topic.send(msg).is_err() {
            std::hint::spin_loop();
        }
    }
}

fn run_child_consumer(topic_name: &str, count: u64) {
    let topic: Topic<CmdVel> = Topic::new(topic_name).unwrap();

    let mut received = 0u64;
    while received < count {
        if topic.recv().is_some() {
            received += 1;
        } else {
            std::hint::spin_loop();
        }
    }
}
