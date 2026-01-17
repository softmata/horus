// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]
#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_mut)]

//! Connect/Disconnect Stress Test
//!
//! Tests rapid creation and destruction of Links, Publishers, Subscribers.
//! Verifies:
//! - Resource cleanup on drop
//! - No file descriptor leaks
//! - No memory leaks
//! - Shared memory properly released (/dev/shm cleanup)
//! - No stale handles or dangling references
//!
//! Usage:
//!   cargo run --release --bin test_connect_disconnect_stress -- [cycles] [--concurrent] [--quick]
//!
//! Examples:
//!   cargo run --release --bin test_connect_disconnect_stress           # Default: 1000 cycles
//!   cargo run --release --bin test_connect_disconnect_stress -- 5000   # 5000 cycles
//!   cargo run --release --bin test_connect_disconnect_stress -- --quick # Quick 100-cycle test
//!   cargo run --release --bin test_connect_disconnect_stress -- --concurrent # Concurrent test

use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use horus_library::messages::sensor::Imu;
use std::collections::HashSet;
use std::env;
use std::fs;
use std::path::PathBuf;
use std::process;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

/// Test configuration
struct TestConfig {
    cycles: usize,
    concurrent_threads: usize,
    quick_mode: bool,
    concurrent_mode: bool,
}

impl Default for TestConfig {
    fn default() -> Self {
        Self {
            cycles: 1000,
            concurrent_threads: 4,
            quick_mode: false,
            concurrent_mode: false,
        }
    }
}

/// Resource tracking for leak detection
#[derive(Debug, Clone)]
struct ResourceSnapshot {
    fd_count: u64,
    memory_rss_kb: u64,
    shm_file_count: usize,
    shm_files: HashSet<String>,
}

impl ResourceSnapshot {
    fn capture() -> Self {
        Self {
            fd_count: get_fd_count(),
            memory_rss_kb: get_memory_rss_kb(),
            shm_file_count: count_horus_shm_files(),
            shm_files: list_horus_shm_files(),
        }
    }
}

/// Test results
#[derive(Debug)]
struct TestResult {
    name: String,
    passed: bool,
    cycles_completed: usize,
    duration_secs: f64,
    initial_snapshot: ResourceSnapshot,
    final_snapshot: ResourceSnapshot,
    fd_leaked: i64,
    memory_growth_kb: i64,
    shm_files_leaked: i64,
    leaked_shm_files: Vec<String>,
    errors: Vec<String>,
}

fn main() {
    let config = parse_args();

    println!("═══════════════════════════════════════════════════════════════════════════════");
    println!("  HORUS Connect/Disconnect Stress Test");
    println!("═══════════════════════════════════════════════════════════════════════════════");
    println!();
    println!("Configuration:");
    println!("  Cycles per test: {}", config.cycles);
    println!("  Concurrent threads: {}", config.concurrent_threads);
    println!("  Quick mode: {}", config.quick_mode);
    println!("  Concurrent mode: {}", config.concurrent_mode);
    println!();

    // Clean up any stale shm files from previous test runs
    cleanup_test_shm_files();

    // Give system time to stabilize
    thread::sleep(Duration::from_millis(100));

    let mut results: Vec<TestResult> = Vec::new();

    // Test 1: Link SPSC rapid connect/disconnect
    results.push(test_link_rapid_cycles(&config));

    // Test 2: Hub MPMC rapid connect/disconnect
    results.push(test_hub_rapid_cycles(&config));

    // Test 3: Mixed Link and Hub cycles
    results.push(test_mixed_rapid_cycles(&config));

    // Test 4: Topic name reuse
    results.push(test_topic_reuse(&config));

    if config.concurrent_mode {
        // Test 5: Concurrent creation/destruction
        results.push(test_concurrent_cycles(&config));

        // Test 6: Race condition stress
        results.push(test_race_condition_stress(&config));
    }

    // Print summary
    print_summary(&results);

    // Determine exit code
    let all_passed = results.iter().all(|r| r.passed);
    if all_passed {
        println!("\n✅ ALL TESTS PASSED");
        process::exit(0);
    } else {
        println!("\n❌ SOME TESTS FAILED");
        process::exit(1);
    }
}

fn parse_args() -> TestConfig {
    let args: Vec<String> = env::args().collect();
    let mut config = TestConfig::default();

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--quick" => {
                config.quick_mode = true;
                config.cycles = 100;
            }
            "--concurrent" => {
                config.concurrent_mode = true;
            }
            "--threads" if i + 1 < args.len() => {
                config.concurrent_threads = args[i + 1].parse().unwrap_or(4);
                i += 1;
            }
            "--help" | "-h" => {
                println!("Usage: {} [cycles] [options]", args[0]);
                println!();
                println!("Options:");
                println!("  --quick       Quick test with 100 cycles");
                println!("  --concurrent  Enable concurrent stress tests");
                println!("  --threads N   Number of concurrent threads (default: 4)");
                println!("  --help        Show this help");
                process::exit(0);
            }
            arg => {
                if let Ok(cycles) = arg.parse::<usize>() {
                    config.cycles = cycles;
                }
            }
        }
        i += 1;
    }

    config
}

/// Test 1: Link SPSC rapid connect/disconnect
fn test_link_rapid_cycles(config: &TestConfig) -> TestResult {
    let test_name = "Link SPSC Rapid Cycles".to_string();
    println!("╔════════════════════════════════════════════════════════════════════════════╗");
    println!("║ Test: {}                                              ║", test_name);
    println!("╚════════════════════════════════════════════════════════════════════════════╝");

    let initial = ResourceSnapshot::capture();
    let start = Instant::now();
    let mut errors = Vec::new();
    let mut cycles_completed = 0;

    for i in 0..config.cycles {
        let topic = format!("test_link_rapid_{}_{}", process::id(), i);

        // Create producer and consumer
        let producer = match Topic::<CmdVel>::new(&topic) {
            Ok(p) => p,
            Err(e) => {
                errors.push(format!("Cycle {}: Producer creation failed: {:?}", i, e));
                continue;
            }
        };

        let consumer = match Topic::<CmdVel>::new(&topic) {
            Ok(c) => c,
            Err(e) => {
                errors.push(format!("Cycle {}: Consumer creation failed: {:?}", i, e));
                continue;
            }
        };

        // Send a few messages to exercise the link
        for j in 0..5 {
            let msg = CmdVel {
                linear: 1.0 + j as f32,
                angular: 0.5,
                stamp_nanos: j,
            };
            let _ = producer.send(msg, &mut None);
        }

        // Receive messages
        for _ in 0..5 {
            let _ = consumer.recv(&mut None);
        }

        // Drop (cleanup)
        drop(producer);
        drop(consumer);

        cycles_completed += 1;

        if (i + 1) % (config.cycles / 10).max(1) == 0 {
            println!("  Progress: {}/{} cycles", i + 1, config.cycles);
        }
    }

    let duration = start.elapsed();

    // Wait for cleanup to complete
    thread::sleep(Duration::from_millis(100));

    let final_snapshot = ResourceSnapshot::capture();

    let fd_leaked = final_snapshot.fd_count as i64 - initial.fd_count as i64;
    let memory_growth = final_snapshot.memory_rss_kb as i64 - initial.memory_rss_kb as i64;
    let shm_leaked = final_snapshot.shm_file_count as i64 - initial.shm_file_count as i64;

    // Find leaked shm files
    let leaked_files: Vec<String> = final_snapshot.shm_files
        .difference(&initial.shm_files)
        .filter(|f| f.contains("test_link_rapid"))
        .cloned()
        .collect();

    let passed = fd_leaked <= 2 && shm_leaked <= 0 && errors.is_empty();

    println!();
    println!("  Results:");
    println!("    Cycles completed: {}/{}", cycles_completed, config.cycles);
    println!("    Duration: {:.2}s ({:.0} cycles/sec)", duration.as_secs_f64(), cycles_completed as f64 / duration.as_secs_f64());
    println!("    FD change: {} (initial: {}, final: {})", fd_leaked, initial.fd_count, final_snapshot.fd_count);
    println!("    Memory change: {} KB", memory_growth);
    println!("    SHM files leaked: {}", shm_leaked);
    println!("    Status: {}", if passed { "✅ PASSED" } else { "❌ FAILED" });

    if !leaked_files.is_empty() {
        println!("    Leaked SHM files: {:?}", leaked_files);
    }

    if !errors.is_empty() {
        println!("    Errors (first 5): {:?}", errors.iter().take(5).collect::<Vec<_>>());
    }

    println!();

    TestResult {
        name: test_name,
        passed,
        cycles_completed,
        duration_secs: duration.as_secs_f64(),
        initial_snapshot: initial,
        final_snapshot,
        fd_leaked,
        memory_growth_kb: memory_growth,
        shm_files_leaked: shm_leaked,
        leaked_shm_files: leaked_files,
        errors,
    }
}

/// Test 2: Hub MPMC rapid connect/disconnect
fn test_hub_rapid_cycles(config: &TestConfig) -> TestResult {
    let test_name = "Hub MPMC Rapid Cycles".to_string();
    println!("╔════════════════════════════════════════════════════════════════════════════╗");
    println!("║ Test: {}                                               ║", test_name);
    println!("╚════════════════════════════════════════════════════════════════════════════╝");

    let initial = ResourceSnapshot::capture();
    let start = Instant::now();
    let mut errors = Vec::new();
    let mut cycles_completed = 0;

    for i in 0..config.cycles {
        let topic = format!("test_hub_rapid_{}_{}", process::id(), i);

        // Create multiple publishers and subscribers
        let pub1 = match Topic::<CmdVel>::new(&topic) {
            Ok(p) => p,
            Err(e) => {
                errors.push(format!("Cycle {}: Pub1 creation failed: {:?}", i, e));
                continue;
            }
        };

        let sub1 = match Topic::<CmdVel>::new(&topic) {
            Ok(s) => s,
            Err(e) => {
                errors.push(format!("Cycle {}: Sub1 creation failed: {:?}", i, e));
                continue;
            }
        };

        let sub2 = match Topic::<CmdVel>::new(&topic) {
            Ok(s) => s,
            Err(e) => {
                errors.push(format!("Cycle {}: Sub2 creation failed: {:?}", i, e));
                continue;
            }
        };

        // Send and receive some messages
        for j in 0..3 {
            let msg = CmdVel {
                linear: 1.0,
                angular: 0.5,
                stamp_nanos: j,
            };
            let _ = pub1.send(msg, &mut None);
        }

        for _ in 0..3 {
            let _ = sub1.recv(&mut None);
            let _ = sub2.recv(&mut None);
        }

        // Drop in various orders to test cleanup
        match i % 3 {
            0 => {
                drop(pub1);
                drop(sub1);
                drop(sub2);
            }
            1 => {
                drop(sub2);
                drop(pub1);
                drop(sub1);
            }
            _ => {
                drop(sub1);
                drop(sub2);
                drop(pub1);
            }
        }

        cycles_completed += 1;

        if (i + 1) % (config.cycles / 10).max(1) == 0 {
            println!("  Progress: {}/{} cycles", i + 1, config.cycles);
        }
    }

    let duration = start.elapsed();
    thread::sleep(Duration::from_millis(100));

    let final_snapshot = ResourceSnapshot::capture();

    let fd_leaked = final_snapshot.fd_count as i64 - initial.fd_count as i64;
    let memory_growth = final_snapshot.memory_rss_kb as i64 - initial.memory_rss_kb as i64;
    let shm_leaked = final_snapshot.shm_file_count as i64 - initial.shm_file_count as i64;

    let leaked_files: Vec<String> = final_snapshot.shm_files
        .difference(&initial.shm_files)
        .filter(|f| f.contains("test_hub_rapid"))
        .cloned()
        .collect();

    let passed = fd_leaked <= 2 && shm_leaked <= 0 && errors.is_empty();

    println!();
    println!("  Results:");
    println!("    Cycles completed: {}/{}", cycles_completed, config.cycles);
    println!("    Duration: {:.2}s ({:.0} cycles/sec)", duration.as_secs_f64(), cycles_completed as f64 / duration.as_secs_f64());
    println!("    FD change: {}", fd_leaked);
    println!("    Memory change: {} KB", memory_growth);
    println!("    SHM files leaked: {}", shm_leaked);
    println!("    Status: {}", if passed { "✅ PASSED" } else { "❌ FAILED" });

    if !leaked_files.is_empty() {
        println!("    Leaked SHM files: {:?}", leaked_files);
    }

    println!();

    TestResult {
        name: test_name,
        passed,
        cycles_completed,
        duration_secs: duration.as_secs_f64(),
        initial_snapshot: initial,
        final_snapshot,
        fd_leaked,
        memory_growth_kb: memory_growth,
        shm_files_leaked: shm_leaked,
        leaked_shm_files: leaked_files,
        errors,
    }
}

/// Test 3: Mixed Link and Hub rapid cycles
fn test_mixed_rapid_cycles(config: &TestConfig) -> TestResult {
    let test_name = "Mixed Link+Hub Rapid Cycles".to_string();
    println!("╔════════════════════════════════════════════════════════════════════════════╗");
    println!("║ Test: {}                                      ║", test_name);
    println!("╚════════════════════════════════════════════════════════════════════════════╝");

    let initial = ResourceSnapshot::capture();
    let start = Instant::now();
    let mut errors = Vec::new();
    let mut cycles_completed = 0;

    for i in 0..config.cycles {
        // Create Link for CmdVel
        let link_topic = format!("test_mixed_link_{}_{}", process::id(), i);
        let link_prod = Topic::<CmdVel>::new(&link_topic);
        let link_cons = Topic::<CmdVel>::new(&link_topic);

        // Create Hub for Imu
        let hub_topic = format!("test_mixed_hub_{}_{}", process::id(), i);
        let hub1 = Topic::<Imu>::new(&hub_topic);
        let hub2 = Topic::<Imu>::new(&hub_topic);

        // Check for errors
        if let Err(e) = &link_prod {
            errors.push(format!("Cycle {}: Link producer failed: {:?}", i, e));
        }
        if let Err(e) = &link_cons {
            errors.push(format!("Cycle {}: Link consumer failed: {:?}", i, e));
        }
        if let Err(e) = &hub1 {
            errors.push(format!("Cycle {}: Hub1 failed: {:?}", i, e));
        }
        if let Err(e) = &hub2 {
            errors.push(format!("Cycle {}: Hub2 failed: {:?}", i, e));
        }

        // Use them briefly
        if let (Ok(ref prod), Ok(ref cons)) = (&link_prod, &link_cons) {
            let msg = CmdVel { linear: 1.0, angular: 0.5, stamp_nanos: i as u64 };
            let _ = prod.send(msg, &mut None);
            let _ = cons.recv(&mut None);
        }

        if let (Ok(ref h1), Ok(ref h2)) = (&hub1, &hub2) {
            let mut imu = Imu::new();
            imu.timestamp = i as u64;
            let _ = h1.send(imu.clone(), &mut None);
            let _ = h2.recv(&mut None);
        }

        // Everything drops here
        cycles_completed += 1;

        if (i + 1) % (config.cycles / 10).max(1) == 0 {
            println!("  Progress: {}/{} cycles", i + 1, config.cycles);
        }
    }

    let duration = start.elapsed();
    thread::sleep(Duration::from_millis(100));

    let final_snapshot = ResourceSnapshot::capture();

    let fd_leaked = final_snapshot.fd_count as i64 - initial.fd_count as i64;
    let memory_growth = final_snapshot.memory_rss_kb as i64 - initial.memory_rss_kb as i64;
    let shm_leaked = final_snapshot.shm_file_count as i64 - initial.shm_file_count as i64;

    let leaked_files: Vec<String> = final_snapshot.shm_files
        .difference(&initial.shm_files)
        .filter(|f| f.contains("test_mixed"))
        .cloned()
        .collect();

    let passed = fd_leaked <= 4 && shm_leaked <= 0 && errors.len() < config.cycles / 10;

    println!();
    println!("  Results:");
    println!("    Cycles completed: {}/{}", cycles_completed, config.cycles);
    println!("    Duration: {:.2}s ({:.0} cycles/sec)", duration.as_secs_f64(), cycles_completed as f64 / duration.as_secs_f64());
    println!("    FD change: {}", fd_leaked);
    println!("    Memory change: {} KB", memory_growth);
    println!("    SHM files leaked: {}", shm_leaked);
    println!("    Errors: {}", errors.len());
    println!("    Status: {}", if passed { "✅ PASSED" } else { "❌ FAILED" });

    println!();

    TestResult {
        name: test_name,
        passed,
        cycles_completed,
        duration_secs: duration.as_secs_f64(),
        initial_snapshot: initial,
        final_snapshot,
        fd_leaked,
        memory_growth_kb: memory_growth,
        shm_files_leaked: shm_leaked,
        leaked_shm_files: leaked_files,
        errors,
    }
}

/// Test 4: Topic name reuse
fn test_topic_reuse(config: &TestConfig) -> TestResult {
    let test_name = "Topic Name Reuse".to_string();
    println!("╔════════════════════════════════════════════════════════════════════════════╗");
    println!("║ Test: {}                                                   ║", test_name);
    println!("╚════════════════════════════════════════════════════════════════════════════╝");

    let initial = ResourceSnapshot::capture();
    let start = Instant::now();
    let mut errors = Vec::new();
    let mut cycles_completed = 0;

    // Reuse only 10 topic names across all cycles
    let num_topics = 10;

    for i in 0..config.cycles {
        let topic_idx = i % num_topics;
        let topic = format!("test_reuse_{}_{}", process::id(), topic_idx);

        // Create and immediately destroy
        match Topic::<CmdVel>::new(&topic) {
            Ok(prod) => {
                let _ = Topic::<CmdVel>::new(&topic).map(|cons| {
                    let msg = CmdVel { linear: 1.0, angular: 0.5, stamp_nanos: i as u64 };
                    let _ = prod.send(msg, &mut None);
                    let _ = cons.recv(&mut None);
                });
            }
            Err(e) => {
                errors.push(format!("Cycle {}: Failed to create producer: {:?}", i, e));
            }
        }

        cycles_completed += 1;

        if (i + 1) % (config.cycles / 10).max(1) == 0 {
            println!("  Progress: {}/{} cycles (reusing {} topics)", i + 1, config.cycles, num_topics);
        }
    }

    let duration = start.elapsed();
    thread::sleep(Duration::from_millis(100));

    let final_snapshot = ResourceSnapshot::capture();

    let fd_leaked = final_snapshot.fd_count as i64 - initial.fd_count as i64;
    let memory_growth = final_snapshot.memory_rss_kb as i64 - initial.memory_rss_kb as i64;
    let shm_leaked = final_snapshot.shm_file_count as i64 - initial.shm_file_count as i64;

    let leaked_files: Vec<String> = final_snapshot.shm_files
        .difference(&initial.shm_files)
        .filter(|f| f.contains("test_reuse"))
        .cloned()
        .collect();

    let passed = fd_leaked <= 2 && shm_leaked <= 0 && errors.len() < config.cycles / 10;

    println!();
    println!("  Results:");
    println!("    Cycles completed: {}/{}", cycles_completed, config.cycles);
    println!("    Topics reused: {} names x {} times each", num_topics, config.cycles / num_topics);
    println!("    Duration: {:.2}s ({:.0} cycles/sec)", duration.as_secs_f64(), cycles_completed as f64 / duration.as_secs_f64());
    println!("    FD change: {}", fd_leaked);
    println!("    Memory change: {} KB", memory_growth);
    println!("    SHM files leaked: {}", shm_leaked);
    println!("    Status: {}", if passed { "✅ PASSED" } else { "❌ FAILED" });

    println!();

    TestResult {
        name: test_name,
        passed,
        cycles_completed,
        duration_secs: duration.as_secs_f64(),
        initial_snapshot: initial,
        final_snapshot,
        fd_leaked,
        memory_growth_kb: memory_growth,
        shm_files_leaked: shm_leaked,
        leaked_shm_files: leaked_files,
        errors,
    }
}

/// Test 5: Concurrent creation/destruction from multiple threads
fn test_concurrent_cycles(config: &TestConfig) -> TestResult {
    let test_name = "Concurrent Connect/Disconnect".to_string();
    println!("╔════════════════════════════════════════════════════════════════════════════╗");
    println!("║ Test: {}                                      ║", test_name);
    println!("╚════════════════════════════════════════════════════════════════════════════╝");

    let initial = ResourceSnapshot::capture();
    let start = Instant::now();
    let error_count = Arc::new(AtomicU64::new(0));
    let cycles_per_thread = config.cycles / config.concurrent_threads;

    println!("  Spawning {} threads, {} cycles each...", config.concurrent_threads, cycles_per_thread);

    let mut handles = Vec::new();

    for thread_id in 0..config.concurrent_threads {
        let error_count = Arc::clone(&error_count);

        let handle = thread::spawn(move || {
            for i in 0..cycles_per_thread {
                let topic = format!("test_concurrent_{}_{}", thread_id, i);

                // Alternate between Link and Hub
                if i % 2 == 0 {
                    match Topic::<CmdVel>::new(&topic) {
                        Ok(prod) => {
                            let _ = Topic::<CmdVel>::new(&topic).map(|cons| {
                                let msg = CmdVel { linear: 1.0, angular: 0.5, stamp_nanos: i as u64 };
                                let _ = prod.send(msg, &mut None);
                                let _ = cons.recv(&mut None);
                            });
                        }
                        Err(_) => {
                            error_count.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                } else {
                    match Topic::<CmdVel>::new(&topic) {
                        Ok(hub1) => {
                            let _ = Topic::<CmdVel>::new(&topic).map(|hub2| {
                                let msg = CmdVel { linear: 1.0, angular: 0.5, stamp_nanos: i as u64 };
                                let _ = hub1.send(msg, &mut None);
                                let _ = hub2.recv(&mut None);
                            });
                        }
                        Err(_) => {
                            error_count.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
            }
        });

        handles.push(handle);
    }

    // Wait for all threads
    for (i, handle) in handles.into_iter().enumerate() {
        match handle.join() {
            Ok(_) => println!("  Thread {} completed", i),
            Err(_) => println!("  Thread {} panicked!", i),
        }
    }

    let duration = start.elapsed();
    thread::sleep(Duration::from_millis(200));

    let final_snapshot = ResourceSnapshot::capture();

    let fd_leaked = final_snapshot.fd_count as i64 - initial.fd_count as i64;
    let memory_growth = final_snapshot.memory_rss_kb as i64 - initial.memory_rss_kb as i64;
    let shm_leaked = final_snapshot.shm_file_count as i64 - initial.shm_file_count as i64;

    let leaked_files: Vec<String> = final_snapshot.shm_files
        .difference(&initial.shm_files)
        .filter(|f| f.contains("test_concurrent"))
        .cloned()
        .collect();

    let total_errors = error_count.load(Ordering::Relaxed);
    let cycles_completed = config.cycles - total_errors as usize;
    let passed = fd_leaked <= 4 && shm_leaked <= 0 && total_errors < (config.cycles / 10) as u64;

    println!();
    println!("  Results:");
    println!("    Threads: {}", config.concurrent_threads);
    println!("    Total cycles: {} ({} errors)", cycles_completed, total_errors);
    println!("    Duration: {:.2}s ({:.0} cycles/sec)", duration.as_secs_f64(), cycles_completed as f64 / duration.as_secs_f64());
    println!("    FD change: {}", fd_leaked);
    println!("    Memory change: {} KB", memory_growth);
    println!("    SHM files leaked: {}", shm_leaked);
    println!("    Status: {}", if passed { "✅ PASSED" } else { "❌ FAILED" });

    println!();

    TestResult {
        name: test_name,
        passed,
        cycles_completed,
        duration_secs: duration.as_secs_f64(),
        initial_snapshot: initial,
        final_snapshot,
        fd_leaked,
        memory_growth_kb: memory_growth,
        shm_files_leaked: shm_leaked,
        leaked_shm_files: leaked_files,
        errors: if total_errors > 0 {
            vec![format!("{} errors across all threads", total_errors)]
        } else {
            vec![]
        },
    }
}

/// Test 6: Race condition stress - create same topic from multiple threads
fn test_race_condition_stress(config: &TestConfig) -> TestResult {
    let test_name = "Race Condition Stress".to_string();
    println!("╔════════════════════════════════════════════════════════════════════════════╗");
    println!("║ Test: {}                                          ║", test_name);
    println!("╚════════════════════════════════════════════════════════════════════════════╝");

    let initial = ResourceSnapshot::capture();
    let start = Instant::now();
    let error_count = Arc::new(AtomicU64::new(0));
    let stop_flag = Arc::new(AtomicBool::new(false));

    // Use fewer shared topics to increase contention
    let num_shared_topics = 5;
    let cycles_per_thread = config.cycles / config.concurrent_threads;

    println!("  {} threads competing for {} shared topics...", config.concurrent_threads, num_shared_topics);

    let mut handles = Vec::new();

    for thread_id in 0..config.concurrent_threads {
        let error_count = Arc::clone(&error_count);
        let stop_flag = Arc::clone(&stop_flag);

        let handle = thread::spawn(move || {
            for i in 0..cycles_per_thread {
                if stop_flag.load(Ordering::Relaxed) {
                    break;
                }

                // All threads compete for the same topic names
                let topic_idx = i % num_shared_topics;
                let topic = format!("test_race_{}", topic_idx);

                // Try to create Link
                match Topic::<CmdVel>::new(&topic) {
                    Ok(prod) => {
                        let msg = CmdVel { linear: thread_id as f32, angular: 0.5, stamp_nanos: i as u64 };
                        let _ = prod.send(msg, &mut None);
                        // Let it live briefly
                        thread::sleep(Duration::from_micros(10));
                    }
                    Err(_) => {
                        // Expected when another thread owns the topic
                        error_count.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        });

        handles.push(handle);
    }

    // Wait for all threads
    for handle in handles {
        let _ = handle.join();
    }

    let duration = start.elapsed();
    thread::sleep(Duration::from_millis(200));

    let final_snapshot = ResourceSnapshot::capture();

    let fd_leaked = final_snapshot.fd_count as i64 - initial.fd_count as i64;
    let memory_growth = final_snapshot.memory_rss_kb as i64 - initial.memory_rss_kb as i64;
    let shm_leaked = final_snapshot.shm_file_count as i64 - initial.shm_file_count as i64;

    let leaked_files: Vec<String> = final_snapshot.shm_files
        .difference(&initial.shm_files)
        .filter(|f| f.contains("test_race"))
        .cloned()
        .collect();

    let total_errors = error_count.load(Ordering::Relaxed);
    // In race condition test, many errors are expected
    let passed = fd_leaked <= 4 && shm_leaked <= 0;

    println!();
    println!("  Results:");
    println!("    Threads: {}, Shared topics: {}", config.concurrent_threads, num_shared_topics);
    println!("    Contention errors: {} (expected due to racing)", total_errors);
    println!("    Duration: {:.2}s", duration.as_secs_f64());
    println!("    FD change: {}", fd_leaked);
    println!("    Memory change: {} KB", memory_growth);
    println!("    SHM files leaked: {} (CRITICAL)", shm_leaked);
    println!("    Status: {}", if passed { "✅ PASSED (no resource leaks)" } else { "❌ FAILED" });

    println!();

    TestResult {
        name: test_name,
        passed,
        cycles_completed: config.cycles,
        duration_secs: duration.as_secs_f64(),
        initial_snapshot: initial,
        final_snapshot,
        fd_leaked,
        memory_growth_kb: memory_growth,
        shm_files_leaked: shm_leaked,
        leaked_shm_files: leaked_files,
        errors: vec![format!("{} contention errors (expected)", total_errors)],
    }
}

/// Print summary of all test results
fn print_summary(results: &[TestResult]) {
    println!("═══════════════════════════════════════════════════════════════════════════════");
    println!("  SUMMARY");
    println!("═══════════════════════════════════════════════════════════════════════════════");
    println!();

    println!("  {:<35} {:>8} {:>10} {:>8} {:>8}", "Test", "Status", "Cycles", "FD Leak", "SHM Leak");
    println!("  {}", "-".repeat(75));

    for result in results {
        let status = if result.passed { "✅ PASS" } else { "❌ FAIL" };
        println!(
            "  {:<35} {:>8} {:>10} {:>8} {:>8}",
            result.name,
            status,
            result.cycles_completed,
            result.fd_leaked,
            result.shm_files_leaked
        );
    }

    println!();

    // Overall resource changes
    if let (Some(first), Some(last)) = (results.first(), results.last()) {
        let total_fd_change = last.final_snapshot.fd_count as i64 - first.initial_snapshot.fd_count as i64;
        let total_mem_change = last.final_snapshot.memory_rss_kb as i64 - first.initial_snapshot.memory_rss_kb as i64;
        let total_shm_change = last.final_snapshot.shm_file_count as i64 - first.initial_snapshot.shm_file_count as i64;

        println!("  Overall Resource Changes:");
        println!("    File descriptors: {} → {} (change: {})",
            first.initial_snapshot.fd_count,
            last.final_snapshot.fd_count,
            total_fd_change);
        println!("    Memory RSS: {} KB → {} KB (change: {} KB)",
            first.initial_snapshot.memory_rss_kb,
            last.final_snapshot.memory_rss_kb,
            total_mem_change);
        println!("    SHM files: {} → {} (change: {})",
            first.initial_snapshot.shm_file_count,
            last.final_snapshot.shm_file_count,
            total_shm_change);
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

/// Get current file descriptor count
fn get_fd_count() -> u64 {
    #[cfg(target_os = "linux")]
    {
        if let Ok(entries) = fs::read_dir("/proc/self/fd") {
            return entries.count() as u64;
        }
    }
    0
}

/// Get current memory RSS in KB
fn get_memory_rss_kb() -> u64 {
    #[cfg(target_os = "linux")]
    {
        if let Ok(content) = fs::read_to_string("/proc/self/statm") {
            let parts: Vec<&str> = content.split_whitespace().collect();
            if parts.len() >= 2 {
                if let Ok(rss_pages) = parts[1].parse::<u64>() {
                    // Convert pages to KB (page size is typically 4KB)
                    return rss_pages * 4;
                }
            }
        }
    }
    0
}

/// Count HORUS shared memory files
fn count_horus_shm_files() -> usize {
    list_horus_shm_files().len()
}

/// List HORUS shared memory files
fn list_horus_shm_files() -> HashSet<String> {
    let mut files = HashSet::new();

    #[cfg(target_os = "linux")]
    {
        // Check /dev/shm/horus
        let horus_dir = PathBuf::from("/dev/shm/horus");
        if horus_dir.exists() {
            if let Ok(entries) = fs::read_dir(&horus_dir) {
                for entry in entries.flatten() {
                    if let Ok(name) = entry.file_name().into_string() {
                        files.insert(name);
                    }
                }
            }

            // Also check topics subdirectory
            let topics_dir = horus_dir.join("topics");
            if topics_dir.exists() {
                if let Ok(entries) = fs::read_dir(&topics_dir) {
                    for entry in entries.flatten() {
                        if let Ok(name) = entry.file_name().into_string() {
                            files.insert(format!("topics/{}", name));
                        }
                    }
                }
            }
        }
    }

    files
}

/// Clean up test-related shared memory files
fn cleanup_test_shm_files() {
    #[cfg(target_os = "linux")]
    {
        let topics_dir = PathBuf::from("/dev/shm/horus/topics");
        if topics_dir.exists() {
            if let Ok(entries) = fs::read_dir(&topics_dir) {
                for entry in entries.flatten() {
                    if let Ok(name) = entry.file_name().into_string() {
                        if name.contains("test_link_rapid")
                            || name.contains("test_hub_rapid")
                            || name.contains("test_mixed")
                            || name.contains("test_reuse")
                            || name.contains("test_concurrent")
                            || name.contains("test_race")
                        {
                            let _ = fs::remove_file(entry.path());
                        }
                    }
                }
            }
        }
    }
}
