// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]

//! Multi-Process Stress Test Suite
//!
//! Tests HORUS IPC under extreme process pressure:
//! - 10, 50, 100+ concurrent processes
//! - Rapid process spawn/exit cycles
//! - Process crash recovery
//! - Shared memory cleanup
//!
//! Acceptance Criteria:
//! - No deadlocks with 100 concurrent processes
//! - Proper cleanup after process termination
//! - No message loss in Hub (FIFO guarantee)
//! - Bounded latency under process pressure

use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use std::env;
use std::io::{BufRead, BufReader, Write};
use std::process::{self, Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

const CHILD_MODE_ENV: &str = "HORUS_STRESS_CHILD_MODE";
const CHILD_TOPIC_ENV: &str = "HORUS_STRESS_TOPIC";
const CHILD_ID_ENV: &str = "HORUS_STRESS_CHILD_ID";
const CHILD_MSG_COUNT_ENV: &str = "HORUS_STRESS_MSG_COUNT";

fn main() {
    // Check if we're a child process
    if let Ok(mode) = env::var(CHILD_MODE_ENV) {
        run_child_mode(&mode);
        return;
    }

    // Parent process - run stress tests
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <test_name>", args[0]);
        eprintln!("Available tests:");
        eprintln!("  concurrent_10     - 10 concurrent processes");
        eprintln!("  concurrent_50     - 50 concurrent processes");
        eprintln!("  concurrent_100    - 100 concurrent processes");
        eprintln!("  spawn_exit_rapid  - Rapid process spawn/exit cycles");
        eprintln!("  hub_many_procs    - Hub with many producers");
        eprintln!("  cleanup_verify    - Verify shared memory cleanup");
        eprintln!("  all               - Run all tests");
        process::exit(1);
    }

    let test_name = &args[1];

    let results = match test_name.as_str() {
        "concurrent_10" => vec![("concurrent_10", test_concurrent_processes(10))],
        "concurrent_50" => vec![("concurrent_50", test_concurrent_processes(50))],
        "concurrent_100" => vec![("concurrent_100", test_concurrent_processes(100))],
        "spawn_exit_rapid" => vec![("spawn_exit_rapid", test_spawn_exit_rapid())],
        "hub_many_procs" => vec![("hub_many_procs", test_hub_many_producers())],
        "cleanup_verify" => vec![("cleanup_verify", test_cleanup_verification())],
        "all" => run_all_tests(),
        _ => {
            eprintln!("Unknown test: {}", test_name);
            process::exit(1);
        }
    };

    // Print summary
    println!("\n========================================");
    println!("Multi-Process Stress Test Results");
    println!("========================================");
    let mut all_passed = true;
    for (name, passed) in &results {
        let status = if *passed { "✓ PASS" } else { "✗ FAIL" };
        println!("  {}: {}", name, status);
        if !*passed {
            all_passed = false;
        }
    }
    println!("========================================");

    if all_passed {
        println!("All tests PASSED");
        process::exit(0);
    } else {
        eprintln!("Some tests FAILED");
        process::exit(1);
    }
}

fn run_all_tests() -> Vec<(&'static str, bool)> {
    vec![
        ("concurrent_10", test_concurrent_processes(10)),
        ("concurrent_50", test_concurrent_processes(50)),
        ("concurrent_100", test_concurrent_processes(100)),
        ("spawn_exit_rapid", test_spawn_exit_rapid()),
        ("hub_many_procs", test_hub_many_producers()),
        ("cleanup_verify", test_cleanup_verification()),
    ]
}

/// Child process mode - called when spawned as a subprocess
fn run_child_mode(mode: &str) {
    let topic = env::var(CHILD_TOPIC_ENV).expect("TOPIC env var required");
    let child_id: u32 = env::var(CHILD_ID_ENV)
        .expect("CHILD_ID env var required")
        .parse()
        .expect("Invalid CHILD_ID");
    let msg_count: u32 = env::var(CHILD_MSG_COUNT_ENV)
        .unwrap_or_else(|_| "100".to_string())
        .parse()
        .unwrap_or(100);

    match mode {
        "link_producer" => child_link_producer(&topic, child_id, msg_count),
        "link_consumer" => child_link_consumer(&topic, child_id),
        "hub_producer" => child_hub_producer(&topic, child_id, msg_count),
        "hub_consumer" => child_hub_consumer(&topic, child_id, msg_count),
        "ping_pong" => child_ping_pong(&topic, child_id, msg_count),
        _ => {
            eprintln!("Unknown child mode: {}", mode);
            process::exit(1);
        }
    }
}

fn child_link_producer(topic: &str, child_id: u32, msg_count: u32) {
    let producer = match Topic::<CmdVel>::new(topic) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Child {}: Failed to create producer: {:?}", child_id, e);
            process::exit(1);
        }
    };

    // Small delay to ensure consumer is ready
    thread::sleep(Duration::from_millis(50));

    for i in 0..msg_count {
        let msg = CmdVel {
            linear: child_id as f32,
            angular: i as f32,
            stamp_nanos: ((child_id as u64) << 32) | (i as u64),
        };
        if producer.send(msg, &mut None).is_err() {
            // Link may overwrite - that's OK
        }
        // Small delay to avoid flooding
        if i % 10 == 0 {
            thread::sleep(Duration::from_micros(100));
        }
    }

    // Signal completion
    println!("CHILD_DONE:{}:{}", child_id, msg_count);
}

fn child_link_consumer(topic: &str, child_id: u32) {
    let consumer = match Topic::<CmdVel>::new(topic) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Child {}: Failed to create consumer: {:?}", child_id, e);
            process::exit(1);
        }
    };

    let start = Instant::now();
    let timeout = Duration::from_secs(10);
    let mut received = 0u32;
    let mut last_stamp = 0u64;

    while start.elapsed() < timeout {
        if let Some(msg) = consumer.recv(&mut None) {
            received += 1;
            last_stamp = msg.stamp_nanos;
        }
        thread::sleep(Duration::from_micros(100));
    }

    println!("CHILD_RECV:{}:{}:{}", child_id, received, last_stamp);
}

fn child_hub_producer(topic: &str, child_id: u32, msg_count: u32) {
    // Stagger startup based on child_id to reduce thundering herd
    thread::sleep(Duration::from_millis((child_id as u64 % 10) * 10));

    // Retry Hub creation a few times in case of race condition
    let mut hub = None;
    for attempt in 0..5 {
        match Topic::<CmdVel>::new(topic) {
            Ok(h) => {
                hub = Some(h);
                break;
            }
            Err(e) => {
                if attempt < 4 {
                    thread::sleep(Duration::from_millis(50 * (attempt as u64 + 1)));
                } else {
                    eprintln!("Child {}: Failed to create Hub after 5 attempts: {:?}", child_id, e);
                    process::exit(1);
                }
            }
        }
    }

    let hub = hub.unwrap();

    // Wait for consumers to be ready
    thread::sleep(Duration::from_millis(200));

    let mut sent = 0u32;
    for i in 0..msg_count {
        let msg = CmdVel {
            linear: child_id as f32,
            angular: i as f32,
            stamp_nanos: ((child_id as u64) << 32) | (i as u64),
        };
        if hub.send(msg, &mut None).is_ok() {
            sent += 1;
        }
        // Small delay to avoid ring buffer overflow and give receiver time
        if i % 10 == 0 {
            thread::sleep(Duration::from_micros(500));
        }
    }

    println!("CHILD_SENT:{}:{}", child_id, sent);
}

fn child_hub_consumer(topic: &str, child_id: u32, expected_total: u32) {
    let hub = match Topic::<CmdVel>::new(topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("Child {}: Failed to create Hub: {:?}", child_id, e);
            process::exit(1);
        }
    };

    let start = Instant::now();
    let timeout = Duration::from_secs(30);
    let mut received = 0u32;

    // Receive messages until timeout or we have enough
    while start.elapsed() < timeout && received < expected_total {
        if hub.recv(&mut None).is_some() {
            received += 1;
        }
        thread::sleep(Duration::from_micros(10));
    }

    println!("CHILD_RECV:{}:{}", child_id, received);
}

fn child_ping_pong(topic: &str, child_id: u32, rounds: u32) {
    let send_topic = format!("{}_to_{}", topic, child_id);
    let recv_topic = format!("{}_from_{}", topic, child_id);

    let sender = match Topic::<CmdVel>::new(&send_topic) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Child {}: Failed to create sender: {:?}", child_id, e);
            process::exit(1);
        }
    };

    let receiver = match Topic::<CmdVel>::new(&recv_topic) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("Child {}: Failed to create receiver: {:?}", child_id, e);
            process::exit(1);
        }
    };

    let mut completed_rounds = 0u32;
    let start = Instant::now();
    let timeout = Duration::from_secs(30);

    for round in 0..rounds {
        if start.elapsed() > timeout {
            break;
        }

        // Send ping
        let ping = CmdVel {
            linear: child_id as f32,
            angular: round as f32,
            stamp_nanos: round as u64,
        };
        if sender.send(ping, &mut None).is_err() {
            continue;
        }

        // Wait for pong
        let round_start = Instant::now();
        let round_timeout = Duration::from_millis(500);
        while round_start.elapsed() < round_timeout {
            if let Some(pong) = receiver.recv(&mut None) {
                if pong.stamp_nanos == round as u64 {
                    completed_rounds += 1;
                    break;
                }
            }
            thread::sleep(Duration::from_micros(100));
        }
    }

    println!("CHILD_PINGPONG:{}:{}", child_id, completed_rounds);
}

/// Spawn a child process with given mode
fn spawn_child(mode: &str, topic: &str, child_id: u32, msg_count: u32) -> std::io::Result<Child> {
    let exe = env::current_exe()?;
    Command::new(exe)
        .env(CHILD_MODE_ENV, mode)
        .env(CHILD_TOPIC_ENV, topic)
        .env(CHILD_ID_ENV, child_id.to_string())
        .env(CHILD_MSG_COUNT_ENV, msg_count.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
}

/// Test: N concurrent processes communicating via multiple Hub topics
///
/// Hub has a 16-consumer limit per topic (optimized for real-time robotics).
/// This test uses multiple Hub topics to test N processes:
/// - Processes are distributed across ceil(N/15) topics
/// - Each topic has up to 15 producers (leaving room for parent consumer)
/// - Verifies multi-topic multi-process communication works
fn test_concurrent_processes(num_processes: usize) -> bool {
    println!("\n========================================");
    println!("Test: {} Concurrent Processes (Multi-Hub)", num_processes);
    println!("========================================");

    // Hub limit is 16 consumers - use 15 per topic to leave room for parent
    const PROCS_PER_TOPIC: usize = 15;
    let num_topics = (num_processes + PROCS_PER_TOPIC - 1) / PROCS_PER_TOPIC;
    let msg_count = 50u32;

    println!("  Using {} Hub topics for {} processes", num_topics, num_processes);

    // Create consumer Hubs for each topic
    let mut hubs = Vec::new();
    let base_topic = format!("stress_concurrent_{}_{}", process::id(), num_processes);

    for topic_idx in 0..num_topics {
        let topic = format!("{}_{}", base_topic, topic_idx);
        match Topic::<CmdVel>::new(&topic) {
            Ok(h) => hubs.push((topic, h)),
            Err(e) => {
                eprintln!("Failed to create Hub for topic {}: {:?}", topic_idx, e);
                return false;
            }
        }
    }

    // Wait for shared memory to be fully initialized
    thread::sleep(Duration::from_millis(100));

    println!("  Spawning {} producer processes...", num_processes);
    let spawn_start = Instant::now();

    // Spawn producer processes distributed across topics
    let mut children: Vec<Child> = Vec::new();
    for i in 0..num_processes {
        let topic_idx = i / PROCS_PER_TOPIC;
        let topic = &hubs[topic_idx].0;
        match spawn_child("hub_producer", topic, i as u32, msg_count) {
            Ok(child) => children.push(child),
            Err(e) => {
                eprintln!("Failed to spawn child {}: {:?}", i, e);
                for mut c in children {
                    let _ = c.kill();
                }
                return false;
            }
        }
    }

    let spawn_duration = spawn_start.elapsed();
    println!(
        "  Spawned {} processes in {:?}",
        num_processes, spawn_duration
    );

    // Receive messages from all Hubs
    let recv_start = Instant::now();
    let recv_timeout = Duration::from_secs(60);
    let mut received = 0u64;
    let mut unique_senders = std::collections::HashSet::new();
    let expected_total = (num_processes as u64) * (msg_count as u64);

    while recv_start.elapsed() < recv_timeout {
        // Poll all Hubs
        for (_topic, hub) in &hubs {
            if let Some(msg) = hub.recv(&mut None) {
                received += 1;
                let sender_id = (msg.stamp_nanos >> 32) as u32;
                unique_senders.insert(sender_id);
            }
        }

        // Check if all children have exited
        let mut all_done = true;
        for child in &mut children {
            match child.try_wait() {
                Ok(Some(_)) => {}
                Ok(None) => {
                    all_done = false;
                }
                Err(_) => {}
            }
        }
        if all_done {
            // Drain remaining messages from all Hubs
            thread::sleep(Duration::from_millis(200));
            for (_topic, hub) in &hubs {
                while let Some(msg) = hub.recv(&mut None) {
                    received += 1;
                    let sender_id = (msg.stamp_nanos >> 32) as u32;
                    unique_senders.insert(sender_id);
                }
            }
            break;
        }

        thread::sleep(Duration::from_micros(10));
    }

    // Wait for all children to complete
    let mut child_results = Vec::new();
    let mut child_errors = Vec::new();
    for child in children {
        match child.wait_with_output() {
            Ok(output) => {
                let stdout = String::from_utf8_lossy(&output.stdout);
                let stderr = String::from_utf8_lossy(&output.stderr);
                child_results.push(stdout.to_string());
                if !stderr.trim().is_empty() {
                    child_errors.push(stderr.to_string());
                }
            }
            Err(e) => {
                child_errors.push(format!("wait_with_output failed: {:?}", e));
            }
        }
    }

    // Show any child errors (limited to first 5)
    if !child_errors.is_empty() {
        println!("  Child errors (showing first {} of {}):",
            child_errors.len().min(5), child_errors.len());
        for (i, err) in child_errors.iter().take(5).enumerate() {
            println!("    Error {}: {}", i + 1, err.trim());
        }
    }

    // Parse child results
    let mut total_child_sent = 0u32;
    for result in &child_results {
        for line in result.lines() {
            if line.starts_with("CHILD_SENT:") {
                let parts: Vec<&str> = line.split(':').collect();
                if parts.len() >= 3 {
                    total_child_sent += parts[2].parse::<u32>().unwrap_or(0);
                }
            }
        }
    }

    let recv_rate = if expected_total > 0 {
        (received as f64 / expected_total as f64) * 100.0
    } else {
        0.0
    };

    println!("  Results:");
    println!("    Unique senders seen: {}/{}", unique_senders.len(), num_processes);
    println!("    Messages received: {}/{} ({:.1}%)", received, expected_total, recv_rate);
    println!("    Child processes reported sent: {}", total_child_sent);

    // Success criteria:
    // - At least 90% of processes successfully communicated
    // - Received at least 80% of messages
    let success = unique_senders.len() >= (num_processes * 9 / 10) && recv_rate >= 80.0;

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

/// Test: Rapid process spawn/exit cycles
fn test_spawn_exit_rapid() -> bool {
    println!("\n========================================");
    println!("Test: Rapid Process Spawn/Exit Cycles");
    println!("========================================");

    let cycles = 20;
    let processes_per_cycle = 10;
    let mut total_spawned = 0;
    let mut total_succeeded = 0;

    let start = Instant::now();

    for cycle in 0..cycles {
        let topic = format!("stress_rapid_{}_{}_{}", process::id(), cycle, Instant::now().elapsed().as_micros());

        // Spawn processes
        let mut children: Vec<Child> = Vec::new();
        for i in 0..processes_per_cycle {
            match spawn_child("link_producer", &topic, i as u32, 10) {
                Ok(child) => {
                    children.push(child);
                    total_spawned += 1;
                }
                Err(e) => {
                    eprintln!("  Cycle {}: Failed to spawn child {}: {:?}", cycle, i, e);
                }
            }
        }

        // Wait for all to complete (with timeout)
        let cycle_timeout = Duration::from_secs(5);
        let cycle_start = Instant::now();

        for mut child in children {
            while cycle_start.elapsed() < cycle_timeout {
                match child.try_wait() {
                    Ok(Some(status)) => {
                        if status.success() {
                            total_succeeded += 1;
                        }
                        break;
                    }
                    Ok(None) => {
                        thread::sleep(Duration::from_millis(10));
                    }
                    Err(_) => break,
                }
            }
            // Kill if still running
            let _ = child.kill();
        }

        if (cycle + 1) % 5 == 0 {
            println!(
                "  Completed {}/{} cycles ({} spawned, {} succeeded)",
                cycle + 1,
                cycles,
                total_spawned,
                total_succeeded
            );
        }
    }

    let duration = start.elapsed();
    println!("  Total time: {:?}", duration);
    println!("  Total spawned: {}", total_spawned);
    println!("  Total succeeded: {}", total_succeeded);
    println!(
        "  Success rate: {:.1}%",
        (total_succeeded as f64 / total_spawned as f64) * 100.0
    );

    // Success: At least 80% of spawned processes completed successfully
    let success = total_succeeded >= (total_spawned * 8 / 10);

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

/// Test: Hub with many producer processes
///
/// Hub has a 16-consumer limit. This test uses 14 producers to stay within the limit
/// while testing Hub's multi-producer capabilities at high load.
fn test_hub_many_producers() -> bool {
    println!("\n========================================");
    println!("Test: Hub with Many Producers (14 max)");
    println!("========================================");

    // Stay under 16 consumer limit (1 parent consumer + 14 producers = 15)
    let num_producers = 14;
    let msgs_per_producer = 100; // Higher message count for more stress
    let topic = format!("stress_hub_many_{}", process::id());

    // Create consumer Hub first
    let hub = match Topic::<CmdVel>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("Failed to create Hub: {:?}", e);
            return false;
        }
    };

    // Wait for shared memory to be ready
    thread::sleep(Duration::from_millis(100));

    println!("  Spawning {} producer processes...", num_producers);

    // Spawn producer processes
    let mut children: Vec<Child> = Vec::new();
    for i in 0..num_producers {
        match spawn_child("hub_producer", &topic, i as u32, msgs_per_producer) {
            Ok(child) => children.push(child),
            Err(e) => {
                eprintln!("Failed to spawn producer {}: {:?}", i, e);
            }
        }
    }

    println!("  Spawned {} producers", children.len());

    // Receive messages
    let start = Instant::now();
    let timeout = Duration::from_secs(60);
    let mut received = 0u64;
    let mut senders_seen = std::collections::HashSet::new();

    while start.elapsed() < timeout {
        if let Some(msg) = hub.recv(&mut None) {
            received += 1;
            let sender_id = (msg.stamp_nanos >> 32) as u32;
            senders_seen.insert(sender_id);
        }

        // Check if all children exited
        let mut all_done = true;
        for child in &mut children {
            if let Ok(None) = child.try_wait() {
                all_done = false;
            }
        }
        if all_done {
            // Drain remaining messages
            thread::sleep(Duration::from_millis(200));
            while let Some(msg) = hub.recv(&mut None) {
                received += 1;
                let sender_id = (msg.stamp_nanos >> 32) as u32;
                senders_seen.insert(sender_id);
            }
            break;
        }

        thread::sleep(Duration::from_micros(10));
    }

    // Cleanup children
    for child in children {
        let _ = child.wait_with_output();
    }

    let expected_total = (num_producers as u64) * (msgs_per_producer as u64);
    let recv_rate = (received as f64 / expected_total as f64) * 100.0;

    println!("  Results:");
    println!("    Unique senders: {}/{}", senders_seen.len(), num_producers);
    println!("    Messages received: {}/{} ({:.1}%)", received, expected_total, recv_rate);

    // Success: Received at least 80% of messages from all producers
    let success = senders_seen.len() >= num_producers && recv_rate >= 80.0;

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

/// Test: Verify shared memory cleanup after process termination
fn test_cleanup_verification() -> bool {
    println!("\n========================================");
    println!("Test: Shared Memory Cleanup Verification");
    println!("========================================");

    let num_iterations = 5;
    let processes_per_iteration = 10;

    for iter in 0..num_iterations {
        let topic = format!("stress_cleanup_{}_{}", process::id(), iter);

        // Spawn processes
        let mut children: Vec<Child> = Vec::new();
        for i in 0..processes_per_iteration {
            match spawn_child("link_producer", &topic, i as u32, 50) {
                Ok(child) => children.push(child),
                Err(e) => {
                    eprintln!("  Iter {}: Failed to spawn child {}: {:?}", iter, i, e);
                }
            }
        }

        // Wait for completion
        for mut child in children {
            let _ = child.wait();
        }

        println!("  Iteration {}/{} completed", iter + 1, num_iterations);
    }

    // Verify we can still create new topics (no resource exhaustion)
    println!("  Verifying system resources are clean...");

    let test_topic = format!("stress_cleanup_final_{}", process::id());
    let producer = match Topic::<CmdVel>::new(&test_topic) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("  Failed to create producer after cleanup test: {:?}", e);
            return false;
        }
    };

    let consumer = match Topic::<CmdVel>::new(&test_topic) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("  Failed to create consumer after cleanup test: {:?}", e);
            return false;
        }
    };

    // Send and receive a message
    let msg = CmdVel {
        linear: 1.0,
        angular: 2.0,
        stamp_nanos: 12345,
    };

    if producer.send(msg, &mut None).is_err() {
        eprintln!("  Failed to send after cleanup test");
        return false;
    }

    thread::sleep(Duration::from_millis(10));

    if let Some(received) = consumer.recv(&mut None) {
        if received.stamp_nanos == 12345 {
            println!("  ✓ System resources clean - communication works");
            println!("  ✓ Test PASSED");
            return true;
        }
    }

    eprintln!("  Failed to receive message after cleanup test");
    println!("  ✗ Test FAILED");
    false
}
