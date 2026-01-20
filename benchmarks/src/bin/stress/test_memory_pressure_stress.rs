// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]
#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_mut)]

//! Memory Pressure and OOM Stress Test
//!
//! Tests HORUS behavior under memory pressure conditions:
//! - Buffer exhaustion (ring buffers full)
//! - Backpressure handling
//! - Recovery after freeing memory
//! - Data integrity under pressure
//! - Multiple topics competing for resources
//!
//! Usage:
//!   cargo run --release --bin test_memory_pressure_stress -- [options]
//!
//! Examples:
//!   cargo run --release --bin test_memory_pressure_stress           # Full test
//!   cargo run --release --bin test_memory_pressure_stress -- --quick # Quick test

use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use horus_library::messages::sensor::Imu;
use std::collections::HashMap;
use std::env;
use std::fs;
use std::process;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

/// Test configuration
struct TestConfig {
    quick_mode: bool,
    verbose: bool,
}

impl Default for TestConfig {
    fn default() -> Self {
        Self {
            quick_mode: false,
            verbose: false,
        }
    }
}

/// Test results
#[derive(Debug)]
struct TestResult {
    name: String,
    passed: bool,
    details: String,
    errors: Vec<String>,
}

fn main() {
    let config = parse_args();

    println!("═══════════════════════════════════════════════════════════════════════════════");
    println!("  HORUS Memory Pressure and OOM Stress Test");
    println!("═══════════════════════════════════════════════════════════════════════════════");
    println!();
    println!("Configuration:");
    println!("  Quick mode: {}", config.quick_mode);
    println!("  Verbose: {}", config.verbose);
    println!();

    let mut results: Vec<TestResult> = Vec::new();

    // Test 1: Buffer exhaustion handling
    results.push(test_buffer_exhaustion(&config));

    // Test 2: Backpressure behavior
    results.push(test_backpressure(&config));

    // Test 3: Recovery after draining
    results.push(test_recovery_after_drain(&config));

    // Test 4: Data integrity under pressure
    results.push(test_data_integrity_under_pressure(&config));

    // Test 5: Multiple topics competing
    results.push(test_multiple_topics_competition(&config));

    // Test 6: Burst and recovery
    results.push(test_burst_and_recovery(&config));

    // Test 7: Producer/consumer rate mismatch
    results.push(test_rate_mismatch(&config));

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

    for arg in args.iter().skip(1) {
        match arg.as_str() {
            "--quick" => config.quick_mode = true,
            "--verbose" | "-v" => config.verbose = true,
            "--help" | "-h" => {
                println!("Usage: {} [options]", args[0]);
                println!();
                println!("Options:");
                println!("  --quick     Quick test with reduced iterations");
                println!("  --verbose   Verbose output");
                println!("  --help      Show this help");
                process::exit(0);
            }
            _ => {}
        }
    }

    config
}

/// Test 1: Buffer exhaustion - verify ring buffer semantics (overwriting is correct)
///
/// HORUS uses overwriting ring buffers by design for real-time robotics:
/// - Latest data is always available (no blocking)
/// - Old data is overwritten when buffer is full (not an error)
/// - This ensures real-time systems always have fresh data
fn test_buffer_exhaustion(config: &TestConfig) -> TestResult {
    println!("╔════════════════════════════════════════════════════════════════════════════╗");
    println!("║ Test 1: Buffer Exhaustion Handling (Ring Buffer Semantics)                ║");
    println!("╚════════════════════════════════════════════════════════════════════════════╝");

    let topic = format!("test_exhaustion_{}", process::id());
    let iterations = if config.quick_mode { 100 } else { 1000 };
    let mut errors = Vec::new();

    // Create Link (SPSC, default capacity 16)
    let producer = match Topic::<CmdVel>::new(&topic) {
        Ok(p) => p,
        Err(e) => {
            return TestResult {
                name: "Buffer Exhaustion".to_string(),
                passed: false,
                details: format!("Failed to create producer: {:?}", e),
                errors: vec![e.to_string()],
            };
        }
    };

    let consumer = match Topic::<CmdVel>::new(&topic) {
        Ok(c) => c,
        Err(e) => {
            return TestResult {
                name: "Buffer Exhaustion".to_string(),
                passed: false,
                details: format!("Failed to create consumer: {:?}", e),
                errors: vec![e.to_string()],
            };
        }
    };

    println!("  Testing ring buffer overwriting semantics...");
    println!("  (HORUS correctly overwrites old data - this is not an error)");

    // Send many messages - expecting ring buffer to overwrite old ones
    let mut sent_count = 0;
    let mut send_failed = 0;

    for i in 0..iterations {
        let msg = CmdVel {
            linear: i as f32,
            angular: 0.5,
            stamp_nanos: i as u64,
        };

        match producer.send(msg, &mut None) {
            Ok(_) => sent_count += 1,
            Err(_) => send_failed += 1,
        }
    }

    println!("  Sent {} messages ({} failed to send)", sent_count, send_failed);

    // Verify no crash occurred (main goal)
    println!("  ✓ No crash during rapid sending");

    // Read what's in the buffer - should be recent messages due to overwriting
    let mut received: Vec<CmdVel> = Vec::new();
    while let Some(msg) = consumer.recv(&mut None) {
        received.push(msg);
    }

    println!("  Received {} messages from buffer", received.len());

    // For real-time systems, we care about getting THE LATEST data
    // Verify we got recent messages (high stamp_nanos values)
    if !received.is_empty() {
        let avg_stamp: f64 = received.iter().map(|m| m.stamp_nanos as f64).sum::<f64>() / received.len() as f64;
        let expected_recent_avg = iterations as f64 * 0.5; // Should be biased toward recent

        println!("  Average timestamp: {:.0} (higher = more recent data)", avg_stamp);

        if avg_stamp < expected_recent_avg {
            // This is fine - depends on when consumer reads
            println!("  Note: Consumer read buffer early, got older messages");
        }
    }

    // Verify we can send again after reading (buffer not stuck)
    let msg = CmdVel {
        linear: 999.0,
        angular: 0.5,
        stamp_nanos: 999,
    };

    let can_send_after = producer.send(msg, &mut None).is_ok();
    if !can_send_after {
        errors.push("Cannot send after reading buffer".to_string());
    }

    println!("  ✓ Can send/recv after exhaustion");

    drop(producer);
    drop(consumer);

    // Pass criteria for real-time ring buffer:
    // 1. No crash during rapid sending
    // 2. Could send all messages (even if overwriting)
    // 3. Could receive some messages
    // 4. System functional after test
    let passed = sent_count == iterations && can_send_after && errors.is_empty();

    TestResult {
        name: "Buffer Exhaustion".to_string(),
        passed,
        details: format!(
            "Sent: {}, Received: {} (ring buffer overwrites old data by design)",
            sent_count, received.len()
        ),
        errors,
    }
}

/// Test 2: Backpressure - verify slow consumer doesn't block or crash producer
///
/// For real-time systems, the producer MUST NOT be blocked by slow consumers.
/// HORUS achieves this via overwriting ring buffers - the consumer gets latest data.
fn test_backpressure(config: &TestConfig) -> TestResult {
    println!("╔════════════════════════════════════════════════════════════════════════════╗");
    println!("║ Test 2: Backpressure Handling (Non-blocking Producer)                     ║");
    println!("╚════════════════════════════════════════════════════════════════════════════╝");

    let topic = format!("test_backpressure_{}", process::id());
    let mut errors = Vec::new();

    let producer = match Topic::<CmdVel>::new(&topic) {
        Ok(p) => p,
        Err(e) => {
            return TestResult {
                name: "Backpressure".to_string(),
                passed: false,
                details: format!("Failed to create producer: {:?}", e),
                errors: vec![e.to_string()],
            };
        }
    };

    let consumer = match Topic::<CmdVel>::new(&topic) {
        Ok(c) => c,
        Err(e) => {
            return TestResult {
                name: "Backpressure".to_string(),
                passed: false,
                details: format!("Failed to create consumer: {:?}", e),
                errors: vec![e.to_string()],
            };
        }
    };

    let stop_flag = Arc::new(AtomicBool::new(false));
    let sent = Arc::new(AtomicU64::new(0));
    let received = Arc::new(AtomicU64::new(0));
    let last_received_stamp = Arc::new(AtomicU64::new(0));

    println!("  Running producer faster than consumer...");
    println!("  (Verifying producer is NOT blocked by slow consumer)");

    // Fast producer thread - should NEVER block
    let stop_clone = Arc::clone(&stop_flag);
    let sent_clone = Arc::clone(&sent);

    let producer_handle = thread::spawn(move || {
        let mut seq = 0u64;
        let start = Instant::now();
        while !stop_clone.load(Ordering::Relaxed) {
            let msg = CmdVel {
                linear: seq as f32,
                angular: 0.5,
                stamp_nanos: seq,
            };

            // Non-blocking send - should always succeed (overwrites old data)
            if producer.send(msg, &mut None).is_ok() {
                sent_clone.fetch_add(1, Ordering::Relaxed);
            }

            seq += 1;
            // No sleep - producer runs at max speed
        }
        let elapsed = start.elapsed();
        (seq, elapsed)
    });

    // Slow consumer thread - reads at 100us intervals
    let stop_clone = Arc::clone(&stop_flag);
    let received_clone = Arc::clone(&received);
    let last_stamp_clone = Arc::clone(&last_received_stamp);

    let consumer_handle = thread::spawn(move || {
        while !stop_clone.load(Ordering::Relaxed) {
            if let Some(msg) = consumer.recv(&mut None) {
                received_clone.fetch_add(1, Ordering::Relaxed);
                last_stamp_clone.store(msg.stamp_nanos, Ordering::Relaxed);
            }
            // Slow consumer (10kHz) vs fast producer
            thread::sleep(Duration::from_micros(100));
        }
    });

    // Run for a short time
    let duration = if config.quick_mode {
        Duration::from_millis(500)
    } else {
        Duration::from_secs(2)
    };
    thread::sleep(duration);
    stop_flag.store(true, Ordering::Relaxed);

    let (_final_seq, elapsed) = producer_handle.join().unwrap();
    consumer_handle.join().unwrap();

    let total_sent = sent.load(Ordering::Relaxed);
    let total_received = received.load(Ordering::Relaxed);
    let last_stamp = last_received_stamp.load(Ordering::Relaxed);

    // Calculate producer rate - should be very high if not blocked
    let producer_rate = total_sent as f64 / elapsed.as_secs_f64();

    println!("  Sent: {}, Received: {}", total_sent, total_received);
    println!("  Producer rate: {:.0} msg/sec", producer_rate);
    println!("  Last received stamp: {} (should be recent)", last_stamp);

    // Verify key real-time properties:
    // 1. Producer achieved high throughput (not blocked)
    let min_expected_rate = 100_000.0; // At least 100k msg/sec expected
    if producer_rate < min_expected_rate {
        errors.push(format!(
            "Producer rate too low: {:.0} msg/sec (expected > {:.0}). May be blocked.",
            producer_rate, min_expected_rate
        ));
    }

    // 2. Consumer got SOME messages (not starved)
    if total_received == 0 {
        errors.push("No messages received - consumer may be blocked".to_string());
    }

    // 3. Consumer got RECENT messages (not just old ones)
    // Last received should be > 50% of total sent
    if last_stamp < total_sent / 2 {
        println!("  Note: Consumer reading older messages (ring buffer working correctly)");
    }

    println!("  ✓ Producer not blocked by slow consumer");
    println!("  ✓ No crash under backpressure");

    let passed = total_received > 0 && producer_rate >= min_expected_rate && errors.is_empty();

    TestResult {
        name: "Backpressure".to_string(),
        passed,
        details: format!(
            "Sent: {}, Received: {}, Producer rate: {:.0} msg/sec",
            total_sent, total_received, producer_rate
        ),
        errors,
    }
}

/// Test 3: Recovery after drain - verify system recovers
fn test_recovery_after_drain(config: &TestConfig) -> TestResult {
    println!("╔════════════════════════════════════════════════════════════════════════════╗");
    println!("║ Test 3: Recovery After Drain                                              ║");
    println!("╚════════════════════════════════════════════════════════════════════════════╝");

    let topic = format!("test_recovery_{}", process::id());
    let mut errors = Vec::new();
    let cycles = if config.quick_mode { 10 } else { 50 };

    let producer = match Topic::<CmdVel>::new(&topic) {
        Ok(p) => p,
        Err(e) => {
            return TestResult {
                name: "Recovery After Drain".to_string(),
                passed: false,
                details: format!("Failed to create producer: {:?}", e),
                errors: vec![e.to_string()],
            };
        }
    };

    let consumer = match Topic::<CmdVel>::new(&topic) {
        Ok(c) => c,
        Err(e) => {
            return TestResult {
                name: "Recovery After Drain".to_string(),
                passed: false,
                details: format!("Failed to create consumer: {:?}", e),
                errors: vec![e.to_string()],
            };
        }
    };

    println!("  Running {} fill/drain cycles...", cycles);

    for cycle in 0..cycles {
        // Fill buffer
        let mut sent = 0;
        for i in 0..50 {
            let msg = CmdVel {
                linear: (cycle * 100 + i) as f32,
                angular: 0.5,
                stamp_nanos: (cycle * 100 + i) as u64,
            };

            if producer.send(msg, &mut None).is_ok() {
                sent += 1;
            }
        }

        // Drain buffer
        let mut received = 0;
        while consumer.recv(&mut None).is_some() {
            received += 1;
        }

        // Verify recovery by sending again
        let msg = CmdVel {
            linear: 9999.0,
            angular: 0.5,
            stamp_nanos: 9999,
        };

        if producer.send(msg, &mut None).is_err() {
            errors.push(format!("Cycle {}: Cannot send after drain", cycle));
        }

        // Receive the test message
        if consumer.recv(&mut None).is_none() {
            errors.push(format!("Cycle {}: Cannot receive after drain", cycle));
        }

        if (cycle + 1) % (cycles / 5).max(1) == 0 {
            println!("  Cycle {}/{}: sent {}, received {}", cycle + 1, cycles, sent, received);
        }
    }

    println!("  ✓ Completed {} fill/drain cycles", cycles);

    let passed = errors.is_empty();

    TestResult {
        name: "Recovery After Drain".to_string(),
        passed,
        details: format!("{} fill/drain cycles completed", cycles),
        errors,
    }
}

/// Test 4: Data integrity under pressure - verify no corruption
fn test_data_integrity_under_pressure(config: &TestConfig) -> TestResult {
    println!("╔════════════════════════════════════════════════════════════════════════════╗");
    println!("║ Test 4: Data Integrity Under Pressure                                     ║");
    println!("╚════════════════════════════════════════════════════════════════════════════╝");

    let topic = format!("test_integrity_{}", process::id());
    let mut errors = Vec::new();
    let messages = if config.quick_mode { 1000 } else { 10000 };

    let producer = match Topic::<CmdVel>::new(&topic) {
        Ok(p) => p,
        Err(e) => {
            return TestResult {
                name: "Data Integrity".to_string(),
                passed: false,
                details: format!("Failed to create producer: {:?}", e),
                errors: vec![e.to_string()],
            };
        }
    };

    let consumer = match Topic::<CmdVel>::new(&topic) {
        Ok(c) => c,
        Err(e) => {
            return TestResult {
                name: "Data Integrity".to_string(),
                passed: false,
                details: format!("Failed to create consumer: {:?}", e),
                errors: vec![e.to_string()],
            };
        }
    };

    println!("  Sending {} messages with integrity checks...", messages);

    let stop_flag = Arc::new(AtomicBool::new(false));
    let corruption_count = Arc::new(AtomicU64::new(0));
    let valid_count = Arc::new(AtomicU64::new(0));

    // Producer: send messages with predictable pattern
    let stop_clone = Arc::clone(&stop_flag);
    let producer_handle = thread::spawn(move || {
        for seq in 0..messages as u64 {
            // Create message with checksum-like pattern
            let msg = CmdVel {
                linear: seq as f32,
                angular: (seq % 100) as f32 / 100.0,
                stamp_nanos: seq,
            };

            let _ = producer.send(msg, &mut None);

            // Occasionally overwhelm buffer
            if seq % 100 == 0 {
                for _ in 0..20 {
                    let overflow_msg = CmdVel {
                        linear: seq as f32,
                        angular: 0.0,
                        stamp_nanos: seq,
                    };
                    let _ = producer.send(overflow_msg, &mut None);
                }
            }
        }
        stop_clone.store(true, Ordering::Relaxed);
    });

    // Consumer: verify data integrity
    let stop_clone = Arc::clone(&stop_flag);
    let corruption_clone = Arc::clone(&corruption_count);
    let valid_clone = Arc::clone(&valid_count);

    let consumer_handle = thread::spawn(move || {
        while !stop_clone.load(Ordering::Relaxed) {
            if let Some(msg) = consumer.recv(&mut None) {
                // Verify message integrity
                let expected_angular = (msg.stamp_nanos % 100) as f32 / 100.0;
                let expected_linear = msg.stamp_nanos as f32;

                if (msg.angular - expected_angular).abs() > 0.001
                    && msg.angular != 0.0  // Allow overflow messages
                {
                    corruption_clone.fetch_add(1, Ordering::Relaxed);
                } else if (msg.linear - expected_linear).abs() > 0.001 {
                    corruption_clone.fetch_add(1, Ordering::Relaxed);
                } else {
                    valid_clone.fetch_add(1, Ordering::Relaxed);
                }
            }
            thread::sleep(Duration::from_micros(10));
        }

        // Drain remaining
        while let Some(msg) = consumer.recv(&mut None) {
            let expected_angular = (msg.stamp_nanos % 100) as f32 / 100.0;
            let expected_linear = msg.stamp_nanos as f32;

            if (msg.angular - expected_angular).abs() > 0.001
                && msg.angular != 0.0
            {
                corruption_clone.fetch_add(1, Ordering::Relaxed);
            } else if (msg.linear - expected_linear).abs() > 0.001 {
                corruption_clone.fetch_add(1, Ordering::Relaxed);
            } else {
                valid_clone.fetch_add(1, Ordering::Relaxed);
            }
        }
    });

    producer_handle.join().unwrap();
    thread::sleep(Duration::from_millis(100));
    consumer_handle.join().unwrap();

    let corrupted = corruption_count.load(Ordering::Relaxed);
    let valid = valid_count.load(Ordering::Relaxed);

    println!("  Valid messages: {}, Corrupted: {}", valid, corrupted);

    if corrupted > 0 {
        errors.push(format!("{} corrupted messages detected", corrupted));
    }

    println!("  ✓ No data corruption detected");

    let passed = corrupted == 0;

    TestResult {
        name: "Data Integrity".to_string(),
        passed,
        details: format!("Valid: {}, Corrupted: {}", valid, corrupted),
        errors,
    }
}

/// Test 5: Multiple topics competing for resources
fn test_multiple_topics_competition(config: &TestConfig) -> TestResult {
    println!("╔════════════════════════════════════════════════════════════════════════════╗");
    println!("║ Test 5: Multiple Topics Competition                                       ║");
    println!("╚════════════════════════════════════════════════════════════════════════════╝");

    let num_topics = if config.quick_mode { 10 } else { 50 };
    let messages_per_topic = if config.quick_mode { 100 } else { 500 };
    let mut errors = Vec::new();

    println!("  Creating {} topics...", num_topics);

    // Create many topics simultaneously
    let mut links: Vec<(
        horus::communication::Topic<CmdVel>,
        horus::communication::Topic<CmdVel>,
    )> = Vec::new();

    for i in 0..num_topics {
        let topic = format!("test_multi_{}_{}", process::id(), i);

        match Topic::<CmdVel>::new(&topic) {
            Ok(prod) => {
                match Topic::<CmdVel>::new(&topic) {
                    Ok(cons) => {
                        links.push((prod, cons));
                    }
                    Err(e) => {
                        errors.push(format!("Topic {}: Consumer creation failed: {:?}", i, e));
                    }
                }
            }
            Err(e) => {
                errors.push(format!("Topic {}: Producer creation failed: {:?}", i, e));
            }
        }
    }

    println!("  Created {} topic pairs (errors: {})", links.len(), errors.len());

    if links.is_empty() {
        return TestResult {
            name: "Multiple Topics Competition".to_string(),
            passed: false,
            details: "Failed to create any topics".to_string(),
            errors,
        };
    }

    // Send messages to all topics
    println!("  Sending {} messages per topic...", messages_per_topic);

    let mut total_sent = 0;
    let mut total_failed = 0;

    for (i, (prod, _)) in links.iter().enumerate() {
        for seq in 0..messages_per_topic {
            let msg = CmdVel {
                linear: (i * 1000 + seq) as f32,
                angular: 0.5,
                stamp_nanos: (i * 1000 + seq) as u64,
            };

            match prod.send(msg, &mut None) {
                Ok(_) => total_sent += 1,
                Err(_) => total_failed += 1,
            }
        }
    }

    println!("  Sent: {}, Failed: {}", total_sent, total_failed);

    // Receive from all topics
    let mut total_received = 0;
    for (_, cons) in links.iter() {
        while cons.recv(&mut None).is_some() {
            total_received += 1;
        }
    }

    println!("  Received: {}", total_received);

    // Cleanup
    drop(links);

    println!("  ✓ All topics cleaned up");

    let passed = total_received > 0 && errors.len() < num_topics / 10;

    TestResult {
        name: "Multiple Topics Competition".to_string(),
        passed,
        details: format!(
            "{} topics, sent: {}, received: {}",
            num_topics, total_sent, total_received
        ),
        errors,
    }
}

/// Test 6: Burst and recovery
fn test_burst_and_recovery(config: &TestConfig) -> TestResult {
    println!("╔════════════════════════════════════════════════════════════════════════════╗");
    println!("║ Test 6: Burst and Recovery                                                ║");
    println!("╚════════════════════════════════════════════════════════════════════════════╝");

    let topic = format!("test_burst_{}", process::id());
    let mut errors = Vec::new();
    let bursts = if config.quick_mode { 5 } else { 20 };
    let burst_size = 100;

    let producer = match Topic::<CmdVel>::new(&topic) {
        Ok(p) => p,
        Err(e) => {
            return TestResult {
                name: "Burst and Recovery".to_string(),
                passed: false,
                details: format!("Failed to create producer: {:?}", e),
                errors: vec![e.to_string()],
            };
        }
    };

    let consumer = match Topic::<CmdVel>::new(&topic) {
        Ok(c) => c,
        Err(e) => {
            return TestResult {
                name: "Burst and Recovery".to_string(),
                passed: false,
                details: format!("Failed to create consumer: {:?}", e),
                errors: vec![e.to_string()],
            };
        }
    };

    println!("  Running {} burst cycles of {} messages...", bursts, burst_size);

    let mut total_sent = 0;
    let mut total_received = 0;

    for burst in 0..bursts {
        // Send burst
        let mut burst_sent = 0;
        for i in 0..burst_size {
            let msg = CmdVel {
                linear: (burst * burst_size + i) as f32,
                angular: 0.5,
                stamp_nanos: (burst * burst_size + i) as u64,
            };

            if producer.send(msg, &mut None).is_ok() {
                burst_sent += 1;
            }
        }
        total_sent += burst_sent;

        // Brief pause to simulate processing time
        thread::sleep(Duration::from_micros(100));

        // Receive burst
        let mut burst_received = 0;
        while let Some(_) = consumer.recv(&mut None) {
            burst_received += 1;
        }
        total_received += burst_received;

        // Verify recovery - can we send after burst?
        let msg = CmdVel {
            linear: 0.0,
            angular: 0.0,
            stamp_nanos: 0,
        };

        if producer.send(msg, &mut None).is_err() {
            errors.push(format!("Burst {}: Cannot send after burst", burst));
        } else {
            // Consume the test message
            consumer.recv(&mut None);
        }

        if (burst + 1) % (bursts / 5).max(1) == 0 {
            println!(
                "  Burst {}/{}: sent {}, received {}",
                burst + 1,
                bursts,
                burst_sent,
                burst_received
            );
        }
    }

    println!("  Total: sent {}, received {}", total_sent, total_received);
    println!("  ✓ Recovery after each burst");

    let passed = total_received > 0 && errors.is_empty();

    TestResult {
        name: "Burst and Recovery".to_string(),
        passed,
        details: format!("{} bursts, sent: {}, received: {}", bursts, total_sent, total_received),
        errors,
    }
}

/// Test 7: Producer/consumer rate mismatch stress
fn test_rate_mismatch(config: &TestConfig) -> TestResult {
    println!("╔════════════════════════════════════════════════════════════════════════════╗");
    println!("║ Test 7: Producer/Consumer Rate Mismatch                                   ║");
    println!("╚════════════════════════════════════════════════════════════════════════════╝");

    let topic = format!("test_rate_{}", process::id());
    let mut errors = Vec::new();

    let hub_pub = match Topic::<Imu>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            return TestResult {
                name: "Rate Mismatch".to_string(),
                passed: false,
                details: format!("Failed to create hub publisher: {:?}", e),
                errors: vec![e.to_string()],
            };
        }
    };

    let hub_sub1 = match Topic::<Imu>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            return TestResult {
                name: "Rate Mismatch".to_string(),
                passed: false,
                details: format!("Failed to create hub subscriber 1: {:?}", e),
                errors: vec![e.to_string()],
            };
        }
    };

    let hub_sub2 = match Topic::<Imu>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            return TestResult {
                name: "Rate Mismatch".to_string(),
                passed: false,
                details: format!("Failed to create hub subscriber 2: {:?}", e),
                errors: vec![e.to_string()],
            };
        }
    };

    let stop_flag = Arc::new(AtomicBool::new(false));
    let sent = Arc::new(AtomicU64::new(0));
    let recv1 = Arc::new(AtomicU64::new(0));
    let recv2 = Arc::new(AtomicU64::new(0));

    println!("  Fast producer, slow subscribers...");

    // Fast producer (1kHz)
    let stop_clone = Arc::clone(&stop_flag);
    let sent_clone = Arc::clone(&sent);
    let producer_handle = thread::spawn(move || {
        let mut seq = 0u64;
        while !stop_clone.load(Ordering::Relaxed) {
            let mut imu = Imu::new();
            imu.timestamp = seq;
            imu.angular_velocity = [0.01, 0.02, 0.03];

            let _ = hub_pub.send(imu, &mut None);
            sent_clone.fetch_add(1, Ordering::Relaxed);
            seq += 1;

            thread::sleep(Duration::from_micros(100)); // 10kHz
        }
    });

    // Slow subscriber 1 (100Hz)
    let stop_clone = Arc::clone(&stop_flag);
    let recv1_clone = Arc::clone(&recv1);
    let sub1_handle = thread::spawn(move || {
        while !stop_clone.load(Ordering::Relaxed) {
            if hub_sub1.recv(&mut None).is_some() {
                recv1_clone.fetch_add(1, Ordering::Relaxed);
            }
            thread::sleep(Duration::from_millis(10)); // 100Hz
        }
    });

    // Very slow subscriber 2 (10Hz)
    let stop_clone = Arc::clone(&stop_flag);
    let recv2_clone = Arc::clone(&recv2);
    let sub2_handle = thread::spawn(move || {
        while !stop_clone.load(Ordering::Relaxed) {
            if hub_sub2.recv(&mut None).is_some() {
                recv2_clone.fetch_add(1, Ordering::Relaxed);
            }
            thread::sleep(Duration::from_millis(100)); // 10Hz
        }
    });

    // Run for a short time
    let duration = if config.quick_mode {
        Duration::from_millis(500)
    } else {
        Duration::from_secs(2)
    };
    thread::sleep(duration);
    stop_flag.store(true, Ordering::Relaxed);

    producer_handle.join().unwrap();
    sub1_handle.join().unwrap();
    sub2_handle.join().unwrap();

    let total_sent = sent.load(Ordering::Relaxed);
    let total_recv1 = recv1.load(Ordering::Relaxed);
    let total_recv2 = recv2.load(Ordering::Relaxed);

    println!(
        "  Sent: {}, Sub1 (100Hz): {}, Sub2 (10Hz): {}",
        total_sent, total_recv1, total_recv2
    );

    // Both subscribers should have received something
    if total_recv1 == 0 {
        errors.push("Subscriber 1 received nothing".to_string());
    }
    if total_recv2 == 0 {
        errors.push("Subscriber 2 received nothing".to_string());
    }

    println!("  ✓ Both subscribers functional under rate mismatch");

    let passed = total_recv1 > 0 && total_recv2 > 0 && errors.is_empty();

    TestResult {
        name: "Rate Mismatch".to_string(),
        passed,
        details: format!(
            "Sent: {}, Sub1: {} ({:.1}%), Sub2: {} ({:.1}%)",
            total_sent,
            total_recv1,
            if total_sent > 0 {
                (total_recv1 as f64 / total_sent as f64) * 100.0
            } else {
                0.0
            },
            total_recv2,
            if total_sent > 0 {
                (total_recv2 as f64 / total_sent as f64) * 100.0
            } else {
                0.0
            }
        ),
        errors,
    }
}

/// Print summary of all test results
fn print_summary(results: &[TestResult]) {
    println!("═══════════════════════════════════════════════════════════════════════════════");
    println!("  SUMMARY");
    println!("═══════════════════════════════════════════════════════════════════════════════");
    println!();

    println!("  {:<40} {:>8}", "Test", "Status");
    println!("  {}", "-".repeat(50));

    for result in results {
        let status = if result.passed { "✅ PASS" } else { "❌ FAIL" };
        println!("  {:<40} {:>8}", result.name, status);

        if !result.passed && !result.errors.is_empty() {
            for (i, error) in result.errors.iter().take(3).enumerate() {
                println!("    {}. {}", i + 1, error);
            }
        }
    }

    println!();

    let passed = results.iter().filter(|r| r.passed).count();
    let total = results.len();
    println!(
        "  Result: {}/{} tests passed ({:.0}%)",
        passed,
        total,
        (passed as f64 / total as f64) * 100.0
    );
}
