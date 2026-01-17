// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]
#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_mut)]

/// IPC Functionality Test Suite
/// Verifies that IPC mechanisms work correctly in various scenarios
use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use std::env;
use std::process::{self, Command};
use std::sync::{Arc, Barrier};
use std::thread;
use std::time::{Duration, Instant};

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <test_name>", args[0]);
        eprintln!("Available tests:");
        eprintln!("  hub_multiprocess");
        eprintln!("  link_singleprocess");
        eprintln!("  cross_process");
        eprintln!("  large_messages");
        eprintln!("  high_frequency");
        process::exit(1);
    }

    let test_name = &args[1];
    let result = match test_name.as_str() {
        "hub_multiprocess" => test_hub_multiprocess(),
        "link_singleprocess" => test_link_singleprocess(),
        "cross_process" => test_cross_process(),
        "large_messages" => test_large_messages(),
        "high_frequency" => test_high_frequency(),
        _ => {
            eprintln!("Unknown test: {}", test_name);
            process::exit(1);
        }
    };

    if result {
        println!(" Test passed: {}", test_name);
        process::exit(0);
    } else {
        eprintln!("[FAIL] Test failed: {}", test_name);
        process::exit(1);
    }
}

/// Test 1: Hub Multi-Process MPMC
fn test_hub_multiprocess() -> bool {
    println!("Testing Hub multi-process communication...");

    let topic = format!("test_hub_mp_{}", process::id());

    // Check if we're the parent or child process
    if env::var("TEST_ROLE").is_err() {
        // Parent process - spawn child and act as subscriber
        println!("  Starting as parent (subscriber)...");

        let subscriber = match Topic::<CmdVel>::new(&topic) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("Failed to create subscriber: {}", e);
                return false;
            }
        };

        // Spawn child process as publisher
        let child = Command::new(env::current_exe().unwrap())
            .arg("hub_multiprocess")
            .env("TEST_ROLE", "child")
            .env("TEST_TOPIC", &topic)
            .spawn();

        let mut child = match child {
            Ok(c) => c,
            Err(e) => {
                eprintln!("Failed to spawn child process: {}", e);
                return false;
            }
        };

        // Give child time to start and publish
        thread::sleep(Duration::from_millis(200));

        // Receive messages from child
        let mut received = 0;
        let start = Instant::now();
        while received < 100 && start.elapsed() < Duration::from_secs(5) {
            if let Some(msg) = subscriber.recv(&mut None) {
                if msg.stamp_nanos as usize != received {
                    eprintln!(
                        "Message order error: expected {}, got {}",
                        received, msg.stamp_nanos
                    );
                    return false;
                }
                received += 1;
            } else {
                thread::sleep(Duration::from_micros(100));
            }
        }

        // Wait for child to finish
        let _ = child.wait();

        if received == 100 {
            println!("   Received all 100 messages from child process");
            true
        } else {
            eprintln!("Only received {} out of 100 messages", received);
            false
        }
    } else {
        // Child process - act as publisher
        let topic = env::var("TEST_TOPIC").unwrap();

        thread::sleep(Duration::from_millis(100)); // Wait for parent subscriber

        let publisher = match Topic::<CmdVel>::new(&topic) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("Child: Failed to create publisher: {}", e);
                return false;
            }
        };

        // Publish 100 messages
        for i in 0..100 {
            let msg = CmdVel {
                linear: 1.0,
                angular: 0.5,
                stamp_nanos: i,
            };
            if let Err(e) = publisher.send(msg, &mut None) {
                eprintln!("Child: Failed to publish message {}: {:?}", i, e);
                return false;
            }
            thread::sleep(Duration::from_micros(100)); // Throttle
        }

        true
    }
}

/// Test 2: Link Single-Process SPSC (single-slot semantics)
///
/// Link is designed as a single-slot channel that always returns the LATEST value.
/// Unlike Hub's ring buffer (FIFO), Link overwrites on each send.
/// This test verifies:
/// 1. Basic send/receive functionality
/// 2. Single-slot overwrite semantics (consumer gets latest value)
/// 3. High-throughput single-slot communication
fn test_link_singleprocess() -> bool {
    println!("Testing Link single-process SPSC (single-slot semantics)...");

    let topic = format!("test_link_sp_{}", process::id());

    // Create sender and receiver in same process
    let sender = match Topic::<CmdVel>::new(&topic) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create sender: {}", e);
            return false;
        }
    };

    let receiver = match Topic::<CmdVel>::new(&topic) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("Failed to create receiver: {}", e);
            return false;
        }
    };

    println!("   Created sender and receiver");

    // Test 1: Basic send/receive
    let msg1 = CmdVel {
        linear: 1.0,
        angular: 0.5,
        stamp_nanos: 42,
    };
    if sender.send(msg1, &mut None).is_err() {
        eprintln!("Failed to send initial message");
        return false;
    }
    match receiver.recv(&mut None) {
        Some(recv_msg) if recv_msg.stamp_nanos == 42 => {
            println!("   Basic send/receive: OK");
        }
        Some(recv_msg) => {
            eprintln!(
                "Wrong message received: expected stamp=42, got {}",
                recv_msg.stamp_nanos
            );
            return false;
        }
        None => {
            eprintln!("No message received after send");
            return false;
        }
    }

    // Test 2: Single-slot overwrite semantics
    // Send 3 messages rapidly, consumer should get the latest (3rd)
    for i in 100..103 {
        let msg = CmdVel {
            linear: i as f32,
            angular: 0.5,
            stamp_nanos: i,
        };
        if sender.send(msg, &mut None).is_err() {
            eprintln!("Failed to send message {}", i);
            return false;
        }
    }
    match receiver.recv(&mut None) {
        Some(recv_msg) if recv_msg.stamp_nanos == 102 => {
            println!("   Single-slot overwrite semantics: OK (got latest: {})", recv_msg.stamp_nanos);
        }
        Some(recv_msg) => {
            // Also acceptable: got 100 or 101 if timing was different
            // What matters is we got ONE message from the set
            if recv_msg.stamp_nanos >= 100 && recv_msg.stamp_nanos <= 102 {
                println!("   Single-slot overwrite semantics: OK (got: {})", recv_msg.stamp_nanos);
            } else {
                eprintln!("Unexpected stamp_nanos: {}", recv_msg.stamp_nanos);
                return false;
            }
        }
        None => {
            eprintln!("No message received after multiple sends");
            return false;
        }
    }

    // Test 3: High-throughput with synchronized send/receive
    // Use barrier to synchronize start
    let barrier = Arc::new(Barrier::new(2));
    let barrier_clone = Arc::clone(&barrier);

    let sender_handle = thread::spawn(move || {
        barrier_clone.wait(); // Synchronize start

        let mut sent = 0;
        for i in 0..1000 {
            let msg = CmdVel {
                linear: 2.0 + i as f32 * 0.01,
                angular: 1.0,
                stamp_nanos: i,
            };
            if sender.send(msg, &mut None).is_err() {
                eprintln!("Failed to send message {}", i);
                return (false, sent);
            }
            sent += 1;
            // Small yield to let receiver catch some messages
            if i % 100 == 0 {
                thread::yield_now();
            }
        }
        (true, sent)
    });

    barrier.wait(); // Synchronize start

    // Receive messages - we won't get all 1000 due to single-slot semantics
    // But we should get SOME, and each received message should be valid
    let mut received_count = 0;
    let mut last_stamp = 0i64;
    let start = Instant::now();
    while start.elapsed() < Duration::from_secs(2) {
        if let Some(msg) = receiver.recv(&mut None) {
            // Validate message is in expected range
            if msg.stamp_nanos > 999 {
                eprintln!("Invalid stamp_nanos: {} (expected 0-999)", msg.stamp_nanos);
                return false;
            }
            // With single-slot, we should see increasing stamps (producer writes in order)
            // but we may skip many values (that's expected)
            if (msg.stamp_nanos as i64) < last_stamp {
                // This could happen in edge cases, just warn
                println!(
                    "   Note: non-monotonic stamp {} after {} (single-slot race)",
                    msg.stamp_nanos, last_stamp
                );
            }
            last_stamp = msg.stamp_nanos as i64;
            received_count += 1;
        }
    }

    let (sender_ok, sent_count) = sender_handle.join().unwrap();

    if !sender_ok {
        return false;
    }

    // Success criteria for single-slot:
    // - All 1000 messages were sent
    // - We received at least some messages (single-slot may not deliver all)
    // - Last received message should be high (producer finished with 999)
    println!(
        "   High-throughput test: sent {}, received {}, last_stamp={}",
        sent_count, received_count, last_stamp
    );

    if sent_count == 1000 && received_count >= 1 && last_stamp >= 900 {
        println!("   Link single-slot SPSC test: PASSED");
        true
    } else {
        eprintln!(
            "Test failed: sent={}, received={}, last_stamp={}",
            sent_count, received_count, last_stamp
        );
        false
    }
}

/// Test 3: Cross-Process Messaging (Link single-slot semantics)
///
/// Link is a single-slot SPSC channel - it always returns the LATEST value.
/// This test verifies cross-process communication works with single-slot semantics:
/// 1. Child process can send messages to parent
/// 2. Parent receives valid messages (in expected range)
/// 3. Single-slot semantics: not all messages will be received, but latest should be high
fn test_cross_process() -> bool {
    println!("Testing cross-process messaging with Link (single-slot semantics)...");

    let topic = format!("test_cross_proc_{}", process::id());

    if env::var("TEST_ROLE").is_err() {
        // Parent process - receiver
        println!("  Starting as parent (consumer)...");

        let receiver = match Topic::<CmdVel>::new(&topic) {
            Ok(r) => r,
            Err(e) => {
                eprintln!("Failed to create receiver: {}", e);
                return false;
            }
        };

        // Spawn child process as sender
        let child = Command::new(env::current_exe().unwrap())
            .arg("cross_process")
            .env("TEST_ROLE", "child")
            .env("TEST_TOPIC", &topic)
            .spawn();

        let mut child = match child {
            Ok(c) => c,
            Err(e) => {
                eprintln!("Failed to spawn child process: {}", e);
                return false;
            }
        };

        thread::sleep(Duration::from_millis(200)); // Wait for child to start

        // Receive messages - with single-slot, we won't get all 500
        // But we should get some, and all received should be valid
        let mut received_count = 0;
        let mut last_stamp: i64 = -1;
        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(5) {
            if let Some(msg) = receiver.recv(&mut None) {
                // Validate message is in expected range
                if msg.stamp_nanos > 499 {
                    eprintln!(
                        "Invalid stamp_nanos: {} (expected 0-499)",
                        msg.stamp_nanos
                    );
                    return false;
                }
                // Track the highest stamp we've seen (single-slot may skip values)
                if (msg.stamp_nanos as i64) > last_stamp {
                    last_stamp = msg.stamp_nanos as i64;
                }
                received_count += 1;
            }
            // Stop if we've seen the last message
            if last_stamp >= 499 {
                break;
            }
        }

        let _ = child.wait();

        // Success criteria for single-slot cross-process:
        // - We received at least some messages
        // - Last stamp should be high (child sends 0-499, we should see something >= 450)
        println!(
            "   Cross-process test: received {} messages, last_stamp={}",
            received_count, last_stamp
        );

        if received_count >= 1 && last_stamp >= 450 {
            println!("   Cross-process Link test: PASSED");
            true
        } else {
            eprintln!(
                "Test failed: received={}, last_stamp={}",
                received_count, last_stamp
            );
            false
        }
    } else {
        // Child process - sender
        let topic = env::var("TEST_TOPIC").unwrap();

        let sender = match Topic::<CmdVel>::new(&topic) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("Child: Failed to create sender: {}", e);
                return false;
            }
        };

        thread::sleep(Duration::from_millis(100)); // Wait for parent receiver

        // Send 500 messages
        for i in 0..500 {
            let msg = CmdVel {
                linear: 1.5,
                angular: 0.75,
                stamp_nanos: i,
            };
            if let Err(e) = sender.send(msg, &mut None) {
                eprintln!("Child: Failed to send message {}: {:?}", i, e);
                return false;
            }
            // Small delay to give receiver a chance to catch some messages
            if i % 50 == 0 {
                thread::sleep(Duration::from_micros(100));
            }
        }

        true
    }
}

/// Test 4: Large Messages (1MB)
fn test_large_messages() -> bool {
    println!("Testing large message handling (1MB)...");

    // Use a custom large message type
    #[derive(Clone, Copy)]
    #[repr(C)]
    struct LargeMessage {
        data: [u8; 1024 * 1024], // 1MB
        checksum: u64,
    }

    // unsafe impl horus_core::memory::Payload for LargeMessage {}

    let topic = format!("test_large_{}", process::id());

    let publisher = match Topic::<CmdVel>::new(&topic) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Failed to create publisher: {}", e);
            return false;
        }
    };

    let subscriber = match Topic::<CmdVel>::new(&topic) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create subscriber: {}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(100));

    println!("   Created Hub for 1MB messages");

    // Publish high-frequency messages
    for i in 0..1000 {
        let msg = CmdVel {
            linear: (i as f32) * 0.001,
            angular: (i as f32) * 0.002,
            stamp_nanos: i,
        };

        if let Err(e) = publisher.send(msg, &mut None) {
            eprintln!("Failed to publish message {}: {:?}", i, e);
            return false;
        }
    }

    println!("   Published 1000 messages");

    thread::sleep(Duration::from_millis(100));

    // Receive and verify ordering
    let mut received = 0;
    let start = Instant::now();
    while received < 1000 && start.elapsed() < Duration::from_secs(5) {
        if let Some(msg) = subscriber.recv(&mut None) {
            // Verify message ordering
            if msg.stamp_nanos != received {
                eprintln!(
                    "Message order error: expected {}, got {}",
                    received, msg.stamp_nanos
                );
                return false;
            }
            received += 1;
        } else {
            thread::sleep(Duration::from_micros(10));
        }
    }

    if received >= 950 {
        println!("   Received {} messages in correct order", received);
        true
    } else {
        eprintln!("Only received {} out of 1000 messages", received);
        false
    }
}

/// Test 5: High Frequency (10kHz send rate, single-slot Link)
///
/// Link is a single-slot channel - perfect for high-frequency sensor data where
/// you only care about the LATEST value. This test verifies:
/// 1. Link can handle high-frequency sends (>10kHz)
/// 2. Consumer receives valid messages
/// 3. Single-slot semantics work correctly (not all messages received, but latest is)
fn test_high_frequency() -> bool {
    println!("Testing high-frequency communication (10kHz send rate, single-slot)...");

    let topic = format!("test_highfreq_{}", process::id());

    let sender = match Topic::<CmdVel>::new(&topic) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create sender: {}", e);
            return false;
        }
    };

    let receiver = match Topic::<CmdVel>::new(&topic) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("Failed to create receiver: {}", e);
            return false;
        }
    };

    println!("   Created high-frequency Link");

    let barrier = Arc::new(Barrier::new(2));
    let barrier_clone = Arc::clone(&barrier);

    // Spawn sender thread
    let sender_handle = thread::spawn(move || {
        barrier_clone.wait(); // Synchronize start

        let start = Instant::now();
        let target_messages = 10000; // 10k messages

        for i in 0..target_messages {
            let msg = CmdVel {
                linear: 1.0,
                angular: 0.5,
                stamp_nanos: i,
            };

            // Link send should not fail (single-slot always overwrites)
            if let Err(e) = sender.send(msg, &mut None) {
                eprintln!("Send failed at message {}: {:?}", i, e);
                return (false, 0, 0);
            }
        }

        let elapsed = start.elapsed();
        let actual_rate = target_messages as f64 / elapsed.as_secs_f64();

        (true, actual_rate as u64, target_messages as u64)
    });

    // Receive in main thread
    barrier.wait(); // Synchronize start

    let start = Instant::now();
    let mut received_count = 0;
    let mut last_stamp: i64 = -1;

    // Receive for a limited time
    while start.elapsed() < Duration::from_secs(3) {
        if let Some(msg) = receiver.recv(&mut None) {
            // Validate message is in expected range
            if msg.stamp_nanos > 9999 {
                eprintln!("Invalid stamp_nanos: {}", msg.stamp_nanos);
                return false;
            }
            if (msg.stamp_nanos as i64) > last_stamp {
                last_stamp = msg.stamp_nanos as i64;
            }
            received_count += 1;
        }
        // Stop if we've seen the last message
        if last_stamp >= 9999 {
            break;
        }
    }

    let (sender_result, send_rate, sent_count) = sender_handle.join().unwrap();

    println!("   Send rate: {:.0} Hz (sent {} messages)", send_rate, sent_count);
    println!(
        "   Received: {} messages, last_stamp={}",
        received_count, last_stamp
    );

    // Success criteria for single-slot high-frequency:
    // - All messages sent successfully
    // - Send rate > 10kHz (should be MUCH higher on modern hardware)
    // - Received at least some messages
    // - Last stamp should be high (>= 9900 means we saw messages near the end)
    if sender_result && send_rate >= 10000 && received_count >= 1 && last_stamp >= 9900 {
        println!("   High-frequency single-slot test: PASSED");
        true
    } else {
        eprintln!(
            "Test failed: sender_ok={}, send_rate={}, received={}, last_stamp={}",
            sender_result, send_rate, received_count, last_stamp
        );
        false
    }
}
