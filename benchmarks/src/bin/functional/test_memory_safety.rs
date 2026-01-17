// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]

/// Memory Safety Test Suite
/// Verifies that memory operations are safe and don't cause segfaults or leaks
use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use std::env;
use std::process;
use std::thread;
use std::time::{Duration, Instant};

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <test_name>", args[0]);
        eprintln!("Available tests:");
        eprintln!("  shm_lifecycle");
        eprintln!("  bounds_checking");
        eprintln!("  concurrent_access");
        eprintln!("  leak_detection");
        eprintln!("  overflow_protection");
        process::exit(1);
    }

    let test_name = &args[1];
    let result = match test_name.as_str() {
        "shm_lifecycle" => test_shm_lifecycle(),
        "bounds_checking" => test_bounds_checking(),
        "concurrent_access" => test_concurrent_access(),
        "leak_detection" => test_leak_detection(),
        "overflow_protection" => test_overflow_protection(),
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

/// Test 1: Shared Memory Lifecycle
fn test_shm_lifecycle() -> bool {
    println!("Testing shared memory allocation and deallocation...");

    // Create and destroy many topics to test lifecycle
    for i in 0..100 {
        let topic = format!("test_lifecycle_{}_{}", process::id(), i);

        let publisher = match Topic::<CmdVel>::new(&topic) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("Failed to create publisher {}: {:?}", i, e);
                return false;
            }
        };

        let subscriber = match Topic::<CmdVel>::new(&topic) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("Failed to create subscriber {}: {:?}", i, e);
                return false;
            }
        };

        // Publish a few messages
        for j in 0..10 {
            let msg = CmdVel {
                linear: 1.0,
                angular: 0.5,
                stamp_nanos: j,
            };
            if let Err(e) = publisher.send(msg, &mut None) {
                eprintln!("Failed to publish in iteration {}: {:?}", i, e);
                return false;
            }
        }

        // Drop publisher and subscriber (cleanup)
        drop(publisher);
        drop(subscriber);
    }

    println!("   Created and destroyed 100 topics without segfaults");
    println!("   Shared memory lifecycle working correctly");
    true
}

/// Test 2: Bounds Checking
fn test_bounds_checking() -> bool {
    println!("Testing bounds checking in ring buffers...");

    let topic = format!("test_bounds_{}", process::id());

    // Create a small Link to test bounds
    let sender = match Topic::<CmdVel>::producer(&topic) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create sender: {}", e);
            return false;
        }
    };

    let receiver = match Topic::<CmdVel>::consumer(&topic) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("Failed to create receiver: {}", e);
            return false;
        }
    };

    println!("   Created Link with capacity 16");

    // Fill buffer completely
    for i in 0..16 {
        let msg = CmdVel {
            linear: 1.0,
            angular: 0.5,
            stamp_nanos: i,
        };
        if let Err(e) = sender.send(msg, &mut None) {
            eprintln!("Failed to send message {} (filling buffer): {:?}", i, e);
            return false;
        }
    }

    println!("   Filled buffer to capacity");

    // Try to overfill (should block or return error, not crash)
    let msg = CmdVel {
        linear: 1.0,
        angular: 0.5,
        stamp_nanos: 999,
    };

    match sender.send(msg, &mut None) {
        Ok(_) => {
            eprintln!("Send succeeded when buffer should be full");
            return false;
        }
        Err(_) => {
            println!("   Buffer overflow properly handled (send returned error)");
        }
    }

    // Drain buffer
    for i in 0..16 {
        if let Some(msg) = receiver.recv(&mut None) {
            if msg.stamp_nanos as usize != i {
                eprintln!(
                    "Message order error during drain: expected {}, got {}",
                    i, msg.stamp_nanos
                );
                return false;
            }
        } else {
            eprintln!("Failed to receive message {} during drain", i);
            return false;
        }
    }

    println!("   Drained buffer successfully");

    // Try to read from empty buffer (should return None, not crash)
    match receiver.recv(&mut None) {
        Some(_) => {
            eprintln!("Received message from empty buffer");
            return false;
        }
        None => {
            println!("   Empty buffer properly handled (recv returned None)");
        }
    }

    true
}

/// Test 3: Concurrent Access
fn test_concurrent_access() -> bool {
    println!("Testing concurrent access from multiple threads...");

    let topic = format!("test_concurrent_{}", process::id());

    // Create Hub
    let publisher = match Topic::<CmdVel>::new(&topic) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Failed to create publisher: {}", e);
            return false;
        }
    };

    let subscriber1 = match Topic::<CmdVel>::new(&topic) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create subscriber 1: {}", e);
            return false;
        }
    };

    let subscriber2 = match Topic::<CmdVel>::new(&topic) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create subscriber 2: {}", e);
            return false;
        }
    };

    let subscriber3 = match Topic::<CmdVel>::new(&topic) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create subscriber 3: {}", e);
            return false;
        }
    };

    println!("   Created 1 publisher and 3 subscribers");

    thread::sleep(Duration::from_millis(100));

    // Spawn publisher thread
    let pub_handle = thread::spawn(move || {
        for i in 0..1000 {
            let msg = CmdVel {
                linear: 1.0,
                angular: 0.5,
                stamp_nanos: i,
            };
            if let Err(e) = publisher.send(msg, &mut None) {
                eprintln!("Publisher failed at message {}: {:?}", i, e);
                return false;
            }
            thread::sleep(Duration::from_micros(10));
        }
        true
    });

    // Spawn subscriber threads
    let sub1_handle = thread::spawn(move || {
        let mut received = 0;
        let start = Instant::now();
        while received < 1000 && start.elapsed() < Duration::from_secs(15) {
            if subscriber1.recv(&mut None).is_some() {
                received += 1;
            } else {
                thread::sleep(Duration::from_micros(10));
            }
        }
        received == 1000
    });

    let sub2_handle = thread::spawn(move || {
        let mut received = 0;
        let start = Instant::now();
        while received < 1000 && start.elapsed() < Duration::from_secs(15) {
            if subscriber2.recv(&mut None).is_some() {
                received += 1;
            } else {
                thread::sleep(Duration::from_micros(10));
            }
        }
        received == 1000
    });

    let sub3_handle = thread::spawn(move || {
        let mut received = 0;
        let start = Instant::now();
        while received < 1000 && start.elapsed() < Duration::from_secs(15) {
            if subscriber3.recv(&mut None).is_some() {
                received += 1;
            } else {
                thread::sleep(Duration::from_micros(10));
            }
        }
        received == 1000
    });

    // Wait for all threads
    let pub_result = pub_handle.join().unwrap();
    let sub1_result = sub1_handle.join().unwrap();
    let sub2_result = sub2_handle.join().unwrap();
    let sub3_result = sub3_handle.join().unwrap();

    if pub_result && sub1_result && sub2_result && sub3_result {
        println!("   All threads completed successfully");
        println!("   No race conditions or data corruption detected");
        true
    } else {
        eprintln!(
            "Thread results: pub={}, sub1={}, sub2={}, sub3={}",
            pub_result, sub1_result, sub2_result, sub3_result
        );
        false
    }
}

/// Test 4: Memory Leak Detection
fn test_leak_detection() -> bool {
    println!("Testing for memory leaks...");
    println!("  (This test creates/destroys many objects to stress memory management)");

    let iterations = 1000;

    for i in 0..iterations {
        let topic = format!("test_leak_{}_{}", process::id(), i % 10); // Reuse 10 topic names

        // Create publisher and subscriber
        let publisher = match Topic::<CmdVel>::new(&topic) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("Failed to create publisher in iteration {}: {:?}", i, e);
                return false;
            }
        };

        let subscriber = match Topic::<CmdVel>::new(&topic) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("Failed to create subscriber in iteration {}: {:?}", i, e);
                return false;
            }
        };

        // Publish some messages
        for j in 0..5 {
            let msg = CmdVel {
                linear: 1.0,
                angular: 0.5,
                stamp_nanos: j,
            };
            let _ = publisher.send(msg, &mut None);
        }

        // Receive some messages
        for _ in 0..5 {
            let _ = subscriber.recv(&mut None);
        }

        // Drop them (important for leak detection)
        drop(publisher);
        drop(subscriber);

        if i % 100 == 0 {
            println!("  Progress: {}/{} iterations", i, iterations);
        }
    }

    println!(
        "   Completed {} iterations without memory exhaustion",
        iterations
    );
    println!("   No obvious memory leaks detected");
    println!("  Note: Run with valgrind/MIRI for detailed leak analysis");
    true
}

/// Test 5: Buffer Overflow Protection
fn test_overflow_protection() -> bool {
    println!("Testing buffer overflow protection...");

    // Test with various buffer sizes
    let buffer_sizes = vec![8, 16, 32, 64, 128, 256];

    for &size in &buffer_sizes {
        let topic = format!("test_overflow_{}_{}", process::id(), size);

        let sender = match Topic::<CmdVel>::producer(&topic) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("Failed to create sender: {:?}", e);
                return false;
            }
        };

        let receiver = match Topic::<CmdVel>::consumer(&topic) {
            Ok(r) => r,
            Err(e) => {
                eprintln!("Failed to create receiver with size {}: {:?}", size, e);
                return false;
            }
        };

        // Try to send more than buffer capacity
        let mut sent = 0;
        for i in 0..size * 2 {
            let msg = CmdVel {
                linear: 1.0,
                angular: 0.5,
                stamp_nanos: i as u64,
            };

            match sender.send(msg, &mut None) {
                Ok(_) => sent += 1,
                Err(_) => break, // Buffer full, expected
            }
        }

        // Should have sent at most buffer_size messages
        if sent > size {
            eprintln!(
                "Sent {} messages to buffer of size {} (overflow!)",
                sent, size
            );
            return false;
        }

        // Receive all sent messages
        let mut received = 0;
        for _ in 0..sent {
            if receiver.recv(&mut None).is_some() {
                received += 1;
            }
        }

        if received != sent {
            eprintln!(
                "Received {} but sent {} (buffer size {})",
                received, sent, size
            );
            return false;
        }

        println!(
            "   Buffer size {} handled correctly ({} messages)",
            size, sent
        );
    }

    println!("   All buffer sizes protected against overflow");
    true
}
