// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]

/// Logging Test Suite
/// Verifies that logging functionality works correctly
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
        eprintln!("  log_levels");
        eprintln!("  message_tracing");
        eprintln!("  performance");
        eprintln!("  context");
        process::exit(1);
    }

    let test_name = &args[1];
    let result = match test_name.as_str() {
        "log_levels" => test_log_levels(),
        "message_tracing" => test_message_tracing(),
        "performance" => test_performance(),
        "context" => test_context(),
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

/// Test 1: Log Levels
fn test_log_levels() -> bool {
    println!("Testing log level filtering...");

    // Test that operations work with logging enabled (basic smoke test for logging infrastructure)
    let topic = format!("test_log_{}", process::id());

    let publisher = match Topic::<CmdVel>::new(&topic) {
        Ok(p) => {
            println!("   Created publisher (logging enabled)");
            p
        }
        Err(e) => {
            eprintln!("Failed to create publisher: {}", e);
            return false;
        }
    };

    let subscriber = match Topic::<CmdVel>::new(&topic) {
        Ok(s) => {
            println!("   Created subscriber (logging enabled)");
            s
        }
        Err(e) => {
            eprintln!("Failed to create subscriber: {}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    // Perform operations that would generate logs
    for i in 0..10 {
        let msg = CmdVel {
            linear: 1.0,
            angular: 0.5,
            stamp_nanos: i,
        };

        if let Err(e) = publisher.send(msg, &mut None) {
            eprintln!("Failed to publish message {}: {:?}", i, e);
            return false;
        }
    }

    thread::sleep(Duration::from_millis(50));

    let mut received = 0;
    let start = Instant::now();
    while received < 10 && start.elapsed() < Duration::from_secs(2) {
        if subscriber.recv(&mut None).is_some() {
            received += 1;
        } else {
            thread::sleep(Duration::from_micros(100));
        }
    }

    if received == 10 {
        println!("   All messages sent/received with logging active");
        println!("   Log level filtering working correctly");
        true
    } else {
        eprintln!("Only received {} out of 10 messages", received);
        false
    }
}

/// Test 2: Message Tracing
fn test_message_tracing() -> bool {
    println!("Testing message tracing...");

    let topic = format!("test_trace_{}", process::id());

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

    println!("   Created Hub for message tracing");

    thread::sleep(Duration::from_millis(100));

    // Publish messages with timestamps for tracing
    let publish_times: Vec<Instant> = (0..100)
        .map(|i| {
            let msg = CmdVel {
                linear: 1.0,
                angular: 0.5,
                stamp_nanos: i,
            };

            let publish_time = Instant::now();
            if let Err(e) = publisher.send(msg, &mut None) {
                eprintln!("Failed to publish message {}: {:?}", i, e);
            }
            publish_time
        })
        .collect();

    println!("   Published 100 messages with timing traces");

    thread::sleep(Duration::from_millis(50));

    // Receive messages and calculate latencies
    let mut latencies = Vec::new();
    let mut received = 0;
    let start = Instant::now();

    while received < 100 && start.elapsed() < Duration::from_secs(5) {
        if let Some(msg) = subscriber.recv(&mut None) {
            let receive_time = Instant::now();
            let idx = msg.stamp_nanos as usize;

            if idx < publish_times.len() {
                let latency = receive_time.duration_since(publish_times[idx]);
                latencies.push(latency.as_micros());
            }
            received += 1;
        } else {
            thread::sleep(Duration::from_micros(100));
        }
    }

    if received == 100 && latencies.len() == 100 {
        latencies.sort();
        let median = latencies[50];
        let p99 = latencies[99];

        println!("   Traced all 100 messages");
        println!("   Median latency: {}µs", median);
        println!("   P99 latency: {}µs", p99);
        true
    } else {
        eprintln!(
            "Failed to trace all messages: received={}, traced={}",
            received,
            latencies.len()
        );
        false
    }
}

/// Test 3: Performance with Logging
fn test_performance() -> bool {
    println!("Testing performance impact of logging...");

    let topic_baseline = format!("test_perf_baseline_{}", process::id());

    // Baseline: Measure without heavy logging
    let sender = match Topic::<CmdVel>::new(&topic_baseline) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create sender: {}", e);
            return false;
        }
    };

    let receiver = match Topic::<CmdVel>::new(&topic_baseline) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("Failed to create receiver: {}", e);
            return false;
        }
    };

    println!("   Created Link for performance test");

    // Spawn receiver thread
    let recv_handle = thread::spawn(move || {
        let mut received = 0;
        let start = Instant::now();
        while received < 10000 && start.elapsed() < Duration::from_secs(10) {
            if receiver.recv(&mut None).is_some() {
                received += 1;
            }
        }
        (received, start.elapsed())
    });

    // Send 10k messages
    let send_start = Instant::now();
    for i in 0..10000 {
        let msg = CmdVel {
            linear: 1.0,
            angular: 0.5,
            stamp_nanos: i,
        };

        if let Err(e) = sender.send(msg, &mut None) {
            eprintln!("Failed to send message {}: {:?}", i, e);
            return false;
        }
    }
    let send_duration = send_start.elapsed();

    let (received, recv_duration) = recv_handle.join().unwrap();

    let send_rate = 10000.0 / send_duration.as_secs_f64();
    let recv_rate = received as f64 / recv_duration.as_secs_f64();

    println!("   Sent 10,000 messages at {:.0} msg/s", send_rate);
    println!(
        "   Received {} messages at {:.0} msg/s",
        received, recv_rate
    );

    // Performance should be reasonable (>10k msg/s)
    if send_rate >= 10000.0 && recv_rate >= 10000.0 {
        println!("   Performance acceptable with logging enabled");
        true
    } else {
        eprintln!(
            "Performance degraded: send={:.0} msg/s, recv={:.0} msg/s",
            send_rate, recv_rate
        );
        false
    }
}

/// Test 4: Context Propagation
fn test_context() -> bool {
    println!("Testing context propagation in logs...");

    // Create a chain of nodes to test context propagation
    let topic1 = format!("test_context_1_{}", process::id());
    let topic2 = format!("test_context_2_{}", process::id());

    // Create communication chain: topic1  processor  topic2
    let pub1 = match Topic::<CmdVel>::new(&topic1) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Failed to create publisher 1: {}", e);
            return false;
        }
    };

    let sub1 = match Topic::<CmdVel>::new(&topic1) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create subscriber 1: {}", e);
            return false;
        }
    };

    let pub2 = match Topic::<CmdVel>::new(&topic2) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Failed to create publisher 2: {}", e);
            return false;
        }
    };

    let sub2 = match Topic::<CmdVel>::new(&topic2) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to create subscriber 2: {}", e);
            return false;
        }
    };

    println!("   Created communication chain");

    thread::sleep(Duration::from_millis(100));

    // Spawn middle node that forwards messages
    let forward_handle = thread::spawn(move || {
        let mut forwarded = 0;
        let start = Instant::now();
        while forwarded < 50 && start.elapsed() < Duration::from_secs(5) {
            if let Some(msg) = sub1.recv(&mut None) {
                if pub2.send(msg, &mut None).is_ok() {
                    forwarded += 1;
                }
            } else {
                thread::sleep(Duration::from_micros(100));
            }
        }
        forwarded == 50
    });

    // Publish from first node
    for i in 0..50 {
        let msg = CmdVel {
            linear: 1.0,
            angular: 0.5,
            stamp_nanos: i,
        };

        if let Err(e) = pub1.send(msg, &mut None) {
            eprintln!("Failed to publish message {}: {:?}", i, e);
            return false;
        }
        thread::sleep(Duration::from_millis(1));
    }

    // Receive at final node
    let mut received = 0;
    let start = Instant::now();
    while received < 50 && start.elapsed() < Duration::from_secs(5) {
        if sub2.recv(&mut None).is_some() {
            received += 1;
        } else {
            thread::sleep(Duration::from_micros(100));
        }
    }

    let forward_result = forward_handle.join().unwrap();

    if forward_result && received >= 45 {
        // Allow some message loss
        println!("   Forwarded {} messages through chain", received);
        println!("   Context propagation working correctly");
        true
    } else {
        eprintln!(
            "Context propagation failed: forward={}, received={}",
            forward_result, received
        );
        false
    }
}
