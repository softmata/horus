// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]

/// Stress Test Suite
/// Verifies system behavior under extreme conditions
use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use std::env;
use std::process;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <test_name>", args[0]);
        eprintln!("Available tests:");
        eprintln!("  many_topics");
        eprintln!("  many_nodes");
        eprintln!("  sustained_high_freq");
        eprintln!("  memory_pressure");
        eprintln!("  long_running");
        process::exit(1);
    }

    let test_name = &args[1];
    let result = match test_name.as_str() {
        "many_topics" => test_many_topics(),
        "many_nodes" => test_many_nodes(),
        "sustained_high_freq" => test_sustained_high_freq(),
        "memory_pressure" => test_memory_pressure(),
        "long_running" => test_long_running(),
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

/// Test 1: 1000+ Topics
fn test_many_topics() -> bool {
    println!("Testing with 1000+ topics...");

    let topic_count = 1000;
    let mut publishers = Vec::new();
    let mut subscribers = Vec::new();

    // Create 1000 topics
    for i in 0..topic_count {
        let topic = format!("stress_topic_{}_{}", process::id(), i);

        let pub_result = Topic::<CmdVel>::new(&topic);
        if pub_result.is_err() {
            eprintln!(
                "Failed to create publisher {} of {}: {}",
                i,
                topic_count,
                pub_result.unwrap_err()
            );
            return false;
        }

        let sub_result = Topic::<CmdVel>::new(&topic);
        if sub_result.is_err() {
            eprintln!(
                "Failed to create subscriber {} of {}: {}",
                i,
                topic_count,
                sub_result.unwrap_err()
            );
            return false;
        }

        publishers.push(pub_result.unwrap());
        subscribers.push(sub_result.unwrap());

        if (i + 1) % 100 == 0 {
            println!("  Created {}/{} topics", i + 1, topic_count);
        }
    }

    println!("   Created {} topics successfully", topic_count);

    thread::sleep(Duration::from_millis(500));

    // Publish to all topics
    for (i, pub_ref) in publishers.iter().enumerate() {
        let msg = CmdVel {
            linear: 1.0,
            angular: 0.5,
            stamp_nanos: i as u64,
        };

        if let Err(e) = pub_ref.send(msg, &mut None) {
            eprintln!("Failed to publish to topic {}: {:?}", i, e);
            return false;
        }
    }

    println!("   Published to all {} topics", topic_count);

    thread::sleep(Duration::from_millis(500));

    // Receive from all topics
    let mut received_count = 0;
    for (i, sub_ref) in subscribers.iter().enumerate() {
        if let Some(msg) = sub_ref.recv(&mut None) {
            if msg.stamp_nanos as usize != i {
                eprintln!(
                    "Message mismatch on topic {}: expected {}, got {}",
                    i, i, msg.stamp_nanos
                );
                return false;
            }
            received_count += 1;
        }
    }

    if received_count >= topic_count - 10 {
        // Allow a few drops
        println!("   Received from {}/{} topics", received_count, topic_count);
        true
    } else {
        eprintln!(
            "Only received from {} out of {} topics",
            received_count, topic_count
        );
        false
    }
}

/// Test 2: 100+ Concurrent Communication Channels
fn test_many_nodes() -> bool {
    println!("Testing with 100+ concurrent communication channels...");

    let channel_count = 100;
    let mut senders = Vec::new();
    let mut receivers = Vec::new();

    // Create 100 Link channels
    for i in 0..channel_count {
        let topic = format!("stress_channel_{}_{}", process::id(), i);

        let sender = match Topic::<CmdVel>::new(&topic) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("Failed to create sender {}: {:?}", i, e);
                return false;
            }
        };

        let receiver = match Topic::<CmdVel>::new(&topic) {
            Ok(r) => r,
            Err(e) => {
                eprintln!("Failed to create receiver {}: {:?}", i, e);
                return false;
            }
        };

        senders.push(sender);
        receivers.push(receiver);

        if (i + 1) % 10 == 0 {
            println!("  Created {}/{} channels", i + 1, channel_count);
        }
    }

    println!("   Created {} channels successfully", channel_count);

    // Send and receive on all channels
    let mut total_sent = 0;
    let mut total_received = 0;

    for i in 0..channel_count {
        let msg = CmdVel {
            linear: 1.0,
            angular: 0.5,
            stamp_nanos: i as u64,
        };

        if senders[i].send(msg, &mut None).is_ok() {
            total_sent += 1;
        }
    }

    println!("   Sent {} messages", total_sent);

    thread::sleep(Duration::from_millis(100));

    for i in 0..channel_count {
        if receivers[i].recv(&mut None).is_some() {
            total_received += 1;
        }
    }

    if total_sent == channel_count && total_received >= channel_count - 5 {
        println!(
            "   All channels functional (sent: {}, received: {})",
            total_sent, total_received
        );
        true
    } else {
        eprintln!(
            "Channel communication incomplete: sent={}, received={}",
            total_sent, total_received
        );
        false
    }
}

/// Test 3: Sustained High Frequency (5 minutes)
fn test_sustained_high_freq() -> bool {
    println!("Testing sustained high-frequency messaging (5 minutes)...");

    let topic = format!("stress_highfreq_{}", process::id());

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

    let test_duration = Duration::from_secs(300); // 5 minutes
    let sent_count = Arc::new(Mutex::new(0u64));
    let recv_count = Arc::new(Mutex::new(0u64));
    let errors = Arc::new(Mutex::new(0u64));

    let sent_clone = Arc::clone(&sent_count);
    let errors_clone = Arc::clone(&errors);

    // Spawn sender thread
    let sender_handle = thread::spawn(move || {
        let start = Instant::now();
        let mut count = 0u64;

        while start.elapsed() < test_duration {
            let msg = CmdVel {
                linear: 1.0,
                angular: 0.5,
                stamp_nanos: count,
            };

            match sender.send(msg, &mut None) {
                Ok(_) => count += 1,
                Err(_) => {
                    let mut errs = errors_clone.lock().unwrap();
                    *errs += 1;
                }
            }

            // Target: 10kHz
            thread::sleep(Duration::from_micros(100));
        }

        *sent_clone.lock().unwrap() = count;
    });

    let recv_clone = Arc::clone(&recv_count);

    // Spawn receiver thread
    let receiver_handle = thread::spawn(move || {
        let start = Instant::now();
        let mut count = 0u64;

        while start.elapsed() < test_duration {
            if receiver.recv(&mut None).is_some() {
                count += 1;
            }
        }

        *recv_clone.lock().unwrap() = count;
    });

    // Monitor progress
    let monitor_start = Instant::now();
    while monitor_start.elapsed() < test_duration {
        thread::sleep(Duration::from_secs(30));
        let elapsed = monitor_start.elapsed().as_secs();
        let sent = *sent_count.lock().unwrap();
        let recv = *recv_count.lock().unwrap();
        let errs = *errors.lock().unwrap();
        println!(
            "  Progress: {}s - sent: {}, recv: {}, errors: {}",
            elapsed, sent, recv, errs
        );
    }

    sender_handle.join().unwrap();
    receiver_handle.join().unwrap();

    let final_sent = *sent_count.lock().unwrap();
    let final_recv = *recv_count.lock().unwrap();
    let final_errors = *errors.lock().unwrap();

    println!("   Sustained test completed");
    println!("  Total sent: {}", final_sent);
    println!("  Total received: {}", final_recv);
    println!("  Total errors: {}", final_errors);

    // Should have sent/received millions of messages
    let min_expected = 1_000_000; // At least 1M messages in 5 minutes

    if final_sent >= min_expected && final_recv >= min_expected * 95 / 100 {
        println!("   Sustained high-frequency messaging successful");
        true
    } else {
        eprintln!(
            "Insufficient message throughput: sent={}, recv={}",
            final_sent, final_recv
        );
        false
    }
}

/// Test 4: Memory Pressure
fn test_memory_pressure() -> bool {
    println!("Testing under memory pressure...");

    let iteration_count = 10000;
    let mut topics = Vec::new();

    for i in 0..iteration_count {
        // Create topic
        let topic = format!("stress_mem_{}_{}", process::id(), i % 100); // Reuse 100 topic names

        let publisher = match Topic::<CmdVel>::new(&topic) {
            Ok(p) => p,
            Err(e) => {
                eprintln!("Failed to create publisher at iteration {}: {:?}", i, e);
                return false;
            }
        };

        let subscriber = match Topic::<CmdVel>::new(&topic) {
            Ok(s) => s,
            Err(e) => {
                eprintln!("Failed to create subscriber at iteration {}: {:?}", i, e);
                return false;
            }
        };

        // Publish messages
        for j in 0..10 {
            let msg = CmdVel {
                linear: 1.0,
                angular: 0.5,
                stamp_nanos: j,
            };
            let _ = publisher.send(msg, &mut None);
        }

        // Keep some topics alive, drop others
        if i % 10 == 0 {
            topics.push((publisher, subscriber));
            if topics.len() > 100 {
                topics.remove(0); // Drop oldest
            }
        }

        if (i + 1) % 1000 == 0 {
            println!("  Completed {}/{} iterations", i + 1, iteration_count);
        }
    }

    println!(
        "   Completed {} iterations without memory exhaustion",
        iteration_count
    );
    println!("   Currently holding {} active topics", topics.len());
    true
}

/// Test 5: Long Running Stability (30 minutes)
fn test_long_running() -> bool {
    println!("Testing long-running stability (30 minutes)...");
    println!("  WARNING: This test will run for 30 minutes");

    let topic = format!("stress_longrun_{}", process::id());

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

    println!("   Created long-running Link");

    let test_duration = Duration::from_secs(1800); // 30 minutes
    let sent_count = Arc::new(Mutex::new(0u64));
    let recv_count = Arc::new(Mutex::new(0u64));

    let sent_clone = Arc::clone(&sent_count);

    // Spawn sender thread
    let sender_handle = thread::spawn(move || {
        let start = Instant::now();
        let mut count = 0u64;
        let mut last_error_time = Instant::now();

        while start.elapsed() < test_duration {
            let msg = CmdVel {
                linear: (count as f32 * 0.001).sin(),
                angular: (count as f32 * 0.002).cos(),
                stamp_nanos: count,
            };

            match sender.send(msg, &mut None) {
                Ok(_) => count += 1,
                Err(_) => {
                    // Log errors but don't fail immediately
                    if last_error_time.elapsed() > Duration::from_secs(10) {
                        eprintln!("  Warning: Send errors occurring at count {}", count);
                        last_error_time = Instant::now();
                    }
                }
            }

            // 1kHz rate
            thread::sleep(Duration::from_millis(1));
        }

        *sent_clone.lock().unwrap() = count;
        count
    });

    let recv_clone = Arc::clone(&recv_count);

    // Spawn receiver thread
    let receiver_handle = thread::spawn(move || {
        let start = Instant::now();
        let mut count = 0u64;
        let mut expected_timestamp = 0u64;
        let mut order_errors = 0u64;

        while start.elapsed() < test_duration {
            if let Some(msg) = receiver.recv(&mut None) {
                // Check message order
                if msg.stamp_nanos < expected_timestamp {
                    order_errors += 1;
                }
                expected_timestamp = msg.stamp_nanos + 1;
                count += 1;
            }
        }

        if order_errors > 0 {
            eprintln!("  Warning: {} message order errors detected", order_errors);
        }

        *recv_clone.lock().unwrap() = count;
        (count, order_errors)
    });

    // Monitor progress every 5 minutes
    let monitor_start = Instant::now();
    while monitor_start.elapsed() < test_duration {
        thread::sleep(Duration::from_secs(300)); // 5 minutes
        let elapsed = monitor_start.elapsed().as_secs() / 60;
        let sent = *sent_count.lock().unwrap();
        let recv = *recv_count.lock().unwrap();
        println!(
            "  Progress: {} minutes - sent: {}, recv: {}",
            elapsed, sent, recv
        );
    }

    let final_sent = sender_handle.join().unwrap();
    let (final_recv, order_errors) = receiver_handle.join().unwrap();

    println!("   Long-running test completed (30 minutes)");
    println!("  Total sent: {}", final_sent);
    println!("  Total received: {}", final_recv);
    println!("  Message order errors: {}", order_errors);

    // Should have exchanged millions of messages
    let min_expected = 1_000_000; // At least 1M in 30 minutes

    if final_sent >= min_expected && final_recv >= min_expected * 95 / 100 && order_errors < 1000 {
        println!("   Long-running stability test passed");
        true
    } else {
        eprintln!(
            "Stability test failed: sent={}, recv={}, errors={}",
            final_sent, final_recv, order_errors
        );
        false
    }
}
