// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]
#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_mut)]

//! Pub/Sub Patterns Stress Test Suite
//!
//! Tests HORUS IPC with various pub/sub topologies:
//! - 1 publisher : N subscribers (fan-out)
//! - N publishers : 1 subscriber (fan-in)
//! - N publishers : M subscribers (mesh)
//! - Dynamic join/leave patterns
//!
//! Acceptance Criteria:
//! - All subscribers receive messages
//! - Fair scheduling across publishers
//! - Dynamic subscription works correctly

use bytemuck::{Pod, Zeroable};
use horus::prelude::Topic;
use horus_core::communication::PodMessage;
use horus_core::core::LogSummary;
use horus_library::messages::cmd_vel::CmdVel;
use serde::{Serialize, Deserialize};
use std::collections::HashSet;
use std::env;
use std::process;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <test_name>", args[0]);
        eprintln!("Available tests (Hub MAX_CONSUMERS=16 per topic):");
        eprintln!("  fanout_N          - 1 pub : N subs (N+1 <= 16)");
        eprintln!("  fanin_N           - N pubs : 1 sub (N+1 <= 16)");
        eprintln!("  mesh_NxM          - N pubs : M subs (N+M <= 16)");
        eprintln!("  dynamic_join      - Dynamic join/leave");
        eprintln!("  link_fanout       - Link fan-out test");
        eprintln!("  multi_topic_fanout - Large fan-out via multiple topics");
        eprintln!("  all               - Run all tests");
        process::exit(1);
    }

    let test_name = &args[1];

    let results = match test_name.as_str() {
        "dynamic_join" => vec![("dynamic_join", test_dynamic_join_leave())],
        "link_fanout" => vec![("link_fanout", test_link_fanout())],
        "multi_topic_fanout" => vec![("multi_topic_fanout", test_multi_topic_fanout())],
        "all" => run_all_tests(),
        _ => {
            // Parse fanout_N, fanin_N, mesh_NxM patterns
            if test_name.starts_with("fanout_") {
                if let Ok(n) = test_name[7..].parse::<usize>() {
                    vec![(test_name.as_str(), test_fanout(n))]
                } else {
                    eprintln!("Invalid fanout count: {}", test_name);
                    process::exit(1);
                }
            } else if test_name.starts_with("fanin_") {
                if let Ok(n) = test_name[6..].parse::<usize>() {
                    vec![(test_name.as_str(), test_fanin(n))]
                } else {
                    eprintln!("Invalid fanin count: {}", test_name);
                    process::exit(1);
                }
            } else if test_name.starts_with("mesh_") {
                let parts: Vec<&str> = test_name[5..].split('x').collect();
                if parts.len() == 2 {
                    if let (Ok(n), Ok(m)) = (parts[0].parse::<usize>(), parts[1].parse::<usize>()) {
                        vec![(test_name.as_str(), test_mesh(n, m))]
                    } else {
                        eprintln!("Invalid mesh dimensions: {}", test_name);
                        process::exit(1);
                    }
                } else {
                    eprintln!("Invalid mesh format (use mesh_NxM): {}", test_name);
                    process::exit(1);
                }
            } else {
                eprintln!("Unknown test: {}", test_name);
                process::exit(1);
            }
        }
    };

    // Print summary
    println!("\n========================================");
    println!("Pub/Sub Patterns Stress Test Results");
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
    // Hub has MAX_CONSUMERS = 16 per topic
    // Tests are adjusted to stay within this limit
    vec![
        ("fanout_8", test_fanout(8)),      // 1 pub + 8 subs = 9 consumers
        ("fanout_14", test_fanout(14)),    // 1 pub + 14 subs = 15 consumers
        ("fanin_8", test_fanin(8)),        // 8 pubs + 1 sub = 9 consumers
        ("fanin_14", test_fanin(14)),      // 14 pubs + 1 sub = 15 consumers
        ("mesh_4x4", test_mesh(4, 4)),     // 4 pubs + 4 subs = 8 consumers
        ("mesh_7x7", test_mesh(7, 7)),     // 7 pubs + 7 subs = 14 consumers
        ("dynamic_join", test_dynamic_join_leave()),
        ("link_fanout", test_link_fanout()),
        ("multi_topic_fanout", test_multi_topic_fanout()), // Tests large fan-out via multiple topics
    ]
}

/// Test: 1 Publisher to N Subscribers (fan-out)
fn test_fanout(num_subs: usize) -> bool {
    println!("\n========================================");
    println!("Test: 1 Publisher : {} Subscribers (Fan-out)", num_subs);
    println!("========================================");

    let topic = format!("fanout_{}_{}", num_subs, process::id());
    let num_messages = 1000;

    // Create publisher
    let pub_hub = match Topic::<CmdVel>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    // Create subscribers
    let mut subscribers = Vec::with_capacity(num_subs);
    for i in 0..num_subs {
        match Topic::<CmdVel>::new(&topic) {
            Ok(h) => subscribers.push(h),
            Err(e) => {
                eprintln!("  Failed to create subscriber {}: {:?}", i, e);
                return false;
            }
        }
    }

    thread::sleep(Duration::from_millis(100));

    // Atomic counters for each subscriber
    let recv_counts: Vec<Arc<AtomicU64>> = (0..num_subs)
        .map(|_| Arc::new(AtomicU64::new(0)))
        .collect();

    let stop_flag = Arc::new(AtomicBool::new(false));

    // Spawn subscriber threads
    let mut sub_handles = Vec::new();
    for (i, sub) in subscribers.into_iter().enumerate() {
        let count = recv_counts[i].clone();
        let stop = stop_flag.clone();

        let handle = thread::spawn(move || {
            while !stop.load(Ordering::Relaxed) {
                if sub.recv(&mut None).is_some() {
                    count.fetch_add(1, Ordering::Relaxed);
                } else {
                    thread::sleep(Duration::from_micros(100));
                }
            }
        });
        sub_handles.push(handle);
    }

    // Send messages
    let start = Instant::now();
    let mut sent = 0u64;
    for i in 0..num_messages {
        let msg = CmdVel {
            linear: i as f32,
            angular: 0.5,
            stamp_nanos: i as u64,
        };
        if pub_hub.send(msg, &mut None).is_ok() {
            sent += 1;
        }
        thread::sleep(Duration::from_millis(1)); // 1kHz
    }

    // Allow time for delivery
    thread::sleep(Duration::from_millis(200));
    stop_flag.store(true, Ordering::SeqCst);

    // Wait for threads
    for handle in sub_handles {
        let _ = handle.join();
    }

    // Calculate results
    let total_recv: u64 = recv_counts.iter()
        .map(|c| c.load(Ordering::Relaxed))
        .sum();

    let min_recv = recv_counts.iter()
        .map(|c| c.load(Ordering::Relaxed))
        .min()
        .unwrap_or(0);

    let max_recv = recv_counts.iter()
        .map(|c| c.load(Ordering::Relaxed))
        .max()
        .unwrap_or(0);

    let avg_recv = total_recv as f64 / num_subs as f64;
    let fairness = if max_recv > 0 { min_recv as f64 / max_recv as f64 } else { 0.0 };

    println!("  Messages sent: {}", sent);
    println!("  Total received (all subs): {}", total_recv);
    println!("  Per-subscriber: min={}, max={}, avg={:.1}", min_recv, max_recv, avg_recv);
    println!("  Fairness ratio: {:.2}", fairness);

    // Hub broadcasts messages to all subscribers
    // Each subscriber receives a copy, so total_recv ≈ sent × num_subs
    // Expected recv_ratio ≈ num_subs
    let expected_total = sent * num_subs as u64;
    let broadcast_ratio = total_recv as f64 / expected_total as f64;

    // Success criteria:
    // 1. Each subscriber receives >= 90% of messages
    // 2. Fairness ratio >= 0.9 (subscribers receive similar amounts)
    let per_sub_ratio = min_recv as f64 / sent as f64;
    let success = per_sub_ratio >= 0.90 && fairness >= 0.9;

    println!("  Per-sub receive rate: {:.1}%", per_sub_ratio * 100.0);
    println!("  Broadcast efficiency: {:.1}%", broadcast_ratio * 100.0);

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

/// Test: N Publishers to 1 Subscriber (fan-in)
fn test_fanin(num_pubs: usize) -> bool {
    println!("\n========================================");
    println!("Test: {} Publishers : 1 Subscriber (Fan-in)", num_pubs);
    println!("========================================");

    let topic = format!("fanin_{}_{}", num_pubs, process::id());
    let messages_per_pub = 100;
    let expected_total = num_pubs * messages_per_pub;

    // Create subscriber first
    let sub_hub = match Topic::<CmdVel>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    // Create publishers
    let mut publishers = Vec::with_capacity(num_pubs);
    for i in 0..num_pubs {
        match Topic::<CmdVel>::new(&topic) {
            Ok(h) => publishers.push(h),
            Err(e) => {
                eprintln!("  Failed to create publisher {}: {:?}", i, e);
                return false;
            }
        }
    }

    thread::sleep(Duration::from_millis(100));

    let recv_count = Arc::new(AtomicU64::new(0));
    let stop_flag = Arc::new(AtomicBool::new(false));
    let received_values = Arc::new(Mutex::new(HashSet::new()));

    // Subscriber thread
    let recv_sub = recv_count.clone();
    let stop_sub = stop_flag.clone();
    let values_sub = received_values.clone();
    let sub_handle = thread::spawn(move || {
        while !stop_sub.load(Ordering::Relaxed) {
            if let Some(msg) = sub_hub.recv(&mut None) {
                recv_sub.fetch_add(1, Ordering::Relaxed);
                if let Ok(mut set) = values_sub.lock() {
                    set.insert(msg.stamp_nanos);
                }
            } else {
                thread::sleep(Duration::from_micros(50));
            }
        }
    });

    // Spawn publisher threads
    let mut pub_handles = Vec::new();
    for (pub_id, pub_hub) in publishers.into_iter().enumerate() {
        let handle = thread::spawn(move || {
            let mut sent = 0u64;
            for msg_id in 0..messages_per_pub {
                let msg = CmdVel {
                    linear: pub_id as f32,
                    angular: msg_id as f32,
                    stamp_nanos: (pub_id * 10000 + msg_id) as u64, // Unique ID
                };
                if pub_hub.send(msg, &mut None).is_ok() {
                    sent += 1;
                }
                thread::sleep(Duration::from_micros(500)); // Stagger sends
            }
            sent
        });
        pub_handles.push(handle);
    }

    // Wait for publishers
    let mut total_sent = 0u64;
    for handle in pub_handles {
        total_sent += handle.join().unwrap_or(0);
    }

    // Allow time for delivery
    thread::sleep(Duration::from_millis(500));
    stop_flag.store(true, Ordering::SeqCst);
    let _ = sub_handle.join();

    let total_recv = recv_count.load(Ordering::Relaxed);
    let unique_values = received_values.lock().map(|s| s.len()).unwrap_or(0);
    let recv_ratio = if total_sent > 0 { total_recv as f64 / total_sent as f64 } else { 0.0 };

    println!("  Publishers: {}", num_pubs);
    println!("  Messages per publisher: {}", messages_per_pub);
    println!("  Total sent: {}", total_sent);
    println!("  Total received: {}", total_recv);
    println!("  Unique values: {}", unique_values);
    println!("  Receive ratio: {:.2}", recv_ratio);

    // Success: Should receive a good portion of messages
    // With Hub MPMC, some overwrites expected at high rates
    let success = recv_ratio >= 0.80;

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

/// Test: N Publishers to M Subscribers (mesh)
fn test_mesh(num_pubs: usize, num_subs: usize) -> bool {
    println!("\n========================================");
    println!("Test: {} Publishers : {} Subscribers (Mesh)", num_pubs, num_subs);
    println!("========================================");

    let topic = format!("mesh_{}_{}_{}", num_pubs, num_subs, process::id());
    let messages_per_pub = 50;

    // Create subscribers
    let mut subscribers = Vec::with_capacity(num_subs);
    for i in 0..num_subs {
        match Topic::<CmdVel>::new(&topic) {
            Ok(h) => subscribers.push(h),
            Err(e) => {
                eprintln!("  Failed to create subscriber {}: {:?}", i, e);
                return false;
            }
        }
    }

    // Create publishers
    let mut publishers = Vec::with_capacity(num_pubs);
    for i in 0..num_pubs {
        match Topic::<CmdVel>::new(&topic) {
            Ok(h) => publishers.push(h),
            Err(e) => {
                eprintln!("  Failed to create publisher {}: {:?}", i, e);
                return false;
            }
        }
    }

    thread::sleep(Duration::from_millis(100));

    // Subscriber counters
    let recv_counts: Vec<Arc<AtomicU64>> = (0..num_subs)
        .map(|_| Arc::new(AtomicU64::new(0)))
        .collect();

    let stop_flag = Arc::new(AtomicBool::new(false));

    // Spawn subscriber threads
    let mut sub_handles = Vec::new();
    for (i, sub) in subscribers.into_iter().enumerate() {
        let count = recv_counts[i].clone();
        let stop = stop_flag.clone();

        let handle = thread::spawn(move || {
            while !stop.load(Ordering::Relaxed) {
                if sub.recv(&mut None).is_some() {
                    count.fetch_add(1, Ordering::Relaxed);
                } else {
                    thread::sleep(Duration::from_micros(50));
                }
            }
        });
        sub_handles.push(handle);
    }

    // Spawn publisher threads
    let mut pub_handles = Vec::new();
    for (pub_id, pub_hub) in publishers.into_iter().enumerate() {
        let handle = thread::spawn(move || {
            let mut sent = 0u64;
            for msg_id in 0..messages_per_pub {
                let msg = CmdVel {
                    linear: pub_id as f32,
                    angular: msg_id as f32,
                    stamp_nanos: (pub_id * 10000 + msg_id) as u64,
                };
                if pub_hub.send(msg, &mut None).is_ok() {
                    sent += 1;
                }
                thread::sleep(Duration::from_micros(200));
            }
            sent
        });
        pub_handles.push(handle);
    }

    // Wait for publishers
    let mut total_sent = 0u64;
    for handle in pub_handles {
        total_sent += handle.join().unwrap_or(0);
    }

    // Allow time for delivery
    thread::sleep(Duration::from_millis(300));
    stop_flag.store(true, Ordering::SeqCst);

    for handle in sub_handles {
        let _ = handle.join();
    }

    // Calculate results
    let total_recv: u64 = recv_counts.iter()
        .map(|c| c.load(Ordering::Relaxed))
        .sum();

    let min_recv = recv_counts.iter()
        .map(|c| c.load(Ordering::Relaxed))
        .min()
        .unwrap_or(0);

    let max_recv = recv_counts.iter()
        .map(|c| c.load(Ordering::Relaxed))
        .max()
        .unwrap_or(0);

    let recv_ratio = if total_sent > 0 { total_recv as f64 / total_sent as f64 } else { 0.0 };
    let fairness = if max_recv > 0 { min_recv as f64 / max_recv as f64 } else { 0.0 };

    println!("  Total sent (all pubs): {}", total_sent);
    println!("  Total received (all subs): {}", total_recv);
    println!("  Per-subscriber: min={}, max={}", min_recv, max_recv);
    println!("  Fairness ratio: {:.2}", fairness);
    println!("  Overall recv ratio: {:.2}", recv_ratio);

    // Success: total messages received ≈ total sent (Hub MPMC)
    let success = recv_ratio >= 0.70;

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

/// Test: Dynamic join and leave
fn test_dynamic_join_leave() -> bool {
    println!("\n========================================");
    println!("Test: Dynamic Join/Leave");
    println!("========================================");

    let topic = format!("dynamic_{}", process::id());
    let phases = 5;
    let messages_per_phase = 100;

    // Create persistent publisher
    let pub_hub = match Topic::<CmdVel>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let total_recv = Arc::new(AtomicU64::new(0));
    let active_subs = Arc::new(AtomicU64::new(0));
    let stop_flag = Arc::new(AtomicBool::new(false));

    // Run phases where subscribers join and leave
    let mut phase_results = Vec::new();

    for phase in 0..phases {
        let num_subs = (phase + 1) * 2; // 2, 4, 6, 8, 10 subscribers

        // Create subscribers for this phase
        let mut subs = Vec::new();
        for _ in 0..num_subs {
            if let Ok(h) = Topic::<CmdVel>::new(&topic) {
                subs.push(h);
            }
        }

        active_subs.store(subs.len() as u64, Ordering::Relaxed);
        thread::sleep(Duration::from_millis(50));

        // Spawn subscriber threads
        let recv_phase = Arc::new(AtomicU64::new(0));
        let stop_phase = Arc::new(AtomicBool::new(false));
        let mut handles = Vec::new();

        for sub in subs.into_iter() {
            let recv = recv_phase.clone();
            let stop = stop_phase.clone();
            handles.push(thread::spawn(move || {
                while !stop.load(Ordering::Relaxed) {
                    if sub.recv(&mut None).is_some() {
                        recv.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }));
        }

        // Send messages for this phase
        let mut sent = 0u64;
        for i in 0..messages_per_phase {
            let msg = CmdVel {
                linear: phase as f32,
                angular: i as f32,
                stamp_nanos: (phase * 10000 + i) as u64,
            };
            if pub_hub.send(msg, &mut None).is_ok() {
                sent += 1;
            }
            thread::sleep(Duration::from_millis(2));
        }

        // Short delay for message delivery
        thread::sleep(Duration::from_millis(100));
        stop_phase.store(true, Ordering::SeqCst);

        // Wait for threads
        for handle in handles {
            let _ = handle.join();
        }

        let recv = recv_phase.load(Ordering::Relaxed);
        let recv_ratio = if sent > 0 { recv as f64 / sent as f64 } else { 0.0 };
        total_recv.fetch_add(recv, Ordering::Relaxed);

        println!("  Phase {}: {} subs, sent={}, recv={}, ratio={:.2}",
            phase, num_subs, sent, recv, recv_ratio);

        phase_results.push(recv_ratio >= 0.70);

        // Short pause between phases
        thread::sleep(Duration::from_millis(100));
    }

    let total = total_recv.load(Ordering::Relaxed);
    let all_phases_passed = phase_results.iter().all(|&p| p);

    println!("  Total received across all phases: {}", total);

    if all_phases_passed {
        println!("  ✓ Test PASSED (all phases successful)");
    } else {
        println!("  ✗ Test FAILED (some phases failed)");
    }

    all_phases_passed
}

/// Test: Link fan-out (1 producer, many consumers)
fn test_link_fanout() -> bool {
    println!("\n========================================");
    println!("Test: Link Fan-out (Multiple Topics)");
    println!("========================================");

    let num_topics = 10;
    let messages_per_topic = 100;

    // Create producer-consumer pairs for each topic
    let mut producers = Vec::new();
    let mut consumers = Vec::new();

    for i in 0..num_topics {
        let topic = format!("link_fanout_{}_{}", i, process::id());

        match Topic::<CmdVel>::new(&topic) {
            Ok(p) => producers.push(p),
            Err(e) => {
                eprintln!("  Failed to create producer {}: {:?}", i, e);
                return false;
            }
        }

        match Topic::<CmdVel>::new(&topic) {
            Ok(c) => consumers.push(c),
            Err(e) => {
                eprintln!("  Failed to create consumer {}: {:?}", i, e);
                return false;
            }
        }
    }

    thread::sleep(Duration::from_millis(50));

    // Counters
    let recv_counts: Vec<Arc<AtomicU64>> = (0..num_topics)
        .map(|_| Arc::new(AtomicU64::new(0)))
        .collect();

    let stop_flag = Arc::new(AtomicBool::new(false));

    // Spawn consumer threads
    let mut cons_handles = Vec::new();
    for (i, consumer) in consumers.into_iter().enumerate() {
        let count = recv_counts[i].clone();
        let stop = stop_flag.clone();

        cons_handles.push(thread::spawn(move || {
            while !stop.load(Ordering::Relaxed) {
                if consumer.recv(&mut None).is_some() {
                    count.fetch_add(1, Ordering::Relaxed);
                } else {
                    thread::sleep(Duration::from_micros(50));
                }
            }
        }));
    }

    // Send messages to each topic
    let start = Instant::now();
    let mut total_sent = 0u64;

    for (topic_id, producer) in producers.iter().enumerate() {
        for msg_id in 0..messages_per_topic {
            let msg = CmdVel {
                linear: topic_id as f32,
                angular: msg_id as f32,
                stamp_nanos: (topic_id * 1000 + msg_id) as u64,
            };
            if producer.send(msg, &mut None).is_ok() {
                total_sent += 1;
            }
        }
    }

    let send_duration = start.elapsed();

    // Allow time for delivery
    thread::sleep(Duration::from_millis(200));
    stop_flag.store(true, Ordering::SeqCst);

    for handle in cons_handles {
        let _ = handle.join();
    }

    // Calculate results
    let total_recv: u64 = recv_counts.iter()
        .map(|c| c.load(Ordering::Relaxed))
        .sum();

    let per_topic: Vec<u64> = recv_counts.iter()
        .map(|c| c.load(Ordering::Relaxed))
        .collect();

    let min_recv = *per_topic.iter().min().unwrap_or(&0);
    let max_recv = *per_topic.iter().max().unwrap_or(&0);

    println!("  Topics: {}", num_topics);
    println!("  Total sent: {} in {:?}", total_sent, send_duration);
    println!("  Total received: {}", total_recv);
    println!("  Per-topic: min={}, max={}", min_recv, max_recv);

    // With Link (SPSC single-slot), only last message survives
    // So total_recv ≤ num_topics (one per topic)
    // Actually, if we send slowly enough, more can be received
    let success = total_recv > 0 && max_recv > 0;

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

/// Test: Large fan-out via multiple topics (workaround for 16-consumer limit)
///
/// Creates N topics, each with 1 pub and up to 15 subs.
/// Publisher broadcasts to all topics, each topic's subs receive.
fn test_multi_topic_fanout() -> bool {
    println!("\n========================================");
    println!("Test: Multi-Topic Fan-out (100 total subscribers)");
    println!("========================================");

    let num_topics = 10;  // 10 topics
    let subs_per_topic = 10;  // 10 subs per topic = 100 total
    let num_messages = 500;

    // Create topics with publishers and subscribers
    let mut topic_pubs = Vec::new();
    let mut all_subs = Vec::new();

    for topic_id in 0..num_topics {
        let topic = format!("multi_fanout_{}_{}", topic_id, process::id());

        // Create publisher for this topic
        match Topic::<CmdVel>::new(&topic) {
            Ok(h) => topic_pubs.push(h),
            Err(e) => {
                eprintln!("  Failed to create publisher for topic {}: {:?}", topic_id, e);
                return false;
            }
        }

        // Create subscribers for this topic
        for sub_id in 0..subs_per_topic {
            match Topic::<CmdVel>::new(&topic) {
                Ok(h) => all_subs.push((topic_id, sub_id, h)),
                Err(e) => {
                    eprintln!("  Failed to create sub {}/{}: {:?}", topic_id, sub_id, e);
                    return false;
                }
            }
        }
    }

    thread::sleep(Duration::from_millis(100));

    println!("  Topics: {}", num_topics);
    println!("  Subs per topic: {}", subs_per_topic);
    println!("  Total subscribers: {}", all_subs.len());

    // Atomic counter for each subscriber
    let recv_counts: Vec<Arc<AtomicU64>> = (0..all_subs.len())
        .map(|_| Arc::new(AtomicU64::new(0)))
        .collect();

    let stop_flag = Arc::new(AtomicBool::new(false));

    // Spawn subscriber threads
    let mut sub_handles = Vec::new();
    for (i, (_topic_id, _sub_id, sub)) in all_subs.into_iter().enumerate() {
        let count = recv_counts[i].clone();
        let stop = stop_flag.clone();

        let handle = thread::spawn(move || {
            while !stop.load(Ordering::Relaxed) {
                if sub.recv(&mut None).is_some() {
                    count.fetch_add(1, Ordering::Relaxed);
                } else {
                    thread::sleep(Duration::from_micros(50));
                }
            }
        });
        sub_handles.push(handle);
    }

    // Broadcast messages to all topics
    let start = Instant::now();
    let mut total_sent = 0u64;

    for i in 0..num_messages {
        let msg = CmdVel {
            linear: i as f32,
            angular: 0.5,
            stamp_nanos: i as u64,
        };

        // Send to all topic publishers
        for pub_hub in &topic_pubs {
            if pub_hub.send(msg, &mut None).is_ok() {
                total_sent += 1;
            }
        }
        thread::sleep(Duration::from_millis(2)); // 500 Hz
    }

    let send_duration = start.elapsed();

    // Allow time for delivery
    thread::sleep(Duration::from_millis(300));
    stop_flag.store(true, Ordering::SeqCst);

    // Wait for threads
    for handle in sub_handles {
        let _ = handle.join();
    }

    // Calculate results
    let total_recv: u64 = recv_counts.iter()
        .map(|c| c.load(Ordering::Relaxed))
        .sum();

    let min_recv = recv_counts.iter()
        .map(|c| c.load(Ordering::Relaxed))
        .min()
        .unwrap_or(0);

    let max_recv = recv_counts.iter()
        .map(|c| c.load(Ordering::Relaxed))
        .max()
        .unwrap_or(0);

    // Each message is sent to num_topics publishers
    let expected_per_sub = num_messages as u64;
    let avg_recv = total_recv as f64 / (num_topics * subs_per_topic) as f64;
    let fairness = if max_recv > 0 { min_recv as f64 / max_recv as f64 } else { 0.0 };

    println!("  Total messages sent (across all topics): {}", total_sent);
    println!("  Send duration: {:?}", send_duration);
    println!("  Total received (all subs): {}", total_recv);
    println!("  Per-subscriber: min={}, max={}, avg={:.1}", min_recv, max_recv, avg_recv);
    println!("  Expected per sub: {}", expected_per_sub);
    println!("  Fairness ratio: {:.2}", fairness);

    // Success: each subscriber receives >= 90% of expected messages
    let per_sub_ratio = min_recv as f64 / expected_per_sub as f64;
    let success = per_sub_ratio >= 0.90 && fairness >= 0.8;

    println!("  Per-sub receive rate: {:.1}%", per_sub_ratio * 100.0);

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}
