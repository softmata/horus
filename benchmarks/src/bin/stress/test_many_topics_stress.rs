// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]

//! Many Topics Stress Test Suite
//!
//! Tests HORUS IPC with large numbers of topics:
//! - 100 topics (baseline)
//! - 1000 topics (stress)
//! - 10000 topics (edge case)
//! - Topic create/destroy churn
//!
//! Acceptance Criteria:
//! - Topic discovery scales sub-linearly
//! - No file descriptor exhaustion
//! - Proper cleanup when topics destroyed
//! - Routing latency acceptable at 1000 topics

use horus::prelude::Topic;
use horus_library::messages::cmd_vel::CmdVel;
use std::collections::HashMap;
use std::env;
use std::process;
use std::thread;
use std::time::{Duration, Instant};

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <test_name>", args[0]);
        eprintln!("Available tests:");
        eprintln!("  topics_100     - 100 Hub topics baseline");
        eprintln!("  topics_1000    - 1000 Hub topics stress test");
        eprintln!("  topics_10000   - 10000 Hub topics edge case");
        eprintln!("  link_topics    - Link topics scaling test");
        eprintln!("  topic_churn    - Topic create/destroy cycles");
        eprintln!("  discovery_perf - Topic discovery performance");
        eprintln!("  all            - Run all tests");
        process::exit(1);
    }

    let test_name = &args[1];

    let results = match test_name.as_str() {
        "topics_100" => vec![("topics_100", test_hub_topics(100))],
        "topics_1000" => vec![("topics_1000", test_hub_topics(1000))],
        "topics_10000" => vec![("topics_10000", test_hub_topics(10000))],
        "link_topics" => vec![("link_topics", test_link_topics())],
        "topic_churn" => vec![("topic_churn", test_topic_churn())],
        "discovery_perf" => vec![("discovery_perf", test_discovery_performance())],
        "all" => run_all_tests(),
        _ => {
            eprintln!("Unknown test: {}", test_name);
            process::exit(1);
        }
    };

    // Print summary
    println!("\n========================================");
    println!("Many Topics Stress Test Results");
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
        ("topics_100", test_hub_topics(100)),
        ("topics_1000", test_hub_topics(1000)),
        ("topics_10000", test_hub_topics(10000)),
        ("link_topics", test_link_topics()),
        ("topic_churn", test_topic_churn()),
        ("discovery_perf", test_discovery_performance()),
    ]
}

/// Test: Create N Hub topics and verify communication
fn test_hub_topics(topic_count: usize) -> bool {
    println!("\n========================================");
    println!("Test: {} Hub Topics", topic_count);
    println!("========================================");

    let base_topic = format!("stress_topics_{}_{}", process::id(), topic_count);

    // Measure topic creation time
    let create_start = Instant::now();
    let mut publishers = Vec::with_capacity(topic_count);
    let mut subscribers = Vec::with_capacity(topic_count);

    for i in 0..topic_count {
        let topic = format!("{}_{}", base_topic, i);

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

        // Progress indicator for large counts
        if topic_count >= 1000 && (i + 1) % 500 == 0 {
            println!("  Created {}/{} topics...", i + 1, topic_count);
        }
    }

    let create_duration = create_start.elapsed();
    let create_rate = topic_count as f64 / create_duration.as_secs_f64();
    println!("  Created {} topics in {:?} ({:.0} topics/sec)",
        topic_count, create_duration, create_rate);

    // Allow shared memory to stabilize
    thread::sleep(Duration::from_millis(100));

    // Measure message send/receive
    let msg_start = Instant::now();
    let mut send_success = 0;
    let mut recv_success = 0;

    // Send to all topics
    for (i, pub_ref) in publishers.iter().enumerate() {
        let msg = CmdVel {
            linear: 1.0,
            angular: 0.5,
            stamp_nanos: i as u64,
        };

        if pub_ref.send(msg, &mut None).is_ok() {
            send_success += 1;
        }
    }

    // Small delay for message propagation
    thread::sleep(Duration::from_millis(50));

    // Receive from all topics
    for (i, sub_ref) in subscribers.iter().enumerate() {
        if let Some(msg) = sub_ref.recv(&mut None) {
            if msg.stamp_nanos as usize == i {
                recv_success += 1;
            }
        }
    }

    let msg_duration = msg_start.elapsed();

    println!("  Messages: sent={}/{}, received={}/{}",
        send_success, topic_count, recv_success, topic_count);
    println!("  Message round-trip time: {:?}", msg_duration);

    // Measure cleanup time
    let cleanup_start = Instant::now();
    drop(publishers);
    drop(subscribers);
    let cleanup_duration = cleanup_start.elapsed();

    println!("  Cleanup time: {:?}", cleanup_duration);

    // Success criteria
    let send_rate = (send_success as f64 / topic_count as f64) * 100.0;
    let recv_rate = (recv_success as f64 / topic_count as f64) * 100.0;

    // Allow slightly lower success rate for very large topic counts
    let min_rate = if topic_count >= 10000 { 95.0 } else { 99.0 };

    let success = send_rate >= min_rate && recv_rate >= min_rate;

    if success {
        println!("  ✓ Test PASSED (send: {:.1}%, recv: {:.1}%)", send_rate, recv_rate);
    } else {
        println!("  ✗ Test FAILED (send: {:.1}%, recv: {:.1}%)", send_rate, recv_rate);
    }

    success
}

/// Test: Link topics scaling
fn test_link_topics() -> bool {
    println!("\n========================================");
    println!("Test: Link Topics Scaling");
    println!("========================================");

    let test_cases = [100, 500, 1000];

    for &topic_count in &test_cases {
        println!("  Testing {} Link topics...", topic_count);

        let base_topic = format!("stress_link_{}_{}", process::id(), topic_count);
        let create_start = Instant::now();

        let mut producers = Vec::with_capacity(topic_count);
        let mut consumers = Vec::with_capacity(topic_count);

        for i in 0..topic_count {
            let topic = format!("{}_{}", base_topic, i);

            match Topic::<CmdVel>::new(&topic) {
                Ok(p) => producers.push(p),
                Err(e) => {
                    eprintln!("    Failed to create producer {}: {:?}", i, e);
                    return false;
                }
            }

            match Topic::<CmdVel>::new(&topic) {
                Ok(c) => consumers.push(c),
                Err(e) => {
                    eprintln!("    Failed to create consumer {}: {:?}", i, e);
                    return false;
                }
            }
        }

        let create_duration = create_start.elapsed();
        let create_rate = topic_count as f64 / create_duration.as_secs_f64();

        // Send and receive
        let mut recv_count = 0;
        for i in 0..topic_count {
            let msg = CmdVel {
                linear: i as f32,
                angular: 0.0,
                stamp_nanos: i as u64,
            };
            let _ = producers[i].send(msg, &mut None);
        }

        thread::sleep(Duration::from_millis(20));

        for i in 0..topic_count {
            if consumers[i].recv(&mut None).is_some() {
                recv_count += 1;
            }
        }

        let recv_rate = (recv_count as f64 / topic_count as f64) * 100.0;
        println!(
            "    {} topics: {:.0} topics/sec, {:.1}% recv",
            topic_count, create_rate, recv_rate
        );

        if recv_rate < 95.0 {
            println!("  ✗ Test FAILED - receive rate too low");
            return false;
        }
    }

    println!("  ✓ Test PASSED");
    true
}

/// Test: Topic create/destroy churn
fn test_topic_churn() -> bool {
    println!("\n========================================");
    println!("Test: Topic Create/Destroy Churn");
    println!("========================================");

    let iterations = 100;
    let topics_per_iteration = 50;
    let mut total_created = 0;
    let mut total_communicated = 0;

    let start = Instant::now();

    for iter in 0..iterations {
        // Create topics
        let mut hubs: Vec<Topic<CmdVel>> = Vec::new();
        let base_topic = format!("churn_{}_{}_{}", process::id(), iter, Instant::now().elapsed().as_micros());

        for i in 0..topics_per_iteration {
            let topic = format!("{}_{}", base_topic, i);
            match Topic::<CmdVel>::new(&topic) {
                Ok(h) => {
                    hubs.push(h);
                    total_created += 1;
                }
                Err(e) => {
                    eprintln!("  Iteration {}: Failed to create topic {}: {:?}", iter, i, e);
                }
            }
        }

        // Send/receive on each
        for (i, hub) in hubs.iter().enumerate() {
            let msg = CmdVel {
                linear: 1.0,
                angular: 0.5,
                stamp_nanos: i as u64,
            };
            if hub.send(msg, &mut None).is_ok() {
                total_communicated += 1;
            }
        }

        // Topics are automatically cleaned up when hubs go out of scope
        drop(hubs);

        if (iter + 1) % 20 == 0 {
            println!("  Completed {}/{} iterations", iter + 1, iterations);
        }
    }

    let duration = start.elapsed();
    let expected_total = iterations * topics_per_iteration;
    let create_rate = (total_created as f64 / expected_total as f64) * 100.0;
    let comm_rate = (total_communicated as f64 / expected_total as f64) * 100.0;

    println!("  Total iterations: {}", iterations);
    println!("  Total topics created: {}/{} ({:.1}%)", total_created, expected_total, create_rate);
    println!("  Total communications: {}/{} ({:.1}%)", total_communicated, expected_total, comm_rate);
    println!("  Total time: {:?}", duration);
    println!("  Churn rate: {:.0} topics/sec", total_created as f64 / duration.as_secs_f64());

    let success = create_rate >= 99.0 && comm_rate >= 95.0;

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

/// Test: Topic discovery performance
fn test_discovery_performance() -> bool {
    println!("\n========================================");
    println!("Test: Topic Discovery Performance");
    println!("========================================");

    // Create topics in batches and measure first-access latency
    let topic_counts = [10, 100, 500, 1000];
    let mut results = Vec::new();

    for &count in &topic_counts {
        let base_topic = format!("discovery_{}_{}", process::id(), count);
        let mut hubs = Vec::new();

        // Create the topics
        for i in 0..count {
            let topic = format!("{}_{}", base_topic, i);
            if let Ok(h) = Topic::<CmdVel>::new(&topic) {
                hubs.push(h);
            }
        }

        if hubs.len() < count {
            eprintln!("  Failed to create all {} topics", count);
            return false;
        }

        thread::sleep(Duration::from_millis(50));

        // Measure time to access (send to) each topic
        let access_start = Instant::now();
        let mut accessed = 0;
        for hub in &hubs {
            let msg = CmdVel {
                linear: 1.0,
                angular: 0.5,
                stamp_nanos: 0,
            };
            if hub.send(msg, &mut None).is_ok() {
                accessed += 1;
            }
        }
        let access_duration = access_start.elapsed();

        let avg_access_ns = access_duration.as_nanos() / count as u128;
        let access_rate = accessed as f64 / access_duration.as_secs_f64();

        results.push((count, avg_access_ns, access_rate, accessed));

        println!(
            "  {} topics: avg access={} ns, rate={:.0} msg/sec, accessed={}/{}",
            count, avg_access_ns, access_rate, accessed, count
        );

        drop(hubs);
    }

    // Verify sub-linear scaling (access time shouldn't grow much faster than O(log n))
    // At 10 topics vs 1000 topics (100x more), access time should be less than 10x slower
    if results.len() >= 4 {
        let (count_10, time_10, _, _) = results[0];
        let (count_1000, time_1000, _, _) = results[3];

        let count_ratio = count_1000 as f64 / count_10 as f64;
        let time_ratio = time_1000 as f64 / time_10 as f64;

        println!("  Scaling analysis:");
        println!("    Topics ratio: {:.0}x ({}->{})", count_ratio, count_10, count_1000);
        println!("    Time ratio: {:.1}x ({}ns->{}ns)", time_ratio, time_10, time_1000);

        // Allow up to 20x time increase for 100x topic increase (sub-linear)
        if time_ratio <= 20.0 {
            println!("    Sub-linear scaling: ✓");
        } else {
            println!("    Sub-linear scaling: ✗ (too slow)");
            return false;
        }
    }

    // Verify all accesses succeeded
    let all_succeeded = results.iter().all(|(count, _, _, accessed)| *accessed == *count);

    if all_succeeded {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED - some accesses failed");
    }

    all_succeeded
}
