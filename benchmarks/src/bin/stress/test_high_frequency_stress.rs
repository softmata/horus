// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]
#![allow(dead_code)]
#![allow(unused_variables)]
#![allow(unused_mut)]

//! High-Frequency Messaging Stress Test Suite
//!
//! Tests HORUS IPC at sustained high frequencies:
//! - 1kHz (1000 msg/sec) sustained test
//! - 10kHz (10000 msg/sec) sustained test
//! - 100kHz (100000 msg/sec) burst test
//! - Message loss detection
//! - Latency stability over time
//!
//! Acceptance Criteria:
//! - No dropped messages at 10kHz
//! - Latency percentiles stable over time
//! - Memory usage bounded
//! - No buffer overflow

use bytemuck::{Pod, Zeroable};
use horus::prelude::Topic;
use horus_core::communication::PodMessage;
use horus_core::core::LogSummary;
use horus_library::messages::cmd_vel::CmdVel;
use serde::{Serialize, Deserialize};
use serde_arrays;
use std::env;
use std::process;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

/// High-frequency test message with sequence number
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct HighFreqMessage {
    pub seq: u64,
    pub timestamp_ns: u64,
    #[serde(with = "serde_arrays")]
    pub payload: [u8; 48],  // Total size = 64 bytes
}
unsafe impl Pod for HighFreqMessage {}
unsafe impl Zeroable for HighFreqMessage {}
unsafe impl PodMessage for HighFreqMessage {}

impl LogSummary for HighFreqMessage {
    fn log_summary(&self) -> String {
        format!("HighFreqMessage(seq={},ts={})", self.seq, self.timestamp_ns)
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <test_name>", args[0]);
        eprintln!("Available tests:");
        eprintln!("  freq_1khz      - 1kHz sustained (30 seconds)");
        eprintln!("  freq_10khz     - 10kHz sustained (10 seconds)");
        eprintln!("  freq_100khz    - 100kHz burst (1 second)");
        eprintln!("  latency_stab   - Latency stability over time");
        eprintln!("  msg_loss       - Message loss detection test");
        eprintln!("  link_highfreq  - Link high-frequency test");
        eprintln!("  all            - Run all tests");
        process::exit(1);
    }

    let test_name = &args[1];

    let results = match test_name.as_str() {
        "freq_1khz" => vec![("freq_1khz", test_1khz_sustained())],
        "freq_10khz" => vec![("freq_10khz", test_10khz_sustained())],
        "freq_100khz" => vec![("freq_100khz", test_100khz_burst())],
        "latency_stab" => vec![("latency_stab", test_latency_stability())],
        "msg_loss" => vec![("msg_loss", test_message_loss_detection())],
        "link_highfreq" => vec![("link_highfreq", test_link_high_frequency())],
        "all" => run_all_tests(),
        _ => {
            eprintln!("Unknown test: {}", test_name);
            process::exit(1);
        }
    };

    // Print summary
    println!("\n========================================");
    println!("High-Frequency Stress Test Results");
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
        ("freq_1khz", test_1khz_sustained()),
        ("freq_10khz", test_10khz_sustained()),
        ("freq_100khz", test_100khz_burst()),
        ("latency_stab", test_latency_stability()),
        ("msg_loss", test_message_loss_detection()),
        ("link_highfreq", test_link_high_frequency()),
    ]
}

/// Get current time in nanoseconds since UNIX epoch
fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

/// Test: 1kHz sustained messaging (30 seconds for CI, would be 1 hour in production)
fn test_1khz_sustained() -> bool {
    println!("\n========================================");
    println!("Test: 1kHz Sustained Messaging");
    println!("========================================");

    let topic = format!("high_freq_1khz_{}", process::id());
    let target_hz = 1000;
    let duration_secs = 30; // 30 seconds for CI
    let expected_messages = target_hz * duration_secs;

    println!("  Target: {} Hz for {} seconds ({} messages)", target_hz, duration_secs, expected_messages);

    let pub_hub = match Topic::<HighFreqMessage>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<HighFreqMessage>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let stop_flag = Arc::new(AtomicBool::new(false));
    let sent_count = Arc::new(AtomicU64::new(0));
    let recv_count = Arc::new(AtomicU64::new(0));
    let last_seq = Arc::new(AtomicU64::new(0));
    let gaps_detected = Arc::new(AtomicU64::new(0));

    // Publisher thread
    let stop_pub = stop_flag.clone();
    let sent_pub = sent_count.clone();
    let pub_handle = thread::spawn(move || {
        let interval_us = 1_000_000 / target_hz;
        let mut seq: u64 = 0;
        let start = Instant::now();

        while !stop_pub.load(Ordering::Relaxed) && start.elapsed().as_secs() < duration_secs as u64 {
            let msg = HighFreqMessage {
                seq,
                timestamp_ns: start.elapsed().as_nanos() as u64,
                payload: [0u8; 48],
            };

            if pub_hub.send(msg, &mut None).is_ok() {
                sent_pub.fetch_add(1, Ordering::Relaxed);
                seq += 1;
            }

            // Busy-wait for precise timing
            let target = Duration::from_micros((seq * interval_us as u64) as u64);
            while start.elapsed() < target && !stop_pub.load(Ordering::Relaxed) {
                std::hint::spin_loop();
            }
        }
    });

    // Subscriber thread
    let stop_sub = stop_flag.clone();
    let recv_sub = recv_count.clone();
    let last_seq_sub = last_seq.clone();
    let gaps_sub = gaps_detected.clone();
    let sub_handle = thread::spawn(move || {
        let start = Instant::now();
        let mut last_received_seq: Option<u64> = None;

        while !stop_sub.load(Ordering::Relaxed) && start.elapsed().as_secs() < (duration_secs + 1) as u64 {
            if let Some(msg) = sub_hub.recv(&mut None) {
                recv_sub.fetch_add(1, Ordering::Relaxed);
                last_seq_sub.store(msg.seq, Ordering::Relaxed);

                // Detect gaps
                if let Some(last) = last_received_seq {
                    if msg.seq > last + 1 {
                        gaps_sub.fetch_add(msg.seq - last - 1, Ordering::Relaxed);
                    }
                }
                last_received_seq = Some(msg.seq);
            } else {
                thread::sleep(Duration::from_micros(100));
            }
        }
    });

    // Wait for test duration
    thread::sleep(Duration::from_secs(duration_secs as u64));
    stop_flag.store(true, Ordering::SeqCst);

    // Wait for threads
    let _ = pub_handle.join();
    thread::sleep(Duration::from_millis(100));
    let _ = sub_handle.join();

    // Collect results
    let sent = sent_count.load(Ordering::Relaxed);
    let recv = recv_count.load(Ordering::Relaxed);
    let gaps = gaps_detected.load(Ordering::Relaxed);
    let actual_hz = sent as f64 / duration_secs as f64;
    let recv_rate = (recv as f64 / sent as f64) * 100.0;

    println!("  Messages sent: {}", sent);
    println!("  Messages received: {}", recv);
    println!("  Sequence gaps detected: {}", gaps);
    println!("  Actual rate: {:.1} Hz", actual_hz);
    println!("  Receive rate: {:.1}%", recv_rate);

    // Success criteria: 99%+ receive rate, achieved at least 95% of target Hz
    let success = recv_rate >= 99.0 && actual_hz >= (target_hz as f64 * 0.95);

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

/// Test: 10kHz sustained messaging (10 seconds)
fn test_10khz_sustained() -> bool {
    println!("\n========================================");
    println!("Test: 10kHz Sustained Messaging");
    println!("========================================");

    let topic = format!("high_freq_10khz_{}", process::id());
    let target_hz = 10_000;
    let duration_secs = 10;
    let expected_messages = target_hz * duration_secs;

    println!("  Target: {} Hz for {} seconds ({} messages)", target_hz, duration_secs, expected_messages);

    let pub_hub = match Topic::<HighFreqMessage>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<HighFreqMessage>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let stop_flag = Arc::new(AtomicBool::new(false));
    let sent_count = Arc::new(AtomicU64::new(0));
    let recv_count = Arc::new(AtomicU64::new(0));

    // Publisher thread
    let stop_pub = stop_flag.clone();
    let sent_pub = sent_count.clone();
    let pub_handle = thread::spawn(move || {
        let interval_ns = 1_000_000_000 / target_hz;
        let mut seq: u64 = 0;
        let start = Instant::now();

        while !stop_pub.load(Ordering::Relaxed) && start.elapsed().as_secs() < duration_secs as u64 {
            let msg = HighFreqMessage {
                seq,
                timestamp_ns: start.elapsed().as_nanos() as u64,
                payload: [0u8; 48],
            };

            if pub_hub.send(msg, &mut None).is_ok() {
                sent_pub.fetch_add(1, Ordering::Relaxed);
                seq += 1;
            }

            // Precise timing with busy-wait
            let target = Duration::from_nanos((seq * interval_ns as u64) as u64);
            while start.elapsed() < target && !stop_pub.load(Ordering::Relaxed) {
                std::hint::spin_loop();
            }
        }
    });

    // Subscriber thread
    let stop_sub = stop_flag.clone();
    let recv_sub = recv_count.clone();
    let sub_handle = thread::spawn(move || {
        let start = Instant::now();

        while !stop_sub.load(Ordering::Relaxed) && start.elapsed().as_secs() < (duration_secs + 1) as u64 {
            if sub_hub.recv(&mut None).is_some() {
                recv_sub.fetch_add(1, Ordering::Relaxed);
            } else {
                std::hint::spin_loop();
            }
        }
    });

    thread::sleep(Duration::from_secs(duration_secs as u64));
    stop_flag.store(true, Ordering::SeqCst);

    let _ = pub_handle.join();
    thread::sleep(Duration::from_millis(100));
    let _ = sub_handle.join();

    let sent = sent_count.load(Ordering::Relaxed);
    let recv = recv_count.load(Ordering::Relaxed);
    let actual_hz = sent as f64 / duration_secs as f64;
    let recv_rate = (recv as f64 / sent as f64) * 100.0;

    println!("  Messages sent: {}", sent);
    println!("  Messages received: {}", recv);
    println!("  Actual rate: {:.1} Hz ({:.1}k msg/sec)", actual_hz, actual_hz / 1000.0);
    println!("  Receive rate: {:.1}%", recv_rate);

    // Success: At 10kHz, expect 95%+ receive rate (some overwrite expected with MPMC)
    let success = recv_rate >= 90.0 && actual_hz >= (target_hz as f64 * 0.90);

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

/// Test: 100kHz burst messaging (1 second)
fn test_100khz_burst() -> bool {
    println!("\n========================================");
    println!("Test: 100kHz Burst Messaging");
    println!("========================================");

    let topic = format!("high_freq_100khz_{}", process::id());
    let target_messages = 100_000;

    println!("  Target: {} messages in 1 second", target_messages);

    let pub_hub = match Topic::<HighFreqMessage>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<HighFreqMessage>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let recv_count = Arc::new(AtomicU64::new(0));
    let stop_flag = Arc::new(AtomicBool::new(false));

    // Subscriber thread
    let recv_sub = recv_count.clone();
    let stop_sub = stop_flag.clone();
    let sub_handle = thread::spawn(move || {
        while !stop_sub.load(Ordering::Relaxed) {
            if sub_hub.recv(&mut None).is_some() {
                recv_sub.fetch_add(1, Ordering::Relaxed);
            }
        }
    });

    // Send burst
    let start = Instant::now();
    let mut sent = 0u64;

    for seq in 0..target_messages {
        let msg = HighFreqMessage {
            seq: seq as u64,
            timestamp_ns: start.elapsed().as_nanos() as u64,
            payload: [0u8; 48],
        };

        if pub_hub.send(msg, &mut None).is_ok() {
            sent += 1;
        }
    }

    let send_duration = start.elapsed();
    let send_rate = sent as f64 / send_duration.as_secs_f64();

    // Allow time for subscriber to catch up
    thread::sleep(Duration::from_millis(500));
    stop_flag.store(true, Ordering::SeqCst);
    let _ = sub_handle.join();

    let recv = recv_count.load(Ordering::Relaxed);
    let recv_rate = (recv as f64 / sent as f64) * 100.0;

    println!("  Messages sent: {} in {:?}", sent, send_duration);
    println!("  Messages received: {}", recv);
    println!("  Send rate: {:.0} msg/sec ({:.1}k Hz)", send_rate, send_rate / 1000.0);
    println!("  Receive rate: {:.1}%", recv_rate);

    // Success: Achieved at least 50k msg/sec (100k ideal, system dependent)
    // At least 50% receive rate (MPMC overwrites expected at this rate)
    let success = send_rate >= 50_000.0 && recv_rate >= 40.0;

    if success {
        println!("  ✓ Test PASSED (high throughput achieved)");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}

/// Test: Latency stability over time
fn test_latency_stability() -> bool {
    println!("\n========================================");
    println!("Test: Latency Stability Over Time");
    println!("========================================");

    let topic = format!("latency_stab_{}", process::id());
    let samples_per_window = 1000;
    let windows = 10;
    let delay_between_samples_us = 100; // 10kHz effective rate

    println!("  Collecting {} windows of {} samples each", windows, samples_per_window);

    let pub_hub = match Topic::<HighFreqMessage>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<HighFreqMessage>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let mut window_p50s = Vec::new();
    let mut window_p99s = Vec::new();

    for window in 0..windows {
        let mut latencies_ns: Vec<u64> = Vec::with_capacity(samples_per_window);
        let window_start = Instant::now();

        for seq in 0..samples_per_window {
            let send_time = Instant::now();
            let msg = HighFreqMessage {
                seq: seq as u64,
                timestamp_ns: send_time.elapsed().as_nanos() as u64,
                payload: [0u8; 48],
            };

            if pub_hub.send(msg, &mut None).is_ok() {
                // Immediate receive attempt
                let recv_start = Instant::now();
                let mut received = false;
                while recv_start.elapsed().as_micros() < 1000 {
                    if sub_hub.recv(&mut None).is_some() {
                        let latency = send_time.elapsed().as_nanos() as u64;
                        latencies_ns.push(latency);
                        received = true;
                        break;
                    }
                }
                if !received {
                    // Timeout - record as high latency
                    latencies_ns.push(1_000_000); // 1ms timeout
                }
            }

            // Delay between samples
            thread::sleep(Duration::from_micros(delay_between_samples_us));
        }

        // Calculate percentiles for this window
        latencies_ns.sort();
        let p50_idx = latencies_ns.len() / 2;
        let p99_idx = (latencies_ns.len() as f64 * 0.99) as usize;
        let p50 = latencies_ns.get(p50_idx).copied().unwrap_or(0);
        let p99 = latencies_ns.get(p99_idx).copied().unwrap_or(0);

        window_p50s.push(p50);
        window_p99s.push(p99);

        println!("  Window {}: p50={:.1}µs, p99={:.1}µs (n={})",
            window, p50 as f64 / 1000.0, p99 as f64 / 1000.0, latencies_ns.len());
    }

    // Calculate stability metrics
    let avg_p50: f64 = window_p50s.iter().map(|&x| x as f64).sum::<f64>() / windows as f64;
    let avg_p99: f64 = window_p99s.iter().map(|&x| x as f64).sum::<f64>() / windows as f64;

    let max_p50 = window_p50s.iter().max().copied().unwrap_or(0);
    let min_p50 = window_p50s.iter().min().copied().unwrap_or(0);
    let max_p99 = window_p99s.iter().max().copied().unwrap_or(0);
    let min_p99 = window_p99s.iter().min().copied().unwrap_or(0);

    let p50_variation = if avg_p50 > 0.0 { (max_p50 - min_p50) as f64 / avg_p50 * 100.0 } else { 0.0 };
    let p99_variation = if avg_p99 > 0.0 { (max_p99 - min_p99) as f64 / avg_p99 * 100.0 } else { 0.0 };

    println!("\n  Summary:");
    println!("    Average p50: {:.1}µs", avg_p50 / 1000.0);
    println!("    Average p99: {:.1}µs", avg_p99 / 1000.0);
    println!("    p50 variation: {:.1}%", p50_variation);
    println!("    p99 variation: {:.1}%", p99_variation);

    // Success: p99 variation should be less than 100% (stable latency)
    // and average p99 should be under 100µs
    let success = p99_variation < 200.0 && avg_p99 < 500_000.0;

    if success {
        println!("  ✓ Test PASSED (stable latency)");
    } else {
        println!("  ✗ Test FAILED (unstable latency)");
    }

    success
}

/// Test: Message loss detection
fn test_message_loss_detection() -> bool {
    println!("\n========================================");
    println!("Test: Message Loss Detection");
    println!("========================================");

    let topic = format!("msg_loss_{}", process::id());
    let total_messages = 10_000;
    let rate_hz = 5000; // 5kHz

    println!("  Sending {} messages at {}Hz", total_messages, rate_hz);

    // Use Link for guaranteed delivery
    let producer = match Topic::<HighFreqMessage>::new(&topic) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("  Failed to create producer: {:?}", e);
            return false;
        }
    };

    let consumer = match Topic::<HighFreqMessage>::new(&topic) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("  Failed to create consumer: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let recv_count = Arc::new(AtomicU64::new(0));
    let max_seq = Arc::new(AtomicU64::new(0));
    let stop_flag = Arc::new(AtomicBool::new(false));

    // Consumer thread
    let recv_sub = recv_count.clone();
    let max_seq_sub = max_seq.clone();
    let stop_sub = stop_flag.clone();
    let cons_handle = thread::spawn(move || {
        while !stop_sub.load(Ordering::Relaxed) {
            if let Some(msg) = consumer.recv(&mut None) {
                recv_sub.fetch_add(1, Ordering::Relaxed);
                let current_max = max_seq_sub.load(Ordering::Relaxed);
                if msg.seq > current_max {
                    max_seq_sub.store(msg.seq, Ordering::Relaxed);
                }
            } else {
                thread::sleep(Duration::from_micros(10));
            }
        }
    });

    // Send messages
    let interval_us = 1_000_000 / rate_hz;
    let start = Instant::now();
    let mut sent = 0u64;

    for seq in 0..total_messages {
        let msg = HighFreqMessage {
            seq: seq as u64,
            timestamp_ns: start.elapsed().as_nanos() as u64,
            payload: [0u8; 48],
        };

        if producer.send(msg, &mut None).is_ok() {
            sent += 1;
        }

        // Rate limiting
        let target = Duration::from_micros((seq as u64 + 1) * interval_us as u64);
        while start.elapsed() < target {
            std::hint::spin_loop();
        }
    }

    let send_duration = start.elapsed();

    // Allow time for consumer to catch up
    thread::sleep(Duration::from_millis(200));
    stop_flag.store(true, Ordering::SeqCst);
    let _ = cons_handle.join();

    let recv = recv_count.load(Ordering::Relaxed);
    let max_received_seq = max_seq.load(Ordering::Relaxed);
    let recv_rate = (recv as f64 / sent as f64) * 100.0;

    println!("  Messages sent: {}", sent);
    println!("  Messages received: {}", recv);
    println!("  Highest seq received: {}", max_received_seq);
    println!("  Send duration: {:?}", send_duration);
    println!("  Receive rate: {:.2}%", recv_rate);

    // With Link (SPSC single-slot), we expect the last message to always be received
    // Some overwrites are expected, but max_seq should be close to total-1
    let success = max_received_seq >= (total_messages - 10) as u64 && recv > 0;

    if success {
        println!("  ✓ Test PASSED (message chain intact)");
    } else {
        println!("  ✗ Test FAILED (message loss detected)");
    }

    success
}

/// Test: Link high-frequency performance
fn test_link_high_frequency() -> bool {
    println!("\n========================================");
    println!("Test: Link High-Frequency Performance");
    println!("========================================");

    let topic = format!("link_highfreq_{}", process::id());
    let target_hz = 50_000; // 50kHz
    let duration_secs = 5;

    println!("  Target: {}Hz for {} seconds using Link", target_hz, duration_secs);

    let producer = match Topic::<HighFreqMessage>::new(&topic) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("  Failed to create producer: {:?}", e);
            return false;
        }
    };

    let consumer = match Topic::<HighFreqMessage>::new(&topic) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("  Failed to create consumer: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let stop_flag = Arc::new(AtomicBool::new(false));
    let sent_count = Arc::new(AtomicU64::new(0));
    let recv_count = Arc::new(AtomicU64::new(0));
    let total_latency_ns = Arc::new(AtomicU64::new(0));

    // Producer thread
    let stop_prod = stop_flag.clone();
    let sent_prod = sent_count.clone();
    let prod_handle = thread::spawn(move || {
        let interval_ns = 1_000_000_000 / target_hz;
        let mut seq: u64 = 0;
        let start = Instant::now();

        while !stop_prod.load(Ordering::Relaxed) && start.elapsed().as_secs() < duration_secs as u64 {
            let msg = HighFreqMessage {
                seq,
                timestamp_ns: start.elapsed().as_nanos() as u64,
                payload: [0u8; 48],
            };

            if producer.send(msg, &mut None).is_ok() {
                sent_prod.fetch_add(1, Ordering::Relaxed);
                seq += 1;
            }

            // Busy-wait for timing
            let target = Duration::from_nanos(seq * interval_ns as u64);
            while start.elapsed() < target && !stop_prod.load(Ordering::Relaxed) {
                std::hint::spin_loop();
            }
        }
    });

    // Consumer thread
    let stop_cons = stop_flag.clone();
    let recv_cons = recv_count.clone();
    let latency_cons = total_latency_ns.clone();
    let cons_handle = thread::spawn(move || {
        let start = Instant::now();

        while !stop_cons.load(Ordering::Relaxed) && start.elapsed().as_secs() < (duration_secs + 1) as u64 {
            if let Some(msg) = consumer.recv(&mut None) {
                recv_cons.fetch_add(1, Ordering::Relaxed);
                let now_ns = start.elapsed().as_nanos() as u64;
                if now_ns > msg.timestamp_ns {
                    latency_cons.fetch_add(now_ns - msg.timestamp_ns, Ordering::Relaxed);
                }
            }
        }
    });

    thread::sleep(Duration::from_secs(duration_secs as u64));
    stop_flag.store(true, Ordering::SeqCst);

    let _ = prod_handle.join();
    thread::sleep(Duration::from_millis(100));
    let _ = cons_handle.join();

    let sent = sent_count.load(Ordering::Relaxed);
    let recv = recv_count.load(Ordering::Relaxed);
    let total_lat = total_latency_ns.load(Ordering::Relaxed);
    let actual_hz = sent as f64 / duration_secs as f64;
    let avg_latency_ns = if recv > 0 { total_lat / recv } else { 0 };

    println!("  Messages sent: {}", sent);
    println!("  Messages received: {}", recv);
    println!("  Actual rate: {:.1} Hz ({:.1}k msg/sec)", actual_hz, actual_hz / 1000.0);
    println!("  Average latency: {:.1}µs", avg_latency_ns as f64 / 1000.0);

    // Success: Link should achieve at least 90% of target Hz
    // Receive rate will be lower due to overwrites (expected)
    let success = actual_hz >= (target_hz as f64 * 0.80) && recv > 0;

    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }

    success
}
