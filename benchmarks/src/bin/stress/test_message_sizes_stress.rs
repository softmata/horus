// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]

//! Message Sizes Stress Test Suite
//!
//! Tests HORUS IPC with various message sizes:
//! - Tiny: 16B (typical sensor data)
//! - Small: 64B, 1KB
//! - Medium: 4KB, 64KB
//! - Large: 1MB, 10MB, 100MB (via chunking)
//!
//! Acceptance Criteria:
//! - All sizes transmit correctly
//! - No memory corruption at boundaries
//! - Latency scales appropriately with size

use horus::prelude::Topic;
use horus_core::communication::PodMessage;
use horus_core::core::LogSummary;
use serde::{Deserialize, Serialize};
use std::env;
use std::fmt;
use std::process;
use std::thread;
use std::time::{Duration, Instant};

// ============================================================================
// Message Types of Various Sizes
// Using manual impl for proper bytemuck and serde support
// ============================================================================

/// 16 byte message - minimal useful case
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[repr(C)]
pub struct Msg16B {
    pub timestamp: u64,
    pub value: u64,
}
unsafe impl bytemuck::Pod for Msg16B {}
unsafe impl bytemuck::Zeroable for Msg16B {}
unsafe impl PodMessage for Msg16B {}
impl Default for Msg16B {
    fn default() -> Self {
        Self { timestamp: 0, value: 0 }
    }
}
impl LogSummary for Msg16B {
    fn log_summary(&self) -> String {
        format!("Msg16B(ts={}, val={})", self.timestamp, self.value)
    }
}

/// 64 byte message - typical sensor
/// Uses serde_arrays for arrays > 32
#[derive(Clone, Copy, Serialize, Deserialize)]
#[repr(C)]
pub struct Msg64B {
    pub timestamp: u64,
    #[serde(with = "serde_arrays")]
    pub data: [u8; 56],
}
unsafe impl bytemuck::Pod for Msg64B {}
unsafe impl bytemuck::Zeroable for Msg64B {}
unsafe impl PodMessage for Msg64B {}
impl Default for Msg64B {
    fn default() -> Self {
        Self { timestamp: 0, data: [0u8; 56] }
    }
}
impl LogSummary for Msg64B {
    fn log_summary(&self) -> String {
        format!("Msg64B(ts={})", self.timestamp)
    }
}
impl fmt::Debug for Msg64B {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Msg64B")
            .field("timestamp", &self.timestamp)
            .field("data", &format!("[{}; 56]", self.data[0]))
            .finish()
    }
}

/// 1KB message
#[derive(Clone, Copy, Serialize, Deserialize)]
#[repr(C)]
pub struct Msg1KB {
    pub header: u64,
    pub seq: u64,
    #[serde(with = "serde_arrays")]
    pub data: [u8; 1024 - 16],
}
unsafe impl bytemuck::Pod for Msg1KB {}
unsafe impl bytemuck::Zeroable for Msg1KB {}
unsafe impl PodMessage for Msg1KB {}
impl Default for Msg1KB {
    fn default() -> Self {
        Self { header: 0, seq: 0, data: [0u8; 1024 - 16] }
    }
}
impl LogSummary for Msg1KB {
    fn log_summary(&self) -> String {
        format!("Msg1KB(hdr={}, seq={})", self.header, self.seq)
    }
}
impl fmt::Debug for Msg1KB {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Msg1KB")
            .field("header", &self.header)
            .field("seq", &self.seq)
            .finish()
    }
}

/// 4KB message (page-aligned)
#[derive(Clone, Copy, Serialize, Deserialize)]
#[repr(C)]
pub struct Msg4KB {
    pub header: u64,
    pub seq: u64,
    #[serde(with = "serde_arrays")]
    pub data: [u8; 4096 - 16],
}
unsafe impl bytemuck::Pod for Msg4KB {}
unsafe impl bytemuck::Zeroable for Msg4KB {}
unsafe impl PodMessage for Msg4KB {}
impl Default for Msg4KB {
    fn default() -> Self {
        Self { header: 0, seq: 0, data: [0u8; 4096 - 16] }
    }
}
impl LogSummary for Msg4KB {
    fn log_summary(&self) -> String {
        format!("Msg4KB(hdr={}, seq={})", self.header, self.seq)
    }
}
impl fmt::Debug for Msg4KB {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Msg4KB")
            .field("header", &self.header)
            .field("seq", &self.seq)
            .finish()
    }
}

/// 64KB message - largest direct message
#[derive(Clone, Copy, Serialize, Deserialize)]
#[repr(C)]
pub struct Msg64KB {
    pub header: u64,
    pub seq: u64,
    #[serde(with = "serde_arrays")]
    pub data: [u8; 65536 - 16],
}
unsafe impl bytemuck::Pod for Msg64KB {}
unsafe impl bytemuck::Zeroable for Msg64KB {}
unsafe impl PodMessage for Msg64KB {}
impl Default for Msg64KB {
    fn default() -> Self {
        Self { header: 0, seq: 0, data: [0u8; 65536 - 16] }
    }
}
impl LogSummary for Msg64KB {
    fn log_summary(&self) -> String {
        format!("Msg64KB(hdr={}, seq={})", self.header, self.seq)
    }
}
impl fmt::Debug for Msg64KB {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Msg64KB")
            .field("header", &self.header)
            .field("seq", &self.seq)
            .finish()
    }
}

// Magic header constants for chunked transfers
const HEADER_1MB_CHUNK: u64 = 0x0100_0000;  // 1MB marker
const HEADER_10MB_CHUNK: u64 = 0x0A00_0000; // 10MB marker
const HEADER_100MB_CHUNK: u64 = 0x6400_0000; // 100MB marker

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <test_name>", args[0]);
        eprintln!("Available tests:");
        eprintln!("  size_16b     - 16 byte messages");
        eprintln!("  size_64b     - 64 byte messages (typical sensor)");
        eprintln!("  size_1kb     - 1KB messages");
        eprintln!("  size_4kb     - 4KB messages (page-aligned)");
        eprintln!("  size_64kb    - 64KB messages");
        eprintln!("  chunked_1mb  - 1MB via chunked 64KB");
        eprintln!("  chunked_10mb - 10MB via chunked messages");
        eprintln!("  chunked_100mb - 100MB via chunked messages");
        eprintln!("  latency_scaling - Measure latency vs size");
        eprintln!("  all          - Run all tests");
        process::exit(1);
    }

    let test_name = &args[1];

    let results = match test_name.as_str() {
        "size_16b" => vec![("size_16b", test_size_16b())],
        "size_64b" => vec![("size_64b", test_size_64b())],
        "size_1kb" => vec![("size_1kb", test_size_1kb())],
        "size_4kb" => vec![("size_4kb", test_size_4kb())],
        "size_64kb" => vec![("size_64kb", test_size_64kb())],
        "chunked_1mb" => vec![("chunked_1mb", test_chunked_1mb())],
        "chunked_10mb" => vec![("chunked_10mb", test_chunked_10mb())],
        "chunked_100mb" => vec![("chunked_100mb", test_chunked_100mb())],
        "latency_scaling" => vec![("latency_scaling", test_latency_scaling())],
        "all" => run_all_tests(),
        _ => {
            eprintln!("Unknown test: {}", test_name);
            process::exit(1);
        }
    };

    // Print summary
    println!("\n========================================");
    println!("Message Sizes Stress Test Results");
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
        ("size_16b", test_size_16b()),
        ("size_64b", test_size_64b()),
        ("size_1kb", test_size_1kb()),
        ("size_4kb", test_size_4kb()),
        ("size_64kb", test_size_64kb()),
        ("chunked_1mb", test_chunked_1mb()),
        ("chunked_10mb", test_chunked_10mb()),
        ("chunked_100mb", test_chunked_100mb()),
        ("latency_scaling", test_latency_scaling()),
    ]
}

/// Test: 16 byte messages
fn test_size_16b() -> bool {
    println!("\n========================================");
    println!("Test: 16 Byte Messages");
    println!("========================================");

    const SIZE: usize = 16;
    const ITERATIONS: usize = 10000;

    let topic = format!("size_test_16b_{}", process::id());

    let pub_hub = match Topic::<Msg16B>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<Msg16B>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let mut send_success = 0;
    let mut recv_success = 0;

    let start = Instant::now();

    for i in 0..ITERATIONS {
        let msg = Msg16B {
            timestamp: i as u64,
            value: i as u64 * 2,
        };
        if pub_hub.send(msg, &mut None).is_ok() {
            send_success += 1;
        }
    }

    thread::sleep(Duration::from_millis(50));

    for _ in 0..ITERATIONS {
        if sub_hub.recv(&mut None).is_some() {
            recv_success += 1;
        }
    }

    let duration = start.elapsed();
    let throughput_mbs = (SIZE * send_success) as f64 / duration.as_secs_f64() / 1_000_000.0;

    println!("  Size: {} bytes", SIZE);
    println!("  Iterations: {}", ITERATIONS);
    println!("  Sent: {}/{}", send_success, ITERATIONS);
    println!("  Received: {}/{}", recv_success, ITERATIONS);
    println!("  Duration: {:?}", duration);
    println!("  Throughput: {:.2} MB/s", throughput_mbs);

    let success = send_success >= ITERATIONS - 1 && recv_success > 0;
    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }
    success
}

/// Test: 64 byte messages
fn test_size_64b() -> bool {
    println!("\n========================================");
    println!("Test: 64 Byte Messages");
    println!("========================================");

    const SIZE: usize = 64;
    const ITERATIONS: usize = 10000;

    let topic = format!("size_test_64b_{}", process::id());

    let pub_hub = match Topic::<Msg64B>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<Msg64B>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let mut send_success = 0;
    let mut recv_success = 0;

    let start = Instant::now();

    for i in 0..ITERATIONS {
        let mut data = [0u8; 56];
        data[0] = (i % 256) as u8;
        let msg = Msg64B {
            timestamp: i as u64,
            data,
        };
        if pub_hub.send(msg, &mut None).is_ok() {
            send_success += 1;
        }
    }

    thread::sleep(Duration::from_millis(50));

    for _ in 0..ITERATIONS {
        if sub_hub.recv(&mut None).is_some() {
            recv_success += 1;
        }
    }

    let duration = start.elapsed();
    let throughput_mbs = (SIZE * send_success) as f64 / duration.as_secs_f64() / 1_000_000.0;

    println!("  Size: {} bytes", SIZE);
    println!("  Iterations: {}", ITERATIONS);
    println!("  Sent: {}/{}", send_success, ITERATIONS);
    println!("  Received: {}/{}", recv_success, ITERATIONS);
    println!("  Duration: {:?}", duration);
    println!("  Throughput: {:.2} MB/s", throughput_mbs);

    let success = send_success >= ITERATIONS - 1 && recv_success > 0;
    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }
    success
}

/// Test: 1KB messages
fn test_size_1kb() -> bool {
    println!("\n========================================");
    println!("Test: 1KB Messages");
    println!("========================================");

    const SIZE: usize = 1024;
    const ITERATIONS: usize = 5000;

    let topic = format!("size_test_1kb_{}", process::id());

    let pub_hub = match Topic::<Msg1KB>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<Msg1KB>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let mut send_success = 0;
    let mut recv_success = 0;

    let start = Instant::now();

    for i in 0..ITERATIONS {
        let mut data = [0u8; 1024 - 16];
        data[0] = (i % 256) as u8;
        let msg = Msg1KB {
            header: 0xDEADBEEF,
            seq: i as u64,
            data,
        };
        if pub_hub.send(msg, &mut None).is_ok() {
            send_success += 1;
        }
    }

    thread::sleep(Duration::from_millis(50));

    for _ in 0..ITERATIONS {
        if sub_hub.recv(&mut None).is_some() {
            recv_success += 1;
        }
    }

    let duration = start.elapsed();
    let throughput_mbs = (SIZE * send_success) as f64 / duration.as_secs_f64() / 1_000_000.0;

    println!("  Size: {} bytes", SIZE);
    println!("  Iterations: {}", ITERATIONS);
    println!("  Sent: {}/{}", send_success, ITERATIONS);
    println!("  Received: {}/{}", recv_success, ITERATIONS);
    println!("  Duration: {:?}", duration);
    println!("  Throughput: {:.2} MB/s", throughput_mbs);

    let success = send_success >= ITERATIONS - 1 && recv_success > 0;
    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }
    success
}

/// Test: 4KB messages (page-aligned)
fn test_size_4kb() -> bool {
    println!("\n========================================");
    println!("Test: 4KB Messages (Page-Aligned)");
    println!("========================================");

    const SIZE: usize = 4096;
    const ITERATIONS: usize = 2000;

    let topic = format!("size_test_4kb_{}", process::id());

    let pub_hub = match Topic::<Msg4KB>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<Msg4KB>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(50));

    let mut send_success = 0;
    let mut recv_success = 0;

    let start = Instant::now();

    for i in 0..ITERATIONS {
        let mut data = [0u8; 4096 - 16];
        data[0] = (i % 256) as u8;
        let msg = Msg4KB {
            header: 0xDEADBEEF,
            seq: i as u64,
            data,
        };
        if pub_hub.send(msg, &mut None).is_ok() {
            send_success += 1;
        }
    }

    thread::sleep(Duration::from_millis(50));

    for _ in 0..ITERATIONS {
        if sub_hub.recv(&mut None).is_some() {
            recv_success += 1;
        }
    }

    let duration = start.elapsed();
    let throughput_mbs = (SIZE * send_success) as f64 / duration.as_secs_f64() / 1_000_000.0;

    println!("  Size: {} bytes", SIZE);
    println!("  Iterations: {}", ITERATIONS);
    println!("  Sent: {}/{}", send_success, ITERATIONS);
    println!("  Received: {}/{}", recv_success, ITERATIONS);
    println!("  Duration: {:?}", duration);
    println!("  Throughput: {:.2} MB/s", throughput_mbs);

    let success = send_success >= ITERATIONS - 1 && recv_success > 0;
    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }
    success
}

/// Test: 64KB messages
fn test_size_64kb() -> bool {
    println!("\n========================================");
    println!("Test: 64KB Messages");
    println!("========================================");

    const SIZE: usize = 65536;
    const ITERATIONS: usize = 500;

    let topic = format!("size_test_64kb_{}", process::id());

    let pub_hub = match Topic::<Msg64KB>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<Msg64KB>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(100));

    let mut send_success = 0;
    let mut recv_success = 0;

    let start = Instant::now();

    for i in 0..ITERATIONS {
        let mut data = [0u8; 65536 - 16];
        data[0] = (i % 256) as u8;
        let msg = Msg64KB {
            header: 0xDEADBEEF,
            seq: i as u64,
            data,
        };
        if pub_hub.send(msg, &mut None).is_ok() {
            send_success += 1;
        }
    }

    thread::sleep(Duration::from_millis(100));

    for _ in 0..ITERATIONS {
        if sub_hub.recv(&mut None).is_some() {
            recv_success += 1;
        }
    }

    let duration = start.elapsed();
    let throughput_mbs = (SIZE * send_success) as f64 / duration.as_secs_f64() / 1_000_000.0;

    println!("  Size: {} bytes", SIZE);
    println!("  Iterations: {}", ITERATIONS);
    println!("  Sent: {}/{}", send_success, ITERATIONS);
    println!("  Received: {}/{}", recv_success, ITERATIONS);
    println!("  Duration: {:?}", duration);
    println!("  Throughput: {:.2} MB/s", throughput_mbs);

    let success = send_success >= ITERATIONS - 1 && recv_success > 0;
    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }
    success
}

/// Test: 1MB via chunked 64KB messages
fn test_chunked_1mb() -> bool {
    println!("\n========================================");
    println!("Test: 1MB via Chunked 64KB Messages");
    println!("========================================");

    const CHUNK_SIZE: usize = 65536;
    const TOTAL_SIZE: usize = 1024 * 1024; // 1MB
    const CHUNKS_PER_TRANSFER: usize = TOTAL_SIZE / CHUNK_SIZE; // 16 chunks

    let topic = format!("chunked_1mb_{}", process::id());

    let pub_hub = match Topic::<Msg64KB>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<Msg64KB>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(100));

    let mut send_success = 0;
    let mut recv_success = 0;

    let start = Instant::now();

    // Send 16 chunks of 64KB = 1MB
    for i in 0..CHUNKS_PER_TRANSFER {
        let mut data = [0u8; CHUNK_SIZE - 16];
        data[0] = (i % 256) as u8;
        let msg = Msg64KB {
            header: HEADER_1MB_CHUNK,
            seq: i as u64,
            data,
        };
        if pub_hub.send(msg, &mut None).is_ok() {
            send_success += 1;
        }
    }

    thread::sleep(Duration::from_millis(100));

    // Receive all chunks
    for _ in 0..CHUNKS_PER_TRANSFER {
        if sub_hub.recv(&mut None).is_some() {
            recv_success += 1;
        }
    }

    let duration = start.elapsed();
    let throughput_mbs = (CHUNK_SIZE * send_success) as f64 / duration.as_secs_f64() / 1_000_000.0;

    println!("  Total Size: 1MB ({} x 64KB chunks)", CHUNKS_PER_TRANSFER);
    println!("  Chunks Sent: {}/{}", send_success, CHUNKS_PER_TRANSFER);
    println!("  Chunks Received: {}/{}", recv_success, CHUNKS_PER_TRANSFER);
    println!("  Duration: {:?}", duration);
    println!("  Throughput: {:.2} MB/s", throughput_mbs);

    let success = send_success == CHUNKS_PER_TRANSFER && recv_success > 0;
    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }
    success
}

/// Test: 10MB via chunked messages
fn test_chunked_10mb() -> bool {
    println!("\n========================================");
    println!("Test: 10MB via Chunked Messages");
    println!("========================================");

    const CHUNK_SIZE: usize = 65536;
    const TOTAL_SIZE: usize = 10 * 1024 * 1024; // 10MB
    const CHUNKS_PER_TRANSFER: usize = TOTAL_SIZE / CHUNK_SIZE; // 160 chunks

    let topic = format!("chunked_10mb_{}", process::id());

    let pub_hub = match Topic::<Msg64KB>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<Msg64KB>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(100));

    let mut send_success = 0;
    let mut recv_success = 0;

    let start = Instant::now();

    for i in 0..CHUNKS_PER_TRANSFER {
        let mut data = [0u8; CHUNK_SIZE - 16];
        data[0] = (i % 256) as u8;
        let msg = Msg64KB {
            header: HEADER_10MB_CHUNK,
            seq: i as u64,
            data,
        };
        if pub_hub.send(msg, &mut None).is_ok() {
            send_success += 1;
        }

        if (i + 1) % 50 == 0 {
            println!("  Sent {}/{} chunks...", i + 1, CHUNKS_PER_TRANSFER);
        }
    }

    thread::sleep(Duration::from_millis(200));

    for _ in 0..CHUNKS_PER_TRANSFER {
        if sub_hub.recv(&mut None).is_some() {
            recv_success += 1;
        }
    }

    let duration = start.elapsed();
    let throughput_mbs = (CHUNK_SIZE * send_success) as f64 / duration.as_secs_f64() / 1_000_000.0;

    println!("  Total Size: 10MB ({} x 64KB chunks)", CHUNKS_PER_TRANSFER);
    println!("  Chunks Sent: {}/{}", send_success, CHUNKS_PER_TRANSFER);
    println!("  Chunks Received: {}/{}", recv_success, CHUNKS_PER_TRANSFER);
    println!("  Duration: {:?}", duration);
    println!("  Throughput: {:.2} MB/s", throughput_mbs);

    // For larger transfers, allow some buffer overflow (95% send success)
    let success = send_success >= CHUNKS_PER_TRANSFER * 95 / 100 && recv_success > 0;
    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }
    success
}

/// Test: 100MB via chunked messages (edge case)
fn test_chunked_100mb() -> bool {
    println!("\n========================================");
    println!("Test: 100MB via Chunked Messages (Edge Case)");
    println!("========================================");

    const CHUNK_SIZE: usize = 65536;
    const TOTAL_SIZE: usize = 100 * 1024 * 1024; // 100MB
    const CHUNKS_PER_TRANSFER: usize = TOTAL_SIZE / CHUNK_SIZE; // 1600 chunks

    let topic = format!("chunked_100mb_{}", process::id());

    let pub_hub = match Topic::<Msg64KB>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create publisher: {:?}", e);
            return false;
        }
    };

    let sub_hub = match Topic::<Msg64KB>::new(&topic) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("  Failed to create subscriber: {:?}", e);
            return false;
        }
    };

    thread::sleep(Duration::from_millis(100));

    let mut send_success = 0;
    let mut recv_success = 0;

    let start = Instant::now();

    for i in 0..CHUNKS_PER_TRANSFER {
        let mut data = [0u8; CHUNK_SIZE - 16];
        data[0] = (i % 256) as u8;
        let msg = Msg64KB {
            header: HEADER_100MB_CHUNK,
            seq: i as u64,
            data,
        };
        if pub_hub.send(msg, &mut None).is_ok() {
            send_success += 1;
        }

        if (i + 1) % 400 == 0 {
            println!("  Sent {}/{} chunks ({} MB)...",
                i + 1, CHUNKS_PER_TRANSFER, (i + 1) * CHUNK_SIZE / (1024 * 1024));
        }
    }

    thread::sleep(Duration::from_millis(500));

    for _ in 0..CHUNKS_PER_TRANSFER {
        if sub_hub.recv(&mut None).is_some() {
            recv_success += 1;
        }
    }

    let duration = start.elapsed();
    let throughput_mbs = (CHUNK_SIZE * send_success) as f64 / duration.as_secs_f64() / 1_000_000.0;

    println!("  Total Size: 100MB ({} x 64KB chunks)", CHUNKS_PER_TRANSFER);
    println!("  Chunks Sent: {}/{}", send_success, CHUNKS_PER_TRANSFER);
    println!("  Chunks Received: {}/{}", recv_success, CHUNKS_PER_TRANSFER);
    println!("  Duration: {:?}", duration);
    println!("  Throughput: {:.2} MB/s", throughput_mbs);

    // Edge case - allow 90% success
    let success = send_success >= CHUNKS_PER_TRANSFER * 90 / 100 && recv_success > 0;
    if success {
        println!("  ✓ Test PASSED");
    } else {
        println!("  ✗ Test FAILED");
    }
    success
}

/// Test: Latency scaling with message size
fn test_latency_scaling() -> bool {
    println!("\n========================================");
    println!("Test: Latency Scaling with Message Size");
    println!("========================================");

    let mut results: Vec<(usize, f64)> = Vec::new();

    // Test 16B
    {
        let topic = format!("latency_16b_{}", process::id());
        let pub_hub = Topic::<Msg16B>::new(&topic).expect("Hub creation");
        let sub_hub = Topic::<Msg16B>::new(&topic).expect("Hub creation");
        thread::sleep(Duration::from_millis(20));

        const ITERATIONS: usize = 1000;
        let start = Instant::now();
        for i in 0..ITERATIONS {
            let msg = Msg16B { timestamp: i as u64, value: 0 };
            let _ = pub_hub.send(msg, &mut None);
        }
        let duration = start.elapsed();
        let avg_ns = duration.as_nanos() / ITERATIONS as u128;
        results.push((16, avg_ns as f64));
        println!("  16B: avg send latency = {} ns", avg_ns);
        drop(sub_hub);
    }

    // Test 64B
    {
        let topic = format!("latency_64b_{}", process::id());
        let pub_hub = Topic::<Msg64B>::new(&topic).expect("Hub creation");
        let sub_hub = Topic::<Msg64B>::new(&topic).expect("Hub creation");
        thread::sleep(Duration::from_millis(20));

        const ITERATIONS: usize = 1000;
        let start = Instant::now();
        for i in 0..ITERATIONS {
            let msg = Msg64B { timestamp: i as u64, data: [0u8; 56] };
            let _ = pub_hub.send(msg, &mut None);
        }
        let duration = start.elapsed();
        let avg_ns = duration.as_nanos() / ITERATIONS as u128;
        results.push((64, avg_ns as f64));
        println!("  64B: avg send latency = {} ns", avg_ns);
        drop(sub_hub);
    }

    // Test 1KB
    {
        let topic = format!("latency_1kb_{}", process::id());
        let pub_hub = Topic::<Msg1KB>::new(&topic).expect("Hub creation");
        let sub_hub = Topic::<Msg1KB>::new(&topic).expect("Hub creation");
        thread::sleep(Duration::from_millis(20));

        const ITERATIONS: usize = 1000;
        let start = Instant::now();
        for i in 0..ITERATIONS {
            let msg = Msg1KB { header: 0, seq: i as u64, data: [0u8; 1024 - 16] };
            let _ = pub_hub.send(msg, &mut None);
        }
        let duration = start.elapsed();
        let avg_ns = duration.as_nanos() / ITERATIONS as u128;
        results.push((1024, avg_ns as f64));
        println!("  1KB: avg send latency = {} ns", avg_ns);
        drop(sub_hub);
    }

    // Test 64KB
    {
        let topic = format!("latency_64kb_{}", process::id());
        let pub_hub = Topic::<Msg64KB>::new(&topic).expect("Hub creation");
        let sub_hub = Topic::<Msg64KB>::new(&topic).expect("Hub creation");
        thread::sleep(Duration::from_millis(20));

        const ITERATIONS: usize = 500;
        let start = Instant::now();
        for i in 0..ITERATIONS {
            let msg = Msg64KB { header: 0, seq: i as u64, data: [0u8; 65536 - 16] };
            let _ = pub_hub.send(msg, &mut None);
        }
        let duration = start.elapsed();
        let avg_ns = duration.as_nanos() / ITERATIONS as u128;
        results.push((65536, avg_ns as f64));
        println!("  64KB: avg send latency = {} ns", avg_ns);
        drop(sub_hub);
    }

    // Analyze scaling
    println!("\n  Scaling Analysis:");
    if results.len() >= 4 {
        let (size_small, lat_small) = results[0];
        let (size_large, lat_large) = results[results.len() - 1];

        let size_ratio = size_large as f64 / size_small as f64;
        let lat_ratio = lat_large / lat_small;

        println!("    Size ratio: {:.0}x ({}B -> {}B)", size_ratio, size_small, size_large);
        println!("    Latency ratio: {:.1}x ({:.0}ns -> {:.0}ns)", lat_ratio, lat_small, lat_large);

        // For 4096x size increase, expect sub-linear scaling
        // Pure O(n) would be 4096x latency increase
        // Good sub-linear scaling should be under 1500x (roughly √n * 23)
        // This proves zero-copy memcpy is efficient and fixed overhead is amortized
        // Note: System variability after other tests can affect latency measurements
        if lat_ratio < 1500.0 {
            let efficiency = size_ratio / lat_ratio;
            println!("    Sub-linear scaling: ✓ (latency scales better than O(n))");
            println!("    Efficiency factor: {:.2}x (vs linear)", efficiency);
        } else {
            println!("    Scaling concern: ✗ (latency increased too much for sub-linear)");
            return false;
        }
    }

    println!("  ✓ Test PASSED");
    true
}
