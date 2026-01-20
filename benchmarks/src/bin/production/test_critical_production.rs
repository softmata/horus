// Benchmark binary - allow clippy warnings
#![allow(unused_imports)]
#![allow(unused_assignments)]
#![allow(unreachable_patterns)]
#![allow(clippy::all)]
#![allow(deprecated)]

/// Critical Test Suite for Single-Slot Link Implementation
///
/// This comprehensive test suite validates three critical areas:
/// 1. Big Data Types - Large messages (64B to 16KB) for camera/LiDAR data
/// 2. Fast Storing - High-throughput scenarios (1000Hz sustained, burst tests)
/// 3. Fast Tracking - Real-time telemetry and monitoring with low overhead
///
/// Run with: cargo run --release --bin test_critical_production <test_name>
/// Or run all: cargo run --release --bin test_critical_production all
use horus::prelude::Topic;
use serde::{Deserialize, Serialize};
use serde_arrays;
use std::env;
use std::process;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

// ============================================================================
// Realistic Robotics Data Types for Testing
// ============================================================================

/// Small message: 64 bytes
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct SmallMessage {
    timestamp: u64,
    sequence: u64,
    sensor_id: u32,
    checksum: u32,
    data: [f32; 10],
}

impl SmallMessage {
    fn new(sequence: u64) -> Self {
        let data = [sequence as f32; 10];
        let checksum = Self::compute_checksum(sequence, &data);
        Self {
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            sequence,
            sensor_id: 42,
            checksum,
            data,
        }
    }

    fn compute_checksum(sequence: u64, data: &[f32; 10]) -> u32 {
        let mut sum = sequence as u32;
        for &val in data {
            sum = sum.wrapping_add(val.to_bits());
        }
        sum
    }

    fn verify(&self) -> bool {
        let expected = Self::compute_checksum(self.sequence, &self.data);
        self.checksum == expected
    }
}

impl horus::core::LogSummary for SmallMessage {
    fn log_summary(&self) -> String {
        format!("SmallMsg(seq:{}, id:{})", self.sequence, self.sensor_id)
    }
}

/// Medium message: 256 bytes (camera metadata)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct CameraMetadata {
    timestamp: u64,
    sequence: u64,
    frame_id: u32,
    width: u32,
    height: u32,
    encoding: [u8; 16],
    checksum: u32,
    #[serde(with = "serde_arrays")]
    padding: [u8; 200],
}

impl CameraMetadata {
    fn new(sequence: u64) -> Self {
        let mut encoding = [0u8; 16];
        encoding[..4].copy_from_slice(b"RGB8");
        let padding = [sequence as u8; 200];
        let checksum = Self::compute_checksum(sequence, &padding);
        Self {
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            sequence,
            frame_id: sequence as u32,
            width: 1920,
            height: 1080,
            encoding,
            checksum,
            padding,
        }
    }

    fn compute_checksum(sequence: u64, padding: &[u8; 200]) -> u32 {
        let mut sum = sequence as u32;
        for &byte in padding {
            sum = sum.wrapping_add(byte as u32);
        }
        sum
    }

    fn verify(&self) -> bool {
        let expected = Self::compute_checksum(self.sequence, &self.padding);
        self.checksum == expected
    }
}

impl horus::core::LogSummary for CameraMetadata {
    fn log_summary(&self) -> String {
        format!(
            "CameraMeta(frame:{}, {}x{})",
            self.frame_id, self.width, self.height
        )
    }
}

/// Large message: 1KB (robot state)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct LargeState {
    timestamp: u64,
    sequence: u64,
    joint_positions: [f64; 32],
    joint_velocities: [f64; 32],
    joint_efforts: [f64; 32],
    #[serde(with = "serde_arrays")]
    sensor_readings: [f32; 64],
    checksum: u64,
}

impl LargeState {
    fn new(sequence: u64) -> Self {
        let joint_positions = [sequence as f64 * 0.1; 32];
        let joint_velocities = [sequence as f64 * 0.01; 32];
        let joint_efforts = [sequence as f64 * 0.001; 32];
        let sensor_readings = [sequence as f32 * 0.1; 64];
        let checksum = Self::compute_checksum(sequence, &joint_positions);
        Self {
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            sequence,
            joint_positions,
            joint_velocities,
            joint_efforts,
            sensor_readings,
            checksum,
        }
    }

    fn compute_checksum(sequence: u64, positions: &[f64; 32]) -> u64 {
        let mut sum = sequence;
        for &pos in positions {
            sum = sum.wrapping_add(pos.to_bits());
        }
        sum
    }

    fn verify(&self) -> bool {
        let expected = Self::compute_checksum(self.sequence, &self.joint_positions);
        self.checksum == expected
    }
}

impl horus::core::LogSummary for LargeState {
    fn log_summary(&self) -> String {
        format!("LargeState(seq:{}, joints:32)", self.sequence)
    }
}

/// Very large message: 4KB (image patch / small point cloud)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct ImagePatch {
    timestamp: u64,
    sequence: u64,
    width: u32,
    height: u32,
    #[serde(with = "serde_arrays")]
    data: [u8; 4000],
    checksum: u64,
}

impl ImagePatch {
    fn new(sequence: u64) -> Self {
        let mut data = [0u8; 4000];
        for (i, byte) in data.iter_mut().enumerate() {
            *byte = ((sequence + i as u64) % 256) as u8;
        }
        let checksum = Self::compute_checksum(sequence, &data);
        Self {
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            sequence,
            width: 64,
            height: 64,
            data,
            checksum,
        }
    }

    fn compute_checksum(sequence: u64, data: &[u8; 4000]) -> u64 {
        let mut sum = sequence;
        for &byte in data {
            sum = sum.wrapping_add(byte as u64);
        }
        sum
    }

    fn verify(&self) -> bool {
        let expected = Self::compute_checksum(self.sequence, &self.data);
        self.checksum == expected
    }
}

impl horus::core::LogSummary for ImagePatch {
    fn log_summary(&self) -> String {
        format!(
            "ImagePatch(seq:{}, {}x{})",
            self.sequence, self.width, self.height
        )
    }
}

/// Extra large message: 16KB (LiDAR point cloud)
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct PointCloud {
    timestamp: u64,
    sequence: u64,
    point_count: u32,
    #[serde(with = "serde_arrays")]
    points: [[f32; 3]; 1365], // 1365 points * 12 bytes = ~16KB
    checksum: u64,
}

impl PointCloud {
    fn new(sequence: u64) -> Self {
        let mut points = [[0.0f32; 3]; 1365];
        for (i, point) in points.iter_mut().enumerate() {
            point[0] = (sequence as f32 + i as f32) * 0.01;
            point[1] = (sequence as f32 + i as f32) * 0.02;
            point[2] = (sequence as f32 + i as f32) * 0.03;
        }
        let checksum = Self::compute_checksum(sequence, &points);
        Self {
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            sequence,
            point_count: 1365,
            points,
            checksum,
        }
    }

    fn compute_checksum(sequence: u64, points: &[[f32; 3]; 1365]) -> u64 {
        let mut sum = sequence;
        for point in points {
            sum = sum.wrapping_add(point[0].to_bits() as u64);
            sum = sum.wrapping_add(point[1].to_bits() as u64);
            sum = sum.wrapping_add(point[2].to_bits() as u64);
        }
        sum
    }

    fn verify(&self) -> bool {
        let expected = Self::compute_checksum(self.sequence, &self.points);
        self.checksum == expected
    }
}

impl horus::core::LogSummary for PointCloud {
    fn log_summary(&self) -> String {
        format!(
            "PointCloud(seq:{}, points:{})",
            self.sequence, self.point_count
        )
    }
}

/// Telemetry data for tracking performance
#[derive(Debug, Clone)]
struct TelemetryData {
    message_count: u64,
    total_latency_ns: u64,
    min_latency_ns: u64,
    max_latency_ns: u64,
    corruption_count: u64,
    out_of_order_count: u64,
}

impl Default for TelemetryData {
    fn default() -> Self {
        Self {
            message_count: 0,
            total_latency_ns: 0,
            min_latency_ns: u64::MAX,
            max_latency_ns: 0,
            corruption_count: 0,
            out_of_order_count: 0,
        }
    }
}

impl TelemetryData {
    fn record_latency(&mut self, latency_ns: u64) {
        self.message_count += 1;
        self.total_latency_ns += latency_ns;
        self.min_latency_ns = self.min_latency_ns.min(latency_ns);
        self.max_latency_ns = self.max_latency_ns.max(latency_ns);
    }

    fn avg_latency_ns(&self) -> u64 {
        if self.message_count > 0 {
            self.total_latency_ns / self.message_count
        } else {
            0
        }
    }

    fn avg_latency_us(&self) -> f64 {
        self.avg_latency_ns() as f64 / 1000.0
    }

    fn min_latency_us(&self) -> f64 {
        if self.min_latency_ns == u64::MAX {
            0.0
        } else {
            self.min_latency_ns as f64 / 1000.0
        }
    }

    fn max_latency_us(&self) -> f64 {
        self.max_latency_ns as f64 / 1000.0
    }

    fn report(&self, test_name: &str) {
        println!("\n  Telemetry Report for {}:", test_name);
        println!("    Messages: {}", self.message_count);
        println!("    Avg Latency: {:.2} µs", self.avg_latency_us());
        println!("    Min Latency: {:.2} µs", self.min_latency_us());
        println!("    Max Latency: {:.2} µs", self.max_latency_us());
        println!("    Corruptions: {}", self.corruption_count);
        println!("    Out of Order: {}", self.out_of_order_count);
    }
}

// ============================================================================
// Test Result Structure
// ============================================================================

struct TestResult {
    success: bool,
    message: String,
}

impl TestResult {
    fn success(msg: impl Into<String>) -> Self {
        Self {
            success: true,
            message: msg.into(),
        }
    }

    fn failure(msg: impl Into<String>) -> Self {
        Self {
            success: false,
            message: msg.into(),
        }
    }
}

// ============================================================================
// Main Entry Point
// ============================================================================

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        print_usage();
        process::exit(1);
    }

    let test_name = &args[1];
    let result = match test_name.as_str() {
        // Big Data Types tests
        "large_messages_64b" => test_large_messages_64b(),
        "large_messages_256b" => test_large_messages_256b(),
        "large_messages_1kb" => test_large_messages_1kb(),
        "large_messages_4kb" => test_large_messages_4kb(),
        "large_messages_16kb" => test_large_messages_16kb(),
        "mixed_size_workload" => test_mixed_size_workload(),

        // Fast Storing tests
        "sustained_1khz" => test_sustained_1khz(),
        "burst_throughput" => test_burst_throughput(),
        "multi_producer_stress" => test_multi_producer_stress(),

        // Fast Tracking tests
        "telemetry_overhead" => test_telemetry_overhead(),
        "multi_link_monitoring" => test_multi_link_monitoring(),

        // Suite runners
        "big_data" => run_big_data_suite(),
        "fast_storing" => run_fast_storing_suite(),
        "fast_tracking" => run_fast_tracking_suite(),
        "all" => run_all_tests(),

        _ => {
            eprintln!("Unknown test: {}", test_name);
            print_usage();
            process::exit(1);
        }
    };

    if result.success {
        println!("\n{}", "=".repeat(70));
        println!("TEST PASSED: {}", test_name);
        println!("{}", "=".repeat(70));
        println!("{}", result.message);
        process::exit(0);
    } else {
        eprintln!("\n{}", "=".repeat(70));
        eprintln!("TEST FAILED: {}", test_name);
        eprintln!("{}", "=".repeat(70));
        eprintln!("{}", result.message);
        process::exit(1);
    }
}

fn print_usage() {
    eprintln!("Critical Test Suite for Single-Slot Link");
    eprintln!("\nUsage: test_critical_production <test_name>");
    eprintln!("\n=== BIG DATA TYPES Tests ===");
    eprintln!("  large_messages_64b    - Test 64-byte messages");
    eprintln!("  large_messages_256b   - Test 256-byte messages (camera metadata)");
    eprintln!("  large_messages_1kb    - Test 1KB messages (robot state)");
    eprintln!("  large_messages_4kb    - Test 4KB messages (image patches)");
    eprintln!("  large_messages_16kb   - Test 16KB messages (point clouds)");
    eprintln!("  mixed_size_workload   - Test mixed message sizes");
    eprintln!("\n=== FAST STORING Tests ===");
    eprintln!("  sustained_1khz        - Sustained 1000Hz (1ms) operation");
    eprintln!("  burst_throughput      - Maximum burst throughput");
    eprintln!("  multi_producer_stress - Multiple producers stress test");
    eprintln!("\n=== FAST TRACKING Tests ===");
    eprintln!("  telemetry_overhead    - Measure telemetry overhead");
    eprintln!("  multi_link_monitoring - Monitor multiple Links simultaneously");
    eprintln!("\n=== Test Suites ===");
    eprintln!("  big_data              - Run all big data tests");
    eprintln!("  fast_storing          - Run all fast storing tests");
    eprintln!("  fast_tracking         - Run all fast tracking tests");
    eprintln!("  all                   - Run complete test suite");
}

// ============================================================================
// BIG DATA TYPES Tests
// ============================================================================

fn test_large_messages_64b() -> TestResult {
    println!("Testing 64-byte messages (small sensor data)...");

    let topic = format!("test_64b_{}", process::id());
    let producer = match Topic::<SmallMessage>::new(&topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create producer: {}", e)),
    };

    let consumer = match Topic::<SmallMessage>::new(&topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create consumer: {}", e)),
    };

    let running = Arc::new(AtomicBool::new(true));
    let telemetry = Arc::new(Mutex::new(TelemetryData::default()));

    // Consumer thread
    let consumer_running = running.clone();
    let consumer_telemetry = telemetry.clone();
    let consumer_thread = thread::spawn(move || {
        let mut last_seq = 0u64;
        while consumer_running.load(Ordering::Relaxed) {
            if let Some(msg) = consumer.recv(&mut None) {
                let now = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as u64;
                let latency = now.saturating_sub(msg.timestamp);
                let mut telem = consumer_telemetry.lock().unwrap();
                telem.record_latency(latency);

                if !msg.verify() {
                    telem.corruption_count += 1;
                }
                if msg.sequence < last_seq {
                    telem.out_of_order_count += 1;
                }
                last_seq = msg.sequence;
            }
            thread::sleep(Duration::from_micros(10));
        }
    });

    // Send 10,000 messages at 100Hz
    let start = Instant::now();
    for i in 0..10000 {
        let msg = SmallMessage::new(i);
        if producer.send(msg, &mut None).is_err() {
            return TestResult::failure("Send failed");
        }
        thread::sleep(Duration::from_micros(100));
    }

    let duration = start.elapsed();
    thread::sleep(Duration::from_millis(100));
    running.store(false, Ordering::Relaxed);
    consumer_thread.join().unwrap();

    let telem = telemetry.lock().unwrap();
    telem.report("64B Messages");

    println!("  Total time: {:?}", duration);
    println!("  Message size: 64 bytes");
    println!(
        "  Throughput: {:.2} msg/s",
        10000.0 / duration.as_secs_f64()
    );
    println!(
        "  Bandwidth: {:.2} KB/s",
        (10000.0 * 64.0) / 1024.0 / duration.as_secs_f64()
    );

    if telem.corruption_count > 0 {
        return TestResult::failure(format!(
            "Data corruption detected: {}",
            telem.corruption_count
        ));
    }

    if telem.message_count < 9000 {
        return TestResult::failure(format!(
            "Insufficient messages received: {}",
            telem.message_count
        ));
    }

    TestResult::success(format!(
        "64B message test passed:\n\
         - Sent 10,000 messages\n\
         - Received {} messages\n\
         - Avg latency: {:.2} µs\n\
         - No corruption detected",
        telem.message_count,
        telem.avg_latency_us()
    ))
}

fn test_large_messages_256b() -> TestResult {
    println!("Testing 256-byte messages (camera metadata)...");

    let topic = format!("test_256b_{}", process::id());
    let producer = match Topic::<CameraMetadata>::new(&topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create producer: {}", e)),
    };

    let consumer = match Topic::<CameraMetadata>::new(&topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create consumer: {}", e)),
    };

    let running = Arc::new(AtomicBool::new(true));
    let telemetry = Arc::new(Mutex::new(TelemetryData::default()));

    // Consumer thread
    let consumer_running = running.clone();
    let consumer_telemetry = telemetry.clone();
    let consumer_thread = thread::spawn(move || {
        let mut last_seq = 0u64;
        while consumer_running.load(Ordering::Relaxed) {
            if let Some(msg) = consumer.recv(&mut None) {
                let now = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as u64;
                let latency = now.saturating_sub(msg.timestamp);
                let mut telem = consumer_telemetry.lock().unwrap();
                telem.record_latency(latency);

                if !msg.verify() {
                    telem.corruption_count += 1;
                }
                if msg.sequence < last_seq {
                    telem.out_of_order_count += 1;
                }
                last_seq = msg.sequence;
            }
            thread::sleep(Duration::from_micros(10));
        }
    });

    // Send 5,000 messages at 60Hz (camera rate)
    let start = Instant::now();
    for i in 0..5000 {
        let msg = CameraMetadata::new(i);
        if producer.send(msg, &mut None).is_err() {
            return TestResult::failure("Send failed");
        }
        thread::sleep(Duration::from_micros(167)); // ~60Hz
    }

    let duration = start.elapsed();
    thread::sleep(Duration::from_millis(100));
    running.store(false, Ordering::Relaxed);
    consumer_thread.join().unwrap();

    let telem = telemetry.lock().unwrap();
    telem.report("256B Messages");

    println!("  Total time: {:?}", duration);
    println!("  Message size: 256 bytes");
    println!("  Throughput: {:.2} msg/s", 5000.0 / duration.as_secs_f64());
    println!(
        "  Bandwidth: {:.2} KB/s",
        (5000.0 * 256.0) / 1024.0 / duration.as_secs_f64()
    );

    if telem.corruption_count > 0 {
        return TestResult::failure(format!(
            "Data corruption detected: {}",
            telem.corruption_count
        ));
    }

    if telem.message_count < 4500 {
        return TestResult::failure(format!(
            "Insufficient messages received: {}",
            telem.message_count
        ));
    }

    TestResult::success(format!(
        "256B message test passed:\n\
         - Sent 5,000 messages\n\
         - Received {} messages\n\
         - Avg latency: {:.2} µs\n\
         - No corruption detected",
        telem.message_count,
        telem.avg_latency_us()
    ))
}

fn test_large_messages_1kb() -> TestResult {
    println!("Testing 1KB messages (robot state)...");

    let topic = format!("test_1kb_{}", process::id());
    let producer = match Topic::<LargeState>::new(&topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create producer: {}", e)),
    };

    let consumer = match Topic::<LargeState>::new(&topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create consumer: {}", e)),
    };

    let running = Arc::new(AtomicBool::new(true));
    let telemetry = Arc::new(Mutex::new(TelemetryData::default()));

    // Consumer thread
    let consumer_running = running.clone();
    let consumer_telemetry = telemetry.clone();
    let consumer_thread = thread::spawn(move || {
        let mut last_seq = 0u64;
        while consumer_running.load(Ordering::Relaxed) {
            if let Some(msg) = consumer.recv(&mut None) {
                let now = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as u64;
                let latency = now.saturating_sub(msg.timestamp);
                let mut telem = consumer_telemetry.lock().unwrap();
                telem.record_latency(latency);

                if !msg.verify() {
                    telem.corruption_count += 1;
                }
                if msg.sequence < last_seq {
                    telem.out_of_order_count += 1;
                }
                last_seq = msg.sequence;
            }
            thread::sleep(Duration::from_micros(10));
        }
    });

    // Send 3,000 messages at 100Hz
    let start = Instant::now();
    for i in 0..3000 {
        let msg = LargeState::new(i);
        if producer.send(msg, &mut None).is_err() {
            return TestResult::failure("Send failed");
        }
        thread::sleep(Duration::from_micros(333)); // ~100Hz
    }

    let duration = start.elapsed();
    thread::sleep(Duration::from_millis(100));
    running.store(false, Ordering::Relaxed);
    consumer_thread.join().unwrap();

    let telem = telemetry.lock().unwrap();
    telem.report("1KB Messages");

    println!("  Total time: {:?}", duration);
    println!("  Message size: ~1KB");
    println!("  Throughput: {:.2} msg/s", 3000.0 / duration.as_secs_f64());
    println!(
        "  Bandwidth: {:.2} MB/s",
        (3000.0 * 1024.0) / (1024.0 * 1024.0) / duration.as_secs_f64()
    );

    if telem.corruption_count > 0 {
        return TestResult::failure(format!(
            "Data corruption detected: {}",
            telem.corruption_count
        ));
    }

    if telem.message_count < 2700 {
        return TestResult::failure(format!(
            "Insufficient messages received: {}",
            telem.message_count
        ));
    }

    TestResult::success(format!(
        "1KB message test passed:\n\
         - Sent 3,000 messages\n\
         - Received {} messages\n\
         - Avg latency: {:.2} µs\n\
         - No corruption detected",
        telem.message_count,
        telem.avg_latency_us()
    ))
}

fn test_large_messages_4kb() -> TestResult {
    println!("Testing 4KB messages (image patches)...");

    let topic = format!("test_4kb_{}", process::id());
    let producer = match Topic::<ImagePatch>::new(&topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create producer: {}", e)),
    };

    let consumer = match Topic::<ImagePatch>::new(&topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create consumer: {}", e)),
    };

    let running = Arc::new(AtomicBool::new(true));
    let telemetry = Arc::new(Mutex::new(TelemetryData::default()));

    // Consumer thread
    let consumer_running = running.clone();
    let consumer_telemetry = telemetry.clone();
    let consumer_thread = thread::spawn(move || {
        let mut last_seq = 0u64;
        while consumer_running.load(Ordering::Relaxed) {
            if let Some(msg) = consumer.recv(&mut None) {
                let now = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as u64;
                let latency = now.saturating_sub(msg.timestamp);
                let mut telem = consumer_telemetry.lock().unwrap();
                telem.record_latency(latency);

                if !msg.verify() {
                    telem.corruption_count += 1;
                }
                if msg.sequence < last_seq {
                    telem.out_of_order_count += 1;
                }
                last_seq = msg.sequence;
            }
            thread::sleep(Duration::from_micros(20));
        }
    });

    // Send 1,000 messages at 50Hz
    let start = Instant::now();
    for i in 0..1000 {
        let msg = ImagePatch::new(i);
        if producer.send(msg, &mut None).is_err() {
            return TestResult::failure("Send failed");
        }
        thread::sleep(Duration::from_micros(1000)); // 50Hz
    }

    let duration = start.elapsed();
    thread::sleep(Duration::from_millis(100));
    running.store(false, Ordering::Relaxed);
    consumer_thread.join().unwrap();

    let telem = telemetry.lock().unwrap();
    telem.report("4KB Messages");

    println!("  Total time: {:?}", duration);
    println!("  Message size: ~4KB");
    println!("  Throughput: {:.2} msg/s", 1000.0 / duration.as_secs_f64());
    println!(
        "  Bandwidth: {:.2} MB/s",
        (1000.0 * 4096.0) / (1024.0 * 1024.0) / duration.as_secs_f64()
    );

    if telem.corruption_count > 0 {
        return TestResult::failure(format!(
            "Data corruption detected: {}",
            telem.corruption_count
        ));
    }

    if telem.message_count < 900 {
        return TestResult::failure(format!(
            "Insufficient messages received: {}",
            telem.message_count
        ));
    }

    TestResult::success(format!(
        "4KB message test passed:\n\
         - Sent 1,000 messages\n\
         - Received {} messages\n\
         - Avg latency: {:.2} µs\n\
         - No corruption detected",
        telem.message_count,
        telem.avg_latency_us()
    ))
}

fn test_large_messages_16kb() -> TestResult {
    println!("Testing 16KB messages (point clouds)...");

    let topic = format!("test_16kb_{}", process::id());
    let producer = match Topic::<PointCloud>::new(&topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create producer: {}", e)),
    };

    let consumer = match Topic::<PointCloud>::new(&topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create consumer: {}", e)),
    };

    let running = Arc::new(AtomicBool::new(true));
    let telemetry = Arc::new(Mutex::new(TelemetryData::default()));

    // Consumer thread
    let consumer_running = running.clone();
    let consumer_telemetry = telemetry.clone();
    let consumer_thread = thread::spawn(move || {
        let mut last_seq = 0u64;
        while consumer_running.load(Ordering::Relaxed) {
            if let Some(msg) = consumer.recv(&mut None) {
                let now = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_nanos() as u64;
                let latency = now.saturating_sub(msg.timestamp);
                let mut telem = consumer_telemetry.lock().unwrap();
                telem.record_latency(latency);

                if !msg.verify() {
                    telem.corruption_count += 1;
                }
                if msg.sequence < last_seq {
                    telem.out_of_order_count += 1;
                }
                last_seq = msg.sequence;
            }
            thread::sleep(Duration::from_micros(50));
        }
    });

    // Send 500 messages at 10Hz (LiDAR rate)
    let start = Instant::now();
    for i in 0..500 {
        let msg = PointCloud::new(i);
        if producer.send(msg, &mut None).is_err() {
            return TestResult::failure("Send failed");
        }
        thread::sleep(Duration::from_micros(2000)); // 10Hz
    }

    let duration = start.elapsed();
    thread::sleep(Duration::from_millis(100));
    running.store(false, Ordering::Relaxed);
    consumer_thread.join().unwrap();

    let telem = telemetry.lock().unwrap();
    telem.report("16KB Messages");

    println!("  Total time: {:?}", duration);
    println!("  Message size: ~16KB");
    println!("  Throughput: {:.2} msg/s", 500.0 / duration.as_secs_f64());
    println!(
        "  Bandwidth: {:.2} MB/s",
        (500.0 * 16384.0) / (1024.0 * 1024.0) / duration.as_secs_f64()
    );

    if telem.corruption_count > 0 {
        return TestResult::failure(format!(
            "Data corruption detected: {}",
            telem.corruption_count
        ));
    }

    if telem.message_count < 450 {
        return TestResult::failure(format!(
            "Insufficient messages received: {}",
            telem.message_count
        ));
    }

    TestResult::success(format!(
        "16KB message test passed:\n\
         - Sent 500 messages\n\
         - Received {} messages\n\
         - Avg latency: {:.2} µs\n\
         - Max latency: {:.2} µs\n\
         - No corruption detected",
        telem.message_count,
        telem.avg_latency_us(),
        telem.max_latency_us()
    ))
}

fn test_mixed_size_workload() -> TestResult {
    println!("Testing mixed message size workload...");

    // Create links for different message sizes
    let topic_small = format!("test_mixed_small_{}", process::id());
    let topic_medium = format!("test_mixed_medium_{}", process::id());
    let topic_large = format!("test_mixed_large_{}", process::id());

    let producer_small = match Topic::<SmallMessage>::new(&topic_small) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create small producer: {}", e)),
    };
    let consumer_small = match Topic::<SmallMessage>::new(&topic_small) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create small consumer: {}", e)),
    };

    let producer_medium = match Topic::<CameraMetadata>::new(&topic_medium) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create medium producer: {}", e)),
    };
    let consumer_medium = match Topic::<CameraMetadata>::new(&topic_medium) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create medium consumer: {}", e)),
    };

    let producer_large = match Topic::<ImagePatch>::new(&topic_large) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create large producer: {}", e)),
    };
    let consumer_large = match Topic::<ImagePatch>::new(&topic_large) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create large consumer: {}", e)),
    };

    let running = Arc::new(AtomicBool::new(true));
    let small_count = Arc::new(AtomicU64::new(0));
    let medium_count = Arc::new(AtomicU64::new(0));
    let large_count = Arc::new(AtomicU64::new(0));

    // Consumer threads
    let small_running = running.clone();
    let small_cnt = small_count.clone();
    let small_thread = thread::spawn(move || {
        while small_running.load(Ordering::Relaxed) {
            if consumer_small.recv(&mut None).is_some() {
                small_cnt.fetch_add(1, Ordering::Relaxed);
            }
            thread::sleep(Duration::from_micros(10));
        }
    });

    let medium_running = running.clone();
    let medium_cnt = medium_count.clone();
    let medium_thread = thread::spawn(move || {
        while medium_running.load(Ordering::Relaxed) {
            if consumer_medium.recv(&mut None).is_some() {
                medium_cnt.fetch_add(1, Ordering::Relaxed);
            }
            thread::sleep(Duration::from_micros(20));
        }
    });

    let large_running = running.clone();
    let large_cnt = large_count.clone();
    let large_thread = thread::spawn(move || {
        while large_running.load(Ordering::Relaxed) {
            if consumer_large.recv(&mut None).is_some() {
                large_cnt.fetch_add(1, Ordering::Relaxed);
            }
            thread::sleep(Duration::from_micros(50));
        }
    });

    // Send mixed messages for 2 seconds
    let start = Instant::now();
    let mut small_sent = 0;
    let mut medium_sent = 0;
    let mut large_sent = 0;

    while start.elapsed() < Duration::from_secs(2) {
        let elapsed_us = start.elapsed().as_micros();

        // Small: 200Hz
        if elapsed_us % 5000 < 100 {
            let _ = producer_small.send(SmallMessage::new(small_sent), &mut None);
            small_sent += 1;
        }

        // Medium: 60Hz
        if elapsed_us % 16667 < 100 {
            let _ = producer_medium.send(CameraMetadata::new(medium_sent), &mut None);
            medium_sent += 1;
        }

        // Large: 20Hz
        if elapsed_us % 50000 < 100 {
            let _ = producer_large.send(ImagePatch::new(large_sent), &mut None);
            large_sent += 1;
        }

        thread::sleep(Duration::from_micros(100));
    }

    thread::sleep(Duration::from_millis(100));
    running.store(false, Ordering::Relaxed);

    small_thread.join().unwrap();
    medium_thread.join().unwrap();
    large_thread.join().unwrap();

    let small_rcvd = small_count.load(Ordering::Relaxed);
    let medium_rcvd = medium_count.load(Ordering::Relaxed);
    let large_rcvd = large_count.load(Ordering::Relaxed);

    println!(
        "  Small (64B): sent={}, received={}",
        small_sent, small_rcvd
    );
    println!(
        "  Medium (256B): sent={}, received={}",
        medium_sent, medium_rcvd
    );
    println!(
        "  Large (4KB): sent={}, received={}",
        large_sent, large_rcvd
    );

    // Validation
    if small_rcvd < (small_sent as f64 * 0.8) as u64 {
        return TestResult::failure(format!(
            "Too few small messages received: {}/{}",
            small_rcvd, small_sent
        ));
    }
    if medium_rcvd < (medium_sent as f64 * 0.8) as u64 {
        return TestResult::failure(format!(
            "Too few medium messages received: {}/{}",
            medium_rcvd, medium_sent
        ));
    }
    if large_rcvd < (large_sent as f64 * 0.8) as u64 {
        return TestResult::failure(format!(
            "Too few large messages received: {}/{}",
            large_rcvd, large_sent
        ));
    }

    TestResult::success(format!(
        "Mixed workload test passed:\n\
         - Small (64B): {}/{} received\n\
         - Medium (256B): {}/{} received\n\
         - Large (4KB): {}/{} received\n\
         - All message sizes handled correctly",
        small_rcvd, small_sent, medium_rcvd, medium_sent, large_rcvd, large_sent
    ))
}

// ============================================================================
// FAST STORING Tests
// ============================================================================

fn test_sustained_1khz() -> TestResult {
    println!("Testing sustained 1000Hz (1ms period) operation...");

    let topic = format!("test_1khz_{}", process::id());
    let producer = match Topic::<SmallMessage>::new(&topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create producer: {}", e)),
    };

    let consumer = match Topic::<SmallMessage>::new(&topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create consumer: {}", e)),
    };

    let running = Arc::new(AtomicBool::new(true));
    let received = Arc::new(AtomicU64::new(0));
    let telemetry = Arc::new(Mutex::new(TelemetryData::default()));

    // Consumer thread
    let consumer_running = running.clone();
    let consumer_received = received.clone();
    let consumer_telemetry = telemetry.clone();
    let consumer_thread = thread::spawn(move || {
        while consumer_running.load(Ordering::Relaxed) {
            let read_start = Instant::now();
            if let Some(msg) = consumer.recv(&mut None) {
                let latency = read_start.elapsed().as_nanos() as u64;
                consumer_received.fetch_add(1, Ordering::Relaxed);

                let mut telem = consumer_telemetry.lock().unwrap();
                telem.record_latency(latency);
                if !msg.verify() {
                    telem.corruption_count += 1;
                }
            }
            // High-frequency polling
            thread::sleep(Duration::from_micros(10));
        }
    });

    // Send at 1000Hz for 2 seconds
    let duration = Duration::from_secs(2);
    let period = Duration::from_micros(1000); // 1ms = 1kHz
    let start = Instant::now();
    let mut sent = 0u64;
    let mut missed_deadlines = 0;

    while start.elapsed() < duration {
        let cycle_start = Instant::now();

        let msg = SmallMessage::new(sent);
        if producer.send(msg, &mut None).is_err() {
            return TestResult::failure("Send failed");
        }
        sent += 1;

        let cycle_time = cycle_start.elapsed();
        if cycle_time > period {
            missed_deadlines += 1;
        }

        // Wait for next cycle
        if let Some(sleep_time) = period.checked_sub(cycle_time) {
            thread::sleep(sleep_time);
        }
    }

    let actual_duration = start.elapsed();
    thread::sleep(Duration::from_millis(100));
    running.store(false, Ordering::Relaxed);
    consumer_thread.join().unwrap();

    let rcvd = received.load(Ordering::Relaxed);
    let telem = telemetry.lock().unwrap();

    let achieved_hz = sent as f64 / actual_duration.as_secs_f64();

    println!("  Messages sent: {}", sent);
    println!("  Messages received: {}", rcvd);
    println!("  Target frequency: 1000 Hz");
    println!("  Achieved frequency: {:.2} Hz", achieved_hz);
    println!("  Missed deadlines: {}", missed_deadlines);
    telem.report("1kHz Sustained");

    // Validation
    if achieved_hz < 900.0 {
        return TestResult::failure(format!("Failed to sustain 1kHz: {:.2} Hz", achieved_hz));
    }

    if missed_deadlines > sent / 10 {
        return TestResult::failure(format!("Too many missed deadlines: {}", missed_deadlines));
    }

    if telem.corruption_count > 0 {
        return TestResult::failure(format!(
            "Data corruption at high frequency: {}",
            telem.corruption_count
        ));
    }

    if rcvd < (sent as f64 * 0.7) as u64 {
        return TestResult::failure(format!("Too many lost messages: {}/{}", rcvd, sent));
    }

    TestResult::success(format!(
        "Sustained 1kHz test passed:\n\
         - Sent {} messages at {:.2} Hz\n\
         - Received {} messages\n\
         - Avg latency: {:.2} µs\n\
         - Max latency: {:.2} µs\n\
         - Missed deadlines: {} ({:.1}%)\n\
         - No corruption detected",
        sent,
        achieved_hz,
        rcvd,
        telem.avg_latency_us(),
        telem.max_latency_us(),
        missed_deadlines,
        (missed_deadlines as f64 / sent as f64) * 100.0
    ))
}

fn test_burst_throughput() -> TestResult {
    println!("Testing burst throughput (maximum rate)...");

    let topic = format!("test_burst_{}", process::id());
    let producer = match Topic::<SmallMessage>::new(&topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create producer: {}", e)),
    };

    let consumer = match Topic::<SmallMessage>::new(&topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create consumer: {}", e)),
    };

    let running = Arc::new(AtomicBool::new(true));
    let received = Arc::new(AtomicU64::new(0));

    // Consumer thread
    let consumer_running = running.clone();
    let consumer_received = received.clone();
    let consumer_thread = thread::spawn(move || {
        while consumer_running.load(Ordering::Relaxed) {
            if consumer.recv(&mut None).is_some() {
                consumer_received.fetch_add(1, Ordering::Relaxed);
            }
            // Minimal sleep for maximum throughput
        }
    });

    // Send as fast as possible for 1 second
    let start = Instant::now();
    let mut sent = 0u64;

    while start.elapsed() < Duration::from_secs(1) {
        let msg = SmallMessage::new(sent);
        if producer.send(msg, &mut None).is_ok() {
            sent += 1;
        }
    }

    let duration = start.elapsed();
    thread::sleep(Duration::from_millis(200));
    running.store(false, Ordering::Relaxed);
    consumer_thread.join().unwrap();

    let rcvd = received.load(Ordering::Relaxed);

    let throughput = sent as f64 / duration.as_secs_f64();
    let bandwidth_mbps = (sent as f64 * 64.0) / duration.as_secs_f64() / (1024.0 * 1024.0);

    println!("  Messages sent: {}", sent);
    println!("  Messages received: {}", rcvd);
    println!("  Duration: {:?}", duration);
    println!(
        "  Throughput: {:.2} msg/s ({:.0} kHz)",
        throughput,
        throughput / 1000.0
    );
    println!("  Bandwidth: {:.2} MB/s", bandwidth_mbps);
    println!(
        "  Avg send time: {:.2} ns",
        duration.as_nanos() as f64 / sent as f64
    );

    // Validation
    if throughput < 100000.0 {
        return TestResult::failure(format!("Throughput too low: {:.2} msg/s", throughput));
    }

    if rcvd < (sent / 100) {
        return TestResult::failure(format!(
            "Consumer missed too many messages: {}/{}",
            rcvd, sent
        ));
    }

    TestResult::success(format!(
        "Burst throughput test passed:\n\
         - Sent {} messages in {:?}\n\
         - Throughput: {:.2} kHz\n\
         - Bandwidth: {:.2} MB/s\n\
         - Consumer received {} messages\n\
         - System handles extreme burst rates",
        sent,
        duration,
        throughput / 1000.0,
        bandwidth_mbps,
        rcvd
    ))
}

fn test_multi_producer_stress() -> TestResult {
    println!("Testing multiple producers (separate Links) to single consumer thread (stress)...");

    // NOTE: Single-slot Link is designed for SPSC (Single Producer Single Consumer)
    // This test validates multiple separate producer Links being monitored by one consumer thread
    // Each producer has its own Link - this is the correct usage pattern

    let num_producers = 4;
    let running = Arc::new(AtomicBool::new(true));
    let received_counts = Arc::new(Mutex::new(vec![0u64; num_producers]));
    let corrupted = Arc::new(AtomicU64::new(0));

    // Create producer and consumer pairs
    let mut producers = vec![];
    let mut consumers = vec![];

    for i in 0..num_producers {
        let topic = format!("test_multi_prod_{}_{}", i, process::id());

        let producer = match Topic::<SmallMessage>::new(&topic) {
            Ok(p) => p,
            Err(e) => {
                return TestResult::failure(format!("Failed to create producer {}: {}", i, e))
            }
        };

        let consumer = match Topic::<SmallMessage>::new(&topic) {
            Ok(c) => c,
            Err(e) => {
                return TestResult::failure(format!("Failed to create consumer {}: {}", i, e))
            }
        };

        producers.push(producer);
        consumers.push(consumer);
    }

    // Single consumer thread monitoring all Links
    let consumer_running = running.clone();
    let consumer_received = received_counts.clone();
    let consumer_corrupted = corrupted.clone();
    let consumer_thread = thread::spawn(move || {
        while consumer_running.load(Ordering::Relaxed) {
            for (i, consumer) in consumers.iter().enumerate() {
                if let Some(msg) = consumer.recv(&mut None) {
                    let mut counts = consumer_received.lock().unwrap();
                    counts[i] += 1;

                    if !msg.verify() {
                        consumer_corrupted.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
            thread::sleep(Duration::from_micros(5));
        }
    });

    // Spawn producer threads
    let mut producer_threads = vec![];
    let sent_counts = Arc::new(Mutex::new(vec![0u64; num_producers]));

    for (producer_id, producer) in producers.into_iter().enumerate() {
        let running_clone = running.clone();
        let sent_counts_clone = sent_counts.clone();

        let thread = thread::spawn(move || {
            let start = Instant::now();
            let mut sent = 0u64;

            while start.elapsed() < Duration::from_secs(2) && running_clone.load(Ordering::Relaxed)
            {
                let seq = (producer_id as u64) * 1000000 + sent;
                let msg = SmallMessage::new(seq);
                if producer.send(msg, &mut None).is_ok() {
                    sent += 1;
                }
                thread::sleep(Duration::from_micros(100)); // 10kHz per producer
            }

            let mut counts = sent_counts_clone.lock().unwrap();
            counts[producer_id] = sent;
            sent
        });

        producer_threads.push(thread);
    }

    // Wait for producers to finish
    for thread in producer_threads {
        thread.join().unwrap();
    }

    thread::sleep(Duration::from_millis(200));
    running.store(false, Ordering::Relaxed);
    consumer_thread.join().unwrap();

    let rcvd_counts = received_counts.lock().unwrap();
    let sent_counts = sent_counts.lock().unwrap();
    let total_sent: u64 = sent_counts.iter().sum();
    let total_rcvd: u64 = rcvd_counts.iter().sum();
    let corrupt = corrupted.load(Ordering::Relaxed);

    println!("  Producers: {}", num_producers);
    for i in 0..num_producers {
        println!(
            "    Producer {}: sent={}, received={}",
            i, sent_counts[i], rcvd_counts[i]
        );
    }
    println!("  Total sent: {}", total_sent);
    println!("  Total received: {}", total_rcvd);
    println!("  Corrupted: {}", corrupt);

    // Validation
    if corrupt > 0 {
        return TestResult::failure(format!("Data corruption under stress: {}", corrupt));
    }

    if total_rcvd < (total_sent / 2) {
        return TestResult::failure(format!(
            "Too few messages received: {}/{}",
            total_rcvd, total_sent
        ));
    }

    TestResult::success(format!(
        "Multi-producer stress test passed:\n\
         - {} producers (separate Links) sent {} total messages\n\
         - Single consumer thread received {} messages\n\
         - No corruption detected\n\
         - Proper SPSC usage pattern validated",
        num_producers, total_sent, total_rcvd
    ))
}

// ============================================================================
// FAST TRACKING Tests
// ============================================================================

fn test_telemetry_overhead() -> TestResult {
    println!("Testing telemetry overhead impact...");

    let topic = format!("test_telemetry_{}", process::id());
    let producer = match Topic::<SmallMessage>::new(&topic) {
        Ok(p) => p,
        Err(e) => return TestResult::failure(format!("Failed to create producer: {}", e)),
    };

    let consumer = match Topic::<SmallMessage>::new(&topic) {
        Ok(c) => c,
        Err(e) => return TestResult::failure(format!("Failed to create consumer: {}", e)),
    };

    // Test 1: Without telemetry tracking
    println!("  Baseline test (no tracking)...");
    let start = Instant::now();
    for i in 0..10000 {
        let msg = SmallMessage::new(i);
        let _ = producer.send(msg, &mut None);
    }
    let baseline_duration = start.elapsed();
    let baseline_ns_per_msg = baseline_duration.as_nanos() / 10000;

    println!(
        "    Baseline: {:?} ({} ns/msg)",
        baseline_duration, baseline_ns_per_msg
    );

    // Test 2: With telemetry tracking
    println!("  With telemetry tracking...");
    let telemetry = Arc::new(Mutex::new(TelemetryData::default()));
    let running = Arc::new(AtomicBool::new(true));

    let consumer_running = running.clone();
    let consumer_telemetry = telemetry.clone();
    let consumer_thread = thread::spawn(move || {
        while consumer_running.load(Ordering::Relaxed) {
            let read_start = Instant::now();
            if let Some(msg) = consumer.recv(&mut None) {
                let latency = read_start.elapsed().as_nanos() as u64;
                let mut telem = consumer_telemetry.lock().unwrap();
                telem.record_latency(latency);
                if !msg.verify() {
                    telem.corruption_count += 1;
                }
            }
            thread::sleep(Duration::from_micros(5));
        }
    });

    let start = Instant::now();
    for i in 0..10000 {
        let send_start = Instant::now();
        let msg = SmallMessage::new(i);
        let _ = producer.send(msg, &mut None);
        let send_latency = send_start.elapsed().as_nanos() as u64;

        // Track send latency
        let mut telem = telemetry.lock().unwrap();
        telem.record_latency(send_latency);
    }
    let tracked_duration = start.elapsed();
    let tracked_ns_per_msg = tracked_duration.as_nanos() / 10000;

    thread::sleep(Duration::from_millis(100));
    running.store(false, Ordering::Relaxed);
    consumer_thread.join().unwrap();

    let overhead_ns = tracked_ns_per_msg - baseline_ns_per_msg;
    let overhead_percent = (overhead_ns as f64 / baseline_ns_per_msg as f64) * 100.0;

    println!(
        "    With tracking: {:?} ({} ns/msg)",
        tracked_duration, tracked_ns_per_msg
    );
    println!(
        "    Overhead: {} ns/msg ({:.2}%)",
        overhead_ns, overhead_percent
    );

    let telem = telemetry.lock().unwrap();
    telem.report("Telemetry Overhead");

    // Validation: Overhead from mutex locking is expected
    // The raw Link operations are very fast (24ns), so any tracking adds relative overhead
    // What matters is absolute overhead stays reasonable (< 1us per message)
    if overhead_ns > 1000 {
        return TestResult::failure(format!(
            "Telemetry overhead too high: {} ns/msg",
            overhead_ns
        ));
    }

    TestResult::success(format!(
        "Telemetry overhead test passed:\n\
         - Baseline: {} ns/msg (raw Link performance)\n\
         - With tracking: {} ns/msg\n\
         - Overhead: {} ns ({:.2}%)\n\
         - Absolute overhead acceptable (< 1µs)\n\
         - Note: Mutex locking adds overhead, but ensures thread-safe tracking",
        baseline_ns_per_msg, tracked_ns_per_msg, overhead_ns, overhead_percent
    ))
}

fn test_multi_link_monitoring() -> TestResult {
    println!("Testing simultaneous monitoring of multiple Links...");

    // Create 5 different Links with different message types
    let topics: Vec<String> = (0..5)
        .map(|i| format!("test_monitor_{}_{}", i, process::id()))
        .collect();

    let mut producers = vec![];
    let mut consumers = vec![];

    for topic in &topics {
        let producer = match Topic::<SmallMessage>::new(topic) {
            Ok(p) => p,
            Err(e) => return TestResult::failure(format!("Failed to create producer: {}", e)),
        };
        let consumer = match Topic::<SmallMessage>::new(topic) {
            Ok(c) => c,
            Err(e) => return TestResult::failure(format!("Failed to create consumer: {}", e)),
        };
        producers.push(producer);
        consumers.push(consumer);
    }

    let running = Arc::new(AtomicBool::new(true));
    let telemetries: Vec<Arc<Mutex<TelemetryData>>> = (0..5)
        .map(|_| Arc::new(Mutex::new(TelemetryData::default())))
        .collect();

    // Spawn consumer threads for each Link
    let mut consumer_threads = vec![];
    for (i, consumer) in consumers.into_iter().enumerate() {
        let running_clone = running.clone();
        let telemetry = telemetries[i].clone();

        let thread = thread::spawn(move || {
            while running_clone.load(Ordering::Relaxed) {
                let read_start = Instant::now();
                if let Some(msg) = consumer.recv(&mut None) {
                    let latency = read_start.elapsed().as_nanos() as u64;
                    let mut telem = telemetry.lock().unwrap();
                    telem.record_latency(latency);
                    if !msg.verify() {
                        telem.corruption_count += 1;
                    }
                }
                thread::sleep(Duration::from_micros(10));
            }
        });
        consumer_threads.push(thread);
    }

    // Send messages on all Links simultaneously
    let start = Instant::now();
    let duration = Duration::from_secs(2);
    let mut sent_counts = vec![0u64; 5];

    while start.elapsed() < duration {
        for (i, producer) in producers.iter().enumerate() {
            let msg = SmallMessage::new(sent_counts[i]);
            if producer.send(msg, &mut None).is_ok() {
                sent_counts[i] += 1;
            }
        }
        thread::sleep(Duration::from_micros(100)); // ~10kHz per Link
    }

    thread::sleep(Duration::from_millis(200));
    running.store(false, Ordering::Relaxed);

    for thread in consumer_threads {
        thread.join().unwrap();
    }

    println!("\n  Per-Link Statistics:");
    let mut total_messages = 0u64;
    let mut total_corrupted = 0u64;

    for (i, telemetry) in telemetries.iter().enumerate() {
        let telem = telemetry.lock().unwrap();
        println!(
            "    Link {}: {} msg, {:.2} µs avg, {} corrupted",
            i,
            telem.message_count,
            telem.avg_latency_us(),
            telem.corruption_count
        );
        total_messages += telem.message_count;
        total_corrupted += telem.corruption_count;
    }

    println!("\n  Total sent: {}", sent_counts.iter().sum::<u64>());
    println!("  Total received: {}", total_messages);
    println!("  Total corrupted: {}", total_corrupted);

    // Validation
    if total_corrupted > 0 {
        return TestResult::failure(format!("Data corruption detected: {}", total_corrupted));
    }

    let min_expected = sent_counts.iter().sum::<u64>() / 2;
    if total_messages < min_expected {
        return TestResult::failure(format!("Too few messages received: {}", total_messages));
    }

    TestResult::success(format!(
        "Multi-Link monitoring test passed:\n\
         - Monitored 5 Links simultaneously\n\
         - Total messages: {}\n\
         - No corruption detected\n\
         - All Links tracked independently\n\
         - Monitoring scales to multiple Links",
        total_messages
    ))
}

// ============================================================================
// Test Suite Runners
// ============================================================================

fn run_big_data_suite() -> TestResult {
    println!("Running BIG DATA TYPES test suite...\n");

    let tests: Vec<(&str, fn() -> TestResult)> = vec![
        ("large_messages_64b", test_large_messages_64b),
        ("large_messages_256b", test_large_messages_256b),
        ("large_messages_1kb", test_large_messages_1kb),
        ("large_messages_4kb", test_large_messages_4kb),
        ("large_messages_16kb", test_large_messages_16kb),
        ("mixed_size_workload", test_mixed_size_workload),
    ];

    run_test_suite("BIG DATA TYPES", tests)
}

fn run_fast_storing_suite() -> TestResult {
    println!("Running FAST STORING test suite...\n");

    let tests: Vec<(&str, fn() -> TestResult)> = vec![
        ("sustained_1khz", test_sustained_1khz),
        ("burst_throughput", test_burst_throughput),
        ("multi_producer_stress", test_multi_producer_stress),
    ];

    run_test_suite("FAST STORING", tests)
}

fn run_fast_tracking_suite() -> TestResult {
    println!("Running FAST TRACKING test suite...\n");

    let tests: Vec<(&str, fn() -> TestResult)> = vec![
        ("telemetry_overhead", test_telemetry_overhead),
        ("multi_link_monitoring", test_multi_link_monitoring),
    ];

    run_test_suite("FAST TRACKING", tests)
}

fn run_all_tests() -> TestResult {
    println!("Running COMPLETE CRITICAL TEST SUITE...\n");

    let suites: Vec<(&str, fn() -> TestResult)> = vec![
        ("BIG DATA TYPES", run_big_data_suite),
        ("FAST STORING", run_fast_storing_suite),
        ("FAST TRACKING", run_fast_tracking_suite),
    ];

    let mut passed = 0;
    let mut failed = 0;

    for (name, suite_fn) in suites {
        println!("\n{}", "=".repeat(70));
        println!("SUITE: {}", name);
        println!("{}", "=".repeat(70));

        let result = suite_fn();
        if result.success {
            println!("\nSUITE PASSED: {}", name);
            passed += 1;
        } else {
            eprintln!("\nSUITE FAILED: {}", name);
            eprintln!("{}", result.message);
            failed += 1;
        }
    }

    println!("\n{}", "=".repeat(70));
    println!(
        "FINAL SUMMARY: {} suites passed, {} suites failed",
        passed, failed
    );
    println!("{}", "=".repeat(70));

    if failed == 0 {
        TestResult::success(format!(
            "ALL {} TEST SUITES PASSED!\n\
             System validated for:\n\
             - Large data transfers (up to 16KB)\n\
             - High-frequency operation (1kHz+)\n\
             - Real-time telemetry and monitoring",
            passed
        ))
    } else {
        TestResult::failure(format!(
            "{} out of {} test suites failed.\n\
             System needs fixes before deployment.",
            failed,
            passed + failed
        ))
    }
}

fn run_test_suite(suite_name: &str, tests: Vec<(&str, fn() -> TestResult)>) -> TestResult {
    let mut passed = 0;
    let mut failed = 0;

    for (name, test_fn) in tests {
        println!("\n{}", "-".repeat(70));
        println!("Running test: {}", name);
        println!("{}", "-".repeat(70));

        let result = test_fn();
        if result.success {
            println!("\nPASSED: {}", name);
            passed += 1;
        } else {
            eprintln!("\nFAILED: {}", name);
            eprintln!("  {}", result.message);
            failed += 1;
        }

        // Small delay between tests
        thread::sleep(Duration::from_millis(100));
    }

    println!("\n{}", "-".repeat(70));
    println!(
        "{} Suite Summary: {} passed, {} failed",
        suite_name, passed, failed
    );
    println!("{}", "-".repeat(70));

    if failed == 0 {
        TestResult::success(format!(
            "{} suite: All {} tests passed!",
            suite_name, passed
        ))
    } else {
        TestResult::failure(format!(
            "{} suite: {} out of {} tests failed.",
            suite_name,
            failed,
            passed + failed
        ))
    }
}
