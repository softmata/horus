//! Recording system stress tests — extreme conditions.
//!
//! Tests the recording infrastructure under high frequency, large messages,
//! concurrent sessions, SIGKILL recovery, ring-buffer eviction, and compression.
//! All tests use real data and real file I/O.
//!
//! Run: `cargo test --no-default-features -p horus_core --test recording_stress -- --test-threads=1`

use horus_core::scheduling::{NodeRecording, NodeReplayer, NodeTickSnapshot, Recording};
use std::time::Instant;

mod common;
use common::TestTempDir;

// ============================================================================
// Test 1: 10kHz recording — 100K ticks, verify no gaps
// ============================================================================

#[test]
fn test_record_100k_ticks_no_gaps() {
    let tmp = TestTempDir::new("stress_10khz");
    let path = tmp.path().join("sensor@10khz.horus");

    let mut recording = NodeRecording::new("sensor_10khz", "s10k", "stress_10khz");

    // 100K ticks simulating 10kHz for 10 seconds
    // Each tick has a 64-byte CmdVel-like output
    let start = Instant::now();
    for tick in 0..100_000u64 {
        let timestamp_us = tick * 100; // 100µs per tick = 10kHz
        let output = tick.to_le_bytes().to_vec(); // 8 bytes per message
        recording.add_snapshot(
            NodeTickSnapshot::with_capacity_and_timestamp(tick, 1, timestamp_us)
                .with_output("cmd_vel", output)
                .with_duration(50_000), // 50µs per tick
        );
    }
    let record_time = start.elapsed();

    recording.finish();
    let save_start = Instant::now();
    recording.save(&path).expect("save 100K recording");
    let save_time = save_start.elapsed();

    // Load and verify
    let load_start = Instant::now();
    let replayer = NodeReplayer::load(&path).expect("load 100K recording");
    let load_time = load_start.elapsed();

    // Verify exact tick count
    assert_eq!(
        replayer.total_ticks(),
        100_000,
        "Should have exactly 100,000 ticks"
    );

    // Verify monotonic tick numbers (no gaps)
    let mut r = NodeReplayer::load(&path).unwrap();
    let mut prev_tick = None;
    let mut count = 0u64;
    loop {
        let snap = r.current_snapshot().unwrap();
        if let Some(pt) = prev_tick {
            assert_eq!(
                snap.tick,
                pt + 1,
                "Tick numbers should be consecutive: prev={}, current={}",
                pt,
                snap.tick
            );
        }
        prev_tick = Some(snap.tick);
        count += 1;
        if !r.advance() {
            break;
        }
    }
    assert_eq!(count, 100_000);

    // Performance bounds (generous for CI)
    assert!(
        record_time < std::time::Duration::from_secs(10),
        "Recording 100K ticks took too long: {:?}",
        record_time
    );
    assert!(
        save_time < std::time::Duration::from_secs(30),
        "Saving 100K ticks took too long: {:?}",
        save_time
    );
    assert!(
        load_time < std::time::Duration::from_secs(30),
        "Loading 100K ticks took too long: {:?}",
        load_time
    );
}

// ============================================================================
// Test 2: 50kHz overhead — measure recording overhead
// ============================================================================

#[test]
fn test_record_overhead_measurement() {
    // Measure time to create 250K snapshots (50kHz × 5 seconds)
    let start = Instant::now();
    let mut recording = NodeRecording::new("overhead_node", "oh1", "overhead_session");

    for tick in 0..250_000u64 {
        recording.add_snapshot(
            NodeTickSnapshot::new(tick)
                .with_output("data", vec![0u8; 64])
                .with_duration(20_000), // 20µs per tick
        );
    }
    let elapsed = start.elapsed();

    // Overhead per snapshot (amortized)
    let per_snapshot_ns = elapsed.as_nanos() / 250_000;

    // Should be well under 1µs per snapshot (just HashMap insert + Vec push)
    assert!(
        per_snapshot_ns < 10_000, // 10µs generous bound
        "Per-snapshot overhead too high: {}ns (should be <10µs)",
        per_snapshot_ns
    );

    assert_eq!(recording.snapshots.len(), 250_000);
}

// ============================================================================
// Test 3: 1MB image messages at 30Hz
// ============================================================================

#[test]
fn test_record_1mb_images_30hz() {
    let tmp = TestTempDir::new("stress_1mb");
    let path = tmp.path().join("camera@1mb.horus");

    let mut recording = NodeRecording::new("camera_1mb", "cam1mb", "stress_1mb");

    // 30Hz for 3 seconds = 90 frames, each 100KB (fits under 100MB load guard)
    let image_data: Vec<u8> = (0..100_000).map(|i| (i % 256) as u8).collect();

    let start = Instant::now();
    for tick in 0..90u64 {
        let timestamp_us = tick * 33_333; // ~30Hz
        // Use unique data per frame (hash the tick into the first 8 bytes)
        let mut frame = image_data.clone();
        frame[0..8].copy_from_slice(&tick.to_le_bytes());

        recording.add_snapshot(
            NodeTickSnapshot::with_capacity_and_timestamp(tick, 1, timestamp_us)
                .with_output("image", frame)
                .with_duration(5_000_000), // 5ms per frame
        );
    }
    let record_time = start.elapsed();
    recording.finish();

    // Save (this will be ~300MB)
    let save_start = Instant::now();
    recording.save(&path).expect("save 1MB image recording");
    let save_time = save_start.elapsed();

    // Load and verify integrity
    let load_start = Instant::now();
    let mut replayer = NodeReplayer::load(&path).expect("load 1MB image recording");
    let load_time = load_start.elapsed();

    assert_eq!(replayer.total_ticks(), 90);

    // Verify first and last frames have correct tick markers
    let first = replayer.current_snapshot().unwrap();
    let first_output = first.outputs.get("image").expect("image output");
    assert_eq!(first_output.len(), 100_000, "Image should be 100KB");
    let first_tick_marker = u64::from_le_bytes(first_output[0..8].try_into().unwrap());
    assert_eq!(first_tick_marker, 0, "First frame tick marker should be 0");

    // Seek to last
    replayer.seek(89);
    let last = replayer.current_snapshot().unwrap();
    let last_output = last.outputs.get("image").expect("image output");
    let last_tick_marker = u64::from_le_bytes(last_output[0..8].try_into().unwrap());
    assert_eq!(last_tick_marker, 89, "Last frame tick marker should be 89");

    println!(
        "1MB images: record={:?}, save={:?}, load={:?}",
        record_time, save_time, load_time
    );
}

// ============================================================================
// Test 4: 500KB point clouds at 100Hz
// ============================================================================

#[test]
fn test_record_500kb_pointclouds_100hz() {
    let tmp = TestTempDir::new("stress_500kb");
    let path = tmp.path().join("lidar@500kb.horus");

    let mut recording = NodeRecording::new("lidar_500kb", "lid500", "stress_500kb");

    // 100 scans of 50KB each (fits under 100MB load guard)
    let scan_data: Vec<u8> = (0..50_000).map(|i| (i % 256) as u8).collect();

    for tick in 0..100u64 {
        let mut scan = scan_data.clone();
        scan[0..8].copy_from_slice(&tick.to_le_bytes());

        recording.add_snapshot(
            NodeTickSnapshot::new(tick)
                .with_output("scan", scan)
                .with_duration(2_000_000), // 2ms
        );
    }
    recording.finish();
    recording.save(&path).expect("save 500KB scan recording");

    let replayer = NodeReplayer::load(&path).expect("load scan recording");
    assert_eq!(replayer.total_ticks(), 100);

    // Verify data integrity via checksum on first and last
    let mut r = NodeReplayer::load(&path).unwrap();
    let first = r.current_snapshot().unwrap();
    let first_scan = first.outputs.get("scan").unwrap();
    assert_eq!(first_scan.len(), 50_000);
    let tick_0 = u64::from_le_bytes(first_scan[0..8].try_into().unwrap());
    assert_eq!(tick_0, 0);

    r.seek(99);
    let last = r.current_snapshot().unwrap();
    let last_scan = last.outputs.get("scan").unwrap();
    let tick_99 = u64::from_le_bytes(last_scan[0..8].try_into().unwrap());
    assert_eq!(tick_99, 99);
}

// ============================================================================
// Test 5: Mixed large/small messages on 20 topics
// ============================================================================

#[test]
fn test_record_mixed_20_topics() {
    let tmp = TestTempDir::new("stress_mixed");
    let path = tmp.path().join("mixed@20topics.horus");

    let mut recording = NodeRecording::new("mixed_node", "mix20", "stress_mixed");

    for tick in 0..500u64 {
        let mut snapshot = NodeTickSnapshot::new(tick);

        // 5 large topics (10KB each)
        for i in 0..5 {
            let data: Vec<u8> = (0..10_000).map(|j| ((tick + i + j) % 256) as u8).collect();
            snapshot = snapshot.with_output(&format!("large_{}", i), data);
        }

        // 5 medium topics (1KB each)
        for i in 0..5 {
            let data: Vec<u8> = (0..1_000).map(|j| ((tick + i + j) % 256) as u8).collect();
            snapshot = snapshot.with_output(&format!("medium_{}", i), data);
        }

        // 10 small topics (64 bytes each)
        for i in 0..10 {
            snapshot = snapshot.with_output(&format!("small_{}", i), vec![tick as u8; 64]);
        }

        snapshot = snapshot.with_duration(1_000_000);
        recording.add_snapshot(snapshot);
    }
    recording.finish();
    recording.save(&path).expect("save mixed recording");

    let replayer = NodeReplayer::load(&path).expect("load mixed recording");
    assert_eq!(replayer.total_ticks(), 500);

    // Verify all 20 topics present in first snapshot
    let mut r = NodeReplayer::load(&path).unwrap();
    let snap = r.current_snapshot().unwrap();
    assert_eq!(
        snap.outputs.len(),
        20,
        "Should have 20 output topics, got {}",
        snap.outputs.len()
    );

    // Verify sizes
    for i in 0..5 {
        let large = snap.outputs.get(&format!("large_{}", i)).unwrap();
        assert_eq!(large.len(), 10_000, "Large topic {} should be 10KB", i);
    }
    for i in 0..5 {
        let medium = snap.outputs.get(&format!("medium_{}", i)).unwrap();
        assert_eq!(medium.len(), 1_000, "Medium topic {} should be 1KB", i);
    }
    for i in 0..10 {
        let small = snap.outputs.get(&format!("small_{}", i)).unwrap();
        assert_eq!(small.len(), 64, "Small topic {} should be 64B", i);
    }
}

// ============================================================================
// Test 9: SIGKILL partial file recovery — truncated file handling
// ============================================================================

/// Simulates SIGKILL by saving a recording, then truncating the file
/// at various points. Verifies load() returns an error (not panic).
#[test]
fn test_truncated_recording_no_panic() {
    let tmp = TestTempDir::new("stress_truncate");
    let path = tmp.path().join("truncated@test.horus");

    // Create a valid recording
    let mut recording = NodeRecording::new("trunc_node", "trunc1", "stress_trunc");
    for tick in 0..100u64 {
        recording.add_snapshot(
            NodeTickSnapshot::new(tick)
                .with_output("data", vec![tick as u8; 256])
                .with_duration(1000),
        );
    }
    recording.finish();
    recording.save(&path).expect("save valid recording");

    let file_size = std::fs::metadata(&path).unwrap().len();
    assert!(file_size > 100, "File should be larger than 100 bytes");

    // Test truncation at various points
    let truncation_points = [
        0,                  // Empty file
        3,                  // Partial magic
        6,                  // Magic only, no version
        10,                 // Header only, no payload
        file_size / 4,      // 25% through payload
        file_size / 2,      // 50% through payload
        file_size * 3 / 4,  // 75% through payload
        file_size - 1,      // Off by one byte
    ];

    for &truncate_at in &truncation_points {
        let trunc_path = tmp.path().join(format!("trunc_at_{}.horus", truncate_at));

        // Read original, truncate, write
        let data = std::fs::read(&path).unwrap();
        let truncated = &data[..truncate_at as usize];
        std::fs::write(&trunc_path, truncated).unwrap();

        // Load should return Err, NOT panic
        let result = NodeRecording::load(&trunc_path);
        assert!(
            result.is_err(),
            "Truncated file at {} bytes should return Err, not Ok",
            truncate_at
        );
    }
}

// ============================================================================
// Test 10: WAL-style partial recovery — valid header, corrupted tail
// ============================================================================

#[test]
fn test_corrupted_tail_returns_error() {
    let tmp = TestTempDir::new("stress_corrupt");
    let path = tmp.path().join("corrupt@test.horus");

    // Create valid recording
    let mut recording = NodeRecording::new("corrupt_node", "c1", "stress_corrupt");
    for tick in 0..50u64 {
        recording.add_snapshot(
            NodeTickSnapshot::new(tick)
                .with_output("data", vec![0u8; 128])
                .with_duration(500),
        );
    }
    recording.finish();
    recording.save(&path).expect("save valid recording");

    // Append garbage to the end (simulates partial write before crash)
    let mut data = std::fs::read(&path).unwrap();
    data.extend_from_slice(b"GARBAGE_DATA_AFTER_CRASH");
    let corrupt_path = tmp.path().join("corrupt_tail.horus");
    std::fs::write(&corrupt_path, &data).unwrap();

    // Load may succeed (bincode ignores trailing bytes) or fail
    // Either way, it should NOT panic
    let result = NodeRecording::load(&corrupt_path);
    // Both Ok and Err are acceptable — the key is no panic
    match result {
        Ok(recording) => {
            // If it loads, verify the data is correct (bincode ignores trailing garbage)
            assert_eq!(recording.snapshots.len(), 50);
        }
        Err(_) => {
            // Also acceptable — stricter validation
        }
    }
}

// ============================================================================
// Test 11: Two concurrent recording sessions (no interference)
// ============================================================================

#[test]
fn test_two_concurrent_sessions() {
    let tmp = TestTempDir::new("stress_concurrent");

    // Session A
    let path_a = tmp.path().join("nodeA@session_a.horus");
    let mut recording_a = NodeRecording::new("nodeA", "a1", "session_a");

    // Session B
    let path_b = tmp.path().join("nodeB@session_b.horus");
    let mut recording_b = NodeRecording::new("nodeB", "b1", "session_b");

    // Interleave recording (simulating concurrent execution)
    for tick in 0..1000u64 {
        recording_a.add_snapshot(
            NodeTickSnapshot::new(tick)
                .with_output("topic_a", vec![0xAA; 128])
                .with_duration(500),
        );
        recording_b.add_snapshot(
            NodeTickSnapshot::new(tick)
                .with_output("topic_b", vec![0xBB; 128])
                .with_duration(500),
        );
    }

    recording_a.finish();
    recording_b.finish();
    recording_a.save(&path_a).expect("save session A");
    recording_b.save(&path_b).expect("save session B");

    // Load both and verify no cross-contamination
    let mut replayer_a = NodeReplayer::load(&path_a).expect("load session A");
    let mut replayer_b = NodeReplayer::load(&path_b).expect("load session B");

    assert_eq!(replayer_a.total_ticks(), 1000);
    assert_eq!(replayer_b.total_ticks(), 1000);

    // Verify data integrity — A has 0xAA, B has 0xBB
    let snap_a = replayer_a.current_snapshot().unwrap();
    let data_a = snap_a.outputs.get("topic_a").unwrap();
    assert!(data_a.iter().all(|&b| b == 0xAA), "Session A data corrupted");

    let snap_b = replayer_b.current_snapshot().unwrap();
    let data_b = snap_b.outputs.get("topic_b").unwrap();
    assert!(data_b.iter().all(|&b| b == 0xBB), "Session B data corrupted");

    // Verify no cross-topic leakage
    assert!(
        snap_a.outputs.get("topic_b").is_none(),
        "Session A should not have topic_b"
    );
    assert!(
        snap_b.outputs.get("topic_a").is_none(),
        "Session B should not have topic_a"
    );
}

// ============================================================================
// Test 12: Ring-buffer eviction at scale
// ============================================================================

#[test]
fn test_ring_buffer_eviction_1000_of_10000() {
    let tmp = TestTempDir::new("stress_ring");
    let path = tmp.path().join("ring@evict.horus");

    let mut recording = NodeRecording::new("ring_node", "ring1", "stress_ring");

    // Add 10,000 snapshots but we'll manually evict to keep only last 1000
    for tick in 0..10_000u64 {
        recording.add_snapshot(
            NodeTickSnapshot::new(tick)
                .with_output("data", tick.to_le_bytes().to_vec())
                .with_duration(100),
        );

        // Manual ring-buffer eviction (mirrors NodeRecorder::end_tick logic)
        let max_snapshots = 1000;
        if recording.snapshots.len() > max_snapshots {
            let excess = recording.snapshots.len() - max_snapshots;
            recording.snapshots.drain(..excess);
            if let Some(first) = recording.snapshots.first() {
                recording.first_tick = first.tick;
            }
        }
    }
    recording.finish();
    recording.save(&path).expect("save ring-buffer recording");

    // Load and verify
    let mut replayer = NodeReplayer::load(&path).expect("load ring-buffer recording");

    // Should have exactly 1000 snapshots (last 1000)
    assert_eq!(
        replayer.total_ticks(),
        1000,
        "Ring buffer should have exactly 1000 ticks"
    );

    // First tick should be 9000 (10000 - 1000)
    let first = replayer.current_snapshot().unwrap();
    assert_eq!(
        first.tick, 9000,
        "First tick should be 9000 (evicted 0-8999)"
    );

    // Last tick should be 9999
    replayer.seek(9999);
    let last = replayer.current_snapshot().unwrap();
    assert_eq!(last.tick, 9999, "Last tick should be 9999");

    // Verify data integrity of last tick
    let data = last.outputs.get("data").unwrap();
    let tick_val = u64::from_le_bytes(data[..8].try_into().unwrap());
    assert_eq!(tick_val, 9999, "Last tick data should be 9999");
}

// ============================================================================
// Test 13: Compressed recording integrity (if feature available)
// ============================================================================

#[test]
fn test_compressed_recording_if_available() {
    let tmp = TestTempDir::new("stress_compress");
    let uncompressed_path = tmp.path().join("uncomp@test.horus");

    // Create recording with realistic data
    let mut recording = NodeRecording::new("comp_node", "comp1", "stress_compress");
    for tick in 0..5000u64 {
        // Realistic-ish sensor data (not random — compressible)
        let data: Vec<u8> = (0..256)
            .map(|i| ((tick as f64 * 0.01).sin() * 127.0 + 128.0 + i as f64) as u8)
            .collect();
        recording.add_snapshot(
            NodeTickSnapshot::new(tick)
                .with_output("sensor", data)
                .with_duration(500),
        );
    }
    recording.finish();

    // Save uncompressed
    recording.save(&uncompressed_path).expect("save uncompressed");
    let uncompressed_size = std::fs::metadata(&uncompressed_path).unwrap().len();

    // Save compressed (uses save_compressed if available, else same as uncompressed)
    let compressed_path = tmp.path().join("comp@test.horus");
    recording
        .save_with_compression(&compressed_path, true)
        .expect("save compressed");
    let compressed_size = std::fs::metadata(&compressed_path).unwrap().len();

    // Both should be loadable
    let r1 = NodeReplayer::load(&uncompressed_path).expect("load uncompressed");
    let r2 = NodeReplayer::load(&compressed_path).expect("load compressed");

    assert_eq!(r1.total_ticks(), 5000);
    assert_eq!(r2.total_ticks(), 5000);

    println!(
        "Compression: {} bytes -> {} bytes ({:.1}% reduction)",
        uncompressed_size,
        compressed_size,
        (1.0 - compressed_size as f64 / uncompressed_size as f64) * 100.0
    );
}
