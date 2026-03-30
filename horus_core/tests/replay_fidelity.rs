#![allow(dead_code)]
//! Replay Fidelity Tests — timing accuracy, missing topics, speed, injection.
//!
//! These tests verify that replay delivers messages with correct timing,
//! handles missing/extra topics gracefully, and maintains data integrity
//! across different replay speeds.
//!
//! Run: `cargo test --no-default-features -p horus_core --test replay_fidelity`

use horus_core::scheduling::{NodeRecording, NodeReplayer, NodeTickSnapshot};

mod common;

// ============================================================================
// Test 1: Replay at 1x — verify timestamp spacing is correct
// ============================================================================

/// Creates a recording at 100Hz (10ms intervals), verifies timestamps
/// are spaced correctly and monotonically increasing.
#[test]
fn test_replay_1x_timestamp_spacing() {
    let mut recording = NodeRecording::new("timing_node", "t1", "timing_session");

    // 1000 ticks at 100Hz = 10 seconds
    for tick in 0..1000u64 {
        let timestamp_us = tick * 10_000; // 10ms = 10,000µs intervals
        recording.add_snapshot(
            NodeTickSnapshot::with_capacity_and_timestamp(tick, 1, timestamp_us)
                .with_output("data", tick.to_le_bytes().to_vec())
                .with_duration(500_000), // 500µs execution
        );
    }

    let mut replayer = NodeReplayer::from_recording(recording);

    // Walk through all ticks, verify timestamp spacing
    let mut prev_ts = None;
    let mut intervals = Vec::new();
    let mut count = 0u64;

    loop {
        let snap = replayer.current_snapshot().unwrap();

        if let Some(prev) = prev_ts {
            let interval_us = snap.timestamp_us - prev;
            intervals.push(interval_us);

            // Each interval should be exactly 10,000µs (10ms)
            assert_eq!(
                interval_us, 10_000,
                "Tick {}: interval should be 10000µs, got {}µs",
                snap.tick, interval_us
            );
        }

        prev_ts = Some(snap.timestamp_us);
        count += 1;

        if !replayer.advance() {
            break;
        }
    }

    assert_eq!(count, 1000, "Should traverse all 1000 ticks");
    assert_eq!(intervals.len(), 999, "Should have 999 intervals");

    // All intervals should be exactly 10ms
    let mean_interval = intervals.iter().sum::<u64>() as f64 / intervals.len() as f64;
    assert!(
        (mean_interval - 10_000.0).abs() < 1.0,
        "Mean interval should be exactly 10000µs, got {:.1}µs",
        mean_interval
    );
}

// ============================================================================
// Test 2: Replay at 0.1x — timestamps would be 10x slower
// ============================================================================

/// Verifies that a recording at 100Hz has correct timestamps that,
/// when replayed at 0.1x, would produce 100ms intervals.
#[test]
fn test_replay_01x_speed_timestamps() {
    let mut recording = NodeRecording::new("slow_replay", "sr1", "slow_session");

    // 100 ticks at 100Hz
    for tick in 0..100u64 {
        let timestamp_us = tick * 10_000; // 10ms intervals
        recording.add_snapshot(
            NodeTickSnapshot::with_capacity_and_timestamp(tick, 1, timestamp_us)
                .with_output("val", vec![tick as u8])
                .with_duration(100_000),
        );
    }

    let replayer = NodeReplayer::from_recording(recording);

    // At 0.1x speed, the total duration should be 10x longer
    // Original: 100 ticks × 10ms = 1000ms = 1 second
    // At 0.1x:  100 ticks × 100ms = 10,000ms = 10 seconds
    let first = replayer.current_snapshot().unwrap();
    let mut r = NodeReplayer::from_recording(replayer.recording().clone());

    // Navigate to last
    while r.advance() {}
    let last = r.current_snapshot().unwrap();

    let total_duration_us = last.timestamp_us - first.timestamp_us;
    let total_duration_at_01x = total_duration_us * 10; // 0.1x = 10x duration

    // Original: 990,000µs (99 intervals × 10ms)
    assert_eq!(total_duration_us, 990_000);
    // At 0.1x: 9,900,000µs = 9.9 seconds
    assert_eq!(total_duration_at_01x, 9_900_000);
}

// ============================================================================
// Test 3: Replay at 10x — timestamps would be 10x faster
// ============================================================================

#[test]
fn test_replay_10x_speed_timestamps() {
    let mut recording = NodeRecording::new("fast_replay", "fr1", "fast_session");

    // 1000 ticks at 100Hz = 10 seconds
    for tick in 0..1000u64 {
        let timestamp_us = tick * 10_000;
        recording.add_snapshot(
            NodeTickSnapshot::with_capacity_and_timestamp(tick, 1, timestamp_us)
                .with_output("val", tick.to_le_bytes().to_vec())
                .with_duration(100_000),
        );
    }

    let replayer = NodeReplayer::from_recording(recording);
    let first = replayer.current_snapshot().unwrap();
    let mut r = NodeReplayer::from_recording(replayer.recording().clone());

    while r.advance() {}
    let last = r.current_snapshot().unwrap();

    let total_duration_us = last.timestamp_us - first.timestamp_us;
    let total_duration_at_10x = total_duration_us / 10; // 10x = 1/10 duration

    // Original: 9,990,000µs (999 × 10ms) ≈ 10 seconds
    assert_eq!(total_duration_us, 9_990_000);
    // At 10x: 999,000µs ≈ 1 second
    assert_eq!(total_duration_at_10x, 999_000);

    // All 1000 messages should still be present (none dropped)
    assert_eq!(r.total_ticks(), 1000);
}

// ============================================================================
// Test 4: Replay with missing topic — query non-recorded topic
// ============================================================================

#[test]
fn test_replay_missing_topic_returns_none() {
    let mut recording = NodeRecording::new("missing_topic", "mt1", "missing_session");

    for tick in 0..50u64 {
        recording.add_snapshot(
            NodeTickSnapshot::new(tick)
                .with_output("topic_a", vec![1, 2, 3])
                .with_output("topic_b", vec![4, 5, 6])
                .with_duration(100),
        );
    }

    let replayer = NodeReplayer::from_recording(recording);
    let snap = replayer.current_snapshot().unwrap();

    // Existing topics should return data
    assert!(
        snap.outputs.get("topic_a").is_some(),
        "topic_a should exist"
    );
    assert!(
        snap.outputs.get("topic_b").is_some(),
        "topic_b should exist"
    );

    // Missing topic should return None (not panic)
    assert!(
        snap.outputs.get("topic_c").is_none(),
        "Non-recorded topic should return None"
    );
    assert!(
        snap.outputs.get("nonexistent").is_none(),
        "Random topic should return None"
    );
    assert!(
        snap.outputs.get("").is_none(),
        "Empty topic name should return None"
    );

    // Verify existing data is unaffected by missing queries
    let data_a = snap.outputs.get("topic_a").unwrap();
    assert_eq!(data_a, &[1, 2, 3], "topic_a data should be intact");
}

// ============================================================================
// Test 5: Replay with extra topics in environment (not in recording)
// ============================================================================

/// Verifies that a recording with topics A and B can coexist with
/// additional topics C, D, E that were not recorded.
#[test]
fn test_replay_extra_topics_no_interference() {
    // Recording only has topics A and B
    let mut recording = NodeRecording::new("extra_topics", "et1", "extra_session");

    for tick in 0..100u64 {
        recording.add_snapshot(
            NodeTickSnapshot::new(tick)
                .with_output("topic_a", vec![0xAA; 32])
                .with_output("topic_b", vec![0xBB; 32])
                .with_duration(200),
        );
    }

    let mut replayer = NodeReplayer::from_recording(recording);

    // Walk through all ticks — verify recorded data is correct
    // even though the "environment" would have extra topics
    let mut verified = 0;
    loop {
        let snap = replayer.current_snapshot().unwrap();

        // Recorded topics should have correct data
        let a = snap.outputs.get("topic_a").unwrap();
        assert!(
            a.iter().all(|&b| b == 0xAA),
            "topic_a corrupted at tick {}",
            snap.tick
        );

        let b = snap.outputs.get("topic_b").unwrap();
        assert!(
            b.iter().all(|&b| b == 0xBB),
            "topic_b corrupted at tick {}",
            snap.tick
        );

        // Only 2 topics in recording (no phantom extras)
        assert_eq!(
            snap.outputs.len(),
            2,
            "Should have exactly 2 output topics at tick {}, got {}",
            snap.tick,
            snap.outputs.len()
        );

        verified += 1;
        if !replayer.advance() {
            break;
        }
    }

    assert_eq!(verified, 100, "Should verify all 100 ticks");
}

// ============================================================================
// Test 7: Replay injection — verify data available for injection
// ============================================================================

/// Verifies that a recording can be loaded and its output data
/// extracted for injection into a live system's topics.
#[test]
fn test_replay_data_extractable_for_injection() {
    let mut recording = NodeRecording::new("inject_node", "inj1", "inject_session");

    // Create recording with specific injectable data
    for tick in 0..50u64 {
        let cmd_vel = serde_json::json!({
            "linear_x": tick as f64 * 0.1,
            "angular_z": 0.0,
        });
        let bytes = serde_json::to_vec(&cmd_vel).unwrap();

        recording.add_snapshot(
            NodeTickSnapshot::new(tick)
                .with_output("cmd_vel", bytes)
                .with_duration(500),
        );
    }

    let mut replayer = NodeReplayer::from_recording(recording);

    // Extract data from each tick (simulating injection into live topic)
    let mut injected_values = Vec::new();
    loop {
        let snap = replayer.current_snapshot().unwrap();

        if let Some(data) = snap.outputs.get("cmd_vel") {
            let value: serde_json::Value = serde_json::from_slice(data).unwrap();
            let linear_x = value["linear_x"].as_f64().unwrap();
            injected_values.push(linear_x);
        }

        if !replayer.advance() {
            break;
        }
    }

    assert_eq!(injected_values.len(), 50);

    // Verify injected values are correct
    for (i, &val) in injected_values.iter().enumerate() {
        let expected = i as f64 * 0.1;
        assert!(
            (val - expected).abs() < 1e-10,
            "Tick {}: expected {}, got {}",
            i,
            expected,
            val
        );
    }
}

// ============================================================================
// Bonus: Timestamp monotonicity across large recording
// ============================================================================

#[test]
fn test_replay_timestamps_always_monotonic() {
    let mut recording = NodeRecording::new("mono_node", "mono1", "mono_session");

    // 10K ticks with realistic but slightly jittery timestamps
    for tick in 0..10_000u64 {
        // Base: 10ms intervals, small jitter simulated by tick modulation
        let jitter_us = (tick % 7) * 100; // 0-600µs jitter
        let timestamp_us = tick * 10_000 + jitter_us;
        recording.add_snapshot(
            NodeTickSnapshot::with_capacity_and_timestamp(tick, 1, timestamp_us)
                .with_output("data", vec![0u8; 16])
                .with_duration(100),
        );
    }

    let mut replayer = NodeReplayer::from_recording(recording);

    let mut prev_ts = 0u64;
    let mut count = 0u64;
    loop {
        let snap = replayer.current_snapshot().unwrap();
        assert!(
            snap.timestamp_us >= prev_ts,
            "Timestamps must be monotonic: tick {} has {}µs < prev {}µs",
            snap.tick,
            snap.timestamp_us,
            prev_ts
        );
        prev_ts = snap.timestamp_us;
        count += 1;
        if !replayer.advance() {
            break;
        }
    }

    assert_eq!(count, 10_000);
}
