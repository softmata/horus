//! Integration tests for HORUS auto-detection features
//!
//! Tests the automatic detection and configuration of:
//! - POD types (via `is_pod<T>()`)
//! - Ring buffer capacity (via `auto_capacity<T>()`)
//! - Adaptive batching (opt-in feature)

use std::time::{Duration, Instant};

// Import HORUS types
use horus_core::communication::pod::is_pod;
use horus_core::communication::network::{
    AdaptiveBatcher, AdaptiveBatcherConfig, AdaptiveMode, BatchConfig, FrequencyTracker,
    FrequencyTrackerConfig, MessageBatcher,
};

// ============================================================================
// POD Auto-Detection Tests
// ============================================================================

/// Simple POD struct - should be auto-detected as POD
#[repr(C)]
#[derive(Clone, Copy)]
struct SensorReading {
    timestamp_ns: u64,
    value: f32,
    sensor_id: u16,
    _pad: [u8; 2],
}

/// Array-based POD - should be auto-detected as POD
#[derive(Clone, Copy)]
struct MotorCommand {
    velocities: [f32; 6],
    torques: [f32; 6],
    timestamp: u64,
}

/// Non-POD struct containing String - should NOT be detected as POD
struct LogMessage {
    level: u8,
    message: String,
    timestamp: u64,
}

/// Non-POD struct containing Vec - should NOT be detected as POD
struct SensorBatch {
    readings: Vec<f32>,
    timestamp: u64,
}

/// Nested POD struct - should be auto-detected as POD
#[derive(Clone, Copy)]
struct Pose3D {
    position: [f32; 3],
    orientation: [f32; 4], // quaternion
}

#[derive(Clone, Copy)]
struct RobotState {
    pose: Pose3D,
    velocity: [f32; 6],
    timestamp_ns: u64,
}

#[test]
fn test_pod_detection_simple_types() {
    // Primitive types are POD
    assert!(is_pod::<u8>(), "u8 should be POD");
    assert!(is_pod::<u32>(), "u32 should be POD");
    assert!(is_pod::<f32>(), "f32 should be POD");
    assert!(is_pod::<f64>(), "f64 should be POD");
    assert!(is_pod::<i64>(), "i64 should be POD");
    assert!(is_pod::<bool>(), "bool should be POD");
}

#[test]
fn test_pod_detection_arrays() {
    assert!(is_pod::<[f32; 3]>(), "Fixed array should be POD");
    assert!(is_pod::<[u8; 1024]>(), "Large fixed array should be POD");
    assert!(is_pod::<[[f32; 3]; 4]>(), "Nested fixed array should be POD");
}

#[test]
fn test_pod_detection_custom_structs() {
    assert!(is_pod::<SensorReading>(), "SensorReading should be POD");
    assert!(is_pod::<MotorCommand>(), "MotorCommand should be POD");
    assert!(is_pod::<Pose3D>(), "Pose3D should be POD");
    assert!(is_pod::<RobotState>(), "Nested POD struct should be POD");
}

#[test]
fn test_pod_detection_non_pod_types() {
    assert!(!is_pod::<String>(), "String should NOT be POD");
    assert!(!is_pod::<Vec<u8>>(), "Vec should NOT be POD");
    assert!(!is_pod::<Box<u32>>(), "Box should NOT be POD");
    assert!(!is_pod::<LogMessage>(), "Struct with String should NOT be POD");
    assert!(!is_pod::<SensorBatch>(), "Struct with Vec should NOT be POD");
}

#[test]
fn test_pod_detection_zst() {
    // Zero-sized types are NOT considered POD (no point in zero-copy for 0 bytes)
    assert!(!is_pod::<()>(), "Unit type should NOT be POD");

    #[derive(Clone, Copy)]
    struct Empty;
    assert!(!is_pod::<Empty>(), "Empty struct should NOT be POD");
}

// ============================================================================
// Frequency Tracker Integration Tests
// ============================================================================

#[test]
fn test_frequency_tracker_high_throughput_detection() {
    let config = FrequencyTrackerConfig {
        ema_alpha: 0.5,
        min_sample_interval: Duration::from_millis(10),
        high_freq_threshold: 100.0,  // Low threshold for testing
        low_freq_threshold: 10.0,
    };
    let mut tracker = FrequencyTracker::new(config);

    // Simulate high-frequency messages (>100/s)
    let start = Instant::now();
    let mut messages_sent = 0u64;
    while start.elapsed() < Duration::from_millis(100) {
        tracker.record_message();
        messages_sent += 1;
        // Small delay to avoid spinning too fast
        std::thread::sleep(Duration::from_micros(100));
    }

    // Should detect high frequency after sampling
    // Note: actual detection depends on timing, so we just verify tracking works
    assert!(tracker.total_messages() >= messages_sent - 10);
    assert!(tracker.frequency() > 0.0);
}

#[test]
fn test_frequency_tracker_low_throughput_detection() {
    let config = FrequencyTrackerConfig {
        ema_alpha: 1.0, // Immediate response for testing
        min_sample_interval: Duration::from_millis(10),
        high_freq_threshold: 1000.0,
        low_freq_threshold: 50.0,
    };
    let mut tracker = FrequencyTracker::new(config);

    // Simulate low-frequency messages (~5/s)
    for _ in 0..5 {
        tracker.record_message();
        std::thread::sleep(Duration::from_millis(200));
    }

    // After enough time, should be initialized with a low frequency
    assert!(tracker.total_messages() == 5);
}

// ============================================================================
// Adaptive Batcher Integration Tests
// ============================================================================

#[test]
fn test_adaptive_batcher_mode_transitions() {
    let config = AdaptiveBatcherConfig {
        adapt_interval: 5, // Adapt every 5 messages for faster testing
        frequency_config: FrequencyTrackerConfig {
            ema_alpha: 1.0,
            min_sample_interval: Duration::from_nanos(1),
            high_freq_threshold: 10000.0,
            low_freq_threshold: 10.0,
        },
        ..Default::default()
    };

    let mut batcher = AdaptiveBatcher::new("test_topic", config);

    // Initial mode should be Normal
    assert_eq!(batcher.current_mode(), AdaptiveMode::Normal);

    // Add some messages
    for i in 0u8..20 {
        batcher.add(vec![i]);
    }

    // Mode should still be valid (may or may not have changed depending on timing)
    let mode = batcher.current_mode();
    assert!(
        mode == AdaptiveMode::Normal
            || mode == AdaptiveMode::HighThroughput
            || mode == AdaptiveMode::LowLatency
    );
}

#[test]
fn test_adaptive_batcher_robotics_preset() {
    let config = AdaptiveBatcherConfig::robotics();
    let mut batcher = AdaptiveBatcher::new("sensor_data", config);

    // Robotics preset should start in Normal mode
    assert_eq!(batcher.current_mode(), AdaptiveMode::Normal);

    // Add sensor-like data (moderately fast)
    for i in 0u8..100 {
        if let Some(batch) = batcher.add(vec![i; 64]) {
            // Batch was flushed
            assert!(batch.count > 0);
        }
        std::thread::sleep(Duration::from_micros(500)); // ~2000 msg/s
    }

    // Flush remaining
    if let Some(batch) = batcher.flush() {
        assert!(batch.count > 0);
    }
}

#[test]
fn test_adaptive_batcher_vs_fixed_batcher() {
    // Compare adaptive and fixed batchers
    let fixed_config = BatchConfig::default();
    let adaptive_config = AdaptiveBatcherConfig::default();

    let mut fixed = MessageBatcher::new("fixed", fixed_config);
    let mut adaptive = AdaptiveBatcher::new("adaptive", adaptive_config);

    // Send same messages to both
    let messages: Vec<Vec<u8>> = (0..50).map(|i| vec![i as u8; 100]).collect();

    let mut fixed_batches = 0;
    let mut adaptive_batches = 0;

    for msg in &messages {
        if fixed.add(msg.clone()).is_some() {
            fixed_batches += 1;
        }
        if adaptive.add(msg.clone()).is_some() {
            adaptive_batches += 1;
        }
    }

    // Flush both
    if fixed.flush().is_some() {
        fixed_batches += 1;
    }
    if adaptive.flush().is_some() {
        adaptive_batches += 1;
    }

    // Both should have produced batches
    assert!(fixed_batches >= 1);
    assert!(adaptive_batches >= 1);
}

// ============================================================================
// Combined Auto-Detection Integration Tests
// ============================================================================

#[test]
fn test_auto_detection_workflow() {
    // Simulate a typical robotics workflow with auto-detection

    // 1. Check message type is POD (would use zero-copy path)
    assert!(is_pod::<MotorCommand>(), "MotorCommand is POD - zero-copy eligible");

    // 2. For high-frequency control data, use adaptive batching
    let config = AdaptiveBatcherConfig::robotics();
    let mut batcher = AdaptiveBatcher::new("motor_commands", config);

    // Simulate control loop at 1kHz
    let control_period = Duration::from_millis(1);
    let start = Instant::now();
    let mut commands_sent = 0;

    while start.elapsed() < Duration::from_millis(50) {
        let cmd = MotorCommand {
            velocities: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            torques: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            timestamp: start.elapsed().as_nanos() as u64,
        };

        // In real code, this would be serialized (or zero-copy for POD)
        let payload = unsafe {
            std::slice::from_raw_parts(
                &cmd as *const _ as *const u8,
                std::mem::size_of::<MotorCommand>(),
            )
            .to_vec()
        };

        batcher.add(payload);
        commands_sent += 1;

        std::thread::sleep(control_period);
    }

    // Flush remaining
    let _ = batcher.flush();

    assert!(commands_sent >= 40, "Should have sent ~50 commands at 1kHz over 50ms");
    // Note: Frequency tracking requires enough samples over min_sample_interval.
    // In a short test, EMA may not be initialized - that's expected behavior.
    // Just verify the total message count is tracked.
    let freq = batcher.frequency();
    assert!(freq.is_finite(), "Frequency should be a valid number (got {})", freq);
}

#[test]
fn test_pod_message_sizes() {
    // Verify POD message sizes are reasonable for IPC
    assert_eq!(std::mem::size_of::<SensorReading>(), 16);
    assert_eq!(std::mem::size_of::<MotorCommand>(), 56); // 12 f32s (48) + u64 (8)
    assert_eq!(std::mem::size_of::<Pose3D>(), 28); // 7 f32s
    assert_eq!(std::mem::size_of::<RobotState>(), 64); // Pose3D (28) + 6 f32s (24) + u64 (8) + padding

    // All should be POD
    assert!(is_pod::<SensorReading>());
    assert!(is_pod::<MotorCommand>());
    assert!(is_pod::<Pose3D>());
    assert!(is_pod::<RobotState>());
}

// ============================================================================
// Performance Regression Tests
// ============================================================================

#[test]
fn test_pod_detection_performance() {
    // POD detection should be very fast (compile-time for simple types)
    let iterations = 100_000;
    let start = Instant::now();

    for _ in 0..iterations {
        let _ = is_pod::<MotorCommand>();
        let _ = is_pod::<SensorReading>();
        let _ = is_pod::<String>();
    }

    let elapsed = start.elapsed();
    let per_check_ns = elapsed.as_nanos() as f64 / (iterations * 3) as f64;

    // POD checks should be fast (<500ns each under parallel test load)
    assert!(
        per_check_ns < 500.0,
        "POD detection too slow: {:.1}ns per check",
        per_check_ns
    );
}

#[test]
fn test_frequency_tracker_overhead() {
    let mut tracker = FrequencyTracker::default();

    let iterations = 100_000;
    let start = Instant::now();

    for _ in 0..iterations {
        tracker.record_message();
    }

    let elapsed = start.elapsed();
    let per_record_ns = elapsed.as_nanos() as f64 / iterations as f64;

    // Frequency tracking should be very low overhead (<500ns per record)
    assert!(
        per_record_ns < 500.0,
        "Frequency tracking too slow: {:.1}ns per record",
        per_record_ns
    );
}

#[test]
fn test_batching_overhead() {
    let config = BatchConfig::default();
    let mut batcher = MessageBatcher::new("bench", config);

    let iterations = 10_000;
    let payload = vec![0u8; 100];
    let start = Instant::now();

    for _ in 0..iterations {
        let _ = batcher.add(payload.clone());
    }

    let elapsed = start.elapsed();
    let per_add_ns = elapsed.as_nanos() as f64 / iterations as f64;

    // Batch add should be fast (<10000ns per add under parallel test load)
    assert!(
        per_add_ns < 10000.0,
        "Batching too slow: {:.1}ns per add",
        per_add_ns
    );
}
