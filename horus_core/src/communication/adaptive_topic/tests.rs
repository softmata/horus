//! Tests for the adaptive topic system.

use super::*;
use std::mem;
use std::sync::atomic::Ordering;

use header::current_time_ms;

#[test]
fn test_local_state_size() {
    let size = mem::size_of::<LocalState>();
    println!(
        "LocalState size: {} bytes ({} cache lines)",
        size,
        size.div_ceil(64)
    );
    assert!(
        size <= 128,
        "LocalState should fit in 2 cache lines, got {} bytes",
        size
    );
}

#[test]
fn test_backend_mode_conversion() {
    assert_eq!(
        AdaptiveBackendMode::from(1),
        AdaptiveBackendMode::DirectChannel
    );
    assert_eq!(AdaptiveBackendMode::from(10), AdaptiveBackendMode::MpmcShm);
    assert_eq!(AdaptiveBackendMode::from(255), AdaptiveBackendMode::Unknown);
}

#[test]
fn test_thread_id_hash() {
    let id = std::thread::current().id();
    let hash1 = header::hash_thread_id(id);
    let hash2 = header::hash_thread_id(id);
    assert_eq!(hash1, hash2);
}

#[test]
fn test_current_time_ms() {
    let t1 = current_time_ms();
    std::thread::sleep(std::time::Duration::from_millis(10));
    let t2 = current_time_ms();
    assert!(t2 > t1);
}

#[test]
fn test_backend_mode_latency() {
    assert_eq!(AdaptiveBackendMode::DirectChannel.expected_latency_ns(), 3);
    assert_eq!(AdaptiveBackendMode::SpscIntra.expected_latency_ns(), 18);
    assert_eq!(AdaptiveBackendMode::MpmcShm.expected_latency_ns(), 167);
}

#[test]
fn test_participant_entry() {
    let entry = ParticipantEntry {
        pid: std::sync::atomic::AtomicU32::new(0),
        thread_id_hash: std::sync::atomic::AtomicU32::new(0),
        role: std::sync::atomic::AtomicU8::new(0),
        active: std::sync::atomic::AtomicU8::new(0),
        _pad: [0; 6],
        lease_expires_ms: std::sync::atomic::AtomicU64::new(0),
    };

    assert!(entry.is_empty());
    assert!(entry.is_lease_expired(current_time_ms()));
}

#[test]
fn test_participant_entry_size() {
    assert_eq!(mem::size_of::<ParticipantEntry>(), 24);
}

#[test]
fn test_topic_role() {
    assert!(!TopicRole::Unregistered.can_send());
    assert!(!TopicRole::Unregistered.can_recv());
    assert!(TopicRole::Producer.can_send());
    assert!(!TopicRole::Producer.can_recv());
    assert!(!TopicRole::Consumer.can_send());
    assert!(TopicRole::Consumer.can_recv());
    assert!(TopicRole::Both.can_send());
    assert!(TopicRole::Both.can_recv());
}

#[test]
fn test_migrator_creation() {
    let mut header = AdaptiveTopicHeader::zeroed();
    header.init(8, 4, true, 100, 8);

    let migrator = BackendMigrator::new(&header);
    assert_eq!(migrator.current_epoch(), 0);
    assert!(!migrator.is_migration_in_progress());
}

#[test]
fn test_migrator_with_custom_timeout() {
    let mut header = AdaptiveTopicHeader::zeroed();
    header.init(8, 4, true, 100, 8);

    let _migrator = BackendMigrator::with_drain_timeout(&header, 500);
}

#[test]
fn test_migration_not_needed() {
    let mut header = AdaptiveTopicHeader::zeroed();
    header.init(8, 4, true, 100, 8);
    header
        .backend_mode
        .store(AdaptiveBackendMode::MpmcShm as u8, Ordering::Release);

    let migrator = BackendMigrator::new(&header);
    let result = migrator.try_migrate(AdaptiveBackendMode::MpmcShm);
    assert_eq!(result, MigrationResult::NotNeeded);
}

#[test]
fn test_migration_success() {
    let mut header = AdaptiveTopicHeader::zeroed();
    header.init(8, 4, true, 100, 8);
    header
        .backend_mode
        .store(AdaptiveBackendMode::Unknown as u8, Ordering::Release);

    let migrator = BackendMigrator::new(&header);
    let result = migrator.try_migrate(AdaptiveBackendMode::SpscIntra);

    match result {
        MigrationResult::Success { new_epoch } => {
            assert_eq!(new_epoch, 1);
            assert_eq!(header.mode(), AdaptiveBackendMode::SpscIntra);
        }
        other => panic!("Expected Success, got {:?}", other),
    }
}

#[test]
fn test_migration_concurrent_lock() {
    let mut header = AdaptiveTopicHeader::zeroed();
    header.init(8, 4, true, 100, 8);

    assert!(header.try_lock_migration());
    assert!(header.migration_lock.load(Ordering::Acquire) != 0);

    let migrator = BackendMigrator::new(&header);
    let result = migrator.try_migrate(AdaptiveBackendMode::SpscIntra);
    assert_eq!(result, MigrationResult::AlreadyInProgress);

    header.unlock_migration();
    assert!(!migrator.is_migration_in_progress());
}

#[test]
fn test_migrator_is_optimal() {
    let mut header = AdaptiveTopicHeader::zeroed();
    header.init(8, 4, true, 100, 8);

    let optimal = header.detect_optimal_backend();
    header.backend_mode.store(optimal as u8, Ordering::Release);

    let migrator = BackendMigrator::new(&header);
    assert!(migrator.is_optimal());

    header
        .backend_mode
        .store(AdaptiveBackendMode::DirectChannel as u8, Ordering::Release);
    assert!(!migrator.is_optimal());
}

#[test]
fn test_migrator_stats() {
    let mut header = AdaptiveTopicHeader::zeroed();
    header.init(8, 4, true, 100, 8);
    header
        .backend_mode
        .store(AdaptiveBackendMode::SpscIntra as u8, Ordering::Release);
    header.migration_epoch.store(42, Ordering::Release);

    let migrator = BackendMigrator::new(&header);
    let stats = migrator.stats();

    assert_eq!(stats.current_mode, AdaptiveBackendMode::SpscIntra);
    assert_eq!(stats.current_epoch, 42);
    assert!(!stats.is_locked);
    assert_eq!(stats.publisher_count, 0);
    assert_eq!(stats.subscriber_count, 0);
}

#[test]
fn test_migrate_to_optimal() {
    let mut header = AdaptiveTopicHeader::zeroed();
    header.init(8, 4, true, 100, 8);
    header
        .backend_mode
        .store(AdaptiveBackendMode::Unknown as u8, Ordering::Release);

    let migrator = BackendMigrator::new(&header);
    let result = migrator.migrate_to_optimal();

    match result {
        MigrationResult::Success { .. } => {
            assert_eq!(header.mode(), header.detect_optimal_backend());
        }
        MigrationResult::NotNeeded => {
            // Already optimal - also acceptable
        }
        other => panic!("Unexpected result: {:?}", other),
    }
}

#[test]
fn test_epoch_increments_on_migration() {
    let mut header = AdaptiveTopicHeader::zeroed();
    header.init(8, 4, true, 100, 8);
    header
        .backend_mode
        .store(AdaptiveBackendMode::Unknown as u8, Ordering::Release);

    let migrator = BackendMigrator::new(&header);

    let result1 = migrator.try_migrate(AdaptiveBackendMode::SpscIntra);
    assert!(matches!(result1, MigrationResult::Success { new_epoch: 1 }));

    let result2 = migrator.try_migrate(AdaptiveBackendMode::MpmcShm);
    assert!(matches!(result2, MigrationResult::Success { new_epoch: 2 }));

    let result3 = migrator.try_migrate(AdaptiveBackendMode::SpscIntra);
    assert!(matches!(result3, MigrationResult::Success { new_epoch: 3 }));

    assert_eq!(migrator.current_epoch(), 3);
}

// ========================================================================
// AdaptiveTopic tests
// ========================================================================

#[test]
fn test_adaptive_topic_creation() {
    let topic: AdaptiveTopic<u64> =
        AdaptiveTopic::new("test_adaptive_create").expect("Failed to create topic");

    assert_eq!(topic.name(), "test_adaptive_create");
    assert_eq!(topic.role(), TopicRole::Unregistered);
    assert_eq!(
        topic
            .adaptive_metrics()
            .messages_sent
            .load(Ordering::Relaxed),
        0
    );
    assert_eq!(
        topic
            .adaptive_metrics()
            .messages_received
            .load(Ordering::Relaxed),
        0
    );
}

#[test]
#[ignore] // Flaky on CI - requires shared memory setup
fn test_adaptive_topic_send_registers_producer() {
    let topic: AdaptiveTopic<u64> =
        AdaptiveTopic::new("test_adaptive_send").expect("Failed to create topic");

    assert_eq!(topic.role(), TopicRole::Unregistered);

    let _ = topic.send(42u64);

    assert_eq!(topic.role(), TopicRole::Producer);
}

#[test]
fn test_adaptive_topic_recv_registers_consumer() {
    let topic: AdaptiveTopic<u64> =
        AdaptiveTopic::new("test_adaptive_recv").expect("Failed to create topic");

    assert_eq!(topic.role(), TopicRole::Unregistered);

    let result = topic.recv();
    assert!(result.is_none());

    assert_eq!(topic.role(), TopicRole::Consumer);
}

#[test]
#[ignore] // Flaky on CI - requires shared memory setup
fn test_adaptive_topic_send_recv_roundtrip() {
    let topic: AdaptiveTopic<u64> =
        AdaptiveTopic::new("test_adaptive_roundtrip").expect("Failed to create topic");

    let _ = topic.send(12345u64);

    let consumer = topic.clone();

    let received = consumer.recv();
    assert_eq!(received, Some(12345u64));
}

#[test]
fn test_adaptive_topic_multiple_messages() {
    let unique_name = format!("test_multi_{}", std::process::id());

    let topic: AdaptiveTopic<u32> =
        AdaptiveTopic::new(&unique_name).expect("Failed to create topic");

    for i in 0..10u32 {
        let _ = topic.send(i);

        let consumer = topic.clone();

        let received = consumer.recv();
        assert_eq!(received, Some(i), "Expected latest value {}", i);
    }

    let _ = topic.send(999u32);
    let consumer = topic.clone();
    let received = consumer.recv();
    assert_eq!(received, Some(999));

    let received = consumer.recv();
    assert!(received.is_none());
}

#[test]
fn test_separate_topic_instances() {
    let unique_name = format!("test_separate_{}", std::process::id());

    let publisher: AdaptiveTopic<f32> =
        AdaptiveTopic::new(&unique_name).expect("Failed to create publisher");

    let subscriber: AdaptiveTopic<f32> =
        AdaptiveTopic::new(&unique_name).expect("Failed to create subscriber");

    let pub_header = publisher.header();
    let sub_header = subscriber.header();
    assert_eq!(
        pub_header.magic, sub_header.magic,
        "Headers should have same magic"
    );

    let mut temperature = 20.0f32;

    temperature += 0.1;
    let _ = publisher.send(temperature);
    let received = subscriber.recv();
    assert_eq!(
        received,
        Some(20.1),
        "First recv should get 20.1, got {:?}",
        received
    );

    temperature += 0.1;
    let _ = publisher.send(temperature);
    let received = subscriber.recv();
    assert_eq!(
        received,
        Some(20.2),
        "Second recv should get 20.2, got {:?}",
        received
    );

    temperature += 0.1;
    let _ = publisher.send(temperature);
    let received = subscriber.recv();
    assert!(
        (received.unwrap() - 20.3).abs() < 0.001,
        "Third recv should get ~20.3, got {:?}",
        received
    );

    let sub_local = subscriber.local();
    assert_eq!(
        sub_local.cached_mode,
        AdaptiveBackendMode::DirectChannel,
        "Should use DirectChannel for same-thread separate instances"
    );
}

#[test]
fn test_adaptive_topic_has_message() {
    let topic: AdaptiveTopic<String> =
        AdaptiveTopic::new("test_adaptive_has_msg").expect("Failed to create topic");

    assert!(!topic.has_message());
    assert_eq!(topic.pending_count(), 0);

    let _ = topic.send("hello".to_string());

    assert!(topic.has_message() || topic.pending_count() > 0);

    let consumer = topic.clone();
    let _ = consumer.recv();

    let has_msg = topic.has_message();
    let pending = topic.pending_count();
    assert!(pending == 0 || has_msg);
}

#[test]
fn test_adaptive_topic_mode_detection() {
    let topic: AdaptiveTopic<u64> =
        AdaptiveTopic::new("test_adaptive_mode").expect("Failed to create topic");

    let _mode = topic.mode();

    let _ = topic.send(1);

    let _new_mode = topic.mode();

    let stats = topic.migration_stats();
    assert!(stats.publisher_count >= 1);
}

#[test]
fn test_adaptive_topic_clone() {
    let topic: AdaptiveTopic<u64> =
        AdaptiveTopic::new("test_adaptive_clone").expect("Failed to create topic");

    let _ = topic.send(100);

    let cloned = topic.clone();

    assert_eq!(cloned.role(), TopicRole::Unregistered);

    assert_eq!(
        topic
            .adaptive_metrics()
            .messages_sent
            .load(Ordering::Relaxed),
        cloned
            .adaptive_metrics()
            .messages_sent
            .load(Ordering::Relaxed)
    );

    let _ = cloned.send(200);
    assert_eq!(cloned.role(), TopicRole::Producer);
}

#[test]
#[ignore] // Flaky on CI - requires shared memory setup
fn test_adaptive_topic_metrics() {
    let topic: AdaptiveTopic<u64> =
        AdaptiveTopic::new("test_adaptive_metrics").expect("Failed to create topic");

    let metrics = topic.metrics();
    assert_eq!(metrics.messages_sent, 0);
    assert_eq!(metrics.messages_received, 0);

    let adaptive = topic.adaptive_metrics();
    assert_eq!(adaptive.migrations.load(Ordering::Relaxed), 0);

    for i in 0..5 {
        topic.send(i);
    }
}

#[test]
fn test_adaptive_topic_latency_validation() {
    let unique_name = format!("test_latency_{}", std::process::id());

    let topic: AdaptiveTopic<u64> =
        AdaptiveTopic::new(&unique_name).expect("Failed to create topic");

    for _ in 0..10 {
        let _ = topic.send(1u64);
        let consumer = topic.clone();
        let _ = consumer.recv();
    }

    let iterations = 100;
    let start = std::time::Instant::now();

    for i in 0..iterations {
        let _ = topic.send(i as u64);
        let consumer = topic.clone();
        let _ = consumer.recv();
    }

    let elapsed = start.elapsed();
    let avg_latency_ns = elapsed.as_nanos() / iterations as u128;

    eprintln!(
        "AdaptiveTopic latency: {}ns avg ({} iterations)",
        avg_latency_ns, iterations
    );
    eprintln!("Backend mode: {:?}", topic.mode());

    #[cfg(debug_assertions)]
    let threshold = 100000u128;
    #[cfg(not(debug_assertions))]
    let threshold = 5000u128;

    assert!(
        avg_latency_ns < threshold,
        "Latency {}ns exceeds {}ns threshold",
        avg_latency_ns,
        threshold
    );
}

#[test]
fn test_adaptive_topic_throughput() {
    let unique_name = format!("test_throughput_{}", std::process::id());

    let topic: AdaptiveTopic<u64> =
        AdaptiveTopic::new(&unique_name).expect("Failed to create topic");

    let iterations = 100;
    let start = std::time::Instant::now();

    for i in 0..iterations {
        let _ = topic.send(i as u64);
        let consumer = topic.clone();
        let msg = consumer.recv();
        assert_eq!(msg, Some(i as u64), "Message {} mismatch", i);
    }

    let elapsed = start.elapsed();

    let consumer = topic.clone();
    let received = consumer.recv();
    assert!(
        received.is_none() || received == Some((iterations - 1) as u64),
        "Topic should be empty or contain last value"
    );

    let ops_per_sec = (iterations as f64 * 2.0) / elapsed.as_secs_f64();
    eprintln!(
        "Throughput test: {} send+recv pairs in {:?} ({:.0} ops/sec)",
        iterations, elapsed, ops_per_sec
    );
}

#[test]
fn test_adaptive_topic_backend_names() {
    let modes = [
        AdaptiveBackendMode::Unknown,
        AdaptiveBackendMode::DirectChannel,
        AdaptiveBackendMode::SpscIntra,
        AdaptiveBackendMode::SpmcIntra,
        AdaptiveBackendMode::MpscIntra,
        AdaptiveBackendMode::MpmcIntra,
        AdaptiveBackendMode::PodShm,
        AdaptiveBackendMode::SpscShm,
        AdaptiveBackendMode::SpmcShm,
        AdaptiveBackendMode::MpscShm,
        AdaptiveBackendMode::MpmcShm,
    ];

    for mode in modes {
        let latency = mode.expected_latency_ns();
        assert!(latency > 0, "Mode {:?} has invalid latency", mode);
    }
}

#[test]
fn test_auto_capacity_small_messages() {
    assert_eq!(auto_capacity::<u64>(), 512);
    assert_eq!(auto_capacity::<u32>(), 1024);
    assert_eq!(auto_capacity::<u16>(), 1024);
    assert_eq!(auto_capacity::<u8>(), 1024);
}

#[test]
fn test_auto_capacity_medium_messages() {
    assert_eq!(auto_capacity::<[u8; 64]>(), 64);
    assert_eq!(auto_capacity::<[u8; 128]>(), 32);
}

#[test]
fn test_auto_capacity_large_messages() {
    assert_eq!(auto_capacity::<[u8; 1024]>(), 16);
    assert_eq!(auto_capacity::<[u8; 8192]>(), 16);
}

#[test]
fn test_auto_capacity_zero_sized() {
    assert_eq!(auto_capacity::<()>(), MIN_CAPACITY);
}

#[test]
fn test_auto_capacity_bounds() {
    assert!(auto_capacity::<u8>() >= MIN_CAPACITY);
    assert!(auto_capacity::<u8>() <= MAX_CAPACITY);
    assert!(auto_capacity::<[u8; 16384]>() >= MIN_CAPACITY);
    assert!(auto_capacity::<[u8; 16384]>() <= MAX_CAPACITY);
}
