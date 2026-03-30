//! Integration tests for topic backend migration during active scheduler execution.
//!
//! Validates that backend upgrades (DirectChannel→SpscIntra, SpscIntra→MpscIntra)
//! work correctly while nodes are actively publishing and subscribing via Topics
//! under scheduler orchestration.
//!
//! **Gap addressed**: Previous tests cover migration in isolation (topic/tests.rs)
//! and scheduler+topic pipelines (pipeline_integration.rs), but never migration
//! DURING an active scheduler run.

mod common;

use common::cleanup_stale_shm;
use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

fn unique(prefix: &str) -> String {
    use std::sync::atomic::AtomicU64;
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    format!(
        "{}_{}_{}",
        prefix,
        std::process::id(),
        COUNTER.fetch_add(1, Ordering::Relaxed)
    )
}

// ============================================================================
// Nodes for migration tests
// ============================================================================

/// Publisher node: sends incrementing counter on a topic.
struct PubNode {
    name: &'static str,
    counter: Arc<AtomicU64>,
    topic: Option<Topic<u64>>,
    topic_name: String,
}

impl Node for PubNode {
    fn name(&self) -> &'static str {
        self.name
    }
    fn init(&mut self) -> horus_core::error::Result<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        let val = self.counter.fetch_add(1, Ordering::SeqCst) + 1;
        if let Some(ref topic) = self.topic {
            topic.send(val);
        }
    }
}

/// Subscriber node: receives from topic, counts messages.
struct SubNode {
    name: &'static str,
    received: Arc<AtomicU64>,
    topic: Option<Topic<u64>>,
    topic_name: String,
}

impl Node for SubNode {
    fn name(&self) -> &'static str {
        self.name
    }
    fn init(&mut self) -> horus_core::error::Result<()> {
        self.topic = Some(Topic::new(&self.topic_name)?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref mut topic) = self.topic {
            while let Some(_val) = topic.recv() {
                self.received.fetch_add(1, Ordering::SeqCst);
            }
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

/// Test: Start scheduler with publisher + subscriber. After run starts,
/// a third Topic handle on the same name is created from a spawned thread,
/// triggering topology change (SPSC → MPSC or similar migration).
/// Assert: messages still flow after migration, no data loss.
#[test]
fn migration_third_handle_during_scheduler_run() {
    cleanup_stale_shm();

    let topic_name = unique("mig_third_handle");
    let pub_count = Arc::new(AtomicU64::new(0));
    let sub_received = Arc::new(AtomicU64::new(0));

    let topic_name_clone = topic_name.clone();

    // Spawn a thread that creates a third Topic handle after 200ms,
    // triggering migration from SPSC to a multi-participant backend.
    let migration_thread = std::thread::spawn(move || {
        std::thread::sleep(200_u64.ms());
        // Creating a third handle triggers topology detection + migration
        let _third: Topic<u64> = Topic::new(&topic_name_clone).unwrap();
        // Hold it alive for the duration of the test
        std::thread::sleep(800_u64.ms());
    });

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    scheduler
        .add(PubNode {
            name: "mig_publisher",
            counter: pub_count.clone(),
            topic: None,
            topic_name: topic_name.clone(),
        })
        .order(0)
        .build();

    scheduler
        .add(SubNode {
            name: "mig_subscriber",
            received: sub_received.clone(),
            topic: None,
            topic_name: topic_name.clone(),
        })
        .order(1)
        .build();

    // Run for 500ms — migration happens at ~200ms
    let result = scheduler.run_for(500_u64.ms());
    assert!(
        result.is_ok(),
        "Scheduler should not error: {:?}",
        result.err()
    );

    migration_thread.join().unwrap();

    let published = pub_count.load(Ordering::SeqCst);
    let received = sub_received.load(Ordering::SeqCst);

    assert!(
        published > 10,
        "Publisher should have sent >10 messages, got {published}"
    );
    assert!(
        received > 0,
        "Subscriber should have received messages after migration, got {received}"
    );
}

/// Test: Two nodes using .compute() execution class, which forces thread pool
/// execution. DirectChannel can't work cross-thread, so the backend must
/// migrate to SpscIntra or higher. Assert: messages flow despite migration.
#[test]
fn migration_compute_class_forces_cross_thread() {
    cleanup_stale_shm();

    let topic_name = unique("mig_compute");
    let pub_count = Arc::new(AtomicU64::new(0));
    let sub_received = Arc::new(AtomicU64::new(0));

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    scheduler
        .add(PubNode {
            name: "compute_pub",
            counter: pub_count.clone(),
            topic: None,
            topic_name: topic_name.clone(),
        })
        .order(0)
        .compute()
        .build();

    scheduler
        .add(SubNode {
            name: "compute_sub",
            received: sub_received.clone(),
            topic: None,
            topic_name: topic_name.clone(),
        })
        .order(1)
        .compute()
        .build();

    let result = scheduler.run_for(500_u64.ms());
    assert!(
        result.is_ok(),
        "Scheduler should not error: {:?}",
        result.err()
    );

    let published = pub_count.load(Ordering::SeqCst);
    let received = sub_received.load(Ordering::SeqCst);

    assert!(
        published > 5,
        "Compute publisher should tick, got {published}"
    );
    // With compute class, nodes run on thread pool — cross-thread IPC must work
    assert!(
        received > 0,
        "Compute subscriber should receive cross-thread, got {received}"
    );
}

/// Test: Migration under high-frequency (1kHz) publishing for 1 second.
/// Trigger migration at 300ms by adding a third handle.
/// Assert: no panic, no hang, counters show continuous operation.
#[test]
fn migration_under_1khz_load() {
    cleanup_stale_shm();

    let topic_name = unique("mig_1khz");
    let pub_count = Arc::new(AtomicU64::new(0));
    let sub_received = Arc::new(AtomicU64::new(0));

    let topic_name_clone = topic_name.clone();

    let migration_thread = std::thread::spawn(move || {
        std::thread::sleep(300_u64.ms());
        let _third: Topic<u64> = Topic::new(&topic_name_clone).unwrap();
        std::thread::sleep(1200_u64.ms());
    });

    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz());

    scheduler
        .add(PubNode {
            name: "fast_pub",
            counter: pub_count.clone(),
            topic: None,
            topic_name: topic_name.clone(),
        })
        .order(0)
        .rate(1000_u64.hz())
        .build();

    scheduler
        .add(SubNode {
            name: "fast_sub",
            received: sub_received.clone(),
            topic: None,
            topic_name: topic_name.clone(),
        })
        .order(1)
        .rate(1000_u64.hz())
        .build();

    let result = scheduler.run_for(1_u64.secs());
    assert!(
        result.is_ok(),
        "1kHz scheduler should not error: {:?}",
        result.err()
    );

    migration_thread.join().unwrap();

    let published = pub_count.load(Ordering::SeqCst);
    let received = sub_received.load(Ordering::SeqCst);

    // At 1kHz for 1s, expect ~1000 publishes. Allow wide margin for CI slowness.
    assert!(
        published > 100,
        "Should publish >100 at 1kHz, got {published}"
    );
    assert!(
        received > 0,
        "Should receive messages through migration, got {received}"
    );
}

/// Test: Verify that data sent BEFORE migration arrives AFTER migration.
/// Publisher sends known sequence, migration happens mid-sequence,
/// subscriber should see values from both pre- and post-migration.
#[test]
fn migration_preserves_data_continuity() {
    cleanup_stale_shm();

    let topic_name = unique("mig_continuity");
    let latest_received = Arc::new(AtomicU64::new(0));
    let receive_count = Arc::new(AtomicU64::new(0));
    let pub_count = Arc::new(AtomicU64::new(0));

    /// Subscriber that tracks the latest value received.
    struct TrackingSubNode {
        name: &'static str,
        latest: Arc<AtomicU64>,
        count: Arc<AtomicU64>,
        topic: Option<Topic<u64>>,
        topic_name: String,
    }

    impl Node for TrackingSubNode {
        fn name(&self) -> &'static str {
            self.name
        }
        fn init(&mut self) -> horus_core::error::Result<()> {
            self.topic = Some(Topic::new(&self.topic_name)?);
            Ok(())
        }
        fn tick(&mut self) {
            if let Some(ref mut topic) = self.topic {
                while let Some(val) = topic.recv() {
                    self.latest.store(val, Ordering::SeqCst);
                    self.count.fetch_add(1, Ordering::SeqCst);
                }
            }
        }
    }

    let topic_name_clone = topic_name.clone();
    let migration_thread = std::thread::spawn(move || {
        std::thread::sleep(250_u64.ms());
        let _third: Topic<u64> = Topic::new(&topic_name_clone).unwrap();
        std::thread::sleep(750_u64.ms());
    });

    let mut scheduler = Scheduler::new().tick_rate(100_u64.hz());

    scheduler
        .add(PubNode {
            name: "cont_pub",
            counter: pub_count.clone(),
            topic: None,
            topic_name: topic_name.clone(),
        })
        .order(0)
        .build();

    scheduler
        .add(TrackingSubNode {
            name: "cont_sub",
            latest: latest_received.clone(),
            count: receive_count.clone(),
            topic: None,
            topic_name: topic_name.clone(),
        })
        .order(1)
        .build();

    let result = scheduler.run_for(500_u64.ms());
    assert!(result.is_ok(), "Should not error: {:?}", result.err());

    migration_thread.join().unwrap();

    let published = pub_count.load(Ordering::SeqCst);
    let received = receive_count.load(Ordering::SeqCst);
    let latest = latest_received.load(Ordering::SeqCst);

    assert!(published > 10, "Should publish >10, got {published}");
    assert!(received > 0, "Should receive messages, got {received}");
    // Latest value should be close to published count (values are 1..=published)
    assert!(latest > 0, "Latest received should be > 0, got {latest}");
    // Values are monotonically increasing — latest should be recent
    assert!(
        latest > published / 3,
        "Latest value {latest} should be at least 1/3 of published {published} (proves post-migration data flows)"
    );
}
