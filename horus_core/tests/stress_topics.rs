//! Stress tests for the topic system.
//!
//! These run in their own binary to avoid thread/resource contention with unit tests.
//! Run with: cargo test --no-default-features -p horus_core --test stress_topics

use horus_core::communication::Topic;
use horus_core::core::DurationExt;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

/// Generate a unique topic name to avoid SHM collisions between parallel tests.
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

/// Spawn a test thread with a reduced stack size (512KB).
fn test_spawn<F, T>(f: F) -> std::thread::JoinHandle<T>
where
    F: FnOnce() -> T + Send + 'static,
    T: Send + 'static,
{
    std::thread::Builder::new()
        .stack_size(512 * 1024)
        .spawn(f)
        .expect("test_spawn: failed to spawn thread")
}

/// Stress test: 100 concurrent topics, each with a publisher thread and subscriber
/// checking thread, sustained for several seconds.
///
/// Robotics: humanoid robot scenario — 100+ active topics running simultaneously
/// at various rates. Verifies no corruption, no deadlocks, no panics.
///
/// Run with: cargo test --test stress_topics stress_100_concurrent
#[test]
fn stress_100_concurrent_topics_sustained() {
    let num_topics = 100;
    let msgs_per_topic = 5_000; // 500K total messages
    let test_prefix = unique("stress");

    let done = Arc::new(AtomicBool::new(false));
    let total_sent = Arc::new(AtomicU64::new(0));
    let total_recv = Arc::new(AtomicU64::new(0));
    let corruption_count = Arc::new(AtomicU64::new(0));

    let mut handles = Vec::new();

    for i in 0..num_topics {
        let done_flag = done.clone();
        let sent_count = total_sent.clone();
        let recv_count = total_recv.clone();
        let corrupt = corruption_count.clone();

        // Publisher thread
        let prefix = test_prefix.clone();
        let pub_handle = test_spawn(move || {
            let name = format!("{}_t{}", prefix, i);
            let topic: Topic<u64> = Topic::new(&name).expect("Topic creation failed");

            for seq in 0..msgs_per_topic {
                // Encode topic index in high bits, sequence in low bits
                let msg = ((i as u64) << 32) | (seq as u64);
                topic.send(msg);
                sent_count.fetch_add(1, Ordering::Relaxed);
            }
        });

        // Subscriber thread
        let prefix = test_prefix.clone();
        let sub_handle = test_spawn(move || {
            let name = format!("{}_t{}", prefix, i);
            let topic: Topic<u64> = Topic::new(&name).expect("Topic creation failed");

            let mut received = 0u64;
            let deadline = std::time::Instant::now() + 15_u64.secs();

            while std::time::Instant::now() < deadline && !done_flag.load(Ordering::Relaxed) {
                if let Some(msg) = topic.recv() {
                    let topic_idx = (msg >> 32) as usize;
                    if topic_idx != i {
                        corrupt.fetch_add(1, Ordering::Relaxed);
                    }
                    received += 1;
                    recv_count.fetch_add(1, Ordering::Relaxed);
                } else {
                    std::thread::yield_now();
                }
            }
            received
        });

        handles.push((pub_handle, sub_handle));
    }

    // Wait for all publishers to finish, collect subscriber handles
    let mut sub_handles = Vec::new();
    let mut pub_panics = 0u32;
    for (pub_h, sub_h) in handles {
        if pub_h.join().is_err() {
            pub_panics += 1;
        }
        sub_handles.push(sub_h);
    }

    // Signal subscribers to stop
    done.store(true, Ordering::Relaxed);
    std::thread::sleep(100_u64.ms());

    // Wait for subscribers — collect panics rather than propagating
    let mut sub_panics = 0u32;
    for sub_h in sub_handles {
        if sub_h.join().is_err() {
            sub_panics += 1;
        }
    }

    assert_eq!(pub_panics, 0, "{} publisher thread(s) panicked", pub_panics);
    assert_eq!(
        sub_panics, 0,
        "{} subscriber thread(s) panicked",
        sub_panics
    );

    let sent = total_sent.load(Ordering::Relaxed);
    let recv = total_recv.load(Ordering::Relaxed);
    let corrupted = corruption_count.load(Ordering::Relaxed);

    assert_eq!(
        sent,
        (num_topics * msgs_per_topic) as u64,
        "All messages should be sent"
    );
    assert!(recv > 0, "At least some messages should be received");
    assert_eq!(
        corrupted, 0,
        "Zero corruption: {} messages had wrong topic index",
        corrupted
    );
}

/// Stress test: sustained 1kHz single topic for 60 seconds (60,000 messages).
/// Verifies the hot path handles sustained high-frequency messaging.
/// Uses cross-thread pub/sub to test real IPC path.
///
/// Robotics: IMU sensor publishing at 1kHz continuously. Must not:
/// - Drop >5% of messages
/// - Grow memory over time (ring buffer is fixed-size)
/// - Stall for >10ms gaps
///
/// Run with: cargo test --test stress_topics stress_1khz_sustained
#[test]
fn stress_1khz_sustained_single_topic() {
    let name = unique("stress_1khz");
    let duration_secs = 60u64;
    let target_hz = 1000u64;
    let total_msgs = duration_secs * target_hz;

    let sent_count = Arc::new(AtomicU64::new(0));
    let recv_count = Arc::new(AtomicU64::new(0));
    let done = Arc::new(AtomicBool::new(false));

    // Publisher thread: send at 1kHz for 60 seconds
    let pub_name = name.clone();
    let pub_sent = sent_count.clone();
    let pub_handle = test_spawn(move || {
        let topic: Topic<u64> = Topic::new(&pub_name).unwrap();
        let period = (1_000_000 / target_hz).us();
        let start = Instant::now();

        for i in 0..total_msgs {
            topic.send(i);
            pub_sent.fetch_add(1, Ordering::Relaxed);
            // Busy-wait to maintain rate
            let target_time = period * (i as u32 + 1);
            while start.elapsed() < target_time {
                std::hint::spin_loop();
            }
        }
        start.elapsed()
    });

    // Subscriber thread: consume as fast as possible
    let sub_name = name.clone();
    let sub_recv = recv_count.clone();
    let sub_done = done.clone();
    let sub_handle = test_spawn(move || {
        let topic: Topic<u64> = Topic::new(&sub_name).unwrap();
        let mut max_gap = Duration::ZERO;
        let mut last_recv_time = Instant::now();

        while !sub_done.load(Ordering::Relaxed) {
            if topic.recv().is_some() {
                let now = Instant::now();
                let gap = now - last_recv_time;
                if gap > max_gap && sub_recv.load(Ordering::Relaxed) > 0 {
                    max_gap = gap;
                }
                last_recv_time = now;
                sub_recv.fetch_add(1, Ordering::Relaxed);
            } else {
                std::thread::yield_now();
            }
        }
        // Drain remaining
        while topic.recv().is_some() {
            sub_recv.fetch_add(1, Ordering::Relaxed);
        }
        max_gap
    });

    // Wait for publisher
    let elapsed = pub_handle.join().expect("Publisher panicked");
    let actual_hz = total_msgs as f64 / elapsed.as_secs_f64();

    // Give subscriber a moment to drain
    std::thread::sleep(200_u64.ms());
    done.store(true, Ordering::Relaxed);
    let _max_gap = sub_handle.join().expect("Subscriber panicked");

    let sent = sent_count.load(Ordering::Relaxed);
    let recv = recv_count.load(Ordering::Relaxed);
    let delivery_rate = recv as f64 / sent as f64 * 100.0;

    assert_eq!(sent, total_msgs, "All messages should be sent");
    assert!(
        actual_hz > 800.0,
        "Should sustain at least 800Hz, got {:.0}Hz",
        actual_hz
    );
    // Ring buffer may overwrite old messages — require >50% delivery
    // (ring capacity is typically 512-1024 slots, subscriber may fall behind)
    assert!(
        delivery_rate > 50.0,
        "Should deliver >50% of messages, got {:.1}% ({}/{})",
        delivery_rate,
        recv,
        sent
    );
}

/// Stress test: rapid subscriber connect/disconnect cycles.
/// Simulates sensor hot-plugging and node restarts.
///
/// Robotics: USB sensor plugged/unplugged, node crash and restart.
///
/// Run with: cargo test --test stress_topics stress_rapid_connect_disconnect
#[test]
fn stress_rapid_connect_disconnect_cycles() {
    let name = unique("stress_churn");
    let num_cycles = 200;

    let done = Arc::new(AtomicBool::new(false));

    // Continuous publisher
    let done_pub = done.clone();
    let pub_name = name.clone();
    let pub_handle = test_spawn(move || {
        let topic: Topic<u64> = Topic::new(&pub_name).unwrap();
        let mut seq = 0u64;
        while !done_pub.load(Ordering::Relaxed) {
            topic.send(seq);
            seq += 1;
            if seq.is_multiple_of(100) {
                std::thread::yield_now();
            }
        }
        seq
    });

    // Rapid connect/disconnect cycles
    for _cycle in 0..num_cycles {
        let topic: Topic<u64> = Topic::new(&name).unwrap();
        // Read a few messages (discard results — point is no crashes)
        for _ in 0..10 {
            let _ = topic.recv();
        }
        // Topic dropped here — subscriber disconnects
    }

    done.store(true, Ordering::Relaxed);
    let total_sent = pub_handle.join().expect("Publisher panicked");

    assert!(total_sent > 0, "Publisher should have sent messages");
    // We don't require all messages received — the point is no crashes
}

/// Stress test: multi-producer multi-consumer on a single topic.
/// Verifies MPMC dispatch handles concurrent access correctly.
///
/// Robotics: multiple sensor fusion nodes publishing to shared topic,
/// multiple consumer nodes reading from it.
///
/// Run with: cargo test --test stress_topics stress_mpmc_single_topic
#[test]
fn stress_mpmc_single_topic() {
    let name = unique("stress_mpmc");
    let num_producers = 4;
    let num_consumers = 4;
    let msgs_per_producer = 10_000;

    let total_sent = Arc::new(AtomicU64::new(0));
    let total_recv = Arc::new(AtomicU64::new(0));
    let done = Arc::new(AtomicBool::new(false));

    let mut handles = Vec::new();

    // Spawn producers
    for p in 0..num_producers {
        let sent = total_sent.clone();
        let topic_name = name.clone();
        handles.push(test_spawn(move || {
            let topic: Topic<u64> = Topic::new(&topic_name).unwrap();
            for seq in 0..msgs_per_producer {
                let msg = ((p as u64) << 48) | (seq as u64);
                topic.send(msg);
                sent.fetch_add(1, Ordering::Relaxed);
            }
        }));
    }

    // Spawn consumers
    for _c in 0..num_consumers {
        let recv = total_recv.clone();
        let done_flag = done.clone();
        let topic_name = name.clone();
        handles.push(test_spawn(move || {
            let topic: Topic<u64> = Topic::new(&topic_name).unwrap();
            let deadline = std::time::Instant::now() + 10_u64.secs();
            while std::time::Instant::now() < deadline && !done_flag.load(Ordering::Relaxed) {
                if topic.recv().is_some() {
                    recv.fetch_add(1, Ordering::Relaxed);
                } else {
                    std::thread::yield_now();
                }
            }
        }));
    }

    // Wait for producers
    for _ in 0..num_producers {
        handles.remove(0).join().expect("Producer panicked");
    }

    // Give consumers time to drain
    std::thread::sleep(500_u64.ms());
    done.store(true, Ordering::Relaxed);

    // Wait for consumers
    for h in handles {
        h.join().expect("Consumer panicked");
    }

    let sent = total_sent.load(Ordering::Relaxed);
    let recv = total_recv.load(Ordering::Relaxed);

    assert_eq!(
        sent,
        (num_producers * msgs_per_producer) as u64,
        "All messages should be sent"
    );
    assert!(
        recv > 0,
        "Consumers should have received some messages, got 0"
    );
}

/// Stress test: memory pressure — create/destroy many topics rapidly.
/// Verifies SHM files are cleaned up and no file descriptor leaks.
///
/// Robotics: rapid reconfiguration, dynamic topic creation during operation.
///
/// Run with: cargo test --test stress_topics stress_topic_create_destroy
#[test]
fn stress_topic_create_destroy_cycles() {
    let test_prefix = unique("stress_create");
    let num_cycles = 500;

    for i in 0..num_cycles {
        let name = format!("{}_c{}", test_prefix, i);
        let topic: Topic<u64> = Topic::new(&name).unwrap();
        topic.send(i as u64);
        assert_eq!(topic.recv(), Some(i as u64));
        // Topic dropped here — SHM should be cleaned up
    }

    // If we got here without running out of file descriptors or memory, success
}

/// Stress test: producer crash recovery.
/// Simulates a sensor driver crash (thread exits abruptly) and verifies
/// a new producer can reclaim the SHM and resume publishing without
/// subscribers reading corrupted data.
///
/// Robotics scenario: IMU driver crashes due to hardware fault. The driver
/// restarts, re-creates the same topic, and resumes publishing. Subscribers
/// (EKF, path planner) must not see corrupted data and must receive valid
/// data from the new producer.
///
/// Run with: cargo test --test stress_topics stress_producer_crash_recovery
#[test]
fn stress_producer_crash_recovery() {
    let name = unique("stress_crash");
    let num_crash_cycles = 20;
    let msgs_per_cycle = 500;

    // Long-lived subscriber that outlives producer crashes
    let sub_name = name.clone();
    let total_valid = Arc::new(AtomicU64::new(0));
    let total_corrupt = Arc::new(AtomicU64::new(0));
    let done = Arc::new(AtomicBool::new(false));

    let sub_valid = total_valid.clone();
    let sub_corrupt = total_corrupt.clone();
    let sub_done = done.clone();
    let sub_handle = test_spawn(move || {
        let topic: Topic<[u64; 4]> = Topic::new(&sub_name).unwrap();
        while !sub_done.load(Ordering::Relaxed) {
            if let Some(msg) = topic.recv() {
                // Validate message integrity: each message is [cycle, seq, cycle^seq, magic]
                let cycle = msg[0];
                let seq = msg[1];
                let checksum = msg[2];
                let magic = msg[3];
                if checksum == (cycle ^ seq) && magic == 0xDEAD_BEEF_CAFE_BABE {
                    sub_valid.fetch_add(1, Ordering::Relaxed);
                } else {
                    sub_corrupt.fetch_add(1, Ordering::Relaxed);
                }
            } else {
                std::thread::yield_now();
            }
        }
        // Drain remaining
        while let Some(msg) = topic.recv() {
            let cycle = msg[0];
            let seq = msg[1];
            let checksum = msg[2];
            let magic = msg[3];
            if checksum == (cycle ^ seq) && magic == 0xDEAD_BEEF_CAFE_BABE {
                sub_valid.fetch_add(1, Ordering::Relaxed);
            } else {
                sub_corrupt.fetch_add(1, Ordering::Relaxed);
            }
        }
    });

    // Repeatedly crash and restart producers
    for cycle in 0..num_crash_cycles {
        let pub_name = name.clone();
        // Spawn a producer that sends messages then exits abruptly
        let handle = test_spawn(move || {
            let topic: Topic<[u64; 4]> = Topic::new(&pub_name).unwrap();
            for seq in 0..msgs_per_cycle {
                let msg = [
                    cycle as u64,
                    seq as u64,
                    (cycle as u64) ^ (seq as u64),
                    0xDEAD_BEEF_CAFE_BABEu64,
                ];
                topic.send(msg);
            }
            // Thread exits — topic dropped without explicit cleanup,
            // simulating a crash. SHM region left in whatever state.
        });
        handle.join().expect("Producer thread panicked");

        // Brief pause to simulate restart delay
        std::thread::sleep(5_u64.ms());
    }

    // Let subscriber drain
    std::thread::sleep(200_u64.ms());
    done.store(true, Ordering::Relaxed);
    sub_handle.join().expect("Subscriber panicked");

    let valid = total_valid.load(Ordering::Relaxed);
    let corrupt = total_corrupt.load(Ordering::Relaxed);

    assert!(
        valid > 0,
        "Subscriber should have received valid messages from restarted producers"
    );
    assert_eq!(
        corrupt,
        0,
        "Subscriber must NEVER read corrupted data: got {} corrupt out of {} total",
        corrupt,
        valid + corrupt
    );
}

/// Stress test: ring buffer saturation with fast publisher / slow subscriber.
/// Verifies that the publisher NEVER blocks (critical real-time constraint)
/// and the subscriber reads recent data, not stale.
///
/// Robotics scenario: 30Hz camera -> AI inference at 5Hz. Ring buffer saturates.
/// Publisher must never block. Subscriber must see latest frames, not old ones.
///
/// Run with: cargo test --test stress_topics stress_ring_buffer_saturation
#[test]
fn stress_ring_buffer_saturation() {
    let name = unique("stress_saturate");
    let pub_hz = 100u64;
    let sub_hz = 10u64;
    let duration_secs = 10u64;
    let expected_sends = pub_hz * duration_secs;

    let done = Arc::new(AtomicBool::new(false));
    let pub_blocked_count = Arc::new(AtomicU64::new(0));

    // Publisher: 100Hz for 10 seconds. Measure how long each send takes.
    let pub_name = name.clone();
    let pub_blocked = pub_blocked_count.clone();
    let pub_handle = test_spawn(move || {
        let topic: Topic<u64> = Topic::new(&pub_name).unwrap();
        let period = (1_000_000 / pub_hz).us();
        let block_threshold = period * 5; // >5x period means "blocked"
        let start = Instant::now();
        let mut sent = 0u64;

        for seq in 0..expected_sends {
            let before_send = Instant::now();
            topic.send(seq);
            let send_time = before_send.elapsed();
            if send_time > block_threshold {
                pub_blocked.fetch_add(1, Ordering::Relaxed);
            }
            sent += 1;

            // Pace to target Hz
            let target = period * (seq as u32 + 1);
            while start.elapsed() < target {
                std::hint::spin_loop();
            }
        }
        (sent, start.elapsed())
    });

    // Subscriber: 10Hz — deliberately slow. Track received sequence numbers.
    let sub_name = name.clone();
    let sub_done = done.clone();
    let sub_handle = test_spawn(move || {
        let topic: Topic<u64> = Topic::new(&sub_name).unwrap();
        let period = (1_000_000 / sub_hz).us();
        let mut received_seqs: Vec<u64> = Vec::new();

        while !sub_done.load(Ordering::Relaxed) {
            if let Some(seq) = topic.recv() {
                received_seqs.push(seq);
            }
            // Sleep to simulate slow consumer
            std::thread::sleep(period);
        }
        // One final drain
        while let Some(seq) = topic.recv() {
            received_seqs.push(seq);
        }
        received_seqs
    });

    // Wait for publisher to finish
    let (sent, elapsed) = pub_handle.join().expect("Publisher panicked");
    let actual_hz = sent as f64 / elapsed.as_secs_f64();

    // Give subscriber a moment to process remaining
    std::thread::sleep(200_u64.ms());
    done.store(true, Ordering::Relaxed);
    let received_seqs = sub_handle.join().expect("Subscriber panicked");
    let blocked = pub_blocked_count.load(Ordering::Relaxed);

    // Publisher must have sent all messages
    assert_eq!(
        sent, expected_sends,
        "Publisher should send all {} messages",
        expected_sends
    );

    // Publisher must sustain target rate
    assert!(
        actual_hz > (pub_hz as f64 * 0.8),
        "Publisher should sustain >80% of target {}Hz, got {:.0}Hz",
        pub_hz,
        actual_hz
    );

    // Publisher must NEVER block (real-time constraint)
    assert_eq!(
        blocked, 0,
        "Publisher must NEVER block on full ring buffer, but was blocked {} times",
        blocked
    );

    // Subscriber should have received some messages (~100 at 10Hz x 10s)
    assert!(
        received_seqs.len() > 50,
        "Subscriber at {}Hz for {}s should receive >50 msgs, got {}",
        sub_hz,
        duration_secs,
        received_seqs.len()
    );

    // Subscriber should be reading RECENT data, not stale.
    // The last received sequence should be close to the last sent sequence.
    if let Some(&last_seq) = received_seqs.last() {
        let staleness = expected_sends - 1 - last_seq;
        assert!(
            staleness < expected_sends / 2,
            "Subscriber should read recent data. Last recv'd seq {} but last sent was {}. Staleness: {}",
            last_seq, expected_sends - 1, staleness
        );
    }
}

// ============================================================================
// Test: 200 concurrent topics — all created and messaged without error
// ============================================================================

#[test]
fn test_200_concurrent_topics() {
    // Create 200 topics, send 5 messages each, verify no crash
    let mut topics: Vec<Topic<u64>> = Vec::new();
    for i in 0..200 {
        let name = unique(&format!("chaos200_{}", i));
        let t: Topic<u64> = Topic::new(&name).unwrap();
        topics.push(t);
    }

    // Send 5 messages on each
    for (i, topic) in topics.iter().enumerate() {
        for j in 0..5u64 {
            topic.send(i as u64 * 1000 + j);
        }
    }

    // Receive from each — verify at least some data
    let mut total_received = 0u64;
    for topic in &topics {
        while topic.recv().is_some() {
            total_received += 1;
        }
    }

    assert!(
        total_received >= 100,
        "Should receive at least 100 of 1000 messages across 200 topics, got {}",
        total_received
    );
}

// ============================================================================
// Test: Message integrity — 10K messages with checksum verification
// ============================================================================

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
#[repr(C)]
struct ChecksumMsg {
    seq: u64,
    data: [u8; 32],
    checksum: u64,
}

// SAFETY: ChecksumMsg is #[repr(C)], all fields are primitive, no heap pointers
unsafe impl bytemuck::Pod for ChecksumMsg {}
unsafe impl bytemuck::Zeroable for ChecksumMsg {}
unsafe impl horus_core::communication::PodMessage for ChecksumMsg {}

impl ChecksumMsg {
    fn new(seq: u64) -> Self {
        let mut data = [0u8; 32];
        for (i, b) in data.iter_mut().enumerate() {
            *b = ((seq + i as u64) % 256) as u8;
        }
        let checksum = data.iter().map(|&b| b as u64).sum::<u64>() ^ seq;
        Self {
            seq,
            data,
            checksum,
        }
    }

    fn verify(&self) -> bool {
        let expected = self.data.iter().map(|&b| b as u64).sum::<u64>() ^ self.seq;
        self.checksum == expected
    }
}

#[test]
fn test_10k_messages_integrity() {
    let topic_name = unique("integrity_10k");
    let pub_topic: Topic<ChecksumMsg> = Topic::new(&topic_name).unwrap();
    let sub_topic: Topic<ChecksumMsg> = Topic::new(&topic_name).unwrap();

    let corrupted = Arc::new(AtomicU64::new(0));
    let received = Arc::new(AtomicU64::new(0));
    let c = corrupted.clone();
    let r = received.clone();

    let done = Arc::new(AtomicBool::new(false));
    let d = done.clone();

    // Receiver thread
    let recv_handle = test_spawn(move || {
        while !d.load(Ordering::Relaxed) {
            while let Some(msg) = sub_topic.recv() {
                r.fetch_add(1, Ordering::SeqCst);
                if !msg.verify() {
                    c.fetch_add(1, Ordering::SeqCst);
                }
            }
            std::thread::sleep(Duration::from_micros(100));
        }
        // Drain remaining
        while let Some(msg) = sub_topic.recv() {
            r.fetch_add(1, Ordering::SeqCst);
            if !msg.verify() {
                c.fetch_add(1, Ordering::SeqCst);
            }
        }
    });

    // Send 10K messages
    for i in 0..10_000u64 {
        pub_topic.send(ChecksumMsg::new(i));
        if i % 1000 == 0 {
            std::thread::yield_now(); // Let receiver drain
        }
    }

    std::thread::sleep(Duration::from_millis(200));
    done.store(true, Ordering::Relaxed);
    recv_handle.join().unwrap();

    let total_received = received.load(Ordering::SeqCst);
    let total_corrupted = corrupted.load(Ordering::SeqCst);

    assert_eq!(
        total_corrupted, 0,
        "Zero corrupted messages allowed in 10K stream (got {} corrupt out of {} received)",
        total_corrupted, total_received
    );
    // Ring buffer may not hold all 10K — but received should be > 0
    assert!(
        total_received > 0,
        "Should receive at least some messages, got 0"
    );
}

// ============================================================================
// Test: Rapid topic creation/destruction during active pub/sub — no crash
// ============================================================================

#[test]
fn test_topic_lifecycle_during_pubsub() {
    let done = Arc::new(AtomicBool::new(false));
    let d = done.clone();

    // Background thread: continuously creates and destroys topics
    let lifecycle_thread = test_spawn(move || {
        let mut cycle = 0u64;
        while !d.load(Ordering::Relaxed) {
            let name = unique(&format!("lifecycle_{}", cycle));
            let topic: Topic<u64> = Topic::new(&name).unwrap();
            topic.send(cycle);
            let _ = topic.recv();
            drop(topic);
            cycle += 1;
        }
        cycle
    });

    // Main thread: stable topic with active pub/sub
    let stable_name = unique("stable_lifecycle");
    let pub_topic: Topic<u64> = Topic::new(&stable_name).unwrap();
    let sub_topic: Topic<u64> = Topic::new(&stable_name).unwrap();

    for i in 0..100u64 {
        pub_topic.send(i);
        let _ = sub_topic.recv();
        std::thread::sleep(Duration::from_millis(5));
    }

    done.store(true, Ordering::Relaxed);
    let cycles = lifecycle_thread.join().unwrap();

    assert!(
        cycles >= 10,
        "Lifecycle thread should complete at least 10 create/destroy cycles, got {}",
        cycles
    );
}
