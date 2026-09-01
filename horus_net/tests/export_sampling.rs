//! Export sampling — what a replicated topic puts on the wire, and what it
//! admits to leaving behind.
//!
//! The export loop polls SHM on a 50 ms timer. Taking only the newest slot on
//! each poll pins every replicated topic at ~20 Hz however fast its publisher
//! runs, which is correct for state (a pose, a battery level) and wrong for a
//! measurement stream (odometry, joint states, lidar). Both modes exist here;
//! what neither is allowed to do is lose messages quietly.
//!
//! These drive the real SHM ring through `write_topic_slot_bytes`, which is the
//! cross-process contract `ShmRingReader` reads on a robot.

use horus_core::communication::{write_topic_slot_bytes, Topic};
use horus_net::config::NetConfig;
use horus_net::shm_reader::{ExportSampling, ShmRingReader};
use horus_robotics::CmdVel;
use horus_sys::shm::shm_topics_dir;

/// Ring capacity a `Topic<CmdVel>` gets: 4096-byte page / 16-byte message.
const RING_CAPACITY: u32 = 256;

fn unique_topic(base: &str) -> String {
    use std::sync::atomic::{AtomicU32, Ordering};
    static COUNTER: AtomicU32 = AtomicU32::new(0);
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("sampling_{base}_{id}_{}", std::process::id())
}

fn shm_path_for(topic_name: &str) -> std::path::PathBuf {
    shm_topics_dir().join(topic_name)
}

fn publish(path: &std::path::Path, linear: f32) {
    let cmd = CmdVel::new(linear, 0.0);
    // SAFETY: CmdVel is a POD message; this is the byte form the SHM ring holds.
    let bytes: &[u8] = unsafe {
        std::slice::from_raw_parts(
            &cmd as *const CmdVel as *const u8,
            std::mem::size_of::<CmdVel>(),
        )
    };
    assert!(
        write_topic_slot_bytes(path, bytes),
        "raw SHM publish must succeed"
    );
}

fn linear_of(msg: &horus_net::shm_reader::RawShmMessage) -> f32 {
    assert_eq!(msg.data.len(), std::mem::size_of::<CmdVel>());
    // SAFETY: the slot holds exactly the CmdVel bytes written above.
    let cmd: CmdVel = unsafe { std::ptr::read_unaligned(msg.data.as_ptr() as *const CmdVel) };
    cmd.linear
}

// ═══════════════════════════════════════════════════════════════════════════
// The burst must cross intact in stream mode
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn a_burst_crosses_intact_in_stream_mode() {
    let name = unique_topic("burst");
    let _topic: Topic<CmdVel> = Topic::new(&name).expect("create topic");
    let path = shm_path_for(&name);

    let mut reader = ShmRingReader::new(&name).with_sampling(ExportSampling::AllSlots);

    // The first poll starts the reader at "now" rather than replaying history.
    publish(&path, 0.0);
    let first = reader.read_pending(64);
    assert_eq!(first.len(), 1);
    assert_eq!(linear_of(&first[0]), 0.0);

    // A publisher running far faster than the export tick: 50 messages between
    // one poll and the next.
    const BURST: usize = 50;
    for i in 1..=BURST {
        publish(&path, i as f32);
    }

    let batch = reader.read_pending(64);
    assert_eq!(
        batch.len(),
        BURST,
        "every message published between two polls must cross, not just the newest"
    );
    let seen: Vec<f32> = batch.iter().map(linear_of).collect();
    let expected: Vec<f32> = (1..=BURST).map(|i| i as f32).collect();
    assert_eq!(seen, expected, "and in publish order, oldest first");

    // Sequence numbers are the ring's own write indices, so the receiving peer
    // can tell a gap from a reorder.
    let idx: Vec<u64> = batch.iter().map(|m| m.write_idx).collect();
    let expected_idx: Vec<u64> = (2..=(BURST as u64 + 1)).collect();
    assert_eq!(idx, expected_idx);

    assert_eq!(
        reader.skipped_total(),
        0,
        "nothing was skipped, so nothing may be reported as skipped"
    );
    assert_eq!(reader.forwarded(), BURST as u64 + 1);
    assert!(
        reader.due_report(std::time::Instant::now()).is_none(),
        "a topic that lost nothing must not cry wolf"
    );
}

#[test]
fn a_bounded_batch_resumes_on_the_next_poll_rather_than_dropping() {
    let name = unique_topic("resume");
    let _topic: Topic<CmdVel> = Topic::new(&name).expect("create topic");
    let path = shm_path_for(&name);

    let mut reader = ShmRingReader::new(&name).with_sampling(ExportSampling::AllSlots);
    publish(&path, 0.0);
    assert_eq!(reader.read_pending(64).len(), 1);

    // More than one poll's worth, but well inside the ring.
    for i in 1..=200 {
        publish(&path, i as f32);
    }

    let mut seen: Vec<f32> = Vec::new();
    for _ in 0..10 {
        let batch = reader.read_pending(64);
        assert!(batch.len() <= 64, "the per-poll bound must be honoured");
        if batch.is_empty() {
            break;
        }
        seen.extend(batch.iter().map(linear_of));
    }

    let expected: Vec<f32> = (1..=200).map(|i| i as f32).collect();
    assert_eq!(
        seen, expected,
        "the bound caps a poll's work; it must not lose the remainder"
    );
    assert_eq!(reader.skipped_total(), 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// The skipped count must be non-zero and correct in latest-only mode
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn latest_only_export_counts_every_sample_it_drops() {
    let name = unique_topic("decimated");
    let _topic: Topic<CmdVel> = Topic::new(&name).expect("create topic");
    let path = shm_path_for(&name);

    // The shipped default: no per-topic configuration at all.
    let mut reader = ShmRingReader::new(&name);
    assert_eq!(reader.sampling(), ExportSampling::LatestOnly);

    publish(&path, 0.0);
    assert_eq!(reader.read_pending(64).len(), 1);
    assert_eq!(
        reader.skipped_decimated(),
        0,
        "starting at 'now' is not decimation"
    );

    // One export tick's worth of a 500 Hz publisher: 25 messages per 50 ms.
    for i in 1..=25 {
        publish(&path, i as f32);
    }

    let batch = reader.read_pending(64);
    assert_eq!(
        batch.len(),
        1,
        "latest-only still takes the newest slot only"
    );
    assert_eq!(linear_of(&batch[0]), 25.0, "and it is the freshest one");
    assert_eq!(
        reader.skipped_decimated(),
        24,
        "the 24 messages this topic published and never exported must be counted"
    );
    assert_eq!(reader.skipped_lapped(), 0, "the ring never lapped");
    assert_eq!(reader.skipped_total(), 24);

    // Again, so the counter is cumulative rather than per-poll.
    for i in 26..=50 {
        publish(&path, i as f32);
    }
    assert_eq!(reader.read_pending(64).len(), 1);
    assert_eq!(reader.skipped_decimated(), 48);
}

#[test]
fn a_decimated_topic_says_so_on_the_console() {
    let name = unique_topic("loud");
    let _topic: Topic<CmdVel> = Topic::new(&name).expect("create topic");
    let path = shm_path_for(&name);

    let mut reader = ShmRingReader::new(&name);
    publish(&path, 0.0);
    reader.read_pending(64);
    for i in 1..=25 {
        publish(&path, i as f32);
    }
    reader.read_pending(64);

    let report = reader
        .due_report(std::time::Instant::now())
        .expect("a topic losing 24 of every 25 messages must not be silent");
    assert_eq!(report.topic, name);
    assert_eq!(report.decimated, 24);
    assert_eq!(report.forwarded, 2);
    assert_eq!(report.sampling, ExportSampling::LatestOnly);

    let msg = report.message();
    assert!(msg.contains(&name), "the line must name the topic: {msg}");
    assert!(msg.contains("24"), "and the count: {msg}");
    assert!(
        msg.contains("HORUS_NET_EXPORT_STREAM"),
        "and the way to stop it: {msg}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// A reader that falls a whole ring behind must climb out, and say what it lost
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn a_stream_reader_lapped_by_the_publisher_counts_the_loss_and_catches_up() {
    let name = unique_topic("lapped");
    let _topic: Topic<CmdVel> = Topic::new(&name).expect("create topic");
    let path = shm_path_for(&name);

    let mut reader = ShmRingReader::new(&name).with_sampling(ExportSampling::AllSlots);
    publish(&path, 0.0);
    assert_eq!(reader.read_pending(64).len(), 1);

    // Far more than the ring holds between two polls: the publisher overwrites
    // slots this reader has not reached. That is a real loss, and the point is
    // that it is counted rather than silently absent from the peer's stream.
    let burst = RING_CAPACITY as usize + 144;
    for i in 1..=burst {
        publish(&path, i as f32);
    }

    let mut delivered = 0u64;
    for _ in 0..64 {
        let batch = reader.read_pending(64);
        if batch.is_empty() && reader.last_write_idx() > burst as u64 {
            break;
        }
        delivered += batch.len() as u64;
    }

    assert!(
        reader.skipped_lapped() > 0,
        "the ring demonstrably lapped; the reader must not report a clean run"
    );
    assert_eq!(
        delivered + reader.skipped_lapped(),
        burst as u64,
        "every message published is either delivered or counted lost — no third \
         category, and no silent hole"
    );
    assert_eq!(
        reader.last_write_idx(),
        burst as u64 + 1,
        "a reader lapped by a whole ring must catch up to the head, not wedge \
         re-counting a window it can never reach"
    );

    let report = reader
        .due_report(std::time::Instant::now())
        .expect("a lapped stream must report");
    assert_eq!(report.decimated, 0, "stream mode does not decimate");
    assert!(report.lapped > 0);
    let msg = report.message();
    assert!(
        msg.contains("overwritten in the local ring"),
        "the operator must be pointed at the ring, not at the sampling mode: {msg}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// Config selects the mode per topic
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn config_picks_the_mode_per_topic() {
    let mut config = NetConfig::test_config(0);
    assert_eq!(
        config.export_sampling("odom"),
        ExportSampling::LatestOnly,
        "the default is unchanged: sampled, and now loud about it"
    );

    config.export_stream = vec!["odom".into(), "robot.*.scan".into()];
    assert_eq!(config.export_sampling("odom"), ExportSampling::AllSlots);
    assert_eq!(
        config.export_sampling("robot.front_lidar.scan"),
        ExportSampling::AllSlots
    );
    assert_eq!(
        config.export_sampling("battery"),
        ExportSampling::LatestOnly,
        "a pose or a battery level is right to sample"
    );
}

#[test]
fn a_per_topic_override_beats_the_pattern_list() {
    let mut config = NetConfig::test_config(0);
    config.export_stream = vec!["*".into()];
    config.topic_overrides.insert(
        "battery".into(),
        horus_net::config::TopicNetConfig {
            export_sampling: Some("latest".into()),
            ..Default::default()
        },
    );
    assert_eq!(config.export_sampling("odom"), ExportSampling::AllSlots);
    assert_eq!(
        config.export_sampling("battery"),
        ExportSampling::LatestOnly
    );

    // A typo must not quietly choose a mode on the operator's behalf; it falls
    // through to the pattern list, which here says stream.
    config.topic_overrides.insert(
        "imu".into(),
        horus_net::config::TopicNetConfig {
            export_sampling: Some("streem".into()),
            ..Default::default()
        },
    );
    assert_eq!(config.export_sampling("imu"), ExportSampling::AllSlots);
}

// ═══════════════════════════════════════════════════════════════════════════
// A publisher that restarts must not silently end the topic's replication
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn a_stream_reader_resynchronises_when_the_publisher_recreates_the_ring() {
    // A node restarts, its backing file is recreated, and the write index goes
    // back to 1 while the exporter's cursor sits well above it. Latest-only
    // export recovers from that by construction — any index that differs from
    // the cursor is new data. A streaming reader asks "what came after my
    // cursor?", and the honest answer is "nothing, ever again": the topic would
    // stop crossing the LAN with nothing logged and every counter clean, which
    // is the same silence this whole change exists to remove.
    let live = unique_topic("resync_live");
    let fresh = unique_topic("resync_fresh");
    let _live_topic: Topic<CmdVel> = Topic::new(&live).expect("create topic");
    let _fresh_topic: Topic<CmdVel> = Topic::new(&fresh).expect("create replacement topic");
    let live_path = shm_path_for(&live);
    let fresh_path = shm_path_for(&fresh);

    let mut reader = ShmRingReader::new(&live).with_sampling(ExportSampling::AllSlots);
    // The first poll starts the reader at "now"; the other 29 stream through.
    publish(&live_path, 0.0);
    let mut delivered = reader.read_pending(64).len();
    for i in 1..30 {
        publish(&live_path, i as f32);
    }
    for _ in 0..4 {
        delivered += reader.read_pending(64).len();
    }
    assert_eq!(delivered, 30);
    assert_eq!(reader.last_write_idx(), 30);

    // The publisher is replaced: same topic path, brand-new ring at index 0.
    std::fs::rename(&fresh_path, &live_path).expect("swap in the recreated ring");
    publish(&live_path, 99.0);

    let batch = reader.read_pending(64);
    assert_eq!(
        batch.len(),
        1,
        "a recreated ring must be picked up, not waited on forever"
    );
    assert_eq!(linear_of(&batch[0]), 99.0);
    assert_eq!(reader.last_write_idx(), 1);
}
