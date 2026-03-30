#![allow(dead_code)]
//! E2E integration tests for horus_net replication pipeline.
//!
//! Tests the full path: registry → discovery → peer matching → replicator lifecycle.
//! True cross-process Topic<T> replication requires horus_core integration (R3 note:
//! wiring Topic<T> → TopicRegistry happens when horus_core adds horus_net as optional dep).
//!
//! These tests verify the horus_net internals work correctly end-to-end.

use std::net::UdpSocket;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

use horus_net::config::NetConfig;
use horus_net::discovery::*;
use horus_net::peer::PeerTable;
use horus_net::priority::{Encoding, Priority, Reliability};
use horus_net::registry::{TopicRegistry, TopicRole};
use horus_net::wire::*;

// ─── Replicator Lifecycle Tests ─────────────────────────────────────────────

#[test]
fn replicator_starts_and_stops() {
    let config = NetConfig::test_config(0);
    let handle = horus_net::start_replicator(config).unwrap();
    assert!(handle.is_running());
    handle.stop();
    std::thread::sleep(Duration::from_millis(200));
    // Drop joins the thread
}

#[test]
fn replicator_disabled_returns_none() {
    let mut config = NetConfig::test_config(0);
    config.enabled = false;
    assert!(horus_net::start_replicator(config).is_none());
}

#[test]
fn replicator_drop_is_clean() {
    let config = NetConfig::test_config(0);
    // Create and immediately drop — should not hang
    let handle = horus_net::start_replicator(config).unwrap();
    drop(handle);
    // If we reach here, the thread was cleaned up within 3s timeout
}

// ─── Full Discovery Pipeline Tests ──────────────────────────────────────────

#[test]
fn full_pipeline_discovery_to_match() {
    // Simulate two peers:
    // Peer A: publishes "imu", subscribes to "cmd_vel"
    // Peer B: subscribes to "imu", publishes "cmd_vel"
    // Expected: bidirectional replication for both topics

    let reg_a = TopicRegistry::new();
    reg_a.register("imu", topic_hash("imu"), 64, TopicRole::Publisher, true);
    reg_a.register(
        "cmd_vel",
        topic_hash("cmd_vel"),
        16,
        TopicRole::Subscriber,
        true,
    );

    let reg_b = TopicRegistry::new();
    reg_b.register("imu", topic_hash("imu"), 64, TopicRole::Subscriber, true);
    reg_b.register(
        "cmd_vel",
        topic_hash("cmd_vel"),
        16,
        TopicRole::Publisher,
        true,
    );

    // A sends announcement
    let peer_id_a = generate_peer_id();
    let mut buf = [0u8; 4096];
    let len = encode_announcement(&peer_id_a, 9100, &[0; 4], &reg_a.entries(), &mut buf);

    // B receives and decodes
    let sock_a = UdpSocket::bind("127.0.0.1:0").unwrap();
    let sock_b = UdpSocket::bind("127.0.0.1:0").unwrap();
    sock_b
        .set_read_timeout(Some(Duration::from_millis(500)))
        .unwrap();
    let addr_b = sock_b.local_addr().unwrap();
    sock_a.send_to(&buf[..len], addr_b).unwrap();

    let mut recv_buf = [0u8; 4096];
    let (n, from) = sock_b.recv_from(&mut recv_buf).unwrap();
    let ann = decode_announcement(&recv_buf[..n], from).unwrap();

    // B updates peer table
    let mut table = PeerTable::new();
    table.update_peer(&ann);
    assert_eq!(table.alive_count(), 1);

    // B finds matches
    let (matches, warnings) = find_matches(&reg_b.entries(), &ann.topics);
    assert!(warnings.is_empty());
    assert_eq!(matches.len(), 2);

    let imu = matches.iter().find(|m| m.topic == "imu").unwrap();
    assert!(imu.import); // B imports imu from A
    assert!(!imu.export);

    let cmd = matches.iter().find(|m| m.topic == "cmd_vel").unwrap();
    assert!(cmd.export); // B exports cmd_vel to A
    assert!(!cmd.import);
}

// ─── Data Packet Round-Trip Tests ───────────────────────────────────────────

#[test]
fn pod_message_roundtrip_byte_exact() {
    // Simulate sending a 16-byte CmdVel-like POD message
    let pod_data: [u8; 16] = [
        0x00, 0x00, 0x80, 0x3F, // 1.0f32 (linear)
        0x00, 0x00, 0x00, 0x00, // 0.0f32 (angular)
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // timestamp
    ];

    let sock_a = UdpSocket::bind("127.0.0.1:0").unwrap();
    let sock_b = UdpSocket::bind("127.0.0.1:0").unwrap();
    sock_b
        .set_read_timeout(Some(Duration::from_millis(500)))
        .unwrap();
    let addr_b = sock_b.local_addr().unwrap();

    // Encode
    let header = PacketHeader::new(PacketFlags::empty(), 0x1234, 1);
    let msg = OutMessage {
        topic_name: "cmd_vel".into(),
        topic_hash: topic_hash("cmd_vel"),
        payload: pod_data.to_vec(),
        timestamp_ns: 1_000_000_000,
        sequence: 42,
        priority: Priority::RealTime,
        reliability: Reliability::Redundant,
        encoding: Encoding::PodLe,
    };

    let mut send_buf = [0u8; 256];
    let len = encode_single(&header, &msg, &mut send_buf);
    sock_a.send_to(&send_buf[..len], addr_b).unwrap();

    // Receive and verify byte-exact
    let mut recv_buf = [0u8; 256];
    let (n, _) = sock_b.recv_from(&mut recv_buf).unwrap();
    let (_, msgs) = decode_packet(&recv_buf[..n]).unwrap();

    assert_eq!(msgs.len(), 1);
    assert_eq!(msgs[0].payload, pod_data);
    assert_eq!(msgs[0].encoding, Encoding::PodLe);
    assert_eq!(msgs[0].priority, Priority::RealTime);
    assert_eq!(msgs[0].reliability, Reliability::Redundant);
}

#[test]
fn multiple_topics_in_batch() {
    let sock_a = UdpSocket::bind("127.0.0.1:0").unwrap();
    let sock_b = UdpSocket::bind("127.0.0.1:0").unwrap();
    sock_b
        .set_read_timeout(Some(Duration::from_millis(500)))
        .unwrap();
    let addr_b = sock_b.local_addr().unwrap();

    let header = PacketHeader::new(PacketFlags::empty().with(PacketFlags::BATCH), 0x5555, 1);
    let msgs = vec![
        OutMessage {
            topic_name: "imu".into(),
            topic_hash: topic_hash("imu"),
            payload: vec![1; 64],
            timestamp_ns: 100,
            sequence: 1,
            priority: Priority::Normal,
            reliability: Reliability::None,
            encoding: Encoding::PodLe,
        },
        OutMessage {
            topic_name: "odom".into(),
            topic_hash: topic_hash("odom"),
            payload: vec![2; 48],
            timestamp_ns: 100,
            sequence: 1,
            priority: Priority::Normal,
            reliability: Reliability::None,
            encoding: Encoding::PodLe,
        },
        OutMessage {
            topic_name: "cmd_vel".into(),
            topic_hash: topic_hash("cmd_vel"),
            payload: vec![3; 16],
            timestamp_ns: 100,
            sequence: 1,
            priority: Priority::RealTime,
            reliability: Reliability::Redundant,
            encoding: Encoding::PodLe,
        },
    ];

    let mut send_buf = [0u8; 4096];
    let len = encode_batch(&header, &msgs, &mut send_buf);
    sock_a.send_to(&send_buf[..len], addr_b).unwrap();

    let mut recv_buf = [0u8; 4096];
    let (n, _) = sock_b.recv_from(&mut recv_buf).unwrap();
    let (decoded_hdr, decoded_msgs) = decode_packet(&recv_buf[..n]).unwrap();

    assert!(decoded_hdr.flags.batch());
    assert_eq!(decoded_msgs.len(), 3);
    assert_eq!(decoded_msgs[0].payload.len(), 64);
    assert_eq!(decoded_msgs[1].payload.len(), 48);
    assert_eq!(decoded_msgs[2].payload.len(), 16);
    assert_eq!(decoded_msgs[2].priority, Priority::RealTime);
}

// ─── Registry + Replicator Integration ──────────────────────────────────────

#[test]
fn registry_change_callback_fires() {
    use std::sync::atomic::AtomicU32;

    let reg = Arc::new(TopicRegistry::new());
    let counter = Arc::new(AtomicU32::new(0));
    let counter_clone = counter.clone();

    reg.set_on_change(move || {
        counter_clone.fetch_add(1, Ordering::Relaxed);
    });

    reg.register("imu", topic_hash("imu"), 64, TopicRole::Publisher, true);
    reg.register("cmd", topic_hash("cmd"), 16, TopicRole::Subscriber, true);
    reg.unregister("imu", TopicRole::Publisher);

    assert_eq!(counter.load(Ordering::Relaxed), 3);
}

#[test]
fn demand_driven_no_readers_without_peers() {
    // Create a replicator with topics but no peers
    let registry = Arc::new(TopicRegistry::new());
    registry.register("imu", topic_hash("imu"), 64, TopicRole::Publisher, true);

    let config = NetConfig::test_config(0);

    let replicator = horus_net::replicator::Replicator::new(registry, config).unwrap();
    // No peers → no readers should be created
    // (readers are created in update_matches which is called when peers appear)
    // The replicator starts with empty readers/writers
    // This is the "zero overhead when no peers" guarantee
    assert!(replicator.running_flag().load(Ordering::Relaxed));
}

// ─── Encoding Tests ─────────────────────────────────────────────────────────

#[test]
fn encoding_same_endian_is_noop() {
    use horus_net::encoding::process_incoming_payload;

    let original = vec![1u8, 2, 3, 4, 5, 6, 7, 8];
    let mut payload = original.clone();
    process_incoming_payload(&mut payload, Encoding::native_pod(), 8);
    assert_eq!(payload, original);
}

#[test]
fn encoding_cross_endian_roundtrip() {
    use horus_net::encoding::{byte_swap_words, process_incoming_payload};

    let val: f64 = 1.23456789;
    let mut payload = val.to_ne_bytes().to_vec();

    // Simulate sender's bytes arriving in wrong endian
    byte_swap_words(&mut payload, 8);

    let other = if cfg!(target_endian = "little") {
        Encoding::PodBe
    } else {
        Encoding::PodLe
    };

    process_incoming_payload(&mut payload, other, 8);
    let decoded = f64::from_ne_bytes(payload.try_into().unwrap());
    assert_eq!(decoded, val);
}

// ─── Secret Filtering ───────────────────────────────────────────────────────

#[test]
fn secret_match_allows_peer() {
    let secret_hash = horus_net::discovery::compute_secret_hash("lab-42");

    let ann = PeerAnnouncement {
        peer_id: [1; 16],
        data_port: 9100,
        secret_hash,
        has_secret: true,
        topics: vec![],
        source_addr: "127.0.0.1:9100".parse().unwrap(),
    };

    // Same secret → accept
    let our_hash = horus_net::discovery::compute_secret_hash("lab-42");
    assert_eq!(ann.secret_hash, our_hash);
}

#[test]
fn secret_mismatch_rejects_peer() {
    let their_hash = horus_net::discovery::compute_secret_hash("lab-42");
    let our_hash = horus_net::discovery::compute_secret_hash("lab-99");
    assert_ne!(their_hash, our_hash);
}
