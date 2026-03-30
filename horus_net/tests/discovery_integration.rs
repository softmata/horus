#![allow(dead_code)]
//! Integration tests for discovery — simulates two peers exchanging announcements
//! over UDP loopback and verifying topic matching.

use std::net::UdpSocket;
use std::time::Duration;

use horus_net::discovery::*;
use horus_net::peer::PeerTable;
use horus_net::registry::{TopicEntry, TopicRegistry, TopicRole};
use horus_net::wire::topic_hash;

fn make_entry(name: &str, role: TopicRole) -> TopicEntry {
    // type_hash is the hash of the TYPE name, not the topic name.
    // For testing, we use the topic name as a stand-in.
    TopicEntry {
        name: name.into(),
        type_hash: topic_hash(name),
        type_size: 64,
        role,
        is_pod: true,
        is_system: false,
    }
}

/// Simulate peer A sending an announcement, peer B receiving and matching.
#[test]
fn announcement_over_udp_loopback() {
    let sock_a = UdpSocket::bind("127.0.0.1:0").unwrap();
    let sock_b = UdpSocket::bind("127.0.0.1:0").unwrap();
    sock_b
        .set_read_timeout(Some(Duration::from_millis(500)))
        .unwrap();
    let addr_b = sock_b.local_addr().unwrap();

    // Peer A publishes imu
    let peer_id_a = generate_peer_id();
    let topics_a = vec![make_entry("imu", TopicRole::Publisher)];

    let mut buf = [0u8; 4096];
    let len = encode_announcement(&peer_id_a, 9100, &[0; 4], &topics_a, &mut buf);
    sock_a.send_to(&buf[..len], addr_b).unwrap();

    // Peer B receives
    let mut recv_buf = [0u8; 4096];
    let (n, from) = sock_b.recv_from(&mut recv_buf).unwrap();
    let ann = decode_announcement(&recv_buf[..n], from).unwrap();

    assert_eq!(ann.peer_id, peer_id_a);
    assert_eq!(ann.topics.len(), 1);
    assert_eq!(ann.topics[0].name, "imu");
}

/// Peer A publishes imu, Peer B subscribes — verify match found.
#[test]
fn topic_matching_over_udp() {
    let sock_a = UdpSocket::bind("127.0.0.1:0").unwrap();
    let sock_b = UdpSocket::bind("127.0.0.1:0").unwrap();
    sock_b
        .set_read_timeout(Some(Duration::from_millis(500)))
        .unwrap();
    let addr_b = sock_b.local_addr().unwrap();

    // A publishes imu
    let peer_id_a = generate_peer_id();
    let topics_a = vec![make_entry("imu", TopicRole::Publisher)];
    let mut buf = [0u8; 4096];
    let len = encode_announcement(&peer_id_a, 9100, &[0; 4], &topics_a, &mut buf);
    sock_a.send_to(&buf[..len], addr_b).unwrap();

    // B receives and decodes
    let mut recv_buf = [0u8; 4096];
    let (n, from) = sock_b.recv_from(&mut recv_buf).unwrap();
    let ann = decode_announcement(&recv_buf[..n], from).unwrap();

    // B subscribes to imu locally
    let local_b = vec![make_entry("imu", TopicRole::Subscriber)];
    let (matches, warnings) = find_matches(&local_b, &ann.topics);

    assert!(warnings.is_empty());
    assert_eq!(matches.len(), 1);
    assert!(matches[0].import); // B imports from A
    assert!(!matches[0].export);
}

/// Type hash mismatch — no match, warning produced.
#[test]
fn type_hash_mismatch_produces_warning() {
    let local = vec![TopicEntry {
        name: "imu".into(),
        type_hash: 111,
        type_size: 64,
        role: TopicRole::Subscriber,
        is_pod: true,
        is_system: false,
    }];
    let remote = vec![WireTopicEntry {
        name: "imu".into(),
        type_hash: 222, // Different!
        type_size: 64,
        role: 1,
        is_pod: 1,
        priority: 2,
    }];

    let (matches, warnings) = find_matches(&local, &remote);
    assert!(matches.is_empty());
    assert_eq!(warnings.len(), 1);
    assert!(warnings[0].contains("type mismatch"));
}

/// PeerTable tracks peers from announcements and detects death.
#[test]
fn peer_table_lifecycle() {
    let peer_id = generate_peer_id();
    let ann = PeerAnnouncement {
        peer_id,
        data_port: 9100,
        secret_hash: [0; 4],
        has_secret: false,
        topics: vec![WireTopicEntry {
            name: "imu".into(),
            type_hash: topic_hash("imu"),
            type_size: 64,
            role: 1,
            is_pod: 1,
            priority: 2,
        }],
        source_addr: "127.0.0.1:9100".parse().unwrap(),
    };

    let mut table = PeerTable::new();
    table.update_peer(&ann);
    assert_eq!(table.alive_count(), 1);
    assert!(table.has_remote_publishers("imu"));

    // Peer is still alive immediately
    let dead = table.check_liveness();
    assert!(dead.is_empty());
}

/// TopicRegistry register/unregister lifecycle.
#[test]
fn registry_lifecycle() {
    let reg = TopicRegistry::new();

    reg.register("imu", topic_hash("Imu"), 64, TopicRole::Publisher, true);
    assert_eq!(reg.len(), 1);
    assert!(reg.has_publishers("imu"));

    reg.register("imu", topic_hash("Imu"), 64, TopicRole::Subscriber, true);
    assert_eq!(reg.len(), 1); // Same topic, merged
    let entry = reg.get("imu").unwrap();
    assert_eq!(entry.role, TopicRole::Both);

    reg.unregister("imu", TopicRole::Publisher);
    let entry = reg.get("imu").unwrap();
    assert_eq!(entry.role, TopicRole::Subscriber);

    reg.unregister("imu", TopicRole::Subscriber);
    assert!(reg.is_empty());
}

/// Full round trip: registry → announcement → UDP → decode → peer table → match.
#[test]
fn full_discovery_pipeline() {
    // Peer A: registry with imu publisher + cmd_vel subscriber
    // Use topic name as type hash (consistent with make_entry helper)
    let reg_a = TopicRegistry::new();
    reg_a.register("imu", topic_hash("imu"), 64, TopicRole::Publisher, true);
    reg_a.register(
        "cmd_vel",
        topic_hash("cmd_vel"),
        16,
        TopicRole::Subscriber,
        true,
    );
    let entries_a = reg_a.entries();

    // Encode announcement
    let peer_id_a = generate_peer_id();
    let mut buf = [0u8; 4096];
    let len = encode_announcement(&peer_id_a, 9100, &[0; 4], &entries_a, &mut buf);

    // Send over UDP
    let sock_a = UdpSocket::bind("127.0.0.1:0").unwrap();
    let sock_b = UdpSocket::bind("127.0.0.1:0").unwrap();
    sock_b
        .set_read_timeout(Some(Duration::from_millis(500)))
        .unwrap();
    let addr_b = sock_b.local_addr().unwrap();
    sock_a.send_to(&buf[..len], addr_b).unwrap();

    // Receive and decode
    let mut recv_buf = [0u8; 4096];
    let (n, from) = sock_b.recv_from(&mut recv_buf).unwrap();
    let ann = decode_announcement(&recv_buf[..n], from).unwrap();

    // Update peer table
    let mut table = PeerTable::new();
    table.update_peer(&ann);
    assert_eq!(table.alive_count(), 1);

    // Peer B: subscribes to imu, publishes cmd_vel
    let local_b = vec![
        make_entry("imu", TopicRole::Subscriber),
        make_entry("cmd_vel", TopicRole::Publisher),
    ];

    // Match
    let (matches, warnings) = find_matches(&local_b, &ann.topics);
    assert!(warnings.is_empty());
    assert_eq!(matches.len(), 2);

    // imu: B imports from A
    let imu_match = matches.iter().find(|m| m.topic == "imu").unwrap();
    assert!(imu_match.import);
    assert!(!imu_match.export);

    // cmd_vel: B exports to A
    let cmd_match = matches.iter().find(|m| m.topic == "cmd_vel").unwrap();
    assert!(cmd_match.export);
    assert!(!cmd_match.import);
}
