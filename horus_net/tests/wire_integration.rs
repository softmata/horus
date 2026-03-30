#![allow(dead_code)]
//! Integration tests for wire format + transport.
//!
//! Tests the full path: encode → send over UDP loopback → recv → decode.

use horus_net::priority::{Encoding, Priority, Reliability};
use horus_net::wire::*;
use std::net::UdpSocket;
use std::time::Duration;

/// Helper: create a pair of connected UDP sockets on loopback.
fn loopback_pair() -> (UdpSocket, UdpSocket, std::net::SocketAddr) {
    let a = UdpSocket::bind("127.0.0.1:0").unwrap();
    let b = UdpSocket::bind("127.0.0.1:0").unwrap();
    a.set_nonblocking(false).unwrap();
    b.set_nonblocking(false).unwrap();
    a.set_read_timeout(Some(Duration::from_millis(500)))
        .unwrap();
    b.set_read_timeout(Some(Duration::from_millis(500)))
        .unwrap();
    let b_addr = b.local_addr().unwrap();
    (a, b, b_addr)
}

#[test]
fn single_message_over_udp_loopback() {
    let (sender, receiver, recv_addr) = loopback_pair();

    let header = PacketHeader::new(PacketFlags::empty(), 0xABCD, 1);
    let msg = OutMessage {
        topic_name: "robot.imu".into(),
        topic_hash: topic_hash("robot.imu"),
        payload: vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16],
        timestamp_ns: 1_000_000_000,
        sequence: 42,
        priority: Priority::Normal,
        reliability: Reliability::None,
        encoding: Encoding::PodLe,
    };

    // Encode and send
    let mut send_buf = [0u8; 4096];
    let len = encode_single(&header, &msg, &mut send_buf);
    sender.send_to(&send_buf[..len], recv_addr).unwrap();

    // Receive and decode
    let mut recv_buf = [0u8; 4096];
    let (n, _from) = receiver.recv_from(&mut recv_buf).unwrap();

    let (decoded_header, decoded_msgs) = decode_packet(&recv_buf[..n]).unwrap();
    assert_eq!(decoded_header.magic, MAGIC);
    assert_eq!(decoded_header.sender_id_hash, 0xABCD);
    assert_eq!(decoded_header.packet_sequence, 1);
    assert_eq!(decoded_msgs.len(), 1);

    let m = &decoded_msgs[0];
    assert_eq!(m.topic_hash, topic_hash("robot.imu"));
    assert_eq!(m.payload, msg.payload);
    assert_eq!(m.timestamp_ns, 1_000_000_000);
    assert_eq!(m.sequence, 42);
    assert_eq!(m.priority, Priority::Normal);
    assert_eq!(m.reliability, Reliability::None);
    assert_eq!(m.encoding, Encoding::PodLe);
}

#[test]
fn batch_5_messages_over_udp_loopback() {
    let (sender, receiver, recv_addr) = loopback_pair();

    let header = PacketHeader::new(PacketFlags::empty().with(PacketFlags::BATCH), 0x1111, 99);
    let msgs: Vec<OutMessage> = (0..5u32)
        .map(|i| OutMessage {
            topic_name: format!("sensor.{i}"),
            topic_hash: topic_hash(&format!("sensor.{i}")),
            payload: vec![i as u8; 32],
            timestamp_ns: 5000 + i as u64,
            sequence: i,
            priority: Priority::Normal,
            reliability: Reliability::None,
            encoding: Encoding::PodLe,
        })
        .collect();

    let mut send_buf = [0u8; 4096];
    let len = encode_batch(&header, &msgs, &mut send_buf);
    sender.send_to(&send_buf[..len], recv_addr).unwrap();

    let mut recv_buf = [0u8; 4096];
    let (n, _) = receiver.recv_from(&mut recv_buf).unwrap();
    let (_, decoded_msgs) = decode_packet(&recv_buf[..n]).unwrap();

    assert_eq!(decoded_msgs.len(), 5);
    for (i, m) in decoded_msgs.iter().enumerate() {
        assert_eq!(m.topic_hash, topic_hash(&format!("sensor.{i}")));
        assert_eq!(m.payload.len(), 32);
        assert_eq!(m.payload[0], i as u8);
        assert_eq!(m.sequence, i as u32);
    }
}

#[test]
fn heartbeat_over_udp_loopback() {
    let (sender, receiver, recv_addr) = loopback_pair();

    let payload = HeartbeatPayload {
        peer_id: [0xDE; 16],
        heartbeat_sequence: 777,
    };
    let mut send_buf = [0u8; 64];
    let len = encode_heartbeat(0x2222, 50, &payload, &mut send_buf);
    assert_eq!(len, 32); // 12 + 20

    sender.send_to(&send_buf[..len], recv_addr).unwrap();

    let mut recv_buf = [0u8; 64];
    let (n, _) = receiver.recv_from(&mut recv_buf).unwrap();
    assert_eq!(n, 32);

    let header = PacketHeader::decode(&recv_buf).unwrap();
    assert!(header.flags.heartbeat());
    assert_eq!(header.sender_id_hash, 0x2222);

    let decoded = decode_heartbeat(&recv_buf).unwrap();
    assert_eq!(decoded.peer_id, [0xDE; 16]);
    assert_eq!(decoded.heartbeat_sequence, 777);
}

#[test]
fn ack_over_udp_loopback() {
    let (sender, receiver, recv_addr) = loopback_pair();

    let payload = AckPayload {
        acked_topic_hash: topic_hash("robot.estop"),
        acked_sequence: 1,
    };
    let mut send_buf = [0u8; 64];
    let len = encode_ack(0x3333, 10, &payload, &mut send_buf);
    assert_eq!(len, 20); // 12 + 8

    sender.send_to(&send_buf[..len], recv_addr).unwrap();

    let mut recv_buf = [0u8; 64];
    let (n, _) = receiver.recv_from(&mut recv_buf).unwrap();
    assert_eq!(n, 20);

    let header = PacketHeader::decode(&recv_buf).unwrap();
    assert!(header.flags.ack());

    let decoded = decode_ack(&recv_buf).unwrap();
    assert_eq!(decoded.acked_topic_hash, topic_hash("robot.estop"));
    assert_eq!(decoded.acked_sequence, 1);
}

#[test]
fn large_payload_over_udp() {
    let (sender, receiver, recv_addr) = loopback_pair();

    // ~1400 byte payload (just under typical MTU)
    let payload_data: Vec<u8> = (0..1400u16).map(|i| (i % 256) as u8).collect();

    let header = PacketHeader::new(PacketFlags::empty(), 0x4444, 1);
    let msg = OutMessage {
        topic_name: "camera.compressed".into(),
        topic_hash: topic_hash("camera.compressed"),
        payload: payload_data.clone(),
        timestamp_ns: 99,
        sequence: 1,
        priority: Priority::Bulk,
        reliability: Reliability::None,
        encoding: Encoding::Bincode,
    };

    let mut send_buf = [0u8; 4096];
    let len = encode_single(&header, &msg, &mut send_buf);
    sender.send_to(&send_buf[..len], recv_addr).unwrap();

    let mut recv_buf = [0u8; 4096];
    let (n, _) = receiver.recv_from(&mut recv_buf).unwrap();
    let (_, msgs) = decode_packet(&recv_buf[..n]).unwrap();

    assert_eq!(msgs.len(), 1);
    assert_eq!(msgs[0].payload, payload_data);
    assert_eq!(msgs[0].priority, Priority::Bulk);
    assert_eq!(msgs[0].encoding, Encoding::Bincode);
}

#[test]
fn zero_payload_message() {
    let header = PacketHeader::new(PacketFlags::empty(), 0, 0);
    let msg = OutMessage {
        topic_name: "heartbeat.node".into(),
        topic_hash: topic_hash("heartbeat.node"),
        payload: vec![],
        timestamp_ns: 0,
        sequence: 0,
        priority: Priority::Normal,
        reliability: Reliability::None,
        encoding: Encoding::PodLe,
    };

    let mut buf = [0u8; 256];
    let len = encode_single(&header, &msg, &mut buf);
    assert_eq!(len, 36); // 12 + 24 + 0 payload

    let (_, msgs) = decode_packet(&buf[..len]).unwrap();
    assert_eq!(msgs.len(), 1);
    assert!(msgs[0].payload.is_empty());
}

#[test]
fn all_priorities_and_reliabilities_roundtrip() {
    let priorities = [
        Priority::Immediate,
        Priority::RealTime,
        Priority::Normal,
        Priority::Bulk,
    ];
    let reliabilities = [
        Reliability::None,
        Reliability::Redundant,
        Reliability::Latched,
    ];

    for &p in &priorities {
        for &r in &reliabilities {
            let header = PacketHeader::new(PacketFlags::empty(), 0, 0);
            let msg = OutMessage {
                topic_name: "test".into(),
                topic_hash: topic_hash("test"),
                payload: vec![42],
                timestamp_ns: 0,
                sequence: 0,
                priority: p,
                reliability: r,
                encoding: Encoding::PodLe,
            };

            let mut buf = [0u8; 256];
            let len = encode_single(&header, &msg, &mut buf);
            let (_, msgs) = decode_packet(&buf[..len]).unwrap();

            assert_eq!(msgs[0].priority, p, "priority mismatch for {p:?}");
            assert_eq!(msgs[0].reliability, r, "reliability mismatch for {r:?}");
        }
    }
}

#[test]
fn fnv1a_hash_10k_no_collisions() {
    use std::collections::HashSet;
    let mut hashes = HashSet::new();
    for i in 0..10_000u32 {
        let name = format!("robot_{}.sensor_{}.data_{}", i / 100, (i / 10) % 10, i % 10);
        let h = topic_hash(&name);
        assert!(hashes.insert(h), "collision at index {i}: {name}");
    }
}

#[test]
fn fragment_header_roundtrip_over_udp() {
    let (sender, receiver, recv_addr) = loopback_pair();

    // Simulate a fragment packet: packet header + message header + fragment header + payload
    let pkt_header = PacketHeader::new(PacketFlags::empty().with(PacketFlags::FRAGMENT), 0x5555, 1);
    let frag = FragmentHeader {
        fragment_id: 42,
        fragment_index: 0,
        fragment_count: 3,
        total_payload_len: 4200,
    };

    let mut send_buf = [0u8; 256];
    pkt_header.encode(&mut send_buf[..PacketHeader::SIZE]);
    frag.encode(&mut send_buf[PacketHeader::SIZE..PacketHeader::SIZE + FragmentHeader::SIZE]);
    let payload = b"fragment zero data";
    let data_start = PacketHeader::SIZE + FragmentHeader::SIZE;
    send_buf[data_start..data_start + payload.len()].copy_from_slice(payload);
    let total = data_start + payload.len();

    sender.send_to(&send_buf[..total], recv_addr).unwrap();

    let mut recv_buf = [0u8; 256];
    let (n, _) = receiver.recv_from(&mut recv_buf).unwrap();

    let header = PacketHeader::decode(&recv_buf[..n]).unwrap();
    assert!(header.flags.fragment());

    let decoded_frag = FragmentHeader::decode(&recv_buf[PacketHeader::SIZE..]).unwrap();
    assert_eq!(decoded_frag.fragment_id, 42);
    assert_eq!(decoded_frag.fragment_index, 0);
    assert_eq!(decoded_frag.fragment_count, 3);
    assert_eq!(decoded_frag.total_payload_len, 4200);

    let recv_payload = &recv_buf[data_start..data_start + payload.len()];
    assert_eq!(recv_payload, payload);
}
