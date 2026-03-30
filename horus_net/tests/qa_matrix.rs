//! QA Matrix — exhaustive production testing for robotics deployment.
//!
//! Tests every combination that matters for safety-critical robots:
//! - Message type matrix (POD sizes, serialized, boundary sizes)
//! - Priority × Reliability matrix (all 12 combinations)
//! - Encoding matrix (PodLe, PodBe forced, Bincode)
//! - Failure modes (corrupt, truncated, wrong magic, type mismatch)
//! - Scale (50+ topics discovery, sequence wrapping)
//! - Fragmentation boundaries (exact MTU, MTU+1, max)

use std::net::UdpSocket;
use std::time::Duration;

use horus_core::communication::Topic;
use horus_robotics::{CmdVel, Imu, JointState, LaserScan, Odometry};
use horus_types::Pose2D;
use horus_net::discovery::*;
use horus_net::encoding::{byte_swap_words, process_incoming_payload, DecodeResult};
use horus_net::fragment::{Fragmenter, Reassembler, MAX_FRAGMENT_PAYLOAD, MAX_REASSEMBLY_SIZE};
use horus_net::guard::{ExportMode, ImportExportGuard, ImportMode};
use horus_net::optimize::{Optimizer, OptimizerChain};
use horus_net::optimize::delta::DeltaOptimizer;
use horus_net::optimize::fusion::FusionOptimizer;
use horus_net::priority::{Encoding, Priority, Reliability};
use horus_net::registry::{TopicEntry, TopicRegistry, TopicRole};
use horus_net::reliability::ReliabilityLayer;
use horus_net::wire::*;

fn udp_pair() -> (UdpSocket, UdpSocket, std::net::SocketAddr) {
    let a = UdpSocket::bind("127.0.0.1:0").unwrap();
    let b = UdpSocket::bind("127.0.0.1:0").unwrap();
    b.set_read_timeout(Some(Duration::from_millis(500))).unwrap();
    let addr = b.local_addr().unwrap();
    (a, b, addr)
}

fn roundtrip_bytes(payload: &[u8], priority: Priority, reliability: Reliability, encoding: Encoding) {
    let (sa, sb, addr) = udp_pair();
    let hdr = PacketHeader::new(PacketFlags::empty(), 0xAA, 1);
    let msg = OutMessage {
        topic_name: "test".into(),
        topic_hash: topic_hash("test"),
        payload: payload.to_vec(),
        timestamp_ns: 12345,
        sequence: 1,
        priority,
        reliability,
        encoding,
    };
    let mut buf = [0u8; 65536];
    let len = encode_single(&hdr, &msg, &mut buf);
    sa.send_to(&buf[..len], addr).unwrap();

    let mut recv = [0u8; 65536];
    let (n, _) = sb.recv_from(&mut recv).unwrap();
    let (_, msgs) = decode_packet(&recv[..n]).unwrap();
    assert_eq!(msgs.len(), 1);
    assert_eq!(msgs[0].payload, payload, "payload mismatch for {priority:?}/{reliability:?}/{encoding:?}");
    assert_eq!(msgs[0].priority, priority);
    assert_eq!(msgs[0].reliability, reliability);
    assert_eq!(msgs[0].encoding, encoding);
}

// ═══════════════════════════════════════════════════════════════════════════
// MESSAGE TYPE MATRIX
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn msg_type_cmdvel_16b() {
    let cmd = CmdVel::new(1.0, 0.5);
    let bytes = unsafe { std::slice::from_raw_parts(&cmd as *const _ as *const u8, std::mem::size_of::<CmdVel>()) };
    roundtrip_bytes(bytes, Priority::RealTime, Reliability::Redundant, Encoding::PodLe);
}

#[test]
fn msg_type_imu() {
    let bytes = vec![0xAA; std::mem::size_of::<Imu>()];
    roundtrip_bytes(&bytes, Priority::Normal, Reliability::None, Encoding::PodLe);
}

#[test]
fn msg_type_pose2d() {
    let bytes = vec![0xBB; std::mem::size_of::<Pose2D>()];
    roundtrip_bytes(&bytes, Priority::Normal, Reliability::None, Encoding::PodLe);
}

#[test]
fn msg_type_laserscan() {
    let bytes = vec![0xCC; std::mem::size_of::<LaserScan>()];
    roundtrip_bytes(&bytes, Priority::Normal, Reliability::None, Encoding::PodLe);
}

#[test]
fn msg_type_jointstate() {
    let bytes = vec![0xDD; std::mem::size_of::<JointState>()];
    roundtrip_bytes(&bytes, Priority::Normal, Reliability::None, Encoding::PodLe);
}

#[test]
fn msg_type_odometry() {
    let bytes = vec![0xEE; std::mem::size_of::<Odometry>()];
    roundtrip_bytes(&bytes, Priority::Normal, Reliability::None, Encoding::PodLe);
}

#[test]
fn msg_size_8_bytes() {
    roundtrip_bytes(&[42u8; 8], Priority::Normal, Reliability::None, Encoding::PodLe);
}

#[test]
fn msg_size_1_byte() {
    roundtrip_bytes(&[0xFF], Priority::Normal, Reliability::None, Encoding::PodLe);
}

#[test]
fn msg_size_0_bytes() {
    roundtrip_bytes(&[], Priority::Normal, Reliability::None, Encoding::PodLe);
}

#[test]
fn msg_size_1kb() {
    roundtrip_bytes(&vec![0x11; 1024], Priority::Normal, Reliability::None, Encoding::PodLe);
}

#[test]
fn msg_size_64kb() {
    roundtrip_bytes(&vec![0x22; 65000], Priority::Bulk, Reliability::None, Encoding::PodLe);
}

// ═══════════════════════════════════════════════════════════════════════════
// PRIORITY × RELIABILITY MATRIX (all 12 combinations)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn matrix_immediate_none() {
    roundtrip_bytes(&[1; 16], Priority::Immediate, Reliability::None, Encoding::PodLe);
}
#[test]
fn matrix_immediate_redundant() {
    roundtrip_bytes(&[2; 16], Priority::Immediate, Reliability::Redundant, Encoding::PodLe);
}
#[test]
fn matrix_immediate_latched() {
    roundtrip_bytes(&[3; 16], Priority::Immediate, Reliability::Latched, Encoding::PodLe);
}
#[test]
fn matrix_realtime_none() {
    roundtrip_bytes(&[4; 16], Priority::RealTime, Reliability::None, Encoding::PodLe);
}
#[test]
fn matrix_realtime_redundant() {
    roundtrip_bytes(&[5; 16], Priority::RealTime, Reliability::Redundant, Encoding::PodLe);
}
#[test]
fn matrix_realtime_latched() {
    roundtrip_bytes(&[6; 16], Priority::RealTime, Reliability::Latched, Encoding::PodLe);
}
#[test]
fn matrix_normal_none() {
    roundtrip_bytes(&[7; 16], Priority::Normal, Reliability::None, Encoding::PodLe);
}
#[test]
fn matrix_normal_redundant() {
    roundtrip_bytes(&[8; 16], Priority::Normal, Reliability::Redundant, Encoding::PodLe);
}
#[test]
fn matrix_normal_latched() {
    roundtrip_bytes(&[9; 16], Priority::Normal, Reliability::Latched, Encoding::PodLe);
}
#[test]
fn matrix_bulk_none() {
    roundtrip_bytes(&[10; 16], Priority::Bulk, Reliability::None, Encoding::PodLe);
}
#[test]
fn matrix_bulk_redundant() {
    roundtrip_bytes(&[11; 16], Priority::Bulk, Reliability::Redundant, Encoding::PodLe);
}
#[test]
fn matrix_bulk_latched() {
    roundtrip_bytes(&[12; 16], Priority::Bulk, Reliability::Latched, Encoding::PodLe);
}

// ═══════════════════════════════════════════════════════════════════════════
// ENCODING MATRIX
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn encoding_pod_le() {
    roundtrip_bytes(&[1; 64], Priority::Normal, Reliability::None, Encoding::PodLe);
}

#[test]
fn encoding_pod_be() {
    roundtrip_bytes(&[2; 64], Priority::Normal, Reliability::None, Encoding::PodBe);
}

#[test]
fn encoding_bincode() {
    roundtrip_bytes(&[3; 64], Priority::Normal, Reliability::None, Encoding::Bincode);
}

#[test]
fn encoding_forced_swap_roundtrip() {
    let val = 3.14159265f64;
    let mut payload = val.to_ne_bytes().to_vec();
    let other = if cfg!(target_endian = "little") { Encoding::PodBe } else { Encoding::PodLe };
    byte_swap_words(&mut payload, 8);
    process_incoming_payload(&mut payload, other, 8);
    let decoded = f64::from_ne_bytes(payload.try_into().unwrap());
    assert_eq!(decoded, val);
}

#[test]
fn encoding_forced_swap_f32_array() {
    let vals: Vec<f32> = (0..10).map(|i| i as f32 * 1.1).collect();
    let mut payload: Vec<u8> = vals.iter().flat_map(|v| v.to_ne_bytes()).collect();
    let original = payload.clone();

    let other = if cfg!(target_endian = "little") { Encoding::PodBe } else { Encoding::PodLe };
    byte_swap_words(&mut payload, 4);
    assert_ne!(payload, original);
    process_incoming_payload(&mut payload, other, 4);
    assert_eq!(payload, original);
}

// ═══════════════════════════════════════════════════════════════════════════
// FAILURE MODES
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn corrupt_packet_random_bytes() {
    let garbage: Vec<u8> = (0..100).map(|i| (i * 37 % 256) as u8).collect();
    assert!(decode_packet(&garbage).is_none());
}

#[test]
fn corrupt_packet_wrong_magic() {
    let mut buf = [0u8; 64];
    let hdr = PacketHeader::new(PacketFlags::empty(), 0, 0);
    hdr.encode(&mut buf);
    buf[0] = 0xFF; // Corrupt magic
    assert!(decode_packet(&buf).is_none());
}

#[test]
fn corrupt_packet_wrong_version() {
    let mut buf = [0u8; 64];
    let hdr = PacketHeader::new(PacketFlags::empty(), 0, 0);
    hdr.encode(&mut buf);
    buf[4] = 99; // Wrong version
    assert!(decode_packet(&buf).is_none());
}

#[test]
fn truncated_at_every_byte() {
    let hdr = PacketHeader::new(PacketFlags::empty(), 0xAA, 1);
    let msg = OutMessage {
        topic_name: "t".into(),
        topic_hash: 1,
        payload: vec![42; 8],
        timestamp_ns: 0,
        sequence: 1,
        priority: Priority::Normal,
        reliability: Reliability::None,
        encoding: Encoding::PodLe,
    };
    let mut buf = [0u8; 256];
    let len = encode_single(&hdr, &msg, &mut buf);

    // Try decoding at every truncation point — should never panic
    for cut in 0..len {
        let result = decode_packet(&buf[..cut]);
        // Some truncations return None, some return partial — but never panic
        if let Some((_, msgs)) = result {
            // If header decoded but message truncated, msgs should be empty
            if cut < PacketHeader::SIZE + MessageHeader::SIZE + 8 {
                assert!(msgs.is_empty() || msgs[0].payload.len() <= 8);
            }
        }
    }
}

#[test]
fn type_hash_mismatch_rejected() {
    let local = vec![TopicEntry {
        name: "imu".into(),
        type_hash: 111,
        type_size: 64,
        role: TopicRole::Subscriber,
        is_pod: true,
    }];
    let remote = vec![WireTopicEntry {
        name: "imu".into(),
        type_hash: 222,
        type_size: 64,
        role: 1,
        is_pod: 1,
        priority: 2,
    }];
    let (matches, warnings) = find_matches(&local, &remote);
    assert!(matches.is_empty());
    assert!(!warnings.is_empty());
}

#[test]
fn sequence_wrapping_at_u32_max() {
    let mut layer = ReliabilityLayer::new();
    // Near wrap point
    assert!(layer.is_new_message(0x1234, 100, u32::MAX - 1));
    assert!(layer.is_new_message(0x1234, 100, u32::MAX));
    assert!(layer.is_new_message(0x1234, 100, 0)); // Wrapped!
    assert!(layer.is_new_message(0x1234, 100, 1));
    // Old sequence after wrap should be rejected
    assert!(!layer.is_new_message(0x1234, 100, 0));
}

// ═══════════════════════════════════════════════════════════════════════════
// FRAGMENTATION BOUNDARIES
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn fragment_boundary_exact_mtu() {
    let mut f = Fragmenter::new();
    let msg = OutMessage {
        topic_name: "t".into(),
        topic_hash: 1,
        payload: vec![0; MAX_FRAGMENT_PAYLOAD],
        timestamp_ns: 0, sequence: 1,
        priority: Priority::Normal, reliability: Reliability::None, encoding: Encoding::PodLe,
    };
    let frags = f.fragment(&msg);
    assert_eq!(frags.len(), 1, "exactly MTU should NOT fragment");
}

#[test]
fn fragment_boundary_mtu_plus_1() {
    let mut f = Fragmenter::new();
    let msg = OutMessage {
        topic_name: "t".into(),
        topic_hash: 1,
        payload: vec![0; MAX_FRAGMENT_PAYLOAD + 1],
        timestamp_ns: 0, sequence: 1,
        priority: Priority::Normal, reliability: Reliability::None, encoding: Encoding::PodLe,
    };
    let frags = f.fragment(&msg);
    assert_eq!(frags.len(), 2, "MTU+1 should produce 2 fragments");

    let mut r = Reassembler::new();
    let mut result = None;
    for frag in frags { if let Some(m) = r.feed(frag) { result = Some(m); } }
    assert_eq!(result.unwrap().payload.len(), MAX_FRAGMENT_PAYLOAD + 1);
}

#[test]
fn fragment_boundary_max_size() {
    let mut f = Fragmenter::new();
    let msg = OutMessage {
        topic_name: "t".into(),
        topic_hash: 1,
        payload: vec![0; MAX_REASSEMBLY_SIZE],
        timestamp_ns: 0, sequence: 1,
        priority: Priority::Normal, reliability: Reliability::None, encoding: Encoding::PodLe,
    };
    let frags = f.fragment(&msg);
    assert!(!frags.is_empty());
    let expected = (MAX_REASSEMBLY_SIZE + MAX_FRAGMENT_PAYLOAD - 1) / MAX_FRAGMENT_PAYLOAD;
    assert_eq!(frags.len(), expected);

    let mut r = Reassembler::new();
    let mut result = None;
    for frag in frags { if let Some(m) = r.feed(frag) { result = Some(m); } }
    assert_eq!(result.unwrap().payload.len(), MAX_REASSEMBLY_SIZE);
}

#[test]
fn fragment_boundary_over_max_rejected() {
    let mut f = Fragmenter::new();
    let msg = OutMessage {
        topic_name: "t".into(),
        topic_hash: 1,
        payload: vec![0; MAX_REASSEMBLY_SIZE + 1],
        timestamp_ns: 0, sequence: 1,
        priority: Priority::Normal, reliability: Reliability::None, encoding: Encoding::PodLe,
    };
    assert!(f.fragment(&msg).is_empty());
}

// ═══════════════════════════════════════════════════════════════════════════
// DISCOVERY SCALE
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn discovery_50_topics() {
    let reg = TopicRegistry::new();
    for i in 0..50 {
        reg.register(
            &format!("robot.sensor_{i}"),
            topic_hash(&format!("Sensor{i}")),
            64,
            TopicRole::Publisher,
            true,
        );
    }
    let entries = reg.entries();
    assert_eq!(entries.len(), 50);

    let peer_id = generate_peer_id();
    let mut buf = [0u8; 16384];
    let len = encode_announcement(&peer_id, 9100, &[0; 4], &entries, &mut buf);
    assert!(len > 0);

    let ann = decode_announcement(&buf[..len], "127.0.0.1:9100".parse().unwrap()).unwrap();
    assert_eq!(ann.topics.len(), 50);
}

#[test]
fn discovery_100_topics() {
    let reg = TopicRegistry::new();
    for i in 0..100 {
        reg.register(
            &format!("robot.t_{i}"),
            topic_hash(&format!("Type{i}")),
            32,
            if i % 2 == 0 { TopicRole::Publisher } else { TopicRole::Subscriber },
            true,
        );
    }
    let entries = reg.entries();
    assert_eq!(entries.len(), 100);

    let peer_id = generate_peer_id();
    let mut buf = [0u8; 32768];
    let len = encode_announcement(&peer_id, 9100, &[0; 4], &entries, &mut buf);
    let ann = decode_announcement(&buf[..len], "127.0.0.1:9100".parse().unwrap()).unwrap();
    assert_eq!(ann.topics.len(), 100);
}

// ═══════════════════════════════════════════════════════════════════════════
// IMPORT GUARD MATRIX
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn guard_deny_blocks_all_10_topics() {
    let guard = ImportExportGuard::new(ImportMode::Deny, ExportMode::All, None);
    for i in 0..10 {
        assert!(!guard.allow_import(&format!("topic_{i}")));
    }
}

#[test]
fn guard_allowlist_exact() {
    let guard = ImportExportGuard::new(
        ImportMode::AllowList(vec!["cmd_vel".into(), "estop".into(), "nav_goal".into()]),
        ExportMode::All,
        None,
    );
    assert!(guard.allow_import("cmd_vel"));
    assert!(guard.allow_import("estop"));
    assert!(guard.allow_import("nav_goal"));
    assert!(!guard.allow_import("imu"));
    assert!(!guard.allow_import("odom"));
    assert!(!guard.allow_import("camera.rgb"));
}

#[test]
fn guard_denylist_glob_patterns() {
    let guard = ImportExportGuard::new(
        ImportMode::Deny,
        ExportMode::DenyList(vec![
            "camera.*".into(),
            "debug.*".into(),
            "internal.*".into(),
            "*.raw".into(),
        ]),
        None,
    );
    assert!(guard.allow_export("imu"));
    assert!(guard.allow_export("cmd_vel"));
    assert!(!guard.allow_export("camera.rgb"));
    assert!(!guard.allow_export("camera.depth"));
    assert!(!guard.allow_export("debug.profiling"));
    assert!(!guard.allow_export("internal.state"));
    assert!(!guard.allow_export("lidar.raw"));
}

// ═══════════════════════════════════════════════════════════════════════════
// OPTIMIZER UNDER LOAD
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn optimizer_chain_100_messages_no_crash() {
    let mut chain = OptimizerChain::from_config(&["fusion".into(), "delta".into()]);
    for i in 0..100u32 {
        let mut msgs = vec![OutMessage {
            topic_name: "sensor".into(),
            topic_hash: topic_hash("sensor"),
            payload: vec![i as u8; 64],
            timestamp_ns: i as u64 * 1000,
            sequence: i,
            priority: if i == 50 { Priority::Immediate } else { Priority::Normal },
            reliability: Reliability::None,
            encoding: Encoding::PodLe,
        }];
        chain.process_outgoing(&mut msgs);
        // Should never panic regardless of input
    }
}

#[test]
fn delta_with_real_jointstate_pattern() {
    let mut opt = DeltaOptimizer::with_keyframe_interval(50);

    // Simulate: 32-joint robot, only joints 0 and 1 moving
    let mut payload = vec![0u8; 512]; // 32 joints × 16 bytes each

    // Keyframe
    let mut m1 = vec![OutMessage {
        topic_name: "joints".into(),
        topic_hash: topic_hash("joints"),
        payload: payload.clone(),
        timestamp_ns: 0, sequence: 1,
        priority: Priority::Normal, reliability: Reliability::None, encoding: Encoding::PodLe,
    }];
    opt.on_outgoing(&mut m1);
    assert_eq!(m1.len(), 1);

    // Change only joints 0 and 1 (first 32 bytes)
    for b in &mut payload[0..32] { *b = 0xFF; }

    let mut m2 = vec![OutMessage {
        topic_name: "joints".into(),
        topic_hash: topic_hash("joints"),
        payload: payload.clone(),
        timestamp_ns: 0, sequence: 2,
        priority: Priority::Normal, reliability: Reliability::None, encoding: Encoding::PodLe,
    }];
    opt.on_outgoing(&mut m2);
    assert_eq!(m2.len(), 1);
    // Delta should be much smaller than 512 bytes
    assert!(m2[0].payload.len() < 512, "delta={} should be < 512", m2[0].payload.len());
}

// ═══════════════════════════════════════════════════════════════════════════
// FNV-1a HASH QUALITY
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn fnv1a_no_collisions_robotics_topics() {
    use std::collections::HashSet;
    let mut hashes = HashSet::new();
    let topics = [
        "robot.imu", "robot.odom", "robot.cmd_vel", "robot.scan",
        "robot.camera.rgb", "robot.camera.depth", "robot.camera.compressed",
        "robot.joint_state", "robot.battery", "robot.estop",
        "robot.tf", "robot.tf_static", "robot.nav_goal", "robot.nav_path",
        "robot.wrench", "robot.force", "robot.temperature",
        "fleet.pose", "fleet.status", "fleet.command",
    ];
    for t in &topics {
        let h = topic_hash(t);
        assert!(hashes.insert(h), "collision on {t}");
    }
}
