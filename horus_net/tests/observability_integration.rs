//! Observability integration tests for horus_net modules.
//!
//! Tests e-stop encode/decode, presence broadcast, and log replication
//! encode/decode paths.
//!
//! Run: `cargo test --no-default-features -p horus_net --test observability_integration`

use std::time::{Duration, Instant};

// ============================================================================
// E-stop encode/decode roundtrip
// ============================================================================

#[test]
fn test_estop_encode_decode_roundtrip() {
    use horus_net::estop::{decode_estop, encode_estop};

    let peer_id_hash = 0xABCD_u16;
    let reason = "Critical safety violation: motor current exceeded 15A";

    let start = Instant::now();
    let encoded = encode_estop(peer_id_hash, reason);
    let decoded = decode_estop(&encoded);
    let elapsed = start.elapsed();

    let (host_id, decoded_reason, timestamp) = decoded.expect("decode should succeed");
    assert_eq!(host_id, peer_id_hash);
    assert_eq!(decoded_reason, reason);
    assert!(timestamp > 0);
    assert!(elapsed < Duration::from_millis(1), "< 1ms: {:?}", elapsed);
}

#[test]
fn test_estop_malformed_data_no_panic() {
    use horus_net::estop::decode_estop;

    assert!(decode_estop(&[]).is_none());
    assert!(decode_estop(&[0, 1, 2]).is_none());
    assert!(decode_estop(&[0xCD, 0xAB, 0xFF, 0x00]).is_none());
    assert!(decode_estop(&[0xCD, 0xAB, 0x05, 0x00, b'a', b'b', b'c']).is_none());
}

#[test]
fn test_estop_long_reason_truncated() {
    use horus_net::estop::{decode_estop, encode_estop};

    let long_reason = "x".repeat(500); // 500 chars, should be capped at 200
    let encoded = encode_estop(0x1234, &long_reason);
    let (_, decoded_reason, _) = decode_estop(&encoded).expect("should decode");
    assert!(
        decoded_reason.len() <= 200,
        "Reason should be truncated to 200 bytes, got {}",
        decoded_reason.len()
    );
}

// ============================================================================
// Presence broadcast encode/decode
// ============================================================================

#[test]
fn test_presence_receiver_handles_valid_data() {
    use horus_net::presence::PresenceReceiver;

    // Manually build a minimal presence broadcast payload
    let mut payload = Vec::new();
    let ns = b"default";
    payload.extend_from_slice(&(ns.len() as u16).to_le_bytes());
    payload.extend_from_slice(ns);
    let hid = b"abcd";
    payload.extend_from_slice(&(hid.len() as u16).to_le_bytes());
    payload.extend_from_slice(hid);
    let now_ns = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64;
    payload.extend_from_slice(&now_ns.to_le_bytes());
    payload.extend_from_slice(&1u16.to_le_bytes()); // 1 node
    let name = b"test_node";
    payload.extend_from_slice(&(name.len() as u16).to_le_bytes());
    payload.extend_from_slice(name);
    payload.extend_from_slice(&100.0f64.to_bits().to_le_bytes()); // rate_hz

    let mut receiver = PresenceReceiver::new();
    receiver.handle_broadcast(&payload); // should not panic

    // Verify file was written
    let nodes_dir = horus_sys::shm::shm_nodes_dir();
    let path = nodes_dir.join("remote_abcd.json");
    assert!(path.exists(), "Presence file should be created");

    // Cleanup
    receiver.cleanup_all();
    assert!(!path.exists(), "Presence file should be removed on cleanup");
}

#[test]
fn test_presence_receiver_rejects_wrong_namespace() {
    use horus_net::presence::PresenceReceiver;

    let mut payload = Vec::new();
    let ns = b"other_namespace"; // different from local "default"
    payload.extend_from_slice(&(ns.len() as u16).to_le_bytes());
    payload.extend_from_slice(ns);
    let hid = b"wxyz";
    payload.extend_from_slice(&(hid.len() as u16).to_le_bytes());
    payload.extend_from_slice(hid);
    payload.extend_from_slice(&0u64.to_le_bytes());
    payload.extend_from_slice(&0u16.to_le_bytes());

    let mut receiver = PresenceReceiver::new();
    receiver.handle_broadcast(&payload);

    // File should NOT be created (wrong namespace)
    let nodes_dir = horus_sys::shm::shm_nodes_dir();
    let path = nodes_dir.join("remote_wxyz.json");
    assert!(!path.exists(), "Should reject different namespace");
}

// ============================================================================
// Log replication handle_remote_logs
// ============================================================================

#[test]
fn test_log_replication_decode_and_write() {
    use horus_core::core::log_buffer::GLOBAL_REMOTE_LOG_BUFFER;
    use horus_net::log_replication::handle_remote_logs;

    let before = GLOBAL_REMOTE_LOG_BUFFER.write_idx();

    let mut payload = Vec::new();
    let host = b"robot_2";
    payload.extend_from_slice(&(host.len() as u16).to_le_bytes());
    payload.extend_from_slice(host);
    payload.extend_from_slice(&2u16.to_le_bytes());

    // Entry 1
    let n1 = b"sensor";
    let m1 = b"IMU timeout";
    payload.extend_from_slice(&(n1.len() as u16).to_le_bytes());
    payload.extend_from_slice(n1);
    payload.push(4); // Error
    payload.extend_from_slice(&(m1.len() as u16).to_le_bytes());
    payload.extend_from_slice(m1);

    // Entry 2
    let n2 = b"planner";
    let m2 = b"Path blocked";
    payload.extend_from_slice(&(n2.len() as u16).to_le_bytes());
    payload.extend_from_slice(n2);
    payload.push(3); // Warning
    payload.extend_from_slice(&(m2.len() as u16).to_le_bytes());
    payload.extend_from_slice(m2);

    handle_remote_logs(&payload);

    let after = GLOBAL_REMOTE_LOG_BUFFER.write_idx();
    assert_eq!(after - before, 2, "Should write 2 entries");

    let entries = GLOBAL_REMOTE_LOG_BUFFER.get_all();
    let robot2: Vec<_> = entries
        .iter()
        .filter(|e| e.node_name.starts_with("robot_2/"))
        .collect();
    assert!(robot2.len() >= 2, "Should find robot_2 entries");
}

#[test]
fn test_log_replication_malformed_no_panic() {
    use horus_net::log_replication::handle_remote_logs;

    handle_remote_logs(&[]); // empty
    handle_remote_logs(&[0, 1]); // too short
    handle_remote_logs(&[0, 0, 0xFF, 0xFF]); // huge host_id_len
    // None should panic
}
