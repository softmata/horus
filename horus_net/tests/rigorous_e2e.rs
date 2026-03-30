//! Rigorous E2E tests — real SHM, real data, full encode→UDP→decode→SHM pipeline.
//!
//! Architecture note: horus_core Topic<T> uses in-process backends (not SHM) when
//! publisher and subscriber are in the same process. Cross-process topics use SHM.
//! Since we can't spawn separate processes in unit tests, we test using the raw SHM
//! API (read_latest_slot_bytes / write_topic_slot_bytes) which IS the cross-process
//! contract that horus_net operates on.
//!
//! The full integration (Topic::send → Replicator export → network → Replicator import
//! → Topic::recv) requires the cross-crate wiring that plugs horus_net into horus_core.

use std::sync::Arc;
use std::time::{Duration, Instant};

use horus_core::communication::{read_latest_slot_bytes, write_topic_slot_bytes, Topic};
use horus_net::config::NetConfig;
use horus_net::fragment::{Fragmenter, Reassembler};
use horus_net::priority::{Encoding, Priority, Reliability};
use horus_net::registry::TopicRole;
use horus_net::reliability::ReliabilityLayer;
use horus_net::wire::*;
use horus_robotics::CmdVel;

use horus_sys::shm::shm_topics_dir;

fn unique_topic(base: &str) -> String {
    use std::sync::atomic::{AtomicU32, Ordering};
    static COUNTER: AtomicU32 = AtomicU32::new(0);
    let id = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("e2e_{base}_{id}")
}

fn shm_path_for(topic_name: &str) -> std::path::PathBuf {
    shm_topics_dir().join(format!("horus_{topic_name}"))
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 1: Topic creation produces valid SHM file
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn topic_creates_valid_shm_file() {
    let name = unique_topic("valid_shm");
    let _topic: Topic<CmdVel> = Topic::new(&name).expect("create topic");
    let path = shm_path_for(&name);
    assert!(path.exists(), "SHM file should exist at {path:?}");
    assert!(
        path.metadata().unwrap().len() >= 640,
        "SHM file should be at least header size"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 2: Raw SHM write → read roundtrip (cross-process contract)
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn raw_shm_write_read_roundtrip() {
    let name = unique_topic("raw_rw");
    let _topic: Topic<CmdVel> = Topic::new(&name).expect("create topic");
    let path = shm_path_for(&name);

    // Write raw CmdVel bytes into SHM (simulates cross-process publisher)
    let cmd = CmdVel::new(42.0, -2.75);
    let bytes: &[u8] = unsafe {
        std::slice::from_raw_parts(
            &cmd as *const CmdVel as *const u8,
            std::mem::size_of::<CmdVel>(),
        )
    };
    assert!(
        write_topic_slot_bytes(&path, bytes),
        "raw SHM write should succeed"
    );

    // Read back (simulates cross-process subscriber or horus_net ShmRingReader)
    let slot = read_latest_slot_bytes(&path, 0).unwrap();
    assert!(slot.is_pod);
    assert_eq!(slot.payload.len(), std::mem::size_of::<CmdVel>());
    assert_eq!(slot.write_idx, 1);

    // Decode
    let decoded: CmdVel =
        unsafe { std::ptr::read_unaligned(slot.payload.as_ptr() as *const CmdVel) };
    assert_eq!(decoded.linear, 42.0);
    assert!((decoded.angular - (-2.75)).abs() < 0.01);
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 3: Multiple raw SHM writes, incremental reading
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn raw_shm_multiple_writes_incremental_read() {
    let name = unique_topic("multi_rw");
    let _topic: Topic<CmdVel> = Topic::new(&name).expect("create topic");
    let path = shm_path_for(&name);

    let mut last_idx = 0u64;
    for i in 0..10u32 {
        let cmd = CmdVel::new(i as f32, 0.0);
        let bytes: &[u8] = unsafe {
            std::slice::from_raw_parts(
                &cmd as *const CmdVel as *const u8,
                std::mem::size_of::<CmdVel>(),
            )
        };
        assert!(write_topic_slot_bytes(&path, bytes));

        let slot = read_latest_slot_bytes(&path, last_idx).unwrap();
        last_idx = slot.write_idx;
        assert_eq!(slot.write_idx, (i + 1) as u64);

        let decoded: CmdVel =
            unsafe { std::ptr::read_unaligned(slot.payload.as_ptr() as *const CmdVel) };
        assert!((decoded.linear - i as f32).abs() < 0.01);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 4: ShmRingReader with raw SHM writes
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn shm_ring_reader_with_raw_writes() {
    let name = unique_topic("reader_raw");
    let _topic: Topic<CmdVel> = Topic::new(&name).expect("create topic");
    let path = shm_path_for(&name);
    let mut reader = horus_net::shm_reader::ShmRingReader::new(&name);

    // No data yet
    assert!(reader.try_read_latest().is_none());

    // Write first message
    let cmd1 = CmdVel::new(1.0, 0.0);
    let bytes: &[u8] = unsafe {
        std::slice::from_raw_parts(
            &cmd1 as *const _ as *const u8,
            std::mem::size_of::<CmdVel>(),
        )
    };
    write_topic_slot_bytes(&path, bytes);

    let msg = reader.try_read_latest();
    assert!(msg.is_some(), "reader should see first write");
    assert_eq!(reader.last_write_idx(), 1);

    // No new data
    assert!(reader.try_read_latest().is_none());

    // Write second
    let cmd2 = CmdVel::new(2.0, 0.0);
    let bytes2: &[u8] = unsafe {
        std::slice::from_raw_parts(
            &cmd2 as *const _ as *const u8,
            std::mem::size_of::<CmdVel>(),
        )
    };
    write_topic_slot_bytes(&path, bytes2);
    assert!(reader.try_read_latest().is_some());
    assert_eq!(reader.last_write_idx(), 2);
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 5: Full pipeline — SHM write → read → encode → UDP → decode → SHM write → read
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn full_pipeline_shm_udp_shm() {
    use std::net::UdpSocket;

    let name_a = unique_topic("pipe_a");
    let name_b = unique_topic("pipe_b");
    let _topic_a: Topic<CmdVel> = Topic::new(&name_a).expect("create A");
    let _topic_b: Topic<CmdVel> = Topic::new(&name_b).expect("create B");
    let path_a = shm_path_for(&name_a);
    let path_b = shm_path_for(&name_b);

    // Step 1: Write CmdVel to A's SHM (simulating a local publisher on machine A)
    let cmd = CmdVel::new(3.0, 1.5);
    let cmd_bytes: &[u8] = unsafe {
        std::slice::from_raw_parts(&cmd as *const _ as *const u8, std::mem::size_of::<CmdVel>())
    };
    assert!(write_topic_slot_bytes(&path_a, cmd_bytes));

    // Step 2: Read from A's SHM (ShmRingReader on machine A)
    let slot = read_latest_slot_bytes(&path_a, 0).unwrap();
    assert_eq!(slot.payload.len(), std::mem::size_of::<CmdVel>());

    // Step 3: Encode into wire format
    let header = PacketHeader::new(PacketFlags::empty(), 0xABCD, 1);
    let out_msg = OutMessage {
        topic_name: name_a.clone(),
        topic_hash: topic_hash(&name_a),
        payload: slot.payload.clone(),
        timestamp_ns: 1_000_000_000,
        sequence: 1,
        priority: Priority::RealTime,
        reliability: Reliability::Redundant,
        encoding: Encoding::PodLe,
    };
    let mut send_buf = [0u8; 4096];
    let len = encode_single(&header, &out_msg, &mut send_buf);

    // Step 4: Send over UDP loopback
    let sock_a = UdpSocket::bind("127.0.0.1:0").unwrap();
    let sock_b = UdpSocket::bind("127.0.0.1:0").unwrap();
    sock_b
        .set_read_timeout(Some(Duration::from_millis(500)))
        .unwrap();
    sock_a
        .send_to(&send_buf[..len], sock_b.local_addr().unwrap())
        .unwrap();

    // Step 5: Receive and decode
    let mut recv_buf = [0u8; 4096];
    let (n, _) = sock_b.recv_from(&mut recv_buf).unwrap();
    let (_, msgs) = decode_packet(&recv_buf[..n]).unwrap();
    assert_eq!(msgs.len(), 1);
    assert_eq!(msgs[0].payload, slot.payload); // Byte-exact

    // Step 6: Write received bytes into B's SHM (ShmRingWriter on machine B)
    assert!(write_topic_slot_bytes(&path_b, &msgs[0].payload));

    // Step 7: Read from B's SHM (local subscriber on machine B)
    let slot_b = read_latest_slot_bytes(&path_b, 0).unwrap();
    let received: CmdVel =
        unsafe { std::ptr::read_unaligned(slot_b.payload.as_ptr() as *const CmdVel) };
    assert_eq!(received.linear, 3.0);
    assert_eq!(received.angular, 1.5);
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 6: Stress — 1000 messages through full encode→UDP→decode pipeline
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn stress_1000_messages() {
    use std::net::UdpSocket;

    let name = unique_topic("stress");
    let _topic: Topic<CmdVel> = Topic::new(&name).expect("create topic");
    let path = shm_path_for(&name);
    let th = topic_hash(&name);

    let sock_a = UdpSocket::bind("127.0.0.1:0").unwrap();
    let sock_b = UdpSocket::bind("127.0.0.1:0").unwrap();
    sock_b
        .set_read_timeout(Some(Duration::from_millis(10)))
        .unwrap();
    let addr_b = sock_b.local_addr().unwrap();

    let mut sent = 0u32;
    let mut recv = 0u32;

    for i in 0..1000u32 {
        // Write to SHM
        let cmd = CmdVel::new(i as f32, 0.0);
        let bytes: &[u8] = unsafe {
            std::slice::from_raw_parts(&cmd as *const _ as *const u8, std::mem::size_of::<CmdVel>())
        };
        write_topic_slot_bytes(&path, bytes);

        // Read + encode + send
        if let Some(slot) = read_latest_slot_bytes(&path, i as u64) {
            let hdr = PacketHeader::new(PacketFlags::empty(), 0x1234, i);
            let msg = OutMessage {
                topic_name: name.clone(),
                topic_hash: th,
                payload: slot.payload,
                timestamp_ns: 0,
                sequence: i,
                priority: Priority::Normal,
                reliability: Reliability::None,
                encoding: Encoding::PodLe,
            };
            let mut buf = [0u8; 256];
            let len = encode_single(&hdr, &msg, &mut buf);
            let _ = sock_a.send_to(&buf[..len], addr_b);
            sent += 1;
        }
    }

    // Drain receiver
    let mut recv_buf = [0u8; 256];
    loop {
        match sock_b.recv_from(&mut recv_buf) {
            Ok((n, _)) => {
                if let Some((_, msgs)) = decode_packet(&recv_buf[..n]) {
                    for m in &msgs {
                        let cmd: CmdVel = unsafe {
                            std::ptr::read_unaligned(m.payload.as_ptr() as *const CmdVel)
                        };
                        assert!(cmd.linear >= 0.0 && cmd.linear < 1000.0);
                        recv += 1;
                    }
                }
            }
            Err(_) => break,
        }
    }

    assert_eq!(sent, 1000);
    assert!(recv > 900, "expected >900 on loopback, got {recv}");
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 7: Concurrent raw SHM writers + readers
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn concurrent_raw_shm_access() {
    let name = unique_topic("conc");
    let _topic: Topic<CmdVel> = Topic::new(&name).expect("create topic");
    let path = shm_path_for(&name);

    let read_count = Arc::new(std::sync::atomic::AtomicU32::new(0));
    let rc = read_count.clone();
    let path_w = path.clone();
    let path_r = path.clone();

    let writer = std::thread::spawn(move || {
        for i in 0..500u32 {
            let cmd = CmdVel::new(i as f32, 0.0);
            let bytes: &[u8] = unsafe {
                std::slice::from_raw_parts(
                    &cmd as *const _ as *const u8,
                    std::mem::size_of::<CmdVel>(),
                )
            };
            write_topic_slot_bytes(&path_w, bytes);
            std::thread::sleep(Duration::from_micros(50));
        }
    });

    let reader = std::thread::spawn(move || {
        let mut last_idx = 0u64;
        let deadline = Instant::now() + Duration::from_secs(3);
        while Instant::now() < deadline {
            if let Some(slot) = read_latest_slot_bytes(&path_r, last_idx) {
                last_idx = slot.write_idx;
                rc.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            }
            std::thread::yield_now();
        }
    });

    writer.join().unwrap();
    reader.join().unwrap();

    let reads = read_count.load(std::sync::atomic::Ordering::Relaxed);
    assert!(
        reads > 100,
        "reader should have caught many messages, got {reads}"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 8: Replicator runs without crash alongside real SHM topics
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn replicator_alongside_real_shm() {
    let name = unique_topic("rep_shm");
    let _topic: Topic<CmdVel> = Topic::new(&name).expect("create topic");
    let path = shm_path_for(&name);

    let registry = horus_net::registry::global_registry();
    registry.register(
        &name,
        topic_hash(&name),
        std::mem::size_of::<CmdVel>() as u32,
        TopicRole::Publisher,
        true,
    );

    let config = NetConfig::test_config(0);
    let handle = horus_net::start_replicator(config).unwrap();

    // Write data while replicator is running
    for i in 0..50 {
        let cmd = CmdVel::new(i as f32, 0.0);
        let bytes: &[u8] = unsafe {
            std::slice::from_raw_parts(&cmd as *const _ as *const u8, std::mem::size_of::<CmdVel>())
        };
        write_topic_slot_bytes(&path, bytes);
    }

    std::thread::sleep(Duration::from_millis(200));
    assert!(handle.is_running(), "replicator should not crash");

    handle.stop();
    registry.unregister(&name, TopicRole::Publisher);
    std::thread::sleep(Duration::from_millis(200));
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 9: Fragment + reassemble with real CmdVel payload bytes
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn fragment_reassemble_real_payload() {
    let name = unique_topic("frag");
    let _topic: Topic<CmdVel> = Topic::new(&name).expect("create topic");
    let path = shm_path_for(&name);

    let cmd = CmdVel::new(99.0, -1.0);
    let bytes: &[u8] = unsafe {
        std::slice::from_raw_parts(&cmd as *const _ as *const u8, std::mem::size_of::<CmdVel>())
    };
    write_topic_slot_bytes(&path, bytes);
    let slot = read_latest_slot_bytes(&path, 0).unwrap();

    let mut fragmenter = Fragmenter::new();
    let out_msg = OutMessage {
        topic_name: name.clone(),
        topic_hash: topic_hash(&name),
        payload: slot.payload.clone(),
        timestamp_ns: 0,
        sequence: 1,
        priority: Priority::Normal,
        reliability: Reliability::None,
        encoding: Encoding::PodLe,
    };

    let frags = fragmenter.fragment(&out_msg);
    assert_eq!(frags.len(), 1); // CmdVel is small

    let mut reassembler = Reassembler::new();
    let reassembled = reassembler.feed(frags.into_iter().next().unwrap()).unwrap();
    assert_eq!(reassembled.payload, slot.payload);

    let decoded: CmdVel =
        unsafe { std::ptr::read_unaligned(reassembled.payload.as_ptr() as *const CmdVel) };
    assert_eq!(decoded.linear, 99.0);
    assert_eq!(decoded.angular, -1.0);
}

// ═══════════════════════════════════════════════════════════════════════════
// TEST 10: Dedup with real message data
// ═══════════════════════════════════════════════════════════════════════════

#[test]
fn dedup_with_real_data() {
    let mut layer = ReliabilityLayer::new();
    let th = topic_hash("robot.cmd_vel");

    // 3 redundant copies (same sequence)
    assert!(layer.is_new_message(0x1234, th, 1));
    assert!(!layer.is_new_message(0x1234, th, 1));
    assert!(!layer.is_new_message(0x1234, th, 1));

    // New sequence
    assert!(layer.is_new_message(0x1234, th, 2));

    // Different peer, same sequence — both accepted
    assert!(layer.is_new_message(0x5678, th, 1));
}
