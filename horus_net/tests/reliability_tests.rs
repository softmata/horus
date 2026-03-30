#![allow(dead_code)]
//! Reliability + fragmentation integration tests.

use std::time::Duration;

use horus_net::flow_control::FlowController;
use horus_net::fragment::{Fragmenter, Reassembler, MAX_FRAGMENT_PAYLOAD};
use horus_net::priority::{Encoding, Priority, Reliability};
use horus_net::reliability::ReliabilityLayer;
use horus_net::wire::*;

fn make_out_msg(name: &str, payload: Vec<u8>, priority: Priority) -> OutMessage {
    OutMessage {
        topic_name: name.into(),
        topic_hash: topic_hash(name),
        payload,
        timestamp_ns: 1000,
        sequence: 1,
        priority,
        reliability: Reliability::default_for(priority),
        encoding: Encoding::PodLe,
    }
}

// ─── Reliability ────────────────────────────────────────────────────────────

#[test]
fn estop_latched_delivery_with_ack() {
    let mut layer = ReliabilityLayer::new();
    let th = topic_hash("robot.estop");

    // Start latched message
    layer.start_latch(th, 1, vec![0xFF]);
    assert_eq!(layer.pending_latches(), 1);

    // Simulate ACK
    layer.on_ack(&AckPayload {
        acked_topic_hash: th,
        acked_sequence: 1,
    });
    assert_eq!(layer.pending_latches(), 0);
}

#[test]
fn redundant_dedup_3_copies_to_1() {
    let mut layer = ReliabilityLayer::new();

    // Send 3 copies of same message (redundant)
    assert!(layer.is_new_message(0x1234, 100, 42)); // 1st: new
    assert!(!layer.is_new_message(0x1234, 100, 42)); // 2nd: dup
    assert!(!layer.is_new_message(0x1234, 100, 42)); // 3rd: dup
                                                     // Next sequence: new
    assert!(layer.is_new_message(0x1234, 100, 43));
}

#[test]
fn latched_resends_then_gives_up() {
    let mut layer = ReliabilityLayer::new();
    layer.start_latch(100, 1, vec![42]);

    // Simulate time passing and many resend ticks
    let mut total_resends = 0;
    for _ in 0..200 {
        std::thread::sleep(Duration::from_millis(11));
        let resends = layer.tick_resends();
        total_resends += resends.len();
        if layer.pending_latches() == 0 {
            break;
        }
    }
    // Should have given up
    assert_eq!(layer.pending_latches(), 0);
    assert!(total_resends > 0);
}

// ─── Fragmentation ──────────────────────────────────────────────────────────

#[test]
fn fragment_500kb_roundtrip() {
    let mut fragmenter = Fragmenter::new();
    let payload = vec![0xBB; 500_000]; // 500KB
    let msg = make_out_msg("big_data", payload.clone(), Priority::Normal);

    let fragments = fragmenter.fragment(&msg);
    let expected_count = 500_000_usize.div_ceil(MAX_FRAGMENT_PAYLOAD);
    assert_eq!(fragments.len(), expected_count);

    // Reassemble
    let mut reasm = Reassembler::new();
    let mut result = None;
    for frag in fragments {
        if let Some(msg) = reasm.feed(frag) {
            result = Some(msg);
        }
    }
    let reassembled = result.unwrap();
    assert_eq!(reassembled.payload.len(), 500_000);
    assert_eq!(reassembled.payload, payload);
}

#[test]
fn fragment_timeout_discards_partial() {
    let mut fragmenter = Fragmenter::new();
    let msg = make_out_msg("data", vec![0; 5000], Priority::Normal);
    let fragments = fragmenter.fragment(&msg);
    assert!(fragments.len() > 1);

    let mut reasm = Reassembler::new();
    // Feed only first 2 fragments
    reasm.feed(fragments[0].clone());
    reasm.feed(fragments[1].clone());
    assert_eq!(reasm.pending_count(), 1);

    // Wait for timeout
    std::thread::sleep(Duration::from_millis(150));
    let discarded = reasm.gc_stale();
    assert_eq!(discarded, 1);
    assert_eq!(reasm.pending_count(), 0);
}

#[test]
fn message_over_1mb_rejected() {
    let mut fragmenter = Fragmenter::new();
    let msg = make_out_msg("huge", vec![0; 2_000_000], Priority::Bulk);
    let fragments = fragmenter.fragment(&msg);
    assert!(fragments.is_empty());
}

// ─── Priority Auto-Inference ────────────────────────────────────────────────

#[test]
fn priority_auto_estop_is_immediate() {
    assert_eq!(
        Priority::auto_infer("robot.estop", false, 8),
        Priority::Immediate
    );
}

#[test]
fn priority_auto_rt_node_is_realtime() {
    assert_eq!(
        Priority::auto_infer("cmd_vel", true, 16),
        Priority::RealTime
    );
}

// ─── Sequence Gap Tracking ──────────────────────────────────────────────────

#[test]
fn gap_tracking_detects_loss() {
    let mut fc = FlowController::new();
    fc.on_received(0x1234, 100, 1);
    fc.on_received(0x1234, 100, 2);
    fc.on_received(0x1234, 100, 5); // gap: 3,4 missing
    fc.on_received(0x1234, 100, 8); // gap: 6,7 missing

    let loss = fc.loss_rate(0x1234, 100);
    assert!(loss > 0.0, "expected positive loss rate, got {loss}");
}
