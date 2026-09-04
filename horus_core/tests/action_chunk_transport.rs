//! `ActionChunk` over a topic, with its actions still alive at the other end.
//!
//! `ActionChunk` is `Pod`, so it has always been *sendable*: `send()` puts its
//! 256-byte descriptor on the wire. But the descriptor only NAMES a pool slot,
//! and nothing on that path took a reference on it, so the actions could be
//! recycled while a subscriber still held the descriptor. Because a recycled
//! slot is valid memory holding somebody else's numbers, the symptom is a
//! servo loop tracking the wrong trajectory, not a crash.
//!
//! The seam this matters on is the one the type was built for: a policy
//! publishing at 10-50 Hz into a control loop sampling at 500-1000 Hz. The
//! subscriber holds each chunk for tens of its own periods — precisely the
//! window in which the producer publishes again and recycles the slot.

use horus_core::communication::topic::Topic;
use horus_core::memory::ActionChunkHandle;
use horus_core::types::{ActionAt, ActionChunk, Device, TensorDtype};

mod common;
use common::{cleanup_stale_shm, unique};

const HORIZON: u32 = 8;
const ACTION_DIM: u32 = 4;
const DT_NS: u64 = 20_000_000; // 50 Hz policy cadence

/// Fill a chunk's actions so row `r`, column `c` holds `r * 100 + c`.
/// Any misread — wrong row, recycled slot, stale generation — shows up as a
/// number that could not have come from here.
fn fill(handle: &ActionChunkHandle) {
    let actions = handle
        .actions_f32_mut()
        .expect("freshly allocated f32 actions must be readable");
    assert_eq!(actions.len(), (HORIZON * ACTION_DIM) as usize);
    for r in 0..HORIZON {
        for c in 0..ACTION_DIM {
            actions[(r * ACTION_DIM + c) as usize] = (r * 100 + c) as f32;
        }
    }
}

#[test]
fn a_chunk_survives_the_trip_with_its_actions_intact() {
    let _shm = cleanup_stale_shm();
    let name = unique("ac.roundtrip");
    let topic: Topic<ActionChunk> = Topic::new(&name).unwrap();

    let t0 = 1_000_000_000u64;
    let handle = topic
        .alloc_chunk(
            HORIZON,
            ACTION_DIM,
            TensorDtype::F32,
            Device::cpu(),
            t0,
            DT_NS,
        )
        .unwrap();
    fill(&handle);
    topic.send_chunk(&handle);

    let got = topic.recv_chunk().expect("the chunk must arrive");
    assert_eq!(got.chunk().horizon(), HORIZON);
    assert_eq!(got.chunk().action_dim(), ACTION_DIM);
    assert_eq!(got.chunk().t0_ns(), t0);
    assert_eq!(got.chunk().dt_ns(), DT_NS);

    let actions = got
        .actions_f32()
        .expect("received actions must be readable");
    for r in 0..HORIZON {
        for c in 0..ACTION_DIM {
            assert_eq!(
                actions[(r * ACTION_DIM + c) as usize],
                (r * 100 + c) as f32,
                "row {r} col {c} is not what was written"
            );
        }
    }
}

/// The defect, directly.
///
/// The producer publishes and drops its handle — the ordinary shape of a
/// policy that computes a chunk, sends it, and moves on — and then publishes
/// several more chunks, recycling pool slots. The subscriber, holding the
/// first chunk the whole time, must still read the actions that were written
/// into it.
///
/// Without a reference transferred by the send, the first chunk's slot is
/// free the moment its producer handle drops, and the later allocations take
/// it. The data then reads as another chunk's numbers.
#[test]
fn a_held_chunk_is_not_recycled_by_later_publishes() {
    let _shm = cleanup_stale_shm();
    let name = unique("ac.keepalive");
    let topic: Topic<ActionChunk> = Topic::new(&name).unwrap();

    let t0 = 5_000_000_000u64;
    {
        let first = topic
            .alloc_chunk(
                HORIZON,
                ACTION_DIM,
                TensorDtype::F32,
                Device::cpu(),
                t0,
                DT_NS,
            )
            .unwrap();
        fill(&first);
        topic.send_chunk(&first);
        // The producer is done with it. Everything keeping these actions alive
        // from here is the reference the send transferred.
    }

    let held = topic.recv_chunk().expect("the first chunk must arrive");

    // Churn: more chunks allocated, filled with a value that cannot be
    // confused with the first, sent, and dropped.
    for i in 0..6u32 {
        let later = topic
            .alloc_chunk(
                HORIZON,
                ACTION_DIM,
                TensorDtype::F32,
                Device::cpu(),
                t0 + (i as u64 + 1) * 1_000_000_000,
                DT_NS,
            )
            .unwrap();
        for v in later.actions_f32_mut().unwrap().iter_mut() {
            *v = -1.0;
        }
        topic.send_chunk(&later);
        let _ = topic.recv_chunk();
    }

    let actions = held
        .actions_f32()
        .expect("the held chunk's actions must still be readable");
    for r in 0..HORIZON {
        for c in 0..ACTION_DIM {
            let got = actions[(r * ACTION_DIM + c) as usize];
            assert_eq!(
                got,
                (r * 100 + c) as f32,
                "row {r} col {c} reads {got} after six later publishes — the slot was \
                 recycled while this chunk was still held"
            );
        }
    }
    assert_eq!(held.chunk().t0_ns(), t0, "and it is still the first chunk");
}

/// Sampling across the seam: a 50 Hz chunk queried at servo rate.
#[test]
fn a_received_chunk_samples_before_inside_and_stale() {
    let _shm = cleanup_stale_shm();
    let name = unique("ac.sample");
    let topic: Topic<ActionChunk> = Topic::new(&name).unwrap();

    let t0 = 2_000_000_000u64;
    let handle = topic
        .alloc_chunk(
            HORIZON,
            ACTION_DIM,
            TensorDtype::F32,
            Device::cpu(),
            t0,
            DT_NS,
        )
        .unwrap();
    fill(&handle);
    topic.send_chunk(&handle);
    let got = topic.recv_chunk().unwrap();

    let mut out = [0.0f32; ACTION_DIM as usize];

    // Before the chunk starts.
    let before = got.locate(t0 - 1_000_000);
    assert!(
        matches!(before, ActionAt::Before { .. }),
        "a query before t0 must report Before, got {before:?}"
    );

    // Exactly on the first knot: the stored row, bit for bit.
    let at = got.sample_into(t0, &mut out);
    assert!(
        matches!(at, ActionAt::Inside { lo: 0, .. }),
        "a query at t0 must land on row 0, got {at:?}"
    );
    for (c, v) in out.iter().enumerate() {
        assert_eq!(*v, c as f32, "row 0 col {c} should be reproduced exactly");
    }

    // Past the end. The safety-relevant case: `out` must not be touched.
    let mut untouched = [f32::NAN; ACTION_DIM as usize];
    let span = DT_NS * HORIZON as u64;
    let stale = got.sample_into(t0 + span + 1_000_000_000, &mut untouched);
    assert!(
        matches!(stale, ActionAt::Stale { .. }),
        "a query past the chunk must report Stale, got {stale:?}"
    );
    assert!(
        untouched.iter().all(|v| v.is_nan()),
        "a stale chunk must not write the output buffer — extrapolating past a \
         policy's last action is the failure this type exists to refuse"
    );
}

/// The pool-id guard: a handle may not be paired with a chunk from a different
/// pool, because releasing it would decrement the wrong refcount.
#[test]
fn a_chunk_and_a_handle_from_different_pools_are_refused() {
    let _shm = cleanup_stale_shm();
    let a: Topic<ActionChunk> = Topic::new(unique("ac.pool_a")).unwrap();
    let b: Topic<ActionChunk> = Topic::new(unique("ac.pool_b")).unwrap();

    let ha = a
        .alloc_chunk(
            HORIZON,
            ACTION_DIM,
            TensorDtype::F32,
            Device::cpu(),
            0,
            DT_NS,
        )
        .unwrap();
    let hb = b
        .alloc_chunk(
            HORIZON,
            ACTION_DIM,
            TensorDtype::F32,
            Device::cpu(),
            0,
            DT_NS,
        )
        .unwrap();

    if ha.chunk().tensor().pool_id == hb.chunk().tensor().pool_id {
        eprintln!("both topics resolved to one pool on this host — nothing to check");
        return;
    }

    let mismatched = ActionChunkHandle::from_owned(*ha.chunk(), hb.tensor().clone());
    assert!(
        mismatched.is_err(),
        "pairing a chunk with a handle in another pool must be refused, not accepted"
    );
}

/// A chunk that is not `f32` must refuse the f32 read paths.
///
/// `bytemuck` will not catch this on its own: casting `&[u8]` to `&[f32]`
/// fails only on alignment or a length that is not a multiple of four, and an
/// `f64` buffer satisfies both. The cast SUCCEEDS and reinterprets each pair of
/// `f64` halves as two `f32`s. The numbers that come out are garbage, and on
/// this type they are actuator commands — so the dtype is checked explicitly.
#[test]
fn a_non_f32_chunk_refuses_the_f32_paths_instead_of_reinterpreting_bytes() {
    let _shm = cleanup_stale_shm();
    let topic: Topic<ActionChunk> = Topic::new(unique("ac.dtype")).unwrap();

    let h = topic
        .alloc_chunk(
            HORIZON,
            ACTION_DIM,
            TensorDtype::F64,
            Device::cpu(),
            1_000,
            DT_NS,
        )
        .expect("an f64 chunk allocates");

    assert!(
        h.actions_f32().is_none(),
        "actions_f32 must refuse an f64 chunk rather than reinterpreting its bytes"
    );
    assert!(
        h.actions_f32_mut().is_none(),
        "actions_f32_mut must refuse an f64 chunk"
    );

    let mut out = [0.0f32; ACTION_DIM as usize];
    assert_eq!(
        h.sample_into(1_000, &mut out),
        ActionAt::Malformed,
        "sample_into must report Malformed for a chunk it cannot read as f32, \
         not hand back reinterpreted f64 halves"
    );
    assert_eq!(
        out, [0.0; ACTION_DIM as usize],
        "and it must not have written anything"
    );
}
