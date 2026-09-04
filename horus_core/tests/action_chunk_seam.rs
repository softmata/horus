//! The seam `ActionChunk` exists for: a slow policy feeding a fast control loop.
//!
//! A chunk covers a span of future actions so the consumer can keep producing
//! commands between publishes. That is the whole design: the policy runs at
//! 10-50 Hz, the servo loop at 500-1000 Hz, and the loop interpolates within
//! the newest chunk rather than holding the last command or extrapolating past
//! it.
//!
//! Two properties matter and neither was covered anywhere:
//!
//! 1. While the policy keeps up, the loop samples INSIDE a chunk essentially
//!    every tick — the point of publishing a horizon rather than a point.
//! 2. When the policy stops, the loop goes STALE and stops being given
//!    actions. It must not extrapolate off the end of the last chunk, and
//!    `sample_into` must leave the output buffer untouched so a caller that
//!    ignores the return value cannot silently drive on stale data.

use horus_core::communication::topic::Topic;
use horus_core::types::{ActionAt, ActionChunk, Device, TensorDtype};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Instant;

mod common;
use common::{cleanup_stale_shm, unique};

const HORIZON: u32 = 8;
const ACTION_DIM: u32 = 4;
/// 50 Hz policy: 20ms between actions, so a chunk spans 160ms.
const DT_NS: u64 = 20_000_000;
const SPAN_NS: u64 = DT_NS * HORIZON as u64;

fn origin() -> &'static Instant {
    static O: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    O.get_or_init(Instant::now)
}
fn now_ns() -> u64 {
    origin().elapsed().as_nanos() as u64
}

#[derive(Default)]
struct Outcomes {
    before: AtomicU64,
    inside: AtomicU64,
    stale: AtomicU64,
    malformed: AtomicU64,
}

impl Outcomes {
    fn record(&self, at: ActionAt) {
        match at {
            ActionAt::Before { .. } => &self.before,
            ActionAt::Inside { .. } => &self.inside,
            ActionAt::Stale { .. } => &self.stale,
            ActionAt::Malformed => &self.malformed,
        }
        .fetch_add(1, Ordering::Relaxed);
    }
    fn get(&self) -> (u64, u64, u64, u64) {
        (
            self.before.load(Ordering::Relaxed),
            self.inside.load(Ordering::Relaxed),
            self.stale.load(Ordering::Relaxed),
            self.malformed.load(Ordering::Relaxed),
        )
    }
}

/// Publish one chunk whose actions all equal `value`, starting at `t0`.
fn publish(topic: &Topic<ActionChunk>, t0: u64, value: f32, seq: u64) {
    let mut h = topic
        .alloc_chunk(
            HORIZON,
            ACTION_DIM,
            TensorDtype::F32,
            Device::cpu(),
            t0,
            DT_NS,
        )
        .expect("chunk allocates");
    for v in h.actions_f32_mut().expect("f32 actions").iter_mut() {
        *v = value;
    }
    h.chunk_mut().set_seq(seq);
    topic.send_chunk(&h);
}

/// A policy that keeps up, sampled at servo rate, lands INSIDE.
#[test]
fn a_servo_loop_samples_inside_while_the_policy_keeps_up() {
    let _shm = cleanup_stale_shm();
    let topic: Topic<ActionChunk> = Topic::new(&unique("ac.seam.keepup")).unwrap();
    let outcomes = Outcomes::default();

    // 500 Hz servo against a 50 Hz policy: ten servo ticks per publish.
    const SERVO_TICKS: u64 = 200;
    const SERVO_PERIOD_NS: u64 = 2_000_000;
    const PUBLISH_EVERY: u64 = 10;

    let mut held = None;
    let mut out = [0.0f32; ACTION_DIM as usize];
    let start = now_ns();

    for tick in 0..SERVO_TICKS {
        let t = start + tick * SERVO_PERIOD_NS;

        // The policy publishes a chunk starting AT the current instant, so the
        // horizon always covers the interval before the next publish.
        if tick % PUBLISH_EVERY == 0 {
            publish(&topic, t, tick as f32, tick / PUBLISH_EVERY);
        }
        if let Some(h) = topic.recv_chunk() {
            held = Some(h);
        }

        if let Some(ref h) = held {
            outcomes.record(h.sample_into(t, &mut out));
        }
    }

    let (before, inside, stale, malformed) = outcomes.get();
    assert_eq!(malformed, 0, "no sample should be malformed");
    assert_eq!(
        stale, 0,
        "the policy published every 20ms into a 160ms horizon — nothing should \
         have gone stale (before={before} inside={inside} stale={stale})"
    );
    assert!(
        inside >= SERVO_TICKS - PUBLISH_EVERY,
        "the servo should be inside a chunk on essentially every tick, got \
         {inside} of {SERVO_TICKS} (before={before})"
    );
}

/// When the policy stops, the loop must go stale rather than extrapolate.
#[test]
fn a_stopped_policy_goes_stale_and_never_extrapolates() {
    let _shm = cleanup_stale_shm();
    let topic: Topic<ActionChunk> = Topic::new(&unique("ac.seam.stall")).unwrap();

    let t0 = now_ns();
    publish(&topic, t0, 7.0, 0);
    let held = topic.recv_chunk().expect("the chunk arrives");

    // Inside the horizon: the action is produced.
    let mut out = [0.0f32; ACTION_DIM as usize];
    let at = held.sample_into(t0 + DT_NS, &mut out);
    assert!(
        matches!(at, ActionAt::Inside { .. }),
        "a query one dt in must be inside, got {at:?}"
    );
    assert_eq!(out, [7.0; ACTION_DIM as usize]);

    // Past the horizon, with the policy silent. The output buffer carries the
    // PREVIOUS action, which is exactly the value a caller that ignored the
    // return would keep driving. It must not be overwritten with an
    // extrapolation, and it must not be updated at all.
    let mut latched = out;
    let stale = held.sample_into(t0 + SPAN_NS + DT_NS, &mut latched);
    match stale {
        ActionAt::Stale { by_ns } => {
            assert!(by_ns > 0, "a stale chunk must report how far past it is");
        }
        other => panic!("a query past the horizon must be Stale, got {other:?}"),
    }
    assert_eq!(
        latched, out,
        "sample_into must not touch the output buffer when it returns Stale — \
         writing an extrapolated action here is how a robot keeps moving on a \
         policy that has stopped publishing"
    );
}

/// A late chunk is not a stale one: a query before `t0` reports Before, and
/// the loop must not be handed the first action early.
#[test]
fn a_chunk_that_has_not_started_reports_before_and_writes_nothing() {
    let _shm = cleanup_stale_shm();
    let topic: Topic<ActionChunk> = Topic::new(&unique("ac.seam.early")).unwrap();

    let t0 = now_ns() + 1_000_000_000;
    publish(&topic, t0, 3.0, 0);
    let held = topic.recv_chunk().expect("the chunk arrives");

    let mut out = [f32::NAN; ACTION_DIM as usize];
    let at = held.sample_into(t0 - 500_000_000, &mut out);
    match at {
        ActionAt::Before { by_ns } => {
            assert!(by_ns > 0, "Before must say how long until the chunk starts");
        }
        other => panic!("a query before t0 must be Before, got {other:?}"),
    }
    assert!(
        out.iter().all(|v| v.is_nan()),
        "a chunk that has not started must not have its first action applied early"
    );
}
