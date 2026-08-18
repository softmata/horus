//! A message that changed shape must not silently share a topic with its
//! previous self.
//!
//! `Topic::new` validates the message type's **short** name (the last `::`
//! segment) and, for POD types, `type_size`. Neither describes field layout, so
//! two revisions of the same message that keep the name and the size while
//! reordering fields both open the topic:
//!
//! ```text
//! v1::Pose { x: f32, y: f32 }   sent (x=1, y=2)
//! v2::Pose { y: f32, x: f32 }   received Pose { y: 1.0, x: 2.0 }
//! ```
//!
//! No error and no warning — the coordinates simply arrive swapped. Two nodes
//! built against different revisions of a message crate is what a fleet looks
//! like halfway through a rollout, and a pose read backwards is a robot driving
//! the wrong way.
//!
//! `Topic::new_checked(name, hash)` closes this. Types declared with `message!`
//! carry `LAYOUT_HASH` and expose `Type::topic(name)`, which supplies it.
//!
//! Run: `cargo test -p horus_core --test message_layout_contract`

use horus_core::communication::Topic;
use horus_core::message;

fn unique(tag: &str) -> String {
    use std::sync::atomic::{AtomicU32, Ordering};
    static N: AtomicU32 = AtomicU32::new(0);
    format!(
        "layout_{tag}_{}_{}",
        std::process::id(),
        N.fetch_add(1, Ordering::Relaxed)
    )
}

// Two revisions of the same message. Same short name, same size, different
// meaning — the exact shape of the hazard.
mod v1 {
    horus_core::message! { Pose { x: f32, y: f32 } }
}
mod v2 {
    horus_core::message! { Pose { y: f32, x: f32 } }
}

// ---------------------------------------------------------------------------
// The hash itself
// ---------------------------------------------------------------------------

#[test]
fn reordering_fields_changes_the_layout_hash() {
    assert_ne!(
        v1::Pose::LAYOUT_HASH,
        v2::Pose::LAYOUT_HASH,
        "Pose {{x, y}} and Pose {{y, x}} hash the same, so the check cannot \
         distinguish them — this is precisely the pair that silently swaps \
         coordinates"
    );
}

message! { Renamed { alpha: f32, beta: f32 } }
message! { RenamedToo { alpha: f32, gamma: f32 } }

#[test]
fn renaming_a_field_changes_the_layout_hash() {
    assert_ne!(
        Renamed::LAYOUT_HASH,
        RenamedToo::LAYOUT_HASH,
        "a renamed field must change the hash: same size and same order, but a \
         consumer reading `beta` would be handed `gamma`"
    );
}

message! { Retyped { count: u32 } }
message! { RetypedToo { count: i32 } }

#[test]
fn changing_a_field_type_changes_the_layout_hash() {
    assert_ne!(
        Retyped::LAYOUT_HASH,
        RetypedToo::LAYOUT_HASH,
        "u32 and i32 are both 4 bytes, so size validation cannot tell them \
         apart; -1 would read as 4294967295"
    );
}

message! { Grown { a: f32 } }
message! { GrownToo { a: f32, b: f32 } }

#[test]
fn adding_a_field_changes_the_layout_hash() {
    assert_ne!(Grown::LAYOUT_HASH, GrownToo::LAYOUT_HASH);
}

/// The hash must be stable across builds, or it would reject every peer.
#[test]
fn the_layout_hash_is_deterministic() {
    assert_eq!(v1::Pose::LAYOUT_HASH, v1::Pose::LAYOUT_HASH);
    assert_ne!(
        v1::Pose::LAYOUT_HASH,
        0,
        "0 means 'unchecked' and must not be a real hash"
    );
}

// ---------------------------------------------------------------------------
// Enforcement
// ---------------------------------------------------------------------------

/// The headline case: the exact pair that silently swapped coordinates.
#[test]
fn a_reordered_message_is_refused_rather_than_misread() {
    let name = unique("refused");

    let _tx = v1::Pose::topic(&name).expect("first open should succeed");

    let err = v2::Pose::topic(&name)
        .err()
        .expect("opening the same topic with a reordered Pose must fail");

    let msg = err.to_string();
    assert!(
        msg.contains("layout"),
        "the error should say the layout differs, got: {msg}"
    );
}

/// Identical definitions must still interoperate — a check that rejects
/// everything is not a check.
#[test]
fn identical_messages_still_share_a_topic() {
    let name = unique("compatible");
    let tx = v1::Pose::topic(&name).expect("publisher");
    let rx = v1::Pose::topic(&name).expect("subscriber with the same definition");

    tx.send(v1::Pose { x: 1.0, y: 2.0 });
    let got = rx.recv().expect("message should arrive");
    assert_eq!(got.x, 1.0);
    assert_eq!(got.y, 2.0);
}

/// `Topic::new` keeps working exactly as before. Existing code must not break,
/// and hand-written structs have no hash to supply.
#[test]
fn the_unchecked_constructor_is_unchanged() {
    let name = unique("unchecked");
    let tx: Topic<v1::Pose> = Topic::new(&name).expect("plain new must still work");
    tx.send(v1::Pose { x: 3.0, y: 4.0 });

    let rx: Topic<v1::Pose> = Topic::new(&name).expect("second handle");
    let got = rx.recv().expect("message should arrive");
    assert_eq!(got.x, 3.0);
}

/// A checked handle must not reject an unchecked peer. Mixed fleets exist, and
/// rejecting the old build would be worse than not protecting it.
#[test]
fn an_unchecked_peer_is_not_rejected() {
    let name = unique("mixed");

    let unchecked: Topic<v1::Pose> = Topic::new(&name).expect("old-style handle");
    let checked = v1::Pose::topic(&name).expect("a checked handle must still open the topic");

    unchecked.send(v1::Pose { x: 5.0, y: 6.0 });
    let got = checked.recv().expect("message should arrive");
    assert_eq!(got.x, 5.0);
}

/// Order must not matter: checked first, then unchecked, is the same story.
#[test]
fn an_unchecked_peer_can_join_a_checked_topic() {
    let name = unique("mixed_rev");
    let _checked = v1::Pose::topic(&name).expect("checked handle first");
    let unchecked: Topic<v1::Pose> =
        Topic::new(&name).expect("an unchecked handle must still be able to join");
    drop(unchecked);
}
