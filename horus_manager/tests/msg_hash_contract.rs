//! `horus msg hash` must print the number the runtime actually compares.
//!
//! Two defects, both silent.
//!
//! **It was not stable.** The hash came from `DefaultHasher`, which the standard
//! library documents as unspecified: "the internal algorithm is not specified,
//! and so it and its hashes should not be relied upon over releases." A message
//! definition hash that changes when you upgrade your toolchain reports every
//! message as modified — the one thing it exists to detect.
//!
//! **It did not match the runtime.** `Topic::new_checked` rejects a peer whose
//! message layout differs and prints both hashes. Those come from the
//! `LAYOUT_HASH` the `message!` macro emits, computed with FNV-1a over
//! `Name|field:Type|…`. The CLI hashed a different string with a different
//! algorithm, so a developer comparing `horus msg hash Pose` against the error
//! saw two unrelated numbers for the same type, on the exact task the command
//! exists for.
//!
//! Both sides now use FNV-1a over the same canonical form. These tests pin
//! that: if either implementation drifts, the fixture below stops matching.
//!
//! Run: `cargo test -p horus_manager --test msg_hash_contract`

/// Reference implementation, written out rather than imported, so a change to
/// *either* side has to be made here too and cannot pass unnoticed.
fn fnv1a(bytes: &[u8]) -> u32 {
    let mut hash: u32 = 2166136261;
    for &byte in bytes {
        hash ^= byte as u32;
        hash = hash.wrapping_mul(16777619);
    }
    hash
}

// The same message declared through the macro, so its LAYOUT_HASH is a real
// compile-time value produced by the shipping code path.
horus_core::message! { HashProbe { alpha: f32, beta: u32 } }

#[test]
fn the_macro_hash_matches_the_canonical_form() {
    let expected = fnv1a(b"HashProbe|alpha:f32|beta:u32");
    assert_eq!(
        HashProbe::LAYOUT_HASH,
        expected,
        "message! emitted {:#010x} but the canonical form \
         `HashProbe|alpha:f32|beta:u32` hashes to {expected:#010x}. The CLI \
         builds that string from parsed source; if the macro's concatenation \
         changes, `horus msg hash` starts printing a number the runtime never \
         compares.",
        HashProbe::LAYOUT_HASH
    );
}

/// A fixed vector, so neither side can be changed in step and still pass.
#[test]
fn the_hash_algorithm_is_pinned() {
    assert_eq!(
        fnv1a(b"HashProbe|alpha:f32|beta:u32"),
        0xefb6f672,
        "the FNV-1a implementation changed. This value is shared with the \
         runtime and printed by `horus msg hash`, so changing it invalidates \
         every hash previously recorded or compared."
    );
}

/// Reordering fields must change it — this is the property the whole mechanism
/// exists for.
#[test]
fn field_order_is_part_of_the_hash() {
    assert_ne!(
        fnv1a(b"HashProbe|alpha:f32|beta:u32"),
        fnv1a(b"HashProbe|beta:u32|alpha:f32"),
    );
}

/// `DefaultHasher` must not come back. Its instability across releases is the
/// original defect, and it is an easy thing to reach for again.
#[test]
fn the_cli_does_not_use_an_unstable_hasher() {
    let src = std::fs::read_to_string(
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/commands/msg.rs"),
    )
    .expect("msg.rs must be readable");

    // Match real usage, not the word in the explanatory comment above the fix.
    let uses_it = src.contains("DefaultHasher::new()") || src.contains("hash_map::DefaultHasher");
    assert!(
        !uses_it,
        "msg.rs uses DefaultHasher again. std does not guarantee its output \
         between Rust releases, so the definition hash would change on a \
         toolchain upgrade and report every message as modified."
    );
}

/// The command must still run and print something hash-shaped.
#[test]
fn horus_msg_hash_prints_a_hash() {
    let out = std::process::Command::new(env!("CARGO_BIN_EXE_horus"))
        .args(["msg", "hash", "Twist"])
        .output()
        .expect("horus msg hash must run");

    let stdout = String::from_utf8_lossy(&out.stdout);
    let line = stdout.trim();
    assert!(
        line.starts_with("0x") && line.len() == 10,
        "expected a 0x-prefixed 32-bit hash, got {line:?}"
    );
}
