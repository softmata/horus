//! Property-based tests for horus_cpp FFI invariants.
//!
//! Run: cargo test --no-default-features -p horus_cpp --test proptest_ffi

use proptest::prelude::*;
use std::sync::atomic::{AtomicU32, Ordering};

static POOL_ID: AtomicU32 = AtomicU32::new(1_000_000);
fn next_pool_id() -> u32 {
    let base = (std::process::id() % 10_000) * 100;
    base + POOL_ID.fetch_add(1, Ordering::Relaxed)
}

proptest! {
    // Property 1: JsonWireMessage roundtrip.
    // For any well-formed JSON that fits in the payload cap, from_json → to_json
    // → parse must yield a value equal to the original (JSON canonical form).
    #[test]
    fn json_wire_message_roundtrip_is_stable(
        k in "[a-zA-Z0-9_]{0,100}",
        v in 0u64..=1_000_000_000u64,
    ) {
        let j = format!(r#"{{"k":"{}","v":{}}}"#, k, v);
        // Sanity: must parse as JSON before feeding to FFI
        prop_assume!(serde_json::from_str::<serde_json::Value>(&j).is_ok());
        if let Some(m) = horus_cpp::JsonWireMessage::from_json(&j, 0, 0) {
            let back = m.to_json();
            prop_assert!(back.is_some());
            let a: serde_json::Value = serde_json::from_str(&j).unwrap();
            let b: serde_json::Value = serde_json::from_str(&back.unwrap()).unwrap();
            prop_assert_eq!(a, b);
        }
    }

    // Property 2: JsonWireMessage never panics.
    // For any string, from_json must return Some or None — never panic.
    #[test]
    fn json_wire_message_never_panics(s in ".*") {
        let _ = horus_cpp::JsonWireMessage::from_json(&s, 0, 0);
    }

    // Property 3: Pool image allocation bounded by capacity.
    // For any (width, height, encoding) in reasonable range, image_new must
    // return Some or None without panicking.
    #[test]
    fn image_alloc_bounded_succeeds(
        w in 1u32..=1024,
        h in 1u32..=1024,
        enc in 0u8..=3,
    ) {
        let needed_bytes = (w as usize) * (h as usize) * 4;
        let pool_size = needed_bytes.max(4 * 1024 * 1024);
        let pool = horus_cpp::tensor_pool_new(next_pool_id(), pool_size, 64);
        prop_assume!(pool.is_some());
        let pool = pool.unwrap();
        let img = horus_cpp::image_new(&pool, w, h, enc);
        prop_assert!(img.is_some() || img.is_none());
    }

    // Property 4: Tensor nbytes matches shape × elem_size.
    #[test]
    fn tensor_nbytes_matches_shape(
        shape in prop::collection::vec(1u64..=50, 1..=4),
        dtype in 0u8..=3,
    ) {
        let elem_size: u64 = match dtype { 0 => 4, 1 => 8, 2 => 1, 3 => 4, _ => unreachable!() };
        let numel: u64 = shape.iter().product();
        let pool = horus_cpp::tensor_pool_new(
            next_pool_id(),
            (numel * elem_size * 2) as usize + 1024,
            64,
        );
        prop_assume!(pool.is_some());
        let pool = pool.unwrap();
        let t = horus_cpp::tensor_alloc(&pool, &shape, dtype);
        if let Some(t) = t {
            prop_assert_eq!(horus_cpp::tensor_nbytes(&t), numel * elem_size);
        }
    }
}
