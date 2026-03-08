//! Fuzz target for URDF XML parsing.
//!
//! URDF files describe robot models and come from external sources
//! (downloaded robot descriptions, user-created files, etc.). The
//! parser must handle arbitrary malformed XML without panicking.
//!
//! Uses the `urdf_rs` crate which is a workspace dependency.

#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    // urdf_rs::read_from_string takes a &str — only valid UTF-8 is parseable
    if let Ok(xml_str) = std::str::from_utf8(data) {
        // Must not panic on any input — only return Ok or Err
        let _ = urdf_rs::read_from_string(xml_str);
    }
});
