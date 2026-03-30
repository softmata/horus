//! Fuzz target for HORUS topic header and slot parsing.
//!
//! Exercises `read_latest_slot_bytes()` — the public function that reads
//! a topic's shared memory backing file, validates the header, and extracts
//! the latest message payload. This handles raw bytes from potentially
//! corrupted or malicious shared memory.
//!
//! The function must:
//! - Validate magic number
//! - Bounds-check all header field reads
//! - Handle corrupted capacity/slot_size/type_size
//! - Never panic or trigger undefined behavior

#![no_main]

use libfuzzer_sys::fuzz_target;
use std::io::Write;

fuzz_target!(|data: &[u8]| {
    // Write fuzzer-provided bytes to a temp file, then call
    // read_latest_slot_bytes on it. This exercises the real parsing path.
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("fuzz_topic.shm");

    {
        let mut f = std::fs::File::create(&path).unwrap();
        f.write_all(data).unwrap();
    }

    // Exercise with last_write_idx = 0 (first read — will read any available data)
    let _ = horus_core::communication::read_latest_slot_bytes(&path, 0);

    // Exercise with a non-zero last_write_idx to test the "no new data" path
    let _ = horus_core::communication::read_latest_slot_bytes(&path, 1);
    let _ = horus_core::communication::read_latest_slot_bytes(&path, u64::MAX);

    // If the file happens to have a valid-looking header, exercise with the
    // returned write_idx to test the "same idx" dedup path.
    if let Some(result) = horus_core::communication::read_latest_slot_bytes(&path, 0) {
        // Re-read with the same write_idx — should return None (no new data)
        let _ = horus_core::communication::read_latest_slot_bytes(&path, result.write_idx);
    }
});
