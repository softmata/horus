//! Fuzz target for HORUS message deserialization (bincode path).
//!
//! This exercises the same logic as `read_serde_slot<T>()` in dispatch.rs:
//!
//!   Slot layout: [8B ready_flag | 8B length | data...]
//!
//! The fuzzer feeds arbitrary bytes and we verify that deserialization
//! either succeeds with a valid value or returns None — never panics or
//! triggers undefined behavior.

#![no_main]

use libfuzzer_sys::fuzz_target;
use serde::{Deserialize, Serialize};

// Representative message types that match real HORUS usage patterns.
// These cover the most common field combinations seen in horus_library messages.

#[derive(Clone, Debug, Serialize, Deserialize)]
struct FuzzMotorCmd {
    velocity: f32,
    torque: f32,
    timestamp_ns: u64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct FuzzSensorReading {
    x: f64,
    y: f64,
    z: f64,
    confidence: f32,
    valid: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct FuzzNavGoal {
    position: [f64; 3],
    orientation: [f64; 4],
    frame_id: String,
    stamp_ns: u64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct FuzzDiagnostic {
    name: String,
    message: String,
    level: u8,
    values: Vec<(String, f64)>,
}

/// Simulate the `read_serde_slot` parsing logic:
/// Extract length from slot header, bounds-check, then deserialize the data region.
fn simulate_read_serde_slot<T: serde::de::DeserializeOwned>(data: &[u8], slot_size: usize) -> Option<T> {
    let max_data_len = slot_size.saturating_sub(16);

    if data.len() < 16 {
        return None;
    }

    // Read the length field (bytes 8..16), matching the volatile read in dispatch.rs
    let len = u64::from_ne_bytes(data[8..16].try_into().ok()?) as usize;

    if len > max_data_len {
        return None; // Corrupted length — same check as dispatch.rs
    }

    let available = data.len().saturating_sub(16);
    if len > available {
        return None; // Not enough data in the fuzz input
    }

    let slice = &data[16..16 + len];
    bincode::deserialize(slice).ok()
}

fuzz_target!(|data: &[u8]| {
    // Use various slot sizes to exercise boundary conditions.
    // Real HORUS slots are typically 256, 1024, 4096, or 65536 bytes.
    let slot_sizes = [64, 256, 1024, 4096, 65536];

    for &slot_size in &slot_sizes {
        // Exercise all representative message types — none should panic.
        let _: Option<FuzzMotorCmd> = simulate_read_serde_slot(data, slot_size);
        let _: Option<FuzzSensorReading> = simulate_read_serde_slot(data, slot_size);
        let _: Option<FuzzNavGoal> = simulate_read_serde_slot(data, slot_size);
        let _: Option<FuzzDiagnostic> = simulate_read_serde_slot(data, slot_size);

        // Also exercise primitive types that may be used as topic payloads
        let _: Option<f64> = simulate_read_serde_slot(data, slot_size);
        let _: Option<u64> = simulate_read_serde_slot(data, slot_size);
        let _: Option<String> = simulate_read_serde_slot(data, slot_size);
        let _: Option<Vec<u8>> = simulate_read_serde_slot(data, slot_size);
    }

    // Also test raw bincode deserialization without the slot header,
    // exercising the pure deserialization path.
    let _: Option<FuzzMotorCmd> = bincode::deserialize(data).ok();
    let _: Option<FuzzSensorReading> = bincode::deserialize(data).ok();
    let _: Option<FuzzNavGoal> = bincode::deserialize(data).ok();
    let _: Option<FuzzDiagnostic> = bincode::deserialize(data).ok();
});
