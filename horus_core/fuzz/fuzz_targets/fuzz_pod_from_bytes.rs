//! Fuzz target for HORUS PodMessage::from_bytes() path.
//!
//! PodMessage types use `bytemuck::try_from_bytes()` for zero-copy
//! interpretation of raw bytes. This fuzzer verifies that arbitrary byte
//! sequences never cause panics or UB — only clean None returns for
//! wrong-sized or misaligned inputs.

#![no_main]

use libfuzzer_sys::fuzz_target;
use bytemuck::{Pod, Zeroable};

// Replicate the PodMessage::from_bytes logic without depending on the
// unsafe trait (which requires types to be defined in horus_core).

fn pod_from_bytes<T: Pod>(bytes: &[u8]) -> Option<&T> {
    if bytes.len() != std::mem::size_of::<T>() {
        return None;
    }
    bytemuck::try_from_bytes(bytes).ok()
}

// Representative POD layouts matching real HORUS types.

/// Matches horus_core::types::Tensor layout (6 x f32 = 24 bytes)
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct FuzzTensor {
    data: [f32; 6],
}

/// Matches typical motor command (timestamp + velocity + torque + pad = 16 bytes)
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct FuzzMotorPod {
    timestamp_ns: u64,
    velocity: f32,
    torque: f32,
}

/// Matches ImageDescriptor-like layout (larger POD struct)
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct FuzzImageDesc {
    width: u32,
    height: u32,
    encoding: u32,
    step: u32,
    data_offset: u64,
    data_length: u64,
    timestamp_ns: u64,
}

/// Matches PointXYZ layout (3 x f32 = 12 bytes)
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct FuzzPointXYZ {
    x: f32,
    y: f32,
    z: f32,
}

/// Matches PointXYZRGB layout (3 x f32 + u32 = 16 bytes)
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct FuzzPointXYZRGB {
    x: f32,
    y: f32,
    z: f32,
    rgb: u32,
}

/// Large POD (64 bytes) — exercises alignment edge cases
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct FuzzLargePod {
    values: [f64; 8],
}

fuzz_target!(|data: &[u8]| {
    // Exercise from_bytes with various POD sizes.
    // Each call must either return Some (valid interpretation) or None — never panic.
    let _ = pod_from_bytes::<FuzzTensor>(data);
    let _ = pod_from_bytes::<FuzzMotorPod>(data);
    let _ = pod_from_bytes::<FuzzImageDesc>(data);
    let _ = pod_from_bytes::<FuzzPointXYZ>(data);
    let _ = pod_from_bytes::<FuzzPointXYZRGB>(data);
    let _ = pod_from_bytes::<FuzzLargePod>(data);

    // Also exercise with sub-slices to catch off-by-one in length checks
    if data.len() > 1 {
        let _ = pod_from_bytes::<FuzzMotorPod>(&data[1..]);
        let _ = pod_from_bytes::<FuzzPointXYZ>(&data[1..]);
    }
    if data.len() > 4 {
        let _ = pod_from_bytes::<FuzzTensor>(&data[4..]);
        let _ = pod_from_bytes::<FuzzImageDesc>(&data[4..]);
    }

    // Exercise primitive POD types
    let _ = pod_from_bytes::<f32>(data);
    let _ = pod_from_bytes::<f64>(data);
    let _ = pod_from_bytes::<u64>(data);
    let _ = pod_from_bytes::<[f32; 3]>(data);
    let _ = pod_from_bytes::<[u8; 16]>(data);
});
