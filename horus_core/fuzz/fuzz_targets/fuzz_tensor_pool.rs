//! Fuzz target for TensorPool alloc/retain/release sequences.
//!
//! Exercises random sequences of pool operations to find:
//! - Double-free (release after refcount already 0)
//! - Use-after-free (retain after release)
//! - Pool exhaustion handling (alloc when full)
//! - Refcount overflow/underflow
//! - Concurrent-safe free-stack corruption
//!
//! Uses a small pool (64KB, 8 slots) to hit exhaustion quickly.

#![no_main]

use horus_core::memory::{TensorPool, TensorPoolConfig};
use horus_core::types::{Device, Tensor, TensorDtype};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    // Use a unique pool_id per run to avoid SHM collisions between fuzzer workers.
    let pool_id = 0xF0000
        | (std::process::id() & 0xFFFF)
        | ((data.first().copied().unwrap_or(0) as u32) << 16);

    let config = TensorPoolConfig {
        pool_size: 64 * 1024, // 64KB — small to hit exhaustion quickly
        max_slots: 8,         // Only 8 slots — forces reuse and exhaustion
        slot_alignment: 64,
        ..Default::default()
    };

    let pool = match TensorPool::new(pool_id, config) {
        Ok(p) => p,
        Err(_) => return, // SHM creation can fail in fuzzer sandbox — skip
    };

    // Track live tensors for retain/release operations
    let mut live: Vec<Tensor> = Vec::new();

    // Interpret each byte as an operation:
    //   0x00..0x3F → alloc (shape derived from next 2 bytes)
    //   0x40..0x7F → retain a random live tensor
    //   0x80..0xBF → release a random live tensor (and remove from live)
    //   0xC0..0xFF → drop a random live tensor (test RAII cleanup)
    let mut i = 0;
    while i < data.len() {
        let op = data[i];
        i += 1;

        match op {
            0x00..=0x3F => {
                // Alloc: use next 2 bytes for width/height (small shapes)
                let w = data.get(i).copied().unwrap_or(1).max(1) as u64;
                let h = data.get(i + 1).copied().unwrap_or(1).max(1) as u64;
                i += 2;

                // Keep shapes small to fit in 64KB pool
                let w = w.min(32);
                let h = h.min(32);

                match pool.alloc(&[w, h], TensorDtype::F32, Device::cpu()) {
                    Ok(tensor) => {
                        live.push(tensor);
                    }
                    Err(_) => {
                        // Pool exhaustion is expected — must not panic
                    }
                }
            }
            0x40..=0x7F => {
                // Retain: bump refcount on a random live tensor
                if !live.is_empty() {
                    let idx = (op as usize) % live.len();
                    pool.retain(&live[idx]);
                    // Push a clone so we need to release/drop it too
                    live.push(live[idx].clone());
                }
            }
            0x80..=0xBF => {
                // Release: decrement refcount and remove from tracking
                if !live.is_empty() {
                    let idx = (op as usize) % live.len();
                    let tensor = live.swap_remove(idx);
                    pool.release(&tensor);
                }
            }
            0xC0..=0xFF => {
                // Drop: let RAII handle cleanup (tests Drop impl)
                if !live.is_empty() {
                    let idx = (op as usize) % live.len();
                    let _dropped = live.swap_remove(idx);
                    // Tensor::drop() should call pool.release() internally
                }
            }
        }
    }

    // All remaining live tensors dropped here — tests cleanup-on-drop.
    // Pool dropped here — tests SHM cleanup.
    drop(live);
    drop(pool);
});
