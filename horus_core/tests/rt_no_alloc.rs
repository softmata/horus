//! Proof that the **steady-state** real-time hot path is allocation-free.
//!
//! This test registers [`RtAwareAllocator`] as the global allocator for this test
//! binary, then exercises the canonical robotics hot path — receive a POD message,
//! do control math, publish a POD message — inside the RT allocation-free context
//! (`enter_rt_context` / `leave_rt_context`, the same bracket the RT executor puts
//! around `tick()` for `.no_alloc()` nodes). If any step heap-allocates, the
//! allocator panics and the test fails.
//!
//! ## Why it warms up first
//!
//! A `Topic`'s shared-memory backend is initialized **lazily on first `recv`/`send`**
//! (see `RingTopic::ensure_consumer` → `initialize_backend`). That one-time init
//! allocates. This test performs the warmup *outside* the RT context, then measures
//! the steady state — which is what a control loop spends ~all of its time in.
//!
//! (Separately: because that init is lazy and happens inside the first `tick()`,
//! `.no_alloc()` currently trips on tick 1 for any topic-using RT node unless the
//! backend is pre-warmed. That is a distinct issue tracked outside this test.)
//!
//! ## Regression coverage
//!   1. The zero-copy POD `Topic` path (`recv`/`send`) is allocation-free once
//!      initialized.
//!   2. `enter_rt_context` does not allocate per tick — it previously `Box::leak`ed
//!      a fresh `String` every tick (a per-tick heap allocation + unbounded leak).

mod common;
use common::cleanup_stale_shm;

use horus_core::communication::Topic;
use horus_core::memory::rt_allocator::{enter_rt_context, leave_rt_context};

// SAFETY (both `Pod` impls): `#[repr(C)]`, all fields are `Copy` primitives of
// equal alignment ⇒ no padding bytes, no `Drop`, every bit pattern is valid.

/// POD sensor reading (two `f32`s ⇒ 8 bytes, no padding).
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, serde::Serialize, serde::Deserialize)]
struct Reading {
    value: f32,
    seq: f32,
}
unsafe impl horus_core::bytemuck::Zeroable for Reading {}
unsafe impl horus_core::bytemuck::Pod for Reading {}
unsafe impl horus_core::communication::PodMessage for Reading {}

/// POD control command (same zero-copy properties as `Reading`).
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, serde::Serialize, serde::Deserialize)]
struct Command {
    output: f32,
    seq: f32,
}
unsafe impl horus_core::bytemuck::Zeroable for Command {}
unsafe impl horus_core::bytemuck::Pod for Command {}
unsafe impl horus_core::communication::PodMessage for Command {}

#[test]
fn rt_steady_state_recv_compute_send_is_allocation_free() {
    let _shm_guard = cleanup_stale_shm();

    let reading_topic = common::unique("rt.noalloc.reading");
    let command_topic = common::unique("rt.noalloc.command");

    let reading_pub = Topic::<Reading>::new(&reading_topic).expect("create reading pub");
    let reading_sub = Topic::<Reading>::new(&reading_topic).expect("create reading sub");
    let command_pub = Topic::<Command>::new(&command_topic).expect("create command pub");

    // ── Warmup (outside the RT context) ──────────────────────────────────────
    // The first send/recv lazily initializes each topic's SHM backend, which
    // allocates. Do it here, where allocations are allowed, so the measured loop
    // below runs against fully-initialized backends.
    reading_pub.send(Reading { value: 1.0, seq: 0.0 });
    let _ = reading_sub.recv();
    command_pub.send(Command {
        output: 0.0,
        seq: 0.0,
    });

    // ── Steady state (inside the RT context — allocations now forbidden) ──────
    // Bracket exactly the work a `.no_alloc()` node does each tick: recv a POD
    // reading, compute, send a POD command. If any of this allocates, the global
    // allocator panics and the test fails.
    let mut state = 0.0f32;
    for i in 0..200u32 {
        reading_pub.send(Reading {
            value: i as f32,
            seq: i as f32,
        });

        enter_rt_context("rt_control_node");
        // recv: zero-copy read out of the SHM ring (POD memcpy).
        if let Some(r) = reading_sub.recv() {
            state = r.value * 0.5; // trivial control law
        }
        // send: zero-copy write into the SHM ring (POD memcpy).
        command_pub.send(Command {
            output: state,
            seq: i as f32,
        });
        leave_rt_context();
    }

    // Reaching here means 200 steady-state recv → compute → send cycles ran with
    // zero heap allocations under the RT guard.
    assert_eq!(state, 199.0 * 0.5, "control loop should have processed readings");
}
