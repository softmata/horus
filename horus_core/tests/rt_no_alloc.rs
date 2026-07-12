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
use horus_core::core::{DurationExt, Node};
use horus_core::memory::rt_allocator::{enter_rt_context, leave_rt_context};
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};

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

/// Ticks the executor-driven `.no_alloc()` controller completed. A `static` +
/// atomic so incrementing it never allocates.
static EXEC_TICKS: AtomicU64 = AtomicU64::new(0);

/// End-to-end through the RT executor: a `.no_alloc()` control node ticking
/// `recv → compute → send` runs continuously without tripping the allocator.
///
/// This exercises the executor's **warmup exemption** — the first tick is not
/// alloc-checked, so the `Topic`'s lazy SHM-backend init may allocate — followed
/// by steady-state enforcement from the second tick on. That combination is what
/// makes `.no_alloc()` usable for real control nodes. Without the exemption the
/// node would panic on tick 1 (lazy init) and the counter would freeze near zero.
#[test]
fn no_alloc_node_runs_through_the_rt_executor() {
    let _shm_guard = cleanup_stale_shm();

    let reading_topic = common::unique("rt.noalloc.exec.reading");
    let command_topic = common::unique("rt.noalloc.exec.command");

    struct Producer {
        out: Topic<Reading>,
        n: u64,
    }
    impl Node for Producer {
        fn name(&self) -> &str {
            "producer"
        }
        fn tick(&mut self) {
            self.n += 1;
            self.out.send(Reading {
                value: self.n as f32,
                seq: self.n as f32,
            });
        }
    }

    struct Controller {
        input: Topic<Reading>,
        output: Topic<Command>,
        state: f32,
    }
    impl Node for Controller {
        fn name(&self) -> &str {
            "controller"
        }
        fn tick(&mut self) {
            if let Some(r) = self.input.recv() {
                self.state = r.value * 0.5;
            }
            self.output.send(Command {
                output: self.state,
                seq: 0.0,
            });
            EXEC_TICKS.fetch_add(1, Ordering::Relaxed);
        }
    }

    let prod_out = Topic::<Reading>::new(&reading_topic).expect("create reading pub");
    let ctrl_in = Topic::<Reading>::new(&reading_topic).expect("create reading sub");
    let ctrl_out = Topic::<Command>::new(&command_topic).expect("create command pub");

    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz());
    scheduler
        .add(Producer { out: prod_out, n: 0 })
        .order(0)
        .build()
        .unwrap();
    // Real-time (`.rate()`) + `.no_alloc()` ⇒ runs on the RT executor with the
    // allocation guard active from the second tick.
    scheduler
        .add(Controller {
            input: ctrl_in,
            output: ctrl_out,
            state: 0.0,
        })
        .order(1)
        .rate(1000_u64.hz())
        .no_alloc()
        .build()
        .unwrap();

    scheduler.run_for(300_u64.ms()).unwrap();

    let ticks = EXEC_TICKS.load(Ordering::Relaxed);
    assert!(
        ticks > 20,
        ".no_alloc() controller should have run many ticks through the RT executor \
         (tick 1 warms up the topic backend, steady state is alloc-checked); got {ticks}. \
         A low count means it panicked on a heap allocation."
    );
}
