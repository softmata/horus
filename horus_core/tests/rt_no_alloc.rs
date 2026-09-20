//! `.no_alloc()` **without** the allocator installed — the documented no-op, pinned.
//!
//! This binary does **not** register [`RtAwareAllocator`] as its
//! `#[global_allocator]`, and that is deliberate. Its sibling
//! `rt_alloc_guard_installed.rs` does, and that is where the guarantee is
//! actually proved. Together they cover both halves of the downstream opt-in:
//! what you get when you install the allocator, and what you get when you do
//! not.
//!
//! ## What this file used to claim
//!
//! Its doc comment used to open with "This test registers `RtAwareAllocator` as
//! the global allocator for this test binary … If any step heap-allocates, the
//! allocator panics and the test fails." It did not register anything — no
//! `#[global_allocator]` existed anywhere in the repository — so the tests below
//! ran with the system allocator and could not fail the way they said they
//! could. The claim is removed rather than made true here, because the
//! not-installed case needs test coverage of its own and this is the binary that
//! provides it.
//!
//! ## What is asserted here
//!
//! 1. The runtime probe correctly reports that enforcement is **absent**, and
//!    `warn_if_unenforced()` says so rather than letting a user assume they are
//!    protected. (`is_installed()` returning a false positive would silence the
//!    one warning that tells an operator their RT guarantee is not being
//!    checked, so its negative case is worth a test.)
//! 2. Without the allocator, an allocation inside a `.no_alloc()` bracket is
//!    genuinely undetected — the flag is inert. This pins the documented
//!    behaviour and is the reason the warning in (1) has to exist.
//! 3. The `enter_rt_context` / `leave_rt_context` bracket and the RT executor's
//!    warmup exemption work — plumbing that is independent of which allocator is
//!    installed.
//!
//! [`RtAwareAllocator`]: horus_core::memory::rt_allocator::RtAwareAllocator

mod common;
use common::cleanup_stale_shm;

use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::memory::rt_allocator::{self, enter_rt_context_guarded};
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

// ──────────────────── the not-installed case, made explicit ──────────────────

/// The probe must report enforcement as absent here.
///
/// Also a guard against "fixing" this binary by installing the allocator in it:
/// doing so would silently change what every other test in this file means, and
/// would delete the only coverage of the not-installed path.
#[test]
fn this_binary_has_no_rt_allocator_and_reports_that_honestly() {
    assert!(
        !rt_allocator::is_installed(),
        "this binary deliberately does not install RtAwareAllocator — it is the \
         negative control for rt_alloc_guard_installed.rs. If the probe says \
         otherwise, either someone added a #[global_allocator] here (install it \
         there instead) or the probe reports false positives, which would \
         suppress the .no_alloc() warning in every unprotected binary."
    );
    assert!(
        !rt_allocator::warn_if_unenforced("unprotected_node"),
        "a .no_alloc() node registered without the allocator must be told it is \
         unenforced, not left to assume it is protected"
    );
}

/// Registering a `.no_alloc()` node must actually print the warning.
///
/// `warn_if_unenforced` names its own call site — "Intended call site: node
/// registration, once per `.no_alloc()` node" — and had no caller anywhere
/// outside the two tests that invoke it directly. So the function worked
/// perfectly and no user ever saw it: `.no_alloc()` in a binary without the
/// allocator was silently inert, which is the precise failure the warning text
/// exists to prevent.
///
/// Calling the function proves the function. This builds a node and requires
/// the warning to come out of a real process's stderr, which is the part that
/// was missing. The child is this same test binary — the one that deliberately
/// does not install the allocator.
#[test]
fn building_a_no_alloc_node_warns_that_nothing_is_enforcing_it() {
    const CHILD: &str = "HORUS_NO_ALLOC_WARN_CHILD";

    struct Inert;
    impl Node for Inert {
        fn name(&self) -> &str {
            "unenforced_controller"
        }
        fn tick(&mut self) {}
    }

    if std::env::var_os(CHILD).is_some() {
        let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz());
        scheduler
            .add(Inert)
            .order(0)
            .rate(1000_u64.hz())
            .no_alloc()
            .build()
            .expect("a .no_alloc() RT node must still build — this warns, it does not fail");
        return;
    }

    let exe = std::env::current_exe().expect("test binary path");
    let out = std::process::Command::new(exe)
        .args([
            "--exact",
            "building_a_no_alloc_node_warns_that_nothing_is_enforcing_it",
            "--nocapture",
            "--test-threads",
            "1",
        ])
        .env(CHILD, "1")
        .output()
        .expect("re-running the test binary should work");

    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(
        stdout.contains("1 passed"),
        "the child ran no test — was this test renamed?\n{stdout}"
    );

    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.contains(".no_alloc() IS NOT BEING ENFORCED"),
        "registering a .no_alloc() node in a binary with no RtAwareAllocator \
         printed no warning, so the user is told nothing.\nstderr:\n{stderr}"
    );
    assert!(
        stderr.contains("unenforced_controller"),
        "the warning must name the node that asked for the guarantee.\nstderr:\n{stderr}"
    );
}

/// Without the allocator, `.no_alloc()` is inert — allocating inside the bracket
/// is not detected. Asserting the no-op keeps the documentation honest and makes
/// the warning above load-bearing rather than decorative.
///
/// This is the exact allocation that
/// `rt_alloc_guard_installed::an_allocation_inside_an_rt_tick_is_detected_counted_and_panics`
/// catches. The only difference between the two binaries is the
/// `#[global_allocator]` line.
#[test]
fn without_the_allocator_an_allocating_rt_tick_goes_undetected() {
    let before = rt_allocator::violation_count();

    let outcome = std::panic::catch_unwind(|| {
        let _guard = enter_rt_context_guarded("unprotected_node");
        let v: Vec<u8> = Vec::with_capacity(64);
        std::hint::black_box(&v);
    });

    assert!(
        outcome.is_ok(),
        "with no RtAwareAllocator installed nothing inspects the RT flag, so this \
         allocation must pass silently — that no-op is the documented behaviour \
         and the reason .no_alloc() has to warn at registration"
    );
    assert_eq!(
        rt_allocator::violation_count(),
        before,
        "an uninstalled allocator cannot count violations"
    );
    assert!(
        !rt_allocator::is_rt_context(),
        "the guard must still clear the RT flag"
    );
}

// ───────────────────────────── bracket plumbing ──────────────────────────────

/// The RT bracket wraps the canonical robotics hot path — receive a POD message,
/// do control math, publish a POD message — without disturbing it.
///
/// **This does not prove the loop is allocation-free.** No allocator is
/// installed in this binary, so nothing inspects the flag; that proof lives in
/// `rt_alloc_guard_installed::steady_state_topic_recv_compute_send_does_not_allocate`,
/// which runs the same loop with the allocator live. What is covered here is the
/// bracket itself: `enter_rt_context` takes a borrowed `&str` and allocates
/// nothing per tick (it once `Box::leak`ed a fresh `String` every tick — a
/// per-tick heap allocation plus an unbounded leak), the guard clears the flag
/// on both exits, and 200 warm send/recv cycles run to completion under it.
///
/// The warmup outside the bracket is kept because it is part of the shape the
/// sibling test depends on: a `Topic`'s SHM backend is initialised lazily on
/// first `recv`/`send` and that one-time init allocates.
#[test]
fn rt_bracket_wraps_the_steady_state_topic_path_without_disturbing_it() {
    let _shm_guard = cleanup_stale_shm();

    let reading_topic = common::unique("rt.noalloc.reading");
    let command_topic = common::unique("rt.noalloc.command");

    let reading_pub = Topic::<Reading>::new(&reading_topic).expect("create reading pub");
    let reading_sub = Topic::<Reading>::new(&reading_topic).expect("create reading sub");
    let command_pub = Topic::<Command>::new(&command_topic).expect("create command pub");

    // ── Warmup (outside the RT context) ──────────────────────────────────────
    reading_pub.send(Reading {
        value: 1.0,
        seq: 0.0,
    });
    let _ = reading_sub.recv();
    command_pub.send(Command {
        output: 0.0,
        seq: 0.0,
    });

    // ── Steady state (inside the RT context) ─────────────────────────────────
    let mut state = 0.0f32;
    for i in 0..200u32 {
        reading_pub.send(Reading {
            value: i as f32,
            seq: i as f32,
        });

        let _guard = enter_rt_context_guarded("rt_control_node");
        // recv: zero-copy read out of the SHM ring (POD memcpy).
        if let Some(r) = reading_sub.recv() {
            state = r.value * 0.5; // trivial control law
        }
        // send: zero-copy write into the SHM ring (POD memcpy).
        command_pub.send(Command {
            output: state,
            seq: i as f32,
        });
    }

    assert!(
        !rt_allocator::is_rt_context(),
        "the guard must clear the RT flag at the end of every tick"
    );
    assert_eq!(
        state,
        199.0 * 0.5,
        "control loop should have processed readings"
    );
}

/// Ticks the executor-driven `.no_alloc()` controller completed. A `static` +
/// atomic so incrementing it never allocates.
static EXEC_TICKS: AtomicU64 = AtomicU64::new(0);

/// End-to-end through the RT executor: a `.no_alloc()` control node ticking
/// `recv → compute → send` runs continuously.
///
/// This exercises the executor's **warmup exemption** — the first tick is not
/// alloc-checked, so the `Topic`'s lazy SHM-backend init may allocate — followed
/// by enforcement from the second tick on. That combination is what makes
/// `.no_alloc()` usable for real control nodes.
///
/// In *this* binary the enforcement half is inert (no allocator installed), so
/// what is covered here is the executor plumbing: the flag is entered and left
/// around each tick and the node keeps running.
/// `rt_alloc_guard_installed::a_clean_no_alloc_node_survives_the_rt_executor`
/// runs the same graph with the allocator live and asserts zero violations.
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
        .add(Producer {
            out: prod_out,
            n: 0,
        })
        .order(0)
        .build()
        .unwrap();
    // Real-time (`.rate()`) + `.no_alloc()` ⇒ runs on the RT executor with the
    // allocation bracket around every tick from the second on.
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
         (tick 1 warms up the topic backend, steady state is bracketed); got {ticks}."
    );
}
