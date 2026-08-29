//! The `.no_alloc()` guarantee, proved with the allocator **actually installed**.
//!
//! This is the only binary in the repository where `RtAwareAllocator` is the
//! `#[global_allocator]`, and it exists because until now nothing was: the flag
//! was inert everywhere, including in the test whose doc comment claimed to
//! install it. A `.no_alloc()` node could allocate freely on its RT path and
//! nothing noticed.
//!
//! An integration test target is its own binary, so it may legitimately install
//! a global allocator. `horus_core` itself must not — it is a library, and a
//! library that installs a `#[global_allocator]` hijacks its consumer's binary.
//! Installation stays a documented one-line downstream opt-in; this file is the
//! proof that the opt-in works, and `rt_no_alloc.rs` is the matching proof of
//! what happens when you skip it.
//!
//! ## What is asserted here
//!
//! | Test | Claim |
//! |---|---|
//! | `the_guard_is_actually_installed_in_this_binary` | the runtime probe sees the allocator, and `warn_if_unenforced` stays quiet |
//! | `an_allocation_inside_an_rt_tick_is_detected_counted_and_panics` | a violation is caught, counted, and reported |
//! | `a_tick_that_does_not_allocate_is_not_flagged` | no false positives |
//! | `steady_state_topic_recv_compute_send_does_not_allocate` | the canonical robotics hot path is really allocation-free |
//! | `a_clean_no_alloc_node_survives_the_rt_executor` | end-to-end, no false positives through the executor |
//! | `an_allocating_no_alloc_node_is_caught_by_the_rt_executor` | end-to-end detection, contained to the offending node |
//!
//! ## Why the counter and not just the panic
//!
//! A violation panics inside `tick()`, and `NodeRunner::run_tick` catches that
//! panic — so from outside the executor there is nothing to observe unless the
//! node's `FailurePolicy` happens to escalate. The default policy does not.
//! `rt_allocator::violation_count()` is therefore the assertion target: it is
//! bumped on the cold path before the panic and survives any policy.

// The exact three lines `rt_allocator`'s module docs tell users to paste into
// their `main.rs`. Compiled here so the documented path can never rot again —
// it previously read `horus_core::memory::RtAwareAllocator`, which does not
// exist, so the copy-paste snippet did not build.
#[global_allocator]
static ALLOC: horus_core::memory::rt_allocator::RtAwareAllocator =
    horus_core::memory::rt_allocator::RtAwareAllocator;

mod common;
use common::cleanup_stale_shm;

use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::memory::rt_allocator;
use horus_core::scheduling::Scheduler;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, MutexGuard};

/// `violation_count()` is process-global and monotonic, so any test that reads a
/// delta must not run concurrently with another that can bump it. Tests take
/// this before `cleanup_stale_shm()` so the two locks are always acquired in the
/// same order.
fn counter_gate() -> MutexGuard<'static, ()> {
    static GATE: Mutex<()> = Mutex::new(());
    // A test that panics inside the gate poisons it; the poison carries no
    // meaning here (the counter is an atomic, not a broken invariant).
    GATE.lock().unwrap_or_else(|e| e.into_inner())
}

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

// ───────────────────────────── installation ──────────────────────────────────

/// Everything else in this file is meaningless if the allocator is not really
/// installed, so assert that first — and assert it through the same runtime
/// probe that `warn_if_unenforced` uses at node registration, so this test also
/// covers the probe's positive case. Its negative case is covered by the lib
/// unit test and by `rt_no_alloc.rs`.
#[test]
fn the_guard_is_actually_installed_in_this_binary() {
    assert!(
        rt_allocator::is_installed(),
        "RtAwareAllocator is this binary's #[global_allocator], so the runtime \
         probe must detect it; if this fails the probe is broken and every \
         .no_alloc() user would be told they are protected when they are not"
    );
    assert!(
        rt_allocator::warn_if_unenforced("installed_probe_node"),
        "with the allocator installed, registration must report enforcement as \
         live and print no warning"
    );
    assert!(
        !rt_allocator::is_rt_context(),
        "the probe must restore the RT flag it borrowed"
    );
}

// ─────────────────────────── direct detection ────────────────────────────────

/// The core claim: an allocation inside the RT bracket is detected.
///
/// Also pins the *containment* half of the chosen violation behaviour — the
/// panic is an ordinary unwind that `catch_unwind` sees, not an abort. The
/// allocator clears the RT flag before building its diagnostic precisely so the
/// panic machinery's own allocations do not re-enter and turn this into a
/// `SIGABRT`.
#[test]
fn an_allocation_inside_an_rt_tick_is_detected_counted_and_panics() {
    let _gate = counter_gate();
    let before = rt_allocator::violation_count();

    let outcome = std::panic::catch_unwind(|| {
        let _guard = rt_allocator::enter_rt_context_guarded("offending_node");
        let v: Vec<u8> = Vec::with_capacity(64);
        std::hint::black_box(&v);
    });

    assert!(
        outcome.is_err(),
        "allocating inside a .no_alloc() bracket must panic the tick"
    );
    assert_eq!(
        rt_allocator::violation_count() - before,
        1,
        "the violation must be counted exactly once, so telemetry sees it even \
         when the node's FailurePolicy discards the panic"
    );
    assert!(
        !rt_allocator::is_rt_context(),
        "the RT flag must be clear after the violation unwinds"
    );
}

/// No false positives. With a global allocator installed process-wide, a guard
/// that fired on compliant work would be worse than no guard — it would make
/// `.no_alloc()` unusable and train operators to remove it.
#[test]
fn a_tick_that_does_not_allocate_is_not_flagged() {
    let _gate = counter_gate();
    let before = rt_allocator::violation_count();

    let mut acc = 0.0f64;
    {
        let _guard = rt_allocator::enter_rt_context_guarded("clean_node");
        for i in 0..10_000u32 {
            acc += f64::from(i) * 0.5;
        }
        std::hint::black_box(acc);
    }

    assert_eq!(
        rt_allocator::violation_count(),
        before,
        "arithmetic-only work must not be reported as an allocation"
    );
    assert!(acc > 0.0);
}

/// The claim `rt_no_alloc.rs` used to make and could not back: the canonical
/// robotics hot path — receive a POD message, do control math, publish a POD
/// message — is allocation-free once the topic backends are warm.
///
/// A `Topic`'s SHM backend is initialised lazily on first `recv`/`send`, and
/// that one-time init allocates, so the warmup runs outside the bracket. What is
/// measured is the steady state, which is where a control loop spends
/// essentially all of its time.
#[test]
fn steady_state_topic_recv_compute_send_does_not_allocate() {
    let _gate = counter_gate();
    let _shm_guard = cleanup_stale_shm();

    let reading_topic = common::unique("rt.guard.reading");
    let command_topic = common::unique("rt.guard.command");

    let reading_pub = Topic::<Reading>::new(&reading_topic).expect("create reading pub");
    let reading_sub = Topic::<Reading>::new(&reading_topic).expect("create reading sub");
    let command_pub = Topic::<Command>::new(&command_topic).expect("create command pub");

    // Warmup, outside the bracket: lazy backend init is allowed to allocate.
    reading_pub.send(Reading {
        value: 1.0,
        seq: 0.0,
    });
    let _ = reading_sub.recv();
    command_pub.send(Command {
        output: 0.0,
        seq: 0.0,
    });

    let before = rt_allocator::violation_count();
    let mut state = 0.0f32;

    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        for i in 0..200u32 {
            reading_pub.send(Reading {
                value: i as f32,
                seq: i as f32,
            });

            let _guard = rt_allocator::enter_rt_context_guarded("rt_control_node");
            if let Some(r) = reading_sub.recv() {
                state = r.value * 0.5;
            }
            command_pub.send(Command {
                output: state,
                seq: i as f32,
            });
        }
    }));

    let violations = rt_allocator::violation_count() - before;
    assert!(
        outcome.is_ok() && violations == 0,
        "the steady-state POD topic path allocated ({violations} violation(s)). \
         This is now measurable for the first time — before this binary existed \
         no allocator was installed anywhere, so the same loop 'passed' without \
         checking anything. A failure here is a real finding about Topic::send / \
         Topic::recv, not about the guard."
    );
    assert_eq!(
        state,
        199.0 * 0.5,
        "control loop should have processed readings"
    );
}

/// The tensor-backed publish path — allocate a pooled frame, publish it, receive
/// it — does not touch the heap.
///
/// This is the path a camera or lidar node runs, and it is the one where a
/// hidden allocation is easiest to miss: the pixel buffer comes from a
/// shared-memory pool rather than the heap, so `Image::new` *looks* like it
/// cannot allocate, and until recently it did anyway — `vec![height, width,
/// channels]` built a heap Vec purely to be copied into `Tensor`'s fixed shape
/// array and dropped.
///
/// Publishing is a descriptor copy and a refcount bump, so it should be free of
/// the allocator regardless of frame size. 64x64 keeps the scrub on drop cheap;
/// what is under test is the allocation behaviour, not the throughput.
#[test]
fn tensor_backed_publish_does_not_allocate() {
    use horus_core::memory::Image;
    use horus_core::types::ImageEncoding;

    let _gate = counter_gate();
    let _shm_guard = cleanup_stale_shm();

    let name = common::unique("rt.guard.frames");
    let tx = Topic::<Image>::new(&name).expect("create frame pub");
    let rx = Topic::<Image>::new(&name).expect("create frame sub");

    // Warmup outside the bracket: lazy pool creation, SHM mapping and backend
    // selection are all allowed to allocate once.
    let warm = Image::new(64, 64, ImageEncoding::Mono8).expect("warmup frame");
    tx.send(warm);
    let _ = rx.recv();

    let before = rt_allocator::violation_count();
    let mut received = 0u32;

    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        for i in 0..64u32 {
            let _guard = rt_allocator::enter_rt_context_guarded("rt_camera_node");
            let frame = Image::new(64, 64, ImageEncoding::Mono8).expect("frame");
            frame.data_mut()[0] = i as u8;
            tx.send(frame);
            if rx.recv().is_some() {
                received += 1;
            }
        }
    }));

    let violations = rt_allocator::violation_count() - before;
    assert!(
        outcome.is_ok() && violations == 0,
        "the tensor-backed publish path allocated ({violations} violation(s)). \
         Image::new draws its buffer from a shared-memory pool, so a heap \
         allocation here is incidental bookkeeping rather than the payload — \
         find it and put it on the stack, do not relax this test."
    );
    assert!(
        received > 0,
        "the subscriber received nothing, so the loop above proved nothing"
    );
}

/// The serde send path reuses its scratch buffer rather than allocating a
/// serialisation buffer per message.
///
/// A non-POD topic — anything with a `Drop`, so anything carrying a `String` or
/// a `Vec` — cannot move its value through the ring as raw bytes and has to
/// serialise. That is a per-message encode on the publish path, and the obvious
/// implementation allocates a fresh buffer for it every time.
///
/// `dispatch::serialize_into_scratch` writes into `local.serde_scratch`
/// instead, so the buffer is allocated once per handle. This pins that: if the
/// scratch is ever removed or accidentally re-created per send, an RT publisher
/// of a non-POD message starts taking the allocator lock at its tick rate.
///
/// Only the SEND side is asserted. Receiving a `String` has to allocate the
/// `String` — that is the message, not overhead — so a no-alloc receive of an
/// owned type is not a property worth claiming.
#[test]
fn serde_send_reuses_its_scratch_buffer() {
    #[derive(Clone, serde::Serialize, serde::Deserialize)]
    struct Named {
        // A `String` field makes this non-POD (`needs_drop`), which is what
        // routes it to the serde path rather than the POD one.
        label: String,
        value: f32,
    }

    let _gate = counter_gate();
    let _shm_guard = cleanup_stale_shm();

    let name = common::unique("rt.guard.serde");
    let tx = Topic::<Named>::new(&name).expect("create serde pub");

    // Warmup outside the bracket, with a message the SAME size as the ones
    // below. The scratch is a `Vec<u8>` that grows to fit the largest message
    // encoded so far and is then cleared (keeping capacity) per send, so the
    // first message larger than any before it costs one realloc. That growth is
    // amortised and is not what this test is about — warming with a SHORTER
    // label made the first loop iteration pay it and the test fail for the
    // wrong reason.
    tx.send(Named {
        label: "sensor".to_string(),
        value: 0.0,
    });

    // Build the messages up front. Constructing the `String` allocates, and
    // that is the caller's cost, not the transport's — measuring it here would
    // make the test fail for a reason it is not about.
    let msgs: Vec<Named> = (0..64u32)
        .map(|i| Named {
            label: "sensor".to_string(),
            value: i as f32,
        })
        .collect();

    let before = rt_allocator::violation_count();
    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        // Consumed by value: `send` takes ownership, and cloning inside the
        // guarded region would allocate the `String` again and count against
        // the transport. Only `alloc` is instrumented, so the drop of each
        // message inside `send` is not counted either way.
        for m in msgs {
            let _guard = rt_allocator::enter_rt_context_guarded("rt_serde_node");
            tx.send(m);
        }
    }));

    let violations = rt_allocator::violation_count() - before;
    assert!(
        outcome.is_ok() && violations == 0,
        "the serde send path allocated ({violations} violation(s)) beyond the \
         message itself. The scratch buffer in LocalState exists so that \
         encoding does not allocate per message; if it has been removed, an RT \
         publisher of a non-POD message now takes the allocator lock every tick."
    );
}

// ─────────────────────────── through the executor ────────────────────────────

/// Ticks completed by the clean executor-driven node. A `static` + atomic so
/// incrementing it never allocates.
static CLEAN_TICKS: AtomicU64 = AtomicU64::new(0);

/// End-to-end negative control: a real `.no_alloc()` node on the RT executor,
/// doing the recv → compute → send work a controller does, with the allocator
/// live. This is the assertion that `.no_alloc()` is usable rather than merely
/// enforceable.
///
/// It also exercises the executor's warmup exemption — tick 1 is not checked, so
/// the topic's lazy SHM init may allocate; enforcement starts at tick 2.
#[test]
fn a_clean_no_alloc_node_survives_the_rt_executor() {
    let _gate = counter_gate();
    let _shm_guard = cleanup_stale_shm();

    let reading_topic = common::unique("rt.guard.exec.reading");
    let command_topic = common::unique("rt.guard.exec.command");

    struct Producer {
        out: Topic<Reading>,
        n: u64,
    }
    impl Node for Producer {
        fn name(&self) -> &str {
            "clean_producer"
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
            "clean_controller"
        }
        fn tick(&mut self) {
            if let Some(r) = self.input.recv() {
                self.state = r.value * 0.5;
            }
            self.output.send(Command {
                output: self.state,
                seq: 0.0,
            });
            CLEAN_TICKS.fetch_add(1, Ordering::Relaxed);
        }
    }

    let prod_out = Topic::<Reading>::new(&reading_topic).expect("create reading pub");
    let ctrl_in = Topic::<Reading>::new(&reading_topic).expect("create reading sub");
    let ctrl_out = Topic::<Command>::new(&command_topic).expect("create command pub");

    let before = rt_allocator::violation_count();

    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz());
    scheduler
        .add(Producer {
            out: prod_out,
            n: 0,
        })
        .order(0)
        .build()
        .unwrap();
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

    let ticks = CLEAN_TICKS.load(Ordering::Relaxed);
    let violations = rt_allocator::violation_count() - before;
    assert_eq!(
        violations, 0,
        "a compliant .no_alloc() controller tripped the guard {violations} time(s) \
         over {ticks} ticks"
    );
    assert!(
        ticks > 20,
        "the .no_alloc() controller should have run many ticks through the RT \
         executor; got {ticks}. A low count means it panicked."
    );
}

/// Ticks attempted by the deliberately-offending node.
static OFFENDER_TICKS: AtomicU64 = AtomicU64::new(0);

/// The test the task exists for: a `.no_alloc()` node that allocates on its RT
/// path is **detected**, end to end, through the real executor.
///
/// The node allocates on exactly its second and third ticks. The first is
/// exempt (the executor's warmup exemption for lazy initialisation), so at least
/// one of those two must be caught however the exemption is counted — and after
/// they pass, the node stops allocating, which bounds how much diagnostic noise
/// this test produces.
///
/// No topics are involved on purpose: this isolates "the guard catches an
/// allocation" from "the topic path is clean", which
/// `steady_state_topic_recv_compute_send_does_not_allocate` covers separately.
///
/// The second assertion is about robot behaviour, not detection: the offending
/// node keeps ticking afterwards. The violation panics *that tick*, the executor
/// catches it, and the node's `FailurePolicy` decides what happens next — the
/// default being log-and-continue. The RT thread must not die, and the scheduler
/// must not stop, because one node allocated.
#[test]
fn an_allocating_no_alloc_node_is_caught_by_the_rt_executor() {
    let _gate = counter_gate();
    let _shm_guard = cleanup_stale_shm();

    struct Offender {
        ticks: u64,
    }
    impl Node for Offender {
        fn name(&self) -> &str {
            "offending_controller"
        }
        fn tick(&mut self) {
            self.ticks += 1;
            OFFENDER_TICKS.store(self.ticks, Ordering::Relaxed);
            if self.ticks == 2 || self.ticks == 3 {
                // Exactly the kind of thing that must never appear in a
                // hard-real-time tick: an unbounded-latency heap allocation.
                let v: Vec<u8> = Vec::with_capacity(128);
                std::hint::black_box(&v);
            }
        }
    }

    let before = rt_allocator::violation_count();

    // `verbose(false)` only trims the executor's optional `[RT-thread]` chatter;
    // the violation itself is reported by the allocator and counted regardless.
    let mut scheduler = Scheduler::new().tick_rate(1000_u64.hz()).verbose(false);
    scheduler
        .add(Offender { ticks: 0 })
        .rate(500_u64.hz())
        .no_alloc()
        .build()
        .unwrap();

    scheduler.run_for(200_u64.ms()).unwrap();

    let violations = rt_allocator::violation_count() - before;
    let ticks = OFFENDER_TICKS.load(Ordering::Relaxed);

    assert!(
        violations >= 1,
        "a .no_alloc() node allocated on its RT path and nothing detected it \
         ({violations} violations over {ticks} ticks). This is exactly the \
         no-op guarantee this binary exists to prevent."
    );
    assert!(
        violations <= 2,
        "only ticks 2 and 3 allocate, so at most two violations are possible; \
         got {violations}, which means the warmup exemption or the counter is wrong"
    );
    // Deliberately loose: this asserts containment (the node outlived the
    // violating ticks), not throughput. ~100 ticks are expected at 500 Hz over
    // 200 ms, but this machine has no CPU isolation and a powersave governor, so
    // a tight bound here would be a flake, not a measurement.
    assert!(
        ticks > 5,
        "the offending node must keep ticking after the violation — the panic is \
         contained to one tick and routed through the node's FailurePolicy, it \
         must not kill the RT thread or stop the scheduler; got {ticks} ticks"
    );
}
