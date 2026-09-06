//! What a helper thread inherits from the thread that spawned it.
//!
//! A thread inherits both its creator's scheduling policy (`copy_process`
//! duplicates `policy` and `rt_priority`, and glibc's `pthread_create` defaults
//! to `PTHREAD_INHERIT_SCHED`) and its creator's CPU mask (`cpus_allowed` is
//! copied by `dup_task_struct`, and no kernel flag resets it). HORUS applies its
//! RT setup on the thread that builds the scheduler, *before* the lifecycle
//! hooks run — so before this suite existed, the net replicator, the telemetry
//! HTTP thread, the log drain, the diagnostic drain and every per-goal action
//! thread came up at real-time priority, pinned to the cores reserved for the
//! control loop, and nothing in the tree asserted otherwise.
//!
//! # Why every test here carries its own ablation
//!
//! Each assertion is paired with a plain `std::thread::spawn` from the
//! identical parent state. If the plain child *also* comes back clean, the
//! inheritance this suite is guarding against is not happening on this machine
//! and the test would be passing for the wrong reason — so that case fails
//! loudly rather than going green.
//!
//! `SCHED_BATCH` stands in for `SCHED_FIFO` throughout: it is inherited by the
//! identical kernel mechanism and, unlike `SCHED_FIFO`, it is settable without
//! `CAP_SYS_NICE`, which no developer machine and no CI container has.

#![cfg(target_os = "linux")]

use horus_sys::rt::{
    best_effort_demotions, current_affinity, helper_thread_cpus, set_best_effort_class,
};

/// `SCHED_RESET_ON_FORK`, which the kernel ORs into `sched_getscheduler`.
const RESET_ON_FORK: libc::c_int = 0x4000_0000;

/// Serialises the tests in this file.
///
/// `reserve_rt_cpus` accumulates into one process-wide set for the lifetime of
/// the process — correct in production, where RT cores stay reserved — but
/// `cargo test` runs this binary's tests concurrently in one process, so
/// without this a test could compute `helper_thread_cpus()` and then have
/// another test reserve a core before it reads a child's mask back.
static SERIALISE: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn serialised() -> std::sync::MutexGuard<'static, ()> {
    SERIALISE.lock().unwrap_or_else(|e| e.into_inner())
}

fn policy_now() -> libc::c_int {
    // SAFETY: pid 0 = current thread; `sched_getscheduler` only reads.
    unsafe { libc::sched_getscheduler(0) & !RESET_ON_FORK }
}

fn set_policy(policy: libc::c_int) -> Result<(), std::io::Error> {
    // SAFETY: `sched_param` is a POD struct of integer fields for which
    // all-zero is a valid bit pattern; SCHED_OTHER and SCHED_BATCH both require
    // `sched_priority == 0`.
    let rc = unsafe {
        let mut param: libc::sched_param = std::mem::zeroed();
        param.sched_priority = 0;
        libc::sched_setscheduler(0, policy, &param)
    };
    if rc == 0 {
        Ok(())
    } else {
        Err(std::io::Error::last_os_error())
    }
}

/// Puts the calling thread back on SCHED_OTHER with its original mask when
/// dropped, including on a panic.
///
/// `cargo test` runs every test in this binary as a thread of one process, so a
/// test that leaked a narrowed mask or a non-default policy would corrupt
/// whichever test the harness scheduled onto that thread next.
struct ThreadStateGuard {
    mask: Vec<usize>,
}

impl ThreadStateGuard {
    fn capture() -> Self {
        Self {
            mask: current_affinity(),
        }
    }
}

impl Drop for ThreadStateGuard {
    fn drop(&mut self) {
        let _ = set_policy(libc::SCHED_OTHER);
        let _ = horus_sys::rt::pin_to_cores(&self.mask);
    }
}

/// The state a helper thread must not inherit: a non-default policy and a
/// one-CPU mask. Returns the CPU it narrowed to.
fn narrow_and_batch() -> usize {
    let usable = current_affinity();
    let hi = *usable.last().expect("a thread runs on at least one CPU");
    horus_sys::rt::pin_to_cores(&[hi]).expect("narrowing to one CPU is always permitted");
    set_policy(libc::SCHED_BATCH).unwrap_or_else(|e| {
        // Deliberately not a skip. A silent skip here is a guard that proves
        // nothing, and this suite exists precisely because nothing was
        // asserting on thread inheritance.
        panic!(
            "could not put the calling thread on SCHED_BATCH ({e}) — unprivileged \
             SCHED_BATCH is standard Linux behaviour, so a refusal means a seccomp \
             filter or hardened kernel that this suite cannot test under"
        )
    });
    hi
}

#[test]
fn a_helper_thread_does_not_inherit_a_narrowed_cpu_mask() {
    let _serial = serialised();
    let _guard = ThreadStateGuard::capture();
    // Capture before narrowing, exactly as the scheduler does.
    horus_sys::rt::capture_helper_baseline_cpus();
    let expected = helper_thread_cpus();
    let hi = narrow_and_batch();

    let via_helper = horus_core::scheduling::spawn_best_effort("t-helper", 0, current_affinity)
        .expect("spawn")
        .join()
        .expect("join");

    let plain = std::thread::spawn(current_affinity).join().expect("join");

    assert_eq!(
        via_helper, expected,
        "a thread spawned through spawn_best_effort must run on the helper CPU set, \
         not on the mask it inherited"
    );
    assert_eq!(
        plain,
        vec![hi],
        "ABLATION: a plain std::thread::spawn from the same parent must still \
         inherit the narrowed mask. If it does not, this kernel is not inheriting \
         affinity and the assertion above proves nothing."
    );
}

#[test]
fn a_helper_thread_does_not_inherit_a_non_default_scheduling_policy() {
    let _serial = serialised();
    let _guard = ThreadStateGuard::capture();
    horus_sys::rt::capture_helper_baseline_cpus();
    narrow_and_batch();

    let via_helper = horus_core::scheduling::spawn_best_effort("t-policy", 0, policy_now)
        .expect("spawn")
        .join()
        .expect("join");

    let plain = std::thread::spawn(policy_now).join().expect("join");

    assert_eq!(
        via_helper,
        libc::SCHED_OTHER,
        "a helper thread must be demoted to the ordinary time-shared class"
    );
    assert_eq!(
        plain,
        libc::SCHED_BATCH,
        "ABLATION: a plain std::thread::spawn must still inherit SCHED_BATCH. If it \
         does not, this kernel is not inheriting the policy and the assertion above \
         proves nothing."
    );
}

#[test]
fn entering_the_best_effort_class_on_a_correct_thread_issues_no_syscall() {
    let _serial = serialised();
    let _guard = ThreadStateGuard::capture();
    horus_sys::rt::capture_helper_baseline_cpus();

    // Establish the precondition explicitly rather than assuming it: another
    // test in this binary may already have reserved a core, so "the mask a
    // helper belongs on" is `helper_thread_cpus()`, not the raw baseline.
    horus_sys::rt::pin_to_cores(&helper_thread_cpus()).expect("pin to the helper set");

    // Already SCHED_OTHER on the helper mask: there is nothing to undo, and a
    // fix that taxed this path would put two syscalls on every thread spawned
    // by every process that never configured RT at all — which is every
    // developer machine and every CI job.
    // `policy_changed` / `affinity_changed` are not a summary of what happened,
    // they ARE the branch the syscalls live inside: `set_best_effort_class`
    // issues `sched_setscheduler` only when it sets the first and
    // `sched_setaffinity` only when it sets the second. All-false across 100
    // calls is therefore the "no syscall" assertion, stated per call.
    //
    // The process-wide counter is deliberately NOT asserted by equality here:
    // `cargo test` runs this binary's tests concurrently and the other tests in
    // this file demote on purpose, so an equality check would be measuring
    // their threads as well as this one. It is checked for monotonicity below,
    // which is all it can honestly promise.
    let before = best_effort_demotions();
    for _ in 0..100 {
        let report = set_best_effort_class(0);
        assert!(!report.policy_changed, "nothing to demote: {report:?}");
        assert!(!report.affinity_changed, "nothing to re-pin: {report:?}");
        assert!(report.refusal.is_none(), "{report:?}");
    }

    // And the complement, so all-false above cannot be a function that does
    // nothing at all.
    narrow_and_batch();
    let report = set_best_effort_class(0);
    assert!(report.policy_changed, "{report:?}");
    assert!(report.affinity_changed, "{report:?}");
    assert!(
        best_effort_demotions() > before,
        "a demotion that actually did something must reach the counter"
    );
}

/// The blocking pool is grown lazily, at runtime, from whichever thread is
/// executing the runtime — which in the scheduler is the one
/// `apply_rt_optimizations` just put on SCHED_FIFO. So the first slow async
/// node materialises a pool thread at real-time priority on the reserved core,
/// minutes into a run, with nothing in the logs.
///
/// This also pins a dependency-version-sensitive fact: `on_thread_start`
/// reaches blocking-pool threads even on a `new_current_thread` runtime.
#[test]
fn tokio_blocking_pool_threads_are_best_effort() {
    let _serial = serialised();
    let _guard = ThreadStateGuard::capture();
    horus_sys::rt::capture_helper_baseline_cpus();
    let expected = helper_thread_cpus();
    let hi = narrow_and_batch();

    let gated = tokio::runtime::Builder::new_current_thread()
        .enable_time()
        .on_thread_start(|| {
            horus_core::scheduling::enter_best_effort("tokio-blocking", 5);
        })
        .build()
        .expect("runtime");
    let (policy, mask) = gated.block_on(async {
        tokio::task::spawn_blocking(|| (policy_now(), current_affinity()))
            .await
            .expect("blocking task")
    });
    assert_eq!(policy, libc::SCHED_OTHER);
    assert_eq!(mask, expected);

    let ungated = tokio::runtime::Builder::new_current_thread()
        .enable_time()
        .build()
        .expect("runtime");
    let (policy, mask) = ungated.block_on(async {
        tokio::task::spawn_blocking(|| (policy_now(), current_affinity()))
            .await
            .expect("blocking task")
    });
    assert_eq!(
        policy,
        libc::SCHED_BATCH,
        "ABLATION: without the hook a blocking-pool thread must still inherit the policy"
    );
    assert_eq!(
        mask,
        vec![hi],
        "ABLATION: without the hook a blocking-pool thread must still inherit the mask"
    );
}

// ---------------------------------------------------------------------------
// The load-bearing test: every thread a running scheduler creates.
// ---------------------------------------------------------------------------

/// Read a task's scheduling policy, reset-on-fork bit stripped.
fn policy_of_tid(tid: i32) -> Option<libc::c_int> {
    // SAFETY: `sched_getscheduler` only reads, and a stale tid returns -1/ESRCH
    // rather than misbehaving.
    let raw = unsafe { libc::sched_getscheduler(tid) };
    (raw >= 0).then_some(raw & !RESET_ON_FORK)
}

/// Read a task's CPU mask as kernel CPU ids.
fn affinity_of_tid(tid: i32) -> Option<Vec<usize>> {
    // SAFETY: `set` is a stack `cpu_set_t` for which all-zero is a valid bit
    // pattern and `sched_getaffinity` only writes into it.
    let mut set: libc::cpu_set_t = unsafe { std::mem::zeroed() };
    let rc =
        unsafe { libc::sched_getaffinity(tid, std::mem::size_of::<libc::cpu_set_t>(), &mut set) };
    if rc != 0 {
        return None;
    }
    let mut cpus = Vec::new();
    for cpu in 0..libc::CPU_SETSIZE as usize {
        // SAFETY: `cpu` is below CPU_SETSIZE, the bound the macro indexes within.
        if unsafe { libc::CPU_ISSET(cpu, &set) } {
            cpus.push(cpu);
        }
    }
    Some(cpus)
}

fn live_tids() -> Vec<i32> {
    std::fs::read_dir("/proc/self/task")
        .map(|d| {
            d.filter_map(|e| e.ok())
                .filter_map(|e| e.file_name().to_string_lossy().parse::<i32>().ok())
                .collect()
        })
        .unwrap_or_default()
}

fn comm_of_tid(tid: i32) -> String {
    std::fs::read_to_string(format!("/proc/self/task/{tid}/comm"))
        .map(|s| s.trim().to_string())
        .unwrap_or_default()
}

struct Ticker;

impl horus_core::core::Node for Ticker {
    fn name(&self) -> &'static str {
        "ticker"
    }
    fn init(&mut self) -> horus_core::error::Result<()> {
        Ok(())
    }
    fn tick(&mut self) {}
}

macro_rules! ticker {
    ($n:literal) => {{
        struct N;
        impl horus_core::core::Node for N {
            fn name(&self) -> &'static str {
                $n
            }
            fn init(&mut self) -> horus_core::error::Result<()> {
                Ok(())
            }
            fn tick(&mut self) {}
        }
        N
    }};
}

/// The one test that catches a *missed* call site.
///
/// The fix is a wide, shallow diff — a dozen spawn sites across five crates —
/// and a missed one is invisible: the thread still starts, still works, and
/// shows up only as jitter under load. So rather than asserting on the sites
/// that were converted, this walks `/proc/self/task` after a real scheduler run
/// and asserts on every thread that appeared, whatever created it.
#[test]
fn every_thread_a_running_scheduler_creates_is_best_effort() {
    use horus_core::core::DurationExt;

    let _serial = serialised();
    let _guard = ThreadStateGuard::capture();
    horus_sys::rt::capture_helper_baseline_cpus();

    // Reserve the highest CPU for "RT" and hand the scheduler the same core, so
    // it narrows its OWN thread onto it through `apply_rt_optimizations` — the
    // real ordering, rather than a simulation of it — and adds nothing new to
    // the process-global reservation while it runs. (The SCHED_FIFO half of
    // that setup is refused without CAP_SYS_NICE; the affinity half always
    // applies, and it is the half with no kernel backstop, since
    // `SCHED_RESET_ON_FORK` does not reset a CPU mask.)
    let hi = *current_affinity().last().expect("at least one CPU");
    horus_sys::rt::reserve_rt_cpus(&[hi]);
    let expected = helper_thread_cpus();
    assert!(
        !expected.contains(&hi) || expected.len() == 1,
        "the helper set must exclude the reserved core unless there is only one CPU"
    );

    let before: std::collections::HashSet<i32> = live_tids().into_iter().collect();

    // The scheduler runs on its own thread and the sweep happens WHILE it runs:
    // the compute, async-io and event threads are joined during shutdown, so a
    // sweep taken afterwards sees only the ones that outlive the run and would
    // pass by inspecting almost nothing.
    //
    // A plain `std::thread::spawn` on purpose — `spawn_best_effort` would demote
    // the scheduler thread before it ever got the chance to narrow itself.
    let runner_tid = std::sync::Arc::new(std::sync::atomic::AtomicI32::new(0));
    let runner_tid_w = std::sync::Arc::clone(&runner_tid);
    let runner = std::thread::spawn(move || {
        // SAFETY: `gettid` takes no arguments and cannot fail.
        runner_tid_w.store(
            unsafe { libc::syscall(libc::SYS_gettid) } as i32,
            std::sync::atomic::Ordering::SeqCst,
        );
        let mut sched = horus_core::scheduling::Scheduler::new()
            .cores(&[hi])
            .tick_rate(1000_u64.hz());
        sched.add(Ticker).rate(500_u64.hz()).build().unwrap();
        sched
            .add(ticker!("second"))
            .rate(500_u64.hz())
            .build()
            .unwrap();
        sched.add(ticker!("crunch")).compute().build().unwrap();
        sched.add(ticker!("io")).async_io().build().unwrap();
        sched
            .add(ticker!("watcher"))
            .on("/helper_isolation_probe")
            .build()
            .unwrap();
        sched.run_for(900_u64.ms()).unwrap();
        current_affinity()
    });

    // Let every executor start. Not a timing assertion: the sweep's own
    // "how many did I see" check below is what decides whether this was long
    // enough, so a slow machine fails loudly rather than passing vacuously.
    std::thread::sleep(std::time::Duration::from_millis(400));

    let mut horus_helpers = 0usize;
    let mut offenders: Vec<String> = Vec::new();
    let mut seen: Vec<String> = Vec::new();
    for tid in live_tids() {
        if before.contains(&tid) {
            continue;
        }
        // The scheduler's own thread is supposed to be on the RT core — it is
        // the thread `apply_rt_optimizations` narrowed, and the one everything
        // below inherits from.
        if tid == runner_tid.load(std::sync::atomic::Ordering::SeqCst) {
            continue;
        }
        let comm = comm_of_tid(tid);
        // The RT tick threads are supposed to keep their policy and their
        // cores; they are the reason the reservation exists.
        if comm.starts_with("horus-rt") && comm != "horus-rt-diag" {
            continue;
        }
        let (Some(policy), Some(mask)) = (policy_of_tid(tid), affinity_of_tid(tid)) else {
            // Exited between the listing and the read. Not an offender.
            continue;
        };
        seen.push(comm.clone());

        if policy != libc::SCHED_OTHER {
            offenders.push(format!("{comm} (tid {tid}) is on policy {policy}"));
        }

        if comm.starts_with("horus") || comm.starts_with("tokio") {
            // A thread HORUS named is a thread HORUS spawned, and every one of
            // those goes through `spawn_best_effort`; a tokio thread is covered
            // by the `on_thread_start` hook on both runtimes. Exact equality
            // here is what catches a spawn site missed in the conversion: a
            // missed site shows up carrying the RT core in its mask.
            horus_helpers += 1;
            if mask != expected {
                offenders.push(format!("{comm} (tid {tid}) is on CPUs {mask:?}"));
            }
        } else {
            // A thread a third-party crate spawned — in practice `ctrl-c`,
            // whose thread `ctrlc::set_handler` creates with no hook to
            // configure it. The scheduler now registers that handler before
            // `apply_rt_optimizations` narrows anything, so it holds the
            // unrestricted baseline rather than the RT core. What must hold is
            // the weaker but still meaningful property: it is not CONFINED to
            // the CPUs reserved for the control loop.
            if mask.iter().all(|c| *c == hi) {
                offenders.push(format!(
                    "{comm} (tid {tid}) is confined to the reserved RT core: {mask:?}"
                ));
            }
        }
    }

    let scheduler_mask = runner.join().expect("the scheduler thread must not panic");

    assert_eq!(
        scheduler_mask,
        vec![hi],
        "the scheduler must actually have narrowed its own thread onto the RT core — \
         otherwise nothing was there to inherit and the sweep proves nothing"
    );
    assert!(
        offenders.is_empty(),
        "threads created by the scheduler inherited the RT policy or the RT core mask \
         (helper set is {expected:?}, threads seen were {seen:?}): {offenders:?}"
    );
    assert!(
        horus_helpers >= 3,
        "expected to inspect at least 3 HORUS-named helper threads, saw \
         {horus_helpers} ({seen:?}) — the scheduler did not start its helpers, so \
         the assertion above proved nothing"
    );
}
