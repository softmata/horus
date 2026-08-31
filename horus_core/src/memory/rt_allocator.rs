//! RT-aware allocator for detecting heap allocations during real-time ticks.
//!
//! When a node has `.no_alloc()` set, the executor that runs it brackets
//! `tick()` with [`enter_rt_context`] / [`leave_rt_context`] (RT executor:
//! `rt_executor::tick_node`; deterministic mode: `Scheduler::run_node_tick`).
//! If — and only if — [`RtAwareAllocator`] is the process's
//! `#[global_allocator]`, any allocation inside that bracket is a violation.
//!
//! # Setup (downstream opt-in — deliberately NOT automatic)
//!
//! `horus_core` is a **library**. A library cannot install a `#[global_allocator]`
//! into a user's binary without hijacking it, so installation is one line the
//! user writes in their own `main.rs`:
//!
//! ```rust,ignore
//! #[global_allocator]
//! static ALLOC: horus_core::memory::rt_allocator::RtAwareAllocator =
//!     horus_core::memory::rt_allocator::RtAwareAllocator;
//! ```
//!
//! That exact snippet is compiled and executed by
//! `horus_core/tests/rt_alloc_guard_installed.rs`, which then asserts that a
//! `.no_alloc()` node which allocates is actually caught. (It used to be written
//! here as `horus_core::memory::RtAwareAllocator`, a path that does not exist —
//! `memory/mod.rs` re-exports no such name. Nothing compiled the snippet, so the
//! one line users were told to copy did not build.)
//!
//! # If you do not install it, you are not protected — and you can find out
//!
//! Without the `#[global_allocator]` line, `.no_alloc()` is inert: the bracket
//! still toggles the thread-local flag, but nothing consults it. That is a
//! silent downgrade from "this node is proven allocation-free" to "this node is
//! unchecked", which is the worst possible failure mode for a safety property.
//!
//! [`is_installed`] answers the question at runtime, so the downgrade can be
//! *reported* rather than assumed away. [`warn_if_unenforced`] is the intended
//! call at node registration: it returns whether enforcement is live and emits
//! one process-wide warning if it is not.
//!
//! # What happens on a violation
//!
//! The chosen behaviour is **panic inside the tick, contained to that node, then
//! routed through the node's declared [`FailurePolicy`]**, plus a monotonic
//! process-wide counter for telemetry. Concretely:
//!
//! 1. [`violation_count`] is incremented, and a diagnostic naming the node is
//!    written to stderr.
//! 2. The allocating call panics. It does **not** abort: the RT flag is cleared
//!    *before* the diagnostic is built (the panic path itself allocates) and the
//!    check is skipped while `std::thread::panicking()`, so a violation can never
//!    escalate into a panic-during-panic `SIGABRT`.
//! 3. `NodeRunner::run_tick` catches the unwind, so the panic never reaches the
//!    executor loop and never kills the RT thread.
//! 4. The executor's failure arm records a tick failure, marks the node
//!    `Unhealthy` after `FAILURES_BEFORE_UNHEALTHY` consecutive ones, writes a
//!    `BlackBoxEvent::NodeError`, calls `Node::on_error`, and applies the node's
//!    `FailurePolicy` → `FailureAction` (`RestartNode`, `SkipNode`,
//!    `StopScheduler`/`FatalAfterRestarts` → `enter_safe_state()` then stop).
//!
//! Why panic rather than log-and-continue: by the time `alloc` is reached the
//! timing guarantee the operator asked for is *already* broken — this tick may
//! block on a page fault, an `mmap`, or an arena lock for an unbounded time.
//! Truncating that tick is strictly better than letting it run to completion and
//! emitting a late actuator command. Why *not* abort the process: a robot mid-
//! motion is more dangerous stopped dead than degraded, so the decision belongs
//! to the node's `FailurePolicy`, not to the allocator.
//!
//! **Caveat worth knowing before you rely on this.** `NodeRegistration`'s
//! default `failure_policy` is `None`, which `apply_failure_policy_after_panic`
//! treats as log-and-continue. A `.no_alloc()` node with no explicit policy
//! therefore keeps ticking and keeps violating — detected and counted, but not
//! acted on. Pair `.no_alloc()` with an explicit `.failure_policy(...)` if you
//! want the violation to change what the robot does. [`violation_count`] is the
//! channel that stays truthful either way.
//!
//! # Cost
//!
//! - Without the `#[global_allocator]` line: zero. The system allocator runs.
//! - With it, no `.no_alloc()` node ticking: one thread-local load and a
//!   not-taken branch per allocation.
//! - With it, inside a `.no_alloc()` tick: the same load and branch. The whole
//!   diagnostic-and-panic path lives in one `#[cold] #[inline(never)]` function
//!   so it is *not* inlined into every `alloc` call site — the hot path stays a
//!   few instructions and the icache footprint of the process's hottest function
//!   does not grow. That is a tail/jitter property, not a median one: the cold
//!   body would otherwise be duplicated at every inlined allocation site.
//!
//! [`FailurePolicy`]: crate::scheduling::fault_tolerance::FailurePolicy

use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Once, OnceLock};

thread_local! {
    /// Whether the current thread is in an RT tick context where allocations are forbidden.
    static RT_ALLOC_CONTEXT: Cell<bool> = const { Cell::new(false) };
    /// Borrowed pointer to the name of the node currently being ticked (for panic
    /// messages). Set by [`enter_rt_context`] from a `&str` that outlives the
    /// enter..leave window; it is never owned and never allocated. Read only while
    /// [`RT_ALLOC_CONTEXT`] is set.
    static RT_NODE_NAME: Cell<Option<*const str>> = const { Cell::new(None) };
    /// Set only by [`probe_global_allocator`]. Turns the violation path into a
    /// silent observation: no counter bump, no diagnostic, no panic, and the
    /// allocation is served normally. Read exclusively from the cold path, so
    /// this costs nothing when no violation occurs.
    static RT_ALLOC_PROBE: Cell<bool> = const { Cell::new(false) };
    /// Set by the cold path when it observes a probe allocation — i.e. when
    /// [`RtAwareAllocator`] really is the global allocator.
    static RT_ALLOC_PROBE_TRIPPED: Cell<bool> = const { Cell::new(false) };
}

/// Monotonic count of detected RT allocation violations, process-wide.
///
/// Bumped on the cold path only, so it adds nothing to a compliant tick.
static RT_ALLOC_VIOLATIONS: AtomicU64 = AtomicU64::new(0);

/// Number of heap allocations detected inside a `.no_alloc()` RT context since
/// process start.
///
/// This is the telemetry channel for the guarantee, and the one signal that
/// stays truthful when a node's `FailurePolicy` swallows the panic (the default
/// policy does — see the module docs). Monotonic and never reset, so a monitor
/// samples it as a delta.
///
/// Always `0` when [`RtAwareAllocator`] is not the `#[global_allocator]` — a
/// zero here means "no violations *or* no enforcement", so check
/// [`is_installed`] before reading it as a clean bill of health.
pub fn violation_count() -> u64 {
    RT_ALLOC_VIOLATIONS.load(Ordering::Relaxed)
}

/// Enter the RT allocation-free context for the named node.
///
/// Called by the executor before `tick()` when the node has `.no_alloc()`.
/// Any heap allocation after this call (and before [`leave_rt_context`]) is a
/// violation — but only if [`RtAwareAllocator`] is the global allocator. See
/// [`warn_if_unenforced`].
pub fn enter_rt_context(node_name: &str) {
    // Store a *borrowed* raw pointer to the caller's name — no allocation, no
    // leak. The pointer stays valid until `leave_rt_context()` because the node
    // (and its `Arc<str>` name) outlives the tick that enter/leave bracket.
    //
    // This runs on the RT hot path *every tick* a `.no_alloc()` node executes, so
    // it must not allocate. (A previous version `Box::leak`ed a fresh `String`
    // here every tick — a per-tick heap allocation plus an unbounded leak.)
    RT_NODE_NAME.with(|c| c.set(Some(node_name as *const str)));
    RT_ALLOC_CONTEXT.with(|c| c.set(true));
}

/// Leave the RT allocation-free context.
///
/// Called by the RT executor after `tick()`. Allocations are allowed again.
pub fn leave_rt_context() {
    RT_ALLOC_CONTEXT.with(|c| c.set(false));
    // Drop the borrowed name pointer so it can never be read outside the window.
    RT_NODE_NAME.with(|c| c.set(None));
}

/// Leaves the RT allocation-free context on drop — including when `tick()`
/// unwinds.
///
/// The executors used to call [`leave_rt_context`] on the straight-line path
/// after `tick()`, so a panicking tick skipped it: the thread was left with the
/// RT flag set and, worse, with `RT_NODE_NAME` still holding a raw pointer into
/// a `&str` whose borrow window had closed. Bracket the tick with this guard
/// instead and both are cleared on either exit.
pub struct RtContextGuard(());

impl Drop for RtContextGuard {
    fn drop(&mut self) {
        leave_rt_context();
    }
}

/// Enter the RT allocation-free context, leaving it when the guard is dropped.
///
/// Prefer this over the bare [`enter_rt_context`] / [`leave_rt_context`] pair:
/// see [`RtContextGuard`] for what the straight-line pair leaks on an unwind.
pub fn enter_rt_context_guarded(node_name: &str) -> RtContextGuard {
    enter_rt_context(node_name);
    RtContextGuard(())
}

/// Read the current RT node's name from the borrowed pointer set by
/// [`enter_rt_context`]. Returns `"<unknown>"` if none is set.
///
/// # Safety
/// Sound to call only while [`RT_ALLOC_CONTEXT`] is `true`: the pointer was set
/// from a `&str` that outlives the enter..leave window, so the borrow is valid.
#[inline]
fn rt_node_name<'a>() -> &'a str {
    match RT_NODE_NAME.with(|c| c.get()) {
        // SAFETY: see fn docs — pointer valid whenever the RT flag is set.
        Some(p) => unsafe { &*p },
        None => "<unknown>",
    }
}

/// Check if the current thread is in an RT allocation-free context.
pub fn is_rt_context() -> bool {
    RT_ALLOC_CONTEXT.with(|c| c.get())
}

// ─────────────────────────── installation detection ──────────────────────────

/// Whether [`RtAwareAllocator`] is actually this process's `#[global_allocator]`
/// — i.e. whether `.no_alloc()` enforces anything at all.
///
/// There is no reflective way to ask Rust what the global allocator is, so this
/// performs one controlled experiment, exactly once per process, and caches the
/// answer: it sets the RT flag on the calling thread, marks the allocation as a
/// *probe* (which makes the violation path silent and non-panicking), then routes
/// one small allocation through `std::alloc::alloc` — the `#[global_allocator]`
/// entry point. If [`RtAwareAllocator`] is installed, the cold path observes the
/// probe and sets a flag; if it is not, the allocation goes straight to the
/// system allocator and nothing is observed.
///
/// This costs one allocation, once, and adds **nothing** to the allocation hot
/// path — it rides the branch that already exists.
///
/// Call it at startup or node registration. It allocates, so do not call it from
/// inside an RT tick; it is safe to call from anywhere else, including while
/// another thread is mid-tick (all the state it touches is thread-local).
pub fn is_installed() -> bool {
    static INSTALLED: OnceLock<bool> = OnceLock::new();
    *INSTALLED.get_or_init(probe_global_allocator)
}

/// One-shot experiment backing [`is_installed`]. See its docs for the mechanism.
fn probe_global_allocator() -> bool {
    // Save every flag we touch: this may run on a thread that is (or later will
    // be) inside a real RT context, and it must leave no trace.
    let saved_ctx = RT_ALLOC_CONTEXT.with(|c| c.get());
    let saved_probe = RT_ALLOC_PROBE.with(|c| c.get());
    let saved_tripped = RT_ALLOC_PROBE_TRIPPED.with(|c| c.get());

    RT_ALLOC_PROBE_TRIPPED.with(|c| c.set(false));
    RT_ALLOC_PROBE.with(|c| c.set(true));
    RT_ALLOC_CONTEXT.with(|c| c.set(true));

    // `from_size_align` cannot fail for a power-of-two align and a size that
    // does not overflow when rounded up; `unwrap_or` keeps this allocation-free
    // regardless (a failing `expect` would format a message).
    let layout = Layout::from_size_align(64, 16).unwrap_or(Layout::new::<u64>());
    // SAFETY: `layout` has non-zero size; the returned pointer is written to
    // within `layout.size()` bytes and freed with the same layout through the
    // same global allocator.
    unsafe {
        let p = std::alloc::alloc(layout);
        if !p.is_null() {
            // Write, then escape the pointer. LLVM is allowed to delete an
            // alloc/dealloc pair whose result never escapes — that optimisation
            // would make the probe answer "not installed" in release builds
            // while answering "installed" in debug, i.e. it would silently
            // disable the very warning this function exists to emit.
            std::ptr::write_bytes(p, 0xA5, layout.size());
            std::hint::black_box(p);
            std::alloc::dealloc(p, layout);
        }
    }

    let tripped = RT_ALLOC_PROBE_TRIPPED.with(|c| c.get());

    RT_ALLOC_CONTEXT.with(|c| c.set(saved_ctx));
    RT_ALLOC_PROBE.with(|c| c.set(saved_probe));
    RT_ALLOC_PROBE_TRIPPED.with(|c| c.set(saved_tripped));

    tripped
}

/// Report whether `.no_alloc()` is actually enforced in this process, warning
/// once if it is not.
///
/// Returns `true` when [`RtAwareAllocator`] is the `#[global_allocator]` and the
/// guarantee is real. Returns `false` — and, the first time only, prints an
/// explanation naming `node_name` and the one line that fixes it — when it is
/// not, so a user who wrote `.no_alloc()` learns that they are unprotected
/// instead of assuming they are.
///
/// Intended call site: node registration, once per `.no_alloc()` node. The
/// warning is process-wide-once (a robot with twenty RT nodes should print one
/// banner, not twenty) and the cached [`is_installed`] probe makes every call
/// after the first a single atomic load.
///
/// This deliberately warns rather than fails. Refusing to build a `.no_alloc()`
/// node without the allocator would make the flag unusable for prototyping and
/// would effectively force installation on library consumers, which is exactly
/// what a library must not do.
pub fn warn_if_unenforced(node_name: &str) -> bool {
    if is_installed() {
        return true;
    }
    static WARNED: Once = Once::new();
    WARNED.call_once(|| {
        crate::terminal::eprint_line(&format!(
            "\n\x1b[1;33m.no_alloc() IS NOT BEING ENFORCED\x1b[0m\n\
             Node '{node_name}' requests allocation-free ticks, but this binary does not\n\
             install HORUS's allocator, so nothing checks it. The flag is inert and the\n\
             node's RT path may be allocating freely.\n\
             \n\
             Fix: add these three lines to your main.rs:\n\
             \n\
             \x20   #[global_allocator]\n\
             \x20   static ALLOC: horus_core::memory::rt_allocator::RtAwareAllocator =\n\
             \x20       horus_core::memory::rt_allocator::RtAwareAllocator;\n\
             \n\
             (Reported once per process. Other .no_alloc() nodes are equally unchecked.)\n"
        ));
    });
    false
}

// ───────────────────────────────── allocator ─────────────────────────────────

/// Global allocator that detects heap allocation during an RT tick context.
///
/// Delegates everything to the system allocator. When [`enter_rt_context`] has
/// been called on the current thread, an allocation is counted
/// ([`violation_count`]), reported, and panicked on — see the module docs for
/// why that is the right behaviour on a robot and what the executor does with
/// the panic.
///
/// # Usage
///
/// ```rust,ignore
/// #[global_allocator]
/// static ALLOC: horus_core::memory::rt_allocator::RtAwareAllocator =
///     horus_core::memory::rt_allocator::RtAwareAllocator;
/// ```
pub struct RtAwareAllocator;

/// Handle a detected allocation inside an RT context.
///
/// `#[cold]` + `#[inline(never)]` on purpose: this body contains `format!`, the
/// panic machinery and a stderr write, none of which should be inlined into the
/// process's hottest function at every allocation site. Keeping it out of line
/// leaves each `GlobalAlloc` method as a thread-local load, a statically
/// not-taken branch, and a tail call to `System`.
///
/// Returns normally (and the caller then serves the allocation) only for the
/// [`is_installed`] probe; in every other case it panics.
#[cold]
#[inline(never)]
fn rt_allocation_violation(what: &str) {
    if RT_ALLOC_PROBE.with(|c| c.get()) {
        // This is `probe_global_allocator` asking whether we are installed.
        // Observe it and get out of the way: no counter, no output, no panic.
        RT_ALLOC_PROBE_TRIPPED.with(|c| c.set(true));
        return;
    }

    // Clear the flag BEFORE building the diagnostic: the panic/eprintln path
    // itself allocates, and if the flag were still set that alloc would re-enter
    // this branch and double-panic into a *process abort*. Clearing it first
    // turns detection into a catchable per-node panic the executor can route
    // through the node's FailurePolicy — not a whole-robot crash.
    RT_ALLOC_CONTEXT.with(|c| c.set(false));

    // Count before reporting, so the telemetry channel is truthful even if the
    // stderr write is swallowed or the panic is discarded by the failure policy.
    RT_ALLOC_VIOLATIONS.fetch_add(1, Ordering::Relaxed);

    let name = rt_node_name();
    crate::terminal::eprint_line(&format!(
        "\n\x1b[1;31mHEAP ALLOCATION IN RT TICK!\x1b[0m\n\
         Node '{name}' performed a {what} during tick().\n\
         This causes unpredictable latency in real-time code.\n\
         \n\
         Fix: Remove Vec::push(), String::from(), format!(), Box::new(),\n\
         or any other heap allocation from {name}::tick().\n\
         \n\
         To allow allocations (prototyping), remove .no_alloc() from the node builder.\n"
    ));
    panic!("{what} in RT tick of '{name}'. Remove allocations from tick() or remove .no_alloc().");
}

/// True when the current allocation must be treated as an RT violation.
///
/// The `!panicking()` half covers a hazard distinct from the flag itself: when
/// `tick()` panics for an unrelated reason, the panic runtime allocates (boxing
/// the payload, formatting the message) while the RT flag is still set, because
/// `leave_rt_context()` has not run yet. Panicking again from inside that
/// allocation is a panic-during-panic, which Rust turns into `SIGABRT` — an
/// ordinary recoverable node bug became a whole-process abort that
/// `catch_unwind` never saw. `std::thread::panicking()` reads a const-initialized
/// thread-local counter that std increments *before* running the panic hook, and
/// it cannot allocate or re-enter here.
#[inline(always)]
fn is_violation() -> bool {
    RT_ALLOC_CONTEXT.with(|c| c.get()) && !std::thread::panicking()
}

unsafe impl GlobalAlloc for RtAwareAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if is_violation() {
            rt_allocation_violation("heap allocation");
        }
        System.alloc(layout)
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // Deallocation is deliberately unchecked. `free` is not the unbounded
        // operation `malloc` is, and a `.no_alloc()` tick that drops a value
        // allocated before the bracket opened is legitimate — flagging it would
        // make the guard unusable.
        System.dealloc(ptr, layout)
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        if is_violation() {
            rt_allocation_violation("zeroed heap allocation");
        }
        System.alloc_zeroed(layout)
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        if is_violation() {
            rt_allocation_violation("heap reallocation");
        }
        System.realloc(ptr, layout, new_size)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rt_context_enter_leave() {
        assert!(!is_rt_context());
        enter_rt_context("test_node");
        assert!(is_rt_context());
        leave_rt_context();
        assert!(!is_rt_context());
    }

    #[test]
    fn test_rt_context_default_off() {
        assert!(!is_rt_context());
    }

    #[test]
    fn rt_context_guard_leaves_on_unwind() {
        let r = std::panic::catch_unwind(|| {
            let _guard = enter_rt_context_guarded("guarded_node");
            assert!(is_rt_context());
            panic!("tick failed");
        });
        assert!(r.is_err());
        assert!(
            !is_rt_context(),
            "the guard must leave the RT context even when tick() unwinds"
        );
    }

    // `is_installed`, `warn_if_unenforced` and `violation_count` are deliberately
    // NOT unit-tested here. What they report is a property of the *binary* — which
    // allocator it registered — and a lib unit test can only ever observe one side
    // of it. Both sides are covered by a matched pair of integration binaries:
    //
    //   tests/rt_alloc_guard_installed.rs  installs RtAwareAllocator; asserts the
    //     probe detects it, that an allocating `.no_alloc()` node is caught and
    //     counted end-to-end through the RT executor, and that compliant nodes are
    //     not flagged.
    //   tests/rt_no_alloc.rs               deliberately does not install it; asserts
    //     the probe reports enforcement as absent, that `warn_if_unenforced` says so,
    //     and that the documented no-op really is a no-op.

    /// A panic inside a `.no_alloc()` tick used to be escalated from a caught
    /// panic to a whole-process `SIGABRT`: the panic runtime allocates while the
    /// RT flag is still set, `alloc` panicked again, and a panic during a panic
    /// aborts. This exercises the allocator directly (it is not the global
    /// allocator in the lib test binary) from inside a `Drop` running on the
    /// unwinding path, where `std::thread::panicking()` is true. Before the fix
    /// this test aborted the whole test binary rather than failing.
    #[test]
    fn alloc_defers_to_the_system_allocator_while_a_panic_unwinds() {
        use std::sync::atomic::{AtomicBool, Ordering};

        static ALLOCATED_WHILE_UNWINDING: AtomicBool = AtomicBool::new(false);

        struct AllocOnUnwind;
        impl Drop for AllocOnUnwind {
            fn drop(&mut self) {
                let layout = Layout::from_size_align(64, 8).unwrap();
                // SAFETY: matched alloc/dealloc of a valid non-zero layout.
                unsafe {
                    let p = RtAwareAllocator.alloc(layout);
                    ALLOCATED_WHILE_UNWINDING.store(!p.is_null(), Ordering::SeqCst);
                    if !p.is_null() {
                        RtAwareAllocator.dealloc(p, layout);
                    }
                }
            }
        }

        let r = std::panic::catch_unwind(|| {
            let _guard = enter_rt_context_guarded("panicky");
            let _alloc_on_unwind = AllocOnUnwind;
            panic!("an ordinary node bug");
        });

        assert!(r.is_err(), "the original panic must reach catch_unwind");
        assert!(
            ALLOCATED_WHILE_UNWINDING.load(Ordering::SeqCst),
            "an allocation during unwinding must fall through to the system allocator"
        );
        assert!(!is_rt_context());
    }
}
