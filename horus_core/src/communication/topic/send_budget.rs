//! The wall-clock budget the lossy publish path may spend before it drops.
//!
//! # Why a budget, and why it has to be anchored at entry
//!
//! [`Topic::send`](super::Topic::send) is the lossy publish: its contract is
//! "never block, never fail — drop". When the ring is full it takes a cold
//! path that spins, then yields, then gives up. `std::thread::yield_now` is a
//! scheduling *request* with no upper bound on when the thread is rescheduled;
//! four unconditional yields on a full ring measured 26.4 ms mean / 58.6 ms
//! worst with 32 competing threads, and 109.6 ms / 199.0 ms with 128, on a call
//! made from a loop that may be running at 1 kHz. A robot running more nodes
//! than it has cores is oversubscribed by design, so that is the ordinary case.
//!
//! The budget that was supposed to bound this could not. Its check was the last
//! statement of the yield loop, so the first yield was unconditional — a
//! deadline tested *after* a blocking operation bounds the number of blocking
//! operations, never the time spent in them. And its clock was anchored on the
//! line immediately above the loop, so even moving the check to the top of the
//! body would have read a few nanoseconds on the first iteration and never
//! fired.
//!
//! Both halves matter. The deadline is now anchored at function entry, where it
//! absorbs the migration check, the participant sweep and the spin phase, so a
//! pre-yield check has something real to observe.
//!
//! # Why an RT thread never yields here
//!
//! Under `SCHED_FIFO`, `sched_yield` moves the thread to the tail of its own
//! priority runqueue and returns only once every same-priority peer has
//! blocked. That is unbounded by construction rather than by load, and it is
//! the exact thing a control loop cannot afford. So a thread that the kernel
//! actually granted a real-time policy runs this path with a zero budget: one
//! attempt, no yields, drop. [`mark_current_thread_realtime`] is how the RT
//! executor says so, once, after the kernel has answered.

use std::cell::Cell;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;

/// Wall-clock the lossy publish cold path may spend before it gives up.
///
/// Far above the ~1.4 µs four yields cost on an idle machine, so nothing
/// changes when the box is quiet; it only bites once yielding has demonstrably
/// become expensive.
pub const DEFAULT_SEND_RETRY_BUDGET: Duration = Duration::from_micros(200);

/// Environment override, in microseconds. `0` means "never yield".
pub const SEND_RETRY_BUDGET_ENV: &str = "HORUS_SEND_RETRY_BUDGET_US";

/// Process-wide budget in nanoseconds. `u64::MAX` = not yet resolved.
static BUDGET_NS: AtomicU64 = AtomicU64::new(u64::MAX);

/// `yield_now()` calls made from the publish cold path, process-wide.
static YIELDS: AtomicU64 = AtomicU64::new(0);

/// `try_send` attempts made from the publish cold path, process-wide.
static ATTEMPTS: AtomicU64 = AtomicU64::new(0);

thread_local! {
    /// Whether the kernel granted this thread a real-time policy.
    static THREAD_IS_RT: Cell<bool> = const { Cell::new(false) };
}

/// Declare whether the calling thread holds a real-time scheduling policy.
///
/// Called once by the RT executor after the kernel has answered, never on the
/// tick path. It costs one TLS store per thread at startup and one TLS load per
/// cold-path entry.
pub fn mark_current_thread_realtime(is_rt: bool) {
    // `try_with`, not `with`: a `with` on a thread whose TLS is already being
    // destroyed panics, and topics are dropped during thread teardown.
    let _ = THREAD_IS_RT.try_with(|c| c.set(is_rt));
}

/// Override the retry budget process-wide. `Duration::ZERO` means "never
/// yield": one attempt, then drop.
pub fn set_send_retry_budget(budget: Duration) {
    BUDGET_NS.store(
        budget.as_nanos().min(u64::MAX as u128) as u64,
        Ordering::Relaxed,
    );
}

/// The retry budget in effect for the calling thread.
#[inline]
pub fn send_retry_budget() -> Duration {
    if THREAD_IS_RT.try_with(Cell::get).unwrap_or(false) {
        return Duration::ZERO;
    }
    match BUDGET_NS.load(Ordering::Relaxed) {
        u64::MAX => resolve_from_env(),
        ns => Duration::from_nanos(ns),
    }
}

/// Read the override once and cache it. Cold: reached only on the first lossy
/// retry in a process that never called [`set_send_retry_budget`].
#[cold]
#[inline(never)]
fn resolve_from_env() -> Duration {
    let budget = std::env::var(SEND_RETRY_BUDGET_ENV)
        .ok()
        .and_then(|s| s.trim().parse::<u64>().ok())
        .map(Duration::from_micros)
        .unwrap_or(DEFAULT_SEND_RETRY_BUDGET);
    set_send_retry_budget(budget);
    budget
}

/// `yield_now`, counted. Cold path only.
#[inline]
pub(crate) fn yield_now_counted() {
    YIELDS.fetch_add(1, Ordering::Relaxed);
    std::thread::yield_now();
}

/// Record one cold-path `try_send` attempt.
#[inline]
pub(crate) fn note_attempt() {
    ATTEMPTS.fetch_add(1, Ordering::Relaxed);
}

/// Yields performed by the lossy publish path since process start.
///
/// This is the number a jitter investigation wants: it is monotonic, it is
/// exact, and unlike a wall-clock measurement it does not need a quiet machine
/// to mean something.
pub fn send_yield_total() -> u64 {
    YIELDS.load(Ordering::Relaxed)
}

/// Cold-path `try_send` attempts since process start.
pub fn send_retry_attempt_total() -> u64 {
    ATTEMPTS.load(Ordering::Relaxed)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_realtime_thread_gets_a_zero_budget() {
        std::thread::spawn(|| {
            set_send_retry_budget(DEFAULT_SEND_RETRY_BUDGET);
            assert_eq!(send_retry_budget(), DEFAULT_SEND_RETRY_BUDGET);
            mark_current_thread_realtime(true);
            assert_eq!(
                send_retry_budget(),
                Duration::ZERO,
                "a thread the kernel put on a real-time policy must never yield from \
                 a lossy send: under SCHED_FIFO the yield returns only when every \
                 same-priority peer blocks"
            );
            mark_current_thread_realtime(false);
            assert_eq!(send_retry_budget(), DEFAULT_SEND_RETRY_BUDGET);
        })
        .join()
        .unwrap();
    }

    #[test]
    fn the_realtime_flag_is_per_thread() {
        std::thread::spawn(|| {
            set_send_retry_budget(DEFAULT_SEND_RETRY_BUDGET);
            mark_current_thread_realtime(true);
            let elsewhere = std::thread::spawn(send_retry_budget).join().unwrap();
            assert_eq!(
                elsewhere, DEFAULT_SEND_RETRY_BUDGET,
                "marking one thread real-time must not silence yielding everywhere"
            );
        })
        .join()
        .unwrap();
    }
}
