//! Priority-inheriting mutex — the lock a 1 kHz control thread is allowed to wait on.
//!
//! # The problem this exists to solve
//!
//! Every other mutex in this workspace (`std::sync::Mutex`, `parking_lot::Mutex`)
//! is a plain futex. A plain futex has no idea who is waiting on it, so the
//! kernel has no reason to run the thread that holds it any sooner. On a
//! `SCHED_FIFO` system that is unbounded priority inversion:
//!
//! ```text
//!   prio 80  control     ─────────────┐ blocked on L ┌────────────
//!   prio 40  telemetry            ┌───┴──────────────┴───┐  runs freely
//!   prio 10  logger      ──[takes L]──[preempted]────────[resumes]──[releases L]
//! ```
//!
//! The logger holds `L`. The control thread blocks on `L`. Any mid-priority
//! thread now preempts the logger, and the control thread waits for as long as
//! that thread feels like running — a bound set by the *mid*-priority work,
//! which nobody analysed, rather than by the critical section, which somebody
//! did. This is the Mars Pathfinder failure verbatim, and on a joint controller
//! it shows up as a deadline miss that looks like a scheduling bug rather than
//! a locking bug.
//!
//! With `PTHREAD_PRIO_INHERIT` the kernel knows the waiter, so for as long as
//! the control thread is blocked the logger runs *at the control thread's
//! priority*. The telemetry thread cannot preempt it, and the control thread's
//! blocking time is bounded by the length of the critical section.
//!
//! # Platform support — read this before relying on it
//!
//! | Platform | Backing | Inheritance |
//! |----------|---------|-------------|
//! | Linux (glibc, musl) | `pthread_mutex_t` + `PTHREAD_PRIO_INHERIT` | **yes** |
//! | macOS | [`std::sync::Mutex`] | no |
//! | Windows | [`std::sync::Mutex`] | no |
//!
//! The API is identical everywhere so call sites need no `cfg`, but the
//! guarantee is not, and this module never pretends otherwise:
//! [`PiMutex::has_priority_inheritance`] answers for a specific mutex and
//! [`priority_inheritance_available`] answers for the platform. On Linux the
//! primitive also degrades at *runtime* — an old musl, a kernel without
//! `FUTEX_LOCK_PI`, a seccomp filter — in which case it becomes an ordinary
//! `pthread` mutex, logs a warning once, and reports `false`.
//!
//! macOS declares `PTHREAD_PRIO_INHERIT` in its headers, but Apple's
//! `pthread_mutexattr_setprotocol` does not implement inheritance; production
//! hard-RT for this framework is Linux (see [`super::set_realtime_priority`],
//! which is likewise unimplemented on macOS rather than faked).
//!
//! # Poisoning: this mutex does not poison, and that is deliberate
//!
//! [`std::sync::Mutex`] marks itself poisoned when a thread panics holding the
//! lock, so every later `lock()` returns `Err`. `pthread` has no such concept.
//! This type follows `pthread`, for three reasons:
//!
//! 1. **`unwrap()` on the RT path is the worst possible robot behaviour.** A
//!    poisoning `lock()` leaves a caller two choices: `.unwrap()`, which turns
//!    one thread's panic into a panic in the 1 kHz control thread — the actuator
//!    output stops mid-motion — or ignore the flag. There is no third option
//!    that helps a machine that is holding something heavy.
//! 2. **This codebase already ignores it.** Every `std::sync::Mutex` call site
//!    on a critical path here is written `.lock().unwrap_or_else(|e|
//!    e.into_inner())` (see `safety_monitor::take_pending_local_estop` and
//!    `NodeInfo::notify_event_registered`). Encoding a flag in the type that
//!    every caller immediately discards buys nothing and costs a `Result` on
//!    the hot path.
//! 3. **The guard still runs.** A panic unwinding through a [`PiMutexGuard`]
//!    releases the lock exactly as it releases a `std::sync::MutexGuard`. The
//!    difference is only whether the *next* acquirer is told about it.
//!
//! The consequence is real and callers must know it: if a thread panics halfway
//! through updating the protected data, the next `lock()` observes that
//! half-finished state with no warning. Guard invariants that a panic can break
//! by keeping the critical section short enough to be a single logical update,
//! or by checking them on acquire. The non-Linux fallback swallows the std
//! poison flag (`unwrap_or_else(|e| e.into_inner())`) so this behaviour is the
//! same on every platform, not just where the `pthread` backend runs.
//!
//! # Other differences from `std::sync::Mutex`
//!
//! * `PiMutex::new` allocates (see the note on relocation in the Linux impl),
//!   so it is not `const` and a `PiMutex` cannot live in a `static`. Construct
//!   it once at node setup, never on the hot path.
//! * Locking a `PiMutex` twice from the same thread deadlocks, exactly like
//!   `std::sync::Mutex`. glibc's PI path detects the self-deadlock (the kernel
//!   returns `EDEADLK`) and then deliberately blocks forever to preserve POSIX
//!   "normal mutex" semantics, so the failure looks identical to std's.
//!   `PTHREAD_MUTEX_ERRORCHECK` would turn that hang into a diagnosable error
//!   and is worth considering later; it is a separate behavioural change from
//!   the inheritance protocol this module is about.
//!
//! # Not yet done here
//!
//! This mutex is process-private: the attribute is left `PTHREAD_PROCESS_PRIVATE`
//! and the object is heap-allocated, so it cannot be placed in the shared-memory
//! segments in [`crate::shm`]. Cross-process priority inheritance needs
//! `PTHREAD_PROCESS_SHARED` plus a robust attribute (a peer that dies holding the
//! lock must not wedge the robot), which is a larger design with its own failure
//! modes. Nothing in the workspace is converted to `PiMutex` by the change that
//! introduced it — call-site migration has its own blast radius.

/// Only the pthread backend stores the protected value itself; the fallback
/// hands it to `std::sync::Mutex`.
#[cfg(target_os = "linux")]
use std::cell::UnsafeCell;
use std::fmt;
use std::ops::{Deref, DerefMut};

// ============================================================================
// Linux — pthread_mutex_t with PTHREAD_PRIO_INHERIT
// ============================================================================

#[cfg(target_os = "linux")]
mod imp {
    use super::*;
    use libc::c_int;

    /// Mutex protocol constants from `<pthread.h>`.
    ///
    /// `libc` 0.2 declares `pthread_mutexattr_{get,set}protocol` for
    /// `target_os = "linux"` but exports the protocol *constants* only for
    /// l4re, apple and aix, so the values are spelled out here. Both glibc and
    /// musl define them as the anonymous `enum { PTHREAD_PRIO_NONE,
    /// PTHREAD_PRIO_INHERIT, PTHREAD_PRIO_PROTECT }`, i.e. 0/1/2, and the
    /// numbering is ABI. `pi_mutex_setprotocol_round_trips_prio_inherit` in
    /// this module's tests checks the value against the live libc rather than
    /// trusting this comment.
    pub(super) const PTHREAD_PRIO_NONE: c_int = 0;
    pub(super) const PTHREAD_PRIO_INHERIT: c_int = 1;

    /// The raw lock, kept behind a `Box` by [`PiMutex`].
    ///
    /// # Why this is boxed and not stored inline
    ///
    /// A `pthread_mutex_t` is not a relocatable object. POSIX says copying one
    /// is undefined, and on Linux the PI path makes that concrete: glibc hands
    /// `&mutex->__data.__lock` to `futex(FUTEX_LOCK_PI)`, and the kernel keys
    /// the `rt_mutex` and the owner's PI waiter list on *that address*. Moving
    /// the bytes while any of that state is live leaves the kernel pointing at
    /// an address that is no longer the lock.
    ///
    /// Rust values move. `Box` buys a heap address the mutex holds for its
    /// entire life: moving a `PiMutex` moves a pointer, and `pthread_mutex_init`
    /// below runs on the final address, never on a stack temporary that is then
    /// relocated. The cost is one allocation per mutex (construction only, never
    /// on the hot path) and one pointer indirection per `lock()`.
    pub(super) struct RawLock {
        m: UnsafeCell<libc::pthread_mutex_t>,
        /// The protocol read back off the attribute that was actually passed to
        /// `pthread_mutex_init`, so [`PiMutex::has_priority_inheritance`] reports
        /// what the C library agreed to rather than what we asked for.
        protocol: c_int,
        /// Whether `pthread_mutex_init` succeeded on `m`.
        ///
        /// Only false on the path where init failed and `new` is about to
        /// panic: the `Box` is dropped by the unwind, and destroying a
        /// `pthread_mutex_t` that was never initialised is undefined. Cheaper
        /// and clearer than leaking the allocation to dodge the same problem.
        initialised: bool,
    }

    impl RawLock {
        pub(super) fn new() -> Box<Self> {
            // The mutex is zeroed here only to have a valid bit pattern to move
            // to the heap; it is not a usable mutex yet. `pthread_mutex_init`
            // below overwrites it in place, at the address it keeps for life.
            //
            // SAFETY: `libc::pthread_mutex_t` is a plain byte array with an
            // alignment marker, for which all-zero is a valid bit pattern.
            let mut raw = Box::new(RawLock {
                m: UnsafeCell::new(unsafe { std::mem::zeroed() }),
                protocol: PTHREAD_PRIO_NONE,
                initialised: false,
            });

            // SAFETY: `attr` is a POD struct owned by this frame;
            // `pthread_mutexattr_init` fully initialises it and every path
            // below destroys it exactly once before returning. `raw.m.get()`
            // points at heap storage this function exclusively owns and which
            // no other thread can observe yet.
            unsafe {
                let mut attr: libc::pthread_mutexattr_t = std::mem::zeroed();
                let attr_ok = libc::pthread_mutexattr_init(&mut attr) == 0;

                let mut protocol = PTHREAD_PRIO_NONE;
                if attr_ok
                    && libc::pthread_mutexattr_setprotocol(&mut attr, PTHREAD_PRIO_INHERIT) == 0
                {
                    // Read it back off the very object about to be handed to
                    // pthread_mutex_init. A libc that accepts the call and
                    // stores something else would otherwise be indistinguishable
                    // from one that honoured it.
                    let mut got: c_int = PTHREAD_PRIO_NONE;
                    if libc::pthread_mutexattr_getprotocol(&attr, &mut got) == 0 {
                        protocol = got;
                    }
                }

                let use_attr = attr_ok && protocol == PTHREAD_PRIO_INHERIT;
                let attr_ptr = if use_attr {
                    &attr as *const libc::pthread_mutexattr_t
                } else {
                    std::ptr::null()
                };

                let mut rc = libc::pthread_mutex_init(raw.m.get(), attr_ptr);
                if rc != 0 && use_attr {
                    // The attribute took PI but the mutex would not: fall back
                    // to a plain mutex rather than leaving the caller with no
                    // lock at all, and stop claiming inheritance.
                    protocol = PTHREAD_PRIO_NONE;
                    rc = libc::pthread_mutex_init(raw.m.get(), std::ptr::null());
                }

                if attr_ok {
                    libc::pthread_mutexattr_destroy(&mut attr);
                }

                // Only ENOMEM/EAGAIN/EPERM/EINVAL reach here, and none of them
                // is recoverable at this call site: there is no mutex. Failing
                // loudly at construction beats handing back an object whose
                // `lock()` is undefined.
                assert_eq!(
                    rc,
                    0,
                    "pthread_mutex_init failed: {}",
                    std::io::Error::from_raw_os_error(rc)
                );

                raw.protocol = protocol;
                raw.initialised = true;
            }

            if raw.protocol != PTHREAD_PRIO_INHERIT {
                warn_no_inheritance();
            }
            raw
        }

        #[inline]
        pub(super) fn protocol(&self) -> c_int {
            self.protocol
        }

        #[inline]
        pub(super) fn lock(&self) {
            // SAFETY: `self.m` was initialised by `RawLock::new` and has not
            // moved since — it lives inside a `Box` owned by the `PiMutex`.
            let rc = unsafe { libc::pthread_mutex_lock(self.m.get()) };
            if rc != 0 {
                lock_failed("pthread_mutex_lock", rc);
            }
        }

        #[inline]
        pub(super) fn try_lock(&self) -> bool {
            // SAFETY: as `lock` above.
            let rc = unsafe { libc::pthread_mutex_trylock(self.m.get()) };
            match rc {
                0 => true,
                libc::EBUSY => false,
                other => lock_failed("pthread_mutex_trylock", other),
            }
        }

        /// # Safety
        ///
        /// The calling thread must currently hold this mutex. `pthread` requires
        /// the unlocking thread to be the owner, which [`PiMutexGuard`]'s `!Send`
        /// bound plus its `Drop` are what actually guarantee.
        #[inline]
        pub(super) unsafe fn unlock(&self) {
            let rc = unsafe { libc::pthread_mutex_unlock(self.m.get()) };
            if rc != 0 {
                lock_failed("pthread_mutex_unlock", rc);
            }
        }

        /// Address of the raw `pthread_mutex_t`, for tests that inspect the
        /// libc representation directly.
        #[cfg(test)]
        pub(super) fn as_ptr(&self) -> *const libc::pthread_mutex_t {
            self.m.get()
        }
    }

    impl Drop for RawLock {
        fn drop(&mut self) {
            // `Drop` lives on `RawLock` rather than on `PiMutex` so that
            // `PiMutex::into_inner` can destructure the mutex and still destroy
            // the lock exactly once.
            //
            if !self.initialised {
                // `pthread_mutex_init` failed and `new` panicked; there is no
                // mutex here to destroy, only zeroed bytes.
                return;
            }
            // SAFETY: a live `PiMutexGuard` borrows the `PiMutex`, so no guard
            // can exist while this runs, which means the mutex is unlocked and
            // `pthread_mutex_destroy` is legal.
            let rc = unsafe { libc::pthread_mutex_destroy(self.m.get()) };
            debug_assert_eq!(
                rc,
                0,
                "pthread_mutex_destroy failed: {}",
                std::io::Error::from_raw_os_error(rc)
            );
        }
    }

    /// Report the loss of the inheritance guarantee once per process.
    ///
    /// Once, not per mutex: a node that builds a hundred of these on a libc
    /// without PI support must not turn one missing feature into a hundred log
    /// lines on a robot, but it must not stay quiet either.
    fn warn_no_inheritance() {
        static ONCE: std::sync::Once = std::sync::Once::new();
        ONCE.call_once(|| {
            log::warn!(
                "PiMutex: this C library refused PTHREAD_PRIO_INHERIT; locks are plain \
                 pthread mutexes and a high-priority waiter will NOT boost the holder. \
                 RT deadline analysis that assumes bounded blocking does not hold here."
            );
        });
    }

    /// Turn an unexpected `pthread` return code into a panic naming the call.
    ///
    /// Outlined and `#[cold]` so the success path of `lock`/`try_lock` stays a
    /// call and a compare. Every code that reaches here is a programming error
    /// (`EINVAL` on a destroyed or corrupted mutex, `EAGAIN` on recursion
    /// overflow) or a state this type never creates (`EOWNERDEAD` needs a robust
    /// mutex), so continuing would be operating on a lock whose state is unknown.
    #[cold]
    #[inline(never)]
    fn lock_failed(what: &str, rc: c_int) -> ! {
        panic!("{what} failed: {}", std::io::Error::from_raw_os_error(rc));
    }

    /// Probe whether this platform really implements `PTHREAD_PRIO_INHERIT`.
    ///
    /// Builds and destroys a throwaway attribute and mutex; touches no global
    /// state. On musl this is what triggers libc's one-time `FUTEX_LOCK_PI`
    /// kernel probe, so it answers for the running kernel and not just for the
    /// C library that was linked.
    pub(super) fn available() -> bool {
        // SAFETY: `attr` and `m` are locals owned by this frame; each is
        // initialised before use and destroyed on every path out.
        unsafe {
            let mut attr: libc::pthread_mutexattr_t = std::mem::zeroed();
            if libc::pthread_mutexattr_init(&mut attr) != 0 {
                return false;
            }
            let mut ok = libc::pthread_mutexattr_setprotocol(&mut attr, PTHREAD_PRIO_INHERIT) == 0;
            if ok {
                let mut got: c_int = PTHREAD_PRIO_NONE;
                ok = libc::pthread_mutexattr_getprotocol(&attr, &mut got) == 0
                    && got == PTHREAD_PRIO_INHERIT;
            }
            if ok {
                let mut m: libc::pthread_mutex_t = std::mem::zeroed();
                ok = libc::pthread_mutex_init(&mut m, &attr) == 0;
                if ok {
                    libc::pthread_mutex_destroy(&mut m);
                }
            }
            libc::pthread_mutexattr_destroy(&mut attr);
            ok
        }
    }
}

/// A mutex whose holder is boosted to the priority of its highest-priority waiter.
///
/// Mirrors [`std::sync::Mutex`] apart from the two documented differences —
/// `lock()` does not return a `Result` because this mutex does not poison, and
/// `new` is not `const` — both explained in the [module docs](self).
///
/// The inheritance guarantee is **Linux-only**. On macOS and Windows this is a
/// [`std::sync::Mutex`] with the same API and no boosting; ask
/// [`PiMutex::has_priority_inheritance`] rather than assuming.
///
/// ```
/// use horus_sys::rt::PiMutex;
/// use std::sync::Arc;
///
/// let counter = Arc::new(PiMutex::new(0u64));
/// let worker = {
///     let counter = Arc::clone(&counter);
///     std::thread::spawn(move || {
///         *counter.lock() += 1;
///     })
/// };
/// worker.join().unwrap();
/// assert_eq!(*counter.lock(), 1);
/// ```
#[cfg(target_os = "linux")]
pub struct PiMutex<T: ?Sized> {
    raw: Box<imp::RawLock>,
    /// Last field so `PiMutex<T>` supports unsized `T`, as `std::sync::Mutex` does.
    data: UnsafeCell<T>,
}

// SAFETY: `PiMutex` gives out `&mut T` only through a guard, and the guard is
// handed out only to the thread that owns the lock, so at most one thread can
// reach the `T` at a time. Sending the mutex itself to another thread sends the
// `T` with it, hence `T: Send`; sharing `&PiMutex<T>` lets another thread take
// the lock and obtain `&mut T`, which also only needs `T: Send` (never `&T` on
// two threads at once). These are the bounds `std::sync::Mutex` carries, for
// the same reasons. The raw `pthread_mutex_t` is itself safe to use from any
// thread; `PiMutex` is `!Send`/`!Sync` by default only because it contains an
// `UnsafeCell`.
#[cfg(target_os = "linux")]
unsafe impl<T: ?Sized + Send> Send for PiMutex<T> {}
#[cfg(target_os = "linux")]
unsafe impl<T: ?Sized + Send> Sync for PiMutex<T> {}

#[cfg(target_os = "linux")]
impl<T> PiMutex<T> {
    /// Create a new priority-inheriting mutex holding `value`.
    ///
    /// Allocates. Build these during node setup, not inside a control loop.
    pub fn new(value: T) -> Self {
        Self {
            raw: imp::RawLock::new(),
            data: UnsafeCell::new(value),
        }
    }

    /// Consume the mutex and return the protected value.
    pub fn into_inner(self) -> T {
        // `PiMutex` has no `Drop` of its own (it lives on `RawLock`), so this
        // destructuring move is allowed and still destroys the lock exactly once.
        let Self { raw, data } = self;
        drop(raw);
        data.into_inner()
    }
}

#[cfg(target_os = "linux")]
impl<T: ?Sized> PiMutex<T> {
    /// Acquire the lock, blocking until it is available.
    ///
    /// While this thread is blocked, the current holder inherits this thread's
    /// scheduling priority (Linux only — see [`Self::has_priority_inheritance`]).
    ///
    /// Does not poison and does not return a `Result`; see the [module docs](self).
    /// Locking twice from the same thread deadlocks, as with [`std::sync::Mutex`].
    #[inline]
    pub fn lock(&self) -> PiMutexGuard<'_, T> {
        self.raw.lock();
        PiMutexGuard {
            lock: self,
            _not_send: std::marker::PhantomData,
        }
    }

    /// Acquire the lock if it is free, returning `None` if another thread holds it.
    ///
    /// Never blocks, so it never inherits either — a `try_lock` that misses tells
    /// the kernel nothing about who wanted the lock. Use [`Self::lock`] on paths
    /// where the priority boost is the point.
    #[inline]
    pub fn try_lock(&self) -> Option<PiMutexGuard<'_, T>> {
        if self.raw.try_lock() {
            Some(PiMutexGuard {
                lock: self,
                _not_send: std::marker::PhantomData,
            })
        } else {
            None
        }
    }

    /// Borrow the protected value without locking.
    ///
    /// Sound because `&mut self` proves no other reference — and therefore no
    /// guard — exists.
    #[inline]
    pub fn get_mut(&mut self) -> &mut T {
        self.data.get_mut()
    }

    /// Whether *this* mutex actually carries `PTHREAD_PRIO_INHERIT`.
    ///
    /// `false` means the lock works but a high-priority waiter will not boost
    /// the holder, so blocking on it is unbounded. Always `false` off Linux;
    /// `false` on Linux only when the C library or kernel refused the protocol,
    /// which also logs a warning once per process.
    #[inline]
    pub fn has_priority_inheritance(&self) -> bool {
        self.raw.protocol() == imp::PTHREAD_PRIO_INHERIT
    }
}

/// RAII guard released on drop, exactly like [`std::sync::MutexGuard`].
#[cfg(target_os = "linux")]
#[must_use = "if the guard is dropped immediately the lock is released immediately"]
pub struct PiMutexGuard<'a, T: ?Sized> {
    lock: &'a PiMutex<T>,
    /// `!Send`. `pthread_mutex_unlock` must be called by the thread that locked
    /// the mutex — for a PI mutex the kernel checks the owner TID stored in the
    /// futex word — and unlocking is what `Drop` does, so the guard must not be
    /// able to cross threads. A `*const ()` is the stable way to say that;
    /// negative impls are not.
    _not_send: std::marker::PhantomData<*const ()>,
}

// SAFETY: `PiMutexGuard` hands out `&T` through `Deref` and `&mut T` through
// `DerefMut`. Sharing `&PiMutexGuard<T>` across threads therefore shares `&T`,
// which needs `T: Sync` and nothing more — `&mut T` requires `&mut` on the
// guard, which sharing cannot produce. Same bound as `std::sync::MutexGuard`.
#[cfg(target_os = "linux")]
unsafe impl<T: ?Sized + Sync> Sync for PiMutexGuard<'_, T> {}

#[cfg(target_os = "linux")]
impl<T: ?Sized> Deref for PiMutexGuard<'_, T> {
    type Target = T;

    #[inline]
    fn deref(&self) -> &T {
        // SAFETY: this thread holds the lock for as long as the guard lives, so
        // no other thread can hold a reference to the data.
        unsafe { &*self.lock.data.get() }
    }
}

#[cfg(target_os = "linux")]
impl<T: ?Sized> DerefMut for PiMutexGuard<'_, T> {
    #[inline]
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: as `deref`, and `&mut self` proves this guard is not itself
        // aliased, so the `&mut T` is unique.
        unsafe { &mut *self.lock.data.get() }
    }
}

#[cfg(target_os = "linux")]
impl<T: ?Sized> Drop for PiMutexGuard<'_, T> {
    #[inline]
    fn drop(&mut self) {
        // SAFETY: the guard exists only because this thread locked the mutex,
        // and the guard is `!Send`, so this is the owning thread.
        unsafe { self.lock.raw.unlock() }
    }
}

/// Whether this platform implements priority inheritance for mutexes.
///
/// A live probe on Linux (it builds and destroys a throwaway PI mutex, which is
/// also what makes musl ask the kernel about `FUTEX_LOCK_PI`), a constant
/// `false` on macOS and Windows.
#[cfg(target_os = "linux")]
pub fn priority_inheritance_available() -> bool {
    imp::available()
}

// ============================================================================
// macOS / Windows — std::sync::Mutex, same API, no inheritance
// ============================================================================

/// A mutex whose holder is boosted to the priority of its highest-priority waiter.
///
/// **On this platform there is no boosting.** macOS and Windows expose no
/// working priority-inheritance protocol, so this is a [`std::sync::Mutex`]
/// wearing the same API: code compiles and behaves identically, but blocking on
/// it is unbounded in the presence of mid-priority threads.
/// [`PiMutex::has_priority_inheritance`] returns `false` here, always.
///
/// See the [module docs](self) for the full platform matrix and for why this
/// type does not poison — the std poison flag is deliberately swallowed here so
/// that the *semantics*, and not just the signatures, match the Linux backend.
///
/// ```
/// use horus_sys::rt::PiMutex;
/// use std::sync::Arc;
///
/// let counter = Arc::new(PiMutex::new(0u64));
/// let worker = {
///     let counter = Arc::clone(&counter);
///     std::thread::spawn(move || {
///         *counter.lock() += 1;
///     })
/// };
/// worker.join().unwrap();
/// assert_eq!(*counter.lock(), 1);
/// ```
#[cfg(not(target_os = "linux"))]
pub struct PiMutex<T: ?Sized> {
    inner: std::sync::Mutex<T>,
}

#[cfg(not(target_os = "linux"))]
impl<T> PiMutex<T> {
    /// Create a new mutex holding `value`.
    ///
    /// Not `const`, to match the Linux backend, which allocates.
    pub fn new(value: T) -> Self {
        Self {
            inner: std::sync::Mutex::new(value),
        }
    }

    /// Consume the mutex and return the protected value.
    pub fn into_inner(self) -> T {
        // Poisoning is discarded on purpose — see the module docs. `into_inner`
        // on the poison error yields the value the panicking thread left behind,
        // which is exactly what the pthread backend would hand back.
        self.inner.into_inner().unwrap_or_else(|e| e.into_inner())
    }
}

#[cfg(not(target_os = "linux"))]
impl<T: ?Sized> PiMutex<T> {
    /// Acquire the lock, blocking until it is available.
    ///
    /// No priority inheritance on this platform.
    #[inline]
    pub fn lock(&self) -> PiMutexGuard<'_, T> {
        PiMutexGuard {
            inner: self.inner.lock().unwrap_or_else(|e| e.into_inner()),
        }
    }

    /// Acquire the lock if it is free, returning `None` if another thread holds it.
    #[inline]
    pub fn try_lock(&self) -> Option<PiMutexGuard<'_, T>> {
        match self.inner.try_lock() {
            Ok(inner) => Some(PiMutexGuard { inner }),
            // A poisoned-but-free mutex is available here, because this type
            // does not poison: the Linux backend would have handed it over.
            Err(std::sync::TryLockError::Poisoned(e)) => Some(PiMutexGuard {
                inner: e.into_inner(),
            }),
            Err(std::sync::TryLockError::WouldBlock) => None,
        }
    }

    /// Borrow the protected value without locking.
    #[inline]
    pub fn get_mut(&mut self) -> &mut T {
        self.inner.get_mut().unwrap_or_else(|e| e.into_inner())
    }

    /// Whether *this* mutex actually carries priority inheritance.
    ///
    /// Always `false` on macOS and Windows.
    #[inline]
    pub fn has_priority_inheritance(&self) -> bool {
        false
    }
}

/// RAII guard released on drop, exactly like [`std::sync::MutexGuard`].
#[cfg(not(target_os = "linux"))]
#[must_use = "if the guard is dropped immediately the lock is released immediately"]
pub struct PiMutexGuard<'a, T: ?Sized> {
    /// `std::sync::MutexGuard` is already `!Send`, which this type needs for
    /// the same reason its Linux counterpart does.
    inner: std::sync::MutexGuard<'a, T>,
}

#[cfg(not(target_os = "linux"))]
impl<T: ?Sized> Deref for PiMutexGuard<'_, T> {
    type Target = T;

    #[inline]
    fn deref(&self) -> &T {
        &self.inner
    }
}

#[cfg(not(target_os = "linux"))]
impl<T: ?Sized> DerefMut for PiMutexGuard<'_, T> {
    #[inline]
    fn deref_mut(&mut self) -> &mut T {
        &mut self.inner
    }
}

/// Whether this platform implements priority inheritance for mutexes.
///
/// Always `false` on macOS and Windows.
#[cfg(not(target_os = "linux"))]
pub fn priority_inheritance_available() -> bool {
    false
}

// ============================================================================
// Platform-independent trait impls
// ============================================================================

impl<T: ?Sized + fmt::Debug> fmt::Debug for PiMutex<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // `try_lock`, never `lock`: formatting a mutex the current thread
        // already holds must not deadlock, and neither must formatting one that
        // a 1 kHz thread is holding right now.
        let mut d = f.debug_struct("PiMutex");
        match self.try_lock() {
            Some(guard) => d.field("data", &&*guard),
            None => d.field("data", &format_args!("<locked>")),
        };
        d.field("priority_inheritance", &self.has_priority_inheritance())
            .finish()
    }
}

impl<T: Default> Default for PiMutex<T> {
    fn default() -> Self {
        Self::new(T::default())
    }
}

impl<T> From<T> for PiMutex<T> {
    fn from(value: T) -> Self {
        Self::new(value)
    }
}

impl<T: ?Sized + fmt::Debug> fmt::Debug for PiMutexGuard<'_, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(&**self, f)
    }
}

impl<T: ?Sized + fmt::Display> fmt::Display for PiMutexGuard<'_, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(&**self, f)
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::{Arc, Barrier};
    use std::time::{Duration, Instant};

    // ── Portable behaviour: identical on Linux, macOS and Windows ──────────

    /// Mutual exclusion under real contention.
    ///
    /// Eight threads, ten thousand read-modify-writes each, through `lock()`.
    /// A lock that lets two threads in at once loses increments, and the barrier
    /// makes them all start together so the window is actually contended rather
    /// than eight sequential runs.
    #[test]
    fn pi_mutex_serialises_concurrent_increments() {
        const THREADS: usize = 8;
        const PER_THREAD: u64 = 10_000;

        let counter = Arc::new(PiMutex::new(0u64));
        let barrier = Arc::new(Barrier::new(THREADS));

        let workers: Vec<_> = (0..THREADS)
            .map(|_| {
                let counter = Arc::clone(&counter);
                let barrier = Arc::clone(&barrier);
                std::thread::spawn(move || {
                    barrier.wait();
                    for _ in 0..PER_THREAD {
                        let mut guard = counter.lock();
                        *guard += 1;
                    }
                })
            })
            .collect();

        for w in workers {
            w.join().expect("worker thread must not panic");
        }

        let total = Arc::try_unwrap(counter)
            .expect("every worker has joined, so this is the last Arc")
            .into_inner();
        assert_eq!(
            total,
            THREADS as u64 * PER_THREAD,
            "increments were lost, so two threads held the lock at once"
        );
    }

    /// Contention through `try_lock` must also never lose an update, and must
    /// make progress rather than livelock.
    #[test]
    fn pi_mutex_try_lock_under_contention_never_loses_an_update() {
        const THREADS: usize = 4;
        const PER_THREAD: u64 = 5_000;

        let counter = Arc::new(PiMutex::new(0u64));
        let barrier = Arc::new(Barrier::new(THREADS));

        let workers: Vec<_> = (0..THREADS)
            .map(|_| {
                let counter = Arc::clone(&counter);
                let barrier = Arc::clone(&barrier);
                std::thread::spawn(move || {
                    barrier.wait();
                    let mut done = 0u64;
                    while done < PER_THREAD {
                        if let Some(mut guard) = counter.try_lock() {
                            *guard += 1;
                            done += 1;
                        } else {
                            std::hint::spin_loop();
                        }
                    }
                })
            })
            .collect();

        for w in workers {
            w.join().expect("worker thread must not panic");
        }

        let total = Arc::try_unwrap(counter)
            .expect("every worker has joined, so this is the last Arc")
            .into_inner();
        assert_eq!(total, THREADS as u64 * PER_THREAD);
    }

    /// `try_lock` reports the lock as busy while another thread holds it, and
    /// free again once that thread lets go.
    ///
    /// Deliberately probed from a *second* thread: probing from the holding
    /// thread would be a self-relock, which is undefined for this mutex.
    #[test]
    fn pi_mutex_try_lock_reports_busy_only_while_held() {
        let lock = Arc::new(PiMutex::new(7u32));
        let held = Arc::new(AtomicBool::new(false));
        let release = Arc::new(AtomicBool::new(false));

        let holder = {
            let lock = Arc::clone(&lock);
            let held = Arc::clone(&held);
            let release = Arc::clone(&release);
            std::thread::spawn(move || {
                let guard = lock.lock();
                held.store(true, Ordering::Release);
                let deadline = Instant::now() + Duration::from_secs(10);
                while !release.load(Ordering::Acquire) && Instant::now() < deadline {
                    std::thread::sleep(Duration::from_micros(200));
                }
                drop(guard);
            })
        };

        let deadline = Instant::now() + Duration::from_secs(10);
        while !held.load(Ordering::Acquire) {
            assert!(
                Instant::now() < deadline,
                "the holder thread never acquired the lock"
            );
            std::thread::sleep(Duration::from_micros(200));
        }

        assert!(
            lock.try_lock().is_none(),
            "try_lock handed out a second guard while another thread held the lock"
        );

        release.store(true, Ordering::Release);
        holder.join().expect("holder thread must not panic");

        let guard = lock
            .try_lock()
            .expect("try_lock must succeed once the holder has released");
        assert_eq!(*guard, 7);
    }

    /// The guard is a real reference to the protected value in both directions.
    #[test]
    fn pi_mutex_guard_reads_and_writes_through_deref() {
        let lock = PiMutex::new(vec![1u8, 2, 3]);
        {
            let mut guard = lock.lock();
            assert_eq!(&*guard, &[1, 2, 3]);
            guard.push(4);
        }
        assert_eq!(&*lock.lock(), &[1, 2, 3, 4]);
    }

    /// `get_mut` and `into_inner` reach the value without taking the lock, and
    /// `get_mut`'s write is visible to a later `lock()`.
    #[test]
    fn pi_mutex_get_mut_and_into_inner_bypass_the_lock() {
        let mut lock = PiMutex::new(String::from("a"));
        lock.get_mut().push('b');
        assert_eq!(&*lock.lock(), "ab");
        assert_eq!(lock.into_inner(), "ab");
    }

    /// A panic while holding the lock releases it and does NOT poison.
    ///
    /// This pins down the decision documented on the module: the next `lock()`
    /// succeeds and observes whatever the panicking thread left behind. If this
    /// type ever grows poisoning, this test is the thing that has to be
    /// rewritten deliberately rather than a behaviour that drifts.
    #[test]
    fn pi_mutex_is_not_poisoned_by_a_panicking_holder() {
        let lock = Arc::new(PiMutex::new(0u32));

        let poisoner = {
            let lock = Arc::clone(&lock);
            std::thread::spawn(move || {
                let mut guard = lock.lock();
                *guard = 42;
                panic!("deliberate panic while holding the lock");
            })
        };
        assert!(
            poisoner.join().is_err(),
            "the helper thread was supposed to panic"
        );

        // Lock is free (the guard's Drop ran during unwinding) and usable.
        let guard = lock
            .try_lock()
            .expect("a panicking holder must still release the lock");
        assert_eq!(
            *guard, 42,
            "the next acquirer sees exactly what the panicking thread left"
        );
        drop(guard);

        // And the blocking path is equally unpoisoned.
        assert_eq!(*lock.lock(), 42);
    }

    /// `Debug` must never block, so it stays usable from a diagnostic path
    /// while a control thread holds the lock.
    #[test]
    fn pi_mutex_debug_reports_locked_instead_of_blocking() {
        let lock = PiMutex::new(5u8);
        let guard = lock.lock();
        let rendered = format!("{:?}", lock);
        assert!(
            rendered.contains("<locked>"),
            "Debug on a held mutex should say <locked>, got {rendered}"
        );
        drop(guard);

        let rendered = format!("{:?}", lock);
        assert!(
            rendered.contains('5'),
            "Debug on a free mutex should show the value, got {rendered}"
        );
    }

    /// The mutex is `Send + Sync` for `Send` data, so it can be shared through
    /// an `Arc` the way every call site will want to. A compile-time check.
    #[test]
    fn pi_mutex_is_send_and_sync_for_send_data() {
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<PiMutex<u64>>();
        assert_send_sync::<Arc<PiMutex<Vec<u8>>>>();
        // The guard is deliberately NOT Send (pthread requires the owner to
        // unlock), which cannot be asserted positively here; the `PhantomData`
        // on the struct is what enforces it.
    }

    /// What the platform claims and what an individual mutex carries must agree.
    ///
    /// If these ever disagree, one of them is lying to RT deadline analysis.
    #[test]
    fn pi_mutex_agrees_with_the_platform_probe() {
        let lock = PiMutex::new(());
        assert_eq!(
            lock.has_priority_inheritance(),
            priority_inheritance_available(),
            "a mutex reports {} for priority inheritance while the platform probe reports {}",
            lock.has_priority_inheritance(),
            priority_inheritance_available(),
        );
    }

    /// Off Linux the answer must be an honest `false`, not a hopeful `true`.
    #[cfg(not(target_os = "linux"))]
    #[test]
    fn pi_mutex_does_not_claim_inheritance_off_linux() {
        assert!(
            !priority_inheritance_available(),
            "macOS/Windows have no working PTHREAD_PRIO_INHERIT; claiming otherwise would \
             let RT analysis assume bounded blocking that does not exist"
        );
        assert!(!PiMutex::new(0u8).has_priority_inheritance());
    }

    // ── Linux: the protocol is really PTHREAD_PRIO_INHERIT ─────────────────

    /// The attribute round-trips through the live libc.
    ///
    /// `PTHREAD_PRIO_INHERIT` is not exported by the `libc` crate for
    /// linux-gnu/linux-musl, so its value is a constant in this module. This
    /// asks the C library itself: set the protocol, read it back with
    /// `pthread_mutexattr_getprotocol`, and require the same value out. A libc
    /// where the constant were wrong would either reject it (`EINVAL`) or hand
    /// back something else.
    #[cfg(target_os = "linux")]
    #[test]
    fn pi_mutex_setprotocol_round_trips_prio_inherit() {
        // SAFETY: `attr` is a local POD struct, initialised before use and
        // destroyed on the way out.
        unsafe {
            let mut attr: libc::pthread_mutexattr_t = std::mem::zeroed();
            assert_eq!(
                libc::pthread_mutexattr_init(&mut attr),
                0,
                "pthread_mutexattr_init failed"
            );

            let mut default_protocol: libc::c_int = -1;
            assert_eq!(
                libc::pthread_mutexattr_getprotocol(&attr, &mut default_protocol),
                0,
                "pthread_mutexattr_getprotocol failed on a fresh attribute"
            );
            assert_eq!(
                default_protocol,
                imp::PTHREAD_PRIO_NONE,
                "a default mutex attribute must be PTHREAD_PRIO_NONE — which is exactly \
                 why every std::sync::Mutex on this system inverts priority"
            );

            let rc = libc::pthread_mutexattr_setprotocol(&mut attr, imp::PTHREAD_PRIO_INHERIT);
            assert_eq!(
                rc,
                0,
                "pthread_mutexattr_setprotocol(PTHREAD_PRIO_INHERIT) failed: {}",
                std::io::Error::from_raw_os_error(rc)
            );

            let mut got: libc::c_int = -1;
            assert_eq!(
                libc::pthread_mutexattr_getprotocol(&attr, &mut got),
                0,
                "pthread_mutexattr_getprotocol failed"
            );
            assert_eq!(
                got,
                imp::PTHREAD_PRIO_INHERIT,
                "libc did not read back the protocol it was just given"
            );

            libc::pthread_mutexattr_destroy(&mut attr);
        }
    }

    /// A `PiMutex` built by `PiMutex::new` really carries the inherit protocol.
    ///
    /// `has_priority_inheritance` is not a hardcoded `true`: it reports the
    /// value `RawLock::new` read back with `pthread_mutexattr_getprotocol` off
    /// the attribute object it then handed to `pthread_mutex_init`.
    #[cfg(target_os = "linux")]
    #[test]
    fn pi_mutex_is_constructed_with_prio_inherit_on_linux() {
        assert!(
            priority_inheritance_available(),
            "this Linux system refused PTHREAD_PRIO_INHERIT (old musl, or a kernel/seccomp \
             without FUTEX_LOCK_PI). PiMutex still locks correctly here, but it provides no \
             priority inheritance, so this is a real result and not a test to skip."
        );
        let lock = PiMutex::new(0u32);
        assert!(
            lock.has_priority_inheritance(),
            "PiMutex::new did not apply PTHREAD_PRIO_INHERIT even though the platform \
             supports it"
        );
    }

    /// The *mutex object*, not just the attribute, is a PI mutex.
    ///
    /// The attribute round-trip above proves what we asked for; this proves what
    /// we got. A glibc PI mutex must store the owner's TID in its futex word,
    /// because that is what `FUTEX_LOCK_PI` requires the kernel to find there —
    /// it is the defining observable of the protocol. A plain mutex stores a
    /// small state counter instead, which the control mutex below pins down so
    /// the probe cannot pass by accident.
    ///
    /// `__lock` is the first member of glibc's `struct __pthread_mutex_s` on
    /// every architecture, so reading offset 0 is arch-independent — but it is
    /// glibc's layout, hence the `target_env` gate. musl keeps its type word
    /// first instead, and this test says nothing about it.
    #[cfg(all(target_os = "linux", target_env = "gnu"))]
    #[test]
    fn pi_mutex_stores_the_owner_tid_in_its_futex_word() {
        /// glibc masks the futex word with FUTEX_TID_MASK to recover the owner;
        /// the top two bits are FUTEX_WAITERS and FUTEX_OWNER_DIED.
        const FUTEX_TID_MASK: u32 = 0x3fff_ffff;

        // SAFETY: SYS_gettid takes no arguments and only reads kernel state.
        let tid = unsafe { libc::syscall(libc::SYS_gettid) } as u32;
        assert_ne!(tid, 0, "gettid returned 0");

        /// Read the first 4 bytes of a `pthread_mutex_t` — glibc's `__lock`.
        ///
        /// # Safety
        ///
        /// `m` must point at an initialised `pthread_mutex_t`.
        unsafe fn futex_word(m: *const libc::pthread_mutex_t) -> u32 {
            unsafe { std::ptr::read_volatile(m.cast::<u32>()) }
        }

        let pi = PiMutex::new(());
        assert!(
            pi.has_priority_inheritance(),
            "this test is only meaningful on a mutex that claims inheritance"
        );

        let guard = pi.lock();
        // SAFETY: `pi.raw` holds a live, initialised pthread_mutex_t, and this
        // thread holds it, so the word is stable while we read it.
        let held_word = unsafe { futex_word(pi.raw.as_ptr()) };
        drop(guard);

        assert_eq!(
            held_word & FUTEX_TID_MASK,
            tid,
            "a locked PI mutex must carry the owner TID ({tid}) in its futex word, got {held_word:#x} \
             — the mutex was initialised without PTHREAD_PRIO_INHERIT"
        );

        // Control: the same probe on a deliberately plain mutex must NOT see a
        // TID, otherwise the assertion above proves nothing.
        // SAFETY: `plain` is a local, initialised with a null attribute (the
        // libc default, PTHREAD_PRIO_NONE), locked and unlocked by this thread
        // only, and destroyed before it goes out of scope.
        unsafe {
            let mut plain: libc::pthread_mutex_t = std::mem::zeroed();
            assert_eq!(libc::pthread_mutex_init(&mut plain, std::ptr::null()), 0);
            assert_eq!(libc::pthread_mutex_lock(&mut plain), 0);
            let plain_word = futex_word(&plain);
            assert_eq!(libc::pthread_mutex_unlock(&mut plain), 0);
            libc::pthread_mutex_destroy(&mut plain);

            assert_ne!(
                plain_word & FUTEX_TID_MASK,
                tid,
                "a plain (PTHREAD_PRIO_NONE) mutex stored the TID too, so this probe cannot \
                 tell PI from non-PI and the assertion above is worthless"
            );
        }
    }

    // ── Linux: inheritance actually happens (needs CAP_SYS_NICE) ───────────

    /// The whole point, end to end: a low-priority holder is boosted to the
    /// priority of the high-priority thread blocked behind it.
    ///
    /// `#[ignore]` because it needs `SCHED_FIFO`, i.e. `CAP_SYS_NICE` or a
    /// non-zero `RLIMIT_RTPRIO`, which neither CI runners nor a normal
    /// development machine give a test binary. It is not skipped-when-
    /// unprivileged on purpose: a test that quietly passes without checking
    /// anything is worse than one that is explicitly not run. To run it:
    ///
    /// ```text
    /// cargo test -p horus_sys --lib pi_mutex -- --ignored --exact \
    ///     rt::pi_mutex::tests::pi_mutex_boosts_a_low_priority_holder
    /// # then run the built binary under sudo, or:
    /// sudo -E $(command -v cargo) test -p horus_sys --lib -- --ignored
    /// ```
    ///
    /// How the boost is observed: field 18 of `/proc/<pid>/task/<tid>/stat` is
    /// `task_prio(p) = p->prio - MAX_RT_PRIO`, the *effective* priority, which
    /// is what PI raises. Field 40 is `rt_priority`, the thread's own base
    /// priority, which PI must leave alone. For an RT thread of base priority
    /// `R`, field 18 reads `-1 - R`. `sched_getparam` is no use here — it
    /// returns the base priority and so cannot see a boost at all.
    #[cfg(target_os = "linux")]
    #[test]
    #[ignore = "requires CAP_SYS_NICE / RLIMIT_RTPRIO to use SCHED_FIFO"]
    fn pi_mutex_boosts_a_low_priority_holder() {
        use std::sync::atomic::AtomicI32;

        const LOW: libc::c_int = 10;
        const HIGH: libc::c_int = 30;

        /// Put the *calling* thread on SCHED_FIFO at `prio`.
        fn become_fifo(prio: libc::c_int) {
            // SAFETY: pthread_self() is this thread; `param` is a POD struct
            // for which all-zero is a valid bit pattern.
            let rc = unsafe {
                let mut param: libc::sched_param = std::mem::zeroed();
                param.sched_priority = prio;
                libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_FIFO, &param)
            };
            assert_eq!(
                rc,
                0,
                "pthread_setschedparam(SCHED_FIFO, {prio}) failed: {} — this test needs \
                 CAP_SYS_NICE",
                std::io::Error::from_raw_os_error(rc)
            );
        }

        /// Field 18 (`priority`, the PI-boosted effective priority) and field 40
        /// (`rt_priority`, the base) of a thread's `/proc` stat line.
        fn proc_priorities(tid: i32) -> (i32, i32) {
            let path = format!("/proc/self/task/{tid}/stat");
            let stat = std::fs::read_to_string(&path)
                .unwrap_or_else(|e| panic!("reading {path} failed: {e}"));
            // The comm field is parenthesised and may itself contain spaces and
            // parentheses, so split after the LAST ')'. What follows is field 3
            // onwards, i.e. token[i] is field i + 3.
            let rest = &stat[stat.rfind(')').expect("no ')' in stat line") + 1..];
            let fields: Vec<&str> = rest.split_whitespace().collect();
            let get = |field: usize| -> i32 {
                fields
                    .get(field - 3)
                    .unwrap_or_else(|| panic!("stat line has no field {field}: {stat}"))
                    .parse()
                    .unwrap_or_else(|e| panic!("field {field} is not an integer: {e}"))
            };
            (get(18), get(40))
        }

        assert!(
            priority_inheritance_available(),
            "no PTHREAD_PRIO_INHERIT on this system"
        );

        let lock = Arc::new(PiMutex::new(0u64));
        let holder_tid = Arc::new(AtomicI32::new(0));
        let holding = Arc::new(AtomicBool::new(false));
        let release = Arc::new(AtomicBool::new(false));
        let waiter_started = Arc::new(AtomicBool::new(false));

        // Low-priority holder: takes the lock and sits on it.
        let low = {
            let lock = Arc::clone(&lock);
            let holder_tid = Arc::clone(&holder_tid);
            let holding = Arc::clone(&holding);
            let release = Arc::clone(&release);
            std::thread::spawn(move || {
                become_fifo(LOW);
                // SAFETY: SYS_gettid takes no arguments and only reads.
                holder_tid.store(
                    unsafe { libc::syscall(libc::SYS_gettid) } as i32,
                    Ordering::Release,
                );
                let mut guard = lock.lock();
                *guard += 1;
                holding.store(true, Ordering::Release);
                // Sleep rather than spin: a spinning SCHED_FIFO thread is a
                // good way to make a machine unpleasant, and the boost is a
                // property of the task, not of whether it is on a CPU.
                let deadline = Instant::now() + Duration::from_secs(10);
                while !release.load(Ordering::Acquire) && Instant::now() < deadline {
                    std::thread::sleep(Duration::from_micros(500));
                }
                drop(guard);
            })
        };

        let deadline = Instant::now() + Duration::from_secs(5);
        while !holding.load(Ordering::Acquire) {
            assert!(
                Instant::now() < deadline,
                "the low-priority thread never took the lock"
            );
            std::thread::sleep(Duration::from_millis(1));
        }
        let tid = holder_tid.load(Ordering::Acquire);

        let (base_effective, base_rt) = proc_priorities(tid);
        assert_eq!(
            (base_effective, base_rt),
            (-1 - LOW, LOW),
            "the holder is not sitting at SCHED_FIFO {LOW} before the high-priority waiter \
             arrives, so nothing after this measures inheritance"
        );

        // High-priority waiter: blocks on the lock the low thread holds.
        let high = {
            let lock = Arc::clone(&lock);
            let waiter_started = Arc::clone(&waiter_started);
            std::thread::spawn(move || {
                become_fifo(HIGH);
                waiter_started.store(true, Ordering::Release);
                let mut guard = lock.lock();
                *guard += 1;
            })
        };

        // Poll for the boost. The window between `waiter_started` and the
        // waiter actually being queued on the futex is short but not zero.
        let mut observed = (base_effective, base_rt);
        let deadline = Instant::now() + Duration::from_secs(5);
        while Instant::now() < deadline {
            if waiter_started.load(Ordering::Acquire) {
                observed = proc_priorities(tid);
                if observed.0 == -1 - HIGH {
                    break;
                }
            }
            std::thread::sleep(Duration::from_millis(1));
        }

        // Let everybody go BEFORE asserting, so a failure does not leave the
        // high-priority thread blocked forever.
        release.store(true, Ordering::Release);
        low.join().expect("low-priority thread must not panic");
        high.join().expect("high-priority thread must not panic");

        assert_eq!(
            observed.0,
            -1 - HIGH,
            "the holder's effective priority stayed at {} while a SCHED_FIFO {HIGH} thread was \
             blocked on its lock — expected it to be boosted to {}. That is unbounded priority \
             inversion: the mutex is not inheriting.",
            observed.0,
            -1 - HIGH
        );
        assert_eq!(
            observed.1, LOW,
            "inheritance must boost the effective priority only; the holder's base rt_priority \
             changed from {LOW} to {}",
            observed.1
        );
        assert_eq!(
            *lock.lock(),
            2,
            "both threads should have taken the lock once"
        );
    }
}
