//! `hlog!` from inside a real-time `tick()` must not block on stderr.
//!
//! `hlog!` is the only logging API node code has, and its direct path made
//! about nine heap allocations, took two process-global mutexes (the shared log
//! buffer's, twice over for Error and Warning) plus stderr's own, called
//! `chrono::Local::now()` — which revalidates its zone cache with an
//! `env::var("TZ")` and an `lstat` of /etc/localtime — asked `isatty` and
//! `tcgetattr` whether the terminal was raw, and then issued an unbuffered
//! blocking `write(2)` + `flush`.
//!
//! `write_console_line`'s own doc comment already conceded the consequence: "on
//! a slow or stopped stderr reader it blocks for as long as the reader takes,
//! and a `hlog!` from inside a `tick()` inherits that unbounded wait."
//!
//! # What these assert, and why not timing
//!
//! The property is "the RT thread did not do the work", and the exact,
//! machine-independent evidence for that is that the entry is sitting in the
//! ring rather than having been written. A wall-clock assertion would measure
//! the host scheduler, whose wake jitter here is ~253 µs mean — three orders of
//! magnitude above the effect.

use horus_core::core::hlog_rt;

/// Serialises these tests against each other.
///
/// There is one ring per process by design, and this binary runs its tests
/// concurrently, so without this one test's flood drains another's entries.
static SERIALISE: std::sync::Mutex<()> = std::sync::Mutex::new(());

fn serialised() -> std::sync::MutexGuard<'static, ()> {
    SERIALISE.lock().unwrap_or_else(|e| e.into_inner())
}

/// The marker that decides which path a line takes must be per-thread and must
/// nest, or a nested guard's drop would silently put the RT thread back on the
/// blocking path mid-tick.
#[test]
fn the_realtime_marker_is_scoped_to_the_thread_that_set_it() {
    let _serial = serialised();
    assert!(!hlog_rt::in_rt_thread());
    let outer = hlog_rt::enter_rt_thread();
    assert!(hlog_rt::in_rt_thread());

    let elsewhere = std::thread::spawn(hlog_rt::in_rt_thread)
        .join()
        .expect("probe thread");
    assert!(
        !elsewhere,
        "the marker leaked to another thread: every non-RT thread would then \
         start queueing its log lines instead of writing them"
    );

    drop(outer);
    assert!(!hlog_rt::in_rt_thread());
}

/// The load-bearing assertion: on an RT thread the entry goes to the ring, and
/// nothing is written until a non-RT drain runs.
#[test]
fn a_log_line_from_an_rt_thread_is_queued_rather_than_written() {
    let _serial = serialised();
    std::thread::spawn(|| {
        // Drain anything an earlier test left, so the count below is ours.
        hlog_rt::flush();

        let _rt = hlog_rt::enter_rt_thread();
        horus_core::hlog!(error, "rt marker {}", 4242);
        horus_core::hlog!(warn, "second line");

        // Still on the RT thread: the entries must NOT have been emitted yet.
        // `flush` is the only thing that emits them, and it is called from a
        // non-RT drain thread or from `RtExecutor::stop`.
        let emitted = {
            // Leaving the RT context first, because `flush` itself logs when it
            // has drops to report and must not queue that.
            drop(_rt);
            hlog_rt::flush()
        };
        assert!(
            emitted >= 2,
            "the two RT log lines must have been sitting in the ring for the drain \
             to emit; got {emitted}"
        );
    })
    .join()
    .expect("the probe thread must not panic");
}

/// A line logged from an ordinary thread must still be written immediately.
///
/// The fix must not turn every log line in the process into a queued one: a
/// CLI, a test, and every non-RT thread depend on `hlog!` being synchronous.
#[test]
fn a_log_line_from_an_ordinary_thread_still_takes_the_direct_path() {
    let _serial = serialised();
    hlog_rt::flush();
    assert!(!hlog_rt::in_rt_thread());
    horus_core::hlog!(error, "direct path marker");
    assert_eq!(
        hlog_rt::flush(),
        0,
        "a line logged off an RT thread must already have been written, so there \
         is nothing for the drain to emit"
    );
}

/// The ring drops rather than blocks, and says how many it dropped.
///
/// Blocking the producer is the entire defect being removed, so the overflow
/// behaviour has to be loss — but silent loss would be a different defect.
#[test]
fn the_ring_drops_under_a_storm_and_counts_what_it_dropped() {
    let _serial = serialised();
    std::thread::spawn(|| {
        hlog_rt::flush();
        let dropped_before = hlog_rt::dropped_count();
        {
            let _rt = hlog_rt::enter_rt_thread();
            // Far more than the ring holds, with no drain in between.
            for i in 0..4096 {
                horus_core::hlog!(error, "storm line {}", i);
            }
        }
        let emitted = hlog_rt::flush();
        assert!(
            emitted <= 512,
            "the ring must be bounded, not grow to hold 4096 entries; emitted {emitted}"
        );
        assert!(
            hlog_rt::dropped_count() > dropped_before || emitted == 4096,
            "entries were lost without being counted — silent thinning under load is \
             a different defect from bounded loss"
        );
    })
    .join()
    .expect("the probe thread must not panic");
}
