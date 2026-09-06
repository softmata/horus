//! `hlog!` from a real-time tick, without the allocator, the mutex or the
//! blocking write.
//!
//! # What this exists to remove
//!
//! `hlog!` is the only logging API node code has, and every arm ended in
//! `log_with_context(level, format!(..))`. Counting one `hlog!(error, "…")`
//! from inside `tick()`:
//!
//! * ~9 heap allocations — the `format!`, the node-name clone, the console
//!   line, the timestamp string, the entry clone into the error buffer, and two
//!   `bincode::serialize` `Vec<u8>`s;
//! * two process-global mutexes, because `publish_log` pushes to the error
//!   buffer *and* the main buffer for Error and Warning, each holding
//!   `SharedLogBuffer`'s `Mutex<MmapMut>` across a bincode serialise and a slot
//!   copy — plus stderr's own reentrant lock;
//! * `chrono::Local::now()`, which revalidates its zone cache once a second
//!   with `env::var("TZ")` (process-global environ lock) and an `lstat` of
//!   `/etc/localtime`;
//! * `is_raw_mode()`, an `isatty` plus a `tcgetattr`, on every single line;
//! * and finally an unbuffered blocking `write(2)` + `flush` to stderr, whose
//!   own doc comment already conceded the defect: "on a slow or stopped stderr
//!   reader it blocks for as long as the reader takes, and a `hlog!` from
//!   inside a `tick()` inherits that unbounded wait".
//!
//! # Why the message has to arrive as `Arguments`
//!
//! The macro used to commit to a `String` before anything could intervene. By
//! the time any RT-aware code could run, the allocation had already happened —
//! and on a `.no_alloc()` node that allocation is itself the violation. So the
//! message travels as `core::fmt::Arguments` all the way to a fixed-size slot,
//! exactly as `rt_diag(format_args!(..))` already does. Formatting writes
//! straight into the slot's byte array through `core::fmt`; there is no
//! `format!` anywhere on this path.
//!
//! # Why this is not `rt_diag`
//!
//! `rt_diag` carries a rendered line for a human. A log entry has structure the
//! drain has to rebuild — level, node name, tick number, tick duration — and
//! the shared-memory log buffer, `horus log`, and the console line all want the
//! fields rather than the string. So this is the same seqlock protocol with a
//! structured slot.
//!
//! # What it costs, stated rather than buried
//!
//! * **Latency of the line.** A log entry reaches stderr and the shared buffer
//!   up to `POLL` late. `RtExecutor::stop` flushes, so a run shorter than one
//!   poll does not lose its output.
//! * **Loss under a storm.** The ring overwrites its oldest entry rather than
//!   blocking the producer; blocking the producer is the entire defect being
//!   removed. Drops are counted exactly and reported, so output is compressed
//!   under load, never silently thinned.
//! * **Ordering.** A queued line and a line written directly from a non-RT
//!   thread are not ordered against each other. Both carry their own timestamp.

use std::fmt;
use std::sync::atomic::{AtomicU32, AtomicU64, AtomicU8, Ordering};
use std::sync::{Mutex, Once};
use std::time::{Duration, Instant};

use crate::core::log_buffer::LogType;

/// Bytes reserved for one queued message. The shared log buffer truncates to
/// its own limit anyway.
const MSG_CAP: usize = 224;

/// Bytes reserved for the node name. Matches the log buffer's own field cap.
const NAME_CAP: usize = 64;

/// Slots in the ring. Must be a power of two. ~80 KB of BSS.
const SLOTS: usize = 256;

const SLOT_MASK: u64 = SLOTS as u64 - 1;

/// How often the drain empties the ring. Same cadence as the RT diagnostic
/// drain, for the same reason: the only thing it delays is a line for a human.
const POLL: Duration = Duration::from_millis(25);

/// One queued log entry.
struct LogSlot {
    /// Seqlock. `0` = never written, odd = write in progress, even and nonzero
    /// = complete. Derived from the global claim index, so a slot's sequence is
    /// strictly increasing across reuse and a reader can tell "this is the
    /// entry I asked for" from "this slot has already been recycled".
    seq: AtomicU64,
    level: AtomicU8,
    /// Monotonic nanoseconds since process start, for the drain to reconstruct
    /// a wall-clock timestamp without calling `chrono` on the RT thread.
    mono_ns: AtomicU64,
    tick_number: AtomicU64,
    tick_us: AtomicU64,
    name_len: AtomicU32,
    msg_len: AtomicU32,
    /// `AtomicU8` rather than `UnsafeCell<[u8; N]>` deliberately, for the same
    /// reason `rt_diag`'s payload is: a producer lapped mid-write by `SLOTS`
    /// other producers would otherwise be a data race, which is undefined
    /// behaviour whether or not anyone reads the result. With atomic bytes the
    /// worst case is a garbled entry that the seqlock discards.
    name: [AtomicU8; NAME_CAP],
    msg: [AtomicU8; MSG_CAP],
}

impl LogSlot {
    const fn new() -> Self {
        Self {
            seq: AtomicU64::new(0),
            level: AtomicU8::new(0),
            mono_ns: AtomicU64::new(0),
            tick_number: AtomicU64::new(0),
            tick_us: AtomicU64::new(0),
            name_len: AtomicU32::new(0),
            msg_len: AtomicU32::new(0),
            name: [const { AtomicU8::new(0) }; NAME_CAP],
            msg: [const { AtomicU8::new(0) }; MSG_CAP],
        }
    }
}

static RING: [LogSlot; SLOTS] = [const { LogSlot::new() }; SLOTS];
/// Next claim index. Monotonic; slot is `index & SLOT_MASK`.
static HEAD: AtomicU64 = AtomicU64::new(0);
/// Entries the ring overwrote that the drain has not yet reported.
///
/// Reset by each drain, because the console line it produces says "since the
/// last report". [`DROPPED_TOTAL`] is the number a monitor samples.
static DROPPED_PENDING: AtomicU64 = AtomicU64::new(0);
/// Entries the ring overwrote, since process start. Monotonic, never reset.
static DROPPED_TOTAL: AtomicU64 = AtomicU64::new(0);
/// Entries whose message exceeded `MSG_CAP`.
static TRUNCATED: AtomicU64 = AtomicU64::new(0);
/// Index of the next entry to drain, and the lock serialising the two drainers
/// (the drain thread and the executor's final flush).
///
/// **Producers never touch this.** That is the point: an RT thread emitting a
/// log line can never wait on a drainer.
static TAIL: Mutex<u64> = Mutex::new(0);
static DRAIN_STARTED: Once = Once::new();

thread_local! {
    /// Nesting depth of `enter_rt_thread` guards on this thread.
    ///
    /// A depth rather than a bool so a nested guard cannot clear the outer
    /// one's marker on drop.
    static RT_DEPTH: std::cell::Cell<u32> = const { std::cell::Cell::new(0) };
}

/// Process-start reference for the monotonic timestamps in the ring.
fn mono_base() -> Instant {
    static BASE: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    *BASE.get_or_init(Instant::now)
}

/// Whether the calling thread is inside an RT tick loop.
///
/// Distinct from `rt_allocator::is_rt_context()`, which is entered only for
/// `.no_alloc()` nodes and only from their second tick — an ordinary RT node is
/// invisible to it, and so are the panic hook and the emergency-stop latch,
/// which are exactly the paths that most need this.
pub fn in_rt_thread() -> bool {
    RT_DEPTH.try_with(|d| d.get() > 0).unwrap_or(false)
}

/// Marks the calling thread as an RT tick thread until dropped.
pub struct RtThreadGuard(());

impl Drop for RtThreadGuard {
    fn drop(&mut self) {
        let _ = RT_DEPTH.try_with(|d| d.set(d.get().saturating_sub(1)));
    }
}

/// Mark the calling thread as an RT tick thread for the guard's lifetime.
pub fn enter_rt_thread() -> RtThreadGuard {
    let _ = RT_DEPTH.try_with(|d| d.set(d.get() + 1));
    RtThreadGuard(())
}

/// Formats `core::fmt` output straight into a slot's byte array.
struct SlotWriter<'a> {
    bytes: &'a [AtomicU8],
    len: usize,
    truncated: bool,
}

impl fmt::Write for SlotWriter<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let room = self.bytes.len() - self.len;
        if room == 0 {
            self.truncated = true;
            return Err(fmt::Error);
        }
        let n = if s.len() <= room {
            s.len()
        } else {
            self.truncated = true;
            // Never split a multi-byte character: the drain reads the slot back
            // as UTF-8 and would discard the whole entry.
            let mut n = room;
            while n > 0 && (s.as_bytes()[n] & 0xC0) == 0x80 {
                n -= 1;
            }
            n
        };
        for (i, b) in s.as_bytes()[..n].iter().enumerate() {
            self.bytes[self.len + i].store(*b, Ordering::Relaxed);
        }
        self.len += n;
        // Returning `Err` once the slot is full stops `core::fmt` evaluating
        // the remaining arguments — the truncation is already recorded, so
        // continuing would be pure cost on the RT thread.
        if self.truncated {
            Err(fmt::Error)
        } else {
            Ok(())
        }
    }
}

/// Queue one log entry. Never blocks, never allocates, never enters the kernel.
///
/// Returns the claim index it wrote, which is what lets a caller read back
/// exactly the entry it queued rather than guessing from `HEAD`.
pub(crate) fn queue(
    level: LogType,
    node_name: &str,
    tick_number: u64,
    tick_us: u64,
    args: fmt::Arguments<'_>,
) -> u64 {
    let idx = HEAD.fetch_add(1, Ordering::Relaxed);
    let slot = &RING[(idx & SLOT_MASK) as usize];

    // Odd sequence: a reader that lands here mid-write discards the slot. The
    // store is `Relaxed` and the ordering comes from the fence after it — a
    // `Release` store would order everything *before* it, which is the wrong
    // direction; what this needs is the odd marker visible before the byte
    // stores that follow. (Free on x86: a compiler barrier, no instruction.)
    slot.seq
        .store(idx.wrapping_mul(2).wrapping_add(1), Ordering::Relaxed);
    std::sync::atomic::fence(Ordering::Release);

    slot.level.store(level_to_u8(level), Ordering::Relaxed);
    slot.mono_ns
        .store(mono_base().elapsed().as_nanos() as u64, Ordering::Relaxed);
    slot.tick_number.store(tick_number, Ordering::Relaxed);
    slot.tick_us.store(tick_us, Ordering::Relaxed);

    let mut name_w = SlotWriter {
        bytes: &slot.name,
        len: 0,
        truncated: false,
    };
    let _ = fmt::Write::write_str(&mut name_w, node_name);
    slot.name_len.store(name_w.len as u32, Ordering::Relaxed);

    let mut msg_w = SlotWriter {
        bytes: &slot.msg,
        len: 0,
        truncated: false,
    };
    let _ = fmt::Write::write_fmt(&mut msg_w, args);
    if msg_w.truncated {
        TRUNCATED.fetch_add(1, Ordering::Relaxed);
    }
    slot.msg_len.store(msg_w.len as u32, Ordering::Relaxed);

    // Release publishes every store above to any reader that acquires this.
    slot.seq
        .store(idx.wrapping_mul(2).wrapping_add(2), Ordering::Release);
    idx
}

/// Stable byte for a level, so it survives the ring.
fn level_to_u8(level: LogType) -> u8 {
    match level {
        LogType::Error => 0,
        LogType::Warning => 1,
        LogType::Info => 2,
        LogType::Debug => 3,
        LogType::Publish => 4,
        LogType::Subscribe => 5,
    }
}

fn level_from_u8(v: u8) -> LogType {
    match v {
        0 => LogType::Error,
        1 => LogType::Warning,
        3 => LogType::Debug,
        4 => LogType::Publish,
        5 => LogType::Subscribe,
        // Anything unrecognised is Info: a garbled level must not silently
        // become an Error and page someone.
        _ => LogType::Info,
    }
}

/// Read one complete entry out of a slot, or `None` if it was recycled or is
/// mid-write.
fn read_slot(idx: u64) -> Option<(LogType, String, u64, u64, u64, String)> {
    let slot = &RING[(idx & SLOT_MASK) as usize];
    let want = idx.wrapping_mul(2).wrapping_add(2);
    if slot.seq.load(Ordering::Acquire) != want {
        return None;
    }

    let level = level_from_u8(slot.level.load(Ordering::Relaxed));
    let mono_ns = slot.mono_ns.load(Ordering::Relaxed);
    let tick_number = slot.tick_number.load(Ordering::Relaxed);
    let tick_us = slot.tick_us.load(Ordering::Relaxed);
    let name_len = (slot.name_len.load(Ordering::Relaxed) as usize).min(NAME_CAP);
    let msg_len = (slot.msg_len.load(Ordering::Relaxed) as usize).min(MSG_CAP);

    let name: Vec<u8> = slot.name[..name_len]
        .iter()
        .map(|b| b.load(Ordering::Relaxed))
        .collect();
    let msg: Vec<u8> = slot.msg[..msg_len]
        .iter()
        .map(|b| b.load(Ordering::Relaxed))
        .collect();

    // Re-check the sequence: a producer that lapped us mid-copy invalidates
    // everything above, and a garbled line is worse than a dropped one.
    if slot.seq.load(Ordering::Acquire) != want {
        return None;
    }

    Some((
        level,
        String::from_utf8(name).ok()?,
        tick_number,
        tick_us,
        mono_ns,
        String::from_utf8(msg).ok()?,
    ))
}

/// Drain every complete entry into the real logging path. Returns how many were
/// emitted. Never called from an RT thread.
pub(crate) fn drain() -> usize {
    let mut tail = TAIL.lock().unwrap_or_else(|e| e.into_inner());
    let head = HEAD.load(Ordering::Acquire);

    // The ring holds only the newest `SLOTS` entries; anything older was
    // overwritten. Count what was lost rather than replaying garbage.
    if head.saturating_sub(*tail) > SLOTS as u64 {
        let lost = head - *tail - SLOTS as u64;
        DROPPED_PENDING.fetch_add(lost, Ordering::Relaxed);
        DROPPED_TOTAL.fetch_add(lost, Ordering::Relaxed);
        *tail = head - SLOTS as u64;
    }

    let mut emitted = 0;
    while *tail < head {
        let idx = *tail;
        *tail += 1;
        if let Some((level, name, tick_number, tick_us, _mono, msg)) = read_slot(idx) {
            crate::core::hlog::emit_recorded(&level, &name, &msg, tick_number, tick_us);
            emitted += 1;
        }
    }

    let dropped = DROPPED_PENDING.swap(0, Ordering::Relaxed);
    if dropped > 0 {
        crate::core::hlog::emit_console(
            &LogType::Warning,
            "horus-hlog-drain",
            &format!("{dropped} RT log lines were dropped: the ring filled faster than it drained"),
        );
    }
    emitted
}

/// Start the drain thread. Idempotent.
///
/// The thread runs for the life of the process and is one wakeup every `POLL`.
/// It must outlive any individual executor: an executor being torn down is
/// precisely when its last log lines matter.
pub fn start_drain() {
    DRAIN_STARTED.call_once(|| {
        let spawned = crate::scheduling::rt::spawn_best_effort("horus-hlog-drain", 5, || loop {
            drain();
            crate::core::hlog::refresh_raw_mode();
            std::thread::sleep(POLL);
        });
        if spawned.is_err() {
            crate::terminal::print_line(
                "[RT] WARNING: could not spawn the RT log drain thread; RT log lines will \
                 only be flushed at shutdown",
            );
        }
    });
}

/// Drain synchronously. Called from `RtExecutor::stop`, so a run shorter than
/// one poll does not exit with its lines still in the ring.
pub fn flush() -> usize {
    drain()
}

/// Entries dropped because the ring filled faster than it drained, since
/// process start.
///
/// Monotonic and never reset, so a monitor samples it as a delta — the same
/// shape as `rt_allocator::violation_count()`.
pub fn dropped_count() -> u64 {
    DROPPED_TOTAL.load(Ordering::Relaxed)
}

/// Entries whose message was truncated to fit a slot.
pub fn truncated_count() -> u64 {
    TRUNCATED.load(Ordering::Relaxed)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Serialises the tests that read a specific slot back.
    ///
    /// `HEAD` and the ring are process-global by design — there is one ring per
    /// process — and this binary runs its tests concurrently, so without this
    /// the flood test laps the slot another test is about to read.
    static RING: std::sync::Mutex<()> = std::sync::Mutex::new(());

    fn serialised() -> std::sync::MutexGuard<'static, ()> {
        RING.lock().unwrap_or_else(|e| e.into_inner())
    }

    #[test]
    fn the_rt_marker_is_per_thread_and_nests() {
        assert!(!in_rt_thread());
        {
            let _outer = enter_rt_thread();
            assert!(in_rt_thread());
            {
                let _inner = enter_rt_thread();
                assert!(in_rt_thread());
            }
            assert!(
                in_rt_thread(),
                "a nested guard's drop must not clear the outer one's marker"
            );
            let elsewhere = std::thread::spawn(in_rt_thread).join().unwrap();
            assert!(!elsewhere, "the marker must not leak to another thread");
        }
        assert!(!in_rt_thread());
    }

    #[test]
    fn a_queued_entry_survives_the_ring_intact() {
        let _serial = serialised();
        let idx = queue(
            LogType::Warning,
            "probe_node",
            4242,
            17,
            format_args!("value {} and {:?}", 7, Some(3u8)),
        );
        let entry = read_slot(idx).expect("the entry just written must read back");
        assert_eq!(entry.0, LogType::Warning);
        assert_eq!(entry.1, "probe_node");
        assert_eq!(entry.2, 4242);
        assert_eq!(entry.3, 17);
        assert_eq!(entry.5, "value 7 and Some(3)");
    }

    #[test]
    fn a_message_longer_than_a_slot_is_truncated_on_a_utf8_boundary() {
        let _serial = serialised();
        let long = "é".repeat(MSG_CAP);
        let idx = queue(LogType::Info, "n", 0, 0, format_args!("{long}"));
        let entry = read_slot(idx).expect("a truncated entry is still a valid entry");
        assert!(entry.5.len() <= MSG_CAP);
        assert!(
            !entry.5.is_empty(),
            "truncation must not discard the whole message"
        );
        // The decisive property: it is still UTF-8. Splitting a multi-byte
        // character would make `String::from_utf8` fail and the drain would
        // drop the entry entirely.
        assert!(entry.5.chars().all(|c| c == 'é'));
    }

    #[test]
    fn the_ring_is_bounded_and_drops_the_oldest() {
        let _serial = serialised();
        // Fill well past capacity; the ring must overwrite rather than grow or
        // block. Blocking the producer is the entire defect being removed.
        let flood = SLOTS * 3;
        let mut first = 0;
        let mut last = 0;
        for i in 0..flood {
            let idx = queue(
                LogType::Info,
                "flood",
                i as u64,
                0,
                format_args!("line {i}"),
            );
            if i == 0 {
                first = idx;
            }
            last = idx;
        }
        assert!(
            read_slot(first).is_none(),
            "the oldest entry must have been overwritten, not retained"
        );
        assert!(
            read_slot(last).is_some(),
            "the newest entry must survive: the ring keeps the most recent SLOTS"
        );
    }
}
