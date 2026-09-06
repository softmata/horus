//! The flight recorder's producer side: a wait-free, allocation-free event
//! ring that keeps `serde_json` and the write(2) off the tick thread.
//!
//! # What this removes
//!
//! `BlackBox::record` runs, in order, on whichever thread called it:
//! `serde_json::to_string` (several heap allocations as the `Vec<u8>` grows), a
//! `writeln!` that memcpys into an 8 KiB `BufWriter` and issues a `write(2)`
//! whenever that fills, and a `flush()` — another `write(2)` — every
//! `wal_flush_interval` records. The shipped interval is 64 and nothing
//! configures it.
//!
//! Every tick-path caller was inside the loop:
//!
//! * the RT threads, at SCHED_FIFO where granted, from inside `tick_node` for
//!   `BudgetViolation`, `DeadlineMiss` and `NodeError`;
//! * the main tick thread, whose `while is_running()` body took a **blocking**
//!   `lock()` on the recorder once per iteration just to bump a counter, and
//!   again per node per tick for budget and deadline events;
//! * the compute, event and async workers, for `NodeError`.
//!
//! Two things the report of this defect did not name make it worse.
//!
//! **It is unthrottled.** `DiagThrottle` caps the *console* line at one per
//! second; the blackbox write sits outside that gate and fires on every miss. A
//! 1 kHz node in sustained overload does a thousand serialise-and-write cycles
//! per second on its RT thread.
//!
//! **The loss was silent, and the inversion ran both ways.** Every executor
//! site was `if let Ok(mut bb) = bb.try_lock()` with no `else`: when the main
//! thread held the lock — which it did every loop iteration — the RT thread's
//! deadline-miss record was dropped and nothing counted it (`lost_records`
//! counts only `VecDeque` evictions). And the main thread's *blocking* `lock()`
//! could block behind an RT thread that had won `try_lock` and was inside
//! `serde_json` plus a `write(2)` — so the main loop's period was coupled to
//! disk latency on a real-time thread.
//!
//! # Why the ring carries a POD event and not bytes
//!
//! Pre-serialising on the producer would move nothing: the serialisation IS the
//! cost. The slot carries fixed-size fields and the drain rebuilds the
//! `BlackBoxEvent` and serialises it off-thread.
//!
//! # What it costs
//!
//! A record reaches the WAL up to one 10 ms poll late. The blackbox's whole
//! purpose is surviving a crash, so `emergency_drain` runs synchronously from
//! the panic path and from `finalize_run` before the stop marker is written.
//! Under a storm the ring overwrites its oldest entry and counts the loss —
//! which is strictly better than the previous behaviour, where a lost record
//! was invisible.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use super::blackbox::{BlackBox, BlackBoxEvent};

/// Inline capacity for a node name.
const NAME_CAP: usize = 64;
/// Inline capacity for an error string or a `Custom` body.
const TEXT_CAP: usize = 192;
/// Slots in the ring. Power of two.
const SLOTS: usize = 512;
const SLOT_MASK: u64 = SLOTS as u64 - 1;
/// How often the drain empties the ring into the recorder.
const POLL: Duration = Duration::from_millis(10);

/// Which event a slot holds.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
enum BbKind {
    NodeTick = 1,
    NodeError = 2,
    DeadlineMiss = 3,
    BudgetViolation = 4,
    EmergencyStop = 5,
    Custom = 6,
}

impl BbKind {
    fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            1 => Self::NodeTick,
            2 => Self::NodeError,
            3 => Self::DeadlineMiss,
            4 => Self::BudgetViolation,
            5 => Self::EmergencyStop,
            6 => Self::Custom,
            _ => return None,
        })
    }
}

fn severity_to_u8(s: crate::error::Severity) -> u8 {
    match s {
        crate::error::Severity::Transient => 0,
        crate::error::Severity::Permanent => 1,
        crate::error::Severity::Fatal => 2,
    }
}

fn severity_from_u8(v: u8) -> crate::error::Severity {
    match v {
        0 => crate::error::Severity::Transient,
        1 => crate::error::Severity::Permanent,
        // Anything unrecognised is Fatal: under-reporting the severity of a
        // recorded fault is the worse direction for a flight recorder.
        _ => crate::error::Severity::Fatal,
    }
}

/// One queued event, in fixed-size form.
struct BbSlot {
    /// Seqlock, same protocol as the RT diagnostic ring: `0` = never written,
    /// odd = write in progress, even and nonzero = complete.
    seq: AtomicU64,
    kind: AtomicU8,
    severity: AtomicU8,
    flag: AtomicBool,
    a_us: AtomicU64,
    b_us: AtomicU64,
    tick: AtomicU64,
    timestamp_us: AtomicU64,
    name_len: AtomicU32,
    text_len: AtomicU32,
    name: [AtomicU8; NAME_CAP],
    text: [AtomicU8; TEXT_CAP],
}

impl BbSlot {
    const fn new() -> Self {
        Self {
            seq: AtomicU64::new(0),
            kind: AtomicU8::new(0),
            severity: AtomicU8::new(0),
            flag: AtomicBool::new(false),
            a_us: AtomicU64::new(0),
            b_us: AtomicU64::new(0),
            tick: AtomicU64::new(0),
            timestamp_us: AtomicU64::new(0),
            name_len: AtomicU32::new(0),
            text_len: AtomicU32::new(0),
            name: [const { AtomicU8::new(0) }; NAME_CAP],
            text: [const { AtomicU8::new(0) }; TEXT_CAP],
        }
    }
}

/// The producer handle every executor gets.
///
/// It can queue events and nothing else. An executor cannot reach the recorder,
/// its `VecDeque` or its WAL — that is enforced by this type rather than by a
/// comment, which is what the six `try_lock` sites needed.
pub struct BbRing {
    slots: Box<[BbSlot; SLOTS]>,
    head: AtomicU64,
    dropped: AtomicU64,
    /// Scheduler tick counter, bumped once per loop iteration. This replaced a
    /// blocking `lock()` on the recorder, taken every iteration for exactly
    /// this.
    tick: AtomicU64,
    /// Next index to drain. Drainers only; producers never touch it.
    tail: Mutex<u64>,
}

impl Default for BbRing {
    fn default() -> Self {
        Self::new()
    }
}

impl BbRing {
    pub fn new() -> Self {
        Self {
            slots: Box::new([const { BbSlot::new() }; SLOTS]),
            head: AtomicU64::new(0),
            dropped: AtomicU64::new(0),
            tick: AtomicU64::new(0),
            tail: Mutex::new(0),
        }
    }

    /// Advance the scheduler tick counter.
    pub fn bump_tick(&self) {
        self.tick.fetch_add(1, Ordering::Relaxed);
    }

    /// The current tick, for the drain to stamp cold records with.
    pub fn current_tick(&self) -> u64 {
        self.tick.load(Ordering::Relaxed)
    }

    /// Events overwritten before a drain reached them. Monotonic.
    pub fn dropped_count(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    #[allow(clippy::too_many_arguments)]
    fn emit(
        &self,
        kind: BbKind,
        name: &str,
        text: &str,
        severity: u8,
        flag: bool,
        a_us: u64,
        b_us: u64,
    ) {
        let idx = self.head.fetch_add(1, Ordering::Relaxed);
        let slot = &self.slots[(idx & SLOT_MASK) as usize];

        slot.seq
            .store(idx.wrapping_mul(2).wrapping_add(1), Ordering::Relaxed);
        std::sync::atomic::fence(Ordering::Release);

        slot.kind.store(kind as u8, Ordering::Relaxed);
        slot.severity.store(severity, Ordering::Relaxed);
        slot.flag.store(flag, Ordering::Relaxed);
        slot.a_us.store(a_us, Ordering::Relaxed);
        slot.b_us.store(b_us, Ordering::Relaxed);
        slot.tick
            .store(self.tick.load(Ordering::Relaxed), Ordering::Relaxed);
        // The record has to carry the time it HAPPENED rather than the time the
        // drain got to it — a flight recorder whose timestamps are the drain's
        // is useless for ordering a fault.
        //
        // Cost is a property of the host, not of this call. `SystemTime::now`
        // is served from the vDSO only when the active clocksource has a vDSO
        // mode: `tsc` and `kvm-clock` do, `hpet` lost its page in 4.20 and
        // `acpi_pm` never had one, and on those it is a real syscall — measured
        // 25.4 ns against 187.3 ns on this project's own clocksource probe.
        // `RtCapabilities::clocksource` reports which one the box is on, and
        // that is the number to look at before blaming this line.
        slot.timestamp_us.store(
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64,
            Ordering::Relaxed,
        );

        let n = copy_bytes(&slot.name, name);
        slot.name_len.store(n as u32, Ordering::Relaxed);
        let t = copy_bytes(&slot.text, text);
        slot.text_len.store(t as u32, Ordering::Relaxed);

        slot.seq
            .store(idx.wrapping_mul(2).wrapping_add(2), Ordering::Release);
    }

    pub fn emit_node_tick(&self, name: &str, duration_us: u64, success: bool) {
        self.emit(BbKind::NodeTick, name, "", 0, success, duration_us, 0);
    }

    pub fn emit_node_error(&self, name: &str, error: &str, severity: crate::error::Severity) {
        self.emit(
            BbKind::NodeError,
            name,
            error,
            severity_to_u8(severity),
            false,
            0,
            0,
        );
    }

    pub fn emit_deadline_miss(&self, name: &str, deadline_us: u64, actual_us: u64) {
        self.emit(
            BbKind::DeadlineMiss,
            name,
            "",
            0,
            false,
            deadline_us,
            actual_us,
        );
    }

    pub fn emit_budget_violation(&self, name: &str, budget_us: u64, actual_us: u64) {
        self.emit(
            BbKind::BudgetViolation,
            name,
            "",
            0,
            false,
            budget_us,
            actual_us,
        );
    }

    pub fn emit_emergency_stop(&self, reason: &str) {
        self.emit(BbKind::EmergencyStop, "", reason, 0, false, 0, 0);
    }

    pub fn emit_custom(&self, category: &str, message: &str) {
        self.emit(BbKind::Custom, category, message, 0, false, 0, 0);
    }

    /// Read one complete slot back, or `None` if it was recycled or is
    /// mid-write.
    fn read(&self, idx: u64) -> Option<(BlackBoxEvent, u64, u64)> {
        let slot = &self.slots[(idx & SLOT_MASK) as usize];
        let want = idx.wrapping_mul(2).wrapping_add(2);
        if slot.seq.load(Ordering::Acquire) != want {
            return None;
        }

        let kind = BbKind::from_u8(slot.kind.load(Ordering::Relaxed))?;
        let severity = severity_from_u8(slot.severity.load(Ordering::Relaxed));
        let flag = slot.flag.load(Ordering::Relaxed);
        let a_us = slot.a_us.load(Ordering::Relaxed);
        let b_us = slot.b_us.load(Ordering::Relaxed);
        let tick = slot.tick.load(Ordering::Relaxed);
        let timestamp_us = slot.timestamp_us.load(Ordering::Relaxed);
        let name = read_bytes(&slot.name, slot.name_len.load(Ordering::Relaxed) as usize)?;
        let text = read_bytes(&slot.text, slot.text_len.load(Ordering::Relaxed) as usize)?;

        // Re-check: a producer that lapped us mid-copy invalidates all of it,
        // and a garbled record is worse than a dropped one.
        if slot.seq.load(Ordering::Acquire) != want {
            return None;
        }

        let event = match kind {
            BbKind::NodeTick => BlackBoxEvent::NodeTick {
                name,
                duration_us: a_us,
                success: flag,
            },
            BbKind::NodeError => BlackBoxEvent::NodeError {
                name,
                error: text,
                severity,
            },
            BbKind::DeadlineMiss => BlackBoxEvent::DeadlineMiss {
                name,
                deadline_us: a_us,
                actual_us: b_us,
            },
            BbKind::BudgetViolation => BlackBoxEvent::BudgetViolation {
                name,
                budget_us: a_us,
                actual_us: b_us,
            },
            BbKind::EmergencyStop => BlackBoxEvent::EmergencyStop { reason: text },
            BbKind::Custom => BlackBoxEvent::Custom {
                category: name,
                message: text,
            },
        };
        Some((event, tick, timestamp_us))
    }

    /// Move every complete event into the recorder. Returns how many.
    ///
    /// Never called from a tick thread: it is where the `serde_json` and the
    /// `write(2)` happen.
    pub fn drain_into(&self, bb: &mut BlackBox) -> usize {
        let mut tail = self.tail.lock().unwrap_or_else(|e| e.into_inner());
        let head = self.head.load(Ordering::Acquire);

        if head.saturating_sub(*tail) > SLOTS as u64 {
            let lost = head - *tail - SLOTS as u64;
            self.dropped.fetch_add(lost, Ordering::Relaxed);
            *tail = head - SLOTS as u64;
        }

        let mut drained = 0;
        while *tail < head {
            let idx = *tail;
            *tail += 1;
            if let Some((event, tick, timestamp_us)) = self.read(idx) {
                bb.record_prestamped(event, tick, timestamp_us);
                drained += 1;
            }
        }
        drained
    }
}

fn copy_bytes(dst: &[AtomicU8], s: &str) -> usize {
    let bytes = s.as_bytes();
    let mut n = bytes.len().min(dst.len());
    // Never split a multi-byte character: the drain reads it back as UTF-8 and
    // would discard the whole record.
    while n > 0 && n < bytes.len() && (bytes[n] & 0xC0) == 0x80 {
        n -= 1;
    }
    for (i, b) in bytes[..n].iter().enumerate() {
        dst[i].store(*b, Ordering::Relaxed);
    }
    n
}

fn read_bytes(src: &[AtomicU8], len: usize) -> Option<String> {
    let len = len.min(src.len());
    let bytes: Vec<u8> = src[..len]
        .iter()
        .map(|b| b.load(Ordering::Relaxed))
        .collect();
    String::from_utf8(bytes).ok()
}

/// Start a drain thread that empties `ring` into `bb` until `running` clears.
///
/// Spawned on the caller's thread — the main scheduler thread — so it cannot
/// inherit SCHED_FIFO or the reserved CPU mask.
pub fn start_drain(
    ring: Arc<BbRing>,
    bb: Arc<Mutex<BlackBox>>,
    running: Arc<AtomicBool>,
) -> Option<std::thread::JoinHandle<()>> {
    crate::scheduling::rt::spawn_best_effort("horus-blackbox-drain", 5, move || {
        while running.load(Ordering::Relaxed) {
            {
                let mut guard = bb.lock().unwrap_or_else(|e| e.into_inner());
                ring.drain_into(&mut guard);
            }
            std::thread::sleep(POLL);
        }
        // Final pass: a run that ends between polls must not lose the events
        // that describe why it ended.
        let mut guard = bb.lock().unwrap_or_else(|e| e.into_inner());
        ring.drain_into(&mut guard);
        guard.flush_wal();
    })
    .ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_event_kind_round_trips_through_a_slot() {
        let ring = BbRing::new();
        ring.bump_tick();
        ring.emit_budget_violation("motor", 1000, 1500);
        ring.emit_deadline_miss("planner", 5000, 7200);
        ring.emit_node_error("vision", "camera timeout", crate::error::Severity::Fatal);
        ring.emit_emergency_stop("watchdog starved");
        ring.emit_custom("safety", "entered safe state");
        ring.emit_node_tick("arm", 42, true);

        let events: Vec<BlackBoxEvent> = (0..6).filter_map(|i| ring.read(i).map(|r| r.0)).collect();
        assert_eq!(events.len(), 6, "every queued event must read back");

        assert!(matches!(
            &events[0],
            BlackBoxEvent::BudgetViolation { name, budget_us: 1000, actual_us: 1500 } if name == "motor"
        ));
        assert!(matches!(
            &events[1],
            BlackBoxEvent::DeadlineMiss { name, deadline_us: 5000, actual_us: 7200 } if name == "planner"
        ));
        assert!(matches!(
            &events[2],
            BlackBoxEvent::NodeError { name, error, severity: crate::error::Severity::Fatal }
                if name == "vision" && error == "camera timeout"
        ));
        assert!(
            matches!(&events[3], BlackBoxEvent::EmergencyStop { reason } if reason == "watchdog starved")
        );
        assert!(matches!(
            &events[4],
            BlackBoxEvent::Custom { category, message } if category == "safety" && message == "entered safe state"
        ));
        assert!(matches!(
            &events[5],
            BlackBoxEvent::NodeTick { name, duration_us: 42, success: true } if name == "arm"
        ));

        // The tick the event happened on, not the tick the drain ran at.
        assert_eq!(ring.read(0).unwrap().1, 1);
    }

    #[test]
    fn the_ring_is_bounded_and_counts_what_it_dropped() {
        let ring = BbRing::new();
        for i in 0..(SLOTS * 3) {
            ring.emit_custom("flood", &format!("event {i}"));
        }
        assert!(
            ring.read(0).is_none(),
            "the oldest event must have been overwritten rather than retained"
        );

        let mut bb = BlackBox::new(1);
        let drained = ring.drain_into(&mut bb);
        assert!(
            drained <= SLOTS,
            "the ring must hold at most {SLOTS} events, drained {drained}"
        );
        assert!(
            ring.dropped_count() > 0,
            "loss must be counted: the previous behaviour dropped a record on a \
             failed try_lock and nothing recorded that it had happened"
        );
    }

    /// A record must carry the time it happened, not the time the drain got to
    /// it — a flight recorder whose timestamps are the drain's cannot order a
    /// fault.
    #[test]
    fn a_queued_event_keeps_its_own_timestamp_and_tick() {
        let ring = BbRing::new();
        for _ in 0..7 {
            ring.bump_tick();
        }
        ring.emit_node_error("n", "boom", crate::error::Severity::Permanent);
        let (_, tick, ts) = ring.read(0).expect("event");
        assert_eq!(tick, 7);
        assert!(ts > 0, "the timestamp must be stamped by the producer");
    }

    #[test]
    fn an_oversized_string_is_truncated_on_a_utf8_boundary() {
        let ring = BbRing::new();
        let long = "é".repeat(TEXT_CAP);
        ring.emit_node_error("n", &long, crate::error::Severity::Fatal);
        let (event, _, _) = ring.read(0).expect("a truncated event is still an event");
        match event {
            BlackBoxEvent::NodeError { error, .. } => {
                assert!(!error.is_empty());
                assert!(error.len() <= TEXT_CAP);
                assert!(
                    error.chars().all(|c| c == 'é'),
                    "splitting a character would make the whole record undecodable"
                );
            }
            other => panic!("wrong kind: {other:?}"),
        }
    }
}
