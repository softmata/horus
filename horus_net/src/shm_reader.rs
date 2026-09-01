//! ShmRingReader — read new data from local SHM topics for network export.
//!
//! Wraps `horus_core::communication::topic::read_latest_slot_bytes` and
//! `read_slots_since` to poll local SHM ring buffers and yield new entries
//! since the last read.
//!
//! Two things a reader owes its caller: the messages, and an honest account of
//! the ones it did not hand over. The export path runs on a 50 ms timer, so a
//! reader that only ever takes the newest slot caps every replicated topic at
//! ~20 Hz however fast the publisher runs. That is the right trade for state —
//! a pose, a battery level, a mode flag, where the freshest value is the only
//! one worth a datagram — and the wrong one for a measurement stream, where the
//! consumer integrates every sample. Whichever it is, the count of what was
//! dropped is not optional: a fleet robot whose odometry crosses the LAN at
//! 20 Hz instead of 500 Hz looks, from the other end, exactly like a robot
//! whose odometry runs at 20 Hz.

use std::path::PathBuf;
use std::time::{Duration, Instant};

use horus_core::communication::topic::{read_latest_slot_bytes, read_slots_since, TopicSlotRead};
use horus_sys::shm::topic_shm_path;

use crate::priority::Encoding;

/// How much of a topic's ring the exporter takes on each poll.
///
/// The export loop ticks at a fixed rate, so this is the difference between a
/// topic being *sampled* onto the network and being *streamed* onto it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ExportSampling {
    /// Newest slot only; everything published since the previous poll is
    /// dropped, and counted in [`ShmRingReader::skipped_decimated`].
    ///
    /// Correct for state-like topics, where a stale sample has no value once a
    /// fresher one exists. It is the default because it is also the bandwidth
    /// floor: switching a fleet to full streams without asking could saturate
    /// the link the safety heartbeat shares.
    #[default]
    LatestOnly,
    /// Every message published since the previous poll, oldest first, up to the
    /// caller's per-poll bound. Anything beyond the bound is not lost — it is
    /// returned by the next poll, because the reader resumes from its own
    /// cursor rather than from the head.
    AllSlots,
}

impl ExportSampling {
    /// Parse a configured mode name. `None` for anything unrecognised, so a
    /// caller can fall through to its own default rather than guessing.
    pub fn parse(raw: &str) -> Option<Self> {
        match raw.trim().to_ascii_lowercase().as_str() {
            "all" | "all_slots" | "stream" | "every" | "full" => Some(Self::AllSlots),
            "latest" | "latest_only" | "newest" | "sample" | "sampled" => Some(Self::LatestOnly),
            _ => None,
        }
    }

    /// Whether this mode replicates every message.
    pub fn is_stream(self) -> bool {
        matches!(self, Self::AllSlots)
    }
}

/// How often a topic that is losing messages says so, at most.
///
/// The first loss on a topic is reported immediately — an operator should not
/// have to wait out an interval to learn the fleet is dropping data — and every
/// report after that is spaced by this, because the alternative on a 1 kHz topic
/// is a log line per tick that nobody reads.
pub const SKIP_REPORT_INTERVAL: Duration = Duration::from_secs(10);

/// Raw message read from SHM, ready for network transmission.
#[derive(Debug, Clone)]
pub struct RawShmMessage {
    /// Raw payload bytes (POD struct bytes or bincode-serialized).
    pub data: Vec<u8>,
    /// Monotonic write index (serves as sequence number).
    pub write_idx: u64,
    /// Whether the payload is POD (raw bytes) or serialized.
    pub is_pod: bool,
    /// Encoding tag for the wire format.
    pub encoding: Encoding,
}

/// An account of what a topic did not put on the wire, ready to be logged.
///
/// Covers the window since the previous report for this topic, so the rates it
/// quotes are the rates an operator would measure over that window.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SkipReport {
    /// Topic this is about.
    pub topic: String,
    /// Sampling mode in force while the window ran.
    pub sampling: ExportSampling,
    /// Messages exported during the window.
    pub forwarded: u64,
    /// Messages dropped because latest-only sampling took the newest slot only.
    pub decimated: u64,
    /// Messages the publisher overwrote before the exporter reached them.
    pub lapped: u64,
    /// Wall time the window covers.
    pub window: Duration,
}

impl SkipReport {
    /// Messages that existed locally during the window, exported or not.
    pub fn published(&self) -> u64 {
        self.forwarded
            .saturating_add(self.decimated)
            .saturating_add(self.lapped)
    }

    /// Messages that existed locally and did not leave this machine.
    pub fn skipped(&self) -> u64 {
        self.decimated.saturating_add(self.lapped)
    }

    /// The operator-facing line. Names the topic, the numbers, the cause, and
    /// the one setting that changes it — a warning that does not say what to do
    /// about it is just noise on a robot's console.
    pub fn message(&self) -> String {
        let secs = self.window.as_secs_f64().max(0.001);
        let published = self.published();
        let mut msg = format!(
            "[horus_net] Topic '{}' exported {} of {} messages in the last {:.1}s \
             ({:.0} Hz of {:.0} Hz published).",
            self.topic,
            self.forwarded,
            published,
            secs,
            self.forwarded as f64 / secs,
            published as f64 / secs,
        );
        if self.decimated > 0 {
            msg.push_str(&format!(
                " {} were dropped by latest-only export, which takes one newest sample \
                 per tick: correct for state (a pose, a battery level), wrong for a \
                 measurement stream (odometry, joint states, lidar). \
                 Set HORUS_NET_EXPORT_STREAM='{}' (globs allowed) to replicate every message.",
                self.decimated, self.topic
            ));
        }
        if self.lapped > 0 {
            msg.push_str(&format!(
                " {} were overwritten in the local ring before the exporter reached them — \
                 the publisher is outrunning the export tick. Give the topic a larger ring \
                 or publish more slowly.",
                self.lapped
            ));
        }
        msg
    }
}

/// Reader that polls a local SHM topic ring buffer for new data.
///
/// Each reader tracks its last-read position. Calling `read_pending()` returns
/// what is new since that position, under the reader's [`ExportSampling`] mode,
/// and every message it does *not* return is counted.
pub struct ShmRingReader {
    /// Path to the SHM backing file for this topic.
    shm_path: PathBuf,
    /// Last write index we've seen — used to detect new data.
    last_write_idx: u64,
    /// Topic name (for logging).
    topic_name: String,
    /// How much of the ring each poll takes.
    sampling: ExportSampling,
    // ── What this reader did and did not put on the wire ──
    /// Messages handed to the export path, for the life of the reader.
    forwarded: u64,
    /// Messages that were in the ring and were skipped because latest-only
    /// sampling takes the newest slot and leaves the rest.
    skipped_decimated: u64,
    /// Messages the publisher overwrote before this reader reached them.
    skipped_lapped: u64,
    // ── The same three, since the last report ──
    window_forwarded: u64,
    window_decimated: u64,
    window_lapped: u64,
    /// When the current reporting window opened.
    window_start: Instant,
    /// Whether a report has ever been emitted for this reader.
    reported: bool,
}

impl ShmRingReader {
    /// Create a reader for a specific topic.
    pub fn new(topic_name: &str) -> Self {
        // The path must come from the same helper `ShmRegion::new` uses. This
        // previously built `horus_<sanitized>`, which stopped matching when the
        // `horus_` prefix was dropped on 2026-03-29 — so the export side of LAN
        // replication silently read nothing for four months.
        let shm_path = topic_shm_path(topic_name);
        Self::at(topic_name, shm_path)
    }

    /// Create a reader with a custom SHM path (for testing).
    pub fn with_path(topic_name: &str, shm_path: PathBuf) -> Self {
        Self::at(topic_name, shm_path)
    }

    fn at(topic_name: &str, shm_path: PathBuf) -> Self {
        Self {
            shm_path,
            last_write_idx: 0,
            topic_name: topic_name.to_string(),
            sampling: ExportSampling::default(),
            forwarded: 0,
            skipped_decimated: 0,
            skipped_lapped: 0,
            window_forwarded: 0,
            window_decimated: 0,
            window_lapped: 0,
            window_start: Instant::now(),
            reported: false,
        }
    }

    /// Builder form of [`Self::set_sampling`].
    pub fn with_sampling(mut self, sampling: ExportSampling) -> Self {
        self.sampling = sampling;
        self
    }

    /// Change how much of the ring each poll takes.
    pub fn set_sampling(&mut self, sampling: ExportSampling) {
        self.sampling = sampling;
    }

    /// Current sampling mode.
    pub fn sampling(&self) -> ExportSampling {
        self.sampling
    }

    /// Read what is new since the last poll, under this reader's sampling mode.
    ///
    /// `max` bounds the work of one poll in [`ExportSampling::AllSlots`]; the
    /// remainder is returned by the next poll rather than dropped. In
    /// [`ExportSampling::LatestOnly`] the result is at most one message and
    /// `max` is irrelevant.
    pub fn read_pending(&mut self, max: usize) -> Vec<RawShmMessage> {
        match self.sampling {
            ExportSampling::LatestOnly => self.try_read_latest().into_iter().collect(),
            ExportSampling::AllSlots => self.try_read_all(max),
        }
    }

    /// Try to read the latest message from the SHM ring buffer.
    ///
    /// Returns `Some(RawShmMessage)` if new data is available since the last read.
    /// Returns `None` if no new data, or if the SHM file doesn't exist yet.
    ///
    /// This is non-blocking and allocation-free on the "no new data" path.
    pub fn try_read_latest(&mut self) -> Option<RawShmMessage> {
        let slot = read_latest_slot_bytes(&self.shm_path, self.last_write_idx)?;

        // Everything published between the previous poll and this one was in the
        // ring and is not going anywhere: this call hands back the newest slot
        // and nothing else. The gap used to be computed right here and thrown
        // away inside an empty `if gap > 1 {}` block — that empty block was the
        // whole of the "downsampling is fine for network replication" argument,
        // and it is what made a 25x decimation of a robot's odometry invisible
        // at both ends of the link.
        //
        // A first read is not decimation: a reader that has never polled starts
        // at "now" rather than replaying whatever history the ring holds.
        if self.last_write_idx > 0 {
            let skipped = slot
                .write_idx
                .saturating_sub(self.last_write_idx)
                .saturating_sub(1);
            self.skipped_decimated = self.skipped_decimated.saturating_add(skipped);
            self.window_decimated = self.window_decimated.saturating_add(skipped);
        }
        self.last_write_idx = slot.write_idx;
        self.count_forwarded(1);

        Some(Self::to_raw(slot))
    }

    /// Try to read every message published since the last poll, oldest first.
    ///
    /// At most `max` per call; the rest arrive on the next one. Messages the
    /// publisher overwrote before this reader reached them are counted in
    /// [`Self::skipped_lapped`] rather than left as a silent hole in the
    /// sequence numbering the peer sees.
    pub fn try_read_all(&mut self, max: usize) -> Vec<RawShmMessage> {
        if max == 0 {
            return Vec::new();
        }

        // Probe the head first. `read_slots_since` cannot tell "no new data"
        // apart from "the ring restarted below our cursor" — both come back as
        // an empty batch — and the second case would wedge this reader forever:
        // a publisher process that is replaced, recreating the backing file,
        // resets the write index to 1 while our cursor sits at, say, 40000, and
        // the topic would then never export again with nothing said about it.
        // The latest-only path self-heals from that by construction, so the
        // streaming path has to as well.
        let Some(head) = read_latest_slot_bytes(&self.shm_path, self.last_write_idx) else {
            return Vec::new();
        };
        if head.write_idx < self.last_write_idx {
            self.last_write_idx = 0;
        }

        // First poll (or a resynchronisation): start at the head rather than
        // replaying a full ring of history nobody asked for.
        if self.last_write_idx == 0 {
            self.last_write_idx = head.write_idx;
            self.count_forwarded(1);
            return vec![Self::to_raw(head)];
        }

        let (slots, lapped) = read_slots_since(&self.shm_path, self.last_write_idx, max);
        if slots.is_empty() && lapped == 0 {
            return Vec::new();
        }

        // `read_slots_since` walks the ordinals `cursor+1 ..= cursor+want`
        // contiguously and accounts for every one of them: it either returns the
        // slot or counts it lapped. So the cursor advances by exactly what was
        // accounted for — which also means a reader that fell an entire ring
        // behind climbs out instead of re-counting the same unreachable window
        // on every tick.
        let accounted = (slots.len() as u64).saturating_add(lapped);
        self.last_write_idx = self.last_write_idx.saturating_add(accounted);

        if lapped > 0 {
            self.skipped_lapped = self.skipped_lapped.saturating_add(lapped);
            self.window_lapped = self.window_lapped.saturating_add(lapped);
        }
        self.count_forwarded(slots.len() as u64);

        slots.into_iter().map(Self::to_raw).collect()
    }

    /// A due account of the messages this topic is not replicating, or `None`
    /// when there is nothing to report or the previous report is too recent.
    ///
    /// `now` is passed in rather than read here so the cadence is testable
    /// without sleeping.
    pub fn due_report(&mut self, now: Instant) -> Option<SkipReport> {
        if self.window_decimated == 0 && self.window_lapped == 0 {
            return None;
        }
        let window = now.saturating_duration_since(self.window_start);
        if self.reported && window < SKIP_REPORT_INTERVAL {
            return None;
        }

        let report = SkipReport {
            topic: self.topic_name.clone(),
            sampling: self.sampling,
            forwarded: self.window_forwarded,
            decimated: self.window_decimated,
            lapped: self.window_lapped,
            window,
        };

        self.reported = true;
        self.window_start = now;
        self.window_forwarded = 0;
        self.window_decimated = 0;
        self.window_lapped = 0;

        Some(report)
    }

    /// Check if the SHM file exists (topic has been created locally).
    pub fn shm_exists(&self) -> bool {
        self.shm_path.exists()
    }

    /// Topic name.
    pub fn topic_name(&self) -> &str {
        &self.topic_name
    }

    /// Current write index (0 if never read).
    pub fn last_write_idx(&self) -> u64 {
        self.last_write_idx
    }

    /// Messages handed to the export path over this reader's life.
    pub fn forwarded(&self) -> u64 {
        self.forwarded
    }

    /// Messages left behind by latest-only sampling over this reader's life.
    pub fn skipped_decimated(&self) -> u64 {
        self.skipped_decimated
    }

    /// Messages the publisher overwrote before this reader reached them.
    pub fn skipped_lapped(&self) -> u64 {
        self.skipped_lapped
    }

    /// Every local message this reader did not put on the wire.
    pub fn skipped_total(&self) -> u64 {
        self.skipped_decimated.saturating_add(self.skipped_lapped)
    }

    fn count_forwarded(&mut self, n: u64) {
        self.forwarded = self.forwarded.saturating_add(n);
        self.window_forwarded = self.window_forwarded.saturating_add(n);
    }

    fn to_raw(slot: TopicSlotRead) -> RawShmMessage {
        let encoding = if slot.is_pod {
            Encoding::native_pod()
        } else {
            Encoding::Bincode
        };

        RawShmMessage {
            data: slot.payload,
            write_idx: slot.write_idx,
            is_pod: slot.is_pod,
            encoding,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reader_creation() {
        let reader = ShmRingReader::new("robot.imu");
        assert_eq!(reader.topic_name(), "robot.imu");
        assert_eq!(reader.last_write_idx(), 0);
        // SHM file won't exist in test — that's fine
    }

    #[test]
    fn reader_nonexistent_shm_returns_none() {
        let mut reader = ShmRingReader::with_path(
            "ghost_topic",
            PathBuf::from("/tmp/nonexistent_shm_file_horus_net_test"),
        );
        assert!(reader.try_read_latest().is_none());
        assert!(reader.try_read_all(64).is_empty());
        assert_eq!(reader.skipped_total(), 0);
    }

    #[test]
    fn reader_path_matches_where_horus_core_actually_writes() {
        // This test previously asserted the mangled `horus_robot_front_lidar_scan`
        // form, which is how the export seam stayed broken for four months with a
        // green suite: the test encoded the bug rather than the contract. The
        // contract is `ShmRegion::new`'s path, and nothing else.
        let name = "robot.front_lidar.scan";
        let reader = ShmRingReader::new(name);
        assert_eq!(
            reader.shm_path,
            topic_shm_path(name),
            "reader must open exactly the file ShmRegion::new creates"
        );
        assert!(
            reader.shm_path.ends_with(name),
            "the topic name is used verbatim — no prefix, no character mangling"
        );
    }

    #[test]
    fn reader_and_writer_agree_on_the_path() {
        // The two halves of the replication seam diverging is the original bug;
        // pin them to each other so they cannot drift apart again silently.
        let name = "robot.imu";
        let reader = ShmRingReader::new(name);
        assert_eq!(reader.shm_path, topic_shm_path(name));
    }

    #[test]
    fn default_sampling_is_latest_only() {
        let reader = ShmRingReader::new("robot.pose");
        assert_eq!(reader.sampling(), ExportSampling::LatestOnly);
        assert!(!reader.sampling().is_stream());
    }

    #[test]
    fn sampling_mode_parses_both_spellings_and_rejects_nonsense() {
        for stream in ["stream", "all", "ALL_SLOTS", " every ", "full"] {
            assert_eq!(
                ExportSampling::parse(stream),
                Some(ExportSampling::AllSlots)
            );
        }
        for latest in ["latest", "latest_only", "Newest", "sample", "sampled"] {
            assert_eq!(
                ExportSampling::parse(latest),
                Some(ExportSampling::LatestOnly)
            );
        }
        // A typo must not silently pick a mode for the operator.
        assert_eq!(ExportSampling::parse("streem"), None);
        assert_eq!(ExportSampling::parse(""), None);
    }

    fn report(decimated: u64, lapped: u64) -> SkipReport {
        SkipReport {
            topic: "odom".into(),
            sampling: if lapped > 0 {
                ExportSampling::AllSlots
            } else {
                ExportSampling::LatestOnly
            },
            forwarded: 20,
            decimated,
            lapped,
            window: Duration::from_secs(10),
        }
    }

    #[test]
    fn a_decimation_report_names_the_topic_the_count_and_the_way_out() {
        let msg = report(480, 0).message();
        assert!(msg.contains("'odom'"), "{msg}");
        assert!(msg.contains("exported 20 of 500"), "{msg}");
        assert!(
            msg.contains("480 were dropped by latest-only export"),
            "{msg}"
        );
        assert!(
            msg.contains("HORUS_NET_EXPORT_STREAM='odom'"),
            "a warning with no remedy is noise: {msg}"
        );
    }

    #[test]
    fn a_lapped_report_blames_the_ring_not_the_sampling() {
        let msg = report(0, 7).message();
        assert!(
            msg.contains("7 were overwritten in the local ring"),
            "{msg}"
        );
        assert!(
            !msg.contains("latest-only export"),
            "stream mode does not decimate; saying so would send the operator \
             chasing the wrong setting: {msg}"
        );
    }

    #[test]
    fn report_arithmetic_counts_every_local_message_once() {
        let r = report(480, 7);
        assert_eq!(r.published(), 507);
        assert_eq!(r.skipped(), 487);
    }

    #[test]
    fn no_report_while_nothing_is_being_skipped() {
        let mut reader = ShmRingReader::new("robot.pose");
        reader.count_forwarded(100);
        assert!(reader.due_report(Instant::now()).is_none());
    }

    #[test]
    fn the_first_skip_is_reported_at_once_then_rate_limited() {
        let mut reader = ShmRingReader::new("robot.odom");
        let t0 = Instant::now();

        reader.count_forwarded(1);
        reader.window_decimated = 24;
        let first = reader
            .due_report(t0)
            .expect("the first loss on a topic must not wait out an interval");
        assert_eq!(first.decimated, 24);
        assert_eq!(first.forwarded, 1);

        // Counters reset with the window, so an immediate re-poll says nothing.
        assert!(reader.due_report(t0).is_none());

        // More loss, but too soon: still silent.
        reader.window_decimated = 24;
        assert!(reader.due_report(t0 + Duration::from_secs(3)).is_none());

        // Past the interval it speaks again, and reports the whole backlog —
        // the losses that accumulated while it was quiet are not forgotten.
        reader.window_decimated += 100;
        let second = reader
            .due_report(t0 + SKIP_REPORT_INTERVAL)
            .expect("a report is due once the interval has passed");
        assert_eq!(second.decimated, 124);
        assert_eq!(second.window, SKIP_REPORT_INTERVAL);
    }
}
