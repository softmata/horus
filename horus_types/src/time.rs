// Clock and time synchronization message types
//
// This module provides time-related messages for simulation time,
// replay synchronization, and external time source references.

use horus_macros::LogSummary;
use serde::{Deserialize, Serialize};

/// Get current timestamp in nanoseconds since UNIX epoch.
fn timestamp_now() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

macro_rules! impl_pod_message {
    ($($ty:ty),+ $(,)?) => {
        $(
            unsafe impl horus_core::bytemuck::Pod for $ty {}
            unsafe impl horus_core::bytemuck::Zeroable for $ty {}
            unsafe impl horus_core::communication::PodMessage for $ty {}
        )+
    };
}

/// Clock source: wall clock time
pub const SOURCE_WALL: u8 = 0;
/// Clock source: simulation time
pub const SOURCE_SIM: u8 = 1;
/// Clock source: replay/log playback time
pub const SOURCE_REPLAY: u8 = 2;

/// Simulation/replay time broadcast
///
/// Published on a well-known topic (e.g. `clock`) to synchronize all nodes
/// to a common time source. Essential for simulation and log replay where
/// wall-clock time doesn't match the intended execution time.
///
/// Implements `PodMessage` for zero-copy shared memory transfer.
///
/// # Fields
///
/// - `clock_ns`: The current time according to the clock source (sim or replay time)
/// - `realtime_ns`: Wall clock time for comparison / rate computation
/// - `sim_speed`: Playback speed multiplier (1.0 = real-time, 2.0 = 2x speed)
/// - `paused`: Whether the clock is paused (0 = running, 1 = paused)
/// - `source`: Clock source (SOURCE_WALL=0, SOURCE_SIM=1, SOURCE_REPLAY=2)
///
/// # Example
///
/// ```rust,ignore
/// use horus::prelude::*;
///
/// // Publish simulation time
/// let topic: Topic<Clock> = Topic::new("clock")?;
/// let clock = Clock::sim_time(1_000_000_000, 1.0); // 1 second in, real-time speed
/// topic.send(clock);
/// ```
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct Clock {
    /// Current simulation/replay time in nanoseconds
    pub clock_ns: u64,
    /// Wall clock time for comparison
    pub realtime_ns: u64,
    /// Playback speed (1.0 = real-time, 2.0 = 2x, 0.5 = half speed)
    pub sim_speed: f64,
    /// 0 = running, 1 = paused
    pub paused: u8,
    /// SOURCE_WALL=0, SOURCE_SIM=1, SOURCE_REPLAY=2
    pub source: u8,
    /// Padding for alignment
    _pad: [u8; 6],
    /// Timestamp when this message was published
    pub timestamp_ns: u64,
}

impl Clock {
    /// Create a wall-clock time message (real-time, not simulated)
    pub fn wall_clock() -> Self {
        let now = timestamp_now();
        Self {
            clock_ns: now,
            realtime_ns: now,
            sim_speed: 1.0,
            paused: 0,
            source: SOURCE_WALL,
            _pad: [0; 6],
            timestamp_ns: now,
        }
    }

    /// Create a simulation time message
    pub fn sim_time(sim_ns: u64, speed: f64) -> Self {
        let now = timestamp_now();
        Self {
            clock_ns: sim_ns,
            realtime_ns: now,
            sim_speed: speed,
            paused: 0,
            source: SOURCE_SIM,
            _pad: [0; 6],
            timestamp_ns: now,
        }
    }

    /// Create a replay time message
    pub fn replay_time(replay_ns: u64, speed: f64) -> Self {
        let now = timestamp_now();
        Self {
            clock_ns: replay_ns,
            realtime_ns: now,
            sim_speed: speed,
            paused: 0,
            source: SOURCE_REPLAY,
            _pad: [0; 6],
            timestamp_ns: now,
        }
    }

    /// Elapsed time between this clock message and another (in nanoseconds)
    pub fn elapsed_since(&self, earlier: &Clock) -> u64 {
        self.clock_ns.saturating_sub(earlier.clock_ns)
    }

    /// Whether the clock is currently paused
    pub fn is_paused(&self) -> bool {
        self.paused != 0
    }

    /// Set paused state
    pub fn set_paused(mut self, paused: bool) -> Self {
        self.paused = if paused { 1 } else { 0 };
        self
    }
}

/// External time reference for synchronization
///
/// Used to communicate the offset between the local system clock and an
/// external reference (GPS, NTP, PTP). Nodes that need precise absolute
/// time can subscribe to this to correct their local timestamps.
///
/// Implements `PodMessage` for zero-copy shared memory transfer.
///
/// # Fields
///
/// - `time_ref_ns`: The reference time from the external source
/// - `source`: Name of the reference (e.g. "gps", "ntp", "ptp")
/// - `offset_ns`: Signed offset = local_time - reference_time
///
/// # Example
///
/// ```rust,ignore
/// use horus::prelude::*;
///
/// let topic: Topic<TimeReference> = Topic::new("time_reference")?;
/// let tref = TimeReference::new(gps_time_ns, "gps", local_ns as i64 - gps_time_ns as i64);
/// topic.send(tref);
/// ```
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct TimeReference {
    /// External reference time in nanoseconds
    pub time_ref_ns: u64,
    /// Source identifier (e.g. "gps", "ntp", "ptp"), null-terminated
    pub source: [u8; 32],
    /// Signed offset: local_time - reference_time (nanoseconds)
    pub offset_ns: i64,
    /// Timestamp when this message was published
    pub timestamp_ns: u64,
}

impl TimeReference {
    /// Create a new time reference
    pub fn new(time_ref_ns: u64, source_name: &str, offset_ns: i64) -> Self {
        let mut source = [0u8; 32];
        let bytes = source_name.as_bytes();
        let len = bytes.len().min(31);
        source[..len].copy_from_slice(&bytes[..len]);

        Self {
            time_ref_ns,
            source,
            offset_ns,
            timestamp_ns: timestamp_now(),
        }
    }

    /// Get the source name as a string
    pub fn source_name(&self) -> &str {
        let len = self.source.iter().position(|&b| b == 0).unwrap_or(32);
        std::str::from_utf8(&self.source[..len]).unwrap_or("")
    }

    /// Correct a local timestamp using this reference's offset
    pub fn correct_timestamp(&self, local_ns: u64) -> u64 {
        if self.offset_ns >= 0 {
            local_ns.saturating_sub(self.offset_ns as u64)
        } else {
            local_ns.saturating_add((-self.offset_ns) as u64)
        }
    }
}

// Register Pod types for zero-copy IPC
impl_pod_message!(Clock, TimeReference, SimSync, RateRequest);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clock_wall() {
        let c = Clock::wall_clock();
        assert_eq!(c.source, SOURCE_WALL);
        assert!(!c.is_paused());
        assert!((c.sim_speed - 1.0).abs() < f64::EPSILON);
        assert!(c.clock_ns > 0);
    }

    #[test]
    fn test_clock_sim() {
        let c = Clock::sim_time(500_000_000, 2.0);
        assert_eq!(c.source, SOURCE_SIM);
        assert_eq!(c.clock_ns, 500_000_000);
        assert!((c.sim_speed - 2.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_clock_replay() {
        let c = Clock::replay_time(1_000_000_000, 0.5);
        assert_eq!(c.source, SOURCE_REPLAY);
        assert_eq!(c.clock_ns, 1_000_000_000);
        assert!((c.sim_speed - 0.5).abs() < f64::EPSILON);
    }

    #[test]
    fn test_clock_paused() {
        let c = Clock::sim_time(0, 1.0).set_paused(true);
        assert!(c.is_paused());
        let c = c.set_paused(false);
        assert!(!c.is_paused());
    }

    #[test]
    fn test_clock_elapsed() {
        let c1 = Clock {
            clock_ns: 100,
            ..Default::default()
        };
        let c2 = Clock {
            clock_ns: 350,
            ..Default::default()
        };
        assert_eq!(c2.elapsed_since(&c1), 250);
    }

    #[test]
    fn test_time_reference_new() {
        let tr = TimeReference::new(1_000_000_000, "gps", -500);
        assert_eq!(tr.source_name(), "gps");
        assert_eq!(tr.time_ref_ns, 1_000_000_000);
        assert_eq!(tr.offset_ns, -500);
    }

    #[test]
    fn test_time_reference_correct() {
        // local is 500ns ahead of reference
        let tr = TimeReference::new(1000, "ntp", 500);
        // Correct local timestamp 1500 → should give 1000 (reference time)
        assert_eq!(tr.correct_timestamp(1500), 1000);

        // local is 500ns behind reference
        let tr = TimeReference::new(1000, "ptp", -500);
        assert_eq!(tr.correct_timestamp(500), 1000);
    }

    #[test]
    fn test_time_reference_source_name_truncation() {
        let long_name = "a_very_long_source_name_that_exceeds_32_bytes_limit";
        let tr = TimeReference::new(0, long_name, 0);
        assert!(tr.source_name().len() <= 31);
    }

    #[test]
    fn test_sizeof_time_types() {
        use std::mem::size_of;

        assert!(size_of::<Clock>() <= 128);
        assert!(size_of::<TimeReference>() <= 128);
        assert!(size_of::<SimSync>() <= 128);
        assert!(size_of::<RateRequest>() <= 128);
    }

    // ── SimSync Tests ──

    #[test]
    fn test_simsync_waiting() {
        let s = SimSync::waiting(42, 1_000_000_000, 1_000_000);
        assert_eq!(s.step, 42);
        assert_eq!(s.sim_time_ns, 1_000_000_000);
        assert_eq!(s.dt_ns, 1_000_000);
        assert_eq!(s.state, SimSync::WAITING);
    }

    #[test]
    fn test_simsync_done() {
        let s = SimSync::done(42);
        assert_eq!(s.step, 42);
        assert_eq!(s.state, SimSync::DONE);
    }

    #[test]
    fn test_simsync_default_is_idle() {
        let s = SimSync::default();
        assert_eq!(s.state, SimSync::IDLE);
        assert_eq!(s.step, 0);
    }

    // ── RateRequest Tests ──

    #[test]
    fn test_rate_request_new() {
        let rr = RateRequest::new("imu", 100.0, 50.0, 200.0);
        assert_eq!(rr.topic(), "imu");
        assert!((rr.desired_hz - 100.0).abs() < f64::EPSILON);
        assert!((rr.min_hz - 50.0).abs() < f64::EPSILON);
        assert!((rr.max_hz - 200.0).abs() < f64::EPSILON);
        assert_eq!(rr.requester_id, 0);
        assert!(rr.timestamp_ns > 0);
    }

    #[test]
    fn test_rate_request_with_requester() {
        let rr = RateRequest::new("cmd_vel", 30.0, 10.0, 60.0).with_requester(99);
        assert_eq!(rr.requester_id, 99);
        assert_eq!(rr.topic(), "cmd_vel");
    }

    #[test]
    fn test_rate_request_topic_truncation() {
        let long_name = "a".repeat(100);
        let rr = RateRequest::new(&long_name, 1.0, 1.0, 1.0);
        assert!(rr.topic().len() <= 31);
    }
}

/// Simulation synchronization message for deterministic co-simulation.
///
/// Enables lockstep mode between a simulator (e.g., horus-sim3d) and a
/// controller. The protocol:
///
/// 1. Simulator publishes `SimSync { step: N, state: WAITING }` on `sim_sync`
/// 2. Controller reads sensors, computes, publishes commands
/// 3. Controller publishes `SimSync { step: N, state: DONE }` on `sim_sync_ack`
/// 4. Simulator advances physics to step N+1
///
/// Without lockstep (default): both run asynchronously at their own rates.
/// With lockstep: deterministic, reproducible simulation at the cost of throughput.
///
/// # Topics
///
/// - `sim_sync` — published by simulator, read by controller
/// - `sim_sync_ack` — published by controller, read by simulator
///
/// # Example
///
/// ```rust,ignore
/// use horus::prelude::*;
/// use horus_types::time::SimSync;
///
/// // Controller side: wait for sim step, process, acknowledge
/// let sync_sub: Topic<SimSync> = Topic::new("sim_sync")?;
/// let sync_pub: Topic<SimSync> = Topic::new("sim_sync_ack")?;
///
/// if let Some(sync) = sync_sub.recv() {
///     if sync.state == SimSync::WAITING {
///         // Read sensors, compute, send commands...
///         sync_pub.send(SimSync::done(sync.step));
///     }
/// }
/// ```
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct SimSync {
    /// Simulation step number (monotonically increasing)
    pub step: u64,
    /// Simulation time in nanoseconds at this step
    pub sim_time_ns: u64,
    /// Physics timestep in nanoseconds (e.g., 1_000_000 for 1ms)
    pub dt_ns: u64,
    /// State: 0 = IDLE, 1 = WAITING (sim waiting for controller), 2 = DONE (controller done)
    pub state: u8,
    /// Padding for alignment
    pub _pad: [u8; 7],
}

impl SimSync {
    /// State: simulator is idle (lockstep not enabled)
    pub const IDLE: u8 = 0;
    /// State: simulator is waiting for controller to finish this step
    pub const WAITING: u8 = 1;
    /// State: controller has finished processing this step
    pub const DONE: u8 = 2;

    /// Create a "waiting" sync message (published by simulator)
    pub fn waiting(step: u64, sim_time_ns: u64, dt_ns: u64) -> Self {
        Self {
            step,
            sim_time_ns,
            dt_ns,
            state: Self::WAITING,
            _pad: [0; 7],
        }
    }

    /// Create a "done" sync message (published by controller)
    pub fn done(step: u64) -> Self {
        Self {
            step,
            state: Self::DONE,
            ..Default::default()
        }
    }
}

/// Rate negotiation request for dynamic rate adjustment.
///
/// Allows nodes to request a different publish/subscribe rate for a topic.
/// The scheduler or a rate-manager node can arbitrate between competing
/// requests and set the actual rate.
///
/// # Topics
///
/// - `rate_request` — published by any node wanting a rate change
/// - `rate_response` — published by scheduler/arbitrator with the granted rate
///
/// # Example
///
/// ```rust,ignore
/// use horus::prelude::*;
/// use horus_types::time::RateRequest;
///
/// // Node requests 100Hz for the "imu" topic
/// let rate_pub: Topic<RateRequest> = Topic::new("rate_request")?;
/// rate_pub.send(RateRequest::new("imu", 100.0, 50.0, 200.0));
/// ```
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct RateRequest {
    /// Topic name being negotiated (null-terminated, max 31 chars)
    pub topic_name: [u8; 32],
    /// Desired rate in Hz
    pub desired_hz: f64,
    /// Minimum acceptable rate in Hz
    pub min_hz: f64,
    /// Maximum acceptable rate in Hz
    pub max_hz: f64,
    /// ID of the requesting node/process
    pub requester_id: u64,
    /// Timestamp in nanoseconds
    pub timestamp_ns: u64,
}

impl RateRequest {
    /// Create a new rate request.
    pub fn new(topic: &str, desired_hz: f64, min_hz: f64, max_hz: f64) -> Self {
        let mut topic_name = [0u8; 32];
        let bytes = topic.as_bytes();
        let len = bytes.len().min(31);
        topic_name[..len].copy_from_slice(&bytes[..len]);

        Self {
            topic_name,
            desired_hz,
            min_hz,
            max_hz,
            requester_id: 0,
            timestamp_ns: timestamp_now(),
        }
    }

    /// Get the topic name as a string.
    pub fn topic(&self) -> &str {
        let len = self.topic_name.iter().position(|&b| b == 0).unwrap_or(32);
        std::str::from_utf8(&self.topic_name[..len]).unwrap_or("")
    }

    /// Set the requester ID.
    pub fn with_requester(mut self, id: u64) -> Self {
        self.requester_id = id;
        self
    }
}
