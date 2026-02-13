use horus_macros::LogSummary;
// Time synchronization and scheduling message types for robotics
//
// This module provides messages for time synchronization across distributed
// systems, scheduling, and temporal coordination in multi-robot systems.

use serde::{Deserialize, Serialize};
use serde_arrays;

/// Time synchronization message for distributed systems
///
/// Based on Network Time Protocol (NTP) and Precision Time Protocol (PTP)
/// concepts for synchronizing clocks across multiple robots/computers.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct TimeSync {
    /// Master/reference time in nanoseconds since epoch
    pub master_time: u64,
    /// Local time when message was sent in nanoseconds since epoch
    pub local_send_time: u64,
    /// Local time when message was received in nanoseconds since epoch
    pub local_receive_time: u64,
    /// Estimated network delay in nanoseconds
    pub network_delay: u64,
    /// Clock offset from master in nanoseconds (signed)
    pub clock_offset: i64,
    /// Clock drift rate (parts per million)
    pub drift_rate: f64,
    /// Synchronization accuracy estimate in nanoseconds
    pub accuracy_estimate: u64,
    /// Synchronization confidence (0.0-1.0)
    pub confidence: f32,
    /// Master clock identifier
    pub master_id: [u8; 32],
    /// Sync message sequence number
    pub sequence: u32,
    /// Time synchronization quality indicator
    pub sync_quality: SyncQuality,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

/// Time synchronization quality levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize, LogSummary)]
#[repr(u8)]
#[derive(Default)]
pub enum SyncQuality {
    /// No synchronization available
    #[default]
    None = 0,
    /// Poor synchronization (>1ms accuracy)
    Poor = 1,
    /// Fair synchronization (100us-1ms accuracy)
    Fair = 2,
    /// Good synchronization (10-100us accuracy)
    Good = 3,
    /// Excellent synchronization (<10us accuracy)
    Excellent = 4,
    /// Precision synchronization (<1us accuracy)
    Precision = 5,
}

impl TimeSync {
    /// Create a new time sync message
    pub fn new(master_id: &str) -> Self {
        let mut sync = Self {
            master_time: Self::current_time_ns(),
            local_send_time: Self::current_time_ns(),
            timestamp_ns: Self::current_time_ns(),
            confidence: 0.5,
            ..Default::default()
        };

        // Set master ID
        let id_bytes = master_id.as_bytes();
        let len = id_bytes.len().min(31);
        sync.master_id[..len].copy_from_slice(&id_bytes[..len]);
        sync.master_id[len] = 0;

        sync
    }

    /// Get current system time in nanoseconds
    fn current_time_ns() -> u64 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64
    }

    /// Update sync message with receive time
    pub fn update_receive_time(&mut self) {
        self.local_receive_time = Self::current_time_ns();
        self.calculate_sync_parameters();
    }

    /// Calculate synchronization parameters
    fn calculate_sync_parameters(&mut self) {
        if self.local_send_time > 0 && self.local_receive_time > 0 {
            // Estimate network round-trip delay
            let round_trip = self.local_receive_time.saturating_sub(self.local_send_time);
            self.network_delay = round_trip / 2;

            // Calculate clock offset
            let estimated_master_receive_time = self.master_time + self.network_delay;
            self.clock_offset =
                estimated_master_receive_time as i64 - self.local_receive_time as i64;

            // Estimate accuracy based on network delay
            self.accuracy_estimate = self.network_delay + 1000; // Add 1us base uncertainty

            // Determine sync quality
            self.sync_quality = if self.accuracy_estimate < 1_000 {
                SyncQuality::Precision
            } else if self.accuracy_estimate < 10_000 {
                SyncQuality::Excellent
            } else if self.accuracy_estimate < 100_000 {
                SyncQuality::Good
            } else if self.accuracy_estimate < 1_000_000 {
                SyncQuality::Fair
            } else {
                SyncQuality::Poor
            };

            // Update confidence based on quality
            self.confidence = match self.sync_quality {
                SyncQuality::Precision => 0.95,
                SyncQuality::Excellent => 0.9,
                SyncQuality::Good => 0.8,
                SyncQuality::Fair => 0.6,
                SyncQuality::Poor => 0.3,
                SyncQuality::None => 0.0,
            };
        }
    }

    /// Get synchronized time
    pub fn synchronized_time(&self) -> u64 {
        let local_time = Self::current_time_ns();
        (local_time as i64 + self.clock_offset) as u64
    }

    /// Check if synchronization is valid
    pub fn is_valid(&self) -> bool {
        self.confidence > 0.1 && self.sync_quality != SyncQuality::None
    }

    /// Get master ID as string
    pub fn master_id_str(&self) -> String {
        let end = self.master_id.iter().position(|&b| b == 0).unwrap_or(32);
        String::from_utf8_lossy(&self.master_id[..end]).into_owned()
    }
}

/// Scheduled task/event message
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct ScheduledEvent {
    /// Unique event identifier
    pub event_id: u32,
    /// Event type/category
    pub event_type: EventType,
    /// Scheduled execution time (synchronized time)
    pub scheduled_time: u64,
    /// Event duration in nanoseconds (0 = instantaneous)
    pub duration: u64,
    /// Event priority (0 = highest)
    pub priority: u8,
    /// Repeat interval (0 = no repeat)
    pub repeat_interval: u64,
    /// Maximum repeat count (0 = infinite)
    pub max_repeats: u32,
    /// Current repeat count
    pub repeat_count: u32,
    /// Event status
    pub status: EventStatus,
    /// Event parameters (flexible data)
    pub parameters: [u8; 32],
    /// Time tolerance (acceptable execution window)
    pub tolerance: u64,
    /// Robot/system responsible for this event
    pub executor_id: [u8; 32],
    /// Event creation time
    pub created_time: u64,
    /// Last execution time
    pub last_execution: u64,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

/// Event type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, LogSummary)]
#[repr(u8)]
#[derive(Default)]
pub enum EventType {
    /// Start a task or operation
    TaskStart = 0,
    /// Stop a task or operation
    TaskStop = 1,
    /// Send a command
    #[default]
    Command = 2,
    /// Collect sensor data
    DataCollection = 3,
    /// Perform maintenance operation
    Maintenance = 4,
    /// Synchronization checkpoint
    Synchronization = 5,
    /// System state change
    StateChange = 6,
    /// Communication event
    Communication = 7,
    /// Calibration procedure
    Calibration = 8,
    /// Safety check
    SafetyCheck = 9,
    /// Formation update
    FormationUpdate = 10,
    /// Custom user event
    Custom = 255,
}

/// Event execution status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, LogSummary)]
#[repr(u8)]
#[derive(Default)]
pub enum EventStatus {
    /// Event scheduled but not executed
    #[default]
    Scheduled = 0,
    /// Event is being executed
    Executing = 1,
    /// Event completed successfully
    Completed = 2,
    /// Event failed to execute
    Failed = 3,
    /// Event cancelled
    Cancelled = 4,
    /// Event missed its execution window
    Missed = 5,
    /// Event postponed to later time
    Postponed = 6,
}

impl ScheduledEvent {
    /// Create a new scheduled event
    pub fn new(event_id: u32, event_type: EventType, scheduled_time: u64) -> Self {
        Self {
            event_id,
            event_type,
            scheduled_time,
            tolerance: 1_000_000, // 1ms default tolerance
            created_time: TimeSync::current_time_ns(),
            timestamp_ns: TimeSync::current_time_ns(),
            ..Default::default()
        }
    }

    /// Create a repeating event
    pub fn repeating(event_id: u32, event_type: EventType, start_time: u64, interval: u64) -> Self {
        Self {
            event_id,
            event_type,
            scheduled_time: start_time,
            repeat_interval: interval,
            tolerance: 1_000_000,
            created_time: TimeSync::current_time_ns(),
            timestamp_ns: TimeSync::current_time_ns(),
            ..Default::default()
        }
    }

    /// Check if event is due for execution
    pub fn is_due(&self, current_time: u64) -> bool {
        if self.status != EventStatus::Scheduled {
            return false;
        }

        current_time >= self.scheduled_time.saturating_sub(self.tolerance)
            && current_time <= self.scheduled_time.saturating_add(self.tolerance)
    }

    /// Check if event has missed its execution window
    pub fn is_missed(&self, current_time: u64) -> bool {
        if self.status != EventStatus::Scheduled {
            return false;
        }

        current_time > self.scheduled_time + self.tolerance
    }

    /// Update event status
    pub fn update_status(&mut self, status: EventStatus) {
        self.status = status;
        self.timestamp_ns = TimeSync::current_time_ns();

        if status == EventStatus::Completed || status == EventStatus::Failed {
            self.last_execution = self.timestamp_ns;

            // Schedule next repeat if applicable
            if self.repeat_interval > 0 {
                self.repeat_count += 1;

                // Only reschedule if we haven't reached max repeats
                if self.max_repeats == 0 || self.repeat_count < self.max_repeats {
                    self.scheduled_time += self.repeat_interval;
                    self.status = EventStatus::Scheduled;
                }
            }
        }
    }

    /// Calculate time until execution
    pub fn time_until_execution(&self, current_time: u64) -> Option<u64> {
        if self.status == EventStatus::Scheduled && current_time < self.scheduled_time {
            Some(self.scheduled_time - current_time)
        } else {
            None
        }
    }

    /// Set executor ID
    pub fn with_executor(mut self, executor_id: &str) -> Self {
        let id_bytes = executor_id.as_bytes();
        let len = id_bytes.len().min(31);
        self.executor_id[..len].copy_from_slice(&id_bytes[..len]);
        self.executor_id[len] = 0;
        self
    }
}

/// Timeline/schedule of events
#[derive(Debug, Clone, Serialize, Deserialize, LogSummary)]
pub struct Timeline {
    /// Array of scheduled events (max 64)
    #[serde(with = "serde_arrays")]
    pub events: [ScheduledEvent; 64],
    /// Number of active events
    pub event_count: u8,
    /// Timeline identifier
    pub timeline_id: [u8; 32],
    /// Timeline start time
    pub start_time: u64,
    /// Timeline end time (0 = indefinite)
    pub end_time: u64,
    /// Current timeline position
    pub current_time: u64,
    /// Timeline execution status
    pub status: TimelineStatus,
    /// Timeline owner/coordinator
    pub coordinator_id: [u8; 32],
    /// Synchronization reference time
    pub sync_reference: u64,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

/// Timeline execution status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, LogSummary)]
#[repr(u8)]
#[derive(Default)]
pub enum TimelineStatus {
    /// Timeline created but not started
    #[default]
    Created = 0,
    /// Timeline is running
    Running = 1,
    /// Timeline paused
    Paused = 2,
    /// Timeline completed
    Completed = 3,
    /// Timeline aborted due to error
    Aborted = 4,
    /// Timeline stopped by user
    Stopped = 5,
}

impl Default for Timeline {
    fn default() -> Self {
        Self {
            events: [ScheduledEvent::default(); 64],
            event_count: 0,
            timeline_id: [0; 32],
            start_time: 0,
            end_time: 0,
            current_time: 0,
            status: TimelineStatus::Created,
            coordinator_id: [0; 32],
            sync_reference: 0,
            timestamp_ns: 0,
        }
    }
}

impl Timeline {
    /// Create a new timeline
    pub fn new(timeline_id: &str, start_time: u64) -> Self {
        let mut timeline = Self {
            start_time,
            current_time: start_time,
            status: TimelineStatus::Created,
            sync_reference: TimeSync::current_time_ns(),
            timestamp_ns: TimeSync::current_time_ns(),
            ..Default::default()
        };

        // Set timeline ID
        let id_bytes = timeline_id.as_bytes();
        let len = id_bytes.len().min(31);
        timeline.timeline_id[..len].copy_from_slice(&id_bytes[..len]);
        timeline.timeline_id[len] = 0;

        timeline
    }

    /// Add event to timeline
    pub fn add_event(&mut self, event: ScheduledEvent) -> Result<(), &'static str> {
        if self.event_count >= 64 {
            return Err("Maximum 64 events supported");
        }

        self.events[self.event_count as usize] = event;
        self.event_count += 1;

        // Sort events by scheduled time (simple bubble sort for small arrays)
        self.sort_events();

        self.timestamp_ns = TimeSync::current_time_ns();
        Ok(())
    }

    /// Sort events by scheduled time
    fn sort_events(&mut self) {
        let events_slice = &mut self.events[..self.event_count as usize];
        events_slice.sort_by_key(|event| event.scheduled_time);
    }

    /// Get active events
    pub fn get_events(&self) -> &[ScheduledEvent] {
        &self.events[..self.event_count as usize]
    }

    /// Get events due for execution
    pub fn get_due_events(&self, current_time: u64) -> Vec<usize> {
        self.get_events()
            .iter()
            .enumerate()
            .filter_map(|(i, event)| {
                if event.is_due(current_time) {
                    Some(i)
                } else {
                    None
                }
            })
            .collect()
    }

    /// Update timeline execution
    pub fn update(&mut self, current_synchronized_time: u64) {
        self.current_time = current_synchronized_time;

        // Check for missed events
        for event in &mut self.events[..self.event_count as usize] {
            if event.is_missed(current_synchronized_time) {
                event.update_status(EventStatus::Missed);
            }
        }

        // Check if timeline is complete
        if self.status == TimelineStatus::Running {
            let all_complete = self.get_events().iter().all(|event| {
                matches!(
                    event.status,
                    EventStatus::Completed
                        | EventStatus::Failed
                        | EventStatus::Cancelled
                        | EventStatus::Missed
                )
            });

            if all_complete {
                self.status = TimelineStatus::Completed;
            }
        }

        self.timestamp_ns = TimeSync::current_time_ns();
    }

    /// Start timeline execution
    pub fn start(&mut self) {
        if self.status == TimelineStatus::Created {
            self.status = TimelineStatus::Running;
            self.timestamp_ns = TimeSync::current_time_ns();
        }
    }

    /// Pause timeline execution
    pub fn pause(&mut self) {
        if self.status == TimelineStatus::Running {
            self.status = TimelineStatus::Paused;
            self.timestamp_ns = TimeSync::current_time_ns();
        }
    }

    /// Resume timeline execution
    pub fn resume(&mut self) {
        if self.status == TimelineStatus::Paused {
            self.status = TimelineStatus::Running;
            self.timestamp_ns = TimeSync::current_time_ns();
        }
    }

    /// Stop timeline execution
    pub fn stop(&mut self) {
        self.status = TimelineStatus::Stopped;
        self.timestamp_ns = TimeSync::current_time_ns();
    }
}

/// Clock synchronization statistics
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct ClockStats {
    /// Current clock offset from master (nanoseconds)
    pub current_offset: i64,
    /// Average offset over measurement period
    pub average_offset: i64,
    /// Offset standard deviation
    pub offset_std_dev: u64,
    /// Clock drift rate (parts per million)
    pub drift_ppm: f64,
    /// Number of sync messages received
    pub sync_count: u32,
    /// Number of sync messages lost/missed
    pub sync_lost: u32,
    /// Last sync message timestamp
    pub last_sync_time: u64,
    /// Sync interval in nanoseconds
    pub sync_interval: u64,
    /// Current sync quality
    pub sync_quality: SyncQuality,
    /// Master clock identifier
    pub master_id: [u8; 32],
    /// Statistics collection period
    pub measurement_period: u64,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl ClockStats {
    /// Create new clock statistics
    pub fn new(master_id: &str) -> Self {
        let mut stats = Self {
            sync_interval: 1_000_000_000,       // 1 second default
            measurement_period: 60_000_000_000, // 1 minute
            timestamp_ns: TimeSync::current_time_ns(),
            ..Default::default()
        };

        // Set master ID
        let id_bytes = master_id.as_bytes();
        let len = id_bytes.len().min(31);
        stats.master_id[..len].copy_from_slice(&id_bytes[..len]);
        stats.master_id[len] = 0;

        stats
    }

    /// Update statistics with new sync measurement
    pub fn update_sync(&mut self, offset: i64, quality: SyncQuality) {
        self.current_offset = offset;
        self.sync_quality = quality;
        self.sync_count += 1;
        self.last_sync_time = TimeSync::current_time_ns();

        // Update average offset (simple exponential moving average)
        if self.sync_count == 1 {
            self.average_offset = offset;
        } else {
            const ALPHA: f64 = 0.1; // Smoothing factor
            self.average_offset =
                (ALPHA * offset as f64 + (1.0 - ALPHA) * self.average_offset as f64) as i64;
        }

        self.timestamp_ns = self.last_sync_time;
    }

    /// Calculate sync success rate
    pub fn sync_success_rate(&self) -> f32 {
        if self.sync_count + self.sync_lost == 0 {
            return 1.0;
        }
        self.sync_count as f32 / (self.sync_count + self.sync_lost) as f32
    }

    /// Check if synchronization is healthy
    pub fn is_sync_healthy(&self) -> bool {
        let current_time = TimeSync::current_time_ns();
        let time_since_sync = current_time.saturating_sub(self.last_sync_time);

        self.sync_quality >= SyncQuality::Fair &&
        time_since_sync < self.sync_interval * 3 && // Allow up to 3 missed syncs
        self.sync_success_rate() > 0.8
    }

    /// Get estimated time accuracy in nanoseconds
    pub fn estimated_accuracy(&self) -> u64 {
        match self.sync_quality {
            SyncQuality::Precision => 1_000,  // 1 microsecond
            SyncQuality::Excellent => 10_000, // 10 microseconds
            SyncQuality::Good => 100_000,     // 100 microseconds
            SyncQuality::Fair => 1_000_000,   // 1 millisecond
            SyncQuality::Poor => 10_000_000,  // 10 milliseconds
            SyncQuality::None => u64::MAX,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_time_sync() {
        let mut sync = TimeSync::new("master_clock");
        assert_eq!(sync.master_id_str(), "master_clock");
        assert_eq!(sync.sync_quality, SyncQuality::None);

        // Simulate message round trip
        sync.local_send_time = 1000000000; // 1 second
        sync.master_time = 1000000000;
        sync.update_receive_time();

        assert!(sync.network_delay > 0);
        assert!(sync.is_valid());
    }

    #[test]
    fn test_scheduled_event() {
        let mut event = ScheduledEvent::new(1, EventType::TaskStart, 2000000000);

        // Test event timing
        assert!(event.is_due(1999999000)); // Within tolerance
        assert!(!event.is_due(1000000000)); // Too early
        assert!(event.is_missed(3000000000)); // Too late

        // Test status updates
        event.update_status(EventStatus::Completed);
        assert_eq!(event.status, EventStatus::Completed);
        assert!(event.last_execution > 0);
    }

    #[test]
    fn test_repeating_event() {
        let mut event =
            ScheduledEvent::repeating(2, EventType::DataCollection, 1000000000, 500000000);
        event.max_repeats = 3;

        assert_eq!(event.repeat_count, 0);
        assert_eq!(event.scheduled_time, 1000000000);

        // Complete first execution
        event.update_status(EventStatus::Completed);
        assert_eq!(event.repeat_count, 1);
        assert_eq!(event.scheduled_time, 1500000000); // +500ms
        assert_eq!(event.status, EventStatus::Scheduled); // Auto-rescheduled

        // Complete remaining executions
        event.update_status(EventStatus::Completed);
        event.update_status(EventStatus::Completed);

        assert_eq!(event.repeat_count, 3);
        assert_eq!(event.status, EventStatus::Completed); // No more repeats
    }

    #[test]
    fn test_timeline() {
        let mut timeline = Timeline::new("test_timeline", 1000000000);

        let event1 = ScheduledEvent::new(1, EventType::TaskStart, 1100000000);
        let event2 = ScheduledEvent::new(2, EventType::TaskStop, 1050000000); // Earlier

        timeline.add_event(event1).unwrap();
        timeline.add_event(event2).unwrap();

        assert_eq!(timeline.event_count, 2);
        // Events should be sorted by time
        assert_eq!(timeline.events[0].scheduled_time, 1050000000);
        assert_eq!(timeline.events[1].scheduled_time, 1100000000);

        timeline.start();
        assert_eq!(timeline.status, TimelineStatus::Running);

        // Test due events
        let due_events = timeline.get_due_events(1050000000);
        assert_eq!(due_events.len(), 1);
        assert_eq!(due_events[0], 0); // First event is due
    }

    #[test]
    fn test_clock_stats() {
        let mut stats = ClockStats::new("test_master");

        assert_eq!(stats.sync_count, 0);
        assert_eq!(stats.sync_success_rate(), 1.0);

        stats.update_sync(-5000, SyncQuality::Good);
        assert_eq!(stats.current_offset, -5000);
        assert_eq!(stats.average_offset, -5000);
        assert_eq!(stats.sync_count, 1);

        stats.update_sync(-3000, SyncQuality::Good);
        assert_eq!(stats.current_offset, -3000);
        // Average should be between -5000 and -3000
        assert!(stats.average_offset > -5000 && stats.average_offset < -3000);

        assert!(stats.is_sync_healthy());
        assert_eq!(stats.estimated_accuracy(), 100_000); // Good quality = 100us
    }

    #[test]
    fn test_sync_quality_ordering() {
        assert!(SyncQuality::Precision > SyncQuality::Excellent);
        assert!(SyncQuality::Excellent > SyncQuality::Good);
        assert!(SyncQuality::Good > SyncQuality::Fair);
        assert!(SyncQuality::Fair > SyncQuality::Poor);
        assert!(SyncQuality::Poor > SyncQuality::None);
    }
}
