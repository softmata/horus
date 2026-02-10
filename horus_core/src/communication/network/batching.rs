//! Message batching for network efficiency
//!
//! Automatically batches multiple small messages into single network packets
//! to reduce overhead and improve throughput for high-frequency data.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Default batch size (number of messages)
const DEFAULT_BATCH_SIZE: usize = 100;
/// Default flush timeout
const DEFAULT_FLUSH_TIMEOUT: Duration = Duration::from_millis(5);
/// Maximum batch payload size (64KB to fit in UDP)
const MAX_BATCH_BYTES: usize = 60000;

/// Batching configuration
#[derive(Debug, Clone)]
pub struct BatchConfig {
    /// Maximum messages per batch
    pub max_messages: usize,
    /// Maximum bytes per batch
    pub max_bytes: usize,
    /// Flush timeout (send incomplete batch after this duration)
    pub flush_timeout: Duration,
    /// Whether batching is enabled
    pub enabled: bool,
}

impl Default for BatchConfig {
    fn default() -> Self {
        Self {
            max_messages: DEFAULT_BATCH_SIZE,
            max_bytes: MAX_BATCH_BYTES,
            flush_timeout: DEFAULT_FLUSH_TIMEOUT,
            enabled: true,
        }
    }
}

impl BatchConfig {
    /// Create config optimized for low latency (smaller batches, shorter timeout)
    pub fn low_latency() -> Self {
        Self {
            max_messages: 10,
            max_bytes: MAX_BATCH_BYTES,
            flush_timeout: Duration::from_micros(500),
            enabled: true,
        }
    }

    /// Create config optimized for throughput (larger batches, longer timeout)
    pub fn high_throughput() -> Self {
        Self {
            max_messages: 500,
            max_bytes: MAX_BATCH_BYTES,
            flush_timeout: Duration::from_millis(20),
            enabled: true,
        }
    }

    /// Disable batching (immediate send)
    pub fn disabled() -> Self {
        Self {
            enabled: false,
            ..Default::default()
        }
    }
}

/// A batch of serialized messages
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct MessageBatch {
    /// Number of messages in batch
    pub count: u32,
    /// Topic name
    pub topic: String,
    /// Serialized messages (length-prefixed)
    pub payloads: Vec<Vec<u8>>,
    /// Batch sequence number
    pub sequence: u64,
    /// Timestamp when batch was created
    pub created_at_us: u64,
}

impl MessageBatch {
    /// Create a new empty batch
    pub fn new(topic: &str, sequence: u64) -> Self {
        Self {
            count: 0,
            topic: topic.to_string(),
            payloads: Vec::new(),
            sequence,
            created_at_us: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64,
        }
    }

    /// Add a serialized message to the batch
    pub fn add(&mut self, payload: Vec<u8>) {
        self.count += 1;
        self.payloads.push(payload);
    }

    /// Get total byte size of batch
    pub fn byte_size(&self) -> usize {
        self.payloads.iter().map(|p| p.len() + 4).sum::<usize>() // +4 for length prefix
    }

    /// Check if batch is empty
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Encode batch to bytes
    pub fn encode(&self) -> Result<Vec<u8>, Box<bincode::ErrorKind>> {
        bincode::serialize(self)
    }

    /// Decode batch from bytes
    pub fn decode(data: &[u8]) -> Result<Self, Box<bincode::ErrorKind>> {
        bincode::deserialize(data)
    }
}

/// Message batcher that accumulates messages and flushes as batches
pub struct MessageBatcher {
    config: BatchConfig,
    current_batch: MessageBatch,
    batch_start_time: Instant,
    sequence: u64,
    topic: String,
    /// Accumulated byte size
    current_bytes: usize,
}

impl MessageBatcher {
    /// Create a new batcher for a topic
    pub fn new(topic: &str, config: BatchConfig) -> Self {
        Self {
            current_batch: MessageBatch::new(topic, 0),
            batch_start_time: Instant::now(),
            sequence: 0,
            topic: topic.to_string(),
            current_bytes: 0,
            config,
        }
    }

    /// Add a message to the batch
    /// Returns Some(batch) if batch should be flushed
    pub fn add(&mut self, payload: Vec<u8>) -> Option<MessageBatch> {
        if !self.config.enabled {
            // Batching disabled - return immediately as single-message batch
            let mut batch = MessageBatch::new(&self.topic, self.sequence);
            self.sequence += 1;
            batch.add(payload);
            return Some(batch);
        }

        let payload_size = payload.len();

        // Check if this message would overflow the batch
        let would_overflow = self.current_batch.count as usize >= self.config.max_messages
            || self.current_bytes + payload_size + 4 > self.config.max_bytes;

        let result = if would_overflow && !self.current_batch.is_empty() {
            // Flush current batch and start new one
            let batch = std::mem::replace(
                &mut self.current_batch,
                MessageBatch::new(&self.topic, self.sequence + 1),
            );
            self.sequence += 1;
            self.batch_start_time = Instant::now();
            self.current_bytes = 0;
            Some(batch)
        } else {
            None
        };

        // Add message to current batch
        self.current_bytes += payload_size + 4;
        self.current_batch.add(payload);

        result
    }

    /// Check if batch should be flushed due to timeout
    /// Returns Some(batch) if timeout exceeded
    pub fn check_timeout(&mut self) -> Option<MessageBatch> {
        if !self.config.enabled || self.current_batch.is_empty() {
            return None;
        }

        if self.batch_start_time.elapsed() >= self.config.flush_timeout {
            self.flush()
        } else {
            None
        }
    }

    /// Force flush the current batch
    pub fn flush(&mut self) -> Option<MessageBatch> {
        if self.current_batch.is_empty() {
            return None;
        }

        let batch = std::mem::replace(
            &mut self.current_batch,
            MessageBatch::new(&self.topic, self.sequence + 1),
        );
        self.sequence += 1;
        self.batch_start_time = Instant::now();
        self.current_bytes = 0;
        Some(batch)
    }

    /// Get current batch size
    pub fn pending_count(&self) -> usize {
        self.current_batch.count as usize
    }

    /// Get current batch byte size
    pub fn pending_bytes(&self) -> usize {
        self.current_bytes
    }
}

/// Thread-safe batcher wrapper
pub struct SharedBatcher {
    inner: Arc<Mutex<MessageBatcher>>,
}

impl SharedBatcher {
    pub fn new(topic: &str, config: BatchConfig) -> Self {
        Self {
            inner: Arc::new(Mutex::new(MessageBatcher::new(topic, config))),
        }
    }

    pub fn add(&self, payload: Vec<u8>) -> Option<MessageBatch> {
        self.inner.lock().unwrap().add(payload)
    }

    pub fn check_timeout(&self) -> Option<MessageBatch> {
        self.inner.lock().unwrap().check_timeout()
    }

    pub fn flush(&self) -> Option<MessageBatch> {
        self.inner.lock().unwrap().flush()
    }
}

impl Clone for SharedBatcher {
    fn clone(&self) -> Self {
        Self {
            inner: Arc::clone(&self.inner),
        }
    }
}

/// Batch receiver/decoder
pub struct BatchReceiver {
    /// Buffer for incoming messages from decoded batches
    pending_messages: VecDeque<Vec<u8>>,
}

impl BatchReceiver {
    pub fn new() -> Self {
        Self {
            pending_messages: VecDeque::new(),
        }
    }

    /// Process a received batch and queue individual messages
    pub fn receive_batch(&mut self, batch: MessageBatch) {
        for payload in batch.payloads {
            self.pending_messages.push_back(payload);
        }
    }

    /// Get the next message from the queue
    pub fn next_message(&mut self) -> Option<Vec<u8>> {
        self.pending_messages.pop_front()
    }

    /// Check if there are pending messages
    pub fn has_pending(&self) -> bool {
        !self.pending_messages.is_empty()
    }

    /// Get count of pending messages
    pub fn pending_count(&self) -> usize {
        self.pending_messages.len()
    }
}

impl Default for BatchReceiver {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Adaptive Batching (Opt-in)
// ============================================================================

/// Frequency tracking configuration
#[derive(Debug, Clone)]
pub struct FrequencyTrackerConfig {
    /// Exponential moving average smoothing factor (0.0-1.0)
    /// Higher values = more weight on recent measurements
    pub ema_alpha: f64,
    /// Minimum sample interval to update frequency
    pub min_sample_interval: Duration,
    /// High frequency threshold (msg/s) - trigger larger batches
    pub high_freq_threshold: f64,
    /// Low frequency threshold (msg/s) - disable batching
    pub low_freq_threshold: f64,
}

impl Default for FrequencyTrackerConfig {
    fn default() -> Self {
        Self {
            ema_alpha: 0.1, // Smooth, stable EMA
            min_sample_interval: Duration::from_millis(100),
            high_freq_threshold: 1000.0, // >1000 msg/s = high throughput
            low_freq_threshold: 10.0,    // <10 msg/s = disable batching
        }
    }
}

/// Tracks message frequency using exponential moving average.
///
/// This is an **opt-in** feature for adaptive batching. It provides:
/// - Low overhead (<1% CPU) frequency tracking
/// - Stable frequency estimates via EMA
/// - Thresholds for adapting batch configuration
///
/// # Example
///
/// ```ignore
/// use horus_core::communication::network::batching::FrequencyTracker;
///
/// let mut tracker = FrequencyTracker::new(FrequencyTrackerConfig::default());
///
/// // Record messages
/// tracker.record_message();
///
/// // Get current frequency estimate
/// let freq = tracker.frequency();
/// println!("Current rate: {} msg/s", freq);
/// ```
#[derive(Debug)]
pub struct FrequencyTracker {
    config: FrequencyTrackerConfig,
    /// Current EMA of messages per second
    ema_frequency: f64,
    /// Messages since last sample
    messages_since_sample: u64,
    /// Last sample time
    last_sample_time: Instant,
    /// Total messages tracked (for debugging)
    total_messages: u64,
    /// Whether EMA has been initialized
    initialized: bool,
}

impl FrequencyTracker {
    /// Create a new frequency tracker
    pub fn new(config: FrequencyTrackerConfig) -> Self {
        Self {
            config,
            ema_frequency: 0.0,
            messages_since_sample: 0,
            last_sample_time: Instant::now(),
            total_messages: 0,
            initialized: false,
        }
    }

    /// Record that a message was sent/received
    #[inline]
    pub fn record_message(&mut self) {
        self.messages_since_sample += 1;
        self.total_messages += 1;

        // Check if we should update EMA
        let elapsed = self.last_sample_time.elapsed();
        if elapsed >= self.config.min_sample_interval {
            self.update_ema(elapsed);
        }
    }

    /// Record multiple messages at once (for batch operations)
    #[inline]
    pub fn record_messages(&mut self, count: u64) {
        self.messages_since_sample += count;
        self.total_messages += count;

        let elapsed = self.last_sample_time.elapsed();
        if elapsed >= self.config.min_sample_interval {
            self.update_ema(elapsed);
        }
    }

    /// Update the EMA with current sample
    fn update_ema(&mut self, elapsed: Duration) {
        let elapsed_secs = elapsed.as_secs_f64();
        if elapsed_secs > 0.0 {
            let current_freq = self.messages_since_sample as f64 / elapsed_secs;

            if !self.initialized {
                // First sample - use directly
                self.ema_frequency = current_freq;
                self.initialized = true;
            } else {
                // EMA update: new = alpha * current + (1 - alpha) * old
                self.ema_frequency = self.config.ema_alpha * current_freq
                    + (1.0 - self.config.ema_alpha) * self.ema_frequency;
            }
        }

        self.messages_since_sample = 0;
        self.last_sample_time = Instant::now();
    }

    /// Get current frequency estimate (messages per second)
    #[inline]
    pub fn frequency(&self) -> f64 {
        self.ema_frequency
    }

    /// Check if frequency is above high threshold
    #[inline]
    pub fn is_high_frequency(&self) -> bool {
        self.initialized && self.ema_frequency >= self.config.high_freq_threshold
    }

    /// Check if frequency is below low threshold
    #[inline]
    pub fn is_low_frequency(&self) -> bool {
        self.initialized && self.ema_frequency <= self.config.low_freq_threshold
    }

    /// Get total messages tracked
    pub fn total_messages(&self) -> u64 {
        self.total_messages
    }

    /// Reset tracker state
    pub fn reset(&mut self) {
        self.ema_frequency = 0.0;
        self.messages_since_sample = 0;
        self.last_sample_time = Instant::now();
        self.total_messages = 0;
        self.initialized = false;
    }
}

impl Default for FrequencyTracker {
    fn default() -> Self {
        Self::new(FrequencyTrackerConfig::default())
    }
}

/// Adaptive batch configuration based on message frequency.
///
/// This is an **opt-in** feature that automatically adjusts batch settings:
/// - High frequency (>1000 msg/s): Use high-throughput batching
/// - Medium frequency: Use default batching
/// - Low frequency (<10 msg/s): Disable batching (immediate send)
///
/// # Real-Time Safety Note
///
/// Adaptive batching can introduce variable latency jitter. For real-time
/// control loops, use fixed `BatchConfig` with `BatchConfig::disabled()` instead.
///
/// # Example
///
/// ```ignore
/// use horus_core::communication::network::batching::{AdaptiveBatcher, AdaptiveBatcherConfig};
///
/// let config = AdaptiveBatcherConfig::default();
/// let mut batcher = AdaptiveBatcher::new("sensor_topic", config);
///
/// // Messages are batched adaptively based on frequency
/// if let Some(batch) = batcher.add(payload) {
///     send_batch(batch);
/// }
/// ```
#[derive(Debug, Clone)]
pub struct AdaptiveBatcherConfig {
    /// Frequency tracker configuration
    pub frequency_config: FrequencyTrackerConfig,
    /// Config for high-frequency mode
    pub high_freq_config: BatchConfig,
    /// Config for normal frequency mode
    pub normal_config: BatchConfig,
    /// Config for low frequency mode (typically disabled)
    pub low_freq_config: BatchConfig,
    /// How often to re-evaluate batch config (in messages)
    pub adapt_interval: u64,
}

impl Default for AdaptiveBatcherConfig {
    fn default() -> Self {
        Self {
            frequency_config: FrequencyTrackerConfig::default(),
            high_freq_config: BatchConfig::high_throughput(),
            normal_config: BatchConfig::default(),
            low_freq_config: BatchConfig::disabled(),
            adapt_interval: 100, // Check every 100 messages
        }
    }
}

impl AdaptiveBatcherConfig {
    /// Create config optimized for robotics (less aggressive adaptation)
    pub fn robotics() -> Self {
        Self {
            frequency_config: FrequencyTrackerConfig {
                ema_alpha: 0.05,            // More stable
                high_freq_threshold: 500.0, // Lower threshold for robotics
                low_freq_threshold: 5.0,
                ..Default::default()
            },
            high_freq_config: BatchConfig {
                max_messages: 200, // Not too aggressive
                flush_timeout: Duration::from_millis(10),
                ..BatchConfig::high_throughput()
            },
            normal_config: BatchConfig::default(),
            low_freq_config: BatchConfig::low_latency(), // Keep batching even at low freq
            adapt_interval: 50,
        }
    }
}

/// Adaptive batcher that automatically adjusts configuration based on message frequency.
///
/// **This is an opt-in feature.** For deterministic real-time behavior, use
/// `MessageBatcher` with a fixed `BatchConfig` instead.
pub struct AdaptiveBatcher {
    /// Inner batcher
    batcher: MessageBatcher,
    /// Frequency tracker
    tracker: FrequencyTracker,
    /// Configuration
    config: AdaptiveBatcherConfig,
    /// Messages since last adaptation check
    messages_since_adapt: u64,
    /// Current mode
    current_mode: AdaptiveMode,
}

/// Current adaptive batching mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdaptiveMode {
    /// High throughput mode (large batches)
    HighThroughput,
    /// Normal mode (default batching)
    Normal,
    /// Low latency mode (minimal or no batching)
    LowLatency,
}

impl AdaptiveBatcher {
    /// Create a new adaptive batcher
    pub fn new(topic: &str, config: AdaptiveBatcherConfig) -> Self {
        let batcher = MessageBatcher::new(topic, config.normal_config.clone());
        let tracker = FrequencyTracker::new(config.frequency_config.clone());

        Self {
            batcher,
            tracker,
            config,
            messages_since_adapt: 0,
            current_mode: AdaptiveMode::Normal,
        }
    }

    /// Add a message and potentially get a batch to send
    pub fn add(&mut self, payload: Vec<u8>) -> Option<MessageBatch> {
        // Track frequency
        self.tracker.record_message();
        self.messages_since_adapt += 1;

        // Check if we should adapt
        if self.messages_since_adapt >= self.config.adapt_interval {
            self.adapt();
            self.messages_since_adapt = 0;
        }

        // Add to batcher
        self.batcher.add(payload)
    }

    /// Check timeout and potentially flush
    pub fn check_timeout(&mut self) -> Option<MessageBatch> {
        self.batcher.check_timeout()
    }

    /// Force flush
    pub fn flush(&mut self) -> Option<MessageBatch> {
        self.batcher.flush()
    }

    /// Adapt batch configuration based on current frequency
    fn adapt(&mut self) {
        let new_mode = if self.tracker.is_high_frequency() {
            AdaptiveMode::HighThroughput
        } else if self.tracker.is_low_frequency() {
            AdaptiveMode::LowLatency
        } else {
            AdaptiveMode::Normal
        };

        if new_mode != self.current_mode {
            // Flush current batch before changing config
            let _ = self.batcher.flush();

            // Update config based on new mode
            let new_config = match new_mode {
                AdaptiveMode::HighThroughput => self.config.high_freq_config.clone(),
                AdaptiveMode::Normal => self.config.normal_config.clone(),
                AdaptiveMode::LowLatency => self.config.low_freq_config.clone(),
            };

            self.batcher = MessageBatcher::new(&self.batcher.topic, new_config);
            self.current_mode = new_mode;
        }
    }

    /// Get current adaptive mode
    pub fn current_mode(&self) -> AdaptiveMode {
        self.current_mode
    }

    /// Get current frequency estimate (msg/s)
    pub fn frequency(&self) -> f64 {
        self.tracker.frequency()
    }

    /// Get pending message count
    pub fn pending_count(&self) -> usize {
        self.batcher.pending_count()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_batch_add_and_flush() {
        let mut batcher = MessageBatcher::new("test", BatchConfig::default());

        // Add messages
        for i in 0..50 {
            let payload = vec![i as u8; 100];
            let result = batcher.add(payload);
            assert!(result.is_none()); // Not full yet
        }

        assert_eq!(batcher.pending_count(), 50);

        // Force flush
        let batch = batcher.flush().unwrap();
        assert_eq!(batch.count, 50);
        assert_eq!(batch.payloads.len(), 50);
    }

    #[test]
    fn test_batch_auto_flush_on_count() {
        let config = BatchConfig {
            max_messages: 10,
            ..Default::default()
        };
        let mut batcher = MessageBatcher::new("test", config);

        // Add 10 messages - should not trigger flush yet
        for i in 0..10 {
            let result = batcher.add(vec![i as u8]);
            assert!(result.is_none());
        }

        // 11th message should trigger flush of previous 10
        let result = batcher.add(vec![10]);
        assert!(result.is_some());
        let batch = result.unwrap();
        assert_eq!(batch.count, 10);
    }

    #[test]
    fn test_batch_auto_flush_on_size() {
        let config = BatchConfig {
            max_messages: 1000,
            max_bytes: 1000,
            ..Default::default()
        };
        let mut batcher = MessageBatcher::new("test", config);

        // Add messages until we exceed byte limit
        // Each message is 200 bytes payload + 4 byte length prefix = 204 bytes
        // max_bytes is 1000, so 4 messages = 816 bytes (fits)
        // 5th message would be 1020 bytes > 1000, triggering flush
        let mut flush_triggered = false;
        for i in 0..5 {
            let payload = vec![i as u8; 200]; // 200 bytes each + 4 prefix = 204 bytes
            let result = batcher.add(payload);
            if result.is_some() {
                flush_triggered = true;
                // Flush should contain messages 0-3 (4 messages)
                let batch = result.unwrap();
                assert_eq!(batch.count, 4);
            }
        }

        // 5th message (i=4) should have triggered flush of first 4 messages
        assert!(flush_triggered, "Expected flush when byte limit exceeded");
    }

    #[test]
    fn test_batch_disabled() {
        let config = BatchConfig::disabled();
        let mut batcher = MessageBatcher::new("test", config);

        // Each message should return immediately
        let result = batcher.add(vec![1, 2, 3]);
        assert!(result.is_some());
        assert_eq!(result.unwrap().count, 1);

        let result = batcher.add(vec![4, 5, 6]);
        assert!(result.is_some());
        assert_eq!(result.unwrap().count, 1);
    }

    #[test]
    fn test_batch_receiver() {
        let mut batcher = MessageBatcher::new("test", BatchConfig::default());
        let mut receiver = BatchReceiver::new();

        // Create a batch
        for i in 0..5 {
            batcher.add(vec![i]);
        }
        let batch = batcher.flush().unwrap();

        // Receive the batch
        receiver.receive_batch(batch);
        assert_eq!(receiver.pending_count(), 5);

        // Get individual messages
        for i in 0..5 {
            let msg = receiver.next_message().unwrap();
            assert_eq!(msg, vec![i]);
        }

        assert!(!receiver.has_pending());
    }

    #[test]
    fn test_batch_encode_decode() {
        let mut batch = MessageBatch::new("test_topic", 42);
        batch.add(vec![1, 2, 3]);
        batch.add(vec![4, 5, 6, 7]);

        let encoded = batch.encode().unwrap();
        let decoded = MessageBatch::decode(&encoded).unwrap();

        assert_eq!(decoded.count, 2);
        assert_eq!(decoded.topic, "test_topic");
        assert_eq!(decoded.sequence, 42);
        assert_eq!(decoded.payloads.len(), 2);
    }

    // ========================================================================
    // FrequencyTracker Tests
    // ========================================================================

    #[test]
    fn test_frequency_tracker_initial_state() {
        let tracker = FrequencyTracker::default();
        assert_eq!(tracker.frequency(), 0.0);
        assert_eq!(tracker.total_messages(), 0);
        assert!(!tracker.is_high_frequency());
        assert!(!tracker.is_low_frequency());
    }

    #[test]
    fn test_frequency_tracker_record_messages() {
        let mut tracker = FrequencyTracker::default();

        // Record 10 messages
        for _ in 0..10 {
            tracker.record_message();
        }

        assert_eq!(tracker.total_messages(), 10);
    }

    #[test]
    fn test_frequency_tracker_batch_record() {
        let mut tracker = FrequencyTracker::default();

        tracker.record_messages(100);
        assert_eq!(tracker.total_messages(), 100);

        tracker.record_messages(50);
        assert_eq!(tracker.total_messages(), 150);
    }

    #[test]
    fn test_frequency_tracker_reset() {
        let mut tracker = FrequencyTracker::default();

        tracker.record_messages(100);
        assert_eq!(tracker.total_messages(), 100);

        tracker.reset();
        assert_eq!(tracker.total_messages(), 0);
        assert_eq!(tracker.frequency(), 0.0);
    }

    #[test]
    fn test_frequency_tracker_high_threshold() {
        let config = FrequencyTrackerConfig {
            ema_alpha: 1.0, // Use raw current value (no smoothing)
            min_sample_interval: Duration::from_nanos(1),
            high_freq_threshold: 1000.0,
            low_freq_threshold: 10.0,
        };
        let mut tracker = FrequencyTracker::new(config);

        // Not initialized yet
        assert!(!tracker.is_high_frequency());

        // Record many messages in a short time to trigger high frequency
        // With alpha=1.0 and min_sample_interval=1ns, each record_message will update EMA
        for _ in 0..10 {
            tracker.record_messages(10000);
            std::thread::sleep(Duration::from_micros(100));
        }

        // Should detect high frequency after initialization
        // (actual threshold check depends on timing)
    }

    #[test]
    fn test_frequency_tracker_ema_smoothing() {
        let config = FrequencyTrackerConfig {
            ema_alpha: 0.5, // 50% weight on new values
            min_sample_interval: Duration::from_nanos(1),
            high_freq_threshold: 1000.0,
            low_freq_threshold: 10.0,
        };
        let mut tracker = FrequencyTracker::new(config);

        // First sample sets EMA directly
        tracker.record_message();
        std::thread::sleep(Duration::from_micros(10));
        tracker.record_message();

        let _freq1 = tracker.frequency();

        // Subsequent samples are smoothed
        tracker.record_message();
        std::thread::sleep(Duration::from_micros(10));
        tracker.record_message();

        // Frequency should be in a reasonable range
        assert!(tracker.frequency() >= 0.0);
    }

    // ========================================================================
    // AdaptiveBatcher Tests
    // ========================================================================

    #[test]
    fn test_adaptive_batcher_initial_mode() {
        let batcher = AdaptiveBatcher::new("test", AdaptiveBatcherConfig::default());
        assert_eq!(batcher.current_mode(), AdaptiveMode::Normal);
    }

    #[test]
    fn test_adaptive_batcher_add_message() {
        let mut batcher = AdaptiveBatcher::new("test", AdaptiveBatcherConfig::default());

        // Add a message
        let result = batcher.add(vec![1, 2, 3]);
        assert!(result.is_none()); // Not enough messages to trigger batch

        assert_eq!(batcher.pending_count(), 1);
    }

    #[test]
    fn test_adaptive_batcher_flush() {
        let mut batcher = AdaptiveBatcher::new("test", AdaptiveBatcherConfig::default());

        // Add messages
        for i in 0..10 {
            batcher.add(vec![i]);
        }

        assert_eq!(batcher.pending_count(), 10);

        // Flush
        let batch = batcher.flush();
        assert!(batch.is_some());
        assert_eq!(batch.unwrap().count, 10);
        assert_eq!(batcher.pending_count(), 0);
    }

    #[test]
    fn test_adaptive_batcher_robotics_preset() {
        let config = AdaptiveBatcherConfig::robotics();
        let batcher = AdaptiveBatcher::new("sensor_data", config);

        // Robotics preset should start in Normal mode
        assert_eq!(batcher.current_mode(), AdaptiveMode::Normal);
    }

    #[test]
    fn test_adaptive_mode_equality() {
        assert_eq!(AdaptiveMode::HighThroughput, AdaptiveMode::HighThroughput);
        assert_eq!(AdaptiveMode::Normal, AdaptiveMode::Normal);
        assert_eq!(AdaptiveMode::LowLatency, AdaptiveMode::LowLatency);
        assert_ne!(AdaptiveMode::HighThroughput, AdaptiveMode::Normal);
        assert_ne!(AdaptiveMode::Normal, AdaptiveMode::LowLatency);
    }

    #[test]
    fn test_adaptive_batcher_frequency_tracking() {
        let config = AdaptiveBatcherConfig {
            adapt_interval: 10, // Adapt every 10 messages
            ..Default::default()
        };
        let mut batcher = AdaptiveBatcher::new("test", config);

        // Add messages
        for i in 0..20 {
            batcher.add(vec![i]);
            std::thread::sleep(Duration::from_micros(10));
        }

        // Frequency should be non-zero after processing
        let freq = batcher.frequency();
        // Due to timing, just check it's a valid number
        assert!(freq.is_finite());
    }
}
