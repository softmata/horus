//! Connection Health Monitoring for HORUS P2P
//!
//! This module provides real-time health monitoring with proactive alerts
//! for connection issues. It builds on the ConnectionQuality metrics to
//! provide:
//!
//! - Heartbeat mechanism for proactive health checks
//! - RTT, jitter, and packet loss tracking
//! - Quality degradation detection with configurable thresholds
//! - Alert callbacks for connection issues
//! - Prometheus-compatible metrics export
//! - Dashboard-ready JSON output
//!
//! # Example
//!
//! ```ignore
//! use horus_core::communication::network::health_monitor::*;
//!
//! let config = HealthMonitorConfig::default();
//! let monitor = HealthMonitor::new(config);
//!
//! // Register alert callback
//! monitor.on_alert(|alert| {
//!     log::warn!("Connection alert: {:?}", alert);
//! });
//!
//! // Start monitoring
//! monitor.start();
//!
//! // Get current health status
//! let health = monitor.health_status();
//! println!("Quality: {} (score: {})", health.quality_description, health.quality_score);
//! ```

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

use super::reconnect::ConnectionQuality;

// ============================================================================
// Configuration
// ============================================================================

/// Configuration for the health monitor
#[derive(Debug, Clone)]
pub struct HealthMonitorConfig {
    /// Interval between heartbeat packets (default: 1s)
    pub heartbeat_interval: Duration,

    /// Timeout for heartbeat response (default: 5s)
    pub heartbeat_timeout: Duration,

    /// Number of consecutive failures before marking connection as dead
    pub max_consecutive_failures: u32,

    /// RTT threshold for "warning" alert (default: 100ms)
    pub rtt_warning_threshold: Duration,

    /// RTT threshold for "critical" alert (default: 500ms)
    pub rtt_critical_threshold: Duration,

    /// Packet loss threshold for "warning" alert (default: 2%)
    pub packet_loss_warning_threshold: f64,

    /// Packet loss threshold for "critical" alert (default: 10%)
    pub packet_loss_critical_threshold: f64,

    /// Jitter threshold for "warning" alert (default: 50ms)
    pub jitter_warning_threshold: Duration,

    /// Jitter threshold for "critical" alert (default: 200ms)
    pub jitter_critical_threshold: Duration,

    /// Minimum time between alerts of the same type (rate limiting)
    pub alert_cooldown: Duration,

    /// Enable automatic recovery detection
    pub detect_recovery: bool,

    /// Window size for calculating metrics (number of samples)
    pub metrics_window_size: usize,
}

impl Default for HealthMonitorConfig {
    fn default() -> Self {
        Self {
            heartbeat_interval: Duration::from_secs(1),
            heartbeat_timeout: Duration::from_secs(5),
            max_consecutive_failures: 5,
            rtt_warning_threshold: Duration::from_millis(100),
            rtt_critical_threshold: Duration::from_millis(500),
            packet_loss_warning_threshold: 2.0,
            packet_loss_critical_threshold: 10.0,
            jitter_warning_threshold: Duration::from_millis(50),
            jitter_critical_threshold: Duration::from_millis(200),
            alert_cooldown: Duration::from_secs(30),
            detect_recovery: true,
            metrics_window_size: 100,
        }
    }
}

impl HealthMonitorConfig {
    /// Create a config optimized for low-latency robotics
    pub fn low_latency() -> Self {
        Self {
            heartbeat_interval: Duration::from_millis(100),
            heartbeat_timeout: Duration::from_millis(500),
            max_consecutive_failures: 3,
            rtt_warning_threshold: Duration::from_millis(10),
            rtt_critical_threshold: Duration::from_millis(50),
            packet_loss_warning_threshold: 0.5,
            packet_loss_critical_threshold: 2.0,
            jitter_warning_threshold: Duration::from_millis(5),
            jitter_critical_threshold: Duration::from_millis(20),
            alert_cooldown: Duration::from_secs(5),
            detect_recovery: true,
            metrics_window_size: 50,
        }
    }

    /// Create a config for WAN connections (more tolerant of latency)
    pub fn wan_tolerant() -> Self {
        Self {
            heartbeat_interval: Duration::from_secs(5),
            heartbeat_timeout: Duration::from_secs(30),
            max_consecutive_failures: 10,
            rtt_warning_threshold: Duration::from_millis(300),
            rtt_critical_threshold: Duration::from_secs(1),
            packet_loss_warning_threshold: 5.0,
            packet_loss_critical_threshold: 20.0,
            jitter_warning_threshold: Duration::from_millis(100),
            jitter_critical_threshold: Duration::from_millis(500),
            alert_cooldown: Duration::from_secs(60),
            detect_recovery: true,
            metrics_window_size: 200,
        }
    }
}

// ============================================================================
// Health Status Types
// ============================================================================

/// Overall health level
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HealthLevel {
    /// Everything is working well
    Healthy,
    /// Some degradation detected
    Degraded,
    /// Significant issues detected
    Warning,
    /// Critical issues, connection may fail soon
    Critical,
    /// Connection is dead
    Dead,
}

impl HealthLevel {
    /// Get a color code for display (ANSI)
    pub fn color_code(&self) -> &'static str {
        match self {
            HealthLevel::Healthy => "\x1b[32m",  // Green
            HealthLevel::Degraded => "\x1b[33m", // Yellow
            HealthLevel::Warning => "\x1b[33m",  // Yellow
            HealthLevel::Critical => "\x1b[31m", // Red
            HealthLevel::Dead => "\x1b[91m",     // Bright red
        }
    }

    /// Get a human-readable description
    pub fn description(&self) -> &'static str {
        match self {
            HealthLevel::Healthy => "Healthy",
            HealthLevel::Degraded => "Degraded",
            HealthLevel::Warning => "Warning",
            HealthLevel::Critical => "Critical",
            HealthLevel::Dead => "Dead",
        }
    }
}

/// Current health status snapshot
#[derive(Debug, Clone)]
pub struct HealthStatus {
    /// Overall health level
    pub level: HealthLevel,

    /// Quality score (0-100)
    pub quality_score: u32,

    /// Quality description
    pub quality_description: String,

    /// Current RTT
    pub rtt: Duration,

    /// RTT variance (jitter)
    pub jitter: Duration,

    /// Packet loss percentage
    pub packet_loss: f64,

    /// Packets sent since start
    pub packets_sent: u64,

    /// Packets received since start
    pub packets_received: u64,

    /// Packets lost since start
    pub packets_lost: u64,

    /// Bytes sent since start
    pub bytes_sent: u64,

    /// Bytes received since start
    pub bytes_received: u64,

    /// Time since last successful heartbeat
    pub time_since_last_heartbeat: Duration,

    /// Consecutive heartbeat failures
    pub consecutive_failures: u32,

    /// Number of active alerts
    pub active_alerts: usize,

    /// Uptime since monitor started
    pub uptime: Duration,

    /// Timestamp of this snapshot
    pub timestamp: Instant,
}

impl HealthStatus {
    /// Convert to JSON for dashboard display
    pub fn to_json(&self) -> String {
        format!(
            r#"{{
  "level": "{}",
  "quality_score": {},
  "quality_description": "{}",
  "rtt_ms": {:.2},
  "jitter_ms": {:.2},
  "packet_loss_percent": {:.2},
  "packets_sent": {},
  "packets_received": {},
  "packets_lost": {},
  "bytes_sent": {},
  "bytes_received": {},
  "time_since_heartbeat_ms": {},
  "consecutive_failures": {},
  "active_alerts": {},
  "uptime_secs": {}
}}"#,
            self.level.description(),
            self.quality_score,
            self.quality_description,
            self.rtt.as_secs_f64() * 1000.0,
            self.jitter.as_secs_f64() * 1000.0,
            self.packet_loss,
            self.packets_sent,
            self.packets_received,
            self.packets_lost,
            self.bytes_sent,
            self.bytes_received,
            self.time_since_last_heartbeat.as_millis(),
            self.consecutive_failures,
            self.active_alerts,
            self.uptime.as_secs()
        )
    }

    /// Convert to Prometheus-compatible metrics
    pub fn to_prometheus(&self, prefix: &str, labels: &str) -> String {
        format!(
            r#"# HELP {prefix}_health_level Connection health level (0=healthy, 1=degraded, 2=warning, 3=critical, 4=dead)
# TYPE {prefix}_health_level gauge
{prefix}_health_level{{{labels}}} {}

# HELP {prefix}_quality_score Connection quality score (0-100)
# TYPE {prefix}_quality_score gauge
{prefix}_quality_score{{{labels}}} {}

# HELP {prefix}_rtt_seconds Round-trip time in seconds
# TYPE {prefix}_rtt_seconds gauge
{prefix}_rtt_seconds{{{labels}}} {}

# HELP {prefix}_jitter_seconds RTT variance (jitter) in seconds
# TYPE {prefix}_jitter_seconds gauge
{prefix}_jitter_seconds{{{labels}}} {}

# HELP {prefix}_packet_loss_ratio Packet loss ratio (0-1)
# TYPE {prefix}_packet_loss_ratio gauge
{prefix}_packet_loss_ratio{{{labels}}} {}

# HELP {prefix}_packets_sent_total Total packets sent
# TYPE {prefix}_packets_sent_total counter
{prefix}_packets_sent_total{{{labels}}} {}

# HELP {prefix}_packets_received_total Total packets received
# TYPE {prefix}_packets_received_total counter
{prefix}_packets_received_total{{{labels}}} {}

# HELP {prefix}_packets_lost_total Total packets lost
# TYPE {prefix}_packets_lost_total counter
{prefix}_packets_lost_total{{{labels}}} {}

# HELP {prefix}_bytes_sent_total Total bytes sent
# TYPE {prefix}_bytes_sent_total counter
{prefix}_bytes_sent_total{{{labels}}} {}

# HELP {prefix}_bytes_received_total Total bytes received
# TYPE {prefix}_bytes_received_total counter
{prefix}_bytes_received_total{{{labels}}} {}

# HELP {prefix}_consecutive_failures Current consecutive heartbeat failures
# TYPE {prefix}_consecutive_failures gauge
{prefix}_consecutive_failures{{{labels}}} {}

# HELP {prefix}_uptime_seconds Monitor uptime in seconds
# TYPE {prefix}_uptime_seconds gauge
{prefix}_uptime_seconds{{{labels}}} {}
"#,
            match self.level {
                HealthLevel::Healthy => 0,
                HealthLevel::Degraded => 1,
                HealthLevel::Warning => 2,
                HealthLevel::Critical => 3,
                HealthLevel::Dead => 4,
            },
            self.quality_score,
            self.rtt.as_secs_f64(),
            self.jitter.as_secs_f64(),
            self.packet_loss / 100.0,
            self.packets_sent,
            self.packets_received,
            self.packets_lost,
            self.bytes_sent,
            self.bytes_received,
            self.consecutive_failures,
            self.uptime.as_secs(),
            prefix = prefix,
            labels = labels
        )
    }
}

// ============================================================================
// Alerts
// ============================================================================

/// Type of health alert
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AlertType {
    /// RTT exceeded warning threshold
    HighLatency,
    /// RTT exceeded critical threshold
    CriticalLatency,
    /// Jitter exceeded warning threshold
    HighJitter,
    /// Jitter exceeded critical threshold
    CriticalJitter,
    /// Packet loss exceeded warning threshold
    PacketLoss,
    /// Packet loss exceeded critical threshold
    CriticalPacketLoss,
    /// Heartbeat timeout
    HeartbeatTimeout,
    /// Multiple consecutive heartbeat failures
    ConsecutiveFailures,
    /// Connection is considered dead
    ConnectionDead,
    /// Connection has recovered from degraded state
    Recovered,
}

impl AlertType {
    /// Get severity level (0 = info, 1 = warning, 2 = error, 3 = critical)
    pub fn severity(&self) -> u8 {
        match self {
            AlertType::Recovered => 0,
            AlertType::HighLatency | AlertType::HighJitter | AlertType::PacketLoss => 1,
            AlertType::HeartbeatTimeout | AlertType::ConsecutiveFailures => 2,
            AlertType::CriticalLatency
            | AlertType::CriticalJitter
            | AlertType::CriticalPacketLoss
            | AlertType::ConnectionDead => 3,
        }
    }
}

/// Health alert
#[derive(Debug, Clone)]
pub struct HealthAlert {
    /// Type of alert
    pub alert_type: AlertType,

    /// When the alert was triggered
    pub timestamp: Instant,

    /// Current value that triggered the alert
    pub current_value: String,

    /// Threshold that was exceeded
    pub threshold: String,

    /// Human-readable message
    pub message: String,

    /// Whether this alert is still active
    pub active: bool,
}

impl HealthAlert {
    /// Convert to JSON
    pub fn to_json(&self) -> String {
        format!(
            r#"{{
  "type": "{:?}",
  "severity": {},
  "current_value": "{}",
  "threshold": "{}",
  "message": "{}",
  "active": {}
}}"#,
            self.alert_type,
            self.alert_type.severity(),
            self.current_value,
            self.threshold,
            self.message,
            self.active
        )
    }
}

/// Alert callback type
pub type AlertCallback = Box<dyn Fn(&HealthAlert) + Send + Sync>;

// ============================================================================
// Heartbeat Types
// ============================================================================

/// Heartbeat packet (sent to peer)
#[derive(Debug, Clone)]
pub struct HeartbeatRequest {
    /// Sequence number for matching responses
    pub sequence: u64,
    /// Timestamp when sent
    pub sent_at: Instant,
    /// Payload (can be used for additional data)
    pub payload: Vec<u8>,
}

impl HeartbeatRequest {
    /// Create a new heartbeat request
    pub fn new(sequence: u64) -> Self {
        Self {
            sequence,
            sent_at: Instant::now(),
            payload: Vec::new(),
        }
    }

    /// Serialize to bytes for transmission
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(17 + self.payload.len());
        buf.push(0x48); // 'H' for heartbeat
        buf.extend_from_slice(&self.sequence.to_be_bytes());
        buf.extend_from_slice(&(self.payload.len() as u64).to_be_bytes());
        buf.extend_from_slice(&self.payload);
        buf
    }

    /// Deserialize from bytes
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 17 || data[0] != 0x48 {
            return None;
        }
        let sequence = u64::from_be_bytes(data[1..9].try_into().ok()?);
        let payload_len = u64::from_be_bytes(data[9..17].try_into().ok()?) as usize;
        if data.len() < 17 + payload_len {
            return None;
        }
        Some(Self {
            sequence,
            sent_at: Instant::now(), // Will be overwritten by caller
            payload: data[17..17 + payload_len].to_vec(),
        })
    }
}

/// Heartbeat response (echoed back by peer)
#[derive(Debug, Clone)]
pub struct HeartbeatResponse {
    /// Sequence number (echoed from request)
    pub sequence: u64,
    /// Timestamp when received
    pub received_at: Instant,
}

impl HeartbeatResponse {
    /// Create a response for a request
    pub fn for_request(request: &HeartbeatRequest) -> Self {
        Self {
            sequence: request.sequence,
            received_at: Instant::now(),
        }
    }

    /// Serialize to bytes
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(9);
        buf.push(0x68); // 'h' for heartbeat response
        buf.extend_from_slice(&self.sequence.to_be_bytes());
        buf
    }

    /// Deserialize from bytes
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 9 || data[0] != 0x68 {
            return None;
        }
        let sequence = u64::from_be_bytes(data[1..9].try_into().ok()?);
        Some(Self {
            sequence,
            received_at: Instant::now(),
        })
    }
}

// ============================================================================
// Health Monitor
// ============================================================================

/// Health monitor for P2P connections
pub struct HealthMonitor {
    /// Configuration
    config: HealthMonitorConfig,

    /// Connection quality metrics
    quality: Arc<ConnectionQuality>,

    /// Whether monitoring is active
    running: Arc<AtomicBool>,

    /// Current health level
    health_level: RwLock<HealthLevel>,

    /// Next heartbeat sequence number
    next_sequence: AtomicU64,

    /// Pending heartbeats (sequence -> sent_at)
    pending_heartbeats: RwLock<HashMap<u64, Instant>>,

    /// Consecutive heartbeat failures
    consecutive_failures: AtomicU64,

    /// Last successful heartbeat time
    last_successful_heartbeat: RwLock<Option<Instant>>,

    /// Active alerts
    active_alerts: RwLock<Vec<HealthAlert>>,

    /// Last alert time per type (for rate limiting)
    last_alert_times: RwLock<HashMap<AlertType, Instant>>,

    /// Alert callbacks
    alert_callbacks: RwLock<Vec<AlertCallback>>,

    /// Monitor start time
    started_at: RwLock<Option<Instant>>,
}

impl HealthMonitor {
    /// Create a new health monitor
    pub fn new(config: HealthMonitorConfig) -> Self {
        Self {
            config,
            quality: Arc::new(ConnectionQuality::new()),
            running: Arc::new(AtomicBool::new(false)),
            health_level: RwLock::new(HealthLevel::Healthy),
            next_sequence: AtomicU64::new(1),
            pending_heartbeats: RwLock::new(HashMap::new()),
            consecutive_failures: AtomicU64::new(0),
            last_successful_heartbeat: RwLock::new(None),
            active_alerts: RwLock::new(Vec::new()),
            last_alert_times: RwLock::new(HashMap::new()),
            alert_callbacks: RwLock::new(Vec::new()),
            started_at: RwLock::new(None),
        }
    }

    /// Create with default configuration
    pub fn default_monitor() -> Self {
        Self::new(HealthMonitorConfig::default())
    }

    /// Get the underlying quality metrics
    pub fn quality(&self) -> Arc<ConnectionQuality> {
        Arc::clone(&self.quality)
    }

    /// Register an alert callback
    pub fn on_alert(&self, callback: AlertCallback) {
        self.alert_callbacks.write().unwrap().push(callback);
    }

    /// Start monitoring
    pub fn start(&self) {
        self.running.store(true, Ordering::SeqCst);
        *self.started_at.write().unwrap() = Some(Instant::now());
        log::info!("Health monitor started");
    }

    /// Stop monitoring
    pub fn stop(&self) {
        self.running.store(false, Ordering::SeqCst);
        log::info!("Health monitor stopped");
    }

    /// Check if monitor is running
    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::SeqCst)
    }

    /// Create a heartbeat request to send to peer
    pub fn create_heartbeat(&self) -> HeartbeatRequest {
        let sequence = self.next_sequence.fetch_add(1, Ordering::SeqCst);
        let request = HeartbeatRequest::new(sequence);

        // Track pending heartbeat
        self.pending_heartbeats
            .write()
            .unwrap()
            .insert(sequence, request.sent_at);

        // Record as sent
        self.quality.record_send(request.to_bytes().len() as u64);

        request
    }

    /// Process a heartbeat response from peer
    pub fn process_heartbeat_response(&self, response: HeartbeatResponse) -> Option<Duration> {
        // Find the matching request
        let sent_at = self
            .pending_heartbeats
            .write()
            .unwrap()
            .remove(&response.sequence)?;

        // Calculate RTT
        let rtt = response.received_at.duration_since(sent_at);

        // Update metrics
        self.quality.update_rtt(rtt);
        self.quality.record_receive(9); // Response is 9 bytes

        // Reset failure counter
        self.consecutive_failures.store(0, Ordering::Relaxed);
        *self.last_successful_heartbeat.write().unwrap() = Some(Instant::now());

        // Check for recovery
        self.check_recovery();

        // Check thresholds
        self.check_thresholds();

        Some(rtt)
    }

    /// Handle a heartbeat timeout
    pub fn handle_heartbeat_timeout(&self, sequence: u64) {
        if self
            .pending_heartbeats
            .write()
            .unwrap()
            .remove(&sequence)
            .is_some()
        {
            let failures = self.consecutive_failures.fetch_add(1, Ordering::Relaxed) + 1;
            self.quality.record_loss();

            // Fire timeout alert
            self.fire_alert(AlertType::HeartbeatTimeout, "1", "0", "Heartbeat timeout");

            // Check for consecutive failures
            if failures >= self.config.max_consecutive_failures as u64 {
                self.fire_alert(
                    AlertType::ConsecutiveFailures,
                    &failures.to_string(),
                    &self.config.max_consecutive_failures.to_string(),
                    &format!("{} consecutive heartbeat failures", failures),
                );

                // Mark connection as dead
                *self.health_level.write().unwrap() = HealthLevel::Dead;
                self.fire_alert(
                    AlertType::ConnectionDead,
                    &failures.to_string(),
                    &self.config.max_consecutive_failures.to_string(),
                    "Connection is dead",
                );
            }

            self.update_health_level();
        }
    }

    /// Clean up expired pending heartbeats
    pub fn cleanup_expired_heartbeats(&self) {
        let timeout = self.config.heartbeat_timeout;
        let now = Instant::now();

        let expired: Vec<u64> = self
            .pending_heartbeats
            .read()
            .unwrap()
            .iter()
            .filter(|(_, sent_at)| now.duration_since(**sent_at) > timeout)
            .map(|(seq, _)| *seq)
            .collect();

        for seq in expired {
            self.handle_heartbeat_timeout(seq);
        }
    }

    /// Get current health status
    pub fn health_status(&self) -> HealthStatus {
        let quality = &self.quality;
        let active_alerts = self.active_alerts.read().unwrap();
        let last_hb = self.last_successful_heartbeat.read().unwrap();
        let started = self.started_at.read().unwrap();

        HealthStatus {
            level: *self.health_level.read().unwrap(),
            quality_score: quality.quality_score(),
            quality_description: quality.quality_description().to_string(),
            rtt: quality.rtt(),
            jitter: quality.rtt_variance(),
            packet_loss: quality.packet_loss(),
            packets_sent: quality.packets_sent.load(Ordering::Relaxed),
            packets_received: quality.packets_received.load(Ordering::Relaxed),
            packets_lost: quality.packets_lost.load(Ordering::Relaxed),
            bytes_sent: quality.bytes_sent.load(Ordering::Relaxed),
            bytes_received: quality.bytes_received.load(Ordering::Relaxed),
            time_since_last_heartbeat: last_hb.map(|t| t.elapsed()).unwrap_or(Duration::MAX),
            consecutive_failures: self.consecutive_failures.load(Ordering::Relaxed) as u32,
            active_alerts: active_alerts.iter().filter(|a| a.active).count(),
            uptime: started.map(|t| t.elapsed()).unwrap_or_default(),
            timestamp: Instant::now(),
        }
    }

    /// Get active alerts
    pub fn active_alerts(&self) -> Vec<HealthAlert> {
        self.active_alerts
            .read()
            .unwrap()
            .iter()
            .filter(|a| a.active)
            .cloned()
            .collect()
    }

    /// Get all alerts (including inactive)
    pub fn all_alerts(&self) -> Vec<HealthAlert> {
        self.active_alerts.read().unwrap().clone()
    }

    /// Clear resolved alerts
    pub fn clear_resolved_alerts(&self) {
        self.active_alerts.write().unwrap().retain(|a| a.active);
    }

    /// Export metrics in Prometheus format
    pub fn prometheus_metrics(&self, prefix: &str, labels: &str) -> String {
        self.health_status().to_prometheus(prefix, labels)
    }

    /// Export status as JSON
    pub fn json_status(&self) -> String {
        self.health_status().to_json()
    }

    // ========================================================================
    // Internal Methods
    // ========================================================================

    fn check_thresholds(&self) {
        let quality = &self.quality;
        let rtt = quality.rtt();
        let jitter = quality.rtt_variance();
        let loss = quality.packet_loss();

        // RTT thresholds
        if rtt >= self.config.rtt_critical_threshold {
            self.fire_alert(
                AlertType::CriticalLatency,
                &format!("{:.1}ms", rtt.as_secs_f64() * 1000.0),
                &format!(
                    "{:.1}ms",
                    self.config.rtt_critical_threshold.as_secs_f64() * 1000.0
                ),
                "Critical latency detected",
            );
        } else if rtt >= self.config.rtt_warning_threshold {
            self.fire_alert(
                AlertType::HighLatency,
                &format!("{:.1}ms", rtt.as_secs_f64() * 1000.0),
                &format!(
                    "{:.1}ms",
                    self.config.rtt_warning_threshold.as_secs_f64() * 1000.0
                ),
                "High latency detected",
            );
        }

        // Jitter thresholds
        if jitter >= self.config.jitter_critical_threshold {
            self.fire_alert(
                AlertType::CriticalJitter,
                &format!("{:.1}ms", jitter.as_secs_f64() * 1000.0),
                &format!(
                    "{:.1}ms",
                    self.config.jitter_critical_threshold.as_secs_f64() * 1000.0
                ),
                "Critical jitter detected",
            );
        } else if jitter >= self.config.jitter_warning_threshold {
            self.fire_alert(
                AlertType::HighJitter,
                &format!("{:.1}ms", jitter.as_secs_f64() * 1000.0),
                &format!(
                    "{:.1}ms",
                    self.config.jitter_warning_threshold.as_secs_f64() * 1000.0
                ),
                "High jitter detected",
            );
        }

        // Packet loss thresholds
        if loss >= self.config.packet_loss_critical_threshold {
            self.fire_alert(
                AlertType::CriticalPacketLoss,
                &format!("{:.1}%", loss),
                &format!("{:.1}%", self.config.packet_loss_critical_threshold),
                "Critical packet loss detected",
            );
        } else if loss >= self.config.packet_loss_warning_threshold {
            self.fire_alert(
                AlertType::PacketLoss,
                &format!("{:.1}%", loss),
                &format!("{:.1}%", self.config.packet_loss_warning_threshold),
                "Packet loss detected",
            );
        }

        self.update_health_level();
    }

    fn check_recovery(&self) {
        if !self.config.detect_recovery {
            return;
        }

        let current_level = *self.health_level.read().unwrap();
        if current_level != HealthLevel::Healthy {
            let quality = &self.quality;
            let rtt = quality.rtt();
            let jitter = quality.rtt_variance();
            let loss = quality.packet_loss();

            // Check if all metrics are back to healthy
            if rtt < self.config.rtt_warning_threshold
                && jitter < self.config.jitter_warning_threshold
                && loss < self.config.packet_loss_warning_threshold
            {
                self.fire_alert(
                    AlertType::Recovered,
                    &format!("score: {}", quality.quality_score()),
                    "healthy",
                    "Connection has recovered",
                );

                // Mark inactive all current alerts
                let mut alerts = self.active_alerts.write().unwrap();
                for alert in alerts.iter_mut() {
                    if alert.alert_type != AlertType::Recovered {
                        alert.active = false;
                    }
                }
            }
        }
    }

    fn update_health_level(&self) {
        let quality = &self.quality;
        let rtt = quality.rtt();
        let jitter = quality.rtt_variance();
        let loss = quality.packet_loss();
        let failures = self.consecutive_failures.load(Ordering::Relaxed);

        let new_level = if failures >= self.config.max_consecutive_failures as u64 {
            HealthLevel::Dead
        } else if rtt >= self.config.rtt_critical_threshold
            || jitter >= self.config.jitter_critical_threshold
            || loss >= self.config.packet_loss_critical_threshold
        {
            HealthLevel::Critical
        } else if failures > 0
            || rtt >= self.config.rtt_warning_threshold
            || jitter >= self.config.jitter_warning_threshold
            || loss >= self.config.packet_loss_warning_threshold
        {
            HealthLevel::Warning
        } else if quality.quality_score() < 70 {
            HealthLevel::Degraded
        } else {
            HealthLevel::Healthy
        };

        *self.health_level.write().unwrap() = new_level;
    }

    fn fire_alert(&self, alert_type: AlertType, current: &str, threshold: &str, message: &str) {
        // Rate limit alerts
        let now = Instant::now();
        {
            let last_times = self.last_alert_times.read().unwrap();
            if let Some(last) = last_times.get(&alert_type) {
                if now.duration_since(*last) < self.config.alert_cooldown {
                    return; // Skip due to cooldown
                }
            }
        }

        // Update last alert time
        self.last_alert_times
            .write()
            .unwrap()
            .insert(alert_type, now);

        let alert = HealthAlert {
            alert_type,
            timestamp: now,
            current_value: current.to_string(),
            threshold: threshold.to_string(),
            message: message.to_string(),
            active: true,
        };

        // Add to active alerts
        self.active_alerts.write().unwrap().push(alert.clone());

        // Fire callbacks
        let callbacks = self.alert_callbacks.read().unwrap();
        for callback in callbacks.iter() {
            callback(&alert);
        }

        // Log the alert
        match alert_type.severity() {
            0 => log::info!("Health alert: {}", message),
            1 => log::warn!("Health alert: {}", message),
            2 => log::error!("Health alert: {}", message),
            _ => log::error!("CRITICAL health alert: {}", message),
        }
    }
}

impl Default for HealthMonitor {
    fn default() -> Self {
        Self::new(HealthMonitorConfig::default())
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_heartbeat_serialization() {
        let request = HeartbeatRequest::new(12345);
        let bytes = request.to_bytes();
        let decoded = HeartbeatRequest::from_bytes(&bytes).unwrap();
        assert_eq!(decoded.sequence, 12345);
    }

    #[test]
    fn test_heartbeat_response_serialization() {
        let request = HeartbeatRequest::new(54321);
        let response = HeartbeatResponse::for_request(&request);
        let bytes = response.to_bytes();
        let decoded = HeartbeatResponse::from_bytes(&bytes).unwrap();
        assert_eq!(decoded.sequence, 54321);
    }

    #[test]
    fn test_health_monitor_creation() {
        let monitor = HealthMonitor::default_monitor();
        assert!(!monitor.is_running());
        monitor.start();
        assert!(monitor.is_running());
        monitor.stop();
        assert!(!monitor.is_running());
    }

    #[test]
    fn test_heartbeat_round_trip() {
        // Use generous thresholds so the test is stable under CPU contention
        // (thread::sleep can overshoot significantly during parallel test execution)
        let config = HealthMonitorConfig {
            rtt_warning_threshold: Duration::from_secs(5),
            rtt_critical_threshold: Duration::from_secs(10),
            jitter_warning_threshold: Duration::from_secs(5),
            jitter_critical_threshold: Duration::from_secs(10),
            ..Default::default()
        };
        let monitor = HealthMonitor::new(config);
        monitor.start();

        // Create heartbeat
        let request = monitor.create_heartbeat();
        assert_eq!(request.sequence, 1);

        // Simulate response after small delay
        std::thread::sleep(Duration::from_millis(5));
        let response = HeartbeatResponse::for_request(&request);

        // Process response
        let rtt = monitor.process_heartbeat_response(response);
        assert!(rtt.is_some());
        assert!(rtt.unwrap() >= Duration::from_millis(5));

        // Check health status
        let status = monitor.health_status();
        assert_eq!(status.level, HealthLevel::Healthy);
        assert_eq!(status.consecutive_failures, 0);
    }

    #[test]
    fn test_heartbeat_timeout() {
        let config = HealthMonitorConfig {
            max_consecutive_failures: 3,
            ..Default::default()
        };
        let monitor = HealthMonitor::new(config);
        monitor.start();

        // Simulate timeouts
        for i in 1..=3 {
            let request = monitor.create_heartbeat();
            monitor.handle_heartbeat_timeout(request.sequence);
            assert_eq!(
                monitor.consecutive_failures.load(Ordering::Relaxed),
                i as u64
            );
        }

        // Should be marked as dead
        let status = monitor.health_status();
        assert_eq!(status.level, HealthLevel::Dead);
    }

    #[test]
    fn test_prometheus_export() {
        let monitor = HealthMonitor::default_monitor();
        monitor.start();

        let metrics = monitor.prometheus_metrics("horus_p2p", "peer=\"test\"");
        assert!(metrics.contains("horus_p2p_health_level"));
        assert!(metrics.contains("horus_p2p_quality_score"));
        assert!(metrics.contains("horus_p2p_rtt_seconds"));
    }

    #[test]
    fn test_json_export() {
        let monitor = HealthMonitor::default_monitor();
        monitor.start();

        let json = monitor.json_status();
        assert!(json.contains("\"level\": \"Healthy\""));
        assert!(json.contains("\"quality_score\":"));
    }

    #[test]
    fn test_alert_callback() {
        use std::sync::atomic::AtomicUsize;

        let alert_count = Arc::new(AtomicUsize::new(0));
        let alert_count_clone = Arc::clone(&alert_count);

        let config = HealthMonitorConfig {
            rtt_warning_threshold: Duration::from_millis(10),
            alert_cooldown: Duration::from_millis(1), // Short cooldown for test
            ..Default::default()
        };
        let monitor = HealthMonitor::new(config);

        monitor.on_alert(Box::new(move |_alert| {
            alert_count_clone.fetch_add(1, Ordering::SeqCst);
        }));

        monitor.start();

        // Simulate high RTT that should trigger alert
        monitor.quality.update_rtt(Duration::from_millis(50));
        monitor.check_thresholds();

        // Small delay to allow callback
        std::thread::sleep(Duration::from_millis(10));

        assert!(alert_count.load(Ordering::SeqCst) >= 1);
    }
}
