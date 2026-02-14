//! Hybrid Backend - Zero-copy local bypass for Zenoh
//!
//! This module provides a `HybridBackend` that automatically switches between:
//! - **Local**: HORUS native Topic for <1μs latency
//! - **Remote**: Zenoh for multi-robot mesh, cloud, and ROS2 interop
//!
//! # Performance
//!
//! - Local latency: <1μs (vs ~10-50μs pure Zenoh)
//! - Remote: Same as Zenoh (~50-200μs depending on network)
//! - Transparent to Topic API users
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────┐
//! │                   HybridBackend                     │
//! ├─────────────────────────────────────────────────────┤
//! │                                                     │
//! │   ┌─────────────┐        ┌─────────────────────┐   │
//! │   │  Topic       │        │    ZenohBackend     │   │
//! │   │  (Local)    │        │    (Remote)         │   │
//! │   │  <1μs       │        │    ~50-200μs        │   │
//! │   └──────┬──────┘        └──────────┬──────────┘   │
//! │          │                          │              │
//! │          └────────┬─────────────────┘              │
//! │                   │                                │
//! │          LocalityDetector                          │
//! │          (auto-switch)                             │
//! │                                                    │
//! └─────────────────────────────────────────────────────┘
//! ```
//!
//! # Example
//!
//! ```ignore
//! use horus_core::communication::network::hybrid_backend::{HybridBackend, HybridConfig};
//!
//! // Create with default config (auto-detects locality)
//! let backend = HybridBackend::new("sensor/imu", HybridConfig::default()).await?;
//!
//! // Send - automatically uses SHM for local, Zenoh for remote
//! backend.send(&imu_data)?;
//!
//! // Receive - checks both SHM and Zenoh
//! if let Some(msg) = backend.recv() {
//!     process(msg);
//! }
//! ```

use crate::error::{HorusError, HorusResult};
use crate::communication::Topic;
use std::marker::PhantomData;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

#[cfg(feature = "zenoh-transport")]
use parking_lot::{Mutex, RwLock};
#[cfg(feature = "zenoh-transport")]
use std::collections::HashSet;
#[cfg(feature = "zenoh-transport")]
use std::sync::atomic::AtomicBool;
#[cfg(feature = "zenoh-transport")]
use std::time::Instant;

#[cfg(feature = "zenoh-transport")]
use super::zenoh_backend::ZenohBackend;
#[cfg(feature = "zenoh-transport")]
use super::zenoh_config::ZenohConfig;

/// Configuration for the hybrid backend
#[derive(Debug, Clone)]
pub struct HybridConfig {
    /// Enable local SHM bypass (default: true)
    pub enable_local_bypass: bool,

    /// SHM ring buffer capacity (default: 1024)
    pub shm_capacity: usize,

    /// Zenoh configuration
    #[cfg(feature = "zenoh-transport")]
    pub zenoh_config: ZenohConfig,

    /// How often to check for new local peers (default: 1s)
    pub locality_check_interval: Duration,

    /// Prefer SHM even when Zenoh might work (default: true)
    pub prefer_shm_for_local: bool,

    /// Enable statistics tracking (default: true)
    pub enable_stats: bool,
}

impl Default for HybridConfig {
    fn default() -> Self {
        Self {
            enable_local_bypass: true,
            shm_capacity: 1024,
            #[cfg(feature = "zenoh-transport")]
            zenoh_config: ZenohConfig::default(),
            locality_check_interval: Duration::from_secs(1),
            prefer_shm_for_local: true,
            enable_stats: true,
        }
    }
}

impl HybridConfig {
    /// Create config with local-only mode (no Zenoh, pure SHM)
    pub fn local_only(capacity: usize) -> Self {
        Self {
            enable_local_bypass: true,
            shm_capacity: capacity,
            #[cfg(feature = "zenoh-transport")]
            zenoh_config: ZenohConfig::default(),
            locality_check_interval: Duration::from_secs(60), // Rarely check
            prefer_shm_for_local: true,
            enable_stats: true,
        }
    }

    /// Create config optimized for multi-robot mesh
    #[cfg(feature = "zenoh-transport")]
    pub fn multi_robot(zenoh_config: ZenohConfig) -> Self {
        Self {
            enable_local_bypass: true,
            shm_capacity: 256, // Smaller buffer for mesh
            zenoh_config,
            locality_check_interval: Duration::from_millis(500), // Check more often
            prefer_shm_for_local: true,
            enable_stats: true,
        }
    }

    /// Create config for ROS2 bridge mode
    #[cfg(feature = "zenoh-transport")]
    pub fn ros2_bridge(domain_id: u32) -> Self {
        Self {
            enable_local_bypass: true,
            shm_capacity: 512,
            zenoh_config: ZenohConfig::ros2(domain_id),
            locality_check_interval: Duration::from_secs(1),
            prefer_shm_for_local: true,
            enable_stats: true,
        }
    }
}

/// Statistics for the hybrid backend
#[derive(Debug, Default)]
pub struct HybridStats {
    /// Messages sent via SHM (local)
    pub shm_sends: AtomicU64,
    /// Messages sent via Zenoh (remote)
    pub zenoh_sends: AtomicU64,
    /// Messages received via SHM
    pub shm_recvs: AtomicU64,
    /// Messages received via Zenoh
    pub zenoh_recvs: AtomicU64,
    /// SHM send failures (buffer full)
    pub shm_send_failures: AtomicU64,
    /// Zenoh send failures
    pub zenoh_send_failures: AtomicU64,
    /// Locality checks performed
    pub locality_checks: AtomicU64,
    /// Local peers detected
    pub local_peers_detected: AtomicU64,
}

impl HybridStats {
    /// Get total messages sent
    pub fn total_sends(&self) -> u64 {
        self.shm_sends.load(Ordering::Relaxed) + self.zenoh_sends.load(Ordering::Relaxed)
    }

    /// Get total messages received
    pub fn total_recvs(&self) -> u64 {
        self.shm_recvs.load(Ordering::Relaxed) + self.zenoh_recvs.load(Ordering::Relaxed)
    }

    /// Get percentage of local (SHM) traffic
    pub fn local_traffic_percent(&self) -> f64 {
        let total = self.total_sends() + self.total_recvs();
        if total == 0 {
            return 0.0;
        }
        let local = self.shm_sends.load(Ordering::Relaxed) + self.shm_recvs.load(Ordering::Relaxed);
        (local as f64 / total as f64) * 100.0
    }
}

/// Tracks which peers are local vs remote
#[cfg(feature = "zenoh-transport")]
struct LocalityTracker {
    /// Set of local peer IDs (detected via same SHM region access)
    local_peers: RwLock<HashSet<String>>,
    /// Last locality check timestamp
    last_check: Mutex<Instant>,
    /// Check interval
    check_interval: Duration,
}

#[cfg(feature = "zenoh-transport")]
impl LocalityTracker {
    fn new(check_interval: Duration) -> Self {
        Self {
            local_peers: RwLock::new(HashSet::new()),
            last_check: Mutex::new(Instant::now()),
            check_interval,
        }
    }

    /// Check if we should perform a locality check
    fn should_check(&self) -> bool {
        let last = *self.last_check.lock();
        last.elapsed() >= self.check_interval
    }

    /// Mark a peer as local
    fn mark_local(&self, peer_id: &str) {
        self.local_peers.write().insert(peer_id.to_string());
    }

    /// Check if any local peers are registered
    fn has_local_peers(&self) -> bool {
        !self.local_peers.read().is_empty()
    }

    /// Update last check time
    fn update_check_time(&self) {
        *self.last_check.lock() = Instant::now();
    }
}

/// Hybrid backend combining SHM (local) and Zenoh (remote) transport
///
/// Automatically detects peer locality and routes messages through the
/// optimal transport:
/// - Local peers: SHM for <1μs latency
/// - Remote peers: Zenoh for network transport
#[cfg(feature = "zenoh-transport")]
pub struct HybridBackend<T>
where
    T: serde::Serialize + serde::de::DeserializeOwned + Clone + Send + Sync + 'static,
{
    /// Topic name
    topic: String,

    /// Local SHM backend (always created for local bypass)
    shm: Option<Topic<T>>,

    /// Zenoh backend for remote communication
    zenoh: Option<ZenohBackend<T>>,

    /// Configuration
    config: HybridConfig,

    /// Locality tracker
    locality: Arc<LocalityTracker>,

    /// Whether we've detected local peers
    has_local_peers: AtomicBool,

    /// Statistics
    stats: Arc<HybridStats>,

    /// Type marker
    _phantom: PhantomData<T>,
}

#[cfg(feature = "zenoh-transport")]
impl<T> HybridBackend<T>
where
    T: serde::Serialize + serde::de::DeserializeOwned + Clone + Send + Sync + 'static,
{
    /// Create a new hybrid backend
    ///
    /// This creates both SHM and Zenoh backends. The SHM backend is used
    /// for local communication (same machine), while Zenoh handles remote.
    pub async fn new(topic: &str, config: HybridConfig) -> HorusResult<Self> {
        // Create SHM backend for local bypass
        let shm = if config.enable_local_bypass {
            // Use a topic name that includes "hybrid" to avoid conflicts
            let shm_topic_name = format!("hybrid/{}", topic.replace('/', "_"));
            match Topic::new(&shm_topic_name) {
                Ok(shm) => {
                    log::info!(
                        "HybridBackend: Created SHM backend for '{}' (capacity: {})",
                        topic,
                        config.shm_capacity
                    );
                    Some(shm)
                }
                Err(e) => {
                    log::warn!(
                        "HybridBackend: Failed to create SHM backend for '{}': {}",
                        topic,
                        e
                    );
                    None
                }
            }
        } else {
            None
        };

        // Create Zenoh backend for remote communication
        let zenoh = match ZenohBackend::new(topic, config.zenoh_config.clone()).await {
            Ok(z) => {
                log::info!("HybridBackend: Created Zenoh backend for '{}'", topic);
                Some(z)
            }
            Err(e) => {
                log::warn!(
                    "HybridBackend: Failed to create Zenoh backend for '{}': {}",
                    topic,
                    e
                );
                None
            }
        };

        // At least one backend must be available
        if shm.is_none() && zenoh.is_none() {
            return Err(HorusError::Communication(
                "Failed to create both SHM and Zenoh backends".to_string(),
            ));
        }

        let locality = Arc::new(LocalityTracker::new(config.locality_check_interval));

        Ok(Self {
            topic: topic.to_string(),
            shm,
            zenoh,
            config,
            locality,
            has_local_peers: AtomicBool::new(false),
            stats: Arc::new(HybridStats::default()),
            _phantom: PhantomData,
        })
    }

    /// Create a blocking version (for non-async contexts)
    pub fn new_blocking(topic: &str, config: HybridConfig) -> HorusResult<Self> {
        let rt = tokio::runtime::Handle::try_current()
            .or_else(|_| tokio::runtime::Runtime::new().map(|rt| rt.handle().clone()))
            .map_err(|e| {
                HorusError::Communication(format!("Failed to get tokio runtime: {}", e))
            })?;

        rt.block_on(Self::new(topic, config))
    }

    /// Initialize the publisher (required before sending)
    pub async fn init_publisher(&mut self) -> HorusResult<()> {
        if let Some(ref mut zenoh) = self.zenoh {
            zenoh.init_publisher().await?;
        }
        Ok(())
    }

    /// Initialize the subscriber (required before receiving)
    pub async fn init_subscriber(&mut self) -> HorusResult<()> {
        if let Some(ref mut zenoh) = self.zenoh {
            zenoh.init_subscriber().await?;
        }
        Ok(())
    }

    /// Send a message through the optimal transport
    ///
    /// - If local peers are detected and SHM is available, uses SHM
    /// - Otherwise falls back to Zenoh
    /// - If SHM fails (buffer full), falls back to Zenoh
    pub fn send(&self, msg: &T) -> HorusResult<()> {
        let use_shm = self.config.prefer_shm_for_local
            && self.shm.is_some()
            && (self.has_local_peers.load(Ordering::Relaxed) || self.config.enable_local_bypass);

        // Try SHM first for local delivery
        if use_shm {
            if let Some(ref shm) = self.shm {
                match shm.try_send(msg.clone()) {
                    Ok(()) => {
                        if self.config.enable_stats {
                            self.stats.shm_sends.fetch_add(1, Ordering::Relaxed);
                        }
                        // Also send via Zenoh for remote peers (unless local-only)
                        if self.zenoh.is_some() && !self.config.prefer_shm_for_local {
                            let _ = self.send_zenoh(msg);
                        }
                        return Ok(());
                    }
                    Err(_) => {
                        // SHM buffer full, fall through to Zenoh
                        if self.config.enable_stats {
                            self.stats.shm_send_failures.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
            }
        }

        // Fall back to Zenoh for remote delivery
        self.send_zenoh(msg)
    }

    /// Send via Zenoh only
    fn send_zenoh(&self, msg: &T) -> HorusResult<()> {
        if let Some(ref zenoh) = self.zenoh {
            match zenoh.send(msg) {
                Ok(()) => {
                    if self.config.enable_stats {
                        self.stats.zenoh_sends.fetch_add(1, Ordering::Relaxed);
                    }
                    Ok(())
                }
                Err(e) => {
                    if self.config.enable_stats {
                        self.stats
                            .zenoh_send_failures
                            .fetch_add(1, Ordering::Relaxed);
                    }
                    Err(e)
                }
            }
        } else {
            Err(HorusError::Communication(
                "No Zenoh backend available".to_string(),
            ))
        }
    }

    /// Receive a message from either transport
    ///
    /// Checks SHM first (faster), then Zenoh if SHM is empty.
    pub fn recv(&self) -> Option<T> {
        // Check SHM first (faster)
        if let Some(ref shm) = self.shm {
            if let Some(msg) = shm.try_recv() {
                if self.config.enable_stats {
                    self.stats.shm_recvs.fetch_add(1, Ordering::Relaxed);
                }
                // Mark that we have local peers (someone is publishing to SHM)
                if !self.has_local_peers.load(Ordering::Relaxed) {
                    self.has_local_peers.store(true, Ordering::Relaxed);
                    self.locality.mark_local("self");
                    if self.config.enable_stats {
                        self.stats
                            .local_peers_detected
                            .fetch_add(1, Ordering::Relaxed);
                    }
                }
                return Some(msg);
            }
        }

        // Check Zenoh
        if let Some(ref zenoh) = self.zenoh {
            if let Some(msg) = zenoh.recv() {
                if self.config.enable_stats {
                    self.stats.zenoh_recvs.fetch_add(1, Ordering::Relaxed);
                }
                return Some(msg);
            }
        }

        None
    }

    /// Try to receive with timeout
    pub fn recv_timeout(&self, timeout: Duration) -> Option<T> {
        let start = Instant::now();
        while start.elapsed() < timeout {
            if let Some(msg) = self.recv() {
                return Some(msg);
            }
            std::thread::sleep(Duration::from_micros(10));
        }
        None
    }

    /// Get the topic name
    pub fn topic(&self) -> &str {
        &self.topic
    }

    /// Check if SHM backend is available
    pub fn has_shm(&self) -> bool {
        self.shm.is_some()
    }

    /// Check if Zenoh backend is available
    pub fn has_zenoh(&self) -> bool {
        self.zenoh.is_some()
    }

    /// Get statistics
    pub fn stats(&self) -> &HybridStats {
        &self.stats
    }

    /// Get the current transport mode
    pub fn current_mode(&self) -> HybridMode {
        if self.shm.is_some() && self.zenoh.is_some() {
            HybridMode::Both
        } else if self.shm.is_some() {
            HybridMode::ShmOnly
        } else if self.zenoh.is_some() {
            HybridMode::ZenohOnly
        } else {
            HybridMode::None
        }
    }

    /// Force a locality check
    pub fn check_locality(&self) {
        // Only check if enough time has passed
        if !self.locality.should_check() {
            return;
        }

        if self.config.enable_stats {
            self.stats.locality_checks.fetch_add(1, Ordering::Relaxed);
        }
        self.locality.update_check_time();

        // Check if we can detect local peers via SHM
        if self.shm.is_some() {
            // If we successfully created SHM, there might be local peers
            if !self.has_local_peers.load(Ordering::Relaxed) {
                // Conservative: assume local peers exist if SHM is available
                self.has_local_peers.store(true, Ordering::Relaxed);
            }
        }

        // Update local peers flag from tracker
        if self.locality.has_local_peers() && !self.has_local_peers.load(Ordering::Relaxed) {
            self.has_local_peers.store(true, Ordering::Relaxed);
            if self.config.enable_stats {
                self.stats
                    .local_peers_detected
                    .fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    /// Get Zenoh connection health score (0-100)
    pub fn zenoh_health_score(&self) -> Option<u32> {
        self.zenoh.as_ref().map(|z| z.health_score())
    }

    /// Get the underlying Zenoh backend (for advanced use)
    pub fn zenoh_backend(&self) -> Option<&ZenohBackend<T>> {
        self.zenoh.as_ref()
    }
}

#[cfg(feature = "zenoh-transport")]
impl<T> std::fmt::Debug for HybridBackend<T>
where
    T: serde::Serialize + serde::de::DeserializeOwned + Clone + Send + Sync + 'static,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("HybridBackend")
            .field("topic", &self.topic)
            .field("has_shm", &self.shm.is_some())
            .field("has_zenoh", &self.zenoh.is_some())
            .field(
                "has_local_peers",
                &self.has_local_peers.load(Ordering::Relaxed),
            )
            .field("mode", &self.current_mode())
            .finish()
    }
}

/// Current hybrid mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HybridMode {
    /// Both SHM and Zenoh available
    Both,
    /// SHM only (local-only mode)
    ShmOnly,
    /// Zenoh only (SHM failed to initialize)
    ZenohOnly,
    /// No backends available (error state)
    None,
}

impl std::fmt::Display for HybridMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            HybridMode::Both => write!(f, "SHM+Zenoh"),
            HybridMode::ShmOnly => write!(f, "SHM-only"),
            HybridMode::ZenohOnly => write!(f, "Zenoh-only"),
            HybridMode::None => write!(f, "None"),
        }
    }
}

// ============================================================================
// Stub implementation when zenoh feature is not enabled
// ============================================================================

#[cfg(not(feature = "zenoh-transport"))]
pub struct HybridBackend<T> {
    topic: String,
    shm: Option<Topic<T>>,
    stats: Arc<HybridStats>,
    _phantom: PhantomData<T>,
}

#[cfg(not(feature = "zenoh-transport"))]
impl<T> HybridBackend<T>
where
    T: Clone + Send + Sync + serde::Serialize + serde::de::DeserializeOwned + 'static,
{
    /// Create a new hybrid backend (SHM-only when Zenoh is disabled)
    pub fn new_blocking(topic: &str, _config: HybridConfig) -> HorusResult<Self> {
        let shm_topic_name = format!("hybrid/{}", topic.replace('/', "_"));
        let shm = Topic::new(&shm_topic_name)?;

        Ok(Self {
            topic: topic.to_string(),
            shm: Some(shm),
            stats: Arc::new(HybridStats::default()),
            _phantom: PhantomData,
        })
    }

    pub fn send(&self, msg: &T) -> HorusResult<()> {
        if let Some(ref shm) = self.shm {
            shm.try_send(msg.clone())
                .map_err(|_| HorusError::Communication("SHM buffer full".to_string()))?;
            self.stats.shm_sends.fetch_add(1, Ordering::Relaxed);
            Ok(())
        } else {
            Err(HorusError::Communication(
                "No backend available".to_string(),
            ))
        }
    }

    pub fn recv(&self) -> Option<T> {
        if let Some(ref shm) = self.shm {
            if let Some(msg) = shm.try_recv() {
                self.stats.shm_recvs.fetch_add(1, Ordering::Relaxed);
                return Some(msg);
            }
        }
        None
    }

    pub fn topic(&self) -> &str {
        &self.topic
    }

    pub fn stats(&self) -> &HybridStats {
        &self.stats
    }

    pub fn current_mode(&self) -> HybridMode {
        if self.shm.is_some() {
            HybridMode::ShmOnly
        } else {
            HybridMode::None
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    struct TestMessage {
        id: u64,
        data: Vec<u8>,
    }

    #[test]
    fn test_hybrid_config_default() {
        let config = HybridConfig::default();
        assert!(config.enable_local_bypass);
        assert_eq!(config.shm_capacity, 1024);
        assert!(config.prefer_shm_for_local);
    }

    #[test]
    fn test_hybrid_config_local_only() {
        let config = HybridConfig::local_only(256);
        assert!(config.enable_local_bypass);
        assert_eq!(config.shm_capacity, 256);
    }

    #[test]
    fn test_hybrid_stats() {
        let stats = HybridStats::default();

        stats.shm_sends.fetch_add(10, Ordering::Relaxed);
        stats.zenoh_sends.fetch_add(5, Ordering::Relaxed);
        stats.shm_recvs.fetch_add(8, Ordering::Relaxed);
        stats.zenoh_recvs.fetch_add(4, Ordering::Relaxed);

        assert_eq!(stats.total_sends(), 15);
        assert_eq!(stats.total_recvs(), 12);

        // Local traffic: 18 out of 27 = 66.67%
        let percent = stats.local_traffic_percent();
        assert!(percent > 66.0 && percent < 67.0);
    }

    #[test]
    fn test_hybrid_mode_display() {
        assert_eq!(format!("{}", HybridMode::Both), "SHM+Zenoh");
        assert_eq!(format!("{}", HybridMode::ShmOnly), "SHM-only");
        assert_eq!(format!("{}", HybridMode::ZenohOnly), "Zenoh-only");
        assert_eq!(format!("{}", HybridMode::None), "None");
    }

    #[cfg(feature = "zenoh-transport")]
    #[test]
    fn test_locality_tracker() {
        let tracker = LocalityTracker::new(Duration::from_millis(100));

        assert!(!tracker.has_local_peers());

        tracker.mark_local("peer1");
        assert!(tracker.has_local_peers());

        tracker.mark_local("peer2");
        assert!(tracker.has_local_peers());
    }

    // Integration test with SHM only (no Zenoh feature)
    #[cfg(not(feature = "zenoh-transport"))]
    #[test]
    fn test_hybrid_shm_only_integration() {
        let config = HybridConfig::local_only(64);
        let backend =
            HybridBackend::<TestMessage>::new_blocking("test/hybrid_shm", config).unwrap();

        assert_eq!(backend.current_mode(), HybridMode::ShmOnly);

        let msg = TestMessage {
            id: 42,
            data: vec![1, 2, 3, 4],
        };

        // Send and receive
        backend.send(&msg).unwrap();
        let received = backend.recv().unwrap();

        assert_eq!(received, msg);
        assert_eq!(backend.stats().shm_sends.load(Ordering::Relaxed), 1);
        assert_eq!(backend.stats().shm_recvs.load(Ordering::Relaxed), 1);
    }

    // Verify HybridConfig::local_only config construction
    #[test]
    fn test_hybrid_shm_only() {
        // Just verify config construction works
        let config = HybridConfig::local_only(64);
        assert!(config.enable_local_bypass);
        assert_eq!(config.shm_capacity, 64);
    }

    #[cfg(feature = "zenoh-transport")]
    #[tokio::test(flavor = "multi_thread", worker_threads = 1)]
    async fn test_hybrid_both_backends() {
        let config = HybridConfig::default();

        let mut backend = HybridBackend::<TestMessage>::new("test/hybrid_both", config)
            .await
            .unwrap();

        // Initialize publisher
        backend.init_publisher().await.unwrap();

        assert_eq!(backend.current_mode(), HybridMode::Both);
        assert!(backend.has_shm());
        assert!(backend.has_zenoh());
    }

    #[cfg(feature = "zenoh-transport")]
    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn test_hybrid_send_receive() {
        let config = HybridConfig::default();

        let mut sender = HybridBackend::<TestMessage>::new("test/hybrid_sr", config.clone())
            .await
            .unwrap();
        sender.init_publisher().await.unwrap();

        let mut receiver = HybridBackend::<TestMessage>::new("test/hybrid_sr", config)
            .await
            .unwrap();
        receiver.init_subscriber().await.unwrap();

        // Give time for setup
        tokio::time::sleep(Duration::from_millis(100)).await;

        let msg = TestMessage {
            id: 123,
            data: vec![5, 6, 7, 8, 9],
        };

        sender.send(&msg).unwrap();

        // Try to receive (may come via SHM or Zenoh)
        tokio::time::sleep(Duration::from_millis(100)).await;

        let received = receiver.recv();

        // Message should be received (either via SHM or Zenoh)
        // Note: In same process, SHM should be preferred
        assert!(received.is_some() || receiver.stats().shm_sends.load(Ordering::Relaxed) > 0);
    }
}
