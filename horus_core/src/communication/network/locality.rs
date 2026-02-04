//! Locality Detection for Zenoh Peers
//!
//! Detects when Zenoh publishers and subscribers are on the same machine or process
//! to enable local optimizations like shared memory bypass.
//!
//! # Locality Levels
//!
//! - **Same Process**: Peers share the same memory space (fastest)
//! - **Same Machine**: Peers are on the same host (can use SHM/Unix sockets)
//! - **Same Network**: Peers are on the same LAN
//! - **Remote**: Peers are across WAN/Internet
//!
//! # Detection Methods
//!
//! 1. **Process ID comparison**: Same PID = same process
//! 2. **Hostname comparison**: Same hostname = same machine
//! 3. **Zenoh session ID**: Unique per session, can track peers
//! 4. **IP address check**: Local addresses (127.x, ::1) = same machine
//!
//! # Example
//!
//! ```ignore
//! use horus_core::communication::network::locality::{LocalityDetector, LocalityLevel};
//!
//! let detector = LocalityDetector::new("robot1");
//!
//! // Register a peer
//! detector.register_peer("peer-123", LocalityPeerInfo {
//!     hostname: "robot1".to_string(),
//!     pid: 12345,
//!     ..Default::default()
//! });
//!
//! // Check locality
//! if detector.is_local_peer("peer-123") {
//!     // Use SHM for this peer
//! }
//! ```

use parking_lot::RwLock;
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

/// Locality level between two peers
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum LocalityLevel {
    /// Same process (share memory space)
    SameProcess = 0,
    /// Same machine (can use SHM/Unix sockets)
    SameMachine = 1,
    /// Same network subnet
    SameNetwork = 2,
    /// Remote (across WAN/Internet)
    Remote = 3,
    /// Unknown (not enough info to determine)
    #[default]
    Unknown = 4,
}

impl std::fmt::Display for LocalityLevel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SameProcess => write!(f, "same-process"),
            Self::SameMachine => write!(f, "same-machine"),
            Self::SameNetwork => write!(f, "same-network"),
            Self::Remote => write!(f, "remote"),
            Self::Unknown => write!(f, "unknown"),
        }
    }
}

impl LocalityLevel {
    /// Check if this locality level allows SHM optimization
    pub fn allows_shm(&self) -> bool {
        matches!(self, Self::SameProcess | Self::SameMachine)
    }

    /// Check if this is a local peer (same machine or process)
    pub fn is_local(&self) -> bool {
        matches!(self, Self::SameProcess | Self::SameMachine)
    }
}

/// Information about a peer
#[derive(Debug, Clone, Default)]
pub struct LocalityPeerInfo {
    /// Peer's hostname
    pub hostname: String,
    /// Peer's process ID
    pub pid: u32,
    /// Peer's Zenoh session ID (if known)
    pub session_id: Option<String>,
    /// Peer's IP addresses
    pub addresses: Vec<String>,
    /// When this peer was first seen
    pub first_seen: Option<Instant>,
    /// When this peer was last seen
    pub last_seen: Option<Instant>,
    /// Computed locality level
    pub locality: LocalityLevel,
    /// Custom metadata
    pub metadata: HashMap<String, String>,
}

impl LocalityPeerInfo {
    /// Create a new peer info with hostname and PID
    pub fn new(hostname: &str, pid: u32) -> Self {
        Self {
            hostname: hostname.to_string(),
            pid,
            first_seen: Some(Instant::now()),
            last_seen: Some(Instant::now()),
            ..Default::default()
        }
    }

    /// Create peer info for the current process
    pub fn current_process() -> Self {
        Self {
            hostname: get_hostname(),
            pid: std::process::id(),
            first_seen: Some(Instant::now()),
            last_seen: Some(Instant::now()),
            locality: LocalityLevel::SameProcess,
            ..Default::default()
        }
    }

    /// Check if this peer is stale (not seen for a while)
    pub fn is_stale(&self, timeout: Duration) -> bool {
        if let Some(last_seen) = self.last_seen {
            last_seen.elapsed() > timeout
        } else {
            true
        }
    }

    /// Update last seen timestamp
    pub fn touch(&mut self) {
        self.last_seen = Some(Instant::now());
    }

    /// Add an IP address
    pub fn with_address(mut self, addr: &str) -> Self {
        self.addresses.push(addr.to_string());
        self
    }

    /// Set session ID
    pub fn with_session_id(mut self, id: &str) -> Self {
        self.session_id = Some(id.to_string());
        self
    }
}

/// Statistics for locality detection
#[derive(Debug, Default)]
pub struct LocalityStats {
    /// Number of locality lookups
    pub lookups: AtomicU64,
    /// Cache hits
    pub cache_hits: AtomicU64,
    /// Cache misses (had to recompute)
    pub cache_misses: AtomicU64,
    /// Number of registered peers
    pub registered_peers: AtomicU64,
    /// Number of local peers detected
    pub local_peers: AtomicU64,
    /// Number of remote peers detected
    pub remote_peers: AtomicU64,
}

impl LocalityStats {
    /// Get cache hit rate as percentage (0-100)
    pub fn cache_hit_rate(&self) -> f64 {
        let hits = self.cache_hits.load(Ordering::Relaxed);
        let misses = self.cache_misses.load(Ordering::Relaxed);
        let total = hits + misses;
        if total == 0 {
            100.0
        } else {
            (hits as f64 / total as f64) * 100.0
        }
    }
}

/// Locality detector for Zenoh peers
///
/// Maintains a cache of peer information and computes locality levels
/// for efficient routing decisions.
pub struct LocalityDetector {
    /// Our own hostname
    our_hostname: String,
    /// Our own PID
    our_pid: u32,
    /// Our Zenoh session ID (if known)
    our_session_id: Option<String>,
    /// Registered peers
    peers: RwLock<HashMap<String, LocalityPeerInfo>>,
    /// Locality cache (peer_id -> locality level)
    locality_cache: RwLock<HashMap<String, LocalityLevel>>,
    /// Statistics
    stats: LocalityStats,
    /// Stale peer timeout
    stale_timeout: Duration,
}

impl LocalityDetector {
    /// Create a new locality detector
    pub fn new(hostname: &str) -> Self {
        Self {
            our_hostname: hostname.to_string(),
            our_pid: std::process::id(),
            our_session_id: None,
            peers: RwLock::new(HashMap::new()),
            locality_cache: RwLock::new(HashMap::new()),
            stats: LocalityStats::default(),
            stale_timeout: Duration::from_secs(60),
        }
    }

    /// Create with auto-detected hostname
    pub fn auto() -> Self {
        Self::new(&get_hostname())
    }

    /// Set our Zenoh session ID
    pub fn set_session_id(&mut self, session_id: &str) {
        self.our_session_id = Some(session_id.to_string());
    }

    /// Set stale peer timeout
    pub fn set_stale_timeout(&mut self, timeout: Duration) {
        self.stale_timeout = timeout;
    }

    /// Register a peer
    pub fn register_peer(&self, peer_id: &str, mut info: LocalityPeerInfo) {
        // Compute locality for this peer
        info.locality = self.compute_locality(&info);

        // Update stats
        if info.locality.is_local() {
            self.stats.local_peers.fetch_add(1, Ordering::Relaxed);
        } else {
            self.stats.remote_peers.fetch_add(1, Ordering::Relaxed);
        }
        self.stats.registered_peers.fetch_add(1, Ordering::Relaxed);

        // Insert into peers map
        let mut peers = self.peers.write();
        peers.insert(peer_id.to_string(), info.clone());

        // Update cache
        let mut cache = self.locality_cache.write();
        cache.insert(peer_id.to_string(), info.locality);

        log::debug!(
            "Registered peer '{}' with locality: {}",
            peer_id,
            info.locality
        );
    }

    /// Unregister a peer
    pub fn unregister_peer(&self, peer_id: &str) -> Option<LocalityPeerInfo> {
        let mut peers = self.peers.write();
        let info = peers.remove(peer_id);

        let mut cache = self.locality_cache.write();
        cache.remove(peer_id);

        if let Some(ref info) = info {
            if info.locality.is_local() {
                self.stats.local_peers.fetch_sub(1, Ordering::Relaxed);
            } else {
                self.stats.remote_peers.fetch_sub(1, Ordering::Relaxed);
            }
            self.stats.registered_peers.fetch_sub(1, Ordering::Relaxed);
        }

        info
    }

    /// Get the locality level for a peer
    pub fn get_locality(&self, peer_id: &str) -> LocalityLevel {
        self.stats.lookups.fetch_add(1, Ordering::Relaxed);

        // Check cache first
        {
            let cache = self.locality_cache.read();
            if let Some(&locality) = cache.get(peer_id) {
                self.stats.cache_hits.fetch_add(1, Ordering::Relaxed);
                return locality;
            }
        }

        // Cache miss - check peers map
        self.stats.cache_misses.fetch_add(1, Ordering::Relaxed);

        let locality = {
            let peers = self.peers.read();
            peers.get(peer_id).map(|info| info.locality)
        };

        if let Some(locality) = locality {
            // Update cache
            let mut cache = self.locality_cache.write();
            cache.insert(peer_id.to_string(), locality);
            locality
        } else {
            LocalityLevel::Unknown
        }
    }

    /// Check if a peer is local (same process or same machine)
    ///
    /// This is the primary API for routing decisions.
    pub fn is_local_peer(&self, peer_id: &str) -> bool {
        self.get_locality(peer_id).is_local()
    }

    /// Check if a peer is on the same machine
    pub fn is_same_machine(&self, peer_id: &str) -> bool {
        let locality = self.get_locality(peer_id);
        matches!(locality, LocalityLevel::SameProcess | LocalityLevel::SameMachine)
    }

    /// Check if a peer is in the same process
    pub fn is_same_process(&self, peer_id: &str) -> bool {
        matches!(self.get_locality(peer_id), LocalityLevel::SameProcess)
    }

    /// Get peer info
    pub fn get_peer(&self, peer_id: &str) -> Option<LocalityPeerInfo> {
        self.peers.read().get(peer_id).cloned()
    }

    /// Get all peers
    pub fn all_peers(&self) -> Vec<(String, LocalityPeerInfo)> {
        self.peers
            .read()
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
    }

    /// Get local peers only
    pub fn local_peers(&self) -> Vec<(String, LocalityPeerInfo)> {
        self.peers
            .read()
            .iter()
            .filter(|(_, v)| v.locality.is_local())
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
    }

    /// Get statistics
    pub fn stats(&self) -> &LocalityStats {
        &self.stats
    }

    /// Update peer's last seen time
    pub fn touch_peer(&self, peer_id: &str) {
        let mut peers = self.peers.write();
        if let Some(info) = peers.get_mut(peer_id) {
            info.touch();
        }
    }

    /// Remove stale peers
    pub fn cleanup_stale_peers(&self) -> usize {
        let mut peers = self.peers.write();
        let mut cache = self.locality_cache.write();

        let stale: Vec<String> = peers
            .iter()
            .filter(|(_, info)| info.is_stale(self.stale_timeout))
            .map(|(id, _)| id.clone())
            .collect();

        let count = stale.len();
        for id in stale {
            if let Some(info) = peers.remove(&id) {
                cache.remove(&id);
                if info.locality.is_local() {
                    self.stats.local_peers.fetch_sub(1, Ordering::Relaxed);
                } else {
                    self.stats.remote_peers.fetch_sub(1, Ordering::Relaxed);
                }
                self.stats.registered_peers.fetch_sub(1, Ordering::Relaxed);
            }
        }

        count
    }

    /// Compute locality level for a peer
    fn compute_locality(&self, info: &LocalityPeerInfo) -> LocalityLevel {
        // Same PID = same process
        if info.pid == self.our_pid {
            return LocalityLevel::SameProcess;
        }

        // Same hostname = same machine
        if !info.hostname.is_empty()
            && info.hostname.to_lowercase() == self.our_hostname.to_lowercase()
        {
            return LocalityLevel::SameMachine;
        }

        // Check if any address is local
        for addr in &info.addresses {
            if is_local_address(addr) {
                return LocalityLevel::SameMachine;
            }
        }

        // Check for local addresses that would indicate same machine
        // (e.g., if peer connects from 127.0.0.1)
        for addr in &info.addresses {
            if is_private_address(addr) {
                return LocalityLevel::SameNetwork;
            }
        }

        LocalityLevel::Remote
    }

    /// Detect locality from a Zenoh locator string
    ///
    /// Parses locator strings like:
    /// - "tcp/127.0.0.1:7447" -> SameMachine
    /// - "tcp/192.168.1.100:7447" -> SameNetwork
    /// - "tcp/cloud.example.com:7447" -> Remote
    pub fn detect_from_locator(&self, locator: &str) -> LocalityLevel {
        // Parse the locator to extract address
        let parts: Vec<&str> = locator.split('/').collect();
        if parts.len() < 2 {
            return LocalityLevel::Unknown;
        }

        let addr_port = parts[1];
        let addr = addr_port.split(':').next().unwrap_or("");

        if is_local_address(addr) {
            LocalityLevel::SameMachine
        } else if is_private_address(addr) {
            LocalityLevel::SameNetwork
        } else if addr.is_empty() {
            LocalityLevel::Unknown
        } else {
            LocalityLevel::Remote
        }
    }

    /// Register a peer from locator
    pub fn register_from_locator(&self, peer_id: &str, locator: &str) {
        let locality = self.detect_from_locator(locator);

        // Extract address from locator
        let parts: Vec<&str> = locator.split('/').collect();
        let addr = if parts.len() >= 2 {
            parts[1].split(':').next().unwrap_or("").to_string()
        } else {
            String::new()
        };

        let info = LocalityPeerInfo {
            hostname: String::new(),
            pid: 0,
            addresses: if addr.is_empty() { vec![] } else { vec![addr] },
            locality,
            first_seen: Some(Instant::now()),
            last_seen: Some(Instant::now()),
            ..Default::default()
        };

        // Register with pre-computed locality
        let mut peers = self.peers.write();
        peers.insert(peer_id.to_string(), info.clone());

        let mut cache = self.locality_cache.write();
        cache.insert(peer_id.to_string(), locality);

        if locality.is_local() {
            self.stats.local_peers.fetch_add(1, Ordering::Relaxed);
        } else {
            self.stats.remote_peers.fetch_add(1, Ordering::Relaxed);
        }
        self.stats.registered_peers.fetch_add(1, Ordering::Relaxed);
    }
}

impl Default for LocalityDetector {
    fn default() -> Self {
        Self::auto()
    }
}

/// Get the current hostname
pub fn get_hostname() -> String {
    // Try using nix crate first (which is already a dependency)
    #[cfg(unix)]
    {
        use std::ffi::CStr;
        let mut buf = [0u8; 256];
        unsafe {
            if libc::gethostname(buf.as_mut_ptr() as *mut libc::c_char, buf.len()) == 0 {
                if let Ok(cstr) = CStr::from_ptr(buf.as_ptr() as *const libc::c_char).to_str() {
                    return cstr.to_string();
                }
            }
        }
    }

    // Fallback: try reading from /etc/hostname
    #[cfg(unix)]
    if let Ok(hostname) = std::fs::read_to_string("/etc/hostname") {
        let hostname = hostname.trim();
        if !hostname.is_empty() {
            return hostname.to_string();
        }
    }

    // Windows fallback
    #[cfg(windows)]
    if let Ok(hostname) = std::env::var("COMPUTERNAME") {
        return hostname;
    }

    "unknown".to_string()
}

/// Check if an address is a local loopback address
pub fn is_local_address(addr: &str) -> bool {
    addr == "127.0.0.1"
        || addr == "::1"
        || addr == "localhost"
        || addr.starts_with("127.")
}

/// Check if an address is a private/LAN address
pub fn is_private_address(addr: &str) -> bool {
    // IPv4 private ranges
    if addr.starts_with("10.")
        || addr.starts_with("172.16.")
        || addr.starts_with("172.17.")
        || addr.starts_with("172.18.")
        || addr.starts_with("172.19.")
        || addr.starts_with("172.2")
        || addr.starts_with("172.30.")
        || addr.starts_with("172.31.")
        || addr.starts_with("192.168.")
    {
        return true;
    }

    // IPv6 private ranges
    if addr.starts_with("fe80:")
        || addr.starts_with("fc00:")
        || addr.starts_with("fd")
    {
        return true;
    }

    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_locality_level_ordering() {
        assert!(LocalityLevel::SameProcess < LocalityLevel::SameMachine);
        assert!(LocalityLevel::SameMachine < LocalityLevel::SameNetwork);
        assert!(LocalityLevel::SameNetwork < LocalityLevel::Remote);
    }

    #[test]
    fn test_locality_level_local() {
        assert!(LocalityLevel::SameProcess.is_local());
        assert!(LocalityLevel::SameMachine.is_local());
        assert!(!LocalityLevel::SameNetwork.is_local());
        assert!(!LocalityLevel::Remote.is_local());
        assert!(!LocalityLevel::Unknown.is_local());
    }

    #[test]
    fn test_locality_level_allows_shm() {
        assert!(LocalityLevel::SameProcess.allows_shm());
        assert!(LocalityLevel::SameMachine.allows_shm());
        assert!(!LocalityLevel::SameNetwork.allows_shm());
        assert!(!LocalityLevel::Remote.allows_shm());
    }

    #[test]
    fn test_is_local_address() {
        assert!(is_local_address("127.0.0.1"));
        assert!(is_local_address("127.0.1.1"));
        assert!(is_local_address("::1"));
        assert!(is_local_address("localhost"));
        assert!(!is_local_address("192.168.1.1"));
        assert!(!is_local_address("10.0.0.1"));
    }

    #[test]
    fn test_is_private_address() {
        assert!(is_private_address("10.0.0.1"));
        assert!(is_private_address("172.16.0.1"));
        assert!(is_private_address("192.168.1.1"));
        assert!(is_private_address("fe80::1"));
        assert!(!is_private_address("8.8.8.8"));
        assert!(!is_private_address("cloud.example.com"));
    }

    #[test]
    fn test_peer_info_current_process() {
        let info = LocalityPeerInfo::current_process();
        assert_eq!(info.pid, std::process::id());
        assert!(!info.hostname.is_empty());
        assert_eq!(info.locality, LocalityLevel::SameProcess);
    }

    #[test]
    fn test_locality_detector_same_process() {
        let detector = LocalityDetector::new("test-host");
        let our_pid = std::process::id();

        // Register a peer with same PID
        detector.register_peer(
            "peer1",
            LocalityPeerInfo::new("different-host", our_pid),
        );

        // Should detect as same process (PID match takes precedence)
        assert!(detector.is_same_process("peer1"));
        assert!(detector.is_local_peer("peer1"));
    }

    #[test]
    fn test_locality_detector_same_machine() {
        let detector = LocalityDetector::new("test-host");

        // Register a peer with same hostname, different PID
        detector.register_peer(
            "peer2",
            LocalityPeerInfo::new("test-host", 99999),
        );

        assert!(!detector.is_same_process("peer2"));
        assert!(detector.is_same_machine("peer2"));
        assert!(detector.is_local_peer("peer2"));
    }

    #[test]
    fn test_locality_detector_remote() {
        let detector = LocalityDetector::new("test-host");

        // Register a peer with different hostname
        let mut info = LocalityPeerInfo::new("remote-host", 12345);
        info.addresses = vec!["8.8.8.8".to_string()];
        detector.register_peer("peer3", info);

        assert!(!detector.is_local_peer("peer3"));
        assert_eq!(detector.get_locality("peer3"), LocalityLevel::Remote);
    }

    #[test]
    fn test_locality_detector_unknown() {
        let detector = LocalityDetector::auto();

        // Unknown peer
        assert_eq!(detector.get_locality("nonexistent"), LocalityLevel::Unknown);
        assert!(!detector.is_local_peer("nonexistent"));
    }

    #[test]
    fn test_locality_detector_cache() {
        let detector = LocalityDetector::new("test-host");
        detector.register_peer("peer1", LocalityPeerInfo::new("test-host", 12345));

        // All lookups are cache hits since register_peer pre-populates the cache
        let _ = detector.get_locality("peer1");
        let _ = detector.get_locality("peer1");
        let _ = detector.get_locality("peer1");

        let stats = detector.stats();
        assert_eq!(stats.lookups.load(Ordering::Relaxed), 3);
        assert_eq!(stats.cache_hits.load(Ordering::Relaxed), 3);
        assert_eq!(stats.cache_misses.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn test_locality_detector_unregister() {
        let detector = LocalityDetector::new("test-host");
        detector.register_peer("peer1", LocalityPeerInfo::new("test-host", 12345));

        assert!(detector.is_local_peer("peer1"));

        let info = detector.unregister_peer("peer1");
        assert!(info.is_some());
        assert!(!detector.is_local_peer("peer1"));
    }

    #[test]
    fn test_locality_detector_locator_parsing() {
        let detector = LocalityDetector::auto();

        assert_eq!(
            detector.detect_from_locator("tcp/127.0.0.1:7447"),
            LocalityLevel::SameMachine
        );
        assert_eq!(
            detector.detect_from_locator("tcp/192.168.1.100:7447"),
            LocalityLevel::SameNetwork
        );
        assert_eq!(
            detector.detect_from_locator("tcp/8.8.8.8:7447"),
            LocalityLevel::Remote
        );
    }

    #[test]
    fn test_locality_detector_register_from_locator() {
        let detector = LocalityDetector::auto();

        detector.register_from_locator("local-peer", "tcp/127.0.0.1:7447");
        assert!(detector.is_local_peer("local-peer"));

        detector.register_from_locator("remote-peer", "tcp/8.8.8.8:7447");
        assert!(!detector.is_local_peer("remote-peer"));
    }

    #[test]
    fn test_locality_stats() {
        let stats = LocalityStats::default();

        // No lookups yet
        assert_eq!(stats.cache_hit_rate(), 100.0);

        stats.cache_hits.fetch_add(7, Ordering::Relaxed);
        stats.cache_misses.fetch_add(3, Ordering::Relaxed);

        assert!((stats.cache_hit_rate() - 70.0).abs() < 0.1);
    }

    #[test]
    fn test_cleanup_stale_peers() {
        let mut detector = LocalityDetector::new("test-host");
        detector.set_stale_timeout(Duration::from_millis(10));

        detector.register_peer("stale-peer", LocalityPeerInfo::new("other-host", 123));

        // Wait for it to become stale
        std::thread::sleep(Duration::from_millis(20));

        let cleaned = detector.cleanup_stale_peers();
        assert_eq!(cleaned, 1);
        assert_eq!(detector.get_locality("stale-peer"), LocalityLevel::Unknown);
    }

    #[test]
    fn test_local_peers_list() {
        let detector = LocalityDetector::new("test-host");

        detector.register_peer("local1", LocalityPeerInfo::new("test-host", 111));
        detector.register_peer("local2", LocalityPeerInfo::new("test-host", 222));
        detector.register_peer("remote", LocalityPeerInfo::new("other-host", 333).with_address("8.8.8.8"));

        let local = detector.local_peers();
        assert_eq!(local.len(), 2);

        let all = detector.all_peers();
        assert_eq!(all.len(), 3);
    }

    #[test]
    fn test_peer_info_builder() {
        let info = LocalityPeerInfo::new("host1", 12345)
            .with_address("192.168.1.100")
            .with_session_id("zenoh-session-123");

        assert_eq!(info.hostname, "host1");
        assert_eq!(info.pid, 12345);
        assert_eq!(info.addresses, vec!["192.168.1.100"]);
        assert_eq!(info.session_id, Some("zenoh-session-123".to_string()));
    }
}
