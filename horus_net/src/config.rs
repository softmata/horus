//! Network configuration — parsed from [network] in horus.toml or defaults.

use std::net::{SocketAddr, ToSocketAddrs};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

/// Network configuration for the replicator.
#[derive(Debug, Clone)]
pub struct NetConfig {
    /// Enable/disable networking entirely. Env: HORUS_NO_NETWORK=1 disables.
    pub enabled: bool,
    /// UDP data port. Default: 9100. Env: HORUS_NET_PORT.
    pub port: u16,
    /// Multicast group for discovery. Default: 224.0.69.72. Env: HORUS_NET_MULTICAST.
    pub multicast_group: String,
    /// Direct unicast peers (skip multicast). Env: HORUS_NET_PEER=ip1,ip2.
    pub peers: Vec<String>,
    /// Shared secret for peer filtering (NOT security). Env: HORUS_NET_SECRET.
    pub secret: Option<String>,
    /// Import control: "deny", "auto", or explicit list of topic patterns.
    pub import: ImportConfig,
    /// Export deny patterns (e.g., ["camera.*", "debug.*"]).
    pub deny_export: Vec<String>,
    /// Safety heartbeat settings.
    pub safety: SafetyConfig,
    /// Enabled optimizers (e.g., ["fusion", "delta"]).
    pub optimizers: Vec<String>,
    /// Per-topic overrides.
    pub topic_overrides: std::collections::HashMap<String, TopicNetConfig>,
}

/// Import control configuration.
#[derive(Debug, Clone)]
pub enum ImportConfig {
    /// Deny all imports.
    Deny,
    /// Auto: import topics we subscribe to but don't publish.
    Auto,
    /// Explicit list of allowed topic patterns.
    AllowList(Vec<String>),
}

impl Default for ImportConfig {
    fn default() -> Self {
        Self::Auto
    }
}

/// Safety heartbeat configuration.
#[derive(Debug, Clone)]
pub struct SafetyConfig {
    /// Heartbeat interval in milliseconds. Default: 50.
    pub heartbeat_ms: u64,
    /// Number of missed heartbeats before link is declared dead. Default: 3.
    pub missed_threshold: u32,
    /// Default action when a link is lost. Default: "warn".
    pub on_link_lost: String,
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self {
            heartbeat_ms: 50,
            missed_threshold: 3,
            on_link_lost: "warn".into(),
        }
    }
}

/// Per-topic network configuration override.
#[derive(Debug, Clone, Default)]
pub struct TopicNetConfig {
    /// Priority override: "immediate", "realtime", "normal", "bulk".
    pub priority: Option<String>,
    /// Reliability override: "none", "redundant", "latched".
    pub reliability: Option<String>,
    /// Number of redundant copies (only if reliability=redundant).
    pub redundant_copies: Option<u8>,
    /// Link-lost action override for this topic.
    pub on_link_lost: Option<String>,
    /// Optimizers enabled for this topic.
    pub optimizers: Option<Vec<String>>,
    /// Spatial radius (for spatial optimizer).
    pub spatial_radius: Option<f64>,
    /// Predict threshold (for predict optimizer).
    pub predict_threshold: Option<f64>,
}

impl Default for NetConfig {
    fn default() -> Self {
        let no_network = std::env::var("HORUS_NO_NETWORK")
            .map(|v| v == "1" || v.eq_ignore_ascii_case("true"))
            .unwrap_or(false);

        Self {
            enabled: !no_network,
            port: std::env::var("HORUS_NET_PORT")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(9100),
            multicast_group: std::env::var("HORUS_NET_MULTICAST")
                .unwrap_or_else(|_| "224.0.69.72".into()),
            peers: std::env::var("HORUS_NET_PEER")
                .map(|v| {
                    v.split(',')
                        .map(|s| s.trim().to_string())
                        .filter(|s| !s.is_empty())
                        .collect()
                })
                .unwrap_or_default(),
            secret: std::env::var("HORUS_NET_SECRET").ok(),
            import: ImportConfig::default(),
            deny_export: Vec::new(),
            safety: SafetyConfig::default(),
            optimizers: Vec::new(),
            topic_overrides: std::collections::HashMap::new(),
        }
    }
}

impl NetConfig {
    /// Determine discovery mode from config.
    pub fn discovery_mode(&self) -> DiscoveryMode {
        if self.peers.is_empty() {
            DiscoveryMode::Multicast {
                group: self.multicast_group.clone(),
            }
        } else {
            let addrs = self
                .peers
                .iter()
                .filter_map(|p| resolve_peer_addr(p, self.port))
                .collect();
            DiscoveryMode::Unicast { peers: addrs }
        }
    }

    /// Create a minimal config for testing (no env var reads).
    pub fn test_config(port: u16) -> Self {
        Self {
            enabled: true,
            port,
            multicast_group: "224.0.69.72".into(),
            peers: vec![],
            secret: None,
            import: ImportConfig::Auto,
            deny_export: vec![],
            safety: SafetyConfig::default(),
            optimizers: vec![],
            topic_overrides: std::collections::HashMap::new(),
        }
    }

    /// Get per-topic config override, if any. Checks exact name first, then glob patterns.
    pub fn topic_config(&self, topic_name: &str) -> Option<&TopicNetConfig> {
        // Exact match first
        if let Some(cfg) = self.topic_overrides.get(topic_name) {
            return Some(cfg);
        }
        // Glob match
        for (pattern, cfg) in &self.topic_overrides {
            if pattern.contains('*') && crate::guard::glob_match_topic(topic_name, pattern) {
                return Some(cfg);
            }
        }
        None
    }

    /// Compute the secret hash (4 bytes) if a secret is configured.
    pub fn secret_hash(&self) -> [u8; 4] {
        match &self.secret {
            Some(s) if !s.is_empty() => crate::discovery::compute_secret_hash(s),
            _ => [0u8; 4],
        }
    }
}

/// How peers are discovered.
#[derive(Debug, Clone)]
pub enum DiscoveryMode {
    /// UDP multicast on the LAN (default, zero config).
    Multicast { group: String },
    /// Direct unicast to specific peers (HORUS_NET_PEER).
    /// Skips multicast entirely.
    Unicast { peers: Vec<SocketAddr> },
}

/// Resolve a peer address string to a SocketAddr.
/// Accepts "ip", "ip:port", or hostname.
fn resolve_peer_addr(peer: &str, default_port: u16) -> Option<SocketAddr> {
    // Try as-is (ip:port)
    if let Ok(addr) = peer.parse::<SocketAddr>() {
        return Some(addr);
    }
    // Try as ip (no port) — append default port
    let with_port = format!("{peer}:{default_port}");
    with_port.to_socket_addrs().ok()?.next()
}

// ─── No-Peers Diagnostic ────────────────────────────────────────────────────

/// Tracks whether the no-peers diagnostic has been printed.
/// Only prints once per process lifetime.
static NO_PEERS_PRINTED: AtomicBool = AtomicBool::new(false);

/// Check if we should print the no-peers diagnostic.
/// Call this periodically from the replicator. Returns the message if it should be printed.
///
/// Prints after `timeout` with zero discovered peers, exactly once.
pub fn check_no_peers_diagnostic(
    alive_peer_count: usize,
    start_time: Instant,
    timeout: Duration,
) -> Option<String> {
    if alive_peer_count > 0 {
        return None;
    }
    if start_time.elapsed() < timeout {
        return None;
    }
    // Only print once
    if NO_PEERS_PRINTED.swap(true, Ordering::Relaxed) {
        return None;
    }

    Some(
        "[horus_net] No remote peers found after 5s.\n  \
         - Multicast discovery only works on the same LAN subnet\n  \
         - WiFi networks often block multicast\n  \
         For direct connection:  HORUS_NET_PEER=<robot-ip> horus run\n  \
         For cross-subnet/WiFi:  horus install horus-zenoh"
            .to_string(),
    )
}

/// Reset the no-peers diagnostic flag (for testing).
#[cfg(test)]
pub fn reset_no_peers_diagnostic() {
    NO_PEERS_PRINTED.store(false, Ordering::Relaxed);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config() -> NetConfig {
        NetConfig {
            enabled: true,
            port: 9100,
            multicast_group: "224.0.69.72".into(),
            peers: vec![],
            secret: None,
            import: ImportConfig::Auto,
            deny_export: vec![],
            safety: SafetyConfig::default(),
            optimizers: vec![],
            topic_overrides: std::collections::HashMap::new(),
        }
    }

    #[test]
    fn default_config() {
        let config = make_config();
        assert!(config.enabled);
        assert_eq!(config.port, 9100);
        assert!(config.peers.is_empty());
        assert!(matches!(config.import, ImportConfig::Auto));
        assert_eq!(config.safety.heartbeat_ms, 50);
        assert_eq!(config.safety.missed_threshold, 3);
    }

    #[test]
    fn discovery_mode_multicast() {
        let config = make_config();
        match config.discovery_mode() {
            DiscoveryMode::Multicast { group } => assert_eq!(group, "224.0.69.72"),
            _ => panic!("expected multicast"),
        }
    }

    #[test]
    fn discovery_mode_unicast() {
        let mut config = make_config();
        config.peers = vec!["192.168.1.42".into()];
        match config.discovery_mode() {
            DiscoveryMode::Unicast { peers } => {
                assert_eq!(peers.len(), 1);
                assert_eq!(peers[0].ip().to_string(), "192.168.1.42");
                assert_eq!(peers[0].port(), 9100);
            }
            _ => panic!("expected unicast"),
        }
    }

    #[test]
    fn discovery_mode_unicast_with_port() {
        let mut config = make_config();
        config.peers = vec!["192.168.1.42:9200".into()];
        match config.discovery_mode() {
            DiscoveryMode::Unicast { peers } => {
                assert_eq!(peers[0].port(), 9200);
            }
            _ => panic!("expected unicast"),
        }
    }

    #[test]
    fn discovery_mode_multiple_unicast() {
        let mut config = make_config();
        config.peers = vec!["192.168.1.42".into(), "192.168.1.43".into()];
        match config.discovery_mode() {
            DiscoveryMode::Unicast { peers } => assert_eq!(peers.len(), 2),
            _ => panic!("expected unicast"),
        }
    }

    #[test]
    fn secret_hash_none() {
        let config = make_config();
        assert_eq!(config.secret_hash(), [0u8; 4]);
    }

    #[test]
    fn secret_hash_some() {
        let mut config = make_config();
        config.secret = Some("lab-42".into());
        assert_ne!(config.secret_hash(), [0u8; 4]);
    }

    #[test]
    fn topic_config_exact_match() {
        let mut config = make_config();
        config.topic_overrides.insert(
            "cmd_vel".into(),
            TopicNetConfig {
                priority: Some("realtime".into()),
                redundant_copies: Some(2),
                ..Default::default()
            },
        );
        let tc = config.topic_config("cmd_vel").unwrap();
        assert_eq!(tc.priority.as_deref(), Some("realtime"));
        assert_eq!(tc.redundant_copies, Some(2));
        assert!(config.topic_config("imu").is_none());
    }

    #[test]
    fn topic_config_glob_match() {
        let mut config = make_config();
        config.topic_overrides.insert(
            "fleet.*".into(),
            TopicNetConfig {
                optimizers: Some(vec!["spatial".into()]),
                spatial_radius: Some(15.0),
                ..Default::default()
            },
        );
        let tc = config.topic_config("fleet.pose").unwrap();
        assert_eq!(tc.spatial_radius, Some(15.0));
        assert!(config.topic_config("sensor.imu").is_none());
    }

    #[test]
    fn safety_defaults() {
        let config = make_config();
        assert_eq!(config.safety.heartbeat_ms, 50);
        assert_eq!(config.safety.missed_threshold, 3);
        assert_eq!(config.safety.on_link_lost, "warn");
    }

    #[test]
    fn no_peers_diagnostic_fires_once() {
        reset_no_peers_diagnostic();
        let start = Instant::now() - Duration::from_secs(10); // pretend started 10s ago

        let msg = check_no_peers_diagnostic(0, start, Duration::from_secs(5));
        assert!(msg.is_some());
        assert!(msg.unwrap().contains("No remote peers"));

        // Second call: should NOT print again
        let msg2 = check_no_peers_diagnostic(0, start, Duration::from_secs(5));
        assert!(msg2.is_none());

        reset_no_peers_diagnostic();
    }

    #[test]
    fn no_peers_diagnostic_suppressed_when_peers_exist() {
        reset_no_peers_diagnostic();
        let start = Instant::now() - Duration::from_secs(10);

        let msg = check_no_peers_diagnostic(1, start, Duration::from_secs(5));
        assert!(msg.is_none());

        reset_no_peers_diagnostic();
    }

    #[test]
    fn no_peers_diagnostic_suppressed_before_timeout() {
        reset_no_peers_diagnostic();
        let start = Instant::now(); // just started

        let msg = check_no_peers_diagnostic(0, start, Duration::from_secs(5));
        assert!(msg.is_none());

        reset_no_peers_diagnostic();
    }

    #[test]
    fn resolve_ip_only() {
        let addr = resolve_peer_addr("192.168.1.42", 9100).unwrap();
        assert_eq!(addr.ip().to_string(), "192.168.1.42");
        assert_eq!(addr.port(), 9100);
    }

    #[test]
    fn resolve_ip_port() {
        let addr = resolve_peer_addr("192.168.1.42:9200", 9100).unwrap();
        assert_eq!(addr.port(), 9200);
    }

    #[test]
    fn resolve_invalid() {
        assert!(resolve_peer_addr("not-a-valid-address-xxxxx", 9100).is_none());
    }
}
