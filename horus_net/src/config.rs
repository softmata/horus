//! Network configuration — parsed from `[network]` in horus.toml or defaults.

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
    /// Posture for acting on authenticated remote e-stop packets.
    /// `HORUS_ESTOP_KEY` is mandatory. Env: HORUS_ESTOP_REMOTE = warn | off.
    pub estop_remote: EstopRemotePolicy,
    /// Enabled optimizers (e.g., ["fusion", "spatial"]).
    ///
    /// "delta" is NOT supported: it encodes but never decodes, so enabling it
    /// corrupts replicated data. `OptimizerChain::from_config` refuses it.
    pub optimizers: Vec<String>,
    /// Per-topic overrides.
    pub topic_overrides: std::collections::HashMap<String, TopicNetConfig>,
}

/// Policy for acting on e-stop packets received from the network.
///
/// Networked e-stop requires HMAC authentication with `HORUS_ESTOP_KEY`.
/// This policy controls whether a valid, authenticated packet is acted upon.
/// Env: `HORUS_ESTOP_REMOTE` = `warn` (default) | `off`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EstopRemotePolicy {
    /// Act on a valid authenticated remote e-stop. Without a key, all remote
    /// e-stop packets are rejected regardless of this setting.
    #[default]
    Warn,
    /// Ignore remote e-stop entirely. Local (watchdog/deadline) e-stop is unaffected.
    Off,
}

impl EstopRemotePolicy {
    /// Parse from the `HORUS_ESTOP_REMOTE` env var. Unknown/empty → `Warn` (default).
    pub fn from_env() -> Self {
        match std::env::var("HORUS_ESTOP_REMOTE")
            .unwrap_or_default()
            .trim()
            .to_lowercase()
            .as_str()
        {
            "off" | "ignore" | "0" | "false" => Self::Off,
            _ => Self::Warn,
        }
    }
}

/// Import control configuration.
///
/// This is reach reduction, NOT an authorization control: the data plane is
/// unauthenticated (see the crate docs' trust model), so whatever this admits,
/// *any* host that can send UDP to this process can write into local SHM —
/// including actuation topics. The shipped default is [`Auto`](Self::Auto),
/// which is what makes zero-config LAN replication work; choose
/// [`Deny`](Self::Deny) (`HORUS_NET_IMPORT=deny`) on any network where an
/// untrusted host might be reachable.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub enum ImportConfig {
    /// Deny all imports. The only setting that keeps a remote host from writing
    /// into local SHM on a network you do not fully control.
    Deny,
    /// Auto: import topics we subscribe to but don't publish.
    ///
    /// Admits any topic this process subscribes to, from any peer that passes
    /// the source-address filter, with no proof of the sender's identity.
    #[default]
    Auto,
    /// Explicit list of allowed topic patterns.
    AllowList(Vec<String>),
}

impl ImportConfig {
    /// Parse from `HORUS_NET_IMPORT`.
    ///
    /// * unset / empty / `auto` → [`ImportConfig::Auto`] (the default)
    /// * `deny` / `none` / `off` → [`ImportConfig::Deny`]
    /// * anything else → a comma-separated allowlist of topic patterns
    ///
    /// Before this existed there was no env read, so `[network] import` in
    /// horus.toml could not reach the replicator and both `Deny` and
    /// `AllowList` were unreachable in shipped code.
    pub fn from_env() -> Self {
        let Ok(raw) = std::env::var("HORUS_NET_IMPORT") else {
            return Self::Auto;
        };
        Self::parse(&raw)
    }

    /// Parse an import spec. Exposed for tests and for horus.toml wiring.
    pub fn parse(raw: &str) -> Self {
        let trimmed = raw.trim();
        if trimmed.is_empty() {
            return Self::Auto;
        }
        match trimmed.to_lowercase().as_str() {
            "auto" => Self::Auto,
            "deny" | "none" | "off" | "false" => Self::Deny,
            _ => {
                let patterns: Vec<String> = trimmed
                    .split(',')
                    .map(|s| s.trim().to_string())
                    .filter(|s| !s.is_empty())
                    .collect();
                if patterns.is_empty() {
                    // A spec that parses to nothing must NOT silently become
                    // "allow everything" — deny is the safe reading of "the
                    // operator tried to restrict imports".
                    Self::Deny
                } else {
                    Self::AllowList(patterns)
                }
            }
        }
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

/// Detect if running inside WSL2 (Windows Subsystem for Linux).
/// WSL2's virtualized network adapter adds 50-200ms jitter to UDP,
/// causing false heartbeat timeouts with native 50ms intervals.
pub(crate) fn is_wsl2() -> bool {
    // WSL2 sets "microsoft" in the kernel version string
    std::fs::read_to_string("/proc/version")
        .map(|v| v.to_lowercase().contains("microsoft"))
        .unwrap_or(false)
}

impl Default for SafetyConfig {
    fn default() -> Self {
        let wsl = is_wsl2();

        // Environment overrides take precedence, then WSL2 defaults, then native defaults.
        // WSL2's virtualized networking adds significant jitter — use relaxed timeouts
        // to avoid false "link lost" spam.
        let heartbeat_ms = std::env::var("HORUS_NET_HEARTBEAT_MS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(if wsl { 200 } else { 50 });

        let missed_threshold = std::env::var("HORUS_NET_MISSED_THRESHOLD")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(if wsl { 5 } else { 3 });

        // `on_link_lost` had no env read at all — it was hard-coded to "warn",
        // so `safety.on_link_lost = "safe_state"` in horus.toml was silently
        // inert and the documented comms-loss safe-state never fired. That is a
        // safety gap, not just a config one.
        let on_link_lost = std::env::var("HORUS_NET_ON_LINK_LOST")
            .ok()
            .map(|v| v.trim().to_lowercase())
            .filter(|v| !v.is_empty())
            .unwrap_or_else(|| "warn".into());

        Self {
            heartbeat_ms,
            missed_threshold,
            on_link_lost,
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
            // These three were hard-coded to their defaults with NO env read,
            // which is why the entire `[network]` section of horus.toml was
            // parsed and then never applied: `import`, `deny_export` and
            // `optimize` had no way to reach the replicator at all, and
            // `ImportMode::Deny` / `AllowList` were unreachable in shipped code
            // — the guard was permanently `Auto`.
            import: ImportConfig::from_env(),
            deny_export: csv_env("HORUS_NET_DENY_EXPORT"),
            safety: SafetyConfig::default(),
            estop_remote: EstopRemotePolicy::from_env(),
            optimizers: csv_env("HORUS_NET_OPTIMIZERS"),
            topic_overrides: std::collections::HashMap::new(),
        }
    }
}

/// Read a comma-separated env var into a list, dropping empty entries.
fn csv_env(key: &str) -> Vec<String> {
    std::env::var(key)
        .map(|v| {
            v.split(',')
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty())
                .collect()
        })
        .unwrap_or_default()
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
            estop_remote: EstopRemotePolicy::Warn,
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

    /// Serialises the tests that touch process-global state.
    ///
    /// Two kinds of global here: the environment, which `SafetyConfig::default()`
    /// and friends read at call time while other tests set `HORUS_NET_*` to
    /// prove the read happens; and `NO_PEERS_PRINTED`, the fire-once latch that
    /// three tests reset. Cargo runs these on 16 threads, so without the lock a
    /// reader intermittently observes another test's value — `cargo test -p
    /// horus_net` failed roughly one run in three, on a different test each
    /// time, which reads as flaky infrastructure rather than the collision it
    /// is.
    ///
    /// The guard is recovered from poisoning: it protects ordering, not data,
    /// so one failing test must not cascade into every other.
    static GLOBAL_STATE_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    fn env_guard() -> std::sync::MutexGuard<'static, ()> {
        GLOBAL_STATE_LOCK.lock().unwrap_or_else(|e| e.into_inner())
    }

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
            estop_remote: EstopRemotePolicy::Warn,
            optimizers: vec![],
            topic_overrides: std::collections::HashMap::new(),
        }
    }

    #[test]
    fn default_config() {
        let _env = env_guard();
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
        let _env = env_guard();
        let config = make_config();
        match config.discovery_mode() {
            DiscoveryMode::Multicast { group } => assert_eq!(group, "224.0.69.72"),
            _ => panic!("expected multicast"),
        }
    }

    #[test]
    fn discovery_mode_unicast() {
        let _env = env_guard();
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
        let _env = env_guard();
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
        let _env = env_guard();
        let mut config = make_config();
        config.peers = vec!["192.168.1.42".into(), "192.168.1.43".into()];
        match config.discovery_mode() {
            DiscoveryMode::Unicast { peers } => assert_eq!(peers.len(), 2),
            _ => panic!("expected unicast"),
        }
    }

    #[test]
    fn secret_hash_none() {
        let _env = env_guard();
        let config = make_config();
        assert_eq!(config.secret_hash(), [0u8; 4]);
    }

    #[test]
    fn secret_hash_some() {
        let _env = env_guard();
        let mut config = make_config();
        config.secret = Some("lab-42".into());
        assert_ne!(config.secret_hash(), [0u8; 4]);
    }

    #[test]
    fn topic_config_exact_match() {
        let _env = env_guard();
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
        let _env = env_guard();
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
        let _env = env_guard();
        let config = make_config();
        assert_eq!(config.safety.heartbeat_ms, 50);
        assert_eq!(config.safety.missed_threshold, 3);
        assert_eq!(config.safety.on_link_lost, "warn");
    }

    #[test]
    fn no_peers_diagnostic_fires_once() {
        let _env = env_guard();
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
        let _env = env_guard();
        reset_no_peers_diagnostic();
        let start = Instant::now() - Duration::from_secs(10);

        let msg = check_no_peers_diagnostic(1, start, Duration::from_secs(5));
        assert!(msg.is_none());

        reset_no_peers_diagnostic();
    }

    #[test]
    fn no_peers_diagnostic_suppressed_before_timeout() {
        let _env = env_guard();
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

    // ─── horus.toml [network] wiring ────────────────────────────────────────
    //
    // These fields had NO env read at all, which is why the entire [network]
    // section of horus.toml was parsed and then silently discarded. `parse` is
    // tested directly rather than through env vars so the cases are independent
    // of process-global state.

    #[test]
    fn import_spec_modes() {
        assert_eq!(ImportConfig::parse("auto"), ImportConfig::Auto);
        assert_eq!(ImportConfig::parse("AUTO"), ImportConfig::Auto);
        assert_eq!(ImportConfig::parse(""), ImportConfig::Auto);
        assert_eq!(ImportConfig::parse("   "), ImportConfig::Auto);

        assert_eq!(ImportConfig::parse("deny"), ImportConfig::Deny);
        assert_eq!(ImportConfig::parse("none"), ImportConfig::Deny);
        assert_eq!(ImportConfig::parse("off"), ImportConfig::Deny);
    }

    #[test]
    fn import_spec_allowlist() {
        assert_eq!(
            ImportConfig::parse("cmd_vel, robot.imu ,tf"),
            ImportConfig::AllowList(vec!["cmd_vel".into(), "robot.imu".into(), "tf".into()])
        );
    }

    #[test]
    fn import_spec_that_parses_to_nothing_denies_rather_than_allows() {
        // An operator writing an import restriction that ends up empty must not
        // silently get "import everything".
        assert_eq!(ImportConfig::parse(",,,"), ImportConfig::Deny);
        assert_eq!(ImportConfig::parse(" , "), ImportConfig::Deny);
    }

    #[test]
    fn on_link_lost_is_no_longer_hardcoded_to_warn() {
        let _env = env_guard();
        // The safety-critical one: this field had no env read, so
        // `safety.on_link_lost = "safe_state"` in horus.toml never took effect
        // and the documented comms-loss safe-state could not fire.
        let restore = std::env::var("HORUS_NET_ON_LINK_LOST").ok();
        std::env::set_var("HORUS_NET_ON_LINK_LOST", "safe_state");
        assert_eq!(SafetyConfig::default().on_link_lost, "safe_state");

        std::env::set_var("HORUS_NET_ON_LINK_LOST", "  STOP  ");
        assert_eq!(
            SafetyConfig::default().on_link_lost,
            "stop",
            "value must be trimmed and lowercased for LinkLostAction::from_str"
        );

        std::env::remove_var("HORUS_NET_ON_LINK_LOST");
        assert_eq!(
            SafetyConfig::default().on_link_lost,
            "warn",
            "default is still warn when unset"
        );
        if let Some(v) = restore {
            std::env::set_var("HORUS_NET_ON_LINK_LOST", v);
        }
    }

    #[test]
    fn on_link_lost_maps_to_a_real_action() {
        // Guard against the value being read but then not recognised — the
        // string has to survive all the way into LinkLostAction. Note an
        // unknown string falls back to Warn, which is why safe_state and stop
        // are what actually prove the plumbing works.
        use crate::heartbeat::LinkLostAction;
        assert_eq!(LinkLostAction::from_str("warn"), LinkLostAction::Warn);
        assert_eq!(
            LinkLostAction::from_str("safe_state"),
            LinkLostAction::SafeState,
            "the safety-critical value must reach the action, not fall back to Warn"
        );
        assert_eq!(LinkLostAction::from_str("stop"), LinkLostAction::Stop);
        assert_eq!(
            LinkLostAction::from_str("definitely-not-an-action"),
            LinkLostAction::Warn,
            "unknown values fall back to the least-drastic action"
        );
    }

    #[test]
    fn on_link_lost_survives_env_to_action_end_to_end() {
        let _env = env_guard();
        // The gap was between the env var and the action. Walk the whole path.
        use crate::heartbeat::LinkLostAction;
        let restore = std::env::var("HORUS_NET_ON_LINK_LOST").ok();
        std::env::set_var("HORUS_NET_ON_LINK_LOST", "safe_state");
        let cfg = SafetyConfig::default();
        assert_eq!(
            LinkLostAction::from_str(&cfg.on_link_lost),
            LinkLostAction::SafeState
        );
        std::env::remove_var("HORUS_NET_ON_LINK_LOST");
        if let Some(v) = restore {
            std::env::set_var("HORUS_NET_ON_LINK_LOST", v);
        }
    }

    #[test]
    fn csv_env_splits_and_trims() {
        let _env = env_guard();
        let restore = std::env::var("HORUS_NET_DENY_EXPORT").ok();
        std::env::set_var("HORUS_NET_DENY_EXPORT", "camera.*, debug.* ,");
        assert_eq!(
            csv_env("HORUS_NET_DENY_EXPORT"),
            vec!["camera.*".to_string(), "debug.*".to_string()]
        );
        std::env::remove_var("HORUS_NET_DENY_EXPORT");
        assert!(csv_env("HORUS_NET_DENY_EXPORT").is_empty());
        if let Some(v) = restore {
            std::env::set_var("HORUS_NET_DENY_EXPORT", v);
        }
    }
}
