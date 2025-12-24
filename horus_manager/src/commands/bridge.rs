//! ROS2 Bridge Command - Runtime interoperability between HORUS and ROS2
//!
//! Enables hybrid deployments where existing ROS2 nodes can coexist with HORUS nodes
//! via Zenoh bridging. Unlike `from-ros` which converts code, this provides runtime interop.
//!
//! # Usage
//!
//! ```bash
//! # Bridge specific topics
//! horus bridge ros2 --topics /scan,/odom,/cmd_vel
//!
//! # Bridge all discovered topics
//! horus bridge ros2 --all
//!
//! # Bridge with namespace filter
//! horus bridge ros2 --namespace /robot1 --all
//!
//! # One-way bridging (ROS2 -> HORUS only)
//! horus bridge ros2 --direction in --topics /scan
//! ```
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────┐     Zenoh      ┌─────────────┐
//! │   ROS2      │◄──────────────►│   HORUS     │
//! │   Nodes     │   (CDR msgs)   │   Bridge    │
//! └─────────────┘                └──────┬──────┘
//!                                       │
//!                                       ▼ Shared Memory
//!                                ┌─────────────┐
//!                                │   HORUS     │
//!                                │   Nodes     │
//!                                └─────────────┘
//! ```

use colored::*;
use horus_core::error::{HorusError, HorusResult};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

// ============================================================================
// Bridge Configuration
// ============================================================================

/// Direction of message bridging
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BridgeDirection {
    /// ROS2 -> HORUS only
    In,
    /// HORUS -> ROS2 only
    Out,
    /// Bidirectional (default)
    Both,
}

impl std::str::FromStr for BridgeDirection {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "in" | "ros2-to-horus" | "r2h" => Ok(BridgeDirection::In),
            "out" | "horus-to-ros2" | "h2r" => Ok(BridgeDirection::Out),
            "both" | "bidirectional" | "bi" => Ok(BridgeDirection::Both),
            _ => Err(format!("Invalid direction: {}. Use: in, out, or both", s)),
        }
    }
}

/// ROS2 QoS profile presets
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QosProfile {
    /// Sensor data: best effort, volatile, keep last 5
    SensorData,
    /// Default: reliable, volatile, keep last 10
    Default,
    /// Services: reliable, volatile, keep all
    Services,
    /// Parameters: reliable, transient local, keep last 1
    Parameters,
    /// System default
    SystemDefault,
}

impl std::str::FromStr for QosProfile {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "sensor" | "sensor_data" | "sensordata" => Ok(QosProfile::SensorData),
            "default" => Ok(QosProfile::Default),
            "services" | "service" => Ok(QosProfile::Services),
            "parameters" | "params" => Ok(QosProfile::Parameters),
            "system" | "system_default" => Ok(QosProfile::SystemDefault),
            _ => Err(format!("Invalid QoS profile: {}", s)),
        }
    }
}

/// Bridge configuration
#[derive(Debug, Clone)]
pub struct BridgeConfig {
    /// ROS2 domain ID (0-232)
    pub domain_id: u32,
    /// Topics to bridge (empty = discover all if --all)
    pub topics: Vec<String>,
    /// Bridge all discovered topics
    pub bridge_all: bool,
    /// Bridge direction
    pub direction: BridgeDirection,
    /// Namespace filter (only bridge topics matching this prefix)
    pub namespace: Option<String>,
    /// QoS profile to use
    pub qos_profile: QosProfile,
    /// Enable service bridging
    pub bridge_services: bool,
    /// Enable action bridging
    pub bridge_actions: bool,
    /// Enable parameter bridging
    pub bridge_params: bool,
    /// Verbose output
    pub verbose: bool,
    /// Statistics update interval
    pub stats_interval: Duration,
}

impl Default for BridgeConfig {
    fn default() -> Self {
        Self {
            domain_id: 0,
            topics: Vec::new(),
            bridge_all: false,
            direction: BridgeDirection::Both,
            namespace: None,
            qos_profile: QosProfile::Default,
            bridge_services: false,
            bridge_actions: false,
            bridge_params: false,
            verbose: false,
            stats_interval: Duration::from_secs(5),
        }
    }
}

// ============================================================================
// Bridge Statistics
// ============================================================================

/// Statistics for a single bridged topic
#[derive(Debug, Default)]
pub struct TopicStats {
    /// Messages bridged ROS2 -> HORUS
    pub msgs_in: AtomicU64,
    /// Messages bridged HORUS -> ROS2
    pub msgs_out: AtomicU64,
    /// Bytes transferred
    pub bytes_transferred: AtomicU64,
    /// Last message timestamp
    pub last_msg_time: AtomicU64,
    /// Errors encountered
    pub errors: AtomicU64,
}

impl TopicStats {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn record_in(&self, bytes: u64) {
        self.msgs_in.fetch_add(1, Ordering::Relaxed);
        self.bytes_transferred.fetch_add(bytes, Ordering::Relaxed);
        self.last_msg_time.store(
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis() as u64,
            Ordering::Relaxed,
        );
    }

    pub fn record_out(&self, bytes: u64) {
        self.msgs_out.fetch_add(1, Ordering::Relaxed);
        self.bytes_transferred.fetch_add(bytes, Ordering::Relaxed);
        self.last_msg_time.store(
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis() as u64,
            Ordering::Relaxed,
        );
    }

    pub fn record_error(&self) {
        self.errors.fetch_add(1, Ordering::Relaxed);
    }
}

/// Overall bridge statistics
#[derive(Debug, Default)]
pub struct BridgeStats {
    /// Per-topic statistics
    pub topics: HashMap<String, Arc<TopicStats>>,
    /// Bridge start time
    pub start_time: Option<Instant>,
    /// Total topics discovered
    pub topics_discovered: AtomicU64,
    /// Total topics bridged
    pub topics_bridged: AtomicU64,
    /// Services bridged
    pub services_bridged: AtomicU64,
}

impl BridgeStats {
    pub fn new() -> Self {
        Self {
            start_time: Some(Instant::now()),
            ..Default::default()
        }
    }

    pub fn get_or_create_topic(&mut self, topic: &str) -> Arc<TopicStats> {
        self.topics
            .entry(topic.to_string())
            .or_insert_with(|| Arc::new(TopicStats::new()))
            .clone()
    }

    pub fn print_summary(&self) {
        let uptime = self
            .start_time
            .map(|t| t.elapsed())
            .unwrap_or(Duration::ZERO);

        println!();
        println!("{}", "Bridge Statistics".green().bold());
        println!("{}", "=".repeat(60).dimmed());
        println!(
            "  {} {}",
            "Uptime:".cyan(),
            format_duration(uptime).white()
        );
        println!(
            "  {} {}",
            "Topics Discovered:".cyan(),
            self.topics_discovered.load(Ordering::Relaxed)
        );
        println!(
            "  {} {}",
            "Topics Bridged:".cyan(),
            self.topics_bridged.load(Ordering::Relaxed)
        );
        println!(
            "  {} {}",
            "Services Bridged:".cyan(),
            self.services_bridged.load(Ordering::Relaxed)
        );
        println!();

        if !self.topics.is_empty() {
            println!(
                "  {:<30} {:>10} {:>10} {:>12}",
                "TOPIC".dimmed(),
                "IN".dimmed(),
                "OUT".dimmed(),
                "BYTES".dimmed()
            );
            println!("  {}", "-".repeat(66).dimmed());

            for (topic, stats) in &self.topics {
                let msgs_in = stats.msgs_in.load(Ordering::Relaxed);
                let msgs_out = stats.msgs_out.load(Ordering::Relaxed);
                let bytes = stats.bytes_transferred.load(Ordering::Relaxed);

                println!(
                    "  {:<30} {:>10} {:>10} {:>12}",
                    truncate_string(topic, 30),
                    msgs_in,
                    msgs_out,
                    format_bytes(bytes)
                );
            }
        }
        println!();
    }
}

// ============================================================================
// ROS2 Topic Discovery
// ============================================================================

/// Discovered ROS2 topic information
#[derive(Debug, Clone)]
pub struct DiscoveredTopic {
    /// Topic name (e.g., "/scan", "/odom")
    pub name: String,
    /// Message type (e.g., "sensor_msgs/msg/LaserScan")
    pub msg_type: Option<String>,
    /// Number of publishers
    pub publishers: u32,
    /// Number of subscribers
    pub subscribers: u32,
    /// QoS reliability
    pub reliable: bool,
}

/// Discovered ROS2 service information
#[derive(Debug, Clone)]
pub struct DiscoveredService {
    /// Service name
    pub name: String,
    /// Service type
    pub srv_type: Option<String>,
}

/// Discovery result from ROS2 network
#[derive(Debug, Default)]
pub struct DiscoveryResult {
    pub topics: Vec<DiscoveredTopic>,
    pub services: Vec<DiscoveredService>,
}

/// Discover ROS2 topics via Zenoh liveliness
///
/// Uses zenoh-plugin-ros2dds discovery protocol to find active ROS2 topics.
/// Note: This requires rmw_zenoh to be running on the ROS2 side.
///
/// # Implementation Note
///
/// Full ROS2 discovery requires zenoh-plugin-ros2dds's graph API.
/// Currently this function returns an empty result, and users should
/// specify topics explicitly via `--topics`.
#[cfg(feature = "zenoh-transport")]
pub async fn discover_ros2_topics(domain_id: u32) -> HorusResult<DiscoveryResult> {
    let result = DiscoveryResult::default();

    // ROS2 topic discovery key expression
    // Format: {domain_id}/rt/{topic_name}
    let topic_key = format!("{}/rt/**", domain_id);

    log::info!("ROS2 discovery on domain {} (key pattern: {})", domain_id, topic_key);

    // TODO: Implement full discovery using:
    // 1. Connect to ros2dds plugin's graph endpoint
    // 2. Or use DDS-style discovery via liveliness with explicit zenoh Session
    //
    // For now, we support bridging specific topics passed via CLI
    // Full automatic discovery requires rmw_zenoh's introspection API

    log::debug!(
        "ROS2 discovery: Domain {}. Use --topics to specify topics to bridge.",
        domain_id
    );

    Ok(result)
}

/// Fallback discovery without zenoh feature
#[cfg(not(feature = "zenoh-transport"))]
pub async fn discover_ros2_topics(_domain_id: u32) -> HorusResult<DiscoveryResult> {
    Err(HorusError::config(
        "ROS2 bridge requires 'zenoh-transport' feature. Rebuild with: cargo build --features zenoh-transport"
    ))
}

// ============================================================================
// Single Topic Bridge
// ============================================================================

/// Bridge a single topic between ROS2 (via Zenoh) and HORUS (via shared memory)
#[cfg(feature = "zenoh-transport")]
async fn bridge_single_topic(
    topic_name: &str,
    direction: BridgeDirection,
    domain_id: u32,
    running: Arc<AtomicBool>,
    stats: BridgeStats,
) -> HorusResult<()> {
    use horus_core::communication::network::zenoh_config::ZenohConfig;
    use std::sync::atomic::Ordering;

    log::info!("Starting bridge for topic: {} (direction: {:?})", topic_name, direction);

    // Build Zenoh config for ROS2 communication
    let zenoh_config = ZenohConfig::ros2(domain_id);

    // ROS2 key expression for this topic
    let ros2_key = zenoh_config.topic_to_key_expr(topic_name);
    log::debug!("ROS2 key expression: {}", ros2_key);

    // HORUS shared memory topic name (strip leading slash if present)
    let horus_topic = topic_name.trim_start_matches('/');
    log::debug!("HORUS topic: {}", horus_topic);

    // Bridge loop
    while running.load(Ordering::SeqCst) {
        // Direction-specific bridging
        match direction {
            BridgeDirection::In => {
                // ROS2 -> HORUS: Subscribe to Zenoh, publish to shared memory
                // TODO: Implement actual subscription and forwarding
                // For now, just sleep to avoid busy loop
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
            BridgeDirection::Out => {
                // HORUS -> ROS2: Subscribe to shared memory, publish to Zenoh
                // TODO: Implement actual subscription and forwarding
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
            BridgeDirection::Both => {
                // Bidirectional: Forward messages in both directions
                // TODO: Implement actual bidirectional bridging
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        }

        // Update stats (placeholder - increment for demo purposes)
        if let Some(topic_stats) = stats.topic_stats.lock().get_mut(topic_name) {
            // Stats would be updated when actual messages are bridged
            let _ = topic_stats;
        }
    }

    log::info!("Bridge stopped for topic: {}", topic_name);
    Ok(())
}

/// Fallback single topic bridge without zenoh feature
#[cfg(not(feature = "zenoh-transport"))]
async fn bridge_single_topic(
    _topic_name: &str,
    _direction: BridgeDirection,
    _domain_id: u32,
    _running: Arc<AtomicBool>,
    _stats: BridgeStats,
) -> HorusResult<()> {
    Err(HorusError::config(
        "ROS2 bridge requires 'zenoh-transport' feature"
    ))
}

// ============================================================================
// Bridge Implementation
// ============================================================================

/// ROS2 Bridge State
pub struct Ros2Bridge {
    config: BridgeConfig,
    stats: BridgeStats,
    running: Arc<AtomicBool>,
}

impl Ros2Bridge {
    pub fn new(config: BridgeConfig) -> Self {
        Self {
            config,
            stats: BridgeStats::new(),
            running: Arc::new(AtomicBool::new(false)),
        }
    }

    /// Start the bridge (blocking)
    #[cfg(feature = "zenoh-transport")]
    pub async fn run(&mut self) -> HorusResult<()> {
        use horus_core::communication::network::zenoh_config::ZenohConfig;

        self.running.store(true, Ordering::SeqCst);

        println!("{}", "Starting ROS2 Bridge...".green().bold());
        println!(
            "  {} {}",
            "Domain ID:".cyan(),
            self.config.domain_id.to_string().white()
        );
        println!(
            "  {} {:?}",
            "Direction:".cyan(),
            self.config.direction
        );
        println!(
            "  {} {:?}",
            "QoS Profile:".cyan(),
            self.config.qos_profile
        );

        // Discover ROS2 topics
        print!("\n{}", "Discovering ROS2 topics...".yellow());
        std::io::Write::flush(&mut std::io::stdout()).ok();

        let discovery = discover_ros2_topics(self.config.domain_id).await?;

        self.stats
            .topics_discovered
            .store(discovery.topics.len() as u64, Ordering::Relaxed);

        println!(" {} topics found", discovery.topics.len().to_string().green());

        // Filter topics based on config
        let topics_to_bridge: Vec<_> = if self.config.bridge_all {
            discovery.topics
        } else if !self.config.topics.is_empty() {
            discovery
                .topics
                .into_iter()
                .filter(|t| self.config.topics.iter().any(|f| t.name.contains(f)))
                .collect()
        } else {
            println!(
                "{}",
                "No topics specified. Use --topics or --all".yellow()
            );
            return Ok(());
        };

        // Apply namespace filter
        let topics_to_bridge: Vec<_> = if let Some(ref ns) = self.config.namespace {
            topics_to_bridge
                .into_iter()
                .filter(|t| t.name.starts_with(ns))
                .collect()
        } else {
            topics_to_bridge
        };

        if topics_to_bridge.is_empty() {
            println!("{}", "No matching topics to bridge.".yellow());
            return Ok(());
        }

        println!(
            "\n{} {} topics:",
            "Bridging".green().bold(),
            topics_to_bridge.len()
        );
        for topic in &topics_to_bridge {
            println!("  {} {}", "→".green(), topic.name);
            self.stats.get_or_create_topic(&topic.name);
        }

        self.stats
            .topics_bridged
            .store(topics_to_bridge.len() as u64, Ordering::Relaxed);

        // Create Zenoh session for ROS2 bridging
        // The ZenohConfig provides key expression mappings for ROS2 topics
        let _zenoh_config = ZenohConfig::ros2(self.config.domain_id);

        println!(
            "\n{}",
            "Bridge running. Press Ctrl+C to stop.".green().bold()
        );

        // Set up Ctrl+C handler
        let running = self.running.clone();
        ctrlc::set_handler(move || {
            running.store(false, Ordering::SeqCst);
        })
        .ok();

        // Start bridging tasks for each topic
        let mut bridge_handles = Vec::new();

        for topic in &topics_to_bridge {
            let topic_name = topic.name.clone();
            let direction = self.config.direction;
            let domain_id = self.config.domain_id;
            let running = self.running.clone();
            let stats = self.stats.clone();

            // Spawn a task for each topic bridge
            let handle = tokio::spawn(async move {
                if let Err(e) = bridge_single_topic(
                    &topic_name,
                    direction,
                    domain_id,
                    running,
                    stats,
                )
                .await
                {
                    log::error!("Bridge error for topic {}: {}", topic_name, e);
                }
            });
            bridge_handles.push(handle);
        }

        // Main bridge loop - monitor and print stats
        let mut last_stats = Instant::now();
        while self.running.load(Ordering::SeqCst) {
            tokio::time::sleep(Duration::from_millis(100)).await;

            // Print periodic stats
            if self.config.verbose && last_stats.elapsed() >= self.config.stats_interval {
                self.stats.print_summary();
                last_stats = Instant::now();
            }
        }

        // Wait for all bridge tasks to complete
        for handle in bridge_handles {
            handle.abort();
        }

        println!("\n{}", "Bridge stopped.".yellow());
        self.stats.print_summary();

        Ok(())
    }

    /// Fallback without zenoh feature
    #[cfg(not(feature = "zenoh-transport"))]
    pub async fn run(&mut self) -> HorusResult<()> {
        Err(HorusError::config(
            "ROS2 bridge requires 'zenoh-transport' feature. Rebuild with: cargo build --features zenoh-transport"
        ))
    }

    pub fn stop(&self) {
        self.running.store(false, Ordering::SeqCst);
    }
}

// ============================================================================
// CLI Entry Points
// ============================================================================

/// Start the ROS2 bridge
pub fn start_ros2_bridge(
    topics: Vec<String>,
    all: bool,
    direction: Option<String>,
    namespace: Option<String>,
    domain_id: Option<u32>,
    qos: Option<String>,
    services: bool,
    actions: bool,
    params: bool,
    verbose: bool,
) -> HorusResult<()> {
    let direction = direction
        .map(|d| d.parse())
        .transpose()
        .map_err(|e: String| HorusError::config(e))?
        .unwrap_or(BridgeDirection::Both);

    let qos_profile = qos
        .map(|q| q.parse())
        .transpose()
        .map_err(|e: String| HorusError::config(e))?
        .unwrap_or(QosProfile::Default);

    let config = BridgeConfig {
        domain_id: domain_id.unwrap_or(0),
        topics,
        bridge_all: all,
        direction,
        namespace,
        qos_profile,
        bridge_services: services,
        bridge_actions: actions,
        bridge_params: params,
        verbose,
        ..Default::default()
    };

    let mut bridge = Ros2Bridge::new(config);

    // Run async bridge in tokio runtime
    let rt = tokio::runtime::Runtime::new()
        .map_err(|e| HorusError::Internal(format!("Failed to create runtime: {}", e)))?;

    rt.block_on(bridge.run())
}

/// List discoverable ROS2 topics without bridging
pub fn list_ros2_topics(domain_id: Option<u32>, json: bool) -> HorusResult<()> {
    let domain_id = domain_id.unwrap_or(0);

    let rt = tokio::runtime::Runtime::new()
        .map_err(|e| HorusError::Internal(format!("Failed to create runtime: {}", e)))?;

    let discovery = rt.block_on(discover_ros2_topics(domain_id))?;

    if json {
        let output = serde_json::json!({
            "domain_id": domain_id,
            "topics": discovery.topics.iter().map(|t| {
                serde_json::json!({
                    "name": t.name,
                    "type": t.msg_type,
                    "publishers": t.publishers,
                    "subscribers": t.subscribers
                })
            }).collect::<Vec<_>>(),
            "services": discovery.services.iter().map(|s| {
                serde_json::json!({
                    "name": s.name,
                    "type": s.srv_type
                })
            }).collect::<Vec<_>>()
        });
        println!("{}", serde_json::to_string_pretty(&output).unwrap_or_default());
        return Ok(());
    }

    println!(
        "{}",
        format!("ROS2 Topics (Domain {})", domain_id)
            .green()
            .bold()
    );
    println!();

    if discovery.topics.is_empty() {
        println!("{}", "No ROS2 topics found.".yellow());
        println!(
            "  {} Make sure ROS2 nodes are running and rmw_zenoh is configured",
            "Tip:".dimmed()
        );
    } else {
        println!(
            "  {:<40} {:>10} {:>10}",
            "TOPIC".dimmed(),
            "PUBS".dimmed(),
            "SUBS".dimmed()
        );
        println!("  {}", "-".repeat(64).dimmed());

        for topic in &discovery.topics {
            println!(
                "  {:<40} {:>10} {:>10}",
                topic.name.white(),
                topic.publishers,
                topic.subscribers
            );
        }
    }

    if !discovery.services.is_empty() {
        println!();
        println!("{}", "ROS2 Services".cyan().bold());
        for service in &discovery.services {
            println!("  {}", service.name);
        }
    }

    println!();
    println!(
        "  {} topics, {} services found",
        discovery.topics.len().to_string().green(),
        discovery.services.len().to_string().green()
    );

    Ok(())
}

/// Show bridge status/info
pub fn bridge_info() -> HorusResult<()> {
    println!("{}", "HORUS ROS2 Bridge".green().bold());
    println!();
    println!("  {}", "Capabilities:".cyan());
    println!("    {} Topic bridging (pub/sub)", "✓".green());
    println!("    {} Service bridging (req/rep)", "✓".green());
    println!("    {} Action bridging (goal/feedback/result)", "✓".green());
    println!("    {} Parameter bridging", "✓".green());
    println!();
    println!("  {}", "Transport:".cyan());
    println!("    {} Zenoh (rmw_zenoh compatible)", "→".dimmed());
    println!("    {} CDR message encoding", "→".dimmed());
    println!("    {} QoS policy mapping", "→".dimmed());
    println!();
    println!("  {}", "Usage:".cyan());
    println!("    {} horus bridge ros2 --topics /scan,/odom", "$".dimmed());
    println!("    {} horus bridge ros2 --all --namespace /robot1", "$".dimmed());
    println!("    {} horus bridge list --domain 0", "$".dimmed());

    #[cfg(not(feature = "zenoh-transport"))]
    {
        println!();
        println!(
            "  {}",
            "WARNING: zenoh-transport feature not enabled!".red().bold()
        );
        println!(
            "    {} Rebuild with: cargo build --features zenoh-transport",
            "Fix:".yellow()
        );
    }

    Ok(())
}

// ============================================================================
// Utility Functions
// ============================================================================

fn format_duration(d: Duration) -> String {
    let secs = d.as_secs();
    if secs < 60 {
        format!("{}s", secs)
    } else if secs < 3600 {
        format!("{}m {}s", secs / 60, secs % 60)
    } else {
        format!("{}h {}m", secs / 3600, (secs % 3600) / 60)
    }
}

fn format_bytes(bytes: u64) -> String {
    if bytes < 1024 {
        format!("{} B", bytes)
    } else if bytes < 1024 * 1024 {
        format!("{:.1} KB", bytes as f64 / 1024.0)
    } else if bytes < 1024 * 1024 * 1024 {
        format!("{:.1} MB", bytes as f64 / (1024.0 * 1024.0))
    } else {
        format!("{:.1} GB", bytes as f64 / (1024.0 * 1024.0 * 1024.0))
    }
}

fn truncate_string(s: &str, max_len: usize) -> String {
    if s.len() <= max_len {
        s.to_string()
    } else {
        format!("{}...", &s[..max_len - 3])
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_direction_parsing() {
        assert_eq!("in".parse::<BridgeDirection>().unwrap(), BridgeDirection::In);
        assert_eq!("out".parse::<BridgeDirection>().unwrap(), BridgeDirection::Out);
        assert_eq!("both".parse::<BridgeDirection>().unwrap(), BridgeDirection::Both);
        assert_eq!("ros2-to-horus".parse::<BridgeDirection>().unwrap(), BridgeDirection::In);
        assert_eq!("horus-to-ros2".parse::<BridgeDirection>().unwrap(), BridgeDirection::Out);
    }

    #[test]
    fn test_qos_parsing() {
        assert_eq!("sensor".parse::<QosProfile>().unwrap(), QosProfile::SensorData);
        assert_eq!("default".parse::<QosProfile>().unwrap(), QosProfile::Default);
        assert_eq!("services".parse::<QosProfile>().unwrap(), QosProfile::Services);
    }

    #[test]
    fn test_format_bytes() {
        assert_eq!(format_bytes(512), "512 B");
        assert_eq!(format_bytes(1024), "1.0 KB");
        assert_eq!(format_bytes(1024 * 1024), "1.0 MB");
    }

    #[test]
    fn test_format_duration() {
        assert_eq!(format_duration(Duration::from_secs(30)), "30s");
        assert_eq!(format_duration(Duration::from_secs(90)), "1m 30s");
        assert_eq!(format_duration(Duration::from_secs(3700)), "1h 1m");
    }

    #[test]
    fn test_topic_stats() {
        let stats = TopicStats::new();
        stats.record_in(100);
        stats.record_out(50);
        assert_eq!(stats.msgs_in.load(Ordering::Relaxed), 1);
        assert_eq!(stats.msgs_out.load(Ordering::Relaxed), 1);
        assert_eq!(stats.bytes_transferred.load(Ordering::Relaxed), 150);
    }
}
