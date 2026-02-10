//! Offline Profiling System for Deterministic Execution
//!
//! This module provides offline profiling capabilities that allow:
//! 1. Profile once in controlled environment
//! 2. Save profile to file
//! 3. Load profile at runtime for deterministic, optimized execution
//!
//! This replaces the non-deterministic runtime learning phase.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::path::Path;
use std::time::Duration;

/// Node tier for explicit annotation by developers
///
/// Use this to declare a node's execution characteristics at compile time,
/// avoiding the non-deterministic runtime learning phase.
///
/// # Example
/// ```ignore
/// use horus_core::scheduling::{NodeTier, Scheduler};
///
/// let mut scheduler = Scheduler::new();
/// scheduler.add(pid_node).order(0).tier(NodeTier::UltraFast).done();
/// scheduler.add(sensor_node).order(1).tier(NodeTier::Fast).done();
/// scheduler.add(logger_node).order(5).tier(NodeTier::Background).done();
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum NodeTier {
    /// Ultra-fast nodes (<1μs) - inline execution for maximum speed
    /// Use for: PID controllers, simple math, data transformations
    UltraFast,

    /// Fast nodes (<10μs) - Inline execution with minimal overhead
    /// Use for: Sensor readers, filter calculations, state machines
    #[default]
    Fast,

    /// Normal nodes (<100μs) - Standard scheduling
    /// Use for: Complex algorithms, data processing
    Normal,

    /// Async I/O nodes - Non-blocking execution
    /// Use for: Network communication, file I/O, cloud sync
    AsyncIO,

    /// Background nodes (>100μs) - Low-priority thread
    /// Use for: Logging, diagnostics, non-critical tasks
    Background,

    /// Isolated nodes - Process isolation for fault tolerance
    /// Use for: Untrusted code, high-failure-rate nodes
    Isolated,

    /// Auto-detect tier from profile or runtime characteristics
    /// Only use if you have a profile file or explicitly enable learning
    Auto,
}

impl NodeTier {
    /// Get human-readable description
    pub fn description(&self) -> &'static str {
        match self {
            NodeTier::UltraFast => "Ultra-fast inline (<1μs)",
            NodeTier::Fast => "Inline execution (<10μs)",
            NodeTier::Normal => "Standard scheduling (<100μs)",
            NodeTier::AsyncIO => "Async I/O (non-blocking)",
            NodeTier::Background => "Background thread (>100μs)",
            NodeTier::Isolated => "Process isolation (fault tolerance)",
            NodeTier::Auto => "Auto-detect from profile",
        }
    }
}

/// Statistics for a profiled node
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeProfile {
    /// Node name
    pub name: String,

    /// Recommended tier based on profiling
    pub tier: NodeTier,

    /// Average execution time in microseconds
    pub avg_us: f64,

    /// Standard deviation in microseconds
    pub stddev_us: f64,

    /// Minimum execution time observed
    pub min_us: f64,

    /// Maximum execution time observed
    pub max_us: f64,

    /// Number of samples collected
    pub sample_count: usize,

    /// Is execution time deterministic? (low variance)
    pub is_deterministic: bool,

    /// Is this node I/O heavy?
    pub is_io_heavy: bool,

    /// Is this node CPU bound?
    pub is_cpu_bound: bool,
}

impl NodeProfile {
    /// Create a new node profile with explicit tier
    pub fn new(name: &str, tier: NodeTier) -> Self {
        Self {
            name: name.to_string(),
            tier,
            avg_us: 0.0,
            stddev_us: 0.0,
            min_us: 0.0,
            max_us: 0.0,
            sample_count: 0,
            is_deterministic: true,
            is_io_heavy: false,
            is_cpu_bound: false,
        }
    }

    /// Classify tier based on measured statistics
    pub fn classify_tier(&mut self) {
        // Coefficient of variation for determinism check
        let cv = if self.avg_us > 0.0 {
            self.stddev_us / self.avg_us
        } else {
            0.0
        };

        self.is_deterministic = cv < 0.10;
        self.is_io_heavy = cv > 0.30 && self.max_us > self.avg_us * 2.0;
        self.is_cpu_bound = self.avg_us > 100.0 && cv < 0.20;

        // Auto-classify tier based on characteristics
        self.tier = if self.is_io_heavy {
            NodeTier::AsyncIO
        } else if self.avg_us < 1.0 && self.is_deterministic {
            NodeTier::UltraFast
        } else if self.avg_us < 10.0 {
            NodeTier::Fast
        } else if self.avg_us < 100.0 {
            NodeTier::Normal
        } else {
            NodeTier::Background
        };
    }
}

/// Profile data containing all node profiles
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProfileData {
    /// Version for compatibility checking
    pub version: u32,

    /// Profile name/description
    pub name: String,

    /// When the profile was created (Unix timestamp)
    pub created_at: u64,

    /// Number of profiling ticks used
    pub profiling_ticks: usize,

    /// Node profiles by name
    pub nodes: HashMap<String, NodeProfile>,

    /// Global scheduler tick rate used during profiling
    pub tick_rate_hz: f64,

    /// Hardware info for profile portability warnings
    pub hardware_info: HardwareInfo,
}

/// Hardware information for profile portability
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct HardwareInfo {
    pub cpu_model: String,
    pub cpu_cores: usize,
    pub cpu_freq_mhz: u64,
    pub os: String,
}

impl HardwareInfo {
    /// Collect current hardware info
    pub fn current() -> Self {
        let (cpu_model, cpu_freq_mhz) = Self::read_cpu_info();
        Self {
            cpu_model,
            cpu_cores: num_cpus::get(),
            cpu_freq_mhz,
            os: std::env::consts::OS.to_string(),
        }
    }

    /// Read CPU model and frequency from /proc/cpuinfo (Linux) or return defaults
    #[cfg(target_os = "linux")]
    fn read_cpu_info() -> (String, u64) {
        use std::fs;
        let mut model = "Unknown".to_string();
        let mut freq_mhz = 0u64;

        if let Ok(contents) = fs::read_to_string("/proc/cpuinfo") {
            for line in contents.lines() {
                if line.starts_with("model name") {
                    if let Some(value) = line.split(':').nth(1) {
                        model = value.trim().to_string();
                    }
                } else if line.starts_with("cpu MHz") {
                    if let Some(value) = line.split(':').nth(1) {
                        if let Ok(mhz) = value.trim().parse::<f64>() {
                            freq_mhz = mhz as u64;
                        }
                    }
                }
                // Stop after finding both values
                if model != "Unknown" && freq_mhz > 0 {
                    break;
                }
            }
        }
        (model, freq_mhz)
    }

    #[cfg(not(target_os = "linux"))]
    fn read_cpu_info() -> (String, u64) {
        ("Unknown".to_string(), 0)
    }

    /// Check if profiles are portable (same core count)
    pub fn is_compatible(&self, other: &HardwareInfo) -> bool {
        self.cpu_cores == other.cpu_cores
    }
}

impl ProfileData {
    /// Create a new empty profile
    pub fn new(name: &str) -> Self {
        Self {
            version: 1,
            name: name.to_string(),
            created_at: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs(),
            profiling_ticks: 0,
            nodes: HashMap::new(),
            tick_rate_hz: 60.0,
            hardware_info: HardwareInfo::current(),
        }
    }

    /// Add or update a node profile
    pub fn add_node(&mut self, profile: NodeProfile) {
        self.nodes.insert(profile.name.clone(), profile);
    }

    /// Get a node's profile
    pub fn get_node(&self, name: &str) -> Option<&NodeProfile> {
        self.nodes.get(name)
    }

    /// Get tier for a node (or default)
    pub fn get_tier(&self, name: &str) -> NodeTier {
        self.nodes
            .get(name)
            .map(|p| p.tier)
            .unwrap_or(NodeTier::Fast)
    }

    /// Save profile to file (binary format for speed)
    pub fn save<P: AsRef<Path>>(&self, path: P) -> Result<(), ProfileError> {
        let data = bincode::serialize(self).map_err(|e| ProfileError::Serialize(e.to_string()))?;
        fs::write(&path, data).map_err(|e| ProfileError::Io(e.to_string()))?;
        Ok(())
    }

    /// Save profile to file (JSON format for human readability)
    pub fn save_json<P: AsRef<Path>>(&self, path: P) -> Result<(), ProfileError> {
        let json = serde_json::to_string_pretty(self)
            .map_err(|e| ProfileError::Serialize(e.to_string()))?;
        fs::write(&path, json).map_err(|e| ProfileError::Io(e.to_string()))?;
        Ok(())
    }

    /// Load profile from file (auto-detects format)
    pub fn load<P: AsRef<Path>>(path: P) -> Result<Self, ProfileError> {
        let data = fs::read(&path).map_err(|e| ProfileError::Io(e.to_string()))?;

        // Try binary first, then JSON
        if let Ok(profile) = bincode::deserialize(&data) {
            return Ok(profile);
        }

        let json_str =
            String::from_utf8(data).map_err(|e| ProfileError::Deserialize(e.to_string()))?;
        serde_json::from_str(&json_str).map_err(|e| ProfileError::Deserialize(e.to_string()))
    }

    /// Check hardware compatibility and warn if different
    pub fn check_compatibility(&self) -> Vec<String> {
        let current = HardwareInfo::current();
        let mut warnings = Vec::new();

        if !self.hardware_info.is_compatible(&current) {
            warnings.push(format!(
                "Profile was created on {} cores, running on {} cores",
                self.hardware_info.cpu_cores, current.cpu_cores
            ));
        }

        if self.hardware_info.os != current.os {
            warnings.push(format!(
                "Profile was created on {}, running on {}",
                self.hardware_info.os, current.os
            ));
        }

        warnings
    }

    /// Print profile summary
    pub fn print_summary(&self) {
        println!("\n=== Profile: {} ===", self.name);
        println!("Created: {}", self.created_at);
        println!("Profiling ticks: {}", self.profiling_ticks);
        println!("Tick rate: {} Hz", self.tick_rate_hz);
        println!("Nodes: {}", self.nodes.len());

        let warnings = self.check_compatibility();
        if !warnings.is_empty() {
            println!("\nWarnings:");
            for warning in warnings {
                println!("  - {}", warning);
            }
        }

        println!("\nNode Tiers:");
        println!("{:<30} {:>12} {:>12}", "Node", "Tier", "Avg (μs)");
        println!("{}", "-".repeat(60));

        let mut nodes: Vec<_> = self.nodes.values().collect();
        nodes.sort_by(|a, b| a.avg_us.partial_cmp(&b.avg_us).unwrap());

        for profile in nodes {
            println!(
                "{:<30} {:>12} {:>12.2}",
                profile.name,
                format!("{:?}", profile.tier),
                profile.avg_us
            );
        }
        println!();
    }
}

/// Offline profiler for collecting node execution data
pub struct OfflineProfiler {
    /// Profile being built
    profile: ProfileData,

    /// Running statistics (Welford's algorithm)
    stats: HashMap<String, WelfordStats>,

    /// Target number of profiling ticks
    target_ticks: usize,

    /// Current tick count
    current_ticks: usize,
}

/// Welford's online algorithm state for computing mean/variance
#[derive(Default)]
struct WelfordStats {
    count: usize,
    mean: f64,
    m2: f64,
    min: f64,
    max: f64,
}

impl WelfordStats {
    fn update(&mut self, value: f64) {
        // Initialize min on first value
        if self.count == 0 {
            self.min = f64::MAX;
        }
        self.count += 1;
        self.min = self.min.min(value);
        self.max = self.max.max(value);

        let delta = value - self.mean;
        self.mean += delta / self.count as f64;
        let delta2 = value - self.mean;
        self.m2 += delta * delta2;
    }

    fn stddev(&self) -> f64 {
        if self.count > 1 {
            (self.m2 / (self.count - 1) as f64).sqrt()
        } else {
            0.0
        }
    }
}

impl OfflineProfiler {
    /// Create a new offline profiler
    pub fn new(name: &str, target_ticks: usize) -> Self {
        Self {
            profile: ProfileData::new(name),
            stats: HashMap::new(),
            target_ticks,
            current_ticks: 0,
        }
    }

    /// Record execution time for a node
    pub fn record(&mut self, node_name: &str, duration: Duration) {
        let duration_us = duration.as_micros() as f64;

        self.stats
            .entry(node_name.to_string())
            .or_default()
            .update(duration_us);
    }

    /// Advance tick counter
    pub fn tick(&mut self) {
        self.current_ticks += 1;
    }

    /// Check if profiling is complete
    pub fn is_complete(&self) -> bool {
        self.current_ticks >= self.target_ticks
    }

    /// Get profiling progress (0.0 to 1.0)
    pub fn progress(&self) -> f64 {
        if self.target_ticks == 0 {
            1.0
        } else {
            (self.current_ticks as f64 / self.target_ticks as f64).min(1.0)
        }
    }

    /// Finalize and return the profile
    pub fn finalize(mut self) -> ProfileData {
        self.profile.profiling_ticks = self.current_ticks;

        // Convert stats to node profiles
        for (name, stats) in self.stats {
            let mut profile = NodeProfile {
                name: name.clone(),
                tier: NodeTier::Auto,
                avg_us: stats.mean,
                stddev_us: stats.stddev(),
                min_us: if stats.min == f64::MAX {
                    0.0
                } else {
                    stats.min
                },
                max_us: stats.max,
                sample_count: stats.count,
                is_deterministic: false,
                is_io_heavy: false,
                is_cpu_bound: false,
            };

            profile.classify_tier();
            self.profile.add_node(profile);
        }

        self.profile
    }

    /// Set tick rate for the profile
    pub fn set_tick_rate(&mut self, rate_hz: f64) {
        self.profile.tick_rate_hz = rate_hz;
    }
}

/// Profile-related errors
#[derive(Debug)]
pub enum ProfileError {
    Io(String),
    Serialize(String),
    Deserialize(String),
    NotFound(String),
    Incompatible(String),
}

impl std::fmt::Display for ProfileError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ProfileError::Io(msg) => write!(f, "I/O error: {}", msg),
            ProfileError::Serialize(msg) => write!(f, "Serialization error: {}", msg),
            ProfileError::Deserialize(msg) => write!(f, "Deserialization error: {}", msg),
            ProfileError::NotFound(msg) => write!(f, "Profile not found: {}", msg),
            ProfileError::Incompatible(msg) => write!(f, "Incompatible profile: {}", msg),
        }
    }
}

impl std::error::Error for ProfileError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_tier_default() {
        assert_eq!(NodeTier::default(), NodeTier::Fast);
    }

    #[test]
    fn test_node_profile_classify() {
        let mut profile = NodeProfile::new("test", NodeTier::Auto);
        profile.avg_us = 0.5;
        profile.stddev_us = 0.04; // CV = 0.04/0.5 = 0.08 < 0.10 (deterministic)
        profile.min_us = 0.4;
        profile.max_us = 0.6;
        profile.sample_count = 100;

        profile.classify_tier();

        assert!(
            profile.is_deterministic,
            "Expected deterministic (CV = {})",
            profile.stddev_us / profile.avg_us
        );
        assert_eq!(profile.tier, NodeTier::UltraFast);
    }

    #[test]
    fn test_profile_data_roundtrip() {
        let mut profile = ProfileData::new("test_profile");
        profile.add_node(NodeProfile::new("node1", NodeTier::UltraFast));
        profile.add_node(NodeProfile::new("node2", NodeTier::Fast));

        // Test JSON roundtrip
        let json = serde_json::to_string(&profile).unwrap();
        let loaded: ProfileData = serde_json::from_str(&json).unwrap();

        assert_eq!(loaded.name, "test_profile");
        assert_eq!(loaded.nodes.len(), 2);
        assert_eq!(loaded.get_tier("node1"), NodeTier::UltraFast);
    }

    #[test]
    fn test_offline_profiler() {
        let mut profiler = OfflineProfiler::new("test", 10);

        for _ in 0..10 {
            profiler.record("fast_node", Duration::from_nanos(500));
            profiler.record("slow_node", Duration::from_micros(100));
            profiler.tick();
        }

        assert!(profiler.is_complete());

        let profile = profiler.finalize();
        assert_eq!(profile.nodes.len(), 2);

        let fast = profile.get_node("fast_node").unwrap();
        assert!(fast.avg_us < 1.0);

        let slow = profile.get_node("slow_node").unwrap();
        assert!(slow.avg_us > 50.0);
    }
}
