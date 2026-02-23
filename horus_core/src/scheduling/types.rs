//! Scheduler types and data structures
//!
//! This module contains the public types used by the scheduler.

use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use super::fault_tolerance::FailureHandler;
use super::record_replay::NodeRecorder;
use crate::core::{Node, NodeInfo, RtNode, RtStats};

/// Node tier for explicit annotation by developers
///
/// Use this to declare a node's execution characteristics at compile time.
/// The tier influences the default failure policy applied to the node.
///
/// # Example
/// ```ignore
/// use horus_core::scheduling::{NodeTier, Scheduler};
///
/// let mut scheduler = Scheduler::new();
/// scheduler.add(pid_node).order(0).tier(NodeTier::UltraFast).done();
/// scheduler.add(sensor_node).order(1).tier(NodeTier::Fast).done();
/// scheduler.add(logger_node).order(5).tier(NodeTier::Normal).done();
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
pub enum NodeTier {
    /// Ultra-fast nodes (<1us) - highest priority inline execution
    /// Use for: PID controllers, simple math, data transformations
    UltraFast,

    /// Fast nodes (<10us) - high priority inline execution
    /// Use for: Sensor readers, filter calculations, state machines
    #[default]
    Fast,

    /// Normal nodes (<100us+) - standard scheduling
    /// Use for: Complex algorithms, data processing, logging, diagnostics
    Normal,
}

impl NodeTier {
    /// Get human-readable description
    pub fn description(&self) -> &'static str {
        match self {
            NodeTier::UltraFast => "Ultra-fast inline (<1us)",
            NodeTier::Fast => "Inline execution (<10us)",
            NodeTier::Normal => "Standard scheduling (<100us+)",
        }
    }

    /// Get the default failure policy for this tier.
    ///
    /// Each tier has a sensible default that matches its criticality:
    /// - **UltraFast/Fast**: `Fatal` — these are control-loop nodes; failure = stop.
    /// - **Normal**: `Restart` with exponential backoff (5 attempts, 100ms initial).
    pub fn default_failure_policy(&self) -> super::fault_tolerance::failure_policy::FailurePolicy {
        use super::fault_tolerance::failure_policy::FailurePolicy;
        match self {
            NodeTier::UltraFast | NodeTier::Fast => FailurePolicy::Fatal,
            NodeTier::Normal => FailurePolicy::restart(5, 100),
        }
    }
}

#[cfg(test)]
mod node_tier_tests {
    use super::*;

    #[test]
    fn test_node_tier_default() {
        assert_eq!(NodeTier::default(), NodeTier::Fast);
    }

    #[test]
    fn test_node_tier_descriptions() {
        assert!(!NodeTier::UltraFast.description().is_empty());
        assert!(!NodeTier::Fast.description().is_empty());
        assert!(!NodeTier::Normal.description().is_empty());
    }
}

/// Wrapper that unifies regular nodes and RT nodes under a single dispatch.
pub(crate) enum NodeKind {
    /// A regular Node (may still have RT metadata from builder `.rt()`)
    Regular(Box<dyn Node>),
    /// A node implementing the full RtNode trait
    Rt(Box<dyn RtNode>),
}

impl NodeKind {
    // ---- Node trait delegation ----

    #[inline]
    pub(crate) fn tick(&mut self) {
        match self {
            NodeKind::Regular(n) => n.tick(),
            NodeKind::Rt(n) => n.tick(),
        }
    }

    #[inline]
    pub(crate) fn name(&self) -> &str {
        match self {
            NodeKind::Regular(n) => n.name(),
            NodeKind::Rt(n) => n.name(),
        }
    }

    pub(crate) fn init(&mut self) -> crate::error::HorusResult<()> {
        match self {
            NodeKind::Regular(n) => n.init(),
            NodeKind::Rt(n) => n.init(),
        }
    }

    pub(crate) fn shutdown(&mut self) -> crate::error::HorusResult<()> {
        match self {
            NodeKind::Regular(n) => n.shutdown(),
            NodeKind::Rt(n) => n.shutdown(),
        }
    }

    pub(crate) fn on_error(&mut self, error: &str) {
        match self {
            NodeKind::Regular(n) => n.on_error(error),
            NodeKind::Rt(n) => n.on_error(error),
        }
    }

    pub(crate) fn rate_hz(&self) -> Option<f64> {
        match self {
            NodeKind::Regular(n) => n.rate_hz(),
            NodeKind::Rt(n) => n.rate_hz(),
        }
    }

    pub(crate) fn publishers(&self) -> Vec<crate::core::TopicMetadata> {
        match self {
            NodeKind::Regular(n) => n.publishers(),
            NodeKind::Rt(n) => n.publishers(),
        }
    }

    pub(crate) fn subscribers(&self) -> Vec<crate::core::TopicMetadata> {
        match self {
            NodeKind::Regular(n) => n.subscribers(),
            NodeKind::Rt(n) => n.subscribers(),
        }
    }

    // ---- RtNode-specific access ----

    /// Get a mutable reference to the RtNode, if this is an Rt variant.
    #[inline]
    pub(crate) fn as_rt_mut(&mut self) -> Option<&mut dyn RtNode> {
        match self {
            NodeKind::Rt(n) => Some(n.as_mut()),
            NodeKind::Regular(_) => None,
        }
    }

    /// Get a reference to the RtNode, if this is an Rt variant.
    #[inline]
    pub(crate) fn as_rt(&self) -> Option<&dyn RtNode> {
        match self {
            NodeKind::Rt(n) => Some(n.as_ref()),
            NodeKind::Regular(_) => None,
        }
    }
}

/// Enhanced node registration info with lifecycle tracking and per-node rate control
pub(crate) struct RegisteredNode {
    pub(crate) node: NodeKind,
    /// Cached node name — captured once at registration, used everywhere.
    pub(crate) name: String,
    pub(crate) priority: u32,
    pub(crate) initialized: bool,
    pub(crate) context: Option<NodeInfo>,
    pub(crate) rate_hz: Option<f64>,
    pub(crate) last_tick: Option<Instant>,
    pub(crate) failure_handler: FailureHandler,
    pub(crate) is_rt_node: bool,
    pub(crate) wcet_budget: Option<Duration>,
    pub(crate) deadline: Option<Duration>,
    pub(crate) recorder: Option<NodeRecorder>,
    pub(crate) is_stopped: bool,
    pub(crate) is_paused: bool,
    /// Per-node real-time statistics (populated for RT nodes)
    pub(crate) rt_stats: Option<RtStats>,
}

/// Performance metrics for a scheduler node
///
/// Returned by `Scheduler::metrics()` to provide performance data
/// for monitoring and debugging.
#[derive(Debug, Clone, Default)]
pub struct SchedulerNodeMetrics {
    /// Node name
    pub name: String,
    /// Node priority (lower = higher priority)
    pub priority: u32,
    /// Total number of ticks executed
    pub total_ticks: u64,
    /// Number of successful ticks
    pub successful_ticks: u64,
    /// Number of failed ticks
    pub failed_ticks: u64,
    /// Average tick duration in milliseconds
    pub avg_tick_duration_ms: f64,
    /// Maximum tick duration observed
    pub max_tick_duration_ms: f64,
    /// Minimum tick duration observed
    pub min_tick_duration_ms: f64,
    /// Duration of the last tick
    pub last_tick_duration_ms: f64,
    /// Total messages sent by this node
    pub messages_sent: u64,
    /// Total messages received by this node
    pub messages_received: u64,
    /// Total error count
    pub errors_count: u64,
    /// Total warning count
    pub warnings_count: u64,
    /// Node uptime in seconds
    pub uptime_seconds: f64,
}
