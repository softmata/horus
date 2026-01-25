//! Scheduler types and data structures
//!
//! This module contains the public types used by the scheduler.

use std::time::{Duration, Instant};

use crate::core::{Node, NodeInfo};
use super::fault_tolerance::CircuitBreaker;
use super::jit::CompiledDataflow;
use super::record_replay::NodeRecorder;

/// Node control command for IPC-based lifecycle management
#[derive(Debug, Clone, PartialEq)]
pub enum NodeControlCommand {
    /// Stop the node (won't tick anymore)
    Stop,
    /// Restart the node (re-initialize and resume)
    Restart,
    /// Pause execution (can resume later)
    Pause,
    /// Resume paused node
    Resume,
}

/// Enhanced node registration info with lifecycle tracking and per-node rate control
pub(crate) struct RegisteredNode {
    pub(crate) node: Box<dyn Node>,
    pub(crate) priority: u32,
    pub(crate) initialized: bool,
    pub(crate) context: Option<NodeInfo>,
    pub(crate) rate_hz: Option<f64>,
    pub(crate) last_tick: Option<Instant>,
    pub(crate) circuit_breaker: CircuitBreaker,
    pub(crate) is_rt_node: bool,
    pub(crate) wcet_budget: Option<Duration>,
    pub(crate) deadline: Option<Duration>,
    pub(crate) is_jit_compiled: bool,
    pub(crate) jit_stats: Option<CompiledDataflow>,
    pub(crate) recorder: Option<NodeRecorder>,
    #[allow(dead_code)]
    pub(crate) is_replay_node: bool,
    pub(crate) is_stopped: bool,
    pub(crate) is_paused: bool,
}

/// Performance metrics for a scheduler node
///
/// Returned by `Scheduler::get_metrics()` to provide performance data
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
