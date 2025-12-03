//! Monitoring System - Heartbeat integration for dashboard visibility
//!
//! This module provides HORUS dashboard integration for sim3d, allowing the
//! simulator to be visible in monitoring tools like `horus monitor` and `horus dashboard`.

use bevy::prelude::*;
use horus_core::memory::platform::shm_heartbeats_dir;
use horus_core::{NodeHeartbeat, NodeMetrics, NodeState};
use tracing::{info, warn};

/// Simulator monitoring state for dashboard integration
#[derive(Resource)]
pub struct SimMonitor {
    /// Node name for heartbeat registration
    pub node_name: String,
    /// Current state of the simulator
    pub state: NodeState,
    /// Metrics tracking
    pub metrics: NodeMetrics,
    /// Last heartbeat write time (to throttle writes)
    last_heartbeat_time: f64,
    /// Heartbeat interval in seconds
    heartbeat_interval: f64,
}

impl SimMonitor {
    /// Create a new simulator monitor
    pub fn new(node_name: impl Into<String>) -> Self {
        Self {
            node_name: node_name.into(),
            state: NodeState::Initializing,
            metrics: NodeMetrics::default(),
            last_heartbeat_time: 0.0,
            heartbeat_interval: 1.0, // Write heartbeat every 1 second
        }
    }

    /// Write heartbeat to shared memory for dashboard visibility
    pub fn write_heartbeat(&self) {
        let heartbeat = NodeHeartbeat::from_metrics(self.state.clone(), &self.metrics);
        if let Err(e) = heartbeat.write_to_file(&self.node_name) {
            warn!("Failed to write heartbeat for {}: {}", self.node_name, e);
        }
    }

    /// Remove heartbeat file on shutdown
    pub fn cleanup_heartbeat(&self) {
        let path = shm_heartbeats_dir().join(&self.node_name);
        if path.exists() {
            let _ = std::fs::remove_file(&path);
        }
    }

    /// Update metrics from frame timing
    pub fn update_metrics(&mut self, frame_time_ms: f64) {
        self.metrics.total_ticks += 1;
        self.metrics.avg_tick_duration_ms =
            (self.metrics.avg_tick_duration_ms * 0.95) + (frame_time_ms * 0.05);
    }

    /// Check if it's time to write a heartbeat
    pub fn should_write_heartbeat(&self, current_time: f64) -> bool {
        current_time - self.last_heartbeat_time >= self.heartbeat_interval
    }

    /// Mark heartbeat as written
    pub fn mark_heartbeat_written(&mut self, current_time: f64) {
        self.last_heartbeat_time = current_time;
    }
}

impl Default for SimMonitor {
    fn default() -> Self {
        // Create a unique name based on session ID if available
        let node_name = if let Ok(session_id) = std::env::var("HORUS_SESSION_ID") {
            format!("sim3d.{}", session_id)
        } else {
            "sim3d".to_string()
        };
        Self::new(node_name)
    }
}

/// Startup system to initialize monitoring and write initial heartbeat
pub fn monitor_startup_system(mut monitor: ResMut<SimMonitor>) {
    monitor.state = NodeState::Running;
    monitor.write_heartbeat();
    info!(
        "sim3d registered with HORUS monitor as '{}'",
        monitor.node_name
    );
}

/// Periodic heartbeat system - writes heartbeat every interval
pub fn monitor_heartbeat_system(time: Res<Time>, mut monitor: ResMut<SimMonitor>) {
    let current_time = time.elapsed_secs_f64();
    let frame_time_ms = time.delta_secs_f64() * 1000.0;

    // Update metrics
    monitor.update_metrics(frame_time_ms);

    // Write heartbeat at configured interval
    if monitor.should_write_heartbeat(current_time) {
        monitor.write_heartbeat();
        monitor.mark_heartbeat_written(current_time);
    }
}

/// Exit check system - monitors for AppExit events and writes final heartbeat
pub fn monitor_exit_check_system(
    mut exit_events: EventReader<bevy::app::AppExit>,
    monitor: Res<SimMonitor>,
) {
    for _event in exit_events.read() {
        // Write final "Stopped" heartbeat before exit
        let final_heartbeat = NodeHeartbeat::from_metrics(NodeState::Stopped, &monitor.metrics);
        let _ = final_heartbeat.write_to_file(&monitor.node_name);
        info!(
            "sim3d shutting down, wrote final heartbeat for '{}'",
            monitor.node_name
        );
    }
}

/// Plugin to add monitoring systems to the Bevy app
pub struct SimMonitorPlugin;

impl Plugin for SimMonitorPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SimMonitor>()
            .add_systems(Startup, monitor_startup_system)
            .add_systems(Update, monitor_heartbeat_system)
            .add_systems(Last, monitor_exit_check_system);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sim_monitor_creation() {
        let monitor = SimMonitor::new("test_sim3d");
        assert_eq!(monitor.node_name, "test_sim3d");
        assert!(matches!(monitor.state, NodeState::Initializing));
    }

    #[test]
    fn test_sim_monitor_metrics_update() {
        let mut monitor = SimMonitor::new("test_sim3d");
        monitor.update_metrics(16.0); // ~60fps
        assert_eq!(monitor.metrics.total_ticks, 1);
        assert!(monitor.metrics.avg_tick_duration_ms > 0.0);
    }

    #[test]
    fn test_heartbeat_throttling() {
        let monitor = SimMonitor::new("test_sim3d");
        assert!(monitor.should_write_heartbeat(0.0)); // First time always true
        assert!(!monitor.should_write_heartbeat(0.5)); // Under interval
        assert!(monitor.should_write_heartbeat(1.5)); // Over interval
    }
}
