use crate::{BatteryState, EmergencyStop, ResourceUsage, SafetyStatus, StatusLevel};
use horus_core::error::HorusResult;

// Import algorithms from horus_library/algorithms
use crate::algorithms::safety_layer::SafetyLayer;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Hub, Node, NodeInfo};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};
#[cfg(feature = "sysinfo")]
use sysinfo::System;

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

/// Safety Monitor Node - Monitors critical safety systems and conditions
///
/// Watches system resources, emergency stops, battery levels, communication health,
/// and other safety-critical parameters. Triggers safety responses when limits exceeded.
///
/// This node is a thin wrapper around the pure algorithms in horus_library/algorithms.
///
/// # Hybrid Pattern
///
/// ```rust,ignore
/// let node = SafetyMonitorNode::builder()
///     .with_filter(|status| {
///         // Only publish on status changes
///         if status.mode != SafetyStatus::MODE_NORMAL {
///             Some(status)
///         } else {
///             None
///         }
///     })
///     .build()?;
/// ```
pub struct SafetyMonitorNode<P = PassThrough<SafetyStatus>>
where
    P: Processor<SafetyStatus>,
{
    publisher: Hub<SafetyStatus>,
    emergency_subscriber: Hub<EmergencyStop>,
    battery_subscriber: Hub<BatteryState>,
    resource_subscriber: Hub<ResourceUsage>,

    // Algorithm instance
    safety_layer: SafetyLayer,

    #[cfg(feature = "sysinfo")]
    system: System,
    safety_checks: Arc<Mutex<HashMap<String, SafetyCheck>>>,

    // Configuration
    cpu_threshold: f32,
    memory_threshold: f32,
    disk_threshold: f32,
    communication_timeout_ms: u64,

    // State
    current_velocity: f64,
    obstacle_distance: f64,
    battery_percent: f64,
    temperature: f64,
    last_emergency_time: u64,
    last_battery_time: u64,
    last_resource_time: u64,
    current_safety_level: StatusLevel,

    // Processor for hybrid pattern
    processor: P,
}

#[derive(Clone)]
struct SafetyCheck {
    #[allow(dead_code)]
    name: String,
    status: StatusLevel,
    message: String,
    last_update: u64,
    timeout_ms: u64,
}

impl SafetyMonitorNode {
    /// Create a new safety monitor node with default topic "safety_status"
    pub fn new() -> Result<Self> {
        Self::new_with_topic("safety_status")
    }

    /// Create a new safety monitor node with custom topic
    pub fn new_with_topic(topic: &str) -> Result<Self> {
        // Create safety layer with default limits
        let mut safety_layer = SafetyLayer::new();
        safety_layer.set_max_velocity(2.0); // 2 m/s max
        safety_layer.set_min_obstacle_distance(0.3); // 30cm min distance
        safety_layer.set_min_battery(15.0); // 15% battery
        safety_layer.set_max_temperature(80.0); // 80°C max

        Ok(Self {
            publisher: Hub::new(topic)?,
            emergency_subscriber: Hub::new("emergency_stop")?,
            battery_subscriber: Hub::new("battery_state")?,
            resource_subscriber: Hub::new("resource_usage")?,

            safety_layer,

            #[cfg(feature = "sysinfo")]
            system: System::new_all(),
            safety_checks: Arc::new(Mutex::new(HashMap::new())),

            // Default thresholds
            cpu_threshold: 90.0,            // 90% CPU usage
            memory_threshold: 85.0,         // 85% memory usage
            disk_threshold: 95.0,           // 95% disk usage
            communication_timeout_ms: 5000, // 5 second timeout

            // State
            current_velocity: 0.0,
            obstacle_distance: 100.0, // Start with large distance (no obstacle)
            battery_percent: 100.0,
            temperature: 25.0, // Room temperature default
            last_emergency_time: 0,
            last_battery_time: 0,
            last_resource_time: 0,
            current_safety_level: StatusLevel::Ok,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> SafetyMonitorNodeBuilder<PassThrough<SafetyStatus>> {
        SafetyMonitorNodeBuilder::new()
    }
}

impl<P> SafetyMonitorNode<P>
where
    P: Processor<SafetyStatus>,
{
    /// Set CPU usage threshold (0-100%)
    pub fn set_cpu_threshold(&mut self, threshold: f32) {
        self.cpu_threshold = threshold.clamp(0.0, 100.0);
    }

    /// Set memory usage threshold (0-100%)
    pub fn set_memory_threshold(&mut self, threshold: f32) {
        self.memory_threshold = threshold.clamp(0.0, 100.0);
    }

    /// Set disk usage threshold (0-100%)
    pub fn set_disk_threshold(&mut self, threshold: f32) {
        self.disk_threshold = threshold.clamp(0.0, 100.0);
    }

    /// Set temperature threshold (°C)
    pub fn set_temperature_threshold(&mut self, threshold: f32) {
        self.safety_layer.set_max_temperature(threshold as f64);
    }

    /// Set battery threshold (0-100%)
    pub fn set_battery_threshold(&mut self, threshold: f32) {
        let threshold = threshold.clamp(0.0, 100.0);
        self.safety_layer.set_min_battery(threshold as f64);
    }

    /// Set communication timeout in milliseconds
    pub fn set_communication_timeout(&mut self, timeout_ms: u64) {
        self.communication_timeout_ms = timeout_ms;
    }

    /// Add a custom safety check
    pub fn add_safety_check(&mut self, name: &str, timeout_ms: u64) {
        if let Ok(mut checks) = self.safety_checks.lock() {
            checks.insert(
                name.to_string(),
                SafetyCheck {
                    name: name.to_string(),
                    status: StatusLevel::Ok,
                    message: "Not initialized".to_string(),
                    last_update: 0,
                    timeout_ms,
                },
            );
        }
    }

    /// Update a custom safety check
    pub fn update_safety_check(&mut self, name: &str, status: StatusLevel, message: &str) {
        if let Ok(mut checks) = self.safety_checks.lock() {
            if let Some(check) = checks.get_mut(name) {
                check.status = status;
                check.message = message.to_string();
                check.last_update = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_millis() as u64;
            }
        }
    }

    fn check_system_resources(&mut self) -> StatusLevel {
        #[cfg(feature = "sysinfo")]
        {
            self.system.refresh_all();

            // Check CPU usage
            let cpu_usage = self.system.global_cpu_info().cpu_usage();
            if cpu_usage > self.cpu_threshold {
                return StatusLevel::Fatal;
            } else if cpu_usage > self.cpu_threshold * 0.8 {
                return StatusLevel::Error;
            }

            // Check memory usage
            let total_memory = self.system.total_memory();
            let used_memory = self.system.used_memory();
            let memory_usage = (used_memory as f32 / total_memory as f32) * 100.0;

            if memory_usage > self.memory_threshold {
                return StatusLevel::Fatal;
            } else if memory_usage > self.memory_threshold * 0.8 {
                return StatusLevel::Error;
            }
        }

        #[cfg(not(feature = "sysinfo"))]
        {
            // Without sysinfo, just return OK (could add basic resource checks here)
        }

        StatusLevel::Ok
    }

    fn check_communication_health(&self) -> StatusLevel {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        // Check if we've received emergency stop updates recently
        if current_time - self.last_emergency_time > self.communication_timeout_ms {
            return StatusLevel::Error;
        }

        // Check if we've received battery updates recently
        if current_time - self.last_battery_time > self.communication_timeout_ms {
            return StatusLevel::Warn;
        }

        StatusLevel::Ok
    }

    fn check_custom_safety_checks(&self) -> StatusLevel {
        if let Ok(checks) = self.safety_checks.lock() {
            let current_time = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64;

            let mut worst_status = StatusLevel::Ok;

            for check in checks.values() {
                // Check for timeout
                if current_time - check.last_update > check.timeout_ms {
                    worst_status = StatusLevel::Error;
                    continue;
                }

                // Update worst status
                worst_status = worst_status.max(check.status);
            }

            return worst_status;
        }

        StatusLevel::Ok
    }

    fn determine_overall_safety_level(&mut self) -> StatusLevel {
        let resource_status = self.check_system_resources();
        let comm_status = self.check_communication_health();
        let custom_status = self.check_custom_safety_checks();

        // Use safety layer for comprehensive safety check
        use crate::algorithms::safety_layer::SafetyStatus as AlgoSafetyStatus;
        let safety_check = self.safety_layer.check_all(
            self.current_velocity,
            self.obstacle_distance,
            self.battery_percent,
            self.temperature,
        );

        // Map algorithm safety status to node status level
        let safety_status = match safety_check {
            AlgoSafetyStatus::Safe => StatusLevel::Ok,
            AlgoSafetyStatus::Warning => StatusLevel::Warn,
            AlgoSafetyStatus::Critical => StatusLevel::Fatal,
        };

        // Return the worst (highest priority) status
        resource_status
            .max(comm_status)
            .max(custom_status)
            .max(safety_status)
    }

    fn publish_safety_status(&mut self) {
        let mut status = SafetyStatus::new();

        match self.current_safety_level {
            StatusLevel::Ok => {
                // Default SafetyStatus::new() is already configured for normal operation
            }
            StatusLevel::Warn => {
                status.mode = SafetyStatus::MODE_REDUCED;
                status.set_fault(1); // Warning fault code
            }
            StatusLevel::Error => {
                status.mode = SafetyStatus::MODE_REDUCED;
                status.set_fault(2); // Error fault code
            }
            StatusLevel::Fatal => {
                status.estop_engaged = true;
                status.mode = SafetyStatus::MODE_SAFE_STOP;
                status.set_fault(999); // Critical fault code
            }
        }

        // Process through pipeline
        if let Some(processed) = self.processor.process(status) {
            let _ = self.publisher.send(processed, &mut None);
        }
    }
}

impl<P> Node for SafetyMonitorNode<P>
where
    P: Processor<SafetyStatus>,
{
    fn name(&self) -> &'static str {
        "SafetyMonitorNode"
    }

    fn init(&mut self, _ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_start();
        Ok(())
    }

    fn shutdown(&mut self, _ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_shutdown();
        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        self.processor.on_tick();
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        // Check for incoming emergency stop messages
        if let Some(emergency_msg) = self.emergency_subscriber.recv(&mut None) {
            self.last_emergency_time = current_time;
            if emergency_msg.engaged {
                self.current_safety_level = StatusLevel::Fatal;
            }
        }

        // Check for incoming battery messages
        if let Some(battery_msg) = self.battery_subscriber.recv(&mut None) {
            self.last_battery_time = current_time;
            self.battery_percent = battery_msg.percentage as f64;

            // Temperature from battery if available
            if battery_msg.temperature > 0.0 {
                self.temperature = battery_msg.temperature as f64;
            }
        }

        // Check for incoming resource usage messages
        if let Some(_resource_msg) = self.resource_subscriber.recv(&mut None) {
            self.last_resource_time = current_time;
        }

        // Determine overall safety level
        self.current_safety_level = self.determine_overall_safety_level();

        // Publish safety status
        self.publish_safety_status();
    }
}

// Default impl removed - use SafetyMonitorNode::new() instead which returns HorusResult

/// Builder for SafetyMonitorNode with processor configuration
pub struct SafetyMonitorNodeBuilder<P>
where
    P: Processor<SafetyStatus>,
{
    topic: String,
    processor: P,
}

impl SafetyMonitorNodeBuilder<PassThrough<SafetyStatus>> {
    /// Create a new builder with default PassThrough processor
    pub fn new() -> Self {
        Self {
            topic: "safety_status".to_string(),
            processor: PassThrough::new(),
        }
    }
}

impl Default for SafetyMonitorNodeBuilder<PassThrough<SafetyStatus>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P> SafetyMonitorNodeBuilder<P>
where
    P: Processor<SafetyStatus>,
{
    /// Set output topic
    pub fn topic(mut self, topic: &str) -> Self {
        self.topic = topic.to_string();
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> SafetyMonitorNodeBuilder<P2>
    where
        P2: Processor<SafetyStatus>,
    {
        SafetyMonitorNodeBuilder {
            topic: self.topic,
            processor,
        }
    }

    /// Set a closure-based processor
    pub fn with_closure<F>(
        self,
        f: F,
    ) -> SafetyMonitorNodeBuilder<ClosureProcessor<SafetyStatus, SafetyStatus, F>>
    where
        F: FnMut(SafetyStatus) -> SafetyStatus + Send + 'static,
    {
        SafetyMonitorNodeBuilder {
            topic: self.topic,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Set a filter-based processor
    pub fn with_filter<F>(
        self,
        f: F,
    ) -> SafetyMonitorNodeBuilder<FilterProcessor<SafetyStatus, SafetyStatus, F>>
    where
        F: FnMut(SafetyStatus) -> Option<SafetyStatus> + Send + 'static,
    {
        SafetyMonitorNodeBuilder {
            topic: self.topic,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor (pipe)
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> SafetyMonitorNodeBuilder<Pipeline<SafetyStatus, SafetyStatus, SafetyStatus, P, P2>>
    where
        P2: Processor<SafetyStatus, SafetyStatus>,
    {
        SafetyMonitorNodeBuilder {
            topic: self.topic,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the node
    pub fn build(self) -> Result<SafetyMonitorNode<P>> {
        // Create safety layer with default limits
        let mut safety_layer = SafetyLayer::new();
        safety_layer.set_max_velocity(2.0);
        safety_layer.set_min_obstacle_distance(0.3);
        safety_layer.set_min_battery(15.0);
        safety_layer.set_max_temperature(80.0);

        Ok(SafetyMonitorNode {
            publisher: Hub::new(&self.topic)?,
            emergency_subscriber: Hub::new("emergency_stop")?,
            battery_subscriber: Hub::new("battery_state")?,
            resource_subscriber: Hub::new("resource_usage")?,

            safety_layer,

            #[cfg(feature = "sysinfo")]
            system: System::new_all(),
            safety_checks: Arc::new(Mutex::new(HashMap::new())),

            cpu_threshold: 90.0,
            memory_threshold: 85.0,
            disk_threshold: 95.0,
            communication_timeout_ms: 5000,

            current_velocity: 0.0,
            obstacle_distance: 100.0,
            battery_percent: 100.0,
            temperature: 25.0,
            last_emergency_time: 0,
            last_battery_time: 0,
            last_resource_time: 0,
            current_safety_level: StatusLevel::Ok,
            processor: self.processor,
        })
    }
}
