use crate::{DigitalIO, EmergencyStop, LaserScan, Odometry};
use horus_core::error::HorusResult;

// Import algorithms from horus_library/algorithms
use crate::algorithms::aabb::AABB;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, Topic};
use std::collections::VecDeque;
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

/// Collision Detector Node - Safety system for obstacle avoidance and collision prevention
///
/// Monitors lidar and other sensors to detect potential collisions and trigger
/// emergency stops or evasive actions to ensure robot and operator safety.
///
/// # Hybrid Pattern
///
/// ```rust,ignore
/// let node = CollisionDetectorNode::builder()
///     .with_filter(|estop| {
///         // Only forward actual emergency stops
///         if estop.engaged {
///             Some(estop)
///         } else {
///             None
///         }
///     })
///     .build()?;
/// ```
pub struct CollisionDetectorNode<P = PassThrough<EmergencyStop>>
where
    P: Processor<EmergencyStop>,
{
    emergency_publisher: Topic<EmergencyStop>,
    lidar_subscriber: Topic<LaserScan>,
    odometry_subscriber: Topic<Odometry>,
    digital_io_subscriber: Topic<DigitalIO>, // For safety sensors

    // Safety zones (distances in meters)
    critical_zone: f64,   // Immediate stop zone
    warning_zone: f64,    // Slow down zone
    monitoring_zone: f64, // Track obstacles zone

    // Robot geometry
    robot_width: f64,
    robot_length: f64,
    safety_margin: f64,

    // Current state
    current_velocity: (f64, f64, f64), // (vx, vy, omega)
    current_pose: (f64, f64, f64),     // (x, y, theta)

    // Collision detection state
    collision_imminent: bool,
    warning_active: bool,
    obstacles_detected: Vec<(f64, f64)>, // Obstacle positions
    safety_sensors_active: bool,

    // Dynamic safety parameters
    velocity_dependent_zones: bool,
    min_stopping_distance: f64,
    max_deceleration: f64, // m/s²

    // Collision history for filtering
    collision_history: VecDeque<bool>,
    history_length: usize,

    // Safety sensor configuration
    safety_sensor_pins: Vec<u8>, // Digital input pins for safety sensors

    // Timing
    last_lidar_time: u64,
    emergency_cooldown: u64,
    last_emergency_time: u64,

    // Processor for hybrid pattern
    processor: P,
}

impl CollisionDetectorNode {
    /// Create a new collision detector node with default topics
    pub fn new() -> Result<Self> {
        Self::new_with_topics("emergency_stop", "lidar_scan", "odom", "digital_input")
    }

    /// Create a new collision detector node with custom topics
    pub fn new_with_topics(
        emergency_topic: &str,
        lidar_topic: &str,
        odom_topic: &str,
        io_topic: &str,
    ) -> Result<Self> {
        Ok(Self {
            emergency_publisher: Topic::new(emergency_topic)?,
            lidar_subscriber: Topic::new(lidar_topic)?,
            odometry_subscriber: Topic::new(odom_topic)?,
            digital_io_subscriber: Topic::new(io_topic)?,

            // Default safety zones
            critical_zone: 0.3,   // 30cm immediate stop
            warning_zone: 0.8,    // 80cm slow down
            monitoring_zone: 2.0, // 2m obstacle tracking

            // Default robot geometry
            robot_width: 0.6,   // 60cm wide robot
            robot_length: 0.8,  // 80cm long robot
            safety_margin: 0.1, // 10cm additional margin

            current_velocity: (0.0, 0.0, 0.0),
            current_pose: (0.0, 0.0, 0.0),

            collision_imminent: false,
            warning_active: false,
            obstacles_detected: Vec::new(),
            safety_sensors_active: false,

            velocity_dependent_zones: true,
            min_stopping_distance: 0.2, // 20cm minimum
            max_deceleration: 2.0,      // 2 m/s² max braking

            collision_history: VecDeque::new(),
            history_length: 5, // Track last 5 readings

            safety_sensor_pins: vec![0, 1, 2, 3], // Default safety sensor pins

            last_lidar_time: 0,
            emergency_cooldown: 500, // 500ms cooldown between emergency stops
            last_emergency_time: 0,
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> CollisionDetectorNodeBuilder<PassThrough<EmergencyStop>> {
        CollisionDetectorNodeBuilder::new()
    }
}

impl<P> CollisionDetectorNode<P>
where
    P: Processor<EmergencyStop>,
{
    /// Configure safety zones
    pub fn set_safety_zones(&mut self, critical: f64, warning: f64, monitoring: f64) {
        self.critical_zone = critical;
        self.warning_zone = warning;
        self.monitoring_zone = monitoring;
    }

    /// Configure robot geometry for collision detection
    pub fn set_robot_geometry(&mut self, width: f64, length: f64, margin: f64) {
        self.robot_width = width;
        self.robot_length = length;
        self.safety_margin = margin;
    }

    /// Configure safety sensor input pins
    pub fn set_safety_sensor_pins(&mut self, pins: Vec<u8>) {
        self.safety_sensor_pins = pins;
    }

    /// Enable/disable velocity-dependent safety zones
    pub fn set_velocity_dependent_zones(&mut self, enabled: bool) {
        self.velocity_dependent_zones = enabled;
    }

    /// Get current collision status
    pub fn is_collision_imminent(&self) -> bool {
        self.collision_imminent
    }

    /// Get warning status
    pub fn is_warning_active(&self) -> bool {
        self.warning_active
    }

    /// Get detected obstacles
    pub fn get_obstacles(&self) -> &Vec<(f64, f64)> {
        &self.obstacles_detected
    }

    fn calculate_dynamic_zones(&self) -> (f64, f64, f64) {
        if !self.velocity_dependent_zones {
            return (self.critical_zone, self.warning_zone, self.monitoring_zone);
        }

        let speed = (self.current_velocity.0.powi(2) + self.current_velocity.1.powi(2)).sqrt();

        // Calculate stopping distance: v²/(2*a) + safety margin
        let stopping_distance =
            (speed * speed) / (2.0 * self.max_deceleration) + self.min_stopping_distance;

        let dynamic_critical = stopping_distance.max(self.critical_zone);
        let dynamic_warning = (stopping_distance * 2.0).max(self.warning_zone);
        let dynamic_monitoring = (stopping_distance * 3.0).max(self.monitoring_zone);

        (dynamic_critical, dynamic_warning, dynamic_monitoring)
    }

    fn detect_lidar_obstacles(&mut self, lidar: &LaserScan) -> (bool, bool) {
        self.obstacles_detected.clear();

        let (critical_zone, warning_zone, _monitoring_zone) = self.calculate_dynamic_zones();

        let mut critical_collision = false;
        let mut warning_collision = false;

        let robot_x = self.current_pose.0;
        let robot_y = self.current_pose.1;
        let robot_theta = self.current_pose.2;

        // Analyze lidar scan for obstacles in robot's path
        for (i, &range) in lidar.ranges.iter().enumerate() {
            if range > 0.1 && range < lidar.range_max {
                let beam_angle = lidar.angle_min as f64 + i as f64 * lidar.angle_increment as f64;
                let absolute_angle = robot_theta + beam_angle;

                // Convert to world coordinates
                let obstacle_x = robot_x + range as f64 * absolute_angle.cos();
                let obstacle_y = robot_y + range as f64 * absolute_angle.sin();

                // Check if obstacle is in robot's collision envelope
                if self.is_in_collision_path(obstacle_x, obstacle_y, range as f64, beam_angle) {
                    self.obstacles_detected.push((obstacle_x, obstacle_y));

                    // Determine collision severity
                    if (range as f64) < critical_zone {
                        critical_collision = true;
                    } else if (range as f64) < warning_zone {
                        warning_collision = true;
                    }
                }
            }
        }

        (critical_collision, warning_collision)
    }

    fn is_in_collision_path(
        &self,
        obstacle_x: f64,
        obstacle_y: f64,
        range: f64,
        beam_angle: f64,
    ) -> bool {
        // Use AABB for collision detection
        let robot_x = self.current_pose.0;
        let robot_y = self.current_pose.1;

        // Create robot AABB (centered at current pose, expanded by safety margin)
        let robot_bbox = AABB::from_center(
            robot_x,
            robot_y,
            self.robot_width + self.safety_margin,
            self.robot_length + self.safety_margin,
        );

        // Create small AABB for obstacle point (treat as small circle)
        let obstacle_radius = 0.05; // 5cm obstacle radius
        let obstacle_bbox = AABB::from_center(
            obstacle_x,
            obstacle_y,
            obstacle_radius * 2.0,
            obstacle_radius * 2.0,
        );

        // Check AABB intersection
        let intersects = robot_bbox.intersects(&obstacle_bbox);

        // Additional check for forward direction and angular cone
        let robot_theta = self.current_pose.2;
        let dx = obstacle_x - robot_x;
        let dy = obstacle_y - robot_y;
        let local_x = dx * robot_theta.cos() + dy * robot_theta.sin();

        // Only consider obstacles in front of robot
        let in_front = local_x > -self.robot_length / 2.0;

        // Additional check for angular obstacles in forward cone
        let in_forward_cone = range < 1.0 && beam_angle.abs() < std::f64::consts::PI / 3.0;

        intersects && in_front || in_forward_cone
    }

    fn check_safety_sensors(&mut self, digital_io: &DigitalIO) -> bool {
        if digital_io.pin_count == 0 {
            return false;
        }

        let mut safety_triggered = false;

        // Check configured safety sensor pins
        for &pin in &self.safety_sensor_pins {
            if pin < digital_io.pin_count {
                let pin_state = if (pin as usize) < digital_io.pins.len() {
                    digital_io.pins[pin as usize]
                } else {
                    false
                };

                // Assume safety sensors are normally open (true = triggered)
                if pin_state {
                    safety_triggered = true;
                    break;
                }
            }
        }

        self.safety_sensors_active = safety_triggered;
        safety_triggered
    }

    fn filter_collision_detection(&mut self, collision_detected: bool) -> bool {
        // Add current detection to history
        self.collision_history.push_back(collision_detected);

        // Maintain history length
        while self.collision_history.len() > self.history_length {
            self.collision_history.pop_front();
        }

        // Apply filtering - require majority consensus for collision
        let collision_count = self.collision_history.iter().filter(|&&x| x).count();
        collision_count > self.history_length / 2
    }

    fn trigger_emergency_stop(&mut self, reason: &str) {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        // Check emergency cooldown to prevent spam
        if current_time - self.last_emergency_time < self.emergency_cooldown {
            return;
        }

        let emergency_msg = EmergencyStop::engage(reason);
        // Process through pipeline
        if let Some(processed) = self.processor.process(emergency_msg) {
            let _ = self.emergency_publisher.send(processed, &mut None);
        }

        self.last_emergency_time = current_time;
    }

    fn update_collision_state(&mut self, critical: bool, warning: bool, safety_sensors: bool) {
        let previous_collision = self.collision_imminent;
        let previous_warning = self.warning_active;

        // Apply filtering to critical collisions
        self.collision_imminent = self.filter_collision_detection(critical || safety_sensors);
        self.warning_active = warning && !self.collision_imminent;

        // Trigger emergency stop on collision state change
        if self.collision_imminent && !previous_collision {
            if safety_sensors {
                self.trigger_emergency_stop("Safety sensor triggered");
            } else {
                self.trigger_emergency_stop("Collision imminent - obstacle detected");
            }
        }

        // Log state changes for debugging
        if self.collision_imminent != previous_collision || self.warning_active != previous_warning
        {
            // In a real implementation, this would log to a proper logging system
        }
    }

    /// Manual emergency stop trigger
    pub fn trigger_manual_emergency(&mut self) {
        self.trigger_emergency_stop("Manual emergency stop");
        self.collision_imminent = true;
    }

    /// Reset collision state (after manual intervention)
    pub fn reset_collision_state(&mut self) {
        self.collision_imminent = false;
        self.warning_active = false;
        self.obstacles_detected.clear();
        self.collision_history.clear();
        self.safety_sensors_active = false;
    }

    /// Get collision detection statistics
    pub fn get_detection_stats(&self) -> (usize, f64) {
        let total_obstacles = self.obstacles_detected.len();
        let min_distance = self
            .obstacles_detected
            .iter()
            .map(|&(x, y)| {
                let dx = x - self.current_pose.0;
                let dy = y - self.current_pose.1;
                (dx * dx + dy * dy).sqrt()
            })
            .fold(f64::INFINITY, f64::min);

        (
            total_obstacles,
            if min_distance.is_finite() {
                min_distance
            } else {
                0.0
            },
        )
    }
}

impl<P> Node for CollisionDetectorNode<P>
where
    P: Processor<EmergencyStop>,
{
    fn name(&self) -> &'static str {
        "CollisionDetectorNode"
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

        // Update current pose and velocity
        if let Some(odom) = self.odometry_subscriber.recv(&mut None) {
            self.current_pose = (odom.pose.x, odom.pose.y, odom.pose.theta);

            self.current_velocity = (
                odom.twist.linear[0],
                odom.twist.linear[1],
                odom.twist.angular[2],
            );
        }

        let mut critical_collision = false;
        let mut warning_collision = false;
        let mut safety_sensors_triggered = false;

        // Check lidar for obstacles
        if let Some(lidar) = self.lidar_subscriber.recv(&mut None) {
            if lidar.timestamp > self.last_lidar_time {
                let (critical, warning) = self.detect_lidar_obstacles(&lidar);
                critical_collision = critical;
                warning_collision = warning;
                self.last_lidar_time = lidar.timestamp;
            }
        }

        // Check safety sensors
        if let Some(digital_io) = self.digital_io_subscriber.recv(&mut None) {
            safety_sensors_triggered = self.check_safety_sensors(&digital_io);
        }

        // Update collision state and trigger emergency stops if necessary
        self.update_collision_state(
            critical_collision,
            warning_collision,
            safety_sensors_triggered,
        );
    }
}

// Default impl removed - use CollisionDetectorNode::new() instead which returns HorusResult
// The default configuration is already built into new()

/// Builder for CollisionDetectorNode with processor configuration
pub struct CollisionDetectorNodeBuilder<P>
where
    P: Processor<EmergencyStop>,
{
    emergency_topic: String,
    lidar_topic: String,
    odom_topic: String,
    io_topic: String,
    processor: P,
}

impl CollisionDetectorNodeBuilder<PassThrough<EmergencyStop>> {
    /// Create a new builder with default PassThrough processor
    pub fn new() -> Self {
        Self {
            emergency_topic: "emergency_stop".to_string(),
            lidar_topic: "lidar_scan".to_string(),
            odom_topic: "odom".to_string(),
            io_topic: "digital_input".to_string(),
            processor: PassThrough::new(),
        }
    }
}

impl Default for CollisionDetectorNodeBuilder<PassThrough<EmergencyStop>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P> CollisionDetectorNodeBuilder<P>
where
    P: Processor<EmergencyStop>,
{
    /// Set emergency output topic
    pub fn emergency_topic(mut self, topic: &str) -> Self {
        self.emergency_topic = topic.to_string();
        self
    }

    /// Set lidar input topic
    pub fn lidar_topic(mut self, topic: &str) -> Self {
        self.lidar_topic = topic.to_string();
        self
    }

    /// Set odometry input topic
    pub fn odom_topic(mut self, topic: &str) -> Self {
        self.odom_topic = topic.to_string();
        self
    }

    /// Set digital IO input topic
    pub fn io_topic(mut self, topic: &str) -> Self {
        self.io_topic = topic.to_string();
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> CollisionDetectorNodeBuilder<P2>
    where
        P2: Processor<EmergencyStop>,
    {
        CollisionDetectorNodeBuilder {
            emergency_topic: self.emergency_topic,
            lidar_topic: self.lidar_topic,
            odom_topic: self.odom_topic,
            io_topic: self.io_topic,
            processor,
        }
    }

    /// Set a closure-based processor
    pub fn with_closure<F>(
        self,
        f: F,
    ) -> CollisionDetectorNodeBuilder<ClosureProcessor<EmergencyStop, EmergencyStop, F>>
    where
        F: FnMut(EmergencyStop) -> EmergencyStop + Send + 'static,
    {
        CollisionDetectorNodeBuilder {
            emergency_topic: self.emergency_topic,
            lidar_topic: self.lidar_topic,
            odom_topic: self.odom_topic,
            io_topic: self.io_topic,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Set a filter-based processor
    pub fn with_filter<F>(
        self,
        f: F,
    ) -> CollisionDetectorNodeBuilder<FilterProcessor<EmergencyStop, EmergencyStop, F>>
    where
        F: FnMut(EmergencyStop) -> Option<EmergencyStop> + Send + 'static,
    {
        CollisionDetectorNodeBuilder {
            emergency_topic: self.emergency_topic,
            lidar_topic: self.lidar_topic,
            odom_topic: self.odom_topic,
            io_topic: self.io_topic,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor (pipe)
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> CollisionDetectorNodeBuilder<Pipeline<EmergencyStop, EmergencyStop, EmergencyStop, P, P2>>
    where
        P2: Processor<EmergencyStop, EmergencyStop>,
    {
        CollisionDetectorNodeBuilder {
            emergency_topic: self.emergency_topic,
            lidar_topic: self.lidar_topic,
            odom_topic: self.odom_topic,
            io_topic: self.io_topic,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the node
    pub fn build(self) -> Result<CollisionDetectorNode<P>> {
        Ok(CollisionDetectorNode {
            emergency_publisher: Topic::new(&self.emergency_topic)?,
            lidar_subscriber: Topic::new(&self.lidar_topic)?,
            odometry_subscriber: Topic::new(&self.odom_topic)?,
            digital_io_subscriber: Topic::new(&self.io_topic)?,

            // Default safety zones
            critical_zone: 0.3,
            warning_zone: 0.8,
            monitoring_zone: 2.0,

            // Default robot geometry
            robot_width: 0.6,
            robot_length: 0.8,
            safety_margin: 0.1,

            current_velocity: (0.0, 0.0, 0.0),
            current_pose: (0.0, 0.0, 0.0),

            collision_imminent: false,
            warning_active: false,
            obstacles_detected: Vec::new(),
            safety_sensors_active: false,

            velocity_dependent_zones: true,
            min_stopping_distance: 0.2,
            max_deceleration: 2.0,

            collision_history: VecDeque::new(),
            history_length: 5,

            safety_sensor_pins: vec![0, 1, 2, 3],

            last_lidar_time: 0,
            emergency_cooldown: 500,
            last_emergency_time: 0,
            processor: self.processor,
        })
    }
}
