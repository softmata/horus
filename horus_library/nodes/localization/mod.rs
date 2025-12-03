#![allow(clippy::needless_range_loop)] // Matrix indexing patterns are clearer with explicit indices

use crate::{Imu, LaserScan, Odometry};
use horus_core::error::HorusResult;

// Import algorithms from horus_library/algorithms
use crate::algorithms::ekf::EKF;
use crate::algorithms::sensor_fusion::SensorFusion;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Hub, Node, NodeInfo};
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

/// Localization Node - Robot position estimation using sensor fusion
///
/// Fuses odometry, IMU, and lidar data to estimate robot pose using
/// Extended Kalman Filter (EKF) for accurate localization.
///
/// This node is a thin wrapper around the pure algorithms in horus_library/algorithms.
///
/// # Hybrid Pattern
///
/// ```rust,ignore
/// let node = LocalizationNode::builder()
///     .with_filter(|odom| {
///         // Only publish when uncertainty is below threshold
///         if odom.pose_covariance[0] < 0.1 {
///             Some(odom)
///         } else {
///             None
///         }
///     })
///     .build()?;
/// ```
pub struct LocalizationNode<P = PassThrough<Odometry>>
where
    P: Processor<Odometry>,
{
    pose_publisher: Hub<Odometry>,
    odometry_subscriber: Hub<Odometry>,
    imu_subscriber: Hub<Imu>,
    lidar_subscriber: Hub<LaserScan>,

    // Algorithm instances
    ekf: EKF,
    angular_velocity_fusion: SensorFusion,

    // Localization parameters
    frame_id: String,
    child_frame_id: String,
    initial_pose_set: bool,

    // Timing
    last_update_time: u64,
    last_odometry_time: u64,
    last_imu_time: u64,

    // Reference landmarks for correction (simplified SLAM)
    landmarks: Vec<(f64, f64)>, // Known landmark positions
    landmark_detection_range: f64,

    // Processor for hybrid pattern
    processor: P,
}

impl LocalizationNode {
    /// Create a new localization node with default topic "pose"
    pub fn new() -> Result<Self> {
        Self::new_with_topics("pose", "odom", "imu", "lidar_scan")
    }

    /// Create a new localization node with custom topics
    pub fn new_with_topics(
        pose_topic: &str,
        odom_topic: &str,
        imu_topic: &str,
        lidar_topic: &str,
    ) -> Result<Self> {
        // Create EKF instance with default noise parameters
        let mut ekf = EKF::new();

        // Configure process noise (motion model uncertainty)
        let mut process_noise = [[0.0; 6]; 6];
        process_noise[0][0] = 0.1; // x position
        process_noise[1][1] = 0.1; // y position
        process_noise[2][2] = 0.05; // theta
        process_noise[3][3] = 0.2; // vx
        process_noise[4][4] = 0.2; // vy
        process_noise[5][5] = 0.1; // omega
        ekf.set_process_noise(process_noise);

        // Configure odometry measurement noise
        let mut odometry_noise = [[0.0; 3]; 3];
        odometry_noise[0][0] = 0.05; // x measurement noise
        odometry_noise[1][1] = 0.05; // y measurement noise
        odometry_noise[2][2] = 0.02; // theta measurement noise
        ekf.set_odometry_noise(odometry_noise);

        // Create sensor fusion for angular velocity (odometry + IMU)
        let angular_velocity_fusion = SensorFusion::new();

        Ok(Self {
            pose_publisher: Hub::new(pose_topic)?,
            odometry_subscriber: Hub::new(odom_topic)?,
            imu_subscriber: Hub::new(imu_topic)?,
            lidar_subscriber: Hub::new(lidar_topic)?,

            ekf,
            angular_velocity_fusion,

            frame_id: "map".to_string(),
            child_frame_id: "base_link".to_string(),
            initial_pose_set: false,

            last_update_time: 0,
            last_odometry_time: 0,
            last_imu_time: 0,

            landmarks: Vec::new(),
            landmark_detection_range: 10.0, // 10m detection range
            processor: PassThrough::new(),
        })
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> LocalizationNodeBuilder<PassThrough<Odometry>> {
        LocalizationNodeBuilder::new()
    }
}

impl<P> LocalizationNode<P>
where
    P: Processor<Odometry>,
{
    /// Set initial pose estimate
    pub fn set_initial_pose(&mut self, x: f64, y: f64, theta: f64) {
        let initial_state = [x, y, theta, 0.0, 0.0, 0.0];
        self.ekf.set_state(initial_state);

        // Reset covariance to moderate uncertainty
        let mut initial_cov = [[0.0; 6]; 6];
        for i in 0..6 {
            initial_cov[i][i] = 0.5;
        }
        self.ekf.set_covariance(initial_cov);

        self.initial_pose_set = true;
    }

    /// Set coordinate frame IDs
    pub fn set_frame_ids(&mut self, frame_id: &str, child_frame_id: &str) {
        self.frame_id = frame_id.to_string();
        self.child_frame_id = child_frame_id.to_string();
    }

    /// Add known landmark for localization correction
    pub fn add_landmark(&mut self, x: f64, y: f64) {
        self.landmarks.push((x, y));
    }

    /// Get current pose estimate
    pub fn get_pose(&self) -> (f64, f64, f64) {
        let state = self.ekf.get_state();
        (state[0], state[1], state[2])
    }

    /// Get current velocity estimate
    pub fn get_velocity(&self) -> (f64, f64, f64) {
        let state = self.ekf.get_state();
        (state[3], state[4], state[5])
    }

    /// Get pose uncertainty (position covariance)
    pub fn get_position_uncertainty(&self) -> f64 {
        let cov = self.ekf.get_covariance();
        (cov[0][0] + cov[1][1]).sqrt()
    }

    fn predict_step(&mut self, dt: f64) {
        // Use EKF prediction step
        self.ekf.predict(dt);
    }

    fn update_with_odometry(&mut self, odom: &Odometry) {
        if !self.initial_pose_set {
            // Initialize pose from first odometry reading
            self.set_initial_pose(odom.pose.x, odom.pose.y, odom.pose.theta);

            // Also initialize velocities
            let mut state = self.ekf.get_state();
            state[3] = odom.twist.linear[0]; // vx
            state[4] = odom.twist.linear[1]; // vy
            state[5] = odom.twist.angular[2]; // omega
            self.ekf.set_state(state);
            return;
        }

        // Measurement vector: [x, y, theta]
        let measurement = [odom.pose.x, odom.pose.y, odom.pose.theta];

        // Use EKF odometry update
        self.ekf.update_odometry(measurement);

        // Update velocities from odometry twist (direct copy)
        let mut state = self.ekf.get_state();
        state[3] = odom.twist.linear[0]; // vx
        state[4] = odom.twist.linear[1]; // vy
        state[5] = odom.twist.angular[2]; // omega
        self.ekf.set_state(state);
    }

    fn update_with_imu(&mut self, imu: &Imu) {
        if !self.initial_pose_set {
            return; // Need initial pose before using IMU
        }

        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();

        // Use sensor fusion for angular velocity (odometry + IMU)
        let state = self.ekf.get_state();
        let odom_omega = state[5];
        let imu_omega = imu.angular_velocity[2];

        // Clear old measurements and add new ones
        self.angular_velocity_fusion.clear();

        // Odometry has higher variance for angular velocity
        self.angular_velocity_fusion.add_measurement_with_time(
            "odom",
            odom_omega,
            0.1,
            current_time,
        );

        // IMU has lower variance for angular velocity (more accurate)
        self.angular_velocity_fusion.add_measurement_with_time(
            "imu",
            imu_omega,
            0.05,
            current_time,
        );

        // Fuse angular velocities
        if let Some(fused_omega) = self.angular_velocity_fusion.fuse() {
            let mut updated_state = state;
            updated_state[5] = fused_omega;
            self.ekf.set_state(updated_state);
        }

        // Use IMU accelerations to validate velocity changes (simplified)
        let accel_x = imu.linear_acceleration[0];
        let accel_y = imu.linear_acceleration[1];

        // Simple acceleration-based velocity correction
        let dt = 0.01; // Assume ~100Hz IMU rate
        let mut updated_state = self.ekf.get_state();
        updated_state[3] += accel_x * dt * 0.1; // Small correction factor
        updated_state[4] += accel_y * dt * 0.1;
        self.ekf.set_state(updated_state);
    }

    fn update_with_landmarks(&mut self, lidar: &LaserScan) {
        if !self.initial_pose_set || self.landmarks.is_empty() {
            return;
        }

        // Simplified landmark-based correction
        let state = self.ekf.get_state();
        let robot_x = state[0];
        let robot_y = state[1];
        let robot_theta = state[2];

        // Extract potential landmark observations from lidar
        for (i, &range) in lidar.ranges.iter().enumerate() {
            if range > 0.5 && range < self.landmark_detection_range as f32 {
                let angle =
                    lidar.angle_min as f64 + i as f64 * lidar.angle_increment as f64 + robot_theta;

                let observed_x = robot_x + range as f64 * angle.cos();
                let observed_y = robot_y + range as f64 * angle.sin();

                // Find closest known landmark
                let mut min_distance = f64::INFINITY;
                let mut closest_landmark = None;

                for &(lm_x, lm_y) in &self.landmarks {
                    let distance =
                        ((observed_x - lm_x).powi(2) + (observed_y - lm_y).powi(2)).sqrt();
                    if distance < min_distance && distance < 1.0 {
                        // 1m association threshold
                        min_distance = distance;
                        closest_landmark = Some((lm_x, lm_y));
                    }
                }

                // Apply landmark correction if association found
                if let Some((lm_x, lm_y)) = closest_landmark {
                    let correction_weight = 0.1;
                    let position_error_x = observed_x - lm_x;
                    let position_error_y = observed_y - lm_y;

                    // Correct robot position estimate
                    let mut updated_state = state;
                    updated_state[0] -= correction_weight * position_error_x;
                    updated_state[1] -= correction_weight * position_error_y;
                    self.ekf.set_state(updated_state);
                }
            }
        }
    }

    fn _normalize_angle(&self, angle: f64) -> f64 {
        let mut normalized = angle;
        while normalized > std::f64::consts::PI {
            normalized -= 2.0 * std::f64::consts::PI;
        }
        while normalized < -std::f64::consts::PI {
            normalized += 2.0 * std::f64::consts::PI;
        }
        normalized
    }

    fn publish_pose(&mut self) {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        let mut localized_pose = Odometry::new();

        // Set frame information
        localized_pose.frame_id = self
            .frame_id
            .clone()
            .into_bytes()
            .try_into()
            .unwrap_or([0; 32]);
        localized_pose.child_frame_id = self
            .child_frame_id
            .clone()
            .into_bytes()
            .try_into()
            .unwrap_or([0; 32]);

        // Get state from EKF
        let state = self.ekf.get_state();
        let covariance = self.ekf.get_covariance();

        // Set pose
        localized_pose.pose.x = state[0];
        localized_pose.pose.y = state[1];
        localized_pose.pose.theta = state[2];

        // Set twist
        localized_pose.twist.linear[0] = state[3];
        localized_pose.twist.linear[1] = state[4];
        localized_pose.twist.angular[2] = state[5];

        // Set covariances (simplified - only diagonal elements)
        for i in 0..6 {
            localized_pose.pose_covariance[i * 6 + i] = covariance[i][i];
        }

        localized_pose.timestamp = current_time;

        // Process through pipeline
        if let Some(processed) = self.processor.process(localized_pose) {
            let _ = self.pose_publisher.send(processed, &mut None);
        }
    }

    /// Reset localization (useful for relocalization)
    pub fn reset(&mut self) {
        // Reset EKF state
        self.ekf.set_state([0.0; 6]);

        // Reset covariance to high uncertainty
        let mut reset_cov = [[0.0; 6]; 6];
        for i in 0..6 {
            reset_cov[i][i] = 1.0;
        }
        self.ekf.set_covariance(reset_cov);

        self.initial_pose_set = false;
    }

    /// Check if localization is well-converged
    pub fn is_converged(&self) -> bool {
        self.initial_pose_set && self.get_position_uncertainty() < 0.3 // 30cm uncertainty
    }
}

impl<P> Node for LocalizationNode<P>
where
    P: Processor<Odometry>,
{
    fn name(&self) -> &'static str {
        "LocalizationNode"
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

        // Calculate time step
        let dt = if self.last_update_time > 0 {
            (current_time - self.last_update_time) as f64 / 1000.0
        } else {
            0.01 // 10ms default
        };

        if dt > 0.001 {
            // Minimum 1ms update interval
            // Prediction step
            self.predict_step(dt);
            self.last_update_time = current_time;
        }

        // Update with odometry data
        if let Some(odom) = self.odometry_subscriber.recv(&mut None) {
            if odom.timestamp > self.last_odometry_time {
                self.update_with_odometry(&odom);
                self.last_odometry_time = odom.timestamp;
            }
        }

        // Update with IMU data
        if let Some(imu) = self.imu_subscriber.recv(&mut None) {
            if imu.timestamp > self.last_imu_time {
                self.update_with_imu(&imu);
                self.last_imu_time = imu.timestamp;
            }
        }

        // Update with lidar landmarks
        if let Some(lidar) = self.lidar_subscriber.recv(&mut None) {
            self.update_with_landmarks(&lidar);
        }

        // Publish localized pose
        if self.initial_pose_set {
            self.publish_pose();
        }
    }
}

// Default impl removed - use Node::new() instead which returns HorusResult

/// Builder for LocalizationNode with processor configuration
pub struct LocalizationNodeBuilder<P>
where
    P: Processor<Odometry>,
{
    pose_topic: String,
    odom_topic: String,
    imu_topic: String,
    lidar_topic: String,
    processor: P,
}

impl LocalizationNodeBuilder<PassThrough<Odometry>> {
    /// Create a new builder with default PassThrough processor
    pub fn new() -> Self {
        Self {
            pose_topic: "pose".to_string(),
            odom_topic: "odom".to_string(),
            imu_topic: "imu".to_string(),
            lidar_topic: "lidar_scan".to_string(),
            processor: PassThrough::new(),
        }
    }
}

impl Default for LocalizationNodeBuilder<PassThrough<Odometry>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P> LocalizationNodeBuilder<P>
where
    P: Processor<Odometry>,
{
    /// Set pose output topic
    pub fn pose_topic(mut self, topic: &str) -> Self {
        self.pose_topic = topic.to_string();
        self
    }

    /// Set odometry input topic
    pub fn odom_topic(mut self, topic: &str) -> Self {
        self.odom_topic = topic.to_string();
        self
    }

    /// Set IMU input topic
    pub fn imu_topic(mut self, topic: &str) -> Self {
        self.imu_topic = topic.to_string();
        self
    }

    /// Set lidar input topic
    pub fn lidar_topic(mut self, topic: &str) -> Self {
        self.lidar_topic = topic.to_string();
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> LocalizationNodeBuilder<P2>
    where
        P2: Processor<Odometry>,
    {
        LocalizationNodeBuilder {
            pose_topic: self.pose_topic,
            odom_topic: self.odom_topic,
            imu_topic: self.imu_topic,
            lidar_topic: self.lidar_topic,
            processor,
        }
    }

    /// Set a closure-based processor
    pub fn with_closure<F>(
        self,
        f: F,
    ) -> LocalizationNodeBuilder<ClosureProcessor<Odometry, Odometry, F>>
    where
        F: FnMut(Odometry) -> Odometry + Send + 'static,
    {
        LocalizationNodeBuilder {
            pose_topic: self.pose_topic,
            odom_topic: self.odom_topic,
            imu_topic: self.imu_topic,
            lidar_topic: self.lidar_topic,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Set a filter-based processor
    pub fn with_filter<F>(
        self,
        f: F,
    ) -> LocalizationNodeBuilder<FilterProcessor<Odometry, Odometry, F>>
    where
        F: FnMut(Odometry) -> Option<Odometry> + Send + 'static,
    {
        LocalizationNodeBuilder {
            pose_topic: self.pose_topic,
            odom_topic: self.odom_topic,
            imu_topic: self.imu_topic,
            lidar_topic: self.lidar_topic,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor (pipe)
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> LocalizationNodeBuilder<Pipeline<Odometry, Odometry, Odometry, P, P2>>
    where
        P2: Processor<Odometry, Odometry>,
    {
        LocalizationNodeBuilder {
            pose_topic: self.pose_topic,
            odom_topic: self.odom_topic,
            imu_topic: self.imu_topic,
            lidar_topic: self.lidar_topic,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the node
    pub fn build(self) -> Result<LocalizationNode<P>> {
        // Create EKF instance with default noise parameters
        let mut ekf = EKF::new();

        // Configure process noise (motion model uncertainty)
        let mut process_noise = [[0.0; 6]; 6];
        process_noise[0][0] = 0.1; // x position
        process_noise[1][1] = 0.1; // y position
        process_noise[2][2] = 0.05; // theta
        process_noise[3][3] = 0.2; // vx
        process_noise[4][4] = 0.2; // vy
        process_noise[5][5] = 0.1; // omega
        ekf.set_process_noise(process_noise);

        // Configure odometry measurement noise
        let mut odometry_noise = [[0.0; 3]; 3];
        odometry_noise[0][0] = 0.05; // x measurement noise
        odometry_noise[1][1] = 0.05; // y measurement noise
        odometry_noise[2][2] = 0.02; // theta measurement noise
        ekf.set_odometry_noise(odometry_noise);

        // Create sensor fusion for angular velocity (odometry + IMU)
        let angular_velocity_fusion = SensorFusion::new();

        Ok(LocalizationNode {
            pose_publisher: Hub::new(&self.pose_topic)?,
            odometry_subscriber: Hub::new(&self.odom_topic)?,
            imu_subscriber: Hub::new(&self.imu_topic)?,
            lidar_subscriber: Hub::new(&self.lidar_topic)?,

            ekf,
            angular_velocity_fusion,

            frame_id: "map".to_string(),
            child_frame_id: "base_link".to_string(),
            initial_pose_set: false,

            last_update_time: 0,
            last_odometry_time: 0,
            last_imu_time: 0,

            landmarks: Vec::new(),
            landmark_detection_range: 10.0,
            processor: self.processor,
        })
    }
}
