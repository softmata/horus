use horus_core::core::LogSummary;
// Multi-robot coordination and fleet management message types
//
// This module provides messages for coordinating multiple robots,
// task assignment, swarm behavior, and fleet management systems.

use crate::messages::diagnostics::StatusLevel;
use crate::messages::geometry::{Pose2D, Twist};
use serde::{Deserialize, Serialize};
use serde_arrays;

/// Robot state information for fleet management
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct RobotState {
    /// Unique robot identifier
    pub robot_id: [u8; 32],
    /// Current pose in world coordinates
    pub pose: Pose2D,
    /// Current velocity
    pub velocity: Twist,
    /// Current operational status
    pub status: StatusLevel,
    /// Current battery level (0-100)
    pub battery_level: f32,
    /// Current task ID (0 = no task)
    pub current_task_id: u32,
    /// Robot capabilities (bitmask)
    pub capabilities: u32,
    /// Load/cargo capacity utilization (0.0-1.0)
    pub load_factor: f32,
    /// Robot type classification
    pub robot_type: RobotType,
    /// Priority level (0 = highest)
    pub priority: u8,
    /// Communication quality (0.0-1.0)
    pub comm_quality: f32,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

/// Robot type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
#[derive(Default)]
pub enum RobotType {
    /// General purpose mobile robot
    #[default]
    Mobile = 0,
    /// Manipulator arm
    Manipulator = 1,
    /// Aerial drone/UAV
    Aerial = 2,
    /// Marine/underwater vehicle
    Marine = 3,
    /// Stationary sensor/processing unit
    Stationary = 4,
    /// Specialized tool/equipment
    Tool = 5,
    /// Logistics/transport robot
    Transport = 6,
    /// Cleaning/maintenance robot
    Service = 7,
}

impl Default for RobotState {
    fn default() -> Self {
        Self {
            robot_id: [0; 32],
            pose: Pose2D::default(),
            velocity: Twist::default(),
            status: StatusLevel::Ok,
            battery_level: 100.0,
            current_task_id: 0,
            capabilities: 0,
            load_factor: 0.0,
            robot_type: RobotType::Mobile,
            priority: 5, // Medium priority
            comm_quality: 1.0,
            timestamp_ns: 0,
        }
    }
}

impl RobotState {
    /// Create a new robot state
    pub fn new(robot_id: &str, robot_type: RobotType) -> Self {
        let mut state = Self {
            robot_type,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
            ..Default::default()
        };

        // Set robot ID
        let id_bytes = robot_id.as_bytes();
        let len = id_bytes.len().min(31);
        state.robot_id[..len].copy_from_slice(&id_bytes[..len]);
        state.robot_id[len] = 0;

        state
    }

    /// Get robot ID as string
    pub fn robot_id_str(&self) -> String {
        let end = self.robot_id.iter().position(|&b| b == 0).unwrap_or(32);
        String::from_utf8_lossy(&self.robot_id[..end]).into_owned()
    }

    /// Check if robot is available for new tasks
    pub fn is_available(&self) -> bool {
        matches!(self.status, StatusLevel::Ok)
            && self.current_task_id == 0
            && self.battery_level > 20.0
            && self.comm_quality > 0.5
    }

    /// Check if robot needs maintenance
    pub fn needs_maintenance(&self) -> bool {
        self.battery_level < 15.0
            || matches!(self.status, StatusLevel::Error | StatusLevel::Fatal)
            || self.comm_quality < 0.3
    }

    /// Set capability flag
    pub fn set_capability(&mut self, capability: RobotCapability, enabled: bool) {
        if enabled {
            self.capabilities |= capability as u32;
        } else {
            self.capabilities &= !(capability as u32);
        }
    }

    /// Check if robot has capability
    pub fn has_capability(&self, capability: RobotCapability) -> bool {
        (self.capabilities & (capability as u32)) != 0
    }

    /// Update position and velocity
    pub fn update_motion(&mut self, pose: Pose2D, velocity: Twist) {
        self.pose = pose;
        self.velocity = velocity;
        self.timestamp_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
    }
}

/// Robot capability flags
#[derive(Debug, Clone, Copy)]
#[repr(u32)]
pub enum RobotCapability {
    /// Can navigate autonomously
    Navigation = 1 << 0,
    /// Can manipulate objects
    Manipulation = 1 << 1,
    /// Has vision/camera capability
    Vision = 1 << 2,
    /// Can carry/transport items
    Transport = 1 << 3,
    /// Can perform cleaning tasks
    Cleaning = 1 << 4,
    /// Can inspect/monitor
    Inspection = 1 << 5,
    /// Can communicate with external systems
    Communication = 1 << 6,
    /// Emergency response capability
    Emergency = 1 << 7,
    /// Precision positioning capability
    Precision = 1 << 8,
    /// Outdoor operation capability
    Outdoor = 1 << 9,
    /// Underwater operation capability
    Underwater = 1 << 10,
    /// Aerial operation capability
    Aerial = 1 << 11,
    /// Heavy lifting capability
    HeavyLifting = 1 << 12,
    /// Hazardous environment operation
    Hazmat = 1 << 13,
    /// Real-time processing capability
    RealTime = 1 << 14,
    /// Multi-robot coordination capability
    Swarm = 1 << 15,
}

/// Fleet status overview
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FleetStatus {
    /// Array of robot states (max 64 robots)
    #[serde(with = "serde_arrays")]
    pub robots: [RobotState; 64],
    /// Number of active robots
    pub robot_count: u8,
    /// Fleet identifier
    pub fleet_id: [u8; 32],
    /// Total active tasks
    pub active_tasks: u32,
    /// Fleet coordination mode
    pub coordination_mode: CoordinationMode,
    /// Central coordinator robot ID (if any)
    pub coordinator_id: [u8; 32],
    /// Fleet-wide emergency status
    pub emergency_active: bool,
    /// Average fleet battery level
    pub average_battery: f32,
    /// Fleet communication health (0.0-1.0)
    pub comm_health: f32,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

/// Coordination mode for fleet operation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
#[derive(Default)]
pub enum CoordinationMode {
    /// Decentralized - robots coordinate directly
    #[default]
    Decentralized = 0,
    /// Centralized - single coordinator assigns tasks
    Centralized = 1,
    /// Hierarchical - multiple levels of coordination
    Hierarchical = 2,
    /// Market-based - robots bid for tasks
    MarketBased = 3,
    /// Swarm behavior - emergent coordination
    Swarm = 4,
}

impl Default for FleetStatus {
    fn default() -> Self {
        Self {
            robots: [RobotState::default(); 64],
            robot_count: 0,
            fleet_id: [0; 32],
            active_tasks: 0,
            coordination_mode: CoordinationMode::Decentralized,
            coordinator_id: [0; 32],
            emergency_active: false,
            average_battery: 0.0,
            comm_health: 1.0,
            timestamp_ns: 0,
        }
    }
}

impl FleetStatus {
    /// Create a new fleet status
    pub fn new(fleet_id: &str) -> Self {
        let mut status = Self {
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
            ..Default::default()
        };

        // Set fleet ID
        let id_bytes = fleet_id.as_bytes();
        let len = id_bytes.len().min(31);
        status.fleet_id[..len].copy_from_slice(&id_bytes[..len]);
        status.fleet_id[len] = 0;

        status
    }

    /// Add or update robot in fleet
    pub fn update_robot(&mut self, robot_state: RobotState) -> Result<(), &'static str> {
        // Find existing robot or add new one
        let robot_id_str = robot_state.robot_id_str();

        for i in 0..self.robot_count {
            if self.robots[i as usize].robot_id_str() == robot_id_str {
                self.robots[i as usize] = robot_state;
                self.update_statistics();
                return Ok(());
            }
        }

        // Add new robot
        if self.robot_count >= 64 {
            return Err("Maximum 64 robots supported");
        }

        self.robots[self.robot_count as usize] = robot_state;
        self.robot_count += 1;
        self.update_statistics();
        Ok(())
    }

    /// Remove robot from fleet
    pub fn remove_robot(&mut self, robot_id: &str) -> bool {
        for i in 0..self.robot_count {
            if self.robots[i as usize].robot_id_str() == robot_id {
                // Shift remaining robots down
                for j in i..self.robot_count.saturating_sub(1) {
                    self.robots[j as usize] = self.robots[(j + 1) as usize];
                }
                self.robot_count -= 1;
                self.update_statistics();
                return true;
            }
        }
        false
    }

    /// Get active robots
    pub fn get_robots(&self) -> &[RobotState] {
        &self.robots[..self.robot_count as usize]
    }

    /// Get available robots for task assignment
    pub fn available_robots(&self) -> Vec<&RobotState> {
        self.get_robots()
            .iter()
            .filter(|robot| robot.is_available())
            .collect()
    }

    /// Get robots needing maintenance
    pub fn maintenance_needed(&self) -> Vec<&RobotState> {
        self.get_robots()
            .iter()
            .filter(|robot| robot.needs_maintenance())
            .collect()
    }

    /// Update fleet statistics
    fn update_statistics(&mut self) {
        if self.robot_count == 0 {
            self.average_battery = 0.0;
            self.comm_health = 0.0;
            return;
        }

        // Calculate average battery
        let battery_sum: f32 = self.robots[..self.robot_count as usize]
            .iter()
            .map(|r| r.battery_level)
            .sum();
        self.average_battery = battery_sum / self.robot_count as f32;

        // Calculate communication health
        let comm_sum: f32 = self.robots[..self.robot_count as usize]
            .iter()
            .map(|r| r.comm_quality)
            .sum();
        self.comm_health = comm_sum / self.robot_count as f32;

        // Check emergency status
        self.emergency_active = self.robots[..self.robot_count as usize]
            .iter()
            .any(|r| matches!(r.status, StatusLevel::Fatal));

        self.timestamp_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
    }
}

/// Task assignment message
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct TaskAssignment {
    /// Unique task identifier
    pub task_id: u32,
    /// Robot assigned to this task
    pub robot_id: [u8; 32],
    /// Task type/category
    pub task_type: TaskType,
    /// Task priority (0 = highest)
    pub priority: u8,
    /// Task deadline (0 = no deadline)
    pub deadline: u64,
    /// Estimated task duration in seconds
    pub estimated_duration: f64,
    /// Required robot capabilities (bitmask)
    pub required_capabilities: u32,
    /// Task parameters (flexible data)
    #[serde(with = "serde_arrays")]
    pub parameters: [u8; 64],
    /// Task assignment timestamp
    pub assigned_time: u64,
    /// Expected completion time
    pub expected_completion: u64,
    /// Task status
    pub status: TaskStatus,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

/// Task type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
#[derive(Default)]
pub enum TaskType {
    /// Navigate to a location
    #[default]
    Navigation = 0,
    /// Pick and place operation
    Manipulation = 1,
    /// Transport cargo
    Transport = 2,
    /// Inspect area or object
    Inspection = 3,
    /// Cleaning operation
    Cleaning = 4,
    /// Surveillance/monitoring
    Surveillance = 5,
    /// Emergency response
    Emergency = 6,
    /// Maintenance task
    Maintenance = 7,
    /// Data collection
    DataCollection = 8,
    /// Communication relay
    Communication = 9,
    /// Formation/coordination task
    Formation = 10,
    /// Custom user-defined task
    Custom = 255,
}

/// Task execution status
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
#[derive(Default)]
pub enum TaskStatus {
    /// Task assigned but not started
    #[default]
    Assigned = 0,
    /// Task in progress
    InProgress = 1,
    /// Task completed successfully
    Completed = 2,
    /// Task failed
    Failed = 3,
    /// Task cancelled
    Cancelled = 4,
    /// Task paused/suspended
    Paused = 5,
    /// Task aborted due to emergency
    Aborted = 6,
}

impl Default for TaskAssignment {
    fn default() -> Self {
        Self {
            task_id: 0,
            robot_id: [0; 32],
            task_type: TaskType::Navigation,
            priority: 5,
            deadline: 0,
            estimated_duration: 0.0,
            required_capabilities: 0,
            parameters: [0; 64],
            assigned_time: 0,
            expected_completion: 0,
            status: TaskStatus::Assigned,
            timestamp_ns: 0,
        }
    }
}

impl TaskAssignment {
    /// Create a new task assignment
    pub fn new(task_id: u32, robot_id: &str, task_type: TaskType) -> Self {
        let mut assignment = Self {
            task_id,
            task_type,
            assigned_time: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
            ..Default::default()
        };

        // Set robot ID
        let id_bytes = robot_id.as_bytes();
        let len = id_bytes.len().min(31);
        assignment.robot_id[..len].copy_from_slice(&id_bytes[..len]);
        assignment.robot_id[len] = 0;

        assignment
    }

    /// Set task priority and deadline
    pub fn with_priority_deadline(mut self, priority: u8, deadline: u64) -> Self {
        self.priority = priority;
        self.deadline = deadline;
        self
    }

    /// Set required capabilities
    pub fn with_capabilities(mut self, capabilities: u32) -> Self {
        self.required_capabilities = capabilities;
        self
    }

    /// Update task status
    pub fn update_status(&mut self, status: TaskStatus) {
        self.status = status;
        self.timestamp_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        if status == TaskStatus::Completed {
            self.expected_completion = self.timestamp_ns;
        }
    }

    /// Check if task is overdue
    pub fn is_overdue(&self) -> bool {
        if self.deadline == 0 {
            return false;
        }

        let current_time = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        current_time > self.deadline
            && !matches!(
                self.status,
                TaskStatus::Completed | TaskStatus::Cancelled | TaskStatus::Aborted
            )
    }

    /// Get robot ID as string
    pub fn robot_id_str(&self) -> String {
        let end = self.robot_id.iter().position(|&b| b == 0).unwrap_or(32);
        String::from_utf8_lossy(&self.robot_id[..end]).into_owned()
    }
}

/// Formation control parameters for coordinated movement
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct FormationControl {
    /// Formation type
    pub formation_type: FormationType,
    /// Leader robot ID (for leader-follower formations)
    pub leader_id: [u8; 32],
    /// Robot's position in formation (index)
    pub formation_index: u8,
    /// Desired relative position [x, y] from formation center/leader
    pub relative_position: [f64; 2],
    /// Desired relative orientation
    pub relative_orientation: f64,
    /// Formation scale factor
    pub scale: f64,
    /// Spacing between robots
    pub spacing: f64,
    /// Formation movement velocity
    pub formation_velocity: Twist,
    /// Stiffness of formation (0.0-1.0)
    pub stiffness: f32,
    /// Enable formation keeping
    pub enabled: bool,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

/// Formation type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
#[derive(Default)]
pub enum FormationType {
    /// Line formation
    #[default]
    Line = 0,
    /// Column formation
    Column = 1,
    /// Wedge/V formation
    Wedge = 2,
    /// Circle formation
    Circle = 3,
    /// Grid formation
    Grid = 4,
    /// Diamond formation
    Diamond = 5,
    /// Leader-follower
    LeaderFollower = 6,
    /// Custom formation
    Custom = 255,
}

impl FormationControl {
    /// Create a leader-follower formation
    pub fn leader_follower(leader_id: &str, relative_pos: [f64; 2]) -> Self {
        let mut formation = Self {
            formation_type: FormationType::LeaderFollower,
            relative_position: relative_pos,
            spacing: 1.0,
            scale: 1.0,
            stiffness: 0.8,
            enabled: true,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
            ..Default::default()
        };

        // Set leader ID
        let id_bytes = leader_id.as_bytes();
        let len = id_bytes.len().min(31);
        formation.leader_id[..len].copy_from_slice(&id_bytes[..len]);
        formation.leader_id[len] = 0;

        formation
    }

    /// Create a circle formation
    pub fn circle(index: u8, total_robots: u8, radius: f64) -> Self {
        let angle = 2.0 * std::f64::consts::PI * index as f64 / total_robots as f64;
        let relative_pos = [radius * angle.cos(), radius * angle.sin()];

        Self {
            formation_type: FormationType::Circle,
            formation_index: index,
            relative_position: relative_pos,
            spacing: radius,
            scale: 1.0,
            stiffness: 0.8,
            enabled: true,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
            ..Default::default()
        }
    }

    /// Enable/disable formation control
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        self.timestamp_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;
    }
}

/// Auction bid for task allocation (market-based coordination)
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct AuctionBid {
    /// Task being bid on
    pub task_id: u32,
    /// Bidding robot ID
    pub robot_id: [u8; 32],
    /// Bid value (cost/utility)
    pub bid_value: f64,
    /// Estimated completion time
    pub estimated_time: f64,
    /// Robot's current capability score for this task
    pub capability_score: f32,
    /// Robot's current availability
    pub availability: f32,
    /// Bid submission time
    pub bid_time: u64,
    /// Bid expiration time
    pub expiration_time: u64,
    /// Bid status
    pub status: BidStatus,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

/// Bid status enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
#[derive(Default)]
pub enum BidStatus {
    /// Bid submitted and active
    #[default]
    Active = 0,
    /// Bid won the auction
    Won = 1,
    /// Bid lost the auction
    Lost = 2,
    /// Bid withdrawn by robot
    Withdrawn = 3,
    /// Bid expired
    Expired = 4,
}

impl AuctionBid {
    /// Create a new auction bid
    pub fn new(task_id: u32, robot_id: &str, bid_value: f64) -> Self {
        let mut bid = Self {
            task_id,
            bid_value,
            bid_time: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
            ..Default::default()
        };

        // Set robot ID
        let id_bytes = robot_id.as_bytes();
        let len = id_bytes.len().min(31);
        bid.robot_id[..len].copy_from_slice(&id_bytes[..len]);
        bid.robot_id[len] = 0;

        bid
    }

    /// Check if bid is still valid
    pub fn is_valid(&self) -> bool {
        if self.expiration_time == 0 {
            return matches!(self.status, BidStatus::Active);
        }

        let current_time = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        current_time <= self.expiration_time && matches!(self.status, BidStatus::Active)
    }

    /// Calculate total bid score (lower is better)
    pub fn total_score(&self) -> f64 {
        // Combine bid value, time, capability, and availability
        self.bid_value * (1.0 + self.estimated_time)
            / (self.capability_score as f64 * self.availability as f64).max(0.1)
    }
}

impl LogSummary for RobotState {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for FleetStatus {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for TaskAssignment {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for FormationControl {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for AuctionBid {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for RobotType {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for RobotCapability {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for CoordinationMode {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for TaskType {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for TaskStatus {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for FormationType {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

impl LogSummary for BidStatus {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}
