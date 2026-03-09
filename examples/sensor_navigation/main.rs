/// Sensor-based navigation through an obstacle course.
///
/// Multi-node pipeline:
///   LiDAR (100Hz) → ObstacleDetector (50Hz) → Navigator (20Hz) → MotorDriver (50Hz)
///
/// Demonstrates multi-rate scheduling, sensor processing, and reactive control.

use horus::prelude::*;
use std::time::Duration;

message! {
    /// Detected obstacle bearing and distance
    ObstacleAlert {
        angle_rad: f64,
        distance_m: f64,
        sector: u32,
    }

    /// Navigation waypoint
    NavGoal {
        target_x: f64,
        target_z: f64,
        reached: u8,
    }
}

// ── LiDAR Processor ────────────────────────────────────────────────────────

/// Processes raw LaserScan data and publishes obstacle alerts.
struct LidarProcessor {
    scan_sub: Topic<LaserScan>,
    alert_pub: Topic<ObstacleAlert>,
    min_distance: f32,
}

impl LidarProcessor {
    fn new() -> Result<Self> {
        Ok(Self {
            scan_sub: Topic::new("lidar/scan")?,
            alert_pub: Topic::new("obstacles")?,
            min_distance: 0.5, // 50cm warning distance
        })
    }
}

impl Node for LidarProcessor {
    fn name(&self) -> &str {
        "LidarProcessor"
    }

    fn tick(&mut self) {
        if let Some(scan) = self.scan_sub.recv() {
            // Check if any range is below threshold
            if scan.range < self.min_distance {
                let alert = ObstacleAlert {
                    angle_rad: scan.angle_min as f64,
                    distance_m: scan.range as f64,
                    sector: 0,
                };
                self.alert_pub.send(alert);
            }
        }
    }
}

// ── Navigator ──────────────────────────────────────────────────────────────

/// Reactive navigator — drives forward, turns away from obstacles.
struct Navigator {
    alert_sub: Topic<ObstacleAlert>,
    imu_sub: Topic<Imu>,
    cmd_pub: Topic<CmdVel>,
    goal_pub: Topic<NavGoal>,
    obstacle_detected: bool,
    turn_ticks: u32,
}

impl Navigator {
    fn new() -> Result<Self> {
        Ok(Self {
            alert_sub: Topic::new("obstacles")?,
            imu_sub: Topic::new("imu/data")?,
            cmd_pub: Topic::new("cmd_vel")?,
            goal_pub: Topic::new("nav/goal")?,
            obstacle_detected: false,
            turn_ticks: 0,
        })
    }
}

impl Node for Navigator {
    fn name(&self) -> &str {
        "Navigator"
    }

    fn tick(&mut self) {
        // Check for obstacles
        if let Some(alert) = self.alert_sub.recv() {
            self.obstacle_detected = true;
            self.turn_ticks = 40; // Turn for ~2 seconds at 20Hz
            hlog!(
                "Obstacle at {:.2}m, angle={:.1}deg — turning",
                alert.distance_m,
                alert.angle_rad.to_degrees()
            );
        }

        let cmd = if self.turn_ticks > 0 {
            self.turn_ticks -= 1;
            if self.turn_ticks == 0 {
                self.obstacle_detected = false;
            }
            // Turn away from obstacle
            CmdVel::new(0.05, 0.8)
        } else {
            // Drive forward
            CmdVel::new(0.3, 0.0)
        };

        self.cmd_pub.send(cmd);

        // Read IMU for heading (logging only)
        if let Some(imu) = self.imu_sub.recv() {
            hlog_every!(50, "heading gyro_z={:.3}", imu.angular_velocity[2]);
        }
    }
}

// ── Telemetry Logger ───────────────────────────────────────────────────────

/// Low-rate logger for debugging and post-mortem analysis.
struct TelemetryLogger {
    cmd_sub: Topic<CmdVel>,
    imu_sub: Topic<Imu>,
    tick_count: u64,
}

impl TelemetryLogger {
    fn new() -> Result<Self> {
        Ok(Self {
            cmd_sub: Topic::new("cmd_vel")?,
            imu_sub: Topic::new("imu/data")?,
            tick_count: 0,
        })
    }
}

impl Node for TelemetryLogger {
    fn name(&self) -> &str {
        "TelemetryLogger"
    }

    fn tick(&mut self) {
        self.tick_count += 1;

        let cmd_info = self.cmd_sub.recv()
            .map(|c| format!("lin={:.2} ang={:.2}", c.linear, c.angular))
            .unwrap_or_else(|| "no cmd".to_string());

        let imu_info = self.imu_sub.recv()
            .map(|i| format!("acc=[{:.1},{:.1},{:.1}]",
                i.linear_acceleration[0],
                i.linear_acceleration[1],
                i.linear_acceleration[2]))
            .unwrap_or_else(|| "no imu".to_string());

        hlog!("[T{}] {} | {}", self.tick_count, cmd_info, imu_info);
    }
}

// ── Frame Setup ────────────────────────────────────────────────────────────

/// Publishes static sensor transforms.
struct SensorFrames {
    hf: HFrame,
}

impl SensorFrames {
    fn new() -> Result<Self> {
        let hf = HFrame::new();
        hf.add_frame("map").build()?;
        hf.add_frame("odom").parent("map").build()?;
        hf.add_frame("base_link").parent("odom").build()?;

        // Static sensor mounts (from URDF)
        hf.add_frame("lidar_link")
            .parent("base_link")
            .static_transform(&Transform::xyz(0.05, 0.0, 0.10))
            .build()?;
        hf.add_frame("imu_link")
            .parent("base_link")
            .static_transform(&Transform::xyz(0.0, 0.0, 0.06))
            .build()?;

        Ok(Self { hf })
    }
}

impl Node for SensorFrames {
    fn name(&self) -> &str {
        "SensorFrames"
    }

    fn tick(&mut self) {
        // Static frames don't need updating, but we keep the node alive
        // so horus frame tree shows the full tree
    }
}

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new().tick_hz(100.0);

    // Sensor pipeline: fast → slow
    scheduler.add(LidarProcessor::new()?).order(0).rate_hz(100.0).build()?;
    scheduler.add(Navigator::new()?).order(1).rate_hz(20.0).build()?;
    scheduler.add(SensorFrames::new()?).order(2).rate_hz(1.0).build()?;
    scheduler.add(TelemetryLogger::new()?).order(10).rate_hz(2.0).build()?;

    scheduler.run_for(Duration::from_secs(60))?;
    Ok(())
}
