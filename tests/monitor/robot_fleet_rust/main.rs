// Complete Fleet Management System - Rust
// This demonstrates a full robotics application with:
// - Multiple sensor nodes (Camera, LIDAR, IMU, GPS)
// - Control nodes (Navigation, Obstacle Avoidance)
// - Actuator nodes (Motor Controller, Arm Controller)
// - Monitoring nodes (Battery Monitor, System Health)

use horus::prelude::*;
use horus_library::messages::cmd_vel::CmdVel;
use horus_library::messages::sensor::{Imu, LaserScan, Odometry};
use horus_library::messages::vision::{Image, ImageEncoding};
use std::time::{SystemTime, UNIX_EPOCH};

// ============================================================================
// CUSTOM MESSAGES
// ============================================================================

message!(GpsCoordinates = (f64, f64, f64)); // lat, lon, altitude
message!(BatteryStatus = (f32, bool)); // voltage, is_charging
message!(SystemHealth = (u8, u32)); // health_percent, error_count
message!(ObstacleAlert = (f32, f32)); // distance, angle

// ============================================================================
// SENSOR NODES
// ============================================================================

node! {
    CameraNode {
        pub {
            camera: Image -> "sensors.camera",
        }

        sub {}

        data {
            frame_count: u64 = 0,
        }

        tick(ctx) {
            // Simulate camera frame capture
            self.frame_count += 1;

            // Create empty image data (in production, you'd capture actual frames)
            let data = Vec::new();

            let mut img = Image::new(640, 480, ImageEncoding::Rgb8, data);
            img.timestamp = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;

            self.camera.send(img, &mut ctx).ok();
        }
    }
}

node! {
    LidarNode {
        pub {
            lidar: LaserScan -> "sensors.lidar",
        }

        sub {}

        data {
            scan_count: u64 = 0,
        }

        tick(ctx) {
            self.scan_count += 1;

            // Simulate LIDAR scan (360 degree, 1 degree resolution)
            let mut scan = LaserScan::new();
            for i in 0..360 {
                // Simulate varying distances (2-10 meters)
                scan.ranges[i] = 2.0 + (i as f32 * 0.02).sin() * 8.0;
            }
            scan.timestamp = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;

            self.lidar.send(scan, &mut ctx).ok();
        }
    }
}

node! {
    ImuNode {
        pub {
            imu: Imu -> "sensors.imu",
        }

        sub {}

        data {
            sample_count: u64 = 0,
        }

        tick(ctx) {
            self.sample_count += 1;
            let t = self.sample_count as f64 * 0.01;

            let mut imu = Imu::new();
            imu.orientation = [
                (t * 0.5).cos(),
                (t * 0.3).sin(),
                (t * 0.7).cos(),
                (t * 0.4).sin(),
            ];
            imu.angular_velocity = [0.01, -0.02, 0.05];
            imu.linear_acceleration = [0.0, 0.0, 9.81];
            imu.timestamp = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;

            self.imu.send(imu, &mut ctx).ok();
        }
    }
}

node! {
    GpsNode {
        pub {
            gps: GpsCoordinates -> "sensors.gps",
        }

        sub {}

        data {
            position: (f64, f64, f64) = (37.7749, -122.4194, 10.0),
        }

        tick(ctx) {
            // Simulate slight GPS drift
            self.position.0 += 0.00001 * (rand::random::<f64>() - 0.5);
            self.position.1 += 0.00001 * (rand::random::<f64>() - 0.5);

            let coords = GpsCoordinates(
                self.position.0,
                self.position.1,
                self.position.2
            );

            self.gps.send(coords, &mut ctx).ok();
        }
    }
}

// ============================================================================
// CONTROL NODES
// ============================================================================

node! {
    NavigationNode {
        pub {
            cmd: CmdVel -> "control.cmd_vel",
        }

        sub {
            gps: GpsCoordinates -> "sensors.gps",
            lidar: LaserScan -> "sensors.lidar",
        }

        data {
            waypoint: (f64, f64) = (37.7750, -122.4195),
        }

        tick(ctx) {
            // Read GPS position
            if let Some(gps_data) = self.gps.recv(&mut ctx) {
                // Simple navigation: move toward waypoint
                let dx = self.waypoint.0 - gps_data.0;
                let dy = self.waypoint.1 - gps_data.1;
                let distance = (dx * dx + dy * dy).sqrt();

                if distance > 0.0001 {
                    let linear = (distance * 1000.0).min(1.5) as f32;
                    let angular = (dy.atan2(dx) * 0.5) as f32;

                    let cmd = CmdVel::new(linear, angular);
                    self.cmd.send(cmd, &mut ctx).ok();
                }
            }
        }
    }
}

node! {
    ObstacleAvoidanceNode {
        pub {
            alert: ObstacleAlert -> "safety.obstacle_alert",
            override_cmd: CmdVel -> "control.cmd_vel_override",
        }

        sub {
            lidar: LaserScan -> "sensors.lidar",
        }

        tick(ctx) {
            if let Some(scan) = self.lidar.recv(&mut ctx) {
                // Check front 60 degrees for obstacles
                let start_idx = 150; // -30 degrees
                let end_idx = 210;   // +30 degrees

                let mut min_dist = f32::MAX;
                let mut min_angle = 0.0f32;

                for i in start_idx..end_idx {
                    if scan.ranges[i] < min_dist {
                        min_dist = scan.ranges[i];
                        min_angle = scan.angle_min + (i as f32 * scan.angle_increment);
                    }
                }

                // If obstacle within 1.5m, send alert and emergency stop
                if min_dist < 1.5 {
                    let alert_msg = ObstacleAlert(min_dist, min_angle);
                    self.alert.send(alert_msg, &mut ctx).ok();

                    // Emergency stop
                    let stop = CmdVel::zero();
                    self.override_cmd.send(stop, &mut ctx).ok();
                }
            }
        }
    }
}

// ============================================================================
// ACTUATOR NODES
// ============================================================================

node! {
    MotorControllerNode {
        pub {
            odometry: Odometry -> "sensors.odometry",
        }

        sub {
            cmd: CmdVel -> "control.cmd_vel",
            override_cmd: CmdVel -> "control.cmd_vel_override",
        }

        data {
            current_velocity: (f32, f32) = (0.0, 0.0),
        }

        tick(ctx) {
            // Check for emergency override first
            if let Some(override_vel) = self.override_cmd.recv(&mut ctx) {
                self.current_velocity = (override_vel.linear, override_vel.angular);
            } else if let Some(cmd_vel) = self.cmd.recv(&mut ctx) {
                self.current_velocity = (cmd_vel.linear, cmd_vel.angular);
            }

            // Publish odometry feedback
            let mut odom = Odometry::new();
            odom.twist.linear[0] = self.current_velocity.0 as f64;
            odom.twist.angular[2] = self.current_velocity.1 as f64;
            odom.timestamp = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;

            self.odometry.send(odom, &mut ctx).ok();
        }
    }
}

// ============================================================================
// MONITORING NODES
// ============================================================================

node! {
    BatteryMonitorNode {
        pub {
            battery: BatteryStatus -> "system.battery",
        }

        sub {}

        data {
            voltage: f32 = 12.6,
        }

        tick(ctx) {
            // Simulate battery drain
            self.voltage -= 0.0001;
            if self.voltage < 10.5 {
                self.voltage = 12.6; // Simulate recharge
            }

            let is_charging = self.voltage > 12.4;
            let status = BatteryStatus(self.voltage, is_charging);

            self.battery.send(status, &mut ctx).ok();
        }
    }
}

node! {
    SystemHealthNode {
        pub {
            health: SystemHealth -> "system.health",
        }

        sub {}

        data {
            error_count: u32 = 0,
        }

        tick(ctx) {
            // Random error simulation
            if rand::random::<f32>() > 0.95 {
                self.error_count += 1;
            }

            let health_percent = ((1000 - self.error_count) * 100 / 1000).min(100) as u8;
            let health_msg = SystemHealth(health_percent, self.error_count);

            self.health.send(health_msg, &mut ctx).ok();
        }
    }
}

// ============================================================================
// MAIN
// ============================================================================

fn main() -> HorusResult<()> {
    println!(" Starting Robot Fleet Management System (Rust)");
    println!(" Monitor available at: http://localhost:8080");
    println!(" Run 'horus monitor' in another terminal to view\n");

    let mut scheduler = Scheduler::new();

    // Sensor Layer (Priority 0-9: Highest)
    scheduler.add(Box::new(CameraNode::new()), 0, Some(true));
    scheduler.add(Box::new(LidarNode::new()), 1, Some(true));
    scheduler.add(Box::new(ImuNode::new()), 2, Some(true));
    scheduler.add(Box::new(GpsNode::new()), 3, Some(true));

    // Control Layer (Priority 10-19: High)
    scheduler.add(Box::new(ObstacleAvoidanceNode::new()), 10, Some(true));
    scheduler.add(Box::new(NavigationNode::new()), 11, Some(true));

    // Actuation Layer (Priority 20-29: Medium)
    scheduler.add(Box::new(MotorControllerNode::new()), 20, Some(true));

    // Monitoring Layer (Priority 30-39: Low)
    scheduler.add(Box::new(BatteryMonitorNode::new()), 30, Some(true));
    scheduler.add(Box::new(SystemHealthNode::new()), 31, Some(true));

    println!(" All nodes registered");
    println!(" Starting scheduler...\n");

    scheduler.run()
}
