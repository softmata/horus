/// Multi-robot formation control.
///
/// Three scout robots maintain a triangular formation while
/// moving toward a goal. Each robot runs its own controller
/// node in a separate namespace.

use horus::prelude::*;
use std::time::Duration;

message! {
    /// Robot's self-reported position
    RobotPose {
        robot_id: u32,
        x: f64,
        z: f64,
        theta: f64,
    }

    /// Formation command from coordinator
    FormationCmd {
        target_x: f64,
        target_z: f64,
        speed: f64,
    }
}

/// Individual scout controller — drives toward its formation position.
struct ScoutController {
    robot_id: u32,
    cmd_pub: Topic<CmdVel>,
    pose_pub: Topic<RobotPose>,
    formation_sub: Topic<FormationCmd>,
    formation_offset_x: f64,
    formation_offset_z: f64,
    x: f64,
    z: f64,
    theta: f64,
}

impl ScoutController {
    fn new(robot_id: u32, offset_x: f64, offset_z: f64) -> Result<Self> {
        Ok(Self {
            robot_id,
            cmd_pub: Topic::new(&format!("scout_{}/cmd_vel", robot_id))?,
            pose_pub: Topic::new("fleet/poses")?,
            formation_sub: Topic::new("fleet/formation_cmd")?,
            formation_offset_x: offset_x,
            formation_offset_z: offset_z,
            x: offset_x * 2.0, // Start spread out
            z: offset_z * 2.0,
            theta: 0.0,
        })
    }
}

impl Node for ScoutController {
    fn name(&self) -> &str {
        "ScoutController"
    }

    fn tick(&mut self) {
        // Get formation center from coordinator
        let (target_x, target_z) = if let Some(cmd) = self.formation_sub.recv() {
            (
                cmd.target_x + self.formation_offset_x,
                cmd.target_z + self.formation_offset_z,
            )
        } else {
            (self.formation_offset_x, self.formation_offset_z)
        };

        // Simple proportional control toward target
        let dx = target_x - self.x;
        let dz = target_z - self.z;
        let dist = (dx * dx + dz * dz).sqrt();

        let (linear, angular) = if dist > 0.05 {
            let desired_theta = dz.atan2(dx);
            let angle_err = desired_theta - self.theta;
            (
                (dist * 0.5).min(0.3) as f32,
                (angle_err * 1.5).clamp(-1.0, 1.0) as f32,
            )
        } else {
            (0.0, 0.0)
        };

        self.cmd_pub.send(CmdVel::new(linear, angular));

        // Simulate motion (in real deployment, odom feedback replaces this)
        self.x += linear as f64 * self.theta.cos() * 0.05;
        self.z += linear as f64 * self.theta.sin() * 0.05;
        self.theta += angular as f64 * 0.05;

        // Broadcast pose
        self.pose_pub.send(RobotPose {
            robot_id: self.robot_id,
            x: self.x,
            z: self.z,
            theta: self.theta,
        });
    }
}

/// Coordinates the formation — sends target center position.
struct FormationCoordinator {
    cmd_pub: Topic<FormationCmd>,
    pose_sub: Topic<RobotPose>,
    center_x: f64,
    center_z: f64,
    tick_count: u64,
}

impl FormationCoordinator {
    fn new() -> Result<Self> {
        Ok(Self {
            cmd_pub: Topic::new("fleet/formation_cmd")?,
            pose_sub: Topic::new("fleet/poses")?,
            center_x: 0.0,
            center_z: 0.0,
            tick_count: 0,
        })
    }
}

impl Node for FormationCoordinator {
    fn name(&self) -> &str {
        "FormationCoordinator"
    }

    fn tick(&mut self) {
        self.tick_count += 1;

        // Move formation center in a slow circle
        let t = self.tick_count as f64 * 0.01;
        self.center_x = 2.0 * t.cos();
        self.center_z = 2.0 * t.sin();

        self.cmd_pub.send(FormationCmd {
            target_x: self.center_x,
            target_z: self.center_z,
            speed: 0.2,
        });

        // Log received poses
        while let Some(pose) = self.pose_sub.recv() {
            if self.tick_count % 20 == 0 {
                hlog!(
                    "Scout {} at ({:.2}, {:.2})",
                    pose.robot_id, pose.x, pose.z
                );
            }
        }
    }
}

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new().tick_hz(20.0);

    // Three scout controllers
    scheduler.add(ScoutController::new(1, -1.0, -1.0)?).order(0).rate_hz(20.0).build()?;
    scheduler.add(ScoutController::new(2, 1.0, -1.0)?).order(0).rate_hz(20.0).build()?;
    scheduler.add(ScoutController::new(3, 0.0, 1.0)?).order(0).rate_hz(20.0).build()?;

    // Coordinator runs slower
    scheduler.add(FormationCoordinator::new()?).order(5).rate_hz(5.0).build()?;

    scheduler.run_for(Duration::from_secs(60))?;
    Ok(())
}
