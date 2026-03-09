/// 6-DOF robot arm controller.
///
/// Demonstrates joint-space control, frame transforms, and services.
/// The arm sweeps through a predefined joint trajectory.

use horus::prelude::*;
use std::time::Duration;

message! {
    /// Joint positions for all 6 joints (radians)
    JointState {
        j1: f64,
        j2: f64,
        j3: f64,
        j4: f64,
        j5: f64,
        j6: f64,
    }

    /// End-effector pose in Cartesian space
    EndEffectorPose {
        x: f64,
        y: f64,
        z: f64,
        roll: f64,
        pitch: f64,
        yaw: f64,
    }
}

service! {
    /// Move the arm to a named position
    MoveToNamed {
        request {
            position_name: u64,
        }
        response {
            success: u8,
            reached_j1: f64,
            reached_j2: f64,
            reached_j3: f64,
        }
    }
}

/// Predefined arm positions
struct ArmPositions;
impl ArmPositions {
    fn home() -> JointState {
        JointState { j1: 0.0, j2: 0.0, j3: 0.0, j4: 0.0, j5: 0.0, j6: 0.0 }
    }
    fn ready() -> JointState {
        JointState { j1: 0.0, j2: -0.5, j3: 1.0, j4: 0.0, j5: -0.5, j6: 0.0 }
    }
    fn pick() -> JointState {
        JointState { j1: 0.3, j2: -0.8, j3: 1.4, j4: 0.0, j5: -0.6, j6: 0.0 }
    }
    fn place() -> JointState {
        JointState { j1: -0.3, j2: -0.8, j3: 1.4, j4: 0.0, j5: -0.6, j6: 0.0 }
    }
}

/// Joint trajectory controller — interpolates between waypoints.
struct TrajectoryController {
    joint_pub: Topic<JointState>,
    ee_pub: Topic<EndEffectorPose>,
    waypoints: Vec<JointState>,
    current_wp: usize,
    progress: f64, // 0.0 to 1.0 within current segment
    current_joints: JointState,
}

impl TrajectoryController {
    fn new() -> Result<Self> {
        Ok(Self {
            joint_pub: Topic::new("joint_commands")?,
            ee_pub: Topic::new("ee_pose")?,
            waypoints: vec![
                ArmPositions::home(),
                ArmPositions::ready(),
                ArmPositions::pick(),
                ArmPositions::ready(),
                ArmPositions::place(),
                ArmPositions::ready(),
                ArmPositions::home(),
            ],
            current_wp: 0,
            progress: 0.0,
            current_joints: ArmPositions::home(),
        })
    }

    fn interpolate(a: &JointState, b: &JointState, t: f64) -> JointState {
        JointState {
            j1: a.j1 + (b.j1 - a.j1) * t,
            j2: a.j2 + (b.j2 - a.j2) * t,
            j3: a.j3 + (b.j3 - a.j3) * t,
            j4: a.j4 + (b.j4 - a.j4) * t,
            j5: a.j5 + (b.j5 - a.j5) * t,
            j6: a.j6 + (b.j6 - a.j6) * t,
        }
    }
}

impl Node for TrajectoryController {
    fn name(&self) -> &str {
        "TrajectoryController"
    }

    fn tick(&mut self) {
        if self.current_wp + 1 >= self.waypoints.len() {
            // Restart loop
            self.current_wp = 0;
            self.progress = 0.0;
        }

        let from = &self.waypoints[self.current_wp].clone();
        let to = &self.waypoints[self.current_wp + 1].clone();

        self.progress += 0.005; // ~4 seconds per segment at 50Hz
        if self.progress >= 1.0 {
            self.current_wp += 1;
            self.progress = 0.0;
        }

        let t = self.progress.min(1.0);
        self.current_joints = Self::interpolate(from, to, t);
        self.joint_pub.send(self.current_joints.clone());

        // Publish approximate end-effector pose (simplified FK)
        let ee = EndEffectorPose {
            x: 0.3 * self.current_joints.j2.sin() + 0.25 * self.current_joints.j3.sin(),
            y: 0.0,
            z: 0.1 + 0.3 * self.current_joints.j2.cos() + 0.25 * self.current_joints.j3.cos(),
            roll: self.current_joints.j4,
            pitch: self.current_joints.j5,
            yaw: self.current_joints.j1,
        };
        self.ee_pub.send(ee);
    }
}

/// Frame transform publisher — maintains the TF tree.
struct FramePublisher {
    hf: HFrame,
    joint_sub: Topic<JointState>,
}

impl FramePublisher {
    fn new() -> Result<Self> {
        let hf = HFrame::new();
        hf.add_frame("world").build()?;
        hf.add_frame("base_link").parent("world").build()?;
        hf.add_frame("shoulder").parent("base_link").build()?;
        hf.add_frame("upper_arm").parent("shoulder").build()?;
        hf.add_frame("forearm").parent("upper_arm").build()?;
        hf.add_frame("wrist_1").parent("forearm").build()?;
        hf.add_frame("wrist_2").parent("wrist_1").build()?;
        hf.add_frame("ee").parent("wrist_2").build()?;

        Ok(Self {
            hf,
            joint_sub: Topic::new("joint_commands")?,
        })
    }
}

impl Node for FramePublisher {
    fn name(&self) -> &str {
        "FramePublisher"
    }

    fn tick(&mut self) {
        if let Some(joints) = self.joint_sub.recv() {
            let ts = timestamp_now();
            // Update each joint transform
            let _ = self.hf.update_transform(
                "shoulder", &Transform::yaw(joints.j1), ts
            );
            let _ = self.hf.update_transform(
                "upper_arm",
                &Transform::xyz(0.0, 0.0, 0.20).with_yaw(joints.j2),
                ts,
            );
            let _ = self.hf.update_transform(
                "forearm",
                &Transform::xyz(0.0, 0.0, 0.30).with_yaw(joints.j3),
                ts,
            );

            // Log ee→base transform periodically
            if ts % 1_000_000_000 < 20_000_000 {
                if let Ok(tf) = self.hf.tf("ee", "base_link") {
                    hlog!(
                        "ee→base: [{:.3}, {:.3}, {:.3}]",
                        tf.translation[0],
                        tf.translation[1],
                        tf.translation[2]
                    );
                }
            }
        }
    }
}

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new().tick_hz(50.0);

    scheduler.add(TrajectoryController::new()?).order(0).rate_hz(50.0).build()?;
    scheduler.add(FramePublisher::new()?).order(1).rate_hz(50.0).build()?;

    scheduler.run_for(Duration::from_secs(60))?;
    Ok(())
}
