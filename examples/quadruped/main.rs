/// Quadruped walking gait controller.
///
/// Implements a trot gait (diagonal legs move together) with
/// RT-priority joint control and IMU-based balance feedback.
///
/// Node pipeline:
///   GaitGenerator (200Hz, RT) → JointController (200Hz, RT) → BalanceMonitor (50Hz)

use horus::prelude::*;
use std::time::Duration;

message! {
    /// All 12 joint positions (4 legs x 3 joints)
    LegJoints {
        fl_hip: f64, fl_upper: f64, fl_lower: f64,
        fr_hip: f64, fr_upper: f64, fr_lower: f64,
        rl_hip: f64, rl_upper: f64, rl_lower: f64,
        rr_hip: f64, rr_upper: f64, rr_lower: f64,
    }

    /// Body orientation from IMU
    BodyState {
        roll: f64,
        pitch: f64,
        yaw: f64,
        height: f64,
    }
}

/// Generates trot gait joint trajectories.
struct GaitGenerator {
    joint_pub: Topic<LegJoints>,
    body_sub: Topic<BodyState>,
    phase: f64,
    speed: f64,
    standing_height: f64,
}

impl GaitGenerator {
    fn new() -> Result<Self> {
        Ok(Self {
            joint_pub: Topic::new("joint_targets")?,
            body_sub: Topic::new("body_state")?,
            phase: 0.0,
            speed: 1.0,        // gait frequency Hz
            standing_height: 0.2,
        })
    }

    /// Compute single leg joint angles for a trot gait phase.
    /// phase_offset: 0.0 for diagonal pair A, PI for pair B
    fn leg_ik(&self, phase: f64, phase_offset: f64) -> (f64, f64, f64) {
        let t = phase + phase_offset;
        let swing = t.sin().max(0.0); // positive = swing phase

        let hip = 0.0; // no lateral sway for now
        let upper = -0.4 + 0.15 * t.cos(); // forward/back swing
        let lower = -1.0 + 0.3 * swing;     // lift during swing

        (hip, upper, lower)
    }
}

impl Node for GaitGenerator {
    fn name(&self) -> &str {
        "GaitGenerator"
    }

    fn tick(&mut self) {
        self.phase += self.speed * 0.005; // dt = 1/200

        // Trot gait: FL+RR in phase, FR+RL offset by PI
        let (fl_h, fl_u, fl_l) = self.leg_ik(self.phase, 0.0);
        let (fr_h, fr_u, fr_l) = self.leg_ik(self.phase, std::f64::consts::PI);
        let (rl_h, rl_u, rl_l) = self.leg_ik(self.phase, std::f64::consts::PI);
        let (rr_h, rr_u, rr_l) = self.leg_ik(self.phase, 0.0);

        // Apply balance correction from IMU
        let (roll_comp, pitch_comp) = if let Some(body) = self.body_sub.recv() {
            (-body.roll * 0.3, -body.pitch * 0.3) // simple P-control
        } else {
            (0.0, 0.0)
        };

        let joints = LegJoints {
            fl_hip: fl_h + roll_comp, fl_upper: fl_u + pitch_comp, fl_lower: fl_l,
            fr_hip: fr_h - roll_comp, fr_upper: fr_u + pitch_comp, fr_lower: fr_l,
            rl_hip: rl_h + roll_comp, rl_upper: rl_u - pitch_comp, rl_lower: rl_l,
            rr_hip: rr_h - roll_comp, rr_upper: rr_u - pitch_comp, rr_lower: rr_l,
        };

        self.joint_pub.send(joints);
    }
}

/// RT-priority joint position controller — sends commands to sim3d.
struct JointController {
    target_sub: Topic<LegJoints>,
    cmd_pub: Topic<LegJoints>,
    current: LegJoints,
    smoothing: f64,
}

impl JointController {
    fn new() -> Result<Self> {
        let standing = LegJoints {
            fl_hip: 0.0, fl_upper: -0.4, fl_lower: -1.0,
            fr_hip: 0.0, fr_upper: -0.4, fr_lower: -1.0,
            rl_hip: 0.0, rl_upper: -0.4, rl_lower: -1.0,
            rr_hip: 0.0, rr_upper: -0.4, rr_lower: -1.0,
        };
        Ok(Self {
            target_sub: Topic::new("joint_targets")?,
            cmd_pub: Topic::new("joint_commands")?,
            current: standing,
            smoothing: 0.1,
        })
    }

    fn lerp(current: f64, target: f64, alpha: f64) -> f64 {
        current + (target - current) * alpha
    }
}

impl Node for JointController {
    fn name(&self) -> &str {
        "JointController"
    }

    fn tick(&mut self) {
        if let Some(target) = self.target_sub.recv() {
            let a = self.smoothing;
            self.current = LegJoints {
                fl_hip: Self::lerp(self.current.fl_hip, target.fl_hip, a),
                fl_upper: Self::lerp(self.current.fl_upper, target.fl_upper, a),
                fl_lower: Self::lerp(self.current.fl_lower, target.fl_lower, a),
                fr_hip: Self::lerp(self.current.fr_hip, target.fr_hip, a),
                fr_upper: Self::lerp(self.current.fr_upper, target.fr_upper, a),
                fr_lower: Self::lerp(self.current.fr_lower, target.fr_lower, a),
                rl_hip: Self::lerp(self.current.rl_hip, target.rl_hip, a),
                rl_upper: Self::lerp(self.current.rl_upper, target.rl_upper, a),
                rl_lower: Self::lerp(self.current.rl_lower, target.rl_lower, a),
                rr_hip: Self::lerp(self.current.rr_hip, target.rr_hip, a),
                rr_upper: Self::lerp(self.current.rr_upper, target.rr_upper, a),
                rr_lower: Self::lerp(self.current.rr_lower, target.rr_lower, a),
            };
        }

        self.cmd_pub.send(self.current.clone());
    }
}

/// Monitors body orientation and logs balance status.
struct BalanceMonitor {
    imu_sub: Topic<Imu>,
    state_pub: Topic<BodyState>,
    joint_sub: Topic<LegJoints>,
}

impl BalanceMonitor {
    fn new() -> Result<Self> {
        Ok(Self {
            imu_sub: Topic::new("imu/data")?,
            state_pub: Topic::new("body_state")?,
            joint_sub: Topic::new("joint_commands")?,
        })
    }
}

impl Node for BalanceMonitor {
    fn name(&self) -> &str {
        "BalanceMonitor"
    }

    fn tick(&mut self) {
        if let Some(imu) = self.imu_sub.recv() {
            let roll = imu.linear_acceleration[1].atan2(imu.linear_acceleration[2]);
            let pitch = (-imu.linear_acceleration[0])
                .atan2((imu.linear_acceleration[1].powi(2) + imu.linear_acceleration[2].powi(2)).sqrt());

            let state = BodyState {
                roll,
                pitch,
                yaw: imu.angular_velocity[2],
                height: 0.2,
            };

            // Alert on excessive tilt
            if roll.abs() > 0.3 || pitch.abs() > 0.3 {
                hlog!("BALANCE WARNING: roll={:.2} pitch={:.2}", roll, pitch);
            }

            self.state_pub.send(state);
        }

        // Log joint commands periodically
        if let Some(joints) = self.joint_sub.recv() {
            hlog_every!(100,
                "FL({:.2},{:.2},{:.2}) FR({:.2},{:.2},{:.2})",
                joints.fl_hip, joints.fl_upper, joints.fl_lower,
                joints.fr_hip, joints.fr_upper, joints.fr_lower
            );
        }
    }
}

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new().tick_hz(200.0);

    // RT nodes for gait + joint control
    scheduler.add(GaitGenerator::new()?).order(0).rate_hz(200.0).build()?;
    scheduler.add(JointController::new()?)
        .order(1)
        .rate_hz(200.0)
        .budget(100.us())          // 100μs max execution time
        .deadline(500.us())        // 500μs deadline
        .on_miss(Miss::Skip)       // Skip tick on deadline miss
        .build()?;

    // Balance monitor at lower rate
    scheduler.add(BalanceMonitor::new()?).order(5).rate_hz(50.0).build()?;

    scheduler.run_for(Duration::from_secs(30))?;
    Ok(())
}
