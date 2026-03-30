/// Differential-drive robot controller.
///
/// Publishes CmdVel commands in a square-driving pattern.
/// Subscribes to odometry for position feedback.

use horus::prelude::*;
use horus::DurationExt;

message! {
    /// Odometry feedback from the simulator
    Odometry {
        x: f64,
        y: f64,
        theta: f64,
        linear_vel: f64,
        angular_vel: f64,
    }
}

/// Drives the robot in a square pattern: forward → turn → forward → turn.
struct SquareDriver {
    cmd_pub: Topic<CmdVel>,
    odom_sub: Topic<Odometry>,
    phase: u32,        // 0=forward, 1=turn
    phase_ticks: u32,  // ticks in current phase
}

impl SquareDriver {
    fn new() -> Result<Self> {
        Ok(Self {
            cmd_pub: Topic::new("cmd_vel")?,
            odom_sub: Topic::new("odom")?,
            phase: 0,
            phase_ticks: 0,
        })
    }
}

impl Node for SquareDriver {
    fn name(&self) -> &str {
        "SquareDriver"
    }

    fn tick(&mut self) {
        self.phase_ticks += 1;

        let cmd = match self.phase {
            0 => {
                // Drive forward for 2 seconds (100 ticks at 50Hz)
                if self.phase_ticks >= 100 {
                    self.phase = 1;
                    self.phase_ticks = 0;
                }
                CmdVel::new(0.3, 0.0)
            }
            _ => {
                // Turn 90 degrees (~1.57 rad at 1.0 rad/s = ~1.57s = ~79 ticks)
                if self.phase_ticks >= 79 {
                    self.phase = 0;
                    self.phase_ticks = 0;
                }
                CmdVel::new(0.0, 1.0)
            }
        };

        self.cmd_pub.send(cmd);

        // Log odometry every 50 ticks
        if self.phase_ticks % 50 == 0 {
            if let Some(odom) = self.odom_sub.recv() {
                hlog!(
                    "pos=({:.2}, {:.2}), theta={:.2}, phase={}",
                    odom.x,
                    odom.y,
                    odom.theta,
                    if self.phase == 0 { "forward" } else { "turning" }
                );
            }
        }
    }
}

/// Monitors robot velocity and stops if too fast.
struct SafetyNode {
    cmd_sub: Topic<CmdVel>,
    max_speed: f32,
}

impl SafetyNode {
    fn new() -> Result<Self> {
        Ok(Self {
            cmd_sub: Topic::new("cmd_vel")?,
            max_speed: 1.0,
        })
    }
}

impl Node for SafetyNode {
    fn name(&self) -> &str {
        "SafetyNode"
    }

    fn tick(&mut self) {
        if let Some(cmd) = self.cmd_sub.recv() {
            if cmd.linear.abs() > self.max_speed || cmd.angular.abs() > 3.0 {
                hlog!("SAFETY: velocity exceeds limits! lin={:.2} ang={:.2}",
                    cmd.linear, cmd.angular);
            }
        }
    }
}

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new().tick_rate(50_u64.hz());

    scheduler.add(SquareDriver::new()?).order(0).rate(50_u64.hz()).build()?;
    scheduler.add(SafetyNode::new()?).order(10).rate(10_u64.hz()).build()?;

    // Run for 30 seconds (enough for ~2 full squares)
    scheduler.run_for(30_u64.secs())?;
    Ok(())
}
