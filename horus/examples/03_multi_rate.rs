//! # Example 3: Multi-Rate Nodes
//!
//! Different nodes running at different tick rates — a common pattern
//! in robotics where sensors, controllers, and loggers have different
//! frequency requirements.
//!
//! ```bash
//! cargo run --example 03_multi_rate
//! ```

use horus::prelude::*;

message! {
    /// Velocity command for a differential-drive robot
    CmdVelMsg {
        linear_x: f64,
        angular_z: f64,
    }
}

/// Fast sensor node — runs at 10 Hz.
struct SensorNode {
    ticks: u32,
}

impl Node for SensorNode {
    fn name(&self) -> &str {
        "SensorNode"
    }

    fn tick(&mut self) {
        self.ticks += 1;
        if self.ticks.is_multiple_of(5) {
            println!("[Sensor @10Hz] tick {} — reading LiDAR", self.ticks);
        }
    }
}

/// Control loop — runs at 5 Hz.
struct ControlNode {
    publisher: Topic<CmdVelMsg>,
    ticks: u32,
}

impl ControlNode {
    fn new() -> Result<Self> {
        Ok(Self {
            publisher: Topic::new("cmd_vel")?,
            ticks: 0,
        })
    }
}

impl Node for ControlNode {
    fn name(&self) -> &str {
        "ControlNode"
    }

    fn tick(&mut self) {
        self.ticks += 1;
        let cmd = CmdVelMsg {
            linear_x: 0.5,
            angular_z: 0.1 * (self.ticks as f64).sin(),
        };
        println!(
            "[Control @5Hz] tick {} — sending vel({:.2}, {:.2})",
            self.ticks, cmd.linear_x, cmd.angular_z
        );
        self.publisher.send(cmd);
    }
}

/// Slow logger — runs at 1 Hz.
struct LoggerNode {
    subscriber: Topic<CmdVelMsg>,
    ticks: u32,
}

impl LoggerNode {
    fn new() -> Result<Self> {
        Ok(Self {
            subscriber: Topic::new("cmd_vel")?,
            ticks: 0,
        })
    }
}

impl Node for LoggerNode {
    fn name(&self) -> &str {
        "LoggerNode"
    }

    fn tick(&mut self) {
        self.ticks += 1;
        if let Some(cmd) = self.subscriber.recv() {
            println!(
                "[Logger @1Hz]  tick {} — latest cmd_vel: ({:.2}, {:.2})",
                self.ticks, cmd.linear_x, cmd.angular_z
            );
        }
    }
}

fn main() -> Result<()> {
    println!("=== HORUS Example 3: Multi-Rate Nodes ===\n");

    let mut scheduler = Scheduler::new().tick_rate(10_u64.hz());

    // Each node gets its own rate
    scheduler
        .add(SensorNode { ticks: 0 })
        .order(0)
        .rate(10_u64.hz())
        .build()?;

    scheduler
        .add(ControlNode::new()?)
        .order(1)
        .rate(5_u64.hz())
        .build()?;

    scheduler
        .add(LoggerNode::new()?)
        .order(2)
        .rate(1_u64.hz())
        .build()?;

    scheduler.run_for(3_u64.secs())?;

    println!("\nDone!");
    Ok(())
}
