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
use std::time::Duration;

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
    fn name(&self) -> &'static str {
        "SensorNode"
    }

    fn tick(&mut self) {
        self.ticks += 1;
        if self.ticks % 5 == 0 {
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
    fn new() -> Self {
        Self {
            publisher: Topic::create("cmd_vel"),
            ticks: 0,
        }
    }
}

impl Node for ControlNode {
    fn name(&self) -> &'static str {
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
    fn new() -> Self {
        Self {
            subscriber: Topic::create("cmd_vel"),
            ticks: 0,
        }
    }
}

impl Node for LoggerNode {
    fn name(&self) -> &'static str {
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

    let mut scheduler = Scheduler::new().tick_hz(10.0);

    // Each node gets its own rate
    scheduler
        .add(SensorNode { ticks: 0 })
        .order(0)
        .rate_hz(10.0)
        .build();

    scheduler
        .add(ControlNode::new())
        .order(1)
        .rate_hz(5.0)
        .build();

    scheduler
        .add(LoggerNode::new())
        .order(2)
        .rate_hz(1.0)
        .build();

    scheduler.run_for(Duration::from_secs(3))?;

    println!("\nDone!");
    Ok(())
}
