/// QA Test Project: 3-node pub/sub pipeline (runs indefinitely)
///
/// sensor_node:     publishes CmdVel at 50Hz
/// controller_node: subscribes CmdVel, publishes CmdVel at 100Hz
/// motor_node:      subscribes CmdVel at 10Hz
///
/// Terminal 1: cargo run --no-default-features --example qa_pubsub
/// Terminal 2: horus topic list, horus node list, etc.

use horus::prelude::*;
use horus::DurationExt;

// --- Sensor Node: publishes CmdVel at 50Hz ---

struct SensorNode {
    cmd_pub: Topic<CmdVel>,
    tick_count: u64,
}

impl SensorNode {
    fn new() -> Result<Self> {
        Ok(Self {
            cmd_pub: Topic::new("sensor.cmd_vel")?,
            tick_count: 0,
        })
    }
}

impl Node for SensorNode {
    fn name(&self) -> &str {
        "sensor_node"
    }

    fn tick(&mut self) {
        self.tick_count += 1;
        let linear = 0.3 * (self.tick_count as f32 * 0.02).sin();
        let angular = 0.1 * (self.tick_count as f32 * 0.05).cos();
        self.cmd_pub.send(CmdVel::new(linear, angular));
    }
}

// --- Controller Node: subscribes + republishes at 100Hz ---

struct ControllerNode {
    input_sub: Topic<CmdVel>,
    output_pub: Topic<CmdVel>,
    tick_count: u64,
}

impl ControllerNode {
    fn new() -> Result<Self> {
        Ok(Self {
            input_sub: Topic::new("sensor.cmd_vel")?,
            output_pub: Topic::new("motor.cmd_vel")?,
            tick_count: 0,
        })
    }
}

impl Node for ControllerNode {
    fn name(&self) -> &str {
        "controller_node"
    }

    fn tick(&mut self) {
        self.tick_count += 1;
        if let Some(cmd) = self.input_sub.recv() {
            // Simple scaling
            let scaled = CmdVel::new(cmd.linear * 0.8, cmd.angular * 0.8);
            self.output_pub.send(scaled);
        }
    }
}

// --- Motor Node: subscribes at 10Hz ---

struct MotorNode {
    cmd_sub: Topic<CmdVel>,
    tick_count: u64,
}

impl MotorNode {
    fn new() -> Result<Self> {
        Ok(Self {
            cmd_sub: Topic::new("motor.cmd_vel")?,
            tick_count: 0,
        })
    }
}

impl Node for MotorNode {
    fn name(&self) -> &str {
        "motor_node"
    }

    fn tick(&mut self) {
        self.tick_count += 1;
        let _cmd = self.cmd_sub.recv();
    }
}

fn main() -> Result<()> {
    println!("=== QA Pub/Sub: 3 nodes (Ctrl+C to stop) ===");

    let mut scheduler = Scheduler::new()
        .name("qa_pubsub")
        .tick_rate(100_u64.hz())
        .prefer_rt();

    scheduler
        .add(SensorNode::new()?)
        .rate(50_u64.hz())
        .order(0)
        .build()?;

    scheduler
        .add(ControllerNode::new()?)
        .rate(100_u64.hz())
        .order(1)
        .build()?;

    scheduler
        .add(MotorNode::new()?)
        .rate(10_u64.hz())
        .order(2)
        .build()?;

    // Run forever until Ctrl+C
    scheduler.run()?;

    Ok(())
}
