/// QA Launch: RT control process — publishes joint_state at 100Hz, subscribes cmd_vel.
/// Part of the multi-process launch QA test.
use horus::prelude::*;

// Use Twist from horus_types (available via prelude) instead of CmdVel
// which lives in horus-robotics (only a dev-dependency, not available for examples).
struct ControlNode {
    joint_pub: Topic<Twist>,
    cmd_sub: Topic<Twist>,
    tick_count: u64,
}

impl ControlNode {
    fn new() -> Result<Self> {
        Ok(Self {
            joint_pub: Topic::new("joint_state")?,
            cmd_sub: Topic::new("cmd_vel")?,
            tick_count: 0,
        })
    }
}

impl Node for ControlNode {
    fn name(&self) -> &str {
        "control_node"
    }
    fn tick(&mut self) {
        self.tick_count += 1;
        let _cmd = self.cmd_sub.recv();
        self.joint_pub.send(Twist::new_2d(
            (self.tick_count as f64 * 0.01).sin() * 0.5,
            0.0,
        ));
    }
}

fn main() -> Result<()> {
    let mut sched = Scheduler::new().name("qa_control").tick_rate(100_u64.hz());
    sched
        .add(ControlNode::new()?)
        .rate(100_u64.hz())
        .order(0)
        .build()?;
    sched.run()
}
