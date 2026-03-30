/// QA Launch: Planner process — subscribes detections + joint_state, publishes cmd_vel at 10Hz.
/// Part of the multi-process launch QA test.

use horus::prelude::*;

struct PlannerNode {
    det_sub: Topic<CmdVel>,
    joint_sub: Topic<CmdVel>,
    cmd_pub: Topic<CmdVel>,
    tick_count: u64,
}

impl PlannerNode {
    fn new() -> Result<Self> {
        Ok(Self {
            det_sub: Topic::new("detections")?,
            joint_sub: Topic::new("joint_state")?,
            cmd_pub: Topic::new("cmd_vel")?,
            tick_count: 0,
        })
    }
}

impl Node for PlannerNode {
    fn name(&self) -> &str { "planner_node" }
    fn tick(&mut self) {
        self.tick_count += 1;
        let _det = self.det_sub.recv();
        let _joint = self.joint_sub.recv();
        self.cmd_pub.send(CmdVel::new(0.2, 0.1));
    }
}

fn main() -> Result<()> {
    let mut sched = Scheduler::new()
        .name("qa_planner")
        .tick_rate(10_u64.hz());
    sched.add(PlannerNode::new()?).rate(10_u64.hz()).order(0).build()?;
    sched.run()
}
