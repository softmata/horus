/// QA Launch: Vision process — publishes detections at 30Hz.
/// Part of the multi-process launch QA test.

use horus::prelude::*;

struct VisionNode {
    det_pub: Topic<CmdVel>,
    tick_count: u64,
}

impl VisionNode {
    fn new() -> Result<Self> {
        Ok(Self {
            det_pub: Topic::new("detections")?,
            tick_count: 0,
        })
    }
}

impl Node for VisionNode {
    fn name(&self) -> &str { "vision_node" }
    fn tick(&mut self) {
        self.tick_count += 1;
        self.det_pub.send(CmdVel::new(self.tick_count as f32, 0.0));
    }
}

fn main() -> Result<()> {
    let mut sched = Scheduler::new()
        .name("qa_vision")
        .tick_rate(30_u64.hz());
    sched.add(VisionNode::new()?).rate(30_u64.hz()).order(0).build()?;
    sched.run()
}
