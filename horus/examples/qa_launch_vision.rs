/// QA Launch: Vision process — publishes detections at 30Hz.
/// Part of the multi-process launch QA test.
use horus::prelude::*;

// Use Twist from horus_types (available via prelude) instead of CmdVel
// which lives in horus-robotics (only a dev-dependency, not available for examples).
struct VisionNode {
    det_pub: Topic<Twist>,
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
    fn name(&self) -> &str {
        "vision_node"
    }
    fn tick(&mut self) {
        self.tick_count += 1;
        self.det_pub
            .send(Twist::new_2d(self.tick_count as f64, 0.0));
    }
}

fn main() -> Result<()> {
    let mut sched = Scheduler::new().name("qa_vision").tick_rate(30_u64.hz());
    sched
        .add(VisionNode::new()?)
        .rate(30_u64.hz())
        .order(0)
        .build()?;
    sched.run()
}
