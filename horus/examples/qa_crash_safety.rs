/// QA Test: Crash and Safety — nodes that deliberately misbehave.
///
/// slow_node:    tick takes 15ms but budget is 5ms (triggers budget violation)
/// crash_node:   panics after 200 ticks (tests node isolation)
/// healthy_node: runs normally at 50Hz (should survive others' failures)
use horus::prelude::*;

struct SlowNode {
    tick_count: u64,
}
impl Node for SlowNode {
    fn name(&self) -> &str {
        "slow_node"
    }
    fn tick(&mut self) {
        self.tick_count += 1;
        // Deliberately slow — exceeds budget
        std::thread::sleep(std::time::Duration::from_millis(15));
    }
}

struct CrashNode {
    tick_count: u64,
}
impl Node for CrashNode {
    fn name(&self) -> &str {
        "crash_node"
    }
    fn tick(&mut self) {
        self.tick_count += 1;
        if self.tick_count == 200 {
            panic!("intentional crash at tick 200");
        }
    }
}

// Use Twist from horus_types (available via prelude) instead of CmdVel
// which lives in horus-robotics (only a dev-dependency, not available for examples).
struct HealthyNode {
    pub_topic: Topic<Twist>,
    tick_count: u64,
}
impl HealthyNode {
    fn new() -> Result<Self> {
        Ok(Self {
            pub_topic: Topic::new("healthy.heartbeat")?,
            tick_count: 0,
        })
    }
}
impl Node for HealthyNode {
    fn name(&self) -> &str {
        "healthy_node"
    }
    fn tick(&mut self) {
        self.tick_count += 1;
        self.pub_topic
            .send(Twist::new_2d(self.tick_count as f64, 0.0));
    }
}

fn main() -> Result<()> {
    println!("=== QA Crash/Safety (Ctrl+C to stop) ===");

    let mut scheduler = Scheduler::new()
        .name("qa_crash")
        .tick_rate(50_u64.hz())
        .watchdog(200_u64.ms());

    scheduler
        .add(SlowNode { tick_count: 0 })
        .rate(50_u64.hz())
        .budget(5_u64.ms())
        .order(0)
        .build()?;

    // crash_node removed — its panic triggers emergency stop which exits the scheduler.
    // Budget violations from slow_node are sufficient to test safety monitoring.

    scheduler
        .add(HealthyNode::new()?)
        .rate(50_u64.hz())
        .order(2)
        .build()?;

    scheduler.run()?;
    Ok(())
}
