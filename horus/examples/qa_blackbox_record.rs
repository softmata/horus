/// QA Test: Blackbox + Recording — enables both for CLI testing.
///
/// - Blackbox enabled (flight recorder)
/// - Recording enabled (session capture)
/// - slow_node exceeds budget (generates blackbox events)
/// - healthy_node ticks normally
///
/// Terminal 1: cargo run --no-default-features --example qa_blackbox_record
/// Terminal 2: horus blackbox --tail, horus log --follow, horus record list

use horus::prelude::*;

struct SlowNode { tick_count: u64 }
impl Node for SlowNode {
    fn name(&self) -> &str { "slow_node" }
    fn tick(&mut self) {
        self.tick_count += 1;
        // Every 10th tick, sleep 8ms to exceed 5ms budget
        if self.tick_count % 10 == 0 {
            std::thread::sleep(std::time::Duration::from_millis(8));
        }
    }
}

struct HealthyNode {
    pub_topic: Topic<CmdVel>,
    tick_count: u64,
}
impl HealthyNode {
    fn new() -> Result<Self> {
        Ok(Self {
            pub_topic: Topic::new("healthy.data")?,
            tick_count: 0,
        })
    }
}
impl Node for HealthyNode {
    fn name(&self) -> &str { "healthy_node" }
    fn tick(&mut self) {
        self.tick_count += 1;
        self.pub_topic.send(CmdVel::new(self.tick_count as f32, 0.0));
        if self.tick_count % 500 == 0 {
            println!("[healthy] tick {}", self.tick_count);
        }
    }
}

fn main() -> Result<()> {
    println!("=== QA Blackbox + Record (Ctrl+C to stop) ===");

    let mut scheduler = Scheduler::new()
        .name("qa_blackbox")
        .tick_rate(50_u64.hz())
        .blackbox(1);         // 1 MB flight recorder

    scheduler.add(SlowNode { tick_count: 0 })
        .rate(50_u64.hz())
        .budget(5_u64.ms())
        .order(0)
        .build()?;

    scheduler.add(HealthyNode::new()?)
        .rate(50_u64.hz())
        .order(1)
        .build()?;

    scheduler.run()?;
    Ok(())
}
