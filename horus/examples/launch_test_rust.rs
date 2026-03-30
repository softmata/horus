/// Launch System + RT E2E Test Binary
///
/// Tests all launch-citizen + RT features:
/// - Reads HORUS_PARAM_* via RuntimeParams (max_speed, robot_id)
/// - Uses HORUS_NODE_NAME as Scheduler name (fallback)
/// - Uses prefer_rt() for RT degradation testing
/// - Uses .deadline_scheduler() for SCHED_DEADLINE testing
/// - Publishes Twist on `launch_test.data` topic
/// - Prints verification lines for automated checking
///
/// Usage:
///   cargo build --no-default-features -p horus --example launch_test_rust
///   HORUS_PARAM_MAX_SPEED=1.5 HORUS_PARAM_ROBOT_ID=42 HORUS_NODE_NAME=ctrl \
///     ./target/debug/examples/launch_test_rust
use horus::prelude::*;

struct ParamTestNode {
    pub_topic: Topic<Twist>,
    max_speed: f64,
    robot_id: i64,
    tick_count: u64,
}

impl ParamTestNode {
    fn new(max_speed: f64, robot_id: i64) -> Result<Self> {
        Ok(Self {
            pub_topic: Topic::new("launch_test.data")?,
            max_speed,
            robot_id,
            tick_count: 0,
        })
    }
}

impl Node for ParamTestNode {
    fn name(&self) -> &str {
        "param_test_node"
    }

    fn tick(&mut self) {
        self.tick_count += 1;
        self.pub_topic.send(Twist::new(
            [self.max_speed, 0.0, 0.0],
            [0.0, 0.0, self.robot_id as f64 * 0.01],
        ));
    }

    fn shutdown(&mut self) -> Result<()> {
        println!("SHUTDOWN: param_test_node (tick_count={})", self.tick_count);
        Ok(())
    }
}

fn main() -> Result<()> {
    // Read params from HORUS_PARAM_* env vars (set by launch system)
    let params = horus_core::params::RuntimeParams::new()?;
    let max_speed: f64 = params.get("max_speed").unwrap_or(1.0);
    let robot_id: i64 = params.get("robot_id").unwrap_or(0);
    println!("PARAMS: max_speed={} robot_id={}", max_speed, robot_id);

    // Scheduler uses HORUS_NODE_NAME as fallback name (no explicit .name())
    // prefer_rt() tests RT degradation system
    let mut sched = Scheduler::new().tick_rate(50_u64.hz()).prefer_rt();
    println!("SCHEDULER: name={}", sched.scheduler_name());
    println!(
        "RT: has_full_rt={} degradations={}",
        sched.has_full_rt(),
        sched.degradations().len()
    );
    for deg in sched.degradations() {
        println!("  RT_DEGRADED: {:?} — {}", deg.feature, deg.reason);
    }

    // Test SCHED_DEADLINE + no_alloc (both degrade gracefully)
    sched
        .add(ParamTestNode::new(max_speed, robot_id)?)
        .rate(50_u64.hz())
        .budget(16_000_u64.us())
        .deadline_scheduler()
        .order(0)
        .build()?;

    println!("READY: launch_test_rust running");
    sched.run()
}
