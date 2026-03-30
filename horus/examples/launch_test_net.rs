/// Launch test binary with horus_net enabled.
/// Publishes CmdVel on `net_test.cmd_vel` topic with network replication.
///
/// Usage:
///   HORUS_NET_ENABLED=true HORUS_NODE_NAME=pub1 ./launch_test_net

use horus::prelude::*;

struct NetTestNode {
    pub_topic: Topic<CmdVel>,
    sub_topic: Topic<CmdVel>,
    tick_count: u64,
    role: String,
}

impl NetTestNode {
    fn new(role: &str) -> Result<Self> {
        Ok(Self {
            pub_topic: Topic::new("net_test.cmd_vel")?,
            sub_topic: Topic::new("net_test.cmd_vel")?,
            tick_count: 0,
            role: role.to_string(),
        })
    }
}

impl Node for NetTestNode {
    fn name(&self) -> &str {
        "net_test_node"
    }

    fn tick(&mut self) {
        self.tick_count += 1;
        if self.role == "pub" {
            self.pub_topic
                .send(CmdVel::new(self.tick_count as f32 * 0.01, 0.0));
        }
        if let Some(msg) = self.sub_topic.recv() {
            if self.tick_count % 50 == 0 {
                println!(
                    "[{}] tick={} recv: linear={:.3}",
                    self.role, self.tick_count, msg.linear
                );
            }
        }
    }
}

fn main() -> Result<()> {
    let params = horus_core::params::RuntimeParams::new()?;
    let role: String = params.get("role").unwrap_or_else(|| "pub".to_string());
    println!("NET_TEST: role={}", role);

    let mut sched = Scheduler::new()
        .tick_rate(50_u64.hz())
        ;
    println!("SCHEDULER: name={} net=enabled", sched.scheduler_name());

    sched
        .add(NetTestNode::new(&role)?)
        .rate(50_u64.hz())
        .order(0)
        .build()?;

    println!("READY: launch_test_net running (role={})", role);
    sched.run()
}
