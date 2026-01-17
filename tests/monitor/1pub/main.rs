// Mobile robot controller

use horus::prelude::*;

struct Controller {
    cmd_vel: Topic<CmdVel>,
}

impl Controller {
    fn new() -> HorusResult<Self> {
        Ok(Self {
            cmd_vel: Topic::new("motors.cmd_vel")?,
        })
    }
}

impl Node for Controller {
    fn name(&self) -> &'static str {
        "controller"
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Your control logic here
        // ctx provides node state, timing info, and monitoring data
        let msg = CmdVel::new(1.0, 0.0);
        self.cmd_vel.send(msg, &mut ctx).ok();
    }
}

fn main() -> HorusResult<()> {
    let mut scheduler = Scheduler::new();

    // Add the controller node with priority 0 (highest)
    scheduler.add(
        Box::new(Controller::new()?),
        0,          // priority (0 = highest)
        Some(true), // logging config
    );
    
    // Run the scheduler
    scheduler.run()
}
