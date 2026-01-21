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

    fn tick(&mut self) {
        // Your control logic here
        // Note: ctx is available for node metadata but send() doesn't need it
        let msg = CmdVel::new(1.0, 0.0);
        self.cmd_vel.send(msg).ok();
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
