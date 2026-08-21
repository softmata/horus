//! Scratch probe: a two-node graph for exercising the CLI against a live robot.
use horus::prelude::*;

struct Imu {
    out: Option<Topic<Twist>>,
    n: u64,
}

impl Node for Imu {
    fn name(&self) -> &str {
        "imu_driver"
    }
    fn init(&mut self) -> Result<()> {
        self.out = Some(Topic::new("sensors.imu")?);
        hlog!(info, "imu up");
        Ok(())
    }
    fn tick(&mut self) {
        self.n += 1;
        if let Some(ref t) = self.out {
            t.send(Twist::new_2d(self.n as f64 * 0.01, 0.0));
        }
        if self.n % 200 == 0 {
            hlog!(warn, "imu tick {}", self.n);
        }
    }
}

struct Ctl {
    inp: Option<Topic<Twist>>,
    seen: u64,
}

impl Node for Ctl {
    fn name(&self) -> &str {
        "controller"
    }
    fn init(&mut self) -> Result<()> {
        self.inp = Some(Topic::new("sensors.imu")?);
        Ok(())
    }
    fn tick(&mut self) {
        if let Some(ref t) = self.inp {
            while t.recv().is_some() {
                self.seen += 1;
            }
        }
    }
}

fn main() -> Result<()> {
    let mut s = Scheduler::new();
    s.add(Imu { out: None, n: 0 }).rate(100_u64.hz()).build()?;
    s.add(Ctl {
        inp: None,
        seen: 0,
    })
    .rate(50_u64.hz())
    .build()?;
    s.run()
}
