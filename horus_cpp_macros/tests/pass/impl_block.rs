use horus_cpp_macros::horus_api;

pub struct Scheduler {
    rate: f64,
}

#[horus_api]
impl Scheduler {
    pub fn new() -> Self {
        Scheduler { rate: 0.0 }
    }

    pub fn tick_rate(&mut self, hz: f64) {
        self.rate = hz;
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }
}

fn main() {
    let mut s = Scheduler::new();
    s.tick_rate(100.0);
    assert_eq!(s.rate(), 100.0);
}
