// Figure-8 Racing Controller for sim3d
//
// This controller makes the robot trace a figure-8 (infinity symbol) pattern.
// The robot continuously races around the track in smooth loops.
//
// Usage:
//   1. Start sim3d with the world file:
//      horus sim3d --world tests/sim3d/figure8_racer/world.yaml
//
//   2. In another terminal, run the controller:
//      cd tests/sim3d/figure8_racer && horus run
//
// The world.yaml defines:
//   - A differential drive robot at the center
//   - Figure-8 track markers (red=right loop, blue=left loop, yellow=center)
//   - Boundary walls
//
// The robot will trace a figure-8 pattern indefinitely until stopped with Ctrl+C.

use horus::prelude::*;

/// States for the figure-8 pattern
#[derive(Debug, Clone, Copy, PartialEq)]
enum Figure8State {
    /// First loop - turning right (clockwise)
    RightLoop,
    /// Transition from right to left loop
    CrossoverRightToLeft,
    /// Second loop - turning left (counter-clockwise)
    LeftLoop,
    /// Transition from left to right loop
    CrossoverLeftToRight,
}

impl Figure8State {
    fn next(self) -> Self {
        match self {
            Figure8State::RightLoop => Figure8State::CrossoverRightToLeft,
            Figure8State::CrossoverRightToLeft => Figure8State::LeftLoop,
            Figure8State::LeftLoop => Figure8State::CrossoverLeftToRight,
            Figure8State::CrossoverLeftToRight => Figure8State::RightLoop,
        }
    }
}

struct Figure8Racer {
    cmd_vel: Topic<CmdVel>,

    // State machine
    state: Figure8State,
    state_ticks: u64,
    total_ticks: u64,
    lap_count: u32,

    // Motion parameters (tunable)
    linear_speed: f32,      // Forward speed (m/s)
    turn_rate: f32,         // Angular velocity for loops (rad/s)
    loop_duration: u64,     // Ticks to complete one loop
    crossover_duration: u64, // Ticks for crossover straight section
}

impl Figure8Racer {
    fn new() -> HorusResult<Self> {
        println!("===========================================");
        println!("   Figure-8 Racing Controller for sim3d");
        println!("===========================================");
        println!();
        println!("Connecting to robot.cmd_vel...");

        Ok(Self {
            cmd_vel: Topic::new("robot.cmd_vel")?,

            state: Figure8State::RightLoop,
            state_ticks: 0,
            total_ticks: 0,
            lap_count: 0,

            // Motion parameters - adjust these for different track sizes
            linear_speed: 0.15,      // 15 cm/s forward (slow for tight turns)
            turn_rate: 0.8,          // 0.8 rad/s turn rate
            loop_duration: 400,      // ~4 seconds per loop at 100Hz
            crossover_duration: 100, // ~1 second crossover
        })
    }

    /// Get the current velocity command based on state
    fn get_velocity(&self) -> (f32, f32) {
        match self.state {
            Figure8State::RightLoop => {
                // Turn right (negative angular = clockwise)
                (self.linear_speed, -self.turn_rate)
            }
            Figure8State::CrossoverRightToLeft => {
                // Go straight through the crossover
                (self.linear_speed * 1.2, 0.0)
            }
            Figure8State::LeftLoop => {
                // Turn left (positive angular = counter-clockwise)
                (self.linear_speed, self.turn_rate)
            }
            Figure8State::CrossoverLeftToRight => {
                // Go straight through the crossover
                (self.linear_speed * 1.2, 0.0)
            }
        }
    }

    /// Get the duration for the current state
    fn get_state_duration(&self) -> u64 {
        match self.state {
            Figure8State::RightLoop | Figure8State::LeftLoop => self.loop_duration,
            Figure8State::CrossoverRightToLeft | Figure8State::CrossoverLeftToRight => self.crossover_duration,
        }
    }

    /// Check if we should transition to the next state
    fn should_transition(&self) -> bool {
        self.state_ticks >= self.get_state_duration()
    }

    /// Transition to the next state
    fn transition(&mut self) {
        let old_state = self.state;
        self.state = self.state.next();
        self.state_ticks = 0;

        // Count laps (one full figure-8 = going through all 4 states)
        if self.state == Figure8State::RightLoop && old_state == Figure8State::CrossoverLeftToRight {
            self.lap_count += 1;
            println!();
            println!("=== Completed Lap {} ===", self.lap_count);
            println!();
        }

        println!("[Lap {}] State: {:?}", self.lap_count + 1, self.state);
    }
}

impl Node for Figure8Racer {
    fn name(&self) -> &'static str {
        "figure8_racer"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> HorusResult<()> {
        ctx.log_info("Figure-8 Racer initialized!");
        ctx.log_info(&format!("Parameters: speed={:.2}m/s, turn_rate={:.2}rad/s",
                              self.linear_speed, self.turn_rate));
        ctx.log_info("Starting figure-8 pattern...");
        println!();
        println!("[Lap 1] State: {:?}", self.state);
        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        self.total_ticks += 1;
        self.state_ticks += 1;

        // Check for state transition
        if self.should_transition() {
            self.transition();
        }

        // Get velocity for current state
        let (linear, angular) = self.get_velocity();

        // Send velocity command
        let cmd = CmdVel::new(linear, angular);
        self.cmd_vel.send(cmd, &mut ctx).ok();

        // Print status every 100 ticks (~1 second)
        if self.total_ticks % 100 == 0 {
            println!("[tick {:>5}] Lap {} | {:?} | linear={:.2}, angular={:.2}",
                     self.total_ticks,
                     self.lap_count + 1,
                     self.state,
                     linear,
                     angular);
        }
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> HorusResult<()> {
        // Stop the robot
        let stop_cmd = CmdVel::new(0.0, 0.0);
        self.cmd_vel.send(stop_cmd, &mut Some(ctx)).ok();

        ctx.log_info(&format!("Figure-8 Racer stopped after {} laps, {} ticks",
                              self.lap_count, self.total_ticks));
        println!();
        println!("===========================================");
        println!("   Racing Complete!");
        println!("   Total Laps: {}", self.lap_count);
        println!("   Total Ticks: {}", self.total_ticks);
        println!("===========================================");
        Ok(())
    }
}

fn main() -> HorusResult<()> {
    println!();
    println!("Make sure sim3d is running: horus sim3d");
    println!("Press Ctrl+C to stop the racer.");
    println!();

    let mut scheduler = Scheduler::new();

    // Add the figure-8 racer node
    scheduler.add(
        Box::new(Figure8Racer::new()?),
        0,           // priority (0 = highest)
        Some(true),  // enable logging
    );

    // Run the scheduler (runs indefinitely until Ctrl+C)
    scheduler.run()
}
