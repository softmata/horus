//! # Example 1: Hello Node
//!
//! The simplest HORUS program — a single node that prints a message each tick.
//!
//! ```bash
//! cargo run --example 01_hello_node
//! ```

use horus::prelude::*;
use std::time::Duration;

/// A minimal node that prints "Hello" each tick.
struct HelloNode {
    count: u32,
}

impl Node for HelloNode {
    fn name(&self) -> &'static str {
        "HelloNode"
    }

    fn tick(&mut self) {
        self.count += 1;
        println!("[{}] Hello from HORUS! (tick {})", self.name(), self.count);
    }
}

fn main() -> Result<()> {
    println!("=== HORUS Example 1: Hello Node ===\n");

    // Create a scheduler running at 2 Hz (slow enough to read output)
    let mut scheduler = Scheduler::new().tick_hz(2.0);

    // Add the node with execution order 0 (first to run)
    scheduler.add(HelloNode { count: 0 }).order(0).build();

    // Run for 3 seconds
    scheduler.run_for(Duration::from_secs(3))?;

    println!("\nDone!");
    Ok(())
}
