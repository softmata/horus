# HORUS - The Unified Crate

The main entry point for the HORUS robotics framework, providing a clean and unified API.

## Usage

```rust
use horus::prelude::*;
```

## What's Included

The `horus` crate re-exports everything you need:

- **Core Framework** (`horus_core`)
  - Node trait and types
  - Scheduler with presets (`deploy()`, `safety_critical()`, `deterministic()`, etc.)
  - Communication (Topic API with 10 auto-selected backends)
  - Memory management and tensor pools

- **Standard Library** (`horus_library`)
  - Message types (CmdVel, Imu, LaserScan, Pose2D, etc.)
  - Built-in hardware nodes (32 production-ready nodes)
  - HFrame coordinate transforms

- **AI** (`horus_ai`)
  - ML model registry and loader

- **Macros** (`horus_macros`)
  - `node!` macro for zero-boilerplate nodes
  - `message!` macro for zero-copy message types

- **Common Types**
  - Result and error types (Error, HorusError)
  - Duration, Instant, Arc, Mutex
  - Serde traits

## Example

```rust
use horus::prelude::*;

message!(SensorData = (f64, u32));

pub struct MyNode {
    publisher: Topic<SensorData>,
    counter: u32,
}

impl Node for MyNode {
    fn name(&self) -> &'static str { "MyNode" }

    fn tick(&mut self) {
        let data = SensorData(self.counter as f64 * 0.1, self.counter);
        self.publisher.send(data);
        self.counter += 1;
    }
}

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new()
        .with_name("my_app");

    scheduler.add(MyNode {
        publisher: Topic::new("sensor_data", None)?,
        counter: 0,
    }).order(0).done();

    scheduler.run()
}
```

## Scheduler Presets

```rust
Scheduler::new()                // Lightweight, no syscalls
Scheduler::deploy()             // Production: RT + BlackBox + profiling
Scheduler::safety_critical()    // WCET + watchdog + sequential
Scheduler::high_performance()   // Parallel + 10kHz
Scheduler::hard_realtime()      // Strict deadlines
Scheduler::deterministic()      // Reproducible execution
```

## Benefits

1. **Cleaner imports** - Single `use horus::prelude::*` statement
2. **Consistent API** - Everything through one crate
3. **Better discoverability** - All types in one place
4. **Simplified dependency** - Add only `horus` to your Cargo.toml
