# HORUS - The Unified Crate

The main entry point for the HORUS robotics framework.

## Usage

```bash
horus new my_robot
cd my_robot
horus run
```

```rust
use horus::prelude::*;
```

## What's Included

The `horus` crate re-exports everything you need:

- **Core Framework** - Node trait, Scheduler, Topic communication
- **Standard Library** - Message types (CmdVel, Imu, LaserScan, Pose2D, Twist, etc.) and TransformFrame coordinate transforms
- **Macros** - `node!` and `message!` macros for zero-boilerplate development
- **Common Types** - Result/Error types, Duration, Arc, Mutex, Serde traits

## Example

```rust
use horus::prelude::*;

message!(SensorData = (f64, u32));

pub struct MyNode {
    publisher: Topic<SensorData>,
    counter: u32,
}

impl Node for MyNode {
    fn name(&self) -> &str { "MyNode" }

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
        publisher: Topic::new("sensor_data")?,
        counter: 0,
    }).order(0).build();

    scheduler.run()
}
```

## Scheduler Configuration

```rust
// Lightweight default
Scheduler::new()

// Composable builder methods — chain what you need
Scheduler::new()
    .monitoring(true)          // budget + deadline + watchdog + safety
    .with_blackbox(64)         // flight recorder (64MB)
    .with_profiling()          // runtime profiling
    .tick_rate(1000.hz())      // 1kHz control loop
    .deterministic(true)       // reproducible execution order
    .prefer_rt()               // try RT, warn if unavailable
    .max_deadline_misses(3)    // strict deadline enforcement
```

## License

Apache License 2.0 - see [LICENSE](../LICENSE) for details.
