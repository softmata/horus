# HORUS Core

**Internal implementation crate.** Use the `horus` crate instead:

```rust
use horus::prelude::*;
```

## Overview

The core runtime for HORUS, providing:

- **Node** - Simple trait with `init/tick/shutdown` lifecycle
- **Scheduler** - Runs nodes in order with configurable rates and presets
- **Topic** - Pub/sub communication that auto-selects the fastest transport
- **Logging** - `hlog!()` macro with context capture

## Quick Start

```rust
use horus::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    scheduler.add(SensorNode::new()?).order(0).done();
    scheduler.add(ControlNode::new()?).order(1).done();

    scheduler.run()
}
```

## Node Trait

```rust
pub trait Node: Send {
    fn name(&self) -> &'static str;
    fn init(&mut self) -> Result<()>;
    fn tick(&mut self);
    fn shutdown(&mut self) -> Result<()>;
}
```

## Scheduler

```rust
// Lightweight default
let mut scheduler = Scheduler::new();

// Presets for common configurations
let mut scheduler = Scheduler::deploy();          // Production
let mut scheduler = Scheduler::safety_critical(); // Safety systems
let mut scheduler = Scheduler::high_performance();// Parallel + 10kHz
let mut scheduler = Scheduler::deterministic();   // Reproducible

// Builder for custom configuration
let mut scheduler = Scheduler::new()
    .realtime()
    .tick_hz(1000.0)
    .with_blackbox(16)
    .with_name("my_robot");

// Add nodes with execution order
scheduler.add(my_node).order(0).rate_hz(100.0).done();

// Run
scheduler.run()?;
scheduler.run_for(Duration::from_secs(10))?;
```

## Topic Communication

```rust
let topic: Topic<f64> = Topic::new("sensor_data", None)?;

topic.send(42.0);
if let Some(value) = topic.recv() {
    println!("Got: {}", value);
}
```

The transport backend is auto-selected based on topology:

| Scenario | Latency |
|----------|---------|
| Same thread | ~3 ns |
| Same process | 18-36 ns |
| Cross process | 50-167 ns |

## Logging

```rust
use horus_core::hlog;

hlog!(info, "Node initialized");
hlog!(warn, "Low battery: {}%", percentage);
hlog!(error, "Sensor read failed: {}", err);
```

## Order Guidelines

- **0-9**: Critical (motor control, safety)
- **10-49**: High priority (sensors, fast loops)
- **50-99**: Normal (processing, planning)
- **100-199**: Low (logging, diagnostics)
- **200+**: Background (telemetry)

## License

Apache License 2.0 - see [LICENSE](../LICENSE) for details.
