# HORUS Core

**Internal implementation crate.** Use the `horus` crate instead:

```rust
use horus::prelude::*;
```

## Overview

The core runtime for HORUS, providing:

- **Node** - Simple trait with `init/tick/shutdown` lifecycle
- **Scheduler** - Runs nodes in order with configurable rates and composable builders
- **Topic** - Pub/sub communication that auto-selects the fastest transport
- **Logging** - `hlog!()` macro with context capture

## Quick Start

```rust
use horus::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    scheduler.add(SensorNode::new()?).order(0).build()?;
    scheduler.add(ControlNode::new()?).order(1).build()?;

    scheduler.run()
}
```

## Node Trait

```rust
pub trait Node: Send {
    fn name(&self) -> &str;
    fn init(&mut self) -> Result<()>;
    fn tick(&mut self);
    fn shutdown(&mut self) -> Result<()>;
}
```

## Scheduler

```rust
// Lightweight default
let mut scheduler = Scheduler::new();

// Per-node execution classes via fluent builders
scheduler.add(motor).order(0).rate(1000_u64.hz()).build()?;   // RT (auto-derived)
scheduler.add(planner).order(5).compute().build()?;
scheduler.add(telemetry).order(10).async_io().rate(1_u64.hz()).build()?;

// Production-ready with watchdog, blackbox, RT
let mut scheduler = Scheduler::new()
    .watchdog(500_u64.ms())
    .blackbox(64)
    .tick_rate(1000_u64.hz())
    .prefer_rt();

// Add nodes with execution order
scheduler.add(my_node).order(0).rate(100_u64.hz()).build()?;

// Run
scheduler.run()?;
scheduler.run_for(10_u64.secs())?;
```

## Topic Communication

```rust
let topic: Topic<f64> = Topic::new("sensor_data")?;

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
