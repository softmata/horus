# HORUS Core

**Internal implementation package - Use `horus` crate instead**

```rust
// Correct - use the main horus crate
use horus::prelude::*;

// Don't use horus_core directly
// use horus_core::prelude::*;
```

Rust-first robotics runtime: Node trait with priority scheduler, shared-memory IPC (Topic), and POSIX shared memory regions.

## Overview

HORUS Core provides lightweight primitives for robotics applications:

- **Nodes**: Simple `Node` trait with `init/tick/shutdown` lifecycle
- **Scheduler**: Priority-driven executor with opt-in RT features and presets
- **Topic**: Unified IPC API with 10 auto-selected backends (3ns - 167ns)
- **NodeInfo**: Context for logging and metrics tracking

## Architecture

```
horus_core/
-- core/                  # Core framework types
   -- node.rs           # Node trait + NodeInfo context
   -- log_buffer.rs     # Global log buffer
-- communication/        # IPC primitives
   -- topic/            # Topic unified IPC API (10 backends)
-- memory/               # Shared memory
   -- shm_topic.rs      # Lock-free ring buffer
-- scheduling/           # Task scheduling
   -- scheduler.rs      # Scheduler with presets and builder API
   -- intelligence/     # Runtime profiling & classification
   -- executors/        # Async I/O, parallel, and background execution
   -- fault_tolerance/  # Circuit breaker pattern
   -- record_replay/    # Deterministic record/replay
-- params/               # Runtime parameters
    -- mod.rs            # Parameter system
```

## Core Modules

### 1. Node Trait

```rust
pub trait Node: Send {
    fn name(&self) -> &'static str;
    fn init(&mut self) -> Result<()>;
    fn tick(&mut self);
    fn shutdown(&mut self) -> Result<()>;
    fn publishers(&self) -> Vec<TopicMetadata> { Vec::new() }
    fn subscribers(&self) -> Vec<TopicMetadata> { Vec::new() }
}
```

### 2. Scheduler

```rust
impl Scheduler {
    // Construction — lightweight, no syscalls
    pub fn new() -> Self;

    // Presets — bundle common configurations
    pub fn deploy() -> Self;            // RT + BlackBox + profiling
    pub fn safety_critical() -> Self;   // WCET + watchdog + sequential
    pub fn high_performance() -> Self;  // Parallel + 10kHz
    pub fn hard_realtime() -> Self;     // Strict deadlines
    pub fn deterministic() -> Self;     // Reproducible execution

    // Builder — opt-in to features
    pub fn realtime(self) -> Self;          // RT priority + memory lock + CPU pin
    pub fn with_blackbox(self, mb: usize) -> Self;  // Flight recorder
    pub fn tick_hz(self, hz: f64) -> Self;  // Global tick rate
    pub fn with_name(self, name: &str) -> Self;

    // Node management — fluent API
    pub fn add(&mut self, node: impl Node) -> NodeBuilder;
    pub fn add_dyn(&mut self, node: Box<dyn Node>, order: u32) -> &mut Self;

    // Execution
    pub fn run(&mut self) -> HorusResult<()>;
    pub fn run_for(&mut self, duration: Duration) -> HorusResult<()>;
    pub fn tick(&mut self, node_names: &[&str]) -> HorusResult<()>;
    pub fn stop(&self);
    pub fn is_running(&self) -> bool;
}
```

### 3. Topic Communication

```rust
impl<T> Topic<T> {
    pub fn new(topic_name: &str, config: Option<TopicConfig>) -> Result<Self>;
    pub fn with_capacity(name: &str, capacity: u32, config: Option<TopicConfig>) -> Result<Self>;

    pub fn send(&self, msg: T);             // Fire-and-forget
    pub fn try_send(&self, msg: T) -> Result<(), T>;  // Returns msg on failure
    pub fn recv(&self) -> Option<T>;
    pub fn try_recv(&self) -> Option<T>;

    pub fn name(&self) -> &str;
    pub fn mode(&self) -> BackendMode;
    pub fn metrics(&self) -> TopicMetrics;
}
```

**10 auto-selected backends:**

| Backend | Topology | Latency |
|---------|----------|---------|
| DirectChannel | Same thread, 1P-1C, POD | ~3ns |
| SpscIntra | Same process, 1P-1C | ~18ns |
| SpmcIntra | Same process, 1P-MC | ~24ns |
| MpscIntra | Same process, MP-1C | ~26ns |
| MpmcIntra | Same process, MPMC | ~36ns |
| PodShm | Cross process, POD broadcast | ~50ns |
| MpscShm | Cross process, MP-1C | ~65ns |
| SpmcShm | Cross process, 1P-MC | ~70ns |
| SpscShm | Cross process, 1P-1C | ~85ns |
| MpmcShm | Cross process, MPMC | ~167ns |

### 4. hlog!() Macro

```rust
use horus_core::hlog;

hlog!(info, "Node initialized successfully");
hlog!(warn, "Low battery: {}%", percentage);
hlog!(error, "Sensor read failed: {}", err);
```

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

## Best Practices

### Priority Assignment (Order)

- **0-9**: Critical real-time (motor control, safety)
- **10-49**: High priority (sensors, fast control loops)
- **50-99**: Normal priority (processing, planning)
- **100-199**: Low priority (logging, diagnostics)
- **200+**: Background (telemetry, non-essential)

### Error Handling

```rust
impl Node for RobustNode {
    fn tick(&mut self) {
        match self.publisher.try_send(data) {
            Ok(()) => { /* Success */ }
            Err(_msg) => {
                // Log error but don't panic - keep system running
            }
        }
    }
}
```

## License

Apache License 2.0 - see [LICENSE](../LICENSE) for details.
