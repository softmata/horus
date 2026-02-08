# HORUS Core

** Internal Implementation Package - Use `horus` crate instead**

This is the internal implementation package for HORUS. Application developers should use the main `horus` crate:

```rust
//  Correct - use the main horus crate
use horus::prelude::*;

//  Wrong - don't use horus_core directly
use horus_core::prelude::*;
```

Rust-first robotics runtime: Node trait with priority scheduler, shared-memory IPC (Topic), and POSIX shared memory regions.

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Core Modules](#core-modules)
- [Quick Start](#quick-start)
- [Performance](#performance)
- [Best Practices](#best-practices)

## Overview

HORUS Core provides lightweight primitives for robotics applications:

- **Nodes**: Simple `Node` trait with `init/tick/shutdown` lifecycle
- **Scheduler**: Priority-driven executor (0 = highest) with Ctrl+C handling
- **Topic**: Unified IPC API with automatic backend selection
- **NodeInfo**: Context for logging and metrics tracking

## Architecture

```
horus_core/
── core/                  # Core framework types
   ── node.rs           # Node trait + NodeInfo context
   ── log_buffer.rs     # Global log buffer
── communication/        # IPC primitives
   ── topic.rs          # Topic unified IPC API
── memory/               # Shared memory
   ── shm_topic.rs      # Lock-free ring buffer
── scheduling/           # Task scheduling
   ── scheduler.rs      # Enhanced smart scheduler
   ── intelligence/     # Runtime profiling & classification
   ── jit/              # JIT compilation for hot paths
   ── executors/        # Async I/O and parallel execution
   ── fault_tolerance/  # Circuit breaker pattern
── params/               # Runtime parameters
    ── mod.rs            # Parameter system
```

## Core Modules

### 1. Node Trait

From `horus_core/src/core/node.rs`:

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

**Key Points:**
- `init()` returns Result<()>, use `hlog!()` macro for logging
- `tick()` takes no arguments and returns nothing
- `shutdown()` returns Result<()>, use `hlog!()` macro for logging
- All lifecycle methods use `Result<()>` for errors (via prelude alias)

### 2. hlog!() Macro

Use the `hlog!()` macro for logging from nodes. Context is automatically captured:

```rust
use horus_core::hlog;

// In your Node implementation:
hlog!(info, "Node initialized successfully");
hlog!(warn, "Low battery: {}%", percentage);
hlog!(error, "Sensor read failed: {}", err);
hlog!(debug, "Processing frame {}", frame_id);
```

**Available Metrics:**
- `total_ticks` - Total number of ticks
- `avg_tick_duration_ms` - Average tick time
- `max_tick_duration_ms` - Worst-case tick time
- `messages_sent` - Published messages
- `messages_received` - Subscribed messages
- `errors_count` - Error count
- `uptime_seconds` - Node uptime

### 3. Topic Communication

From `horus_core/src/communication/topic.rs`:

```rust
impl<T> Topic<T> {
    pub fn new(topic_name: &str) -> Result<Self>;
    pub fn producer(topic_name: &str) -> Result<Self>;  // SPSC producer
    pub fn consumer(topic_name: &str) -> Result<Self>;  // SPSC consumer
    pub fn send(&self, msg: T) -> Result<(), T>;
    pub fn recv(&self) -> Option<T>;
    pub fn name(&self) -> &str;
    pub fn metrics(&self) -> TopicMetrics;
}
```

**Topic Features:**
- Lock-free atomic operations
- Cache-line aligned (64 bytes)
- Cross-platform shared memory (Linux: `/dev/shm/horus/`, macOS: `/tmp/horus/`, Windows: `%TEMP%\horus\`)
- Default capacity: 1024 slots per topic
- Multiple backends: SPSC, MPMC, DirectChannel, IntraProcess

**Performance (on modern x86_64 systems):**
- **Topic SPSC**: 248ns median latency, 6M+ msg/s throughput
- **Topic MPMC**: 481ns median latency, flexible pub/sub
- SPSC is 48% faster than MPMC in 1P1C scenarios
- Production-validated with 6.2M+ test messages

*Performance varies by hardware. See `benchmarks/` directory for detailed results.*

### 4. Scheduler

From `horus_core/src/scheduling/scheduler.rs`:

```rust
impl Scheduler {
    pub fn new() -> Self;
    pub fn add(&mut self, node: Box<dyn Node>, priority: u32) -> &mut Self;
    pub fn run(&mut self) -> HorusResult<()>;
    pub fn run_for(&mut self, duration: Duration) -> HorusResult<()>;
    pub fn tick(&mut self, node_names: &[&str]) -> HorusResult<()>;
    pub fn tick_for(&mut self, node_names: &[&str], duration: Duration) -> HorusResult<()>;
    pub fn set_node_rate(&mut self, name: &str, rate_hz: f64) -> &mut Self;
    pub fn stop(&self);
    pub fn is_running(&self) -> bool;
}
```

**Scheduler Details:**
- Uses `tokio::runtime::Runtime` internally
- Default tick rate: ~60 FPS (16ms sleep between ticks)
- Per-node rate control: Set custom tick rates with `set_node_rate()`
- Sorts nodes by priority each tick (0 = highest)
- Built-in Ctrl+C handling
- Writes heartbeats to platform-specific path (Linux: `/dev/shm/horus/heartbeats/`, macOS: `/tmp/horus/heartbeats/`, Windows: `%TEMP%\horus\heartbeats\`)

**Usage Examples:**
```rust
// Run continuously until Ctrl+C
scheduler.run()?;

// Run for a specific duration
scheduler.run_for(Duration::from_secs(10))?;

// Run only specific nodes continuously
scheduler.tick(&["node1", "node2"])?;

// Set per-node tick rate (30 Hz for sensor_node)
scheduler.set_node_rate("sensor_node", 30.0);
```

## Quick Start

### 1. Basic Node Implementation

```rust
use horus::prelude::*;

pub struct SensorNode {
    publisher: Topic<f64>,
    counter: u32,
}

impl SensorNode {
    pub fn new() -> Result<Self> {
        Ok(Self {
            publisher: Topic::new("sensor_data")?,
            counter: 0,
        })
    }
}

impl Node for SensorNode {
    fn name(&self) -> &'static str { "SensorNode" }

    // Optional: Called once at startup
    fn init(&mut self) -> Result<()> {
        hlog!(info, "SensorNode initialized");
        Ok(())
    }

    // Required: Called repeatedly by scheduler
    fn tick(&mut self) {
        let reading = self.counter as f64 * 0.1;
        let _ = self.publisher.send(reading);
        self.counter += 1;
    }

    // Optional: Called once at shutdown
    fn shutdown(&mut self) -> Result<()> {
        hlog!(info, "SensorNode shutdown");
        Ok(())
    }
}
```

### 2. Subscriber Node

```rust
pub struct ControlNode {
    subscriber: Topic<f64>,
}

impl ControlNode {
    pub fn new() -> Result<Self> {
        Ok(Self {
            subscriber: Topic::new("sensor_data")?,
        })
    }
}

impl Node for ControlNode {
    fn name(&self) -> &'static str { "ControlNode" }

    // Optional: Called once at startup
    fn init(&mut self) -> Result<()> {
        hlog!(info, "ControlNode initialized");
        Ok(())
    }

    // Required: Called repeatedly by scheduler
    fn tick(&mut self) {
        if let Some(data) = self.subscriber.recv() {
            // Process the received data
            hlog!(info, "Received: {}", data);
        }
    }

    // Optional: Called once at shutdown
    fn shutdown(&mut self) -> Result<()> {
        hlog!(info, "ControlNode shutdown");
        Ok(())
    }
}
```

### 3. Complete Application

```rust
use horus::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    scheduler
        .add(Box::new(SensorNode::new()?), 0, Some(true))
        .add(Box::new(ControlNode::new()?), 1, Some(true));

    eprintln!("Starting scheduler...");
    scheduler.run()?;

    Ok(())
}
```

## Performance

### Enhanced Smart Scheduler

The HORUS scheduler has been enhanced with intelligent runtime optimization that automatically adapts to your workload:

**Key Features:**
- **JIT Compilation**: Hot paths compiled to native code using Cranelift (37ns tick time achieved)
- **Async I/O Tier**: Non-blocking execution for I/O-heavy operations via Tokio runtime
- **Fault Tolerance**: Circuit breaker pattern with automatic recovery
- **Smart Classification**: 5-tier automatic node categorization based on runtime profiling
- **Zero Configuration**: All optimizations happen automatically with the same simple API

**Benchmark Results:**
| Workload | Performance | Description |
|----------|-------------|-------------|
| UltraFastControl | 2.387s | JIT-optimized control loops |
| FastSensor | 2.382s | High-frequency sensor fusion |
| HeavyIO | 3.988s | Async I/O handling |
| MixedRealistic | 4.064s | Real-world mixed workload |
| 10-200 nodes | 106-120ms | Near-linear scaling |

### Communication Latency

**HORUS Topic API provides optimized IPC with automatic backend selection:**

**Topic with SPSC Backend - Cross-Core:**
- Median latency: 248ns (496 cycles @ 2GHz)
- P95 latency: 444ns, P99 latency: 578ns
- Burst throughput: 6.05 MHz (6M+ msg/s)
- Bandwidth: Up to 369 MB/s
- **Best for**: Point-to-point communication, control loops
- **Auto-selected for**: Single publisher, single subscriber

**Topic with MPMC Backend - Cross-Core:**
- Median latency: 481ns (962 cycles @ 2GHz)
- P95 latency: 653ns
- Flexible pub/sub architecture
- **Best for**: Multi-subscriber topics, sensor broadcasting
- **Auto-selected for**: Multiple publishers or subscribers

**Key Results:**
- SPSC backend is 29% faster than MPMC for 1P1C scenarios
- Sub-microsecond latency on modern x86_64 systems
- Production-validated with 6.2M+ test messages
- Zero corruptions detected

### Memory Layout

- Lock-free ring buffers
- Cache-line aligned structures (64 bytes)
- Cross-platform shared memory (auto-detected at compile time)
- Zero-copy within shared memory

### Shared Memory Configuration

**Platform-specific paths:**
| Platform | Base Path | Notes |
|----------|-----------|-------|
| Linux | `/dev/shm/horus/` | Native POSIX shm, fastest |
| macOS | `/tmp/horus/` | tmpfs-backed |
| Windows | `%TEMP%\horus\` | Temp directory |

```bash
# Linux: View shared memory regions
ls -lh /dev/shm/horus/
df -h /dev/shm

# macOS: View shared memory regions
ls -lh /tmp/horus/

# Windows (PowerShell): View shared memory regions
dir $env:TEMP\horus
```

**Custom capacity:**
```rust
let topic = Topic::<MyMessage>::new_with_capacity("large_topic", 2048)?;
```

## Message Safety

All shared memory messages must use fixed-size structures:

```rust
//  Good: Fixed-size types
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
struct SafeMessage {
    data: [f32; 64],       // Fixed-size array
    timestamp: u64,        // Primitive type
    counter: u32,          // Primitive type
}

//  Bad: Dynamic allocation
#[derive(Debug, Clone)]
struct UnsafeMessage {
    data: String,          // Heap pointer - causes segfaults!
    values: Vec<f64>,      // Heap pointer - causes segfaults!
}
```

## Best Practices

### 1. Node Design

```rust
impl Node for WellDesignedNode {
    fn tick(&mut self) {
        //  Good: Bounded execution (one message per tick)
        if let Some(data) = self.input.recv() {
            let result = process_data(data);
            let _ = self.output.send(result);
        }

        //  Bad: Blocking operations in tick()
        // std::thread::sleep(Duration::from_secs(1)); // Blocks other nodes!
    }
}
```

### 2. Priority Assignment

- **0**: Critical safety nodes (emergency stop, watchdog)
- **1-5**: Control loops (motion control, stabilization)
- **6-10**: Application logic (navigation, planning)
- **11+**: Visualization, logging, non-critical tasks

### 3. Error Handling

```rust
impl Node for RobustNode {
    fn tick(&mut self) {
        // Handle communication errors gracefully
        match self.publisher.send(data) {
            Ok(()) => { /* Success */ }
            Err(_) => {
                // Log error but don't panic - keep system running
                // Use external introspection tools to monitor errors
            }
        }
    }
}
```

### 4. Logging with hlog!()

```rust
use horus_core::hlog;

impl Node for MyNode {
    fn init(&mut self) -> Result<()> {
        // Use hlog!() macro for logging - context is automatic
        hlog!(info, "Node initialized");
        Ok(())
    }

    fn tick(&mut self) {
        // hlog!() works in tick() too - logs are captured with node context
        hlog!(debug, "Processing tick");
    }

    fn shutdown(&mut self) -> Result<()> {
        hlog!(info, "Node shutting down");
        Ok(())
    }
}
```

## Performance Tips

1. **Use appropriate priorities**: Critical control loops should have priority 0
2. **Enable logging selectively**: Only enable logging during development/debugging
3. **Use fixed-size messages**: Avoid dynamic allocation in shared memory
4. **Batch processing**: Process multiple messages per tick when possible
5. **Custom capacity**: Use `new_with_capacity()` for high-throughput topics

## Examples

### App (Multi-Node Application)

See the SnakeSim example in `horus_library/apps/snakesim/` which demonstrates:
- Multiple nodes with different priorities
- Built-in logging for debugging message flow
- Real-time game loop execution

Example structure:
```rust
let mut scheduler = Scheduler::new();
scheduler
    .add(Box::new(KeyboardInputNode::new()?), 0, Some(true))
    .add(Box::new(SnakeControlNode::new()?), 2, Some(true))
    .add(Box::new(GUINode::new()?), 3, Some(true));
scheduler.run()?;
```

## Development

### Building from Source

```bash
cd horus_core
cargo build --release
```

### Running Tests

```bash
cargo test
```

### Benchmarks

```bash
cd benchmarks
cargo bench
```

## Contributing

See the main [HORUS README](../README.md) for guidelines.

## License

Apache License 2.0 - see [LICENSE](../LICENSE) for details.
