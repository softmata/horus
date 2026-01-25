# HORUS Macros

Procedural macros for the HORUS robotics framework that eliminate boilerplate and provide a clean, section-based API for building robot nodes.

## Overview

`horus_macros` provides a single, powerful `node!` macro that generates complete HORUS node implementations with automatic topic registration, lifecycle management, and Hub initialization.

## Architecture Analysis

### Current Implementation

**Single Macro Design:**
- Only one macro: `node!` (function-like macro)
- No attribute macros (`#[node]`) - despite some test references, only the function-like macro is implemented
- Section-based syntax with clear separation of concerns

**Generated Code:**
- Complete struct definition with typed Hub fields
- `new()` constructor with error handling
- `Node` trait implementation with lifecycle methods
- `Default` trait implementation
- Automatic CamelCase  snake_case naming conversion

**Key Features:**
- **Zero unsafe code** - all generated code is safe Rust
- **Compile-time validation** - syntax errors caught at macro expansion
- **Minimal runtime overhead** - thin wrapper around manual implementation

## Usage

### Basic Node Structure

```rust
use horus_macros::node;
use horus::prelude::*;

node! {
    RobotController {
        // Publishers: data flowing OUT of this node
        pub {
            cmd_vel: CmdVel -> "robot/cmd_vel",
            status: Status -> "robot/status",
        }

        // Subscribers: data flowing INTO this node
        sub {
            sensor_data: SensorData -> "sensors/lidar",
            user_command: Command -> "user/command",
        }

        // Internal state (optional)
        data {
            last_command: Option<Command> = None,
            safety_timeout: Duration = Duration::from_secs(5),
        }

        // Main execution loop (REQUIRED)
        tick {
            // Process incoming messages
            if let Some(data) = self.sensor_data.recv() {
                let cmd = self.process_sensor_data(data);
                self.cmd_vel.send(cmd).ok();
            }
        }

        // Optional initialization
        init {
            hlog!(info, "Robot controller initialized");
            Ok(())
        }

        // Optional cleanup
        shutdown {
            self.cmd_vel.send(CmdVel::stop(), None).ok();
            Ok(())
        }

        // Additional methods
        impl {
            fn process_sensor_data(&self, data: SensorData) -> CmdVel {
                // Your logic here
                CmdVel::new(1.0, 0.0)
            }
        }
    }
}
```

### Minimal Node

```rust
node! {
    SimpleNode {
        tick {
            // Just the required tick implementation
            hlog!(info, "Node running...");
        }
    }
}
```

### Producer-Only Node

```rust
node! {
    SensorNode {
        pub {
            readings: SensorReading -> "sensors/temperature",
        }

        data {
            sensor_id: u32 = 0,
        }

        tick {
            let reading = self.read_hardware();
            self.readings.send(reading, None).ok();
        }

        impl {
            fn read_hardware(&self) -> SensorReading {
                // Hardware interface
                SensorReading::new(25.0)
            }
        }
    }
}
```

## Section Reference

| Section | Required | Syntax | Purpose |
|---------|----------|--------|---------|
| `pub {}` | No | `name: Type -> "topic"` | Publishers - data outputs |
| `sub {}` | No | `name: Type -> "topic"` | Subscribers - data inputs |
| `data {}` | No | `name: Type = default` | Internal state fields |
| `tick {}` | **YES** | `tick { ... }` | Main execution loop |
| `init {}` | No | `init { ... }` | Initialization logic |
| `shutdown {}` | No | `shutdown { ... }` | Cleanup logic |
| `impl {}` | No | `fn method(&self) { ... }` | Additional methods |

**Logging:**
- Use `hlog!()` macro for logging: `hlog!(info, "message")`, `hlog!(warn, "...")`, `hlog!(error, "...")`
- The scheduler automatically sets the node context before each lifecycle call
- Logs are published to the shared memory buffer for the HORUS monitor

## Generated API

For a node named `MyRobotNode`, the macro generates:

```rust
pub struct MyRobotNode {
    // Hub fields for each pub/sub topic
    // Data fields for internal state
}

impl MyRobotNode {
    pub fn new() -> Result<Self> {
        // Creates all Hubs with proper error handling
    }
}

impl Node for MyRobotNode {
    fn name(&self) -> &'static str {
        "my_robot_node"  // Auto-converted to snake_case
    }

    fn tick(&mut self) {
        // Your tick implementation
    }

    // Optional: init, shutdown if defined
}

impl Default for MyRobotNode {
    fn default() -> Self {
        Self::new().expect("Failed to create node")
    }
}

// Any methods from impl section
```

## Message Types

Messages are simple Rust structs - no special macros needed:

```rust
use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
struct RobotPose {
    x: f64,
    y: f64,
    theta: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct CmdVel {
    linear: f64,
    angular: f64,
}

impl CmdVel {
    fn new(linear: f64, angular: f64) -> Self {
        Self { linear, angular }
    }

    fn stop() -> Self {
        Self::new(0.0, 0.0)
    }
}
```

## Testing

The crate includes comprehensive tests:

- **Unit tests** (`node_macro_test.rs`) - Test macro expansion with mock types
- **Compile tests** (`compile_tests.rs`) - Ensure generated code compiles correctly
- **Integration tests** - Test with actual HORUS types

```bash
cargo test
```

## Design Principles

1. **Section-based clarity** - Clear visual separation of data flow
2. **Minimal boilerplate** - Generate repetitive Hub creation code
3. **Type safety** - Compile-time validation of topic types
4. **Error handling** - Proper Result types throughout
5. **Zero runtime cost** - Macro expansion only, no runtime overhead

## Implementation Details

**Dependencies:**
- `proc-macro2` - Token stream manipulation
- `quote` - Code generation
- `syn` - Rust syntax parsing (with "full" and "extra-traits" features)

**Code Generation:**
- Parses sections in any order
- Validates required sections (only `tick` is mandatory)
- Generates safe Hub initialization with error propagation
- Converts CamelCase node names to snake_case automatically

## Comparison with Manual Implementation

**Manual approach:**
```rust
pub struct MyNode {
    pub_topic: Topic<String>,
    sub_topic: Topic<i32>,
    counter: u32,
}

impl MyNode {
    pub fn new() -> Result<Self> {
        Ok(Self {
            pub_topic: Topic::new("output")?,
            sub_topic: Topic::new("input")?,
            counter: 0,
        })
    }
}

impl Node for MyNode {
    fn name(&self) -> &'static str { "my_node" }
    fn tick(&mut self) {
        // Implementation
    }
}

impl Default for MyNode {
    fn default() -> Self {
        Self::new().expect("Failed to create MyNode")
    }
}
```

**Macro approach:**
```rust
node! {
    MyNode {
        pub { pub_topic: String -> "output" }
        sub { sub_topic: i32 -> "input" }
        data { counter: u32 = 0 }
        tick {
            // Implementation
        }
    }
}
```

The macro eliminates ~20 lines of boilerplate per node while maintaining the same performance and safety characteristics.

## Limitations

**Current Implementation:**
- Function-like macro only (`node! {}`)
- No attribute macro support (`#[node]`)
- Single macro - no specialized macros for services, actions, etc.
- No compile-time topic validation beyond Rust's type system

## License

Part of the HORUS robotics framework.
