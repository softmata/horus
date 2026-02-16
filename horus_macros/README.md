# HORUS Macros

Procedural macros for the HORUS robotics framework that eliminate boilerplate and provide clean, ergonomic APIs for building robot nodes and messages.

## Overview

`horus_macros` provides two macros:

1. **`node!`** - Generate complete HORUS node implementations with automatic topic registration
2. **`message!`** - Define message types with automatic zero-copy optimization and String handling

## Available Macros

### 1. `node!` - Node Generation Macro

Generates complete HORUS node implementations with automatic topic registration, lifecycle management, and Topic initialization.

```rust
use horus::prelude::*;

node! {
    RobotController {
        pub {
            cmd_vel: CmdVel -> "robot/cmd_vel",
            status: Status -> "robot/status",
        }

        sub {
            sensor_data: SensorData -> "sensors/lidar",
        }

        data {
            safety_timeout: Duration = Duration::from_secs(5),
        }

        tick {
            if let Some(data) = self.sensor_data.recv() {
                let cmd = self.process_sensor_data(data);
                self.cmd_vel.send(cmd).ok();
            }
        }

        init {
            hlog!(info, "Robot controller initialized");
            Ok(())
        }

        shutdown {
            self.cmd_vel.send(CmdVel::stop()).ok();
            Ok(())
        }

        impl {
            fn process_sensor_data(&self, data: SensorData) -> CmdVel {
                CmdVel::new(1.0, 0.0)
            }
        }
    }
}
```

#### Section Reference

| Section | Required | Syntax | Purpose |
|---------|----------|--------|---------|
| `pub {}` | No | `name: Type -> "topic"` | Publishers - data outputs |
| `sub {}` | No | `name: Type -> "topic"` | Subscribers - data inputs |
| `data {}` | No | `name: Type = default` | Internal state fields |
| `tick {}` | **YES** | `tick { ... }` | Main execution loop |
| `init {}` | No | `init { ... }` | Initialization logic |
| `shutdown {}` | No | `shutdown { ... }` | Cleanup logic |
| `impl {}` | No | `fn method(&self) { ... }` | Additional methods |

### 2. `message!` - Message Definition Macro

Defines message types with automatic trait implementations, zero-copy optimization, and smart String handling.

#### Simple Tuple Messages

```rust
use horus::prelude::*;

message!(Position = (f32, f32));
message!(Velocity = (f64, f64, f64));

let pos = Position(1.5, 2.3);
let bytes = pos.as_bytes();  // Zero-copy!
```

#### Struct Messages

```rust
message! {
    RobotStatus {
        position_x: f32,
        position_y: f32,
        battery: u8,
        is_moving: bool,
    }
}
```

#### Messages with String Fields

```rust
message! {
    RobotInfo {
        #[max_len = 32]
        name: String,  // Becomes FixedString<32> internally

        #[max_len = 64]
        description: String,

        id: u32,
        battery: u8,
    }
}

// Clean usage - users never see FixedString!
let info = RobotInfo::new("turtlebot", "Differential drive robot", 42, 85);
println!("Name: {}", info.name());       // Returns &str
info.set_name("turtlebot_burger");        // Accepts &str
let bytes = info.as_bytes();              // Still zero-copy!
```

#### Auto-Generated Traits

For all message types:
- `Debug`, `Clone`, `Serialize`, `Deserialize`
- `LogSummary` for efficient logging
- `Pod`, `Zeroable` for zero-copy (when all fields are Pod-compatible)
- String field accessors returning `&str`

## Design Principles

1. **Zero boilerplate** - One line defines complete message types
2. **Automatic optimization** - Zero-copy for Pod types, no manual work
3. **Clean abstractions** - Users work with familiar types (`String`, `&str`)
4. **Type safety** - Compile-time validation of all definitions
5. **Performance** - Zero runtime cost, all work done at compile time

## License

Part of the HORUS robotics framework. Apache License 2.0.
