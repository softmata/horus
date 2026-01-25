# HORUS Macros

Procedural macros for the HORUS robotics framework that eliminate boilerplate and provide clean, ergonomic APIs for building robot nodes and messages.

## Overview

`horus_macros` provides two powerful macros:

1. **`node!`** - Generate complete HORUS node implementations with automatic topic registration
2. **`message!`** - Define message types with automatic zero-copy optimization and String handling

## Available Macros

### 1. `node!` - Node Generation Macro

Generates complete HORUS node implementations with automatic topic registration, lifecycle management, and Topic initialization.

#### Basic Usage

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
            self.cmd_vel.send(CmdVel::stop()).ok();
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

#### Features

- **Automatic zero-copy** - Pod-compatible types get `as_bytes()` and `from_bytes()` methods
- **Smart String handling** - `String` fields automatically become `FixedString<N>` internally
- **Clean API** - Users work with `&str`, macro handles fixed-size storage
- **Auto-implemented traits** - `Debug`, `Clone`, `Serialize`, `Deserialize`, `LogSummary`, `Pod`, `Zeroable`

#### Simple Tuple Messages

```rust
use horus_macros::message;

// Tuple-style (recommended for simple types)
message!(Position = (f32, f32));
message!(Color = (u8, u8, u8));
message!(Velocity = (f64, f64, f64));

// Usage
let pos = Position(1.5, 2.3);
let bytes = pos.as_bytes();  // Zero-copy!
let restored = Position::from_bytes(bytes).unwrap();
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

// Usage
let status = RobotStatus {
    position_x: 1.0,
    position_y: 2.0,
    battery: 85,
    is_moving: true,
};
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
let info = RobotInfo::new(
    "turtlebot_waffle",
    "Differential drive robot with LiDAR",
    42,
    85,
);

// String accessors return &str
println!("Name: {}", info.name());
println!("Description: {}", info.description());

// Setters accept any string-like type
info.set_name("turtlebot_burger");

// Still zero-copy!
let bytes = info.as_bytes();
let restored = RobotInfo::from_bytes(bytes).unwrap();
assert_eq!(restored.name(), "turtlebot_waffle");
```

#### Generated API

For `message!(Position = (f32, f32))`, the macro generates:

```rust
#[derive(Debug, Clone, Serialize, Deserialize)]
#[repr(C)]
pub struct Position(pub f32, pub f32);

impl LogSummary for Position {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}

// Zero-copy methods (auto-generated for Pod types)
unsafe impl bytemuck::Pod for Position {}
unsafe impl bytemuck::Zeroable for Position {}

impl Position {
    pub const SIZE: usize = std::mem::size_of::<Self>();

    pub fn as_bytes(&self) -> &[u8] { ... }
    pub fn from_bytes(bytes: &[u8]) -> Option<&Self> { ... }
    pub fn from_bytes_copy(bytes: &[u8]) -> Option<Self> { ... }
}
```

For struct messages with `String` fields, auto-generates:
- Constructor accepting `impl AsRef<str>` for string fields
- Getter methods returning `&str` (e.g., `name() -> &str`)
- Setter methods accepting `impl AsRef<str>` (e.g., `set_name(s)`)

## Design Principles

1. **Zero boilerplate** - One line defines complete message types
2. **Automatic optimization** - Zero-copy for Pod types, no manual work
3. **Clean abstractions** - Users work with familiar types (`String`, `&str`)
4. **Type safety** - Compile-time validation of all message definitions
5. **Performance** - Zero runtime cost, all work done at compile time

## Generated Code Characteristics

**Node Macro:**
- Complete struct definition with typed Topic fields
- `new()` constructor with proper error handling
- `Node` trait implementation with lifecycle methods
- `Default` trait implementation
- Automatic CamelCase → snake_case naming conversion

**Message Macro:**
- `Debug`, `Clone`, `Serialize`, `Deserialize` traits
- `LogSummary` for efficient logging (auto-implemented)
- `Pod`, `Zeroable` for zero-copy (when all fields are Pod-compatible)
- String field conversion to `FixedString<N>` with clean accessors
- Compile-time size calculation (`SIZE` constant)

## Testing

```bash
cargo test
```

The crate includes:
- Unit tests for macro expansion
- Compile tests for generated code
- Integration tests with actual HORUS types

## Implementation Details

**Dependencies:**
- `proc-macro2` - Token stream manipulation
- `quote` - Code generation
- `syn` - Rust syntax parsing

**Code Generation:**
- Parses input at compile time
- Validates syntax and required sections
- Generates safe, idiomatic Rust code
- No runtime overhead - pure compile-time work

## Examples

See `horus/examples/` for complete examples:
- `message_zero_copy.rs` - Zero-copy messages with String handling
- Node examples showing full node lifecycle

## Why Procedural Macros?

Rust requires procedural macros to be in a dedicated crate with `proc-macro = true`. This is a compiler requirement for build ordering and dynamic library loading during compilation.

## License

Part of the HORUS robotics framework.
