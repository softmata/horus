# HORUS Macros

Procedural macros for the HORUS robotics framework that eliminate boilerplate.

## Macros

### `node!` - Node Generation

Generates a complete HORUS node with automatic topic setup and lifecycle management.

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

#### Sections

| Section | Required | Syntax | Purpose |
|---------|----------|--------|---------|
| `pub {}` | No | `name: Type -> "topic"` | Publishers |
| `sub {}` | No | `name: Type -> "topic"` | Subscribers |
| `data {}` | No | `name: Type = default` | Internal state |
| `tick {}` | **Yes** | `tick { ... }` | Main loop |
| `init {}` | No | `init { ... }` | Setup logic |
| `shutdown {}` | No | `shutdown { ... }` | Cleanup logic |
| `impl {}` | No | `fn method(&self) { ... }` | Helper methods |

### `message!` - Message Types

Define message types in one line:

```rust
use horus::prelude::*;

// Tuple messages
message!(Position = (f32, f32));
message!(Velocity = (f64, f64, f64));

// Struct messages
message! {
    RobotStatus {
        position_x: f32,
        position_y: f32,
        battery: u8,
        is_moving: bool,
    }
}

// Messages with strings
message! {
    RobotInfo {
        #[max_len = 32]
        name: String,

        #[max_len = 64]
        description: String,

        id: u32,
        battery: u8,
    }
}

let info = RobotInfo::new("turtlebot", "Differential drive robot", 42, 85);
println!("Name: {}", info.name());       // Returns &str
info.set_name("turtlebot_burger");        // Accepts &str
```

All message types automatically get `Debug`, `Clone`, `Serialize`, `Deserialize`, and logging support.

## License

Part of the HORUS robotics framework. Apache License 2.0.
