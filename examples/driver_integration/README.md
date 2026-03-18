# Driver Integration — Hardware Abstraction & Terra HAL

Demonstrates how to integrate real hardware into horus using the `[drivers]` configuration in `horus.toml` and the Terra Hardware Abstraction Layer.

## Architecture

```
horus.toml [drivers]
    ├── motor = { terra = "dynamixel" }    → SimMotorDriver (200Hz)
    ├── imu = { terra = "bno055" }         → SimImuDriver (200Hz)
    ├── lidar = { terra = "rplidar" }      → (not used in this example)
    └── gripper = { node = "GripperDriver" } → SimGripperDriver (50Hz)
                                                      │
                                              ArmController (50Hz)
                                                uses motor + gripper topics
```

## What This Demonstrates

- **`[drivers]` in horus.toml**: Declarative hardware configuration
- **Terra drivers**: `dynamixel`, `bno055`, `rplidar` — standard robotics peripherals
- **Custom drivers**: `{ node = "GripperDriver" }` for non-standard hardware
- **Driver lifecycle**: `init()` connects hardware, `tick()` reads/writes, `shutdown()` disables
- **Simulated drivers**: Same API as real hardware, runs without physical devices

## horus.toml Driver Config

```toml
[drivers]
motor = { terra = "dynamixel", port = "/dev/ttyUSB0", baudrate = "1000000" }
imu = { terra = "bno055", bus = "/dev/i2c-1", address = "0x28" }
lidar = { terra = "rplidar", port = "/dev/ttyUSB1", baudrate = "256000" }
gripper = { node = "GripperDriver" }
```

## ROS2 Equivalent

| ROS2 | Horus |
|------|-------|
| `ros2_control` hardware_interface | `[drivers]` in horus.toml + Terra HAL |
| `controller_manager` | `Scheduler` with driver nodes at high priority |
| `micro-ros-agent` | Terra serial/I2C/CAN drivers |
| Custom `LifecycleNode` hardware driver | Node with `init()`/`tick()`/`shutdown()` |
| `robot_state_publisher` | Driver node publishes to topic directly |

## Key Patterns

### Loading Drivers from Config (Real Hardware)
```rust
let hw = horus::drivers::load()?;
let motor = hw.dynamixel("motor")?;    // From [drivers.motor]
let imu = hw.bno055("imu")?;          // From [drivers.imu]
```

### Driver Node Lifecycle
```rust
impl Node for MotorDriver {
    fn init(&mut self) -> Result<()> {
        self.hw.connect()?;           // Open serial port
        self.hw.set_torque(true)?;    // Enable motors
        Ok(())
    }

    fn tick(&mut self) {
        let cmd = self.cmd_sub.recv();   // Get command
        self.hw.write_position(cmd)?;    // Send to hardware
        let fb = self.hw.read_state()?;  // Read feedback
        self.feedback_pub.send(fb);      // Publish to topic
    }

    fn shutdown(&mut self) -> Result<()> {
        self.hw.set_torque(false)?;   // Disable motors
        self.hw.disconnect()?;        // Close port
        Ok(())
    }
}
```

## Run

```bash
horus run main.rs           # Runs with simulated drivers
horus doctor                # Check if real hardware is connected
horus topic echo motor.feedback  # Watch motor state
```
