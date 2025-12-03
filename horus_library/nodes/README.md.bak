# HORUS Built-in Nodes - Production-Ready Hardware Drivers

This directory contains **32 production-ready nodes** with real hardware integration. These are NOT prototypes - they're production-grade drivers ready for deployment in real robots.

## Hardware-Only Design Philosophy

**Important**: Hardware nodes in HORUS are **real drivers only** - no simulation fallback.

- **Clear intent**: Using `CameraNode` means you're controlling actual hardware
- **Compile-time safety**: Missing hardware features = compile error, not runtime surprise
- **No silent failures**: Can't accidentally test with mock data and deploy to production
- **No simulation mode**: For testing without hardware, use `sim2d` or `sim3d` tools

### Feature-Gated Compilation

Hardware nodes only compile when their required Cargo feature is enabled:

```yaml
# horus.yaml
dependencies:
  - horus_library@0.1.5:features=opencv-backend,serial-hardware
```

Without the feature? **Compile error** - not a runtime failure. This is intentional.

**Use built-in nodes as-is for 90% of robotics applications. Wrap them only when adding custom algorithms on top.**

---

## Node Feature Requirements

| Node | Required Feature | Always Available |
|------|-----------------|------------------|
| **Safety & Monitoring** |||
| EmergencyStopNode | - | Yes |
| SafetyMonitorNode | - | Yes |
| **Vision** |||
| CameraNode | `opencv-backend` or `v4l2-backend` or `realsense` or `zed` | No |
| DepthCameraNode | `realsense` | No |
| ImageProcessorNode | `opencv-backend` | No |
| **Input Devices** |||
| JoystickNode | `gilrs` | No |
| KeyboardInputNode | `crossterm` | No |
| **Sensors** |||
| BatteryMonitorNode | `i2c-hardware` | No |
| ImuNode | `bno055-imu` or `mpu6050-imu` | No |
| GpsNode | `nmea-gps` | No |
| LidarNode | `rplidar` | No |
| ForceTorqueSensorNode | `netft` | No |
| EncoderNode | `gpio-hardware` | No |
| UltrasonicNode | `gpio-hardware` | No |
| **Motors/Actuators** |||
| BldcMotorNode | `gpio-hardware` | No |
| DcMotorNode | `gpio-hardware` | No |
| StepperMotorNode | `gpio-hardware` | No |
| ServoControllerNode | `gpio-hardware` | No |
| DynamixelNode | `serial-hardware` | No |
| RoboclawMotorNode | `serial-hardware` | No |
| **Industrial** |||
| CanBusNode | `can-hardware` | No |
| DigitalIONode | `gpio-hardware` | No |
| I2cBusNode | `i2c-hardware` | No |
| ModbusNode | `modbus-hardware` | No |
| SerialNode | `serial-hardware` | No |
| SpiBusNode | `spi-hardware` | No |
| **Navigation** |||
| PathPlannerNode | - | Yes |
| LocalizationNode | - | Yes |
| OdometryNode | - | Yes |
| CollisionDetectorNode | - | Yes |
| DifferentialDriveNode | - | Yes |
| PidControllerNode | - | Yes |

## Complete Node Catalog (32 Nodes)

### Safety & Monitoring (2 nodes)
- **EmergencyStopNode** - Hardware e-stop with watchdog timeout
- **SafetyMonitorNode** - Multi-layered safety (battery, CPU, memory, temperature)

### Sensor Interface (9 nodes)
- **CameraNode** - OpenCV, V4L2 backends
- **DepthCameraNode** - Intel RealSense (D415, D435, D455, L515), ZED, Kinect (Full RealSense support)
- **LidarNode** - Laser range finder (2D/3D)
- **IMUNode** - Accelerometer, gyroscope, magnetometer with calibration
- **EncoderNode** - Quadrature decoder for odometry
- **GPSNode** - NMEA serial (u-blox, MTK, etc.) (Full support)
- **UltrasonicNode** - HC-SR04, US-100, Maxbotix (up to 16 sensors)
- **BatteryMonitorNode** - I2C fuel gauges (INA219, INA226, BQ27441) (Full I2C support)
- **ForceTorqueSensorNode** - 6-axis F/T sensors (ATI, Robotiq)

### Control & Actuation (8 nodes)
- **DcMotorNode** - L298N, TB6612 motor controllers
- **BldcMotorNode** - BLDC ESCs (PWM, DShot, VESC, CAN) (Full GPIO PWM support)
- **StepperMotorNode** - A4988, DRV8825, TMC2208 drivers
- **ServoControllerNode** - Multi-servo control with limits
- **DynamixelNode** - Dynamixel smart servos (Protocol 1.0/2.0)
- **RoboclawNode** - BasicMicro Roboclaw dual-channel controllers (Full serial protocol support)
- **PidControllerNode** - Generic PID with anti-windup
- **DifferentialDriveNode** - Mobile robot base control

### Navigation (4 nodes)
- **PathPlannerNode** - A*, RRT, Dijkstra algorithms
- **LocalizationNode** - Robot position estimation
- **OdometryNode** - Dead reckoning (differential, mecanum, ackermann)
- **CollisionDetectorNode** - Real-time collision detection

### Industrial Integration (6 nodes)
- **CANBusNode** - Linux SocketCAN (CAN 2.0A/B, CAN-FD) (Full SocketCAN support)
- **ModbusNode** - Modbus TCP/RTU for industrial PLCs
- **SerialNode** - UART/Serial communication
- **I2CBusNode** - I2C bus communication
- **SPIBusNode** - SPI communication
- **DigitalIONode** - GPIO control with debounce

### Vision & Image Processing (1 node)
- **ImageProcessorNode** - Filtering, color conversion

### Input Devices (2 nodes)
- **KeyboardInputNode** - Keyboard capture for teleoperation
- **JoystickInputNode** - Gamepad/joystick input

---

## Hardware Integration Status

| Node | Hardware Status | Tested On |
|------|----------------|-----------|
| DepthCameraNode | Full | Intel RealSense D435, D455, L515 |
| BatteryMonitorNode | Full | I2C INA219, INA226 on Raspberry Pi |
| BldcMotorNode | Full | Raspberry Pi GPIO PWM, ESC protocols |
| CANBusNode | Full | Linux SocketCAN (virtual and real) |
| RoboclawNode | Full | Roboclaw 2x7A through 2x160A models |
| GPSNode | Full | u-blox NEO/ZED, MTK3339 via NMEA |
| SafetyMonitorNode | Full | System monitoring on Linux/Pi |

All other nodes: Complete implementation with simulation fallback

---

## Quick Links

Each node has its own folder with code (`mod.rs`) and documentation (`README.md`):

### Control Nodes
- [pid_controller/](./pid_controller/) - Generic PID controller
- [differential_drive/](./differential_drive/) - Mobile robot base controller
- [servo_controller/](./servo_controller/) - Multi-servo controller

### Sensor Nodes
- [camera/](./camera/) - Camera image capture
- [imu/](./imu/) - Inertial measurement unit
- [lidar/](./lidar/) - Laser range finder
- [encoder/](./encoder/) - Rotary encoder input
- [joystick/](./joystick/) - Joystick/gamepad input

### Safety and Monitoring
- [emergency_stop/](./emergency_stop/) - Emergency stop system
- [safety_monitor/](./safety_monitor/) - Safety condition monitoring
- [collision_detector/](./collision_detector/) - Collision detection

### I/O and Communication
- [digital_io/](./digital_io/) - Digital GPIO control
- [modbus/](./modbus/) - Modbus RTU/TCP communication
- [keyboard_input/](./keyboard_input/) - Keyboard teleoperation

### Navigation
- [path_planner/](./path_planner/) - Path planning algorithms
- [localization/](./localization/) - Robot localization

## Available Nodes

### Control Nodes

- [PID Controller](#pid-controller-node)
- [Differential Drive](#differential-drive-node)
- [Servo Controller](#servo-controller-node)

### Sensor Nodes

- [Camera](#camera-node)
- [IMU](#imu-node)
- [Lidar](#lidar-node)
- [Encoder](#encoder-node)
- [Joystick](#joystick-node)

### Safety and Monitoring

- [Emergency Stop](#emergency-stop-node)
- [Safety Monitor](#safety-monitor-node)
- [Collision Detector](#collision-detector-node)

### I/O and Communication

- [Digital I/O](#digital-io-node)
- [Modbus](#modbus-node)
- [Keyboard Input](#keyboard-input-node)

### Navigation

- [Path Planner](#path-planner-node)
- [Localization](#localization-node)

---

## PID Controller Node

**Purpose**: Generic PID controller for position/velocity control applications.

### Topics

**Subscribers**:
- `setpoint` (f32) - Target value for the controller
- `feedback` (f32) - Current measurement from sensor
- `pid_config` (PidConfig) - Runtime PID parameter updates

**Publishers**:
- `pid_output` (MotorCommand) - Control output command

### Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| kp | f32 | 1.0 | Proportional gain |
| ki | f32 | 0.1 | Integral gain |
| kd | f32 | 0.05 | Derivative gain |
| output_min | f32 | -100.0 | Minimum output value |
| output_max | f32 | 100.0 | Maximum output value |
| integral_min | f32 | -50.0 | Anti-windup minimum |
| integral_max | f32 | 50.0 | Anti-windup maximum |
| deadband | f32 | 0.01 | Error threshold |
| motor_id | u8 | 0 | Motor identifier |

### Public API

```rust
// Create with default topics
let mut pid = PidControllerNode::new()?;

// Create with custom topics
let mut pid = PidControllerNode::new_with_topics(
    "setpoint",
    "feedback",
    "output",
    "config"
)?;

// Configure PID gains
pid.set_gains(1.5, 0.2, 0.1);

// Set output limits
pid.set_output_limits(-255.0, 255.0);

// Set integral limits (anti-windup)
pid.set_integral_limits(-100.0, 100.0);

// Set error deadband
pid.set_deadband(0.05);

// Set motor ID
pid.set_motor_id(1);

// Reset PID state
pid.reset();

// Get current state (setpoint, feedback, error, integral)
let (sp, fb, err, int) = pid.get_state();
```

### Usage Example

```rust
use horus_library::nodes::PidControllerNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create PID controller for motor position control
    let mut pid = PidControllerNode::new_with_topics(
        "target_position",
        "encoder_position",
        "motor_command",
        "pid_config"
    )?;

    // Configure for position control
    pid.set_gains(2.0, 0.5, 0.1);
    pid.set_output_limits(-100.0, 100.0);
    pid.set_motor_id(0);

    runtime.add_node(pid);
    runtime.run()?;

    Ok(())
}
```

---

## Differential Drive Node

**Purpose**: Converts Twist velocity commands to differential drive motor commands and publishes odometry.

### Topics

**Subscribers**:
- `cmd_vel` (Twist) - Velocity commands (linear and angular)

**Publishers**:
- `drive_command` (DifferentialDriveCommand) - Left/right wheel speeds
- `odom` (Odometry) - Robot odometry (position, orientation, velocities)

### Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| wheel_base | f32 | 0.5 | Distance between wheels (m) |
| wheel_radius | f32 | 0.1 | Wheel radius (m) |
| max_linear_vel | f32 | 2.0 | Maximum linear velocity (m/s) |
| max_angular_vel | f32 | 3.14 | Maximum angular velocity (rad/s) |

### Public API

```rust
// Create with default topics
let mut drive = DifferentialDriveNode::new()?;

// Create with custom topics
let mut drive = DifferentialDriveNode::new_with_topics(
    "cmd_vel",
    "drive_command",
    "odom"
)?;

// Set robot dimensions
drive.set_wheel_base(0.6);  // 60cm between wheels
drive.set_wheel_radius(0.08);  // 8cm wheels

// Set velocity limits
drive.set_velocity_limits(1.5, 2.0);  // 1.5 m/s linear, 2.0 rad/s angular

// Reset odometry to origin
drive.reset_odometry();

// Get current position (x, y, theta)
let (x, y, theta) = drive.get_position();
```

### Message Types

**Twist** (input):
```rust
pub struct Twist {
    pub linear: [f64; 3],   // [x, y, z] in m/s
    pub angular: [f64; 3],  // [roll, pitch, yaw] in rad/s
}
```

**DifferentialDriveCommand** (output):
```rust
pub struct DifferentialDriveCommand {
    pub left_speed: f64,   // Left wheel speed
    pub right_speed: f64,  // Right wheel speed
}
```

**Odometry** (output):
```rust
pub struct Odometry {
    pub pose: Pose2D,           // Position (x, y, theta)
    pub twist: Twist,           // Velocities
    pub frame_id: [u8; 32],     // Reference frame ("odom")
    pub child_frame_id: [u8; 32], // Child frame ("base_link")
    pub timestamp: u64,
}
```

### Usage Example

```rust
use horus_library::nodes::DifferentialDriveNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create differential drive controller
    let mut drive = DifferentialDriveNode::new()?;

    // Configure for specific robot
    drive.set_wheel_base(0.45);      // 45cm wheelbase
    drive.set_wheel_radius(0.075);   // 7.5cm wheels
    drive.set_velocity_limits(1.0, 2.0);

    runtime.add_node(drive);
    runtime.run()?;

    Ok(())
}
```

---

## Camera Node

**Purpose**: Captures images from camera device and publishes as Image messages.

### Topics

**Publishers**:
- `camera/image_raw` (Image) - Raw camera images

### Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| device_id | u32 | 0 | Camera device ID |
| width | u32 | 640 | Image width |
| height | u32 | 480 | Image height |
| fps | u32 | 30 | Frames per second |
| format | String | "RGB8" | Pixel format |

### Public API

```rust
// Create with default settings
let mut camera = CameraNode::new()?;

// Create with custom topic
let mut camera = CameraNode::new_with_topic("camera/image")?;

// Configure camera
camera.set_device(0);
camera.set_resolution(1920, 1080);
camera.set_fps(60);
camera.set_format("YUYV");

// Get camera info
let (w, h, fps) = camera.get_info();
```

### Usage Example

```rust
use horus_library::nodes::CameraNode;
use horus_core::{Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create camera node for 720p @ 30fps
    let mut camera = CameraNode::new_with_topic("front_camera/image")?;
    camera.set_resolution(1280, 720);
    camera.set_fps(30);

    runtime.add_node(camera);
    runtime.run()?;

    Ok(())
}
```

---

## IMU Node

**Purpose**: Reads inertial measurement unit data (accelerometer, gyroscope, magnetometer).

### Topics

**Publishers**:
- `imu/data` (ImuData) - IMU measurements

### Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| device_path | String | "/dev/imu0" | IMU device path |
| sample_rate | u32 | 100 | Sampling rate (Hz) |
| accel_range | f32 | 16.0 | Accelerometer range (g) |
| gyro_range | f32 | 2000.0 | Gyroscope range (deg/s) |

### Public API

```rust
// Create with default settings
let mut imu = ImuNode::new()?;

// Create with custom topic
let mut imu = ImuNode::new_with_topic("imu/raw")?;

// Configure IMU
imu.set_device("/dev/i2c-1");
imu.set_sample_rate(200);
imu.set_ranges(8.0, 1000.0);  // accel_g, gyro_dps

// Calibrate
imu.calibrate()?;
```

### Message Type

**ImuData**:
```rust
pub struct ImuData {
    pub accel: [f32; 3],      // Acceleration [x, y, z] in m/s^2
    pub gyro: [f32; 3],       // Angular velocity [x, y, z] in rad/s
    pub mag: [f32; 3],        // Magnetic field [x, y, z] in uT
    pub temperature: f32,      // Temperature in Celsius
    pub timestamp: u64,
}
```

---

## Lidar Node

**Purpose**: Captures laser range finder data and publishes LaserScan messages.

### Topics

**Publishers**:
- `scan` (LaserScan) - Laser scan data

### Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| port | String | "/dev/ttyUSB0" | Serial port |
| baud_rate | u32 | 115200 | Serial baud rate |
| angle_min | f32 | 0.0 | Minimum scan angle (rad) |
| angle_max | f32 | 2π | Maximum scan angle (rad) |
| range_min | f32 | 0.15 | Minimum range (m) |
| range_max | f32 | 12.0 | Maximum range (m) |

### Public API

```rust
// Create with default settings
let mut lidar = LidarNode::new()?;

// Create with custom topic
let mut lidar = LidarNode::new_with_topic("front_scan")?;

// Configure lidar
lidar.set_port("/dev/ttyUSB0");
lidar.set_angle_range(0.0, std::f32::consts::PI * 2.0);
lidar.set_range_limits(0.1, 30.0);
```

### Message Type

**LaserScan**:
```rust
pub struct LaserScan {
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub ranges: Vec<f32>,
    pub intensities: Vec<f32>,
    pub timestamp: u64,
}
```

---

## Emergency Stop Node

**Purpose**: Monitors safety conditions and publishes emergency stop commands.

### Topics

**Subscribers**:
- `estop_trigger` (bool) - External emergency stop trigger
- `safety_status` (SafetyStatus) - Safety system status

**Publishers**:
- `estop_state` (EmergencyStopState) - Current E-stop state
- `motor_enable` (bool) - Motor enable/disable signal

### Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| auto_reset | bool | false | Automatically reset after clear |
| timeout_ms | u64 | 5000 | Watchdog timeout (ms) |

### Public API

```rust
// Create with default topics
let mut estop = EmergencyStopNode::new()?;

// Trigger emergency stop
estop.trigger();

// Reset emergency stop (if safe)
estop.reset()?;

// Check if stopped
let is_stopped = estop.is_stopped();

// Enable auto-reset
estop.set_auto_reset(true);
estop.set_timeout(3000);  // 3 second timeout
```

---

## Keyboard Input Node

**Purpose**: Captures keyboard input for teleoperation and control.

### Topics

**Publishers**:
- `cmd_vel` (Twist) - Velocity commands from keyboard
- `key_events` (KeyEvent) - Raw keyboard events

### Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| linear_speed | f32 | 0.5 | Linear velocity step (m/s) |
| angular_speed | f32 | 1.0 | Angular velocity step (rad/s) |
| enable_repeat | bool | true | Key repeat enabled |

### Public API

```rust
// Create with default settings
let mut keyboard = KeyboardInputNode::new()?;

// Create with custom topics
let mut keyboard = KeyboardInputNode::new_with_topics(
    "teleop/cmd_vel",
    "teleop/keys"
)?;

// Configure speeds
keyboard.set_linear_speed(1.0);
keyboard.set_angular_speed(2.0);

// Set key bindings
keyboard.set_forward_key('w');
keyboard.set_backward_key('s');
keyboard.set_left_key('a');
keyboard.set_right_key('d');
keyboard.set_stop_key(' ');
```

### Key Bindings (Default)

- `W` / `Up` - Move forward
- `S` / `Down` - Move backward
- `A` / `Left` - Turn left
- `D` / `Right` - Turn right
- `Space` - Stop
- `Q` - Quit

---

## Path Planner Node

**Purpose**: Computes collision-free paths from current position to goal.

### Topics

**Subscribers**:
- `goal` (Pose2D) - Target position
- `odom` (Odometry) - Current robot position
- `map` (OccupancyGrid) - Environment map

**Publishers**:
- `path` (Path) - Planned path
- `cmd_vel` (Twist) - Velocity commands to follow path

### Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| algorithm | String | "A*" | Planning algorithm |
| max_speed | f32 | 1.0 | Maximum speed (m/s) |
| goal_tolerance | f32 | 0.1 | Goal reached threshold (m) |
| replan_rate | f32 | 1.0 | Replanning frequency (Hz) |

### Public API

```rust
// Create with default settings
let mut planner = PathPlannerNode::new()?;

// Set algorithm
planner.set_algorithm("RRT");  // "A*", "RRT", "Dijkstra"

// Configure planning
planner.set_max_speed(2.0);
planner.set_goal_tolerance(0.05);
planner.set_replan_rate(2.0);

// Set goal
planner.set_goal(10.0, 5.0, 0.0);  // x, y, theta

// Check if goal reached
let reached = planner.is_goal_reached();
```

---

## Safety Monitor Node

**Purpose**: Monitors robot safety conditions and triggers protective actions.

### Topics

**Subscribers**:
- `odom` (Odometry) - Robot velocity
- `scan` (LaserScan) - Obstacle distances
- `battery` (BatteryState) - Battery status
- `temperature` (Temperature) - Component temperatures

**Publishers**:
- `safety_status` (SafetyStatus) - Overall safety state
- `warnings` (DiagnosticArray) - Safety warnings

### Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| max_velocity | f32 | 2.0 | Maximum safe velocity (m/s) |
| min_obstacle_dist | f32 | 0.3 | Minimum obstacle distance (m) |
| min_battery | f32 | 10.0 | Battery low threshold (%) |
| max_temp | f32 | 80.0 | Maximum temperature (C) |

### Public API

```rust
// Create with default settings
let mut monitor = SafetyMonitorNode::new()?;

// Configure safety limits
monitor.set_velocity_limit(1.5);
monitor.set_obstacle_threshold(0.5);
monitor.set_battery_threshold(15.0);
monitor.set_temperature_limit(70.0);

// Enable/disable specific checks
monitor.enable_velocity_check(true);
monitor.enable_obstacle_check(true);
monitor.enable_battery_check(true);
monitor.enable_temperature_check(true);
```

---

## Servo Controller Node

**Purpose**: Controls multiple servo motors with position/velocity commands.

### Topics

**Subscribers**:
- `servo_command` (ServoCommand) - Servo position/velocity commands

**Publishers**:
- `servo_state` (ServoState) - Current servo positions and status

### Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| num_servos | usize | 8 | Number of servos |
| device_path | String | "/dev/ttyUSB0" | Serial device |
| baud_rate | u32 | 115200 | Communication baud rate |

### Public API

```rust
// Create with default settings
let mut servos = ServoControllerNode::new()?;

// Configure servos
servos.set_num_servos(16);
servos.set_device("/dev/servo_controller");

// Set servo limits
servos.set_servo_limits(0, 0, 180);  // servo_id, min_deg, max_deg

// Set servo speed
servos.set_servo_speed(0, 60);  // servo_id, deg_per_sec

// Home all servos
servos.home_all();
```

---

## Digital I/O Node

**Purpose**: Controls digital input/output pins (GPIO).

### Topics

**Subscribers**:
- `digital_output` (DigitalOutput) - Output pin commands

**Publishers**:
- `digital_input` (DigitalInput) - Input pin states

### Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| num_inputs | usize | 8 | Number of input pins |
| num_outputs | usize | 8 | Number of output pins |
| pull_up | bool | true | Enable pull-up resistors |

### Public API

```rust
// Create with default settings
let mut io = DigitalIONode::new()?;

// Configure pins
io.set_input_pin(0, 17);   // input_id, gpio_pin
io.set_output_pin(0, 18);  // output_id, gpio_pin

// Enable pull-up resistors
io.enable_pullup(true);

// Set debounce time for inputs
io.set_debounce(50);  // 50ms
```

---

## Common Patterns

### Node Lifecycle

All nodes follow the same lifecycle:

```rust
use horus_core::{Node, Runtime};

// 1. Create node
let mut node = SomeNode::new()?;

// 2. Configure node
node.set_some_parameter(value);

// 3. Add to runtime
runtime.add_node(node);

// 4. Run (tick() called automatically)
runtime.run()?;
```

### Topic Naming Conventions

- Use lowercase with underscores: `cmd_vel`, `sensor_data`
- Use namespaces for organization: `camera/image_raw`, `lidar/front/scan`
- Suffix raw sensor data with `_raw`: `imu_raw`, `encoder_raw`
- Use standard names for common topics:
  - `cmd_vel` - Velocity commands
  - `odom` - Odometry
  - `scan` - Laser scan data
  - `image` - Camera images

### Error Handling

All node constructors return `HorusResult<Self>`:

```rust
use horus_core::error::HorusResult;

fn create_nodes() -> HorusResult<()> {
    let node1 = PidControllerNode::new()?;
    let node2 = DifferentialDriveNode::new()?;
    // ... use nodes
    Ok(())
}
```

### Custom Topics

Many nodes support custom topic names:

```rust
// Default topics
let node = PidControllerNode::new()?;

// Custom topics
let node = PidControllerNode::new_with_topics(
    "my_setpoint",
    "my_feedback",
    "my_output",
    "my_config"
)?;
```

---

## Message Type Reference

Common message types used across nodes:

### Twist
Velocity command (linear and angular):
```rust
pub struct Twist {
    pub linear: [f64; 3],   // [x, y, z] in m/s
    pub angular: [f64; 3],  // [roll, pitch, yaw] in rad/s
}
```

### Pose2D
2D position and orientation:
```rust
pub struct Pose2D {
    pub x: f64,      // Position x (m)
    pub y: f64,      // Position y (m)
    pub theta: f64,  // Orientation (rad)
}
```

### MotorCommand
Generic motor command:
```rust
pub enum MotorCommand {
    Velocity { motor_id: u8, value: f64 },
    Position { motor_id: u8, value: f64 },
    Torque { motor_id: u8, value: f64 },
}
```

### Image
Camera image data:
```rust
pub struct Image {
    pub width: u32,
    pub height: u32,
    pub encoding: String,
    pub data: Vec<u8>,
    pub timestamp: u64,
}
```

---

## See Also

- [HORUS Core Documentation](../../horus_core/README.md)
- [Message Types](../msgs/README.md)
- [Building Custom Nodes](../../docs/CUSTOM_NODES.md)
- [Example Applications](../apps/README.md)
