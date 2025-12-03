# Joystick Input Node

Gamepad and joystick input capture for robot teleoperation and manual control.

## Overview

The Joystick Input Node provides a standardized interface for capturing gamepad and joystick input events. It publishes joystick state messages including button presses, analog stick movements, D-pad input, and controller connection events. This node is essential for teleoperation, manual robot control, and human-in-the-loop applications.

The node supports multiple controllers simultaneously, handles axis mapping and deadzone processing, and provides a consistent cross-platform interface for various game controllers including Xbox, PlayStation, and generic USB gamepads.

## Topics

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `joystick_input` | `JoystickInput` | Raw joystick events (buttons, axes, hats, connections) |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `device_id` | `u32` | `0` | Joystick device identifier (0-3 for multiple controllers) |
| `deadzone` | `f32` | `0.1` | Axis deadzone threshold (0.0-1.0) to prevent drift |
| `axis_invert_x` | `bool` | `false` | Invert X-axis (left stick horizontal) |
| `axis_invert_y` | `bool` | `false` | Invert Y-axis (left stick vertical) |
| `axis_invert_rx` | `bool` | `false` | Invert RX-axis (right stick horizontal) |
| `axis_invert_ry` | `bool` | `false` | Invert RY-axis (right stick vertical) |
| `trigger_as_axis` | `bool` | `true` | Treat triggers as axes (0.0-1.0) instead of buttons |
| `poll_rate` | `u32` | `100` | Polling rate in Hz (10-1000) |
| `button_mapping` | `ButtonMap` | `Xbox360` | Button mapping profile (Xbox, PlayStation, Generic) |

## Message Types

### JoystickInput

Main joystick event message containing all controller state information:

```rust
pub struct JoystickInput {
    pub joystick_id: u32,       // Controller ID (0, 1, 2, 3...)
    pub event_type: [u8; 16],   // Event type: "button", "axis", "hat", "connection"
    pub element_id: u32,        // Button/axis/hat number
    pub element_name: [u8; 32], // Element name (e.g., "ButtonA", "LeftStickX")
    pub value: f32,             // Button: 0.0/1.0, Axis: -1.0 to 1.0
    pub pressed: bool,          // For buttons: true when pressed
    pub timestamp: u64,         // Unix timestamp in milliseconds
}
```

### Helper Methods

```rust
impl JoystickInput {
    // Create button event
    pub fn new_button(joystick_id: u32, button_id: u32, button_name: String, pressed: bool) -> Self;

    // Create axis event
    pub fn new_axis(joystick_id: u32, axis_id: u32, axis_name: String, value: f32) -> Self;

    // Create hat/D-pad event
    pub fn new_hat(joystick_id: u32, hat_id: u32, hat_name: String, value: f32) -> Self;

    // Create connection event
    pub fn new_connection(joystick_id: u32, connected: bool) -> Self;

    // Query methods
    pub fn get_event_type(&self) -> String;
    pub fn get_element_name(&self) -> String;
    pub fn is_button(&self) -> bool;
    pub fn is_axis(&self) -> bool;
    pub fn is_hat(&self) -> bool;
    pub fn is_connection_event(&self) -> bool;
    pub fn is_connected(&self) -> bool;
}
```

### Event Types

- **"button"** - Button press/release events
  - `value`: 0.0 (released) or 1.0 (pressed)
  - `pressed`: true when button is pressed
  - Common buttons: A, B, X, Y, LB, RB, Start, Select

- **"axis"** - Analog stick and trigger movements
  - `value`: -1.0 to 1.0 for sticks, 0.0 to 1.0 for triggers
  - Common axes: LeftStickX, LeftStickY, RightStickX, RightStickY, LeftTrigger, RightTrigger

- **"hat"** - D-pad directional input
  - `value`: Directional encoding (platform-dependent)
  - Typically 8-way directional input

- **"connection"** - Controller connect/disconnect
  - `value`: 1.0 (connected) or 0.0 (disconnected)
  - `element_name`: "Connected" or "Disconnected"

## Public API

### Construction

```rust
use horus_library::nodes::JoystickInputNode;

// Create with default topic "joystick_input"
let mut joystick = JoystickInputNode::new()?;

// Create with custom topic
let mut joystick = JoystickInputNode::new_with_topic("gamepad_events")?;
```

### Configuration Methods (Planned)

```rust
// Set device ID for multiple controllers
joystick.set_device_id(0);

// Configure axis deadzone
joystick.set_deadzone(0.15);

// Invert specific axes
joystick.set_axis_inversion(true, false, false, false);  // Invert X only

// Set button mapping profile
joystick.set_button_mapping(ButtonMapping::Xbox360);
joystick.set_button_mapping(ButtonMapping::PlayStation4);
joystick.set_button_mapping(ButtonMapping::Generic);

// Configure polling rate
joystick.set_poll_rate(100);  // 100 Hz

// Calibrate axes
joystick.calibrate_axes();

// Get current controller state
let connected = joystick.is_connected();
let battery_level = joystick.get_battery_level();
```

## Usage Examples

### Basic Robot Teleoperation

```rust
use horus_library::nodes::JoystickInputNode;
use horus_library::JoystickInput;
use horus_core::{Hub, Node, Runtime};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create joystick input node
    let joystick = JoystickInputNode::new()?;

    // Create a node to process joystick input
    let mut controller = TeleopNode::new()?;

    runtime.add_node(joystick);
    runtime.add_node(controller);
    runtime.run()?;

    Ok(())
}

// Teleoperation node that converts joystick to robot commands
struct TeleopNode {
    joystick_sub: Hub<JoystickInput>,
    cmd_pub: Hub<Twist>,
    max_speed: f32,
    max_turn: f32,
}

impl Node for TeleopNode {
    fn name(&self) -> &'static str {
        "TeleopNode"
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        // Process joystick events
        while let Ok(Some(input)) = self.joystick_sub.recv_latest() {
            if input.is_axis() {
                let axis_name = input.get_element_name();

                match axis_name.as_str() {
                    "LeftStickY" => {
                        // Forward/backward control
                        let linear = -input.value * self.max_speed;
                        let mut twist = Twist::default();
                        twist.linear[0] = linear as f64;
                        let _ = self.cmd_pub.send(twist, None);
                    }
                    "RightStickX" => {
                        // Rotation control
                        let angular = input.value * self.max_turn;
                        let mut twist = Twist::default();
                        twist.angular[2] = angular as f64;
                        let _ = self.cmd_pub.send(twist, None);
                    }
                    _ => {}
                }
            }
        }
    }
}
```

### Drone Control

```rust
use horus_library::nodes::JoystickInputNode;
use horus_library::JoystickInput;
use horus_core::{Hub, Node, Runtime};

struct DroneController {
    joystick_sub: Hub<JoystickInput>,
    cmd_pub: Hub<DroneCommand>,
    armed: bool,
}

impl Node for DroneController {
    fn name(&self) -> &'static str {
        "DroneController"
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        while let Ok(Some(input)) = self.joystick_sub.recv_latest() {
            // Button handling
            if input.is_button() {
                match input.get_element_name().as_str() {
                    "ButtonA" if input.pressed => {
                        // Arm/disarm toggle
                        self.armed = !self.armed;
                    }
                    "ButtonB" if input.pressed => {
                        // Emergency land
                        self.emergency_land();
                    }
                    "ButtonX" if input.pressed => {
                        // Take off
                        if self.armed {
                            self.takeoff();
                        }
                    }
                    _ => {}
                }
            }

            // Axis handling for flight control
            if input.is_axis() && self.armed {
                match input.get_element_name().as_str() {
                    "LeftStickY" => {
                        // Throttle (altitude control)
                        let throttle = (-input.value + 1.0) / 2.0;  // 0.0-1.0
                        self.set_throttle(throttle);
                    }
                    "LeftStickX" => {
                        // Yaw (rotation)
                        self.set_yaw(input.value);
                    }
                    "RightStickY" => {
                        // Pitch (forward/backward)
                        self.set_pitch(-input.value);
                    }
                    "RightStickX" => {
                        // Roll (left/right)
                        self.set_roll(input.value);
                    }
                    _ => {}
                }
            }
        }
    }
}

impl DroneController {
    fn emergency_land(&mut self) {
        // Emergency landing logic
    }

    fn takeoff(&mut self) {
        // Takeoff sequence
    }

    fn set_throttle(&mut self, value: f32) {
        // Throttle control
    }

    fn set_yaw(&mut self, value: f32) {
        // Yaw control
    }

    fn set_pitch(&mut self, value: f32) {
        // Pitch control
    }

    fn set_roll(&mut self, value: f32) {
        // Roll control
    }
}
```

### Advanced Button Mapping

```rust
use horus_library::nodes::JoystickInputNode;
use horus_library::JoystickInput;
use std::collections::HashMap;

struct ButtonMapper {
    joystick_sub: Hub<JoystickInput>,
    action_pub: Hub<RobotAction>,
    button_actions: HashMap<String, RobotAction>,
    button_states: HashMap<String, bool>,
}

impl ButtonMapper {
    fn new() -> Result<Self> {
        let mut button_actions = HashMap::new();

        // Map buttons to actions
        button_actions.insert("ButtonA".to_string(), RobotAction::Grab);
        button_actions.insert("ButtonB".to_string(), RobotAction::Release);
        button_actions.insert("ButtonX".to_string(), RobotAction::ToggleLight);
        button_actions.insert("ButtonY".to_string(), RobotAction::Horn);
        button_actions.insert("LB".to_string(), RobotAction::SpeedDecrease);
        button_actions.insert("RB".to_string(), RobotAction::SpeedIncrease);
        button_actions.insert("Start".to_string(), RobotAction::EmergencyStop);
        button_actions.insert("Select".to_string(), RobotAction::Reset);

        Ok(Self {
            joystick_sub: Hub::new("joystick_input")?,
            action_pub: Hub::new("robot_actions")?,
            button_actions,
            button_states: HashMap::new(),
        })
    }
}

impl Node for ButtonMapper {
    fn name(&self) -> &'static str {
        "ButtonMapper"
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        while let Ok(Some(input)) = self.joystick_sub.recv_latest() {
            if input.is_button() {
                let button_name = input.get_element_name();

                // Track button state changes
                let previous_state = self.button_states
                    .get(&button_name)
                    .copied()
                    .unwrap_or(false);

                if input.pressed != previous_state {
                    self.button_states.insert(button_name.clone(), input.pressed);

                    // Trigger action on button press
                    if input.pressed {
                        if let Some(action) = self.button_actions.get(&button_name) {
                            let _ = self.action_pub.send(action.clone(), None);
                        }
                    }
                }
            }
        }
    }
}

#[derive(Clone)]
enum RobotAction {
    Grab,
    Release,
    ToggleLight,
    Horn,
    SpeedIncrease,
    SpeedDecrease,
    EmergencyStop,
    Reset,
}
```

### Dual-Stick Tank Control

```rust
use horus_library::nodes::JoystickInputNode;
use horus_library::JoystickInput;

struct TankController {
    joystick_sub: Hub<JoystickInput>,
    left_motor_pub: Hub<MotorCommand>,
    right_motor_pub: Hub<MotorCommand>,
    left_speed: f32,
    right_speed: f32,
}

impl Node for TankController {
    fn name(&self) -> &'static str {
        "TankController"
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        while let Ok(Some(input)) = self.joystick_sub.recv_latest() {
            if input.is_axis() {
                match input.get_element_name().as_str() {
                    "LeftStickY" => {
                        // Left track control
                        self.left_speed = -input.value;
                        let cmd = MotorCommand::Velocity {
                            motor_id: 0,
                            value: self.left_speed as f64,
                        };
                        let _ = self.left_motor_pub.send(cmd, None);
                    }
                    "RightStickY" => {
                        // Right track control
                        self.right_speed = -input.value;
                        let cmd = MotorCommand::Velocity {
                            motor_id: 1,
                            value: self.right_speed as f64,
                        };
                        let _ = self.right_motor_pub.send(cmd, None);
                    }
                    _ => {}
                }
            }
        }
    }
}
```

## Supported Controllers

### Xbox Controllers

**Xbox 360 / Xbox One / Xbox Series X|S**

| Element | ID | Name | Range | Description |
|---------|-----|------|-------|-------------|
| Button A | 0 | ButtonA | 0/1 | Primary action button |
| Button B | 1 | ButtonB | 0/1 | Secondary action button |
| Button X | 2 | ButtonX | 0/1 | Tertiary action button |
| Button Y | 3 | ButtonY | 0/1 | Quaternary action button |
| LB | 4 | LeftBumper | 0/1 | Left shoulder button |
| RB | 5 | RightBumper | 0/1 | Right shoulder button |
| Start | 6 | Start | 0/1 | Start/menu button |
| Select | 7 | Select | 0/1 | Select/view button |
| L3 | 8 | LeftStick | 0/1 | Left stick click |
| R3 | 9 | RightStick | 0/1 | Right stick click |
| Left Stick X | 0 | LeftStickX | -1 to 1 | Horizontal movement |
| Left Stick Y | 1 | LeftStickY | -1 to 1 | Vertical movement |
| Right Stick X | 2 | RightStickX | -1 to 1 | Horizontal camera |
| Right Stick Y | 3 | RightStickY | -1 to 1 | Vertical camera |
| Left Trigger | 4 | LeftTrigger | 0 to 1 | Left trigger pressure |
| Right Trigger | 5 | RightTrigger | 0 to 1 | Right trigger pressure |

### PlayStation Controllers

**DualShock 4 / DualSense**

| Element | ID | Name | Range | Description |
|---------|-----|------|-------|-------------|
| Cross (✕) | 0 | ButtonCross | 0/1 | Primary action button |
| Circle (○) | 1 | ButtonCircle | 0/1 | Secondary action button |
| Square (□) | 2 | ButtonSquare | 0/1 | Tertiary action button |
| Triangle (△) | 3 | ButtonTriangle | 0/1 | Quaternary action button |
| L1 | 4 | L1 | 0/1 | Left shoulder button |
| R1 | 5 | R1 | 0/1 | Right shoulder button |
| Share | 6 | Share | 0/1 | Share button |
| Options | 7 | Options | 0/1 | Options button |
| L3 | 8 | L3 | 0/1 | Left stick click |
| R3 | 9 | R3 | 0/1 | Right stick click |
| Left Stick X | 0 | LeftStickX | -1 to 1 | Horizontal movement |
| Left Stick Y | 1 | LeftStickY | -1 to 1 | Vertical movement |
| Right Stick X | 2 | RightStickX | -1 to 1 | Horizontal camera |
| Right Stick Y | 3 | RightStickY | -1 to 1 | Vertical camera |
| L2 | 4 | L2 | 0 to 1 | Left trigger pressure |
| R2 | 5 | R2 | 0 to 1 | Right trigger pressure |

### Generic USB Gamepads

Most generic USB gamepads follow similar conventions:
- 4 face buttons (usually numbered 0-3)
- 2-4 shoulder buttons
- 2 analog sticks (4 axes)
- Optional triggers (2 axes)
- D-pad (hat or 4 buttons)

The node automatically detects controller type and applies appropriate mapping.

## Axis Calibration and Deadzone

### Deadzone Configuration

Deadzones prevent unintended movement from stick drift or resting position variations:

```rust
// Set global deadzone (applies to all axes)
joystick.set_deadzone(0.1);  // 10% deadzone

// Per-axis deadzones (planned)
joystick.set_axis_deadzone("LeftStickX", 0.15);
joystick.set_axis_deadzone("LeftStickY", 0.12);
```

**Recommended deadzone values:**
- **New controllers**: 0.05 - 0.10 (5-10%)
- **Standard usage**: 0.10 - 0.15 (10-15%)
- **Worn controllers**: 0.15 - 0.25 (15-25%)

### Deadzone Application

The deadzone is applied with smooth scaling to prevent sudden jumps:

```
if abs(value) < deadzone:
    output = 0.0
else:
    output = (value - sign(value) * deadzone) / (1.0 - deadzone)
```

This ensures:
- Small movements near center produce zero output
- Full deflection still produces ±1.0 output
- Smooth transition at deadzone boundary

### Axis Calibration

Calibration compensates for manufacturing variations:

```rust
// Run calibration routine
joystick.calibrate_axes();

// Manual calibration
joystick.calibrate_axis("LeftStickX",
    center: 0.02,      // Center position offset
    min: -0.98,        // Minimum value
    max: 1.0,          // Maximum value
);
```

**Calibration procedure:**
1. Leave all sticks centered
2. Call `calibrate_axes()`
3. Move each stick to full deflection in all directions
4. Node records min/max values and center offsets
5. Future readings are normalized to -1.0 to 1.0 range

## Troubleshooting

### Issue: Controller Not Detected

**Symptoms:**
- No joystick events received
- Connection events show "Disconnected"
- Node starts but produces no output

**Possible Causes:**
1. Controller not plugged in or powered on
2. Incorrect device ID
3. Permission issues (Linux)
4. Driver not installed (Windows)

**Solutions:**

```rust
// Check controller connection
if !joystick.is_connected() {
    eprintln!("Controller not detected on device {}", joystick.device_id);
}

// Try different device IDs
for id in 0..4 {
    let mut joy = JoystickInputNode::new()?;
    joy.set_device_id(id);
    if joy.is_connected() {
        eprintln!("Controller found on device {}", id);
        break;
    }
}
```

**Linux permissions:**
```bash
# Add user to input group
sudo usermod -a -G input $USER

# Set udev rules for joystick access
echo 'KERNEL=="js*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-joystick.rules
sudo udevadm control --reload-rules
```

**Windows drivers:**
- Xbox controllers: Install Xbox Accessories app
- PlayStation controllers: Install DS4Windows or DualSense drivers
- Generic controllers: Usually work with built-in HID drivers

### Issue: Axis Drift

**Symptoms:**
- Robot moves without stick input
- Non-zero axis values when stick is centered
- Gradual drift over time

**Possible Causes:**
1. Deadzone too small
2. Controller wear (potentiometer drift)
3. No calibration applied
4. Electrical interference

**Solutions:**

```rust
// Increase deadzone
joystick.set_deadzone(0.2);  // 20% deadzone for worn controllers

// Calibrate axes
joystick.calibrate_axes();

// Apply exponential response curve to reduce sensitivity near center
fn apply_expo(value: f32, expo: f32) -> f32 {
    value.abs().powf(expo) * value.signum()
}

let processed_value = apply_expo(input.value, 1.5);  // Expo factor 1.5
```

**Hardware solutions:**
- Clean controller contacts
- Replace worn controller
- Use compressed air to clean stick mechanisms

### Issue: Incorrect Button Mapping

**Symptoms:**
- Button A does different action than expected
- Axis movements are swapped
- Inverted axis directions

**Solutions:**

```rust
// Set correct button mapping profile
joystick.set_button_mapping(ButtonMapping::PlayStation4);

// Invert specific axes
joystick.set_axis_inversion(
    invert_x: false,
    invert_y: true,   // Invert Y for aircraft-style controls
    invert_rx: false,
    invert_ry: false
);

// Create custom mapping
let mut mapper = ButtonMapper::new()?;
mapper.map_button("ButtonA", RobotAction::Grab);
mapper.map_button("ButtonB", RobotAction::Release);
mapper.map_axis("LeftStickY", AxisAction::Forward);
```

### Issue: Delayed or Laggy Response

**Symptoms:**
- Noticeable delay between input and robot response
- Choppy or jerky movement
- Missed button presses

**Possible Causes:**
1. Low polling rate
2. Slow tick rate
3. Network latency (remote control)
4. USB bus saturation

**Solutions:**

```rust
// Increase polling rate
joystick.set_poll_rate(125);  // 125 Hz (8ms)

// Use wired connection instead of wireless
// Wired: ~1-2ms latency
// Wireless: ~4-8ms latency

// Optimize runtime tick rate
runtime.set_tick_rate(100.0);  // 100 Hz (10ms)
```

**Performance benchmarks:**
- USB polling: 125-1000 Hz (1-8ms)
- Bluetooth: 125-250 Hz (4-8ms)
- Recommended robot control: 50-100 Hz

### Issue: Battery Drain

**Symptoms:**
- Controller disconnects frequently
- Low battery warnings
- Reduced operating time

**Solutions:**

```rust
// Check battery level
let battery = joystick.get_battery_level();
if battery < 0.2 {
    eprintln!("Warning: Controller battery low ({}%)", battery * 100.0);
}

// Reduce polling rate to save battery
joystick.set_poll_rate(60);  // 60 Hz instead of 125 Hz

// Disable vibration/haptics (if supported)
joystick.set_rumble(0.0, 0.0);
```

**Battery optimization:**
- Use wired mode when possible
- Reduce poll rate for non-critical applications
- Disable LED lights/haptics
- Use auto-sleep features

## Integration with Motion Control

### Differential Drive Integration

```rust
use horus_library::nodes::{JoystickInputNode, DifferentialDriveNode};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Joystick input
    let joystick = JoystickInputNode::new()?;

    // Convert joystick to Twist messages
    let teleop = TeleopNode::new_with_topics(
        "joystick_input",
        "cmd_vel"
    )?;

    // Differential drive controller
    let mut drive = DifferentialDriveNode::new()?;
    drive.set_wheel_base(0.5);
    drive.set_velocity_limits(1.0, 2.0);

    runtime.add_node(joystick);
    runtime.add_node(teleop);
    runtime.add_node(drive);
    runtime.run()?;

    Ok(())
}
```

### PID Controller Integration

```rust
use horus_library::nodes::{JoystickInputNode, PidControllerNode};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Joystick input
    let joystick = JoystickInputNode::new()?;

    // Convert joystick axis to position setpoint
    let setpoint_converter = AxisToSetpointNode::new(
        "joystick_input",
        "LeftStickX",
        "position_setpoint",
        -90.0,   // Min angle
        90.0,    // Max angle
    )?;

    // PID controller for servo position
    let mut pid = PidControllerNode::new_with_topics(
        "position_setpoint",
        "encoder_position",
        "servo_command",
        "pid_config"
    )?;
    pid.set_gains(2.0, 0.5, 0.1);

    runtime.add_node(joystick);
    runtime.add_node(setpoint_converter);
    runtime.add_node(pid);
    runtime.run()?;

    Ok(())
}
```

### Multi-Axis Coordinated Control

```rust
struct CoordinatedArmController {
    joystick_sub: Hub<JoystickInput>,
    joint_publishers: Vec<Hub<f32>>,
    joint_positions: Vec<f32>,
}

impl Node for CoordinatedArmController {
    fn name(&self) -> &'static str {
        "CoordinatedArmController"
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        while let Ok(Some(input)) = self.joystick_sub.recv_latest() {
            if input.is_axis() {
                match input.get_element_name().as_str() {
                    "LeftStickX" => {
                        // Shoulder joint
                        self.joint_positions[0] = input.value;
                        let _ = self.joint_publishers[0].send(input.value, None);
                    }
                    "LeftStickY" => {
                        // Elbow joint
                        self.joint_positions[1] = -input.value;
                        let _ = self.joint_publishers[1].send(-input.value, None);
                    }
                    "RightStickX" => {
                        // Wrist rotation
                        self.joint_positions[2] = input.value;
                        let _ = self.joint_publishers[2].send(input.value, None);
                    }
                    "RightStickY" => {
                        // Wrist pitch
                        self.joint_positions[3] = -input.value;
                        let _ = self.joint_publishers[3].send(-input.value, None);
                    }
                    "LeftTrigger" | "RightTrigger" => {
                        // Gripper control
                        let grip = if input.get_element_name() == "LeftTrigger" {
                            -input.value  // Close
                        } else {
                            input.value   // Open
                        };
                        self.joint_positions[4] = grip;
                        let _ = self.joint_publishers[4].send(grip, None);
                    }
                    _ => {}
                }
            }
        }
    }
}
```

## Performance Considerations

### Polling Rate vs CPU Usage

| Poll Rate | Latency | CPU Usage | Use Case |
|-----------|---------|-----------|----------|
| 30 Hz | ~33ms | Very Low | Slow robots, monitoring |
| 60 Hz | ~16ms | Low | Standard teleoperation |
| 100 Hz | ~10ms | Medium | Responsive control |
| 125 Hz | ~8ms | Medium | High-performance control |
| 250 Hz | ~4ms | High | Precision applications |
| 1000 Hz | ~1ms | Very High | Research/special applications |

**Recommended settings:**
- **Mobile robots**: 50-100 Hz
- **Robotic arms**: 100-125 Hz
- **Drones**: 100-250 Hz
- **Industrial machines**: 60-125 Hz

### Memory Footprint

- **Node state**: ~200 bytes
- **Message size**: 80 bytes per event
- **Minimal allocation**: Events are stack-allocated where possible

### Event Processing

- **Average processing time**: < 10 microseconds per event
- **Maximum event rate**: > 10,000 events/second
- **Queue depth**: 100 events (configurable)

## Related Nodes

- **DifferentialDriveNode**: Mobile robot base control
- **PidControllerNode**: Closed-loop servo control
- **ServoControllerNode**: Multi-joint robot control
- **EmergencyStopNode**: Safety system integration
- **TeleopNode**: Custom teleoperation logic

## See Also

- [SDL2 Game Controller Documentation](https://wiki.libsdl.org/CategoryGameController)
- [USB HID Specification](https://www.usb.org/hid)
- [Xbox Controller Protocol](https://docs.microsoft.com/en-us/gaming/xbox-live/controls)
- [DualShock 4 Technical Details](https://www.psdevwiki.com/ps4/DualShock_4)
- [Joystick Input Best Practices for Robotics](https://robotics.stackexchange.com/)
