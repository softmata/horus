# Digital I/O Node

Basic digital input/output node for industrial sensors and actuators with GPIO support.

## Overview

The Digital I/O Node manages digital input and output pins for reading industrial sensors (limit switches, proximity sensors, push buttons) and controlling actuators (relays, solenoids, LEDs, indicators). It supports both GPIO pins (Raspberry Pi) and industrial I/O modules, providing debounced inputs, configurable update rates, and real-time state monitoring.

Key features:
- Configurable input/output pin counts (up to 32 pins each)
- Change-detection publishing for minimal latency
- Pin naming for intuitive monitoring
- Hardware abstraction for GPIO and industrial I/O
- Input simulation mode for testing
- Debounced input reading

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `digital_output` | `DigitalIO` | Commands to set output pin states |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `digital_input` | `DigitalIO` | Current state of all input pins |
| `io_status` | `DigitalIO` | Combined status of all input and output pins |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_pin_count` | `u8` | `8` | Number of digital input pins |
| `output_pin_count` | `u8` | `8` | Number of digital output pins |
| `update_rate` | `f32` | `10.0` | Publishing rate in Hz (0.1-1000.0) |
| `simulate_inputs` | `bool` | `true` | Enable input simulation mode |
| `input_pin_names` | `HashMap<u8, String>` | `DI0-DI7` | Custom names for input pins |
| `output_pin_names` | `HashMap<u8, String>` | `DO0-DO7` | Custom names for output pins |

### Pin Configuration (Raspberry Pi GPIO)

When using the `raspberry-pi` feature, pins are mapped to BCM GPIO numbers:

| BCM Pin | Common Use | Notes |
|---------|------------|-------|
| GPIO 2-3 | I2C (Reserved) | Avoid for digital I/O |
| GPIO 4 | General I/O | Safe to use |
| GPIO 5-13 | General I/O | Safe to use |
| GPIO 14-15 | UART (Optional) | Can be used if UART disabled |
| GPIO 17-27 | General I/O | Safe to use |

## Message Types

### DigitalIO

Message structure for digital I/O state and commands:

```rust
pub struct DigitalIO {
    /// Number of active pins (1-32)
    pub pin_count: u8,

    /// State of each pin (true = HIGH/ON, false = LOW/OFF)
    pub pins: [bool; 32],

    /// Human-readable labels for each pin (null-terminated)
    pub pin_labels: [[u8; 16]; 32],

    /// Timestamp in nanoseconds since UNIX epoch
    pub timestamp: u64,

    // Additional fields...
}
```

### Pin States

- `true` / `HIGH` / `1`: Pin is active (3.3V for Raspberry Pi GPIO)
- `false` / `LOW` / `0`: Pin is inactive (0V for Raspberry Pi GPIO)

## Public API

### Construction

```rust
use horus_library::nodes::DigitalIONode;

// Create with default topics
let mut io_node = DigitalIONode::new()?;

// Create with custom topics
let mut io_node = DigitalIONode::new_with_topics(
    "sensor_inputs",     // input topic
    "actuator_outputs",  // output topic
    "io_diagnostics"     // status topic
)?;
```

### Configuration Methods

```rust
// Set number of I/O pins
io_node.set_pin_counts(16, 8);  // 16 inputs, 8 outputs

// Set update rate in Hz
io_node.set_update_rate(50.0);  // 50 Hz = 20ms update interval

// Set pin names for monitoring
io_node.set_input_pin_name(0, "limit_switch_x");
io_node.set_input_pin_name(1, "limit_switch_y");
io_node.set_input_pin_name(2, "emergency_stop");
io_node.set_output_pin_name(0, "main_relay");
io_node.set_output_pin_name(1, "status_led");

// Enable/disable simulation mode
io_node.set_simulation(false);  // Use real hardware
```

### Runtime Methods

```rust
// Read input pin state
if let Some(state) = io_node.get_input(0) {
    eprintln!("Limit switch: {}", if state { "TRIGGERED" } else { "OPEN" });
}

// Read output pin state
if let Some(state) = io_node.get_output(0) {
    eprintln!("Relay: {}", if state { "ON" } else { "OFF" });
}

// Set output pin directly (programmatic control)
io_node.set_output(0, true);   // Turn on relay
io_node.set_output(1, false);  // Turn off LED
```

## Usage Examples

### Basic Button Input and LED Control

```rust
use horus_library::nodes::DigitalIONode;
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create digital I/O node
    let mut io_node = DigitalIONode::new()?;
    io_node.set_pin_counts(4, 4);  // 4 inputs, 4 outputs
    io_node.set_update_rate(20.0); // 20 Hz
    io_node.set_simulation(false); // Use real GPIO

    // Name the pins
    io_node.set_input_pin_name(0, "button_1");
    io_node.set_input_pin_name(1, "button_2");
    io_node.set_output_pin_name(0, "led_green");
    io_node.set_output_pin_name(1, "led_red");

    runtime.add_node(io_node);
    runtime.run()?;

    Ok(())
}
```

### Industrial Relay Control

```rust
use horus_library::nodes::DigitalIONode;
use horus_core::{Node, Runtime, Hub};
use crate::DigitalIO;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create I/O node for relay control
    let mut relay_controller = DigitalIONode::new_with_topics(
        "sensor_status",
        "relay_commands",
        "relay_status"
    )?;

    relay_controller.set_pin_counts(0, 8);  // 8 relay outputs
    relay_controller.set_update_rate(10.0);
    relay_controller.set_simulation(false);

    // Name the relay outputs
    relay_controller.set_output_pin_name(0, "pump_1");
    relay_controller.set_output_pin_name(1, "pump_2");
    relay_controller.set_output_pin_name(2, "valve_inlet");
    relay_controller.set_output_pin_name(3, "valve_outlet");
    relay_controller.set_output_pin_name(4, "heater");
    relay_controller.set_output_pin_name(5, "cooler");
    relay_controller.set_output_pin_name(6, "alarm");
    relay_controller.set_output_pin_name(7, "status_light");

    runtime.add_node(relay_controller);
    runtime.run()?;

    Ok(())
}
```

### Limit Switch Monitoring

```rust
use horus_library::nodes::DigitalIONode;
use horus_core::{Node, Runtime, Hub};
use crate::DigitalIO;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create I/O node for CNC machine limit switches
    let mut limit_switches = DigitalIONode::new_with_topics(
        "limit_status",
        "unused_output",
        "safety_status"
    )?;

    limit_switches.set_pin_counts(6, 0);  // 6 limit switches, no outputs
    limit_switches.set_update_rate(100.0); // Fast updates for safety
    limit_switches.set_simulation(false);

    // Name the limit switches
    limit_switches.set_input_pin_name(0, "x_min");
    limit_switches.set_input_pin_name(1, "x_max");
    limit_switches.set_input_pin_name(2, "y_min");
    limit_switches.set_input_pin_name(3, "y_max");
    limit_switches.set_input_pin_name(4, "z_min");
    limit_switches.set_input_pin_name(5, "z_max");

    // Subscribe to limit switch status
    let mut status_sub = Hub::<DigitalIO>::new("limit_status")?;

    runtime.add_node(limit_switches);
    runtime.run()?;

    Ok(())
}
```

### Emergency Stop and Safety System

```rust
use horus_library::nodes::DigitalIONode;
use horus_core::{Node, Runtime, Hub};
use crate::DigitalIO;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Safety monitoring node
    let mut safety_io = DigitalIONode::new_with_topics(
        "safety_inputs",
        "safety_outputs",
        "safety_status"
    )?;

    safety_io.set_pin_counts(4, 4);
    safety_io.set_update_rate(50.0);  // 50 Hz for fast safety response
    safety_io.set_simulation(false);

    // Safety inputs
    safety_io.set_input_pin_name(0, "emergency_stop");
    safety_io.set_input_pin_name(1, "safety_gate");
    safety_io.set_input_pin_name(2, "light_curtain");
    safety_io.set_input_pin_name(3, "enable_switch");

    // Safety outputs
    safety_io.set_output_pin_name(0, "main_contactor");
    safety_io.set_output_pin_name(1, "brake_release");
    safety_io.set_output_pin_name(2, "alarm_horn");
    safety_io.set_output_pin_name(3, "warning_light");

    runtime.add_node(safety_io);
    runtime.run()?;

    Ok(())
}
```

### Mixed I/O Control with State Machine

```rust
use horus_library::nodes::DigitalIONode;
use horus_core::{Node, Runtime, Hub};
use crate::DigitalIO;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create I/O node
    let mut io = DigitalIONode::new()?;
    io.set_pin_counts(8, 8);
    io.set_update_rate(20.0);
    io.set_simulation(false);

    // Configure inputs
    io.set_input_pin_name(0, "start_button");
    io.set_input_pin_name(1, "stop_button");
    io.set_input_pin_name(2, "door_closed");
    io.set_input_pin_name(3, "part_present");

    // Configure outputs
    io.set_output_pin_name(0, "conveyor_motor");
    io.set_output_pin_name(1, "gripper_solenoid");
    io.set_output_pin_name(2, "indicator_running");
    io.set_output_pin_name(3, "indicator_fault");

    // Subscribe to inputs and publish outputs
    let mut input_sub = Hub::<DigitalIO>::new("digital_input")?;
    let mut output_pub = Hub::<DigitalIO>::new("digital_output")?;

    runtime.add_node(io);

    // State machine logic would go here
    // Read inputs, make decisions, publish output commands

    runtime.run()?;

    Ok(())
}
```

## GPIO Pin Configuration

### Raspberry Pi GPIO Setup

When using the `raspberry-pi` feature, the node uses the `rppal` library for GPIO access.

#### Input Configuration

```rust
// Inputs are configured with internal pull-up resistors by default
// This means:
// - Open/unpressed = HIGH (true)
// - Closed/pressed = LOW (false)
// - Suitable for normally-open switches connected to GND
```

**Hardware Connection for Inputs:**
```
Button/Switch:
  GPIO Pin ---[Switch]--- GND
  (Internal pull-up resistor to 3.3V)

Limit Switch (NO):
  GPIO Pin ---[Switch]--- GND

Proximity Sensor (NPN):
  GPIO Pin --- Sensor Output
  Sensor GND --- System GND
  Sensor V+ --- 5V or 12V (per sensor spec)
```

#### Output Configuration

```rust
// Outputs are configured as push-pull outputs
// HIGH = 3.3V, LOW = 0V
// Maximum current: 16mA per pin
// Use transistors/relays for higher current loads
```

**Hardware Connection for Outputs:**

```
LED (Direct - Low Current):
  GPIO Pin ---[220Ω Resistor]---[LED]--- GND

Relay Module:
  GPIO Pin --- Relay Input
  Relay GND --- System GND
  Relay VCC --- 5V

MOSFET Driver (High Current):
  GPIO Pin --- MOSFET Gate (via 100Ω resistor)
  MOSFET Source --- GND
  MOSFET Drain --- Load --- V+
```

### GPIO Permissions (Linux)

To access GPIO without root:

```bash
# Add user to gpio group
sudo usermod -a -G gpio $USER

# Create udev rule for GPIO access
echo 'SUBSYSTEM=="gpio", GROUP="gpio", MODE="0660"' | \
  sudo tee /etc/udev/rules.d/99-gpio.rules

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Logout and login for group changes to take effect
```

## Debouncing Strategies

### Hardware Debouncing

**RC Filter** (Simple, effective for most applications):
```
Button --- [10kΩ Resistor] --- GPIO Pin
                |
           [100nF Cap]
                |
               GND
```
Time constant: ~1ms (sufficient for mechanical switches)

**Schmitt Trigger IC** (Best for noisy environments):
- Use 74HC14 or similar Schmitt trigger buffer
- Provides hysteresis for clean transitions
- Recommended for industrial applications

### Software Debouncing

The Digital I/O Node implements change-detection which provides implicit debouncing:

```rust
// Inputs are published only on state change
// Rapid bounces within the update interval are filtered

// For 10 Hz update rate (100ms interval):
// - Bounces < 100ms are ignored
// - Effectively debounces mechanical switches

// For better debouncing, reduce update rate:
io_node.set_update_rate(5.0);  // 200ms debounce time
```

**Advanced Software Debouncing Pattern:**

```rust
use horus_core::Hub;
use crate::DigitalIO;
use std::time::{Duration, Instant};

struct DebouncedInput {
    last_change: Instant,
    stable_state: bool,
    debounce_time: Duration,
}

impl DebouncedInput {
    fn new(debounce_ms: u64) -> Self {
        Self {
            last_change: Instant::now(),
            stable_state: false,
            debounce_time: Duration::from_millis(debounce_ms),
        }
    }

    fn update(&mut self, new_state: bool) -> Option<bool> {
        let now = Instant::now();

        if new_state != self.stable_state {
            if now.duration_since(self.last_change) > self.debounce_time {
                self.stable_state = new_state;
                self.last_change = now;
                return Some(new_state);  // Stable state change
            }
        } else {
            self.last_change = now;  // Reset timer
        }

        None  // No stable change yet
    }
}
```

## Troubleshooting

### Issue: Permission denied when accessing GPIO

**Cause**: User not in gpio group or insufficient permissions

**Solution**:
```bash
# Check current groups
groups

# Add user to gpio group (if not present)
sudo usermod -a -G gpio $USER

# For Raspberry Pi, also add to spi and i2c groups
sudo usermod -a -G spi,i2c,gpio $USER

# Reboot or logout/login
sudo reboot
```

### Issue: Pin conflicts with other peripherals

**Cause**: GPIO pins used by I2C, SPI, UART, or other systems

**Solution**:
```rust
// Avoid these BCM pins on Raspberry Pi:
// GPIO 2-3: I2C (SCL/SDA)
// GPIO 7-11: SPI (CE0, CE1, MISO, MOSI, SCLK)
// GPIO 14-15: UART (TX/RX)

// Use safe GPIO pins:
// GPIO 4, 5, 6, 12, 13, 16, 17, 18, 22, 23, 24, 25, 26, 27

// Example safe configuration:
let safe_input_pins = [4, 5, 6, 12, 13, 16];  // BCM numbering
let safe_output_pins = [17, 18, 22, 23, 24, 25];
```

### Issue: Inputs always read HIGH or LOW

**Cause**: Missing pull-up/pull-down resistors or incorrect wiring

**Solution**:
```rust
// For Raspberry Pi, inputs use internal pull-up by default
// Connect normally-open switches to GND

// If switch connects to 3.3V instead:
// - Add external 10kΩ pull-down resistor to GND
// - Or modify code to use pull-down (requires feature flag)

// Check with multimeter:
// - Open switch should read ~3.3V (pull-up active)
// - Closed switch should read ~0V
```

### Issue: Outputs don't control external devices

**Cause**: Insufficient current or wrong voltage level

**Solution**:
```rust
// GPIO pins provide only 3.3V @ 16mA max
// Use transistor/relay for higher loads:

// For 5V devices:
// GPIO --- [1kΩ] --- NPN Base
//                    NPN Collector --- Device --- 5V
//                    NPN Emitter --- GND

// For 12V/24V industrial devices:
// GPIO --- Relay Module --- Industrial Load
// Relay module isolates GPIO from high voltage
```

### Issue: Erratic input readings / bouncing

**Cause**: Switch bounce or electrical noise

**Solution**:
```rust
// 1. Reduce update rate for longer debounce
io_node.set_update_rate(5.0);  // 200ms debounce

// 2. Add hardware RC filter (see Debouncing Strategies)

// 3. For industrial environments, use opto-isolated inputs

// 4. Implement software debounce counter:
let mut debounce_count = 0;
const DEBOUNCE_THRESHOLD: u32 = 3;

if input_changed {
    debounce_count += 1;
    if debounce_count >= DEBOUNCE_THRESHOLD {
        // Confirmed stable change
        debounce_count = 0;
    }
} else {
    debounce_count = 0;
}
```

### Issue: Simulation mode won't disable

**Cause**: Feature flag not set or hardware access failing silently

**Solution**:
```rust
// Ensure raspberry-pi feature is enabled in Cargo.toml:
// horus_library = { version = "0.1", features = ["raspberry-pi"] }

// Check if GPIO hardware is accessible:
io_node.set_simulation(false);

// Read test pin to verify hardware access
if let Some(state) = io_node.get_input(0) {
    eprintln!("Hardware access OK: pin 0 = {}", state);
} else {
    eprintln!("Hardware access failed - check permissions");
}
```

## Integration with Control Systems

### Emergency Stop Integration

```rust
use horus_library::nodes::DigitalIONode;
use horus_core::{Node, Hub};
use crate::DigitalIO;

// Monitor emergency stop button
let mut io_node = DigitalIONode::new()?;
io_node.set_input_pin_name(0, "emergency_stop");
io_node.set_update_rate(50.0);  // Fast response

let mut input_sub = Hub::<DigitalIO>::new("digital_input")?;

// In control loop:
if let Some(inputs) = input_sub.recv(None) {
    let estop_pressed = inputs.pins[0];  // Active LOW with pull-up
    if !estop_pressed {  // Button pressed (LOW)
        // Immediately halt all motion
        // Set all outputs to safe state
        // Publish emergency stop message
        eprintln!("EMERGENCY STOP ACTIVATED");
    }
}
```

### Interlock System

```rust
use horus_library::nodes::DigitalIONode;
use horus_core::{Node, Hub};
use crate::DigitalIO;

// Safety interlock example
let mut io = DigitalIONode::new()?;
io.set_pin_counts(3, 1);

io.set_input_pin_name(0, "door_closed");
io.set_input_pin_name(1, "guard_in_place");
io.set_input_pin_name(2, "enable_key");
io.set_output_pin_name(0, "machine_enable");

let mut input_sub = Hub::<DigitalIO>::new("digital_input")?;
let mut output_pub = Hub::<DigitalIO>::new("digital_output")?;

// Safety logic
if let Some(inputs) = input_sub.recv(None) {
    let door_ok = inputs.pins[0];
    let guard_ok = inputs.pins[1];
    let key_ok = inputs.pins[2];

    // All conditions must be met
    let safe_to_run = door_ok && guard_ok && key_ok;

    // Send output command
    let mut output = DigitalIO::default();
    output.pin_count = 1;
    output.pins[0] = safe_to_run;
    output_pub.send(output, None)?;
}
```

### Status Indication

```rust
use horus_library::nodes::DigitalIONode;
use horus_core::Node;

// Triple-light tower: Red/Yellow/Green status indication
let mut indicators = DigitalIONode::new_with_topics(
    "unused_input",
    "indicator_commands",
    "indicator_status"
)?;

indicators.set_pin_counts(0, 3);
indicators.set_output_pin_name(0, "red_light");
indicators.set_output_pin_name(1, "yellow_light");
indicators.set_output_pin_name(2, "green_light");

// Set status based on system state
enum SystemState {
    Stopped,
    Running,
    Warning,
    Fault,
}

fn set_indicator_lights(state: SystemState, io: &mut DigitalIONode) {
    match state {
        SystemState::Stopped => {
            io.set_output(0, false);  // Red OFF
            io.set_output(1, false);  // Yellow OFF
            io.set_output(2, false);  // Green OFF
        },
        SystemState::Running => {
            io.set_output(0, false);  // Red OFF
            io.set_output(1, false);  // Yellow OFF
            io.set_output(2, true);   // Green ON
        },
        SystemState::Warning => {
            io.set_output(0, false);  // Red OFF
            io.set_output(1, true);   // Yellow ON
            io.set_output(2, false);  // Green OFF
        },
        SystemState::Fault => {
            io.set_output(0, true);   // Red ON
            io.set_output(1, false);  // Yellow OFF
            io.set_output(2, false);  // Green OFF
        },
    }
}
```

### Sensor Fusion with Other Nodes

```rust
use horus_library::nodes::{DigitalIONode, PidControllerNode};
use horus_core::{Node, Runtime, Hub};
use crate::DigitalIO;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Digital I/O for limit switches
    let mut io = DigitalIONode::new()?;
    io.set_pin_counts(2, 0);
    io.set_input_pin_name(0, "home_switch");
    io.set_input_pin_name(1, "limit_switch");

    // PID controller for position control
    let mut pid = PidControllerNode::new()?;
    pid.set_gains(2.0, 0.5, 0.1);

    // Subscribe to limit switch status
    let mut limit_sub = Hub::<DigitalIO>::new("digital_input")?;

    // Control logic (in separate node):
    // - Monitor limit switches
    // - Disable PID when limit hit
    // - Initiate homing sequence on home switch

    runtime.add_node(io);
    runtime.add_node(pid);
    runtime.run()?;

    Ok(())
}
```

## Performance Considerations

### Update Rate Selection

Choose update rate based on application requirements:

| Application | Recommended Rate | Rationale |
|-------------|------------------|-----------|
| Button inputs | 10-20 Hz | Human reaction time ~100ms |
| Limit switches | 50-100 Hz | Fast response for safety |
| Status indicators | 5-10 Hz | Visual updates don't need high rate |
| Relay control | 10-20 Hz | Relay switching speed ~10ms |
| Emergency stop | 100+ Hz | Critical safety function |

```rust
// Safety-critical inputs: high rate
safety_io.set_update_rate(100.0);  // 10ms latency

// User interface: moderate rate
ui_io.set_update_rate(20.0);  // 50ms latency

// Status displays: low rate
status_io.set_update_rate(5.0);  // 200ms latency
```

### CPU Usage

- GPIO read/write: ~1-5 microseconds per pin
- Message publishing: ~10-50 microseconds
- Total overhead: Minimal, suitable for real-time systems

```rust
// Example CPU usage for 16 inputs, 8 outputs @ 50 Hz:
// - 16 GPIO reads: ~80 µs
// - 8 GPIO writes: ~40 µs
// - Publishing: ~30 µs
// Total: ~150 µs per update = 7.5ms/s CPU time
```

### Memory Usage

- Fixed overhead: ~1 KB per node
- Per-pin overhead: ~20 bytes
- Example: 16 inputs + 8 outputs = ~1.5 KB total

## Implementation Details

### Change Detection Algorithm

The node publishes inputs immediately when any pin state changes, minimizing latency for critical signals:

```
For each update cycle:
  1. Read all input pins
  2. Compare with last known states
  3. If any pin changed:
     - Publish immediately
     - Update last known states
  4. Else if time since last publish > interval:
     - Publish periodic update
```

This ensures:
- Fast response to critical events (emergency stops, limit switches)
- Regular status updates even when no changes occur
- Minimal message traffic during steady state

### GPIO Abstraction

The node uses conditional compilation for hardware support:

```rust
// With raspberry-pi feature:
#[cfg(feature = "raspberry-pi")]
fn read_gpio_pin(&self, pin: u8) -> bool {
    // Use rppal library for actual GPIO access
}

// Without raspberry-pi feature (simulation/testing):
#[cfg(not(feature = "raspberry-pi"))]
fn read_gpio_pin(&self, pin: u8) -> bool {
    false  // Return safe default
}
```

This allows the same code to run on:
- Raspberry Pi with GPIO hardware
- Development machine (simulation mode)
- Industrial PC with I/O modules (extend with custom implementation)

## Related Nodes

- **PidControllerNode**: Use limit switch inputs to constrain PID control
- **EncoderNode**: Combine with home switch for absolute positioning
- **ServoControllerNode**: Use digital outputs to enable/disable servos
- **DifferentialDriveNode**: Use bumper switches as safety inputs

## See Also

- [Raspberry Pi GPIO Documentation](https://www.raspberrypi.com/documentation/computers/os.html#gpio)
- [RPPAL Library](https://github.com/golemparts/rppal)
- [Industrial I/O Best Practices](https://www.automation.com/en-us/articles/digital-io-basics)
- [Switch Debouncing Guide](https://www.ganssle.com/debouncing.htm)
