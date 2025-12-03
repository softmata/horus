# I2C Bus Communication Node

I2C/TWI bus interface for sensors, displays, EEPROMs, and other I2C peripherals with multi-bus support.

## Overview

The I2C Bus Node provides low-level I2C communication for interfacing with sensors (IMU, magnetometer, pressure), displays (OLED, LCD), EEPROMs, power monitors, and other I2C peripherals. It supports multiple I2C buses simultaneously with configurable clock speeds and automatic hardware/simulation fallback.

Supports standard I2C protocol (100kHz), fast mode (400kHz), and fast mode plus (1MHz) on Linux systems via i2cdev.

Key features:
- Multi-bus support (I2C-0, I2C-1, I2C-2, etc.)
- 7-bit device addressing (0x08-0x77)
- Four transaction types: READ, WRITE, READ_REGISTER, WRITE_REGISTER
- Configurable clock speed (100kHz to 1MHz)
- Automatic retry on failure
- Device labeling for debugging
- Bus scanning for device discovery
- Simulation fallback when hardware unavailable
- Hardware I2C control via i2cdev Linux interface

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `i2c.request` | `I2cMessage` | I2C transaction requests from client nodes |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `i2c.response` | `I2cMessage` | I2C transaction responses with data and status |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `bus_number` | `u8` | `1` | I2C bus number (0, 1, 2, etc.) |
| `clock_speed` | `u32` | `400000` | Clock frequency in Hz (100kHz, 400kHz, 1MHz) |
| `retry_count` | `u8` | `3` | Number of retries on transaction failure |
| `timeout_ms` | `u64` | `100` | Transaction timeout in milliseconds |

### I2C Bus Selection

On Raspberry Pi and similar Linux systems:
- `/dev/i2c-0` - Usually reserved for hardware (HAT EEPROM)
- `/dev/i2c-1` - Main user I2C bus (GPIO 2/3)
- `/dev/i2c-2` - Secondary bus (if configured)

```rust
i2c.set_bus_number(1);  // Use /dev/i2c-1
```

## Message Types

### I2cMessage

I2C transaction message for read/write operations:

```rust
pub struct I2cMessage {
    pub device_address: u16,      // 7-bit I2C address (0x00-0x7F)
    pub register_address: u8,     // Register to read/write
    pub transaction_type: u8,     // READ, WRITE, READ_REG, WRITE_REG
    pub data: [u8; 256],         // Data buffer (max 256 bytes)
    pub data_length: u8,         // Number of bytes to read/write
    pub bus_number: u8,          // I2C bus (0, 1, 2, etc.)
    pub clock_speed: u32,        // Clock frequency in Hz
    pub success: bool,           // Transaction result
    pub error_code: u8,          // Error code if failed
    pub timestamp: u64,          // Transaction time (ns since epoch)
}
```

**Transaction Types**:
- `I2cMessage::TYPE_READ = 0` - Simple read from device
- `I2cMessage::TYPE_WRITE = 1` - Simple write to device
- `I2cMessage::TYPE_READ_REGISTER = 2` - Write register address, then read
- `I2cMessage::TYPE_WRITE_REGISTER = 3` - Write register address and data

**Clock Speed Constants**:
- `I2cMessage::SPEED_STANDARD = 100000` - 100 kHz (standard mode)
- `I2cMessage::SPEED_FAST = 400000` - 400 kHz (fast mode)
- `I2cMessage::SPEED_FAST_PLUS = 1000000` - 1 MHz (fast mode plus)

**Error Codes**:
- `0` - Success
- `1` - Out of bounds
- `2` - Unknown transaction type
- `3` - Device not found
- Hardware errors - NACK or permission denied

## Public API

### Construction

```rust
use horus_library::nodes::I2cBusNode;

// Create with default configuration (bus 1, standard topics)
let mut i2c = I2cBusNode::new()?;

// Create with custom bus number and topics
let mut i2c = I2cBusNode::new_with_config(
    1,                  // Bus number
    "i2c.request",      // Request topic
    "i2c.response"      // Response topic
)?;
```

### Configuration Methods

```rust
// Set I2C bus number (0, 1, 2, etc.)
i2c.set_bus_number(1);

// Set clock speed in Hz
i2c.set_clock_speed(100_000);   // 100 kHz standard mode
i2c.set_clock_speed(400_000);   // 400 kHz fast mode
i2c.set_clock_speed(1_000_000); // 1 MHz fast mode plus

// Set retry count for failed transactions
i2c.set_retry_count(3);

// Set transaction timeout in milliseconds
i2c.set_timeout(100);

// Register known devices with labels (for debugging)
i2c.register_device(0x68, "MPU6050 IMU");
i2c.register_device(0x40, "INA219 Power Monitor");
i2c.register_device(0x3C, "SSD1306 OLED");
```

### Query Methods

```rust
// Get transaction statistics
let (total, successful, failed, last_error) = i2c.get_stats();
println!("Total: {}, Success: {}, Failed: {}", total, successful, failed);
println!("Last error code: {}", last_error);
```

### Testing Methods

```rust
// Add simulated I2C device for testing without hardware
i2c.add_simulated_device(0x68, 256);  // 256-byte memory simulation

// Scan I2C bus for connected devices
let devices = i2c.scan_bus(None);
println!("Found {} devices: {:?}", devices.len(), devices);
```

## Usage Examples

### Basic I2C Communication

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create I2C bus node
    let mut i2c = I2cBusNode::new()?;
    i2c.set_bus_number(1);
    i2c.set_clock_speed(400_000);  // 400 kHz
    i2c.register_device(0x68, "MPU6050");

    scheduler.add(Box::new(i2c), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Reading from I2C Device

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Add I2C bus node
    let mut i2c = I2cBusNode::new()?;
    i2c.set_bus_number(1);
    scheduler.add(Box::new(i2c), 1, Some(true));

    // Node to read from MPU6050 accelerometer
    let reader = node! {
        name: "mpu6050_reader",
        tick: |ctx| {
            // Create I2C read request (register 0x3B, 6 bytes)
            let request = I2cMessage::read_register(
                1,      // Bus number
                0x68,   // MPU6050 address
                0x3B,   // ACCEL_XOUT_H register
                6       // Read 6 bytes (X, Y, Z acceleration)
            );

            // Send request
            let req_hub = Hub::<I2cMessage>::new("i2c.request")?;
            req_hub.send(request, &mut None)?;

            // Wait for response
            let resp_hub = Hub::<I2cMessage>::new("i2c.response")?;
            if let Some(response) = resp_hub.recv(None) {
                if response.success {
                    let data = response.get_data();
                    let ax = i16::from_be_bytes([data[0], data[1]]);
                    let ay = i16::from_be_bytes([data[2], data[3]]);
                    let az = i16::from_be_bytes([data[4], data[5]]);
                    ctx.log_info(&format!("Accel: x={}, y={}, z={}", ax, ay, az));
                } else {
                    ctx.log_error(&format!("I2C error: {}", response.error_code));
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(reader), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Writing to I2C Device

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut i2c = I2cBusNode::new()?;
    scheduler.add(Box::new(i2c), 1, Some(true));

    // Node to configure MPU6050
    let writer = node! {
        name: "mpu6050_config",
        tick: |ctx| {
            // Wake up MPU6050 by writing 0x00 to PWR_MGMT_1 (register 0x6B)
            let mut request = I2cMessage::write_register(1, 0x68, 0x6B, &[0x00]);

            let req_hub = Hub::<I2cMessage>::new("i2c.request")?;
            req_hub.send(request, &mut None)?;

            let resp_hub = Hub::<I2cMessage>::new("i2c.response")?;
            if let Some(response) = resp_hub.recv(None) {
                if response.success {
                    ctx.log_info("MPU6050 configured successfully");
                } else {
                    ctx.log_error(&format!("Config failed: {}", response.error_code));
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(writer), 2, Some(false));  // Run once
    scheduler.run()?;
    Ok(())
}
```

### Multiple I2C Devices

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create I2C bus node with device labels
    let mut i2c = I2cBusNode::new()?;
    i2c.set_bus_number(1);
    i2c.set_clock_speed(400_000);
    i2c.register_device(0x68, "MPU6050 IMU");
    i2c.register_device(0x40, "INA219 Power Monitor");
    i2c.register_device(0x3C, "SSD1306 Display");

    scheduler.add(Box::new(i2c), 1, Some(true));

    // Node to read from multiple devices
    let multi_reader = node! {
        name: "multi_device_reader",
        tick: |ctx| {
            let req_hub = Hub::<I2cMessage>::new("i2c.request")?;
            let resp_hub = Hub::<I2cMessage>::new("i2c.response")?;

            // Read IMU
            let imu_req = I2cMessage::read_register(1, 0x68, 0x3B, 6);
            req_hub.send(imu_req, &mut None)?;
            if let Some(resp) = resp_hub.recv(None) {
                if resp.success {
                    ctx.log_info("IMU data received");
                }
            }

            // Read power monitor
            let power_req = I2cMessage::read_register(1, 0x40, 0x02, 2);
            req_hub.send(power_req, &mut None)?;
            if let Some(resp) = resp_hub.recv(None) {
                if resp.success {
                    let voltage_raw = u16::from_be_bytes([resp.data[0], resp.data[1]]);
                    let voltage = (voltage_raw >> 3) as f32 * 0.004; // 4mV per LSB
                    ctx.log_info(&format!("Bus voltage: {:.3} V", voltage));
                }
            }

            Ok(())
        }
    };

    scheduler.add(Box::new(multi_reader), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Bus Scanning

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut i2c = I2cBusNode::new()?;
    i2c.set_bus_number(1);

    // Scan for devices before starting
    let devices = i2c.scan_bus(None);
    println!("Found {} I2C devices:", devices.len());
    for addr in devices {
        println!("  0x{:02X}", addr);
    }

    scheduler.add(Box::new(i2c), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Using Helper Functions

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut i2c = I2cBusNode::new()?;
    scheduler.add(Box::new(i2c), 1, Some(true));

    let helper = node! {
        name: "i2c_helper",
        tick: |ctx| {
            let req_hub = Hub::<I2cMessage>::new("i2c.request")?;
            let resp_hub = Hub::<I2cMessage>::new("i2c.response")?;

            // Simple read (no register address)
            let read_msg = I2cMessage::read(1, 0x68, 6);
            req_hub.send(read_msg, &mut None)?;

            // Simple write (no register address)
            let write_data = [0x6B, 0x00];
            let write_msg = I2cMessage::write(1, 0x68, &write_data);
            req_hub.send(write_msg, &mut None)?;

            // Register read (specify register first)
            let reg_read = I2cMessage::read_register(1, 0x68, 0x75, 1); // WHO_AM_I
            req_hub.send(reg_read, &mut None)?;

            // Register write (register + data)
            let reg_write = I2cMessage::write_register(1, 0x68, 0x6B, &[0x00]);
            req_hub.send(reg_write, &mut None)?;

            Ok(())
        }
    };

    scheduler.add(Box::new(helper), 2, Some(false));
    scheduler.run()?;
    Ok(())
}
```

## Hardware Setup

### Wiring Diagram

```
Raspberry Pi          I2C Device
GPIO 2 (SDA) <------> SDA
GPIO 3 (SCL) <------> SCL
3.3V         -------> VCC
GND          -------> GND
```

### Pull-up Resistors

I2C requires pull-up resistors on SDA and SCL lines (typically 4.7kOhm to 10kOhm):

```
        3.3V                    3.3V
         |                       |
        [4.7k]                 [4.7k]
         |                       |
    SDA |----[Device 1]----[Device 2]
         |
    SCL |----[Device 1]----[Device 2]
```

Most Raspberry Pi boards have built-in pull-ups, but you may need external resistors for:
- Long cables (>30cm)
- Multiple devices (>3)
- Fast mode plus (1MHz)

### System Requirements

```bash
# Install I2C tools
sudo apt install i2c-tools libi2c-dev

# Enable I2C interface
sudo raspi-config
# Select: Interface Options -> I2C -> Enable

# Add user to i2c group
sudo usermod -a -G i2c $USER

# Reboot or re-login for group changes to take effect
sudo reboot
```

### Verify Hardware

```bash
# List available I2C buses
ls /dev/i2c-*

# Scan for devices on bus 1
i2cdetect -y 1

# Read from device at 0x68, register 0x75 (WHO_AM_I)
i2cget -y 1 0x68 0x75

# Write 0x00 to device 0x68, register 0x6B
i2cset -y 1 0x68 0x6B 0x00

# Check bus permissions
ls -l /dev/i2c-1
```

Expected output for `i2cdetect -y 1`:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- 3c -- -- --
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```
This shows devices at addresses 0x3C, 0x40, and 0x68.

### Enable in Project

```toml
[dependencies]
horus_library = { version = "0.1", features = ["i2c-hardware"] }
```

```bash
cargo build --features="i2c-hardware"
```

## Common I2C Devices

### MPU6050 IMU (0x68 or 0x69)

6-axis accelerometer + gyroscope:

```rust
// Wake up device (disable sleep mode)
let wake_cmd = I2cMessage::write_register(1, 0x68, 0x6B, &[0x00]);

// Read WHO_AM_I register (should return 0x68)
let who_am_i = I2cMessage::read_register(1, 0x68, 0x75, 1);

// Read accelerometer data (6 bytes from register 0x3B)
let accel_data = I2cMessage::read_register(1, 0x68, 0x3B, 6);

// Read gyroscope data (6 bytes from register 0x43)
let gyro_data = I2cMessage::read_register(1, 0x68, 0x43, 6);

// Read temperature (2 bytes from register 0x41)
let temp_data = I2cMessage::read_register(1, 0x68, 0x41, 2);
```

### INA219 Power Monitor (0x40-0x45)

Current/voltage/power monitor:

```rust
// Read bus voltage (register 0x02)
let voltage_req = I2cMessage::read_register(1, 0x40, 0x02, 2);
// Voltage = (raw_value >> 3) * 4mV

// Read shunt voltage (register 0x01)
let shunt_req = I2cMessage::read_register(1, 0x40, 0x01, 2);
// Voltage = raw_value * 10uV

// Read current (register 0x04)
let current_req = I2cMessage::read_register(1, 0x40, 0x04, 2);

// Read power (register 0x03)
let power_req = I2cMessage::read_register(1, 0x40, 0x03, 2);

// Configure (register 0x00)
let config_cmd = I2cMessage::write_register(1, 0x40, 0x00, &[0x39, 0x9F]);
```

### SSD1306 OLED Display (0x3C or 0x3D)

128x64 monochrome OLED:

```rust
// Send command (control byte 0x00)
let mut cmd_msg = I2cMessage::write(1, 0x3C, &[0x00, 0xAF]); // Display ON

// Send data (control byte 0x40)
let mut data_msg = I2cMessage::write(1, 0x3C, &[0x40, 0xFF, 0x00, 0xFF]);

// Initialize sequence
let init_cmds = [
    0x00, 0xAE, // Display OFF
    0x00, 0xD5, // Set clock divide
    0x00, 0x80,
    0x00, 0xA8, // Set multiplex ratio
    0x00, 0x3F,
    0x00, 0xAF, // Display ON
];
let init_msg = I2cMessage::write(1, 0x3C, &init_cmds);
```

### BMP280 Pressure Sensor (0x76 or 0x77)

Temperature and barometric pressure:

```rust
// Read chip ID (register 0xD0, should return 0x58)
let id_req = I2cMessage::read_register(1, 0x76, 0xD0, 1);

// Read calibration data (registers 0x88-0xA1)
let calib_req = I2cMessage::read_register(1, 0x76, 0x88, 26);

// Configure (register 0xF4: oversampling and mode)
let config_cmd = I2cMessage::write_register(1, 0x76, 0xF4, &[0x2F]);

// Read pressure (registers 0xF7-0xF9)
let pressure_req = I2cMessage::read_register(1, 0x76, 0xF7, 3);

// Read temperature (registers 0xFA-0xFC)
let temp_req = I2cMessage::read_register(1, 0x76, 0xFA, 3);
```

### BNO055 9-DOF IMU (0x28 or 0x29)

Absolute orientation sensor with sensor fusion:

```rust
// Read chip ID (register 0x00, should return 0xA0)
let id_req = I2cMessage::read_register(1, 0x28, 0x00, 1);

// Set operation mode (register 0x3D)
let mode_cmd = I2cMessage::write_register(1, 0x28, 0x3D, &[0x0C]); // NDOF mode

// Read Euler angles (6 bytes from register 0x1A)
let euler_req = I2cMessage::read_register(1, 0x28, 0x1A, 6);

// Read quaternion (8 bytes from register 0x20)
let quat_req = I2cMessage::read_register(1, 0x28, 0x20, 8);

// Read linear acceleration (6 bytes from register 0x28)
let lin_accel_req = I2cMessage::read_register(1, 0x28, 0x28, 6);
```

### AT24C256 EEPROM (0x50-0x57)

256Kbit I2C EEPROM:

```rust
// Write data to address 0x0100
let addr_high = 0x01;
let addr_low = 0x00;
let write_data = [addr_high, addr_low, 0x42, 0x43, 0x44]; // Address + data
let write_cmd = I2cMessage::write(1, 0x50, &write_data);

// Read data from address 0x0100
// First, write address
let addr_write = I2cMessage::write(1, 0x50, &[addr_high, addr_low]);
// Then, read data
let read_cmd = I2cMessage::read(1, 0x50, 3);
```

### PCA9685 PWM Driver (0x40-0x7F)

16-channel 12-bit PWM driver:

```rust
// Configure mode (register 0x00)
let mode_cmd = I2cMessage::write_register(1, 0x40, 0x00, &[0x20]);

// Set PWM frequency (register 0xFE)
let freq_cmd = I2cMessage::write_register(1, 0x40, 0xFE, &[0x79]); // 50Hz

// Set channel 0 PWM (registers 0x06-0x09)
let pwm_data = [0x00, 0x00, 0x00, 0x10]; // ON=0, OFF=4096
let pwm_cmd = I2cMessage::write_register(1, 0x40, 0x06, &pwm_data);

// Set all channels (register 0xFA)
let all_off_cmd = I2cMessage::write_register(1, 0x40, 0xFA, &[0x00, 0x00, 0x00, 0x00]);
```

## Best Practices

1. **Always check transaction success**:
   ```rust
   if response.success {
       // Process data
   } else {
       ctx.log_error(&format!("I2C error: {}", response.error_code));
   }
   ```

2. **Use device labels for debugging**:
   ```rust
   i2c.register_device(0x68, "MPU6050 IMU");
   i2c.register_device(0x40, "Power Monitor");
   // Logs: "I2C transaction to device MPU6050 IMU (0x68)"
   ```

3. **Match clock speed to device capability**:
   ```rust
   // Most devices support fast mode (400kHz)
   i2c.set_clock_speed(400_000);

   // Some older devices require standard mode (100kHz)
   i2c.set_clock_speed(100_000);
   ```

4. **Enable retries for unreliable connections**:
   ```rust
   i2c.set_retry_count(5);  // Retry up to 5 times
   ```

5. **Scan bus during development**:
   ```rust
   let devices = i2c.scan_bus(None);
   println!("Found devices: {:?}", devices);
   ```

6. **Use simulated devices for testing**:
   ```rust
   i2c.add_simulated_device(0x68, 256);  // 256-byte memory
   // Test code without hardware
   ```

7. **Verify device address in datasheet**:
   - Some devices have configurable addresses (via address pins)
   - MPU6050: 0x68 (AD0=LOW) or 0x69 (AD0=HIGH)
   - BMP280: 0x76 (SDO=LOW) or 0x77 (SDO=HIGH)
   - Check with `i2cdetect -y 1`

8. **Use appropriate transaction type**:
   ```rust
   // For devices with register addressing (most sensors):
   I2cMessage::read_register(1, 0x68, 0x3B, 6);
   I2cMessage::write_register(1, 0x68, 0x6B, &[0x00]);

   // For devices without register addressing:
   I2cMessage::read(1, 0x50, 32);
   I2cMessage::write(1, 0x50, &data);
   ```

9. **Add delays for device initialization**:
   ```rust
   // Some devices need time after power-on or reset
   std::thread::sleep(std::time::Duration::from_millis(100));
   ```

10. **Check for address conflicts**:
    ```rust
    // Many devices share common addresses
    // INA219: 0x40-0x45
    // PCA9685: 0x40-0x7F
    // Use configurable address pins to avoid conflicts
    ```

## Troubleshooting

### "Hardware unavailable - using SIMULATION mode"

```
[WARN] I2cBusNode: Hardware unavailable - using SIMULATION mode
[WARN]   Tried: /dev/i2c-1
[WARN]   Error: Permission denied (os error 13)
[WARN]   Fix:
[WARN]     1. Install: sudo apt install i2c-tools
[WARN]     2. Enable I2C: sudo raspi-config -> Interface Options -> I2C
[WARN]     3. Add user to group: sudo usermod -a -G i2c $USER
[WARN]     4. Reboot or re-login
[WARN]     5. Rebuild with: cargo build --features="i2c-hardware"
```

**Solutions:**
1. Check if I2C device exists: `ls /dev/i2c-*`
2. Check permissions: `ls -l /dev/i2c-1`
3. Install I2C tools: `sudo apt install i2c-tools`
4. Enable I2C: `sudo raspi-config` -> Interface Options -> I2C
5. Add user to i2c group: `sudo usermod -a -G i2c $USER`
6. Reboot: `sudo reboot`
7. Rebuild with hardware support: `cargo build --features="i2c-hardware"`

### Device not responding (error code 3)

**Solutions:**
1. Verify device address with `i2cdetect -y 1`
2. Check wiring (SDA, SCL, VCC, GND)
3. Check pull-up resistors (4.7k-10k to 3.3V)
4. Verify device is powered (check voltage with multimeter)
5. Check device datasheet for correct address
6. Try lower clock speed: `i2c.set_clock_speed(100_000)`

### Transaction fails intermittently

**Solutions:**
1. Add pull-up resistors (4.7kOhm recommended)
2. Reduce clock speed to 100kHz
3. Shorten cables (keep under 30cm)
4. Check for loose connections
5. Add decoupling capacitors near device VCC (0.1uF + 10uF)
6. Increase retry count: `i2c.set_retry_count(5)`
7. Increase timeout: `i2c.set_timeout(200)`

### Wrong data received

**Solutions:**
1. Verify byte order (big-endian vs little-endian)
2. Check register address in datasheet
3. Ensure device is initialized properly
4. Read multiple times to verify consistency
5. Check for address conflicts with other devices

### Bus hangs or freezes

**Solutions:**
1. Power cycle the device
2. Reset I2C bus: `sudo i2cdetect -y 1`
3. Check for short circuit on SDA or SCL
4. Verify pull-up resistors are correct value
5. Reduce bus capacitance (shorter wires, fewer devices)
6. Add bus reset logic in your code

### Multiple devices on same bus interfere

**Solutions:**
1. Use devices with configurable addresses
2. Add individual device reset lines
3. Increase pull-up resistor strength (lower value: 2.2k-4.7k)
4. Reduce clock speed to 100kHz
5. Add bus buffer (PCA9517) for long cables or many devices

### Permission denied on /dev/i2c-X

**Solutions:**
```bash
# Check current permissions
ls -l /dev/i2c-1

# Add user to i2c group
sudo usermod -a -G i2c $USER

# Alternative: Change device permissions (not recommended)
sudo chmod 666 /dev/i2c-1

# Verify group membership
groups $USER

# Reboot or re-login
sudo reboot
```

## Simulation Mode

When hardware is unavailable, the node operates in simulation mode:

```
[WARN] I2cBusNode: Hardware unavailable - using SIMULATION mode
[INFO] I2C transaction to device MPU6050 (0x68)
```

Simulated behavior:
- Supports all transaction types
- Returns simulated data from virtual device memory
- Requires explicit device registration via `add_simulated_device()`
- Returns error code 3 (device not found) for unknown devices
- Useful for logic testing without hardware
- Timing characteristics not simulated

Example simulation setup:
```rust
let mut i2c = I2cBusNode::new()?;
i2c.add_simulated_device(0x68, 256);  // MPU6050 with 256 bytes
i2c.add_simulated_device(0x40, 128);  // INA219 with 128 bytes
// Now you can test I2C transactions without hardware
```

## Performance Considerations

### Clock Speed vs Reliability

| Speed | Mode | Use Case | Cable Length |
|-------|------|----------|--------------|
| 100 kHz | Standard | Long cables, multiple devices | <1m |
| 400 kHz | Fast | Most applications (recommended) | <30cm |
| 1 MHz | Fast Plus | Short connections, few devices | <10cm |

### Transaction Overhead

- Simple read/write: ~1ms @ 400kHz
- Register read/write: ~1.5ms @ 400kHz
- Bus scan (112 addresses): ~200-300ms
- Retry on failure: +10-100ms per retry

### Multi-Device Timing

```rust
// Sequential access to 3 devices @ 400kHz
// Time per transaction: ~1.5ms
// Total time: ~4.5ms (660 Hz maximum rate)

// For high-frequency polling, use dedicated node per device
// or implement round-robin scheduling
```

## See Also

- [SpiBusNode](../spi_bus/) - SPI communication for high-speed peripherals
- [CanBusNode](../can_bus/) - CAN bus for automotive/industrial
- [SerialNode](../serial/) - UART/Serial communication
- [ImuNode](../imu/) - Uses I2C for MPU6050, BNO055
- [BatteryMonitorNode](../battery_monitor/) - Uses I2C for INA219
