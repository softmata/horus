# Serial Communication Node

UART/RS232/RS485 serial communication for sensors, GPS modules, motor controllers, and serial peripherals with configurable baud rates and data formats.

## Quick Start

```rust
use horus_library::nodes::{SerialNode, SerialBackend};
use horus_library::SerialData;
use horus_core::{Scheduler, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut scheduler = Scheduler::new();

    // Create serial node
    let serial = SerialNode::new_with_backend(
        "/dev/ttyUSB0",  // Serial port
        9600,            // Baud rate
        "serial.rx",     // Receive topic
        "serial.tx",     // Transmit topic
        SerialBackend::Hardware,
    )?;

    scheduler.add(Box::new(serial), 50, Some(true));
    scheduler.run()?;
    Ok(())
}

// Send data from another node:
let tx_hub = Hub::<SerialData>::new("serial.tx")?;
tx_hub.send(SerialData::from_bytes(b"Hello Arduino!"), None);

// Receive data in your node:
let rx_hub = Hub::<SerialData>::new("serial.rx")?;
if let Some(data) = rx_hub.recv_latest() {
    println!("Received: {:?}", data.as_bytes());
}
```

**Subscribes to:** `serial.tx` (data to send)
**Publishes to:** `serial.rx` (received data)

## Overview

The Serial Node provides UART communication with serial devices via standard serial ports. It supports various baud rates, data formats, and flow control options for interfacing with GPS modules, Arduino boards, telemetry radios, Modbus devices, and other serial peripherals.

Supports RS232, RS485, TTL serial, and USB-to-Serial adapters.

Key features:
- Multiple serial port support (ttyUSB*, ttyS*, ttyAMA*)
- Configurable baud rates (9600 to 921600 baud)
- Data format configuration (data bits, parity, stop bits)
- Hardware and software flow control
- Binary and ASCII data modes
- Configurable read/write timeouts
- Simulation fallback when hardware unavailable
- Hardware serial control via serialport-rs

## Topics

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `serial/rx` | `SerialData` | Data received from serial port |

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `serial/tx` | `SerialData` | Data to transmit on serial port |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port_path` | `String` | `/dev/ttyUSB0` | Serial port device path |
| `baud_rate` | `u32` | `9600` | Communication speed in bits per second |
| `data_bits` | `u8` | `8` | Number of data bits (5, 6, 7, or 8) |
| `stop_bits` | `u8` | `1` | Number of stop bits (1 or 2) |
| `parity` | `u8` | `0` (None) | Parity checking (0=None, 1=Odd, 2=Even) |
| `flow_control` | `bool` | `false` | Hardware flow control (RTS/CTS) enable |
| `read_timeout_ms` | `u64` | `100` | Read timeout in milliseconds |

### Common Baud Rates

- 9600: GPS modules, Arduino (default)
- 19200: Modbus RTU
- 38400: Industrial sensors
- 57600: Firmata protocol, fast sensors
- 115200: High-speed communication
- 230400: Very high-speed data
- 460800: Maximum speed (some adapters)
- 921600: Maximum speed (high-quality adapters)

### Parity Options

- `SerialData::PARITY_NONE` (0): No parity checking (most common)
- `SerialData::PARITY_ODD` (1): Odd parity
- `SerialData::PARITY_EVEN` (2): Even parity

## Message Types

### SerialData

Serial communication message:

```rust
pub struct SerialData {
    pub port_id: [u8; 64],      // Serial port identifier
    pub data: [u8; 1024],       // Raw data bytes (max 1024 per message)
    pub data_length: u16,       // Number of valid bytes in data array
    pub baud_rate: u32,         // Baud rate
    pub data_bits: u8,          // Data bits (5, 6, 7, or 8)
    pub stop_bits: u8,          // Stop bits (1 or 2)
    pub parity: u8,             // Parity (0=None, 1=Odd, 2=Even)
    pub timestamp: u64,         // Message time (ns since epoch)
}
```

**Constants**:
- `SerialData::PARITY_NONE = 0`
- `SerialData::PARITY_ODD = 1`
- `SerialData::PARITY_EVEN = 2`

**Helper Methods**:
```rust
// Create new message
SerialData::new(port: &str) -> Self

// Port operations
set_port(port: &str)
get_port() -> String

// Data operations (binary)
set_data(data: &[u8]) -> bool
get_data() -> &[u8]

// Data operations (string)
set_string(text: &str) -> bool
get_string() -> Option<String>
```

## Public API

### Construction

```rust
use horus_library::nodes::SerialNode;

// Create with default configuration (/dev/ttyUSB0 @ 9600 baud)
let mut serial = SerialNode::new()?;

// Create with custom configuration
let mut serial = SerialNode::new_with_config(
    "/dev/ttyUSB0",  // port
    115200,          // baud_rate
    "serial.rx",     // rx_topic
    "serial.tx"      // tx_topic
)?;
```

### Configuration Methods

```rust
// Set serial port path
serial.set_port("/dev/ttyUSB0");

// Set baud rate (9600-921600)
serial.set_baud_rate(115200);

// Set data format (data_bits, stop_bits, parity)
serial.set_format(8, 1, SerialData::PARITY_NONE);  // 8N1 (most common)
serial.set_format(7, 1, SerialData::PARITY_EVEN);  // 7E1
serial.set_format(8, 2, SerialData::PARITY_EVEN);  // 8E2

// Enable/disable hardware flow control
serial.set_flow_control(true);   // Enable RTS/CTS
serial.set_flow_control(false);  // Disable flow control

// Set read timeout in milliseconds
serial.set_read_timeout(1000);  // 1 second timeout
```

### Query Methods

```rust
// Check if port is open
if serial.is_open() {
    println!("Port is open");
}

// Get statistics (bytes_rx, bytes_tx, errors)
let (rx_bytes, tx_bytes, errors) = serial.get_stats();
println!("RX: {} bytes, TX: {} bytes, Errors: {}", rx_bytes, tx_bytes, errors);
```

### Simulation Methods

```rust
// Simulate receiving data (for testing without hardware)
serial.simulate_receive(b"Hello from device\n");
```

## Usage Examples

### Basic Serial Communication

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create serial node
    let mut serial = SerialNode::new()?;
    serial.set_port("/dev/ttyUSB0");
    serial.set_baud_rate(9600);
    serial.set_format(8, 1, SerialData::PARITY_NONE);  // 8N1

    scheduler.add(Box::new(serial), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### GPS Module Communication

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure GPS serial port (typical: 9600 baud, 8N1)
    let mut serial = SerialNode::new_with_config(
        "/dev/ttyUSB0",
        9600,
        "gps/serial/rx",
        "gps/serial/tx"
    )?;
    serial.set_format(8, 1, SerialData::PARITY_NONE);

    scheduler.add(Box::new(serial), 1, Some(true));

    // GPS parser node
    let gps_parser = node! {
        name: "gps_parser",
        tick: |ctx| {
            let hub = Hub::<SerialData>::new("gps/serial/rx")?;

            while let Some(data) = hub.recv(None) {
                if let Some(nmea) = data.get_string() {
                    // Parse NMEA sentences
                    for line in nmea.lines() {
                        if line.starts_with("$GPGGA") {
                            ctx.log_info(&format!("GPS Fix: {}", line));
                        } else if line.starts_with("$GPRMC") {
                            ctx.log_info(&format!("GPS RMC: {}", line));
                        }
                    }
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(gps_parser), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Arduino Communication (Firmata)

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Arduino typically uses 57600 baud for Firmata
    let mut serial = SerialNode::new_with_config(
        "/dev/ttyUSB0",
        57600,
        "arduino.rx",
        "arduino.tx"
    )?;

    scheduler.add(Box::new(serial), 1, Some(true));

    // Arduino controller node
    let controller = node! {
        name: "arduino_controller",
        tick: |ctx| {
            let tx_hub = Hub::<SerialData>::new("arduino.tx")?;
            let rx_hub = Hub::<SerialData>::new("arduino.rx")?;

            // Send digital write command (Firmata protocol)
            let mut msg = SerialData::new("/dev/ttyUSB0");
            msg.set_data(&[0xF5, 0x01, 0x00])?;  // Digital write pin 1 LOW
            tx_hub.send(msg, &mut None)?;

            // Process responses
            while let Some(response) = rx_hub.recv(None) {
                ctx.log_debug(&format!("Arduino response: {:02X?}", response.get_data()));
            }

            Ok(())
        }
    };

    scheduler.add(Box::new(controller), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Modbus RTU Communication

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Modbus RTU typically uses 19200 baud, 8E1 format
    let mut serial = SerialNode::new_with_config(
        "/dev/ttyUSB0",
        19200,
        "modbus.rx",
        "modbus.tx"
    )?;
    serial.set_format(8, 1, SerialData::PARITY_EVEN);  // 8E1

    scheduler.add(Box::new(serial), 1, Some(true));

    // Modbus master node
    let modbus = node! {
        name: "modbus_master",
        tick: |ctx| {
            let tx_hub = Hub::<SerialData>::new("modbus.tx")?;
            let rx_hub = Hub::<SerialData>::new("modbus.rx")?;

            // Read holding registers (function code 0x03)
            let mut msg = SerialData::new("/dev/ttyUSB0");
            let modbus_request = vec![
                0x01,       // Slave address
                0x03,       // Function code: Read holding registers
                0x00, 0x00, // Start address
                0x00, 0x0A, // Number of registers
                0xC5, 0xCD  // CRC16
            ];
            msg.set_data(&modbus_request)?;
            tx_hub.send(msg, &mut None)?;

            // Process response
            if let Some(response) = rx_hub.recv(None) {
                let data = response.get_data();
                if data.len() >= 5 {
                    ctx.log_info(&format!("Modbus response: {:02X?}", data));
                }
            }

            Ok(())
        }
    };

    scheduler.add(Box::new(modbus), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Telemetry Radio Communication

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // 3DR Radio / SiK radio uses 57600 baud
    let mut serial = SerialNode::new_with_config(
        "/dev/ttyUSB0",
        57600,
        "telemetry.rx",
        "telemetry.tx"
    )?;

    scheduler.add(Box::new(serial), 1, Some(true));

    // Telemetry node
    let telemetry = node! {
        name: "telemetry",
        tick: |ctx| {
            let tx_hub = Hub::<SerialData>::new("telemetry.tx")?;
            let rx_hub = Hub::<SerialData>::new("telemetry.rx")?;

            // Send MAVLink heartbeat
            let mut msg = SerialData::new("/dev/ttyUSB0");
            let heartbeat = vec![
                0xFE,       // STX (MAVLink 1.0)
                0x09,       // Payload length
                0x00,       // Sequence
                0x01,       // System ID
                0x00,       // Component ID
                0x00,       // Message ID (HEARTBEAT)
                // ... payload and checksum
            ];
            msg.set_data(&heartbeat)?;
            tx_hub.send(msg, &mut None)?;

            // Receive telemetry
            while let Some(data) = rx_hub.recv(None) {
                ctx.log_debug(&format!("Telemetry: {} bytes", data.data_length));
            }

            Ok(())
        }
    };

    scheduler.add(Box::new(telemetry), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Bidirectional Communication

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut serial = SerialNode::new()?;
    serial.set_baud_rate(115200);

    scheduler.add(Box::new(serial), 1, Some(true));

    // Communication node (both TX and RX)
    let comm = node! {
        name: "serial_comm",
        tick: |ctx| {
            let tx_hub = Hub::<SerialData>::new("serial.tx")?;
            let rx_hub = Hub::<SerialData>::new("serial.rx")?;

            // Send command
            let mut cmd = SerialData::new("/dev/ttyUSB0");
            cmd.set_string("AT+VERSION\r\n")?;
            tx_hub.send(cmd, &mut None)?;

            // Receive response
            while let Some(response) = rx_hub.recv(None) {
                if let Some(text) = response.get_string() {
                    ctx.log_info(&format!("Device response: {}", text.trim()));

                    // Echo back
                    if text.contains("OK") {
                        let mut reply = SerialData::new("/dev/ttyUSB0");
                        reply.set_string("AT+STATUS\r\n")?;
                        tx_hub.send(reply, &mut None)?;
                    }
                }
            }

            Ok(())
        }
    };

    scheduler.add(Box::new(comm), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Binary Protocol Communication

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut serial = SerialNode::new_with_config(
        "/dev/ttyUSB0",
        115200,
        "binary.rx",
        "binary.tx"
    )?;

    scheduler.add(Box::new(serial), 1, Some(true));

    // Binary protocol handler
    let binary = node! {
        name: "binary_protocol",
        tick: |ctx| {
            let tx_hub = Hub::<SerialData>::new("binary.tx")?;
            let rx_hub = Hub::<SerialData>::new("binary.rx")?;

            // Send binary packet (custom protocol)
            let mut msg = SerialData::new("/dev/ttyUSB0");
            let packet = vec![
                0xAA, 0x55,     // Sync bytes
                0x10,           // Packet ID
                0x04,           // Length
                0x12, 0x34, 0x56, 0x78, // Payload
                0x00            // Checksum
            ];
            msg.set_data(&packet)?;
            tx_hub.send(msg, &mut None)?;

            // Process binary responses
            while let Some(response) = rx_hub.recv(None) {
                let data = response.get_data();
                if data.len() >= 2 && data[0] == 0xAA && data[1] == 0x55 {
                    ctx.log_info(&format!("Valid packet received: {:02X?}", data));
                }
            }

            Ok(())
        }
    };

    scheduler.add(Box::new(binary), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

## Hardware Setup

### USB-to-Serial Adapters

Common adapters and their device paths:

```
FTDI FT232RL    -> /dev/ttyUSB0 (most common)
Prolific PL2303 -> /dev/ttyUSB0
CP2102/CP2104   -> /dev/ttyUSB0
CH340           -> /dev/ttyUSB0
```

### Raspberry Pi Hardware Serial

```
UART0: /dev/ttyAMA0 (GPIO 14/15)
UART1: /dev/ttyS0   (GPIO 14/15, mini UART)
```

### RS232 Wiring (DB9 Connector)

```
Computer (DB9)    Device
Pin 2 (RX)  <--   TX
Pin 3 (TX)  -->   RX
Pin 5 (GND) ---   GND
Pin 7 (RTS) -->   CTS (if flow control)
Pin 8 (CTS) <--   RTS (if flow control)
```

### RS485 Wiring (Half-Duplex)

```
RS485 Adapter     Device
A (Data+)   ---   A/D+
B (Data-)   ---   B/D-
GND         ---   GND
```

### TTL Serial Wiring (3.3V)

```
Raspberry Pi      Device
GPIO 14 (TX) -->  RX
GPIO 15 (RX) <--  TX
GND          ---  GND
```

### System Requirements

```bash
# Install serial tools
sudo apt install setserial minicom

# Add user to dialout group for serial port access
sudo usermod -a -G dialout $USER

# Verify serial ports
ls -l /dev/ttyUSB* /dev/ttyS* /dev/ttyAMA*

# Check port information
dmesg | grep tty
lsusb
```

### Enable in Project

```toml
[dependencies]
horus_library = { version = "0.1", features = ["serial-hardware"] }
```

```bash
cargo build --features="serial-hardware"
```

## Data Formats

### Standard Serial Formats

**8N1** (Most Common):
- 8 data bits
- No parity
- 1 stop bit
- Used by: Arduino, GPS, most sensors

**7E1**:
- 7 data bits
- Even parity
- 1 stop bit
- Used by: Some industrial protocols

**8E1** (Modbus RTU):
- 8 data bits
- Even parity
- 1 stop bit
- Used by: Modbus RTU

**8E2**:
- 8 data bits
- Even parity
- 2 stop bits
- Used by: Older industrial equipment

### Baud Rate Selection

- **9600**: Arduino default, GPS, low-speed sensors
- **19200**: Modbus RTU, industrial sensors
- **38400**: Medium-speed data
- **57600**: Firmata, 3DR radios
- **115200**: High-speed data, ESP32/ESP8266
- **230400**: Very high-speed (requires good cables)
- **460800**: Maximum for many adapters
- **921600**: Maximum for high-quality adapters

## Best Practices

1. **Always verify port permissions**:
   ```bash
   # Check current user groups
   groups

   # Should include "dialout" - if not, add user:
   sudo usermod -a -G dialout $USER
   # Then logout/login for changes to take effect
   ```

2. **Use correct data format for your device**:
   ```rust
   // Most devices use 8N1
   serial.set_format(8, 1, SerialData::PARITY_NONE);

   // Check device datasheet for specific format
   ```

3. **Set appropriate timeouts**:
   ```rust
   // Prevent blocking forever on slow devices
   serial.set_read_timeout(1000);  // 1 second
   ```

4. **Check baud rate matches device**:
   ```rust
   // Wrong baud rate = garbage data
   serial.set_baud_rate(9600);  // Must match device
   ```

5. **Handle binary and text data appropriately**:
   ```rust
   // For text protocols
   if let Some(text) = data.get_string() {
       println!("Text: {}", text);
   }

   // For binary protocols
   let bytes = data.get_data();
   println!("Binary: {:02X?}", bytes);
   ```

6. **Use flow control for high-speed transfers**:
   ```rust
   serial.set_flow_control(true);  // Enable RTS/CTS
   ```

7. **Verify cables and connections**:
   - Use quality cables (especially for high baud rates)
   - Check TX/RX are not swapped
   - Verify ground connection
   - Keep cable length reasonable (<6 feet for high speeds)

## Troubleshooting

### "Hardware unavailable - using SIMULATION mode"

```
[INFO] Opening serial port /dev/ttyUSB0 @ 9600 baud
[WARN] Hardware unavailable - using SIMULATION mode
```

**Solutions:**
1. Check port exists: `ls -l /dev/ttyUSB*`
2. Verify user permissions: `groups` (should include "dialout")
3. Add to dialout group: `sudo usermod -a -G dialout $USER`
4. Logout and login for group changes
5. Check if port is in use: `lsof /dev/ttyUSB0`
6. Rebuild with `--features="serial-hardware"`

### Receiving garbage characters

**Symptoms**: Random characters, squares, or gibberish

**Solutions:**
1. Verify baud rate matches device
2. Check data format (data bits, parity, stop bits)
3. Verify TX/RX are not swapped
4. Check ground connection
5. Try different cable
6. Reduce baud rate to test

### No data received

**Symptoms**: Port opens but no data arrives

**Solutions:**
1. Check TX/RX wiring (often swapped)
2. Verify device is powered on
3. Test with loopback (connect TX to RX)
4. Check device configuration
5. Verify baud rate and format
6. Try different USB port

### "Permission denied" error

```
Error: Permission denied (os error 13)
```

**Solutions:**
1. Add user to dialout: `sudo usermod -a -G dialout $USER`
2. Logout and login
3. Check port permissions: `ls -l /dev/ttyUSB0`
4. Try with sudo (temporarily for testing)
5. Check SELinux/AppArmor policies

### Port not found

```
Error: No such file or directory
```

**Solutions:**
1. Check USB connection
2. Verify port name: `ls -l /dev/ttyUSB*`
3. Check dmesg: `dmesg | grep tty`
4. Verify driver loaded: `lsmod | grep usb`
5. Try different USB port
6. Check adapter LED

### Data corruption at high baud rates

**Symptoms**: Errors increase with baud rate

**Solutions:**
1. Use shorter, higher-quality cable
2. Enable flow control
3. Reduce baud rate
4. Check cable shielding
5. Minimize cable movement
6. Add ferrite beads

### Intermittent connection

**Symptoms**: Connection works sometimes

**Solutions:**
1. Check cable connections (loose connector)
2. Verify power supply stability
3. Check for electromagnetic interference
4. Replace cable
5. Check USB hub (try direct connection)
6. Monitor system logs: `dmesg -w`

## Testing

### Loopback Test

```bash
# Connect TX to RX on adapter
# Send data and verify it echoes back
```

```rust
// Loopback test code
let mut serial = SerialNode::new()?;
serial.set_baud_rate(115200);

// Send test data
let mut msg = SerialData::new("/dev/ttyUSB0");
msg.set_string("LOOPBACK_TEST\n")?;
tx_hub.send(msg, &mut None)?;

// Should receive same data back
if let Some(response) = rx_hub.recv(None) {
    assert_eq!(response.get_string(), Some("LOOPBACK_TEST\n".to_string()));
}
```

### Command Line Tools

```bash
# Send data to serial port
echo "Hello" > /dev/ttyUSB0

# Receive data from serial port
cat /dev/ttyUSB0

# Interactive terminal
screen /dev/ttyUSB0 9600
# Detach: Ctrl-A, then K

# Alternative terminal
minicom -D /dev/ttyUSB0 -b 9600
```

### Port Information

```bash
# List USB devices
lsusb

# Check kernel messages
dmesg | grep tty

# Port details
udevadm info /dev/ttyUSB0

# Port speed/settings
stty -F /dev/ttyUSB0
```

## Simulation Mode

When hardware is unavailable, the node operates in simulation mode:

```
[INFO] Opening serial port /dev/ttyUSB0 @ 9600 baud
```

Simulated behavior:
- Port appears open but no actual hardware communication
- Transmit operations succeed but don't send data
- No receive data unless injected via `simulate_receive()`
- Useful for logic testing without hardware
- Statistics tracking works normally

To inject test data in simulation:
```rust
serial.simulate_receive(b"Test data\n");
```

## Common Device Configurations

### GPS Module

```rust
serial.set_baud_rate(9600);
serial.set_format(8, 1, SerialData::PARITY_NONE);  // 8N1
```

### Arduino (Standard)

```rust
serial.set_baud_rate(9600);
serial.set_format(8, 1, SerialData::PARITY_NONE);  // 8N1
```

### Arduino (Firmata)

```rust
serial.set_baud_rate(57600);
serial.set_format(8, 1, SerialData::PARITY_NONE);  // 8N1
```

### Modbus RTU

```rust
serial.set_baud_rate(19200);
serial.set_format(8, 1, SerialData::PARITY_EVEN);  // 8E1
```

### 3DR Telemetry Radio

```rust
serial.set_baud_rate(57600);
serial.set_format(8, 1, SerialData::PARITY_NONE);  // 8N1
```

### ESP32/ESP8266

```rust
serial.set_baud_rate(115200);
serial.set_format(8, 1, SerialData::PARITY_NONE);  // 8N1
```

### Industrial Sensor (Generic)

```rust
serial.set_baud_rate(9600);
serial.set_format(8, 1, SerialData::PARITY_NONE);  // 8N1
```

## See Also

- [ModbusNode](../modbus/) - Modbus RTU/TCP protocol implementation
- [CanBusNode](../canbus/) - CAN bus communication
- [I2cNode](../i2c/) - I2C bus communication
- [SpiNode](../spi/) - SPI bus communication
- [GpsNode](../gps/) - GPS-specific NMEA parsing
