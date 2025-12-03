# Modbus Node

Industrial protocol handler for Modbus TCP/RTU communication with industrial equipment.

## Overview

The Modbus Node implements the Modbus protocol for industrial automation and control systems. It supports both Modbus TCP and RTU variants, enabling communication with PLCs, sensors, actuators, and other industrial devices. The node handles reading/writing coils, discrete inputs, holding registers, and input registers according to the Modbus specification.

## Topics

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `modbus_request` | `ModbusMessage` | Incoming Modbus requests to execute |

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `modbus_response` | `ModbusMessage` | Modbus responses from device |
| `modbus_status` | `NetworkStatus` | Connection status and statistics |

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `server_address` | `String` | `"127.0.0.1"` | Modbus server IP address or serial port |
| `server_port` | `u16` | `502` | TCP port (standard Modbus TCP port is 502) |
| `slave_id` | `u8` | `1` | Modbus slave/unit identifier (1-247) |
| `timeout_ms` | `u64` | `5000` | Communication timeout in milliseconds |
| `baud_rate` | `u32` | `9600` | Serial baud rate for RTU (9600, 19200, 38400, etc.) |
| `parity` | `Parity` | `None` | Serial parity for RTU (None, Even, Odd) |
| `stop_bits` | `u8` | `1` | Serial stop bits for RTU (1 or 2) |
| `protocol` | `Protocol` | `TCP` | Protocol variant (TCP or RTU) |

## Message Types

### ModbusMessage

Main message structure for Modbus communication:

```rust
pub struct ModbusMessage {
    pub slave_id: u8,           // Modbus slave/unit ID (1-247)
    pub function_code: u8,      // Modbus function code (1-127)
    pub start_address: u16,     // Starting register/coil address
    pub quantity: u16,          // Number of registers/coils to read/write
    pub data: [u16; 125],       // Data payload (max 125 registers)
    pub data_length: u8,        // Actual data length
    pub is_request: bool,       // True for requests, false for responses
    pub timestamp: u64,         // Message timestamp in nanoseconds
}
```

### NetworkStatus

Network connection status and statistics:

```rust
pub struct NetworkStatus {
    pub interface_name: String, // "modbus"
    pub link_up: bool,          // Connection established
    pub tx_packets: u64,        // Transmitted packets
    pub rx_packets: u64,        // Received packets
    pub tx_errors: u64,         // Transmission errors
    pub rx_errors: u64,         // Reception errors
}
```

### Register Types

Modbus defines four data types:

1. **Coils (0x)**: Single-bit read-write outputs
2. **Discrete Inputs (1x)**: Single-bit read-only inputs
3. **Holding Registers (4x)**: 16-bit read-write registers
4. **Input Registers (3x)**: 16-bit read-only registers

## Public API

### Construction

```rust
use horus_library::nodes::ModbusNode;

// Create with default topics
let mut modbus = ModbusNode::new()?;

// Create with custom topics
let mut modbus = ModbusNode::new_with_topics(
    "device_request",    // request topic
    "device_response",   // response topic
    "device_status"      // status topic
)?;
```

### Configuration Methods

```rust
// Set Modbus server connection parameters
modbus.set_connection("192.168.1.100", 502, 1);

// Set communication timeout
modbus.set_timeout(3000);  // 3 seconds

// Check connection status
if modbus.is_connected() {
    eprintln!("Connected to Modbus device");
}

// Get connection statistics
let (attempts, last_activity, cache_size) = modbus.get_stats();
eprintln!("Connection attempts: {}", attempts);
eprintln!("Last activity: {} ns", last_activity);
eprintln!("Cached registers: {}", cache_size);
```

## Usage Examples

### Basic Modbus TCP Communication

```rust
use horus_library::nodes::ModbusNode;
use horus_library::ModbusMessage;
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create Modbus TCP node
    let mut modbus = ModbusNode::new()?;
    modbus.set_connection("192.168.1.10", 502, 1);
    modbus.set_timeout(5000);

    runtime.add_node(modbus);

    // Create request publisher
    let request_hub = Hub::new("modbus_request")?;

    // Read 10 holding registers starting at address 100
    let mut request = ModbusMessage::default();
    request.slave_id = 1;
    request.function_code = 3;  // Read Holding Registers
    request.start_address = 100;
    request.quantity = 10;
    request.is_request = true;

    request_hub.send(request, None)?;

    runtime.run()?;
    Ok(())
}
```

### Reading Temperature Sensors

```rust
use horus_library::nodes::ModbusNode;
use horus_library::ModbusMessage;
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create Modbus node for temperature controller
    let mut modbus = ModbusNode::new_with_topics(
        "temp_request",
        "temp_response",
        "temp_status"
    )?;
    modbus.set_connection("192.168.1.20", 502, 1);

    runtime.add_node(modbus);

    // Read temperature from input register 0
    let request_hub = Hub::new("temp_request")?;
    let response_hub = Hub::new("temp_response")?;

    let mut request = ModbusMessage::default();
    request.slave_id = 1;
    request.function_code = 4;  // Read Input Registers
    request.start_address = 0;
    request.quantity = 1;
    request.is_request = true;

    request_hub.send(request, None)?;

    // Receive response
    if let Some(response) = response_hub.recv(None) {
        let temp_raw = response.data[0];
        let temperature = temp_raw as f32 / 10.0;  // Typical scaling
        eprintln!("Temperature: {}°C", temperature);
    }

    Ok(())
}
```

### Controlling Actuators

```rust
use horus_library::nodes::ModbusNode;
use horus_library::ModbusMessage;
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create Modbus node for actuator control
    let mut modbus = ModbusNode::new_with_topics(
        "actuator_cmd",
        "actuator_response",
        "actuator_status"
    )?;
    modbus.set_connection("192.168.1.30", 502, 1);

    runtime.add_node(modbus);

    let command_hub = Hub::new("actuator_cmd")?;

    // Write to holding register to set valve position (0-100%)
    let mut write_request = ModbusMessage::default();
    write_request.slave_id = 1;
    write_request.function_code = 6;  // Write Single Register
    write_request.start_address = 200;
    write_request.data[0] = 75;  // 75% open
    write_request.data_length = 1;
    write_request.is_request = true;

    command_hub.send(write_request, None)?;

    Ok(())
}
```

### Modbus RTU Communication

```rust
use horus_library::nodes::ModbusNode;
use horus_library::ModbusMessage;
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create Modbus RTU node (serial communication)
    let mut modbus = ModbusNode::new()?;

    // For RTU, address is the serial port path
    modbus.set_connection("/dev/ttyUSB0", 0, 1);  // Port not used for RTU
    modbus.set_timeout(2000);

    // RTU-specific configuration would be:
    // modbus.set_serial_config(9600, Parity::Even, 1);

    runtime.add_node(modbus);

    let request_hub = Hub::new("modbus_request")?;

    // Read coils (digital outputs)
    let mut request = ModbusMessage::default();
    request.slave_id = 1;
    request.function_code = 1;  // Read Coils
    request.start_address = 0;
    request.quantity = 8;
    request.is_request = true;

    request_hub.send(request, None)?;

    runtime.run()?;
    Ok(())
}
```

### Multi-Device Polling

```rust
use horus_library::nodes::ModbusNode;
use horus_library::ModbusMessage;
use horus_core::{Node, Runtime, Hub};
use std::time::Duration;
use std::thread;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Single Modbus node can handle multiple slave devices
    let mut modbus = ModbusNode::new()?;
    modbus.set_connection("192.168.1.10", 502, 1);

    runtime.add_node(modbus);

    let request_hub = Hub::new("modbus_request")?;
    let response_hub = Hub::new("modbus_response")?;

    // Poll multiple slave devices
    for slave_id in 1..=5 {
        let mut request = ModbusMessage::default();
        request.slave_id = slave_id;
        request.function_code = 3;  // Read Holding Registers
        request.start_address = 0;
        request.quantity = 10;
        request.is_request = true;

        request_hub.send(request, None)?;
        thread::sleep(Duration::from_millis(100));  // Delay between polls
    }

    // Process responses
    while let Some(response) = response_hub.recv(None) {
        eprintln!("Slave {}: {:?}", response.slave_id, &response.data[..response.data_length as usize]);
    }

    Ok(())
}
```

### SCADA System Integration

```rust
use horus_library::nodes::ModbusNode;
use horus_library::ModbusMessage;
use horus_core::{Node, Runtime, Hub};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut runtime = Runtime::new()?;

    // Create Modbus node for PLC communication
    let mut plc_modbus = ModbusNode::new_with_topics(
        "plc_request",
        "plc_response",
        "plc_status"
    )?;
    plc_modbus.set_connection("192.168.0.100", 502, 1);

    // Create Modbus node for sensor network
    let mut sensor_modbus = ModbusNode::new_with_topics(
        "sensor_request",
        "sensor_response",
        "sensor_status"
    )?;
    sensor_modbus.set_connection("192.168.0.200", 502, 1);

    runtime.add_node(plc_modbus);
    runtime.add_node(sensor_modbus);

    let plc_hub = Hub::new("plc_request")?;
    let sensor_hub = Hub::new("sensor_request")?;

    // Read process values from PLC
    let mut plc_request = ModbusMessage::default();
    plc_request.slave_id = 1;
    plc_request.function_code = 4;  // Read Input Registers
    plc_request.start_address = 1000;  // Process data area
    plc_request.quantity = 20;
    plc_request.is_request = true;
    plc_hub.send(plc_request, None)?;

    // Read sensor data
    let mut sensor_request = ModbusMessage::default();
    sensor_request.slave_id = 1;
    sensor_request.function_code = 3;  // Read Holding Registers
    sensor_request.start_address = 0;
    sensor_request.quantity = 5;
    sensor_request.is_request = true;
    sensor_hub.send(sensor_request, None)?;

    runtime.run()?;
    Ok(())
}
```

## Modbus Protocol Overview

### Modbus TCP vs RTU

| Feature | Modbus TCP | Modbus RTU |
|---------|------------|------------|
| **Physical Layer** | Ethernet (TCP/IP) | RS-232/RS-485 Serial |
| **Default Port** | 502 | N/A (serial port) |
| **Baud Rate** | N/A | 9600-115200 bps |
| **Error Checking** | TCP checksum | CRC-16 |
| **Max Nodes** | Limited by network | 247 devices |
| **Distance** | 100m per segment (Cat5e) | 1200m (RS-485) |
| **Speed** | 100 Mbps+ | 9600-115200 bps |
| **Topology** | Star/Tree | Bus/Multidrop |

### Function Codes

| Code | Name | Description | Data Type |
|------|------|-------------|-----------|
| **1** | Read Coils | Read 1-2000 coils | Discrete Output (0x) |
| **2** | Read Discrete Inputs | Read 1-2000 discrete inputs | Discrete Input (1x) |
| **3** | Read Holding Registers | Read 1-125 holding registers | Holding Register (4x) |
| **4** | Read Input Registers | Read 1-125 input registers | Input Register (3x) |
| **5** | Write Single Coil | Write single coil | Discrete Output (0x) |
| **6** | Write Single Register | Write single holding register | Holding Register (4x) |
| **15** | Write Multiple Coils | Write 1-1968 coils | Discrete Output (0x) |
| **16** | Write Multiple Registers | Write 1-123 holding registers | Holding Register (4x) |

### Address Notation

Modbus uses different address notations:

```
Traditional Notation:
0xxxxx = Coils (00001-09999)
1xxxxx = Discrete Inputs (10001-19999)
3xxxxx = Input Registers (30001-39999)
4xxxxx = Holding Registers (40001-49999)

Protocol Address (zero-based):
Function Code 1/5/15:  0-65535 (coils)
Function Code 2:       0-65535 (discrete inputs)
Function Code 3/6/16:  0-65535 (holding registers)
Function Code 4:       0-65535 (input registers)
```

**Example**: To read holding register 40100 in traditional notation:
- Function Code: 3 (Read Holding Registers)
- Start Address: 99 (40100 - 40001 = 99)

## Register Types

### Coils (Function Codes 1, 5, 15)

Single-bit read-write values, typically used for:
- Digital outputs (relay control, solenoid valves)
- Binary flags and status bits
- On/off control signals

```rust
// Read 8 coils starting at address 0
let mut request = ModbusMessage::default();
request.function_code = 1;  // Read Coils
request.start_address = 0;
request.quantity = 8;

// Write single coil at address 5 to ON
let mut write_request = ModbusMessage::default();
write_request.function_code = 5;  // Write Single Coil
write_request.start_address = 5;
write_request.data[0] = 0xFF00;  // ON (0x0000 = OFF)
```

### Discrete Inputs (Function Code 2)

Single-bit read-only values, typically used for:
- Digital inputs (limit switches, sensors)
- Alarm status
- Safety interlocks

```rust
// Read 16 discrete inputs starting at address 100
let mut request = ModbusMessage::default();
request.function_code = 2;  // Read Discrete Inputs
request.start_address = 100;
request.quantity = 16;
```

### Holding Registers (Function Codes 3, 6, 16)

16-bit read-write values, typically used for:
- Setpoints (temperature, pressure, speed)
- Configuration parameters
- Control values

```rust
// Read 10 holding registers starting at address 200
let mut request = ModbusMessage::default();
request.function_code = 3;  // Read Holding Registers
request.start_address = 200;
request.quantity = 10;

// Write single holding register
let mut write_request = ModbusMessage::default();
write_request.function_code = 6;  // Write Single Register
write_request.start_address = 200;
write_request.data[0] = 1500;  // Value to write
```

### Input Registers (Function Code 4)

16-bit read-only values, typically used for:
- Sensor readings (temperature, pressure, flow)
- Process variables
- Status values

```rust
// Read 5 input registers starting at address 0
let mut request = ModbusMessage::default();
request.function_code = 4;  // Read Input Registers
request.start_address = 0;
request.quantity = 5;
```

## Data Type Conversions

### 16-bit Integer

```rust
let value: u16 = response.data[0];
let signed_value: i16 = response.data[0] as i16;
```

### 32-bit Integer (2 registers)

```rust
// Big-endian (high word first)
let high = response.data[0] as u32;
let low = response.data[1] as u32;
let value: u32 = (high << 16) | low;

// Little-endian (low word first)
let low = response.data[0] as u32;
let high = response.data[1] as u32;
let value: u32 = (high << 16) | low;
```

### 32-bit Float (2 registers)

```rust
let high = response.data[0] as u32;
let low = response.data[1] as u32;
let bits: u32 = (high << 16) | low;
let float_value: f32 = f32::from_bits(bits);
```

### Scaled Values

```rust
// Temperature sensor with 0.1°C resolution
let temp_raw = response.data[0];
let temperature = temp_raw as f32 / 10.0;

// Pressure sensor 0-100 bar mapped to 0-10000
let pressure_raw = response.data[0];
let pressure = (pressure_raw as f32 / 10000.0) * 100.0;
```

## Troubleshooting

### Issue: Connection Timeout

**Symptoms**: No response from device, timeout errors

**Possible Causes**:
1. Incorrect IP address or port
2. Network connectivity issues
3. Device offline or not responding
4. Firewall blocking port 502

**Solutions**:

```rust
// Increase timeout
modbus.set_timeout(10000);  // 10 seconds

// Verify connection parameters
modbus.set_connection("192.168.1.100", 502, 1);

// Check network connectivity
// ping 192.168.1.100
// telnet 192.168.1.100 502
```

**Debugging Steps**:
1. Verify device IP with `ping`
2. Check if port 502 is open: `telnet <ip> 502`
3. Verify slave ID matches device configuration
4. Check network cables and switches
5. Review firewall rules

### Issue: CRC Errors (Modbus RTU)

**Symptoms**: Invalid CRC in responses, communication errors

**Possible Causes**:
1. Incorrect baud rate
2. Wrong parity setting
3. Electrical noise on serial line
4. Cable too long or poor quality
5. Incorrect termination resistors

**Solutions**:

```rust
// Verify serial settings match device
// Common configurations:
// 9600-8-N-1 (9600 baud, 8 data bits, No parity, 1 stop bit)
// 19200-8-E-1 (19200 baud, 8 data bits, Even parity, 1 stop bit)

// For RTU, ensure proper serial configuration
modbus.set_connection("/dev/ttyUSB0", 0, 1);
```

**Hardware Checks**:
- Use shielded twisted-pair cable
- Keep cable length under 1200m for RS-485
- Install 120-ohm termination resistors at both ends
- Separate from high-voltage cables
- Check ground connections

### Issue: Exception Responses

**Symptoms**: Function code with MSB set (e.g., 0x83 instead of 0x03)

**Exception Codes**:

| Code | Name | Description | Solution |
|------|------|-------------|----------|
| **01** | Illegal Function | Unsupported function code | Use supported function code |
| **02** | Illegal Data Address | Invalid register address | Check address range |
| **03** | Illegal Data Value | Invalid data value | Verify data range |
| **04** | Slave Device Failure | Device error | Check device status |
| **05** | Acknowledge | Long operation in progress | Wait and retry |
| **06** | Slave Device Busy | Device busy | Retry after delay |

**Handling Exceptions**:

```rust
if response.function_code & 0x80 != 0 {
    let exception_code = response.data[0];
    match exception_code {
        0x01 => eprintln!("Illegal function"),
        0x02 => eprintln!("Illegal data address"),
        0x03 => eprintln!("Illegal data value"),
        0x04 => eprintln!("Slave device failure"),
        _ => eprintln!("Unknown exception: {}", exception_code),
    }
}
```

### Issue: No Response from Device

**Symptoms**: Request sent but no response received

**Possible Causes**:
1. Wrong slave ID
2. Device not configured for Modbus
3. Broadcast address used (slave ID 0)
4. Device in wrong mode

**Solutions**:

```rust
// Verify slave ID (1-247)
modbus.set_connection("192.168.1.100", 502, 1);

// Try scanning for devices
for slave_id in 1..=247 {
    modbus.set_connection("192.168.1.100", 502, slave_id);
    // Send test request
    // Check for response
}
```

**Debugging**:
1. Verify slave ID in device configuration
2. Check device mode (Modbus TCP vs RTU)
3. Ensure device is not in local/manual mode
4. Review device manual for Modbus settings

### Issue: Data Corruption or Incorrect Values

**Symptoms**: Received values don't match expected range

**Possible Causes**:
1. Wrong byte order (endianness)
2. Incorrect scaling/conversion
3. Reading wrong register type
4. Mismatched data types

**Solutions**:

```rust
// Try different byte orders for 32-bit values
// Big-endian (ABCD)
let value_be = (response.data[0] as u32) << 16 | response.data[1] as u32;

// Little-endian (CDAB)
let value_le = (response.data[1] as u32) << 16 | response.data[0] as u32;

// Mid-big-endian (BADC)
let value_mbe = (response.data[0] as u32) | (response.data[1] as u32) << 16;

// Check data type and scaling
let scaled = response.data[0] as f32 / 100.0;  // If device uses 0.01 resolution
```

### Issue: Slow Response Times

**Symptoms**: Communication works but is slow

**Possible Causes**:
1. Network congestion
2. Long timeout values
3. Polling too many registers
4. Device processing time

**Solutions**:

```rust
// Reduce timeout for faster failure detection
modbus.set_timeout(1000);  // 1 second

// Read only necessary registers
request.quantity = 5;  // Instead of 50

// Implement polling strategy
// - Poll critical data more frequently
// - Poll non-critical data less frequently
// - Use separate Modbus nodes for different devices
```

## Integration with Industrial Equipment

### PLC Integration

```rust
// Typical PLC register mapping:
// 40001-40100: Digital I/O status
// 40101-40200: Analog input values
// 40201-40300: Setpoints and parameters
// 40301-40400: Process values

let mut plc = ModbusNode::new()?;
plc.set_connection("192.168.1.10", 502, 1);

// Read digital I/O status
let mut io_request = ModbusMessage::default();
io_request.function_code = 3;
io_request.start_address = 0;    // 40001 in traditional notation
io_request.quantity = 100;

// Write setpoint
let mut setpoint_request = ModbusMessage::default();
setpoint_request.function_code = 6;
setpoint_request.start_address = 200;  // 40201
setpoint_request.data[0] = 750;        // Setpoint value
```

### Variable Frequency Drive (VFD)

```rust
// Common VFD Modbus registers:
// Frequency setpoint: 40002 (0.01 Hz resolution)
// Run command: 40001 (1=run, 0=stop)
// Status: 30001 (running, fault, etc.)
// Actual frequency: 30002

let mut vfd = ModbusNode::new()?;
vfd.set_connection("192.168.1.50", 502, 1);

// Start motor at 50.00 Hz
let mut start_cmd = ModbusMessage::default();
start_cmd.function_code = 16;  // Write Multiple Registers
start_cmd.start_address = 0;
start_cmd.quantity = 2;
start_cmd.data[0] = 1;      // Run command
start_cmd.data[1] = 5000;   // 50.00 Hz (0.01 Hz resolution)
start_cmd.data_length = 2;
```

### Temperature Controller

```rust
// Typical temperature controller registers:
// PV (Process Value): 30001 (0.1°C resolution)
// SV (Setpoint Value): 40001 (0.1°C resolution)
// Output %: 30002 (0-1000 = 0-100.0%)
// Alarm status: 10001 (discrete input)

let mut temp_controller = ModbusNode::new()?;
temp_controller.set_connection("192.168.1.60", 502, 1);

// Read current temperature
let mut read_pv = ModbusMessage::default();
read_pv.function_code = 4;  // Read Input Registers
read_pv.start_address = 0;  // PV address
read_pv.quantity = 1;

// Set target temperature to 85.5°C
let mut write_sv = ModbusMessage::default();
write_sv.function_code = 6;
write_sv.start_address = 0;  // SV address
write_sv.data[0] = 855;      // 85.5°C with 0.1°C resolution
```

### Flow Meter

```rust
// Flow meter registers:
// Instantaneous flow: 30001-30002 (32-bit float)
// Total flow: 30003-30004 (32-bit float)
// Unit selection: 40001
// Flow direction: 10001

let mut flow_meter = ModbusNode::new()?;
flow_meter.set_connection("192.168.1.70", 502, 1);

// Read instantaneous flow (32-bit float)
let mut read_flow = ModbusMessage::default();
read_flow.function_code = 4;
read_flow.start_address = 0;
read_flow.quantity = 2;

// Convert response to float
let high = response.data[0] as u32;
let low = response.data[1] as u32;
let flow_bits = (high << 16) | low;
let flow_rate = f32::from_bits(flow_bits);
```

### Pressure Transmitter

```rust
// 4-20mA pressure transmitter with Modbus:
// Pressure value: 30001 (scaled 0-10000 = 0-100 bar)
// Status bits: 10001
// Configuration: 40001-40010

let mut pressure = ModbusNode::new()?;
pressure.set_connection("192.168.1.80", 502, 1);

// Read pressure
let mut read_pressure = ModbusMessage::default();
read_pressure.function_code = 4;
read_pressure.start_address = 0;
read_pressure.quantity = 1;

// Convert to engineering units
let pressure_raw = response.data[0];
let pressure_bar = (pressure_raw as f32 / 10000.0) * 100.0;
```

## Performance Considerations

### Polling Rates

**Recommended polling intervals by application**:

| Application | Polling Rate | Notes |
|-------------|-------------|-------|
| Fast process control | 10-50 ms | VFDs, servo drives |
| Standard I/O | 100-500 ms | Digital I/O, relays |
| Analog measurements | 200-1000 ms | Temperature, pressure |
| Status monitoring | 1-5 seconds | Alarms, diagnostics |
| Configuration | On-demand | Parameter changes |

### Network Bandwidth

```
Modbus TCP frame overhead: ~12 bytes (MBAP header)
Modbus PDU overhead: ~2 bytes (function code + byte count)
Data: 2 bytes per register

Example: Reading 10 registers
Request: 12 + 6 = 18 bytes
Response: 12 + 2 + 20 = 34 bytes
Total: 52 bytes per transaction

At 100 ms polling rate:
Bandwidth = 52 bytes * 10 polls/sec = 520 bytes/sec = 4.16 kbps
```

### CPU Usage

Minimal CPU usage - primarily network I/O bound. The Modbus node handles:
- Request/response matching
- Register caching
- Timeout management
- Error handling

### Memory Usage

Small fixed memory footprint:
- Node structure: ~100 bytes
- Register cache: 2 bytes per cached register
- Message buffers: 256 bytes per message

## Best Practices

### 1. Connection Management

```rust
// Use connection pooling for multiple devices
let mut modbus_nodes = vec![];
for (address, slave_id) in device_list {
    let mut node = ModbusNode::new_with_topics(
        &format!("device_{}_req", slave_id),
        &format!("device_{}_resp", slave_id),
        &format!("device_{}_status", slave_id)
    )?;
    node.set_connection(&address, 502, slave_id);
    modbus_nodes.push(node);
}
```

### 2. Error Handling

```rust
// Always check for exceptions
if response.function_code & 0x80 != 0 {
    // Handle exception
    eeprintln!("Modbus exception: {}", response.data[0]);
    return Err("Modbus exception");
}

// Implement retry logic
let max_retries = 3;
for attempt in 0..max_retries {
    match send_request(&request) {
        Ok(response) => break,
        Err(e) if attempt < max_retries - 1 => {
            thread::sleep(Duration::from_millis(100));
            continue;
        },
        Err(e) => return Err(e),
    }
}
```

### 3. Register Mapping

```rust
// Use constants for register addresses
const TEMP_SETPOINT: u16 = 100;
const TEMP_PROCESS_VALUE: u16 = 0;
const MOTOR_SPEED: u16 = 200;
const MOTOR_RUN_CMD: u16 = 50;

// Document register maps
// Register Map:
// 40001-40010: Temperature control
// 40011-40020: Motor control
// 30001-30010: Process values
// 10001-10020: Alarm status
```

### 4. Data Validation

```rust
// Validate received data
if response.data_length != request.quantity {
    eeprintln!("Invalid response length");
    return Err("Data length mismatch");
}

// Check value ranges
let temp = response.data[0];
if temp < 0 || temp > 2000 {  // 0-200.0°C range
    eeprintln!("Temperature out of range: {}", temp);
}
```

## Related Nodes

- **TcpClientNode**: Lower-level TCP communication
- **SerialNode**: Serial port communication for RTU
- **PidControllerNode**: Can use Modbus data for control
- **DataLoggerNode**: Log Modbus data for analysis

## See Also

- [Modbus Protocol Specification](https://modbus.org/specs.php)
- [Modbus TCP Security](https://www.modbus.org/docs/MB-TCP-Security-v21_2018-07-24.pdf)
- [Industrial Ethernet Guide](https://en.wikipedia.org/wiki/Modbus)
- [tokio-modbus Rust Library](https://github.com/slowtec/tokio-modbus)
