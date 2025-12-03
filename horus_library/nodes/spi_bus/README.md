# SPI Bus Node

Serial Peripheral Interface (SPI) bus communication for high-speed peripherals including sensors, displays, ADCs, DACs, memory, and RF modules with configurable modes and chip select management.

## Overview

The SPI Bus Node provides full-duplex SPI communication via the Linux spidev interface. It supports multiple SPI buses, up to 8 chip select lines per bus, all 4 SPI modes, and configurable clock speeds up to 50MHz+, with automatic hardware/simulation fallback.

Supports standard SPI devices: ADXL345, L3GD20, BMP280, SSD1306, ILI9341, ST7735, MCP3008, MCP4921, W25Q32, SX1276, RFM95W, nRF24L01+, TMC2130, BMI088, ICM-20948, and more.

Key features:
- Multiple SPI bus support (/dev/spidev0.0, /dev/spidev1.0, etc.)
- Up to 8 chip select (CS) lines per bus
- All 4 SPI modes (CPOL/CPHA combinations)
- Configurable clock speeds (100kHz - 50MHz+)
- Full-duplex communication
- Per-device configuration
- Transaction queuing
- Error detection and statistics
- Simulation fallback when hardware unavailable
- Preset configurations for common devices

## Topics

### Publishers

| Topic | Type | Description |
|-------|------|-------------|
| `spi{bus}/response` | `SpiMessage` | SPI transaction responses with received data |

### Subscribers

| Topic | Type | Description |
|-------|------|-------------|
| `spi{bus}/request` | `SpiMessage` | SPI transaction requests to execute |

Note: `{bus}` is the SPI bus number (e.g., `spi0/request`, `spi1/response`)

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `bus_number` | `u8` | `0` | SPI bus number (0, 1, 2, etc.) |
| `default_speed_hz` | `u32` | `1000000` | Default clock speed in Hz (1 MHz) |
| `default_mode` | `u8` | `0` | Default SPI mode (0-3) |
| `default_bits_per_word` | `u8` | `8` | Default bits per word (typically 8) |

### Per-Device Configuration

Configure individual devices on specific chip select lines:

| Parameter | Type | Description |
|-----------|------|-------------|
| `chip_select` | `u8` | CS line number (0-7) |
| `speed_hz` | `u32` | Device-specific clock speed |
| `mode` | `u8` | SPI mode (0-3) |
| `bits_per_word` | `u8` | Bits per word (typically 8) |
| `cs_high` | `bool` | Chip select active high (default: false = active low) |
| `lsb_first` | `bool` | LSB first bit order (default: false = MSB first) |
| `three_wire` | `bool` | 3-wire bidirectional mode (default: false) |

### SPI Modes

| Mode | CPOL | CPHA | Clock Polarity | Clock Phase | Common Devices |
|------|------|------|----------------|-------------|----------------|
| 0 | 0 | 0 | Idle low | Sample on leading edge | MCP3008, BME280, nRF24L01+ |
| 1 | 0 | 1 | Idle low | Sample on trailing edge | - |
| 2 | 1 | 0 | Idle high | Sample on leading edge | - |
| 3 | 1 | 1 | Idle high | Sample on trailing edge | ADXL345, TMC2130 |

### Hardware Device Paths

SPI devices are accessed via Linux spidev:
- `/dev/spidev0.0` - Bus 0, CS 0
- `/dev/spidev0.1` - Bus 0, CS 1
- `/dev/spidev1.0` - Bus 1, CS 0
- etc.

## Message Types

### SpiMessage

SPI transaction message for request/response:

```rust
pub struct SpiMessage {
    pub bus: u8,                  // SPI bus number (0, 1, 2, etc.)
    pub chip_select: u8,          // CS line (0-7)
    pub mode: u8,                 // SPI mode (0-3)
    pub speed_hz: u32,            // Clock speed in Hz
    pub bits_per_word: u8,        // Bits per word (typically 8)
    pub tx_data: [u8; 256],       // Data to transmit (MOSI)
    pub rx_data: [u8; 256],       // Data received (MISO)
    pub length: u16,              // Transaction length in bytes
    pub cs_high: bool,            // CS active high (default: false)
    pub lsb_first: bool,          // LSB first (default: false)
    pub three_wire: bool,         // 3-wire mode (default: false)
    pub success: bool,            // Transaction successful
    pub timestamp: u64,           // Transaction time (ns since epoch)
}
```

**Mode Constants**:
- `SpiMessage::MODE_0 = 0` - CPOL=0, CPHA=0 (most common)
- `SpiMessage::MODE_1 = 1` - CPOL=0, CPHA=1
- `SpiMessage::MODE_2 = 2` - CPOL=1, CPHA=0
- `SpiMessage::MODE_3 = 3` - CPOL=1, CPHA=1

**Helper Methods**:
```rust
SpiMessage::new(bus, chip_select, data);           // Create basic message
SpiMessage::with_config(bus, cs, data, mode, hz);  // Create with config
msg.set_mode(mode);                                // Set SPI mode
msg.set_tx_data(data);                             // Set transmit data
msg.rx_data_slice();                               // Get received data slice
msg.tx_data_slice();                               // Get transmit data slice
```

## Public API

### Construction

```rust
use horus_library::nodes::SpiBusNode;

// Create SPI bus node for bus 0
let mut spi = SpiBusNode::new(0)?;

// Create SPI bus node for bus 1
let mut spi1 = SpiBusNode::new(1)?;
```

### Configuration Methods

```rust
// Set default SPI speed in Hz
spi.set_default_speed(1_000_000);   // 1 MHz
spi.set_default_speed(10_000_000);  // 10 MHz

// Set default SPI mode (0-3)
spi.set_default_mode(SpiMessage::MODE_0);  // Mode 0
spi.set_default_mode(SpiMessage::MODE_3);  // Mode 3

// Set default bits per word
spi.set_default_bits_per_word(8);

// Configure a specific device on a chip select line
spi.configure_device(0, 2_000_000, SpiMessage::MODE_0);  // CS0: 2MHz, Mode 0
spi.configure_device(1, 500_000, SpiMessage::MODE_3);    // CS1: 500kHz, Mode 3

// Set chip select polarity
spi.set_cs_high(0, false);  // Active low (default)
spi.set_cs_high(1, true);   // Active high

// Set bit order
spi.set_lsb_first(0, false);  // MSB first (default)
spi.set_lsb_first(1, true);   // LSB first

// Enable 3-wire mode
spi.set_three_wire(0, false);  // Standard 4-wire (default)
spi.set_three_wire(1, true);   // 3-wire bidirectional
```

### Preset Device Configurations

```rust
// Configure for common SPI devices
spi.configure_adxl345(0);      // ADXL345 accelerometer (Mode 3, 5MHz)
spi.configure_bme280(0);       // BME280 sensor (Mode 0, 10MHz)
spi.configure_mcp3008(0);      // MCP3008 ADC (Mode 0, 1.35MHz)
spi.configure_nrf24l01(0);     // nRF24L01+ RF (Mode 0, 10MHz)
spi.configure_w25q_flash(0);   // W25Q flash memory (Mode 0, 50MHz)
spi.configure_st7735(0);       // ST7735 TFT display (Mode 0, 15MHz)
spi.configure_tmc2130(0);      // TMC2130 stepper driver (Mode 3, 4MHz)
```

### Direct Transfer Methods

```rust
// Write data to SPI device (transmit only)
let data = [0x01, 0x02, 0x03];
let success = spi.write(0, &data, None);  // CS 0

// Read data from SPI device (receive only, transmit zeros)
if let Some(rx_data) = spi.read(0, 4, None) {  // CS 0, read 4 bytes
    println!("Received: {:02X?}", rx_data);
}

// Write then read (common pattern for reading registers)
let write_data = [0x80 | 0x01];  // Read register 0x01
if let Some(rx_data) = spi.write_read(0, &write_data, 2, None) {
    println!("Register data: {:02X?}", rx_data);
}

// Execute full-duplex transfer
let mut msg = SpiMessage::new(0, 0, &[0x00, 0x01]);
spi.transfer(&mut msg, None);
if msg.success {
    println!("RX: {:02X?}", &msg.rx_data[..msg.length as usize]);
}
```

### Statistics Methods

```rust
// Get statistics (transactions, errors, bytes TX/RX)
let (tx_count, error_count, bytes_tx, bytes_rx) = spi.get_statistics();
println!("Transactions: {}, Errors: {}", tx_count, error_count);
println!("TX: {} bytes, RX: {} bytes", bytes_tx, bytes_rx);

// Reset statistics
spi.reset_statistics();
```

## Usage Examples

### Basic SPI Communication

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Create SPI bus node
    let mut spi = SpiBusNode::new(0)?;
    spi.set_default_speed(1_000_000);  // 1 MHz
    spi.set_default_mode(SpiMessage::MODE_0);

    // Configure device on CS 0
    spi.configure_device(0, 2_000_000, SpiMessage::MODE_0);

    scheduler.add(Box::new(spi), 1, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### Reading from MCP3008 ADC

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure SPI for MCP3008
    let mut spi = SpiBusNode::new(0)?;
    spi.configure_mcp3008(0);  // CS 0: 1.35MHz, Mode 0

    scheduler.add(Box::new(spi), 1, Some(true));

    // ADC reader node
    let reader = node! {
        name: "adc_reader",
        tick: |ctx| {
            // Read channel 0 from MCP3008
            let mut msg = SpiMessage::new(0, 0, &[0x01, 0x80, 0x00]);

            let req_hub = Hub::<SpiMessage>::new("spi0/request")?;
            req_hub.send(msg, &mut None)?;

            let resp_hub = Hub::<SpiMessage>::new("spi0/response")?;
            if let Some(resp) = resp_hub.recv(None) {
                if resp.success {
                    let value = ((resp.rx_data[1] as u16 & 0x03) << 8)
                              | (resp.rx_data[2] as u16);
                    let voltage = (value as f32 / 1023.0) * 3.3;
                    ctx.log_info(&format!("ADC: {} ({}V)", value, voltage));
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

### Multiple SPI Devices

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    // Configure multiple devices on different CS lines
    let mut spi = SpiBusNode::new(0)?;

    // CS 0: ADXL345 accelerometer
    spi.configure_adxl345(0);

    // CS 1: BME280 environmental sensor
    spi.configure_bme280(1);

    // CS 2: W25Q flash memory
    spi.configure_w25q_flash(2);

    scheduler.add(Box::new(spi), 1, Some(true));

    // Multi-device reader
    let reader = node! {
        name: "multi_device",
        tick: |ctx| {
            let req_hub = Hub::<SpiMessage>::new("spi0/request")?;
            let resp_hub = Hub::<SpiMessage>::new("spi0/response")?;

            // Read from ADXL345 (CS 0)
            let msg_accel = SpiMessage::new(0, 0, &[0x80 | 0x00, 0x00]);
            req_hub.send(msg_accel, &mut None)?;

            // Read from BME280 (CS 1)
            let msg_bme = SpiMessage::new(0, 1, &[0xD0, 0x00]);
            req_hub.send(msg_bme, &mut None)?;

            // Process responses
            while let Some(resp) = resp_hub.recv(None) {
                if resp.success {
                    ctx.log_info(&format!(
                        "CS{}: {:02X?}",
                        resp.chip_select,
                        &resp.rx_data[..resp.length as usize]
                    ));
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

### ADXL345 Accelerometer Communication

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut spi = SpiBusNode::new(0)?;
    spi.configure_adxl345(0);  // Mode 3, 5MHz

    scheduler.add(Box::new(spi), 1, Some(true));

    let accel = node! {
        name: "adxl345",
        tick: |ctx| {
            let req_hub = Hub::<SpiMessage>::new("spi0/request")?;
            let resp_hub = Hub::<SpiMessage>::new("spi0/response")?;

            // Read DEVID register (should return 0xE5)
            let mut msg = SpiMessage::new(0, 0, &[0x80 | 0x00, 0x00]);
            req_hub.send(msg, &mut None)?;

            if let Some(resp) = resp_hub.recv(None) {
                if resp.success && resp.rx_data[1] == 0xE5 {
                    ctx.log_info("ADXL345 detected!");

                    // Put in measurement mode
                    let power_msg = SpiMessage::new(0, 0, &[0x2D, 0x08]);
                    req_hub.send(power_msg, &mut None)?;

                    // Read acceleration data (6 bytes from register 0x32)
                    let data_msg = SpiMessage::new(0, 0, &[
                        0x80 | 0x40 | 0x32,  // Read, multi-byte, register 0x32
                        0, 0, 0, 0, 0, 0
                    ]);
                    req_hub.send(data_msg, &mut None)?;

                    if let Some(data_resp) = resp_hub.recv(None) {
                        if data_resp.success {
                            let x = i16::from_le_bytes([
                                data_resp.rx_data[1],
                                data_resp.rx_data[2]
                            ]);
                            let y = i16::from_le_bytes([
                                data_resp.rx_data[3],
                                data_resp.rx_data[4]
                            ]);
                            let z = i16::from_le_bytes([
                                data_resp.rx_data[5],
                                data_resp.rx_data[6]
                            ]);

                            ctx.log_info(&format!("Accel: X={}, Y={}, Z={}", x, y, z));
                        }
                    }
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(accel), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### nRF24L01+ RF Module

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut spi = SpiBusNode::new(0)?;
    spi.configure_nrf24l01(0);  // Mode 0, 10MHz

    scheduler.add(Box::new(spi), 1, Some(true));

    let rf = node! {
        name: "nrf24l01",
        tick: |ctx| {
            let req_hub = Hub::<SpiMessage>::new("spi0/request")?;
            let resp_hub = Hub::<SpiMessage>::new("spi0/response")?;

            // Read CONFIG register
            let msg = SpiMessage::new(0, 0, &[0x00, 0x00]);  // R_REGISTER | 0x00
            req_hub.send(msg, &mut None)?;

            if let Some(resp) = resp_hub.recv(None) {
                if resp.success {
                    ctx.log_info(&format!("CONFIG: 0x{:02X}", resp.rx_data[1]));

                    // Write to CONFIG register (power up, enable RX)
                    let write_msg = SpiMessage::new(0, 0, &[0x20, 0x0F]);
                    req_hub.send(write_msg, &mut None)?;

                    ctx.log_info("nRF24L01+ configured");
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(rf), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

### W25Q Flash Memory

```rust
use horus_library::prelude::*;

fn main() -> Result<()> {
    let mut scheduler = Scheduler::new();

    let mut spi = SpiBusNode::new(0)?;
    spi.configure_w25q_flash(0);  // Mode 0, 50MHz

    scheduler.add(Box::new(spi), 1, Some(true));

    let flash = node! {
        name: "w25q_flash",
        tick: |ctx| {
            let req_hub = Hub::<SpiMessage>::new("spi0/request")?;
            let resp_hub = Hub::<SpiMessage>::new("spi0/response")?;

            // Read JEDEC ID (0x9F command)
            let msg = SpiMessage::new(0, 0, &[0x9F, 0x00, 0x00, 0x00]);
            req_hub.send(msg, &mut None)?;

            if let Some(resp) = resp_hub.recv(None) {
                if resp.success {
                    let manufacturer = resp.rx_data[1];
                    let mem_type = resp.rx_data[2];
                    let capacity = resp.rx_data[3];

                    ctx.log_info(&format!(
                        "Flash ID: Mfr=0x{:02X}, Type=0x{:02X}, Cap=0x{:02X}",
                        manufacturer, mem_type, capacity
                    ));

                    // Read first 16 bytes from address 0x000000
                    let read_msg = SpiMessage::new(0, 0, &[
                        0x03,  // Read data command
                        0x00, 0x00, 0x00,  // Address
                        0, 0, 0, 0, 0, 0, 0, 0,  // 16 dummy bytes
                        0, 0, 0, 0, 0, 0, 0, 0
                    ]);
                    req_hub.send(read_msg, &mut None)?;

                    if let Some(data_resp) = resp_hub.recv(None) {
                        if data_resp.success {
                            ctx.log_info(&format!(
                                "Data: {:02X?}",
                                &data_resp.rx_data[4..20]
                            ));
                        }
                    }
                }
            }
            Ok(())
        }
    };

    scheduler.add(Box::new(flash), 2, Some(true));
    scheduler.run()?;
    Ok(())
}
```

## Hardware Setup

### Raspberry Pi Wiring

```
Raspberry Pi          SPI Device
GPIO 10 (MOSI)   -->  MOSI (SDI/DI/SI)
GPIO 9  (MISO)   <--  MISO (SDO/DO/SO)
GPIO 11 (SCLK)   -->  SCLK (SCK/CLK)
GPIO 8  (CE0)    -->  CS/SS
3.3V             -->  VCC (or 5V if device requires)
GND              -->  GND
```

### Multiple Chip Selects

```
Raspberry Pi          Device 1    Device 2
GPIO 10 (MOSI)   -->  MOSI   -->  MOSI
GPIO 9  (MISO)   <--  MISO   <--  MISO
GPIO 11 (SCLK)   -->  SCLK   -->  SCLK
GPIO 8  (CE0)    -->  CS
GPIO 7  (CE1)    ------------->  CS
```

### SPI Pin Assignments

| Signal | Raspberry Pi | BCM GPIO | Function |
|--------|--------------|----------|----------|
| MOSI | Pin 19 | GPIO 10 | Master Out Slave In |
| MISO | Pin 21 | GPIO 9 | Master In Slave Out |
| SCLK | Pin 23 | GPIO 11 | Serial Clock |
| CE0 | Pin 24 | GPIO 8 | Chip Enable 0 |
| CE1 | Pin 26 | GPIO 7 | Chip Enable 1 |

### System Requirements

```bash
# Install SPI development tools
sudo apt install libraspberrypi-dev

# Enable SPI interface
sudo raspi-config
# Select: Interface Options -> SPI -> Enable

# Add user to SPI group
sudo usermod -a -G spi $USER

# Reboot or re-login for group changes
sudo reboot

# Verify SPI devices
ls -l /dev/spidev*
# Should show: /dev/spidev0.0, /dev/spidev0.1, etc.

# Check SPI module loaded
lsmod | grep spi
# Should show: spi_bcm2835
```

### Enable in Project

```toml
[dependencies]
horus_library = { version = "0.1", features = ["spi-hardware"] }
```

```bash
cargo build --features="spi-hardware"
```

## SPI Protocol Details

### SPI Communication Overview

SPI is a synchronous serial communication protocol:

1. **Master** (Raspberry Pi) generates clock signal
2. **Slave** (device) responds to clock
3. **Full-duplex**: Data transmitted and received simultaneously
4. **Four signals**:
   - MOSI: Master Out Slave In (TX from master)
   - MISO: Master In Slave Out (RX to master)
   - SCLK: Serial Clock (generated by master)
   - CS/SS: Chip Select (active low typically)

### SPI Modes (CPOL/CPHA)

| Mode | CPOL | CPHA | Idle State | Sample Edge | Shift Edge |
|------|------|------|------------|-------------|------------|
| 0 | 0 | 0 | Low | Rising | Falling |
| 1 | 0 | 1 | Low | Falling | Rising |
| 2 | 1 | 0 | High | Falling | Rising |
| 3 | 1 | 1 | High | Rising | Falling |

**CPOL** (Clock Polarity): Clock idle state
- 0 = Clock idle low
- 1 = Clock idle high

**CPHA** (Clock Phase): Sample/shift timing
- 0 = Sample on leading edge, shift on trailing edge
- 1 = Shift on leading edge, sample on trailing edge

### Clock Speed Selection

| Speed | Frequency | Use Case |
|-------|-----------|----------|
| 100 kHz | 100,000 Hz | Slow sensors, long wires |
| 500 kHz | 500,000 Hz | General purpose |
| 1 MHz | 1,000,000 Hz | Default, most devices |
| 10 MHz | 10,000,000 Hz | Fast sensors, displays |
| 50 MHz | 50,000,000 Hz | Flash memory, high-speed |

**Factors affecting maximum speed**:
- Wire length (shorter = faster)
- Device capability (check datasheet)
- Capacitance and noise
- Voltage levels

## Best Practices

1. **Use correct SPI mode for your device**:
   ```rust
   // Always check device datasheet for CPOL/CPHA requirements
   spi.configure_device(0, 1_000_000, SpiMessage::MODE_0);  // Most common
   ```

2. **Start with lower clock speeds**:
   ```rust
   // Start slow (1MHz) and increase if stable
   spi.set_default_speed(1_000_000);  // 1 MHz
   // If working well, try higher speeds
   // spi.set_default_speed(10_000_000);  // 10 MHz
   ```

3. **Keep wires short and shielded**:
   - Use <15cm wires for speeds >10MHz
   - Use twisted pairs for MOSI/MISO with ground
   - Avoid running SPI wires parallel to noisy signals

4. **Check device voltage levels**:
   ```rust
   // Raspberry Pi GPIOs are 3.3V
   // Some devices require level shifters for 5V logic
   ```

5. **Use pull-up/pull-down resistors**:
   ```
   - CS lines: 10kΩ pull-up to ensure inactive (high) when not driven
   - MISO: 10kΩ pull-down for multi-device buses
   ```

6. **Handle CS timing correctly**:
   ```rust
   // spidev automatically handles CS timing
   // For manual control, use GPIO and delay appropriately
   ```

7. **Verify communication with simple commands**:
   ```rust
   // Read device ID or WHO_AM_I register first
   let id_msg = SpiMessage::new(0, 0, &[0x80 | 0x00, 0x00]);
   // Verify expected response before proceeding
   ```

8. **Use per-device configuration**:
   ```rust
   // Configure each device with optimal settings
   spi.configure_device(0, 5_000_000, SpiMessage::MODE_3);   // ADXL345
   spi.configure_device(1, 10_000_000, SpiMessage::MODE_0);  // BME280
   ```

## Common Device Examples

### ADXL345 Accelerometer

- **Mode**: 3 (CPOL=1, CPHA=1)
- **Speed**: 5 MHz max
- **Wiring**: 3.3V or 5V supply
- **Read format**: Bit 7 = R/W (1=read), Bit 6 = MB (1=multi-byte)

### MCP3008 ADC

- **Mode**: 0 (CPOL=0, CPHA=0)
- **Speed**: 1.35 MHz max (at 2.7V), 3.6 MHz (at 5V)
- **Data**: 10-bit resolution
- **Format**: Start bit + SGL/DIFF + D2/D1/D0 + receive 10 bits

### nRF24L01+ RF Module

- **Mode**: 0 (CPOL=0, CPHA=0)
- **Speed**: 10 MHz max
- **Commands**: Read (0x00-0x1F), Write (0x20-0x3F)
- **Additional pins**: CE (chip enable), IRQ (interrupt)

### W25Q Flash Memory

- **Mode**: 0 or 3
- **Speed**: 50-104 MHz (device dependent)
- **Commands**: Read (0x03), Fast Read (0x0B), Page Program (0x02)
- **Important**: Write Enable (0x06) before any write operation

## Troubleshooting

### "Hardware unavailable - using SIMULATION mode"

```
[WARN] SpiBusNode: Hardware unavailable - using SIMULATION mode
[WARN]   Tried: /dev/spidev0.0
[WARN]   Error: Permission denied
```

**Solutions:**
1. Check SPI device exists:
   ```bash
   ls -l /dev/spidev*
   ```

2. Enable SPI interface:
   ```bash
   sudo raspi-config
   # Interface Options -> SPI -> Enable
   ```

3. Check permissions:
   ```bash
   ls -l /dev/spidev0.0
   # Should show: crw-rw---- 1 root spi
   ```

4. Add user to spi group:
   ```bash
   sudo usermod -a -G spi $USER
   sudo reboot
   ```

5. Verify group membership:
   ```bash
   groups
   # Should include: spi
   ```

6. Rebuild with SPI hardware support:
   ```bash
   cargo build --features="spi-hardware"
   ```

### Reading all zeros or 0xFF

**Solutions:**
1. Check wiring - MOSI and MISO are often swapped:
   ```
   Device MOSI <--> RPi MOSI (GPIO 10)
   Device MISO <--> RPi MISO (GPIO 9)
   ```

2. Verify SPI mode matches device datasheet:
   ```rust
   spi.set_default_mode(SpiMessage::MODE_0);  // Try different modes
   ```

3. Check clock speed (too fast can cause errors):
   ```rust
   spi.set_default_speed(500_000);  // Start slow
   ```

4. Verify device power:
   ```bash
   # Measure VCC and GND with multimeter
   # Check current draw is reasonable
   ```

5. Check CS signal:
   ```
   - Ensure CS goes low during transaction
   - Verify correct CS line for device
   ```

### Incorrect data or corruption

**Solutions:**
1. Lower clock speed:
   ```rust
   spi.set_default_speed(1_000_000);  // Reduce from 10MHz to 1MHz
   ```

2. Shorten wires:
   ```
   Use <10cm wires for high speeds
   Twist MOSI/MISO pairs together
   ```

3. Add ground wire between devices:
   ```
   Connect device GND directly to RPi GND
   Use star ground topology
   ```

4. Check for electrical noise:
   ```
   Separate SPI wires from PWM/motor wires
   Add 0.1μF capacitor near device VCC/GND
   ```

5. Verify bit order (MSB/LSB first):
   ```rust
   spi.set_lsb_first(0, false);  // MSB first (default)
   // Some devices need LSB first
   ```

### Device not responding

**Solutions:**
1. Check device address/CS line:
   ```rust
   // Ensure correct chip select
   let msg = SpiMessage::new(0, 0, &data);  // CS 0
   ```

2. Verify device reset/initialization:
   ```
   Some devices need power-on delay
   Check for reset pin requirement
   ```

3. Check register addresses:
   ```rust
   // Read/write bit in MSB for some devices
   let read_reg = 0x80 | register_addr;
   let write_reg = 0x00 | register_addr;
   ```

4. Test with known-good command:
   ```rust
   // Read device ID or WHO_AM_I register
   // Compare with datasheet expected value
   ```

### Transaction errors

Check transaction statistics:
```rust
let (tx_count, error_count, bytes_tx, bytes_rx) = spi.get_statistics();
if error_count > 0 {
    println!("Error rate: {:.1}%", (error_count as f32 / tx_count as f32) * 100.0);
}
```

## Simulation Mode

When hardware is unavailable, the node operates in simulation mode:

```
[DEBUG] SPI0.0: TX 3 bytes @ 1000000Hz mode=0 | [01, 80, 00]
[DEBUG] SPI0.0 (SIM): RX 3 bytes | [01, 80, 00]
```

Simulated behavior:
- Returns loopback data (TX data = RX data)
- Always reports success
- Useful for logic testing without hardware
- Allows development without physical devices

To disable simulation warnings:
```rust
// Simulation is automatic fallback
// Check success flag in responses
if msg.success && hardware_enabled {
    // Process real data
}
```

## Performance Considerations

### Transaction Overhead

Each SPI transaction includes:
- Message serialization/deserialization
- Hub publish/subscribe
- Device configuration
- CS assertion/deassertion

**For high-throughput applications**:
```rust
// Batch multiple bytes in single transaction
let msg = SpiMessage::new(0, 0, &[reg, 0, 0, 0, 0, 0]);  // Read 5 bytes at once

// Use lower-level transfer() for minimal overhead
spi.transfer(&mut msg, None);
```

### Maximum Throughput

Theoretical maximum (50 MHz, 8-bit):
- 50,000,000 bits/sec ÷ 8 = 6.25 MB/sec

Practical limits:
- Hub messaging overhead: ~100 µs per transaction
- spidev system calls: ~10-50 µs
- Real-world: 1-2 MB/sec sustained

### Optimization Tips

1. **Minimize transactions**: Batch reads/writes
2. **Use higher clock speeds**: Where stable
3. **Reduce logging**: Use DEBUG level only when needed
4. **Pipeline requests**: Send multiple before waiting for responses

## See Also

- [I2cBusNode](../i2c_bus/) - I2C communication for low-speed devices
- [CanBusNode](../can_bus/) - CAN bus for automotive/industrial
- [SerialNode](../serial/) - UART communication
- [GpioNode](../gpio/) - Direct GPIO control
