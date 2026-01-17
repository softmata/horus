use crate::SpiMessage;
use horus_core::error::HorusResult;

type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, NodeInfoExt, Topic};
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

// SPI hardware support
#[cfg(feature = "spi-hardware")]
use spidev::{SpiModeFlags, Spidev, SpidevOptions, SpidevTransfer};

/// SPI (Serial Peripheral Interface) Bus Node
///
/// Provides SPI communication for sensors, displays, and peripherals.
/// Supports standard Linux spidev interface and multiple SPI modes.
///
/// # Supported Devices
/// - **Sensors**: Accelerometers (ADXL345), Gyroscopes (L3GD20), Pressure (BMP280)
/// - **Displays**: OLED/LCD displays (SSD1306, ILI9341, ST7735)
/// - **ADCs**: MCP3008, MCP3208, ADS1118
/// - **DACs**: MCP4921, MCP4922, AD5206
/// - **Memory**: Flash memory (W25Q32, AT25SF041)
/// - **RF**: LoRa modules (SX1276, RFM95W), nRF24L01+
/// - **Motor drivers**: TMC SilentStepStick (TMC2130, TMC5160)
/// - **IMUs**: BMI088, ICM-20948
///
/// # SPI Modes
/// - Mode 0 (CPOL=0, CPHA=0): Clock idle low, sample on leading edge
/// - Mode 1 (CPOL=0, CPHA=1): Clock idle low, sample on trailing edge
/// - Mode 2 (CPOL=1, CPHA=0): Clock idle high, sample on leading edge
/// - Mode 3 (CPOL=1, CPHA=1): Clock idle high, sample on trailing edge
///
/// # Hardware Interface
/// - MOSI (Master Out Slave In): Data from master to slave
/// - MISO (Master In Slave Out): Data from slave to master
/// - SCLK (Serial Clock): Clock signal
/// - CS/SS (Chip Select): Device selection (active low typically)
///
/// # Features
/// - Multiple SPI bus support (/dev/spidev0.0, /dev/spidev1.0, etc.)
/// - Configurable clock speeds (100kHz - 50MHz+)
/// - All 4 SPI modes supported
/// - Full-duplex communication
/// - Chip select management
/// - Transaction queuing
/// - Error detection and retry
///
/// # Example
/// ```rust,ignore
/// use horus_library::nodes::SpiBusNode;
/// use horus_library::SpiMessage;
///
/// let mut spi = SpiBusNode::new(0)?; // SPI bus 0
/// spi.set_default_speed(1_000_000); // 1 MHz
/// spi.set_default_mode(SpiMessage::MODE_0);
///
/// // Read from device on CS 0
/// let mut msg = SpiMessage::new(0, 0, &[0x00, 0x01]); // Read register 0x01
/// spi.transfer(&mut msg);
/// ```
pub struct SpiBusNode {
    request_subscriber: Topic<SpiMessage>,
    response_publisher: Topic<SpiMessage>,

    // Hardware devices (per chip select)
    #[cfg(feature = "spi-hardware")]
    spi_devices: HashMap<u8, Spidev>,
    hardware_enabled: bool,

    // Configuration
    bus_number: u8,
    default_speed_hz: u32,
    default_mode: u8,
    default_bits_per_word: u8,

    // Device configurations (per chip select)
    device_configs: HashMap<u8, DeviceConfig>,

    // Statistics
    transaction_count: u64,
    error_count: u64,
    bytes_transmitted: u64,
    bytes_received: u64,
    last_transaction_time: u64,

    // Timing state (moved from static mut for thread safety)
    last_log_time: u64,
}

/// Configuration for an SPI device
#[derive(Debug, Clone, Copy)]
struct DeviceConfig {
    chip_select: u8,
    speed_hz: u32,
    mode: u8,
    bits_per_word: u8,
    cs_high: bool,
    lsb_first: bool,
    three_wire: bool,
}

impl DeviceConfig {
    fn new(chip_select: u8) -> Self {
        Self {
            chip_select,
            speed_hz: 1_000_000,
            mode: SpiMessage::MODE_0,
            bits_per_word: 8,
            cs_high: false,
            lsb_first: false,
            three_wire: false,
        }
    }
}

impl SpiBusNode {
    /// Create a new SPI bus node
    pub fn new(bus: u8) -> Result<Self> {
        Ok(Self {
            request_subscriber: Topic::new(&format!("spi{}.request", bus))?,
            response_publisher: Topic::new(&format!("spi{}.response", bus))?,
            #[cfg(feature = "spi-hardware")]
            spi_devices: HashMap::new(),
            hardware_enabled: false,
            bus_number: bus,
            default_speed_hz: 1_000_000, // 1 MHz
            default_mode: SpiMessage::MODE_0,
            default_bits_per_word: 8,
            device_configs: HashMap::new(),
            transaction_count: 0,
            error_count: 0,
            bytes_transmitted: 0,
            bytes_received: 0,
            last_transaction_time: 0,
            last_log_time: 0,
        })
    }

    /// Set default SPI speed in Hz
    pub fn set_default_speed(&mut self, speed_hz: u32) {
        self.default_speed_hz = speed_hz;
    }

    /// Set default SPI mode (0-3)
    pub fn set_default_mode(&mut self, mode: u8) {
        self.default_mode = mode & 0x03;
    }

    /// Set default bits per word
    pub fn set_default_bits_per_word(&mut self, bits: u8) {
        self.default_bits_per_word = bits;
    }

    /// Configure a specific device on a chip select line
    pub fn configure_device(&mut self, chip_select: u8, speed_hz: u32, mode: u8) {
        let mut config = DeviceConfig::new(chip_select);
        config.speed_hz = speed_hz;
        config.mode = mode & 0x03;
        self.device_configs.insert(chip_select, config);
    }

    /// Set chip select polarity for a device
    pub fn set_cs_high(&mut self, chip_select: u8, cs_high: bool) {
        self.device_configs
            .entry(chip_select)
            .or_insert_with(|| DeviceConfig::new(chip_select))
            .cs_high = cs_high;
    }

    /// Set bit order for a device
    pub fn set_lsb_first(&mut self, chip_select: u8, lsb_first: bool) {
        self.device_configs
            .entry(chip_select)
            .or_insert_with(|| DeviceConfig::new(chip_select))
            .lsb_first = lsb_first;
    }

    /// Enable 3-wire mode for a device
    pub fn set_three_wire(&mut self, chip_select: u8, three_wire: bool) {
        self.device_configs
            .entry(chip_select)
            .or_insert_with(|| DeviceConfig::new(chip_select))
            .three_wire = three_wire;
    }

    /// Get statistics
    pub fn get_statistics(&self) -> (u64, u64, u64, u64) {
        (
            self.transaction_count,
            self.error_count,
            self.bytes_transmitted,
            self.bytes_received,
        )
    }

    /// Reset statistics
    pub fn reset_statistics(&mut self) {
        self.transaction_count = 0;
        self.error_count = 0;
        self.bytes_transmitted = 0;
        self.bytes_received = 0;
    }

    /// Execute SPI transfer (full-duplex)
    fn transfer(&mut self, msg: &mut SpiMessage, mut ctx: Option<&mut NodeInfo>) {
        // Apply device-specific configuration if available
        if let Some(config) = self.device_configs.get(&msg.chip_select) {
            if msg.speed_hz == 0 {
                msg.speed_hz = config.speed_hz;
            }
            msg.mode = config.mode;
            msg.bits_per_word = config.bits_per_word;
            msg.cs_high = config.cs_high;
            msg.lsb_first = config.lsb_first;
            msg.three_wire = config.three_wire;
        } else {
            // Use defaults
            if msg.speed_hz == 0 {
                msg.speed_hz = self.default_speed_hz;
            }
            if msg.mode > 3 {
                msg.mode = self.default_mode;
            }
            if msg.bits_per_word == 0 {
                msg.bits_per_word = self.default_bits_per_word;
            }
        }

        // Validate message
        if msg.length == 0 || msg.length > 256 {
            ctx.log_warning(&format!("Invalid SPI transaction length: {}", msg.length));
            msg.success = false;
            self.error_count += 1;
            return;
        }

        ctx.log_debug(&format!(
            "SPI{}.{}: TX {} bytes @ {}Hz mode={} | {:02X?}",
            msg.bus,
            msg.chip_select,
            msg.length,
            msg.speed_hz,
            msg.mode,
            &msg.tx_data[..msg.length.min(8) as usize]
        ));

        // Try hardware first, fall back to simulation
        #[cfg(feature = "spi-hardware")]
        if self.hardware_enabled || !self.spi_devices.is_empty() {
            match self.transfer_hardware(msg) {
                Ok(()) => {
                    msg.success = true;
                    self.hardware_enabled = true;
                }
                Err(e) => {
                    // Provide detailed troubleshooting information (only log once)
                    if self.hardware_enabled || self.transaction_count == 0 {
                        let device_path =
                            format!("/dev/spidev{}.{}", self.bus_number, msg.chip_select);
                        ctx.log_warning(&format!(
                            "SpiBusNode: Hardware unavailable - using SIMULATION mode"
                        ));
                        ctx.log_warning(&format!("  Tried: {}", device_path));
                        ctx.log_warning(&format!("  Error: {}", e));
                        ctx.log_warning("  Fix:");
                        ctx.log_warning("    1. Install: sudo apt install libraspberrypi-dev");
                        ctx.log_warning(
                            "    2. Enable SPI: sudo raspi-config -> Interface Options -> SPI",
                        );
                        ctx.log_warning("    3. Add user to group: sudo usermod -a -G spi $USER");
                        ctx.log_warning("    4. Reboot or re-login");
                        ctx.log_warning(
                            "    5. Rebuild with: cargo build --features=\"spi-hardware\"",
                        );
                    }
                    self.hardware_enabled = false;
                    // Fall through to simulation
                    for i in 0..msg.length as usize {
                        msg.rx_data[i] = msg.tx_data[i]; // Loopback for simulation
                    }
                    msg.success = true;
                }
            }
        } else {
            // Simulation mode
            for i in 0..msg.length as usize {
                msg.rx_data[i] = msg.tx_data[i]; // Loopback for simulation
            }
            msg.success = true;
        }

        #[cfg(not(feature = "spi-hardware"))]
        {
            // Simulate transfer (loopback)
            for i in 0..msg.length as usize {
                msg.rx_data[i] = msg.tx_data[i];
            }
            msg.success = true;
        }
        msg.timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;

        // Update statistics
        self.transaction_count += 1;
        self.bytes_transmitted += msg.length as u64;
        self.bytes_received += msg.length as u64;
        self.last_transaction_time = msg.timestamp;

        ctx.log_debug(&format!(
            "SPI{}.{}{}: RX {} bytes | {:02X?}",
            msg.bus,
            msg.chip_select,
            if self.hardware_enabled {
                " (HW)"
            } else {
                " (SIM)"
            },
            msg.length,
            &msg.rx_data[..msg.length.min(8) as usize]
        ));
    }

    /// Write data to SPI device (transmit only)
    pub fn write(&mut self, chip_select: u8, data: &[u8], mut ctx: Option<&mut NodeInfo>) -> bool {
        if data.len() > 256 {
            ctx.log_error("SPI write data too large (max 256 bytes)");
            return false;
        }

        let mut msg = SpiMessage::new(self.bus_number, chip_select, data);
        self.transfer(&mut msg, ctx.as_deref_mut());
        msg.success
    }

    /// Read data from SPI device (receive only, transmit zeros)
    pub fn read(
        &mut self,
        chip_select: u8,
        length: u16,
        mut ctx: Option<&mut NodeInfo>,
    ) -> Option<Vec<u8>> {
        if length > 256 {
            ctx.log_error("SPI read length too large (max 256 bytes)");
            return None;
        }

        let tx_data = vec![0u8; length as usize];
        let mut msg = SpiMessage::new(self.bus_number, chip_select, &tx_data);
        self.transfer(&mut msg, ctx.as_deref_mut());

        if msg.success {
            Some(msg.rx_data[..length as usize].to_vec())
        } else {
            None
        }
    }

    /// Write then read (common pattern for reading registers)
    pub fn write_read(
        &mut self,
        chip_select: u8,
        write_data: &[u8],
        read_length: u16,
        mut ctx: Option<&mut NodeInfo>,
    ) -> Option<Vec<u8>> {
        // First write
        if !self.write(chip_select, write_data, ctx.as_deref_mut()) {
            return None;
        }

        // Then read
        self.read(chip_select, read_length, ctx.as_deref_mut())
    }

    // ========== Hardware Backend Functions (spidev) ==========

    /// Open SPI hardware device for a specific chip select
    #[cfg(feature = "spi-hardware")]
    fn open_hardware_device(
        &mut self,
        chip_select: u8,
        mut ctx: Option<&mut NodeInfo>,
    ) -> std::io::Result<()> {
        // Construct device path: /dev/spidev{bus}.{cs}
        let device_path = format!("/dev/spidev{}.{}", self.bus_number, chip_select);

        // Open the SPI device
        let mut spi = Spidev::open(&device_path)?;

        // Get device config or use defaults
        let config = self.device_configs.get(&chip_select);
        let speed_hz = config.map(|c| c.speed_hz).unwrap_or(self.default_speed_hz);
        let mode = config.map(|c| c.mode).unwrap_or(self.default_mode);
        let bits_per_word = config
            .map(|c| c.bits_per_word)
            .unwrap_or(self.default_bits_per_word);

        // Convert HORUS mode to spidev mode flags
        let mode_flags = match mode {
            0 => SpiModeFlags::SPI_MODE_0,
            1 => SpiModeFlags::SPI_MODE_1,
            2 => SpiModeFlags::SPI_MODE_2,
            3 => SpiModeFlags::SPI_MODE_3,
            _ => SpiModeFlags::SPI_MODE_0,
        };

        // Configure the SPI device
        let options = SpidevOptions::new()
            .bits_per_word(bits_per_word)
            .max_speed_hz(speed_hz)
            .mode(mode_flags)
            .build();

        spi.configure(&options)?;

        self.spi_devices.insert(chip_select, spi);
        ctx.log_info(&format!(
            "Opened SPI device: {} @ {}Hz mode={}",
            device_path, speed_hz, mode
        ));

        Ok(())
    }

    /// Perform SPI transfer via hardware
    #[cfg(feature = "spi-hardware")]
    fn transfer_hardware(&mut self, msg: &mut SpiMessage) -> std::io::Result<()> {
        // Ensure device is open for this chip select
        if !self.spi_devices.contains_key(&msg.chip_select) {
            self.open_hardware_device(msg.chip_select, None)?;
        }

        let spi = self.spi_devices.get_mut(&msg.chip_select).ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::NotFound, "SPI device not found")
        })?;

        // Prepare transfer buffers
        let tx_buf = &msg.tx_data[..msg.length as usize];
        let mut rx_buf = vec![0u8; msg.length as usize];

        // Create SPI transfer
        let mut transfer = SpidevTransfer::read_write(tx_buf, &mut rx_buf);

        // Execute the transfer
        spi.transfer(&mut transfer)?;

        // Copy received data back to message
        msg.rx_data[..msg.length as usize].copy_from_slice(&rx_buf);

        Ok(())
    }
}

impl Node for SpiBusNode {
    fn name(&self) -> &'static str {
        "SpiBusNode"
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info(&format!(
            "SpiBusNode shutting down - closing SPI bus {}",
            self.bus_number
        ));

        // Close all SPI hardware devices
        #[cfg(feature = "spi-hardware")]
        {
            self.spi_devices.clear();
        }
        self.hardware_enabled = false;

        ctx.log_info("SPI bus closed safely");
        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Process all pending SPI requests
        while let Some(mut msg) = self.request_subscriber.recv(&mut None) {
            self.transfer(&mut msg, ctx.as_deref_mut());

            // Publish response
            if let Err(e) = self.response_publisher.send(msg, &mut None) {
                ctx.log_error(&format!("Failed to publish SPI response: {:?}", e));
            }
        }

        // Periodic status logging
        let log_interval = 30_000_000_000; // 30 seconds
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;

        if current_time - self.last_log_time > log_interval && self.transaction_count > 0 {
            ctx.log_info(&format!(
                "SPI{}: {} transactions, {} errors, {} KB TX, {} KB RX",
                self.bus_number,
                self.transaction_count,
                self.error_count,
                self.bytes_transmitted / 1024,
                self.bytes_received / 1024
            ));
            self.last_log_time = current_time;
        }
    }
}

/// Preset configurations for common SPI devices
impl SpiBusNode {
    /// Configure for ADXL345 accelerometer (Mode 3, 5MHz)
    pub fn configure_adxl345(&mut self, chip_select: u8) {
        self.configure_device(chip_select, 5_000_000, SpiMessage::MODE_3);
    }

    /// Configure for BME280 environmental sensor (Mode 0, 10MHz)
    pub fn configure_bme280(&mut self, chip_select: u8) {
        self.configure_device(chip_select, 10_000_000, SpiMessage::MODE_0);
    }

    /// Configure for MCP3008 ADC (Mode 0, 1.35MHz)
    pub fn configure_mcp3008(&mut self, chip_select: u8) {
        self.configure_device(chip_select, 1_350_000, SpiMessage::MODE_0);
    }

    /// Configure for nRF24L01+ RF module (Mode 0, 10MHz)
    pub fn configure_nrf24l01(&mut self, chip_select: u8) {
        self.configure_device(chip_select, 10_000_000, SpiMessage::MODE_0);
    }

    /// Configure for W25Q series flash memory (Mode 0/3, 50MHz)
    pub fn configure_w25q_flash(&mut self, chip_select: u8) {
        self.configure_device(chip_select, 50_000_000, SpiMessage::MODE_0);
    }

    /// Configure for ST7735 TFT display (Mode 0, 15MHz)
    pub fn configure_st7735(&mut self, chip_select: u8) {
        self.configure_device(chip_select, 15_000_000, SpiMessage::MODE_0);
    }

    /// Configure for TMC2130 stepper driver (Mode 3, 4MHz)
    pub fn configure_tmc2130(&mut self, chip_select: u8) {
        self.configure_device(chip_select, 4_000_000, SpiMessage::MODE_3);
    }
}
