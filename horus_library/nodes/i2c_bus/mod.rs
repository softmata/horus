use crate::I2cMessage;
use horus_core::error::HorusResult;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Hub, Node, NodeInfo, NodeInfoExt};
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

// I2C hardware support
#[cfg(feature = "i2c-hardware")]
use i2cdev::core::I2CDevice;
#[cfg(feature = "i2c-hardware")]
use i2cdev::linux::LinuxI2CDevice;

/// I2C Bus Communication Node
///
/// Handles I2C bus communication with devices like sensors, displays,
/// EEPROMs, and other I2C peripherals. Supports multiple I2C buses
/// and device addressing.
pub struct I2cBusNode {
    request_subscriber: Hub<I2cMessage>, // I2C transaction requests
    response_publisher: Hub<I2cMessage>, // I2C transaction responses

    // Hardware devices (per I2C address)
    #[cfg(feature = "i2c-hardware")]
    i2c_devices: HashMap<u16, LinuxI2CDevice>,
    hardware_enabled: bool,

    // Configuration
    bus_number: u8,
    clock_speed: u32,
    retry_count: u8,
    timeout_ms: u64,

    // State tracking
    device_addresses: HashMap<u16, String>, // Known devices
    transactions_total: u64,
    transactions_successful: u64,
    transactions_failed: u64,
    last_error_code: u8,

    // Simulation state
    sim_devices: HashMap<u16, Vec<u8>>, // Simulated device memory
}

impl I2cBusNode {
    /// Create a new I2C bus node with default configuration
    pub fn new() -> Result<Self> {
        Self::new_with_config(1, "i2c.request", "i2c.response")
    }

    /// Create with custom bus number and topics
    pub fn new_with_config(
        bus_number: u8,
        request_topic: &str,
        response_topic: &str,
    ) -> Result<Self> {
        Ok(Self {
            request_subscriber: Hub::new(request_topic)?,
            response_publisher: Hub::new(response_topic)?,
            #[cfg(feature = "i2c-hardware")]
            i2c_devices: HashMap::new(),
            hardware_enabled: false,
            bus_number,
            clock_speed: I2cMessage::SPEED_STANDARD,
            retry_count: 3,
            timeout_ms: 100,
            device_addresses: HashMap::new(),
            transactions_total: 0,
            transactions_successful: 0,
            transactions_failed: 0,
            last_error_code: 0,
            sim_devices: HashMap::new(),
        })
    }

    /// Set I2C bus number (0, 1, 2, etc.)
    pub fn set_bus_number(&mut self, bus: u8) {
        self.bus_number = bus;
    }

    /// Set I2C clock speed (100kHz, 400kHz, 1MHz)
    pub fn set_clock_speed(&mut self, speed: u32) {
        self.clock_speed = speed;
    }

    /// Set number of retries on failure
    pub fn set_retry_count(&mut self, retries: u8) {
        self.retry_count = retries;
    }

    /// Set transaction timeout in milliseconds
    pub fn set_timeout(&mut self, timeout_ms: u64) {
        self.timeout_ms = timeout_ms;
    }

    /// Register a known device with label
    pub fn register_device(&mut self, address: u16, label: &str) {
        self.device_addresses.insert(address, label.to_string());
    }

    /// Get transaction statistics
    pub fn get_stats(&self) -> (u64, u64, u64, u8) {
        (
            self.transactions_total,
            self.transactions_successful,
            self.transactions_failed,
            self.last_error_code,
        )
    }

    /// Add simulated I2C device (for testing)
    pub fn add_simulated_device(&mut self, address: u16, memory_size: usize) {
        self.sim_devices.insert(address, vec![0u8; memory_size]);
    }

    /// Execute I2C transaction
    fn execute_transaction(
        &mut self,
        mut request: I2cMessage,
        mut ctx: Option<&mut NodeInfo>,
    ) -> I2cMessage {
        self.transactions_total += 1;
        request.bus_number = self.bus_number;
        request.clock_speed = self.clock_speed;

        // Log transaction
        if let Some(label) = self.device_addresses.get(&request.device_address) {
            ctx.log_debug(&format!(
                "I2C transaction to device {} (0x{:02X})",
                label, request.device_address
            ));
        } else {
            ctx.log_debug(&format!(
                "I2C transaction to device 0x{:02X}",
                request.device_address
            ));
        }

        // Try hardware first, fall back to simulation
        #[cfg(feature = "i2c-hardware")]
        if self.hardware_enabled || !self.i2c_devices.is_empty() {
            match self.execute_hardware(&mut request) {
                Ok(()) => {
                    request.success = true;
                    request.error_code = 0;
                    self.hardware_enabled = true;
                    // Skip simulation, return early after stats update
                    if request.success {
                        self.transactions_successful += 1;
                    } else {
                        self.transactions_failed += 1;
                        self.last_error_code = request.error_code;
                    }
                    request.timestamp = SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap()
                        .as_nanos() as u64;
                    return request;
                }
                Err(e) => {
                    // Provide detailed troubleshooting information (only log once)
                    if self.hardware_enabled || self.transactions_total == 1 {
                        let device_path = format!("/dev/i2c-{}", self.bus_number);
                        ctx.log_warning(&format!(
                            "I2cBusNode: Hardware unavailable - using SIMULATION mode"
                        ));
                        ctx.log_warning(&format!("  Tried: {}", device_path));
                        ctx.log_warning(&format!("  Error: {}", e));
                        ctx.log_warning("  Fix:");
                        ctx.log_warning("    1. Install: sudo apt install i2c-tools");
                        ctx.log_warning(
                            "    2. Enable I2C: sudo raspi-config -> Interface Options -> I2C",
                        );
                        ctx.log_warning("    3. Add user to group: sudo usermod -a -G i2c $USER");
                        ctx.log_warning("    4. Reboot or re-login");
                        ctx.log_warning(
                            "    5. Rebuild with: cargo build --features=\"i2c-hardware\"",
                        );
                    }
                    self.hardware_enabled = false;
                    // Fall through to simulation
                }
            }
        }

        // Simulation fallback
        if let Some(device_memory) = self.sim_devices.get_mut(&request.device_address) {
            match request.transaction_type {
                I2cMessage::TYPE_READ => {
                    // Read data from simulated device
                    let len = request.data_length.min(device_memory.len() as u8);
                    request.data[..len as usize].copy_from_slice(&device_memory[..len as usize]);
                    request.success = true;
                    request.error_code = 0;
                }
                I2cMessage::TYPE_WRITE => {
                    // Write data to simulated device
                    let len = request.data_length.min(device_memory.len() as u8);
                    device_memory[..len as usize].copy_from_slice(&request.data[..len as usize]);
                    request.success = true;
                    request.error_code = 0;
                }
                I2cMessage::TYPE_READ_REGISTER => {
                    // Read from specific register
                    let reg = request.register_address as usize;
                    let len = request.data_length as usize;
                    if reg + len <= device_memory.len() {
                        request.data[..len].copy_from_slice(&device_memory[reg..reg + len]);
                        request.success = true;
                        request.error_code = 0;
                    } else {
                        request.success = false;
                        request.error_code = 1; // Out of bounds
                    }
                }
                I2cMessage::TYPE_WRITE_REGISTER => {
                    // Write to specific register
                    let reg = request.register_address as usize;
                    let len = request.data_length as usize;
                    if reg + len <= device_memory.len() {
                        device_memory[reg..reg + len].copy_from_slice(&request.data[..len]);
                        request.success = true;
                        request.error_code = 0;
                    } else {
                        request.success = false;
                        request.error_code = 1; // Out of bounds
                    }
                }
                _ => {
                    request.success = false;
                    request.error_code = 2; // Unknown transaction type
                }
            }
        } else {
            // No simulated device - would access real hardware here
            // For now, return error
            request.success = false;
            request.error_code = 3; // Device not found
        }

        // Update statistics
        if request.success {
            self.transactions_successful += 1;
        } else {
            self.transactions_failed += 1;
            self.last_error_code = request.error_code;

            ctx.log_warning(&format!(
                "I2C transaction failed: error code {}",
                request.error_code
            ));
        }

        request.timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        request
    }

    /// Scan I2C bus for devices
    pub fn scan_bus(&mut self, mut ctx: Option<&mut NodeInfo>) -> Vec<u16> {
        let mut found_devices = Vec::new();

        ctx.log_info("Scanning I2C bus for devices...");

        // Scan 7-bit address space (0x08-0x77)
        for addr in 0x08..=0x77 {
            // Try to read from device
            let request = I2cMessage::read(self.bus_number, addr, 1);
            let response = self.execute_transaction(request, None);

            if response.success {
                found_devices.push(addr);
                ctx.log_info(&format!("Found I2C device at 0x{:02X}", addr));
            }
        }

        ctx.log_info(&format!(
            "I2C scan complete: {} devices found",
            found_devices.len()
        ));

        found_devices
    }

    // ========== Hardware Backend Functions (i2cdev) ==========

    /// Open I2C hardware device for a specific address
    #[cfg(feature = "i2c-hardware")]
    fn open_hardware_device(
        &mut self,
        address: u16,
        mut ctx: Option<&mut NodeInfo>,
    ) -> std::io::Result<()> {
        // Construct device path: /dev/i2c-{bus}
        let device_path = format!("/dev/i2c-{}", self.bus_number);

        // Open the I2C device for this address
        let dev = LinuxI2CDevice::new(&device_path, address)?;

        self.i2c_devices.insert(address, dev);
        ctx.log_info(&format!(
            "Opened I2C device: {} address 0x{:02X}",
            device_path, address
        ));

        Ok(())
    }

    /// Execute transaction via hardware
    #[cfg(feature = "i2c-hardware")]
    fn execute_hardware(&mut self, request: &mut I2cMessage) -> std::io::Result<()> {
        // Ensure device is open for this address
        if !self.i2c_devices.contains_key(&request.device_address) {
            self.open_hardware_device(request.device_address, None)?;
        }

        let dev = self
            .i2c_devices
            .get_mut(&request.device_address)
            .ok_or_else(|| {
                std::io::Error::new(std::io::ErrorKind::NotFound, "I2C device not found")
            })?;

        // Execute based on transaction type
        match request.transaction_type {
            I2cMessage::TYPE_READ => {
                // Simple read from device
                let mut buf = vec![0u8; request.data_length as usize];
                dev.read(&mut buf)?;
                request.data[..buf.len()].copy_from_slice(&buf);
            }
            I2cMessage::TYPE_WRITE => {
                // Simple write to device
                let data = &request.data[..request.data_length as usize];
                dev.write(data)?;
            }
            I2cMessage::TYPE_READ_REGISTER => {
                // Write register address, then read data
                let reg_addr = [request.register_address];
                dev.write(&reg_addr)?;

                let mut buf = vec![0u8; request.data_length as usize];
                dev.read(&mut buf)?;
                request.data[..buf.len()].copy_from_slice(&buf);
            }
            I2cMessage::TYPE_WRITE_REGISTER => {
                // Write register address followed by data
                let mut write_buf = vec![request.register_address];
                write_buf.extend_from_slice(&request.data[..request.data_length as usize]);
                dev.write(&write_buf)?;
            }
            _ => {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    "Unknown transaction type",
                ));
            }
        }

        Ok(())
    }
}

impl Node for I2cBusNode {
    fn name(&self) -> &'static str {
        "I2cBusNode"
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info(&format!(
            "I2cBusNode shutting down - closing I2C bus {}",
            self.bus_number
        ));

        // Close all I2C hardware devices
        #[cfg(feature = "i2c-hardware")]
        {
            self.i2c_devices.clear();
        }
        self.hardware_enabled = false;

        ctx.log_info("I2C bus closed safely");
        Ok(())
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info(&format!(
            "I2C bus {} initialized @ {} Hz",
            self.bus_number, self.clock_speed
        ));
        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Process all pending I2C requests
        while let Some(request) = self.request_subscriber.recv(&mut None) {
            let response = self.execute_transaction(request, ctx.as_deref_mut());

            // Publish response
            let _ = self.response_publisher.send(response, &mut None);
        }
    }
}
