use crate::SerialData;
use horus_core::error::HorusResult;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Hub, Node, NodeInfo, NodeInfoExt};

#[cfg(feature = "serial-hardware")]
use serialport::SerialPort;

#[cfg(feature = "serial-hardware")]
use std::io::{Read, Write};

/// Serial backend type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SerialBackend {
    Simulation,
    Hardware,
}

/// Serial/UART Communication Node
///
/// Handles serial/UART communication with devices like GPS modules,
/// Arduino boards, sensors, and other serial peripherals.
/// Supports various baud rates, data formats, and flow control.
///
/// Supported backends:
/// - Hardware (serialport crate for actual serial communication)
/// - Simulation mode for testing
pub struct SerialNode {
    rx_publisher: Hub<SerialData>,  // Received data
    tx_subscriber: Hub<SerialData>, // Data to transmit

    // Configuration
    port_path: String,
    baud_rate: u32,
    data_bits: u8,
    stop_bits: u8,
    parity: u8,
    flow_control: bool,
    read_timeout_ms: u64,
    backend: SerialBackend,

    // State
    port_open: bool,
    bytes_received: u64,
    bytes_transmitted: u64,
    errors: u64,

    // Hardware driver
    #[cfg(feature = "serial-hardware")]
    serial: Option<Box<dyn SerialPort>>,

    // Buffering (for simulation)
    receive_buffer: Vec<u8>,
}

impl SerialNode {
    /// Create a new serial node with default port and topics in simulation mode
    pub fn new() -> Result<Self> {
        Self::new_with_backend(
            "/dev/ttyUSB0",
            9600,
            "serial.rx",
            "serial.tx",
            SerialBackend::Simulation,
        )
    }

    /// Create a new serial node with custom configuration
    pub fn new_with_config(
        port: &str,
        baud_rate: u32,
        rx_topic: &str,
        tx_topic: &str,
    ) -> Result<Self> {
        Self::new_with_backend(
            port,
            baud_rate,
            rx_topic,
            tx_topic,
            SerialBackend::Simulation,
        )
    }

    /// Create a new serial node with specific backend
    pub fn new_with_backend(
        port: &str,
        baud_rate: u32,
        rx_topic: &str,
        tx_topic: &str,
        backend: SerialBackend,
    ) -> Result<Self> {
        Ok(Self {
            rx_publisher: Hub::new(rx_topic)?,
            tx_subscriber: Hub::new(tx_topic)?,
            port_path: port.to_string(),
            baud_rate,
            data_bits: 8,
            stop_bits: 1,
            parity: SerialData::PARITY_NONE,
            flow_control: false,
            read_timeout_ms: 100,
            backend,
            port_open: false,
            bytes_received: 0,
            bytes_transmitted: 0,
            errors: 0,
            #[cfg(feature = "serial-hardware")]
            serial: None,
            receive_buffer: Vec::new(),
        })
    }

    /// Set serial backend
    pub fn set_backend(&mut self, backend: SerialBackend) {
        self.backend = backend;
        self.port_open = false; // Need to reopen with new backend
    }

    /// Set serial port path
    pub fn set_port(&mut self, port: &str) {
        self.port_path = port.to_string();
    }

    /// Set baud rate
    pub fn set_baud_rate(&mut self, baud_rate: u32) {
        self.baud_rate = baud_rate;
    }

    /// Set data format (data_bits, stop_bits, parity)
    pub fn set_format(&mut self, data_bits: u8, stop_bits: u8, parity: u8) {
        self.data_bits = data_bits;
        self.stop_bits = stop_bits;
        self.parity = parity;
    }

    /// Enable/disable hardware flow control
    pub fn set_flow_control(&mut self, enabled: bool) {
        self.flow_control = enabled;
    }

    /// Set read timeout in milliseconds
    pub fn set_read_timeout(&mut self, timeout_ms: u64) {
        self.read_timeout_ms = timeout_ms;
    }

    /// Check if port is open
    pub fn is_open(&self) -> bool {
        self.port_open
    }

    /// Get statistics
    pub fn get_stats(&self) -> (u64, u64, u64) {
        (self.bytes_received, self.bytes_transmitted, self.errors)
    }

    /// Open the serial port
    fn open_port(&mut self, mut ctx: Option<&mut NodeInfo>) -> bool {
        match self.backend {
            SerialBackend::Simulation => {
                ctx.log_info(&format!(
                    "Opening serial port {} @ {} baud (simulation mode)",
                    self.port_path, self.baud_rate
                ));
                self.port_open = true;
                true
            }
            #[cfg(feature = "serial-hardware")]
            SerialBackend::Hardware => {
                use serialport::available_ports;

                ctx.log_info(&format!(
                    "Opening hardware serial port {} @ {} baud",
                    self.port_path, self.baud_rate
                ));

                // Check if port exists
                if let Ok(ports) = available_ports() {
                    let port_exists = ports.iter().any(|p| p.port_name == self.port_path);
                    if !port_exists {
                        ctx.log_error(&format!("Serial port {} not found", self.port_path));
                        ctx.log_info("Available ports:");
                        for port in ports {
                            ctx.log_info(&format!("  - {}", port.port_name));
                        }
                        ctx.log_warning("Falling back to simulation mode");
                        self.backend = SerialBackend::Simulation;
                        self.port_open = true;
                        return true;
                    }
                }

                // Configure serial port parameters
                let mut builder = serialport::new(&self.port_path, self.baud_rate)
                    .timeout(std::time::Duration::from_millis(self.read_timeout_ms));

                // Set data bits
                builder = match self.data_bits {
                    5 => builder.data_bits(serialport::DataBits::Five),
                    6 => builder.data_bits(serialport::DataBits::Six),
                    7 => builder.data_bits(serialport::DataBits::Seven),
                    8 => builder.data_bits(serialport::DataBits::Eight),
                    _ => builder.data_bits(serialport::DataBits::Eight),
                };

                // Set stop bits
                builder = match self.stop_bits {
                    1 => builder.stop_bits(serialport::StopBits::One),
                    2 => builder.stop_bits(serialport::StopBits::Two),
                    _ => builder.stop_bits(serialport::StopBits::One),
                };

                // Set parity
                builder = match self.parity {
                    SerialData::PARITY_NONE => builder.parity(serialport::Parity::None),
                    SerialData::PARITY_ODD => builder.parity(serialport::Parity::Odd),
                    SerialData::PARITY_EVEN => builder.parity(serialport::Parity::Even),
                    _ => builder.parity(serialport::Parity::None),
                };

                // Set flow control
                builder = if self.flow_control {
                    builder.flow_control(serialport::FlowControl::Hardware)
                } else {
                    builder.flow_control(serialport::FlowControl::None)
                };

                // Open port
                match builder.open() {
                    Ok(port) => {
                        self.serial = Some(port);
                        self.port_open = true;
                        ctx.log_info(&format!(
                            "Hardware serial port {} opened successfully",
                            self.port_path
                        ));
                        true
                    }
                    Err(e) => {
                        ctx.log_error(&format!(
                            "Failed to open serial port {}: {:?}",
                            self.port_path, e
                        ));
                        ctx.log_warning("Falling back to simulation mode");
                        self.backend = SerialBackend::Simulation;
                        self.port_open = true;
                        true
                    }
                }
            }
            #[cfg(not(feature = "serial-hardware"))]
            SerialBackend::Hardware => {
                ctx.log_warning(
                    "Hardware backend requested but serial-hardware feature not enabled",
                );
                ctx.log_warning("Falling back to simulation mode");
                self.backend = SerialBackend::Simulation;
                self.port_open = true;
                true
            }
        }
    }

    /// Close the serial port (reserved for graceful shutdown)
    fn _close_port(&mut self, mut ctx: Option<&mut NodeInfo>) {
        if self.port_open {
            ctx.log_info(&format!("Closing serial port {}", self.port_path));

            #[cfg(feature = "serial-hardware")]
            {
                self.serial = None;
            }

            self.port_open = false;
        }
    }

    /// Read data from serial port
    fn read_serial(&mut self, mut ctx: Option<&mut NodeInfo>) {
        if !self.port_open {
            return;
        }

        match self.backend {
            SerialBackend::Simulation => {
                // For simulation, read from buffer
                if self.receive_buffer.is_empty() {
                    return;
                }

                // Create message with received data
                let mut msg = SerialData::new(&self.port_path);
                msg.baud_rate = self.baud_rate;
                msg.data_bits = self.data_bits;
                msg.stop_bits = self.stop_bits;
                msg.parity = self.parity;

                // Copy data from buffer (up to 1024 bytes)
                let available = self.receive_buffer.len().min(1024);
                if msg.set_data(&self.receive_buffer[..available]) {
                    self.bytes_received += available as u64;
                    self.receive_buffer.drain(..available);

                    // Publish received data
                    let _ = self.rx_publisher.send(msg, &mut None);

                    ctx.log_debug(&format!(
                        "Received {} bytes from serial port (sim)",
                        available
                    ));
                }
            }
            #[cfg(feature = "serial-hardware")]
            SerialBackend::Hardware => {
                if let Some(ref mut port) = self.serial {
                    let mut buffer = [0u8; 1024];

                    match port.read(&mut buffer) {
                        Ok(n) if n > 0 => {
                            // Create message with received data
                            let mut msg = SerialData::new(&self.port_path);
                            msg.baud_rate = self.baud_rate;
                            msg.data_bits = self.data_bits;
                            msg.stop_bits = self.stop_bits;
                            msg.parity = self.parity;

                            if msg.set_data(&buffer[..n]) {
                                self.bytes_received += n as u64;

                                // Publish received data
                                let _ = self.rx_publisher.send(msg, &mut None);

                                ctx.log_debug(&format!(
                                    "Received {} bytes from hardware serial port",
                                    n
                                ));
                            }
                        }
                        Ok(_) => {
                            // No data available (timeout)
                        }
                        Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                            // Timeout is normal, not an error
                        }
                        Err(e) => {
                            self.errors += 1;
                            ctx.log_error(&format!("Serial read error: {:?}", e));
                        }
                    }
                }
            }
            #[cfg(not(feature = "serial-hardware"))]
            SerialBackend::Hardware => {
                // Should not reach here due to fallback in open_port
            }
        }
    }

    /// Write data to serial port
    fn write_serial(&mut self, data: &SerialData, mut ctx: Option<&mut NodeInfo>) {
        if !self.port_open {
            ctx.log_warning("Cannot write: serial port not open");
            return;
        }

        let bytes = data.get_data();
        if bytes.is_empty() {
            return;
        }

        match self.backend {
            SerialBackend::Simulation => {
                // In simulation, just count bytes
                self.bytes_transmitted += bytes.len() as u64;

                ctx.log_debug(&format!(
                    "Transmitted {} bytes to serial port (sim)",
                    bytes.len()
                ));

                // Log as string if valid UTF-8
                if let Some(text) = data.get_string() {
                    ctx.log_debug(&format!("TX: {}", text.trim()));
                }
            }
            #[cfg(feature = "serial-hardware")]
            SerialBackend::Hardware => {
                if let Some(ref mut port) = self.serial {
                    match port.write_all(bytes) {
                        Ok(_) => {
                            // Flush to ensure data is sent
                            let _ = port.flush();

                            self.bytes_transmitted += bytes.len() as u64;

                            ctx.log_debug(&format!(
                                "Transmitted {} bytes to hardware serial port",
                                bytes.len()
                            ));

                            // Log as string if valid UTF-8
                            if let Some(text) = data.get_string() {
                                ctx.log_debug(&format!("TX: {}", text.trim()));
                            }
                        }
                        Err(e) => {
                            self.errors += 1;
                            ctx.log_error(&format!("Serial write error: {:?}", e));
                        }
                    }
                }
            }
            #[cfg(not(feature = "serial-hardware"))]
            SerialBackend::Hardware => {
                // Should not reach here due to fallback in open_port
            }
        }
    }

    /// Simulate receiving data (for testing without hardware)
    pub fn simulate_receive(&mut self, data: &[u8]) {
        self.receive_buffer.extend_from_slice(data);
    }
}

impl Node for SerialNode {
    fn name(&self) -> &'static str {
        "SerialNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("Serial node initialized");

        match self.backend {
            SerialBackend::Simulation => {
                ctx.log_info("Serial simulation mode enabled");
            }
            SerialBackend::Hardware => {
                ctx.log_info(&format!(
                    "Serial hardware: {} @ {} baud",
                    self.port_path, self.baud_rate
                ));
            }
        }

        // Open serial port
        if !self.open_port(Some(ctx)) {
            ctx.log_error("Failed to open serial port");
        }

        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Read incoming data
        self.read_serial(ctx.as_deref_mut());

        // Process outgoing data
        while let Some(tx_data) = self.tx_subscriber.recv(&mut None) {
            self.write_serial(&tx_data, ctx.as_deref_mut());
        }
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        if self.port_open {
            ctx.log_info(&format!("Closing serial port {}", self.port_path));
            self.port_open = false;
        }
        Ok(())
    }
}
