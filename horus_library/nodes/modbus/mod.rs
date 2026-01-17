use crate::{ModbusMessage, NetworkStatus};
use horus_core::error::HorusResult;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, NodeInfoExt, Topic};
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

#[cfg(feature = "modbus-hardware")]
use tokio_modbus::prelude::*;

#[cfg(feature = "modbus-hardware")]
use tokio::runtime::Runtime;

/// Modbus backend type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ModbusBackend {
    Simulation,
    ModbusTcp,
    ModbusRtu,
}

/// Modbus Node - Industrial protocol handler for Modbus TCP/RTU communication
///
/// Handles Modbus protocol communication with industrial equipment.
/// Supports reading/writing coils, discrete inputs, holding registers, and input registers.
///
/// Supported backends:
/// - Modbus TCP (Ethernet-based)
/// - Modbus RTU (Serial-based)
/// - Simulation mode for testing
pub struct ModbusNode {
    publisher: Topic<ModbusMessage>,
    status_publisher: Topic<NetworkStatus>,
    request_subscriber: Topic<ModbusMessage>,

    // Configuration
    server_address: String,
    server_port: u16,
    slave_id: u8,
    timeout_ms: u64,
    backend: ModbusBackend,
    serial_port: String, // For RTU mode
    baud_rate: u32,      // For RTU mode

    // State
    is_connected: bool,
    connection_attempts: u32,
    last_activity: u64,
    device_cache: HashMap<u16, u16>, // address -> value cache

    // Hardware drivers
    #[cfg(feature = "modbus-hardware")]
    runtime: Option<Runtime>,

    #[cfg(feature = "modbus-hardware")]
    tcp_context: Option<tokio_modbus::client::Context>,

    #[cfg(feature = "modbus-hardware")]
    rtu_context: Option<tokio_modbus::client::Context>,
}

impl ModbusNode {
    /// Create a new Modbus node with default topics in simulation mode
    pub fn new() -> Result<Self> {
        Self::new_with_backend(
            "modbus_request",
            "modbus_response",
            "modbus_status",
            ModbusBackend::Simulation,
        )
    }

    /// Create a new Modbus node with custom topics
    pub fn new_with_topics(
        request_topic: &str,
        response_topic: &str,
        status_topic: &str,
    ) -> Result<Self> {
        Self::new_with_backend(
            request_topic,
            response_topic,
            status_topic,
            ModbusBackend::Simulation,
        )
    }

    /// Create a new Modbus node with specific backend
    pub fn new_with_backend(
        request_topic: &str,
        response_topic: &str,
        status_topic: &str,
        backend: ModbusBackend,
    ) -> Result<Self> {
        Ok(Self {
            publisher: Topic::new(response_topic)?,
            status_publisher: Topic::new(status_topic)?,
            request_subscriber: Topic::new(request_topic)?,

            server_address: "127.0.0.1".to_string(),
            server_port: 502,
            slave_id: 1,
            timeout_ms: 5000,
            backend,
            serial_port: "/dev/ttyUSB0".to_string(),
            baud_rate: 19200,

            is_connected: false,
            connection_attempts: 0,
            last_activity: 0,
            device_cache: HashMap::new(),

            #[cfg(feature = "modbus-hardware")]
            runtime: None,

            #[cfg(feature = "modbus-hardware")]
            tcp_context: None,

            #[cfg(feature = "modbus-hardware")]
            rtu_context: None,
        })
    }

    /// Set Modbus backend
    pub fn set_backend(&mut self, backend: ModbusBackend) {
        self.backend = backend;
        self.is_connected = false;
    }

    /// Set Modbus TCP server connection parameters
    pub fn set_connection(&mut self, address: &str, port: u16, slave_id: u8) {
        self.server_address = address.to_string();
        self.server_port = port;
        self.slave_id = slave_id;
    }

    /// Set Modbus RTU serial configuration
    pub fn set_rtu_config(&mut self, port: &str, baud_rate: u32, slave_id: u8) {
        self.serial_port = port.to_string();
        self.baud_rate = baud_rate;
        self.slave_id = slave_id;
    }

    /// Set communication timeout in milliseconds
    pub fn set_timeout(&mut self, timeout_ms: u64) {
        self.timeout_ms = timeout_ms;
    }

    /// Check if connected to Modbus server
    pub fn is_connected(&self) -> bool {
        self.is_connected
    }

    /// Get connection statistics
    pub fn get_stats(&self) -> (u32, u64, usize) {
        (
            self.connection_attempts,
            self.last_activity,
            self.device_cache.len(),
        )
    }

    /// Initialize Modbus connection
    fn initialize_connection(&mut self) -> bool {
        if self.is_connected {
            return true;
        }

        self.connection_attempts += 1;

        match self.backend {
            ModbusBackend::Simulation => {
                // Simulate connection success after a few attempts
                if self.connection_attempts > 2 {
                    self.is_connected = true;
                    true
                } else {
                    self.is_connected = false;
                    false
                }
            }
            #[cfg(feature = "modbus-hardware")]
            ModbusBackend::ModbusTcp => {
                // Initialize tokio runtime if needed
                if self.runtime.is_none() {
                    match Runtime::new() {
                        Ok(rt) => self.runtime = Some(rt),
                        Err(e) => {
                            eprintln!("Failed to create tokio runtime: {:?}", e);
                            eprintln!("Falling back to simulation mode");
                            self.backend = ModbusBackend::Simulation;
                            return false;
                        }
                    }
                }

                // Try to connect to Modbus TCP server
                if let Some(ref rt) = self.runtime {
                    let socket_addr = format!("{}:{}", self.server_address, self.server_port);

                    match rt.block_on(async { tcp::connect(socket_addr.parse().ok()?).await.ok() })
                    {
                        Some(ctx) => {
                            self.tcp_context = Some(ctx);
                            self.is_connected = true;
                            true
                        }
                        None => {
                            eprintln!(
                                "Failed to connect to Modbus TCP server {}:{}",
                                self.server_address, self.server_port
                            );
                            if self.connection_attempts > 3 {
                                eprintln!("Falling back to simulation mode");
                                self.backend = ModbusBackend::Simulation;
                                self.is_connected = true;
                            }
                            false
                        }
                    }
                } else {
                    false
                }
            }
            #[cfg(feature = "modbus-hardware")]
            ModbusBackend::ModbusRtu => {
                // Initialize tokio runtime if needed
                if self.runtime.is_none() {
                    match Runtime::new() {
                        Ok(rt) => self.runtime = Some(rt),
                        Err(e) => {
                            eprintln!("Failed to create tokio runtime: {:?}", e);
                            eprintln!("Falling back to simulation mode");
                            self.backend = ModbusBackend::Simulation;
                            return false;
                        }
                    }
                }

                // Try to open RTU serial port
                if let Some(ref rt) = self.runtime {
                    match rt.block_on(async {
                        let builder = tokio_serial::new(&self.serial_port, self.baud_rate);
                        rtu::connect_slave(builder, Slave(self.slave_id)).await.ok()
                    }) {
                        Some(ctx) => {
                            self.rtu_context = Some(ctx);
                            self.is_connected = true;
                            true
                        }
                        None => {
                            eprintln!(
                                "Failed to open Modbus RTU port {} @ {} baud",
                                self.serial_port, self.baud_rate
                            );
                            if self.connection_attempts > 3 {
                                eprintln!("Falling back to simulation mode");
                                self.backend = ModbusBackend::Simulation;
                                self.is_connected = true;
                            }
                            false
                        }
                    }
                } else {
                    false
                }
            }
            #[cfg(not(feature = "modbus-hardware"))]
            ModbusBackend::ModbusTcp | ModbusBackend::ModbusRtu => {
                eprintln!("Hardware backend requested but modbus-hardware feature not enabled");
                eprintln!("Falling back to simulation mode");
                self.backend = ModbusBackend::Simulation;
                self.is_connected = true;
                true
            }
        }
    }

    fn handle_modbus_request(&mut self, request: ModbusMessage) -> Option<ModbusMessage> {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        self.last_activity = current_time;

        let mut response = request.clone();
        response.is_request = false;
        response.timestamp = current_time;

        match self.backend {
            ModbusBackend::Simulation => {
                // Simulate processing the request
                match request.function_code {
                    1 | 2 => {
                        // Read Coils / Discrete Inputs
                        response.data_length = request.quantity as u8;
                        for i in 0..response.data_length as usize {
                            response.data[i] = ((request.start_address as usize + i) % 2) as u16;
                        }
                    }
                    3 | 4 => {
                        // Read Holding / Input Registers
                        response.data_length = request.quantity as u8;
                        for i in 0..response.data_length as usize {
                            let addr = request.start_address + i as u16;
                            let value = self.device_cache.get(&addr).cloned().unwrap_or(addr * 10);
                            response.data[i] = value;
                        }
                    }
                    5 | 6 => {
                        // Write Single Coil / Register
                        if request.data_length > 0 {
                            self.device_cache
                                .insert(request.start_address, request.data[0]);
                        }
                        response.data[0] = request.data[0];
                    }
                    15 | 16 => {
                        // Write Multiple Coils / Registers
                        for i in 0..request.data_length as usize {
                            let addr = request.start_address + i as u16;
                            self.device_cache.insert(addr, request.data[i]);
                        }
                    }
                    _ => {
                        response.function_code = request.function_code | 0x80;
                    }
                }
                Some(response)
            }
            #[cfg(feature = "modbus-hardware")]
            ModbusBackend::ModbusTcp | ModbusBackend::ModbusRtu => {
                let context = if self.backend == ModbusBackend::ModbusTcp {
                    self.tcp_context.as_mut()
                } else {
                    self.rtu_context.as_mut()
                };

                if let (Some(ctx), Some(ref rt)) = (context, &self.runtime) {
                    let result = rt.block_on(async {
                        match request.function_code {
                            1 => {
                                // Read Coils
                                ctx.read_coils(request.start_address, request.quantity)
                                    .await
                                    .ok()
                                    .map(|coils| {
                                        response.data_length = coils.len().min(256) as u8;
                                        for (i, coil) in coils.iter().enumerate().take(256) {
                                            response.data[i] = if *coil { 1 } else { 0 };
                                        }
                                        response
                                    })
                            }
                            2 => {
                                // Read Discrete Inputs
                                ctx.read_discrete_inputs(request.start_address, request.quantity)
                                    .await
                                    .ok()
                                    .map(|inputs| {
                                        response.data_length = inputs.len().min(256) as u8;
                                        for (i, input) in inputs.iter().enumerate().take(256) {
                                            response.data[i] = if *input { 1 } else { 0 };
                                        }
                                        response
                                    })
                            }
                            3 => {
                                // Read Holding Registers
                                ctx.read_holding_registers(request.start_address, request.quantity)
                                    .await
                                    .ok()
                                    .map(|regs| {
                                        response.data_length = regs.len().min(256) as u8;
                                        for (i, &reg) in regs.iter().enumerate().take(256) {
                                            response.data[i] = reg;
                                        }
                                        response
                                    })
                            }
                            4 => {
                                // Read Input Registers
                                ctx.read_input_registers(request.start_address, request.quantity)
                                    .await
                                    .ok()
                                    .map(|regs| {
                                        response.data_length = regs.len().min(256) as u8;
                                        for (i, &reg) in regs.iter().enumerate().take(256) {
                                            response.data[i] = reg;
                                        }
                                        response
                                    })
                            }
                            5 => {
                                // Write Single Coil
                                let value = request.data[0] != 0;
                                ctx.write_single_coil(request.start_address, value)
                                    .await
                                    .ok()
                                    .map(|_| {
                                        response.data[0] = request.data[0];
                                        response
                                    })
                            }
                            6 => {
                                // Write Single Register
                                ctx.write_single_register(request.start_address, request.data[0])
                                    .await
                                    .ok()
                                    .map(|_| {
                                        response.data[0] = request.data[0];
                                        response
                                    })
                            }
                            15 => {
                                // Write Multiple Coils
                                let coils: Vec<bool> = request.data[..request.data_length as usize]
                                    .iter()
                                    .map(|&v| v != 0)
                                    .collect();
                                ctx.write_multiple_coils(request.start_address, &coils)
                                    .await
                                    .ok()
                                    .map(|_| response)
                            }
                            16 => {
                                // Write Multiple Registers
                                let regs = &request.data[..request.data_length as usize];
                                ctx.write_multiple_registers(request.start_address, regs)
                                    .await
                                    .ok()
                                    .map(|_| response)
                            }
                            _ => {
                                // Unsupported function code
                                response.function_code = request.function_code | 0x80;
                                Some(response)
                            }
                        }
                    });

                    result
                } else {
                    None
                }
            }
            #[cfg(not(feature = "modbus-hardware"))]
            ModbusBackend::ModbusTcp | ModbusBackend::ModbusRtu => None,
        }
    }

    fn publish_network_status(&self) {
        let _current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        let mut status = NetworkStatus::new("modbus");
        status.link_up = self.is_connected;
        status.tx_packets = self.connection_attempts as u64;
        status.rx_packets = if self.last_activity > 0 { 1 } else { 0 };
        status.tx_errors = 0; // Would track actual errors
        status.rx_errors = 0;

        let _ = self.status_publisher.send(status, &mut None);
    }
}

impl Node for ModbusNode {
    fn name(&self) -> &'static str {
        "ModbusNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("Modbus node initialized");

        match self.backend {
            ModbusBackend::Simulation => {
                ctx.log_info("Modbus simulation mode enabled");
            }
            ModbusBackend::ModbusTcp => {
                ctx.log_info(&format!(
                    "Modbus TCP: {}:{} (slave {})",
                    self.server_address, self.server_port, self.slave_id
                ));
            }
            ModbusBackend::ModbusRtu => {
                ctx.log_info(&format!(
                    "Modbus RTU: {} @ {} baud (slave {})",
                    self.serial_port, self.baud_rate, self.slave_id
                ));
            }
        }

        Ok(())
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        // Try to connect if not connected
        if !self.is_connected {
            self.initialize_connection();
        }

        // Handle incoming requests
        if let Some(request) = self.request_subscriber.recv(&mut None) {
            if self.is_connected {
                if let Some(response) = self.handle_modbus_request(request) {
                    let _ = self.publisher.send(response, &mut None);
                }
            }
        }

        // Publish network status periodically
        self.publish_network_status();
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        if self.is_connected {
            ctx.log_info("Closing Modbus connection");

            #[cfg(feature = "modbus-hardware")]
            {
                self.tcp_context = None;
                self.rtu_context = None;
                self.runtime = None;
            }

            self.is_connected = false;
        }
        Ok(())
    }
}

// Default impl removed - use Node::new() instead which returns HorusResult
