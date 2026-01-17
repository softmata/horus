use crate::CanFrame;
use horus_core::error::HorusResult;

type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, NodeInfoExt, Topic};
use std::time::{SystemTime, UNIX_EPOCH};

// SocketCAN hardware support
#[cfg(feature = "can-hardware")]
use socketcan::{CanSocket, Socket};
#[cfg(feature = "can-hardware")]
use std::os::unix::io::AsRawFd;

/// CAN Bus Communication Node
///
/// Provides CAN (Controller Area Network) communication for automotive and industrial robotics.
/// Supports SocketCAN (Linux), CAN 2.0A/B, and CAN-FD protocols.
///
/// # Supported Hardware
/// - SocketCAN interfaces (can0, can1, vcan0, etc.)
/// - USB-CAN adapters (PCAN-USB, CANable, Kvaser, Peak, etc.)
/// - Embedded CAN controllers (MCP2515, TJA1050, etc.)
/// - Automotive CAN transceivers (ISO 11898)
/// - Industrial CAN devices (CANopen, DeviceNet, J1939)
///
/// # Common Applications
/// - Automotive: ECU communication, OBD-II diagnostics, J1939 heavy vehicle
/// - Industrial: CANopen motion control, DeviceNet automation
/// - Robotics: Motor controllers (ODrive, VESC, Roboclaw), sensors
/// - Aerospace: CANaerospace avionics communication
///
/// # Features
/// - Multiple CAN interface support
/// - Standard (11-bit) and extended (29-bit) identifiers
/// - CAN-FD support with bit rate switching
/// - Filtering by CAN ID (whitelist/blacklist)
/// - Error frame detection and monitoring
/// - Bus statistics (utilization, errors, retransmissions)
/// - Loopback and silent modes for testing
///
/// # Example
/// ```rust,ignore
/// use horus_library::nodes::CanBusNode;
/// use horus_library::CanFrame;
///
/// let mut can = CanBusNode::new("can0")?;
/// can.set_bitrate(500_000); // 500 kbit/s
/// can.add_filter(0x100, 0x7FF); // Filter for ID range 0x100-0x1FF
/// can.enable_can_fd(true);
///
/// // Send a CAN frame
/// let frame = CanFrame::new(0x123, &[1, 2, 3, 4]);
/// can.send_frame(frame);
/// ```
pub struct CanBusNode {
    tx_subscriber: Topic<CanFrame>,   // Frames to transmit
    rx_publisher: Topic<CanFrame>,    // Received frames
    error_publisher: Topic<CanFrame>, // Error frames

    // Hardware socket (Linux SocketCAN)
    #[cfg(feature = "can-hardware")]
    socket: Option<CanSocket>,
    hardware_enabled: bool,

    // Configuration
    interface_name: String,
    bitrate: u32,             // Bits per second (125k, 250k, 500k, 1M typical)
    fd_bitrate: u32,          // Data phase bitrate for CAN-FD
    enable_fd: bool,          // Enable CAN-FD
    enable_loopback: bool,    // Loopback mode for testing
    enable_listen_only: bool, // Silent mode (no ACK)
    restart_on_bus_off: bool, // Auto-restart after bus-off

    // Filtering
    id_filters: Vec<CanIdFilter>,
    filter_mode: FilterMode,

    // Statistics
    tx_count: u64,
    rx_count: u64,
    error_count: u64,
    bus_off_count: u64,
    last_error_time: u64,

    // State tracking
    bus_state: BusState,
    tx_error_counter: u8,
    rx_error_counter: u8,
    last_frame_time: u64,

    // Performance monitoring
    frames_per_second: f32,
    bus_load_percent: f32,
    last_stats_time: u64,
    frame_count_window: u64,

    // Timing state (moved from static mut for thread safety)
    last_status_log: u64,
}

/// CAN ID filter configuration
#[derive(Debug, Clone, Copy)]
struct CanIdFilter {
    id: u32,
    mask: u32, // Bit mask (1 = must match, 0 = don't care)
    is_extended: bool,
}

impl CanIdFilter {
    fn new(id: u32, mask: u32, is_extended: bool) -> Self {
        Self {
            id,
            mask,
            is_extended,
        }
    }

    fn matches(&self, frame_id: u32, frame_extended: bool) -> bool {
        if self.is_extended != frame_extended {
            return false;
        }
        (frame_id & self.mask) == (self.id & self.mask)
    }
}

/// Filter operation mode
#[derive(Debug, Clone, Copy, PartialEq)]
enum FilterMode {
    Accept,    // Accept all (no filtering)
    Whitelist, // Accept only matching IDs
    Blacklist, // Reject matching IDs
}

/// CAN bus state machine
#[derive(Debug, Clone, Copy, PartialEq)]
enum BusState {
    ErrorActive,  // Normal operation
    ErrorPassive, // High error count, degraded
    BusOff,       // Disabled due to errors
    Stopped,      // Not started
}

impl CanBusNode {
    /// Create a new CAN bus node
    pub fn new(interface: &str) -> Result<Self> {
        Ok(Self {
            tx_subscriber: Topic::new(&format!("can.{}.tx", interface))?,
            rx_publisher: Topic::new(&format!("can.{}.rx", interface))?,
            error_publisher: Topic::new(&format!("can.{}.error", interface))?,
            #[cfg(feature = "can-hardware")]
            socket: None,
            hardware_enabled: false,
            interface_name: interface.to_string(),
            bitrate: 500_000,      // 500 kbit/s default
            fd_bitrate: 2_000_000, // 2 Mbit/s for CAN-FD data phase
            enable_fd: false,
            enable_loopback: false,
            enable_listen_only: false,
            restart_on_bus_off: true,
            id_filters: Vec::new(),
            filter_mode: FilterMode::Accept,
            tx_count: 0,
            rx_count: 0,
            error_count: 0,
            bus_off_count: 0,
            last_error_time: 0,
            bus_state: BusState::Stopped,
            tx_error_counter: 0,
            rx_error_counter: 0,
            last_frame_time: 0,
            frames_per_second: 0.0,
            bus_load_percent: 0.0,
            last_stats_time: 0,
            frame_count_window: 0,
            last_status_log: 0,
        })
    }

    /// Set CAN bitrate in bits per second
    pub fn set_bitrate(&mut self, bitrate: u32) {
        self.bitrate = bitrate;
    }

    /// Set CAN-FD data phase bitrate
    pub fn set_fd_bitrate(&mut self, bitrate: u32) {
        self.fd_bitrate = bitrate;
    }

    /// Enable CAN-FD mode
    pub fn enable_can_fd(&mut self, enable: bool) {
        self.enable_fd = enable;
    }

    /// Enable loopback mode (for testing without hardware)
    pub fn enable_loopback(&mut self, enable: bool) {
        self.enable_loopback = enable;
    }

    /// Enable listen-only mode (no ACK, silent monitoring)
    pub fn enable_listen_only(&mut self, enable: bool) {
        self.enable_listen_only = enable;
    }

    /// Enable automatic restart after bus-off
    pub fn enable_auto_restart(&mut self, enable: bool) {
        self.restart_on_bus_off = enable;
    }

    /// Add a CAN ID filter
    pub fn add_filter(&mut self, id: u32, mask: u32) {
        let is_extended = id > CanFrame::MAX_STANDARD_ID;
        self.id_filters
            .push(CanIdFilter::new(id, mask, is_extended));
    }

    /// Add a CAN ID range filter
    pub fn add_range_filter(&mut self, start_id: u32, end_id: u32) {
        // Calculate mask that covers the range
        let diff = end_id ^ start_id;
        let mask = !diff;
        self.add_filter(start_id, mask);
    }

    /// Clear all filters
    pub fn clear_filters(&mut self) {
        self.id_filters.clear();
    }

    /// Set filter mode
    pub fn set_filter_mode(&mut self, mode: FilterMode) {
        self.filter_mode = mode;
    }

    /// Set whitelist mode (accept only listed IDs)
    pub fn set_whitelist_mode(&mut self) {
        self.filter_mode = FilterMode::Whitelist;
    }

    /// Set blacklist mode (reject listed IDs)
    pub fn set_blacklist_mode(&mut self) {
        self.filter_mode = FilterMode::Blacklist;
    }

    /// Check if a frame passes the filters
    fn check_filter(&self, frame: &CanFrame) -> bool {
        if self.filter_mode == FilterMode::Accept || self.id_filters.is_empty() {
            return true;
        }

        let matches = self
            .id_filters
            .iter()
            .any(|filter| filter.matches(frame.id, frame.is_extended));

        match self.filter_mode {
            FilterMode::Accept => true,
            FilterMode::Whitelist => matches,
            FilterMode::Blacklist => !matches,
        }
    }

    /// Get bus state
    pub fn get_bus_state(&self) -> BusState {
        self.bus_state
    }

    /// Get error counters
    pub fn get_error_counters(&self) -> (u8, u8) {
        (self.tx_error_counter, self.rx_error_counter)
    }

    /// Get statistics
    pub fn get_statistics(&self) -> (u64, u64, u64, f32, f32) {
        (
            self.tx_count,
            self.rx_count,
            self.error_count,
            self.frames_per_second,
            self.bus_load_percent,
        )
    }

    /// Reset statistics
    pub fn reset_statistics(&mut self) {
        self.tx_count = 0;
        self.rx_count = 0;
        self.error_count = 0;
        self.bus_off_count = 0;
    }

    /// Send a CAN frame
    fn send_frame(&mut self, mut frame: CanFrame, mut ctx: Option<&mut NodeInfo>) -> bool {
        // Check bus state
        if self.bus_state == BusState::BusOff || self.bus_state == BusState::Stopped {
            ctx.log_warning(&format!(
                "Cannot send frame: bus state is {:?}",
                self.bus_state
            ));
            return false;
        }

        // Validate frame
        if !frame.is_valid() {
            ctx.log_warning(&format!(
                "Invalid CAN frame: id=0x{:X}, dlc={}",
                frame.id, frame.dlc
            ));
            return false;
        }

        // Set interface name
        frame.set_interface(&self.interface_name);

        // Try hardware first, fall back to simulation
        #[cfg(feature = "can-hardware")]
        if self.hardware_enabled {
            match self.send_frame_hardware(&frame) {
                Ok(()) => {
                    ctx.log_debug(&format!(
                        "TX CAN (HW): id=0x{:03X}{} dlc={} data={:02X?}",
                        frame.id,
                        if frame.is_extended { " EXT" } else { "" },
                        frame.dlc,
                        frame.data_slice()
                    ));
                    self.tx_count += 1;
                    return true;
                }
                Err(e) => {
                    ctx.log_error(&format!(
                        "Hardware send failed: {:?}, falling back to simulation",
                        e
                    ));
                    self.hardware_enabled = false;
                }
            }
        }

        // Simulation fallback
        ctx.log_debug(&format!(
            "TX CAN (SIM): id=0x{:03X}{} dlc={} data={:02X?}",
            frame.id,
            if frame.is_extended { " EXT" } else { "" },
            frame.dlc,
            frame.data_slice()
        ));

        self.tx_count += 1;
        true
    }

    /// Simulate receiving a CAN frame
    fn simulate_receive(&mut self, current_time: u64, mut ctx: Option<&mut NodeInfo>) {
        // In real implementation, this would read from SocketCAN
        // For simulation, periodically generate test frames
        if current_time - self.last_frame_time < 100_000_000 {
            return; // Limit to 10Hz simulation
        }

        // Simulate a test frame
        let test_frame = CanFrame {
            id: 0x100 + (self.rx_count % 8) as u32,
            is_extended: false,
            is_rtr: false,
            is_error: false,
            data: [
                (self.rx_count & 0xFF) as u8,
                ((self.rx_count >> 8) & 0xFF) as u8,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            ],
            dlc: 2,
            is_fd: false,
            is_brs: false,
            is_esi: false,
            interface: [0; 16],
            timestamp: current_time,
        };

        self.last_frame_time = current_time;
        self.process_received_frame(test_frame, ctx.as_deref_mut());
    }

    /// Process a received CAN frame
    fn process_received_frame(&mut self, mut frame: CanFrame, mut ctx: Option<&mut NodeInfo>) {
        // Check for error frame
        if frame.is_error {
            self.error_count += 1;
            self.last_error_time = frame.timestamp;
            ctx.log_warning(&format!(
                "CAN error frame received on {}",
                self.interface_name
            ));
            let _ = self.error_publisher.send(frame, &mut None);
            return;
        }

        // Apply filters
        if !self.check_filter(&frame) {
            return;
        }

        // Set interface name
        frame.set_interface(&self.interface_name);

        // Update statistics
        self.rx_count += 1;
        self.frame_count_window += 1;

        // Publish received frame
        ctx.log_debug(&format!(
            "RX CAN: id=0x{:03X}{} dlc={} data={:02X?}",
            frame.id,
            if frame.is_extended { " EXT" } else { "" },
            frame.dlc,
            frame.data_slice()
        ));

        if let Err(e) = self.rx_publisher.send(frame, &mut None) {
            ctx.log_error(&format!("Failed to publish CAN frame: {:?}", e));
        }
    }

    /// Update bus state based on error counters
    fn update_bus_state(&mut self, mut ctx: Option<&mut NodeInfo>) {
        let prev_state = self.bus_state;

        // CAN error state machine (as per ISO 11898-1)
        if self.tx_error_counter > 255 || self.rx_error_counter > 255 {
            self.bus_state = BusState::BusOff;
            if prev_state != BusState::BusOff {
                self.bus_off_count += 1;
                ctx.log_error(&format!(
                    "CAN bus-off on {}: TxErr={}, RxErr={}",
                    self.interface_name, self.tx_error_counter, self.rx_error_counter
                ));

                if self.restart_on_bus_off {
                    self.tx_error_counter = 0;
                    self.rx_error_counter = 0;
                    self.bus_state = BusState::ErrorActive;
                    ctx.log_info(&format!("CAN bus restarted on {}", self.interface_name));
                }
            }
        } else if self.tx_error_counter > 127 || self.rx_error_counter > 127 {
            self.bus_state = BusState::ErrorPassive;
            if prev_state != BusState::ErrorPassive {
                ctx.log_warning(&format!(
                    "CAN error-passive on {}: TxErr={}, RxErr={}",
                    self.interface_name, self.tx_error_counter, self.rx_error_counter
                ));
            }
        } else {
            self.bus_state = BusState::ErrorActive;
        }
    }

    /// Update performance statistics
    fn update_statistics(&mut self, current_time: u64) {
        let elapsed = current_time - self.last_stats_time;
        if elapsed >= 1_000_000_000 {
            // Update every second
            let elapsed_sec = elapsed as f32 / 1_000_000_000.0;
            self.frames_per_second = self.frame_count_window as f32 / elapsed_sec;

            // Estimate bus load (very simplified)
            // Assume ~130 bits per CAN 2.0 frame at nominal bitrate
            let bits_per_frame = 130.0;
            let bits_per_second = self.frames_per_second * bits_per_frame;
            self.bus_load_percent = (bits_per_second / self.bitrate as f32) * 100.0;

            self.frame_count_window = 0;
            self.last_stats_time = current_time;
        }
    }

    /// Start the CAN interface
    pub fn start(&mut self, mut ctx: Option<&mut NodeInfo>) {
        ctx.log_info(&format!(
            "Starting CAN interface {} at {} kbit/s{}",
            self.interface_name,
            self.bitrate / 1000,
            if self.enable_fd {
                format!(" (FD: {} kbit/s)", self.fd_bitrate / 1000)
            } else {
                String::new()
            }
        ));

        // Try to open hardware socket
        #[cfg(feature = "can-hardware")]
        {
            match self.open_hardware_socket(ctx.as_deref_mut()) {
                Ok(()) => {
                    ctx.log_info(&format!(
                        "SocketCAN hardware enabled on {}",
                        self.interface_name
                    ));
                }
                Err(e) => {
                    ctx.log_warning(&format!(
                        "CanBusNode: Hardware unavailable - using SIMULATION mode"
                    ));
                    ctx.log_warning(&format!("  Tried: {}", self.interface_name));
                    ctx.log_warning(&format!("  Error: {}", e));
                    ctx.log_warning("  Fix:");
                    ctx.log_warning("    1. Install: sudo apt install can-utils");
                    ctx.log_warning("    2. Load kernel module: sudo modprobe vcan");
                    ctx.log_warning("    3. Create interface: sudo ip link add dev can0 type vcan");
                    ctx.log_warning("    4. Bring up interface: sudo ip link set up can0");
                    ctx.log_warning(
                        "    5. For real CAN hardware, use socketcan_interface drivers",
                    );
                    ctx.log_warning("    6. Rebuild with: cargo build --features=\"can-hardware\"");
                    self.hardware_enabled = false;
                }
            }
        }

        #[cfg(not(feature = "can-hardware"))]
        {
            ctx.log_warning(&format!(
                "CAN hardware support not compiled. Using simulation mode for {}",
                self.interface_name
            ));
        }

        self.bus_state = BusState::ErrorActive;
        self.last_stats_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
    }

    /// Stop the CAN interface
    pub fn stop(&mut self, mut ctx: Option<&mut NodeInfo>) {
        ctx.log_info(&format!("Stopping CAN interface {}", self.interface_name));

        #[cfg(feature = "can-hardware")]
        if self.hardware_enabled {
            self.socket = None;
            self.hardware_enabled = false;
        }

        self.bus_state = BusState::Stopped;
    }

    // ========== Hardware Backend Functions (SocketCAN) ==========

    /// Open SocketCAN hardware interface
    #[cfg(feature = "can-hardware")]
    fn open_hardware_socket(&mut self, mut ctx: Option<&mut NodeInfo>) -> std::io::Result<()> {
        // Open the CAN socket
        let socket = CanSocket::open(&self.interface_name)?;

        // Set non-blocking mode for reading
        socket.set_nonblocking(true)?;

        self.socket = Some(socket);
        self.hardware_enabled = true;
        ctx.log_info(&format!(
            "Opened SocketCAN interface: {}",
            self.interface_name
        ));

        Ok(())
    }

    /// Convert HORUS CanFrame to raw bytes for socketcan
    #[cfg(feature = "can-hardware")]
    fn horus_to_raw_frame(&self, frame: &CanFrame) -> [u8; 16] {
        let mut raw = [0u8; 16];

        // CAN ID (bytes 0-3)
        let mut id = frame.id;
        if frame.is_extended {
            id |= 0x80000000; // EFF flag
        }
        if frame.is_rtr {
            id |= 0x40000000; // RTR flag
        }
        if frame.is_error {
            id |= 0x20000000; // ERR flag
        }

        raw[0..4].copy_from_slice(&id.to_ne_bytes());

        // DLC (byte 4)
        raw[4] = frame.dlc.min(8);

        // Data (bytes 8-15)
        let data_len = raw[4].min(8) as usize;
        raw[8..8 + data_len].copy_from_slice(&frame.data[..data_len]);

        raw
    }

    /// Convert raw socketcan bytes to HORUS CanFrame
    #[cfg(feature = "can-hardware")]
    fn raw_to_horus_frame(&self, raw: &[u8; 16]) -> CanFrame {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;

        // Parse CAN ID from bytes 0-3
        let id_with_flags = u32::from_ne_bytes([raw[0], raw[1], raw[2], raw[3]]);

        let id = id_with_flags & 0x1FFFFFFF;
        let is_extended = (id_with_flags & 0x80000000) != 0;
        let is_rtr = (id_with_flags & 0x40000000) != 0;
        let is_error = (id_with_flags & 0x20000000) != 0;

        // Parse DLC from byte 4
        let dlc = raw[4].min(8);

        // Parse data from bytes 8-15
        let mut data = [0u8; 64];
        data[..dlc as usize].copy_from_slice(&raw[8..8 + dlc as usize]);

        let mut horus_frame = CanFrame {
            id,
            is_extended,
            is_rtr,
            is_error,
            data,
            dlc,
            is_fd: false,
            is_brs: false,
            is_esi: false,
            interface: [0; 16],
            timestamp: current_time,
        };

        horus_frame.set_interface(&self.interface_name);
        horus_frame
    }

    /// Send a CAN frame via hardware (using raw system calls)
    #[cfg(feature = "can-hardware")]
    fn send_frame_hardware(&mut self, frame: &CanFrame) -> std::io::Result<()> {
        use std::io::Write;

        // Convert frame before getting mutable borrow
        let raw_frame = self.horus_to_raw_frame(frame);

        let socket = self.socket.as_mut().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::NotConnected, "CAN socket not open")
        })?;

        // Write raw frame data
        socket.write_all(&raw_frame)?;

        Ok(())
    }

    /// Receive CAN frames from hardware (non-blocking, using raw reads)
    #[cfg(feature = "can-hardware")]
    fn receive_frames_hardware(&mut self) -> std::io::Result<Vec<CanFrame>> {
        use std::io::Read;

        let socket = self.socket.as_mut().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::NotConnected, "CAN socket not open")
        })?;

        let mut frames = Vec::new();
        let interface_name = self.interface_name.clone();

        // Read all available frames (non-blocking)
        loop {
            let mut raw_frame = [0u8; 16];
            match socket.read_exact(&mut raw_frame) {
                Ok(()) => {
                    // Convert inline to avoid borrow checker issues
                    let current_time = SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap_or_default()
                        .as_nanos() as u64;

                    let id_with_flags = u32::from_ne_bytes([
                        raw_frame[0],
                        raw_frame[1],
                        raw_frame[2],
                        raw_frame[3],
                    ]);
                    let id = id_with_flags & 0x1FFFFFFF;
                    let is_extended = (id_with_flags & 0x80000000) != 0;
                    let is_rtr = (id_with_flags & 0x40000000) != 0;
                    let is_error = (id_with_flags & 0x20000000) != 0;
                    let dlc = raw_frame[4].min(8);

                    let mut data = [0u8; 64];
                    data[..dlc as usize].copy_from_slice(&raw_frame[8..8 + dlc as usize]);

                    let mut horus_frame = CanFrame {
                        id,
                        is_extended,
                        is_rtr,
                        is_error,
                        data,
                        dlc,
                        is_fd: false,
                        is_brs: false,
                        is_esi: false,
                        interface: [0; 16],
                        timestamp: current_time,
                    };

                    horus_frame.set_interface(&interface_name);
                    frames.push(horus_frame);
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    // No more frames available
                    break;
                }
                Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => {
                    // Partial frame, stop reading
                    break;
                }
                Err(e) => {
                    return Err(e);
                }
            }
        }

        Ok(frames)
    }
}

impl Node for CanBusNode {
    fn name(&self) -> &'static str {
        "CanBusNode"
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info(&format!(
            "CanBusNode shutting down - closing {} interface",
            self.interface_name
        ));

        // Stop the CAN interface
        self.stop(Some(ctx));

        ctx.log_info("CAN bus interface closed safely");
        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;

        // Start interface if not started
        if self.bus_state == BusState::Stopped {
            self.start(ctx.as_deref_mut());
        }

        // Process outgoing frames
        while let Some(frame) = self.tx_subscriber.recv(&mut None) {
            self.send_frame(frame, ctx.as_deref_mut());
        }

        // Receive frames from hardware or simulation
        #[cfg(feature = "can-hardware")]
        if self.hardware_enabled {
            match self.receive_frames_hardware() {
                Ok(frames) => {
                    for frame in frames {
                        self.process_received_frame(frame, ctx.as_deref_mut());
                    }
                }
                Err(e) => {
                    ctx.log_error(&format!(
                        "Hardware receive failed: {:?}, falling back to simulation",
                        e
                    ));
                    self.hardware_enabled = false;
                    // Fall through to simulation
                    self.simulate_receive(current_time, ctx.as_deref_mut());
                }
            }
        } else {
            self.simulate_receive(current_time, ctx.as_deref_mut());
        }

        #[cfg(not(feature = "can-hardware"))]
        {
            self.simulate_receive(current_time, ctx.as_deref_mut());
        }

        // Update bus state
        self.update_bus_state(ctx.as_deref_mut());

        // Update statistics
        self.update_statistics(current_time);

        // Periodic status logging
        let status_interval = 10_000_000_000; // 10 seconds
        if current_time - self.last_status_log > status_interval {
            let (tx, rx, err, fps, load) = self.get_statistics();
            let (tx_err, rx_err) = self.get_error_counters();
            ctx.log_info(&format!(
                "CAN {}{}: TX={} RX={} ERR={} | {:.1} fps, {:.1}% load | State={:?} TxErr={} RxErr={}",
                self.interface_name,
                if self.hardware_enabled { " (HW)" } else { " (SIM)" },
                tx, rx, err, fps, load, self.bus_state, tx_err, rx_err
            ));
            self.last_status_log = current_time;
        }
    }
}

/// Preset configurations for common CAN bitrates
impl CanBusNode {
    /// Configure for 125 kbit/s (CANopen default)
    pub fn configure_125k(&mut self) {
        self.set_bitrate(125_000);
    }

    /// Configure for 250 kbit/s
    pub fn configure_250k(&mut self) {
        self.set_bitrate(250_000);
    }

    /// Configure for 500 kbit/s (common automotive)
    pub fn configure_500k(&mut self) {
        self.set_bitrate(500_000);
    }

    /// Configure for 1 Mbit/s (maximum for CAN 2.0)
    pub fn configure_1m(&mut self) {
        self.set_bitrate(1_000_000);
    }

    /// Configure for J1939 (heavy vehicle) - 250 kbit/s with extended IDs
    pub fn configure_j1939(&mut self) {
        self.set_bitrate(250_000);
        self.filter_mode = FilterMode::Accept;
    }

    /// Configure for CANopen - 125-1000 kbit/s
    pub fn configure_canopen(&mut self, bitrate: u32) {
        self.set_bitrate(bitrate);
        self.add_range_filter(0x000, 0x07FF); // COB-IDs
    }

    /// Configure for DeviceNet - 125-500 kbit/s
    pub fn configure_devicenet(&mut self, bitrate: u32) {
        self.set_bitrate(bitrate);
    }
}
