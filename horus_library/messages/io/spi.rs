//! SPI (Serial Peripheral Interface) message type

use horus_core::core::LogSummary;
use serde::{Deserialize, Serialize};
use serde_arrays;

/// SPI (Serial Peripheral Interface) Transaction Message
///
/// Standard SPI transaction for communicating with SPI devices.
/// Supports full-duplex communication with configurable modes.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SpiMessage {
    /// SPI bus number (e.g., 0 for /dev/spidev0.0)
    pub bus: u8,
    /// Chip select line (0-7)
    pub chip_select: u8,
    /// SPI mode (0-3): CPOL and CPHA combinations
    pub mode: u8,
    /// Clock speed in Hz
    pub speed_hz: u32,
    /// Bits per word (typically 8)
    pub bits_per_word: u8,
    /// Data to transmit (MOSI)
    #[serde(with = "serde_arrays")]
    pub tx_data: [u8; 256],
    /// Data received (MISO)
    #[serde(with = "serde_arrays")]
    pub rx_data: [u8; 256],
    /// Transaction length in bytes
    pub length: u16,
    /// Chip select active high (default: false = active low)
    pub cs_high: bool,
    /// LSB first (default: false = MSB first)
    pub lsb_first: bool,
    /// 3-wire mode (bidirectional)
    pub three_wire: bool,
    /// Transaction successful
    pub success: bool,
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for SpiMessage {
    fn default() -> Self {
        Self {
            bus: 0,
            chip_select: 0,
            mode: 0,
            speed_hz: 1_000_000,
            bits_per_word: 8,
            tx_data: [0; 256],
            rx_data: [0; 256],
            length: 0,
            cs_high: false,
            lsb_first: false,
            three_wire: false,
            success: false,
            timestamp_ns: 0,
        }
    }
}

impl SpiMessage {
    /// SPI Mode 0: CPOL=0, CPHA=0 (most common)
    pub const MODE_0: u8 = 0;
    /// SPI Mode 1: CPOL=0, CPHA=1
    pub const MODE_1: u8 = 1;
    /// SPI Mode 2: CPOL=1, CPHA=0
    pub const MODE_2: u8 = 2;
    /// SPI Mode 3: CPOL=1, CPHA=1
    pub const MODE_3: u8 = 3;

    /// Create a new SPI message for transmission
    pub fn new(bus: u8, chip_select: u8, data: &[u8]) -> Self {
        let mut msg = Self {
            bus,
            chip_select,
            mode: Self::MODE_0,
            speed_hz: 1_000_000, // 1 MHz default
            bits_per_word: 8,
            tx_data: [0; 256],
            rx_data: [0; 256],
            length: data.len().min(256) as u16,
            cs_high: false,
            lsb_first: false,
            three_wire: false,
            success: false,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        };
        msg.tx_data[..data.len().min(256)].copy_from_slice(&data[..data.len().min(256)]);
        msg
    }

    /// Create SPI message with specific mode and speed
    pub fn with_config(bus: u8, cs: u8, data: &[u8], mode: u8, speed_hz: u32) -> Self {
        let mut msg = Self::new(bus, cs, data);
        msg.mode = mode;
        msg.speed_hz = speed_hz;
        msg
    }

    /// Set SPI mode
    pub fn set_mode(&mut self, mode: u8) {
        self.mode = mode & 0x03;
    }

    /// Set transmission data
    pub fn set_tx_data(&mut self, data: &[u8]) -> bool {
        if data.len() > 256 {
            return false;
        }
        self.tx_data[..data.len()].copy_from_slice(data);
        self.length = data.len() as u16;
        true
    }

    /// Get received data as slice
    pub fn rx_data_slice(&self) -> &[u8] {
        &self.rx_data[..self.length as usize]
    }

    /// Get transmitted data as slice
    pub fn tx_data_slice(&self) -> &[u8] {
        &self.tx_data[..self.length as usize]
    }
}

impl LogSummary for SpiMessage {
    fn log_summary(&self) -> String {
        format!(
            "SPI[{}.{}]: {} bytes @ {}Hz mode={}",
            self.bus, self.chip_select, self.length, self.speed_hz, self.mode
        )
    }
}
