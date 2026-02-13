//! Digital I/O message type

use horus_macros::LogSummary;
use serde::{Deserialize, Serialize};
use serde_arrays;

/// Digital I/O state message
///
/// Represents the state of digital input/output pins, typically used
/// for interfacing with sensors, actuators, and industrial equipment.
#[repr(C)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default, LogSummary)]
pub struct DigitalIO {
    /// Pin states (1 = high/on, 0 = low/off)
    #[serde(with = "serde_arrays")]
    pub pins: [u8; 32],
    /// Number of active pins
    pub pin_count: u8,
    /// Pin direction mask (1 = output, 0 = input)
    #[serde(with = "serde_arrays")]
    pub pin_directions: [u8; 32],
    /// Pull-up resistor enable mask (1 = enabled, 0 = disabled)
    #[serde(with = "serde_arrays")]
    pub pullup_enable: [u8; 32],
    /// Pin labels for identification
    pub pin_labels: [[u8; 16]; 32],
    /// I/O board identifier
    pub board_id: [u8; 32],
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl DigitalIO {
    /// Create a new digital I/O message
    pub fn new(pin_count: u8) -> Self {
        Self {
            pin_count,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            ..Default::default()
        }
    }

    /// Set pin state
    pub fn set_pin(&mut self, pin: u8, state: bool) -> bool {
        if pin < self.pin_count && (pin as usize) < self.pins.len() {
            self.pins[pin as usize] = state as u8;
            true
        } else {
            false
        }
    }

    /// Get pin state
    pub fn get_pin(&self, pin: u8) -> Option<bool> {
        if pin < self.pin_count && (pin as usize) < self.pins.len() {
            Some(self.pins[pin as usize] != 0)
        } else {
            None
        }
    }

    /// Set pin direction (true = output, false = input)
    pub fn set_pin_direction(&mut self, pin: u8, is_output: bool) -> bool {
        if pin < self.pin_count && (pin as usize) < self.pin_directions.len() {
            self.pin_directions[pin as usize] = is_output as u8;
            true
        } else {
            false
        }
    }

    /// Set pin label
    pub fn set_pin_label(&mut self, pin: u8, label: &str) -> bool {
        if pin < self.pin_count && (pin as usize) < self.pin_labels.len() {
            let label_bytes = label.as_bytes();
            let len = label_bytes.len().min(15);
            self.pin_labels[pin as usize][..len].copy_from_slice(&label_bytes[..len]);
            self.pin_labels[pin as usize][len] = 0;
            true
        } else {
            false
        }
    }

    /// Get pin label as string
    pub fn get_pin_label(&self, pin: u8) -> Option<String> {
        if pin < self.pin_count && (pin as usize) < self.pin_labels.len() {
            let label_bytes = &self.pin_labels[pin as usize];
            let end = label_bytes.iter().position(|&b| b == 0).unwrap_or(16);
            Some(String::from_utf8_lossy(&label_bytes[..end]).into_owned())
        } else {
            None
        }
    }

    /// Count active (high) pins
    pub fn count_active(&self) -> u8 {
        (0..self.pin_count)
            .filter(|&pin| self.pins[pin as usize] != 0)
            .count() as u8
    }

    /// Get bitmask representation
    pub fn as_bitmask(&self) -> u32 {
        let mut mask = 0u32;
        for i in 0..self.pin_count.min(32) {
            if self.pins[i as usize] != 0 {
                mask |= 1 << i;
            }
        }
        mask
    }

    /// Set from bitmask
    pub fn from_bitmask(&mut self, mask: u32) {
        for i in 0..self.pin_count.min(32) {
            self.pins[i as usize] = ((mask & (1 << i)) != 0) as u8;
        }
        self.timestamp_ns = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
    }
}

// =============================================================================
// POD (Plain Old Data) Message Support
// =============================================================================

unsafe impl horus_core::bytemuck::Pod for DigitalIO {}
unsafe impl horus_core::bytemuck::Zeroable for DigitalIO {}
unsafe impl horus_core::communication::PodMessage for DigitalIO {}
