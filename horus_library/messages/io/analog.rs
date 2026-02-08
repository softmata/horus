//! Analog I/O message type

use horus_core::core::LogSummary;
use serde::{Deserialize, Serialize};
use serde_arrays;

/// Analog I/O measurements message
///
/// Represents analog input/output channels, typically used for
/// sensors, actuators, and continuous control signals.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalogIO {
    /// Channel values (in volts or engineering units)
    #[serde(with = "serde_arrays")]
    pub channels: [f64; 16],
    /// Number of active channels
    pub channel_count: u8,
    /// Channel ranges [min, max] for each channel
    pub channel_ranges: [[f64; 2]; 16],
    /// Engineering unit labels ("V", "mA", "°C", etc.)
    pub unit_labels: [[u8; 8]; 16],
    /// Channel names for identification
    pub channel_labels: [[u8; 16]; 16],
    /// ADC resolution in bits
    pub resolution_bits: u8,
    /// Sampling frequency in Hz
    pub sampling_frequency: f32,
    /// I/O board identifier
    pub board_id: [u8; 32],
    /// Timestamp in nanoseconds since epoch
    pub timestamp_ns: u64,
}

impl Default for AnalogIO {
    fn default() -> Self {
        Self {
            channels: [0.0; 16],
            channel_count: 0,
            channel_ranges: [[-10.0, 10.0]; 16], // Default ±10V range
            unit_labels: [[0; 8]; 16],
            channel_labels: [[0; 16]; 16],
            resolution_bits: 16,
            sampling_frequency: 1000.0,
            board_id: [0; 32],
            timestamp_ns: 0,
        }
    }
}

impl AnalogIO {
    /// Create a new analog I/O message
    pub fn new(channel_count: u8) -> Self {
        Self {
            channel_count,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
            ..Default::default()
        }
    }

    /// Set channel value
    pub fn set_channel(&mut self, channel: u8, value: f64) -> bool {
        if channel < self.channel_count && (channel as usize) < self.channels.len() {
            self.channels[channel as usize] = value;
            true
        } else {
            false
        }
    }

    /// Get channel value
    pub fn get_channel(&self, channel: u8) -> Option<f64> {
        if channel < self.channel_count && (channel as usize) < self.channels.len() {
            Some(self.channels[channel as usize])
        } else {
            None
        }
    }

    /// Set channel range
    pub fn set_channel_range(&mut self, channel: u8, min_val: f64, max_val: f64) -> bool {
        if channel < self.channel_count && (channel as usize) < self.channel_ranges.len() {
            self.channel_ranges[channel as usize] = [min_val, max_val];
            true
        } else {
            false
        }
    }

    /// Convert raw ADC value to engineering units
    pub fn raw_to_engineering(&self, channel: u8, raw_value: u16) -> Option<f64> {
        if channel < self.channel_count && (channel as usize) < self.channel_ranges.len() {
            let max_raw = (1 << self.resolution_bits) - 1;
            let normalized = raw_value as f64 / max_raw as f64;
            let range = &self.channel_ranges[channel as usize];
            Some(range[0] + normalized * (range[1] - range[0]))
        } else {
            None
        }
    }

    /// Convert engineering units to raw ADC value
    pub fn engineering_to_raw(&self, channel: u8, eng_value: f64) -> Option<u16> {
        if channel < self.channel_count && (channel as usize) < self.channel_ranges.len() {
            let range = &self.channel_ranges[channel as usize];
            let normalized = (eng_value - range[0]) / (range[1] - range[0]);
            let max_raw = (1 << self.resolution_bits) - 1;
            Some((normalized * max_raw as f64).clamp(0.0, max_raw as f64) as u16)
        } else {
            None
        }
    }

    /// Set channel label and unit
    pub fn set_channel_info(&mut self, channel: u8, label: &str, unit: &str) -> bool {
        if channel < self.channel_count && (channel as usize) < self.channel_labels.len() {
            // Set label
            let label_bytes = label.as_bytes();
            let len = label_bytes.len().min(15);
            self.channel_labels[channel as usize][..len].copy_from_slice(&label_bytes[..len]);
            self.channel_labels[channel as usize][len] = 0;

            // Set unit
            let unit_bytes = unit.as_bytes();
            let len = unit_bytes.len().min(7);
            self.unit_labels[channel as usize][..len].copy_from_slice(&unit_bytes[..len]);
            self.unit_labels[channel as usize][len] = 0;

            true
        } else {
            false
        }
    }
}

impl LogSummary for AnalogIO {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}
