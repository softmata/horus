//! PWM chip and channel discovery for HORUS (Linux only).
//!
//! Scans for available PWM chips and their channels via sysfs.

use serde::{Deserialize, Serialize};
use std::fs;
use std::path::PathBuf;

/// PWM channel information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PwmChannel {
    /// Chip number
    pub chip_number: u8,
    /// Channel number within the chip
    pub channel_number: u32,
    /// Sysfs path for this channel
    pub sysfs_path: PathBuf,
    /// Whether the channel is exported and accessible
    pub exported: bool,
    /// Whether the channel is enabled
    pub enabled: bool,
    /// Current period in nanoseconds
    pub period_ns: Option<u64>,
    /// Current duty cycle in nanoseconds
    pub duty_cycle_ns: Option<u64>,
    /// Polarity (normal or inverted)
    pub polarity: PwmPolarity,
}

impl PwmChannel {
    /// Get device specification string
    pub fn device_spec(&self) -> String {
        format!("pwmchip{}:pwm{}", self.chip_number, self.channel_number)
    }

    /// Get duty cycle as percentage (0.0 to 100.0)
    pub fn duty_cycle_percent(&self) -> Option<f64> {
        match (self.duty_cycle_ns, self.period_ns) {
            (Some(duty), Some(period)) if period > 0 => Some((duty as f64 / period as f64) * 100.0),
            _ => None,
        }
    }

    /// Get frequency in Hz
    pub fn frequency_hz(&self) -> Option<f64> {
        self.period_ns.map(|p| 1_000_000_000.0 / p as f64)
    }
}

/// PWM polarity
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PwmPolarity {
    /// Normal polarity (duty cycle is high time)
    Normal,
    /// Inverted polarity (duty cycle is low time)
    Inverted,
}

impl From<&str> for PwmPolarity {
    fn from(s: &str) -> Self {
        match s.trim().to_lowercase().as_str() {
            "inversed" | "inverted" => PwmPolarity::Inverted,
            _ => PwmPolarity::Normal,
        }
    }
}

/// PWM chip information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PwmChip {
    /// Chip number (e.g., 0 for pwmchip0)
    pub chip_number: u8,
    /// Sysfs path
    pub sysfs_path: PathBuf,
    /// Number of PWM channels on this chip
    pub npwm: u32,
    /// Device name/description
    pub device_name: Option<String>,
    /// Controller type
    pub controller_type: Option<String>,
    /// Exported channels
    pub channels: Vec<PwmChannel>,
}

impl PwmChip {
    /// Get all channel numbers
    pub fn channel_numbers(&self) -> Vec<u32> {
        (0..self.npwm).collect()
    }

    /// Get exported channel count
    pub fn exported_channel_count(&self) -> usize {
        self.channels.iter().filter(|c| c.exported).count()
    }
}

/// PWM discovery
pub struct PwmDiscovery {
    /// Discovered chips
    chips: Vec<PwmChip>,
}

impl PwmDiscovery {
    /// Create a new PWM discovery instance
    pub fn new() -> Self {
        Self { chips: Vec::new() }
    }

    /// Enumerate PWM chips and their channels
    pub fn enumerate_chips(&mut self) -> Vec<PwmChip> {
        self.chips.clear();

        // Scan /sys/class/pwm for pwmchip* entries
        if let Ok(entries) = fs::read_dir("/sys/class/pwm") {
            for entry in entries.flatten() {
                let name = entry.file_name().to_string_lossy().to_string();
                if let Some(suffix) = name.strip_prefix("pwmchip") {
                    if let Ok(chip_num) = suffix.parse::<u8>() {
                        if let Some(chip) = self.probe_chip(chip_num, entry.path()) {
                            self.chips.push(chip);
                        }
                    }
                }
            }
        }

        // Sort by chip number
        self.chips.sort_by_key(|c| c.chip_number);
        self.chips.clone()
    }

    /// Get cached chips
    pub fn chips(&self) -> &[PwmChip] {
        &self.chips
    }

    /// Get total channel count across all chips
    pub fn total_channel_count(&self) -> u32 {
        self.chips.iter().map(|c| c.npwm).sum()
    }

    /// Get total exported channel count
    pub fn exported_channel_count(&self) -> usize {
        self.chips.iter().map(|c| c.exported_channel_count()).sum()
    }

    fn probe_chip(&self, chip_number: u8, sysfs_path: PathBuf) -> Option<PwmChip> {
        // Read npwm (number of PWM channels)
        let npwm_path = sysfs_path.join("npwm");
        let npwm = fs::read_to_string(&npwm_path)
            .ok()
            .and_then(|s| s.trim().parse().ok())
            .unwrap_or(0);

        if npwm == 0 {
            return None;
        }

        // Get device name
        let device_name = self.get_device_name(&sysfs_path);

        // Get controller type
        let controller_type = self.get_controller_type(&sysfs_path);

        // Scan for exported channels
        let channels = self.scan_channels(chip_number, &sysfs_path, npwm);

        Some(PwmChip {
            chip_number,
            sysfs_path,
            npwm,
            device_name,
            controller_type,
            channels,
        })
    }

    #[allow(clippy::ptr_arg)]
    fn scan_channels(&self, chip_number: u8, chip_path: &PathBuf, npwm: u32) -> Vec<PwmChannel> {
        let mut channels = Vec::new();

        for channel_num in 0..npwm {
            let channel_path = chip_path.join(format!("pwm{}", channel_num));
            let exported = channel_path.exists();

            let (enabled, period_ns, duty_cycle_ns, polarity) = if exported {
                (
                    self.read_bool(&channel_path.join("enable")),
                    self.read_u64(&channel_path.join("period")),
                    self.read_u64(&channel_path.join("duty_cycle")),
                    self.read_polarity(&channel_path.join("polarity")),
                )
            } else {
                (false, None, None, PwmPolarity::Normal)
            };

            channels.push(PwmChannel {
                chip_number,
                channel_number: channel_num,
                sysfs_path: channel_path,
                exported,
                enabled,
                period_ns,
                duty_cycle_ns,
                polarity,
            });
        }

        channels
    }

    #[allow(clippy::ptr_arg)]
    fn get_device_name(&self, chip_path: &PathBuf) -> Option<String> {
        // Try device tree name
        let of_name = chip_path.join("device/of_node/name");
        if let Ok(name) = fs::read_to_string(&of_name) {
            let name = name.trim().trim_end_matches('\0');
            if !name.is_empty() {
                return Some(name.to_string());
            }
        }

        // Try to get from device link
        let device_link = chip_path.join("device");
        if let Ok(target) = fs::read_link(&device_link) {
            if let Some(name) = target.file_name() {
                return Some(name.to_string_lossy().to_string());
            }
        }

        None
    }

    #[allow(clippy::ptr_arg)]
    fn get_controller_type(&self, chip_path: &PathBuf) -> Option<String> {
        // Try driver name
        let driver_link = chip_path.join("device/driver");
        if let Ok(target) = fs::read_link(&driver_link) {
            if let Some(driver) = target.file_name() {
                let driver_name = driver.to_string_lossy().to_string();
                return Some(self.friendly_controller_name(&driver_name));
            }
        }

        // Try compatible string
        let compat_path = chip_path.join("device/of_node/compatible");
        if let Ok(compat) = fs::read_to_string(compat_path) {
            return Some(self.friendly_controller_name(compat.trim()));
        }

        None
    }

    fn friendly_controller_name(&self, raw: &str) -> String {
        if raw.contains("bcm2835") || raw.contains("bcm2711") {
            "Raspberry Pi PWM".to_string()
        } else if raw.contains("tegra") {
            "NVIDIA Tegra PWM".to_string()
        } else if raw.contains("omap") || raw.contains("ti,") || raw.contains("ehrpwm") {
            "TI PWM".to_string()
        } else if raw.contains("imx") || raw.contains("fsl,") {
            "NXP i.MX PWM".to_string()
        } else if raw.contains("rockchip") {
            "Rockchip PWM".to_string()
        } else if raw.contains("allwinner") || raw.contains("sun") {
            "Allwinner PWM".to_string()
        } else if raw.contains("pca9685") {
            "NXP PCA9685 I2C PWM".to_string()
        } else if raw.contains("stm32") {
            "STM32 PWM".to_string()
        } else {
            raw.to_string()
        }
    }

    fn read_bool(&self, path: &PathBuf) -> bool {
        fs::read_to_string(path)
            .ok()
            .map(|s| s.trim() == "1")
            .unwrap_or(false)
    }

    fn read_u64(&self, path: &PathBuf) -> Option<u64> {
        fs::read_to_string(path)
            .ok()
            .and_then(|s| s.trim().parse().ok())
    }

    fn read_polarity(&self, path: &PathBuf) -> PwmPolarity {
        fs::read_to_string(path)
            .map(|s| PwmPolarity::from(s.as_str()))
            .unwrap_or(PwmPolarity::Normal)
    }
}

impl Default for PwmDiscovery {
    fn default() -> Self {
        Self::new()
    }
}

/// Format period/frequency for display
pub fn format_frequency(period_ns: u64) -> String {
    let freq_hz = 1_000_000_000.0 / period_ns as f64;
    if freq_hz >= 1_000_000.0 {
        format!("{:.2} MHz", freq_hz / 1_000_000.0)
    } else if freq_hz >= 1_000.0 {
        format!("{:.2} kHz", freq_hz / 1_000.0)
    } else {
        format!("{:.2} Hz", freq_hz)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enumerate_chips() {
        let mut discovery = PwmDiscovery::new();
        let chips = discovery.enumerate_chips();

        println!("Found {} PWM chips:", chips.len());
        for chip in &chips {
            println!("  pwmchip{}: {} channels", chip.chip_number, chip.npwm);
            if let Some(ref name) = chip.device_name {
                println!("    Device: {}", name);
            }
            if let Some(ref ctrl) = chip.controller_type {
                println!("    Controller: {}", ctrl);
            }
            println!(
                "    Exported: {}/{}",
                chip.exported_channel_count(),
                chip.npwm
            );

            for channel in &chip.channels {
                if channel.exported {
                    print!("    pwm{}: ", channel.channel_number);
                    if channel.enabled {
                        print!("enabled, ");
                    } else {
                        print!("disabled, ");
                    }
                    if let Some(period) = channel.period_ns {
                        print!("{}", format_frequency(period));
                    }
                    if let Some(duty) = channel.duty_cycle_percent() {
                        print!(", {:.1}% duty", duty);
                    }
                    println!();
                }
            }
        }
    }

    #[test]
    fn test_format_frequency() {
        // period_ns=1_000_000_000 (1 second) → freq=1 Hz
        assert_eq!(format_frequency(1_000_000_000), "1.00 Hz");
        // period_ns=1_000_000 (1 ms) → freq=1000 Hz = 1 kHz
        assert_eq!(format_frequency(1_000_000), "1.00 kHz");
        // period_ns=1_000 (1 μs) → freq=1,000,000 Hz = 1 MHz
        assert_eq!(format_frequency(1_000), "1.00 MHz");
        // period_ns=50 → freq=20,000,000 Hz = 20 MHz
        assert_eq!(format_frequency(50), "20.00 MHz");
    }

    #[test]
    fn test_pwm_polarity() {
        assert_eq!(PwmPolarity::from("normal"), PwmPolarity::Normal);
        assert_eq!(PwmPolarity::from("inversed"), PwmPolarity::Inverted);
        assert_eq!(PwmPolarity::from("inverted"), PwmPolarity::Inverted);
    }
}
