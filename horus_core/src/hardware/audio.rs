//! Audio device discovery for HORUS (Linux only).
//!
//! Scans for available audio devices via ALSA sysfs and procfs interfaces.

use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

/// Audio device type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AudioDeviceType {
    /// USB audio device
    Usb,
    /// PCI/PCIe audio card
    Pci,
    /// Platform/SoC audio (I2S, etc.)
    Platform,
    /// HDMI audio output
    Hdmi,
    /// Bluetooth audio
    Bluetooth,
    /// Virtual/loopback device
    Virtual,
    /// Unknown type
    Unknown,
}

/// Audio direction capabilities
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AudioDirection {
    /// Playback only
    Playback,
    /// Capture only
    Capture,
    /// Both playback and capture
    Duplex,
}

/// Audio stream information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AudioStream {
    /// Stream direction
    pub direction: AudioDirection,
    /// Number of channels
    pub channels: Option<u8>,
    /// Sample rate in Hz
    pub sample_rate: Option<u32>,
    /// Bit depth
    pub bit_depth: Option<u8>,
    /// Device path (e.g., "hw:0,0")
    pub device_path: String,
}

/// Audio card information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AudioCard {
    /// Card number (ALSA card index)
    pub card_number: u8,
    /// Card ID (short name)
    pub card_id: String,
    /// Card long name
    pub card_name: String,
    /// Device type
    pub device_type: AudioDeviceType,
    /// Driver name
    pub driver: Option<String>,
    /// Mixer name
    pub mixer_name: Option<String>,
    /// Audio devices on this card
    pub devices: Vec<AudioDevice>,
    /// Sysfs path
    pub sysfs_path: PathBuf,
}

impl AudioCard {
    /// Get device specification string
    pub fn device_spec(&self) -> String {
        format!("hw:{}", self.card_number)
    }

    /// Check if card has playback capability
    pub fn can_playback(&self) -> bool {
        self.devices.iter().any(|d| {
            matches!(
                d.direction,
                AudioDirection::Playback | AudioDirection::Duplex
            )
        })
    }

    /// Check if card has capture capability
    pub fn can_capture(&self) -> bool {
        self.devices.iter().any(|d| {
            matches!(
                d.direction,
                AudioDirection::Capture | AudioDirection::Duplex
            )
        })
    }
}

/// Audio device (PCM device within a card)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AudioDevice {
    /// Card number
    pub card_number: u8,
    /// Device number within card
    pub device_number: u8,
    /// Device ID
    pub device_id: String,
    /// Device name
    pub device_name: String,
    /// Direction capabilities
    pub direction: AudioDirection,
    /// Device path
    pub device_path: String,
}

impl AudioDevice {
    /// Get device specification string
    pub fn device_spec(&self) -> String {
        format!("hw:{},{}", self.card_number, self.device_number)
    }
}

/// Audio discovery
pub struct AudioDiscovery {
    /// Discovered audio cards
    cards: Vec<AudioCard>,
}

impl AudioDiscovery {
    /// Create a new audio discovery instance
    pub fn new() -> Self {
        Self { cards: Vec::new() }
    }

    /// Enumerate audio cards and devices
    pub fn enumerate_cards(&mut self) -> Vec<AudioCard> {
        self.cards.clear();

        // Read /proc/asound/cards for card list
        if let Ok(cards_content) = fs::read_to_string("/proc/asound/cards") {
            for line in cards_content.lines() {
                // Lines look like: " 0 [PCH            ]: HDA-Intel - HDA Intel PCH"
                if let Some(card) = self.parse_card_line(line) {
                    self.cards.push(card);
                }
            }
        }

        // Alternatively, scan sysfs
        if self.cards.is_empty() {
            if let Ok(entries) = fs::read_dir("/sys/class/sound") {
                for entry in entries.flatten() {
                    let name = entry.file_name().to_string_lossy().to_string();
                    if let Some(suffix) = name.strip_prefix("card") {
                        if let Ok(card_num) = suffix.parse::<u8>() {
                            if let Some(card) = self.probe_card_sysfs(card_num) {
                                self.cards.push(card);
                            }
                        }
                    }
                }
            }
        }

        // Sort by card number
        self.cards.sort_by_key(|c| c.card_number);
        self.cards.clone()
    }

    /// Get cached cards
    pub fn cards(&self) -> &[AudioCard] {
        &self.cards
    }

    /// Get total device count across all cards
    pub fn total_device_count(&self) -> usize {
        self.cards.iter().map(|c| c.devices.len()).sum()
    }

    /// Get cards with playback capability
    pub fn playback_cards(&self) -> Vec<&AudioCard> {
        self.cards.iter().filter(|c| c.can_playback()).collect()
    }

    /// Get cards with capture capability
    pub fn capture_cards(&self) -> Vec<&AudioCard> {
        self.cards.iter().filter(|c| c.can_capture()).collect()
    }

    fn parse_card_line(&self, line: &str) -> Option<AudioCard> {
        let line = line.trim();
        if line.is_empty() || line.starts_with('[') {
            return None;
        }

        // Parse format: " 0 [PCH            ]: HDA-Intel - HDA Intel PCH"
        let parts: Vec<&str> = line.splitn(2, '[').collect();
        if parts.len() < 2 {
            return None;
        }

        let card_num: u8 = parts[0].trim().parse().ok()?;
        let rest = parts[1];

        // Split by ]: to get ID and the rest
        let id_parts: Vec<&str> = rest.splitn(2, "]:").collect();
        if id_parts.len() < 2 {
            return None;
        }

        let card_id = id_parts[0].trim().to_string();
        let name_parts: Vec<&str> = id_parts[1].splitn(2, " - ").collect();
        let driver = name_parts.first().map(|s| s.trim().to_string());
        let card_name = name_parts
            .get(1)
            .map(|s| s.trim().to_string())
            .unwrap_or_else(|| driver.clone().unwrap_or_default());

        let sysfs_path = PathBuf::from(format!("/sys/class/sound/card{}", card_num));
        let device_type = self.detect_device_type(&sysfs_path, &card_id);
        let mixer_name = self.get_mixer_name(card_num);
        let devices = self.enumerate_devices(card_num);

        Some(AudioCard {
            card_number: card_num,
            card_id,
            card_name,
            device_type,
            driver,
            mixer_name,
            devices,
            sysfs_path,
        })
    }

    fn probe_card_sysfs(&self, card_number: u8) -> Option<AudioCard> {
        let sysfs_path = PathBuf::from(format!("/sys/class/sound/card{}", card_number));

        if !sysfs_path.exists() {
            return None;
        }

        // Read card ID
        let card_id = self
            .read_sysfs_string(&sysfs_path.join("id"))
            .unwrap_or_else(|| format!("card{}", card_number));

        // Read card number (to verify)
        let card_name = self
            .get_card_name_from_procfs(card_number)
            .unwrap_or_else(|| card_id.clone());

        // Get driver
        let driver = self.get_driver(&sysfs_path);

        // Detect type
        let device_type = self.detect_device_type(&sysfs_path, &card_id);

        // Get mixer
        let mixer_name = self.get_mixer_name(card_number);

        // Enumerate devices
        let devices = self.enumerate_devices(card_number);

        Some(AudioCard {
            card_number,
            card_id,
            card_name,
            device_type,
            driver,
            mixer_name,
            devices,
            sysfs_path,
        })
    }

    fn get_card_name_from_procfs(&self, card_number: u8) -> Option<String> {
        let path = format!("/proc/asound/card{}/id", card_number);
        self.read_file_string(&PathBuf::from(path))
    }

    fn detect_device_type(&self, sysfs_path: &Path, card_id: &str) -> AudioDeviceType {
        // Check device link for type hints
        let device_link = sysfs_path.join("device");
        if let Ok(target) = fs::read_link(&device_link) {
            let path_str = target.to_string_lossy().to_string();

            if path_str.contains("/usb") {
                return AudioDeviceType::Usb;
            }
            if path_str.contains("/pci") {
                // Check for HDMI
                if card_id.to_lowercase().contains("hdmi") {
                    return AudioDeviceType::Hdmi;
                }
                return AudioDeviceType::Pci;
            }
            if path_str.contains("/platform") || path_str.contains("/i2s") {
                return AudioDeviceType::Platform;
            }
        }

        // Check by card ID patterns
        let id_lower = card_id.to_lowercase();
        if id_lower.contains("hdmi") {
            AudioDeviceType::Hdmi
        } else if id_lower.contains("usb") {
            AudioDeviceType::Usb
        } else if id_lower.contains("loopback") || id_lower.contains("dummy") {
            AudioDeviceType::Virtual
        } else if id_lower.contains("bluetooth") || id_lower.contains("bt") {
            AudioDeviceType::Bluetooth
        } else {
            AudioDeviceType::Unknown
        }
    }

    fn get_driver(&self, sysfs_path: &Path) -> Option<String> {
        let driver_link = sysfs_path.join("device/driver");
        fs::read_link(&driver_link)
            .ok()
            .and_then(|p| p.file_name().map(|n| n.to_string_lossy().to_string()))
    }

    fn get_mixer_name(&self, card_number: u8) -> Option<String> {
        // Read from /proc/asound/cardX/id or similar
        let mixer_path = format!("/proc/asound/card{}/mixer", card_number);
        if PathBuf::from(&mixer_path).exists() {
            if let Ok(content) = fs::read_to_string(&mixer_path) {
                if let Some(first_line) = content.lines().next() {
                    return Some(first_line.trim().to_string());
                }
            }
        }
        None
    }

    fn enumerate_devices(&self, card_number: u8) -> Vec<AudioDevice> {
        let mut devices = Vec::new();

        // Read from /proc/asound/pcm
        if let Ok(pcm_content) = fs::read_to_string("/proc/asound/pcm") {
            for line in pcm_content.lines() {
                // Format: "00-00: ALC892 Analog : ALC892 Analog : playback 1 : capture 1"
                if let Some(device) = self.parse_pcm_line(line, card_number) {
                    devices.push(device);
                }
            }
        }

        // Fallback: scan sysfs for pcmC*D* entries
        if devices.is_empty() {
            if let Ok(entries) = fs::read_dir("/sys/class/sound") {
                for entry in entries.flatten() {
                    let name = entry.file_name().to_string_lossy().to_string();
                    // Format: pcmC0D0p (card 0, device 0, playback)
                    if name.starts_with(&format!("pcmC{}D", card_number)) {
                        if let Some(device) = self.probe_pcm_device(&name, card_number) {
                            // Avoid duplicates
                            if !devices
                                .iter()
                                .any(|d| d.device_number == device.device_number)
                            {
                                devices.push(device);
                            }
                        }
                    }
                }
            }
        }

        devices.sort_by_key(|d| d.device_number);
        devices
    }

    fn parse_pcm_line(&self, line: &str, target_card: u8) -> Option<AudioDevice> {
        // Format: "00-00: ALC892 Analog : ALC892 Analog : playback 1 : capture 1"
        let parts: Vec<&str> = line.split(':').collect();
        if parts.len() < 2 {
            return None;
        }

        // Parse card-device from first part
        let card_dev: Vec<&str> = parts[0].trim().split('-').collect();
        if card_dev.len() != 2 {
            return None;
        }

        let card_num: u8 = card_dev[0].parse().ok()?;
        if card_num != target_card {
            return None;
        }

        let device_num: u8 = card_dev[1].parse().ok()?;
        let device_id = parts
            .get(1)
            .map(|s| s.trim().to_string())
            .unwrap_or_default();
        let device_name = parts
            .get(2)
            .map(|s| s.trim().to_string())
            .unwrap_or_else(|| device_id.clone());

        // Determine direction from remaining parts
        let remaining = parts[3..].join(":");
        let has_playback = remaining.contains("playback");
        let has_capture = remaining.contains("capture");

        let direction = match (has_playback, has_capture) {
            (true, true) => AudioDirection::Duplex,
            (true, false) => AudioDirection::Playback,
            (false, true) => AudioDirection::Capture,
            (false, false) => AudioDirection::Playback, // Default assumption
        };

        Some(AudioDevice {
            card_number: card_num,
            device_number: device_num,
            device_id,
            device_name,
            direction,
            device_path: format!("hw:{},{}", card_num, device_num),
        })
    }

    fn probe_pcm_device(&self, name: &str, card_number: u8) -> Option<AudioDevice> {
        // Parse pcmC0D0p format
        let stripped = name.strip_prefix(&format!("pcmC{}D", card_number))?;

        // Get device number and direction suffix
        let (dev_str, dir_char) = if stripped.ends_with('p') || stripped.ends_with('c') {
            (&stripped[..stripped.len() - 1], stripped.chars().last())
        } else {
            (stripped, None)
        };

        let device_number: u8 = dev_str.parse().ok()?;

        // Read device info
        let sysfs_path = PathBuf::from(format!("/sys/class/sound/{}", name));
        let device_id = self
            .read_sysfs_string(&sysfs_path.join("id"))
            .unwrap_or_else(|| format!("pcm{}", device_number));

        let direction = match dir_char {
            Some('p') => AudioDirection::Playback,
            Some('c') => AudioDirection::Capture,
            _ => AudioDirection::Duplex,
        };

        Some(AudioDevice {
            card_number,
            device_number,
            device_id: device_id.clone(),
            device_name: device_id,
            direction,
            device_path: format!("hw:{},{}", card_number, device_number),
        })
    }

    fn read_sysfs_string(&self, path: &PathBuf) -> Option<String> {
        fs::read_to_string(path)
            .ok()
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
    }

    fn read_file_string(&self, path: &PathBuf) -> Option<String> {
        fs::read_to_string(path)
            .ok()
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty())
    }
}

impl Default for AudioDiscovery {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enumerate_cards() {
        let mut discovery = AudioDiscovery::new();
        let cards = discovery.enumerate_cards();

        println!("Found {} audio cards:", cards.len());
        for card in &cards {
            println!(
                "  Card {}: {} ({:?})",
                card.card_number, card.card_name, card.device_type
            );
            println!("    ID: {}", card.card_id);
            if let Some(ref driver) = card.driver {
                println!("    Driver: {}", driver);
            }
            println!(
                "    Capabilities: {}{}",
                if card.can_playback() { "playback " } else { "" },
                if card.can_capture() { "capture" } else { "" }
            );

            for device in &card.devices {
                println!(
                    "    Device {}: {} ({:?}) - {}",
                    device.device_number, device.device_name, device.direction, device.device_path
                );
            }
        }
    }

    #[test]
    fn test_playback_and_capture_cards() {
        let mut discovery = AudioDiscovery::new();
        discovery.enumerate_cards();

        println!("Playback cards: {}", discovery.playback_cards().len());
        println!("Capture cards: {}", discovery.capture_cards().len());
    }
}
