//! GPIO chip and line discovery for HORUS (Linux only).
//!
//! Uses the Linux GPIO character device interface to enumerate GPIO chips
//! and their available lines.

use std::fs;
use std::os::unix::io::AsRawFd;
use std::path::{Path, PathBuf};

/// GPIO chip information
#[derive(Debug, Clone)]
pub struct GpioChip {
    /// Chip name (e.g., "gpiochip0")
    pub name: String,
    /// Device path
    pub device_path: PathBuf,
    /// Chip label
    pub label: String,
    /// Number of lines
    pub num_lines: u32,
    /// Lines (if enumerated)
    pub lines: Vec<GpioLine>,
}

impl GpioChip {
    /// Get the device path as a string
    pub fn device(&self) -> String {
        self.device_path.display().to_string()
    }
}

/// GPIO line information
#[derive(Debug, Clone)]
pub struct GpioLine {
    /// Line offset within the chip
    pub offset: u32,
    /// Line name (if set in device tree)
    pub name: Option<String>,
    /// Consumer name (if in use)
    pub consumer: Option<String>,
    /// Whether the line is in use
    pub in_use: bool,
    /// Line direction
    pub direction: GpioDirection,
    /// Active low flag
    pub active_low: bool,
}

/// GPIO direction
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GpioDirection {
    /// Input
    Input,
    /// Output
    Output,
    /// Unknown/not configured
    Unknown,
}

/// GPIO discovery
pub struct GpioDiscovery {
    /// Discovered chips
    chips: Vec<GpioChip>,
}

impl GpioDiscovery {
    /// Create a new GPIO discovery instance
    pub fn new() -> Self {
        Self { chips: Vec::new() }
    }

    /// Enumerate GPIO chips
    pub fn enumerate(&mut self) -> Vec<GpioChip> {
        self.chips.clear();

        // Scan /dev for gpiochip* devices
        if let Ok(entries) = fs::read_dir("/dev") {
            for entry in entries.flatten() {
                let name = entry.file_name().to_string_lossy().to_string();
                if name.starts_with("gpiochip") {
                    let device_path = entry.path();

                    if let Some(chip) = self.read_chip_info(&device_path, &name) {
                        self.chips.push(chip);
                    }
                }
            }
        }

        // Sort by name
        self.chips.sort_by(|a, b| a.name.cmp(&b.name));
        self.chips.clone()
    }

    /// Enumerate chips with line details
    pub fn enumerate_with_lines(&mut self) -> Vec<GpioChip> {
        self.enumerate();

        // Collect chip info first to avoid borrow conflicts
        let chip_info: Vec<(PathBuf, u32)> = self
            .chips
            .iter()
            .map(|c| (c.device_path.clone(), c.num_lines))
            .collect();

        // Read lines for each chip
        for (i, (device_path, num_lines)) in chip_info.into_iter().enumerate() {
            if let Ok(lines) = self.read_lines(&device_path, num_lines) {
                self.chips[i].lines = lines;
            }
        }

        self.chips.clone()
    }

    /// Get cached chips
    pub fn chips(&self) -> &[GpioChip] {
        &self.chips
    }

    /// Find chip by name
    pub fn find_by_name(&self, name: &str) -> Option<&GpioChip> {
        self.chips.iter().find(|c| c.name == name)
    }

    /// Find chip by label
    pub fn find_by_label(&self, label: &str) -> Option<&GpioChip> {
        self.chips.iter().find(|c| c.label == label)
    }

    /// Get total number of GPIO lines across all chips
    pub fn total_lines(&self) -> u32 {
        self.chips.iter().map(|c| c.num_lines).sum()
    }

    fn read_chip_info(&self, device_path: &Path, name: &str) -> Option<GpioChip> {
        // Open the device
        let file = fs::File::open(device_path).ok()?;
        let fd = file.as_raw_fd();

        // GPIO ioctl constants
        const GPIO_GET_CHIPINFO_IOCTL: u64 = 0x8044B401;

        #[repr(C)]
        struct GpioChipInfo {
            name: [u8; 32],
            label: [u8; 32],
            lines: u32,
        }

        let mut info = GpioChipInfo {
            name: [0; 32],
            label: [0; 32],
            lines: 0,
        };

        let result =
            unsafe { libc::ioctl(fd, GPIO_GET_CHIPINFO_IOCTL as libc::c_ulong, &mut info) };

        if result < 0 {
            // Fallback: try to read from sysfs
            return self.read_chip_info_sysfs(device_path, name);
        }

        let label = String::from_utf8_lossy(&info.label)
            .trim_end_matches('\0')
            .to_string();

        Some(GpioChip {
            name: name.to_string(),
            device_path: device_path.to_path_buf(),
            label: if label.is_empty() {
                name.to_string()
            } else {
                label
            },
            num_lines: info.lines,
            lines: Vec::new(),
        })
    }

    fn read_chip_info_sysfs(&self, device_path: &Path, name: &str) -> Option<GpioChip> {
        // Try to get info from sysfs
        let chip_num = name.strip_prefix("gpiochip")?;
        let sysfs_path = format!("/sys/class/gpio/gpiochip{}", chip_num);

        if !Path::new(&sysfs_path).exists() {
            return None;
        }

        let label = fs::read_to_string(format!("{}/label", sysfs_path))
            .unwrap_or_else(|_| name.to_string())
            .trim()
            .to_string();

        let ngpio: u32 = fs::read_to_string(format!("{}/ngpio", sysfs_path))
            .ok()?
            .trim()
            .parse()
            .ok()?;

        Some(GpioChip {
            name: name.to_string(),
            device_path: device_path.to_path_buf(),
            label,
            num_lines: ngpio,
            lines: Vec::new(),
        })
    }

    fn read_lines(&self, device_path: &Path, num_lines: u32) -> Result<Vec<GpioLine>, String> {
        let file = fs::File::open(device_path)
            .map_err(|e| format!("Failed to open GPIO device: {}", e))?;
        let fd = file.as_raw_fd();

        const GPIO_GET_LINEINFO_IOCTL: u64 = 0xC048B402;

        #[repr(C)]
        struct GpioLineInfo {
            line_offset: u32,
            flags: u32,
            name: [u8; 32],
            consumer: [u8; 32],
        }

        // Line flags
        const GPIOLINE_FLAG_KERNEL: u32 = 1 << 0;
        const GPIOLINE_FLAG_IS_OUT: u32 = 1 << 1;
        const GPIOLINE_FLAG_ACTIVE_LOW: u32 = 1 << 2;

        let mut lines = Vec::new();

        for offset in 0..num_lines {
            let mut info = GpioLineInfo {
                line_offset: offset,
                flags: 0,
                name: [0; 32],
                consumer: [0; 32],
            };

            let result =
                unsafe { libc::ioctl(fd, GPIO_GET_LINEINFO_IOCTL as libc::c_ulong, &mut info) };

            if result < 0 {
                // Skip lines we can't read
                continue;
            }

            let name = String::from_utf8_lossy(&info.name)
                .trim_end_matches('\0')
                .to_string();

            let consumer = String::from_utf8_lossy(&info.consumer)
                .trim_end_matches('\0')
                .to_string();

            let in_use = (info.flags & GPIOLINE_FLAG_KERNEL) != 0 || !consumer.is_empty();
            let direction = if (info.flags & GPIOLINE_FLAG_IS_OUT) != 0 {
                GpioDirection::Output
            } else if in_use {
                GpioDirection::Input
            } else {
                GpioDirection::Unknown
            };
            let active_low = (info.flags & GPIOLINE_FLAG_ACTIVE_LOW) != 0;

            lines.push(GpioLine {
                offset,
                name: if name.is_empty() { None } else { Some(name) },
                consumer: if consumer.is_empty() {
                    None
                } else {
                    Some(consumer)
                },
                in_use,
                direction,
                active_low,
            });
        }

        Ok(lines)
    }
}

impl Default for GpioDiscovery {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enumerate_gpio() {
        let mut discovery = GpioDiscovery::new();
        let chips = discovery.enumerate();

        println!("Found {} GPIO chips:", chips.len());
        for chip in &chips {
            println!(
                "  {} ({}) - {} lines",
                chip.name, chip.label, chip.num_lines
            );
        }

        println!("Total lines: {}", discovery.total_lines());
    }

    #[test]
    #[ignore] // May require specific hardware
    fn test_enumerate_with_lines() {
        let mut discovery = GpioDiscovery::new();
        let chips = discovery.enumerate_with_lines();

        for chip in &chips {
            println!("{} ({}):", chip.name, chip.label);
            for line in &chip.lines {
                if line.name.is_some() || line.consumer.is_some() {
                    println!(
                        "  Line {}: {} ({})",
                        line.offset,
                        line.name.as_deref().unwrap_or("unnamed"),
                        if line.in_use {
                            line.consumer.as_deref().unwrap_or("in use")
                        } else {
                            "free"
                        }
                    );
                }
            }
        }
    }
}
