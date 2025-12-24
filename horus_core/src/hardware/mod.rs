//! # HORUS Hardware Discovery
//!
//! Unified hardware discovery system for HORUS.
//!
//! This module provides comprehensive hardware detection across various platforms
//! and bus types, including:
//!
//! - **Platform Detection**: Identify the host platform (Raspberry Pi, Jetson, etc.)
//! - **USB Discovery**: Enumerate USB devices with VID/PID identification
//! - **Serial Discovery**: Find serial ports and identify connected devices
//! - **I2C Discovery**: Scan I2C buses for connected devices
//! - **GPIO Discovery**: Enumerate available GPIO chips and lines
//! - **Camera Discovery**: Find video capture devices
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use horus_core::hardware::{HardwareDiscovery, Platform};
//!
//! // Create discovery instance
//! let discovery = HardwareDiscovery::new().unwrap();
//!
//! // Get platform info
//! println!("Platform: {:?}", discovery.platform());
//!
//! // Scan all hardware
//! let report = discovery.scan_all();
//! for device in report.usb_devices {
//!     println!("USB: {} (VID:{:04X} PID:{:04X})",
//!         device.product.as_deref().unwrap_or("Unknown"),
//!         device.vendor_id,
//!         device.product_id
//!     );
//! }
//! ```

mod database;
mod discovery;
mod platform;
mod serial;
mod usb;

#[cfg(target_os = "linux")]
mod i2c;

#[cfg(target_os = "linux")]
mod gpio;

#[cfg(target_os = "linux")]
mod camera;

// Re-export main types
pub use database::{DeviceCategory, DeviceDatabase, DeviceInfo, DriverMatch, MatchConfidence};
pub use discovery::{DiscoveredDevice, DiscoveryReport, HardwareDiscovery};
pub use platform::{GpuType, Platform, PlatformCapabilities, PlatformDetector};
pub use serial::{SerialDiscovery, SerialPort, SerialPortType};
pub use usb::{UsbDevice, UsbDiscovery};

#[cfg(target_os = "linux")]
pub use i2c::{I2cBus, I2cDevice, I2cDiscovery};

#[cfg(target_os = "linux")]
pub use gpio::{GpioChip, GpioDiscovery, GpioLine};

#[cfg(target_os = "linux")]
pub use camera::{Camera, CameraDiscovery, CameraType, VideoFormat};
