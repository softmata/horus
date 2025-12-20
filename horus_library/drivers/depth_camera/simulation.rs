//! Simulation Depth Camera driver
//!
//! Always-available simulation driver that generates synthetic RGB-D data.
//! Useful for testing and development without hardware.

use std::time::{SystemTime, UNIX_EPOCH};

use horus_core::driver::DriverStatus;
use horus_core::error::HorusResult;

use super::DepthCameraFrame;

/// Simulation depth camera driver configuration
#[derive(Debug, Clone)]
pub struct SimulationDepthCameraConfig {
    /// Sample rate in Hz
    pub sample_rate: f32,
    /// RGB resolution
    pub rgb_resolution: (u32, u32),
    /// Depth resolution
    pub depth_resolution: (u32, u32),
    /// Minimum depth in meters
    pub min_depth: f32,
    /// Maximum depth in meters
    pub max_depth: f32,
    /// Depth units (meters per unit)
    pub depth_units: f32,
}

impl Default for SimulationDepthCameraConfig {
    fn default() -> Self {
        Self {
            sample_rate: 30.0, // 30 Hz
            rgb_resolution: (640, 480),
            depth_resolution: (640, 480),
            min_depth: 0.3,     // 30cm
            max_depth: 10.0,    // 10m
            depth_units: 0.001, // 1mm per unit
        }
    }
}

/// Simulation Depth Camera driver
///
/// Generates synthetic RGB and depth data without requiring hardware.
/// Always available for testing and development.
pub struct SimulationDepthCameraDriver {
    config: SimulationDepthCameraConfig,
    status: DriverStatus,
    start_time: Option<u64>,
    frame_count: u64,
}

impl SimulationDepthCameraDriver {
    /// Create a new simulation depth camera driver with default configuration
    pub fn new() -> Self {
        Self {
            config: SimulationDepthCameraConfig::default(),
            status: DriverStatus::Uninitialized,
            start_time: None,
            frame_count: 0,
        }
    }

    /// Create a new simulation depth camera driver with custom configuration
    pub fn with_config(config: SimulationDepthCameraConfig) -> Self {
        Self {
            config,
            status: DriverStatus::Uninitialized,
            start_time: None,
            frame_count: 0,
        }
    }

    /// Set RGB resolution
    pub fn set_rgb_resolution(&mut self, width: u32, height: u32) {
        self.config.rgb_resolution = (width, height);
    }

    /// Set depth resolution
    pub fn set_depth_resolution(&mut self, width: u32, height: u32) {
        self.config.depth_resolution = (width, height);
    }

    /// Get the current timestamp in nanoseconds
    fn now_nanos(&self) -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64
    }

    /// Generate synthetic RGB data
    fn generate_rgb_data(&self) -> Vec<u8> {
        let (width, height) = self.config.rgb_resolution;
        let size = (width * height * 3) as usize;

        // Create a simple gradient pattern
        let mut data = vec![0u8; size];
        for y in 0..height {
            for x in 0..width {
                let idx = ((y * width + x) * 3) as usize;
                data[idx] = (x * 255 / width) as u8; // R: horizontal gradient
                data[idx + 1] = (y * 255 / height) as u8; // G: vertical gradient
                data[idx + 2] = ((self.frame_count * 3) % 256) as u8; // B: varying
            }
        }
        data
    }

    /// Generate synthetic depth data
    fn generate_depth_data(&self) -> Vec<u16> {
        let (width, height) = self.config.depth_resolution;
        let size = (width * height) as usize;

        // Create a radial depth pattern (closer in center, farther at edges)
        let mut data = vec![0u16; size];
        let cx = width as f32 / 2.0;
        let cy = height as f32 / 2.0;
        let max_dist = (cx * cx + cy * cy).sqrt();

        for y in 0..height {
            for x in 0..width {
                let dx = x as f32 - cx;
                let dy = y as f32 - cy;
                let dist = (dx * dx + dy * dy).sqrt();
                let normalized = dist / max_dist;

                // Map to depth range (closer in center)
                let depth_m = self.config.min_depth
                    + (self.config.max_depth - self.config.min_depth) * normalized;
                let depth_units = (depth_m / self.config.depth_units) as u16;

                data[(y * width + x) as usize] = depth_units;
            }
        }
        data
    }

    /// Initialize the driver
    pub fn init(&mut self) -> HorusResult<()> {
        self.start_time = Some(self.now_nanos());
        self.frame_count = 0;
        self.status = DriverStatus::Ready;
        Ok(())
    }

    /// Shutdown the driver
    pub fn shutdown(&mut self) -> HorusResult<()> {
        self.status = DriverStatus::Shutdown;
        Ok(())
    }

    /// Check if driver is available
    pub fn is_available(&self) -> bool {
        true // Simulation is always available
    }

    /// Get driver status
    pub fn status(&self) -> DriverStatus {
        self.status.clone()
    }

    /// Read depth camera frame
    pub fn read(&mut self) -> HorusResult<DepthCameraFrame> {
        if self.status != DriverStatus::Ready && self.status != DriverStatus::Running {
            return Err(horus_core::error::HorusError::driver(
                "Driver not initialized",
            ));
        }
        self.status = DriverStatus::Running;
        Ok(self.generate_frame())
    }

    /// Check if data is available
    pub fn has_data(&self) -> bool {
        matches!(self.status, DriverStatus::Ready | DriverStatus::Running)
    }

    /// Get sample rate
    pub fn sample_rate(&self) -> Option<f32> {
        Some(self.config.sample_rate)
    }

    /// Generate a complete frame
    fn generate_frame(&mut self) -> DepthCameraFrame {
        self.frame_count += 1;

        DepthCameraFrame {
            rgb_data: self.generate_rgb_data(),
            depth_data: self.generate_depth_data(),
            rgb_resolution: self.config.rgb_resolution,
            depth_resolution: self.config.depth_resolution,
            timestamp: self.now_nanos(),
        }
    }
}

impl Default for SimulationDepthCameraDriver {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulation_driver_lifecycle() {
        let mut driver = SimulationDepthCameraDriver::new();

        assert_eq!(driver.status(), DriverStatus::Uninitialized);
        assert!(driver.is_available());

        driver.init().unwrap();
        assert_eq!(driver.status(), DriverStatus::Ready);

        let frame = driver.read().unwrap();
        assert_eq!(driver.status(), DriverStatus::Running);

        // Check frame data
        assert!(!frame.rgb_data.is_empty());
        assert!(!frame.depth_data.is_empty());
        assert_eq!(frame.rgb_resolution, (640, 480));
        assert_eq!(frame.depth_resolution, (640, 480));

        driver.shutdown().unwrap();
        assert_eq!(driver.status(), DriverStatus::Shutdown);
    }
}
