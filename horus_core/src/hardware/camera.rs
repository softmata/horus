//! Camera/Video4Linux discovery for HORUS (Linux only).
//!
//! Discovers video capture devices using the V4L2 API.

use std::fs;
use std::os::unix::io::AsRawFd;
use std::path::{Path, PathBuf};

/// Camera type classification
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CameraType {
    /// Standard RGB camera
    Rgb,
    /// Depth camera
    Depth,
    /// Infrared camera
    Infrared,
    /// Thermal camera
    Thermal,
    /// USB webcam
    Webcam,
    /// CSI/MIPI camera
    Csi,
    /// Unknown type
    Unknown,
}

impl CameraType {
    /// Get human-readable name
    pub fn name(&self) -> &'static str {
        match self {
            Self::Rgb => "RGB Camera",
            Self::Depth => "Depth Camera",
            Self::Infrared => "Infrared Camera",
            Self::Thermal => "Thermal Camera",
            Self::Webcam => "USB Webcam",
            Self::Csi => "CSI Camera",
            Self::Unknown => "Unknown Camera",
        }
    }
}

/// Video format information
#[derive(Debug, Clone)]
pub struct VideoFormat {
    /// FourCC code
    pub fourcc: String,
    /// Description
    pub description: String,
    /// Supported resolutions
    pub resolutions: Vec<(u32, u32)>,
}

/// Camera information
#[derive(Debug, Clone)]
pub struct Camera {
    /// Device path (e.g., /dev/video0)
    pub device_path: PathBuf,
    /// Device name
    pub name: String,
    /// Driver name
    pub driver: String,
    /// Card name
    pub card: Option<String>,
    /// Bus info
    pub bus_info: Option<String>,
    /// Camera type
    pub camera_type: CameraType,
    /// Capabilities flags
    pub capabilities: u32,
    /// Is this a capture device
    pub is_capture: bool,
    /// Is this a metadata device
    pub is_metadata: bool,
    /// Supported formats
    pub formats: Vec<VideoFormat>,
}

impl Camera {
    /// Get display name
    pub fn display_name(&self) -> String {
        self.card.clone().unwrap_or_else(|| self.name.clone())
    }

    /// Get device number
    pub fn device_number(&self) -> Option<u32> {
        let name = self.device_path.file_name()?.to_string_lossy();
        name.strip_prefix("video")?.parse().ok()
    }

    /// Check if device supports streaming
    pub fn supports_streaming(&self) -> bool {
        const V4L2_CAP_STREAMING: u32 = 0x04000000;
        (self.capabilities & V4L2_CAP_STREAMING) != 0
    }

}

/// Camera discovery
pub struct CameraDiscovery {
    /// Discovered cameras
    cameras: Vec<Camera>,
}

impl CameraDiscovery {
    /// Create a new camera discovery instance
    pub fn new() -> Self {
        Self {
            cameras: Vec::new(),
        }
    }

    /// Enumerate cameras
    pub fn enumerate(&mut self) -> Vec<Camera> {
        self.cameras.clear();

        // Scan /dev for video* devices
        if let Ok(entries) = fs::read_dir("/dev") {
            for entry in entries.flatten() {
                let name = entry.file_name().to_string_lossy().to_string();
                if name.starts_with("video") {
                    if let Some(camera) = self.probe_device(&entry.path()) {
                        // Only add capture devices (not metadata or output)
                        if camera.is_capture {
                            self.cameras.push(camera);
                        }
                    }
                }
            }
        }

        // Sort by device number
        self.cameras
            .sort_by_key(|c| c.device_number().unwrap_or(999));
        self.cameras.clone()
    }

    /// Get cached cameras
    pub fn cameras(&self) -> &[Camera] {
        &self.cameras
    }

    /// Find camera by name
    pub fn find_by_name(&self, name: &str) -> Option<&Camera> {
        self.cameras.iter().find(|c| {
            c.name.contains(name) || c.card.as_ref().is_some_and(|card| card.contains(name))
        })
    }

    /// Find cameras by type
    pub fn find_by_type(&self, camera_type: CameraType) -> Vec<&Camera> {
        self.cameras
            .iter()
            .filter(|c| c.camera_type == camera_type)
            .collect()
    }

    /// Find depth cameras
    pub fn find_depth_cameras(&self) -> Vec<&Camera> {
        self.find_by_type(CameraType::Depth)
    }

    fn probe_device(&self, path: &Path) -> Option<Camera> {
        let file = fs::File::open(path).ok()?;
        let fd = file.as_raw_fd();

        // V4L2 VIDIOC_QUERYCAP
        const VIDIOC_QUERYCAP: u64 = 0x80685600;

        #[repr(C)]
        struct V4l2Capability {
            driver: [u8; 16],
            card: [u8; 32],
            bus_info: [u8; 32],
            version: u32,
            capabilities: u32,
            device_caps: u32,
            reserved: [u32; 3],
        }

        let mut cap = V4l2Capability {
            driver: [0; 16],
            card: [0; 32],
            bus_info: [0; 32],
            version: 0,
            capabilities: 0,
            device_caps: 0,
            reserved: [0; 3],
        };

        // SAFETY: fd is a valid open file descriptor. cap is a properly sized struct for VIDIOC_QUERYCAP.
        let result = unsafe { libc::ioctl(fd, VIDIOC_QUERYCAP as libc::c_ulong, &mut cap) };

        if result < 0 {
            return None;
        }

        let driver = String::from_utf8_lossy(&cap.driver)
            .trim_end_matches('\0')
            .to_string();

        let card = String::from_utf8_lossy(&cap.card)
            .trim_end_matches('\0')
            .to_string();

        let bus_info = String::from_utf8_lossy(&cap.bus_info)
            .trim_end_matches('\0')
            .to_string();

        // Capability flags
        const V4L2_CAP_VIDEO_CAPTURE: u32 = 0x00000001;
        const V4L2_CAP_VIDEO_CAPTURE_MPLANE: u32 = 0x00001000;
        const V4L2_CAP_META_CAPTURE: u32 = 0x00800000;
        const V4L2_CAP_DEVICE_CAPS: u32 = 0x80000000;

        // Use device_caps if available, otherwise capabilities
        let actual_caps = if (cap.capabilities & V4L2_CAP_DEVICE_CAPS) != 0 {
            cap.device_caps
        } else {
            cap.capabilities
        };

        let is_capture = (actual_caps & V4L2_CAP_VIDEO_CAPTURE) != 0
            || (actual_caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE) != 0;

        let is_metadata = (actual_caps & V4L2_CAP_META_CAPTURE) != 0;

        // Determine camera type
        let camera_type = self.classify_camera(&driver, &card, &bus_info);

        // Query supported formats
        let formats = self.query_formats(&file);

        Some(Camera {
            device_path: path.to_path_buf(),
            name: path.file_name()?.to_string_lossy().to_string(),
            driver,
            card: if card.is_empty() { None } else { Some(card) },
            bus_info: if bus_info.is_empty() {
                None
            } else {
                Some(bus_info)
            },
            camera_type,
            capabilities: actual_caps,
            is_capture,
            is_metadata,
            formats,
        })
    }

    fn classify_camera(&self, driver: &str, card: &str, bus_info: &str) -> CameraType {
        let combined = format!("{} {} {}", driver, card, bus_info).to_lowercase();

        // Depth cameras
        if combined.contains("realsense")
            || combined.contains("orbbec")
            || combined.contains("depth")
            || combined.contains("zed")
            || combined.contains("kinect")
        {
            return CameraType::Depth;
        }

        // Infrared cameras
        if combined.contains("infrared") || combined.contains(" ir") || combined.contains("-ir") {
            return CameraType::Infrared;
        }

        // Thermal cameras
        if combined.contains("thermal") || combined.contains("flir") || combined.contains("lepton")
        {
            return CameraType::Thermal;
        }

        // CSI cameras
        if combined.contains("csi")
            || combined.contains("unicam")
            || combined.contains("tegra-video")
            || combined.contains("imx")
            || combined.contains("ov5647")
            || combined.contains("arducam")
        {
            return CameraType::Csi;
        }

        // USB webcams
        if combined.contains("uvc") || bus_info.starts_with("usb") {
            return CameraType::Webcam;
        }

        CameraType::Unknown
    }

    fn query_formats(&self, file: &fs::File) -> Vec<VideoFormat> {
        let fd = file.as_raw_fd();

        const VIDIOC_ENUM_FMT: u64 = 0xC0405602;
        const V4L2_BUF_TYPE_VIDEO_CAPTURE: u32 = 1;

        #[repr(C)]
        struct V4l2Fmtdesc {
            index: u32,
            buf_type: u32,
            flags: u32,
            description: [u8; 32],
            pixelformat: u32,
            reserved: [u32; 4],
        }

        let mut formats = Vec::new();
        let mut index = 0u32;

        loop {
            let mut fmt = V4l2Fmtdesc {
                index,
                buf_type: V4L2_BUF_TYPE_VIDEO_CAPTURE,
                flags: 0,
                description: [0; 32],
                pixelformat: 0,
                reserved: [0; 4],
            };

            // SAFETY: fd is a valid open file descriptor. fmt is a properly sized struct for VIDIOC_ENUM_FMT.
            let result = unsafe { libc::ioctl(fd, VIDIOC_ENUM_FMT as libc::c_ulong, &mut fmt) };

            if result < 0 {
                break;
            }

            let description = String::from_utf8_lossy(&fmt.description)
                .trim_end_matches('\0')
                .to_string();

            // Convert pixelformat to FourCC string
            let fourcc = format!(
                "{}{}{}{}",
                (fmt.pixelformat & 0xFF) as u8 as char,
                ((fmt.pixelformat >> 8) & 0xFF) as u8 as char,
                ((fmt.pixelformat >> 16) & 0xFF) as u8 as char,
                ((fmt.pixelformat >> 24) & 0xFF) as u8 as char
            );

            formats.push(VideoFormat {
                fourcc,
                description,
                resolutions: Vec::new(), // Could enumerate frame sizes here
            });

            index += 1;
            if index > 100 {
                // Safety limit
                break;
            }
        }

        formats
    }
}

impl Default for CameraDiscovery {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enumerate_cameras() {
        let mut discovery = CameraDiscovery::new();
        let cameras = discovery.enumerate();

        println!("Found {} cameras:", cameras.len());
        for camera in &cameras {
            println!(
                "  {} - {} ({})",
                camera.device_path.display(),
                camera.display_name(),
                camera.camera_type.name()
            );
            println!("    Driver: {}", camera.driver);
            if let Some(bus) = &camera.bus_info {
                println!("    Bus: {}", bus);
            }
            println!(
                "    Formats: {:?}",
                camera.formats.iter().map(|f| &f.fourcc).collect::<Vec<_>>()
            );
        }
    }

    #[test]
    fn test_find_depth_cameras() {
        let mut discovery = CameraDiscovery::new();
        discovery.enumerate();
        let depth = discovery.find_depth_cameras();

        println!("Found {} depth cameras:", depth.len());
        for camera in depth {
            println!("  {}", camera.display_name());
        }
    }
}
