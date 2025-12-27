//! Platform detection for HORUS hardware discovery.
//!
//! Detects the host platform (Raspberry Pi, Jetson, BeagleBone, etc.)
//! and provides platform-specific capabilities information.

use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

/// Detected platform type
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Platform {
    // Raspberry Pi family
    RaspberryPi3,
    RaspberryPi3B,
    RaspberryPi3BPlus,
    RaspberryPi4,
    RaspberryPi4B,
    RaspberryPi5,
    RaspberryPiZero,
    RaspberryPiZero2W,
    RaspberryPiPico,

    // NVIDIA Jetson family
    JetsonNano,
    JetsonNanoOrin,
    JetsonXavierNx,
    JetsonOrinNano,
    JetsonOrinNx,
    JetsonAgxOrin,

    // BeagleBone family
    BeagleBoneBlack,
    BeagleBoneAi,
    BeagleBoneAi64,
    BeaglePlay,
    PocketBeagle,

    // Orange Pi family
    OrangePi5,
    OrangePi5Plus,
    OrangePiZero2,
    OrangePi3Lts,

    // Radxa / Rock family
    RockPi4,
    Rock5A,
    Rock5B,

    // Khadas
    KhadasVim3,
    KhadasVim4,

    // Odroid
    OdroidXu4,
    OdroidN2,
    OdroidC4,
    OdroidM1,

    // x86 boards
    IntelNuc,
    UpBoard,
    UpSquared,
    LattePanda,
    LattePandaDelta,

    // Pine64
    PineA64,
    PineH64,
    Pinebook,
    PinebookPro,

    // Milk-V / RISC-V
    MilkVDuo,
    MilkVMars,
    VisionFive2,

    // Generic
    GenericArm64,
    GenericArmv7,
    GenericX86_64,
    GenericRiscV64,
    GenericLinux,
    Unknown,
}

impl Platform {
    /// Get human-readable platform name
    pub fn name(&self) -> &'static str {
        match self {
            Platform::RaspberryPi3 => "Raspberry Pi 3",
            Platform::RaspberryPi3B => "Raspberry Pi 3 Model B",
            Platform::RaspberryPi3BPlus => "Raspberry Pi 3 Model B+",
            Platform::RaspberryPi4 => "Raspberry Pi 4",
            Platform::RaspberryPi4B => "Raspberry Pi 4 Model B",
            Platform::RaspberryPi5 => "Raspberry Pi 5",
            Platform::RaspberryPiZero => "Raspberry Pi Zero",
            Platform::RaspberryPiZero2W => "Raspberry Pi Zero 2 W",
            Platform::RaspberryPiPico => "Raspberry Pi Pico",

            Platform::JetsonNano => "NVIDIA Jetson Nano",
            Platform::JetsonNanoOrin => "NVIDIA Jetson Orin Nano",
            Platform::JetsonXavierNx => "NVIDIA Jetson Xavier NX",
            Platform::JetsonOrinNano => "NVIDIA Jetson Orin Nano",
            Platform::JetsonOrinNx => "NVIDIA Jetson Orin NX",
            Platform::JetsonAgxOrin => "NVIDIA Jetson AGX Orin",

            Platform::BeagleBoneBlack => "BeagleBone Black",
            Platform::BeagleBoneAi => "BeagleBone AI",
            Platform::BeagleBoneAi64 => "BeagleBone AI-64",
            Platform::BeaglePlay => "BeaglePlay",
            Platform::PocketBeagle => "PocketBeagle",

            Platform::OrangePi5 => "Orange Pi 5",
            Platform::OrangePi5Plus => "Orange Pi 5 Plus",
            Platform::OrangePiZero2 => "Orange Pi Zero 2",
            Platform::OrangePi3Lts => "Orange Pi 3 LTS",

            Platform::RockPi4 => "Rock Pi 4",
            Platform::Rock5A => "Rock 5A",
            Platform::Rock5B => "Rock 5B",

            Platform::KhadasVim3 => "Khadas VIM3",
            Platform::KhadasVim4 => "Khadas VIM4",

            Platform::OdroidXu4 => "Odroid XU4",
            Platform::OdroidN2 => "Odroid N2",
            Platform::OdroidC4 => "Odroid C4",
            Platform::OdroidM1 => "Odroid M1",

            Platform::IntelNuc => "Intel NUC",
            Platform::UpBoard => "UP Board",
            Platform::UpSquared => "UP Squared",
            Platform::LattePanda => "LattePanda",
            Platform::LattePandaDelta => "LattePanda Delta",

            Platform::PineA64 => "Pine A64",
            Platform::PineH64 => "Pine H64",
            Platform::Pinebook => "Pinebook",
            Platform::PinebookPro => "Pinebook Pro",

            Platform::MilkVDuo => "Milk-V Duo",
            Platform::MilkVMars => "Milk-V Mars",
            Platform::VisionFive2 => "VisionFive 2",

            Platform::GenericArm64 => "Generic ARM64 Linux",
            Platform::GenericArmv7 => "Generic ARMv7 Linux",
            Platform::GenericX86_64 => "Generic x86_64 Linux",
            Platform::GenericRiscV64 => "Generic RISC-V 64 Linux",
            Platform::GenericLinux => "Generic Linux",
            Platform::Unknown => "Unknown Platform",
        }
    }

    /// Check if this is a Raspberry Pi variant
    pub fn is_raspberry_pi(&self) -> bool {
        matches!(
            self,
            Platform::RaspberryPi3
                | Platform::RaspberryPi3B
                | Platform::RaspberryPi3BPlus
                | Platform::RaspberryPi4
                | Platform::RaspberryPi4B
                | Platform::RaspberryPi5
                | Platform::RaspberryPiZero
                | Platform::RaspberryPiZero2W
                | Platform::RaspberryPiPico
        )
    }

    /// Check if this is an NVIDIA Jetson variant
    pub fn is_jetson(&self) -> bool {
        matches!(
            self,
            Platform::JetsonNano
                | Platform::JetsonNanoOrin
                | Platform::JetsonXavierNx
                | Platform::JetsonOrinNano
                | Platform::JetsonOrinNx
                | Platform::JetsonAgxOrin
        )
    }

    /// Check if this is a BeagleBone variant
    pub fn is_beaglebone(&self) -> bool {
        matches!(
            self,
            Platform::BeagleBoneBlack
                | Platform::BeagleBoneAi
                | Platform::BeagleBoneAi64
                | Platform::BeaglePlay
                | Platform::PocketBeagle
        )
    }

    /// Check if this platform has CUDA support
    pub fn has_cuda(&self) -> bool {
        self.is_jetson()
    }
}

/// GPU type available on the platform
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GpuType {
    // Raspberry Pi
    VideoCore4,
    VideoCore5,
    VideoCore6,
    VideoCore7,

    // NVIDIA
    Maxwell, // Jetson Nano
    Volta,   // Xavier
    Ampere,  // Orin

    // Mali
    MaliG52,
    MaliG57,
    MaliG610,
    MaliG710,

    // Intel
    IntelUhd,
    IntelIris,

    // AMD
    AmdVega,
    AmdRdna,

    // Other
    None,
    Unknown,
}

/// Platform capabilities and available interfaces
#[derive(Debug, Clone, Default)]
pub struct PlatformCapabilities {
    /// Available GPIO chips
    pub gpio_chips: Vec<String>,
    /// Total GPIO pins available
    pub gpio_pins: u32,
    /// Available I2C bus numbers
    pub i2c_buses: Vec<u8>,
    /// Available SPI bus numbers
    pub spi_buses: Vec<u8>,
    /// Available UART/serial ports
    pub uart_ports: Vec<String>,
    /// Available PWM chips
    pub pwm_chips: Vec<u8>,
    /// Number of ADC channels
    pub adc_channels: u8,
    /// Available CAN interfaces
    pub can_interfaces: Vec<String>,
    /// Has CSI camera connector
    pub has_camera_csi: bool,
    /// Has DSI display connector
    pub has_display_dsi: bool,
    /// Has PCIe interface
    pub has_pcie: bool,
    /// Has NVMe support
    pub has_nvme: bool,
    /// GPU type
    pub gpu_type: Option<GpuType>,
    /// CUDA available
    pub cuda_available: bool,
    /// Has Tensor Cores
    pub tensor_cores: bool,
    /// NPU/AI accelerator available
    pub npu_available: bool,
    /// NPU performance in TOPS
    pub npu_tops: Option<f32>,
    /// BeagleBone PRU available
    pub pru_available: bool,
    /// RP2040 PIO available
    pub pio_available: bool,
    /// Number of CPU cores
    pub cpu_cores: u32,
    /// CPU architecture
    pub cpu_arch: String,
    /// Total RAM in MB
    pub ram_mb: u32,
}

/// Platform detector
pub struct PlatformDetector;

impl PlatformDetector {
    /// Detect the current platform
    pub fn detect() -> Platform {
        // Try device tree model first (most reliable on ARM)
        if let Some(platform) = Self::detect_from_device_tree() {
            return platform;
        }

        // Check for Jetson-specific files
        if Path::new("/etc/nv_tegra_release").exists() {
            return Self::detect_jetson();
        }

        // Check for x86 boards via DMI
        if let Some(platform) = Self::detect_from_dmi() {
            return platform;
        }

        // Fall back to architecture detection
        Self::detect_generic()
    }

    /// Detect from device tree model
    fn detect_from_device_tree() -> Option<Platform> {
        // Try /proc/device-tree/model
        if let Ok(model) = fs::read_to_string("/proc/device-tree/model") {
            let model = model.trim_end_matches('\0').to_lowercase();
            return Self::parse_device_tree_model(&model);
        }

        // Try compatible string
        if let Ok(compatible) = fs::read_to_string("/proc/device-tree/compatible") {
            let compatible = compatible.replace('\0', " ").to_lowercase();
            return Self::parse_compatible(&compatible);
        }

        None
    }

    /// Parse device tree model string
    fn parse_device_tree_model(model: &str) -> Option<Platform> {
        // Raspberry Pi
        if model.contains("raspberry pi 5") {
            return Some(Platform::RaspberryPi5);
        }
        if model.contains("raspberry pi 4 model b") {
            return Some(Platform::RaspberryPi4B);
        }
        if model.contains("raspberry pi 4") {
            return Some(Platform::RaspberryPi4);
        }
        if model.contains("raspberry pi 3 model b+")
            || model.contains("raspberry pi 3 model b plus")
        {
            return Some(Platform::RaspberryPi3BPlus);
        }
        if model.contains("raspberry pi 3 model b") {
            return Some(Platform::RaspberryPi3B);
        }
        if model.contains("raspberry pi 3") {
            return Some(Platform::RaspberryPi3);
        }
        if model.contains("raspberry pi zero 2") {
            return Some(Platform::RaspberryPiZero2W);
        }
        if model.contains("raspberry pi zero") {
            return Some(Platform::RaspberryPiZero);
        }

        // BeagleBone
        if model.contains("beaglebone ai-64") || model.contains("beaglebone-ai-64") {
            return Some(Platform::BeagleBoneAi64);
        }
        if model.contains("beaglebone ai") {
            return Some(Platform::BeagleBoneAi);
        }
        if model.contains("beaglebone black") {
            return Some(Platform::BeagleBoneBlack);
        }
        if model.contains("beagleplay") {
            return Some(Platform::BeaglePlay);
        }
        if model.contains("pocketbeagle") {
            return Some(Platform::PocketBeagle);
        }

        // Orange Pi
        if model.contains("orange pi 5 plus") {
            return Some(Platform::OrangePi5Plus);
        }
        if model.contains("orange pi 5") {
            return Some(Platform::OrangePi5);
        }
        if model.contains("orange pi zero 2") {
            return Some(Platform::OrangePiZero2);
        }

        // Rock / Radxa
        if model.contains("rock 5b") || model.contains("rock5b") {
            return Some(Platform::Rock5B);
        }
        if model.contains("rock 5a") || model.contains("rock5a") {
            return Some(Platform::Rock5A);
        }
        if model.contains("rock pi 4") || model.contains("rockpi4") {
            return Some(Platform::RockPi4);
        }

        // Khadas
        if model.contains("khadas vim4") {
            return Some(Platform::KhadasVim4);
        }
        if model.contains("khadas vim3") {
            return Some(Platform::KhadasVim3);
        }

        // Odroid
        if model.contains("odroid-xu4") || model.contains("odroid xu4") {
            return Some(Platform::OdroidXu4);
        }
        if model.contains("odroid-n2") || model.contains("odroid n2") {
            return Some(Platform::OdroidN2);
        }
        if model.contains("odroid-c4") || model.contains("odroid c4") {
            return Some(Platform::OdroidC4);
        }
        if model.contains("odroid-m1") || model.contains("odroid m1") {
            return Some(Platform::OdroidM1);
        }

        // Pine64
        if model.contains("pinebook pro") {
            return Some(Platform::PinebookPro);
        }
        if model.contains("pinebook") {
            return Some(Platform::Pinebook);
        }
        if model.contains("pine h64") {
            return Some(Platform::PineH64);
        }
        if model.contains("pine a64") {
            return Some(Platform::PineA64);
        }

        // RISC-V
        if model.contains("visionfive 2") || model.contains("visionfive2") {
            return Some(Platform::VisionFive2);
        }
        if model.contains("milk-v mars") {
            return Some(Platform::MilkVMars);
        }
        if model.contains("milk-v duo") {
            return Some(Platform::MilkVDuo);
        }

        None
    }

    /// Parse compatible string
    fn parse_compatible(compatible: &str) -> Option<Platform> {
        if compatible.contains("brcm,bcm2711") {
            return Some(Platform::RaspberryPi4);
        }
        if compatible.contains("brcm,bcm2837") {
            return Some(Platform::RaspberryPi3);
        }
        if compatible.contains("brcm,bcm2712") {
            return Some(Platform::RaspberryPi5);
        }
        if compatible.contains("rockchip,rk3588") {
            // Could be Orange Pi 5, Rock 5, etc.
            return Some(Platform::Rock5B);
        }
        if compatible.contains("rockchip,rk3399") {
            return Some(Platform::RockPi4);
        }
        if compatible.contains("amlogic,a311d") {
            return Some(Platform::KhadasVim3);
        }
        if compatible.contains("ti,am5729") {
            return Some(Platform::BeagleBoneAi);
        }
        if compatible.contains("ti,j721e") {
            return Some(Platform::BeagleBoneAi64);
        }
        if compatible.contains("starfive,jh7110") {
            return Some(Platform::VisionFive2);
        }

        None
    }

    /// Detect Jetson model
    fn detect_jetson() -> Platform {
        // Read tegra release to determine model
        if let Ok(release) = fs::read_to_string("/etc/nv_tegra_release") {
            let release = release.to_lowercase();

            if release.contains("r36") || release.contains("orin") {
                // Check for specific Orin model
                if let Ok(model) = fs::read_to_string("/proc/device-tree/model") {
                    let model = model.to_lowercase();
                    if model.contains("agx orin") {
                        return Platform::JetsonAgxOrin;
                    }
                    if model.contains("orin nx") {
                        return Platform::JetsonOrinNx;
                    }
                    if model.contains("orin nano") {
                        return Platform::JetsonOrinNano;
                    }
                }
                return Platform::JetsonOrinNano;
            }
            if release.contains("r35") || release.contains("r34") {
                return Platform::JetsonXavierNx;
            }
            if release.contains("r32") {
                return Platform::JetsonNano;
            }
        }

        Platform::JetsonNano
    }

    /// Detect from DMI (x86 systems)
    fn detect_from_dmi() -> Option<Platform> {
        // Check board name
        if let Ok(board) = fs::read_to_string("/sys/class/dmi/id/board_name") {
            let board = board.trim().to_lowercase();

            if board.contains("nuc") {
                return Some(Platform::IntelNuc);
            }
            if board.contains("up-apl") || board.contains("upboard") {
                return Some(Platform::UpBoard);
            }
            if board.contains("up-squared") {
                return Some(Platform::UpSquared);
            }
            if board.contains("lattepanda") {
                if board.contains("delta") {
                    return Some(Platform::LattePandaDelta);
                }
                return Some(Platform::LattePanda);
            }
        }

        None
    }

    /// Detect generic platform based on architecture
    fn detect_generic() -> Platform {
        #[cfg(target_arch = "x86_64")]
        return Platform::GenericX86_64;

        #[cfg(target_arch = "aarch64")]
        return Platform::GenericArm64;

        #[cfg(target_arch = "arm")]
        return Platform::GenericArmv7;

        #[cfg(target_arch = "riscv64")]
        return Platform::GenericRiscV64;

        #[cfg(not(any(
            target_arch = "x86_64",
            target_arch = "aarch64",
            target_arch = "arm",
            target_arch = "riscv64"
        )))]
        return Platform::Unknown;
    }

    /// Get platform capabilities
    pub fn capabilities(platform: &Platform) -> PlatformCapabilities {
        match platform {
            Platform::RaspberryPi5 => PlatformCapabilities {
                gpio_chips: vec!["gpiochip0".into(), "gpiochip10".into()],
                gpio_pins: 28,
                i2c_buses: vec![1, 3],
                spi_buses: vec![0, 1],
                uart_ports: vec![
                    "/dev/ttyAMA0".into(),
                    "/dev/ttyAMA2".into(),
                    "/dev/ttyAMA3".into(),
                    "/dev/ttyAMA4".into(),
                ],
                pwm_chips: vec![0, 1, 2, 3],
                has_camera_csi: true,
                has_display_dsi: true,
                has_pcie: true,
                has_nvme: true,
                gpu_type: Some(GpuType::VideoCore7),
                cpu_cores: 4,
                cpu_arch: "aarch64".into(),
                ram_mb: 8192,
                ..Default::default()
            },
            Platform::RaspberryPi4 | Platform::RaspberryPi4B => PlatformCapabilities {
                gpio_chips: vec!["gpiochip0".into()],
                gpio_pins: 28,
                i2c_buses: vec![1],
                spi_buses: vec![0, 1],
                uart_ports: vec!["/dev/ttyAMA0".into(), "/dev/ttyS0".into()],
                pwm_chips: vec![0, 1],
                has_camera_csi: true,
                has_display_dsi: true,
                gpu_type: Some(GpuType::VideoCore6),
                cpu_cores: 4,
                cpu_arch: "aarch64".into(),
                ram_mb: 4096,
                ..Default::default()
            },
            Platform::RaspberryPi3 | Platform::RaspberryPi3B | Platform::RaspberryPi3BPlus => {
                PlatformCapabilities {
                    gpio_chips: vec!["gpiochip0".into()],
                    gpio_pins: 28,
                    i2c_buses: vec![1],
                    spi_buses: vec![0, 1],
                    uart_ports: vec!["/dev/ttyAMA0".into(), "/dev/ttyS0".into()],
                    pwm_chips: vec![0],
                    has_camera_csi: true,
                    has_display_dsi: true,
                    gpu_type: Some(GpuType::VideoCore4),
                    cpu_cores: 4,
                    cpu_arch: "aarch64".into(),
                    ram_mb: 1024,
                    ..Default::default()
                }
            }
            Platform::RaspberryPiZero2W => PlatformCapabilities {
                gpio_chips: vec!["gpiochip0".into()],
                gpio_pins: 28,
                i2c_buses: vec![1],
                spi_buses: vec![0, 1],
                uart_ports: vec!["/dev/ttyAMA0".into()],
                pwm_chips: vec![0],
                has_camera_csi: true,
                gpu_type: Some(GpuType::VideoCore4),
                cpu_cores: 4,
                cpu_arch: "aarch64".into(),
                ram_mb: 512,
                ..Default::default()
            },
            Platform::JetsonAgxOrin => PlatformCapabilities {
                gpio_chips: vec!["gpiochip0".into(), "gpiochip1".into()],
                gpio_pins: 40,
                i2c_buses: vec![0, 1, 2, 7, 8],
                spi_buses: vec![0, 1, 2],
                uart_ports: vec!["/dev/ttyTHS0".into(), "/dev/ttyTHS1".into()],
                can_interfaces: vec!["can0".into(), "can1".into()],
                pwm_chips: vec![0, 1, 2],
                has_camera_csi: true,
                has_pcie: true,
                has_nvme: true,
                gpu_type: Some(GpuType::Ampere),
                cuda_available: true,
                tensor_cores: true,
                npu_available: true,
                npu_tops: Some(275.0),
                cpu_cores: 12,
                cpu_arch: "aarch64".into(),
                ram_mb: 65536,
                ..Default::default()
            },
            Platform::JetsonOrinNano | Platform::JetsonOrinNx => PlatformCapabilities {
                gpio_chips: vec!["gpiochip0".into(), "gpiochip1".into()],
                gpio_pins: 40,
                i2c_buses: vec![1, 7, 8],
                spi_buses: vec![0, 1],
                uart_ports: vec!["/dev/ttyTHS0".into(), "/dev/ttyTHS1".into()],
                pwm_chips: vec![0, 1],
                has_camera_csi: true,
                has_pcie: true,
                has_nvme: true,
                gpu_type: Some(GpuType::Ampere),
                cuda_available: true,
                tensor_cores: true,
                npu_available: true,
                npu_tops: Some(70.0),
                cpu_cores: 6,
                cpu_arch: "aarch64".into(),
                ram_mb: 8192,
                ..Default::default()
            },
            Platform::JetsonNano => PlatformCapabilities {
                gpio_chips: vec!["gpiochip0".into()],
                gpio_pins: 40,
                i2c_buses: vec![0, 1],
                spi_buses: vec![0, 1],
                uart_ports: vec!["/dev/ttyTHS1".into()],
                pwm_chips: vec![0],
                has_camera_csi: true,
                gpu_type: Some(GpuType::Maxwell),
                cuda_available: true,
                cpu_cores: 4,
                cpu_arch: "aarch64".into(),
                ram_mb: 4096,
                ..Default::default()
            },
            Platform::BeagleBoneBlack => PlatformCapabilities {
                gpio_chips: vec![
                    "gpiochip0".into(),
                    "gpiochip1".into(),
                    "gpiochip2".into(),
                    "gpiochip3".into(),
                ],
                gpio_pins: 92,
                i2c_buses: vec![1, 2],
                spi_buses: vec![0, 1],
                uart_ports: vec![
                    "/dev/ttyO0".into(),
                    "/dev/ttyO1".into(),
                    "/dev/ttyO2".into(),
                    "/dev/ttyO4".into(),
                ],
                can_interfaces: vec!["can0".into(), "can1".into()],
                pwm_chips: vec![0, 2, 4, 7],
                adc_channels: 7,
                pru_available: true,
                cpu_cores: 1,
                cpu_arch: "armv7".into(),
                ram_mb: 512,
                ..Default::default()
            },
            Platform::BeagleBoneAi64 => PlatformCapabilities {
                gpio_chips: vec!["gpiochip0".into(), "gpiochip1".into()],
                gpio_pins: 92,
                i2c_buses: vec![1, 2, 3, 4, 5, 6],
                spi_buses: vec![0, 1, 2, 3],
                uart_ports: vec!["/dev/ttyS2".into(), "/dev/ttyS3".into()],
                can_interfaces: vec!["can0".into(), "can1".into()],
                pwm_chips: vec![0, 1, 2],
                adc_channels: 7,
                has_pcie: true,
                npu_available: true,
                npu_tops: Some(8.0),
                cpu_cores: 4,
                cpu_arch: "aarch64".into(),
                ram_mb: 4096,
                ..Default::default()
            },
            Platform::OrangePi5 | Platform::OrangePi5Plus => PlatformCapabilities {
                gpio_chips: vec!["gpiochip0".into(), "gpiochip1".into()],
                gpio_pins: 40,
                i2c_buses: vec![0, 3, 5],
                spi_buses: vec![0, 1],
                uart_ports: vec!["/dev/ttyS0".into(), "/dev/ttyS2".into()],
                pwm_chips: vec![0, 1],
                has_pcie: true,
                has_nvme: true,
                gpu_type: Some(GpuType::MaliG610),
                npu_available: true,
                npu_tops: Some(6.0),
                cpu_cores: 8,
                cpu_arch: "aarch64".into(),
                ram_mb: 8192,
                ..Default::default()
            },
            Platform::Rock5B => PlatformCapabilities {
                gpio_chips: vec!["gpiochip0".into(), "gpiochip1".into()],
                gpio_pins: 40,
                i2c_buses: vec![0, 3, 5, 7],
                spi_buses: vec![0, 1],
                uart_ports: vec!["/dev/ttyS0".into(), "/dev/ttyS2".into()],
                pwm_chips: vec![0, 1, 2],
                has_pcie: true,
                has_nvme: true,
                gpu_type: Some(GpuType::MaliG610),
                npu_available: true,
                npu_tops: Some(6.0),
                cpu_cores: 8,
                cpu_arch: "aarch64".into(),
                ram_mb: 16384,
                ..Default::default()
            },
            Platform::KhadasVim3 => PlatformCapabilities {
                gpio_chips: vec!["gpiochip0".into()],
                gpio_pins: 40,
                i2c_buses: vec![1, 2, 3, 4],
                spi_buses: vec![0, 1],
                uart_ports: vec!["/dev/ttyS0".into(), "/dev/ttyS4".into()],
                pwm_chips: vec![0, 1],
                has_pcie: true,
                gpu_type: Some(GpuType::MaliG52),
                npu_available: true,
                npu_tops: Some(5.0),
                cpu_cores: 6,
                cpu_arch: "aarch64".into(),
                ram_mb: 4096,
                ..Default::default()
            },
            // Generic x86_64
            Platform::GenericX86_64
            | Platform::IntelNuc
            | Platform::UpBoard
            | Platform::LattePanda => PlatformCapabilities {
                cpu_arch: "x86_64".into(),
                cpu_cores: num_cpus(),
                ram_mb: total_ram_mb(),
                ..Default::default()
            },
            // Default for unknown platforms
            _ => PlatformCapabilities {
                cpu_arch: std::env::consts::ARCH.into(),
                cpu_cores: num_cpus(),
                ram_mb: total_ram_mb(),
                ..Default::default()
            },
        }
    }

    /// Get actual system memory in MB
    pub fn system_memory_mb() -> u32 {
        total_ram_mb()
    }

    /// Get actual CPU count
    pub fn cpu_count() -> u32 {
        num_cpus()
    }
}

/// Get number of CPUs
fn num_cpus() -> u32 {
    std::thread::available_parallelism()
        .map(|p| p.get() as u32)
        .unwrap_or(1)
}

/// Get total RAM in MB
fn total_ram_mb() -> u32 {
    #[cfg(target_os = "linux")]
    {
        if let Ok(meminfo) = fs::read_to_string("/proc/meminfo") {
            for line in meminfo.lines() {
                if line.starts_with("MemTotal:") {
                    let parts: Vec<&str> = line.split_whitespace().collect();
                    if parts.len() >= 2 {
                        if let Ok(kb) = parts[1].parse::<u64>() {
                            return (kb / 1024) as u32;
                        }
                    }
                }
            }
        }
    }
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_platform_detection() {
        let platform = PlatformDetector::detect();
        println!("Detected platform: {:?} ({})", platform, platform.name());
        // Should not panic
    }

    #[test]
    fn test_platform_capabilities() {
        let platform = PlatformDetector::detect();
        let caps = PlatformDetector::capabilities(&platform);
        println!("CPU cores: {}", caps.cpu_cores);
        println!("RAM: {} MB", caps.ram_mb);
        println!("CPU arch: {}", caps.cpu_arch);
    }

    #[test]
    fn test_parse_rpi_models() {
        assert_eq!(
            PlatformDetector::parse_device_tree_model("raspberry pi 4 model b rev 1.4"),
            Some(Platform::RaspberryPi4B)
        );
        assert_eq!(
            PlatformDetector::parse_device_tree_model("raspberry pi 5"),
            Some(Platform::RaspberryPi5)
        );
        assert_eq!(
            PlatformDetector::parse_device_tree_model("raspberry pi zero 2 w rev 1.0"),
            Some(Platform::RaspberryPiZero2W)
        );
    }

    #[test]
    fn test_system_info() {
        let cores = PlatformDetector::cpu_count();
        let ram = PlatformDetector::system_memory_mb();
        assert!(cores > 0);
        // RAM might be 0 on non-Linux systems
        println!("System: {} cores, {} MB RAM", cores, ram);
    }
}
