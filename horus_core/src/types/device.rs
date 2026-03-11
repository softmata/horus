//! Device specification for tensor placement
//!
//! Defines where tensor data resides (CPU or CUDA GPU).
//! Pod-safe repr(C) struct with unlimited GPU index support.

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};
use std::fmt;

/// Device where tensor data resides
///
/// Pod-safe, repr(C), 8-byte struct supporting unlimited CUDA device indices.
/// Replaces the old `TensorDevice` enum which was limited to 4 GPUs.
///
/// # Layout
/// ```text
/// device_type (1 byte): 0=CPU, 1=CUDA
/// _pad (3 bytes): alignment padding
/// device_id (4 bytes): GPU index (unlimited)
/// Total: 8 bytes
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Pod, Zeroable)]
pub struct Device {
    /// Device type: 0 = CPU, 1 = CUDA
    pub device_type: u8,
    /// Alignment padding
    pub _pad: [u8; 3],
    /// Device index (GPU number for CUDA, 0 for CPU)
    pub device_id: u32,
}

/// Device type constants
pub const DEVICE_TYPE_CPU: u8 = 0;
pub const DEVICE_TYPE_CUDA: u8 = 1;

impl Default for Device {
    fn default() -> Self {
        Self::CPU
    }
}

impl Device {
    /// CPU device constant
    pub const CPU: Device = Device {
        device_type: DEVICE_TYPE_CPU,
        _pad: [0; 3],
        device_id: 0,
    };

    /// CUDA device 0 constant
    pub const CUDA0: Device = Device {
        device_type: DEVICE_TYPE_CUDA,
        _pad: [0; 3],
        device_id: 0,
    };

    /// Create a CPU device
    #[inline]
    pub const fn cpu() -> Self {
        Self::CPU
    }

    /// Create a CUDA device with given index
    #[inline]
    pub const fn cuda(index: u32) -> Self {
        Device {
            device_type: DEVICE_TYPE_CUDA,
            _pad: [0; 3],
            device_id: index,
        }
    }

    /// Check if device is CPU
    #[inline]
    pub const fn is_cpu(&self) -> bool {
        self.device_type == DEVICE_TYPE_CPU
    }

    /// Check if device is CUDA
    #[inline]
    pub const fn is_cuda(&self) -> bool {
        self.device_type == DEVICE_TYPE_CUDA
    }

    /// Parse device from string (e.g., "cpu", "cuda", "cuda:0", "cuda:1")
    pub fn parse(s: &str) -> Option<Self> {
        let s = s.to_lowercase();
        match s.as_str() {
            "cpu" => Some(Device::cpu()),
            "cuda" | "gpu" => Some(Device::cuda(0)),
            _ if s.starts_with("cuda:") || s.starts_with("gpu:") => {
                let idx_str = s.split(':').nth(1)?;
                let idx: u32 = idx_str.parse().ok()?;
                Some(Device::cuda(idx))
            }
            _ => None,
        }
    }

    /// Convert to DLPack device type code
    #[inline]
    pub const fn to_dlpack_device_type(&self) -> i32 {
        match self.device_type {
            DEVICE_TYPE_CPU => 1,  // kDLCPU
            DEVICE_TYPE_CUDA => 2, // kDLCUDA
            _ => 1,                // default to CPU for unknown types
        }
    }

    /// Convert to DLPack device id
    #[inline]
    pub const fn to_dlpack_device_id(&self) -> i32 {
        if self.is_cpu() {
            0
        } else {
            self.device_id as i32
        }
    }

    /// Create from DLPack device type and id
    pub const fn from_dlpack(device_type: i32, device_id: i32) -> Option<Self> {
        match device_type {
            1 => Some(Device::cpu()), // kDLCPU
            2 => {
                if device_id < 0 {
                    None
                } else {
                    Some(Device::cuda(device_id as u32)) // kDLCUDA
                }
            }
            _ => None,
        }
    }
}

impl fmt::Display for Device {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_cpu() {
            write!(f, "cpu")
        } else {
            write!(f, "cuda:{}", self.device_id)
        }
    }
}

// Custom Serde: serialize as string ("cpu", "cuda:0", "cuda:1", etc.)
impl Serialize for Device {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        serializer.serialize_str(&self.to_string())
    }
}

impl<'de> Deserialize<'de> for Device {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let s = String::deserialize(deserializer)?;
        Device::parse(&s).ok_or_else(|| serde::de::Error::custom(format!("invalid device: {}", s)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_device_size_and_alignment() {
        assert_eq!(std::mem::size_of::<Device>(), 8);
        // Alignment must be 4 (from u32 device_id field)
        assert_eq!(std::mem::align_of::<Device>(), 4);
        // 8 bytes = power of 2, fits in a single cache line operation
        assert!(std::mem::size_of::<Device>().is_power_of_two());
    }

    #[test]
    fn test_device_constructors() {
        let cpu = Device::cpu();
        assert!(cpu.is_cpu());
        assert!(!cpu.is_cuda());
        assert_eq!(cpu.device_id, 0);

        let cuda0 = Device::cuda(0);
        assert!(!cuda0.is_cpu());
        assert!(cuda0.is_cuda());
        assert_eq!(cuda0.device_id, 0);

        let cuda7 = Device::cuda(7);
        assert!(cuda7.is_cuda());
        assert_eq!(cuda7.device_id, 7);

        // Verify constants match constructors
        assert_eq!(Device::CPU, Device::cpu());
        assert_eq!(Device::CUDA0, Device::cuda(0));

        // Verify copy semantics: mutating a copy doesn't affect original
        let mut copy = cpu;
        copy.device_type = DEVICE_TYPE_CUDA;
        assert!(cpu.is_cpu(), "original must be unaffected by copy mutation");
        assert!(copy.is_cuda());
    }

    #[test]
    fn test_device_parse() {
        assert_eq!(Device::parse("cpu"), Some(Device::cpu()));
        assert_eq!(Device::parse("cuda"), Some(Device::cuda(0)));
        assert_eq!(Device::parse("cuda:0"), Some(Device::cuda(0)));
        assert_eq!(Device::parse("cuda:1"), Some(Device::cuda(1)));
        assert_eq!(Device::parse("cuda:15"), Some(Device::cuda(15)));
        assert_eq!(Device::parse("gpu:2"), Some(Device::cuda(2)));
        assert_eq!(Device::parse("CPU"), Some(Device::cpu()));
        assert_eq!(Device::parse("CUDA:3"), Some(Device::cuda(3)));
        assert_eq!(Device::parse("invalid"), None);
    }

    #[test]
    fn test_device_display() {
        assert_eq!(format!("{}", Device::cpu()), "cpu");
        assert_eq!(format!("{}", Device::cuda(0)), "cuda:0");
        assert_eq!(format!("{}", Device::cuda(3)), "cuda:3");
        assert_eq!(format!("{}", Device::cuda(15)), "cuda:15");
    }

    #[test]
    fn test_device_dlpack() {
        assert_eq!(Device::cpu().to_dlpack_device_type(), 1);
        assert_eq!(Device::cuda(0).to_dlpack_device_type(), 2);
        assert_eq!(Device::cuda(1).to_dlpack_device_id(), 1);
        assert_eq!(Device::cpu().to_dlpack_device_id(), 0);

        assert_eq!(Device::from_dlpack(1, 0), Some(Device::cpu()));
        assert_eq!(Device::from_dlpack(2, 0), Some(Device::cuda(0)));
        assert_eq!(Device::from_dlpack(2, 1), Some(Device::cuda(1)));
        assert_eq!(Device::from_dlpack(2, -1), None);
        assert_eq!(Device::from_dlpack(99, 0), None);
    }

    #[test]
    fn test_device_pod_soundness() {
        let device = Device::cuda(42);
        let bytes = bytemuck::bytes_of(&device);
        assert_eq!(bytes.len(), 8);
        assert_eq!(bytes[0], DEVICE_TYPE_CUDA);
        // device_id = 42 in little-endian at offset 4
        assert_eq!(
            u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
            42
        );
    }

    #[test]
    fn test_device_serde_roundtrip() {
        for device in [
            Device::cpu(),
            Device::cuda(0),
            Device::cuda(1),
            Device::cuda(15),
        ] {
            let json = serde_json::to_string(&device).unwrap();
            let recovered: Device = serde_json::from_str(&json).unwrap();
            assert_eq!(recovered, device, "roundtrip failed for {}", device);
        }
    }

    #[test]
    fn test_default_is_cpu() {
        let d = Device::default();
        assert_eq!(d, Device::cpu());
        // Default must differ from any CUDA device
        assert_ne!(d, Device::cuda(0));
        assert_ne!(d, Device::cuda(1));
    }

    #[test]
    fn test_device_clone_and_debug() {
        let cuda3 = Device::cuda(3);
        let cloned = cuda3;
        assert_eq!(cloned, cuda3);
        // Debug output must contain identifying information
        let dbg = format!("{:?}", cuda3);
        assert!(dbg.contains("device_type"), "Debug must show device_type field");
        assert!(dbg.contains("device_id"), "Debug must show device_id field");
    }

    #[test]
    fn test_device_hash_distinguishes_devices() {
        use std::collections::HashSet;
        let devices: HashSet<Device> = [
            Device::cpu(),
            Device::cuda(0),
            Device::cuda(1),
            Device::cuda(2),
        ]
        .into_iter()
        .collect();
        assert_eq!(devices.len(), 4, "all four devices must hash uniquely");
    }

    #[test]
    fn test_device_padding_zeroed() {
        // Ensure padding bytes are always zero (important for Pod byte equality)
        let cpu = Device::cpu();
        assert_eq!(cpu._pad, [0, 0, 0]);
        let cuda = Device::cuda(42);
        assert_eq!(cuda._pad, [0, 0, 0]);
    }
}
