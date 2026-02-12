//! Device specification for tensor placement
//!
//! Defines where tensor data resides (CPU or CUDA GPU).

use std::fmt;

/// Device where tensor data resides
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Device {
    /// CPU memory (accessible via shared memory)
    #[default]
    Cpu,
    /// CUDA GPU with device index
    Cuda(u32),
}

impl Device {
    /// CPU device constant
    pub const CPU: Device = Device::Cpu;

    /// CUDA device 0
    pub const CUDA0: Device = Device::Cuda(0);

    /// Check if device is CPU
    #[inline]
    pub const fn is_cpu(&self) -> bool {
        matches!(self, Device::Cpu)
    }

    /// Check if device is CUDA
    #[inline]
    pub const fn is_cuda(&self) -> bool {
        matches!(self, Device::Cuda(_))
    }

    /// Get CUDA device index, or None for CPU
    #[inline]
    pub const fn cuda_index(&self) -> Option<u32> {
        match self {
            Device::Cpu => None,
            Device::Cuda(idx) => Some(*idx),
        }
    }

    /// Create CUDA device with given index
    #[inline]
    pub const fn cuda(index: u32) -> Self {
        Device::Cuda(index)
    }

    /// Parse device from string (e.g., "cpu", "cuda", "cuda:0", "cuda:1")
    pub fn parse(s: &str) -> Option<Self> {
        let s = s.to_lowercase();
        match s.as_str() {
            "cpu" => Some(Device::Cpu),
            "cuda" | "gpu" => Some(Device::Cuda(0)),
            _ if s.starts_with("cuda:") || s.starts_with("gpu:") => {
                let idx_str = s.split(':').nth(1)?;
                let idx: u32 = idx_str.parse().ok()?;
                Some(Device::Cuda(idx))
            }
            _ => None,
        }
    }

    /// Convert to DLPack device type code
    #[inline]
    pub const fn to_dlpack_device_type(&self) -> i32 {
        match self {
            Device::Cpu => 1,     // kDLCPU
            Device::Cuda(_) => 2, // kDLCUDA
        }
    }

    /// Convert to DLPack device id
    #[inline]
    pub const fn to_dlpack_device_id(&self) -> i32 {
        match self {
            Device::Cpu => 0,
            Device::Cuda(idx) => *idx as i32,
        }
    }

    /// Create from DLPack device type and id
    pub const fn from_dlpack(device_type: i32, device_id: i32) -> Option<Self> {
        match device_type {
            1 => Some(Device::Cpu), // kDLCPU
            2 => {
                // Validate device_id is non-negative before casting to u32
                if device_id < 0 {
                    None
                } else {
                    Some(Device::Cuda(device_id as u32)) // kDLCUDA
                }
            }
            _ => None,
        }
    }
}

impl fmt::Display for Device {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Device::Cpu => write!(f, "cpu"),
            Device::Cuda(idx) => write!(f, "cuda:{}", idx),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_device_parse() {
        assert_eq!(Device::parse("cpu"), Some(Device::Cpu));
        assert_eq!(Device::parse("cuda"), Some(Device::Cuda(0)));
        assert_eq!(Device::parse("cuda:0"), Some(Device::Cuda(0)));
        assert_eq!(Device::parse("cuda:1"), Some(Device::Cuda(1)));
        assert_eq!(Device::parse("gpu:2"), Some(Device::Cuda(2)));
        assert_eq!(Device::parse("invalid"), None);
    }

    #[test]
    fn test_device_dlpack() {
        assert_eq!(Device::Cpu.to_dlpack_device_type(), 1);
        assert_eq!(Device::Cuda(0).to_dlpack_device_type(), 2);
        assert_eq!(Device::Cuda(1).to_dlpack_device_id(), 1);

        assert_eq!(Device::from_dlpack(1, 0), Some(Device::Cpu));
        assert_eq!(Device::from_dlpack(2, 1), Some(Device::Cuda(1)));
    }
}
