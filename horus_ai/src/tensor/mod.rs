//! Tensor types and utilities
//!
//! This module provides tensor data types and descriptors.

// Re-export TensorDtype from horus_types (replaces old dtype submodule)
pub use horus_types::TensorDtype;

use horus_types::Device;

/// Tensor descriptor containing metadata about a tensor
///
/// This is a lightweight struct that describes a tensor's properties
/// without owning the actual data.
#[derive(Clone, Debug)]
pub struct TensorDescriptor {
    /// Shape of the tensor
    pub shape: Vec<u64>,
    /// Strides in bytes for each dimension
    pub strides: Vec<u64>,
    /// Data type
    pub dtype: TensorDtype,
    /// Device where data resides
    pub device: Device,
    /// Total size in bytes
    pub size_bytes: u64,
    /// Data pointer (for reference, not ownership)
    pub data_ptr: u64,
}

impl TensorDescriptor {
    /// Create a new tensor descriptor
    pub fn new(shape: &[u64], dtype: TensorDtype, device: Device) -> Self {
        let ndim = shape.len();
        let mut strides = vec![0u64; ndim];

        // Calculate row-major (C-contiguous) strides
        if ndim > 0 {
            strides[ndim - 1] = dtype.size_bytes() as u64;
            for i in (0..ndim - 1).rev() {
                strides[i] = strides[i + 1] * shape[i + 1];
            }
        }

        let num_elements: u64 = shape.iter().product();
        let size_bytes = num_elements * dtype.size_bytes() as u64;

        Self {
            shape: shape.to_vec(),
            strides,
            dtype,
            device,
            size_bytes,
            data_ptr: 0,
        }
    }

    /// Number of dimensions
    #[inline]
    pub fn ndim(&self) -> usize {
        self.shape.len()
    }

    /// Total number of elements
    #[inline]
    pub fn numel(&self) -> u64 {
        self.shape.iter().product()
    }

    /// Check if tensor is contiguous (row-major)
    pub fn is_contiguous(&self) -> bool {
        if self.shape.is_empty() {
            return true;
        }

        let mut expected = self.dtype.size_bytes() as u64;
        for i in (0..self.shape.len()).rev() {
            if self.strides[i] != expected {
                return false;
            }
            expected *= self.shape[i];
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tensor_descriptor() {
        let desc = TensorDescriptor::new(&[2, 3, 4], TensorDtype::F32, Device::cpu());

        assert_eq!(desc.ndim(), 3);
        assert_eq!(desc.numel(), 24);
        assert_eq!(desc.size_bytes, 24 * 4); // 24 elements * 4 bytes
        assert!(desc.is_contiguous());

        // Strides for [2, 3, 4] with f32:
        // stride[2] = 4 (element size)
        // stride[1] = 4 * 4 = 16
        // stride[0] = 16 * 3 = 48
        assert_eq!(desc.strides, vec![48, 16, 4]);
    }
}
