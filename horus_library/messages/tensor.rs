//! Zero-copy tensor types for HORUS
//!
//! All types are re-exported from `horus_types` (the canonical source).
//! This module provides backward-compatible access.

// Re-export all tensor types from horus_types
pub use horus_types::{
    dlpack_codes, Device, HorusTensor, TensorDtype, CUDA_IPC_HANDLE_SIZE, MAX_TENSOR_DIMS,
};

/// Backward-compatible alias for the old TensorDevice enum
pub type TensorDevice = Device;

// LogSummary impl is in horus_core::core::node (where the trait is defined)

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tensor_dtype_sizes() {
        assert_eq!(TensorDtype::F32.element_size(), 4);
        assert_eq!(TensorDtype::F64.element_size(), 8);
        assert_eq!(TensorDtype::F16.element_size(), 2);
        assert_eq!(TensorDtype::U8.element_size(), 1);
        assert_eq!(TensorDtype::I64.element_size(), 8);
    }

    #[test]
    fn test_device() {
        assert!(Device::cpu().is_cpu());
        assert!(!Device::cpu().is_cuda());
        assert!(Device::cuda(0).is_cuda());
        assert_eq!(Device::cuda(0).cuda_index(), Some(0));
        assert_eq!(Device::cuda(3).cuda_index(), Some(3));
        assert_eq!(Device::cpu().cuda_index(), None);
        // No longer limited to 4 GPUs
        assert_eq!(Device::cuda(15).cuda_index(), Some(15));
    }

    #[test]
    fn test_tensor_creation() {
        let tensor = HorusTensor::new(
            1,
            42,
            1,
            0,
            &[1080, 1920, 3],
            TensorDtype::U8,
            Device::cpu(),
        );

        assert_eq!(tensor.shape(), &[1080, 1920, 3]);
        assert_eq!(tensor.ndim, 3);
        assert_eq!(tensor.numel(), 1080 * 1920 * 3);
        assert_eq!(tensor.nbytes(), 1080 * 1920 * 3);
        assert!(tensor.is_contiguous());
    }

    #[test]
    fn test_tensor_is_pod() {
        let tensor = HorusTensor::default();
        let bytes: &[u8] = horus_core::bytemuck::bytes_of(&tensor);
        assert_eq!(bytes.len(), std::mem::size_of::<HorusTensor>());
    }

    #[test]
    fn test_tensor_size() {
        assert_eq!(std::mem::size_of::<HorusTensor>(), 232);
    }

    #[test]
    fn test_backward_compat_alias() {
        // TensorDevice is now a type alias for Device
        let dev: TensorDevice = Device::cpu();
        assert!(dev.is_cpu());
    }
}
