//! Zero-copy tensor descriptor for shared memory communication
//!
//! [`HorusTensor`] is a lightweight 232-byte descriptor that flows through Topic
//! like any other message, while the actual tensor data lives in a shared memory pool.
//!
//! # Memory Layout (232 bytes, repr(C), Pod-safe)
//!
//! ```text
//! Pool identification:  pool_id(4) + slot_id(4) + generation(4) + _pad0(4) = 16 bytes
//! Data location:        offset(8) + size(8)                                = 16 bytes
//! Tensor metadata:      dtype(1) + ndim(1) + device_type(1) + _pad1(1) + device_id(4) = 8 bytes
//! Shape + strides:      shape(64) + strides(64)                            = 128 bytes
//! CUDA IPC:             cuda_ipc_handle(64)                                = 64 bytes
//! Total:                                                                   = 232 bytes
//! ```

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

use crate::device::{Device, DEVICE_TYPE_CPU, DEVICE_TYPE_CUDA};
use crate::dtype::TensorDtype;

/// Maximum number of dimensions supported by HorusTensor
pub const MAX_TENSOR_DIMS: usize = 8;

/// Size of CUDA IPC handle (cudaIpcMemHandle_t)
pub const CUDA_IPC_HANDLE_SIZE: usize = 64;

/// Zero-copy tensor descriptor for shared memory communication
///
/// This is a lightweight message type (232 bytes) that describes a tensor
/// stored in a shared memory pool. It flows through Topic like any other
/// message, but the actual tensor data lives in the pool.
///
/// # Reference Counting
///
/// Tensors use reference counting for memory management:
/// - `pool_id` + `slot_id` identify the memory slot
/// - `generation` prevents ABA problems when slots are reused
/// - The pool manages refcounts atomically
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct HorusTensor {
    // === Pool identification (16 bytes) ===
    /// ID of the pool that owns this tensor
    pub pool_id: u32,
    /// Slot index within the pool
    pub slot_id: u32,
    /// Generation counter for ABA prevention
    pub generation: u32,
    /// Reserved for future use
    pub _pad0: u32,

    // === Data location (16 bytes) ===
    /// Byte offset from pool base to tensor data
    pub offset: u64,
    /// Total size in bytes
    pub size: u64,

    // === Tensor metadata (8 bytes) ===
    /// Element data type
    pub dtype: TensorDtype,
    /// Number of dimensions (1-8)
    pub ndim: u8,
    /// Device type: 0=CPU, 1=CUDA (part of flattened Device)
    pub device_type: u8,
    /// Padding for alignment
    pub _pad1: u8,
    /// Device index (GPU number for CUDA, 0 for CPU)
    pub device_id: u32,

    // === Shape and strides (128 bytes) ===
    /// Dimensions of the tensor (up to 8)
    pub shape: [u64; MAX_TENSOR_DIMS],
    /// Byte strides for each dimension (enables views)
    pub strides: [u64; MAX_TENSOR_DIMS],

    // === CUDA IPC (64 bytes) ===
    /// CUDA IPC memory handle (only valid if device is CUDA)
    pub cuda_ipc_handle: [u8; CUDA_IPC_HANDLE_SIZE],
}

// Safety: HorusTensor is repr(C) with explicit padding, no implicit padding exists.
// All fields are Pod types (u8, u32, u64, [u8; N], [u64; N], TensorDtype which is repr(u8)).
unsafe impl Pod for HorusTensor {}
unsafe impl Zeroable for HorusTensor {}

impl Default for HorusTensor {
    fn default() -> Self {
        Self {
            pool_id: 0,
            slot_id: 0,
            generation: 0,
            _pad0: 0,
            offset: 0,
            size: 0,
            dtype: TensorDtype::F32,
            ndim: 0,
            device_type: DEVICE_TYPE_CPU,
            _pad1: 0,
            device_id: 0,
            shape: [0; MAX_TENSOR_DIMS],
            strides: [0; MAX_TENSOR_DIMS],
            cuda_ipc_handle: [0; CUDA_IPC_HANDLE_SIZE],
        }
    }
}

impl HorusTensor {
    /// Create a new tensor descriptor
    ///
    /// This is typically called by TensorPool, not directly.
    pub fn new(
        pool_id: u32,
        slot_id: u32,
        generation: u32,
        offset: u64,
        shape: &[u64],
        dtype: TensorDtype,
        device: Device,
    ) -> Self {
        let ndim = shape.len().min(MAX_TENSOR_DIMS) as u8;

        // Calculate size
        let num_elements: u64 = shape.iter().product();
        let size = num_elements * dtype.element_size() as u64;

        // Calculate row-major strides
        let mut strides = [0u64; MAX_TENSOR_DIMS];
        if ndim > 0 {
            strides[(ndim - 1) as usize] = dtype.element_size() as u64;
            for i in (0..(ndim - 1) as usize).rev() {
                strides[i] = strides[i + 1] * shape[i + 1];
            }
        }

        // Copy shape
        let mut shape_arr = [0u64; MAX_TENSOR_DIMS];
        for (i, &dim) in shape.iter().take(MAX_TENSOR_DIMS).enumerate() {
            shape_arr[i] = dim;
        }

        Self {
            pool_id,
            slot_id,
            generation,
            _pad0: 0,
            offset,
            size,
            dtype,
            ndim,
            device_type: device.device_type,
            _pad1: 0,
            device_id: device.device_id,
            shape: shape_arr,
            strides,
            cuda_ipc_handle: [0; CUDA_IPC_HANDLE_SIZE],
        }
    }

    /// Get the device this tensor resides on
    #[inline]
    pub const fn device(&self) -> Device {
        Device {
            device_type: self.device_type,
            _pad: [0; 3],
            device_id: self.device_id,
        }
    }

    /// Set the device for this tensor
    #[inline]
    pub fn set_device(&mut self, device: Device) {
        self.device_type = device.device_type;
        self.device_id = device.device_id;
    }

    /// Get the shape as a slice
    #[inline]
    pub fn shape(&self) -> &[u64] {
        &self.shape[..self.ndim as usize]
    }

    /// Get the strides as a slice
    #[inline]
    pub fn strides(&self) -> &[u64] {
        &self.strides[..self.ndim as usize]
    }

    /// Get total number of elements
    #[inline]
    pub fn numel(&self) -> u64 {
        self.shape().iter().product()
    }

    /// Check if tensor is contiguous (row-major)
    pub fn is_contiguous(&self) -> bool {
        if self.ndim == 0 {
            return true;
        }

        let mut expected_stride = self.dtype.element_size() as u64;
        for i in (0..self.ndim as usize).rev() {
            if self.strides[i] != expected_stride {
                return false;
            }
            expected_stride *= self.shape[i];
        }
        true
    }

    /// Check if this tensor is on CPU
    #[inline]
    pub const fn is_cpu(&self) -> bool {
        self.device_type == DEVICE_TYPE_CPU
    }

    /// Check if this tensor is on CUDA
    #[inline]
    pub const fn is_cuda(&self) -> bool {
        self.device_type == DEVICE_TYPE_CUDA
    }

    /// Get size in bytes
    #[inline]
    pub const fn nbytes(&self) -> u64 {
        self.size
    }

    /// Create a view of this tensor with different shape
    ///
    /// Returns None if the new shape is incompatible.
    pub fn view(&self, new_shape: &[u64]) -> Option<Self> {
        let old_numel: u64 = self.shape().iter().product();
        let new_numel: u64 = new_shape.iter().product();
        if old_numel != new_numel {
            return None;
        }

        if !self.is_contiguous() {
            return None;
        }

        Some(Self::new(
            self.pool_id,
            self.slot_id,
            self.generation,
            self.offset,
            new_shape,
            self.dtype,
            self.device(),
        ))
    }

    /// Create a slice/view of this tensor (first dimension only)
    pub fn slice_first_dim(&self, start: u64, end: u64) -> Option<Self> {
        if self.ndim == 0 || start >= end || end > self.shape[0] {
            return None;
        }

        let mut new_tensor = *self;
        new_tensor.shape[0] = end - start;
        new_tensor.offset += start * self.strides[0];
        let new_numel: u64 = new_tensor.shape[..new_tensor.ndim as usize]
            .iter()
            .product();
        new_tensor.size = new_numel * self.dtype.element_size() as u64;

        Some(new_tensor)
    }
}

// Custom Serialize/Deserialize to handle large arrays
impl Serialize for HorusTensor {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("HorusTensor", 11)?;
        state.serialize_field("pool_id", &self.pool_id)?;
        state.serialize_field("slot_id", &self.slot_id)?;
        state.serialize_field("generation", &self.generation)?;
        state.serialize_field("offset", &self.offset)?;
        state.serialize_field("size", &self.size)?;
        state.serialize_field("dtype", &self.dtype)?;
        state.serialize_field("ndim", &self.ndim)?;
        state.serialize_field("device", &self.device())?;
        state.serialize_field("shape", &self.shape[..])?;
        state.serialize_field("strides", &self.strides[..])?;
        state.serialize_field("cuda_ipc_handle", &self.cuda_ipc_handle[..])?;
        state.end()
    }
}

impl<'de> Deserialize<'de> for HorusTensor {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        use serde::de::{self, MapAccess, Visitor};

        struct HorusTensorVisitor;

        impl<'de> Visitor<'de> for HorusTensorVisitor {
            type Value = HorusTensor;

            fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                formatter.write_str("struct HorusTensor")
            }

            fn visit_map<V>(self, mut map: V) -> Result<HorusTensor, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut tensor = HorusTensor::default();

                while let Some(key) = map.next_key::<String>()? {
                    match key.as_str() {
                        "pool_id" => tensor.pool_id = map.next_value()?,
                        "slot_id" => tensor.slot_id = map.next_value()?,
                        "generation" => tensor.generation = map.next_value()?,
                        "offset" => tensor.offset = map.next_value()?,
                        "size" => tensor.size = map.next_value()?,
                        "dtype" => tensor.dtype = map.next_value()?,
                        "ndim" => tensor.ndim = map.next_value()?,
                        "device" => {
                            let device: Device = map.next_value()?;
                            tensor.device_type = device.device_type;
                            tensor.device_id = device.device_id;
                        }
                        "shape" => {
                            let v: Vec<u64> = map.next_value()?;
                            for (i, &val) in v.iter().take(MAX_TENSOR_DIMS).enumerate() {
                                tensor.shape[i] = val;
                            }
                        }
                        "strides" => {
                            let v: Vec<u64> = map.next_value()?;
                            for (i, &val) in v.iter().take(MAX_TENSOR_DIMS).enumerate() {
                                tensor.strides[i] = val;
                            }
                        }
                        "cuda_ipc_handle" => {
                            let v: Vec<u8> = map.next_value()?;
                            for (i, &val) in v.iter().take(CUDA_IPC_HANDLE_SIZE).enumerate() {
                                tensor.cuda_ipc_handle[i] = val;
                            }
                        }
                        _ => {
                            let _: de::IgnoredAny = map.next_value()?;
                        }
                    }
                }

                Ok(tensor)
            }
        }

        deserializer.deserialize_struct(
            "HorusTensor",
            &[
                "pool_id",
                "slot_id",
                "generation",
                "offset",
                "size",
                "dtype",
                "ndim",
                "device",
                "shape",
                "strides",
                "cuda_ipc_handle",
            ],
            HorusTensorVisitor,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tensor_size() {
        assert_eq!(
            std::mem::size_of::<HorusTensor>(),
            232,
            "HorusTensor must be exactly 232 bytes"
        );
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
        assert_eq!(tensor.nbytes(), 1080 * 1920 * 3); // U8 = 1 byte
        assert!(tensor.is_contiguous());
        assert!(tensor.is_cpu());
        assert!(!tensor.is_cuda());
        assert_eq!(tensor.device(), Device::cpu());
    }

    #[test]
    fn test_tensor_cuda() {
        let tensor = HorusTensor::new(1, 0, 0, 0, &[100, 100], TensorDtype::F32, Device::cuda(3));

        assert!(tensor.is_cuda());
        assert!(!tensor.is_cpu());
        assert_eq!(tensor.device(), Device::cuda(3));
        assert_eq!(tensor.device().cuda_index(), Some(3));
    }

    #[test]
    fn test_tensor_unlimited_gpu() {
        // Test GPU indices beyond old 4-GPU limit
        for gpu_id in [0, 1, 4, 7, 15, 100, 1000] {
            let tensor =
                HorusTensor::new(0, 0, 0, 0, &[10], TensorDtype::F32, Device::cuda(gpu_id));
            assert_eq!(tensor.device(), Device::cuda(gpu_id));
            assert_eq!(tensor.device().cuda_index(), Some(gpu_id));
        }
    }

    #[test]
    fn test_tensor_strides() {
        let tensor = HorusTensor::new(0, 0, 0, 0, &[2, 3, 4], TensorDtype::F32, Device::cpu());

        // Row-major strides for [2, 3, 4] with f32:
        // stride[2] = 4 (element size)
        // stride[1] = 4 * 4 = 16
        // stride[0] = 16 * 3 = 48
        assert_eq!(tensor.strides(), &[48, 16, 4]);
    }

    #[test]
    fn test_tensor_view() {
        let tensor = HorusTensor::new(0, 0, 0, 0, &[2, 3, 4], TensorDtype::F32, Device::cpu());

        let view = tensor.view(&[6, 4]).unwrap();
        assert_eq!(view.shape(), &[6, 4]);
        assert_eq!(view.numel(), tensor.numel());

        // Invalid reshape (wrong element count)
        assert!(tensor.view(&[5, 5]).is_none());
    }

    #[test]
    fn test_tensor_slice() {
        let tensor = HorusTensor::new(1, 2, 3, 0, &[10, 5], TensorDtype::F32, Device::cpu());

        let slice = tensor.slice_first_dim(2, 7).unwrap();
        assert_eq!(slice.shape(), &[5, 5]);
        assert_eq!(slice.offset, 2 * 5 * 4); // 2 rows * 5 cols * 4 bytes
    }

    #[test]
    fn test_tensor_pod_soundness() {
        let tensor = HorusTensor::default();
        let bytes: &[u8] = bytemuck::bytes_of(&tensor);
        assert_eq!(bytes.len(), 232);

        // Roundtrip through bytes
        let recovered: &HorusTensor = bytemuck::from_bytes(bytes);
        assert_eq!(recovered.dtype, TensorDtype::F32);
        assert_eq!(recovered.ndim, 0);
        assert!(recovered.is_cpu());
    }

    #[test]
    fn test_tensor_serde_roundtrip() {
        let tensor = HorusTensor::new(
            1,
            42,
            3,
            1024,
            &[480, 640, 3],
            TensorDtype::U8,
            Device::cuda(2),
        );

        let json = serde_json::to_string(&tensor).unwrap();
        let recovered: HorusTensor = serde_json::from_str(&json).unwrap();

        assert_eq!(recovered.pool_id, tensor.pool_id);
        assert_eq!(recovered.slot_id, tensor.slot_id);
        assert_eq!(recovered.generation, tensor.generation);
        assert_eq!(recovered.offset, tensor.offset);
        assert_eq!(recovered.size, tensor.size);
        assert_eq!(recovered.dtype, tensor.dtype);
        assert_eq!(recovered.ndim, tensor.ndim);
        assert_eq!(recovered.device(), tensor.device());
        assert_eq!(recovered.shape(), tensor.shape());
        assert_eq!(recovered.strides(), tensor.strides());
    }

    #[test]
    fn test_tensor_set_device() {
        let mut tensor = HorusTensor::new(0, 0, 0, 0, &[10], TensorDtype::F32, Device::cpu());
        assert!(tensor.is_cpu());

        tensor.set_device(Device::cuda(5));
        assert!(tensor.is_cuda());
        assert_eq!(tensor.device(), Device::cuda(5));

        tensor.set_device(Device::cpu());
        assert!(tensor.is_cpu());
    }

    #[test]
    fn test_tensor_default() {
        let tensor = HorusTensor::default();
        assert_eq!(tensor.pool_id, 0);
        assert_eq!(tensor.slot_id, 0);
        assert_eq!(tensor.generation, 0);
        assert_eq!(tensor.offset, 0);
        assert_eq!(tensor.size, 0);
        assert_eq!(tensor.dtype, TensorDtype::F32);
        assert_eq!(tensor.ndim, 0);
        assert!(tensor.is_cpu());
        assert_eq!(tensor.device(), Device::cpu());
    }
}
