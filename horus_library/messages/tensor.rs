//! Zero-copy tensor types for HORUS
//!
//! This module provides high-performance tensor types designed for zero-copy
//! shared memory communication between nodes, especially for ML/AI workloads.
//!
//! # Overview
//!
//! The tensor system consists of:
//! - [`HorusTensor`]: A lightweight descriptor that flows through Hub/Link
//! - [`TensorDtype`]: Data type enumeration (f32, f16, u8, etc.)
//! - [`TensorDevice`]: Device location (CPU or CUDA:N)
//!
//! # Zero-Copy Design
//!
//! Unlike traditional message types that copy data, `HorusTensor` is a descriptor
//! that points to data in a shared memory pool. This enables:
//! - Zero-copy tensor sharing between processes
//! - Direct numpy/torch interop via `__array_interface__`
//! - GPU tensor sharing via CUDA IPC
//!
//! # Example
//!
//! ```rust,ignore
//! use horus::prelude::*;
//! use horus_library::messages::tensor::{HorusTensor, TensorDtype, TensorDevice};
//!
//! // Tensor descriptors flow through Topic like any message
//! let topic = Topic::<HorusTensor>::new("camera/frames")?;
//!
//! // Receive tensor descriptor (actual data is in shared memory pool)
//! if let Some(tensor) = topic.recv(ctx) {
//!     println!("Received {}x{} tensor", tensor.shape[0], tensor.shape[1]);
//! }
//! ```

use horus_core::bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

/// Maximum number of dimensions supported by HorusTensor
pub const MAX_TENSOR_DIMS: usize = 8;

/// Size of CUDA IPC handle (cudaIpcMemHandle_t)
pub const CUDA_IPC_HANDLE_SIZE: usize = 64;

/// Data type for tensor elements
///
/// Matches common ML framework dtypes for seamless interop.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TensorDtype {
    /// 32-bit floating point (default for most ML)
    #[default]
    F32 = 0,
    /// 64-bit floating point
    F64 = 1,
    /// 16-bit floating point (half precision)
    F16 = 2,
    /// Brain floating point (bfloat16)
    BF16 = 3,
    /// Signed 8-bit integer
    I8 = 4,
    /// Signed 16-bit integer
    I16 = 5,
    /// Signed 32-bit integer
    I32 = 6,
    /// Signed 64-bit integer
    I64 = 7,
    /// Unsigned 8-bit integer (common for images)
    U8 = 8,
    /// Unsigned 16-bit integer
    U16 = 9,
    /// Unsigned 32-bit integer
    U32 = 10,
    /// Unsigned 64-bit integer
    U64 = 11,
    /// Boolean (stored as u8)
    Bool = 12,
}

impl TensorDtype {
    /// Get the size in bytes of a single element
    #[inline]
    pub const fn element_size(&self) -> usize {
        match self {
            TensorDtype::F32 => 4,
            TensorDtype::F64 => 8,
            TensorDtype::F16 => 2,
            TensorDtype::BF16 => 2,
            TensorDtype::I8 => 1,
            TensorDtype::I16 => 2,
            TensorDtype::I32 => 4,
            TensorDtype::I64 => 8,
            TensorDtype::U8 => 1,
            TensorDtype::U16 => 2,
            TensorDtype::U32 => 4,
            TensorDtype::U64 => 8,
            TensorDtype::Bool => 1,
        }
    }

    /// Get numpy dtype string (for __array_interface__)
    pub const fn numpy_typestr(&self) -> &'static str {
        match self {
            TensorDtype::F32 => "<f4",
            TensorDtype::F64 => "<f8",
            TensorDtype::F16 => "<f2",
            TensorDtype::BF16 => "<V2", // bfloat16 not directly supported
            TensorDtype::I8 => "|i1",
            TensorDtype::I16 => "<i2",
            TensorDtype::I32 => "<i4",
            TensorDtype::I64 => "<i8",
            TensorDtype::U8 => "|u1",
            TensorDtype::U16 => "<u2",
            TensorDtype::U32 => "<u4",
            TensorDtype::U64 => "<u8",
            TensorDtype::Bool => "|b1",
        }
    }
}

// Safety: TensorDtype is repr(u8) with valid values 0-12, all bit patterns in that range are valid
unsafe impl Pod for TensorDtype {}
unsafe impl Zeroable for TensorDtype {}

/// Device location for tensor data
///
/// Supports CPU and up to 4 CUDA GPUs.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TensorDevice {
    /// CPU memory (accessible via shared memory)
    #[default]
    Cpu = 0,
    /// CUDA GPU 0
    Cuda0 = 1,
    /// CUDA GPU 1
    Cuda1 = 2,
    /// CUDA GPU 2
    Cuda2 = 3,
    /// CUDA GPU 3
    Cuda3 = 4,
}

impl TensorDevice {
    /// Check if device is a CUDA GPU
    #[inline]
    pub const fn is_cuda(&self) -> bool {
        !matches!(self, TensorDevice::Cpu)
    }

    /// Get CUDA device index (0-3), or None for CPU
    #[inline]
    pub const fn cuda_device_id(&self) -> Option<u32> {
        match self {
            TensorDevice::Cpu => None,
            TensorDevice::Cuda0 => Some(0),
            TensorDevice::Cuda1 => Some(1),
            TensorDevice::Cuda2 => Some(2),
            TensorDevice::Cuda3 => Some(3),
        }
    }

    /// Create from CUDA device ID
    pub const fn from_cuda_id(id: u32) -> Option<Self> {
        match id {
            0 => Some(TensorDevice::Cuda0),
            1 => Some(TensorDevice::Cuda1),
            2 => Some(TensorDevice::Cuda2),
            3 => Some(TensorDevice::Cuda3),
            _ => None,
        }
    }
}

// Safety: TensorDevice is repr(u8) with valid values 0-4, all bit patterns in that range are valid
unsafe impl Pod for TensorDevice {}
unsafe impl Zeroable for TensorDevice {}

impl std::fmt::Display for TensorDevice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TensorDevice::Cpu => write!(f, "cpu"),
            TensorDevice::Cuda0 => write!(f, "cuda:0"),
            TensorDevice::Cuda1 => write!(f, "cuda:1"),
            TensorDevice::Cuda2 => write!(f, "cuda:2"),
            TensorDevice::Cuda3 => write!(f, "cuda:3"),
        }
    }
}

/// Zero-copy tensor descriptor for shared memory communication
///
/// This is a lightweight message type (~200 bytes) that describes a tensor
/// stored in a shared memory pool. It flows through Hub/Link like any other
/// message, but the actual tensor data lives in the pool.
///
/// # Memory Layout
///
/// The struct is `#[repr(C)]` with careful padding to ensure:
/// - 8-byte alignment for all u64 fields
/// - Cache-line friendly layout
/// - Compatible with C/Python bindings
///
/// # Reference Counting
///
/// Tensors use reference counting for memory management:
/// - `pool_id` + `slot_id` identify the memory slot
/// - `generation` prevents ABA problems when slots are reused
/// - The pool manages refcounts atomically
///
/// # Example
///
/// ```rust,ignore
/// // Create a tensor descriptor (typically done by TensorPool)
/// let tensor = HorusTensor {
///     pool_id: 1,
///     slot_id: 42,
///     generation: 1,
///     offset: 0,
///     size: 1920 * 1080 * 3,  // RGB image
///     dtype: TensorDtype::U8,
///     ndim: 3,
///     device: TensorDevice::Cpu,
///     shape: [1080, 1920, 3, 0, 0, 0, 0, 0],
///     strides: [1920 * 3, 3, 1, 0, 0, 0, 0, 0],
///     cuda_ipc_handle: [0; 64],
///     ..Default::default()
/// };
/// ```
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

    // === Tensor metadata (8 bytes, padded to 8-byte alignment) ===
    /// Element data type
    pub dtype: TensorDtype,
    /// Number of dimensions (1-8)
    pub ndim: u8,
    /// Device where data resides
    pub device: TensorDevice,
    /// Reserved padding to align to 8 bytes
    pub _pad1: [u8; 5],

    // === Shape and strides (128 bytes) ===
    /// Dimensions of the tensor (up to 8)
    pub shape: [u64; MAX_TENSOR_DIMS],
    /// Byte strides for each dimension (enables views)
    pub strides: [u64; MAX_TENSOR_DIMS],

    // === CUDA IPC (64 bytes) ===
    /// CUDA IPC memory handle (only valid if device is CUDA)
    pub cuda_ipc_handle: [u8; CUDA_IPC_HANDLE_SIZE],
}

// Safety: HorusTensor is repr(C) with explicit padding, no implicit padding exists
unsafe impl Pod for HorusTensor {}
unsafe impl Zeroable for HorusTensor {}

// Enable ultra-fast POD messaging (~50ns vs ~250ns with bincode)
unsafe impl horus_core::communication::PodMessage for HorusTensor {}

// Register for smart detection - Topic::new() will auto-select POD backend
horus_core::register_pod_type!(HorusTensor);

// Custom Serialize/Deserialize to handle large arrays
impl Serialize for HorusTensor {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("HorusTensor", 12)?;
        state.serialize_field("pool_id", &self.pool_id)?;
        state.serialize_field("slot_id", &self.slot_id)?;
        state.serialize_field("generation", &self.generation)?;
        state.serialize_field("offset", &self.offset)?;
        state.serialize_field("size", &self.size)?;
        state.serialize_field("dtype", &self.dtype)?;
        state.serialize_field("ndim", &self.ndim)?;
        state.serialize_field("device", &self.device)?;
        // Serialize arrays as slices
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
                        "device" => tensor.device = map.next_value()?,
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
            device: TensorDevice::Cpu,
            _pad1: [0; 5],
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
        device: TensorDevice,
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
            device,
            _pad1: [0; 5],
            shape: shape_arr,
            strides,
            cuda_ipc_handle: [0; CUDA_IPC_HANDLE_SIZE],
        }
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
    pub fn is_cpu(&self) -> bool {
        self.device == TensorDevice::Cpu
    }

    /// Check if this tensor is on CUDA
    #[inline]
    pub fn is_cuda(&self) -> bool {
        self.device.is_cuda()
    }

    /// Get size in bytes
    #[inline]
    pub fn nbytes(&self) -> u64 {
        self.size
    }

    /// Create a view of this tensor with different shape
    ///
    /// Returns None if the new shape is incompatible.
    pub fn view(&self, new_shape: &[u64]) -> Option<Self> {
        // Check element count matches
        let old_numel: u64 = self.shape().iter().product();
        let new_numel: u64 = new_shape.iter().product();
        if old_numel != new_numel {
            return None;
        }

        // Must be contiguous for reshape
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
            self.device,
        ))
    }

    /// Create a slice/view of this tensor
    ///
    /// Only supports slicing the first dimension for simplicity.
    pub fn slice_first_dim(&self, start: u64, end: u64) -> Option<Self> {
        if self.ndim == 0 || start >= end || end > self.shape[0] {
            return None;
        }

        let mut new_tensor = *self;
        new_tensor.shape[0] = end - start;
        new_tensor.offset += start * self.strides[0];
        new_tensor.size = new_tensor.numel() * self.dtype.element_size() as u64;

        Some(new_tensor)
    }
}

impl horus_core::core::LogSummary for HorusTensor {
    fn log_summary(&self) -> String {
        let shape_str: Vec<String> = self.shape().iter().map(|d| d.to_string()).collect();
        format!(
            "Tensor([{}], dtype={:?}, device={}, pool={}/slot={})",
            shape_str.join(", "),
            self.dtype,
            self.device,
            self.pool_id,
            self.slot_id
        )
    }
}

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
    fn test_tensor_device() {
        assert!(!TensorDevice::Cpu.is_cuda());
        assert!(TensorDevice::Cuda0.is_cuda());
        assert_eq!(TensorDevice::Cuda0.cuda_device_id(), Some(0));
        assert_eq!(TensorDevice::Cuda3.cuda_device_id(), Some(3));
        assert_eq!(TensorDevice::Cpu.cuda_device_id(), None);
    }

    #[test]
    fn test_tensor_creation() {
        let tensor = HorusTensor::new(
            1,                 // pool_id
            42,                // slot_id
            1,                 // generation
            0,                 // offset
            &[1080, 1920, 3],  // shape (H, W, C)
            TensorDtype::U8,   // dtype
            TensorDevice::Cpu, // device
        );

        assert_eq!(tensor.shape(), &[1080, 1920, 3]);
        assert_eq!(tensor.ndim, 3);
        assert_eq!(tensor.numel(), 1080 * 1920 * 3);
        assert_eq!(tensor.nbytes(), 1080 * 1920 * 3); // U8 = 1 byte
        assert!(tensor.is_contiguous());
    }

    #[test]
    fn test_tensor_strides() {
        let tensor = HorusTensor::new(0, 0, 0, 0, &[2, 3, 4], TensorDtype::F32, TensorDevice::Cpu);

        // Row-major strides for [2, 3, 4] with f32:
        // stride[2] = 4 (element size)
        // stride[1] = 4 * 4 = 16
        // stride[0] = 16 * 3 = 48
        assert_eq!(tensor.strides(), &[48, 16, 4]);
    }

    #[test]
    fn test_tensor_view() {
        let tensor = HorusTensor::new(0, 0, 0, 0, &[2, 3, 4], TensorDtype::F32, TensorDevice::Cpu);

        // Reshape to [6, 4]
        let view = tensor.view(&[6, 4]).unwrap();
        assert_eq!(view.shape(), &[6, 4]);
        assert_eq!(view.numel(), tensor.numel());

        // Invalid reshape (wrong element count)
        assert!(tensor.view(&[5, 5]).is_none());
    }

    #[test]
    fn test_tensor_slice() {
        let tensor = HorusTensor::new(1, 2, 3, 0, &[10, 5], TensorDtype::F32, TensorDevice::Cpu);

        let slice = tensor.slice_first_dim(2, 7).unwrap();
        assert_eq!(slice.shape(), &[5, 5]);
        assert_eq!(slice.offset, 2 * 5 * 4); // 2 rows * 5 cols * 4 bytes
    }

    #[test]
    fn test_tensor_is_pod() {
        // Verify HorusTensor can be safely cast to bytes
        let tensor = HorusTensor::default();
        let bytes: &[u8] = horus_core::bytemuck::bytes_of(&tensor);
        assert_eq!(bytes.len(), std::mem::size_of::<HorusTensor>());
    }

    #[test]
    fn test_tensor_size() {
        // Verify struct size is reasonable for IPC
        let size = std::mem::size_of::<HorusTensor>();
        println!("HorusTensor size: {} bytes", size);
        assert!(
            size < 256,
            "HorusTensor should be <256 bytes for efficient IPC"
        );
    }
}
