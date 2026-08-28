#![allow(dead_code)]
//! Zero-copy tensor descriptor for shared memory communication
//!
//! [`Tensor`] is a lightweight 168-byte descriptor that flows through Topic
//! like any other message, while the actual tensor data lives in a shared memory pool.
//!
//! # Memory Layout (168 bytes, repr(C), Pod-safe)
//!
//! ```text
//! Pool identification:  pool_id(4) + slot_id(4) + generation(4) + generation_hi(4) = 16 bytes
//! Data location:        offset(8) + size(8)                                = 16 bytes
//! Tensor metadata:      dtype(1) + ndim(1) + device_type(1) + _pad1(1) + device_id(4) = 8 bytes
//! Shape + strides:      shape(64) + strides(64)                            = 128 bytes
//! Total:                                                                   = 168 bytes
//! ```

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

use super::device::{Device, DEVICE_TYPE_CPU, DEVICE_TYPE_CUDA};
use super::dtype::TensorDtype;

/// Maximum number of dimensions supported by Tensor
pub(crate) const MAX_TENSOR_DIMS: usize = 8;

/// Zero-copy tensor descriptor for shared memory communication
///
/// This is a lightweight message type (168 bytes) that describes a tensor
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
pub struct Tensor {
    // === Pool identification (16 bytes) ===
    /// ID of the pool that owns this tensor
    pub pool_id: u32,
    /// Slot index within the pool
    pub slot_id: u32,
    /// Generation counter for ABA prevention — low 32 bits of a 64-bit generation.
    ///
    /// The full 64-bit generation is `generation_full()`.  Always use
    /// `generation_full()` for pool comparisons; never compare `generation`
    /// alone, as that only catches 32-bit wraparound.
    pub generation: u32,
    /// High 32 bits of the 64-bit generation counter.
    ///
    /// Previously named `_pad0` (always 0).  Cross-process: old descriptors
    /// produced before this field was added will have `generation_hi = 0`,
    /// making their `generation_full()` equal to the raw 32-bit counter —
    /// which is backward-compatible for the first 2^32 cycles.
    pub generation_hi: u32,

    // === Data location (16 bytes) ===
    /// Byte offset from pool base to tensor data
    pub offset: u64,
    /// Total size in bytes
    pub size: u64,

    // === Tensor metadata (8 bytes) ===
    /// Element data type, held as the raw `repr(u8)` discriminant byte.
    ///
    /// This was `pub dtype: TensorDtype`, and it was unsound: the descriptor is
    /// read byte-for-byte out of peer-writable shared memory, so a byte outside
    /// `0..=TensorDtype::MAX_DISCRIMINANT` materialised an invalid enum
    /// discriminant — undefined behaviour committed *before* any sanitiser
    /// could run.  The byte is private now and every reader goes through
    /// [`Tensor::dtype`], which launders it via `TensorDtype::from_raw`.
    /// Same offset and size, so the 168-byte wire layout is unchanged.
    dtype_raw: u8,
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
}

// Safety: Tensor is repr(C) with explicit padding, no implicit padding exists.
// Every field is a plain integer or an array of them (u8, u32, u64, [u64; N]) —
// including the dtype discriminant, which is stored as a raw `u8` precisely so
// that every one of the 256 byte values really is valid, as `Pod` requires.
unsafe impl Pod for Tensor {}
unsafe impl Zeroable for Tensor {}

impl Default for Tensor {
    fn default() -> Self {
        Self {
            pool_id: 0,
            slot_id: 0,
            generation: 0,
            generation_hi: 0,
            offset: 0,
            size: 0,
            dtype_raw: TensorDtype::F32 as u8,
            ndim: 0,
            device_type: DEVICE_TYPE_CPU,
            _pad1: 0,
            device_id: 0,
            shape: [0; MAX_TENSOR_DIMS],
            strides: [0; MAX_TENSOR_DIMS],
        }
    }
}

impl Tensor {
    /// Element data type.
    ///
    /// Launders the stored discriminant byte through [`TensorDtype::from_raw`],
    /// so a corrupt or hostile wire descriptor yields `F32` rather than an
    /// invalid enum value.  This is the only way to read the dtype.
    #[inline]
    pub const fn dtype(&self) -> TensorDtype {
        TensorDtype::from_raw(self.dtype_raw)
    }

    /// Set the element data type.
    #[inline]
    pub fn set_dtype(&mut self, dtype: TensorDtype) {
        self.dtype_raw = dtype as u8;
    }

    /// Plant a raw discriminant byte, valid or not.
    ///
    /// **Test-only**: deliberately `#[doc(hidden)]` — the same pattern as
    /// `SharedLogBuffer::corrupt_slot_seqlock`.  Tests outside this crate use it
    /// to simulate the corrupt wire descriptor that the private `dtype_raw`
    /// field otherwise makes unreachable.  Production code must use
    /// [`set_dtype`](Self::set_dtype).
    #[doc(hidden)]
    pub fn set_dtype_raw_for_test(&mut self, raw: u8) {
        self.dtype_raw = raw;
    }

    /// Sanitize a Tensor read from untrusted bytes (SHM, network, file).
    ///
    /// Clamps `ndim` to the shape/strides array bound and normalises the raw
    /// dtype byte to a valid discriminant.  [`dtype`](Self::dtype) already
    /// launders the byte on every read — this only makes the *stored* byte
    /// agree, so a descriptor forwarded on the wire carries the clamped value.
    #[inline]
    pub fn sanitize_from_shm(&mut self) {
        self.ndim = self.ndim.min(MAX_TENSOR_DIMS as u8);
        self.dtype_raw = self.dtype() as u8;
    }

    /// Create a new tensor descriptor
    ///
    /// This is typically called by TensorPool, not directly.
    ///
    /// `generation_full` is the full 64-bit generation counter from the pool's
    /// `SlotHeader`.  It is split into low/high 32-bit halves stored in
    /// `generation` and `generation_hi` respectively.
    pub fn new(
        pool_id: u32,
        slot_id: u32,
        generation_full: u64,
        offset: u64,
        shape: &[u64],
        dtype: TensorDtype,
        device: Device,
    ) -> Self {
        let ndim = shape.len().min(MAX_TENSOR_DIMS) as u8;

        // Calculate size
        let num_elements: u64 = shape
            .iter()
            .copied()
            .try_fold(1u64, |acc, dim| acc.checked_mul(dim))
            .expect("Tensor::new: shape product overflows u64");
        let size = num_elements
            .checked_mul(dtype.element_size() as u64)
            .expect("Tensor::new: total size overflows u64");

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
            generation: generation_full as u32,
            generation_hi: (generation_full >> 32) as u32,
            offset,
            size,
            dtype_raw: dtype as u8,
            ndim,
            device_type: device.device_type,
            _pad1: 0,
            device_id: device.device_id,
            shape: shape_arr,
            strides,
        }
    }

    /// Full 64-bit generation counter for ABA prevention.
    ///
    /// Reconstructed from `generation` (low 32 bits) and `generation_hi`
    /// (high 32 bits).  Always use this for pool comparisons — never compare
    /// `generation` alone, as that only catches 32-bit wraparound.
    #[inline]
    pub fn generation_full(&self) -> u64 {
        (self.generation_hi as u64) << 32 | self.generation as u64
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

    /// Get the shape as a slice
    #[inline]
    pub fn shape(&self) -> &[u64] {
        &self.shape[..(self.ndim as usize).min(MAX_TENSOR_DIMS)]
    }

    /// Get the strides as a slice
    #[inline]
    pub fn strides(&self) -> &[u64] {
        &self.strides[..(self.ndim as usize).min(MAX_TENSOR_DIMS)]
    }

    /// Byte extent the shape/strides descriptor actually addresses.
    ///
    /// For a strided layout the last addressable byte is
    /// `sum_i (shape[i] - 1) * strides[i] + element_size`, which is what a
    /// consumer handing this descriptor to numpy or torch will read up to.
    /// Returns `None` on overflow.
    #[inline]
    pub fn addressed_extent_bytes(&self) -> Option<u64> {
        let elem = self.dtype().element_size() as u64;
        let mut extent: u64 = elem;
        for (&dim, &stride) in self.shape().iter().zip(self.strides().iter()) {
            if dim == 0 {
                return Some(0); // empty tensor addresses nothing
            }
            extent = extent.checked_add(dim.checked_sub(1)?.checked_mul(stride)?)?;
        }
        Some(extent)
    }

    /// Whether `count` elements of this tensor's dtype fit inside its allocation.
    ///
    /// For the image/pointcloud wrappers, the exported numpy shape is built from
    /// message fields (`width`, `height`, `channels`) rather than from
    /// `tensor.shape`, so `descriptor_is_within_allocation` does not cover them.
    /// Those fields arrive over the wire, and the export hands numpy a RAW
    /// POINTER — so an oversized width/height made every zero-copy consumer read
    /// past the end of the pool slot.
    #[inline]
    pub fn element_count_fits(&self, count: u64) -> bool {
        match count.checked_mul(self.dtype().element_size() as u64) {
            Some(bytes) => bytes <= self.size,
            None => false, // overflow — reject
        }
    }

    /// Whether this descriptor's shape/strides stay inside its own allocation.
    ///
    /// A `Tensor` descriptor travels over a topic, so `shape`, `strides`, `ndim`
    /// and `size` all arrive from another process. `shape()`/`strides()` clamp
    /// `ndim`, but nothing checked the described extent against `size` — so a
    /// descriptor claiming a large shape made every zero-copy consumer
    /// (numpy via `__array_interface__`, torch via `__cuda_array_interface__`)
    /// read past the end of the pool slot. Callers that expose the raw pointer
    /// MUST check this first.
    #[inline]
    pub fn descriptor_is_within_allocation(&self) -> bool {
        match self.addressed_extent_bytes() {
            Some(extent) => extent <= self.size,
            None => false, // overflow — reject
        }
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

        let mut expected_stride = self.dtype().element_size() as u64;
        for i in (0..(self.ndim as usize).min(MAX_TENSOR_DIMS)).rev() {
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
            self.generation_full(),
            self.offset,
            new_shape,
            self.dtype(),
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
        new_tensor.size = new_numel * self.dtype().element_size() as u64;

        Some(new_tensor)
    }
}

// Custom Serialize/Deserialize to handle large arrays
impl Serialize for Tensor {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("Tensor", 11)?;
        state.serialize_field("pool_id", &self.pool_id)?;
        state.serialize_field("slot_id", &self.slot_id)?;
        state.serialize_field("generation", &self.generation)?;
        state.serialize_field("generation_hi", &self.generation_hi)?;
        state.serialize_field("offset", &self.offset)?;
        state.serialize_field("size", &self.size)?;
        state.serialize_field("dtype", &self.dtype())?;
        state.serialize_field("ndim", &self.ndim)?;
        state.serialize_field("device", &self.device())?;
        state.serialize_field("shape", &self.shape[..])?;
        state.serialize_field("strides", &self.strides[..])?;
        state.end()
    }
}

impl<'de> Deserialize<'de> for Tensor {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        use serde::de::{self, MapAccess, Visitor};

        struct TensorVisitor;

        impl<'de> Visitor<'de> for TensorVisitor {
            type Value = Tensor;

            fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                formatter.write_str("struct Tensor")
            }

            fn visit_map<V>(self, mut map: V) -> Result<Tensor, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut tensor = Tensor::default();

                while let Some(key) = map.next_key::<String>()? {
                    match key.as_str() {
                        "pool_id" => tensor.pool_id = map.next_value()?,
                        "slot_id" => tensor.slot_id = map.next_value()?,
                        "generation" => tensor.generation = map.next_value()?,
                        "generation_hi" => tensor.generation_hi = map.next_value()?,
                        "offset" => tensor.offset = map.next_value()?,
                        "size" => tensor.size = map.next_value()?,
                        "dtype" => {
                            let dtype: TensorDtype = map.next_value()?;
                            tensor.dtype_raw = dtype as u8;
                        }
                        "ndim" => {
                            let raw: u8 = map.next_value()?;
                            tensor.ndim = raw.min(MAX_TENSOR_DIMS as u8);
                        }
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
                        _ => {
                            let _: de::IgnoredAny = map.next_value()?;
                        }
                    }
                }

                Ok(tensor)
            }
        }

        deserializer.deserialize_struct(
            "Tensor",
            &[
                "pool_id",
                "slot_id",
                "generation",
                "generation_hi",
                "offset",
                "size",
                "dtype",
                "ndim",
                "device",
                "shape",
                "strides",
            ],
            TensorVisitor,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tensor_size_and_alignment() {
        assert_eq!(
            std::mem::size_of::<Tensor>(),
            168,
            "Tensor must be exactly 168 bytes"
        );
        // Alignment is 8 (from u64 fields)
        assert_eq!(std::mem::align_of::<Tensor>(), 8);
        // Size must be a multiple of alignment (no trailing padding)
        assert_eq!(
            168 % 8,
            0,
            "Tensor size must be a multiple of its alignment"
        );
    }

    #[test]
    fn test_tensor_creation() {
        let tensor = Tensor::new(
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
        let tensor = Tensor::new(1, 0, 0, 0, &[100, 100], TensorDtype::F32, Device::cuda(3));

        assert!(tensor.is_cuda());
        assert!(!tensor.is_cpu());
        assert_eq!(tensor.device(), Device::cuda(3));
    }

    #[test]
    fn test_tensor_unlimited_gpu() {
        // Test GPU indices beyond old 4-GPU limit
        for gpu_id in [0, 1, 4, 7, 15, 100, 1000] {
            let tensor = Tensor::new(0, 0, 0, 0, &[10], TensorDtype::F32, Device::cuda(gpu_id));
            assert_eq!(tensor.device(), Device::cuda(gpu_id));
        }
    }

    #[test]
    fn test_tensor_strides() {
        let tensor = Tensor::new(0, 0, 0, 0, &[2, 3, 4], TensorDtype::F32, Device::cpu());

        // Row-major strides for [2, 3, 4] with f32:
        // stride[2] = 4 (element size)
        // stride[1] = 4 * 4 = 16
        // stride[0] = 16 * 3 = 48
        assert_eq!(tensor.strides(), &[48, 16, 4]);
    }

    #[test]
    fn test_tensor_view() {
        let tensor = Tensor::new(0, 0, 0, 0, &[2, 3, 4], TensorDtype::F32, Device::cpu());

        let view = tensor.view(&[6, 4]).unwrap();
        assert_eq!(view.shape(), &[6, 4]);
        assert_eq!(view.numel(), tensor.numel());

        // Invalid reshape (wrong element count)
        assert!(tensor.view(&[5, 5]).is_none());
    }

    #[test]
    fn test_tensor_slice() {
        let tensor = Tensor::new(1, 2, 3, 0, &[10, 5], TensorDtype::F32, Device::cpu());

        let slice = tensor.slice_first_dim(2, 7).unwrap();
        assert_eq!(slice.shape(), &[5, 5]);
        assert_eq!(slice.offset, 2 * 5 * 4); // 2 rows * 5 cols * 4 bytes
    }

    #[test]
    fn test_tensor_pod_soundness() {
        let tensor = Tensor::default();
        let bytes: &[u8] = bytemuck::bytes_of(&tensor);
        assert_eq!(bytes.len(), 168);

        // Roundtrip through bytes
        let recovered: &Tensor = bytemuck::from_bytes(bytes);
        assert_eq!(recovered.dtype(), TensorDtype::F32);
        assert_eq!(recovered.ndim, 0);
        assert!(recovered.is_cpu());
    }

    #[test]
    fn test_tensor_serde_roundtrip() {
        let tensor = Tensor::new(
            1,
            42,
            3,
            1024,
            &[480, 640, 3],
            TensorDtype::U8,
            Device::cuda(2),
        );

        let json = serde_json::to_string(&tensor).unwrap();
        let recovered: Tensor = serde_json::from_str(&json).unwrap();

        assert_eq!(recovered.pool_id, tensor.pool_id);
        assert_eq!(recovered.slot_id, tensor.slot_id);
        assert_eq!(recovered.generation, tensor.generation);
        assert_eq!(recovered.offset, tensor.offset);
        assert_eq!(recovered.size, tensor.size);
        assert_eq!(recovered.dtype(), tensor.dtype());
        assert_eq!(recovered.ndim, tensor.ndim);
        assert_eq!(recovered.device(), tensor.device());
        assert_eq!(recovered.shape(), tensor.shape());
        assert_eq!(recovered.strides(), tensor.strides());
    }

    #[test]
    fn test_tensor_default() {
        let tensor = Tensor::default();
        assert_eq!(tensor.pool_id, 0);
        assert_eq!(tensor.slot_id, 0);
        assert_eq!(tensor.generation, 0);
        assert_eq!(tensor.generation_hi, 0);
        assert_eq!(tensor.generation_full(), 0);
        assert_eq!(tensor.offset, 0);
        assert_eq!(tensor.size, 0);
        assert_eq!(tensor.dtype(), TensorDtype::F32);
        assert_eq!(tensor.ndim, 0);
        assert!(tensor.is_cpu());
        assert_eq!(tensor.device(), Device::cpu());
        assert_eq!(tensor.numel(), 1); // empty shape product = 1
        assert!(tensor.is_contiguous());

        // Default must differ from a configured tensor
        let configured = Tensor::new(1, 0, 0, 0, &[10], TensorDtype::U8, Device::cpu());
        assert_ne!(tensor.pool_id, configured.pool_id);
        assert_ne!(tensor.ndim, configured.ndim);
    }

    #[test]
    fn test_tensor_generation_full_split() {
        // Verify 64-bit generation is correctly split into lo/hi halves
        let gen: u64 = 0xDEAD_BEEF_CAFE_BABEu64;
        let tensor = Tensor::new(0, 0, gen, 0, &[1], TensorDtype::F32, Device::cpu());
        assert_eq!(tensor.generation, gen as u32);
        assert_eq!(tensor.generation_hi, (gen >> 32) as u32);
        assert_eq!(tensor.generation_full(), gen);

        // Edge case: generation that fits in 32 bits has hi == 0
        let small_gen = 42u64;
        let t2 = Tensor::new(0, 0, small_gen, 0, &[1], TensorDtype::F32, Device::cpu());
        assert_eq!(t2.generation, 42);
        assert_eq!(t2.generation_hi, 0);
        assert_eq!(t2.generation_full(), 42);
    }

    #[test]
    fn test_tensor_clone_independence() {
        let t = Tensor::new(1, 2, 3, 0, &[480, 640, 3], TensorDtype::U8, Device::cpu());
        let mut _copy = t;
        _copy.pool_id = 99;
        _copy.shape[0] = 1;
        assert_eq!(t.pool_id, 1, "original must be unaffected by copy mutation");
        assert_eq!(t.shape[0], 480);
    }

    /// `Tensor` used to carry `pub dtype: TensorDtype` under `unsafe impl Pod`,
    /// so a descriptor byte-copied out of peer-writable SHM could hold a
    /// discriminant no variant names — undefined behaviour the moment anything
    /// matched on it.  The byte is private and laundered now, so the same input
    /// is merely wrong, not unsound; and it must still sit at the same offset,
    /// because the wire layout is a cross-process contract.
    #[test]
    fn an_invalid_dtype_byte_is_laundered_and_stays_at_its_wire_offset() {
        const DTYPE_BYTE_OFFSET: usize = 32; // 16 (ids) + 16 (offset/size)

        let t = Tensor::new(1, 0, 0, 0, &[4, 4], TensorDtype::U8, Device::CPU);
        assert_eq!(
            bytemuck::bytes_of(&t)[DTYPE_BYTE_OFFSET],
            TensorDtype::U8 as u8,
            "the dtype discriminant must stay at wire offset 32"
        );

        let mut corrupt = t;
        corrupt.set_dtype_raw_for_test(200);
        assert_eq!(
            corrupt.dtype(),
            TensorDtype::F32,
            "a discriminant no variant names must read back as the F32 fallback"
        );
        // The reader is what makes it safe, but sanitize_from_shm must also
        // rewrite the stored byte so a forwarded descriptor carries the fallback.
        corrupt.sanitize_from_shm();
        assert_eq!(
            bytemuck::bytes_of(&corrupt)[DTYPE_BYTE_OFFSET],
            TensorDtype::F32 as u8
        );
    }

    #[test]
    fn descriptor_bounds_reject_an_oversized_shape() {
        // A Tensor descriptor arrives over a topic, so shape/strides/size all
        // come from another process. A shape that addresses more bytes than the
        // allocation must be refused before any consumer turns it into a
        // zero-copy numpy/torch view.
        let mut t = Tensor::new(1, 0, 0, 0, &[4, 4], TensorDtype::F32, Device::CPU);
        assert!(
            t.descriptor_is_within_allocation(),
            "honest descriptor is fine"
        );

        // Claim a much larger shape while leaving `size` alone.
        t.shape[0] = 4096;
        t.shape[1] = 4096;
        assert!(
            !t.descriptor_is_within_allocation(),
            "a shape larger than the allocation must be rejected"
        );
    }

    #[test]
    fn descriptor_bounds_reject_overflowing_strides() {
        let mut t = Tensor::new(1, 0, 0, 0, &[2, 2], TensorDtype::F32, Device::CPU);
        t.strides[0] = u64::MAX;
        assert!(
            !t.descriptor_is_within_allocation(),
            "stride arithmetic that overflows must reject, not wrap"
        );
    }

    #[test]
    fn descriptor_bounds_allow_an_empty_tensor() {
        let mut t = Tensor::new(1, 0, 0, 0, &[2, 2], TensorDtype::F32, Device::CPU);
        t.shape[0] = 0;
        assert!(
            t.descriptor_is_within_allocation(),
            "a zero-length dimension addresses nothing and is legal"
        );
    }
}

#[cfg(test)]
mod element_count_tests {
    use super::*;

    /// The image/pointcloud numpy exports build their shape from WIRE fields
    /// (width, height, channels, point_count) rather than from `tensor.shape`,
    /// so `descriptor_is_within_allocation` does not cover them — and those
    /// exports hand numpy a raw pointer into the pool slot.
    #[test]
    fn element_count_fits_rejects_an_oversized_shape() {
        let t = Tensor::new(1, 0, 0, 0, &[64, 64], TensorDtype::U8, Device::CPU);
        assert!(t.element_count_fits(64 * 64), "the honest size fits");
        assert!(
            !t.element_count_fits(100_000 * 100_000),
            "a wire-inflated width*height must be refused"
        );
    }

    #[test]
    fn element_count_fits_rejects_overflow() {
        let t = Tensor::new(1, 0, 0, 0, &[4, 4], TensorDtype::F32, Device::CPU);
        assert!(
            !t.element_count_fits(u64::MAX),
            "count * element_size must reject on overflow, not wrap"
        );
    }

    #[test]
    fn element_count_fits_accounts_for_dtype_width() {
        // 16 elements allocated as F32 = 64 bytes. As F32 that is exactly 16
        // elements; the same byte budget must not admit 64.
        let t = Tensor::new(1, 0, 0, 0, &[16], TensorDtype::F32, Device::CPU);
        assert!(t.element_count_fits(16));
        assert!(!t.element_count_fits(64));
    }
}
