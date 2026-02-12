//! RAII handle for tensor memory management
//!
//! This module provides a safe wrapper around `HorusTensor` that automatically
//! manages reference counting through the tensor pool.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::memory::{TensorPool, TensorHandle, TensorPoolConfig};
//! use horus_core::memory::tensor_pool::{TensorDtype, TensorDevice};
//!
//! let pool = Arc::new(TensorPool::new(1, TensorPoolConfig::default())?);
//!
//! // Allocate a tensor wrapped in a handle
//! let handle = TensorHandle::alloc(pool.clone(), &[100, 100], TensorDtype::F32, TensorDevice::Cpu)?;
//!
//! // Access data
//! let data = handle.data_slice_mut();
//!
//! // Clone increases refcount
//! let handle2 = handle.clone();
//!
//! // Refcount automatically decremented when handles are dropped
//! drop(handle);
//! drop(handle2); // Tensor memory is freed when last handle is dropped
//! ```

use super::tensor_pool::{HorusTensor, TensorDevice, TensorDtype, TensorPool, MAX_TENSOR_DIMS};
use crate::error::HorusResult;
use std::sync::Arc;

/// RAII handle that automatically manages tensor reference counting
///
/// When a `TensorHandle` is cloned, the reference count is incremented.
/// When it is dropped, the reference count is decremented. When the count
/// reaches zero, the tensor's memory slot is returned to the pool.
pub struct TensorHandle {
    tensor: HorusTensor,
    pool: Arc<TensorPool>,
}

impl TensorHandle {
    /// Create a new handle from an existing tensor descriptor
    ///
    /// This increments the reference count. Use this when receiving a tensor
    /// from Topic or when wrapping a tensor you don't own yet.
    pub fn new(tensor: HorusTensor, pool: Arc<TensorPool>) -> Self {
        pool.retain(&tensor);
        Self { tensor, pool }
    }

    /// Create a handle without incrementing the reference count
    ///
    /// Use this when you've just allocated a tensor and already have a refcount of 1.
    /// This is an internal API - prefer `alloc()` for new tensors.
    pub(crate) fn from_owned(tensor: HorusTensor, pool: Arc<TensorPool>) -> Self {
        Self { tensor, pool }
    }

    /// Allocate a new tensor and return a handle
    ///
    /// This is the recommended way to create new tensors.
    pub fn alloc(
        pool: Arc<TensorPool>,
        shape: &[u64],
        dtype: TensorDtype,
        device: TensorDevice,
    ) -> HorusResult<Self> {
        let tensor = pool.alloc(shape, dtype, device)?;
        Ok(Self::from_owned(tensor, pool))
    }

    /// Get the underlying tensor descriptor
    ///
    /// The descriptor can be sent through Topic. The receiver should
    /// wrap it in a new `TensorHandle` to manage the reference count.
    #[inline]
    pub fn tensor(&self) -> &HorusTensor {
        &self.tensor
    }

    /// Get mutable reference to the underlying tensor descriptor
    ///
    /// Use with care - modifying the tensor descriptor can break invariants.
    /// Primarily used for setting CUDA IPC handles after GPU allocation.
    #[inline]
    pub fn tensor_mut(&mut self) -> &mut HorusTensor {
        &mut self.tensor
    }

    /// Get raw pointer to tensor data
    #[inline]
    pub fn data_ptr(&self) -> *mut u8 {
        self.pool.data_ptr(&self.tensor)
    }

    /// Get tensor data as a slice
    #[inline]
    pub fn data_slice(&self) -> &[u8] {
        self.pool.data_slice(&self.tensor)
    }

    /// Get tensor data as a mutable slice
    #[inline]
    pub fn data_slice_mut(&self) -> &mut [u8] {
        self.pool.data_slice_mut(&self.tensor)
    }

    /// Get tensor data as a typed slice
    ///
    /// # Safety
    /// Caller must ensure the dtype matches the requested type T.
    #[inline]
    pub unsafe fn data_as<T: Copy>(&self) -> &[T] {
        let bytes = self.data_slice();
        let ptr = bytes.as_ptr() as *const T;
        debug_assert!(
            ptr.is_aligned(),
            "tensor data pointer is not aligned for type {} (requires {} bytes, got address {:p})",
            std::any::type_name::<T>(),
            std::mem::align_of::<T>(),
            ptr
        );
        let len = bytes.len() / std::mem::size_of::<T>();
        // SAFETY: Caller guarantees T matches the tensor dtype. Pointer is valid for `len` elements.
        std::slice::from_raw_parts(ptr, len)
    }

    /// Get tensor data as a mutable typed slice
    ///
    /// # Safety
    /// Caller must ensure the dtype matches the requested type T.
    #[inline]
    #[allow(clippy::mut_from_ref)]
    pub unsafe fn data_as_mut<T: Copy>(&self) -> &mut [T] {
        let bytes = self.data_slice_mut();
        let ptr = bytes.as_mut_ptr() as *mut T;
        debug_assert!(
            ptr.is_aligned(),
            "tensor data pointer is not aligned for type {} (requires {} bytes, got address {:p})",
            std::any::type_name::<T>(),
            std::mem::align_of::<T>(),
            ptr
        );
        let len = bytes.len() / std::mem::size_of::<T>();
        // SAFETY: Caller guarantees T matches the tensor dtype. Pointer is valid and writable for `len` elements.
        std::slice::from_raw_parts_mut(ptr, len)
    }

    /// Get the current reference count
    pub fn refcount(&self) -> u32 {
        self.pool.refcount(&self.tensor)
    }

    /// Get the pool this tensor belongs to
    #[inline]
    pub fn pool(&self) -> &Arc<TensorPool> {
        &self.pool
    }

    /// Get tensor shape
    #[inline]
    pub fn shape(&self) -> &[u64] {
        let ndim = (self.tensor.ndim as usize).min(MAX_TENSOR_DIMS);
        &self.tensor.shape[..ndim]
    }

    /// Get tensor strides
    #[inline]
    pub fn strides(&self) -> &[u64] {
        let ndim = (self.tensor.ndim as usize).min(MAX_TENSOR_DIMS);
        &self.tensor.strides[..ndim]
    }

    /// Get tensor dtype
    #[inline]
    pub fn dtype(&self) -> TensorDtype {
        self.tensor.dtype
    }

    /// Get tensor device
    #[inline]
    pub fn device(&self) -> TensorDevice {
        self.tensor.device
    }

    /// Get total number of elements
    #[inline]
    pub fn numel(&self) -> u64 {
        self.shape().iter().product()
    }

    /// Get total size in bytes
    #[inline]
    pub fn nbytes(&self) -> u64 {
        self.tensor.size
    }

    /// Create a view (slice) of the first dimension
    ///
    /// This creates a new handle with adjusted shape that shares
    /// the same underlying memory (increments refcount).
    pub fn slice_first_dim(&self, start: u64, end: u64) -> Option<Self> {
        let sliced = self.tensor.slice_first_dim(start, end)?;
        Some(TensorHandle::new(sliced, Arc::clone(&self.pool)))
    }

    /// Create a reshaped view of the tensor
    ///
    /// This creates a new handle with a different shape that shares
    /// the same underlying memory (increments refcount).
    /// Only works on contiguous tensors.
    pub fn view(&self, new_shape: &[u64]) -> Option<Self> {
        let viewed = self.tensor.view(new_shape)?;
        Some(TensorHandle::new(viewed, Arc::clone(&self.pool)))
    }

    /// Check if tensor is contiguous
    pub fn is_contiguous(&self) -> bool {
        self.tensor.is_contiguous()
    }

    /// Check if tensor is on CPU
    #[inline]
    pub fn is_cpu(&self) -> bool {
        self.tensor.device == TensorDevice::Cpu
    }

    /// Check if tensor is on CUDA
    #[inline]
    pub fn is_cuda(&self) -> bool {
        self.tensor.device.is_cuda()
    }
}

impl Clone for TensorHandle {
    fn clone(&self) -> Self {
        self.pool.retain(&self.tensor);
        Self {
            tensor: self.tensor,
            pool: Arc::clone(&self.pool),
        }
    }
}

impl Drop for TensorHandle {
    fn drop(&mut self) {
        self.pool.release(&self.tensor);
    }
}

impl std::fmt::Debug for TensorHandle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TensorHandle")
            .field("shape", &self.shape())
            .field("dtype", &self.dtype())
            .field("device", &self.device())
            .field("refcount", &self.refcount())
            .field("pool_id", &self.tensor.pool_id)
            .finish()
    }
}

// Safety: TensorHandle can be sent between threads
// The underlying pool uses atomic operations for all shared state
unsafe impl Send for TensorHandle {}
unsafe impl Sync for TensorHandle {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::memory::TensorPoolConfig;
    use std::sync::atomic::{AtomicU32, Ordering};

    // Use atomic counter to generate unique pool IDs for each test
    static POOL_ID_COUNTER: AtomicU32 = AtomicU32::new(88000);

    fn create_test_pool() -> Arc<TensorPool> {
        let pool_id = POOL_ID_COUNTER.fetch_add(1, Ordering::Relaxed);
        let config = TensorPoolConfig {
            pool_size: 1024 * 1024,
            max_slots: 16,
            slot_alignment: 64,
        };
        Arc::new(TensorPool::new(pool_id, config).expect("Failed to create pool"))
    }

    #[test]
    fn test_handle_alloc() {
        let pool = create_test_pool();
        let handle =
            TensorHandle::alloc(pool.clone(), &[10, 20], TensorDtype::F32, TensorDevice::Cpu)
                .expect("Failed to allocate");

        assert_eq!(handle.shape(), &[10, 20]);
        assert_eq!(handle.dtype(), TensorDtype::F32);
        assert_eq!(handle.refcount(), 1);
        assert_eq!(handle.numel(), 200);
        assert_eq!(handle.nbytes(), 800); // 200 * 4 bytes

        std::fs::remove_file(pool.shm_path()).ok();
    }

    #[test]
    fn test_handle_clone_drop() {
        let pool = create_test_pool();
        let handle1 = TensorHandle::alloc(pool.clone(), &[10], TensorDtype::U8, TensorDevice::Cpu)
            .expect("Failed to allocate");

        assert_eq!(handle1.refcount(), 1);

        let handle2 = handle1.clone();
        assert_eq!(handle1.refcount(), 2);
        assert_eq!(handle2.refcount(), 2);

        drop(handle1);
        assert_eq!(handle2.refcount(), 1);

        drop(handle2);
        // Tensor should be freed now

        std::fs::remove_file(pool.shm_path()).ok();
    }

    #[test]
    fn test_data_access() {
        let pool = create_test_pool();
        let handle = TensorHandle::alloc(pool.clone(), &[4], TensorDtype::F32, TensorDevice::Cpu)
            .expect("Failed to allocate");

        // SAFETY: Handle was allocated with shape [4] and dtype F32, so data_as_mut::<f32>()
        // returns a valid &mut [f32; 4] slice backed by the tensor's shared memory region.
        let data = unsafe { handle.data_as_mut::<f32>() };
        data[0] = 1.0;
        data[1] = 2.0;
        data[2] = 3.0;
        data[3] = 4.0;

        // SAFETY: Same allocation as above; data_as::<f32>() reads the previously written values.
        let data = unsafe { handle.data_as::<f32>() };
        assert_eq!(data, &[1.0, 2.0, 3.0, 4.0]);

        std::fs::remove_file(pool.shm_path()).ok();
    }

    #[test]
    fn test_slice() {
        let pool = create_test_pool();
        let handle =
            TensorHandle::alloc(pool.clone(), &[10, 5], TensorDtype::F32, TensorDevice::Cpu)
                .expect("Failed to allocate");

        assert_eq!(handle.refcount(), 1);

        let slice = handle.slice_first_dim(2, 7).expect("Failed to slice");
        assert_eq!(slice.shape(), &[5, 5]);
        // Both handles share the same slot, so they see the same refcount
        assert_eq!(slice.refcount(), 2);
        assert_eq!(handle.refcount(), 2);

        // Drop slice, refcount decreases
        drop(slice);
        assert_eq!(handle.refcount(), 1);

        std::fs::remove_file(pool.shm_path()).ok();
    }
}
